#!/usr/bin/env python3
"""Fit the velocity-dependent FRICTION RESIDUAL that the wheel-based linear plant misses.

DEVELOPMENT TOOL ONLY -- NOT A GATE. The on-road IMU settle metric (settle_peak_imu_jerk /
settle_peak_imu_decel) stays the promoter; this curve exists ONLY so tools/stopping/sim_replay.py
can iterate the anti-stiction pre-release offline against a plant that reproduces the terminal
brake-friction "disc-grab". It must never be wired into similarity_gate or any pass/fail gate.

WHY A RESIDUAL: the linear AR(1) plant (selfdrive/controls/lib/stopping_plant.PlantModel) is
identified on WHEEL aEgo, which quantizes/floors to ~0 below ~0.03 m/s and is therefore BLIND to
the felt grab. The device IMU longitudinal channel (a_long_imu = livePose.accelerationDevice.x,
gravity-removed/pitch-compensated, ~20 Hz held onto the 100 Hz sample clock) SEES it. The grab is
the residual:

    a_imu(v) ~= linear_plant_response + friction_residual(v_ego)
    friction_residual(v) = c0 + c1 * exp(-v / v0)        (Stribeck-like, 3 params)

We do NOT decompose linear_plant_response here. Instead we measure the residual directly as the
SETTLE-WINDOW gap (a_long_imu - a_wheel) -- that gap IS what the wheel plant is missing relative to
the IMU truth, frame by frame, and it is exactly what the FrictionPlant layer adds back. Fitting the
gap (rather than a_imu itself) keeps the linear plant's job and the friction layer's job cleanly
separated and avoids re-fitting the wheel dynamics on this thin IMU subset.

DATA (assess phase 2026-06-14, "coarse-provisional"):
  ~32 distinct IMU-bearing engaged stops across ~10 routes, mostly low-speed/driveway, 20 Hz.
  The 20 Hz JERK channel is ALIASED for the sub-100 ms grab peak (do NOT fit on jerk); the DECEL
  channel agrees with raw 100 Hz within ~13% and IS trustworthy. So:
    * the residual is sampled and fitted on the DECEL gap (a_imu - a_wheel),
    * the per-stop peak |a_imu| (settle_peak_imu_decel) is the holdout PREDICTION TARGET / ranking
      check (NOT jerk),
    * the data supports at most ~3 friction params; do not over-parameterize.

HOLD-OUT VALIDATION is mandatory and is leave-one-route-out across the routes: fit the curve on all
but one route, predict the held-out route's per-stop peak settle decel, and report in-sample vs
held-out error AND rank correlation (does it call a harsh driveway grab harsher than a gentle one).

Window/exclusions reuse tools/stopping/build_event_store.SETTLE_STANDSTILL_SPEED and the same
settle-window definition as settle_imu_jerk() (engaged + long-control-active, terminating at the
first v_ego <= 0.06 standstill, 0.6 s pre-settle lead), so the residual is measured over exactly the
window the on-road metric is read over.
"""

from __future__ import annotations

import argparse
import json
import sys
from dataclasses import dataclass, field
from datetime import UTC, datetime
from pathlib import Path
from typing import Any

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
  sys.path.insert(0, str(REPO_ROOT))

from openpilot.selfdrive.controls.lib.stopping_plant import (
  FRICTION_COEF_KEYS,
  FrictionResidual,
  FrictionResidualParams,
)

DEFAULT_EVENT_STORE = Path.home() / ".comma" / "stopping_behavior" / "event_store"
DEFAULT_OUTPUT = REPO_ROOT / "docs" / "stopping" / "archive" / f"friction_residual_{datetime.now(UTC):%Y%m%d}.json"

# Settle window definition -- kept in lockstep with build_event_store.settle_imu_jerk().
SETTLE_STANDSTILL_SPEED = 0.06   # genuine-first-standstill band (felt-grab speed, above the 0.04 StopReq-A gate)
PRE_SETTLE_S = 0.6               # window lead before first standstill
SCAN_TAIL_FRAMES = 30            # scan a small tail past hold for the genuine ~0.05 standstill

# Fit bounds (physical priors; 3-param Stribeck curve, dev tool only). The assess phase pinned the
# above-grab offset at the calibration level (~ -0.02 m/s^2) and per-stop grab amplitudes <= ~0.69
# m/s^2, so c0 is held to a tight calibration band and c1 to a sane grab amplitude rather than letting
# either chase the (real but transient) mid-band wheel-overshoot undershoot. With this data shape the
# 3-param exp sits near these rails -- reported as an uncertainty, NOT papered over.
C0_BOUNDS = (-0.10, 0.10)        # calibration-level offset
C1_BOUNDS = (0.0, 1.20)          # grab amplitude (positive net decel as v -> 0)
V0_BOUNDS = (0.02, 0.30)         # onset velocity scale (m/s)


@dataclass
class SettleTrace:
  """One physical stop's IMU-bearing settle window."""
  route: str
  key: dict[str, Any]
  v: np.ndarray              # v_ego over the settle window (m/s)
  a_wheel: np.ndarray        # a_ego (wheel) over the window (m/s^2) -- wheel-blind near standstill
  resid: np.ndarray          # a_long_imu - a_wheel over the window (m/s^2): the friction-fit target
  peak_imu_decel: float      # stored settle_peak_imu_decel (peak |a_long_imu|) -- holdout target
  peak_imu_jerk: float       # stored settle_peak_imu_jerk -- recorded ONLY, NOT a fit/holdout target (aliased)
  touched_by_disengage_or_brake: bool

  def grab_signature(self) -> tuple[float, float]:
    return (round(float(self.peak_imu_decel), 4), round(float(self.peak_imu_jerk), 4))


def _residual_curve(v: np.ndarray, c0: float, c1: float, v0: float) -> np.ndarray:
  return c0 + c1 * np.exp(-np.maximum(0.0, v) / max(v0, 1e-6))


def extract_settle_window(npz: dict[str, np.ndarray]) -> tuple[np.ndarray, np.ndarray, np.ndarray, bool] | None:
  """Reproduce build_event_store.settle_imu_jerk()'s window selection on a stored npz trace.

  Returns (v_window, a_wheel_window, resid_window, touched) where resid = a_long_imu - a_ego over the
  masked, IMU-present settle window, or None when there is no engaged IMU settle. `touched` flags a
  window that contains a disengaged or brake-pressed frame (the assess phase counted ~4/37 such
  windows; they are kept but flagged so the report can note them)."""
  if "a_long_imu" not in npz:
    return None
  t = np.asarray(npz["t"], dtype=float)
  v = np.asarray(npz["v_ego"], dtype=float)
  ae = np.asarray(npz["a_ego"], dtype=float)
  imu = np.asarray(npz["a_long_imu"], dtype=float)
  en = np.asarray(npz["enabled"]).astype(bool)
  bp = np.asarray(npz["brake_pressed"]).astype(bool)
  n = len(t)
  finite_t = np.isfinite(t)
  # first sample reaching the genuine standstill band
  standstill = np.where(finite_t & (v <= SETTLE_STANDSTILL_SPEED))[0]
  if len(standstill) == 0:
    return None
  fs = int(standstill[0])
  t0 = t[fs] - PRE_SETTLE_S
  # masked, IMU-present settle window: engaged, finite, IMU-present, t in [t0, t_fs]
  idx = np.arange(n)
  in_window = finite_t & (t >= t0) & (idx <= fs)
  engaged = in_window & en & np.isfinite(imu) & np.isfinite(ae) & np.isfinite(v)
  sel = np.where(engaged & ~bp)[0]
  if len(sel) < 2:
    return None
  v_w = v[sel]
  a_wheel_w = ae[sel]
  resid_w = imu[sel] - ae[sel]
  # was the *window region* touched by a disengaged or brake-pressed frame?
  region = np.where(in_window)[0]
  touched = bool((~en[region]).any() or bp[region].any())
  return v_w, a_wheel_w, resid_w, touched


def load_settle_traces(store_dir: Path, dedup: bool = True) -> list[SettleTrace]:
  """Load IMU-bearing engaged settles from the event store, collapsing same-route/same-grab dupes
  (a single physical stop is seen by the speed/signal/hybrid detectors and would otherwise inflate
  n). Dedup key = (route, rounded peak_imu_decel, rounded peak_imu_jerk)."""
  events_path = store_dir / "events.jsonl"
  if not events_path.is_file():
    raise FileNotFoundError(f"event store index not found: {events_path}")
  raw: list[SettleTrace] = []
  with open(events_path) as f:
    for line in f:
      line = line.strip()
      if not line:
        continue
      record = json.loads(line)
      m = record.get("metrics_100hz", {})
      pk_decel = m.get("settle_peak_imu_decel")
      pk_jerk = m.get("settle_peak_imu_jerk")
      if pk_decel is None or pk_jerk is None:
        continue
      trace_path = store_dir / record["trace_ref"]
      if not trace_path.is_file():
        continue
      with np.load(trace_path) as z:
        npz = {k: z[k] for k in z.keys()}
      window = extract_settle_window(npz)
      if window is None:
        continue
      v_w, a_wheel_w, resid_w, touched = window
      raw.append(SettleTrace(
        route=str(record["key"]["route"]),
        key=dict(record["key"]),
        v=v_w, a_wheel=a_wheel_w, resid=resid_w,
        peak_imu_decel=float(pk_decel), peak_imu_jerk=float(pk_jerk),
        touched_by_disengage_or_brake=touched,
      ))
  if not dedup:
    return raw
  seen: dict[tuple[str, tuple[float, float]], SettleTrace] = {}
  for st in raw:
    seen.setdefault((st.route, st.grab_signature()), st)
  return list(seen.values())


def gather_residual_samples(traces: list[SettleTrace]) -> tuple[np.ndarray, np.ndarray]:
  if not traces:
    return np.zeros(0), np.zeros(0)
  v = np.concatenate([t.v for t in traces])
  r = np.concatenate([t.resid for t in traces])
  return v, r


def fit_curve(v: np.ndarray, resid: np.ndarray) -> FrictionResidualParams:
  """Bounded nonlinear least squares for resid(v) = c0 + c1*exp(-v/v0).

  scipy is not a hard dependency of this tool path, so we run a small bounded grid over the (very
  nonlinear) onset scale v0 and solve the (linear-in-c0,c1) least squares at each v0, taking the
  best SSE. This is robust, deterministic, and adequate for a 3-param dev-tool fit."""
  if len(v) < 4:
    raise RuntimeError(f"need >=4 residual samples to fit, got {len(v)}")
  v = np.maximum(0.0, np.asarray(v, dtype=float))
  resid = np.asarray(resid, dtype=float)
  best: tuple[float, float, float, float] | None = None  # (sse, c0, c1, v0)
  for v0 in np.linspace(V0_BOUNDS[0], V0_BOUNDS[1], 200):
    basis = np.exp(-v / v0)
    a = np.column_stack([np.ones_like(v), basis])  # [c0, c1]
    coef, *_ = np.linalg.lstsq(a, resid, rcond=None)
    c0, c1 = float(coef[0]), float(coef[1])
    c0 = min(max(c0, C0_BOUNDS[0]), C0_BOUNDS[1])
    c1 = min(max(c1, C1_BOUNDS[0]), C1_BOUNDS[1])
    sse = float(np.sum((a @ np.array([c0, c1]) - resid) ** 2))
    if best is None or sse < best[0]:
      best = (sse, c0, c1, float(v0))
  assert best is not None
  _, c0, c1, v0 = best
  return FrictionResidualParams(c0=c0, c1=c1, v0=v0)


def predict_peak_decel(friction: FrictionResidual, trace: SettleTrace) -> float:
  """Predicted per-stop peak |a_imu| over the settle window, EXACTLY as FrictionPlant produces it:

      a_imu_pred(v) = a_wheel(v) + friction_residual(v)

  This is the prediction the sim makes -- the friction residual is ADDED to the (per-stop) wheel /
  linear-plant trajectory, never evaluated alone. The wheel channel supplies the stop-to-stop shape
  (how the wheel a_ego decays into the window) and the friction curve supplies the terminal grab the
  wheel is blind to. The predicted peak |a_imu| is comparable to the stored settle_peak_imu_decel.

  NOTE (honest, load-bearing): a velocity-ONLY curve max|friction_residual(v)| is near-CONSTANT
  across stops (all settles sweep the same ~0.06->0.3 m/s band), so it cannot rank harsh-vs-smooth on
  its own. The discrimination comes from adding the per-stop wheel trajectory back in. Here we use the
  OBSERVED wheel trace as a stand-in for the linear-plant trajectory the sim would roll; in
  sim_replay the same `a_wheel + friction_residual(v)` is produced by FrictionPlant.predict_next_imu."""
  a_imu_pred = trace.a_wheel + _residual_curve(trace.v, friction.params.c0, friction.params.c1, friction.params.v0)
  return float(np.max(np.abs(a_imu_pred)))


def _spearman(a: np.ndarray, b: np.ndarray) -> float:
  if len(a) < 2:
    return float("nan")
  ra = np.argsort(np.argsort(a)).astype(float)
  rb = np.argsort(np.argsort(b)).astype(float)
  ra -= ra.mean()
  rb -= rb.mean()
  denom = np.sqrt(np.sum(ra ** 2) * np.sum(rb ** 2))
  if denom <= 1e-12:
    return float("nan")
  return float(np.sum(ra * rb) / denom)


@dataclass
class HoldoutResult:
  n_routes: int
  n_stops: int
  per_stop_abs_err: list[float] = field(default_factory=list)
  predicted: list[float] = field(default_factory=list)
  observed: list[float] = field(default_factory=list)
  fold_params: list[dict[str, float]] = field(default_factory=list)   # per-fold fitted coef (uncertainty)

  @property
  def mae(self) -> float:
    return float(np.mean(self.per_stop_abs_err)) if self.per_stop_abs_err else float("nan")

  @property
  def rmse(self) -> float:
    return float(np.sqrt(np.mean(np.square(self.per_stop_abs_err)))) if self.per_stop_abs_err else float("nan")

  @property
  def spearman(self) -> float:
    return _spearman(np.array(self.predicted), np.array(self.observed))

  def param_spread(self) -> dict[str, dict[str, float]]:
    """Min/median/max of each coefficient across the leave-one-route-out folds -- the honest
    parameter-uncertainty proxy at this n (a coefficient that swings wildly fold-to-fold is poorly
    constrained by the data)."""
    out: dict[str, dict[str, float]] = {}
    for key in FRICTION_COEF_KEYS:
      vals = [fp[key] for fp in self.fold_params]
      out[key] = {"min": float(min(vals)), "median": float(np.median(vals)), "max": float(max(vals))} if vals else {}
    return out


def leave_one_route_out(traces: list[SettleTrace]) -> HoldoutResult:
  """Fit on all-but-one route, predict the held-out route's per-stop peak settle decel."""
  routes = sorted({t.route for t in traces})
  res = HoldoutResult(n_routes=len(routes), n_stops=len(traces))
  for held in routes:
    train = [t for t in traces if t.route != held]
    test = [t for t in traces if t.route == held]
    v_tr, r_tr = gather_residual_samples(train)
    if len(v_tr) < 4 or not test:
      continue
    friction = FrictionResidual(fit_curve(v_tr, r_tr))
    res.fold_params.append(friction.as_dict())
    for st in test:
      pred = predict_peak_decel(friction, st)
      res.predicted.append(pred)
      res.observed.append(st.peak_imu_decel)
      res.per_stop_abs_err.append(abs(pred - st.peak_imu_decel))
  return res


def insample_eval(traces: list[SettleTrace], friction: FrictionResidual) -> dict[str, float]:
  preds = np.array([predict_peak_decel(friction, t) for t in traces])
  obs = np.array([t.peak_imu_decel for t in traces])
  err = np.abs(preds - obs)
  # frame-level residual fit quality on the DECEL gap
  v, r = gather_residual_samples(traces)
  fit_curve_vals = _residual_curve(v, friction.params.c0, friction.params.c1, friction.params.v0)
  frame_rmse = float(np.sqrt(np.mean((fit_curve_vals - r) ** 2)))
  return {
    "peak_decel_mae": float(np.mean(err)),
    "peak_decel_rmse": float(np.sqrt(np.mean(err ** 2))),
    "peak_decel_spearman": _spearman(preds, obs),
    "frame_residual_rmse": frame_rmse,
    "n_residual_samples": int(len(v)),
  }


def _harsh_vs_smooth(holdout: HoldoutResult) -> dict[str, Any]:
  """Concrete ranking check on the held-out predictions: split the observed peak decel into the
  harshest and smoothest terciles and report whether the model's mean prediction separates them in
  the right order (does it call the hard driveway grab harsher than the gentle one)."""
  pred = np.array(holdout.predicted)
  obs = np.array(holdout.observed)
  if len(obs) < 6:
    return {"n": int(len(obs)), "verdict": "too few held-out stops for a tercile check"}
  order = np.argsort(obs)
  k = max(1, len(obs) // 3)
  smooth_idx = order[:k]
  harsh_idx = order[-k:]
  pred_smooth = float(np.mean(pred[smooth_idx]))
  pred_harsh = float(np.mean(pred[harsh_idx]))
  return {
    "n": int(len(obs)),
    "tercile_size": int(k),
    "observed_smooth_mean_decel": float(np.mean(obs[smooth_idx])),
    "observed_harsh_mean_decel": float(np.mean(obs[harsh_idx])),
    "predicted_smooth_mean_decel": pred_smooth,
    "predicted_harsh_mean_decel": pred_harsh,
    "ranks_harsh_above_smooth": bool(pred_harsh > pred_smooth),
  }


def build_archive(traces: list[SettleTrace], friction: FrictionResidual, insample: dict[str, float],
                  holdout: HoldoutResult, event_store: str) -> dict[str, Any]:
  jerks = sorted(t.peak_imu_jerk for t in traces)
  decels = sorted(t.peak_imu_decel for t in traces)
  per_route = {r: sum(1 for t in traces if t.route == r) for r in sorted({t.route for t in traces})}
  return {
    "_banner": "DEVELOPMENT TOOL -- NOT A GATE. On-road IMU settle metric (settle_peak_imu_jerk / " +
               "settle_peak_imu_decel) remains the promoter. Do NOT wire this into similarity_gate " +
               "or any pass/fail gate. This curve exists only for offline sim iteration of the " +
               "anti-stiction pre-release.",
    "generated_utc": datetime.now(UTC).isoformat(),
    "event_store": event_store,
    "model_form": "friction_residual(v) = c0 + c1 * exp(-v / v0); a_imu_pred = linear_plant + friction_residual",
    "signal": "DECEL channel: residual = a_long_imu(livePose.accelerationDevice.x) - a_ego(wheel), " +
              "20 Hz held; jerk channel deliberately NOT used (aliased per assess phase).",
    "provenance": {
      "n_distinct_stops": len(traces),
      "n_routes": len(per_route),
      "per_route_stop_count": per_route,
      "n_windows_touched_by_disengage_or_brake": sum(1 for t in traces if t.touched_by_disengage_or_brake),
      "settle_window": {
        "standstill_speed_mps": SETTLE_STANDSTILL_SPEED,
        "pre_settle_s": PRE_SETTLE_S,
        "definition": "matches build_event_store.settle_imu_jerk() window (engaged + long-control-active, " +
                      "terminates at first v_ego <= standstill_speed)",
      },
      "stored_peak_imu_jerk_distribution": {
        "median": float(np.median(jerks)), "max": float(max(jerks)),
        "note": "recorded for reference only; the 20 Hz held jerk is aliased/artifact-prone and is " +
                "NOT a fit or holdout target -- fit/ranking are on the decel channel.",
      },
      "peak_imu_decel_distribution": {
        "min": float(min(decels)), "median": float(np.median(decels)), "max": float(max(decels)),
      },
    },
    "friction_residual": {
      "coefficients": friction.as_dict(),
      "coef_keys": list(FRICTION_COEF_KEYS),
      "bounds": {"c0": list(C0_BOUNDS), "c1": list(C1_BOUNDS), "v0": list(V0_BOUNDS)},
    },
    "insample": insample,
    "holdout_leave_one_route_out": {
      "n_routes": holdout.n_routes,
      "n_stops_predicted": len(holdout.predicted),
      "peak_decel_mae": holdout.mae,
      "peak_decel_rmse": holdout.rmse,
      "peak_decel_spearman": holdout.spearman,
      "harsh_vs_smooth_ranking": _harsh_vs_smooth(holdout),
      "fold_param_spread": holdout.param_spread(),
      "fold_param_spread_note": "min/median/max of each coefficient across leave-one-route-out folds; " +
                                "wide swings indicate the coefficient is poorly constrained at this n.",
    },
    "confounds": "HEV: regen vs friction not separable (regenBraking always False, brake always 0); " +
                 "residual is a LUMPED net-decel-vs-v curve, not physical friction. Brake temperature " +
                 "unobservable. ~32 route-correlated stops, low-speed/driveway-skewed.",
    "verdict": "coarse-provisional (assess 2026-06-14). Trust for: relative harsh-vs-smooth ranking of " +
               "terminal settles in offline sim, and reproducing the wheel-blind grab DECEL the linear " +
               "plant misses. Do NOT trust for: absolute physical friction values, jerk-peak magnitude, " +
               "rolling-traffic stops (data is low-speed-skewed), or any gating decision.",
  }


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
  parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  parser.add_argument("--event-store", default=str(DEFAULT_EVENT_STORE))
  parser.add_argument("--output", default=str(DEFAULT_OUTPUT), help="Archive JSON path")
  parser.add_argument("--no-dedup", action="store_true", help="Do NOT collapse same-route/same-grab detector dupes (debug)")
  return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
  args = parse_args(argv)
  traces = load_settle_traces(Path(args.event_store), dedup=not args.no_dedup)
  if len(traces) < 4:
    print(f"error: only {len(traces)} distinct IMU-bearing settles -- too few to fit (need >=4). " +
          "Collect more engaged stops with a_long_imu in the event store.", file=sys.stderr)
    return 1
  v, r = gather_residual_samples(traces)
  friction = FrictionResidual(fit_curve(v, r))
  insample = insample_eval(traces, friction)
  holdout = leave_one_route_out(traces)
  archive = build_archive(traces, friction, insample, holdout, str(args.event_store))

  out_path = Path(args.output)
  out_path.parent.mkdir(parents=True, exist_ok=True)
  with open(out_path, "w") as f:
    json.dump(archive, f, indent=2, sort_keys=True)

  c = friction.params
  print("DEVELOPMENT TOOL -- NOT A GATE (on-road IMU settle metric promotes).")
  print(f"fit: resid(v) = {c.c0:+.4f} + {c.c1:+.4f}*exp(-v/{c.v0:.4f})  " +
        f"[n={len(traces)} stops, {holdout.n_routes} routes, {insample['n_residual_samples']} frames]")
  print(f"in-sample: frame-resid RMSE={insample['frame_residual_rmse']:.4f} m/s^2, " +
        f"peak-decel MAE={insample['peak_decel_mae']:.4f}, spearman={insample['peak_decel_spearman']:.3f}")
  print(f"holdout (leave-one-route-out, n={len(holdout.predicted)} stops): " +
        f"peak-decel MAE={holdout.mae:.4f} RMSE={holdout.rmse:.4f}, spearman={holdout.spearman:.3f}")
  print(f"wrote {out_path}")
  return 0


if __name__ == "__main__":
  sys.exit(main())
