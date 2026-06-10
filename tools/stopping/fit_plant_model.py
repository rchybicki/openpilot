#!/usr/bin/env python3
"""System-ID refit of the 7-feature stopping plant on event-store traces (spec section 7.5 / WP3).

Refits the AR(1)-with-dead-time plant (schema preserved verbatim from stopping_model.FEATURE_NAMES)
on rlog-derived event-store stop events, resampled to 10 Hz (plus an optional 20 Hz sensitivity
variant), with the spec exclusions:

  (i)  the first 4 s after every engagement for all telemetry_version == 1 data; post-engagement
       windows from telemetry_version >= 2 routes are admissible ONLY via the spec-4.3 source
       switch (the event record must declare accel_cmd_source == 'carOutput' -- the sent value;
       a version flag alone, with pre-cap carControl accel in the trace, would re-create the
       post-engagement blind spot, now trusted),
  (ii) driver-brake frames,
  (iii) the plant has no lead-gap feature (asserted -- pre-dRel-flip routes need no exclusion).

Holdout-route validation RMSE is always reported (never in-sample only). Acceptance to replace
stopping_params.PLANT_MODEL_REF: holdout RMSE <= 1.1x the archived 20260531 fit equivalent
(evaluated on the SAME holdout rows) AND check_leapfrog_alignment recall >= current (run
separately; this tool records the criterion as pending).

Event-store input contract (built by tools/stopping/build_event_store.py, spec 7.1):
  <store>/events.jsonl -- one JSON record per stop event with at least:
    key: {route, seg, hold_mono_ns}     stable event key
    telemetry_version: 1|2              spec 4.3
    signals_version: 1|2                spec 4.2 (recorded; the plant uses no lead features)
    trace_ref: "events/<key>.npz"       relative to the store dir
    accel_cmd_source: "carControl"|"carOutput"   (optional; required to trust v2 engagement windows)
  trace npz arrays (aligned, one value per logged frame):
    t (s, monotonic), v_ego (m/s), a_ego (m/s^2), accel_cmd (m/s^2, version-correct command
    stream per spec 4.3), enabled (0/1), brake_pressed (0/1); optional should_stop (0/1).

Output JSON keeps the legacy FittedStoppingModel schema under "model" (loadable by
stopping_plant.plant_params_from_legacy_json) plus a "fit_plant_model" metadata block.
"""

from __future__ import annotations

import argparse
import json
import math
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
  PLANT_COEF_KEYS,
  PlantModel,
  plant_params_from_legacy_json,
)

FEATURE_NAMES = PLANT_COEF_KEYS  # ("intercept", "a_ego_prev", "accel_cmd_delayed", "v_ego", "relief", "low_speed", "cmd_x_low_speed")
TELEMETRY_V1_ENGAGEMENT_EXCLUSION_S = 4.0
DEFAULT_EVENT_STORE = Path.home() / ".comma" / "stopping_behavior" / "event_store"
DEFAULT_HOLDOUT_ROUTES = Path(__file__).resolve().parent / "holdout_routes.txt"


def assert_no_lead_features() -> None:
  """Spec 7.5 exclusion (iii): the plant must have no lead-gap feature; pre-dRel-flip routes
  would otherwise need a signals_version exclusion."""
  banned = ("lead", "gap", "d_rel", "drel")
  offenders = [name for name in FEATURE_NAMES if any(token in name.lower() for token in banned)]
  if offenders:
    raise AssertionError(f"plant features must not include lead-gap signals, found: {offenders}")


@dataclass
class EventTrace:
  route: str
  key: dict[str, Any]
  telemetry_version: int
  signals_version: int
  accel_cmd_source: str
  t: np.ndarray
  v_ego: np.ndarray
  a_ego: np.ndarray
  accel_cmd: np.ndarray
  enabled: np.ndarray
  brake_pressed: np.ndarray
  should_stop: np.ndarray | None = None

  def trusted_post_engagement(self) -> bool:
    """Engagement windows are admissible only when accel_cmd is the SENT value (spec 4.3)."""
    return self.telemetry_version >= 2 and self.accel_cmd_source == "carOutput"


@dataclass
class FitResult:
  delay_frames: int
  coefficients: dict[str, float]
  rmse: float
  mae: float
  r2: float
  sample_count: int
  dt_s: float
  delay_sweep: list[dict[str, float]] = field(default_factory=list)


def _step_sample(t_src: np.ndarray, values: np.ndarray, t_grid: np.ndarray) -> np.ndarray:
  """Most-recent-sample (zero-order hold) resampling for flags and commands."""
  idx = np.clip(np.searchsorted(t_src, t_grid, side="right") - 1, 0, len(t_src) - 1)
  return values[idx]


def resample_trace(trace: EventTrace, dt: float) -> EventTrace:
  t0, t1 = float(trace.t[0]), float(trace.t[-1])
  n = int(math.floor((t1 - t0) / dt)) + 1
  t_grid = t0 + np.arange(n) * dt
  return EventTrace(
    route=trace.route,
    key=trace.key,
    telemetry_version=trace.telemetry_version,
    signals_version=trace.signals_version,
    accel_cmd_source=trace.accel_cmd_source,
    t=t_grid,
    v_ego=np.interp(t_grid, trace.t, trace.v_ego),
    a_ego=np.interp(t_grid, trace.t, trace.a_ego),
    accel_cmd=_step_sample(trace.t, trace.accel_cmd, t_grid),
    enabled=_step_sample(trace.t, trace.enabled.astype(bool), t_grid),
    brake_pressed=_step_sample(trace.t, trace.brake_pressed.astype(bool), t_grid),
    should_stop=None if trace.should_stop is None else _step_sample(trace.t, trace.should_stop.astype(bool), t_grid),
  )


def engagement_exclusion_mask(t: np.ndarray, enabled: np.ndarray, exclusion_s: float = TELEMETRY_V1_ENGAGEMENT_EXCLUSION_S) -> np.ndarray:
  """True where a frame falls within `exclusion_s` after an engagement rising edge (incl. the edge).

  A trace that STARTS enabled is treated as a rising edge at its first frame: the engagement
  may have happened just before the recorded window, so the conservative call is to exclude.
  """
  enabled = enabled.astype(bool)
  excluded = np.zeros(len(t), dtype=bool)
  rising = np.flatnonzero(enabled & ~np.concatenate(([False], enabled[:-1])))
  for edge in rising:
    excluded |= (t >= t[edge]) & (t - t[edge] < exclusion_s)
  return excluded


def usable_frame_mask(trace: EventTrace) -> np.ndarray:
  """Frames admissible as features/targets after the spec 7.5 exclusions."""
  mask = trace.enabled.astype(bool) & ~trace.brake_pressed.astype(bool)
  if not trace.trusted_post_engagement():
    mask &= ~engagement_exclusion_mask(trace.t, trace.enabled)
  if trace.should_stop is not None:
    mask &= trace.should_stop.astype(bool)
  finite = np.isfinite(trace.v_ego) & np.isfinite(trace.a_ego) & np.isfinite(trace.accel_cmd)
  return mask & finite


def _design_row(a_prev: float, cmd_delayed: float, v_ego: float, relief_cmd_threshold: float, low_speed_ref: float) -> list[float]:
  relief = max(0.0, cmd_delayed - relief_cmd_threshold)
  low_speed = min(max((low_speed_ref - v_ego) / max(low_speed_ref, 1e-6), 0.0), 1.0)
  return [1.0, a_prev, cmd_delayed, v_ego, relief, low_speed, cmd_delayed * low_speed]


def build_design(traces: list[EventTrace], delay_frames: int, min_speed: float, max_speed: float,
                 relief_cmd_threshold: float, low_speed_ref: float) -> tuple[np.ndarray, np.ndarray]:
  rows: list[list[float]] = []
  targets: list[float] = []
  for trace in traces:
    mask = usable_frame_mask(trace)
    for k in range(delay_frames, len(trace.t) - 1):
      if not (mask[k] and mask[k + 1] and mask[k - delay_frames]):
        continue
      v = float(trace.v_ego[k])
      if not (min_speed <= v <= max_speed):
        continue
      rows.append(_design_row(float(trace.a_ego[k]), float(trace.accel_cmd[k - delay_frames]), v, relief_cmd_threshold, low_speed_ref))
      targets.append(float(trace.a_ego[k + 1]))
  if not rows:
    return np.zeros((0, len(FEATURE_NAMES))), np.zeros((0,))
  return np.asarray(rows, dtype=float), np.asarray(targets, dtype=float)


def _score(y: np.ndarray, y_pred: np.ndarray) -> tuple[float, float, float]:
  err = y_pred - y
  rmse = float(np.sqrt(np.mean(np.square(err))))
  mae = float(np.mean(np.abs(err)))
  denom = float(np.sum(np.square(y - np.mean(y))))
  r2 = 0.0 if denom <= 1e-9 else 1.0 - float(np.sum(np.square(err))) / denom
  return rmse, mae, r2


def fit_plant(traces: list[EventTrace], dt: float, max_delay_frames: int, min_speed: float, max_speed: float,
              relief_cmd_threshold: float, low_speed_ref: float, min_rows: int,
              delay_min_sample_ratio: float = 0.40, delay_rmse_tolerance: float = 0.03) -> FitResult:
  """Least-squares fit with a dead-time sweep; smallest delay within the RMSE tolerance wins
  (same selection policy as the legacy fit_stopping_model)."""
  candidates: list[tuple[int, np.ndarray, float, float, float, int]] = []
  sweep: list[dict[str, float]] = []
  for delay in range(max_delay_frames + 1):
    x, y = build_design(traces, delay, min_speed, max_speed, relief_cmd_threshold, low_speed_ref)
    if len(y) == 0:
      sweep.append({"delay_frames": delay, "sample_count": 0, "rmse": float("inf")})
      continue
    coef, *_ = np.linalg.lstsq(x, y, rcond=None)
    rmse, mae, r2 = _score(y, x @ coef)
    sweep.append({"delay_frames": delay, "sample_count": len(y), "rmse": rmse})
    candidates.append((delay, coef, rmse, mae, r2, len(y)))
  if not candidates:
    raise RuntimeError("Unable to fit plant model: no valid training rows")

  max_rows = max(c[5] for c in candidates)
  floor_rows = max(min_rows, int(max_rows * delay_min_sample_ratio))
  eligible = [c for c in candidates if c[5] >= floor_rows] or candidates
  best_rmse = min(c[2] for c in eligible)
  near_best = [c for c in eligible if c[2] <= best_rmse * (1.0 + delay_rmse_tolerance)]
  delay, coef, rmse, mae, r2, n = min(near_best, key=lambda c: (c[0], c[2]))
  if n < min_rows:
    raise RuntimeError(f"Unable to fit plant model: only {n} rows (min_rows={min_rows})")
  return FitResult(
    delay_frames=delay,
    coefficients={name: float(coef[i]) for i, name in enumerate(FEATURE_NAMES)},
    rmse=rmse, mae=mae, r2=r2, sample_count=n, dt_s=dt, delay_sweep=sweep,
  )


def evaluate_model_on_traces(model_payload: dict[str, Any], traces: list[EventTrace], dt: float,
                             min_speed: float, max_speed: float) -> dict[str, float]:
  """One-step holdout RMSE for any legacy-schema model payload, through the real PlantModel code path."""
  params = plant_params_from_legacy_json(model_payload)
  plant = PlantModel(params, dt)
  x, y = build_design(traces, plant.delay_frames, min_speed, max_speed, params.relief_cmd_threshold, params.low_speed_ref)
  if len(y) == 0:
    return {"sample_count": 0, "rmse": float("nan"), "mae": float("nan")}
  y_pred = np.array([plant.predict_next(row[1], row[2], row[3]) for row in x])
  rmse, mae, _ = _score(y, y_pred)
  return {"sample_count": int(len(y)), "rmse": rmse, "mae": mae}


def load_event_store(store_dir: Path, min_telemetry_version: int = 1) -> list[EventTrace]:
  events_path = store_dir / "events.jsonl"
  if not events_path.is_file():
    raise FileNotFoundError(f"event store index not found: {events_path} (build it with tools/stopping/build_event_store.py)")
  traces: list[EventTrace] = []
  skipped = 0
  with open(events_path) as f:
    for line in f:
      line = line.strip()
      if not line:
        continue
      record = json.loads(line)
      if int(record.get("telemetry_version", 1)) < min_telemetry_version:
        continue
      trace_path = store_dir / record["trace_ref"]
      if not trace_path.is_file():
        skipped += 1
        continue
      with np.load(trace_path) as npz:
        if not all(name in npz for name in ("t", "v_ego", "a_ego", "accel_cmd", "enabled", "brake_pressed")):
          skipped += 1
          continue
        traces.append(EventTrace(
          route=str(record["key"]["route"]),
          key=dict(record["key"]),
          telemetry_version=int(record.get("telemetry_version", 1)),
          signals_version=int(record.get("signals_version", 1)),
          accel_cmd_source=str(record.get("accel_cmd_source", "carControl")),
          t=np.asarray(npz["t"], dtype=float),
          v_ego=np.asarray(npz["v_ego"], dtype=float),
          a_ego=np.asarray(npz["a_ego"], dtype=float),
          accel_cmd=np.asarray(npz["accel_cmd"], dtype=float),
          enabled=np.asarray(npz["enabled"]),
          brake_pressed=np.asarray(npz["brake_pressed"]),
          should_stop=np.asarray(npz["should_stop"]) if "should_stop" in npz else None,
        ))
  if skipped:
    print(f"warning: skipped {skipped} event-store records with missing/incomplete traces", file=sys.stderr)
  return traces


def load_holdout_routes(path: Path) -> set[str]:
  if not path.is_file():
    return set()
  return {line.strip() for line in path.read_text().splitlines() if line.strip() and not line.strip().startswith("#")}


def run_fit(traces: list[EventTrace], dt: float, args: argparse.Namespace, holdout_routes: set[str],
            baseline_payload: dict[str, Any] | None) -> dict[str, Any]:
  train = [resample_trace(tr, dt) for tr in traces if tr.route not in holdout_routes]
  holdout = [resample_trace(tr, dt) for tr in traces if tr.route in holdout_routes]
  fit = fit_plant(train, dt, args.max_delay_frames, args.min_speed, args.max_speed,
                  args.relief_cmd_threshold, args.low_speed_ref, args.min_rows)
  model_payload = {
    "delay_frames": fit.delay_frames,
    "coefficients": fit.coefficients,
    "rmse": fit.rmse, "mae": fit.mae, "r2": fit.r2,
    "sample_count": fit.sample_count,
    "dt_s": dt,
    "relief_cmd_threshold": args.relief_cmd_threshold,
    "low_speed_ref": args.low_speed_ref,
    "model_kind": "linear",
    "feature_names": list(FEATURE_NAMES),
  }
  holdout_eval = evaluate_model_on_traces({"model": model_payload}, holdout, dt, args.min_speed, args.max_speed)
  result: dict[str, Any] = {
    "dt_s": dt,
    "model": model_payload,
    "delay_sweep": fit.delay_sweep,
    "train_events": len(train),
    "holdout_events": len(holdout),
    "holdout_routes": sorted({tr.route for tr in holdout}),
    "holdout": holdout_eval,
    "telemetry_version_counts": {
      "v1": sum(1 for tr in traces if tr.telemetry_version == 1),
      "v2": sum(1 for tr in traces if tr.telemetry_version >= 2),
      "v2_trusted_post_engagement": sum(1 for tr in traces if tr.trusted_post_engagement()),
    },
  }
  if baseline_payload is not None:
    baseline_eval = evaluate_model_on_traces(baseline_payload, holdout, dt, args.min_speed, args.max_speed)
    ratio = holdout_eval["rmse"] / baseline_eval["rmse"] if baseline_eval["sample_count"] and baseline_eval["rmse"] > 0.0 else float("nan")
    result["acceptance"] = {
      "baseline_holdout": baseline_eval,
      "holdout_rmse_ratio": ratio,
      # spec 7.5: holdout RMSE <= 1.1x the archived-fit equivalent on the SAME holdout rows
      "rmse_criterion_met": bool(math.isfinite(ratio) and ratio <= 1.1),
      "leapfrog_alignment_recall": "pending -- run tools/stopping/check_leapfrog_alignment.py against this fit (criterion: recall >= current)",
    }
  return result


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
  parser = argparse.ArgumentParser(description="Refit the 7-feature stopping plant on event-store traces (spec 7.5)")
  parser.add_argument("--event-store", default=str(DEFAULT_EVENT_STORE), help="Event store dir containing events.jsonl (spec 7.1)")
  parser.add_argument("--dt", type=float, default=0.10, help="Primary fit dt in seconds (default 0.10 = 10 Hz)")
  parser.add_argument("--sensitivity-dt", type=float, default=None, help="Optional second dt for a sensitivity variant (e.g. 0.05 = 20 Hz)")
  parser.add_argument("--max-delay-frames", type=int, default=8, help="Dead-time sweep upper bound in frames at --dt")
  parser.add_argument("--min-speed", type=float, default=0.0)
  parser.add_argument("--max-speed", type=float, default=2.0)
  parser.add_argument("--relief-cmd-threshold", type=float, default=-0.25)
  parser.add_argument("--low-speed-ref", type=float, default=1.2)
  parser.add_argument("--min-rows", type=int, default=120)
  parser.add_argument("--holdout-routes", default=str(DEFAULT_HOLDOUT_ROUTES), help="File with one holdout route name per line")
  parser.add_argument("--baseline-json", default=None,
                      help="Archived fit JSON for the acceptance comparison (e.g. docs/stopping/archive/plant_model_20260531T075153Z_all.json)")
  parser.add_argument("--output", required=True, help="Output JSON path")
  return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
  assert_no_lead_features()
  args = parse_args(argv)
  traces = load_event_store(Path(args.event_store))
  if not traces:
    print("error: event store contains no usable traces", file=sys.stderr)
    return 1
  holdout_routes = load_holdout_routes(Path(args.holdout_routes))
  baseline_payload = None
  if args.baseline_json:
    with open(args.baseline_json) as f:
      baseline_payload = json.load(f)

  output: dict[str, Any] = {
    "generated_utc": datetime.now(UTC).isoformat(),
    "event_store": str(args.event_store),
    "events_total": len(traces),
  }
  output.update(run_fit(traces, args.dt, args, holdout_routes, baseline_payload))
  if args.sensitivity_dt is not None:
    output["sensitivity"] = run_fit(traces, args.sensitivity_dt, args, holdout_routes, baseline_payload)

  out_path = Path(args.output)
  out_path.parent.mkdir(parents=True, exist_ok=True)
  with open(out_path, "w") as f:
    json.dump(output, f, indent=2, sort_keys=True)
  model = output["model"]
  holdout = output["holdout"]
  print(f"fit: delay={model['delay_frames']} frames @ {args.dt}s, rows={model['sample_count']}, rmse={model['rmse']:.4f}")
  print(f"holdout: rmse={holdout['rmse']:.4f} (n={holdout['sample_count']}, routes={len(output['holdout_routes'])})")
  if "acceptance" in output:
    acc = output["acceptance"]
    verdict = "PASS" if acc["rmse_criterion_met"] else "FAIL"
    print(f"acceptance: holdout rmse ratio={acc['holdout_rmse_ratio']:.3f} (<= 1.1 required) -> {verdict}; leapfrog-alignment recall pending")
  print(f"wrote {out_path}")
  return 0


if __name__ == "__main__":
  sys.exit(main())
