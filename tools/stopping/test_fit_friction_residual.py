"""Offline tests for the friction-residual fitter (DEVELOPMENT TOOL -- NOT A GATE).

Synthetic recovery of a known Stribeck curve, the settle-window extractor, detector-dup dedup, the
leave-one-route-out holdout, the harsh-vs-smooth ranking check, and the JSONL/npz round trip.
Runs without a real event store (uses tmp_path).
"""

import json

import numpy as np
import pytest

from openpilot.selfdrive.controls.lib.stopping_plant import FrictionResidual, FrictionResidualParams
from openpilot.tools.stopping import fit_friction_residual as FF
from openpilot.tools.stopping.fit_friction_residual import (
  SettleTrace,
  build_archive,
  extract_settle_window,
  fit_curve,
  gather_residual_samples,
  insample_eval,
  leave_one_route_out,
  load_settle_traces,
  predict_peak_decel,
)

TRUE = FrictionResidualParams(c0=-0.05, c1=0.80, v0=0.06)


def _curve(v: np.ndarray) -> np.ndarray:
  return TRUE.c0 + TRUE.c1 * np.exp(-np.maximum(0.0, v) / TRUE.v0)


def test_fit_recovers_known_curve():
  """Sample the known curve over the settle speed band and recover its parameters."""
  rng = np.random.default_rng(0)
  v = rng.uniform(0.03, 0.30, size=4000)
  resid = _curve(v) + rng.normal(0.0, 1e-3, size=v.size)
  p = fit_curve(v, resid)
  assert p.c0 == pytest.approx(TRUE.c0, abs=0.02)
  assert p.c1 == pytest.approx(TRUE.c1, abs=0.05)
  assert p.v0 == pytest.approx(TRUE.v0, abs=0.01)


def test_fit_too_few_samples_raises():
  with pytest.raises(RuntimeError, match="need >=4"):
    fit_curve(np.array([0.05, 0.06]), np.array([0.2, 0.1]))


def _synthetic_stop(route: str, scale: float, seed: int, n: int = 60) -> SettleTrace:
  """A settle window sweeping 0.30 -> 0.05 m/s. `scale` sets per-stop harshness.

  Matches the real-data mechanism (verified on the event store): a harsher stop has a DEEPER wheel
  decel AND a larger friction excursion, so peak |a_imu| rises with scale. The friction residual
  follows the shared population curve (scaled by `scale`), which is what the v-curve fit recovers; the
  per-stop peak is dominated by the wheel trajectory the prediction adds the curve to."""
  rng = np.random.default_rng(seed)
  v = np.linspace(0.30, 0.05, n)
  a_wheel = -0.5 * scale * np.ones(n) + 0.01 * rng.normal(size=n)   # deeper wheel decel when harsher
  resid = scale * _curve(v) + rng.normal(0.0, 1e-3, size=n)
  a_imu = a_wheel + resid
  peak_decel = float(np.max(np.abs(a_imu)))
  return SettleTrace(route=route, key={"route": route, "seg": 0, "hold_mono_ns": seed},
                     v=v, a_wheel=a_wheel, resid=resid, peak_imu_decel=peak_decel,
                     peak_imu_jerk=20.0 + 30.0 * scale, touched_by_disengage_or_brake=False)


def test_predict_peak_decel_uses_wheel_plus_residual():
  """predict_peak_decel adds the friction curve to the per-stop wheel trace (the FrictionPlant
  semantics) -- it must equal max|a_wheel + resid_model(v)|."""
  st = _synthetic_stop("r", scale=1.0, seed=1)
  f = FrictionResidual(TRUE)
  expected = float(np.max(np.abs(st.a_wheel + _curve(st.v))))
  assert predict_peak_decel(f, st) == pytest.approx(expected, abs=1e-9)


def test_holdout_ranks_harsh_above_smooth():
  """Leave-one-route-out across routes with varied grab amplitude: the model must rank the harsh
  stops' predicted peak decel above the smooth ones (the project's stated ranking requirement)."""
  traces = []
  scales = [0.4, 0.7, 1.0, 1.3, 1.6, 1.9]
  for i, sc in enumerate(scales):
    # two stops per route so each held-out fold has data and the global fit stays well-posed
    traces.append(_synthetic_stop(f"route_{i}", scale=sc, seed=10 * i + 1))
    traces.append(_synthetic_stop(f"route_{i}", scale=sc + 0.05, seed=10 * i + 2))
  holdout = leave_one_route_out(traces)
  assert holdout.n_routes == len(scales)
  assert len(holdout.predicted) == len(traces)
  assert holdout.spearman > 0.8, f"holdout must rank harsh-vs-smooth (spearman={holdout.spearman})"
  hvs = FF._harsh_vs_smooth(holdout)
  assert hvs["ranks_harsh_above_smooth"], hvs


def test_velocity_only_curve_cannot_rank_without_wheel_trace():
  """Honesty pin: max|resid_model(v)| alone is near-constant across stops (all sweep the same band),
  so the discrimination MUST come from adding the per-stop wheel trace. This documents the assess
  finding so a future refactor cannot silently drop the wheel term and still claim ranking."""
  scales = [0.5, 1.0, 1.5, 2.0]
  traces = [_synthetic_stop(f"r{i}", scale=sc, seed=i) for i, sc in enumerate(scales)]
  f = FrictionResidual(TRUE)
  # NOTE: synthetic stops all sweep an identical v grid, so v-only is exactly constant here.
  v_only = [float(np.max(np.abs(_curve(t.v)))) for t in traces]
  assert np.std(v_only) < 1e-9
  with_wheel = [predict_peak_decel(f, t) for t in traces]
  assert np.std(with_wheel) > 0.0   # the wheel trace restores stop-to-stop discrimination


def _write_event(store, route, seg, hold_ns, v, a_ego, a_imu, enabled=None, brake=None,
                 peak_decel=0.5, peak_jerk=25.0):
  events_dir = store / "events"
  events_dir.mkdir(parents=True, exist_ok=True)
  n = len(v)
  t = np.arange(n) * 0.01
  enabled = np.ones(n, dtype=np.uint8) if enabled is None else enabled
  brake = np.zeros(n, dtype=np.uint8) if brake is None else brake
  ref = f"events/{route}__{seg}__{hold_ns}.npz"
  np.savez(store / ref, t=t, v_ego=v, a_ego=a_ego, a_long_imu=a_imu,
           accel_cmd=np.full(n, -0.3), enabled=enabled, brake_pressed=brake,
           should_stop=np.ones(n, dtype=np.uint8))
  return {
    "key": {"route": route, "seg": seg, "hold_mono_ns": hold_ns},
    "trace_ref": ref,
    "metrics_100hz": {"settle_peak_imu_decel": peak_decel, "settle_peak_imu_jerk": peak_jerk},
  }


def _stop_arrays(scale=1.0, n=80):
  # sweep 0.30 -> 0.04 so a genuine standstill (<=0.06) lands in the window; deeper wheel when harsher
  v = np.linspace(0.30, 0.04, n)
  a_wheel = -0.5 * scale * np.ones(n)
  a_imu = a_wheel + scale * _curve(v)
  return v, a_wheel, a_imu


def test_extract_settle_window_finds_engaged_settle(tmp_path):
  v, a_wheel, a_imu = _stop_arrays()
  npz = {"t": np.arange(len(v)) * 0.01, "v_ego": v, "a_ego": a_wheel, "a_long_imu": a_imu,
         "enabled": np.ones(len(v), dtype=np.uint8), "brake_pressed": np.zeros(len(v), dtype=np.uint8)}
  out = extract_settle_window(npz)
  assert out is not None
  v_w, a_wheel_w, resid_w, touched = out
  assert not touched
  assert len(v_w) >= 2
  assert v_w.min() <= FF.SETTLE_STANDSTILL_SPEED


def test_extract_returns_none_without_standstill(tmp_path):
  v = np.linspace(2.0, 0.5, 50)   # never reaches the standstill band
  npz = {"t": np.arange(50) * 0.01, "v_ego": v, "a_ego": -np.ones(50), "a_long_imu": -np.ones(50),
         "enabled": np.ones(50, dtype=np.uint8), "brake_pressed": np.zeros(50, dtype=np.uint8)}
  assert extract_settle_window(npz) is None


def test_load_dedups_same_route_same_grab(tmp_path):
  """One physical stop seen by the speed AND signal detectors (identical grab signature) must
  collapse to a single SettleTrace."""
  store = tmp_path / "event_store"
  v, a_wheel, a_imu = _stop_arrays(scale=1.2)
  e_speed = _write_event(store, "route_a", 5, 100, v, a_wheel, a_imu, peak_decel=0.571, peak_jerk=18.67)
  e_signal = _write_event(store, "route_a", 5, 101, v, a_wheel, a_imu, peak_decel=0.571, peak_jerk=18.67)
  v2, w2, i2 = _stop_arrays(scale=0.6)
  e_other = _write_event(store, "route_b", 2, 200, v2, w2, i2, peak_decel=0.30, peak_jerk=12.0)
  with open(store / "events.jsonl", "w") as f:
    for e in (e_speed, e_signal, e_other):
      f.write(json.dumps(e) + "\n")
  deduped = load_settle_traces(store, dedup=True)
  raw = load_settle_traces(store, dedup=False)
  assert len(raw) == 3
  assert len(deduped) == 2   # the speed/signal pair collapsed
  assert {t.route for t in deduped} == {"route_a", "route_b"}


def test_load_skips_records_without_imu_metrics(tmp_path):
  store = tmp_path / "event_store"
  v, a_wheel, a_imu = _stop_arrays()
  good = _write_event(store, "route_a", 1, 1, v, a_wheel, a_imu)
  no_imu = {"key": {"route": "route_b", "seg": 1, "hold_mono_ns": 2}, "trace_ref": "events/x.npz",
            "metrics_100hz": {"settle_peak_imu_decel": None, "settle_peak_imu_jerk": None}}
  with open(store / "events.jsonl", "w") as f:
    f.write(json.dumps(good) + "\n")
    f.write(json.dumps(no_imu) + "\n")
  traces = load_settle_traces(store)
  assert len(traces) == 1
  assert traces[0].route == "route_a"


def test_archive_carries_not_a_gate_banner_and_jerk_caveat(tmp_path):
  traces = [_synthetic_stop(f"route_{i}", scale=sc, seed=i) for i, sc in enumerate([0.6, 1.0, 1.4, 1.8, 0.8, 1.2])]
  v, r = gather_residual_samples(traces)
  friction = FrictionResidual(fit_curve(v, r))
  insample = insample_eval(traces, friction)
  holdout = leave_one_route_out(traces)
  archive = build_archive(traces, friction, insample, holdout, "test_store")
  assert "NOT A GATE" in archive["_banner"]
  assert "similarity_gate" in archive["_banner"]
  # jerk must be recorded only, never a fit/holdout target
  assert "NOT a fit" in archive["provenance"]["stored_peak_imu_jerk_distribution"]["note"]
  assert "DECEL channel" in archive["signal"]
  assert archive["friction_residual"]["coef_keys"] == ["c0", "c1", "v0"]


def test_main_round_trip_writes_archive(tmp_path):
  store = tmp_path / "event_store"
  records = []
  for i, sc in enumerate([0.5, 0.8, 1.1, 1.4, 1.7, 2.0]):
    v, a_wheel, a_imu = _stop_arrays(scale=sc)
    peak = float(np.max(np.abs(a_imu)))
    records.append(_write_event(store, f"route_{i}", i, i, v, a_wheel, a_imu, peak_decel=peak, peak_jerk=20.0 + 20.0 * sc))
  with open(store / "events.jsonl", "w") as f:
    for r in records:
      f.write(json.dumps(r) + "\n")
  out = tmp_path / "friction.json"
  rc = FF.main(["--event-store", str(store), "--output", str(out)])
  assert rc == 0
  data = json.loads(out.read_text())
  # round-trip the archived coefficients back into a usable model
  p = FF.FrictionResidualParams(**data["friction_residual"]["coefficients"])
  assert p.v0 > 0.0
  assert data["holdout_leave_one_route_out"]["n_routes"] == 6


def test_main_exits_nonzero_when_too_few_settles(tmp_path, capsys):
  store = tmp_path / "event_store"
  v, a_wheel, a_imu = _stop_arrays()
  rec = _write_event(store, "route_a", 0, 0, v, a_wheel, a_imu)
  with open(store / "events.jsonl", "w") as f:
    f.write(json.dumps(rec) + "\n")
  rc = FF.main(["--event-store", str(store), "--output", str(tmp_path / "f.json")])
  assert rc == 1
  assert "too few" in capsys.readouterr().err
  assert not (tmp_path / "f.json").exists()
