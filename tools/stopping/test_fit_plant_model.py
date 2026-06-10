"""Offline tests for the plant refit pipeline (spec 7.5 / WP3): synthetic-plant recovery,
telemetry-version exclusion handling (red-team F9), resampling, and the no-lead-feature guard.

Runs without an event store; the JSONL/npz round-trip uses tmp_path.
"""

import json

import numpy as np
import pytest

from openpilot.tools.stopping.fit_plant_model import (
  EventTrace,
  assert_no_lead_features,
  build_design,
  engagement_exclusion_mask,
  evaluate_model_on_traces,
  fit_plant,
  load_event_store,
  resample_trace,
  usable_frame_mask,
)

TRUE_COEF = {
  "intercept": -0.003,
  "a_ego_prev": 0.8715,
  "accel_cmd_delayed": 0.372,
  "v_ego": 0.0135,
  "relief": 0.287,
  "low_speed": -0.0145,
  "cmd_x_low_speed": -0.4514,
}
DT = 0.10
TRUE_DELAY = 1


def _predict(a_prev: float, cmd_delayed: float, v: float) -> float:
  relief = max(0.0, cmd_delayed - (-0.25))
  ls = min(max((1.2 - v) / 1.2, 0.0), 1.0)
  c = TRUE_COEF
  return (c["intercept"] + c["a_ego_prev"] * a_prev + c["accel_cmd_delayed"] * cmd_delayed
          + c["v_ego"] * v + c["relief"] * relief + c["low_speed"] * ls + c["cmd_x_low_speed"] * cmd_delayed * ls)


def _synthetic_trace(route: str, n: int = 400, seed: int = 0, telemetry_version: int = 1,
                     accel_cmd_source: str = "carControl", start_enabled: bool = True) -> EventTrace:
  """Roll the known plant through a braking profile with small process noise."""
  rng = np.random.default_rng(seed)
  t = np.arange(n) * DT
  cmd = -0.15 - 0.45 * (0.5 + 0.5 * np.sin(np.arange(n) * 0.05 + seed))   # varied braking commands
  a = np.zeros(n)
  v = np.zeros(n)
  v[0] = 1.8
  for k in range(n - 1):
    cmd_delayed = cmd[max(0, k - TRUE_DELAY)]
    a[k + 1] = _predict(a[k], cmd_delayed, v[k]) + rng.normal(0.0, 1e-4)
    v[k + 1] = max(0.0, v[k] + a[k + 1] * DT)
  enabled = np.ones(n, dtype=bool)
  if not start_enabled:
    enabled[: n // 4] = False
  return EventTrace(
    route=route,
    key={"route": route, "seg": 0, "hold_mono_ns": 0},
    telemetry_version=telemetry_version,
    signals_version=1,
    accel_cmd_source=accel_cmd_source,
    t=t, v_ego=v, a_ego=a, accel_cmd=cmd,
    enabled=enabled,
    brake_pressed=np.zeros(n, dtype=bool),
  )


def test_no_lead_feature_guard_passes():
  assert_no_lead_features()


def test_fit_recovers_known_plant():
  traces = [_synthetic_trace(f"route_{i}", seed=i, telemetry_version=2, accel_cmd_source="carOutput") for i in range(4)]
  fit = fit_plant(traces, DT, max_delay_frames=4, min_speed=0.0, max_speed=2.0,
                  relief_cmd_threshold=-0.25, low_speed_ref=1.2, min_rows=100)
  assert fit.delay_frames == TRUE_DELAY
  for name, value in TRUE_COEF.items():
    assert fit.coefficients[name] == pytest.approx(value, abs=0.02), f"coefficient {name} not recovered"
  assert fit.rmse < 5e-3


def test_engagement_exclusion_mask_windows():
  t = np.arange(100) * DT
  enabled = np.zeros(100, dtype=bool)
  enabled[20:60] = True   # rising edge at frame 20 (t=2.0)
  enabled[80:] = True     # second rising edge at frame 80 (t=8.0)
  mask = engagement_exclusion_mask(t, enabled, exclusion_s=4.0)
  assert mask[20] and mask[59], "first 4 s after engagement must be excluded"
  assert not mask[60], "frames at/after edge + 4 s are admissible"
  assert mask[80] and mask[99], "every rising edge opens its own window"
  assert not mask[:20].any()


def test_trace_starting_enabled_is_treated_as_rising_edge():
  t = np.arange(100) * DT
  enabled = np.ones(100, dtype=bool)
  mask = engagement_exclusion_mask(t, enabled, exclusion_s=4.0)
  assert mask[0] and mask[39]
  assert not mask[40:].any()


def test_telemetry_v1_excludes_post_engagement_v2_carOutput_admits_it():
  v1 = _synthetic_trace("r1", telemetry_version=1)
  v2_trusted = _synthetic_trace("r2", telemetry_version=2, accel_cmd_source="carOutput")
  # F9: a v2 record WITHOUT the carOutput source switch must stay on the conservative path
  v2_untrusted = _synthetic_trace("r3", telemetry_version=2, accel_cmd_source="carControl")
  n_excluded = int(4.0 / DT)
  assert not usable_frame_mask(v1)[:n_excluded].any()
  assert usable_frame_mask(v2_trusted)[:n_excluded].all()
  assert not usable_frame_mask(v2_untrusted)[:n_excluded].any()


def test_brake_pressed_frames_excluded():
  trace = _synthetic_trace("r1", telemetry_version=2, accel_cmd_source="carOutput")
  trace.brake_pressed[100:120] = True
  mask = usable_frame_mask(trace)
  assert not mask[100:120].any()
  assert mask[120:140].all()
  # design rows touching the brake window (target, feature, or delayed cmd) are dropped
  x_clean, _ = build_design([_synthetic_trace("r1", telemetry_version=2, accel_cmd_source="carOutput")], 1, 0.0, 2.0, -0.25, 1.2)
  x_masked, _ = build_design([trace], 1, 0.0, 2.0, -0.25, 1.2)
  assert len(x_masked) < len(x_clean)


def test_resample_identity_on_grid():
  trace = _synthetic_trace("r1")
  resampled = resample_trace(trace, DT)
  assert resampled.t.shape == trace.t.shape
  np.testing.assert_allclose(resampled.v_ego, trace.v_ego, atol=1e-9)
  np.testing.assert_allclose(resampled.accel_cmd, trace.accel_cmd, atol=1e-9)


def test_resample_downsamples_100hz_to_10hz():
  n = 1000
  t = np.arange(n) * 0.01
  trace = EventTrace(route="r", key={"route": "r", "seg": 0, "hold_mono_ns": 0}, telemetry_version=2,
                     signals_version=1, accel_cmd_source="carOutput",
                     t=t, v_ego=2.0 - t, a_ego=np.full(n, -1.0), accel_cmd=np.full(n, -0.5),
                     enabled=np.ones(n, dtype=bool), brake_pressed=np.zeros(n, dtype=bool))
  resampled = resample_trace(trace, 0.1)
  assert len(resampled.t) == 100
  np.testing.assert_allclose(np.diff(resampled.t), 0.1, atol=1e-9)
  np.testing.assert_allclose(resampled.v_ego, 2.0 - resampled.t, atol=1e-9)


def test_evaluate_model_through_plantmodel_codepath():
  traces = [_synthetic_trace("r_hold", seed=9, telemetry_version=2, accel_cmd_source="carOutput")]
  payload = {"model": {
    "delay_frames": TRUE_DELAY, "coefficients": TRUE_COEF, "rmse": 0.0, "mae": 0.0, "r2": 1.0,
    "sample_count": 1, "dt_s": DT, "relief_cmd_threshold": -0.25, "low_speed_ref": 1.2, "model_kind": "linear",
  }}
  result = evaluate_model_on_traces(payload, traces, DT, 0.0, 2.0)
  assert result["sample_count"] > 100
  assert result["rmse"] < 5e-3, "the generating model must score near-zero holdout RMSE"


def test_event_store_round_trip(tmp_path):
  store = tmp_path / "event_store"
  (store / "events").mkdir(parents=True)
  trace = _synthetic_trace("route_a", telemetry_version=2, accel_cmd_source="carOutput")
  np.savez(store / "events" / "ev0.npz", t=trace.t, v_ego=trace.v_ego, a_ego=trace.a_ego,
           accel_cmd=trace.accel_cmd, enabled=trace.enabled, brake_pressed=trace.brake_pressed)
  record = {
    "key": {"route": "route_a", "seg": 3, "hold_mono_ns": 123},
    "telemetry_version": 2, "signals_version": 2,
    "accel_cmd_source": "carOutput",
    "trace_ref": "events/ev0.npz",
  }
  missing = {
    "key": {"route": "route_b", "seg": 0, "hold_mono_ns": 5},
    "telemetry_version": 1, "signals_version": 1,
    "trace_ref": "events/missing.npz",
  }
  with open(store / "events.jsonl", "w") as f:
    f.write(json.dumps(record) + "\n")
    f.write(json.dumps(missing) + "\n")
  traces = load_event_store(store)
  assert len(traces) == 1
  assert traces[0].route == "route_a"
  assert traces[0].trusted_post_engagement()
  np.testing.assert_allclose(traces[0].a_ego, trace.a_ego)
