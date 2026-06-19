from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
  sys.path.insert(0, str(REPO_ROOT))

from openpilot.tools.stopping.stopping_model import FittedStoppingModel
import openpilot.tools.stopping.check_harsh_stops_model as check_model_module
from openpilot.tools.stopping.check_harsh_stops_model import (
  build_result,
  classify_stop_distance,
  classify_pred_leapfrog,
  compute_entry_stop_sharpness_metrics,
  compute_end_stop_sharpness_metrics,
  compute_pred_leapfrog_metrics,
  compute_predicted_lead_distance_metrics,
  controller_should_stop_flags,
  iter_summary_event_groups,
  jerk_window_metrics,
  last_contiguous_index_span,
  stopping_accel_breakpoints,
  simulate_event_with_controller,
  score_event_metrics,
)
from openpilot.selfdrive.controls.lib.longcontrol import force_coast_no_target_pid_brake_cap, force_coast_no_target_pid_brake_step
from openpilot.frogpilot.controls.lib.force_coast import get_force_coast_target_accel


@dataclass
class FakeSample:
  t: float
  v_ego: float
  a_ego: float
  accel_cmd: float | None
  should_stop: bool = True
  distance_to_stop_target_m: float | None = None
  lead_status: bool = False
  lead_d_rel_m: float | None = None
  force_coast: bool = False


def simple_model() -> FittedStoppingModel:
  return FittedStoppingModel(
    delay_frames=3,
    coefficients={
      "intercept": 0.0,
      "a_ego_prev": 0.88,
      "accel_cmd_delayed": 0.12,
      "v_ego": -0.06,
      "relief": 0.15,
      "low_speed": -0.03,
      "cmd_x_low_speed": 0.08,
    },
    rmse=0.1,
    mae=0.08,
    r2=0.9,
    sample_count=300,
    dt_s=0.05,
    relief_cmd_threshold=-0.25,
    low_speed_ref=1.2,
  )


def speed_band_model_from_json() -> FittedStoppingModel:
  return FittedStoppingModel.from_json({
    "delay_frames": 3,
    "coefficients": {
      "intercept": 0.0,
      "a_ego_prev": 0.88,
      "accel_cmd_delayed": 0.12,
      "v_ego": -0.06,
      "relief": 0.15,
      "low_speed": -0.03,
      "cmd_x_low_speed": 0.08,
    },
    "rmse": 0.08,
    "mae": 0.06,
    "r2": 0.92,
    "sample_count": 300,
    "dt_s": 0.05,
    "relief_cmd_threshold": -0.25,
    "low_speed_ref": 1.2,
    "model_kind": "speed_band_linear",
    "speed_split_mps": 0.45,
    "band_coefficients": {
      "low": {
        "intercept": 0.01,
        "a_ego_prev": 0.94,
        "accel_cmd_delayed": 0.20,
        "v_ego": -0.08,
        "relief": 0.24,
        "low_speed": -0.06,
        "cmd_x_low_speed": 0.12,
      },
      "high": {
        "intercept": -0.01,
        "a_ego_prev": 0.80,
        "accel_cmd_delayed": 0.10,
        "v_ego": -0.04,
        "relief": 0.10,
        "low_speed": -0.02,
        "cmd_x_low_speed": 0.04,
      },
    },
    "band_sample_counts": {
      "low": 140,
      "high": 160,
    },
  })


def regression_model_20260208() -> FittedStoppingModel:
  return FittedStoppingModel(
    delay_frames=5,
    coefficients={
      "intercept": -0.04320604031092344,
      "a_ego_prev": 1.010074012410447,
      "accel_cmd_delayed": -0.31926101673078167,
      "v_ego": -0.12176045422426689,
      "relief": 0.534523007098092,
      "low_speed": 0.05826100487596623,
      "cmd_x_low_speed": 0.2548958826663355,
    },
    rmse=0.11811660842898317,
    mae=0.07128108502605691,
    r2=0.9019443311679881,
    sample_count=252,
    dt_s=0.09997653450000143,
    relief_cmd_threshold=-0.25,
    low_speed_ref=1.2,
  )


def regression_model_20260210_all() -> FittedStoppingModel:
  return FittedStoppingModel(
    delay_frames=5,
    coefficients={
      "intercept": 0.7171392187297003,
      "a_ego_prev": 0.9283853183491164,
      "accel_cmd_delayed": -0.38180645096752963,
      "v_ego": -0.853275021307164,
      "relief": 0.7562979111171099,
      "low_speed": -0.6452345225482649,
      "cmd_x_low_speed": 0.4299314042218597,
    },
    rmse=0.25237077078284865,
    mae=0.16290529411941158,
    r2=0.7917169680702614,
    sample_count=3384,
    dt_s=0.10016518399999995,
    relief_cmd_threshold=-0.25,
    low_speed_ref=1.2,
  )


def build_samples(count: int = 120, dt_s: float = 0.05) -> list[FakeSample]:
  samples: list[FakeSample] = []
  v_ego = 1.0
  a_ego = -0.2
  for idx in range(count):
    t = idx * dt_s
    accel_cmd = -0.24 + (0.08 * max(0.0, (count - idx) / count))
    samples.append(FakeSample(
      t=t,
      v_ego=max(0.0, v_ego),
      a_ego=a_ego,
      accel_cmd=accel_cmd,
    ))
    a_ego = max(-1.2, (a_ego * 0.92) - 0.004)
    v_ego = max(0.0, v_ego + (a_ego * dt_s))
  return samples


def build_entry_and_lead_metric_samples() -> list[FakeSample]:
  return [
    FakeSample(t=0.0, v_ego=0.82, a_ego=-0.08, accel_cmd=-0.12, should_stop=False, lead_status=True, lead_d_rel_m=6.2),
    FakeSample(t=0.1, v_ego=0.80, a_ego=-0.10, accel_cmd=-0.12, should_stop=False, lead_status=True, lead_d_rel_m=6.1),
    FakeSample(t=0.2, v_ego=0.78, a_ego=-0.14, accel_cmd=-0.38, should_stop=True, lead_status=True, lead_d_rel_m=5.9),
    FakeSample(t=0.3, v_ego=0.73, a_ego=-0.28, accel_cmd=-0.44, should_stop=True, lead_status=True, lead_d_rel_m=5.7),
    FakeSample(t=0.4, v_ego=0.63, a_ego=-0.40, accel_cmd=-0.46, should_stop=True, lead_status=True, lead_d_rel_m=5.4),
    FakeSample(t=0.5, v_ego=0.47, a_ego=-0.46, accel_cmd=-0.42, should_stop=True, lead_status=True, lead_d_rel_m=5.0),
    FakeSample(t=0.6, v_ego=0.28, a_ego=-0.39, accel_cmd=-0.34, should_stop=True, lead_status=True, lead_d_rel_m=4.6),
    FakeSample(t=0.7, v_ego=0.11, a_ego=-0.22, accel_cmd=-0.28, should_stop=True, lead_status=True, lead_d_rel_m=4.2),
    FakeSample(t=0.8, v_ego=0.03, a_ego=-0.08, accel_cmd=-0.24, should_stop=True, lead_status=True, lead_d_rel_m=4.0),
  ]


def build_regression_seed_samples_ce_event6() -> list[FakeSample]:
  dt_s = 0.09997653450000143
  start_t = 38.159
  initial = [
    FakeSample(t=start_t + (0 * dt_s), v_ego=1.1098029613494873, a_ego=0.40280288457870483, accel_cmd=-0.3443150520324707),
    FakeSample(t=start_t + (1 * dt_s), v_ego=1.073291301727295, a_ego=-0.09874717891216278, accel_cmd=-0.480320543050766),
    FakeSample(t=start_t + (2 * dt_s), v_ego=1.0061943531036377, a_ego=-0.47208571434020996, accel_cmd=-0.6147042512893677),
    FakeSample(t=start_t + (3 * dt_s), v_ego=0.9250645637512207, a_ego=-0.6961544752120972, accel_cmd=-0.7462438344955444),
    FakeSample(t=start_t + (4 * dt_s), v_ego=0.8390381932258606, a_ego=-0.8071169853210449, accel_cmd=-0.8743184804916382),
    FakeSample(t=start_t + (5 * dt_s), v_ego=0.7475503087043762, a_ego=-0.8757506608963013, accel_cmd=-0.8995009064674377),
  ]
  tail = [
    FakeSample(
      t=start_t + ((6 + idx) * dt_s),
      v_ego=0.7475503087043762,
      a_ego=-0.8757506608963013,
      accel_cmd=-0.8995009064674377,
    )
    for idx in range(13)
  ]
  return initial + tail


def build_regression_seed_samples_670_event2() -> list[FakeSample]:
  # Seeded from route 00000670--ac394eda2a signal-event 2 (engaged), observed with high rollout and rebound.
  dt_s = 0.10016518400000001
  initial = [
    FakeSample(t=275.527612402, v_ego=0.990301728, a_ego=-0.419936508, accel_cmd=-0.6355980038642883),
    FakeSample(t=275.627777586, v_ego=0.946920514, a_ego=-0.424563378, accel_cmd=-0.6355980038642883),
    FakeSample(t=275.727910479, v_ego=0.889226973, a_ego=-0.529990494, accel_cmd=-0.6355980038642883),
    FakeSample(t=275.827217911, v_ego=0.830120265, a_ego=-0.575811446, accel_cmd=-0.6355980038642883),
    FakeSample(t=275.928531573, v_ego=0.766679466, a_ego=-0.615593791, accel_cmd=-0.6355980038642883),
    FakeSample(t=276.028204522, v_ego=0.698763847, a_ego=-0.658544362, accel_cmd=-0.6355980038642883),
    FakeSample(t=276.127313623, v_ego=0.626335621, a_ego=-0.701765001, accel_cmd=-0.6355980038642883),
    FakeSample(t=276.228578223, v_ego=0.549463332, a_ego=-0.742492855, accel_cmd=0.0),
    FakeSample(t=276.328497888, v_ego=0.490577906, a_ego=-0.628429651, accel_cmd=0.0),
    FakeSample(t=276.427005277, v_ego=0.448249489, a_ego=-0.502008975, accel_cmd=-0.10000000149011612),
    FakeSample(t=276.527901912, v_ego=0.416863948, a_ego=-0.384274483, accel_cmd=-0.10000000149011612),
    FakeSample(t=276.629012138, v_ego=0.393661886, a_ego=-0.288399667, accel_cmd=-0.10000000149011612),
    FakeSample(t=276.727557651, v_ego=0.371425748, a_ego=-0.251847535, accel_cmd=-0.10000000149011612),
    FakeSample(t=276.828389963, v_ego=0.349975854, a_ego=-0.224917307, accel_cmd=0.0),
    FakeSample(t=276.928830717, v_ego=0.336013973, a_ego=-0.172482044, accel_cmd=0.0),
    FakeSample(t=277.027400606, v_ego=0.318587959, a_ego=-0.169424742, accel_cmd=0.0),
    FakeSample(t=277.128455780, v_ego=0.310015947, a_ego=-0.112147771, accel_cmd=0.0),
    FakeSample(t=277.227707849, v_ego=0.303226709, a_ego=-0.088361032, accel_cmd=-0.10000000149011612),
    FakeSample(t=277.327342549, v_ego=0.288577884, a_ego=-0.149834886, accel_cmd=-0.10000000149011612),
    FakeSample(t=277.427030487, v_ego=0.274916887, a_ego=-0.119609423, accel_cmd=-0.10000000149011612),
    FakeSample(t=277.527407169, v_ego=0.259314656, a_ego=-0.158567756, accel_cmd=-0.10000000149011612),
    FakeSample(t=277.627245927, v_ego=0.243689865, a_ego=-0.153263301, accel_cmd=-0.10000000149011612),
  ]
  tail = [
    FakeSample(
      t=initial[-1].t + ((idx + 1) * dt_s),
      v_ego=initial[-1].v_ego,
      a_ego=initial[-1].a_ego,
      accel_cmd=initial[-1].accel_cmd,
    )
    for idx in range(56)
  ]
  return initial + tail


def test_jerk_window_metrics_handles_short_window() -> None:
  times = [0.0, 0.1, 0.2]
  predicted = [-0.1, -0.3, -0.5]
  max_jerk, pred_min_a = jerk_window_metrics(times, predicted, hold_time_s=times[-1])
  assert max_jerk is not None
  assert pred_min_a == -0.5


def test_simulate_event_with_controller_returns_predictions() -> None:
  samples = build_samples()
  model = simple_model()
  result = simulate_event_with_controller(
    samples=samples,
    start_idx=10,
    hold_idx=70,
    model=model,
    stopping_speed_breakpoint=0.4,
    stop_accel=-2.0,
  )

  assert len(result["times"]) == 61
  assert len(result["predicted_a_ego"]) == 61
  assert len(result["predicted_v_ego"]) == 61
  assert result["pred_end_stop_jerk_mps3"] is not None
  assert result["pred_min_a_ego_mps2"] <= min(result["predicted_a_ego"][-30:])
  assert result["pred_rollout_distance_m"] > 0.0
  assert "pred_entry_stop_jerk_mps3" in result
  assert "pred_lead_distance_hold_m" in result


def test_simulate_event_with_controller_accepts_speed_band_model_loaded_from_json() -> None:
  result = simulate_event_with_controller(
    samples=build_entry_and_lead_metric_samples(),
    start_idx=0,
    hold_idx=8,
    model=speed_band_model_from_json(),
    stopping_speed_breakpoint=0.4,
    stop_accel=-2.0,
    controller_should_stop_source="recorded",
  )

  assert result["pred_end_stop_jerk_mps3"] is not None
  assert result["pred_min_a_ego_mps2"] is not None
  assert result["pred_entry_stop_jerk_mps3"] is not None
  assert result["pred_entry_stop_cmd_jerk_mps3"] is not None
  assert result["pred_lead_distance_stop_entry_m"] is not None
  assert "pred_lead_distance_hold_m" in result
  assert "pred_speed_rebound_while_should_stop_mps" in result
  assert "pred_should_stop_unexpected_accel_mps2" in result


def test_simulate_event_with_controller_forwards_distance_to_stop_target(monkeypatch) -> None:
  seen_distances: list[float | None] = []

  class SpyController:
    def seed_command_history(self, commands):
      return None

    def update(self, **kwargs):
      seen_distances.append(kwargs.get("distance_to_stop_target_m"))
      return SimpleNamespace(output_accel=kwargs["output_accel"], release_lock_active=False)

  samples = [
    FakeSample(t=0.0, v_ego=0.80, a_ego=-0.10, accel_cmd=-0.20, should_stop=True, distance_to_stop_target_m=0.80),
    FakeSample(t=0.1, v_ego=0.65, a_ego=-0.18, accel_cmd=-0.24, should_stop=True, distance_to_stop_target_m=0.55),
    FakeSample(t=0.2, v_ego=0.42, a_ego=-0.22, accel_cmd=-0.28, should_stop=True, distance_to_stop_target_m=0.32),
    FakeSample(t=0.3, v_ego=0.18, a_ego=-0.12, accel_cmd=-0.22, should_stop=True, distance_to_stop_target_m=0.10),
    FakeSample(t=0.4, v_ego=0.04, a_ego=-0.05, accel_cmd=-0.20, should_stop=True, distance_to_stop_target_m=0.0),
  ]

  monkeypatch.setattr(check_model_module, "StoppingController", SpyController)
  simulate_event_with_controller(
    samples=samples,
    start_idx=0,
    hold_idx=4,
    model=simple_model(),
    stopping_speed_breakpoint=0.4,
    stop_accel=-2.0,
    controller_should_stop_source="recorded",
  )

  assert seen_distances[:4] == pytest.approx([0.80, 0.55, 0.32, 0.10], abs=1e-12)


def test_simulate_event_with_controller_blocks_far_lead_release_when_force_coast_active(monkeypatch) -> None:
  seen_should_stop: list[bool] = []
  seen_outputs: list[float] = []

  class SpyController:
    def seed_command_history(self, commands):
      return None

    def update(self, **kwargs):
      seen_should_stop.append(bool(kwargs["should_stop"]))
      seen_outputs.append(float(kwargs["output_accel"]))
      return SimpleNamespace(output_accel=kwargs["output_accel"], release_lock_active=False)

  samples = [
    FakeSample(t=0.0, v_ego=0.009, a_ego=0.0, accel_cmd=-0.50, should_stop=True, lead_status=True, lead_d_rel_m=24.26, force_coast=True),
    FakeSample(t=0.1, v_ego=0.009, a_ego=0.0, accel_cmd=-0.40, should_stop=True, lead_status=True, lead_d_rel_m=24.26, force_coast=True),
    FakeSample(t=0.2, v_ego=0.009, a_ego=0.0, accel_cmd=-0.30, should_stop=True, lead_status=True, lead_d_rel_m=24.26, force_coast=True),
  ]

  monkeypatch.setattr(check_model_module, "StoppingController", SpyController)
  simulate_event_with_controller(
    samples=samples,
    start_idx=0,
    hold_idx=2,
    model=simple_model(),
    stopping_speed_breakpoint=0.4,
    stop_accel=-2.0,
    controller_should_stop_source="recorded",
  )

  assert seen_should_stop == [True, True]
  assert seen_outputs[0] == pytest.approx(-0.50, abs=1e-12)


def test_simulate_event_with_controller_caps_force_coast_no_target_pid_spike(monkeypatch) -> None:
  seen_outputs: list[float] = []

  class SpyController:
    def seed_command_history(self, commands):
      return None

    def update(self, **kwargs):
      seen_outputs.append(float(kwargs["output_accel"]))
      return SimpleNamespace(output_accel=kwargs["output_accel"], release_lock_active=False)

  samples = [
    FakeSample(t=0.0, v_ego=4.66, a_ego=-2.04, accel_cmd=-1.44, should_stop=False, distance_to_stop_target_m=-1.0, force_coast=True),
    FakeSample(t=0.1, v_ego=4.50, a_ego=-1.80, accel_cmd=-1.44, should_stop=False, distance_to_stop_target_m=-1.0, force_coast=True),
    FakeSample(t=0.2, v_ego=4.30, a_ego=-1.60, accel_cmd=-1.44, should_stop=False, distance_to_stop_target_m=-1.0, force_coast=True),
  ]

  monkeypatch.setattr(check_model_module, "StoppingController", SpyController)
  simulate_event_with_controller(
    samples=samples,
    start_idx=0,
    hold_idx=2,
    model=simple_model(),
    stopping_speed_breakpoint=0.4,
    stop_accel=-2.0,
    controller_should_stop_source="recorded",
  )

  assert seen_outputs[0] == pytest.approx(force_coast_no_target_pid_brake_cap(4.66, get_force_coast_target_accel(4.66, 0.2)), abs=1e-12)
  assert seen_outputs[0] > -1.44


def test_simulate_event_with_controller_adds_force_coast_no_target_braking_when_recorded_cmd_coasts(monkeypatch) -> None:
  seen_outputs: list[float] = []

  class SpyController:
    def seed_command_history(self, commands):
      return None

    def update(self, **kwargs):
      seen_outputs.append(float(kwargs["output_accel"]))
      return SimpleNamespace(output_accel=kwargs["output_accel"], release_lock_active=False)

  samples = [
    FakeSample(t=0.0, v_ego=4.66, a_ego=0.0, accel_cmd=0.0, should_stop=False, distance_to_stop_target_m=-1.0, force_coast=True),
    FakeSample(t=0.1, v_ego=4.60, a_ego=0.0, accel_cmd=0.0, should_stop=False, distance_to_stop_target_m=-1.0, force_coast=True),
  ]

  monkeypatch.setattr(check_model_module, "StoppingController", SpyController)
  simulate_event_with_controller(
    samples=samples,
    start_idx=0,
    hold_idx=1,
    model=simple_model(),
    stopping_speed_breakpoint=0.4,
    stop_accel=-2.0,
    controller_should_stop_source="recorded",
  )

  assert seen_outputs[0] == pytest.approx(-force_coast_no_target_pid_brake_step(4.66), abs=1e-12)
  assert seen_outputs[0] < 0.0


def test_simulate_event_with_controller_regression_seed_ce_event6_limits_predicted_jerk() -> None:
  # Seeded from route 000006ce--d41951b402 signal-event 6 (engaged, harsh final stop).
  samples = build_regression_seed_samples_ce_event6()
  model = regression_model_20260208()
  result = simulate_event_with_controller(
    samples=samples,
    start_idx=5,
    hold_idx=18,
    model=model,
    stopping_speed_breakpoint=0.4,
    stop_accel=-2.0,
  )

  assert result["pred_end_stop_jerk_mps3"] is not None
  assert result["pred_end_stop_jerk_mps3"] <= 0.80


def test_simulate_event_with_controller_regression_seed_670_event2_limits_rollout_and_leapfrog() -> None:
  samples = build_regression_seed_samples_670_event2()
  model = regression_model_20260210_all()
  result = simulate_event_with_controller(
    samples=samples,
    start_idx=5,
    hold_idx=len(samples) - 1,
    model=model,
    stopping_speed_breakpoint=0.4,
    stop_accel=-2.0,
  )

  assert result["pred_rollout_distance_m"] <= 3.00
  assert result["pred_speed_rebound_while_should_stop_mps"] <= 0.55
  assert result["pred_should_stop_unexpected_accel_mps2"] <= 0.45


def test_simulate_event_with_controller_regression_seed_fa_event13_limits_leapfrog() -> None:
  event = SeedEvent(
    name="000006fa_f6612b6cbc_ev13",
    window_len=117,
    start_v_ego=1.4700915813446045,
    start_a_ego=-1.0285084247589111,
    cmd_history=(
      -1.4364358186721802,
      -1.3292930126190186,
      -1.2893695831298828,
      -1.221420407295227,
      -1.1313979625701904,
      -1.087642788887024,
      -1.087642788887024,
      -1.087642788887024,
    ),
  )
  model = regression_model_20260210_all_events()
  samples, start_idx, hold_idx = build_seed_samples(event, model)
  result = simulate_event_with_controller(
    samples=samples,
    start_idx=start_idx,
    hold_idx=hold_idx,
    model=model,
    stopping_speed_breakpoint=0.4,
    stop_accel=-2.0,
  )

  assert result["pred_speed_rebound_while_should_stop_mps"] <= 0.77
  assert result["pred_should_stop_unexpected_accel_mps2"] <= 0.85


def test_score_event_metrics_penalizes_rollout_and_harsh_decel() -> None:
  smooth_short = score_event_metrics(pred_jerk=0.42, pred_min_a=-0.95, pred_rollout_m=1.2, max_rollout_m=2.0)
  smooth_long = score_event_metrics(pred_jerk=0.42, pred_min_a=-0.95, pred_rollout_m=2.8, max_rollout_m=2.0)
  harsh_short = score_event_metrics(pred_jerk=0.92, pred_min_a=-1.30, pred_rollout_m=1.2, max_rollout_m=2.0)

  assert smooth_short < smooth_long
  assert smooth_short < harsh_short


def test_score_event_metrics_prefers_midband_lead_hold_gap() -> None:
  center = score_event_metrics(
    pred_jerk=0.42,
    pred_min_a=-0.95,
    pred_rollout_m=3.8,
    max_rollout_m=2.0,
    pred_lead_hold_m=2.25,
    min_lead_hold_m=1.5,
    max_lead_hold_m=3.0,
  )
  edge = score_event_metrics(
    pred_jerk=0.42,
    pred_min_a=-0.95,
    pred_rollout_m=1.0,
    max_rollout_m=2.0,
    pred_lead_hold_m=3.0,
    min_lead_hold_m=1.5,
    max_lead_hold_m=3.0,
  )
  outside = score_event_metrics(
    pred_jerk=0.42,
    pred_min_a=-0.95,
    pred_rollout_m=1.0,
    max_rollout_m=2.0,
    pred_lead_hold_m=3.6,
    min_lead_hold_m=1.5,
    max_lead_hold_m=3.0,
  )

  assert center < edge
  assert edge < outside


def test_score_event_metrics_respects_recorded_wide_gap_baseline() -> None:
  without_recorded = score_event_metrics(
    pred_jerk=0.42,
    pred_min_a=-0.95,
    pred_rollout_m=1.0,
    max_rollout_m=2.0,
    pred_lead_hold_m=3.6,
    min_lead_hold_m=1.5,
    max_lead_hold_m=3.0,
  )
  with_recorded = score_event_metrics(
    pred_jerk=0.42,
    pred_min_a=-0.95,
    pred_rollout_m=1.0,
    max_rollout_m=2.0,
    pred_lead_hold_m=3.6,
    recorded_lead_hold_m=3.7,
    min_lead_hold_m=1.5,
    max_lead_hold_m=3.0,
  )

  assert with_recorded < without_recorded


def test_score_event_metrics_can_enforce_absolute_wide_gap_limit() -> None:
  relaxed = score_event_metrics(
    pred_jerk=0.42,
    pred_min_a=-0.95,
    pred_rollout_m=1.0,
    max_rollout_m=2.0,
    pred_lead_hold_m=5.6,
    recorded_lead_hold_m=5.8,
    min_lead_hold_m=2.5,
    max_lead_hold_m=5.0,
  )
  absolute = score_event_metrics(
    pred_jerk=0.42,
    pred_min_a=-0.95,
    pred_rollout_m=1.0,
    max_rollout_m=2.0,
    pred_lead_hold_m=5.6,
    recorded_lead_hold_m=5.8,
    min_lead_hold_m=2.5,
    max_lead_hold_m=5.0,
    allow_recorded_lead_hold_long_slack=False,
  )

  assert absolute > relaxed


def test_score_event_metrics_penalizes_cmd_jerk_and_accel_step_exceedance() -> None:
  baseline = score_event_metrics(
    pred_jerk=0.42,
    pred_min_a=-0.95,
    pred_rollout_m=1.2,
    max_rollout_m=2.0,
    pred_cmd_jerk=2.5,
    max_cmd_jerk=3.0,
    pred_accel_step=0.05,
    max_accel_step=0.08,
  )
  harsher = score_event_metrics(
    pred_jerk=0.42,
    pred_min_a=-0.95,
    pred_rollout_m=1.2,
    max_rollout_m=2.0,
    pred_cmd_jerk=4.2,
    max_cmd_jerk=3.0,
    pred_accel_step=0.12,
    max_accel_step=0.08,
  )
  assert baseline < harsher


def test_compute_end_stop_sharpness_metrics_detects_cmd_jerk_and_accel_step() -> None:
  times = [0.0, 0.1, 0.2, 0.3, 0.4, 0.5]
  predicted_a = [-0.45, -0.43, -0.40, -0.15, -0.05, -0.02]
  predicted_cmd = [-0.50, -0.45, -0.42, -0.20, -0.05, -0.03]
  cmd_jerk, accel_step = compute_end_stop_sharpness_metrics(
    times=times,
    predicted_a=predicted_a,
    hold_time_s=0.3,
    predicted_cmd=predicted_cmd,
  )
  assert cmd_jerk is not None
  assert cmd_jerk >= 2.0
  assert accel_step is not None
  assert accel_step >= 0.20


def test_compute_entry_stop_sharpness_metrics_detects_cmd_jerk_and_accel_step() -> None:
  times = [0.0, 0.1, 0.2, 0.3, 0.4, 0.5]
  predicted_a = [-0.08, -0.10, -0.12, -0.34, -0.42, -0.44]
  predicted_cmd = [-0.10, -0.10, -0.12, -0.40, -0.45, -0.45]
  accel_jerk, accel_step, cmd_jerk, cmd_step = compute_entry_stop_sharpness_metrics(
    times=times,
    predicted_a=predicted_a,
    entry_time_s=0.2,
    predicted_cmd=predicted_cmd,
  )
  assert accel_jerk is not None
  assert accel_jerk >= 1.5
  assert accel_step is not None
  assert accel_step >= 0.12
  assert cmd_jerk is not None
  assert cmd_jerk >= 2.0
  assert cmd_step is not None
  assert cmd_step >= 0.15


def test_compute_predicted_lead_distance_metrics_reports_entry_and_hold_gaps() -> None:
  samples = [
    FakeSample(t=0.0, v_ego=0.60, a_ego=-0.10, accel_cmd=-0.10, should_stop=False, lead_status=True, lead_d_rel_m=6.0),
    FakeSample(t=0.1, v_ego=0.45, a_ego=-0.20, accel_cmd=-0.20, should_stop=True, lead_status=True, lead_d_rel_m=5.5),
    FakeSample(t=0.2, v_ego=0.20, a_ego=-0.30, accel_cmd=-0.30, should_stop=True, lead_status=True, lead_d_rel_m=5.0),
    FakeSample(t=0.3, v_ego=0.02, a_ego=-0.05, accel_cmd=-0.25, should_stop=True, lead_status=True, lead_d_rel_m=4.8),
  ]
  entry_gap, hold_gap, recorded_hold_gap = compute_predicted_lead_distance_metrics(
    samples=samples,
    replay_sample_indices=[0, 1, 2, 3],
    times=[0.0, 0.1, 0.2, 0.3],
    predicted_v=[0.60, 0.40, 0.16, 0.01],
    predicted_distance_m=[0.0, 0.05, 0.10, 0.12],
    entry_time_s=0.1,
    hold_time_s=0.3,
  )
  assert entry_gap is not None
  assert hold_gap is not None
  assert recorded_hold_gap is not None
  assert hold_gap < entry_gap
  assert recorded_hold_gap == 4.8


def test_compute_predicted_lead_distance_metrics_falls_back_to_final_window_when_not_quite_stopped() -> None:
  samples = [
    FakeSample(t=0.0, v_ego=0.60, a_ego=-0.10, accel_cmd=-0.10, should_stop=False, lead_status=True, lead_d_rel_m=6.0),
    FakeSample(t=0.1, v_ego=0.45, a_ego=-0.20, accel_cmd=-0.20, should_stop=True, lead_status=True, lead_d_rel_m=5.5),
    FakeSample(t=0.2, v_ego=0.20, a_ego=-0.30, accel_cmd=-0.30, should_stop=True, lead_status=True, lead_d_rel_m=5.0),
    FakeSample(t=0.3, v_ego=0.06, a_ego=-0.05, accel_cmd=-0.25, should_stop=True, lead_status=True, lead_d_rel_m=4.8),
  ]
  entry_gap, hold_gap, recorded_hold_gap = compute_predicted_lead_distance_metrics(
    samples=samples,
    replay_sample_indices=[0, 1, 2, 3],
    times=[0.0, 0.1, 0.2, 0.3],
    predicted_v=[0.60, 0.40, 0.16, 0.051],
    predicted_distance_m=[0.0, 0.05, 0.10, 0.12],
    entry_time_s=0.1,
    hold_time_s=0.3,
  )
  assert entry_gap is not None
  assert hold_gap is not None
  assert recorded_hold_gap is not None
  assert hold_gap < entry_gap


def test_classify_stop_distance_uses_lead_hold_when_available() -> None:
  lead_flags, lead_source, lead_value = classify_stop_distance(
    pred_rollout_m=2.8,
    pred_lead_hold_m=3.4,
    max_rollout_m=2.0,
    min_lead_hold_m=1.5,
    max_lead_hold_m=3.0,
  )
  rollout_flags, rollout_source, rollout_value = classify_stop_distance(
    pred_rollout_m=2.8,
    pred_lead_hold_m=None,
    max_rollout_m=2.0,
    min_lead_hold_m=1.5,
    max_lead_hold_m=3.0,
  )

  assert lead_flags == ["pred_lead_distance_hold_long"]
  assert lead_source == "lead_hold"
  assert lead_value == 3.4
  assert rollout_flags == ["pred_rollout"]
  assert rollout_source == "rollout_2mps"
  assert rollout_value == 2.8


def test_classify_stop_distance_relaxes_long_gap_when_recorded_stop_was_already_wide() -> None:
  flags, source, value = classify_stop_distance(
    pred_rollout_m=1.2,
    pred_lead_hold_m=3.6,
    max_rollout_m=2.0,
    min_lead_hold_m=1.5,
    max_lead_hold_m=3.0,
    recorded_lead_hold_m=3.7,
  )

  assert flags == []
  assert source == "lead_hold"
  assert value == 3.6


def test_classify_stop_distance_can_enforce_absolute_wide_gap_limit() -> None:
  flags, source, value = classify_stop_distance(
    pred_rollout_m=1.2,
    pred_lead_hold_m=5.6,
    max_rollout_m=2.0,
    min_lead_hold_m=2.5,
    max_lead_hold_m=5.0,
    recorded_lead_hold_m=5.8,
    allow_recorded_lead_hold_long_slack=False,
  )

  assert flags == ["pred_lead_distance_hold_long"]
  assert source == "lead_hold"
  assert value == 5.6


def test_iter_summary_event_groups_expands_corpus_summary() -> None:
  summary = {
    "host": "comma",
    "routes": [
      {"route": "route-a", "events": [{"event_id": 1}]},
      {"route": "route-b", "host": "commawifi", "events": [{"event_id": 2}]},
    ],
  }

  assert iter_summary_event_groups(summary) == [
    {"host": "comma", "route": "route-a", "events": [{"event_id": 1}]},
    {"host": "commawifi", "route": "route-b", "events": [{"event_id": 2}]},
  ]


def test_compute_pred_leapfrog_metrics_detects_rebound_and_unexpected_accel() -> None:
  max_accel_v_bp, max_accel_bp = stopping_accel_breakpoints(0.4)
  rebound, unexpected_accel = compute_pred_leapfrog_metrics(
    predicted_v=[0.55, 0.36, 0.18, 0.06, 0.04, 0.11, 0.09],
    predicted_a=[-0.60, -0.46, -0.34, -0.18, -0.05, 0.28, 0.12],
    max_accel_v_bp=max_accel_v_bp,
    max_accel_bp=max_accel_bp,
  )
  assert rebound >= 0.069
  assert unexpected_accel >= 0.30


def test_compute_pred_leapfrog_metrics_honors_should_stop_mask() -> None:
  max_accel_v_bp, max_accel_bp = stopping_accel_breakpoints(0.4)
  predicted_v = [0.55, 0.36, 0.18, 0.06, 0.04, 0.21, 0.24]
  predicted_a = [-0.60, -0.46, -0.34, -0.18, -0.05, 0.28, 0.22]

  rebound_all, unexpected_all = compute_pred_leapfrog_metrics(
    predicted_v=predicted_v,
    predicted_a=predicted_a,
    max_accel_v_bp=max_accel_v_bp,
    max_accel_bp=max_accel_bp,
  )
  rebound_masked, unexpected_masked = compute_pred_leapfrog_metrics(
    predicted_v=predicted_v,
    predicted_a=predicted_a,
    max_accel_v_bp=max_accel_v_bp,
    max_accel_bp=max_accel_bp,
    should_stop_mask=[True, True, True, True, False, False, False],
  )

  assert rebound_all > 0.0
  assert unexpected_all > 0.0
  assert rebound_masked == 0.0
  assert unexpected_masked == 0.0


def test_controller_should_stop_flags_uses_recorded_values_with_fallback() -> None:
  samples = [
    FakeSample(t=0.0, v_ego=0.3, a_ego=-0.2, accel_cmd=-0.2, should_stop=True),
    FakeSample(t=0.1, v_ego=0.2, a_ego=-0.2, accel_cmd=-0.2, should_stop=False),
    FakeSample(t=0.2, v_ego=0.1, a_ego=-0.2, accel_cmd=-0.2, should_stop=True),
  ]

  assert controller_should_stop_flags(samples, 0, 2, "constant_true") == [True, True, True]
  assert controller_should_stop_flags(samples, 0, 2, "recorded") == [True, False, True]

  class NoShouldStop:
    def __init__(self) -> None:
      self.t = 0.0
      self.v_ego = 0.0
      self.a_ego = 0.0
      self.accel_cmd = -0.1

  fallback_samples = [NoShouldStop(), NoShouldStop()]
  assert controller_should_stop_flags(fallback_samples, 0, 1, "recorded") == [True, True]


def test_last_contiguous_index_span_returns_latest_active_run() -> None:
  samples = [
    FakeSample(t=0.0, v_ego=0.5, a_ego=-0.1, accel_cmd=-0.2, should_stop=False),
    FakeSample(t=0.1, v_ego=0.4, a_ego=-0.1, accel_cmd=-0.2, should_stop=True),
    FakeSample(t=0.2, v_ego=0.3, a_ego=-0.1, accel_cmd=-0.2, should_stop=True),
    FakeSample(t=0.3, v_ego=0.2, a_ego=-0.1, accel_cmd=-0.2, should_stop=False),
    FakeSample(t=0.4, v_ego=0.1, a_ego=-0.1, accel_cmd=-0.2, should_stop=True),
  ]
  assert last_contiguous_index_span(samples, 0, 4, lambda item: item.should_stop) == (4, 4)


def test_last_contiguous_index_span_returns_none_when_inactive() -> None:
  samples = [
    FakeSample(t=0.0, v_ego=0.5, a_ego=-0.1, accel_cmd=-0.2, should_stop=False),
    FakeSample(t=0.1, v_ego=0.4, a_ego=-0.1, accel_cmd=-0.2, should_stop=False),
  ]
  assert last_contiguous_index_span(samples, 0, 1, lambda item: item.should_stop) is None


def test_simulate_event_with_controller_recorded_should_stop_all_false_zeros_leapfrog_metrics() -> None:
  samples = [
    FakeSample(t=sample.t, v_ego=sample.v_ego, a_ego=sample.a_ego, accel_cmd=sample.accel_cmd, should_stop=False)
    for sample in build_samples(count=50, dt_s=0.05)
  ]
  model = simple_model()
  result = simulate_event_with_controller(
    samples=samples,
    start_idx=10,
    hold_idx=45,
    model=model,
    stopping_speed_breakpoint=0.4,
    stop_accel=-2.0,
    controller_should_stop_source="recorded",
  )
  assert result["pred_speed_rebound_while_should_stop_mps"] == 0.0
  assert result["pred_should_stop_unexpected_accel_mps2"] == 0.0


def test_classify_pred_leapfrog_requires_rebound_for_leapfrog_tag() -> None:
  args = SimpleNamespace(
    max_pred_speed_rebound_while_should_stop=0.08,
    max_pred_should_stop_unexpected_accel=0.10,
  )
  assert classify_pred_leapfrog(0.12, 0.32, args) == ["pred_leapfrog_rebound_should_stop", "pred_leapfrog"]
  assert classify_pred_leapfrog(0.12, 0.05, args) == ["pred_leapfrog_rebound_should_stop"]
  assert classify_pred_leapfrog(0.04, 0.32, args) == []


def test_build_result_reports_harsh_and_leapfrog_counts_separately() -> None:
  args = SimpleNamespace(
    min_events=1,
    min_entry_speed=0.2,
    max_harsh_rate=0.2,
    max_leapfrog_rate=0.2,
    max_leapfrog_count=0,
    max_pred_end_jerk=0.8,
    max_pred_end_cmd_jerk=3.0,
    max_pred_end_accel_step=0.08,
    min_pred_a_floor=-1.1,
    max_pred_rollout_m=2.0,
    min_pred_lead_hold_distance_m=1.5,
    max_pred_lead_hold_distance_m=3.0,
    max_pred_speed_rebound_while_should_stop=0.08,
    max_pred_should_stop_unexpected_accel=0.10,
    command_source="controller",
  )
  rows = [
    {"is_harsh": True, "is_leapfrog": False, "event_score": 1.2},
    {"is_harsh": False, "is_leapfrog": True, "event_score": 0.4},
    {"is_harsh": False, "is_leapfrog": False, "event_score": 0.2},
  ]
  result = build_result(status="pass", reasons=[], event_rows=rows, args=args)
  assert result["events_considered"] == 3
  assert result["harsh_events"] == 1
  assert result["leapfrog_events"] == 1
  assert result["harsh_rate"] == 1.0 / 3.0
  assert result["leapfrog_rate"] == 1.0 / 3.0


@dataclass(frozen=True)
class SeedEvent:
  name: str
  window_len: int
  start_v_ego: float
  start_a_ego: float
  cmd_history: tuple[float, ...]


def regression_model_20260210_all_events() -> FittedStoppingModel:
  # Snapshot from ~/.comma/stopping_behavior/models/stopping_model_20260210T060712Z_all.json (fit from ALL stop events).
  return FittedStoppingModel(
    delay_frames=7,
    coefficients={
      "intercept": 0.7171392187297003,
      "a_ego_prev": 0.9283853183491164,
      "accel_cmd_delayed": -0.38180645096752963,
      "v_ego": -0.853275021307164,
      "relief": 0.7562979111171099,
      "low_speed": -0.6452345225482649,
      "cmd_x_low_speed": 0.4299314042218597,
    },
    rmse=0.11272807291143332,
    mae=0.061612748757286596,
    r2=0.8848686342000676,
    sample_count=624,
    dt_s=0.1000038370002585,
    relief_cmd_threshold=-0.25,
    low_speed_ref=1.2,
  )


def build_seed_samples(event: SeedEvent, model: FittedStoppingModel) -> tuple[list[FakeSample], int, int]:
  start_idx = len(event.cmd_history) - 1
  if start_idx <= 0:
    raise ValueError("SeedEvent cmd_history must include at least start sample")
  if start_idx != model.delay_frames:
    raise ValueError(f"SeedEvent cmd_history length mismatch: expected {model.delay_frames + 1}, got {len(event.cmd_history)}")
  if event.window_len <= 0:
    raise ValueError("SeedEvent window_len must be positive")

  total = len(event.cmd_history) + event.window_len
  dt_s = float(model.dt_s)
  samples = [
    FakeSample(
      t=(idx * dt_s),
      v_ego=event.start_v_ego,
      a_ego=event.start_a_ego,
      accel_cmd=event.cmd_history[idx] if idx < len(event.cmd_history) else event.cmd_history[-1],
    )
    for idx in range(total)
  ]
  hold_idx = len(samples) - 1
  return samples, start_idx, hold_idx
