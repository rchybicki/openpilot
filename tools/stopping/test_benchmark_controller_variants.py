from __future__ import annotations

from dataclasses import dataclass
from types import SimpleNamespace

from openpilot.tools.stopping.benchmark_controller_variants import (
  classify,
  simulate_event_with_inverse_v3_controller,
  simulate_event_with_legacy_controller,
)
from openpilot.tools.stopping.stopping_model import FittedStoppingModel


@dataclass
class FakeSample:
  t: float
  v_ego: float
  a_ego: float
  accel_cmd: float | None
  should_stop: bool = True
  lead_status: bool = False
  lead_d_rel_m: float | None = None
  enabled: bool = True


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


def test_classify_uses_lead_hold_band_instead_of_rollout_when_present() -> None:
  args = SimpleNamespace(
    max_pred_end_jerk=0.70,
    max_pred_end_cmd_jerk=3.0,
    max_pred_end_accel_step=0.08,
    min_pred_a_floor=-1.10,
    max_pred_rollout_m=2.0,
    min_pred_lead_hold_distance_m=2.0,
    max_pred_lead_hold_distance_m=4.0,
    max_pred_speed_rebound_while_should_stop=0.08,
    max_pred_should_stop_unexpected_accel=0.10,
  )
  metrics = {
    "pred_end_stop_jerk_mps3": 0.42,
    "pred_end_stop_cmd_jerk_mps3": 2.5,
    "pred_end_stop_accel_step_mps2": 0.05,
    "pred_min_a_ego_mps2": -0.95,
    "pred_rollout_from_2mps_m": 3.1,
    "pred_lead_distance_hold_m": 3.0,
    "pred_speed_rebound_while_should_stop_mps": 0.0,
    "pred_should_stop_unexpected_accel_mps2": 0.0,
  }

  result = classify(metrics, args)

  assert result.is_harsh is False
  assert result.flags == []
  assert result.distance_gate_source == "lead_hold"
  assert result.pred_lead_distance_hold_m == 3.0


def test_classify_flags_out_of_band_lead_hold_gap() -> None:
  args = SimpleNamespace(
    max_pred_end_jerk=0.70,
    max_pred_end_cmd_jerk=3.0,
    max_pred_end_accel_step=0.08,
    min_pred_a_floor=-1.10,
    max_pred_rollout_m=2.0,
    min_pred_lead_hold_distance_m=2.0,
    max_pred_lead_hold_distance_m=4.0,
    max_pred_speed_rebound_while_should_stop=0.08,
    max_pred_should_stop_unexpected_accel=0.10,
  )
  centered = classify(
    {
      "pred_end_stop_jerk_mps3": 0.42,
      "pred_end_stop_cmd_jerk_mps3": 2.5,
      "pred_end_stop_accel_step_mps2": 0.05,
      "pred_min_a_ego_mps2": -0.95,
      "pred_rollout_from_2mps_m": 0.8,
      "pred_lead_distance_hold_m": 3.0,
      "pred_speed_rebound_while_should_stop_mps": 0.0,
      "pred_should_stop_unexpected_accel_mps2": 0.0,
    },
    args,
  )
  long_gap = classify(
    {
      "pred_end_stop_jerk_mps3": 0.42,
      "pred_end_stop_cmd_jerk_mps3": 2.5,
      "pred_end_stop_accel_step_mps2": 0.05,
      "pred_min_a_ego_mps2": -0.95,
      "pred_rollout_from_2mps_m": 0.8,
      "pred_lead_distance_hold_m": 4.6,
      "pred_speed_rebound_while_should_stop_mps": 0.0,
      "pred_should_stop_unexpected_accel_mps2": 0.0,
    },
    args,
  )

  assert long_gap.is_harsh is True
  assert long_gap.flags == ["pred_lead_distance_hold_long"]
  assert centered.event_score < long_gap.event_score


def test_inverse_and_legacy_replay_emit_lead_distance_metrics() -> None:
  samples = build_entry_and_lead_metric_samples()
  model = simple_model()

  inverse_result = simulate_event_with_inverse_v3_controller(
    samples=samples,
    start_idx=0,
    hold_idx=len(samples) - 1,
    model=model,
    stop_accel=-2.0,
    stopping_speed_breakpoint=0.4,
    tau_s=1.12,
    max_ref_decel=1.46,
    hold_cmd_cap=-0.23,
    hold_cmd_speed=0.05,
    risk_hold_cmd_cap=-0.59,
    dropout_hold_cmd_cap=-0.78,
    extra_decel_scale=0.02,
    rollout_floor_scale=0.60,
    kp=0.12,
    ki=0.03,
    step_scale=0.71,
    brake_step_scale=0.45,
    release_step_scale=1.14,
  )
  legacy_result = simulate_event_with_legacy_controller(
    samples=samples,
    start_idx=0,
    hold_idx=len(samples) - 1,
    model=model,
    stop_accel=-2.0,
    stopping_speed_breakpoint=0.4,
    stopping_error_factor=1.3,
  )

  assert inverse_result["pred_lead_distance_stop_entry_m"] is not None
  assert legacy_result["pred_lead_distance_stop_entry_m"] is not None
  assert "pred_lead_distance_hold_m" in inverse_result
  assert "pred_lead_distance_hold_m" in legacy_result
