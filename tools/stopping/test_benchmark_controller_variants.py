from __future__ import annotations

from dataclasses import dataclass
from types import SimpleNamespace

import pytest

from openpilot.tools.stopping.benchmark_controller_variants import (
  aggregate_horizon_teacher,
  classify,
  iter_route_summaries,
  parse_args,
  summarize_horizon_teacher,
  simulate_event_with_legacy_controller,
)
from openpilot.tools.stopping.check_harsh_stops_model import simulate_event_with_controller
from openpilot.tools.stopping.horizon_optimizer import HorizonOptimizerConfig, simulate_event_with_horizon_v1_controller
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


def speed_band_model() -> FittedStoppingModel:
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
    rmse=0.08,
    mae=0.06,
    r2=0.92,
    sample_count=300,
    dt_s=0.05,
    relief_cmd_threshold=-0.25,
    low_speed_ref=1.2,
    model_kind="speed_band_linear",
    speed_split_mps=0.45,
    band_coefficients={
      "low": {
        "intercept": -0.04,
        "a_ego_prev": 0.94,
        "accel_cmd_delayed": 0.34,
        "v_ego": -0.10,
        "relief": 0.18,
        "low_speed": -0.06,
        "cmd_x_low_speed": 0.12,
      },
      "high": {
        "intercept": 0.0,
        "a_ego_prev": 0.88,
        "accel_cmd_delayed": 0.12,
        "v_ego": -0.06,
        "relief": 0.15,
        "low_speed": -0.03,
        "cmd_x_low_speed": 0.08,
      },
    },
    band_sample_counts={"low": 140, "high": 160},
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


def build_short_standstill_samples() -> list[FakeSample]:
  return [
    FakeSample(t=0.0, v_ego=0.08, a_ego=-0.20, accel_cmd=-0.34, should_stop=True),
    FakeSample(t=0.1, v_ego=0.05, a_ego=-0.17, accel_cmd=-0.33, should_stop=True),
    FakeSample(t=0.2, v_ego=0.03, a_ego=-0.15, accel_cmd=-0.32, should_stop=True),
    FakeSample(t=0.3, v_ego=0.01, a_ego=-0.13, accel_cmd=-0.31, should_stop=True),
  ]


def test_classify_uses_lead_hold_band_instead_of_rollout_when_present() -> None:
  args = SimpleNamespace(
    max_pred_end_jerk=0.70,
    max_pred_end_cmd_jerk=3.0,
    max_pred_end_accel_step=0.08,
    min_pred_a_floor=-1.10,
    max_pred_rollout_m=2.0,
    min_pred_lead_hold_distance_m=1.5,
    max_pred_lead_hold_distance_m=3.0,
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
    "recorded_lead_distance_hold_m": 3.2,
    "pred_speed_rebound_while_should_stop_mps": 0.0,
    "pred_should_stop_unexpected_accel_mps2": 0.0,
  }

  result = classify(metrics, args)

  assert result.is_harsh is False
  assert result.flags == []
  assert result.distance_gate_source == "lead_hold"
  assert result.pred_lead_distance_hold_m == 3.0
  assert result.recorded_lead_distance_hold_m == 3.2


def test_classify_flags_out_of_band_lead_hold_gap() -> None:
  args = SimpleNamespace(
    max_pred_end_jerk=0.70,
    max_pred_end_cmd_jerk=3.0,
    max_pred_end_accel_step=0.08,
    min_pred_a_floor=-1.10,
    max_pred_rollout_m=2.0,
    min_pred_lead_hold_distance_m=1.5,
    max_pred_lead_hold_distance_m=3.0,
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
      "pred_lead_distance_hold_m": 2.25,
      "recorded_lead_distance_hold_m": 2.25,
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
      "pred_lead_distance_hold_m": 3.6,
      "recorded_lead_distance_hold_m": 2.9,
      "pred_speed_rebound_while_should_stop_mps": 0.0,
      "pred_should_stop_unexpected_accel_mps2": 0.0,
    },
    args,
  )

  assert long_gap.is_harsh is True
  assert long_gap.flags == ["pred_lead_distance_hold_long"]
  assert centered.event_score < long_gap.event_score


def test_classify_ignores_far_non_actionable_lead_gap() -> None:
  args = SimpleNamespace(
    max_pred_end_jerk=0.70,
    max_pred_end_cmd_jerk=3.0,
    max_pred_end_accel_step=0.08,
    min_pred_a_floor=-1.10,
    max_pred_rollout_m=2.0,
    min_pred_lead_hold_distance_m=2.5,
    max_pred_lead_hold_distance_m=5.0,
    max_pred_speed_rebound_while_should_stop=0.08,
    max_pred_should_stop_unexpected_accel=0.10,
    absolute_pred_lead_hold_distance=True,
  )

  result = classify(
    {
      "pred_end_stop_jerk_mps3": 0.20,
      "pred_end_stop_cmd_jerk_mps3": 0.50,
      "pred_end_stop_accel_step_mps2": 0.03,
      "pred_min_a_ego_mps2": -0.65,
      "pred_rollout_from_2mps_m": 0.7,
      "pred_lead_distance_stop_entry_m": 15.0,
      "pred_lead_distance_hold_m": 26.0,
      "recorded_lead_distance_hold_m": 26.0,
      "pred_speed_rebound_while_should_stop_mps": 0.0,
      "pred_should_stop_unexpected_accel_mps2": 0.0,
    },
    args,
  )

  assert result.distance_gate_source == "rollout_2mps"
  assert result.flags == []


def test_classify_assigns_comfort_quality_buckets() -> None:
  args = SimpleNamespace(
    max_pred_end_jerk=0.70,
    max_pred_end_cmd_jerk=3.0,
    max_pred_end_accel_step=0.08,
    min_pred_a_floor=-1.10,
    max_pred_rollout_m=2.0,
    min_pred_lead_hold_distance_m=2.5,
    max_pred_lead_hold_distance_m=5.0,
    max_pred_speed_rebound_while_should_stop=0.08,
    max_pred_should_stop_unexpected_accel=0.10,
  )
  perfect = classify(
    {
      "pred_end_stop_jerk_mps3": 0.10,
      "pred_end_stop_cmd_jerk_mps3": 0.50,
      "pred_end_stop_accel_step_mps2": 0.02,
      "pred_min_a_ego_mps2": -0.65,
      "pred_rollout_from_2mps_m": 0.8,
      "pred_lead_distance_hold_m": 3.70,
      "recorded_lead_distance_hold_m": 3.70,
      "pred_speed_rebound_while_should_stop_mps": 0.0,
      "pred_should_stop_unexpected_accel_mps2": 0.0,
    },
    args,
  )
  mediocre = classify(
    {
      "pred_end_stop_jerk_mps3": 0.32,
      "pred_end_stop_cmd_jerk_mps3": 1.20,
      "pred_end_stop_accel_step_mps2": 0.05,
      "pred_min_a_ego_mps2": -0.86,
      "pred_rollout_from_2mps_m": 0.8,
      "pred_lead_distance_hold_m": 3.70,
      "recorded_lead_distance_hold_m": 3.70,
      "pred_speed_rebound_while_should_stop_mps": 0.0,
      "pred_should_stop_unexpected_accel_mps2": 0.0,
    },
    args,
  )
  hard_fail = classify(
    {
      "pred_end_stop_jerk_mps3": 0.20,
      "pred_end_stop_cmd_jerk_mps3": 0.50,
      "pred_end_stop_accel_step_mps2": 0.03,
      "pred_min_a_ego_mps2": -1.20,
      "pred_rollout_from_2mps_m": 0.8,
      "pred_lead_distance_hold_m": 3.70,
      "recorded_lead_distance_hold_m": 3.70,
      "pred_speed_rebound_while_should_stop_mps": 0.0,
      "pred_should_stop_unexpected_accel_mps2": 0.0,
    },
    args,
  )

  assert perfect.quality_bucket == "perfect"
  assert mediocre.quality_bucket == "mediocre"
  assert hard_fail.quality_bucket == "hard_fail"


def test_classify_does_not_flag_long_gap_when_recorded_stop_was_already_wide() -> None:
  args = SimpleNamespace(
    max_pred_end_jerk=0.70,
    max_pred_end_cmd_jerk=3.0,
    max_pred_end_accel_step=0.08,
    min_pred_a_floor=-1.10,
    max_pred_rollout_m=2.0,
    min_pred_lead_hold_distance_m=1.5,
    max_pred_lead_hold_distance_m=3.0,
    max_pred_speed_rebound_while_should_stop=0.08,
    max_pred_should_stop_unexpected_accel=0.10,
  )
  relaxed = classify(
    {
      "pred_end_stop_jerk_mps3": 0.42,
      "pred_end_stop_cmd_jerk_mps3": 2.5,
      "pred_end_stop_accel_step_mps2": 0.05,
      "pred_min_a_ego_mps2": -0.95,
      "pred_rollout_from_2mps_m": 0.8,
      "pred_lead_distance_hold_m": 3.6,
      "recorded_lead_distance_hold_m": 3.7,
      "pred_speed_rebound_while_should_stop_mps": 0.0,
      "pred_should_stop_unexpected_accel_mps2": 0.0,
    },
    args,
  )

  assert relaxed.is_harsh is False
  assert relaxed.flags == []


def test_horizon_and_legacy_replay_emit_lead_distance_metrics() -> None:
  samples = build_entry_and_lead_metric_samples()
  model = simple_model()

  current_result = simulate_event_with_controller(
    samples=samples,
    start_idx=0,
    hold_idx=len(samples) - 1,
    model=model,
    stop_accel=-2.0,
    stopping_speed_breakpoint=0.4,
    return_trace=True,
  )
  horizon_result = simulate_event_with_horizon_v1_controller(
    samples=samples,
    current_replay=current_result,
    model=model,
    config=HorizonOptimizerConfig(
      horizon_s=0.6,
      block_s=0.10,
      beam_width=8,
      residual_grid_mps2=(-0.06, 0.0, 0.06),
    ),
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

  assert horizon_result["pred_lead_distance_stop_entry_m"] is not None
  assert legacy_result["pred_lead_distance_stop_entry_m"] is not None
  assert "pred_lead_distance_hold_m" in horizon_result
  assert "pred_lead_distance_hold_m" in legacy_result
  assert "recorded_lead_distance_hold_m" in horizon_result
  assert "recorded_lead_distance_hold_m" in legacy_result


def test_horizon_replay_applies_same_standstill_cmd_jerk_penalty_as_current() -> None:
  samples = build_short_standstill_samples()
  model = simple_model()

  current_result = simulate_event_with_controller(
    samples=samples,
    start_idx=0,
    hold_idx=len(samples) - 1,
    model=model,
    stop_accel=-2.0,
    stopping_speed_breakpoint=0.4,
    return_trace=True,
  )
  horizon_result = simulate_event_with_horizon_v1_controller(
    samples=samples,
    current_replay=current_result,
    model=model,
    config=HorizonOptimizerConfig(
      horizon_s=0.4,
      block_s=0.10,
      beam_width=1,
      residual_grid_mps2=(0.0,),
    ),
  )

  assert horizon_result["pred_end_stop_cmd_jerk_mps3"] == pytest.approx(current_result["pred_end_stop_cmd_jerk_mps3"])
  assert horizon_result["pred_end_stop_jerk_mps3"] == pytest.approx(current_result["pred_end_stop_jerk_mps3"])


def test_speed_band_model_uses_low_band_coefficients_below_split() -> None:
  baseline_model = simple_model()
  banded_model = speed_band_model()

  baseline_low_speed = baseline_model.effective_coefficients(0.20)
  banded_low_speed = banded_model.effective_coefficients(0.20)
  baseline_high_speed = baseline_model.effective_coefficients(0.80)
  banded_high_speed = banded_model.effective_coefficients(0.80)

  assert banded_low_speed["accel_cmd_delayed"] != pytest.approx(baseline_low_speed["accel_cmd_delayed"])
  assert banded_low_speed["a_ego_prev"] != pytest.approx(baseline_low_speed["a_ego_prev"])
  assert banded_high_speed == pytest.approx(baseline_high_speed)


def test_horizon_teacher_summary_classifies_deepen_then_soften_and_current_triggers() -> None:
  current = {
    "trace": {
      "output_trace": [-0.30, -0.34, -0.38, -0.42, -0.44, -0.46, -0.48],
      "debug_trace": [
        {"triggers": ["rollout_push"], "phase": 1},
        {"triggers": ["rollout_push"], "phase": 1},
        {"triggers": ["end_stop_cap_active"], "phase": 1},
        {"triggers": ["end_stop_cap_active"], "phase": 2},
        {"triggers": ["soft_landing_release"], "phase": 2},
        {"triggers": ["soft_landing_release"], "phase": 2},
      ],
    },
  }
  horizon_v1 = {
    "optimizer_search_start_step": 0,
    "trace": {
      "output_trace": [-0.30, -0.39, -0.45, -0.46, -0.39, -0.34, -0.32],
    },
  }

  teacher = summarize_horizon_teacher(current, horizon_v1, current_score=2.0, horizon_score=1.2)

  assert teacher["status"] == "ok"
  assert teacher["intent"] == "deepen_then_soften"
  assert teacher["score_delta"] == pytest.approx(-0.8)
  assert teacher["first_third_avg_cmd_delta_mps2"] < 0.0
  assert teacher["final_third_avg_cmd_delta_mps2"] > 0.0
  assert teacher["top_current_triggers"][0] == {"name": "rollout_push", "count": 2}
  assert teacher["top_current_phases"][0] == {"name": "1", "count": 3}


def test_horizon_teacher_summary_handles_missing_trace() -> None:
  teacher = summarize_horizon_teacher({}, {}, current_score=1.0, horizon_score=0.5)

  assert teacher == {
    "status": "missing_trace",
    "score_delta": -0.5,
  }


def test_aggregate_horizon_teacher_groups_intents_and_trigger_owners() -> None:
  rows = [
    {
      "horizon_teacher": {
        "status": "ok",
        "intent": "tail_soften",
        "score_delta": -0.4,
        "top_current_triggers": [{"name": "end_stop_cap_active", "count": 3}],
      },
    },
    {
      "horizon_teacher": {
        "status": "ok",
        "intent": "tail_soften",
        "score_delta": -0.2,
        "top_current_triggers": [{"name": "end_stop_cap_active", "count": 2}],
      },
    },
    {
      "horizon_teacher": {
        "status": "ok",
        "intent": "deepen",
        "score_delta": 0.1,
        "top_current_triggers": [{"name": "rollout_push", "count": 1}],
      },
    },
    {"horizon_teacher": {"status": "missing_trace", "score_delta": -0.1}},
  ]

  summary = aggregate_horizon_teacher(rows)

  assert summary["intent_counts"] == {"deepen": 1, "tail_soften": 2}
  assert summary["improved_intent_counts"] == {"tail_soften": 2}
  assert summary["worsened_intent_counts"] == {"deepen": 1}
  assert summary["avg_score_delta_by_intent"]["tail_soften"] == pytest.approx(-0.3)
  assert summary["top_improved_current_triggers"] == [{"name": "end_stop_cap_active", "count": 5}]
  assert summary["top_worsened_current_triggers"] == [{"name": "rollout_push", "count": 1}]
  assert summary["missing_trace_events"] == 1


def test_iter_route_summaries_expands_corpus_summary(tmp_path) -> None:
  summary_path = tmp_path / "corpus.json"
  summary_path.write_text(
    """
{
  "host": "comma",
  "routes": [
    {"route": "route-a", "events": [{"event_id": 1}]},
    {"route": "route-b", "host": "commawifi", "events": [{"event_id": 2}]}
  ]
}
""".strip()
    + "\n",
  )

  summaries = list(iter_route_summaries(summary_path))

  assert summaries == [
    {"route": "route-a", "host": "comma", "events": [{"event_id": 1}]},
    {"route": "route-b", "host": "commawifi", "events": [{"event_id": 2}]},
  ]


def test_parse_args_defaults_to_recorded_controller_should_stop(monkeypatch) -> None:
  monkeypatch.setattr("sys.argv", [
    "benchmark_controller_variants.py",
    "--model-json",
    "model.json",
    "--summary-json",
    "summary.json",
  ])

  args = parse_args()

  assert args.controller_should_stop_source == "recorded"
