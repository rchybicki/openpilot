from __future__ import annotations

import json
from dataclasses import dataclass

import pytest

from openpilot.selfdrive.controls.lib.stopping_profile_selector import (
  PROFILE_GLIDE_SOFTEN,
  PROFILE_NO_CHANGE,
  PROFILE_PRESERVE_BRAKE,
  PROFILE_TAIL_DEEPEN,
  PrototypeStoppingProfileSelector,
  StoppingProfileFeatures,
  nearest_exemplar_distance_for_profile,
  profile_label_from_horizon_teacher,
  residual_template_for_profile,
)
from openpilot.tools.stopping.benchmark_controller_variants import selector_features_from_replay, selector_residual_target_from_replay
from openpilot.tools.stopping.train_profile_selector import SelectorTrainingRow, build_selector_model, evaluate_selector_model, load_training_rows


@dataclass
class FakeSample:
  lead_status: bool = False
  lead_d_rel_m: float | None = None
  lead_v: float = 0.0


def _features(
  *,
  v_ego_mps: float,
  a_ego_mps2: float,
  last_output_accel_mps2: float,
  rollout_m: float,
  remaining_m: float | None,
) -> dict[str, float]:
  return StoppingProfileFeatures(
    v_ego_mps=v_ego_mps,
    a_ego_mps2=a_ego_mps2,
    last_output_accel_mps2=last_output_accel_mps2,
    rollout_m=rollout_m,
    remaining_m=remaining_m,
    lead_d_rel_m=None,
    lead_v_mps=0.0,
    phase=1,
    release_lock_active=False,
    rebound_arrest_active=False,
    explicit_target_available=remaining_m is not None,
    should_stop=True,
  ).as_dict()


def test_profile_label_from_horizon_teacher_requires_real_improvement() -> None:
  assert profile_label_from_horizon_teacher({"status": "missing_trace"}) == PROFILE_NO_CHANGE
  assert profile_label_from_horizon_teacher({"status": "ok", "intent": "deepen", "score_delta": -0.001}) == PROFILE_NO_CHANGE
  assert profile_label_from_horizon_teacher({"status": "ok", "intent": "deepen_then_soften", "score_delta": -0.20}) == PROFILE_PRESERVE_BRAKE
  assert profile_label_from_horizon_teacher({"status": "ok", "intent": "tail_deepen", "score_delta": -0.20}) == PROFILE_TAIL_DEEPEN
  assert profile_label_from_horizon_teacher({"status": "ok", "intent": "tail_soften", "score_delta": -0.20}) == PROFILE_GLIDE_SOFTEN


def test_prototype_selector_trains_and_selects_profile_from_rows() -> None:
  rows = [
    SelectorTrainingRow(
      "bench.json", "route-a", "1", PROFILE_PRESERVE_BRAKE,
      _features(v_ego_mps=0.55, a_ego_mps2=-0.40, last_output_accel_mps2=-0.55, rollout_m=0.60, remaining_m=None),
      [-0.04, 0.02, 0.08],
    ),
    SelectorTrainingRow(
      "bench.json", "route-a", "2", PROFILE_PRESERVE_BRAKE,
      _features(v_ego_mps=0.60, a_ego_mps2=-0.42, last_output_accel_mps2=-0.58, rollout_m=0.64, remaining_m=None),
      [-0.02, 0.04, 0.10],
    ),
    SelectorTrainingRow(
      "bench.json", "route-b", "1", PROFILE_TAIL_DEEPEN,
      _features(v_ego_mps=0.18, a_ego_mps2=-0.08, last_output_accel_mps2=-0.22, rollout_m=1.30, remaining_m=0.10),
    ),
    SelectorTrainingRow(
      "bench.json", "route-b", "2", PROFILE_TAIL_DEEPEN,
      _features(v_ego_mps=0.20, a_ego_mps2=-0.06, last_output_accel_mps2=-0.25, rollout_m=1.36, remaining_m=0.12),
    ),
  ]
  model = build_selector_model(rows, min_profile_count=1, source_paths=[])
  selector = PrototypeStoppingProfileSelector.from_json(model)

  preserve_decision = selector.select(StoppingProfileFeatures.from_mapping(
    _features(v_ego_mps=0.58, a_ego_mps2=-0.41, last_output_accel_mps2=-0.56, rollout_m=0.62, remaining_m=None),
  ))
  tail_decision = selector.select(StoppingProfileFeatures.from_mapping(
    _features(v_ego_mps=0.19, a_ego_mps2=-0.07, last_output_accel_mps2=-0.24, rollout_m=1.34, remaining_m=0.11),
  ))

  assert preserve_decision.profile == PROFILE_PRESERVE_BRAKE
  assert tail_decision.profile == PROFILE_TAIL_DEEPEN
  assert residual_template_for_profile(model, PROFILE_PRESERVE_BRAKE) == pytest.approx([-0.03, 0.03, 0.09])
  assert nearest_exemplar_distance_for_profile(
    model,
    PROFILE_PRESERVE_BRAKE,
    _features(v_ego_mps=0.58, a_ego_mps2=-0.41, last_output_accel_mps2=-0.56, rollout_m=0.62, remaining_m=None),
  ) is not None
  assert evaluate_selector_model(rows, model)["accuracy"] == pytest.approx(1.0)


def test_knn_selector_selects_from_local_exemplars() -> None:
  rows = [
    SelectorTrainingRow(
      "bench.json", "route-a", "1", PROFILE_NO_CHANGE,
      _features(v_ego_mps=1.30, a_ego_mps2=-0.05, last_output_accel_mps2=-0.10, rollout_m=2.00, remaining_m=3.0),
      [0.0] * 12,
    ),
    SelectorTrainingRow(
      "bench.json", "route-a", "2", PROFILE_NO_CHANGE,
      _features(v_ego_mps=1.35, a_ego_mps2=-0.04, last_output_accel_mps2=-0.10, rollout_m=2.10, remaining_m=3.1),
      [0.0] * 12,
    ),
    SelectorTrainingRow(
      "bench.json", "route-b", "1", PROFILE_GLIDE_SOFTEN,
      _features(v_ego_mps=0.02, a_ego_mps2=0.0, last_output_accel_mps2=-0.12, rollout_m=0.0, remaining_m=0.8),
      [0.08] * 12,
    ),
  ]
  model = build_selector_model(rows, min_profile_count=1, selector_kind="knn", knn_k=1, source_paths=[])
  selector = PrototypeStoppingProfileSelector.from_json(model)

  decision = selector.select(StoppingProfileFeatures.from_mapping(
    _features(v_ego_mps=0.03, a_ego_mps2=0.0, last_output_accel_mps2=-0.12, rollout_m=0.0, remaining_m=0.82),
  ))

  assert decision.profile == PROFILE_GLIDE_SOFTEN
  assert decision.confidence > 0.9


def test_selector_features_from_replay_uses_horizon_search_start_snapshot() -> None:
  samples = [
    FakeSample(),
    FakeSample(lead_status=True, lead_d_rel_m=5.4, lead_v=-0.1),
    FakeSample(lead_status=True, lead_d_rel_m=5.2, lead_v=-0.2),
  ]
  current = {
    "trace": {
      "output_trace": [-0.30, -0.42, -0.50],
      "predicted_a": [-0.20, -0.35, -0.48],
      "predicted_v": [0.70, 0.55, 0.40],
      "replay_sample_indices": [0, 1, 2],
      "should_stop_trace": [True, True, True],
      "debug_trace": [
        {"phase": 1, "rollout_m": 0.20, "remaining_m": 1.8, "distance_to_stop_target_m": 1.8},
        {"phase": 1, "rollout_m": 0.55, "remaining_m": 1.2, "distance_to_stop_target_m": 1.2, "release_lock_active": True},
      ],
    },
  }
  horizon_v1 = {"optimizer_search_start_step": 2}

  features = selector_features_from_replay(samples, current, horizon_v1)

  assert features["v_ego_mps"] == pytest.approx(0.40)
  assert features["a_ego_mps2"] == pytest.approx(-0.48)
  assert features["last_output_accel_mps2"] == pytest.approx(-0.50)
  assert features["rollout_m"] == pytest.approx(0.55)
  assert features["lead_d_rel_m"] == pytest.approx(5.2)
  assert features["lead_v_mps"] == pytest.approx(-0.2)
  assert features["release_lock_active"] == pytest.approx(1.0)


def test_selector_residual_target_from_replay_averages_horizon_delta_shape() -> None:
  current = {"trace": {"output_trace": [-0.30, -0.40, -0.50, -0.60, -0.70]}}
  horizon_v1 = {
    "optimizer_search_start_step": 1,
    "trace": {"output_trace": [-0.30, -0.44, -0.56, -0.54, -0.58]},
  }

  target = selector_residual_target_from_replay(current, horizon_v1, block_count=3)

  assert target["residual_template_mps2"] == pytest.approx([-0.06, 0.06, 0.12])
  assert target["max_deepen_mps2"] == pytest.approx(0.06)
  assert target["max_soften_mps2"] == pytest.approx(0.12)


def test_load_training_rows_applies_route_filters(tmp_path) -> None:
  benchmark_path = tmp_path / "benchmark.json"
  row_template = {
    "selector_features": _features(v_ego_mps=0.58, a_ego_mps2=-0.41, last_output_accel_mps2=-0.56, rollout_m=0.62, remaining_m=None),
    "horizon_teacher": {"status": "ok", "intent": "deepen_then_soften", "score_delta": -0.20},
    "selector_residual_target": {"residual_template_mps2": [-0.02, 0.03, 0.05]},
  }
  benchmark_path.write_text(
    '{"event_rows": ['
    + ",".join([
      json.dumps({**row_template, "route": "route-a", "event_id": "1"}),
      json.dumps({**row_template, "route": "route-b", "event_id": "2"}),
      json.dumps({**row_template, "route": "route-c", "event_id": "3"}),
    ])
    + "]}\n",
  )

  rows, skipped = load_training_rows(
    [benchmark_path],
    min_improvement=0.01,
    include_routes={"route-a", "route-b"},
    exclude_routes={"route-b"},
  )

  assert [row.route for row in rows] == ["route-a"]
  assert skipped == {"route_excluded": 1, "route_not_in_include": 1}
