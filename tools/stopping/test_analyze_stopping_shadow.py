from __future__ import annotations

import json

import pytest

from openpilot.tools.stopping.analyze_stopping_shadow import (
  ShadowDecision,
  event_segments,
  parse_shadow_log_message,
  route_shadow_verdict,
  shadow_decisions_for_event,
  summarize_event_shadow,
)


def _decision(
  *,
  mono_time_s: float = 100.0,
  observer_scope: str = "stopping",
  profile: str = "glide_soften",
  reason: str = "accepted",
  score_delta: float = -0.05,
  first_output_accel: float = -0.42,
  actual_output_accel: float = -0.50,
  selected_harsh: bool = False,
  selected_leapfrog: bool = False,
) -> ShadowDecision:
  return ShadowDecision(
    mono_time_s=mono_time_s,
    segment=2,
    observer_scope=observer_scope,
    profile=profile,
    reason=reason,
    score_delta=score_delta,
    confidence=0.12,
    first_output_accel=first_output_accel,
    actual_output_accel=actual_output_accel,
    v_ego=0.30,
    a_ego=-0.20,
    remaining_m=0.20,
    rollout_m=1.20,
    lead_status=False,
    lead_d_rel=None,
    selected_rollout_m=1.35,
    selected_min_a_ego=-0.25,
    selected_final_v_ego=0.02,
    selected_harsh=selected_harsh,
    selected_leapfrog=selected_leapfrog,
    commit="abc123",
    created=1234.5,
  )


def test_parse_shadow_log_message_extracts_runtime_decision() -> None:
  raw = json.dumps({
    "msg": {
      "event": "stopping_shadow",
      "observer_scope": "pid_stop_intent",
      "profile": "glide_soften",
      "reason": "accepted",
      "score_delta": -0.04,
      "confidence": 0.09,
      "first_output_accel": -0.43,
      "actual_output_accel": -0.49,
      "selected_harsh": False,
      "selected_leapfrog": False,
    },
    "ctx": {"commit": "abc123"},
    "created": 123.0,
  })

  decision = parse_shadow_log_message(raw, mono_time_s=10.0, segment=4)

  assert decision is not None
  assert decision.profile == "glide_soften"
  assert decision.observer_scope == "pid_stop_intent"
  assert decision.accepted_safe is True
  assert decision.command_relief_mps2 == pytest.approx(0.06)
  assert decision.commit == "abc123"


def test_shadow_decisions_for_event_uses_absolute_mono_window() -> None:
  event = {"start_time_s": 10.0, "stop_hold_time_s": 12.0}
  decisions = [
    _decision(mono_time_s=109.4),
    _decision(mono_time_s=109.6),
    _decision(mono_time_s=112.9),
    _decision(mono_time_s=113.2),
  ]

  matched = shadow_decisions_for_event(event, decisions, first_mono_time_s=100.0, pre_window_s=0.5, post_window_s=1.0)

  assert [item.mono_time_s for item in matched] == [109.6, 112.9]


def test_summarize_event_shadow_marks_actionable_harsh_relief() -> None:
  event = {
    "event_id": 7,
    "start_segment": 2,
    "start_time_s": 10.0,
    "min_a_ego_mps2": -1.8,
    "hard_decel_duration_s": 0.2,
  }

  summary = summarize_event_shadow(event, [_decision()], first_mono_time_s=100.0, min_command_relief_mps2=0.03)

  assert summary.actual_harsh is True
  assert summary.observer_scope_counts == {"stopping": 1}
  assert summary.accepted_relief_count == 1
  assert summary.verdict == "actionable_soften_candidate"
  assert route_shadow_verdict([summary]) == "valuable_promote_narrow_candidate"


def test_summarize_event_shadow_keeps_unsafe_prediction_separate() -> None:
  event = {
    "event_id": 8,
    "start_segment": 2,
    "start_time_s": 10.0,
    "min_a_ego_mps2": -1.8,
    "hard_decel_duration_s": 0.2,
  }

  summary = summarize_event_shadow(
    event,
    [_decision(), _decision(profile="soften_then_deepen", selected_leapfrog=True)],
    first_mono_time_s=100.0,
    min_command_relief_mps2=0.03,
  )

  assert summary.accepted_relief_count == 1
  assert summary.accepted_unsafe_count == 1
  assert summary.verdict == "mixed_shadow_signal"
  assert route_shadow_verdict([summary]) == "valuable_but_needs_stronger_guards"


def test_summarize_event_shadow_marks_harsh_events_without_shadow_data() -> None:
  event = {
    "event_id": 9,
    "start_segment": 24,
    "start_time_s": 10.0,
    "min_a_ego_mps2": -1.9,
    "hard_decel_duration_s": 0.3,
  }

  summary = summarize_event_shadow(event, [], first_mono_time_s=100.0, min_command_relief_mps2=0.03)

  assert summary.verdict == "missing_harsh_shadow_data"
  assert route_shadow_verdict([summary]) == "not_usable_no_shadow_data"


def test_event_segments_includes_short_cross_segment_stops() -> None:
  events = [
    {"start_segment": 2, "stop_segment": 2},
    {"start_segment": 4, "stop_segment": 6},
  ]

  assert event_segments(events) == [2, 4, 5, 6]
