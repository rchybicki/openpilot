from __future__ import annotations

import pytest

from openpilot.tools.stopping.analyze_stopping_behavior import (
  Sample,
  compute_event,
  compute_transition_sharpness_metrics,
)


def _sample(
  t: float,
  *,
  v_ego: float,
  a_ego: float,
  long_state: str = "pid",
  long_state_cmd: str = "pid",
  should_stop: bool = False,
  accel_cmd: float | None = None,
  standstill: bool = False,
  lead_status: bool = False,
  lead_d_rel_m: float | None = None,
) -> Sample:
  return Sample(
    t=t,
    segment=0,
    v_ego=v_ego,
    v_wheel_avg=v_ego,
    a_ego=a_ego,
    standstill=standstill,
    brake_pressed=False,
    gas_pressed=False,
    enabled=True,
    long_state=long_state,
    long_state_cmd=long_state_cmd,
    should_stop=should_stop,
    a_target=None,
    accel_cmd=accel_cmd,
    lead_status=lead_status,
    lead_d_rel_m=lead_d_rel_m,
    forcing_stop=False,
    red_light=False,
  )


def test_compute_transition_sharpness_metrics_uses_anchor_window():
  samples = [
    _sample(0.0, v_ego=1.20, a_ego=-0.05, accel_cmd=-0.05),
    _sample(0.1, v_ego=1.05, a_ego=-0.06, accel_cmd=-0.06),
    _sample(0.2, v_ego=0.90, a_ego=-0.10, accel_cmd=-0.10, should_stop=True, long_state="stopping", long_state_cmd="stopping"),
    _sample(0.3, v_ego=0.70, a_ego=-0.18, accel_cmd=-0.20, should_stop=True, long_state="stopping", long_state_cmd="stopping"),
    _sample(0.4, v_ego=0.50, a_ego=-0.22, accel_cmd=-0.22, should_stop=True, long_state="stopping", long_state_cmd="stopping"),
    _sample(0.5, v_ego=0.30, a_ego=-0.25, accel_cmd=-0.24, should_stop=True, long_state="stopping", long_state_cmd="stopping", standstill=True),
  ]

  accel_jerk, accel_step, cmd_jerk, cmd_step = compute_transition_sharpness_metrics(
    samples=samples,
    start_idx=0,
    end_idx=5,
    anchor_time_s=0.2,
  )

  assert accel_jerk == pytest.approx(0.8, abs=1e-6)
  assert accel_step == pytest.approx(0.085, abs=1e-6)
  assert cmd_jerk == pytest.approx(1.0, abs=1e-6)
  assert cmd_step == pytest.approx(0.095, abs=1e-6)


def test_compute_event_falls_back_to_should_stop_for_entry_metrics():
  samples = [
    _sample(0.0, v_ego=0.80, a_ego=-0.04, accel_cmd=-0.04),
    _sample(0.1, v_ego=0.60, a_ego=-0.05, accel_cmd=-0.05, should_stop=True),
    _sample(0.2, v_ego=0.40, a_ego=-0.12, accel_cmd=-0.14, should_stop=True),
    _sample(0.3, v_ego=0.20, a_ego=-0.20, accel_cmd=-0.18, should_stop=True),
    _sample(0.4, v_ego=0.04, a_ego=-0.18, accel_cmd=-0.16, should_stop=True, standstill=True),
    _sample(0.5, v_ego=0.00, a_ego=-0.16, accel_cmd=-0.15, should_stop=True, standstill=True),
  ]

  event = compute_event(
    event_id=1,
    event_source="speed",
    samples=samples,
    start_idx=0,
    stop_idx=4,
    hold_idx=4,
    approach_speed=1.2,
    graph_file="plot.html",
  )

  assert event.should_stop_to_stopping_s is None
  assert event.entry_stop_jerk_mps3 == pytest.approx(0.8, abs=1e-6)
  assert event.entry_stop_cmd_jerk_mps3 == pytest.approx(0.9, abs=1e-6)
  assert event.entry_stop_accel_step_mps2 == pytest.approx(0.045, abs=1e-6)
  assert event.entry_stop_cmd_step_mps2 == pytest.approx(0.055, abs=1e-6)


def test_compute_event_tracks_lead_distance_at_stop_entry_and_hold():
  samples = [
    _sample(0.0, v_ego=1.20, a_ego=-0.03, accel_cmd=-0.03, lead_status=True, lead_d_rel_m=16.0),
    _sample(0.1, v_ego=0.95, a_ego=-0.05, accel_cmd=-0.04, should_stop=True, lead_status=True, lead_d_rel_m=15.5),
    _sample(0.2, v_ego=0.70, a_ego=-0.08, accel_cmd=-0.08, should_stop=True, long_state="stopping", long_state_cmd="stopping", lead_status=True, lead_d_rel_m=15.0),
    _sample(0.3, v_ego=0.35, a_ego=-0.12, accel_cmd=-0.11, should_stop=True, long_state="stopping", long_state_cmd="stopping", lead_status=True, lead_d_rel_m=14.7),
    _sample(0.4, v_ego=0.03, a_ego=-0.10, accel_cmd=-0.10, should_stop=True, long_state="stopping", long_state_cmd="stopping", standstill=True, lead_status=True, lead_d_rel_m=5.2),
    _sample(0.5, v_ego=0.00, a_ego=-0.08, accel_cmd=-0.09, should_stop=True, long_state="stopping", long_state_cmd="stopping", standstill=True, lead_status=True, lead_d_rel_m=5.0),
    _sample(0.6, v_ego=0.00, a_ego=-0.08, accel_cmd=-0.09, should_stop=True, long_state="stopping", long_state_cmd="stopping", standstill=True, lead_status=True, lead_d_rel_m=4.9),
  ]

  event = compute_event(
    event_id=2,
    event_source="speed",
    samples=samples,
    start_idx=0,
    stop_idx=4,
    hold_idx=5,
    approach_speed=1.2,
    graph_file="plot.html",
  )

  assert event.lead_distance_stop_entry_m == pytest.approx(14.85, abs=1e-6)
  assert event.lead_distance_hold_m == pytest.approx(5.0, abs=1e-6)
