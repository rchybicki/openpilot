from types import SimpleNamespace

import pytest

from openpilot.selfdrive.controls.radard import (LaneChangeDirection, LaneChangeState, RadarD, SURROGATE_DREL_OFFSET, SURROGATE_MIN_V_EGO,
                                                 SURROGATE_PHASE_EXEC, SURROGATE_PHASE_OFF, SURROGATE_VLEAD_DELTA)


class StubSubMaster:
  def __init__(self):
    self.modelV2 = SimpleNamespace(meta=SimpleNamespace(laneChangeState=LaneChangeState.laneChangeStarting,
                                                        laneChangeDirection=LaneChangeDirection.left))
    self.seen = {"frogpilotPlan": True}

  def __getitem__(self, key):
    return getattr(self, key)


def make_radard(registered_tracks=None):
  rd = RadarD.__new__(RadarD)
  rd.frogpilot_toggles = SimpleNamespace(human_lane_changes=True, lane_detection_width=2.7)
  rd.ready = True
  rd.current_time = 0.0
  rd.v_ego = 35.0
  rd.surrogate_track_ids = set(registered_tracks or [])
  rd.target_lane_released_track_ids = set()
  rd.target_lane_released_leads = []
  rd.target_lane_crossing_counts = {}
  rd.main_untracked_active = False
  rd.main_untracked_sign = 0
  rd.surrogate_untracked_side_signs = set()
  rd.prev_lane_change_state = LaneChangeState.laneChangeStarting
  rd.lc_direction_sign = 1
  rd.center_surrogate_enabled = False
  rd.divider_lane_line_idx = -1
  rd.divider_initial_sign = 0
  rd.divider_crossed_counter = 0
  rd.divider_crossed = False
  rd.surrogate_phase = SURROGATE_PHASE_EXEC
  rd.surrogate_speed_gate_open = True
  return rd


def make_lead(**overrides):
  lead = {
    "status": True,
    "dRel": 68.0,
    "yRel": -0.8,
    "vRel": -11.0,
    "vLead": 24.0,
    "vLeadK": 24.0,
    "fcw": True,
    "modelProb": 0.99,
    "radarTrackId": 123,
    "radar": True,
  }
  lead.update(overrides)
  return lead


def test_registered_surrogate_survives_transient_exempt_yrel():
  rd = make_radard(registered_tracks={123})
  sm = StubSubMaster()
  lead = make_lead(yRel=0.4)

  rd._update_lane_change_surrogates(sm, lead)
  assert 123 in rd.surrogate_track_ids
  assert 123 not in rd.target_lane_released_track_ids

  new_lead, applied = rd._apply_overtake_surrogate(lead, sm)

  assert applied
  assert new_lead["dRel"] == pytest.approx(lead["dRel"] + SURROGATE_DREL_OFFSET)
  assert new_lead["vRel"] == pytest.approx(SURROGATE_VLEAD_DELTA)
  assert new_lead["vLead"] == pytest.approx(rd.v_ego + SURROGATE_VLEAD_DELTA)


def test_registered_surrogate_releases_when_lead_reaches_target_lane():
  rd = make_radard(registered_tracks={123})
  sm = StubSubMaster()
  lead = make_lead(yRel=0.8)

  rd._update_lane_change_surrogates(sm, lead)
  assert 123 not in rd.surrogate_track_ids
  assert 123 in rd.target_lane_released_track_ids

  new_lead, applied = rd._apply_overtake_surrogate(lead, sm)

  assert not applied
  assert new_lead == lead


def test_registered_surrogate_releases_when_lead_crosses_target_divider_with_ego():
  rd = make_radard(registered_tracks={123})
  rd.divider_lane_line_idx = 1
  sm = StubSubMaster()
  sm.modelV2.laneLines = [
    SimpleNamespace(x=[0.0, 60.0], y=[-4.5, -4.5]),
    SimpleNamespace(x=[0.0, 6.0, 30.0, 60.0], y=[-1.5, -1.5, 1.0, 1.0]),
    SimpleNamespace(x=[0.0, 60.0], y=[1.5, 1.5]),
  ]
  sm.modelV2.laneLineProbs = [0.9, 0.8, 0.9]
  lead = make_lead(dRel=30.0, yRel=-0.6)

  for _ in range(2):
    rd._update_lane_change_surrogates(sm, lead)
    assert 123 in rd.surrogate_track_ids
    _, applied = rd._apply_overtake_surrogate(lead, sm)
    assert applied

  rd._update_lane_change_surrogates(sm, lead)

  assert 123 not in rd.surrogate_track_ids
  assert 123 in rd.target_lane_released_track_ids
  new_lead, applied = rd._apply_overtake_surrogate(lead, sm)
  assert not applied
  assert new_lead == lead


def test_released_target_lane_track_does_not_re_register_near_center():
  rd = make_radard(registered_tracks={123})
  sm = StubSubMaster()
  target_lane_lead = make_lead(yRel=0.8)

  rd._update_lane_change_surrogates(sm, target_lane_lead)

  center_lead = make_lead(yRel=0.0)
  new_lead, applied = rd._apply_overtake_surrogate(center_lead, sm)

  assert not applied
  assert new_lead == center_lead
  assert 123 not in rd.surrogate_track_ids


def test_unregistered_target_lane_track_does_not_register_near_center():
  rd = make_radard()
  sm = StubSubMaster()
  target_lane_lead = make_lead(yRel=0.8)

  rd._update_lane_change_surrogates(sm, target_lane_lead)
  assert 123 in rd.target_lane_released_track_ids

  center_lead = make_lead(yRel=0.0)
  new_lead, applied = rd._apply_overtake_surrogate(center_lead, sm)

  assert not applied
  assert new_lead == center_lead
  assert 123 not in rd.surrogate_track_ids


def test_released_target_lane_track_reset_does_not_re_register_near_center():
  rd = make_radard(registered_tracks={123})
  sm = StubSubMaster()
  target_lane_lead = make_lead(yRel=0.8, dRel=31.5, vLead=14.5)

  rd._update_lane_change_surrogates(sm, target_lane_lead)

  rd.current_time = 1.0
  reset_track_lead = make_lead(radarTrackId=456, yRel=0.0, dRel=32.0, vLead=14.8)
  new_lead, applied = rd._apply_overtake_surrogate(reset_track_lead, sm)

  assert not applied
  assert new_lead == reset_track_lead
  assert 456 not in rd.surrogate_track_ids


def test_lane_change_starting_registers_replacement_track_before_divider_crossing():
  rd = make_radard()
  sm = StubSubMaster()
  lead = make_lead(radarTrackId=456, yRel=-0.8)

  new_lead, applied = rd._apply_overtake_surrogate(lead, sm)

  assert applied
  assert 456 in rd.surrogate_track_ids
  assert new_lead["dRel"] == pytest.approx(lead["dRel"] + SURROGATE_DREL_OFFSET)


def test_lane_change_starting_does_not_register_untracked_exempt_lead():
  rd = make_radard()
  sm = StubSubMaster()
  lead = make_lead(radarTrackId=456, yRel=0.4)

  new_lead, applied = rd._apply_overtake_surrogate(lead, sm)

  assert not applied
  assert new_lead == lead
  assert 456 not in rd.surrogate_track_ids


def _fresh_radard(v_ego):
  """A RadarD that has not seen a lane change yet, about to see preLaneChange -> laneChangeStarting."""
  rd = make_radard()
  rd.v_ego = v_ego
  rd.prev_lane_change_state = LaneChangeState.off
  rd.lc_direction_sign = 0
  rd.surrogate_phase = SURROGATE_PHASE_OFF
  rd.surrogate_speed_gate_open = False
  return rd


def _passed_car():
  # a slower car straight ahead in the source lane: the classic surrogate case
  return make_lead(yRel=0.0, vRel=-3.0, vLead=9.0, vLeadK=9.0, dRel=25.0)


def _drive_through_lane_change(rd, lead, v_ego_by_state):
  """Feed preLaneChange then laneChangeStarting; return whether the surrogate was applied in the starting frame."""
  sm = StubSubMaster()
  sm.modelV2.laneLines = [SimpleNamespace(x=[0.0, 100.0], y=[y, y]) for y in (-3.5, -1.75, 1.75, 3.5)]
  sm.modelV2.laneLineProbs = [1.0, 1.0, 1.0, 1.0]
  sm.frogpilotPlan = SimpleNamespace(laneWidthLeft=3.5, laneWidthRight=3.5)
  applied = None
  for state, v_ego in v_ego_by_state:
    rd.v_ego = v_ego
    sm.modelV2.meta.laneChangeState = state
    rd._update_lane_change_surrogates(sm, lead)
    _, applied = rd._apply_overtake_surrogate(lead, sm)
  return applied


def test_surrogate_off_below_speed_gate():
  rd = _fresh_radard(SURROGATE_MIN_V_EGO - 1.0)
  applied = _drive_through_lane_change(rd, _passed_car(), [(LaneChangeState.preLaneChange, SURROGATE_MIN_V_EGO - 1.0),
                                                            (LaneChangeState.laneChangeStarting, SURROGATE_MIN_V_EGO - 1.0)])
  assert applied is False
  assert rd.surrogate_speed_gate_open is False
  assert rd.surrogate_phase == SURROGATE_PHASE_OFF


def test_surrogate_on_at_speed_gate():
  rd = _fresh_radard(SURROGATE_MIN_V_EGO)
  applied = _drive_through_lane_change(rd, _passed_car(), [(LaneChangeState.preLaneChange, SURROGATE_MIN_V_EGO),
                                                            (LaneChangeState.laneChangeStarting, SURROGATE_MIN_V_EGO)])
  assert applied is True
  assert rd.surrogate_speed_gate_open is True


def test_speed_gate_closes_when_the_lateral_move_starts_slow():
  # blinker on at 13 m/s while still slowing toward a queue, lateral move starts at 10: no surrogate for this maneuver
  rd = _fresh_radard(SURROGATE_MIN_V_EGO + 1.0)
  applied = _drive_through_lane_change(rd, _passed_car(), [(LaneChangeState.preLaneChange, SURROGATE_MIN_V_EGO + 1.0),
                                                            (LaneChangeState.laneChangeStarting, SURROGATE_MIN_V_EGO - 2.0)])
  assert applied is False
  assert rd.surrogate_speed_gate_open is False
  assert rd.surrogate_phase == SURROGATE_PHASE_OFF


def test_speed_gate_stays_open_once_the_move_has_started():
  # opened at 13, started at 13, ego slows to 10 during the move: the published lead must not flip mid-maneuver
  rd = _fresh_radard(SURROGATE_MIN_V_EGO + 1.0)
  sm_states = [(LaneChangeState.preLaneChange, SURROGATE_MIN_V_EGO + 1.0), (LaneChangeState.laneChangeStarting, SURROGATE_MIN_V_EGO + 1.0),
               (LaneChangeState.laneChangeStarting, SURROGATE_MIN_V_EGO - 2.0)]
  applied = _drive_through_lane_change(rd, _passed_car(), sm_states)
  assert applied is True
  assert rd.surrogate_speed_gate_open is True


def test_speed_gate_never_reopens_mid_maneuver():
  # closed at 11 m/s, ego speeds up to 13 mid-maneuver: stays closed
  rd = _fresh_radard(SURROGATE_MIN_V_EGO - 1.0)
  applied = _drive_through_lane_change(rd, _passed_car(), [(LaneChangeState.preLaneChange, SURROGATE_MIN_V_EGO - 1.0),
                                                            (LaneChangeState.laneChangeStarting, SURROGATE_MIN_V_EGO + 1.0)])
  assert applied is False


def test_speed_gate_reopens_on_the_next_maneuver():
  rd = _fresh_radard(SURROGATE_MIN_V_EGO - 1.0)
  _drive_through_lane_change(rd, _passed_car(), [(LaneChangeState.preLaneChange, SURROGATE_MIN_V_EGO - 1.0),
                                                  (LaneChangeState.laneChangeStarting, SURROGATE_MIN_V_EGO - 1.0)])
  assert rd.surrogate_speed_gate_open is False
  applied = _drive_through_lane_change(rd, _passed_car(), [(LaneChangeState.off, SURROGATE_MIN_V_EGO + 3.0),
                                                            (LaneChangeState.preLaneChange, SURROGATE_MIN_V_EGO + 3.0),
                                                            (LaneChangeState.laneChangeStarting, SURROGATE_MIN_V_EGO + 3.0)])
  assert applied is True
