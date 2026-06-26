from types import SimpleNamespace

import pytest

from openpilot.selfdrive.controls.radard import LaneChangeDirection, LaneChangeState, RadarD, SURROGATE_DREL_OFFSET, SURROGATE_PHASE_EXEC, SURROGATE_VLEAD_DELTA


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
