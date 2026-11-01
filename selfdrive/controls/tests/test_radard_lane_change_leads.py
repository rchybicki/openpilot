from types import SimpleNamespace

import pytest

from cereal import car, log
import openpilot.selfdrive.controls.radard as radard_module
from openpilot.selfdrive.controls.tests.test_radard_lane_change_surrogate import _real_radard

LaneChangeDirection = log.LaneChangeDirection
LaneChangeState = log.LaneChangeState
LANE_CHANGE = [LaneChangeState.off] * 3 + [LaneChangeState.preLaneChange] * 3 + [LaneChangeState.laneChangeStarting] * 8 + \
              [LaneChangeState.laneChangeFinishing] * 3 + [LaneChangeState.off] * 2
V_EGO = 25.0

# radar tracks (id, dRel, yRel, ground speed) and model leads (dRel, prob); yRel > 0 is the left lane
CASES = {
  "left_with_target_lane_car": (LaneChangeDirection.left, [(100, 25.0, 0.0, 20.0), (300, 40.0, 3.2, 23.0)], [(25.0, 0.95), (40.0, 0.6)]),
  "right_with_target_lane_car": (LaneChangeDirection.right, [(100, 25.0, 0.0, 20.0), (300, 40.0, -3.2, 23.0)], [(25.0, 0.95), (40.0, 0.6)]),
  "left_target_lane_car_only": (LaneChangeDirection.left, [(300, 35.0, 3.2, 22.0)], [(35.0, 0.4), (60.0, 0.2)]),
  "left_two_in_lane_leads": (LaneChangeDirection.left, [(100, 25.0, 0.0, 20.0), (200, 50.0, 0.2, 21.0), (300, 40.0, 3.2, 23.0)],
                             [(25.0, 0.95), (50.0, 0.8)]),
}

# leadOne / leadTwo per frame (status, radarTrackId, dRel) of today's radard (!my-fp-new, cleanup/fp1031-subtract c022a9890c)
BASELINE = {
  "left_with_target_lane_car": [
    [True, 100, 25.0, True, 300, 40.0], [True, 100, 25.0, True, 300, 40.0], [True, 100, 25.0, True, 300, 40.0], [True, 100, 65.0, True, 300, 40.0],
    [True, 100, 65.0, True, 300, 40.0], [True, 100, 65.0, True, 300, 40.0], [True, 100, 65.0, True, 300, 40.0], [True, 100, 65.0, True, 300, 40.0],
    [True, 100, 65.0, True, 300, 40.0], [True, 100, 65.0, True, 300, 40.0], [True, 100, 65.0, True, 300, 40.0], [True, 100, 65.0, True, 300, 40.0],
    [True, 100, 65.0, True, 300, 40.0], [True, 100, 65.0, True, 300, 40.0], [True, 100, 25.0, True, 300, 40.0], [True, 100, 25.0, True, 300, 40.0],
    [True, 100, 25.0, True, 300, 40.0], [True, 100, 25.0, True, 300, 40.0], [True, 100, 25.0, True, 300, 40.0],
  ],
  "right_with_target_lane_car": [
    [True, 100, 25.0, True, 300, 40.0], [True, 100, 25.0, True, 300, 40.0], [True, 100, 25.0, True, 300, 40.0], [True, 100, 65.0, True, 300, 40.0],
    [True, 100, 65.0, True, 300, 40.0], [True, 100, 65.0, True, 300, 40.0], [True, 100, 65.0, True, 300, 40.0], [True, 100, 65.0, True, 300, 40.0],
    [True, 100, 65.0, True, 300, 40.0], [True, 100, 65.0, True, 300, 40.0], [True, 100, 65.0, True, 300, 40.0], [True, 100, 65.0, True, 300, 40.0],
    [True, 100, 65.0, True, 300, 40.0], [True, 100, 65.0, True, 300, 40.0], [True, 100, 25.0, True, 300, 40.0], [True, 100, 25.0, True, 300, 40.0],
    [True, 100, 25.0, True, 300, 40.0], [True, 100, 25.0, True, 300, 40.0], [True, 100, 25.0, True, 300, 40.0],
  ],
  "left_target_lane_car_only": [
    [True, 300, 35.0, False, -1, 0.0], [True, 300, 35.0, False, -1, 0.0], [True, 300, 35.0, False, -1, 0.0], [True, 300, 35.0, False, -1, 0.0],
    [True, 300, 35.0, False, -1, 0.0], [True, 300, 35.0, False, -1, 0.0], [True, 300, 35.0, False, -1, 0.0], [True, 300, 35.0, False, -1, 0.0],
    [True, 300, 35.0, False, -1, 0.0], [True, 300, 35.0, False, -1, 0.0], [True, 300, 35.0, False, -1, 0.0], [True, 300, 35.0, False, -1, 0.0],
    [True, 300, 35.0, False, -1, 0.0], [True, 300, 35.0, False, -1, 0.0], [True, 300, 35.0, False, -1, 0.0], [True, 300, 35.0, False, -1, 0.0],
    [True, 300, 35.0, False, -1, 0.0], [True, 300, 35.0, False, -1, 0.0], [True, 300, 35.0, False, -1, 0.0],
  ],
  "left_two_in_lane_leads": [
    [True, 100, 25.0, True, 200, 50.0], [True, 100, 25.0, True, 200, 50.0], [True, 100, 25.0, True, 200, 50.0], [True, 100, 65.0, True, 200, 90.0],
    [True, 100, 65.0, True, 200, 90.0], [True, 100, 65.0, True, 200, 90.0], [True, 100, 65.0, True, 200, 90.0], [True, 100, 65.0, True, 200, 90.0],
    [True, 100, 65.0, True, 200, 90.0], [True, 100, 65.0, True, 200, 90.0], [True, 100, 65.0, True, 200, 90.0], [True, 100, 65.0, True, 200, 90.0],
    [True, 100, 65.0, True, 200, 90.0], [True, 100, 65.0, True, 200, 90.0], [True, 100, 25.0, True, 200, 50.0], [True, 100, 25.0, True, 200, 50.0],
    [True, 100, 25.0, True, 200, 50.0], [True, 100, 25.0, True, 200, 50.0], [True, 100, 25.0, True, 200, 50.0],
  ],
}


def run_case(name):
  direction, tracks, model_leads = CASES[name]
  toggles = SimpleNamespace(human_lane_changes=True, lane_detection_width=2.7, lead_detection_probability=0.35, adjacent_lead_tracking=True)
  real_toggles = radard_module.get_frogpilot_toggles
  radard_module.get_frogpilot_toggles = lambda sm=None: toggles
  try:
    return drive(_real_radard(V_EGO), toggles, direction, tracks, model_leads)
  finally:
    radard_module.get_frogpilot_toggles = real_toggles


def drive(rd, toggles, direction, tracks, model_leads):
  rd.frogpilot_toggles = toggles

  class _SM:
    msgs: dict = {}
    logMonoTime: dict = {}
    recv_frame: dict = {}
    seen = {'modelV2': True, 'frogpilotPlan': True}

    def __getitem__(self, k):
      return self.msgs[k]

    def all_checks(self):
      return True

  sm = _SM()
  lines = [SimpleNamespace(x=[0.0, 100.0], y=[y, y]) for y in (-3.5, -1.75, 1.75, 3.5)]
  out = []
  for frame, state in enumerate(LANE_CHANGE, start=1):
    t = frame * 50_000_000
    leads = [SimpleNamespace(prob=p, x=[d] * 6, xStd=[2.0] * 6, y=[0.0] * 6, yStd=[1.0] * 6, v=[20.0] * 6, vStd=[2.0] * 6, a=[0.0] * 6)
             for d, p in model_leads]
    model = SimpleNamespace(meta=SimpleNamespace(laneChangeState=state, laneChangeDirection=direction), laneLines=lines, laneLineProbs=[1.0] * 4,
                            position=SimpleNamespace(x=[0.0, 100.0]), velocity=SimpleNamespace(x=[V_EGO]), leadsV3=leads)
    rr = car.RadarData.new_message()
    pts = rr.init('points', len(tracks))
    for i, (tid, d, y, v) in enumerate(tracks):
      pts[i].trackId, pts[i].dRel, pts[i].yRel, pts[i].vRel, pts[i].measured = tid, d, y, v - V_EGO, True
    for svc, msg in (('carState', SimpleNamespace(vEgo=V_EGO, brakePressed=False)),
                     ('frogpilotPlan', SimpleNamespace(laneWidthLeft=3.5, laneWidthRight=3.5, increasedStoppedDistance=0.0)),
                     ('modelV2', model), ('liveTracks', rr.as_reader())):
      sm.msgs[svc] = msg
      sm.logMonoTime[svc] = t
      sm.recv_frame[svc] = frame
    rd.update(sm, rr.as_reader())
    one, two = rd.radar_state.leadOne, rd.radar_state.leadTwo
    out.append([bool(one.status), int(one.radarTrackId), round(float(one.dRel), 2), bool(two.status), int(two.radarTrackId), round(float(two.dRel), 2)])
  return out


@pytest.mark.parametrize("name", CASES)
def test_lane_change_leads_match_todays_radard(name):
  assert run_case(name) == BASELINE[name]
