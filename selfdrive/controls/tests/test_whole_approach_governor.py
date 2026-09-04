"""Real-function tests for the planner-only whole-approach governor shadow."""

import inspect
import math
from types import SimpleNamespace

import numpy as np
import pytest

from cereal import log
from openpilot.selfdrive.controls.lib import longitudinal_planner as planner_module
from openpilot.selfdrive.controls.lib import stopping_flags
from openpilot.selfdrive.controls.lib.longitudinal_planner import (
  LongitudinalPlanner,
  get_whole_approach_shadow_candidate,
  reset_whole_approach_certificate,
  update_whole_approach_certificate,
)
from openpilot.selfdrive.controls.lib.stopping_governor import (
  capture_reserve,
  comfort_slew,
  gap_ref,
  profile_decel,
  whole_approach_demand,
)
from openpilot.selfdrive.controls.lib.stop_context import StopContext


def make_state():
  state = SimpleNamespace()
  reset_whole_approach_certificate(state)
  return state


def make_lead(*, status=True, d_rel=40.0, v_lead=0.0, a_lead_k=0.0, track_id=1, model_prob=0.8):
  return SimpleNamespace(status=status, dRel=d_rel, vLead=v_lead, vRel=v_lead - 8.0,
                         aLeadK=a_lead_k, radarTrackId=track_id, modelProb=model_prob)


def make_signals(*, gap_source="measured", track_age_s=0.5, motion=True, stopped=False, d_gap=None):
  return SimpleNamespace(d_gap=d_gap, gap_source=gap_source, track_age_s=track_age_s,
                         lead_motion_earned=motion, lead_confirmed_stopped=stopped, dropout_active=False, wheel_stop_latched=False)


def certificate(state, lead=None, lead_two=None, signals=None, **kwargs):
  lead = lead or make_lead()
  signals = signals or make_signals()
  if signals.d_gap is None:
    signals.d_gap = lead.dRel
  return update_whole_approach_certificate(
    state,
    santa_fe=kwargs.pop("santa_fe", True),
    blended=kwargs.pop("blended", True),
    engaged=kwargs.pop("engaged", True),
    force_coast=kwargs.pop("force_coast", False),
    v_ego=kwargs.pop("v_ego", 8.0),
    lead=lead or make_lead(),
    lead_two=lead_two or make_lead(status=False),
    signals=signals or make_signals(),
    fcw=kwargs.pop("fcw", False),
    isd=kwargs.pop("isd", 0.3),
    actual_a_target=kwargs.pop("actual_a_target", -0.5),
    standstill=kwargs.pop("standstill", False),
  )


def test_profile_endpoints_monotonic_distance_and_inverse():
  assert profile_decel(0.0) == pytest.approx(0.6)
  assert profile_decel(1e6) == pytest.approx(1.2)
  accelerations = [profile_decel(q) for q in range(12)]
  distances = [gap_ref(q, 0.3) for q in range(12)]
  assert accelerations == sorted(accelerations)
  assert distances == sorted(distances)
  for q in (0.0, 1.0, 4.0, 8.0, 11.0):
    demand = whole_approach_demand(q, 0.0, gap_ref(q, 0.3), 0.3)
    assert demand is not None
    assert demand[1] == pytest.approx(q, abs=1e-10)


def test_profile_distances_capture_slew_and_nonfinite():
  assert gap_ref(8.0, 0.3) == pytest.approx(45.0, abs=1.0)
  assert gap_ref(11.0, 0.3) == pytest.approx(73.0, abs=1.0)
  assert capture_reserve(8.0) == pytest.approx(7.6, abs=0.5)
  assert comfort_slew(-0.5, -2.0, 1.0) == pytest.approx(-1.1)
  assert comfort_slew(-1.0, 0.5, 1.0) == pytest.approx(-0.2)
  for values in ((math.nan, 0.0, 10.0, 0.3), (8.0, math.inf, 10.0, 0.3),
                 (8.0, 0.0, math.nan, 0.3), (8.0, 0.0, 10.0, math.inf)):
    assert whole_approach_demand(*values) is None


@pytest.mark.parametrize(("overrides", "lead", "signals", "lead_two", "expected"), [
  ({"santa_fe": False}, None, None, None, "not_santa_fe"),
  ({"blended": False}, None, None, None, "mode"),
  ({"engaged": False}, None, None, None, "disengaged"),
  ({"force_coast": True}, None, None, None, "force_coast"),
  ({}, make_lead(status=False), None, None, "lead"),
  ({}, make_lead(d_rel=-1.0), None, None, "gap"),
  ({}, make_lead(track_id=-1), None, None, "identity"),
  ({}, None, make_signals(track_age_s=0.45), None, "identity"),
  ({}, make_lead(model_prob=0.4), None, None, "prob"),
  ({"fcw": None}, None, None, None, "fcw_unavailable"),
  ({"fcw": True}, None, None, None, "fcw"),
  ({}, None, None, make_lead(d_rel=39.0), "lead2"),
  ({}, make_lead(v_lead=-5.5), None, None, "reversing"),
  ({}, make_lead(v_lead=2.0), None, None, "not_stopping"),
])
def test_certificate_ineligibility_reasons(overrides, lead, signals, lead_two, expected):
  reason, release = certificate(make_state(), lead=lead, signals=signals, lead_two=lead_two, **overrides)
  assert (reason, release) == (expected, None)


def test_stopped_branch_arms_at_capture_boundary():
  boundary = gap_ref(8.0, 0.3) + capture_reserve(8.0) + 4.0
  state = make_state()
  for _ in range(10):
    reason, _ = certificate(state, lead=make_lead(d_rel=boundary + 0.01), signals=make_signals(stopped=True))
  assert reason == "not_armed"
  assert not state.wa_committed
  for _ in range(10):
    reason, _ = certificate(state, lead=make_lead(d_rel=boundary - 0.01), signals=make_signals(stopped=True))
  assert reason == "ok"
  assert state.wa_committed
  assert state.wa_commit_reason == "stopped"


def test_real_stop_context_identity_and_stopped_dwells_complete_together():
  state = make_state()
  context = StopContext()
  lead = make_lead(d_rel=40.0, v_lead=0.0, track_id=7)
  for frame in range(10):
    signals = context.update(
      v_ego=8.0, a_ego=0.0, a_cmd=-0.5, lead_status=True, lead_v=0.0,
      lead_d_rel=40.0, lead_track_id=7, dt=0.05)
    reason, _ = certificate(state, lead=lead, signals=signals)
    if frame < 9:
      assert not state.wa_committed
  assert reason == "ok"
  assert signals.track_age_s == pytest.approx(0.5)
  assert state.wa_committed


def test_stopping_branch_arms_on_least_severe_window_and_projected_gap():
  boundary = gap_ref(8.0, 0.3) + capture_reserve(8.0) + 4.0
  state = make_state()
  lead = make_lead(d_rel=boundary - 8.1, v_lead=4.0, a_lead_k=-1.0)
  for _ in range(9):
    assert certificate(state, lead=lead)[0] == "not_stopping"
  assert certificate(state, lead=lead)[0] == "ok"
  assert state.wa_committed
  assert state.wa_commit_reason == "stopping"

  outside = make_state()
  outside_lead = make_lead(d_rel=boundary - 7.9, v_lead=4.0, a_lead_k=-1.0)
  for _ in range(10):
    reason, _ = certificate(outside, lead=outside_lead)
  assert reason == "not_armed"
  assert not outside.wa_committed


def test_phantom_and_identityless_leads_never_commit():
  for lead, expected in ((make_lead(v_lead=-5.5), "reversing"), (make_lead(track_id=-1), "identity")):
    state = make_state()
    for _ in range(20):
      reason, _ = certificate(state, lead=lead, signals=make_signals(stopped=True))
      assert reason == expected
    assert not state.wa_committed


def commit_stopped(state, *, track_id=1, d_rel=40.0):
  for _ in range(10):
    reason, _ = certificate(state, lead=make_lead(track_id=track_id, d_rel=d_rel), signals=make_signals(stopped=True))
  assert reason == "ok" and state.wa_committed


def test_release_on_departure_and_planner_go():
  departure = make_state()
  commit_stopped(departure)
  for _ in range(9):
    assert certificate(departure, lead=make_lead(d_rel=40.4, v_lead=9.0))[1] is None
  assert certificate(departure, lead=make_lead(d_rel=40.4, v_lead=9.0)) == ("released", "departure")

  planner_go = make_state()
  commit_stopped(planner_go)
  for _ in range(3):
    assert certificate(planner_go, lead=make_lead(v_lead=8.0), actual_a_target=0.3)[1] is None
  assert certificate(planner_go, lead=make_lead(v_lead=8.0), actual_a_target=0.3) == ("released", "planner_go")


def test_track_replacement_rate_limited_release_and_fresh_certificate():
  state = make_state()
  commit_stopped(state)
  state.wa_candidate = -1.0
  assert certificate(state, lead=make_lead(track_id=2), signals=make_signals(track_age_s=0.05)) == ("released", "track")
  assert state.wa_releasing
  assert comfort_slew(state.wa_candidate, -0.5, 0.05) == pytest.approx(-0.96)
  for frame in range(1, 10):
    reason, _ = certificate(state, lead=make_lead(track_id=2), signals=make_signals(track_age_s=frame * 0.05, stopped=True))
    assert reason == "identity"
    assert not state.wa_committed
  assert certificate(state, lead=make_lead(track_id=2), signals=make_signals(track_age_s=0.5, stopped=True))[0] == "ok"
  assert state.wa_committed


@pytest.mark.parametrize("santa_fe", [False, True])
@pytest.mark.parametrize("raise_shadow", [False, True])
def test_real_planner_update_and_publish_are_identical(monkeypatch, santa_fe, raise_shadow):
  # Only the native solver is substituted. Both planners execute the real update/publish paths.
  class Solver:
    def __init__(self, **kwargs):
      self.mode, self.source, self.crash_cnt, self.solve_time = "blended", "e2e", 0, 0.0
      self.lead_xv_0 = self.lead_xv_1 = np.zeros((13, 2))

    def set_weights(self, *args, **kwargs):
      pass

    def set_cur_state(self, *args):
      pass

    def update(self, cruise, model, radar, x, v, a, j, *args, **kwargs):
      self.v_solution, self.a_solution, self.j_solution = v, a, j[:-1]

  monkeypatch.setattr(planner_module, "LongitudinalMpc", Solver)
  cp = SimpleNamespace(carFingerprint=planner_module.HYUNDAI_CAR.HYUNDAI_SANTA_FE_HEV_2022 if santa_fe else "other",
                       openpilotLongitudinalControl=True, wheelbase=2.7, steerRatio=15.0)
  planners = [LongitudinalPlanner(cp) for _ in range(2)]
  toggles = SimpleNamespace(taco_tune=False, long_distance_factor=1.0, short_distance_factor=1.0,
                            longitudinalActuatorDelay=.45, vEgoStopping=.25, lead_detection_probability=.5)
  captured = []
  pm = SimpleNamespace(send=lambda name, message: captured.append(message.longitudinalPlan.to_dict()))
  class Messages(dict):
    updated = {"radarState": True}
    valid = {"radarState": True}
    alive = {"radarState": True}
    logMonoTime = {"modelV2": 0}

    def all_checks(self, **kwargs):
      return True

  sm = Messages(
    carState=SimpleNamespace(vEgo=8.0, aEgo=-.6, vCruise=50, steeringAngleDeg=0.0, standstill=False,
                             gasPressed=False, brakePressed=False),
    controlsState=SimpleNamespace(longControlState=planner_module.LongCtrlState.pid, forceDecel=False),
    selfdriveState=SimpleNamespace(experimentalMode=True, enabled=True, personality=log.LongitudinalPersonality.standard),
    carControl=SimpleNamespace(orientationNED=[0.0, 0.0, 0.0], actuators=SimpleNamespace(accel=-.6)),
    liveParameters=SimpleNamespace(angleOffsetDeg=0.0),
    frogpilotCarState=SimpleNamespace(forceCoast=False),
    frogpilotPlan=SimpleNamespace(vCruise=8.0, minAcceleration=-3.5, maxAcceleration=2.0, cscControllingSpeed=False,
                                  laneWidthLeft=3.5, accelerationJerk=1.0, dangerJerk=1.0, speedJerk=1.0,
                                  dangerFactor=1.0, tFollow=1.5, increasedStoppedDistance=.3),
    radarState=SimpleNamespace(leadOne=make_lead(), leadTwo=make_lead(status=False)),
    modelV2=SimpleNamespace(position=SimpleNamespace(x=[]), velocity=SimpleNamespace(x=[]), acceleration=SimpleNamespace(x=[]),
                           meta=SimpleNamespace(disengagePredictions=SimpleNamespace(gasPressProbs=[1.0, 1.0])),
                           action=SimpleNamespace(desiredAcceleration=-.6, shouldStop=False), leadsV3=[SimpleNamespace(prob=.9)]),
  )
  saw_commit = False
  if raise_shadow:
    def fail(**kwargs):
      raise RuntimeError("shadow fault injection")
    monkeypatch.setattr(planners[1], "_update_whole_approach_shadow", fail)
  for frame in range(160):
    sm['carState'].vEgo = max(8.0 - frame * .065, 0.0)
    sm['carState'].standstill = sm['carState'].vEgo < .05
    sm['radarState'].leadOne.dRel = max(40.0 - frame * .29, 4.3)
    sm.updated['radarState'] = not 40 <= frame < 55
    sm['carState'].gasPressed = 60 <= frame < 70
    sm['frogpilotCarState'].forceCoast = 80 <= frame < 90
    sm['selfdriveState'].experimentalMode = not 90 <= frame < 100
    sm['controlsState'].longControlState = planner_module.LongCtrlState.off if 100 <= frame < 110 else planner_module.LongCtrlState.pid
    times = np.array(planner_module.ModelConstants.T_IDXS)
    sm['modelV2'].velocity.x = np.maximum(sm['carState'].vEgo - .6 * times, 0.0).tolist()
    sm['modelV2'].position.x = (sm['carState'].vEgo * times).tolist()
    sm['modelV2'].acceleration.x = [-.6] * len(times)
    for flag, planner in zip(("off", "shadow"), planners, strict=True):
      monkeypatch.setattr(stopping_flags, "WHOLE_APPROACH_GOVERNOR", flag)
      planner.update(sm, toggles)
      planner.publish(sm, pm, toggles)
    saw_commit |= planners[1].wa_committed
    if sm['carState'].gasPressed or sm['carState'].standstill or not sm.updated['radarState']:
      assert not planners[1].wa_committed
    actual = [{k: v for k, v in message.items() if not k.startswith('wholeApproach') and k != 'processingDelay'}
              for message in captured[-2:]]
    assert actual[0] == actual[1]
  if santa_fe and not raise_shadow:
    assert saw_commit


def test_flag_off_publishes_only_sentinels_without_touching_context(monkeypatch):
  class _ResetOnly:
    def reset(self):
      pass

    def update(self, *args, **kwargs):
      raise AssertionError("off mode must not evaluate context")

  monkeypatch.setattr(stopping_flags, "WHOLE_APPROACH_GOVERNOR", "off")
  planner = LongitudinalPlanner.__new__(LongitudinalPlanner)
  reset_whole_approach_certificate(planner)
  planner.wa_stats = {"must": "clear"}
  planner._wa_lead_auth = planner._wa_stop_ctx = _ResetOnly()
  planner._update_whole_approach_shadow(
    santa_fe=True, blended=True, engaged=True, force_coast=False, v_ego=8.0, a_ego=0.0,
    actual_a_target=-0.5, a_target_mpc=-0.6, lead=make_lead(), lead_two=make_lead(status=False),
    fcw=False, isd=0.3, standstill=False, radar_fresh=True)
  assert math.isnan(planner.whole_approach_demand)
  assert math.isnan(planner.whole_approach_safety_min)
  assert math.isnan(planner.whole_approach_deficit)
  assert planner.whole_approach_reason == "off"
  assert not planner.wa_committed


def test_shadow_call_is_after_final_control_assignments_and_has_no_control_writer():
  update_source = inspect.getsource(LongitudinalPlanner.update)
  assert update_source.index("self.output_a_target =") < update_source.index("self._update_whole_approach_shadow(")
  shadow_source = inspect.getsource(LongitudinalPlanner._update_whole_approach_shadow)
  assert "self.output_a_target =" not in shadow_source
  assert "self.output_a_target_trajectory =" not in shadow_source
  assert "self.output_should_stop =" not in shadow_source


def test_mpc_hazard_lane_binds_when_deeper():
  result = get_whole_approach_shadow_candidate(
    None, v_ego=8.0, a_ego=0.0, v_lead=0.0, d_rel=50.0, isd=0.3,
    actual_a_target=-0.5, a_target_mpc=-2.0, dt=0.05)
  assert result is not None
  candidate, safety_min, lanes = result
  assert lanes["mpc"] == safety_min == candidate == -2.0


def test_one_bounded_telemetry_event_per_approach_end(monkeypatch):
  events = []
  monkeypatch.setattr("openpilot.selfdrive.controls.lib.longitudinal_planner.cloudlog.event",
                      lambda name, **payload: events.append((name, payload)))
  planner = LongitudinalPlanner.__new__(LongitudinalPlanner)
  planner.wa_stats = {
    "frames": 42, "entry_v": 8.0, "entry_gap": 45.0, "commit_reason": "stopped",
    "max_candidate": -0.4, "min_candidate": -1.2, "deeper_frames": 12, "shallower_frames": 3,
    "max_candidate_step": 0.03, "safety_bound_frames": {"kin": 2, "pred": 4, "mpc": 6},
  }
  planner._finish_whole_approach("stop")
  planner._finish_whole_approach("stop")
  assert len(events) == 1
  assert events[0][0] == "whole_approach"
  assert events[0][1]["frames_committed"] == 42
  assert events[0][1]["safety_bound_frames"] == {"kin": 2, "pred": 4, "mpc": 6}


@pytest.mark.parametrize("signals", [make_signals(gap_source="held"), make_signals(gap_source="decay"),
                                   make_signals(track_age_s=math.nan)])
def test_untrusted_signals_never_commit(signals):
  state = make_state()
  for _ in range(20):
    certificate(state, signals=signals)
    assert not state.wa_committed


def test_stop_and_same_track_fault_need_fresh_certificate():
  state = make_state()
  commit_stopped(state)
  assert certificate(state, standstill=True)[1] == "stop"
  for _ in range(30):
    assert certificate(state, standstill=True)[0] == "stop"
    assert not state.wa_committed
  commit_stopped(state)
  assert certificate(state, lead=make_lead(v_lead=math.nan))[1] == "lead"
  for _ in range(9):
    certificate(state, signals=make_signals(stopped=True))
    assert not state.wa_committed
  assert certificate(state, signals=make_signals(stopped=True))[0] == "ok"


def test_stopped_branch_requires_conditioned_stopped_evidence():
  state = make_state()
  for _ in range(30):
    certificate(state, lead=make_lead(v_lead=-.2), signals=make_signals(stopped=False))
    assert not state.wa_committed


def test_measured_gap_still_uses_the_conditioned_outward_limit():
  state = make_state()
  for _ in range(10):
    certificate(state, lead=make_lead(d_rel=70), signals=make_signals(stopped=True, d_gap=40))
  assert state.wa_committed
  assert state.wa_entry_gap == 40
