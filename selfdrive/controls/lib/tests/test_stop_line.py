"""Cycle 2026-10-04 entry-bite fix, upstream part: the stop line (stopping_flags.SANTA_FE_STOP_LINE). Real-function tests.
Round 2 (eb2 red-team): a queue restart above the band ends the line fast and never holds the launch command; inside the band
the service owns the stop, so the line no longer deepens there. Round 3 (eb3 verifier): a Doppler burst on a still-stopped lead
(its reported speed alone jumps to a level) is held, never released and re-grabbed; a launch (a rising speed) still ends it.
Round 4 (Astra code review): an armed line ramps out on a current provenance rejection or an uncertified replacement track; a
positive planner command never becomes a floor."""
from types import SimpleNamespace

import numpy as np
import pytest

from cereal import log
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.controls.lib import longitudinal_planner as planner_module
from openpilot.selfdrive.controls.lib import stopping_flags
from openpilot.selfdrive.controls.lib.longitudinal_planner import (
  SANTA_FE_STOP_AIM_CAP,
  SANTA_FE_STOP_AIM_STOP_WITHIN_M,
  SANTA_FE_STOP_LINE_BURST_FRAMES,
  SANTA_FE_STOP_LINE_J,
  SANTA_FE_STOP_LINE_J_RELEASE,
  SANTA_FE_STOP_LINE_OUT_FRAMES,
  LongitudinalPlanner,
  get_santa_fe_stop_line_demand,
  update_santa_fe_stop_line,
)
from openpilot.selfdrive.controls.lib.stopping_service import GOV_LAG, ServiceParams, governor_demand

ISD = 0.3
V_E = ServiceParams.V_ENTER
UP = SANTA_FE_STOP_LINE_J_RELEASE * DT_MDL


def lead(d_rel=20.0, v_lead=0.0, a_lead_k=0.0, tid=7, status=True):
  return SimpleNamespace(status=status, dRel=d_rel, vLead=v_lead, vRel=v_lead - 8.0, aLeadK=a_lead_k, radarTrackId=tid, modelProb=1.0)


NO_LEAD_TWO = lead(status=False, tid=-1)


def step(line, v, ld, cmd=-0.5, certified=True):
  """certified: the stop-commit persistence and track certificate on this frame (a class excursion resets both)"""
  return update_santa_fe_stop_line(line, v, ld, cmd, certified, certified, NO_LEAD_TWO, ISD)


def conflict(d):
  """the existing stop-commit provenance rejection: a radar return without model association in front of a strongly
  model-confirmed lead 10 m farther (santa_fe_stop_commit_track_provenance_ok)"""
  ld = lead(d)
  ld.modelProb = 0.0
  return ld, lead(d + 10.0, tid=99)


def armed_on_stopped(v=7.0, d=30.0, cmd=-0.8, frames=40):
  line = None
  for k in range(frames):
    line = step(line, v, lead(d - k * v * DT_MDL), cmd)
  assert line is not None and line[1] == 7
  return line


@pytest.fixture(params=[False, True], ids=["tau", "band"])
def profile(request, monkeypatch):
  # the line rides the governor profile the flag selects (the service runs the band profile on a stop the line shaped)
  monkeypatch.setattr(stopping_flags, "GOVERNOR_BAND_PROFILE", request.param)
  return request.param


def test_one_switch_trial_form():
  # car trial 2026-10-05 14:51, reverted the same day: the standard sim runner found hard-gate failures the earlier validation did
  # not cover (PLAN section 80). False turns both parts off on the next process start.
  assert stopping_flags.SANTA_FE_STOP_LINE is False
  assert stopping_flags.GOVERNOR_BAND_PROFILE is stopping_flags.SANTA_FE_STOP_LINE   # one switch: the band part is derived


def test_demand_is_the_governor_law_in_the_band(profile):
  for v, gap in ((2.4, 7.0), (1.5, 6.0), (0.8, 5.0)):
    g = governor_demand(v, 0.0, gap, ISD, band=profile)[0]
    assert g < 0.0 and get_santa_fe_stop_line_demand(v, gap, 0.0, ISD) == g


def test_demand_hands_over_without_a_step(profile):
  # above the band: the decel whose projected V_ENTER crossing makes the governor ask exactly that decel
  for v, gap in ((9.33, 44.9), (6.0, 20.0), (4.3, 16.9), (3.0, 9.0)):
    a = -get_santa_fe_stop_line_demand(v, gap, 0.0, ISD)
    crossing = gap - GOV_LAG * (v - V_E) - (v * v - V_E * V_E) / (2.0 * a)
    assert abs(governor_demand(V_E, 0.0, crossing, ISD, band=profile)[0] + a) < 1e-3
  assert abs(get_santa_fe_stop_line_demand(V_E + 1e-4, 8.1, 0.0, ISD) - get_santa_fe_stop_line_demand(V_E, 8.1, 0.0, ISD)) < 1e-2


def test_no_line_while_the_governor_allows_acceleration(profile):
  assert get_santa_fe_stop_line_demand(3.0, 30.0, 0.0, ISD) is None


def test_line_is_never_deeper_than_the_aim_cap(profile):
  # a comfort line: deeper needs stay with the existing lanes (aim/commit floors, MPC, service safety lanes), as on HEAD
  for v, gap in ((8.0, 12.0), (6.0, 8.0), (2.4, 4.6), (1.2, 4.35)):
    assert get_santa_fe_stop_line_demand(v, gap, 0.0, ISD) >= -SANTA_FE_STOP_AIM_CAP - 1e-9


def test_arms_only_certified_engages_from_the_command_and_deepens_at_most_j():
  stopped = lead(30.0)
  assert step(None, 7.0, stopped, -0.8, certified=False) is None
  line = step(None, 7.0, stopped, -0.8)
  assert line[1] == 7 and line[0] >= -0.8 - SANTA_FE_STOP_LINE_J * DT_MDL - 1e-9
  for _ in range(60):
    prev = line[0]
    line = step(line, 7.0, stopped, -0.8)
    assert line[0] >= min(prev, -0.8) - SANTA_FE_STOP_LINE_J * DT_MDL - 1e-9
  assert abs(line[0] - get_santa_fe_stop_line_demand(7.0, 30.0, 0.0, ISD)) < 1e-9


def test_no_fresh_line_inside_the_service_band():
  # the service owns the band (as on HEAD): a stopped lead first certified at or below V_ENTER arms nothing
  assert step(None, V_E, lead(7.0), -0.5) is None
  assert step(None, 1.5, lead(5.5), -0.5) is None


def test_moving_leads_never_arm():
  # stopped-lead scope: crawlers, decelerating leads and leads beyond the stop window arm nothing
  for vl, alk in ((0.45, 0.0), (0.8, 0.0), (1.5, 0.3), (5.0, -1.5), (2.0, -2.5)):
    assert step(None, 7.5, lead(15.0, vl, alk), -0.5) is None
  assert step(None, 9.0, lead(SANTA_FE_STOP_AIM_STOP_WITHIN_M + 1.0), -0.5) is None


@pytest.mark.parametrize("speeds", [(0.35, 0.45, 0.55), (0.39, 0.53, 0.61), (0.37, 0.43, 0.40)],
                         ids=["ramp", "jump_then_rise_203a", "noisy_20c0"])
def test_a_lead_that_starts_moving_ends_the_line(speeds):
  # a launch: the reported speed keeps rising past its first reading out of the stopped class (recorded queue restarts
  # 0000203a 455.85 and 000020c0 3486.76: the rise on the second frame counts even when the third reads lower)
  line = armed_on_stopped()
  d = line[3]
  for v_lead in speeds[:SANTA_FE_STOP_LINE_OUT_FRAMES]:
    d -= 7.0 * DT_MDL
    line = step(line, 7.0, lead(d, v_lead, 0.4), -0.2)
    assert line[1] == 7
  assert _ramps(line, step(line, 7.0, lead(d - 7.0 * DT_MDL, speeds[SANTA_FE_STOP_LINE_OUT_FRAMES], 0.4), -0.2), -0.2)


def _ramps(line, nxt, cmd):
  """an end: the floor rises by exactly J_RELEASE * dt, or the line is done because it reached the command or zero"""
  if nxt is None:
    return line[0] + UP >= min(cmd, 0.0) - 1e-9
  return nxt[1] is None and abs(nxt[0] - (line[0] + UP)) < 1e-9


def test_every_end_is_a_bounded_release():
  line = armed_on_stopped()
  d = line[3]
  cases = {
    'exit speed': (0.25, lead(d)),
    'different lead': (7.0, lead(d - 6.0, tid=9)),
    'governor no longer brakes': (3.0, lead(30.0)),
  }
  for name, (v, ld) in cases.items():
    nxt = step(line, v, ld, -0.2, certified=False)   # a new track id restarts the stop-commit certification
    assert _ramps(line, nxt, -0.2), name
  # the ramp continues every frame until the command is no longer below the floor
  nxt, frames = step(line, 0.25, lead(d), -0.2), 1
  while nxt is not None:
    prev = nxt
    nxt = step(nxt, 0.25, lead(d), -0.2)
    assert _ramps(prev, nxt, -0.2)
    frames += 1
  assert frames > 3


def test_release_ends_at_zero_and_never_holds_a_launch_command():
  # the brake release is J-limited; once it reaches zero the line is done: a positive (launch) command is never held
  line = armed_on_stopped(v=3.0, d=12.0)
  line = step(line, 0.25, lead(line[3]), 0.6)   # exit speed with the planner already asking +0.6 (lead departed)
  rises = []
  while line is not None:
    assert line[0] <= 0.0
    prev = line[0]
    line = step(line, 0.25, lead(5.0), 0.6, certified=False)
    if line is not None:
      rises.append(line[0] - prev)
  assert rises and max(rises) <= UP + 1e-9 and prev + UP >= 0.0 - 1e-9


@pytest.mark.parametrize("amp", [0.4, 0.6, 0.9, 1.2])
def test_doppler_burst_on_a_still_stopped_lead_never_releases_the_line(amp):
  # eb3 verifier MEDIUM: the stopped lead's reported speed alone jumps to a level for up to 0.36 s (recorded; 8 frames here)
  # while its range keeps following a stopped lead. The line holds without deepening through the whole burst and stays armed
  # after it without the stop-commit re-certification (a burst above 0.5 m/s resets it): no release, no re-grab.
  line = armed_on_stopped()
  d, f0 = line[3], line[0]
  for k in range(SANTA_FE_STOP_LINE_BURST_FRAMES):
    d -= 7.0 * DT_MDL
    line = step(line, 7.0, lead(d, amp + (0.02 if k % 2 else -0.02), 0.5), -0.2, certified=False)   # a level with noise
    assert line[1] == 7 and line[0] == f0
  d -= 7.0 * DT_MDL
  back = step(line, 7.0, lead(d), -0.2, certified=False)   # the burst ends: the same armed line, back on the law
  assert back[1] == 7 and f0 - SANTA_FE_STOP_LINE_J * DT_MDL - 1e-9 <= back[0] <= f0 + UP + 1e-9


def test_a_level_longer_than_any_recorded_burst_releases():
  # a lead that holds a level out of the stopped class longer than BURST_FRAMES (0.4 s) is not a burst: bounded release
  line = armed_on_stopped()
  d = line[3]
  for _ in range(SANTA_FE_STOP_LINE_BURST_FRAMES):
    d -= 7.0 * DT_MDL
    line = step(line, 7.0, lead(d, 0.6, 0.0), -0.2)
    assert line[1] == 7
  assert _ramps(line, step(line, 7.0, lead(d - 7.0 * DT_MDL, 0.6, 0.0), -0.2), -0.2)


def test_dropped_lead_frames_are_held_then_ramp():
  line = armed_on_stopped()
  f0 = line[0]
  for _ in range(SANTA_FE_STOP_LINE_OUT_FRAMES):
    line = step(line, 7.0, lead(status=False, tid=-1), -0.2)
    assert line[0] == f0 and line[1] == 7
  assert _ramps(line, step(line, 7.0, lead(status=False, tid=-1), -0.2), -0.2)


def _queue_restart(a_go=2.0, v=4.0, d=13.0, cmd0=-0.6, cmd_rate=1.5, cmd_max=1.0, seed=-1.2):
  """eb2 red-team HIGH (synthetic 'go' family, recorded 000020c0 3486): the line brakes behind a stopped lead above the band
  when the lead launches at a_go; the planner's own command rises at cmd_rate (the MPC relaxing into the launch). Returns
  the per-frame (t, command, output) after the launch onset; output = min(command, floor). seed: the planner's command when
  the stopped lead is certified; it falls at 2 m/s^3 to -1.2 during the approach (a positive seed: still accelerating)."""
  line, gap = None, d + 40 * v * DT_MDL
  for k in range(40):
    cmd = max(seed - 2.0 * k * DT_MDL, -1.2)
    prev, line = line, step(line, v, lead(gap - k * v * DT_MDL), cmd)
    assert line is None or line[0] < 0.0                                       # never a positive floor
    assert cmd <= 0.0 or line is None or prev is not None                      # arms only on a braking command ...
    assert prev is not None or line is None or line[0] >= cmd - SANTA_FE_STOP_LINE_J * DT_MDL - 1e-9   # ... from it (smooth)
  assert line is not None and line[1] == 7
  assert line[0] < cmd0 - 0.3
  out, lead_v, gap = [], 0.0, line[3]
  for k in range(60):
    t = k * DT_MDL
    lead_v += a_go * DT_MDL
    gap += (lead_v - v) * DT_MDL
    cmd = min(cmd0 + cmd_rate * t, cmd_max)
    line = step(line, v, lead(gap, lead_v, a_go), cmd)
    out.append((t, cmd, cmd if line is None else min(cmd, line[0]), lead_v))
  return out


def test_queue_restart_above_the_band_ends_the_line_fast():
  # the launch is confirmed after OUT_FRAMES out of the stopped class; the floor then rises at J_RELEASE and is gone well before
  # the planner's launch would be held (round 1: 0.3 s hold + 1.5 m/s^3 ramp through zero held it for ~1.5 s)
  for a_go in (1.0, 2.0):
    out = _queue_restart(a_go)
    t_out = min(t for t, _, _, lv in out if lv > 0.3)
    bound = [t for t, cmd, o, _ in out if o < cmd - 1e-9]
    assert bound and max(bound) - t_out <= 0.75
    assert SANTA_FE_STOP_LINE_J_RELEASE >= 2.5 and SANTA_FE_STOP_LINE_OUT_FRAMES * DT_MDL <= 0.1 + 1e-9


@pytest.mark.parametrize("seed", [-1.2, 0.8], ids=["braking", "positive"])
def test_queue_restart_never_holds_the_launch_command(seed):
  out = _queue_restart(2.0, cmd_rate=3.0, seed=seed)
  assert all(o <= 0.0 or o >= cmd - 1e-9 for _, cmd, o, _ in out)   # a positive command is never capped
  ups = [b[2] - a[2] for a, b in zip(out, out[1:], strict=False) if b[2] < b[1] - 1e-9]
  assert max(ups) <= UP + 1e-9                                      # while the line binds it releases at J_RELEASE at most


def test_line_never_deepens_inside_the_band(profile):
  # eb2 red-team MEDIUM (00002086 s8: a stopped lead certified at the band edge): inside the band the service owns the stop, so a
  # line armed above V_ENTER follows the law only upward there -- it never deepens below its hand-over value (round 1 kept
  # deepening at J toward the planner's raw governor law and moved the cost into the landing)
  line = None
  for k in range(4):
    line = step(line, V_E + 0.2, lead(6.4 - k * 0.13), -0.6)
  assert line is not None and line[1] == 7
  floor = line[0]
  for k in range(30):
    v = 2.2 - 0.03 * k
    nxt = step(line, v, lead(5.9 - 0.08 * k), -0.4)
    assert nxt is None or nxt[0] >= min(line[0], -0.4) - 1e-9
    line = nxt
    if line is None:
      break
  assert line is None or line[0] >= floor - 1e-9


def test_track_id_flip_with_continuous_range_keeps_the_line():
  line = armed_on_stopped()
  d = line[3] - 7.0 * DT_MDL
  flipped = step(line, 7.0, lead(d, tid=12), -0.2)
  assert flipped[1] == 12 and flipped[0] <= line[0] + UP + 1e-9
  # ... and a discontinuous range on the new id is a different lead (bounded release)
  other = step(line, 7.0, lead(d - 3.0, tid=12), -0.2, certified=False)
  assert _ramps(line, other, -0.2)
  # a certified new stopped lead re-arms from the releasing floor (continuous, deepens at most J)
  rearmed = step(other, 7.0, lead(d - 3.0 - 7.0 * DT_MDL, tid=12), -0.2)
  assert rearmed[1] == 12 and rearmed[0] >= other[0] - SANTA_FE_STOP_LINE_J * DT_MDL - 1e-9


def test_an_armed_line_ramps_out_on_a_provenance_conflict():
  # Astra LFL review P1: the line earned on a certified stopped lead kept deepening after the existing stop-commit provenance
  # check rejected that return (radar conflict with a model-confirmed farther leadTwo). The track certificate itself is not lost
  # (same track), so only the provenance check sees it: the line must ramp out (J_RELEASE) and stay out while the conflict lasts,
  # also from inside a held Doppler burst.
  for burst in (False, True):
    line = armed_on_stopped()
    d = line[3]
    if burst:
      d -= 7.0 * DT_MDL
      line = step(line, 7.0, lead(d, 0.6, 0.5), -0.2, certified=False)
      assert line[1] == 7 and line[2] == 1
    for k in range(40):
      d -= 7.0 * DT_MDL
      ld, ld2 = conflict(d)
      if burst and k < 3:
        ld.vLead = 0.6
      nxt = update_santa_fe_stop_line(line, 7.0, ld, -0.2, True, True, ld2, ISD)
      assert _ramps(line, nxt, -0.2), (burst, k)
      if nxt is None:
        break
      line = nxt
    assert nxt is None


def test_an_uncertified_replacement_track_ends_the_line():
  # Astra LFL review P1: a different track id with continuous range and speed is the same lead only when it holds its own
  # stop-commit certificate (a radar-only track first seen this close does not); the line's own track keeps the line through
  # a certificate reset by a class excursion (the deliberate burst/dwell continuity)
  line = armed_on_stopped()
  d = line[3] - 7.0 * DT_MDL
  replacement = lead(d, tid=12)
  replacement.modelProb = 0.0
  assert _ramps(line, update_santa_fe_stop_line(line, 7.0, replacement, -0.2, False, False, NO_LEAD_TWO, ISD), -0.2)
  assert update_santa_fe_stop_line(line, 7.0, replacement, -0.2, True, True, NO_LEAD_TWO, ISD)[1] == 12   # certified: kept
  assert update_santa_fe_stop_line(line, 7.0, lead(d), -0.2, False, False, NO_LEAD_TWO, ISD)[1] == 7      # own track: kept


def test_a_positive_command_never_becomes_a_floor():
  # Astra LFL review P2: a fresh line seeded from a positive command stored a positive floor (+0.76) that the level-burst hold
  # kept, capping a departure (+0.56 against +0.80 for 8 ticks). A line arms only on a braking command, so a positive command is
  # never capped and the hold can only hold braking; once the planner brakes the line starts from its command, at most J deeper.
  d = 15.0
  line = None
  for k in range(15):
    line = step(line, 3.0, lead(d - k * 0.15), 0.8)
    assert line is None
  for k in range(8):   # the lead moves at a level 0.4 m/s (would be a held excursion of an armed line)
    assert step(None, 3.0, lead(d - (15 + k) * 0.13, 0.4), 0.8) is None


def test_floor_never_rises_faster_than_the_release_rate():
  # a lead speed jump inside the stopped class changes the demand; the floor follows it up at most J_RELEASE
  line = armed_on_stopped()
  d = line[3] - 7.0 * DT_MDL
  nxt = step(line, 7.0, lead(d, 0.29), -0.2)
  assert nxt[1] == 7 and nxt[0] <= line[0] + UP + 1e-9


class _Solver:
  def __init__(self, **kwargs):
    self.mode, self.source, self.crash_cnt, self.solve_time = "blended", "e2e", 0, 0.0
    self.lead_xv_0 = self.lead_xv_1 = np.zeros((13, 2))

  def set_weights(self, *args, **kwargs):
    pass

  def set_cur_state(self, *args):
    pass

  def update(self, cruise, model, radar, x, v, a, j, *args, **kwargs):
    self.v_solution, self.a_solution, self.j_solution = v, a, j[:-1]


class _Messages(dict):
  updated = {"radarState": True}
  valid = {"radarState": True}
  alive = {"radarState": True}
  logMonoTime = {"modelV2": 0}

  def all_checks(self, **kwargs):
    return True


def _planner_world(monkeypatch):
  monkeypatch.setattr(planner_module, "LongitudinalMpc", _Solver)
  cp = SimpleNamespace(carFingerprint=planner_module.HYUNDAI_CAR.HYUNDAI_SANTA_FE_HEV_2022, openpilotLongitudinalControl=True,
                       wheelbase=2.7, steerRatio=15.0)
  toggles = SimpleNamespace(taco_tune=False, long_distance_factor=1.0, short_distance_factor=1.0,
                            longitudinalActuatorDelay=.45, vEgoStopping=.25, lead_detection_probability=.5)
  sm = _Messages(
    carState=SimpleNamespace(vEgo=8.0, aEgo=-.6, vCruise=50, steeringAngleDeg=0.0, standstill=False, gasPressed=False, brakePressed=False),
    controlsState=SimpleNamespace(longControlState=planner_module.LongCtrlState.pid, forceDecel=False),
    selfdriveState=SimpleNamespace(experimentalMode=True, enabled=True, personality=log.LongitudinalPersonality.standard),
    carControl=SimpleNamespace(orientationNED=[0.0, 0.0, 0.0], actuators=SimpleNamespace(accel=-.6)),
    liveParameters=SimpleNamespace(angleOffsetDeg=0.0),
    frogpilotCarState=SimpleNamespace(forceCoast=False, trafficModeEnabled=False),
    frogpilotPlan=SimpleNamespace(vCruise=8.0, minAcceleration=-3.5, maxAcceleration=2.0, cscControllingSpeed=False, laneWidthLeft=3.5,
                                  accelerationJerk=1.0, dangerJerk=1.0, speedJerk=1.0, dangerFactor=1.0, tFollow=1.5,
                                  increasedStoppedDistance=ISD),
    radarState=SimpleNamespace(leadOne=lead(34.0), leadTwo=lead(status=False)),
    modelV2=SimpleNamespace(position=SimpleNamespace(x=[]), velocity=SimpleNamespace(x=[]), acceleration=SimpleNamespace(x=[]),
                            meta=SimpleNamespace(disengagePredictions=SimpleNamespace(gasPressProbs=[1.0, 1.0])),
                            action=SimpleNamespace(desiredAcceleration=-.6, shouldStop=False), leadsV3=[SimpleNamespace(prob=.9)]),
  )
  return cp, toggles, sm


def _set_frame(sm, frame):
  v = max(8.0 - frame * .05, 0.0)
  sm['carState'].vEgo, sm['carState'].standstill = v, v < .05
  sm['radarState'].leadOne.dRel = max(34.0 - frame * .2, 4.3)
  times = np.array(planner_module.ModelConstants.T_IDXS)
  sm['modelV2'].velocity.x = np.maximum(v - .6 * times, 0.0).tolist()
  sm['modelV2'].position.x = (v * times).tolist()
  sm['modelV2'].acceleration.x = [-.6] * len(times)


def test_real_planner_flag_off_untouched_and_flag_on_only_deepens(monkeypatch):
  cp, toggles, sm = _planner_world(monkeypatch)
  planners = [LongitudinalPlanner(cp) for _ in range(2)]
  captured = []
  pm = SimpleNamespace(send=lambda name, message: captured.append(message.longitudinalPlan.aTarget))
  deeper = 0
  for frame in range(140):
    _set_frame(sm, frame)
    sm['carState'].gasPressed = 60 <= frame < 64
    for flag, planner in zip((False, True), planners, strict=True):
      monkeypatch.setattr(stopping_flags, "SANTA_FE_STOP_LINE", flag)
      planner.update(sm, toggles)
      planner.publish(sm, pm, toggles)
    off, on = captured[-2:]
    assert planners[0].stop_line is None
    assert on <= off + 1e-6
    deeper += on < off - 1e-3
    if sm['carState'].gasPressed:
      assert planners[1].stop_line is None
  assert deeper > 0


def test_real_planner_mode_edge_ramps_the_line_out(monkeypatch):
  cp, toggles, sm = _planner_world(monkeypatch)
  monkeypatch.setattr(stopping_flags, "SANTA_FE_STOP_LINE", True)
  planner = LongitudinalPlanner(cp)
  pm = SimpleNamespace(send=lambda name, message: None)
  for frame in range(30):
    _set_frame(sm, frame)
    planner.update(sm, toggles)
    planner.publish(sm, pm, toggles)
  assert planner.stop_line is not None and planner.stop_line[1] is not None
  floor = planner.stop_line[0]
  monkeypatch.setattr(stopping_flags, "SANTA_FE_STOP_COMMIT_ENVELOPE", False)   # a mode edge: the stop-commit block is not eligible
  _set_frame(sm, 30)
  planner.update(sm, toggles)
  assert planner.stop_line is None or (planner.stop_line[1] is None and abs(planner.stop_line[0] - (floor + UP)) < 1e-9)


def test_real_planner_lead_input_fault_holds_the_floor_then_ramps_it_out(monkeypatch):
  # a sustained nonfinite lead input: the fail-closed branch holds the previous non-positive command and the line's floor (no
  # release step through unusable inputs); from the recovery frame on the floor releases at exactly J_RELEASE per frame
  cp, toggles, sm = _planner_world(monkeypatch)
  monkeypatch.setattr(stopping_flags, "SANTA_FE_STOP_LINE", True)
  planner = LongitudinalPlanner(cp)
  pm = SimpleNamespace(send=lambda name, message: None)
  for frame in range(30):
    _set_frame(sm, frame)
    planner.update(sm, toggles)
    planner.publish(sm, pm, toggles)
  assert planner.stop_line is not None and planner.stop_line[1] is not None
  floor, held = planner.stop_line[0], float(planner.output_a_target)
  assert held <= floor + 1e-9
  for frame in range(30, 35):
    _set_frame(sm, frame)
    sm['radarState'].leadOne.dRel = float('nan')
    planner.update(sm, toggles)
    assert planner.lead_input_fault and planner.stop_line == (floor, None, 0, 0.0, 0.0, None)
    assert float(planner.output_a_target) == min(held, 0.0)
  prev, steps = floor, 0
  for frame in range(35, 70):
    _set_frame(sm, frame)
    planner.update(sm, toggles)
    if planner.stop_line is None or planner.stop_line[1] is not None:   # done, or re-armed fresh on the re-certified lead
      break
    assert abs(planner.stop_line[0] - (prev + UP)) < 1e-9
    prev, steps = planner.stop_line[0], steps + 1
  assert steps >= 1


def _scan_world(monkeypatch, v, cmd):
  """Astra's review world: the real planner on the test MPC stub at a constant ego speed, model/trajectory command cmd"""
  cp, toggles, sm = _planner_world(monkeypatch)
  _set_frame(sm, 0)
  sm['carState'].vEgo = v
  times = np.array(planner_module.ModelConstants.T_IDXS)
  sm['modelV2'].velocity.x = np.maximum(v + cmd * times, 0.0).tolist()
  sm['modelV2'].position.x = (v * times).tolist()
  sm['modelV2'].acceleration.x = [cmd] * len(times)
  sm['modelV2'].action.desiredAcceleration = cmd
  return cp, toggles, sm, [LongitudinalPlanner(cp) for _ in range(2)]


def test_real_planner_line_earned_then_rejected_returns_to_the_flag_off_command(monkeypatch):
  # Astra LFL review P1 (provenance_scan.py): 3 m/s, stopped radar track 7 at 15 m, command -0.2, range -0.15 m per frame. From
  # frame 15 the track loses its model association and a strongly model-confirmed leadTwo reads 10 m farther (the existing
  # stop-commit provenance rejection). Round 3 kept the line armed and deepening: -2.24 at 6.15 m against the flag-off -1.15.
  cp, toggles, sm, planners = _scan_world(monkeypatch, 3.0, -0.2)
  prev = None
  for k in range(60):
    gap = 15.0 - k * 0.15
    sm['radarState'].leadOne.dRel, sm['radarState'].leadOne.vRel = gap, -3.0
    if k >= 15:
      sm['radarState'].leadOne.modelProb = 0.0
      sm['radarState'].leadTwo = lead(gap + 10.0, tid=99)
    for flag, planner in zip((False, True), planners, strict=True):
      monkeypatch.setattr(stopping_flags, "SANTA_FE_STOP_LINE", flag)
      planner.update(sm, toggles)
    off, on = float(planners[0].output_a_target), float(planners[1].output_a_target)
    line = planners[1].stop_line
    if k == 14:
      assert line is not None and line[1] == 7 and on < off - 0.05   # the line was earned and binds
    if k >= 15:
      assert line is None or line[1] is None                         # authority lost: never armed again
      assert on <= off + 1e-9 and on - prev[1] <= max(UP, off - prev[0]) + 1e-9   # deepen-only, J-limited ramp out
    prev = (off, on)
  assert on == pytest.approx(off, abs=1e-9) and off == pytest.approx(-1.15, abs=1e-6)


def test_real_planner_positive_command_toward_a_stopped_lead_is_never_capped(monkeypatch):
  # Astra LFL review P2 (positive_launch.py): 3 m/s, command +0.8, a stopped lead from 16.5 m closing at 3 m/s; from frame 15 it
  # moves at a level 0.4 m/s. Round 3 armed a +0.76 floor and held +0.56 for eight ticks against the flag-off +0.80.
  cp, toggles, sm, planners = _scan_world(monkeypatch, 3.0, 0.8)
  gap = 16.5
  for k in range(26):
    v_lead = 0.0 if k < 15 else 0.4
    gap -= (3.0 - v_lead) * DT_MDL
    sm['radarState'].leadOne = lead(gap, v_lead)
    sm['radarState'].leadOne.vRel = v_lead - 3.0
    for flag, planner in zip((False, True), planners, strict=True):
      monkeypatch.setattr(stopping_flags, "SANTA_FE_STOP_LINE", flag)
      planner.update(sm, toggles)
    assert planners[1].stop_line is None
    assert float(planners[1].output_a_target) == float(planners[0].output_a_target) == pytest.approx(0.8)
