"""Identification step hook (pure module) pins: arming, every precondition, the hold-and-release trigger, exact
100 Hz schedule timing, speed floor and deadline, same-frame lead abort, second-press abort, handback release
bound with deeper safety winning, trial counting, exception latch. Real functions only."""
import math
from dataclasses import replace

import pytest

from openpilot.selfdrive.controls.lib import identification_hook as ih
from openpilot.selfdrive.controls.lib.identification_hook import (HookInputs, IdentificationHook, precondition_failure,
                                                                  HOLD_S, PRECONDITION_S, V_END, RELEASE_JERK, MAX_TRIALS, TRIALS)


def good(**kw) -> HookInputs:
  base = HookInputs(valid=True, santa_fe=True, long_active=True, enabled=True, pid_state=True, v_ego=10.5, gas=False, brake=False,
                    force_coast=False, pause_long=False, standstill=False, steer_deg=1.0, yaw_rate=0.01, blinker=False, steer_fault=False,
                    esp_active=False, acc_faulted=False, can_valid=True, gear_drive=True, stock_aeb=False, stock_fcw=False,
                    lead_status=False, radar_error=False, lead_prob=0.02, plan_has_lead=False, plan_should_stop=False, plan_fcw=False,
                    stop_target_m=-1.0, distance_pressed=False, mapping_ok=True)
  return replace(base, **kw)


def run(hook, inputs_fn, n, normal=0.0, dt=0.01):
  outs = []
  for k in range(n):
    outs.append(hook.update(inputs_fn(k), normal, dt))
  return outs


def start_trial(hook, v=10.5, normal=0.0):
  """settle preconditions 2 s, hold 1.5 s, release -> the first ACTIVE frame's output."""
  run(hook, lambda k: good(v_ego=v), int(PRECONDITION_S * 100) + 5, normal)
  run(hook, lambda k: good(v_ego=v, distance_pressed=True), int(HOLD_S * 100) + 2, normal)
  return hook.update(good(v_ego=v, distance_pressed=False), normal, 0.01)


# -- preconditions ----------------------------------------------------------------------------------
@pytest.mark.parametrize("kw,reason", [
  (dict(valid=False), "inputs"), (dict(v_ego=float("nan")), "inputs"), (dict(santa_fe=False), "car"), (dict(mapping_ok=False), "mapping"),
  (dict(enabled=False), "disengaged"), (dict(long_active=False), "disengaged"), (dict(pid_state=False), "state"), (dict(standstill=True), "state"),
  (dict(gas=True), "pedal"), (dict(brake=True), "pedal"), (dict(force_coast=True), "pedal"), (dict(pause_long=True), "pedal"),
  (dict(v_ego=6.9), "speed"), (dict(v_ego=11.1), "speed"), (dict(lead_status=True), "lead"), (dict(lead_prob=0.10), "lead"),
  (dict(plan_has_lead=True), "lead"), (dict(radar_error=True), "fcw"), (dict(stock_aeb=True), "fcw"), (dict(stock_fcw=True), "fcw"),
  (dict(plan_fcw=True), "fcw"), (dict(plan_should_stop=True), "stop"), (dict(stop_target_m=40.0), "stop"), (dict(steer_deg=6.0), "steer"),
  (dict(yaw_rate=0.05), "steer"), (dict(blinker=True), "steer"), (dict(steer_fault=True), "steer"), (dict(esp_active=True), "vehicle"),
  (dict(acc_faulted=True), "vehicle"), (dict(can_valid=False), "vehicle"), (dict(gear_drive=False), "vehicle"),
])
def test_every_precondition_has_a_reason(kw, reason):
  assert precondition_failure(good(), True) is None
  assert precondition_failure(good(**kw), True) == reason


def test_during_a_trial_the_abort_band_applies_not_the_arming_band():
  assert precondition_failure(good(v_ego=6.0), for_start=False) is None
  assert precondition_failure(good(v_ego=3.9), for_start=False) == "speed"
  assert precondition_failure(good(v_ego=12.1), for_start=False) == "speed"


# -- arming and trigger -----------------------------------------------------------------------------
def test_unarmed_hook_never_acts():
  hook = IdentificationHook(armed=False)
  outs = run(hook, lambda k: good(distance_pressed=k % 400 < 200), 2000)
  assert all(not o.active and not o.handback for o in outs) and hook.state == "DISARMED"


def test_start_needs_two_seconds_of_preconditions_and_a_1p5s_hold_then_release():
  hook = IdentificationHook(armed=True)
  run(hook, lambda k: good(), 50)                       # only 0.5 s settled
  run(hook, lambda k: good(distance_pressed=True), 200)  # long hold
  o = hook.update(good(distance_pressed=False), 0.0)
  assert not o.active and hook.state in ("ARMED", "READY")   # the hold started before preconditions settled 2 s? -> still counts
  # settled: release after >= 1.5 s hold starts; a short hold does not
  run(hook, lambda k: good(), 250)
  run(hook, lambda k: good(distance_pressed=True), 100)     # 1.0 s only
  assert not hook.update(good(), 0.0).active
  o = start_trial(hook)
  assert o.active and hook.state == "ACTIVE" and hook.trial == 1 and o.accel == TRIALS[0][1][0]


def test_a_lead_or_pedal_on_the_settling_frames_resets_the_two_second_clock():
  hook = IdentificationHook(armed=True)
  run(hook, lambda k: good(), 150)
  hook.update(good(lead_status=True), 0.0)
  run(hook, lambda k: good(), 150)                       # only 1.5 s since the lead frame
  run(hook, lambda k: good(distance_pressed=True), 160)
  assert not hook.update(good(), 0.0).active


# -- schedule timing --------------------------------------------------------------------------------
def test_step_trial_runs_the_exact_duration_at_100hz_then_hands_back():
  hook = IdentificationHook(armed=True)
  o = start_trial(hook)
  assert o.accel == -0.5
  outs = run(hook, lambda k: good(v_ego=10.0), 299)
  assert all(o.active and o.accel == -0.5 for o in outs)   # 3.0 s = 300 frames incl. the start frame
  o = hook.update(good(v_ego=10.0), 0.0)
  assert o.handback and o.reason == "complete" and not o.active


def test_ramp_trial_interpolates_and_the_crossing_steps_at_one_second():
  hook = IdentificationHook(armed=True)
  hook.trial = 5                                          # next = ramp -0.5>-2.0
  start_trial(hook)
  outs = run(hook, lambda k: good(v_ego=10.0), 150)
  assert outs[-1].accel == pytest.approx(-0.5 + (-1.5) * 1.51 / 3.0, abs=0.02)
  hook2 = IdentificationHook(armed=True)
  hook2.trial = 7                                         # next = crossing
  start_trial(hook2)
  outs = run(hook2, lambda k: good(v_ego=10.0), 120)
  assert outs[95].accel == -0.8 and outs[105].accel == -2.2


def test_command_ends_early_at_the_speed_floor_and_never_goes_positive():
  hook = IdentificationHook(armed=True)
  start_trial(hook)
  outs = run(hook, lambda k: good(v_ego=10.0 - 0.05 * k), 200)
  ended = [o for o in outs if o.handback]
  assert ended and ended[0].reason == "complete"
  idx = outs.index(ended[0])
  assert 10.0 - 0.05 * (idx) <= V_END + 0.06
  assert all(o.accel <= 0.0 for o in outs)


# -- aborts -----------------------------------------------------------------------------------------
def test_lead_appearing_aborts_on_the_same_frame():
  hook = IdentificationHook(armed=True)
  start_trial(hook)
  run(hook, lambda k: good(v_ego=10.0), 50)
  o = hook.update(good(v_ego=10.0, lead_prob=0.3), 0.0)
  assert not o.active and o.handback and o.reason == "lead"


def test_second_press_pedal_and_disengage_abort():
  for kw, reason in ((dict(distance_pressed=True), "press"), (dict(brake=True), "pedal"), (dict(enabled=False), "disengaged")):
    hook = IdentificationHook(armed=True)
    start_trial(hook)
    run(hook, lambda k: good(v_ego=10.0), 60)
    o = hook.update(good(v_ego=10.0, **kw), 0.0)
    assert o.handback and o.reason == reason, (kw, o.reason)


def test_handback_releases_at_the_jerk_bound_and_deeper_safety_wins_immediately():
  hook = IdentificationHook(armed=True)
  start_trial(hook)
  run(hook, lambda k: good(v_ego=10.0), 60)
  o = hook.update(good(v_ego=10.0, lead_prob=0.5), 0.0)      # abort at -0.5
  assert o.handback and o.accel == pytest.approx(-0.5, abs=1e-9)
  o = hook.update(good(v_ego=10.0, lead_prob=0.5), 0.0)      # normal chain coasting: release cap climbs 0.8/s
  assert o.handback and o.accel == pytest.approx(-0.5 + RELEASE_JERK * 0.01, abs=1e-9)
  o = hook.update(good(v_ego=10.0, lead_prob=0.5), -1.5)     # normal chain demands deeper: it wins at once
  assert o.accel == pytest.approx(-0.5 + 2 * RELEASE_JERK * 0.01, abs=1e-9)   # the CAP is reported; the wire takes min(normal, cap)
  assert hook._last_cmd == pytest.approx(-1.5)
  # release completes when the normal chain is at/above the cap
  outs = run(hook, lambda k: good(v_ego=10.0, lead_prob=0.5), 300, normal=0.0)
  assert hook.state == "ARMED" and not outs[-1].handback


def test_trials_are_counted_and_capped():
  hook = IdentificationHook(armed=True)
  hook.trial = MAX_TRIALS
  run(hook, lambda k: good(), 250)
  run(hook, lambda k: good(distance_pressed=True), 200)
  o = hook.update(good(), 0.0)
  assert not o.active and "DONE" in o.text1


def test_exception_latches_the_hook_off_through_the_release_bound(monkeypatch):
  hook = IdentificationHook(armed=True)
  start_trial(hook)
  monkeypatch.setattr(ih, "precondition_failure", lambda *a, **k: (_ for _ in ()).throw(RuntimeError("boom")))
  o = hook.update(good(v_ego=10.0), 0.0)
  assert o.reason == "exception" and o.handback and hook.state == "HANDBACK"   # bounded release first (R1)
  monkeypatch.undo()
  outs = run(hook, lambda k: good(), 500)
  assert all(not o.active for o in outs) and hook.state == "DISARMED"


def test_schedule_constants_match_protocol_v2():
  assert [t[2][-1] for t in TRIALS] == [3.0, 3.0, 3.0, 2.5, 2.0, 3.0, 3.0, 3.0]
  assert all(max(t[1]) <= 0.0 for t in TRIALS)
  assert math.isclose(HOLD_S, 1.5) and math.isclose(PRECONDITION_S, 2.0)


# -- LongControl integration: the FINAL writer, PID reseed, flag-off equality ------------------------
def _lc_frames(monkeypatch, flag, armed, n=450, inputs_fn=None, a_target=-0.3, v=10.0):
  from openpilot.selfdrive.controls.lib import longcontrol as lcm, stopping_flags
  from openpilot.selfdrive.controls.lib.longcontrol import LongControl, LongCtrlState
  from openpilot.selfdrive.controls.lib.tests.test_longcontrol_fast_release import DummyCarParams, DummyCarState, DummyFrogPilotToggles
  monkeypatch.setattr(stopping_flags, "IDENTIFICATION_HOOK", flag)
  monkeypatch.setattr(lcm.os.path, "exists", lambda p: armed and p == ih.ARM_FILE)
  cp = DummyCarParams()
  cp.longitudinalTuning.kiV = [0.3]      # a real integrator so the reseed is meaningful
  lc = LongControl(cp)
  lc.long_control_state = LongCtrlState.pid
  wires, pid_i = [], []
  for k in range(n):
    inp = inputs_fn(k) if inputs_fn else None
    w = lc.update(active=True, CS=DummyCarState(v_ego=v, a_ego=-0.3), a_target=a_target, should_stop=False, distance_to_stop_target_m=-1.0,
                  accel_limits=(-3.0, 2.0), frogpilot_toggles=DummyFrogPilotToggles(), id_inputs=inp)
    wires.append(float(w))
    pid_i.append(float(lc.pid.i))
  return lc, wires, pid_i


def _press_schedule(k, v=10.0, **kw):
  # 0-2.5 s settle, 2.5-4.1 s hold (1.6 s), release at 4.1 s
  return good(v_ego=v, distance_pressed=250 <= k < 410, **kw)


def test_longcontrol_flag_off_or_unarmed_is_byte_identical(monkeypatch):
  _, base, _ = _lc_frames(monkeypatch, False, False, inputs_fn=None)
  _, off, _ = _lc_frames(monkeypatch, False, False, inputs_fn=_press_schedule)
  _, unarmed, _ = _lc_frames(monkeypatch, True, False, inputs_fn=_press_schedule)
  assert off == base and unarmed == base


def test_longcontrol_armed_trial_owns_the_wire_and_reseeds_the_pid(monkeypatch):
  lc, wires, pid_i = _lc_frames(monkeypatch, True, True, inputs_fn=_press_schedule, n=430)
  assert lc._id_hook is not None and lc._id_hook.armed
  assert lc.id_hook_out.active and lc.id_hook_out.trial == 1
  assert wires[-1] == -0.5 and wires[409] != -0.5           # scripted -0.5 from the release frame (410) on
  assert pid_i[-1] == pytest.approx(lc.last_output_accel - (lc.pid.p + lc.pid.d + lc.pid.f), abs=1e-9)


def test_longcontrol_handback_is_release_bounded_and_deeper_normal_wins(monkeypatch):
  def sched(k):
    if k < 450:
      return _press_schedule(k)
    return good(v_ego=10.0, lead_prob=0.5)                 # a lead appears mid-trial at 4.5 s -> abort
  lc, wires, _ = _lc_frames(monkeypatch, True, True, inputs_fn=sched, n=520, a_target=-0.3)
  assert wires[449] == -0.5 and lc.id_hook_out.reason == "lead"
  # after the abort the wire may rise at most 0.8 m/s^3 toward the normal chain (-0.3 here)
  rises = [wires[k] - wires[k - 1] for k in range(451, 520)]
  assert max(rises) <= 0.8 * 0.01 + 1e-9 and wires[-1] >= -0.5 and wires[-1] <= -0.29
  # a deeper normal demand wins at once during handback
  # a deeper normal demand wins AT ONCE during handback (no release step toward the cap): the wire goes deeper
  # than the scripted -0.5 on the abort frame and never rises while the normal chain stays deeper
  lc2, wires2, _ = _lc_frames(monkeypatch, True, True, inputs_fn=sched, n=520, a_target=-1.5)
  assert wires2[449] == -0.5 and wires2[450] <= -0.6 and wires2[-1] < -1.0
  assert all(wires2[k + 1] <= wires2[k] + 1e-9 for k in range(450, 519))


# -- R1 regressions -----------------------------------------------------------------------------------
def test_any_press_during_a_trial_aborts_even_on_the_first_frame():
  hook = IdentificationHook(armed=True)
  start_trial(hook)
  o = hook.update(good(v_ego=10.0, distance_pressed=True), 0.0)
  assert o.handback and o.reason == "press" and not o.active
  hook2 = IdentificationHook(armed=True)
  start_trial(hook2)
  run(hook2, lambda k: good(v_ego=10.0), 10)
  o = hook2.update(good(v_ego=10.0, distance_pressed=True), 0.0)    # a short press inside the first 300 ms
  assert o.handback and o.reason == "press"


def test_exception_handback_stays_release_bounded_then_disarms(monkeypatch):
  hook = IdentificationHook(armed=True)
  start_trial(hook)
  run(hook, lambda k: good(v_ego=10.0), 30)
  monkeypatch.setattr(ih, "precondition_failure", lambda *a, **k: (_ for _ in ()).throw(RuntimeError("boom")))
  o = hook.update(good(v_ego=10.0), 0.0)
  monkeypatch.undo()
  assert o.handback and o.reason == "exception" and o.accel == pytest.approx(-0.5)
  caps = [hook.update(good(v_ego=10.0), +0.5, 0.01).accel for _ in range(3)]      # normal wants +0.5: release bounded
  assert caps[0] == pytest.approx(-0.5 + RELEASE_JERK * 0.01) and caps[2] == pytest.approx(-0.5 + 3 * RELEASE_JERK * 0.01)
  outs = run(hook, lambda k: good(v_ego=10.0), 200, normal=0.5)
  assert hook.state == "DISARMED" and all(not o.active for o in outs)
  run(hook, lambda k: good(), 250)
  run(hook, lambda k: good(distance_pressed=True), 200)
  assert not hook.update(good(), 0.0).active                                    # no future trial this drive


def test_external_abort_hands_back_and_clears_qualification():
  hook = IdentificationHook(armed=True)
  start_trial(hook)
  hook.abort("fault")
  assert hook.state == "HANDBACK"
  o = hook.update(good(v_ego=10.0), 0.0)
  assert o.handback and not o.active and o.accel == pytest.approx(-0.5 + RELEASE_JERK * 0.01)
  hook2 = IdentificationHook(armed=True)
  run(hook2, lambda k: good(), 250)
  hook2.abort("reset")
  run(hook2, lambda k: good(distance_pressed=True), 200)
  assert not hook2.update(good(), 0.0).active                                    # the 2 s clock restarted


def test_longcontrol_input_fault_aborts_the_trial_and_never_resumes(monkeypatch):
  from openpilot.selfdrive.controls.lib import stopping_flags
  from openpilot.selfdrive.controls.lib.longcontrol import LongControl, LongCtrlState
  from openpilot.selfdrive.controls.lib import longcontrol as lcm
  from openpilot.selfdrive.controls.lib.tests.test_longcontrol_fast_release import DummyCarParams, DummyCarState, DummyFrogPilotToggles
  monkeypatch.setattr(stopping_flags, "IDENTIFICATION_HOOK", True)
  monkeypatch.setattr(lcm.os.path, "exists", lambda p: p == ih.ARM_FILE)
  lc = LongControl(DummyCarParams())
  lc.long_control_state = LongCtrlState.pid
  def step(k, plan_valid=True):
    return float(lc.update(active=True, CS=DummyCarState(v_ego=10.0, a_ego=-0.3), a_target=-0.3, should_stop=False, distance_to_stop_target_m=-1.0,
                           accel_limits=(-3.0, 2.0), frogpilot_toggles=DummyFrogPilotToggles(), id_inputs=_press_schedule(k), plan_valid=plan_valid))
  wires = [step(k) for k in range(430)]
  assert wires[-1] == -0.5 and lc.id_hook_out.active
  w_fault = step(430, plan_valid=False)                                          # invalid plan frame: fault path
  assert w_fault <= 0.0 and lc._id_hook.state == "HANDBACK" and lc._id_hook._reason == "fault"
  after = [step(k) for k in range(431, 600)]
  assert all(not (lc.id_hook_out and lc.id_hook_out.active) for _ in [0]) and lc._id_hook.trial == 1
  assert max(after[k + 1] - after[k] for k in range(len(after) - 1)) <= 0.8 * 0.01 + 1e-9   # bounded release after recovery
