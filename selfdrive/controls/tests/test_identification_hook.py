"""Brake-response test mode (pure module + LongControl + controlsd wiring) pins: long-press arming and disarming, the
debounced short start, cancel, one repeated step, deeper-demand abort, driver override, fault lock, routine resets,
restart, banners and the alertDebug publisher. Real functions only."""
import math
from dataclasses import replace
from types import SimpleNamespace

import pytest

import cereal.messaging as messaging
from openpilot.selfdrive.controls.lib import identification_hook as ih
from openpilot.selfdrive.controls.lib.identification_hook import (HookInputs, IdentificationHook, precondition_failure, NOTICE_S,
                                                                  PRECONDITION_S, RELEASE_JERK, STEP_ACCEL, STEP_S, V_END)

SHORT = 10                  # frames: a short press (< CRUISE_LONG_PRESS = 50)
LONG = 60                   # frames: a long press (arms/disarms at its 50th frame)
SETTLE = 5                  # frames of release that end a press (MIN_PRESS_S)
ARM = range(10, 10 + LONG)  # LongControl schedule: a fresh long press arms test mode at frame 59
START = range(400, 400 + SHORT)   # a short press in READY; its release ends at frame 414
T0 = START.stop + SETTLE - 1      # first ACTIVE frame


def good(**kw) -> HookInputs:
  base = HookInputs(valid=True, santa_fe=True, long_active=True, enabled=True, pid_state=True, v_ego=10.5, gas=False, brake=False,
                    force_coast=False, pause_long=False, standstill=False, steer_deg=1.0, yaw_rate=0.01, blinker=False, steer_fault=False,
                    esp_active=False, acc_faulted=False, can_valid=True, gear_drive=True, stock_aeb=False, stock_fcw=False,
                    lead_status=False, radar_error=False, lead_prob=0.02, plan_has_lead=False, plan_should_stop=False, plan_fcw=False,
                    stop_target_m=-1.0, plan_accel=0.0, distance_pressed=False, distance_long=False, mapping_ok=True)
  return replace(base, **kw)


def run(hook, inputs_fn, n, normal=0.0, dt=0.01):
  return [hook.update(inputs_fn(k), normal, dt) for k in range(n)]


def press(hook, frames, normal=0.0, **kw):
  """press for `frames`, then release until the press ends; returns every output"""
  return (run(hook, lambda k: good(distance_pressed=True, **kw), frames, normal) +
          run(hook, lambda k: good(**kw), SETTLE, normal))


def armed(hook=None, **kw):
  hook = hook or IdentificationHook()
  if hook.state == "OFF":
    run(hook, lambda k: good(**kw), SETTLE)   # a release must be observed first
    press(hook, LONG, **kw)
  assert hook.state in ("ARMED", "READY")
  return hook


def ready(hook=None, **kw):
  hook = armed(hook, **kw)
  run(hook, lambda k: good(**kw), int(PRECONDITION_S * 100) + 5)
  assert hook.state == "READY"
  return hook


def start_trial(hook=None, **kw):
  """READY, a short press and its release -> the first ACTIVE frame's output"""
  hook = ready(hook, **kw)
  o = press(hook, SHORT, **kw)[-1]
  assert o.active and hook.state == "ACTIVE"
  return o


# -- preconditions ----------------------------------------------------------------------------------
@pytest.mark.parametrize("kw,reason", [
  (dict(valid=False), "inputs"), (dict(v_ego=float("nan")), "inputs"), (dict(plan_accel=float("nan")), "inputs"), (dict(santa_fe=False), "car"),
  (dict(mapping_ok=False), "mapping"), (dict(enabled=False), "disengaged"), (dict(long_active=False), "disengaged"), (dict(pid_state=False), "state"),
  (dict(standstill=True), "state"), (dict(gas=True), "pedal"), (dict(brake=True), "pedal"), (dict(force_coast=True), "pedal"),
  (dict(pause_long=True), "pedal"), (dict(v_ego=6.9), "speed"), (dict(v_ego=11.1), "speed"), (dict(lead_status=True), "lead"),
  (dict(lead_prob=0.10), "lead"), (dict(plan_has_lead=True), "lead"), (dict(radar_error=True), "fcw"), (dict(stock_aeb=True), "fcw"),
  (dict(stock_fcw=True), "fcw"), (dict(plan_fcw=True), "fcw"), (dict(plan_should_stop=True), "stop"), (dict(stop_target_m=40.0), "stop"),
  (dict(plan_accel=-0.51), "demand"), (dict(steer_deg=6.0), "steer"), (dict(yaw_rate=0.05), "steer"), (dict(blinker=True), "steer"),
  (dict(steer_fault=True), "steer"), (dict(esp_active=True), "vehicle"), (dict(acc_faulted=True), "vehicle"), (dict(can_valid=False), "vehicle"),
  (dict(gear_drive=False), "vehicle"),
])
def test_every_precondition_has_a_reason(kw, reason):
  assert precondition_failure(good(), True, 0.0) is None
  assert precondition_failure(good(**kw), True, 0.0) == reason


@pytest.mark.parametrize("kw,reason", [(dict(lead_status=True, plan_fcw=True), "fcw"), (dict(gas=True, acc_faulted=True), "vehicle"),
                                       (dict(enabled=False, radar_error=True), "fcw"), (dict(lead_prob=0.5, valid=False), "inputs")])
def test_a_fault_reports_before_an_ordinary_reason(kw, reason):
  assert precondition_failure(good(**kw), False, 0.0) == reason


@pytest.mark.parametrize("normal,plan,fails", [(STEP_ACCEL, STEP_ACCEL, False), (-0.51, 0.0, True), (0.0, -0.51, True), (0.5, 0.3, False),
                                               (float("nan"), 0.0, "inputs")])
def test_any_normal_or_planner_demand_deeper_than_the_step_fails(normal, plan, fails):
  r = precondition_failure(good(plan_accel=plan), False, normal)
  assert r == (fails if isinstance(fails, str) else "demand" if fails else None)


def test_during_a_trial_the_abort_band_applies_not_the_arming_band():
  assert precondition_failure(good(v_ego=6.0), False, 0.0) is None
  assert precondition_failure(good(v_ego=3.9), False, 0.0) == "speed"
  assert precondition_failure(good(v_ego=12.1), False, 0.0) == "speed"


# -- arming, disarming, restart -----------------------------------------------------------------------
def test_a_new_instance_is_off_and_short_presses_never_act():
  hook = IdentificationHook()
  outs = run(hook, lambda k: good(distance_pressed=k % 40 < SHORT), 2000)
  assert hook.state == "OFF" and not any(o.active or o.handback or o.text1 for o in outs)


def test_a_fresh_long_press_arms_at_its_threshold_and_the_rest_of_it_does_nothing():
  hook = IdentificationHook()
  run(hook, lambda k: good(), SETTLE)
  outs = run(hook, lambda k: good(distance_pressed=True), 300)   # held 3 s: arms at 0.5 s, then READY while still held
  arm_at = next(k for k, o in enumerate(outs) if o.state != "OFF")
  assert arm_at == 49 and outs[arm_at].changed and outs[arm_at].text1 == "TEST MODE ARMED - waiting: settling"
  assert outs[-1].state == "READY" and not any(o.active or o.handback for o in outs)
  outs = run(hook, lambda k: good(), 50)                         # its release neither starts a trial nor disarms
  assert hook.state == "READY" and not any(o.active for o in outs) and hook.trial == 0


def test_arming_applies_no_command_and_needs_no_engagement():
  hook = IdentificationHook()
  parked = dict(v_ego=0.0, standstill=True, long_active=False, enabled=False, gear_drive=False, pid_state=False)
  run(hook, lambda k: good(**parked), SETTLE)
  outs = press(hook, LONG, **parked)
  assert hook.state == "ARMED" and all(not o.active and not o.handback and o.accel == 0.0 for o in outs)
  assert outs[-1].text1 == "TEST MODE ARMED - waiting: vehicle"


def test_a_press_held_through_startup_never_arms():
  hook = IdentificationHook()                                    # controlsd restart with the button held
  run(hook, lambda k: good(distance_pressed=True), 300)
  assert hook.state == "OFF"
  run(hook, lambda k: good(), SETTLE)
  press(hook, LONG)
  assert hook.state == "ARMED"                                   # a fresh long press after the release still arms


@pytest.mark.parametrize("gap", ["invalid", "interrupt"])
def test_a_press_across_an_input_gap_never_counts(gap):
  hook = IdentificationHook()
  run(hook, lambda k: good(), SETTLE)
  run(hook, lambda k: good(distance_pressed=True), 20)
  for _ in range(10):
    if gap == "invalid":
      hook.update(good(distance_pressed=True, valid=False), 0.0)
    else:
      hook.interrupt()
  run(hook, lambda k: good(distance_pressed=True), 200)          # still held after the gap: no arming
  assert hook.state == "OFF"
  run(hook, lambda k: good(), SETTLE - 1)
  run(hook, lambda k: good(distance_pressed=True), LONG)         # a release shorter than 50 ms is not a release
  assert hook.state == "OFF"
  run(hook, lambda k: good(), SETTLE)
  press(hook, LONG)
  assert hook.state == "ARMED"


def test_arming_needs_the_test_scope():
  hook = IdentificationHook()
  run(hook, lambda k: good(mapping_ok=False), SETTLE)
  press(hook, LONG, mapping_ok=False)
  assert hook.state == "OFF"


@pytest.mark.parametrize("from_ready", [False, True])
def test_a_long_press_while_armed_turns_test_mode_off(from_ready):
  hook = ready() if from_ready else armed()
  outs = press(hook, LONG)
  assert hook.state == "OFF" and not any(o.active for o in outs) and hook.trial == 0
  assert (outs[-1].text1, outs[-1].text2) == ("TEST MODE OFF", "long press distance to arm")
  outs = run(hook, lambda k: good(), int(NOTICE_S * 100) + 10)   # the notice clears; nothing is shown while OFF
  assert outs[0].text1 == "TEST MODE OFF" and outs[-1].text1 == outs[-1].text2 == ""
  press(hook, SHORT)
  assert hook.state == "OFF"
  press(hook, LONG)
  assert hook.state == "ARMED"


# -- the short start ---------------------------------------------------------------------------------
def test_start_needs_two_seconds_of_preconditions_then_a_short_press_and_its_release():
  hook = armed()
  run(hook, lambda k: good(), 50)                                # only 0.5 s settled
  outs = press(hook, SHORT)
  assert not any(o.active for o in outs) and hook.state == "ARMED"
  run(hook, lambda k: good(), 300)                               # READY now: the early press was discarded, not queued
  assert hook.state == "READY" and not hook.update(good(), 0.0).active
  outs = run(hook, lambda k: good(distance_pressed=True), SHORT) + run(hook, lambda k: good(), SETTLE - 1)
  assert not any(o.active for o in outs) and outs[-1].text1 == "TEST READY - short press distance to start"
  o = hook.update(good(), 0.0)                                   # the press ends after 50 ms of release: start
  assert o.active and o.changed and hook.state == "ACTIVE" and hook.trial == 1 and o.accel == STEP_ACCEL
  assert (o.text1, o.text2) == ("TEST ACTIVE -0.50 m/s^2 - 0.0 s", "trial 1 - press distance to cancel")


@pytest.mark.parametrize("settled,starts", [(199, False), (200, True)])
def test_readiness_is_judged_before_the_press_frame_is_credited(settled, starts):
  hook = armed()
  run(hook, lambda k: good(), settled - round(hook._pre_t * 100))   # arming already credited its own frames
  assert any(o.active for o in press(hook, SHORT)) == starts


@pytest.mark.parametrize("frames,starts", [(1, False), (4, False), (5, True), (49, True), (50, False)])
def test_observed_press_duration_rejects_glitches_and_long_presses(frames, starts):
  hook = ready()
  outs = press(hook, frames)
  assert outs[-1].active == starts
  if frames >= 50:
    assert hook.state == "OFF"                                   # a long press disarms instead


@pytest.mark.parametrize("long_from", [SHORT - 1, SHORT])        # the card flags it while held, or only on the release frame
def test_the_card_long_flag_discards_a_start(long_from):
  hook = ready()
  run(hook, lambda k: good(distance_pressed=True, distance_long=k >= long_from), SHORT)
  outs = run(hook, lambda k: good(distance_long=long_from == SHORT and k == 0), SETTLE)
  assert not any(o.active for o in outs) and hook.state == "READY" and hook.trial == 0
  assert start_trial(hook).active


def test_a_release_dropout_inside_a_hold_is_not_a_release():
  hook = ready()
  outs = run(hook, lambda k: good(distance_pressed=True), 10)
  for gap in range(1, SETTLE):                                   # 10-40 ms dropouts inside one hold
    outs += run(hook, lambda k: good(), gap) + run(hook, lambda k: good(distance_pressed=True), 10)
  outs += run(hook, lambda k: good(distance_pressed=True), 10)
  assert not any(o.active for o in outs) and hook.state == "OFF" and hook.trial == 0   # it stayed one long press


@pytest.mark.parametrize("kw", [dict(lead_status=True), dict(gas=True), dict(valid=False), dict(mapping_ok=False), dict(plan_accel=-0.6)])
@pytest.mark.parametrize("when", ["held", "release"])
def test_a_failed_precondition_during_the_press_needs_a_fresh_press(kw, when):
  hook = ready()
  run(hook, lambda k: good(distance_pressed=True), 3)
  if when == "held":
    outs = [hook.update(good(distance_pressed=True, **kw), 0.0)] + press(hook, 3)
  else:
    outs = run(hook, lambda k: good(**kw) if k == 2 else good(), SETTLE)
  assert not any(o.active for o in outs) and hook.trial == 0
  assert start_trial(hook).active and hook.trial == 1


# -- the step ---------------------------------------------------------------------------------------
def test_every_trial_repeats_the_same_step_for_exactly_three_seconds():
  hook = IdentificationHook()
  results = []
  for n in (1, 2, 3):
    o = start_trial(hook)
    outs = [o] + run(hook, lambda k: good(), 299)
    assert all(o.active and o.accel == STEP_ACCEL for o in outs) and hook.trial == n
    o = hook.update(good(), 0.0)
    assert o.handback and o.reason == "complete" and (o.text1, o.text2) == (
      f"TEST {n} COMPLETE", "braking releases; normal cruise resumes and can accelerate")
    results.append(run(hook, lambda k: good(), 100)[-1])
  assert STEP_S == 3.0 and all(r.state == "ARMED" and r.text2 == f"last: trial {n} complete" for n, r in zip((1, 2, 3), results, strict=True))


def test_the_step_ends_early_at_the_speed_floor_and_never_goes_positive():
  hook = IdentificationHook()
  start_trial(hook)
  outs = run(hook, lambda k: good(v_ego=10.0 - 0.05 * k), 200)
  ended = [o for o in outs if o.handback]
  assert ended and ended[0].reason == "complete" and 10.0 - 0.05 * outs.index(ended[0]) <= V_END + 0.06
  assert all(o.accel <= 0.0 for o in outs)


def test_a_press_cancels_on_its_first_frame_and_does_nothing_else():
  hook = IdentificationHook()
  start_trial(hook)
  run(hook, lambda k: good(), 30)
  o = hook.update(good(distance_pressed=True), 0.0)
  assert o.handback and not o.active and o.reason == "press" and o.accel == STEP_ACCEL
  outs = run(hook, lambda k: good(distance_pressed=True), 300)   # held 3 s more: no disarm, re-arm or start
  outs += run(hook, lambda k: good(), SETTLE)
  assert hook.state == "READY" and hook.trial == 1 and not any(o.active for o in outs) and "OFF" not in {o.state for o in outs}
  assert outs[-1].text2 == "last: trial 1 aborted - press; long press = off"
  assert start_trial(hook).active and hook.trial == 2           # only a fresh press starts the next trial


def test_a_double_tap_starts_and_cancels():
  hook = ready()
  press(hook, SHORT)
  o = hook.update(good(distance_pressed=True), 0.0)
  assert o.handback and o.reason == "press" and hook.trial == 1


@pytest.mark.parametrize("kw,reason", [(dict(lead_prob=0.3), "lead"), (dict(steer_deg=8.0), "steer"), (dict(plan_should_stop=True), "stop"),
                                       (dict(plan_accel=-0.6), "demand"), (dict(v_ego=3.9), "speed")])
def test_an_ordinary_abort_hands_back_on_the_same_frame_and_stays_armed(kw, reason):
  hook = IdentificationHook()
  start_trial(hook)
  run(hook, lambda k: good(), 50)
  o = hook.update(good(**kw), 0.0)
  assert not o.active and o.handback and o.reason == reason and o.text1 == f"TEST 1 ABORTED - {reason}"
  run(hook, lambda k: good(), 100)
  assert hook.state == "ARMED" and hook.update(good(), 0.0).text2 == f"last: trial 1 aborted - {reason}"


def test_a_deeper_normal_demand_aborts_and_is_released_to_at_once():
  hook = IdentificationHook()
  start_trial(hook)
  o = hook.update(good(), -0.6)
  assert o.handback and o.reason == "demand" and o.accel == STEP_ACCEL   # LongControl sends min(normal, cap) = -0.6
  o = hook.update(good(), -0.6)
  assert hook.state == "ARMED"                                   # the normal chain is already below the cap


def test_handback_releases_at_the_jerk_bound_and_deeper_safety_wins_immediately():
  hook = IdentificationHook()
  start_trial(hook)
  run(hook, lambda k: good(), 60)
  o = hook.update(good(lead_prob=0.5), 0.0)                      # abort at -0.5
  assert o.handback and o.accel == pytest.approx(-0.5, abs=1e-9)
  o = hook.update(good(lead_prob=0.5), 0.0)                      # normal chain coasting: the release cap climbs 0.8/s
  assert o.handback and o.accel == pytest.approx(-0.5 + RELEASE_JERK * 0.01, abs=1e-9)
  o = hook.update(good(lead_prob=0.5), -1.5)                     # the normal chain demands deeper: it wins at once
  assert o.accel == pytest.approx(-0.5 + 2 * RELEASE_JERK * 0.01, abs=1e-9) and hook._last_cmd == pytest.approx(-1.5)
  outs = run(hook, lambda k: good(lead_prob=0.5), 300, normal=0.0)
  assert hook.state == "ARMED" and not outs[-1].handback


# -- driver, faults, resets -------------------------------------------------------------------------
@pytest.mark.parametrize("kw,reason", [(dict(brake=True), "pedal"), (dict(gas=True), "pedal"), (dict(enabled=False), "disengaged"),
                                       (dict(long_active=False), "disengaged"), (dict(long_active=False, brake=True), "pedal")])
def test_a_driver_action_during_the_step_ends_authority_at_once_and_turns_test_mode_off(kw, reason):
  hook = IdentificationHook()
  start_trial(hook)
  run(hook, lambda k: good(), 60)
  o = hook.update(good(**kw), 0.0)
  assert not o.active and not o.handback and o.accel == 0.0 and o.changed and o.state == hook.state == "OFF" and o.reason == reason
  assert (o.text1, o.text2) == (f"TEST 1 ABORTED - {reason}", "test mode off; long press distance to arm")
  outs = run(hook, lambda k: good(), 300)
  outs += press(hook, SHORT) + run(hook, lambda k: good(), 300)
  assert hook.state == "OFF" and not any(o.active for o in outs)
  press(hook, LONG)
  assert hook.state == "ARMED" and hook.trial == 1


@pytest.mark.parametrize("ended_by,after", [(dict(lead_prob=0.5), "ARMED"), (dict(pause_long=True), "OFF")])
def test_a_driver_action_during_the_release_ends_authority_and_keeps_the_trial_result(ended_by, after):
  hook = IdentificationHook()
  start_trial(hook)
  assert hook.update(good(**ended_by), 0.0).handback
  o = hook.update(good(brake=True, enabled=False), 0.0)
  assert not o.active and not o.handback and o.accel == 0.0 and hook.state == after


@pytest.mark.parametrize("kw,reason", [(dict(plan_fcw=True), "fcw"), (dict(stock_aeb=True), "fcw"), (dict(acc_faulted=True), "vehicle"),
                                       (dict(valid=False), "inputs"), (dict(mapping_ok=False), "mapping"), (dict(lead_prob=0.5, plan_fcw=True), "fcw")])
@pytest.mark.parametrize("phase", ["ACTIVE", "HANDBACK"])
def test_a_fault_during_the_step_or_its_release_locks_test_mode(kw, reason, phase):
  hook = IdentificationHook()
  start_trial(hook)
  if phase == "HANDBACK":
    assert hook.update(good(distance_pressed=True), 0.0).reason == "press"   # cancel first; the fault comes during the release
  o = hook.update(good(**kw), 0.0)
  assert o.handback and not o.active and o.text2 == "braking releases; test mode LOCKED"
  outs = run(hook, lambda k: good(), 300)
  assert hook.state == "LOCKED" and outs[-1].reason == reason and not any(o.active for o in outs)
  outs = press(hook, LONG) + run(hook, lambda k: good(), 300) + press(hook, SHORT)
  assert hook.state == "LOCKED" and not any(o.active for o in outs)
  assert (outs[LONG - 1].text1, outs[LONG - 1].text2) == (f"TEST MODE LOCKED - {reason}", "restart the car to use test mode again")


def test_a_driver_action_never_clears_a_lock():
  hook = IdentificationHook()
  start_trial(hook)
  o = hook.update(good(plan_fcw=True, brake=True, enabled=False), 0.0)
  assert not o.handback and hook.state == "LOCKED"


def test_routine_resets_keep_test_mode_armed_and_restart_qualification():
  hook = ready()
  for _ in range(500):                                           # controlsd while disengaged: LoC.reset() then update
    hook.reset()
    hook.update(good(long_active=False, enabled=False), 0.0)
  assert hook.state == "ARMED"
  run(hook, lambda k: good(), 150)                               # re-engaged: only 1.5 s qualified
  assert not any(o.active for o in press(hook, SHORT))
  assert start_trial(hook).active


def test_an_input_fault_while_armed_only_restarts_qualification():
  hook = ready()
  o = hook.interrupt()
  assert hook.state == "ARMED" and o.text1 == "TEST MODE ARMED - waiting: fault" and not o.active
  assert start_trial(hook).active


def test_an_input_fault_during_the_step_locks_and_its_release_stays_bounded():
  hook = IdentificationHook()
  start_trial(hook)
  o = hook.interrupt()
  assert o.handback and o.reason == "fault" and o.accel == STEP_ACCEL and o.text2 == "braking releases; test mode LOCKED"
  caps = [o.accel for o in run(hook, lambda k: good(), 100, normal=0.5)]
  assert caps[0] == pytest.approx(STEP_ACCEL + RELEASE_JERK * 0.01) and hook.state == "LOCKED"


def test_an_exception_during_the_step_locks_through_the_release_bound(monkeypatch):
  hook = IdentificationHook()
  start_trial(hook)
  run(hook, lambda k: good(), 30)
  monkeypatch.setattr(ih, "precondition_failure", lambda *a, **k: (_ for _ in ()).throw(RuntimeError("boom")))
  outs = run(hook, lambda k: good(), 120, normal=0.5)            # raising on EVERY frame
  caps = [o.accel for o in outs if o.handback]
  assert outs[0].reason == "exception" and len(caps) >= 60 and hook.state == "LOCKED" and not any(o.active for o in outs)
  assert all(caps[k + 1] - caps[k] <= RELEASE_JERK * 0.01 + 1e-9 for k in range(len(caps) - 1))


def test_an_exception_while_armed_locks_without_a_command(monkeypatch):
  hook = ready()
  monkeypatch.setattr(ih, "precondition_failure", lambda *a, **k: (_ for _ in ()).throw(RuntimeError("boom")))
  o = hook.update(good(), 0.0)
  assert hook.state == "LOCKED" and not o.active and not o.handback and o.accel == 0.0 and o.text1 == "TEST MODE LOCKED - exception"


def test_no_banner_text_can_trigger_the_prompt_sound():
  # events.longitudinal_maneuver_alert plays AudibleAlert.prompt when "Active" (case-sensitive) is in alertText1
  hook = IdentificationHook()
  outs = run(hook, lambda k: good(), SETTLE) + press(hook, LONG) + run(hook, lambda k: good(), 250) + press(hook, SHORT)
  outs += run(hook, lambda k: good(), 400) + press(hook, LONG) + run(hook, lambda k: good(), 400)
  assert {o.state for o in outs} >= {"OFF", "ARMED", "READY", "ACTIVE", "HANDBACK"}
  assert not any("Active" in o.text1 for o in outs)


# -- LongControl integration: the FINAL writer, PID reseed, flag-off equality ------------------------
def _press_schedule(k, v=10.0, **kw):
  return good(v_ego=v, distance_pressed=k in ARM or k in START, **kw)


def _lc(monkeypatch, flag=True, ki=0.0, kp=0.0):
  from openpilot.selfdrive.controls.lib import stopping_flags
  from openpilot.selfdrive.controls.lib.longcontrol import LongControl, LongCtrlState
  from openpilot.selfdrive.controls.lib.tests.test_longcontrol_fast_release import DummyCarParams
  monkeypatch.setattr(stopping_flags, "IDENTIFICATION_HOOK", flag)
  cp = DummyCarParams()
  cp.longitudinalTuning.kpV, cp.longitudinalTuning.kiV = [kp], [ki]   # the Santa Fe HEV runs kp = ki = 0
  lc = LongControl(cp)
  lc.long_control_state = LongCtrlState.pid
  return lc


def _step(lc, inputs, a_target=-0.3, v=10.0, active=True, **kw):
  from openpilot.selfdrive.controls.lib.tests.test_longcontrol_fast_release import DummyCarState, DummyFrogPilotToggles
  return float(lc.update(active=active, CS=DummyCarState(v_ego=v, a_ego=-0.3, brake_pressed=kw.pop("brake_pressed", False)), a_target=a_target,
                         should_stop=False, distance_to_stop_target_m=-1.0, accel_limits=(-3.0, 2.0), frogpilot_toggles=DummyFrogPilotToggles(),
                         id_inputs=inputs, **kw))


def _frames(lc, n, inputs_fn, a_target=-0.3):
  return [_step(lc, inputs_fn(k), a_target=a_target) for k in range(n)]


@pytest.mark.parametrize("kp,ki", [(0.0, 0.0), (1.0, 0.3)])
def test_longcontrol_flag_off_or_never_armed_is_byte_identical(monkeypatch, kp, ki):
  base = _frames(_lc(monkeypatch, False, ki, kp), 500, lambda k: None)
  off = _frames(_lc(monkeypatch, False, ki, kp), 500, _press_schedule)
  shorts = _frames(_lc(monkeypatch, True, ki, kp), 500, lambda k: good(distance_pressed=k % 40 < SHORT))
  assert off == base and shorts == base


def test_longcontrol_construction_is_off_and_scoped(monkeypatch):
  from openpilot.selfdrive.controls.lib.longcontrol import LongControl
  from openpilot.selfdrive.controls.lib.tests.test_longcontrol_fast_release import DummyCarParams
  assert _lc(monkeypatch)._id_hook.state == "OFF" and _lc(monkeypatch, flag=False)._id_hook is None
  assert LongControl(DummyCarParams(car_fingerprint="HYUNDAI_ELANTRA_2021"))._id_hook is None


@pytest.mark.parametrize("kp,ki", [(0.0, 0.0), (1.0, 0.3)])
def test_longcontrol_trial_owns_the_wire_and_reseeds_an_integrator(monkeypatch, kp, ki):
  lc = _lc(monkeypatch, ki=ki, kp=kp)
  wires = _frames(lc, T0 + 20, _press_schedule)
  assert lc._id_hook.state == "ACTIVE" and lc.id_hook_out.active and lc.id_hook_out.trial == 1
  assert wires[T0 - 1] != STEP_ACCEL and wires[T0:] == [STEP_ACCEL] * 20
  if ki:
    assert lc.pid.i == pytest.approx(lc.last_output_accel - (lc.pid.p + lc.pid.d + lc.pid.f), abs=1e-9)


@pytest.mark.parametrize("kp,ki", [(0.0, 0.0), (1.0, 0.3)])
def test_longcontrol_a_deeper_planner_demand_aborts_and_passes_at_once(monkeypatch, kp, ki):
  # with an integrator the per-frame reseed anchors the normal command at the step, so the planner target decides
  lc = _lc(monkeypatch, ki=ki, kp=kp)
  _frames(lc, T0 + 50, _press_schedule)
  wire = _step(lc, good(plan_accel=-2.0), a_target=-2.0)
  assert lc.id_hook_out.reason == "demand" and lc.id_hook_out.handback and wire < STEP_ACCEL
  later = [_step(lc, good(plan_accel=-2.0), a_target=-2.0) for _ in range(100)]
  assert all(w <= STEP_ACCEL for w in later) and later[-1] < -1.5 and lc._id_hook.state == "ARMED"


def test_longcontrol_handback_is_release_bounded(monkeypatch):
  lc = _lc(monkeypatch)
  _frames(lc, T0 + 50, _press_schedule)
  wires, handback = [], []
  for _ in range(80):
    wires.append(_step(lc, good(lead_prob=0.5), a_target=0.5))
    handback.append(lc.id_hook_out.handback)
  n = handback.index(False)
  assert wires[0] == STEP_ACCEL and lc.id_hook_out.reason == "lead" and n == 64 and wires[n - 1] == 0.0   # 0.625 s release
  assert max(b - a for a, b in zip(wires[:n], wires[1:n], strict=False)) <= RELEASE_JERK * 0.01 + 1e-9
  assert wires[-1] == 0.5                                        # then normal cruise owns the wire and can accelerate


@pytest.mark.parametrize("kw", [dict(enabled=False, brake=True), dict(enabled=False), dict(pause_long=True)], ids=["brake", "cancel", "pause"])
def test_longcontrol_off_after_a_step_requests_zero_on_every_frame(monkeypatch, kw):
  # controlsd with longActive False: LoC.reset() then LoC.update(active=False) on every frame
  lc = _lc(monkeypatch)
  _frames(lc, T0 + 50, _press_schedule)
  off = []
  for _ in range(300):
    lc.reset()
    off.append(_step(lc, good(long_active=False, **kw), active=False, brake_pressed=kw.get("brake", False)))
  assert off == [0.0] * 300 and lc._id_hook.state == "OFF" and lc._id_hook.trial == 1


def test_longcontrol_gas_while_active_drops_the_hook_bound(monkeypatch):
  lc = _lc(monkeypatch)
  _frames(lc, T0 + 50, _press_schedule)
  wire = _step(lc, good(gas=True), a_target=0.5, freeze_integrator=True)
  assert not lc.id_hook_out.handback and lc.id_hook_out.reason == "pedal" and wire > STEP_ACCEL + 0.1 and lc._id_hook.state == "OFF"


def test_longcontrol_arms_while_parked_across_routine_resets(monkeypatch):
  lc = _lc(monkeypatch)
  for k in range(200):
    lc.reset()
    _step(lc, good(long_active=False, enabled=False, v_ego=0.0, standstill=True, distance_pressed=k in ARM), v=0.0, active=False)
  assert lc._id_hook.state == "ARMED" and lc.id_hook_out.text1 == "TEST MODE ARMED - waiting: disengaged"


def test_longcontrol_input_fault_while_armed_does_not_lock_but_during_a_step_does(monkeypatch):
  lc = _lc(monkeypatch)
  _frames(lc, 300, _press_schedule)
  _step(lc, good(), plan_valid=False)
  assert lc._id_hook.state == "ARMED" and lc.id_hook_out.text1 == "TEST MODE ARMED - waiting: fault"
  _frames(lc, 250, lambda k: good())
  _frames(lc, SHORT + SETTLE, lambda k: good(distance_pressed=k < SHORT))
  assert lc._id_hook.state == "ACTIVE" and lc._id_hook.trial == 1
  held = _step(lc, good(), plan_valid=False)
  assert held == STEP_ACCEL and lc._id_hook.state == "HANDBACK" and lc.id_hook_out.reason == "fault"
  released = []
  while lc._id_hook.state == "HANDBACK":
    released.append(_step(lc, good(), a_target=0.5))
  assert len(released) == 64 and released[0] == STEP_ACCEL and released[-1] == 0.0
  assert all(0.0 <= b - a <= RELEASE_JERK * 0.01 + 1e-9 for a, b in zip(released, released[1:], strict=False))
  assert lc._id_hook.state == "LOCKED" and not any(_step(lc, _press_schedule(k)) == STEP_ACCEL for k in range(500))


# -- controlsd: inputs from genuine cereal messages, and the alertDebug banner ------------------------
class _SubMaster(dict):
  valid = dict.fromkeys(('carState', 'radarState', 'modelV2', 'longitudinalPlan', 'livePose', 'frogpilotCarState', 'selfdriveState'), True)
  alive = valid


def _controls_inputs(lead_probs, distance_pressed=False, fcs=None, toggles=None, maneuver_mode=False, a_target=0.0):
  from cereal import car
  from openpilot.selfdrive.controls.controlsd import Controls
  from openpilot.selfdrive.controls.lib.longcontrol import LongCtrlState
  msgs = {s: messaging.new_message(s) for s in ('modelV2', 'radarState', 'longitudinalPlan', 'frogpilotCarState', 'selfdriveState')}
  msgs['modelV2'].modelV2.init('leadsV3', len(lead_probs))
  for ld, p in zip(msgs['modelV2'].modelV2.leadsV3, lead_probs, strict=True):
    ld.prob = p
  msgs['longitudinalPlan'].longitudinalPlan.distanceToStopTarget = -1.0
  msgs['longitudinalPlan'].longitudinalPlan.aTarget = a_target
  msgs['frogpilotCarState'].frogpilotCarState.distancePressed = distance_pressed
  if fcs is not None:
    msgs['frogpilotCarState'].frogpilotCarState = fcs
  msgs['selfdriveState'].selfdriveState.enabled = True
  ctl = SimpleNamespace(sm=_SubMaster({s: getattr(m.as_reader(), s) for s, m in msgs.items()}),
                        frogpilot_toggles=toggles or SimpleNamespace(identification_mode=True), maneuver_mode=maneuver_mode,
                        CP=car.CarParams.new_message(carFingerprint="HYUNDAI_SANTA_FE_HEV_2022", openpilotLongitudinalControl=True),
                        LoC=SimpleNamespace(long_control_state=LongCtrlState.pid))
  CS = car.CarState.new_message(vEgo=10.5, canValid=True, gearShifter=car.CarState.GearShifter.drive).as_reader()
  return Controls._identification_inputs(ctl, CS, car.CarControl.new_message(longActive=True).as_reader())


def _start_from(lead_probs):
  hook = IdentificationHook()
  released, pressed = _controls_inputs(lead_probs), _controls_inputs(lead_probs, distance_pressed=True)
  run(hook, lambda k: released, SETTLE)
  run(hook, lambda k: pressed, LONG)
  outs = run(hook, lambda k: released, int(PRECONDITION_S * 100) + 5) + run(hook, lambda k: pressed, SHORT) + run(hook, lambda k: released, SETTLE)
  return hook, outs[-1]


# baseline (controlsd.py b37a78a9) started a -0.5 trial for the first six ([], [0.02], [0.02, nan], [-inf, 0.02], [0.02, -0.5],
# [-0.5, -0.2]); its max() already rejected [nan, 0.02] and [0.02, inf] (non-finite max -> 'inputs') and [0.02, 1.5] (-> 'lead')
MALFORMED_MODEL_LEADS = [[], [0.02], [0.02, math.nan], [-math.inf, 0.02], [0.02, -0.5], [-0.5, -0.2],
                         [math.nan, 0.02], [0.02, math.inf], [0.02, 1.5]]


@pytest.mark.parametrize("lead_probs", MALFORMED_MODEL_LEADS)
def test_controlsd_malformed_model_leads_fail_closed(lead_probs):
  assert precondition_failure(_controls_inputs(lead_probs), True, 0.0) == "inputs"
  hook, o = _start_from(lead_probs)
  assert not o.active and hook.trial == 0


@pytest.mark.parametrize("lead_probs", MALFORMED_MODEL_LEADS)
def test_controlsd_malformed_model_leads_abort_an_active_trial_and_lock(lead_probs):
  hook, o = _start_from([0.02, 0.01])
  assert o.active
  o = hook.update(_controls_inputs(lead_probs), 0.0)
  assert not o.active and o.handback and o.reason == "inputs"
  run(hook, lambda k: good(), 100)
  assert hook.state == "LOCKED"


@pytest.mark.parametrize("lead_probs", [[0.02, 0.01], [0.0, 0.0], [0.02, 0.01, 0.9]])   # modeld publishes three rows
def test_controlsd_two_low_model_leads_still_start_a_trial(lead_probs):
  inputs = _controls_inputs(lead_probs)
  assert inputs.valid and inputs.lead_prob == pytest.approx(max(lead_probs[:2]))
  hook, o = _start_from(lead_probs)
  assert o.active and hook.trial == 1 and o.accel == STEP_ACCEL


@pytest.mark.parametrize("lead_probs", [[0.02, 0.1], [0.3, 0.01], [0.0, 1.0]])
def test_controlsd_genuine_model_lead_still_blocks(lead_probs):
  assert precondition_failure(_controls_inputs(lead_probs), True, 0.0) == "lead"
  assert not _start_from(lead_probs)[1].active


def test_controlsd_passes_the_planner_target():
  assert _controls_inputs([0.02, 0.01], a_target=-0.7).plan_accel == pytest.approx(-0.7)
  assert precondition_failure(_controls_inputs([0.02, 0.01], a_target=-0.7), True, 0.0) == "demand"


@pytest.mark.parametrize("toggles,maneuver", [(dict(), False), (dict(identification_mode=False), False),
                                              (dict(identification_mode=True, force_coast_via_distance=True), False),
                                              (dict(identification_mode=True, traffic_mode_via_distance_very_long=True), False),
                                              (dict(identification_mode=True), True)])
def test_controlsd_mapping_gate_needs_the_scope_every_distance_mapping_off_and_no_maneuver_mode(toggles, maneuver):
  inputs = _controls_inputs([0.02, 0.01], toggles=SimpleNamespace(**toggles), maneuver_mode=maneuver)
  assert precondition_failure(inputs, True, 0.0) == "mapping"
  hook = IdentificationHook()
  run(hook, lambda k: inputs, SETTLE)
  run(hook, lambda k: replace(inputs, distance_pressed=True), LONG)
  assert hook.state == "OFF"                                     # no arming outside the scope or in maneuver mode


class _PM:
  def __init__(self, fail_on=None):
    self.sent, self.fail_on = [], fail_on

  def send(self, service, msg):
    if self.fail_on == "send":
      raise messaging.MultiplePublishersError(b"alertDebug")
    self.sent.append((service, msg.alertDebug.alertText1))


def _banner_controls(monkeypatch, fail_on=None):
  from openpilot.selfdrive.controls.controlsd import Controls
  created = []

  def pub_master(services):
    created.append(services)
    if fail_on == "create":
      raise messaging.MultiplePublishersError(b"alertDebug")    # the ZMQ backend refuses the second publisher here
    return pm
  pm = _PM(fail_on)
  monkeypatch.setattr(messaging, "PubMaster", pub_master)
  hook = IdentificationHook()
  ctl = SimpleNamespace(LoC=SimpleNamespace(_id_hook=hook, id_hook_out=None), id_banner_pm=None, id_banner_lost=False)
  publish = lambda: Controls._publish_id_banner(ctl)  # noqa: E731
  return ctl, hook, pm, created, publish


def test_controlsd_registers_no_banner_publisher_until_test_mode_has_text(monkeypatch):
  ctl, hook, pm, created, publish = _banner_controls(monkeypatch)
  for k in range(300):
    ctl.LoC.id_hook_out = hook.update(good(distance_pressed=k % 40 < SHORT), 0.0)
    publish()
  assert created == [] and pm.sent == []
  for k in range(SETTLE + LONG):
    ctl.LoC.id_hook_out = hook.update(good(distance_pressed=k >= SETTLE), 0.0)
    publish()
  assert created == [['alertDebug']] and pm.sent[-1] == ('alertDebug', "TEST MODE ARMED - waiting: settling")


@pytest.mark.parametrize("fail_on", ["create", "send"])
def test_controlsd_banner_loss_locks_the_hook_and_never_retakes_the_channel(monkeypatch, fail_on):
  ctl, hook, pm, created, publish = _banner_controls(monkeypatch, fail_on)
  start_trial(hook)
  ctl.LoC.id_hook_out = hook.update(good(), 0.0)
  publish()                                                      # the channel was taken by fullupdate.sh or maneuversd
  assert ctl.id_banner_lost and ctl.LoC.id_hook_out.handback and ctl.LoC.id_hook_out.reason == "banner"
  for _ in range(300):
    ctl.LoC.id_hook_out = hook.update(good(), 0.5)
    publish()
  assert hook.state == "LOCKED" and len(created) == 1 and pm.sent == []
