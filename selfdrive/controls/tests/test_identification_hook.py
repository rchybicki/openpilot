"""Brake-response test program KCS1 (pure module + LongControl + controlsd wiring) pins: long-press arming and disarming,
the debounced short start and the READY auto-start countdown, the maneuver order and block end, every segment walk and
switch, the stop intent and `own` (a running or held rep owns the wire; an abort, a finish or a locked hold does not),
the held stop and its brake end, cancel and aborts before/after the intent, stalled reps, the 30 s cap, driver override,
fault lock, progress load/save, banners and the alertDebug publisher. Real functions only."""
import json
import math
from dataclasses import replace
from itertools import takewhile
from types import SimpleNamespace

import pytest

import cereal.messaging as messaging
from openpilot.selfdrive.controls.lib import identification_hook as ih
from openpilot.selfdrive.controls.lib.identification_hook import (A_HOLD, AUTO_START_S, BLOCKS, CAP_S, DT, HOLD_BRAKE_S, HOLD_MIN_S, J_HOLD,
                                                                  INTENT_V, N_REPS, NOTICE_S, PRECONDITION_S, RELEASE_JERK, V_OVER,
                                                                  HookInputs, IdentificationHook, precondition_failure)

SHORT = 10                  # frames: a short press (< CRUISE_LONG_PRESS = 50)
LONG = 60                   # frames: a long press (arms/disarms at its 50th frame)
SETTLE = 5                  # frames of release that end a press (MIN_PRESS_S)
ARM = range(10, 10 + LONG)  # LongControl schedule: a fresh long press arms test mode at frame 59
START = range(400, 400 + SHORT)   # a short press in READY; its release ends at frame 414
T0 = START.stop + SETTLE - 1      # first ACTIVE frame
V0 = 5.56                   # the 20 km/h cruise
STANDSTILL = 0.05           # the pure plant reports standstill below this
PLAN_ID = "KCS1"            # these tests pin the KCS1 table (the module may run a later block; KCS2 has its own tests)
MANEUVERS = BLOCKS[PLAN_ID]
V_INTENT = INTENT_V[PLAN_ID]
IDS = tuple(m[0] for m in MANEUVERS)
SEGS = {m[0]: m[2] for m in MANEUVERS}
TEXT = {m[0]: m[1] for m in MANEUVERS}
ZERO = dict.fromkeys(IDS, 0)
HOLD_N = round(HOLD_MIN_S / DT)   # held frames a counted hold needs before the brake
GATE = SEGS["B"][0].accel         # the first command of the first maneuver: the start gate of a fresh hook


@pytest.fixture(autouse=True)
def _no_auto_start(monkeypatch):
  """the press-driven tests: READY waits for a press (the auto-start tests set the real AUTO_START_S back); the KCS1 table"""
  monkeypatch.setattr(ih, "AUTO_START_S", math.inf)
  monkeypatch.setattr(ih, "PLAN_ID", PLAN_ID)
  monkeypatch.setattr(ih, "MANEUVERS", MANEUVERS)


def good(**kw) -> HookInputs:
  """READY-capable inputs: a steady 20 km/h cruise at the set speed, no lead, no demand, button released."""
  base = HookInputs(valid=True, santa_fe=True, long_active=True, enabled=True, pid_state=True, v_ego=V0, a_ego=0.0, v_cruise=V0, gas=False,
                    brake=False, force_coast=False, pause_long=False, standstill=False, steer_deg=1.0, yaw_rate=0.01, blinker=False,
                    steer_fault=False, esp_active=False, acc_faulted=False, can_valid=True, gear_drive=True, stock_aeb=False, stock_fcw=False,
                    lead_status=False, radar_error=False, lead_prob=0.02, plan_has_lead=False, plan_should_stop=False, plan_fcw=False,
                    stop_target_m=-1.0, plan_accel=0.0, distance_pressed=False, distance_long=False, mapping_ok=True)
  return replace(base, **kw)


def run(hook, inputs_fn, n, normal=0.0, dt=DT):
  return [hook.update(inputs_fn(k), normal, dt) for k in range(n)]


def feed(hook, vs, normal=0.0, **kw):
  """one frame per speed (standstill below STANDSTILL)"""
  return [hook.update(good(v_ego=v, standstill=v < STANDSTILL, **kw), normal) for v in vs]


def press(hook, frames, normal=0.0, **kw):
  """press for `frames`, then release until the press ends; returns every output"""
  return (run(hook, lambda k: good(distance_pressed=True, **kw), frames, normal) +
          run(hook, lambda k: good(**kw), SETTLE, normal))


def plan(man):
  """a progress record whose next maneuver is `man` (every other maneuver one rep ahead)"""
  return {"plan": PLAN_ID, "done": {m: int(m != man) for m in IDS}}


def armed(hook=None, **kw):
  hook = hook or IdentificationHook()
  if hook.state == "OFF":
    run(hook, lambda k: good(**kw), SETTLE)   # a release must be observed first
    press(hook, LONG, **kw)
  assert hook.state in ("ARMED", "READY")
  return hook


def ready(hook=None, **kw):
  hook = armed(hook, **kw)
  run(hook, lambda k: good(**kw), round(PRECONDITION_S / DT) + 5)
  assert hook.state == "READY"
  return hook


def start(hook=None, man=None, **kw):
  """READY, a short press and its release -> (hook, the first ACTIVE frame's output); `man` loads a record making it next"""
  hook = hook or IdentificationHook()
  if man is not None:
    hook.load(plan(man))
  hook = ready(hook, **kw)
  o = press(hook, SHORT, **kw)[-1]
  assert hook.state == o.state == "ACTIVE" and o.changed
  return hook, o


def to_stop(hook, o, v=V0, normal=0.0, **kw):
  """From an ACTIVE frame (output o, fed speed v): a delay-free plant (the car follows the floor exactly) until the rep
  leaves ACTIVE. Returns every output and the speed fed on its frame, the given frame first."""
  outs, vs = [o], [v]
  while hook.state == "ACTIVE" and len(outs) < 6000:
    v = max(0.0, v + outs[-1].floor * DT)
    outs.append(hook.update(good(v_ego=v, standstill=v < STANDSTILL, **kw), normal))
    vs.append(v)
  return outs, vs


def hold(hook, n, normal=0.0, **kw):
  return run(hook, lambda k: good(v_ego=0.0, standstill=True, **kw), n, normal)


def brake(hook, **kw):
  """the driver's brake at rest (openpilot disengages on the same frame)"""
  return hook.update(good(v_ego=0.0, standstill=True, brake=True, enabled=False, long_active=False, **kw), 0.0)


def full_rep(hook, man=None, held=HOLD_N):
  hook, o = start(hook, man)
  outs, _ = to_stop(hook, o)
  return outs + hold(hook, held) + [brake(hook)]


def _boom(*a, **k):
  raise RuntimeError("boom")


# -- preconditions ----------------------------------------------------------------------------------
@pytest.mark.parametrize("kw,reason", [
  (dict(valid=False), "inputs"), (dict(v_ego=math.nan), "inputs"), (dict(a_ego=math.inf), "inputs"), (dict(v_cruise=math.nan), "inputs"),
  (dict(plan_accel=math.nan), "inputs"), (dict(lead_prob=math.nan), "inputs"), (dict(stop_target_m=math.nan), "inputs"),
  (dict(steer_deg=math.inf), "inputs"), (dict(yaw_rate=math.nan), "inputs"), (dict(santa_fe=False), "car"), (dict(mapping_ok=False), "mapping"),
  (dict(enabled=False), "disengaged"), (dict(long_active=False), "disengaged"), (dict(pid_state=False), "state"), (dict(standstill=True), "state"),
  (dict(gas=True), "pedal"), (dict(brake=True), "pedal"), (dict(force_coast=True), "pedal"), (dict(pause_long=True), "pedal"),
  (dict(v_ego=3.49, v_cruise=3.49), "speed"), (dict(v_ego=9.01, v_cruise=9.01), "speed"), (dict(lead_status=True), "lead"),
  (dict(lead_prob=0.10), "lead"), (dict(plan_has_lead=True), "lead"), (dict(radar_error=True), "fcw"), (dict(stock_aeb=True), "fcw"),
  (dict(stock_fcw=True), "fcw"), (dict(plan_fcw=True), "fcw"), (dict(plan_should_stop=True), "stop"), (dict(stop_target_m=40.0), "stop"),
  (dict(stop_target_m=0.0), "stop"), (dict(plan_accel=GATE - 0.01), "demand"), (dict(steer_deg=6.0), "steer"), (dict(yaw_rate=-0.05), "steer"),
  (dict(blinker=True), "steer"), (dict(steer_fault=True), "steer"), (dict(esp_active=True), "vehicle"), (dict(acc_faulted=True), "vehicle"),
  (dict(can_valid=False), "vehicle"), (dict(gear_drive=False), "vehicle"), (dict(v_ego=V0 + 0.31), "settling"), (dict(v_cruise=V0 - 0.31), "settling"),
  (dict(a_ego=0.21), "settling"), (dict(a_ego=-0.21), "settling"), (dict(plan_accel=0.16), "settling"), (dict(plan_accel=-0.16), "settling"),
])
def test_every_start_condition_has_a_reason(kw, reason):
  assert precondition_failure(good(), 0.0, GATE) is None
  assert precondition_failure(good(**kw), 0.0, GATE) == reason


@pytest.mark.parametrize("kw", [dict(v_ego=3.5, v_cruise=3.5), dict(v_ego=9.0, v_cruise=9.0), dict(v_ego=V0 + 0.29), dict(a_ego=-0.2),
                                dict(plan_accel=0.15), dict(plan_accel=-0.15), dict(lead_prob=0.099), dict(stop_target_m=200.0),
                                dict(steer_deg=-5.0), dict(yaw_rate=0.03)])
def test_the_start_gate_boundaries_pass(kw):
  assert precondition_failure(good(**kw), 0.0, GATE) is None


@pytest.mark.parametrize("kw,reason", [(dict(lead_status=True, plan_fcw=True), "fcw"), (dict(gas=True, acc_faulted=True), "vehicle"),
                                       (dict(enabled=False, radar_error=True), "fcw"), (dict(lead_prob=0.5, valid=False), "inputs"),
                                       (dict(v_ego=20.0, stock_aeb=True), "fcw"), (dict(plan_accel=-2.0, lead_status=True), "lead")])
@pytest.mark.parametrize("gate", [GATE, None], ids=["start", "rep"])
def test_a_fault_reports_before_an_ordinary_reason(kw, reason, gate):
  assert precondition_failure(good(**kw), 0.0, gate) == reason


@pytest.mark.parametrize("man", IDS)
def test_the_start_gate_rejects_a_demand_deeper_than_the_first_command(man):
  first = SEGS[man][0].accel
  assert precondition_failure(good(), first, first) is None                  # the normal chain exactly at the first command
  assert precondition_failure(good(), first - 0.01, first) == "demand"
  assert precondition_failure(good(plan_accel=first - 0.01), 0.0, first) == "demand"   # reported before the settling check
  assert precondition_failure(good(), math.nan, first) == "inputs"


def test_the_hook_gates_on_the_next_maneuvers_first_command():
  a_next = IdentificationHook()
  a_next.load(plan("A"))                                          # A starts at -0.5
  armed(a_next)
  outs = run(a_next, lambda k: good(), 300, normal=-0.6)
  assert a_next.state == "ARMED" and outs[-1].text1 == "TEST ARMED - waiting: demand"
  b_next = armed()                                                # B starts at -1.0
  run(b_next, lambda k: good(), 300, normal=-0.6)
  assert b_next.state == "READY"


def test_a_running_rep_has_no_start_band_and_the_intent_expects_the_stopping_state():
  for kw in (dict(v_ego=1.0), dict(v_ego=12.0), dict(a_ego=-1.5), dict(plan_accel=-2.0), dict(v_cruise=9.0)):
    assert precondition_failure(good(**kw), -3.0) is None       # no band, demand or settling check during a rep
  for kw, reason in ((dict(pid_state=False), "state"), (dict(standstill=True), "state"), (dict(plan_should_stop=True), "stop"),
                     (dict(stop_target_m=5.0), "stop")):
    assert precondition_failure(good(**kw), 0.0) == reason
    assert precondition_failure(good(**kw), 0.0, intent=True) is None
  for kw, reason in ((dict(lead_status=True), "lead"), (dict(plan_fcw=True), "fcw"), (dict(brake=True), "pedal"), (dict(enabled=False), "disengaged"),
                     (dict(steer_deg=8.0), "steer"), (dict(valid=False), "inputs")):
    assert precondition_failure(good(**kw), 0.0, intent=True) == reason   # the intent never relaxes these


# -- arming, disarming, restart -----------------------------------------------------------------------
def test_a_new_instance_is_off_and_short_presses_never_act():
  hook = IdentificationHook()
  outs = run(hook, lambda k: good(distance_pressed=k % 40 < SHORT), 2000)
  assert hook.state == "OFF" and not any(o.floor is not None or o.stop_intent or o.text1 or o.text2 for o in outs)


def test_a_fresh_long_press_arms_at_its_threshold_and_the_rest_of_it_does_nothing():
  hook = IdentificationHook()
  run(hook, lambda k: good(), SETTLE)
  outs = run(hook, lambda k: good(distance_pressed=True), 300)   # held 3 s: arms at 0.5 s, then READY while still held
  arm_at = next(k for k, o in enumerate(outs) if o.state != "OFF")
  assert arm_at == 49 and outs[arm_at].changed and outs[arm_at].state == "ARMED"
  assert (outs[arm_at].text1, outs[arm_at].text2) == ("TEST ARMED - waiting: settling", "next B 1/6: -1.0 to stop; long press = off")
  assert outs[-1].state == "READY" and all(o.floor is None for o in outs)
  outs = run(hook, lambda k: good(), 50)                         # its release neither starts a rep nor disarms
  assert hook.state == "READY" and all(o.floor is None for o in outs) and hook.done == ZERO


def test_arming_applies_no_command_and_needs_no_engagement():
  hook = IdentificationHook()
  parked = dict(v_ego=0.0, standstill=True, long_active=False, enabled=False, gear_drive=False, pid_state=False)
  run(hook, lambda k: good(**parked), SETTLE)
  outs = press(hook, LONG, **parked)
  assert hook.state == "ARMED" and all(o.floor is None and not o.stop_intent for o in outs)
  assert outs[-1].text1 == "TEST ARMED - waiting: vehicle"


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


@pytest.mark.parametrize("kw", [dict(mapping_ok=False), dict(santa_fe=False)])
def test_arming_needs_the_test_scope(kw):
  hook = IdentificationHook()
  run(hook, lambda k: good(**kw), SETTLE)
  press(hook, LONG, **kw)
  assert hook.state == "OFF"


@pytest.mark.parametrize("from_ready", [False, True])
def test_a_long_press_while_armed_turns_test_mode_off(from_ready):
  hook = ready() if from_ready else armed()
  outs = press(hook, LONG)
  assert hook.state == "OFF" and all(o.floor is None for o in outs) and hook.done == ZERO
  assert (outs[-1].text1, outs[-1].text2) == ("TEST MODE OFF", "long press distance to arm")
  outs = run(hook, lambda k: good(), round(NOTICE_S / DT) + 10)   # the notice clears; nothing is shown while OFF
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
  assert all(o.floor is None for o in outs) and hook.state == "ARMED"
  run(hook, lambda k: good(), 300)                               # READY now: the early press was discarded, not queued
  assert hook.state == "READY" and hook.update(good(), 0.0).floor is None
  outs = run(hook, lambda k: good(distance_pressed=True), SHORT) + run(hook, lambda k: good(), SETTLE - 1)
  assert all(o.floor is None for o in outs)
  assert outs[-1].text1.startswith("TEST B 1/6 STARTS IN ") and outs[-1].text2 == "B: -1.0 to stop; brake = not now"   # no auto-start here
  o = hook.update(good(), 0.0)                                   # the press ends after 50 ms of release: start
  assert o.state == hook.state == "ACTIVE" and o.changed and o.floor == -1.0 and o.own and not o.stop_intent and o.rep_done == ""
  assert (o.maneuver, o.rep, o.seg) == ("B", 1, 1) and hook.done == ZERO
  assert (o.text1, o.text2) == ("TEST B 1/6 s1 -1.00", "press = cancel (releases, cruise resumes)")


@pytest.mark.parametrize("settled,starts", [(199, False), (200, True)])
def test_readiness_is_judged_before_the_press_frame_is_credited(settled, starts):
  hook = armed()
  run(hook, lambda k: good(), settled - round(hook._pre_t / DT))   # arming already credited its own frames
  assert any(o.state == "ACTIVE" for o in press(hook, SHORT)) == starts


@pytest.mark.parametrize("frames,starts", [(1, False), (4, False), (5, True), (49, True), (50, False)])
def test_observed_press_duration_rejects_glitches_and_long_presses(frames, starts):
  hook = ready()
  outs = press(hook, frames)
  assert (outs[-1].state == "ACTIVE") == starts
  if frames >= 50:
    assert hook.state == "OFF"                                   # a long press disarms instead


@pytest.mark.parametrize("long_from", [SHORT - 1, SHORT])        # the card flags it while held, or only on the release frame
def test_the_card_long_flag_discards_a_start(long_from):
  hook = ready()
  run(hook, lambda k: good(distance_pressed=True, distance_long=k >= long_from), SHORT)
  outs = run(hook, lambda k: good(distance_long=long_from == SHORT and k == 0), SETTLE)
  assert all(o.floor is None for o in outs) and hook.state == "READY" and hook.done == ZERO
  assert start(hook)[1].floor == -1.0


def test_a_release_dropout_inside_a_hold_is_not_a_release():
  hook = ready()
  outs = run(hook, lambda k: good(distance_pressed=True), 10)
  for gap in range(1, SETTLE):                                   # 10-40 ms dropouts inside one hold
    outs += run(hook, lambda k: good(), gap) + run(hook, lambda k: good(distance_pressed=True), 10)
  outs += run(hook, lambda k: good(distance_pressed=True), 10)
  assert all(o.floor is None for o in outs) and hook.state == "OFF" and hook.done == ZERO   # it stayed one long press


@pytest.mark.parametrize("dropout,after", [(1, 25), (4, 22), (4, 24)])
def test_a_dropout_counts_toward_the_hold_length(dropout, after):
  # review reproduction: 24 pressed + 4 released + 24 pressed = a 0.52 s hold with only 48 pressed frames
  hook = ready()
  outs = (run(hook, lambda k: good(distance_pressed=True), 24) + run(hook, lambda k: good(), dropout) +
          run(hook, lambda k: good(distance_pressed=True), after) + run(hook, lambda k: good(), SETTLE))
  assert all(o.floor is None for o in outs) and hook.state == "OFF" and hook.done == ZERO


@pytest.mark.parametrize("kw", [dict(lead_status=True), dict(gas=True), dict(valid=False), dict(mapping_ok=False), dict(plan_accel=GATE - 0.1),
                                dict(a_ego=0.3)], ids=["lead", "gas", "invalid", "mapping", "demand", "settling"])
@pytest.mark.parametrize("when", ["held", "release"])
def test_a_failed_precondition_during_the_press_needs_a_fresh_press(kw, when):
  hook = ready()
  run(hook, lambda k: good(distance_pressed=True), 3)
  if when == "held":
    outs = [hook.update(good(distance_pressed=True, **kw), 0.0)] + press(hook, 3)
  else:
    outs = run(hook, lambda k: good(**kw) if k == 2 else good(), SETTLE)
  assert all(o.floor is None for o in outs) and hook.state != "ACTIVE"
  assert start(hook)[1].maneuver == "B"


# -- the READY auto-start countdown (the real AUTO_START_S) ------------------------------------------------
AUTO_N = round((PRECONDITION_S + AUTO_START_S) / DT)   # good frames from the first one through the auto-start frame


def countdown(monkeypatch):
  """the real AUTO_START_S; ARMED with the start conditions failed on the last frame (the countdown restarts)"""
  monkeypatch.setattr(ih, "AUTO_START_S", AUTO_START_S)
  hook = armed()
  assert hook.update(good(a_ego=0.3), 0.0).state == "ARMED"
  return hook


def first_active(outs):
  return next((k for k, x in enumerate(outs) if x.state == "ACTIVE"), None)


def test_ready_starts_the_shown_maneuver_by_itself_like_a_short_press(monkeypatch):
  hook = countdown(monkeypatch)
  outs = run(hook, lambda k: good(), AUTO_N)
  assert first_active(outs) == AUTO_N - 1 and all(x.floor is None for x in outs[:-1])   # PRECONDITION_S + AUTO_START_S, no press
  assert [x.state for x in outs[:-1]] == ["ARMED"] * (round(PRECONDITION_S / DT) - 1) + ["READY"] * round(AUTO_START_S / DT)
  pressed = start()[1]
  fields = lambda x: (x.state, x.changed, x.floor, x.own, x.stop_intent, x.maneuver, x.rep, x.seg, x.rep_done, x.text1, x.text2)  # noqa: E731
  assert fields(outs[-1]) == fields(pressed) == ("ACTIVE", True, -1.0, True, False, "B", 1, 1, "", "TEST B 1/6 s1 -1.00",
                                                 "press = cancel (releases, cruise resumes)")
  held = to_stop(hook, outs[-1])[0][-1]
  assert held.state == "HELD" and all(x.own for x in hold(hook, HOLD_N))
  assert brake(hook).rep_done == "B" and hook.done == {**ZERO, "B": 1}


def test_the_ready_banner_counts_down_to_the_start(monkeypatch):
  hook = countdown(monkeypatch)
  outs = run(hook, lambda k: good(), AUTO_N)
  ready_at = round(PRECONDITION_S / DT) - 1
  assert outs[ready_at - 1].state == "ARMED" and outs[ready_at - 1].text1 == "TEST ARMED - waiting: settling"
  shown = {k: outs[k].text1 for k in (ready_at, ready_at + 50, ready_at + 100, ready_at + 150, AUTO_N - 2)}
  assert shown == {ready_at: "TEST B 1/6 STARTS IN 2.0 s", ready_at + 50: "TEST B 1/6 STARTS IN 1.5 s", ready_at + 100: "TEST B 1/6 STARTS IN 1.0 s",
                   ready_at + 150: "TEST B 1/6 STARTS IN 0.5 s", AUTO_N - 2: "TEST B 1/6 STARTS IN 0.0 s"}
  assert all(x.text2 == "B: -1.0 to stop; brake = not now" for x in outs[ready_at:-1]) and outs[-1].state == "ACTIVE"


@pytest.mark.parametrize("kw", [dict(lead_status=True), dict(a_ego=0.3), dict(plan_accel=GATE - 0.1), dict(steer_deg=8.0),
                                dict(brake=True, enabled=False, long_active=False)], ids=["lead", "settling", "demand", "steer", "brake"])
@pytest.mark.parametrize("left", [1.5, 0.01])                    # seconds of the countdown left when the condition fails
def test_a_failed_condition_during_the_countdown_restarts_it(monkeypatch, kw, left):
  hook = countdown(monkeypatch)
  outs = run(hook, lambda k: good(), AUTO_N - round(left / DT))
  assert outs[-1].state == "READY" and first_active(outs) is None
  o = hook.update(good(**kw), 0.0)
  assert o.state == hook.state == "ARMED" and o.floor is None
  outs = run(hook, lambda k: good(), AUTO_N)
  assert first_active(outs) == AUTO_N - 1                        # the whole PRECONDITION_S + AUTO_START_S again


@pytest.mark.parametrize("frames", [SHORT, 45])
def test_a_held_button_pauses_the_countdown_until_the_press_ends(monkeypatch, frames):
  hook = countdown(monkeypatch)
  run(hook, lambda k: good(), AUTO_N - 10)                       # READY, 0.1 s left
  outs = press(hook, frames)                                     # held past the countdown's end
  assert first_active(outs) == len(outs) - 1 and outs[-1].floor == -1.0 and outs[-1].own   # only on the press end (release dwell)
  assert all(x.state == "READY" and x.floor is None for x in outs[:-1])


def test_a_long_press_during_the_countdown_still_turns_test_mode_off(monkeypatch):
  hook = countdown(monkeypatch)
  run(hook, lambda k: good(), AUTO_N - 10)
  outs = press(hook, LONG) + run(hook, lambda k: good(), 2 * AUTO_N)
  assert first_active(outs) is None and all(x.floor is None for x in outs) and hook.state == "OFF" and hook.done == ZERO
  assert outs[48].state == "READY" and outs[49].state == "OFF"   # held 0.39 s past the countdown's end, then off at 0.5 s


def test_after_a_counted_rep_and_resume_the_next_maneuver_starts_by_itself(monkeypatch):
  hook = countdown(monkeypatch)
  hook, o = start(hook)
  to_stop(hook, o)
  hold(hook, HOLD_N)
  assert brake(hook).rep_done == "B"
  outs = run(hook, lambda k: good(v_ego=0.0, standstill=True, brake=True, enabled=False, long_active=False), 500)   # parked, brake held
  outs += run(hook, lambda k: good(v_ego=0.0, standstill=True), 100)                    # RESUME at rest
  outs += run(hook, lambda k: good(v_ego=V0 * (k + 1) / 300, a_ego=1.0, v_cruise=V0), 300)   # accelerating back to the set speed
  assert all(x.state == "ARMED" and x.floor is None for x in outs)
  outs = run(hook, lambda k: good(), AUTO_N)                     # steady cruise again, no press
  assert first_active(outs) == AUTO_N - 1 and (outs[-1].maneuver, outs[-1].rep, outs[-1].floor, outs[-1].own) == ("A", 1, -0.5, True)


# -- KCS1: the maneuver order and the block --------------------------------------------------------------
@pytest.mark.parametrize("done,nxt", [({}, "B"), ({"B": 1}, "A"), ({"B": 1, "A": 1}, "C"), ({"B": 1, "A": 1, "C": 1}, "D"),
                                      ({"B": 1, "A": 1, "C": 1, "D": 1}, "E"), (dict.fromkeys(IDS, 1), "B"),
                                      ({"B": 2, "A": 1, "C": 1, "D": 1, "E": 1}, "A"), ({"B": 6, "A": 6, "C": 5, "D": 5, "E": 6}, "C"),
                                      ({"B": 3, "A": 2, "C": 4, "D": 2, "E": 2}, "A"), ({"B": 6, "A": 6, "C": 6, "D": 6, "E": 5}, "E")])
def test_the_next_maneuver_is_the_fewest_done_then_table_order(done, nxt):
  hook = IdentificationHook()
  hook.load({"plan": PLAN_ID, "done": done})
  n = hook.done[nxt] + 1
  run(hook, lambda k: good(), SETTLE)
  o = press(hook, LONG)[49]                                      # the arming frame (the 50th pressed frame)
  assert o.text2 == f"next {nxt} {n}/{N_REPS}: {TEXT[nxt]}; long press = off"
  hook, o = start(hook)
  assert (o.maneuver, o.rep, o.seg, o.floor) == (nxt, n, 1, SEGS[nxt][0].accel)
  assert o.text1 == f"TEST {nxt} {n}/{N_REPS} s1 {SEGS[nxt][0].accel:+.2f}"


def test_block_complete_after_n_reps_of_every_maneuver():
  hook = IdentificationHook()
  hook.load({"plan": PLAN_ID, "done": dict.fromkeys(IDS, N_REPS)})
  run(hook, lambda k: good(), SETTLE)
  outs = press(hook, LONG) + run(hook, lambda k: good(), 300) + press(hook, SHORT) + run(hook, lambda k: good(), 50)
  assert hook.state == "ARMED" and all(o.floor is None and o.state in ("OFF", "ARMED") for o in outs)
  assert (outs[-1].text1, outs[-1].text2) == ("TEST BLOCK COMPLETE - long press = off", f"plan {PLAN_ID}: {N_REPS} reps of every maneuver")
  press(hook, LONG)
  assert hook.state == "OFF"


def test_a_whole_block_runs_the_maneuvers_in_rounds_then_completes():
  hook, order = IdentificationHook(), []
  for _ in range(N_REPS * len(IDS)):
    outs = full_rep(hook)
    order += [o.rep_done for o in outs if o.rep_done]
    assert hook.progress()["done"][order[-1]] == order.count(order[-1])      # the count grows once per rep_done
  assert order == list(IDS) * N_REPS and hook.done == dict.fromkeys(IDS, N_REPS)
  assert (outs[-1].text1, outs[-1].text2) == (f"E {N_REPS}/{N_REPS} DONE", "block complete")
  o = run(hook, lambda k: good(), 400)[-1]
  assert o.state == "ARMED" and o.text1 == "TEST BLOCK COMPLETE - long press = off"


# -- KCS1: the segment walk, the stop intent and `own` ------------------------------------------------------
@pytest.mark.parametrize("man", IDS)
def test_every_maneuver_walks_its_segments_and_asserts_the_stop_intent(man):
  hook, o = start(man=man)
  outs, vs = to_stop(hook, o)
  segs, active = SEGS[man], outs[:-1]
  assert outs[-1].state == "HELD" and all(x.state == "ACTIVE" for x in active)
  assert all(x.floor == segs[x.seg - 1].accel and x.floor <= 0.0 for x in active)   # exactly the segment's command, never positive
  seg_of = [x.seg for x in active]
  assert seg_of == sorted(seg_of) and sorted(set(seg_of)) == list(range(1, len(segs) + 1))
  for j, s in enumerate(segs[:-1]):
    enter, leave = seg_of.index(j + 1), seg_of.index(j + 2)
    assert outs[leave].changed
    if s.v_end is not None:                                      # speed trigger: the first frame at or below v_end
      assert vs[leave] <= s.v_end and all(v > s.v_end for v in vs[enter:leave])
    else:                                                        # time trigger: exactly t_s
      assert leave - enter == round(s.t_s / DT)
  # the intent: on the first frame at or below V_INTENT in a speed/standstill-ended segment, then sticky
  eligible = [k for k, x in enumerate(active) if vs[k] <= V_INTENT and segs[x.seg - 1].t_s is None]
  first = next(k for k, x in enumerate(outs) if x.stop_intent)
  assert first == eligible[0] and all(x.stop_intent for x in outs[first:])
  assert all(x.own for x in outs)                               # a normal rep owns the wire on every frame, the first HELD frame too
  assert not hook._stalled


def test_the_intent_never_asserts_in_a_time_segment():
  hook, _ = start(man="D")
  outs = feed(hook, [3.0] * 10 + [2.5] + [1.9] * 200)            # D's 0.0 coast from 2.5 m/s, below V_INTENT throughout
  seg2 = outs[10:210]
  assert all(x.seg == 2 and x.floor == 0.0 and not x.stop_intent and x.own for x in seg2)   # the 0.0 coast is owned too
  assert (outs[210].seg, outs[210].floor, outs[210].stop_intent, outs[210].own) == (3, -0.8, True, True)   # seg 3's first frame


def test_the_intent_stays_through_a_later_time_segment():
  hook, _ = start(man="E")
  outs = feed(hook, [2.0, 1.5] + [1.4] * 200)
  assert (outs[0].seg, outs[0].floor, outs[0].stop_intent, outs[0].own) == (1, -0.8, True, True)   # E's intent is in seg 1
  assert all(x.seg == 2 and x.floor == -0.3 and x.stop_intent and x.own for x in outs[1:201])
  assert (outs[201].seg, outs[201].floor, outs[201].own) == (3, -0.8, True)


def test_the_over_speed_guard_is_measured_from_each_segments_first_frame():
  hook, _ = start(man="C")
  outs = feed(hook, [3.0, 3.49, 2.5, 2.4, 2.89])                 # seg 1 from 3.0; seg 2's first frame at 2.4
  assert all(x.state == "ACTIVE" for x in outs) and outs[-1].seg == 2
  o = hook.update(good(v_ego=2.4 + V_OVER + 0.01), 0.0)
  assert o.state == "HANDBACK" and o.reason == "speed"


# -- KCS1: the held stop -----------------------------------------------------------------------------
@pytest.mark.parametrize("man", IDS)
def test_standstill_holds_and_the_floor_deepens_at_j_hold_to_the_secure_hold(man):
  hook, o = start(man=man)
  held = to_stop(hook, o)[0][-1]
  last = SEGS[man][-1].accel
  assert held.state == "HELD" and held.changed and held.stop_intent and held.own and held.floor == last
  assert (held.text1, held.text2) == (f"TEST {man} 1/{N_REPS} STOPPED - hold 0.0 s", "brake to finish")
  outs = hold(hook, 500)
  assert all(x.own and x.stop_intent for x in outs)             # the hold build is the wire
  floors = [x.floor for x in outs]
  expected = [max(last - J_HOLD * DT * (k + 1), A_HOLD) if last > A_HOLD else last for k in range(500)]
  assert floors == pytest.approx(expected, abs=1e-9) and floors[-1] == min(last, A_HOLD)
  assert all(b <= a for a, b in zip([last] + floors[:-1], floors, strict=True)) and hook.state == "HELD"


@pytest.mark.parametrize("held,counted", [(HOLD_N - 1, False), (HOLD_N, True), (1000, True)])
def test_the_brake_ends_the_hold_and_counts_only_after_the_minimum_hold(held, counted):
  hook, o = start()
  to_stop(hook, o)
  hold(hook, held)
  o = brake(hook)
  assert o.floor is None and not o.stop_intent and not o.own and o.changed and o.state == hook.state == "ARMED" and (o.maneuver, o.rep) == ("B", 1)
  if counted:
    assert o.rep_done == "B" and o.reason == "complete" and hook.done == {**ZERO, "B": 1} and hook._last == "last: B 1/6 done"
    assert (o.text1, o.text2) == ("B 1/6 DONE", "next A 1/6: -0.5 to stop")
  else:
    assert o.rep_done == "" and o.reason == "short-hold" and hook.done == ZERO
    assert (o.text1, o.text2) == ("B 1/6 NOT COUNTED - short-hold", "next B 1/6: -1.0 to stop")
  after = run(hook, lambda k: good(v_ego=0.0, standstill=True, brake=True, enabled=False, long_active=False), 300)
  assert all(x.rep_done == "" and x.floor is None for x in after) and hook.state == "ARMED"   # rep_done exactly once
  assert start(hook)[1].maneuver == ("A" if counted else "B")    # the rest advances the maneuver only when counted


def test_the_rep_result_stays_on_screen_until_ready():
  hook, o = start()
  to_stop(hook, o)
  hold(hook, HOLD_N)
  brake(hook)
  outs = run(hook, lambda k: good(v_ego=0.0, standstill=True, brake=True, enabled=False, long_active=False), round(NOTICE_S / DT) + 10)
  shown = list(takewhile(lambda x: x.text1 == "B 1/6 DONE", outs))
  assert abs(len(shown) * DT - NOTICE_S) <= 2 * DT and all(x.text2 == "next A 1/6: -0.5 to stop" for x in shown)
  assert outs[-1].text1 == "TEST ARMED - waiting: disengaged"
  hook, o = start(IdentificationHook())                          # READY replaces the notice at once
  to_stop(hook, o)
  hold(hook, HOLD_N)
  brake(hook)
  outs = run(hook, lambda k: good(), 250)
  ready_at = next(k for k, x in enumerate(outs) if x.state == "READY")
  assert all(x.text1 == "B 1/6 DONE" for x in outs[:ready_at]) and outs[ready_at].text1.startswith("TEST A 1/6 STARTS IN ")


def test_the_hold_banner_asks_for_the_brake_after_hold_brake_s():
  hook, o = start()
  to_stop(hook, o)
  n = round(HOLD_BRAKE_S / DT) + 100
  outs = hold(hook, n)
  first = next(k for k, x in enumerate(outs) if "BRAKE NOW" in x.text1)
  assert abs((first + 1) * DT - HOLD_BRAKE_S) <= DT + 1e-9
  assert outs[99].text1 == "TEST B 1/6 STOPPED - hold 1.0 s" and outs[-1].text1 == f"TEST B 1/6 STOPPED - hold {n * DT:.1f} s - BRAKE NOW"
  assert all(x.text2 == "brake to finish" for x in outs)


@pytest.mark.parametrize("kw,reason", [(dict(gas=True), "pedal"), (dict(gas=True, brake=True), "pedal"), (dict(enabled=False, long_active=False), "disengaged"),
                                       (dict(long_active=False), "disengaged")])
def test_gas_or_a_cancel_in_the_hold_turns_test_mode_off_uncounted(kw, reason):
  hook, o = start()
  to_stop(hook, o)
  hold(hook, 200)
  o = hook.update(good(v_ego=0.0, standstill=True, **kw), 0.0)
  assert o.state == hook.state == "OFF" and o.floor is None and not o.stop_intent and o.reason == reason and o.rep_done == "" and hook.done == ZERO
  assert (o.text1, o.text2) == (f"TEST B 1/6 ABORTED - {reason}", "test mode off; long press distance to arm")
  assert hook._last == f"last: B 1/6 aborted - {reason}"


def test_presses_in_the_hold_are_ignored():
  hook, o = start()
  to_stop(hook, o)
  outs = (hold(hook, 20) + hold(hook, 200, distance_pressed=True) + hold(hook, SETTLE) + hold(hook, SHORT, distance_pressed=True) +
          hold(hook, 20) + hold(hook, 30, distance_pressed=True))
  assert all(x.state == "HELD" and x.stop_intent for x in outs)
  assert all(b.floor <= a.floor for a, b in zip(outs, outs[1:], strict=False))
  o = brake(hook, distance_pressed=True)                         # braked with the button still held
  assert o.rep_done == "B"
  outs = hold(hook, 100, distance_pressed=True, brake=True) + hold(hook, SETTLE, brake=True)
  assert hook.state == "ARMED" and all(x.state == "ARMED" and x.floor is None for x in outs)   # the rest of that press does nothing


# -- KCS1: aborts before and after the intent --------------------------------------------------------------
@pytest.mark.parametrize("kw,reason", [(dict(lead_status=True), "lead"), (dict(lead_prob=0.3), "lead"), (dict(steer_deg=8.0), "steer"),
                                       (dict(blinker=True), "steer"), (dict(v_ego=V0 + V_OVER + 0.01), "speed"), (dict(distance_pressed=True), "press"),
                                       (dict(plan_should_stop=True), "stop"), (dict(stop_target_m=30.0), "stop"), (dict(pid_state=False), "state")])
def test_an_abort_before_the_intent_hands_back_with_the_bounded_release(kw, reason):
  hook, _ = start()
  assert all(x.own for x in feed(hook, [V0] * 50))
  o = hook.update(good(**kw), 0.0)
  assert o.state == hook.state == "HANDBACK" and o.changed and o.reason == reason and o.floor == -1.0 and not o.own and not o.stop_intent
  assert (o.text1, o.text2) == (f"TEST B 1/6 ABORTED - {reason}", "releasing; cruise resumes and can accelerate")
  outs = run(hook, lambda k: good(), 200)
  floors = [x.floor for x in takewhile(lambda x: x.floor is not None, outs)]
  assert floors == pytest.approx([min(-1.0 + RELEASE_JERK * DT * (k + 1), 0.0) for k in range(len(floors))], abs=1e-9)
  assert abs(len(floors) - 1.0 / (RELEASE_JERK * DT)) <= 1 and not any(x.stop_intent or x.own for x in outs)
  assert hook.state == "ARMED" and hook.done == ZERO and hook._last == f"last: B 1/6 aborted - {reason}"
  assert start(hook)[1].maneuver == "B"                          # not counted: the same maneuver again


def test_a_cancel_press_acts_on_its_first_frame_and_does_nothing_else():
  hook, _ = start()
  feed(hook, [V0] * 30)
  o = hook.update(good(distance_pressed=True), 0.0)
  assert o.state == "HANDBACK" and o.reason == "press" and o.floor == -1.0
  outs = run(hook, lambda k: good(distance_pressed=True), 300)   # held 3 s more: no disarm, re-arm or start
  outs += run(hook, lambda k: good(), SETTLE)
  assert hook.state in ("ARMED", "READY") and not {"ACTIVE", "OFF"} & {x.state for x in outs}
  assert hook._last == "last: B 1/6 aborted - press" and hook.done == ZERO
  assert start(hook)[1].maneuver == "B"                          # only a fresh press starts the next rep


def test_a_double_tap_starts_and_cancels():
  hook = ready()
  press(hook, SHORT)
  o = hook.update(good(distance_pressed=True), 0.0)
  assert o.state == "HANDBACK" and o.reason == "press" and hook.done == ZERO


@pytest.mark.parametrize("kw,reason,locked", [(dict(lead_status=True), "lead", False), (dict(steer_deg=8.0), "steer", False),
                                              (dict(distance_pressed=True), "press", False), (dict(v_ego=V0 + V_OVER + 0.01), "speed", False),
                                              (dict(plan_fcw=True), "fcw", True), (dict(acc_faulted=True), "vehicle", True),
                                              (dict(valid=False), "inputs", True)])
def test_an_abort_after_the_intent_finishes_the_stop_uncounted(kw, reason, locked):
  hook, _ = start(man="E")
  assert feed(hook, [V0, 1.9])[-1].own                           # E's intent in seg 1 (-0.8 to 1.5 m/s)
  o = hook.update(good(**{"v_ego": 1.8, **kw}), 0.0)
  assert o.state == hook.state == "ACTIVE" and o.floor == -0.8 and o.stop_intent and not o.own and o.reason == reason
  assert (o.text1, o.text2) == (f"TEST E 1/6 ABORTED - {reason}", "finishing the stop; brake to end")
  rest = {k: v for k, v in kw.items() if k != "v_ego"}
  outs, vs = to_stop(hook, o, v=1.8, **rest)                     # the abort condition persists: ignored while finishing
  assert min(vs) < 1.5 and all(x.state == "ACTIVE" and x.floor == -0.8 and x.seg == 1 and x.stop_intent and not x.own for x in outs[:-1])
  assert all(x.text2 == "finishing the stop; brake to end" for x in outs[1:-1])
  held = outs[-1]
  assert held.state == "HELD" and held.floor == -0.8 and held.stop_intent and not held.own
  if locked:
    assert (held.text1, held.text2) == (f"TEST LOCKED - {reason} - HELD", "brake to end; restart the car")
  else:
    assert (held.text1, held.text2) == ("TEST E 1/6 STOPPED - hold 0.0 s", "brake to finish (not counted)")
  hold(hook, 300)
  o = brake(hook)
  assert o.rep_done == "" and o.floor is None and hook.done == plan("E")["done"]
  if locked:
    assert o.state == "LOCKED" and (o.text1, o.reason) == (f"TEST MODE LOCKED - {reason}", reason)
  else:
    assert o.state == "ARMED" and (o.text1, o.text2) == (f"E 1/6 NOT COUNTED - {reason}", "next E 1/6: -0.8 to 5 km/h, -0.3 for 2 s, -0.8 to stop")


@pytest.mark.parametrize("kw,reason", [(dict(distance_pressed=True), "press"), (dict(lead_status=True), "lead"), (dict(steer_deg=8.0), "steer"),
                                       (dict(plan_fcw=True), "fcw")])
def test_an_abort_at_or_below_v_intent_finishes_the_stop_even_without_the_intent(kw, reason):
  hook, _ = start(man="D")
  outs = feed(hook, [3.0, 2.5] + [2.0] * 20)                     # D's 0.0 coast: no intent in a time segment
  assert outs[-1].seg == 2 and outs[-1].floor == 0.0 and not outs[-1].stop_intent
  o = hook.update(good(v_ego=2.0, **kw), 0.0)
  assert o.state == "ACTIVE" and o.floor == 0.0 and o.stop_intent and not o.own and o.reason == reason and hook._finish
  outs = feed(hook, [1.5, 1.0, 0.5, 0.2, 0.0])                   # the stopping state (not the 0.0 floor) brings the car to rest
  assert all(x.state == "ACTIVE" and x.floor == 0.0 and x.stop_intent and not x.own for x in outs[:-1]) and outs[-1].state == "HELD"
  assert outs[-1].text2 == ("brake to end; restart the car" if reason == "fcw" else "brake to finish (not counted)")
  assert hold(hook, HOLD_N)[-1].floor < 0.0                      # the hold still builds from the 0.0 floor
  o = brake(hook)
  assert o.rep_done == "" and hook.done == plan("D")["done"]
  assert o.text1 == ("TEST MODE LOCKED - fcw" if reason == "fcw" else f"D 1/6 NOT COUNTED - {reason}")
  hook, _ = start(man="D")                                       # just above V_INTENT it still hands back
  feed(hook, [3.0, 2.5] + [2.01] * 20)
  assert hook.update(good(v_ego=2.01, **kw), 0.0).state == "HANDBACK"


@pytest.mark.parametrize("fault", ["interrupt", "banner", "exception"])
def test_a_lock_at_or_below_v_intent_finishes_the_stop_like_any_abort(monkeypatch, fault):
  hook, _ = start(man="D")
  feed(hook, [3.0, 2.5] + [2.0] * 20)
  if fault == "interrupt":
    o = hook.interrupt()
  elif fault == "banner":
    o = hook.lock("banner")
  else:
    monkeypatch.setattr(ih, "precondition_failure", _boom)
    o = hook.update(good(v_ego=2.0), 0.0)
  assert o.state == "ACTIVE" and o.stop_intent and hook._finish


@pytest.mark.parametrize("man", IDS)
def test_a_deeper_normal_demand_while_a_rep_runs_or_holds_is_owned_over_and_the_rep_counts(man):
  # drive 0000212e: the normal chain lagged behind the script (D's 0.0 coast carried -0.2) and held its own -0.70 at
  # standstill for the hook's stop intent; while a rep runs or holds, the floor is the wire (own) and nothing marks the rep
  hook, o = start(man=man)
  outs, _ = to_stop(hook, o, normal=-3.0)
  assert outs[-1].state == "HELD" and all(x.own and x.floor == SEGS[man][x.seg - 1].accel for x in outs[:-1])
  assert all(x.own and x.floor >= min(SEGS[man][-1].accel, A_HOLD) for x in hold(hook, HOLD_N, normal=-2.0))
  o = brake(hook)
  assert o.rep_done == man and o.reason == "complete" and o.text1 == f"{man} 1/6 DONE" and hook.done[man] == 1


@pytest.mark.parametrize("kw,reason", [(dict(lead_status=True), "lead"), (dict(plan_fcw=True), "fcw"), (dict(steer_deg=8.0), "steer"),
                                       (dict(distance_pressed=True), "press")])
@pytest.mark.parametrize("phase", ["before", "after"])           # the stop intent: hand back, or finish the stop
def test_an_abort_ends_ownership_on_its_frame_so_a_deeper_normal_demand_passes(kw, reason, phase):
  hook, _ = start(man="E")
  v = V0 if phase == "before" else 1.9                           # E's intent in seg 1 (-0.8 to 1.5 m/s)
  assert all(x.own for x in feed(hook, [V0] * 20 + [v], normal=-3.0))
  o = hook.update(good(v_ego=v, **kw), -3.0)                     # LongControl: wire = min(normal, floor) = -3.0 from this frame
  assert o.reason == reason and o.floor == -0.8 and not o.own and o.stop_intent == (phase == "after")
  assert o.state == ("HANDBACK" if phase == "before" else "ACTIVE")


# -- KCS1: the stall rule and the cap ------------------------------------------------------------------
def test_a_stall_deepens_at_j_hold_to_the_secure_hold_never_releases_and_still_counts():
  hook, _ = start(man="A")
  outs = feed(hook, [2.4] * 300)
  k = next(j for j, x in enumerate(outs) if x.stop_intent)
  assert k == 200 and outs[k].changed and outs[k].own and hook._stalled   # 2 s of samples in the window
  assert all(x.floor == -0.5 and x.own for x in outs[:k])      # owned before the stall too
  floors = [x.floor for x in outs[k:]]
  assert floors == pytest.approx([max(-0.5 - J_HOLD * DT * (j + 1), A_HOLD) for j in range(len(floors))], abs=1e-9) and floors[-1] == A_HOLD
  outs, _ = to_stop(hook, outs[-1], v=2.4)                       # the car slows now: the floor never releases
  assert all(x.floor == A_HOLD for x in outs) and outs[-1].state == "HELD"
  hold(hook, HOLD_N)
  o = brake(hook)
  assert o.rep_done == "A" and o.reason == "stalled" and o.text1 == "A 1/6 DONE" and hook._last == "last: A 1/6 done (stalled)"


@pytest.mark.parametrize("rate,stalls", [(0.0, True), (0.07, True), (0.08, False)])   # m/s^2 of slowing: 0.14 / 0.16 m/s in 2 s
def test_the_stall_rule_needs_less_than_the_minimum_slowing_over_two_seconds(rate, stalls):
  hook, _ = start(man="C")
  feed(hook, [3.0, 2.5])                                         # seg 2 (-0.3 to stop)
  outs = feed(hook, [2.4 - rate * DT * k for k in range(300)])
  assert any(x.stop_intent for x in outs) == stalls and hook._stalled == stalls


def test_the_stall_rule_needs_the_speed_below_its_threshold():
  hook, _ = start(man="C")
  outs = feed(hook, [3.0] + [2.5] * 400)
  assert outs[-1].seg == 2 and not hook._stalled and not outs[-1].stop_intent


def test_the_stall_window_restarts_at_a_segment_change():
  hook, _ = start(man="D")
  outs = feed(hook, [3.0] + [2.4] * 500)                         # 2 s of 0.0 coast (never a stall), then -0.8 at the same speed
  seg3 = next(j for j, x in enumerate(outs) if x.seg == 3)
  first = next(j for j, x in enumerate(outs) if x.stop_intent)
  assert first - seg3 == 200 and all(x.floor == -0.8 and x.own for x in outs[seg3:first]) and hook._stalled


def test_a_stall_never_releases_into_a_shallower_segment():
  hook, _ = start(man="E")
  outs = feed(hook, [1.8] * 250 + [1.5] + [1.4] * 250)
  assert hook._stalled and [x.seg for x in outs[249:252]] == [1, 2, 2] and outs[-1].seg == 3
  assert all(x.floor == -0.8 for x in outs)                      # E's -0.3 segment never releases the stalled -0.8


def test_no_standstill_within_thirty_seconds_hands_back_and_locks_vehicle():
  hook, _ = start()
  outs = feed(hook, [V0] * 3300)
  k = next(j for j, x in enumerate(outs) if x.state != "ACTIVE")
  assert abs((k + 1) * DT - CAP_S) <= DT + 1e-9 and outs[k].state == "HANDBACK" and outs[k].reason == "vehicle"
  assert outs[k].text2 == "releasing; test mode LOCKED" and hook.state == "LOCKED" and outs[-1].reason == "vehicle"


def test_the_cap_after_the_intent_finishes_into_a_locked_hold():
  hook, _ = start(man="A")
  outs = feed(hook, [1.9] * 3100)                                # intent at once, stall at 2 s, no standstill
  k = next(j for j, x in enumerate(outs) if not x.own)
  assert abs((k + 1) * DT - CAP_S) <= DT + 1e-9 and outs[k].state == "ACTIVE" and outs[k].stop_intent and outs[k].floor == A_HOLD
  assert outs[k].text1 == "TEST A 1/6 ABORTED - vehicle" and outs[k + 1].state == "HELD" and outs[k + 1].floor == A_HOLD
  assert (outs[k + 1].text1, outs[k + 1].text2) == ("TEST LOCKED - vehicle - HELD", "brake to end; restart the car")
  o = brake(hook)
  assert o.state == "LOCKED" and o.rep_done == "" and hook.done == plan("A")["done"]


# -- KCS1: faults in the hold, progress ------------------------------------------------------------------
@pytest.mark.parametrize("fault,reason", [(dict(plan_fcw=True), "fcw"), (dict(acc_faulted=True), "vehicle"), (dict(valid=False), "inputs"),
                                          ("interrupt", "fault"), ("banner", "banner")])
def test_a_fault_in_the_hold_keeps_the_hold_until_the_brake_then_locks(fault, reason):
  hook, o = start(man="C")
  to_stop(hook, o)
  assert all(x.own for x in hold(hook, 20))
  if fault == "interrupt":
    o = hook.interrupt()
  elif fault == "banner":
    o = hook.lock("banner")
  else:
    o = hook.update(good(v_ego=0.0, standstill=True, **fault), 0.0)
  assert o.state == "HELD" and o.stop_intent and o.floor < -0.3 and (o.text1, o.text2) == (f"TEST LOCKED - {reason} - HELD", "brake to end; restart the car")
  assert not o.own                                               # a locked hold is a floor: min(normal, floor)
  outs = hold(hook, 300, lead_status=True)                       # the hold keeps building to A_HOLD and stays; a lead changes nothing
  assert all(x.state == "HELD" and x.text1 == f"TEST LOCKED - {reason} - HELD" and not x.own for x in outs) and outs[-1].floor == A_HOLD
  o = brake(hook)
  assert o.state == hook.state == "LOCKED" and o.floor is None and o.rep_done == "" and o.reason == reason and hook.done == plan("C")["done"]
  assert (o.text1, o.text2) == (f"TEST MODE LOCKED - {reason}", "restart the car to use test mode again")


@pytest.mark.parametrize("record,done", [
  (None, {}), ("KCS1", {}), ([], {}), ({}, {}), ({"plan": "KCS0", "done": {"B": 3}}, {}), ({"plan": PLAN_ID}, {}), ({"plan": PLAN_ID, "done": [3]}, {}),
  ({"plan": PLAN_ID, "done": {"B": 3, "E": 1}}, {"B": 3, "E": 1}),
  ({"plan": PLAN_ID, "done": {"B": -2, "A": True, "C": 99, "D": 2.0, "E": "4", "Z": 5}}, {"C": N_REPS}),
  ({"plan": PLAN_ID, "done": {"A": False, "D": N_REPS, "b": 2}}, {"D": N_REPS}),
])
def test_load_accepts_only_this_plans_integer_counts(record, done):
  hook = IdentificationHook()
  hook.load({"plan": PLAN_ID, "done": dict.fromkeys(IDS, 2)})    # anything loaded before is replaced, never merged
  hook.load(record)
  assert hook.done == {**ZERO, **done}


def test_progress_round_trips_through_json_and_is_a_copy():
  hook = IdentificationHook()
  hook.load({"plan": PLAN_ID, "done": {"B": 2, "A": 1, "E": 6}})
  record = hook.progress()
  assert record == {"plan": PLAN_ID, "done": {"B": 2, "A": 1, "C": 0, "D": 0, "E": 6}}
  record["done"]["B"] = 5
  assert hook.done["B"] == 2
  other = IdentificationHook()
  other.load(json.loads(json.dumps(hook.progress())))
  assert other.done == hook.done and other.progress() == hook.progress()
  full_rep(other)
  assert other.progress()["done"] == {"B": 2, "A": 1, "C": 1, "D": 0, "E": 6}


# -- banners -----------------------------------------------------------------------------------------
def test_banner_strings_through_one_rep():
  hook = IdentificationHook()
  hook.load(plan("C"))
  ready(hook)
  o = hook.update(good(), 0.0)
  assert o.text1.startswith("TEST C 1/6 STARTS IN ") and o.text2 == "C: -1.0 to 9 km/h, -0.3 to stop; brake = not now"
  o = press(hook, SHORT)[-1]
  assert (o.text1, o.text2) == ("TEST C 1/6 s1 -1.00", "press = cancel (releases, cruise resumes)")
  outs = feed(hook, [4.04, 2.5, 1.96])
  assert (outs[0].text1, outs[0].text2) == ("TEST C 1/6 s1 -1.00 - 4.0 m/s", "press = cancel (releases, cruise resumes)")
  assert outs[1].text1 == "TEST C 1/6 s2 -0.30 - 2.5 m/s" and outs[1].changed
  assert (outs[2].text1, outs[2].text2) == ("TEST C 1/6 s2 -0.30 - 2.0 m/s", "press = finish and hold")
  o = feed(hook, [0.0])[0]
  assert (o.text1, o.text2) == ("TEST C 1/6 STOPPED - hold 0.0 s", "brake to finish")
  n = round(HOLD_BRAKE_S / DT) + 50
  outs = hold(hook, n)
  assert outs[149].text1 == "TEST C 1/6 STOPPED - hold 1.5 s" and outs[-1].text1 == f"TEST C 1/6 STOPPED - hold {n * DT:.1f} s - BRAKE NOW"
  o = brake(hook)
  assert (o.text1, o.text2) == ("C 1/6 DONE", "next B 2/6: -1.0 to stop")


def test_no_banner_text_can_trigger_the_prompt_sound(monkeypatch):
  # events.longitudinal_maneuver_alert plays AudibleAlert.prompt when "Active" (case-sensitive) is in alertText1
  seen, finish_out = [], IdentificationHook._finish_out

  def record(self, out, prev):
    seen.append(finish_out(self, out, prev))
    return seen[-1]
  monkeypatch.setattr(IdentificationHook, "_finish_out", record)
  hook = IdentificationHook()
  for man in IDS:                                                # every walk, a BRAKE NOW hold and its DONE notice
    full_rep(hook, man, held=round(HOLD_BRAKE_S / DT) + 50)
    run(hook, lambda k: good(brake=True, enabled=False), 50)
  press(hook, LONG)                                              # OFF notice
  hook, _ = start(man="A")
  hook.update(good(distance_pressed=True), 0.0)                  # cancel: HANDBACK
  run(hook, lambda k: good(), 200)
  hook, _ = start(man="E")
  feed(hook, [V0, 1.9])
  hook.update(good(v_ego=1.8, lead_status=True), 0.0)            # finish, then an uncounted hold
  feed(hook, [1.0, 0.0])
  hold(hook, 150)
  brake(hook)
  hook, _ = start()
  hook.update(good(gas=True), 0.0)                               # driver abort: OFF notice
  hook, _ = start()
  hook.interrupt()                                               # fault: LOCKED
  run(hook, lambda k: good(), 200)
  press(hook, LONG)
  hook = IdentificationHook()
  hook.load({"plan": PLAN_ID, "done": dict.fromkeys(IDS, N_REPS)})
  armed(hook)
  run(hook, lambda k: good(), 20)                                # BLOCK COMPLETE
  assert {o.state for o in seen} >= {"OFF", "ARMED", "READY", "ACTIVE", "HELD", "HANDBACK", "LOCKED"}
  assert {"B 1/6 DONE", "TEST MODE OFF", "TEST BLOCK COMPLETE - long press = off", "TEST MODE LOCKED - fault"} <= {o.text1 for o in seen}
  assert any("BRAKE NOW" in o.text1 for o in seen) and any("NOT COUNTED" in o.text1 for o in seen)
  assert not any("Active" in o.text1 for o in seen)


# -- driver, faults, resets -------------------------------------------------------------------------
@pytest.mark.parametrize("kw,reason", [(dict(brake=True), "pedal"), (dict(gas=True), "pedal"), (dict(enabled=False), "disengaged"),
                                       (dict(long_active=False), "disengaged"), (dict(long_active=False, brake=True), "pedal")])
@pytest.mark.parametrize("phase", ["before", "after"])           # the stop intent
def test_a_driver_action_while_moving_ends_authority_at_once_and_turns_test_mode_off(kw, reason, phase):
  hook, _ = start()
  v = V0 if phase == "before" else 1.9
  assert feed(hook, [V0] * 50 + [v])[-1].stop_intent == (phase == "after")
  o = hook.update(good(v_ego=v, **kw), 0.0)
  assert o.floor is None and not o.own and not o.stop_intent and o.changed and o.state == hook.state == "OFF" and o.reason == reason
  assert (o.text1, o.text2) == (f"TEST B 1/6 ABORTED - {reason}", "test mode off; long press distance to arm")
  outs = run(hook, lambda k: good(), 300)
  outs += press(hook, SHORT) + run(hook, lambda k: good(), 300)
  assert hook.state == "OFF" and all(x.floor is None for x in outs)
  press(hook, LONG)
  assert hook.state == "ARMED" and hook.done == ZERO


@pytest.mark.parametrize("ended_by,after", [(dict(lead_prob=0.5), "ARMED"), (dict(pause_long=True), "OFF"), (dict(force_coast=True), "OFF")])
@pytest.mark.parametrize("driver", ["brake", "wait"])
def test_a_driver_action_during_the_release_ends_authority_and_keeps_the_rep_result(ended_by, after, driver):
  hook, _ = start()
  assert hook.update(good(**ended_by), 0.0).state == "HANDBACK"
  if driver == "brake":
    o = hook.update(good(brake=True, enabled=False), 0.0)
    assert o.floor is None and not o.stop_intent and hook.state == after
  else:
    run(hook, lambda k: good(**ended_by), 300)                   # the release runs out on its own
    assert hook.state == after
  assert hook.done == ZERO


@pytest.mark.parametrize("kw,reason", [(dict(plan_fcw=True), "fcw"), (dict(stock_aeb=True), "fcw"), (dict(acc_faulted=True), "vehicle"),
                                       (dict(valid=False), "inputs"), (dict(mapping_ok=False), "mapping"), (dict(lead_prob=0.5, plan_fcw=True), "fcw")])
@pytest.mark.parametrize("phase", ["ACTIVE", "HANDBACK"])
def test_a_fault_during_a_rep_or_its_release_locks_test_mode(kw, reason, phase):
  hook, _ = start()
  if phase == "HANDBACK":
    assert hook.update(good(distance_pressed=True), 0.0).reason == "press"   # cancel first; the fault comes during the release
  o = hook.update(good(**kw), 0.0)
  assert o.state == "HANDBACK" and not o.stop_intent and o.text2 == "releasing; test mode LOCKED"
  outs = run(hook, lambda k: good(), 300)
  assert hook.state == "LOCKED" and outs[-1].reason == reason and all(x.state != "ACTIVE" for x in outs)
  outs = press(hook, LONG) + run(hook, lambda k: good(), 300) + press(hook, SHORT)
  assert hook.state == "LOCKED" and all(x.floor is None for x in outs)
  assert (outs[LONG - 1].text1, outs[LONG - 1].text2) == (f"TEST MODE LOCKED - {reason}", "restart the car to use test mode again")


def test_a_driver_action_never_clears_a_lock():
  hook, _ = start()
  o = hook.update(good(plan_fcw=True, brake=True, enabled=False), 0.0)
  assert o.floor is None and hook.state == "LOCKED"


def test_routine_resets_keep_test_mode_armed_and_restart_qualification():
  hook = ready()
  for _ in range(500):                                           # controlsd while disengaged: LoC.reset() then update
    hook.reset()
    hook.update(good(long_active=False, enabled=False), 0.0)
  assert hook.state == "ARMED"
  run(hook, lambda k: good(), 150)                               # re-engaged: only 1.5 s qualified
  assert all(o.state != "ACTIVE" for o in press(hook, SHORT))
  assert start(hook)[1].floor == -1.0


def test_an_input_fault_while_armed_only_restarts_qualification():
  hook = ready()
  o = hook.interrupt()
  assert hook.state == o.state == "ARMED" and o.floor is None and (o.text1, o.text2) == ("TEST ARMED - waiting: fault", "long press = off")
  assert start(hook)[1].floor == -1.0


def test_an_input_fault_during_a_rep_locks_and_its_release_stays_bounded():
  hook, _ = start()
  o = hook.interrupt()
  assert o.state == "HANDBACK" and o.reason == "fault" and o.floor == -1.0 and o.text2 == "releasing; test mode LOCKED"
  caps = [x.floor for x in run(hook, lambda k: good(), 200, normal=0.5) if x.floor is not None]
  assert caps[0] == pytest.approx(-1.0 + RELEASE_JERK * DT) and hook.state == "LOCKED"
  assert all(0.0 <= b - a <= RELEASE_JERK * DT + 1e-9 for a, b in zip(caps, caps[1:], strict=False))


def test_an_input_fault_after_the_intent_finishes_the_stop_and_locks():
  hook, _ = start(man="E")
  feed(hook, [V0, 1.9])
  o = hook.interrupt()
  assert o.state == "ACTIVE" and o.floor == -0.8 and o.stop_intent and not o.own and hook._finish
  outs, _ = to_stop(hook, o, v=1.9)
  assert outs[-1].state == "HELD" and outs[-1].text1 == "TEST LOCKED - fault - HELD"
  assert brake(hook).state == "LOCKED" and hook.done == plan("E")["done"]


def test_an_exception_during_a_rep_locks_through_the_release_bound(monkeypatch):
  hook, _ = start()
  feed(hook, [V0] * 30)
  monkeypatch.setattr(ih, "precondition_failure", _boom)
  outs = run(hook, lambda k: good(), 200, normal=0.5)            # raising on EVERY frame
  caps = [x.floor for x in outs if x.floor is not None]
  assert outs[0].reason == "exception" and outs[0].state == "HANDBACK" and hook.state == "LOCKED" and all(x.state != "ACTIVE" for x in outs)
  assert len(caps) >= 120 and all(0.0 <= b - a <= RELEASE_JERK * DT + 1e-9 for a, b in zip(caps, caps[1:], strict=False))


def test_an_exception_after_the_intent_keeps_the_floor_until_the_driver_acts(monkeypatch):
  hook, _ = start(man="E")
  feed(hook, [V0, 1.9])
  monkeypatch.setattr(ih, "precondition_failure", _boom)
  outs = run(hook, lambda k: good(v_ego=1.0), 200, normal=0.5)
  assert all(x.floor == -0.8 and x.stop_intent and not x.own for x in outs) and hook.state == "ACTIVE" and hook._locked == "exception"
  o = hook.update(good(brake=True), 0.5)
  assert o.floor is None and not o.stop_intent and hook.state == "LOCKED"


@pytest.mark.parametrize("kw", [dict(enabled=False, long_active=False), dict(brake=True), dict(gas=True)], ids=["cancel", "brake", "gas"])
@pytest.mark.parametrize("phase", ["ACTIVE", "HANDBACK", "HELD"])
def test_a_driver_action_ends_authority_even_while_every_frame_raises(monkeypatch, kw, phase):
  hook, o = start()
  if phase == "HELD":
    to_stop(hook, o)
    hold(hook, 2 * HOLD_N)
  monkeypatch.setattr(ih, "precondition_failure", _boom)
  if phase == "HANDBACK":
    assert hook.update(good(), 0.5).state == "HANDBACK"
  outs = run(hook, lambda k: good(**kw), 50, normal=0.5)
  assert all(x.floor is None and not x.stop_intent and not x.own for x in outs) and hook.state == "LOCKED" and hook.done == ZERO


@pytest.mark.parametrize("bad", [None, SimpleNamespace(valid=True)], ids=["none", "partial"])
@pytest.mark.parametrize("phase", ["ACTIVE", "HELD"])
def test_a_malformed_input_object_during_a_rep_gives_up_authority(bad, phase):
  hook, o = start()
  if phase == "HELD":
    to_stop(hook, o)
  o = hook.update(bad, 0.0)
  assert o.floor is None and not o.stop_intent and hook.state == "LOCKED" and hook.done == ZERO


def test_an_exception_while_armed_locks_without_a_command(monkeypatch):
  hook = ready()
  monkeypatch.setattr(ih, "precondition_failure", _boom)
  o = hook.update(good(), 0.0)
  assert hook.state == "LOCKED" and o.floor is None and not o.stop_intent and o.text1 == "TEST MODE LOCKED - exception"


# -- LongControl integration: the FINAL writer, the stop intent, flag-off equality --------------------------
def _press_schedule(k, v=V0, **kw):
  return good(v_ego=v, distance_pressed=k in ARM or k in START, **kw)


def _lc(monkeypatch, flag=True, ki=0.0):
  from openpilot.selfdrive.controls.lib import stopping_flags
  from openpilot.selfdrive.controls.lib.longcontrol import LongControl, LongCtrlState
  from openpilot.selfdrive.controls.lib.tests.test_longcontrol_fast_release import DummyCarParams
  monkeypatch.setattr(stopping_flags, "IDENTIFICATION_HOOK", flag)
  cp = DummyCarParams()
  cp.longitudinalTuning.kpV, cp.longitudinalTuning.kiV = [0.0], [ki]   # the Santa Fe HEV runs kp = ki = 0
  lc = LongControl(cp)
  lc.long_control_state = LongCtrlState.pid
  return lc


def _step(lc, inputs, a_target=-0.3, v=V0, active=True, **kw):
  from openpilot.selfdrive.controls.lib.tests.test_longcontrol_fast_release import DummyCarState, DummyFrogPilotToggles
  return float(lc.update(active=active, CS=DummyCarState(v_ego=v, a_ego=-0.3, brake_pressed=kw.pop("brake_pressed", False)), a_target=a_target,
                         should_stop=False, distance_to_stop_target_m=-1.0, accel_limits=(-3.0, 2.0), frogpilot_toggles=DummyFrogPilotToggles(),
                         id_inputs=inputs, **kw))


def _frames(lc, n, inputs_fn, a_target=-0.3):
  return [_step(lc, inputs_fn(k), a_target=a_target) for k in range(n)]


def _drive(monkeypatch, man, ki=0.0, hold_frames=150, delay=45, rep_target=None):
  """Real LongControl + a pure-delay plant (the car's accel = the wire 0.45 s earlier) from the arming press through the brake
  that ends the hold (openpilot disengages: LoC.reset() + update(active=False)); the planner wants the set speed back, or
  rep_target (if given) while the rep runs or holds. Every frame: (hook output, wire, long_control_state after the update,
  service owning)."""
  from openpilot.selfdrive.controls.lib.longcontrol import LongCtrlState
  from openpilot.selfdrive.controls.lib.tests.test_longcontrol_fast_release import DummyCarState, DummyFrogPilotToggles
  lc = _lc(monkeypatch, ki=ki)
  lc._id_hook.load(plan(man))
  v, pipe, rows, held_at, brake_at = V0, [0.0] * delay, [], None, None
  for k in range(6000):
    a = pipe[0]
    braking = held_at is not None and k >= held_at + hold_frames
    standstill = v < 0.1
    a_target = 0.3 if v < 5.5 else 0.0
    if rep_target is not None and lc.id_hook_out is not None and lc.id_hook_out.state in ("ACTIVE", "HELD"):
      a_target = rep_target
    inputs = _press_schedule(k, v=v, a_ego=a, pid_state=lc.long_control_state == LongCtrlState.pid, standstill=standstill, plan_accel=a_target,
                             brake=braking, enabled=not braking, long_active=not braking)
    if braking:
      lc.reset()
    wire = float(lc.update(not braking, DummyCarState(v_ego=v, a_ego=a, brake_pressed=braking, standstill=standstill), a_target, False, -1.0,
                           (-3.5, 2.0), DummyFrogPilotToggles(), id_inputs=inputs))
    rows.append((lc.id_hook_out, wire, lc.long_control_state, lc._service_live_owning))
    if held_at is None and lc.id_hook_out.state == "HELD":
      held_at = k
    if braking and brake_at is None:
      brake_at = k
    if brake_at is not None and k >= brake_at + 100:
      break
    pipe = pipe[1:] + [-1.5 if braking else wire]
    v = max(0.0, v + a * DT)
  return lc, rows


@pytest.mark.parametrize("ki", [0.0, 0.3])
def test_longcontrol_flag_off_or_never_armed_is_byte_identical(monkeypatch, ki):
  base = _frames(_lc(monkeypatch, False, ki), 500, lambda k: None)
  off = _frames(_lc(monkeypatch, False, ki), 500, _press_schedule)
  shorts = _frames(_lc(monkeypatch, True, ki), 500, lambda k: good(distance_pressed=k % 40 < SHORT))
  assert off == base and shorts == base


def test_longcontrol_construction_is_off_and_scoped(monkeypatch):
  from openpilot.selfdrive.controls.lib.longcontrol import LongControl
  from openpilot.selfdrive.controls.lib.tests.test_longcontrol_fast_release import DummyCarParams
  assert _lc(monkeypatch)._id_hook.state == "OFF" and _lc(monkeypatch, flag=False)._id_hook is None
  assert LongControl(DummyCarParams(car_fingerprint="HYUNDAI_ELANTRA_2021"))._id_hook is None


@pytest.mark.parametrize("man,ki", [(m, 0.0) for m in IDS] + [("B", 0.3), ("E", 0.3)])
def test_longcontrol_every_maneuver_puts_its_script_on_the_wire(monkeypatch, man, ki):
  from openpilot.selfdrive.controls.lib.longcontrol import LongCtrlState
  lc, rows = _drive(monkeypatch, man, ki)
  outs, wires, states = [r[0] for r in rows], [r[1] for r in rows], [r[2] for r in rows]
  active = [k for k, o in enumerate(outs) if o.state == "ACTIVE"]
  assert active == list(range(T0, active[-1] + 1)) and outs[active[-1] + 1].state == "HELD"
  assert all(wires[k] == outs[k].floor == SEGS[man][outs[k].seg - 1].accel for k in active)   # the script, exactly, on every frame
  assert sorted({outs[k].seg for k in active}) == list(range(1, len(SEGS[man]) + 1))
  intent = next(k for k in active if outs[k].stop_intent)
  assert all(s == LongCtrlState.pid for s in states[T0:intent + 1]) and states[intent + 1] == LongCtrlState.stopping   # the next frame
  assert all(outs[k].own for k in active)                        # the rep owns the wire from its first frame
  held = [k for k, o in enumerate(outs) if o.state == "HELD"]
  assert all(states[k] == LongCtrlState.stopping and wires[k] == outs[k].floor and outs[k].own for k in held)
  done = [k for k, o in enumerate(outs) if o.rep_done]
  assert [outs[k].rep_done for k in done] == [man] and outs[done[0]].reason == "complete" and lc._id_hook.done[man] == 1
  assert wires[done[0]:] == [0.0] * (len(wires) - done[0])       # disengaged by the brake: zero on every frame
  assert not any(r[3] for r in rows)                             # the stopping service never owns


@pytest.mark.parametrize("man", ["C", "D"])                    # the floor RISES before the intent (C's -0.3, D's 0.0 after -1.0)
def test_longcontrol_with_an_integrator_a_rising_floor_before_the_intent_is_followed(monkeypatch, man):
  lc, rows = _drive(monkeypatch, man, ki=0.3)
  active = [r for r in rows if r[0].state == "ACTIVE"]
  assert all(r[1] == r[0].floor for r in active) and lc._id_hook.done[man] == 1


def test_longcontrol_a_long_hold_stays_on_the_hook_floor_and_counts(monkeypatch):
  lc, rows = _drive(monkeypatch, "A", hold_frames=800)           # 8 s: the stopping chain never gets deeper than the hold
  held = [r for r in rows if r[0].state == "HELD"]
  assert len(held) == 800 and all(r[1] == r[0].floor for r in held) and held[-1][1] == A_HOLD and lc._id_hook.done["A"] == 1


@pytest.mark.parametrize("ki", [0.0, 0.3])
def test_longcontrol_a_rep_floors_the_wire_and_reseeds_an_integrator(monkeypatch, ki):
  lc = _lc(monkeypatch, ki=ki)
  wires = _frames(lc, T0 + 20, _press_schedule)
  assert lc._id_hook.state == "ACTIVE" and lc.id_hook_out.maneuver == "B" and lc.id_hook_out.floor == -1.0
  assert wires[T0 - 1] != -1.0 and wires[T0:] == [-1.0] * 20
  if ki:
    assert lc.pid.i == pytest.approx(lc.last_output_accel - (lc.pid.p + lc.pid.d + lc.pid.f), abs=1e-9)


@pytest.mark.parametrize("ki", [0.0, 0.3])
def test_longcontrol_a_deeper_planner_demand_while_a_rep_runs_stays_off_the_wire(monkeypatch, ki):
  lc = _lc(monkeypatch, ki=ki)
  lc._id_hook.load(plan("A"))
  _frames(lc, T0 + 50, _press_schedule)
  assert lc._id_hook.state == "ACTIVE" and lc.last_output_accel == -0.5
  later = [_step(lc, good(plan_accel=-2.0), a_target=-2.0) for _ in range(100)]
  assert later == [-0.5] * 100 and lc._id_hook.state == "ACTIVE" and lc.id_hook_out.own and lc.id_hook_out.floor == -0.5


@pytest.mark.parametrize("man", IDS)
def test_longcontrol_a_deeper_planner_demand_through_a_rep_and_its_hold_stays_off_the_wire_and_counts(monkeypatch, man):
  lc, rows = _drive(monkeypatch, man, rep_target=-2.0)
  owned = [r for r in rows if r[0].state in ("ACTIVE", "HELD")]
  assert all(r[0].own and r[1] == r[0].floor for r in owned) and {r[0].state for r in owned} == {"ACTIVE", "HELD"}
  assert lc._id_hook.done[man] == 1 and [r[0].reason for r in rows if r[0].rep_done] == ["complete"]


@pytest.mark.parametrize("ki", [0.0, 0.3])
@pytest.mark.parametrize("kw,reason", [(dict(lead_status=True), "lead"), (dict(plan_fcw=True), "fcw"), (dict(steer_deg=8.0), "steer"),
                                       (dict(distance_pressed=True), "press")])
def test_longcontrol_after_an_abort_a_deeper_planner_demand_passes_on_that_frame(monkeypatch, kw, reason, ki):
  lc = _lc(monkeypatch, ki=ki)
  lc._id_hook.load(plan("A"))
  _frames(lc, T0 + 50, _press_schedule)
  owned = [_step(lc, good(plan_accel=-2.0), a_target=-2.0) for _ in range(20)]
  wire = _step(lc, good(plan_accel=-2.0, **kw), a_target=-2.0)
  out = lc.id_hook_out
  assert owned == [-0.5] * 20 and out.state == "HANDBACK" and out.reason == reason and not out.own and out.floor == -0.5
  # min(normal, floor). With kiV > 0 the owned frames reseeded pid.i to the wire, so the normal chain resumes from about -0.5
  # and deepens at the integrator rate; the Santa Fe HEV runs kiV = 0, where the full demand passes at once
  assert wire < -0.5 and (ki or wire == -2.0)


def test_longcontrol_handback_is_release_bounded(monkeypatch):
  lc = _lc(monkeypatch)
  _frames(lc, T0 + 50, _press_schedule)
  wires, states = [], []
  for _ in range(200):
    wires.append(_step(lc, good(lead_prob=0.5), a_target=0.5))
    states.append(lc.id_hook_out.state)
  n = states.index("ARMED")                                      # the frame the release reaches the normal chain
  assert wires[0] == -1.0 and states[0] == "HANDBACK" and abs(n - 1.0 / (RELEASE_JERK * DT)) <= 1 and wires[n] == pytest.approx(0.0, abs=1e-9)
  assert all(0.0 <= b - a <= RELEASE_JERK * DT + 1e-9 for a, b in zip(wires[:n], wires[1:n + 1], strict=True))
  assert wires[-1] == 0.5 and not lc.id_hook_out.stop_intent      # then normal cruise owns the wire and can accelerate


@pytest.mark.parametrize("kw", [dict(enabled=False, brake=True), dict(enabled=False), dict(pause_long=True)], ids=["brake", "cancel", "pause"])
def test_longcontrol_off_after_a_rep_requests_zero_on_every_frame(monkeypatch, kw):
  # controlsd with longActive False: LoC.reset() then LoC.update(active=False) on every frame
  lc = _lc(monkeypatch)
  _frames(lc, T0 + 50, _press_schedule)
  off = []
  for _ in range(300):
    lc.reset()
    off.append(_step(lc, good(long_active=False, **kw), active=False, brake_pressed=kw.get("brake", False)))
  assert off == [0.0] * 300 and lc._id_hook.state == "OFF" and lc._id_hook.done == ZERO and not lc.id_hook_out.stop_intent


def test_longcontrol_cancel_while_every_frame_raises_requests_zero(monkeypatch):
  lc = _lc(monkeypatch)
  _frames(lc, T0 + 50, _press_schedule)
  monkeypatch.setattr(ih, "precondition_failure", _boom)
  assert _step(lc, good(), a_target=0.5) == -1.0 and lc._id_hook.state == "HANDBACK"
  off = []
  for _ in range(100):
    lc.reset()
    off.append(_step(lc, good(enabled=False, long_active=False), active=False))
  assert off == [0.0] * 100 and lc._id_hook.state == "LOCKED"


def test_longcontrol_gas_while_active_drops_the_hook_floor(monkeypatch):
  lc = _lc(monkeypatch)
  _frames(lc, T0 + 50, _press_schedule)
  wire = _step(lc, good(gas=True), a_target=0.5, freeze_integrator=True)
  assert lc.id_hook_out.floor is None and lc.id_hook_out.reason == "pedal" and wire > -1.0 + 0.1 and lc._id_hook.state == "OFF"


def test_longcontrol_arms_while_parked_across_routine_resets(monkeypatch):
  lc = _lc(monkeypatch)
  for k in range(200):
    lc.reset()
    _step(lc, good(long_active=False, enabled=False, v_ego=0.0, standstill=True, distance_pressed=k in ARM), v=0.0, active=False)
  assert lc._id_hook.state == "ARMED" and lc.id_hook_out.text1 == "TEST ARMED - waiting: disengaged"


def test_longcontrol_input_fault_while_armed_only_restarts_qualification_but_during_a_rep_locks(monkeypatch):
  lc = _lc(monkeypatch)
  _frames(lc, 300, _press_schedule)
  assert lc._id_hook.state == "READY"
  _step(lc, good(), plan_valid=False)
  assert lc._id_hook.state == "ARMED" and lc.id_hook_out.text1 == "TEST ARMED - waiting: fault"
  _frames(lc, 250, lambda k: good())
  _frames(lc, SHORT + SETTLE, lambda k: good(distance_pressed=k < SHORT))
  assert lc._id_hook.state == "ACTIVE" and lc.id_hook_out.maneuver == "B"
  held = _step(lc, good(), plan_valid=False)
  assert held == -1.0 and lc._id_hook.state == "HANDBACK" and lc.id_hook_out.reason == "fault" and lc._id_hook._locked == "fault"
  released = []
  while lc._id_hook.state == "HANDBACK":
    released.append(_step(lc, good(), a_target=0.5))
  assert released[0] == -1.0 and released[-1] == pytest.approx(0.0, abs=1e-9) and abs(len(released) - 1 - 1.0 / (RELEASE_JERK * DT)) <= 1
  assert all(0.0 <= b - a <= RELEASE_JERK * DT + 1e-9 for a, b in zip(released, released[1:], strict=False))
  assert lc._id_hook.state == "LOCKED" and not any(_step(lc, _press_schedule(k)) == -1.0 for k in range(500))


# -- controlsd: inputs from genuine cereal messages, and the alertDebug banner ------------------------
class _SubMaster(dict):
  valid = dict.fromkeys(('carState', 'radarState', 'modelV2', 'longitudinalPlan', 'livePose', 'frogpilotCarState', 'selfdriveState'), True)
  alive = valid


def _controls_inputs(lead_probs, distance_pressed=False, fcs=None, toggles=None, maneuver_mode=False, a_target=0.0, v_ego=V0, a_ego=0.0,
                     v_cruise_kph=float(ih.SET_SPEED_KPH)):
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
  CS = car.CarState.new_message(vEgo=v_ego, aEgo=a_ego, vCruise=v_cruise_kph, canValid=True, gearShifter=car.CarState.GearShifter.drive).as_reader()
  return Controls._identification_inputs(ctl, CS, car.CarControl.new_message(longActive=True).as_reader())


def _start_from(lead_probs):
  hook = IdentificationHook()
  released, pressed = _controls_inputs(lead_probs), _controls_inputs(lead_probs, distance_pressed=True)
  run(hook, lambda k: released, SETTLE)
  run(hook, lambda k: pressed, LONG)
  outs = run(hook, lambda k: released, round(PRECONDITION_S / DT) + 5) + run(hook, lambda k: pressed, SHORT) + run(hook, lambda k: released, SETTLE)
  return hook, outs[-1]


# baseline (controlsd.py b37a78a9) started a trial for the first six ([], [0.02], [0.02, nan], [-inf, 0.02], [0.02, -0.5],
# [-0.5, -0.2]); its max() already rejected [nan, 0.02] and [0.02, inf] (non-finite max -> 'inputs') and [0.02, 1.5] (-> 'lead')
MALFORMED_MODEL_LEADS = [[], [0.02], [0.02, math.nan], [-math.inf, 0.02], [0.02, -0.5], [-0.5, -0.2],
                         [math.nan, 0.02], [0.02, math.inf], [0.02, 1.5]]


@pytest.mark.parametrize("lead_probs", MALFORMED_MODEL_LEADS)
def test_controlsd_malformed_model_leads_fail_closed(lead_probs):
  assert precondition_failure(_controls_inputs(lead_probs), 0.0, GATE) == "inputs"
  hook, o = _start_from(lead_probs)
  assert o.state != "ACTIVE" and o.floor is None and hook.done == ZERO


@pytest.mark.parametrize("lead_probs", MALFORMED_MODEL_LEADS)
def test_controlsd_malformed_model_leads_abort_an_active_rep_and_lock(lead_probs):
  hook, o = _start_from([0.02, 0.01])
  assert o.state == "ACTIVE"
  o = hook.update(_controls_inputs(lead_probs), 0.0)
  assert o.state == "HANDBACK" and o.reason == "inputs"
  run(hook, lambda k: good(), 200)
  assert hook.state == "LOCKED"


@pytest.mark.parametrize("lead_probs", [[0.02, 0.01], [0.0, 0.0], [0.02, 0.01, 0.9]])   # modeld publishes three rows
def test_controlsd_two_low_model_leads_still_start_a_rep(lead_probs):
  inputs = _controls_inputs(lead_probs)
  assert inputs.valid and inputs.lead_prob == pytest.approx(max(lead_probs[:2]))
  hook, o = _start_from(lead_probs)
  assert o.state == "ACTIVE" and o.maneuver == "B" and o.floor == -1.0


@pytest.mark.parametrize("lead_probs", [[0.02, 0.1], [0.3, 0.01], [0.0, 1.0]])
def test_controlsd_genuine_model_lead_still_blocks(lead_probs):
  assert precondition_failure(_controls_inputs(lead_probs), 0.0, GATE) == "lead"
  assert _start_from(lead_probs)[1].state != "ACTIVE"


def test_controlsd_passes_the_planner_target_the_measured_accel_and_the_set_speed():
  inputs = _controls_inputs([0.02, 0.01], a_target=-0.7, a_ego=-0.4, v_cruise_kph=30.0)
  assert (inputs.plan_accel, inputs.a_ego, inputs.v_cruise) == pytest.approx((-0.7, -0.4, 30.0 / 3.6))
  assert precondition_failure(_controls_inputs([0.02, 0.01], a_target=-0.7), 0.0, GATE) == "settling"
  assert precondition_failure(_controls_inputs([0.02, 0.01], a_target=-1.1), 0.0, GATE) == "demand"
  assert precondition_failure(_controls_inputs([0.02, 0.01], a_ego=0.3), 0.0, GATE) == "settling"
  assert precondition_failure(_controls_inputs([0.02, 0.01], v_cruise_kph=25.0), 0.0, GATE) == "settling"   # not at the set speed
  assert precondition_failure(_controls_inputs([0.02, 0.01]), 0.0, GATE) is None                            # 20 km/h at the set speed


@pytest.mark.parametrize("toggles,maneuver", [(dict(), False), (dict(identification_mode=False), False),
                                              (dict(identification_mode=True, force_coast_via_distance=True), False),
                                              (dict(identification_mode=True, traffic_mode_via_distance_very_long=True), False),
                                              (dict(identification_mode=True), True)])
def test_controlsd_mapping_gate_needs_the_scope_every_distance_mapping_off_and_no_maneuver_mode(toggles, maneuver):
  inputs = _controls_inputs([0.02, 0.01], toggles=SimpleNamespace(**toggles), maneuver_mode=maneuver)
  assert precondition_failure(inputs, 0.0, GATE) == "mapping"
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
  assert created == [['alertDebug']] and pm.sent[-1] == ('alertDebug', "TEST ARMED - waiting: settling")


@pytest.mark.parametrize("fail_on", ["create", "send"])
@pytest.mark.parametrize("phase", ["ACTIVE", "HELD"])
def test_controlsd_banner_loss_locks_the_hook_and_never_retakes_the_channel(monkeypatch, fail_on, phase):
  ctl, hook, pm, created, publish = _banner_controls(monkeypatch, fail_on)
  _, o = start(hook)
  if phase == "HELD":
    to_stop(hook, o)
  ctl.LoC.id_hook_out = hook.update(good(v_ego=0.0, standstill=True) if phase == "HELD" else good(), 0.0)
  publish()                                                      # the channel was taken by fullupdate.sh or maneuversd
  out = ctl.LoC.id_hook_out
  assert ctl.id_banner_lost and hook._locked == "banner" and out.state == ("HANDBACK" if phase == "ACTIVE" else "HELD")
  if phase == "HELD":                                            # a hold stays until the brake
    assert out.stop_intent and out.floor is not None and out.text1 == "TEST LOCKED - banner - HELD"
    ctl.LoC.id_hook_out = brake(hook)
    publish()
  for _ in range(300):
    ctl.LoC.id_hook_out = hook.update(good(), 0.5)
    publish()
  assert hook.state == "LOCKED" and len(created) == 1 and pm.sent == [] and hook.done == ZERO


# -- KCS2 (the running block): its own table through the real module and LongControl ------------------
KCS2 = BLOCKS["KCS2"]
RUNNING_PLAN = ih.PLAN_ID   # read at import, before the autouse fixture pins KCS1


def test_the_module_runs_kcs2_with_unique_ids_across_blocks():
  assert RUNNING_PLAN == "KCS2" and [m[0] for m in KCS2] == ["P", "L", "I", "M", "J", "N", "K"]
  ids = [m[0] for block in BLOCKS.values() for m in block]
  assert len(ids) == len(set(ids))                               # the log analysis maps ids across plans
  assert set(INTENT_V) == set(BLOCKS) and INTENT_V["KCS2"] == 0.5
  assert all(seg.accel <= 0.0 for block in BLOCKS.values() for m in block for seg in m[2])
  assert all(m[2][-1].v_end is None and m[2][-1].t_s is None for block in BLOCKS.values() for m in block)   # every rep ends at the stop
  assert all(m[2][0].jerk is None for block in BLOCKS.values() for m in block)   # the start gate compares the first command


def _kcs2(monkeypatch):
  monkeypatch.setattr(ih, "PLAN_ID", "KCS2")
  monkeypatch.setattr(ih, "MANEUVERS", KCS2)


@pytest.mark.parametrize("man", ["P", "L", "I", "M", "J", "N", "K"])
def test_longcontrol_every_kcs2_maneuver_puts_its_script_on_the_wire_and_counts(monkeypatch, man):   # the car runs kp = ki = 0
  _kcs2(monkeypatch)
  from openpilot.selfdrive.controls.lib.longcontrol import LongCtrlState
  lc = _lc(monkeypatch)
  hook = lc._id_hook
  hook.load({"plan": "KCS2", "done": {m[0]: int(m[0] != man) for m in KCS2}})
  segs = {m[0]: m[2] for m in KCS2}[man]
  v, a_hist, held_at, k, last_w, states = V0, [0.0] * 46, None, 0, None, []
  schedule = [False] * SETTLE + [True] * LONG + [False] * 250 + [True] * SHORT + [False] * SETTLE
  while k < 6000:
    brake = held_at is not None and k >= held_at + 150
    inp = good(v_ego=v, a_ego=a_hist[-1], pid_state=lc.long_control_state == LongCtrlState.pid, standstill=v <= 0.104,
               distance_pressed=schedule[k] if k < len(schedule) else False, plan_accel=0.3 if v < 5.5 else 0.0, brake=brake,
               enabled=not brake, long_active=not brake)
    if brake:
      lc.reset()
    w = _step(lc, inp, a_target=0.3 if v < 5.5 else 0.0, v=v, active=not brake, brake_pressed=brake)
    out = lc.id_hook_out
    if out.state == "ACTIVE":
      seg = segs[out.seg - 1]
      assert w == out.floor and out.own
      ramp = seg.jerk is not None and last_w is not None and w != seg.accel and abs(w - last_w) <= seg.jerk * DT + 1e-9
      assert w == seg.accel or ramp or hook._stalled
      states.append((v, lc.long_control_state))
    last_w = w
    if held_at is None and out.state == "HELD":
      held_at = k
    a_hist.append(w if not brake else -1.5)
    v = max(0.0, v + (a_hist[-46] if v > 0 or a_hist[-46] > 0 else 0.0) * DT)
    if out.rep_done:
      break
    k += 1
  assert out.rep_done == man and hook.done[man] == 1 and hook._reason in ("complete", "stalled")
  # the normal chain's states: pid down to the KCS2 intent speed (0.5 m/s), then stopping
  assert all(st == LongCtrlState.pid for vv, st in states if vv > INTENT_V["KCS2"] + 0.02)
  assert any(st == LongCtrlState.stopping for vv, st in states if vv <= INTENT_V["KCS2"])


def test_a_kcs2_ease_rises_at_its_jerk_and_holds_the_level(monkeypatch):
  _kcs2(monkeypatch)
  hook = IdentificationHook()
  hook.load({"plan": "KCS2", "done": {m[0]: int(m[0] != "I") for m in KCS2}})
  hook, o = start(hook)
  assert o.floor == -1.0 and hook._man == 2
  outs = feed(hook, [3.0] * 20 + [2.4] * 60)
  i0 = next(k for k, x in enumerate(outs) if x.seg == 2)
  floors = [x.floor for x in outs[i0:]]
  assert floors[0] == pytest.approx(-1.0 + 1.5 * DT)
  steps = [b - a for a, b in zip(floors, floors[1:], strict=False) if b > a]
  assert all(x == pytest.approx(1.5 * DT) for x in steps[:-1]) and 0.0 < steps[-1] <= 1.5 * DT + 1e-9   # the last step lands on -0.5
  assert floors[-1] == -0.5 and floors.index(-0.5) == pytest.approx(0.5 / (1.5 * DT) - 1, abs=1)
  assert all(x.own and not x.stop_intent for x in outs)          # 2.4 m/s is above the KCS2 intent speed


@pytest.mark.parametrize("v,state", [(1.0, "HANDBACK"), (0.4, "ACTIVE")])
def test_kcs2_an_abort_above_its_intent_speed_hands_back_below_finishes(monkeypatch, v, state):
  _kcs2(monkeypatch)
  hook = IdentificationHook()
  hook.load({"plan": "KCS2", "done": {m[0]: int(m[0] != "L") for m in KCS2}})
  hook, o = start(hook)
  feed(hook, [3.0] * 10 + [v] * 5)
  o = hook.update(good(v_ego=v, lead_status=True, plan_has_lead=True), 0.0)
  assert hook.state == state and (o.stop_intent == (state == "ACTIVE"))


# -- review 20260926-151117 regressions -----------------------------------------------------------------
@pytest.mark.parametrize("kw,reason", [(dict(v_ego=1.0, standstill=False), "rolling"),
                                       (dict(v_ego=0.0, standstill=True, lead_status=True, plan_has_lead=True), "lead")])
def test_a_rolling_hold_or_a_lead_in_the_hold_ends_ownership_so_a_deeper_demand_passes(kw, reason):
  # finding 1: the HELD branch ignored rolling and a lead, so a -3.0 normal demand stayed off the wire at -0.7
  hook, o = start()
  to_stop(hook, o)
  hold(hook, 150)
  outs = [hook.update(good(plan_accel=-3.0, **kw), -3.0) for _ in range(50)]
  assert all(x.state == "HELD" and not x.own and x.stop_intent for x in outs)
  assert all(min(-3.0, x.floor) == -3.0 for x in outs)          # LongControl: wire = min(normal, floor) passes the -3.0
  assert outs[0].changed and hook._reason == reason
  o = hook.update(good(v_ego=0.0, standstill=True, brake=True, enabled=False, long_active=False), 0.0)
  assert o.rep_done == "" and o.text1.endswith(f"NOT COUNTED - {reason}")


def test_a_one_frame_standstill_flicker_at_zero_speed_keeps_the_hold_and_counts():
  # re-review 20260926-161602: one standstill=False frame at v 0 set "rolling" and dropped the -0.7 hold
  hook, o = start()
  to_stop(hook, o)
  hold(hook, 150)
  x = hook.update(good(v_ego=0.0, a_ego=0.0, standstill=False), -3.0)
  assert x.own and x.floor <= ih.A_HOLD and not hook._finish
  hold(hook, 10)
  o = hook.update(good(v_ego=0.0, standstill=True, brake=True, enabled=False, long_active=False), 0.0)
  assert o.rep_done == "B" and hook.done == {**ZERO, "B": 1}


def test_longcontrol_a_brake_during_an_input_fault_ends_the_hold_before_re_engagement(monkeypatch):
  # finding 2: fault frames skipped the hook, so the brake never reached it and re-engaging restored -0.7 at once
  from openpilot.selfdrive.controls.lib.longcontrol import LongCtrlState
  from openpilot.selfdrive.controls.lib.tests.test_longcontrol_fast_release import DummyCarState, DummyFrogPilotToggles
  lc = _lc(monkeypatch)
  hook, o = start(lc._id_hook)
  to_stop(hook, o)
  hold(hook, 150)
  lc.id_hook_out = hook.update(good(v_ego=0.0, standstill=True), 0.0)
  lc.long_control_state = LongCtrlState.stopping
  lc.last_output_accel = lc.id_hook_out.floor

  def step(active, brake, valid):
    if not active:
      lc.reset()
    return float(lc.update(active, DummyCarState(v_ego=0.0, a_ego=0.0, standstill=True, brake_pressed=brake), 0.3, False, -1.0, (-3.5, 2.0),
                           DummyFrogPilotToggles(), plan_valid=valid,
                           id_inputs=good(v_ego=0.0, standstill=True, brake=brake, long_active=active, enabled=active)))
  wires = [step(False, True, False), step(False, True, False), step(False, True, True), step(True, False, True)]
  assert wires[:3] == [0.0, 0.0, 0.0] and hook.state == "LOCKED" and lc.id_hook_out.floor is None
  assert wires[3] > ih.A_HOLD and not lc.id_hook_out.stop_intent and hook.done == ZERO


def test_an_interrupt_without_a_driver_action_keeps_the_hold():
  hook, o = start()
  to_stop(hook, o)
  hold(hook, 150)
  o = hook.interrupt(driver=False)
  assert o.floor == hook._last_cmd and o.stop_intent and hook.state == "HELD" and hook._locked == "fault"


@pytest.mark.parametrize("fail", ["mkstemp", "unlink"])
def test_a_progress_save_error_is_logged_and_does_not_escape(monkeypatch, tmp_path, fail):
  # re-review 20260926-161602: a mkstemp error escaped the worker; an unlink error escaped the cleanup
  import errno
  from openpilot.selfdrive.controls import controlsd
  monkeypatch.setattr(controlsd, "ID_PROGRESS_FILE", str(tmp_path / "identification_progress.json"))
  monkeypatch.setattr(controlsd, "_id_progress_saved", [0])
  logged = []
  monkeypatch.setattr(controlsd.cloudlog, "exception", logged.append)

  def boom(*a, **k):
    raise OSError(errno.ENOSPC, "no space")
  if fail == "mkstemp":
    monkeypatch.setattr(controlsd.tempfile, "mkstemp", boom)
  else:
    monkeypatch.setattr(controlsd.os, "replace", boom)
    monkeypatch.setattr(controlsd.os, "unlink", boom)
  controlsd._save_id_progress({"plan": "KCS2", "done": {"G": 1}}, 1)
  assert logged == ["identification hook progress not saved"] and controlsd._id_progress_saved == [0]


def test_concurrent_progress_saves_keep_the_newest_record(monkeypatch, tmp_path):
  # finding 3: two writers shared one .tmp inode, so a late older save overwrote the newer counts
  import json
  import threading
  from openpilot.selfdrive.controls import controlsd
  path = tmp_path / "identification_progress.json"
  monkeypatch.setattr(controlsd, "ID_PROGRESS_FILE", str(path))
  monkeypatch.setattr(controlsd, "_id_progress_saved", [0])
  old, new = {"plan": "KCS2", "done": {"G": 1, "F": 0, "H": 0}}, {"plan": "KCS2", "done": {"G": 1, "F": 1, "H": 0}}
  entered, release, real_dump = threading.Event(), threading.Event(), json.dump

  def delayed_dump(record, f):
    if record is old:
      entered.set()
      assert release.wait(5)
    real_dump(record, f)
  monkeypatch.setattr(controlsd.json, "dump", delayed_dump)
  first = threading.Thread(target=controlsd._save_id_progress, args=(old, 1))
  first.start()
  assert entered.wait(5)
  second = threading.Thread(target=controlsd._save_id_progress, args=(new, 2))
  second.start()                                                  # waits for the writer lock
  release.set()
  first.join(5)
  second.join(5)
  assert json.loads(path.read_text()) == new and sorted(p.name for p in tmp_path.iterdir()) == ["identification_progress.json"]
  controlsd._save_id_progress(old, 1)                             # an older save that runs late is dropped
  assert json.loads(path.read_text()) == new
