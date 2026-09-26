"""TEMPORARY identification test scope (frogpilot_variables identification_mode = master flag + Santa Fe HEV + openpilot
longitudinal): all three distance mappings act as NOTHING, the physical wheel button only, canonical Standard, Traffic
off, saved Params untouched. Real FrogPilotVariables, FrogPilotCard, Car.state_update, SelfdriveD and Controls
functions on the temporary Params prefix, with the user's saved 2/1/6 mappings (LKAS = personality)."""
import json
from types import SimpleNamespace

import pytest

import cereal.messaging as messaging
from cereal import car, custom, log
from openpilot.common.params import Params
from openpilot.frogpilot.common import frogpilot_variables as fpv
from openpilot.frogpilot.controls import frogpilot_card as fpc
from openpilot.selfdrive.car import card as card_mod
from openpilot.selfdrive.controls.lib import stopping_flags
from openpilot.selfdrive.car.cruise import VCruiseHelper
from openpilot.selfdrive.controls.lib.identification_hook import SET_SPEED_KPH, IdentificationHook, PressTimer
from openpilot.selfdrive.controls.tests.test_identification_hook import _controls_inputs
from openpilot.selfdrive.selfdrived.selfdrived import SelfdriveD

SAVED = {"DistanceButtonControl": 2, "LongDistanceButtonControl": 1, "VeryLongDistanceButtonControl": 6, "LKASButtonControl": 1,
         "InitialSetSpeed": 160}
ACTIONS = ("experimental_mode", "force_coast", "pause_lateral", "pause_longitudinal", "personality_profile", "traffic_mode")
PRESSES = {"short": 10, "long": 60, "very_long": 260}   # frames; CRUISE_LONG_PRESS = 50, very long = 250


@pytest.fixture
def params(monkeypatch):
  for mod in (fpv, fpc):   # no /dev/shm on the host: memory Params share the temporary prefix store
    monkeypatch.setattr(mod, "Params", lambda *a, memory=False, **k: Params(*a, **k))
  p = Params()
  for k, v in SAVED.items():
    p.put(k, v)
  p.put("LongitudinalPersonality", log.LongitudinalPersonality.aggressive)
  return p


def _toggles(monkeypatch, params, flag=True, fingerprint="HYUNDAI_SANTA_FE_HEV_2022", long=True):
  monkeypatch.setattr(stopping_flags, "IDENTIFICATION_HOOK", flag)
  cp = car.CarParams.new_message(carFingerprint=fingerprint, openpilotLongitudinalControl=long, brand="hyundai")
  params.put("CarParamsPersistent", cp.to_bytes())
  return fpv.FrogPilotVariables().frogpilot_toggles


def _distance_actions(t):
  return {f"{a}_via_distance{p}" for a in ACTIONS for p in ("", "_long", "_very_long") if getattr(t, f"{a}_via_distance{p}")}


class _SM(dict):
  def __init__(self):
    super().__init__({s: getattr(messaging.new_message(s).as_reader(), s) for s in
                      ("frogpilotPlan", "liveCalibration", "selfdriveState", "frogpilotSelfdriveState")})
    self["carControl"] = car.CarControl.new_message(enabled=True, longActive=True).as_reader()
    self.updated = {"frogpilotPlan": False}

  def update(self, timeout):
    pass


def _card():
  return fpc.FrogPilotCard(car.CarParams.new_message(brand="hyundai").as_reader(), custom.FrogPilotCarParams.new_message())


def _press(card, toggles, frames, settle=5, after=5):
  outs = []
  for k in range(settle + frames + after):
    CS = car.CarState.new_message(gearShifter=car.CarState.GearShifter.drive, vEgo=8.3).as_reader()
    outs.append(card.update(CS, custom.FrogPilotCarState.new_message(distancePressed=settle <= k < settle + frames), _SM(), toggles))
  return outs


def test_scope_maps_every_distance_press_and_the_lkas_personality_to_nothing_without_saving(monkeypatch, params):
  t = _toggles(monkeypatch, params)
  assert t.identification_mode and _distance_actions(t) == set() and not t.personality_profile_via_lkas and not t.experimental_mode_via_press
  assert {k: params.get(k) for k in SAVED} == SAVED


@pytest.mark.parametrize("kw,initial", [(dict(), SET_SPEED_KPH), (dict(flag=False), 160), (dict(fingerprint="HYUNDAI_ELANTRA_2021"), 160)])
def test_the_scope_starts_cruise_at_the_test_speed_without_saving(monkeypatch, params, kw, initial):
  assert _toggles(monkeypatch, params, **kw).initial_set_speed == initial and params.get("InitialSetSpeed") == 160


@pytest.mark.parametrize("kw", [dict(flag=False), dict(fingerprint="HYUNDAI_ELANTRA_2021"), dict(long=False)])
def test_outside_the_scope_the_saved_mappings_are_unchanged(monkeypatch, params, kw):
  t = _toggles(monkeypatch, params, **kw)
  normal = {"force_coast_via_distance", "personality_profile_via_distance_long", "traffic_mode_via_distance_very_long"}
  assert not t.identification_mode and t.personality_profile_via_lkas == t.openpilot_longitudinal
  assert _distance_actions(t) == (normal if t.openpilot_longitudinal else set())


@pytest.mark.parametrize("press", PRESSES)
def test_physical_presses_in_scope_fire_no_action_and_keep_the_classification(monkeypatch, params, press):
  outs = _press(_card(), _toggles(monkeypatch, params), PRESSES[press])
  assert not any(o.forceCoast or o.pauseLateral or o.pauseLongitudinal or o.trafficModeEnabled for o in outs)
  assert any(o.distanceLongPressed for o in outs) == (press != "short") and any(o.distanceVeryLongPressed for o in outs) == (press == "very_long")
  assert not params.get_bool("ExperimentalMode") and params.get("LongitudinalPersonality") == log.LongitudinalPersonality.aggressive


def test_the_same_presses_act_normally_outside_the_scope(monkeypatch, params):
  t = _toggles(monkeypatch, params, flag=False)
  assert _press(_card(), t, PRESSES["short"])[-1].forceCoast                  # 2 = Force Coast
  assert _press(_card(), t, PRESSES["very_long"])[-1].trafficModeEnabled      # 6 = Traffic


def test_the_on_screen_button_is_ignored_only_in_scope(monkeypatch, params):
  params.put_bool("OnroadDistanceButtonPressed", True)
  assert not any(o.distancePressed for o in _press(_card(), _toggles(monkeypatch, params), 0, settle=0, after=5))
  assert all(o.distancePressed for o in _press(_card(), _toggles(monkeypatch, params, flag=False), 0, settle=0, after=5))


@pytest.mark.parametrize("physical,expected", [(False, False), (True, True), (None, False)], ids=["screen_only", "wheel", "missing"])
def test_car_state_update_restores_the_physical_button_before_the_card(monkeypatch, params, physical, expected):
  # interfaces.py already ORed the on-screen button into distancePressed; the car interface keeps the raw wheel button
  car_obj = card_mod.Car.__new__(card_mod.Car)
  CS_raw = SimpleNamespace() if physical is None else SimpleNamespace(distance_button=physical)
  merged = custom.FrogPilotCarState.new_message(distancePressed=True)
  car_obj.__dict__.update(
    can_sock=None, CP=SimpleNamespace(brand="hyundai"), RI=SimpleNamespace(update=lambda can_list: None), sm=_SM(), can_rcv_cum_timeout_counter=0,
    CI=SimpleNamespace(CS=CS_raw, update=lambda can_list, toggles: (car.CarState.new_message(), merged)), is_metric=True,
    v_cruise_helper=SimpleNamespace(update_v_cruise=lambda *a: None, v_cruise_kph=30.0, v_cruise_cluster_kph=30.0),
    CC_prev=SimpleNamespace(enabled=True), resume_prev_button=False, frogpilot_card=_card(), id_press=PressTimer(), live_update_handoff_pressed_buttons=set(),
    live_update_handoff_state="", frogpilot_toggles=_toggles(monkeypatch, params))
  monkeypatch.setattr(card_mod.messaging, "drain_sock_raw", lambda sock, wait_for_one: [])
  assert car_obj.state_update()[2].distancePressed == expected
  normal = _toggles(monkeypatch, params, flag=False)
  older = SimpleNamespace(**{k: v for k, v in vars(normal).items() if k != "identification_mode"})   # older serialized toggles
  for car_obj.frogpilot_toggles in (normal, older):
    merged.distancePressed = True
    assert car_obj.state_update()[2].distancePressed                       # outside the scope the merged input stays


class _RecordingParams:
  def __init__(self, params):
    self.params, self.writes = params, []

  def get(self, *a, **k):
    return self.params.get(*a, **k)

  def put_nonblocking(self, key, value):
    self.writes.append((key, value))


def _selfdrived(params, toggles):
  return SimpleNamespace(params=_RecordingParams(params), frogpilot_toggles=toggles, personality=log.LongitudinalPersonality.relaxed,
                         personality_param_write_value=None, personality_param_write_t=0.0)


def test_selfdrived_keeps_standard_in_scope_without_reading_or_writing_the_saved_personality(params):
  sd = _selfdrived(params, SimpleNamespace(identification_mode=True))
  SelfdriveD.update_personality_from_params(sd)                       # background refresh (a UI write lands in Params)
  assert sd.personality == log.LongitudinalPersonality.standard
  SelfdriveD.set_personality(sd, log.LongitudinalPersonality.aggressive)   # the only setter (distance/LKAS decrement)
  assert sd.personality == log.LongitudinalPersonality.standard and sd.params.writes == []
  assert params.get("LongitudinalPersonality") == log.LongitudinalPersonality.aggressive


@pytest.mark.parametrize("toggles", [SimpleNamespace(identification_mode=False), SimpleNamespace()], ids=["normal", "older_toggles"])
def test_selfdrived_uses_and_saves_the_personality_outside_the_scope(params, toggles):
  sd = _selfdrived(params, toggles)
  SelfdriveD.update_personality_from_params(sd)
  assert sd.personality == log.LongitudinalPersonality.aggressive
  SelfdriveD.set_personality(sd, log.LongitudinalPersonality.standard)
  assert sd.personality == log.LongitudinalPersonality.standard and sd.params.writes == [("LongitudinalPersonality", log.LongitudinalPersonality.standard)]


def _chain(hook, card, t, frames, settle=5, after=5):
  # physical press -> FrogPilotCard classification -> Controls._identification_inputs -> hook
  return [hook.update(_controls_inputs([0.02, 0.01], fcs=fcs, toggles=t), 0.0) for fcs in _press(card, t, frames, settle=settle, after=after)]


@pytest.mark.parametrize("press", PRESSES)
def test_only_a_long_physical_press_arms_through_the_real_chain(monkeypatch, params, press):
  t, hook = _toggles(monkeypatch, params), IdentificationHook()
  outs = _chain(hook, _card(), t, PRESSES[press])
  assert hook.state in (("OFF",) if press == "short" else ("ARMED", "READY")) and not any(o.active for o in outs)


def test_a_long_hold_with_a_dropout_turns_test_mode_off_through_the_real_chain(monkeypatch, params):
  # review reproduction: the card resets its classification in the 40 ms dropout; the hook's own timer must not
  t, hook, card = _toggles(monkeypatch, params), IdentificationHook(), _card()
  _chain(hook, card, t, PRESSES["long"])
  _chain(hook, card, t, 0, settle=250, after=0)
  outs = _chain(hook, card, t, 24, settle=0, after=4) + _chain(hook, card, t, 24, settle=0, after=5)
  assert not any(o.active for o in outs) and hook.state == "OFF" and hook.trial == 0


@pytest.mark.parametrize("press", PRESSES)
def test_once_armed_only_a_short_physical_press_starts_and_a_longer_one_turns_test_mode_off(monkeypatch, params, press):
  t, hook, card = _toggles(monkeypatch, params), IdentificationHook(), _card()
  _chain(hook, card, t, PRESSES["long"])
  outs = _chain(hook, card, t, PRESSES[press], settle=250)      # READY after 2 s
  assert any(o.active for o in outs) == (press == "short") and hook.state == ("ACTIVE" if press == "short" else "OFF")


@pytest.mark.parametrize("flag", [True, False])
def test_traffic_stays_off_in_scope_even_from_an_lkas_mapping(monkeypatch, params, flag):
  params.put("LKASButtonControl", 6)   # Traffic would override the Standard gap/jerk
  t, card = _toggles(monkeypatch, params, flag=flag), _card()
  lkas = [car.CarState.ButtonEvent.new_message(pressed=True, type=car.CarState.ButtonEvent.Type.lkas)]
  CS = car.CarState.new_message(gearShifter=car.CarState.GearShifter.drive, vEgo=8.3, buttonEvents=lkas).as_reader()
  assert card.update(CS, custom.FrogPilotCarState.new_message(), _SM(), t).trafficModeEnabled == (not flag)


def test_older_serialized_toggles_without_the_field_act_normally_and_the_hook_fails_closed(monkeypatch, params):
  # a frogpilotPlan JSON from a build without identification_mode (replay fixture, stale publisher): real decode path
  current = vars(_toggles(monkeypatch, params, flag=False)).copy()
  current.pop("identification_mode")
  old = fpv.process_frogpilot_toggles(json.dumps(current))
  assert not hasattr(old, "identification_mode")
  assert _press(_card(), old, PRESSES["short"])[-1].forceCoast                 # FrogPilotCard: the saved 2 = Force Coast
  params.put_bool("OnroadDistanceButtonPressed", True)
  assert all(o.distancePressed for o in _press(_card(), old, 0, settle=0, after=3))   # the on-screen merge stays
  assert _controls_inputs([0.02, 0.01], toggles=old).mapping_ok is False       # controlsd: no test without the scope


class _RecordingPM:
  def __init__(self):
    self.sent = {}

  def send(self, service, msg):
    self.sent[service] = msg


@pytest.mark.parametrize("toggles,published", [
  (SimpleNamespace(identification_mode=True), log.LongitudinalPersonality.standard),
  (SimpleNamespace(identification_mode=False), log.LongitudinalPersonality.relaxed),
  (SimpleNamespace(), log.LongitudinalPersonality.relaxed),
], ids=["scope", "normal", "older_toggles"])
def test_published_selfdrive_state_is_standard_in_scope_even_with_a_stale_cached_personality(toggles, published):
  # the cache still holds relaxed: the scope began after startup and the 100 ms Params reader has not refreshed it yet
  from openpilot.selfdrive.selfdrived.alertmanager import AlertManager
  from openpilot.selfdrive.selfdrived.events import Events
  from openpilot.selfdrive.selfdrived.state import StateMachine
  sd = SimpleNamespace(enabled=True, active=True, state_machine=StateMachine(), live_update_handoff_state="", events=Events(),
                       frogpilot_events=Events(frogpilot=True), experimental_mode=False, personality=log.LongitudinalPersonality.relaxed,
                       AM=AlertManager(), frogpilot_AM=AlertManager(), pm=_RecordingPM(), sm=SimpleNamespace(frame=1), events_prev=[],
                       frogpilot_events_prev=[], frogpilot_toggles=toggles)
  SelfdriveD.publish_selfdriveState(sd, car.CarState.new_message())
  assert sd.pm.sent["selfdriveState"].selfdriveState.personality == published
  assert sd.personality == log.LongitudinalPersonality.relaxed                  # the cache itself is untouched


def _cruising_car(monkeypatch, params, enabled=True, flag=True):
  # a real Car.state_update with a real VCruiseHelper (openpilot longitudinal: non-PCM set speed), cruising at 50 km/h
  car_obj = card_mod.Car.__new__(card_mod.Car)
  CP = car.CarParams.new_message(carFingerprint="HYUNDAI_SANTA_FE_HEV_2022", pcmCruise=False, brand="hyundai")
  wheel = SimpleNamespace(distance_button=False)
  sm = _SM()
  sm["carControl"] = car.CarControl.new_message(enabled=enabled, longActive=enabled).as_reader()

  def ci_update(can_list, toggles):
    CS = car.CarState.new_message(vEgo=50 / 3.6, gearShifter=car.CarState.GearShifter.drive)
    CS.cruiseState.available = True
    return CS, custom.FrogPilotCarState.new_message(distancePressed=wheel.distance_button)
  car_obj.__dict__.update(
    can_sock=None, CP=CP, RI=SimpleNamespace(update=lambda can_list: None), sm=sm, can_rcv_cum_timeout_counter=0,
    CI=SimpleNamespace(CS=wheel, update=ci_update), is_metric=True, v_cruise_helper=VCruiseHelper(CP), CC_prev=SimpleNamespace(enabled=enabled),
    resume_prev_button=False, frogpilot_card=_card(), id_press=PressTimer(), live_update_handoff_pressed_buttons=set(),
    live_update_handoff_state="", frogpilot_toggles=_toggles(monkeypatch, params, flag=flag))
  car_obj.v_cruise_helper.v_cruise_kph = car_obj.v_cruise_helper.v_cruise_cluster_kph = 50
  monkeypatch.setattr(card_mod.messaging, "drain_sock_raw", lambda sock, wait_for_one: [])

  def buttons(pattern):
    speeds = []
    for pressed in pattern:
      wheel.distance_button = pressed
      speeds.append(car_obj.state_update()[0].vCruise)
    return speeds
  return car_obj, buttons


RELEASED = [False] * 5   # the hook and card need an observed release before a press counts


@pytest.mark.parametrize("press", PRESSES)
@pytest.mark.parametrize("enabled", [True, False], ids=["engaged", "disengaged"])
def test_a_long_press_sets_the_test_speed_engaged_or_not(monkeypatch, params, press, enabled):
  car_obj, buttons = _cruising_car(monkeypatch, params, enabled=enabled)
  speeds = buttons(RELEASED + [True] * PRESSES[press] + RELEASED)
  if press == "short":
    assert set(speeds) == {50}
  else:                                                          # set on the 0.5 s mark, shown from the next frame
    assert speeds[:55] == [50] * 55 and set(speeds[55:]) == {SET_SPEED_KPH}
    assert car_obj.v_cruise_helper.v_cruise_cluster_kph == SET_SPEED_KPH


def test_a_dropout_inside_the_hold_still_sets_the_test_speed(monkeypatch, params):
  # review reproduction: 240 ms, 40 ms dropout, 240 ms - the hook arms, so card must set 30 too
  car_obj, buttons = _cruising_car(monkeypatch, params)
  buttons(RELEASED + [True] * 24 + [False] * 4 + [True] * 24 + RELEASED)
  assert car_obj.v_cruise_helper.v_cruise_kph == SET_SPEED_KPH


def test_a_press_held_through_startup_leaves_the_set_speed(monkeypatch, params):
  car_obj, buttons = _cruising_car(monkeypatch, params)
  buttons([True] * 300 + RELEASED)
  assert car_obj.v_cruise_helper.v_cruise_kph == 50


def test_a_long_press_outside_the_scope_leaves_the_set_speed(monkeypatch, params):
  car_obj, buttons = _cruising_car(monkeypatch, params, flag=False)
  buttons(RELEASED + [True] * PRESSES["long"] + RELEASED)
  assert car_obj.v_cruise_helper.v_cruise_kph == 50


def test_resume_after_a_disengaged_long_press_returns_to_the_test_speed(monkeypatch, params):
  # review reproduction: RESUME takes the remembered set speed, so the long press must set it while disengaged too
  car_obj, buttons = _cruising_car(monkeypatch, params, enabled=False)
  buttons(RELEASED + [True] * PRESSES["long"] + RELEASED)
  resume = car.CarState.new_message(vEgo=20 / 3.6, buttonEvents=[car.CarState.ButtonEvent.new_message(type=car.CarState.ButtonEvent.Type.resumeCruise)])
  car_obj.v_cruise_helper.initialize_v_cruise(resume, False, False, car_obj.frogpilot_toggles)
  assert car_obj.v_cruise_helper.v_cruise_kph == SET_SPEED_KPH


@pytest.mark.parametrize("v_kph,expected", [(0, SET_SPEED_KPH), (20, SET_SPEED_KPH), (45, 45)])
def test_engaging_in_the_scope_starts_at_the_test_speed_or_the_current_speed(monkeypatch, params, v_kph, expected):
  helper = VCruiseHelper(car.CarParams.new_message(carFingerprint="HYUNDAI_SANTA_FE_HEV_2022", pcmCruise=False))
  helper.initialize_v_cruise(car.CarState.new_message(vEgo=v_kph / 3.6), False, False, _toggles(monkeypatch, params))
  assert helper.v_cruise_kph == expected
