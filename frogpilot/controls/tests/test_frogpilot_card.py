from types import SimpleNamespace

import pytest

from cereal import car, custom
from openpilot.frogpilot.common.frogpilot_variables import BUTTON_FUNCTIONS
from openpilot.frogpilot.controls import frogpilot_card
from openpilot.frogpilot.controls.frogpilot_card import FrogPilotCard
from openpilot.selfdrive.selfdrived.events import ET, EVENT_NAME, EVENTS, FROGPILOT_EVENT_NAME, FROGPILOT_EVENTS, Events


def make_card():
  # the constructor opens a messaging socket; these tests drive the button logic only
  card = FrogPilotCard.__new__(FrogPilotCard)
  card.force_coast = False
  card.pause_lateral = False
  card.pause_longitudinal = False
  card.traffic_mode_enabled = False
  card.experimental_mode_press_count = 0
  return card


def sm_with(long_active):
  return {"carControl": SimpleNamespace(longActive=long_active), "selfdriveState": SimpleNamespace(experimentalMode=False)}


@pytest.mark.parametrize("long_active", [False, True])
def test_traffic_mode_toggles_with_or_without_longitudinal_control(long_active):
  # today's behaviour (c022a9890c:frogpilot/controls/frogpilot_card.py:46): the Traffic Mode button works whatever longActive is
  card = make_card()
  toggles = SimpleNamespace(conditional_experimental_mode=False)
  card.handle_button_event(BUTTON_FUNCTIONS["TRAFFIC_MODE"], sm_with(long_active), toggles)
  assert card.traffic_mode_enabled
  card.handle_button_event(BUTTON_FUNCTIONS["TRAFFIC_MODE"], sm_with(long_active), toggles)
  assert not card.traffic_mode_enabled


def make_driving_card():
  card = make_card()
  card.CP = SimpleNamespace(brand="hyundai")
  card.always_on_lateral_allowed = False
  card.always_on_lateral_set = True
  card.distance_pressed_previously = False
  card.frogs_go_moo = False
  card.onroad_distance_button = False
  card.ui_event_sock = None
  card.accel_press_count = 0
  card.decel_press_count = 0
  card.gap_counter = 0
  card.lkas_button_press_count = 0
  card.long_press_threshold = 50
  card.very_long_press_threshold = 250
  return card


EVENT_ALERTS = [(False, e, et) for e in EVENTS for et in EVENTS[e]] + [(True, e, et) for e in FROGPILOT_EVENTS for et in FROGPILOT_EVENTS[e]]


@pytest.mark.parametrize("frogpilot,event,event_type", EVENT_ALERTS,
                         ids=[f"{(FROGPILOT_EVENT_NAME if fp else EVENT_NAME)[e]}/{et}" for fp, e, et in EVENT_ALERTS])
def test_always_on_lateral_pauses_only_on_an_immediate_disable_alert(monkeypatch, tmp_path, frogpilot, event, event_type):
  # today's gate (c022a9890c:frogpilot/controls/frogpilot_card.py:74): always on lateral pauses while the shown alert, openpilot's or
  # FrogPilot's, is an immediate disable alert; soft disable, no entry and warning alerts (tooDistracted included) leave it on
  monkeypatch.setattr(frogpilot_card.messaging, "drain_sock", lambda sock: [])
  monkeypatch.setattr(frogpilot_card.frogpilot_paths, "ERROR_LOG_PATH", tmp_path / "error.txt")

  events = Events(frogpilot=frogpilot)
  events.add(event)
  alert_type = f"{events.event_names[event]}/{event_type}"
  onroad_events = [] if frogpilot else events.to_msg()

  sm = {
    "carControl": car.CarControl.new_message(),
    "frogpilotPlan": SimpleNamespace(lateralCheck=True),
    "frogpilotSelfdriveState": SimpleNamespace(alertType=alert_type if frogpilot else "", hasDisableEvents=events.contains_disable_event()),
    "liveCalibration": SimpleNamespace(calPerc=100),
    "onroadEvents": onroad_events,
    "selfdriveState": SimpleNamespace(alertType="" if frogpilot else alert_type, experimentalMode=False),
  }
  toggles = SimpleNamespace(always_on_lateral_main=True, always_on_lateral_lkas=False, always_on_lateral_pause_speed=0,
                            distance_button_function=BUTTON_FUNCTIONS["NOTHING"], lkas_button_function=BUTTON_FUNCTIONS["NOTHING"])
  CS = car.CarState.new_message(gearShifter=car.CarState.GearShifter.drive, vEgo=20)
  CS.cruiseState.available = True

  FPCS = make_driving_card().update(CS, custom.FrogPilotCarState.new_message(), sm, toggles)
  assert FPCS.alwaysOnLateralEnabled == (event_type != ET.IMMEDIATE_DISABLE)
