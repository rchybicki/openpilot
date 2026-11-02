from types import SimpleNamespace

import pytest

from cereal import car, custom
from openpilot.frogpilot.common.frogpilot_variables import BUTTON_FUNCTIONS
from openpilot.frogpilot.controls import frogpilot_card
from openpilot.frogpilot.controls.frogpilot_card import FrogPilotCard
from openpilot.selfdrive.car.cruise import CRUISE_LONG_PRESS
from openpilot.selfdrive.selfdrived.events import Events
from openpilot.selfdrive.selfdrived.selfdrived import SelfdriveD

ButtonType = car.CarState.ButtonEvent.Type

# the device: distance = force coast, long = personality profile, very long = traffic mode, LKAS = experimental mode
DEVICE = {"distance": "FORCE_COAST", "long": "PERSONALITY_PROFILE", "very_long": "TRAFFIC_MODE", "lkas": "EXPERIMENTAL_MODE"}
DISTANCE_ONLY = {"distance": "PERSONALITY_PROFILE", "long": "NOTHING", "very_long": "NOTHING", "lkas": "NOTHING"}

SHORT, LONG, VERY_LONG = CRUISE_LONG_PRESS // 2, CRUISE_LONG_PRESS * 2, CRUISE_LONG_PRESS * 6


def press(monkeypatch, functions, frames):
  # one press of the distance button through the card and selfdrived, frame by frame; returns the personality changes and the
  # Traffic Mode toggles
  monkeypatch.setattr(frogpilot_card.messaging, "drain_sock", lambda sock: [])
  toggles = SimpleNamespace(always_on_lateral_main=False, always_on_lateral_lkas=False, always_on_lateral_pause_speed=0, conditional_experimental_mode=False,
                            distance_button_function=BUTTON_FUNCTIONS[functions["distance"]],
                            distance_long_button_function=BUTTON_FUNCTIONS[functions["long"]],
                            distance_very_long_button_function=BUTTON_FUNCTIONS[functions["very_long"]],
                            lkas_button_function=BUTTON_FUNCTIONS[functions["lkas"]])

  card = FrogPilotCard.__new__(FrogPilotCard)
  card.CP = SimpleNamespace(brand="hyundai")
  card.always_on_lateral_allowed = card.always_on_lateral_set = card.frogs_go_moo = False
  card.distance_pressed_previously = card.force_coast = card.onroad_distance_button = False
  card.pause_lateral = card.pause_longitudinal = card.traffic_mode_enabled = False
  card.accel_press_count = card.decel_press_count = card.experimental_mode_press_count = card.gap_counter = card.lkas_button_press_count = 0
  card.long_press_threshold = CRUISE_LONG_PRESS
  card.very_long_press_threshold = CRUISE_LONG_PRESS * 5
  card.ui_event_sock = None
  card_sm = {"carControl": car.CarControl.new_message(longActive=True), "frogpilotPlan": SimpleNamespace(lateralCheck=True),
             "frogpilotSelfdriveState": SimpleNamespace(alertType=""), "liveCalibration": SimpleNamespace(calPerc=100),
             "selfdriveState": SimpleNamespace(alertType="", experimentalMode=False)}

  personality_changes = []
  sd = SelfdriveD.__new__(SelfdriveD)
  sd.frogpilot_toggles = toggles
  sd.distance_pressed_previously = False
  sd.display_timer = 0
  sd.has_menu = False
  sd.personality = 1
  sd.events = Events()
  sd.set_personality = personality_changes.append

  traffic_mode_toggles = 0
  pressed_frames = [True] * frames + [False] * 10
  for i, pressed in enumerate(pressed_frames):
    CS = car.CarState.new_message(gearShifter=car.CarState.GearShifter.drive, vEgo=10)
    if pressed != (pressed_frames[i - 1] if i else False):
      CS.buttonEvents = [car.CarState.ButtonEvent(pressed=pressed, type=ButtonType.gapAdjustCruise)]

    traffic_mode = card.traffic_mode_enabled
    sd.sm = {"frogpilotCarState": card.update(CS, custom.FrogPilotCarState.new_message(distancePressed=pressed), card_sm, toggles)}
    traffic_mode_toggles += card.traffic_mode_enabled != traffic_mode
    sd.update_personality_buttons(CS)

  return len(personality_changes), traffic_mode_toggles


# today's behaviour (c022a9890c:selfdrive/selfdrived/selfdrived.py:482-499): the distance release counts only when neither long flag is
# set, whatever the long functions are, and a press held to the very long threshold ends the long press there, so it cycles the
# personality once (long = personality profile) and toggles Traffic Mode (very long = traffic mode)
@pytest.mark.parametrize("functions,frames,expected", [
  (DEVICE, SHORT, (0, 0)),
  (DEVICE, LONG, (1, 0)),
  (DEVICE, VERY_LONG, (1, 1)),
  (DISTANCE_ONLY, SHORT, (1, 0)),
  (DISTANCE_ONLY, LONG, (0, 0)),
  (DISTANCE_ONLY, VERY_LONG, (0, 0)),
], ids=["device-short", "device-long", "device-very-long", "distance-only-short", "distance-only-long", "distance-only-very-long"])
def test_distance_button_presses(monkeypatch, functions, frames, expected):
  assert press(monkeypatch, functions, frames) == expected
