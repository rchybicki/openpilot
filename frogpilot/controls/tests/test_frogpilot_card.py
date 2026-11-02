from types import SimpleNamespace

import pytest

from openpilot.frogpilot.common.frogpilot_variables import BUTTON_FUNCTIONS
from openpilot.frogpilot.controls.frogpilot_card import FrogPilotCard


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
