import itertools
import math
from types import SimpleNamespace

import numpy as np
import pytest

from parameterized import parameterized_class
from cereal import log
from openpilot.selfdrive.car.cruise import CRUISE_LONG_PRESS, VCruiseHelper, V_CRUISE_MIN, V_CRUISE_MAX, V_CRUISE_INITIAL, IMPERIAL_INCREMENT
from cereal import car
from openpilot.common.constants import CV
from openpilot.selfdrive.test.longitudinal_maneuvers.maneuver import Maneuver

ButtonEvent = car.CarState.ButtonEvent
ButtonType = car.CarState.ButtonEvent.Type


def run_cruise_simulation(cruise, e2e, personality, t_end=20.):
  man = Maneuver(
    '',
    duration=t_end,
    initial_speed=max(cruise - 1., 0.0),
    lead_relevancy=True,
    initial_distance_lead=100,
    cruise_values=[cruise],
    prob_lead_values=[0.0],
    breakpoints=[0.],
    e2e=e2e,
    personality=personality,
  )
  valid, output = man.evaluate()
  assert valid
  return output[-1, 3]


@parameterized_class(("e2e", "personality", "speed"), itertools.product(
                      [True, False], # e2e
                      log.LongitudinalPersonality.schema.enumerants, # personality
                      [5,35])) # speed
class TestCruiseSpeed:
  def test_cruise_speed(self):
    print(f'Testing {self.speed} m/s')
    cruise_speed = float(self.speed)

    simulation_steady_state = run_cruise_simulation(cruise_speed, self.e2e, self.personality)
    assert simulation_steady_state == pytest.approx(cruise_speed, abs=.01), f'Did not reach {self.speed} m/s'


# TODO: test pcmCruise
@parameterized_class(('pcm_cruise',), [(False,)])
class TestVCruiseHelper:
  def setup_method(self):
    self.CP = car.CarParams(pcmCruise=self.pcm_cruise)
    self.v_cruise_helper = VCruiseHelper(self.CP)
    self.frogpilot_toggles = SimpleNamespace(conditional_experimental_mode=False, cruise_increase=1, cruise_increase_long=5,
                                             set_speed_offset=0)
    self.reset_cruise_speed_state()

  def reset_cruise_speed_state(self):
    # Two resets previous cruise speed
    for _ in range(2):
      self.v_cruise_helper.update_v_cruise(car.CarState(cruiseState={"available": False}), enabled=False, is_metric=False,
                                           frogpilot_toggles=self.frogpilot_toggles)

  def enable(self, v_ego, experimental_mode):
    # Simulates user pressing set with a current speed
    self.v_cruise_helper.initialize_v_cruise(car.CarState(vEgo=v_ego), experimental_mode, False, self.frogpilot_toggles)

  def press_button(self, button_type, v_ego_kph, gas_pressed=False, long_press=False):
    CS = car.CarState(vEgo=v_ego_kph * CV.KPH_TO_MS, gasPressed=gas_pressed, cruiseState={"available": True})
    CS.buttonEvents = [ButtonEvent(type=button_type, pressed=True)]
    self.v_cruise_helper.update_v_cruise(CS, enabled=True, is_metric=True, frogpilot_toggles=self.frogpilot_toggles)

    if long_press:
      self.v_cruise_helper.button_timers[button_type] = CRUISE_LONG_PRESS
      CS.buttonEvents = []
    else:
      CS.buttonEvents = [ButtonEvent(type=button_type, pressed=False)]

    self.v_cruise_helper.update_v_cruise(CS, enabled=True, is_metric=True, frogpilot_toggles=self.frogpilot_toggles)

  def hold_button(self, button_type, v_ego_kph, repeat_count=2):
    CS = car.CarState(vEgo=v_ego_kph * CV.KPH_TO_MS, cruiseState={"available": True})
    CS.buttonEvents = [ButtonEvent(type=button_type, pressed=True)]
    self.v_cruise_helper.update_v_cruise(CS, enabled=True, is_metric=True, frogpilot_toggles=self.frogpilot_toggles)

    CS.buttonEvents = []
    for repeat in range(1, repeat_count + 1):
      self.v_cruise_helper.button_timers[button_type] = CRUISE_LONG_PRESS * repeat
      self.v_cruise_helper.update_v_cruise(CS, enabled=True, is_metric=True, frogpilot_toggles=self.frogpilot_toggles)

  def test_adjust_speed(self):
    """
    Asserts speed changes on falling edges of buttons.
    """

    self.enable(V_CRUISE_INITIAL * CV.KPH_TO_MS, False)

    for btn in (ButtonType.accelCruise, ButtonType.decelCruise):
      for pressed in (True, False):
        CS = car.CarState(cruiseState={"available": True})
        CS.buttonEvents = [ButtonEvent(type=btn, pressed=pressed)]

        self.v_cruise_helper.update_v_cruise(CS, enabled=True, is_metric=False, frogpilot_toggles=self.frogpilot_toggles)
        assert pressed == (self.v_cruise_helper.v_cruise_kph == self.v_cruise_helper.v_cruise_kph_last)

  @pytest.mark.parametrize(("v_cruise_kph", "v_ego_kph", "button_type", "long_press", "expected_v_cruise_kph"), [
    (160, 95, ButtonType.decelCruise, False, 100),
    (160, 90, ButtonType.decelCruise, False, 100),
    (50, 80, ButtonType.accelCruise, False, 90),
    (160, 97, ButtonType.decelCruise, True, 100),
    (50, 80, ButtonType.accelCruise, True, 85),
    (160, 95, ButtonType.accelCruise, False, 170),
    (50, 80, ButtonType.decelCruise, False, 40),
  ])
  def test_adjust_speed_from_current_speed(self, v_cruise_kph, v_ego_kph, button_type, long_press, expected_v_cruise_kph):
    self.frogpilot_toggles.cruise_increase = 10
    self.v_cruise_helper.v_cruise_kph = v_cruise_kph

    self.press_button(button_type, v_ego_kph, long_press=long_press)

    assert self.v_cruise_helper.v_cruise_kph == expected_v_cruise_kph

  def test_decel_catchup_continues_normally(self):
    self.frogpilot_toggles.cruise_increase = 10
    self.v_cruise_helper.v_cruise_kph = 160

    self.press_button(ButtonType.decelCruise, 95)
    assert self.v_cruise_helper.v_cruise_kph == 100

    self.press_button(ButtonType.decelCruise, 95)
    assert self.v_cruise_helper.v_cruise_kph == 90

  @pytest.mark.parametrize(("v_cruise_kph", "v_ego_kph", "button_type", "expected_v_cruise_kph"), [
    (100, 100, ButtonType.accelCruise, 105),
    (100, 100, ButtonType.decelCruise, 95),
    (95, 100, ButtonType.accelCruise, 100),
    (105, 100, ButtonType.decelCruise, 100),
    (90, 100, ButtonType.accelCruise, 95),
    (110, 100, ButtonType.decelCruise, 105),
  ])
  def test_long_press_near_current_speed_changes_once(self, v_cruise_kph, v_ego_kph, button_type, expected_v_cruise_kph):
    self.frogpilot_toggles.cruise_increase = 10
    self.v_cruise_helper.v_cruise_kph = v_cruise_kph

    self.hold_button(button_type, v_ego_kph)

    assert self.v_cruise_helper.v_cruise_kph == expected_v_cruise_kph

  @pytest.mark.parametrize(("v_cruise_kph", "v_ego_kph", "button_type", "expected_v_cruise_kph"), [
    (50, 80, ButtonType.accelCruise, 85),
    (160, 97, ButtonType.decelCruise, 100),
  ])
  def test_long_press_far_from_current_speed_keeps_catchup(self, v_cruise_kph, v_ego_kph, button_type, expected_v_cruise_kph):
    self.frogpilot_toggles.cruise_increase = 10
    self.v_cruise_helper.v_cruise_kph = v_cruise_kph

    self.hold_button(button_type, v_ego_kph)

    assert self.v_cruise_helper.v_cruise_kph == expected_v_cruise_kph

  @pytest.mark.parametrize(("v_cruise_kph", "v_ego_kph", "expected_v_cruise_kph"), [
    (90, 95, 90),
    (90, 100, 90),
    (90, 100.1, 100),
    (90, 101, 100),
    (100, 99, 100),
    (120, 130, 120),
    (120, 130.1, 130),
    (120, 139.9, 130),
  ])
  def test_gas_override_bumps_cruise_after_grace_interval(self, v_cruise_kph, v_ego_kph, expected_v_cruise_kph):
    self.frogpilot_toggles.cruise_increase = 10
    self.v_cruise_helper.v_cruise_kph = v_cruise_kph
    CS = car.CarState(vEgo=v_ego_kph * CV.KPH_TO_MS, gasPressed=True, cruiseState={"available": True})

    self.v_cruise_helper.update_v_cruise(CS, enabled=True, is_metric=True, frogpilot_toggles=self.frogpilot_toggles)

    assert self.v_cruise_helper.v_cruise_kph == expected_v_cruise_kph

  def test_gas_override_tracks_acceleration_and_holds_after_release(self):
    self.frogpilot_toggles.cruise_increase = 10
    self.v_cruise_helper.v_cruise_kph = 90

    for v_ego_kph, gas_pressed, expected_v_cruise_kph in ((95, True, 90), (100, True, 90), (100.1, True, 100),
                                                           (108, True, 100), (108, False, 100)):
      CS = car.CarState(vEgo=v_ego_kph * CV.KPH_TO_MS, gasPressed=gas_pressed, cruiseState={"available": True})
      self.v_cruise_helper.update_v_cruise(CS, enabled=True, is_metric=True, frogpilot_toggles=self.frogpilot_toggles)
      assert self.v_cruise_helper.v_cruise_kph == expected_v_cruise_kph

  def test_gas_override_repeats_grace_interval_from_new_set_speed(self):
    self.frogpilot_toggles.cruise_increase = 10
    self.v_cruise_helper.v_cruise_kph = 120

    for v_ego_kph, gas_pressed, expected_v_cruise_kph in ((130, True, 120), (130.1, True, 130), (140, True, 130),
                                                           (140.1, True, 140), (140.1, False, 140)):
      CS = car.CarState(vEgo=v_ego_kph * CV.KPH_TO_MS, gasPressed=gas_pressed, cruiseState={"available": True})
      self.v_cruise_helper.update_v_cruise(CS, enabled=True, is_metric=True, frogpilot_toggles=self.frogpilot_toggles)
      assert self.v_cruise_helper.v_cruise_kph == expected_v_cruise_kph

  def test_gas_override_does_not_change_cruise_while_disengaged(self):
    self.frogpilot_toggles.cruise_increase = 10
    self.v_cruise_helper.v_cruise_kph = 90
    CS = car.CarState(vEgo=95 * CV.KPH_TO_MS, gasPressed=True, cruiseState={"available": True})

    self.v_cruise_helper.update_v_cruise(CS, enabled=False, is_metric=True, frogpilot_toggles=self.frogpilot_toggles)

    assert self.v_cruise_helper.v_cruise_kph == 90

  def test_rising_edge_enable(self):
    """
    Some car interfaces may enable on rising edge of a button,
    ensure we don't adjust speed if enabled changes mid-press.
    """

    # NOTE: enabled is always one frame behind the result from button press in controlsd
    for enabled, pressed in ((False, False),
                             (False, True),
                             (True, False)):
      CS = car.CarState(cruiseState={"available": True})
      CS.buttonEvents = [ButtonEvent(type=ButtonType.decelCruise, pressed=pressed)]
      self.v_cruise_helper.update_v_cruise(CS, enabled=enabled, is_metric=False, frogpilot_toggles=self.frogpilot_toggles)
      if pressed:
        self.enable(V_CRUISE_INITIAL * CV.KPH_TO_MS, False)

      # Expected diff on enabling. Speed should not change on falling edge of pressed
      assert not pressed == self.v_cruise_helper.v_cruise_kph == self.v_cruise_helper.v_cruise_kph_last

  def test_resume_in_standstill(self):
    """
    Asserts we don't increment set speed if user presses resume/accel to exit cruise standstill.
    """

    self.enable(0, False)

    for standstill in (True, False):
      for pressed in (True, False):
        CS = car.CarState(cruiseState={"available": True, "standstill": standstill})
        CS.buttonEvents = [ButtonEvent(type=ButtonType.accelCruise, pressed=pressed)]
        self.v_cruise_helper.update_v_cruise(CS, enabled=True, is_metric=False, frogpilot_toggles=self.frogpilot_toggles)

        # speed should only update if not at standstill and button falling edge
        should_equal = standstill or pressed
        assert should_equal == (self.v_cruise_helper.v_cruise_kph == self.v_cruise_helper.v_cruise_kph_last)

  def test_set_gas_pressed(self):
    """
    Asserts gas override raises a set speed below vEgo, while preserving
    the existing set-button behavior when the set speed is above vEgo.
    """

    for v_ego in np.linspace(0, 100, 101):
      self.reset_cruise_speed_state()
      self.enable(V_CRUISE_INITIAL * CV.KPH_TO_MS, False)

      # first decrement speed, then perform gas pressed logic
      v_ego_kph = round(v_ego * CV.MS_TO_KPH, 1)
      v_cruise_catchup_delta = IMPERIAL_INCREMENT * self.frogpilot_toggles.cruise_increase
      v_cruise_below_ego = math.floor(v_ego_kph / v_cruise_catchup_delta) * v_cruise_catchup_delta
      v_cruise_above_ego = (math.floor(v_ego_kph / v_cruise_catchup_delta) + 1) * v_cruise_catchup_delta
      gas_override_bump = V_CRUISE_MIN <= v_ego_kph and self.v_cruise_helper.v_cruise_kph + v_cruise_catchup_delta < v_ego_kph
      if gas_override_bump:
        expected_v_cruise_kph = v_cruise_below_ego
      elif V_CRUISE_MIN <= v_ego_kph and self.v_cruise_helper.v_cruise_kph > v_cruise_above_ego:
        expected_v_cruise_kph = v_cruise_above_ego
      else:
        expected_v_cruise_kph = self.v_cruise_helper.v_cruise_kph - IMPERIAL_INCREMENT
      if not gas_override_bump:
        expected_v_cruise_kph = max(expected_v_cruise_kph, v_ego_kph)  # clip to min of vEgo
      expected_v_cruise_kph = float(np.clip(round(expected_v_cruise_kph, 1), V_CRUISE_MIN, V_CRUISE_MAX))

      CS = car.CarState(vEgo=float(v_ego), gasPressed=True, cruiseState={"available": True})
      CS.buttonEvents = [ButtonEvent(type=ButtonType.decelCruise, pressed=False)]
      self.v_cruise_helper.update_v_cruise(CS, enabled=True, is_metric=False, frogpilot_toggles=self.frogpilot_toggles)

      # TODO: fix skipping first run due to enabled on rising edge exception
      if v_ego == 0.0:
        continue
      assert expected_v_cruise_kph == self.v_cruise_helper.v_cruise_kph

  def test_initialize_v_cruise(self):
    """
    Asserts allowed cruise speeds on enabling with SET.
    """

    for experimental_mode in (True, False):
      for v_ego in np.linspace(0, 100, 101):
        self.reset_cruise_speed_state()
        assert not self.v_cruise_helper.v_cruise_initialized

        self.enable(float(v_ego), experimental_mode)
        assert V_CRUISE_INITIAL <= self.v_cruise_helper.v_cruise_kph <= V_CRUISE_MAX
        assert self.v_cruise_helper.v_cruise_initialized
