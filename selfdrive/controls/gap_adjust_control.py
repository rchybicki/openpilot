# PFEIFER - GAC
# Acknowledgements:
# HKG dashboard display pulled from sunnypilot. https://github.com/sunnyhaibin/sunnypilot
# GM dashboard display pulled from OPGM. https://github.com/opgm/openpilot

from openpilot.common.params import Params, put_nonblocking
from enum import IntEnum
from openpilot.selfdrive.controls.gap_adjust_button import gap_adjust_button, GapButtonState
from openpilot.selfdrive.controls.lfa_button import lfa_button, LFAButtonState

params = Params()

class GapAdjustState(IntEnum):
  AGGRESSIVE = 0
  STANDARD = 1
  RELAXED = 2
  SNOW = 3

class GapAdjust:
  state: GapAdjustState = GapAdjustState.STANDARD
  gap_button_transition_id = 0
  lfa_button_transition_id = 0
  disable_default_update = False

  def __init__(self) -> None:
    self.load_state()

  def load_state(self) -> None:
    try:
      self.state = GapAdjustState(int(params.get('LongitudinalPersonality')))
    except:
      self.state = GapAdjustState.STANDARD

  def write_state(self) -> None:
    put_nonblocking('LongitudinalPersonality', str(int(self.state)))

  def update(self, load_state=True, write_state=True, load_button_state=True) -> None:
    if self.disable_default_update:
      return

    if load_button_state:
      gap_adjust_button.load_state()
      lfa_button.load_state()

    gap_transition_id = gap_adjust_button.simple_transition_id

    if self.gap_button_transition_id != gap_transition_id:
      self.gap_button_transition_id = gap_transition_id
      if gap_adjust_button.simple_state == GapButtonState.DOUBLE_PRESS:
        if load_state:
            self.load_state()

        # Decrement state value and ensure it's non-negative
        new_state_value = int(self.state) - 1
        if new_state_value < 0:
            new_state_value = 3  # Assuming 4 states (0, 1, 2, 3)

        self.state = GapAdjustState(new_state_value)

        if write_state:
            self.write_state()


    lfa_transition_id = lfa_button.simple_transition_id

    if self.lfa_button_transition_id != lfa_transition_id:
      self.lfa_button_transition_id = lfa_transition_id
      if lfa_button.simple_state == LFAButtonState.DOUBLE_PRESS:
        if load_state:
            self.load_state()

        # Increment state value and wrap around using modulo
        self.state = GapAdjustState((int(self.state) + 1) % 4)

        if write_state:
            self.write_state()


  def transition_car_state(self, load_button_state=True):
    transition_id = gap_adjust_button.simple_transition_id;

    if self.gap_button_transition_id != transition_id:
      self.gap_button_transition_id = transition_id
      if gap_adjust_button.simple_state == GapButtonState.SINGLE_PRESS:
        return True

    return False


  def update_from_car_state(self, state: GapAdjustState, write_state=True) -> None:
    self.disable_default_update = True

    old_state = self.state
    self.state = GapAdjustState(state % 4)

    if write_state and old_state != self.state:
      self.write_state()

gap_adjust = GapAdjust()
