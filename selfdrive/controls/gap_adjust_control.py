# PFEIFER - GAC

from common.params import Params, put_nonblocking
from enum import IntEnum
from selfdrive.controls.gap_adjust_button import gap_adjust_button, GapButtonState

params = Params()

class GapAdjustState(IntEnum):
  AGGRESSIVE = 0
  STANDARD = 1
  RELAXED = 2

class GapAdjust:
  state: GapAdjustState = GapAdjustState.STANDARD
  button_transition_id = 0

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
    if load_button_state:
      gap_adjust_button.load_state()

    transition_id = gap_adjust_button.simple_transition_id;

    if self.button_transition_id != transition_id:
      self.button_transition_id = transition_id
      if gap_adjust_button.simple_state == GapButtonState.SINGLE_PRESS:
        if load_state:
          self.load_state()

        self.state = GapAdjustState((int(self.state) + 1) % 3)

        if write_state:
          self.write_state()

gap_adjust = GapAdjust()
