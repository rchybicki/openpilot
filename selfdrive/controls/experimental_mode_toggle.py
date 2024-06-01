# PFEIFER - EMT

from openpilot.common.params import Params
from openpilot.selfdrive.controls.button_manager import bm
from cereal import car
ButtonType = car.CarState.ButtonEvent.Type

params = Params()

class ExperimentalModeToggle:
  button_transition_id = 0
  state: bool

  def __init__(self) -> None:
    self.load_state()

  def load_state(self) -> None:
    try:
      self.state = params.get_bool("ExperimentalMode")
    except:
      self.state = False

  def write_state(self) -> None:
    params.put_bool_nonblocking('ExperimentalMode', self.state)

  def update(self, load_state=True, write_state=True, load_button_state=True) -> None:
    if bm.read_long_press(ButtonType.gapAdjustCruise):
      if load_state:
        self.load_state()

      self.state = not self.state

      if write_state:
        self.write_state()

emt = ExperimentalModeToggle()
