# PFEIFER - BM

from cereal import car
from openpilot.common.params import Params
from time import time
from typing import Dict
import json
import capnp
ButtonType = car.CarState.ButtonEvent.Type

mem_params = Params("/dev/shm/params")

LONG_PRESS_LENGTH = 0.4 #s
PRESS_INTERVAL = 0.2 #s
BUTTON_COUNT = 20 # More than in capnp schema for sake of future additions

class ButtonState:
  last_update_time = time()
  last_transition_time = time()
  click_count = 0
  pressed = False
  long_press = False
  reset = True
  read = False
  manager = None

  def __init__(self, state_dict: dict = None, manager: ButtonManager = None) -> None:
    self.manager = manager
    if state_dict:
      self.last_update_time = state_dict["last_update_time"]
      self.last_transition_time = state_dict["last_transition_time"]
      self.click_count = state_dict["click_count"]
      self.pressed = state_dict["pressed"]
      self.long_press = state_dict["long_press"]
      self.reset = state_dict["reset"]
      self.read = state_dict["read"]
    else:
      self.last_update_time = time()
      self.last_transition_time = time()
      self.click_count = 0
      self.pressed = False
      self.long_press = False
      self.reset = True
      self.read = False

  def update(self, button_pressed: bool) -> None:
    self.last_update_time = time()
    if button_pressed:
      if not self.pressed:
        if self.reset:
          self.click_count = 0
          self.reset = False
          self.long_press = False
          self.read = False
        self.click_count += 1
        self.transition(button_pressed)
    else:
      if self.pressed:
        self.transition(button_pressed)
      elif self.press_interval_elapsed():
        self.reset = True

  def update_no_btn(self) -> None:
    if self.pressed:
      if self.click_count == 1 and self.long_press_length_elapsed():
        self.long_press = True
    else:
      if self.press_interval_elapsed():
        self.reset = True

  def transition(self, button_pressed: bool):
    now = time()
    if self.long_press_length_elapsed() and self.click_count == 1:
      self.long_press = True
    else:
      self.long_press = False

    self.last_transition_time = time()
    self.pressed = button_pressed


  def press_interval_elapsed(self) -> bool:
    return (time() - self.last_update_time) > PRESS_INTERVAL

  def long_press_length_elapsed(self) -> bool:
    return (time() - self.last_update_time) > LONG_PRESS_LENGTH

  def read_presses(self, click_count: int, write_state = True) -> bool:
    if not self.read:
      self.read = True
      if write_state and self.manager is not None:
        self.manager.write_state()
      if self.click_count == click_count and not self.long_press:
        return True
    return False

  def read_long_press(self) -> bool:
    if not self.read:
      self.read = True
      if self.long_press:
        return True
    return False

  @property
  def dict(self):
    return {
      "last_update_time": self.last_update_time,
      "last_transition_time": self.last_transition_time,
      "click_count": self.click_count,
      "pressed": self.pressed,
      "long_press": self.long_press,
      "reset": self.reset,
      "read": self.read
    }


class ButtonManager:
  states: dict[int, ButtonState] = {}

  def __init__(self, name: str) -> None:
    for button in range(0, BUTTON_COUNT):
      self.states[button] = ButtonState(manager=self)

    self.write_state()

  @property
  def dict(self):
    return {button: self.states[button].dict for button in range(0, BUTTON_COUNT)}

  def write_state(self) -> None:
    mem_params.put("ButtonManagerStates", json.dumps(self.dict))

  def load_state(self) -> None:
    try:
      state_dict = json.loads(mem_params.get("ButtonManagerStates"))
      for button in range(0, BUTTON_COUNT):
        self.states[button] = ButtonState(state_dict[button], self)

    except:
      for button in range(0, BUTTON_COUNT):
        self.states[button] = ButtonState(manager=self)

  def update(self, cur_btn: int, prev_btn: int, buttons_dict: Dict[int, capnp.lib.capnp._EnumModule], unpressed_btn: int = 0, load_state = True, write_state = True):
    if load_state:
      self.load_state()

    if cur_btn not in buttons_dict and prev_btn not in buttons_dict:
      for button in range(0, BUTTON_COUNT):
        self.states[button].update_no_btn()
    else:
      if cur_btn in buttons_dict:
        self.states[buttons_dict[cur_btn]].update(cur_btn != unpressed_btn)
      elif prev_btn in buttons_dict:
        self.states[buttons_dict[prev_btn]].update(False)

    if write_state:
      self.write_state()

  def get_state(self, button: int, load_state = True) -> ButtonState:
    if load_state:
      self.load_state()
    return self.states[button]

bm = ButtonManager("ButtonManager")
