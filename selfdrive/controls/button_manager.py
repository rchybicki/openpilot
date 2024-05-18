# PFEIFER - BM

from openpilot.common.params import Params
from enum import IntEnum
from time import time
import json

mem_params = Params("/dev/shm/params")

LONG_PRESS_LENGTH = 0.4 #s
PRESS_INTERVAL = 0.2 #s
DEBOUNCE_LENGTH = 0.04 #s

class ButtonState(IntEnum):
  VOID = 0
  WAITING_SINGLE_RELEASE = 1
  LONG_PRESS_WAITING_RELEASE = 2
  LONG_PRESS = 3
  SINGLE_PRESS_WAITING_DOUBLE = 4
  SINGLE_PRESS = 5
  DOUBLE_PRESS_WAITING_RELEASE = 6
  DOUBLE_PRESS_WAITING_TRIPLE = 7
  DOUBLE_PRESS = 8
  TRIPLE_PRESS_WAITING_RELEASE = 9
  TRIPLE_PRESS = 10

simple_states = [ButtonState.VOID, ButtonState.LONG_PRESS, ButtonState.SINGLE_PRESS, ButtonState.DOUBLE_PRESS, ButtonState.TRIPLE_PRESS]

class Button:
  name: str
  state: ButtonState = ButtonState.VOID
  transition_id: int = 0
  last_transition_time: float = time()

  def __init__(self, name: str) -> None:
    self.name = name

    self.write_state()

    self.state_transition = {
      ButtonState.VOID: self.update_void_state,
      ButtonState.WAITING_SINGLE_RELEASE: self.update_waiting_single_release_state,
      ButtonState.LONG_PRESS_WAITING_RELEASE: self.update_long_press_waiting_release_state,
      ButtonState.LONG_PRESS: self.update_long_press_state,
      ButtonState.SINGLE_PRESS_WAITING_DOUBLE: self.update_single_press_waiting_double_state,
      ButtonState.SINGLE_PRESS: self.update_single_press_state,
      ButtonState.DOUBLE_PRESS_WAITING_RELEASE: self.update_double_press_waiting_release_state,
      ButtonState.DOUBLE_PRESS_WAITING_TRIPLE: self.update_double_press_waiting_triple_state,
      ButtonState.DOUBLE_PRESS: self.update_double_press_state,
      ButtonState.TRIPLE_PRESS_WAITING_RELEASE: self.update_triple_press_waiting_release_state,
      ButtonState.TRIPLE_PRESS: self.update_triple_press_state,
    }

  def update_transition_time(self) -> None:
    self.last_transition_time = time()

  def press_interval_elapsed(self) -> bool:
    return (time() - self.last_transition_time) > PRESS_INTERVAL

  def long_press_length_elapsed(self) -> bool:
    return (time() - self.last_transition_time) > LONG_PRESS_LENGTH

  def debounce_length_elapsed(self) -> bool:
    return (time() - self.last_transition_time) > DEBOUNCE_LENGTH


  @property
  def simple_state(self) -> ButtonState:
    """
    For most transitional states this returns void. For transitional states that
    only have a single possible final state this returns the final state.
    Possible output states are the following: VOID, LONG_PRESS, SINGLE_PRESS,
    DOUBLE_PRESS, TRIPLE_PRESS
    """
    if self.state in simple_states:
      return self.state

    if self.state == ButtonState.TRIPLE_PRESS_WAITING_RELEASE:
      return ButtonState.TRIPLE_PRESS

    if self.state == ButtonState.LONG_PRESS_WAITING_RELEASE:
      return ButtonState.LONG_PRESS

    return ButtonState.VOID

  @property
  def simple_transition_id(self) -> int:
    """
    If a state only has a single possible final state this returns the next
    transition id, otherwise it responds with the current transition id. Useful
    For determining changes in simple state.
    """
    if self.state == ButtonState.TRIPLE_PRESS_WAITING_RELEASE:
      return self.transition_id + 1

    if self.state == ButtonState.LONG_PRESS_WAITING_RELEASE:
      return self.transition_id + 1

    return self.transition_id

  @property
  def dict(self):
    return {
      "state": int(self.state),
      "transition_id": self.transition_id,
      "last_transition_time": self.last_transition_time
    }

  def write_state(self) -> None:
    mem_params.put(self.name, json.dumps(self.dict))

  def load_state(self) -> None:
    try:
      state_dict = json.loads(mem_params.get(self.name))
      self.state = ButtonState(state_dict["state"])
      self.transition_id = state_dict["transition_id"]
      self.last_transition_time = state_dict["last_transition_time"]
    except:
      self.state = ButtonState.VOID
      self.transition_id = 0
      self.last_transition_time = time()

  def update(self, button_pressed: bool, load_state = True, write_state = True) -> ButtonState:
    if load_state:
      self.load_state()

    if not self.debounce_length_elapsed():
      return self.state

    state = self.state_transition[self.state](button_pressed)

    if self.state != state:
      self.state = state
      self.transition_id += 1
      self.update_transition_time()
      if write_state:
        self.write_state()

    return self.state

  # ---------------- STATE TRANSITIONS ----------------------------
  def update_void_state(self, button_pressed: bool) -> ButtonState:
    if button_pressed:
      return ButtonState.WAITING_SINGLE_RELEASE

    return self.state

  def update_waiting_single_release_state(self, button_pressed: bool) -> ButtonState:
    if button_pressed and self.long_press_length_elapsed():
      return ButtonState.LONG_PRESS_WAITING_RELEASE

    if not button_pressed:
      return ButtonState.SINGLE_PRESS_WAITING_DOUBLE

    return self.state

  def update_long_press_waiting_release_state(self, button_pressed: bool) -> ButtonState:
    if not button_pressed:
      return ButtonState.LONG_PRESS

    return self.state

  def update_long_press_state(self, button_pressed: bool) -> ButtonState:
    if button_pressed:
      return ButtonState.WAITING_SINGLE_RELEASE

    return self.state

  def update_single_press_waiting_double_state(self, button_pressed: bool) -> ButtonState:
    if not button_pressed and self.press_interval_elapsed():
      return ButtonState.SINGLE_PRESS

    if button_pressed:
      return ButtonState.DOUBLE_PRESS_WAITING_RELEASE

    return self.state

  def update_single_press_state(self, button_pressed: bool) -> ButtonState:
    if button_pressed:
      return ButtonState.WAITING_SINGLE_RELEASE

    return self.state

  def update_double_press_waiting_release_state(self, button_pressed: bool) -> ButtonState:
      if not button_pressed:
        return ButtonState.DOUBLE_PRESS_WAITING_TRIPLE

      return self.state

  def update_double_press_waiting_triple_state(self, button_pressed: bool) -> ButtonState:
      if not button_pressed and self.press_interval_elapsed():
        return ButtonState.DOUBLE_PRESS
      if button_pressed:
        return ButtonState.TRIPLE_PRESS_WAITING_RELEASE

      return self.state

  def update_double_press_state(self, button_pressed: bool) -> ButtonState:
      if button_pressed:
        return ButtonState.WAITING_SINGLE_RELEASE

      return self.state

  def update_triple_press_waiting_release_state(self, button_pressed: bool) -> ButtonState:
    if not button_pressed:
        return ButtonState.TRIPLE_PRESS

    return self.state

  def update_triple_press_state(self, button_pressed: bool) -> ButtonState:
    if button_pressed:
      return ButtonState.WAITING_SINGLE_RELEASE

    return self.state
