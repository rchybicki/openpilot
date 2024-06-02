# PFEIFER - BM

from cereal import car
from openpilot.common.params import Params
from time import time
from typing import List, Dict, Any
import json
import capnp
ButtonType = car.CarState.ButtonEvent.Type

mem_params = Params("/dev/shm/params")

LONG_PRESS_LENGTH = 0.4 #s
PRESS_INTERVAL = 0.2 #s
BUTTON_COUNT = 20 # More than in capnp schema for sake of future additions

class ButtonManager:
  states: List[Dict[str, Any]] = []

  def __init__(self) -> None:
    self.states = [self.new_state() for _ in range(0, BUTTON_COUNT)]
    self.write_state()

  def new_state(self) -> Dict[str, Any]:
    return {
      "presses": 0,
      "long_press": False,
      "pressed": False,
      "last_transition": time(),
      "ready_for_read": False,
      "reset": True
    }

  def write_state(self) -> None:
    mem_params.put("ButtonManagerStates", json.dumps(self.states))

  def load_state(self) -> None:
    try:
      self.states = json.loads(mem_params.get("ButtonManagerStates"))
    except:
      self.states = [self.new_state() for _ in range(0, BUTTON_COUNT)]
      self.write_state()

  def unknown_update(self) -> None:
    for button in range(0, BUTTON_COUNT):
      if not self.states[button]["pressed"]:
        if time() - self.states[button]["last_transition"] > PRESS_INTERVAL:
          self.states[button]["ready_for_read"] = True
          self.states[button]["reset"] = True

  def button_update(self, button: int, pressed: bool) -> None:
    if pressed:
      if self.states[button]["reset"]:
        self.states[button]["reset"] = False
        self.states[button]["presses"] = 0

      if not self.states[button]["pressed"]:
        self.states[button]["presses"] += 1
        self.states[button]["pressed"] = True
        self.states[button]["last_transition"] = time()
      elif time() - self.states[button]["last_transition"] > LONG_PRESS_LENGTH and self.states[button]["presses"] == 1:
        self.states[button]["ready_for_read"] = True
        self.states[button]["long_press"] = True
        self.states[button]["last_transition"] = time()
    elif self.states[button]["pressed"] and not self.states[button]["long_press"]:
      self.states[button]["pressed"] = False
      self.states[button]["last_transition"] = time()

  def read_presses(self, button: int, count: int) -> bool:
    if self.states[button]["ready_for_read"]:
      presses = self.states[button]["presses"]
      if presses == count:
        self.states[button]["ready_for_read"] = False
        self.states[button]["long_press"] = False
        self.states[button]["presses"] = 0
        self.write_state()
        return True
    return False

  def read_long_press(self, button: int) -> bool:
    if self.states[button]["ready_for_read"]:
      if self.states[button]["long_press"]:
        self.states[button]["ready_for_read"] = False
        self.states[button]["long_press"] = False
        self.states[button]["presses"] = 0
        self.write_state()
        return True
    return False

  def update(self, cur_btn: int, prev_btn: int, buttons_dict: Dict[int, capnp.lib.capnp._EnumModule], unpressed_btn: int = 0):
    self.load_state()

    if cur_btn not in buttons_dict and prev_btn not in buttons_dict:
      self.unknown_update()
    else:
      if cur_btn in buttons_dict:
        self.button_update(buttons_dict[cur_btn], cur_btn != unpressed_btn)
      elif prev_btn in buttons_dict:
        self.button_update(buttons_dict[prev_btn], False)

    self.write_state()

bm = ButtonManager()
