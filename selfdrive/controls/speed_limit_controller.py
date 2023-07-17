from common.params import Params
from common.conversions import Conversions as CV
from selfdrive.controls.gap_adjust_button import gap_adjust_button, GapButtonState
import json

mem_params = Params("/dev/shm/params")
params = Params()

class SpeedLimitController:
  nav_speed_limit: float = 0 # m/s
  map_speed_limit: float = 0 # m/s
  car_speed_limit: float = 0 # m/s
  offset: float = 0 # m/s
  nav_enabled: bool = False
  car_enabled: bool = False
  speed_enabled: bool = False
  last_transition_id: int = 0
  current_max_velocity_update_count: int = 0

  def __init__(self) -> None:
    self.load_persistent_enabled()
    self.write_state()

  def update_current_max_velocity(self, vEgo: float, load_state: bool = True, write_state: bool = True) -> None:
    self.current_max_velocity_update_count += 1
    self.current_max_velocity_update_count = self.current_max_velocity_update_count % 100
    if load_state:
      self.load_state()
      if self.current_max_velocity_update_count == 0:
        self.load_persistent_enabled()

    gap_adjust_button.load_state()
    if self.last_transition_id != gap_adjust_button.simple_transition_id:
      self.last_transition_id = gap_adjust_button.simple_transition_id
      if gap_adjust_button.simple_state == GapButtonState.DOUBLE_PRESS:
        if self.offset == 0 and self.speed_limit > 0:
          self.offset = vEgo - self.speed_limit
        else:
          self.offset = 0

        if write_state:
          self.write_state()

  @property
  def speed_limit(self) -> float:
    limit: float = 0
    if self.nav_enabled and self.nav_speed_limit != 0:
      limit = self.nav_speed_limit
    elif self.map_enabled and self.map_speed_limit != 0:
      limit = self.map_speed_limit
    elif self.car_enabled and self.car_speed_limit != 0:
      limit = self.car_speed_limit

    return limit

  @property
  def speed_limit_mph(self) -> float:
    return self.speed_limit * CV.MS_TO_MPH

  @property
  def speed_limit_kph(self) -> float:
    return self.speed_limit * CV.MS_TO_KPH

  @property
  def offset_mph(self) -> float:
    return self.offset * CV.MS_TO_MPH

  @property
  def offset_kph(self) -> float:
    return self.offset * CV.MS_TO_KPH

  def write_state(self, write_persistent_enabled=False):
    data = json.dumps({
      "nav_speed_limit": self.nav_speed_limit,
      "map_speed_limit": self.map_speed_limit,
      "car_speed_limit": self.car_speed_limit,
      "offset": self.offset,
      "nav_enabled": self.nav_enabled,
      "car_enabled": self.car_enabled,
      "map_enabled": self.map_enabled,
    })
    mem_params.put("SpeedLimitControlData", data)
    if write_persistent_enabled:
      self.write_persistent_enabled()

  def load_state(self, load_persistent_enabled=False):
    data = json.loads(mem_params.get("SpeedLimitControlData"))
    self.nav_enabled = data["nav_enabled"]
    self.car_enabled = data["car_enabled"]
    self.map_enabled = data["map_enabled"]
    self.offset = data["offset"]
    self.nav_speed_limit = data["nav_speed_limit"]
    self.map_speed_limit = data["map_speed_limit"]
    self.car_speed_limit = data["car_speed_limit"]

    if load_persistent_enabled:
      self.load_persistent_enabled()

  def load_persistent_enabled(self):
    self.nav_enabled = params.get_bool("NavSpeedLimitControl")
    self.car_enabled = params.get_bool("CarSpeedLimitControl")
    self.map_enabled = params.get_bool("MapSpeedLimitControl")


  def write_persistent_enabled(self):
    params.put_bool("NavSpeedLimitControl", self.nav_enabled)
    params.put_bool("CarSpeedLimitControl", self.car_enabled)
    params.put_bool("MapSpeedLimitControl", self.map_enabled)


slc = SpeedLimitController()
