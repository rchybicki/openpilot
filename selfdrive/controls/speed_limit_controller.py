import numpy as np
from cereal import log
from openpilot.common.params import Params
from openpilot.common.conversions import Conversions as CV
from openpilot.selfdrive.controls.gap_adjust_button import gap_adjust_button, GapButtonState
import json
import math


mem_params = Params("/dev/shm/params")
params = Params()

# Lookup table for speed limit percent offset depending on speed, RCH Custom
                        # km/h  14     15    41    42     59    60   61     99   100
_LIMIT_PERC_OFFSET_BP =      [ 4.15,  4.16, 11.3, 11.4, 16.4, 16.6, 16.7, 27.5, 27.7 ] 
_LIMIT_PERC_OFFSET_V_GAP4 =  [ 0,     0,      0,   0,     0,    0,    0,    0,    0 ]
_LIMIT_PERC_OFFSET_V_GAP3 =  [ 0,     1.94, 1.38, 1.94, 1.94, 1.94, 3.33, 3.33, 4.16 ]
_LIMIT_PERC_OFFSET_V_GAP2 =  [ 0,     1.94, 1.38, 2.77, 2.77, 2.77, 4.16, 4.16, 5.55 ]
_LIMIT_PERC_OFFSET_V_GAP1 =  [ 0,     3.33, 4.16, 5.55, 5.55, 5.55, 5.55, 5.55, 8.33 ]
                # km/h  5     
                # 3   0.83
                # 5   1.38
                # 7   1.94
                # 10  2.77
                # 12  3.33
                # 15  4.16 
                # 17  4.72
                # 20  5.55
                # 25  5.55
                # 30  8.33

class SpeedLimitController:
  nav_speed_limit: float = 0 # m/s
  map_speed_limit: float = 0 # m/s
  map_next_speed_limit: float = 0 # m/s
  map_next_speed_limit_distance: float = 0 # m
  car_speed_limit: float = 0 # m/s
  _offset: float = 0 # m/s
  nav_enabled: bool = False
  car_enabled: bool = False
  speed_enabled: bool = False
  last_transition_id: int = 0
  last_speed_limit: float = 0
  switched_to_next_limit: bool = False
  current_max_velocity_update_count: int = 0
  vEgo: float = 0

  def __init__(self) -> None:
    self.load_persistent_enabled()
    self.write_nav_state()
    self.write_map_state()
    self.write_car_state()
    self.write_offset_state()

  def update_current_max_velocity(self, personality, vEgo: float, load_state: bool = True, write_state: bool = True) -> None:
    self.vEgo = vEgo
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
        if self._offset == 0 and self.speed_limit > 0:
          self._offset = vEgo - (self.speed_limit + self.offset(personality))
        else:
          self._offset = 0
        if write_state:
          self.write_offset_state()

  @property
  def speed_limit(self) -> float:
    limit: float = 0

    if self.map_enabled and self.map_speed_limit != 0:
      limit = self.map_speed_limit
      if self.last_speed_limit != limit:
        self.switched_to_next_limit = False
        self._offset = 0
        self.write_offset_state()
      if self.map_next_speed_limit != 0:
        next_speed_limit_switch_distance = abs(self.map_next_speed_limit - self.vEgo) * self.vEgo \
                  * (0.7 if self.map_next_speed_limit < self.vEgo else 1.5)
        if self.map_next_speed_limit_distance <= next_speed_limit_switch_distance or self.switched_to_next_limit:
          limit = self.map_next_speed_limit
          self.switched_to_next_limit = True

      self.last_speed_limit = self.map_speed_limit

    if self.nav_enabled and self.nav_speed_limit != 0 and limit == 0:
      limit = self.nav_speed_limit
        
    if self.car_enabled and self.car_speed_limit != 0 and limit == 0:
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
    return self._offset * CV.MS_TO_MPH

  @property
  def offset_kph(self) -> float:
    return self._offset * CV.MS_TO_KPH

  def offset(self, personality):
      if personality==log.LongitudinalPersonality.relaxed:
        return np.interp(self.speed_limit, _LIMIT_PERC_OFFSET_BP, _LIMIT_PERC_OFFSET_V_GAP3) + self._offset
      elif personality==log.LongitudinalPersonality.standard:
        return np.interp(self.speed_limit, _LIMIT_PERC_OFFSET_BP, _LIMIT_PERC_OFFSET_V_GAP2) + self._offset
      elif personality==log.LongitudinalPersonality.aggressive:
        return np.interp(self.speed_limit, _LIMIT_PERC_OFFSET_BP, _LIMIT_PERC_OFFSET_V_GAP1) + self._offset
      else: #snow
        return np.interp(self.speed_limit, _LIMIT_PERC_OFFSET_BP, _LIMIT_PERC_OFFSET_V_GAP4) + self._offset

  def write_nav_state(self):
    mem_params.put("NavSpeedLimit", json.dumps(self.nav_speed_limit))
    mem_params.put_bool("NavSpeedLimitControl", self.nav_enabled)

  def write_map_state(self):
    mem_params.put("MapSpeedLimit", json.dumps(self.map_speed_limit))
    mem_params.put("MapSpeedLimitNext", json.dumps(self.map_next_speed_limit))
    mem_params.put("MapSpeedLimitNextDistance", json.dumps(self.map_next_speed_limit_distance))
    mem_params.put_bool("MapSpeedLimitControl", self.map_enabled)

  def write_car_state(self):
    mem_params.put("CarSpeedLimit", json.dumps(self.car_speed_limit))
    mem_params.put_bool("CarSpeedLimitControl", self.car_enabled)

  def write_offset_state(self):
    mem_params.put("SpeedLimitOffset", json.dumps(self._offset))

  def load_state(self, load_persistent_enabled=False):
    self.nav_enabled = mem_params.get("NavSpeedLimitControl")
    self.car_enabled = mem_params.get("CarSpeedLimitControl")
    self.map_enabled = mem_params.get("MapSpeedLimitControl")
    self._offset = json.loads(mem_params.get("SpeedLimitOffset"))
    self.nav_speed_limit = json.loads(mem_params.get("NavSpeedLimit"))
    self.map_speed_limit = json.loads(mem_params.get("MapSpeedLimit"))
    self.map_next_speed_limit = json.loads(mem_params.get("MapSpeedLimitNext"))
    self.map_next_speed_limit_distance = json.loads(mem_params.get("MapSpeedLimitNextDistance"))
  
    self.car_speed_limit = json.loads(mem_params.get("CarSpeedLimit"))

    if load_persistent_enabled:
      self.load_persistent_enabled()

  def load_persistent_enabled(self):
    self.nav_enabled = params.get_bool("NavSpeedLimitControl")
    self.car_enabled = params.get_bool("CarSpeedLimitControl")
    self.map_enabled = params.get_bool("MapSpeedLimitControl")
    mem_params.put_bool("NavSpeedLimitControl", self.nav_enabled)
    mem_params.put_bool("MapSpeedLimitControl", self.map_enabled)
    mem_params.put_bool("CarSpeedLimitControl", self.car_enabled)


  def write_persistent_enabled(self):
    params.put_bool("NavSpeedLimitControl", self.nav_enabled)
    params.put_bool("CarSpeedLimitControl", self.car_enabled)
    params.put_bool("MapSpeedLimitControl", self.map_enabled)


slc = SpeedLimitController()
