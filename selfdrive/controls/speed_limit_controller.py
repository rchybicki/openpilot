import numpy as np
from cereal import log
from openpilot.common.params import Params
from openpilot.common.conversions import Conversions as CV
from openpilot.selfdrive.controls.gap_adjust_button import gap_adjust_button, GapButtonState
from openpilot.selfdrive.controls.lfa_button import lfa_button, LFAButtonState
from openpilot.selfdrive.mapd.lib.geo import DIRECTION
import json
import os
import math


mem_params = Params("/dev/shm/params")
params = Params()



OVERRIDES_PATH = '/data/media/0/overrides'

# Lookup table for speed limit percent offset depending on speed, RCH Custom
_LIMIT_PERC_OFFSET_BP     =  [ 14.9, 15,   41.9, 42.,  59.9, 60.,  60.1, 99.9, 100. ]
_LIMIT_PERC_OFFSET_V_GAP4 =  [ 0,     0,    0,    0,    0,    0,    0,    0,     0  ]
_LIMIT_PERC_OFFSET_V_GAP3 =  [ 0,     5.,   5.,   5.,   5.,   5.,  10.,  10.,   10. ]
_LIMIT_PERC_OFFSET_V_GAP2 =  [ 0,     5.,  10.,  10.,  10.,  10.,  15.,  15.,   20. ]
_LIMIT_PERC_OFFSET_V_GAP1 =  [ 0,    10.,  15.,  20.,  20.,  20.,  25.,  25.,   30. ]


class SpeedLimitController:
  nav_speed_limit: float = 0 # m/s
  map_speed_limit: float = 0 # m/s
  map_speed_limit_with_upcoming: float = 0 # m/s
  map_next_speed_limit: float = 0 # m/s
  map_next_speed_limit_distance: float = 0 # m
  map_way_id: int =  0
  last_way_id: int = 0
  map_distance_to_end_of_current_way: int = 0
  last_map_distance_to_end_of_current_way: int = 0
  map_distance_to_end_of_next_way: int = 0
  last_map_distance_to_end_of_next_way: int = 0
  map_way_direction = None
  last_way_direction = None
  map_next_way_id: int = 0
  map_next_way_direction = None
  car_speed_limit: float = 0 # m/s
  _offset: float = 0 # m/s
  nav_enabled: bool = False
  car_enabled: bool = False
  speed_enabled: bool = False
  gap_last_transition_id: int = 0
  lfa_last_transition_id: int = 0
  last_speed_limit: float = 0
  switched_to_next_limit: bool = False
  current_max_velocity_update_count: int = 0
  vEgo: float = 0
  overrides = {}
  way_id_offset: int = 0

  def __init__(self) -> None:
    self.load_persistent_enabled()
    self.write_nav_state()
    self.write_map_state()
    self.write_car_state()
    self.write_offset_state()

  
  def read_overrides(self):
    if os.path.exists(OVERRIDES_PATH):
        with open(OVERRIDES_PATH, 'r') as file:
            try:
                overrides_raw = json.load(file)  # Load the JSON content
                self.overrides = {}
                for key, value in overrides_raw.items():
                    if isinstance(value, dict):
                        # Convert distance keys from strings to integers
                        self.overrides[key] = {int(dist): offset for dist, offset in value.items()}
                    else:
                        # Handle old format (single value) by initializing an empty dictionary
                        self.overrides[key] = {}
                print(f"SLC Read overrides {self.overrides}")
            except:
                print("SLC error occurred while reading overrides")


  def write_overrides(self):
    with open(OVERRIDES_PATH, 'w') as file:
        json.dump(self.overrides, file) 


  def get_override(self, current_way_id, current_way_direction, current_distance):
    key = str(current_way_id) + str(current_way_direction)

    if current_way_id != 0 and current_way_direction is not None and key in self.overrides:
        distance_overrides = self.overrides[key]
        # Find the closest distance that is equal or larger than current_distance
        closest_distance = min((d for d in distance_overrides.keys() if d >= current_distance), default=None)
        if closest_distance is not None:
            way_id_offset = distance_overrides[closest_distance]
            print(f"SLC way read way offset to {way_id_offset} for direction {current_way_direction} and current_distance {current_distance}")
            return distance_overrides[closest_distance]
        
    return None


  def set_override(self, way_id, direction, distance, vego, override_value):
    key = str(way_id) + str(direction)
    # Adjust distance for 0.5 seconds back and round to nearest multiple of 5
    adjusted_distance = round((distance + vego / 2) / 5) * 5

    # Initialize the overrides map for the key if it doesn't exist
    if key not in self.overrides:
        self.overrides[key] = {}

    # Remove keys within 1 second forward and backward
    distances_to_remove = [d for d in self.overrides[key] if adjusted_distance - vego <= d <= adjusted_distance + vego]
    for d in distances_to_remove:
        del self.overrides[key][d]

    # Check if the previous (larger distance) entry has the same override, if so, don't add
    larger_distances = [d for d in self.overrides[key] if d > adjusted_distance]
    if larger_distances and self.overrides[key][min(larger_distances)] == override_value:
        return

    # Check and remove the next (smaller distance) entry if it has the same override
    smaller_distances = [d for d in self.overrides[key] if d < adjusted_distance]
    if smaller_distances:
        next_smaller_distance = max(smaller_distances)
        if self.overrides[key][next_smaller_distance] == override_value:
            del self.overrides[key][next_smaller_distance]

    # Finally, set the override
    self.overrides[key][adjusted_distance] = override_value
    self.write_overrides()
    print(f"SLC set override for {key} at adjusted distance {adjusted_distance} to {override_value}")

  def clear_nearby_overrides(self, way_id, direction, current_distance, vEgo):
    key = str(way_id) + str(direction)
    three_seconds_distance = 3 * vEgo # 3 seconds worth of distance at the current velocity
    # Check if there are any overrides for the current way_id and direction
    if key in self.overrides:
        distances_to_clear = [d for d in self.overrides[key]
                              if current_distance - three_seconds_distance <= d <= current_distance + three_seconds_distance]
        for d in distances_to_clear:
            del self.overrides[key][d]
    print(f"SLC cleared overrides in range for way id {way_id}, direction {direction}, around distance {current_distance}")

  def update_load_state(self, vEgo): 
    self.vEgo = vEgo
    self.current_max_velocity_update_count += 1
    self.current_max_velocity_update_count = self.current_max_velocity_update_count % 100

    self.load_state()

    if self.current_max_velocity_update_count == 0:
      self.load_persistent_enabled()

  def update_button_presses(self, current_way_id, current_way_direction, current_distance_to_end_of_way, vEgo):
    offset_tick = 1.38 # 5 km/h

    gap_adjust_button.load_state()
    if self.gap_last_transition_id != gap_adjust_button.simple_transition_id:
      self.gap_last_transition_id = gap_adjust_button.simple_transition_id
      if gap_adjust_button.simple_state == GapButtonState.SINGLE_PRESS and self.speed_limit > 0 and current_way_id != 0 \
              and current_distance_to_end_of_way != 0 and current_way_direction is not None:
          self.way_id_offset += offset_tick
          self.set_override(current_way_id, current_way_direction, current_distance_to_end_of_way, vEgo, self.way_id_offset)
          self.write_offset_state()
          print(f"SLC increasing override to {self.way_id_offset}, saving overrides")

    lfa_button.load_state()
    if self.lfa_last_transition_id != lfa_button.simple_transition_id:
      self.lfa_last_transition_id = lfa_button.simple_transition_id
      if lfa_button.simple_state == LFAButtonState.SINGLE_PRESS and self.speed_limit > 0 and current_way_id != 0 \
              and current_distance_to_end_of_way != 0 and current_way_direction is not None:
        self.way_id_offset -= offset_tick
        self.set_override(current_way_id, current_way_direction, current_distance_to_end_of_way, vEgo, self.way_id_offset)
        self.write_offset_state()
      elif lfa_button.simple_state == LFAButtonState.LONG_PRESS and self.speed_limit > 0 and current_way_id != 0 \
              and current_distance_to_end_of_way != 0 and current_way_direction is not None:
        self.clear_nearby_overrides(current_way_id, current_way_direction, current_distance_to_end_of_way, vEgo)
        self.way_id_offset = 0
        self.write_offset_state()
        
    

  def update_current_max_velocity(self, personality, vEgo: float) -> None:
    self.update_load_state(vEgo)
    
    self.map_speed_limit_with_upcoming = self.map_speed_limit

    if self.last_speed_limit != self.map_speed_limit:
      self.switched_to_next_limit = False
      self.way_id_offset = 0
      self.write_offset_state()
      self.last_speed_limit = self.map_speed_limit

    current_way_id = self.map_way_id
    current_way_direction = self.map_way_direction
    current_distance_to_end_of_way = self.map_distance_to_end_of_current_way

    if self.map_next_speed_limit != 0:
      next_way_id_offset = self.get_override(self.map_next_way_id, self.map_next_way_direction, self.map_distance_to_end_of_next_way - self.map_next_speed_limit_distance) 
      next_way_id_offset = next_way_id_offset if next_way_id_offset is not None else 0
      
      next_speed = self.map_next_speed_limit + next_way_id_offset
      next_speed_limit_switch_distance = abs(next_speed - self.vEgo) * self.vEgo \
                * (0.7 if next_speed < self.vEgo else 1.5)
      if self.map_next_speed_limit_distance <= next_speed_limit_switch_distance or self.switched_to_next_limit:
        self.map_speed_limit_with_upcoming = self.map_next_speed_limit
        current_way_id = self.map_next_way_id
        current_way_direction = self.map_next_way_direction
        current_distance_to_end_of_way = self.map_distance_to_end_of_next_way
        self.switched_to_next_limit = True

    if self.last_map_distance_to_end_of_current_way != current_distance_to_end_of_way:
      new_offset = self.get_override(current_way_id, current_way_direction, current_distance_to_end_of_way)
      if new_offset is not None:
        self.way_id_offset = new_offset
        self.write_offset_state()
      self.last_map_distance_to_end_of_current_way = current_distance_to_end_of_way

    if self.last_way_id != current_way_id:
      new_offset = self.get_override(current_way_id, current_way_direction, current_distance_to_end_of_way)
      if new_offset is not None:
        self.way_id_offset = new_offset
        self.write_offset_state()
      self.last_way_id = current_way_id
        

    if self.last_way_direction != current_way_direction:
      new_offset =  self.get_override(current_way_id, current_way_direction, current_distance_to_end_of_way) 
      if new_offset is not None:
        self.way_id_offset = new_offset
        self.write_offset_state()
      self.last_way_direction = current_way_direction

    self.update_button_presses(current_way_id, current_way_direction, current_distance_to_end_of_way, vEgo)


  @property
  def speed_limit(self) -> float:
    limit: float = 0

    if self.map_enabled and self.map_speed_limit_with_upcoming != 0:
      limit = self.map_speed_limit_with_upcoming

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

  def offset(self, personality):
    personality_gaps = {
        log.LongitudinalPersonality.relaxed: _LIMIT_PERC_OFFSET_V_GAP3,
        log.LongitudinalPersonality.standard: _LIMIT_PERC_OFFSET_V_GAP2,
        log.LongitudinalPersonality.aggressive: _LIMIT_PERC_OFFSET_V_GAP1,
        #snow
        3: _LIMIT_PERC_OFFSET_V_GAP4 
    }

    gap_values = personality_gaps.get(personality)

    return np.interp(self.speed_limit_kph, _LIMIT_PERC_OFFSET_BP, gap_values) * CV.KPH_TO_MS + self.way_id_offset


  def write_nav_state(self):
    mem_params.put("NavSpeedLimit", json.dumps(self.nav_speed_limit))
    mem_params.put_bool("NavSpeedLimitControl", self.nav_enabled)

  def write_map_state(self):
    mem_params.put("MapSpeedLimit", json.dumps(self.map_speed_limit))
    mem_params.put("MapSpeedLimitWithUpcoming", json.dumps(self.map_speed_limit_with_upcoming))
    mem_params.put("MapSpeedLimitNext", json.dumps(self.map_next_speed_limit))
    mem_params.put("MapSpeedLimitNextDistance", json.dumps(self.map_next_speed_limit_distance))
    mem_params.put("MapWayId", json.dumps(self.map_way_id))
    mem_params.put("DistanceToEndOfCurrentWay", json.dumps(self.map_distance_to_end_of_current_way))
    mem_params.put("MapNextWayId", json.dumps(self.map_next_way_id))
    mem_params.put("DistanceToEndOfNextWay", json.dumps(self.map_distance_to_end_of_next_way))
    mem_params.put("MapWayDirection", json.dumps(self.map_way_direction))
    mem_params.put("MapNextWayDirection", json.dumps(self.map_next_way_direction))
    mem_params.put_bool("MapSpeedLimitControl", self.map_enabled)

  def write_car_state(self):
    mem_params.put("CarSpeedLimit", json.dumps(self.car_speed_limit))
    mem_params.put_bool("CarSpeedLimitControl", self.car_enabled)

  def write_offset_state(self):
    mem_params.put("MapWayIdOffset", json.dumps(self.way_id_offset))

  def load_state(self, load_persistent_enabled=False):
    self.nav_enabled = mem_params.get("NavSpeedLimitControl")
    self.car_enabled = mem_params.get("CarSpeedLimitControl")
    self.map_enabled = mem_params.get("MapSpeedLimitControl")
    self.way_id_offset = json.loads(mem_params.get("MapWayIdOffset"))
    self.nav_speed_limit = json.loads(mem_params.get("NavSpeedLimit"))
    self.map_speed_limit = json.loads(mem_params.get("MapSpeedLimit"))
    self.map_speed_limit_with_upcoming = json.loads(mem_params.get("MapSpeedLimitWithUpcoming"))
    self.map_next_speed_limit = json.loads(mem_params.get("MapSpeedLimitNext"))
    self.map_next_speed_limit_distance = json.loads(mem_params.get("MapSpeedLimitNextDistance"))
    self.map_way_id = json.loads(mem_params.get("MapWayId"))
    self.map_distance_to_end_of_current_way = json.loads(mem_params.get("DistanceToEndOfCurrentWay"))
    self.map_way_direction = json.loads(mem_params.get("MapWayDirection"))
    self.map_next_way_id = json.loads(mem_params.get("MapNextWayId"))
    self.map_distance_to_end_of_next_way = json.loads(mem_params.get("DistanceToEndOfNextWay"))
    self.map_next_way_direction = json.loads(mem_params.get("MapNextWayDirection"))
  
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
