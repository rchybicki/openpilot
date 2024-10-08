# PFEIFER - SLC - Modified by FrogAi for FrogPilot
import json
import math
import numpy as np

from openpilot.common.conversions import Conversions as CV
from openpilot.common.params import Params
from cereal import log

from openpilot.selfdrive.frogpilot.frogpilot_variables import FrogPilotVariables

R = 6373000.0  # approximate radius of earth in meters
TO_RADIANS = math.pi / 180

# Lookup table for speed limit kph offset depending on speed, RCH Custom
_LIMIT_PERC_OFFSET_BP = [14.9, 15, 41.9, 42.0, 59.9, 60.0, 60.1, 99.9, 100.0]
_LIMIT_PERC_OFFSET_V_GAP4 = [0, 0, 0, 0, 0, 0, 0, 0, 0]
_LIMIT_PERC_OFFSET_V_GAP3 = [0, 5.0, 5.0, 5.0, 5.0, 5.0, 10.0, 10.0, 10.0]
_LIMIT_PERC_OFFSET_V_GAP2 = [0, 5.0, 10.0, 10.0, 10.0, 10.0, 15.0, 15.0, 20.0]
_LIMIT_PERC_OFFSET_V_GAP1 = [0, 10.0, 15.0, 20.0, 20.0, 20.0, 25.0, 25.0, 30.0]

# points should be in radians
# output is meters
def distance_to_point(ax, ay, bx, by):
  a = math.sin((bx - ax) / 2) * math.sin((bx - ax) / 2) + math.cos(ax) * math.cos(bx) * math.sin((by - ay) / 2) * math.sin((by - ay) / 2)
  c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
  return R * c  # in meters

class SpeedLimitController:
  def __init__(self):
    self.frogpilot_toggles = FrogPilotVariables.toggles
    FrogPilotVariables.update_frogpilot_params()

    self.params = Params()
    self.params_memory = Params("/dev/shm/params")

    self.car_speed_limit = 0  # m/s
    self.map_speed_limit = 0  # m/s
    self.max_speed_limit = 0  # m/s
    self.nav_speed_limit = 0  # m/s
    self.personality = log.LongitudinalPersonality.standard
    self.prv_speed_limit = self.params.get_float("PreviousSpeedLimit")

  def get_param_memory(self, key, is_json=False):
    param_value = self.params_memory.get(key)
    if param_value is None:
      return {} if is_json else 0.0
    return json.loads(param_value) if is_json else float(param_value)

  def update_previous_limit(self, speed_limit):
    if self.prv_speed_limit != speed_limit:
      self.params.put_float_nonblocking("PreviousSpeedLimit", speed_limit)
      self.prv_speed_limit = speed_limit

  def update(self, dashboardSpeedLimit, enabled, navigationSpeedLimit, v_cruise, v_ego, frogpilot_toggles, personality):
    self.personality = personality
    self.car_speed_limit = dashboardSpeedLimit
    self.write_map_state(v_ego)
    self.nav_speed_limit = navigationSpeedLimit

    self.max_speed_limit = v_cruise if enabled else 0

    self.frogpilot_toggles = frogpilot_toggles

  def calculate_change_distance(self, vEgo, vDesired):
    # Determine if we are accelerating or decelerating
    if vDesired > vEgo:
      a = 1  # Accelerating
    else:
      a = -1.25  # Decelerating

    u = vEgo  # Initial velocity in m/s
    v = vDesired  # Desired final velocity in m/s
    d = (v**2 - u**2) / (2 * a)
    return d

  def write_map_state(self, v_ego):
    self.map_speed_limit = self.get_param_memory("MapSpeedLimit")

    next_map_speed_limit = self.get_param_memory("NextMapSpeedLimit", is_json=True)
    next_map_speed_limit_value = next_map_speed_limit.get("speedlimit", 0)
    next_map_speed_limit_lat = next_map_speed_limit.get("latitude", 0)
    next_map_speed_limit_lon = next_map_speed_limit.get("longitude", 0)

    position = self.get_param_memory("LastGPSPosition", is_json=True)
    lat = position.get("latitude", 0)
    lon = position.get("longitude", 0)

    if next_map_speed_limit_value > 1:
      d = distance_to_point(lat * TO_RADIANS, lon * TO_RADIANS, next_map_speed_limit_lat * TO_RADIANS, next_map_speed_limit_lon * TO_RADIANS)
      change_distance = self.calculate_change_distance(v_ego, next_map_speed_limit_value)

      if d < change_distance:
        self.map_speed_limit = next_map_speed_limit_value

  @property
  def experimental_mode(self):
    return self.speed_limit == 0 and self.frogpilot_toggles.slc_fallback_experimental

  @property
  def desired_speed_limit(self):
    if self.speed_limit > 1:
      self.update_previous_limit(self.speed_limit)
      return self.speed_limit + self.offset
    return 0

  @property
  def offset(self):
    # personality_gaps = {
    #   log.LongitudinalPersonality.relaxed: _LIMIT_PERC_OFFSET_V_GAP2,
    #   log.LongitudinalPersonality.standard: _LIMIT_PERC_OFFSET_V_GAP2,
    #   log.LongitudinalPersonality.aggressive: _LIMIT_PERC_OFFSET_V_GAP1,
    #   # snow
    #   3: _LIMIT_PERC_OFFSET_V_GAP3,
    # }

    # gap_values = personality_gaps.get(self.personality)

    return float(np.interp(self.speed_limit * CV.MS_TO_KPH, _LIMIT_PERC_OFFSET_BP, _LIMIT_PERC_OFFSET_V_GAP2) * CV.KPH_TO_MS)

  @property
  def speed_limit(self):
    limits = [self.car_speed_limit, self.map_speed_limit, self.nav_speed_limit]
    filtered_limits = [float(limit) for limit in limits if limit > 1]

    if self.frogpilot_toggles.speed_limit_priority_highest and filtered_limits:
      return max(filtered_limits)
    if self.frogpilot_toggles.speed_limit_priority_lowest and filtered_limits:
      return min(filtered_limits)

    speed_limits = {
      "Dashboard": self.car_speed_limit,
      "Offline Maps": self.map_speed_limit,
      "Navigation": self.nav_speed_limit,
    }

    for priority in [
      self.frogpilot_toggles.speed_limit_priority1,
      self.frogpilot_toggles.speed_limit_priority2,
      self.frogpilot_toggles.speed_limit_priority3,
    ]:
      if speed_limits.get(priority, 0) in filtered_limits:
        return speed_limits[priority]

    if self.frogpilot_toggles.slc_fallback_previous:
      return self.prv_speed_limit

    if self.frogpilot_toggles.use_set_speed:
      return self.max_speed_limit

    return 0
