# PFEIFER - SLC - Modified by FrogAi for FrogPilot
import json
import math
import numpy as np

from openpilot.common.conversions import Conversions as CV
from cereal import log
from openpilot.selfdrive.frogpilot.frogpilot_utilities import calculate_distance_to_point
from openpilot.selfdrive.frogpilot.frogpilot_variables import TO_RADIANS, params, params_memory


# Lookup table for speed limit kph offset depending on speed, RCH Custom
_LIMIT_PERC_OFFSET_BP = [14.9, 15, 41.9, 42.0, 59.9, 60.0, 60.1, 99.9, 100.0]
_LIMIT_PERC_OFFSET_V_GAP4 = [0, 0, 0, 0, 0, 0, 0, 0, 0]
_LIMIT_PERC_OFFSET_V_GAP3 = [0, 5.0, 5.0, 5.0, 5.0, 5.0, 10.0, 10.0, 10.0]
_LIMIT_PERC_OFFSET_V_GAP2 = [0, 5.0, 10.0, 10.0, 10.0, 10.0, 15.0, 15.0, 20.0]
_LIMIT_PERC_OFFSET_V_GAP1 = [0, 10.0, 15.0, 20.0, 20.0, 20.0, 25.0, 25.0, 30.0]


class SpeedLimitController:
  def __init__(self):
    self.experimental_mode = False

    self.desired_speed_limit = 0
    self.map_speed_limit = 0
    self.offset = 0
    self.speed_limit = 0
    self.upcoming_speed_limit = 0

    self.source = "None"

    self.previous_speed_limit = params.get_float("PreviousSpeedLimit")

  def update(self, dashboard_speed_limit, enabled, navigation_speed_limit, v_cruise, v_ego, frogpilot_toggles):
    self.update_map_speed_limit(v_ego, frogpilot_toggles)
    max_speed_limit = v_cruise if enabled else 0

    self.speed_limit = self.get_speed_limit(dashboard_speed_limit, max_speed_limit, navigation_speed_limit, frogpilot_toggles)
    self.offset = self.get_offset(frogpilot_toggles)
    self.desired_speed_limit = self.get_desired_speed_limit()

    self.experimental_mode = frogpilot_toggles.slc_fallback_experimental_mode and self.speed_limit == 0

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


  def get_desired_speed_limit(self):
    if self.speed_limit > 1:
      if self.previous_speed_limit != self.speed_limit:
        params.put_float_nonblocking("PreviousSpeedLimit", self.speed_limit)
        self.previous_speed_limit = self.speed_limit
      return self.speed_limit + self.offset
    return 0

  def update_map_speed_limit(self, v_ego, frogpilot_toggles):
    self.map_speed_limit = params_memory.get_float("MapSpeedLimit")

    next_map_speed_limit = json.loads(params_memory.get("NextMapSpeedLimit", "{}"))
    next_lat = next_map_speed_limit.get("latitude", 0)
    next_lon = next_map_speed_limit.get("longitude", 0)
    self.upcoming_speed_limit = next_map_speed_limit.get("speedlimit", 0)

    position = self.get_param_memory("LastGPSPosition", is_json=True)
    position = json.loads(params_memory.get("LastGPSPosition", "{}"))
    lat = position.get("latitude", 0)
    lon = position.get("longitude", 0)

    if next_map_speed_limit_value > 1:
      d = distance_to_point(lat * TO_RADIANS, lon * TO_RADIANS, next_map_speed_limit_lat * TO_RADIANS, next_map_speed_limit_lon * TO_RADIANS)
    if self.upcoming_speed_limit > 1:
      distance = calculate_distance_to_point(lat * TO_RADIANS, lon * TO_RADIANS, next_lat * TO_RADIANS, next_lon * TO_RADIANS)
      change_distance = self.calculate_change_distance(v_ego, next_map_speed_limit_value)

      if d < change_distance:
        self.map_speed_limit = next_map_speed_limit_value

  @property
  def experimental_mode(self):
    return self.speed_limit == 0 and self.frogpilot_toggles.use_experimental_mode

  @property
  def desired_speed_limit(self):
    if self.speed_limit > 1:
      self.update_previous_limit(self.speed_limit)
      return self.speed_limit + self.offset
    return 0

      if distance < change_distance:
        self.map_speed_limit = self.upcoming_speed_limit

  def get_offset(self, frogpilot_toggles):
    # personality_gaps = {
    #   log.LongitudinalPersonality.relaxed: _LIMIT_PERC_OFFSET_V_GAP2,
    #   log.LongitudinalPersonality.standard: _LIMIT_PERC_OFFSET_V_GAP2,
    #   log.LongitudinalPersonality.aggressive: _LIMIT_PERC_OFFSET_V_GAP1,
    #   # snow
    #   3: _LIMIT_PERC_OFFSET_V_GAP3,
    # }

    # gap_values = personality_gaps.get(self.personality)

    return float(np.interp(self.speed_limit * CV.MS_TO_KPH, _LIMIT_PERC_OFFSET_BP, _LIMIT_PERC_OFFSET_V_GAP2) * CV.KPH_TO_MS)


  def get_speed_limit(self, dashboard_speed_limit, max_speed_limit, navigation_speed_limit, frogpilot_toggles):
    limits = {
      "Dashboard": dashboard_speed_limit,
      "Map Data": self.map_speed_limit,
      "Navigation": navigation_speed_limit
    }
    filtered_limits = {source: float(limit) for source, limit in limits.items() if limit > 1}

    if filtered_limits:
      if frogpilot_toggles.speed_limit_priority_highest:
        self.source = max(filtered_limits, key=filtered_limits.get)
        return filtered_limits[self.source]

      if frogpilot_toggles.speed_limit_priority_lowest:
        self.source = min(filtered_limits, key=filtered_limits.get)
        return filtered_limits[self.source]

      for priority in [
        frogpilot_toggles.speed_limit_priority1,
        frogpilot_toggles.speed_limit_priority2,
        frogpilot_toggles.speed_limit_priority3
      ]:
        if priority is not None and priority in filtered_limits:
          self.source = priority
          return filtered_limits[priority]

    self.source = "None"

    if frogpilot_toggles.slc_fallback_previous_speed_limit:
      return self.previous_speed_limit

    if frogpilot_toggles.slc_fallback_set_speed:
      return max_speed_limit

    return 0
