#!/usr/bin/env python3
# PFEIFER - SLC - Modified by FrogAi for FrogPilot
import calendar
import numpy as np
import requests

from concurrent.futures import ThreadPoolExecutor

from openpilot.common.constants import CV
from openpilot.common.realtime import DT_MDL

from openpilot.frogpilot.common.frogpilot_utilities import calculate_bearing_offset, is_url_pingable

FREE_MAPBOX_REQUESTS = 100_000
MAX_NON_POLAND_OFFSET_KPH = 10
MAX_NON_POLAND_OFFSET = MAX_NON_POLAND_OFFSET_KPH * CV.KPH_TO_MS
MAX_SUPPORTED_SPEED_LIMIT = 150 * CV.KPH_TO_MS

# Simplified Poland border for offline country detection in the driving loop.
POLAND_BOUNDARY = [
  (49.5103, 18.8332), (49.9072, 18.5592), (50.0468, 18.0024), (49.9736, 17.8394), (50.1063, 17.6328),
  (50.3110, 17.7080), (50.2406, 17.4242), (50.4329, 16.8930), (50.2185, 17.0148), (50.0930, 16.6606),
  (50.4063, 16.1996), (50.5676, 16.4259), (50.6440, 16.3316), (50.6036, 15.9820), (50.7427, 15.7922),
  (50.7755, 15.3561), (51.0116, 15.1444), (50.8584, 14.8104), (51.2717, 15.0195), (51.8039, 14.5858),
  (52.0767, 14.7614), (52.3822, 14.5454), (52.5769, 14.6448), (52.8507, 14.1239), (53.2518, 14.4416),
  (53.7000, 14.2639), (53.5984, 14.5906), (53.8515, 14.6306), (53.9065, 14.1753), (54.2630, 16.1792),
  (54.5572, 16.5696), (54.8383, 18.1524), (54.6901, 18.7517), (54.6031, 18.8353), (54.7465, 18.4131),
  (54.4337, 18.5881), (54.3502, 18.8859), (54.4567, 19.6095), (54.4009, 22.8376), (54.1549, 23.4490),
  (53.6113, 23.5909), (53.1520, 23.8937), (52.7426, 23.9225), (52.2894, 23.1656), (52.0845, 23.6375),
  (51.5927, 23.5434), (51.4044, 23.6976), (51.3047, 23.6352), (50.8564, 24.1432), (50.8080, 23.9576),
  (50.5408, 24.1077), (50.3682, 23.6822), (49.5674, 22.6658), (49.1612, 22.6817), (48.9940, 22.8553),
  (49.4265, 21.6012), (49.4192, 21.0688), (49.2903, 20.9190), (49.3916, 20.3177), (49.1732, 20.0505),
  (49.1942, 19.7607), (49.3931, 19.7693), (49.5981, 19.4573), (49.3942, 19.1417), (49.3892, 18.9623),
  (49.5043, 18.9322), (49.5103, 18.8332),
]
POLAND_MIN_LATITUDE = min(point[0] for point in POLAND_BOUNDARY)
POLAND_MAX_LATITUDE = max(point[0] for point in POLAND_BOUNDARY)
POLAND_MIN_LONGITUDE = min(point[1] for point in POLAND_BOUNDARY)
POLAND_MAX_LONGITUDE = max(point[1] for point in POLAND_BOUNDARY)

# Lookup table for speed limit kph offset depending on speed.
LIMIT_PERC_OFFSET_BP = [14.9, 15.0, 41.9, 42.0, 59.9, 60.0, 60.1, 99.9, 100.0, 119.9, 120.0, 129.9, 130.0, 139.9, 140.0, 144.9, 145.0]
LIMIT_PERC_OFFSET_V_GAP2 = [0, 5.0, 10.0, 10.0, 10.0, 10.0, 15.0, 15.0, 20.0, 20.0, 20.0, 20.0, 15.0, 5.0, 5.0, 5.0, 0]

OFFSET_MAP_IMPERIAL = [
  (0, 11.2, "speed_limit_offset1"),     # 0–24 mph
  (11.2, 15.2, "speed_limit_offset2"),  # 25–34
  (15.2, 19.6, "speed_limit_offset3"),  # 35–44
  (19.6, 24.1, "speed_limit_offset4"),  # 45–54
  (24.1, 28.6, "speed_limit_offset5"),  # 55–64
  (28.6, 33.1, "speed_limit_offset6"),  # 65–74
  (33.1, 44.2, "speed_limit_offset7"),  # 75–99
]

OFFSET_MAP_METRIC = [
  (0, 8.1, "speed_limit_offset1"),      # 0–29 km/h
  (8.1, 13.6, "speed_limit_offset2"),   # 30–49
  (13.6, 16.4, "speed_limit_offset3"),  # 50–59
  (16.4, 21.9, "speed_limit_offset4"),  # 60–79
  (21.9, 27.5, "speed_limit_offset5"),  # 80–99
  (27.5, 33.1, "speed_limit_offset6"),  # 100–119
  (33.1, 38.9, "speed_limit_offset7"),  # 120–140
]

def coordinate_in_polygon(latitude, longitude, polygon):
  inside = False
  previous_latitude, previous_longitude = polygon[-1]

  for current_latitude, current_longitude in polygon:
    crosses_latitude = (current_latitude > latitude) != (previous_latitude > latitude)
    if crosses_latitude:
      edge_longitude = (previous_longitude - current_longitude) * (latitude - current_latitude) / (previous_latitude - current_latitude) + current_longitude
      if longitude < edge_longitude:
        inside = not inside

    previous_latitude = current_latitude
    previous_longitude = current_longitude

  return inside

def coordinate_in_poland(latitude, longitude):
  if not POLAND_MIN_LATITUDE <= latitude <= POLAND_MAX_LATITUDE:
    return False
  if not POLAND_MIN_LONGITUDE <= longitude <= POLAND_MAX_LONGITUDE:
    return False
  return coordinate_in_polygon(latitude, longitude, POLAND_BOUNDARY)

class SpeedLimitController:
  def __init__(self, FrogPilotVCruise):
    self.frogpilot_planner = FrogPilotVCruise.frogpilot_planner

    self.calling_mapbox = False
    self.override_slc = False

    self.denied_target = 0
    self.map_speed_limit = 0
    self.mapbox_limit = 0
    self.next_speed_limit = 0
    self.overridden_speed = 0
    self.segment_distance = 0
    self.speed_limit_changed_timer = 0
    self.target = 0
    self.unconfirmed_speed_limit = 0

    self.previous_source = "None"
    self.source = "None"

    self.mapbox_requests = self.frogpilot_planner.params.get("MapBoxRequests")
    self.mapbox_requests.setdefault("total_requests", 0)
    self.mapbox_requests.setdefault("max_requests", FREE_MAPBOX_REQUESTS - (28 * 100))

    self.mapbox_host = "https://api.mapbox.com"
    self.mapbox_token = self.frogpilot_planner.params.get("MapboxSecretKey")

    self.previous_target = self.supported_speed_limit(self.frogpilot_planner.params.get("PreviousSpeedLimit"))

    self.executor = ThreadPoolExecutor(max_workers=1)

    self.session = requests.Session()
    self.session.headers.update({"Accept-Language": "en"})
    self.session.headers.update({"User-Agent": "frogpilot-mapbox-speed-limit-retriever/1.0 (https://github.com/FrogAi/FrogPilot)"})

  @property
  def experimental_mode(self):
    return self.target == 0 and self.frogpilot_toggles.slc_fallback_experimental_mode

  @property
  def offset(self):
    return self.get_offset(self.target)

  @property
  def in_poland(self):
    if not self.frogpilot_planner.gps_valid or self.frogpilot_planner.gps_position is None:
      return True

    latitude = self.frogpilot_planner.gps_position.get("latitude")
    longitude = self.frogpilot_planner.gps_position.get("longitude")
    if latitude is None or longitude is None:
      return True

    return coordinate_in_poland(latitude, longitude)

  def get_offset(self, speed_limit):
    offset = float(np.interp(speed_limit * CV.MS_TO_KPH, LIMIT_PERC_OFFSET_BP, LIMIT_PERC_OFFSET_V_GAP2))
    if not self.in_poland:
      offset = min(offset, MAX_NON_POLAND_OFFSET_KPH)
    return offset * CV.KPH_TO_MS

  def supported_speed_limit(self, speed_limit):
    return min(speed_limit, MAX_SUPPORTED_SPEED_LIMIT) if speed_limit > 0 else 0

  def calculate_change_distance(self, v_ego, v_desired):
    accel = 0.95 / 0.9 if v_desired > v_ego else -1.2 / 1.1
    return max((v_desired**2 - v_ego**2) / (2 * accel), 0)

  def get_mapbox_speed_limit(self, now, time_validated, v_ego, sm):
    if not self.frogpilot_planner.gps_valid or not self.mapbox_token or (sm["carState"].steeringAngleDeg - sm["liveParameters"].angleOffsetDeg) >= 45:
      self.mapbox_limit = 0
      self.segment_distance = 0
      return

    if v_ego < 1:
      return

    if self.segment_distance > 0:
      self.segment_distance -= v_ego * DT_MDL
      return

    if self.calling_mapbox:
      self.segment_distance = v_ego
      return

    def make_request():
      try:
        self.calling_mapbox = True

        successful = False

        if not is_url_pingable(self.mapbox_host):
          self.segment_distance = 1000
          return None

        if time_validated:
          current_month = now.month
          if current_month != self.mapbox_requests.get("month"):
            self.mapbox_requests.update({
              "month": current_month,
              "total_requests": 0,
              "max_requests": FREE_MAPBOX_REQUESTS - calendar.monthrange(now.year, current_month)[1] * 100,
            })

        self.mapbox_requests["total_requests"] += 1
        self.frogpilot_planner.params.put_nonblocking("MapBoxRequests", self.mapbox_requests)

        current_bearing = self.frogpilot_planner.gps_position.get("bearing")
        current_latitude = self.frogpilot_planner.gps_position.get("latitude")
        current_longitude = self.frogpilot_planner.gps_position.get("longitude")

        future_latitude, future_longitude = calculate_bearing_offset(current_latitude, current_longitude, current_bearing, v_ego)

        url = (
          f"{self.mapbox_host}/matching/v5/mapbox/driving/"
          f"{current_longitude},{current_latitude};"
          f"{future_longitude},{future_latitude}.json"
        )

        mapbox_params = {
          "access_token": self.mapbox_token,
          "annotations": "maxspeed,distance",
          "geometries": "polyline6",
          "overview": "full",
          "steps": "false",
          "radiuses": "10;10",
          "tidy": "true",
        }

        response = self.session.get(url, params=mapbox_params, timeout=10)
        response.raise_for_status()

        successful = True

        return response.json()
      except Exception as exception:
        print(f"Unexpected error in Mapbox request: {exception}")
      finally:
        self.calling_mapbox = False

        if not successful:
          self.mapbox_limit = 0
          self.segment_distance = v_ego

          return None

    def complete_request(future):
      try:
        data = future.result()
        if data:
          matchings = data.get("matchings") or []
          if not matchings:
            self.mapbox_limit = 0
            self.segment_distance = v_ego
            return

          legs = (matchings[0] or {}).get("legs") or []
          if not legs:
            self.mapbox_limit = 0
            self.segment_distance = v_ego
            return

          annotation = legs[0].get("annotation") or {}

          distances = annotation.get("distance") or [v_ego]
          segment_distance = distances[0]

          speed_data = annotation.get("maxspeed", [])
          speed_limit_kph = 0
          if speed_data:
            first_segment_speed = speed_data[0]
            speed_limit_kph = (first_segment_speed.get("speed") if first_segment_speed.get("speed") != "none" else 0) or 0

          if speed_limit_kph > 0:
            self.mapbox_limit = self.supported_speed_limit(speed_limit_kph * CV.KPH_TO_MS)
            self.segment_distance = segment_distance
            return

        self.mapbox_limit = 0
        self.segment_distance = v_ego

      except Exception as exception:
        print(f"Mapbox Callback Error: {exception}")
        self.mapbox_limit = 0
        self.segment_distance = v_ego

    future = self.executor.submit(make_request)
    future.add_done_callback(complete_request)

  def handle_limit_change(self, desired_source, desired_target, sm):
    self.speed_limit_changed_timer += DT_MDL

    speed_limit_accepted = (sm["frogpilotCarState"].accelPressed and sm["carControl"].longActive) or self.frogpilot_planner.params_memory.get_bool("SpeedLimitAccepted")
    speed_limit_denied = sm["frogpilotCarState"].decelPressed or (self.speed_limit_changed_timer >= 30)

    if speed_limit_accepted:
      self.overridden_speed = 0

      self.source = desired_source
      self.target = desired_target

      self.frogpilot_planner.params_memory.remove("SpeedLimitAccepted")

    elif speed_limit_denied:
      self.denied_target = desired_target

      self.previous_source = desired_source
      self.previous_target = desired_target

    elif desired_target < self.target and not self.frogpilot_toggles.speed_limit_confirmation_lower:
      self.source = desired_source
      self.target = desired_target

    elif desired_target > self.target and not self.frogpilot_toggles.speed_limit_confirmation_higher:
      self.source = desired_source
      self.target = desired_target

    else:
      self.source = "None"
      self.unconfirmed_speed_limit = desired_target

    if self.target != self.previous_target and self.target > 0 and not speed_limit_denied:
      self.denied_target = 0

      self.previous_source = self.source
      self.previous_target = self.target

      self.frogpilot_planner.params.put_nonblocking("PreviousSpeedLimit", self.target)

  def update_limits(self, dashboard_speed_limit, now, time_validated, v_cruise, v_ego, sm):
    self.update_map_speed_limit(v_ego, sm)

    limits = {
      "Dashboard": self.supported_speed_limit(dashboard_speed_limit),
      "Map Data": self.map_speed_limit
    }
    filtered_limits = {source: limit for source, limit in limits.items() if limit >= 1}

    if self.frogpilot_toggles.speed_limit_priority_highest:
      desired_source = max(filtered_limits, key=filtered_limits.get, default="None")
      desired_target = filtered_limits.get(desired_source, 0)

    elif self.frogpilot_toggles.speed_limit_priority_lowest:
      desired_source = min(filtered_limits, key=filtered_limits.get, default="None")
      desired_target = filtered_limits.get(desired_source, 0)

    elif filtered_limits:
      for priority in [
        self.frogpilot_toggles.speed_limit_priority1,
        self.frogpilot_toggles.speed_limit_priority2
      ]:
        if priority in filtered_limits:
          desired_source = priority
          desired_target = filtered_limits[desired_source]
          break
      else:
        desired_source = "None"
        desired_target = 0

    else:
      desired_source = "None"
      desired_target = 0

    if desired_target == 0 or self.target == 0:
      if self.mapbox_requests["total_requests"] < self.mapbox_requests["max_requests"] and self.frogpilot_toggles.slc_mapbox_filler:
        self.get_mapbox_speed_limit(now, time_validated, v_ego, sm)

        if self.mapbox_limit >= 1:
          desired_source = "Mapbox"
          desired_target = self.mapbox_limit

      if desired_target == 0 or self.target == 0:
        if self.denied_target != self.previous_target > 0 and self.frogpilot_toggles.slc_fallback_previous_speed_limit:
          desired_source = self.previous_source
          desired_target = self.previous_target

          self.target = desired_target

        elif sm["selfdriveState"].enabled and self.frogpilot_toggles.slc_fallback_set_speed:
          desired_source = "None"
          desired_target = v_cruise
    else:
      self.mapbox_limit = 0
      self.segment_distance = 0

    if abs(desired_target - self.previous_target) >= 1:
      self.handle_limit_change(desired_source, desired_target, sm)
    elif desired_source != self.source and abs(desired_target - self.target) < 1:
      self.source = desired_source
    else:
      self.speed_limit_changed_timer = 0
      self.unconfirmed_speed_limit = 0

  def update_map_speed_limit(self, v_ego, sm):
    self.map_speed_limit = self.supported_speed_limit(sm["mapdOut"].speedLimit)
    self.next_speed_limit = self.supported_speed_limit(sm["mapdOut"].nextSpeedLimit)

    if self.next_speed_limit > 0:
      change_distance = self.calculate_change_distance(v_ego, self.next_speed_limit)

      if self.map_speed_limit < self.next_speed_limit:
        configured_lookahead = self.frogpilot_toggles.map_speed_lookahead_higher * v_ego
      elif self.map_speed_limit > self.next_speed_limit:
        configured_lookahead = self.frogpilot_toggles.map_speed_lookahead_lower * v_ego
      else:
        configured_lookahead = 0

      max_lookahead = max(change_distance, configured_lookahead)
      if sm["mapdOut"].nextSpeedLimitDistance < max_lookahead:
        self.map_speed_limit = self.next_speed_limit

  def update_override(self, v_cruise, v_cruise_diff, v_ego, v_ego_diff, sm):
    offset = self.get_offset(self.target)
    minimum_overridden_speed = self.target + offset
    maximum_set_speed_override = v_cruise + v_cruise_diff
    if not self.in_poland and self.target > 0:
      maximum_set_speed_override = min(maximum_set_speed_override, self.target + MAX_NON_POLAND_OFFSET)
    maximum_set_speed_override = max(maximum_set_speed_override, minimum_overridden_speed)
    maximum_gas_override = max(v_cruise + v_cruise_diff, minimum_overridden_speed)

    self.override_slc = self.overridden_speed > minimum_overridden_speed > 0
    self.override_slc |= sm["carState"].gasPressed and v_ego + v_ego_diff > minimum_overridden_speed > 0
    self.override_slc &= sm["selfdriveState"].enabled

    if self.override_slc:
      if self.frogpilot_toggles.speed_limit_controller_override_manual:
        if sm["carState"].gasPressed:
          self.overridden_speed = max(v_ego + v_ego_diff, self.overridden_speed)
        self.overridden_speed = float(np.clip(self.overridden_speed, minimum_overridden_speed, maximum_gas_override))
      elif self.frogpilot_toggles.speed_limit_controller_override_set_speed:
        self.overridden_speed = maximum_set_speed_override

      self.source = "None"
    else:
      self.overridden_speed = 0
