#!/usr/bin/env python3
import json
import threading
from traceback import print_exception
import numpy as np
from cereal import log
import cereal.messaging as messaging
from openpilot.common.params import Params
from openpilot.common.realtime import Ratekeeper
from openpilot.selfdrive.mapd.lib.osm import OSM
from openpilot.selfdrive.mapd.lib.geo import distance_to_points
from openpilot.selfdrive.mapd.lib.WayCollection import WayCollection
from openpilot.selfdrive.mapd.config import QUERY_RADIUS, MIN_DISTANCE_FOR_NEW_QUERY
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.controls.speed_limit_controller import slc
from openpilot.selfdrive.mapd.lib.geo import DIRECTION

_DEBUG = False
_CLOUDLOG_DEBUG = True
ROAD_NAME_TIMEOUT = 30 # secs


def _debug(msg, log_to_cloud=True):
  if _CLOUDLOG_DEBUG and log_to_cloud:
    cloudlog.debug(msg)
  if _DEBUG:
    print(msg)


def excepthook(args):
  _debug(f'MapD: Threading exception:\n{args}')
  print_exception(args.exc_type, args.exc_value, args.exc_traceback)


threading.excepthook = excepthook


class MapD():
  def __init__(self):
    self.osm = OSM()
    self.way_collection = None
    self.route = None
    self.location_deg = None  # The current location in degrees.
    self.location_rad = None  # The current location in radians as a Numpy array.
    self.bearing_rad = None
    self.location_stdev = 1  # The current location accuracy in mts. 1 standard devitation.
    self.last_fetch_location = None
    self._query_thread = None
    self.params = Params()
    self._lock = threading.RLock()

    # dp - use LastGPSPosition as init position (if we are in a undercover car park?)
    # this way we can prefetch osm data before we get a fix.
    last_pos = Params().get("LastGPSPosition")
    if last_pos is not None and last_pos != "":
      l = json.loads(last_pos)
      lat = float(l["latitude"])
      lon = float(l["longitude"])
      self.location_rad = np.radians(np.array([lat, lon], dtype=float))
      self.location_deg = (lat, lon)
      self.bearing_rad = np.radians(0, dtype=float)


  def update_gps(self, sm):
    sock = 'liveLocationKalman'
    if not sm.updated[sock] or not sm.valid[sock]:
      return

    location = sm[sock]
    self.last_gps = location

    locationd_valid = (location.status == log.LiveLocationKalman.Status.valid) and location.positionGeodetic.valid

    self.location_valid = locationd_valid

    if locationd_valid:
      self.bearing_rad = location.calibratedOrientationNED.value[2]
      self.location_rad = np.radians(np.array([location.positionGeodetic.value[0], location.positionGeodetic.value[1]]))
      self.location_deg = (location.positionGeodetic.value[0], location.positionGeodetic.value[1])


  def _query_osm_not_blocking(self):
    def query(osm, location_deg, location_rad, radius):
      lat, lon = location_deg
      areas, ways = osm.fetch_road_ways_around_location(lat, lon, radius)

      # Only issue an update if we received some ways. Otherwise it is most likely a connectivity issue.
      # Will retry on next loop.
      if len(ways) > 0:
        new_way_collection = WayCollection(areas, ways, location_rad)

        # Use the lock to update the way_collection as it might be being used to update the route.
        with self._lock:
          self.way_collection = new_way_collection
          self.last_fetch_location = location_rad


    # Ignore if we have a query thread already running.
    if self._query_thread is not None and self._query_thread.is_alive():
      return

    self._query_thread = threading.Thread(target=query, args=(self.osm, self.location_deg, self.location_rad, QUERY_RADIUS))
    self._query_thread.start()

  def updated_osm_data(self):
    if self.route is not None:
      distance_to_end = self.route.distance_to_end
      if distance_to_end is not None and distance_to_end >= MIN_DISTANCE_FOR_NEW_QUERY:
        # do not query as long as we have a route with enough distance ahead.
        return

    if self.location_rad is None:
      return

    if self.last_fetch_location is not None:
      distance_since_last = distance_to_points(self.last_fetch_location, np.array([self.location_rad]))[0]
      if distance_since_last < QUERY_RADIUS - MIN_DISTANCE_FOR_NEW_QUERY:
        # do not query if are still not close to the border of previous query area
        return

    self._query_osm_not_blocking()

  def update_route(self):
    def update_proc():
      if self.way_collection is None or self.location_rad is None or self.bearing_rad is None:
        return

      # Create the route if not existent or if it was generated by an older way collection
      if self.route is None or self.route.way_collection_id != self.way_collection.id:
        self.route = self.way_collection.get_route(self.location_rad, self.bearing_rad, self.location_stdev)
        return

      self.route.update(self.location_rad, self.bearing_rad, self.location_stdev)
      if self.route.located:
        return

      # if an old route did not mange to locate, attempt to regenerate form way collection.
      self.route = self.way_collection.get_route(self.location_rad, self.bearing_rad, self.location_stdev)

    # We use the lock when updating the route, as it reads `way_collection` which can ben updated by
    # a new query result from the _query_thread.
    with self._lock:
      update_proc()


  def publish(self, pm, sm):
    # Ensure we have a route currently located
    if self.route is None or not self.route.located:
      # PFEIFER - SLC {{
      slc.load_state()
      slc.map_speed_limit = 0
      slc.write_map_state()
      # }} PFEIFER - SLC
      return
    
    next_speed_limit_section = self.route.next_speed_limit_section
    # turn_speed_limit_section = self.route.current_curvature_speed_limit_section
    # next_turn_speed_limit_sections = self.route.next_curvature_speed_limit_sections(horizon_mts)
    # current_road_name = self.route.current_road_name

    # map_data_msg.liveMapData.forceExperimentalMode = self.route.force_experimental_mode
    # map_data_msg.liveMapData.speedLimitValid = bool(speed_limit is not None)
    # map_data_msg.liveMapData.speedLimit = float(speed_limit if speed_limit is not None else 0.0)
    # map_data_msg.liveMapData.speedLimitAheadValid = bool(next_speed_limit_section is not None)
    # map_data_msg.liveMapData.speedLimitAhead = float(next_speed_limit_section.value
    #                                                  if next_speed_limit_section is not None else 0.0)
    # map_data_msg.liveMapData.speedLimitAheadDistance = float(next_speed_limit_section.start
    #                                                          if next_speed_limit_section is not None else 0.0)

    # map_data_msg.liveMapData.turnSpeedLimitValid = bool(turn_speed_limit_section is not None)
    # map_data_msg.liveMapData.turnSpeedLimit = float(turn_speed_limit_section.value
    #                                                 if turn_speed_limit_section is not None else 0.0)
    # map_data_msg.liveMapData.turnSpeedLimitSign = int(turn_speed_limit_section.curv_sign
    #                                                   if turn_speed_limit_section is not None else 0)
    # map_data_msg.liveMapData.turnSpeedLimitEndDistance = float(turn_speed_limit_section.end
    #                                                            if turn_speed_limit_section is not None else 0.0)
    # map_data_msg.liveMapData.turnSpeedLimitsAhead = [float(s.value) for s in next_turn_speed_limit_sections]
    # map_data_msg.liveMapData.turnSpeedLimitsAheadDistances = [float(s.start) for s in next_turn_speed_limit_sections]
    # map_data_msg.liveMapData.turnSpeedLimitsAheadSigns = [float(s.curv_sign) for s in next_turn_speed_limit_sections]

    # PFEIFER - SLC {{
    slc.load_state()
    slc.map_speed_limit = self.route.current_speed_limit
    slc.map_next_speed_limit = float(next_speed_limit_section.value \
                                                    if next_speed_limit_section is not None else 0.0)
    slc.map_next_speed_limit_distance = float(next_speed_limit_section.start \
                                                    if next_speed_limit_section is not None else 0.0)
    next_speed_limit_way, next_speed_limit_way_distance_to_end = self.route.next_speed_limit_way_id_and_distance_to_end
    slc.map_next_speed_limit_way_id = 0 if next_speed_limit_way is None else next_speed_limit_way.id
    slc.map_next_speed_limit_way_distance_to_end = 0 if next_speed_limit_way_distance_to_end is None else (next_speed_limit_way_distance_to_end // 5) * 5
    slc.map_way_id = 0 if self.route.current_wr is None else self.route.current_wr.id
    slc.map_next_way_id = 0 if self.route.next_wr is None else self.route.next_wr.id
    slc.map_distance_to_end_of_current_way = 0 if self.route.distance_to_end_of_current_wr is None else (self.route.distance_to_end_of_current_wr // 5) * 5
    slc.map_distance_to_end_of_next_way = 0 if self.route.distance_to_end_of_next_wr is None else (self.route.distance_to_end_of_next_wr // 5) * 5


    slc.map_way_direction = None
    if self.route.current_wr is not None:
      if self.route.current_wr.direction == DIRECTION.FORWARD:
        slc.map_way_direction = "FORWARD"
      elif self.route.current_wr.direction == DIRECTION.BACKWARD:
        slc.map_way_direction = "BACKWARD"

    slc.map_next_way_direction = None
    if self.route.next_wr is not None:
      if self.route.next_wr.direction == DIRECTION.FORWARD:
        slc.map_next_way_direction = "FORWARD"
      elif self.route.next_wr.direction == DIRECTION.BACKWARD:
        slc.map_next_way_direction = "BACKWARD"

    slc.map_next_speed_limit_way_direction = None
    if next_speed_limit_way is not None:
      if next_speed_limit_way.direction == DIRECTION.FORWARD:
        slc.map_next_speed_limit_way_direction = "FORWARD"
      elif next_speed_limit_way.direction == DIRECTION.BACKWARD:
        slc.map_next_speed_limit_way_direction = "BACKWARD"

    slc.write_map_state()
    # }} PFEIFER - SLC


# provides live map data information
def mapd_thread(sm=None, pm=None):
  mapd = MapD()
  rk = Ratekeeper(1., print_delay_threshold=None)  # Keeps rate at 1 hz

  # *** setup messaging
  if sm is None:
    sm = messaging.SubMaster(['liveLocationKalman', 'carControl'])

  while True:
    sm.update()
    mapd.update_gps(sm)
    mapd.updated_osm_data()
    mapd.update_route()
    mapd.publish(pm, sm)
    rk.keep_time()


def main(sm=None, pm=None):
  mapd_thread(sm, pm)


if __name__ == "__main__":
  main()
