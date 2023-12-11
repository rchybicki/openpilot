from openpilot.common.params import Params
from openpilot.common.conversions import Conversions as CV
import json
import math
from openpilot.common.numpy_fast import interp
from time import time

mem_params = Params("/dev/shm/params")
params = Params()

R = 6373000.0 # approximate radius of earth in meters
TO_RADIANS = math.pi / 180
TO_DEGREES = 180 / math.pi
TARGET_JERK = -1.0 # m/s^3 should match up with the long planner
TARGET_ACCEL = -1.2 # m/s^2 should match up with the long planner
TARGET_JOUNCE = -0.08 # m/s^5 should match up with the long planner
TARGET_OFFSET = 3.0 # seconds - This controls how soon before the curve you reach the target velocity. It also helps
                    # reach the target velocity when innacuracies in the distance modeling logic would cause overshoot.
                    # The value is multiplied against the target velocity to determine the additional distance. This is
                    # done to keep the distance calculations consistent but results in the offset actually being less
                    # time than specified depending on how much of a speed diffrential there is between v_ego and the
                    # target velocity.

def calculate_jerk(t, j_ego):
  return j_ego + TARGET_JOUNCE * t

def calculate_accel(t, target_jounce, j_ego, a_ego):
  return a_ego  + j_ego * t + target_jounce/2 * (t ** 2)

def calculate_velocity(t, target_jounce, j_ego, a_ego, v_ego):
  return v_ego + a_ego * t + j_ego/2 * (t ** 2) + target_jounce/6 * (t ** 3)

def calculate_distance(t, target_jounce, j_ego, a_ego, v_ego):
  return t * v_ego + a_ego/2 * (t ** 2) + j_ego/6 * (t ** 3) + target_jounce/24 * (t ** 4)


# points should be in radians
# output is meters
def distance_to_point(ax, ay, bx, by):
  a = math.sin((bx-ax)/2)*math.sin((bx-ax)/2) + math.cos(ax) * math.cos(bx)*math.sin((by-ay)/2)*math.sin((by-ay)/2)
  c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))

  return R * c  # in meters

class MapTurnSpeedController:
  def __init__(self):
    self.enabled = params.get_bool("MTSCEnabled")
    self.last_params_update = time()

  def update_params(self):
    t = time()
    if t > self.last_params_update + 5.0:
      self.enabled = params.get_bool("MTSCEnabled")
      self.last_params_update = t

  def target_speed(self, v_ego, a_ego, j_ego) -> float:
    self.update_params()

    if not self.enabled:
      return 0.0

    lat = 0.0
    lon = 0.0
    try:
      position = json.loads(mem_params.get("LastGPSPosition"))
      lat = position["latitude"]
      lon = position["longitude"]
    except: return 0.0

    try:
      target_velocities = json.loads(mem_params.get("MapTargetVelocities"))
    except: return 0.0

    min_dist = 1000
    min_idx = 0
    distances = []

    # find our location in the path
    for i in range(len(target_velocities)):
      target_velocity = target_velocities[i]
      tlat = target_velocity["latitude"]
      tlon = target_velocity["longitude"]
      d = distance_to_point(lat * TO_RADIANS, lon * TO_RADIANS, tlat * TO_RADIANS, tlon * TO_RADIANS)
      distances.append(d)
      if d < min_dist:
        min_dist = d
        min_idx = i

    # only look at values from our current position forward
    forward_points = target_velocities[min_idx:]
    forward_distances = distances[min_idx:]

    # find velocities that we are within the distance we need to adjust for
    valid_velocities = []
    for i in range(len(forward_points)):
      target_velocity = forward_points[i]
      tlat = target_velocity["latitude"]
      tlon = target_velocity["longitude"]
      tv = target_velocity["velocity"]
      if tv > v_ego:
        continue

      d = forward_distances[i]
      target_jerk_t = max(0.1, abs((j_ego - TARGET_JERK) / TARGET_JOUNCE)) # seconds to reach target jerk

      a = TARGET_JOUNCE/2
      b = j_ego
      c = a_ego - TARGET_ACCEL
      t_a = -1 * ((b**2 - 4 * a * c) ** 0.5 + b) / 2 * a
      t_b = ((b**2 - 4 * a * c) ** 0.5 - b) / 2 * a
      target_accel_t = 0.0
      if not isinstance(t_a, complex) and t_a > 0:
          target_accel_t = t_a
      else:
          target_accel_t = t_b
      if isinstance(target_accel_t, complex):
        continue

      min_accel_v = 0.0
      min_jerk_v = 0.0
      a_intermediate = 0.0
      v_intermediate = 0.0
      additional_accel_t = 0.0
      if target_jerk_t < target_accel_t:
        a_intermediate = calculate_accel(target_jerk_t, TARGET_JOUNCE, j_ego, a_ego)
        v_intermediate = calculate_velocity(target_jerk_t, TARGET_JOUNCE, j_ego, a_ego, v_ego)
        a_diff = (a_intermediate - TARGET_ACCEL)
        additional_accel_t = abs(a_diff / TARGET_JERK)
        min_accel_v = calculate_velocity(additional_accel_t, 0, TARGET_JERK, a_intermediate, v_intermediate)
      else:
        target_jerk_t = target_accel_t
        min_jerk_v = calculate_velocity(target_accel_t, TARGET_JOUNCE, j_ego, a_ego, v_ego)
        min_accel_v = min_jerk_v

      max_d = 0
      if tv > min_jerk_v:
        # we're gonna fudge the numbers a bit because calculating time from jounce is difficult
        v = 100
        t = target_jerk_t / (v_ego - min_jerk_v) * (v_ego - tv)
        while v > tv: # add time until the calculated velocity is below the target
            t += 0.5
            v = calculate_velocity(t, TARGET_JOUNCE, j_ego, a_ego, v_ego)

        max_d = calculate_distance(t, TARGET_JOUNCE, j_ego, a_ego, v_ego)
      elif tv > min_accel_v:
        t = target_jerk_t
        max_d = calculate_distance(t, TARGET_JOUNCE, j_ego, a_ego, v_ego)

        # calculate additional time needed based on target jerk
        a = 0.5 * TARGET_JERK
        b = a_intermediate
        c = v_intermediate - tv
        t_a = -1 * ((b**2 - 4 * a * c) ** 0.5 + b) / 2 * a
        t_b = ((b**2 - 4 * a * c) ** 0.5 - b) / 2 * a
        if not isinstance(t_a, complex) and t_a > 0:
          t = t_a
        else:
          t = t_b
        if isinstance(t, complex):
          continue

        max_d = max_d + calculate_distance(t, 0, TARGET_JERK, a_intermediate, v_intermediate)

      else:
        t = target_jerk_t
        max_d = calculate_distance(t, TARGET_JOUNCE, j_ego, a_ego, v_ego)

        # calculate distance traveled to reach the min accel
        t = additional_accel_t
        max_d += calculate_distance(t, 0, TARGET_JERK, a_intermediate, v_intermediate)

        # calculate additional time needed based on target accel
        t = abs((min_accel_v - tv) / TARGET_ACCEL)
        max_d += calculate_distance(t, 0, 0, TARGET_ACCEL, min_accel_v)

      if d < max_d + tv * TARGET_OFFSET:
        valid_velocities.append(float(tv))

    # Find the smallest velocity we need to adjust for
    min_v = 100.0
    for tv in valid_velocities:
      if tv < min_v:
        min_v = tv


    return min_v

mtsc = MapTurnSpeedController()
