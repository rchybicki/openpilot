# PFEIFER - LD

# Acknowledgements:
# Lane detection logic was pulled from frogpilot. A huge thanks to FrogAi! https://github.com/FrogAi/FrogPilot

import numpy as np
from openpilot.common.params import Params
params = Params()

class LaneDetection:
  lane_lines = []

  @property
  def enabled(self) -> bool:
    try:
      return params.get_bool("LaneDetection")
    except:
      return False

  def update(self, model):
    self.lane_lines = model.laneLines


  def lane_valid(self, one_blinker: bool, carstate):
    enabled = self.enabled
    if not enabled:
      return True

    if not one_blinker:
      return False

    # Set the minimum lane threshold to 2.8 meters
    min_lane_threshold = 2.8
    # Set the blinker index based on which signal is on
    blinker_index = 0 if carstate.leftBlinker else 1
    current_lane = self.lane_lines[blinker_index + 1]
    desired_lane = self.lane_lines[blinker_index] if carstate.leftBlinker else self.lane_lines[blinker_index + 2]
    # Check if both the desired lane and the current lane have valid x and y values
    if all([desired_lane.x, desired_lane.y, current_lane.x, current_lane.y]) and len(desired_lane.x) == len(current_lane.x):
      # Interpolate the x and y values to the same length
      x = np.linspace(desired_lane.x[0], desired_lane.x[-1], num=len(desired_lane.x))
      lane_y = np.interp(x, current_lane.x, current_lane.y)
      desired_y = np.interp(x, desired_lane.x, desired_lane.y)
      # Calculate the width of the lane we're wanting to change into
      lane_width = np.abs(desired_y - lane_y)
      # Set lane_available to True if the lane width is larger than the threshold
      return np.amax(lane_width) >= min_lane_threshold
    else:
      return False

ld = LaneDetection()
