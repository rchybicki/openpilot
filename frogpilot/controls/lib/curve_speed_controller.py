#!/usr/bin/env python3
import numpy as np

from openpilot.common.realtime import DT_MDL

from openpilot.frogpilot.common.frogpilot_variables import CRUISING_SPEED, DEFAULT_LATERAL_ACCELERATION, MINIMUM_LATERAL_ACCELERATION, PLANNER_TIME

CALIBRATION_PROGRESS_THRESHOLD = 10 / DT_MDL
CURVATURE_DATA_VERSION = 2
CURVATURE_EPSILON = 1e-4
DEFAULT_CURVATURE_BP = [0.003, 0.007, 0.01, 0.013, 0.02, 0.05, 0.1]
DEFAULT_LATERAL_ACCELERATION_SCALE = [0.72, 0.80, 0.88, 1.0, 1.10, 1.20, 1.25]
LEARNING_CONFIDENCE_SAMPLES = 10 / DT_MDL
LEARNING_KERNEL_MIN_WIDTH = 0.002
LEARNING_KERNEL_RELATIVE_WIDTH = 0.25
LATERAL_ACCELERATION_MAX = 4.0
LATERAL_ACCELERATION_MIN = 1.0
MAX_CURVATURE = 0.1
MIN_CURVATURE = 0.003
PERCENTILE = 90
ROUNDING_PRECISION = 3
STEP = 0.001
TARGET_RELEASE_ACCELERATION = 0.5

def get_curve_speed(curvature, lateral_acceleration):
  if abs(curvature) < CURVATURE_EPSILON:
    return float("inf")
  return max((lateral_acceleration / abs(curvature))**0.5, CRUISING_SPEED)

def get_curve_target(current_target, v_ego, curve_speed, time_to_curve):
  if curve_speed < current_target:
    if v_ego <= curve_speed:
      return curve_speed

    decel_rate = (v_ego - curve_speed) / max(time_to_curve, DT_MDL)
    return max(current_target - decel_rate * DT_MDL, curve_speed)

  return min(current_target + TARGET_RELEASE_ACCELERATION * DT_MDL, curve_speed)

def get_weighted_percentile(values, weights, percentile):
  order = np.argsort(values)
  sorted_values = np.asarray(values)[order]
  sorted_weights = np.asarray(weights)[order]
  threshold = np.sum(sorted_weights) * percentile / 100
  index = min(int(np.searchsorted(np.cumsum(sorted_weights), threshold, side="left")), len(sorted_values) - 1)
  return float(sorted_values[index])

class CurveSpeedController:
  def __init__(self, FrogPilotVCruise):
    self.frogpilot_planner = FrogPilotVCruise.frogpilot_planner

    self.enable_training = False
    self.target_set = False

    self.training_timer = 0

    raw_curvature_data = self.frogpilot_planner.params.get("CurvatureData")
    if isinstance(raw_curvature_data, dict) and raw_curvature_data.get("version") == CURVATURE_DATA_VERSION:
      self.curvature_data = raw_curvature_data
      self.curvature_samples = self.curvature_data.setdefault("samples", {})
    else:
      self.curvature_data = {"version": CURVATURE_DATA_VERSION, "samples": {}}
      self.curvature_samples = self.curvature_data["samples"]
      self.frogpilot_planner.params.put_nonblocking("CalibrationProgress", 0.0)
      self.frogpilot_planner.params.put_nonblocking("CurvatureData", self.curvature_data)

    self.calculate_weights()
    self.update_lateral_acceleration()

  def calculate_weights(self):
    curvatures = np.arange(MIN_CURVATURE, MAX_CURVATURE + STEP, STEP)
    mid_point = (MIN_CURVATURE + MAX_CURVATURE) / 2

    self.curvature_weights = {}
    for curvature in curvatures:
      distance = abs(curvature - mid_point) / (MAX_CURVATURE - MIN_CURVATURE)
      weight = 1.0 + (4.0 * (1 - distance))
      self.curvature_weights[str(round(curvature, ROUNDING_PRECISION))] = weight

  def log_data(self, long_control_active, v_ego, sm):
    self.enable_training = v_ego > CRUISING_SPEED
    self.enable_training &= not self.frogpilot_planner.tracking_lead
    self.enable_training &= not long_control_active

    if self.enable_training:
      self.training_timer += DT_MDL

      current_curvature = abs(float(sm["controlsState"].curvature))
      lateral_acceleration = v_ego**2 * current_curvature
      valid_curve = MIN_CURVATURE <= current_curvature <= MAX_CURVATURE
      valid_curve &= lateral_acceleration >= MINIMUM_LATERAL_ACCELERATION
      valid_curve &= not (sm["carState"].leftBlinker or sm["carState"].rightBlinker)

      if valid_curve:
        road_curvature = abs(round(current_curvature, ROUNDING_PRECISION))
        key = str(road_curvature)
        if key in self.curvature_samples:
          data = self.curvature_samples[key]

          average = data["average"]
          count = data["count"]

          self.curvature_samples[key] = {
            "average": ((average * count) + lateral_acceleration) / (count + 1),
            "count": count + 1
          }
        else:
          self.curvature_samples[key] = {
            "average": lateral_acceleration,
            "count": 1
          }

      if self.training_timer >= PLANNER_TIME:
        self.save_training_data()

    elif self.training_timer > 0:
      self.save_training_data()

  def save_training_data(self):
    progress = 0.0
    total_weight = 0.0

    for key in list(self.curvature_weights.keys()):
      if key in self.curvature_samples:
        progress += min(self.curvature_samples[key]["count"] / CALIBRATION_PROGRESS_THRESHOLD, 1.0) * self.curvature_weights[key]

      total_weight += self.curvature_weights[key]

    self.frogpilot_planner.params.put_nonblocking("CalibrationProgress", float(min((progress / total_weight) * 100, 100.0)))
    self.frogpilot_planner.params.put_nonblocking("CurvatureData", self.curvature_data)
    self.update_lateral_acceleration()

    self.training_timer = 0

  def update_lateral_acceleration(self):
    calculated_lateral_acceleration = DEFAULT_LATERAL_ACCELERATION
    if self.curvature_samples:
      all_samples = [data["average"] for data in self.curvature_samples.values()]
      sample_counts = [data["count"] for data in self.curvature_samples.values()]
      calculated_lateral_acceleration = get_weighted_percentile(all_samples, sample_counts, PERCENTILE)

    self.frogpilot_planner.params.put_nonblocking("CalibratedLateralAcceleration", calculated_lateral_acceleration)

    override_value = self.frogpilot_planner.params.get("CalibratedLateralAccelerationOverride")
    try:
      override = float(override_value) if override_value is not None else 0.0
    except (TypeError, ValueError):
      override = 0.0

    if override > 0:
      override = float(np.clip(override, LATERAL_ACCELERATION_MIN, LATERAL_ACCELERATION_MAX))
      self.curve_speed_factor = override / max(calculated_lateral_acceleration, LATERAL_ACCELERATION_MIN)
    else:
      self.curve_speed_factor = 1.0
    self.lateral_acceleration = calculated_lateral_acceleration

  def get_lateral_acceleration(self, curvature):
    curvature = abs(curvature)
    default_scale = float(np.interp(curvature, DEFAULT_CURVATURE_BP, DEFAULT_LATERAL_ACCELERATION_SCALE))
    default_lateral_acceleration = float(np.clip(self.lateral_acceleration * default_scale, LATERAL_ACCELERATION_MIN, LATERAL_ACCELERATION_MAX))
    if not self.curvature_samples:
      return float(np.clip(default_lateral_acceleration * self.curve_speed_factor, LATERAL_ACCELERATION_MIN, LATERAL_ACCELERATION_MAX))

    bandwidth = max(LEARNING_KERNEL_MIN_WIDTH, curvature * LEARNING_KERNEL_RELATIVE_WIDTH)
    weighted_sum = 0.0
    total_weight = 0.0
    for key, data in self.curvature_samples.items():
      sample_curvature = float(key)
      distance = (sample_curvature - curvature) / bandwidth
      weight = data["count"] * np.exp(-0.5 * distance**2)
      weighted_sum += weight * np.clip(data["average"], LATERAL_ACCELERATION_MIN, LATERAL_ACCELERATION_MAX)
      total_weight += weight

    if total_weight == 0:
      return float(np.clip(default_lateral_acceleration * self.curve_speed_factor, LATERAL_ACCELERATION_MIN, LATERAL_ACCELERATION_MAX))

    learned_lateral_acceleration = weighted_sum / total_weight
    confidence = min(total_weight / LEARNING_CONFIDENCE_SAMPLES, 1.0)
    lateral_acceleration = ((1 - confidence) * default_lateral_acceleration + confidence * learned_lateral_acceleration) * self.curve_speed_factor
    return float(np.clip(lateral_acceleration, LATERAL_ACCELERATION_MIN, LATERAL_ACCELERATION_MAX))

  def update_target(self, v_ego):
    lateral_acceleration = self.get_lateral_acceleration(self.frogpilot_planner.road_curvature)
    if self.frogpilot_planner.frogpilot_weather.weather_id != 0:
      lateral_acceleration -= lateral_acceleration * self.frogpilot_planner.frogpilot_weather.reduce_lateral_acceleration

    if self.target_set:
      csc_speed = get_curve_speed(self.frogpilot_planner.road_curvature, lateral_acceleration)
      self.target = get_curve_target(self.target, v_ego, csc_speed, self.frogpilot_planner.time_to_curve)
    else:
      self.target_set = True
      self.target = v_ego
