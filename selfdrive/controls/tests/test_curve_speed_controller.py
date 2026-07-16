from types import SimpleNamespace

import pytest

from openpilot.frogpilot.common.frogpilot_utilities import calculate_road_curvature
from openpilot.frogpilot.controls.lib.curve_speed_controller import (
  CurveSpeedController,
  DEFAULT_LATERAL_ACCELERATION,
  get_curve_speed,
  get_curve_target,
)


def test_curve_selection_uses_sharpest_curve_not_fastest_predicted_lateral_acceleration():
  model_data = SimpleNamespace(
    orientationRate=SimpleNamespace(z=[0.1, 0.2], t=[2.0, 4.0]),
    velocity=SimpleNamespace(x=[20.0, 5.0]),
  )

  curvature, time_to_curve = calculate_road_curvature(model_data)

  assert curvature == pytest.approx(0.04)
  assert time_to_curve == pytest.approx(4.0)


def test_default_curve_shape_is_faster_tight_and_slower_broad():
  controller = CurveSpeedController.__new__(CurveSpeedController)
  controller.lateral_acceleration = DEFAULT_LATERAL_ACCELERATION
  controller.curve_speed_factor = 1.0
  controller.curvature_samples = {}

  broad_lateral_acceleration = controller.get_lateral_acceleration(0.005)
  tight_lateral_acceleration = controller.get_lateral_acceleration(0.05)

  assert broad_lateral_acceleration < DEFAULT_LATERAL_ACCELERATION
  assert tight_lateral_acceleration > DEFAULT_LATERAL_ACCELERATION
  assert get_curve_speed(0.005, broad_lateral_acceleration) < get_curve_speed(0.005, DEFAULT_LATERAL_ACCELERATION)
  assert get_curve_speed(0.05, tight_lateral_acceleration) > get_curve_speed(0.05, DEFAULT_LATERAL_ACCELERATION)


def test_default_curve_shape_crosses_near_mid_forty_kph_target():
  controller = CurveSpeedController.__new__(CurveSpeedController)
  controller.lateral_acceleration = DEFAULT_LATERAL_ACCELERATION
  controller.curve_speed_factor = 1.0
  controller.curvature_samples = {}

  assert controller.get_lateral_acceleration(0.01) < DEFAULT_LATERAL_ACCELERATION
  assert controller.get_lateral_acceleration(0.015) > DEFAULT_LATERAL_ACCELERATION


def test_calibrated_curve_keeps_tight_and_broad_preferences_separate():
  controller = CurveSpeedController.__new__(CurveSpeedController)
  controller.lateral_acceleration = DEFAULT_LATERAL_ACCELERATION
  controller.curve_speed_factor = 1.0
  controller.curvature_samples = {
    "0.005": {"average": 1.3, "count": 400},
    "0.05": {"average": 2.8, "count": 400},
  }

  assert controller.get_lateral_acceleration(0.005) == pytest.approx(1.3, abs=0.02)
  assert controller.get_lateral_acceleration(0.05) == pytest.approx(2.8, abs=0.02)


def test_curve_speed_baseline_scales_without_flattening_calibrated_curve():
  controller = CurveSpeedController.__new__(CurveSpeedController)
  controller.lateral_acceleration = DEFAULT_LATERAL_ACCELERATION
  controller.curve_speed_factor = 1.1
  controller.curvature_samples = {
    "0.005": {"average": 1.3, "count": 400},
    "0.05": {"average": 2.8, "count": 400},
  }

  assert controller.get_lateral_acceleration(0.005) == pytest.approx(1.43, abs=0.02)
  assert controller.get_lateral_acceleration(0.05) == pytest.approx(3.08, abs=0.02)


def test_curve_target_uses_time_to_curve_instead_of_jumping_to_curve_speed():
  current_target = 25.0
  curve_speed = 15.0

  next_target = get_curve_target(current_target, v_ego=25.0, curve_speed=curve_speed, time_to_curve=5.0)

  assert curve_speed < next_target < current_target
  assert next_target == pytest.approx(24.9)


def test_curve_target_never_ramps_below_curve_speed():
  assert get_curve_target(15.1, v_ego=25.0, curve_speed=15.0, time_to_curve=0.01) == pytest.approx(15.0)


def test_curve_speed_has_a_low_speed_floor():
  assert get_curve_speed(0.1, 1.0) == pytest.approx(5.0)
