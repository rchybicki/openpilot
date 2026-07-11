import pytest

from openpilot.frogpilot.controls.lib.force_coast import get_force_coast_ramped_accel, get_force_coast_target_accel as get_force_coast_min_accel


def test_force_coast_strength_scales_target_deceleration():
  stop_gate = 0.2

  weaker = get_force_coast_min_accel(3.0, stop_gate, 0.5)
  default = get_force_coast_min_accel(3.0, stop_gate, 1.0)
  stronger = get_force_coast_min_accel(3.0, stop_gate, 1.5)

  assert weaker > default > stronger


def test_force_coast_strength_preserves_near_stop_shape():
  stop_gate = 0.2

  near_stop = get_force_coast_min_accel(0.2, stop_gate, 1.0)
  mid_speed = get_force_coast_min_accel(1.0, stop_gate, 1.0)
  high_speed = get_force_coast_min_accel(3.0, stop_gate, 1.0)

  assert near_stop > mid_speed > high_speed


def test_force_coast_ramp_reaches_target_in_one_second():
  assert get_force_coast_ramped_accel(0.2, -1.2, 0.0) == pytest.approx(0.2)
  assert get_force_coast_ramped_accel(0.2, -1.2, 0.5) == pytest.approx(-0.5)
  assert get_force_coast_ramped_accel(0.2, -1.2, 1.0) == pytest.approx(-1.2)
  assert get_force_coast_ramped_accel(0.2, -1.2, 2.0) == pytest.approx(-1.2)
