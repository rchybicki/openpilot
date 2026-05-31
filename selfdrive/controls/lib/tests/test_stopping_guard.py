import pytest

from openpilot.selfdrive.controls.lib.stopping_guard import apply_low_speed_output_slew


def test_low_speed_output_slew_inactive_above_speed_gate():
  out = apply_low_speed_output_slew(
    output_accel=-0.35,
    last_output_accel=-0.20,
    should_stop=False,
    v_ego=1.25,
    a_ego=-0.2,
    max_expected_accel=-0.1,
    allow_fast_release=False,
    release_lock_active=False,
  )
  assert out == pytest.approx(-0.35)


def test_low_speed_output_slew_limits_brake_step_non_disturbance():
  out = apply_low_speed_output_slew(
    output_accel=-0.50,
    last_output_accel=-0.20,
    should_stop=True,
    v_ego=0.2,
    a_ego=-0.3,
    max_expected_accel=-0.1,
    allow_fast_release=False,
    release_lock_active=False,
  )
  # brake_step=0.009 -> floor at -0.209
  assert out == pytest.approx(-0.209, abs=1e-12)


def test_low_speed_output_slew_limits_release_step_when_not_resuming():
  out = apply_low_speed_output_slew(
    output_accel=-0.05,
    last_output_accel=-0.20,
    should_stop=False,
    v_ego=0.2,
    a_ego=-0.3,
    max_expected_accel=-0.1,
    allow_fast_release=False,
    release_lock_active=False,
  )
  # release_step=0.004 -> cap at -0.196
  assert out == pytest.approx(-0.196, abs=1e-12)


def test_low_speed_output_slew_allows_faster_release_with_resume_intent():
  out = apply_low_speed_output_slew(
    output_accel=-0.05,
    last_output_accel=-0.20,
    should_stop=False,
    v_ego=0.2,
    a_ego=-0.3,
    max_expected_accel=-0.1,
    allow_fast_release=True,
    release_lock_active=False,
  )
  # fast release_step=0.026 -> cap at -0.174
  assert out == pytest.approx(-0.174, abs=1e-12)


def test_low_speed_output_slew_allows_faster_brake_under_disturbance():
  out = apply_low_speed_output_slew(
    output_accel=-0.50,
    last_output_accel=-0.20,
    should_stop=True,
    v_ego=0.2,
    a_ego=0.2,
    max_expected_accel=-0.1,
    allow_fast_release=False,
    release_lock_active=False,
  )
  # disturbance active -> brake_step=0.016 -> floor at -0.216
  assert out == pytest.approx(-0.216, abs=1e-12)


def test_low_speed_output_slew_tightens_release_when_lock_active():
  out = apply_low_speed_output_slew(
    output_accel=-0.05,
    last_output_accel=-0.20,
    should_stop=True,
    v_ego=0.2,
    a_ego=-0.3,
    max_expected_accel=-0.1,
    allow_fast_release=False,
    release_lock_active=True,
  )
  # release lock -> release_step=0.0015 -> cap at -0.1985
  assert out == pytest.approx(-0.1985, abs=1e-12)
