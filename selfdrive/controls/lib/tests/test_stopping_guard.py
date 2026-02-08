import pytest

from openpilot.selfdrive.controls.lib.stopping_guard import (
  apply_should_stop_disturbance_guard,
  apply_should_stop_output_slew,
  apply_should_stop_soft_landing,
)


DT = 0.01
STOPPING_V_BP = [0.01, 0.2, 0.5]


def test_guard_inactive_when_should_stop_false():
  out = apply_should_stop_disturbance_guard(
    output_accel=-0.20,
    last_output_accel=-0.30,
    should_stop=False,
    v_ego=0.2,
    a_ego=0.3,
    max_expected_accel=-0.2,
    stopping_v_bp=STOPPING_V_BP,
    dt=DT,
  )
  assert out == pytest.approx(-0.20)


def test_guard_inactive_above_low_speed_gate():
  out = apply_should_stop_disturbance_guard(
    output_accel=-0.20,
    last_output_accel=-0.30,
    should_stop=True,
    v_ego=0.9,
    a_ego=0.3,
    max_expected_accel=-0.2,
    stopping_v_bp=STOPPING_V_BP,
    dt=DT,
  )
  assert out == pytest.approx(-0.20)


def test_guard_inactive_for_small_disturbance():
  out = apply_should_stop_disturbance_guard(
    output_accel=-0.20,
    last_output_accel=-0.30,
    should_stop=True,
    v_ego=0.2,
    a_ego=-0.17,
    max_expected_accel=-0.18,
    stopping_v_bp=STOPPING_V_BP,
    dt=DT,
  )
  assert out == pytest.approx(-0.20)


def test_guard_blocks_release_and_adds_brake():
  out = apply_should_stop_disturbance_guard(
    output_accel=-0.20,
    last_output_accel=-0.30,
    should_stop=True,
    v_ego=0.2,
    a_ego=0.20,
    max_expected_accel=-0.20,
    stopping_v_bp=STOPPING_V_BP,
    dt=DT,
  )
  # disturbance=0.40, gain=2.0 at v=0.2, delta=0.008
  assert out == pytest.approx(-0.308, abs=1e-6)


def test_guard_clips_large_disturbance():
  out = apply_should_stop_disturbance_guard(
    output_accel=-0.20,
    last_output_accel=-0.30,
    should_stop=True,
    v_ego=0.01,
    a_ego=2.0,
    max_expected_accel=-1.0,
    stopping_v_bp=STOPPING_V_BP,
    dt=DT,
  )
  # disturbance clip=0.8, gain=3.0 at v=0.01, delta=0.024
  assert out == pytest.approx(-0.324, abs=1e-6)


def test_soft_landing_inactive_when_should_stop_false():
  out = apply_should_stop_soft_landing(
    output_accel=-0.20,
    last_output_accel=-0.30,
    should_stop=False,
    v_ego=0.2,
    a_ego=-0.2,
    max_expected_accel=-0.1,
  )
  assert out == pytest.approx(-0.20)


def test_soft_landing_inactive_above_speed_gate():
  out = apply_should_stop_soft_landing(
    output_accel=-0.20,
    last_output_accel=-0.30,
    should_stop=True,
    v_ego=0.6,
    a_ego=-0.2,
    max_expected_accel=-0.1,
  )
  assert out == pytest.approx(-0.20)


def test_soft_landing_skips_when_disturbance_response_needed():
  out = apply_should_stop_soft_landing(
    output_accel=-0.20,
    last_output_accel=-0.30,
    should_stop=True,
    v_ego=0.2,
    a_ego=0.05,
    max_expected_accel=-0.1,
  )
  assert out == pytest.approx(-0.20)


def test_soft_landing_limits_brake_step_near_hold():
  out = apply_should_stop_soft_landing(
    output_accel=-0.35,
    last_output_accel=-0.20,
    should_stop=True,
    v_ego=0.2,
    a_ego=-0.2,
    max_expected_accel=-0.05,
  )
  assert out == pytest.approx(-0.204333333333, abs=1e-12)


def test_soft_landing_limits_release_step_near_hold():
  out = apply_should_stop_soft_landing(
    output_accel=-0.10,
    last_output_accel=-0.25,
    should_stop=True,
    v_ego=0.05,
    a_ego=-0.2,
    max_expected_accel=-0.05,
  )
  assert out == pytest.approx(-0.248, abs=1e-12)


def test_output_slew_inactive_without_should_stop():
  out = apply_should_stop_output_slew(
    output_accel=-0.35,
    last_output_accel=-0.20,
    should_stop=False,
    v_ego=0.3,
    a_ego=-0.2,
    max_expected_accel=-0.1,
  )
  assert out == pytest.approx(-0.35)


def test_output_slew_limits_brake_step_non_disturbance():
  out = apply_should_stop_output_slew(
    output_accel=-0.50,
    last_output_accel=-0.20,
    should_stop=True,
    v_ego=0.2,
    a_ego=-0.3,
    max_expected_accel=-0.1,
  )
  # brake_step=0.009 -> floor at -0.209
  assert out == pytest.approx(-0.209, abs=1e-12)


def test_output_slew_limits_release_step():
  out = apply_should_stop_output_slew(
    output_accel=-0.05,
    last_output_accel=-0.20,
    should_stop=True,
    v_ego=0.2,
    a_ego=-0.3,
    max_expected_accel=-0.1,
  )
  # release_step=0.005 -> cap at -0.195
  assert out == pytest.approx(-0.195, abs=1e-12)


def test_output_slew_allows_faster_brake_under_disturbance():
  out = apply_should_stop_output_slew(
    output_accel=-0.50,
    last_output_accel=-0.20,
    should_stop=True,
    v_ego=0.2,
    a_ego=0.2,
    max_expected_accel=-0.1,
  )
  # disturbance active -> brake_step=0.016 -> floor at -0.216
  assert out == pytest.approx(-0.216, abs=1e-12)
