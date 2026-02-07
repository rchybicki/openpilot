import pytest

from openpilot.selfdrive.controls.lib.stopping_guard import apply_should_stop_disturbance_guard


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
