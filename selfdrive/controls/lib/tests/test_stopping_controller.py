from openpilot.selfdrive.controls.lib.stopping_controller import StoppingController, StoppingPhase


def test_stopping_controller_passes_through_when_should_stop_false():
  controller = StoppingController()
  _ = controller.update(
    output_accel=-0.18,
    last_output_accel=-0.20,
    should_stop=True,
    v_ego=0.3,
    a_ego=0.2,
    max_expected_accel=-0.1,
    min_expected_accel=-0.5,
    stop_accel=-2.0,
    dt=0.01,
  )
  assert controller.release_lock_counter > 0

  result = controller.update(
    output_accel=-0.12,
    last_output_accel=-0.12,
    should_stop=False,
    v_ego=0.3,
    a_ego=-0.1,
    max_expected_accel=-0.1,
    min_expected_accel=-0.5,
    stop_accel=-2.0,
    dt=0.01,
  )
  assert result.output_accel == -0.12
  assert not result.release_lock_active
  assert controller.release_lock_counter == 0
  assert controller.low_speed_rollout_m == 0.0
  assert controller.phase == StoppingPhase.APPROACH


def test_stopping_controller_near_hold_moves_toward_hold_target():
  controller = StoppingController()
  result = controller.update(
    output_accel=-0.05,
    last_output_accel=-0.05,
    should_stop=True,
    v_ego=0.20,
    a_ego=-0.12,
    max_expected_accel=-0.15,
    min_expected_accel=-0.5,
    stop_accel=-2.0,
    dt=0.01,
  )
  assert controller.phase == StoppingPhase.NEAR_HOLD
  assert result.output_accel < -0.055
  assert result.output_accel > -0.08


def test_stopping_controller_enters_near_hold_at_mid_low_speeds():
  controller = StoppingController()
  result = controller.update(
    output_accel=-0.09,
    last_output_accel=-0.09,
    should_stop=True,
    v_ego=0.70,
    a_ego=-0.08,
    max_expected_accel=-0.12,
    min_expected_accel=-0.50,
    stop_accel=-2.0,
    dt=0.01,
  )
  assert controller.phase == StoppingPhase.NEAR_HOLD
  assert result.output_accel < -0.094


def test_stopping_controller_disturbance_sets_release_lock():
  controller = StoppingController()
  result = controller.update(
    output_accel=-0.18,
    last_output_accel=-0.20,
    should_stop=True,
    v_ego=0.30,
    a_ego=0.20,
    max_expected_accel=-0.10,
    min_expected_accel=-0.50,
    stop_accel=-2.0,
    dt=0.01,
  )
  assert result.release_lock_active
  assert controller.release_lock_counter > 0


def test_stopping_controller_release_lock_tightens_release_step():
  locked_controller = StoppingController()
  _ = locked_controller.update(
    output_accel=-0.22,
    last_output_accel=-0.24,
    should_stop=True,
    v_ego=1.10,
    a_ego=0.20,
    max_expected_accel=-0.10,
    min_expected_accel=-0.50,
    stop_accel=-2.0,
    dt=0.01,
  )
  locked_result = locked_controller.update(
    output_accel=0.05,
    last_output_accel=-0.24,
    should_stop=True,
    v_ego=1.10,
    a_ego=-0.20,
    max_expected_accel=-0.10,
    min_expected_accel=-0.50,
    stop_accel=-2.0,
    dt=0.01,
  )
  assert locked_result.release_lock_active

  unlocked_controller = StoppingController()
  unlocked_result = unlocked_controller.update(
    output_accel=0.05,
    last_output_accel=-0.24,
    should_stop=True,
    v_ego=1.10,
    a_ego=-0.20,
    max_expected_accel=-0.10,
    min_expected_accel=-0.50,
    stop_accel=-2.0,
    dt=0.01,
  )
  assert not unlocked_result.release_lock_active
  assert locked_result.output_accel < unlocked_result.output_accel - 1e-4


def test_stopping_controller_rollout_tightening_strengthens_brake_when_low_speed_rollout_grows():
  controller = StoppingController()
  output = -0.09
  for _ in range(220):
    result = controller.update(
      output_accel=-0.09,
      last_output_accel=output,
      should_stop=True,
      v_ego=0.70,
      a_ego=-0.08,
      max_expected_accel=-0.12,
      min_expected_accel=-0.50,
      stop_accel=-2.0,
      dt=0.01,
    )
    output = result.output_accel

  baseline_controller = StoppingController()
  baseline = baseline_controller.update(
    output_accel=-0.09,
    last_output_accel=-0.09,
    should_stop=True,
    v_ego=0.70,
    a_ego=-0.08,
    max_expected_accel=-0.12,
    min_expected_accel=-0.50,
    stop_accel=-2.0,
    dt=0.01,
  )

  assert controller.low_speed_rollout_m > 1.0
  assert output < baseline.output_accel - 0.04


def test_stopping_controller_delay_release_guard_limits_release_relief():
  guarded = StoppingController()
  for _ in range(8):
    _ = guarded.update(
      output_accel=-0.35,
      last_output_accel=-0.35,
      should_stop=True,
      v_ego=0.95,
      a_ego=-0.20,
      max_expected_accel=-0.10,
      min_expected_accel=-0.45,
      stop_accel=-2.0,
      dt=0.01,
    )

  guarded_result = guarded.update(
    output_accel=0.10,
    last_output_accel=-0.20,
    should_stop=True,
    v_ego=0.95,
    a_ego=-0.18,
    max_expected_accel=-0.10,
    min_expected_accel=-0.45,
    stop_accel=-2.0,
    dt=0.01,
  )

  baseline = StoppingController()
  baseline_result = baseline.update(
    output_accel=0.10,
    last_output_accel=-0.20,
    should_stop=True,
    v_ego=0.95,
    a_ego=-0.18,
    max_expected_accel=-0.10,
    min_expected_accel=-0.45,
    stop_accel=-2.0,
    dt=0.01,
  )

  assert guarded_result.output_accel < baseline_result.output_accel - 0.001


def test_stopping_controller_over_brake_damping_relieves_harsh_decel():
  controller = StoppingController()
  harsh_result = controller.update(
    output_accel=-0.26,
    last_output_accel=-0.26,
    should_stop=True,
    v_ego=0.22,
    a_ego=-1.05,
    max_expected_accel=-0.12,
    min_expected_accel=-0.45,
    stop_accel=-2.0,
    dt=0.01,
  )

  baseline_controller = StoppingController()
  baseline_result = baseline_controller.update(
    output_accel=-0.26,
    last_output_accel=-0.26,
    should_stop=True,
    v_ego=0.22,
    a_ego=-0.35,
    max_expected_accel=-0.12,
    min_expected_accel=-0.45,
    stop_accel=-2.0,
    dt=0.01,
  )

  assert harsh_result.output_accel > baseline_result.output_accel + 0.0002

