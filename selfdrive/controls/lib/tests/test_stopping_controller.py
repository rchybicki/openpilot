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


def test_stopping_controller_near_hold_target_is_not_overly_deep_for_light_stopping():
  controller = StoppingController()
  output = -0.05
  for _ in range(40):
    result = controller.update(
      output_accel=-0.05,
      last_output_accel=output,
      should_stop=True,
      v_ego=0.20,
      a_ego=-0.10,
      max_expected_accel=-0.10,
      min_expected_accel=-0.50,
      stop_accel=-2.0,
      dt=0.01,
    )
    output = result.output_accel
  assert controller.phase == StoppingPhase.NEAR_HOLD
  assert output > -0.21


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


def test_stopping_controller_low_speed_disturbance_sets_release_lock():
  controller = StoppingController()
  result = controller.update(
    output_accel=-0.10,
    last_output_accel=-0.275,
    should_stop=True,
    v_ego=0.045,
    a_ego=0.03,
    max_expected_accel=-0.02,
    min_expected_accel=-0.50,
    stop_accel=-2.0,
    dt=0.01,
  )
  assert result.release_lock_active
  assert controller.release_lock_counter > 0


def test_stopping_controller_regression_seed_20260212_leapfrog_onset_sets_release_lock():
  # Seeded from engaged stop-event onset samples before rebound on:
  # - route_000006f1--1eeed096b0 event 3
  # - route_000006f2--ef82b286ad event 3
  for v_ego, a_ego, max_expected_accel in (
    (0.041427, 0.016056, -0.0249),
    (0.041805, 0.022024, -0.0251),
  ):
    controller = StoppingController()
    result = controller.update(
      output_accel=-0.10,
      last_output_accel=-0.275,
      should_stop=True,
      v_ego=v_ego,
      a_ego=a_ego,
      max_expected_accel=max_expected_accel,
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


def test_stopping_controller_low_speed_disturbance_applies_extra_brake():
  disturbed_controller = StoppingController()
  disturbed_result = disturbed_controller.update(
    output_accel=-0.10,
    last_output_accel=-0.275,
    should_stop=True,
    v_ego=0.045,
    a_ego=0.03,
    max_expected_accel=-0.02,
    min_expected_accel=-0.50,
    stop_accel=-2.0,
    dt=0.01,
  )

  nominal_controller = StoppingController()
  nominal_result = nominal_controller.update(
    output_accel=-0.10,
    last_output_accel=-0.275,
    should_stop=True,
    v_ego=0.045,
    a_ego=-0.02,
    max_expected_accel=-0.02,
    min_expected_accel=-0.50,
    stop_accel=-2.0,
    dt=0.01,
  )

  assert disturbed_result.output_accel < nominal_result.output_accel - 1e-4


def test_stopping_controller_low_speed_rebound_cap_brakes_more_when_decel_weakens():
  weak_decel_controller = StoppingController()
  for _ in range(5):
    _ = weak_decel_controller.update(
      output_accel=-0.275,
      last_output_accel=-0.275,
      should_stop=True,
      v_ego=0.06,
      a_ego=-0.30,
      max_expected_accel=-0.02,
      min_expected_accel=-0.50,
      stop_accel=-2.0,
      dt=0.01,
    )
  weak_decel_result = weak_decel_controller.update(
    output_accel=-0.275,
    last_output_accel=-0.275,
    should_stop=True,
    v_ego=0.03,
    a_ego=-0.16,
    max_expected_accel=-0.015,
    min_expected_accel=-0.50,
    stop_accel=-2.0,
    dt=0.01,
  )

  nominal_controller = StoppingController()
  for _ in range(5):
    _ = nominal_controller.update(
      output_accel=-0.275,
      last_output_accel=-0.275,
      should_stop=True,
      v_ego=0.06,
      a_ego=-0.30,
      max_expected_accel=-0.02,
      min_expected_accel=-0.50,
      stop_accel=-2.0,
      dt=0.01,
    )
  nominal_result = nominal_controller.update(
    output_accel=-0.275,
    last_output_accel=-0.275,
    should_stop=True,
    v_ego=0.03,
    a_ego=-0.40,
    max_expected_accel=-0.015,
    min_expected_accel=-0.50,
    stop_accel=-2.0,
    dt=0.01,
  )

  assert weak_decel_result.output_accel < nominal_result.output_accel - 1e-4


def test_stopping_controller_low_speed_rebound_cap_active_extends_up_to_point_one_mps():
  controller = StoppingController()
  debug: dict[str, object] = {}
  result = controller.update(
    output_accel=-0.10,
    last_output_accel=-0.10,
    should_stop=True,
    v_ego=0.055,
    a_ego=-0.10,
    max_expected_accel=-0.02,
    min_expected_accel=-0.50,
    stop_accel=-2.0,
    dt=0.01,
    debug=debug,
  )

  triggers = debug.get("triggers", ())
  assert "low_speed_rebound_cap_active" in triggers
  assert result.output_accel < -0.10


def test_stopping_controller_rebound_arrest_arms_only_below_low_speed_gate():
  controller = StoppingController()
  for _ in range(520):
    _ = controller.update(
      output_accel=-0.275,
      last_output_accel=-0.275,
      should_stop=True,
      v_ego=0.07,
      a_ego=-0.30,
      max_expected_accel=-0.02,
      min_expected_accel=-0.50,
      stop_accel=-2.0,
      dt=0.01,
    )

  above_gate = controller.update(
    output_accel=-0.275,
    last_output_accel=-0.275,
    should_stop=True,
    v_ego=0.055,
    a_ego=-0.18,
    max_expected_accel=-0.02,
    min_expected_accel=-0.50,
    stop_accel=-2.0,
      dt=0.01,
    )
  assert controller.rebound_arrest_counter == 0


def test_stopping_controller_low_rollout_soft_landing_cap_triggers():
  controller = StoppingController()
  last_output = -1.20
  debug: dict[str, object] = {}
  result = controller.update(
    output_accel=last_output,
    last_output_accel=last_output,
    should_stop=True,
    v_ego=0.10,
    a_ego=-0.30,
    max_expected_accel=-0.10,
    min_expected_accel=-0.50,
    stop_accel=-2.0,
    dt=0.01,
    debug=debug,
  )
  triggers = debug.get("triggers", ())
  assert "low_rollout_soft_landing_cap" in triggers
  assert result.output_accel > last_output


def test_stopping_controller_low_rollout_soft_landing_release_step_not_too_aggressive():
  controller = StoppingController()
  last_output = -1.20
  debug: dict[str, object] = {}
  result = controller.update(
    output_accel=last_output,
    last_output_accel=last_output,
    should_stop=True,
    v_ego=0.10,
    a_ego=-0.30,
    max_expected_accel=-0.10,
    min_expected_accel=-0.50,
    stop_accel=-2.0,
    dt=0.01,
    debug=debug,
  )
  triggers = debug.get("triggers", ())
  assert "low_rollout_soft_landing_cap" in triggers
  assert (result.output_accel - last_output) <= 0.020


def test_stopping_controller_end_stop_cap_release_step_not_too_aggressive():
  controller = StoppingController()
  last_output = -1.20
  debug: dict[str, object] = {}
  result = controller.update(
    output_accel=last_output,
    last_output_accel=last_output,
    should_stop=True,
    v_ego=0.35,
    a_ego=-0.30,
    max_expected_accel=-0.10,
    min_expected_accel=-0.50,
    stop_accel=-2.0,
    dt=0.01,
    debug=debug,
  )
  triggers = debug.get("triggers", ())
  assert "end_stop_cap_active" in triggers
  assert (result.output_accel - last_output) <= 0.013


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


def test_stopping_controller_rollout_oscillation_damping_holds_firmer_brake_when_rollout_is_high():
  high_rollout = StoppingController()
  high_rollout.low_speed_rollout_m = 2.4
  high_rollout.release_lock_counter = 8
  high_rollout.phase = StoppingPhase.NEAR_HOLD
  high_result = high_rollout.update(
    output_accel=0.05,
    last_output_accel=-0.60,
    should_stop=True,
    v_ego=0.65,
    a_ego=0.02,
    max_expected_accel=-0.12,
    min_expected_accel=-0.50,
    stop_accel=-2.0,
    dt=0.01,
  )

  low_rollout = StoppingController()
  low_rollout.low_speed_rollout_m = 0.4
  low_rollout.release_lock_counter = 8
  low_rollout.phase = StoppingPhase.NEAR_HOLD
  low_result = low_rollout.update(
    output_accel=0.05,
    last_output_accel=-0.60,
    should_stop=True,
    v_ego=0.65,
    a_ego=0.02,
    max_expected_accel=-0.12,
    min_expected_accel=-0.50,
    stop_accel=-2.0,
    dt=0.01,
  )

  assert high_result.output_accel < low_result.output_accel - 0.004


def test_stopping_controller_severe_rebound_guard_adds_brake_when_rollout_is_large_and_decel_collapses():
  high_rollout = StoppingController()
  high_rollout.low_speed_rollout_m = 0.90
  high_result = high_rollout.update(
    output_accel=-0.30,
    last_output_accel=-0.30,
    should_stop=True,
    v_ego=0.42,
    a_ego=0.04,
    max_expected_accel=-0.12,
    min_expected_accel=-0.50,
    stop_accel=-2.0,
    dt=0.01,
  )

  low_rollout = StoppingController()
  low_rollout.low_speed_rollout_m = 0.20
  low_result = low_rollout.update(
    output_accel=-0.30,
    last_output_accel=-0.30,
    should_stop=True,
    v_ego=0.42,
    a_ego=0.04,
    max_expected_accel=-0.12,
    min_expected_accel=-0.50,
    stop_accel=-2.0,
    dt=0.01,
  )

  assert high_rollout.low_speed_rollout_m > 0.80
  assert high_result.output_accel < low_result.output_accel - 0.010


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


def test_stopping_controller_ineffective_brake_guard_prevents_deep_windup_near_hold():
  controller = StoppingController()
  output = -0.80

  # Simulate a stop where decel is strong initially but becomes ineffective near hold (e.g. drivetrain/clutch push).
  # Without a guard, the controller can ratchet toward stop_accel aggressively.
  for idx in range(200):
    v_ego = max(0.06, 0.70 - (0.003 * idx))
    a_ego = -0.75 if v_ego > 0.32 else -0.20
    result = controller.update(
      output_accel=output,
      last_output_accel=output,
      should_stop=True,
      v_ego=v_ego,
      a_ego=a_ego,
      max_expected_accel=-0.10,
      min_expected_accel=-0.50,
      stop_accel=-2.0,
      dt=0.10,
    )
    output = result.output_accel

  assert output > -1.50


def test_stopping_controller_soft_landing_releases_in_hold_when_decel_is_stable():
  controller = StoppingController()
  output = -0.22

  for idx in range(50):
    v_ego = max(0.0, 0.20 * (1.0 - (idx / 49.0)))
    result = controller.update(
      output_accel=output,
      last_output_accel=output,
      should_stop=True,
      v_ego=v_ego,
      a_ego=-0.20,
      max_expected_accel=-0.10,
      min_expected_accel=-0.50,
      stop_accel=-2.0,
      dt=0.01,
    )
    output = result.output_accel

  for _ in range(150):
    result = controller.update(
      output_accel=output,
      last_output_accel=output,
      should_stop=True,
      v_ego=0.0,
      a_ego=0.0,
      max_expected_accel=-0.10,
      min_expected_accel=-0.50,
      stop_accel=-2.0,
      dt=0.01,
    )
    output = result.output_accel

  assert controller.phase == StoppingPhase.HOLD
  assert output > -0.18
