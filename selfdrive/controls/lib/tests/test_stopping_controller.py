from dataclasses import dataclass

from openpilot.selfdrive.controls.lib.stopping_controller import StoppingController, StoppingPhase
from openpilot.tools.stopping.check_harsh_stops_model import simulate_event_with_controller
from openpilot.tools.stopping.stopping_model import FittedStoppingModel


@dataclass
class FakeSample:
  t: float
  v_ego: float
  a_ego: float
  accel_cmd: float | None
  should_stop: bool = True


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


def _build_20260228_model_for_holdout_replay() -> FittedStoppingModel:
  return FittedStoppingModel(
    delay_frames=0,
    coefficients={
      "intercept": -0.253992889313543,
      "a_ego_prev": 0.8707248413581602,
      "accel_cmd_delayed": 0.2638422343345787,
      "v_ego": 0.29304613067352236,
      "relief": 0.19640459646345515,
      "low_speed": 0.3326724195052019,
      "cmd_x_low_speed": -0.05456194852856041,
    },
    rmse=0.03986682356619745,
    mae=0.02857283946972372,
    r2=0.9680007360030826,
    sample_count=855,
    dt_s=0.1000026359979529,
    relief_cmd_threshold=-0.25,
    low_speed_ref=1.2,
  )


def _simulate_20260228_holdout_seed(samples: list[FakeSample], start_idx: int, hold_idx: int) -> dict[str, float | list[float] | None]:
  model = _build_20260228_model_for_holdout_replay()
  return simulate_event_with_controller(
    samples=samples,
    start_idx=start_idx,
    hold_idx=hold_idx,
    model=model,
    stopping_speed_breakpoint=0.4,
    stop_accel=-2.0,
    controller_should_stop_source="recorded",
  )


def _build_regression_seed_samples_71c_event14() -> list[FakeSample]:
  # Seeded from route 0000071c--fb4cca0034 event 14 (controller replay: stopping_state -> last_stopping_state).
  return [
    FakeSample(t=0.000000000, v_ego=0.145192429, a_ego=-0.480262429, accel_cmd=0.000000000, should_stop=False),
    FakeSample(t=0.100135481, v_ego=0.099171445, a_ego=-0.455735594, accel_cmd=0.000000000, should_stop=False),
    FakeSample(t=0.200578667, v_ego=0.073696688, a_ego=-0.317390710, accel_cmd=0.000000000, should_stop=False),
    FakeSample(t=0.299972905, v_ego=0.062364805, a_ego=-0.184303403, accel_cmd=0.000000000, should_stop=False),
    FakeSample(t=0.399909898, v_ego=0.055598494, a_ego=-0.110088125, accel_cmd=0.000000000, should_stop=False),
    FakeSample(t=0.500773549, v_ego=0.048564520, a_ego=-0.082931839, accel_cmd=0.000000000, should_stop=False),
    FakeSample(t=0.600475544, v_ego=0.045693733, a_ego=-0.049658697, accel_cmd=-0.097595550, should_stop=False),
  ]


def _build_regression_seed_samples_71c_event15() -> list[FakeSample]:
  # Seeded from route 0000071c--fb4cca0034 event 15 (controller replay: stopping_state -> last_stopping_state).
  return [
    FakeSample(t=0.000000000, v_ego=0.143859372, a_ego=-0.567591488, accel_cmd=0.000000000, should_stop=False),
    FakeSample(t=0.100081783, v_ego=0.115193650, a_ego=-0.371072471, accel_cmd=0.000000000, should_stop=False),
    FakeSample(t=0.200227941, v_ego=0.093056388, a_ego=-0.277767003, accel_cmd=0.000000000, should_stop=False),
    FakeSample(t=0.299908790, v_ego=0.073021226, a_ego=-0.226987854, accel_cmd=0.000000000, should_stop=False),
    FakeSample(t=0.398311682, v_ego=0.060816433, a_ego=-0.156830788, accel_cmd=0.000000000, should_stop=False),
    FakeSample(t=0.500241574, v_ego=0.053067580, a_ego=-0.101316400, accel_cmd=-0.019456718, should_stop=False),
    FakeSample(t=0.599118160, v_ego=0.048325647, a_ego=-0.062349379, accel_cmd=-0.100000001, should_stop=False),
    FakeSample(t=0.699042392, v_ego=0.042290162, a_ego=-0.063477121, accel_cmd=-0.100000001, should_stop=False),
  ]


def _build_regression_seed_samples_71c_event19() -> list[FakeSample]:
  # Seeded from route 0000071c--fb4cca0034 event 19 (controller replay: stopping_state -> last_stopping_state).
  return [
    FakeSample(t=0.000000000, v_ego=1.950278521, a_ego=-1.707341313, accel_cmd=-1.298798203, should_stop=False),
    FakeSample(t=0.100469540, v_ego=1.763089061, a_ego=-1.810387373, accel_cmd=-1.136835337, should_stop=False),
    FakeSample(t=0.200701791, v_ego=1.596643090, a_ego=-1.707637072, accel_cmd=-1.006954432, should_stop=False),
    FakeSample(t=0.299749365, v_ego=1.460950613, a_ego=-1.457232594, accel_cmd=-0.899742305, should_stop=False),
    FakeSample(t=0.399939324, v_ego=1.357057691, a_ego=-1.181920409, accel_cmd=-0.808082163, should_stop=False),
    FakeSample(t=0.501044327, v_ego=1.277657866, a_ego=-0.936747730, accel_cmd=-0.767186880, should_stop=True),
    FakeSample(t=0.599420762, v_ego=1.199124217, a_ego=-0.848762989, accel_cmd=-0.767186880, should_stop=True),
    FakeSample(t=0.700146862, v_ego=1.123829246, a_ego=-0.785991609, accel_cmd=-0.767186880, should_stop=True),
    FakeSample(t=0.801071762, v_ego=1.050020218, a_ego=-0.760630608, accel_cmd=-0.767186880, should_stop=True),
    FakeSample(t=0.900498291, v_ego=0.966204226, a_ego=-0.816253662, accel_cmd=-0.767186880, should_stop=True),
    FakeSample(t=1.000092214, v_ego=0.888874352, a_ego=-0.787938714, accel_cmd=-0.767186880, should_stop=True),
    FakeSample(t=1.101141645, v_ego=0.810444832, a_ego=-0.791749120, accel_cmd=-0.735343456, should_stop=True),
    FakeSample(t=1.200659319, v_ego=0.733558297, a_ego=-0.777034998, accel_cmd=-0.579783440, should_stop=True),
    FakeSample(t=1.299893766, v_ego=0.655196548, a_ego=-0.779718101, accel_cmd=-0.535524547, should_stop=True),
    FakeSample(t=1.401156788, v_ego=0.594471335, a_ego=-0.667412162, accel_cmd=-0.520203590, should_stop=True),
    FakeSample(t=1.500079207, v_ego=0.536299586, a_ego=-0.613014638, accel_cmd=-0.520203590, should_stop=True),
    FakeSample(t=1.600731558, v_ego=0.483337045, a_ego=-0.563229620, accel_cmd=-0.520203590, should_stop=True),
    FakeSample(t=1.701268181, v_ego=0.427170098, a_ego=-0.563264191, accel_cmd=-0.520203590, should_stop=True),
    FakeSample(t=1.799582741, v_ego=0.372866124, a_ego=-0.545366764, accel_cmd=-0.519969583, should_stop=True),
    FakeSample(t=1.899274267, v_ego=0.319620907, a_ego=-0.536725461, accel_cmd=-0.519061685, should_stop=True),
    FakeSample(t=2.001531291, v_ego=0.271066517, a_ego=-0.518130779, accel_cmd=-0.517975986, should_stop=True),
    FakeSample(t=2.100622146, v_ego=0.221925080, a_ego=-0.521571219, accel_cmd=-0.516499519, should_stop=True),
    FakeSample(t=2.198704208, v_ego=0.184645012, a_ego=-0.427489012, accel_cmd=-0.407109290, should_stop=True),
    FakeSample(t=2.300876597, v_ego=0.134508610, a_ego=-0.464466721, accel_cmd=-0.287008464, should_stop=True),
    FakeSample(t=2.400454478, v_ego=0.113339633, a_ego=-0.293460011, accel_cmd=-0.263307571, should_stop=True),
    FakeSample(t=2.499475282, v_ego=0.095889792, a_ego=-0.220270500, accel_cmd=-0.258771390, should_stop=True),
    FakeSample(t=2.600796741, v_ego=0.078135438, a_ego=-0.187114209, accel_cmd=-0.254999995, should_stop=True),
    FakeSample(t=2.700077855, v_ego=0.072472245, a_ego=-0.102088101, accel_cmd=-0.254999995, should_stop=True),
    FakeSample(t=2.799897661, v_ego=0.070264116, a_ego=-0.053798035, accel_cmd=-0.314819843, should_stop=True),
    FakeSample(t=2.901767553, v_ego=0.060129650, a_ego=-0.085584521, accel_cmd=-0.319867402, should_stop=True),
  ]


def _build_regression_seed_samples_721_event4() -> list[FakeSample]:
  # Seeded from route 00000721--2b37d8d4a9 event 4 (controller replay: stopping_state -> last_stopping_state).
  return [
    FakeSample(t=0.000000000, v_ego=1.235621214, a_ego=-0.530675113, accel_cmd=-0.555719316, should_stop=False),
    FakeSample(t=0.099714845, v_ego=1.183831930, a_ego=-0.524957478, accel_cmd=-0.542232335, should_stop=False),
    FakeSample(t=0.199100215, v_ego=1.135973692, a_ego=-0.493288368, accel_cmd=-0.529286087, should_stop=False),
    FakeSample(t=0.300399096, v_ego=1.089928985, a_ego=-0.477086425, accel_cmd=-0.522819698, should_stop=False),
    FakeSample(t=0.398843908, v_ego=1.042990327, a_ego=-0.473472536, accel_cmd=-0.522869527, should_stop=False),
    FakeSample(t=0.499206242, v_ego=0.997534871, a_ego=-0.460935026, accel_cmd=-0.521466672, should_stop=True),
    FakeSample(t=0.599363840, v_ego=0.951765239, a_ego=-0.460146070, accel_cmd=-0.521466672, should_stop=True),
    FakeSample(t=0.700086324, v_ego=0.903934777, a_ego=-0.472543865, accel_cmd=-0.521466672, should_stop=True),
    FakeSample(t=0.798791965, v_ego=0.858401299, a_ego=-0.461935610, accel_cmd=-0.521466672, should_stop=True),
    FakeSample(t=0.900139492, v_ego=0.812758863, a_ego=-0.463700324, accel_cmd=-0.521466672, should_stop=True),
    FakeSample(t=0.999079765, v_ego=0.764690399, a_ego=-0.473284155, accel_cmd=-0.521466672, should_stop=True),
    FakeSample(t=1.099277206, v_ego=0.714496791, a_ego=-0.489017397, accel_cmd=-0.521466672, should_stop=True),
    FakeSample(t=1.199206057, v_ego=0.664312601, a_ego=-0.506144524, accel_cmd=-0.521466672, should_stop=True),
    FakeSample(t=1.299196938, v_ego=0.613953650, a_ego=-0.502093792, accel_cmd=-0.521466672, should_stop=True),
    FakeSample(t=1.398339031, v_ego=0.556666493, a_ego=-0.550569415, accel_cmd=-0.521466672, should_stop=True),
    FakeSample(t=1.499975251, v_ego=0.501445174, a_ego=-0.547923744, accel_cmd=-0.521466672, should_stop=True),
    FakeSample(t=1.598717975, v_ego=0.451876104, a_ego=-0.516575396, accel_cmd=-0.521466672, should_stop=True),
    FakeSample(t=1.698654639, v_ego=0.398472369, a_ego=-0.534453332, accel_cmd=-0.521466672, should_stop=True),
    FakeSample(t=1.800043102, v_ego=0.346907854, a_ego=-0.525960922, accel_cmd=-0.521383762, should_stop=True),
    FakeSample(t=1.899943725, v_ego=0.299348772, a_ego=-0.498036742, accel_cmd=-0.520783603, should_stop=True),
    FakeSample(t=1.998564315, v_ego=0.250725865, a_ego=-0.492574543, accel_cmd=-0.519876361, should_stop=True),
    FakeSample(t=2.100311731, v_ego=0.206416100, a_ego=-0.466788203, accel_cmd=-0.518469870, should_stop=True),
    FakeSample(t=2.199803298, v_ego=0.169414371, a_ego=-0.403295517, accel_cmd=-0.455607384, should_stop=True),
    FakeSample(t=2.299185283, v_ego=0.125532642, a_ego=-0.423131227, accel_cmd=-0.306111902, should_stop=True),
    FakeSample(t=2.400454998, v_ego=0.101517759, a_ego=-0.302089244, accel_cmd=-0.266869694, should_stop=True),
    FakeSample(t=2.499091942, v_ego=0.079871409, a_ego=-0.247792944, accel_cmd=-0.254999995, should_stop=True),
    FakeSample(t=2.599364017, v_ego=0.065384440, a_ego=-0.177497581, accel_cmd=-0.254999995, should_stop=True),
    FakeSample(t=2.700431861, v_ego=0.059103657, a_ego=-0.102388658, accel_cmd=-0.254999995, should_stop=True),
    FakeSample(t=2.799008390, v_ego=0.055978604, a_ego=-0.054795101, accel_cmd=-0.254999995, should_stop=True),
    FakeSample(t=2.898765056, v_ego=0.048441369, a_ego=-0.068337142, accel_cmd=-0.330511928, should_stop=True),
    FakeSample(t=3.000229716, v_ego=0.042945202, a_ego=-0.060058888, accel_cmd=-0.335996509, should_stop=True),
  ]


def test_stopping_controller_regression_seed_71c_event14_reduces_end_stop_jerk() -> None:
  result = _simulate_20260228_holdout_seed(_build_regression_seed_samples_71c_event14(), start_idx=5, hold_idx=6)
  assert result["pred_end_stop_jerk_mps3"] is not None
  assert result["pred_end_stop_jerk_mps3"] <= 0.95


def test_stopping_controller_regression_seed_71c_event15_reduces_end_stop_jerk_and_step() -> None:
  result = _simulate_20260228_holdout_seed(_build_regression_seed_samples_71c_event15(), start_idx=5, hold_idx=7)
  assert result["pred_end_stop_jerk_mps3"] is not None
  assert result["pred_end_stop_jerk_mps3"] <= 0.95
  assert result["pred_end_stop_accel_step_mps2"] is not None
  assert result["pred_end_stop_accel_step_mps2"] <= 0.08


def test_stopping_controller_regression_seed_71c_event19_reduces_end_stop_accel_step() -> None:
  result = _simulate_20260228_holdout_seed(_build_regression_seed_samples_71c_event19(), start_idx=5, hold_idx=29)
  assert result["pred_end_stop_accel_step_mps2"] is not None
  assert result["pred_end_stop_accel_step_mps2"] <= 0.08


def test_stopping_controller_regression_seed_721_event4_reduces_end_stop_accel_step() -> None:
  result = _simulate_20260228_holdout_seed(_build_regression_seed_samples_721_event4(), start_idx=5, hold_idx=30)
  assert result["pred_end_stop_accel_step_mps2"] is not None
  assert result["pred_end_stop_accel_step_mps2"] <= 0.08


def _build_20260302_model_for_holdout_replay() -> FittedStoppingModel:
  return FittedStoppingModel(
    delay_frames=0,
    coefficients={
      "intercept": -0.20482419699316673,
      "a_ego_prev": 0.8626149044685716,
      "accel_cmd_delayed": 0.23804013346480102,
      "v_ego": 0.24195134871355753,
      "relief": 0.15926397088416644,
      "low_speed": 0.3007732335278943,
      "cmd_x_low_speed": 0.02268693223791858,
    },
    rmse=0.03974499391846442,
    mae=0.028530463993002183,
    r2=0.9685625039187713,
    sample_count=1003,
    dt_s=0.1000020400019821,
    relief_cmd_threshold=-0.25,
    low_speed_ref=1.2,
  )


def _simulate_20260302_holdout_seed(samples: list[FakeSample], start_idx: int, hold_idx: int) -> dict[str, float | list[float] | None]:
  model = _build_20260302_model_for_holdout_replay()
  return simulate_event_with_controller(
    samples=samples,
    start_idx=start_idx,
    hold_idx=hold_idx,
    model=model,
    stopping_speed_breakpoint=0.4,
    stop_accel=-2.0,
    controller_should_stop_source="recorded",
  )


def _build_regression_seed_samples_71c_event2() -> list[FakeSample]:
  # Seeded from route 0000071c--fb4cca0034 event 2 (controller replay: stopping_state -> last_stopping_state).
  return [
    FakeSample(t=0.000000000, v_ego=0.784829557, a_ego=-0.300528258, accel_cmd=-0.419662148, should_stop=False),
    FakeSample(t=0.099421778, v_ego=0.755568802, a_ego=-0.293384612, accel_cmd=-0.407163411, should_stop=False),
    FakeSample(t=0.199871410, v_ego=0.740811944, a_ego=-0.193549901, accel_cmd=-0.394391775, should_stop=False),
    FakeSample(t=0.299521726, v_ego=0.731671453, a_ego=-0.132301435, accel_cmd=-0.381100953, should_stop=False),
    FakeSample(t=0.399597664, v_ego=0.723472178, a_ego=-0.097458981, accel_cmd=-0.367715061, should_stop=False),
    FakeSample(t=0.499750528, v_ego=0.720975935, a_ego=-0.048895724, accel_cmd=-0.369047791, should_stop=False),
    FakeSample(t=0.600071203, v_ego=0.715618789, a_ego=-0.057813413, accel_cmd=-0.370818585, should_stop=True),
    FakeSample(t=0.700044381, v_ego=0.709748328, a_ego=-0.055558875, accel_cmd=-0.372517794, should_stop=True),
    FakeSample(t=0.800595575, v_ego=0.705446243, a_ego=-0.052377995, accel_cmd=-0.374266744, should_stop=True),
    FakeSample(t=0.899975061, v_ego=0.694480717, a_ego=-0.093083411, accel_cmd=-0.375968009, should_stop=True),
    FakeSample(t=1.000146050, v_ego=0.681506038, a_ego=-0.115166686, accel_cmd=-0.377381086, should_stop=True),
    FakeSample(t=1.100773336, v_ego=0.672038257, a_ego=-0.101815224, accel_cmd=-0.427187353, should_stop=True),
    FakeSample(t=1.200167770, v_ego=0.657218933, a_ego=-0.134072050, accel_cmd=-0.511181116, should_stop=True),
    FakeSample(t=1.300701099, v_ego=0.634683073, a_ego=-0.199412122, accel_cmd=-0.594658911, should_stop=True),
    FakeSample(t=1.400168814, v_ego=0.605225265, a_ego=-0.262456536, accel_cmd=-0.677341163, should_stop=True),
    FakeSample(t=1.501204742, v_ego=0.565025449, a_ego=-0.352642268, accel_cmd=-0.758976936, should_stop=True),
    FakeSample(t=1.599829705, v_ego=0.520543337, a_ego=-0.415924728, accel_cmd=-0.783206701, should_stop=True),
    FakeSample(t=1.699828872, v_ego=0.463843226, a_ego=-0.513532281, accel_cmd=-0.783206701, should_stop=True),
    FakeSample(t=1.800914435, v_ego=0.400093853, a_ego=-0.588400960, accel_cmd=-0.769700110, should_stop=True),
    FakeSample(t=1.900260589, v_ego=0.331509173, a_ego=-0.652881086, accel_cmd=-0.684400201, should_stop=True),
    FakeSample(t=1.999404921, v_ego=0.266150385, a_ego=-0.652163923, accel_cmd=-0.586427510, should_stop=True),
    FakeSample(t=2.100092988, v_ego=0.202109933, a_ego=-0.651271820, accel_cmd=-0.427551717, should_stop=True),
    FakeSample(t=2.199688566, v_ego=0.162661925, a_ego=-0.477582484, accel_cmd=-0.279214501, should_stop=True),
    FakeSample(t=2.300709859, v_ego=0.122712113, a_ego=-0.430180639, accel_cmd=-0.249695733, should_stop=True),
    FakeSample(t=2.400323562, v_ego=0.086582892, a_ego=-0.380344450, accel_cmd=-0.254999995, should_stop=True),
    FakeSample(t=2.499936587, v_ego=0.069540530, a_ego=-0.242060229, accel_cmd=-0.254999995, should_stop=True),
    FakeSample(t=2.599404614, v_ego=0.059791360, a_ego=-0.149460718, accel_cmd=-0.254999995, should_stop=True),
    FakeSample(t=2.700223461, v_ego=0.051809248, a_ego=-0.105145313, accel_cmd=-0.254999995, should_stop=True),
    FakeSample(t=2.799296857, v_ego=0.047344718, a_ego=-0.062766537, accel_cmd=-0.254999995, should_stop=True),
    FakeSample(t=2.900214349, v_ego=0.043110732, a_ego=-0.049691800, accel_cmd=-0.368265837, should_stop=True),
    FakeSample(t=3.000358306, v_ego=0.041525561, a_ego=-0.025518790, accel_cmd=-0.672250092, should_stop=True),
  ]


def test_stopping_controller_regression_seed_20260302_event2_targets_accel_step() -> None:
  result = _simulate_20260302_holdout_seed(_build_regression_seed_samples_71c_event2(), start_idx=5, hold_idx=30)
  assert result["pred_end_stop_accel_step_mps2"] is not None
  assert result["pred_end_stop_accel_step_mps2"] <= 0.10


def test_stopping_controller_regression_seed_20260302_event14_targets_end_stop_jerk() -> None:
  result = _simulate_20260302_holdout_seed(_build_regression_seed_samples_71c_event14(), start_idx=5, hold_idx=6)
  assert result["pred_end_stop_jerk_mps3"] is not None
  assert result["pred_end_stop_jerk_mps3"] <= 1.00


def test_stopping_controller_regression_seed_20260302_event15_targets_jerk_and_step() -> None:
  result = _simulate_20260302_holdout_seed(_build_regression_seed_samples_71c_event15(), start_idx=5, hold_idx=7)
  assert result["pred_end_stop_jerk_mps3"] is not None
  assert result["pred_end_stop_jerk_mps3"] <= 1.00
  assert result["pred_end_stop_accel_step_mps2"] is not None
  assert result["pred_end_stop_accel_step_mps2"] <= 0.13


def test_stopping_controller_regression_seed_20260302_event19_targets_accel_step() -> None:
  result = _simulate_20260302_holdout_seed(_build_regression_seed_samples_71c_event19(), start_idx=5, hold_idx=29)
  assert result["pred_end_stop_accel_step_mps2"] is not None
  assert result["pred_end_stop_accel_step_mps2"] <= 0.08
