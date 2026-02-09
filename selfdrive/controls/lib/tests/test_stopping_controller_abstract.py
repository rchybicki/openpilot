from openpilot.selfdrive.controls.lib.stopping_controller_abstract import (
  AbstractStoppingControllerV2,
  AbstractStoppingControllerV3,
)
from openpilot.selfdrive.controls.lib.stopping_controller_factory import build_stopping_controller


def test_abstract_stopping_controllers_passthrough_when_should_stop_false() -> None:
  for controller in (AbstractStoppingControllerV2(), AbstractStoppingControllerV3()):
    _ = controller.update(
      output_accel=-0.22,
      last_output_accel=-0.25,
      should_stop=True,
      v_ego=0.50,
      a_ego=0.10,
      max_expected_accel=-0.1,
      min_expected_accel=-0.5,
      stop_accel=-2.0,
      dt=0.01,
    )
    result = controller.update(
      output_accel=-0.09,
      last_output_accel=-0.09,
      should_stop=False,
      v_ego=0.50,
      a_ego=-0.05,
      max_expected_accel=-0.1,
      min_expected_accel=-0.5,
      stop_accel=-2.0,
      dt=0.01,
    )
    assert result.output_accel == -0.09
    assert controller.low_speed_rollout_m == 0.0


def test_abstract_v3_tightens_more_than_v2_after_rollout_growth() -> None:
  v2 = AbstractStoppingControllerV2()
  v3 = AbstractStoppingControllerV3()
  v2_out = -0.10
  v3_out = -0.10
  for _ in range(260):
    v2_out = v2.update(
      output_accel=-0.10,
      last_output_accel=v2_out,
      should_stop=True,
      v_ego=0.65,
      a_ego=-0.06,
      max_expected_accel=-0.12,
      min_expected_accel=-0.50,
      stop_accel=-2.0,
      dt=0.01,
    ).output_accel
    v3_out = v3.update(
      output_accel=-0.10,
      last_output_accel=v3_out,
      should_stop=True,
      v_ego=0.65,
      a_ego=-0.06,
      max_expected_accel=-0.12,
      min_expected_accel=-0.50,
      stop_accel=-2.0,
      dt=0.01,
    ).output_accel

  assert v2.low_speed_rollout_m > 1.0
  assert v3.low_speed_rollout_m > 1.0
  assert v3_out < v2_out - 0.005


def test_controller_factory_returns_abstract_and_legacy_controllers() -> None:
  abstract = build_stopping_controller("abstract_v2")
  legacy = build_stopping_controller("legacy_v2")

  abstract_result = abstract.update(
    output_accel=-0.15,
    last_output_accel=-0.20,
    should_stop=True,
    v_ego=0.4,
    a_ego=-0.1,
    max_expected_accel=-0.1,
    min_expected_accel=-0.5,
    stop_accel=-2.0,
    dt=0.01,
  )
  legacy_result = legacy.update(
    output_accel=-0.15,
    last_output_accel=-0.20,
    should_stop=True,
    v_ego=0.4,
    a_ego=-0.1,
    max_expected_accel=-0.1,
    min_expected_accel=-0.5,
    stop_accel=-2.0,
    dt=0.01,
  )

  assert abstract_result.output_accel <= -0.05
  assert legacy_result.output_accel <= -0.05
