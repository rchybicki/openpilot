from types import SimpleNamespace

from opendbc.car.interfaces import ACCEL_MIN, ACCEL_MAX
from openpilot.selfdrive.selfdrived.helpers import ExcessiveActuationCheck, ExcessiveActuationType, MIN_EXCESSIVE_ACTUATION_COUNT


def run_excessive_longitudinal_check(longitudinal_active_with_gas, gas_pressed, acceleration):
  check = ExcessiveActuationCheck(longitudinal_active_with_gas)
  sm = {
    "carControl": SimpleNamespace(longActive=True, latActive=False),
    "liveParameters": SimpleNamespace(roll=0.0),
  }
  car_state = SimpleNamespace(aEgo=acceleration, gasPressed=gas_pressed, steeringPressed=False, vEgo=10.0)
  calibrated_pose = SimpleNamespace(
    acceleration=SimpleNamespace(x=acceleration),
    angular_velocity=SimpleNamespace(yaw=0.0),
  )

  result = None
  for _ in range(MIN_EXCESSIVE_ACTUATION_COUNT + 1):
    result = check.update(sm, car_state, calibrated_pose)
  return result


def test_excessive_positive_actuation_ignores_driver_gas_only_when_enabled() -> None:
  acceleration = ACCEL_MAX * 2 + 0.1
  assert run_excessive_longitudinal_check(False, True, acceleration) == ExcessiveActuationType.LONGITUDINAL
  assert run_excessive_longitudinal_check(True, True, acceleration) is None
  assert run_excessive_longitudinal_check(True, False, acceleration) == ExcessiveActuationType.LONGITUDINAL


def test_excessive_negative_actuation_is_never_ignored() -> None:
  acceleration = ACCEL_MIN * 2 - 0.1
  assert run_excessive_longitudinal_check(True, True, acceleration) == ExcessiveActuationType.LONGITUDINAL
