from types import SimpleNamespace

from opendbc.car.interfaces import ACCEL_MIN, ACCEL_MAX
from opendbc.car.lateral import ISO_LATERAL_ACCEL
from openpilot.selfdrive.selfdrived.helpers import ExcessiveActuationCheck, ExcessiveActuationType, MIN_ENGAGE_BUFFER, MIN_EXCESSIVE_ACTUATION_COUNT

V_EGO = 20.0


def run_check(frames, longitudinal_active_with_gas=False, gas_pressed=False, acceleration=0.0, long_active=True, lat_active=False,
              yaw_rate=0.0, curvature=0.0, params_valid=True):
  check = ExcessiveActuationCheck(longitudinal_active_with_gas)
  sm = {
    "carControl": SimpleNamespace(longActive=long_active, latActive=lat_active),
    "liveParameters": SimpleNamespace(roll=0.0, valid=params_valid),
    "controlsState": SimpleNamespace(curvature=curvature),
  }
  car_state = SimpleNamespace(aEgo=acceleration, gasPressed=gas_pressed, steeringPressed=False, vEgo=V_EGO)
  calibrated_pose = SimpleNamespace(
    acceleration=SimpleNamespace(x=acceleration),
    angular_velocity=SimpleNamespace(yaw=yaw_rate),
  )

  result = None
  for _ in range(frames):
    result = check.update(sm, car_state, calibrated_pose)
  return result


ENGAGED_FRAMES = MIN_ENGAGE_BUFFER + MIN_EXCESSIVE_ACTUATION_COUNT + 2


def test_excessive_positive_actuation_ignores_driver_gas_only_when_enabled() -> None:
  acceleration = ACCEL_MAX * 2 + 0.1
  assert run_check(ENGAGED_FRAMES, False, True, acceleration) == ExcessiveActuationType.LONGITUDINAL
  assert run_check(ENGAGED_FRAMES, True, True, acceleration) is None
  assert run_check(ENGAGED_FRAMES, True, False, acceleration) == ExcessiveActuationType.LONGITUDINAL


def test_excessive_negative_actuation_is_never_ignored() -> None:
  acceleration = ACCEL_MIN * 2 - 0.1
  assert run_check(ENGAGED_FRAMES, True, True, acceleration) == ExcessiveActuationType.LONGITUDINAL


def test_longitudinal_needs_one_second_engaged() -> None:
  # upstream #39384: a gas release right at engagement (Kia EV6) is not excessive actuation
  assert run_check(MIN_ENGAGE_BUFFER, acceleration=ACCEL_MIN * 2 - 0.1) is None


LAT_YAW = ISO_LATERAL_ACCEL * 2.5 / V_EGO  # roll-compensated lateral accel above 2x ISO


def test_lateral_fires_when_steering_agrees() -> None:
  curvature = ISO_LATERAL_ACCEL * 2 / V_EGO ** 2
  assert run_check(ENGAGED_FRAMES, long_active=False, lat_active=True, yaw_rate=LAT_YAW, curvature=curvature) == ExcessiveActuationType.LATERAL


def test_lateral_ignores_yaw_the_steering_does_not_explain() -> None:
  # upstream #39384: a loose mount (Hyundai Palisade) gives a yaw rate with straight steering, or the opposite sign
  for curvature in (0.0, -ISO_LATERAL_ACCEL * 2 / V_EGO ** 2):
    assert run_check(ENGAGED_FRAMES, long_active=False, lat_active=True, yaw_rate=LAT_YAW, curvature=curvature) is None


def test_lateral_needs_valid_live_parameters() -> None:
  curvature = ISO_LATERAL_ACCEL * 2 / V_EGO ** 2
  assert run_check(ENGAGED_FRAMES, long_active=False, lat_active=True, yaw_rate=LAT_YAW, curvature=curvature, params_valid=False) is None
