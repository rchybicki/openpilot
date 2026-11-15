import pytest
from opendbc.car.hyundai.interface import CarInterface
from opendbc.car.hyundai.values import CAR

from openpilot.common.params import Params
from openpilot.frogpilot.common.frogpilot_variables import FrogPilotVariables
from openpilot.selfdrive.locationd.lagd import LateralLagEstimator

# the owner's stored values on 2026-11-15 for the keys of parity batch 3 (Santa Fe HEV 2022, openpilot longitudinal)
OWNER_STORED = {
  "AdvancedLateralTune": False, "AdvancedLongitudinalTune": False, "SteerDelay": 0.1, "SteerFriction": 0.103841, "SteerLatAccel": 2.75,
  "SteerKP": 1.0, "SteerRatio": 15.0, "ForceAutoTune": False, "ForceAutoTuneOff": False, "LongitudinalActuatorDelay": 0.5,
  "GreenLightAlert": False, "LeadDepartingAlert": False, "RandomEvents": False, "SpeedLimitController": True, "ShowSpeedLimits": True,
  "TuningLevel": 3,
}


def car_toggles(**stored):
  """The toggles FrogPilotVariables computes for the owner's car with these stored values on top of the owner's."""
  CP = CarInterface.get_non_essential_params(CAR.HYUNDAI_SANTA_FE_HEV_2022)
  CP.openpilotLongitudinalControl = True
  params = Params()
  params.put("CarParamsPersistent", CP.to_bytes())
  params.put_bool("TuningLevelConfirmed", True)
  for key, value in {**OWNER_STORED, **stored}.items():
    params.put(key, value)
  variables = FrogPilotVariables()
  variables.update()
  return CP, variables.frogpilot_toggles


def published_lateral_delay(CP, toggles):
  estimator = LateralLagEstimator(CP, 0.05)
  estimator.frogpilot_toggles = toggles
  return estimator.get_msg(True).liveDelay.lateralDelay


def test_b01_owner_lateral_delay_unchanged():
  CP, toggles = car_toggles()
  assert not toggles.use_custom_steerActuatorDelay
  assert published_lateral_delay(CP, toggles) == pytest.approx(CP.steerActuatorDelay + 0.2)   # lagd's initial lag, not a custom one


def test_b01_custom_steering_delay_is_published_unchanged():
  CP, toggles = car_toggles(AdvancedLateralTune=True, SteerDelay=0.45)
  assert toggles.use_custom_steerActuatorDelay
  assert published_lateral_delay(CP, toggles) == pytest.approx(0.45)


def test_b02_owner_tuning_values_unchanged():
  CP, toggles = car_toggles()
  assert toggles.steerActuatorDelay == pytest.approx(CP.steerActuatorDelay)
  assert toggles.friction == pytest.approx(CP.lateralTuning.torque.friction)
  assert toggles.latAccelFactor == pytest.approx(CP.lateralTuning.torque.latAccelFactor)
  assert toggles.longitudinalActuatorDelay == pytest.approx(CP.longitudinalActuatorDelay)
  assert not toggles.use_custom_steerActuatorDelay and not toggles.use_custom_friction and not toggles.use_custom_latAccelFactor


def test_b02_stored_zero_is_a_value_under_the_old_clamps():
  CP, toggles = car_toggles(AdvancedLateralTune=True, SteerDelay=0.0, SteerFriction=0.0, SteerLatAccel=0.0,
                            AdvancedLongitudinalTune=True, LongitudinalActuatorDelay=0.0)
  assert toggles.steerActuatorDelay == pytest.approx(0.01)
  assert toggles.friction == 0.0
  assert toggles.latAccelFactor == pytest.approx(CP.lateralTuning.torque.latAccelFactor * 0.5)
  assert toggles.longitudinalActuatorDelay == 0.0


def test_b03_owner_custom_torque_flags_unchanged():
  _, toggles = car_toggles()
  assert toggles.has_auto_tune
  assert not toggles.use_custom_friction and not toggles.use_custom_latAccelFactor


def test_b03_custom_torque_values_apply_on_an_auto_tune_car():
  _, toggles = car_toggles(AdvancedLateralTune=True, SteerFriction=0.2, SteerLatAccel=3.0)
  assert toggles.has_auto_tune and not toggles.force_auto_tune_off
  assert toggles.use_custom_friction and toggles.use_custom_latAccelFactor
