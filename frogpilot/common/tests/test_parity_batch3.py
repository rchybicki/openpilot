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
