import pytest

from cereal import car
from openpilot.common.params import Params
from openpilot.frogpilot.common.frogpilot_variables import get_frogpilot_toggles
from openpilot.frogpilot.controls.lib import force_coast

# the driver's stored settings (route 00002310 initData) -> the toggle the longitudinal core reads
USER_SETTINGS = {
  "CEExperimentalBoostGain": ("experimental_lead_boost_gain", 1.8),
  "CEExperimentalBoostBrakeCutoff": ("experimental_boost_brake_cutoff", -0.25),
  "CEExperimentalNoLeadBoostGain": ("experimental_no_lead_boost_gain", 1.2),
  "CEForceCoastStrength": ("force_coast_strength", 1.4),
}


@pytest.fixture
def toggles():
  params = Params()
  CP = car.CarParams(brand="hyundai", carFingerprint="HYUNDAI_SANTA_FE_HEV_2022", openpilotLongitudinalControl=True,
                     safetyConfigs=[car.CarParams.SafetyConfig(safetyModel=car.CarParams.SafetyModel.hyundai)])
  params.put("CarParamsPersistent", CP.to_bytes())
  for key, (_, value) in USER_SETTINGS.items():
    params.put(key, value)
  return get_frogpilot_toggles()


@pytest.mark.parametrize("key", USER_SETTINGS)
def test_stored_setting_reaches_the_core_toggle(toggles, key):
  name, value = USER_SETTINGS[key]
  assert getattr(toggles, name) == pytest.approx(value)


def test_force_coast_brakes_at_the_stored_strength(toggles):
  v_ego = 5.0
  stop_gate = max(toggles.vEgoStopping, 0.2)
  target = force_coast.get_force_coast_target_from_toggles(v_ego, toggles)
  assert target == force_coast.get_force_coast_target_accel(v_ego, stop_gate, 1.4)
  assert target < force_coast.get_force_coast_target_accel(v_ego, stop_gate, force_coast.FORCE_COAST_STRENGTH_DEFAULT)
