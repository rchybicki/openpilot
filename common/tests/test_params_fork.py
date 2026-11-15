import pytest

from openpilot.common.params import Params, ParamKeyFlag

MANAGER_START_FLAGS = (ParamKeyFlag.CLEAR_ON_MANAGER_START, ParamKeyFlag.CLEAR_ON_ONROAD_TRANSITION,
                       ParamKeyFlag.CLEAR_ON_OFFROAD_TRANSITION, ParamKeyFlag.CLEAR_ON_IGNITION_ON)

# Raw bytes as the !my-fp-new device store held them on 2026-10-25 (initData of route 00002642--79aa2f251a),
# and the value the ported code reads after manager start.
STORED_VALUES = {
  "BuildMetadata": (b'{"channel": "!my-fp-new"}', '{"channel": "!my-fp-new"}'),
  "CECscCurves": (b"0", False),
  "CEExperimentalBoostBrakeCutoff": (b"-0.250000", -0.25),
  "CEExperimentalBoostGain": (b"1.800000", 1.8),
  "CEExperimentalNoLeadBoostGain": (b"1.200000", 1.2),
  "CEForceCoastStrength": (b"1.400000", 1.4),
  "CEModelStopTime": (b"7.000000", 7.0),
  "CENavigation": (b"0", False),
  "CSCBrakingForce": (b"1.200000", 1.2),
  "CSCBrakingForceHighSpeedReduction": (b"30.000000", 30.0),
  "CSCSpeedFactor": (b"1.0", 1.0),
  "CalibratedLateralAccelerationOverride": (b"2.000000", 2.0),
  "ExperimentalGMTune": (b"0", False),
  "Hyundai-RadarTracks": (b"1", True),
  "InitialSetSpeed": (b"160", 160),
  "LaneCenterOffset": (b"0.0", 0.0),
  "LaneCentering": (b"1", True),
  "LaneCenteringE2EAuthority": (b"0.000000", 0.0),
  "LongDistanceFactor": (b"1.800000", 1.8),
  "LongitudinalActiveWithGas": (b"1", True),
  "NewLongAPI": (b"1", True),
  "ShortDistanceFactor": (b"0.000000", 0.0),
  "TailscaledPid": (b"51673", 51673),
  "TrafficFollow": (b"0.5", 0.5),
  "TrafficJerkAcceleration": (b"50.0", 50.0),
  "TrafficJerkDanger": (b"100.0", 100.0),
  "TrafficJerkDeceleration": (b"50.0", 50.0),
  "TrafficJerkSpeed": (b"50.0", 50.0),
  "TrafficJerkSpeedDecrease": (b"50.0", 50.0),
  "TrafficPersonalityProfile": (b"0", False),
  # retyped by the new base: BOOL -> INT (speed, 0 = off) and BOOL -> STRING (profile name, UI only)
  "PauseAOLOnBrake": (b"0", 0),
  "AggressivePersonalityProfile": (b"0", "0"),
}

# runtime state, cleared at every manager start by design
CLEARED_AT_START = {
  "CEStatus": b"3",
  "LiveUpdateHandoffBlockers": b"cruise main on",
  "LiveUpdateHandoffState": b"requested",
}


def _write_raw(params, key, raw):
  with open(params.get_param_path(key), "wb") as f:
    f.write(raw)


def _manager_start(params):
  for flag in MANAGER_START_FLAGS:
    params.clear_all(flag)


@pytest.mark.parametrize("key", sorted(STORED_VALUES))
def test_stored_value_survives_manager_start(key):
  params = Params()
  raw, expected = STORED_VALUES[key]
  _write_raw(params, key, raw)

  _manager_start(params)

  value = params.get(key)
  assert type(value) is type(expected)
  if isinstance(expected, float):
    assert value == pytest.approx(expected)
  else:
    assert value == expected


@pytest.mark.parametrize("key", sorted(CLEARED_AT_START))
def test_runtime_state_cleared_at_manager_start(key):
  params = Params()
  _write_raw(params, key, CLEARED_AT_START[key])

  _manager_start(params)

  assert params.get(key) is None


def test_instant_replay_off_by_default():
  # a new-base feature the old branch never had: 0 shows no capture button and keeps no replay buffer
  params = Params()
  params.remove("InstantReplay")

  assert params.get_default_value("InstantReplay") == 0
  assert params.get("InstantReplay", return_default=True) == 0
