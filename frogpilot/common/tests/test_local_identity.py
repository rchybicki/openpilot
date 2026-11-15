import re

import pytest

from openpilot.common.params import Params
from openpilot.frogpilot.common import frogpilot_functions
from openpilot.system.hardware.hardwared import is_startup_registered

IDENTITY_KEYS = ("DongleId", "StockDongleId", "KonikDongleId", "FrogPilotDongleId")


@pytest.fixture
def params():
  params = Params()
  for key in IDENTITY_KEYS:
    params.remove(key)
  return params


def test_keyless_device_gets_a_local_identity(params):
  # no registration keys and no network: nothing stores an identity, and hardwared would keep the device off-road
  assert not is_startup_registered(params)

  frogpilot_functions.create_local_identity(params)

  assert re.fullmatch(r"[a-z0-9]{16}", params.get("FrogPilotDongleId"))
  assert is_startup_registered(params)


@pytest.mark.parametrize("key, value", [
  ("FrogPilotDongleId", "frogpilotserver1"),
  ("StockDongleId", "0123456789abcdef"),
  ("KonikDongleId", "fedcba9876543210"),
])
def test_a_stored_identity_is_kept(params, key, value):
  params.put(key, value)

  frogpilot_functions.create_local_identity(params)

  assert params.get(key) == value
  if key != "FrogPilotDongleId":
    assert params.get("FrogPilotDongleId") is None
