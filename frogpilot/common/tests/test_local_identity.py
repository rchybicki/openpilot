import re
from types import SimpleNamespace

import pytest

from openpilot.common.params import Params
from openpilot.frogpilot.common import frogpilot_api, frogpilot_functions
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
  params.put("FrogPilotRegistration", "digest of the registration")

  frogpilot_functions.create_local_identity(params)

  assert params.get(key) == value
  assert params.get("FrogPilotRegistration") == "digest of the registration"
  if key != "FrogPilotDongleId":
    assert params.get("FrogPilotDongleId") is None


class StopsAtTheIdentityWrite:
  # the device loses power between the two writes of create_local_identity
  def __init__(self, params):
    self.params = params

  def __getattr__(self, name):
    return getattr(self.params, name)

  def put(self, key, value):
    raise SystemExit(77)


def test_the_cached_registration_goes_before_the_local_identity(params):
  # a stop between the two writes must not leave a local identity next to a cached registration (the API would then never register)
  params.put("FrogPilotRegistration", "digest of an earlier registration")

  with pytest.raises(SystemExit):
    frogpilot_functions.create_local_identity(StopsAtTheIdentityWrite(params))

  assert params.get("FrogPilotDongleId") is None
  assert params.get("FrogPilotRegistration") is None


class InlineThread:
  def __init__(self, target, daemon):
    self.target = target

  def start(self):
    self.target()


def test_a_local_identity_does_not_count_as_a_completed_registration(params, monkeypatch):
  # the identity keys are gone, but the API token and the registration digest of this build and key are still stored
  posts = []

  def signed_post(self, path, payload, session=None):
    posts.append(path)
    return SimpleNamespace(status_code=200, json=lambda: {"frogpilot_dongle_id": "server0123456789"})

  monkeypatch.setattr(frogpilot_api, "get_key_pair", lambda: ("RS256", "private key", "public key"))
  monkeypatch.setattr(frogpilot_api, "system_time_valid", lambda: True)
  monkeypatch.setattr(frogpilot_api, "threading", SimpleNamespace(Thread=InlineThread))
  monkeypatch.setattr(frogpilot_api.FrogPilotAPI, "signed_post", signed_post)
  api = frogpilot_api.FrogPilotAPI(params)

  api.register_device()
  assert params.get("FrogPilotDongleId") == "server0123456789"
  assert params.get("FrogPilotApiToken") and params.get("FrogPilotRegistration")

  for key in IDENTITY_KEYS:
    params.remove(key)
  frogpilot_functions.create_local_identity(params)
  api.register_device()

  assert posts == ["/v1/register", "/v1/register"], "the local identity stopped the registration"
  assert params.get("FrogPilotDongleId") == "server0123456789"
