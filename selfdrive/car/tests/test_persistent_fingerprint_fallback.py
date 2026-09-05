"""VIN-verified persistent fallback when the boot-time fingerprint fails (2026-09-05: the boot after an on-road
live restart got 16/20 firmware answers -> MOCK). Real get_car with fingerprint() replaced by a stub."""
from types import SimpleNamespace

from opendbc.car import car_helpers
from opendbc.car.car_helpers import get_car, persistent_fingerprint_fallback
from opendbc.car.vin import VIN_UNKNOWN
from opendbc.car.structs import CarParams

VIN = "KMHS5811DPU061252"


def _persistent(brand="hyundai", fp="HYUNDAI_SANTA_FE_HEV_2022", vin=VIN, fw_n=20):
  fw = [CarParams.CarFw(ecu=CarParams.Ecu.fwdCamera, fwVersion=b"x", address=0x7c4 + i) for i in range(fw_n)]
  return SimpleNamespace(brand=brand, carFingerprint=fp, carVin=vin, carFw=fw)


def _toggles(force=False, car_model="HYUNDAI_SANTA_FE_HEV_2022"):
  return SimpleNamespace(force_fingerprint=force, car_model=car_model, block_user=False, disable_openpilot_long=False)


class _Params:
  def __init__(self):
    self.store = {}

  def put_nonblocking(self, k, v):
    self.store[k] = v

  def put(self, k, v):
    self.store[k] = v

  def get(self, k, *a, **kw):
    return self.store.get(k)

  def get_bool(self, k, *a, **kw):
    return bool(self.store.get(k))


def _stub_fingerprint(monkeypatch, candidate, vin=VIN):
  monkeypatch.setattr(car_helpers, "fingerprint",
                      lambda *a, **k: (candidate, {0: {}}, vin, [], CarParams.FingerprintSource.can, False))


class _StubInterface:
  """Stands in for every car interface: records the CarParams get_car built (the real Hyundai interface needs a
  full bus fingerprint and toggles set, which is not what this test is about)."""
  def __init__(self, CP, FPCP):
    self.CP, self.FPCP = CP, FPCP

  @staticmethod
  def get_params(candidate, fingerprints, car_fw, alpha_long_allowed, is_release, docs=False, frogpilot_toggles=None):
    cp = CarParams(carFingerprint=candidate, brand="hyundai" if candidate.startswith("HYUNDAI") else "mock")
    cp.alphaLongitudinalAvailable = True
    return cp

  @staticmethod
  def get_frogpilot_params(candidate, fingerprints, car_fw, CP, frogpilot_toggles):
    return SimpleNamespace(flags=0, openpilotLongitudinalControlDisabled=False)


def _stub_interfaces(monkeypatch):
  monkeypatch.setattr(car_helpers, "interfaces", dict.fromkeys(("HYUNDAI_SANTA_FE_HEV_2022", "HYUNDAI_ELANTRA_2021", "MOCK"), _StubInterface))


def _car(monkeypatch, persistent, toggles=None):
  _stub_interfaces(monkeypatch)
  return get_car(None, None, lambda *a: None, True, False, _Params(), 1, None, toggles or _toggles(), persistent_params=persistent)


def test_fallback_needs_a_matching_vin_and_a_recognised_persistent_car():
  assert persistent_fingerprint_fallback(VIN, _persistent()) is not None
  assert persistent_fingerprint_fallback("OTHERVIN000000000", _persistent()) is None
  assert persistent_fingerprint_fallback(VIN_UNKNOWN, _persistent(vin=VIN_UNKNOWN)) is None
  assert persistent_fingerprint_fallback(VIN, _persistent(brand="mock", fp="MOCK")) is None
  assert persistent_fingerprint_fallback(VIN, _persistent(fw_n=0)) is None
  assert persistent_fingerprint_fallback(VIN, None) is None


def test_get_car_uses_the_persistent_car_when_the_query_fails(monkeypatch):
  _stub_fingerprint(monkeypatch, None)
  ci = _car(monkeypatch, _persistent())
  assert ci.CP.carFingerprint == "HYUNDAI_SANTA_FE_HEV_2022" and ci.CP.brand == "hyundai"
  assert ci.CP.fingerprintSource == CarParams.FingerprintSource.fixed and len(ci.CP.carFw) == 20 and ci.CP.carVin == VIN


def test_get_car_stays_mock_without_a_vin_match_or_persistent_record(monkeypatch):
  _stub_fingerprint(monkeypatch, None)
  assert _car(monkeypatch, _persistent(vin="X" * 17)).CP.carFingerprint == "MOCK"
  assert _car(monkeypatch, None).CP.carFingerprint == "MOCK"


def test_fallback_never_overrides_a_successful_or_forced_fingerprint(monkeypatch):
  _stub_fingerprint(monkeypatch, "HYUNDAI_SANTA_FE_HEV_2022")
  ci = _car(monkeypatch, _persistent(fp="HYUNDAI_ELANTRA_2021"))
  assert ci.CP.carFingerprint == "HYUNDAI_SANTA_FE_HEV_2022" and ci.CP.fingerprintSource == CarParams.FingerprintSource.can
  _stub_fingerprint(monkeypatch, None)
  ci = _car(monkeypatch, _persistent(), toggles=_toggles(force=True, car_model="HYUNDAI_ELANTRA_2021"))
  assert ci.CP.carFingerprint == "HYUNDAI_ELANTRA_2021"
