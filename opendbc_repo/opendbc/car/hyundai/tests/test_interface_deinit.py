from types import SimpleNamespace

from opendbc.car.hyundai import interface
from opendbc.car.hyundai.interface import CarInterface
from opendbc.car.hyundai.values import CAR


def test_deinit_enables_radar_without_reconfiguring_tracks(monkeypatch):
  communication_requests = []
  radar_track_calls = []

  def disable_ecu(*args, **kwargs):
    communication_requests.append((kwargs["com_cont_req"], kwargs["retry"]))
    return True

  monkeypatch.setattr(interface, "disable_ecu", disable_ecu)
  monkeypatch.setattr(interface, "enable_radar_tracks", lambda *args: radar_track_calls.append(args))
  monkeypatch.setattr(interface, "params", SimpleNamespace(get_bool=lambda key: True))

  CP = SimpleNamespace(openpilotLongitudinalControl=True, flags=0, carFingerprint=CAR.HYUNDAI_SANTA_FE_HEV_2022)
  assert CarInterface.deinit(CP, object(), object(), retry=2)
  assert communication_requests == [(b"\x28\x80\x01", 2)]
  assert radar_track_calls == []
