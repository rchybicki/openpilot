import json
import threading
from datetime import datetime, timezone
from types import SimpleNamespace

import pytest

from cereal import messaging
from openpilot.frogpilot.common import frogpilot_functions

WROCLAW = json.dumps({"latitude": 51.0936, "longitude": 17.0547, "bearing": 0.0})


class FakePubMaster:
  def __init__(self, services, sent):
    self.sent = sent

  def send(self, service, msg):
    self.sent.append((service, str(msg.mapdIn.type), msg.mapdIn.str))


class FakeSubMaster:
  """mapd answers every download with one active frame, then a finished, complete one."""
  def __init__(self, services):
    self.frames = [SimpleNamespace(active=True, cancelled=False, downloadedFiles=0, totalFiles=1),
                   SimpleNamespace(active=False, cancelled=False, downloadedFiles=1, totalFiles=1)]
    self.updated = {"mapdExtendedOut": False}
    self.current = None

  def update(self, timeout):
    self.updated["mapdExtendedOut"] = bool(self.frames)
    if self.frames:
      self.current = SimpleNamespace(downloadProgress=self.frames.pop(0))

  def __getitem__(self, service):
    return self.current


@pytest.fixture
def mapd_requests(monkeypatch):
  sent = []
  fake = SimpleNamespace(PubMaster=lambda services: FakePubMaster(services, sent), SubMaster=FakeSubMaster, new_message=messaging.new_message)
  monkeypatch.setattr(frogpilot_functions, "messaging", fake)
  monkeypatch.setattr(frogpilot_functions.time, "sleep", lambda s: None)
  return sent


def run_update_maps(maps_selected, metered=False):
  stored = {"LastGPSPosition": WROCLAW, "MapsSelected": maps_selected, "PreferredSchedule": 0, "LastMapsUpdate": ""}
  params = SimpleNamespace(get=stored.get, put=stored.__setitem__)
  sm = {"deviceState": SimpleNamespace(networkMetered=metered)}
  frogpilot_functions.update_maps(datetime(2026, 11, 14, 12, tzinfo=timezone.utc), params, threading.Event(), sm, manual_update=True)
  return stored


def test_no_maps_selected_downloads_nothing_even_with_a_known_position(mapd_requests):
  run_update_maps("")
  assert mapd_requests == []


@pytest.mark.parametrize("metered", [False, True])
def test_a_manual_download_asks_mapd_only_for_the_selected_maps(mapd_requests, metered):
  stored = run_update_maps("nation.PL", metered=metered)
  assert mapd_requests == [("mapdIn", "download", "nation.PL")]
  assert stored["LastMapsUpdate"] == "November 14th, 2026"
