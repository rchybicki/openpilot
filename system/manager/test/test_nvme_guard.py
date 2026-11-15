import errno
import os
import subprocess
import threading
import time
from types import SimpleNamespace

import pytest

import openpilot.selfdrive.selfdrived.alertmanager as alertmanager
import openpilot.selfdrive.selfdrived.events as events
import openpilot.frogpilot.common.frogpilot_functions as frogpilot_functions
import openpilot.system.manager.helpers as helpers

MOUNT_UNIT_START = ["sudo", "-n", "systemctl", "start", "data-media.mount"]


class FakeDevice:
  def __init__(self, tmp_path, monkeypatch):
    self.mountpoint = tmp_path / "media"
    self.mountpoint.mkdir()
    self.device = tmp_path / "nvme0n1"
    self.pci_function = tmp_path / "0001:01:00.0"
    self.pci_function.mkdir()
    self.rescan = tmp_path / "rescan"
    self.fallback = tmp_path / "realdata_fallback"

    self.mounted = False
    self.mountable = True
    self.appears_after_sleeps: int | None = None
    self.rescan_revives = False
    self.run_error: Exception | None = None

    self.calls: list[tuple[list[str], bytes | None]] = []
    self.sleeps = 0
    self.now = 0.0
    self.events: list[dict] = []

    real_ismount = os.path.ismount
    monkeypatch.setattr(helpers.HARDWARE, "get_device_type", lambda: "tici")
    monkeypatch.delenv("LOG_ROOT", raising=False)
    # the guard writes this mark into os.environ itself: register it, so that teardown removes it after the test
    monkeypatch.setenv(helpers.NVME_GUARD_FAILED_ENV, "1")
    monkeypatch.delenv(helpers.NVME_GUARD_FAILED_ENV)
    monkeypatch.setattr(helpers, "NVME_MOUNTPOINT", str(self.mountpoint))
    monkeypatch.setattr(helpers, "NVME_DEVICE", str(self.device))
    monkeypatch.setattr(helpers, "NVME_PCI_FUNCTION", str(self.pci_function))
    monkeypatch.setattr(helpers, "PCI_RESCAN", str(self.rescan))
    monkeypatch.setattr(helpers, "NVME_FALLBACK_LOG_ROOT", str(self.fallback))
    monkeypatch.setattr(helpers, "time", SimpleNamespace(monotonic=lambda: self.now, sleep=self.sleep))
    monkeypatch.setattr(helpers.os.path, "ismount", lambda p: self.mounted if p == str(self.mountpoint) else real_ismount(p))
    monkeypatch.setattr(helpers.subprocess, "run", self.run)
    monkeypatch.setattr(helpers.cloudlog, "event", lambda event, **kwargs: self.events.append({"event": event, **kwargs}))

  @property
  def remove_call(self):
    return ["sudo", "-n", "tee", str(self.pci_function / "remove")], b"1"

  @property
  def rescan_call(self):
    return ["sudo", "-n", "tee", str(self.rescan)], b"1"

  def sleep(self, seconds):
    self.sleeps += 1
    self.now += seconds
    if self.sleeps == self.appears_after_sleeps:
      self.device.touch()

  def run(self, argv, **kwargs):
    self.calls.append((argv, kwargs.get("input")))
    if self.run_error is not None:
      raise self.run_error
    if argv == MOUNT_UNIT_START and self.device.exists() and self.mountable:
      self.mounted = True
    elif (argv, kwargs.get("input")) == self.remove_call:
      self.device.unlink(missing_ok=True)
    elif (argv, kwargs.get("input")) == self.rescan_call and self.rescan_revives:
      self.device.touch()
    return subprocess.CompletedProcess(argv, 0)

  def assert_fallback(self):
    assert os.environ["LOG_ROOT"] == str(self.fallback)
    assert self.fallback.is_dir()
    assert list(self.mountpoint.iterdir()) == [], "nothing written under the bare mountpoint"
    assert any(e["step"] == "fallback" and e.get("error") for e in self.events)


@pytest.fixture
def nvme(tmp_path, monkeypatch):
  return FakeDevice(tmp_path, monkeypatch)


def eio(**kwargs):
  raise OSError(errno.EIO, "Input/output error")


def test_mounted_at_boot_returns_at_once(nvme):
  nvme.mounted = True
  assert helpers.ensure_media_storage()
  assert nvme.calls == []
  assert nvme.sleeps == 0
  assert "LOG_ROOT" not in os.environ
  assert list(nvme.mountpoint.iterdir()) == [], "the probe cleans up"


def test_dead_mount_falls_back_without_commands(nvme, monkeypatch):
  nvme.mounted = True
  nvme.device.touch()
  monkeypatch.setattr(helpers, "tempfile", SimpleNamespace(mkdtemp=eio))
  assert not helpers.ensure_media_storage()
  assert nvme.calls == []
  assert nvme.sleeps == 0
  assert {"event": "nvme_guard", "step": "probe", "result": repr(OSError(errno.EIO, "Input/output error")), "error": True} in nvme.events
  nvme.assert_fallback()


def test_hung_mount_falls_back_within_timeout(nvme, monkeypatch):
  nvme.mounted = True
  release = threading.Event()

  def hang(**kwargs):
    release.wait()
    raise OSError(errno.EIO, "Input/output error")

  monkeypatch.setattr(helpers, "tempfile", SimpleNamespace(mkdtemp=hang))
  monkeypatch.setattr(helpers, "NVME_PROBE_TIMEOUT_S", 0.2)
  start = time.monotonic()
  try:
    assert not helpers.ensure_media_storage()
    assert time.monotonic() - start < 2
  finally:
    release.set()
  assert nvme.calls == []
  assert {"event": "nvme_guard", "step": "probe", "result": "timeout", "error": True} in nvme.events
  nvme.assert_fallback()


def test_late_device_is_mounted(nvme):
  nvme.appears_after_sleeps = 2
  assert helpers.ensure_media_storage()
  assert nvme.calls == [(MOUNT_UNIT_START, None)]
  assert nvme.sleeps == 2
  assert "LOG_ROOT" not in os.environ
  assert not nvme.fallback.exists()


def test_rescan_revives_device(nvme):
  nvme.rescan_revives = True
  assert helpers.ensure_media_storage()
  assert nvme.calls == [nvme.remove_call, nvme.rescan_call, (MOUNT_UNIT_START, None)]
  assert "LOG_ROOT" not in os.environ


def test_missing_device_falls_back_to_internal_storage(nvme):
  assert not helpers.ensure_media_storage()
  assert nvme.calls == [nvme.remove_call, nvme.rescan_call]
  assert 30 + 15 <= nvme.now <= 30 + 15 + 2, "bounded boot delay"
  nvme.assert_fallback()


def test_absent_pci_function_rescans_only(nvme):
  nvme.pci_function.rmdir()
  assert not helpers.ensure_media_storage()
  assert nvme.calls == [nvme.rescan_call]
  nvme.assert_fallback()


def test_late_mount_of_a_dead_nvme_falls_back(nvme, monkeypatch):
  nvme.appears_after_sleeps = 2
  monkeypatch.setattr(helpers, "tempfile", SimpleNamespace(mkdtemp=eio))
  assert not helpers.ensure_media_storage()
  assert nvme.calls == [(MOUNT_UNIT_START, None)], "no reset of a mounted device"
  nvme.assert_fallback()


@pytest.mark.parametrize("run_error", [None, FileNotFoundError("sudo")])
def test_present_device_that_never_mounts_is_not_reset(nvme, run_error):
  nvme.appears_after_sleeps = 3
  nvme.mountable = False
  nvme.run_error = run_error
  assert not helpers.ensure_media_storage()
  assert {tuple(argv) for argv, _ in nvme.calls} == {tuple(MOUNT_UNIT_START)}
  assert nvme.now <= helpers.NVME_MOUNT_WAIT_S + 1, "bounded boot delay"
  nvme.assert_fallback()


@pytest.mark.parametrize("error", [subprocess.TimeoutExpired(["sudo", "-n", "tee"], 10), FileNotFoundError("sudo")])
def test_failing_commands_do_not_raise(nvme, error):
  nvme.run_error = error
  assert not helpers.ensure_media_storage()
  assert nvme.calls == [nvme.remove_call, nvme.rescan_call], "a failed remove still rescans"
  assert [e["result"] for e in nvme.events if "result" in e] == [repr(error)] * 2, "every failure is logged"
  nvme.assert_fallback()


@pytest.mark.parametrize("device_type", ["pc", "mici"])
def test_guard_skips_other_devices(nvme, monkeypatch, device_type):
  monkeypatch.setattr(helpers.HARDWARE, "get_device_type", lambda: device_type)
  assert helpers.ensure_media_storage()
  assert nvme.calls == []
  assert "LOG_ROOT" not in os.environ


def test_guard_skips_preset_log_root(nvme, monkeypatch):
  monkeypatch.setenv("LOG_ROOT", "/custom/root")
  assert helpers.ensure_media_storage()
  assert nvme.calls == []
  assert os.environ["LOG_ROOT"] == "/custom/root"


def test_storage_alert_replaces_startup_alert(monkeypatch):
  store: dict = {}

  class FakeParams:
    def get(self, key):
      return store.get(key)

    def put(self, key, value):
      store[key] = value

    def remove(self, key):
      store.pop(key, None)

  monkeypatch.setattr(alertmanager, "Params", FakeParams)
  monkeypatch.setattr(events, "Params", FakeParams)
  toggles = SimpleNamespace(startup_alert_top="Hop in and buckle up!", startup_alert_bottom="Human-tested")

  def startup_alert():
    return events.custom_startup_alert(None, None, None, True, 0, None, toggles)

  alertmanager.set_offroad_alert("Offroad_StorageMissing", False)
  assert (startup_alert().alert_text_1, startup_alert().alert_text_2) == ("Hop in and buckle up!", "Human-tested")

  alertmanager.set_offroad_alert("Offroad_StorageMissing", True)
  assert store["Offroad_StorageMissing"]["severity"] == 1
  assert startup_alert().alert_text_1.startswith("NVMe failed")
  assert startup_alert().alert_status == events.FrogPilotAlertStatus.userPrompt



def test_storage_alert_kept_for_a_mounted_nvme_that_failed_the_probe(nvme, monkeypatch):
  # hardwared rewrites Offroad_StorageMissing on every pass: a mounted NVMe must not clear the guard's fallback of this boot
  nvme.mounted = True
  monkeypatch.setattr(helpers, "tempfile", SimpleNamespace(mkdtemp=eio))
  assert not helpers.ensure_media_storage()
  assert helpers.media_storage_missing()


def test_storage_alert_follows_the_mount_after_a_good_boot(nvme):
  nvme.mounted = True
  assert helpers.ensure_media_storage()
  assert not helpers.media_storage_missing()

  nvme.mounted = False
  assert helpers.media_storage_missing()


def test_storage_alert_clear_for_an_explicit_fallback_path(nvme, monkeypatch):
  # LOG_ROOT given from outside, also the fallback path, is no guard failure: a healthy NVMe gets no alert
  nvme.mounted = True
  monkeypatch.setenv("LOG_ROOT", str(nvme.fallback))
  assert helpers.ensure_media_storage()
  assert not helpers.media_storage_missing()


def test_storage_alert_ignores_an_inherited_failure_mark(nvme, monkeypatch):
  # only this boot's guard sets the mark
  monkeypatch.setenv(helpers.NVME_GUARD_FAILED_ENV, "1")
  nvme.mounted = True
  assert helpers.ensure_media_storage()
  assert not helpers.media_storage_missing()


@pytest.mark.parametrize("media_ok", [True, False])
def test_install_frogpilot_creates_nvme_dirs_only_with_storage(tmp_path, monkeypatch, media_ok):
  paths = {name: tmp_path / name for name in ("ERROR_LOGS_PATH", "HD_LOGS_PATH", "KONIK_LOGS_PATH", "SCREEN_RECORDINGS_PATH")}
  for name, path in paths.items():
    monkeypatch.setattr(frogpilot_functions.frogpilot_paths, name, path)
  monkeypatch.setattr(frogpilot_functions, "update_boot_logo", lambda *args: None)

  frogpilot_functions.install_frogpilot(media_ok)

  assert paths["ERROR_LOGS_PATH"].is_dir()
  assert paths["HD_LOGS_PATH"].is_dir() == media_ok
  assert paths["KONIK_LOGS_PATH"].is_dir() == media_ok
  assert paths["SCREEN_RECORDINGS_PATH"].is_dir() == media_ok


def test_install_frogpilot_skips_recording_cleanup_without_storage(tmp_path, monkeypatch):
  # the cleanup scans the NVMe: after the guard's fallback it can raise or block the start-up the guard protects
  def scan(**kwargs):
    raise AssertionError("scanned the NVMe after the guard's fallback")

  monkeypatch.setattr(frogpilot_functions.frogpilot_paths, "ERROR_LOGS_PATH", tmp_path / "error_logs")
  monkeypatch.setattr(frogpilot_functions.frogpilot_paths, "SCREEN_RECORDINGS_PATH", SimpleNamespace(glob=lambda pattern: scan()))
  monkeypatch.setattr(frogpilot_functions, "update_boot_logo", lambda *args: None)

  frogpilot_functions.install_frogpilot(False)


def test_install_frogpilot_survives_a_recording_cleanup_error(tmp_path, monkeypatch):
  boot_logo_updates = []
  for name in ("ERROR_LOGS_PATH", "HD_LOGS_PATH", "KONIK_LOGS_PATH"):
    monkeypatch.setattr(frogpilot_functions.frogpilot_paths, name, tmp_path / name)
  monkeypatch.setattr(frogpilot_functions.frogpilot_paths, "SCREEN_RECORDINGS_PATH",
                      SimpleNamespace(mkdir=lambda **kwargs: None, glob=lambda pattern: eio()))
  monkeypatch.setattr(frogpilot_functions, "update_boot_logo", lambda *args: boot_logo_updates.append(args))

  frogpilot_functions.install_frogpilot(True)

  assert boot_logo_updates, "an I/O error in the recording cleanup stopped manager start"
