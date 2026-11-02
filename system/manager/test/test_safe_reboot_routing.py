import asyncio
import os
from contextlib import nullcontext
from pathlib import Path
from types import SimpleNamespace

import pytest

import cereal.messaging as messaging
from cereal import car, log
from openpilot.common.basedir import BASEDIR
from openpilot.common.params import Params
import openpilot.system.manager.manager as manager
from openpilot.system.manager.process_config import managed_processes
from openpilot.frogpilot.system.the_pond import device, utilities

SAFE_REBOOT = [os.path.join(BASEDIR, "fullupdate.sh"), "__safe_reboot"]
KEEP_RUNNING = ("card", "pandad")


def settings_reboot(params):
  # selfdrive/ui/qt/offroad/settings.cc:420 and selfdrive/ui/layouts/settings/device.py:180 (the settings Reboot button)
  params.put_bool("DoReboot", True)


def soft_reboot(params):
  # frogpilot/ui/qt/widgets/frogpilot_controls.cc:39 and selfdrive/ui/qt/offroad/settings.cc:447 (Soft Reboot)
  params.put_bool("DoSoftReboot", True)


def pond_endpoint(handler):
  def request(params):
    the_pond = SimpleNamespace(params=params, sm={"deviceState": SimpleNamespace(started=True), "selfdriveState": SimpleNamespace(enabled=False),
                                                  "frogpilotCarState": SimpleNamespace(alwaysOnLateralEnabled=False)})
    response = asyncio.run(handler(SimpleNamespace(app={utilities.THE_POND: the_pond})))
    assert response.status == 204
  return request


REQUESTS = {
  "settings_reboot": (settings_reboot, "DoReboot"),
  "soft_reboot": (soft_reboot, "DoSoftReboot"),
  "pond_reboot": (pond_endpoint(device.reboot), "DoReboot"),
  "pond_soft_reboot": (pond_endpoint(device.soft_reboot), "DoSoftReboot"),
}


class FakeSubMaster:
  """The manager's deviceState / pandaStates / carParams. Each update() first runs the next scripted step; the script ends with DoShutdown."""
  def __init__(self, steps, ignition, started):
    self.steps = list(steps)
    self.data = {"carParams": car.CarParams.new_message(brand="hyundai")}
    device_state = messaging.new_message("deviceState")
    device_state.deviceState.started = started
    self.data["deviceState"] = device_state.deviceState
    panda_states = messaging.new_message("pandaStates", 1)
    panda_states.pandaStates[0].pandaType = log.PandaState.PandaType.tres
    panda_states.pandaStates[0].ignitionLine = ignition
    self.data["pandaStates"] = panda_states.pandaStates

  def extend(self, services):
    return self

  def update(self, timeout=0):
    self.steps.pop(0)()

  def __getitem__(self, service):
    return self.data[service]


class Recorder:
  def __init__(self):
    self.events = []

  def ensure_running(self, procs, started, params=None, CP=None, not_run=None, frogpilot_toggles=None):
    keep = {name: name not in (not_run or []) and managed_processes[name].should_run(started, params, CP, frogpilot_toggles) for name in KEEP_RUNNING}
    self.events.append(("ensure_running", started, keep))

  def popen(self, args, **kwargs):
    assert kwargs["start_new_session"] and kwargs["cwd"] == BASEDIR
    self.events.append(("spawn", args))


def run_manager(monkeypatch, tmp_path, ignition, started, request, after_request=(), spawn_error=False):
  params = Params()
  recorder = Recorder()
  for key in ("DoReboot", "DoSoftReboot", "DoShutdown", "LastManagerExitReason", "LiveUpdateHandoffState"):
    params.remove(key)

  def popen(args, **kwargs):
    if spawn_error:
      raise FileNotFoundError(args[0])
    recorder.popen(args, **kwargs)

  steps = [lambda: None, lambda: request(params), *after_request, lambda: None, lambda: params.put_bool("DoShutdown", True)]
  sm = FakeSubMaster(steps, ignition, started)
  monkeypatch.setattr(manager.messaging, "SubMaster", lambda *args, **kwargs: sm)
  monkeypatch.setattr(manager.messaging, "PubMaster", lambda *args, **kwargs: SimpleNamespace(send=lambda *a: None))
  monkeypatch.setattr(manager, "ensure_running", recorder.ensure_running)
  monkeypatch.setattr(manager.subprocess, "Popen", popen)
  monkeypatch.setattr(manager, "FULLUPDATE_LOG", str(tmp_path / "fullupdate.log"))
  monkeypatch.setattr(manager, "atomic_write", lambda *args, **kwargs: nullcontext(SimpleNamespace(write=lambda data: None)))
  monkeypatch.setattr(manager.frogpilot_variables, "get_frogpilot_toggles", lambda *args: SimpleNamespace(force_onroad=False))

  manager.manager_thread()
  return params, recorder.events


@pytest.mark.parametrize("name", REQUESTS)
@pytest.mark.parametrize("ignition, started", [(True, True), (True, False)])
def test_restart_with_ignition_on_reaches_safe_reboot_before_cleanup(monkeypatch, tmp_path, name, ignition, started):
  request, param = REQUESTS[name]
  params, events = run_manager(monkeypatch, tmp_path, ignition, started, request)

  spawns = [i for i, event in enumerate(events) if event[0] == "spawn"]
  assert [events[i][1] for i in spawns] == [SAFE_REBOOT]
  assert not params.get_bool(param)
  # the loop went on after the spawn: manager_thread returned only for the scripted DoShutdown, so manager_cleanup never ran for the restart
  assert params.get("LastManagerExitReason").startswith("DoShutdown")
  after = [event for event in events[spawns[0]:] if event[0] == "ensure_running"]
  assert len(after) >= 2
  for _, ran_started, keep in after:
    assert ran_started == started
    assert keep == {"card": started, "pandad": True}


@pytest.mark.parametrize("name", REQUESTS)
def test_card_and_pandad_keep_running_while_the_handoff_fails(monkeypatch, tmp_path, name):
  request, _ = REQUESTS[name]

  def handoff_failed():
    Params().put("LiveUpdateHandoffState", "failed")

  params, events = run_manager(monkeypatch, tmp_path, True, True, request, after_request=[handoff_failed, lambda: None, lambda: None])

  assert [event[1] for event in events if event[0] == "spawn"] == [SAFE_REBOOT]
  assert params.get("LastManagerExitReason").startswith("DoShutdown")
  for _, ran_started, keep in [event for event in events if event[0] == "ensure_running"][1:]:
    assert ran_started and keep == {"card": True, "pandad": True}


@pytest.mark.parametrize("name", REQUESTS)
def test_failed_safe_reboot_spawn_never_falls_back_to_a_raw_restart(monkeypatch, tmp_path, name):
  request, param = REQUESTS[name]
  params, events = run_manager(monkeypatch, tmp_path, True, True, request, spawn_error=True)

  assert not params.get_bool(param)
  assert params.get("LastManagerExitReason").startswith("DoShutdown")
  for _, ran_started, keep in [event for event in events if event[0] == "ensure_running"][1:]:
    assert ran_started and keep == {"card": True, "pandad": True}


@pytest.mark.parametrize("name", REQUESTS)
def test_restart_with_ignition_off_keeps_the_upstream_exit(monkeypatch, tmp_path, name):
  request, param = REQUESTS[name]
  params, events = run_manager(monkeypatch, tmp_path, False, False, request)

  assert not any(event[0] == "spawn" for event in events)
  assert params.get_bool(param)
  assert params.get("LastManagerExitReason").startswith(param)


def test_safe_reboot_target_is_the_executable_script_entry_point():
  assert os.access(SAFE_REBOOT[0], os.X_OK)
  assert 'if [ "${1:-}" = "__safe_reboot" ]; then' in Path(SAFE_REBOOT[0]).read_text()
