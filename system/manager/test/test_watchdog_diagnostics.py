import os
import subprocess
import threading
import time
from types import SimpleNamespace

import openpilot.system.manager.process as process
from openpilot.system.manager.process import capture_ui_gdb_backtrace, get_process_diagnostics


def test_get_process_diagnostics(tmp_path):
  pid = 123
  tid = 456
  task_root = tmp_path / "proc" / str(pid) / "task" / str(tid)
  task_root.mkdir(parents=True)

  files = {
    "comm": "ui\n",
    "status": "Name:\tui\nState:\tD (disk sleep)\nPid:\t456\nCpus_allowed_list:\t0-7\nvoluntary_ctxt_switches:\t12\n",
    "wchan": "do_wait\n",
    "syscall": "202 0 0 0 0 0 0\n",
    "schedstat": "100 200 3\n",
    "stack": "[<0>] do_wait+0x1/0x2\n",
  }
  for name, content in files.items():
    (task_root / name).write_text(content)

  phase_prefix = str(tmp_path / "wd_phase_")
  (tmp_path / f"wd_phase_{pid}").write_bytes(b"paint_camera\0ignored")

  diagnostics = get_process_diagnostics(pid, proc_root=str(tmp_path / "proc"), phase_fn_prefix=phase_prefix)

  assert diagnostics["phase"] == "paint_camera"
  assert diagnostics["thread_count"] == 1
  assert diagnostics["threads_captured"] == 1
  assert diagnostics["truncated"] is False
  assert diagnostics["threads"][0] == {
    "tid": tid,
    "comm": "ui",
    "status": "State:\tD (disk sleep) | Pid:\t456 | Cpus_allowed_list:\t0-7 | voluntary_ctxt_switches:\t12",
    "wchan": "do_wait",
    "syscall": "202 0 0 0 0 0 0",
    "schedstat": "100 200 3",
    "stack": "[<0>] do_wait+0x1/0x2",
  }


def test_get_process_diagnostics_handles_missing_process(tmp_path):
  diagnostics = get_process_diagnostics(os.getpid(), proc_root=str(tmp_path / "missing"), phase_fn_prefix=str(tmp_path / "missing_phase_"))

  assert diagnostics["thread_count"] == 0
  assert diagnostics["threads"] == []
  assert diagnostics["task_error"].startswith("<FileNotFoundError:")


def test_capture_ui_gdb_backtrace(monkeypatch, tmp_path):
  output_path = tmp_path / "ui_watchdog_gdb.log"
  calls = []

  def run(command, **kwargs):
    calls.append((command, kwargs))
    return SimpleNamespace(stdout="Thread 1\n#0 wait_for_event ()\n", returncode=0)

  monkeypatch.setattr(subprocess, "run", run)
  diagnostics = capture_ui_gdb_backtrace(123, output_path=str(output_path), timeout=1.5)

  command = calls[0][0]
  assert command.index("thread 1") < command.index("bt 64")
  assert command.index("bt 64") < command.index("thread apply all bt 32")
  assert calls[0][1]["timeout"] == 1.5
  assert diagnostics["gdb_returncode"] == 0
  assert diagnostics["gdb_timed_out"] is False
  assert diagnostics["gdb_error"] == ""
  assert diagnostics["gdb_useful"] is True
  assert diagnostics["gdb_skipped"] is False
  assert diagnostics["gdb_path"] != str(output_path)
  assert os.path.exists(diagnostics["gdb_path"])
  assert "pid=123 returncode=0 timed_out=False error=none" in output_path.read_text()
  assert "#0 wait_for_event ()" in output_path.read_text()

  skipped = capture_ui_gdb_backtrace(123, output_path=str(output_path), timeout=1.5)
  assert skipped["gdb_skipped"] is True
  assert skipped["gdb_path"] == diagnostics["gdb_path"]
  assert len(calls) == 1


def test_capture_ui_gdb_backtrace_timeout(monkeypatch, tmp_path):
  output_path = tmp_path / "ui_watchdog_gdb.log"

  def run(*args, **kwargs):
    raise subprocess.TimeoutExpired("gdb", 0.1, output=b"partial backtrace\n")

  monkeypatch.setattr(subprocess, "run", run)
  diagnostics = capture_ui_gdb_backtrace(456, output_path=str(output_path), timeout=0.1)

  assert diagnostics["gdb_returncode"] is None
  assert diagnostics["gdb_timed_out"] is True
  assert diagnostics["gdb_error"] == "gdb exceeded 0.1s timeout"
  assert diagnostics["gdb_useful"] is False
  assert diagnostics["gdb_skipped"] is False
  assert "partial backtrace" in output_path.read_text()


def test_capture_ui_gdb_backtrace_preserves_main_thread_when_truncated(monkeypatch, tmp_path):
  output_path = tmp_path / "ui_watchdog_gdb.log"

  monkeypatch.setattr(process, "UI_WATCHDOG_GDB_MAX_OUTPUT", 32)
  monkeypatch.setattr(subprocess, "run", lambda *args, **kwargs: SimpleNamespace(stdout="MAIN THREAD\n" + "x" * 100, returncode=0))
  diagnostics = capture_ui_gdb_backtrace(789, output_path=str(output_path))

  assert diagnostics["gdb_output_truncated"] is True
  assert "MAIN THREAD" in output_path.read_text()


def test_ui_watchdog_capture_does_not_block_manager(monkeypatch):
  capture_started = threading.Event()
  release_capture = threading.Event()
  logged_events = []
  restarts = []

  def capture(pid):
    capture_started.set()
    assert release_capture.wait(1.0)
    return {"gdb_path": "/tmp/ui.log", "gdb_error": ""}

  monkeypatch.setattr(process, "ENABLE_WATCHDOG", True)
  monkeypatch.setattr(process, "capture_ui_gdb_backtrace", capture)
  monkeypatch.setattr(process, "get_process_diagnostics", lambda pid: {"pid": pid, "phase": "event_loop_idle"})
  monkeypatch.setattr(process.cloudlog, "error", lambda *args, **kwargs: None)
  monkeypatch.setattr(process.cloudlog, "event", lambda name, **kwargs: logged_events.append((name, kwargs)))

  ui = process.NativeProcess("ui", ".", ["./ui"], lambda *_: True, watchdog_max_dt=5)
  ui.proc = SimpleNamespace(pid=123, exitcode=None)
  ui.watchdog_seen = True
  ui.last_watchdog_time = 0

  def restart():
    restarts.append(True)
    ui.proc = None

  monkeypatch.setattr(ui, "restart", restart)

  started = time.monotonic()
  ui.check_watchdog(started=True)
  elapsed = time.monotonic() - started

  assert capture_started.wait(0.2)
  assert elapsed < 0.5
  assert restarts == []
  assert [name for name, _ in logged_events] == ["watchdog_process_diagnostics"]

  ui.check_watchdog(started=True)
  assert restarts == []

  capture_state = ui.ui_watchdog_capture
  assert capture_state is not None
  release_capture.set()
  capture_state.thread.join(1.0)
  assert not capture_state.thread.is_alive()

  ui.check_watchdog(started=True)
  assert restarts == [True]
  assert [name for name, _ in logged_events] == ["watchdog_process_diagnostics", "watchdog_gdb_backtrace"]
  assert logged_events[-1][1]["watchdog_dt"] > 5
