import os
import subprocess
import threading
import time
from types import SimpleNamespace

import openpilot.system.manager.process as process
from openpilot.system.manager.process import capture_compositor_diagnostics, capture_ui_gdb_backtrace, get_process_diagnostics


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


def test_capture_compositor_diagnostics(tmp_path):
  proc_root = tmp_path / "proc"
  weston_pid, ui_pid = 42, 123

  weston_task = proc_root / str(weston_pid) / "task" / str(weston_pid)
  weston_task.mkdir(parents=True)
  (proc_root / str(weston_pid) / "comm").write_text("weston\n")
  for name, content in {"comm": "weston\n", "status": "State:\tS (sleeping)\n", "wchan": "ep_poll\n", "syscall": "22 0\n",
                        "schedstat": "1 2 3\n", "stack": "[<0>] ep_poll+0x1/0x2\n"}.items():
    (weston_task / name).write_text(content)
  weston_fd = proc_root / str(weston_pid) / "fd"
  weston_fd.mkdir()
  (weston_fd / "7").symlink_to("socket:[1111]")

  ui_fd = proc_root / str(ui_pid) / "fd"
  ui_fd.mkdir(parents=True)
  (ui_fd / "5").symlink_to("socket:[2222]")
  (ui_fd / "6").symlink_to("/dev/null")

  commands = []

  def run_cmd(command, **kwargs):
    commands.append(command)
    if command[0] == "ss":
      return "Netid State Recv-Q Send-Q Local Peer\nu_str ESTAB 4096 0 /run/wayland-0 1111 * 2222\nu_str ESTAB 0 0 * 9999 * 8888\n"
    return "kms state"

  output_path = tmp_path / "ui_watchdog_compositor.log"
  diagnostics = capture_compositor_diagnostics(ui_pid, proc_root=str(proc_root), output_path=str(output_path), run_cmd=run_cmd)

  assert diagnostics["weston_pid"] == weston_pid
  assert diagnostics["weston"]["threads"][0]["wchan"] == "ep_poll"
  assert diagnostics["ui_socket_fds"] == {"5": "2222"}
  assert diagnostics["weston_socket_fds"] == {"7": "1111"}
  assert "wayland-0" in diagnostics["ss_wayland"]
  assert "9999" not in diagnostics["ss_wayland"]
  assert diagnostics["dri_state"] == "kms state"
  assert commands[0][0] == "ss"
  assert commands[1][:2] == ["sudo", "-n"]
  assert os.path.exists(diagnostics["compositor_path"])
  assert f".{ui_pid}.log" in diagnostics["compositor_path"]


def test_capture_compositor_diagnostics_no_weston(tmp_path):
  diagnostics = capture_compositor_diagnostics(123, proc_root=str(tmp_path / "missing"), output_path=str(tmp_path / "comp.log"),
                                               run_cmd=lambda command, **kwargs: "<FileNotFoundError: ss>")

  assert diagnostics["weston_pid"] is None
  assert diagnostics["weston"] is None
  assert diagnostics["ui_socket_fds"] == {}
  assert os.path.exists(diagnostics["compositor_path"])


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
  monkeypatch.setattr(process, "capture_compositor_diagnostics", lambda pid: {"weston_pid": 42})
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
  assert [name for name, _ in logged_events] == ["watchdog_process_diagnostics", "watchdog_compositor_diagnostics", "watchdog_gdb_backtrace"]
  assert logged_events[1][1]["weston_pid"] == 42
  assert logged_events[-1][1]["watchdog_dt"] > 5
