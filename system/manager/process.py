import glob
import importlib
import os
import signal
import struct
import time
import subprocess
import threading
from collections.abc import Callable, ValuesView
from abc import ABC, abstractmethod
from multiprocessing import Process
from types import SimpleNamespace

from setproctitle import setproctitle

from cereal import car, log
import cereal.messaging as messaging
import openpilot.system.sentry as sentry
from openpilot.common.basedir import BASEDIR
from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog
from openpilot.common.watchdog import WATCHDOG_FN, WATCHDOG_PHASE_FN

ENABLE_WATCHDOG = os.getenv("NO_WATCHDOG") is None
WATCHDOG_DIAGNOSTIC_MAX_THREADS = 32
WATCHDOG_DIAGNOSTIC_TIME_BUDGET = 0.25
UI_WATCHDOG_GDB_PATH = "/data/ui_watchdog_gdb.log"
UI_WATCHDOG_GDB_TIMEOUT = 10.0
UI_WATCHDOG_GDB_MAX_OUTPUT = 512 * 1024
UI_WATCHDOG_GDB_HISTORY_LIMIT = 5


def _read_diagnostic_file(path: str, limit: int = 4096) -> str:
  try:
    with open(path, errors="replace") as f:
      return f.read(limit).strip()
  except OSError as e:
    return f"<{type(e).__name__}: {e}>"


def get_process_diagnostics(pid: int, proc_root: str = "/proc", phase_fn_prefix: str = WATCHDOG_PHASE_FN) -> dict:
  started = time.monotonic()
  process_root = os.path.join(proc_root, str(pid))
  task_root = os.path.join(process_root, "task")
  phase = _read_diagnostic_file(f"{phase_fn_prefix}{pid}", 64).split("\0", 1)[0]

  try:
    tids = sorted((entry for entry in os.listdir(task_root) if entry.isdigit()), key=int)
  except OSError as e:
    tids = []
    task_error = f"<{type(e).__name__}: {e}>"
  else:
    task_error = ""

  threads = []
  truncated = len(tids) > WATCHDOG_DIAGNOSTIC_MAX_THREADS
  for tid in tids[:WATCHDOG_DIAGNOSTIC_MAX_THREADS]:
    if time.monotonic() - started >= WATCHDOG_DIAGNOSTIC_TIME_BUDGET:
      truncated = True
      break

    thread_root = os.path.join(task_root, tid)
    status = _read_diagnostic_file(os.path.join(thread_root, "status"), 4096)
    status_lines = tuple(line for line in status.splitlines() if line.startswith((
      "State:", "Tgid:", "Pid:", "PPid:", "Threads:", "Cpus_allowed_list:",
      "voluntary_ctxt_switches:", "nonvoluntary_ctxt_switches:",
    )))
    threads.append({
      "tid": int(tid),
      "comm": _read_diagnostic_file(os.path.join(thread_root, "comm"), 128),
      "status": " | ".join(status_lines) if status_lines else status,
      "wchan": _read_diagnostic_file(os.path.join(thread_root, "wchan"), 256),
      "syscall": _read_diagnostic_file(os.path.join(thread_root, "syscall"), 1024),
      "schedstat": _read_diagnostic_file(os.path.join(thread_root, "schedstat"), 256),
      "stack": _read_diagnostic_file(os.path.join(thread_root, "stack"), 4096),
    })

  return {
    "pid": pid,
    "phase": phase,
    "task_error": task_error,
    "thread_count": len(tids),
    "threads_captured": len(threads),
    "truncated": truncated,
    "capture_time": round(time.monotonic() - started, 4),
    "threads": threads,
  }


def _decode_subprocess_output(output: str | bytes | None) -> str:
  if output is None:
    return ""
  if isinstance(output, bytes):
    return output.decode(errors="replace")
  return output


def capture_ui_gdb_backtrace(pid: int, output_path: str = UI_WATCHDOG_GDB_PATH, timeout: float = UI_WATCHDOG_GDB_TIMEOUT) -> dict:
  started = time.monotonic()
  success_path = f"{output_path}.success"
  try:
    with open(success_path) as f:
      captured_path = f.read().strip()
    if captured_path and os.path.exists(captured_path):
      return {
        "gdb_path": captured_path,
        "gdb_returncode": None,
        "gdb_timed_out": False,
        "gdb_output_bytes": 0,
        "gdb_output_truncated": False,
        "gdb_capture_time": round(time.monotonic() - started, 4),
        "gdb_error": "",
        "gdb_useful": True,
        "gdb_skipped": True,
      }
  except OSError:
    pass

  command = [
    "gdb", "--batch", "--nx", "--quiet",
    "-ex", "set pagination off",
    "-ex", "set confirm off",
    "-ex", "set debuginfod enabled off",
    "-ex", "set print thread-events off",
    "-ex", f"attach {pid}",
    # The main UI thread is the evidence we need and must be captured before slower background
    # thread unwinding. Kernel-level state for every thread is already recorded above from /proc.
    "-ex", "thread 1",
    "-ex", "bt 64",
    "-ex", "thread apply all bt 32",
    "-ex", "detach",
  ]
  returncode = None
  timed_out = False
  capture_error = ""

  try:
    result = subprocess.run(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, errors="replace", timeout=timeout, check=False)
    output = result.stdout
    returncode = result.returncode
  except subprocess.TimeoutExpired as e:
    timed_out = True
    output = _decode_subprocess_output(e.stdout)
    capture_error = f"gdb exceeded {timeout:.1f}s timeout"
  except OSError as e:
    output = ""
    capture_error = f"{type(e).__name__}: {e}"

  encoded_output = output.encode(errors="replace")
  output_truncated = len(encoded_output) > UI_WATCHDOG_GDB_MAX_OUTPUT
  if output_truncated:
    encoded_output = encoded_output[:UI_WATCHDOG_GDB_MAX_OUTPUT]
    output = encoded_output.decode(errors="replace")

  useful = "#0" in output

  captured_at = time.strftime("%Y-%m-%d %H:%M:%S %z")
  path_root, path_ext = os.path.splitext(output_path)
  history_path = f"{path_root}.{time.strftime('%Y%m%d-%H%M%S')}.{pid}{path_ext or '.log'}"
  header = "".join((
    f"UI watchdog GDB capture at {captured_at}\n",
    f"pid={pid} returncode={returncode} timed_out={timed_out} error={capture_error or 'none'}\n",
    f"output_truncated={output_truncated} useful={useful}\n\n",
  ))
  try:
    temp_history_path = f"{history_path}.tmp"
    with open(temp_history_path, "w", errors="replace") as f:
      f.write(header)
      f.write(output)
    os.replace(temp_history_path, history_path)

    temp_path = f"{output_path}.tmp"
    with open(temp_path, "w", errors="replace") as f:
      f.write(header)
      f.write(output)
    os.replace(temp_path, output_path)

    history_pattern = f"{glob.escape(path_root)}.*{glob.escape(path_ext or '.log')}"
    history_files = sorted(glob.glob(history_pattern), key=os.path.getmtime, reverse=True)
    for stale_path in history_files[UI_WATCHDOG_GDB_HISTORY_LIMIT:]:
      os.unlink(stale_path)

    if useful:
      temp_success_path = f"{success_path}.tmp"
      with open(temp_success_path, "w") as f:
        f.write(history_path)
      os.replace(temp_success_path, success_path)
  except OSError as e:
    write_error = f"{type(e).__name__}: {e}"
    capture_error = f"{capture_error}; {write_error}" if capture_error else write_error

  return {
    "gdb_path": history_path,
    "gdb_returncode": returncode,
    "gdb_timed_out": timed_out,
    "gdb_output_bytes": len(encoded_output),
    "gdb_output_truncated": output_truncated,
    "gdb_capture_time": round(time.monotonic() - started, 4),
    "gdb_error": capture_error,
    "gdb_useful": useful,
    "gdb_skipped": False,
  }


class UiWatchdogCapture:
  def __init__(self, pid: int, watchdog_dt: float):
    self.pid = pid
    self.watchdog_dt = watchdog_dt
    self.result: dict | None = None
    self.thread = threading.Thread(target=self._run, name=f"ui-watchdog-gdb-{pid}", daemon=True)

  def _run(self) -> None:
    try:
      self.result = capture_ui_gdb_backtrace(self.pid)
    except Exception as e:
      self.result = {
        "gdb_path": UI_WATCHDOG_GDB_PATH,
        "gdb_returncode": None,
        "gdb_timed_out": False,
        "gdb_output_bytes": 0,
        "gdb_output_truncated": False,
        "gdb_capture_time": 0.0,
        "gdb_error": f"{type(e).__name__}: {e}",
      }

  def start(self) -> None:
    self.thread.start()


def launcher(proc: str, name: str) -> None:
  try:
    # import the process
    mod = importlib.import_module(proc)

    # rename the process
    setproctitle(proc)

    # create new context since we forked
    messaging.reset_context()

    # add daemon name tag to logs
    cloudlog.bind(daemon=name)
    sentry.set_tag("daemon", name)

    # exec the process
    mod.main()
  except KeyboardInterrupt:
    cloudlog.warning(f"child {proc} got SIGINT")
  except Exception:
    # can't install the crash handler because sys.excepthook doesn't play nice
    # with threads, so catch it here.
    sentry.capture_exception()
    raise


def nativelauncher(pargs: list[str], cwd: str, name: str) -> None:
  os.environ['MANAGER_DAEMON'] = name

  # exec the process
  os.chdir(cwd)
  os.execvp(pargs[0], pargs)


def join_process(process: Process, timeout: float) -> None:
  # Process().join(timeout) will hang due to a python 3 bug: https://bugs.python.org/issue28382
  # We have to poll the exitcode instead
  t = time.monotonic()
  while time.monotonic() - t < timeout and process.exitcode is None:
    time.sleep(0.001)


class ManagerProcess(ABC):
  daemon = False
  sigkill = False
  should_run: Callable[[bool, Params, car.CarParams, SimpleNamespace], bool]
  proc: Process | None = None
  enabled = True
  name = ""

  last_watchdog_time = 0
  watchdog_max_dt: int | None = None
  watchdog_seen = False
  ui_watchdog_capture: UiWatchdogCapture | None = None
  shutting_down = False

  @abstractmethod
  def prepare(self) -> None:
    pass

  @abstractmethod
  def start(self) -> None:
    pass

  def restart(self) -> None:
    self.stop(sig=signal.SIGKILL)
    self.start()

  def check_watchdog(self, started: bool) -> None:
    capture = self.ui_watchdog_capture
    if capture is not None:
      if capture.thread.is_alive():
        return

      capture.thread.join()
      self.ui_watchdog_capture = None
      gdb_diagnostics = capture.result or {
        "gdb_path": UI_WATCHDOG_GDB_PATH,
        "gdb_error": "capture worker exited without a result",
      }
      cloudlog.event("watchdog_gdb_backtrace", process=self.name, watchdog_dt=round(capture.watchdog_dt, 3), error=True, **gdb_diagnostics)
      if self.proc is not None and self.proc.pid == capture.pid and not self.shutting_down:
        self.restart()
      return

    if self.watchdog_max_dt is None or self.proc is None:
      return

    try:
      fn = WATCHDOG_FN + str(self.proc.pid)
      with open(fn, "rb") as f:
        self.last_watchdog_time = struct.unpack('Q', f.read())[0]
    except Exception:
      pass

    dt = time.monotonic() - self.last_watchdog_time / 1e9

    if dt > self.watchdog_max_dt:
      if self.watchdog_seen and ENABLE_WATCHDOG:
        cloudlog.error(f"Watchdog timeout for {self.name} (exitcode {self.proc.exitcode}) restarting ({started=})")
        diagnostics = get_process_diagnostics(self.proc.pid)
        cloudlog.event("watchdog_process_diagnostics", process=self.name, watchdog_dt=round(dt, 3), error=True, **diagnostics)
        if self.name == "ui":
          self.ui_watchdog_capture = UiWatchdogCapture(self.proc.pid, dt)
          self.ui_watchdog_capture.start()
        else:
          self.restart()
    else:
      self.watchdog_seen = True

  def stop(self, retry: bool = True, block: bool = True, sig: signal.Signals = None) -> int | None:
    if self.proc is None:
      return None

    if self.proc.exitcode is None:
      if not self.shutting_down:
        cloudlog.info(f"killing {self.name}")
        if sig is None:
          sig = signal.SIGKILL if self.sigkill else signal.SIGINT
        self.signal(sig)
        self.shutting_down = True

        if not block:
          return None

      join_process(self.proc, 5)

      # If process failed to die send SIGKILL
      if self.proc.exitcode is None and retry:
        cloudlog.info(f"killing {self.name} with SIGKILL")
        self.signal(signal.SIGKILL)
        self.proc.join()

    ret = self.proc.exitcode
    cloudlog.info(f"{self.name} is dead with {ret}")

    if self.proc.exitcode is not None:
      self.shutting_down = False
      self.proc = None

    return ret

  def signal(self, sig: int) -> None:
    if self.proc is None:
      return

    # Don't signal if already exited
    if self.proc.exitcode is not None and self.proc.pid is not None:
      return

    # Can't signal if we don't have a pid
    if self.proc.pid is None:
      return

    cloudlog.info(f"sending signal {sig} to {self.name}")
    os.kill(self.proc.pid, sig)

  def get_process_state_msg(self):
    state = log.ManagerState.ProcessState.new_message()
    state.name = self.name
    proc = self.proc  # called from the managerState heartbeat thread while the main loop can clear self.proc
    if proc:
      state.running = proc.is_alive()
      state.shouldBeRunning = not self.shutting_down
      state.pid = proc.pid or 0
      state.exitCode = proc.exitcode or 0
    return state

  def _clear_dead_proc(self) -> None:
    if self.proc is not None and self.proc.exitcode is not None:
      cloudlog.warning(f"{self.name} exited with {self.proc.exitcode}, clearing stale process handle")
      self.proc = None
      self.shutting_down = False


class NativeProcess(ManagerProcess):
  def __init__(self, name, cwd, cmdline, should_run, enabled=True, sigkill=False, watchdog_max_dt=None):
    self.name = name
    self.cwd = cwd
    self.cmdline = cmdline
    self.should_run = should_run
    self.enabled = enabled
    self.sigkill = sigkill
    self.watchdog_max_dt = watchdog_max_dt
    self.launcher = nativelauncher

  def prepare(self) -> None:
    pass

  def start(self) -> None:
    # In case we only tried a non blocking stop we need to stop it before restarting
    if self.shutting_down:
      self.stop()

    self._clear_dead_proc()

    if self.proc is not None:
      return

    cwd = os.path.join(BASEDIR, self.cwd)
    cloudlog.info(f"starting process {self.name}")
    self.proc = Process(name=self.name, target=self.launcher, args=(self.cmdline, cwd, self.name))
    self.proc.start()
    self.watchdog_seen = False
    self.shutting_down = False


class PythonProcess(ManagerProcess):
  def __init__(self, name, module, should_run, enabled=True, sigkill=False, watchdog_max_dt=None):
    self.name = name
    self.module = module
    self.should_run = should_run
    self.enabled = enabled
    self.sigkill = sigkill
    self.watchdog_max_dt = watchdog_max_dt
    self.launcher = launcher

  def prepare(self) -> None:
    if self.enabled:
      cloudlog.info(f"preimporting {self.module}")
      importlib.import_module(self.module)

  def start(self) -> None:
    # In case we only tried a non blocking stop we need to stop it before restarting
    if self.shutting_down:
      self.stop()

    self._clear_dead_proc()

    if self.proc is not None:
      return

    # TODO: this is just a workaround for this tinygrad check:
    # https://github.com/tinygrad/tinygrad/blob/ac9c96dae1656dc220ee4acc39cef4dd449aa850/tinygrad/device.py#L26
    name = self.name if "modeld" not in self.name else "MainProcess"

    cloudlog.info(f"starting python {self.module}")
    self.proc = Process(name=name, target=self.launcher, args=(self.module, self.name))
    self.proc.start()
    self.watchdog_seen = False
    self.shutting_down = False


class DaemonProcess(ManagerProcess):
  """Python process that has to stay running across manager restart.
  This is used for athena so you don't lose SSH access when restarting manager."""
  def __init__(self, name, module, param_name, enabled=True):
    self.name = name
    self.module = module
    self.param_name = param_name
    self.enabled = enabled
    self.params = None

  @staticmethod
  def should_run(started, params, CP, frogpilot_toggles):
    return True

  def prepare(self) -> None:
    pass

  def start(self) -> None:
    if self.params is None:
      self.params = Params()

    pid = self.params.get(self.param_name)
    if pid is not None:
      try:
        os.kill(int(pid), 0)
        with open(f'/proc/{pid}/cmdline') as f:
          if self.module in f.read():
            # daemon is running
            return
      except (OSError, FileNotFoundError):
        # process is dead
        pass

    cloudlog.info(f"starting daemon {self.name}")
    proc = subprocess.Popen(['python', '-m', self.module],
                               stdin=open('/dev/null'),
                               stdout=open('/dev/null', 'w'),
                               stderr=open('/dev/null', 'w'),
                               preexec_fn=os.setpgrp)

    self.params.put(self.param_name, proc.pid)

  def stop(self, retry=True, block=True, sig=None) -> None:
    pass


def ensure_running(procs: ValuesView[ManagerProcess], started: bool, params=None, CP: car.CarParams=None,
                   not_run: list[str] | None=None, frogpilot_toggles: SimpleNamespace=None) -> list[ManagerProcess]:
  if not_run is None:
    not_run = []

  running = []
  for p in procs:
    if p.enabled and p.name not in not_run and p.should_run(started, params, CP, frogpilot_toggles):
      running.append(p)
    else:
      p.stop(block=False)

    p.check_watchdog(started)

  for p in running:
    p.start()

  return running
