import errno
import fcntl
import os
import sys
import pathlib
import shutil
import signal
import subprocess
import tempfile
import threading
import time

from openpilot.common.basedir import BASEDIR
from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog
from openpilot.system.hardware import HARDWARE

NVME_MOUNTPOINT = "/data/media"
NVME_DEVICE = "/dev/nvme0n1"
NVME_PCI_FUNCTION = "/sys/bus/pci/devices/0001:01:00.0"
PCI_RESCAN = "/sys/bus/pci/rescan"
NVME_MOUNT_UNIT = "data-media.mount"
NVME_FALLBACK_LOG_ROOT = "/data/realdata_fallback"
NVME_MOUNT_WAIT_S = 30
NVME_RESCAN_WAIT_S = 15

def unblock_stdout() -> None:
  # get a non-blocking stdout
  child_pid, child_pty = os.forkpty()
  if child_pid != 0:  # parent

    # child is in its own process group, manually pass kill signals
    signal.signal(signal.SIGINT, lambda signum, frame: os.kill(child_pid, signal.SIGINT))
    signal.signal(signal.SIGTERM, lambda signum, frame: os.kill(child_pid, signal.SIGTERM))

    fcntl.fcntl(sys.stdout, fcntl.F_SETFL, fcntl.fcntl(sys.stdout, fcntl.F_GETFL) | os.O_NONBLOCK)

    while True:
      try:
        dat = os.read(child_pty, 4096)
      except OSError as e:
        if e.errno == errno.EIO:
          break
        continue

      if not dat:
        break

      try:
        sys.stdout.write(dat.decode('utf8'))
      except (OSError, UnicodeDecodeError):
        pass

    # os.wait() returns a tuple with the pid and a 16 bit value
    # whose low byte is the signal number and whose high byte is the exit status
    exit_status = os.wait()[1] >> 8
    os._exit(exit_status)


def write_onroad_params(started, params):
  params.put_bool("IsOnroad", started)
  params.put_bool("IsOffroad", not started)


def ensure_media_storage() -> bool:
  if HARDWARE.get_device_type() != "tici" or os.environ.get("LOG_ROOT") or os.path.ismount(NVME_MOUNTPOINT):
    return True

  def run(step: str, cmd: list[str], **kwargs) -> None:
    try:
      result = subprocess.run(["sudo", "-n", *cmd], timeout=10, check=False, **kwargs).returncode
    except (subprocess.SubprocessError, OSError) as e:
      result = repr(e)
    cloudlog.event("nvme_guard", step=step, result=result)

  def wait_for_mount(step: str, wait_s: float) -> bool:
    deadline = time.monotonic() + wait_s
    while True:
      if os.path.exists(NVME_DEVICE):
        run(step, ["systemctl", "start", NVME_MOUNT_UNIT])
      if os.path.ismount(NVME_MOUNTPOINT):
        cloudlog.event("nvme_guard", step=step, mounted=True)
        return True
      if time.monotonic() >= deadline:
        return False
      time.sleep(1)

  cloudlog.event("nvme_guard", step="not_mounted", device_present=os.path.exists(NVME_DEVICE))
  if wait_for_mount("mount", NVME_MOUNT_WAIT_S):
    return True

  # a wedged controller can survive warm reboots; re-enumerating the PCIe function is the only software reset
  if os.path.isdir(NVME_PCI_FUNCTION):
    run("pci_remove", ["tee", os.path.join(NVME_PCI_FUNCTION, "remove")], input=b"1", stdout=subprocess.DEVNULL)
  run("pci_rescan", ["tee", PCI_RESCAN], input=b"1", stdout=subprocess.DEVNULL)
  if wait_for_mount("mount_after_rescan", NVME_RESCAN_WAIT_S):
    return True

  # never log under the bare mountpoint: those files would hide under the NVMe once it mounts again
  try:
    os.makedirs(NVME_FALLBACK_LOG_ROOT, exist_ok=True)
  except OSError as e:
    cloudlog.event("nvme_guard", step="fallback_mkdir", result=repr(e), error=True)
  os.environ["LOG_ROOT"] = NVME_FALLBACK_LOG_ROOT
  cloudlog.event("nvme_guard", step="fallback", log_root=NVME_FALLBACK_LOG_ROOT, error=True)
  return False


def save_bootlog():
  # copy current params
  tmp = tempfile.mkdtemp()
  params_dirname = pathlib.Path(Params().get_param_path()).name
  params_dir = os.path.join(tmp, params_dirname)
  shutil.copytree(Params().get_param_path(), params_dir, dirs_exist_ok=True)

  def fn(tmpdir):
    env = os.environ.copy()
    env['PARAMS_COPY_PATH'] = tmpdir
    subprocess.call("./bootlog", cwd=os.path.join(BASEDIR, "system/loggerd"), env=env)
    shutil.rmtree(tmpdir)
  t = threading.Thread(target=fn, args=(tmp, ))
  t.daemon = True
  t.start()
