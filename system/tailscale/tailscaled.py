#!/usr/bin/env python3

import os
import select
import subprocess
import time
from pathlib import Path

from openpilot.common.swaglog import cloudlog

TAILSCALE_DIR = "/data/media/0/tailscale"
BIN_DIR = f"{TAILSCALE_DIR}/bin"
STATE_DIR = f"{TAILSCALE_DIR}/state"
SOCKET_PATH = f"{TAILSCALE_DIR}/tailscaled.sock"
STATE_PATH = f"{STATE_DIR}/tailscaled.state"
ENABLED_MARKER = f"{TAILSCALE_DIR}/enabled"
TAILSCALE_BIN = f"{BIN_DIR}/tailscale"
TAILSCALED_BIN = f"{BIN_DIR}/tailscaled"


def should_run() -> tuple[bool, str]:
  if not os.path.exists(ENABLED_MARKER):
    return False, f"missing marker: {ENABLED_MARKER}"
  if not os.path.exists(TAILSCALED_BIN):
    return False, f"missing binary: {TAILSCALED_BIN}"
  return True, ""


def tailscaled_ready() -> bool:
  if not os.path.exists(TAILSCALE_BIN):
    return False

  cmd = [TAILSCALE_BIN, "--socket", SOCKET_PATH, "status"]
  if os.geteuid() != 0:
    cmd = ["sudo", "-n", *cmd]

  try:
    proc = subprocess.run(cmd, capture_output=True, text=True, timeout=10, check=False)
  except (subprocess.SubprocessError, OSError):
    return False

  status_out = f"{proc.stdout}\n{proc.stderr}"
  return proc.returncode == 0 or (proc.returncode == 1 and "Logged out." in status_out)


def main():
  run_ok, reason = should_run()
  if not run_ok:
    cloudlog.info("Not starting tailscaled: %s", reason)
    return

  if tailscaled_ready():
    cloudlog.info("tailscaled is already running")
    while True:
      time.sleep(30)
      run_ok, reason = should_run()
      if not run_ok:
        cloudlog.info("Stopping tailscaled monitor: %s", reason)
        return
      if not tailscaled_ready():
        cloudlog.info("tailscaled is no longer ready")
        break

  Path(STATE_DIR).mkdir(parents=True, exist_ok=True)

  cmd = [
    TAILSCALED_BIN,
    "--tun=tailscale0",
    f"--state={STATE_PATH}",
    f"--statedir={STATE_DIR}",
    f"--socket={SOCKET_PATH}",
  ]
  if os.geteuid() != 0:
    cmd = ["sudo", "-n", *cmd]
  cloudlog.info("Starting tailscaled with command: %s", " ".join(cmd))

  proc = None
  try:
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)

    while True:
      ret = proc.poll()
      if ret is not None:
        if ret != 0:
          cloudlog.error("tailscaled exited with code %d", ret)
        else:
          cloudlog.info("tailscaled exited cleanly")
        break

      run_ok, reason = should_run()
      if not run_ok:
        cloudlog.info("Stopping tailscaled: %s", reason)
        proc.terminate()
        break

      if proc.stdout is not None:
        ready, _, _ = select.select([proc.stdout], [], [], 1)
        if ready:
          line = proc.stdout.readline().strip()
          if line:
            cloudlog.info("tailscaled: %s", line)
      else:
        time.sleep(1)
  except Exception:
    cloudlog.exception("tailscaled.exception")
  finally:
    if proc is not None and proc.poll() is None:
      proc.terminate()
      try:
        proc.wait(timeout=5)
      except subprocess.TimeoutExpired:
        cloudlog.warning("tailscaled did not terminate cleanly, killing it")
        proc.kill()


if __name__ == "__main__":
  main()
