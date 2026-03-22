#!/usr/bin/env python3

import os
import select
import subprocess
import time
from pathlib import Path

from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog
from openpilot.system.hardware import HARDWARE

TAILSCALE_DIR = "/data/media/0/tailscale"
BIN_DIR = f"{TAILSCALE_DIR}/bin"
STATE_DIR = f"{TAILSCALE_DIR}/state"
SOCKET_PATH = f"{TAILSCALE_DIR}/tailscaled.sock"
STATE_PATH = f"{STATE_DIR}/tailscaled.state"
ENABLED_MARKER = f"{TAILSCALE_DIR}/enabled"
TAILSCALED_BIN = f"{BIN_DIR}/tailscaled"


def should_run(params: Params) -> tuple[bool, str]:
  if not params.get_bool("SshEnabled"):
    return False, "SshEnabled is false"
  if not os.path.exists(ENABLED_MARKER):
    return False, f"missing marker: {ENABLED_MARKER}"
  if not os.path.exists(TAILSCALED_BIN):
    return False, f"missing binary: {TAILSCALED_BIN}"
  return True, ""


def main():
  params = Params()
  run_ok, reason = should_run(params)
  if not run_ok:
    cloudlog.info("Not starting tailscaled: %s", reason)
    return

  Path(STATE_DIR).mkdir(parents=True, exist_ok=True)

  while not HARDWARE.get_network_type():
    run_ok, reason = should_run(params)
    if not run_ok:
      cloudlog.info("Stopping tailscaled before start: %s", reason)
      return
    cloudlog.info("Waiting for network connection before tailscaled start")
    time.sleep(5)

  cmd = [
    "sudo",
    TAILSCALED_BIN,
    "--tun=tailscale0",
    f"--state={STATE_PATH}",
    f"--statedir={STATE_DIR}",
    f"--socket={SOCKET_PATH}",
  ]
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

      run_ok, reason = should_run(params)
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
