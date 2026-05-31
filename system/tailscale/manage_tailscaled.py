#!/usr/bin/env python3

import os
import time
from multiprocessing import Process

from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog
from openpilot.system.manager.process import launcher

TAILSCALED_MGR_PID_PARAM = "TailscaledPid"
TAILSCALE_DIR = "/data/media/0/tailscale"
ENABLED_MARKER = f"{TAILSCALE_DIR}/enabled"
TAILSCALE_BIN = f"{TAILSCALE_DIR}/bin/tailscale"
TAILSCALED_BIN = f"{TAILSCALE_DIR}/bin/tailscaled"


def get_blockers(params: Params) -> str:
  blockers = []
  if not params.get_bool("SshEnabled"):
    blockers.append("SshEnabled is false")
  if not os.path.exists(ENABLED_MARKER):
    blockers.append(f"missing marker: {ENABLED_MARKER}")
  if not os.path.exists(TAILSCALE_BIN):
    blockers.append(f"missing binary: {TAILSCALE_BIN}")
  if not os.path.exists(TAILSCALED_BIN):
    blockers.append(f"missing binary: {TAILSCALED_BIN}")
  return "; ".join(blockers)


def main():
  params = Params()
  last_blocker_state = None

  try:
    while True:
      blocker_state = get_blockers(params)
      if blocker_state:
        if blocker_state != last_blocker_state:
          cloudlog.info("Not starting tailscaled: %s", blocker_state)
          last_blocker_state = blocker_state
        time.sleep(10)
        continue

      last_blocker_state = None
      cloudlog.info("Starting tailscaled daemon")
      proc = Process(name="tailscaled", target=launcher, args=("system.tailscale.tailscaled", "tailscaled"))
      proc.start()
      proc.join()
      cloudlog.event("tailscaled exited", exitcode=proc.exitcode)
      time.sleep(10)
  except Exception:
    cloudlog.exception("manage_tailscaled.exception")
  finally:
    params.remove(TAILSCALED_MGR_PID_PARAM)


if __name__ == "__main__":
  main()
