#!/usr/bin/env python3

import os
import time
from multiprocessing import Process

from openpilot.common.params import Params
from openpilot.system.manager.process import launcher
from openpilot.common.swaglog import cloudlog
from openpilot.system.hardware import HARDWARE

SSHREACH_MGR_PID_PARAM = "SshreachdPid"


def main():
  params = Params()
  
  # Check if SSH reach is enabled
  ssh_enabled = params.get_bool("SshEnabled")
  if not ssh_enabled:
    cloudlog.info("SSH reach disabled, not starting sshreachd")
    return
  
  # Check if private key exists in persistent location
  private_key_path = "/data/media/0/sshreach/id_ed25519"
  if not os.path.exists(private_key_path):
    cloudlog.warning("SSH private key not found at %s, not starting sshreachd", private_key_path)
    cloudlog.warning("Please run the install_sshreach.sh script to set up SSH Reach")
    return
  
  try:
    while True:
      # Wait for network to be ready
      if not HARDWARE.get_network_type():
        cloudlog.info("Waiting for network connection...")
        time.sleep(10)
        continue
        
      cloudlog.info("Starting sshreach daemon")
      proc = Process(name='sshreachd', target=launcher, args=('system.sshreach.sshreachd', 'sshreachd'))
      proc.start()
      proc.join()
      cloudlog.event("sshreachd exited", exitcode=proc.exitcode)
      time.sleep(30)  # Wait 30 seconds before restarting
  except Exception:
    cloudlog.exception("manage_sshreachd.exception")
  finally:
    params.remove(SSHREACH_MGR_PID_PARAM)


if __name__ == '__main__':
  main()