#!/usr/bin/env python3

import os
import sys
import subprocess
import time
from pathlib import Path

from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog
from openpilot.system.hardware import HARDWARE

# Check and install websockets if needed
try:
  import websockets
except ImportError:
  cloudlog.info("websockets module not found, installing...")
  try:
    subprocess.check_call([sys.executable, "-m", "pip", "install", "websockets"])
    cloudlog.info("websockets installed successfully")
  except subprocess.CalledProcessError as e:
    cloudlog.error("Failed to install websockets: %s", e)
    sys.exit(1)


def main():
  """Main entry point for sshreachd service"""
  params = Params()
  
  # Check if SSH is enabled
  if not params.get_bool("SshEnabled"):
    cloudlog.info("SSH disabled, exiting sshreachd")
    return
  
  # Set up paths
  private_key_path = Path("/data/media/0/sshreach/id_ed25519")
  sshreach_script = Path(__file__).parent / "sshreach_client.py"
  
  # Verify private key exists
  if not private_key_path.exists():
    cloudlog.error("SSH private key not found at %s", private_key_path)
    cloudlog.error("Please run the install_sshreach.sh script to set up SSH Reach")
    return
  
  # Verify sshreach script exists
  if not sshreach_script.exists():
    cloudlog.error("SSHReach client script not found at %s", sshreach_script)
    return
  
  # Wait for network
  while not HARDWARE.get_network_type():
    cloudlog.info("Waiting for network connection...")
    time.sleep(5)
  
  cloudlog.info("Network available, starting SSHReach client")
  
  # Start the SSH reach client
  # Pass the private key path and enable console logging for debugging
  cmd = [sys.executable, str(sshreach_script), str(private_key_path), "console"]
  
  cloudlog.info("Starting SSHReach with command: %s", " ".join(cmd))
  
  try:
    # Run the SSH reach client
    # It will handle its own reconnection logic
    proc = subprocess.Popen(cmd)
    
    # Monitor the process
    while True:
      ret = proc.poll()
      if ret is not None:
        cloudlog.error("SSHReach client exited with code %d", ret)
        break
      
      # Check if SSH is still enabled
      if not params.get_bool("SshEnabled"):
        cloudlog.info("SSH disabled, stopping sshreachd")
        proc.terminate()
        break
        
      # Check network status
      if not HARDWARE.get_network_type():
        cloudlog.warning("Network connection lost")
      
      time.sleep(10)
      
  except Exception as e:
    cloudlog.exception("Error running SSHReach client: %s", e)
    if proc and proc.poll() is None:
      proc.terminate()


if __name__ == "__main__":
  main()