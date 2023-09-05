#!/usr/bin/env python3
# PFEIFER - MAPD
import os
import sys
import errno
import shutil
import tarfile
import time
import traceback
from openpilot.common.basedir import BASEDIR
from openpilot.common.text_window import TextWindow
import openpilot.selfdrive.sentry as sentry
from urllib.request import urlopen
from glob import glob
import subprocess
import importlib.util

# NOTE: Do NOT import anything here that needs be built (e.g. params)
from openpilot.common.spinner import Spinner


sys.path.append(os.path.join(BASEDIR, "third_party/mapd"))
OVERPY_SPEC = importlib.util.find_spec('overpy')
MAX_BUILD_PROGRESS = 100
TMP_DIR = '/data/tmp'
THIRD_PARTY_DIR = '/data/openpilot/third_party/mapd'
THIRD_PARTY_DIR_SP = '/data/third_party_community'
PRELOADED_DEP_FILE = os.path.join(BASEDIR, "selfdrive/mapd/assets/mapd_deps.tar.xz")


def wait_for_internet_connection(return_on_failure=False):
  retries = 0
  while True:
    try:
      _ = urlopen('https://www.google.com/', timeout=10)
      return True
    except Exception as e:
      print(f'Wait for internet failed: {e}')
      if return_on_failure and retries == 15:
        return False
      retries += 1
      time.sleep(2)  # Wait for 2 seconds before retrying


def install_dep(spinner):
  wait_for_internet_connection()

  TOTAL_PIP_STEPS = 2986

  try:
    os.makedirs(TMP_DIR)
  except OSError as e:
    if e.errno != errno.EEXIST:
      raise
  my_env = os.environ.copy()
  my_env['TMPDIR'] = TMP_DIR

  pip_target = [f'--target={THIRD_PARTY_DIR}']
  packages = []
  if OVERPY_SPEC is None:
    packages.append('overpy==0.6')

  pip = subprocess.Popen([sys.executable, "-m", "pip", "install", "-v"] + pip_target + packages,
                          stdout=subprocess.PIPE, env=my_env)

  # Read progress from pip and update spinner
  steps = 0
  while True:
    output = pip.stdout.readline()
    if pip.poll() is not None:
      break
    if output:
      steps += 1
      spinner.update_progress(MAX_BUILD_PROGRESS * min(1., steps / TOTAL_PIP_STEPS), 100.)
      print(output.decode('utf8', 'replace'))

  shutil.rmtree(TMP_DIR)
  os.unsetenv('TMPDIR')

  dup = f'cp -rf {THIRD_PARTY_DIR} {THIRD_PARTY_DIR_SP}'
  process_dup = subprocess.Popen(dup, stdout=subprocess.PIPE, shell=True)


if __name__ == "__main__" and OVERPY_SPEC is None:
  spinner = Spinner()
  preload_fault = False
  try:
    if os.path.exists(PRELOADED_DEP_FILE):
      spinner.update("Loading preloaded dependencies")
      try:
        with tarfile.open(PRELOADED_DEP_FILE, "r:xz") as tar:
          for member in tar.getmembers():
            split_components = member.name.split('/')
            if len(split_components) > 1:
              member.name = '/'.join(split_components[1:])
            tar.extract(member, path=THIRD_PARTY_DIR)
        print(f"SP_LOG: Preloaded dependencies extracted to {THIRD_PARTY_DIR}")
      except Exception as e:
        preload_fault = True
        print(f"SP_LOG: An error occurred while extracting preloaded dependencies: {e}")
    if not os.path.exists(PRELOADED_DEP_FILE) or preload_fault:
      if os.path.exists(THIRD_PARTY_DIR_SP):
        spinner.update("Loading dependencies")
        command = f'rm -rf {THIRD_PARTY_DIR}; cp -rf {THIRD_PARTY_DIR_SP} {THIRD_PARTY_DIR}'
        process = subprocess.Popen(command, stdout=subprocess.PIPE, shell=True)
        print(f"SP_LOG: Removed directory {THIRD_PARTY_DIR}")
        print(f"SP_LOG: Copied {THIRD_PARTY_DIR_SP} to {THIRD_PARTY_DIR}")
      else:
        spinner.update("Waiting for internet")
        install_dep(spinner)
  except Exception:
    sentry.init(sentry.SentryProject.SELFDRIVE)
    traceback.print_exc()
    sentry.capture_exception()

    error = traceback.format_exc(-3)
    error = "Dependency Manager failed to start\n\n" + error
    with TextWindow(error) as t:
      t.wait_for_exit()
