#!/usr/bin/env python3
import datetime
import os
import signal
import string
import subprocess
import sys
import threading
import time
import traceback

from cereal import log
import cereal.messaging as messaging
import openpilot.system.sentry as sentry
from openpilot.common.basedir import BASEDIR
from openpilot.common.utils import atomic_write
from openpilot.common.params import Params, ParamKeyFlag
from openpilot.common.text_window import TextWindow
from openpilot.system.hardware import HARDWARE
from openpilot.system.manager.helpers import unblock_stdout, write_onroad_params, save_bootlog, ensure_media_storage
from openpilot.system.manager.process import ensure_running
from openpilot.system.manager.process_config import managed_processes
from openpilot.system.athena.registration import register, UNREGISTERED_DONGLE_ID
from openpilot.common.swaglog import cloudlog, add_file_handler
from openpilot.system.version import get_build_metadata, terms_version, training_version
from openpilot.system.hardware.hw import Paths
from openpilot.selfdrive.selfdrived.alertmanager import set_offroad_alert

from openpilot.frogpilot.common import frogpilot_functions, frogpilot_paths, frogpilot_variables

MANAGER_STATE_DT = 0.5  # managerState is a 2Hz service
POWER_WATCHDOG_DT = 0.5
MAIN_LOOP_STALL_LIMIT = 30.0  # stop heartbeating if the main loop has been stuck this long
FULLUPDATE_LOG = "/data/fullupdate.log"


def manager_init() -> None:
  media_ok = ensure_media_storage()
  save_bootlog()

  build_metadata = get_build_metadata()

  params = Params()
  params.clear_all(ParamKeyFlag.CLEAR_ON_MANAGER_START)
  params.clear_all(ParamKeyFlag.CLEAR_ON_ONROAD_TRANSITION)
  params.clear_all(ParamKeyFlag.CLEAR_ON_OFFROAD_TRANSITION)
  params.clear_all(ParamKeyFlag.CLEAR_ON_IGNITION_ON)
  if build_metadata.release_channel:
    params.clear_all(ParamKeyFlag.DEVELOPMENT_ONLY)
  set_offroad_alert("Offroad_StorageMissing", not media_ok)

  if params.get_bool("RecordFrontLock"):
    params.put_bool("RecordFront", True)

  # FrogPilot variables
  params_cache = Params(str(frogpilot_paths.PARAMS_CACHE_PATH))

  frogpilot_functions.migrate_params(params, params_cache)
  frogpilot_functions.sync_params_cache(params, params_cache)

  # set unset params to their default value
  for k in params.all_keys():
    default_value = params.get_default_value(k)
    if default_value is not None and params.get(k) is None:
      params.put(k, default_value)

  # Create folders needed for msgq
  try:
    os.mkdir(Paths.shm_path())
  except FileExistsError:
    pass
  except PermissionError:
    print(f"WARNING: failed to make {Paths.shm_path()}")

  # set params
  serial = HARDWARE.get_serial()
  params.put("Version", build_metadata.openpilot.version)
  params.put("TermsVersion", terms_version)
  params.put("TrainingVersion", training_version)
  params.put("GitCommit", build_metadata.openpilot.git_commit)
  params.put("GitCommitDate", build_metadata.openpilot.git_commit_date)
  params.put("GitBranch", build_metadata.channel)
  params.put("GitRemote", build_metadata.openpilot.git_origin)
  params.put_bool("IsTestedBranch", build_metadata.tested_channel)
  params.put_bool("IsReleaseBranch", build_metadata.release_channel)
  params.put("HardwareSerial", serial)

  # set dongle id
  reg_res = register(show_spinner=True)
  if reg_res:
    dongle_id = reg_res
  else:
    raise Exception(f"Registration failed for device {serial}")
  os.environ['DONGLE_ID'] = dongle_id  # Needed for swaglog
  os.environ['GIT_ORIGIN'] = build_metadata.openpilot.git_normalized_origin # Needed for swaglog
  os.environ['GIT_BRANCH'] = build_metadata.channel # Needed for swaglog
  os.environ['GIT_COMMIT'] = build_metadata.openpilot.git_commit # Needed for swaglog

  if not build_metadata.openpilot.is_dirty:
    os.environ['CLEAN'] = '1'

  # init logging
  sentry.init(sentry.SentryProject.SELFDRIVE)
  cloudlog.bind_global(dongle_id=dongle_id,
                       version=build_metadata.openpilot.version,
                       origin=build_metadata.openpilot.git_normalized_origin,
                       branch=build_metadata.channel,
                       commit=build_metadata.openpilot.git_commit,
                       dirty=build_metadata.openpilot.is_dirty,
                       device=HARDWARE.get_device_type())

  # FrogPilot variables
  frogpilot_functions.run_frogsgomoo(build_metadata)

  # preimport all processes
  for p in managed_processes.values():
    p.prepare()

  # FrogPilot variables
  frogpilot_functions.install_frogpilot(media_ok)
  frogpilot_functions.frogpilot_boot_functions(build_metadata, params)


def manager_cleanup() -> None:
  # send signals to kill all procs
  for p in managed_processes.values():
    p.stop(block=False)

  # ensure all are killed
  for p in managed_processes.values():
    p.stop(block=True)

  cloudlog.info("everything is dead")


def manager_thread() -> None:
  cloudlog.bind(daemon="manager")
  cloudlog.info("manager start")
  cloudlog.info({"environ": os.environ})

  params = Params()

  ignore: list[str] = []

  # FrogPilot variables
  dongle_id = params.get("DongleId")
  if dongle_id in (None, UNREGISTERED_DONGLE_ID) or not all(c in string.hexdigits for c in dongle_id):
    ignore += ["manage_athenad", "uploader"]
  if os.getenv("NOBOARD") is not None:
    ignore.append("pandad")
  ignore += [x for x in os.getenv("BLOCK", "").split(",") if len(x) > 0]

  sm = messaging.SubMaster(['deviceState', 'carParams', 'pandaStates'], poll='deviceState')
  pm = messaging.PubMaster(['managerState'])

  # FrogPilot variables
  sm = sm.extend(['frogpilotProcessState'])

  frogpilot_toggles = frogpilot_variables.get_frogpilot_toggles()

  write_onroad_params(False, params)
  ensure_running(managed_processes.values(), False, params=params, CP=sm['carParams'], not_run=ignore, frogpilot_toggles=frogpilot_toggles)

  started_prev = False
  ignition_prev = False

  # The main loop does filesystem I/O (params reads, pty writes) that can block for
  # seconds during kernel I/O stalls. Publish managerState and feed the power watchdog
  # from dedicated threads so a stalled loop doesn't read as a dead manager onroad.
  main_loop_alive_t = [time.monotonic()]
  stop_threads = threading.Event()
  process_state_lock = threading.Lock()

  def managerstate_heartbeat() -> None:
    while not stop_threads.is_set():
      if time.monotonic() - main_loop_alive_t[0] < MAIN_LOOP_STALL_LIMIT:
        with process_state_lock:
          msg = messaging.new_message('managerState', valid=True)
          msg.managerState.processes = [p.get_process_state_msg() for p in managed_processes.values()]
        pm.send('managerState', msg)
      stop_threads.wait(MANAGER_STATE_DT)

  def power_watchdog_feeder() -> None:
    # Keep the AGNOS power watchdog alive whenever manager is healthy enough to spin.
    # Some devices carry an external power_monitor.service that will power off the unit
    # if this file stops updating, regardless of FrogPilot's own shutdown timer.
    while not stop_threads.is_set():
      try:
        with atomic_write("/var/tmp/power_watchdog", "w", overwrite=True) as f:
          f.write(str(time.monotonic()))
      except Exception:
        pass
      stop_threads.wait(POWER_WATCHDOG_DT)

  threading.Thread(target=managerstate_heartbeat, name="managerstate_heartbeat", daemon=True).start()
  threading.Thread(target=power_watchdog_feeder, name="power_watchdog_feeder", daemon=True).start()

  while True:
    sm.update(1000)
    main_loop_alive_t[0] = time.monotonic()

    started = sm['deviceState'].started

    if started and not started_prev and not frogpilot_toggles.force_onroad:
      params.clear_all(ParamKeyFlag.CLEAR_ON_ONROAD_TRANSITION)
    elif not started and started_prev:
      params.clear_all(ParamKeyFlag.CLEAR_ON_OFFROAD_TRANSITION)

    ignition = any(ps.ignitionLine or ps.ignitionCan for ps in sm['pandaStates'] if ps.pandaType != log.PandaState.PandaType.unknown)
    if ignition and not ignition_prev:
      params.clear_all(ParamKeyFlag.CLEAR_ON_IGNITION_ON)

    # update onroad params, which drives pandad's safety setter thread
    if started != started_prev:
      write_onroad_params(started, params)

    started_prev = started
    ignition_prev = ignition

    with process_state_lock:
      ensure_running(managed_processes.values(), started, params=params, CP=sm['carParams'], not_run=ignore, frogpilot_toggles=frogpilot_toggles)

    running = ' '.join("{}{}\u001b[0m".format("\u001b[32m" if p.proc.is_alive() else "\u001b[31m", p.name)
                       for p in managed_processes.values() if p.proc)
    print(running)
    cloudlog.debug(running)

    # Exit main loop when uninstall/shutdown/reboot is needed
    shutdown = False
    for param in ("DoUninstall", "DoShutdown", "DoReboot", "DoSoftReboot"):
      if params.get_bool(param):
        if param in ("DoReboot", "DoSoftReboot") and (ignition or started):
          # A reboot or soft reboot asked for while the car is on (settings, The Pond, Soft Reboot) must not reach manager_cleanup:
          # stopping card and pandad kills the spoofed SCC stream and faults the cluster. fullupdate.sh __safe_reboot reboots only
          # after the verified stock-SCC handoff (or once off-road); every process keeps running until then, also if the handoff fails.
          params.remove(param)
          cloudlog.warning(f"{param} while the car is on: routed to fullupdate.sh __safe_reboot")
          try:
            with open(FULLUPDATE_LOG, "ab", buffering=0) as log_file:
              subprocess.Popen([os.path.join(BASEDIR, "fullupdate.sh"), "__safe_reboot"], cwd=BASEDIR, stdin=subprocess.DEVNULL, stdout=log_file,
                               stderr=subprocess.STDOUT, start_new_session=True)
          except OSError:
            # no raw fallback: the request is dropped, every process keeps running, and the driver can ask again
            cloudlog.exception(f"{param}: fullupdate.sh __safe_reboot did not start")
          continue
        shutdown = True
        params.put("LastManagerExitReason", f"{param} {datetime.datetime.now()}")
        cloudlog.warning(f"Shutting down manager - {param} set")

    if shutdown:
      break

    # FrogPilot variables
    frogpilot_toggles = frogpilot_variables.get_frogpilot_toggles(sm)

  stop_threads.set()


def main() -> None:
  manager_init()
  if os.getenv("PREPAREONLY") is not None:
    return

  # SystemExit on sigterm
  signal.signal(signal.SIGTERM, lambda signum, frame: sys.exit(1))

  try:
    manager_thread()
  except Exception:
    traceback.print_exc()
    sentry.capture_exception()
  finally:
    manager_cleanup()

  params = Params()
  if params.get_bool("DoUninstall"):
    cloudlog.warning("uninstalling")
    frogpilot_functions.uninstall_frogpilot()
  elif params.get_bool("DoReboot"):
    cloudlog.warning("reboot")
    HARDWARE.reboot()
  elif params.get_bool("DoShutdown"):
    cloudlog.warning("shutdown")
    HARDWARE.shutdown()
  # FrogPilot variables
  elif params.get_bool("DoSoftReboot"):
    cloudlog.warning("soft reboot")
    frogpilot_functions.soft_reboot()


if __name__ == "__main__":
  unblock_stdout()

  try:
    main()
  except KeyboardInterrupt:
    print("got CTRL-C, exiting")
  except Exception:
    add_file_handler(cloudlog)
    cloudlog.exception("Manager failed to start")

    try:
      managed_processes['ui'].stop()
    except Exception:
      pass

    # Show last 3 lines of traceback
    error = traceback.format_exc(-3)
    error = "Manager failed to start\n\n" + error
    with TextWindow(error) as t:
      t.wait_for_exit()

    raise

  # manual exit because we are forked
  sys.exit(0)
