#!/usr/bin/env python3
import json
import subprocess
import time

from multiprocessing import Process
from pathlib import Path

from cereal import messaging
from openpilot.common.basedir import BASEDIR
from openpilot.common.constants import CV
from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog
from openpilot.common.time_helpers import system_time_valid
from openpilot.system.athena.registration import register
from openpilot.system.hardware import HARDWARE, PC

from openpilot.frogpilot.assets.theme_manager import ThemeManager
from openpilot.frogpilot.common import frogpilot_backups, frogpilot_paths, frogpilot_utilities, frogpilot_variables


def boot_backup(build_metadata):
  while not system_time_valid():
    time.sleep(1)

  frogpilot_backups.backup_frogpilot(build_metadata, Params())


def cleanup_screen_recordings(limit_bytes):
  recordings = sorted(frogpilot_paths.SCREEN_RECORDINGS_PATH.glob("*.mp4"), key=lambda recording: recording.stat().st_mtime, reverse=True)

  total = 0
  for index, recording in enumerate(recordings):
    total += recording.stat().st_size
    if index == 0 or total <= limit_bytes:
      continue

    for suffix in frogpilot_variables.SCREEN_RECORDING_COMPANION_SUFFIXES:
      frogpilot_utilities.delete_path(recording.with_suffix(suffix), report=False)
    frogpilot_utilities.delete_path(recording, report=False)


def download_maps(locations, cancel_download):
  pm = messaging.PubMaster(["mapdIn"])
  sm = messaging.SubMaster(["mapdExtendedOut"])

  time.sleep(1)

  msg = messaging.new_message("mapdIn")
  msg.mapdIn.type = "download"
  msg.mapdIn.str = locations
  pm.send("mapdIn", msg)

  download_requested_at = time.monotonic()

  started = False

  while True:
    sm.update(1000)

    if cancel_download.is_set():
      msg = messaging.new_message("mapdIn")
      msg.mapdIn.type = "cancelDownload"
      pm.send("mapdIn", msg)

      return False

    if sm.updated["mapdExtendedOut"]:
      progress = sm["mapdExtendedOut"].downloadProgress

      if progress.active:
        started = True

      if not progress.active and started:
        return not progress.cancelled and progress.downloadedFiles == progress.totalFiles > 0

    if not started and time.monotonic() - download_requested_at >= 10:
      return False


def frogpilot_boot_functions(build_metadata, params):
  if params.get("MapdSettings") == {}:
    params.put("MapdSettings", params.get_default_value("MapdSettings"))

  maps_selected = params.get("MapsSelected")
  if maps_selected:
    try:
      data = json.loads(maps_selected)
      if isinstance(data, dict):
        new_items = []
        for nation in data.get("nations", []):
          new_items.append(f"nation.{nation}")
        for state in data.get("states", []):
          new_items.append(f"us_state.{state}")
        new_items.sort()
        params.put("MapsSelected", ",".join(new_items))
    except json.JSONDecodeError:
      pass

  if params.get("TetheringEnabled") == 3:
    params.remove("TetheringEnabled")

  frogpilot_toggles = frogpilot_variables.get_frogpilot_toggles()

  flag_files_changed = sync_flag_file(frogpilot_paths.HD_PATH, frogpilot_toggles.use_higher_bitrate)
  flag_files_changed |= sync_flag_file(frogpilot_paths.KONIK_PATH, frogpilot_toggles.use_konik_server)

  if flag_files_changed:
    HARDWARE.reboot()

  ThemeManager.copy_default_theme()
  ThemeManager(params).update_active_theme(time_validated=system_time_valid(), frogpilot_toggles=frogpilot_toggles, boot_run=True)

  if frogpilot_utilities.use_konik_server():
    if params.get("KonikDongleId") is not None:
      params.put("DongleId", params.get("KonikDongleId"))
    else:
      Process(target=register_konik, daemon=True).start()
  elif params.get("KonikDongleId") is not None and params.get("DongleId") == params.get("KonikDongleId"):
    if params.get("StockDongleId") is None:
      params.remove("DongleId")
    else:
      params.put("DongleId", params.get("StockDongleId"))

  frogpilot_utilities.delete_path(frogpilot_paths.RESTORE_TEMP_PATH)

  Process(target=boot_backup, args=(build_metadata,), daemon=True).start()


def install_frogpilot(media_ok):
  paths = [
    frogpilot_paths.ERROR_LOGS_PATH
  ]
  if media_ok:
    paths += [frogpilot_paths.HD_LOGS_PATH, frogpilot_paths.KONIK_LOGS_PATH, frogpilot_paths.SCREEN_RECORDINGS_PATH]
  for path in paths:
    path.mkdir(parents=True, exist_ok=True)

  # the recordings live on the NVMe: never scan it after the guard's fallback, and never let the scan stop manager start
  if media_ok:
    try:
      cleanup_screen_recordings(10 * 1024 * 1024 * 1024)
    except OSError:
      cloudlog.exception("screen recording cleanup failed")

  update_boot_logo(frogpilot_paths.FROGPILOT_BOOT_LOGO_PATH)


def migrate_params(params, params_cache):
  param_types = {
    "MaxLateralAcceleration": dict,
  }

  for key, expected_type in param_types.items():
    for param_store in (params, params_cache):
      value = param_store.get(key)

      if value is not None and not isinstance(value, expected_type):
        param_store.remove(key)

  if params.get_bool("IsMetric"):
    metric_conversions = {
      "CESignalSpeed": (CV.MPH_TO_KPH, 0),
      "LaneLinesWidth": (CV.INCH_TO_CM, 0),
      "MinimumLaneChangeSpeed": (CV.MPH_TO_KPH, 0),
      "Offset1": (CV.MPH_TO_KPH, 0),
      "Offset2": (CV.MPH_TO_KPH, 0),
      "Offset3": (CV.MPH_TO_KPH, 0),
      "Offset4": (CV.MPH_TO_KPH, 0),
      "Offset5": (CV.MPH_TO_KPH, 0),
      "Offset6": (CV.MPH_TO_KPH, 0),
      "Offset7": (CV.MPH_TO_KPH, 0),
      "PathWidth": (CV.FOOT_TO_METER, 1),
      "RoadEdgesWidth": (CV.INCH_TO_CM, 0),
    }

    for key, (conversion, decimals) in metric_conversions.items():
      if params.get(key) is None and params_cache.get(key) is None:
        params.put(key, round(params.get_default_value(key) * conversion, decimals))


def register_konik():
  Params().put("KonikDongleId", register(register_konik=True))


def run_frogsgomoo(build_metadata):
  if frogpilot_utilities.is_FrogsGoMoo_development(build_metadata):
    frogpilot_utilities.run_cmd_on_writable_mount("/persist", ["sudo", "python3", frogpilot_paths.FROGS_GO_MOO_PATH], "Failed to run frogsgomoo.py")


def soft_reboot():
  Path("/tmp/booted").touch()

  subprocess.check_call(["sudo", "systemctl", "restart", "--no-block", "comma"])


def sync_flag_file(flag_path, enabled):
  if flag_path.is_file() == enabled:
    return False

  if enabled:
    flag_path.touch()
  else:
    flag_path.unlink()

  return True


def sync_params_cache(params, params_cache):
  for key in params.all_keys():
    current_value = params.get(key)
    cached_value = params_cache.get(key)

    if current_value is None:
      if cached_value is not None:
        params.put(key, cached_value)
    elif current_value != cached_value:
      params_cache.put(key, current_value)


def uninstall_frogpilot():
  update_boot_logo(frogpilot_paths.STOCK_BOOT_LOGO_PATH)

  HARDWARE.uninstall()


def update_boot_logo(target_logo):
  if PC:
    return

  if frogpilot_paths.BOOT_LOGO_PATH.read_bytes() != target_logo.read_bytes():
    frogpilot_utilities.run_cmd_on_writable_mount("/", ["sudo", "cp", target_logo, frogpilot_paths.BOOT_LOGO_PATH], "Failed to replace boot logo")


def update_maps(now, params, cancel_download, sm, manual_update=False):
  cancel_download.clear()

  if sm["deviceState"].networkMetered and not manual_update:
    return

  maps_selected = params.get("MapsSelected")
  if not maps_selected:
    return

  now = now.astimezone()

  day = now.day
  is_first = day == 1
  is_sunday = now.weekday() == 6
  schedule = params.get("PreferredSchedule")

  last_maps_update = params.get("LastMapsUpdate")
  maps_downloaded = frogpilot_paths.MAPS_PATH.exists() and bool(last_maps_update)

  if maps_downloaded and (schedule == 0 or (schedule == 1 and not is_sunday) or (schedule == 2 and not is_first)) and not manual_update:
    return

  suffix = "th" if 11 <= day <= 13 else {1: "st", 2: "nd", 3: "rd"}.get(day % 10, "th")
  todays_date = now.strftime(f"%B {day}{suffix}, %Y")

  if maps_downloaded and last_maps_update == todays_date and not manual_update:
    return

  frogpilot_utilities.delete_path(frogpilot_paths.MAPD_DOWNLOAD_MENU_PATH, report=False)

  if download_maps(maps_selected, cancel_download):
    params.put("LastMapsUpdate", todays_date)


def update_openpilot():
  full_update_script = Path("/data/openpilot/fullupdate.sh")
  if not full_update_script.is_file():
    return

  try:
    current_branch = subprocess.check_output(
      ["git", "symbolic-ref", "--quiet", "--short", "HEAD"], cwd=BASEDIR, stderr=subprocess.DEVNULL, text=True
    ).strip()
    local_commit = subprocess.check_output(
      ["git", "rev-parse", "HEAD"], cwd=BASEDIR, stderr=subprocess.DEVNULL, text=True
    ).strip()

    try:
      upstream_ref = subprocess.check_output(
        ["git", "rev-parse", "--abbrev-ref", "--symbolic-full-name", "@{u}"], cwd=BASEDIR, stderr=subprocess.DEVNULL, text=True
      ).strip()
      remote_name, remote_branch = upstream_ref.split("/", 1)
    except (subprocess.CalledProcessError, ValueError):
      remote_name = "origin"
      remote_branch = current_branch

    remote_output = subprocess.check_output(
      ["git", "ls-remote", remote_name, f"refs/heads/{remote_branch}"], cwd=BASEDIR, stderr=subprocess.DEVNULL, text=True
    ).strip()
  except (OSError, subprocess.CalledProcessError) as exception:
    print(f"Automatic full update check failed: {exception}")
    return

  if not remote_output:
    return

  remote_commit = remote_output.split(maxsplit=1)[0]
  if local_commit == remote_commit:
    return

  print(f"Full update available: {current_branch} {local_commit[:10]} -> {remote_commit[:10]}")
  try:
    with open("/data/fullupdate.log", "ab", buffering=0) as log_file:
      subprocess.Popen(
        [str(full_update_script)],
        cwd=BASEDIR,
        stdin=subprocess.DEVNULL,
        stdout=log_file,
        stderr=subprocess.STDOUT,
        start_new_session=True,
      )
  except OSError as exception:
    print(f"Failed to start automatic full update: {exception}")
