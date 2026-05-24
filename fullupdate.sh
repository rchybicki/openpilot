#!/usr/bin/env bash
set -e

OPENPILOT_DIR="$(git rev-parse --show-toplevel 2>/dev/null || pwd)"
export PATH="/usr/comma/shims:/usr/local/venv/bin:$PATH"
if [ -f "$OPENPILOT_DIR/launch_env.sh" ]; then
  # shellcheck source=/dev/null
  source "$OPENPILOT_DIR/launch_env.sh"
fi
export PYTHONPATH="$OPENPILOT_DIR/frogpilot/third_party:$OPENPILOT_DIR${PYTHONPATH:+:$PYTHONPATH}"
PYTHON="${PYTHON:-python3}"

run_low_priority() {
  nice -n 10 "$@"
}

unsafe_update_reasons() {
  run_low_priority "$PYTHON" - <<'PY'
import time

import cereal.messaging as messaging
from openpilot.common.params import Params

params = Params()
reasons = []

if params.get_bool("IsOnroad"):
  reasons.append("openpilot is onroad")
if params.get_bool("IsEngaged"):
  reasons.append("openpilot is engaged")

sm = messaging.SubMaster(["pandaStates"])
end_t = time.monotonic() + 1.5
while time.monotonic() < end_t:
  sm.update(100)
  if sm.updated["pandaStates"]:
    ignition = any(ps.ignitionLine or ps.ignitionCan for ps in sm["pandaStates"])
    if ignition:
      reasons.append("vehicle ignition is on")
    break

print(", ".join(dict.fromkeys(reasons)))
PY
}

wait_for_onroad_controls_off() {
  run_low_priority "$PYTHON" - <<'PY'
import sys
import time

import cereal.messaging as messaging
from openpilot.common.params import Params

def show_device_alert(pm, text1, text2):
  msg = messaging.new_message("alertDebug")
  msg.alertDebug.alertText1 = text1
  msg.alertDebug.alertText2 = text2
  pm.send("alertDebug", msg)

def waiting_alert_text(blockers):
  if any("AOL" in blocker or "lateral" in blocker for blocker in blockers):
    return "Disable AOL and ACC main"
  if any("engaged" in blocker or "enabled" in blocker or "active" in blocker for blocker in blockers):
    return "Take control, turn ACC off"
  return "Turn ACC main off to reboot"

pm = messaging.PubMaster(["alertDebug"])
params = Params()
sm = messaging.SubMaster(["carState", "carControl", "frogpilotCarState", "pandaStates", "selfdriveState"])
last_blockers = None
alert_started = False

while True:
  sm.update(100)

  is_onroad = params.get_bool("IsOnroad")
  ignition = sm.seen["pandaStates"] and any(ps.ignitionLine or ps.ignitionCan for ps in sm["pandaStates"])
  if not is_onroad and not ignition:
    sys.exit(0)

  blockers = []
  if params.get_bool("IsEngaged"):
    blockers.append("openpilot engaged")

  if is_onroad and not sm.seen["selfdriveState"]:
    blockers.append("waiting for selfdrive state")
  elif sm.seen["selfdriveState"]:
    if sm["selfdriveState"].enabled:
      blockers.append("openpilot enabled")
    if sm["selfdriveState"].active:
      blockers.append("openpilot active")

  if is_onroad and not sm.seen["carControl"]:
    blockers.append("waiting for control state")
  elif sm.seen["carControl"]:
    if sm["carControl"].latActive:
      blockers.append("lateral/AOL active")
    if sm["carControl"].longActive:
      blockers.append("longitudinal active")

  if is_onroad and not sm.seen["frogpilotCarState"]:
    blockers.append("waiting for FrogPilot state")
  elif sm.seen["frogpilotCarState"] and sm["frogpilotCarState"].alwaysOnLateralEnabled:
    blockers.append("AOL active")

  if is_onroad and not sm.seen["carState"]:
    blockers.append("waiting for car state")
  elif sm.seen["carState"]:
    cs = sm["carState"]
    if cs.cruiseState.enabled:
      blockers.append("cruise engaged")
    if cs.cruiseState.available:
      blockers.append("ACC main on")

  blockers = list(dict.fromkeys(blockers))
  if not blockers:
    if alert_started:
      print("openpilot, AOL, and ACC main are off; continuing update/reboot.", flush=True)
    sys.exit(0)

  alert_started = True
  if blockers != last_blockers:
    print("Waiting for ACC/main/lateral to turn off: " + ", ".join(blockers), flush=True)
    last_blockers = blockers

  show_device_alert(pm, "Update Active", waiting_alert_text(blockers))
PY
}

show_update_staged_alert() {
  run_low_priority "$PYTHON" - <<'PY'
import time

import cereal.messaging as messaging

pm = messaging.PubMaster(["alertDebug"])
deadline = time.monotonic() + 5.0
while time.monotonic() < deadline:
  msg = messaging.new_message("alertDebug")
  msg.alertDebug.alertText1 = "Update Staged"
  msg.alertDebug.alertText2 = "Restart after parking"
  pm.send("alertDebug", msg)
  time.sleep(0.1)
PY
}

finish_update() {
  local unsafe_reasons
  unsafe_reasons="$(unsafe_update_reasons)"
  if [ -z "$unsafe_reasons" ]; then
    rm -f /data/openpilot/prebuilt
    sudo reboot
    exit 0
  fi

  echo "Update staged while on-road; ${unsafe_reasons}."
  wait_for_onroad_controls_off
  unsafe_reasons="$(unsafe_update_reasons)"
  rm -f /data/openpilot/prebuilt
  if [ -z "$unsafe_reasons" ]; then
    sudo reboot
    exit 0
  else
    echo "Skipping device reboot while ignition/on-road is active to avoid latching a cruise fault."
    show_update_staged_alert
    exit 0
  fi
}

wait_until_safe_to_update() {
  local phase="${1:-pre}"
  local unsafe_reasons
  unsafe_reasons="$(unsafe_update_reasons)"
  if [ -n "$unsafe_reasons" ]; then
    wait_for_onroad_controls_off
    if [ "$phase" != "reboot" ]; then
      echo "Continuing update while ignition is on; reboot will be skipped if the car stays on-road."
    fi
    return
  fi
}

wait_until_safe_to_update pre

current_branch="$(git symbolic-ref --quiet --short HEAD 2>/dev/null || true)"
if [ -n "$current_branch" ]; then
  upstream_ref="$(git rev-parse --abbrev-ref --symbolic-full-name '@{u}' 2>/dev/null || true)"
  if [ -n "$upstream_ref" ]; then
    remote_name="${upstream_ref%%/*}"
    remote_branch="${upstream_ref#*/}"
    run_low_priority git fetch "$remote_name" "refs/heads/$remote_branch"
  else
    run_low_priority git fetch origin "refs/heads/$current_branch"
  fi
else
  run_low_priority git fetch origin
fi
run_low_priority git reset --hard FETCH_HEAD
run_low_priority git submodule update -f

finish_update
