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

# Prevent overlapping fetch/reset/submodule operations when the updater is launched more than once
# (for example, from both the Software panel and SSH). The detached reboot supervisor deliberately
# skips this lock: it is spawned before the main updater exits and has its own single-instance lock.
if [ "${1:-}" != "__reboot_when_parked" ]; then
  fullupdate_lock_file=/data/fullupdate.lock
  if [ ! -d /data ]; then
    fullupdate_lock_file="${TMPDIR:-/tmp}/openpilot-fullupdate.lock"
  fi

  exec {fullupdate_lock_fd}>"$fullupdate_lock_file"
  if ! flock -n "$fullupdate_lock_fd"; then
    echo "A full update is already running; leaving the existing update in charge."
    exit 0
  fi
fi

run_low_priority() {
  nice -n 10 "$@"
}

runtime_helpers_ready() {
  run_low_priority "$PYTHON" - <<'PY' >/dev/null 2>&1
import cereal.messaging  # noqa: F401
from openpilot.common.params import Params

Params().check_key("BuildMetadata")
PY
}

ensure_runtime_helpers() {
  if runtime_helpers_ready; then
    return
  fi

  echo "Generated runtime modules are missing; building source tree before update safety checks."
  run_low_priority "$PYTHON" "$OPENPILOT_DIR/system/manager/build.py"
}

# On-screen feedback from the moment the update starts: without this, an on-road driver who clicks the
# Full Update button sees nothing on the driving screen until staging finishes and the reboot
# supervisor starts publishing its own banners. The helper closes the inherited updater lock fd (an
# inherited lock fd previously blocked later updates when the supervisor held it) and is stopped
# before the supervisor spawns so only one alertDebug publisher is ever active.
staging_banner_pid=""
start_staging_banner() {
  run_low_priority "$PYTHON" - {fullupdate_lock_fd}>&- <<'PY' &
import time
try:
  import cereal.messaging as messaging

  pm = messaging.PubMaster(["alertDebug"])
  while True:
    msg = messaging.new_message("alertDebug")
    msg.valid = True
    msg.alertDebug.alertText1 = "Update Running"
    msg.alertDebug.alertText2 = "Fetching latest build"
    pm.send("alertDebug", msg)
    time.sleep(0.1)
except Exception:
  pass
PY
  staging_banner_pid=$!
}

stop_staging_banner() {
  if [ -n "${staging_banner_pid:-}" ]; then
    kill "$staging_banner_pid" 2>/dev/null || true
    staging_banner_pid=""
  fi
}

unsafe_update_reasons() {
  run_low_priority "$PYTHON" - <<'PY'
import time

import cereal.messaging as messaging
from cereal import log
from openpilot.common.params import Params

UNKNOWN_PANDA = log.PandaState.PandaType.unknown

params = Params()
reasons = []

if params.get_bool("IsOnroad"):
  reasons.append("openpilot is onroad")
if params.get_bool("IsEngaged"):
  reasons.append("openpilot is engaged")

sm = messaging.SubMaster(["pandaStates"])
saw_valid_frame = False
end_t = time.monotonic() + 1.5
while time.monotonic() < end_t:
  sm.update(100)
  if sm.updated["pandaStates"]:
    # Only a real (non-unknown) panda confirms ignition state. An unknown-only frame (panda still
    # enumerating) must NOT count as confirmation, or it could green-light an immediate reboot.
    valid = [ps for ps in sm["pandaStates"] if ps.pandaType != UNKNOWN_PANDA]
    if valid:
      saw_valid_frame = True
      if any(ps.ignitionLine or ps.ignitionCan for ps in valid):
        reasons.append("vehicle ignition is on")
      break

if not saw_valid_frame:
  # Never confirmed ignition state. Fail SAFE: do not let an empty result green-light an immediate
  # reboot -- force the stage-and-wait path so we only reboot after positively observing off-road.
  reasons.append("panda state unconfirmed")

print(", ".join(dict.fromkeys(reasons)))
PY
}

request_live_update_handoff() {
  run_low_priority "$PYTHON" - <<'PY'
from openpilot.common.params import Params

params = Params()
if params.get_bool("IsOnroad"):
  current_state = (params.get("LiveUpdateHandoffState") or "").partition(":")[0]
  if current_state in ("", "aborted", "unsupported", "failed"):
    params.put("LiveUpdateHandoffState", "requested")
    if current_state == "failed":
      print("Re-arming live restart after a failed handoff; keep cruise off.")
    else:
      print("Live restart armed. Press and release cruise-main once; no screen tap is required.")
  else:
    print(f"Live restart is already in progress ({current_state}); leaving its state unchanged.")
PY
}

wait_for_offroad_reboot() {
  # Exit 0 => either off-road, or a fresh verified stock-SCC handoff permits reboot.
  # Exit 7 => cancel requested, caller must NOT reboot.
  run_low_priority "$PYTHON" - <<'PY'
import os
import sys
import time

import cereal.messaging as messaging
from cereal import car, log
from openpilot.common.params import Params

CANCEL_FILE = "/data/fullupdate_reboot.cancel"
UNKNOWN_PANDA = log.PandaState.PandaType.unknown
READY_MAX_AGE = 2.0
MESSAGE_MAX_AGE = 1.0

params = Params()
pm = messaging.PubMaster(["alertDebug"])
# Poll only on pandaStates (10 Hz). Polling every subscribed socket makes this loop wake on the
# 100 Hz controls services, wasting a third of a CPU core while the update is staged.
sm = messaging.SubMaster(["carControl", "frogpilotCarState", "pandaStates", "selfdriveState"], poll="pandaStates")
last_seen = {service: 0.0 for service in sm.services}
last_reasons = None

while True:
  try:
    sm.update(500)
    now = time.monotonic()
    for service in last_seen:
      if sm.updated[service] and sm.valid[service]:
        last_seen[service] = now

    if os.path.exists(CANCEL_FILE):
      handoff_state = (params.get("LiveUpdateHandoffState") or "").partition(":")[0]
      if handoff_state in ("requested", "aborted", "unsupported"):
        params.remove("LiveUpdateHandoffState")
      print("Cancel requested; not rebooting. Update remains staged on disk.", flush=True)
      sys.exit(7)

    is_onroad = params.get_bool("IsOnroad")
    is_engaged = params.get_bool("IsEngaged")
    panda_fresh = now - last_seen["pandaStates"] <= MESSAGE_MAX_AGE
    valid_pandas = [ps for ps in sm["pandaStates"] if ps.pandaType != UNKNOWN_PANDA] if panda_fresh else []
    ignition = bool(valid_pandas) and any(ps.ignitionLine or ps.ignitionCan for ps in valid_pandas)

    # Off-road keeps the existing fail-open behavior for a missing panda: otherwise a parked car with a
    # down pandad could be trapped forever. The moving path below is deliberately fail-closed.
    if not is_onroad and not is_engaged and not ignition:
      print("Vehicle is off-road; rebooting to finish update.", flush=True)
      sys.exit(0)

    raw_handoff_state = params.get("LiveUpdateHandoffState") or ""
    handoff_state, separator, raw_timestamp = raw_handoff_state.partition(":")
    try:
      handoff_timestamp = float(raw_timestamp) if separator else 0.0
    except ValueError:
      handoff_timestamp = 0.0

    messages_fresh = all(now - last_seen[service] <= MESSAGE_MAX_AGE for service in last_seen)
    panda_ready = (bool(valid_pandas) and
                   all(ps.safetyModel == car.CarParams.SafetyModel.elm327 and len(ps.faults) == 0 for ps in valid_pandas))
    car_control = sm["carControl"]
    frogpilot_car_state = sm["frogpilotCarState"]
    selfdrive_state = sm["selfdriveState"]
    # A fresh READY timestamp is issued and refreshed by card only while its current carState is in
    # Drive or Park with cruise unavailable/disabled and the stock-SCC verifier remains live. Do not
    # add an independent carState subscription here: that feed can be unavailable to a late subscriber
    # even while card is publishing it, which leaves an already-verified handoff stuck forever. Card
    # owns the gear check and immediately revokes READY if the vehicle leaves Drive/Park.
    controls_off = (not is_engaged and not car_control.enabled and not car_control.latActive and not car_control.longActive and
                    not frogpilot_car_state.alwaysOnLateralEnabled and not selfdrive_state.enabled and not selfdrive_state.active)
    handoff_ready = (is_onroad and handoff_state == "ready" and 0.0 <= now - handoff_timestamp <= READY_MAX_AGE and
                     messages_fresh and panda_ready and controls_off)
    if handoff_ready:
      print("Verified stock SCC takeover while on-road; rebooting to finish update.", flush=True)
      sys.exit(0)

    reasons = []
    if is_engaged:
      reasons.append("openpilot is engaged")
    if not messages_fresh:
      reasons.append("live vehicle state is unconfirmed")
    if handoff_state in ("aborted", "failed"):
      reasons.append("stock SCC takeover was not verified")
    elif handoff_state == "unsupported":
      reasons.append("live handoff is unsupported")
    else:
      reasons.append("waiting for cruise off and verified stock SCC")

    reasons = list(dict.fromkeys(reasons))
    if reasons != last_reasons:
      print("Update staged; waiting to reboot: " + ", ".join(reasons), flush=True)
      last_reasons = reasons

    msg = messaging.new_message("alertDebug")
    msg.valid = True
    # These text1 values double as the compact-badge trigger in selfdrive/ui/qt/onroad/alerts.cc
    # (isStagedUpdateAlert); keep them in sync and keep text2 short enough for the badge.
    if handoff_state in ("aborted", "failed", "unsupported"):
      # Terminal for this drive: a live restart is no longer possible, but the parked path still
      # applies the update the moment the car is next turned off. Tell the driver that, not just
      # that something is unavailable.
      msg.alertDebug.alertText1 = "Update Staged"
      msg.alertDebug.alertText2 = "Applies when parked"
    elif handoff_state == "requested" and controls_off:
      msg.alertDebug.alertText1 = "Preparing Restart"
      msg.alertDebug.alertText2 = "Keep cruise off"
    elif handoff_state in ("diagnostic_requested", "diagnostic", "verifying", "ready"):
      msg.alertDebug.alertText1 = "Preparing Restart"
      msg.alertDebug.alertText2 = "Keep cruise off"
    else:
      msg.alertDebug.alertText1 = "Update Ready"
      msg.alertDebug.alertText2 = "Cruise off to restart"
    pm.send("alertDebug", msg)
  except SystemExit:
    raise
  except Exception as e:
    # A transient messaging/params hiccup must NOT kill the supervisor (that would re-create the
    # "updated but never rebooted" bug). Log, back off, and keep waiting. Recreate the banner
    # publisher too: a competing publisher (e.g. a later fullupdate run) can steal the msgq
    # registration, after which every send fails until the socket is rebuilt.
    print(f"wait loop transient error: {e!r}; continuing", flush=True)
    try:
      pm = messaging.PubMaster(["alertDebug"])
    except Exception:
      pass
    time.sleep(1.0)
PY
}

# True only if $1 (a pidfile) names a process that is alive AND is actually one of our supervisors.
# The cmdline check defeats PID reuse: a stale pidfile left by a power-cut whose PID is reused after
# reboot won't be mistaken for a live supervisor.
supervisor_pid_alive() {
  local p
  [ -f "$1" ] || return 1
  p="$(cat "$1" 2>/dev/null)" || return 1
  [ -n "$p" ] || return 1
  kill -0 "$p" 2>/dev/null || return 1
  grep -qa "__reboot_when_parked" "/proc/$p/cmdline" 2>/dev/null
}

# Detached entrypoint (re-invoked via setsid by finish_update). Reboots after either a verified live
# stock-SCC handoff or an off-road transition, unless cancelled. It survives the SSH connection closing.
reboot_when_parked_supervisor() {
  local pidfile=/data/fullupdate_reboot.pid
  local cancelfile=/data/fullupdate_reboot.cancel
  local lockfile=/data/fullupdate_reboot.lock

  # Single-instance, race-free: hold an exclusive lock for our whole lifetime. A second supervisor
  # cannot get it and exits. flock releases automatically on exit/reboot, so there is no stale-lock
  # problem (unlike a pidfile). Use an auto-allocated fd (>=10), which bash opens close-on-exec, so our
  # python children don't inherit the lock and can't hold it past our death.
  local lockfd
  exec {lockfd}>"$lockfile"
  if ! flock -n "$lockfd"; then
    echo "[$(date)] another reboot supervisor already holds the lock; exiting."
    exit 0
  fi

  echo "$$" > "$pidfile"
  # Remove our pidfile on any exit so a crash/kill does not leave a stale 'already running' marker.
  trap 'rm -f "$pidfile"' EXIT

  # The parent ran ensure_runtime_helpers before spawning us, but re-assert it cheaply (fast no-op once
  # built) so a transient missing build in this detached process self-heals instead of silently
  # abandoning the reboot.
  ensure_runtime_helpers

  echo "[$(date)] reboot supervisor started (pid $$); waiting for verified stock SCC takeover or off-road."
  echo "[$(date)] cancel this pending reboot with: touch /data/fullupdate_reboot.cancel"

  local rc tries=0
  while true; do
    rc=0
    wait_for_offroad_reboot || rc=$?
    case "$rc" in
      0)
        rm -f "$cancelfile"
        echo "[$(date)] safe restart condition verified; rebooting to finish update."
        if sudo reboot; then
          return
        fi
        # A failed reboot CALL must not abort the supervisor under set -e (that would strand the staged
        # update). Stay parked and retry; the next loop re-confirms off-road and tries again.
        echo "[$(date)] sudo reboot failed; retrying in 10s." >&2
        sleep 10
        ;;
      7)
        rm -f "$cancelfile"
        echo "[$(date)] reboot cancelled; update stays staged and applies on the next reboot/deploy."
        return
        ;;
      *)
        # The wait loop should never exit non-zero except for cancel (it catches transient errors), so a
        # crash here is e.g. an OOM kill. Retry rather than silently abandon the staged reboot.
        tries=$((tries + 1))
        if [ "$tries" -ge 30 ]; then
          echo "[$(date)] wait loop keeps failing (rc=${rc}) after ${tries} retries; giving up. Update staged; reboot manually." >&2
          return
        fi
        echo "[$(date)] wait loop crashed (rc=${rc}); retry ${tries} in 10s." >&2
        sleep 10
        ;;
    esac
  done
}

# Self-documenting: how to watch / cancel a pending detached reboot. Printed wherever a reboot is pending
# so the operator never has to remember the sentinel path.
print_reboot_controls() {
  echo "  watch:  tail -f /data/fullupdate_reboot.log"
  echo "  cancel: touch /data/fullupdate_reboot.cancel   (aborts only the currently-pending reboot; the update still applies on the next reboot/deploy)"
}

finish_update() {
  local logfile=/data/fullupdate_reboot.log
  local pidfile=/data/fullupdate_reboot.pid
  local cancelfile=/data/fullupdate_reboot.cancel

  # A fresh run supersedes any prior cancel and must never be aborted by a stale sentinel: clear it up
  # front, before any reboot path or the single-instance check.
  rm -f "$cancelfile"

  local unsafe_reasons rc=0
  unsafe_reasons="$(unsafe_update_reasons)" || rc=$?
  if [ "$rc" -ne 0 ]; then
    # The safety check itself crashed (e.g. a broken import in the freshly-reset tree, OOM). Don't die
    # under set -e and strand the staged update -- treat as unsafe and let the supervisor re-evaluate
    # live before rebooting (spawning on uncertainty is strictly safer than never rebooting).
    echo "Safety check errored (rc=${rc}); staging and deferring the reboot decision to the supervisor." >&2
    unsafe_reasons="safety check errored"
  fi
  if [ -z "$unsafe_reasons" ]; then
    rm -f /data/openpilot/prebuilt
    if sudo reboot; then
      exit 0
    fi
    echo "ERROR: sudo reboot failed; reboot manually once parked." >&2
    exit 1
  fi

  echo "Update staged while on-road; ${unsafe_reasons}."
  rm -f /data/openpilot/prebuilt
  request_live_update_handoff

  # Don't spawn a redundant supervisor if one is genuinely still alive (best-effort; the child also
  # self-deduplicates via flock, so a race here is harmless).
  if supervisor_pid_alive "$pidfile"; then
    echo "A reboot-when-parked supervisor is already running (PID $(cat "$pidfile" 2>/dev/null)); leaving it in charge."
    print_reboot_controls
    exit 0
  fi

  if ! command -v setsid >/dev/null 2>&1; then
    echo "ERROR: setsid not found; cannot detach the reboot supervisor. Reboot manually once parked." >&2
    exit 1
  fi

  # Hand banner publishing over to the supervisor before it spawns so only one alertDebug publisher
  # is active at a time.
  stop_staging_banner

  # The main updater lock protects fetch/reset/submodule work only. Bash keeps dynamically allocated
  # descriptors open across exec, so close it explicitly before starting the long-lived supervisor;
  # otherwise every staged on-road update blocks all later fullupdate runs until the car reboots.
  exec {fullupdate_lock_fd}>&-

  # Detach the verified-handoff/off-road reboot supervisor into its own session so it survives SSH closing.
  setsid "$OPENPILOT_DIR/fullupdate.sh" __reboot_when_parked >>"$logfile" 2>&1 </dev/null &
  local child=$!
  disown 2>/dev/null || true

  # Confirm a supervisor actually came up before claiming success: either the spawned process is still
  # alive (still starting), or it has written + identified its pidfile.
  sleep 1
  if ! kill -0 "$child" 2>/dev/null && ! supervisor_pid_alive "$pidfile"; then
    echo "ERROR: reboot supervisor did not start; see ${logfile}. Reboot manually once parked." >&2
    exit 1
  fi

  echo "Update staged. Press and release cruise-main once; reboot is automatic after stock SCC takeover is verified."
  print_reboot_controls
  exit 0
}

wait_until_safe_to_update() {
  local phase="${1:-pre}"
  local unsafe_reasons
  unsafe_reasons="$(unsafe_update_reasons)"
  if [ -n "$unsafe_reasons" ]; then
    if [ "$phase" != "reboot" ]; then
      echo "Staging update while on-road; press and release cruise-main once after staging to finish it."
    fi
    return
  fi
}

# Detached reboot-supervisor entrypoint (re-invoked by finish_update via setsid). Must run after the
# function definitions above and skip the whole fetch/update flow below.
if [ "${1:-}" = "__reboot_when_parked" ]; then
  reboot_when_parked_supervisor
  exit 0
fi

ensure_runtime_helpers
trap stop_staging_banner EXIT
# A live reboot supervisor from an earlier staged update is already publishing alertDebug banners;
# starting a second publisher steals its msgq registration and silences the on-screen notice for the
# rest of the drive. Only show the staging banner when no supervisor holds the channel.
if ! supervisor_pid_alive /data/fullupdate_reboot.pid; then
  start_staging_banner
fi
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

ensure_runtime_helpers
finish_update
