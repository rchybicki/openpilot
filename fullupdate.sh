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

wait_for_offroad_reboot() {
  # Exit 0 => car is off-road, caller should reboot. Exit 7 => cancel requested, caller must NOT reboot.
  run_low_priority "$PYTHON" - <<'PY'
import os
import sys
import time

import cereal.messaging as messaging
from cereal import log
from openpilot.common.params import Params

CANCEL_FILE = "/data/fullupdate_reboot.cancel"
UNKNOWN_PANDA = log.PandaState.PandaType.unknown

params = Params()
pm = messaging.PubMaster(["alertDebug"])
sm = messaging.SubMaster(["pandaStates"])
last_reasons = None

while True:
  try:
    sm.update(500)

    if os.path.exists(CANCEL_FILE):
      print("Cancel requested; not rebooting. Update remains staged on disk.", flush=True)
      sys.exit(7)

    reasons = []
    if params.get_bool("IsOnroad"):
      reasons.append("openpilot is onroad")
    if params.get_bool("IsEngaged"):
      reasons.append("openpilot is engaged")

    # DELIBERATE asymmetry vs unsafe_update_reasons: the supervisor must fail OPEN here. unsafe_update_reasons
    # fails closed (it can defer to this supervisor), but if THIS loop refused to proceed without a fresh
    # panda frame, a down pandad would trap a parked car forever (never reboots). So lean on the live
    # manager-written IsOnroad/IsEngaged params and treat a missing/stale panda as "no ignition reason".
    ignition = sm.seen["pandaStates"] and any(ps.ignitionLine or ps.ignitionCan
                                              for ps in sm["pandaStates"] if ps.pandaType != UNKNOWN_PANDA)
    if ignition:
      reasons.append("vehicle ignition is on")

    if not reasons:
      print("Vehicle is off-road; rebooting to finish update.", flush=True)
      sys.exit(0)

    reasons = list(dict.fromkeys(reasons))
    if reasons != last_reasons:
      print("Update staged; waiting to reboot: " + ", ".join(reasons), flush=True)
      last_reasons = reasons

    msg = messaging.new_message("alertDebug")
    msg.alertDebug.alertText1 = "Update Staged"
    msg.alertDebug.alertText2 = "Will reboot when parked"
    pm.send("alertDebug", msg)
  except SystemExit:
    raise
  except Exception as e:
    # A transient messaging/params hiccup must NOT kill the supervisor (that would re-create the
    # "updated but never rebooted" bug). Log, back off, and keep waiting.
    print(f"wait loop transient error: {e!r}; continuing", flush=True)
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

# Detached entrypoint (re-invoked via setsid by finish_update). Waits for the car to be parked, then
# reboots -- unless cancelled. Runs in its own session so it survives the SSH connection closing.
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

  echo "[$(date)] reboot-when-parked supervisor started (pid $$); waiting for the car to be parked."
  echo "[$(date)] cancel this pending reboot with: touch /data/fullupdate_reboot.cancel"

  local rc tries=0
  while true; do
    rc=0
    wait_for_offroad_reboot || rc=$?
    case "$rc" in
      0)
        rm -f "$cancelfile"
        echo "[$(date)] car is off-road; rebooting to finish update."
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

  # Detach the wait-for-parked-then-reboot step into its own session (setsid) with all stdio off the
  # TTY, so it survives this SSH session closing and reboots only once the car is parked.
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

  echo "Update staged. The car will reboot automatically when next parked -- this now survives closing SSH."
  print_reboot_controls
  exit 0
}

wait_until_safe_to_update() {
  local phase="${1:-pre}"
  local unsafe_reasons
  unsafe_reasons="$(unsafe_update_reasons)"
  if [ -n "$unsafe_reasons" ]; then
    if [ "$phase" != "reboot" ]; then
      echo "Staging update while on-road; reboot will be needed when parked."
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
