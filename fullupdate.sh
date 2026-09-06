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

print(", ".join(dict.fromkeys(reasons)))
PY
}

finish_update() {
  if [ -z "$unsafe_reasons" ]; then
  fi

}

wait_until_safe_to_update() {
  local phase="${1:-pre}"
  local unsafe_reasons
  unsafe_reasons="$(unsafe_update_reasons)"
  if [ -n "$unsafe_reasons" ]; then
    if [ "$phase" != "reboot" ]; then
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
