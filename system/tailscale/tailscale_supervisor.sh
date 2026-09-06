#!/usr/bin/env bash

set -u

TAILSCALE_DIR="${TAILSCALE_DIR:-/data/media/0/tailscale}"
BIN_DIR="${TAILSCALE_DIR}/bin"
STATE_DIR="${TAILSCALE_DIR}/state"
SOCKET_PATH="${TAILSCALE_DIR}/tailscaled.sock"
STATE_PATH="${STATE_DIR}/tailscaled.state"
ENABLED_MARKER="${TAILSCALE_DIR}/enabled"
TAILSCALE_BIN="${BIN_DIR}/tailscale"
TAILSCALED_BIN="${BIN_DIR}/tailscaled"

CHECK_INTERVAL="${TAILSCALE_CHECK_INTERVAL:-30}"
HEALTH_FAILURE_LIMIT="${TAILSCALE_HEALTH_FAILURE_LIMIT:-3}"
RESTART_DELAY="${TAILSCALE_RESTART_DELAY:-10}"
TUN_WAIT_SECONDS="${TAILSCALE_TUN_WAIT_SECONDS:-60}"
CLOCK_WAIT_SECONDS="${TAILSCALE_CLOCK_WAIT_SECONDS:-90}"
MIN_CLOCK_UNIX_TIME="${TAILSCALE_MIN_CLOCK_UNIX_TIME:-1780272000}"  # 2026-06-01
CLOCK_STALE_AT_START=0

log() {
  printf '%s openpilot-tailscaled: %s\n' "$(date -Iseconds 2>/dev/null || date)" "$*"
}

enabled() {
  [[ -f "${ENABLED_MARKER}" ]]
}

tailscale_cmd() {
  "${TAILSCALE_BIN}" --socket "${SOCKET_PATH}" "$@"
}

tailscaled_ready() {
  [[ -x "${TAILSCALE_BIN}" ]] || return 1

  local status_out status_rc
  if command -v timeout >/dev/null 2>&1; then
    status_out="$(timeout 10 "${TAILSCALE_BIN}" --socket "${SOCKET_PATH}" status --json 2>&1)"
  else
    status_out="$(tailscale_cmd status --json 2>&1)"
  fi
  status_rc=$?

  if [[ ${status_rc} -eq 0 ]] || grep -q '"BackendState"' <<< "${status_out}"; then
    return 0
  fi

  if [[ ${status_rc} -eq 1 ]] && grep -q "Logged out." <<< "${status_out}"; then
    return 0
  fi

  return 1
}

matching_tailscaled_running() {
  ps -eo args 2>/dev/null \
    | grep -F -- "${TAILSCALED_BIN}" \
    | grep -F -- "--socket=${SOCKET_PATH}" \
    | grep -v grep >/dev/null 2>&1
}

remove_stale_socket() {
  if [[ -S "${SOCKET_PATH}" ]] && ! matching_tailscaled_running; then
    log "removing stale socket ${SOCKET_PATH}"
    rm -f "${SOCKET_PATH}"
  fi
}

wait_for_tun() {
  local waited=0
  while [[ ${waited} -lt ${TUN_WAIT_SECONDS} ]]; do
    [[ -c /dev/net/tun ]] && return 0
    sleep 1
    waited=$((waited + 1))
  done

  return 1
}

clock_ready() {
  local now
  now="$(date -u +%s 2>/dev/null || echo 0)"
  [[ "${now}" -ge "${MIN_CLOCK_UNIX_TIME}" ]]
}

wait_for_clock() {
  CLOCK_STALE_AT_START=0
  if clock_ready; then
    return 0
  fi

  log "waiting for system clock sync"
  if command -v timedatectl >/dev/null 2>&1; then
    timedatectl set-ntp true >/dev/null 2>&1 || true
  fi
  if command -v systemctl >/dev/null 2>&1; then
    systemctl start systemd-timesyncd.service >/dev/null 2>&1 || true
  fi

  local waited=0
  while [[ ${waited} -lt ${CLOCK_WAIT_SECONDS} ]]; do
    if clock_ready; then
      return 0
    fi
    sleep 2
    waited=$((waited + 2))
  done

  log "system clock is still stale; starting tailscaled anyway"
  CLOCK_STALE_AT_START=1
  return 0
}

stop_matching_tailscaled() {
  if command -v pkill >/dev/null 2>&1; then
    pkill -f "${TAILSCALED_BIN}.*--socket=${SOCKET_PATH}" || true
  fi
}

start_tailscaled() {
  local sudo_prefix=()
  if [[ "$(id -u)" -ne 0 ]]; then
    if ! command -v sudo >/dev/null 2>&1; then
      log "not running as root and sudo is unavailable"
      return 1
    fi
    sudo_prefix=(sudo -n)
  fi

  local cmd=(
    "${sudo_prefix[@]}"
    "${TAILSCALED_BIN}"
    "--tun=tailscale0"
    "--state=${STATE_PATH}"
    "--statedir=${STATE_DIR}"
    "--socket=${SOCKET_PATH}"
  )

  log "starting tailscaled"
  "${cmd[@]}" &
  local child=$!
  local health_failures=0
  local health_waited=0

  while kill -0 "${child}" >/dev/null 2>&1; do
    if ! enabled; then
      log "enabled marker removed; stopping tailscaled"
      kill "${child}" >/dev/null 2>&1 || true
      wait "${child}" >/dev/null 2>&1 || true
      return 0
    fi

    if [[ ${CLOCK_STALE_AT_START} -eq 1 ]] && clock_ready; then
      log "system clock became valid; restarting tailscaled to clear stale TLS state"
      kill "${child}" >/dev/null 2>&1 || true
      wait "${child}" >/dev/null 2>&1 || true
      return 0
    fi

    sleep 5
    health_waited=$((health_waited + 5))
    if [[ ${health_waited} -lt ${CHECK_INTERVAL} ]]; then
      continue
    fi

    health_waited=0
    if tailscaled_ready; then
      health_failures=0
      continue
    fi

    health_failures=$((health_failures + 1))
    log "tailscaled local API health check failed (${health_failures}/${HEALTH_FAILURE_LIMIT})"
    if [[ ${health_failures} -ge ${HEALTH_FAILURE_LIMIT} ]]; then
      log "tailscaled is unresponsive; restarting it"
      kill "${child}" >/dev/null 2>&1 || true
      wait "${child}" >/dev/null 2>&1 || true
      return 1
    fi
  done

  wait "${child}" >/dev/null 2>&1
  local status=$?
  log "tailscaled exited with status ${status}"
  return "${status}"
}

while true; do
  if ! enabled; then
    log "disabled; missing marker ${ENABLED_MARKER}"
    exit 0
  fi

  if [[ ! -x "${TAILSCALED_BIN}" ]]; then
    log "missing executable ${TAILSCALED_BIN}"
    sleep "${RESTART_DELAY}"
    continue
  fi

  if ! mkdir -p "${STATE_DIR}"; then
    log "failed to create state directory ${STATE_DIR}"
    sleep "${RESTART_DELAY}"
    continue
  fi

  if ! wait_for_tun; then
    log "/dev/net/tun is unavailable"
    sleep "${RESTART_DELAY}"
    continue
  fi

  wait_for_clock

  if tailscaled_ready; then
    sleep "${CHECK_INTERVAL}"
    continue
  fi

  remove_stale_socket

  if matching_tailscaled_running; then
    log "tailscaled process exists but is not ready; restarting it"
    stop_matching_tailscaled
    sleep "${RESTART_DELAY}"
    continue
  fi

  start_tailscaled
  sleep "${RESTART_DELAY}"
done
