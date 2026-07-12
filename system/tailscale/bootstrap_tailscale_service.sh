#!/usr/bin/env bash

set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null && pwd)"
TAILSCALE_DIR="/data/media/0/tailscale"
BIN_DIR="${TAILSCALE_DIR}/bin"
ENABLED_MARKER="${TAILSCALE_DIR}/enabled"
SUPERVISOR_PATH="${TAILSCALE_DIR}/tailscale_supervisor.sh"
SERVICE_NAME="openpilot-tailscaled.service"
SERVICE_SYSTEM_PATH="/etc/systemd/system/${SERVICE_NAME}"
SERVICE_RUNTIME_PATH="/run/systemd/system/${SERVICE_NAME}"
SUDO=(sudo -n)

log() {
  echo "tailscale bootstrap: $*"
}

systemd_available() {
  command -v systemctl >/dev/null 2>&1 && [[ -d /run/systemd/system ]]
}

service_path() {
  if "${SUDO[@]}" test -w /etc/systemd/system >/dev/null 2>&1; then
    echo "${SERVICE_SYSTEM_PATH}"
  else
    echo "${SERVICE_RUNTIME_PATH}"
  fi
}

if [[ ! -f "${ENABLED_MARKER}" ]]; then
  exit 0
fi

if [[ ! -x "${BIN_DIR}/tailscale" || ! -x "${BIN_DIR}/tailscaled" ]]; then
  log "binaries missing, skipping"
  exit 0
fi

if ! systemd_available; then
  log "systemd unavailable, skipping"
  exit 0
fi

installed=0
SERVICE_PATH="$(service_path)"

if [[ ! -x "${SUPERVISOR_PATH}" ]] || ! cmp -s "${SCRIPT_DIR}/tailscale_supervisor.sh" "${SUPERVISOR_PATH}"; then
  if "${SUDO[@]}" install -m 755 "${SCRIPT_DIR}/tailscale_supervisor.sh" "${SUPERVISOR_PATH}"; then
    installed=1
  else
    log "failed to install ${SUPERVISOR_PATH}"
  fi
fi

if [[ ! -f "${SERVICE_PATH}" ]] || ! cmp -s "${SCRIPT_DIR}/${SERVICE_NAME}" "${SERVICE_PATH}"; then
  if "${SUDO[@]}" install -m 644 "${SCRIPT_DIR}/${SERVICE_NAME}" "${SERVICE_PATH}"; then
    installed=1
  else
    log "failed to install ${SERVICE_PATH}"
  fi
fi

if [[ ${installed} -eq 1 ]]; then
  "${SUDO[@]}" systemctl daemon-reload || log "daemon-reload failed"
  if [[ "${SERVICE_PATH}" == "${SERVICE_SYSTEM_PATH}" ]]; then
    "${SUDO[@]}" systemctl enable "${SERVICE_NAME}" >/dev/null || log "enable failed"
  fi
  if systemctl is-active --quiet "${SERVICE_NAME}"; then
    "${SUDO[@]}" systemctl restart "${SERVICE_NAME}" || log "restart failed"
  fi
fi

if ! systemctl is-active --quiet "${SERVICE_NAME}"; then
  "${SUDO[@]}" systemctl start "${SERVICE_NAME}" || log "start failed"
fi
