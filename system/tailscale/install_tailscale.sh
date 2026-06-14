#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null && pwd)"
TAILSCALE_DIR="/data/media/0/tailscale"
BIN_DIR="${TAILSCALE_DIR}/bin"
STATE_DIR="${TAILSCALE_DIR}/state"
SOCKET_PATH="${TAILSCALE_DIR}/tailscaled.sock"
STATE_PATH="${STATE_DIR}/tailscaled.state"
ENABLED_MARKER="${TAILSCALE_DIR}/enabled"
SUPERVISOR_PATH="${TAILSCALE_DIR}/tailscale_supervisor.sh"
SERVICE_NAME="openpilot-tailscaled.service"
SERVICE_SYSTEM_PATH="/etc/systemd/system/${SERVICE_NAME}"
SERVICE_RUNTIME_PATH="/run/systemd/system/${SERVICE_NAME}"
PKGS_URL="https://pkgs.tailscale.com/stable"

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

print_step() {
  echo -e "${YELLOW}==>${NC} $1"
}

print_ok() {
  echo -e "${GREEN}OK${NC} $1"
}

print_err() {
  echo -e "${RED}ERR${NC} $1" >&2
}

require_cmd() {
  local cmd="$1"
  if ! command -v "$cmd" >/dev/null 2>&1; then
    print_err "Missing required command: $cmd"
    exit 1
  fi
}

tailscale_cmd() {
  sudo "${BIN_DIR}/tailscale" --socket "${SOCKET_PATH}" "$@"
}

systemd_available() {
  command -v systemctl >/dev/null 2>&1 && [[ -d /run/systemd/system ]]
}

service_path() {
  if sudo -n test -w /etc/systemd/system >/dev/null 2>&1; then
    echo "${SERVICE_SYSTEM_PATH}"
  else
    echo "${SERVICE_RUNTIME_PATH}"
  fi
}

install_systemd_service() {
  if ! systemd_available; then
    echo "Warning: systemd is unavailable. Falling back to the current-session tailscaled starter."
    return
  fi

  local service_path
  service_path="$(service_path)"

  print_step "Installing persistent Tailscale supervisor"
  sudo install -m 755 "${SCRIPT_DIR}/tailscale_supervisor.sh" "${SUPERVISOR_PATH}"
  sudo install -m 644 "${SCRIPT_DIR}/${SERVICE_NAME}" "${service_path}"
  sudo systemctl daemon-reload
  if [[ "${service_path}" == "${SERVICE_SYSTEM_PATH}" ]]; then
    sudo systemctl enable "${SERVICE_NAME}" >/dev/null
    print_ok "Installed and enabled ${SERVICE_NAME}"
  else
    print_ok "Installed runtime ${SERVICE_NAME}; launch bootstrap will recreate it on boot"
  fi
}

determine_arch() {
  case "$(uname -m)" in
    aarch64|arm64) echo "arm64" ;;
    *)
      print_err "Unsupported architecture: $(uname -m). Update the installer mapping first."
      exit 1
      ;;
  esac
}

get_latest_version() {
  local arch="$1"
  curl -fsSL "${PKGS_URL}/" \
    | grep -Eo "tailscale_[0-9]+\.[0-9]+\.[0-9]+_${arch}\.tgz" \
    | sed -E 's/^tailscale_([0-9]+\.[0-9]+\.[0-9]+)_.*/\1/' \
    | sort -V \
    | tail -n 1
}

start_tailscaled_if_needed() {
  if tailscaled_ready; then
    if systemd_available && [[ -x "${SUPERVISOR_PATH}" ]]; then
      if ! sudo systemctl is-active --quiet "${SERVICE_NAME}"; then
        print_step "Starting ${SERVICE_NAME} to supervise the existing tailscaled"
        sudo systemctl start "${SERVICE_NAME}"
      fi
      print_ok "${SERVICE_NAME} is supervising tailscaled"
    else
      print_ok "tailscaled is already running"
    fi
    return
  fi

  if systemd_available && [[ -x "${SUPERVISOR_PATH}" ]]; then
    print_step "Starting ${SERVICE_NAME}"
    sudo systemctl restart "${SERVICE_NAME}"
  elif ! ps aux | grep -F -- "${BIN_DIR}/tailscaled" | grep -F -- "--socket=${SOCKET_PATH}" | grep -v grep >/dev/null 2>&1; then
    print_step "Starting tailscaled"
    sudo "${BIN_DIR}/tailscaled" \
      --tun=tailscale0 \
      --state="${STATE_PATH}" \
      --statedir="${STATE_DIR}" \
      --socket="${SOCKET_PATH}" \
      > "${TAILSCALE_DIR}/tailscaled.setup.log" 2>&1 &
  fi

  for _ in $(seq 1 15); do
    if tailscaled_ready; then
      print_ok "tailscaled started"
      return
    fi
    sleep 1
  done

  print_err "tailscaled did not become ready. Check ${TAILSCALE_DIR}/tailscaled.setup.log"
  exit 1
}

tailscaled_ready() {
  local status_out status_rc
  set +e
  status_out="$(tailscale_cmd status 2>&1)"
  status_rc=$?
  set -e

  if [[ ${status_rc} -eq 0 ]]; then
    return 0
  fi

  if [[ ${status_rc} -eq 1 ]] && grep -q "Logged out." <<< "${status_out}"; then
    return 0
  fi

  return 1
}

print_step "Tailscale setup for comma device"
echo "This script is intended to run on-device (for example via ssh commawifi)."

require_cmd curl
require_cmd tar
require_cmd sha256sum
require_cmd sudo
require_cmd grep
require_cmd sed
require_cmd sort
require_cmd awk
require_cmd install
require_cmd ss
require_cmd python3

if ! sudo -n true >/dev/null 2>&1; then
  print_err "Passwordless sudo is required for this installer."
  exit 1
fi

if [[ ! -c /dev/net/tun ]]; then
  print_err "/dev/net/tun is unavailable. Cannot run tailscaled with kernel TUN mode."
  exit 1
fi

ARCH="$(determine_arch)"
print_ok "Architecture: ${ARCH}"

VERSION="$(get_latest_version "${ARCH}")"
if [[ -z "${VERSION}" ]]; then
  print_err "Failed to resolve latest stable Tailscale version for ${ARCH}"
  exit 1
fi
print_ok "Latest stable version: ${VERSION}"

PKG_NAME="tailscale_${VERSION}_${ARCH}.tgz"
PKG_URL="${PKGS_URL}/${PKG_NAME}"
SUM_URL="${PKG_URL}.sha256"

TMP_DIR="$(mktemp -d)"
cleanup() {
  rm -rf "${TMP_DIR}"
}
trap cleanup EXIT

print_step "Downloading package and checksum"
curl -fsSL "${PKG_URL}" -o "${TMP_DIR}/${PKG_NAME}"
curl -fsSL "${SUM_URL}" -o "${TMP_DIR}/${PKG_NAME}.sha256"

EXPECTED_SUM="$(tr -d '[:space:]' < "${TMP_DIR}/${PKG_NAME}.sha256")"
ACTUAL_SUM="$(sha256sum "${TMP_DIR}/${PKG_NAME}" | awk '{print $1}')"
if [[ "${EXPECTED_SUM}" != "${ACTUAL_SUM}" ]]; then
  print_err "Checksum mismatch for ${PKG_NAME}"
  exit 1
fi
print_ok "Checksum verified"

print_step "Installing binaries to persistent storage"
mkdir -p "${BIN_DIR}" "${STATE_DIR}"
tar -xzf "${TMP_DIR}/${PKG_NAME}" -C "${TMP_DIR}"
EXTRACT_DIR="${TMP_DIR}/tailscale_${VERSION}_${ARCH}"
install -m 755 "${EXTRACT_DIR}/tailscale" "${BIN_DIR}/tailscale"
install -m 755 "${EXTRACT_DIR}/tailscaled" "${BIN_DIR}/tailscaled"
touch "${ENABLED_MARKER}"
print_ok "Installed tailscale and tailscaled to ${BIN_DIR}"

install_systemd_service
start_tailscaled_if_needed

DONGLE_ID="$(cat /data/params/d/DongleId 2>/dev/null || true)"
DONGLE_ID="$(echo "${DONGLE_ID}" | tr -cd '[:alnum:]')"
if [[ -n "${DONGLE_ID}" ]]; then
  NODE_HOSTNAME="comma-${DONGLE_ID,,}"
else
  NODE_HOSTNAME="$(hostname | tr '[:upper:]' '[:lower:]' | tr -cs 'a-z0-9-' '-')"
fi

print_step "Running interactive tailscale login (complete this in your browser)"
UP_LOG="${TMP_DIR}/tailscale-up.log"
set +e
tailscale_cmd up --reset --hostname "${NODE_HOSTNAME}" 2>&1 | tee "${UP_LOG}"
UP_STATUS=${PIPESTATUS[0]}
set -e

AUTH_URL="$(grep -Eo 'https://login\.tailscale\.com/[^[:space:]]+' "${UP_LOG}" | head -n 1 || true)"
if [[ -n "${AUTH_URL}" ]]; then
  echo
  echo "Auth URL:"
  echo "  ${AUTH_URL}"
  echo
fi

if [[ ${UP_STATUS} -ne 0 ]]; then
  print_err "tailscale up failed with exit code ${UP_STATUS}"
  exit ${UP_STATUS}
fi
print_ok "tailscale up completed"

print_step "Verifying connection"
tailscale_cmd status
TAILSCALE_IP="$(tailscale_cmd ip -4 | head -n 1 || true)"
if [[ -z "${TAILSCALE_IP}" ]]; then
  print_err "No Tailscale IPv4 address was assigned."
  exit 1
fi
print_ok "Tailnet IPv4: ${TAILSCALE_IP}"

if ss -lnpt | grep -E '(:22[[:space:]]|:22$)' >/dev/null 2>&1; then
  print_ok "OpenSSH is listening on port 22"
else
  print_err "OpenSSH is not listening on port 22"
  exit 1
fi

DNS_NAME="$(tailscale_cmd status --json 2>/dev/null | python3 -c "import json,sys; print((json.load(sys.stdin).get('Self') or {}).get('DNSName',''))" || true)"

echo
echo -e "${GREEN}Setup complete.${NC}"
echo "Access examples from another device on the same tailnet:"
if [[ -n "${DNS_NAME}" ]]; then
  echo "  ssh comma@${DNS_NAME}"
fi
echo "  ssh comma@${TAILSCALE_IP}"
echo
if systemd_available; then
  echo "Persistent supervisor:"
  echo "  sudo systemctl status ${SERVICE_NAME}"
fi
