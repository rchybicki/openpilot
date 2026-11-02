# Tailscale Integration for OpenPilot

This module adds a Tailscale-based remote access path for comma devices.

## What It Provides

- `manage_tailscaled.py` - Manager daemon wrapper for Tailscale
- `tailscaled.py` - Manager fallback for starting `tailscaled`
- `tailscale_supervisor.sh` - Persistent systemd-safe supervisor for `tailscaled`
- `openpilot-tailscaled.service` - Systemd unit that starts Tailscale after NetworkManager
- `bootstrap_tailscale_service.sh` - Launch-time migration helper that installs the systemd service before `build.py` if Tailscale is already enabled
- `install_tailscale.sh` - On-device installer and interactive authentication helper

## Setup Flow (On Device)

1. Deploy this branch to your comma device.
2. Run the installer on-device:

```bash
ssh commawifi 'cd /data/openpilot && ./system/tailscale/install_tailscale.sh'
```

The installer will:
- Download latest stable `tailscale`/`tailscaled` for arm64.
- Verify checksum.
- Install binaries to `/data/media/0/tailscale/bin`.
- Install and enable `openpilot-tailscaled.service`.
- Start `tailscaled` with kernel TUN mode through the persistent supervisor.
- Run `tailscale up` and print an interactive login URL.
- Verify Tailnet IP assignment and SSH listener status.

## Accessing the Device

After authentication from your browser, connect from any device in your Tailnet:

```bash
ssh comma@<magicdns-name>
# or
ssh comma@<tailscale-ipv4>
```

## Runtime Behavior

- `openpilot-tailscaled.service` starts after NetworkManager and runs a
  persistent copy of `tailscale_supervisor.sh` from `/data/media/0/tailscale`.
- On AGNOS builds where `/etc/systemd/system` is read-only, the unit is installed
  under `/run/systemd/system` and recreated by the launch bootstrap on each boot.
- `launch_chffrplus.sh` runs `bootstrap_tailscale_service.sh` before `build.py`
  so an already-enabled Tailscale install migrates to systemd before a compile
  failure can block openpilot manager startup.
- The OpenPilot process manager still starts `manage_tailscaled` as a fallback
  for devices that have not installed the systemd unit.
- Tailscale starts when all conditions are true:
  - `/data/media/0/tailscale/enabled` exists
  - Tailscale binaries are present

`tailscaled` is started even before Wi-Fi connectivity is fully available; it
handles network transitions and reconnects by itself. Startup does not depend on
openpilot Python imports, model compilation, or the process manager once the
systemd unit has been installed.
The supervisor checks the local daemon API without treating ordinary offline
periods as failures, restarts an unresponsive daemon, and restarts once after a
stale boot clock becomes valid. The launch bootstrap also refreshes and restarts
an installed supervisor when its repo copy changes.

## Disable Tailscale

To stop this integration:

```bash
ssh commawifi 'sudo systemctl disable --now openpilot-tailscaled.service; rm -f /data/media/0/tailscale/enabled && sudo pkill -f /data/media/0/tailscale/bin/tailscaled'
```
