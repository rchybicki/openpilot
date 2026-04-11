# Tailscale Integration for OpenPilot

This module adds a Tailscale-based remote access path for comma devices.

## What It Provides

- `manage_tailscaled.py` - Manager daemon wrapper for Tailscale
- `tailscaled.py` - Starts and supervises the `tailscaled` process
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
- Start `tailscaled` with kernel TUN mode.
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

- The OpenPilot process manager starts `manage_tailscaled` as a daemon.
- `manage_tailscaled` only starts `tailscaled` when all conditions are true:
  - `SshEnabled` is true
  - `/data/media/0/tailscale/enabled` exists
  - Tailscale binaries are present
  - network is available

## Disable Tailscale

To stop this integration:

```bash
ssh commawifi 'rm -f /data/media/0/tailscale/enabled && sudo pkill -f /data/media/0/tailscale/bin/tailscaled'
```
