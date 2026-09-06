# Tailscale Integration for OpenPilot

This module adds a Tailscale-based remote access path for comma devices.

## What It Provides

- `manage_tailscaled.py` - Manager daemon wrapper for Tailscale
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

  - `/data/media/0/tailscale/enabled` exists
  - Tailscale binaries are present


## Disable Tailscale

To stop this integration:

```bash
```
