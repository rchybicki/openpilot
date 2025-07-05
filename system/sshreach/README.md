# SSH Reach Integration for OpenPilot

This module integrates SSH Reach Me (sshreach.me) into OpenPilot, providing persistent reverse SSH access to your comma device even when it's behind NAT or using cellular connectivity.

## Overview

SSH Reach creates a reverse SSH tunnel from your comma device to SSH Reach servers, allowing you to access your device remotely without port forwarding or dynamic DNS.

## Components

- `sshreach_client.py` - The SSH Reach client script that maintains the reverse tunnel
- `sshreachd.py` - Service wrapper that manages the SSH Reach client
- `manage_sshreachd.py` - Daemon manager that ensures the service stays running
- `install_sshreach.sh` - Installation script to set up SSH Reach on your device

## Installation

1. **Prerequisites**:
   - SSH access to your comma device (via `commawifi` hostname)
   - Your SSH private key at `~/.ssh/id_ed25519`
   - SSH Reach account and configuration (in `sshreach_client.py`)

2. **Run the installation script**:
   ```bash
   cd system/sshreach
   ./install_sshreach.sh
   ```

   This script will:
   - Copy your SSH private key to the device's persistent storage (`/data/media/0/sshreach/`)
   - Enable SSH in device parameters
   - Restart OpenPilot to start the SSH Reach service

## How It Works

1. The service is managed by OpenPilot's process manager as a daemon process
2. It only runs when:
   - SSH is enabled (`SshEnabled` parameter is true)
   - The private key exists at `/data/media/0/sshreach/id_ed25519`
   - Network connection is available
3. The private key is stored in `/data/media/0/` which persists across Agnos OS updates
4. The service automatically reconnects if the connection is lost

## Troubleshooting

### Check service status:
```bash
ssh comma@commawifi
journalctl -u comma | grep sshreach
```

### Verify private key exists:
```bash
ssh comma@commawifi
ls -la /data/media/0/sshreach/
```

### Enable SSH manually if needed:
```bash
ssh comma@commawifi
cd /data/openpilot
python3 -c "from openpilot.common.params import Params; Params().put_bool('SshEnabled', True)"
```

### Restart the service:
```bash
ssh comma@commawifi
sudo systemctl restart comma
```

## Security Notes

- Never commit your private key to the repository
- The private key is stored with 600 permissions on the device
- SSH Reach uses your private key for authentication to their servers
- Make sure your SSH Reach account is properly secured

## Logs

The service logs to the standard OpenPilot logging system. View logs with:
```bash
journalctl -u comma | grep -E "(sshreach|manage_sshreachd)"
```

## Uninstall

To disable SSH Reach:
1. SSH into your device
2. Remove the private key: `rm -rf /data/media/0/sshreach`
3. Disable SSH if desired: `python3 -c "from openpilot.common.params import Params; Params().put_bool('SshEnabled', False)"`
4. Restart OpenPilot: `sudo systemctl restart comma`