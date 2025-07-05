#!/bin/bash

# SSH Reach Installation Script
# This script copies your private key to the OpenPilot device and sets up SSH Reach

set -e

# Configuration
DEVICE_HOST="commawifi"
DEVICE_USER="comma"
LOCAL_KEY_PATH="$HOME/.ssh/id_ed25519"
REMOTE_SSHREACH_DIR="/data/media/0/sshreach"
REMOTE_KEY_PATH="$REMOTE_SSHREACH_DIR/id_ed25519"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}SSH Reach Installation Script${NC}"
echo "==============================="

# Check if local private key exists
if [ ! -f "$LOCAL_KEY_PATH" ]; then
    echo -e "${RED}Error: Private key not found at $LOCAL_KEY_PATH${NC}"
    echo "Please ensure your SSH private key exists at the expected location."
    exit 1
fi

echo -e "${YELLOW}Using private key: $LOCAL_KEY_PATH${NC}"
echo -e "${YELLOW}Target device: $DEVICE_HOST${NC}"

# Test SSH connection
echo -n "Testing SSH connection to $DEVICE_HOST... "
if ssh -o ConnectTimeout=5 -o StrictHostKeyChecking=no "$DEVICE_USER@$DEVICE_HOST" "echo 'OK'" &>/dev/null; then
    echo -e "${GREEN}Success${NC}"
else
    echo -e "${RED}Failed${NC}"
    echo "Please ensure:"
    echo "  1. Your device is connected to the same network"
    echo "  2. SSH is enabled on the device"
    echo "  3. You can reach the device at hostname 'commawifi'"
    exit 1
fi

# Create remote directory
echo -n "Creating remote directory $REMOTE_SSHREACH_DIR... "
if ssh "$DEVICE_USER@$DEVICE_HOST" "mkdir -p $REMOTE_SSHREACH_DIR"; then
    echo -e "${GREEN}Done${NC}"
else
    echo -e "${RED}Failed${NC}"
    exit 1
fi

# Copy private key
echo -n "Copying private key to device... "
if scp -q "$LOCAL_KEY_PATH" "$DEVICE_USER@$DEVICE_HOST:$REMOTE_KEY_PATH"; then
    echo -e "${GREEN}Done${NC}"
else
    echo -e "${RED}Failed${NC}"
    exit 1
fi

# Set correct permissions
echo -n "Setting correct permissions on private key... "
if ssh "$DEVICE_USER@$DEVICE_HOST" "chmod 600 $REMOTE_KEY_PATH"; then
    echo -e "${GREEN}Done${NC}"
else
    echo -e "${RED}Failed${NC}"
    exit 1
fi

# Check if SSH is enabled
echo -n "Checking if SSH is enabled... "
SSH_ENABLED=$(ssh "$DEVICE_USER@$DEVICE_HOST" "cat /data/params/d/SshEnabled 2>/dev/null || echo '0'")
if [ "$SSH_ENABLED" = "1" ]; then
    echo -e "${GREEN}Yes${NC}"
else
    echo -e "${YELLOW}No${NC}"
    echo "SSH is not enabled. Please enable it in the device settings."
fi

# Install websockets module if needed
echo -n "Checking for websockets module... "
if ssh "$DEVICE_USER@$DEVICE_HOST" "cd /data/openpilot && /usr/local/pyenv/versions/3.11.4/bin/python3 -c 'import websockets' 2>/dev/null"; then
    echo -e "${GREEN}Already installed${NC}"
else
    echo -e "${YELLOW}Not found, installing...${NC}"
    if ssh "$DEVICE_USER@$DEVICE_HOST" "cd /data/openpilot && /usr/local/pyenv/versions/3.11.4/bin/pip3 install websockets"; then
        echo -e "${GREEN}Installed successfully${NC}"
    else
        echo -e "${RED}Failed to install${NC}"
        echo "The service will try to install it when it starts"
    fi
fi

# Restart OpenPilot to start the service
echo -n "Restarting OpenPilot to start SSH Reach service... "
if ssh "$DEVICE_USER@$DEVICE_HOST" "sudo systemctl restart comma"; then
    echo -e "${GREEN}Done${NC}"
else
    echo -e "${YELLOW}Warning: Could not restart OpenPilot${NC}"
    echo "You may need to restart it manually with: sudo systemctl restart comma"
fi

# Wait for service to start
echo "Waiting for OpenPilot to restart (this takes about 30-45 seconds)..."
for i in {1..9}; do
    echo -n "."
    sleep 5
done
echo " Done"

# Verification and debugging
echo ""
echo -e "${YELLOW}Verifying installation...${NC}"
echo ""

# Check if manage_sshreachd is running
echo -n "Checking if SSH Reach manager is running... "
MANAGER_PID=$(ssh "$DEVICE_USER@$DEVICE_HOST" "ps aux | grep -E 'manage_sshreachd' | grep -v grep | awk '{print \$2}' | head -1")
if [ -n "$MANAGER_PID" ]; then
    echo -e "${GREEN}Yes (PID: $MANAGER_PID)${NC}"
else
    echo -e "${RED}No${NC}"
fi

# Check if sshreachd is running
echo -n "Checking if SSH Reach daemon is running... "
DAEMON_PID=$(ssh "$DEVICE_USER@$DEVICE_HOST" "ps aux | grep -E 'sshreachd\.py' | grep -v grep | awk '{print \$2}' | head -1")
if [ -n "$DAEMON_PID" ]; then
    echo -e "${GREEN}Yes (PID: $DAEMON_PID)${NC}"
else
    echo -e "${RED}No${NC}"
fi

# Check if SSH reach client is running
echo -n "Checking if SSH Reach client is running... "
CLIENT_PID=$(ssh "$DEVICE_USER@$DEVICE_HOST" "ps aux | grep -E 'python.*sshreach_client' | grep -v grep | awk '{print \$2}' | head -1")
if [ -n "$CLIENT_PID" ]; then
    echo -e "${GREEN}Yes (PID: $CLIENT_PID)${NC}"
else
    echo -e "${RED}No${NC}"
fi

# Also check the sshreachd subprocess
echo -n "Checking sshreachd subprocess... "
SUBPROCESS=$(ssh "$DEVICE_USER@$DEVICE_HOST" "ps aux | grep -E 'system.sshreach.sshreachd' | grep -v grep")
if [ -n "$SUBPROCESS" ]; then
    echo -e "${GREEN}Running${NC}"
    echo "  $SUBPROCESS"
else
    echo -e "${RED}Not found${NC}"
fi

# Check for any SSH tunnels
echo -n "Checking for active SSH tunnels... "
TUNNEL_COUNT=$(ssh "$DEVICE_USER@$DEVICE_HOST" "ps aux | grep -E 'ssh.*-R.*sshreach.me' | grep -v grep | wc -l")
if [ "$TUNNEL_COUNT" -gt 0 ]; then
    echo -e "${GREEN}Yes ($TUNNEL_COUNT tunnel(s))${NC}"
else
    echo -e "${YELLOW}No active tunnels${NC}"
fi

# Show recent logs
echo ""
echo -e "${YELLOW}Recent SSH Reach logs:${NC}"
ssh "$DEVICE_USER@$DEVICE_HOST" "journalctl -u comma --since '2 minutes ago' | grep -i sshreach | tail -10"

# Check for errors
echo ""
echo -e "${YELLOW}Checking for errors:${NC}"
ERROR_COUNT=$(ssh "$DEVICE_USER@$DEVICE_HOST" "journalctl -u comma --since '5 minutes ago' | grep -i 'sshreach.*error' | wc -l")
if [ "$ERROR_COUNT" -gt 0 ]; then
    echo -e "${RED}Found $ERROR_COUNT error(s) in logs:${NC}"
    ssh "$DEVICE_USER@$DEVICE_HOST" "journalctl -u comma --since '5 minutes ago' | grep -i 'sshreach.*error' | tail -5"
else
    echo -e "${GREEN}No recent errors found${NC}"
fi

echo ""
echo -e "${GREEN}Installation complete!${NC}"
echo ""

# Summary
if [ -n "$MANAGER_PID" ] && [ -n "$DAEMON_PID" ]; then
    echo -e "${GREEN}✓ SSH Reach services are running${NC}"
    if [ -n "$CLIENT_PID" ] || [ "$TUNNEL_COUNT" -gt 0 ]; then
        echo -e "${GREEN}✓ SSH Reach client is active${NC}"
        echo ""
        echo "Your device should now be accessible via SSH Reach!"
    else
        echo -e "${YELLOW}⚠ SSH Reach client is not running yet${NC}"
        echo ""
        echo "This may be normal if the service is still starting."
        echo "Wait a moment and run this script again to check status."
    fi
else
    echo -e "${RED}✗ SSH Reach services are not running properly${NC}"
    echo ""
    echo "Troubleshooting steps:"
    echo "1. Check if OpenPilot is running: ssh $DEVICE_USER@$DEVICE_HOST 'systemctl status comma'"
    echo "2. Check for errors: ssh $DEVICE_USER@$DEVICE_HOST 'journalctl -u comma | grep -i error | tail -20'"
    echo "3. Try running manually: ssh $DEVICE_USER@$DEVICE_HOST 'cd /data/openpilot && python3 system/sshreach/sshreach_client.py /data/media/0/sshreach/id_ed25519 console'"
fi

echo ""
echo "To monitor the service in real-time, run:"
echo "  ssh $DEVICE_USER@$DEVICE_HOST 'journalctl -u comma -f | grep -i sshreach'"