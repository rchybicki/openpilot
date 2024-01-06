git fetch origin
git reset --hard FETCH_HEAD
git submodule update -f
pkill -f thermald
rm -f /data/openpilot/prebuilt
sudo reboot