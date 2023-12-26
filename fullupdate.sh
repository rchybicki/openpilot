git fetch origin
git reset --hard FETCH_HEAD
git submodule update -f
sudo systemctl stop comma
rm -f /data/openpilot/prebuilt
sudo reboot