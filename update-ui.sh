git fetch origin
git reset --hard FETCH_HEAD
git submodule update -f
rm -f /data/openpilot/prebuilt
./restart.sh
selfdrive/debug/filter_log_message.py