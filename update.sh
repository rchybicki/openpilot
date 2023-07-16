git fetch origin
git reset --hard FETCH_HEAD
git submodule update -f
./restart.sh
selfdrive/debug/filter_log_message.py