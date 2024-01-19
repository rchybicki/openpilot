git fetch origin
git reset --hard FETCH_HEAD
git submodule update -f
./restart.sh
sleep 3
tmux -CC a