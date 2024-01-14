git fetch origin
git reset --hard FETCH_HEAD
git submodule update -f
./restart.sh
tmux -CC a