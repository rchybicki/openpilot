current_branch="$(git symbolic-ref --quiet --short HEAD 2>/dev/null || true)"
if [ -n "$current_branch" ]; then
  upstream_ref="$(git rev-parse --abbrev-ref --symbolic-full-name '@{u}' 2>/dev/null || true)"
  if [ -n "$upstream_ref" ]; then
    remote_name="${upstream_ref%%/*}"
    remote_branch="${upstream_ref#*/}"
    git fetch "$remote_name" "refs/heads/$remote_branch"
  else
    git fetch origin "refs/heads/$current_branch"
  fi
else
  git fetch origin
fi
git reset --hard FETCH_HEAD
git submodule update -f
rm -f /data/openpilot/prebuilt
sudo reboot
