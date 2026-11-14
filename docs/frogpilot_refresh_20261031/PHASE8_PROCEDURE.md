# Phase 8 operating procedure: move the device to `!my-fp-1031`

Status: revision 5, for plan review. The Astra xhigh reviews of
2026-11-02 found REWORK three times: in revision 1 (`f3a2cadec7`, seven
Act-on items), revision 2 (`ede4f0ad8b`, four) and revision 3
(`7b11e2a78e`, two), and approved revision 4 (`50a498eaa2`). The run of
2026-11-14 stopped at box 4.4 with nothing changed: the process check of the
switch script counted the script itself (appendix B.4). Revision 5 corrects
that check in the switch, R2 and R2-key scripts. Do not run box 4.4, R2 or
R2-key before the review of revision 5 closes.

Sources: `PLAN.md` phase 8 and the validation matrix "device" row;
`AGENTS.md` "Device Update Workflow"; `DECISIONS.md` F (network rows);
`CLOSURE.md` (two log schemas, `Params::clearAll`).

Decisions (Radek): the device switches its checked-out branch to
`!my-fp-1031`. `!my-fp-new` stays untouched for rollback. No force-push. The
first transition runs parked, with the ignition off.

## How to use this checklist

- The operator runs every box from the host, in order, in one zsh session.
  Each box has the exact command and a PASS line. Save each output under
  `$P8`, with the box number in the file name. Write PASS or FAIL and the
  time into `$P8/RECORD.md`.
- **[RADEK: physical]**: Radek acts at the car. **[RADEK: go]**: Radek
  approves before the box runs. Nobody needs to be at the car for the other
  boxes.
- On FAIL: do what the box says (STOP, physical recovery, R1, R2, R2-key or
  resume, all in section 6). STOP means: change nothing more on the device,
  report, and wait.
- Each host block starts with `setopt pipefail no_bang_hist`. Variables
  next to a colon are written `${VAR}:` (zsh reads `$VAR:s` as a
  substitution modifier). A block judges a value only after the command that
  made it succeeded.
- Device scripts go to the device as `ssh $DEV bash -s -- args <<'DEV'`. The
  quoted heredoc reaches the device unchanged. Each script runs with
  `set -euo pipefail` and stops at the first error. Do not type device
  commands into an interactive device shell.
- Read-only device boxes only use `ps`, `cat`, `ls`, `stat`, `pgrep`,
  `tmux capture-pane` and copies of closed log files. No `SubMaster`,
  `sub_sock` or other msgq reader on the device, and no import of driving
  modules there (AGENTS.md). Only these scripts change the device: switch,
  R2, R2-key and resume.
- Every ssh call has a hard limit: `timeout -k 5 <seconds> ssh -o
  BatchMode=yes -o ConnectTimeout=8 -o ServerAliveInterval=5 -o
  ServerAliveCountMax=3`. `timeout` bounds the whole call, and the
  ServerAlive options end a dead connection after about 15 s. The limit
  ends the host side only; a device script can still be running, so the box
  says what to check next. The wait loops (4.5, 4.6) run in Python, with a
  `time.monotonic()` deadline and a per-call timeout that never passes the
  deadline. Each states its arithmetic. Their limits are literal constants:
  no environment variable or setting changes them, and a resumed shell waits
  as long as a new one. The host rehearsal runs a copy of the loops with
  scaled constants.
- Do not tap Settings > Software > Full Update, and do not push to
  `!my-fp-new` or `!my-fp-1031` while this procedure runs (except box 2.3).
- Save the appendix files `A1.py`, `A2.py`, `A3.py` and `A4.sh` in `$P8`
  after box 0.2.
- Plan about 2.5 hours with the car parked. The longest step is the first
  build after the switch.
- Host rehearsal of the device scripts: `tools/fullupdate/phase8_rehearse.py`
  (appendix B). Run it after every change to this file.

## Facts this procedure relies on

- **Switch method.** The switch does not run `fullupdate.sh`. That script
  reboots from inside itself, so nothing could check the result between its
  reset and the reboot. Box 4.4 runs the same parked path by hand, with
  checks around each step. The steps are: fetch (box 2.6), stop openpilot,
  make the staged overlay ineligible, `git reset --hard` to a SHA held
  locally, delete `prebuilt`, verify, then reboot. A rollback (R1) runs the
  same script in rollback mode with the old SHA and needs no network.
  Compared with `fullupdate.sh`, the manual path adds the unit stop, both
  update locks and the overlay invalidation. It drops the reboot
  supervisor and the live stock-SCC handoff: Radek confirms the car is
  parked with the ignition off instead. It also drops `git submodule
  update` (neither tree has submodules) and the staging banner. `PLAN.md`
  phase 8 records this exception. `fullupdate.sh`
  (`fullupdate.sh:538-555`) stays the deploy path after the move.
- **The openpilot service and quiescence.** The manager's cgroup names its
  systemd unit. Box 1.3 reads it from `/proc/<manager pid>/cgroup`; AGENTS
  notes call the unit `comma`. It records the unit's cgroup
  (`systemctl show -p ControlGroup`) and proves the manager is in it.
  "Quiesced" means all of these: `systemctl stop` (at most 120 s)
  succeeded; `systemctl is-active` reports `inactive` or `failed` (both
  stopped states, `failed` when the last run ended with an error); and the
  unit's whole cgroup tree, child cgroups included, has no PID on any
  mounted hierarchy (`/sys/fs/cgroup`, `/unified`, `/systemd`). A listing or
  read error in that tree, such as an unreadable `cgroup.procs`, counts as
  not empty: the script stops and changes nothing. Name checks
  catch openpilot, update and flashing processes outside that cgroup, for
  example a `fullupdate.sh` supervisor started over SSH. Each script leaves
  out the lines equal to its own command line (itself and its subshells):
  its arguments hold the cgroup path, which the name pattern matches. If it
  cannot read its own command line, it counts that as a running process. The
  switch, R2 and R2-key reset or restore only after this. The update locks
  (`/data/fullupdate.lock`, `/data/fullupdate_reboot.lock`) also show a
  `fullupdate.sh` run.
- **Automatic updates** (`AutomaticUpdates`, 1 on the device): every 5
  minutes `frogpilot_process` compares `HEAD` with the remote branch and
  starts `fullupdate.sh` when they differ (old branch
  `frogpilot_functions.py:245`, new branch `:308`). After the switch, every
  push to `!my-fp-1031` reaches the car within about 5 minutes.
- **The overlay updater.** `updated` (`system/updated/updated.py`) runs on
  both branches. At each loop, `init_overlay` (`:130-178`) rebuilds the
  staging area and touches `/data/openpilot/.overlay_init` when `.git`
  changed, and `finalize_update` (`:180-206`) writes
  `/data/safe_staging/finalized/.overlay_consistent`. At boot,
  `launch_chffrplus.sh:52-75` installs the finalized overlay in place of
  `/data/openpilot` when `.overlay_init` exists, `.git` is not newer than it,
  and `.overlay_consistent` exists. A running `updated` can therefore
  refresh `.overlay_init` after a reset. The switch stops it first, deletes
  both marker files, and checks before the reboot that they stay deleted.
- **AGNOS.** `agnos_init` flashes AGNOS when `/VERSION` differs from
  `AGNOS_VERSION` after `launch_env.sh` ran (`launch_chffrplus.sh:22-36`).
  `launch_env.sh` keeps an inherited `AGNOS_VERSION` (`:18`) and then sources
  `/data/launch_env.sh` (`:30-33`). Without an inherited value,
  `!my-fp-1031` sets 16, or 12.8 on a "comma tici/tizi" device. `!my-fp-new`
  sets 12.8, then evaluates `frogpilot/system/environment_variables`. Box 2.7
  runs each tree's own `launch_env.sh` in the environment the boot gives it
  (the environment of the running `launch_chffrplus.sh`). The candidate's
  `c3_agnos.json` is byte-identical to the rollback tree's `agnos.json`.
- **Storage.** The NVMe guard (`system/manager/helpers.py:66-127`) probes
  `/data/media` with `mkdtemp` and `rmdir` within 5 s. If the probe fails, it
  sets `LOG_ROOT=/data/realdata_fallback` and the manager raises
  `Offroad_StorageMissing` (`manager.py:38,50`). A dead NVMe can stay
  mounted. Box 1.4 runs the same probe.
- **Free space.** `/data` (the eMMC; `/data/media` is the NVMe) needs about
  3.0 GB, measured on the host on 2026-11-02: fetched objects 0.6 GB (0.23 GB
  on disk), the changed working tree 0.5 GB, build outputs 0.8 GB, the scons
  cache `/data/scons_cache` 0.8 GB (`SConstruct:214`), and the new base's
  code backup in `/data/backups` 0.3 GB (`git archive | zstd -3`). The
  thresholds keep a 1.7x margin: 5 GB before the fetch, 4 GB after the fetch
  and after the backups.
- **Manager start** (`system/manager/manager.py:42-64`) runs `clear_all` for
  the four CLEAR flags, deletes stored keys that the new registry lacks, then
  runs `migrate_params`, syncs `/cache/params` and writes defaults. The probe
  on a copy of today's store
  (`~/.route_sync/work/fp1031/port-scratch/migrate_report_b1.json`) found 50
  keys deleted (8 PERSISTENT: `AvailableModels`, `AvailableModelNames`,
  `DrivingModelName`, `DrivingModelVersion`, `OverpassRequests`,
  `SpeedLimits`, `SpeedLimitsFiltered`, `TinygradUpdateAvailable`; and the
  unregistered `IdentificationTestArmed`), 2 rewritten (`MapdSettings` `{}`
  to `{"settings_version": 2}`, `SecOCKeys` `''` to `[]`), 17 defaults added
  and 55 deleted from `/cache/params`. The probe at `08925e26c9` gives the
  same lists.
- **Retyped keys.** `AggressivePersonalityProfile`,
  `RelaxedPersonalityProfile` and `StandardPersonalityProfile` change from
  BOOL to STRING, `PauseAOLOnBrake` from BOOL to INT, and `SecOCKeys` from
  STRING to JSON. The device stores `0` or an empty value in each.
- **Params stores.** `/data/params/d` is a symlink to
  `/data/params/.tmp_XXXXXX` (`common/params.cc:34-62`), and `/cache/params/d`
  has the same layout. Backup and restore work on the directory contents.
- **Onboarding.** The new onboarding shows the telemetry consent page until
  it is answered (`selfdrive/ui/qt/offroad/onboarding.h:97`). Radek's answer
  is "No Thanks" (`DECISIONS.md` F).
- **Onroad-only processes.** `modeld` and `card` run only with the ignition
  on (`system/manager/process_config.py:109,121`). The model and
  fingerprint checks therefore run in 5b, with the ignition on in Park.
- **Uploads and new services.** Today `NoUploads` is 1. The Pond and the
  speed-limit sign reader never start on this branch
  (`process_config.py:81-88`).

## 0. Start

- [ ] **0.1** **[RADEK: go]** Radek approves phase 8 and the time slot.
  Plan review of this procedure is closed.
- [ ] **0.2** Set the session variables and the record directory.

<!-- p8:session -->
```sh
setopt pipefail no_bang_hist
export DEV=comma                    # if a box cannot reach it: export DEV=commawifi, run the box again
export REPO=~/Repos/openpilot-rch
export PORT=~/.route_sync/work/fp1031/port-wt
export TIP=~/.route_sync/work/fp1031/subtract-wt   # old cereal, for logs recorded on !my-fp-new
export REVIEWED=08925e26c9          # last code head with a SHIP review of record
export MIN_FREE=5000000000
command -v timeout >/dev/null || echo "MISSING: timeout (GNU coreutils), needed by every box"
export P8=~/.route_sync/work/fp1031/phase8/run_$(date -u +%Y%m%dT%H%MZ)
mkdir -p "${P8}" && chmod 700 "${P8}" && touch "${P8}/RECORD.md" && echo "${P8}"
source "${REPO}/.venv/bin/activate"
```

PASS: the directory exists, and no `MISSING` line. Write the variables into
`RECORD.md`. Do the same for `UNIT`, `UNIT_CGROUP`, `OLD_SHA`, `ORIGIN_NEW`
and `CAND` when boxes 1.3, 1.5 and 2.1 set them. After a lost session, export the same values again from
`RECORD.md`; do not run 0.2 again.

## 1. Preconditions

- [ ] **1.1** **[RADEK: physical]** The car is parked in P with the
  ignition off. Nobody drives it for the next 2.5 hours. The 12 V battery is
  in good condition: the device builds for up to an hour with the ignition
  off.
- [ ] **1.2** The device answers (one try, 8 s limit).

<!-- p8:reach -->
```sh
setopt pipefail no_bang_hist
timeout -k 2 20 ssh -o BatchMode=yes -o ConnectTimeout=8 -o ServerAliveInterval=5 -o ServerAliveCountMax=3 $DEV true && echo REACHABLE
```

PASS: `REACHABLE`. If both `comma` and `commawifi` fail, go to the physical
recovery in section 6.

- [ ] **1.3** Record the device state and identify the openpilot unit.

<!-- p8:state -->
```sh
setopt pipefail no_bang_hist
timeout -k 5 60 ssh -o BatchMode=yes -o ConnectTimeout=8 -o ServerAliveInterval=5 -o ServerAliveCountMax=3 $DEV bash -s <<'DEV' | tee "${P8}/01_device_state.txt"
set -euo pipefail
cd /data/openpilot
echo "utc $(date -u +%FT%TZ) uptime_s $(cut -d' ' -f1 /proc/uptime) boot_id $(cat /proc/sys/kernel/random/boot_id)"
echo "model $(tr -d '\0' < /sys/firmware/devicetree/base/model)"
echo "agnos $(cat /VERSION)"
echo "head $(git rev-parse HEAD)"
echo "branch $(git symbolic-ref --short HEAD)"
echo "local_new $(git rev-parse 'refs/heads/!my-fp-new')"
git log -1 --format='commit %H %cI %s'
echo "origin $(git remote get-url origin)"
git config --get-all remote.origin.fetch | sed 's/^/fetch_refspec /' || true
git config --get-regexp '^branch\.' | sed 's/^/branch_config /' || true
echo "tracked_changes $(git status --porcelain --untracked-files=no | wc -l)"
if git show-ref --verify --quiet 'refs/heads/!my-fp-1031'; then echo "branch_1031 EXISTS $(git rev-parse 'refs/heads/!my-fp-1031')"; else echo "branch_1031 absent"; fi
for k in IsOnroad IsEngaged AutomaticUpdates LiveUpdateHandoffState UpdaterState Offroad_NeosUpdate Offroad_StorageMissing; do
  echo "param ${k}=$(cat /data/params/d/$k 2>/dev/null || echo '<absent>')"
done
mgr=$(pgrep -f '[m]anager\.py' | head -n1 || true)
if [ -n "$mgr" ]; then
  cg=$(grep -m1 '\.service$' "/proc/$mgr/cgroup" || true)
  unit=$(printf '%s\n' "$cg" | sed -n 's#.*/\([^/]*\.service\)$#\1#p')
  echo "manager_pid $mgr unit ${unit:-<none>} cgroup ${cg##*:}"
  if [ -n "$unit" ]; then
    systemctl show -p Id -p ActiveState -p Restart "$unit" | sed 's/^/unit_/'
    ucg=$(systemctl show -p ControlGroup --value "$unit")
    echo "unit_cgroup ${ucg:-<none>}"
    if [ -n "$ucg" ]; then
      roots=""
      for r in /sys/fs/cgroup /sys/fs/cgroup/unified /sys/fs/cgroup/systemd; do [ ! -d "$r$ucg" ] || roots="$roots $r"; done
      echo "cgroup_roots${roots:- <none>}"
      members=""
      read_ok=yes
      for r in $roots; do
        files=$(find "$r$ucg" -name cgroup.procs) || read_ok=no
        while IFS= read -r f; do [ -z "$f" ] || members="$members$(cat "$f" 2>/dev/null || echo READ_ERROR)"$'\n'; done <<< "$files"
      done
      case "$members" in *READ_ERROR*) read_ok=no;; esac
      if [ "$read_ok" = yes ] && printf '%s' "$members" | grep -qx "$mgr"; then echo "cgroup_has_manager yes"; else echo "cgroup_has_manager no (read_ok ${read_ok})"; fi
    fi
  fi
else
  echo "manager_pid <none> unit <none>"
fi
echo "supervisor_procs $(pgrep -f '[_]_reboot_when_parked' | wc -l) fullupdate_procs $(pgrep -f '[f]ullupdate\.sh' | wc -l) agnos_procs $(pgrep -f '[a]gnos\.py|[h]ardware/tici/updater' | wc -l)"
if flock -n /data/fullupdate.lock true; then echo "update_lock free"; else echo "update_lock HELD"; fi
if flock -n /data/fullupdate_reboot.lock true; then echo "supervisor_lock free"; else echo "supervisor_lock HELD"; fi
echo "cancel_file $([ -e /data/fullupdate_reboot.cancel ] && echo present || echo absent)"
echo "overlay_init $([ -e .overlay_init ] && echo present || echo absent) overlay_consistent $([ -e /data/safe_staging/finalized/.overlay_consistent ] && echo present || echo absent) merged_mounted $(mountpoint -q /data/safe_staging/merged && echo yes || echo no)"
echo "fallback_entries $(ls /data/realdata_fallback 2>/dev/null | wc -l)"
tail -n 15 /data/fullupdate_reboot.log 2>/dev/null | sed 's/^/reboot_log /' || true
tail -n 15 /data/fullupdate.log 2>/dev/null | sed 's/^/update_log /' || true
DEV
export UNIT=$(awk '$1=="manager_pid" {print $4}' "${P8}/01_device_state.txt")
export UNIT_CGROUP=$(awk '$1=="unit_cgroup" {print $2}' "${P8}/01_device_state.txt")
export DEVICE_HEAD=$(awk '$1=="head" {print $2}' "${P8}/01_device_state.txt")
echo "UNIT=${UNIT} UNIT_CGROUP=${UNIT_CGROUP} DEVICE_HEAD=${DEVICE_HEAD}"
```

PASS, all of these:
- `unit` names a `.service` (expected `comma.service`), with
  `unit_ActiveState=active`. `unit_cgroup` is its cgroup (expected
  `/system.slice/comma.service`), `cgroup_roots` lists at least one
  hierarchy, and `cgroup_has_manager yes`. Otherwise STOP: the switch and
  the restores prove that openpilot stopped through this cgroup.
- `branch` is `!my-fp-new` and `local_new` equals `head`; `origin` is
  `git@github.com:rchybicki/openpilot.git`; `tracked_changes` is 0
  (otherwise STOP and show Radek the changes); `branch_1031 absent`
  (otherwise STOP and look at the earlier attempt).
- `IsOnroad` and `IsEngaged` are 0 or absent; `LiveUpdateHandoffState` is
  absent or empty; `UpdaterState` is `idle`; `Offroad_NeosUpdate` and
  `Offroad_StorageMissing` are absent.
- `supervisor_procs 0`, `fullupdate_procs 0`, `agnos_procs 0`,
  `update_lock free` and `supervisor_lock free`.
- Record `boot_id`, `fallback_entries` and the overlay state. A finalized
  overlay is not a FAIL here; box 4.4 makes it ineligible.

- [ ] **1.4** Storage health: the NVMe guard's probe, the storage alert and
  free space.

<!-- p8:storage -->
```sh
setopt pipefail no_bang_hist
timeout -k 5 60 ssh -o BatchMode=yes -o ConnectTimeout=8 -o ServerAliveInterval=5 -o ServerAliveCountMax=3 $DEV bash -s -- "${MIN_FREE}" <<'DEV' | tee -a "${P8}/01_storage.txt"
set -euo pipefail
fail() { echo "STORAGE_CHECK STOP: $1"; exit 4; }
min=${1:?threshold in bytes}
mountpoint -q /data/media || fail "/data/media is not mounted"
probe=/tmp/p8_storage_probe.$$
rm -f "$probe"
# the guard's probe (system/manager/helpers.py:102-118): mkdtemp + rmdir in /data/media within 5 s; a dead NVMe can block in
# D state, so the probe runs detached and this script stops waiting after 5 s
( d=$(mktemp -d /data/media/.p8probe.XXXXXX) && rmdir "$d" && echo ok > "$probe" ) </dev/null >/dev/null 2>&1 &
for i in 1 2 3 4 5 6 7 8 9 10; do [ -s "$probe" ] && break; sleep 0.5; done
[ -s "$probe" ] || fail "the write probe on /data/media failed or did not finish in 5 s"
rm -f "$probe"
[ ! -e /data/params/d/Offroad_StorageMissing ] || fail "Offroad_StorageMissing is set"
avail=$(df -B1 --output=avail /data | tail -n1 | tr -d ' ') || fail "df /data failed"
echo "data_avail_bytes $avail (needs $min)"
[ "$avail" -ge "$min" ] || fail "/data has $avail bytes free"
echo "STORAGE_CHECK PASS"
DEV
```

PASS: `STORAGE_CHECK PASS`. `STORAGE_CHECK STOP`: STOP. Do not switch with
a dead or full disk. A power cut clears a dead NVMe (2026-10-25), and a full
`/data` needs Radek's decision on what to delete.

- [ ] **1.5** Origin state and the rollback target (on the host).

<!-- p8:origin -->
```sh
setopt pipefail no_bang_hist
git -C "${REPO}" ls-remote origin 'refs/heads/!my-fp-new' 'refs/heads/!my-fp-1031' \
  'refs/heads/backup/!my-fp-new_10-31_pre-fp1031-refresh' | tee "${P8}/01_origin.txt"
export ORIGIN_NEW=$(awk '$2=="refs/heads/!my-fp-new" {print $1}' "${P8}/01_origin.txt")
export OLD_SHA=${DEVICE_HEAD}
git -C "${REPO}" cat-file -e "${OLD_SHA}^{commit}" && git -C "${REPO}" cat-file -e "${ORIGIN_NEW}^{commit}" && echo BOTH_KNOWN
git -C "${REPO}" diff --name-only "${OLD_SHA}" "${ORIGIN_NEW}" -- . ':(exclude)tools' ':(exclude)docs'; echo "end of non-tool changes"
echo "OLD_SHA=${OLD_SHA} ORIGIN_NEW=${ORIGIN_NEW}"
```

PASS: `!my-fp-new` and the backup branch are on origin, and `BOTH_KNOWN`
prints. (On 2026-11-14 the car runs `37ac68c984`, one modeld line after
`f77b26830f`; the candidate carries it as `2322fadf93`.) If it does not, run `git -C "${REPO}" fetch origin 'refs/heads/!my-fp-new'`
and repeat. Nothing is listed before "end of non-tool changes". The rollback
target is `OLD_SHA`, the exact commit the car runs now. Record it. On
2026-11-02, `f77b26830f..6272ccdbaa` changes only `tools/` and `docs/`.

- [ ] **1.6** The rollback target is complete on the device without the
  network.

<!-- p8:offline-check -->
```sh
setopt pipefail no_bang_hist
timeout -k 5 300 ssh -o BatchMode=yes -o ConnectTimeout=8 -o ServerAliveInterval=5 -o ServerAliveCountMax=3 $DEV bash -s -- "${OLD_SHA}" <<'DEV' | tee "${P8}/01_offline_rollback.txt"
set -euo pipefail
cd /data/openpilot
sha=${1:?sha}
git cat-file -e "${sha}^{commit}" || { echo "OFFLINE_ROLLBACK STOP: ${sha} is not a local commit"; exit 6; }
total=$(git ls-tree -r "${sha}" | wc -l)
missing=$(git ls-tree -r "${sha}" | awk '{print $3}' | git cat-file --batch-check | grep -c ' missing$' || true)
echo "commit ${sha} tree $(git rev-parse "${sha}^{tree}") entries ${total} missing ${missing}"
[ "${missing}" = 0 ] || { echo "OFFLINE_ROLLBACK STOP: ${missing} objects are missing"; exit 6; }
echo "OFFLINE_ROLLBACK READY"
DEV
```

PASS: `OFFLINE_ROLLBACK READY`.

## 2. Push the branch

- [ ] **2.1** Fix the candidate (on the host).

<!-- p8:candidate -->
```sh
setopt pipefail no_bang_hist
export CAND=$(git -C "${REPO}" rev-parse 'refs/heads/!my-fp-1031') && echo "${CAND}" | tee "${P8}/02_candidate_sha.txt"
git -C "${REPO}" merge-base --is-ancestor 437be91e24 "${CAND}" && echo ON_NEW_BASE
git -C "${REPO}" diff --name-only "${REVIEWED}" "${CAND}" | grep -v '^docs/' ; echo "end of code changes since review"
git -C "${PORT}" status --porcelain | wc -l
```

PASS: `ON_NEW_BASE`; nothing is listed before "end of code changes since
review"; the port worktree is clean (0). A code change after `REVIEWED` needs
its review first: STOP.

- [ ] **2.2** Origin has no `!my-fp-1031`, or only an ancestor of `CAND`.

<!-- p8:ff -->
```sh
setopt pipefail no_bang_hist
R=$(git -C "${REPO}" ls-remote origin 'refs/heads/!my-fp-1031' | cut -f1); echo "remote: ${R:-<none>}"
[ -z "${R}" ] || git -C "${REPO}" merge-base --is-ancestor "${R}" "${CAND}" && echo FAST_FORWARD_OK
```

PASS: `FAST_FORWARD_OK`. Otherwise STOP. Never force-push.

- [ ] **2.3** Push.

<!-- p8:push -->
```sh
setopt pipefail no_bang_hist
git -C "${REPO}" push origin 'refs/heads/!my-fp-1031:refs/heads/!my-fp-1031' 2>&1 | tee "${P8}/02_push.txt"
```

PASS: exit 0, and the output has no "forced update".

- [ ] **2.4** Verify on origin.

<!-- p8:origin-after -->
```sh
setopt pipefail no_bang_hist
git -C "${REPO}" ls-remote origin 'refs/heads/!my-fp-1031' 'refs/heads/!my-fp-new' | tee "${P8}/02_origin_after.txt"
```

PASS: `!my-fp-1031` equals `CAND`, and `!my-fp-new` still equals
`ORIGIN_NEW`.

- [ ] **2.5** The device sees the branch with its deploy key.

<!-- p8:device-ls-remote -->
```sh
setopt pipefail no_bang_hist
timeout -k 5 60 ssh -o BatchMode=yes -o ConnectTimeout=8 -o ServerAliveInterval=5 -o ServerAliveCountMax=3 $DEV 'cd /data/openpilot && git ls-remote origin "refs/heads/!my-fp-1031"' | tee "${P8}/02_device_ls_remote.txt"
```

PASS: the SHA equals `CAND`. On "Permission denied (publickey)", follow the
key diagnosis in AGENTS.md and STOP.

- [ ] **2.6** The device downloads the objects. This changes no branch and
  no file in the working tree.

<!-- p8:fetch -->
```sh
setopt pipefail no_bang_hist
timeout -k 5 1900 ssh -o BatchMode=yes -o ConnectTimeout=8 -o ServerAliveInterval=5 -o ServerAliveCountMax=3 $DEV bash -s <<'DEV' | tee "${P8}/02_device_fetch.txt"
set -euo pipefail
cd /data/openpilot
timeout 1800 git fetch origin '+refs/heads/!my-fp-1031:refs/remotes/origin/!my-fp-1031'
echo "fetched $(git rev-parse 'refs/remotes/origin/!my-fp-1031')"
echo "branch $(git symbolic-ref --short HEAD) head $(git rev-parse HEAD)"
DEV
```

PASS: `fetched` equals `CAND`; the branch is still `!my-fp-new` and `head`
still equals `OLD_SHA`. The fetch gets at most 30 minutes.

- [ ] **2.7** The AGNOS target of both trees equals the installed OS.

<!-- p8:agnos -->
```sh
setopt pipefail no_bang_hist
timeout -k 5 120 ssh -o BatchMode=yes -o ConnectTimeout=8 -o ServerAliveInterval=5 -o ServerAliveCountMax=3 $DEV bash -s -- "${CAND}" "${OLD_SHA}" "${UNIT}" <<'DEV' | tee "${P8}/02_agnos.txt"
set -euo pipefail
cand=${1:?candidate} old=${2:?rollback} unit=${3:?unit}
cd /data/openpilot
installed=$(cat /VERSION)
model=$(tr -d '\0' < /sys/firmware/devicetree/base/model)
launcher=$(pgrep -f '[l]aunch_chffrplus\.sh' | head -n1 || true)
[ -n "$launcher" ] || { echo "AGNOS_CHECK STOP: no launch_chffrplus.sh process, so the boot environment is unknown"; exit 5; }
mapfile -d '' -t boot_env < "/proc/$launcher/environ"
inherited=$(printf '%s\n' "${boot_env[@]}" | sed -n 's/^AGNOS_VERSION=//p')
echo "installed ${installed} model ${model}"
echo "inherited_AGNOS_VERSION ${inherited:-<unset>} (environment of launch_chffrplus.sh pid $launcher)"
echo "unit_environment_AGNOS $(systemctl show -p Environment "$unit" | grep -o 'AGNOS_VERSION=[^ ]*' || echo '<none>')"
if [ -f /data/launch_env.sh ]; then
  # variable names only; values can be secrets (WEATHER_TOKEN)
  echo "data_launch_env_vars $(grep -oE '^[[:space:]]*(export[[:space:]]+)?[A-Za-z_][A-Za-z0-9_]*=' /data/launch_env.sh | sed -E 's/^[[:space:]]*(export[[:space:]]+)?//; s/=$//' | sort -u | tr '\n' ' ')"
else
  echo "data_launch_env_vars <no /data/launch_env.sh>"
fi
tmp=$(mktemp -d)
status=PASS
for pair in "candidate:${cand}" "rollback:${old}"; do
  name=${pair%%:*}; sha=${pair#*:}
  git show "${sha}:launch_env.sh" > "${tmp}/launch_env_${name}.sh"
  eff=$(env -i "${boot_env[@]}" bash -c 'source "$1" >/dev/null 2>&1; printf "%s" "${AGNOS_VERSION:-}"' _ "${tmp}/launch_env_${name}.sh")
  manifest=agnos.json
  if git show "${sha}:launch_chffrplus.sh" | grep -q c3_agnos.json && printf '%s' "$model" | grep -qE 'comma (tici|tizi)'; then manifest=c3_agnos.json; fi
  digest=$(git show "${sha}:system/hardware/tici/${manifest}" | sha256sum | cut -c1-16)
  echo "tree ${name} ${sha} effective_AGNOS_VERSION ${eff:-<unset>} manifest ${manifest} sha256 ${digest}"
  [ "$eff" = "$installed" ] || status=STOP
done
rm -rf "$tmp"
if [ "$status" = PASS ]; then echo "AGNOS_CHECK PASS: both trees keep AGNOS ${installed}; no boot enters the flash branch"; else echo "AGNOS_CHECK STOP: a boot would flash AGNOS"; exit 5; fi
DEV
```

PASS: `AGNOS_CHECK PASS`. `AGNOS_CHECK STOP`: STOP, and show Radek the
inherited value or the override. The script prints variable names from
`/data/launch_env.sh`, never values.

- [ ] **2.8** Free space after the fetch: `MIN_FREE=4000000000`, then run
  the 1.4 block again.

<!-- p8:min-free-after -->
```sh
setopt pipefail no_bang_hist
export MIN_FREE=4000000000
```

PASS: `STORAGE_CHECK PASS`.

From here on, a new commit on `!my-fp-1031` changes `CAND`. Run 2.1 to 2.8
again before section 4.

## 3. Settings backup and restore rehearsal

The backup contains tokens and keys (the DONT_LOG params). Keep it in `$P8`
(mode 700). Never commit it, copy it to Drive or share it.

- [ ] **3.1** Record owners and modes.

<!-- p8:owners -->
```sh
setopt pipefail no_bang_hist
timeout -k 5 60 ssh -o BatchMode=yes -o ConnectTimeout=8 -o ServerAliveInterval=5 -o ServerAliveCountMax=3 $DEV bash -s <<'DEV' | tee "${P8}/03_owners.txt"
set -euo pipefail
for s in /data/params /cache/params; do
  if [ ! -e "$s/d" ]; then echo "$s/d absent"; continue; fi
  t=$(readlink -f "$s/d"); echo "$s/d -> $t"; stat -c '%U:%G %a %n' "$s" "$t"
  find "$t" -maxdepth 1 -type f -exec stat -c '%U:%G %a' {} + | sort | uniq -c
done
stat -c '%U:%G %a %n' /data/ssh /data/ssh/id_ed25519
DEV
```

PASS: the output is recorded. Every restore must give these values.

- [ ] **3.2** Make the archives on the device (kept there for a fast
  restore).

<!-- p8:backup -->
```sh
setopt pipefail no_bang_hist
timeout -k 5 300 ssh -o BatchMode=yes -o ConnectTimeout=8 -o ServerAliveInterval=5 -o ServerAliveCountMax=3 $DEV bash -s <<'DEV' | tee "${P8}/03_device_backup.txt"
set -euo pipefail
B=/data/phase8_backup
if [ -e "$B" ]; then echo "EXISTS: $B (move it aside first)"; exit 3; fi
mkdir -p "$B"
tar -C /data/params/d -cf "$B/params_d.tar" .
if [ -d /cache/params/d ]; then tar -C /cache/params/d -cf "$B/cache_params_d.tar" .; fi
cd /
L=""
for f in data/toggle_backups data/conditional_experimental_override cache/use_HD cache/use_konik cache/on_backup data/fullupdate.log data/fullupdate_reboot.log; do
  if [ -e "$f" ]; then L="$L $f"; fi
done
echo "files:$L"
if [ -n "$L" ]; then tar -cf "$B/frogpilot_files.tar" $L; fi
{ ls -la /data/backups /data/models /data/tailscale /persist/frogsgomoo.py; du -sh /data/backups; } > "$B/listings.txt" 2>&1 || true
cd "$B" && sha256sum *.tar > SHA256SUMS && cat SHA256SUMS
DEV
```

PASS: exit 0. `SHA256SUMS` lists `params_d.tar`, `frogpilot_files.tar`, and
`cache_params_d.tar` when `/cache/params/d` exists. At boot the new base
keeps only five entries in `/data/toggle_backups` and three code archives in
`/data/backups`. The code archives are listed, not copied: git is the code
backup.

- [ ] **3.3** Copy the archives to the host and check them.

<!-- p8:copy -->
```sh
setopt pipefail no_bang_hist
mkdir -p "${P8}/backup" && timeout -k 5 600 ssh -o BatchMode=yes -o ConnectTimeout=8 -o ServerAliveInterval=5 -o ServerAliveCountMax=3 $DEV 'tar -C /data/phase8_backup -cf - .' | tar -C "${P8}/backup" -xf - \
  && (cd "${P8}/backup" && shasum -a 256 -c SHA256SUMS) && chmod -R go-rwx "${P8}"
```

PASS: every file reports `OK`.

- [ ] **3.4** Extract the reference copies and their listings (both
  stores).

<!-- p8:reference -->
```sh
setopt pipefail no_bang_hist
for s in params_d cache_params_d; do
  [ -f "${P8}/backup/${s}.tar" ] || continue
  mkdir -p "${P8}/backup/${s}" && tar -C "${P8}/backup/${s}" -xpf "${P8}/backup/${s}.tar"
  (cd "${P8}/backup/${s}" && find . -type f ! -name .lock -exec shasum -a 256 {} + | LC_ALL=C sort -k2) > "${P8}/backup/${s}.sha256"
  echo "${s} $(wc -l < "${P8}/backup/${s}.sha256") entries"
done
```

PASS: `params_d` has at least 400 entries (route 00002310 logged 452 keys).

- [ ] **3.5** Record the lane-centering values, the keys the migration
  deletes, rewrites or retypes, and the core settings (appendix A1).

<!-- p8:record -->
```sh
setopt pipefail no_bang_hist
python3 "${P8}/A1.py" "${P8}/backup/params_d" > "${P8}/03_record_keys.txt" && grep -A3 '^## lane centering' "${P8}/03_record_keys.txt"
```

PASS: the lane-centering values are printed. The last known values
(2026-10-04) are `LaneCentering` 1, `LaneCenteringE2EAuthority` 0.0 and
`LaneCenterOffset` 0.0. Show Radek any difference; do not change it. If
`LaneCenteringE2EAuthority` is ever lost, the new default 1.0 applies. Box
5a.5 compares it.

- [ ] **3.6** Restore rehearsal on the host (appendix A4).

<!-- p8:a4-run -->
```sh
setopt pipefail no_bang_hist
bash "${P8}/A4.sh"
```

PASS: `REHEARSAL_PASS`.

- [ ] **3.7** Extract each archive with the device's own tar into a
  scratch directory. Keep the listings on the device for R2, and compare
  them with the host listings. The live stores are not touched.

<!-- p8:device-listing -->
```sh
setopt pipefail no_bang_hist
timeout -k 5 300 ssh -o BatchMode=yes -o ConnectTimeout=8 -o ServerAliveInterval=5 -o ServerAliveCountMax=3 $DEV bash -s <<'DEV' | tee "${P8}/03_device_listing.txt"
set -euo pipefail
B=/data/phase8_backup R=/data/phase8_rehearsal
for s in params_d cache_params_d; do
  [ -f "$B/$s.tar" ] || continue
  rm -rf "$R"; mkdir -p "$R"
  tar -C "$R" -xpf "$B/$s.tar"
  (cd "$R" && find . -type f ! -name .lock -exec sha256sum {} + | LC_ALL=C sort -k2) > "$B/$s.sha256"
  echo "$s $(wc -l < "$B/$s.sha256") entries"
done
rm -rf "$R"
DEV
ok=1
for s in params_d cache_params_d; do
  [ -f "${P8}/backup/${s}.tar" ] || continue
  timeout -k 5 300 ssh -o BatchMode=yes -o ConnectTimeout=8 -o ServerAliveInterval=5 -o ServerAliveCountMax=3 $DEV "cat /data/phase8_backup/${s}.sha256" > "${P8}/03_device_${s}.sha256" || ok=0
  diff -q "${P8}/backup/${s}.sha256" "${P8}/03_device_${s}.sha256" || ok=0
done
(( ok )) && echo DEVICE_TAR_PASS || echo DEVICE_TAR_FAIL
```

PASS: `DEVICE_TAR_PASS`.

- [ ] **3.8** Free space after the backups: run the 1.4 block again
  (`MIN_FREE` is 4000000000 since 2.8). PASS: `STORAGE_CHECK PASS`.

## 4. Switch

- [ ] **4.1** **[RADEK: go]** Radek approves the switch. The car is still
  parked with the ignition off.
- [ ] **4.2** Set the switch target.

<!-- p8:target-switch -->
```sh
setopt pipefail no_bang_hist
export TARGET_SHA=${CAND} TARGET_BRANCH='!my-fp-1031' TARGET_MODE=switch
echo "${TARGET_MODE} ${TARGET_BRANCH} ${TARGET_SHA}"
```

- [ ] **4.3** Repeat 1.3 (state) and 1.4 (storage) as one last check. PASS
  as there, with `branch !my-fp-new`.
- [ ] **4.4** The switch. One device script runs these steps in order:
  1. Check that the target commit and all its objects are local.
  2. Check that the unit runs in the recorded cgroup and that the cgroup
     holds the manager.
  3. Switch mode: off-road, `UpdaterState` idle, no staged AGNOS update.
     Rollback mode: these progress params can be stale after a failed boot,
     so they are printed, not required. Both modes: no AGNOS flashing
     process may run.
  4. Quiesce, then take both update locks. Quiesced means: the unit stopped
     within 120 s, it is inactive, its whole cgroup tree is empty, and no
     openpilot, update or flashing process runs anywhere (the check waits up
     to 30 s).
  5. Unmount the staging overlay, and delete `.overlay_consistent` and
     `.overlay_init`.
  6. Set the branch and its upstream, point `HEAD` at it,
     `git reset --hard` to the SHA, and delete `prebuilt`.
  7. Verify all of this, the empty cgroup included, print `boot_id`, and
     reboot.

  The script stops at the first failed step. Before step 6, nothing in the
  tree changed. After a stop there, use "resume" in section 6. The host
  side ends after at most 900 s. If it ends by its limit, run 4.5: a new
  `boot_id` shows that the script reached the reboot.

<!-- p8:switch -->
```sh
setopt pipefail no_bang_hist
timeout -k 5 900 ssh -o BatchMode=yes -o ConnectTimeout=8 -o ServerAliveInterval=5 -o ServerAliveCountMax=3 $DEV bash -s -- "${UNIT}" "${UNIT_CGROUP}" "${TARGET_SHA}" "${TARGET_BRANCH}" "${TARGET_MODE}" <<'DEV' | tee "${P8}/switch_to_${TARGET_SHA}.txt"
set -euo pipefail
unit=${1:?unit} cg=${2:?cgroup} sha=${3:?sha} branch=${4:?branch} mode=${5:?mode}
fail() { echo "SWITCH FAIL: $1"; exit 6; }
case "$mode" in switch|rollback) ;; *) fail "the mode is switch or rollback, not ${mode}";; esac
self_cmd=$({ tr '\0' ' ' < "/proc/$$/cmdline"; } 2>/dev/null) || self_cmd=""   # this script's own command line, as pgrep -a prints it
self_cmd=${self_cmd% }
writers() {  # openpilot, update and flashing processes anywhere, also outside the unit's cgroup. A line equal to this script's own
             # command line is this script or one of its subshells, and is left out: the arguments hold the cgroup path, which the
             # pattern matches (run of 2026-11-14). An unreadable own command line counts as a running process
  local line
  if [ -z "$self_cmd" ]; then echo "this script cannot read its own command line (/proc/$$/cmdline)"; return 0; fi
  { pgrep -af '[m]anager\.py|[l]aunch_chffrplus|[s]elfdrive\.|[s]ystem\.|[f]rogpilot\.|[b]uild\.py|[s]cons|[f]ullupdate\.sh|[a]gnos\.py|[h]ardware/tici/updater' || true
    pgrep -ax 'ui|_ui|mapd|pandad|loggerd|encoderd|camerad' || true; } | while IFS= read -r line; do
    [ "${line#* }" = "$self_cmd" ] || printf '%s\n' "$line"
  done
}
cgroup_procs() {  # every PID in the unit's cgroup tree, child cgroups included, on each mounted hierarchy. A listing or read error
                  # returns 1 (set -e is off inside $(...)), so an unreadable cgroup.procs never counts as empty
  local r f files
  for r in /sys/fs/cgroup /sys/fs/cgroup/unified /sys/fs/cgroup/systemd; do
    if [ -d "$r$cg" ]; then
      files=$(find "$r$cg" -name cgroup.procs) || return 1
      while IFS= read -r f; do
        [ -n "$f" ] || continue
        [ -r "$f" ] || return 1
        cat "$f" || return 1
      done <<< "$files"
    fi
  done
  return 0
}
quiesce() {
  local state live i
  if [ "$(systemctl is-active "$unit" || true)" = active ]; then
    timeout 120 sudo systemctl stop "$unit" || fail "systemctl stop ${unit} failed or took over 120 s; nothing was changed (resume)"
  fi
  for i in $(seq 1 30); do
    state=$(systemctl is-active "$unit" || true)
    live=$(cgroup_procs) || fail "cannot read cgroup ${cg}; nothing was changed (resume)"
    if { [ "$state" = inactive ] || [ "$state" = failed ]; } && [ -z "${live//[[:space:]]/}" ] && [ -z "$(writers)" ]; then
      echo "QUIESCED: ${unit} ${state}, cgroup ${cg} empty, no openpilot, update or flashing process"; return 0
    fi
    sleep 1
  done
  echo "unit ${state}; PIDs in cgroup ${cg}: $(echo ${live:-none})"; writers
  fail "openpilot did not stop; nothing was changed (resume)"
}
cd /data/openpilot
# 1. the target is complete offline
git cat-file -e "${sha}^{commit}" || fail "${sha} is not a local commit; nothing was changed"
missing=$(git ls-tree -r "${sha}" | awk '{print $3}' | git cat-file --batch-check | grep -c ' missing$' || true)
[ "$missing" = 0 ] || fail "$missing objects of ${sha} are missing; nothing was changed"
# 2. the recorded cgroup is the unit's, and it holds the manager (proves the check below sees openpilot)
if [ "$(systemctl is-active "$unit" || true)" = active ]; then
  cur=$(systemctl show -p ControlGroup --value "$unit")
  [ "$cur" = "$cg" ] || fail "${unit} runs in cgroup ${cur:-<none>}, not the recorded ${cg}; nothing was changed"
  mgr=$(pgrep -f '[m]anager\.py' | head -n1 || true)
  live=$(cgroup_procs) || fail "cannot read cgroup ${cg}; nothing was changed"
  if [ -n "$mgr" ]; then printf '%s\n' "$live" | grep -qx "$mgr" || fail "the manager (pid ${mgr}) is not in cgroup ${cg}; nothing was changed"; fi
fi
# 3. switch: off-road and no pending update work; rollback: the progress params may be stale, only real processes count
if [ "$mode" = switch ]; then
  [ "$(cat /data/params/d/IsOnroad 2>/dev/null || echo 0)" = 0 ] || fail "IsOnroad is set; nothing was changed"
  state=$(cat /data/params/d/UpdaterState 2>/dev/null || echo idle)
  [ "$state" = idle ] || fail "updated is busy (${state}); nothing was changed"
  [ ! -e /data/params/d/Offroad_NeosUpdate ] || fail "an AGNOS update is staged (Offroad_NeosUpdate); nothing was changed"
else
  echo "rollback mode: IsOnroad=$(cat /data/params/d/IsOnroad 2>/dev/null || echo '<absent>') UpdaterState=$(cat /data/params/d/UpdaterState 2>/dev/null || echo '<absent>') Offroad_NeosUpdate=$([ -e /data/params/d/Offroad_NeosUpdate ] && echo set || echo absent); not required (Radek confirmed parked, ignition off)"
fi
! pgrep -af '[a]gnos\.py|[h]ardware/tici/updater' || fail "AGNOS flashing runs; let it finish, then run this block again; nothing was changed"
# 4. quiesce: the unit inactive, its whole cgroup tree empty, no openpilot, update or flashing process anywhere; both update locks
quiesce
exec 8>/data/fullupdate.lock
flock -n 8 || fail "a fullupdate.sh run holds /data/fullupdate.lock; nothing was changed (resume)"
exec 9>/data/fullupdate_reboot.lock
flock -n 9 || fail "a reboot supervisor holds /data/fullupdate_reboot.lock; nothing was changed (resume)"
echo "LOCKS_HELD: /data/fullupdate.lock /data/fullupdate_reboot.lock"
# 5. no staged overlay may install at boot (launch_chffrplus.sh:52-75 needs .overlay_init and .overlay_consistent)
if mountpoint -q /data/safe_staging/merged; then sudo umount -l /data/safe_staging/merged; fi
sudo rm -f /data/safe_staging/finalized/.overlay_consistent /data/openpilot/.overlay_init
# 6. branch, upstream and tree
if git show-ref --verify --quiet "refs/heads/${branch}"; then
  [ "$(git rev-parse "refs/heads/${branch}")" = "${sha}" ] || fail "local ${branch} is at $(git rev-parse "refs/heads/${branch}"), not ${sha}; tree unchanged (resume)"
else
  git branch --no-track "${branch}" "${sha}"
fi
git config "branch.${branch}.remote" origin
git config "branch.${branch}.merge" "refs/heads/${branch}"
git symbolic-ref HEAD "refs/heads/${branch}"
git reset -q --hard "${sha}"
rm -f /data/openpilot/prebuilt
sync
# 7. verify before the reboot
[ "$(git rev-parse HEAD)" = "${sha}" ] || fail "HEAD is not ${sha} after the reset (R1)"
[ "$(git symbolic-ref --short HEAD)" = "${branch}" ] || fail "the branch is not ${branch} (R1)"
[ "$(git status --porcelain --untracked-files=no | wc -l)" = 0 ] || fail "tracked files differ from ${sha} (R1)"
[ ! -e /data/safe_staging/finalized/.overlay_consistent ] && [ ! -e /data/openpilot/.overlay_init ] || fail "an overlay is still eligible (R1)"
[ ! -e /data/openpilot/prebuilt ] || fail "prebuilt still exists (R1)"
live=$(cgroup_procs) || fail "cannot read cgroup ${cg} (R1)"
[ -z "${live//[[:space:]]/}" ] && [ -z "$(writers)" ] || fail "an openpilot process started again (R1)"
echo "OVERLAY_INELIGIBLE"
echo "VERIFIED_BEFORE_REBOOT head ${sha} branch ${branch} mode ${mode}"
echo "boot_id_before $(cat /proc/sys/kernel/random/boot_id)"
sudo reboot
DEV
sed -n 's/^boot_id_before //p' "${P8}/switch_to_${TARGET_SHA}.txt" > "${P8}/boot_id_before.txt"
grep -q '^VERIFIED_BEFORE_REBOOT' "${P8}/switch_to_${TARGET_SHA}.txt" && echo SWITCH_ISSUED || echo "SWITCH NOT ISSUED: read the SWITCH FAIL line"
```

PASS: `QUIESCED`, `OVERLAY_INELIGIBLE`, `VERIFIED_BEFORE_REBOOT` and
`SWITCH_ISSUED`. The SSH session then closes. A `SWITCH FAIL` line names the
next step: "(resume)" means nothing in the tree changed (section 6,
resume); "(R1)" means the reset ran (section 6, R1).

- [ ] **4.5** Wait for the reboot (at most 15 minutes), proven by a new
  `boot_id`.

<!-- p8:wait -->
```sh
setopt pipefail no_bang_hist
python3 - "${DEV}" "${P8}/boot_id_before.txt" <<'PY'
import subprocess, sys, time
# whole wait 900 s, one ssh call at most 30 s (and never past the deadline), 20 s between calls: the loop ends 900 s after it starts
LIMIT, CALL, PAUSE = 900, 30, 20
dev, before = sys.argv[1], open(sys.argv[2]).read().strip()
if not before:
  sys.exit('no boot_id_before recorded')
SSH = ['ssh', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=8', '-o', 'ServerAliveInterval=5', '-o', 'ServerAliveCountMax=3', dev]
start, state, now = time.monotonic(), 'unreachable', ''
while (left := LIMIT - (time.monotonic() - start)) > 0:
  try:
    r = subprocess.run(SSH + ['cat /proc/sys/kernel/random/boot_id'], capture_output=True, text=True, timeout=min(CALL, left))
    if r.returncode == 0:
      now, state = r.stdout.strip(), 'same'
      if now and now != before:
        state = 'rebooted'
        break
  except subprocess.TimeoutExpired:
    pass
  time.sleep(max(0.0, min(PAUSE, LIMIT - (time.monotonic() - start))))
took = time.monotonic() - start
if state == 'rebooted':
  print(f'REBOOTED boot_id {now} (before {before}) after {took:.0f} s')
elif state == 'same':
  print(f'NOT_REBOOTED: the device still answers with boot_id {before} after {took:.0f} s (R1 if the reset ran)')
else:
  print(f'UNREACHABLE for {took:.0f} s: section 6, physical recovery')
sys.exit(0 if state == 'rebooted' else 1)
PY
```

PASS: `REBOOTED`. The loop ends at most 900 s after it starts: each ssh
call gets at most 30 s and never runs past the deadline, and the pause
between calls is cut at the deadline too. `NOT_REBOOTED`: the reset ran but
the reboot did not. Run `timeout -k 5 60 ssh -o BatchMode=yes -o ConnectTimeout=8 -o ServerAliveInterval=5 -o ServerAliveCountMax=3 $DEV sudo reboot`
once with the ignition off, then this box again. `UNREACHABLE`: physical
recovery (section 6).

- [ ] **4.6** Wait for the build (at most 60 minutes after the reboot).

<!-- p8:build-wait -->
```sh
setopt pipefail no_bang_hist
cat > "${P8}/build_check.sh" <<'DEV'
set -uo pipefail
echo "agnos $(cat /VERSION)"
echo "build_procs $(pgrep -f '[s]cons|[b]uild\.py' | wc -l)"
if pgrep -f '[m]anager\.py' >/dev/null; then echo MANAGER_UP; else echo MANAGER_DOWN; fi
DEV
python3 - "${DEV}" "${P8}" <<'PY'
import os, subprocess, sys, time
# whole wait 3600 s, one check at most 60 s (and never past the deadline), 300 s between checks: the loop ends 3600 s after it starts
LIMIT, CALL, PAUSE = 3600, 60, 300
dev, p8 = sys.argv[1], sys.argv[2]
check = open(os.path.join(p8, 'build_check.sh')).read()
SSH = ['ssh', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=8', '-o', 'ServerAliveInterval=5', '-o', 'ServerAliveCountMax=3', dev, 'bash -s']
start, n = time.monotonic(), 0
while (left := LIMIT - (time.monotonic() - start)) > 0:
  n += 1
  try:
    out = subprocess.run(SSH, input=check, capture_output=True, text=True, timeout=min(CALL, left)).stdout
  except subprocess.TimeoutExpired:
    out = 'CHECK_TIMED_OUT\n'
  open(os.path.join(p8, f'build_{n}.txt'), 'w').write(out)
  print(f'check {n} at {time.monotonic() - start:.0f} s: ' + ' '.join(out.split()))
  if 'MANAGER_UP' in out.split() and 'build_procs 0' in out:
    print(f'BUILD_DONE at check {n}, {time.monotonic() - start:.0f} s after the start')
    sys.exit(0)
  time.sleep(max(0.0, min(PAUSE, LIMIT - (time.monotonic() - start))))
print(f'BUILD_DEADLINE: no manager {time.monotonic() - start:.0f} s after the start (R1)')
sys.exit(1)
PY
```

PASS: `BUILD_DONE` with `agnos 12.8`. Record the build time. The loop ends
at most 3600 s after it starts: each check gets at most 60 s, cut at the
deadline, and so is the 300 s pause between checks. Any other `agnos`
value: an AGNOS flash runs. Never interrupt it; let it finish, then R1.
`BUILD_DEADLINE`, or a build error on the screen: R1.

- [ ] **4.7** Full-SHA check.

<!-- p8:sha-check -->
```sh
setopt pipefail no_bang_hist
timeout -k 5 60 ssh -o BatchMode=yes -o ConnectTimeout=8 -o ServerAliveInterval=5 -o ServerAliveCountMax=3 $DEV bash -s -- "${TARGET_SHA}" "${TARGET_BRANCH}" <<'DEV' | tee "${P8}/sha_check_${TARGET_SHA}.txt"
set -euo pipefail
cd /data/openpilot
head=$(git rev-parse HEAD) branch=$(git symbolic-ref --short HEAD) changes=$(git status --porcelain --untracked-files=no | wc -l)
echo "head ${head} branch ${branch} tracked_changes ${changes} upstream $(git rev-parse --abbrev-ref '@{u}' 2>&1)"
[ "${head}" = "$1" ] && [ "${branch}" = "$2" ] && [ "${changes}" = 0 ] && echo "SHA_CHECK PASS" || { echo "SHA_CHECK FAIL"; exit 9; }
DEV
```

PASS: `SHA_CHECK PASS`. `SHA_CHECK FAIL`: R1.

## 5. Verify before any drive (matrix "device" row)

### 5a. Ignition off

Start 5a when `MANAGER_UP` has been true for at least 3 minutes.

- [ ] **5a.1** Core processes run; the processes that must stay off are off.

<!-- p8:processes -->
```sh
setopt pipefail no_bang_hist
timeout -k 5 60 ssh -o BatchMode=yes -o ConnectTimeout=8 -o ServerAliveInterval=5 -o ServerAliveCountMax=3 $DEV bash -s <<'DEV' | tee "${P8}/05a_processes.txt"
set -uo pipefail
for p in '[m]anager\.py' '[s]elfdrive\.pandad\.pandad' '[f]rogpilot\.frogpilot_process' '[s]ystem\.hardware\.hardwared' '[s]ystem\.updated\.updated'; do
  echo "run $p $(pgrep -f "$p" | wc -l)"
done
echo "run ui $(pgrep -x ui | wc -l)"; echo "run mapd $(pgrep -x mapd | wc -l)"
for p in '[f]rogpilot\.system\.the_pond' '[f]rogpilot\.system\.frogpilot_telemetry' '[f]rogpilot\.system\.speed_limit_reader' '[s]ystem\.loggerd\.uploader'; do
  echo "off $p $(pgrep -f "$p" | wc -l)"
done
echo "off port_8082 $(ss -ltn 2>/dev/null | grep -c ':8082 ')"
DEV
```

PASS: each `run` line shows 1 or more; each `off` line shows 0 (`uploader`
is off because `NoUploads` is 1).

- [ ] **5a.2** No process restarts within 2 minutes.

<!-- p8:restarts -->
```sh
setopt pipefail no_bang_hist
snap() { timeout -k 5 60 ssh -o BatchMode=yes -o ConnectTimeout=8 -o ServerAliveInterval=5 -o ServerAliveCountMax=3 $DEV bash -s <<'DEV'
ps -eo pid,args | grep -E 'manager\.py|selfdrive\.|system\.|frogpilot\.|\./(ui|mapd)( |$)' | grep -v -e 'grep -E' | sort -k2
DEV
}
snap > "${P8}/05a_ps1.txt" && sleep 120 && snap > "${P8}/05a_ps2.txt" && diff "${P8}/05a_ps1.txt" "${P8}/05a_ps2.txt" && echo NO_RESTARTS
```

PASS: `NO_RESTARTS`. A process that returns with a new PID is in a crash
loop: read the log (5a.3). If the loop does not stop, R1.

- [ ] **5a.3** The openpilot log: a proven source, no error log, no
  tracebacks. The script finds the tmux pane whose process runs the launcher
  or is one of its ancestors. A failed or empty capture is
  `LOG_EVIDENCE_MISSING`: missing evidence is never a PASS.

<!-- p8:logs -->
```sh
setopt pipefail no_bang_hist
timeout -k 5 60 ssh -o BatchMode=yes -o ConnectTimeout=8 -o ServerAliveInterval=5 -o ServerAliveCountMax=3 $DEV bash -s > "${P8}/05a_logs.txt" <<'DEV'
set -euo pipefail
miss() { echo "LOG_EVIDENCE_MISSING: $1"; exit 7; }
launcher=$(pgrep -f '[l]aunch_chffrplus\.sh' | head -n1 || true)
[ -n "$launcher" ] || miss "no launch_chffrplus.sh process"
chain=" $launcher "
p=$launcher
for i in $(seq 1 30); do
  p=$(awk '/^PPid:/ {print $2}' "/proc/$p/status" 2>/dev/null || true)
  if [ -z "$p" ] || [ "$p" -le 1 ]; then break; fi
  chain="$chain$p "
done
target=""
while read -r pane pid; do
  case "$chain" in *" $pid "*) target=$pane;; esac
done < <(tmux list-panes -a -F '#{session_name}:#{window_index}.#{pane_index} #{pane_pid}' 2>/dev/null || true)
[ -n "$target" ] || miss "no tmux pane runs the launcher (pid $launcher, ancestors${chain})"
out=$(tmux capture-pane -p -t "$target" -S -3000) || miss "tmux capture-pane -t $target failed"
[ -n "$(printf '%s' "$out" | tr -d '[:space:]')" ] || miss "the capture of $target is empty"
echo "log_source tmux pane $target runs launcher pid $launcher"
echo "error_txt $([ -e /data/error_logs/error.txt ] && echo PRESENT || echo absent)"
echo "tracebacks $(printf '%s\n' "$out" | grep -c Traceback || true)"
echo "--- capture"
printf '%s\n' "$out"
DEV
r=$?; head -n 4 "${P8}/05a_logs.txt"; (( r == 0 ))
```

PASS: exit 0 with a `log_source` line, `error_txt absent` (an error log turns
off always-on lateral and shows the error screen) and `tracebacks 0`. Read
each traceback and judge it; one from our code is a FAIL.
`LOG_EVIDENCE_MISSING` is a FAIL.

- [ ] **5a.4** Telemetry consent. First confirm it is unanswered; then
  **[RADEK: physical]** Radek answers on the device screen.

<!-- p8:telemetry -->
```sh
setopt pipefail no_bang_hist
timeout -k 5 60 ssh -o BatchMode=yes -o ConnectTimeout=8 -o ServerAliveInterval=5 -o ServerAliveCountMax=3 $DEV bash -s <<'DEV'
for k in FrogPilotTelemetry FrogPilotTelemetryConfirmed; do echo "${k}=$(cat /data/params/d/$k 2>/dev/null || echo '<absent>')"; done
DEV
```

PASS before: `FrogPilotTelemetryConfirmed` is 0 or absent, and the screen
shows "Help make FrogPilot better ..." with "No Thanks" and "Yes, Share
Data". **[RADEK: physical]** Tap "No Thanks" (decision `DECISIONS.md` F).
Run the block again. PASS after: `FrogPilotTelemetry=0`,
`FrogPilotTelemetryConfirmed=1`, the home screen shows, and the telemetry
`off` line of 5a.1 is still 0.

- [ ] **5a.5** Settings are intact apart from the expected changes
  (appendices A2 and A1).

<!-- p8:settings -->
```sh
setopt pipefail no_bang_hist
mkdir -p "${P8}/post/params_d" && timeout -k 5 300 ssh -o BatchMode=yes -o ConnectTimeout=8 -o ServerAliveInterval=5 -o ServerAliveCountMax=3 $DEV 'tar -C /data/params/d -cf - .' | tar -C "${P8}/post/params_d" -xf - \
  && python3 "${P8}/A2.py" "${P8}/backup/params_d" "${P8}/post/params_d" ~/.route_sync/work/fp1031/port-scratch/migrate_report_b1.json | tee "${P8}/05a_params_compare.txt" \
  && python3 "${P8}/A1.py" "${P8}/post/params_d" > "${P8}/05a_record_keys.txt" && diff "${P8}/03_record_keys.txt" "${P8}/05a_record_keys.txt"
```

PASS: A2 exits 0 (no key missing outside the expected deletions). Judge
every key under "CHANGED, JUDGE EACH" in `RECORD.md`: learned state,
counters, timestamps and caches may change; a user setting must not. The A1
diff shows only the expected deletions (now `<absent>`) and the two
rewrites. The lane-centering, button, upload, Experimental Mode and
personality values are unchanged. A lost or changed setting: R2-key for
that key, unless the new registry lacks it (`clear_all` would delete it again
at the next start). In that case STOP and report: the code needs a registry
row.

- [ ] **5a.6** Offroad alerts. `Offroad_StorageMissing` is a STOP, also
  when the backup has it.

<!-- p8:offroad-alerts -->
```sh
setopt pipefail no_bang_hist
ls "${P8}/backup/params_d" | grep '^Offroad_' || echo "backup: no offroad alerts"
timeout -k 5 60 ssh -o BatchMode=yes -o ConnectTimeout=8 -o ServerAliveInterval=5 -o ServerAliveCountMax=3 $DEV bash -s <<'DEV' | tee "${P8}/05a_offroad_alerts.txt"
set -euo pipefail
cd /data/params/d
n=0
for f in Offroad_*; do [ -e "$f" ] || continue; echo "$f: $(cat "$f")"; n=$((n + 1)); done
echo "alerts $n"
if [ -e Offroad_StorageMissing ]; then echo "STOP: Offroad_StorageMissing is set; the NVMe guard failed and logs go to /data/realdata_fallback"; exit 4; fi
DEV
```

PASS: exit 0, and no alert other than those in the backup. Judge each new
one.

- [ ] **5a.7** Upload and logging flags unchanged.

<!-- p8:flags -->
```sh
setopt pipefail no_bang_hist
tar -tvf "${P8}/backup/frogpilot_files.tar" | grep -E 'cache/use_(HD|konik)|cache/on_backup' || echo "backup: no flag files"
timeout -k 5 60 ssh -o BatchMode=yes -o ConnectTimeout=8 -o ServerAliveInterval=5 -o ServerAliveCountMax=3 $DEV bash -s <<'DEV' | tee "${P8}/05a_cache_flags.txt"
for f in /cache/use_HD /cache/use_konik /cache/on_backup; do echo "$f $([ -e "$f" ] && echo present || echo absent)"; done
DEV
```

PASS: the same flag files exist as in the backup. `NoUploads`,
`DisableOnroadUploads`, `NoLogging`, `UseKonikServer`, `HigherBitrate` and
`GsmMetered` are unchanged in the 5a.5 A1 diff.

- [ ] **5a.8** Updater, safe-reboot path, deploy key and storage after
  boot.

<!-- p8:updater -->
```sh
setopt pipefail no_bang_hist
timeout -k 5 60 ssh -o BatchMode=yes -o ConnectTimeout=8 -o ServerAliveInterval=5 -o ServerAliveCountMax=3 $DEV bash -s <<'DEV' | tee "${P8}/05a_updater.txt"
set -uo pipefail
cd /data/openpilot
echo "head $(git rev-parse HEAD) remote $(git ls-remote origin 'refs/heads/!my-fp-1031' | cut -f1)"
for k in AutomaticUpdates UpdaterTargetBranch UpdateFailedCount; do echo "${k}=$(cat /data/params/d/$k 2>/dev/null || echo '<absent>')"; done
echo "fullupdate_safe_reboot $(grep -c __safe_reboot fullupdate.sh)"
echo "manager_routing $(grep -c 'routed to fullupdate.sh __safe_reboot' system/manager/manager.py)"
stat -c '%U:%G %a %n' /data/ssh /data/ssh/id_ed25519
mountpoint /data/media
echo "fallback_entries $(ls /data/realdata_fallback 2>/dev/null | wc -l)"
DEV
```

PASS: `head` equals `remote`, which equals `CAND`, so the auto-updater has
nothing to do. `AutomaticUpdates` is unchanged from 1.3.
`UpdaterTargetBranch` is `!my-fp-1031` (or absent while `updated` has not
run), and `UpdateFailedCount` is 0 or absent. `fullupdate_safe_reboot` is 2
or more and `manager_routing` is 1. The `/data/ssh` modes equal 3.1 (700,
600). `/data/media` is a mountpoint, and `fallback_entries` equals 1.3.

- [ ] **5a.9** The checked-out model is the pinned tsfdo model.

<!-- p8:model -->
```sh
setopt pipefail no_bang_hist
want=""
if git -C "${REPO}" show "${CAND}:selfdrive/modeld/models/driving_supercombo.onnx" > "${P8}/05a_onnx_expected"; then
  want=$(shasum -a 256 "${P8}/05a_onnx_expected" | cut -d' ' -f1)
else
  echo "MODEL_CHECK FAIL: no driving_supercombo.onnx at ${CAND}"
fi
timeout -k 5 120 ssh -o BatchMode=yes -o ConnectTimeout=8 -o ServerAliveInterval=5 -o ServerAliveCountMax=3 $DEV bash -s <<'DEV' | tee "${P8}/05a_model.txt"
set -euo pipefail
cd /data/openpilot/selfdrive/modeld/models
echo "device_onnx $(sha256sum driving_supercombo.onnx | cut -d' ' -f1)"
ls -la | grep -E 'tinygrad|metadata' || echo "no tinygrad bundle"
DEV
have=$(sed -n 's/^device_onnx //p' "${P8}/05a_model.txt")
echo "expected ${want:-<none>} device ${have:-<none>}"
[ -n "${want}" ] && [ "${want}" = "${have}" ] && echo MODEL_MATCH || { echo "MODEL_CHECK FAIL"; false; }
```

PASS: `MODEL_MATCH`, and the tinygrad driving and DM bundles exist with a
time after the reboot.

### 5b. Ignition on, in Park (no drive)

- [ ] **5b.1** **[RADEK: physical]** At least 3 minutes after 5a.1
  passed, turn the ignition on (READY). Stay in P, do not engage, do not
  drive. Stay like this for at least 4 minutes.
- [ ] **5b.2** Three minutes after the ignition is on: onroad processes run,
  and the logs go to the NVMe.

<!-- p8:onroad -->
```sh
setopt pipefail no_bang_hist
timeout -k 5 60 ssh -o BatchMode=yes -o ConnectTimeout=8 -o ServerAliveInterval=5 -o ServerAliveCountMax=3 $DEV bash -s <<'DEV' | tee "${P8}/05b_processes.txt"
set -uo pipefail
echo "IsOnroad=$(cat /data/params/d/IsOnroad 2>/dev/null || echo '<absent>') FirmwareQueryDone=$(cat /data/params/d/FirmwareQueryDone 2>/dev/null || echo '<absent>')"
ps -eo pid,etimes,args | grep -E 'selfdrive\.(modeld\.(modeld|dmonitoringmodeld)|car\.card|controls\.(controlsd|radard|plannerd)|selfdrived\.selfdrived|locationd\.(locationd|calibrationd))|\./(loggerd|camerad|encoderd)( |$)' | grep -v -e 'grep -E'
lg=$(pgrep -x loggerd | head -n1)
if [ -n "$lg" ]; then echo "loggerd_LOG_ROOT $(tr '\0' '\n' < "/proc/$lg/environ" | sed -n 's/^LOG_ROOT=//p' | head -n1)"; else echo "loggerd_LOG_ROOT <loggerd not running>"; fi
echo "newest_route $(ls -td /data/media/0/realdata/*--0 2>/dev/null | head -n1)"
echo "fallback_entries $(ls /data/realdata_fallback 2>/dev/null | wc -l)"
DEV
```

PASS: `IsOnroad=1` and `FirmwareQueryDone=1`. Each listed process appears,
and its age (`etimes`) grows between two runs. `loggerd_LOG_ROOT` is empty:
the default root on the NVMe. `/data/realdata_fallback` would mean the guard
failed. `newest_route` is newer than the last route before the switch, and
`fallback_entries` equals 1.3.

- [ ] **5b.3** **[RADEK: physical]** Look at the device and the car. The
  device shows the camera view with no red or orange alert. The cluster shows
  no ACC or cruise fault and no new warning light. Record what Radek sees. A
  cruise fault latched during start-up clears only with an ignition cycle
  (2026-09-05). If one appears: ignition off, wait 1 minute, then repeat 5b.1
  to 5b.3 once. A second fault: R1.
- [ ] **5b.4** Copy the first two segments of the park route after segment
  2 has started (about 2.5 minutes after the ignition is on).

<!-- p8:segments -->
```sh
setopt pipefail no_bang_hist
R0=$(timeout -k 5 300 ssh -o BatchMode=yes -o ConnectTimeout=8 -o ServerAliveInterval=5 -o ServerAliveCountMax=3 $DEV 'ls -td /data/media/0/realdata/*--0 | head -n1') && echo "${R0}" | tee "${P8}/05b_route.txt"
RB=${R0%--0}
timeout -k 5 300 ssh -o BatchMode=yes -o ConnectTimeout=8 -o ServerAliveInterval=5 -o ServerAliveCountMax=3 $DEV "ls -d '${RB}--2'" && for s in 0 1; do timeout -k 5 300 ssh -o BatchMode=yes -o ConnectTimeout=8 -o ServerAliveInterval=5 -o ServerAliveCountMax=3 $DEV "cat '${RB}--${s}/rlog.zst'" > "${P8}/05b_rlog_${s}.zst"; done; ls -la "${P8}"/05b_rlog_*.zst
```

PASS: `${RB}--2` exists, so segments 0 and 1 are closed, and both files are
larger than 1 MB.

- [ ] **5b.5** Decode both segments on the host with the port's cereal
  (appendix A3).

<!-- p8:decode -->
```sh
setopt pipefail no_bang_hist
for s in 0 1; do PYTHONPATH="${PORT}:${PORT}/frogpilot/third_party" python3 "${P8}/A3.py" "${P8}/05b_rlog_${s}.zst" | tee "${P8}/05b_log_${s}.txt"; done
```

PASS, both segments:
- Identity: `commit` is `CAND`, `branch` is `!my-fp-1031`, `dirty False`.
- Car: fingerprint `HYUNDAI_SANTA_FE_HEV_2022`, brand `hyundai`, `opLong
  True` (not `mock`).
- Model: `modelV2` at least 1100 per segment (20 Hz model).
- Panda: safety includes `hyundai`, with no faults.
- Processes: `PROCESS_CHECK PASS`. A3 records every process state change
  over the whole segment, with its time. A crash followed by a restart is a
  FAIL, even when the last sample is healthy. So is a PID change of a
  process that was required and running in both samples (a restart between
  samples). The evidence must cover the segment: managerState at 2 Hz, at
  least 90 % of the samples the span should hold, and no gap over 2 s.
  Missing or thin evidence is a FAIL.
- Events, segment 1 (steady state): only `wrongGear`, `wrongCarMode`,
  `preEnableStandstill`, `seatbeltNotLatched`, `doorOpen`, `parkBrake`,
  `pedalPressed`, `gasPressedOverride`, `steerOverride` and `reverseGear`.
- Events, segment 0: in addition, the start-up events
  `selfdriveInitializing`, `commIssue` and `commIssueAvgFreq`, and safety
  `elm327` then `hyundai`, as on today's branch (route 00002310 segment 0).
- Anything else (for example `canError`, `radarFault`, `accFaulted`,
  `processNotRunning`, `carUnrecognized`, `controlsMismatch`,
  `startupNoCar`, `startupNoControl`) is a FAIL. Judge it, and go to R1 if
  it stays after one ignition cycle.

- [ ] **5b.6** **[RADEK: physical]** Turn the ignition off. One minute
  later:

<!-- p8:offroad-again -->
```sh
setopt pipefail no_bang_hist
timeout -k 5 60 ssh -o BatchMode=yes -o ConnectTimeout=8 -o ServerAliveInterval=5 -o ServerAliveCountMax=3 $DEV bash -s <<'DEV'
echo "IsOnroad=$(cat /data/params/d/IsOnroad 2>/dev/null) boot_id $(cat /proc/sys/kernel/random/boot_id)"
if pgrep -f '[m]anager\.py' >/dev/null; then echo MANAGER_UP; else echo MANAGER_DOWN; fi
DEV
```

PASS: `IsOnroad=0`, the `boot_id` equals the one from 4.5 (no reboot), and
`MANAGER_UP`.

- [ ] **5b.7** Report to Radek: all boxes of 4, 5a and 5b with PASS, or the
  open FAILs. **[RADEK: go]** Radek decides on the first drive.

## 6. Rollback

### Decision rule

- Before the first drive: roll back the code (R1) on any FAIL that the box
  itself does not clear in this session. Examples: a build error, or no
  manager after 60 minutes; a boot loop; a SHA mismatch; a crash loop; a
  fingerprint other than `HYUNDAI_SANTA_FE_HEV_2022`, or `mock`, after two
  ignition cycles; no model output; a blocking alert or a cluster fault
  after two ignition cycles; a traceback from our code. These are hard
  triggers: the operator runs R1 and then reports. Radek's only part is the
  parked car with the ignition off.
- Settings: a single lost or changed key gets R2-key. The full settings
  restore (R2) runs after R1, with the old code running again.
- After the first drive: a safety event (unexpected hard braking, a missed
  stop, a steering fault, a repeating cluster fault) means Radek takes over
  and parks; then R1 and R2. A comfort difference is not a rollback trigger.
  Report it with a bookmark; phase 9 compares the drive with last week.
  Radek decides any rollback after the first drive.
- A rollback is complete only after R1, R2, and an ignition-on check in Park
  with the old log schema (R3).

### Physical recovery (both SSH routes fail)

Use this when 1.2 or 4.5 reports the device unreachable. **[RADEK:
physical]**, with the car parked and the ignition off:

1. Look at the device screen.
   - It shows build progress: wait until 60 minutes after the reboot, then
     the operator tries 4.5 again.
   - It shows an error screen: read the text to the operator. Touch nothing
     on it.
   - It is dark or does not respond for 2 minutes: go to step 2.
2. Power cycle the device. Unplug its power cable (the harness connector at
   the device) for 10 seconds, then plug it in again. A power cut also
   clears a dead NVMe (2026-10-25). The device boots with the ignition off.
3. The operator waits with 4.5 (15 minutes).
4. Never touch: "Uninstall", "Reset" or a factory reset, "Reset
   Calibration", Software > Update or Full Update, or an AGNOS update
   prompt.
5. If the device is still unreachable after the power cycle and 15
   minutes, STOP. The car can be driven without openpilot: unplug the
   device. The harness relay then passes the car's own signals, so the
   stock systems work. Report to the operator.

### Resume (a switch, R2 or R2-key stopped before it changed the tree)

<!-- p8:resume -->
```sh
setopt pipefail no_bang_hist
timeout -k 5 360 ssh -o BatchMode=yes -o ConnectTimeout=8 -o ServerAliveInterval=5 -o ServerAliveCountMax=3 $DEV bash -s -- "${UNIT}" <<'DEV'
set -euo pipefail
unit=${1:?unit}
sudo systemctl start "$unit"
for i in $(seq 1 60); do if pgrep -f '[m]anager\.py' >/dev/null; then echo "RESUMED: manager up"; exit 0; fi; sleep 5; done
echo "RESUME FAIL: ${unit} started, no manager after 5 minutes"; exit 10
DEV
```

PASS: `RESUMED`.

### R1. Code rollback to `OLD_SHA`, offline

Precondition **[RADEK: physical]**: the car is parked with the ignition
off. R1 needs no network: box 1.6 proved that `OLD_SHA` and all its objects
are on the device.

<!-- p8:target-rollback -->
```sh
setopt pipefail no_bang_hist
export TARGET_SHA=${OLD_SHA} TARGET_BRANCH='!my-fp-new' TARGET_MODE=rollback
echo "${TARGET_MODE} ${TARGET_BRANCH} ${TARGET_SHA}"
```

Then run the switch block of 4.4 unchanged. In rollback mode it does not
require `UpdaterState` idle or `Offroad_NeosUpdate` absent, because a failed
boot can leave them stale. It requires that no AGNOS flashing process runs.
It stops the unit, which also ends a failed or running build and any
updater, and checks that the unit's cgroup tree is empty. Then it takes
both update locks, makes the overlay ineligible, and resets to `OLD_SHA`
(the local `!my-fp-new` still points there). Then run 4.5, 4.6 and 4.7 unchanged; they
use `TARGET_SHA` and `TARGET_BRANCH`. PASS: `SHA_CHECK PASS` for `OLD_SHA`
on `!my-fp-new`. Then remove the local branch, so a later attempt starts
clean:

<!-- p8:remove-branch -->
```sh
setopt pipefail no_bang_hist
timeout -k 5 60 ssh -o BatchMode=yes -o ConnectTimeout=8 -o ServerAliveInterval=5 -o ServerAliveCountMax=3 $DEV 'cd /data/openpilot && git branch -D "!my-fp-1031" && git rev-parse HEAD && git symbolic-ref --short HEAD'
```

### R2. Full settings restore

Run after R1, with the old code built and the manager up. The script
quiesces as in 4.4: the unit stopped within 120 s and inactive, its whole
cgroup tree empty, no openpilot process anywhere. Only then does it delete
and extract. It verifies both stores against the listings from 3.7, checks
the cgroup again, and starts the unit only after all of this passes. Any
error ends it with the unit stopped. Then use resume, or run R2 again.

<!-- p8:r2 -->
```sh
setopt pipefail no_bang_hist
timeout -k 5 900 ssh -o BatchMode=yes -o ConnectTimeout=8 -o ServerAliveInterval=5 -o ServerAliveCountMax=3 $DEV bash -s -- "${UNIT}" "${UNIT_CGROUP}" <<'DEV' | tee "${P8}/06_r2.txt"
set -euo pipefail
unit=${1:?unit} cg=${2:?cgroup} B=/data/phase8_backup
fail() { echo "RESTORE FAIL: $1"; exit 8; }
self_cmd=$({ tr '\0' ' ' < "/proc/$$/cmdline"; } 2>/dev/null) || self_cmd=""   # this script's own command line, as pgrep -a prints it
self_cmd=${self_cmd% }
writers() {  # openpilot, update and flashing processes anywhere, also outside the unit's cgroup. A line equal to this script's own
             # command line is this script or one of its subshells, and is left out: the arguments hold the cgroup path, which the
             # pattern matches (run of 2026-11-14). An unreadable own command line counts as a running process
  local line
  if [ -z "$self_cmd" ]; then echo "this script cannot read its own command line (/proc/$$/cmdline)"; return 0; fi
  { pgrep -af '[m]anager\.py|[l]aunch_chffrplus|[s]elfdrive\.|[s]ystem\.|[f]rogpilot\.|[b]uild\.py|[s]cons|[f]ullupdate\.sh|[a]gnos\.py|[h]ardware/tici/updater' || true
    pgrep -ax 'ui|_ui|mapd|pandad|loggerd|encoderd|camerad' || true; } | while IFS= read -r line; do
    [ "${line#* }" = "$self_cmd" ] || printf '%s\n' "$line"
  done
}
cgroup_procs() {  # every PID in the unit's cgroup tree, child cgroups included, on each mounted hierarchy. A listing or read error
                  # returns 1 (set -e is off inside $(...)), so an unreadable cgroup.procs never counts as empty
  local r f files
  for r in /sys/fs/cgroup /sys/fs/cgroup/unified /sys/fs/cgroup/systemd; do
    if [ -d "$r$cg" ]; then
      files=$(find "$r$cg" -name cgroup.procs) || return 1
      while IFS= read -r f; do
        [ -n "$f" ] || continue
        [ -r "$f" ] || return 1
        cat "$f" || return 1
      done <<< "$files"
    fi
  done
  return 0
}
quiesce() {
  local state live i
  if [ "$(systemctl is-active "$unit" || true)" = active ]; then
    timeout 120 sudo systemctl stop "$unit" || fail "systemctl stop ${unit} failed or took over 120 s; nothing was changed"
  fi
  for i in $(seq 1 30); do
    state=$(systemctl is-active "$unit" || true)
    live=$(cgroup_procs) || fail "cannot read cgroup ${cg}; nothing was changed"
    if { [ "$state" = inactive ] || [ "$state" = failed ]; } && [ -z "${live//[[:space:]]/}" ] && [ -z "$(writers)" ]; then
      echo "QUIESCED: ${unit} ${state}, cgroup ${cg} empty, no openpilot, update or flashing process"; return 0
    fi
    sleep 1
  done
  echo "unit ${state}; PIDs in cgroup ${cg}: $(echo ${live:-none})"; writers
  fail "openpilot did not stop; nothing was changed"
}
for s in params_d cache_params_d; do
  if [ -f "$B/$s.tar" ] && [ ! -s "$B/$s.sha256" ]; then fail "$B/$s.sha256 is missing (box 3.7); nothing was changed"; fi
done
quiesce
restore_store() {  # <store link> <archive> <listing>
  local dir
  dir=$(readlink -f "$1")
  [ -d "$dir" ] || fail "$1 does not resolve to a directory"
  sudo find "$dir" -maxdepth 1 -type f ! -name .lock -delete
  sudo tar -C "$dir" --same-owner -xpf "$2"
  (cd "$dir" && find . -type f ! -name .lock -exec sha256sum {} + | LC_ALL=C sort -k2) | diff -q "$3" - >/dev/null || fail "$1 differs from the backup after the restore; unit left stopped"
}
restore_store /data/params/d "$B/params_d.tar" "$B/params_d.sha256"
echo "DATA_STORE_VERIFIED"
if [ -f "$B/cache_params_d.tar" ]; then restore_store /cache/params/d "$B/cache_params_d.tar" "$B/cache_params_d.sha256"; echo "CACHE_STORE_VERIFIED"; fi
live=$(cgroup_procs) || fail "cannot read cgroup ${cg}; unit left stopped"
[ -z "${live//[[:space:]]/}" ] && [ -z "$(writers)" ] || fail "an openpilot process started during the restore; unit left stopped"
sudo systemctl start "$unit"
for i in $(seq 1 60); do if pgrep -f '[m]anager\.py' >/dev/null; then echo "RESTORED_AND_STARTED"; exit 0; fi; sleep 5; done
fail "${unit} started, no manager after 5 minutes"
DEV
```

PASS: `QUIESCED`, `DATA_STORE_VERIFIED`, `CACHE_STORE_VERIFIED` (when
`/cache/params/d` exists) and `RESTORED_AND_STARTED`. Then compare the owner
and mode counts with 3.1 (run the 3.1 block). At start, the old manager
deletes its CLEAR_ON_* keys again; this is expected. The files in
`frogpilot_files.tar` (toggle backups, CEM override, cache flags, update
logs) return only on request: `tar -C / -xpf
/data/phase8_backup/frogpilot_files.tar <path>`, with paths as listed by
`tar -tvf`.

### R2-key. Restore one key

<!-- p8:r2-key -->
```sh
setopt pipefail no_bang_hist
K=LaneCenterOffset   # the key to restore; run once per key
timeout -k 5 600 ssh -o BatchMode=yes -o ConnectTimeout=8 -o ServerAliveInterval=5 -o ServerAliveCountMax=3 $DEV bash -s -- "${UNIT}" "${UNIT_CGROUP}" "${K}" <<'DEV' | tee "${P8}/06_r2_key_${K}.txt"
set -euo pipefail
unit=${1:?unit} cg=${2:?cgroup} key=${3:?key} B=/data/phase8_backup
fail() { echo "RESTORE FAIL: $1"; exit 8; }
self_cmd=$({ tr '\0' ' ' < "/proc/$$/cmdline"; } 2>/dev/null) || self_cmd=""   # this script's own command line, as pgrep -a prints it
self_cmd=${self_cmd% }
writers() {  # openpilot, update and flashing processes anywhere, also outside the unit's cgroup. A line equal to this script's own
             # command line is this script or one of its subshells, and is left out: the arguments hold the cgroup path, which the
             # pattern matches (run of 2026-11-14). An unreadable own command line counts as a running process
  local line
  if [ -z "$self_cmd" ]; then echo "this script cannot read its own command line (/proc/$$/cmdline)"; return 0; fi
  { pgrep -af '[m]anager\.py|[l]aunch_chffrplus|[s]elfdrive\.|[s]ystem\.|[f]rogpilot\.|[b]uild\.py|[s]cons|[f]ullupdate\.sh|[a]gnos\.py|[h]ardware/tici/updater' || true
    pgrep -ax 'ui|_ui|mapd|pandad|loggerd|encoderd|camerad' || true; } | while IFS= read -r line; do
    [ "${line#* }" = "$self_cmd" ] || printf '%s\n' "$line"
  done
}
cgroup_procs() {  # every PID in the unit's cgroup tree, child cgroups included, on each mounted hierarchy. A listing or read error
                  # returns 1 (set -e is off inside $(...)), so an unreadable cgroup.procs never counts as empty
  local r f files
  for r in /sys/fs/cgroup /sys/fs/cgroup/unified /sys/fs/cgroup/systemd; do
    if [ -d "$r$cg" ]; then
      files=$(find "$r$cg" -name cgroup.procs) || return 1
      while IFS= read -r f; do
        [ -n "$f" ] || continue
        [ -r "$f" ] || return 1
        cat "$f" || return 1
      done <<< "$files"
    fi
  done
  return 0
}
quiesce() {
  local state live i
  if [ "$(systemctl is-active "$unit" || true)" = active ]; then
    timeout 120 sudo systemctl stop "$unit" || fail "systemctl stop ${unit} failed or took over 120 s; nothing was changed"
  fi
  for i in $(seq 1 30); do
    state=$(systemctl is-active "$unit" || true)
    live=$(cgroup_procs) || fail "cannot read cgroup ${cg}; nothing was changed"
    if { [ "$state" = inactive ] || [ "$state" = failed ]; } && [ -z "${live//[[:space:]]/}" ] && [ -z "$(writers)" ]; then
      echo "QUIESCED: ${unit} ${state}, cgroup ${cg} empty, no openpilot, update or flashing process"; return 0
    fi
    sleep 1
  done
  echo "unit ${state}; PIDs in cgroup ${cg}: $(echo ${live:-none})"; writers
  fail "openpilot did not stop; nothing was changed"
}
want=$(awk -v k="./$key" '$2 == k {print $1}' "$B/params_d.sha256")
[ -n "$want" ] || fail "${key} is not in the backup listing; nothing was changed"
quiesce
dir=$(readlink -f /data/params/d)
sudo tar -C "$dir" --same-owner -xpf "$B/params_d.tar" "./$key"
have=$(sha256sum "$dir/$key" | cut -d' ' -f1)
[ "$have" = "$want" ] || fail "${key} differs from the backup after the restore; unit left stopped"
echo "KEY_VERIFIED ${key}"
sudo systemctl start "$unit"
for i in $(seq 1 60); do if pgrep -f '[m]anager\.py' >/dev/null; then echo "RESTORED_AND_STARTED"; exit 0; fi; sleep 5; done
fail "${unit} started, no manager after 5 minutes"
DEV
```

PASS: `KEY_VERIFIED` and `RESTORED_AND_STARTED`; then 5a.5 passes again.

### R3. Rollback completion check (old log schema)

After R1 and R2: run 5a.1 to 5a.3, 5a.6 and 5b.1 to 5b.6 with these
changes:
- The expected values are `OLD_SHA` on branch `!my-fp-new`.
- The old process list applies: The Pond and the sign reader do not exist
  there, so their `off` lines read 0.
- Decode the park segments with the old cereal (`TIP`, the subtract tip),
  because `!my-fp-new` writes the old schema:

<!-- p8:decode-old -->
```sh
setopt pipefail no_bang_hist
for s in 0 1; do PYTHONPATH="${TIP}:${TIP}/frogpilot/third_party" python3 "${P8}/A3.py" "${P8}/05b_rlog_${s}.zst" | tee "${P8}/06_r3_log_${s}.txt"; done
```

PASS: as 5b.5, with `commit` `OLD_SHA` and `branch` `!my-fp-new`.

## 7. First drive

### Before the drive

- Freeze: no push to `!my-fp-1031` until the drive report is read, unless
  Radek asks. A push reaches the car within about 5 minutes and restarts it
  at the next cruise-off.
- The drive is Radek's call (5b.7).

### What Radek watches (day-one rule: it should drive as today)

- Stops: rest gap mostly 4 to 5 m (3.0 m is the floor); no late grab and no
  pump; the creeping-queue follow; launches behind a departing lead.
- Engagement: the first set speed is the same as today (`InitialSetSpeed`
  160 km/h, or the current speed when that is higher).
- Distance button: short press = force coast; long press (0.5 s) = next
  personality; very long press (2.5 s) = next personality and Traffic Mode.
  LKAS button = Experimental Mode (CEM override).
- Always-on lateral: stays on with cruise main on while disengaged. It
  pauses only on an immediate-disable alert, and not on brake presses.
- Lane centering on curb roads (right-line hugging), curves (CSC) and map
  speed limits (SLC; no sign reader).
- Screens look different (new base UI); this is not a behaviour change.
  Note anything confusing.
- Any new alert text, any cluster warning, or an "Update Ready" notice
  (there must be none: nothing is pushed).

### How to bookmark

On the road, tap the camera view to open the sidebar. Then tap the flag
button at the bottom of the sidebar. Each tap writes a `userBookmark` into
the log. After the drive, tell the operator the approximate time and what
happened at each bookmark.

### After the drive (host)

1. Sync the routes without pruning. Run the dry run first and check that
   the plan deletes nothing; the defaults prune (30 days, 30 GiB).

<!-- p8:sync -->
```sh
setopt pipefail no_bang_hist
cd "${REPO}" && python3 tools/route_sync/refresh_routes.py --include-rlog --keep-days 3650 --max-cache-gib 4000 --max-protected-gib 4000 --dry-run
```

<!-- p8:sync-run -->
```sh
setopt pipefail no_bang_hist
cd "${REPO}" && python3 tools/route_sync/refresh_routes.py --include-rlog --keep-days 3650 --max-cache-gib 4000 --max-protected-gib 4000
```

2. Run the drive report from a `!my-fp-1031` checkout. Two log schemas
   (`CLOSURE.md`): a drive recorded after the switch decodes correctly only
   with the new cereal, and a drive from before the switch only with the old
   one. Never mix both in one report run. Keep post-switch drives out of the
   sim corpus until the tools choose the schema per log:
   `tools/stopping/sim/recorded.py` decodes with the old recording schema.

<!-- p8:report -->
```sh
setopt pipefail no_bang_hist
ROUTE=00000000--0000000000   # replace with the first drive's route name from the sync output
cd "${PORT}" && PYTHONPATH="${PORT}:${PORT}/frogpilot/third_party" python3 tools/stopping/drive_report.py --routes "${ROUTE}"
```

The report goes to `~/.route_sync/reports/drives/<route>.md`, with one line
in `INDEX.md`. Compare it with last week's lines from the old branch (phase
9), and send Radek the summary.

## Appendix A: host snippets

Save each block as the named file in `$P8` after box 0.2. Run it as the
boxes show.

### A1.py: record keys (secrets masked)

<!-- p8:a1 -->
```python
import hashlib, json, re, subprocess, sys
from pathlib import Path
d = Path(sys.argv[1])
rep = json.load(open(Path.home() / '.route_sync/work/fp1031/port-scratch/migrate_report_b1.json'))
ROW = re.compile(r'^\s*\{"([^"]+)",\s*\{(.*)\}\},?')
secret = {m.group(1) for ref in ('c022a9890c', 'refs/heads/!my-fp-1031') for l in subprocess.run(
  ['git', '-C', str(Path.home() / 'Repos/openpilot-rch'), 'show', f'{ref}:common/params_keys.h'], capture_output=True, text=True, check=True).stdout.splitlines()
  if (m := ROW.match(l)) and 'DONT_LOG' in m.group(2)}
groups = {
  'lane centering': ['LaneCentering', 'LaneCenteringE2EAuthority', 'LaneCenterOffset'],
  'migrate deleted': sorted(rep['deleted']), 'migrate rewritten': sorted(rep['rewritten']),
  'retyped': ['AggressivePersonalityProfile', 'PauseAOLOnBrake', 'RelaxedPersonalityProfile', 'SecOCKeys', 'StandardPersonalityProfile'],
  'core': ['AlphaLongitudinalEnabled', 'AlwaysOnLateral', 'AlwaysOnLateralLKAS', 'AutomaticUpdates', 'ConditionalExperimental', 'DisableOnroadUploads',
           'DistanceButtonControl', 'ExperimentalMode', 'GsmMetered', 'HigherBitrate', 'InitialSetSpeed', 'IsMetric', 'LKASButtonControl',
           'LongDistanceButtonControl', 'LongitudinalPersonality', 'NNFF', 'NoLogging', 'NoUploads', 'RecordFront', 'SetSpeedLimit', 'SpeedLimitController',
           'SteerKP', 'UseKonikServer', 'VeryLongDistanceButtonControl'],
}
for group, keys in groups.items():
  print(f'## {group}')
  for k in keys:
    p = d / k
    if not p.is_file():
      v = '<absent>'
    elif k in secret:
      raw = p.read_bytes()
      v = f'<secret: {len(raw)} bytes, sha256 {hashlib.sha256(raw).hexdigest()[:12]}>'
    else:
      v = repr(p.read_bytes())[2:-1][:120]
    print(f'{k}\t{v}')
```

### A2.py: compare the store before and after (exit 1 on an unexpected loss)

<!-- p8:a2 -->
```python
import json, re, subprocess, sys
from pathlib import Path
pre, post, report = Path(sys.argv[1]), Path(sys.argv[2]), json.load(open(sys.argv[3]))
ROW = re.compile(r'^\s*\{"([^"]+)",\s*\{(.*)\}\},?')
old_rows = {m.group(1): m.group(2) for l in subprocess.run(['git', '-C', str(Path.home() / 'Repos/openpilot-rch'), 'show', 'c022a9890c:common/params_keys.h'],
            capture_output=True, text=True, check=True).stdout.splitlines() if (m := ROW.match(l))}
a = {p.name: p.read_bytes() for p in pre.iterdir() if p.is_file() and not p.name.startswith('.')}
b = {p.name: p.read_bytes() for p in post.iterdir() if p.is_file() and not p.name.startswith('.')}
expected_gone = set(report['deleted']) | {k for k, row in old_rows.items() if 'CLEAR_ON_' in row}
lost = sorted(set(a) - set(b) - expected_gone)
changed = sorted(k for k in set(a) & set(b) if a[k] != b[k])
print('missing, expected:', len(set(a) - set(b)) - len(lost))
print('MISSING, NOT EXPECTED (restore these):', lost)
print('added:', sorted(set(b) - set(a)))
print('changed, expected rewrites:', [k for k in changed if k in report['rewritten']])
print('CHANGED, JUDGE EACH (state may change, a setting must not):', [k for k in changed if k not in report['rewritten']])
sys.exit(1 if lost else 0)
```

### A3.py: one log segment, every process failure with its time (run with a tree's PYTHONPATH)

<!-- p8:a3 -->
```python
import collections, statistics, sys
from openpilot.tools.lib.logreader import LogReader
MANAGER_HZ = 2.0      # managerState rate (system/manager/manager.py: MANAGER_STATE_DT = 0.5)
MIN_COVERAGE = 0.9    # at least 90 % of the samples the segment's span should hold
MAX_GAP_S = 2.0       # and no gap over 2 s, at the start and the end of the segment included
c = collections.Counter(); alerts, events, safety, gears = {}, {}, set(), set()
init = cp = first = last = None; mtimes = []; faults = set(); ign = set(); changes = []; state_of = {}; pid_of = {}; samples = []
for m in LogReader(sys.argv[1]):
  first = m.logMonoTime if first is None else first
  last = m.logMonoTime
  t = round((m.logMonoTime - first) / 1e9, 2)
  w = m.which(); c[w] += 1
  if w == 'initData': init = m.initData
  elif w == 'carParams': cp = m.carParams
  elif w == 'modelV2': mtimes.append(m.modelV2.modelExecutionTime)
  elif w == 'selfdriveState' and m.selfdriveState.alertType: alerts.setdefault(m.selfdriveState.alertType, t)
  elif w == 'onroadEvents':
    for e in m.onroadEvents: events.setdefault(str(e.name), t)
  elif w == 'pandaStates':
    for p in m.pandaStates:
      safety.add(str(p.safetyModel)); ign.add((p.ignitionLine, p.ignitionCan)); faults.update(str(f) for f in p.faults)
  elif w == 'carState': gears.add(str(m.carState.gearShifter))
  elif w == 'managerState':
    samples.append(t)
    for p in m.managerState.processes:
      state = 'ok'
      if p.shouldBeRunning and not p.running:
        state = f'not running, exit {p.exitCode}'
      elif p.exitCode != 0:
        state = f'exit {p.exitCode}'
      if state != state_of.get(p.name, 'ok'):
        changes.append((t, p.name, state))
      state_of[p.name] = state
      required = p.shouldBeRunning and p.running
      prev = pid_of.get(p.name)
      if required and prev and prev[1] and prev[0] != p.pid:   # required in both samples, new PID: it restarted in between
        changes.append((t, p.name, f'pid {prev[0]} -> {p.pid}, restarted between samples'))
      pid_of[p.name] = (p.pid, required)
span = (last - first) / 1e9 if first is not None else 0.0
gaps = [b - a for a, b in zip([0.0] + samples, samples + [span])]
max_gap = max(gaps) if samples else span
need = max(1, int(MIN_COVERAGE * MANAGER_HZ * span))
print('commit', init.gitCommit if init else None, 'branch', init.gitBranch if init else None, 'dirty', init.dirty if init else None)
print('fingerprint', cp.carFingerprint if cp else None, 'brand', cp.brand if cp else None, 'opLong', cp.openpilotLongitudinalControl if cp else None,
      'source', cp.fingerprintSource if cp else None)
print('modelV2', c['modelV2'], 'median exec s', round(statistics.median(mtimes), 4) if mtimes else None)
print('alerts (first seen, s)', sorted(alerts.items(), key=lambda x: x[1])); print('events (first seen, s)', sorted(events.items(), key=lambda x: x[1]))
print('safety', sorted(safety), 'ignition', sorted(ign), 'faults', sorted(faults)); print('gears', sorted(gears))
print(f'managerState coverage: {len(samples)} samples over {span:.1f} s (needs >= {need}), largest gap {max_gap:.2f} s (needs <= {MAX_GAP_S})')
print('process state changes (s, name, state)', changes)
bad = [x for x in changes if x[2] != 'ok']
if len(samples) < need or max_gap > MAX_GAP_S:
  print('PROCESS_CHECK FAIL: managerState evidence is missing or too thin')
elif bad:
  print('PROCESS_CHECK FAIL:', bad)
else:
  print('PROCESS_CHECK PASS')
```

### A4.sh: restore rehearsal on the host (box 3.6)

<!-- p8:a4 -->
```bash
set -euo pipefail
R="$P8/rehearsal"; rm -rf "$R"; mkdir -p "$R/params/.tmp_sim"; ln -s .tmp_sim "$R/params/d"
tar -C "$R/params/d" -xpf "$P8/backup/params_d.tar"
python3 - "$R/params/d" <<'PY'
import json, sys
from pathlib import Path
d = Path(sys.argv[1]); rep = json.load(open(Path.home() / '.route_sync/work/fp1031/port-scratch/migrate_report_b1.json'))
for k in rep['deleted']: (d / k).unlink(missing_ok=True)
for k, v in rep['rewritten'].items(): (d / k).write_text(v['after'])
for k, v in rep['added_defaults'].items(): (d / k).write_text(v)
PY
D=$(cd "$R/params/d" && pwd -P)
find "$D" -maxdepth 1 -type f ! -name .lock -delete
tar -C "$D" -xpf "$P8/backup/params_d.tar"
(cd "$R/params/d" && find . -type f ! -name .lock -exec shasum -a 256 {} + | LC_ALL=C sort -k2) > "$R/restored.sha256"
diff "$P8/backup/params_d.sha256" "$R/restored.sha256" && echo REHEARSAL_PASS
```

## Appendix B: host rehearsal record

All runs were recorded on 2026-11-02 on the host (macOS, zsh 5.9, bash 5.3,
GNU coreutils), with the venv active and
`PYTHONPATH=${PORT}:${PORT}/frogpilot/third_party`. Sandbox paths are
shortened to `<sb>`. Each rehearsal runs the reviewer's failure case and
the normal case.

```text
python tools/fullupdate/phase8_rehearse.py --rev f3a2cadec7   # revision 1: exit 1, items 1-11 UNSAFE
python tools/fullupdate/phase8_rehearse.py --rev ede4f0ad8b   # revision 2: exit 1, items 0-7 SAFE, items 8-11 UNSAFE
python tools/fullupdate/phase8_rehearse.py --rev 7b11e2a78e   # revision 3: exit 1, items 0-11 SAFE, items 12-13 UNSAFE
python tools/fullupdate/phase8_rehearse.py --rev 50a498eaa2   # revision 4: exit 1, items 0 and 14 UNSAFE (the 2026-11-02 harness hid the script from pgrep)
python tools/fullupdate/phase8_rehearse.py                    # revision 5: exit 0, items 0-14 SAFE
python -m pytest -n 0 tools/fullupdate/test_phase8_rehearse.py  # each revision on its items, syntax, sandbox containment: 32 passed
```

Syntax, revision 5: 48 host blocks pass `zsh -n`, 24 device scripts and A4
pass `bash -n`, and 3 Python blocks compile.

### B.1 Round 1: revision 1 against revision 2

Revisions 3 and 4 keep all of these SAFE. Revision 1 had 45 host blocks, and
its device commands were quoted strings, so there were no device scripts
to check.

| Item | Revision 1 (`f3a2cadec7`): fails | Revision 2 (`ede4f0ad8b`): passes |
|---|---|---|
| 1 R2 stop and verify | `systemctl stop` rc 5: prints `STILL_RUNNING`, then `RESTORE_PASS`; the store is changed, the unit started, rc 0. Writer alive after a good stop: the same. Normal run: the cache store is not verified. | rc 5: `RESTORE FAIL: systemctl stop comma.service failed; nothing was changed` (rc 8, store unchanged, not started). Writer alive: `RESTORE FAIL: openpilot processes still run; nothing was changed` (rc 8). Normal: `QUIESCED`, `DATA_STORE_VERIFIED`, `CACHE_STORE_VERIFIED`, `RESTORED_AND_STARTED`. |
| 2 effective AGNOS | tici, `/VERSION` 12.8, inherited `AGNOS_VERSION=16` (or one exported by `/data/launch_env.sh`): both trees' `launch_env.sh` give `['16', '16']`, a boot would flash, but the 1.3 predicate accepts (`model ok=True, agnos 12.8=True`). | `inherited_AGNOS_VERSION 16`, `tree candidate ... effective_AGNOS_VERSION 16 manifest c3_agnos.json`, `tree rollback ... effective_AGNOS_VERSION 16 manifest agnos.json`, `AGNOS_CHECK STOP: a boot would flash AGNOS`. Clean device: `AGNOS_CHECK PASS` with 12.8 for both trees (manifest digests equal, `920d24dadb50a70d`). The planted `WEATHER_TOKEN` value is never printed. |
| 3 overlay updater | The switch (`fullupdate.sh` parked path) leaves `updated` running. It touches `.overlay_init` after the reset next to a consistent finalized overlay, so the boot would install the finalized overlay; nothing checks it. | `QUIESCED: comma.service stopped, no openpilot process, both update locks held`, `OVERLAY_INELIGIBLE`, `VERIFIED_BEFORE_REBOOT`. `updated` no longer runs, and the boot would not install an overlay. |
| 4 offline rollback, waits, boot_id | R1b with the network down: `fatal: '<sb>/remote.git' does not appear to be a git repository`, rc 128, the candidate tree stays. The 4.5 wait for an unreachable device was still waiting when killed after 20 s. A device that did not reboot is accepted (any answer ends the wait). | Rollback with the network down: rc 0, `VERIFIED_BEFORE_REBOOT head <OLD_SHA> branch !my-fp-new`, old tree checked out. Unreachable: `UNREACHABLE for 15 minutes: section 6, physical recovery` (rc 1, bounded). Old `boot_id`: `NOT_REBOOTED: the device still answers with boot_id before-... after 15 minutes` (rc 1). |
| 5 storage | NVMe mounted with writes failing or hanging: box 1.3 prints `/data/media is a mountpoint`, rc 0, accepted. `Offroad_StorageMissing` in the backup and live: accepted by "only alerts the backup also had". | Writes failing and writes hanging: `STORAGE_CHECK STOP: the write probe on <sb>/data/media failed or did not finish in 5 s` (rc 4, within the 5 s bound). `STOP: Offroad_StorageMissing is set; the NVMe guard failed ...` (rc 4). |
| 6 log evidence | tmux fails: `tracebacks 0`, rc 0. Two sessions, the first not openpilot: the wrong pane is captured, `tracebacks 0`. Synthetic segment with modeld exit -11 at 20 s and a restart at 21 s: A3 prints `nonzero exit codes []`. | tmux fails: `LOG_EVIDENCE_MISSING: no tmux pane runs the launcher (pid 120, ancestors 120 110 )` (rc 1). Two sessions: `log_source tmux pane comma:0.0 runs launcher pid 120`, `tracebacks 1`. A3: `process state changes [(20.0, 'modeld', 'not running, exit -11'), (21.0, 'modeld', 'ok')]`, `PROCESS_CHECK FAIL`. |
| 7 zsh modifier, pipefail | `fatal: ambiguous argument '<CAND>ld/models/driving_supercombo.onnx'` (zsh `$CAND:s` modifier), then `e3b0c44298fc...` (digest of empty input), rc 0, also with a wrong `CAND`. `$CAND:s` found; 45 of 45 host blocks without `pipefail`. | `expected d2aa14fb... device d2aa14fb...`, `MODEL_MATCH`. Wrong `CAND`: `MODEL_CHECK FAIL: no driving_supercombo.onnx at 0000...` (rc 1). No `$VAR:<modifier>` pattern; 0 of 48 host blocks without `setopt pipefail`. |
| 0 whole procedure | (no block markers) | 37 steps in procedure order, each with its PASS marker: preconditions, backup, switch, reboot, build, 5a and 5b checks, R2-key, rollback switch, R2 (`DATA_STORE_VERIFIED`, `CACHE_STORE_VERIFIED`) and resume. After R2 the data store equals the backup listing. |

### B.2 Round 2: revision 2 against revision 3

| Item | Revision 2 (`ede4f0ad8b`): fails | Revision 3: passes |
|---|---|---|
| 8 (re-review 1) quiescence by cgroup | PID 777 (`git reset --hard`) stays in a child cgroup of `comma.service` after a good stop: `QUIESCED`, `VERIFIED_BEFORE_REBOOT`, `SWITCH_ISSUED`, the reset ran. `./loggerd` survives: R2 prints `QUIESCED` and `RESTORED_AND_STARTED` and changed the store. R2-key with PID 777: `KEY_VERIFIED`, `RESTORED_AND_STARTED`. A hanging `systemctl stop`: still waiting after 30 s (killed). | Switch: `unit inactive; PIDs in cgroup /system.slice/comma.service: 777`, `SWITCH FAIL: openpilot did not stop; nothing was changed (resume)`, no reset. R2 with `./loggerd`: PID 778 listed, `RESTORE FAIL: openpilot did not stop; nothing was changed` (rc 8, store unchanged, not started). R2-key: the same with PID 777. Hanging stop: `SWITCH FAIL: systemctl stop comma.service failed or took over 120 s` (the 120 s limit, scaled). |
| 9 (re-review 2) rollback and stale params | The rollback (switch block with the old SHA) stops on a stale `UpdaterState=checking...` with no updater process: `SWITCH FAIL: updated is busy (checking...)`; the candidate stays installed. A stale `Offroad_NeosUpdate` with no flashing process stops it too. | Rollback mode prints `UpdaterState=checking...` and `not required`, then `QUIESCED`, `VERIFIED_BEFORE_REBOOT head <OLD_SHA> branch !my-fp-new mode rollback`; it also rolls back with a stale `Offroad_NeosUpdate`. A running `agnos.py` still blocks it: `SWITCH FAIL: AGNOS flashing runs; let it finish, then run this block again`. |
| 10 (re-review 3) A3 evidence | 1200 `modelV2` and no managerState: `PROCESS_CHECK PASS (managerState samples: 0 )`. 120 healthy samples with modeld pid 100 to 300 at 20 s: `PROCESS_CHECK PASS`. | No managerState: `managerState coverage: 0 samples over 60.0 s (needs >= 107), largest gap 59.95 s`, `PROCESS_CHECK FAIL: managerState evidence is missing or too thin`. PID change: `(20.0, 'modeld', 'pid 100 -> 300, restarted between samples')`, `PROCESS_CHECK FAIL`. Healthy segment: 120 samples, largest gap 0.50 s, `PROCESS_CHECK PASS`. |
| 11 (re-review 4) hard bounds | ssh stalls after connecting: the 4.5 wait (killed after 15 s), the build wait (killed after 30 s) and box 1.3 (killed after 15 s) all hang. 34 ssh calls in host blocks have no per-call limit. | Scaled by 0.005 (900 s become 4.5 s; revision 3 read the scale from an environment variable, revision 4 lets only the harness scale its copy): 4.5 ends `UNREACHABLE for 5 s: section 6, physical recovery` after 4.5 s. The build wait times out each check (`CHECK_TIMED_OUT`) and ends `BUILD_DEADLINE` after 18.0 s (3600 s scaled). Box 1.3 ends after 0.4 s (its `timeout -k 5 60`, scaled). 0 ssh calls without `timeout -k` and ServerAlive options; both wait loops use `time.monotonic()` and per-call timeouts. |

The PLAN nit (re-review): `PLAN.md` phase 8 now records the manual-switch
exception: what the manual path keeps, adds and drops.

### B.3 Round 3: revision 3 against revision 4

Revision 4 keeps items 0-11 SAFE.

| Item | Revision 3 (`7b11e2a78e`): fails | Revision 4: passes |
|---|---|---|
| 12 (round 3, 1) cgroup read errors | PID 777 in a child cgroup whose `cgroup.procs` has mode 000. v2 layout: `cat: <sb>/sys/fs/cgroup/system.slice/comma.service/payload/cgroup.procs: Permission denied`, then `QUIESCED: ... cgroup /system.slice/comma.service empty`. The switch prints `VERIFIED_BEFORE_REBOOT` and `SWITCH_ISSUED` (reset and reboot ran); R2 prints `RESTORED_AND_STARTED`; R2-key prints `KEY_VERIFIED`, `RESTORED_AND_STARTED`. Both stores changed. The absent `/unified` and `/systemd` hierarchies later in the loop masked the failed read. In this sandbox, v1 and hybrid already stopped, because the unreadable hierarchy came last. | v2, v1 and hybrid: the switch prints `SWITCH FAIL: cannot read cgroup /system.slice/comma.service; nothing was changed (resume)`; R2 and R2-key print `RESTORE FAIL: cannot read cgroup /system.slice/comma.service; nothing was changed` (rc 8). Nothing is reset, the stores are unchanged, the unit is not started. A readable, empty tree still lets the switch run on each layout (`QUIESCED`, `SWITCH_ISSUED`). |
| 13 (round 3, 2) timing from the environment | A shell that inherited `P8_REHEARSAL_TIME_SCALE=0.005`, with the device unreachable: the 4.5 wait ends after 4.54 s (physical recovery), and the build wait after 18.05 s (R1), instead of 900 s and 3600 s. The blocks contain `os.environ.get('P8_REHEARSAL_TIME_SCALE', '1')`. | The same shell: both waits are still waiting after 10 s and 25 s (the rehearsal stops them), as without the variable. No environment-controlled timing in any block; the limits are literal (`LIMIT, CALL, PAUSE = 900, 30, 20` and `3600, 60, 300`). Box 0.2 no longer needs an `unset`. |

### B.4 Run of 2026-11-14: revision 4 against revision 5

Boxes 0.2 to 4.3 passed on the device. Box 4.4 stopped the unit and then
printed, with nothing changed (resume brought the old code back):

```text
unit inactive; PIDs in cgroup /system.slice/comma.service: none
123822 bash -s -- comma.service /system.slice/comma.service 2322fadf9370b27903f956b21a0d94a5c527db3b !my-fp-1031 switch
SWITCH FAIL: openpilot did not stop; nothing was changed (resume)
```

The pattern `[s]ystem\.` of `writers` matches `/system.slice/` in the
script's own arguments. The harness of 2026-11-02 answered `pgrep` from a
table that did not hold the calling script, so it could not show this. The
same defect blocked the rollback (R1 uses the switch block), R2 and R2-key.

| Item | Revision 4 (`50a498eaa2`): fails | Revision 5: passes |
|---|---|---|
| 14 the process check and the script itself | With the script and its subshells in the `pgrep` answer (real PIDs, as on the device): the switch, the rollback, R2 and R2-key all stop at `openpilot did not stop` on a clean device. | Clean device: `SWITCH_ISSUED` for the switch and for the rollback; R2 and R2-key print `RESTORED_AND_STARTED` and restore the key. Still blocked: an updater outside the cgroup (`system.updated.updated`), a second device script with other arguments, and a script that cannot read its own command line. |
| 0 whole procedure | With the same `pgrep` answer: the switch, the rollback, R2 and R2-key steps fail. | All steps print their PASS marker. |

Device evidence for revision 5 (2026-11-14, read-only, unit active, the
script started with the box 4.4 arguments): `self_cmd` equals the `pgrep -a`
text of the script; `writers` lists the 16 openpilot processes and no line
of the script, in a subshell and in the main shell; with `self_cmd` empty it
prints `this script cannot read its own command line`.

What the sandbox cannot show, still to verify on the device or by review:
- the real cgroup layout (v1, v2 or hybrid) and `ControlGroup` value of
  the unit; box 1.3 proves it holds the manager before anything changes;
- the real tmux layout and the real launcher environment;
- GNU tool behaviour (`df --output`, `stat -c`, bash 5 `mapfile -d`);
- an NVMe failing with EIO or a D-state hang (the sandbox uses a write
  refusal and a 30 s hang);
- AGNOS itself, the build time, and ssh behaviour across a real reboot.
