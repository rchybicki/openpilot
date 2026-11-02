# Phase 8 operating procedure: move the device to `!my-fp-1031`

Status: draft for plan review. Do not run it before the review closes and
Radek gives the go in box 0.1.

Sources: `PLAN.md` phase 8 and the validation matrix "device" row;
`AGENTS.md` "Device Update Workflow"; `DECISIONS.md` F (network rows);
`CLOSURE.md` (two log schemas, `Params::clearAll`).

Decisions (Radek): the device switches its checked-out branch to
`!my-fp-1031`. `!my-fp-new` stays untouched for rollback. No force-push. The
first transition runs parked, with the ignition off.

## How to use this checklist

- The operator runs every box from the host, in order, in one zsh session.
  Each box has the exact command and a PASS line. Save each output under
  `$P8` with the box number in the file name. Write PASS or FAIL and the
  time into `$P8/RECORD.md`.
- **[RADEK: physical]**: Radek acts at the car. **[RADEK: go]**: Radek
  approves before the box runs. All other boxes need no one at the car.
- On FAIL: do what the box says (STOP, R1, R2 or R2-key in section 6).
  STOP means: change nothing more on the device, report, and wait.
- Passive reads only: `ps`, `cat`, `ls`, `stat`, `tmux capture-pane` and
  copies of closed log files. No `SubMaster`, `sub_sock` or other msgq reader
  on the device, and no import of driving modules there (AGENTS.md).
- Do not tap Settings > Software > Full Update, and do not push to
  `!my-fp-new` or `!my-fp-1031` while this procedure runs (except box 2.3).
  The device fetches and applies pushes to its branch within about 5 minutes.
- Device commands are single-quoted on the host. Do not type them into an
  interactive device shell: bash expands `!` inside double quotes there.
- Process checks use bracket patterns such as `pgrep -f "[m]anager.py"`.
  ssh runs each command through `bash -c`, so that shell's command line
  contains every pattern in the command. A plain `pgrep -f manager.py` would
  also count that shell. The bracket pattern matches the process but not the
  pattern text. Keep the brackets.
- Save the appendix files `A1.py`, `A2.py`, `A3.py` and `A4.sh` in `$P8`
  after box 0.2.
- Plan about 2.5 hours with the car parked. The longest step is the first
  build after the switch.

## Facts this procedure relies on

- `fullupdate.sh` is the same file on `!my-fp-new` (`6272ccdbaa`), the
  subtract tip and `!my-fp-1031`. It fetches `refs/heads/<branch>` from the
  checked-out branch's upstream remote, or from `origin` when the branch has
  none (`fullupdate.sh:538-550`). Then it runs `git reset --hard FETCH_HEAD`
  (`:551`) and `git submodule update -f` (`:552`, a no-op: neither tree has
  submodules). It takes `/data/fullupdate.lock` (`:22`); a second run prints
  "A full update is already running" and exits. With the car off-road and
  the ignition off, it deletes `/data/openpilot/prebuilt` and reboots at once
  (`:416-418`). Otherwise it stages the update and starts the reboot
  supervisor. Before the fetch it runs `ensure_runtime_helpers` (`:528`). If
  the tree on the device cannot import `cereal` and `Params`, it builds that
  tree first.
- Automatic updates (`AutomaticUpdates`, 1 on the device): every 5 minutes
  `frogpilot_process` compares `HEAD` with `git ls-remote` of the branch's
  upstream. When they differ, it starts `fullupdate.sh` and appends the
  output to `/data/fullupdate.log` (old branch `frogpilot_functions.py:245`,
  new branch `:308`). When `HEAD` equals the remote branch, it does nothing.
  After the switch, every push to `!my-fp-1031` reaches the car.
- openpilot's own `updated` also runs on both branches. Its staged overlay
  installs at boot only if `.git` did not change after `.overlay_init`
  (`launch_chffrplus.sh:52-75`). Every `fullupdate.sh` run changes `.git`.
  The full-SHA check after each reboot catches any other outcome.
- Boot (`launch_chffrplus.sh`): `agnos_init` (`:83`) flashes AGNOS when
  `/VERSION` differs from `AGNOS_VERSION`. `!my-fp-1031` sets 16, or 12.8 on
  a device whose model reads "comma tici" or "comma tizi"
  (`launch_env.sh:18-25`). Box 1.3 confirms 12.8 and tici or tizi, so no
  flash. Then `build.py` runs when `prebuilt` is missing (`:98`). It runs
  scons with at most 4 jobs on the device and retries with fewer, and it
  compiles the driving and driver-monitoring models with tinygrad. Nobody
  has timed a build after a base change on this device.
- Manager start (`system/manager/manager.py:42-64`) runs `clear_all` for
  the four CLEAR flags, deletes stored keys the new registry lacks, then
  runs `migrate_params`, syncs `/cache/params` and writes defaults. The
  migration probe on a copy of today's store
  (`~/.route_sync/work/fp1031/port-scratch/migrate_report_b1.json`) found
  50 keys deleted (8 PERSISTENT: `AvailableModels`, `AvailableModelNames`,
  `DrivingModelName`, `DrivingModelVersion`, `OverpassRequests`,
  `SpeedLimits`, `SpeedLimitsFiltered`, `TinygradUpdateAvailable`; and the
  unregistered `IdentificationTestArmed`), 2 rewritten
  (`MapdSettings` `{}` to `{"settings_version": 2}`, `SecOCKeys` `''` to
  `[]`), 17 defaults added and 55 deleted from `/cache/params`. The same
  probe at `08925e26c9` gives the same lists
  (`port-scratch/phase8/migrate_report_final.json`).
- Five stored keys change type: `AggressivePersonalityProfile`,
  `RelaxedPersonalityProfile` and `StandardPersonalityProfile` (BOOL to
  STRING), `PauseAOLOnBrake` (BOOL to INT) and `SecOCKeys` (STRING to JSON).
  The device stores `0` or an empty value in each.
- `/data/params/d` is a symlink to `/data/params/.tmp_XXXXXX`
  (`common/params.cc:34-62`); `/cache/params/d` has the same layout. Backup
  and restore work on the directory contents, not the symlink.
- The new onboarding shows the telemetry consent page until it is answered
  (`selfdrive/ui/qt/offroad/onboarding.h:97`). Radek's answer is "No Thanks"
  (`DECISIONS.md` F). Terms and training versions are unchanged, so no other
  onboarding page appears.
- `modeld` and `card` run only with the ignition on
  (`system/manager/process_config.py:109,121`). The model, firmware query
  and fingerprint checks therefore run in box group 5b, ignition on in Park.
- Upload state today: `NoUploads` 1, `DisableOnroadUploads` 0, `NoLogging`
  0. The Pond and the speed-limit sign reader never start on this branch
  (`process_config.py:81-88`).

## 0. Start

- [ ] 0.1 **[RADEK: go]** Radek approves phase 8 and the time slot. Plan
  review of this procedure is closed.
- [ ] 0.2 Set the session variables and the record directory.

  ```sh
  setopt no_bang_hist 2>/dev/null || set +H
  export DEV=comma                    # if a box cannot reach it: export DEV=commawifi, run the box again
  export REPO=~/Repos/openpilot-rch
  export PORT=~/.route_sync/work/fp1031/port-wt
  export REVIEWED=08925e26c9          # last code head with a SHIP review of record
  export P8=~/.route_sync/work/fp1031/phase8/run_$(date -u +%Y%m%dT%H%MZ)
  mkdir -p "$P8" && chmod 700 "$P8" && touch "$P8/RECORD.md" && echo "$P8"
  source "$REPO/.venv/bin/activate"
  ```

  PASS: the directory exists. Every later box uses these variables. Write
  them into `RECORD.md`, together with `CAND`, `ORIGIN_NEW` and
  `DEVICE_HEAD` when boxes 1.5 and 2.1 set them. After a lost session,
  export the same values again from `RECORD.md`; do not run 0.2 again.

## 1. Preconditions

- [ ] 1.1 **[RADEK: physical]** The car is parked in P with the ignition
  off. Nobody drives it for the next 2.5 hours. The 12 V battery is in good
  condition: the device builds for up to an hour with the ignition off.
- [ ] 1.2 The device answers.

  ```sh
  ssh -o BatchMode=yes -o ConnectTimeout=8 $DEV true && echo REACHABLE
  ```

  PASS: `REACHABLE`. If both `comma` and `commawifi` fail, the device may be
  asleep. **[RADEK: physical]** Turn the ignition on (stay in P, do not
  drive) until the device boots, then turn it off again. Run the box again
  after 3 minutes.
- [ ] 1.3 Record the device state.

  ```sh
  ssh $DEV 'cd /data/openpilot
    echo "utc $(date -u +%FT%TZ) uptime_s $(cut -d" " -f1 /proc/uptime)"
    echo "model $(tr -d "\0" < /sys/firmware/devicetree/base/model)"; echo "agnos $(cat /VERSION)"
    echo "head $(git rev-parse HEAD)"; echo "branch $(git symbolic-ref --short HEAD)"
    echo "local_new $(git rev-parse "refs/heads/!my-fp-new")"
    git log -1 --format="commit %H %cI %s"
    echo "origin $(git remote get-url origin)"; git config --get-all remote.origin.fetch
    git config --get-regexp "^branch\."
    echo "upstream $(git rev-parse --abbrev-ref "@{u}" 2>&1)"
    echo "tracked_changes $(git status --porcelain --untracked-files=no | wc -l)"
    git show-ref --verify "refs/heads/!my-fp-1031" 2>&1 | sed "s/^/branch_1031 /"
    for k in IsOnroad IsEngaged AutomaticUpdates LiveUpdateHandoffState; do echo "param $k=$(cat /data/params/d/$k 2>/dev/null || echo "<absent>")"; done
    mountpoint /data/media; df -h /data /cache | sed "s/^/df /"' | tee "$P8/01_device_state.txt"
  ```

  PASS, all of these:
  - `model` contains "comma tici" or "comma tizi", and `agnos` is `12.8`.
    Otherwise STOP: the new `launch_env.sh` would flash AGNOS 16 at boot.
  - `branch` is `!my-fp-new` and `local_new` equals `head`.
  - `origin` is `git@github.com:rchybicki/openpilot.git`.
  - `tracked_changes` is 0. Otherwise STOP and show Radek the changes:
    `fullupdate.sh` would discard them.
  - `branch_1031` reports that the ref does not exist. Otherwise STOP: a
    branch from an earlier attempt exists, so look at it before you go on.
  - `IsOnroad` and `IsEngaged` are 0 or absent, and `LiveUpdateHandoffState`
    is absent or empty.
  - `/data/media` is a mountpoint (the NVMe), and `/data` has at least 5 GB
    free.
- [ ] 1.4 No update runs and no reboot is pending.

  ```sh
  ssh $DEV 'echo "supervisor_pid $(cat /data/fullupdate_reboot.pid 2>/dev/null || echo "<none>")"
    echo "supervisor_procs $(pgrep -f "[_]_reboot_when_parked" | wc -l)"
    echo "fullupdate_procs $(pgrep -f "[f]ullupdate.sh" | wc -l)"
    flock -n /data/fullupdate.lock true && echo "update_lock free" || echo "update_lock HELD"
    ls -la /data/fullupdate_reboot.cancel 2>&1 | sed "s/^/cancel /"
    echo "--- fullupdate_reboot.log"; tail -n 15 /data/fullupdate_reboot.log 2>&1
    echo "--- fullupdate.log"; tail -n 15 /data/fullupdate.log 2>&1
    echo "--- overlay"; ls -la /data/safe_staging 2>&1; ls -la /data/openpilot/.overlay_init /data/openpilot/prebuilt 2>&1' | tee "$P8/01_updater_state.txt"
  ```

  PASS: `supervisor_procs 0`, `fullupdate_procs 0` and `update_lock free`.
  A cancel file left from an earlier run is harmless: `fullupdate.sh`
  deletes it at start. Record it anyway.
- [ ] 1.5 Origin state, and what a rollback would install (on the host).

  ```sh
  git -C "$REPO" ls-remote origin 'refs/heads/!my-fp-new' 'refs/heads/!my-fp-1031' \
    'refs/heads/backup/!my-fp-new_10-31_pre-fp1031-refresh' | tee "$P8/01_origin.txt"
  export ORIGIN_NEW=$(awk '$2=="refs/heads/!my-fp-new" {print $1}' "$P8/01_origin.txt")
  export DEVICE_HEAD=$(awk '$1=="head" {print $2}' "$P8/01_device_state.txt")
  git -C "$REPO" cat-file -e "$DEVICE_HEAD^{commit}" && git -C "$REPO" cat-file -e "$ORIGIN_NEW^{commit}" && echo BOTH_KNOWN
  git -C "$REPO" diff --name-only "$DEVICE_HEAD" "$ORIGIN_NEW" -- . ':(exclude)tools' ':(exclude)docs'; echo "end of non-tool changes"
  ```

  PASS: `!my-fp-new` and the backup branch are on origin; `BOTH_KNOWN`
  (otherwise run `git -C "$REPO" fetch origin 'refs/heads/!my-fp-new'` and
  repeat); nothing is listed before "end of non-tool changes", so a rollback
  (R1) installs the same driving code the car runs now. Record `ORIGIN_NEW`
  and `DEVICE_HEAD` in `RECORD.md`. (On 2026-11-02, `f77b26830f..6272ccdbaa` changes
  only `tools/` and `docs/`.) If code files are listed, STOP and show Radek.

## 2. Push the branch

- [ ] 2.1 Fix the candidate (on the host).

  ```sh
  export CAND=$(git -C "$REPO" rev-parse 'refs/heads/!my-fp-1031') && echo "$CAND" | tee "$P8/02_candidate_sha.txt"
  git -C "$REPO" merge-base --is-ancestor 437be91e24 "$CAND" && echo ON_NEW_BASE
  git -C "$REPO" diff --name-only "$REVIEWED" "$CAND" | grep -v '^docs/' ; echo "end of code changes since review"
  git -C "$PORT" status --porcelain | wc -l
  ```

  PASS: `ON_NEW_BASE`; nothing is listed before "end of code changes since
  review"; the port worktree is clean (0). A code change after `$REVIEWED`
  needs its review first: STOP.
- [ ] 2.2 Origin has no `!my-fp-1031`, or only an ancestor of `$CAND`.

  ```sh
  R=$(git -C "$REPO" ls-remote origin 'refs/heads/!my-fp-1031' | cut -f1); echo "remote: ${R:-<none>}"
  [ -z "$R" ] || git -C "$REPO" merge-base --is-ancestor "$R" "$CAND" && echo FAST_FORWARD_OK
  ```

  PASS: `FAST_FORWARD_OK`. Otherwise STOP. Never force-push.
- [ ] 2.3 Push.

  ```sh
  git -C "$REPO" push origin 'refs/heads/!my-fp-1031:refs/heads/!my-fp-1031' 2>&1 | tee "$P8/02_push.txt"
  ```

  PASS: exit 0, and the output has no "forced update".
- [ ] 2.4 Verify on origin.

  ```sh
  git -C "$REPO" ls-remote origin 'refs/heads/!my-fp-1031' 'refs/heads/!my-fp-new' | tee "$P8/02_origin_after.txt"
  ```

  PASS: `!my-fp-1031` equals `$CAND`, and `!my-fp-new` still equals
  `$ORIGIN_NEW`.
- [ ] 2.5 The device sees the branch with its deploy key.

  ```sh
  ssh $DEV 'cd /data/openpilot && git ls-remote origin "refs/heads/!my-fp-1031"' | tee "$P8/02_device_ls_remote.txt"
  ```

  PASS: the SHA equals `$CAND`. On "Permission denied (publickey)", follow
  the key diagnosis in AGENTS.md and STOP.
- [ ] 2.6 The device downloads the objects. This changes no branch and no
  file in the working tree.

  ```sh
  ssh $DEV 'cd /data/openpilot && git fetch origin "+refs/heads/!my-fp-1031:refs/remotes/origin/!my-fp-1031" \
    && git rev-parse "refs/remotes/origin/!my-fp-1031" && git symbolic-ref --short HEAD && git rev-parse HEAD' | tee "$P8/02_device_fetch.txt"
  ```

  PASS: the first SHA equals `$CAND`. The branch is still `!my-fp-new` and
  `HEAD` still equals `DEVICE_HEAD`. A large download can take minutes.

From here on, a new commit on `!my-fp-1031` changes `$CAND`. Then run 2.1
to 2.6 again before section 4.

## 3. Settings backup and restore rehearsal

The backup contains tokens and keys (the DONT_LOG params). Keep it in `$P8`
(mode 700). Never commit it, copy it to Drive or share it.

- [ ] 3.1 Record owners and modes.

  ```sh
  ssh $DEV 'for s in /data/params /cache/params; do [ -e $s/d ] || { echo "$s/d absent"; continue; }
      t=$(readlink -f $s/d); echo "$s/d -> $t"; stat -c "%U:%G %a %n" $s "$t"
      find "$t" -maxdepth 1 -type f -printf "%u:%g %m\n" | sort | uniq -c; done
    stat -c "%U:%G %a %n" /data/ssh /data/ssh/id_ed25519' | tee "$P8/03_owners.txt"
  ```

  PASS: the output is recorded. These values are the target of every
  restore.
- [ ] 3.2 Make the archives on the device (kept there for a fast restore).

  ```sh
  ssh $DEV 'set -e; B=/data/phase8_backup
    if [ -e $B ]; then echo "EXISTS: $B (move it aside first)"; exit 3; fi
    mkdir -p $B
    tar -C /data/params/d -cf $B/params_d.tar .
    if [ -d /cache/params/d ]; then tar -C /cache/params/d -cf $B/cache_params_d.tar .; fi
    cd /; L=""; for f in data/toggle_backups data/conditional_experimental_override cache/use_HD cache/use_konik cache/on_backup \
      data/fullupdate.log data/fullupdate_reboot.log; do [ -e "$f" ] && L="$L $f"; done; echo "files:$L"; tar -cf $B/frogpilot_files.tar $L
    { ls -la /data/backups /data/models /data/tailscale /persist/frogsgomoo.py; du -sh /data/backups; } > $B/listings.txt 2>&1 || true
    cd $B && sha256sum *.tar > SHA256SUMS && cat SHA256SUMS' | tee "$P8/03_device_backup.txt"
  ```

  PASS: exit 0, and `SHA256SUMS` lists `params_d.tar`,
  `frogpilot_files.tar` and (when `/cache/params/d` exists)
  `cache_params_d.tar`. The new base prunes `/data/toggle_backups` to five
  entries and `/data/backups` to three code archives at boot. The
  `/data/backups` code archives are only listed: git is the code backup.
- [ ] 3.3 Copy the archives to the host and check them.

  ```sh
  mkdir -p "$P8/backup" && ssh $DEV 'tar -C /data/phase8_backup -cf - .' | tar -C "$P8/backup" -xf - \
    && (cd "$P8/backup" && shasum -a 256 -c SHA256SUMS) && chmod -R go-rwx "$P8"
  ```

  PASS: every file reports `OK`.
- [ ] 3.4 Extract the reference copy and its listing.

  ```sh
  mkdir -p "$P8/backup/params_d" && tar -C "$P8/backup/params_d" -xpf "$P8/backup/params_d.tar"
  (cd "$P8/backup/params_d" && find . -type f ! -name .lock -exec shasum -a 256 {} + | LC_ALL=C sort -k2) > "$P8/backup/params_d.sha256"
  wc -l < "$P8/backup/params_d.sha256"
  ```

  PASS: at least 400 entries (route 00002310 logged 452 keys).
- [ ] 3.5 Record the lane-centering values, the keys the migration deletes,
  rewrites or retypes, and the core settings. Run appendix A1.

  ```sh
  python3 "$P8/A1.py" "$P8/backup/params_d" > "$P8/03_record_keys.txt"
  ssh $DEV 'cd /data/params/d && for k in LaneCentering LaneCenteringE2EAuthority LaneCenterOffset; do echo "$k=$(cat $k 2>/dev/null || echo "<absent>")"; done' \
    | tee "$P8/03_lane_centering_live.txt"
  ```

  PASS: the file is written, and the three live lane-centering values equal
  the backup. The last known values (2026-10-04) are `LaneCentering` 1,
  `LaneCenteringE2EAuthority` 0.0 and `LaneCenterOffset` 0.0. Show Radek any
  difference; do not change it. If `LaneCenteringE2EAuthority` is ever lost,
  the new default 1.0 applies, so compare it at 5a.5.
- [ ] 3.6 Restore rehearsal on the host (appendix A4). It simulates the
  post-switch store, runs the restore sequence of R2 on it, and compares
  the result with the reference.

  ```sh
  bash "$P8/A4.sh"
  ```

  PASS: `REHEARSAL_PASS`. (Tested 2026-11-02 on a synthetic store with this
  layout: it passes, and it fails when the clear step is left out.)
- [ ] 3.7 Rehearse the extraction with the device's own tar, into a scratch
  directory. The live store is not touched.

  ```sh
  ssh $DEV 'set -e; R=/data/phase8_rehearsal; rm -rf $R; mkdir -p $R/d; tar -C $R/d -xpf /data/phase8_backup/params_d.tar
    cd $R/d && find . -type f ! -name .lock -exec sha256sum {} + | LC_ALL=C sort -k2; rm -rf $R' > "$P8/03_device_rehearsal.sha256"
  diff "$P8/backup/params_d.sha256" "$P8/03_device_rehearsal.sha256" && echo DEVICE_TAR_PASS
  ```

  PASS: `DEVICE_TAR_PASS`.
- [ ] 3.8 Write the restore commands (section 6, R2 and R2-key) into
  `RECORD.md` with the paths of this run. Do not run them now.

## 4. Switch

- [ ] 4.1 **[RADEK: go]** Radek approves the switch. The car is still
  parked with the ignition off.
- [ ] 4.2 Check again just before the switch.

  ```sh
  ssh $DEV 'cd /data/openpilot; for k in IsOnroad IsEngaged; do echo "$k=$(cat /data/params/d/$k 2>/dev/null || echo 0)"; done
    pgrep -f "[f]ullupdate.sh|[_]_reboot_when_parked" | wc -l
    git ls-remote origin "refs/heads/!my-fp-1031"; git rev-parse "refs/remotes/origin/!my-fp-1031"; git rev-parse HEAD'
  ```

  PASS: `IsOnroad=0` and `IsEngaged=0`; 0 update processes; both SHAs equal
  `$CAND`; `HEAD` equals `DEVICE_HEAD`.
- [ ] 4.3 Create the local branch with its upstream. This changes no file.

  ```sh
  ssh $DEV 'cd /data/openpilot && git branch --no-track "!my-fp-1031" '"$CAND"' \
    && git config "branch.!my-fp-1031.remote" origin && git config "branch.!my-fp-1031.merge" "refs/heads/!my-fp-1031" \
    && git rev-parse "refs/heads/!my-fp-1031" && git config --get-regexp "^branch\.!my-fp-1031\." \
    && git rev-parse --abbrev-ref "!my-fp-1031@{u}" 2>&1' | tee "$P8/04_branch.txt"
  ```

  PASS: the branch SHA equals `$CAND`, the remote is `origin` and the merge
  ref is `refs/heads/!my-fp-1031`. The last line is `origin/!my-fp-1031`.
  If instead it says the upstream is "not stored as a remote-tracking
  branch" (the origin fetch refspec in 1.3 does not cover the branch),
  `fullupdate.sh` and the auto-updater use their fallback, `origin` plus the
  branch name, which gives the same result. Record which case applies.
- [ ] 4.4 Point `HEAD` at the new branch, then start the update at once.
  `git symbolic-ref` changes no file; the reset in `fullupdate.sh` writes the
  new tree, as in every deploy. In the seconds between the two commands,
  `git status` shows the old tree as changed. Do not run checkout, stash or
  reset by hand.

  ```sh
  ssh $DEV 'cd /data/openpilot && git symbolic-ref HEAD "refs/heads/!my-fp-1031" && git symbolic-ref --short HEAD && git rev-parse HEAD' \
    | tee "$P8/04_head.txt"
  ssh -tt $DEV 'cd /data/openpilot && ./fullupdate.sh' 2>&1 | tee "$P8/04_fullupdate.txt"
  ```

  PASS: the first command prints `!my-fp-1031` and `$CAND`. The second
  prints the fetch of `refs/heads/!my-fp-1031` and `HEAD is now at <short
  $CAND> ...`. Then the SSH session closes ("closed by remote host") within
  about 2 minutes: the parked path reboots at once.
  - "A full update is already running": another run (UI or auto-updater)
    holds the lock. Go on with 4.5 and read `/data/fullupdate.log` after
    the reboot.
  - "Update staged ... panda state unconfirmed": the supervisor reboots as
    soon as it sees the car off-road. Watch
    `ssh $DEV 'tail -f /data/fullupdate_reboot.log'`.
  - "Update staged ... vehicle ignition is on" or "openpilot is onroad":
    the car is not off. Do not press cruise-main. **[RADEK: physical]**
    Turn the ignition off; the supervisor then reboots.
  - The script ends with an error before "HEAD is now at": run R0 in
    section 6 (return HEAD to `!my-fp-new`), then STOP.
- [ ] 4.5 Wait for the device, then for the build.

  ```sh
  until ssh -o BatchMode=yes -o ConnectTimeout=8 $DEV true 2>/dev/null; do sleep 20; done; date -u | tee "$P8/04_back_online.txt"
  ssh $DEV 'cat /VERSION; S=$(tmux ls -F "#{session_name}" 2>/dev/null | head -1); tmux capture-pane -p -t "$S" -S -40 | tail -n 20
    echo "build_procs $(pgrep -f "[s]cons|[b]uild.py" | wc -l)"; pgrep -f "[m]anager.py" >/dev/null && echo MANAGER_UP || echo MANAGER_DOWN'
  ```

  Run the second command every 5 minutes, and save each output as
  `04_build_<n>.txt`. PASS: `/VERSION` is still `12.8`; within 60 minutes
  of the reboot, `build_procs 0` and `MANAGER_UP`. Record the build time.
  An AGNOS flash in progress: never interrupt it. Let it finish, then go to
  R1. A build error on the screen or in the tmux output, or no
  `MANAGER_UP` after 60 minutes: go to R1 (R1b when `fullupdate.sh` cannot
  start).
- [ ] 4.6 Full-SHA check.

  ```sh
  ssh $DEV 'cd /data/openpilot && git rev-parse HEAD && git symbolic-ref --short HEAD && git rev-parse --abbrev-ref "@{u}" 2>&1; \
    git status --porcelain --untracked-files=no | wc -l' | tee "$P8/04_sha_check.txt"
  ```

  PASS: `HEAD` is the full `$CAND`; the branch is `!my-fp-1031`; 0 tracked
  changes. A mismatch: run `fullupdate.sh` once more (4.4, second command).
  A second mismatch: R1.

## 5. Verify before any drive (matrix "device" row)

### 5a. Ignition off

Start 5a when `MANAGER_UP` has been true for at least 3 minutes.

- [ ] 5a.1 Core processes run; the processes that must stay off are off.

  ```sh
  ssh $DEV 'for p in "[m]anager.py" "[s]elfdrive.pandad.pandad" "[f]rogpilot.frogpilot_process" "[s]ystem.hardware.hardwared" "[s]ystem.updated.updated"; do
      echo "$p $(pgrep -f "$p" | wc -l)"; done
    echo "ui $(pgrep -x ui | wc -l)"; echo "mapd $(pgrep -x mapd | wc -l)"
    for p in "[f]rogpilot.system.the_pond" "[f]rogpilot.system.frogpilot_telemetry" "[f]rogpilot.system.speed_limit_reader" "[s]ystem.loggerd.uploader"; do
      echo "$p $(pgrep -f "$p" | wc -l)"; done
    echo "port_8082 $(ss -ltn 2>/dev/null | grep -c ":8082 ")"' | tee "$P8/05a_processes.txt"
  ```

  PASS: every name in the first group and `ui` and `mapd` show 1 or more.
  The Pond, `frogpilot_telemetry`, `speed_limit_reader` and `uploader` show
  0 (`NoUploads` is 1), and `port_8082` is 0.
- [ ] 5a.2 No process restarts.

  ```sh
  snap() { ssh $DEV 'ps -eo pid,args | grep -E "manager\.py|selfdrive\.|system\.|frogpilot\.|\./(ui|mapd)( |$)" | grep -v -e "ps -eo" -e "grep -E" | sort -k2'; }
  snap > "$P8/05a_ps1.txt"; sleep 120; snap > "$P8/05a_ps2.txt"; diff "$P8/05a_ps1.txt" "$P8/05a_ps2.txt" && echo NO_RESTARTS
  ```

  PASS: `NO_RESTARTS`. A process that comes back with a new PID is crash
  looping: read its output in the tmux log (5a.3). If it does not stop, go
  to R1.
- [ ] 5a.3 No error log and no tracebacks.

  ```sh
  ssh $DEV 'ls -la /data/error_logs 2>&1; S=$(tmux ls -F "#{session_name}" 2>/dev/null | head -1)
    tmux capture-pane -p -t "$S" -S -3000 > /tmp/p8_tmux.txt; echo "tracebacks $(grep -c Traceback /tmp/p8_tmux.txt)"; cat /tmp/p8_tmux.txt; rm /tmp/p8_tmux.txt' \
    > "$P8/05a_logs.txt"; head -5 "$P8/05a_logs.txt"; grep tracebacks "$P8/05a_logs.txt"
  ```

  PASS: no `/data/error_logs/error.txt` (it turns off always-on lateral and
  shows the error screen) and `tracebacks 0`. Read each traceback and judge
  it; one from our code is a FAIL.
- [ ] 5a.4 Telemetry consent. First confirm it is unanswered, then
  **[RADEK: physical]** Radek answers on the device screen.

  ```sh
  ssh $DEV 'for k in FrogPilotTelemetry FrogPilotTelemetryConfirmed; do echo "$k=$(cat /data/params/d/$k 2>/dev/null || echo "<absent>")"; done'
  ```

  PASS before: `FrogPilotTelemetryConfirmed` is 0 or absent, and the screen
  shows "Help make FrogPilot better ..." with "No Thanks" and "Yes, Share
  Data". **[RADEK: physical]** Tap "No Thanks" (decision `DECISIONS.md` F).
  Run the command again. PASS after: `FrogPilotTelemetry=0`,
  `FrogPilotTelemetryConfirmed=1`, the home screen shows, and
  `frogpilot.system.frogpilot_telemetry` is still 0 in 5a.1.
- [ ] 5a.5 Settings are intact apart from the expected changes. Copy the
  live store and run appendix A2.

  ```sh
  mkdir -p "$P8/post/params_d" && ssh $DEV 'tar -C /data/params/d -cf - .' | tar -C "$P8/post/params_d" -xf -
  python3 "$P8/A2.py" "$P8/backup/params_d" "$P8/post/params_d" ~/.route_sync/work/fp1031/port-scratch/migrate_report_b1.json | tee "$P8/05a_params_compare.txt"
  python3 "$P8/A1.py" "$P8/post/params_d" > "$P8/05a_record_keys.txt"; diff "$P8/03_record_keys.txt" "$P8/05a_record_keys.txt"
  ```

  PASS: A2 exits 0 (no key missing outside the expected deletions). Judge
  every key under "CHANGED, JUDGE EACH" in `RECORD.md`: learned state,
  counters, timestamps and caches may change; a user setting must not. The
  A1 diff shows only the expected deletions (now `<absent>`) and the two
  rewrites. The lane-centering, button, upload, Experimental Mode and
  personality values are unchanged. A lost or changed setting: run
  R2-key for that key, unless it is not in the new registry (then
  `clear_all` deletes it again at the next start: STOP and report, because
  the code needs a registry row).
- [ ] 5a.6 Offroad alerts.

  ```sh
  ssh $DEV 'cd /data/params/d && for f in Offroad_*; do [ -e "$f" ] && echo "$f: $(cat "$f")"; done; true' | tee "$P8/05a_offroad_alerts.txt"
  ls "$P8/backup/params_d" | grep '^Offroad_' || true
  ```

  PASS: no alert, or only alerts that the backup also had. Judge each new
  one.
- [ ] 5a.7 Upload and logging flags unchanged.

  ```sh
  ssh $DEV 'ls -la /cache/use_HD /cache/use_konik /cache/on_backup 2>&1' | tee "$P8/05a_cache_flags.txt"
  tar -tvf "$P8/backup/frogpilot_files.tar" | grep -E 'cache/use_(HD|konik)|cache/on_backup' || echo "none in backup"
  ```

  PASS: the same flag files exist as in the backup. `NoUploads`,
  `DisableOnroadUploads`, `NoLogging`, `UseKonikServer`, `HigherBitrate` and
  `GsmMetered` are unchanged in the 5a.5 A1 diff.
- [ ] 5a.8 Updater and the safe-reboot path.

  ```sh
  ssh $DEV 'cd /data/openpilot && git rev-parse HEAD && git ls-remote origin "refs/heads/!my-fp-1031" | cut -f1
    for k in AutomaticUpdates UpdaterTargetBranch UpdateFailedCount; do echo "$k=$(cat /data/params/d/$k 2>/dev/null || echo "<absent>")"; done
    echo "fullupdate_safe_reboot $(grep -c __safe_reboot fullupdate.sh)"
    echo "manager_routing $(grep -c "routed to fullupdate.sh __safe_reboot" system/manager/manager.py)"
    stat -c "%U:%G %a %n" /data/ssh /data/ssh/id_ed25519; mountpoint /data/media' | tee "$P8/05a_updater.txt"
  ```

  PASS: `HEAD` equals the remote SHA, which equals `$CAND`, so the
  auto-updater has nothing to do. `AutomaticUpdates` is unchanged from 1.3.
  `UpdaterTargetBranch` is `!my-fp-1031` (or absent if `updated` has not run
  yet), and `UpdateFailedCount` is 0 or absent.
  `fullupdate_safe_reboot` is 2 or more and `manager_routing` is 1. The
  `/data/ssh` modes equal 3.1 (700 and 600), and `/data/media` is a
  mountpoint.
- [ ] 5a.9 The built model is the pinned tsfdo model.

  ```sh
  git -C "$REPO" show "$CAND:selfdrive/modeld/models/driving_supercombo.onnx" | shasum -a 256
  ssh $DEV 'cd /data/openpilot/selfdrive/modeld/models && sha256sum driving_supercombo.onnx && ls -la | grep -E "tinygrad|metadata"' | tee "$P8/05a_model.txt"
  ```

  PASS: the two SHA-256 values are equal, and the tinygrad driving and DM
  bundles exist with a time after the reboot.

### 5b. Ignition on, in Park (no drive)

- [ ] 5b.1 **[RADEK: physical]** At least 3 minutes after 5a.1 passed,
  turn the ignition on (READY). Stay in P, do not engage, do not drive. Stay
  like this for at least 4 minutes.
- [ ] 5b.2 Three minutes after the ignition is on: onroad processes run.

  ```sh
  ssh $DEV 'echo "IsOnroad=$(cat /data/params/d/IsOnroad 2>/dev/null)"; echo "FirmwareQueryDone=$(cat /data/params/d/FirmwareQueryDone 2>/dev/null)"
    ps -eo pid,etimes,args | grep -E "selfdrive\.(modeld\.(modeld|dmonitoringmodeld)|car\.card|controls\.(controlsd|radard|plannerd)|selfdrived\.selfdrived|locationd\.(locationd|calibrationd))|\./(loggerd|camerad|encoderd)( |$)" \
    | grep -v -e "ps -eo" -e "grep -E"' | tee "$P8/05b_processes.txt"
  ```

  PASS: `IsOnroad=1` and `FirmwareQueryDone=1`. Each listed process appears,
  and its age (`etimes`) grows between two runs of the command.
- [ ] 5b.3 **[RADEK: physical]** Look at the device and the car. The device
  shows the camera view with no red or orange alert. The cluster shows no
  ACC or cruise fault and no new warning light. Record what Radek sees. A
  cruise fault latched during start-up clears only with an ignition cycle
  (2026-09-05). If one appears: ignition off, wait 1 minute, then repeat
  5b.1 to 5b.3 once. A second fault: R1.
- [ ] 5b.4 Copy the first two segments of the park route after segment 2
  has started (about 2.5 minutes after the ignition is on).

  ```sh
  R0=$(ssh $DEV 'ls -td /data/media/0/realdata/*--0 | head -1'); echo "$R0" | tee "$P8/05b_route.txt"
  RB=${R0%--0}
  ssh $DEV "ls -d '$RB--2'" && for s in 0 1; do ssh $DEV "cat '$RB--$s/rlog.zst'" > "$P8/05b_rlog_$s.zst"; done; ls -la "$P8"/05b_rlog_*.zst
  ```

  PASS: `$RB--2` exists (so segments 0 and 1 are closed), and both files
  are larger than 1 MB. The route name is newer than the last route before
  the switch.
- [ ] 5b.5 Decode both segments on the host with the port's cereal
  (appendix A3).

  ```sh
  for s in 0 1; do PYTHONPATH=$PORT:$PORT/frogpilot/third_party python3 "$P8/A3.py" "$P8/05b_rlog_$s.zst" | tee "$P8/05b_log_$s.txt"; done
  ```

  PASS, both segments: `commit` is `$CAND`, `branch` is `!my-fp-1031` and
  `dirty False`. Fingerprint `HYUNDAI_SANTA_FE_HEV_2022`, brand `hyundai`,
  `opLong True` (not `mock`). `modelV2` is at least 1100 per segment (20 Hz
  model). Safety includes `hyundai`, with no faults. "not running but
  should" and "nonzero exit codes" are empty.
  Events on segment 1 (steady state) are only from `wrongGear`,
  `wrongCarMode`, `preEnableStandstill`, `seatbeltNotLatched`, `doorOpen`,
  `parkBrake`, `pedalPressed`, `gasPressedOverride`, `steerOverride` and
  `reverseGear`. Segment 0 may also show the start-up events
  `selfdriveInitializing`, `commIssue` and `commIssueAvgFreq`, and safety
  `elm327` then `hyundai`. Route 00002310 segment 0 on today's branch shows
  the same. Any other event or alert (for example `canError`, `radarFault`,
  `accFaulted`, `processNotRunning`, `carUnrecognized`, `controlsMismatch`,
  `startupNoCar`, `startupNoControl`) is a FAIL. Judge it, and go to R1 if
  it stays after one ignition cycle.
- [ ] 5b.6 **[RADEK: physical]** Turn the ignition off. One minute later:

  ```sh
  ssh $DEV 'echo "IsOnroad=$(cat /data/params/d/IsOnroad 2>/dev/null)"; echo "uptime_s $(cut -d" " -f1 /proc/uptime)"; pgrep -f "[m]anager.py" >/dev/null && echo MANAGER_UP'
  ```

  PASS: `IsOnroad=0`, the uptime continues (no reboot) and `MANAGER_UP`.
- [ ] 5b.7 Report to Radek: all boxes of 4, 5a and 5b with PASS, or the
  open FAILs. **[RADEK: go]** Radek decides on the first drive.

## 6. Rollback

### Decision rule

- Before the first drive: roll back the code (R1) on any FAIL that the box
  itself does not clear in this session. Examples: build error or more than
  60 minutes; boot loop; SHA mismatch after one retry; a crash-looping
  process; fingerprint not `HYUNDAI_SANTA_FE_HEV_2022` or `mock` after two
  ignition cycles; no model output; a blocking alert or a cluster fault after
  two ignition cycles; a traceback from our code. These are hard triggers:
  the operator runs R1 and then reports. Radek's only part is the parked car
  with the ignition off.
- Settings: a single lost or changed key gets R2-key. The full settings
  restore (R2) runs only after R1, when the old code runs again.
- After the first drive: a safety event (unexpected hard braking, a missed
  stop, a steering fault, a repeating cluster fault) means: Radek takes over
  and parks; then R1 and R2. A comfort difference is not a rollback trigger.
  Report it with a bookmark, and phase 9 compares the drive with last week.
  Radek decides any rollback after the first drive.

### R0. Before the reset only: return HEAD

Use this when `fullupdate.sh` in 4.4 failed before "HEAD is now at". The
working tree is still the old tree.

```sh
ssh $DEV 'cd /data/openpilot && git symbolic-ref HEAD "refs/heads/!my-fp-new" && git rev-parse HEAD && git status --porcelain --untracked-files=no | wc -l'
```

PASS: `HEAD` equals `DEVICE_HEAD` and 0 tracked changes.

### R1. Code rollback to `!my-fp-new`

Precondition **[RADEK: physical]**: the car is parked with the ignition
off.

```sh
ssh $DEV 'cd /data/openpilot && git symbolic-ref HEAD "refs/heads/!my-fp-new" && git rev-parse HEAD'
ssh -tt $DEV 'cd /data/openpilot && ./fullupdate.sh' 2>&1 | tee "$P8/06_r1_fullupdate.txt"
```

Expected: the same flow as 4.4, with `origin` `refs/heads/!my-fp-new` and
`HEAD is now at <short $ORIGIN_NEW>`, then a reboot and a full build of the
old tree. If the device's local `!my-fp-new` is older than `$ORIGIN_NEW`, the
running auto-updater can start the same update first. Its "already running"
message is harmless. Then repeat the 4.5 wait and the 4.6 check, with
`$ORIGIN_NEW` and branch `!my-fp-new`. Remove the local branch so a later
attempt starts clean:

```sh
ssh $DEV 'cd /data/openpilot && git branch -D "!my-fp-1031" && git rev-parse HEAD && git symbolic-ref --short HEAD'
```

**R1b, when `fullupdate.sh` cannot start.** It stops before the fetch when
the tree on the device does not build (`ensure_runtime_helpers`,
`fullupdate.sh:528`). Run its parked path by hand. Use it only with the
ignition off: a raw reboot with the ignition on faults the cluster.

```sh
ssh $DEV 'cd /data/openpilot && git symbolic-ref HEAD "refs/heads/!my-fp-new" && git fetch origin "refs/heads/!my-fp-new" \
  && git reset --hard FETCH_HEAD && git rev-parse HEAD && rm -f /data/openpilot/prebuilt && sudo reboot'
```

PASS for R1 and R1b: after the build, `HEAD` is the full `$ORIGIN_NEW`, the
branch is `!my-fp-new`, the manager is up, and 5a.1 to 5a.3 pass with the
old process list (The Pond and the sign reader do not exist there).

### R2. Full settings restore

Run after R1, with the old code built and the manager up. Stop openpilot
first.

```sh
ssh $DEV 'sudo systemctl stop comma; sleep 5; pgrep -f "[m]anager.py" >/dev/null && echo STILL_RUNNING || echo STOPPED'
ssh $DEV 'set -e; B=/data/phase8_backup
  D=$(readlink -f /data/params/d); sudo find "$D" -maxdepth 1 -type f ! -name .lock -delete; sudo tar -C "$D" --same-owner -xpf $B/params_d.tar
  if [ -f $B/cache_params_d.tar ]; then C=$(readlink -f /cache/params/d); sudo find "$C" -maxdepth 1 -type f ! -name .lock -delete; sudo tar -C "$C" --same-owner -xpf $B/cache_params_d.tar; fi
  cd /data/params/d && find . -type f ! -name .lock -exec sha256sum {} + | LC_ALL=C sort -k2' > "$P8/06_r2_restored.sha256"
diff "$P8/backup/params_d.sha256" "$P8/06_r2_restored.sha256" && echo RESTORE_PASS
ssh $DEV 'for s in /data/params /cache/params; do [ -e $s/d ] || continue; t=$(readlink -f $s/d); find "$t" -maxdepth 1 -type f -printf "%u:%g %m\n" | sort | uniq -c; done'
ssh $DEV 'sudo systemctl start comma'
```

PASS: `STOPPED`; `RESTORE_PASS`; the owner and mode counts equal 3.1; the
manager is up within the build time. At start, the old manager deletes its
CLEAR_ON_* keys again; this is expected. The files in
`frogpilot_files.tar` (toggle backups, CEM override, cache flags, update
logs) return from the backup only on request:
`tar -C / -xpf /data/phase8_backup/frogpilot_files.tar <path>` (paths as
listed by `tar -tvf`).

### R2-key. Restore one key

```sh
K=LaneCenterOffset   # the key to restore; repeat per key
ssh $DEV 'sudo systemctl stop comma; sleep 5; D=$(readlink -f /data/params/d); sudo tar -C "$D" --same-owner -xpf /data/phase8_backup/params_d.tar ./'"$K"' \
  && cat "$D/'"$K"'"; echo; sudo systemctl start comma'
```

PASS: the printed value equals the backup (`$P8/03_record_keys.txt`), and
5a.5 passes again after the manager start.

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

1. Sync the routes without pruning. Run the dry run first and check that the
   plan deletes nothing; the defaults prune (30 days, 30 GiB).

   ```sh
   cd "$REPO" && python3 tools/route_sync/refresh_routes.py --include-rlog --keep-days 3650 --max-cache-gib 4000 --max-protected-gib 4000 --dry-run
   cd "$REPO" && python3 tools/route_sync/refresh_routes.py --include-rlog --keep-days 3650 --max-cache-gib 4000 --max-protected-gib 4000
   ```

2. Run the drive report from a `!my-fp-1031` checkout. Two log schemas
   (`CLOSURE.md`): a drive recorded after the switch decodes correctly only
   with the new cereal, and a drive from before the switch only with the old
   one. Never mix both in one report run, and keep post-switch drives out of
   the sim corpus (`tools/stopping/sim/recorded.py` decodes with the old
   recording schema) until the tools choose the schema per log.

   ```sh
   ROUTE=00000000--0000000000   # replace with the first drive's route name from the sync output
   cd "$PORT" && PYTHONPATH=$PORT:$PORT/frogpilot/third_party python3 tools/stopping/drive_report.py --routes "$ROUTE"
   ```

   The report goes to `~/.route_sync/reports/drives/<route>.md`, with one
   line in `INDEX.md`. Compare it with last week's lines from the old
   branch (phase 9) and send Radek the summary.

## Appendix: host snippets

Save each block as the named file in `$P8` after box 0.2, and run it as the
boxes show. Each one was tested on 2026-11-02. A1, A2 and A4 ran on a copy of
today's store with the device's directory layout; A4 also failed as it must
when the clear step was left out. A3 ran on route 00002310 segments 0 and
1.

### A1.py: record keys (secrets masked)

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

### A3.py: one log segment (run with the port's PYTHONPATH)

```python
import collections, statistics, sys
from openpilot.tools.lib.logreader import LogReader
c = collections.Counter(); alerts, events, safety, gears, procs = set(), set(), set(), set(), {}
init = cp = None; mtimes = []; faults = set(); ign = set()
for m in LogReader(sys.argv[1]):
  w = m.which(); c[w] += 1
  if w == 'initData': init = m.initData
  elif w == 'carParams': cp = m.carParams
  elif w == 'modelV2': mtimes.append(m.modelV2.modelExecutionTime)
  elif w == 'selfdriveState' and m.selfdriveState.alertType: alerts.add(m.selfdriveState.alertType)
  elif w == 'onroadEvents': events.update(str(e.name) for e in m.onroadEvents)
  elif w == 'pandaStates':
    for p in m.pandaStates:
      safety.add(str(p.safetyModel)); ign.add((p.ignitionLine, p.ignitionCan)); faults.update(str(f) for f in p.faults)
  elif w == 'carState': gears.add(str(m.carState.gearShifter))
  elif w == 'managerState':
    for p in m.managerState.processes: procs[p.name] = (p.shouldBeRunning, p.running, p.exitCode)
print('commit', init.gitCommit if init else None, 'branch', init.gitBranch if init else None, 'dirty', init.dirty if init else None)
print('fingerprint', cp.carFingerprint if cp else None, 'brand', cp.brand if cp else None, 'opLong', cp.openpilotLongitudinalControl if cp else None,
      'source', cp.fingerprintSource if cp else None)
print('modelV2', c['modelV2'], 'median exec s', round(statistics.median(mtimes), 4) if mtimes else None)
print('alerts', sorted(alerts)); print('events', sorted(events))
print('safety', sorted(safety), 'ignition', sorted(ign), 'faults', sorted(faults)); print('gears', sorted(gears))
print('not running but should', sorted(n for n, (s, r, _) in procs.items() if s and not r))
print('nonzero exit codes', sorted((n, e) for n, (_, _, e) in procs.items() if e))
```

### A4.sh: restore rehearsal on the host (box 3.6)

```sh
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
