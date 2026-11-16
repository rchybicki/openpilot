# FrogPilot Testing Refresh Process

Use this process to move this fork onto a newer `FrogPilot-Testing` source
while the car keeps its behaviour.

The rule of every refresh: **on day one the car drives, looks and behaves as
it does today.** A change of behaviour is a separate, later trial.

This document is the process. The records of the last refresh are the worked
example:

| File in `docs/frogpilot_refresh_20261031/` | What it holds |
|---|---|
| `PLAN.md` | The phases with their check boxes and the validation matrix |
| `INVENTORY_OURS.md` | Our features as units (100 rows), with files and tests |
| `UPSTREAM_DELTA.md` | What the new base changed, by subsystem |
| `DECISIONS.md` | Per feature: port now, test first, fold later, drop |
| `CLOSURE.md` | What each port batch needed, review results, parity fixes |
| `PHASE8_PROCEDURE.md` | The reviewed procedure for the device move and rollback |
| `RUN_20261114.md`, `RUN_20261115.md` | The two device runs (failed and rolled back; successful) |
| `PARITY_AUDIT_BRIEF.md` | The brief of the audit after the first drive |

## History

| Refresh | Old base | New base | Result |
|---|---|---|---|
| 2026-05-31 | older Testing | Testing of 2026-05-31 | Cherry-pick of a cleaned stack |
| 2026-10-31 | `d2db3fad4b` | `437be91e24` (openpilot 0.10.3) | Full process below. 15 days. Car moved 2026-11-15 |

## Choose the size first

Do phases 0 to 2 in every refresh. Phase 0 makes backups, pins refs and syncs
logs. Phases 1 and 2 are read-only. Then choose:

- **Small refresh.** Upstream changed no module that we carry, no log schema,
  no msgq layout, no Params registry rule and no device binary. Use the short
  path at the end of this document.
- **Large refresh.** Any of those changed. Use all phases. The 2026-10-31
  refresh was large: upstream rewrote CEM, CSC and Traffic Mode, renumbered
  the FrogPilot log structs, raised the msgq reader slots from 15 to 20,
  changed Params definitions and migrations, and replaced the mapd binary.

## Ground rules

- Backups first, pushed to `origin`. Never force-push the device branch.
- FrogPilot force-pushes its branches. Fetch with a forced refspec and pin
  the target on `origin` before any work.
- Quote branch names that contain `!`. Write `"${rev}:path"` with braces.
- The device applies every push to its branch within about 5 minutes
  (`AutomaticUpdates`). Parked, it reboots at once. On the road, it waits for
  cruise off. So a new branch name is the safe place for the port: nothing
  reaches the car until the device branch is switched.
- Work in a git worktree outside the main checkout (the last port used
  `~/.route_sync/work/fp1031/port-wt`). One writer at a time in a worktree.
- Never run `fullupdate.sh` on a host. It has no dry-run mode.
- Never run automatic code formatting. Register every Params key in
  `common/params_keys.h`.
- No device access by delegates. On-road, only passive reads of files. No
  diagnostic `SubMaster` readers on the device.
- Never print or copy the settings backups. They contain secrets.
- An expected difference names its upstream cause and is judged. It is never
  accepted only because it is expected.

## Phase 0. Freeze and pin

```sh
git status --short
git fetch origin
git fetch frogpilot '+refs/heads/*:refs/remotes/frogpilot/*' --prune
git show -s --format='%h %ci %s' frogpilot/FrogPilot-Testing frogpilot/FrogPilot-Testing^
```

- Resolve a dirty tree first. A backup branch protects commits only.
- **The source target is not the branch tip.** The tip of `FrogPilot-Testing`
  is "Compile FrogPilot": compiled output with the sources stripped. The
  source is `frogpilot/FrogPilot-Testing^`. Check that its tree equals another
  FrogPilot source ref (`MAKE-PRS-HERE` was identical).
- **Our base is not `git merge-base`.** FrogPilot rebases. Our base is the old
  Testing source commit below our first own commit
  (`skills/git-commit-cleanup` explains how to find it).
- Push a backup branch of the device branch and a pin of the target:

```sh
git branch 'backup/!<device-branch>_<MM-DD>_pre-refresh' HEAD
git push origin 'backup/!<device-branch>_<MM-DD>_pre-refresh'
git push origin '<target-sha>:refs/heads/upstream/FrogPilot-Testing_<date>_src'
```

- **Sync the car's logs to the Mac now, and again just before the device
  move** (`docs/route_refresh_process.md`; always with `--keep-days 3650
  --max-cache-gib 4000 --max-protected-gib 4000`, in parts of about 1,500
  files with `--max-downloads`). The last weeks on the old branch are the
  baseline for every later comparison, and the car deletes its oldest logs
  when its disk is full. In 2026 this was forgotten for three weeks and
  recovered only after the move.

## Phase 1. Inventory our stack (read-only)

List every feature we carry as a unit: what it does, its files, its Params,
its log fields, its tests, and whether the car uses it. `git diff <old-base>
<head>` is the source. The result is `INVENTORY_OURS.md`. Mark dead features
(experiments that are off for good) for removal.

## Phase 2. Map the upstream delta (read-only)

Compare old base and new base by subsystem. The result is `UPSTREAM_DELTA.md`.
Look for these on purpose; each one hurt in 2026:

| Check | Why |
|---|---|
| `cereal/custom.capnp` ordinals of the FrogPilot structs | A renumbered struct makes an old log decode without error and with wrong values |
| msgq `NUM_READERS` and layout (`msgq_repo/msgq/msgq.h`) | A prebuilt binary with the old layout crashes every msgq process |
| Prebuilt binaries in the tree (mapd; anything not built by scons) | The Mac build does not exercise them. Find their source and version |
| On-disk formats (offline map tiles, model bundles, caches) | A new binary can reject or misread old data |
| Params registry: file, defaults, `migrate_params`, `Params::clearAll` | Stored settings can be rewritten or deleted at the first boot |
| New processes and their start conditions (`system/manager/process_config.py`) | New base features start by default (sign reader, The Pond) |
| New network paths (telemetry, registration, downloads) | Privacy decisions, and downloads that overwrite our data |
| New restart paths (`DoReboot`, `DoSoftReboot`, web endpoints, UI buttons) | A restart with the ignition on faults the cluster without the SCC handoff |
| Modules upstream rewrote that we changed (CEM, CSC, Traffic Mode, planner) | Decide: carry ours whole, or take theirs as a later trial |
| Anything that hooks or patches the system at UI start | The 2026 screen recorder patched the Qualcomm display library at every start |
| Default values of settings the car stores, and new settings | Stored values win over defaults, but new keys start at the default |
| AGNOS version of both trees | A version change means a flash at the first boot |

## Phase 3. Decide (the owner)

Write `DECISIONS.md` and get it approved before history changes. Each inventory
row gets one of:

- **Port now.** Our behaviour on day one. All rows that drive the car.
- **Test first.** Upstream has a replacement. Trial it later, one feature per
  trial, never on day one.
- **Fold later.** Port with its switch; delete the old path after it passes on
  the new base.
- **Drop.** Dead. Remove before the port.

Also decide every new upstream feature: on or off on day one, and how it is
kept off (stored setting, a gate in code, or removed). Where upstream rewrote
a module we depend on, carry our module whole and adapt only its interfaces.

## Phase 4. Regroup the unchanged tree

Rebuild our history as one commit per feature on a cleanup branch, with the
same final tree. Use `skills/git-commit-cleanup` (`classify.py`,
`regroup_by_feature.py`).

- The rebuild attributes every line by blame. `SPLITS` and `LINE_RULES` split
  coarse commits that blame cannot split.
- Features to drop go last as `drop_*` units, so their removal is a revert.
- The rebuild must print `TREE_IDENTICAL`. Intermediate commits do not have
  to compile.
- Review: mechanical (the claim is "same tree").

## Phase 5. Remove the dropped features

One commit per removed feature on a second cleanup branch. Then prove that
nothing else changed: the tests, and the stopping simulation with identical
rows at the regroup tip and at the result.

**`Params::clearAll` deletes stored keys that the registry lacks.** It runs at
every manager start. Removing a registry row deletes the car's stored value.
Keep every PERSISTENT row the car uses.

## Phase 6. Port onto the new base, in batches

Branch the port from the pinned target. Port from the final state of the
cleanup branch, not commit by commit. For each file:

- Upstream did not change it (old base equals new base): take ours.
- Only we have it: carry it whole.
- Otherwise: a 3-way merge (`git merge-file`, base = old base, ours = the
  cleanup tip) onto the new base. Resolve toward the new upstream shape and
  reapply our behaviour.

Write the dependency closure first: which features must come together (shared
files, imports, Params, capnp). Port closures, not single commits.

Batch order of 2026 (one commit per batch, stop on a red batch):

1. Base compatibility, schemas, Params registry.
2. Simulation and replay tools prepared for the new base.
3. tinygrad and the driving model runtime. Keep the current model.
4. Hyundai, opendbc, panda safety, card.
5. Stopping service and longitudinal control.
6. Force coast, following, lane change, CEM, CSC, cruise and SLC, lateral.
7. System hardening, NVMe guard, update path, safe reboot.
8. UI.
9. Remaining tools and docs.

Then a batch for each device binary and its data (2026: mapd, see below).

## Phase 7. Verify the branch

Run the validation matrix of `PLAN.md` at the branch head. The proofs that
carried the 2026 port:

- **Equivalence replay.** `tools/stopping/port_check` replays a logged route
  through radard, the planner, LongControl, the FrogPilot planner, controlsd
  and card in the old and the new tree. The comparison is exact, and a
  mutation proves each stage can see a change. Run it after every batch that
  touches driving.
- **Simulation verdict.** `tools/stopping/sim`: identical rows on every job
  that has a row.
- **Updater harness.** `tools/fullupdate/harness.py` runs `fullupdate.sh` with
  stubbed git, sudo and reboot. It covers the first update on the new tree.
- **Settings probe.** Run `migrate_params` and `clearAll` on a copy of the
  car's store and list every key that is deleted, added or rewritten.
- **Old logs.** Tools that read drives must choose the schema per log
  (`tools/stopping/sim/recorded.py` decodes with the recording schema). Keep a
  worktree of the old branch for old-log decoding.

Review: important tier (the new base rewrites and deletes stored settings).

**Run the parity audit of phase 9 here too, before the device move.** In 2026
it ran only after the first drive and listed 39 items that the replay, the
tests and two reviews had not seen. They were in UI and system paths (display,
settings texts, the log cleaner, start-up code) and in driving configurations
that the replay did not exercise, because the car's stored settings hide them.
The audit needs no device.

**Host tests do not cover everything.** The Mac scons build does not exercise
prebuilt device binaries: test them in an ARM container where possible, then
verify them on the device. The Qualcomm display path, AGNOS, the real settings
store and the real CAN bus are tested first on the device. Plan the first boot
as a test with a proven rollback.

### Device binaries: mapd in 2026

The first device boot of 2026-11-14 crash-looped. The port carried our old
mapd binary, built for 15 msgq reader slots. The new base has 20. mapd
panicked and the msgq processes asserted. The rollback worked.

The fix and its rules:

- Rebase our mapd changes onto the mapd version that FrogPilot uses. Source:
  `github.com/rchybicki/pfeifer_mapd`, branch `codex/pfeifer-v240-custom`
  (mapd v2.4.0 plus our Poland way-ID overrides and practical speeds).
- The offline tile format changed with it. **Regenerate the tiles and give
  each binary its own tile directory** (old `/data/media/0/osm`, new
  `/data/media/0/osm-v240`). No rename at switch or rollback. A binary that
  finds no tiles runs without map limits, which is a safe state.
- Remove upstream's automatic tile download: it overwrites our tiles with
  tiles that have no overrides.
- Prove the binary against the new base before the device: it must read
  `NUM_READERS` from the tree's `msgq.h`, and it must run in an arm64
  container against a 20-slot queue.

## Phase 8. Device move

Write the procedure from `PHASE8_PROCEDURE.md` (revision 12 is approved and
device-proven; update the SHAs, the branch names and the checks that the new
base changes). It is a mission-critical plan: Astra reviews it.

What the procedure does:

1. Preconditions: car parked, ignition off, device reachable, NVMe healthy,
   rollback SHA complete on the device without network.
2. Push the reviewed head to the new branch name. Fetch it on the device.
3. Stage device data in a new directory (2026: map tiles, about 2 GB).
4. Settings backup on the device and the host, and a restore rehearsal.
5. Switch by hand, not with `fullupdate.sh`: stop the unit, require its cgroup
   empty, hold both update locks, invalidate the staged overlay,
   `git reset --hard` to the local SHA, delete `prebuilt`, reboot.
6. Checks with the ignition off: SHA, processes, no restarts, no tracebacks,
   binaries, settings compared with the backup.
7. Checks with the ignition on in Park, then the first drive.
8. Rollback: the same switch to the old SHA, then the settings restore.

Run it block by block with `tools/fullupdate/p8run.zsh`, which executes the
marked blocks of the committed procedure and keeps the session variables. Keep
a `RECORD.md` per run. Rehearse on the host first
(`tools/fullupdate/phase8_rehearse.py`).

Lessons of the twelve procedure revisions:

- **Prefer STOP-and-report to automated repair.** Every recovery mechanism
  that was added (tile swap with markers, lock cleanup, device-to-host tile
  backup) became new review surface and caused new defects. After two review
  rounds in a row with self-caused defects, the fix was subtraction: go back
  to the last device-proven revision and add only read-only or create-only
  steps.
- **Never rename or delete old data in the move.** Create new directories
  only. The old tree must find its data untouched after a rollback.
- **A process check must not match itself.** The first run stopped because
  `pgrep` matched the script's own command line.
- **The car's disk is always at the cleaner's limit** (10 %). A free-space
  gate cannot pass. When the gate stops the staging, report to the owner and
  get his decision before a copy that makes the cleaner delete logs (2026:
  "Copy, cleaner deletes"). So sync
  the logs first (phase 0).
- **Known text defects of revision 12**, to fix before the next use: the free
  space gate of box 2a.2; box 5a.8 expects two `fullupdate_safe_reboot` hits
  where the file has one; box 5a.9 expects new model bundles although scons
  reuses unchanged ones; A2 reports `DongleId` missing, which is correct on a
  device without registration keys; the `REVIEWED` default is stale.
- The first build on the device takes 5 to 16 minutes. A boot without
  `prebuilt` runs scons every time. A change of `cereal/custom.capnp`
  recompiles everything.

The owner's part: the go for the slot, the car parked with the ignition off
for the time the procedure states (revision 12: about 2.5 hours, plus 5 to 50
minutes for the tile staging; the run of 2026-11-15 took about 50 minutes to
the first short drive), "No Thanks" on the telemetry page, and the ignition on
in Park only when section 5b asks.

## Phase 9. First day on the new base

1. **Review the first real drive** before any push: identity, fingerprint,
   model rate, panda faults, process restarts, events, and the stops
   (`tools/stopping/drive_report.py`, run from the port's tree). Compare
   doubtful events with the end of a drive on the old code.
2. **Ask the driver what he noticed.** In 2026 he found in one drive what 100
   commits of review had missed: the speed number no longer turned red when
   the car braked.
3. **Run the parity audit** (`PARITY_AUDIT_BRIEF.md`): three read-only
   research lanes compare `old-base..old-head` with `new-base..port`, one each
   for car, UI and settings; controls, model and location; system, update and
   tools. The method that finds the gaps: **follow each value from its
   producer to its consumer.** "The file was ported" proves nothing. The red
   speed was a ported reader whose producer line upstream had moved to
   another message. The audit of 2026 listed 17 + 13 + 9 items; about half
   needed a fix.
4. **Fix in batches by risk**, each reviewed at its tier: what the driver
   sees; system code (log deleter, storage guard, identity: important tier);
   latent differences that the stored settings hide today.
5. **Check the commits made on the old branch after the port branched.** Port
   each one.
6. **Decide each new-base feature that is still on** and was never judged
   (2026: Instant Replay off, screen recorder removed).

After the first reviewed drive, reviewed fixes may be pushed at any time. Do
not push documents only while the car drives: each push restarts the car's
software at the next stop.

## Phase 10. Close

- Update `CLOSURE.md` and `PLAN.md` with the results. Copy the run records
  into the records folder.
- Compare a week of drives with the last weeks on the old branch.
- Keep the old branch and the old data until the new branch is kept. Then:
  delete the old tile directory on the device, delete the settings backups on
  the device and the host (they contain secrets), move the main checkout to
  the new branch, and settle the scratch directories.
- Start the test-first trials, one at a time.
- Update this document with what was new.

## Reviews

| What | Tier |
|---|---|
| The plan and the decision table | Plan review |
| Regroup (same tree) | Mechanical |
| Removal of dropped features | Ordinary |
| The port (rewrites and deletes stored settings) | Important |
| The device procedure | Mission-critical plan |
| A device binary and its data scripts | Important |
| Parity fixes | By content: UI ordinary, deleter and guard important |

Reviewers reproduce or drop. A defect that exists unchanged on the old branch
is a follow-up, not a blocker.

## Traps, in one list

- The Testing tip is compiled output. The source is its parent.
- `git merge-base` is not our base.
- An old log decodes on the new schema without an error and with wrong
  FrogPilot values. The same happens live on the first update: old processes
  still run when the new updater reads their messages.
- `Params::clearAll` deletes every stored key that the registry lacks.
- The Mac build does not test a prebuilt device binary. Prove it in an ARM
  container, then on the device.
- A new binary needs its own data directory.
- Upstream downloads (map tiles, models) overwrite our data unless removed or
  switched off.
- Every new restart path must reach `fullupdate.sh __safe_reboot` before
  `manager_cleanup`.
- A reader can be ported while its producer is gone. Follow values end to end.
- Stored settings hide changed defaults. A fresh settings store would behave
  differently; list those keys.
- The car applies every push. Use a new branch name for the port.
- The car deletes old logs when its disk is full. Sync before the move.

## Short path for a small refresh

Use it only when phase 2 found none of the breaking changes above.

1. Phase 0 (backup, pin, sync).
2. Clean the local stack with `skills/git-commit-cleanup` on a cleanup branch.
   Verify with `git range-diff` against the backup.
3. Branch from the pinned target. Cherry-pick the cleaned commits one logical
   batch at a time. Resolve toward the upstream shape and reapply our
   behaviour narrowly. For model assets use `skills/frogpilot-model-import`.
4. Verify: `git range-diff <old-base>..<cleanup> <target>..HEAD`, focused
   tests, `scons -j8 selfdrive/ui/`, the equivalence replay and the updater
   harness.
5. Review at the tier of the content. Then the device move: with no change
   of a binary, a data format or the settings registry, the normal update
   path applies (push to the device branch; the car updates itself). Any such
   change needs phase 8.

## Device deployment

Follow the device update workflow in `AGENTS.md`. A branch switch or a
rollback always needs the owner's explicit go.
