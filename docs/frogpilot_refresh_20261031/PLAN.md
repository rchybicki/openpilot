# FrogPilot Testing refresh plan (2026-10-31)

Goal. Move this fork onto the new FrogPilot Testing source of 2026-10-31. First
clean our own stack so we know each feature we carry. Then port the features that
must drive on day one, and trial the rest on the new base one at a time.

Status. Planning, revision 5 (four Sol xhigh plan reviews; rounds 1..3
REWORK, round 4 APPROVE WITH CHANGES; eighteen Act-on items, all accepted;
see Appendix A). The day-one rule and
the per-row dispositions live in `DECISIONS.md`. Nothing in history changes
until Radek approves the decision table (phase 3). Phases 1 and 2 are read-only.

## Facts (measured 2026-10-31)

- Target source: `437be91e24` "ZSS support" = `frogpilot/FrogPilot-Testing^`.
  Its tree is identical to `frogpilot/MAKE-PRS-HERE` `af621b82c2` (empty diff).
  The Testing tip `00256341ca` is "Compile FrogPilot" (sources stripped). Never
  base on it.
- Target is still openpilot 0.10.3 (OPGM v0.10.3, RealFast v0.10.3, then 193
  FrogAi feature commits rebuilt on 2026-10-28..31).
- `frogpilot/FrogPilot` and `FrogPilot-Staging` are on the 0.9.7 lineage
  (`common/version.h` 0.9.7). They are not our line.
- Our base: `d2db3fad4b` (old Testing source, 2026-05-25, 162 FrogPilot
  commits). Our stack on top: 253 Radek commits (243 non-merge, 10 merges),
  15 feature commits from the 2026-09-06 regroup, then about 228 cycle commits.
- Upstream delta `d2db3fad4b..437be91e24`: 1060 files, +239635 / -27919.
  Of 150 common subjects, 144 patches changed. 43 new subjects, 12 dropped.
- Dropped upstream that we carry: "long mpc: use model predicted lead
  trajectory", "Wire profile acceleration bounds into long MPC", "Traffic Mode
  v2" (`frogpilot_traffic.py` is gone), "slowdown_detection", "Update mapd to
  v2", "WMI model", "Use C3 AGNOS manifest", "Move to FrogPilot API", "Logging",
  "Fixes".
- The dropped MPC subjects are partly kept and reworked upstream, not gone.
  Model-predicted leads remain conditional; the solver minimum returns to
  `ACCEL_MIN`. Do not restore the two old commits wholesale.
- New upstream with overlap risk for us: Force Stop rewrite (owns hold in the
  planner), Custom driving personalities, Speed Limit Sign Reader (`.mnn`),
  Model Manager and Model Randomizer (default model Dark Souls, automatic
  downloads on), Soft Reboot (restarts without our SCC handoff), The Pond
  (always-run local HTTP service), reworked telemetry with consent, Adjacent
  Lanes. Increased Braking Force, Responsive Pedal, and Smooth Pedal on Hills
  are Honda or GM only.
- 139 files changed on both sides. Heaviest overlap: `longitudinal_planner.py`
  (ours 2393 lines, theirs 110), `longcontrol.py` (2060 / 12), `radard.py`
  (535 / 136), `long_mpc.py` (261 / 148), `modeld.py` (262 / 86), `card.py`,
  `controlsd.py`, CEM, SLC, CSC, `frogpilot_variables.py` (68 / 1699).
- `tinygrad_repo` upstream changed 1 line. Ours is a large vendored bump.
- 37 module-level flags in `selfdrive/controls/lib/stopping_flags.py` (31),
  `longcontrol.py` (3), and the Hyundai `carcontroller.py` (3). Four are False
  (`SANTA_FE_STOP_LINE`, `IDENTIFICATION_HOOK`,
  `STOPPING_CLOSE_GAP_CREEP_ENABLED`, `DYNAMIC_SCC14_JERK`). Three True flags are open car
  trials (`FAST_RELEASE_TO_LANES`, `STARTING_BLIP_GUARD`, `LAUNCH_FOLLOW`).
- The new base changes stored settings. `manager.py` runs `migrate_params`,
  syncs `/cache/params`, and writes defaults. It deletes incompatible
  `MaxLateralAcceleration` values from both stores. Some keys change type,
  for example `AggressivePersonalityProfile` BOOL to STRING and
  `PauseAOLOnBrake` BOOL to INT.
- The stopping sim cannot run on the new base today. `radar_replay.py:454`
  imports `FrogPilotTraffic` (gone upstream), the stopping service is not on
  the new base until ported, and `loader.build` refuses a tree whose compiled
  sources (capnp, params) differ from the working tree.
- The sim freezes SLC and CSC to logged `frogpilotPlan`
  (`radar_replay.py:398-418`), and it runs the Hyundai sender without Panda
  safety. It cannot validate SLC, CSC, cruise, or safety ports.
- `classify.py d2db3fad4b f77b26830f` prints `unmatched: 231`. The regroup
  builder asserts every commit is classified. The classifier needs new rules
  before phase 4.
- The worktree has uncommitted route_sync work (dated 2026-10-02) and an
  untracked `docs/stopping/HANDOVER_2026-09-26.md`. Not ours to change. The
  regroup needs a clean tree, so Radek decides: commit, stash, or exclude.

## Rules for the whole program

- Existing process: `docs/frogpilot_testing_refresh_process.md` and
  `skills/git-commit-cleanup/SKILL.md`. This plan follows them and adds the
  decision table, the validation matrix, and settings preservation.
- Backups first, pushed to origin. Never force-push `!my-fp-new` without
  Radek's go. The device runs `!my-fp-new` and applies each push it fetches.
- FrogPilot force-pushes. Pin the target as a fork-owned ref before work.
- No deploy, no device branch switch, and no push to `!my-fp-new` without
  Radek's explicit go.
- Each port batch ends in its validation-matrix rows. Do not start the next
  batch on a red one.
- Keep every retained flag as a switch through new-base qualification. Fold
  (delete the legacy path) only after the behavior passes on the new base.
- An expected difference must name its upstream cause and be judged. It is
  never accepted only because it is expected.

## Phase 0. Freeze and pin

- [x] Dirty tree resolved: route_sync work committed `32378eb858`, handover
      doc `fc284ba79e` (Radek: commit and carry).
- [x] `git branch 'backup/!my-fp-new_10-31_pre-fp1031-refresh' f77b26830f`
      and push it (on origin 2026-10-31).
- [x] Pin the target: push `437be91e24` to origin as
      `upstream/FrogPilot-Testing_2026-10-31_src` (on origin 2026-10-31).
- [x] History check: `d2db3fad4b..HEAD` and both FrogPilot ranges are
      complete locally; blame inside our range needs no unshallow.

## Phase 1. Inventory our stack (read-only)

Output: `docs/frogpilot_refresh_20261031/INVENTORY_OURS.md`.

- [ ] One row per feature unit (finer than the 15 regroup commits where a
      commit mixes features). Columns: feature, commits, files, flag and its
      state, live-since or trial status, evidence doc, upstream code it
      depends on, conflict risk on the new base, port disposition, switch
      disposition.
- [ ] Port disposition: `port-now` (must drive on day one), `test-first`
      (port later behind its own trial), `drop` (parked, reverted,
      superseded, temporary), `tools-docs` (no runtime effect).
- [ ] Switch disposition for each flag: `retain-switch` (keep through
      qualification) or `fold-later` (fold after it passes on the new base).
- [ ] List dead code: False flags, temporary test modes, shadow-only paths,
      superseded models and their runtime support.

## Phase 2. Map the upstream delta (read-only)

Output: `docs/frogpilot_refresh_20261031/UPSTREAM_DELTA.md`.

- [ ] Per area (longitudinal planner and MPC, CEM, SLC and mapd, CSC,
      following and personalities, acceleration and braking, Hyundai and
      opendbc and panda safety, modeld and Model Manager, UI, system and
      update, telemetry and network writes): what changed, what is new, what
      was removed, with commit ids and file pointers.
- [ ] For each of our features, what the new base changes under it.
- [ ] New outbound network writes upstream adds and their default state.
- [ ] Params changes: keys added, removed, retyped; migrations that delete or
      rewrite stored values.

## Phase 3. Decide (Radek)

- [x] Host writes `DECISIONS.md`: one row per feature with both
      dispositions and the reason. Radek approved revision 2 on 2026-10-31.
- [x] Host lists the upstream defaults that touch the car or the network
      (Increased Braking Force, personalities, telemetry, uploads) with a
      recommended value.

## Phase 4. Regroup the unchanged tree

- [x] Extend `skills/git-commit-cleanup/scripts/classify.py` `FEATURES`,
      `RULES`, `PATH_RULES` (and `BODIES` in `regroup_by_feature.py`) to the
      phase 1 feature units. Run until it prints `unmatched: 0`.
- [x] Build in a worktree from `d2db3fad4b`. Must print `TREE_IDENTICAL`.
- [x] Spot-check shared-file attribution (`longcontrol.py`,
      `longitudinal_planner.py`, `card.py`, `params_keys.h`, `AGENTS.md`).
- [x] Mechanical change: a fresh Sol high reviewer checks the claim and the
      classifier diff (PASS with nits).
- [ ] Tree identity proves the final tree only. It does not prove that a
      subset of commits works. Phase 6 proves the subset.

## Phase 5. Remove `drop` features on the regrouped branch

Phase 4 result (2026-10-31): `cleanup/fp1031-regroup` `7a8bbf13c1`, 28
commits (20 features, 8 `drop:`), tree identical to `6272ccdbaa`; all drop
commits revert cleanly. Residue that blame cannot split stays in kept commits
and is removed by hand here: H06 flag and if/else in the Hyundai
`carcontroller.py`; D01 band governor in `stopping_service.py` and the
`SANTA_FE_STOP_LINE` / `GOVERNOR_BAND_PROFILE` pair; D02 `_id_hook` names in
`longcontrol.py`, `frogpilot_card.py:84-85`, the `IDENTIFICATION_HOOK` flag,
test imports; D04 test imports and the unused planner import; D07 menu
residue. Host decisions: inline the hook constants the KCS extractors need
(`tools/stopping/review/kcs1_reps.py`), so the offline tools survive the D02
drop; selector and whole-approach tools leave with their drop commits (the
backup branch keeps them); keep the `wholeApproach*` capnp fields (log
schema compatibility).

Mechanical review (Sol high, 2026-10-31): PASS. Nits recorded: the CEM
commit carries `HumanFollowing` and the `traffic_mode_active=` planner
arguments, so CEM and following port together (both batch 6); after the
reverts, dropped-symbol references also remain in
`tools/stopping/drive_replay.py`, `drive_report.py`, `test_drive_report.py`
(historical-trial support, keep where they guard existence) and the H06
default assertion in `test_can_bounds_fork.py:314`; the port follows the
phase 6 batch order, not the commit order.

Phase 5 result (2026-11-01): `cleanup/fp1031-subtract` `72ebdab488`, eight
`remove:` commits (-9454 / +221 lines vs the regroup tip). Full sim at the
regroup tip and at the result: same verdict, INCOMPLETE on both (470 jobs
without rows, rlogs lost to the 2026-10-12 prune), 0 per-case row
differences (`~/.route_sync/work/fp1031/logs/cmp_rows_result.txt`);
per-commit test failures are a subset of the baseline's. Host decisions:
the `wholeApproach*` log fields keep their capnp defaults (no reader); the
sim gate from here on is "identical rows on every job that has a row, and
the same failed-job set", because the pruned rlogs make every verdict
INCOMPLETE; the build recipe adds `msgq_repo/msgq/visionipc`,
`selfdrive/pandad`, and `libsafety` (`scons -D` in
`opendbc_repo/opendbc/safety/tests`).

Phase 5 review (2026-11-01): Sol xhigh DO NOT SHIP and Opus high SHIP WITH
CHANGES, one shared Act-on: D07 removed eight PERSISTENT registry rows, and
`Params::clearAll` at manager start would have deleted their stored values.
Fix `c022a9890c` restores the rows (8 of 8 survive `clear_all` in a probe)
and drops D02/D05 leftovers. Fix-round review (Opus high): SHIP. Phase 5
tip: `c022a9890c`.

- [x] One commit per removed feature. Retained switches stay.
- [ ] Per commit: focused pytest, and the full sim verdict (`run_gates.py`
      without `--quick`) at that commit equals the verdict at the regroup
      tip. Any change stops the commit.
- [x] Review: ordinary tier (Sol xhigh + Opus high) on the whole removal; fix round 1 SHIP.

## Phase 6. Port onto the new base

Branch `!my-fp-1031` from `437be91e24`.

- [ ] Write the dependency closure of the `port-now` set: which feature
      commits must come together (shared files, imports, params, capnp).
      Port the closure, not single commits.
- [ ] Batch order:
  1. Base compat, schemas, params registry (`common/params_keys.h`).
  2. Sim and replay tooling (`tools/stopping/sim`, `tools/longitudinal`)
     prepared for the new base: files, build, imports. No verdict yet; the
     sim needs `stopping_flags.py`, `stopping_service.py`, and
     `frogpilot_traffic.py`, which arrive with batch 5 and 6.
  3. tinygrad bump and the driving model runtime. Check upstream Model
     Manager first; keep the current model.
  4. Hyundai, opendbc, panda safety, card.
  5. Stopping service and longitudinal (planner, longcontrol, MPC, radard).
     Keep our MPC lower bound and today's cruise maximum and profile values.
     Do not restore the two dropped MPC commits wholesale.
  6. Force coast, following with Traffic Mode v2, lane-change surrogate, our
     CEM and CSC carried whole, cruise and SLC, lateral learner gates, and the
     sign-reader process gate. The first sim verdict runs at the end of this
     batch, when the stopping closure is complete.
  7. System hardening, NVMe guard, update (`fullupdate.sh`, live SCC
     handoff, settings safe reboot). Intercept every ignition-on restart
     request (`DoReboot`, `DoSoftReboot`, Pond reboot and soft-reboot
     endpoints) before `manager_cleanup` and route it to `__safe_reboot`.
  8. UI.
  9. Remaining tools and docs.
- [ ] Per batch: resolve toward the new upstream shape, then reapply only our
      behavior. Run its validation-matrix rows.

## Validation matrix

Sim comparisons run in two separately built checkouts (worktrees), baseline
`!my-fp-new` and candidate `!my-fp-1031`. Each checkout builds
`scons -j8 selfdrive/ui/ test-dependencies selfdrive/controls/lib/longitudinal_mpc_lib selfdrive/controls/lib/lateral_mpc_lib msgq_repo/msgq/visionipc selfdrive/pandad`, plus `scons -D` in `opendbc_repo/opendbc/safety/tests` for `libsafety`
(`test-dependencies` alone does not build the acados solvers, `loader.py:373`).
A `mkwt.sh` worktree removes its solver links before this build. Each run
records the commit, a nonempty solver identity, and the corpus manifest.
Rows marked NEW are tests we must write in that batch (path and cases named);
they are work items, not existing checks. Compare
per-case outputs across the two runs; a full verdict is required, and every
adversarial failure is listed and judged.

| Batch | Check | Pass |
|---|---|---|
| all | `scons -j8 selfdrive/ui/ test-dependencies`; `git diff --check` | builds, no errors |
| 1 | params registry test; `migrate_params` on a copy of the device params store | every key we use exists with the type our code reads; no stored value of ours is lost |
| 2 | candidate builds with the sim files present | builds; no verdict expected |
| 3 | NEW `selfdrive/modeld/tests/test_model_load.py`: compile and load the driving model with the ported runtime, check output shapes against the metadata | passes on the Mac |
| 4 | Hyundai opendbc tests (`opendbc_repo/opendbc/car/hyundai/tests`); safety library built by `opendbc_repo/opendbc/safety/tests/test.sh` (scons -D), then `test_hyundai.py` and `test_hyundai_canfd.py`; card tests | all pass |
| 5 | stopping and longcontrol pytest; NEW test that the planner and MPC limits equal today's (`accel_clip[0]` lower bound, `CRUISE_MAX_ACCEL` 2.0, `A_CRUISE_MAX_VALS` `[2.0, 1.6, 0.8, 0.6]`) | all pass |
| 6 | full sim verdict, candidate vs baseline; `selfdrive/controls/tests/test_curve_speed_controller.py`, `selfdrive/car/tests/test_cruise_speed.py`; NEW `frogpilot/controls/tests/test_speed_limit_controller.py` for each SLC cap and gas-override rule; NEW CEM case test (54 km/h, SLC 30 km/h gives SPEED); NEW `torqued` and `lagd` gate tests (lat 0.5, long 1.5 rejected; `MIN_VEGO` 50 mph); NEW radard test that `leadOne` and `leadTwo` match baseline on lane-change cases; NEW process test that SLC runs while `speed_limit_reader` stays stopped | verdict equal, or each difference explained and judged; all tests pass |
| 7 | `selfdrive/car/tests/test_live_update_handoff.py`; NVMe guard tests; NEW test that the settings Reboot button with ignition on calls `fullupdate.sh __safe_reboot` and not `DoReboot`; NEW tests that Soft Reboot and both Pond reboot endpoints with ignition on reach `__safe_reboot` before `manager_cleanup`, and that card and pandad keep running while the handoff is pending or fails; NEW host harness for `fullupdate.sh` that stubs git fetch, reset, submodule, Params, and reboot, and runs old script against new tree | all pass |
| 8 | UI build; screenshots of changed screens | builds; screens render |
| device | phase 8 only, after settings backup and restore rehearsal: off-road boot, manager starts, model runs, firmware query, fingerprint, alerts, updater | no errors in logs |

## Phase 7. Verify the branch

- [ ] `git range-diff` regrouped stack vs ported stack. Every hunk lost or
      changed is explained.
- [ ] All host matrix rows green at the branch head. The device row runs in
      phase 8, never before the settings backup.
- [ ] Review: important tier. Triggers present: the new base rewrites and
      deletes stored settings (`migrate_params`), and upstream may add
      external writes (telemetry). Astra xhigh + Opus high.

## Phase 8. Device move (Radek's go)

- [ ] Before the switch, write down: candidate full SHA pushed to origin; the
      device branch (`!my-fp-1031`, tracking `origin`) and the refspec `fullupdate.sh` fetches
      (`fullupdate.sh:538-555` resets to `FETCH_HEAD`); exact switch command;
      exact rollback command; full-SHA check after reboot.
- [ ] Settings backup: copy `/data/params/d`, `/cache/params`, and the
      FrogPilot settings files to the host. Write and test the restore
      command before the switch.
- [ ] Updater compatibility: the first deploy runs the OLD `fullupdate.sh`
      across the NEW tree. The batch 7 host harness covers it. `fullupdate.sh`
      has no dry-run mode; never call the live script as a dry run.
- [ ] Then run the device row of the matrix.
- [ ] First transition ignition off and off-road. Verify boot, model,
      fingerprint, settings, and updater before any drive.
- [ ] Rollback: code (backup branch) and settings (restore command), both
      tested.

## Phase 9. First drive and trials

- [ ] Drive with the per-drive rules; drive_report; compare to the last week
      on the old branch.
- [ ] Test-first items: one feature per trial, behind a flag, same gates as
      today's cycles.
- [ ] Fold-later flags: fold after their behavior passes on the new base.

## Decisions taken (Radek, 2026-10-31)

1. Dirty route_sync work: commit and carry.
2. Decision table approved (`DECISIONS.md` revision 2).
3. Device move: switch the device branch to `!my-fp-1031`; `!my-fp-new`
   stays untouched for rollback. No force-push.

## Appendix A. Plan review record

Round 1, Sol xhigh, verdict REWORK. All six Act-on items accepted.

1. Classifier has 231 unmatched commits. Fixed in phase 4.
2. `fold` removed retained features from the port. Split into port and
   switch dispositions; folding moves after qualification.
3. Cross-base sim not executable. Tooling moves to batch 2; two built
   checkouts; recorded identities.
4. Sim parity does not cover SLC, CSC, cruise, safety, handoff, device.
   Validation matrix added.
5. Git backup does not roll back settings; important tier missed the params
   migration. Settings backup and restore added; phase 7 is important tier.
6. Device move lacked a procedure. Phase 8 procedure added.

Round 2, Sol xhigh, verdict REWORK. Round-1 items 1, 2, 6 closed; 3, 4, 5
partly. Four new Act-on items, all accepted.

7. Build recipe missed the acados solvers. MPC lib targets added.
8. Rows named tests that do not exist (modeld, SLC, safe reboot). Marked NEW
   with paths and cases; CSC suite named.
9. Phase 7 needed a device boot before the settings backup. Device row moved
   to phase 8 after the backup.
10. `fullupdate.sh` has no dry path. Host harness with stubs added.

Round 3, Sol xhigh, verdict REWORK. Round-2 items 7..10 closed. Seven new
Act-on items in `DECISIONS.md`, all accepted.

11. Port-now excluded the fold-later stopping features. All S rows port.
12. Test-first rows ran upstream CEM and lateral learners on day one. Day-one
    rule made strict; our modules port; upstream replacements are trials.
13. F03 contradicted the radard work item. Our surrogate ports.
14. Sign reader had no off gate while SLC runs. New gate and test.
15. Batch 2 needed a sim verdict before its runtime existed. Verdict moved to
    the end of batch 6.
16. Traffic Mode v2 had no owner. Carried in F02.
17. New cruise maximum had no disposition. Today's limits kept, with a test.

Round 4, Sol xhigh, verdict APPROVE WITH CHANGES. Items 11..17 closed.

18. Soft Reboot and Pond reboot paths were not in batch 7 or its tests, and
    the final handler runs after `manager_cleanup`. Batch 7 intercepts before
    cleanup; tests added. Nit: `forceDecel` wording corrected.

Sequencing changed per the review: regroup the unchanged tree first, then
remove `drop` features, port with switches, fold last.
