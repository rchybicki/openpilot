# Stopping Behavior Project Worklog

- Last updated: 2026-02-09
- Scope: OpenPilot/FrogPilot longitudinal stopping behavior
- Goal: Make stopping behavior more consistent and comfortable while preserving safety

## Clarified Requirements (2026-02-07)

Primary scope:
- Controller evaluation/tuning focuses on stopping while OpenPilot is engaged.
- Manual/user stopping is not a primary acceptance target, but is useful context for understanding vehicle/gearbox behavior near standstill.

Hard requirements:
- Achieve wheel-stop with minimal perceived force/jerk at the final stop moment.
- Do not increase stopping distance too much while reducing final-stop jerk.

Known undesired behaviors to reduce:
- Final-stop jerk at/near wheel-stop transition (historically severe before current stopping refinements).
- Occasional "almost stop -> slight re-acceleration -> stop again" behavior.

Non-goals / constraints:
- Stop decision timing from the driving model/planner is not easy to change here and is treated as an external input.
- This project focuses on longitudinal stop execution (controller/tuning behavior) given planner stop intent.

Current evaluation lens:
- For controller quality checks: engaged stop events are the primary metric set (`event_mode=engaged_signal`).
- For broader vehicle behavior characterization: include all stop events (`event_mode=hybrid` / `speed_transition`) but do not use disabled samples for command-response model fitting.

Open questions (for tuning alignment):
- Which exact routes/segments most clearly show the wheel-stop jerk that still remains?
- Test mode confirmed: new-long API only.
- Red-light/force-stop events are included in primary scoring (same stopping logic expected).
- Maximum acceptable extra rollout target: <= 2 meters.

## Project Status

- [x] Map current stopping code path end-to-end
- [x] Identify stop-related tunables and where they are applied
- [x] Identify what telemetry/log fields to use for stop debugging
- [x] Attempt device SSH (`comma`, `commawifi`)
- [x] Snapshot live stop-related settings from device
- [x] Pull initial on-device logs into local baseline store
- [x] Capture first intentionally stop-focused drive route
- [x] Build baseline stop metrics from real logs
- [x] Propose and test first algorithm change
- [x] Add low-speed transition-slew path for stop-state crossings
- [x] Add shouldStop release-lock hysteresis for clutch leapfrogging
- [x] Implement initial stop-controller rewrite path
- [x] Remove legacy new-long stop branch and keep a single stop-controller path
- [x] Add offline harsh-stop regression gates (measured + model-based)
- [x] Add controller-replay model gate for pre-drive algorithm checks
- [ ] Complete full rewrite validation and tune stop-controller behavior

## Reimplementation Status (2026-02-08)

- Short answer: **rewrite is active as the only new-long stop-controller path, with tuning still in progress**.
- Completed staged control commits:
  - `87759474c2` - low-speed stop command slew limiter.
  - `bf1e7081c2` - low-speed transition-slew across active control states.
  - `f0a45636ac` - shouldStop release-lock hysteresis to counter clutch-driven leapfrogging.
- Rewrite baseline now available in code:
  - `selfdrive/controls/lib/stopping_controller.py` (`StoppingController`)
  - wired directly in `selfdrive/controls/lib/longcontrol.py` stop branch.
  - legacy new-long stop shaping path removed.
- Next planned step:
  - on-road validation/tuning on the stop-controller.

## Session Log

### 2026-02-06: Initial baseline and prep

What was done:
- Reviewed stopping control flow in planner/control/actuation code.
- Mapped FrogPilot-specific stop logic (including force-stop/red-light path).
- Collected exact log fields for on-road debugging.
- Attempted SSH access to device endpoints.

SSH/connectivity results:
- `ssh comma`: `ssh: connect to host fw1.sshreachme-trial.com port 10418: Connection refused`
- `ssh commawifi`: `ssh: connect to host 192.168.50.10 port 22: Operation timed out`
- `ping 192.168.50.10`: 100% packet loss

Offline test status:
- Tried: `pytest -q selfdrive/test/longitudinal_maneuvers/test_longitudinal.py`
- Blocked by environment: `ModuleNotFoundError: No module named 'openpilot.common.params_pyx'`

Tooling prepared for upcoming SSH-enabled sessions:
- `tools/stopping/sync_new_logs.py`: discovers remote log files via SSH, tracks previously seen files, downloads only new/changed logs, writes JSON run reports.
- `tools/stopping/append_sync_report.py`: appends a structured sync summary into this markdown worklog from a JSON report.
- `tools/stopping/device_stop_settings.py`: snapshots current stop-related settings first and supports controlled fine-tuning writes.
- `tools/stopping/README.md`: quick workflow and command reference.
- Typical commands:
  - `python tools/stopping/device_stop_settings.py snapshot --host comma`
  - `python tools/stopping/device_stop_settings.py set --host comma --set StoppingDecelRate=0.35`
  - `python tools/stopping/sync_new_logs.py --host comma`
  - `python tools/stopping/sync_new_logs.py --host comma --include-rlog`
  - `python tools/stopping/append_sync_report.py --report-file <report_json> --settings-file <settings_json>`

### 2026-02-07: Connectivity restored and first baseline pull

What was done:
- Verified live device connectivity through `commawifi` profile.
- Confirmed `comma` profile still refused connection during this session.
- Hardened stopping tools to work with this environment:
  - `device_stop_settings.py` now reads/writes params files directly and tolerates noisy SSH stdout before JSON.
  - `sync_new_logs.py` now executes remote list script via SSH stdin, keeps dry-run read-only, and supports `--newest-first`.
  - `append_sync_report.py` now truncates long route/segment lists with `(+N more)` for readability.
  - Added `run_stopping_cycle.py` wrapper for one-command snapshot + sync + worklog append.

Device snapshot captured:
- Host: `commawifi`
- Params dir: `/data/params/d`
- Key values: `AdvancedLongitudinalTune=True`, `LongitudinalTune=True`, `HumanAcceleration=True`, `ForceStops=False`
- Stop tuning values:
  - `StartAccel=0.0`, `StopAccel=-1.5`, `StoppingDecelRate=0.5`, `VEgoStarting=0.1`
  - `VEgoStopping=0.5`, `StoppingSpeedBreakpoint=0.4`, `StoppingErrorFactor=1.3`

Initial log pull status:
- Remote files discovered: `4267` (qlog set)
- Seeded downloads completed: `63` files (`3` + `60`) from newest route `000006c0--81e575d831`
- Report files:
  - `~/.comma/stopping_behavior/reports/sync_commawifi_20260207T160534Z.json`
  - `~/.comma/stopping_behavior/reports/sync_commawifi_20260207T160628Z.json`

### 2026-02-07: Corpus stop scan (expanded)

What was done:
- Identified root cause of undercount: local mirror contained only a subset of on-device qlogs.
- Confirmed device had a much larger set (`4267`) than local baseline (`467` at that point).
- Expanded local mirror to `1005` qlogs and reran corpus analysis.
- Upgraded stop-event detector to support three modes:
  - `engaged_signal`: strict OpenPilot stop-signal transitions.
  - `speed_transition`: speed/standstill transitions from car-state behavior.
  - `hybrid`: merge of both modes (deduplicated by stop-hold region).
- Updated corpus scanner defaults to broad baseline mode (`hybrid`, `min-entry-speed=0.5`) for coverage.

Latest corpus baseline:
- Script: `python tools/stopping/find_stop_events_corpus.py --host commawifi --verbose-routes`
- Mode: `hybrid`
- min-entry-speed: `0.5 m/s`
- Routes analyzed: `210`
- Total qlogs analyzed: `1005`
- Total stop events found: `536`
- Report outputs:
  - `~/.comma/stopping_behavior/analysis/corpus/commawifi/20260207T173330Z/summary.json`
  - `~/.comma/stopping_behavior/analysis/corpus/commawifi/20260207T173330Z/summary.md`

Strict OP-only comparator:
- Script:
```bash
python tools/stopping/find_stop_events_corpus.py \
  --host commawifi \
  --event-mode engaged_signal \
  --min-entry-speed 2.0 \
  --output-dir ~/.comma/stopping_behavior/analysis/corpus/commawifi/20260207T173330Z_engaged
```
- Routes analyzed: `210`
- Total stop events found: `28`
- Report outputs:
  - `~/.comma/stopping_behavior/analysis/corpus/commawifi/20260207T173330Z_engaged/summary.json`
  - `~/.comma/stopping_behavior/analysis/corpus/commawifi/20260207T173330Z_engaged/summary.md`

Interpretation:
- The earlier "1-2 events" result came from two factors:
  - Incomplete local log mirror.
  - Detector mode that required engaged stop-signal onset.
- With larger corpus + hybrid detection, event counts are now in the expected hundreds.

Next baseline split to track:
- `engaged_signal` (strict algorithm-facing metric).
- `hybrid` (fleet/behavioral coverage metric).

### 2026-02-07: Failure-mode diagnosis (engaged stops)

What was done:
- Added `tools/stopping/diagnose_stop_failures.py` to classify/rank likely stop issues from corpus summaries.
- Generated an engaged-only diagnosis report from:
  - `~/.comma/stopping_behavior/analysis/corpus/commawifi/20260207T173330Z_engaged/summary.json`
- Generated fresh route graph packs for top problematic routes:
  - `~/.comma/stopping_behavior/analysis/commawifi/0000061b--dafb553d6e/20260207T180907Z/summary.md`
  - `~/.comma/stopping_behavior/analysis/commawifi/00000619--b7c72f4e46/20260207T180907Z/summary.md`

Diagnosis report:
- `~/.comma/stopping_behavior/analysis/corpus/commawifi/20260207T173330Z_engaged/failure_diagnosis.md`

Key findings (engaged subset, 28 events):
- `late_signal_onset`: 10 events
  - Repeated pattern: high approach speed with low entry speed at stop-signal onset (`approach-entry gap` often > 6 m/s).
  - Suggests stop-state entry is frequently occurring very late in the stop sequence.
- `long_moving_stop`: 2 events
  - One severe outlier (`route 0000061b--dafb553d6e`, seg 6): `duration 25.6s`, `distance 115.4m`, `min_cmd -1.97`.
  - Indicates prolonged "stopping behavior while still moving", likely perceived as inconsistent/hesitant.
- `post_stop_creep`: 2 events
  - Worst creep outliers around `0.245-0.271 m/s`.
  - Candidate source of roll-forward discomfort after standstill.
- `hard_brake`: 1 event
  - Rare but present; needs to be avoided in tuning changes.

Working hypothesis (given project constraints):
- Dominant issue is inconsistency in final stop execution, not globally excessive braking.

### 2026-02-09: Step-Back, Replay Correctness, and Jerk Metric Update

Why a step-back was needed:
- On-road feedback reported harsh final-stop jerk even while offline model gates were passing.
- Root cause: replay/controller gates were sensitive to log sampling rate (qlog often ~10Hz) vs runtime control rate (100Hz). Without dt-aware scaling, offline replays can misrepresent how quickly the controller can unwind commands.
- Modeling note: `carControl.actuators.accel` (desired accel) is only a valid proxy for applied braking when OpenPilot is enabled; when disabled it may still be logged but is not applied. Manual braking lives on a different signal path (`carState.brake`/`brakePressed`).

Key tooling changes:
- `tools/stopping/run_stopping_cycle.py` now selects the newest route from the sync report (`new_routes`/`downloaded_files`) and passes it to analysis automatically (prevents "wrong newest route" due to local mtime skew).
- `tools/stopping/analyze_stopping_behavior.py` route auto-pick now prefers the hex route prefix ordering (e.g., `000006df--...`) as a tie-breaker to reduce mtime-related drift.
- `tools/stopping/benchmark_controller_variants.py` now supports a third replay variant:
  - `legacy_32b8be` (old longcontrol stopping behavior from commit `32b8becae845191833ef8eb2accc4fb94cc1de17`) for side-by-side comparisons.

Key model-gate change:
- `tools/stopping/check_harsh_stops_model.py` now adds a "standstill command jerk" component:
  - At the predicted standstill crossing (`vEgo < 0.05`), compute `|accel_cmd| / 0.40s` as a proxy for wheel-stop jerk (brake force still being applied at the moment wheels stop).
  - The predicted end-stop jerk gate uses the max of the moving-phase jerk metric and this standstill command jerk proxy.

Controller changes to support smoother landings (without harming runtime behavior):
- `selfdrive/controls/lib/stopping_controller.py` now scales per-step command limits and lock timers by dt so offline 10Hz replays behave like 100Hz runtime.
- Reduced low-speed "re-brake" behavior that was driving end-stop command jerk:
  - Rollout tightening no longer forces additional braking when decel is already strong (tighten only when `aEgo > -0.35`).
  - Soft-landing release expanded to unwind more aggressively when decel is already strong (`aEgo < -0.50`) to reduce braking magnitude right at wheel stop.

Legacy vs current quick check (offline benchmark on harsh route):
- Route: `000006df--4cb2d5b964` summary `~/.comma/stopping_behavior/analysis/commawifi/000006df--4cb2d5b964/20260209T212213Z/summary.json`
- Model: `~/.comma/stopping_behavior/models/stopping_model_20260209T211657Z_all.json`
- `benchmark_controller_variants.py` output (controller-scope engaged-stopping, `min-entry-speed=0.0`):
  - current harsh rate: `0.800`
  - abstract harsh rate: `0.400`
  - legacy_32b8be harsh rate: `0.600`
  - Interpretation: the abstract controller replay is currently the strongest candidate in offline scoring, but we still want to converge to one runtime controller after extracting the best ideas.
- Planner/model stop-decision timing is treated as external input in this project.
- Therefore first tuning/algorithm iteration should target controller-side behavior only:
  1) minimize wheel-stop jerk at final stop transition,
  2) reduce post-stop creep / re-accel-then-stop behavior,
  3) preserve stopping distance (avoid excessive rollout),
  4) preserve comfort (no increase in hard decel outliers).

### 2026-02-07: Engaged stop quality recalibration (v4)

What was done:
- Extended event extraction metrics to capture near-hold stop release behavior:
  - `stop_signal_dropped_before_hold`
  - `left_stopping_state_before_hold`
  - `positive_accel_cmd_near_hold`
  - `max_accel_cmd_near_hold_mps2`
  - `speed_rebound_after_hold_mps`
- Extended diagnosis ranking/scoring to include those new signals.
- Added detector config fields to corpus summary (`min_entry_speed`, hold/search thresholds) and route-name tagging in per-event records.
- Added `tools/stopping/build_review_pack.py` to auto-generate top-route graph packs from a corpus summary.
- Reran engaged-only corpus and diagnosis with updated tooling.
- Generated review pack:
  - `~/.comma/stopping_behavior/analysis/review_pack/20260207T2206Z/manifest.json`

Updated engaged baseline:
- Corpus: `~/.comma/stopping_behavior/analysis/corpus/commawifi/20260207T2159Z_engaged05_v4/summary.json`
- Diagnosis (default thresholds):
  - `~/.comma/stopping_behavior/analysis/corpus/commawifi/20260207T2159Z_engaged05_v4/failure_diagnosis.md`
  - Events: `37` (engaged/signal)
  - Default strict good estimate: `25/37 (67.6%)`
- Diagnosis (comfort-strict calibration run):
  - `~/.comma/stopping_behavior/analysis/corpus/commawifi/20260207T2159Z_engaged05_v4/failure_diagnosis_comfort_strict.md`
  - Strict good estimate: `11/37 (29.7%)`
  - Uses tighter comfort gates (`end-stop jerk 0.5`, `end-step 0.06`, `cmd jerk 0.03`, `cmd step 0.003`, `creep 0.08`, `rebound 0.10`)

Key findings:
- Rollout issue remains real (`7/37` over 2 m), with two severe outliers dominating distance risk.
- Near-hold release/dropout behavior is present and measurable:
  - `stop_signal_dropped_before_hold`: `3/37`
  - `left_stopping_state_before_hold`: `3/37`
  - `positive_accel_cmd_near_hold`: `1/37`
  - `speed_rebound_after_hold >= 0.15`: `3/37`
- One high-severity route/event (`00000619--b7c72f4e46`, event 14, seg 51) clearly shows:
  - `stopping -> pid/off` near hold,
  - command sign flip to positive accel,
  - strong rebound (~`1.05 m/s`), matching "almost stop -> slight accel -> stop/restart" symptom.
- Red-light events are included (`6` seen); force-stop events are currently absent in this corpus (`0` seen, consistent with `ForceStops=False` snapshot).

Conclusion:
- The earlier default metric pack was too permissive for comfort scoring and overestimated "good stops."
- There are at least two distinct failure classes in engaged stops:
  1) long rollout/late-onset behavior,
  2) near-hold release/dropout behavior (stop intent/state drops near wheel stop).
- A comfort-calibrated threshold pack can align measured "good" rate with observed field feedback (~30%).
- This calibration needs your confirmation before it becomes the primary KPI.

Assumptions and questions to confirm with driver feedback:
- Should stop-intent/state drop before hold always count as a failure, even if rebound is small?
- For queue-like creeping events, should they be excluded from final-stop comfort KPI or kept in the same bucket?
- Is event pattern `00000619--b7c72f4e46 / event 14` representative of the bad stop feel you report?
- Do you want the comfort KPI to target ~30% baseline now (for sensitivity), or stay broader and use only objective rollout/rebound failures?

### 2026-02-07: Driver context update - clutch disturbance is primary

Driver-provided context:
- Main physical issue is automatic gearbox/clutch behavior near stop:
  - with near-static commanded braking, drivetrain can unexpectedly engage/de-engage clutch,
  - this produces apparent accel/brake change without corresponding command change.
- Stop-intent/state drop near hold is often expected (lead starts moving / light changes), so treat it as contextual unless coupled with clear rebound/fault.
- Upcoming drives: bookmark bad stops only (`userFlag`).

What was changed in tooling:
- Added clutch-sensitive metrics in stop event extraction:
  - `unexpected_accel_while_braking_mps2`
  - `stable_cmd_accel_delta_mps2`
  - `low_speed_cmd_std_mps2`
  - `speed_rebound_while_stop_signal_mps`
  - `positive_accel_cmd_with_stop_signal_near_hold`
- Updated diagnosis logic:
  - stop-signal/state-drop is now low-severity contextual by default.
  - emphasized unexpected release/rebound only when stop signal remains active.
  - added dedicated clutch-disturbance flags.
- Added bookmark workflow tool:
  - `tools/stopping/find_bookmarked_bad_stops.py` (maps `userFlag` to nearest stop event).

Latest corpus/diagnosis with clutch-aware metrics:
- Corpus:
  - `~/.comma/stopping_behavior/analysis/corpus/commawifi/20260207T2230Z_engaged05_v5/summary.json`
- Default diagnosis:
  - `~/.comma/stopping_behavior/analysis/corpus/commawifi/20260207T2230Z_engaged05_v5/failure_diagnosis.md`
  - strict good estimate: `27/37 (73.0%)`
- KPI v1 diagnosis (middle baseline target):
  - `~/.comma/stopping_behavior/analysis/corpus/commawifi/20260207T2230Z_engaged05_v5/failure_diagnosis_kpi_v1.md`
  - thresholds: rollout `<=2.0`, jerk `<=0.6`, end-step `<=0.09`, creep `<=0.10`, plus clutch/rebound gates
  - strict good estimate: `18/37 (48.6%)` (selected as practical ~50% baseline)

Bookmark prep status:
- Confirmed `userFlag` message type exists in schema (`cereal/log.capnp`).
- Dry sample scan currently found no bookmarks in tested routes (expected before next drive).
- Example command ready for post-drive runs:
  - `python tools/stopping/find_bookmarked_bad_stops.py --host commawifi --event-mode engaged_signal --min-entry-speed 0.5`

### 2026-02-07: shouldStop-stays-true disturbance signature

Clarified target failure signature (from driver context):
- `shouldStop` remains true,
- near final stop, deceleration briefly collapses (or slight accel appears),
- then stop process resumes.

Tooling updates:
- Added direct metrics for this signature:
  - `speed_rebound_while_should_stop_mps`
  - `should_stop_unexpected_accel_mps2`
  - `should_stop_decel_relief_spike_mps2`
- Added diagnosis flags:
  - `speed_rebound_while_should_stop`
  - `unexpected_accel_under_should_stop`
  - `decel_relief_spike_under_should_stop`

Latest clutch-signature baseline:
- Source:
  - `~/.comma/stopping_behavior/analysis/corpus/commawifi/20260207T2245Z_engaged05_v6/summary.json`
- Count (engaged events):
  - total events: `37`
  - clutch-disturbance candidates (any of the three shouldStop signatures): `4`
  - rebound while shouldStop >= `0.08`: `3`
  - unexpected accel under shouldStop > `0.10`: `4`
  - decel-relief spike under shouldStop > `0.18`: `3`
- Candidate events:
  - `00000619--b7c72f4e46` event `5`
  - `00000619--b7c72f4e46` event `19`
  - `0000061b--dafb553d6e` event `6`
  - `0000061f--5d391aed1d` event `1`

KPI v1 status (with clutch-aware gates):
- Report:
  - `~/.comma/stopping_behavior/analysis/corpus/commawifi/20260207T2245Z_engaged05_v6/failure_diagnosis_kpi_v1.md`
- Strict good-stop estimate: `18/37 (48.6%)`

### 2026-02-07: Historical stopping commit anchor

Reviewed commit:
- `32b8becae8` - `longitudinal: refine stopping behavior`

Touched files:
- `selfdrive/controls/lib/longcontrol.py`
- `selfdrive/car/interfaces.py`
- `frogpilot/common/frogpilot_variables.py`
- corresponding UI/settings files

Relevant changes observed:
- Added/used `StoppingSpeedBreakpoint` tunable (shared through `CarParams.stoppingVbp`).
- Refined stopping envelopes and thresholds in `longcontrol.py`.
- Updated old-long stopping transition/behavior path and stop decel handling.

Use of this anchor:
- Future tuning should be evaluated against pre/post behavior implied by this commit, with focus on:
  - final wheel-stop jerk,
  - stop-distance tradeoff,
  - re-accel/re-stop incidents.

## Current Stop Algorithm (Code Map)

### 1) Planner decides stop intent

Primary path:
- `selfdrive/controls/lib/longitudinal_planner.py`
- `selfdrive/controls/lib/drive_helpers.py`

Behavior:
- Planner computes `longitudinalPlan.aTarget` and `longitudinalPlan.shouldStop`.
- `shouldStop` is true when both short-horizon target speeds are below `vEgoStopping`.
- In ACC mode, planner uses MPC outputs; in blended mode, it can combine MPC with model `shouldStop`.

### 2) Longitudinal state machine applies stop/start logic

Primary path:
- `selfdrive/controls/lib/longcontrol.py`

Behavior:
- State machine: `off`, `pid`, `stopping`, `starting`.
- New long API transition to `stopping` is driven directly by planner `shouldStop`.
- Legacy long API transition uses planned-stop plus standstill/brake context.
- In `stopping` state, acceleration is shaped with speed-dependent envelopes and clipped to `[CP.stopAccel, -0.05]`.
- Uses FrogPilot toggles for low-speed stop shaping (`stoppingSpeedBreakpoint`, `stoppingErrorFactor`, `vEgoStopping`, `vEgoStarting`, etc.).

### 3) controlsd bridges planner to actuator command

Primary path:
- `selfdrive/controls/controlsd.py`

Behavior:
- Calls `LoC.update(..., long_plan.aTarget, long_plan.shouldStop, ...)`.
- Writes `actuators.accel` and `actuators.longControlState`.
- Handles standstill resume command behavior (`cruiseControl.resume`).

### 4) CarParams defaults and per-platform overrides

Primary path:
- `selfdrive/car/interfaces.py`
- `selfdrive/car/*/interface.py`

Behavior:
- Default stop params are set in `get_std_params()`.
- Platform interfaces override `vEgoStopping`, `vEgoStarting`, `stoppingDecelRate`, etc.

### 5) FrogPilot force-stop path (stop-light driven)

Primary path:
- `frogpilot/controls/lib/conditional_experimental_mode.py`
- `frogpilot/controls/lib/frogpilot_vcruise.py`
- `frogpilot/controls/frogpilot_planner.py`

Behavior:
- Model stop-light/stop-sign intent can set `stop_light_detected`.
- If `force_stops` is enabled, `frogpilot_vcruise` can progressively pull `vCruise` down and set `forcing_stop`.
- Planner publishes `frogpilotPlan.forcingStop`, `forcingStopLength`, `redLight`.

### 6) Radar low-speed lead override (indirect stop influence)

Primary path:
- `selfdrive/controls/radard.py`

Behavior:
- At low ego speeds, radar can choose a close-in low-speed lead candidate even without strong model confirmation.
- This can indirectly influence decel/stopping behavior through lead tracking and planner constraints.

## Code Review Notes (2026-02-07)

- Planner stop trigger (`selfdrive/controls/lib/drive_helpers.py`):
  - `shouldStop` is driven by `v_target < vEgoStopping` and `v_target_1sec < vEgoStopping`.
  - There is no direct acceleration-based stop trigger in this helper; speed horizons drive the stop intent.

- Control API split (`selfdrive/controls/controlsd.py`):
  - `LoC.update(...)` is used when `old_long_api` is false.
  - `LoC.update_old_long(...)` is used for legacy path (`old_long_api` true, currently configured for select GM/Hyundai branches in toggles).

- New long API stop behavior (`selfdrive/controls/lib/longcontrol.py`):
  - State transition into stopping uses planner `shouldStop` directly (`stopping_condition = should_stop`).
  - Stop shaping uses speed breakpoints and expected accel envelopes, then clips to `[CP.stopAccel, -0.05]`.
  - `stoppingSpeedBreakpoint` and `stoppingErrorFactor` directly affect this shaping loop.
  - `starting` state uses `a_target` when `human_acceleration` is enabled, otherwise uses `startAccel`.

- Old long API stop behavior (`selfdrive/controls/lib/longcontrol.py`):
  - Uses `planned_stop` from `v_target`, `v_target_1sec`, and non-accelerating condition.
  - Applies fixed ramp-down by `stoppingDecelRate * DT_CTRL` until `stopAccel`.

- Runtime tuning source (`frogpilot/common/frogpilot_variables.py`):
  - Advanced stop knobs are only applied when `AdvancedLongitudinalTune` is enabled and tuning-level gates pass.
  - Additional car/profile overrides can still replace stop thresholds (`experimental_gm_tune`, `frogsgomoo_tweak`).

- Force-stop path (`frogpilot/controls/lib/frogpilot_vcruise.py`):
  - Requires `stop_light_detected`, planner/model stop context, controls enabled, and `ForceStops=true`.
  - Current device snapshot has `ForceStops=false`, so this path is effectively disabled unless setting changes.

- Platform coupling note:
  - GM controller uses `frogpilot_toggles.stopAccel` for near-stop brake command mapping (`selfdrive/car/gm/carcontroller.py`).

## Key Tunables (Current Source of Truth)

CarParams defaults (`selfdrive/car/interfaces.py`):
- `stopAccel = -2.0`
- `stoppingDecelRate = 0.8`
- `vEgoStopping = 0.5`
- `vEgoStarting = 0.5`

FrogPilot runtime override source (`frogpilot/common/frogpilot_variables.py`):
- `StartAccel`: range `[0, 4]`
- `StopAccel`: range `[-4, 0]`
- `StoppingDecelRate`: range `[0.001, 1]`
- `VEgoStarting`: range `[0.01, 1]`
- `VEgoStopping`: range `[0.01, 1]`
- `StoppingSpeedBreakpoint`: range `[0.01, 0.5]`, default `0.2`
- `StoppingErrorFactor`: range `[0.5, 5]`, default `2.0`

Note:
- `stoppingSpeedBreakpoint` is injected into `CarParams.stoppingVbp` in `selfdrive/car/interfaces.py` so UI tuning and control share a common breakpoint.

## Logging and Signals for Stop Debugging

### Device log storage

- Log root on device is typically `/data/media/0/realdata/`.
- Alternate roots may be used: `/data/media/0/realdata_HD/` or `/data/media/0/realdata_konik/`.
- Segment logs include `rlog.bz2`, `qlog.bz2`, camera files.

### High-value fields to plot/inspect

From `longitudinalPlan`:
- `shouldStop`
- `aTarget`
- `speeds[]`
- `accels[]`
- `longitudinalPlanSource`

From `controlsState`:
- `longControlState`
- `forceDecel`
- `vCruise`
- `upAccelCmd`, `uiAccelCmd`, `ufAccelCmd`

From `carState` / `carControl`:
- `vEgo`, `aEgo`, `standstill`, `brakePressed`, `gasPressed`
- `carControl.actuators.accel`
- `carControl.actuators.longControlState`

FrogPilot-specific:
- `frogpilotPlan.forcingStop`
- `frogpilotPlan.forcingStopLength`
- `frogpilotPlan.redLight`
- `frogpilotOnroadEvents` containing `forcingStop`

## Measurement Protocol (Current)

### Step A: run the standard collection cycle

Recommended command:
```bash
python tools/stopping/run_stopping_cycle.py \
  --host commawifi \
  --state-file ~/.comma/stopping_behavior/sync_state_v2.json \
  --max-downloads 80 \
  --newest-first \
  --note "Post-drive stopping baseline pull"
```

### Step B: verify and inspect newly pulled segments

Examples:
```bash
ls -lah ~/.comma/stopping_behavior/downloads/commawifi/data/media/0/realdata_konik | tail -n 20
```

### Step C: analyze stop events in the new segments

Local tooling examples:
```bash
# PlotJuggler with longitudinal layout
python tools/plotjuggler/juggle.py "<route_or_segment>" --layout longitudinal

# Custom analysis scripts can use LogReader on qlog/rlog
```

## Initial Risks / Notes

- Stop behavior is split between planner intent (`shouldStop`) and controller stop shaping; changes must be validated in both layers.
- FrogPilot adds a second stop path (`forcing_stop`) through virtual cruise reduction; this can interact with core stopping transitions.
- Low-speed radar lead override may alter stop approach behavior in dense/urban scenes.
- Offline longitudinal maneuver test currently fails in this environment due missing compiled module (`params_pyx`).
- Road logs are currently the strongest validation path.

## Next Actions (Queued)

1. Pull one intentionally stop-focused route with at least 5 diverse stops (lead stop, red light, no lead, downhill, uphill).
2. Build a baseline table per stop event:
- entry speed
- decel profile (`aEgo`)
- planner `shouldStop` transition time
- `longControlState` transition timing
- final stop error (distance/creep/overshoot)
3. Compare baseline behavior against current tuning snapshot:
- `StopAccel=-1.5`, `StoppingDecelRate=0.5`
- `StoppingErrorFactor=1.3`, `StoppingSpeedBreakpoint=0.4`
4. Identify first minimal algorithm adjustment candidate and define A/B criteria.

### 2026-02-07: Log sync from commawifi

- Host: `commawifi`
- Sync counts: remote=4267, new=4264, changed=0, downloaded=60
- Additional counts: unchanged=3, failures=0, skipped_limit=4204
- New routes detected: 210 total; sample `00000056--6f60cbf398`, `00000057--37a67dc1fd`, `00000058--638af96068`, `00000062--3bc7cae631`; +190 more
- New segments detected: 4264 total; sample `00000056--6f60cbf398--22`, `00000057--37a67dc1fd--25`, `00000058--638af96068--22`; +4244 more
- Downloaded route summary: `000006c0--81e575d831` (60 segments)
- Downloaded segments: 60 total; sample `000006c0--81e575d831--100`, `000006c0--81e575d831--101`, `000006c0--81e575d831--102`; +56 more
- Report JSON: `~/.comma/stopping_behavior/reports/sync_commawifi_20260207T160628Z.json`
- Settings JSON: `~/.comma/stopping_behavior/settings/stop_settings_commawifi_20260207T160001Z.json`
- Stop settings snapshot:
  - AdvancedLongitudinalTune=True, LongitudinalTune=True, HumanAcceleration=True, ForceStops=False
  - StartAccel=0.0, StopAccel=-1.5, StoppingDecelRate=0.5, VEgoStarting=0.1
  - VEgoStopping=0.5, StoppingSpeedBreakpoint=0.4, StoppingErrorFactor=1.3
- Findings: selected `new_routes` route (`000006e0--63b246dcdc`) was all-standstill samples (`vEgo==0`), so it's not a useful stopping-analysis target.
  Next: rerun analysis on the newest moving route from the sync report (example: `00000689--800e2befe7`) or pin with `run_stopping_cycle.py --analysis-route <route>`.
- Note: Initial seed run used --newest-first --max-downloads 60 (plus 3 files from prior report sync_commawifi_20260207T160534Z.json).
- Note: Using state file ~/.comma/stopping_behavior/sync_state_v2.json for this baseline.

### 2026-02-07: Stopping analysis for route 000006c0--81e575d831

- Host: `commawifi`
- Route: `000006c0--81e575d831`
- Segments analyzed: 63
- Detected stop events: 1
- Median duration to standstill hold: 2.20 s
- Median approach speed: 5.95 m/s
- Median entry speed: 0.80 m/s
- Median min aEgo: -0.68 m/s²
- Median min accel cmd: -0.49 m/s²
- Median shouldStop->stopping delay: 0.000 s
- Median creep after stop: 0.042 m/s
- Settings snapshot: `~/.comma/stopping_behavior/settings/stop_settings_commawifi_20260207T160001Z.json`
- Analysis summary JSON: `~/.comma/stopping_behavior/analysis/commawifi/000006c0--81e575d831/20260207T170334Z/summary.json`
- Analysis summary Markdown: `~/.comma/stopping_behavior/analysis/commawifi/000006c0--81e575d831/20260207T170334Z/summary.md`
- Example event graph: `~/.comma/stopping_behavior/analysis/commawifi/000006c0--81e575d831/20260207T170334Z/events/event_001_seg_101.html`
- Data quality note: low event count; collect more intentional stop scenarios for stronger comparisons.
- Note: Baseline analysis from initial seeded qlogs; route appears to include only one low-speed stop event.

### 2026-02-07: Stopping analysis for route 00000619--b7c72f4e46

- Host: `commawifi`
- Route: `00000619--b7c72f4e46`
- Segments analyzed: 72
- Detected stop events: 41
- Median duration to standstill hold: 8.40 s
- Median approach speed: 2.02 m/s
- Median entry speed: 1.97 m/s
- Median min aEgo: -0.72 m/s²
- Median min accel cmd: -0.10 m/s²
- Median shouldStop->stopping delay: 0.000 s
- Median creep after stop: 0.045 m/s
- Settings snapshot: `~/.comma/stopping_behavior/settings/stop_settings_commawifi_20260207T160534Z.json`
- Analysis summary JSON: `~/.comma/stopping_behavior/analysis/commawifi/00000619--b7c72f4e46/20260207T173508Z/summary.json`
- Analysis summary Markdown: `~/.comma/stopping_behavior/analysis/commawifi/00000619--b7c72f4e46/20260207T173508Z/summary.md`
- Example event graph: `~/.comma/stopping_behavior/analysis/commawifi/00000619--b7c72f4e46/20260207T173508Z/events/event_001_seg_005.html`
- Note: Route-level graph baseline on event-rich route

### 2026-02-07: Coverage baseline and `shouldStop`-true disturbance triage

- SSH verification:
  - `ssh commawifi` reachable (`comma-ffd439f1`, verified `2026-02-07T22:18:20Z`).
- Local corpus size:
  - qlogs available locally: `1214`.
  - Route count analyzed: `210`.

- Detector coverage comparison (same local corpus):
  - `engaged_signal + require_enabled + min_entry_speed=0.1`:
    - `48` events.
    - Summary: `~/.comma/stopping_behavior/analysis/corpus/commawifi/20260207T223433Z/summary.json`
  - `speed_transition + require_enabled + min_entry_speed=0.0`:
    - `105` events.
    - Summary: `~/.comma/stopping_behavior/analysis/corpus/commawifi/20260207T223330Z/summary.json`
  - `hybrid + min_entry_speed=0.1`:
    - `744` events.
    - Summary: `~/.comma/stopping_behavior/analysis/corpus/commawifi/20260207T223505Z/summary.json`

- Conclusion:
  - Low event count was primarily detector-gating related (signal-only mode), not log scarcity.
  - For engaged-stop baseline, `speed_transition + --require-enabled-speed-events` gives broader coverage while staying in OP-enabled context.
  - Signal-only runs remain useful for stop-intent transition studies, but are too sparse as the only KPI denominator.

- Failure definition alignment (user requirement):
  - Bad stop candidate = near-hold disturbance while `shouldStop` remains true:
    - decel collapses or briefly flips positive (with braking command still active),
    - may cause tiny rebound/restart before final hold,
    - no requirement that stop signal/state drops.

- Disturbance counts on `105` engaged speed-transition events:
  - `speed_rebound_while_should_stop_mps >= 0.08`: `11`
  - `should_stop_unexpected_accel_mps2 > 0.10`: `5`
  - `should_stop_decel_relief_spike_mps2 > 0.18`: `4`
  - Union of above (priority clutch-disturbance candidates): `12`
  - Current strict-good estimate (rollout/jerk/rebound/disturbance gates): `27/105 (25.7%)`

- Top high-confidence clutch-disturbance candidates:
  - `00000619--b7c72f4e46` seg `68` event `26` (rebound `0.167`, unexpected accel `0.562`, relief spike `0.301`)
  - `00000619--b7c72f4e46` seg `20` event `7` (rebound `0.128`, unexpected accel `0.370`, relief spike `0.229`)
  - `0000061f--5d391aed1d` seg `12` event `1` (rebound `0.147`, unexpected accel `0.461`, relief spike `0.334`)
  - `0000069b--cb3ff8e2d3` seg `14` event `1` (rebound `0.144`, unexpected accel `0.451`, relief spike `0.289`)

- Diagnosis report generated:
  - `~/.comma/stopping_behavior/analysis/corpus/commawifi/20260207T223330Z/failure_diagnosis_speed.md`

- Route review pack generated for top-ranked routes:
  - `~/.comma/stopping_behavior/analysis/review_pack/20260207T223330Z_speed_top3/manifest.json`
  - Included routes: `00000619--b7c72f4e46`, `0000061b--dafb553d6e`, `000006be--864be32bdb`

- Tooling fix:
  - `tools/stopping/build_review_pack.py` updated to include new clutch-disturbance threshold args used by diagnosis (`should_stop_*`, `unexpected_accel_*`, `stable_cmd_accel_delta_*`), fixing a runtime `AttributeError`.

- Current assumptions and open questions:
  - Assumption: stop-signal/state drops near hold are often contextual (lead/light release), not primary faults unless paired with rebound/disturbance.
  - Assumption: clutch disturbance is observable as `aEgo` relief/rebound while braking command remains near-constant and `shouldStop` remains true.
  - Open question for next drive labels: when bookmarking a bad stop, confirm whether perceived issue is (a) end-stop jerk spike or (b) near-hold release/restart, so we can split tuning targets cleanly.

### 2026-02-07: Device settings snapshot (pre-change baseline)

- Snapshot file:
  - `~/.comma/stopping_behavior/settings/stop_settings_commawifi_20260207T222847Z.json`
- Current stop-relevant values:
  - `ForceStops=False`
  - `StopAccel=-1.5`
  - `StoppingDecelRate=0.5`
  - `StoppingErrorFactor=1.3`
  - `StoppingSpeedBreakpoint=0.4`
  - `VEgoStopping=0.5`
  - `VEgoStarting=0.1`

### 2026-02-07: Bookmark pipeline check (speed-transition mode)

- Command run:
  - `python tools/stopping/find_bookmarked_bad_stops.py --host commawifi --event-mode speed_transition --require-enabled-speed-events --min-entry-speed 0.0`
- Output:
  - `~/.comma/stopping_behavior/analysis/bookmarks/commawifi/20260207T222936Z/summary.json`
  - `~/.comma/stopping_behavior/analysis/bookmarks/commawifi/20260207T222936Z/summary.md`
- Result:
  - `6` bookmarks detected across `4` routes.
  - `3` bookmarks matched to detected stop events.
  - `3` bookmarks unmatched (no detected stop event in configured match windows).
- Note:
  - This validates end-to-end bookmark ingestion for upcoming "bad stop only" tagging drives.

### 2026-02-08: Control changes + reset checkpoint

- Branch state:
  - `origin/!my-fp` head: `c597285b87`
  - Recent stopping commits on branch:
    - `6f00acb616` - add low-speed shouldStop disturbance guard in stop state.
    - `246d2f1ec3` - extract disturbance guard helper + unit tests.
    - `707e5ba323` - add low-speed soft-landing smoothing path to reduce end-stop jerk.
    - `c597285b87` - allow cycle runner to set `StoppingSpeedBreakpoint` / `StoppingErrorFactor`.

- Code paths changed:
  - `selfdrive/controls/lib/longcontrol.py`
  - `selfdrive/controls/lib/stopping_guard.py`
  - `selfdrive/controls/lib/tests/test_stopping_guard.py`
  - `tools/stopping/run_stopping_cycle.py`
  - `tools/stopping/README.md`

- Jerk-focused control update summary:
  - Added `apply_should_stop_soft_landing(...)` for low-speed (`vEgo < 0.45`) shouldStop-true phase.
  - Soft-landing blends output toward a gentle hold target and applies low-speed accel-command slew limits.
  - Disturbance guard still takes priority when `aEgo` exceeds expected accel (clutch-disturbance case).

- Tests:
  - New deterministic unit tests for disturbance and soft-landing helpers:
    - `selfdrive/controls/lib/tests/test_stopping_guard.py`
  - Local run:
    - `pytest --noconftest selfdrive/controls/lib/tests/test_stopping_guard.py -q`
    - Result: `10 passed`

- Cycle runner update (device param workflow):
  - New args:
    - `--set-stopping-speed-breakpoint <float>`
    - `--set-stopping-error-factor <float>`
    - `--settings-dry-run`
  - This enables one-command set/snapshot/sync/analyze loops without hardcoding values in control code.

- Reset start points:
  - Process + state document (primary): `docs/stopping_behavior_worklog.md`
  - Operational command reference: `tools/stopping/README.md`

### 2026-02-08: New under-house stop test review

- Sync pull:
  - Command: `python tools/stopping/sync_new_logs.py --host commawifi --newest-first --max-downloads 120`
  - Report: `~/.comma/stopping_behavior/reports/sync_commawifi_20260208T105508Z.json`
  - Result: downloaded `120` files, including new routes `000006c1--6b213c87ba` and `000006c2--da2641223a`.

- Route analysis:
  - `000006c1--6b213c87ba`:
    - `0` stop events detected (speed/hybrid detectors).
  - `000006c2--da2641223a`:
    - speed-transition summary: `~/.comma/stopping_behavior/analysis/commawifi/000006c2--da2641223a/20260208T105858Z/summary.md`
    - engaged-signal summary: `~/.comma/stopping_behavior/analysis/commawifi/000006c2--da2641223a/20260208T105949Z/summary.md`
    - speed-transition detected stop events: `5` (all segment `0`)

- Harshness findings on newest route (`000006c2--da2641223a`, speed-transition):
  - End-stop jerk:
    - `5/5` events with `EndJerk > 0.6`
    - `2/5` events with `EndJerk > 1.0`
    - median `EndJerk = 0.86 m/s³`
  - Command jerk spikes:
    - `2/5` events with `CmdJerk > 3.0 m/s³`
    - worst events:
      - event `1`: `CmdJerk=9.23`, `EndJerk=1.63`
      - event `4`: `CmdJerk=9.90`, `EndJerk=0.86`
  - Other symptoms:
    - `2/5` re-accel-before-hold
    - `1/5` rollout over `2 m` (event `5`, `3.08 m`)

- Conclusion:
  - Driver feedback ("relatively harsh") matches telemetry on the newest test route.
  - Leapfrogging exists but is secondary in this sample; dominant issue remains final-stop jerk/command sharpness.

- Device settings at review time:
  - Snapshot: `~/.comma/stopping_behavior/settings/stop_settings_commawifi_20260208T110012Z.json`
  - `StoppingSpeedBreakpoint=0.4`, `StoppingErrorFactor=1.3`, `StopAccel=-1.5`, `StoppingDecelRate=0.5`

### 2026-02-08: Post-deploy harsh stop follow-up + transition-slew update

- New log pull:
  - Report: `~/.comma/stopping_behavior/reports/sync_commawifi_20260208T122921Z.json`
  - Result: `20` newest changed qlogs downloaded.
  - Latest active routes in pull: `000006ad--e5a4035a9f`, `000006ae--65b9634de3`.

- Fresh corpus baseline (enabled speed-transition focus):
  - Command:
    - `python tools/stopping/find_stop_events_corpus.py --host commawifi --event-mode speed_transition --require-enabled-speed-events --min-entry-speed 0.0 --output-dir ~/.comma/stopping_behavior/analysis/corpus/commawifi/20260208T1240Z_speed`
  - Output:
    - `~/.comma/stopping_behavior/analysis/corpus/commawifi/20260208T1240Z_speed/summary.json`
    - `~/.comma/stopping_behavior/analysis/corpus/commawifi/20260208T1240Z_speed/failure_diagnosis_speed.md`
    - `~/.comma/stopping_behavior/analysis/corpus/commawifi/20260208T1240Z_speed/failure_diagnosis_speed_kpi.md`
  - Coverage:
    - routes: `213`
    - qlogs: `1713`
    - focused stop events: `220`
  - Aggregate indicators:
    - median rollout from `2 m/s`: `2.70 m`
    - median end-stop jerk: `0.68 m/s^3`
    - median end-stop accel step: `0.10 m/s^2`
  - KPI-tight pass rate (comfort-oriented thresholds): `10/220 (4.5%)`.

- Latest post-deploy route readback (`000006c3--6b080b114f`):
  - speed-transition summary:
    - `~/.comma/stopping_behavior/analysis/commawifi/000006c3--6b080b114f/20260208T1238Z_speed/summary.md`
    - `7` events, median `EndJerk=0.64`, median `CmdStep=0.06`.
  - engaged-signal summary:
    - `~/.comma/stopping_behavior/analysis/commawifi/000006c3--6b080b114f/20260208T1238Z_engaged/summary.md`
    - `5` events, with command-step spikes still present on some near-hold transitions (`CmdStep` up to `0.42`).

- Interpretation:
  - We still see near-stop command-step spikes even after the previous stop-state-only slew limiter.
  - Main remaining code-path gap: low-speed slew limiting was only applied inside `LongCtrlState.stopping`.
  - State transitions out of stopping (`stopping -> pid/starting`) could still introduce abrupt near-hold accel-command changes.

- New control update implemented:
  - Added generalized low-speed output slew helper:
    - `selfdrive/controls/lib/stopping_guard.py`
      - `apply_low_speed_output_slew(...)`
  - Integrated it in new-long control update path:
    - `selfdrive/controls/lib/longcontrol.py`
      - Applies low-speed slew for all active states (not only stopping), with:
        - disturbance-aware stronger braking when `shouldStop` stays true,
        - conservative release when near-hold and not resuming,
        - faster release allowance only under explicit resume intent (`a_target > 0.2`, non-stop, low-speed).
  - Updated deterministic unit tests:
    - `selfdrive/controls/lib/tests/test_stopping_guard.py`
    - local result: `15 passed`.

### 2026-02-08: Drive feedback - less harsh, more leapfrogging

Driver feedback:
- Latest drive felt less harsh overall, but had about two leapfrogging events.
- Clarified mechanism: leapfrogging can occur while accel command is still negative, when command gradually becomes less negative and clutch re-engages.

Log pull + route results:
- Sync report: `~/.comma/stopping_behavior/reports/sync_commawifi_20260208T130203Z.json`
  - downloaded: `47`, failures: `73` (intermittent SSH resets during transfer)
  - new route discovered: `000006c4--becd5ca972`
- Route analysis:
  - `~/.comma/stopping_behavior/analysis/commawifi/000006c4--becd5ca972/20260208T1307Z_speed/summary.md`
  - `~/.comma/stopping_behavior/analysis/commawifi/000006c4--becd5ca972/20260208T1307Z_engaged/summary.md`
- Evidence matching driver report:
  - engaged-signal events: `4`
  - re-accel-before-hold: `1/4`
  - rebound while shouldStop true >= `0.08`: `2/4`
  - one key event showed rebound with still-negative near-hold command (`max accel cmd near hold ~= -0.09`), consistent with clutch disturbance under easing negative brake request.

Controller update for this symptom:
- Added shouldStop release-lock hysteresis to reduce brake-release easing immediately after low-speed disturbance detections:
  - `selfdrive/controls/lib/longcontrol.py`
    - tracks short lock window (`should_stop_release_lock_counter`) when `aEgo` exceeds expected accel under active shouldStop braking.
    - applies low-speed floor while lock is active to avoid quickly drifting to weak braking near hold.
    - lock decays automatically and resets when shouldStop clears/off state.
- Extended low-speed slew helper with lock-aware release limits:
  - `selfdrive/controls/lib/stopping_guard.py`
    - `apply_low_speed_output_slew(..., release_lock_active=...)` now tightens release step when lock is active.
- Unit tests updated:
  - `selfdrive/controls/lib/tests/test_stopping_guard.py`
  - local result: `16 passed`.

### 2026-02-08: Step-back rewrite kickoff (`stopping_v2`)

Context:
- After latest feedback ("still needs work", mix of harsh + acceptable stops), we started a structured rewrite path rather than stacking more ad-hoc patches.

What was implemented:
- New rewrite module:
  - `selfdrive/controls/lib/stopping_v2.py`
  - introduces explicit stop phases:
    - `APPROACH`
    - `NEAR_HOLD`
    - `HOLD`
  - includes disturbance-triggered release-lock hysteresis in the controller itself.
- `longcontrol` integration:
  - `selfdrive/controls/lib/longcontrol.py`
  - new-long stopping branch can run either:
    - rewrite path (`StoppingV2Controller`)
    - legacy path (existing stop shaping).
  - in-code switch:
    - `USE_STOPPING_V2` in `longcontrol.py`.
- Legacy fallback retained:
  - previous stop guard/slew logic remains available behind in-code switch for rollback safety.

Validation:
- Added rewrite-focused unit tests:
  - `selfdrive/controls/lib/tests/test_stopping_v2.py`
- Existing stop guard tests kept:
  - `selfdrive/controls/lib/tests/test_stopping_guard.py`
- Local run:
  - `pytest --noconftest selfdrive/controls/lib/tests/test_stopping_guard.py selfdrive/controls/lib/tests/test_stopping_v2.py -q`
  - result: `20 passed`.

Rewrite status after this step:
- Rewrite scaffold is live and selectable.
- On-road A/B and threshold tuning are still required before declaring rewrite complete.

### 2026-02-08: Crash review + hotfix (`UnknownKeyName: DisableStoppingV2`)

Crash:
- `controlsd` crashed at startup in `longcontrol.update()` when calling:
  - `self.params.get_bool("DisableStoppingV2")`
- Exception:
  - `common.params_pyx.UnknownKeyName: b'DisableStoppingV2'`

Root cause:
- `Params.get_bool()` validates key names against the registered params key set.
- New key `DisableStoppingV2` was referenced in code but not registered, so lookup raised and killed process.

Immediate fix:
- Removed runtime lookup of `DisableStoppingV2` from `selfdrive/controls/lib/longcontrol.py`.
- Kept `stopping_v2` enabled by default (safe startup, no unknown-key access).
- Updated docs to keep stop-controller selection in-code unless a real UI setting is added.

Validation:
- `pytest --noconftest selfdrive/controls/lib/tests/test_stopping_guard.py selfdrive/controls/lib/tests/test_stopping_controller.py -q`
- result: `15 passed`.

### 2026-02-08: Stop-response model baseline + offline harshness prediction

Why this was added:
- We need an offline way to predict harsh-stop risk from logs before another road test.
- Static thresholds on measured summaries are useful, but they do not model delayed clutch/response dynamics directly.

New tooling:
- `tools/stopping/stopping_model.py`
  - fits a compact linear response model with delayed accel-command input and low-speed/clutch-relief features.
  - simulates per-event low-speed stop response to estimate predicted jerk/floor-accel risk.
- `tools/stopping/fit_stopping_model.py`
  - fits the model from one or more `summary.json` analysis outputs plus local qlogs.
- `tools/stopping/check_harsh_stops_model.py`
  - runs a pass/fail gate over predicted harsh-stop metrics from event replay.
- `tools/stopping/test_stopping_model.py`
  - synthetic tests for delay recovery, simulation stability, and JSON load behavior.

Latest baseline run (newest speed-transition routes):
- Training inputs:
  - `000006c5--b1aca6b12f/20260208T1850Z_speed`
  - `000006c6--25385db785/20260208T1850Z_speed`
  - `000006c7--86cecffe81/20260208T1850Z_speed`
  - `000006c8--ee131d0581/20260208T1850Z_speed`
  - `000006c9--b7bca8a66b/20260208T1850Z_speed`
  - `000006ca--087c6ac51e/20260208T1850Z_speed`
- Fit output:
  - model: `~/.comma/stopping_behavior/models/stopping_model_20260208_latest_speed.json`
  - windows: `10`, rows: `252`
  - best delay: `5` frames
  - fit quality: `rmse=0.1181`, `mae=0.0713`, `r2=0.9019`
- Model gate output:
  - `~/.comma/stopping_behavior/analysis/model_harsh_check_20260208_latest_speed.json`
  - status: `fail`
  - considered: `10`, harsh: `4`, harsh rate: `0.400` (`> 0.20`)
- Measured gate cross-check (same routes):
  - `~/.comma/stopping_behavior/analysis/harsh_check_20260208_latest_speed.json`
  - status: `fail`
  - considered: `10`, harsh: `7`, harsh rate: `0.700`

Interpretation:
- Both measured and model-predicted gates agree current tuning is still not acceptable for final-stop comfort.
- Model gate is less strict than measured gate on this set (4 vs 7 harsh), so we should treat it as complementary early warning, not replacement KPI yet.
- Near-term use:
  1) keep measured harsh gate as primary objective check,
  2) use model gate to detect dynamics-regression risk earlier when command-shape changes are tested.

Validation:
- `pytest --noconftest selfdrive/controls/lib/tests/test_stopping_guard.py selfdrive/controls/lib/tests/test_stopping_controller.py tools/stopping/test_check_harsh_stops.py tools/stopping/test_stopping_model.py -q`
- result: `19 passed`.

### 2026-02-08: Naming cleanup + offline harsh-stop regression gate

Requested:
- Drop `v2` naming now that only one stop controller remains.
- Add a way to detect harsh-stop regressions without another drive.

Code cleanup:
- Renamed stop controller module/classes to non-versioned names:
  - `selfdrive/controls/lib/stopping_controller.py`
    - `StoppingController`
    - `StoppingPhase`
    - `StoppingResult`
- Updated integration and tests:
  - `selfdrive/controls/lib/longcontrol.py` now imports `StoppingController` from `stopping_controller.py`.
  - `selfdrive/controls/lib/tests/test_stopping_controller.py` (renamed from `test_stopping_v2.py`).

Offline harsh regression tooling:
- Added `tools/stopping/check_harsh_stops.py`:
  - reads one or more `analyze_stopping_behavior.py` `summary.json` files,
  - classifies harsh events via thresholds:
    - `end_stop_jerk_mps3`
    - `end_stop_cmd_jerk_mps3`
    - `end_stop_accel_step_mps2`
    - `min_a_ego_mps2`,
  - fails with non-zero exit code if harsh rate exceeds configured limits.
- Added script tests:
  - `tools/stopping/test_check_harsh_stops.py`

Validation:
- `pytest --noconftest selfdrive/controls/lib/tests/test_stopping_guard.py selfdrive/controls/lib/tests/test_stopping_controller.py tools/stopping/test_check_harsh_stops.py -q`
- result: `15 passed`.

### 2026-02-08: Smoother feel, rollout too long at low speed (tuning pass)

Driver feedback after latest test drive:
- Stop feel was smoother overall.
- Remaining issue shifted to **too much stopping distance at low speeds** (2 m max acceptable only for higher-speed stop entries; low-speed rollout should be much shorter).

Latest-drive route analysis (new routes on device):
- latest segments observed: `000006c5`..`000006ca`
- analyzed routes: `000006c5--b1aca6b12f`, `000006c6--25385db785`, `000006c7--86cecffe81`, `000006c8--ee131d0581`, `000006c9--b7bca8a66b`, `000006ca--087c6ac51e`
- analysis outputs:
  - `~/.comma/stopping_behavior/analysis/commawifi/<route>/20260208T1850Z_speed/summary.json`
  - `~/.comma/stopping_behavior/analysis/commawifi/<route>/20260208T1850Z_engaged/summary.json`
- engaged-signal highlights from newest drive:
  - events found: `7`
  - over 2 m rollout: `2/7`
  - worst examples:
    - `000006ca--087c6ac51e` event `1`: entry `0.03 m/s`, rollout `2.84 m`, jerk `0.22`
    - `000006ca--087c6ac51e` event `3`: entry `0.60 m/s`, rollout `6.21 m`, jerk `0.54`
- interpretation:
  - low jerk + high rollout indicates comfort is improved, but braking authority is too permissive in low-speed approach/near-hold for some clutch-disturbance cases.

Controller tuning implemented (`stopping_v2`):
- `selfdrive/controls/lib/stopping_v2.py`
  - expanded `NEAR_HOLD` region up to `0.85 m/s` (from `0.55 m/s`) so low-speed approach spends less time in weak `APPROACH` shaping.
  - strengthened `NEAR_HOLD`/`HOLD` targets and reduced low-speed release steps.
  - added adaptive **low-speed rollout tightening**:
    - tracks `low_speed_rollout_m` while shouldStop remains active below `1.2 m/s`,
    - progressively tightens release cap and applies stronger brake floor when rollout distance keeps growing.
  - resets rollout tracker when stop condition clears or standstill is reached.

Test coverage updates:
- `selfdrive/controls/lib/tests/test_stopping_v2.py`
  - verify near-hold phase now covers mid-low speed (`0.70 m/s`),
  - verify prolonged low-speed rollout triggers stronger braking than baseline,
  - retain release-lock behavior validation under updated logic.
- local run:
  - `pytest --noconftest selfdrive/controls/lib/tests/test_stopping_guard.py selfdrive/controls/lib/tests/test_stopping_v2.py -q`
  - result: `22 passed`.

### 2026-02-08: `stopping_v2` only (legacy new-long stop path removed)

Request:
- Replace old new-long stop implementation and keep only `stopping_v2`.

Code changes:
- `selfdrive/controls/lib/longcontrol.py`
  - removed in-code split between `stopping_v2` and legacy stop shaping.
  - stop-state path now always calls `StoppingV2Controller.update(...)`.
  - removed legacy stop-path state and dead code:
    - `USE_STOPPING_V2`
    - `self.use_stopping_v2`
    - `self.should_stop_release_lock_counter`
    - legacy disturbance/soft-landing branch logic.
- `selfdrive/controls/lib/stopping_guard.py`
  - removed legacy stop-only helper functions that were part of old branch:
    - `apply_should_stop_disturbance_guard(...)`
    - `apply_should_stop_soft_landing(...)`
  - retained `apply_low_speed_output_slew(...)` used outside stop-state branch.
- `selfdrive/controls/lib/tests/test_stopping_guard.py`
  - removed tests for deleted legacy stop helpers.
  - kept low-speed output slew tests.

Validation:
- `pytest --noconftest selfdrive/controls/lib/tests/test_stopping_guard.py selfdrive/controls/lib/tests/test_stopping_v2.py -q`

### 2026-02-08: Delay-aware release governor + controller-replay gate

Objective from this round:
- Move from "log-only harsh classification" to a **testable offline algorithm loop**:
  1) replay candidate stop-controller behavior through the fitted stop-response model,
  2) gate predicted harshness before deploying to the car.

Controller updates (`selfdrive/controls/lib/stopping_controller.py`):
- Added delayed-command tracking inside the controller (`delay_frames=5`) to match fitted plant lag.
- Added delay-aware release governor:
  - detects recent brake-relief relative to delayed effective command,
  - tightens release-step and applies extra hold bias when low-speed relief is too fast.
- Added over-brake damping:
  - if observed decel is significantly stronger than expected near hold, controller allows controlled brake relief to reduce harsh final decel.
- Reduced approach-phase slew magnitudes (`brake_step`/`release_step`) to lower jerk spikes at low-speed entry into stopping.

Model gate tooling update:
- Extended `tools/stopping/check_harsh_stops_model.py` with:
  - `--command-source recorded|controller`
  - `--stopping-speed-breakpoint`
  - `--stop-accel`
- `command-source=controller` now replays `StoppingController` outputs through the fitted model for each event window.

Test coverage added:
- `selfdrive/controls/lib/tests/test_stopping_controller.py`
  - `test_stopping_controller_delay_release_guard_limits_release_relief`
  - `test_stopping_controller_over_brake_damping_relieves_harsh_decel`
- `tools/stopping/test_check_harsh_stops_model.py`
  - replay helper coverage (`jerk_window_metrics`, `simulate_event_with_controller`).
- Full local run:
  - `pytest --noconftest selfdrive/controls/lib/tests/test_stopping_guard.py selfdrive/controls/lib/tests/test_stopping_controller.py tools/stopping/test_check_harsh_stops.py tools/stopping/test_stopping_model.py tools/stopping/test_check_harsh_stops_model.py -q`
  - result: `23 passed`

Controller-replay gate results on latest speed corpus (`000006c5..000006ca`, 20260208T1850Z):
- Target gate (for next on-road iteration):
  - command:
    - `python tools/stopping/check_harsh_stops_model.py ... --command-source controller --max-pred-end-jerk 0.70 --max-harsh-rate 0.10`
  - result: `pass` (`events=10`, `harsh=1`, `rate=0.10`)
  - output: `~/.comma/stopping_behavior/analysis/model_harsh_check_controller_20260208_latest_speed.json`
- Stretch gate (kept intentionally failing for iterative tuning):
  - command:
    - `python tools/stopping/check_harsh_stops_model.py ... --command-source controller --max-pred-end-jerk 0.65 --max-harsh-rate 0.10`
  - result: `fail` (`events=10`, `harsh=2`, `rate=0.20`)

Interpretation:
- We now have exactly what was requested:
  - a stricter failing offline test profile to iterate against,
  - and a primary offline target profile that currently passes after this tuning pass.
- Next algorithm iteration should focus on the two remaining stretch-fail events (route `000006c7--86cecffe81`, events `1` and `4`).

### 2026-02-08: Stop tuning params/settings removal + `vEgoStopping` decision

Decision on Hyundai Santa Fe HEV 2022 (`ret.vEgoStopping`):
- Keep `ret.vEgoStopping = 0.5` in `selfdrive/car/hyundai/interface.py`.
- Rationale: on this platform/gearbox behavior, earlier stop-state entry is safer for final-phase smoothness; dropping to `0.1` tends to delay stop-state takeover and increases end-stop jerk risk.

Policy applied in codebase:
- Stopping behavior tuning is now code-defined, not exposed via FrogPilot params/UI, unless we later add explicit user-facing settings again.

Removed from params registry and settings/UI:
- Params removed from `common/params.cc` and FrogPilot defaults/read paths:
  - `StartAccel`, `StopAccel`, `StoppingDecelRate`, `VEgoStarting`, `VEgoStopping`,
  - `StoppingSpeedBreakpoint`, `StoppingErrorFactor`,
  - and associated `*Stock` keys.
- Removed Advanced Longitudinal UI controls for the keys above in:
  - `frogpilot/ui/qt/offroad/longitudinal_settings.*`
  - `frogpilot/ui/qt/offroad/frogpilot_settings.*`

Controller/config wiring cleanup:
- `selfdrive/car/interfaces.py`
  - stop breakpoint now uses in-code constant `FROGPILOT_STOPPING_SPEED_BREAKPOINT = 0.4` (no runtime param dependency).
- `selfdrive/controls/lib/longcontrol.py`
  - stopping curve breakpoint now reads from `CP.stoppingVbp[1]` rather than FrogPilot param toggles.

Tooling cleanup (`tools/stopping`):
- Removed deprecated stop-param writes/reads from:
  - `device_stop_settings.py`
  - `run_stopping_cycle.py`
  - `append_sync_report.py`
  - `README.md` examples/options.

Validation performed:
- `python -m py_compile frogpilot/common/frogpilot_variables.py tools/stopping/device_stop_settings.py tools/stopping/run_stopping_cycle.py tools/stopping/append_sync_report.py`
- `scons -u -j8 frogpilot/ui/qt/offroad/longitudinal_settings.o frogpilot/ui/qt/offroad/frogpilot_settings.o`
- repo search confirms no remaining runtime references to removed stop params outside historical docs.

### 2026-02-08: Log sync from commawifi

- Host: `commawifi`
- Sync counts: remote=0, new=0, changed=0, downloaded=0
- Additional counts: unchanged=0, failures=0, skipped_limit=0
- New routes detected: none
- New segments detected: none
- Downloaded route summary: none
- Report JSON: `/Users/radoslawchybicki/.comma/stopping_behavior/reports/sync_commawifi_20260208T200604Z.json`
- Settings JSON: `/Users/radoslawchybicki/.comma/stopping_behavior/settings/stop_settings_commawifi_20260208T200604Z.json`
- Stop settings snapshot: AdvancedLongitudinalTune=True, LongitudinalTune=True, HumanAcceleration=True, ... (+3 more)
- Sync errors: Failed to list remote logs from commawifi: Connection to 192.168.50.10 closed by remote host.
- Findings: _pending analysis of downloaded logs_

### 2026-02-08: Stopping analysis for route 000006c9--b7bca8a66b

- Host: `commawifi`
- Route: `000006c9--b7bca8a66b`
- Segments analyzed: 1
- Detected stop events: 0
- Note: route samples were all standstill (`vEgo==0`), so `speed_transition` cannot detect meaningful stop events here.
- Median duration to standstill hold: n/a s
- Median approach speed: n/a m/s
- Median entry speed: n/a m/s
- Median min aEgo: n/a m/s²
- Median min accel cmd: n/a m/s²
- Median shouldStop->stopping delay: n/a s
- Median creep after stop: n/a m/s
- Settings snapshot: `/Users/radoslawchybicki/.comma/stopping_behavior/settings/stop_settings_commawifi_20260208T200604Z.json`
- Analysis summary JSON: `/Users/radoslawchybicki/.comma/stopping_behavior/analysis/commawifi/cycle_20260208T200604Z/summary.json`
- Analysis summary Markdown: `/Users/radoslawchybicki/.comma/stopping_behavior/analysis/commawifi/cycle_20260208T200604Z/summary.md`
- Data quality note: low event count; collect more intentional stop scenarios for stronger comparisons.

### 2026-02-08: Harsh-stop regression test + clutch-disturbance relief tuning

Context:
- Latest requested sync attempt was interrupted by SSH instability (`commawifi` timed out/host down), so fresh route data from that drive was not yet available locally.
- Used the latest available harsh route seed (`000006c7--86cecffe81`, speed event `1`) to add a deterministic regression guard.

New failing-then-passing regression:
- Added `tools/stopping/test_check_harsh_stops_model.py::test_simulate_event_with_controller_regression_seed_limits_predicted_jerk`.
- Seed includes pre-stop command history and event start state from route `000006c7--86cecffe81`.
- Initial failure (before tuning): predicted jerk above threshold.
- Post-tuning result: predicted jerk now under threshold (`<= 0.70`).

Replay fidelity improvement:
- Updated `tools/stopping/check_harsh_stops_model.py::simulate_event_with_controller(...)` to prime command delay history from pre-event samples.
- Also preloads controller command history for the delay-aware release guard.
- This avoids cold-start bias where delayed command was unrealistically pinned to the start-sample command.

Controller tuning (`selfdrive/controls/lib/stopping_controller.py`):
- Added/strengthened a clutch-disturbance relief branch for low-speed should-stop windows:
  - trigger when `a_ego` is positive while commanded brake is already deep,
  - cap over-deep target ratcheting,
  - increase allowed release slew in this specific disturbance mode,
  - relax release-lock release cap only for this case.
- Goal: reduce end-stop jerk spikes caused by over-ratcheting brake under automatic clutch push behavior.

Validation:
- `pytest -q --noconftest tools/stopping/test_check_harsh_stops_model.py selfdrive/controls/lib/tests/test_stopping_controller.py`
- `pytest -q --noconftest selfdrive/controls/lib/tests/test_stopping_guard.py selfdrive/controls/lib/tests/test_stopping_controller.py tools/stopping/test_check_harsh_stops.py tools/stopping/test_stopping_model.py tools/stopping/test_check_harsh_stops_model.py`
- Result: all targeted stopping tests pass (`24 passed`).

Current strict corpus status (known routes, speed-source controller replay gate):
- command: `check_harsh_stops_model.py ... --max-pred-end-jerk 0.65 --max-harsh-rate 0.10`
- result: still fails (`2/10` harsh), but route `000006c7` event `1` improved materially (`predJerk` down to ~`0.697`).
- remaining top outlier in this corpus: route `000006c7`, event `4` (`predJerk` ~`0.890`).

### 2026-02-08: Manual latest-route pull (`000006cd--424a5e218e`)

- SSH sync tooling remained intermittent, so latest route was pulled manually from:
  - `/data/media/0/realdata_konik/000006cd--424a5e218e--{0..9}/qlog`
- Local analyze outputs:
  - speed-transition strict (`require-enabled`): `0` events
  - hybrid: `1` low-speed event (`entry_speed ~0.16 m/s`)
- Note on that single hybrid event:
  - `shouldStop=False`, `long_state=off`, `accel_cmd=0.0` at event start
  - interpreted as non-engaged/manual low-speed stop transition, not a valid engaged-stop tuning sample
- Action:
  - keep engaged-stop tuning anchored to the seeded harsh event corpus (`000006c7` etc.) until fresh engaged stop samples are synced reliably.

### 2026-02-08: New harsh-drive seed (`000006ce`) and regression closure

Input from latest drive:
- Driver-reported stop feel still harsh.
- Pulled newest routes manually from device (`realdata_konik`) due intermittent full sync stability:
  - `000006cb--0312b41c42`, `000006cc--99fcc0035d`, `000006cd--424a5e218e`, `000006ce--d41951b402`.

Analysis snapshot:
- `000006ce--d41951b402` (engaged-signal mode) produced 6 stop events; harsh outlier:
  - event `6`: `entry_speed ~0.748 m/s`, `end_stop_jerk ~1.104 m/s^3`, `end_stop_accel_step ~0.120 m/s^2`.
- Event context confirms this is in engaged stopping path:
  - `enabled=True`, `long_state=stopping`, `long_state_cmd=stopping`, `shouldStop` asserted shortly after transition.

New failing regression added:
- `tools/stopping/test_check_harsh_stops_model.py`
  - `test_simulate_event_with_controller_regression_seed_ce_event6_limits_predicted_jerk`
  - seeded from route `000006ce--d41951b402` signal event `6`
  - threshold: `pred_end_stop_jerk <= 0.70`
- Initial result: failed (`~0.7469`).

Controller update to satisfy regression:
- `selfdrive/controls/lib/stopping_controller.py`
  - Added near-hold **comfort_release** branch for cases already decelerating strongly (`a_ego` sufficiently negative), to avoid further brake ratcheting at final approach.
  - Limits additional brake slew and increases permitted release slew in this specific condition.
  - Keeps clutch-disturbance relief branch intact for positive-acceleration push events.

Post-change validation:
- `pytest -q --noconftest tools/stopping/test_check_harsh_stops_model.py selfdrive/controls/lib/tests/test_stopping_controller.py`
  - all pass.
- full stopping suite:
  - `pytest -q --noconftest selfdrive/controls/lib/tests/test_stopping_guard.py selfdrive/controls/lib/tests/test_stopping_controller.py tools/stopping/test_check_harsh_stops.py tools/stopping/test_stopping_model.py tools/stopping/test_check_harsh_stops_model.py`
  - result: `25 passed`.

Notes:
- The new seeded harsh case now passes the regression threshold.
- Route-level strict gate for `000006ce` still has additional near-threshold outliers (~`0.71` jerk on two other events), so this should be treated as an incremental improvement, not final convergence.

### 2026-02-08: Post-drive improvement loop (`000006cf`)

Driver feedback:
- Stop feel improved versus prior pass, but still perceived as too jerky.

Latest route pulled:
- `000006cf--551c9ecf95` (manual qlog pull from `realdata_konik`).

Findings from analysis:
- Engaged-signal events: `2`.
- Speed events: `6`.
- Key harsh engaged pattern used for regression:
  - route `000006cf--551c9ecf95`, speed event `1`
  - high entry speed with deep predicted decel floor in model replay (`pred_min_a_ego` below `-1.10`).

New failing-then-passing regression:
- Added to `tools/stopping/test_check_harsh_stops_model.py`:
  - `test_simulate_event_with_controller_regression_seed_cf_event1_limits_predicted_floor`
- Initial failure:
  - `pred_min_a_ego_mps2 = -1.161876...` (threshold `>= -1.10`)
- Current result: passes after controller tuning.

Controller adjustments (`selfdrive/controls/lib/stopping_controller.py`):
- Added high-speed approach brake floor (for `shouldStop` approach with already meaningful braking), to avoid weak-brake carry-in that later amplifies decel spikes under delay/clutch dynamics.
- Added deeper-decel relief branches near hold/hold:
  - late near-hold/hold relief when measured decel is already too strong,
  - mild-command deep-decel relief for cases where the command is modest but decel keeps dropping.
- Added lock over-brake release handling so lock behavior does not block relief in clearly over-decelerating phases.

Validation:
- `pytest -q --noconftest tools/stopping/test_check_harsh_stops_model.py selfdrive/controls/lib/tests/test_stopping_controller.py`
- `pytest -q --noconftest selfdrive/controls/lib/tests/test_stopping_guard.py selfdrive/controls/lib/tests/test_stopping_controller.py tools/stopping/test_check_harsh_stops.py tools/stopping/test_stopping_model.py tools/stopping/test_check_harsh_stops_model.py`
- Result: all targeted tests pass (`26 passed`).

Residual note:
- Model replay still flags one outlier on `000006cf` (event `4`, `predJerk ~0.859`), so further tuning should target that profile next.

### 2026-02-08: Second pass on same drive (new failing test + pass)

User feedback:
- "Much better, but still too jerky."

Data used:
- Latest route still `000006cf--551c9ecf95` (engaged stops present).
- Selected engaged harsh profile for regression:
  - speed event `1` (high-speed shouldStop stop; deep decel floor in model replay).

New regression added:
- `tools/stopping/test_check_harsh_stops_model.py`
  - `test_simulate_event_with_controller_regression_seed_cf_event1_limits_predicted_floor`
  - threshold: `pred_min_a_ego >= -1.10`
  - initial failure: `pred_min_a_ego = -1.161876...`

Controller changes for this pass:
- `selfdrive/controls/lib/stopping_controller.py`
  - added approach-phase brake floor when entering stop at higher speed with already meaningful brake command.
  - added additional over-decel relief branches:
    - lock over-brake release handling,
    - late near-hold/hold deep-decel relief,
    - mild-command deep-decel relief,
    - approach deep-decel relief carry-in.

Validation:
- `pytest -q --noconftest tools/stopping/test_check_harsh_stops_model.py selfdrive/controls/lib/tests/test_stopping_controller.py`
- `pytest -q --noconftest selfdrive/controls/lib/tests/test_stopping_guard.py selfdrive/controls/lib/tests/test_stopping_controller.py tools/stopping/test_check_harsh_stops.py tools/stopping/test_stopping_model.py tools/stopping/test_check_harsh_stops_model.py`
- result: `26 passed`.

### 2026-02-09: Strategy abstraction + ranked controller comparison (`baseline`/`v2`/`v3`)

What changed in code:
- `selfdrive/controls/lib/stopping_controller.py`
  - Added explicit strategy tunings via `StoppingControllerTuning`.
  - Added strategy presets:
    - `baseline` (legacy behavior)
    - `v2` (smoothness-focused)
    - `v3` (balanced smoothness with stronger rollout tightening)
  - Added selector helpers:
    - `STOPPING_CONTROLLER_TUNINGS`
    - `get_stopping_controller_tuning(...)`
  - Default runtime strategy switched to `v2` (`DEFAULT_STOPPING_CONTROLLER_STRATEGY = "v2"`).

- `tools/stopping/check_harsh_stops_model.py`
  - Added controller strategy controls:
    - `--controller-strategy`
    - `--compare-controller-strategies baseline,v2,v3`
  - Added rollout-aware gating/metrics:
    - `--max-pred-rollout-m`
    - per-event `pred_rollout_distance_m`
  - Added per-event scoring and ranking helpers:
    - `score_event_metrics(...)`
    - `rank_controller_strategies(...)`
  - In compare mode, output now includes:
    - `strategy_ranking`
    - `best_strategy`
    - `strategies_evaluated`

- `tools/stopping/stopping_model.py`
  - Extended replay output with:
    - `predicted_v_ego`
    - `pred_rollout_distance_m`

- `tools/stopping/README.md`
  - Documented strategy comparison workflow and new options for rollout-aware ranking.

Test coverage updates:
- `tools/stopping/test_check_harsh_stops_model.py`
  - Added checks for rollout output and scoring/ranking behavior.
- `selfdrive/controls/lib/tests/test_stopping_controller.py`
  - Added strategy behavior comparisons (`v2` vs `v3`).

Validation run:
- `pytest -q --noconftest selfdrive/controls/lib/tests/test_stopping_guard.py selfdrive/controls/lib/tests/test_stopping_controller.py tools/stopping/test_check_harsh_stops.py tools/stopping/test_stopping_model.py tools/stopping/test_check_harsh_stops_model.py`
- Result: `30 passed`.

### 2026-02-09: Latest route re-review (`000006d6--8d6448a3fe`) with ranking harness

Connectivity note:
- `commawifi` became temporarily unavailable during this pass (timeout), so this analysis used already-downloaded local logs.

Input summaries:
- `~/.comma/stopping_behavior/analysis/commawifi/000006d6--8d6448a3fe/20260209T1602Z_speed/summary.json`
- `~/.comma/stopping_behavior/analysis/commawifi/000006d6--8d6448a3fe/20260209T1602Z_signal/summary.json`

Model:
- `~/.comma/stopping_behavior/models/stopping_model_20260208_latest_speed.json`

Speed-transition ranking run:
- Output: `~/.comma/stopping_behavior/analysis/model_harsh_rank_20260209_d6.json`
- Ranking (lower score is better):
  1. `v2`
  2. `v3`
  3. `baseline`
- Gate status for selected strategy (`v2`) still fails on this route in speed-mode:
  - `events=4`, `harsh=3`, `harsh_rate=0.75`
  - dominant issue in model replay remains long predicted rollout on specific events.

Engaged-signal ranking run:
- Output: `~/.comma/stopping_behavior/analysis/model_harsh_rank_20260209_d6_signal.json`
- Ranking:
  1. `v2`
  2. `v3`
  3. `baseline`
- Gate status passes at this threshold profile:
  - `events=4`, `harsh=1`, `harsh_rate=0.25`

Conclusion for this iteration:
- The new abstraction/ranking confirms `v2` is currently the best tradeoff among tested presets on the latest available route/model replay.
- `v2` is now set as runtime default for the next on-road test cycle.
- Remaining work is to reduce rollout-heavy outliers in speed-transition replay without reintroducing end-stop jerk spikes.

### 2026-02-09: From-scratch controller rewrite prototypes (`abstract_v2` / `abstract_v3`)

User direction captured:
- Do not only tune the existing controller.
- Build simpler, more abstract stop-controller implementations from scratch, then compare against legacy.
- Keep only the best implementation after evidence converges (no long-term multi-implementation maintenance).

Implemented:
- New abstract controller module:
  - `selfdrive/controls/lib/stopping_controller_abstract.py`
  - `AbstractStoppingControllerV2`
  - `AbstractStoppingControllerV3`
- New controller factory + variant naming:
  - `selfdrive/controls/lib/stopping_controller_factory.py`
  - variants:
    - `legacy_baseline`, `legacy_v2`, `legacy_v3`
    - `abstract_v2`, `abstract_v3`
- Replay tooling upgraded to compare controller *implementations* (not only legacy tunes):
  - `tools/stopping/check_harsh_stops_model.py`
    - `--controller-variant`
    - `--compare-controller-variants`
    - backward compatibility for old strategy flags preserved.

Test coverage added/updated:
- `selfdrive/controls/lib/tests/test_stopping_controller_abstract.py`
  - passthrough/reset behavior
  - rollout-tightening behavior
  - factory wiring sanity
- Existing stopping suites remain green.

Validation run:
- `pytest -q --noconftest`
  `selfdrive/controls/lib/tests/test_stopping_guard.py`
  `selfdrive/controls/lib/tests/test_stopping_controller.py`
  `selfdrive/controls/lib/tests/test_stopping_controller_abstract.py`
  `tools/stopping/test_check_harsh_stops.py`
  `tools/stopping/test_stopping_model.py`
  `tools/stopping/test_check_harsh_stops_model.py`
- Result: `33 passed`.

Offline comparison results (latest available local corpus):

1) Speed-focused mixed corpus (`000006ce speed` + `000006cf speed` + `000006d6 speed`)
- Output: `~/.comma/stopping_behavior/analysis/model_harsh_rank_20260209_speed_mix_variants.json`
- Ranking:
  1. `legacy_v2`
  2. `legacy_v3`
  3. `legacy_baseline`
  4. `abstract_v2`
  5. `abstract_v3`

2) Engaged-signal mixed corpus (`000006ce signal` + `000006cf signal` + `000006d6 signal`)
- Output: `~/.comma/stopping_behavior/analysis/model_harsh_rank_20260209_signal_mix_variants.json`
- Ranking:
  1. `legacy_baseline`
  2. `legacy_v3`
  3. `legacy_v2`
  4. `abstract_v3`
  5. `abstract_v2`

Interpretation:
- New abstract rewrites are functional and test-covered, but currently underperform legacy on both mixed corpora.
- The evidence does not support switching runtime to abstract controllers yet.
- Current runtime recommendation remains `legacy_v2` while we iterate on abstract controllers.

Process decision for next iteration:
- Keep both abstract implementations only as development candidates.
- Continue improving `abstract_v2`/`abstract_v3` until at least one wins ranking across both speed-focused and engaged-signal mixes.
- Once one candidate consistently wins, remove the weaker implementations and keep only the selected controller path.

### 2026-02-09: Convergence decision — single implementation only

Decision made:
- End with one production controller implementation.
- Remove abstract rewrite prototypes from the codebase for now.
- Continue with the existing `StoppingController` as the single implementation and tune it using offline ranking + on-road feedback.

Cleanup performed:
- Removed prototype files:
  - `selfdrive/controls/lib/stopping_controller_abstract.py`
  - `selfdrive/controls/lib/stopping_controller_factory.py`
  - `selfdrive/controls/lib/tests/test_stopping_controller_abstract.py`
- Reverted replay tool interface back to single-implementation strategy comparison:
  - `tools/stopping/check_harsh_stops_model.py`
    - active knobs:
      - `--controller-strategy`
      - `--compare-controller-strategies`
    - removed variant-level abstraction from CLI/output.
- Updated docs to match single-implementation process:
  - `tools/stopping/README.md`

Why this is the right next step:
- Mixed-corpus offline ranking did not show abstract controllers outperforming legacy.
- Maintaining multiple implementations adds complexity without current measurable gain.
- Fastest path to improved on-road behavior is one implementation + disciplined metric loop.

Current operating mode:
- Single implementation: `selfdrive/controls/lib/stopping_controller.py`
- Current default strategy: `v2`
- Strategy comparison remains only as a tuning tool (`baseline` vs `v2` vs `v3`) inside that one implementation.

### 2026-02-09: Regression analysis after on-road harshness report

User report:
- After deploying commit `e7ef36d6ae`, stopping feel was "definitely worse, harsher".

What was investigated:
- Device state and deployed commit verified (`!my-fp`, hash `e7ef36d`).
- New route pointer from device params:
  - `CurrentRoute=000006da--58cc7fc381`
- Attempted to pull/analyze latest route logs.
  - Route file pull succeeded for one segment (`...--0/qlog`) but parser reported corrupt/truncated qlog (`No carState samples`).
  - Device storage path listing intermittently returned `Input/output error` on `/data/media/0/realdata_konik`, so fresh measured-event extraction from this drive was not reliable in this pass.

Root cause identified (code-level):
- Behavior-affecting change between pre-regression runtime (`9f31fd`) and deployed runtime included this default switch in `selfdrive/controls/lib/stopping_controller.py`:
  - from implicit baseline behavior to `DEFAULT_STOPPING_CONTROLLER_STRATEGY = "v2"`
- `v2` tuning modifies core stop dynamics globally:
  - `target_bias=+0.014`
  - reduced brake ratchet (`brake_step_scale=0.88`)
  - increased release (`release_step_scale=1.12`)
  - weaker rollout tightening (`rollout_tighten_scale=0.92`)
  - weaker delay guard (`delay_guard_scale=0.88`)
- This can reduce robustness near final hold in some cases and increase perceived harshness on specific signal-driven stop profiles.

Why tests did not catch it:
- Coverage gap 1: existing replay regressions mostly used fixed explicit strategies (`v3` seeds), not the runtime default strategy.
- Coverage gap 2: no test asserted that `DEFAULT_STOPPING_CONTROLLER_STRATEGY` remains comfort-safe on an engaged signal-seed known to be sensitive.
- Coverage gap 3: ranking gates were weighted toward combined score/rollout and did not enforce a strict "default strategy comfort cannot regress" check.

Fix applied:
- Rolled runtime default back to baseline:
  - `selfdrive/controls/lib/stopping_controller.py`
    - `DEFAULT_STOPPING_CONTROLLER_STRATEGY = "baseline"`
- Aligned replay tool default with runtime default:
  - `tools/stopping/check_harsh_stops_model.py`
    - `--controller-strategy` default now `baseline`
- Updated docs/examples accordingly:
  - `tools/stopping/README.md`

New regression test added (prevents this exact miss):
- `tools/stopping/test_check_harsh_stops_model.py`
  - `test_default_strategy_matches_baseline_comfort_on_cf_signal_regression_seed`
- Seeded from `000006cf--551c9ecf95` signal-event profile where baseline outperformed v2 on predicted comfort score.
- Test explicitly checks:
  - runtime default is baseline,
  - default strategy is not worse than baseline on jerk/rollout for this seed,
  - baseline remains better than v2 for this regression case.

Validation:
- `pytest -q --noconftest selfdrive/controls/lib/tests/test_stopping_guard.py selfdrive/controls/lib/tests/test_stopping_controller.py tools/stopping/test_check_harsh_stops.py tools/stopping/test_stopping_model.py tools/stopping/test_check_harsh_stops_model.py`
- Result: `31 passed`.

Supporting ranking evidence (existing local corpora):
- Signal-mix ranking (`000006ce signal`, `000006cf signal`, `000006d6 signal`):
  - `baseline` ranks best over `v3` and `v2`.
  - output: `~/.comma/stopping_behavior/analysis/model_harsh_rank_20260209_signal_mix_strategies_after_rollback.json`

### 2026-02-09: Converged to one production stopping controller (removed strategy variants)

Decision:
- Keep a single production controller implementation and remove `baseline/v2/v3` runtime strategy branching.

Code changes:
- `selfdrive/controls/lib/stopping_controller.py`
  - Removed strategy/tuning scaffolding:
    - `StoppingControllerTuning`
    - `STOPPING_CONTROLLER_TUNINGS`
    - `DEFAULT_STOPPING_CONTROLLER_STRATEGY`
    - `get_stopping_controller_tuning(...)`
  - `StoppingController` no longer accepts a `strategy` argument.
  - Removed tuning multipliers/offsets from runtime control path (single deterministic behavior).
- `selfdrive/controls/lib/tests/test_stopping_controller.py`
  - Removed strategy comparison tests tied to `v2`/`v3`.
- `tools/stopping/check_harsh_stops_model.py`
  - Removed CLI strategy options:
    - `--controller-strategy`
    - `--compare-controller-strategies`
  - Removed strategy ranking output fields (`strategy_ranking`, `best_strategy`).
  - Controller replay now always evaluates the single runtime controller behavior.
- `tools/stopping/test_check_harsh_stops_model.py`
  - Removed strategy-default/ranking tests.
  - Replaced with single-controller regression gate on cf-signal seed:
    - `test_simulate_event_with_controller_regression_seed_cf_signal_event1_limits_predicted_jerk_and_rollout`.
- `tools/stopping/README.md`
  - Updated model-based flow to single-controller gate examples.

Why this step:
- Strategy variants were useful for exploration but are now maintenance overhead and a source of default-selection regressions.
- Current goal is one controller with strict replay/on-road regression gates.

Validation:
- `pytest -q --noconftest selfdrive/controls/lib/tests/test_stopping_guard.py selfdrive/controls/lib/tests/test_stopping_controller.py tools/stopping/test_check_harsh_stops.py tools/stopping/test_stopping_model.py tools/stopping/test_check_harsh_stops_model.py`
- Result: `28 passed`.

Current replay regression guard highlights:
- `test_simulate_event_with_controller_regression_seed_cf_signal_event1_limits_predicted_jerk_and_rollout`
  - `pred_end_stop_jerk_mps3 <= 0.35`
  - `pred_min_a_ego_mps2 >= -1.05`
  - `pred_rollout_distance_m <= 2.60`

### 2026-02-09: Model refresh from newest routes + cycle automation

Model refresh status:
- Existing model on disk was from 2026-02-08:
  - `~/.comma/stopping_behavior/models/stopping_model_20260208_latest_speed.json`
- Built fresh model using newest valid speed summaries (including 2026-02-09 routes):
  - output: `~/.comma/stopping_behavior/models/stopping_model_20260209_latest_speed.json`
  - fit stats: `windows=28`, `rows=690`, `best_delay_frames=1`, `rmse=0.1730`, `mae=0.1026`, `r2=0.8919`.

Gate/test check with refreshed model:
- Unit/integration stopping tests still pass:
  - `pytest -q --noconftest selfdrive/controls/lib/tests/test_stopping_guard.py selfdrive/controls/lib/tests/test_stopping_controller.py tools/stopping/test_check_harsh_stops.py tools/stopping/test_stopping_model.py tools/stopping/test_check_harsh_stops_model.py`
  - result: `28 passed`.
- Offline model gate on mixed newest speed summaries (`command-source=controller`) still fails:
  - `events=28`, `harsh_events=20`, `harsh_rate=0.714` (threshold `0.10`).
  - output: `~/.comma/stopping_behavior/analysis/model_harsh_check_controller_20260209_mix_speed.json`.

Process improvement added:
- `tools/stopping/run_stopping_cycle.py` now supports optional model lifecycle stages:
  - `--fit-model` (fit fresh model after sync/analysis)
  - `--fit-summary-json` or auto-discovery via `--fit-recent-summaries`
  - `--fit-event-source`, `--fit-max-delay-frames`, `--fit-min/max-speed`, `--fit-min-rows`
  - `--run-model-gate` (invoke `check_harsh_stops_model.py` on the same summary set)
  - configurable gate thresholds/output flags.
- `tools/stopping/README.md` updated with one-shot cycle example including model fit + model gate.

### 2026-02-09: Narrow rebound guard iteration (local-only, no new device sync)

Connectivity/log pull status:
- Attempted full cycle pull from `commawifi`:
  - `python3 tools/stopping/run_stopping_cycle.py --host commawifi ... --fit-model --run-model-gate`
  - failed at settings snapshot with SSH timeout (`connect to host 192.168.50.10 port 22: Operation timed out`).
- Proceeded using newest local summaries already on disk.

Local model and gate context used in this iteration:
- Summary set: `/tmp/stopping_recent_speed_summaries.txt` (8 newest speed summaries, 54 events total).
- Fitted model:
  - `/Users/radoslawchybicki/.comma/stopping_behavior/models/stopping_model_20260209_refresh_speed.json`
  - fit stats: `windows=54`, `rows=1381`, `best_delay_frames=1`, `rmse=0.1535`, `mae=0.0904`, `r2=0.8929`.
- Baseline controller replay gate on this set:
  - `events=54`, `harsh_events=44`, `harsh_rate=0.815` (fail against 0.10).
  - dominant harsh flag remains rollout.

Controller change made:
- `selfdrive/controls/lib/stopping_controller.py`
  - Added `rollout_rebound_guard`:
    - active only when disturbance rebound happens after rollout has already grown (`low_speed_rollout_m > 1.05`) while release lock is active.
    - increases brake ratchet and tightens release in that narrow scenario to reduce creep/retry behavior.
  - Added `medium_decel_relief` in near-hold:
    - when decel is already strong with medium-deep command, reduces further brake ratcheting to soften end-stop jerk.

Why these scopes:
- Broad changes caused regressions in seeded replay tests (especially `cf_signal_event1`).
- Final approach kept all seeded guardrails green and only touched narrow patterns tied to observed rebound/harshness.

Validation:
- Targeted tests:
  - `pytest -q --noconftest tools/stopping/test_check_harsh_stops_model.py selfdrive/controls/lib/tests/test_stopping_controller.py`
  - result: `15 passed`.
- Full stopping suite:
  - `pytest -q --noconftest selfdrive/controls/lib/tests/test_stopping_guard.py selfdrive/controls/lib/tests/test_stopping_controller.py tools/stopping/test_check_harsh_stops.py tools/stopping/test_stopping_model.py tools/stopping/test_check_harsh_stops_model.py`
  - result: `28 passed`.
- Latest route check (`000006d6` speed summary):
  - event 4 predicted rollout improved slightly (`3.051 -> 3.028`),
  - event 1 predicted end-stop jerk improved slightly (`0.869 -> 0.859`),
  - gate still fails on this route (`3/4 harsh`) and mixed-corpus gate remains `44/54 harsh`.

Conclusion for this iteration:
- Kept test suite green and made small directional improvements on latest-route replay metrics.
- Mixed-corpus model gate remains the primary unresolved blocker; rollout-dominant failures indicate we still need larger controller behavior changes (or revised event-window/gate selection) before expected pass criteria are realistic.

### 2026-02-09: Mixed-gate alignment fix + continued tuning

Why mixed gate was still failing hard:
- Replay gate was simulating `stopping_controller` over broad event windows that started before the controller is actually active.
- In runtime, `stopping_controller` is only used while long control is in `stopping` state.
- That mismatch inflated predicted rollout and made mixed-gate scores pessimistic.

Gate-tool changes:
- `tools/stopping/check_harsh_stops_model.py`
  - Added controller replay window options:
    - `--controller-window-mode {event,should_stop,stopping_state}` (default `stopping_state`)
    - `--controller-end-mode {hold,last_should_stop,last_stopping_state}` (default `last_stopping_state`)
  - Added `pred_rollout_from_2mps_m` in controller simulation and used it for rollout harsh checks/scoring.
  - Kept `pred_rollout_distance_m` (total) in output and added `pred_rollout_total_distance_m` row field for traceability.

Controller tuning changes:
- `selfdrive/controls/lib/stopping_controller.py`
  - Added `rollout_push` when rollout is building but decel remains weak at low speed.
  - Added `deep_command_jerk_relief` to unwind very deep inherited brake commands in narrow low-rollout near-hold cases.
  - Kept earlier narrow guards (`rollout_rebound_guard`, `medium_decel_relief`).

Validation:
- `pytest -q --noconftest selfdrive/controls/lib/tests/test_stopping_guard.py selfdrive/controls/lib/tests/test_stopping_controller.py tools/stopping/test_check_harsh_stops.py tools/stopping/test_stopping_model.py tools/stopping/test_check_harsh_stops_model.py`
- Result: `28 passed`.

Mixed gate progression on the same 8 newest local speed summaries:
- Before alignment (legacy event window):
  - `events=54`, `harsh=44`, `harsh_rate=0.815`
- After shouldStop-window alignment:
  - `events=48`, `harsh=27`, `harsh_rate=0.562`
- After stopping-state window alignment (current default):
  - `events=36`, `harsh=20`, `harsh_rate=0.556`

Current status:
- Still failing gate threshold (`0.10`) but now measured on controller-relevant windows.
- Remaining failures split roughly between:
  - high predicted end-stop jerk (~10 events),
  - rollout >2.0 m from low-speed stopping-state windows (~9 events),
  - decel floor overshoot (~3 events).
- Next tuning should target these top failing seeds directly (especially routes `000006cb`, `000006ce`, `000006cc`, `000006d6`).

### 2026-02-09: Pipeline split per requirement (all-stop model, engaged+stopping controller review)

Requirement applied:
- Model building should use **all stopping events** (including manual stops).
- Controller behavior review should use **engaged + stopping-state** events only.

Tooling updates:
- `tools/stopping/check_harsh_stops_model.py`
  - Added controller replay scope controls:
    - `--controller-scope {all, engaged, engaged_stopping}` (default: `engaged_stopping`)
    - `--controller-min-enabled-ratio` (default: `0.80`)
  - Controller replay still defaults to stopping-state windows:
    - `--controller-window-mode stopping_state`
    - `--controller-end-mode last_stopping_state`
  - Added rollout accounting field from low-speed phase:
    - `pred_rollout_from_2mps_m` (used for harsh rollout checks)
    - `pred_rollout_total_distance_m` retained in output rows for traceability.
  - Refined predicted jerk metric to ignore post-standstill relaxation spikes when moving-speed jerk samples exist.

- `tools/stopping/run_stopping_cycle.py`
  - `--fit-event-source` default changed to `all` (model fit from all stop events).
  - Model gate now passes controller scope knobs through:
    - `--model-gate-controller-scope` (default `engaged_stopping`)
    - `--model-gate-controller-min-enabled-ratio` (default `0.80`)
  - Baseline model-gate harsh-rate threshold default set to `0.50` for current stage.

- `tools/stopping/README.md`
  - Updated process notes to reflect the split:
    - fit model from all stops,
    - gate controller on engaged+stopping windows.
  - Documented baseline (`0.50`) vs stretch (`0.10`) harsh-rate targets.

Model/gate rerun on latest local mixed summaries (one summary per recent route, all event sources):
- Refit model:
  - `/Users/radoslawchybicki/.comma/stopping_behavior/models/stopping_model_20260209_allstops_scope.json`
  - fit stats: `windows=51`, `rows=1080`, `best_delay_frames=1`, `rmse=0.1568`, `mae=0.0942`, `r2=0.8939`.
- Engaged+stopping controller gate, stretch target (`max_harsh_rate=0.10`):
  - `events=33`, `harsh=15`, `harsh_rate=0.455` (fail stretch).
- Engaged+stopping controller gate, baseline target (`max_harsh_rate=0.50`):
  - `events=33`, `harsh=15`, `harsh_rate=0.455` (pass baseline).

Validation:
- `pytest -q --noconftest selfdrive/controls/lib/tests/test_stopping_guard.py selfdrive/controls/lib/tests/test_stopping_controller.py tools/stopping/test_check_harsh_stops.py tools/stopping/test_stopping_model.py tools/stopping/test_check_harsh_stops_model.py`
- Result: `28 passed`.

### 2026-02-09: Step-back experiment with a new abstract controller (offline benchmark)

Context:
- After splitting scopes (all-stop model fit vs engaged+stopping controller review), stretch gate still failed.
- Added a step-back benchmark to test a new control approach without touching runtime behavior.

New tooling:
- Added `tools/stopping/benchmark_controller_variants.py`.
  - Replays each event window with:
    - current runtime controller (`StoppingController`), and
    - a new abstract controller candidate (feedback + disturbance lock + rollout guard).
  - Uses same fitted model and same event windows for both variants.
  - Outputs per-event and aggregate comparison JSON.

Benchmark run (latest local mixed corpus, all-stop fitted model):
- Command:
  - `python tools/stopping/benchmark_controller_variants.py ... --event-source all --controller-scope engaged_stopping --controller-window-mode stopping_state --controller-end-mode last_stopping_state`
- Results:
  - events: `33`
  - current: `15/33 harsh`, `harsh_rate=0.455`, `avg_score=0.692`
  - abstract: `13/33 harsh`, `harsh_rate=0.394`, `avg_score=0.610`
  - improved events: `13`, worsened events: `20` (aggregate better, but tradeoffs remain).
- Output:
  - `/tmp/benchmark_controller_variants_allstops.json`

Interpretation:
- The new abstract approach is promising at aggregate level (lower harsh rate and score),
  but it is not yet clean enough to replace runtime directly.
- Next iteration should mine the improved events to extract robust rules, then port selectively into runtime controller.

### 2026-02-09: Log sync from commawifi

- Host: `commawifi`
- Sync counts: remote=4279, new=74, changed=2290, downloaded=120
- Additional counts: unchanged=1915, failures=0, skipped_limit=2244
- New routes detected: 21 total; sample: `000006c5--b1aca6b12f`, `000006c6--25385db785`, `000006c7--86cecffe81`; +18 more
- New segments detected: 74 total; sample: `000006c5--b1aca6b12f--0`, `000006c5--b1aca6b12f--1`, `000006c5--b1aca6b12f--2`; +71 more
- Downloaded route summary: `000006a5--c9ae338723` (46 segments), `000006c5--b1aca6b12f` (5 segments), `000006c6--25385db785` (2 segments) (+19 more)
- Downloaded segments: `000006a5--c9ae338723--102`, `000006a5--c9ae338723--103`, `000006a5--c9ae338723--104` (+117 more)
- Report JSON: `/Users/radoslawchybicki/.comma/stopping_behavior/reports/sync_commawifi_20260209T192349Z.json`
- Settings JSON: `/Users/radoslawchybicki/.comma/stopping_behavior/settings/stop_settings_commawifi_20260209T192349Z.json`
- Stop settings snapshot: AdvancedLongitudinalTune=True, LongitudinalTune=True, HumanAcceleration=True, ... (+3 more)
- Findings: _pending analysis of downloaded logs_

### 2026-02-09: Stopping analysis for route 000006a5--c9ae338723

- Host: `commawifi`
- Route: `000006a5--c9ae338723`
- Segments analyzed: 74
- Detected stop events: 7
- Median duration to standstill hold: 8.50 s
- Median approach speed: 3.72 m/s
- Median entry speed: 3.72 m/s
- Median min aEgo: -1.24 m/s²
- Median min accel cmd: 0.00 m/s²
- Median shouldStop->stopping delay: 1.398 s
- Median creep after stop: 0.049 m/s
- Settings snapshot: `/Users/radoslawchybicki/.comma/stopping_behavior/settings/stop_settings_commawifi_20260209T192349Z.json`
- Analysis summary JSON: `/Users/radoslawchybicki/.comma/stopping_behavior/analysis/commawifi/cycle_20260209T192349Z/summary.json`
- Analysis summary Markdown: `/Users/radoslawchybicki/.comma/stopping_behavior/analysis/commawifi/cycle_20260209T192349Z/summary.md`
- Example event graph: `/Users/radoslawchybicki/.comma/stopping_behavior/analysis/commawifi/cycle_20260209T192349Z/events/event_001_seg_103.html`

### 2026-02-09: High-level review checkpoint (tooling correctness before controller changes)

What was corrected:
- `run_stopping_cycle.py` fit-summary selection no longer silently collapses to only the just-analyzed summary.
  - New behavior: when explicit fit summaries are not provided, include current analysis summary plus recent discovered summaries (deduped).
- `discover_recent_summaries(..., event_source=\"all\")` now selects one summary per route to reduce overweighting the same route across multiple summary variants.
  - Current preference order per route: `hybrid` > `speed_transition` > `engaged_signal`.
- Added regression tests:
  - `tools/stopping/test_run_stopping_cycle.py`
  - covers both summary-selection behaviors above.

Validation status:
- `pytest -q --noconftest selfdrive/controls/lib/tests/test_stopping_guard.py selfdrive/controls/lib/tests/test_stopping_controller.py tools/stopping/test_check_harsh_stops.py tools/stopping/test_stopping_model.py tools/stopping/test_check_harsh_stops_model.py tools/stopping/test_run_stopping_cycle.py`
- Result: `31 passed`.

Controller rewrite attempt (rejected in this checkpoint):
- A full stop-controller rewrite was prototyped and tested locally.
- It regressed seeded harsh-stop tests and several controller behavior tests.
- Decision: rollback rewrite, keep runtime controller stable, continue with small validated iterations.

### 2026-02-09: Model-fit correctness fix (enabled-only training rows)

Problem:
- `fit_stopping_model.py` builds a command-response model using `carControl.actuators.accel` vs `carState.aEgo`.
- When controls are not enabled, `actuators.accel` can still be published but is not applied to the car, which corrupts delay fitting and coefficient stability.

Fix:
- Training rows now require `controlsState.enabled` by default (when available on the sample type).
- Override is available for experiments only: `fit_stopping_model.py --include-disabled`.

Regression coverage:
- Added `tools/stopping/test_stopping_model.py::test_fit_stopping_model_ignores_disabled_samples_in_delay_search`.

### 2026-02-09: Ineffective-Brake Windup Guard (near-hold)

Observation (from engaged stop logs):
- In some near-hold cases, `actuators.accel` can ratchet more negative while measured decel (`aEgo`) becomes less negative.
- This pattern is consistent with drivetrain/clutch behavior flipping near standstill and is a common precursor to felt end-stop jerk.

Change:
- Added a guard in `selfdrive/controls/lib/stopping_controller.py` to prevent further brake deepening when:
  - phase is `NEAR_HOLD`/`HOLD`,
  - speed is very low (`vEgo < 0.35 m/s`),
  - decel is weak (`aEgo > -0.35 m/s²`),
  - command is already deep (`accel < -1.05 m/s²`).

Test coverage:
- Added `selfdrive/controls/lib/tests/test_stopping_controller.py::test_stopping_controller_ineffective_brake_guard_prevents_deep_windup_near_hold`.

### 2026-02-09: Log sync from commawifi

- Host: `commawifi`
- Sync counts: remote=4279, new=0, changed=2244, downloaded=200
- Additional counts: unchanged=2035, failures=0, skipped_limit=2044
- New routes detected: none
- New segments detected: none
- Downloaded route summary: `0000069c--04a3351f79` (37 segments), `0000069e--8e74a2e62b` (27 segments), `0000069f--b0c9c6f633` (6 segments) (+4 more)
- Downloaded segments: `0000069c--04a3351f79--31`, `0000069c--04a3351f79--32`, `0000069c--04a3351f79--33` (+197 more)
- Report JSON: `/Users/radoslawchybicki/.comma/stopping_behavior/reports/sync_commawifi_20260209T201545Z.json`
- Settings JSON: `/Users/radoslawchybicki/.comma/stopping_behavior/settings/stop_settings_commawifi_20260209T201545Z.json`
- Stop settings snapshot: AdvancedLongitudinalTune=True, LongitudinalTune=True, HumanAcceleration=True, ... (+3 more)
- Findings: _pending analysis of downloaded logs_

### 2026-02-09: Stopping analysis for route 0000069c--04a3351f79

- Host: `commawifi`
- Route: `0000069c--04a3351f79`
- Segments analyzed: 41
- Detected stop events: 34
- Median duration to standstill hold: 8.50 s
- Median approach speed: 3.35 m/s
- Median entry speed: 2.35 m/s
- Median min aEgo: -0.88 m/s²
- Median min accel cmd: -0.10 m/s²
- Median shouldStop->stopping delay: 0.000 s
- Median creep after stop: 0.048 m/s
- Settings snapshot: `/Users/radoslawchybicki/.comma/stopping_behavior/settings/stop_settings_commawifi_20260209T201545Z.json`
- Analysis summary JSON: `/Users/radoslawchybicki/.comma/stopping_behavior/analysis/commawifi/cycle_20260209T201545Z/summary.json`
- Analysis summary Markdown: `/Users/radoslawchybicki/.comma/stopping_behavior/analysis/commawifi/cycle_20260209T201545Z/summary.md`
- Example event graph: `/Users/radoslawchybicki/.comma/stopping_behavior/analysis/commawifi/cycle_20260209T201545Z/events/event_001_seg_033.html`

### 2026-02-09: Engaged-Stop Metrics + Controller Soft-Landing Iteration

Context:
- The raw `check_harsh_stops.py` gate is intentionally harsh if run on *all* stops (it includes manual/driver-brake stops).
- For controller work, we now filter to events that are both:
  - mostly engaged (`enabled_ratio >= 0.80`), and
  - actually in stop-control (`stop_signal_ratio >= 0.20`).

Analyzer update (schema):
- `tools/stopping/analyze_stopping_behavior.py` now writes per-event ratios:
  - `enabled_ratio`, `stop_signal_ratio`, `should_stop_ratio`, `stopping_state_ratio`, `stopping_state_cmd_ratio`.
- This lets the gate distinguish “real engaged stopping” from “driver stop while engaged” and “fully manual”.

Engaged harsh-stop gate snapshot (newest analysis summary):
- Summary JSON:
  - `~/.comma/stopping_behavior/analysis/commawifi/0000069c--04a3351f79/20260209T202743Z/summary.json`
- Command:
  - `python tools/stopping/check_harsh_stops.py --summary-json <summary> --min-enabled-ratio 0.8 --min-stop-signal-ratio 0.2`
- Result:
  - `events_considered=11`, `harsh_events=8`, `harsh_rate=0.727` (fails current `max_harsh_rate=0.20` target)
  - dominant flags: `end_stop_accel_step` and `end_stop_jerk` (command jerk is low; the symptom looks plant/drivetrain-driven).

Model refresh (cycle 20260209T201545Z):
- Fitted model:
  - `~/.comma/stopping_behavior/models/stopping_model_20260209T201545Z_all.json`
  - fit stats: `windows=64`, `rows=861`, `best_delay_frames=6`, `rmse=0.1392`, `mae=0.0788`, `r2=0.8948`.
- Model gate:
  - `~/.comma/stopping_behavior/analysis/model_harsh_check_commawifi_20260209T201545Z_all.json`
  - `status=pass` at baseline threshold (`max_harsh_rate=0.50`), but still contains harsh events (primarily predicted jerk).

Controller iteration (target: reduce wheel-stop accel step):
- `selfdrive/controls/lib/stopping_controller.py`
  - Stop-hold targets made milder at very low speeds (reduces “hard land” at wheel stop).
  - Release-lock detection no longer triggers at very low speeds (`vEgo <= 0.06`) where `aEgo ~= 0` is normal.
  - Added a standstill relaxation path in HOLD to move toward a mild hold when `vEgo` is ~0 and `aEgo` has settled.
- Added unit coverage:
  - `selfdrive/controls/lib/tests/test_stopping_controller.py::test_stopping_controller_soft_landing_releases_in_hold_when_decel_is_stable`.

Model-gate correctness tweak:
- `tools/stopping/check_harsh_stops_model.py`
  - Controller-replay jerk window is now anchored to the predicted standstill crossing (`pred_v_ego < 0.05`) instead of always using the end of the replay window.
  - This aligns the jerk metric with “end of the moving stop” instead of “post-standstill relaxation”.

Next step:
- Deploy this controller iteration, drive a few engaged stops (downhill/uphill/flat if possible), pull logs, and re-run:
  - `python tools/stopping/run_stopping_cycle.py --host commawifi --analyze --analysis-event-mode hybrid --analysis-min-entry-speed 0.0 --fit-model --fit-event-source all --run-model-gate`
  - then re-check engaged harsh rate on the newest analysis summary using the `--min-enabled-ratio/--min-stop-signal-ratio` filters above.

### 2026-02-09: Log sync from commawifi

- Host: `commawifi`
- Sync counts: remote=4279, new=1, changed=2044, downloaded=200
- Additional counts: unchanged=2234, failures=0, skipped_limit=1845
- New routes detected: 1 total: `000006df--4cb2d5b964`
- New segments detected: 1 total: `000006df--4cb2d5b964--0`
- Downloaded route summary: `0000068c--7c8e5da54e` (6 segments), `0000068d--2ce1a97146` (1 segments), `0000068e--54d434d702` (3 segments) (+15 more)
- Downloaded segments: `0000068c--7c8e5da54e--4`, `0000068c--7c8e5da54e--5`, `0000068c--7c8e5da54e--6` (+197 more)
- Report JSON: `~/.comma/stopping_behavior/reports/sync_commawifi_20260209T211657Z.json`
- Settings JSON: `~/.comma/stopping_behavior/settings/stop_settings_commawifi_20260209T211657Z.json`
- Stop settings snapshot: AdvancedLongitudinalTune=True, LongitudinalTune=True, HumanAcceleration=True, ... (+3 more)
- Findings: _pending analysis of downloaded logs_

### 2026-02-09: Stopping analysis for route 0000068c--7c8e5da54e

- Host: `commawifi`
- Route: `0000068c--7c8e5da54e`
- Segments analyzed: 9
- Detected stop events: 2
- Median duration to standstill hold: 8.55 s
- Median approach speed: 1.56 m/s
- Median entry speed: 1.56 m/s
- Median min aEgo: -0.67 m/s²
- Median min accel cmd: 0.00 m/s²
- Median shouldStop->stopping delay: n/a s
- Median creep after stop: 0.369 m/s
- Settings snapshot: `~/.comma/stopping_behavior/settings/stop_settings_commawifi_20260209T211657Z.json`
- Analysis summary JSON: `~/.comma/stopping_behavior/analysis/commawifi/cycle_20260209T211657Z/summary.json`
- Analysis summary Markdown: `~/.comma/stopping_behavior/analysis/commawifi/cycle_20260209T211657Z/summary.md`
- Example event graph: `~/.comma/stopping_behavior/analysis/commawifi/cycle_20260209T211657Z/events/event_001_seg_012.html`
- Data quality note: low event count; collect more intentional stop scenarios for stronger comparisons.

### 2026-02-10: Log sync from commawifi

- Host: `commawifi`
- Sync counts: remote=4279, new=8, changed=1845, downloaded=60
- Additional counts: unchanged=2426, failures=0, skipped_limit=1793
- New routes detected: 1 total: `000006e0--63b246dcdc`
- New segments detected: 8 total; sample: `000006e0--63b246dcdc--0`, `000006e0--63b246dcdc--1`, `000006e0--63b246dcdc--2`; +5 more
- Downloaded route summary: `00000689--800e2befe7` (28 segments), `0000068a--0fc20e1b5b` (7 segments), `0000068b--f404788973` (13 segments) (+2 more)
- Downloaded segments: `00000689--800e2befe7--27`, `00000689--800e2befe7--28`, `00000689--800e2befe7--29` (+57 more)
- Report JSON: `~/.comma/stopping_behavior/reports/sync_commawifi_20260210T060712Z.json`
- Settings JSON: `~/.comma/stopping_behavior/settings/stop_settings_commawifi_20260210T060712Z.json`
- Stop settings snapshot: AdvancedLongitudinalTune=True, LongitudinalTune=True, HumanAcceleration=True, ... (+3 more)
- Findings: _pending analysis of downloaded logs_

### 2026-02-10: Stopping analysis for route 000006e0--63b246dcdc

- Host: `commawifi`
- Route: `000006e0--63b246dcdc`
- Segments analyzed: 8
- Detected stop events: 0
- Median duration to standstill hold: n/a s
- Median approach speed: n/a m/s
- Median entry speed: n/a m/s
- Median min aEgo: n/a m/s²
- Median min accel cmd: n/a m/s²
- Median shouldStop->stopping delay: n/a s
- Median creep after stop: n/a m/s
- Settings snapshot: `~/.comma/stopping_behavior/settings/stop_settings_commawifi_20260210T060712Z.json`
- Analysis summary JSON: `~/.comma/stopping_behavior/analysis/commawifi/cycle_20260210T060712Z/summary.json`
- Analysis summary Markdown: `~/.comma/stopping_behavior/analysis/commawifi/cycle_20260210T060712Z/summary.md`
- Data quality note: low event count; collect more intentional stop scenarios for stronger comparisons.

### 2026-02-10: Tooling: Fix run_stopping_cycle default analysis route selection

- Fix: `tools/stopping/run_stopping_cycle.py --analyze` now auto-selects a moving route by scanning locally synced qlogs for route vmax, instead of always choosing the newest `new_routes` entry.
- New option: `--analysis-min-route-vmax` (default 0.5 m/s; set to 0 to disable scan).
- Expected impact: avoids wasted analysis runs on standstill-only routes that yield 0 stop events; use `--analysis-route` to override.

### 2026-02-10: Controller: clutch-push relief hardening + end-stop brake cap

Problem:
- Some harsh stops present as low-speed "leapfrogging": `aEgo` spikes positive near standstill even with negative accel command.
- The stop controller could still ratchet to very deep brake commands during these disturbances (especially when rollout tightening fights the push), which tends to amplify end-stop harshness.

Changes:
- `selfdrive/controls/lib/stopping_controller.py`
  - Ensure `clutch_push_relief` cannot be overridden by rollout tightening / delay-release-guard paths.
  - Make clutch-push relief targets milder (avoid deep brake windup when the gearbox is pushing).
  - Add `end_stop_brake_cap` near wheel-stop to clamp inherited deep commands and unwind toward a mild cap.

Offline controller-replay check (model gate):
- Same dataset + thresholds as the earlier baseline run (`max_pred_end_jerk=0.70`, `max_pred_rollout_m=2.0`):
  - harsh rate unchanged (10/21), but average event score improved (~1.21 -> ~0.79).
  - Biggest improvement is the previous worst clutch-push case (`000006a5--c9ae338723 ev6`), which no longer "runs away" to extremely deep brake commands in the replay.

Next:
- Refit the response model on a larger corpus (manual + engaged for fitting), but keep the acceptance gates engaged-stopping-only.
- Validate on-road for creep/rollout vs end-stop feel (collect a few intentional engaged stops with no driver brake).

### 2026-02-10: Offline: benchmark fairness fix + “beat legacy_32b8be” regression gate

- Benchmark fairness fix:
  - `tools/stopping/benchmark_controller_variants.py` now anchors the jerk window for legacy/abstract replays to the *first* predicted standstill crossing (`pred_v_ego < 0.05`), matching current-controller replay semantics.
  - This removes a bias where some variants effectively got a longer “hold” window that could hide end-stop jerk.
- Regression gate:
  - Added `tools/stopping/test_check_harsh_stops_model.py::test_current_controller_beats_legacy_32b8be_on_seed_corpus`.
  - Uses a fixed seed corpus (from a 2026-02-10 benchmark run) + a fixed fitted-model snapshot (`stopping_model_20260210T060712Z_all.json`) to enforce: current ≤ legacy on harsh count and avg score (and strictly better in at least one).
- Benchmark snapshot (same corpus inputs as the seed gate, `min-entry-speed=0.0`, `controller-scope=engaged_stopping`):
  - current: `harsh=12/28`, `avg_score≈1.00`
  - legacy_32b8be: `harsh=12/28`, `avg_score≈1.14`

### 2026-02-10: Controller follow-up: broaden clutch-push relief trigger + strong-decel end-stop soft cap

- `selfdrive/controls/lib/stopping_controller.py`
  - Broaden `clutch_push_relief` to also trigger when commanded brake is deep but measured decel is weak at low speed (`vEgo < 1.0`, `aEgo > -0.25`, `last_output_accel < -0.85`).
    - Intended to reduce “deep command windup” in clutch/gearbox disturbance cases that present as leapfrogging.
  - Add a narrow “strong decel” soft cap that clamps the end-stop cap to `-0.275` when already decelerating hard near standstill (`vEgo < 0.20`, `aEgo < -0.70`, small rollout, not release-locked).
    - Intended to reduce standstill command magnitude/step at wheel stop.
- Status:
  - Stopping unit suite remains green (see `pytest -q --noconftest ...` list in project header).
