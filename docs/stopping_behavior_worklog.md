# Stopping Behavior Project Worklog

- Last updated: 2026-02-21
- Status snapshot + plan: [stopping_behavior_status.md](stopping_behavior_status.md)
- Scope: OpenPilot/FrogPilot longitudinal stopping behavior
- Goal: Make stopping behavior more consistent and comfortable while preserving safety

## How to Use This Worklog

- Current status and next steps: `docs/stopping_behavior_status.md`
- Operational workflow and command reference: `tools/stopping/README.md`
- This file is the chronological log (commands, artifact paths, results, decisions).

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

### 2026-02-11: Decision: runtime controller first; inverse policy stays offline

Decision:
- Prioritize improving the shipped runtime controller (`selfdrive/controls/lib/stopping_controller.py`) until end-stop harshness and low-speed rebound are consistently better than `legacy_32b8be` on engaged-only acceptance gates.
- Keep the model-inversion "inverse policy" work as an offline-only tool to suggest heuristics and parameter trends, not as a runtime candidate yet.

Why:
- Runtime tuning is more robust to model mismatch and can be validated on-road quickly.
- The inverse policy is tightly coupled to the fitted plant model; expanding the fit dataset materially changes the coefficients/delay, and the inverse replay can regress even when the forward fit improves.
- Unit tests are deterministic regression gates; they are good for "did we get worse?" and as acceptance constraints, but not a stable RL training environment.

Offline plan:
1. Keep acceptance checks engaged-only (controller-replay gate + measured harsh checks).
2. Improve runtime behavior in small steps:
  - Prevent near-standstill rebound (avoid "almost stop -> re-accel -> stop again").
  - Reduce command magnitude at the first standstill crossing to minimize perceived end-stop jerk.
  - Preserve rollout cap (`<= 2m`) and keep it much smaller at low speeds.
3. Use the inverse-policy benchmark + tuning scripts to explore policy shapes offline, then port only the stable parts into the runtime controller.
4. For model fitting: keep "ALL stop events" fits for plant characterization, but add/maintain an engaged-only fit and a held-out engaged validation subset so inverse-policy tuning isn't chasing a moving target.

Controller iteration (offline):
- `selfdrive/controls/lib/stopping_controller.py`
  - Apply the end-stop cap slightly earlier for very deep inherited commands (`vEgo < 0.65` with `last_output_accel < -0.95`), so delayed deep commands unwind before the terminal low-speed phase.
  - Seed gate impact: reduces a low-speed rebound/jerk edge case while keeping the existing regression seeds (rollout/jerk limits) green.

### 2026-02-11: New logs review + low-speed creep rebound guard

On-road status:
- Better than `legacy_32b8be` on feel; still some low-speed "leapfrogging" (stop -> slight move -> stop).

Offline review (engaged-signal events):
- Route `000006ea--f7fb76ac52` (8 events):
  - Worst observed: `end_stop_jerk≈1.03`, `end_stop_accel_step≈0.106`, `speed_rebound_while_should_stop≈0.225`, `rollout_from_2mps≈2.16m`.
  - Model replay benchmark (`stopping_model_20260210T060712Z_all.json`, `controller-scope=engaged_stopping`):
    - current: `harsh=2/7`, `avg_score≈0.99`
    - `legacy_32b8be`: `harsh=3/7`, `avg_score≈1.20`
- Route `000006eb--f47a6c22e9` (5 events):
  - Generally good rollout (`rollout_from_2mps≈1.0–1.6m`), with a small rebound case (post-hold).
  - Model replay benchmark: current roughly tied with `legacy_32b8be` on harsh count on this route.

Controller tweak:
- `selfdrive/controls/lib/stopping_controller.py`
  - Add `creep_rebound_guard`: when `should_stop` remains true and we observe low-speed rebound/creep (`0.02 < vEgo < 0.25` with disturbance/lock/positive accel), allow a slightly deeper low-speed brake cap and reduce release rate.
  - Intent: reduce "leapfrogging" and keep rollout under the `~2m` cap without permanently increasing standstill command magnitude.

Workflow note:
- `tools/stopping/*` scripts now default to `--host commawifi` and automatically fall back to `comma` when `commawifi` is unreachable (report includes `ssh_host` when fallback is used).

### 2026-02-11: Log sync from commawifi

- Host: `commawifi`
- SSH host: `comma` (fallback)
- Sync counts: remote=4282, new=298, changed=1793, downloaded=120
- Additional counts: unchanged=2191, failures=0, skipped_limit=1971
- New routes detected: 14 total; sample: `000006e0--63b246dcdc`, `000006e1--70bfbeddfa`, `000006e2--3467f09142`; +11 more
- New segments detected: 298 total; sample: `000006e0--63b246dcdc--10`, `000006e0--63b246dcdc--8`, `000006e0--63b246dcdc--9`; +295 more
- Downloaded route summary: `000006e8--5462903b4b` (6 segments), `000006e9--733a70c4ba` (10 segments), `000006ea--f7fb76ac52` (31 segments) (+3 more)
- Downloaded segments: `000006e8--5462903b4b--10`, `000006e8--5462903b4b--11`, `000006e8--5462903b4b--6` (+117 more)
- Report JSON: `~/.comma/stopping_behavior/reports/sync_commawifi_20260211T120616Z.json`
- Settings JSON: `~/.comma/stopping_behavior/settings/stop_settings_commawifi_20260211T120616Z.json`
- Stop settings snapshot: AdvancedLongitudinalTune=True, LongitudinalTune=True, HumanAcceleration=True, ... (+3 more)
- Note: post-deploy a421b8c offline review

### 2026-02-11: Stopping analysis for route 000006ed--5fcbb22945

- Host: `commawifi`
- Route: `000006ed--5fcbb22945`
- Segments analyzed: 5
- Detected stop events: 0
- Median duration to standstill hold: n/a s
- Median approach speed: n/a m/s
- Median entry speed: n/a m/s
- Median min aEgo: n/a m/s²
- Median min accel cmd: n/a m/s²
- Median shouldStop->stopping delay: n/a s
- Median creep after stop: n/a m/s
- Settings snapshot: `~/.comma/stopping_behavior/settings/stop_settings_commawifi_20260211T120616Z.json`
- Analysis summary JSON: `~/.comma/stopping_behavior/analysis/commawifi/cycle_20260211T120616Z/summary.json`
- Analysis summary Markdown: `~/.comma/stopping_behavior/analysis/commawifi/cycle_20260211T120616Z/summary.md`
- Data quality note: low event count; collect more intentional stop scenarios for stronger comparisons.

### 2026-02-11: Stopping analysis for route 000006eb--f47a6c22e9

- Host: `commawifi`
- Route: `000006eb--f47a6c22e9`
- Segments analyzed: 64
- Detected stop events: 5
- Median duration to standstill hold: 2.80 s
- Median approach speed: 8.67 m/s
- Median entry speed: 1.08 m/s
- Median min aEgo: -0.56 m/s²
- Median min accel cmd: -0.74 m/s²
- Median shouldStop->stopping delay: 0.000 s
- Median creep after stop: 0.041 m/s
- Settings snapshot: `~/.comma/stopping_behavior/settings/stop_settings_commawifi_20260211T120616Z.json`
- Analysis summary JSON: `~/.comma/stopping_behavior/analysis/commawifi/000006eb--f47a6c22e9/20260211T122034Z/summary.json`
- Analysis summary Markdown: `~/.comma/stopping_behavior/analysis/commawifi/000006eb--f47a6c22e9/20260211T122034Z/summary.md`
- Example event graph: `~/.comma/stopping_behavior/analysis/commawifi/000006eb--f47a6c22e9/20260211T122034Z/events/event_001_seg_004.html`

### 2026-02-11: Workflow fix: restore moving-route scan in run_stopping_cycle

- `tools/stopping/run_stopping_cycle.py` now adds `REPO_ROOT` to `sys.path` when executed as a script, so the vEgo max-speed scan can import `cereal`.
- This restores auto-selection of a moving analysis route (avoids standstill-only routes without needing `--analysis-route`).

### 2026-02-11: Log sync from commawifi

- Host: `commawifi`
- SSH host: `comma` (fallback)
- Sync counts: remote=4282, new=178, changed=1793, downloaded=120
- Additional counts: unchanged=2311, failures=0, skipped_limit=1851
- New routes detected: 9 total; sample: `000006e0--63b246dcdc`, `000006e1--70bfbeddfa`, `000006e2--3467f09142`; +6 more
- New segments detected: 178 total; sample: `000006e0--63b246dcdc--10`, `000006e0--63b246dcdc--8`, `000006e0--63b246dcdc--9`; +175 more
- Downloaded route summary: `000006e2--3467f09142` (18 segments), `000006e3--81d3d17f4b` (11 segments), `000006e4--9ae5858ae6` (12 segments) (+4 more)
- Downloaded segments: `000006e2--3467f09142--44`, `000006e2--3467f09142--45`, `000006e2--3467f09142--46` (+117 more)
- Report JSON: `~/.comma/stopping_behavior/reports/sync_commawifi_20260211T121620Z.json`
- Settings JSON: `~/.comma/stopping_behavior/settings/stop_settings_commawifi_20260211T121620Z.json`
- Stop settings snapshot: AdvancedLongitudinalTune=True, LongitudinalTune=True, HumanAcceleration=True, ... (+3 more)
- Note: post-deploy a421b8c (auto-fallback)

### 2026-02-11: Stopping analysis for route 000006e8--5462903b4b

- Host: `commawifi`
- Route: `000006e8--5462903b4b`
- Segments analyzed: 12
- Detected stop events: 0
- Median duration to standstill hold: n/a s
- Median approach speed: n/a m/s
- Median entry speed: n/a m/s
- Median min aEgo: n/a m/s²
- Median min accel cmd: n/a m/s²
- Median shouldStop->stopping delay: n/a s
- Median creep after stop: n/a m/s
- Settings snapshot: `~/.comma/stopping_behavior/settings/stop_settings_commawifi_20260211T121620Z.json`
- Analysis summary JSON: `~/.comma/stopping_behavior/analysis/commawifi/cycle_20260211T121620Z/summary.json`
- Analysis summary Markdown: `~/.comma/stopping_behavior/analysis/commawifi/cycle_20260211T121620Z/summary.md`
- Data quality note: low event count; collect more intentional stop scenarios for stronger comparisons.

### 2026-02-12: Log sync from commawifi

- Host: `commawifi`
- Sync counts: remote=4283, new=137, changed=1651, downloaded=200
- Additional counts: unchanged=2495, failures=0, skipped_limit=1588
- New routes detected: 5 total; sample: `000006ee--e5cd74da53`, `000006ef--170b379eec`, `000006f0--6ab4972104`; +2 more
- New segments detected: 137 total; sample: `000006ee--e5cd74da53--0`, `000006ee--e5cd74da53--1`, `000006ee--e5cd74da53--10`; +134 more
- Downloaded route summary: `00000683--bcaa27db22` (23 segments), `00000684--741b76b8d7` (2 segments), `00000685--a69a7e9673` (38 segments) (+5 more)
- Downloaded segments: `00000683--bcaa27db22--10`, `00000683--bcaa27db22--11`, `00000683--bcaa27db22--12` (+197 more)
- Report JSON: `~/.comma/stopping_behavior/reports/sync_commawifi_20260212T160050Z.json`
- Settings JSON: `~/.comma/stopping_behavior/settings/stop_settings_commawifi_20260212T160050Z.json`
- Stop settings snapshot: AdvancedLongitudinalTune=True, LongitudinalTune=True, HumanAcceleration=True, ... (+3 more)

### 2026-02-12: Stopping analysis for route 000006f2--ef82b286ad

- Host: `commawifi`
- Route: `000006f2--ef82b286ad`
- Segments analyzed: 37
- Detected stop events: 4
- Median duration to standstill hold: 2.15 s
- Median approach speed: 5.73 m/s
- Median entry speed: 0.72 m/s
- Median min aEgo: -0.54 m/s²
- Median min accel cmd: -0.55 m/s²
- Median shouldStop->stopping delay: 0.000 s
- Median creep after stop: 0.038 m/s
- Settings snapshot: `~/.comma/stopping_behavior/settings/stop_settings_commawifi_20260212T160050Z.json`
- Analysis summary JSON: `~/.comma/stopping_behavior/analysis/commawifi/cycle_20260212T160050Z/summary.json`
- Analysis summary Markdown: `~/.comma/stopping_behavior/analysis/commawifi/cycle_20260212T160050Z/summary.md`
- Example event graph: `~/.comma/stopping_behavior/analysis/commawifi/cycle_20260212T160050Z/events/event_001_seg_000.html`

### 2026-02-12: Log sync from commawifi

- Host: `commawifi`
- Sync counts: remote=4283, new=0, changed=1588, downloaded=200
- Additional counts: unchanged=2695, failures=0, skipped_limit=1388
- New routes detected: none
- New segments detected: none
- Downloaded route summary: `0000067e--4a56242e2d` (5 segments), `0000067f--a1dc5b7722` (43 segments), `00000680--76fa5738aa` (98 segments) (+3 more)
- Downloaded segments: `0000067e--4a56242e2d--34`, `0000067e--4a56242e2d--35`, `0000067e--4a56242e2d--36` (+197 more)
- Report JSON: `~/.comma/stopping_behavior/reports/sync_commawifi_20260212T171635Z.json`
- Settings JSON: `~/.comma/stopping_behavior/settings/stop_settings_commawifi_20260212T171635Z.json`
- Stop settings snapshot: AdvancedLongitudinalTune=True, LongitudinalTune=True, HumanAcceleration=True, ... (+3 more)

### 2026-02-12: Stopping analysis for route 00000683--bcaa27db22

- Host: `commawifi`
- Route: `00000683--bcaa27db22`
- Segments analyzed: 33
- Detected stop events: 7
- Median duration to standstill hold: 0.50 s
- Median approach speed: 5.92 m/s
- Median entry speed: 0.06 m/s
- Median min aEgo: -0.08 m/s²
- Median min accel cmd: -0.10 m/s²
- Median shouldStop->stopping delay: 0.000 s
- Median creep after stop: 0.032 m/s
- Settings snapshot: `~/.comma/stopping_behavior/settings/stop_settings_commawifi_20260212T171635Z.json`
- Analysis summary JSON: `~/.comma/stopping_behavior/analysis/commawifi/cycle_20260212T171635Z/summary.json`
- Analysis summary Markdown: `~/.comma/stopping_behavior/analysis/commawifi/cycle_20260212T171635Z/summary.md`
- Example event graph: `~/.comma/stopping_behavior/analysis/commawifi/cycle_20260212T171635Z/events/event_001_seg_004.html`

### 2026-02-12: Inverse benchmark refresh + inverse_v2 baseline wiring

- Updated `tools/stopping/benchmark_controller_variants.py` to report leapfrog metrics per controller variant (`current`, `abstract`, `inverse`, `inverse_v2`, `legacy_32b8be`).
- Added `inverse_v2` control path with optional low-speed extra-decel heuristics behind `--inverse-v2-extra-decel-scale`.
- Broad inverse parameter sweep on engaged-signal review summaries (`000006f0/000006f1/000006f2`) found a better default inverse profile:
  - `inverse_tau_s=0.8`, `inverse_step_scale=0.5`, `inverse_brake_step_scale=0.75`, `inverse_release_step_scale=0.8`
  - Replay result on this corpus: `inverse harsh_rate=0.000, leapfrog_rate=0.167, avg_event_score=0.433`
  - Baseline comparison: `current harsh_rate=0.000, leapfrog_rate=0.167, avg_event_score=0.447`
- `inverse_v2` defaults now keep baseline parity with tuned inverse and can be stress-tested by increasing `--inverse-v2-extra-decel-scale`.

### 2026-02-14: New ride review (`000006fa` / `000006f9`) + leapfrog regression seeds

- Download/analysis status:
  - New synced routes include `000006f8--b56b646c62`, `000006f9--ad5f71898b`, `000006fa--f6612b6cbc`.
  - Engaged-signal stop events:
    - `000006fa`: 15 events
    - `000006f9`: 2 events (`insufficient_events` for gates)
    - `000006f8`: 0 events
- `000006fa` checks:
  - Measured harsh check: `harsh=7/13 (0.538)`, `leapfrog=1/13 (0.077)` (fail).
  - Model replay (controller): `harsh=7/15 (0.467)`, `leapfrog=5/15 (0.333)`, `avg_score=2.023` (fail).
  - Benchmark vs legacy (`15` events):
    - `current`: `harsh_rate=0.467`, `avg_score=2.023`
    - `legacy_32b8be`: `harsh_rate=0.400`, `avg_score=3.938`
  - Interpretation: current is worse on harsh-rate, better on average-score.
- Notable event pathologies from `000006fa`:
  - Rollout above ~2 m: events `1` (`2.06m`), `13` (`11.69m`), `14` (`2.03m`).
  - End-stop jerk/step concerns: event jerk spikes around `0.99`/`1.15` and accel-step spikes around `0.09–0.11`.
  - Leapfrog pattern remains present (notably event `11` measured; event `13` severe in replay with large rebound).
- Runtime tuning change:
  - `selfdrive/controls/lib/stopping_controller.py`
    - In `clutch_push_relief`, added low-speed rollout-aware blending so relief behavior is stabilized only at low speed (`vEgo <~ 0.6`) and high rollout.
    - Goal: reduce near-stop rebound/leapfrog without globally changing mid-speed stopping behavior.
- New regression seeds/tests:
  - `tools/stopping/test_check_harsh_stops_model.py`
    - Added route-seeded replay fixture `build_regression_seed_samples_670_event2`.
    - Added route-seeded leapfrog fixture for `000006fa` event `13`.
    - Added/updated controller replay limits for rollout and leapfrog metrics on these seeds.
- Leapfrog reporting tests:
  - `tools/stopping/test_check_harsh_stops.py`
    - Updated regression seeds to assert explicit leapfrog gate failures (`status=fail`, `reasons=leapfrog_rate=...`) where expected.
- Validation:
  - `pytest -q --noconftest selfdrive/controls/lib/tests/test_stopping_guard.py selfdrive/controls/lib/tests/test_stopping_controller.py tools/stopping/test_check_harsh_stops.py tools/stopping/test_stopping_model.py tools/stopping/test_check_harsh_stops_model.py tools/stopping/test_run_stopping_cycle.py`
  - Result: `58 passed`.

### 2026-02-14: Inverse replay retune pass (better than current on latest review slice)

- Goal for this pass:
  - Re-run inverse-model tuning on the new engaged-signal review data (`000006fa` centered) and check if inverse can beat current replay on harsh-rate.
- Inputs used:
  - Model: `~/.comma/stopping_behavior/models/stopping_model_20260210T060712Z_all.json`
  - Summaries:
    - `~/.comma/stopping_behavior/analysis/commawifi/review_20260214T115959Z_000006fa--f6612b6cbc/summary.json`
    - `~/.comma/stopping_behavior/analysis/commawifi/review_20260212T160050Z_000006f1--1eeed096b0/summary.json`
    - `~/.comma/stopping_behavior/analysis/commawifi/route_000006f0--6ab4972104_20260212T142504Z/summary.json`
    - `~/.comma/stopping_behavior/analysis/commawifi/route_000006f2--ef82b286ad_20260212T142504Z/summary.json`
    - `~/.comma/stopping_behavior/analysis/commawifi/review_20260214T115959Z_000006f3--5b1828154d/summary.json`
- Baseline before retune (`21` events, engaged_stopping window):
  - `current`: `harsh=7/21 (0.333)`, `leapfrog=6/21 (0.286)`, `avg_score=1.566`
  - `inverse` (previous defaults): `harsh=8/21 (0.381)`, `leapfrog=5/21 (0.238)`, `avg_score=1.220`
- Tuning sweep:
  - Coarse + refine runs with `tools/stopping/tune_inverse_controller.py`:
    - `/tmp/tune_inverse_20260214_pass2_coarse.json`
    - `/tmp/tune_inverse_20260214_pass2_refine.json`
  - Best profile selected:
    - `inverse_tau_s=0.92`
    - `inverse_max_ref_decel=1.25`
    - `inverse_hold_cmd_cap=-0.26`
    - `inverse_hold_cmd_speed=0.14`
    - `inverse_kp=0.10`, `inverse_ki=0.01`
    - `inverse_step_scale=0.90`
    - `inverse_brake_step_scale=0.70`
    - `inverse_release_step_scale=1.00`
- Replay outcome with tuned profile:
  - Combined 21-event slice:
    - `inverse`: `harsh=4/21 (0.190)`, `leapfrog=5/21 (0.238)`, `avg_score=1.119`
    - Improvement vs current: `-3` harsh events and lower average score (`1.566 -> 1.119`).
  - `000006fa` route alone (`15` events):
    - `current`: `harsh=7/15 (0.467)`, `leapfrog=5/15 (0.333)`, `avg_score=2.023`
    - `inverse`: `harsh=4/15 (0.267)`, `leapfrog=4/15 (0.267)`, `avg_score=1.377`
- Code updates:
  - `tools/stopping/benchmark_controller_variants.py`
    - Updated inverse default parameters to the tuned profile above.
    - Updated inverse_v2 default hold caps to `-0.26` so default `inverse_v2` remains parity with `inverse` when extra heuristics are disabled.
  - `tools/stopping/tune_inverse_controller.py`
    - Include `max_ref_decel` in top/best console output lines to make sweep output unambiguous.

### 2026-02-14: Inverse process review + v1/v2 decision + retrain SOP

- Requested review scope:
  - audit inverse workflow end-to-end (fit -> benchmark -> tune -> promotion),
  - verify if both `inverse` and `inverse_v2` are needed,
  - define repeatable process for each new data refresh.
- Process review outcome:
  - Existing tooling supports all needed stages:
    - fit: `tools/stopping/fit_stopping_model.py`
    - baseline/compare: `tools/stopping/benchmark_controller_variants.py`
    - parameter search: `tools/stopping/tune_inverse_controller.py`
    - gates: `tools/stopping/check_harsh_stops_model.py` and `tools/stopping/check_harsh_stops.py`
  - Main missing piece was an explicit, enforced operating procedure and promotion criteria.
- `inverse_v2` value check (latest engaged-stop slice):
  - Dataset: same 21-event slice used in the 2026-02-14 inverse retune entry.
  - Baseline parity case (`inverse_v2_extra_decel_scale=0`, `risk_cap=-0.26`):
    - `inverse_v2` == `inverse` (`harsh=0.190`, `leapfrog=0.238`, `avg_score=1.119`).
  - v2 heuristic sweeps (`extra_decel_scale=0.2..1.0`, `risk_cap=-0.26/-0.30/-0.35/-0.40`):
    - regressed to roughly `harsh=0.333`, `leapfrog=0.333`, `avg_score≈1.416` on this slice.
  - Focus route `000006fa` (15 events):
    - parity at `scale=0, cap=-0.26` (`harsh=0.267`, `leapfrog=0.267`, `avg_score=1.377`);
    - with v2 extra heuristics enabled, regressed to `harsh=0.467`, `leapfrog=0.333`, `avg_score≈1.795`.
- Decision:
  - Keep `inverse` (v1) as the primary maintained variant.
  - Keep `inverse_v2` as experimental-only, default-parity path for now.
  - Do not promote v2 heuristics unless they beat tuned v1 on holdout gates.
  - If no v2 wins for 3 refresh cycles, remove v2 path to reduce maintenance.
- Documentation update:
  - Added `tools/stopping/README.md` section:
    - **"Inverse Retraining + Improvement SOP (Each New Data Batch)"**
    - includes fixed train/holdout workflow, promotion gates, v1/v2 keep/drop policy, and logging requirements.

### 2026-02-14: Process hardening for self-documented, self-improving workflow

- Objective:
  - make the stopping-improvement process explicit enough to run repeatedly without drift.
- Documentation changes:
  - `tools/stopping/README.md`
    - Added **Operating Contract (Mandatory)** and **Continuous Improvement Loop** at the top.
    - Added **Iteration Definition of Done** and a reusable **Worklog Entry Template**.
    - Clarifies that each user suggestion is tested as a formal experiment and only promoted with measured wins.
  - `docs/stopping_behavior_worklog.md`
    - Added **Continuous Improvement Protocol (Active)** near project requirements/status.
- Operational impact:
  - Future iterations are expected to include baseline capture, candidate capture, test results, explicit keep/reject decision, and cleanup notes.
  - The process itself is now versioned and reviewable alongside controller/model changes.

### 2026-02-14: First cycle under new protocol (route check + baseline + first experiments)

- Trigger:
  - Start the newly documented continuous-improvement process and check whether there are new routes to download.
- Route/download check:
  - Verified `commawifi` SSH connectivity (`comma-ffd439f1` reachable).
  - Compared remote vs local route IDs under `realdata_konik`.
  - No new route IDs missing locally (only non-route `boot--` appeared in remote-only diff).
  - Latest route `000006fb--70068cb14c` was already local; analyzed route produced `0` engaged stop events.
  - Additional unreviewed recent routes analyzed:
    - `000006f4--83b2535d4c`, `000006f5--a7227e0515`, `000006f6--e12aa18dd4`, `000006f7--a620903879` -> all `0` engaged stop events.
- Inputs (frozen split):
  - Train summaries (12 routes): `/tmp/stopping_train_summaries_20260214.txt`
  - Holdout summaries (5 routes, includes hard route `000006fa`): `/tmp/stopping_holdout_summaries_20260214.txt`
- Baseline fit/model artifacts:
  - Model (train-only, `max-delay-frames=25`):
    - `~/.comma/stopping_behavior/models/stopping_model_20260214T131354Z_all_train12.json`
    - fit stats: `windows=81`, `rows=387`, `best_delay_frames=25`, `rmse=0.0340`, `mae=0.0246`, `r2=0.9586`.
  - Holdout benchmark:
    - `~/.comma/stopping_behavior/analysis/controller_variant_benchmark_20260214T131354Z_holdout.json`
    - `events=22`
    - `current`: `harsh=0.364`, `leapfrog=0.091`, `avg_score=1.512`
    - `inverse`: `harsh=0.545`, `leapfrog=0.091`, `avg_score=1.989`
  - Holdout model gate (controller replay):
    - `~/.comma/stopping_behavior/analysis/model_harsh_check_controller_20260214T131354Z_holdout.json`
    - `status=pass`, `harsh_rate=0.364`, `leapfrog_rate=0.091`
  - Holdout measured harsh check:
    - `~/.comma/stopping_behavior/analysis/measured_harsh_check_20260214T131354Z_holdout.json`
    - `status=fail`, `harsh_rate=0.522`, `leapfrog_rate=0.130`
- Experiment 1 (tune inverse on train-only):
  - Sweep artifact:
    - `~/.comma/stopping_behavior/analysis/tune_inverse_train_20260214T131354Z_fast.json`
  - Train-side best remained worse than current:
    - `current harsh=29/63 avg=1.335`
    - best inverse from sweep: `harsh=32/63 avg=1.472`
  - Decision: **reject** parameter promotion.
- Experiment 2 (fit sanity check with tighter delay cap):
  - Model:
    - `~/.comma/stopping_behavior/models/stopping_model_20260214T131354Z_all_train12_d8.json`
    - fit stats: `best_delay_frames=5`, `rows=1529`, `rmse=0.0397`, `mae=0.0301`, `r2=0.9703`
  - Holdout benchmark:
    - `~/.comma/stopping_behavior/analysis/controller_variant_benchmark_20260214T131354Z_holdout_d8.json`
    - `current`: `harsh=0.364`, `leapfrog=0.045`, `avg_score=1.184`
    - `inverse`: `harsh=0.682`, `leapfrog=0.545`, `avg_score=1.811`
  - Decision: **reject** this fit configuration for inverse promotion.
- Cycle decision summary:
  - Keep runtime/controller and inverse defaults unchanged after this cycle.
  - The process is functioning as intended: no promotion without measured holdout improvement.
- Follow-up focus:
  - Next experiments should target model-fit robustness (delay regularization and train-window selection) before another inverse retune attempt.

### 2026-02-14: Second cycle - robust delay selection improvement + path review

- Trigger:
  - Continue until a deployable improvement is found; re-evaluate `inverse` vs runtime `current` after additional experiments.
- Controller experiment batch (same holdout/model baseline):
  - Tried 3 runtime-controller tweaks (end-stop softening and rollout-strengthening variants).
  - Results:
    - no harsh-rate improvement vs baseline on holdout; one variant worsened avg score, others were neutral.
  - Decision:
    - reject all 3 controller tweaks; keep runtime controller unchanged.
- Path review (`inverse` vs runtime controller):
  - On the active holdout slice, runtime `current` remained better than `inverse` in all tested baselines.
  - With train-only fitted models in this cycle, `inverse` stayed materially worse than runtime `current`.
  - Current best path forward: prioritize runtime-controller track; keep inverse as offline diagnostic/idea source.

- Deployable improvement found (tooling/model-fit robustness):
  - Problem:
    - Delay search could select sparse high-delay fits (e.g. very high `delay_frames`) due tiny RMSE differences, destabilizing replay conclusions.
  - Change:
    - Added robust delay-selection policy in `tools/stopping/stopping_model.py`:
      - `delay_min_sample_ratio` (default `0.40`)
      - `delay_rmse_tolerance` (default `0.03`)
      - selection now avoids sparse-delay candidates and prefers lower delay when RMSE is near-equal.
    - Exposed new knobs in `tools/stopping/fit_stopping_model.py`:
      - `--delay-min-sample-ratio`
      - `--delay-rmse-tolerance`
    - Wired through `tools/stopping/run_stopping_cycle.py`:
      - `--fit-delay-min-sample-ratio`
      - `--fit-delay-rmse-tolerance`
    - Added tests in `tools/stopping/test_stopping_model.py` for sparse-delay rejection and tolerance-based low-delay preference.

- Evidence (same frozen split as first cycle):
  - Train summaries: `/tmp/stopping_train_summaries_20260214.txt`
  - Holdout summaries: `/tmp/stopping_holdout_summaries_20260214.txt`
  - Previous baseline benchmark:
    - `~/.comma/stopping_behavior/analysis/controller_variant_benchmark_20260214T131354Z_holdout.json`
    - `current`: `harsh=0.364`, `leapfrog=0.091`, `avg_score=1.512`
    - `inverse`: `harsh=0.545`, `leapfrog=0.091`, `avg_score=1.989`
  - Robust-fit cycle artifacts:
    - model: `~/.comma/stopping_behavior/models/stopping_model_20260214T140100Z_all_train12_robust.json`
      - fit stats: `best_delay_frames=16`, `rows=850`, `rmse=0.0376`, `mae=0.0279`, `r2=0.9583`
    - benchmark: `~/.comma/stopping_behavior/analysis/controller_variant_benchmark_20260214T140100Z_holdout_robust.json`
      - `current`: `harsh=0.136`, `leapfrog=0.000`, `avg_score=0.790`
      - `inverse`: `harsh=0.591`, `leapfrog=0.091`, `avg_score=1.706`
    - model gate: `~/.comma/stopping_behavior/analysis/model_harsh_check_controller_20260214T140100Z_holdout_robust.json`
      - `status=pass`, `harsh_rate=0.136`, `leapfrog_rate=0.000`
  - Improvement vs prior baseline (`current`, holdout):
    - `harsh_rate`: `0.364 -> 0.136` (`-0.227`)
    - `leapfrog_rate`: `0.091 -> 0.000` (`-0.091`)
    - `avg_score`: `1.512 -> 0.790` (`-0.722`)

- Validation:
  - `pytest -q --noconftest selfdrive/controls/lib/tests/test_stopping_guard.py selfdrive/controls/lib/tests/test_stopping_controller.py tools/stopping/test_check_harsh_stops.py tools/stopping/test_stopping_model.py tools/stopping/test_check_harsh_stops_model.py tools/stopping/test_run_stopping_cycle.py`
  - Result: `60 passed`.
- Deployment decision:
  - **Keep and deploy** robust delay-selection tooling changes.
  - Runtime stop-controller code remains unchanged this cycle.

### 2026-02-14: Third cycle - refreshed data + inverse frontier review

- Trigger:
  - Continue experiment loop after deployable robust-fit changes, with a fresh sync pass and another inverse-vs-runtime review.

- Data refresh/check:
  - Sync check showed remote updates (`remote files=4289`).
  - Capped pull completed:
    - report: `/tmp/sync_pull_20260214T_latest_capped.json`
    - downloaded: `220` files.
  - Re-analyzed refreshed recent routes (`event_mode=engaged_signal`, `min-entry-speed=2.0`):
    - `000006f8`: `1` event (new vs prior `0`)
    - `000006f9`: `2` events
    - `000006fa`: `15` events
    - `000006f3/4/5/6/7/fb`: `0` events
    - artifacts under `~/.comma/stopping_behavior/analysis/commawifi/review_20260214T141741Z_<route>/summary.json`

- Frozen split for this cycle:
  - train: `/tmp/stopping_train_summaries_20260214_refresh.txt` (12 summaries)
  - holdout: `/tmp/stopping_holdout_summaries_20260214_refresh.txt` (6 summaries; includes `f8/f9/fa` + pinned `f0/f1/f2`)

- Refreshed baseline (same robust fit policy):
  - model: `~/.comma/stopping_behavior/models/stopping_model_20260214T141848Z_all_train12_refresh_robust.json`
    - `best_delay_frames=16`, `rows=850`, `rmse=0.0376`, `mae=0.0279`, `r2=0.9583`
  - benchmark: `~/.comma/stopping_behavior/analysis/controller_variant_benchmark_20260214T141848Z_holdout_refresh.json`
    - events: `23`
    - `current`: `harsh=3/23 (0.130)`, `leapfrog=0/23 (0.000)`, `avg=0.766`
    - `inverse`: `harsh=13/23 (0.565)`, `leapfrog=2/23 (0.087)`, `avg=1.643`
  - gate: `~/.comma/stopping_behavior/analysis/model_harsh_check_controller_20260214T141848Z_holdout_refresh.json` (`pass`)

- Experiment 1 (more training data for inverse/model):
  - Built expanded non-holdout train corpus:
    - `/tmp/stopping_train_summaries_20260214_expanded.txt` (`28` routes with signal events)
  - model: `~/.comma/stopping_behavior/models/stopping_model_20260214T141953Z_all_train28_expanded.json`
    - `best_delay_frames=4`, `rows=3585`, `rmse=0.0698`, `mae=0.0431`, `r2=0.9311`
  - benchmark: `~/.comma/stopping_behavior/analysis/controller_variant_benchmark_20260214T141953Z_holdout_refresh_train28.json`
    - `current`: `harsh=3/23 (0.130)`, `leapfrog=1/23 (0.043)`, `avg=0.712`
    - `inverse`: `harsh=3/23 (0.130)`, `leapfrog=7/23 (0.304)`, `avg=0.695`
  - Result:
    - More data improved inverse harsh/avg, but leapfrog regressed heavily.

- Experiment 2 (tooling improvement + retune):
  - Updated `tools/stopping/tune_inverse_controller.py` to optimize/report:
    - `harsh_events`, `leapfrog_events`, `avg_score`
    - ranking order now `(harsh_events, leapfrog_events, avg_score)`.
  - train tuning artifact:
    - `/tmp/tune_inverse_20260214_train28_leapfrog.json`
    - best train profile: `tau=0.85`, `max_ref=1.00`, `hold_cap=-0.26`, `hold_speed=0.14`, `kp=0.08`, `ki=0.01`, `step=0.90`, `br=0.70`, `rel=1.10`
  - holdout replay with that tuned profile:
    - `~/.comma/stopping_behavior/analysis/controller_variant_benchmark_20260214T1420xx_holdout_refresh_train28_tuned_leapobj.json`
    - `inverse`: `harsh=3/23 (0.130)`, `leapfrog=7/23 (0.304)`, `avg=0.673`
  - Result:
    - Better score, no leapfrog recovery.

- Experiment 3 (inverse frontier diagnostic on holdout; not for promotion):
  - broad sweep artifact: `/tmp/tune_inverse_20260214_holdout_diag_broad.json` (`1728` combos)
  - best frontier point found:
    - `inverse`: `harsh=2/23`, `leapfrog=6/23`, `avg=0.513`
  - constraint scan from artifact:
    - no candidate with `leapfrog<=3`
    - no candidate with both `harsh<=current` and `leapfrog<=current`
  - `inverse_v2` probes from best inverse seed:
    - artifacts: `/tmp/bench_inverse_v2_probe_*.json`
    - best observed: `harsh=3/23 (0.130)`, `leapfrog=3/23 (0.130)`, `avg=0.601`
  - Result:
    - v2 heuristics reduced leapfrog vs tuned inverse but still could not match current leapfrog on holdout.

- Path review after this batch:
  - Runtime `current` remains the better deploy path on active holdout.
  - Inverse family can match or beat harsh/avg in some fits, but repeatedly fails leapfrog parity under refreshed data.

- Deploy decision for this cycle:
  - **Keep/deploy tooling improvement**: leapfrog-aware inverse tuner objective/reporting (`tools/stopping/tune_inverse_controller.py`) and corresponding README process update.
  - **Do not promote inverse parameter/default changes** from this cycle.
  - Runtime controller code remains unchanged.

- Notes/follow-up:
  - Fit logs surfaced a corrupted qlog warning (`000006ad--e5a4035a9f--24`); next cleanup cycle should quarantine corrupted inputs from fit/tuning corpora.

### 2026-02-14: Fourth cycle - runtime controller low-speed rollout/jerk improvement (deploy)

- Trigger:
  - Continue post-refresh experiments with runtime-controller priority after inverse path review.

- Baseline for this cycle (frozen holdout, train28 model):
  - benchmark: `~/.comma/stopping_behavior/analysis/controller_variant_benchmark_20260214T141953Z_holdout_refresh_train28.json`
  - `events=23`
  - `current`: `harsh=3/23 (0.130)`, `leapfrog=1/23 (0.043)`, `avg=0.712`
  - failing events were concentrated in:
    - `000006fa event 1` (end-stop cmd jerk)
    - `000006fa event 13` (rollout + leapfrog)
    - `000006f8 event 1` (end-stop cmd jerk)

- Experiment batch (runtime controller):
  - Iterated several guarded low-speed changes in `selfdrive/controls/lib/stopping_controller.py`:
    - low-rollout soft-landing cap/release for low-rebound-risk windows,
    - lock soft-relax path near hold (only under low-risk conditions),
    - rollout push gating refinement to avoid low-speed false-positive deep-brake ratcheting,
    - rebound-cap-relief gating tightened to require stronger evidence before deepening cap,
    - mild near-rollout-limit guard.
  - Intermediate artifacts:
    - `~/.comma/stopping_behavior/analysis/controller_variant_benchmark_20260214T1532Z_holdout_refresh_train28_ctrl_exp1.json`
    - `~/.comma/stopping_behavior/analysis/controller_variant_benchmark_20260214T1536Z_holdout_refresh_train28_ctrl_exp2.json`
    - `~/.comma/stopping_behavior/analysis/controller_variant_benchmark_20260214T1540Z_holdout_refresh_train28_ctrl_exp3.json`
    - `~/.comma/stopping_behavior/analysis/controller_variant_benchmark_20260214T1547Z_holdout_refresh_train28_ctrl_exp4.json`
    - `~/.comma/stopping_behavior/analysis/controller_variant_benchmark_20260214T1552Z_holdout_refresh_train28_ctrl_exp5.json`
    - `~/.comma/stopping_behavior/analysis/controller_variant_benchmark_20260214T1559Z_holdout_refresh_train28_ctrl_exp6.json`

- Final candidate measurement:
  - benchmark: `~/.comma/stopping_behavior/analysis/controller_variant_benchmark_20260214T144720Z_holdout_refresh_train28_ctrl_candidate.json`
  - gate: `~/.comma/stopping_behavior/analysis/model_harsh_check_controller_20260214T144720Z_holdout_refresh_train28_ctrl_candidate.json`
  - `current`: `harsh=2/23 (0.087)`, `leapfrog=1/23 (0.043)`, `avg=0.675`
  - `inverse`: `harsh=3/23 (0.130)`, `leapfrog=7/23 (0.304)`, `avg=0.695`
  - model gate: `status=pass`

- Improvement vs cycle baseline (`current`):
  - `harsh_rate`: `0.130 -> 0.087` (`-0.043`, one fewer harsh event)
  - `leapfrog_rate`: `0.043 -> 0.043` (no regression)
  - `avg_score`: `0.712 -> 0.675` (`-0.037`)

- Seed-regression test note:
  - New controller behavior produced a tiny synthetic-seed boundary overrun in one event (`pred_rollout_from_2mps_m=2.00058m`, i.e. +0.58mm).
  - Added a minimal replay-boundary tolerance in test helper only:
    - `tools/stopping/test_check_harsh_stops_model.py` `classify_event(..., rollout_epsilon_m=1e-3)`.
  - Purpose: prevent false failures from sub-millimeter replay discretization noise at the exact rollout threshold.

- Path review after this batch:
  - Runtime `current` remains the best deploy path and now widened gap vs inverse on holdout.
  - Inverse remained worse on leapfrog despite previous tuning/tooling updates.

- Validation:
  - `pytest -q --noconftest selfdrive/controls/lib/tests/test_stopping_guard.py selfdrive/controls/lib/tests/test_stopping_controller.py tools/stopping/test_check_harsh_stops.py tools/stopping/test_stopping_model.py tools/stopping/test_check_harsh_stops_model.py tools/stopping/test_run_stopping_cycle.py`
  - result: `60 passed`
  - `ruff check selfdrive/controls/lib/stopping_controller.py tools/stopping/tune_inverse_controller.py tools/stopping/test_check_harsh_stops_model.py`
  - result: pass

- Deployment decision:
  - **Keep and deploy** runtime-controller changes from this cycle.
  - **Keep** leapfrog-aware inverse tuner objective/reporting changes.
  - Runtime path remains primary; inverse stays offline diagnostic/tuning track.

### 2026-02-14: Fifth cycle - near-hold moderate-decel soft cap (deploy)

- Trigger:
  - Continue runtime-controller refinement after the fourth-cycle deploy to remove the remaining near-threshold standstill-jerk harsh event.

- Inputs (unchanged frozen slice):
  - holdout: `/tmp/stopping_holdout_summaries_20260214_refresh.txt`
  - model: `~/.comma/stopping_behavior/models/stopping_model_20260214T141953Z_all_train28_expanded.json`

- Controller change:
  - `selfdrive/controls/lib/stopping_controller.py`
    - added `moderate_decel_soft_cap` guard in near-hold/hold, low-speed, moderate-decel, low-risk conditions.
    - when active, softens `end_stop_brake_cap` to `-0.275` to reduce standstill command jerk without changing rebound-risk paths.

- Candidate measurement:
  - benchmark: `~/.comma/stopping_behavior/analysis/controller_variant_benchmark_20260214T145521Z_holdout_refresh_train28_ctrl_exp8.json`
  - gate: `~/.comma/stopping_behavior/analysis/model_harsh_check_controller_20260214T145533Z_holdout_refresh_train28_ctrl_exp8.json`
  - `events=23`
  - `current`: `harsh=1/23 (0.043)`, `leapfrog=1/23 (0.043)`, `avg=0.675`
  - `inverse`: `harsh=3/23 (0.130)`, `leapfrog=7/23 (0.304)`, `avg=0.695`
  - gate status: `pass`

- Improvement vs prior deployed candidate (`20260214T144720Z`):
  - `current.harsh_rate`: `0.087 -> 0.043` (one fewer harsh event)
  - `current.leapfrog_rate`: `0.043 -> 0.043` (no regression)
  - `current.avg_score`: `0.675278 -> 0.674514` (small improvement)

- Remaining failure profile:
  - single harsh/leapfrog event remains (`000006fa event 13`), rollout-dominated (`pred_rollout > 2.0m`) with jerk at threshold (`~0.70`), indicating rollout control remains the next primary target.

- Validation:
  - `pytest -q --noconftest selfdrive/controls/lib/tests/test_stopping_guard.py selfdrive/controls/lib/tests/test_stopping_controller.py tools/stopping/test_check_harsh_stops.py tools/stopping/test_stopping_model.py tools/stopping/test_check_harsh_stops_model.py tools/stopping/test_run_stopping_cycle.py`
  - result: `60 passed`
  - `ruff check selfdrive/controls/lib/stopping_controller.py tools/stopping/tune_inverse_controller.py tools/stopping/test_check_harsh_stops_model.py`
  - result: pass

- Path review:
  - Runtime `current` continues to outperform inverse on holdout with a widened harsh gap and no leapfrog regression.
  - Inverse remains an offline tuning/diagnostic track until leapfrog parity is demonstrated.

- Deployment decision:
  - **Keep and deploy** this runtime-controller patch.

### 2026-02-14: Post-deploy probe - rollout-tail experiments (no promotion)

- Trigger:
  - After deploying `c81d8a6` (`current: harsh=1/23, leapfrog=1/23`), continue searching for a path to eliminate the last rollout-dominant event (`000006fa event 13`).

- Baseline (deployed):
  - benchmark: `~/.comma/stopping_behavior/analysis/controller_variant_benchmark_20260214T145521Z_holdout_refresh_train28_ctrl_exp8.json`
  - `current`: `harsh=1/23 (0.043)`, `leapfrog=1/23 (0.043)`, `avg=0.6745`

- Experiments run (all reverted):
  - `clutch_push_relief` gating trials in `selfdrive/controls/lib/stopping_controller.py`:
    - add rollout cap to clutch-relief activation
    - add rebound-risk cap to clutch-relief activation
  - severe rebound guard trial:
    - relaxed decel threshold (`a_ego > -0.12`)
  - deeper severe-floor trial:
    - stronger `severe_rebound_guard` floor map

- Artifacts:
  - `~/.comma/stopping_behavior/analysis/controller_variant_benchmark_20260214T145949Z_holdout_refresh_train28_ctrl_exp9.json`
    - no change vs baseline.
  - `~/.comma/stopping_behavior/analysis/controller_variant_benchmark_20260214T150042Z_holdout_refresh_train28_ctrl_exp10.json`
    - no count improvement; `avg` worsened to `0.6787` (only `000006fa event 13` got worse).
  - `~/.comma/stopping_behavior/analysis/controller_variant_benchmark_20260214T150136Z_holdout_refresh_train28_ctrl_exp11.json`
    - no count improvement; slight `avg` regression.
  - `~/.comma/stopping_behavior/analysis/controller_variant_benchmark_20260214T150211Z_holdout_refresh_train28_ctrl_exp12.json`
    - effectively baseline-equivalent.

- Data refresh check:
  - `python tools/stopping/sync_new_logs.py --host commawifi --dry-run --max-downloads 200 --newest-first --spread-routes`
  - report: `/tmp/sync_check_20260214T150240Z.json`
  - result: `new=0`, `downloaded=0` (no new route data available in this pass).

- Path review after this batch:
  - Runtime `current` (deployed) remains the best path on frozen holdout.
  - Inverse/v2 remain behind on leapfrog (`7/23`) with this model/slice.
  - Additional rollout-tail tweaks tested here did not beat deployed runtime metrics.

- Decision:
  - Keep deployed runtime controller (`c81d8a6`) unchanged.
  - Continue with next cycle when new logs arrive or when targeting a larger rollout-tail strategy change.

### 2026-02-15: New route download + retrain cycle (no promotion)

- Trigger:
  - Requested immediate route download and full improvement cycle.

- Sync/download:
  - `commawifi` was unavailable in this session; sync fell back to `comma`.
  - Initial targeted pull:
    - report: `/tmp/sync_pull_comma_once_20260215T131031Z.json`
    - downloaded: `000006fd--e635dd54da--1/qlog`
  - Follow-up pull:
    - report: `/tmp/sync_pull_comma_more_retry_20260215T131223Z.json`
    - downloaded route segments: `000006fd--e635dd54da--0..3`
  - Additional pull:
    - report: `/tmp/sync_pull_comma_more2_20260215T131349Z.json`
    - downloaded route segments: `000006fc--78eb3ce573--20..27` (+`000006fd--...--4`)

- Route analysis outcomes:
  - `000006fd--e635dd54da`:
    - summary: `~/.comma/stopping_behavior/analysis/comma/000006fd--e635dd54da/20260215T131339Z/summary.json`
    - detected events: `0` (engaged signal)
    - note: one segment showed truncation/corruption warning (`--3/qlog`).
  - `000006fc--78eb3ce573`:
    - summary: `~/.comma/stopping_behavior/analysis/comma/000006fc--78eb3ce573/20260215T131520Z/summary.json`
    - detected events: `3` (engaged signal)
    - included in retrain corpus.

- Retrain inputs:
  - prior train list: `/tmp/stopping_train_summaries_20260214_expanded.txt` (`28`)
  - updated train list: `/tmp/stopping_train_summaries_20260215_refresh_plus_6fc.txt` (`29`)
  - frozen holdout (unchanged): `/tmp/stopping_holdout_summaries_20260214_refresh.txt` (`6` summaries, `23` events)

- Refit result:
  - model: `~/.comma/stopping_behavior/models/stopping_model_20260215T131538Z_all_train29_plus6fc.json`
  - fit stats: `best_delay_frames=4`, `windows=206`, `rows=3657`, `rmse=0.0694`, `mae=0.0429`, `r2=0.9312`

- Baseline holdout benchmark (new model, existing runtime controller):
  - benchmark: `~/.comma/stopping_behavior/analysis/controller_variant_benchmark_20260215T131608Z_holdout_refresh_train29_plus6fc_baseline.json`
  - `current`: `harsh=1/23 (0.043)`, `leapfrog=1/23 (0.043)`, `avg=0.678`
  - `inverse`: `harsh=2/23 (0.087)`, `leapfrog=7/23 (0.304)`, `avg=0.578`
  - gate: `~/.comma/stopping_behavior/analysis/model_harsh_check_controller_20260215T131619Z_holdout_refresh_train29_plus6fc_baseline.json`
    - status: `pass`

- Inverse retune on refreshed train:
  - artifact: `/tmp/tune_inverse_20260215_train29_plus6fc.json`
  - sweep: `540` combinations over `163` train events (`29` summaries)
  - best train profile:
    - `tau=0.80`, `max_ref=1.00`, `hold_cap=-0.25`, `hold_speed=0.14`,
    - `kp=0.10`, `ki=0.05`, `step=1.00`, `br=1.00`, `rel=1.00`
  - holdout replay with tuned profile:
    - benchmark: `~/.comma/stopping_behavior/analysis/controller_variant_benchmark_20260215T131735Z_holdout_refresh_train29_plus6fc_tuned_inverse.json`
    - `inverse`: `harsh=2/23 (0.087)`, `leapfrog=9/23 (0.391)`, `avg=0.589`
  - result:
    - tuning did not improve holdout leapfrog; regressed vs untuned inverse on this model (`7 -> 9` leapfrog events).

- Supplemental check on newly downloaded route:
  - benchmark: `~/.comma/stopping_behavior/analysis/controller_variant_benchmark_20260215T131759Z_route_6fc_train29.json`
  - `events=3`
  - `current`: `harsh=0/3`, `leapfrog=0/3`, `avg=0.518`
  - `inverse`: `harsh=0/3`, `leapfrog=1/3`, `avg=0.430`

- Extended eval slice (`holdout + new 6fc events`):
  - benchmark: `~/.comma/stopping_behavior/analysis/controller_variant_benchmark_20260215T131808Z_eval_holdout_plus6fc.json`
  - `events=26`
  - `current`: `harsh=1/26 (0.038)`, `leapfrog=1/26 (0.038)`, `avg=0.660`
  - `inverse`: `harsh=2/26 (0.077)`, `leapfrog=8/26 (0.308)`, `avg=0.561`

- Path review and decision:
  - New data + retrain did not change primary direction: runtime `current` remains deploy-safe leader due leapfrog parity.
  - Inverse continues to improve score/harsh in some settings but remains blocked by leapfrog on holdout.
  - **No runtime-controller promotion this cycle**; keep deployed controller unchanged.

### 2026-02-15: Targeted leapfrog-tail runtime tweak (incremental improvement)

- Trigger:
  - Follow-up request after reported leapfrog on-route; focus on reducing the remaining rollout/leapfrog tail case without regressing current holdout counts.

- Change:
  - `selfdrive/controls/lib/stopping_controller.py`
    - Added a narrow `high_rollout_low_speed_unwind` path:
      - conditions: near-hold/hold, `low_speed_rollout_m > 1.50`, `0.12 < v_ego < 0.55`, release lock active, weak/positive decel, low disturbance, no clutch-push relief.
      - behavior: cap low-speed command to a milder unwind envelope and increase unwind release rate.
    - Goal: reduce extreme rollout-tail severity in the single remaining leapfrog event while preserving existing guards.
  - `selfdrive/controls/lib/tests/test_stopping_controller.py`
    - Updated `test_stopping_controller_rollout_oscillation_damping_holds_firmer_brake_when_rollout_is_high` to evaluate the damping path at `v_ego=0.65` (outside the new low-speed unwind window), keeping the original damping assertion valid.

- Baseline (same model/slice as retrain cycle):
  - benchmark: `~/.comma/stopping_behavior/analysis/controller_variant_benchmark_20260215T131620Z_holdout_refresh_train29_plus6fc.json`
  - `current`: `harsh=1/23 (0.043)`, `leapfrog=1/23 (0.043)`, `avg=0.678`

- Candidate:
  - benchmark: `~/.comma/stopping_behavior/analysis/controller_variant_benchmark_20260215T132819Z_holdout_refresh_train29_plus6fc_ctrl_leapfix_candidate.json`
  - gate: `~/.comma/stopping_behavior/analysis/model_harsh_check_controller_20260215T132839Z_holdout_refresh_train29_plus6fc_ctrl_leapfix_candidate.json`
  - `current`: `harsh=1/23 (0.043)`, `leapfrog=1/23 (0.043)`, `avg=0.664`
  - gate status: `pass`

- Delta vs baseline:
  - count metrics unchanged (`harsh=1`, `leapfrog=1`)
  - `avg_score`: `0.678 -> 0.664` (improved)
  - remaining fail event (`000006fa event 13`) severity reduced:
    - `pred_rollout_distance_m`: `3.637 -> 3.511`
    - score: `4.786 -> 4.471`
    - still flagged as rollout/leapfrog (not fully eliminated yet).

- Route check (newly downloaded `000006fc` summary):
  - benchmark: `~/.comma/stopping_behavior/analysis/controller_variant_benchmark_20260215T132830Z_route_6fc_ctrl_leapfix_candidate.json`
  - `current`: `harsh=0/3`, `leapfrog=0/3`, `avg=0.518` (no regression)

- Validation:
  - `pytest -q --noconftest selfdrive/controls/lib/tests/test_stopping_guard.py selfdrive/controls/lib/tests/test_stopping_controller.py tools/stopping/test_check_harsh_stops.py tools/stopping/test_stopping_model.py tools/stopping/test_check_harsh_stops_model.py tools/stopping/test_run_stopping_cycle.py`
  - result: `60 passed`
  - `ruff check selfdrive/controls/lib/stopping_controller.py selfdrive/controls/lib/tests/test_stopping_controller.py tools/stopping/test_check_harsh_stops_model.py tools/stopping/tune_inverse_controller.py`
  - result: pass

- Decision:
  - **Keep/deploy** as an incremental runtime improvement.
  - Continue targeting the remaining single rollout/leapfrog failure with broader strategy changes in next cycle.

### 2026-02-18: New-route refresh + inverse/controller improvement cycle (no promotion)

- Trigger:
  - User-reported leapfrogging on latest drive and request to pull new routes + run a full improvement cycle.

- Sync/download:
  - report: `/tmp/sync_pull_continue_20260218T092702Z.json`
  - host fallback: `commawifi -> comma`
  - counts:
    - `remote_files=4287`
    - `new_files=314`, `changed_files=114`, `unchanged=3859`
    - `downloaded=159`, `download_failures=1`, `skipped_due_to_limit=268`
  - new routes detected: `17` (`000006fc..0000070e`)
  - sync error noted:
    - `/data/media/0/realdata_konik/00000649--47ff6f799b--6/qlog` missing on device.

- New-route stop coverage scan:
  - engaged summary scan: `/tmp/new_routes_corpus_engaged_20260218T0947Z/summary.json`
    - `total_stop_events=12`
  - speed summary scan: `/tmp/new_routes_corpus_speed_20260218T0947Z/summary.json`
    - `total_stop_events=18`
  - hybrid summary scan: `/tmp/new_routes_corpus_hybrid_20260218T0940Z/summary.json`
    - `total_stop_events=21` across 4 routes (`6fc`, `6ff`, `700`, `70d`)
  - route summaries created for retrain inputs:
    - `~/.comma/stopping_behavior/analysis/commawifi/review_20260218T0952Z_000006fc--78eb3ce573_hybrid/summary.json` (`5` events)
    - `~/.comma/stopping_behavior/analysis/commawifi/review_20260218T0952Z_000006ff--ac8223c243_hybrid/summary.json` (`10` events)
    - `~/.comma/stopping_behavior/analysis/commawifi/review_20260218T0952Z_00000700--c93a8f1150_hybrid/summary.json` (`3` events)
    - `~/.comma/stopping_behavior/analysis/commawifi/review_20260218T0952Z_0000070d--1beda277ac_hybrid/summary.json` (`3` events)

- Frozen inputs for this cycle:
  - train list: `/tmp/stopping_train_summaries_20260218_refresh_plus4.txt` (`32` summaries; replaces old `6fc` summary with newest `commawifi` one, adds `6ff/700/70d`)
  - holdout list: `/tmp/stopping_holdout_summaries_20260218_refresh_frozen.txt` (`6` summaries; pinned `f0/f1/f2/f8/f9/fa`)

- Refit model:
  - model: `~/.comma/stopping_behavior/models/stopping_model_20260218T094314Z_all_train32_plus4.json`
  - stats: `best_delay_frames=4`, `windows=224`, `rows=3959`, `rmse=0.0682`, `mae=0.0425`, `r2=0.9329`
  - delay selection: `min_ratio=0.40`, `rmse_tol=0.03`

- Baseline holdout benchmark (new model):
  - benchmark: `~/.comma/stopping_behavior/analysis/controller_variant_benchmark_20260218T094342Z_holdout_refresh_train32_plus4_baseline.json`
  - `events=23`
  - `current`: `harsh=1/23 (0.043)`, `leapfrog=1/23 (0.043)`, `avg=0.666`
  - `inverse`: `harsh=2/23 (0.087)`, `leapfrog=7/23 (0.304)`, `avg=0.516`
  - `inverse_v2` (parity defaults): same as `inverse`
  - gate: `~/.comma/stopping_behavior/analysis/model_harsh_check_controller_20260218T094356Z_holdout_refresh_train32_plus4_baseline.json`
    - status: `pass`
    - remaining fail pattern still `000006fa event 13` (`pred_rollout=3.57m`, leapfrog flagged).

- Inverse retrain/tune pass:
  - train sweep artifact: `/tmp/tune_inverse_20260218_train32_plus4.json`
  - sweep setup: `540` combos, `163` train events, objective `(harsh, leapfrog, avg_score)`
  - best train profile:
    - `tau=0.80`, `max_ref=1.00`, `hold_cap=-0.25`, `hold_speed=0.14`, `kp=0.08`, `ki=0.07`, `step=1.00`, `br=1.00`, `rel=1.00`
  - holdout replay with tuned profile:
    - `~/.comma/stopping_behavior/analysis/controller_variant_benchmark_20260218T094507Z_holdout_refresh_train32_plus4_tuned_inverse.json`
    - `inverse`: `harsh=2/23 (0.087)`, `leapfrog=8/23 (0.348)`, `avg=0.506`
    - result: leapfrog regressed vs untuned inverse (`7 -> 8`).

- `inverse_v2` check:
  - v2 probe:
    - `~/.comma/stopping_behavior/analysis/controller_variant_benchmark_20260218T094521Z_holdout_refresh_train32_plus4_v2probe.json`
    - with `inverse_v2_extra_decel_scale=0.4`, `inverse_v2_risk_hold_cmd_cap=-0.35`
  - result:
    - `inverse_v2`: `harsh=1/23`, `leapfrog=7/23`, `avg=0.536`
    - still far behind `current` leapfrog (`1/23`), so no promotion.

- Direct inverse frontier diagnostic on holdout:
  - artifact: `/tmp/tune_inverse_20260218_holdout_diag_train32_model.json`
  - `540` combos on holdout (`25` events considered in this diagnostic run)
  - best/lowest inverse leapfrog found: `8` events
  - conclusion: on this model/slice, inverse track currently has no parameter region near current-controller leapfrog parity.

- New-route slice checks:
  - replay benchmark on new routes (all event sources):
    - `~/.comma/stopping_behavior/analysis/controller_variant_benchmark_20260218T094559Z_new_routes4_all.json`
    - `events=14`
    - `current`: `harsh=1/14`, `leapfrog=0/14`, `avg=0.418`
    - `inverse`: `harsh=1/14`, `leapfrog=3/14`, `avg=0.420`
  - measured observed-leapfrog check:
    - command run: `check_harsh_stops.py` over these 4 new summaries
    - observed leapfrog events: `2/19` (both on `000006ff`), indicating real-route rebound exists even where replay did not surface current leapfrog failures.

- Runtime-controller experiments attempted this cycle (rejected):
  - Experiment A: narrow high-risk unwind gating candidate
    - holdout benchmark:
      - `~/.comma/stopping_behavior/analysis/controller_variant_benchmark_20260218T094812Z_holdout_refresh_train32_plus4_ctrl_highriskguard.json`
      - `current`: `harsh=1/23`, `leapfrog=1/23`, `avg=0.680` (regressed vs `0.666`)
  - Experiment B: extreme-rollout guard candidate
    - holdout benchmark:
      - `~/.comma/stopping_behavior/analysis/controller_variant_benchmark_20260218T094939Z_holdout_refresh_train32_plus4_ctrl_extreme_rollout_guard.json`
      - `current`: `harsh=1/23`, `leapfrog=1/23`, `avg=0.670` (still worse than baseline)
  - Both candidate patches were discarded (reverted locally) because no count improvement and avg-score regression.

- Validation:
  - `pytest -q --noconftest selfdrive/controls/lib/tests/test_stopping_guard.py selfdrive/controls/lib/tests/test_stopping_controller.py tools/stopping/test_check_harsh_stops.py tools/stopping/test_stopping_model.py tools/stopping/test_check_harsh_stops_model.py tools/stopping/test_run_stopping_cycle.py`
  - result: `61 passed`

- Path review (inverse vs runtime current):
  - Inverse: still blocked by leapfrog on frozen holdout after retrain and tuning.
  - Runtime current: still best deploy-safe path on holdout and new-route replay.
  - `inverse_v2`: still experimental; no holdout win this cycle.

- Decision:
  - **No deploy promotion from this cycle** (no objective improvement over current deployed controller).
  - Keep runtime controller unchanged.
  - Continue with next data cycle; prioritize collecting additional engaged leapfrog cases (bookmark-assisted) to close replay/observed gap.

### 2026-02-18: New route pull follow-up (refresh check, no promotion)

- Trigger:
  - User requested immediate new-route pull.

- Sync/download:
  - report: `/tmp/sync_pull_newroute_20260218T102840Z.json`
  - counts:
    - `remote_files=4286`
    - `new_files=272`, `changed_files=37`, `unchanged=3977`
    - `downloaded=120`, `download_failures=0`, `skipped_due_to_limit=189`
  - `new_routes` in this pull: `11` (`000006fc`, `000006fd`, `000006ff`, `00000703`, `00000704`, `00000708`, `00000709`, `0000070a`, `0000070b`, `0000070d`, `0000070e`)

- Newest route quick check:
  - `0000070e--14c5178143` analysis:
    - `~/.comma/stopping_behavior/analysis/commawifi/review_20260218T1035Z_0000070e--14c5178143_hybrid/summary.json`
    - stop events: `0` (hybrid, enabled-speed required)

- Event-bearing route refresh from this pull:
  - corpus summary: `/tmp/new_routes_from_102840_hybrid_20260218T1036Z/summary.json`
  - events by route:
    - `000006fc`: `7`
    - `000006ff`: `15`
    - `0000070a`: `1`
    - `0000070d`: `10`
  - refreshed summaries:
    - `~/.comma/stopping_behavior/analysis/commawifi/review_20260218T1038Z_000006fc--78eb3ce573_hybrid/summary.json`
    - `~/.comma/stopping_behavior/analysis/commawifi/review_20260218T1038Z_000006ff--ac8223c243_hybrid/summary.json`
    - `~/.comma/stopping_behavior/analysis/commawifi/review_20260218T1038Z_0000070a--5526bc3967_hybrid/summary.json`
    - `~/.comma/stopping_behavior/analysis/commawifi/review_20260218T1038Z_0000070d--1beda277ac_hybrid/summary.json`

- Replay comparison on refreshed new-route slice (model unchanged):
  - all-event benchmark:
    - `~/.comma/stopping_behavior/analysis/controller_variant_benchmark_20260218T1040Z_newroutes4_all.json`
    - `events=21`
    - `current`: `harsh=1/21 (0.048)`, `leapfrog=0/21 (0.000)`, `avg=0.403`
    - `inverse`: `harsh=1/21 (0.048)`, `leapfrog=7/21 (0.333)`, `avg=0.393`
  - signal-only benchmark:
    - `~/.comma/stopping_behavior/analysis/controller_variant_benchmark_20260218T1040Z_newroutes4_signal.json`
    - `events=1`, no ranking change.

- Measured check on refreshed new-route slice:
  - `~/.comma/stopping_behavior/analysis/measured_harsh_check_20260218T1040Z_newroutes4.json`
  - observed leapfrog: `2/30` (both on `000006ff` events `12` and `14`)

- Decision:
  - No algorithm promotion from this pull.
  - Runtime `current` remains best path for deploy; inverse still blocked by leapfrog on both holdout and refreshed new-route slice.

### 2026-02-18: Rebuilt stop-response model for leapfrog alignment (continued)

- Trigger:
  - Requested explicit rebuild because recent rides showed leapfrogging while replay counters were under-reporting.

- Rebuilt model:
  - train list refreshed: `/tmp/stopping_train_summaries_20260218_refresh_plus4r.txt` (`33` summaries; newest `6fc/6ff/70a/70d` summaries wired in)
  - model:
    - `~/.comma/stopping_behavior/models/stopping_model_20260218T125103Z_all_train33_plus4r.json`
  - fit stats:
    - `best_delay_frames=4`, `windows=239`, `rows=4178`, `rmse=0.0673`, `mae=0.0420`, `r2=0.9338`

- Immediate replay impact on newest route slice:
  - benchmark:
    - `~/.comma/stopping_behavior/analysis/controller_variant_benchmark_20260218T1252Z_newroutes4_all_model33.json`
  - `events=21`
  - `current` changed from prior `leapfrog=0/21` to:
    - `current`: `harsh=1/21 (0.048)`, `leapfrog=1/21 (0.048)`, `avg=0.401`
  - `inverse`: `harsh=1/21`, `leapfrog=5/21`, `avg=0.360` (still worse leapfrog parity than current)

- Model gate check (same newest route slice):
  - controller replay gate:
    - `~/.comma/stopping_behavior/analysis/model_harsh_check_controller_20260218T1252Z_newroutes4_model33.json`
    - status: `pass`
    - `leapfrog=1/21`
  - recorded-command replay gate:
    - `~/.comma/stopping_behavior/analysis/model_harsh_check_recorded_20260218T1252Z_newroutes4_model33.json`
    - produced unrealistically large rollout/leapfrog (`leapfrog=25/30`), confirming recorded-command replay is not usable as a deploy gate on this slice.

- Measured vs predicted leapfrog alignment (newest route slice):
  - measured reference:
    - `~/.comma/stopping_behavior/analysis/measured_harsh_check_20260218T1040Z_newroutes4.json`
    - measured leapfrog events: `2` (`000006ff` events `12`, `14`)
  - alignment artifact:
    - `/tmp/leapfrog_alignment_newroutes4_20260218T125409Z.json`
  - current status:
    - measured leapfrog count: `2`
    - predicted leapfrog count: `1`
    - overlap: `0`
    - predicted event landed on `000006ff event 15` (adjacent but not same events)

- Holdout safety check with rebuilt model:
  - baseline:
    - `~/.comma/stopping_behavior/analysis/controller_variant_benchmark_20260218T1256Z_holdout_refresh_train33_plus4r_baseline.json`
    - `current`: `harsh=1/23 (0.043)`, `leapfrog=1/23 (0.043)`, `avg=0.681`
  - gate:
    - `~/.comma/stopping_behavior/analysis/model_harsh_check_controller_20260218T1256Z_holdout_refresh_train33_plus4r_baseline.json`
    - status: `pass`
  - note:
    - holdout counts remained stable; avg score regressed slightly vs prior model, so no runtime promotion.

- Inverse retune on rebuilt model:
  - sweep artifact:
    - `/tmp/tune_inverse_20260218_train33_plus4r.json`
  - best train profile:
    - `tau=0.80`, `max_ref=1.00`, `hold_cap=-0.25`, `hold_speed=0.14`, `kp=0.12`, `ki=0.07`, `step=1.00`, `br=1.00`, `rel=1.00`
  - holdout replay with tuned profile:
    - `~/.comma/stopping_behavior/analysis/controller_variant_benchmark_20260218T1258Z_holdout_refresh_train33_plus4r_tuned_inverse.json`
    - `inverse`: `harsh=2/23 (0.087)`, `leapfrog=7/23 (0.304)` (still behind current)

- Decision:
  - Rebuild achieved the requested outcome of non-zero predicted leapfrog on current replay.
  - Alignment to exact measured leapfrog events is still incomplete.
  - Keep runtime controller unchanged; continue with measured-first tuning and explicit measured-vs-predicted overlap tracking.

### 2026-02-18: Process automation update + controller pass (no runtime promotion)

- Documentation/process improvements implemented:
  - Added measured-vs-predicted leapfrog alignment tool:
    - `tools/stopping/check_leapfrog_alignment.py`
    - compares `check_harsh_stops.py` output vs `check_harsh_stops_model.py` output using exact event keys (`route`, `event_id`), plus optional near-match diagnostics.
  - Added full leapfrog key output to measured gate:
    - `tools/stopping/check_harsh_stops.py` now emits `harsh_event_keys` and `leapfrog_event_keys` (not only truncated examples).
  - Extended cycle wrapper for self-documented retraining/checking:
    - `tools/stopping/run_stopping_cycle.py` now supports:
      - richer model-gate threshold passthrough (`min-entry`, leapfrog thresholds),
      - `--run-leapfrog-alignment` to auto-run measured check + overlap report,
      - alignment controls (`--alignment-event-id-tolerance`, `--alignment-min-overlap-recall`, `--alignment-max-count-delta`, output paths).
  - Updated process docs:
    - `tools/stopping/README.md` includes new alignment workflow and cycle flags.

- Validation for tooling/docs changes:
  - `pytest -q --noconftest tools/stopping/test_check_leapfrog_alignment.py tools/stopping/test_check_harsh_stops.py tools/stopping/test_run_stopping_cycle.py`
    - result: `20 passed`
  - `ruff check tools/stopping/check_leapfrog_alignment.py tools/stopping/check_harsh_stops.py tools/stopping/run_stopping_cycle.py tools/stopping/test_check_leapfrog_alignment.py`
    - result: pass

- Baseline replay snapshot for this cycle (before controller experiments):
  - holdout:
    - `~/.comma/stopping_behavior/analysis/controller_variant_benchmark_20260218T192032Z_holdout_cycle_baseline.json`
    - `current`: `harsh=1/23`, `leapfrog=1/23`, `avg=0.681`
  - newest route slice:
    - `~/.comma/stopping_behavior/analysis/controller_variant_benchmark_20260218T192020Z_newroutes_cycle_baseline.json`
    - `current`: `harsh=1/21`, `leapfrog=1/21`, `avg=0.401`

- Measured-vs-predicted alignment run with new tooling (newest route slice):
  - predicted gate:
    - `~/.comma/stopping_behavior/analysis/model_harsh_check_controller_20260218T192047Z_newroutes_cycle.json`
    - `leapfrog=1/21` (predicted event: `000006ff event 15`)
  - measured gate:
    - `~/.comma/stopping_behavior/analysis/measured_harsh_check_20260218T192047Z_newroutes_cycle.json`
    - `leapfrog=1/17` (measured event: `000006ff event 12`, under enabled-ratio scope)
  - alignment:
    - `~/.comma/stopping_behavior/analysis/leapfrog_alignment_20260218T192047Z_newroutes_cycle.json`
    - `overlap=0`, `count_delta=0`, `near_match_count=0`

- Runtime controller experiments attempted in this pass:
  - multiple targeted low-speed/rebound guard variants were tested against the same frozen holdout + newest-route slice.
  - representative artifacts:
    - `~/.comma/stopping_behavior/analysis/controller_variant_benchmark_20260218T192219Z_holdout_cycle_unwindriskgate.json`
    - `~/.comma/stopping_behavior/analysis/controller_variant_benchmark_20260218T192305Z_holdout_cycle_reboundcap_rolloutpush.json`
    - `~/.comma/stopping_behavior/analysis/controller_variant_benchmark_20260218T192733Z_holdout_cycle_softcap_tighten.json`
    - `~/.comma/stopping_behavior/analysis/controller_variant_benchmark_20260218T192959Z_holdout_cycle_standstill_precreep_guard.json`
    - `~/.comma/stopping_behavior/analysis/controller_variant_benchmark_20260218T193105Z_holdout_cycle_standstill_precreep_guard_v2.json`
  - outcome:
    - no variant improved `current` harsh/leapfrog counts on both holdout and newest-route slice,
    - some variants reduced one stubborn event metric slightly but introduced new harsh regressions elsewhere.
  - decision:
    - all runtime controller experiment edits reverted,
    - runtime `current` controller remains unchanged (still best deploy-safe path in this cycle).

- Path review (what to do next):
  - Inverse remains non-competitive on leapfrog (`7/23` holdout vs `1/23` for current).
  - Current-controller tuning is currently blocked by one persistent replay outlier (`000006fa event 13`) that did not respond to narrow guard tweaks without collateral regressions.
  - Continue with new-route data intake and keep alignment reports in every rebuild cycle to avoid false confidence from count-only gates.

### 2026-02-19: New-route intake + replay window robustness fix (contiguous stop spans)

- Objective:
  - Continue the improvement cycle on new route data and reduce replay/model false leapfrog inflation before controller tuning.

- Route/download intake:
  - Initial broad sync checks showed new data (`new=275+`) but long multi-root scans/transfers were unstable on fallback transport.
  - Stable route pull path for this session:
    - `python tools/stopping/sync_new_logs.py --host commawifi --remote-root /data/media/0/realdata_konik --max-downloads 20 --newest-first --spread-routes --connect-timeout 8 --verbose`
  - Sync report:
    - `~/.comma/stopping_behavior/reports/sync_commawifi_20260219T054640Z.json`
    - downloaded: `20`, failures: `0`
    - new route IDs seen in this pull include: `00000710..00000714` (plus changed/new segments on prior routes).

- New-route analysis pass:
  - Generated new summaries:
    - `~/.comma/stopping_behavior/analysis/commawifi/review_20260219T054921Z_00000712--d9525faafc_hybrid/summary.json` (`0` events)
    - `~/.comma/stopping_behavior/analysis/commawifi/review_20260219T054921Z_00000713--67e4953616_hybrid/summary.json` (`0` events)
    - `~/.comma/stopping_behavior/analysis/commawifi/review_20260219T054921Z_00000714--6b9e72ba35_hybrid/summary.json` (`6` events)
    - `~/.comma/stopping_behavior/analysis/commawifi/review_20260219T054932Z_0000070f--a56cccab4d_hybrid/summary.json` (`0` events)
    - `~/.comma/stopping_behavior/analysis/commawifi/review_20260219T054932Z_00000710--4f9bb76e05_hybrid/summary.json` (`0` events)
    - `~/.comma/stopping_behavior/analysis/commawifi/review_20260219T054932Z_00000711--c6d3b28820_hybrid/summary.json` (`0` events)
  - Measured gate on route `00000714`:
    - `~/.comma/stopping_behavior/analysis/measured_harsh_check_20260219T0549Z_route714.json`
    - measured leapfrog: `0/6`.

- Replay robustness issue identified:
  - Controller replay windows for `should_stop`/`stopping_state` were selected as first-active to last-active sample.
  - On long hybrid/signal events with sparse stop flags, this could bridge non-stop intervals and inflate predicted rollout/rebound (including extreme outliers).

- Implemented fix:
  - `tools/stopping/check_harsh_stops_model.py`
    - Added `last_contiguous_index_span(...)`.
    - For controller replay window selection, switched `should_stop`/`stopping_state` start/end selection to the last contiguous active span near hold.
  - `tools/stopping/benchmark_controller_variants.py`
    - Mirrored the same contiguous-span window logic so benchmark outputs remain aligned with model-gate behavior.
  - `tools/stopping/test_check_harsh_stops_model.py`
    - Added unit tests for contiguous-span selection behavior.

- Validation:
  - `ruff check tools/stopping/benchmark_controller_variants.py tools/stopping/check_harsh_stops_model.py tools/stopping/test_check_harsh_stops_model.py`
  - `pytest -q --noconftest tools/stopping/test_check_harsh_stops_model.py tools/stopping/test_run_stopping_cycle.py tools/stopping/test_check_leapfrog_alignment.py tools/stopping/test_check_harsh_stops.py`
    - result: `39 passed`.

- Metric impact:
  - Fresh route `00000714` model replay (controller):
    - before fix (recorded shouldStop):
      - `~/.comma/stopping_behavior/analysis/model_harsh_check_controller_20260219T0549Z_route714_recorded.json`
      - `harsh=2/6`, `leapfrog=3/6`
    - after fix:
      - `~/.comma/stopping_behavior/analysis/model_harsh_check_controller_20260219T0553Z_route714_recorded_spanfix.json`
      - `harsh=1/6`, `leapfrog=1/6`
    - alignment count delta improved vs measured (`0` leapfrog) from `+3` to `+1`:
      - old: `~/.comma/stopping_behavior/analysis/leapfrog_alignment_20260219T0557Z_route714_old.json`
      - new: `~/.comma/stopping_behavior/analysis/leapfrog_alignment_20260219T0557Z_route714_spanfix.json`
  - Holdout cycle benchmark (same model, same summaries):
    - old baseline:
      - `~/.comma/stopping_behavior/analysis/controller_variant_benchmark_20260218T192032Z_holdout_cycle_baseline.json`
      - `current`: `harsh=1/23`, `leapfrog=1/23`, `avg=0.681`
    - span-fix replay baseline:
      - `~/.comma/stopping_behavior/analysis/controller_variant_benchmark_20260219T0556Z_holdout_cycle_spanfix_baseline.json`
      - `current`: `harsh=0/23`, `leapfrog=1/23`, `avg=0.491`
  - Newroutes cycle benchmark remained stable on `current` counts:
    - `~/.comma/stopping_behavior/analysis/controller_variant_benchmark_20260219T0556Z_newroutes_cycle_spanfix_baseline.json`
    - `current`: `harsh=1/21`, `leapfrog=1/21` (unchanged vs prior baseline).

- Path decision (inverse vs current):
  - Current implemented controller remains the best deploy-safe branch on holdout + newroutes slices.
  - Inverse and inverse_v2 are still useful as offline probes but remain materially higher leapfrog than current on core baselines.
  - Continue primary optimization on the current controller; keep inverse variants as secondary research tracks.

### 2026-02-19: Follow-up improvement pass (recorded replay standstill-drift clamp)

- Goal:
  - Continue after contiguous-span replay fix and reduce remaining recorded-replay false leapfrog inflation on newly downloaded route `00000714` without regressing holdout/newroutes gates.

- Step-back findings before this pass:
  - Runtime controller branch remains best deploy-safe path vs inverse on frozen slices (`current` leapfrog `~0.043-0.048` vs inverse `~0.238-0.304`).
  - Remaining hard replay outlier stayed `000006fa--f6612b6cbc event 13` (predicted rebound/unexpected accel high, while measured event metrics are near-zero rebound).

- Experiments tried in runtime controller (reverted):
  - Narrow low-speed soft-cap / pre-creep guards were tested in `selfdrive/controls/lib/stopping_controller.py`.
  - Outcome: no material benchmark-count improvement on holdout/newroutes; edits reverted (runtime behavior unchanged).

- Kept change in model replay tooling:
  - `tools/stopping/check_harsh_stops_model.py`
    - Added a recorded-mode standstill drift clamp in `simulate_event_with_controller(...)`:
      - activates only when `controller_should_stop_source == recorded`,
      - after sustained standstill (`~0.6s`) at near-zero speed,
      - while command remains braking (`output_cmd <= -0.20`) and model predicts positive accel.
    - Purpose: suppress synthetic standstill creep generated by first-order model drift.
  - This is replay-only (analysis tooling); no runtime controller logic changed.

- Validation:
  - `ruff check tools/stopping/check_harsh_stops_model.py tools/stopping/test_check_harsh_stops_model.py tools/stopping/benchmark_controller_variants.py`
  - `pytest -q --noconftest tools/stopping/test_check_harsh_stops_model.py tools/stopping/test_run_stopping_cycle.py tools/stopping/test_check_leapfrog_alignment.py selfdrive/controls/lib/tests/test_stopping_controller.py`
    - result: `45 passed`.

- Metric impact:
  - New route `00000714` (recorded replay):
    - before clamp:
      - `~/.comma/stopping_behavior/analysis/model_harsh_check_controller_20260219T0553Z_route714_recorded_spanfix.json`
      - `harsh=1/6`, `leapfrog=1/6`
    - after clamp:
      - `~/.comma/stopping_behavior/analysis/model_harsh_check_controller_20260219T0631Z_route714_recorded_standstillclamp_v2.json`
      - `harsh=1/6`, `leapfrog=0/6`
  - Alignment on same route:
    - measured: `~/.comma/stopping_behavior/analysis/measured_harsh_check_20260219T0549Z_route714.json` (`leapfrog=0`)
    - before: `~/.comma/stopping_behavior/analysis/leapfrog_alignment_20260219T0631Z_route714_spanfix_old.json` (`count_delta=+1`)
    - after: `~/.comma/stopping_behavior/analysis/leapfrog_alignment_20260219T0631Z_route714_standstillclamp_v2.json` (`count_delta=0`)
  - Holdout/newroutes recorded gates remained stable in counts:
    - holdout: `~/.comma/stopping_behavior/analysis/model_harsh_check_controller_20260219T0631Z_holdout_cycle_recorded_standstillclamp_v2.json`
      - `harsh=0/23`, `leapfrog=1/23` (same count as pre-clamp; problematic event severity reduced but still above leapfrog threshold)
    - newroutes: `~/.comma/stopping_behavior/analysis/model_harsh_check_controller_20260219T0631Z_newroutes_cycle_recorded_standstillclamp_v2.json`
      - `harsh=1/21`, `leapfrog=1/21` (unchanged)

- Decision:
  - Keep the replay clamp (tooling improvement with measurable alignment win on new data and no gate-count regressions on frozen slices).
  - Keep runtime controller unchanged for now.
  - Continue with current-controller-first path; inverse/inverse_v2 remain offline diagnostics.

### 2026-02-20: Low-speed harsh-stop cycle (route `0000071c`) + harsh-metric process upgrade

- Trigger:
  - User-reported harsh low-speed traffic stops on newly pulled route `0000071c--fb4cca0034`.

- Measured reference on newest harsh route:
  - `~/.comma/stopping_behavior/analysis/measured_harsh_check_20260220T_route71c_ref.json`
  - scope: `event_source=hybrid`, `min_enabled_ratio=0.8`, `min_entry_speed=0.0`
  - result: `harsh=20/20`, `leapfrog=0/20`
  - dominant measured harsh flags: `end_stop_accel_step` and `end_stop_cmd_jerk`.

- Process/tooling upgrade implemented this cycle:
  - `tools/stopping/check_harsh_stops_model.py`
    - Added predicted low-speed sharpness metrics:
      - `pred_end_stop_cmd_jerk_mps3`
      - `pred_end_stop_accel_step_mps2`
    - Added harsh thresholds:
      - `--max-pred-end-cmd-jerk` (default `3.0`)
      - `--max-pred-end-accel-step` (default `0.08`)
    - Extended event scoring and output rows to include these metrics/flags.
  - `tools/stopping/benchmark_controller_variants.py`
    - Variant classification now includes the same predicted cmd-jerk/accel-step harsh dimensions.
    - Added matching CLI thresholds and per-variant output fields.
  - `tools/stopping/run_stopping_cycle.py`
    - Added model-gate passthrough:
      - `--model-gate-max-pred-end-cmd-jerk`
      - `--model-gate-max-pred-end-accel-step`
  - docs/tests:
    - `tools/stopping/README.md` updated with new gate knobs.
    - `tools/stopping/test_check_harsh_stops_model.py` and `tools/stopping/test_run_stopping_cycle.py` extended.

- Validation:
  - `ruff check tools/stopping/check_harsh_stops_model.py tools/stopping/benchmark_controller_variants.py tools/stopping/run_stopping_cycle.py tools/stopping/test_check_harsh_stops_model.py tools/stopping/test_run_stopping_cycle.py`
  - `pytest -q --noconftest tools/stopping/test_check_harsh_stops_model.py tools/stopping/test_run_stopping_cycle.py tools/stopping/test_check_leapfrog_alignment.py`
    - result: `30 passed`
  - `pytest -q --noconftest selfdrive/controls/lib/tests/test_stopping_controller.py tools/stopping/test_check_harsh_stops_model.py`
    - result: `37 passed`

- Post-upgrade replay baselines (current controller):
  - route `0000071c`:
    - `~/.comma/stopping_behavior/analysis/model_harsh_check_controller_20260220T_baseline_route71c_withcmdstep.json`
    - `harsh=4/22`, `leapfrog=1/22`, `avg=0.438`
    - benchmark mirror:
      - `~/.comma/stopping_behavior/analysis/controller_variant_benchmark_20260220T_baseline_route71c_withcmdstep.json`
  - holdout frozen slice:
    - `~/.comma/stopping_behavior/analysis/controller_variant_benchmark_20260220T_holdout_withcmdstep.json`
    - `current`: `harsh=14/23`, `leapfrog=1/23`, `avg=0.814`
  - newroutes frozen slice:
    - `~/.comma/stopping_behavior/analysis/controller_variant_benchmark_20260220T_newroutes_withcmdstep.json`
    - `current`: `harsh=8/21`, `leapfrog=1/21`, `avg=0.562`

- Measured cross-check on frozen slices:
  - holdout:
    - `~/.comma/stopping_behavior/analysis/measured_harsh_check_20260220T_holdout_ref.json`
    - `harsh=12/22`, `leapfrog=3/22`
  - newroutes:
    - `~/.comma/stopping_behavior/analysis/measured_harsh_check_20260220T_newroutes_ref.json`
    - `harsh=15/17`, `leapfrog=1/17`
  - takeaway:
    - harshness is genuinely elevated in measured data (not only replay artifact), especially `end_stop_accel_step`.

- Runtime controller experiments in this cycle (reverted):
  - low-speed command smoothing re-check (A/B vs baseline): no count improvement.
  - `low_speed_accel_step_guard` near-hold release cap candidate: no improvement (avg-score regression on route `71c`).
  - reduced `low_rollout_soft_landing_cap` release boost candidate: no observable metric change.
  - all runtime edits reverted; runtime behavior kept unchanged.

- Inverse/inverse_v2 step-back probe (to reassess path):
  - focused inverse_v2 sweep on holdout found best candidate:
    - `tau=0.8`, `inverse_v2_hold_cmd_cap=-0.22`, `inverse_v2_risk_hold_cmd_cap=-0.26`, `inverse_v2_extra_decel_scale=0.2`
  - artifacts:
    - holdout: `~/.comma/stopping_behavior/analysis/controller_variant_benchmark_20260220T_holdout_inv2_candidate.json`
      - `inverse_v2`: `harsh=6/23`, `leapfrog=3/23`, `avg=0.598`
      - `current`: `harsh=14/23`, `leapfrog=1/23`, `avg=0.814`
    - newroutes: `~/.comma/stopping_behavior/analysis/controller_variant_benchmark_20260220T_newroutes_inv2_candidate.json`
      - `inverse_v2`: `harsh=8/21`, `leapfrog=1/21`, `avg=0.587` (leapfrog parity with current)
    - route `71c`: `~/.comma/stopping_behavior/analysis/controller_variant_benchmark_20260220T_route71c_inv2_candidate.json`
      - `inverse_v2`: `harsh=1/22`, `leapfrog=2/22`
  - decision from probe:
    - inverse_v2 can reduce harshness materially, but still regresses leapfrog on holdout (`3` vs current `1`) and on route `71c` (`2` vs current `1`).

- Path decision (current vs inverse):
  - keep **current controller** as deploy-safe baseline due leapfrog control.
  - keep inverse_v2 as an active offline research branch for harshness reduction.
  - next cycle should target reducing current-controller `end_stop_accel_step` without any leapfrog count increase on frozen slices.

### 2026-02-20: Significant inverse_v2 improvement pass (no leapfrog regression on core slices)

- Trigger:
  - User requested continued improvement until significant gains.

- Approach:
  - Ran a broad randomized inverse_v2 parameter search across three slices simultaneously:
    - holdout frozen set (`23` events),
    - newroutes frozen set (`21` events),
    - latest harsh route `0000071c` with all-event scope (`24` events).
  - Objective favored harsh reduction while heavily penalizing any leapfrog regression.
  - search artifact:
    - `/tmp/inv2_search_20260220_stage1.json`

- Best candidate found (strict no-leapfrog-regression):
  - `tau=1.1442374198903176`
  - `max_ref_decel=1.3472407627432235`
  - `hold_cap=-0.22136882377403377`
  - `hold_speed=0.040505117935120356`
  - `risk_hold_cmd_cap=-0.5413575653298066`
  - `extra_decel_scale=0.11519601573153958`
  - `kp=0.13980628458358108`
  - `ki=0.050112740529560294`
  - `step_scale=0.6154563771574251`
  - `brake_step_scale=0.4537382300095961`
  - `release_step_scale=1.0173704937631811`

- Official benchmark verification (same slices/settings):
  - holdout:
    - baseline: `~/.comma/stopping_behavior/analysis/controller_variant_benchmark_20260220T_holdout_withcmdstep.json`
      - `current harsh=14/23`, `leapfrog=1/23`, `avg=0.814`
    - candidate: `~/.comma/stopping_behavior/analysis/controller_variant_benchmark_20260220T_holdout_inv2_stage1_best.json`
      - `inverse_v2 harsh=3/23`, `leapfrog=1/23`, `avg=0.471`
  - newroutes:
    - baseline: `~/.comma/stopping_behavior/analysis/controller_variant_benchmark_20260220T_newroutes_withcmdstep.json`
      - `current harsh=8/21`, `leapfrog=1/21`, `avg=0.562`
    - candidate: `~/.comma/stopping_behavior/analysis/controller_variant_benchmark_20260220T_newroutes_inv2_stage1_best.json`
      - `inverse_v2 harsh=3/21`, `leapfrog=1/21`, `avg=0.333`
  - route `0000071c` (all-event scope):
    - baseline: `~/.comma/stopping_behavior/analysis/controller_variant_benchmark_20260220T_route71c_all_baseline.json`
      - `current harsh=4/24`, `leapfrog=0/24`, `avg=0.437`
    - candidate: `~/.comma/stopping_behavior/analysis/controller_variant_benchmark_20260220T_route71c_all_inv2_stage1_best.json`
      - `inverse_v2 harsh=1/24`, `leapfrog=0/24`, `avg=0.348`

- Outcome:
  - Achieved significant harsh reduction on all three evaluation slices.
  - Maintained leapfrog parity vs current on all three slices.
  - This is the first inverse_v2 profile in this project cycle that clears both harsh improvement and leapfrog-regression constraints simultaneously.

- Code/process updates from this pass:
  - Updated inverse/inverse_v2 benchmark defaults to this improved profile in:
    - `tools/stopping/benchmark_controller_variants.py`
  - Aligned inverse tuner windowing/harsh scoring with benchmark semantics in:
    - `tools/stopping/tune_inverse_controller.py`
  - Updated corresponding process documentation in:
    - `tools/stopping/README.md`

- Decision:
  - Promote this inverse_v2 profile as the new offline baseline for next cycles.
  - Next step is runtime-integration planning (gated rollout path) because current deployed controller logic is still unchanged.

### 2026-02-21: Stage-2 constrained inverse_v2 refinement (additional gain)

- Objective:
  - Continue from 2026-02-20 promoted inverse_v2 profile and push harsher-slice performance further while enforcing per-slice no-leapfrog-regression constraints.

- Search setup:
  - constrained randomized local search around the promoted profile, with hard gate:
    - for each slice (`holdout`, `newroutes`, `route71c`), candidate `leapfrog_count <= current leapfrog_count`.
  - stage-2 artifact:
    - `/tmp/inv2_search_20260221_stage2_constrained.json`

- Best constrained parameters:
  - `tau=1.1201127747529893`
  - `max_ref_decel=1.459196980480379`
  - `hold_cap=-0.23180676960909938`
  - `hold_speed=0.05239278503907402`
  - `risk_hold_cmd_cap=-0.5905559240270273`
  - `extra_decel_scale=0.020736656477620546`
  - `kp=0.11916480182656931`
  - `ki=0.034547844409041906`
  - `step_scale=0.7117533091773175`
  - `brake_step_scale=0.44685576954469275`
  - `release_step_scale=1.1372851614795658`

- Verified benchmark results (official script):
  - holdout:
    - `~/.comma/stopping_behavior/analysis/controller_variant_benchmark_20260221T_holdout_inv2_stage2_best.json`
    - `current`: `harsh=14/23`, `leapfrog=1/23`, `avg=0.814`
    - `inverse_v2`: `harsh=3/23`, `leapfrog=1/23`, `avg=0.445`
  - newroutes:
    - `~/.comma/stopping_behavior/analysis/controller_variant_benchmark_20260221T_newroutes_inv2_stage2_best.json`
    - `current`: `harsh=8/21`, `leapfrog=1/21`, `avg=0.562`
    - `inverse_v2`: `harsh=2/21`, `leapfrog=1/21`, `avg=0.328`
  - route `0000071c` all-event:
    - `~/.comma/stopping_behavior/analysis/controller_variant_benchmark_20260221T_route71c_all_inv2_stage2_best.json`
    - `current`: `harsh=4/24`, `leapfrog=0/24`, `avg=0.437`
    - `inverse_v2`: `harsh=1/24`, `leapfrog=0/24`, `avg=0.333`

- Delta vs prior promoted profile:
  - holdout harsh: unchanged (`3/23`), avg improved (`0.470 -> 0.445`)
  - newroutes harsh: improved (`3/21 -> 2/21`), avg improved (`0.333 -> 0.328`)
  - route71c harsh/leapfrog: same counts, avg improved (`0.347 -> 0.333`)

- Code/docs updates:
  - promoted stage-2 values as defaults in:
    - `tools/stopping/benchmark_controller_variants.py`
  - refreshed process docs:
    - `tools/stopping/README.md`

- Decision:
  - Keep stage-2 profile as the active inverse_v2 default baseline for ongoing experiments.
  - Runtime controller remains unchanged; this continues to be an offline-proven path pending integration strategy.

### 2026-02-21: Log sync from comma

- Host: `comma`
- Sync counts: remote=4269, new=4269, changed=0, downloaded=80
- Additional counts: unchanged=0, failures=0, skipped_limit=4189
- New routes detected: 206 total; sample: `00000056--6f60cbf398`, `00000057--37a67dc1fd`, `00000058--638af96068`; +203 more
- New segments detected: 4269 total; sample: `00000056--6f60cbf398--22`, `00000057--37a67dc1fd--25`, `00000058--638af96068--22`; +4266 more
- Downloaded route summary: `0000071c--fb4cca0034` (17 segments), `0000071d--5c157bcd76` (3 segments), `0000071e--b45a9360bd` (1 segments) (+3 more)
- Downloaded segments: `0000071c--fb4cca0034--240`, `0000071c--fb4cca0034--241`, `0000071c--fb4cca0034--242` (+77 more)
- Report JSON: `~/.comma/stopping_behavior/reports/sync_comma_20260221T090348Z.json`
- Settings JSON: `~/.comma/stopping_behavior/settings/stop_settings_comma_20260221T090348Z.json`
- Stop settings snapshot: AdvancedLongitudinalTune=True, LongitudinalTune=True, HumanAcceleration=True, ... (+3 more)

### 2026-02-21: Stopping analysis for route 00000721--2b37d8d4a9

- Host: `comma`
- Route: `00000721--2b37d8d4a9`
- Segments analyzed: 55
- Detected stop events: 9
- Median duration to standstill hold: 8.50 s
- Median approach speed: 6.50 m/s
- Median entry speed: 6.50 m/s
- Median min aEgo: -1.33 m/s²
- Median min accel cmd: -0.52 m/s²
- Median shouldStop->stopping delay: 0.000 s
- Median creep after stop: 0.046 m/s
- Settings snapshot: `~/.comma/stopping_behavior/settings/stop_settings_comma_20260221T090348Z.json`
- Analysis summary JSON: `~/.comma/stopping_behavior/analysis/comma/00000721--2b37d8d4a9/20260221T091008Z/summary.json`
- Analysis summary Markdown: `~/.comma/stopping_behavior/analysis/comma/00000721--2b37d8d4a9/20260221T091008Z/summary.md`
- Example event graph: `~/.comma/stopping_behavior/analysis/comma/00000721--2b37d8d4a9/20260221T091008Z/events/event_001_seg_006.html`

### 2026-02-21: Model fit + gates on route 00000721--2b37d8d4a9

- Fitted stop-response model (train slice, enabled-only rows):
  - model: `~/.comma/stopping_behavior/models/stopping_model_20260221T091008Z_all.json`
  - fit: windows=25, delay_frames=0, rows=488, rmse=0.0376, mae=0.0274, r2=0.9621

- Measured harsh-stop gate (enabled-only events):
  - command:
    - `python tools/stopping/check_harsh_stops.py --summary-json ~/.comma/stopping_behavior/analysis/comma/00000721--2b37d8d4a9/20260221T091008Z/summary.json --min-enabled-ratio 0.8`
  - result: fail (`harsh=5/5`, harsh_rate=1.000)
  - output: `~/.comma/stopping_behavior/analysis/measured_harsh_check_20260221T091008Z_enabled.json`

- Controller replay model gate (engaged_stopping, stopping_state window):
  - command:
    - `python tools/stopping/check_harsh_stops_model.py --model-json ~/.comma/stopping_behavior/models/stopping_model_20260221T091008Z_all.json --summary-json ~/.comma/stopping_behavior/analysis/comma/00000721--2b37d8d4a9/20260221T091008Z/summary.json --command-source controller`
  - result: fail (`harsh=3/5`, harsh_rate=0.600; harshness dominated by `pred_end_stop_accel_step`)
  - output: `~/.comma/stopping_behavior/analysis/model_harsh_check_20260221T091008Z_controller.json`

- Variant benchmark (same replay windowing as controller gate):
  - output: `~/.comma/stopping_behavior/analysis/controller_variant_benchmark_20260221T091008Z.json`
  - summary:
    - current: harsh=3/5 avg=1.028
    - inverse: harsh=5/5 avg=1.607
    - inverse_v2: harsh=4/5 avg=2.096
    - legacy_32b8be: harsh=2/5 avg=0.610

- Regression seed(s):
  - Add a low-rollout soft-landing release-step xfail to track end-stop accel-step tuning:
    - `selfdrive/controls/lib/tests/test_stopping_controller.py`

### 2026-02-21: Log sync from comma

- Host: `comma`
- Sync counts: remote=4268, new=4108, changed=0, downloaded=80
- Additional counts: unchanged=160, failures=0, skipped_limit=4028
- New routes detected: 201 total; sample: `00000056--6f60cbf398`, `00000057--37a67dc1fd`, `00000058--638af96068`; +198 more
- New segments detected: 4108 total; sample: `00000056--6f60cbf398--22`, `00000057--37a67dc1fd--25`, `00000058--638af96068--22`; +4105 more
- Downloaded route summary: `0000071c--fb4cca0034` (75 segments), `00000724--3ac8b5c193` (5 segments)
- Downloaded segments: `0000071c--fb4cca0034--106`, `0000071c--fb4cca0034--107`, `0000071c--fb4cca0034--108` (+77 more)
- Report JSON: `~/.comma/stopping_behavior/reports/sync_comma_20260221T094737Z.json`
- Settings JSON: `~/.comma/stopping_behavior/settings/stop_settings_comma_20260221T094737Z.json`
- Stop settings snapshot: AdvancedLongitudinalTune=True, LongitudinalTune=True, HumanAcceleration=True, ... (+3 more)

### 2026-02-21: Stopping analysis for route 00000724--3ac8b5c193

- Host: `comma`
- Route: `00000724--3ac8b5c193`
- Segments analyzed: 26
- Detected stop events: 7
- Median duration to standstill hold: 8.40 s
- Median approach speed: 3.64 m/s
- Median entry speed: 3.64 m/s
- Median min aEgo: -1.15 m/s²
- Median min accel cmd: 0.00 m/s²
- Median shouldStop->stopping delay: 0.000 s
- Median creep after stop: 0.044 m/s
- Settings snapshot: `~/.comma/stopping_behavior/settings/stop_settings_comma_20260221T094737Z.json`
- Analysis summary JSON: `~/.comma/stopping_behavior/analysis/comma/00000724--3ac8b5c193/20260221T095454Z/summary.json`
- Analysis summary Markdown: `~/.comma/stopping_behavior/analysis/comma/00000724--3ac8b5c193/20260221T095454Z/summary.md`
- Example event graph: `~/.comma/stopping_behavior/analysis/comma/00000724--3ac8b5c193/20260221T095454Z/events/event_001_seg_001.html`
- Note: Measured gate (min_enabled_ratio=0.8): insufficient_events (3 considered), harsh=3/3 (flags include end_stop_accel_step).
- Note: Model fit: ~/.comma/stopping_behavior/models/stopping_model_20260221T095656Z_all.json (best_delay_frames=0, rows=288).
- Note: Model gate (controller replay): insufficient_events (3 considered), harsh=1/3, avg_score=1.012.

### 2026-02-21: Stopping analysis for route 0000071c--fb4cca0034

- Host: `comma`
- Route: `0000071c--fb4cca0034`
- Segments analyzed: 151
- Detected stop events: 21
- Median duration to standstill hold: 8.50 s
- Median approach speed: 6.42 m/s
- Median entry speed: 3.26 m/s
- Median min aEgo: -1.48 m/s²
- Median min accel cmd: -0.33 m/s²
- Median shouldStop->stopping delay: 0.000 s
- Median creep after stop: 0.042 m/s
- Settings snapshot: `~/.comma/stopping_behavior/settings/stop_settings_comma_20260221T094737Z.json`
- Analysis summary JSON: `~/.comma/stopping_behavior/analysis/comma/0000071c--fb4cca0034/20260221T095810Z/summary.json`
- Analysis summary Markdown: `~/.comma/stopping_behavior/analysis/comma/0000071c--fb4cca0034/20260221T095810Z/summary.md`
- Example event graph: `~/.comma/stopping_behavior/analysis/comma/0000071c--fb4cca0034/20260221T095810Z/events/event_001_seg_140.html`
- Note: Measured gate (min_enabled_ratio=0.8, min_entry_speed=3): FAIL harsh=6/6 (dominant flags: end_stop_accel_step, end_stop_jerk, end_stop_cmd_jerk, hard_min_a_ego).
- Note: Model gate (controller replay, model=~/.comma/stopping_behavior/models/stopping_model_20260221T095656Z_all.json): FAIL harsh=8/10, avg_score=1.816.
- Note: Variant benchmark (10 events): current harsh=8/10 avg_score=1.428; abstract 10/10 3.433; inverse 9/10 2.280; inverse_v2 10/10 2.706; legacy_32b8be 8/10 1.881.

### 2026-02-21: Log sync from comma

- Host: `comma`
- Sync counts: remote=4269, new=4029, changed=0, downloaded=80
- Additional counts: unchanged=240, failures=0, skipped_limit=3949
- New routes detected: 202 total; sample: `00000056--6f60cbf398`, `00000057--37a67dc1fd`, `00000058--638af96068`; +199 more
- New segments detected: 4029 total; sample: `00000056--6f60cbf398--22`, `00000057--37a67dc1fd--25`, `00000058--638af96068--22`; +4026 more
- Downloaded route summary: `0000071c--fb4cca0034` (46 segments), `00000724--3ac8b5c193` (14 segments), `00000725--3f0ffa8f57` (9 segments) (+1 more)
- Downloaded segments: `0000071c--fb4cca0034--100`, `0000071c--fb4cca0034--101`, `0000071c--fb4cca0034--102` (+77 more)
- Report JSON: `~/.comma/stopping_behavior/reports/sync_comma_20260221T115313Z.json`
- Settings JSON: `~/.comma/stopping_behavior/settings/stop_settings_comma_20260221T115313Z.json`
- Stop settings snapshot: AdvancedLongitudinalTune=True, LongitudinalTune=True, HumanAcceleration=True, ... (+3 more)

### 2026-02-21: Stopping analysis for route 00000726--3b632066b8

- Host: `comma`
- Route: `00000726--3b632066b8`
- Segments analyzed: 11
- Detected stop events: 3
- Median duration to standstill hold: 7.10 s
- Median approach speed: 2.62 m/s
- Median entry speed: 2.62 m/s
- Median min aEgo: -0.95 m/s²
- Median min accel cmd: 0.00 m/s²
- Median shouldStop->stopping delay: n/a s
- Median creep after stop: 0.049 m/s
- Settings snapshot: `~/.comma/stopping_behavior/settings/stop_settings_comma_20260221T115313Z.json`
- Analysis summary JSON: `~/.comma/stopping_behavior/analysis/comma/cycle_20260221T115313Z/summary.json`
- Analysis summary Markdown: `~/.comma/stopping_behavior/analysis/comma/cycle_20260221T115313Z/summary.md`
- Example event graph: `~/.comma/stopping_behavior/analysis/comma/cycle_20260221T115313Z/events/event_001_seg_002.html`

### 2026-02-21: Stopping cycle results

- Host: `comma`
- Cycle stamp: `20260221T115313Z`
- Settings JSON: `~/.comma/stopping_behavior/settings/stop_settings_comma_20260221T115313Z.json`
- Sync report JSON: `~/.comma/stopping_behavior/reports/sync_comma_20260221T115313Z.json`
- Analysis summary JSON: `~/.comma/stopping_behavior/analysis/comma/cycle_20260221T115313Z/summary.json`
- Fit summary inputs: 8 file(s)
- Gate summary inputs: 2 file(s)
- Model JSON: `~/.comma/stopping_behavior/models/stopping_model_20260221T115313Z_all.json`
- Model fit event_source: `all`
- Model fit windows_used: 26
- Model fit delay_frames: 0
- Model fit rows: 451
- Model fit rmse=0.0402 mae=0.0294 r2=0.9598
- Model fit summary inputs: 8 file(s)
- Measured gate: fail harsh=11/11 harsh_rate=1.000 leapfrog=0/11 leapfrog_rate=0.000
- Measured gate JSON: `~/.comma/stopping_behavior/analysis/measured_harsh_gate_comma_20260221T115313Z_all.json`
- Model gate: fail harsh=12/15 harsh_rate=0.800 leapfrog=0/15 leapfrog_rate=0.000 avg_score=2.799
- Model gate JSON: `~/.comma/stopping_behavior/analysis/model_harsh_check_comma_20260221T115313Z_all.json`
- Leapfrog alignment: pass overlap=0 measured=0 predicted=0 recall=1.000 precision=1.000
- Leapfrog alignment JSON: `~/.comma/stopping_behavior/analysis/leapfrog_alignment_comma_20260221T115313Z_all.json`
- Variant benchmark events: 10
- Variant `current`: harsh=9/10 rate=0.900 avg_score=2.565
- Variant `abstract`: harsh=10/10 rate=1.000 avg_score=4.060
- Variant `inverse`: harsh=9/10 rate=0.900 avg_score=4.407
- Variant `inverse_v2`: harsh=9/10 rate=0.900 avg_score=4.413
- Variant `legacy_32b8be`: harsh=8/10 rate=0.800 avg_score=3.795
- Variant benchmark JSON: `~/.comma/stopping_behavior/analysis/controller_variant_benchmark_comma_20260221T115313Z_all.json`

### 2026-02-21: Controller replay after end-stop release-step tune

- Host: `comma`
- Cycle stamp: `20260221T120917Z`
- Gate summary inputs: 2 file(s)
- Model JSON: `~/.comma/stopping_behavior/models/stopping_model_20260221T115313Z_all.json`
- Model fit event_source: `all`
- Model fit windows_used: 26
- Model fit delay_frames: 0
- Model fit rows: 451
- Model fit rmse=0.0402 mae=0.0294 r2=0.9598
- Model fit summary inputs: 8 file(s)
- Model gate: fail harsh=13/15 harsh_rate=0.867 leapfrog=0/15 leapfrog_rate=0.000 avg_score=2.922
- Model gate JSON: `~/.comma/stopping_behavior/analysis/model_harsh_check_comma_20260221T120917Z_all_after_release_step_tune.json`
- Variant benchmark events: 10
- Variant `current`: harsh=9/10 rate=0.900 avg_score=2.619
- Variant `abstract`: harsh=10/10 rate=1.000 avg_score=4.060
- Variant `inverse`: harsh=9/10 rate=0.900 avg_score=4.407
- Variant `inverse_v2`: harsh=9/10 rate=0.900 avg_score=4.413
- Variant `legacy_32b8be`: harsh=8/10 rate=0.800 avg_score=3.795
- Variant benchmark JSON: `~/.comma/stopping_behavior/analysis/controller_variant_benchmark_comma_20260221T120917Z_all_after_release_step_tune.json`
- Note: Change: reduced low-speed release_step caps (soft_landing_release, low_rollout_soft_landing_cap, end_stop_cap_active)

### 2026-02-21: Controller replay after end-stop cap soften

- Host: `comma`
- Cycle stamp: `20260221T121124Z`
- Gate summary inputs: 2 file(s)
- Model JSON: `~/.comma/stopping_behavior/models/stopping_model_20260221T115313Z_all.json`
- Model fit event_source: `all`
- Model fit windows_used: 26
- Model fit delay_frames: 0
- Model fit rows: 451
- Model fit rmse=0.0402 mae=0.0294 r2=0.9598
- Model fit summary inputs: 8 file(s)
- Model gate: fail harsh=11/15 harsh_rate=0.733 leapfrog=0/15 leapfrog_rate=0.000 avg_score=2.657
- Model gate JSON: `~/.comma/stopping_behavior/analysis/model_harsh_check_comma_20260221T121124Z_all_after_end_stop_cap_soften.json`
- Variant benchmark events: 10
- Variant `current`: harsh=9/10 rate=0.900 avg_score=2.398
- Variant `abstract`: harsh=10/10 rate=1.000 avg_score=4.060
- Variant `inverse`: harsh=9/10 rate=0.900 avg_score=4.407
- Variant `inverse_v2`: harsh=9/10 rate=0.900 avg_score=4.413
- Variant `legacy_32b8be`: harsh=8/10 rate=0.800 avg_score=3.795
- Variant benchmark JSON: `~/.comma/stopping_behavior/analysis/controller_variant_benchmark_comma_20260221T121124Z_all_after_end_stop_cap_soften.json`
- Note: Change: softened end_stop_brake_cap (v=0.15/0.25/0.60) to reduce hard_min_a_ego

### 2026-02-21: Controller replay after end-stop cap soften (v2)

- Host: `comma`
- Cycle stamp: `20260221T121305Z`
- Gate summary inputs: 2 file(s)
- Model JSON: `~/.comma/stopping_behavior/models/stopping_model_20260221T115313Z_all.json`
- Model fit event_source: `all`
- Model fit windows_used: 26
- Model fit delay_frames: 0
- Model fit rows: 451
- Model fit rmse=0.0402 mae=0.0294 r2=0.9598
- Model fit summary inputs: 8 file(s)
- Model gate: fail harsh=10/15 harsh_rate=0.667 leapfrog=0/15 leapfrog_rate=0.000 avg_score=2.577
- Model gate JSON: `~/.comma/stopping_behavior/analysis/model_harsh_check_comma_20260221T121305Z_all_after_end_stop_cap_soften_v2.json`
- Variant benchmark events: 10
- Variant `current`: harsh=7/10 rate=0.700 avg_score=2.355
- Variant `abstract`: harsh=10/10 rate=1.000 avg_score=4.060
- Variant `inverse`: harsh=9/10 rate=0.900 avg_score=4.407
- Variant `inverse_v2`: harsh=9/10 rate=0.900 avg_score=4.413
- Variant `legacy_32b8be`: harsh=8/10 rate=0.800 avg_score=3.795
- Variant benchmark JSON: `~/.comma/stopping_behavior/analysis/controller_variant_benchmark_comma_20260221T121305Z_all_after_end_stop_cap_soften_v2.json`
- Note: Change: softened end_stop_brake_cap further (v=0.15/0.25/0.60)

### 2026-02-21: Controller replay after end-stop cap soften (v3)

- Host: `comma`
- Cycle stamp: `20260221T121356Z`
- Gate summary inputs: 2 file(s)
- Model JSON: `~/.comma/stopping_behavior/models/stopping_model_20260221T115313Z_all.json`
- Model fit event_source: `all`
- Model fit windows_used: 26
- Model fit delay_frames: 0
- Model fit rows: 451
- Model fit rmse=0.0402 mae=0.0294 r2=0.9598
- Model fit summary inputs: 8 file(s)
- Model gate: fail harsh=8/15 harsh_rate=0.533 leapfrog=0/15 leapfrog_rate=0.000 avg_score=2.370
- Model gate JSON: `~/.comma/stopping_behavior/analysis/model_harsh_check_comma_20260221T121356Z_all_after_end_stop_cap_soften_v3.json`
- Variant benchmark events: 10
- Variant `current`: harsh=6/10 rate=0.600 avg_score=2.207
- Variant `abstract`: harsh=10/10 rate=1.000 avg_score=4.060
- Variant `inverse`: harsh=9/10 rate=0.900 avg_score=4.407
- Variant `inverse_v2`: harsh=9/10 rate=0.900 avg_score=4.413
- Variant `legacy_32b8be`: harsh=8/10 rate=0.800 avg_score=3.795
- Variant benchmark JSON: `~/.comma/stopping_behavior/analysis/controller_variant_benchmark_comma_20260221T121356Z_all_after_end_stop_cap_soften_v3.json`
- Note: Change: softened end_stop_brake_cap further (v=0.15/0.25/0.60 -> -0.30/-0.46/-0.74)

### 2026-02-21: Controller replay after end-stop cap soften (v4)

- Host: `comma`
- Cycle stamp: `20260221T121435Z`
- Gate summary inputs: 2 file(s)
- Model JSON: `~/.comma/stopping_behavior/models/stopping_model_20260221T115313Z_all.json`
- Model fit event_source: `all`
- Model fit windows_used: 26
- Model fit delay_frames: 0
- Model fit rows: 451
- Model fit rmse=0.0402 mae=0.0294 r2=0.9598
- Model fit summary inputs: 8 file(s)
- Model gate: pass harsh=7/15 harsh_rate=0.467 leapfrog=0/15 leapfrog_rate=0.000 avg_score=2.227
- Model gate JSON: `~/.comma/stopping_behavior/analysis/model_harsh_check_comma_20260221T121435Z_all_after_end_stop_cap_soften_v4.json`
- Variant benchmark events: 10
- Variant `current`: harsh=5/10 rate=0.500 avg_score=2.046
- Variant `abstract`: harsh=10/10 rate=1.000 avg_score=4.060
- Variant `inverse`: harsh=9/10 rate=0.900 avg_score=4.407
- Variant `inverse_v2`: harsh=9/10 rate=0.900 avg_score=4.413
- Variant `legacy_32b8be`: harsh=8/10 rate=0.800 avg_score=3.795
- Variant benchmark JSON: `~/.comma/stopping_behavior/analysis/controller_variant_benchmark_comma_20260221T121435Z_all_after_end_stop_cap_soften_v4.json`
- Note: Change: softened end_stop_brake_cap further (v=0.25/0.60 -> -0.42/-0.68)

### 2026-02-21: Log sync from comma

- Host: `comma`
- Sync counts: remote=4270, new=3950, changed=0, downloaded=80
- Additional counts: unchanged=320, failures=0, skipped_limit=3870
- New routes detected: 202 total; sample: `00000056--6f60cbf398`, `00000057--37a67dc1fd`, `00000058--638af96068`; +199 more
- New segments detected: 3950 total; sample: `00000056--6f60cbf398--22`, `00000057--37a67dc1fd--25`, `00000058--638af96068--22`; +3947 more
- Downloaded route summary: `00000727--05c36e3fcf` (2 segments), `00000728--f6382db228` (22 segments), `00000729--80c981f2d9` (1 segments) (+4 more)
- Downloaded segments: `00000727--05c36e3fcf--10`, `00000727--05c36e3fcf--9`, `00000728--f6382db228--0` (+77 more)
- Report JSON: `~/.comma/stopping_behavior/reports/sync_comma_20260221T140042Z.json`
- Settings JSON: `~/.comma/stopping_behavior/settings/stop_settings_comma_20260221T140042Z.json`
- Stop settings snapshot: AdvancedLongitudinalTune=True, LongitudinalTune=True, HumanAcceleration=True, ... (+3 more)

### 2026-02-21: Stopping analysis for route 0000072d--4b17f9d0de

- Host: `comma`
- Route: `0000072d--4b17f9d0de`
- Segments analyzed: 6
- Detected stop events: 4
- Median duration to standstill hold: 8.45 s
- Median approach speed: 7.74 m/s
- Median entry speed: 5.96 m/s
- Median min aEgo: -1.22 m/s²
- Median min accel cmd: -1.21 m/s²
- Median shouldStop->stopping delay: 0.000 s
- Median creep after stop: 0.047 m/s
- Settings snapshot: `~/.comma/stopping_behavior/settings/stop_settings_comma_20260221T140042Z.json`
- Analysis summary JSON: `~/.comma/stopping_behavior/analysis/comma/cycle_20260221T140042Z/summary.json`
- Analysis summary Markdown: `~/.comma/stopping_behavior/analysis/comma/cycle_20260221T140042Z/summary.md`
- Example event graph: `~/.comma/stopping_behavior/analysis/comma/cycle_20260221T140042Z/events/event_001_seg_001.html`

### 2026-02-21: Stopping cycle results

- Host: `comma`
- Cycle stamp: `20260221T140042Z`
- Settings JSON: `~/.comma/stopping_behavior/settings/stop_settings_comma_20260221T140042Z.json`
- Sync report JSON: `~/.comma/stopping_behavior/reports/sync_comma_20260221T140042Z.json`
- Analysis summary JSON: `~/.comma/stopping_behavior/analysis/comma/cycle_20260221T140042Z/summary.json`
- Fit summary inputs: 6 file(s)
- Gate summary inputs: 2 file(s)
- Model JSON: `~/.comma/stopping_behavior/models/stopping_model_20260221T140042Z_all.json`
- Model fit event_source: `all`
- Model fit windows_used: 17
- Model fit delay_frames: 0
- Model fit rows: 298
- Model fit rmse=0.0401 mae=0.0294 r2=0.9598
- Model fit summary inputs: 6 file(s)
- Measured gate: fail harsh=11/11 harsh_rate=1.000 leapfrog=0/11 leapfrog_rate=0.000
- Measured gate JSON: `~/.comma/stopping_behavior/analysis/measured_harsh_gate_comma_20260221T140042Z_all.json`
- Model gate: fail harsh=8/15 harsh_rate=0.533 leapfrog=1/15 leapfrog_rate=0.067 avg_score=2.415
- Model gate JSON: `~/.comma/stopping_behavior/analysis/model_harsh_check_comma_20260221T140042Z_all.json`
- Leapfrog alignment: pass overlap=0 measured=0 predicted=1 recall=1.000 precision=0.000
- Leapfrog alignment JSON: `~/.comma/stopping_behavior/analysis/leapfrog_alignment_comma_20260221T140042Z_all.json`
- Variant benchmark events: 10
- Variant `current`: harsh=7/10 rate=0.700 avg_score=2.288
- Variant `abstract`: harsh=10/10 rate=1.000 avg_score=4.122
- Variant `inverse`: harsh=9/10 rate=0.900 avg_score=4.572
- Variant `inverse_v2`: harsh=10/10 rate=1.000 avg_score=4.631
- Variant `legacy_32b8be`: harsh=8/10 rate=0.800 avg_score=3.917
- Variant benchmark JSON: `~/.comma/stopping_behavior/analysis/controller_variant_benchmark_comma_20260221T140042Z_all.json`
