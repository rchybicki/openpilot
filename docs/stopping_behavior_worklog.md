# Stopping Behavior Project Worklog

- Last updated: 2026-02-07
- Scope: OpenPilot/FrogPilot longitudinal stopping behavior
- Goal: Make stopping behavior more consistent and comfortable while preserving safety

## Clarified Requirements (2026-02-07)

Primary scope:
- Only analyze and tune stopping while OpenPilot is engaged.
- Manual/user stopping behavior is out of scope for this project.

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
- Engaged stop events are the primary metric set (`event_mode=engaged_signal`).
- Hybrid/speed-transition events remain secondary context only.

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
- [ ] Capture first intentionally stop-focused drive route
- [x] Build baseline stop metrics from real logs
- [ ] Propose and test first algorithm change

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
- Report JSON: `/Users/radoslawchybicki/.comma/stopping_behavior/reports/sync_commawifi_20260207T160628Z.json`
- Settings JSON: `/Users/radoslawchybicki/.comma/stopping_behavior/settings/stop_settings_commawifi_20260207T160001Z.json`
- Stop settings snapshot:
  - AdvancedLongitudinalTune=True, LongitudinalTune=True, HumanAcceleration=True, ForceStops=False
  - StartAccel=0.0, StopAccel=-1.5, StoppingDecelRate=0.5, VEgoStarting=0.1
  - VEgoStopping=0.5, StoppingSpeedBreakpoint=0.4, StoppingErrorFactor=1.3
- Findings: _pending analysis of downloaded logs_
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
- Settings snapshot: `/Users/radoslawchybicki/.comma/stopping_behavior/settings/stop_settings_commawifi_20260207T160001Z.json`
- Analysis summary JSON: `/Users/radoslawchybicki/.comma/stopping_behavior/analysis/commawifi/000006c0--81e575d831/20260207T170334Z/summary.json`
- Analysis summary Markdown: `/Users/radoslawchybicki/.comma/stopping_behavior/analysis/commawifi/000006c0--81e575d831/20260207T170334Z/summary.md`
- Example event graph: `/Users/radoslawchybicki/.comma/stopping_behavior/analysis/commawifi/000006c0--81e575d831/20260207T170334Z/events/event_001_seg_101.html`
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
- Settings snapshot: `/Users/radoslawchybicki/.comma/stopping_behavior/settings/stop_settings_commawifi_20260207T160534Z.json`
- Analysis summary JSON: `/Users/radoslawchybicki/.comma/stopping_behavior/analysis/commawifi/00000619--b7c72f4e46/20260207T173508Z/summary.json`
- Analysis summary Markdown: `/Users/radoslawchybicki/.comma/stopping_behavior/analysis/commawifi/00000619--b7c72f4e46/20260207T173508Z/summary.md`
- Example event graph: `/Users/radoslawchybicki/.comma/stopping_behavior/analysis/commawifi/00000619--b7c72f4e46/20260207T173508Z/events/event_001_seg_005.html`
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
    - Summary: `/Users/radoslawchybicki/.comma/stopping_behavior/analysis/corpus/commawifi/20260207T223433Z/summary.json`
  - `speed_transition + require_enabled + min_entry_speed=0.0`:
    - `105` events.
    - Summary: `/Users/radoslawchybicki/.comma/stopping_behavior/analysis/corpus/commawifi/20260207T223330Z/summary.json`
  - `hybrid + min_entry_speed=0.1`:
    - `744` events.
    - Summary: `/Users/radoslawchybicki/.comma/stopping_behavior/analysis/corpus/commawifi/20260207T223505Z/summary.json`

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
  - `/Users/radoslawchybicki/.comma/stopping_behavior/analysis/corpus/commawifi/20260207T223330Z/failure_diagnosis_speed.md`

- Route review pack generated for top-ranked routes:
  - `/Users/radoslawchybicki/.comma/stopping_behavior/analysis/review_pack/20260207T223330Z_speed_top3/manifest.json`
  - Included routes: `00000619--b7c72f4e46`, `0000061b--dafb553d6e`, `000006be--864be32bdb`

- Tooling fix:
  - `tools/stopping/build_review_pack.py` updated to include new clutch-disturbance threshold args used by diagnosis (`should_stop_*`, `unexpected_accel_*`, `stable_cmd_accel_delta_*`), fixing a runtime `AttributeError`.

- Current assumptions and open questions:
  - Assumption: stop-signal/state drops near hold are often contextual (lead/light release), not primary faults unless paired with rebound/disturbance.
  - Assumption: clutch disturbance is observable as `aEgo` relief/rebound while braking command remains near-constant and `shouldStop` remains true.
  - Open question for next drive labels: when bookmarking a bad stop, confirm whether perceived issue is (a) end-stop jerk spike or (b) near-hold release/restart, so we can split tuning targets cleanly.

### 2026-02-07: Device settings snapshot (pre-change baseline)

- Snapshot file:
  - `/Users/radoslawchybicki/.comma/stopping_behavior/settings/stop_settings_commawifi_20260207T222847Z.json`
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
  - `/Users/radoslawchybicki/.comma/stopping_behavior/analysis/bookmarks/commawifi/20260207T222936Z/summary.json`
  - `/Users/radoslawchybicki/.comma/stopping_behavior/analysis/bookmarks/commawifi/20260207T222936Z/summary.md`
- Result:
  - `6` bookmarks detected across `4` routes.
  - `3` bookmarks matched to detected stop events.
  - `3` bookmarks unmatched (no detected stop event in configured match windows).
- Note:
  - This validates end-to-end bookmark ingestion for upcoming "bad stop only" tagging drives.
