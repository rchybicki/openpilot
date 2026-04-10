# Hyundai Santa Fe HEV 2022 Longitudinal Tuning Worklog

- Last updated: 2026-04-09
- Status snapshot + plan: [longitudinal_tuning_status.md](longitudinal_tuning_status.md)
- Scope: OpenPilot/FrogPilot longitudinal tuning for one car only: `HYUNDAI_SANTA_FE_HEV_2022`
- Goal: Improve requested-vs-actual acceleration tracking and comfort on this car without making cross-car assumptions

## How to Use This Worklog

- Current status and next actions: `docs/longitudinal_tuning_status.md`
- Operational workflow and command reference: `tools/longitudinal/README.md`
- Runtime source of truth:
  - `selfdrive/controls/lib/longcontrol.py`
  - `selfdrive/controls/lib/longitudinal_planner.py`
  - `frogpilot/common/frogpilot_variables.py`
  - `opendbc/car/hyundai/interface.py`
- Shared route intake process:
  - `docs/route_refresh_process.md`
  - `tools/route_sync/refresh_routes.py`
- This worklog records dated commands, artifact paths, measured results, and keep/reject decisions.
- Longitudinal tracking analysis for this workflow lives in:
  - `tools/longitudinal/analyze_longitudinal_tracking.py`

## Research Notes

- Official openpilot already has a route-based longitudinal validation path through the maneuver tool:
  - `tools/longitudinal_maneuvers/README.md`
  - GitHub reference: <https://github.com/commaai/openpilot/blob/master/tools/longitudinal_maneuvers/README.md>
- FrogPilot explicitly exposes longitudinal tuning and advanced longitudinal tuning as user-facing controls:
  - August 9th, 2025 release: <https://github.com/FrogAi/FrogPilot/releases>
    - added "Advanced Longitudinal Tuning" menu
  - November 1st, 2025 release: <https://github.com/FrogAi/FrogPilot/releases>
    - reworked acceleration profiles to be lighter and smoother
- Repo-specific implication:
  - `AdvancedLongitudinalTune` gates whether the custom `LongitudinalActuatorDelay` and `MaxDesiredAcceleration` params actually override the stock per-car defaults.
  - With `AdvancedLongitudinalTune=False`, stored advanced values are mostly inert and the effective behavior is driven by stock Hyundai/FrogPilot logic plus the regular longitudinal profile toggles.

## Improvement Cycle (One Iteration)

1. Refresh the shared route cache from the device.
   - Example:
     `python tools/route_sync/refresh_routes.py --host comma --remote-root /data/media/0/realdata_konik --max-downloads 80 --newest-first`
2. Snapshot the active longitudinal tuning params before analyzing behavior.
   - Example:
     `python3.11 tools/stopping/device_stop_settings.py snapshot --host comma`
3. Run the longitudinal tracking analyzer on the newest complete Santa Fe routes.
   - Example:
     `python3.11 tools/longitudinal/analyze_longitudinal_tracking.py --host comma --max-routes 5 --min-route-segments 5`
4. Review aggregate delay / RMSE / bias first.
   - Treat `carControl.actuators.accel -> carState.aEgo` as the main tracking lane.
   - Treat `longitudinalPlan.aTarget -> carState.aEgo` as the higher-level planner + controller lane.
5. Review the largest mismatch windows.
   - Separate `accel under-response`, `accel overshoot`, `brake under-response`, and `brake overshoot`.
6. Decide whether the issue is global or localized.
   - Global lag across most events suggests a core timing/actuation issue.
   - Sparse high-error windows suggest profile logic, stop/launch heuristics, or situational planner behavior.
7. Make one scoped change for this car only.
   - Prefer per-car tuning or user-facing Hyundai/FrogPilot settings before broad controller changes.
8. Re-run the same analysis on fresh routes and compare against the frozen baseline in this file.

If a change is not backed by a fresh route pull and a tracking comparison, it does not count as a completed iteration.

## Session Log

### 2026-04-09: Initial longitudinal baseline for Santa Fe HEV 2022

What was done:
- Reviewed the repo’s existing route-driven workflow and used the stopping project structure as the process template.
- Confirmed the current Hyundai runtime sources and FrogPilot parameter plumbing for longitudinal behavior.
- Added a repeatable analyzer:
  - `tools/longitudinal/analyze_longitudinal_tracking.py`
- Used the shared route refresh workflow to download recent device qlogs from:
  - `/data/media/0/realdata_konik`
- Confirmed the downloaded routes fingerprint as:
  - `HYUNDAI_SANTA_FE_HEV_2022`

Commands run:

```bash
python tools/route_sync/refresh_routes.py \
  --host comma \
  --remote-root /data/media/0/realdata_konik \
  --max-downloads 80 \
  --newest-first

python3.11 tools/stopping/device_stop_settings.py snapshot --host comma

python3.11 tools/longitudinal/analyze_longitudinal_tracking.py \
  --host comma \
  --max-routes 5 \
  --min-route-segments 5
```

Artifacts:
- Route refresh report:
  - `~/.comma/route_sync/reports/route_refresh_comma_20260409T145432Z.json`
- Param snapshot:
  - `~/.comma/stopping_behavior/settings/stop_settings_comma_20260409T150417Z.json`
- Longitudinal analysis summary:
  - `~/.comma/longitudinal_tuning/analysis/comma/20260409T150343Z/summary.json`
  - `~/.comma/longitudinal_tuning/analysis/comma/20260409T150343Z/summary.md`

Current device longitudinal settings:
- `AdvancedLongitudinalTune=False`
- `LongitudinalTune=True`
- `HumanAcceleration=True`
- `HumanFollowing=True`
- `HumanLaneChanges=True`
- `AccelerationProfile=1` (`Eco`)
- `DecelerationProfile=1` (`Eco`)
- `LongitudinalActuatorDelay=0.500000`
- `MaxDesiredAcceleration=4.0`
- `LeadDetectionThreshold=35`
- `TacoTune=False`

Important interpretation of those settings:
- Advanced longitudinal overrides are currently off, so this baseline is not coming from a custom Santa Fe actuator-delay override.
- The regular FrogPilot longitudinal behavior is enabled, and both accel/decel profiles are set to `Eco`.
- `HumanAcceleration` and `HumanFollowing` are enabled, so some softness in launches and gap-closing is expected by design.

Route coverage:
- Downloaded qlogs: `80`
- Local qlogs now available under the shared cache for this pull: `111`
- Newest complete engaged routes used for scoring:
  - `000009cc--94242f81db`
  - `000009cb--55a0e11719`
  - `000009ca--2b7b178788`
- Also downloaded but effectively excluded from quantitative tracking because longitudinal engagement time was near zero:
  - `000009c8--4ee2b2a6f3`
  - `000009c7--3c840cd413`

Baseline metrics from fresh routes:
- Aggregate `carControl.actuators.accel -> carState.aEgo`
  - all: delay `0.21s`, RMSE `0.164 m/s^2`, bias `-0.001`, corr `0.971`
  - accel-only: delay `0.24s`, RMSE `0.173`, bias `-0.020`, corr `0.874`
  - brake-only: delay `0.21s`, RMSE `0.155`, bias `+0.018`, corr `0.937`
- Aggregate `longitudinalPlan.aTarget -> carState.aEgo`
  - all: delay `0.04s`, RMSE `0.184 m/s^2`, bias `-0.004`, corr `0.963`
  - accel-only: delay `0.09s`, RMSE `0.192`, bias `-0.026`, corr `0.850`
  - brake-only: delay `0.04s`, RMSE `0.176`, bias `+0.017`, corr `0.918`
- Actual-signal agreement sanity check:
  - `livePose.accelerationDevice.x vs carState.aEgo`: RMSE `0.200`, bias `+0.005`, corr `0.918`
  - Conclusion: `carState.aEgo` is good enough for this cycle’s primary tracking metric.

Route ranking:
- `000009ca--2b7b178788` was the best engaged route in this slice:
  - command RMSE `0.141`
- `000009cc--94242f81db` and `000009cb--55a0e11719` were worse but still broadly sane:
  - command RMSE `0.183`
  - command RMSE `0.194`

Largest mismatch windows:
- `000009cb--55a0e11719`, segment `4`, `273.2s`, `1.40s`
  - `accel under-response`
  - requested `+0.97`, actual `+0.20`, mean error `-0.78`, peak abs `1.44`
- `000009cb--55a0e11719`, segment `1`, `94.5s`, `1.40s`
  - `accel under-response`
  - requested `+1.01`, actual `+0.19`, mean error `-0.82`, peak abs `1.18`
- `000009cc--94242f81db`, segment `3`, `216.7s`, `2.30s`
  - `brake under-response`
  - requested `-1.21`, actual `-0.73`, mean error `+0.49`, peak abs `0.58`
- `000009cc--94242f81db`, segment `11`, `664.3s`, `1.20s`
  - `accel under-response`
  - requested `+0.89`, actual `+0.11`, mean error `-0.78`, peak abs `1.07`
- `000009ca--2b7b178788`, segment `27`, `1668.4s`, `1.10s`
  - `accel under-response`
  - requested `+0.73`, actual `+0.21`, mean error `-0.51`, peak abs `1.16`
- One notable counterexample remains:
  - `000009cb--55a0e11719`, segment `8`, `509.2s`
  - `accel overshoot`
  - requested `+0.88`, actual `+1.41`, mean error `+0.53`

Interpretation:
- The tuning is not globally bad.
  - Overall command tracking is fairly tight for fresh real-world routes: `0.164 m/s^2` RMSE with `0.971` correlation is already respectable.
- The biggest problem in this sample is not a constant lag or constant bias.
  - The bias is almost zero overall.
  - The bad behavior is concentrated in a few obvious windows.
- The dominant failure mode is repeated `accel under-response`.
  - Several windows show commanded accel around `+0.7` to `+1.0 m/s^2` while actual accel stays near `+0.0` to `+0.2`.
  - That lines up better with profile softness / launch behavior than with a globally wrong actuator-delay model.
- Brake tracking is better than accel tracking overall, but low-speed `brake under-response` still appears during some arrest phases.
- The planner-to-actual delay is much smaller than the command-to-actual delay.
  - `aTarget -> aEgo` best delay is only about `0.04s`.
  - `accel_cmd -> aEgo` best delay is about `0.21s`.
  - This suggests the planner/controller stack is already compensating for most of the physical lag.
  - Because of that, blindly reducing `LongitudinalActuatorDelay` is not the best first move for this car.
- Current settings already bias the car toward gentler response:
  - `AccelerationProfile=Eco`
  - `DecelerationProfile=Eco`
  - `HumanAcceleration=True`
  - `HumanFollowing=True`

Keep / change decision:
- Keep the codebase unchanged for now.
- Do not start with a global controller change.
- Do not start by lowering `LongitudinalActuatorDelay` just because the output lane shows `~0.21s` delay.
  - With `AdvancedLongitudinalTune=False`, there is no active custom per-car actuator-delay override in effect.
  - The bigger win is more likely in the Santa Fe’s launch/response softness path than in core timing.

Next actions for the next iteration:
1. Freeze `20260409T150343Z` as the baseline comparison point for this car.
2. Collect one dedicated longitudinal maneuver route for the Santa Fe HEV 2022 using the official maneuver workflow.
   - This will separate true actuator lag from profile-intent softness.
3. First candidate change:
   - test a Santa Fe-only reduction in launch softness before touching core actuator delay
   - likely knobs to inspect first: accel profile path and `HumanAcceleration`
4. Second candidate change:
   - inspect whether `HumanFollowing` or Eco decel tuning is contributing to the low-speed `brake under-response` windows
5. Only after the maneuver route confirms a genuine global lag issue:
   - test a small per-car actuator-delay change
   - re-run the exact same route-tracking analysis on a fresh pull

### 2026-04-09: Braking-first baseline

Goal for this phase:
- Focus braking first, specifically:
  - the last `5` seconds before standstill
  - reactions when a lead suddenly decelerates
- Optimize for:
  - lower jerk
  - less controller braking beyond the model request
  - less actual vehicle braking beyond the model request

What was added:
- Brake-specific analyzer:
  - `tools/longitudinal/analyze_braking_focus.py`
- This tool scores two event classes:
  - `stop_final_5s`
  - `lead_decel_response`
- For each event it measures:
  - command over-brake relative to model request: `max(aTarget - accel_cmd, 0)`
  - actual over-brake relative to model request: `max(aTarget - aEgo, 0)`
  - actual under-brake relative to model request: `max(aEgo - aTarget, 0)`
  - command jerk and actual jerk

Command run:

```bash
python3.11 tools/longitudinal/analyze_braking_focus.py \
  --host comma \
  --max-routes 5 \
  --min-route-segments 5
```

Artifacts:
- Braking summary JSON:
  - `~/.comma/longitudinal_tuning/braking/comma/20260409T151357Z/summary.json`
- Braking summary Markdown:
  - `~/.comma/longitudinal_tuning/braking/comma/20260409T151357Z/summary.md`

Coverage:
- Routes analyzed:
  - `000009cc--94242f81db`
  - `000009cb--55a0e11719`
  - `000009ca--2b7b178788`
  - `000009c8--4ee2b2a6f3`
  - `000009c7--3c840cd413`
- Events found:
  - final-stop windows: `4`
  - sudden lead-decel windows: `27`

Aggregate braking metrics:
- `stop_final_5s`
  - median command over-brake peak: `0.240 m/s^2`
  - p95 command over-brake peak: `0.296`
  - median actual over-brake peak: `0.227`
  - p95 actual over-brake peak: `0.255`
  - median actual jerk max: `1.373 m/s^3`
  - p95 actual jerk max: `1.547`
- `lead_decel_response`
  - median command over-brake peak: `0.457 m/s^2`
  - p95 command over-brake peak: `0.853`
  - median actual over-brake peak: `0.402`
  - p95 actual over-brake peak: `0.765`
  - median actual jerk max: `3.349 m/s^3`
  - p95 actual jerk max: `5.941`

Worst final-stop windows:
- `000009cb--55a0e11719`, segments `7->8`, `476.6s`, `4.90s`
  - mean speed `1.45 m/s`
  - model brake peak `-1.33`
  - command over-brake peak `0.24`
  - actual over-brake peak `0.19`
  - actual under-brake peak `0.25`
  - actual jerk max `1.35`
- `000009cb--55a0e11719`, segment `3`, `227.6s`, `5.00s`
  - model brake peak `-1.10`
  - command over-brake peak `0.24`
  - actual over-brake peak `0.26`
  - actual jerk max `1.40`
- `000009ca--2b7b178788`, segment `4`, `268.5s`, `4.90s`
  - model brake peak `-0.59`
  - command over-brake peak `0.23`
  - actual over-brake peak `0.20`
  - actual under-brake peak `0.43`
  - actual jerk max `1.57`

Worst sudden lead-decel windows:
- `000009ca--2b7b178788`, segment `20`, `1225.5s`, `3.00s`
  - mean speed `18.40 m/s`
  - lead distance `18.3 -> 32.3 m`
  - lead accel minimum `-1.39 m/s^2`
  - model brake peak `-2.57`
  - command over-brake peak `0.74`
  - actual over-brake peak `0.69`
  - actual under-brake peak `1.61`
  - actual jerk max `3.69`
- `000009cc--94242f81db`, segment `5`, `307.9s`, `3.00s`
  - model brake peak `-1.77`
  - command over-brake peak `0.37`
  - actual over-brake peak `0.40`
  - actual under-brake peak `0.78`
  - actual jerk max `10.33`
- `000009cc--94242f81db`, segment `11`, `711.2s`, `3.00s`
  - model brake peak `-3.08`
  - command over-brake peak `1.08`
  - actual over-brake peak `0.49`
  - actual jerk max `3.35`

Interpretation:
- Braking near standstill is not the worst part of the current Santa Fe behavior.
  - It is still imperfect, but the excess braking beyond the model request is moderate.
  - Final-stop windows cluster around about `0.23-0.24 m/s^2` command/actual over-brake peaks.
- Sudden lead-decel response is clearly the rougher lane.
  - Median command over-brake is almost `2x` the final-stop lane (`0.457` vs `0.240`).
  - Median actual jerk is much higher (`3.349` vs `1.373 m/s^3`).
  - p95 actual jerk reaches nearly `6 m/s^3`, which matches the “jerky reaction” complaint much better than the terminal stop lane does.
- The lead-decel lane is not just “too much braking”.
  - Some windows show controller/vehicle braking deeper than the model request.
  - Other windows still show large under-brake at the same time.
  - That means the real issue is inconsistent response with sharp corrections, not a simple constant brake offset.
- For the user’s stated priority, the first real tuning target should be:
  - braking response smoothing and clamp discipline in lead-decel reactions
  - then terminal-stop refinement second

Keep / change decision:
- Keep the runtime unchanged for now.
- Change the project priority order:
  1. `lead_decel_response`
  2. `stop_final_5s`
- Use “do not brake materially deeper than model request” as the first explicit braking contract for new changes.

Immediate next actions:
1. Freeze `20260409T151357Z` as the braking-first baseline.
2. On the next iteration, compare candidate changes against these braking metrics first:
   - `lead_decel_response` command over-brake peak
   - `lead_decel_response` actual over-brake peak
   - `lead_decel_response` actual jerk max
3. When changing runtime logic, prefer clamps or smoothing that stop the controller from dipping deeper than `aTarget` during sudden lead reactions.
4. Only after lead-decel jerk improves without new under-response regressions, move to the terminal-stop `5s` lane.

### 2026-04-09: First runtime change for model-request matching

Goal for this change:
- Make the Santa Fe’s plain braking response follow the model request more closely before changing planner behavior.
- Limit controller-only over-brake in sudden lead-decel reactions.
- Leave stop-target and active stopping behavior alone in this first pass.

Code change:
- Updated `selfdrive/controls/lib/longcontrol.py`
- Added a plain-PID brake alignment clamp:
  - only for `HYUNDAI_SANTA_FE_HEV_2022`
  - when in `LongCtrlState.pid`
  - when not in stop-target approach mode
  - when not in active stopping mode
  - when `aTarget` is meaningfully negative
- New rule:
  - do not let the PID path command materially more brake than `aTarget`
  - allow only a small extra margin that scales with speed and present tracking lag

Why this is the first change:
- The fresh Santa Fe braking baseline showed the main problem in `lead_decel_response`, not in the final stop lane.
- In the plain PID lane, the controller was free to turn `aTarget` plus tracking error into a much deeper brake command than the model requested.
- That is the simplest runtime mismatch to remove before questioning whether the model itself is too aggressive.

Guardrails:
- The new clamp does not touch:
  - `LongCtrlState.stopping`
  - stop-target approach shaping
  - standstill/stopping-controller release logic
- This keeps the first iteration narrow and focused on the lead-response mismatch lane.

Verification run:
- Syntax check passed:
  - `python3.11 -m py_compile selfdrive/controls/lib/longcontrol.py selfdrive/controls/lib/tests/test_longcontrol_fast_release.py`
- Added targeted tests in:
  - `selfdrive/controls/lib/tests/test_longcontrol_fast_release.py`
  - plain PID braking now stays close to `aTarget`
  - active stopping remains unclamped
- Local repo pytest is currently blocked by host-environment issues before normal test execution:
  - repo `conftest.py` hits a local `params_pyx` symbol mismatch
  - fallback isolated pytest import is missing `setproctitle`
- Direct targeted smoke verification was still run with a lightweight import stub:
  - plain PID case returned `-0.88` instead of the raw `-1.30`
  - non-Santa-Fe comparison case still returned the raw `-1.30`
  - active stopping case preserved the stopping-controller output at `-1.05`

Status:
- Implemented, but not yet route-validated.
- This does not count as a completed tuning iteration until a fresh post-change route pull is analyzed against the frozen braking baseline.

Next actions:
1. Deploy this runtime to the device.
2. Collect fresh Santa Fe routes with several lead-decel events.
3. Re-run `tools/longitudinal/analyze_braking_focus.py`.
4. Compare against `20260409T151357Z`, prioritizing:
   - `lead_decel_response` command over-brake peak
   - `lead_decel_response` actual over-brake peak
   - `lead_decel_response` actual jerk max
5. If command over-brake drops but actual under-brake rises materially, tighten the margin shape instead of touching the planner yet.

### 2026-04-09: Expanded Santa Fe historical sample and project status snapshot

What was done:
- Reused the shared route cache and reviewed the larger `commawifi` Santa Fe historical sample already present locally.
- Confirmed all analyzed routes in this expanded slice fingerprinted as:
  - `HYUNDAI_SANTA_FE_HEV_2022`
- Added project-facing docs so the longitudinal effort now has the same split as the stopping project:
  - `docs/longitudinal_tuning_status.md`
  - `tools/longitudinal/README.md`

Artifacts:
- Route refresh report:
  - `~/.comma/route_sync/reports/route_refresh_commawifi_20260409T162219Z.json`
- Expanded tracking summary:
  - `~/.comma/longitudinal_tuning/analysis/commawifi/20260409T162700Z/summary.json`
- Expanded braking summary:
  - `~/.comma/longitudinal_tuning/braking/commawifi/20260409T162700Z/summary.json`

Expanded-sample coverage:
- Routes analyzed: `25`
- Strongly engaged routes with at least `120s` longitudinal engagement:
  - `000009a8--9d2767e657`
  - `000009a7--326998e3c9`
  - `000009cb--55a0e11719`
  - `000009cc--94242f81db`
  - `000009ac--0be76cbc43`
  - `000009ca--2b7b178788`
  - `000009a0--e789ee5638`
  - `000009a1--57c722f55a`

Expanded tracking metrics:
- `carControl.actuators.accel -> carState.aEgo`
  - all: delay `0.198s`, RMSE `0.171 m/s^2`, bias `+0.013`, corr `0.962`
  - accel-only: delay `0.196s`, RMSE `0.183`, bias `-0.018`, corr `0.831`
  - brake-only: delay `0.201s`, RMSE `0.159`, bias `+0.037`, corr `0.915`
- `longitudinalPlan.aTarget -> carState.aEgo`
  - all: delay `0.018s`, RMSE `0.192 m/s^2`, bias `+0.008`, corr `0.949`
- Actual-signal agreement sanity check:
  - `livePose.accelerationDevice.x vs carState.aEgo`: RMSE `0.211`, bias `+0.002`, corr `0.899`

Expanded braking metrics:
- `stop_final_5s` (`32` events)
  - median command over-brake peak: `0.320 m/s^2`
  - p95 command over-brake peak: `0.490`
  - median actual over-brake peak: `0.247`
  - p95 actual over-brake peak: `0.612`
  - median actual jerk max: `1.277 m/s^3`
  - p95 actual jerk max: `3.717`
- `lead_decel_response` (`100` events)
  - median command over-brake peak: `0.372 m/s^2`
  - p95 command over-brake peak: `1.140`
  - median actual over-brake peak: `0.268`
  - p95 actual over-brake peak: `0.803`
  - median actual jerk max: `3.049 m/s^3`
  - p95 actual jerk max: `8.440`

Route prioritization from the broader sample:
- Tracking / launch outliers with meaningful engagement:
  - `000009a8--9d2767e657`
  - `000009a7--326998e3c9`
  - `000009cb--55a0e11719`
  - `000009cc--94242f81db`
- Lead-decel concentration by score:
  - `000009ac--0be76cbc43`
  - `000009a1--57c722f55a`
  - `000009a7--326998e3c9`
  - `000009ca--2b7b178788`
  - `000009a8--9d2767e657`

Interpretation:
- The broader sample did not overturn the original diagnosis.
- The Santa Fe still does not look globally laggy enough to justify actuator-delay-first tuning.
- The main braking problem remains the lead-decel lane, not the final `5s` stop lane.
- The most important tracking errors are still localized windows, not a constant bias problem.
- The already-committed Santa Fe-only PID brake-alignment clamp should still be treated as an unvalidated candidate.
  - This expanded historical review helps choose validation targets.
  - It does not count as fresh post-change acceptance evidence.

Next actions:
1. Deploy the current Santa Fe-only `longcontrol.py` change.
2. Collect fresh routes that include several lead-decel reactions and a few ordinary launches.
3. Re-run:
   - `tools/longitudinal/analyze_longitudinal_tracking.py`
   - `tools/longitudinal/analyze_braking_focus.py`
4. Compare first against the expanded `20260409T162700Z` baselines, not only the smaller early-day slice.
5. Keep the clamp only if `lead_decel_response` over-brake / jerk improves without a meaningful under-brake regression.

### 2026-04-09: Full shared-cache sync completed and latest clean baseline frozen

What was done:
- Completed a full sync of the device route store into the shared cache root:
  - `~/.comma/route_sync/downloads/commawifi/data/media/0/realdata_konik/`
- Verified completeness with the normal route refresher after the bulk backfill:
  - remote files: `4626`
  - new: `0`
  - changed: `0`
  - unchanged: `4626`
  - downloaded: `0`
  - failures: `0`
- Removed stale partial download leftovers from the cache before re-analysis.
- Re-ran both longitudinal analyzers on the newest fully synced routes.
- Split the results into:
  - a wider mixed-history `40`-route slice
  - the newest clean homogeneous `20`-route cohort on one commit

Artifacts:
- Route refresh verification:
  - `~/.comma/route_sync/reports/route_refresh_commawifi_20260409T164456Z.json`
- Mixed `40`-route tracking summary:
  - `~/.comma/longitudinal_tuning/analysis/commawifi/20260409T210811Z/summary.json`
- Mixed `40`-route braking summary:
  - `~/.comma/longitudinal_tuning/braking/commawifi/20260409T210811Z/summary.json`
- Clean `20`-route tracking summary:
  - `~/.comma/longitudinal_tuning/analysis/commawifi/20260409T211007Z/summary.json`
- Clean `20`-route braking summary:
  - `~/.comma/longitudinal_tuning/braking/commawifi/20260409T211007Z/summary.json`

Cache / log integrity notes:
- The shared cache is now complete for all currently reachable qlog-family files on the device.
- A few synced files are still corrupt on read, which means they were likely already truncated on-device before sync:
  - `000009ac--0be76cbc43--40/qlog.zst`
  - `000009aa--9a3e23bd35--4/qlog.zst`
  - `000009a7--326998e3c9--52/qlog.zst`
  - `000009a6--7a739dd643--5/qlog.zst`
  - `000009a4--a63d0bc954--3/qlog`
- These did not block the summaries, but they should be remembered when a specific route looks shorter than expected.

Mixed newest-40 route coverage:
- All `40` analyzed routes fingerprinted as:
  - `HYUNDAI_SANTA_FE_HEV_2022`
- Commit distribution inside this wider slice:
  - `04617c4c95bba0d786573909f4f81d01d57adf84`: `20` routes
  - `0c57895e81c7c522eb2ea3955a63f439b48a0830`: `4`
  - `3143cd94a86366ec5bda296c8b8b594cce646141`: `4`
  - `c7a08eea26fd72509ccf429f0c93f6b1303c4dbf`: `4`
  - `c703a9e593fb39521002fe085dedf7dd70a7e86d`: `3`
  - `5e8b7bb0ab120a6094e6f14a258750cf43b26607`: `2`
  - `d2c060d064fa1dc2ecfb856b1d1e941774960dfa`: `1`
  - `0b3924dbe6746ffd29368d50e913919b33b6630c`: `1`
  - `1e123daf693887c3baea99e0f57e0cf4a147d550`: `1`

Mixed newest-40 aggregates:
- Tracking:
  - `carControl.actuators.accel -> carState.aEgo`
    - all: delay `0.229s`, RMSE `0.169 m/s^2`, bias `+0.011`, corr `0.957`
    - accel-only: delay `0.245s`, RMSE `0.179`, bias `-0.026`, corr `0.840`
    - brake-only: delay `0.207s`, RMSE `0.161`, bias `+0.038`, corr `0.903`
  - `longitudinalPlan.aTarget -> carState.aEgo`
    - all: delay `0.041s`, RMSE `0.186 m/s^2`, bias `+0.007`, corr `0.947`
- Braking:
  - `stop_final_5s` (`48` events)
    - median actual over-brake peak: `0.244 m/s^2`
    - p95 actual over-brake peak: `0.530`
    - median actual jerk max: `1.322 m/s^3`
    - p95 actual jerk max: `4.372`
  - `lead_decel_response` (`143` events)
    - median actual over-brake peak: `0.251 m/s^2`
    - p95 actual over-brake peak: `0.801`
    - median actual jerk max: `2.988 m/s^3`
    - p95 actual jerk max: `8.313`

Clean newest homogeneous cohort:
- The newest homogeneous commit visible in the synced routes is:
  - `04617c4c95bba0d786573909f4f81d01d57adf84`
  - commit title: `cem: lower urban speed trigger to 39 kph`
- This `20`-route cohort is the best current "before" baseline because it removes most history-mixing noise.
- Important relationship to the Santa Fe brake-alignment change:
  - `04617c4...` is `11` commits older than `6f002dad20`
  - therefore none of these routes validate the Santa Fe PID braking change yet

Clean-20 tracking baseline:
- `carControl.actuators.accel -> carState.aEgo`
  - all: delay `0.233s`, RMSE `0.163 m/s^2`, bias `+0.028`, corr `0.969`
  - accel-only: delay `0.259s`, RMSE `0.188`, bias `-0.023`, corr `0.886`
  - brake-only: delay `0.200s`, RMSE `0.149`, bias `+0.052`, corr `0.943`
- `longitudinalPlan.aTarget -> carState.aEgo`
  - all: delay `0.000s`, RMSE `0.175 m/s^2`, bias `+0.022`, corr `0.963`
  - brake-only: delay `0.020s`, RMSE `0.158`, bias `+0.048`, corr `0.935`

Clean-20 braking baseline:
- `stop_final_5s` (`15` events)
  - median actual over-brake peak: `0.225 m/s^2`
  - p95 actual over-brake peak: `0.410`
  - median actual jerk max: `1.450 m/s^3`
  - p95 actual jerk max: `3.389`
- `lead_decel_response` (`37` events)
  - median actual over-brake peak: `0.180 m/s^2`
  - p95 actual over-brake peak: `0.491`
  - median actual jerk max: `2.526 m/s^3`
  - p95 actual jerk max: `7.001`

Interpretation:
- The full-cache refresh improved confidence in the historical baseline.
- The Santa Fe still does not look globally mistimed enough to justify actuator-delay-first tuning.
- Lead-decel remains the rougher lane versus terminal stop even in the cleaner homogeneous cohort.
- The larger mixed-history sample shows an ugly tail, but the cleaner `04617c4` cohort is the right frozen comparison point for the next validation drive.
- The current blocker is no longer missing route data.
  - The blocker is the absence of any synced route recorded on `6f002da` or newer.

Updated next actions:
1. Drive the car on the currently deployed branch and deliberately capture several lead-decel reactions.
2. Refresh the shared cache again after that drive.
3. Re-run both analyzers on explicit post-`6f002da` routes only.
4. Compare first against the clean `20260409T211007Z` baseline, then against the broader `20260409T210811Z` tail metrics.
5. Keep the Santa Fe PID clamp only if lead-decel over-brake / jerk improves without creating a clear under-brake regression.

### 2026-04-10: Canonical `realdata` refresh found the first post-fix Santa Fe route, but it is not yet a validation route

What was done:
- Re-read the updated shared route-refresh contract and switched the longitudinal workflow back to the canonical remote root:
  - `/data/media/0/realdata`
- Refreshed the shared cache with the default canonical-root workflow:
  - `python3.11 tools/route_sync/refresh_routes.py --host commawifi --include-rlog --newest-first --max-downloads 160`
- Confirmed the new canonical-root report:
  - remote files: `4`
  - new: `1`
  - changed: `0`
  - unchanged: `3`
  - downloaded: `1`
  - failures: `0`
- Found the newest canonical-root route segment:
  - `00000007--806ca1e3c9--13`
- The cached `qlog.zst` and sibling `rlog.zst` for that segment were both truncated/incomplete as `.zst` frames.
- Updated the longitudinal analyzer log reader so the cycle can continue on partial current-drive segments:
  - it now accepts route-log files, not only qlogs
  - it falls back from a bad `qlog*` to the sibling `rlog*`
  - it keeps the valid readable prefix of a truncated `.zst` file instead of failing the whole segment
- Re-ran both route-specific analyzers on the new canonical-root route after that reader improvement.

Artifacts:
- Canonical refresh report:
  - `~/.comma/route_sync/reports/route_refresh_commawifi_20260410T170222Z.json`
- New-route braking summary:
  - `~/.comma/longitudinal_tuning/braking/commawifi/route_00000007_20260410T170608Z/summary.json`
  - `~/.comma/longitudinal_tuning/braking/commawifi/route_00000007_20260410T170608Z/summary.md`
- New-route tracking summary:
  - `~/.comma/longitudinal_tuning/analysis/commawifi/route_00000007_20260410T170608Z/summary.json`
  - `~/.comma/longitudinal_tuning/analysis/commawifi/route_00000007_20260410T170608Z/summary.md`

What the new route proves:
- Route:
  - `00000007--806ca1e3c9`
- Fingerprint:
  - `HYUNDAI_SANTA_FE_HEV_2022`
- Commit:
  - `03526cfaafe6c2d8cf3c34138f9e0d2d1acb6ffb`
- Coverage:
  - `1` segment
  - `47.1s` total duration
  - `0.0s` engaged longitudinal time
  - `32.6s` pedal override time
- Event coverage:
  - `0` `lead_decel_response` events
  - `0` `stop_final_5s` events

Interpretation:
- The path correction was real: the current device is writing the newest Santa Fe route under canonical `realdata`, not only under the older `realdata_konik` path.
- We now have proof that the device has recorded a post-fix Santa Fe route on the branch containing the brake-alignment change.
- That route is not usable for braking validation because it contains no engaged longitudinal window at all.
- So the project state changed from "no visible post-fix Santa Fe route" to "post-fix route confirmed, but still no post-fix engaged braking evidence."
- The current braking conclusion therefore does not change:
  - the last meaningful braking review is still the pre-fix route `000009cc--94242f81db`
  - the best frozen comparison baseline is still `20260409T211007Z`

Updated next actions:
1. Capture at least one engaged lead-decel route on the current branch, ideally with several clean lead slowdowns.
2. Refresh again with the default canonical-root workflow, not `realdata_konik`.
3. Re-run the two analyzers on explicit post-fix routes only.
4. Compare those routes first against the frozen clean baseline, then against the pre-fix route `000009cc--94242f81db`.

### 2026-04-10: Route-sync contract changed again; live code says merged `~/.route_sync` cache, but there is still no engaged post-fix Santa Fe route

What was done:
- Re-reviewed the route-download docs after the workflow changed again.
- Confirmed the docs are currently inconsistent:
  - `docs/route_refresh_process.md` still describes canonical-only `realdata` under the older `~/.comma/route_sync/...` layout
  - `tools/route_sync/README.md` and the actual route-sync code now use:
    - shared local root: `~/.route_sync`
    - default remote root scan set:
      - `/data/media/0/realdata`
      - `/data/media/0/realdata_HD`
      - `/data/media/0/realdata_konik`
    - canonicalized local storage under:
      - `~/.route_sync/data/media/0/realdata/`
- Treated the live code as the operative contract for the longitudinal cycle and refreshed/verified against that.
- Read the newest merged-cache reports under `~/.route_sync/reports`.
- Scanned the merged cache for Santa Fe routes on the currently deployed branch by reading `initData.gitCommit` and `carParams.carFingerprint` from route logs.

Artifacts:
- Merged-cache refresh verification:
  - `~/.route_sync/reports/route_refresh_commawifi_20260410T175435Z.json`
  - `~/.route_sync/reports/route_refresh_commawifi_20260410T175340Z.json`
  - `~/.route_sync/reports/route_refresh_commawifi_20260410T173959Z.json`

Merged-cache verification result:
- remote roots scanned:
  - `/data/media/0/realdata`
  - `/data/media/0/realdata_HD`
  - `/data/media/0/realdata_konik`
- file set:
  - qlog-family only
- counts:
  - remote files: `4628`
  - new: `0`
  - changed: `0`
  - unchanged: `4628`
  - downloaded: `0`
  - failures: `0`

Post-fix route discovery result from the merged cache:
- Visible Santa Fe routes on commit `03526cfaafe6c2d8cf3c34138f9e0d2d1acb6ffb`:
  - `00000007--806ca1e3c9`
- No additional Santa Fe routes on that commit were visible in the merged cache.
- So after the route-sync contract change, the post-fix longitudinal evidence still consists of exactly one route:
  - `00000007--806ca1e3c9`
  - `1` segment
  - `47.1s` total duration
  - `0.0s` engaged longitudinal time
  - `0` braking-validation events

Interpretation:
- The new route-download contract is now effectively defined by the code, not by the stale canonical-only process doc.
- The merged `~/.route_sync` cache is healthy and complete enough for route selection work.
- The problem is no longer route-root ambiguity.
- The blocking issue remains the same:
  - there is still no engaged post-fix Santa Fe route in the merged cache
  - therefore there is still no route that can validate or reject the Santa Fe brake-alignment clamp

Updated next actions:
1. Capture one or more engaged Santa Fe routes on the deployed branch with deliberate lead-decel reactions.
2. Refresh with the current live route-sync contract, which now means the merged `~/.route_sync` cache.
3. Re-run the two analyzers on those explicit post-fix routes.
4. Compare first against the frozen clean baseline and then against pre-fix route `000009cc--94242f81db`.
