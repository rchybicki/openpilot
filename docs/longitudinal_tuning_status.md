# Hyundai Santa Fe HEV 2022 Longitudinal Tuning: Status and Direction

- Updated: 2026-04-10
- Scope: OpenPilot/FrogPilot longitudinal tuning for `HYUNDAI_SANTA_FE_HEV_2022` only
- Worklog (commands, artifacts, decisions): `docs/longitudinal_tuning_worklog.md`
- Tooling workflow: `tools/longitudinal/README.md`
- Shared route refresh contract: `docs/route_refresh_process.md`

## Problem Statement

The Santa Fe does not look globally mistuned. Across the current local historical sample, requested-vs-actual tracking is already fairly tight most of the time, but the biggest user-visible misses are concentrated in a smaller set of launch and sudden lead-decel windows.

Primary failure lanes right now:

- repeated accel under-response on some launches / gap-close events
- sharp, inconsistent brake corrections during sudden lead decel
- some final-stop roughness, but that is not the dominant lane in the current data

## Scope and Constraints

In scope:

- Santa Fe only
- requested-vs-actual acceleration tracking
- controller behavior that brakes materially deeper than the model request
- FrogPilot longitudinal settings and Santa Fe-specific runtime shaping

Out of scope unless later evidence forces it:

- broad multi-car longitudinal changes
- actuator-delay-first tuning without proof of a true global lag problem
- planner/model redesign before the controller/request mismatch is narrowed down

Acceptance constraints for the next completed iteration:

- improve `lead_decel_response` over-brake and jerk
- do not materially worsen `actual_underbrake_peak_mps2`
- keep `stop_final_5s` flat or better
- validate on fresh post-change routes, not only on the historical cache

## Current Runtime and Settings Snapshot

Relevant runtime/control sources:

- `selfdrive/controls/lib/longcontrol.py`
- `selfdrive/controls/lib/longitudinal_planner.py`
- `frogpilot/common/frogpilot_variables.py`
- `opendbc_repo/opendbc/car/hyundai/interface.py`

Important current constraints from the code:

- Hyundai baseline still sets `ret.longitudinalActuatorDelay = 0.5` in the interface.
- FrogPilot only applies `LongitudinalActuatorDelay` and `MaxDesiredAcceleration` overrides when `AdvancedLongitudinalTune` is enabled.
- The current device snapshot used for this project had:
  - `AdvancedLongitudinalTune=False`
  - `AccelerationProfile=Eco`
  - `DecelerationProfile=Eco`
  - `HumanAcceleration=True`
  - `HumanFollowing=True`

Project implication:

- The current baseline is mostly stock Hyundai/FrogPilot longitudinal behavior with gentle profile choices.
- A pure actuator-delay interpretation is therefore too aggressive as a first diagnosis.

## Current Evidence

Shared cache state:

- route-sync cache is now fully backfilled for the device route root
- verification artifact: `~/.comma/route_sync/reports/route_refresh_commawifi_20260409T164456Z.json`
- verified remote files: `4626`
- all verified as locally present: `4626`
- route-sync code and docs diverged again on `2026-04-10`; the operative contract for this cycle was the live code in `tools/route_sync/common.py` / `tools/route_sync/refresh_routes.py`
  - default shared root: `~/.route_sync`
  - default remote roots scanned together:
    - `/data/media/0/realdata`
    - `/data/media/0/realdata_HD`
    - `/data/media/0/realdata_konik`
  - latest merged-cache verification artifact: `~/.route_sync/reports/route_refresh_commawifi_20260410T175435Z.json`
  - latest merged-cache qlog coverage: `4628` remote files, `4628` unchanged locally, `0` missing
- canonical `realdata` refresh on `2026-04-10` found the first visible post-fix Santa Fe route:
  - refresh artifact: `~/.comma/route_sync/reports/route_refresh_commawifi_20260410T170222Z.json`
  - route: `00000007--806ca1e3c9`
  - commit: `03526cfaafe6c2d8cf3c34138f9e0d2d1acb6ffb`
  - fingerprint: `HYUNDAI_SANTA_FE_HEV_2022`
  - route coverage so far: `1` segment, `47.1s` total, `0.0s` engaged
  - implication: this is proof that canonical `realdata` is the right root and that the device is recording post-fix Santa Fe data there, but it is not yet a usable braking-validation route

Best frozen "before" baseline:

- artifact: `~/.comma/longitudinal_tuning/analysis/commawifi/20260409T211007Z/summary.json`
- artifact: `~/.comma/longitudinal_tuning/braking/commawifi/20260409T211007Z/summary.json`
- routes analyzed: `20`
- car fingerprint(s): `HYUNDAI_SANTA_FE_HEV_2022`
- route cohort commit: `04617c4c95bba0d786573909f4f81d01d57adf84`

Why this cohort matters:

- it is the newest clean homogeneous slice in the fully synced cache
- it removes most mixed-history noise from the earlier broader baseline
- it is still pre-Santa-Fe brake-clamp validation
  - `04617c4...` is `11` commits older than `6f002da`

Aggregate requested-vs-actual metrics:

- `carControl.actuators.accel -> carState.aEgo`
  - all: delay `0.233s`, RMSE `0.163 m/s^2`, bias `+0.028`, corr `0.969`
  - accel-only: delay `0.259s`, RMSE `0.188`, bias `-0.023`, corr `0.886`
  - brake-only: delay `0.200s`, RMSE `0.149`, bias `+0.052`, corr `0.943`
- `longitudinalPlan.aTarget -> carState.aEgo`
  - all: delay `0.000s`, RMSE `0.175 m/s^2`, bias `+0.022`, corr `0.963`
  - brake-only: delay `0.020s`, RMSE `0.158`, bias `+0.048`, corr `0.935`

Aggregate braking metrics:

- `stop_final_5s`: `15` events
  - median actual over-brake `0.225 m/s^2`
  - p95 actual over-brake `0.410`
  - median actual jerk `1.450 m/s^3`
  - p95 actual jerk `3.389`
- `lead_decel_response`: `37` events
  - median actual over-brake `0.180 m/s^2`
  - p95 actual over-brake `0.491`
  - median actual jerk `2.526 m/s^3`
  - p95 actual jerk `7.001`

Broader tail-check slice:

- mixed-history artifact: `~/.comma/longitudinal_tuning/analysis/commawifi/20260409T210811Z/summary.json`
- mixed-history artifact: `~/.comma/longitudinal_tuning/braking/commawifi/20260409T210811Z/summary.json`
- routes analyzed: `40`
- use this slice to watch ugly tails, not to judge small iteration deltas

## What The Data Says

- The Santa Fe is not globally bad at tracking.
  - `0.171 m/s^2` RMSE and `0.962` correlation on `accel_cmd -> aEgo` is already respectable for real mixed routes.
- The dominant issue is not a constant lag or constant offset.
  - Overall bias stays small.
  - The ugly behavior is concentrated in a handful of obvious windows.
- The planner appears to be compensating most physical lag already.
  - `aTarget -> aEgo` best delay is near zero in the clean latest cohort, much smaller than the command-to-actual lane.
  - That makes actuator-delay-first tuning a weak first move.
- Lead-decel behavior is the roughest braking lane.
  - Final-stop windows exist, but their median jerk is much lower than the lead-decel lane.
  - The first priority is therefore sudden lead-response smoothing and clamp discipline, not terminal-stop polish.

## Current Direction

Primary focus:

- Santa Fe-only plain-PID brake alignment and smoothing during lead-decel response

Secondary focus:

- launch / positive-accel under-response after the lead-response lane is cleaner

Current runtime experiment already in-tree:

- `selfdrive/controls/lib/longcontrol.py` contains a Santa Fe-only PID brake-alignment clamp for plain PID braking outside stop-target/stopping lanes.
- This is intentionally narrow.
- It is deployed on the device.
- The first synced post-fix Santa Fe route is now visible in canonical `realdata`, but it contains no engaged longitudinal interval and therefore still does not validate the braking change.
- The merged `~/.route_sync` cache confirms the same result after the docs/code change:
  - the only visible Santa Fe route on `03526cfaafe6c2d8cf3c34138f9e0d2d1acb6ffb` is still `00000007--806ca1e3c9`
  - it still has `0.0s` engaged longitudinal time
  - there is still no post-fix engaged Santa Fe braking route to judge the clamp

## Next Step

The next completed iteration should do all of the following:

1. Drive the car on the currently deployed branch and capture fresh engaged lead-decel-heavy Santa Fe routes in canonical `realdata`.
2. Refresh routes with the shared route-sync cache.
3. Re-run:
   - `tools/longitudinal/analyze_longitudinal_tracking.py`
   - `tools/longitudinal/analyze_braking_focus.py`
4. Compare first against the frozen clean baseline:
   - tracking: `20260409T211007Z`
   - braking: `20260409T211007Z`
5. Use the broader mixed-history tails as a secondary guardrail:
   - tracking: `20260409T210811Z`
   - braking: `20260409T210811Z`
6. If lead-decel over-brake improves without an under-brake regression, keep the clamp.
7. If not, adjust the Santa Fe-only margin shape before touching planner behavior or actuator delay.
