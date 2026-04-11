# Hyundai Santa Fe HEV 2022 Longitudinal Tuning: Status and Direction

- Updated: 2026-04-11
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
- first usable post-fix validation cohort arrived on `2026-04-11`:
  - refresh artifact: `~/.route_sync/reports/route_refresh_commawifi_20260411T095413Z.json`
  - new Santa Fe routes on commit `b1a547dac28804afc0ef6faaae59457f5f55aede`:
    - `00000019--fd616f4757`
    - `0000001a--e7732b5a28`
    - `0000001b--670b297bdc`
    - `0000001c--600d2d8d67`
    - `0000001d--29a0acc670`
  - validation artifacts:
    - `~/.comma/longitudinal_tuning/analysis/commawifi/post_b1a547d_20260411T101146Z/summary.json`
    - `~/.comma/longitudinal_tuning/braking/commawifi/post_b1a547d_filtered_20260411T102303Z/summary.json`
  - engaged coverage inside that cohort:
    - `0000001b`: `1512.8s`
    - `0000001c`: `944.3s`
    - `0000001d`: `676.8s`
    - `00000019`: `8.3s`
    - `0000001a`: `0.0s`

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

First usable post-fix validation cohort:

- tracking artifact: `~/.comma/longitudinal_tuning/analysis/commawifi/post_b1a547d_20260411T101146Z/summary.json`
- braking artifact: `~/.comma/longitudinal_tuning/braking/commawifi/post_b1a547d_filtered_20260411T102303Z/summary.json`
- routes analyzed: `5`
- post-fix route cohort commit: `b1a547dac28804afc0ef6faaae59457f5f55aede`
- braking event scoring now trims each event to controller-owned samples only:
  - engaged
  - no pedal override
  - at least `5` valid control-owned samples
- implication: driver-brake / disengage contamination no longer inflates the lead-decel lane

Post-fix tracking metrics:

- `carControl.actuators.accel -> carState.aEgo`
  - all: delay `0.200s`, RMSE `0.164 m/s^2`, bias `+0.027`, corr `0.979`
  - accel-only: delay `0.127s`, RMSE `0.171`, bias `-0.016`, corr `0.906`
  - brake-only: delay `0.200s`, RMSE `0.156`, bias `+0.058`, corr `0.962`
- `longitudinalPlan.aTarget -> carState.aEgo`
  - all: delay `0.015s`, RMSE `0.180 m/s^2`, bias `+0.024`, corr `0.974`
  - brake-only: delay `0.050s`, RMSE `0.177`, bias `+0.056`, corr `0.950`

Post-fix braking metrics:

- `stop_final_5s`: `16` events
  - median actual over-brake `0.200 m/s^2`
  - p95 actual over-brake `0.453`
  - median actual jerk `1.164 m/s^3`
  - p95 actual jerk `2.201`
- `lead_decel_response`: `55` filtered control-owned events
  - median actual over-brake `0.217 m/s^2`
  - p95 actual over-brake `0.449`
  - median actual jerk `2.703 m/s^3`
  - p95 actual jerk `5.879`

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
- The first usable post-fix validation is mixed, not a clean win.
  - Final-stop behavior improved versus baseline.
  - Lead-decel tails improved somewhat, but median lead-decel over-brake and median jerk got worse.
- In the post-fix cohort, controller output is now usually close to planner request on average.
  - Across `55` filtered lead-decel events, mean `accel_cmd - aTarget` was `-0.026 m/s^2`, median `-0.018`.
  - Using a `0.15 m/s^2` average-delta threshold, `53/55` lead events were near target and `2/55` were deeper.
  - Actual vehicle response was lighter than target in `19/55` lead events on average and deeper in `0/55`.
- That means the main user-visible complaint is no longer well explained by controller-only over-braking.
  - The brake trigger is still primarily planner/request-side.
  - There are still transient controller/plant artifacts, but the worst previously flagged outlier was a disengage/driver-brake artifact rather than controller-owned braking.

## Current Direction

Primary focus:

- planner/request-side lead-decel behavior on Santa Fe, after validating that the controller now broadly tracks the request

Secondary focus:

- targeted transient outlier review in lead-decel events
- launch / positive-accel under-response after the lead-response lane is cleaner

Current runtime experiment already in-tree:

- `selfdrive/controls/lib/longcontrol.py` contains a Santa Fe-only PID brake-alignment clamp for plain PID braking outside stop-target/stopping lanes.
- This is intentionally narrow.
- It is deployed on the device.
- It is now validated on a usable `5`-route post-fix cohort at commit `b1a547dac28804afc0ef6faaae59457f5f55aede`.
- Validation result:
  - good: stop-final behavior improved and controller output is usually near `aTarget`
  - not good enough: typical lead-decel harshness did not improve cleanly
  - implication: do not assume another plain PID clamp adjustment is the best next move

## Next Step

The next completed iteration should do all of the following:

1. Keep the filtered post-fix cohort as the controller-owned braking reference, so disengage and driver-brake artifacts do not drive tuning decisions.
2. For the next software iteration, prioritize planner/request-side lead-decel shaping over another broad plain-PID clamp change.
3. Keep using the same explicit post-fix route analysis flow:
   - `tools/longitudinal/analyze_longitudinal_tracking.py`
   - `tools/longitudinal/analyze_braking_focus.py`
4. Compare first against the frozen clean baseline:
   - tracking: `20260409T211007Z`
   - braking: `20260409T211007Z`
5. Use the current post-fix cohort as the controller-tracking reference point:
   - tracking: `post_b1a547d_20260411T101146Z`
   - braking: `post_b1a547d_filtered_20260411T102303Z`
