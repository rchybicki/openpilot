# Lateral Tuning Project: Status and Direction

- Updated: 2026-04-09
- Scope: torque-controller lateral tracking for `HYUNDAI_SANTA_FE_HEV_2022` only
- Worklog: `docs/lateral_tuning_worklog.md`
- Shared route refresh contract: `docs/route_refresh_process.md`
- Tooling workflow: `tools/lateral/README.md`

## Problem Statement

The question for this car is not whether it can steer, but how closely it follows requested turning on real routes, where it still misses, and whether the static Santa Fe HEV setup is aligned with the behavior that the live learners have already settled on.

For this car, the important paths are:

- stock Hyundai torque tune in `opendbc_repo/opendbc/car/torque_data/params.toml`
- stock Hyundai lateral setup in `opendbc_repo/opendbc/car/hyundai/values.py`
- live torque adaptation in `selfdrive/locationd/torqued.py`
- live steer-ratio adaptation in `selfdrive/locationd/paramsd.py`
- runtime tracking in `selfdrive/controls/lib/latcontrol_torque.py`

## Current Baseline

Latest baseline artifact:

- route refresh report: `~/.comma/route_sync/reports/route_refresh_commawifi_20260409T162219Z.json`
- lateral summary: `~/.comma/lateral_tuning/analysis/commawifi/HYUNDAI_SANTA_FE_HEV_2022/20260409T163148Z/summary.json`
- routes analyzed locally so far: `607` Santa Fe HEV routes with readable `carParams`

Current controller tracking snapshot on the blinker-filtered active slice:

- all active: `2700594` samples / `1763.12 s`, MAE `0.056 m/s²`, median ratio `1.027`, saturation `0.06%`
- turning (`|desired lat accel| >= 0.8`): `260408` samples / `1687.90 s`, MAE `0.129 m/s²`, RMSE `0.176`, median ratio `1.026`, under-response `< 0.8` ratio `1.98%`, saturation `0.63%`
- strong turning (`|desired lat accel| >= 1.5`): `96813` samples / `1447.31 s`, MAE `0.161`, median ratio `1.018`, saturation `1.69%`

Weakest current speed bands:

- `2.5-5 m/s`: MAE `0.155 m/s²`, median ratio `0.927`, under-response `< 0.8` ratio `5.92%`
- `5-10 m/s`: MAE `0.170 m/s²`, median ratio `0.978`, under-response `< 0.8` ratio `5.41%`
- above `10 m/s`, tracking is materially better and broadly centered

Repeated issue windows worth manual review:

- `0000069a--cd2937090d` segment `10` around `657.51-657.81 s`: desired `2.314`, actual `-1.134`, worst error `3.448`
- `00000843--20bf9e95b1` segment `18` around `1125.95-1126.05 s`: desired `2.779`, actual `0.912`, worst error `1.867`
- `00000980--cad3c5f357` segment `10` around `609.62-610.52 s`: desired `1.778`, actual `-0.026`, worst error `1.805`
- `00000988--85fdd5ab4b` segment `19` around `1167.84-1168.94 s`: desired `1.547`, actual `-0.169`, worst error `1.716`

Most top issue windows are `steer-limited=true` while `saturated=false`, which points away from a broad torque-map mismatch and toward low-speed / rate-limited steering behavior in those windows.

## Current On-Device Post-Recalibration State

Fresh-device follow-up on `2026-04-09` changed the process picture even though it did not yet add a new valid turning-review route:

- `realdata_konik` had no newer files than the already analyzed April 2, 2026 Santa Fe routes.
- A new route did appear under the shared default roots:
  - route `00000003--7f9abcaaac`
  - segment `00000003--7f9abcaaac--10`
  - newest segment mtime `2026-04-09T14:14:47Z`
- That segment is still truncated on-device (`qlog.zst` and `rlog.zst` both ended with `premature end`), so the readable slice is only useful for learner-state triage, not for requested-vs-actual turning quality review.

What the device says right now:

- partial-route `liveCalibration` is already complete: `calStatus=calibrated`, `calPerc=100`, `validBlocks=50`
- current `CarParamsPrevRoute` still carries the stock Santa Fe HEV seeds: `steerRatio=16.55`, `latAccelFactor=3.50188`
- current `LiveParametersV2` is valid and already back near the learned steer ratio: `steerRatio=15.04556`, `stiffness=0.99927`
- current `LiveTorqueParameters` is not valid yet:
  - `liveValid=0`
  - `useParams=1`
  - filtered `latAccelFactor=3.50188`
  - filtered `friction=0.10384`
  - filtered `latAccelOffset=0.0`
  - `totalBucketPoints=4315`
  - `calPerc=59`

Implication:

- Camera recalibration itself is already finished.
- The torque learner (`torqued`) has not completed reconvergence yet, so the device is currently running the torque controller with stock filtered torque values.
- `selfdrive/controls/controlsd.py` updates torque params whenever `useParams=1`, even before `liveValid` is true, so an invalid learner state still matters if the filtered values are still stock-like.

## Stock Vs Live-Learned Params

The long-history logs say the car spends most of its time in a stable learned state, and that matters more than the stock table during the drive:

- stock `latAccelFactor` is `3.50188`; live median is `2.63000` (`0.751x` stock)
- stock `steerRatio` is `16.55`; live median is `15.00` (`0.906x` stock)
- stock friction is `0.10384`; live median is `0.10384`
- live `latAccelOffset` sits near `-0.066`
- live lateral delay median is `0.340 s`, but the newest routes cluster lower around `0.292-0.295 s`
- `liveTorqueParameters.useParams=1.0`; `liveValid=0.9686`
- current device FrogPilot lateral knobs are still stock-valued with `AdvancedLateralTune=0`, so the logged `2.63 / 15.0` values are not explained by an active manual UI override

Interpretation:

- the stock Santa Fe HEV static seeds are materially off-center relative to the long-running learned state
- friction does not need a Santa Fe-specific change right now
- delay does not look like the first lever to pull
- historically runtime tracking is broadly good because the live torque path is usually active and stable, but the current post-recalibration device state is temporarily back on stock torque values
- the remaining weak area is low-speed / near-limit behavior, not a broad global torque-map failure

## Current Judgment

- No evidence for a broad lateral-control failure on this car.
- The steady-state learned behavior is better than the stock-like state that still used `latAccelFactor=3.50`.
- A Santa Fe HEV-only seed alignment is justified now:
  - `steerRatio` seed should be `15.0` instead of inheriting `16.55`
  - `latAccelFactor` seed should move to `2.63`
- This seed alignment should improve startup / cache-reset behavior, and the April 9, 2026 post-recalibration device check is direct evidence for that benefit.
- The seed alignment is not expected to fully fix the remaining limited low-speed issue windows.

## Next Phase

1. Re-review the next completed fresh Santa Fe HEV route batch after the static seed alignment lands and after `LiveTorqueParameters` is valid again on-device.
2. Freeze a small holdout instead of always chasing only the newest routes.
3. Manually inspect the repeated low-speed steer-limited issue windows with the existing torque-controller PlotJuggler/JotPluggler layouts:
   - `tools/plotjuggler/layouts/torque-controller.xml`
   - `tools/jotpluggler/layouts/torque-controller.yaml`
4. Only if the same low-speed limited misses keep repeating after the seed update, investigate car-specific steering-limit behavior or low-speed controller shaping rather than touching friction or delay first.
