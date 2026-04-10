# Lateral Tuning Workflow

Scripts in this folder support route-driven lateral review for the Hyundai Santa Fe HEV 2022 after the shared route refresh step has populated the local qlog cache.

## Operating Contract

- Current scope: `HYUNDAI_SANTA_FE_HEV_2022` only.
- North-star goal: requested turning should match actual turning without obvious under-response, overshoot, delay, or sustained steering saturation.
- Runtime source of truth:
  - `selfdrive/controls/lib/latcontrol_torque.py`
  - `selfdrive/controls/controlsd.py`
  - `selfdrive/locationd/torqued.py`
  - `selfdrive/locationd/paramsd.py`
  - `opendbc_repo/opendbc/car/hyundai/interface.py`
  - `opendbc_repo/opendbc/car/torque_data/params.toml`
- Process source of truth:
  - `docs/route_refresh_process.md`
  - `tools/lateral/README.md`
  - `docs/lateral_tuning_status.md`
  - `docs/lateral_tuning_worklog.md`

## Improvement Cycle

1. Refresh the shared route cache after a drive.
   - prefer the shared default roots first:
     - `python tools/route_sync/refresh_routes.py --host commawifi --max-downloads 80 --newest-first`
   - if you intentionally want only the historical konik tree:
     - `python tools/route_sync/refresh_routes.py --host commawifi --remote-root /data/media/0/realdata_konik --max-downloads 40 --newest-first`
2. Analyze the newest Santa Fe HEV routes.
   - `python tools/lateral/analyze_lateral_tuning.py --host commawifi --car-fingerprint HYUNDAI_SANTA_FE_HEV_2022 --max-routes 3`
3. Review the baseline before touching code.
   - Controller tracking on all active samples, turning samples, and strong-turn samples.
   - Speed-bin breakdown, with special attention to the `10-15 m/s` band.
   - Top issue windows: repeated route/segment windows with large requested-vs-actual mismatch.
   - Stock vs live-learned values for `latAccelFactor`, `friction`, `steerRatio`, and `lateralDelay`.
4. Only tune when the same miss repeats on multiple routes.
   - Do not edit the static Hyundai torque tune because of one isolated turn.
   - If live torque / steer-ratio values settle away from stock across multiple cycles and the same issue windows repeat, test one static change at a time.
5. Compare the new run against the frozen baseline.
   - `python tools/lateral/compare_lateral_runs.py --before ~/.comma/lateral_tuning/analysis/commawifi/HYUNDAI_SANTA_FE_HEV_2022/<before_stamp>/summary.json --after ~/.comma/lateral_tuning/analysis/commawifi/HYUNDAI_SANTA_FE_HEV_2022/<after_stamp>/summary.json`
6. Rerun the analyzer on the frozen route set and update the docs.
   - Keep `docs/lateral_tuning_status.md` as the stable snapshot.
   - Append evidence and decisions to `docs/lateral_tuning_worklog.md`.

## Analyzer Behavior

`analyze_lateral_tuning.py` currently measures torque-controller tracking from synced qlogs:

- uses only `torqueState` samples where lateral control is active,
- excludes standstill, steering override, and active blinkers,
- uses `|desired lateral accel| >= 0.8 m/s²` as the turning slice,
- uses `|desired lateral accel| >= 1.5 m/s²` as the strong-turn slice,
- flags issue windows when `|desired| >= 1.0 m/s²` and either:
  - `|actual - desired| >= 0.4 m/s²`,
  - `|actual| / |desired| < 0.8`,
  - or the controller reports saturation.

Output artifacts:

- summary JSON: `~/.comma/lateral_tuning/analysis/<host>/<car_fingerprint>/<stamp>/summary.json`
- summary Markdown: `~/.comma/lateral_tuning/analysis/<host>/<car_fingerprint>/<stamp>/summary.md`

`compare_lateral_runs.py` consumes two `summary.json` artifacts and reports before/after deltas for:

- all-active, turning, and strong-turn tracking slices,
- turning speed bins,
- learned-vs-static parameter alignment,
- the top issue windows from the after run.

## Current Caveat

- Very fresh active segments can land as truncated `.zst` files. Use them for learner-state triage only and wait for a finalized route before judging turning quality.
- Fresh routes can appear under `/data/media/0/realdata`, not only `/data/media/0/realdata_konik`, so the shared default-root refresh should be the first pass after a drive.
- The synced qlog slice used in this workflow does not currently include `modelV2.meta.laneChangeState`, so blinkers are the lane-change proxy for now.
- `steer_limited_ratio` is a monitoring signal, not yet a hard gate. Treat it as context around bad windows, not a standalone regression decision.
