# Longitudinal Tuning Workflow

Scripts in this folder support the Santa Fe longitudinal improvement loop after the shared route-refresh step has populated the local route cache.

## Operating Contract

- Current project scope is one car only: `HYUNDAI_SANTA_FE_HEV_2022`.
- Status / direction lives in `docs/longitudinal_tuning_status.md`.
- Evidence / chronology lives in `docs/longitudinal_tuning_worklog.md`.
- Shared route intake contract lives in `docs/route_refresh_process.md`.
- Runtime source of truth is:
  - `selfdrive/controls/lib/longcontrol.py`
  - `selfdrive/controls/lib/longitudinal_planner.py`
  - `frogpilot/common/frogpilot_variables.py`
  - `opendbc_repo/opendbc/car/hyundai/interface.py`

Do not treat this workflow as a generic multi-car tuning lane. A change only counts if it is justified by Santa Fe data and validated on fresh Santa Fe routes.

## Improvement Cycle

1. Refresh the shared route cache from the device.
   - Example:
     `python tools/route_sync/refresh_routes.py --host commawifi --max-downloads 200 --newest-first`
2. Snapshot active longitudinal settings from the device.
   - Example:
     `python3.11 tools/stopping/device_stop_settings.py snapshot --host commawifi`
3. Run requested-vs-actual tracking analysis.
   - Example:
     `python3.11 tools/longitudinal/analyze_longitudinal_tracking.py --host commawifi --max-routes 25 --min-route-segments 5`
4. Run braking-focused analysis.
   - Example:
     `python3.11 tools/longitudinal/analyze_braking_focus.py --host commawifi --max-routes 25 --min-route-segments 5`
5. Review aggregate metrics before chasing individual windows.
   - `accel_cmd -> aEgo` is the main controller/output lane.
   - `aTarget -> aEgo` is the planner + controller lane.
6. Review worst windows and route clusters.
   - Separate launch / accel under-response from lead-decel braking behavior.
7. Make one scoped Santa Fe-only change.
   - Prefer Santa Fe-only runtime shaping or Santa Fe-specific FrogPilot settings before broad controller changes.
8. Deploy and collect fresh routes.
   - A historical-cache improvement alone does not complete the iteration.
9. Re-run the same analysis and write the result into `docs/longitudinal_tuning_worklog.md`.

If a step is skipped, the iteration is incomplete.

## Route Refresh Notes

- Shared route refresh is owned by `tools/route_sync/refresh_routes.py`.
- Refresh is incremental for a given `--host`, `--download-root`, and `--state-file`.
  - unchanged files are skipped when remote `size` and `mtime` still match the cached state and the local file exists
  - missing local files are re-downloaded
  - changed remote files are re-downloaded
- Each run still has to list the full remote candidate set over SSH, so discovery cost is still proportional to all remote log files even when download cost is only proportional to new files.
- `commawifi` and `comma` now share the same local cache/state identity.
  - Swapping aliases should no longer cause duplicate downloads.
  - It does not remove the full remote discovery cost.

## Scripts

- `analyze_longitudinal_tracking.py`
  - Scores requested-vs-actual acceleration tracking from locally cached qlogs.
  - Main outputs:
    - aggregate delay / RMSE / bias / correlation
    - route ranking
    - worst mismatch windows

- `analyze_braking_focus.py`
  - Scores braking comfort and over-correction from locally cached qlogs.
  - Main outputs:
    - `stop_final_5s`
    - `lead_decel_response`

- `tools/longitudinal_maneuvers/README.md`
  - Upstream-style controlled maneuver workflow.
  - Use this when you need a cleaner actuator-lag / response-shape signal than mixed real traffic can provide.

## Default Local Paths

- Route download root: `~/.comma/route_sync/downloads`
- Route refresh state: `~/.comma/route_sync/state.json`
- Route refresh reports: `~/.comma/route_sync/reports`
- Longitudinal tracking analysis: `~/.comma/longitudinal_tuning/analysis`
- Braking analysis: `~/.comma/longitudinal_tuning/braking`

## Typical Commands

Refresh new local routes:

```bash
python tools/route_sync/refresh_routes.py \
  --host commawifi \
  --max-downloads 200 \
  --newest-first
```

Snapshot current longitudinal settings:

```bash
python3.11 tools/stopping/device_stop_settings.py snapshot --host commawifi
```

Run tracking analysis:

```bash
python3.11 tools/longitudinal/analyze_longitudinal_tracking.py \
  --host commawifi \
  --max-routes 25 \
  --min-route-segments 5
```

Run braking analysis:

```bash
python3.11 tools/longitudinal/analyze_braking_focus.py \
  --host commawifi \
  --max-routes 25 \
  --min-route-segments 5
```

Run a controlled maneuver route report after collecting a dedicated route:

```bash
python tools/longitudinal_maneuvers/generate_report.py <route_id>
```
