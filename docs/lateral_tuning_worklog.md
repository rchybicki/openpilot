# Lateral Tuning Worklog

## 2026-04-09: Initial Santa Fe HEV 2022 baseline

- Trigger:
  - Start a lateral improvement cycle for the user's car only: `HYUNDAI_SANTA_FE_HEV_2022`.
  - Use the shared route download workflow and model the documentation/process after the stopping project.

- Repo research:
  - Confirmed Hyundai torque-control path in `opendbc_repo/opendbc/car/hyundai/interface.py`.
  - Confirmed stock Santa Fe HEV torque params in `opendbc_repo/opendbc/car/torque_data/params.toml`.
  - Confirmed runtime requested-vs-actual signals in `selfdrive/controls/lib/latcontrol_torque.py` and `cereal/log.capnp`.
  - Confirmed live adaptation paths in `selfdrive/locationd/torqued.py` and `selfdrive/locationd/paramsd.py`.
  - Existing plotting layouts already cover the right signals:
    - `tools/plotjuggler/layouts/torque-controller.xml`
    - `tools/jotpluggler/layouts/torque-controller.yaml`

- External reference used for orientation:
  - `comma-steering-control` from comma.ai: <https://github.com/commaai/comma-steering-control>

- Route intake:
  - Command:
    - `python tools/route_sync/refresh_routes.py --host commawifi --remote-root /data/media/0/realdata_konik --max-downloads 40 --newest-first`
  - Result:
    - `commawifi` was unavailable, refresh fell back to `comma`
    - remote files discovered: `4626`
    - downloaded: `40`
    - failures: `0`
  - Refresh artifact:
    - `~/.comma/route_sync/reports/route_refresh_commawifi_20260409T145257Z.json`

- New tooling added:
  - `tools/lateral/analyze_lateral_tuning.py`
  - `tools/lateral/README.md`
  - `docs/lateral_tuning_status.md`
  - `docs/lateral_tuning_worklog.md`

- Baseline analysis command:
  - `python tools/lateral/analyze_lateral_tuning.py --host commawifi --car-fingerprint HYUNDAI_SANTA_FE_HEV_2022 --max-routes 3`

- Baseline analysis artifact:
  - `~/.comma/lateral_tuning/analysis/commawifi/HYUNDAI_SANTA_FE_HEV_2022/20260409T151009Z/summary.json`
  - `~/.comma/lateral_tuning/analysis/commawifi/HYUNDAI_SANTA_FE_HEV_2022/20260409T151009Z/summary.md`

- Routes analyzed:
  - `000009cc--94242f81db`
  - `000009cb--55a0e11719`
  - `000009ca--2b7b178788`
  - all three routes fingerprinted as `HYUNDAI_SANTA_FE_HEV_2022`

- Current baseline:
  - all active tracking: `16267` samples, controller MAE `0.052 m/s²`, median ratio `0.982`, saturation `0.18%`
  - turning slice (`|desired lat accel| >= 0.8`): `1309` samples / `130.81 s`, MAE `0.123`, RMSE `0.168`, median ratio `0.990`, under-response `< 0.8` ratio `3.44%`, saturation `2.22%`
  - strong-turn slice (`|desired lat accel| >= 1.5`): `385` samples / `38.49 s`, MAE `0.154`, median ratio `0.988`, saturation `7.53%`
  - weakest speed band is `10-15 m/s`: MAE `0.153`, median ratio `0.959`, under-response `< 0.8` ratio `10.16%`, saturation `7.75%`

- Stock vs live-learned values:
  - stock `latAccelFactor=3.50188`; live median `2.63000`
  - stock `steerRatio=16.55`; live median `15.00`
  - stock friction `0.10384`; live median `0.11187`
  - live `latAccelOffset` median `-0.11999`
  - delay seed `0.30000`; live lateral delay median `0.29519`
  - live torque path was fully active on this slice: `useParams=1.0`, `liveValid=1.0`

- Repeated issue windows:
  - `000009cc--94242f81db` segment `11`, `681.71-682.51 s`: desired `1.679`, actual `0.986`, ratio `0.587`
  - `000009cb--55a0e11719` segment `11`, `669.08-669.57 s`: desired `1.303`, actual `0.669`, ratio `0.514`
  - `000009cc--94242f81db` segment `8`, `500.61-501.01 s`: desired `1.298`, actual `0.702`, ratio `0.541`
  - repeated misses are medium-speed and mostly unsaturated; they do not currently look like a global high-speed limit issue

- Decision:
  - Do not change static Hyundai lateral tune yet.
  - The first cycle says the car tracks requested turning reasonably well once live learning is active.
  - The static offline torque entry looks mis-centered relative to the settled live value, but there is not enough repeated evidence yet to justify editing `params.toml`.

- Next steps:
  - collect another fresh route batch and rerun the analyzer on the newest `3-5` Santa Fe HEV routes
  - freeze a small holdout once a clean repeated issue set exists
  - manually inspect the repeated medium-speed windows in PlotJuggler before testing a static torque-data change

## 2026-04-09: Broadened local review beyond the first 3 routes

- Trigger:
  - User correctly pointed out that the device has far more history than the first 3-route quick baseline.

- Local cache expansion:
  - Started a broader pull:
    - `python tools/route_sync/refresh_routes.py --host commawifi --remote-root /data/media/0/realdata_konik --max-downloads 300 --newest-first --spread-routes`
  - The pull was still in progress during this review, but enough qlogs had landed to widen the local Santa Fe slice materially.

- Additional local fingerprint review:
  - Current readable local cache already contained `44` routes fingerprinted as `HYUNDAI_SANTA_FE_HEV_2022`.
  - Some newly landing routes were still unreadable / partial during copy, so the analyzer was updated to skip incomplete routes instead of aborting.

- Tooling update:
  - `tools/lateral/analyze_lateral_tuning.py` now skips partially synced routes/segments that do not yet expose readable `carParams` or full event streams.

- Broadened analysis command:
  - `python tools/lateral/analyze_lateral_tuning.py --host commawifi --car-fingerprint HYUNDAI_SANTA_FE_HEV_2022 --max-routes 44`

- Broadened analysis artifact:
  - `~/.comma/lateral_tuning/analysis/commawifi/HYUNDAI_SANTA_FE_HEV_2022/20260409T153719Z/summary.json`

- Broadened baseline:
  - routes analyzed: `44`
  - all active: `18656` samples
  - turning slice: `1363` samples / `136.19 s`, MAE `0.130`, RMSE `0.180`, median ratio `0.988`, under-response `< 0.8` ratio `4.48%`, saturation `2.13%`
  - strong-turn slice: `397` samples / `39.69 s`, MAE `0.155`, median ratio `0.986`, saturation `7.30%`

- Speed-band update:
  - low-speed `2.5-5 m/s` now looks worse than the first quick pass suggested:
    - MAE `0.273`
    - median ratio `0.859`
    - under-response `< 0.8` ratio `21.33%`
  - medium-speed `10-15 m/s` remains a real weakness:
    - MAE `0.153`
    - median ratio `0.959`
    - under-response `< 0.8` ratio `10.16%`
    - saturation `7.75%`

- Top issue windows on the broader slice:
  - `00000984--f68404ddf1` segment `20`, `1220.38-1221.08 s`: desired `-1.069`, actual `-0.230`, ratio `0.215`
  - `000009cc--94242f81db` segment `11`, `681.71-682.51 s`: desired `1.679`, actual `0.986`, ratio `0.587`
  - `000009cb--55a0e11719` segment `11`, `669.08-669.57 s`: desired `1.303`, actual `0.669`, ratio `0.514`

- Param comparison on the broader slice:
  - stock `latAccelFactor=3.50188`; live median `2.63000`
  - stock `steerRatio=16.55`; live median `15.00`
  - stock friction `0.10384`; live median `0.10970`
  - live `latAccelOffset` median `-0.10879`
  - live delay median `0.29519`

- Decision:
  - The broader slice still does not justify an immediate blind static retune.
  - The "good but not finished" judgment holds.
  - The stronger next target is low-speed / medium-speed under-response review, not delay tuning.

## 2026-04-09: Full local Santa Fe HEV history review and first seed correction

- Trigger:
  - Local route sync had grown large enough to review the full readable Santa Fe HEV history from the device instead of a narrow recent slice.

- Full-history analysis command:
  - `python tools/lateral/analyze_lateral_tuning.py --host commawifi --car-fingerprint HYUNDAI_SANTA_FE_HEV_2022 --max-routes 607`

- Full-history analysis artifact:
  - `~/.comma/lateral_tuning/analysis/commawifi/HYUNDAI_SANTA_FE_HEV_2022/20260409T163148Z/summary.json`

- Full-history baseline:
  - routes analyzed: `607`
  - all active: `2700594` samples / `1763.12 s`, MAE `0.056`, median ratio `1.027`
  - turning slice: `260408` samples / `1687.90 s`, MAE `0.129`, RMSE `0.176`, median ratio `1.026`, under-response `< 0.8` ratio `1.98%`, saturation `0.63%`
  - strong-turn slice: `96813` samples / `1447.31 s`, MAE `0.161`, median ratio `1.018`, saturation `1.69%`

- Full-history speed-band view:
  - `2.5-5 m/s`: MAE `0.155`, median ratio `0.927`, under-response `< 0.8` ratio `5.92%`
  - `5-10 m/s`: MAE `0.170`, median ratio `0.978`, under-response `< 0.8` ratio `5.41%`
  - `10-15 m/s`: MAE `0.130`, median ratio `1.006`
  - `15-25 m/s`: MAE `0.119`, median ratio `1.044`
  - `25+ m/s`: MAE `0.096`, median ratio `1.060`

- Key interpretation change from the 44-route slice:
  - The broader history says the main weak zone is `2.5-10 m/s`, not a persistent `10-15 m/s` problem.
  - Above `10 m/s`, the car is broadly centered and slightly over-responsive rather than globally weak.
  - The worst issue windows are commonly `steer-limited=true` while `saturated=false`, so many misses are happening in limited windows rather than because the global torque map is universally too weak.

- Live-parameter review:
  - stock `latAccelFactor=3.50188`; full-history live median `2.63000`
  - stock `steerRatio=16.55`; full-history live median `15.00`
  - stock friction `0.10384`; full-history live median `0.10384`
  - live `latAccelOffset` median `-0.06567`
  - live lateral delay median `0.33958`, but the newest routes still cluster around `0.292-0.295`
  - route history clearly splits into two regimes:
    - a short stock-like period around `latAccelFactor=3.50`
    - a long stable learned regime around `latAccelFactor=2.63` and `steerRatio=15.0`

- Current device-state verification:
  - The device currently reports:
    - `AdvancedLateralTune=0`
    - `ForceAutoTune=0`
    - `ForceAutoTuneOff=0`
    - `SteerDelay=0.100000`
    - `SteerFriction=0.103841`
    - `SteerLatAccel=3.501878`
    - `SteerRatio=16.549999`
  - Interpretation:
    - current user-facing FrogPilot lateral overrides are not forcing the logged `2.63 / 15.0` behavior
    - the stable logged values are coming from the live parameter paths, not from an active manual UI override

- Route-regime comparison:
  - stock-like routes with usable turning data were rare, but they were materially worse:
    - `latAccelFactor=3.50`: weighted turning MAE `0.161`, median ratio `0.940`
    - learned-like `latAccelFactor<=2.70`: weighted turning MAE `0.127`, median ratio `1.024`
  - This is enough evidence to stop treating `3.50 / 16.55` as a good Santa Fe HEV static starting point.

- Decision:
  - Apply a Santa Fe HEV-only seed correction:
    - `opendbc_repo/opendbc/car/hyundai/values.py`: give `HYUNDAI_SANTA_FE_HEV_2022` its own `steerRatio=15.0`
    - `opendbc_repo/opendbc/car/torque_data/params.toml`: set `HYUNDAI_SANTA_FE_HEV_2022` `LAT_ACCEL_FACTOR` to `2.63`
  - Do not change friction.
  - Do not change `steerActuatorDelay`.
  - Treat this as a baseline-alignment improvement, not as the final answer to the remaining low-speed limited windows.

## 2026-04-09: Added before/after comparison tooling for the next drive

- New tool:
  - `tools/lateral/compare_lateral_runs.py`

- Purpose:
  - Compare two lateral `summary.json` artifacts directly instead of re-judging every cycle by eye.
  - Report deltas for:
    - all-active, turning, and strong-turn tracking slices
    - turning speed bins
    - learned-vs-static parameter alignment
    - top issue windows from the after run

- README update:
  - Added comparison-step documentation and command examples to `tools/lateral/README.md`.

- Smoke test:
  - Ran:
    - `python tools/lateral/compare_lateral_runs.py --before ~/.comma/lateral_tuning/analysis/commawifi/HYUNDAI_SANTA_FE_HEV_2022/20260409T153719Z/summary.json --after ~/.comma/lateral_tuning/analysis/commawifi/HYUNDAI_SANTA_FE_HEV_2022/20260409T163148Z/summary.json`
  - Result:
    - tool executed successfully
    - confirmed the broader 607-route slice materially changes the low-speed/medium-speed interpretation relative to the earlier 44-route snapshot

## 2026-04-09: Post-recalibration live-learner check

- Trigger:
  - User noted a recalibration happened on `2026-04-09` and asked whether live tuning had completed afterwards.

- Shared route refresh follow-up:
  - `python tools/route_sync/refresh_routes.py --host commawifi --remote-root /data/media/0/realdata_konik --max-downloads 240 --newest-first --spread-routes`
    - result: no new `realdata_konik` files; newest synced Santa Fe routes still ended on `2026-04-02`
    - report: `~/.comma/route_sync/reports/route_refresh_commawifi_20260409T164328Z.json`
  - `python tools/route_sync/refresh_routes.py --host commawifi --dry-run --newest-first`
    - result: one new route appeared under the shared default roots, not under `realdata_konik`
    - new route: `00000003--7f9abcaaac`
    - new segment: `00000003--7f9abcaaac--10`
    - report: `~/.comma/route_sync/reports/route_refresh_commawifi_20260409T164432Z.json`
  - `python tools/route_sync/refresh_routes.py --host commawifi --remote-root /data/media/0/realdata --max-downloads 20 --newest-first`
  - `python tools/route_sync/refresh_routes.py --host commawifi --remote-root /data/media/0/realdata --file-name rlog.zst --max-downloads 20 --newest-first`
    - reports:
      - `~/.comma/route_sync/reports/route_refresh_commawifi_20260409T164629Z.json`
      - `~/.comma/route_sync/reports/route_refresh_commawifi_20260409T164715Z.json`

- Fresh route state:
  - newest segment mtime on-device: `2026-04-09T14:14:47Z`
  - only segment currently visible was `00000003--7f9abcaaac--10` under `/data/media/0/realdata`
  - both synced `qlog.zst` and `rlog.zst` were truncated (`zstd: premature end`)
  - readable partial data only covered a low-speed near-stop slice, so this route is not yet a valid requested-vs-actual turning quality review target

- Partial route learner-state check:
  - `liveCalibration` in the partial qlog was already complete:
    - `calStatus=calibrated`
    - `calPerc=100`
    - `validBlocks=50`
  - partial qlog `liveTorqueParameters` said:
    - `useParams=1`
    - `liveValid=0`
    - filtered `latAccelFactor=3.50188`
    - filtered `friction=0.10384`
    - filtered `latAccelOffset=0.0`
    - `totalBucketPoints=2047`
    - `calPerc=50`
    - `maxResets=1`
  - partial qlog `liveParameters` said:
    - `valid=1`
    - `steerRatio≈16.00`
    - `stiffness≈1.018`
    - `lateralDelay=0.300`

- Direct device param check after refresh:
  - copied current param files from the device and decoded them locally
  - `CarParamsPrevRoute`:
    - `carFingerprint=HYUNDAI_SANTA_FE_HEV_2022`
    - stock `steerRatio=16.55`
    - stock `latAccelFactor=3.50188`
  - `LiveTorqueParameters`:
    - `liveValid=0`
    - `useParams=1`
    - filtered `latAccelFactor=3.50188`
    - filtered `friction=0.10384`
    - filtered `latAccelOffset=0.0`
    - `totalBucketPoints=4315`
    - `calPerc=59`
    - `maxResets=1`
  - `LiveParametersV2`:
    - `valid=1`
    - `steerRatio=15.04556`
    - `stiffness=0.99927`
    - `angleOffsetAverage=-0.13609`

- Interpretation:
  - Camera recalibration is not the blocker anymore; that part is already complete.
  - Yes: live torque tuning did not complete after the recalibration, at least not yet on the device state inspected here.
  - The steer-ratio learner is already valid again or persisted correctly, but `torqued` is still on the stock filtered torque values.
  - Because `selfdrive/controls/controlsd.py` applies filtered live torque params whenever `useParams=1`, the current runtime torque path is effectively stock until `LiveTorqueParameters.liveValid` goes true again or the filtered value converges.

- Decision:
  - Do not judge `2026-04-09` turning quality from `00000003--7f9abcaaac` yet.
  - Wait for this route to finalize or collect one more completed fresh route after the recalibration drive, then rerun the lateral analyzer.
  - Keep the Santa Fe HEV seed correction direction: this exact invalid-live-torque state is part of why better static seeds are worth landing.
