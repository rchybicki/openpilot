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

## 2026-04-10: First completed post-recalibration route review

- Local route-cache scan:
  - Checked all local route-refresh reports from `2026-04-10`.
  - No new device downloads were reported in those refreshes:
    - `~/.comma/route_sync/reports/route_refresh_commawifi_20260410T153750Z.json`
    - `~/.comma/route_sync/reports/route_refresh_commawifi_20260410T154003Z.json`
    - `~/.comma/route_sync/reports/route_refresh_commawifi_20260410T154053Z.json`
    - `~/.comma/route_sync/reports/route_refresh_commawifi_20260410T154303Z.json`
    - `~/.comma/route_sync/reports/route_refresh_comma_20260410T154600Z.json`
  - The only route files with fresh local mtimes on `2026-04-10` were:
    - `~/.comma/route_sync/downloads/commawifi/data/media/0/realdata_konik/000009cc--94242f81db--14/qlog.zst`
    - `~/.comma/route_sync/downloads/commawifi/data/media/0/realdata_konik/000009cc--94242f81db--14/rlog.zst`
  - The incomplete `realdata` routes from the prior check remained:
    - `00000003--7f9abcaaac--10`
    - `00000007--806ca1e3c9--13`

- Completed route review:
  - Ran:
    - `python tools/lateral/analyze_lateral_tuning.py --host commawifi --route 000009cc--94242f81db`
  - Artifact:
    - `~/.comma/lateral_tuning/analysis/commawifi/HYUNDAI_SANTA_FE_HEV_2022/20260410T154901Z/summary.json`
    - `~/.comma/lateral_tuning/analysis/commawifi/HYUNDAI_SANTA_FE_HEV_2022/20260410T154901Z/summary.md`

- Route result:
  - Route: `000009cc--94242f81db`
  - Active samples: `6128`
  - Turning samples: `419`
  - Strong-turn samples: `103`
  - All-active tracking:
    - MAE `0.052`
    - median ratio `0.963`
    - saturation `0.0%`
  - Turning tracking:
    - MAE `0.135`
    - RMSE `0.184`
    - median ratio `0.980`
    - under-response `< 0.8` ratio `6.44%`
    - saturation `0.0%`
  - Strong turning:
    - MAE `0.157`
    - median ratio `0.943`
    - under-response `< 0.8` ratio `10.68%`
    - saturation `0.0%`

- Speed-bin picture on this route:
  - `2.5-5 m/s`: MAE `0.172`, median ratio `1.051`, only `7` samples
  - `5-10 m/s`: MAE `0.163`, median ratio `0.906`
  - `10-15 m/s`: MAE `0.173`, median ratio `0.962`
  - `15-25 m/s`: MAE `0.057`, median ratio `1.016`
  - Interpretation:
    - low / medium speed under-response remains the weak area
    - `15-25 m/s` is still clearly better behaved

- Live-parameter state on this route:
  - `liveTorqueParameters.useParams=1.0`
  - `liveTorqueParameters.liveValid=1.0`
  - live `latAccelFactor=2.63000`
  - live `friction=0.11246`
  - live `latAccelOffset=-0.12350`
  - live `steerRatio=15.00000`
  - live `lateralDelay=0.29519`

- Top issue windows on this route:
  - `000009cc--94242f81db` seg `11` around `681.71-682.51 s`: desired `1.679`, actual `0.986`, ratio `0.587`, steer-limited `True`, saturated `False`
  - `000009cc--94242f81db` seg `8` around `500.61-501.01 s`: desired `1.298`, actual `0.702`, ratio `0.541`, steer-limited `True`, saturated `False`
  - `000009cc--94242f81db` seg `8` around `501.61 s`: desired `1.114`, actual `1.585`, ratio `1.423`, steer-limited `True`, saturated `False`
  - `000009cc--94242f81db` seg `11` around `680.71-681.11 s`: desired `1.385`, actual `0.919`, ratio `0.663`, steer-limited `True`, saturated `False`

- Comparison against the 607-route historical baseline:
  - Ran:
    - `python tools/lateral/compare_lateral_runs.py --before ~/.comma/lateral_tuning/analysis/commawifi/HYUNDAI_SANTA_FE_HEV_2022/20260409T163148Z/summary.json --after ~/.comma/lateral_tuning/analysis/commawifi/HYUNDAI_SANTA_FE_HEV_2022/20260410T154901Z/summary.json`
  - Result:
    - this single route is directionally consistent with the long-history picture, not a contradiction of it
    - it is somewhat worse than the long-history average in `5-15 m/s`
    - it is better than average in `15-25 m/s`
    - the issue windows are still mostly `steer-limited=True` without controller saturation

- Interpretation:
  - The post-recalibration invalid-torque period was temporary; this route confirms the torque learner is valid again.
  - The car is back on its learned torque state, not stuck on the stock filtered `3.50` state anymore.
  - Even with `liveValid=1`, this route still shows the same low / medium speed under-response pattern already seen in the broader history.
  - This strengthens the existing view:
    - the Santa Fe HEV static seeds are still mis-centered relative to the stable learned state
    - the remaining problem is not a new broad failure mode; it is the familiar low-speed / steer-limited weakness

- Decision:
  - Treat `000009cc--94242f81db` as the first completed post-recalibration confirmation route.
  - Do not change friction or delay based on this route.
  - Keep the Santa Fe HEV-only seed alignment as the right baseline correction to test on-device.
  - After the seed-aligned branch is actually deployed, collect another completed route set and compare it directly against `20260410T154901Z`.

## 2026-04-10: Canonical-cache path correction and newest 5-route review

- Trigger:
  - The route-refresh docs were updated to treat `~/.comma/route_sync/downloads/commawifi/data/media/0/realdata/` as the canonical local cache path, even for routes that were historically sourced from other roots.
  - Re-ran the route pull and route selection using that corrected path.

- Refresh / device state:
  - Ran:
    - `python tools/route_sync/refresh_routes.py --host commawifi --max-downloads 240 --newest-first --spread-routes`
  - Report:
    - `~/.comma/route_sync/reports/route_refresh_commawifi_20260410T170238Z.json`
  - Result:
    - remote files visible on-device under canonical `/data/media/0/realdata`: `2`
    - no newly downloaded qlogs from the device on this pass
  - Direct remote inspection confirmed the live `realdata` tree currently only contains:
    - `00000003--7f9abcaaac--10`
    - `00000007--806ca1e3c9--13`
  - Both of those are still active / incomplete route fragments, not a fresh completed route set.

- Corrected local-cache finding:
  - The normalized canonical local cache now contains the latest complete Santa Fe route family under:
    - `~/.comma/route_sync/downloads/commawifi/data/media/0/realdata/000009c8--...`
    - `~/.comma/route_sync/downloads/commawifi/data/media/0/realdata/000009c9--...`
    - `~/.comma/route_sync/downloads/commawifi/data/media/0/realdata/000009ca--...`
    - `~/.comma/route_sync/downloads/commawifi/data/media/0/realdata/000009cb--...`
    - `~/.comma/route_sync/downloads/commawifi/data/media/0/realdata/000009cc--...`
  - This means the earlier “no new routes” conclusion was too narrow: there were no new completed routes currently on the device, but the corrected canonical local cache already had the newest complete route batch available for analysis.

- Newest readable-route review:
  - Ran:
    - `python tools/lateral/analyze_lateral_tuning.py --host commawifi --car-fingerprint HYUNDAI_SANTA_FE_HEV_2022 --max-routes 5`
  - Routes selected:
    - `000009cc--94242f81db`
    - `000009cb--55a0e11719`
    - `000009ca--2b7b178788`
    - `000009c9--9fc1ef49b2`
    - `000009c8--4ee2b2a6f3`
  - Artifact:
    - `~/.comma/lateral_tuning/analysis/commawifi/HYUNDAI_SANTA_FE_HEV_2022/20260410T170339Z/summary.json`
    - `~/.comma/lateral_tuning/analysis/commawifi/HYUNDAI_SANTA_FE_HEV_2022/20260410T170339Z/summary.md`

- Route-slice result:
  - Active samples: `29285`
  - Turning samples: `3148`
  - Strong-turn samples: `1264`
  - All-active:
    - MAE `0.061`
    - median ratio `0.995`
    - saturation `0.25%`
  - Turning:
    - MAE `0.138`
    - RMSE `0.180`
    - median ratio `1.005`
    - under-response `< 0.8` ratio `3.18%`
    - saturation `2.32%`
  - Strong turning:
    - MAE `0.158`
    - RMSE `0.203`
    - median ratio `1.008`
    - under-response `< 0.8` ratio `2.85%`
    - saturation `5.78%`

- Speed-bin picture:
  - `2.5-5 m/s`: MAE `0.175`, median ratio `0.880`, `29` samples
  - `5-10 m/s`: MAE `0.163`, median ratio `0.957`, saturation `0.0%`
  - `10-15 m/s`: MAE `0.140`, median ratio `0.988`, under-response `< 0.8` ratio `5.18%`, saturation `6.10%`
  - `15-25 m/s`: MAE `0.115`, median ratio `1.063`, saturation `0.0%`
  - `25+ m/s`: MAE `0.148`, median ratio `1.040`, `84` samples

- Per-route notes:
  - `000009cc` remains a mild under-response route in repeated familiar windows.
  - `000009cb` shows both under-response and stronger-turn saturation windows.
  - `000009ca` adds several overshoot windows, not only under-response.
  - `000009c9` contributed no turning samples.
  - `000009c8` adds both overshoot and one saturated steer-limited window.

- Live-parameter state on the newest 5-route slice:
  - `liveTorqueParameters.useParams=1.0`
  - `liveTorqueParameters.liveValid=1.0`
  - live `latAccelFactor=2.63000`
  - live `friction=0.10765`
  - live `latAccelOffset=-0.09653`
  - live `steerRatio=15.00000`
  - live `lateralDelay=0.29407`

- Comparison against the 607-route historical baseline:
  - Ran:
    - `python tools/lateral/compare_lateral_runs.py --before ~/.comma/lateral_tuning/analysis/commawifi/HYUNDAI_SANTA_FE_HEV_2022/20260409T163148Z/summary.json --after ~/.comma/lateral_tuning/analysis/commawifi/HYUNDAI_SANTA_FE_HEV_2022/20260410T170339Z/summary.json`
  - Result:
    - the newest 5-route slice is slightly worse than the long-history average on MAE / RMSE
    - turning median ratio is still close to centered at `1.005`
    - the worst recent regression area is `10-15 m/s`, where saturation and under-response are both higher than the long-history baseline
    - `15-25 m/s` still looks broadly good
    - the dominant bad windows are still `steer-limited=True`, often without controller saturation

- Interpretation:
  - Correcting the cache path did not uncover a radically different tuning story.
  - The route batch is still fundamentally “good but not finished”:
    - learned values remain stable around `2.63 / 15.0`
    - tracking is broadly centered overall
    - the remaining weakness is still low / medium speed steer-limited behavior
  - The newest 5-route slice adds some overshoot windows on `000009ca`, so the problem is not purely one-directional under-response, but it is still concentrated in the same low / mid-speed regime.

- Decision:
  - Use `20260410T170339Z` as the current “newest readable slice” reference artifact.
  - Keep the Santa Fe HEV-only seed alignment direction.
  - Do not add a friction or delay change from this route batch.
  - The next real evidence step is still an on-device test after deploying the committed Santa Fe-only seed change, followed by a compare against both `20260409T163148Z` and `20260410T170339Z`.

## 2026-04-10: Route-sync root migration verified, newest 5-route slice unchanged

- Trigger:
  - Route-download documentation changed again, so the route-sync implementation itself was re-checked before continuing the lateral cycle.

- Code-level route-sync truth at this point:
  - `tools/route_sync/common.py` now sets:
    - default route-sync root: `~/.route_sync`
    - default download root: `~/.route_sync`
    - default report dir: `~/.route_sync/reports`
    - default remote roots:
      - `/data/media/0/realdata`
      - `/data/media/0/realdata_HD`
      - `/data/media/0/realdata_konik`
  - local cache is normalized under:
    - `~/.route_sync/data/media/0/realdata/`
  - older `~/.comma/route_sync/...` paths are now legacy layout / compatibility leftovers, not the active default root

- Refresh from the current code path:
  - Ran:
    - `python tools/route_sync/refresh_routes.py --host commawifi --max-downloads 240 --newest-first --spread-routes`
  - Report:
    - `~/.route_sync/reports/route_refresh_commawifi_20260410T175435Z.json`
  - Result:
    - remote files: `4628`
    - new: `0`
    - changed: `0`
    - downloaded: `0`
  - Interpretation:
    - there were no additional route files to pull after the route-sync root migration
    - the current route set in `~/.route_sync` is already up to date

- Newest readable Santa Fe review from `~/.route_sync`:
  - Ran:
    - `python tools/lateral/analyze_lateral_tuning.py --host commawifi --car-fingerprint HYUNDAI_SANTA_FE_HEV_2022 --max-routes 5`
  - Routes selected:
    - `000009cc--94242f81db`
    - `000009cb--55a0e11719`
    - `000009ca--2b7b178788`
    - `000009c9--9fc1ef49b2`
    - `000009c8--4ee2b2a6f3`
  - Artifact:
    - `~/.comma/lateral_tuning/analysis/commawifi/HYUNDAI_SANTA_FE_HEV_2022/20260410T175553Z/summary.json`
    - `~/.comma/lateral_tuning/analysis/commawifi/HYUNDAI_SANTA_FE_HEV_2022/20260410T175553Z/summary.md`

- Result:
  - This rerun is identical to the earlier `20260410T170339Z` 5-route slice.
  - Verified with:
    - `python tools/lateral/compare_lateral_runs.py --before ~/.comma/lateral_tuning/analysis/commawifi/HYUNDAI_SANTA_FE_HEV_2022/20260410T170339Z/summary.json --after ~/.comma/lateral_tuning/analysis/commawifi/HYUNDAI_SANTA_FE_HEV_2022/20260410T175553Z/summary.json`
  - Every tracked metric came back unchanged:
    - all-active, turning, and strong-turn samples all identical
    - same speed-bin metrics
    - same live-vs-static alignment
    - same top issue windows

- Current 5-route slice, reaffirmed:
  - turning MAE `0.138`
  - turning median ratio `1.005`
  - turning under-response `< 0.8` ratio `3.18%`
  - turning saturation `2.32%`
  - strongest remaining weakness is still `10-15 m/s`
  - live values remain stable around:
    - `latAccelFactor=2.63`
    - `steerRatio=15.0`
    - `lateralDelay≈0.294`
    - `liveValid=1.0`

- Interpretation:
  - The route-download process changed, but the tuning conclusion did not.
  - Re-running from the active `~/.route_sync` root confirms the current readable Santa Fe route slice is the same one already reviewed.
  - The remaining diagnosis still holds:
    - static Santa Fe HEV seeds are mis-centered relative to the learned state
    - the main weakness is low / medium speed steer-limited behavior
    - recent `10-15 m/s` behavior is slightly worse than the long-history average

- Decision:
  - Promote `20260410T175553Z` to the current “verified under active route-sync root” reference artifact.
  - Keep the Santa Fe HEV-only seed alignment direction unchanged.
  - Keep friction and delay unchanged.
  - Next real step is still on-device validation after deploying the committed Santa Fe-only seed change.

## 2026-04-11: First true post-deploy route batch verification

- Shared route refresh result:
  - Report:
    - `~/.route_sync/reports/route_refresh_commawifi_20260411T095413Z.json`
  - Result:
    - remote files: `4650`
    - new files: `134`
    - downloaded: `120`
    - skipped due to limit: `14`
  - New routes added into the active `~/.route_sync` cache:
    - `00000019--fd616f4757`
    - `0000001a--e7732b5a28`
    - `0000001b--670b297bdc`
    - `0000001c--600d2d8d67`
    - `0000001d--29a0acc670`

- Route identity check:
  - `0000001a` through `0000001d` all decode as `HYUNDAI_SANTA_FE_HEV_2022`.
  - `00000019` also decodes as the same car when analyzed directly, but it contributes `0` active lateral samples, so it is not useful for requested-vs-actual turning quality.

- On-road static config check from route `carParams`:
  - Read embedded `carParams` directly from the new routes.
  - Confirmed these new routes were driven with the deployed Santa Fe-only static seeds:
    - `steerRatio=15.0`
    - `latAccelFactor=2.63`
    - `friction=0.10384`
    - `latAccelOffset=0.0`
  - This is the first real route batch that validates the deployed static seed change on-road, not just the local repo config.

- New-route tuning review:
  - Ran:
    - `python tools/lateral/analyze_lateral_tuning.py --host commawifi --route 0000001a--e7732b5a28 --route 0000001b--670b297bdc --route 0000001c--600d2d8d67 --route 0000001d--29a0acc670`
  - Artifact:
    - `~/.comma/lateral_tuning/analysis/commawifi/HYUNDAI_SANTA_FE_HEV_2022/20260411T101228Z/summary.json`
    - `~/.comma/lateral_tuning/analysis/commawifi/HYUNDAI_SANTA_FE_HEV_2022/20260411T101228Z/summary.md`

- Batch result:
  - Active samples: `25738`
  - Turning samples: `1752`
  - Strong-turn samples: `743`
  - All-active:
    - MAE `0.070`
    - median ratio `0.929`
    - saturation `0.0%`
  - Turning:
    - MAE `0.199`
    - RMSE `0.247`
    - median ratio `0.928`
    - under-response `< 0.8` ratio `10.33%`
    - saturation `0.06%`
  - Strong turning:
    - MAE `0.218`
    - RMSE `0.265`
    - median ratio `0.938`
    - under-response `< 0.8` ratio `6.59%`
    - saturation `0.13%`

- Speed-bin picture:
  - `2.5-5 m/s`: MAE `0.196`, median ratio `0.908`, under-response `< 0.8` ratio `19.33%`
  - `5-10 m/s`: MAE `0.221`, median ratio `0.908`, under-response `< 0.8` ratio `15.41%`
  - `10-15 m/s`: MAE `0.261`, median ratio `0.876`, under-response `< 0.8` ratio `14.74%`
  - `15-25 m/s`: MAE `0.142`, median ratio `0.953`
  - `25+ m/s`: MAE `0.162`, median ratio `1.061`
  - Interpretation:
    - this batch is materially worse than the prior verified slice
    - `10-15 m/s` is now clearly the weakest band
    - the regression is dominated by under-response, not saturation

- Per-route notes:
  - `0000001a` contributed active samples but no turning samples.
  - `0000001b`: turning MAE `0.203`, median ratio `0.978`
  - `0000001c`: turning MAE `0.251`, median ratio `0.896`
  - `0000001d`: turning MAE `0.168`, median ratio `0.918`

- Live-parameter state on the new batch:
  - `liveTorqueParameters.useParams=1.0`
  - `liveTorqueParameters.liveValid=1.0`
  - live `latAccelFactor=3.236`
  - live `friction=0.0846`
  - live `latAccelOffset=-0.107`
  - live `steerRatio=15.219`
  - live `lateralDelay=0.376`

- Comparison against the last verified slice:
  - Ran:
    - `python tools/lateral/compare_lateral_runs.py --before ~/.comma/lateral_tuning/analysis/commawifi/HYUNDAI_SANTA_FE_HEV_2022/20260410T175553Z/summary.json --after ~/.comma/lateral_tuning/analysis/commawifi/HYUNDAI_SANTA_FE_HEV_2022/20260411T101228Z/summary.json`
  - Result:
    - turning MAE worsened from `0.138 -> 0.199`
    - turning median ratio worsened from `1.005 -> 0.928`
    - turning under-response `< 0.8` worsened from `3.18% -> 10.33%`
    - strong-turn MAE worsened from `0.158 -> 0.218`
    - saturation decreased, so this is not a simple “more clipping” story
    - steer-limited ratio also decreased, so the route batch is worse even without more time spent at the steering limit

- Comparison against the 607-route historical baseline:
  - Ran:
    - `python tools/lateral/compare_lateral_runs.py --before ~/.comma/lateral_tuning/analysis/commawifi/HYUNDAI_SANTA_FE_HEV_2022/20260409T163148Z/summary.json --after ~/.comma/lateral_tuning/analysis/commawifi/HYUNDAI_SANTA_FE_HEV_2022/20260411T101228Z/summary.json`
  - Result:
    - this post-deploy batch is materially worse than the long-history baseline across every meaningful turning metric
    - the worst degradation is again in `10-15 m/s`

- Interpretation:
  - This is the first real on-road validation of the deployed `latAccelFactor=2.63` / `steerRatio=15.0` setup.
  - It did not validate the full seed change.
  - The new routes are under-responding substantially more than the prior route slice and more than the long-history baseline.
  - `steerRatio=15.0` is still reasonably close to the new live median (`15.219`), so that part does not look obviously wrong.
  - `latAccelFactor=2.63` now looks too low for this current learned regime, because the live learner moved upward to about `3.24` and the controller is under-responding at the same time.
  - Live friction also moved lower than the deployed static friction, but this batch does not justify changing friction before dealing with the more obvious `latAccelFactor` miss.
  - The jump in live delay to about `0.376` is notable, but the route windows still point more strongly at torque-map / response mismatch than at a pure delay-only problem.

- Decision:
  - Treat the deployed `latAccelFactor=2.63` seed as not validated.
  - Keep `steerRatio=15.0` tentatively acceptable.
  - Do not make a friction or delay change first.
  - The next tuning candidate should be a higher Santa Fe HEV `latAccelFactor`, likely toward the new live regime rather than the previous `2.63` target.

## 2026-04-11: Follow-up Santa Fe HEV torque seed correction

- Trigger:
  - The first true post-deploy validation batch (`20260411T101228Z`) showed that `latAccelFactor=2.63` under-responds materially on-road while live learning climbed to about `3.24`.

- Repo change applied:
  - `opendbc_repo/opendbc/car/torque_data/params.toml`
    - `HYUNDAI_SANTA_FE_HEV_2022` `LAT_ACCEL_FACTOR`:
      - `2.63 -> 3.24`
  - Left unchanged:
    - `steerRatio=15.0`
    - friction `0.10384`

- Rationale:
  - The new routes were driven with the deployed `2.63 / 15.0` seeds and still under-responded badly.
  - `steerRatio=15.0` remained close to the new learned state (`~15.219`).
  - The larger miss is torque response:
    - deployed static `latAccelFactor=2.63`
    - new live median `latAccelFactor≈3.236`
  - So the next clean iteration is to re-center only `latAccelFactor` toward the observed live regime instead of changing multiple levers at once.

- Expected effect:
  - Reduce the new batch’s broad under-response, especially in `10-15 m/s`.
  - Preserve the improved steer-ratio seed.
  - Avoid mixing in a friction or delay change before the torque-factor mismatch is resolved.

## 2026-04-18: Reverted-model verification and lower torque-factor follow-up

- Trigger:
  - The user reverted to the non-ping-pong model, then reported some remaining highway wobble and bookmarked a live-route event for inspection.

- Newest completed-route verification after the model revert:
  - Newest Santa Fe HEV routes by local mtime:
    - `00000068--8f1c5a1f59`
    - `00000069--a6ea22da9d`
  - Ran:
    - `python tools/lateral/analyze_lateral_tuning.py --host comma --route 00000068--8f1c5a1f59 --route 00000069--a6ea22da9d`
  - Artifact:
    - `~/.comma/lateral_tuning/analysis/comma/HYUNDAI_SANTA_FE_HEV_2022/20260418T071423Z/summary.json`
  - Result:
    - all active: `9473` samples, MAE `0.085`, median ratio `0.985`, saturation `0.6%`
    - turning: `1341` samples, MAE `0.191`, median ratio `0.981`, under-response `< 0.8` ratio `8.58%`, saturation `4.47%`
    - strong turning: `656` samples, MAE `0.197`, median ratio `0.983`, under-response `< 0.8` ratio `8.84%`, saturation `9.15%`
  - Interpretation:
    - the completed routes no longer show the prior high-speed ping-pong signature
    - the remaining weakness is still low / mid-speed turning, not a broad straight-line highway instability
    - live learning on this reverted-model pair settled at:
      - `latAccelFactor=2.755`
      - `steerRatio=14.870`
      - `lateralDelay=0.357`

- Bookmarked highway-wobble review on the active route:
  - Active route on device:
    - `00000083--9eb611b486`
  - Bookmark markers found:
    - segment `11`, `bookmarkButton` at `668.868 s`
    - segment `11`, `userBookmark` at `668.873 s`
  - Pulled the active segment qlogs directly from `comma` because `commawifi` was timing out.
  - Route-level artifact:
    - `~/.comma/lateral_tuning/analysis/comma/HYUNDAI_SANTA_FE_HEV_2022/20260418T072417Z/summary.json`
  - Bookmarked-window inspection around `±20 s`:
    - speed `137-139 kph`
    - desired lateral accel sign flips: `41`
    - actual lateral accel sign flips: `40`
    - desired vs actual correlation: `0.718`
    - desired vs actual correlation with small lag shift: `0.837`
    - `steer_limited_ratio=0.856`
    - `saturation_ratio=0.0`
  - Worst bookmarked-window mismatch examples:
    - around `+1.13 s`, desired `-0.625`, actual `0.276`, error `0.901`
    - around `+0.02 s`, desired `0.402`, actual `-0.228`, error `-0.630`
    - around `+0.72 s`, desired `0.301`, actual `-0.306`, error `-0.607`
  - Important context:
    - lane-line confidence briefly collapsed during the worst spike:
      - around `+1.13 s`, lane probs were about `0.295 / 0.173`
    - at other bad samples lane confidence remained high, so the full wobble was not one simple lane-loss frame
    - live learning on this bookmarked route was:
      - `latAccelFactor=2.719`
      - `steerRatio=15.347`
      - `lateralDelay=0.372`

- Interpretation:
  - The bookmarked highway wobble was mixed, not purely one-sided:
    - the request itself was unstable and the car mostly followed it with lag
    - the current Santa Fe tune still looks too eager and likely amplifies the wobble once a bad request appears
  - The current static `latAccelFactor=3.24` is now materially above the live-learned range seen on both the reverted-model completed routes and the bookmarked highway route.
  - The evidence on `2026-04-18` is now:
    - `2.63` was too low
    - `3.24` is too high for the current reverted-model regime
    - the next Santa Fe-only static torque candidate should sit near the observed `2.72-2.76` live band

- Repo change applied:
  - `opendbc_repo/opendbc/car/torque_data/params.toml`
    - `HYUNDAI_SANTA_FE_HEV_2022` `LAT_ACCEL_FACTOR`:
      - `3.24 -> 2.75`
  - Left unchanged:
    - `steerRatio=15.0`
    - friction `0.10384`

- Rationale for `2.75`:
  - Reverted-model completed-route live median: `2.755`
  - Bookmarked highway-route live median: `2.719`
  - `2.75` is a clean midpoint that re-centers the static seed near the current learned regime without dropping all the way back to the failed `2.63` setting.

- Next expectation:
  - reduce high-speed over-eagerness relative to the `3.24` seed
  - keep the reverted model’s straight-line stability
  - re-check whether the low / mid-speed under-response returns too strongly after this downward step
