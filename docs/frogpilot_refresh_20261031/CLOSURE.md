# Dependency closure of the port-now set (phase 6)

Source: `cleanup/fp1031-subtract` `c022a9890c` (OURS, the final state after
the phase 5 removals). Target: `437be91e24` (NEW). Old base: `d2db3fad4b`
(OLD). Measured 2026-11-01 from Git objects with three scripts kept in
`~/.route_sync/work/fp1031/port-scratch/tools/`:

- `closure.py`: for every changed `.py` file at OURS, each `openpilot.*`,
  `opendbc.*` and `cereal` import whose module or name is absent at NEW.
- `params_cmp.py`: the `common/params_keys.h` rows of OURS against NEW (key,
  flags, type, default, stock value, tuning level), with the files at OURS
  that name each differing key.
- `capnp_removed.py`: schema fields at OURS that NEW removed, renamed or
  renumbered.

Outputs: `closure_imports.txt`, `params_cmp.txt`, `capnp_removed.txt` in the
same folder. A port batch brings the files it lists below together with the
cross-group items in its row. "Carry whole" means our OURS file replaces the
NEW file; everything else is our hunks reapplied on the NEW file.

## Findings that change the batch plan

1. **The Hyundai controller imports a stopping module.**
   `opendbc/car/hyundai/carcontroller.py:12` imports
   `openpilot.selfdrive.controls.lib.stopping_flags` (`STOPREQ_SETTLED_SET`).
   `stopping_flags.py` has no imports. Batch 4 brings `stopping_flags.py`
   with the Hyundai rows; batch 5 then owns its later edits.
2. **The longitudinal core imports force coast.** `longcontrol.py:57`,
   `longitudinal_planner.py:33` and `frogpilot_acceleration.py:9` import
   `frogpilot/controls/lib/force_coast.py` (F01, 45 lines; it imports
   `stopping_flags`). Batch 5 brings `force_coast.py`; batch 6 keeps only the
   F01 hunks in the FrogPilot acceleration and CEM files.
3. **The planner and longcontrol are shared by S, F01, F02, F04 and F05.**
   The feature commits `b91b539d41`, `be2678f678`, `ca4ab1a1dc` and
   `cb6472b9fd` all edit `longcontrol.py`, `longitudinal_planner.py` and
   `long_mpc.py`; `b91b539d41` and `72f95df2a9` (F03) both edit `radard.py`.
   These four files port once, in batch 5, in their OURS state. Batch 6 does not touch them again except where an interface to a
   NEW FrogPilot module changes.
4. **`frogpilot_traffic.py` is gone at NEW and imported by `frogpilot_planner.py:20`.**
   It is carried whole in batch 6 (F02). It imports `get_max_accel`
   (planner), `LEAD_DANGER_FACTOR`, `desired_follow_distance` (MPC),
   `LEAD_STOP_DISTANCE_TARGET` (`stop_target_helpers.py`, S) and
   `MAX_T_FOLLOW` (present at NEW). It must follow batch 5.
5. **Corpus rlogs do not decode under the NEW FrogPilot schema.** NEW
   renumbered `FrogPilotCarState` (for example `forceCoast` @9 to @12, and
   NEW @12 is OLD `sportGear`) and `FrogPilotPlan` (`increasedStoppedDistance`
   @12 to @14; `frogpilotToggles` moved to `FrogPilotProcessState`), and gave
   `ThePondRequest` the struct id of OLD `FrogPilotCarControl`
   (`0x81c2f05a394cf4af`, Event @107). A candidate checkout reading our
   2026 corpus with its own `custom.capnp` gets wrong values without an
   error. The sim reads `frogpilotCarState.forceCoast/trafficModeEnabled` and
   `frogpilotPlan.increasedStoppedDistance/frogpilotToggles/tFollow/...`
   (`tools/stopping/sim/harness.py:118-121`, `f1.py:120,223-229`,
   `radar_replay.py:398-418`, `tools/stopping/review/plant_data.py:149-164`).
   Work item for batch 6, before the first verdict: the sim decodes corpus
   logs with the schema that recorded them (OURS `cereal/*.capnp`), and the
   candidate code reads only the values the sim passes it. `log.capnp` and
   `car.capnp` keep OURS ordinals (batch 1), so only the FrogPilot structs
   are affected.
6. **NEW has no memory Params.** `Params(memory=True)` and `/dev/shm/params`
   are gone (`common/params.cc`, `params_pyx.pyx`). OURS uses them in eleven
   places, all in OLD FrogPilot files that NEW rewrote, plus our CEM
   (`params_memory.get/put("CEStatus")`). The carried CEM (batch 6) must move
   `CEStatus` to NEW's `frogpilotPlan.ceStatus` path or to a NEW-supported
   store; the registry row stays (batch 1) until that adapter exists.
7. **`Params::clearAll` deletes every stored key that the registry lacks**
   (`437be91e24:common/params.cc:213-235`, run at manager start for four
   flags). The batch 1 registry therefore carries every key our ported code
   reads and every PERSISTENT row OURS added, so no stored value of ours is
   lost. OLD keys that NEW removed and our ported code does not read follow
   NEW (deleted at first boot; listed in the batch 1 report).
8. **NEW reads the model identity outside modeld** (found in batch 3).
   Batch 3 pins `toggles.model = "tsfdo"` and `toggles.model_path =
   STOCK_MODEL_PATH` in `frogpilot_variables.py`. NEW
   `long_mpc.py:316` gates model lead trajectories on
   `predicts_lead_trajectories(model_path)`, which reads
   `supercombo_metadata.pkl` or `driving_*_metadata.pkl` in that folder.
   Our build writes the metadata only inside `driving_tinygrad.pkl`, so the
   check returns False for tsfdo, although its `lead` output is 144 values
   (a trajectory model). OURS uses model lead trajectories under
   HumanFollowing or Traffic Mode with no capability check, and the device
   has `HumanFollowing=1`. Batch 5 must keep OURS' condition or give the
   check tsfdo's metadata; otherwise following changes on day one.
   NEW `longitudinal_planner.py:107` (`is_pre_tomb_raider_model`) and
   `controlsd.py:72` (`get_model_lat_smooth_seconds`, 0.1 s for an id
   outside the catalogue, equal to our `LAT_SMOOTH_SECONDS`) read the id too.

## Per group

Abbreviations: `ctl/` = `selfdrive/controls/`, `lib/` = `selfdrive/controls/lib/`,
`fp/` = `frogpilot/`, `hy/` = `opendbc_repo/opendbc/car/hyundai/`.

| Group (rows) | Batch | Files (OURS state) | Needs from other groups | Needs from OLD, absent or changed at NEW | Params keys it reads (registry, batch 1) | capnp fields |
|---|---|---|---|---|---|---|
| Base (B01, B03) | 1 | `cereal/{log,custom}.capnp`, `cereal/messaging/__init__.py`, `cereal/services.py`, `opendbc/car/car.capnp`, `common/params_keys.h`, `common/SConscript`, `conftest.py`, `panda/SConscript`, `panda/board/{health,comms_definitions}.h`, `system/manager/build.py`, `system/athena/registration.py` and its test | none | NEW `register()` early return without keys; NEW removed the unregistered alert | provides all rows below | provides all fields below |
| Model (M01..M03) | 3 | `tinygrad_repo/` (OURS whole), `selfdrive/modeld/{modeld,compile_modeld,compile_dm_warp,helpers,constants,dmonitoringmodeld,fill_model_msg,get_model_metadata,parse_model_outputs}.py`, `SConscript`, `models/driving_supercombo.onnx`, `common/file_chunker.py`, `system/camerad/cameras/nv12_info.py`, `selfdrive/test/process_replay/{model_replay,process_replay}.py` | `cereal/services.py` 17 Hz DM rates (B01) | NEW Model Manager per-model folders, `frogpilot_model_utilities`, default model "Dark Souls" (decided in batch 3) | `UsbGpuPresent`, `UsbGpuCompiled` only if M04 code stays (M04 is not ported) | none new |
| Hyundai (H01..H05, H07) | 4 | `hy/{carcontroller,carstate,hyundaican,hyundaicanfd,interface,radar_interface,values,enable_radar_tracks}.py`, `hy/tests/{conftest,test_can_bounds_fork}.py`, `opendbc/car/{interfaces,car_helpers}.py`, `opendbc/safety/{declarations,longitudinal}.h`, `safety/modes/{hyundai,hyundai_common}.h`, safety tests, `selfdrive/car/card.py` (H04 hunks), `ctl/controlsd.py` and `lib/drive_helpers.py` (H03 hunks), `selfdrive/selfdrived/helpers.py` | `lib/stopping_flags.py` (S, finding 1); `selfdrive/car/live_update_handoff.py` (U02, imported by `card.py:25`; port the module in batch 4 or keep the card hunk for batch 7) | NEW `CarState(CP)` and `fp_ret`, no toggles argument to `CarController.update`, `CP.safetyConfigs` without the FPCP merge, `frogpilot_variables.update_frogpilot_toggles` (absent at NEW, used by `card.py:33`) | `Hyundai-RadarTracks`, `LongitudinalActiveWithGas` | `FrogPilotCarState.alwaysOnLateralAllowed` (NEW has it at @1) |
| Stopping and longitudinal (S00..S34) | 5 | all `lib/stop*.py`, `lib/lead_provenance.py`, `lib/longitudinal_mpc_lib/stop_target_helpers.py`, `lib/{longcontrol,longitudinal_planner,drive_helpers}.py`, `long_mpc.py`, `ctl/{radard,plannerd,controlsd}.py`, `selfdrive/car/cruise.py` (S hunk), their tests | `fp/controls/lib/force_coast.py` (F01, finding 2); `has_adjacent_lane`, `calculate_lane_width` (OURS `fp/common/frogpilot_utilities.py`, F02); `LEFTMOST_HIGHWAY_LEAD_EASING_SCALE` (F02 in `long_mpc.py`) | NEW planner bounds (`ACCEL_MIN` solver floor, cruise max 1.6) and `LEAD_DANGER_FACTOR`; OURS still passes `dangerFactor` (`longitudinal_planner.py`); `frogpilotPlan.dangerFactor` removed at NEW | none of its own (flags are constants) | `LongitudinalPlan.{distanceToStopTarget,aTargetTrajectory,aTargetTrajectoryValid,distanceToStopTargetModel,wholeApproach*}`, `ControlsState.stoppingControlActive`, `CarControl.driverMonitoringEscalation` |
| Force coast (F01, F06) | 5 (`force_coast.py`) and 6 | `fp/controls/lib/force_coast.py`, F01 hunks in `fp/controls/lib/frogpilot_acceleration.py`, CEM and card | `stopping_flags` (S) | NEW masks throttle instead of `forceDecel` | `CEForceCoastStrength` | `FrogPilotCarState.forceCoast` (NEW @12) |
| Following (F02, F04) | 6 | `fp/controls/lib/frogpilot_traffic.py` (carry whole), F02/F04 hunks in `fp/controls/{frogpilot_planner,frogpilot_card}.py`, `fp/controls/lib/{frogpilot_following,frogpilot_acceleration}.py`, `fp/common/{frogpilot_variables,frogpilot_utilities,frogpilot_functions}.py`, `fp/frogpilot_process.py` | `stop_target_helpers.py`, `get_max_accel` (S, batch 5) | `frogpilot_traffic.py` (deleted at NEW); OLD names `FROGPILOT_API`, `FROGS_GO_MOO_PATH`, `KONIK_PATH`, `ERROR_LOGS_PATH`, `HD_LOGS_PATH`, `KONIK_LOGS_PATH`, `MAPS_PATH`, `THEME_SAVE_PATH` used by OURS `frogpilot_functions.py`/`frogpilot_utilities.py` (NEW moved paths to `frogpilot_paths.py`) | `ShortDistanceFactor`, `LongDistanceFactor`, `CEExperimentalBoostGain`, `CEExperimentalNoLeadBoostGain`, `CEExperimentalBoostBrakeCutoff`, `HumanFollowing` (same at NEW); Traffic v2 rows kept stored (no reader) | `FrogPilotPlan.trackingLead`, `dangerFactor` (removed at NEW, OURS planner writes them) |
| Lane-change lead (F03) | 5 (with `radard.py`, finding 3) | F03 hunks in `ctl/radard.py`, `ctl/tests/test_radard_lane_change_surrogate.py` | `stop_target_helpers.py` (S) | NEW far/radar-only lead and adjacent `leadTwo` (left out on day one) | none | `FrogPilotRadarState.{leadOneSurrogate,leadTwoSurrogate,leadOneRawDRel,leadOneRawVLead}` |
| CEM (F05, F06) | 6 | `fp/controls/lib/conditional_experimental_mode.py` (carry whole), CEM hunks in `fp/controls/frogpilot_card.py`, `fp/common/frogpilot_variables.py` | `COMFORT_BRAKE`, `STOP_DISTANCE`, `get_safe_obstacle_distance` (`long_mpc.py`, batch 5) | `SLOWDOWN_PERCENTAGE`, `CONDITIONAL_EXPERIMENTAL_OVERRIDE_PATH` (absent at NEW `frogpilot_variables.py`); memory Params (finding 6) | `CECscCurves`, `CEModelStopTime` (OLD, removed at NEW), `CEStatus` (OLD, memory store) | NEW `frogpilotPlan.ceStatus` path |
| Cruise and speed limits (C01..C06) | 6 | `selfdrive/car/cruise.py`, test, `fp/controls/lib/curve_speed_controller.py` (carry whole), SLC hunks, `fp/controls/lib/frogpilot_vcruise.py` hunks, `fp/navigation/mapd` (OURS binary) | none outside batch 6 | NEW SLC methods `update_limits`, `update_map_speed_limit`, `update_override`; NEW CSC learners replace OURS training | `InitialSetSpeed`, `CSCSpeedFactor`, `CSCBrakingForce`, `CSCBrakingForceHighSpeedReduction`, `CalibratedLateralAccelerationOverride`; `CalibratedLateralAcceleration`, `CalibrationProgress`, `CurvatureData` (same at NEW) | `FrogPilotCarState.accelPressed/decelPressed` (NEW counters) |
| Lateral (L02, L03, L05..L08) | 6 | `lib/{lane_centering,latcontrol_torque,desire_helper}.py`, `selfdrive/locationd/{lagd,torqued,calibrationd,locationd}.py`, `opendbc/car/torque_data/params.toml`, lateral controlsd hunks | none | L01 not ported: drop `controlsd.py:315` (`curvatureState`) and the `latcontrol_curvature` import; L04 not ported | `LaneCentering`, `LaneCenteringE2EAuthority`, `LaneCenterOffset` (stored; L04 reader optional) | `LiveDelayData.version` |
| System (Y01..Y09) | 7 | `system/manager/{manager,process,process_config,helpers}.py`, `common/watchdog.*`, `system/hardware/*`, `system/loggerd/*`, `system/timed.py`, `selfdrive/selfdrived/{selfdrived,events,state,cruise_helpers}.py` | `ensure_media_storage` (Y08 `system/manager/helpers.py`) for `manager.py` | NEW Soft Reboot path, Pond, telemetry processes | `Offroad_StorageMissing` (same at NEW) | none |
| Update path (U01..U04) | 7 | `fullupdate.sh`, `update.sh`, `restart.sh`, `launch_chffrplus.sh`, `selfdrive/car/live_update_handoff.py`, handoff hunks in `card.py`, `controlsd.py`, `selfdrived.py`, `selfdrive/pandad/*`, `system/tailscale/*` | card/controlsd from batches 4 and 5 | NEW `DoSoftReboot`, Pond reboot endpoints (new interception work) | `LiveUpdateHandoffState`, `LiveUpdateHandoffBlockers`, `BuildMetadata`, `TailscaledPid` | none |
| UI (V01..V03) | 8 | Qt and raylib UI files named in `bb8bb838de`, `0df7c40a56` | stopping telemetry owner state (S) | NEW UI events replace transient Params | menu reads of the keys above | none |
| Tools (T01, T02) | 2 (sim, longitudinal), 9 (rest) | `tools/stopping/sim/`, `tools/longitudinal/`, their tool-side imports (below) | runtime modules of batches 4..6 at run time | none at import time | none | finding 5 |

## Batch 2 tool-side closure

`tools/stopping/sim/*.py` and `tools/longitudinal/*.py` import these
tool modules, which therefore come in batch 2:

- `tools/stopping/review/{plant_data,kcs1_reps,kcs_plant,terminal_smoothness}.py`
  (`harness.py:97,145,324,437`, `rharness.py:49,138`), and the package
  `tools/stopping/review/__init__.py`.
- `tools/stopping/log_schema_helpers.py` (`tools/longitudinal/analyze_*`).
- `tools/stopping/analyze_stopping_behavior.py`
  (`analyze_green_light_launch.py:18`).
- `tools/route_sync/common.py` (`analyze_longitudinal_tracking.py:25`,
  `analyze_braking_focus.py:21`).

Runtime imports that resolve only after later batches (deferred, imported
inside functions, so module import succeeds): `stopping_flags`,
`stopping_service`, `stopping_telemetry`, `longcontrol.LongControl` with the
stopping service (batch 5); `opendbc.car.hyundai.tests.test_can_bounds_fork`
(batch 4); `frogpilot.controls.lib.frogpilot_traffic.FrogPilotTraffic`
(`radar_replay.py:454`, batch 6); `stop_target_helpers` (batch 5).

## Hand-offs from batch 4

- Batch 5, `controlsd.py`: the H03 call sites (`longitudinal_control_active`,
  `longitudinal_accel_with_gas`, `longitudinal_control_override`, and
  `freeze_integrator=gas_override` into our `LongControl.update`) port with
  controlsd. Read the bit from `CP.alternativeExperience`; NEW moved it off
  FPCP, and batch 4's card sets it there. Until then the panda allows
  acceleration with gas but the host never commands it.
- Batch 6, `frogpilot_card.py`: NEW tracks AOL-allowed itself from sampled
  mainCruise/LKAS button events and has no RES/SET arming. OURS used
  `carState.cruiseState.available` (carstate's inferred main, armed by RES/SET)
  for the AOL main mode. Keep ours, or the host stays stricter than the panda
  after a RES/SET on a stale main.
- Batch 7, `card.py`: only the H03/H04 hunks are in; the live-update handoff
  supervision and `controls_quiesced` port with U02. The Hyundai side
  (`CarInterface.init` success/retry, `deinit`) is in.
- Batch 8: the `LongitudinalActiveWithGas` toggle UI.

## Hand-offs from batch 5

Until batch 6, NEW F modules feed the ported core. The equivalence replay
(route 00002310, logged inputs) shows the core itself equal to the subtract tip.
It does not cover these producers:

- Batch 6, `frogpilot_planner.py` and `frogpilot_following.py`: NEW publishes
  `frogpilotPlan` (`tFollow`, min/max acceleration, jerks,
  `increasedStoppedDistance`). `dangerFactor` is `LEAD_DANGER_FACTOR` for now;
  OURS computes it in `frogpilot_following.py`. NEW `frogpilot_following.py`
  calls OURS `desired_follow_distance` with another signature (the UI value
  only). Port OURS `frogpilot_following.py`;
  `test_frogpilot_stop_distance_targets.py` fails until then.
- Batch 6, `lane_centering.py` (L): `controlsd` has no lane-centering call yet;
  `test_pending_brake_publication.py` fails at import until then.
- Batch 6, `frogpilot_card.py`: the force-coast trigger. Both trees map
  `DistanceButtonControl` 2 to `FORCE_COAST`; NEW toggles
  `frogpilotCarState.forceCoast` in `handle_button_event` while `longActive`.
  Compare it with OURS before the verdict.
- Batch 6, lateral (L02): NEW `latcontrol_torque.KP` is 0.8, OURS 1.0. With
  `AdvancedLateralTune` 0, the `steerKp` toggle is that `KP`, so the stored
  `SteerKP` 1.0 does not apply. Lateral Kp stays NEW's until L02 ports.
- Batch 6, any F-module reader of `max_desired_acceleration`: NEW `get_value`
  returns the stock value (2.0) when its condition is false; OURS returned the
  default (4.0). `controlsd` caps the LongControl output with it. The Hyundai
  PID limit is 2.0 and the replay peaks at 1.97 m/s², so the cap does not bind
  there.

## Hand-offs from batch 6

First full sim verdict (2026-11-02): run `20261102-0055_fp1031_port_fc8b326158`
against the subtract tip re-run `20261102-0004_fp1031_tip_c022a9890c`
(itself identical to `20261031-2350_fp1031_candidate_72ebdab488`). Both
INCOMPLETE (470 jobs without rows: rlogs pruned 2026-10-12). Rows identical
on every job with a row (closed 848 per arm, F1 100, judges 15, replay 507
and their plans); the same 205 failed jobs, each `FileNotFoundError` on a
pruned rlog. The tracebacks differ only by the sim's decode path
(`recorded.read_rlog`, finding 5), and the port arm lists two of those jobs
once more (its `--quick` pre-run).

Batch 6 keeps every driving decision of today on the new base. The
equivalence replay of route 00002310 (`tools/stopping/port_check`: radard,
planner, LongControl, FrogPilot planner, controlsd) shows no difference
from the subtract tip. These new-base behaviours stay off on day one; each
is a later trial:

- Lateral: NEW `latcontrol_torque` (KP 0.8, KI 0.15, jerk lookahead) and NEW
  NNFF (batched evaluation, friction override, NNFF-lite friction, a local
  friction factor); the safety-limit freeze while always-on lateral is
  engaged; turn desires gated off during a lane change.
- Longitudinal FrogPilot modules: NEW `frogpilot_acceleration` (ECO table
  `[1.4, 1.3, 1.0, 0.7, 0.6, 0.5, 0.4]`, ECO_PLUS, `CRUISE_MIN_ACCEL`
  floor), NEW following (fixed Traffic Mode), NEW CEM, CSC and SLC, NEW lane
  width filtering and road curvature with roll.
- Cruise: the speed-limit confirmation that suppresses a cruise step.
- Upstream still runs NEW's torque re-application when
  `liveTorqueParameters.useParams` drops; it does not fire with today's
  settings (no custom friction or lat-accel factor).

Later batches:

- Batch 7 (U02): `test_pending_brake_publication.py` handoff cases (4) fail
  until the live-update handoff ports into `controlsd`. Our
  `frogpilot_functions.py`, `frogpilot_process.py` (update cadence,
  telemetry sweep, realtime cores) and `selfdrived.py` (stale deviceState,
  ignore lists, mock-car cleanup guard, personality writes, handoff states)
  hunks port with Y and U.
- Batch 8 (UI): NEW UI labels `ceStatus` 0..8; our CEM also publishes 9
  (FORCE_COAST). Our lateral and longitudinal menu hunks and `torque_bar.py`.
- `max_desired_acceleration`: no F or C module reads it; only `controlsd`
  caps the LongControl output with it (decision 1, stock 2.0).

## Hand-offs from batch 7

The manager now routes `DoReboot` and `DoSoftReboot` to
`fullupdate.sh __safe_reboot` while the ignition is on or the car is started.
This covers the settings buttons, Soft Reboot and the Pond endpoints. If the
script does not start, the request is dropped; there is no raw fallback.

- Batch 8 (UI):
  - The raw `HARDWARE.reboot()` buttons
    (`selfdrive/ui/mici/layouts/offroad_alerts.py:224`,
    `selfdrive/ui/widgets/offroad_alerts.py:113`, Qt `Hardware::reboot`
    callers) set `DoReboot` instead.
  - The NEW settings Reboot and Soft Reboot buttons can stay as they are.
    Our Qt `safeReboot()` (`util.cc`) is not needed: it fell back to a raw
    `DoReboot` when the launch failed.
  - Port our Full Update button and the staged-update compact badge
    (`alerts.cc` `isStagedUpdateAlert`). The badge keys on the supervisor's
    `alertDebug` text1 values: Update Running, Update Ready, Reboot Ready,
    Preparing Restart, Cruise Still On, Restart Paused, Update Staged,
    Reboot Pending.
  - selfdrived's side is in: `alertDebug` shows as `longitudinalManeuver`
    while it is updated and not empty.
  - The selfdrived personality-write hunks (`PERSONALITY_PARAM_WRITE_TIMEOUT`)
    come from `bb8bb838de` (UI), not from Y or U.
- Batch 9: the `AGENTS.md` device-update hunks of `0df7c40a56` go with the
  agent guides. Its line "Nothing deploys a push by itself" is wrong while
  `AutomaticUpdates` is on (the device store has it on). `frogpilot_process`
  runs our `update_openpilot` every 5 minutes. On a remote change, it starts
  `fullupdate.sh`.
- Existing behaviour, not changed by the port (the script is byte-identical):
  - `stop_staging_banner` kills only the subshell, and the banner Python
    keeps running. On the device it exits when the supervisor's publisher
    takes over `alertDebug`: msgq fails the old publisher's next send.
    ZMQ (macOS) refuses the second publisher instead; the harness has a
    shim for that.
  - First deploy only: the new supervisor reads the old card's
    `frogpilotCarState` with the new schema, and `alwaysOnLateralEnabled`
    (new `@2`, bit 65 beyond the old one-word struct) reads False. Its
    always-on-lateral check is blind for that deploy. The old card's own
    READY gate (always-on-lateral suppression, disengagement reasons) still
    applies, and phase 8 plans the first transition off-road.
  - NEW has its own `tailscaled` (`/data/tailscale`, started only after a
    Pond install). Ours is the rescue service (`system/tailscale`,
    `/data/media/0/tailscale`). The Pond is off on day one, so that install
    route is unreachable; no code change (host, 2026-11-02). Later trial: if
    The Pond comes back, decide which Tailscale node stays.
  - NEW `hardwared` also sets `Offroad_StorageMissing` from
    `os.path.ismount("/data/media")` on every loop. This agrees with the boot
    guard.

## Hand-offs from batches 8 and 9

- Two log schemas after the device move. The sim (`recorded.py`) decodes
  FrogPilot events with the old recording schema; `drive_report.py`,
  `analyze_stopping_behavior.py` and the review tools decode with the tree's
  cereal. On this branch the tools read new drives correctly and the old
  corpus wrongly; the sim does the opposite. Before a post-move drive enters
  the sim corpus, or a tool reads both, choose the schema per log (a marker in
  `initData`, or the writer's struct sizes).
- Settings with no effect on day one, as upstream: the Model panel's
  selection, downloads and randomizer (the runtime is pinned to tsfdo); the
  Pond's LAN address in Navigation (The Pond does not start). The stopping
  knobs StartAccel, VEgoStarting, StopAccel, StoppingDecelRate and
  VEgoStopping are off the panel, as on our tip (our toggles take the
  CarParams values).
- The HUD's CarParams stopping arrays have no writer in either tree; their
  N/A lines are left out.
- `test_following_distance.py`: two value tests fail on the port and on our
  tip (1.45 vs 1.25, 0.0 vs 16.0). They predate the port.

## Not ported (no closure edge into port-now code)

L01 `latcontrol_curvature.py` and its `curvatureState`/`SteerControlType.curvature`
renames, L04 offset (key kept stored), M04 USB GPU build path, V04 and V05
menus (their keys follow NEW or stay stored, see batch 1), the phase 5 drop
rows. `CarParams.stoppingVbp/stoppingAccelMax/stoppingAccelMin` have no
writer and no reader at OURS and are not ported.
`common/numpy_fast.py` is used only by `tools/stopping/check_harsh_stops_model.py`
and `horizon_optimizer.py` and moves to batch 9 with them.
