# Our feature inventory for the FrogPilot Testing refresh

Research snapshot, 2026-10-31. Candidate `!my-fp-new`, `f77b26830f416606b90b9d3bb51efd42c242ac5c`. Comparison base `d2db3fad4b`. The range has 253 commits, 243 non-merges and 10 merges (`git rev-list --count [--no-merges|--merges] d2db3fad4b..HEAD`). No build, replay, device query, fetch, or history mutation was run for this inventory.

This is a proposed disposition table, not port approval. `port-now` preserves today's enabled behavior, including trials. `fold` also preserves behavior, but proposes deleting the switch and its unused alternative after the Phase 4 parity check. A fold is not permission to remove fallback code that still runs on exceptions, other fingerprints, or no-lead stops. `test-first` means omit from the first port and run a separate trial. `drop` means omit dormant or superseded runtime, retaining evidence as needed. `tools-docs` has no on-car behavior.

Evidence precedence is current code, dated repository records, then historical agent memory. `M/<name>.md` below expands to `/Users/radoslawchybicki/.claude/projects/-Users-radoslawchybicki-Repos-openpilot-rch/memory/<name>.md`. Memory records have not been re-certified against their external drive artifacts. In particular, the 2026-10-25 six-flag KEEP result is memory evidence (`M/stopping-cycle-20261009.md:168`), while current enabled states are verified in code. All dispositions and conflict assessments are research judgments, hence **inferences**. “Enabled” below means enabled in this tree, not a fresh observation of the device or its saved Params.

The 15 regroup subjects and their coarse ownership are in `skills/git-commit-cleanup/scripts/classify.py:3` and `regroup_by_feature.py:9`. They are not 15 indivisible product features. The rows below split them. Key commits are anchors, not an exhaustive per-row commit list. Original pre-regroup deployment SHAs in older docs are historical evidence; the in-range regroup commit is the port anchor.

## Corrections to the plan's flag census

An AST read of top-level Boolean assignments finds **37 flags in the three named files, 33 True and 4 False**. `stopping_flags.py` has 31, `longcontrol.py` has 3, and Hyundai `carcontroller.py` has 3. The plan's 35/three-False census misses `DYNAMIC_SCC14_JERK=False` (`carcontroller.py:43`). `GOVERNOR_BAND_PROFILE` is additionally False by derivation (`stopping_flags.py:187`). There are also mode strings and numeric switches: `SERVICE_MODE="LIVE"` (:146), `SERVICE_APPROACH_LAW="governor"` (:158), `ATTRIBUTED_SAFETY="live"` (:163), `WHOLE_APPROACH_GOVERNOR="off"` (:195), `SHOULD_STOP_LOOKAHEAD_S=0.0` (:13), and `SHOULD_STOP_FALLING_EDGE_HOLD_S=0.4` (:9). Outside these three files, `ARBITER_LEGACY_DROPOUT_HOLDS=True` remains at `stop_target_arbiter.py:66`.

## Feature decision table

**100 feature units. Proposed dispositions: port-now 65, test-first 12, fold 8, drop 9, tools-docs 6.**

Paths abbreviated in this table only: `ctl/` = `selfdrive/controls/`, `lib/` = `selfdrive/controls/lib/`, `fp/` = `frogpilot/`, `hy/` = `opendbc_repo/opendbc/car/hyundai/`. Hook pointers name the **current old-base integration point**, not an assertion that the function survives on the new base. H/M/L are inferred conflict risks based on these hooks and `PLAN.md:19` through `:46`; the other research delegate owns the exact upstream delta.

| ID / feature | Key commits | Main files / code evidence | Flag(s), value | Status, date and evidence | Upstream hook; risk | Proposed disposition and reason |
|---|---|---|---|---|---|---|
| B01. Schema, Params, panda ABI and build compatibility | `12ddbd9f9c` | `cereal/log.capnp`, `cereal/custom.capnp`, `common/params_keys.h`, `panda/board/{health,comms_definitions}.h`, `panda/SConscript` | No product flag | Present 10-31; regroup diff `12ddbd9f9c` | cereal consumers, panda build and safety declarations; H | port-now; preserve contracts required by the rows below, reconcile against new schemas. |
| B02. Host macOS Python, msgq and build support | `ebd430f220`, `5bf695c99c`, `c041c19180` | `tools/mac_setup.sh`, `tools/install_python_dependencies.sh`, `msgq_repo/msgq/{event.cc,ipc.cc}`, `common/prefix.py`, `conftest.py` | Platform guards, spawn context | Present 10-03; `AGENTS.md` Local macOS Development and named diffs | SCons, pytest messaging fixtures; M | tools-docs; retain supported local validation and explicit unsupported-platform errors. |
| B03. Registration without legacy Konik credentials | `12ddbd9f9c`, `147908d4c4` | `system/athena/registration.py` and `system/athena/tests/test_registration.py`; regroup diffs | Identity/registration guards | Present 09-06; named commit diffs | Athena register and hardware startup identity; M | port-now; retain startup registration compatibility without restoring obsolete credentials. |
| M01. Vendored tinygrad runtime bump | `e0effcee3f` | `tinygrad_repo/`; whole regroup diff | Backend selected by build | Present 09-06; required by combined-model build (`selfdrive/modeld/SConscript:50`) | tinygrad compiler, QCOM and CPU backends; H | port-now; carry the runtime required by today's model until new-base compatibility is proved. |
| M02. tsfdo combined model and compiled runtime | `12dfa61fbd`, `688003be10` | `selfdrive/modeld/{modeld.py:85,compile_modeld.py:1,parse_model_outputs.py,constants.py,helpers.py}`, `common/file_chunker.py:1`, `models/driving_supercombo.onnx` | Current combined bundle; curvature seed when lateral inactive | Present 10-15; model regroup diff and `modeld.py:308` | ModelState.run, model action/history, SCons compilation; H | port-now; keep the current policy and message contract on day one. |
| M03. tinygrad driver-monitoring warp and NV12 support | `12dfa61fbd` | `selfdrive/modeld/{dmonitoringmodeld.py,compile_dm_warp.py}`, `system/camerad/cameras/nv12_info.py:1`, `selfdrive/modeld/SConscript:151` | Standard driver-monitoring path | Present 09-06; regroup diff | DM ModelState, camera transforms and buffer layout; H | port-now; preserve current driver monitoring and camera conversion. |
| M04. Optional USB GPU / big-model build support | `12dfa61fbd` | `selfdrive/modeld/SConscript:95,127`, `modeld.py:147`, `helpers.py` | Hardware presence AND compiled manifest; `BIG_INTO_SMALL` environment override | Conditional, 10-31; code proves conditional support, not attached hardware | model compiler, AMD queues and backend selection; H | test-first; no device evidence here that this branch is required by today's car. |
| M05. Superseded split models and old model revisions | `12dfa61fbd` | `selfdrive/modeld/models/{driving_vision,driving_policy,big_driving_vision,big_driving_policy}.onnx` deleted by regroup | No active selector for these files | Already removed at HEAD; `git diff --name-status d2db3fad4b HEAD` and `git ls-tree -rl HEAD selfdrive/modeld/models` | Old split vision/policy runtime; H if revived | drop; do not bring deleted assets or old deep_rl3/divided-rl/michael-rl/Rebel/rdf variants back from history. |
| H01. Hyundai radar tracks enablement and fallback | `c1a6a7f7a0` | `hy/enable_radar_tracks.py:1`, `hy/interface.py`, `hy/radar_interface.py`, `hy/values.py` | `Hyundai-RadarTracks` default 0; vehicle/longitudinal gates | Present 09-06; `c1a6a7f7a0`; saved value not read | CarInterface.init, radar UDS setup and RadarInterface; H | port-now; preserve the car's radar source and fallback behavior. |
| H02. Hyundai CAN bounds, frame consistency and factory-cruise compatibility | `c1a6a7f7a0`, `84d49e2ec1`, `55533d5701` | `hy/carcontroller.py:141`, `hy/{hyundaican,hyundaicanfd,carstate}.py`, `hy/tests/test_can_bounds_fork.py:1` | Vehicle/protocol gates | Present 09-26; named diffs; factory standstill field fix 09-25 | CarController.update, SCC12/SCC14 packers, CarState; H | port-now; preserve wire bounds, gas-release behavior and native SCC operation. |
| H03. Longitudinal active with gas and gas-override brake inhibition | `c1a6a7f7a0` | `opendbc_repo/opendbc/safety/longitudinal.h:10`, `safety/modes/hyundai_common.h`, `ctl/controlsd.py:113,208,265`, `selfdrive/selfdrived/helpers.py:21` | `LongitudinalActiveWithGas` default 0; alternativeExperience bit | Present 09-06; regroup diff; code verifies both controls and safety gates | state machine, controlsd output, Hyundai panda safety; H | port-now; retain matched host/panda gas-override semantics, not just the UI setting. |
| H04. VIN-verified persistent fingerprint fallback | `c1a6a7f7a0` | `selfdrive/car/card.py:88,198`, `opendbc_repo/opendbc/car/car_helpers.py`, `selfdrive/car/tests/test_persistent_fingerprint_fallback.py:1` | Recognized cache + VIN validation | Present 09-06; `M/fullupdate-detach-deploy.md`; code guards cache overwrite | Car initialization and fingerprint/firmware query; H | port-now; prevent transient MOCK boots and lost longitudinal settings. |
| H05. StopReq latch and low-speed release protocol | `c1a6a7f7a0` | `hy/carcontroller.py:25,35`, `docs/stopping/on_vehicle_protocols.md` | `STOPREQ_LATCH=True`; set <0.01, release >0.10 m/s | Live protocol, historical KEEP; `docs/stopping/worklog.md:110,202` | CarController.update SCC12 StopReq; H | port-now; preserve proven hold/release protocol, including pedal exits. |
| H06. Dynamic SCC14 jerk experiment | `c1a6a7f7a0` | `hy/carcontroller.py:43,244` | `DYNAMIC_SCC14_JERK=False` | Disabled 10-31; code and `docs/stopping/on_vehicle_protocols.md` | SCC14 jerk fields; M | drop; keep static 3.0/1.0/5.0 behavior and remove dormant dynamic experiment. |
| H07. Report the actual sent acceleration | `c1a6a7f7a0` | `hy/carcontroller.py:49`; `hy/tests/test_can_bounds_fork.py` | `REPORT_SENT_ACCEL=True`, `TELEMETRY_VERSION=2` | Enabled 10-31; flag comment distinguishes engagement-capped wire from pre-cap value | CarController actuatorsOutput; M | port-now; stopping evidence depends on honest sent-wire telemetry. |
| S00. V2 fallback, V3 ownership, stop context/arbiter and safety ledger | `f05c127ec2`, `7f3bbaa6a0`, `d6718d473c` | `lib/longcontrol.py:727,743,1426,1818`, `stopping_service.py:725,1211`, `stop_context.py`, `stop_target_arbiter.py:66`, `lead_provenance.py` | `SERVICE_MODE=LIVE`; `ARBITER_LEGACY_DROPOUT_HOLDS=True` | LIVE since 07-02 (`stopping_flags.py:105-146`); gas-state reset fix 09-26 | LongControl.update, planner target, MPC stop helpers and controlsd; H | port-now; retain finite fallbacks, identity, dropout holds and pedal/fault ownership exits. V2 still computes before V3. |
| S01. Universal in-band governor approach | `f05c127ec2` | `lib/stopping_service.py:133,1232`; `stopping_flags.py:153-162` | `SERVICE_APPROACH_LAW=governor` | Flipped 08-29 after cycle 38 gate; flag comment and `docs/stopping/worklog.md:2726` onward | V3 approach, legacy no-lead law and final jerk limiter; H | port-now; the lead approach law is current behavior, while the no-lead legacy law still runs. |
| S02. Attributed safety release | `f05c127ec2` | `lib/stopping_service.py:1766-1825`; `stopping_flags.py:163-170` | `ATTRIBUTED_SAFETY=live` | Live since 09-05; `M/stopping-redesign-review.md`; code verifies mode and eligibility | Governor safety min, a_pred/a_kin/barrier/monitor and planner attribution; H | port-now; retain live release and every fail-closed veto; prune mode-only alternatives separately. |
| S03. Publish true radar lead distance / single ISD offset | `f05c127ec2` | `stopping_flags.py:68-76`, `ctl/radard.py:792`, `lib/longitudinal_mpc_lib/{long_mpc,stop_target_helpers}.py` | `PUBLISH_TRUE_LEAD_DISTANCE=True` | Flipped 06-10; dated flag comment | RadarD.publish and downstream distance consumers; H | port-now; all consumers must use the same distance convention. |
| S04. shouldStop falling-edge hold | `f05c127ec2` | `lib/stopping_flags.py:8-13`, `longitudinal_planner.py:2487` and adjacent latch | Hold 0.4 s | Enabled 10-31; redesign contract in flag comment | LongitudinalPlanner shouldStop producer; H | port-now; retain stop-intent debounce. |
| S05. Creep-to-stop approach extension | `f05c127ec2` | `lib/stopping_flags.py:15-23`; `longitudinal_planner.py` stopped-lead approach helpers | `SANTA_FE_STOPPED_LEAD_CREEP_APPROACH_EXTENSION=True` | Enabled 10-31; dated route rationale in flag comment | Santa Fe planner smooth-approach cap; H | port-now; retain approach braking in the 0.8-2.5 m/s band. |
| S06. Stopping-lead roll-in floor | `f05c127ec2` | `lib/stopping_flags.py:25-40`; planner `get_santa_fe_stopped_lead_hold_gap_required_decel` | `SANTA_FE_STOPPING_LEAD_ROLL_IN=True` | Enabled 10-31; current flag contract | Planner cap/floor pair; H | port-now; preserve the common rest anchor and early roll-in. |
| S07. Stop commitment necessity envelope | `f05c127ec2` | `lib/stopping_flags.py:42-46`; planner commitment helpers | `SANTA_FE_STOP_COMMIT_ENVELOPE=True` | Enabled 10-31; route 1f47 rationale in code | Planner deepen-only stop floor and lead certificate; H | port-now; preserve the 3 m safety floor. |
| S08. Stop aim commitment envelope | `f05c127ec2` | `lib/stopping_flags.py:47`; planner aim helpers | `SANTA_FE_STOP_AIM_ENVELOPE=True` | Enabled 10-31; cycle 24 reference in code | LongitudinalPlanner aim floor; H | port-now; preserve current early stop commitment. |
| S09. Rest-close floor | `f05c127ec2` | `lib/stopping_flags.py:48`; `longitudinal_planner.py:1214` | `SANTA_FE_REST_CLOSE_FLOOR=True` | Enabled 10-31; cycle 31 E1-R reference | Planner rest-close reference and target authority; H | port-now; retain close-rest protection. |
| S10. Arrived stopped-lead synthetic-target gate | `f05c127ec2` | `lib/stopping_flags.py:55-66`; `stop_target_arbiter.py`, `longitudinal_mpc_lib/stop_target_helpers.py` | `STOPPED_LEAD_ARRIVED_GATE_ENABLED=True` | Enabled but bypassed in terminal-profile scope (`stopping_flags.py:89-92`) | Synthetic target producer; M | port-now; retain non-profile/fallback behavior until reachability is proved, not a blanket fold. |
| S11. Terminal glide profile / common rest anchor | `f05c127ec2` | `lib/stopping_flags.py:78-103`; `stopping_trajectory.py`, `longcontrol.py:1426` | `SANTA_FE_TERMINAL_GLIDE_PROFILE_ENABLED=True` | Enabled 10-31; code documents bypasses and coupled target/hold changes | V2 reference/tracker, planner and stop-target producer; H | port-now; preserve common 4 m reference and cap bypasses. |
| S12. Acceleration tracking trim | `f05c127ec2` | `lib/longcontrol.py:254-305`; `stopping_flags.py:148-151` | `SANTA_FE_ACCEL_TRACKING_TRIM=True` | Enabled 10-31, cycle 32; code provenance | LongControl PID output and delayed untrimmed reference; H | port-now; preserve bounded deepen-only tracking correction. |
| S13. Governor moving-lead profile reference | `ed48c9398b`, `ef5a415572` | `lib/stopping_flags.py:171-176`; `stopping_service.py:133` | `GOVERNOR_PROFILE_REFERENCE=True` | KEEP 09-12; `docs/stopping/universal_stop_program.md:1434-1450` | governor_demand moving-lead reference; H | fold; explicit cycle-53 KEEP supports deleting the SUM-reference alternative. |
| S14. Stay in approach while closing | `ed48c9398b`, `ef5a415572` | `lib/stopping_flags.py:188-194`; `stopping_service.py` approach exit | `SERVICE_STAY_WHILE_CLOSING=True` | KEEP 09-12; same program `:1434-1450` | V3 approach exit and planner-go exemption; H | fold; keep the closing stay with its reviewed moving-lead go exit. |
| S15. Bounded governor recovery brake | `459cd0625e`, `557d94854c` | `lib/stopping_flags.py:177-179`, `stopping_service.py:133` | `GOVERNOR_RECOVERY_BRAKE=True` | Enabled 09-12; `docs/stopping/recovery_brake_2026-09-12.md` | governor demand below reference; H | port-now; preserve the bounded release pending an explicit final keep judgment. |
| S16. Creep guard final floor | `a444338c94`, `c1aac2c09b`, `df0b39e3f0` | `lib/stopping_flags.py:212-218`, `longcontrol.py` final guard and handover | `FINAL_FLOOR=True` | Enabled 09-27; `M/hev-creep-regrab.md`, `docs/stopping/universal_stop_program.md:417` | Final sent-request ceiling and V3 handover; H | port-now; preserve HEV creep suppression and final-writer order. |
| S17. Flat landing | `c1aac2c09b` | `lib/stopping_flags.py:219`; `stopping_service.py:911` | `FLAT_LANDING=True`, floor -0.50 | Enabled 09-27; same KCS2 result `universal_stop_program.md:417` | V3 terminal descent capture; H | port-now; preserve delivered landing baseline. |
| S18. Santa Fe trim handoff below 2.5 m/s | `a40098b403`, `70882f2a58` | `lib/stopping_flags.py:288-292`, `longcontrol.py` trim hold/decay | `SANTA_FE_TRIM_HANDOFF=True` | Deployed 10-03 per `M/stopping-cycle-20261002.md`; current flag verifies enabled | PID trim to service/low-speed output seam; H | port-now; preserve grade compensation and gas cancellation. |
| S19. Release-end rehold behind stopped lead | `2e39594627` | `lib/stopping_flags.py:294-299`; `stopping_service.py` RELEASE end | `RELEASE_END_STOPPED_LEAD_REHOLD=True` | Deployed 10-04; `docs/stopping/universal_stop_program.md:348` | V3 RELEASE completion and hold evidence; H | port-now; retain secure rehold after false departures. |
| S20. Keep planner-go evidence until physical departure | `378e7a6cd4`, `1fec042456` | `lib/stopping_flags.py:301-308` | `RELEASE_GO_EVIDENCE_KEEP=True` | All-five bundle deployed 10-11; `M/stopping-cycle-20261009.md:60-66`; `universal_stop_program.md:240` | V3 go/departure evidence; H | port-now; preserve current release ordering. |
| S21. Owned starting-state transit | `378e7a6cd4`, `1fec042456` | `lib/stopping_flags.py:310-317`; `longcontrol.py:1844` | `OWNED_STARTING_TRANSIT=True` | Enabled 10-11; same bundle record | LongControl starting transition and V3 reseed; H | port-now; preserve service ownership through the state seam. |
| S22. Service hold enters LongControl stopping | `378e7a6cd4`, `1fec042456` | `lib/stopping_flags.py:319-326`; `longcontrol.py:1288-1315` | `SERVICE_HOLD_ENTERS_STOPPING=True` | Enabled 10-11; same bundle record | LongCtrlState and service hold latch; H | port-now; keep secure hold and avoid starting-window resets. |
| S23. Settled StopReq set | `378e7a6cd4`, `1fec042456` | `lib/stopping_flags.py:328-335`; `hy/carcontroller.py:40-42` | `STOPREQ_SETTLED_SET=True` | Enabled 10-11, despite older 10-06 “kept ready” record; current code wins | CarController StopReq set dwell; H | port-now; preserve the settled-hold protocol gate. |
| S24. Hold departure release | `378e7a6cd4`, `1fec042456` | `lib/stopping_flags.py:337-352` | `HOLD_DEPARTURE_RELEASE=True` | Enabled 10-11; same bundle record | V3 physical-go dwell, LongControl pid release and reentry mask; H | port-now; retain clean physical-departure launches. |
| S25. Gap-aware terminal guard arming | `9a5e117bcf`, `d143ef422c`, `2b2284e689` | `lib/stopping_flags.py:222-239`; `stopping_service.py` GAT wait/stall latch | `GAP_AWARE_TERMINAL=True` | Trial since 10-12; KEEP week 10-25, memory `M/stopping-cycle-20261009.md:168`; `M/gap-aware-terminal.md` | Final guard distance arm and stall-entry blackout; H | fold; proposed from week KEEP, retaining accepted late-object/launch costs and 0.5 s blackout. |
| S26. Delivered landing ease | `9e0b3c4e82`, `d143ef422c` | `lib/stopping_flags.py:241-258`; `stopping_service.py` delivered landing | `DELIVERED_LANDING=True` | Trial since 10-12, KEEP week 10-25 per memory above; `universal_stop_program.md:141-164` | V3 wire history, landing offset and creep guard; H | fold; proposed from week KEEP; retain FCW/nearer-target/stall re-press vetoes. |
| S27. Landing stopped-lead radar-bias tolerance | `9d78064f1e`, `72db49be11`, `b47939749a` | `lib/stopping_flags.py:268-276`; `stopping_service.py` landing gates | `LANDING_LEAD_TOLERANCE=True` | Trial since 10-17, KEEP week 10-25 per memory; `M/landing-lead-tolerance.md` | Stopped-lead landing fail-safe; H | fold; proposed from week KEEP, retaining bounded reversal acceptance and identity checks. |
| S28. Entry comfort cap | `44a6e179b2`, `337af0b577`, `badd51d561` | `lib/stopping_flags.py:278-286`; `stopping_service.py:201-297` | `ENTRY_COMFORT_CAP=True` | Trial since 10-13, KEEP week 10-25 per memory; `M/entry-comfort-cap.md` | EntryCapEpisode causal travel predictor and demand budget; H | fold; proposed from week KEEP, retain lost-gap hold and raw-demand reserve. |
| S29. Fast release to existing safety lanes | `e5faa3888e`, `85480ed5cb`, `e22ef8c6c9`, `dd34cb57d4` | `lib/stopping_flags.py:354-371`; `stopping_service.py:1110` | `FAST_RELEASE_TO_LANES=True`, J_UP_FAST 2.0 | Trial since 10-18; KEEP week 10-25 per memory; `M/fast-release-to-lanes.md:8-25` | V3 upward jerk limit only; H | fold; proposed from week KEEP; measured benefit still needs a causal M1 event, not a claimed car improvement. |
| S30. Starting blip guard | `85480ed5cb`, `e22ef8c6c9`, `dd34cb57d4` | `lib/stopping_flags.py:373-379`; `longcontrol.py` starting output | `STARTING_BLIP_GUARD=True` | Trial since 10-18; KEEP week 10-25 per memory; `M/starting-blip-guard.md:8-24` | LongControl stopping-to-starting frame, published wire including trim; H | fold; remove obsolete +0.70 flicker path while keeping normal launches. |
| S31. Lead-follow bounded early launch | `10dc808265`, `d1d4ac7e33`, `64b4fd9d53`, `e79b7e5920` | `lib/stopping_flags.py:381-405`; `stopping_service.py` launch_cmd and `longcontrol.py:1886` final bound | `LAUNCH_FOLLOW=True` | Trial since 10-18; N2 rolling-abort grab at gaps <~6 m open (`M/launch-gap-cycle-20261017.md:132-138`) | V3 release decision, arbiter latch, final output ceiling; H | port-now; preserve today's selected launch behavior, keep trial flag until N2 is judged. |
| S32. Planner floor during stopping | `f05c127ec2` | `lib/longcontrol.py:105-163` | `STOPPING_PLANNER_FLOOR_ENABLED=True` | Enabled 10-31; safety rationale in code | LongControl stopping floor, lead distance and aTarget; H | port-now; preserve deepen-only authority above 0.30 m/s. |
| S33. PID stopped/slowing-lead approach caps | `f05c127ec2` | `lib/longcontrol.py:319-421`; planner smooth-approach helpers | `PID_STOPPED_LEAD_APPROACH_ENABLED=True` | Enabled 10-31; code and `docs/longitudinal_tuning_status.md:41-63` (older status) | PID slew, projected closing risk and planner caps; H | port-now; preserve higher-speed approaches outside service ownership. |
| S34. Plan-to-acceleration interpolation and raw stop-intent semantics | `f05c127ec2` | `lib/drive_helpers.py:59-75`; endpoint diff of get_accel_from_plan | MIN_STABLE_DELAY=0.3; default vEgoStopping=0.3 | Present 09-06; code differs from old base in interpolation and should_stop predicate | Model/planner get_accel_from_plan callers; H | port-now; preserve the actual source signal beneath stopping debounce and service entry. |
| D01. Reverted stop line and band-consistent governor | `56e1512892`, `1503e80ce1` | `lib/stopping_flags.py:49-53,180-187`; planner `:1098-1198,2420`; service `:1432` | `SANTA_FE_STOP_LINE=False`; derived `GOVERNOR_BAND_PROFILE=False` | Reverted 10-05; parked after round 6, `universal_stop_program.md:308-317` | Planner stop line plus governor band profile; H | drop; gate failures remain, do not revive the trial during port. |
| D02. Identification/KCS test-session driving mode | `fa7e19a0ec` through `c1aac2c09b`, `2801d2bf0c` | `lib/identification_hook.py:1-691`, `longcontrol.py:791`, controlsd/card/selfdrived and FP variable wiring | `IDENTIFICATION_HOOK=False`; `IdentificationTestArmed` transient | Off since 09-27, tests completed; `M/identification-hook.md:45-47` | Wheel distance button, wire ownership, set speed and Experimental mask; H | drop; collection ended; retain collected data and offline extractors. |
| D03. Post-stop close-gap positive creep | `f05c127ec2` | `lib/longcontrol.py:587-599,1690`; creep helper family | `STOPPING_CLOSE_GAP_CREEP_ENABLED=False` | Disabled 10-31; `stopping_flags.py:121-124` also excludes post-stop motion under service ownership | LongControl stopping output; H | drop; omit disabled positive post-stop motion, not queue-follow behavior. |
| D04. Whole-approach governor research | `f05c127ec2` | `lib/stopping_flags.py:195`, `stopping_governor.py:1`, `longitudinal_planner.py:838,1857` | `WHOLE_APPROACH_GOVERNOR=off` | Gate failed, disabled research since 09-04; `docs/stopping/worklog.md:2983-3014` | Planner shadow certificate/capture reserve; H | drop; no wire authority, failed recorded-input gate. |
| D05. Old shadow oracle and learned profile selector | `f05c127ec2`, `06da9141e4` | `lib/stopping_shadow.py:1-445`, `stopping_profile_selector.py:1-363`, `longcontrol.py:859-898` | No live authority; V2 no longer constructs shadow_oracle | 10-31 static reachability finding; `stopping_controller_v2.py:78-95`; `longcontrol.py:870-872` returns without oracle | Legacy diagnostic logging/oracle; M | drop; remove orphan runtime imports/helpers; archive offline oracle tooling if needed. |
| D06. shouldStop lookahead experiment | `f05c127ec2` | `lib/stopping_flags.py:11-13`, `longitudinal_planner.py:2487-2489` | `SHOULD_STOP_LOOKAHEAD_S=0.0` | Off by design per code | Planner shouldStop assertion | drop; preserve direct current-time assertion and active falling-edge hold. |
| D07. Orphan Params and stale navigation/Traffic settings residue | `12ddbd9f9c`, `2fe9453d8d` | `common/params_keys.h:191,443,494-500`; `fp/ui/qt/offroad/longitudinal_settings.h:33-35`, `.cc:330` | CENavigation=0, SLCPriority3=None, Traffic defaults | No literal runtime consumer found for SLCPriority3/TrafficJerk keys; only menu residue for CENavigation/TrafficProfile, 10-31 static scan | Registry and legacy settings names; dynamic-key use still needs audit | drop; omit registry/menu residue after dynamic-key check, not the existing live Traffic Mode module. |
| F01. Force Coast profile, floor contract and terminal taper | `86f4985ac0` | `fp/controls/lib/force_coast.py:1-45`, `frogpilot_acceleration.py:49`, `lib/longcontrol.py:84-97,310`, `stopping_flags.py:203-210` | `FORCE_COAST_TERMINAL_TAPER=True`; strength setting default 1.0 | Live contract 09-05; `M/stopping-redesign-review.md`; code describes all consumers | FP acceleration limits, MPC zero-cruise, LongControl and lead-free service entry; H | port-now; retain driver-requested braking floor, deeper-demand pass and soft terminal tail. |
| F02. Following easing and model/ACC brake conditioning | `f3aefffbe7` | `fp/controls/lib/frogpilot_following.py:21`, `lib/longitudinal_planner.py` and `longitudinal_mpc_lib/long_mpc.py` | Following/personality Params, lane/lead gates | Present 09-06; regroup diff; newer census `M/launch-gap-cycle-20261017.md:19-22` | FP following update, MPC obstacle distance and blended demand; H | port-now; preserve current selected following before trialing upstream personalities. |
| F03. Lane-change lead surrogate and provenance telemetry | `f3aefffbe7`, `08c65c5e21`, `e6df3b296b` | `ctl/radard.py:306-695`, `cereal/custom.capnp`, `tools/lane_change/README.md` | Speed >=12 m/s and lane-change/divider gates | Present 09-06; speed guard and telemetry diffs; `2505816924` rejects TTC fail-safe | RadarD lead selection / target-lane release; H | test-first; compare with new-base adjacent-lane behavior before restoring this lead rewrite. |
| F04. Experimental lead/no-lead boost and departing-lead exemption | `f3aefffbe7`, `93d7b319fc` | `lib/longcontrol.py:423`, `longitudinal_planner.py` boost helpers; `fp/common/frogpilot_variables.py` | CEExperimentalBoostGain 1.0, NoLeadGain 0.5, BrakeCutoff -0.2 defaults | Present since 09-06; `M/green-light-launch.md`; `M/launch-gap-cycle-20261017.md:19-22` | Planner acceleration shaping, LongControl close-lead cap; H | port-now; preserve departure exemption and current gains, do not infer a launch gain from a saturating slider. |
| F05. Predicted lead trajectory and predictive CEM triggers | `6e53365233` | `fp/controls/lib/conditional_experimental_mode.py:149-233`, `lib/longitudinal_mpc_lib/long_mpc.py` | Lead/curve/urban-limit/CEM Params | Ported 09-06; commit diff and current code | Model lead trajectory anchored to radar, CEM.check_conditions and MPC extrapolation; H | test-first; new base may replace both prediction and trigger contracts. |
| F06. Persistent manual CEM override / Force Coast status | `f3aefffbe7`, `86f4985ac0` | `fp/controls/lib/conditional_experimental_mode.py:37-55,73,96`, `fp/controls/frogpilot_card.py`, `fp/common/frogpilot_variables.py` | Persistent override file; CEStatus force-coast code 9 | Present 09-06; regroup diffs | FP wheel-button mapping, CEM update and status publication; H | port-now; preserve intentional driver mode overrides across process restarts. |
| C01. Cruise button snap, decrement and long-press isolation | `e08996a5f4` | `selfdrive/car/cruise.py:70-162`, `selfdrive/car/tests/test_cruise_speed.py` | CustomCruise / CustomCruiseLong, metric/imperial | Present 09-06; regroup diff | VCruiseHelper._update_v_cruise_non_pcm | port-now; preserve wheel-button set-speed behavior. |
| C02. Initial set speed | `e08996a5f4` | `selfdrive/car/cruise.py:164`, `fp/common/frogpilot_variables.py`, `common/params_keys.h` | `InitialSetSpeed=160` default | Present 09-06; regroup diff | VCruiseHelper.initialize_v_cruise and longitudinal settings | port-now; retain configured engagement target. |
| C03. Gas override set-speed catch-up | `e08996a5f4` | `selfdrive/car/cruise.py:112`; cruise tests | Short-interval threshold, active gas | Present 09-06; code comment and regroup diff | VCruiseHelper update | port-now; retain automatic set-speed rise during driver override. |
| C04. Curve-aware CSC targets, direct speed factor and learned training | `e08996a5f4` | `fp/controls/lib/curve_speed_controller.py:25,48,88,178`, `frogpilot_acceleration.py:52`, variable migration | CSCSpeedFactor 0 sentinel, CSCBrakingForce 0, high-speed reduction 20 defaults | Present 09-06; regroup diff; learning path still in code | CurveSpeedController.update_target, FP acceleration and UI training status; H | test-first; compare with the new curve/deceleration controls before reapplying tuning. |
| C05. SLC Poland/non-Poland offset cap and gas override floor | `e08996a5f4` | `fp/controls/lib/speed_limit_controller.py:132-149,399-418` | Location boundary, manual/set-speed override gates | Present 09-06; regroup diff | SLC.get_offset / update_override; H | port-now; preserve deliberate regional cap and driver-override behavior. |
| C06. mapd binary refresh | `e08996a5f4` | `fp/navigation/mapd` | Executable asset | Present 09-06; binary diff; no version verification here | Navigation map data and SLC/CSC consumers; H | test-first; retain new upstream binary first unless API comparison proves an incompatibility. |
| L01. Upstream curvature controller | `ee3b295826` | `lib/latcontrol_curvature.py:11` | Controller-type gate | Present 09-06; regroup diff | Controlsd controller choice and curvature clipping; H | test-first; accept the new-base upstream implementation first and check lost local semantics. |
| L02. Santa Fe torque tuning and low-speed feedforward | `c1a6a7f7a0`, `ee3b295826` | `lib/latcontrol_torque.py:30-41,93-99`, `opendbc_repo/opendbc/car/torque_data/params.toml`, `hy/values.py` | Fingerprint gate; max FF scale 1.08 at 9 m/s | Present 09-06; `docs/lateral_tuning_status.md:1-19` (older analysis) | LatControlTorque.update and static torque seeds; H | port-now; preserve vehicle-specific steering baseline. |
| L03. Confidence-gated lane centering | `0c995c8b57`, `4dc5ce819a`, `63dbb4cf98`, `d1a05a045a` | `lib/lane_centering.py:17-32,43,89`, `ctl/controlsd.py` lane-centering call | LaneCentering default 0, authority default 1.0; GAIN 0.60, right-line gate 0.3 | Deployed 10-04 with saved enable=1/authority=0 historically; `M/lane-centering-port.md:53-63`; latest drive verdict absent | Post-model desired curvature before clipping; H | port-now; preserve historical enabled steering behavior, retain the toggle for new-base trial. |
| L04. Lane-center offset setting | `9b9205c5d9` | `lib/lane_centering.py:31-32,43`; lateral settings | `LaneCenterOffset=0.0` default, limit +/-0.3 m | Historical +0.20 trial reset to zero 10-04 (`M/lane-centering-port.md:45-54`) | Lane-centering reference midpoint; M | test-first; default-zero option is not evidence of a required nonzero offset. |
| L05. Lane-change desire state simplification | `ee3b295826` | `lib/desire_helper.py`; regroup diff removes finishing/fade and keep-pulse machinery | Blinker/state/timer gates | Present 09-06; endpoint diff | Modeld DesireHelper.update; H | test-first; use new upstream lane-change state flow first, then compare custom turn behavior. |
| L06. Lateral delay learner speed and cache version | `ee3b295826` | `selfdrive/locationd/lagd.py:25,37,355`; test_lagd diff | MIN_VEGO=50 mph, VERSION=1 | Present 09-06; regroup diff | LateralLagEstimator and retrieve_initial_lag; M | test-first; compare upstream learner gates and invalidate caches only under its current contract. |
| L07. Torque learner longitudinal-acceleration filter | `a7b8051d11` | `selfdrive/locationd/torqued.py:33,177,199`; test_torqued diff | LONG_ACC_THRESHOLD=1 | Ported 10-12; `a7b8051d11` | TorqueEstimator sample acceptance; M | test-first; assess new-base estimator filtering before restoring the port. |
| L08. Calibration and pose validity ports | `147908d4c4` | `selfdrive/locationd/{calibrationd.py,locationd.py}` endpoint diffs | Calibration/pose validity checks | Present 09-06; named diffs | locationd and calibrationd update/publish; M | port-now; preserve valid-pose handling unless upstream already contains it. |
| Y01. Selfdrived input liveness, cruise mismatch and settings safety | `147908d4c4` | `selfdrive/selfdrived/{selfdrived.py,events.py:26,cruise_helpers.py}`, `selfdrive/ui/{qt/offroad/developer_panel.cc,layouts/settings/toggles.py}` | deviceState stale timeout 30 s; recognized-car guards | Present 09-06; regroup diff; `M/fullupdate-detach-deploy.md` | SelfdriveD data/event checks, cruise_mismatch_detected and toggle cleanup; H | port-now; preserve real fault detection and saved-setting protection after transient MOCK boots. |
| Y02. Thermal sampling and blocking-I/O isolation | `147908d4c4` | `system/hardware/hardwared.py:66-124`, `system/statsd.py` | Worker/thread/platform gates | Present 09-06; regroup diff | hardware_thread and ThermalReader; M | port-now; retain liveness during slow thermal reads. |
| Y03. Manager process synchronization and power/watchdog feeding | `147908d4c4` | `system/manager/{manager.py:168-177,process.py}`, `common/watchdog.cc` | Existing process/watchdog gates | Present 09-06; regroup diff | Manager heartbeat, process state and watchdog; H | port-now; retain independent heartbeat and process-state semantics. |
| Y04. UI hang diagnostics and watchdog recovery | `147908d4c4` | `system/manager/process.py:446-469`, `system/manager/test/test_watchdog_diagnostics.py`, `system/hardware/tici/updater_weston` | UI watchdog configured in process_config | Live mitigation, historical root cause 07-19 (`M/ui-watchdog-hang.md`), code present 10-31 | UI process restart, gdb/backtrace capture and Weston; M | port-now; preserve watchdog recovery; diagnostic capture may be simplified after parity. |
| Y05. Logger video segment resynchronization | `147908d4c4` | `system/loggerd/loggerd.cc`, `M/rl3-model-deploy.md` | Video/segment gates | Present 09-06; regroup diff | Encoder segment alignment; H | port-now; preserve video alignment after dropped frames. |
| Y06. UTC clock correction | `147908d4c4` | `system/timed.py` diff; `M/timed-clock-bug.md` | GPS time / timezone handling | Present at HEAD; memory's “fix pending” is older than regroup | timed main clock-set path; M | port-now; retain actual UTC handling; do not port an old pending diagnosis as a new patch. |
| Y07. Background CPU isolation and removal of stale telemetry service | `147908d4c4` | `fp/frogpilot_process.py`, deleted `fp/system/frogpilot_telemetry.py` | Background scheduling; telemetry module absent | Present 09-06; deletion `git diff d2db3fad4b HEAD -- frogpilot/system/frogpilot_telemetry.py` | FrogPilot process loop; new upstream network-write policy unknown; H | port-now; preserve scheduling intent and explicit old telemetry removal; re-audit new outbound writers. |
| Y08. Boot-time NVMe guard, fallback LOG_ROOT and alerts | `2eb3bf6250`, `4314fdd2b2`, `729fb99031`, `f77b26830f` | `system/manager/helpers.py:66-128`, `manager.py:34`, `loggerd/deleter.py`, `fp/common/frogpilot_functions.py`, `selfdrive/selfdrived/events.py:414` | tici only, skips explicit LOG_ROOT; probe 5 s | Shipped 10-25 per `M/device-nvme-dropout.md:20-29`; code and `docs/device_nvme.md:18-46` | manager_init before install/logging, all log writers via environment; H | port-now; preserve engagement and logs when storage fails; disclose mid-drive/fallback recovery gaps. |
| Y09. Multi-root log deletion and fallback disk reserve | `147908d4c4`, `f77b26830f` | `system/loggerd/deleter.py:15-53,97`; `docs/device_nvme.md:40-46` | LOG_ROOT and candidate mounts; internal disk reserve | Present 10-25; regroup plus NVMe diffs | deleter_thread managed roots and space accounting; H | port-now; preserve bounded log storage for ordinary and fallback logging. |
| U01. Staged full update and detached reboot supervisor | `8f6495e903` | `fullupdate.sh:1-555`, `update.sh`, `restart.sh`, `launch_chffrplus.sh` | On-road stage, uptime >=180 s, cancel-file and lock | Present 09-06; `AGENTS.md` Device Update Workflow; historical `M/fullupdate-detach-deploy.md` | Device fetch/reset/stage, supervisor and reboot lifecycle; H | port-now; preserve the authorized update path and boot-load guard. |
| U02. Verified live stock-SCC handoff | `8f6495e903` | `selfdrive/car/live_update_handoff.py:1-197`, `card.py:369-544`, `selfdrive/pandad/{pandad.cc,panda_safety.cc}`, `selfdrive/selfdrived/selfdrived.py` | LiveUpdateHandoffState/Blockers; fresh SCC12 passive verification | Present 09-06; `AGENTS.md` gates; handoff tests | card controls quiesce, panda safety mode, stock SCC verification; H | port-now; never replace with raw ignition-on reboot. |
| U03. Full Update, safe Reboot and staged-update UI | `8f6495e903` | `selfdrive/ui/qt/offroad/{software_settings.cc,settings.cc}`, `qt/onroad/alerts.cc`, Python software settings, `fp/ui/qt/offroad/device_settings.cc` | Ignition-on reboot routed via fullupdate safe reboot; passive Update Ready | Present 09-06; regroup diff and `AGENTS.md` | Software panel actions, update banners and DoReboot handling; H | port-now; UI must use the same handoff contract as shell updates. |
| U04. Tailscale rescue installation and supervision | `8f6495e903` | `system/tailscale/{README.md:1,tailscaled.py:1,tailscale_supervisor.sh:1,bootstrap_tailscale_service.sh:1,install_tailscale.sh:1}` | `TailscaledPid`; systemd/service wrapper | Present 09-06; README and regroup diff | DaemonProcess, manager and boot service; M | port-now; preserve remote access and independent rescue service. |
| V01. Onroad HUD, lead overlays and distance-button preview | `2fe9453d8d` | `fp/ui/qt/onroad/frogpilot_annotated_camera.cc`, `selfdrive/ui/qt/onroad/{hud,model,buttons}.cc`, `hy/carstate.py` | Existing UI toggles, Hyundai preview gates | Present 09-06; regroup diff | AnnotatedCameraWidget, HUD/model overlays; M | port-now; retain visible driver cues and wheel-button feedback. |
| V02. Stopping ownership speed color | `ab3464a77d`, `8c0e000f1d` | `selfdrive/ui/ui.{cc,h}`, `qt/onroad/hud.cc`, `lib/stopping_telemetry.py` | Owner state and drive-start reset | Present 09-19; named diffs | UI state and HUD speed color | port-now; preserve controller ownership feedback without stale carryover. |
| V03. Settings restoration and UI event/repaint hardening | `2fe9453d8d`, `147908d4c4` | `fp/ui/qt/offroad/{frogpilot_settings,longitudinal_settings,vehicle_settings,model_settings}.cc`, `fp/ui/qt/onroad/frogpilot_onroad.cc` | Toggle-dependent visibility and click/repaint gates | Present 09-06; two regroup diffs | FrogPilot settings and onroad event delivery; M | port-now; retain settings required by carried features and repaint/hit-test fixes. |
| V04. Custom personality editor menus | `12ddbd9f9c`, `2fe9453d8d` | `fp/ui/qt/offroad/longitudinal_settings.cc:104-130,294-329`; `common/params_keys.h:158,399,461` | Aggressive/Standard/RelaxedPersonalityProfile default 0 | Present 09-06; menu keys select subpanels, not a proved new on-car personality algorithm | FP longitudinal settings and existing personality values; H | test-first; use new upstream personality UI/contracts before carrying old editors. |
| V05. GM/Honda/new-longitudinal-API settings exposure | `12ddbd9f9c`, `2fe9453d8d` | `fp/ui/qt/offroad/vehicle_settings.cc:179-191,322,429`, `.h:26-29` | ExperimentalGMTune, LongPitch, NewLongAPI, HondaAltTune/LowSpeedPedal/MaxBrake default 0 | Present 09-06; literal reads found in UI only, no added vehicle algorithm proved | Vehicle panel and upstream per-make implementations; H | test-first; these settings are independent of today's Santa Fe safety stack and can use new-base support. |
| T01. Stopping replay, drive report, event store, plant fit and review tools | `06da9141e4`, `455b3f876f`, `da0d299914`, `3618c0042f`, later trial/report commits | `tools/stopping/{sim,review}/`, `drive_report.py`, `run_stopping_cycle.py`, `scoring_config.py` | Offline runners; snapshots/flag counterfactuals | Present 10-18; `tools/stopping/sim/README.md:1`, `docs/stopping/eval.md`; old-commit replay concern in memory | Production LongControl/planner/radard imported by replay; H | tools-docs; preserve acceptance evidence and tools, fix compatibility deliberately, run no corpus sweep in this task. |
| T02. Lane-change A/B and lateral/longitudinal analysis | `e64feb5f32`, `2505816924`, `ee3b295826`, `06da9141e4` | `tools/lane_change/README.md:1`, `radard_ab_replay.py`, `tools/{lateral,longitudinal}/README.md` | Offline reference implementation included | Present 09-06; named diffs | Radar and controller offline readers; M | tools-docs; retain A/B evidence even when the associated runtime starts as test-first. |
| T03. Route synchronization tools and contract | `e08996a5f4`, `ebd430f220` | `tools/route_sync/{common.py,refresh_routes.py,test_refresh_routes.py,README.md}`, `docs/route_refresh_process.md` | Cache/remote-root settings | Committed baseline plus pre-existing uncommitted edits, verified git status 10-31 | Offline SSH/file readers, no on-car driving hook | tools-docs; preserve dirty user work separately; never treat it as part of pinned HEAD. |
| T04. Agent guidance, model import, Git regroup and local workflow helpers | `ebd430f220`, `5bf695c99c` | `AGENTS.md`, `CLAUDE.md`, `skills/`, `.claude/`, `.codex/`, `showlogs.sh`, `showtmuxlogs.sh`, favicon | No driving flag | Present 10-31; regroup diff and current tree | Host tools; local-only | tools-docs; bring useful procedures without executing regroup/history-changing helpers. |
| T05. Program records, worklogs, baselines and archived designs | `8f40ebf23f`, later doc commits | `docs/stopping/README.md:1`, `docs/stopping/worklog.md`, `docs/*_status.md`, `docs/*_worklog.md`, `memory-bank/` | No driving flag | Mixed-date evidence, current source outranks stale summaries | No runtime hook | tools-docs; preserve provenance, including rejected/reverted lines; untracked HANDOVER remains excluded. |

## Dead and removable code candidates

Estimates below are **static source-line estimates**, including comments and declarations, not verified deletion patches. They exclude tests/tools unless stated. They overlap and must not be summed. A disabled assignment does not prove every helper it references is unused elsewhere. The Phase 4 subtraction must prove parity. No active fail-safe, safety lane, V2 fallback, no-lead law, or fault latch is marked dead merely because it resembles legacy code.

| Candidate | Evidence / source location at HEAD | Estimated removable lines and limit |
|---|---|---|
| False `DYNAMIC_SCC14_JERK` | `hy/carcontroller.py:43-48,244-260` (hy abbreviation above); static jerk defaults remain | About 20-30 runtime lines including unused dynamic constants; preserve static SCC14 fields. |
| False `SANTA_FE_STOP_LINE` | `lib/stopping_flags.py:49-53`, `longitudinal_planner.py:287-315,1098-1198,2420`; False derived `GOVERNOR_BAND_PROFILE` at flags :187 and service :1432-1440 | About 170-230 planner/service/flag lines, plus band-only branch of `governor_demand` (:133); `test_stop_line.py` 535 and `test_governor_band_profile.py` 309 lines are associated tests, not runtime. Shared certificate helpers need a reference check. |
| False `IDENTIFICATION_HOOK` temporary mode | `lib/identification_hook.py:1-691`; flag :196-201; `longcontrol.py:791`, `ctl/controlsd.py`, `selfdrive/car/card.py:294`, `selfdrive/selfdrived/selfdrived.py` and `fp/common/frogpilot_variables.py:573` | 691-line module plus roughly 200-350 wiring/UI/state lines. Registry key `IdentificationTestArmed` is temporary too. Offline KCS extraction is not dead driving code. |
| False `STOPPING_CLOSE_GAP_CREEP_ENABLED` | `lib/longcontrol.py:587-599`, helper family below, output gate :1690 onward, state initialization/reset | About 130-190 runtime lines. Preserve unrelated service queue release and standstill safety holds. |
| Numeric-off shouldStop lookahead | `lib/stopping_flags.py:11-13`, `longitudinal_planner.py:2487-2489` | About 6-10 lines. The 0.4 s falling-edge hold is active and remains. |
| Off whole-approach governor | `lib/stopping_flags.py:195`; `stopping_governor.py:1-105`; planner constants :230-239, certificate :838-949, shadow candidate, shadow methods :1837-1960 and publication :2549-2553 | About 350-500 lines total, **conditional** on no shared certificate/capture helper use by live floors; do not remove provenance checks used by active aim/commit logic. |
| Old shadow oracle / learned profile runtime | `lib/stopping_shadow.py:1-445`; `stopping_profile_selector.py:1-363`; `longcontrol.py:859-898` reads absent `shadow_oracle`; V2 constructor :78-95 creates no oracle | 808 module lines plus roughly 40-100 orphan observer/import lines. Offline tools still import these modules, so move/archive tool dependencies before deletion or retain a tools-only copy. |
| Pre-band universal-governor shadow | `lib/longcontrol.py:90,1894-1925`; `stopping_telemetry.py` pre-entry ring; docs `pre_entry_context_2026-09-19.md` | About 40-100 lines if its diagnostic evidence is no longer needed; telemetry-only, **not unreachable**. Do not delete the live service merely because its variable names contain `_shadow_`. |
| Attributed safety alternative modes | `lib/stopping_flags.py:163-170`; `stopping_service.py:1766-1825` and subsequent live target application | About 15-35 lines of mode predicates/plumbing after fixing mode to live. The shared candidate/predictor/trust tests run in live mode and **must stay**. Planner-min retention on ineligible frames is live behavior, not dead “off code”. |
| SERVICE_MODE OFF/SHADOW/LIVE_TERMINAL alternatives | `lib/stopping_flags.py:105-146`; `longcontrol.py:87-91,1288,1818` and service application | About 40-90 mode-selection/comment lines if LIVE is made structural. V2 is still called at :1426, and exceptions can disable service for the drive, so deleting V2's 261 lines is **not** justified. |
| Governor `legacy` selector alternative | `lib/stopping_flags.py:153-162`; `stopping_service.py:879-950,1232` | Only lead-specific alternate selection, roughly 10-30 lines, is potentially removable. `_glide_demand`, `_ease_demand` and terminal descent still serve no-lead or non-governor paths. |
| KEEP-flag legacy branches | S13/S14 plus S25-S30; definition pointers in feature table and consumer tests | Roughly 80-180 runtime lines of alternate branches/selector plumbing, not the feature implementations. Must use the individual source branch and parity check; memory-only KEEP outcomes must be accepted before folding. |
| Superseded ONNX assets | M05; deleted split files in `12dfa61fbd`; HEAD tree has only driving_supercombo.onnx and unchanged dmonitoring_model.onnx | **0 additional removable bytes** for deleted assets. Current driving model is 96,635,310 bytes; DM is 6,954,368 bytes and is not obsolete. No big_driving_supercombo.onnx is tracked at HEAD. |
| Dormant big-model build / USB GPU support | `selfdrive/modeld/SConscript:95-99,127-148`, `modeld.py:147-148`, helper/backend code | Roughly 40-90 lines if hardware absence is confirmed. Not proven dead by a host tree read; M04 is test-first, not drop. `BIG_INTO_SMALL` is a build override, not a current driving setting. |
| Unused / historical offline tools | `tools/stopping/train_profile_selector.py`, `benchmark_controller_variants.py`, `review/whole_approach_replay.py`, `tools/lane_change/radard_upstream_13fa3b292a.py` | **No tool is proven unused by call-site absence**: these are direct CLI/reference tools. Roughly 1,000-2,000 lines are archival candidates, not certified deletions. Keep the reference radar for A/B until F03 is decided; retain KCS analysis. |
| Orphan registry/menu residue | `common/params_keys.h:191,443,494-500`; `frogpilot/ui/qt/offroad/longitudinal_settings.h:33-35`, `.cc:330` | 9 registry rows plus about 5-15 menu/helper lines, conditional on a dynamic-key lookup audit. CENavigation and SLCPriority3 are not proof of a working navigation trigger or third-priority SLC source. |
| Reverted or blocked designs with no current implementation | Radar alignment; CVS; MONOTONE_STOP; RELEASE_AT_DECISION; STARTING_FOLLOWS_PLAN; MOVING_LEAD_GAP; GATARM | **0 attributable runtime lines at this HEAD under those flag names**. Their candidate branches are outside this inventory. Evidence `universal_stop_program.md:258,292-317`, `M/launch-gap-cycle-20261017.md:30-47`, `M/stopping-cycle-20261009.md:171`. Do not resurrect them from merge ancestry or docs. |

## Size of the carried stack

Source is pinned Git data, not dirty working files. Columns count **added non-model text lines in the net endpoint diff** `git diff --no-renames --numstat d2db3fad4b HEAD`, including comments and blanks. These are neither churn across 253 commits nor complete upstream file lengths. Removed lines are stated separately. Model binaries are bytes of changed ONNX blobs still present at HEAD; unchanged DM bytes are excluded. Git heuristically reports the combined ONNX as 249,721 added text lines; these are explicitly excluded from runtime LOC. Two removed big-model symlink records (one line each) are also excluded from source LOC. Other changed binary assets are listed separately.

Per-feature allocation is an **estimate**. Whole-file ownership follows the table's main files. Changed Python lines in named feature functions and small flag-conditioned blocks are reassigned to that leaf feature; the smallest matching span wins. Shared headers, orchestration, comments, schemas and generic tests stay with their file owner, chiefly S00/B01/V03. Thus a zero leaf count means its lines remain in a shared pool or have no unique changed span, not zero implementation cost. C++ UI files are assigned by panel, not split into widgets. Cross-flag tests are counted once. Tinygrad is a separate pool even for its tests/docs; it is not our custom runtime. These allocations are planning estimates, not surgical removal budgets. Arithmetic totals below are exact for the stated endpoint diff after excluding model artifacts. S34 remains in shared S00. L05-L08, B03 and Y09 are split by their individual file counts.

| Feature ID | Runtime code | Tests | Tools | Docs | Vendored tinygrad | Model binary bytes |
|---|---:|---:|---:|---:|---:|---:|
| B01 | 197 | 2 | 0 | 0 | 0 | 0 |
| B02 | 14 | 50 | 21 | 0 | 0 | 0 |
| B03 | 17 | 32 | 0 | 0 | 0 | 0 |
| M01 | 0 | 0 | 0 | 0 | 354,217 | 0 |
| M02 | 762 | 0 | 0 | 0 | 0 | 96,635,310 |
| M03 | 107 | 0 | 0 | 0 | 0 | 0 |
| M04 | 0 | 0 | 0 | 0 | 0 | 0 |
| M05 | 0 | 0 | 0 | 0 | 0 | 0 |
| H01 | 103 | 0 | 0 | 0 | 0 | 0 |
| H02 | 142 | 823 | 0 | 0 | 0 | 0 |
| H03 | 12 | 46 | 0 | 0 | 0 | 0 |
| H04 | 137 | 159 | 0 | 0 | 0 | 0 |
| H05 | 23 | 0 | 0 | 0 | 0 | 0 |
| H06 | 9 | 0 | 0 | 0 | 0 | 0 |
| H07 | 2 | 0 | 0 | 0 | 0 | 0 |
| S00 | 9,246 | 13,881 | 0 | 0 | 0 | 0 |
| S01 | 2 | 0 | 0 | 0 | 0 | 0 |
| S02 | 2 | 353 | 0 | 0 | 0 | 0 |
| S03 | 15 | 0 | 0 | 0 | 0 | 0 |
| S04 | 10 | 113 | 0 | 0 | 0 | 0 |
| S05 | 13 | 0 | 0 | 0 | 0 | 0 |
| S06 | 97 | 426 | 0 | 0 | 0 | 0 |
| S07 | 31 | 0 | 0 | 0 | 0 | 0 |
| S08 | 8 | 0 | 0 | 0 | 0 | 0 |
| S09 | 68 | 0 | 0 | 0 | 0 | 0 |
| S10 | 8 | 0 | 0 | 0 | 0 | 0 |
| S11 | 9 | 4 | 0 | 0 | 0 | 0 |
| S12 | 47 | 453 | 0 | 0 | 0 | 0 |
| S13 | 13 | 4 | 0 | 0 | 0 | 0 |
| S14 | 4 | 0 | 0 | 0 | 0 | 0 |
| S15 | 8 | 0 | 0 | 0 | 0 | 0 |
| S16 | 24 | 481 | 0 | 0 | 0 | 0 |
| S17 | 75 | 0 | 0 | 0 | 0 | 0 |
| S18 | 38 | 0 | 0 | 0 | 0 | 0 |
| S19 | 18 | 298 | 0 | 0 | 0 | 0 |
| S20 | 8 | 134 | 0 | 0 | 0 | 0 |
| S21 | 3 | 0 | 0 | 0 | 0 | 0 |
| S22 | 10 | 0 | 0 | 0 | 0 | 0 |
| S23 | 3 | 0 | 0 | 0 | 0 | 0 |
| S24 | 3 | 304 | 0 | 0 | 0 | 0 |
| S25 | 64 | 480 | 0 | 0 | 0 | 0 |
| S26 | 8 | 321 | 0 | 0 | 0 | 0 |
| S27 | 7 | 130 | 0 | 0 | 0 | 0 |
| S28 | 41 | 288 | 0 | 0 | 0 | 0 |
| S29 | 16 | 2,460 | 0 | 0 | 0 | 0 |
| S30 | 5 | 84 | 0 | 0 | 0 | 0 |
| S31 | 5 | 563 | 0 | 0 | 0 | 0 |
| S32 | 37 | 110 | 0 | 0 | 0 | 0 |
| S33 | 108 | 170 | 0 | 0 | 0 | 0 |
| S34 | 0 | 0 | 0 | 0 | 0 | 0 |
| D01 | 95 | 866 | 0 | 0 | 0 | 0 |
| D02 | 752 | 2,494 | 0 | 0 | 0 | 0 |
| D03 | 71 | 85 | 0 | 0 | 0 | 0 |
| D04 | 317 | 383 | 0 | 0 | 0 | 0 |
| D05 | 955 | 310 | 0 | 0 | 0 | 0 |
| D06 | 4 | 0 | 0 | 0 | 0 | 0 |
| D07 | 0 | 0 | 0 | 0 | 0 | 0 |
| F01 | 163 | 649 | 0 | 0 | 0 | 0 |
| F02 | 55 | 79 | 0 | 0 | 0 | 0 |
| F03 | 494 | 365 | 0 | 0 | 0 | 0 |
| F04 | 0 | 0 | 0 | 0 | 0 | 0 |
| F05 | 154 | 192 | 0 | 0 | 0 | 0 |
| F06 | 13 | 0 | 0 | 0 | 0 | 0 |
| C01 | 39 | 181 | 0 | 0 | 0 | 0 |
| C02 | 0 | 0 | 0 | 0 | 0 | 0 |
| C03 | 0 | 0 | 0 | 0 | 0 | 0 |
| C04 | 144 | 105 | 0 | 0 | 0 | 0 |
| C05 | 101 | 0 | 0 | 0 | 0 | 0 |
| C06 | 0 | 0 | 0 | 0 | 0 | 0 |
| L01 | 59 | 0 | 0 | 0 | 0 | 0 |
| L02 | 14 | 0 | 0 | 0 | 0 | 0 |
| L03 | 103 | 170 | 0 | 0 | 0 | 0 |
| L04 | 0 | 0 | 0 | 0 | 0 | 0 |
| L05 | 16 | 0 | 0 | 0 | 0 | 0 |
| L06 | 7 | 17 | 0 | 0 | 0 | 0 |
| L07 | 4 | 31 | 0 | 0 | 0 | 0 |
| L08 | 11 | 0 | 0 | 0 | 0 | 0 |
| Y01 | 130 | 61 | 0 | 0 | 0 | 0 |
| Y02 | 137 | 0 | 0 | 0 | 0 | 0 |
| Y03 | 489 | 38 | 0 | 0 | 0 | 0 |
| Y04 | 0 | 280 | 0 | 0 | 0 | 0 |
| Y05 | 20 | 0 | 0 | 0 | 0 | 0 |
| Y06 | 3 | 0 | 0 | 0 | 0 | 0 |
| Y07 | 7 | 0 | 0 | 0 | 0 | 0 |
| Y08 | 78 | 250 | 0 | 54 | 0 | 0 |
| Y09 | 59 | 40 | 0 | 0 | 0 | 0 |
| U01 | 587 | 0 | 0 | 0 | 0 | 0 |
| U02 | 400 | 703 | 0 | 0 | 0 | 0 |
| U03 | 224 | 0 | 0 | 0 | 0 | 0 |
| U04 | 804 | 0 | 0 | 72 | 0 | 0 |
| V01 | 404 | 0 | 0 | 0 | 0 | 0 |
| V02 | 0 | 0 | 0 | 0 | 0 | 0 |
| V03 | 209 | 0 | 0 | 0 | 0 | 0 |
| V04 | 56 | 0 | 0 | 0 | 0 | 0 |
| V05 | 49 | 0 | 0 | 0 | 0 | 0 |
| T01 | 0 | 12,005 | 35,695 | 739 | 0 | 0 |
| T02 | 0 | 0 | 3,671 | 227 | 0 | 0 |
| T03 | 0 | 334 | 719 | 190 | 0 | 0 |
| T04 | 0 | 0 | 510 | 428 | 0 | 0 |
| T05 | 0 | 0 | 0 | 27,553 | 0 | 0 |
| **Total** | **18,806** | **41,807** | **40,616** | **29,263** | **354,217** | **96,635,310** |

Non-model text lines removed across the endpoint diff: **280,623**. Added non-model text lines: **484,709**, across 1,758 changed paths with rename detection disabled. The deletion total includes replacement of vendored tinygrad; it is not all dead custom code. Shared S00 retains the V2 fallback, live no-lead paths and cross-feature glue.

| Changed binary/model artifact path | Base bytes | HEAD bytes | Feature |
|---|---:|---:|---|
| `favicon.png` | 0 | 350 | T04 |
| `frogpilot/navigation/mapd` | 21,341,584 | 21,341,584 | C06 |
| `selfdrive/modeld/models/big_driving_policy.onnx` (symlink) | 19 | 0 | M05 |
| `selfdrive/modeld/models/big_driving_vision.onnx` (symlink) | 19 | 0 | M05 |
| `selfdrive/modeld/models/driving_supercombo.onnx` | 0 | 96,635,310 | M02 |
| `selfdrive/modeld/models/driving_policy.onnx` | 13,926,324 | 0 | M05 |
| `selfdrive/modeld/models/driving_vision.onnx` | 46,271,942 | 0 | M05 |
| `system/hardware/tici/updater_weston` | 10,448,792 | 10,448,792 | Y04 |
| `tinygrad_repo/examples/flux1_seed0.png` | 292,350 | 0 | M01 |
| `tinygrad_repo/examples/sdxl_seed0.png` | 1,531,568 | 1,671,737 | M01 |
| `tinygrad_repo/examples/stable_diffusion_seed0.png` | 464,573 | 377,786 | M01 |
| `tinygrad_repo/extra/sqtt/examples/gfx1100/profile_empty_run_0.pkl` | 0 | 66,725 | M01 |
| `tinygrad_repo/extra/sqtt/examples/gfx1100/profile_empty_run_1.pkl` | 0 | 66,740 | M01 |
| `tinygrad_repo/extra/sqtt/examples/gfx1100/profile_gemm_run_0.pkl` | 0 | 70,989 | M01 |
| `tinygrad_repo/extra/sqtt/examples/gfx1100/profile_gemm_run_1.pkl` | 0 | 71,008 | M01 |
| `tinygrad_repo/extra/sqtt/examples/gfx1100/profile_plus_run_0.pkl` | 0 | 83,106 | M01 |
| `tinygrad_repo/extra/sqtt/examples/gfx1100/profile_plus_run_1.pkl` | 0 | 82,917 | M01 |
| `tinygrad_repo/extra/sqtt/examples/gfx1100/profile_sync_run_0.pkl` | 0 | 137,026 | M01 |
| `tinygrad_repo/extra/sqtt/examples/gfx1100/profile_sync_run_1.pkl` | 0 | 137,010 | M01 |
| `tinygrad_repo/extra/sqtt/examples/gfx1200/profile_empty_run_0.pkl` | 0 | 45,072 | M01 |
| `tinygrad_repo/extra/sqtt/examples/gfx1200/profile_empty_run_1.pkl` | 0 | 45,065 | M01 |
| `tinygrad_repo/extra/sqtt/examples/gfx1200/profile_gemm_run_0.pkl` | 0 | 51,709 | M01 |
| `tinygrad_repo/extra/sqtt/examples/gfx1200/profile_gemm_run_1.pkl` | 0 | 51,739 | M01 |
| `tinygrad_repo/extra/sqtt/examples/gfx1200/profile_handwritten_run_0.pkl` | 0 | 56,301 | M01 |
| `tinygrad_repo/extra/sqtt/examples/gfx1200/profile_handwritten_run_1.pkl` | 0 | 56,173 | M01 |
| `tinygrad_repo/extra/sqtt/examples/gfx1200/profile_plus_run_0.pkl` | 0 | 60,241 | M01 |
| `tinygrad_repo/extra/sqtt/examples/gfx1200/profile_plus_run_1.pkl` | 0 | 60,227 | M01 |
| `tinygrad_repo/extra/sqtt/examples/gfx1200/profile_sync_run_0.pkl` | 0 | 97,578 | M01 |
| `tinygrad_repo/extra/sqtt/examples/gfx1200/profile_sync_run_1.pkl` | 0 | 97,696 | M01 |
| `tinygrad_repo/extra/sqtt/examples/gfx950/profile_empty_run_0.pkl` | 0 | 99,408 | M01 |
| `tinygrad_repo/extra/sqtt/examples/gfx950/profile_empty_run_1.pkl` | 0 | 99,294 | M01 |
| `tinygrad_repo/extra/sqtt/examples/gfx950/profile_gemm_run_0.pkl` | 0 | 130,767 | M01 |
| `tinygrad_repo/extra/sqtt/examples/gfx950/profile_gemm_run_1.pkl` | 0 | 130,773 | M01 |
| `tinygrad_repo/extra/sqtt/examples/gfx950/profile_plus_run_0.pkl` | 0 | 130,351 | M01 |
| `tinygrad_repo/extra/sqtt/examples/gfx950/profile_plus_run_1.pkl` | 0 | 130,345 | M01 |
| `tinygrad_repo/spec/tinyspec.pdf` | 0 | 98,812 | M01 |

These are Git blob sizes (`git cat-file -s <ref>:<path>`). A zero means absent at that endpoint. The active model is an ordinary Git blob; the two deleted big-model names were 19-byte symlinks; historical deleted model sizes are not current port payload.

## Params keys added relative to the base

**45 net-added names**, all in `common/params_keys.h`; `common/params.cc` has no net change in this range. Registry ownership below does not claim original authorship: compatibility imports can add an upstream name. Defaults are declarations, not saved device values. Flags may use memory Params or marker files separately. The existing DrivingModelName default also changes from WMI to POP in the registry, while M02 actually runs the combined tsfdo asset; do not use that display label to identify the model (`12ddbd9f9c`, `12dfa61fbd`).

| Key | Registry evidence | Owner | Type / declared default / lifetime |
|---|---|---|---|
| `TailscaledPid` | `common/params_keys.h:16` | U04 | `PERSISTENT, INT` |
| `ExperimentalLongitudinalEnabled` | `common/params_keys.h:42` | B01 | `PERSISTENT  /  DEVELOPMENT_ONLY, BOOL, "0"` |
| `LiveUpdateHandoffBlockers` | `common/params_keys.h:86` | U02 | `CLEAR_ON_MANAGER_START  /  CLEAR_ON_ONROAD_TRANSITION, STRING` |
| `LiveUpdateHandoffState` | `common/params_keys.h:87` | U02 | `CLEAR_ON_MANAGER_START  /  CLEAR_ON_ONROAD_TRANSITION, STRING` |
| `LongitudinalActiveWithGas` | `common/params_keys.h:89` | H03 | `PERSISTENT, BOOL, "0"` |
| `Offroad_StorageMissing` | `common/params_keys.h:103` | Y08 | `CLEAR_ON_MANAGER_START, JSON` |
| `UsbGpuCompiled` | `common/params_keys.h:139` | M04 | `CLEAR_ON_MANAGER_START, BOOL` |
| `UsbGpuPresent` | `common/params_keys.h:140` | M04 | `CLEAR_ON_MANAGER_START, BOOL` |
| `AggressivePersonalityProfile` | `common/params_keys.h:158` | V04 | `PERSISTENT, BOOL, "0", "0", 2` |
| `BuildMetadata` | `common/params_keys.h:168` | V03 | `PERSISTENT, STRING, "", "", 0` |
| `CalibratedLateralAccelerationOverride` | `common/params_keys.h:173` | C04 | `PERSISTENT, FLOAT, "0.0", "0.0", 2` |
| `CECscCurves` | `common/params_keys.h:184` | F05 | `PERSISTENT, BOOL, "0", "0", 1` |
| `CEExperimentalBoostGain` | `common/params_keys.h:185` | F04 | `PERSISTENT, FLOAT, "1.0", "1.0", 2` |
| `CEExperimentalBoostBrakeCutoff` | `common/params_keys.h:186` | F04 | `PERSISTENT, FLOAT, "-0.2", "-0.2", 2` |
| `CEExperimentalNoLeadBoostGain` | `common/params_keys.h:187` | F04 | `PERSISTENT, FLOAT, "0.5", "0.5", 2` |
| `CEForceCoastStrength` | `common/params_keys.h:188` | F01 | `PERSISTENT, FLOAT, "1.0", "1.0", 2` |
| `CENavigation` | `common/params_keys.h:191` | D07 | `PERSISTENT, BOOL, "0", "0", 1` |
| `CSCBrakingForce` | `common/params_keys.h:207` | C04 | `PERSISTENT, FLOAT, "0.0", "0.0", 2` |
| `CSCBrakingForceHighSpeedReduction` | `common/params_keys.h:208` | C04 | `PERSISTENT, FLOAT, "20.0", "20.0", 2` |
| `CSCSpeedFactor` | `common/params_keys.h:209` | C04 | `PERSISTENT, FLOAT, "0.0", "0.0", 2` |
| `ExperimentalGMTune` | `common/params_keys.h:255` | V05 | `PERSISTENT, BOOL, "0", "0", 2` |
| `HumanFollowing` | `common/params_keys.h:281` | F02 | `PERSISTENT, BOOL, "1", "0", 2` |
| `HondaAltTune` | `common/params_keys.h:283` | V05 | `PERSISTENT, BOOL, "0", "0", 2` |
| `HondaLowSpeedPedal` | `common/params_keys.h:284` | V05 | `PERSISTENT, BOOL, "0", "0", 2` |
| `HondaMaxBrake` | `common/params_keys.h:285` | V05 | `PERSISTENT, BOOL, "0", "0", 2` |
| `InitialSetSpeed` | `common/params_keys.h:293` | C02 | `PERSISTENT, INT, "160", "160", 1` |
| `LaneCenterOffset` | `common/params_keys.h:302` | L04 | `PERSISTENT, FLOAT, "0.0", "0.0", 2` |
| `LaneCentering` | `common/params_keys.h:303` | L03 | `PERSISTENT, BOOL, "0", "0", 2` |
| `LaneCenteringE2EAuthority` | `common/params_keys.h:304` | L03 | `PERSISTENT, FLOAT, "1.0", "1.0", 2` |
| `LongDistanceFactor` | `common/params_keys.h:318` | F02 | `PERSISTENT, FLOAT, "1.0", "1.0", 3` |
| `LongPitch` | `common/params_keys.h:322` | V05 | `PERSISTENT, BOOL, "0", "0", 2` |
| `NewLongAPI` | `common/params_keys.h:359` | V05 | `PERSISTENT, BOOL, "0", "0", 2` |
| `IdentificationTestArmed` | `common/params_keys.h:362` | D02 | `CLEAR_ON_MANAGER_START  /  CLEAR_ON_OFFROAD_TRANSITION, BOOL, "0", "0"` |
| `Hyundai-RadarTracks` | `common/params_keys.h:378` | H01 | `PERSISTENT, BOOL, "0", "0", 2` |
| `RelaxedPersonalityProfile` | `common/params_keys.h:399` | V04 | `PERSISTENT, BOOL, "0", "0", 2` |
| `SLCPriority3` | `common/params_keys.h:443` | D07 | `PERSISTENT, STRING, "None", "None", 2` |
| `ShortDistanceFactor` | `common/params_keys.h:454` | F02 | `PERSISTENT, FLOAT, "1.0", "1.0", 3` |
| `StandardPersonalityProfile` | `common/params_keys.h:461` | V04 | `PERSISTENT, BOOL, "0", "0", 2` |
| `TrafficFollow` | `common/params_keys.h:494` | D07 | `PERSISTENT, FLOAT, "0.5", "0.5", 2` |
| `TrafficJerkAcceleration` | `common/params_keys.h:495` | D07 | `PERSISTENT, FLOAT, "50.0", "50.0", 3` |
| `TrafficJerkDanger` | `common/params_keys.h:496` | D07 | `PERSISTENT, FLOAT, "100.0", "100.0", 3` |
| `TrafficJerkDeceleration` | `common/params_keys.h:497` | D07 | `PERSISTENT, FLOAT, "50.0", "50.0", 3` |
| `TrafficJerkSpeed` | `common/params_keys.h:498` | D07 | `PERSISTENT, FLOAT, "50.0", "50.0", 3` |
| `TrafficJerkSpeedDecrease` | `common/params_keys.h:499` | D07 | `PERSISTENT, FLOAT, "50.0", "50.0", 3` |
| `TrafficPersonalityProfile` | `common/params_keys.h:500` | D07 | `PERSISTENT, BOOL, "0", "0", 2` |

Consumer checks: U04 uses `system/manager/process_config.py:78`; U02 names both keys in `selfdrive/car/live_update_handoff.py:6-7`; M04 publishes hardware/compiled state at `selfdrive/modeld/modeld.py:150-151`; Y08 sets the alert through manager startup and `selfdrive/selfdrived/alerts_offroad.json:49`. B01 ExperimentalLongitudinalEnabled is retained in the variable cleanup list (`frogpilot/common/frogpilot_variables.py:165`), not evidence of another enabled longitudinal implementation. BuildMetadata is written at `frogpilot/common/frogpilot_functions.py:77`.

The CEM/CSC settings have direct reads at `frogpilot/common/frogpilot_variables.py:447-468`; lane centering at :632-635; HumanFollowing and distance factors at :653-656; initial speed at :683. Personality menu ownership is `frogpilot/ui/qt/offroad/longitudinal_settings.cc:104-130,294-330`; other-vehicle controls are at `vehicle_settings.cc:179-191`. D07 keys need a dynamic-name audit before removal: no literal runtime reader was found for SLCPriority3 or the TrafficJerk names; this is a bounded static finding, not a proof about computed Params names. IdentificationTestArmed has several real readers, all part of disabled test-mode wiring (`frogpilot/controls/frogpilot_planner.py:67`, `selfdrive/car/card.py:294`). No key is removed by this research.

## Open questions and evidence still needed

1. **N2 on the enabled launch trial.** Is the rolling-abort grab at rest gaps below roughly 6 m resolved or accepted? Code still enables LAUNCH_FOLLOW (`stopping_flags.py:402`); last topic record says open (`M/launch-gap-cycle-20261017.md:132-138`). Need the later LAUNCH drive rows and a close-gap re-stop result. Preserve the flag and current behavior until that decision; do not fold it with the six week-judge flags.
2. **Authority for six folds and the certification defect.** The 10-25 KEEP result exists in memory, not a new committed program entry. Need the week-judge report identity/results and the owner's acceptance of folding LAND/GAT/ENTRY/CAPT/REL/BLIP. Also need the independent BASE certificate-lifetime safety finding identified in `M/stopping-cycle-20261009.md:171`; no fix or full reproducer was supplied here. This is not grounds to claim a new defect or alter safety logic during inventory.
3. **Current lateral selection and outcome.** Latest memory has LaneCentering on, authority 0, offset 0, gain 0.60, but no final drive verdict after the gain increase (`M/lane-centering-port.md:53-63`). Need saved current Params and a marked two-way-road drive review. New-base curvature/adjacent-lane changes can change its effect.
4. **Which new-base mechanisms replace custom runtime?** Need the upstream delegate's exact hook/contract comparison for model compilation and Model Manager, predictive lead/CEM, lane-change surrogate, curve braking, following and regional SLC. No local code evidence proves equivalence. USB GPU presence and any requirement for the absent big-model asset are also unverified (`SConscript:127-135`).
5. **NVMe fallback scope and recovery.** Boot fallback is implemented, but memory says Radek never chose A/B/C (`M/device-nvme-dropout.md:27-29`). Need an explicit retention decision, mid-drive dropout policy, fallback-route recovery/sync plan, and physical evidence for PCIe recovery. `docs/device_nvme.md:40-46` confirms fallback logs are not synced and remain after the NVMe returns. Keep the guard on day one; do not silently broaden it.
6. **Which old diagnostics may be retired?** Need direct CLI usage/owner decisions before dropping selector-training, whole-approach replay and old shadow tools. No import-reference scan can prove a direct CLI unused. Shared live safety fallbacks remain in S00.
7. **Dirty work ownership.** Need Radek's/host's choice for the four pre-existing modified route-sync/contract files and untracked `docs/stopping/HANDOVER_2026-09-26.md`. This report inventories committed HEAD, with these exclusions recorded; it does not settle them (`PLAN.md:49-52`).
8. **Model/update deployment facts.** This read-only task did not verify the device HEAD, saved gains, radar-track setting, daemon state, or auto-update behavior. `M/fullupdate-detach-deploy.md` contains conflicting dated auto-deploy observations. Use actual current update/watcher evidence before a push, as the plan already requires. The refresh program has no push/deploy authorization.


## Research verification

- Pinned branch/HEAD were read before research and still match `!my-fp-new` / `f77b26830f416606b90b9d3bb51efd42c242ac5c` at completion.
- Static checks confirm 100 unique feature IDs, one size row per ID, 45 net-added Params names, and coverage of all 37 top-level Boolean flags in the three named driving files. All abbreviated commit anchors resolve locally with `git cat-file --batch-check`. Source-line and byte totals reconcile to the endpoint diff after excluding ONNX/model-symlink records from LOC.
- No build, pytest, corpus harness, live subscription, device operation, network lookup or candidate behavior test was run. Historical PASS/KEEP/SHIP language above is explicitly attributed to records, not a claim of current verification. Static reachability and removal budgets remain estimates until a subtraction diff exists.
- The four pre-existing modified route-sync/contract files and untracked HANDOVER remain outside this artifact. Only INVENTORY_OURS.md was written by this research. The upstream delegate's output is untouched.
