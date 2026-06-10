# Stopping-stack parameters

GENERATED FILE -- do not edit by hand. Source of truth:
`selfdrive/controls/lib/stopping_params.py` (regenerate with
`python3 -m openpilot.selfdrive.controls.lib.stopping_params`).
`test_stopping_params.py` asserts this file matches the dataclass.

Row numbers are the spec section-3 parameter table rows; multiple fields can share a row
(multi-table parameter groups). Provenance cites the legacy forest source at base commit
3be25f5240 and the spec preserve group (G1-G17, spec section 3.1). Initial values reproduce
the forest envelope, not improve it.

Conventions: accel m/s^2 (negative = braking); jerk m/s^3 as positive magnitudes; distances m;
speeds m/s. Tables interpolate via np.interp on the bp axis (v_ego unless noted); legacy
per-frame steps are converted to physical rates by x100 (100 Hz loop).

| # | Name | Unit | Value | Provenance |
|---|------|------|-------|------------|
| 1 | `ACTUATOR_DELAY_S` | s | 0.05 | G2: controller delay compensation, delay_frames = round(0.05/dt) (stopping_controller.py:816) |
| 2 | `CMD_HISTORY_LEN` | frames | 48 | G2: COMMAND_HISTORY_LEN (stopping_controller.py:12) |
| 3 | `PLANT_MODEL_REF` | - | intercept=-0.0032409272, a_ego_prev=0.87153037, accel_cmd_delayed=0.37169542, v_ego=0.013537141, relief=0.28700394, low_speed=-0.014521878, cmd_x_low_speed=-0.45138721 | G1: archived 20260514 fit, SHADOW_MODEL_COEFFICIENTS (stopping_shadow.py:25-33); archived in docs/stopping/archive/; refit per spec 7.5 |
| 3 | `PLANT_MODEL_DT_FIT_S` | s | 0.1 | G1: SHADOW_MODEL_DT_S (stopping_shadow.py:22); dt the coefficients were fitted at |
| 3 | `PLANT_MODEL_DELAY_S` | s | 0.1 | G1: archived 20260514 fit dead time, delay_frames=1 @ 0.1 s (docs/stopping/archive/) |
| 4 | `LOW_SPEED_AUTHORITY_REF` | m/s | 1.2 | G1: SHADOW_MODEL_LOW_SPEED_REF (stopping_shadow.py:34) |
| 5 | `RELIEF_CMD_THRESHOLD` | m/s^2 | -0.25 | G1: SHADOW_MODEL_RELIEF_CMD_THRESHOLD (stopping_shadow.py:35) |
| 6 | `V_NEAR_HOLD` | m/s | 0.85 | G6: NEAR_HOLD phase bound, near_hold_speed_mps (stopping_controller.py:16-18, :37) |
| 7 | `V_SETTLE` | m/s | 0.06 | G6: HOLD phase bound, hold_speed_mps + settle group (stopping_controller.py:34-42) |
| 8 | `V_STANDSTILL_SETTLED` | m/s | 0.02 | G6: standstill_settle_speed_mps (stopping_controller.py:40); below wheel-speed standstill 0.104 m/s |
| 9 | `A_HOLD_TABLE` | m/s^2 | bp=[0, 0.02, 0.06] -> [-0.16, -0.15, -0.13] | G6: HOLD phase hold_target (stopping_controller.py:378) |
| 10 | `A_HOLD_RELAXED_TABLE` | m/s^2 | bp=[0, 0.02] -> [-0.12, -0.1] | G6: standstill_relax target (stopping_controller.py:2450-2465) |
| 11 | `T_HOLD_RELAX_S` | s | 0.8 | G6: standstill_relax_time_s (stopping_controller.py:42) |
| 12 | `A_NEAR_HOLD_TABLE` | m/s^2 | bp=[0.06, 0.15, 0.3, 0.55, 0.85] -> [-0.14, -0.17, -0.2, -0.18, -0.15] | G6: NEAR_HOLD hold_target (stopping_controller.py:371) -- TERMINAL envelope |
| 13 | `A_END_STOP_TABLE` | m/s^2 | bp=[0, 0.1, 0.15, 0.25, 0.6] -> [-0.255, -0.255, -0.3, -0.42, -0.68] | G7: end_stop_brake_cap (stopping_controller.py:2055); nominal soft cap -0.275 |
| 14 | `A_APPROACH_FLOOR_TABLE` | m/s^2 | bp=[0.9, 1.2, 1.6] -> [-0.4, -0.45, -0.5] | G6: APPROACH floor (stopping_controller.py:359-369, :363) |
| 15 | `A_DESIRED_LOWSPEED_TABLE` | m/s^2 | bp=[0.06, 0.2, 0.5, 0.85, 1.2] -> [-0.26, -0.32, -0.42, -0.52, -0.6] | G9: desired_low_speed_accel (stopping_controller.py:849-860, :850) |
| 16 | `J_BRAKE_TABLE` | m/s^3 | bp=[0.06, 0.55, 0.85, 1.2] -> [0.6, 0.8, 0.9, 0.7] | G6/G15: NEAR_HOLD/APPROACH brake_step x100 (stopping_controller.py:368, :375) |
| 17 | `J_RELEASE_TABLE` | m/s^3 | bp=[0, 0.06, 0.2, 0.55, 0.85] -> [0.14, 0.08, 0.12, 0.22, 0.3] | G6: HOLD/NEAR_HOLD release_step x100 (stopping_controller.py:376, :383) |
| 18 | `J_RELEASE_LOCKED_TABLE` | m/s^3 | bp=[0, 0.2, 0.5, 1.2] -> [0.1, 0.15, 0.3, 0.6] | G4: locked release cap x100 (stopping_controller.py:1774). Retirement: spec 3.2 row 4 |
| 19 | `J_SETTLE_RELEASE` | m/s^3 | 0.9 | G7: end-stop fast release 0.009/frame at v=0 (stopping_controller.py:2383-2392) |
| 20 | `DIST_PUSH_THRESH_V_SPLIT` | m/s | 0.08 | G4: disturbance_threshold speed split (stopping_controller.py:99); STEP, not interp |
| 20 | `DIST_PUSH_THRESH_LOW` | m/s^2 | 0.04 | G4: disturbance_threshold below split (stopping_controller.py:99) |
| 20 | `DIST_PUSH_THRESH_HIGH` | m/s^2 | 0.03 | G4: disturbance_threshold at/above split (stopping_controller.py:99) |
| 20 | `DIST_PUSH_V_MIN` | m/s | 0.002 | G4: push detection speed gate, v_ego > 0.002 (stopping_controller.py:101) |
| 20 | `DIST_PUSH_V_MAX` | m/s | 1.2 | G4: push detection speed gate, v_ego < 1.2 (stopping_controller.py:102) |
| 20 | `DIST_PUSH_MIN_BRAKE` | m/s^2 | -0.05 | G4: push detection braking gate, last_output_accel < -0.05 (stopping_controller.py:103) |
| 21 | `DIST_LPF_TAU_S` | s | 0 | new estimator filter (spec 5.5.2); 0.0 = KILL SWITCH bypass to legacy single-frame semantics |
| 22 | `T_RELEASE_INHIBIT_TABLE` | s | bp=[0, 0.2, 0.6, 1.2] -> [1.1, 0.95, 0.7, 0.5] | G4: lock frames /100 (stopping_controller.py:107) |
| 23 | `A_DISTURBANCE_FLOOR_TABLE` | m/s^2 | bp=[0, 0.12, 0.25, 0.5, 1.2] -> [-0.34, -0.31, -0.26, -0.18, -0.11] | G4: lock_floor (stopping_controller.py:1764-1776, :1775) -- also the hill-hold deepening bound |
| 24 | `A_PUSH_RELIEF_CAP_TABLE` | m/s^2 | bp=[0, 0.6, 1.2, 1.8, 2.5] -> [-0.3, -0.34, -0.38, -0.42, -0.46] | G5: clutch/TC push relief target base (stopping_controller.py:1349-1363, :1356) |
| 24 | `PUSH_RELIEF_V_MIN` | m/s | 0.12 | G5: legacy outer speed gate, 0.12 < v_ego (stopping_controller.py:829) |
| 24 | `PUSH_RELIEF_V_MAX` | m/s | 2.5 | G5: legacy outer speed gate, v_ego < 2.5 (stopping_controller.py:829) |
| 25 | `A_ARREST_MAX_TABLE` | m/s^2 | bp=[0, 0.03, 0.06, 0.08] -> [-1.4, -1.15, -0.85, -0.56] | G8: rebound arrest cap (stopping_controller.py:2021) -- absolute deepening clamp in SETTLE/HOLD |
| 26 | `ROLLOUT_BUDGET_NO_TARGET_M` | m | 2 | G12: shadow score rollout knee, no explicit target (stopping_shadow.py:393-409, :397) |
| 26 | `ROLLOUT_BUDGET_EXPLICIT_M` | m | 1.25 | G12: shadow score rollout knee, explicit target (stopping_shadow.py:398-399) |
| 27 | `ROLLOUT_DECAY_MPS` | m/s | 0.35 | G9: standstill_rollout_decay_mps (stopping_controller.py:39) |
| 28 | `RECOVERY_ARM_TABLE` | m | bp=[0.06, 0.2, 0.5, 0.85, 1.2] -> [0.75, 0.9, 1.15, 1.45, 1.8] | G9: rollout_trigger_i (stopping_controller.py:852) |
| 28 | `RECOVERY_GAIN_TABLE` | 1/s | bp=[0.06, 0.2, 0.5, 0.85, 1.2] -> [1, 0.85, 0.65, 0.45, 0.3] | G9: integrator growth gain (stopping_controller.py:853) |
| 28 | `RECOVERY_DECAY_TABLE` | 1/s | bp=[0.06, 0.2, 0.5, 0.85, 1.2] -> [0.3, 0.24, 0.18, 0.14, 0.1] | G9: integrator decay (stopping_controller.py:854) |
| 28 | `RECOVERY_CAP` | m/s^2 | 0.9 | G9: integrator clip (stopping_controller.py:856) |
| 28 | `RECOVERY_APPLY_GAIN_TABLE` | - | bp=[0.06, 0.2, 0.5, 0.85, 1.2] -> [0.22, 0.2, 0.17, 0.13, 0.1] | G9: k_apply, target -= recovery_i * k_apply(v) (stopping_controller.py:1344) |
| 28 | `RECOVERY_BRAKE_FLOOR_TABLE` | m/s^3 | bp=[0.06, 0.2, 0.5, 0.85, 1.2] -> [1.2, 1.6, 2.2, 2.9, 3.6] | G9: recovery brake_step floor x100 (stopping_controller.py:1346) |
| 28 | `RECOVERY_RELEASE_CAP_TABLE` | m/s^3 | bp=[0.06, 0.2, 0.5, 0.85, 1.2] -> [0.09, 0.12, 0.18, 0.26, 0.36] | G9: recovery release_step cap x100 (stopping_controller.py:1347) |
| 29 | `KINEMATIC_MIN_DECEL` | m/s^2 | 0.2 | G14: kinematic fallback min decel magnitude (stopping_controller.py:299-301) |
| 29 | `KINEMATIC_REMAINING_CLIP_M` | m | 3 | G14: kinematic remaining-distance clip (stopping_controller.py:301) |
| 29 | `EXPLICIT_REMAINING_CLIP_M` | m | 6 | G14: explicit-target remaining-distance clip (stopping_controller.py:303-306) |
| 30 | `EXPECTED_ACCEL_V_BP` | m/s | [0.01, 0.2, 0.5] | G3: STOPPING_V_BP (longcontrol.py:36) |
| 30 | `EXPECTED_ACCEL_MAX` | m/s^2 | [-0.01, -0.1, -0.3] | G3: STOPPING_ACCEL_MAX (longcontrol.py:37) |
| 30 | `EXPECTED_ACCEL_MIN` | m/s^2 | [-0.1, -0.5, -1] | G3: STOPPING_ACCEL_MIN (longcontrol.py:38) |
| 31 | `HOLD_GAP_M` | m | 2.75 | G10: LEAD_FOLLOW_MIN_HOLD_GAP_M (longcontrol.py:46) = STOPPED_LEAD_MIN_CONTROL_GAP_M (stop_target_helpers.py:14) |
| 31 | `TARGET_HOLD_GAP_M` | m | 3.75 | G10: LEAD_FOLLOW_TARGET_HOLD_GAP_M (longcontrol.py:47) |
| 31 | `FAR_CRAWL_GAP_M` | m | 5 | G11: FAR_STOPPED_LEAD_CRAWL_GAP_M (longcontrol.py:48) = STOPPED_LEAD_CONTROL_MAX_GAP_M (stop_target_helpers.py:15) |
| 31 | `LEAD_STOP_TARGET_M` | m | 4 | G11: LEAD_STOP_DISTANCE_TARGET (stop_target_helpers.py:16) |
| 32 | `FAR_LEAD_RELEASE_V_TABLE` | m/s | bp=[0, 0.2, 0.55] -> [0.65, 0.45, 0.28] | G11: far_stopped_lead_release (stopping_controller.py:685-692 == longcontrol.py:298-313) |
| 32 | `FAR_LEAD_RELEASE_GAP_M` | m | 5 | G11: far-stopped-lead release gap, lead_d_rel > 5.00 (stopping_controller.py:689) |
| 33 | `T_STOP_INTENT_HOLD_S` | s | 0.4 | new: single falling-edge hold for rolling dropouts; sized inside the legacy 0.35/0.8 s windows (longcontrol.py:441, :492) |
| 34 | `T_STOP_INTENT_HOLD_STANDSTILL_S` | s | 1.4 | new: no-target standstill dropout window; preserves should_hold_no_target_standstill_dropout 1.40 s (longcontrol.py:473) |
| 34 | `HOLD_ESCAPE_A_TARGET` | m/s^2 | 0.12 | legacy standstill-hold escape, a_target <= 0.12 (longcontrol.py:475) |
| 34 | `HOLD_ESCAPE_LAST_OUTPUT` | m/s^2 | -0.08 | legacy standstill-hold escape, last_output_accel > -0.08 releases (longcontrol.py:471) |
| 35 | `STOP_ENTRY_GATE_V_RANGE` | m/s | [0.35, 2.3] | G13: gate speed window (longcontrol.py:403) |
| 35 | `STOP_ENTRY_GATE_A_EGO_RANGE` | m/s^2 | [-1.05, -0.42] | G13: gate a_ego window (longcontrol.py:405) |
| 35 | `STOP_ENTRY_GATE_LAST_CMD_RANGE` | m/s^2 | [-0.88, -0.48] | G13: gate last_output_accel window (longcontrol.py:407) |
| 35 | `STOP_ENTRY_GATE_TARGET_FLOOR_TABLE` | m/s^2 | bp=[0.35, 0.6, 1, 1.5, 2.3] -> [-0.2, -0.28, -0.38, -0.5, -0.6] | G13: gate a_target floor (longcontrol.py:409) |
| 35 | `STOP_ENTRY_GATE_MIN_TARGET_DISTANCE_M` | m | 0.22 | G13: gate disabled below this explicit-target distance (longcontrol.py:412) |
| 35 | `STOP_ENTRY_CAP_SPEED_TABLE` | m/s^2 | bp=[0.35, 0.6, 1, 1.5, 2.3] -> [-0.44, -0.48, -0.56, -0.64, -0.74] | G13: handoff speed cap (longcontrol.py:418) |
| 35 | `STOP_ENTRY_CAP_DISTANCE_TABLE` | m/s^2 | bp=[0.22, 0.4, 0.75, 1.2, 2] -> [-0.72, -0.66, -0.58, -0.52, -0.46] | G13: handoff distance cap, min() with speed cap (longcontrol.py:421); bp axis is distance m |
| 35 | `STOP_ENTRY_OUTPUT_CAP_TABLE` | m/s^2 | bp=[0, 0.12, 0.25, 0.4, 0.8] -> [0, -0.005, -0.015, -0.025, -0.05] | G13: stop_entry_output_cap (stopping_controller.py:2555-2559, :2558) |
| 36 | `DELAY_RELIEF_TRIGGER_TABLE` | m/s^2 | bp=[0, 0.55, 1.2] -> [0.006, 0.014, 0.02] | G2: relief_trigger (stopping_controller.py:325) |
| 36 | `DELAY_RELIEF_SCALE_TABLE` | m/s^2 | bp=[0, 0.55, 1.2] -> [0.02, 0.04, 0.06] | G2: relief_scale (stopping_controller.py:326) |
| 36 | `DELAY_RELIEF_CLIP_MAX` | m/s^2 | 0.35 | G2: release_relief clipped to [0, 0.35] (stopping_controller.py:324) |
| 36 | `DELAY_RELEASE_CAP_TABLE` | m/s^3 | bp=[0, 0.2, 0.55, 1.2] -> [0.04, 0.08, 0.15, 0.24] | G2: delay_release_cap x100 (stopping_controller.py:1726) |
| 36 | `DELAY_RELEASE_BIAS_TABLE` | m/s^2 | bp=[0, 0.2, 0.55, 1.2] -> [0.05, 0.07, 0.1, 0.11] | G2: guard target bias (stopping_controller.py:1728) |
| 37 | `J_END_STOP_RELEASE_TABLE` | m/s^3 | bp=[0, 0.6] -> [0.9, 0.45] | G7: end-stop fast release, release_step interp(v,[0,0.60],[0.009,0.0045]) x100 (stopping_controller.py:2382-2392) |
| 37 | `J_END_STOP_RELEASE_SUPPRESS_V` | m/s | 0.2 | G7: suppress_fast_end_stop_release speed gate (stopping_controller.py:2384) |
| 38 | `J_ARREST_TABLE` | m/s^3 | bp=[0, 0.08] -> [4, 2.2] | G8: arrest brake_step interp(v,[0,0.08],[0.040,0.022]) x100 (stopping_controller.py:2047) |
| 38 | `ARREST_V_MAX` | m/s | 0.08 | spec 5.5.2: arrest arms when the push estimator fires below 0.08 m/s with rising v |
| 38 | `ARREST_EXIT_FALLING_T_S` | s | 0.15 | spec 5.5.2: arrest exits after v falling for >= 0.15 s or push cleared |
| 39 | `OVERBRAKE_RELEASE_FLOOR_TABLE` | m/s^3 | bp=[0, 0.1, 0.3, 0.7, 1.2] -> [1, 1.15, 1.35, 1.6, 1.8] | G4: lock_overbrake_relief release floor x100 (stopping_controller.py:1766) |
| 39 | `OVERBRAKE_TRIGGER_MARGIN` | m/s^2 | 0.12 | G4: lock_overbrake_relief trigger, a_ego < min_expected - 0.12 (stopping_controller.py:827) |
| 40 | `HOLD_ACQ_SOFTEN_V_MAX` | m/s | 0.05 | new: hold-acquisition soften stationary band (stopping_controller.py hold_acquisition_soften); driveway route 00001702--dcdc5c3eea--0 |
| 40 | `HOLD_ACQ_SOFTEN_A_EGO_MAX` | m/s^2 | 0.3 | new: hold-acquisition soften |a_ego| stability band; driveway route 00001702--dcdc5c3eea--0 |
| 40 | `HOLD_ACQ_SOFTEN_DISTURBANCE_MAX` | m/s^2 | 0.04 | new: hold-acquisition soften live-disturbance gate = DIST_PUSH_THRESH_LOW (row 20); driveway route 00001702--dcdc5c3eea--0 |
| 40 | `HOLD_ACQ_SOFTEN_CMD_MAX` | m/s^2 | -0.55 | new: hold-acquisition deep-ramp gate, last_output_accel < -0.55; hill-hold blind-window review 2026-06-10 (felt slam was -0.5..-0.78 -> -1.05) |
| 40 | `J_HOLD_ACQUISITION` | m/s^3 | 1 | new: stationary-stable hold-acquisition deepening rate cap, 0.010/frame x100; driveway route 00001702--dcdc5c3eea--0 |
| 40 | `J_HOLD_ACQUISITION_ARREST` | m/s^3 | 2 | new: hold-acquisition rate floor while rebound_arrest_active, 0.020/frame x100; hill-hold review 2026-06-10 (10%-grade rollback bound) |
