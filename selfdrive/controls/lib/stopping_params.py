"""Frozen stopping-stack parameter registry (stopping redesign spec, section 3).

Every parameter has one definition site (this dataclass), a physical unit, and provenance
traceable to one of the 17 preserve groups (spec section 3.1) of the legacy forest controller
at base commit 3be25f5240. Initial values reproduce the forest envelope, not improve it.

`docs/stopping/parameters.md` is GENERATED from this module:
  python3 -m openpilot.selfdrive.controls.lib.stopping_params
and `test_stopping_params.py` asserts the committed doc matches the dataclass.

Conventions (spec-wide): accel m/s^2 with negative = braking; jerk m/s^3 as positive
magnitudes split into brake (deepening) / release (toward zero); distances m positive;
speeds m/s. Tables are (bp, v) tuple pairs consumed via np.interp unless the provenance
note says STEP. Legacy per-frame step values are converted to physical rates by x100
(the forest tuned steps for the 100 Hz loop); all consumers re-derive per-frame limits
as rate * dt -- no dt_scale anywhere.
"""

from __future__ import annotations

from dataclasses import dataclass, field, fields

Table = tuple[tuple[float, ...], tuple[float, ...]]


def _p(value, *, row: int, unit: str, provenance: str, kind: str = "scalar"):
  return field(default=value, metadata={"row": row, "unit": unit, "provenance": provenance, "kind": kind})


@dataclass(frozen=True)
class StoppingParams:
  # --- G2: actuator delay + command history -------------------------------------------------
  ACTUATOR_DELAY_S: float = _p(0.05, row=1, unit="s",
                               provenance="G2: controller delay compensation, delay_frames = round(0.05/dt) (stopping_controller.py:816)")
  CMD_HISTORY_LEN: int = _p(48, row=2, unit="frames",
                            provenance="G2: COMMAND_HISTORY_LEN (stopping_controller.py:12)", kind="int")

  # --- G1: identified plant model reference (archived 20260514 fit, 862 rows) ----------------
  # AR pole 0.8715 @ 0.1 s (tau ~ 0.727 s); cmd gain 0.372 - 0.451*low_speed. The authority
  # collapse / gain_dc sign inversion near v ~ 0.21 m/s lives in stopping_plant.PlantModel.gain_dc.
  PLANT_MODEL_REF: tuple[tuple[str, float], ...] = _p(
    (
      ("intercept", -0.003240927224571002),
      ("a_ego_prev", 0.8715303653619791),
      ("accel_cmd_delayed", 0.37169541723475924),
      ("v_ego", 0.013537141414051946),
      ("relief", 0.28700393800740076),
      ("low_speed", -0.014521878402947764),
      ("cmd_x_low_speed", -0.45138720666502313),
    ),
    row=3, unit="-",
    provenance="G1: archived 20260514 fit, SHADOW_MODEL_COEFFICIENTS (stopping_shadow.py:25-33); archived in docs/stopping/archive/; refit per spec 7.5",
    kind="coef")
  PLANT_MODEL_DT_FIT_S: float = _p(0.10, row=3, unit="s",
                                   provenance="G1: SHADOW_MODEL_DT_S (stopping_shadow.py:22); dt the coefficients were fitted at")
  PLANT_MODEL_DELAY_S: float = _p(0.10, row=3, unit="s",
                                  provenance="G1: archived 20260514 fit dead time, delay_frames=1 @ 0.1 s (docs/stopping/archive/)")
  LOW_SPEED_AUTHORITY_REF: float = _p(1.20, row=4, unit="m/s",
                                      provenance="G1: SHADOW_MODEL_LOW_SPEED_REF (stopping_shadow.py:34)")
  RELIEF_CMD_THRESHOLD: float = _p(-0.25, row=5, unit="m/s^2",
                                   provenance="G1: SHADOW_MODEL_RELIEF_CMD_THRESHOLD (stopping_shadow.py:35)")

  # --- G6: phase bounds + hold/near-hold force levels ----------------------------------------
  V_NEAR_HOLD: float = _p(0.85, row=6, unit="m/s",
                          provenance="G6: NEAR_HOLD phase bound, near_hold_speed_mps (stopping_controller.py:16-18, :37)")
  V_SETTLE: float = _p(0.06, row=7, unit="m/s",
                       provenance="G6: HOLD phase bound, hold_speed_mps + settle group (stopping_controller.py:34-42)")
  V_STANDSTILL_SETTLED: float = _p(0.02, row=8, unit="m/s",
                                   provenance="G6: standstill_settle_speed_mps (stopping_controller.py:40); below wheel-speed standstill 0.104 m/s")
  A_HOLD_TABLE: Table = _p(((0.00, 0.02, 0.06), (-0.16, -0.15, -0.13)), row=9, unit="m/s^2",
                           provenance="G6: HOLD phase hold_target (stopping_controller.py:378)", kind="table")
  A_HOLD_RELAXED_TABLE: Table = _p(((0.00, 0.02), (-0.12, -0.10)), row=10, unit="m/s^2",
                                   provenance="G6: standstill_relax target (stopping_controller.py:2450-2465)", kind="table")
  T_HOLD_RELAX_S: float = _p(0.8, row=11, unit="s",
                             provenance="G6: standstill_relax_time_s (stopping_controller.py:42)")
  A_NEAR_HOLD_TABLE: Table = _p(((0.06, 0.15, 0.30, 0.55, 0.85), (-0.14, -0.17, -0.20, -0.18, -0.15)), row=12, unit="m/s^2",
                                provenance="G6: NEAR_HOLD hold_target (stopping_controller.py:371) -- TERMINAL envelope", kind="table")

  # --- G7: end-stop brake cap ----------------------------------------------------------------
  # Full 0-0.60 m/s domain is the binding terminal ceiling -- never cap only below 0.15 m/s.
  # Binds the QUIESCENT path only; push/arrest may deepen past it (spec 5.3 scope).
  A_END_STOP_TABLE: Table = _p(((0.00, 0.10, 0.15, 0.25, 0.60), (-0.255, -0.255, -0.30, -0.42, -0.68)), row=13, unit="m/s^2",
                               provenance="G7: end_stop_brake_cap (stopping_controller.py:2055); nominal soft cap -0.275", kind="table")

  A_APPROACH_FLOOR_TABLE: Table = _p(((0.90, 1.20, 1.60), (-0.40, -0.45, -0.50)), row=14, unit="m/s^2",
                                     provenance="G6: APPROACH floor (stopping_controller.py:359-369, :363)", kind="table")
  A_DESIRED_LOWSPEED_TABLE: Table = _p(((0.06, 0.20, 0.50, 0.85, 1.20), (-0.26, -0.32, -0.42, -0.52, -0.60)), row=15, unit="m/s^2",
                                       provenance="G9: desired_low_speed_accel (stopping_controller.py:849-860, :850)", kind="table")

  # --- G6/G15: jerk budgets ------------------------------------------------------------------
  J_BRAKE_TABLE: Table = _p(((0.06, 0.55, 0.85, 1.20), (0.60, 0.80, 0.90, 0.70)), row=16, unit="m/s^3",
                            provenance="G6/G15: NEAR_HOLD/APPROACH brake_step x100 (stopping_controller.py:368, :375)", kind="table")
  # 0.06-0.55 segment matches the legacy NEAR_HOLD release curve [0.08->0.22] (red-team F35);
  # 0.14 @ v=0 is the HOLD-phase value (SETTLE/HOLD use J_SETTLE_RELEASE anyway).
  J_RELEASE_TABLE: Table = _p(((0.00, 0.06, 0.20, 0.55, 0.85), (0.14, 0.08, 0.12, 0.22, 0.30)), row=17, unit="m/s^3",
                              provenance="G6: HOLD/NEAR_HOLD release_step x100 (stopping_controller.py:376, :383)", kind="table")
  J_RELEASE_LOCKED_TABLE: Table = _p(((0.00, 0.20, 0.50, 1.20), (0.10, 0.15, 0.30, 0.60)), row=18, unit="m/s^3",
                                     provenance="G4: locked release cap x100 (stopping_controller.py:1774). Retirement: spec 3.2 row 4", kind="table")
  J_SETTLE_RELEASE: float = _p(0.9, row=19, unit="m/s^3",
                               provenance="G7: end-stop fast release 0.009/frame at v=0 (stopping_controller.py:2383-2392)")

  # --- G4: disturbance / release-inhibit thresholds ------------------------------------------
  # STEP semantics (not interp): threshold = LOW below the split speed, HIGH at or above it.
  DIST_PUSH_THRESH_V_SPLIT: float = _p(0.08, row=20, unit="m/s",
                                       provenance="G4: disturbance_threshold speed split (stopping_controller.py:99); STEP, not interp")
  DIST_PUSH_THRESH_LOW: float = _p(0.04, row=20, unit="m/s^2",
                                   provenance="G4: disturbance_threshold below split (stopping_controller.py:99)")
  DIST_PUSH_THRESH_HIGH: float = _p(0.03, row=20, unit="m/s^2",
                                    provenance="G4: disturbance_threshold at/above split (stopping_controller.py:99)")
  DIST_PUSH_V_MIN: float = _p(0.002, row=20, unit="m/s",
                              provenance="G4: push detection speed gate, v_ego > 0.002 (stopping_controller.py:101)")
  DIST_PUSH_V_MAX: float = _p(1.2, row=20, unit="m/s",
                              provenance="G4: push detection speed gate, v_ego < 1.2 (stopping_controller.py:102)")
  DIST_PUSH_MIN_BRAKE: float = _p(-0.05, row=20, unit="m/s^2",
                                  provenance="G4: push detection braking gate, last_output_accel < -0.05 (stopping_controller.py:103)")
  # KILL SWITCH: 0.0 bypasses the LPF to legacy single-frame trigger semantics (spec 5.5.2).
  # Must pass the estimator-equivalence replay (WP6 artifact) before the similarity gate runs.
  # Set to 0.0 (legacy bypass) for the Commit D gate: tau=0.30 fails the estimator-equivalence
  # artifact on the real event store (13% onset agreement vs 90% floor; only the bypass passes)
  # -- class-B move toward the documented G4 preserve-group source (single-frame semantics).
  DIST_LPF_TAU_S: float = _p(0.0, row=21, unit="s",
                             provenance="new estimator filter (spec 5.5.2); 0.0 = KILL SWITCH bypass to legacy single-frame semantics")
  T_RELEASE_INHIBIT_TABLE: Table = _p(((0.00, 0.20, 0.60, 1.20), (1.10, 0.95, 0.70, 0.50)), row=22, unit="s",
                                      provenance="G4: lock frames /100 (stopping_controller.py:107)", kind="table")
  A_DISTURBANCE_FLOOR_TABLE: Table = _p(((0.00, 0.12, 0.25, 0.50, 1.20), (-0.34, -0.31, -0.26, -0.18, -0.11)), row=23, unit="m/s^2",
                                        provenance="G4: lock_floor (stopping_controller.py:1764-1776, :1775) -- also the hill-hold deepening bound",
                                        kind="table")

  # --- G5: clutch/TC push relief -------------------------------------------------------------
  A_PUSH_RELIEF_CAP_TABLE: Table = _p(((0.00, 0.60, 1.20, 1.80, 2.50), (-0.30, -0.34, -0.38, -0.42, -0.46)), row=24, unit="m/s^2",
                                      provenance="G5: clutch/TC push relief target base (stopping_controller.py:1349-1363, :1356)", kind="table")
  # Below PUSH_RELIEF_V_MIN the disturbance-floor/arrest path owns the response exclusively
  # (spec 5.5.2, red-team F31 -- relief must never fight hill-hold deepening at standstill).
  PUSH_RELIEF_V_MIN: float = _p(0.12, row=24, unit="m/s",
                                provenance="G5: legacy outer speed gate, 0.12 < v_ego (stopping_controller.py:829)")
  PUSH_RELIEF_V_MAX: float = _p(2.5, row=24, unit="m/s",
                                provenance="G5: legacy outer speed gate, v_ego < 2.5 (stopping_controller.py:829)")

  # --- G8: rebound arrest authority ----------------------------------------------------------
  A_ARREST_MAX_TABLE: Table = _p(((0.00, 0.03, 0.06, 0.08), (-1.40, -1.15, -0.85, -0.56)), row=25, unit="m/s^2",
                                 provenance="G8: rebound arrest cap (stopping_controller.py:2021) -- absolute deepening clamp in SETTLE/HOLD",
                                 kind="table")

  # --- G12/G9: rollout budget + odometer -----------------------------------------------------
  ROLLOUT_BUDGET_NO_TARGET_M: float = _p(2.0, row=26, unit="m",
                                         provenance="G12: shadow score rollout knee, no explicit target (stopping_shadow.py:393-409, :397)")
  ROLLOUT_BUDGET_EXPLICIT_M: float = _p(1.25, row=26, unit="m",
                                        provenance="G12: shadow score rollout knee, explicit target (stopping_shadow.py:398-399)")
  ROLLOUT_DECAY_MPS: float = _p(0.35, row=27, unit="m/s",
                                provenance="G9: standstill_rollout_decay_mps (stopping_controller.py:39)")

  # --- G9: recovery integrator (verbatim; retirement: spec 3.2 row 5) ------------------------
  RECOVERY_ARM_TABLE: Table = _p(((0.06, 0.20, 0.50, 0.85, 1.20), (0.75, 0.90, 1.15, 1.45, 1.80)), row=28, unit="m",
                                 provenance="G9: rollout_trigger_i (stopping_controller.py:852)", kind="table")
  RECOVERY_GAIN_TABLE: Table = _p(((0.06, 0.20, 0.50, 0.85, 1.20), (1.00, 0.85, 0.65, 0.45, 0.30)), row=28, unit="1/s",
                                  provenance="G9: integrator growth gain (stopping_controller.py:853)", kind="table")
  RECOVERY_DECAY_TABLE: Table = _p(((0.06, 0.20, 0.50, 0.85, 1.20), (0.30, 0.24, 0.18, 0.14, 0.10)), row=28, unit="1/s",
                                   provenance="G9: integrator decay (stopping_controller.py:854)", kind="table")
  RECOVERY_CAP: float = _p(0.90, row=28, unit="m/s^2",
                           provenance="G9: integrator clip (stopping_controller.py:856)")
  RECOVERY_APPLY_GAIN_TABLE: Table = _p(((0.06, 0.20, 0.50, 0.85, 1.20), (0.22, 0.20, 0.17, 0.13, 0.10)), row=28, unit="-",
                                        provenance="G9: k_apply, target -= recovery_i * k_apply(v) (stopping_controller.py:1344)", kind="table")
  RECOVERY_BRAKE_FLOOR_TABLE: Table = _p(((0.06, 0.20, 0.50, 0.85, 1.20), (1.2, 1.6, 2.2, 2.9, 3.6)), row=28, unit="m/s^3",
                                         provenance="G9: recovery brake_step floor x100 (stopping_controller.py:1346)", kind="table")
  RECOVERY_RELEASE_CAP_TABLE: Table = _p(((0.06, 0.20, 0.50, 0.85, 1.20), (0.09, 0.12, 0.18, 0.26, 0.36)), row=28, unit="m/s^3",
                                         provenance="G9: recovery release_step cap x100 (stopping_controller.py:1347)", kind="table")

  # --- G14: remaining-distance kinematics ----------------------------------------------------
  KINEMATIC_MIN_DECEL: float = _p(0.20, row=29, unit="m/s^2",
                                  provenance="G14: kinematic fallback min decel magnitude (stopping_controller.py:299-301)")
  KINEMATIC_REMAINING_CLIP_M: float = _p(3.0, row=29, unit="m",
                                         provenance="G14: kinematic remaining-distance clip (stopping_controller.py:301)")
  EXPLICIT_REMAINING_CLIP_M: float = _p(6.0, row=29, unit="m",
                                        provenance="G14: explicit-target remaining-distance clip (stopping_controller.py:303-306)")

  # --- G3: expected plant response envelope (CP.stoppingVbp never populated -- hard-coded) ---
  EXPECTED_ACCEL_V_BP: tuple[float, ...] = _p((0.01, 0.2, 0.5), row=30, unit="m/s",
                                              provenance="G3: STOPPING_V_BP (longcontrol.py:36)", kind="list")
  EXPECTED_ACCEL_MAX: tuple[float, ...] = _p((-0.01, -0.1, -0.3), row=30, unit="m/s^2",
                                             provenance="G3: STOPPING_ACCEL_MAX (longcontrol.py:37)", kind="list")
  EXPECTED_ACCEL_MIN: tuple[float, ...] = _p((-0.1, -0.5, -1.0), row=30, unit="m/s^2",
                                             provenance="G3: STOPPING_ACCEL_MIN (longcontrol.py:38)", kind="list")

  # --- G10/G11: hold gaps + stopped-lead target ----------------------------------------------
  HOLD_GAP_M: float = _p(2.75, row=31, unit="m",
                         provenance="G10: LEAD_FOLLOW_MIN_HOLD_GAP_M (longcontrol.py:46) = STOPPED_LEAD_MIN_CONTROL_GAP_M (stop_target_helpers.py:14)")
  TARGET_HOLD_GAP_M: float = _p(3.75, row=31, unit="m",
                                provenance="G10: LEAD_FOLLOW_TARGET_HOLD_GAP_M (longcontrol.py:47)")
  FAR_CRAWL_GAP_M: float = _p(5.0, row=31, unit="m",
                              provenance="G11: FAR_STOPPED_LEAD_CRAWL_GAP_M (longcontrol.py:48) = STOPPED_LEAD_CONTROL_MAX_GAP_M (stop_target_helpers.py:15)")
  LEAD_STOP_TARGET_M: float = _p(4.0, row=31, unit="m",
                                 provenance="G11: LEAD_STOP_DISTANCE_TARGET (stop_target_helpers.py:16)")

  # --- G11: far-stopped-lead release ----------------------------------------------------------
  # bp axis is v_ego; values are the max lead_v that still counts as stopped, at gap > FAR_LEAD_RELEASE_GAP_M.
  FAR_LEAD_RELEASE_V_TABLE: Table = _p(((0.00, 0.20, 0.55), (0.65, 0.45, 0.28)), row=32, unit="m/s",
                                       provenance="G11: far_stopped_lead_release (stopping_controller.py:685-692 == longcontrol.py:298-313)",
                                       kind="table")
  FAR_LEAD_RELEASE_GAP_M: float = _p(5.0, row=32, unit="m",
                                     provenance="G11: far-stopped-lead release gap, lead_d_rel > 5.00 (stopping_controller.py:689)")

  # --- new: arbiter dropout-hold windows (spec 5.2.5) -----------------------------------------
  T_STOP_INTENT_HOLD_S: float = _p(
    0.4, row=33, unit="s",
    provenance="new: single falling-edge hold for rolling dropouts; sized inside the legacy 0.35/0.8 s windows (longcontrol.py:441, :492)")
  # Applies only when NO stopped lead is within FAR_CRAWL_GAP_M -- with a close stopped lead the
  # unbounded STOPPED_LEAD_LATCH owns the hold, not this timer (spec 5.2.5).
  T_STOP_INTENT_HOLD_STANDSTILL_S: float = _p(
    1.4, row=34, unit="s",
    provenance="new: no-target standstill dropout window; preserves should_hold_no_target_standstill_dropout 1.40 s (longcontrol.py:473)")
  HOLD_ESCAPE_A_TARGET: float = _p(0.12, row=34, unit="m/s^2",
                                   provenance="legacy standstill-hold escape, a_target <= 0.12 (longcontrol.py:475)")
  HOLD_ESCAPE_LAST_OUTPUT: float = _p(-0.08, row=34, unit="m/s^2",
                                      provenance="legacy standstill-hold escape, last_output_accel > -0.08 releases (longcontrol.py:471)")

  # --- G13: stop-entry handoff soften (verbatim; retirement: spec 3.2 row 2) ------------------
  STOP_ENTRY_GATE_V_RANGE: tuple[float, float] = _p((0.35, 2.30), row=35, unit="m/s",
                                                    provenance="G13: gate speed window (longcontrol.py:403)", kind="range")
  STOP_ENTRY_GATE_A_EGO_RANGE: tuple[float, float] = _p((-1.05, -0.42), row=35, unit="m/s^2",
                                                        provenance="G13: gate a_ego window (longcontrol.py:405)", kind="range")
  STOP_ENTRY_GATE_LAST_CMD_RANGE: tuple[float, float] = _p((-0.88, -0.48), row=35, unit="m/s^2",
                                                           provenance="G13: gate last_output_accel window (longcontrol.py:407)", kind="range")
  STOP_ENTRY_GATE_TARGET_FLOOR_TABLE: Table = _p(((0.35, 0.60, 1.00, 1.50, 2.30), (-0.20, -0.28, -0.38, -0.50, -0.60)), row=35, unit="m/s^2",
                                                 provenance="G13: gate a_target floor (longcontrol.py:409)", kind="table")
  STOP_ENTRY_GATE_MIN_TARGET_DISTANCE_M: float = _p(0.22, row=35, unit="m",
                                                    provenance="G13: gate disabled below this explicit-target distance (longcontrol.py:412)")
  STOP_ENTRY_CAP_SPEED_TABLE: Table = _p(((0.35, 0.60, 1.00, 1.50, 2.30), (-0.44, -0.48, -0.56, -0.64, -0.74)), row=35, unit="m/s^2",
                                         provenance="G13: handoff speed cap (longcontrol.py:418)", kind="table")
  STOP_ENTRY_CAP_DISTANCE_TABLE: Table = _p(((0.22, 0.40, 0.75, 1.20, 2.00), (-0.72, -0.66, -0.58, -0.52, -0.46)), row=35, unit="m/s^2",
                                            provenance="G13: handoff distance cap, min() with speed cap (longcontrol.py:421); bp axis is distance m",
                                            kind="table")
  # Fresh-entry output cap (spec 5.5.5 step 4 exception): allows up to 0.00 at v = 0 on a fresh
  # stop entry while last_output_accel > -0.02, instead of the normal -0.05 authority ceiling.
  STOP_ENTRY_OUTPUT_CAP_TABLE: Table = _p(((0.00, 0.12, 0.25, 0.40, 0.80), (0.00, -0.005, -0.015, -0.025, -0.05)), row=35, unit="m/s^2",
                                          provenance="G13: stop_entry_output_cap (stopping_controller.py:2555-2559, :2558)", kind="table")

  # --- G2: delay-release guard (verbatim; retirement: spec 3.2 row 3) -------------------------
  DELAY_RELIEF_TRIGGER_TABLE: Table = _p(((0.00, 0.55, 1.20), (0.006, 0.014, 0.020)), row=36, unit="m/s^2",
                                         provenance="G2: relief_trigger (stopping_controller.py:325)", kind="table")
  DELAY_RELIEF_SCALE_TABLE: Table = _p(((0.00, 0.55, 1.20), (0.020, 0.040, 0.060)), row=36, unit="m/s^2",
                                       provenance="G2: relief_scale (stopping_controller.py:326)", kind="table")
  DELAY_RELIEF_CLIP_MAX: float = _p(0.35, row=36, unit="m/s^2",
                                    provenance="G2: release_relief clipped to [0, 0.35] (stopping_controller.py:324)")
  DELAY_RELEASE_CAP_TABLE: Table = _p(((0.00, 0.20, 0.55, 1.20), (0.04, 0.08, 0.15, 0.24)), row=36, unit="m/s^3",
                                      provenance="G2: delay_release_cap x100 (stopping_controller.py:1726)", kind="table")
  DELAY_RELEASE_BIAS_TABLE: Table = _p(((0.00, 0.20, 0.55, 1.20), (0.05, 0.07, 0.10, 0.11)), row=36, unit="m/s^2",
                                       provenance="G2: guard target bias (stopping_controller.py:1728)", kind="table")

  # --- G7: end-stop ceiling-bound fast release (red-team F30) ---------------------------------
  # Release budget whenever the end-stop ceiling BINDS in TERMINAL (not just SETTLE), so an
  # inherited deep brake is shed BEFORE wheel-stop. Suppressed below J_END_STOP_RELEASE_SUPPRESS_V
  # while release-inhibited or arrest-active (mirrors suppress_fast_end_stop_release).
  J_END_STOP_RELEASE_TABLE: Table = _p(
    ((0.00, 0.60), (0.90, 0.45)), row=37, unit="m/s^3",
    provenance="G7: end-stop fast release, release_step interp(v,[0,0.60],[0.009,0.0045]) x100 (stopping_controller.py:2382-2392)", kind="table")
  J_END_STOP_RELEASE_SUPPRESS_V: float = _p(0.20, row=37, unit="m/s",
                                            provenance="G7: suppress_fast_end_stop_release speed gate (stopping_controller.py:2384)")

  # --- G8: arrest deepening rate (red-team F27) -----------------------------------------------
  # Deepening rate when the arrest path fires (push below ARREST_V_MAX with rising v); without it
  # the 0.6-0.9 m/s^3 J_BRAKE budget is 4-6x too slow to catch an HEV creep surge near 0 m/s.
  J_ARREST_TABLE: Table = _p(((0.00, 0.08), (4.0, 2.2)), row=38, unit="m/s^3",
                             provenance="G8: arrest brake_step interp(v,[0,0.08],[0.040,0.022]) x100 (stopping_controller.py:2047)", kind="table")
  ARREST_V_MAX: float = _p(0.08, row=38, unit="m/s",
                           provenance="spec 5.5.2: arrest arms when the push estimator fires below 0.08 m/s with rising v")
  ARREST_EXIT_FALLING_T_S: float = _p(0.15, row=38, unit="s",
                                      provenance="spec 5.5.2: arrest exits after v falling for >= 0.15 s or push cleared")

  # --- G4: overbrake release floor (red-team F35) ---------------------------------------------
  OVERBRAKE_RELEASE_FLOOR_TABLE: Table = _p(((0.00, 0.10, 0.30, 0.70, 1.20), (1.00, 1.15, 1.35, 1.60, 1.80)), row=39, unit="m/s^3",
                                            provenance="G4: lock_overbrake_relief release floor x100 (stopping_controller.py:1766)", kind="table")
  OVERBRAKE_TRIGGER_MARGIN: float = _p(0.12, row=39, unit="m/s^2",
                                       provenance="G4: lock_overbrake_relief trigger, a_ego < min_expected - 0.12 (stopping_controller.py:827)")

  # --- new: stationary-stable hold-acquisition soften (driveway route 00001702--dcdc5c3eea--0) -
  # 2026-06-10 engage-at-standstill diagnosis: the legacy arrest lane ramped the command to the
  # -1.05 hold at 3.0-3.2 m/s^3 while v_ego < 0.045 with no live push. When ALL gates below hold
  # (genuinely stationary and stable), DEEP-ramp deepening toward the full hold depth is
  # rate-capped at J_HOLD_ACQUISITION. SENSING CAVEAT (hill-hold blind-window review,
  # 2026-06-10): the v/a_ego/disturbance gates read quiet during the sensor-blind window of a
  # fresh grade re-roll (wheel-speed deadband ~0.08 m/s + ~0.1 s transport + accel-filter lag),
  # so the gates alone cannot protect the hill-hold catch. Two safeguards bound worst-case
  # 10%-grade rollback to within ~4 cm of legacy (command-domain sim, arrest latched at -0.23,
  # actuator tau 0.2 s): the soften arms only once the command is already below
  # HOLD_ACQ_SOFTEN_CMD_MAX (the shallow catch keeps the full J_ARREST rate), and while the
  # rebound arrest is latched the rate is floored at J_HOLD_ACQUISITION_ARREST. V2 audit (this
  # change): the tracker already satisfies the two-regime split -- quiescent SETTLE/HOLD
  # deepening runs at J_BRAKE_TABLE (0.60 m/s^3 at v <= 0.06, below this ceiling) and J_ARREST
  # (4.0 m/s^3) is reachable only while a live push signature holds arrest_active (never
  # softened) -- so no V2 value change is needed; the params are recorded here so the regime
  # definition has one definition site.
  HOLD_ACQ_SOFTEN_V_MAX: float = _p(
    0.05, row=40, unit="m/s",
    provenance="new: hold-acquisition soften stationary band (stopping_controller.py hold_acquisition_soften); driveway route 00001702--dcdc5c3eea--0")
  HOLD_ACQ_SOFTEN_A_EGO_MAX: float = _p(
    0.30, row=40, unit="m/s^2",
    provenance="new: hold-acquisition soften |a_ego| stability band; driveway route 00001702--dcdc5c3eea--0")
  HOLD_ACQ_SOFTEN_DISTURBANCE_MAX: float = _p(
    0.04, row=40, unit="m/s^2",
    provenance="new: hold-acquisition soften live-disturbance gate = DIST_PUSH_THRESH_LOW (row 20); driveway route 00001702--dcdc5c3eea--0")
  HOLD_ACQ_SOFTEN_CMD_MAX: float = _p(
    -0.55, row=40, unit="m/s^2",
    provenance="new: hold-acquisition deep-ramp gate, last_output_accel < -0.55; hill-hold blind-window review 2026-06-10 (felt slam was -0.5..-0.78 -> -1.05)")
  J_HOLD_ACQUISITION: float = _p(
    1.0, row=40, unit="m/s^3",
    provenance="new: stationary-stable hold-acquisition deepening rate cap, 0.010/frame x100; driveway route 00001702--dcdc5c3eea--0")
  J_HOLD_ACQUISITION_ARREST: float = _p(
    2.0, row=40, unit="m/s^3",
    provenance="new: hold-acquisition rate floor while rebound_arrest_active, 0.020/frame x100; hill-hold review 2026-06-10 (10%-grade rollback bound)")

  # --- new: cranked comfort requirement (2026-06-13 user-felt-forces iteration) ----------------
  # P1 gentle-approach decel cap: while the lead gap is still comfortable (> APPROACH_DECEL_CAP_GAP_FLOOR_M)
  # commanded decel stays <= APPROACH_DECEL_CAP_MPS2 UNLESS kinematically necessary (closing^2 /
  # (2*max(gap-floor, eps)) > cap), the SAME exemption the eval's harsh-classifier uses so the
  # controller and gate agree. Implemented in longcontrol.py:stopping_phase_approach_decel_cap
  # (legacy active path) and mirrored here so the V2 facade inherits the identical physics. P2
  # terminal settle jerk cap: bounds the commanded deepening rate across the terminal settle band
  # to J_TERMINAL_SETTLE -- the V2 tracker's SETTLE/HOLD deepening reads this same budget.
  APPROACH_DECEL_CAP_MPS2: float = _p(
    0.5, row=41, unit="m/s^2",
    provenance="new: cranked P1 gentle-approach decel cap (longcontrol.py APPROACH_DECEL_CAP_MPS2); user stated 0.5 m/s^2 while lead gap > 2 m")
  APPROACH_DECEL_CAP_GAP_FLOOR_M: float = _p(
    2.0, row=41, unit="m",
    provenance="new: cranked P1 comfortable-gap floor (longcontrol.py APPROACH_DECEL_CAP_GAP_FLOOR_M = scoring_config.cranked.approach_gap_floor_m)")
  APPROACH_DECEL_CAP_V_EGO_MIN: float = _p(
    0.30, row=41, unit="m/s",
    provenance="new: cranked P1 lower speed gate (longcontrol.py APPROACH_DECEL_CAP_V_EGO_MIN); below this the terminal settle owns the command")
  APPROACH_DECEL_CAP_RELEASE_MARGIN: float = _p(
    0.18, row=41, unit="m/s^2",
    provenance="new: cranked P1 kinematic-release slack (longcontrol.py APPROACH_DECEL_CAP_RELEASE_MARGIN); > eval 0.12 so controller releases first")
  J_TERMINAL_SETTLE: float = _p(
    1.5, row=41, unit="m/s^3",
    provenance="new: cranked P2 commanded terminal-settle deepening-rate cap (stopping_controller.py J_TERMINAL_SETTLE), 0.015/frame x100; iteration knob")
  TERMINAL_SETTLE_V_MAX: float = _p(
    0.20, row=41, unit="m/s",
    provenance="new: cranked P2 terminal settle band (stopping_controller.py TERMINAL_SETTLE_V_MAX); above the 0.05 hold-acquisition band")


STOPPING_PARAMS = StoppingParams()


def _format_value(value, kind: str) -> str:
  if kind == "coef":
    return ", ".join(f"{name}={coef:.8g}" for name, coef in value)
  if kind == "table":
    bp, v = value
    return f"bp=[{', '.join(f'{x:g}' for x in bp)}] -> [{', '.join(f'{x:g}' for x in v)}]"
  if kind in ("list", "range"):
    return f"[{', '.join(f'{x:g}' for x in value)}]"
  if kind == "int":
    return str(value)
  return f"{value:g}"


def render_parameters_doc(params: StoppingParams = STOPPING_PARAMS) -> str:
  lines = [
    "# Stopping-stack parameters",
    "",
    "GENERATED FILE -- do not edit by hand. Source of truth:",
    "`selfdrive/controls/lib/stopping_params.py` (regenerate with",
    "`python3 -m openpilot.selfdrive.controls.lib.stopping_params`).",
    "`test_stopping_params.py` asserts this file matches the dataclass.",
    "",
    "Row numbers are the spec section-3 parameter table rows; multiple fields can share a row",
    "(multi-table parameter groups). Provenance cites the legacy forest source at base commit",
    "3be25f5240 and the spec preserve group (G1-G17, spec section 3.1). Initial values reproduce",
    "the forest envelope, not improve it.",
    "",
    "Conventions: accel m/s^2 (negative = braking); jerk m/s^3 as positive magnitudes; distances m;",
    "speeds m/s. Tables interpolate via np.interp on the bp axis (v_ego unless noted); legacy",
    "per-frame steps are converted to physical rates by x100 (100 Hz loop).",
    "",
    "| # | Name | Unit | Value | Provenance |",
    "|---|------|------|-------|------------|",
  ]
  for f in fields(params):
    meta = f.metadata
    value = getattr(params, f.name)
    lines.append(f"| {meta['row']} | `{f.name}` | {meta['unit']} | {_format_value(value, meta['kind'])} | {meta['provenance']} |")
  lines.append("")
  return "\n".join(lines)


if __name__ == "__main__":
  import pathlib
  doc_path = pathlib.Path(__file__).resolve().parents[3] / "docs" / "stopping" / "parameters.md"
  doc_path.parent.mkdir(parents=True, exist_ok=True)
  doc_path.write_text(render_parameters_doc())
  print(f"wrote {doc_path}")
