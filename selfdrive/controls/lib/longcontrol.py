import time

import numpy as np
from cereal import car
from openpilot.common.swaglog import cloudlog
from opendbc.car.hyundai.values import CAR as HYUNDAI_CAR
from openpilot.common.pid import PIDController
import math
from collections import deque
from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.controls.lib import stopping_flags
from openpilot.selfdrive.controls.lib.stopping_guard import apply_low_speed_output_slew
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.stop_target_helpers import get_effective_lead_distance, LEAD_STOP_DISTANCE_TARGET
from openpilot.selfdrive.controls.lib.stopping_shadow import (
  STOPPING_SHADOW_LOGGING_ENABLED,
  STOPPING_SHADOW_LOG_PERIOD_S,
  STOPPING_SHADOW_PROFILE_LOG_PERIOD_S,
  STOPPING_SHADOW_SAMPLE_INTERVAL_FRAMES,
  StoppingShadowInput,
  shadow_log_payload,
)
from openpilot.selfdrive.controls.lib.stopping_controller_v2 import StoppingControllerV2
# Stopping Service V3 STAGE 1 SHADOW (docs/stopping/stopping_service_v3_plan.md §6 stage 1): observer-only
# imports; instantiated only for the Santa Fe fingerprint, computed strictly AFTER output_accel is final,
# and NEVER written back to it.
from openpilot.selfdrive.controls.lib.lead_provenance import StoppingLeadAuthority
from openpilot.selfdrive.controls.lib.stop_context import StopContext
from openpilot.selfdrive.controls.lib.stopping_service import Phase as ServicePhase, StoppingService, service_holds_stopping_state
from openpilot.selfdrive.controls.lib.stopping_telemetry import StoppingTelemetry
# Commit B consolidation (FINAL_SPEC §6): the verbatim stop-intent/stop-target predicates moved to
# the arbiter module; longcontrol re-imports them so every public name the kept offline tools and
# tests import keeps resolving until the cleanup commit (alias provision, F24). Names marked
# "alias provision" are not called from this file anymore.
from openpilot.selfdrive.controls.lib.stop_target_arbiter import (
  FAR_STOPPED_LEAD_CLOSE_TARGET_HOLD_M,
  FAR_STOPPED_LEAD_CRAWL_GAP_M,
  LEAD_FOLLOW_TARGET_HOLD_GAP_M,
  MAX_STOP_TARGET_MODE_DISTANCE_M,  # noqa: F401 alias provision
  MIN_STOP_TARGET_MODE_DISTANCE_M,  # noqa: F401 alias provision
  StopTargetArbiter,
  has_explicit_stop_target,
  should_apply_low_speed_stopped_lead_glide_accel_cap,
  should_apply_stop_entry_handoff_soften,
  should_apply_stop_target_approach_mode,  # noqa: F401 alias provision
  should_apply_stop_target_carry_mode,  # noqa: F401 alias provision
  should_enter_stop_target_mode,
  should_hold_low_speed_stop_target_release,  # noqa: F401 alias provision
  should_hold_no_target_standstill_dropout,  # noqa: F401 alias provision
  should_hold_recent_close_stopped_lead_dropout,  # noqa: F401 alias provision
  should_hold_stop_target_dropout,  # noqa: F401 alias provision
  should_hold_stop_target_mode,
  should_release_far_stopped_lead_gap,  # noqa: F401 alias provision
  stop_entry_handoff_accel_cap,
)
from openpilot.frogpilot.controls.lib.force_coast import FORCE_COAST_RAMP_IN_S, get_force_coast_ramped_accel, get_force_coast_target_from_toggles

clip = np.clip
interp = np.interp

STOPPING_V_BP =      [ 0.01,   0.2,   0.5  ]
STOPPING_ACCEL_MAX = [-0.01,  -0.1,   -0.3  ]
STOPPING_ACCEL_MIN = [-0.1,   -0.5,   -1.0  ]

LongCtrlState = car.CarControl.Actuators.LongControlState
EXPERIMENTAL_CLOSE_LEAD_ACCEL_CAP_STRENGTH = 0.5
LEAD_FOLLOW_MIN_HOLD_GAP_M = 2.75

# Force-coast standstill hold magnitude. REGRESSION FIX (routes 00001756/59/5f): the V2-flip replaced the
# legacy StoppingController's FIRM no-target standstill hold (~-0.32..-0.34 m/s^2, baseline a02630ba23) with
# the uniform gentle V2 A_HOLD (~-0.13 relaxing to -0.10). On a gas tip-in out of a no-lead force-coast
# standstill, the car's TCS rejects unwinding from that shallow hold and disables ACC (accFaulted). It did
# NOT reject the firm baseline hold. This restores the baseline magnitude on output_accel (which reaches the
# wire), NOT a_target (discarded in the stopping branch -- why the prior planner fix was a no-op). Tunable.
FORCE_COAST_STANDSTILL_HOLD_ACCEL = -0.32  # m/s^2

# Stopping Service V3 stage-2 LIVE_TERMINAL ownership band (plan §6 stage 2): in
# SERVICE_MODE == "LIVE_TERMINAL" the service owns the stopping-state wire at/below V_OWN;
# once owning, it hands back only above V_RELEASE (hysteresis) or on stopping-state exit.
SERVICE_LIVE_TERMINAL_V_OWN = 0.85      # m/s
SERVICE_LIVE_TERMINAL_V_RELEASE = 0.95  # m/s

FORCE_COAST_NO_TARGET_PID_CAP_BP = [0.0, 1.0, 3.0, 6.0, 10.0]
FORCE_COAST_NO_TARGET_PID_CAP_VALS = [-0.30, -0.45, -0.65, -0.90, -1.05]

# --- Stopping-phase planner-aTarget safety floor (2026-06-18) ---------------------------------
# INCIDENT: route 0000173c seg24 (bookmarked), near-collision driver takeover during a stop. Closing
# on a STOPPED lead, the planner stop target (distanceToStopTarget) correctly collapsed to its 0.05 m
# close-hold pin once dRel reached LEAD_STOP_DISTANCE_TARGET (4.0 m -- the intended rest gap behind the
# lead), so the legacy StoppingController handed command to its terminal glide/hold lane ~4 m early and
# FLAT-FLOORED the command at -0.12 m/s2. At the takeover the car was at v~1.25 m/s with only ~2.2 m to
# the stopped lead: kinematics required >= -0.36 m/s2 just to stop in the gap, and the PLANNER aTarget
# correctly demanded -0.52 m/s2 -- but it was DISCARDED at `output_accel = stop_result.output_accel`,
# so -0.12 coasted the car to a 1.36 m min gap before the driver braked (aEgo dove to -2.06).
#   GRADE NOTE: this was NOT a downhill -- the road was essentially flat (~0.2% grade; verified two
#   ways: seg24 livePose orientationNED.y median -0.066 rad EQUALS the route-wide median -0.066, i.e.
#   that pitch is the camera mount angle, not road grade; and the mount-independent (aEgo - cmd) bias
#   over 1657 rolling-coast frames was only +0.015 m/s2). The original diagnosis mislabeled the mounted
#   pitch as a -6.2% downhill; the real defect is the flat-floor UNDER-BRAKING, which fails on level
#   ground given the gap/speed. (The stop-target "collapse" is correct semantics, NOT a bug: dts is the
#   remaining distance to the intended stop point, which is LEAD_STOP_DISTANCE_TARGET behind the lead,
#   so dts->0.05 at dRel<=4.0 is right; the failure is purely that longcontrol discards the deeper
#   planner demand in the stopping state.)
#
# THE FIX (safe-by-construction, the INVERSE of the reverted P1 approach-decel cap): in the stopping
# phase, while a lead is present and the gap is small and the planner is still demanding decel, the
# command must NEVER be SHALLOWER (less braking) than the planner's aTarget -- take the DEEPER of the
# legacy stopping-controller command and the planner aTarget:
#     output_accel = min(output_accel, a_target)   # more negative = deeper braking
# This can ONLY ADD braking, never reduce it (HARD INVARIANT 1: a one-way deepen; it can never
# produce a command shallower than the cap-off baseline on any frame, so it cannot cause
# under-braking and cannot repeat the P1 failure). It is bounded above (in magnitude) by the planner
# aTarget -- which is itself bounded and already runs through the downstream stop_accel clip -- so it
# cannot run away (INVARIANT 3). It is gated OFF below the standstill band / once the genuine final
# hold is established so it never deepens the gentle terminal hold (INVARIANT 3), and gated to a
# present, small-gap, decel-demanding lead so it never overrides the controller's comfort shaping on
# lead-free stops (INVARIANT 4).
STOPPING_PLANNER_FLOOR_ENABLED = True
STOPPING_PLANNER_FLOOR_V_EGO_MIN = 0.30   # m/s; below this the terminal settle/hold lanes own the command -- floor OFF so it never fights the gentle final hold
STOPPING_PLANNER_FLOOR_GAP_MAX_M = 3.0 * LEAD_STOP_DISTANCE_TARGET  # 12.0 m; only a stop-relevant lead within ~a few * the rest gap arms the floor
STOPPING_PLANNER_FLOOR_A_TARGET_MAX = -0.10  # m/s2; the planner must be demanding meaningful decel (gate off near-zero/positive aTarget)


def stopping_planner_floor_active(v_ego: float, lead_status: bool, lead_v: float, lead_d_rel: float | None,
                                  a_target: float, output_accel: float) -> bool:
  """Gate for the stopping-phase planner-aTarget safety floor (incident 0000173c seg24).

  True only when ALL hold: a lead is present within the stop-relevant gap, the car is still rolling
  ABOVE the terminal-settle/standstill band (so the floor never fights the gentle final hold), the
  planner is demanding meaningful decel, and that planner demand is DEEPER (more negative) than the
  controller's current command (so applying it can only ADD braking -- a one-way deepen). 'Small or
  closing' is satisfied by construction: the lead is inside the stop-relevant gap AND the planner is
  still asking for decel, which is exactly the closing-approach regime the bookmark failed in."""
  if not lead_status or lead_d_rel is None:
    return False
  if v_ego <= STOPPING_PLANNER_FLOOR_V_EGO_MIN:
    return False
  if not (0.0 < lead_d_rel <= STOPPING_PLANNER_FLOOR_GAP_MAX_M):
    return False
  if a_target > STOPPING_PLANNER_FLOOR_A_TARGET_MAX:
    return False
  # one-way deepen: only act when the planner demand is strictly deeper than the current command
  return a_target < output_accel


def should_apply_stopping_planner_floor(cp) -> bool:
  return getattr(cp, "carFingerprint", None) == HYUNDAI_CAR.HYUNDAI_SANTA_FE_HEV_2022


def should_observe_pid_stopping_shadow(
  *,
  v_ego: float,
  a_target: float,
  output_accel: float,
  distance_to_stop_target_m: float | None,
  force_coast: bool,
  lead_status: bool,
  lead_v: float,
  lead_d_rel: float,
  stop_request_active: bool,
  stop_target_approach_active: bool,
  stop_target_carry_active: bool,
) -> bool:
  if v_ego > 2.5:
    return False

  explicit_target = has_explicit_stop_target(distance_to_stop_target_m)
  if stop_request_active or stop_target_approach_active or stop_target_carry_active:
    return True
  if force_coast and output_accel < -0.005:
    return True
  if explicit_target and output_accel < -0.05:
    return True
  if output_accel < -0.18 and a_target < -0.08:
    return True
  if lead_status and lead_v < 0.35 and lead_d_rel > 0.0 and output_accel < -0.05:
    lead_gap_limit = interp(v_ego, [0.0, 1.0, 2.5], [5.2, 6.2, 8.0])
    return lead_d_rel <= lead_gap_limit
  return False


def stop_target_approach_accel_cap(v_ego: float, distance_to_stop_target_m: float | None) -> float:
  distance_to_target = float(clip(0.0 if distance_to_stop_target_m is None else distance_to_stop_target_m, 0.0, 6.0))
  distance_cap = interp(distance_to_target, [0.6, 1.0, 1.6, 2.4, 3.5], [-0.26, -0.22, -0.17, -0.12, -0.08])
  speed_cap = interp(v_ego, [1.0, 2.8, 4.5, 7.0], [-0.08, -0.14, -0.20, -0.26])
  return min(distance_cap, speed_cap)


def stop_target_carry_accel_floor(v_ego: float, distance_to_stop_target_m: float | None) -> float:
  distance_to_target = float(clip(0.0 if distance_to_stop_target_m is None else distance_to_stop_target_m, 0.0, 6.0))
  distance_floor = interp(distance_to_target, [1.4, 2.2, 3.2, 4.5, 6.0], [-0.34, -0.30, -0.26, -0.22, -0.18])
  speed_floor = interp(v_ego, [0.55, 0.75, 0.95, 1.25], [-0.34, -0.30, -0.26, -0.22])
  return max(distance_floor, speed_floor)


def pid_brake_model_alignment_margin(v_ego: float, a_ego: float, a_target: float) -> float:
  # Keep plain PID braking close to planner request. The planner already compensates most physical lag,
  # so runtime should only add a modest extra brake margin instead of re-solving the maneuver itself.
  base_margin = interp(v_ego, [0.3, 2.0, 6.0, 12.0, 25.0], [0.03, 0.04, 0.06, 0.08, 0.10])
  tracking_lag = clip(a_ego - a_target, 0.0, 1.2)
  lag_margin = interp(tracking_lag, [0.0, 0.20, 0.50, 1.20], [0.0, 0.015, 0.040, 0.080])
  return float(base_margin + lag_margin)


def apply_pid_brake_model_alignment(
  output_accel: float,
  a_target: float,
  a_ego: float,
  v_ego: float,
) -> float:
  if a_target >= -0.10:
    return output_accel
  alignment_floor = a_target - pid_brake_model_alignment_margin(v_ego, a_ego, a_target)
  return max(output_accel, alignment_floor)


def should_apply_pid_brake_model_alignment(cp) -> bool:
  return getattr(cp, "carFingerprint", None) == HYUNDAI_CAR.HYUNDAI_SANTA_FE_HEV_2022


# --- cycle-32: Santa Fe HEV approach accel-tracking TRIM -------------------------------------------
# Plant census (routes 00002010-00002013, n=61,960 engaged braking frames, aEgo sampled 0.40 s after
# the sent command): realized/sent ratio 0.97 at -0.6, 0.93 at -1.1..-1.7, 0.89 at -1.9, 0.87 at -2.2,
# 0.78 below -2.5; flat-road rows keep the depth trend (+0.03/+0.08/+0.11/+0.34 by bin), nose-down adds
# ~+0.09; over-delivery (err < -0.2) is 5% of frames. The Hyundai path is pure feedforward (no
# longitudinalTuning gains), so the shortfall is never corrected and hot approaches arrive with end-debt
# (013 s19: driver takeover at 3.2 m/s / 6.7 m after 7.9 s of -1.80 commanded, -1.43 realized; hot
# stops that do land need the -2.25 aim cap -> felt 2.2). The trim is a SEPARATE deepen-only state
# (never pid.i -- the dormant ki reseed branches stay dormant): a bounded, dead-banded integral of the
# DELAY-COMPENSATED tracking error against the UNTRIMMED demand (design review: a sent-referenced
# error against a sub-unity plant is a shortfall estimator that saturates, not a tracking loop -- the
# reference must be what the planner asked for, so the error closes once the car realizes it). Learns
# only while the untrimmed pid demand IS the wire (no downstream cap writer, not at the planner limit,
# not service-owned, no gas override), in the pid state, engaged, V_MIN <= v <= V_MAX, demand <= A_ARM;
# everywhere else it DECAYS toward zero at DECAY (never a step; disengagement resets it). Slow to wind,
# fast to unwind (a regen->friction over-delivery relaxes it in < 1 s; the trim is never positive).
# Applied AFTER the cap family and BEFORE the service takeover, so the service seeds from the trimmed
# wire (continuity) and every pid.i reconstruction above sees the untrimmed value.
SANTA_FE_TRIM_KI_WIND = 1.00        # 1/s  (review proposed 0.50: converges in ~5 s; with the model reference +
                                    # rate guard 1.0 meets the 4.0 s pin at every delay 0.35-0.60 with no
                                    # over-delivery in the nominal lag and a clean 0.78-1.05 x 0.3-0.7 s sweep;
                                    # known edge: delay 0.6 + lag 0.7 shows 0.06 p2p / 0.09 over)
SANTA_FE_TRIM_KI_UNWIND = 2.00      # 1/s
SANTA_FE_TRIM_MAX = 0.40            # m/s^2, deepen-only: trim lives in [-MAX, 0]
SANTA_FE_TRIM_TAU_S = 0.45          # delay compensation: untrimmed demand TAU ago vs aEgo now
SANTA_FE_TRIM_LAG_S = 0.50          # model-reference lag: the plant's first-order response (brake onsets are
                                    # not shortfall -- an uncompensated lag slammed the trim to the bound at
                                    # every ramp-in in the harness, the review's hazard 1)
SANTA_FE_TRIM_WIND_DEADBAND = 0.05  # m/s^2 (0.10 proposed; leaves a 0.10 residual by construction). cycle-33: on
                                    # the road the trim is active on ~20% of braking frames (median -0.09, mean
                                    # -0.025); a re-tune (0.03-0.04 with guard 0.10-0.20) was swept against the
                                    # review's own gates -- every set winds 0.08-0.20 on a step onset at 0.6-0.7 s
                                    # delay for only +4-7 pts of activity -> KEPT; the trim is a small contributor
                                    # by construction (ledger: demand-level plant-gain compensation is a different lever)
SANTA_FE_TRIM_RATE_GUARD_S = 0.25   # s: the wind deadband widens by this x |d(reference)/dt| -- a delay
                                    # mismatch of up to 0.25 s cannot read a ramp-in as shortfall; zero in
                                    # steady state, so the steady residual stays the 0.05 deadband
SANTA_FE_TRIM_UNWIND_START = 0.03   # m/s^2
SANTA_FE_TRIM_LEAK = 0.01           # m/s^3 toward zero inside the deadband (not hold-forever; 0.05 fought the equilibrium)
SANTA_FE_TRIM_DECAY = 1.00          # m/s^3 toward zero whenever not learning
SANTA_FE_TRIM_SLEW = 1.00           # m/s^3 bound on the applied trim's per-frame change, both ways
SANTA_FE_TRIM_A_ARM = -0.75         # m/s^2 demand gate (the shallow bin's shortfall is +0.02)
SANTA_FE_TRIM_V_MIN = 2.5           # m/s: the LIVE service ownership band sits below
SANTA_FE_TRIM_V_MAX = 16.0          # m/s: the census does not establish highway feel
SANTA_FE_TRIM_TAU_FRAMES = int(round(SANTA_FE_TRIM_TAU_S / DT_CTRL))


def update_santa_fe_tracking_trim(trim, ref_demand, a_ego, learn_ok, dt=DT_CTRL, ref_rate=0.0):
  """Pure per-frame trim update. Returns the new trim in [-SANTA_FE_TRIM_MAX, 0], slew-bounded.
  ref_rate: d(reference)/dt this frame -- widens the wind deadband (rate guard), never the unwind."""
  trim = float(trim)
  if learn_ok and ref_demand is not None and math.isfinite(float(a_ego)):
    e = float(ref_demand) - float(a_ego)          # + = over-delivery, - = under-delivery
    wind_band = SANTA_FE_TRIM_WIND_DEADBAND + SANTA_FE_TRIM_RATE_GUARD_S * abs(float(ref_rate))
    if e < -wind_band:
      target = trim + SANTA_FE_TRIM_KI_WIND * (e + wind_band) * dt
    elif e > SANTA_FE_TRIM_UNWIND_START:
      target = trim + SANTA_FE_TRIM_KI_UNWIND * (e - SANTA_FE_TRIM_UNWIND_START) * dt
    else:
      target = min(trim + SANTA_FE_TRIM_LEAK * dt, 0.0)
  else:
    target = min(trim + SANTA_FE_TRIM_DECAY * dt, 0.0)
  target = min(max(target, -SANTA_FE_TRIM_MAX), 0.0)
  step = SANTA_FE_TRIM_SLEW * dt
  return min(max(target, trim - step), trim + step)


def pid_integrator_enabled(pid: PIDController) -> bool:
  return abs(float(pid.k_i)) > 1e-6


def force_coast_no_target_pid_brake_cap(v_ego: float, target_accel: float | None = None) -> float:
  comfort_cap = float(interp(v_ego, FORCE_COAST_NO_TARGET_PID_CAP_BP, FORCE_COAST_NO_TARGET_PID_CAP_VALS))
  return comfort_cap if target_accel is None else float(target_accel)


def should_apply_force_coast_no_target_pid_brake_cap(cp) -> bool:
  return getattr(cp, "carFingerprint", None) == HYUNDAI_CAR.HYUNDAI_SANTA_FE_HEV_2022


PID_STOPPED_LEAD_APPROACH_ENABLED = True
PID_STOPPED_LEAD_APPROACH_V_BP = [6.0, 12.0, 18.0, 22.0]
PID_STOPPED_LEAD_APPROACH_MIN_CLOSING_VALS = [3.0, 4.2, 5.2, 6.0]
PID_STOPPED_LEAD_APPROACH_MAX_LEAD_V_VALS = [2.5, 5.0, 8.5, 10.0]
PID_STOPPED_LEAD_APPROACH_MAX_GAP_VALS = [32.0, 58.0, 80.0, 92.0]
PID_STOPPED_LEAD_APPROACH_RESERVED_GAP_VALS = [9.0, 16.0, 28.0, 34.0]
PID_STOPPED_LEAD_APPROACH_BRAKE_STEP_VALS = [0.012, 0.016, 0.020, 0.024]
PID_SLOWING_LEAD_APPROACH_PROJECT_TIME_S = 1.7
PID_SLOWING_LEAD_APPROACH_MIN_LEAD_DECEL = 0.75
PID_SLOWING_LEAD_APPROACH_MAX_GAP_VALS = [32.0, 46.0, 62.0, 72.0]
PID_SLOWING_LEAD_APPROACH_MAX_STOP_TIME_VALS = [8.5, 7.5, 6.5, 5.8]
PID_SLOWING_LEAD_APPROACH_MIN_PROJECTED_CLOSING_VALS = [2.8, 3.6, 4.4, 5.0]
PID_SLOWING_LEAD_APPROACH_MAX_PROJECTED_TTC_VALS = [5.6, 5.2, 4.8, 4.5]
PID_SLOWING_LEAD_APPROACH_RESERVED_GAP_VALS = [8.0, 13.0, 22.0, 28.0]
PID_SLOWING_LEAD_APPROACH_MIN_DECEL_VALS = [0.85, 1.05, 1.20, 1.30]
PID_SLOWING_LEAD_APPROACH_MAX_DECEL_VALS = [1.35, 1.75, 2.20, 2.35]


def pid_stopped_lead_approach_accel_cap(v_ego: float, lead_v: float, lead_d_rel: float) -> float | None:
  if not (PID_STOPPED_LEAD_APPROACH_V_BP[0] <= v_ego <= PID_STOPPED_LEAD_APPROACH_V_BP[-1]):
    return None
  if lead_d_rel <= 0.0:
    return None

  closing_speed = v_ego - lead_v
  if closing_speed < interp(v_ego, PID_STOPPED_LEAD_APPROACH_V_BP, PID_STOPPED_LEAD_APPROACH_MIN_CLOSING_VALS):
    return None
  if lead_v > interp(v_ego, PID_STOPPED_LEAD_APPROACH_V_BP, PID_STOPPED_LEAD_APPROACH_MAX_LEAD_V_VALS):
    return None
  if lead_d_rel > interp(v_ego, PID_STOPPED_LEAD_APPROACH_V_BP, PID_STOPPED_LEAD_APPROACH_MAX_GAP_VALS):
    return None

  time_gap = lead_d_rel / max(v_ego, 1.0)
  max_time_gap = interp(v_ego, PID_STOPPED_LEAD_APPROACH_V_BP, [4.6, 4.4, 4.2, 4.0])
  if time_gap > max_time_gap:
    return None

  reserved_gap = interp(v_ego, PID_STOPPED_LEAD_APPROACH_V_BP, PID_STOPPED_LEAD_APPROACH_RESERVED_GAP_VALS)
  required_decel = (closing_speed * closing_speed) / (2.0 * max(lead_d_rel - reserved_gap, 1.0))
  if required_decel < 0.45:
    return None

  time_gap_brake = interp(time_gap, [2.6, 3.4, 4.2], [1.45, 1.25, 0.90])
  brake_mag = min(required_decel + 0.08, time_gap_brake)
  return -float(clip(brake_mag, 0.65, 1.45))


def pid_slowing_lead_approach_accel_cap(v_ego: float, lead_v: float, lead_d_rel: float, lead_a: float) -> float | None:
  if not (PID_STOPPED_LEAD_APPROACH_V_BP[0] <= v_ego <= PID_STOPPED_LEAD_APPROACH_V_BP[-1]):
    return None
  if lead_d_rel <= 0.0:
    return None
  if lead_d_rel > interp(v_ego, PID_STOPPED_LEAD_APPROACH_V_BP, PID_SLOWING_LEAD_APPROACH_MAX_GAP_VALS):
    return None

  stopped_lead_v_limit = interp(v_ego, PID_STOPPED_LEAD_APPROACH_V_BP, PID_STOPPED_LEAD_APPROACH_MAX_LEAD_V_VALS)
  if lead_v <= stopped_lead_v_limit:
    return None

  lead_decel = max(-float(lead_a), 0.0)
  if lead_decel < PID_SLOWING_LEAD_APPROACH_MIN_LEAD_DECEL:
    return None

  lead_stop_time = lead_v / max(lead_decel, 1e-3)
  max_stop_time = interp(v_ego, PID_STOPPED_LEAD_APPROACH_V_BP, PID_SLOWING_LEAD_APPROACH_MAX_STOP_TIME_VALS)
  if lead_stop_time > max_stop_time:
    return None

  closing_speed = v_ego - lead_v
  projected_closing_speed = closing_speed + (lead_decel * PID_SLOWING_LEAD_APPROACH_PROJECT_TIME_S)
  min_projected_closing = interp(v_ego, PID_STOPPED_LEAD_APPROACH_V_BP, PID_SLOWING_LEAD_APPROACH_MIN_PROJECTED_CLOSING_VALS)
  if projected_closing_speed < min_projected_closing:
    return None

  projected_ttc = lead_d_rel / max(projected_closing_speed, 0.1)
  max_projected_ttc = interp(v_ego, PID_STOPPED_LEAD_APPROACH_V_BP, PID_SLOWING_LEAD_APPROACH_MAX_PROJECTED_TTC_VALS)
  if projected_ttc > max_projected_ttc:
    return None

  lead_stop_distance = (lead_v * lead_v) / (2.0 * lead_decel)
  reserved_gap = interp(v_ego, PID_STOPPED_LEAD_APPROACH_V_BP, PID_SLOWING_LEAD_APPROACH_RESERVED_GAP_VALS)
  braking_distance = max(lead_d_rel + lead_stop_distance - reserved_gap, 1.0)
  required_decel = (v_ego * v_ego) / (2.0 * braking_distance)
  confidence = interp(max_stop_time - lead_stop_time, [0.0, 1.2, 2.5], [0.72, 0.90, 1.00])
  lead_decel_tighten = interp(lead_decel, [0.75, 1.20, 2.50], [0.00, 0.08, 0.18])
  ttc_tighten = interp(projected_ttc, [2.0, 3.5, 5.5], [0.35, 0.15, 0.00])
  brake_mag = (required_decel * confidence) + lead_decel_tighten + ttc_tighten

  min_meaningful_decel = interp(v_ego, PID_STOPPED_LEAD_APPROACH_V_BP, PID_SLOWING_LEAD_APPROACH_MIN_DECEL_VALS)
  if brake_mag < min_meaningful_decel:
    return None

  max_decel = interp(v_ego, PID_STOPPED_LEAD_APPROACH_V_BP, PID_SLOWING_LEAD_APPROACH_MAX_DECEL_VALS)
  return -float(clip(brake_mag, min_meaningful_decel, max_decel))


def pid_stopped_lead_approach_brake_step(v_ego: float) -> float:
  return float(interp(v_ego, PID_STOPPED_LEAD_APPROACH_V_BP, PID_STOPPED_LEAD_APPROACH_BRAKE_STEP_VALS))


def should_apply_pid_stopped_lead_approach_accel_cap(cp) -> bool:
  return getattr(cp, "carFingerprint", None) == HYUNDAI_CAR.HYUNDAI_SANTA_FE_HEV_2022


def experimental_close_lead_accel_cap(v_ego: float, lead_v: float, lead_d_rel: float) -> float | None:
  if not (4.5 <= v_ego <= 18.0):
    return None
  if lead_d_rel <= 0.0:
    return None

  time_gap = lead_d_rel / max(v_ego, 1.0)
  if time_gap >= 2.8:
    return None

  pullaway_speed = max(lead_v - v_ego, 0.0)
  if pullaway_speed >= 3.0:
    return None

  base_cap = interp(time_gap, [1.2, 1.8, 2.2, 2.8], [-0.05, 0.0, 0.08, 0.45])
  pullaway_allowance = interp(pullaway_speed, [0.0, 0.8, 1.8, 3.0], [0.0, 0.05, 0.20, 0.50])
  return float(min(base_cap + pullaway_allowance, 0.45))


def apply_experimental_close_lead_accel_cap(output_accel: float, close_lead_cap: float) -> float:
  if output_accel <= close_lead_cap:
    return output_accel

  return float(output_accel - ((output_accel - close_lead_cap) * EXPERIMENTAL_CLOSE_LEAD_ACCEL_CAP_STRENGTH))


def low_speed_close_lead_accel_cap(v_ego: float, lead_v: float, lead_d_rel: float) -> float | None:
  if not (0.12 <= v_ego <= 0.95):
    return None
  if lead_d_rel <= 0.0:
    return None

  closing_speed = v_ego - lead_v
  close_to_hold_floor = lead_d_rel <= LEAD_FOLLOW_TARGET_HOLD_GAP_M
  min_closing_speed = 0.04 if close_to_hold_floor else 0.10
  if closing_speed < min_closing_speed:
    return None

  activation_gap = interp(v_ego, [0.12, 0.35, 0.65, 0.95], [3.45, 3.65, 3.95, 4.20])
  if lead_d_rel > activation_gap:
    return None

  gap_cap = interp(lead_d_rel, [2.10, LEAD_FOLLOW_MIN_HOLD_GAP_M, LEAD_FOLLOW_TARGET_HOLD_GAP_M, 3.60, 4.15],
                   [-0.80, -0.72, -0.62, -0.52, -0.45])
  closing_extra = interp(closing_speed, [0.10, 0.35, 0.70, 1.00], [0.00, 0.02, 0.05, 0.08])
  speed_extra = interp(v_ego, [0.12, 0.35, 0.65, 0.95], [0.00, 0.01, 0.025, 0.04])
  return float(clip(gap_cap - closing_extra - speed_extra, -0.90, -0.42))


def low_speed_close_lead_brake_step(v_ego: float, lead_d_rel: float) -> float:
  if lead_d_rel < LEAD_FOLLOW_MIN_HOLD_GAP_M:
    return float(interp(v_ego, [0.12, 0.35, 0.65, 0.95], [0.012, 0.016, 0.020, 0.024]))
  if lead_d_rel < LEAD_FOLLOW_TARGET_HOLD_GAP_M:
    return float(interp(v_ego, [0.12, 0.35, 0.65, 0.95], [0.0045, 0.0055, 0.0065, 0.0080]))
  return float(interp(v_ego, [0.12, 0.35, 0.65, 0.95], [0.003, 0.004, 0.005, 0.006]))


def low_speed_stopped_lead_glide_accel_cap(v_ego: float, lead_v: float, lead_d_rel: float, distance_to_stop_target_m: float | None) -> float | None:
  if not (0.02 <= v_ego <= 1.25):
    return None
  explicit_stop_target = distance_to_stop_target_m is not None and distance_to_stop_target_m >= 0.0
  if not explicit_stop_target and lead_d_rel > FAR_STOPPED_LEAD_CRAWL_GAP_M:
    return None
  lead_v_limit = interp(v_ego, [0.02, 0.10, 0.35, 0.65], [1.00, 0.70, 0.25, 0.25])
  if lead_d_rel <= 0.0 or lead_v > lead_v_limit:
    return None

  closing_speed = v_ego - lead_v
  closing_threshold = interp(v_ego, [0.02, 0.10, 0.20, 0.35, 0.65, 1.25], [-0.75, -0.40, 0.04, 0.12, 0.45, 0.55])
  if closing_speed < closing_threshold:
    return None

  activation_gap = interp(v_ego, [0.02, 0.20, 0.35, 0.65, 0.95, 1.25], [6.4, 6.4, 6.2, 6.8, 8.0, 8.8])
  if lead_d_rel > activation_gap:
    return None

  gap_cap = interp(lead_d_rel, [4.8, 6.0, 7.5, 8.8], [-0.50, -0.44, -0.39, -0.35])
  speed_cap = interp(v_ego, [0.02, 0.10, 0.35, 0.65, 0.95, 1.25], [-0.20, -0.22, -0.35, -0.41, -0.47, -0.53])
  high_speed_weight = clip((v_ego - 0.25) / 0.35, 0.0, 1.0)
  base_cap = ((1.0 - high_speed_weight) * speed_cap) + (high_speed_weight * min(gap_cap, speed_cap))
  if lead_d_rel <= 4.15:
    near_hold_gap_cap = interp(lead_d_rel, [2.10, LEAD_FOLLOW_MIN_HOLD_GAP_M, LEAD_FOLLOW_TARGET_HOLD_GAP_M, 3.60, 4.15],
                               [-0.76, -0.68, -0.58, -0.50, -0.44])
    base_cap = min(base_cap, near_hold_gap_cap)
  closing_extra = interp(closing_speed, [0.00, 0.45, 0.75, 1.10], [0.00, 0.00, 0.04, 0.08])
  distance_relief = 0.0
  if explicit_stop_target:
    distance_relief = interp(distance_to_stop_target_m, [2.0, 3.5, 4.5], [-0.02, 0.0, 0.03])
  return float(clip(base_cap - closing_extra + distance_relief, -0.60, -0.18))


def far_stopped_lead_crawl_accel_cap(v_ego: float, lead_d_rel: float) -> float:
  gap_cap = interp(lead_d_rel, [5.0, 6.5, 9.0, 11.0], [0.02, 0.06, 0.12, 0.16])
  speed_cap = interp(v_ego, [0.00, 0.20, 0.55, 0.85], [0.16, 0.13, 0.08, 0.04])
  return float(min(gap_cap, speed_cap))


def far_stopped_lead_brake_floor(v_ego: float, lead_d_rel: float) -> float:
  gap_floor = interp(lead_d_rel, [5.0, 6.5, 9.0, 11.0], [-0.18, -0.14, -0.08, -0.05])
  speed_floor = interp(v_ego, [0.00, 0.20, 0.55], [-0.08, -0.12, -0.20])
  return float(max(gap_floor, speed_floor))


def far_stopped_lead_settle_accel_cap(v_ego: float, lead_d_rel: float, distance_to_stop_target_m: float | None) -> float | None:
  if distance_to_stop_target_m is None or distance_to_stop_target_m <= FAR_STOPPED_LEAD_CLOSE_TARGET_HOLD_M:
    return None
  if lead_d_rel <= FAR_STOPPED_LEAD_CRAWL_GAP_M or not (0.03 <= v_ego < 0.55):
    return None

  gap_cap = interp(lead_d_rel, [5.0, 6.5, 9.0, 11.0], [-0.30, -0.25, -0.17, -0.12])
  speed_cap = interp(v_ego, [0.03, 0.12, 0.25, 0.55], [-0.18, -0.22, -0.27, -0.33])
  return float(min(gap_cap, speed_cap))


def should_apply_experimental_close_lead_accel_cap(cp, experimental_mode: bool) -> bool:
  return experimental_mode and getattr(cp, "carFingerprint", None) == HYUNDAI_CAR.HYUNDAI_SANTA_FE_HEV_2022


def should_apply_low_speed_close_lead_accel_cap(cp, v_ego: float) -> bool:
  # TERMINAL-GLIDE PROFILE (sub-0.30 redesign): the over-brake this cap induces is the leapfrog
  # cause -- but ONLY ABOVE 0.30 m/s, where over-braking near-stops the car short and the lead
  # leapfrogs it. So the terminal-glide bypass is now V-GATED: bypass (cap OFF) ONLY when the flag
  # is on AND v_ego > STOPPING_PLANNER_FLOOR_V_EGO_MIN (0.30) -- above 0.30 the jerk-limited tracker
  # glides to 4.0 m and the seg24 STOPPING_PLANNER_FLOOR owns anti-collision. At v_ego <= 0.30 the
  # cap stays ACTIVE exactly as legacy (byte-identical sub-0.30 authority), so no new under-brake
  # hole is opened: the deepest-as-today brake answers any closing error in the terminal band.
  if getattr(cp, "carFingerprint", None) != HYUNDAI_CAR.HYUNDAI_SANTA_FE_HEV_2022:
    return False
  if stopping_flags.SANTA_FE_TERMINAL_GLIDE_PROFILE_ENABLED and v_ego > STOPPING_PLANNER_FLOOR_V_EGO_MIN:
    return False
  return True


# --- Close-the-gap forward creep behind a confirmed STOPPED lead (route 00001764 seg27) ----------
# INCIDENT (live): behind a CONFIRMED STOPPED lead the car rested ~5.7 m TRUE (dts~1.0) and HELD,
# never creeping up to the intended rest gap (LEAD_STOP_DISTANCE_TARGET + IncreasedStoppedDistance ISD).
# WHY (verified): low_speed_stopped_lead_glide_accel_cap is a gentle BRAKE cap with no precise stop
# position, so it brakes to a near-stop wherever momentum runs out, then the V2 A_HOLD pins it; the
# arbiter far-stopped crawl is gated off when dts<=1.8; the V2 controller cannot command positive accel.
# StopReq stays 0 at the hold (Kalman vEgo dithers >0.01), so the car is held by the soft command -- a
# tiny positive aReq CAN move it.
# THE FIX (complement of the seg24 planner floor, which DEEPENS to stop short -- this ADDS a tiny crawl
# to close a too-FAR rest): a stateful, latched, slew-limited gentle FORWARD creep applied as the LAST
# writer of the stopping-state output_accel (after every cap, before the force_coast/standstill hold) so
# the glide brake cannot clobber it and last_output_accel ratchets to the creep's own value. POSITIVE-ONLY
# (no lower-bound relax lane). Safe-by-construction: stopping-state only, only a CONFIRMED stopped lead
# (lead_v<=0.30), never under force_coast, never with no lead; latched/hysteretic to avoid the v<=0.06
# re-arm oscillation; disarmed by GAP with an ISD-aware hard floor; bounded gentle accel + slew so jerk is
# tiny; a velocity-safety cap disarms on overspeed. All gap comparisons are EFFECTIVE-space
# (lead_d_rel_eff = true gap - ISD, since PUBLISH_TRUE_LEAD_DISTANCE is True); the eff rest target +
# hard floor are ISD-aware clamped so the TRUE rest stays in [3.0, 5.0] for any ISD (0-3.05 m).
# RETIRED 2026-07-01 (escape-leapfrog review): per the user taxonomy any post-stop motion is
# disliked (a settle followed by a crawl IS the leapfrog feel), and 41 fresh engaged settles show
# the terminal glide lands rests in-band without it -- rests are now FINAL. The staged-backstop
# rationale (close a too-far rest) no longer applies; code kept for one release, then delete.
STOPPING_CLOSE_GAP_CREEP_ENABLED = False   # kill switch
CREEP_ARM_V_EGO_MAX = 0.06                # ARM only at standstill (v_ego <= this)
CREEP_ARM_STANDSTILL_TIME_S = 1.00       # ARM only after stable standstill hold, not while acquiring the stop
CREEP_ARM_A_EGO_ABS_MAX = 0.08           # quietness gate for the stable-hold timer
CREEP_DISARM_V_EGO_MAX = 0.12            # DISARM (safety) if the creep/rebound pushes v above this
CREEP_STOPPED_LEAD_V_MAX = 0.30          # only a CONFIRMED stopped lead may be crept toward
CREEP_ARM_GAP_MARGIN_M = 0.7            # ARM only when eff gap > eff rest_target + this (clearly too far)
CREEP_DISARM_BAND_M = 0.30              # DISARM when eff gap <= eff rest_target + this (~target)
CREEP_REST_GAP_MIN_M = 3.0              # hard TRUE-gap floor: never rest below this (band min, 2026-07-20)
CREEP_REST_GAP_MAX_M = 5.0             # upper bound of the allowed final TRUE rest window
CREEP_ACCEL_MAX = 0.04                # hard ceiling on the gentle positive crawl command (m/s^2)
CREEP_SLEW_UP = 0.0020                # per-frame ramp UP toward the crawl target (tiny jerk)
CREEP_SLEW_DOWN = 0.010               # per-frame ramp DOWN when the crawl target falls / on wind-down


def stopping_close_gap_creep_eff_floor_m(increased_stopped_distance: float) -> float:
  """Hard rest floor in EFF-SPACE. TRUE gap = eff gap + ISD, so eff floor = (MIN - ISD), clamped >=0."""
  return float(max(CREEP_REST_GAP_MIN_M - float(increased_stopped_distance), 0.0))


def stopping_close_gap_creep_rest_target_m(increased_stopped_distance: float) -> float:
  """Intended rest gap in EFF-SPACE (do NOT re-add ISD -- lead_d_rel_eff already subtracts it). Clamped
  so (target + band + ISD) <= CREEP_REST_GAP_MAX_M (true rest <= 5.0) and >= the eff hard floor."""
  eff_floor = stopping_close_gap_creep_eff_floor_m(increased_stopped_distance)
  upper = CREEP_REST_GAP_MAX_M - CREEP_DISARM_BAND_M - float(increased_stopped_distance)
  return float(clip(min(LEAD_STOP_DISTANCE_TARGET, upper), eff_floor, CREEP_REST_GAP_MAX_M))


def stopping_close_gap_creep_should_arm(v_ego: float, lead_status: bool, lead_v: float, lead_d_rel: float,
                                        force_coast: bool, rest_target_m: float,
                                        increased_stopped_distance: float) -> bool:
  """ARM: standstill, confirmed stopped lead, eff gap clearly above the eff rest target (and the eff
  hard floor), never under force_coast / no lead. lead_d_rel is the EFFECTIVE (ISD-compensated) gap."""
  if force_coast or not lead_status or lead_d_rel <= 0.0:
    return False
  if lead_v > CREEP_STOPPED_LEAD_V_MAX:
    return False
  if v_ego > CREEP_ARM_V_EGO_MAX:
    return False
  if lead_d_rel < stopping_close_gap_creep_eff_floor_m(increased_stopped_distance):
    return False
  return lead_d_rel > rest_target_m + CREEP_ARM_GAP_MARGIN_M


def stopping_close_gap_creep_should_disarm(v_ego: float, lead_status: bool, lead_v: float, lead_d_rel: float,
                                           force_coast: bool, rest_target_m: float,
                                           increased_stopped_distance: float) -> bool:
  """DISARM: eff gap reached (target + band) OR hit the eff hard floor, lead departed/lost/not-stopped,
  force_coast asserted, or an overspeed safety. lead_d_rel is the EFFECTIVE (ISD-compensated) gap."""
  if force_coast or not lead_status or lead_d_rel <= 0.0:
    return True
  if lead_v > CREEP_STOPPED_LEAD_V_MAX:
    return True
  if v_ego > CREEP_DISARM_V_EGO_MAX:
    return True
  if lead_d_rel <= stopping_close_gap_creep_eff_floor_m(increased_stopped_distance):
    return True
  return lead_d_rel <= rest_target_m + CREEP_DISARM_BAND_M


def stopping_close_gap_creep_accel_target(v_ego: float, lead_d_rel: float) -> float:
  """Gentle bounded positive crawl. Reuses far_stopped_lead_crawl_accel_cap's gap/speed taper, hard-capped
  at CREEP_ACCEL_MAX so jerk stays tiny. Never negative. lead_d_rel is the EFFECTIVE gap."""
  shaped = far_stopped_lead_crawl_accel_cap(v_ego, lead_d_rel)
  return float(clip(min(shaped, CREEP_ACCEL_MAX), 0.0, CREEP_ACCEL_MAX))


def should_apply_stopping_close_gap_creep(cp) -> bool:
  return getattr(cp, "carFingerprint", None) == HYUNDAI_CAR.HYUNDAI_SANTA_FE_HEV_2022


def long_control_state_trans(CP, active, long_control_state, v_ego,
                             should_stop, brake_pressed, cruise_standstill, frogpilot_toggles, a_target=0.0,
                             distance_to_stop_target_m: float | None = None):
  # Ignore cruise standstill if car has a gas interceptor
  cruise_standstill = cruise_standstill and not CP.enableGasInterceptor
  stopping_condition = should_stop or should_enter_stop_target_mode(v_ego, a_target, distance_to_stop_target_m)
  if long_control_state == LongCtrlState.stopping and not should_stop:
    stopping_condition = stopping_condition or should_hold_stop_target_mode(v_ego, a_target, distance_to_stop_target_m)
  starting_condition = (not stopping_condition and
                        not cruise_standstill and
                        not brake_pressed)
  started_condition = v_ego > frogpilot_toggles.vEgoStarting

  if not active:
    long_control_state = LongCtrlState.off

  else:
    if long_control_state == LongCtrlState.off:
      if not starting_condition:
        long_control_state = LongCtrlState.stopping
      else:
        if starting_condition and CP.startingState:
          long_control_state = LongCtrlState.starting
        else:
          long_control_state = LongCtrlState.pid

    elif long_control_state == LongCtrlState.stopping:
      if starting_condition and CP.startingState:
        long_control_state = LongCtrlState.starting
      elif starting_condition:
        long_control_state = LongCtrlState.pid

    elif long_control_state in [LongCtrlState.starting, LongCtrlState.pid]:
      if stopping_condition:
        long_control_state = LongCtrlState.stopping
      elif started_condition:
        long_control_state = LongCtrlState.pid
  return long_control_state


class LongControl:
  def __init__(self, CP):
    self.CP = CP
    self.long_control_state = LongCtrlState.off
    self.pid = PIDController((CP.longitudinalTuning.kpBP, CP.longitudinalTuning.kpV),
                             (CP.longitudinalTuning.kiBP, CP.longitudinalTuning.kiV),
                             rate=1 / DT_CTRL)
    self.last_output_accel = 0.0
    # Force Coast no-target command ramp. Safety/stop paths disarm it and retain their existing authority.
    self.force_coast_ramp_active = False
    self.force_coast_ramp_elapsed_s = 0.0
    self.force_coast_ramp_start_accel = 0.0
    # Close-the-gap forward-creep latch (route 00001764 seg27): hysteresis so the creep does not
    # oscillate (a pure per-frame v_ego<=0.06 gate would re-arm/re-brake as the creep lifts v above 0.06).
    self.creeping = False
    self.close_gap_creep_standstill_time_s = 0.0
    # THE single StopTargetArbiter in the system (FINAL_SPEC §6 Commit B, F2) -- the V2 facade
    # consumes this arbiter's StopDecision via its trailing kwarg and never instantiates one.
    self.arbiter = StopTargetArbiter(CP)
    self.stopping_controller = StoppingControllerV2(CP)
    self.stopping_shadow_frame = 0
    self.last_stopping_shadow_log_t = 0.0
    self.last_stopping_shadow_profile = ""
    self.last_stopping_shadow_phase_source: tuple | None = None
    # Stopping Service V3 objects (Santa Fe scope only; plan §6). ONE input-assembly path
    # (_run_stopping_service) feeds them in both modes so SHADOW and LIVE cannot drift:
    #   SERVICE_MODE == "SHADOW" (stage 1): written only via _update_stopping_service_shadow,
    #     strictly after output_accel is final; never feeds back into the control path.
    #   SERVICE_MODE == "LIVE_TERMINAL" (stage 2): the service+context run the SAME full-band
    #     observation (warm a_coast/gap-filter/lead-latch + 0.85-2.5 telemetry) and additionally
    #     own the stopping-state wire for v <= SERVICE_LIVE_TERMINAL_V_OWN (takeover block in
    #     update()).
    #   SERVICE_MODE == "LIVE" (stage 3): same observation; the service owns the wire on EVERY
    #     frame it reports active, in BOTH pid and stopping states (no 0.85 seam) -- see the
    #     takeover block in update().
    self._service_shadow_scope = getattr(CP, "carFingerprint", None) == HYUNDAI_CAR.HYUNDAI_SANTA_FE_HEV_2022
    self._service_shadow_disabled = False  # latched True on the first observer exception (no 100 Hz log flood)
    self._service_shadow_ctx = StopContext()
    self._service_shadow_svc = StoppingService()
    self._service_shadow_tel = StoppingTelemetry()
    # The baseline radar/MPC and arbiter paths keep every selected radar lead. Only the custom
    # StoppingService gets this stricter, track-scoped authority boundary.
    self._service_lead_certificate = StoppingLeadAuthority()
    # stage-2 LIVE_TERMINAL ownership state: _service_live_owning is "the service wrote the wire on
    # the PREVIOUS frame" (drives the legacy-cap bypass + the handback hysteresis);
    # _service_live_disabled latches True on the first LIVE exception -- ownership stays OFF for the
    # rest of the drive and the fully-computed legacy chain keeps the wire (never a silent no-brake).
    self._service_live_owning = False
    self._service_live_disabled = False
    # cycle-32 tracking trim (Santa Fe HEV only): separate deepen-only state + untrimmed-demand ring
    self._trim_scope = getattr(CP, "carFingerprint", None) == HYUNDAI_CAR.HYUNDAI_SANTA_FE_HEV_2022
    self._trim_i = 0.0
    self._trim_ref = deque(maxlen=SANTA_FE_TRIM_TAU_FRAMES)
    self._trim_ref_filt = None
    self._trim_clean = 0
    self._trim_pid_untrimmed = None

  def reset(self):
    self.pid.reset()
    self.stopping_controller.reset()
    self.arbiter.reset()
    self.stopping_shadow_frame = 0
    self.creeping = False
    self.close_gap_creep_standstill_time_s = 0.0
    self.force_coast_ramp_active = False
    self.force_coast_ramp_elapsed_s = 0.0
    self.force_coast_ramp_start_accel = 0.0
    # LIVE_TERMINAL ownership drops with the state machine (disengage/off); the exception latch
    # (_service_live_disabled) deliberately survives reset(): it is drive-scoped, not stop-scoped.
    self._service_live_owning = False
    self._trim_i = 0.0                 # cycle-32: disengagement/off resets the trim (no ramp needed)
    self._trim_ref.clear()
    self._trim_ref_filt = None
    self._trim_clean = 0

  def _new_stopping_shadow_debug_if_due(self, observer_scope: str) -> dict[str, object] | None:
    if not STOPPING_SHADOW_LOGGING_ENABLED:
      return None
    self.stopping_shadow_frame += 1
    if self.stopping_shadow_frame % STOPPING_SHADOW_SAMPLE_INTERVAL_FRAMES != 0:
      return None
    return {"shadow_observer_scope": observer_scope}

  def _populate_pid_stopping_shadow_debug(
    self,
    debug: dict[str, object],
    CS,
    output_accel: float,
    distance_to_stop_target_m: float | None,
    lead_status: bool,
    lead_v: float,
    lead_d_rel: float,
    release_lock_active: bool,
  ) -> None:
    oracle = getattr(self.stopping_controller, "shadow_oracle", None)
    if oracle is None:
      return

    explicit_target_available = has_explicit_stop_target(distance_to_stop_target_m)
    remaining_m = float(distance_to_stop_target_m) if explicit_target_available else None
    rollout_m = float(getattr(self.stopping_controller, "low_speed_rollout_m", 0.0))
    phase = int(getattr(self.stopping_controller, "phase", 0))
    shadow_decision = oracle.evaluate(
      StoppingShadowInput(
        output_accel=output_accel,
        last_output_accel=self.last_output_accel,
        should_stop=True,
        v_ego=CS.vEgo,
        a_ego=CS.aEgo,
        stop_accel=self.CP.stopAccel,
        remaining_m=remaining_m,
        explicit_target_available=explicit_target_available,
        rollout_m=rollout_m,
        phase=phase,
        release_lock_active=release_lock_active,
        rebound_arrest_active=False,
        lead_status=lead_status,
        lead_v=lead_v,
        lead_d_rel=lead_d_rel,
      )
    )
    shadow_decision.write_debug(debug)
    debug["shadow_authority_active"] = False
    debug["distance_to_stop_target_m"] = None if distance_to_stop_target_m is None else float(distance_to_stop_target_m)
    debug["remaining_m"] = remaining_m
    debug["phase"] = phase
    debug["rollout_m"] = rollout_m
    debug["release_lock_active"] = bool(release_lock_active)
    debug["rebound_arrest_active"] = False

  def _arbiter_hold_telemetry(self) -> dict[str, object]:
    # §3.2 row 1 retirement instrumentation (Commit C): cumulative legacy-vs-consolidated
    # dropout-hold divergence counters + the per-frame consolidated-hold shadow state. Rides
    # every stopping_shadow event so the soak's rlogs carry the retirement evidence.
    return {
      "legacy_hold_fired": int(self.arbiter.legacy_hold_fired),
      "single_hold_covered": int(self.arbiter.single_hold_covered),
      "hold_divergence": int(self.arbiter.hold_divergence),
      "consolidated_hold_active": bool(self.arbiter.consolidated_hold_active),
      "consolidated_hold_source": int(self.arbiter.consolidated_hold_source),
      "consolidated_target_m": float(self.arbiter.consolidated_target_m),
    }

  def _log_stopping_shadow(
    self,
    debug: dict[str, object],
    CS,
    output_accel: float,
    lead_status: bool,
    lead_v: float,
    lead_d_rel: float,
  ) -> None:
    # Emission gate rewritten for the v2 telemetry contract (FINAL_SPEC §2, F15): the legacy
    # gate early-returned without a non-empty shadow_profile key, which a v2 debug dict never
    # carries -- under the v2 controller zero events would be emitted and the soak's rlog evidence
    # base would silently vanish.
    version = str(debug.get("version", ""))
    profile = str(debug.get("shadow_profile", ""))
    if not version.startswith("v2_") and not profile:
      return

    now = time.monotonic()
    elapsed_s = now - self.last_stopping_shadow_log_t

    if version.startswith("v2_"):
      # v2 cadence: (phase, source) change replaces the legacy 0.8 s profile-change trigger
      # (which has no v2 analog); the 2.0 s periodic floor is kept.
      phase_source = (debug.get("phase"), debug.get("source"))
      change_log_due = phase_source != self.last_stopping_shadow_phase_source
      periodic_log_due = elapsed_s >= STOPPING_SHADOW_LOG_PERIOD_S
      if not change_log_due and not periodic_log_due:
        return
      # v2 payload: passthrough of the facade-filled debug dict + ground-truth fields.
      payload = dict(debug)
      payload.update(
        v_ego=float(CS.vEgo),
        a_ego=float(CS.aEgo),
        output_accel=float(output_accel),
        lead_status=bool(lead_status),
        lead_v=float(lead_v),
        lead_d_rel=float(lead_d_rel),
      )
      payload.update(self._arbiter_hold_telemetry())
      cloudlog.event("stopping_shadow", **payload)
      self.last_stopping_shadow_log_t = now
      self.last_stopping_shadow_phase_source = phase_source
      return

    profile_changed = profile != self.last_stopping_shadow_profile
    profile_log_due = profile != "no_change" and (profile_changed or elapsed_s >= STOPPING_SHADOW_PROFILE_LOG_PERIOD_S)
    periodic_log_due = elapsed_s >= STOPPING_SHADOW_LOG_PERIOD_S
    if not profile_log_due and not periodic_log_due:
      return

    payload = shadow_log_payload(
      debug,
      v_ego=CS.vEgo,
      a_ego=CS.aEgo,
      output_accel=output_accel,
      lead_status=lead_status,
      lead_v=lead_v,
      lead_d_rel=lead_d_rel,
    )
    payload.update(self._arbiter_hold_telemetry())
    cloudlog.event("stopping_shadow", **payload)
    self.last_stopping_shadow_log_t = now
    self.last_stopping_shadow_profile = profile

  def _run_stopping_service(self, *, run, CS, a_target, a_target_trajectory, should_stop, distance_to_stop_target_m,
                            accel_limits, lead_status, lead_v, lead_d_rel, lead_track_id=None,
                            lead_service_authorized=True, increased_stopped_distance, wire_accel, reference_accel=None):
    """Stopping Service V3 -- the SINGLE input-assembly path for both modes (plan §6), so SHADOW and
    LIVE_TERMINAL can never drift on conditioned inputs (provenance-authorized planner shouldStop,
    raw dts/aTarget, TRUE lead distance -- service laws are in TRUE meters, ISD enters only
    D_REST_NOM). Baseline consumers keep the raw selected radar lead.

    ``run`` is the shared full-band gate (v < 2.5 or stopping, and active) in BOTH modes -- LIVE
    runs the identical warm observation and additionally writes the wire only in its own band (see
    the takeover block in update()); when False the service is reset and the per-settle telemetry
    summary emitted (rearm) -- the SHADOW out-of-band semantics, shared.
    ``wire_accel`` in SHADOW is the final wire (observer); in LIVE it is the legacy chain value at
    the takeover seam (pre force-coast hold/final clip): the jerk-consistent takeover SEED (cold
    entry seeds the jerk limiter from it; warm takeover re-anchors via reseed_takeover) and the
    a_coast a_cmd source (within one V2 jerk step of the actual actuated command).
    ``reference_accel`` (LIVE own-band only) flips telemetry to wire=service output vs
    reference=the legacy-would-have chain value, so divergence-vs-legacy stays computable from
    rlogs; on frames where the service did not actually enter (result inactive) the legacy chain
    keeps the wire, so telemetry keeps shadow semantics there too.
    Returns the ServiceResult (None when not run) -- SHADOW callers ignore it."""
    if not run:
      if self._service_shadow_svc.phase != ServicePhase.INACTIVE:  # settle over / out of band: emit summary, rearm
        self._service_shadow_svc.reset()
        self._service_shadow_tel.update(phase="INACTIVE", active=False, shadow_accel=0.0, wire_accel=float(wire_accel),
                                        v_ego=float(CS.vEgo), d_gap=None, dts=None, wheel_stop_latched=False, dt=DT_CTRL)
        self._service_shadow_ctx.reset()
      return None
    service_lead_status = bool(lead_status and lead_service_authorized)
    signals = self._service_shadow_ctx.update(
      v_ego=CS.vEgo, a_ego=CS.aEgo, a_cmd=wire_accel,
      lead_status=service_lead_status, lead_v=float(lead_v),
      lead_d_rel=float(lead_d_rel) if service_lead_status else None,
      lead_track_id=int(lead_track_id) if service_lead_status and lead_track_id is not None else None,
      standstill=bool(getattr(CS, "standstill", False)), dt=DT_CTRL)
    dts = (float(distance_to_stop_target_m)
           if distance_to_stop_target_m is not None and distance_to_stop_target_m >= 0.0 else None)
    result = self._service_shadow_svc.update(
      engaged=True, v_ego=CS.vEgo, a_ego=CS.aEgo, a_target=a_target,
      should_stop=bool(should_stop), dts_planner=dts, planner_min_limit=accel_limits[0],
      signals=signals, lead_status=service_lead_status, lead_v=float(lead_v),
      increased_stopped_distance=float(increased_stopped_distance), dt=DT_CTRL, wire_accel=wire_accel,
      a_target_trajectory=a_target_trajectory)
    if reference_accel is None or not result.active:  # SHADOW / LIVE observation / not entered: wire=the live chain
      tel_shadow, tel_wire = result.accel, float(wire_accel)
    else:                                             # LIVE owned: the service output IS the wire; legacy chain is the reference
      tel_shadow, tel_wire = float(reference_accel), result.accel
    self._service_shadow_tel.update(
      phase=result.phase.name, active=result.active, shadow_accel=tel_shadow, wire_accel=tel_wire,
      v_ego=float(CS.vEgo), d_gap=signals.d_gap, dts=dts,
      wheel_stop_latched=signals.wheel_stop_latched, dt=DT_CTRL,
      gov=(result.debug.get("a_gov"), result.debug.get("a_barrier")) if result.debug else None)
    return result

  def _update_stopping_service_shadow(self, active, CS, a_target, a_target_trajectory, should_stop, distance_to_stop_target_m,
                                      accel_limits, lead_status, lead_v, lead_d_rel, lead_track_id,
                                      lead_service_authorized, increased_stopped_distance, wire_accel) -> None:
    """Stopping Service V3 STAGE 1 SHADOW (plan §6 stage 1). Zero wire impact BY CONSTRUCTION: called
    strictly after self.last_output_accel is assigned, computes only into service-owned objects via
    the shared _run_stopping_service path, and returns None -- nothing here is read by the control
    path."""
    in_band = CS.vEgo < 2.5 or self.long_control_state == LongCtrlState.stopping
    self._run_stopping_service(
      run=in_band and active, CS=CS, a_target=a_target, a_target_trajectory=a_target_trajectory, should_stop=should_stop,
      distance_to_stop_target_m=distance_to_stop_target_m, accel_limits=accel_limits,
      lead_status=lead_status, lead_v=lead_v, lead_d_rel=lead_d_rel, lead_track_id=lead_track_id,
      lead_service_authorized=lead_service_authorized,
      increased_stopped_distance=increased_stopped_distance, wire_accel=wire_accel)

  def update(
    self,
    active,
    CS,
    a_target,
    should_stop,
    distance_to_stop_target_m,
    accel_limits,
    frogpilot_toggles,
    experimental_mode=False,
    lead_status=False,
    lead_v=0.0,
    lead_d_rel=0.0,
    lead_a=0.0,
    lead_track_id=None,
    force_coast=False,
    increased_stopped_distance=0.0,
    a_target_trajectory=None,
    lead_model_prob=None,
    model_should_stop=None,
    freeze_integrator=False,
  ):
    """Update longitudinal control. This updates the state machine and runs a PID loop"""
    self.pid.neg_limit = accel_limits[0]
    self.pid.pos_limit = accel_limits[1]
    human_acceleration_active = frogpilot_toggles.human_acceleration and not experimental_mode
    standstill = bool(getattr(CS, "standstill", False)) or bool(CS.cruiseState.standstill)
    lead_service_authorized = self._service_lead_certificate.update(
      v_ego=float(CS.vEgo), lead_status=bool(active and lead_status), lead_d_rel=float(lead_d_rel),
      lead_track_id=lead_track_id, model_prob=lead_model_prob)
    if model_should_stop is None:
      # Compatibility for direct LongControl callers. The live controlsd seam supplies the model
      # bit, allowing us to distinguish model/force-coast intent from radar-derived MPC intent.
      service_should_stop = bool(should_stop)
    else:
      service_should_stop = bool(should_stop and (lead_service_authorized or model_should_stop or force_coast))

    # Single-point ISD boundary compensation (FINAL_SPEC §4.2.4, F4): computed ONCE here, consumed
    # only by the arbiter call, the stopping-controller update call (the legacy controller's in-layer
    # far-stopped-lead release predicate is distance-tuned, G11 row 32) and the kept-verbatim Santa Fe
    # stopping quirk caps (the named allowlist is AST-guarded in test_stop_target_arbiter.py). Passthrough while
    # stopping_flags.PUBLISH_TRUE_LEAD_DISTANCE is False; subtracts ISD after the flip so the
    # entire longcontrol stopping layer keeps seeing the gap it was tuned against. The
    # experimental moving-lead cap below deliberately keeps the raw published distance.
    # RETIRE: see FINAL_SPEC §3.2 row 6
    lead_d_rel_eff = get_effective_lead_distance(float(lead_d_rel), float(increased_stopped_distance))

    output_accel = self.last_output_accel
    release_lock_active = False
    max_expected_accel = interp(CS.vEgo, STOPPING_V_BP, STOPPING_ACCEL_MAX)

    # One arbiter owns stop intent + stop target (Commit B). last_output_accel is previous-frame;
    # long_control_state is the current (pre-transition) state; standstill is CS.standstill only
    # (the arbiter rebuilds the legacy composite internally).
    decision = self.arbiter.update(
      v_ego=CS.vEgo,
      a_ego=CS.aEgo,
      a_target=a_target,
      raw_should_stop=should_stop,
      planner_target_m=distance_to_stop_target_m,
      lead_status=lead_status,
      lead_v=lead_v,
      lead_d_rel=lead_d_rel_eff,
      increased_stopped_distance_m=float(increased_stopped_distance),
      brake_pressed=CS.brakePressed,
      cruise_standstill=CS.cruiseState.standstill,
      standstill=bool(getattr(CS, "standstill", False)),
      force_coast=force_coast,
      long_control_state=int(self.long_control_state),
      last_output_accel=self.last_output_accel,
      dt=DT_CTRL,
      human_acceleration=bool(frogpilot_toggles.human_acceleration),
      v_ego_starting=float(frogpilot_toggles.vEgoStarting),
    )

    new_control_state = long_control_state_trans(self.CP, active, self.long_control_state, CS.vEgo,
                                                 decision.state_should_stop, CS.brakePressed,
                                                 CS.cruiseState.standstill, frogpilot_toggles,
                                                 a_target=a_target,
                                                 distance_to_stop_target_m=decision.target_distance_m)
    if (
      stopping_flags.SERVICE_MODE == "LIVE"
      and self._service_shadow_scope
      and not self._service_live_disabled
      and self.long_control_state == LongCtrlState.stopping
      and new_control_state != LongCtrlState.stopping
      and service_holds_stopping_state(self._service_shadow_svc.phase)
    ):
      # Stage-3 service owns the settled stop. Do not let ANY legacy transition reason escape
      # through `starting` underneath an active settled phase (00001efe/59 escaped without either
      # named release boolean). A genuine departure or planner go moves the service through its
      # jerk-limited RELEASE; the state machine may leave stopping once that ramp completes and the
      # service becomes INACTIVE.
      new_control_state = LongCtrlState.stopping
    if (
      self.long_control_state == LongCtrlState.stopping
      and new_control_state != LongCtrlState.stopping
      and decision.state_dropout_hold
    ):
      new_control_state = LongCtrlState.stopping
    entered_stopping = self.long_control_state != LongCtrlState.stopping and new_control_state == LongCtrlState.stopping
    self.long_control_state = new_control_state
    if self.long_control_state != LongCtrlState.stopping:
      # drop the close-gap creep latch outside stopping so it can only re-arm from a fresh standstill
      self.creeping = False
      self.close_gap_creep_standstill_time_s = 0.0

    # Stopping Service V3 stage-2/3 cap bypass (plan §6 stages 2-3): while the service owned the
    # wire on the PREVIOUS frame (and is offered ownership again below), the legacy over-brake cap
    # family must not fight it -- low_speed_close_lead_accel_cap, the far_stopped_lead trio and
    # low_speed_stopped_lead_glide_accel_cap are bypassed for the frame, and in stage-3 LIVE the
    # pid-band caps' pid.i side-effects (brake-model alignment C4 + the stopped/slowing-lead
    # approach caps C5) are gated off on owned pid frames too (their wire effect is overridden by
    # the takeover anyway; only their integrator mutation could leak into the handback). All
    # flag-gated CONDITIONS only; the cap code stays intact, SHADOW/OFF byte-identical. Keying on
    # previous-frame ownership keeps the takeover frame itself fully legacy-capped, so the service's
    # jerk-consistent entry seed is the true capped wire. Symmetric accepted cost: on an
    # ownership-LOSS frame (handback/self-release/exception) the legacy fallback for that ONE frame
    # was computed with the caps bypassed -- shallower than a capped-legacy pin, re-pinned at
    # brake_step rate from the next frame (continuity over an instant re-pin; the anti-collision
    # lanes below stay live). Deliberately NOT bypassed: the seg24
    # planner floor (deepen-only min(); the service min()s a_plan internally anyway) and the
    # force-coast -0.32 standstill hold (deepen-only, applied after the takeover).
    service_caps_bypassed = (
      self._service_live_owning
      and not self._service_live_disabled
      and self._service_shadow_scope
      and (
        (stopping_flags.SERVICE_MODE == "LIVE_TERMINAL" and self.long_control_state == LongCtrlState.stopping)
        # stage 3 (plan §6): the service owns every frame it reports active in BOTH pid and
        # stopping states, so the bypass keys on the same two states.
        or (stopping_flags.SERVICE_MODE == "LIVE"
            and self.long_control_state in (LongCtrlState.pid, LongCtrlState.stopping))
      )
    )

    standstill_recent = self.arbiter.time_since_standstill_s < 0.5
    stop_intent_recent = self.arbiter.projected_time_since_stop_intent_s(decision, int(self.long_control_state), DT_CTRL) < 1.0
    stop_intent_active = (decision.stop_request_active or decision.approach_cap_active or decision.carry_floor_active
                          or self.long_control_state == LongCtrlState.stopping)

    if self.long_control_state == LongCtrlState.off or not stop_intent_active:
      # legacy :902-903 -- resets ONLY the stopping controller, never the arbiter (its dropout-
      # hold timers must survive intent loss mid-window; arbiter.reset() lives in self.reset()).
      self.stopping_controller.reset()

    stopping_shadow_debug: dict[str, object] | None = None

    if self.long_control_state == LongCtrlState.off:
      self.reset()
      output_accel = 0.

    elif self.long_control_state == LongCtrlState.stopping:
      handoff_soften_cap: float | None = None

      output_accel = min(output_accel, -0.1)
      if entered_stopping and should_apply_stop_entry_handoff_soften(CS.vEgo, CS.aEgo, a_target, self.last_output_accel, decision.target_distance_m):
        handoff_soften_cap = stop_entry_handoff_accel_cap(CS.vEgo, decision.target_distance_m)
        output_accel = max(output_accel, handoff_soften_cap)

      max_expected_accel = interp(CS.vEgo, STOPPING_V_BP, STOPPING_ACCEL_MAX)
      min_expected_accel = interp(CS.vEgo, STOPPING_V_BP, STOPPING_ACCEL_MIN)

      stopping_shadow_debug = self._new_stopping_shadow_debug_if_due("stopping")

      stop_result_kwargs = dict(
        output_accel=output_accel,
        last_output_accel=max(self.last_output_accel, handoff_soften_cap) if handoff_soften_cap is not None else self.last_output_accel,
        should_stop=decision.stop_request_active,
        v_ego=CS.vEgo,
        a_ego=CS.aEgo,
        max_expected_accel=max_expected_accel,
        min_expected_accel=min_expected_accel,
        distance_to_stop_target_m=decision.target_distance_m,
        raw_should_stop=should_stop,
        stop_accel=self.CP.stopAccel,
        dt=DT_CTRL,
        lead_status=lead_status,
        lead_v=lead_v,
        debug=stopping_shadow_debug,
      )
      # lead_d_rel is passed as the §4.2.4 effective distance directly in the call expression (not
      # via the kwargs dict). The V2 facade REQUIRES the longcontrol-arbiter decision (F2) and reads
      # the gap from the effective distance.
      stop_result = self.stopping_controller.update(**stop_result_kwargs, lead_d_rel=lead_d_rel_eff, decision=decision)
      output_accel = stop_result.output_accel
      release_lock_active = stop_result.release_lock_active
      # Low-speed close-lead over-brake cap. V-GATED bypass under the terminal-glide profile (sub-0.30
      # redesign): above STOPPING_PLANNER_FLOOR_V_EGO_MIN (0.30) the should_apply_* gate retires this
      # cap (the tracker glides to 4.0 m and the seg24 planner floor owns anti-collision); at
      # v_ego <= 0.30 the cap stays active EXACTLY as legacy, so the sub-0.30 anti-collision authority
      # on any closing error is byte-identical to today -- no new under-brake hole by construction.
      # stage-2 LIVE_TERMINAL: bypassed (condition only) while the service owns the wire
      if should_apply_low_speed_close_lead_accel_cap(self.CP, CS.vEgo) and lead_status and not service_caps_bypassed:
        close_lead_cap = low_speed_close_lead_accel_cap(CS.vEgo, lead_v, lead_d_rel_eff)
        if close_lead_cap is not None and output_accel > close_lead_cap:
          close_lead_brake_step = low_speed_close_lead_brake_step(CS.vEgo, lead_d_rel_eff)
          output_accel = max(close_lead_cap, output_accel - close_lead_brake_step)

      # Stopping-phase planner-aTarget safety floor (incident 0000173c seg24 -- see the module-level
      # note). One-way DEEPEN ONLY: when a close lead is present and the planner is still demanding
      # decel deeper than the stopping controller's command, honor the planner so the car does not
      # coast through its stop point into the lead (the bookmark: at v~1.25 m/s with ~2.2 m to a
      # stopped lead the planner asked -0.52 but the legacy terminal lane gave only -0.12 -> coast-in
      # to 1.36 m on flat ground). Applied here, after the controller + close-lead cap, so it sees the
      # deepest legacy command; min() can only make the command more negative, never less, so it
      # cannot under-brake. Gated off below the standstill band so it never fights the gentle final
      # hold. (Grade-agnostic by construction: the planner aTarget already reflects whatever net decel
      # the situation needs, so honoring it covers flat and graded approaches alike.)
      if (
        STOPPING_PLANNER_FLOOR_ENABLED
        and should_apply_stopping_planner_floor(self.CP)
        and stopping_planner_floor_active(CS.vEgo, lead_status, lead_v, lead_d_rel_eff, a_target, output_accel)
      ):
        # One-way DEEPEN to the planner demand: min() can only make the command MORE negative, never
        # less, so on the frame it is applied it can never under-brake the controller's command
        # (INVARIANT 1, structural). It is bounded by a_target (never deeper than the planner asks),
        # which is itself bounded and runs through the downstream stop_accel clip, so it cannot run
        # away (INVARIANT 3). The deepened command is carried forward through last_output_accel so the
        # car stays braked as it crosses into the standstill band (where the gate disarms and the
        # controller's deep terminal hold/glide lanes own the command) -- it never snaps back to the
        # shallow coast-in glide at the closest, slowest moment.
        output_accel = min(output_accel, a_target)

    elif self.long_control_state == LongCtrlState.starting:
      output_accel = (a_target if human_acceleration_active else frogpilot_toggles.startAccel)
      if human_acceleration_active and decision.departing_lead_ready:
        lead_departure_speed = max(float(lead_v) - float(CS.vEgo), 0.0)
        departing_lead_accel_floor = interp(lead_departure_speed, [0.60, 1.20, 2.00, 3.00], [0.12, 0.22, 0.35, 0.45])
        output_accel = max(output_accel, min(float(frogpilot_toggles.startAccel), departing_lead_accel_floor))
      self.reset()

    else:  # LongCtrlState.pid
      error = a_target - CS.aEgo
      # LongitudinalActiveWithGas keeps this loop active during a driver gas override. Freeze its
      # integrator until the driver releases the pedal so the handback does not include windup.
      # stage-3 LIVE: while the service owned the wire on the previous frame (service_caps_bypassed
      # covers pid-state ownership only in LIVE mode) the pid error is measured against the SERVICE
      # trajectory, not the pid's own -- freeze the integrator so it cannot wind up against a wire
      # it is not driving. Together with the owned-frame pid.i reseed in the takeover block below,
      # this guarantees the handback frame resumes from the service command with no integrator step
      # (the reseed sets pid.i each owned frame; the freeze keeps it there through the handback
      # frame itself). LIVE_TERMINAL/SHADOW/OFF: service_caps_bypassed is False in the pid state,
      # so this term is inert -- byte-identical legacy behavior.
      pid_freeze_integrator = freeze_integrator or decision.approach_cap_active or decision.carry_floor_active or service_caps_bypassed
      output_accel = self.pid.update(error, speed=CS.vEgo,
                                     feedforward=a_target,
                                     freeze_integrator=pid_freeze_integrator)
      integrator_enabled = pid_integrator_enabled(self.pid)
      if not integrator_enabled:
        self.pid.i = 0.0
      self._trim_pid_untrimmed = float(output_accel)  # cycle-32: the UNTRIMMED pid demand (trim reference)
      if decision.approach_cap_active:
        output_accel = min(output_accel, stop_target_approach_accel_cap(CS.vEgo, decision.target_distance_m))
      if decision.carry_floor_active:
        output_accel = max(output_accel, stop_target_carry_accel_floor(CS.vEgo, decision.target_distance_m))

    if self.long_control_state != LongCtrlState.off:
      allow_fast_release = (
        not force_coast
        and not decision.stop_request_active and not decision.approach_cap_active
        and self.long_control_state in (LongCtrlState.pid, LongCtrlState.starting)
        and a_target > 0.2
        and CS.vEgo > 0.12
      )
      if decision.departing_lead_release and not force_coast:
        allow_fast_release = True
      if decision.departing_lead_ready and self.long_control_state == LongCtrlState.starting and not force_coast:
        allow_fast_release = True
      if stop_intent_recent and not standstill_recent:
        allow_fast_release = False
      # §6.4 slew-exemption: the V2 tracker jerk-limits internally on EVERY stopping frame, so the
      # global guard is exempted for the whole stopping state.
      apply_global_low_speed_slew = self.long_control_state != LongCtrlState.stopping
      if apply_global_low_speed_slew:
        output_accel = apply_low_speed_output_slew(
          output_accel=output_accel,
          last_output_accel=self.last_output_accel,
          should_stop=(decision.stop_request_active or decision.approach_cap_active or decision.carry_floor_active),
          v_ego=CS.vEgo,
          a_ego=CS.aEgo,
          max_expected_accel=max_expected_accel,
          allow_fast_release=allow_fast_release,
          release_lock_active=release_lock_active,
        )
      # stage-2 LIVE_TERMINAL: the far_stopped_lead trio is bypassed while the service owns the wire
      if decision.far_stopped_lead_release and not service_caps_bypassed:
        output_accel = min(output_accel, far_stopped_lead_crawl_accel_cap(CS.vEgo, lead_d_rel_eff))
        if should_stop or stop_intent_recent:
          settle_cap = far_stopped_lead_settle_accel_cap(CS.vEgo, lead_d_rel_eff, decision.target_distance_m)
          if settle_cap is not None:
            settle_release_step = interp(CS.vEgo, [0.03, 0.20, 0.55], [0.006, 0.008, 0.011])
            output_accel = min(output_accel, settle_cap, self.last_output_accel + settle_release_step)
        far_lead_brake_floor = far_stopped_lead_brake_floor(CS.vEgo, lead_d_rel_eff)
        if output_accel < far_lead_brake_floor:
          far_lead_release_step = interp(CS.vEgo, [0.00, 0.20, 0.55], [0.028, 0.024, 0.018])
          output_accel = min(far_lead_brake_floor, max(output_accel, self.last_output_accel + far_lead_release_step))

      # TERMINAL-GLIDE PROFILE (sub-0.30 redesign): V-GATE the binding glide over-brake cap bypass.
      # The over-brake that CAUSES the leapfrog (near-stop short, lead leapfrogs) lives ABOVE 0.30 m/s,
      # so bypass the cap ONLY when the flag is on AND v_ego > STOPPING_PLANNER_FLOOR_V_EGO_MIN (0.30):
      # there the jerk-limited tracker glides to 4.0 m and the seg24 STOPPING_PLANNER_FLOOR owns
      # anti-collision. At v_ego <= 0.30 the cap stays ACTIVE exactly as legacy (byte-identical sub-0.30
      # authority), so there is no new under-brake hole by construction. The shared
      # should_apply_low_speed_stopped_lead_glide_accel_cap predicate ALSO gates the arbiter's synthetic
      # stopped-lead target (its _quirk_layer_enabled), so it must NOT be flipped; the v-gated bypass is
      # applied here, at the cap's sole application site, leaving the synthetic target intact.
      terminal_glide_bypass_glide_cap = (
        stopping_flags.SANTA_FE_TERMINAL_GLIDE_PROFILE_ENABLED and CS.vEgo > STOPPING_PLANNER_FLOOR_V_EGO_MIN
      )
      stopped_lead_glide_cap = (
        low_speed_stopped_lead_glide_accel_cap(CS.vEgo, lead_v, lead_d_rel_eff, decision.target_distance_m)
        if (
          not terminal_glide_bypass_glide_cap
          and not service_caps_bypassed  # stage-2 LIVE_TERMINAL: bypassed while the service owns the wire
          and should_apply_low_speed_stopped_lead_glide_accel_cap(self.CP)
          and lead_status
          and (decision.stop_request_active or decision.approach_cap_active or decision.carry_floor_active
               or self.long_control_state == LongCtrlState.stopping)
        )
        else None
      )
      if stopped_lead_glide_cap is not None and output_accel > stopped_lead_glide_cap:
        stopped_lead_brake_step = interp(CS.vEgo, [0.02, 0.20, 0.35, 0.65, 0.95, 1.25], [0.004, 0.004, 0.004, 0.006, 0.008, 0.010])
        output_accel = max(stopped_lead_glide_cap, min(output_accel, self.last_output_accel) - stopped_lead_brake_step)

      if (
        should_apply_pid_brake_model_alignment(self.CP)
        and self.long_control_state == LongCtrlState.pid
        and not service_caps_bypassed  # stage-3 LIVE: no pid.i mutation on service-owned frames (wire is overridden anyway)
        and not decision.stop_request_active
        and not decision.approach_cap_active
        and not decision.far_stopped_lead_release
      ):
        aligned_output = apply_pid_brake_model_alignment(output_accel, a_target, CS.aEgo, CS.vEgo)
        if aligned_output > output_accel:
          if integrator_enabled:
            self.pid.i = max(self.pid.i, aligned_output - (self.pid.p + self.pid.d + self.pid.f))
          output_accel = aligned_output
      if (
        PID_STOPPED_LEAD_APPROACH_ENABLED
        and should_apply_pid_stopped_lead_approach_accel_cap(self.CP)
        and self.long_control_state == LongCtrlState.pid
        and not service_caps_bypassed  # stage-3 LIVE: no pid.i mutation on service-owned frames (wire is overridden anyway)
        and not decision.stop_request_active
        and not decision.approach_cap_active
        and not decision.carry_floor_active
        and not decision.far_stopped_lead_release
        and lead_status
      ):
        # High-closing-speed queue approach: add only enough early PID braking to spend less of the
        # lead gap before the planner/stopping handoff. The decelerating-lead path uses aLeadK to cover
        # leads that are not slow enough to count as stopped yet but are clearly becoming a queue stop.
        # Reads raw radar distance like the moving-lead PID cap; the stopped-distance UI offset is a
        # terminal-following concern, not approach range.
        stopped_lead_approach_cap = pid_stopped_lead_approach_accel_cap(CS.vEgo, lead_v, lead_d_rel)
        slowing_lead_approach_cap = pid_slowing_lead_approach_accel_cap(CS.vEgo, lead_v, lead_d_rel, lead_a)
        if slowing_lead_approach_cap is not None:
          stopped_lead_approach_cap = (
            slowing_lead_approach_cap
            if stopped_lead_approach_cap is None
            else min(stopped_lead_approach_cap, slowing_lead_approach_cap)
          )
        if stopped_lead_approach_cap is not None and output_accel > stopped_lead_approach_cap:
          stopped_lead_approach_step = pid_stopped_lead_approach_brake_step(CS.vEgo)
          output_accel = max(stopped_lead_approach_cap, min(output_accel, self.last_output_accel) - stopped_lead_approach_step)
          if integrator_enabled:
            self.pid.i = min(self.pid.i, output_accel - (self.pid.p + self.pid.d + self.pid.f))
      force_coast_no_target_pid_active = (
        should_apply_force_coast_no_target_pid_brake_cap(self.CP)
        and force_coast
        and self.long_control_state == LongCtrlState.pid
        and not decision.stop_request_active
        and not decision.approach_cap_active
        and not decision.carry_floor_active
        and not lead_status
        and decision.target_distance_m < 0.0
      )
      if force_coast_no_target_pid_active:
        force_coast_target_accel = get_force_coast_target_from_toggles(CS.vEgo, frogpilot_toggles)
        if not self.force_coast_ramp_active:
          # Start from the command already on the wire so enabling Force Coast cannot introduce a
          # deceleration step. A pre-existing command below the selected target is capped immediately.
          self.force_coast_ramp_active = True
          self.force_coast_ramp_elapsed_s = 0.0
          self.force_coast_ramp_start_accel = max(float(self.last_output_accel), force_coast_target_accel)
        output_accel = get_force_coast_ramped_accel(
          self.force_coast_ramp_start_accel,
          force_coast_target_accel,
          self.force_coast_ramp_elapsed_s,
        )
        self.force_coast_ramp_elapsed_s = min(self.force_coast_ramp_elapsed_s + DT_CTRL, FORCE_COAST_RAMP_IN_S)
        if integrator_enabled:
          self.pid.i = output_accel - (self.pid.p + self.pid.d + self.pid.f)
      else:
        self.force_coast_ramp_active = False
        self.force_coast_ramp_elapsed_s = 0.0
        self.force_coast_ramp_start_accel = 0.0
      if (
        should_apply_experimental_close_lead_accel_cap(self.CP, experimental_mode)
        and self.long_control_state == LongCtrlState.pid
        and not decision.stop_request_active
        and not decision.approach_cap_active
        and not decision.carry_floor_active
        and lead_status
      ):
        # moving-lead consumer: deliberately reads the RAW published lead distance (§4.2.4)
        close_lead_cap = experimental_close_lead_accel_cap(CS.vEgo, lead_v, lead_d_rel)
        if close_lead_cap is not None and output_accel > close_lead_cap:
          output_accel = apply_experimental_close_lead_accel_cap(output_accel, close_lead_cap)
          if integrator_enabled:
            self.pid.i = min(self.pid.i, output_accel - (self.pid.p + self.pid.d + self.pid.f))

      # Close-the-gap forward creep behind a confirmed STOPPED lead (route 00001764 seg27). THE LAST
      # writer of the stopping-state output_accel (after every cap above, before the force_coast hold
      # below), so the gentle glide BRAKE cap can never clobber it and last_output_accel ratchets to the
      # creep's OWN value (letting the positive slew actually climb). POSITIVE-ONLY/ONE-WAY: there is no
      # lower-bound relax lane. Latched (self.creeping) with hysteresis to avoid the v<=0.06 re-arm
      # oscillation; armed only after a stable standstill behind a confirmed stopped lead with the eff
      # gap clearly above target; disarmed by GAP (ISD-aware hard floor), lead-departure, force_coast,
      # or overspeed.
      if (
        STOPPING_CLOSE_GAP_CREEP_ENABLED
        and should_apply_stopping_close_gap_creep(self.CP)
        and self.long_control_state == LongCtrlState.stopping
      ):
        if bool(getattr(CS, "standstill", False)) and abs(float(CS.aEgo)) <= CREEP_ARM_A_EGO_ABS_MAX:
          self.close_gap_creep_standstill_time_s += DT_CTRL
        else:
          self.close_gap_creep_standstill_time_s = 0.0
        creep_rest_target_m = stopping_close_gap_creep_rest_target_m(increased_stopped_distance)
        if not self.creeping:
          if (
            self.close_gap_creep_standstill_time_s >= CREEP_ARM_STANDSTILL_TIME_S
            and stopping_close_gap_creep_should_arm(CS.vEgo, lead_status, lead_v, lead_d_rel_eff,
                                                   force_coast, creep_rest_target_m, increased_stopped_distance)
          ):
            self.creeping = True
        elif stopping_close_gap_creep_should_disarm(CS.vEgo, lead_status, lead_v, lead_d_rel_eff,
                                                    force_coast, creep_rest_target_m, increased_stopped_distance):
          self.creeping = False
        if self.creeping:
          creep_accel_target = stopping_close_gap_creep_accel_target(CS.vEgo, lead_d_rel_eff)
          # slew from last_output_accel (holds the creep's own value once creep is the last writer)
          creep_cmd = min(creep_accel_target, self.last_output_accel + CREEP_SLEW_UP)
          creep_cmd = max(creep_cmd, self.last_output_accel - CREEP_SLEW_DOWN)
          output_accel = float(clip(creep_cmd, -1.0, CREEP_ACCEL_MAX))

    # --- cycle-32 tracking TRIM (Santa Fe HEV; see the SANTA_FE_TRIM_* block) ----------------------
    # AFTER the cap family (every pid.i reconstruction above saw the untrimmed value; a frame any cap
    # rewrote the pid demand is not a learning frame) and BEFORE the service takeover (the service
    # seeds its limiter from the trimmed wire, so takeover is continuous; once owned the trim decays).
    if self._trim_scope and stopping_flags.SANTA_FE_ACCEL_TRACKING_TRIM:
      trim_in_pid = self.long_control_state == LongCtrlState.pid
      trim_untrimmed = self._trim_pid_untrimmed if trim_in_pid else None
      trim_cap_written = (trim_untrimmed is None
                          or abs(float(output_accel) - trim_untrimmed) > 1e-6
                          or float(output_accel) <= float(accel_limits[0]) + 1e-6)
      # model reference: the untrimmed demand TAU ago, passed through the plant's first-order lag, so
      # a brake ONSET is not read as shortfall; learning needs TAU of clean (pid-is-the-wire) frames
      trim_ref_raw = self._trim_ref[0] if len(self._trim_ref) == SANTA_FE_TRIM_TAU_FRAMES else None
      if trim_cap_written:
        self._trim_ref_filt = None              # the plant followed something else: re-seed at the plant
      if trim_ref_raw is None:
        self._trim_ref_filt = None
      elif self._trim_ref_filt is None:
        # seed at the PLANT's current state (not at the demand: that read every onset as shortfall)
        self._trim_ref_filt = float(CS.aEgo) if math.isfinite(float(CS.aEgo)) else float(trim_ref_raw)
      trim_ref_prev = self._trim_ref_filt
      if trim_ref_raw is not None and trim_ref_prev is not None:
        self._trim_ref_filt = trim_ref_prev + (float(trim_ref_raw) - trim_ref_prev) * (DT_CTRL / SANTA_FE_TRIM_LAG_S)
      trim_ref_rate = ((self._trim_ref_filt - trim_ref_prev) / DT_CTRL
                       if (self._trim_ref_filt is not None and trim_ref_prev is not None) else 0.0)
      self._trim_clean = 0 if trim_cap_written else self._trim_clean + 1
      if self._service_live_owning:
        # The service wrote the wire on the previous frame: the trim's approach job is over. Zero the
        # STATE (no wire effect -- the service writes the wire) and add nothing to the legacy value, so
        # no residual can return as a step through the service-exception fallback (min(legacy, last)
        # on a previously-owned frame) or a quick handback (R1 HIGH). The takeover frame itself ran
        # with the trim applied, so the service seed was continuous.
        self._trim_i = 0.0
      else:
        trim_learn_ok = (bool(active) and trim_in_pid and not freeze_integrator and not trim_cap_written
                         and self._trim_clean >= SANTA_FE_TRIM_TAU_FRAMES
                         and float(a_target) <= SANTA_FE_TRIM_A_ARM
                         and SANTA_FE_TRIM_V_MIN <= float(CS.vEgo) <= SANTA_FE_TRIM_V_MAX)
        self._trim_i = update_santa_fe_tracking_trim(self._trim_i, self._trim_ref_filt, float(CS.aEgo), trim_learn_ok,
                                                    ref_rate=trim_ref_rate)
        # (disengagement zeroes the trim through reset() in the off state -- ONE path, by design)
        # anti-windup against the planner limit: never hold trim the final clip would not send
        self._trim_i = min(0.0, max(self._trim_i, float(accel_limits[0]) - float(output_accel)))
        output_accel = float(output_accel) + self._trim_i
      self._trim_ref.append(trim_untrimmed)   # None outside the pid state (the filter restarts)
    else:
      self._trim_i = 0.0
      self._trim_ref.clear()
      self._trim_ref_filt = None
      self._trim_clean = 0

    # --- Stopping Service V3 stage-2/3 takeover (plan §6 stages 2-3) ------------------------------
    # Stage 2 (LIVE_TERMINAL): the service becomes the LAST writer of the stopping-state wire for
    # v <= 0.85 m/s (handback hysteresis: release only above 0.95 or on stopping-state exit).
    # Stage 3 (LIVE): the full stop-intent band -- the service owns the wire on EVERY frame it
    # reports active, in BOTH the pid and stopping states; its own entry conditions (v < 2.5 AND
    # (shouldStop OR the lead-stopped latch with d_rem < 15)) and its own RELEASE/exit (planner go,
    # or the band exit hysteresis via RELEASE ramping to 0 at J_GO) are the sole ownership
    # authority. This removes the stage-2 0.85 seam: route 00001b72's planner one-frame aTarget
    # slam reached the wire through the PID state at v 0.92 because the stopping state only engaged
    # at v 0.15. Santa Fe HEV fingerprint only in both stages.
    # Placement: AFTER the full legacy chain above (output_accel here is the legacy-would-have
    # reference, fully computed every frame) and BEFORE the force-coast standstill hold (a
    # deepen-only min() that may only DEEPEN the service command) and the final clip.
    # OBSERVATION over the full stage-1 band: the service+context run on every in-band frame
    # (v < 2.5 or stopping) exactly as SHADOW does, so a_coast's 1 s EMA, the gap persistence
    # filter, the 0.3 s lead-confirmed-stopped buffer and the 0.85-2.5 divergence telemetry are all
    # WARM before the own band is reached (the configuration the stage-1 shadow evidence and the
    # offline gate actually validated) -- but the wire is written ONLY in the own band below.
    # Jerk-consistent takeover: on the first owned frame the WARM service re-anchors its jerk
    # limiter on the live chain value (reseed_takeover; a cold service seeds itself at entry from
    # wire_accel), so the wire moves from the ACTUATED trajectory by no more than the service's own
    # jerk limits. Handback keeps the service observing (any re-takeover re-anchors the same way,
    # and the monitor ratchet survives a mid-settle rollaway past 0.95); the legacy chain resumes
    # from last_output_accel, which the service wrote, so continuity is automatic.
    # Robustness (P1): a LIVE exception can never disarm into no-brake -- output_accel keeps the
    # legacy chain value computed THIS frame (on a previously-owned frame that chain ran with the
    # legacy cap family bypassed, so for that ONE frame it can sit shallower than a capped-legacy
    # pin -- the caps re-pin at brake_step rate from the next frame, and the unbypassed seg24
    # planner floor + the force-coast hold below stay live), the failure is logged ONCE, and
    # ownership latches OFF for the rest of the drive (the legacy chain, still computed every
    # frame, keeps the wire).
    service_mode = stopping_flags.SERVICE_MODE
    if (self._service_shadow_scope and not self._service_live_disabled
        and service_mode in ("LIVE_TERMINAL", "LIVE")):
      service_in_band = active and (CS.vEgo < 2.5 or self.long_control_state == LongCtrlState.stopping)
      if service_mode == "LIVE":
        # stage 3: ownership is OFFERED on every active pid/stopping frame; whether the service
        # actually takes the wire is decided solely by its own entry/RELEASE state (result.active
        # below). Handback = the service going inactive (its RELEASE ramp finished, or out of band):
        # the legacy chain -- fully computed above -- resumes from last_output_accel, which the
        # service wrote, plus the reseeded pid integrator (one authority per frame either way).
        service_own_band = active and self.long_control_state in (LongCtrlState.pid, LongCtrlState.stopping)
      else:
        service_own_band = (
          active
          and self.long_control_state == LongCtrlState.stopping
          and (CS.vEgo <= SERVICE_LIVE_TERMINAL_V_OWN
               or (self._service_live_owning and CS.vEgo <= SERVICE_LIVE_TERMINAL_V_RELEASE))
        )
      try:
        if service_own_band and not self._service_live_owning:
          # first owned frame with a warm (observing) service: re-anchor its jerk limiter on the
          # live pre-takeover chain value -- the jerk-consistent takeover, warm-context edition
          self._service_shadow_svc.reseed_takeover(float(output_accel), accel_limits[0])
        service_result = self._run_stopping_service(
          run=service_in_band, CS=CS, a_target=a_target, a_target_trajectory=a_target_trajectory, should_stop=service_should_stop,
          distance_to_stop_target_m=distance_to_stop_target_m, accel_limits=accel_limits,
          lead_status=lead_status, lead_v=lead_v, lead_d_rel=lead_d_rel, lead_track_id=lead_track_id,
          lead_service_authorized=lead_service_authorized,
          increased_stopped_distance=increased_stopped_distance,
          wire_accel=float(output_accel),
          reference_accel=float(output_accel) if service_own_band else None)
      except Exception:
        service_result = None
        self._service_live_disabled = True
        self._service_shadow_svc.reset()
        self._service_shadow_ctx.reset()
        if self._service_live_owning:
          # Codex review 2026-07-02: on a previously-OWNED frame the chain above ran with the cap
          # family bypassed, so falling back to it raw can RELEASE the wire in one frame (probed:
          # -0.149 vs -0.561 capped-legacy at v=0.20 behind a close stopped lead). Never release on
          # the fault frame: hold the previous (service-written, properly deep) wire if it is deeper;
          # the un-bypassed caps re-pin from the next frame and the deepen-only nets stay live.
          output_accel = min(output_accel, float(self.last_output_accel))
        cloudlog.exception("stopping_service %s failed; ownership latched off for this drive (legacy chain keeps the wire)", service_mode)
      if service_own_band and service_result is not None and service_result.active:
        output_accel = float(service_result.accel)  # the service owns the wire this frame
        self._service_live_owning = True
        if self.long_control_state == LongCtrlState.pid and pid_integrator_enabled(self.pid):
          # stage-3 LIVE pid-state ownership (only LIVE offers pid-state own_band): keep the frozen
          # integrator consistent with the SERVICE command each owned frame, so at handback the
          # legacy pid resumes exactly from the service trajectory -- no windup while owned, no
          # integrator step at release beyond the C1 low-speed slew.
          self.pid.i = float(output_accel) - (self.pid.p + self.pid.d + self.pid.f)
      elif service_own_band and service_result is not None and not service_result.active and self._service_live_owning:
        # RELEASE completed inside this update. Keep the prior service-written wire for this one
        # transition frame; the still-stopping legacy chain otherwise re-pins it to -0.10 after the
        # service has reached zero. The state machine may leave stopping on the next frame.
        output_accel = float(self.last_output_accel)
        self._service_live_owning = False
      else:
        self._service_live_owning = False           # observing / handback / not entered: legacy chain keeps the wire

    if force_coast and standstill:
      # Hold FIRM at the baseline magnitude (not just <=0): the gentle V2 hold here is what the car's TCS
      # rejects on a gas tip-in out of a no-lead force-coast stop (accFaulted). min() keeps any deeper
      # command intact and only deepens the shallow hold; applied to output_accel so it reaches the wire.
      output_accel = min(output_accel, FORCE_COAST_STANDSTILL_HOLD_ACCEL)

    if stopping_shadow_debug is None and self.long_control_state == LongCtrlState.pid:
      pid_shadow_active = should_observe_pid_stopping_shadow(
        v_ego=CS.vEgo,
        a_target=a_target,
        output_accel=output_accel,
        distance_to_stop_target_m=decision.target_distance_m,
        force_coast=force_coast,
        lead_status=lead_status,
        lead_v=lead_v,
        lead_d_rel=lead_d_rel,
        stop_request_active=decision.stop_request_active,
        stop_target_approach_active=decision.approach_cap_active,
        stop_target_carry_active=decision.carry_floor_active,
      )
      if pid_shadow_active:
        stopping_shadow_debug = self._new_stopping_shadow_debug_if_due("pid_stop_intent")
        if stopping_shadow_debug is not None:
          self._populate_pid_stopping_shadow_debug(
            stopping_shadow_debug,
            CS,
            output_accel,
            decision.target_distance_m,
            lead_status,
            lead_v,
            lead_d_rel,
            release_lock_active,
          )
      else:
        self.stopping_shadow_frame = 0
    elif self.long_control_state not in (LongCtrlState.stopping,):
      self.stopping_shadow_frame = 0

    if stopping_shadow_debug is not None:
      self._log_stopping_shadow(stopping_shadow_debug, CS, output_accel, lead_status, lead_v, lead_d_rel)

    self.last_output_accel = clip(output_accel, accel_limits[0], accel_limits[1])

    # Stopping Service V3 STAGE 1 SHADOW (plan §6 stage 1): observer only, computed strictly AFTER
    # the wire value above is final; it never writes output_accel / last_output_accel. The blanket
    # except is deliberate and explicit: a defect in the shadow observer must never take down the
    # control path (the observer's whole contract is zero wire impact) -- and it DISARMS the observer
    # for the rest of the drive, so a persistent defect cannot flood cloudlog at 100 Hz either.
    if self._service_shadow_scope and not self._service_shadow_disabled and stopping_flags.SERVICE_MODE == "SHADOW":
      try:
        self._update_stopping_service_shadow(active, CS, a_target, a_target_trajectory, service_should_stop, distance_to_stop_target_m,
                                             accel_limits, lead_status, lead_v, lead_d_rel, lead_track_id,
                                             lead_service_authorized, increased_stopped_distance, float(self.last_output_accel))
      except Exception:
        self._service_shadow_disabled = True
        cloudlog.exception("stopping_service shadow observer failed; observer disarmed for this drive")

    return self.last_output_accel
