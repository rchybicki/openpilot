"""Stateless stopping-trajectory reference law (stopping redesign spec section 5.3, WP5).

``stop_reference`` maps the current kinematic state + arbitrated stop target to a desired accel
and a per-frame jerk budget. It is deliberately MEMORYLESS: the S-curve is implicit -- the
distance-feedback ``a_ref`` plus the tracker's asymmetric jerk limiter (stopping_tracker.py,
spec 5.5.5) produce a jerk-limited profile that re-converges automatically after target jumps,
dropouts, or disturbances. There is no replanning state to corrupt; dt never enters (the same
state maps to the same reference at 100 Hz and 10 Hz, pinned by test_stopping_trajectory.py).

Phases (spec section 2):
  TRACK    v > V_NEAR_HOLD: kinematic decel need, clipped to the approach envelope
  TERMINAL V_SETTLE < v <= V_NEAR_HOLD: near-hold envelope + the A_END_STOP terminal ceiling
           (the ceiling binds the QUIESCENT path only -- the tracker's push/arrest mechanisms
            may deepen past it, spec 5.3 scope / red-team F26)
  SETTLE   v <= V_SETTLE, not yet settled: ramp toward A_HOLD at J_SETTLE_RELEASE
  HOLD     settled standstill: A_HOLD, relaxing to A_HOLD_RELAXED after T_HOLD_RELAX_S

Conventions: accel m/s^2 negative = braking; jerk m/s^3 positive magnitudes; target sentinel
-1.0 = none (exactly 0.0 unreachable at runtime, spec conventions).
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum

import numpy as np

from openpilot.selfdrive.controls.lib import stopping_flags
from openpilot.selfdrive.controls.lib.stopping_params import STOPPING_PARAMS, StoppingParams

interp = np.interp

# Firm terminal-hold magnitude for the Santa-Fe terminal-glide profile (correction 2).
# MUST equal FORCE_COAST_STANDSTILL_HOLD_ACCEL (longcontrol.py:65) -- the already-proven HEV
# creep-counter magnitude; the equality is pinned by test_longcontrol_fast_release.py so the two
# cannot silently drift. The gentle A_HOLD (-0.16..-0.10) is overpowered by HEV clutch/TC creep torque,
# which walks the car forward off the intended 4.0 m rest; this firm value holds it. The deepen from
# the inherited end-stop accel toward this firmer value eases in at the tracker's J_BRAKE rate (the
# deepen direction; J_SETTLE_RELEASE bounds only the shallowing direction). Gated on
# SANTA_FE_TERMINAL_GLIDE_PROFILE_ENABLED.
A_HOLD_FIRM = -0.32


class TrajPhase(IntEnum):
  TRACK = 0      # v > V_NEAR_HOLD: distance-feedback decel profile
  TERMINAL = 1   # V_SETTLE < v <= V_NEAR_HOLD: blend toward end-stop accel
  SETTLE = 2     # v <= V_SETTLE: ramp from end-stop accel to hold accel
  HOLD = 3       # standstill hold (relaxes after T_HOLD_RELAX_S)


@dataclass(frozen=True)
class StopReference:
  a_ref: float           # m/s^2 desired accel this frame (<= -0.05 in stopping authority)
  phase: TrajPhase
  j_brake_max: float     # m/s^3 allowed deepening rate this frame
  j_release_max: float   # m/s^3 allowed release rate this frame
  remaining_m: float     # arbitrated/kinematic remaining distance used


def remaining_distance_m(*, v_ego: float, a_ego: float, target_distance_m: float,
                         p: StoppingParams = STOPPING_PARAMS) -> float:
  """Spec 5.3 / G14 (param #29): explicit target clipped to [0.05, EXPLICIT_REMAINING_CLIP_M],
  else the kinematic fallback v^2 / (2 * max(KINEMATIC_MIN_DECEL, -a_ego)) clipped to
  [0, KINEMATIC_REMAINING_CLIP_M]."""
  if target_distance_m is not None and target_distance_m > 0.0:
    return float(min(max(float(target_distance_m), 0.05), p.EXPLICIT_REMAINING_CLIP_M))
  decel_mag = max(p.KINEMATIC_MIN_DECEL, -float(a_ego))
  return float(min(max((float(v_ego) ** 2) / (2.0 * decel_mag), 0.0), p.KINEMATIC_REMAINING_CLIP_M))


def end_stop_ceiling(v_ego: float, p: StoppingParams = STOPPING_PARAMS) -> float:
  """A_END_STOP_TABLE(v) -- the binding terminal ceiling over its FULL 0-0.60 m/s domain
  (param #13 note: never cap only below 0.15 m/s). Quiescent-path bound only (spec 5.3)."""
  return float(interp(v_ego, p.A_END_STOP_TABLE[0], p.A_END_STOP_TABLE[1]))


def stop_reference(*, v_ego: float, a_ego: float, target_distance_m: float,
                   settled_time_s: float, rollout_m: float,
                   p: StoppingParams = STOPPING_PARAMS,
                   terminal_glide_firm_hold: bool = False) -> StopReference:
  v = max(float(v_ego), 0.0)
  d = remaining_distance_m(v_ego=v, a_ego=a_ego, target_distance_m=target_distance_m, p=p)
  d_eff = max(d - 0.05, 0.10)
  a_need = -(v * v) / (2.0 * d_eff)

  if v > p.V_NEAR_HOLD:
    phase = TrajPhase.TRACK
    approach_floor = float(interp(v, p.A_APPROACH_FLOOR_TABLE[0], p.A_APPROACH_FLOOR_TABLE[1]))
    a_ref = float(np.clip(a_need, approach_floor, -0.05))
    # rollout beyond the recovery arm threshold deepens the reference to the desired
    # low-speed decel (G9 spirit: rollout debt means the plan is behind -- brake at least
    # A_DESIRED_LOWSPEED(v) until the tracker's recovery integrator takes over below 0.85)
    if rollout_m > float(interp(v, p.RECOVERY_ARM_TABLE[0], p.RECOVERY_ARM_TABLE[1])):
      a_ref = min(a_ref, float(interp(v, p.A_DESIRED_LOWSPEED_TABLE[0], p.A_DESIRED_LOWSPEED_TABLE[1])))
    j_brake = float(interp(v, p.J_BRAKE_TABLE[0], p.J_BRAKE_TABLE[1]))
    j_release = float(interp(v, p.J_RELEASE_TABLE[0], p.J_RELEASE_TABLE[1]))
  elif v > p.V_SETTLE:
    phase = TrajPhase.TERMINAL
    desired_low_speed = float(interp(v, p.A_DESIRED_LOWSPEED_TABLE[0], p.A_DESIRED_LOWSPEED_TABLE[1]))
    near_hold = float(interp(v, p.A_NEAR_HOLD_TABLE[0], p.A_NEAR_HOLD_TABLE[1]))
    a_ref = float(np.clip(a_need, desired_low_speed, near_hold))
    # terminal ceiling: as the wheels approach stop, commanded brake is capped at the calibrated
    # no-jolt magnitude regardless of distance error (rollout recovery extends braking DURATION,
    # never terminal magnitude -- the tracker re-applies this ceiling after recovery, spec 5.5.5)
    a_ref = max(a_ref, end_stop_ceiling(v, p))
    j_brake = float(interp(v, p.J_BRAKE_TABLE[0], p.J_BRAKE_TABLE[1]))
    j_release = float(interp(v, p.J_RELEASE_TABLE[0], p.J_RELEASE_TABLE[1]))
  else:
    settled = settled_time_s > 0.0
    hold = float(interp(v, p.A_HOLD_TABLE[0], p.A_HOLD_TABLE[1]))
    # FIRM TERMINAL HOLD (correction 2): the gentle A_HOLD/A_HOLD_RELAXED targets are overpowered by
    # HEV creep torque, which walks the car forward off the 4.0 m rest. With the terminal-glide
    # profile on, hold FIRM at A_HOLD_FIRM (== FORCE_COAST_STANDSTILL_HOLD_ACCEL). min() can only
    # deepen, never make the hold shallower; the existing J_SETTLE_RELEASE eases it in. SCOPING
    # (adversarial verify MAJOR 2): the firm magnitude is a Santa-Fe-HEV creep-torque counter, so it
    # is gated on the caller-threaded terminal_glide_firm_hold (true only for the Santa-Fe fingerprint
    # AND the kill switch -- stopping_controller_v2.py). The module kill switch alone is NOT enough,
    # so other vehicles keep the gentle A_HOLD and are bit-unaffected.
    firm_hold = stopping_flags.SANTA_FE_TERMINAL_GLIDE_PROFILE_ENABLED and terminal_glide_firm_hold
    if firm_hold:
      hold = min(hold, A_HOLD_FIRM)
    if settled:
      phase = TrajPhase.HOLD
      if settled_time_s >= p.T_HOLD_RELAX_S:
        a_ref = float(interp(v, p.A_HOLD_RELAXED_TABLE[0], p.A_HOLD_RELAXED_TABLE[1]))
        if firm_hold:
          a_ref = min(a_ref, A_HOLD_FIRM)
      else:
        a_ref = hold
    else:
      phase = TrajPhase.SETTLE
      # the ramp from the inherited end-stop accel toward the hold target is realized by the
      # tracker's slew at J_SETTLE_RELEASE; the reference is the ramp endpoint
      a_ref = hold
    j_brake = float(interp(v, p.J_BRAKE_TABLE[0], p.J_BRAKE_TABLE[1]))
    j_release = p.J_SETTLE_RELEASE

  return StopReference(a_ref=min(a_ref, -0.05), phase=phase,
                       j_brake_max=j_brake, j_release_max=j_release, remaining_m=d)
