#!/usr/bin/env python3
import numpy as np

from openpilot.selfdrive.controls.lib import stopping_flags

FORCE_COAST_ACCEL_MIN = -3.5
FORCE_COAST_HIGH_SPEED_MIN_ACCEL = -1.2
FORCE_COAST_NEAR_STOP_MIN_ACCEL = -0.7
FORCE_COAST_RAMP_IN_S = 1.5
FORCE_COAST_STRENGTH_DEFAULT = 1.0
# cycle 52 (2026-09-05): the terminal taper. Today's profile holds -0.7 x strength into the wheel stop (at the driver's strength 1.4:
# -0.98 -- the recorded harsh no-lead force-coast stops, a_wheelstop -0.60, jerk 11.8). Below 1 m/s the tapered profile eases so the
# stopping service's hold (entered at v ~0.2) builds the secure pressure once stopped, as on governed lead stops (a_wheelstop -0.31).
# First candidate (red-team 20260905-200130, conservative): at the driver's strength 1.4 the wheel-stop value is -0.70 = the
# service's own hold level (no new creep exposure; the recorded escapes sit at commands -0.32..-0.62), the knee -0.84 at 0.5 m/s.
FORCE_COAST_TAPER_WHEEL_STOP_ACCEL = -0.50   # at the stop gate (x strength)
FORCE_COAST_TAPER_KNEE_V_OFFSET = 0.3        # m/s above the stop gate ...
FORCE_COAST_TAPER_KNEE_ACCEL = -0.60         # ... the profile passes here; -1.0 x strength at gate + 0.8 as before
FORCE_COAST_RELEASE_J = 0.8                  # m/s^3: the force-coast command may RISE (ease) no faster than this


def get_force_coast_target_accel(v_ego, stop_gate, strength=FORCE_COAST_STRENGTH_DEFAULT):
  if stopping_flags.FORCE_COAST_TERMINAL_TAPER:
    bp = [stop_gate, stop_gate + FORCE_COAST_TAPER_KNEE_V_OFFSET, stop_gate + 0.8, stop_gate + 2.2]
    vals = [FORCE_COAST_TAPER_WHEEL_STOP_ACCEL, FORCE_COAST_TAPER_KNEE_ACCEL, -1.0, FORCE_COAST_HIGH_SPEED_MIN_ACCEL]
  else:
    bp = [stop_gate, stop_gate + 0.8, stop_gate + 2.2]
    vals = [FORCE_COAST_NEAR_STOP_MIN_ACCEL, -1.0, FORCE_COAST_HIGH_SPEED_MIN_ACCEL]
  base_force_coast_target = float(np.interp(v_ego, bp, vals))
  return max(base_force_coast_target * max(float(strength), 0.0), FORCE_COAST_ACCEL_MIN)


def get_force_coast_target_from_toggles(v_ego, frogpilot_toggles):
  stop_gate = max(float(getattr(frogpilot_toggles, "vEgoStopping", 0.2)), 0.2)
  strength = getattr(frogpilot_toggles, "force_coast_strength", FORCE_COAST_STRENGTH_DEFAULT)
  return get_force_coast_target_accel(v_ego, stop_gate, strength)


def get_force_coast_ramped_accel(start_accel, target_accel, elapsed_s):
  # Ramp the commanded acceleration itself; limiting only the planner's allowed braking still lets
  # its first Force Coast request step straight to the selected deceleration.
  progress = float(np.clip(float(elapsed_s) / FORCE_COAST_RAMP_IN_S, 0.0, 1.0))
  return float(start_accel + ((target_accel - start_accel) * progress))
