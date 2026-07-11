#!/usr/bin/env python3
import numpy as np

FORCE_COAST_ACCEL_MIN = -3.5
FORCE_COAST_HIGH_SPEED_MIN_ACCEL = -1.2
FORCE_COAST_NEAR_STOP_MIN_ACCEL = -0.7
FORCE_COAST_RAMP_IN_S = 1.0
FORCE_COAST_STRENGTH_DEFAULT = 1.0


def get_force_coast_target_accel(v_ego, stop_gate, strength=FORCE_COAST_STRENGTH_DEFAULT):
  base_force_coast_target = float(np.interp(v_ego,
                                            [stop_gate, stop_gate + 0.8, stop_gate + 2.2],
                                            [FORCE_COAST_NEAR_STOP_MIN_ACCEL, -1.0, FORCE_COAST_HIGH_SPEED_MIN_ACCEL]))
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
