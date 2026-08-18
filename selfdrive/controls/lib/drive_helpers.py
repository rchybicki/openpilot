import numpy as np
from openpilot.common.constants import ACCELERATION_DUE_TO_GRAVITY
from openpilot.common.realtime import DT_CTRL, DT_MDL

MIN_SPEED = 1.0
CONTROL_N = 17
CAR_ROTATION_RADIUS = 0.0
# This is a turn radius smaller than most cars can achieve
MAX_CURVATURE = 0.2
MAX_VEL_ERR = 5.0  # m/s

# EU guidelines
MAX_LATERAL_JERK = 5.0  # m/s^3
MAX_LATERAL_ACCEL_NO_ROLL = 3.0  # m/s^2
MIN_STABLE_DELAY = 0.3


def longitudinal_control_active(enabled, openpilot_longitudinal_control, pause_longitudinal,
                                override_longitudinal, longitudinal_active_with_gas, gas_pressed):
  return enabled and openpilot_longitudinal_control and not pause_longitudinal and \
         (not override_longitudinal or (longitudinal_active_with_gas and gas_pressed))


def longitudinal_accel_with_gas(accel, longitudinal_active_with_gas, gas_pressed):
  return max(accel, 0.0) if longitudinal_active_with_gas and gas_pressed else accel


def longitudinal_control_override(enabled, openpilot_longitudinal_control, long_active,
                                  longitudinal_active_with_gas, gas_pressed):
  gas_override = longitudinal_active_with_gas and gas_pressed
  return enabled and openpilot_longitudinal_control and (not long_active or gas_override)


def clamp(val, min_val, max_val):
  clamped_val = float(np.clip(val, min_val, max_val))
  return clamped_val, clamped_val != val

def smooth_value(val, prev_val, tau, dt=DT_MDL):
  alpha = 1 - np.exp(-dt/tau) if tau > 0 else 1
  return alpha * val + (1 - alpha) * prev_val

def clip_curvature(v_ego, prev_curvature, new_curvature, roll) -> tuple[float, bool]:
  # This function respects ISO lateral jerk and acceleration limits + a max curvature
  v_ego = max(v_ego, MIN_SPEED)
  max_curvature_rate = MAX_LATERAL_JERK / (v_ego ** 2)  # inexact calculation, check https://github.com/commaai/openpilot/pull/24755
  new_curvature = np.clip(new_curvature,
                          prev_curvature - max_curvature_rate * DT_CTRL,
                          prev_curvature + max_curvature_rate * DT_CTRL)

  roll_compensation = roll * ACCELERATION_DUE_TO_GRAVITY
  max_lat_accel = MAX_LATERAL_ACCEL_NO_ROLL + roll_compensation
  min_lat_accel = -MAX_LATERAL_ACCEL_NO_ROLL + roll_compensation
  new_curvature, limited_accel = clamp(new_curvature, min_lat_accel / v_ego ** 2, max_lat_accel / v_ego ** 2)

  new_curvature, limited_max_curv = clamp(new_curvature, -MAX_CURVATURE, MAX_CURVATURE)
  return float(new_curvature), limited_accel or limited_max_curv


def get_accel_from_plan(speeds, accels, t_idxs, action_t=DT_MDL, vEgoStopping=0.3):
  if len(speeds) == len(t_idxs):
    v_now = speeds[0]
    a_now = accels[0]
    if action_t < MIN_STABLE_DELAY:
      v_target = v_now + (action_t / MIN_STABLE_DELAY) * (np.interp(MIN_STABLE_DELAY, t_idxs, speeds) - v_now)
    else:
      v_target = np.interp(action_t, t_idxs, speeds)
    a_target = 2 * (v_target - v_now) / (action_t) - a_now
  else:
    v_now = 0.0
    v_target = 0.0
    a_target = 0.0
  should_stop = (v_now < vEgoStopping and a_target < 0.1)
  return a_target, should_stop


def update_should_stop_falling_edge_hold(raw_should_stop, v_now, a_target, v_ego_stopping, hold_timer_s, hold_s, dt):
  """Falling-edge hold for the planner-published shouldStop flag (stopping redesign §4.1).

  Pure function: returns (held_should_stop, new_hold_timer_s). Strictly additive on the
  deassert side -- a raw True always passes through and re-arms the timer; a raw False can
  only be held True while the timer runs, the plan stays near stopping speed
  (v_now < v_ego_stopping + 0.15) and there is no clear go signal (a_target <= 0.2).
  hold_s == 0.0 is the kill switch: the raw flag is returned unchanged.
  The hold cannot create stops: from a zero timer, output is True only if raw is True.
  """
  if hold_s <= 0.0:
    return raw_should_stop, 0.0
  if raw_should_stop:
    return True, hold_s
  # 1e-9 epsilon so accumulated float error cannot stretch the hold by an extra frame
  if hold_timer_s > 1e-9 and v_now < v_ego_stopping + 0.15 and a_target <= 0.2:
    return True, max(hold_timer_s - dt, 0.0)
  return False, 0.0


def curv_from_psis(psi_target, psi_rate, vego, action_t):
  vego = np.clip(vego, MIN_SPEED, np.inf)
  curv_from_psi = psi_target / (vego * action_t)
  return 2*curv_from_psi - psi_rate / vego

def get_curvature_from_plan(yaws, yaw_rates, t_idxs, vego, action_t):
  if action_t < MIN_STABLE_DELAY:
    psi_target = (action_t / MIN_STABLE_DELAY) * np.interp(MIN_STABLE_DELAY, t_idxs, yaws)
  else:
    psi_target = np.interp(action_t, t_idxs, yaws)
  psi_rate = yaw_rates[0]
  return curv_from_psis(psi_target, psi_rate, vego, action_t)
