import numpy as np
from cereal import car
from opendbc.car.hyundai.values import CAR as HYUNDAI_CAR
from openpilot.common.pid import PIDController
from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.controls.lib.drive_helpers import CONTROL_N
from openpilot.selfdrive.controls.lib.stop_and_go_helpers import should_release_stop_hold_for_departing_lead
from openpilot.selfdrive.controls.lib.stopping_guard import apply_low_speed_output_slew
from openpilot.selfdrive.controls.lib.stopping_controller import StoppingController
from openpilot.selfdrive.modeld.constants import ModelConstants

clip = np.clip
interp = np.interp


def apply_deadzone(error: float, deadzone: float) -> float:
  if error > deadzone:
    return error - deadzone
  if error < -deadzone:
    return error + deadzone
  return 0.0

STOPPING_V_BP =      [ 0.01,   0.2,   0.5  ]
STOPPING_ACCEL_MAX = [-0.01,  -0.1,   -0.3  ]
STOPPING_ACCEL_MIN = [-0.1,   -0.5,   -1.0  ]
MIN_STOP_TARGET_MODE_DISTANCE_M = 0.2
MAX_STOP_TARGET_MODE_DISTANCE_M = 0.5

from cereal import log

CONTROL_N_T_IDX = ModelConstants.T_IDXS[:CONTROL_N]

LongCtrlState = car.CarControl.Actuators.LongControlState
EXPERIMENTAL_CLOSE_LEAD_ACCEL_CAP_STRENGTH = 0.5


def should_enter_stop_target_mode(v_ego: float, a_target: float, distance_to_stop_target_m: float | None) -> bool:
  if distance_to_stop_target_m is None:
    return False

  min_meaningful_distance = clip(
    interp(v_ego, [0.0, 1.0, 2.3, 4.2, 6.0], [0.20, 0.22, 0.34, 0.44, 0.48]) - interp(-a_target, [0.2, 0.6, 1.2, 1.8], [0.0, 0.08, 0.22, 0.30]),
    MIN_STOP_TARGET_MODE_DISTANCE_M,
    MAX_STOP_TARGET_MODE_DISTANCE_M,
  )
  if distance_to_stop_target_m <= min_meaningful_distance:
    return False

  distance_to_target = float(clip(distance_to_stop_target_m, 0.0, 6.0))
  activation_limit = interp(v_ego, [0.0, 0.6, 1.5, 3.0, 5.0], [0.35, 0.65, 1.10, 1.70, 2.30])
  min_stop_approach_accel = interp(v_ego, [0.0, 0.6, 1.5, 3.0, 5.0], [-0.03, -0.06, -0.10, -0.16, -0.22])
  return distance_to_target < activation_limit and a_target < min_stop_approach_accel


def should_hold_stop_target_mode(v_ego: float, a_target: float, distance_to_stop_target_m: float | None) -> bool:
  if should_enter_stop_target_mode(v_ego, a_target, distance_to_stop_target_m):
    return True
  if distance_to_stop_target_m is None or distance_to_stop_target_m <= 0.0:
    return False

  hold_limit = interp(v_ego, [0.0, 0.8, 1.5, 2.4], [1.20, 1.40, 1.65, 1.90])
  hold_accel = interp(v_ego, [0.0, 0.8, 1.5, 2.4], [-0.04, -0.07, -0.10, -0.16])
  return v_ego < 2.5 and distance_to_stop_target_m < hold_limit and a_target < hold_accel


def should_apply_stop_target_approach_mode(v_ego: float, a_target: float, distance_to_stop_target_m: float | None) -> bool:
  if should_enter_stop_target_mode(v_ego, a_target, distance_to_stop_target_m):
    return False
  if distance_to_stop_target_m is None or distance_to_stop_target_m <= 0.0:
    return False

  distance_to_target = float(clip(distance_to_stop_target_m, 0.0, 6.0))
  activation_limit = interp(v_ego, [1.0, 2.8, 4.5, 7.0], [1.0, 3.0, 3.8, 4.8])
  min_stop_approach_accel = interp(v_ego, [1.0, 2.8, 4.5, 7.0], [-0.04, -0.07, -0.10, -0.14])
  return v_ego > 1.0 and distance_to_target < activation_limit and a_target < min_stop_approach_accel


def should_apply_stop_target_carry_mode(v_ego: float, a_target: float, distance_to_stop_target_m: float | None) -> bool:
  if should_enter_stop_target_mode(v_ego, a_target, distance_to_stop_target_m):
    return False
  if should_apply_stop_target_approach_mode(v_ego, a_target, distance_to_stop_target_m):
    return False
  if distance_to_stop_target_m is None or distance_to_stop_target_m <= 0.0:
    return False
  if not (0.55 < v_ego < 1.25):
    return False
  if a_target > -0.08:
    return False

  requested_decel = float(clip(-a_target, 0.12, 0.90))
  predicted_stop_distance = (v_ego * v_ego) / max(2.0 * requested_decel, 0.24)
  carry_margin = interp(v_ego, [0.55, 0.85, 1.25], [0.80, 1.05, 1.35])
  return distance_to_stop_target_m > (predicted_stop_distance + carry_margin)


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
  if closing_speed < 0.12:
    return None

  activation_gap = interp(v_ego, [0.12, 0.35, 0.65, 0.95], [1.80, 2.40, 3.00, 3.40])
  if lead_d_rel > activation_gap:
    return None

  gap_cap = interp(lead_d_rel, [1.20, 1.60, 2.20, 2.80, 3.40], [-0.82, -0.74, -0.64, -0.55, -0.48])
  closing_extra = interp(closing_speed, [0.12, 0.35, 0.70, 1.00], [0.00, 0.04, 0.09, 0.13])
  speed_extra = interp(v_ego, [0.12, 0.35, 0.65, 0.95], [0.00, 0.02, 0.05, 0.08])
  return float(clip(gap_cap - closing_extra - speed_extra, -0.90, -0.42))


def low_speed_stopped_lead_glide_accel_cap(v_ego: float, lead_v: float, lead_d_rel: float, distance_to_stop_target_m: float | None) -> float | None:
  if not (0.35 <= v_ego <= 1.10):
    return None
  if lead_d_rel <= 0.0 or lead_v > 0.25:
    return None

  closing_speed = v_ego - lead_v
  if closing_speed < 0.45:
    return None

  activation_gap = interp(v_ego, [0.35, 0.65, 0.95, 1.10], [5.0, 6.6, 8.0, 8.6])
  if lead_d_rel > activation_gap:
    return None

  gap_cap = interp(lead_d_rel, [4.8, 6.0, 7.5, 8.6], [-0.50, -0.44, -0.39, -0.35])
  speed_cap = interp(v_ego, [0.35, 0.65, 0.95, 1.10], [-0.35, -0.41, -0.47, -0.50])
  closing_extra = interp(closing_speed, [0.45, 0.75, 1.10], [0.00, 0.04, 0.08])
  distance_relief = 0.0
  if distance_to_stop_target_m is not None and distance_to_stop_target_m > 0.0:
    distance_relief = interp(distance_to_stop_target_m, [2.0, 3.5, 4.5], [-0.02, 0.0, 0.03])
  return float(clip(min(gap_cap, speed_cap) - closing_extra + distance_relief, -0.58, -0.34))


def should_apply_experimental_close_lead_accel_cap(cp, experimental_mode: bool) -> bool:
  return experimental_mode and getattr(cp, "carFingerprint", None) == HYUNDAI_CAR.HYUNDAI_SANTA_FE_HEV_2022


def should_apply_low_speed_close_lead_accel_cap(cp) -> bool:
  return getattr(cp, "carFingerprint", None) == HYUNDAI_CAR.HYUNDAI_SANTA_FE_HEV_2022


def should_apply_low_speed_stopped_lead_glide_accel_cap(cp) -> bool:
  return getattr(cp, "carFingerprint", None) == HYUNDAI_CAR.HYUNDAI_SANTA_FE_HEV_2022


def should_apply_stop_entry_handoff_soften(
  v_ego: float,
  a_ego: float,
  a_target: float,
  last_output_accel: float,
  distance_to_stop_target_m: float | None,
) -> bool:
  if not (0.35 < v_ego < 2.30):
    return False
  if not (-1.05 < a_ego < -0.42):
    return False
  if last_output_accel > -0.48 or last_output_accel < -0.88:
    return False
  target_floor = interp(v_ego, [0.35, 0.60, 1.00, 1.50, 2.30], [-0.20, -0.28, -0.38, -0.50, -0.60])
  if a_target < target_floor:
    return False
  if distance_to_stop_target_m is not None and 0.0 <= distance_to_stop_target_m < 0.22:
    return False
  return True


def stop_entry_handoff_accel_cap(v_ego: float, distance_to_stop_target_m: float | None) -> float:
  speed_cap = interp(v_ego, [0.35, 0.60, 1.00, 1.50, 2.30], [-0.44, -0.48, -0.56, -0.64, -0.74])
  if distance_to_stop_target_m is None or distance_to_stop_target_m <= 0.0:
    return speed_cap
  distance_cap = interp(distance_to_stop_target_m, [0.22, 0.40, 0.75, 1.20, 2.00], [-0.72, -0.66, -0.58, -0.52, -0.46])
  return min(speed_cap, distance_cap)


def should_hold_stop_target_dropout(
  v_ego: float,
  a_target: float | None,
  distance_to_stop_target_m: float | None,
  last_distance_to_stop_target_m: float | None,
  last_output_accel: float,
  time_since_stop_intent_s: float,
) -> bool:
  if a_target is None or distance_to_stop_target_m is None or distance_to_stop_target_m <= 0.0:
    return False
  if last_distance_to_stop_target_m is None or last_distance_to_stop_target_m <= 0.0:
    return False
  if not (0.0 < v_ego < 0.22):
    return False
  if last_output_accel > -0.12:
    return False
  if time_since_stop_intent_s > 0.35:
    return False

  hold_distance_limit = interp(v_ego, [0.00, 0.08, 0.16, 0.22], [1.05, 0.98, 0.92, 0.86])
  if distance_to_stop_target_m > hold_distance_limit:
    return False

  growth_allowance = interp(v_ego, [0.00, 0.08, 0.16, 0.22], [0.06, 0.08, 0.10, 0.12])
  if distance_to_stop_target_m > (last_distance_to_stop_target_m + growth_allowance):
    return False

  a_target_ceiling = interp(v_ego, [0.00, 0.08, 0.16, 0.22], [0.30, 0.26, 0.20, 0.14])
  return a_target <= (a_target_ceiling + 1e-6)


def should_hold_low_speed_stop_target_release(
  v_ego: float,
  a_target: float | None,
  distance_to_stop_target_m: float | None,
  last_distance_to_stop_target_m: float | None,
  last_output_accel: float,
  time_since_stop_intent_s: float,
) -> bool:
  if a_target is None or distance_to_stop_target_m is None or distance_to_stop_target_m <= 0.0:
    return False
  if last_distance_to_stop_target_m is None or last_distance_to_stop_target_m <= 0.0:
    return False
  if not (0.0 < v_ego < 0.22):
    return False
  if time_since_stop_intent_s > 0.8:
    return False
  if last_output_accel > -0.18:
    return False

  hold_distance_floor = interp(v_ego, [0.00, 0.04, 0.08, 0.12, 0.22], [0.56, 0.52, 0.46, 0.38, 0.30])
  if distance_to_stop_target_m < hold_distance_floor:
    return False

  growth_allowance = interp(v_ego, [0.00, 0.04, 0.08, 0.12, 0.22], [0.08, 0.10, 0.12, 0.15, 0.18])
  if distance_to_stop_target_m > (last_distance_to_stop_target_m + growth_allowance):
    return False

  release_accel_ceiling = interp(v_ego, [0.00, 0.04, 0.08, 0.12, 0.22], [0.95, 0.82, 0.62, 0.42, 0.18])
  return a_target <= (release_accel_ceiling + 1e-6)


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

def long_control_state_trans_old_long(CP, active, long_control_state, v_ego, v_target,
                                      v_target_1sec, brake_pressed, cruise_standstill, frogpilot_toggles):
  accelerating = v_target_1sec > v_target
  planned_stop = (v_target < frogpilot_toggles.vEgoStopping and
                  v_target_1sec < frogpilot_toggles.vEgoStopping and
                  not accelerating)
  stay_stopped = (v_ego < frogpilot_toggles.vEgoStopping and
                  (brake_pressed or cruise_standstill))
  stopping_condition = planned_stop or stay_stopped

  starting_condition = (v_target_1sec > frogpilot_toggles.vEgoStarting and
                        accelerating and
                        not cruise_standstill and
                        not brake_pressed)
  started_condition = v_ego > frogpilot_toggles.vEgoStarting

  if not active:
    long_control_state = LongCtrlState.off

  else:
    if long_control_state in (LongCtrlState.off, LongCtrlState.pid):
      long_control_state = LongCtrlState.pid
      if stopping_condition:
        long_control_state = LongCtrlState.stopping

    elif long_control_state == LongCtrlState.stopping:
      if starting_condition and CP.startingState:
        long_control_state = LongCtrlState.starting
      elif starting_condition:
        long_control_state = LongCtrlState.pid

    elif long_control_state == LongCtrlState.starting:
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
    self.v_pid = 0.0
    self.last_output_accel = 0.0
    self.prep_stopping = False
    self.breakpoint_v = 1.
    self.initial_stopping_accel = -2
    self.initial_stopping_speed = 1
    self.stopping_breakpoint_recorded = False
    self.stopping_controller = StoppingController()
    self.time_since_standstill_s = 10.0
    self.time_since_stop_intent_s = 10.0
    self.last_distance_to_stop_target_m: float | None = None

  def reset(self):
    self.pid.reset()
    self.stopping_controller.reset()
    self.time_since_standstill_s = 10.0
    self.time_since_stop_intent_s = 10.0
    self.last_distance_to_stop_target_m = None

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
    force_coast=False,
  ):
    """Update longitudinal control. This updates the state machine and runs a PID loop"""
    self.pid.neg_limit = accel_limits[0]
    self.pid.pos_limit = accel_limits[1]
    human_acceleration_active = frogpilot_toggles.human_acceleration and not experimental_mode

    output_accel = self.last_output_accel
    prev_distance_to_stop_target_m = self.last_distance_to_stop_target_m

    release_lock_active = False
    max_expected_accel = interp(CS.vEgo, STOPPING_V_BP, STOPPING_ACCEL_MAX)
    stop_target_request = should_enter_stop_target_mode(CS.vEgo, a_target, distance_to_stop_target_m)
    stop_request_active = should_stop or stop_target_request
    stop_target_approach_active = (
      not stop_request_active
      and should_apply_stop_target_approach_mode(CS.vEgo, a_target, distance_to_stop_target_m)
    )
    stop_target_carry_active = (
      not stop_request_active
      and not stop_target_approach_active
      and should_apply_stop_target_carry_mode(CS.vEgo, a_target, distance_to_stop_target_m)
    )
    departing_lead_release = should_release_stop_hold_for_departing_lead(
      human_acceleration=bool(frogpilot_toggles.human_acceleration),
      output_should_stop=bool(should_stop),
      force_coast=bool(force_coast),
      standstill=bool(getattr(CS, "standstill", False)) or bool(CS.cruiseState.standstill),
      v_ego=float(CS.vEgo),
      v_ego_starting=float(frogpilot_toggles.vEgoStarting),
      lead_status=bool(lead_status),
      lead_v=float(lead_v),
      lead_d_rel=float(lead_d_rel),
    )
    if departing_lead_release:
      stop_request_active = False
      stop_target_approach_active = False
    stop_target_release_hold_active = (
      not departing_lead_release
      and should_hold_low_speed_stop_target_release(
        v_ego=CS.vEgo,
        a_target=a_target,
        distance_to_stop_target_m=distance_to_stop_target_m,
        last_distance_to_stop_target_m=prev_distance_to_stop_target_m,
        last_output_accel=self.last_output_accel,
        time_since_stop_intent_s=self.time_since_stop_intent_s,
      )
    )
    if stop_target_release_hold_active:
      stop_request_active = True
      stop_target_approach_active = False
      stop_target_carry_active = False
    new_control_state = long_control_state_trans(self.CP, active, self.long_control_state, CS.vEgo,
                                                 (should_stop or stop_target_release_hold_active) and not departing_lead_release, CS.brakePressed,
                                                 CS.cruiseState.standstill, frogpilot_toggles,
                                                 a_target=a_target,
                                                 distance_to_stop_target_m=distance_to_stop_target_m)
    if (
      self.long_control_state == LongCtrlState.stopping
      and new_control_state != LongCtrlState.stopping
      and should_hold_stop_target_dropout(
        v_ego=CS.vEgo,
        a_target=a_target,
        distance_to_stop_target_m=distance_to_stop_target_m,
        last_distance_to_stop_target_m=prev_distance_to_stop_target_m,
        last_output_accel=self.last_output_accel,
        time_since_stop_intent_s=self.time_since_stop_intent_s,
      )
    ):
      new_control_state = LongCtrlState.stopping
    entered_stopping = self.long_control_state != LongCtrlState.stopping and new_control_state == LongCtrlState.stopping

    if entered_stopping:
      if self.prep_stopping:
        if CS.vEgo < self.initial_stopping_speed:
          self.prep_stopping = False
          self.initial_stopping_accel = CS.aEgo
      else:
        self.stopping_breakpoint_recorded = False

        self.initial_stopping_accel = CS.aEgo
        self.initial_stopping_speed = CS.vEgo
      # print(f"Starting to stop, initial accel {self.initial_stopping_accel}")

    if new_control_state in (LongCtrlState.stopping, LongCtrlState.pid) and self.prep_stopping:
      self.long_control_state = LongCtrlState.pid
    else:
      self.long_control_state = new_control_state

    standstill = bool(getattr(CS, "standstill", False)) or bool(CS.cruiseState.standstill)
    if standstill:
      self.time_since_standstill_s = 0.0
    else:
      self.time_since_standstill_s = min(self.time_since_standstill_s + DT_CTRL, 10.0)

    stop_intent_active = stop_request_active or stop_target_approach_active or stop_target_carry_active or (self.long_control_state == LongCtrlState.stopping)
    if stop_intent_active:
      self.time_since_stop_intent_s = 0.0
    else:
      self.time_since_stop_intent_s = min(self.time_since_stop_intent_s + DT_CTRL, 10.0)

    standstill_recent = self.time_since_standstill_s < 0.5
    stop_intent_recent = self.time_since_stop_intent_s < 1.0

    if self.long_control_state == LongCtrlState.off or not stop_intent_active:
      self.stopping_controller.reset()

    if self.long_control_state == LongCtrlState.off:
      self.reset()
      self.prep_stopping = False
      output_accel = 0.

    elif self.prep_stopping:
      output_accel = self.initial_stopping_accel

    elif self.long_control_state == LongCtrlState.stopping:
      handoff_soften_cap: float | None = None

      if not self.stopping_breakpoint_recorded and CS.vEgo < 0.5:
        self.stopping_breakpoint_recorded = True
        breakpoint_v_bp = [ -1., -0.1  ]
        breakpoint_v_v =  [  1.,  0.5 ]

        self.breakpoint_v = interp(CS.aEgo, breakpoint_v_bp, breakpoint_v_v)

      output_accel = min(output_accel, -0.1)
      if entered_stopping and should_apply_stop_entry_handoff_soften(CS.vEgo, CS.aEgo, a_target, self.last_output_accel, distance_to_stop_target_m):
        handoff_soften_cap = stop_entry_handoff_accel_cap(CS.vEgo, distance_to_stop_target_m)
        output_accel = max(output_accel, handoff_soften_cap)
                    # km/h
      stopping_mid_bp = self.CP.stoppingVbp[1] if len(self.CP.stoppingVbp) >= 2 else STOPPING_V_BP[1]
      stopping_mid_bp = clip(stopping_mid_bp, STOPPING_V_BP[0] + 0.001, STOPPING_V_BP[-1] - 0.001)
      stopping_v_bp = [STOPPING_V_BP[0], stopping_mid_bp, STOPPING_V_BP[-1]]
      stopping_accel_max = STOPPING_ACCEL_MAX
      stopping_accel_min = STOPPING_ACCEL_MIN

      max_expected_accel = interp(CS.vEgo, stopping_v_bp, stopping_accel_max)
      min_expected_accel = interp(CS.vEgo, stopping_v_bp, stopping_accel_min)

      stop_result = self.stopping_controller.update(
        output_accel=output_accel,
        last_output_accel=max(self.last_output_accel, handoff_soften_cap) if handoff_soften_cap is not None else self.last_output_accel,
        should_stop=stop_request_active,
        v_ego=CS.vEgo,
        a_ego=CS.aEgo,
        max_expected_accel=max_expected_accel,
        min_expected_accel=min_expected_accel,
        distance_to_stop_target_m=distance_to_stop_target_m,
        raw_should_stop=should_stop,
        stop_accel=self.CP.stopAccel,
        dt=DT_CTRL,
        lead_status=lead_status,
        lead_v=lead_v,
        lead_d_rel=lead_d_rel,
      )
      output_accel = stop_result.output_accel
      release_lock_active = stop_result.release_lock_active
      if should_apply_low_speed_close_lead_accel_cap(self.CP) and lead_status:
        close_lead_cap = low_speed_close_lead_accel_cap(CS.vEgo, lead_v, lead_d_rel)
        if close_lead_cap is not None and output_accel > close_lead_cap:
          close_lead_brake_step = interp(CS.vEgo, [0.12, 0.35, 0.65, 0.95], [0.018, 0.024, 0.032, 0.040])
          output_accel = max(close_lead_cap, output_accel - close_lead_brake_step)

    elif self.long_control_state == LongCtrlState.starting:
      output_accel = (a_target if human_acceleration_active else frogpilot_toggles.startAccel)
      self.reset()

    else:  # LongCtrlState.pid
      error = a_target - CS.aEgo
      freeze_integrator = stop_target_approach_active or stop_target_carry_active
      output_accel = self.pid.update(error, speed=CS.vEgo,
                                     feedforward=a_target,
                                     freeze_integrator=freeze_integrator)
      if stop_target_approach_active:
        output_accel = min(output_accel, stop_target_approach_accel_cap(CS.vEgo, distance_to_stop_target_m))
      if stop_target_carry_active:
        output_accel = max(output_accel, stop_target_carry_accel_floor(CS.vEgo, distance_to_stop_target_m))

    if self.long_control_state != LongCtrlState.off:
      allow_fast_release = (
        not stop_request_active and not stop_target_approach_active
        and self.long_control_state in (LongCtrlState.pid, LongCtrlState.starting)
        and a_target > 0.2
        and CS.vEgo > 0.12
      )
      if departing_lead_release:
        allow_fast_release = True
      if stop_intent_recent and not standstill_recent:
        allow_fast_release = False
      apply_global_low_speed_slew = not (self.long_control_state == LongCtrlState.stopping and stop_request_active)
      if apply_global_low_speed_slew:
        output_accel = apply_low_speed_output_slew(
          output_accel=output_accel,
          last_output_accel=self.last_output_accel,
          should_stop=(stop_request_active or stop_target_approach_active or stop_target_carry_active),
          v_ego=CS.vEgo,
          a_ego=CS.aEgo,
          max_expected_accel=max_expected_accel,
          allow_fast_release=allow_fast_release,
          release_lock_active=release_lock_active,
        )

      stopped_lead_glide_cap = (
        low_speed_stopped_lead_glide_accel_cap(CS.vEgo, lead_v, lead_d_rel, distance_to_stop_target_m)
        if (
          should_apply_low_speed_stopped_lead_glide_accel_cap(self.CP)
          and lead_status
          and (stop_request_active or stop_target_approach_active or stop_target_carry_active or self.long_control_state == LongCtrlState.stopping)
        )
        else None
      )
      if stopped_lead_glide_cap is not None and output_accel > stopped_lead_glide_cap:
        stopped_lead_brake_step = interp(CS.vEgo, [0.35, 0.65, 0.95, 1.10], [0.004, 0.006, 0.008, 0.009])
        output_accel = max(stopped_lead_glide_cap, min(output_accel, self.last_output_accel) - stopped_lead_brake_step)

      if should_apply_pid_brake_model_alignment(self.CP) and self.long_control_state == LongCtrlState.pid and not stop_request_active and not stop_target_approach_active:
        aligned_output = apply_pid_brake_model_alignment(output_accel, a_target, CS.aEgo, CS.vEgo)
        if aligned_output > output_accel:
          self.pid.i = max(self.pid.i, aligned_output - (self.pid.p + self.pid.d + self.pid.f))
          output_accel = aligned_output
      if (
        should_apply_experimental_close_lead_accel_cap(self.CP, experimental_mode)
        and self.long_control_state == LongCtrlState.pid
        and not stop_request_active
        and not stop_target_approach_active
        and not stop_target_carry_active
        and lead_status
      ):
        close_lead_cap = experimental_close_lead_accel_cap(CS.vEgo, lead_v, lead_d_rel)
        if close_lead_cap is not None and output_accel > close_lead_cap:
          output_accel = apply_experimental_close_lead_accel_cap(output_accel, close_lead_cap)
          self.pid.i = min(self.pid.i, output_accel - (self.pid.p + self.pid.d + self.pid.f))

    self.last_distance_to_stop_target_m = float(distance_to_stop_target_m) if distance_to_stop_target_m is not None and distance_to_stop_target_m > 0.0 else None
    self.last_output_accel = clip(output_accel, accel_limits[0], accel_limits[1])
    return self.last_output_accel

  def reset_old_long(self, v_pid):
    """Reset PID controller and change setpoint"""
    self.pid.reset()
    self.v_pid = v_pid

  def update_old_long(self, active, CS, long_plan, accel_limits, t_since_plan, frogpilot_toggles):
    """Update longitudinal control. This updates the state machine and runs a PID loop"""
    # Interp control trajectory
    speeds = long_plan.speeds
    if len(speeds) == CONTROL_N:
      v_target_now = interp(t_since_plan, CONTROL_N_T_IDX, speeds)
      a_target_now = interp(t_since_plan, CONTROL_N_T_IDX, long_plan.accels)

      v_target = interp(frogpilot_toggles.longitudinalActuatorDelay + t_since_plan, CONTROL_N_T_IDX, speeds)
      a_target = 2 * (v_target - v_target_now) / frogpilot_toggles.longitudinalActuatorDelay - a_target_now

      v_target_1sec = interp(frogpilot_toggles.longitudinalActuatorDelay + t_since_plan + 1.0, CONTROL_N_T_IDX, speeds)
    else:
      v_target = 0.0
      v_target_now = 0.0
      v_target_1sec = 0.0
      a_target = 0.0

    self.pid.neg_limit = accel_limits[0]
    self.pid.pos_limit = accel_limits[1]

    output_accel = self.last_output_accel
    self.long_control_state = long_control_state_trans_old_long(self.CP, active, self.long_control_state, CS.vEgo,
                                                                v_target, v_target_1sec, CS.brakePressed,
                                                                CS.cruiseState.standstill, frogpilot_toggles)

    if self.long_control_state == LongCtrlState.off:
      self.reset_old_long(CS.vEgo)
      output_accel = 0.

    elif self.long_control_state == LongCtrlState.stopping:
      if output_accel > frogpilot_toggles.stopAccel:
        output_accel = min(output_accel, 0.0)
        output_accel -= frogpilot_toggles.stoppingDecelRate * DT_CTRL
      self.reset_old_long(CS.vEgo)

    elif self.long_control_state == LongCtrlState.starting:
      output_accel = frogpilot_toggles.startAccel
      self.reset_old_long(CS.vEgo)

    elif self.long_control_state == LongCtrlState.pid:
      self.v_pid = v_target_now

      # Toyota starts braking more when it thinks you want to stop
      # Freeze the integrator so we don't accelerate to compensate, and don't allow positive acceleration
      # TODO too complex, needs to be simplified and tested on toyotas
      prevent_overshoot = not self.CP.stoppingControl and CS.vEgo < 1.5 and v_target_1sec < 0.7 and v_target_1sec < self.v_pid
      deadzone = interp(CS.vEgo, self.CP.longitudinalTuning.deadzoneBP, self.CP.longitudinalTuning.deadzoneV)
      freeze_integrator = prevent_overshoot

      error = self.v_pid - CS.vEgo
      error_deadzone = apply_deadzone(error, deadzone)
      output_accel = self.pid.update(error_deadzone, speed=CS.vEgo,
                                     feedforward=a_target,
                                     freeze_integrator=freeze_integrator)

    self.last_output_accel = clip(output_accel, accel_limits[0], accel_limits[1])

    return self.last_output_accel
