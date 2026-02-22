from cereal import car
from openpilot.common.numpy_fast import clip, interp
from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.controls.lib.drive_helpers import CONTROL_N, apply_deadzone
from openpilot.selfdrive.controls.lib.pid import PIDController
from openpilot.selfdrive.controls.lib.stopping_guard import apply_low_speed_output_slew
from openpilot.selfdrive.controls.lib.stopping_controller import StoppingController
from openpilot.selfdrive.modeld.constants import ModelConstants

STOPPING_V_BP =      [ 0.01,   0.2,   0.5  ]
STOPPING_ACCEL_MAX = [-0.01,  -0.1,   -0.3  ]
STOPPING_ACCEL_MIN = [-0.1,   -0.5,   -1.0  ]

from cereal import log

CONTROL_N_T_IDX = ModelConstants.T_IDXS[:CONTROL_N]

LongCtrlState = car.CarControl.Actuators.LongControlState


def long_control_state_trans(CP, active, long_control_state, v_ego,
                             should_stop, brake_pressed, cruise_standstill, frogpilot_toggles):
  # Ignore cruise standstill if car has a gas interceptor
  cruise_standstill = cruise_standstill and not CP.enableGasInterceptor
  stopping_condition = should_stop
  starting_condition = (not should_stop and
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

  def reset(self):
    self.pid.reset()
    self.stopping_controller.reset()
    self.time_since_standstill_s = 10.0
    self.time_since_stop_intent_s = 10.0

  def update(self, active, CS, a_target, should_stop, accel_limits, frogpilot_toggles):
    """Update longitudinal control. This updates the state machine and runs a PID loop"""
    self.pid.neg_limit = accel_limits[0]
    self.pid.pos_limit = accel_limits[1]

    output_accel = self.last_output_accel

    release_lock_active = False
    max_expected_accel = interp(CS.vEgo, STOPPING_V_BP, STOPPING_ACCEL_MAX)
    new_control_state = long_control_state_trans(self.CP, active, self.long_control_state, CS.vEgo,
                                                       should_stop, CS.brakePressed,
                                                       CS.cruiseState.standstill, frogpilot_toggles)

    if self.long_control_state != LongCtrlState.stopping and new_control_state == LongCtrlState.stopping:
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

    stop_intent_active = should_stop or (self.long_control_state == LongCtrlState.stopping)
    if stop_intent_active:
      self.time_since_stop_intent_s = 0.0
    else:
      self.time_since_stop_intent_s = min(self.time_since_stop_intent_s + DT_CTRL, 10.0)

    standstill_recent = self.time_since_standstill_s < 0.5
    stop_intent_recent = self.time_since_stop_intent_s < 1.0

    if self.long_control_state == LongCtrlState.off or not should_stop:
      self.stopping_controller.reset()

    if self.long_control_state == LongCtrlState.off:
      self.reset()
      self.prep_stopping = False
      output_accel = 0.

    elif self.prep_stopping:
      output_accel = self.initial_stopping_accel

    elif self.long_control_state == LongCtrlState.stopping:

      if not self.stopping_breakpoint_recorded and CS.vEgo < 0.5:
        self.stopping_breakpoint_recorded = True
        breakpoint_v_bp = [ -1., -0.1  ]
        breakpoint_v_v =  [  1.,  0.5 ]

        self.breakpoint_v = interp(CS.aEgo, breakpoint_v_bp, breakpoint_v_v)

      output_accel = min(output_accel, -0.1)
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
        last_output_accel=self.last_output_accel,
        should_stop=should_stop,
        v_ego=CS.vEgo,
        a_ego=CS.aEgo,
        max_expected_accel=max_expected_accel,
        min_expected_accel=min_expected_accel,
        stop_accel=self.CP.stopAccel,
        dt=DT_CTRL,
      )
      output_accel = stop_result.output_accel
      release_lock_active = stop_result.release_lock_active

    elif self.long_control_state == LongCtrlState.starting:
      output_accel = (a_target if frogpilot_toggles.human_acceleration else frogpilot_toggles.startAccel)
      self.reset()

    else:  # LongCtrlState.pid
      error = a_target - CS.aEgo
      output_accel = self.pid.update(error, speed=CS.vEgo,
                                     feedforward=a_target)

    if self.long_control_state != LongCtrlState.off:
      allow_fast_release = (
        not should_stop
        and self.long_control_state in (LongCtrlState.pid, LongCtrlState.starting)
        and a_target > 0.2
        and CS.vEgo > 0.12
      )
      if stop_intent_recent and not standstill_recent:
        allow_fast_release = False
      apply_global_low_speed_slew = not (self.long_control_state == LongCtrlState.stopping and should_stop)
      if apply_global_low_speed_slew:
        output_accel = apply_low_speed_output_slew(
          output_accel=output_accel,
          last_output_accel=self.last_output_accel,
          should_stop=should_stop,
          v_ego=CS.vEgo,
          a_ego=CS.aEgo,
          max_expected_accel=max_expected_accel,
          allow_fast_release=allow_fast_release,
          release_lock_active=release_lock_active,
        )

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
