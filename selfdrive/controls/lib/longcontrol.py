import random
from cereal import car
from openpilot.common.numpy_fast import clip, interp
from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.controls.lib.drive_helpers import CONTROL_N
from openpilot.selfdrive.controls.lib.pid import PIDController
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.common.params import Params
from cereal import log

CONTROL_N_T_IDX = ModelConstants.T_IDXS[:CONTROL_N]

LongCtrlState = car.CarControl.Actuators.LongControlState


def long_control_state_trans(CP, active, long_control_state, v_ego,
                             should_stop, brake_pressed, cruise_standstill):
  stopping_condition = should_stop
  starting_condition = (not should_stop and
                        not cruise_standstill and
                        not brake_pressed)
  started_condition = v_ego > CP.vEgoStarting

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
                             k_f=CP.longitudinalTuning.kf, rate=1 / DT_CTRL)

    self.last_output_accel = 0.0
    self.prep_stopping = False
    self.breakpoint_v = 1.
    self.breakpoint_b = 0.1
    self.initial_stopping_accel = -2
    self.initial_stopping_speed = 1
    self.stopping_breakpoint_recorded = False
    self.params = Params()

  def reset(self):
    self.pid.reset()

  def update(self, active, CS, a_target, should_stop, accel_limits):
    """Update longitudinal control. This updates the state machine and runs a PID loop"""
    self.pid.neg_limit = accel_limits[0]
    self.pid.pos_limit = accel_limits[1]

    output_accel = self.last_output_accel
    force_stop = False
    # force_stop = self.CP.carName == "hyundai" and int(self.params.get('LongitudinalPersonality')) and CS.vEgo < 15.
    new_control_state = long_control_state_trans(self.CP, active, self.long_control_state, CS.vEgo,
                                                       should_stop, CS.brakePressed,
                                                       CS.cruiseState.standstill)

    if self.long_control_state != LongCtrlState.stopping and new_control_state == LongCtrlState.stopping:
      if self.prep_stopping:
        if CS.vEgo < self.initial_stopping_speed:
          self.prep_stopping = False
          self.initial_stopping_accel = CS.aEgo
      else:
        self.stopping_breakpoint_recorded = False

        self.initial_stopping_accel = random.random() * -0.4 -0.1 if force_stop else CS.aEgo
        self.initial_stopping_speed = random.random() * 1. + 0.1 if force_stop else CS.vEgo

        if force_stop:
          self.prep_stopping = True
      # print(f"Starting to stop, initial accel {self.initial_stopping_accel}")

    if new_control_state in (LongCtrlState.stopping, LongCtrlState.pid) and self.prep_stopping:
      self.long_control_state = LongCtrlState.pid
    else:
      self.long_control_state = new_control_state

    if self.long_control_state == LongCtrlState.off:
      self.reset()
      self.prep_stopping = False
      output_accel = 0.

    elif self.prep_stopping == True:
      output_accel = self.initial_stopping_accel

    elif self.long_control_state == LongCtrlState.stopping:

      if not self.stopping_breakpoint_recorded and CS.vEgo < 0.4:
        self.stopping_breakpoint_recorded = True
        breakpoint_v_bp = [ -1., -0.1  ]
        breakpoint_v_v =  [  1.,  0.4 ]
        # breakpoint_v_b =  [  0.5, 0.1 ]

        self.breakpoint_v = interp(CS.aEgo, breakpoint_v_bp, breakpoint_v_v)
        # self.breakpoint_b = interp(CS.aEgo, breakpoint_v_bp, breakpoint_v_b)

      output_accel = min(output_accel, -0.1)
                    # km/h
      stopping_v_bp =      [ 0.01,   0.1,     0.4 ]
      stopping_accel_max = [-0.01,  -0.05,   -0.1 ]
      stopping_accel_min = [-0.08,  -0.2,    -0.3 ]
      stopping_v =         [ 0.08,   0.15,   self.breakpoint_v]

      max_expected_accel = interp(CS.vEgo, stopping_v_bp, stopping_accel_max)
      min_expected_accel = interp(CS.vEgo, stopping_v_bp, stopping_accel_min)

      if CS.aEgo > max_expected_accel or CS.vEgo < 0.4 and CS.aEgo < min_expected_accel:
        release_step = interp(CS.vEgo, stopping_v_bp, stopping_v)
        error_factor = 0.1 if CS.aEgo > max_expected_accel else 0.9
        error = max_expected_accel + ((min_expected_accel - max_expected_accel) * error_factor) - CS.aEgo
        step_factor = release_step if CS.aEgo < max_expected_accel or CS.aEgo > 0. else 0.1
        output_accel += error * step_factor * DT_CTRL

      output_accel = clip(output_accel, self.CP.stopAccel, -0.05)

    elif self.long_control_state == LongCtrlState.starting:
      output_accel = self.CP.startAccel
      self.reset()

    else:  # LongCtrlState.pid
      error = a_target - CS.aEgo
      output_accel = self.pid.update(error, speed=CS.vEgo,
                                     feedforward=a_target)

    self.last_output_accel = clip(output_accel, accel_limits[0], accel_limits[1])
    return self.last_output_accel
