#!/usr/bin/env python3
import math
import numpy as np

import cereal.messaging as messaging
from opendbc.car.interfaces import ACCEL_MIN, ACCEL_MAX
from openpilot.common.constants import CV
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.realtime import DT_MDL
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.car.cruise import V_CRUISE_UNSET
from openpilot.selfdrive.controls.lib.drive_helpers import CONTROL_N, get_accel_from_plan
from openpilot.selfdrive.controls.lib.longcontrol import LongCtrlState
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import LongitudinalMpc, SOURCES
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import T_IDXS as T_IDXS_MPC
from openpilot.selfdrive.modeld.constants import ModelConstants

from openpilot.frogpilot.common.frogpilot_variables import MINIMUM_LATERAL_ACCELERATION

LON_MPC_STEP = 0.2  # first step is 0.2s
A_CRUISE_MAX_VALS = [1.6, 1.2, 0.8, 0.6]
A_CRUISE_MAX_BP = [0., 10.0, 25., 40.]
CONTROL_N_T_IDX = ModelConstants.T_IDXS[:CONTROL_N]
ALLOW_THROTTLE_THRESHOLD = 0.4
MIN_ALLOW_THROTTLE_SPEED = 2.5
EXPERIMENTAL_FREE_ROAD_LEAD_TIME = 1.4
EXPERIMENTAL_FREE_ROAD_BOOST_MAX = 1.0
EXPERIMENTAL_FREE_ROAD_BOOST_GAIN_DEFAULT = 1.0
EXPERIMENTAL_FREE_ROAD_BOOST_RAMP_UP = 0.05
EXPERIMENTAL_FREE_ROAD_BOOST_RAMP_DOWN = 0.08

# Lookup table for turns
_A_TOTAL_MAX_V = [1.7, 3.2]
_A_TOTAL_MAX_BP = [20., 40.]


def get_max_accel(v_ego):
  return np.interp(v_ego, A_CRUISE_MAX_BP, A_CRUISE_MAX_VALS)


def get_coast_accel(pitch):
  return np.sin(pitch) * -5.65 - 0.3  # fitted from data using xx/projects/allow_throttle/compute_coast_accel.py


def limit_accel_in_turns(v_ego, angle_steers, a_target, CP):
  """
  This function returns a limited long acceleration allowed, depending on the existing lateral acceleration
  this should avoid accelerating when losing the target in turns
  """
  # FIXME: This function to calculate lateral accel is incorrect and should use the VehicleModel
  # The lookup table for turns should also be updated if we do this
  a_total_max = np.interp(v_ego, _A_TOTAL_MAX_BP, _A_TOTAL_MAX_V)
  a_y = v_ego ** 2 * angle_steers * CV.DEG_TO_RAD / (CP.steerRatio * CP.wheelbase)

  if abs(a_y) > MINIMUM_LATERAL_ACCELERATION:
    a_x_allowed = math.sqrt(max(a_total_max ** 2 - a_y ** 2, 0.))
  else:
    a_x_allowed = a_target[1]

  return [a_target[0], min(a_target[1], a_x_allowed)]


def rate_limit_value(current_value, target_value, up_step, down_step):
  if target_value > current_value:
    return min(target_value, current_value + up_step)
  return max(target_value, current_value - down_step)


def get_experimental_boosted_accel(experimental_base_accel, acc_reference_accel, boost):
  boosted_accel = experimental_base_accel + max(boost, 0.0)

  # Never let the added boost pull Experimental below its own native request.
  # The ACC reference only caps the extra accel we added on top.
  return min(boosted_accel, max(experimental_base_accel, acc_reference_accel))


def experimental_free_road_boost_allowed(mode, allow_throttle, should_stop, lead, v_ego):
  if mode != 'blended' or not allow_throttle or should_stop:
    return False

  if lead.status and (lead.dRel / max(v_ego, 1.0)) <= EXPERIMENTAL_FREE_ROAD_LEAD_TIME:
    return False

  return True


def get_experimental_free_road_boost_target(mode, allow_throttle, should_stop, lead, v_ego, v_cruise,
                                            experimental_base_accel, acc_reference_accel, e2e_accel, boost_gain):
  if not experimental_free_road_boost_allowed(mode, allow_throttle, should_stop, lead, v_ego):
    return 0.0

  accel_gap = max(acc_reference_accel - experimental_base_accel, 0.0)
  speed_error = max(v_cruise - v_ego, 0.0)
  if accel_gap <= 0.0 or speed_error <= 0.0:
    return 0.0

  # Allow a soft pull toward ACC while fading out once the model clearly
  # asks for braking. When a lead is already beyond the allowed time gap,
  # trust the ACC reference directly instead of suppressing the assist just
  # because cruise error is small.
  model_gate = float(np.interp(e2e_accel, [-0.35, -0.15, 0.0, 0.2], [0.0, 0.8, 0.95, 1.0]))
  speed_gate = 1.0 if lead.status else float(np.interp(speed_error, [0.0, 0.5, 2.0], [0.0, 0.4, 1.0]))
  boost_cap = min(EXPERIMENTAL_FREE_ROAD_BOOST_MAX, max(boost_gain, 0.0) * 0.8 * accel_gap)
  return min(accel_gap, boost_cap * model_gate * speed_gate)


def update_experimental_free_road_boost(current_boost, mode, allow_throttle, should_stop, lead, v_ego, v_cruise,
                                        experimental_base_accel, acc_reference_accel, e2e_accel, boost_gain):
  boost_target = get_experimental_free_road_boost_target(mode, allow_throttle, should_stop, lead, v_ego, v_cruise,
                                                         experimental_base_accel, acc_reference_accel, e2e_accel, boost_gain)
  if boost_target <= 0.0:
    return 0.0
  return rate_limit_value(current_boost, boost_target, EXPERIMENTAL_FREE_ROAD_BOOST_RAMP_UP, EXPERIMENTAL_FREE_ROAD_BOOST_RAMP_DOWN)


class LongitudinalPlanner:
  def __init__(self, CP, init_v=0.0, init_a=0.0, dt=DT_MDL):
    self.CP = CP
    self.mpc = LongitudinalMpc(dt=dt)
    self.acc_mpc = LongitudinalMpc(mode='acc', dt=dt)
    self.fcw = False
    self.dt = dt
    self.allow_throttle = True

    self.a_desired = init_a
    self.v_desired_filter = FirstOrderFilter(init_v, 2.0, self.dt)
    self.acc_a_desired = init_a
    self.acc_v_desired_filter = FirstOrderFilter(init_v, 2.0, self.dt)
    self.prev_accel_clip = [ACCEL_MIN, ACCEL_MAX]
    self.output_a_target = 0.0
    self.output_should_stop = False
    self.distance_to_stop_target_m = -1.0
    self.experimental_free_road_boost = 0.0

    self.v_desired_trajectory = np.zeros(CONTROL_N)
    self.a_desired_trajectory = np.zeros(CONTROL_N)
    self.j_desired_trajectory = np.zeros(CONTROL_N)
    self.solverExecutionTime = 0.0

  @staticmethod
  def parse_model(model_msg, v_ego, frogpilot_toggles):
    if (len(model_msg.position.x) == ModelConstants.IDX_N and
      len(model_msg.velocity.x) == ModelConstants.IDX_N and
      len(model_msg.acceleration.x) == ModelConstants.IDX_N):
      x = np.interp(T_IDXS_MPC, ModelConstants.T_IDXS, model_msg.position.x)
      v = np.interp(T_IDXS_MPC, ModelConstants.T_IDXS, model_msg.velocity.x)
      a = np.interp(T_IDXS_MPC, ModelConstants.T_IDXS, model_msg.acceleration.x)
      j = np.zeros(len(T_IDXS_MPC))
    else:
      x = np.zeros(len(T_IDXS_MPC))
      v = np.zeros(len(T_IDXS_MPC))
      a = np.zeros(len(T_IDXS_MPC))
      j = np.zeros(len(T_IDXS_MPC))

    if len(model_msg.meta.disengagePredictions.gasPressProbs) > 1:
      throttle_prob = model_msg.meta.disengagePredictions.gasPressProbs[1]
    else:
      throttle_prob = 1.0

    if frogpilot_toggles.taco_tune:
      max_lat_accel = np.interp(v_ego, [5, 10, 20], [1.5, 2.0, 3.0])
      curvatures = np.interp(T_IDXS_MPC, ModelConstants.T_IDXS, model_msg.orientationRate.z) / np.clip(v, 0.3, 100.0)
      max_v = np.sqrt(max_lat_accel / (np.abs(curvatures) + 1e-3)) - 2.0
      v = np.minimum(max_v, v)

    return x, v, a, j, throttle_prob

  def update(self, sm, frogpilot_toggles):
    mode = 'blended' if sm['selfdriveState'].experimentalMode else 'acc'
    self.mpc.mode = mode

    if len(sm['carControl'].orientationNED) == 3:
      accel_coast = get_coast_accel(sm['carControl'].orientationNED[1])
    else:
      accel_coast = ACCEL_MAX

    v_ego = sm['carState'].vEgo
    v_cruise = sm['frogpilotPlan'].vCruise
    v_cruise_initialized = sm['carState'].vCruise != V_CRUISE_UNSET

    long_control_off = sm['controlsState'].longControlState == LongCtrlState.off
    force_slow_decel = sm['controlsState'].forceDecel

    # Reset current state when not engaged, or user is controlling the speed
    reset_state = long_control_off if self.CP.openpilotLongitudinalControl else not sm['selfdriveState'].enabled
    reset_state = reset_state or not v_cruise_initialized

    # No change cost when user is controlling the speed, or when standstill
    prev_accel_constraint = not (reset_state or sm['carState'].standstill)

    acc_accel_clip = [sm['frogpilotPlan'].minAcceleration, sm['frogpilotPlan'].maxAcceleration]
    steer_angle_without_offset = sm['carState'].steeringAngleDeg - sm['liveParameters'].angleOffsetDeg
    if not sm['frogpilotPlan'].cscControllingSpeed:
      acc_accel_clip = limit_accel_in_turns(v_ego, steer_angle_without_offset, acc_accel_clip, self.CP)

    if mode == 'acc':
      accel_clip = acc_accel_clip.copy()
    else:
      accel_clip = [ACCEL_MIN, ACCEL_MAX]

    if reset_state:
      self.v_desired_filter.x = v_ego
      self.a_desired = np.clip(sm['carState'].aEgo, accel_clip[0], accel_clip[1])
      self.acc_v_desired_filter.x = v_ego
      self.acc_a_desired = np.clip(sm['carState'].aEgo, acc_accel_clip[0], acc_accel_clip[1])
      self.experimental_free_road_boost = 0.0

    self.v_desired_filter.x = max(0.0, self.v_desired_filter.update(v_ego))
    x, v, a, j, throttle_prob = self.parse_model(sm['modelV2'], v_ego, frogpilot_toggles)
    self.allow_throttle = throttle_prob > ALLOW_THROTTLE_THRESHOLD or v_ego <= MIN_ALLOW_THROTTLE_SPEED

    if not self.allow_throttle:
      clipped_accel_coast = max(accel_coast, accel_clip[0])
      clipped_accel_coast_interp = np.interp(v_ego, [MIN_ALLOW_THROTTLE_SPEED, MIN_ALLOW_THROTTLE_SPEED * 2],
                                             [accel_clip[1], clipped_accel_coast])
      accel_clip[1] = min(accel_clip[1], clipped_accel_coast_interp)

    if force_slow_decel:
      v_cruise = 0.0

    self.mpc.set_weights(
      sm['frogpilotPlan'].accelerationJerk,
      sm['frogpilotPlan'].dangerJerk,
      sm['frogpilotPlan'].speedJerk,
      prev_accel_constraint,
      personality=sm['selfdriveState'].personality,
    )
    self.mpc.set_cur_state(self.v_desired_filter.x, self.a_desired)
    self.mpc.update(
      sm['radarState'],
      v_cruise,
      x,
      v,
      a,
      j,
      sm['frogpilotPlan'].tFollow,
      personality=sm['selfdriveState'].personality,
      short_distance_factor=frogpilot_toggles.short_distance_factor,
      long_distance_factor=frogpilot_toggles.long_distance_factor,
      increased_stopped_distance=sm['frogpilotPlan'].increasedStoppedDistance,
    )

    self.v_desired_trajectory = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC, self.mpc.v_solution)
    self.a_desired_trajectory = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC, self.mpc.a_solution)
    self.j_desired_trajectory = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC[:-1], self.mpc.j_solution)
    self.distance_to_stop_target_m = float(getattr(self.mpc, "distance_to_stop_target_m", -1.0))

    self.fcw = self.mpc.crash_cnt > 2 and not sm['carState'].standstill
    if self.fcw:
      cloudlog.info("FCW triggered")

    a_prev = self.a_desired
    self.a_desired = float(np.interp(self.dt, CONTROL_N_T_IDX, self.a_desired_trajectory))
    self.v_desired_filter.x = self.v_desired_filter.x + self.dt * (self.a_desired + a_prev) / 2.0

    action_t = frogpilot_toggles.longitudinalActuatorDelay + DT_MDL
    output_a_target_mpc, output_should_stop_mpc = get_accel_from_plan(
      self.v_desired_trajectory,
      self.a_desired_trajectory,
      CONTROL_N_T_IDX,
      action_t=action_t,
      vEgoStopping=frogpilot_toggles.vEgoStopping,
    )
    output_a_target_e2e = sm['modelV2'].action.desiredAcceleration
    output_should_stop_e2e = sm['modelV2'].action.shouldStop

    if mode == 'acc':
      output_a_target = output_a_target_mpc
      self.output_should_stop = output_should_stop_mpc
      self.experimental_free_road_boost = 0.0
      self.acc_v_desired_filter.x = self.v_desired_filter.x
      self.acc_a_desired = self.a_desired
    else:
      self.acc_mpc.set_weights(
        sm['frogpilotPlan'].accelerationJerk,
        sm['frogpilotPlan'].dangerJerk,
        sm['frogpilotPlan'].speedJerk,
        prev_accel_constraint,
        personality=sm['selfdriveState'].personality,
      )
      self.acc_v_desired_filter.x = max(0.0, self.acc_v_desired_filter.update(v_ego))
      self.acc_mpc.set_cur_state(self.acc_v_desired_filter.x, self.acc_a_desired)
      self.acc_mpc.update(
        sm['radarState'],
        v_cruise,
        x,
        v,
        a,
        j,
        sm['frogpilotPlan'].tFollow,
        personality=sm['selfdriveState'].personality,
        short_distance_factor=frogpilot_toggles.short_distance_factor,
        long_distance_factor=frogpilot_toggles.long_distance_factor,
        increased_stopped_distance=sm['frogpilotPlan'].increasedStoppedDistance,
      )
      acc_v_desired_trajectory = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC, self.acc_mpc.v_solution)
      acc_a_desired_trajectory = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC, self.acc_mpc.a_solution)
      acc_a_prev = self.acc_a_desired
      self.acc_a_desired = float(np.interp(self.dt, CONTROL_N_T_IDX, acc_a_desired_trajectory))
      self.acc_v_desired_filter.x = self.acc_v_desired_filter.x + self.dt * (self.acc_a_desired + acc_a_prev) / 2.0
      output_a_target_acc, _ = get_accel_from_plan(
        acc_v_desired_trajectory,
        acc_a_desired_trajectory,
        CONTROL_N_T_IDX,
        action_t=action_t,
        vEgoStopping=frogpilot_toggles.vEgoStopping,
      )
      output_a_target_acc = float(np.clip(output_a_target_acc, acc_accel_clip[0], acc_accel_clip[1]))
      experimental_base_a_target = min(output_a_target_mpc, output_a_target_e2e)
      self.output_should_stop = output_should_stop_e2e or output_should_stop_mpc
      self.experimental_free_road_boost = update_experimental_free_road_boost(
        self.experimental_free_road_boost,
        mode,
        self.allow_throttle,
        self.output_should_stop,
        sm['radarState'].leadOne,
        v_ego,
        v_cruise,
        experimental_base_a_target,
        output_a_target_acc,
        output_a_target_e2e,
        getattr(frogpilot_toggles, "experimental_free_road_boost_gain", EXPERIMENTAL_FREE_ROAD_BOOST_GAIN_DEFAULT),
      )
      output_a_target = get_experimental_boosted_accel(experimental_base_a_target, output_a_target_acc, self.experimental_free_road_boost)
      if experimental_base_a_target < output_a_target_mpc and output_a_target <= experimental_base_a_target:
        self.mpc.source = SOURCES[3]

    if sm['frogpilotCarState'].forceCoast and sm['carState'].standstill:
      self.output_should_stop = True
      output_a_target = min(output_a_target, 0.0)

    for idx in range(2):
      accel_clip[idx] = np.clip(accel_clip[idx], self.prev_accel_clip[idx] - 0.05, self.prev_accel_clip[idx] + 0.05)
    self.output_a_target = np.clip(output_a_target, accel_clip[0], accel_clip[1])
    self.prev_accel_clip = accel_clip

  def publish(self, sm, pm):
    plan_send = messaging.new_message('longitudinalPlan')

    plan_send.valid = sm.all_checks(service_list=['carState', 'controlsState', 'selfdriveState', 'radarState'])

    longitudinalPlan = plan_send.longitudinalPlan
    longitudinalPlan.modelMonoTime = sm.logMonoTime['modelV2']
    longitudinalPlan.processingDelay = (plan_send.logMonoTime / 1e9) - sm.logMonoTime['modelV2']
    longitudinalPlan.solverExecutionTime = self.mpc.solve_time

    longitudinalPlan.speeds = self.v_desired_trajectory.tolist()
    longitudinalPlan.accels = self.a_desired_trajectory.tolist()
    longitudinalPlan.jerks = self.j_desired_trajectory.tolist()

    longitudinalPlan.hasLead = sm['radarState'].leadOne.status
    longitudinalPlan.longitudinalPlanSource = self.mpc.source
    longitudinalPlan.fcw = self.fcw

    longitudinalPlan.aTarget = float(self.output_a_target)
    longitudinalPlan.shouldStop = bool(self.output_should_stop)
    longitudinalPlan.allowBrake = True
    longitudinalPlan.allowThrottle = bool(self.allow_throttle)
    longitudinalPlan.distanceToStopTarget = float(self.distance_to_stop_target_m)

    pm.send('longitudinalPlan', plan_send)
