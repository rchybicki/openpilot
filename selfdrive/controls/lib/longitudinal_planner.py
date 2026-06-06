#!/usr/bin/env python3
import math
import numpy as np

import cereal.messaging as messaging
from opendbc.car.hyundai.values import CAR as HYUNDAI_CAR
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
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.stop_target_helpers import LEAD_STOP_DISTANCE_TARGET
from openpilot.selfdrive.modeld.constants import ModelConstants

from openpilot.frogpilot.common.frogpilot_variables import MINIMUM_LATERAL_ACCELERATION
from openpilot.frogpilot.controls.lib.force_coast import get_force_coast_target_from_toggles

LON_MPC_STEP = 0.2  # first step is 0.2s
A_CRUISE_MAX_VALS = [2.0, 1.6, 0.8, 0.6]
A_CRUISE_MAX_BP = [0., 10.0, 25., 40.]
CONTROL_N_T_IDX = ModelConstants.T_IDXS[:CONTROL_N]
ALLOW_THROTTLE_THRESHOLD = 0.4
MIN_ALLOW_THROTTLE_SPEED = 2.5
EXPERIMENTAL_FREE_ROAD_LEAD_TIME = 1.4
EXPERIMENTAL_FREE_ROAD_LEAD_TIME_BP = [0.0, 15.0 * CV.KPH_TO_MS, 30.0 * CV.KPH_TO_MS, 50.0 * CV.KPH_TO_MS]
EXPERIMENTAL_FREE_ROAD_LEAD_TIME_VALS = [2.5, 2.3, 1.9, EXPERIMENTAL_FREE_ROAD_LEAD_TIME]
EXPERIMENTAL_FREE_ROAD_LEAD_BOOST_MAX = 1.1
EXPERIMENTAL_FREE_ROAD_NO_LEAD_BOOST_MAX = 0.6
EXPERIMENTAL_FREE_ROAD_LEAD_BOOST_GAIN_DEFAULT = 1.0
EXPERIMENTAL_FREE_ROAD_NO_LEAD_BOOST_GAIN_DEFAULT = 0.5
EXPERIMENTAL_FREE_ROAD_BRAKE_CUTOFF_DEFAULT = -0.2
EXPERIMENTAL_FREE_ROAD_LEAD_BOOST_SCALE = 0.9
EXPERIMENTAL_FREE_ROAD_NO_LEAD_BOOST_SCALE = 0.8
EXPERIMENTAL_FREE_ROAD_NO_LEAD_SPEED_GATE_BP = [0.0, 0.5, 2.0]
EXPERIMENTAL_FREE_ROAD_NO_LEAD_SPEED_GATE_VALS = [0.0, 0.4, 1.0]
EXPERIMENTAL_FREE_ROAD_NO_LEAD_SPEED_GATE_STRENGTH = 1.25
EXPERIMENTAL_FREE_ROAD_LEAD_SPEED_GATE_BP = [0.0, 5.0 * CV.KPH_TO_MS, 10.0 * CV.KPH_TO_MS, 20.0 * CV.KPH_TO_MS, 35.0 * CV.KPH_TO_MS, 50.0 * CV.KPH_TO_MS]
EXPERIMENTAL_FREE_ROAD_LEAD_SPEED_GATE_VALS = [0.25, 0.3, 0.4, 0.55, 0.8, 1.0]
EXPERIMENTAL_FREE_ROAD_LEAD_STANDSTILL_GAP_BP = [0.0, 15.0 * CV.KPH_TO_MS, 30.0 * CV.KPH_TO_MS, 50.0 * CV.KPH_TO_MS]
EXPERIMENTAL_FREE_ROAD_LEAD_STANDSTILL_GAP_VALS = [4.0, 4.0, 2.0, 0.0]
EXPERIMENTAL_FREE_ROAD_LEAD_GAP_MARGIN_BP = [0.0, 1.0, 2.0, 4.0]
EXPERIMENTAL_FREE_ROAD_LEAD_GAP_MARGIN_VALS = [0.0, 0.55, 0.8, 1.0]
EXPERIMENTAL_FREE_ROAD_LEAD_PULLAWAY_SPEED_BP = [0.0, 0.5, 1.5, 3.0]
EXPERIMENTAL_FREE_ROAD_LEAD_PULLAWAY_SPEED_VALS = [0.0, 0.2, 0.6, 1.0]
EXPERIMENTAL_FREE_ROAD_LEAD_PULLAWAY_ACCEL_BP = [-0.2, 0.0, 0.3, 1.0]
EXPERIMENTAL_FREE_ROAD_LEAD_PULLAWAY_ACCEL_VALS = [0.0, 0.2, 0.5, 1.0]
EXPERIMENTAL_FREE_ROAD_LEAD_PULLAWAY_SPEED_INFLUENCE_BP = [0.0, 10.0 * CV.KPH_TO_MS, 30.0 * CV.KPH_TO_MS, 50.0 * CV.KPH_TO_MS]
EXPERIMENTAL_FREE_ROAD_LEAD_PULLAWAY_SPEED_INFLUENCE_VALS = [1.0, 1.0, 0.5, 0.0]
EXPERIMENTAL_FREE_ROAD_LEAD_PULLAWAY_GATE_STRENGTH = 0.5
EXPERIMENTAL_FREE_ROAD_BOOST_RAMP_UP = 0.05
EXPERIMENTAL_FREE_ROAD_BOOST_RAMP_DOWN = 0.08
SANTA_FE_EXPERIMENTAL_LEAD_CAUTION_MAX = 0.45
SANTA_FE_EXPERIMENTAL_LEAD_CAUTION_MAX_SPEED = 12.5
SANTA_FE_EXPERIMENTAL_LEAD_CAUTION_GAP_BP = [0.8, 1.1, 1.8, 2.4, 3.0]
SANTA_FE_EXPERIMENTAL_LEAD_CAUTION_GAP_VALS = [1.0, 1.0, 0.75, 0.2, 0.0]
SANTA_FE_EXPERIMENTAL_LEAD_CAUTION_SPEED_BP = [4.5, 6.0, 8.0, 12.5]
SANTA_FE_EXPERIMENTAL_LEAD_CAUTION_SPEED_VALS = [0.35, 0.55, 0.85, 1.0]
SANTA_FE_EXPERIMENTAL_LEAD_CAUTION_REQUEST_BP = [-3.0, -2.2, -1.5, -0.6, 0.2, 0.6]
SANTA_FE_EXPERIMENTAL_LEAD_CAUTION_REQUEST_VALS = [0.35, 0.5, 0.75, 1.0, 0.55, 0.0]
SANTA_FE_EXPERIMENTAL_LEAD_CAUTION_CLOSING_BP = [0.2, 0.8, 2.0, 4.0]
SANTA_FE_EXPERIMENTAL_LEAD_CAUTION_CLOSING_VALS = [0.0, 0.35, 0.8, 1.0]
SANTA_FE_EXPERIMENTAL_LEAD_CAUTION_TTC_BP = [1.0, 1.8, 2.6, 3.6, 5.0]
SANTA_FE_EXPERIMENTAL_LEAD_CAUTION_TTC_VALS = [1.0, 1.0, 0.7, 0.35, 0.0]
SANTA_FE_EXPERIMENTAL_LEAD_CAUTION_LEAD_SPEED_BP = [0.0, 0.4, 0.7, 1.0]
SANTA_FE_EXPERIMENTAL_LEAD_CAUTION_LEAD_SPEED_VALS = [1.0, 1.0, 0.25, 0.0]
SANTA_FE_EXPERIMENTAL_DECEL_LEAD_MAX_SPEED = 16.0
SANTA_FE_EXPERIMENTAL_DECEL_LEAD_GAP_BP = [1.55, 2.10, 2.70, 3.50]
SANTA_FE_EXPERIMENTAL_DECEL_LEAD_GAP_CAPS = [-0.72, -0.46, -0.18, 0.05]
SANTA_FE_EXPERIMENTAL_DECEL_LEAD_CLOSING_BP = [1.20, 2.00, 3.50, 5.00]
SANTA_FE_EXPERIMENTAL_DECEL_LEAD_CLOSING_TIGHTEN = [0.00, 0.04, 0.12, 0.18]
SANTA_FE_EXPERIMENTAL_DECEL_LEAD_DECEL_BP = [0.00, 0.40, 0.90, 1.50]
SANTA_FE_EXPERIMENTAL_DECEL_LEAD_DECEL_TIGHTEN = [0.00, 0.00, 0.05, 0.11]
SANTA_FE_EXPERIMENTAL_DECEL_LEAD_TTC_BP = [2.50, 4.00, 7.00, 10.00]
SANTA_FE_EXPERIMENTAL_DECEL_LEAD_TTC_TIGHTEN = [0.13, 0.08, 0.02, 0.00]
SANTA_FE_STOPPED_LEAD_SMOOTH_APPROACH_SPEED_BP = [2.50, 5.00, 8.00, 12.50]
SANTA_FE_STOPPED_LEAD_SMOOTH_APPROACH_MAX_DECEL = [1.05, 1.55, 2.05, 2.35]
SANTA_FE_STOPPED_LEAD_SMOOTH_APPROACH_BUFFER_M = [0.35, 0.75, 1.15, 1.65]
SANTA_FE_STOPPED_LEAD_SMOOTH_APPROACH_MIN_CLOSING = [0.55, 0.95, 1.45, 2.20]

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


def apply_experimental_force_coast_cap(output_a_target, acc_reference_accel, force_coast):
  if not force_coast:
    return output_a_target

  return min(output_a_target, acc_reference_accel)


def should_allow_force_coast_stronger_lead_brake(v_ego, lead, output_should_stop):
  if output_should_stop:
    return True
  if not lead.status:
    return False

  d_rel = float(lead.dRel)
  if d_rel <= 0.0:
    return False

  v_rel = float(lead.vRel)
  v_lead = max(float(getattr(lead, "vLead", v_ego + v_rel)), 0.0)
  closing_speed = max(-v_rel, 0.0)
  time_gap = d_rel / max(v_ego, 1.0)
  ttc = d_rel / max(closing_speed, 0.1) if closing_speed > 0.1 else float("inf")

  stopped_lead_close = v_lead < 0.5 and d_rel < max(6.0, v_ego * 1.2)
  return time_gap < 2.0 or ttc < 4.0 or stopped_lead_close


def apply_force_coast_strength_brake_limit(output_a_target, force_coast_target_accel, force_coast, v_ego, lead, output_should_stop, model_accel):
  if not force_coast or output_a_target >= force_coast_target_accel:
    return output_a_target
  if should_allow_force_coast_stronger_lead_brake(v_ego, lead, output_should_stop):
    return output_a_target

  brake_limit = force_coast_target_accel
  if model_accel is not None:
    brake_limit = min(brake_limit, float(model_accel))
  return max(output_a_target, brake_limit)


def get_experimental_free_road_boost_limits(lead, lead_boost_gain, no_lead_boost_gain):
  if lead.status:
    return EXPERIMENTAL_FREE_ROAD_LEAD_BOOST_MAX, EXPERIMENTAL_FREE_ROAD_LEAD_BOOST_SCALE, max(lead_boost_gain, 0.0)

  return EXPERIMENTAL_FREE_ROAD_NO_LEAD_BOOST_MAX, EXPERIMENTAL_FREE_ROAD_NO_LEAD_BOOST_SCALE, max(no_lead_boost_gain, 0.0)


def get_experimental_free_road_model_gate(e2e_accel, brake_cutoff):
  zero_boost_point = min(float(brake_cutoff), -0.02)
  mild_brake_point = zero_boost_point * 0.5
  coast_point = min(max(zero_boost_point * 0.1, -0.02), 0.0)

  return float(np.interp(e2e_accel, [zero_boost_point, mild_brake_point, coast_point, 0.2], [0.0, 0.25, 0.6, 1.0]))


def get_experimental_free_road_lead_time_threshold(v_ego):
  return float(np.interp(v_ego, EXPERIMENTAL_FREE_ROAD_LEAD_TIME_BP, EXPERIMENTAL_FREE_ROAD_LEAD_TIME_VALS))


def get_experimental_free_road_lead_speed_gate(v_ego):
  return float(np.interp(v_ego, EXPERIMENTAL_FREE_ROAD_LEAD_SPEED_GATE_BP, EXPERIMENTAL_FREE_ROAD_LEAD_SPEED_GATE_VALS))


def get_experimental_free_road_no_lead_speed_gate(speed_error):
  raw_gate = float(np.interp(speed_error, EXPERIMENTAL_FREE_ROAD_NO_LEAD_SPEED_GATE_BP,
                             EXPERIMENTAL_FREE_ROAD_NO_LEAD_SPEED_GATE_VALS))
  return min(1.0, raw_gate * EXPERIMENTAL_FREE_ROAD_NO_LEAD_SPEED_GATE_STRENGTH)


def get_experimental_free_road_lead_gap_gate(lead, v_ego):
  standstill_gap = float(np.interp(v_ego, EXPERIMENTAL_FREE_ROAD_LEAD_STANDSTILL_GAP_BP,
                                   EXPERIMENTAL_FREE_ROAD_LEAD_STANDSTILL_GAP_VALS))
  desired_gap = standstill_gap + (v_ego * get_experimental_free_road_lead_time_threshold(v_ego))
  gap_margin = float(lead.dRel) - desired_gap
  return float(np.interp(gap_margin, EXPERIMENTAL_FREE_ROAD_LEAD_GAP_MARGIN_BP, EXPERIMENTAL_FREE_ROAD_LEAD_GAP_MARGIN_VALS))


def get_experimental_free_road_lead_pullaway_gate(lead, v_ego):
  relative_speed = max(float(lead.vLead) - float(v_ego), 0.0)
  relative_speed_gate = float(np.interp(relative_speed, EXPERIMENTAL_FREE_ROAD_LEAD_PULLAWAY_SPEED_BP, EXPERIMENTAL_FREE_ROAD_LEAD_PULLAWAY_SPEED_VALS))
  lead_accel_gate = float(np.interp(float(lead.aLeadK), EXPERIMENTAL_FREE_ROAD_LEAD_PULLAWAY_ACCEL_BP, EXPERIMENTAL_FREE_ROAD_LEAD_PULLAWAY_ACCEL_VALS))

  # Fade the pull-away gating out by 50 kph so higher-speed following keeps the
  # existing lead behavior, while stop-and-go becomes much less eager.
  speed_influence = float(np.interp(v_ego, EXPERIMENTAL_FREE_ROAD_LEAD_PULLAWAY_SPEED_INFLUENCE_BP,
                                    EXPERIMENTAL_FREE_ROAD_LEAD_PULLAWAY_SPEED_INFLUENCE_VALS))
  pullaway_gate = relative_speed_gate * lead_accel_gate
  gated_pullaway = (1.0 - speed_influence) + (speed_influence * pullaway_gate)
  return 1.0 - (EXPERIMENTAL_FREE_ROAD_LEAD_PULLAWAY_GATE_STRENGTH * (1.0 - gated_pullaway))


def experimental_free_road_boost_allowed(mode, allow_throttle, should_stop, force_coast, lead, v_ego):
  if mode != 'blended' or not allow_throttle or should_stop or force_coast:
    return False

  if lead.status and get_experimental_free_road_lead_gap_gate(lead, v_ego) <= 0.0:
    return False

  return True


def get_experimental_free_road_boost_target(mode, allow_throttle, should_stop, force_coast, lead, v_ego, v_cruise,
                                            experimental_base_accel, acc_reference_accel, e2e_accel, lead_boost_gain, no_lead_boost_gain,
                                            brake_cutoff=EXPERIMENTAL_FREE_ROAD_BRAKE_CUTOFF_DEFAULT):
  if not experimental_free_road_boost_allowed(mode, allow_throttle, should_stop, force_coast, lead, v_ego):
    return 0.0

  accel_gap = max(acc_reference_accel - experimental_base_accel, 0.0)
  speed_error = max(v_cruise - v_ego, 0.0)
  if accel_gap <= 0.0 or speed_error <= 0.0:
    return 0.0

  # Allow a soft pull toward ACC while fading out once the model clearly
  # asks for braking. When a lead is already beyond the allowed time gap,
  # trust the ACC reference directly instead of suppressing the assist just
  # because cruise error is small. At stop-and-go speeds, still taper lead
  # boost down to avoid jumping at a moving lead and then braking again.
  model_gate = get_experimental_free_road_model_gate(e2e_accel, brake_cutoff)
  if lead.status:
    speed_gate = (get_experimental_free_road_lead_speed_gate(v_ego) *
                  get_experimental_free_road_lead_gap_gate(lead, v_ego) *
                  get_experimental_free_road_lead_pullaway_gate(lead, v_ego))
  else:
    speed_gate = get_experimental_free_road_no_lead_speed_gate(speed_error)
  boost_max, boost_scale, boost_gain = get_experimental_free_road_boost_limits(lead, lead_boost_gain, no_lead_boost_gain)
  boost_cap = min(boost_max, boost_gain * boost_scale * accel_gap)
  return min(accel_gap, boost_cap * model_gate * speed_gate)


def update_experimental_free_road_boost(current_boost, mode, allow_throttle, should_stop, force_coast, lead, v_ego, v_cruise,
                                        experimental_base_accel, acc_reference_accel, e2e_accel, lead_boost_gain, no_lead_boost_gain,
                                        brake_cutoff=EXPERIMENTAL_FREE_ROAD_BRAKE_CUTOFF_DEFAULT):
  boost_target = get_experimental_free_road_boost_target(mode, allow_throttle, should_stop, force_coast, lead, v_ego, v_cruise,
                                                         experimental_base_accel, acc_reference_accel, e2e_accel, lead_boost_gain,
                                                         no_lead_boost_gain, brake_cutoff)
  if boost_target <= 0.0:
    return 0.0
  return rate_limit_value(current_boost, boost_target, EXPERIMENTAL_FREE_ROAD_BOOST_RAMP_UP, EXPERIMENTAL_FREE_ROAD_BOOST_RAMP_DOWN)


def is_santa_fe_hev_2022(cp):
  return getattr(cp, "carFingerprint", None) == HYUNDAI_CAR.HYUNDAI_SANTA_FE_HEV_2022


def get_santa_fe_experimental_lead_caution_decel(v_ego, lead, output_a_target):
  if v_ego < SANTA_FE_EXPERIMENTAL_LEAD_CAUTION_SPEED_BP[0] or v_ego > SANTA_FE_EXPERIMENTAL_LEAD_CAUTION_MAX_SPEED:
    return 0.0
  if not lead.status:
    return 0.0

  d_rel = float(lead.dRel)
  v_rel = float(lead.vRel)
  v_lead = max(float(getattr(lead, "vLead", v_ego + v_rel)), 0.0)
  if d_rel <= 0.0:
    return 0.0

  time_gap = d_rel / max(v_ego, 1.0)
  closing_speed = max(-v_rel, 0.0)
  ttc = d_rel / max(closing_speed, 0.1) if closing_speed > 0.1 else float("inf")

  gap_factor = float(np.interp(time_gap, SANTA_FE_EXPERIMENTAL_LEAD_CAUTION_GAP_BP, SANTA_FE_EXPERIMENTAL_LEAD_CAUTION_GAP_VALS))
  if gap_factor <= 0.0:
    return 0.0

  speed_factor = float(np.interp(v_ego, SANTA_FE_EXPERIMENTAL_LEAD_CAUTION_SPEED_BP, SANTA_FE_EXPERIMENTAL_LEAD_CAUTION_SPEED_VALS))
  request_factor = float(np.interp(output_a_target, SANTA_FE_EXPERIMENTAL_LEAD_CAUTION_REQUEST_BP, SANTA_FE_EXPERIMENTAL_LEAD_CAUTION_REQUEST_VALS))
  lead_stopped_factor = float(np.interp(v_lead, SANTA_FE_EXPERIMENTAL_LEAD_CAUTION_LEAD_SPEED_BP, SANTA_FE_EXPERIMENTAL_LEAD_CAUTION_LEAD_SPEED_VALS))
  if speed_factor <= 0.0 or request_factor <= 0.0 or lead_stopped_factor <= 0.0:
    return 0.0

  closing_factor = float(np.interp(closing_speed, SANTA_FE_EXPERIMENTAL_LEAD_CAUTION_CLOSING_BP, SANTA_FE_EXPERIMENTAL_LEAD_CAUTION_CLOSING_VALS))
  ttc_factor = 0.0 if not math.isfinite(ttc) else float(np.interp(ttc, SANTA_FE_EXPERIMENTAL_LEAD_CAUTION_TTC_BP, SANTA_FE_EXPERIMENTAL_LEAD_CAUTION_TTC_VALS))

  risk_factor = speed_factor * request_factor * gap_factor * lead_stopped_factor * closing_factor * ttc_factor
  return float(np.clip(SANTA_FE_EXPERIMENTAL_LEAD_CAUTION_MAX * risk_factor, 0.0, SANTA_FE_EXPERIMENTAL_LEAD_CAUTION_MAX))


def get_santa_fe_experimental_decelerating_lead_approach_cap(v_ego, lead):
  if v_ego < SANTA_FE_EXPERIMENTAL_LEAD_CAUTION_SPEED_BP[0] or v_ego > SANTA_FE_EXPERIMENTAL_DECEL_LEAD_MAX_SPEED:
    return None
  if not lead.status:
    return None

  d_rel = float(lead.dRel)
  v_rel = float(lead.vRel)
  if d_rel <= 0.0:
    return None

  closing_speed = max(-v_rel, 0.0)
  if closing_speed < SANTA_FE_EXPERIMENTAL_DECEL_LEAD_CLOSING_BP[0]:
    return None

  time_gap = d_rel / max(v_ego, 1.0)
  lead_decel = max(-float(getattr(lead, "aLeadK", 0.0)), 0.0)
  if time_gap > SANTA_FE_EXPERIMENTAL_DECEL_LEAD_GAP_BP[-1]:
    return None

  ttc = d_rel / max(closing_speed, 0.1)
  gap_cap = float(np.interp(time_gap, SANTA_FE_EXPERIMENTAL_DECEL_LEAD_GAP_BP, SANTA_FE_EXPERIMENTAL_DECEL_LEAD_GAP_CAPS))
  closing_tighten = float(np.interp(closing_speed, SANTA_FE_EXPERIMENTAL_DECEL_LEAD_CLOSING_BP,
                                    SANTA_FE_EXPERIMENTAL_DECEL_LEAD_CLOSING_TIGHTEN))
  lead_decel_tighten = float(np.interp(lead_decel, SANTA_FE_EXPERIMENTAL_DECEL_LEAD_DECEL_BP,
                                       SANTA_FE_EXPERIMENTAL_DECEL_LEAD_DECEL_TIGHTEN))
  ttc_tighten = float(np.interp(ttc, SANTA_FE_EXPERIMENTAL_DECEL_LEAD_TTC_BP, SANTA_FE_EXPERIMENTAL_DECEL_LEAD_TTC_TIGHTEN))
  return float(np.clip(gap_cap - closing_tighten - lead_decel_tighten - ttc_tighten, -0.85, 0.05))


def apply_santa_fe_experimental_decelerating_lead_approach_cap(output_a_target, v_ego, lead):
  cap = get_santa_fe_experimental_decelerating_lead_approach_cap(v_ego, lead)
  if cap is None or output_a_target <= cap:
    return output_a_target

  return cap


def apply_santa_fe_experimental_lead_caution(output_a_target, v_ego, lead):
  extra_decel = get_santa_fe_experimental_lead_caution_decel(v_ego, lead, output_a_target)
  if extra_decel <= 0.0:
    return output_a_target

  return output_a_target - extra_decel


def get_santa_fe_stopped_lead_smooth_approach_cap(v_ego, lead, increased_stopped_distance=0.0, lead_stop_distance_target=LEAD_STOP_DISTANCE_TARGET):
  if v_ego < SANTA_FE_STOPPED_LEAD_SMOOTH_APPROACH_SPEED_BP[0] or v_ego > SANTA_FE_STOPPED_LEAD_SMOOTH_APPROACH_SPEED_BP[-1]:
    return None
  if not lead.status:
    return None

  d_rel = float(lead.dRel)
  if d_rel <= 0.0:
    return None

  v_rel = float(getattr(lead, "vRel", 0.0))
  lead_v = max(float(getattr(lead, "vLead", v_ego + v_rel)), 0.0)
  lead_v_limit = float(np.interp(v_ego, [2.50, 5.00, 8.00, 12.50], [0.55, 0.50, 0.45, 0.35]))
  if lead_v > lead_v_limit:
    return None

  closing_speed = max(v_ego - lead_v, 0.0)
  min_closing = float(np.interp(v_ego, SANTA_FE_STOPPED_LEAD_SMOOTH_APPROACH_SPEED_BP, SANTA_FE_STOPPED_LEAD_SMOOTH_APPROACH_MIN_CLOSING))
  if closing_speed < min_closing:
    return None

  remaining_to_hold_gap = d_rel + float(increased_stopped_distance) - float(lead_stop_distance_target)
  if remaining_to_hold_gap <= 0.0:
    return None

  buffer_m = float(np.interp(v_ego, SANTA_FE_STOPPED_LEAD_SMOOTH_APPROACH_SPEED_BP, SANTA_FE_STOPPED_LEAD_SMOOTH_APPROACH_BUFFER_M))
  braking_distance = max(remaining_to_hold_gap - buffer_m, 0.75)
  required_decel = (v_ego * v_ego) / (2.0 * braking_distance)
  min_meaningful_decel = float(np.interp(v_ego, SANTA_FE_STOPPED_LEAD_SMOOTH_APPROACH_SPEED_BP, [0.55, 0.75, 1.00, 1.20]))
  if required_decel < min_meaningful_decel:
    return None

  max_decel = float(np.interp(v_ego, SANTA_FE_STOPPED_LEAD_SMOOTH_APPROACH_SPEED_BP, SANTA_FE_STOPPED_LEAD_SMOOTH_APPROACH_MAX_DECEL))
  return -float(np.clip(required_decel, min_meaningful_decel, max_decel))


def apply_santa_fe_stopped_lead_smooth_approach_cap(output_a_target, v_ego, lead, increased_stopped_distance=0.0,
                                                    lead_stop_distance_target=LEAD_STOP_DISTANCE_TARGET):
  cap = get_santa_fe_stopped_lead_smooth_approach_cap(v_ego, lead, increased_stopped_distance, lead_stop_distance_target)
  if cap is None or output_a_target <= cap:
    return output_a_target

  return cap


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
      v_cruise,
      sm['modelV2'],
      sm['radarState'],
      x,
      v,
      a,
      j,
      sm['frogpilotPlan'].dangerFactor,
      sm['frogpilotPlan'].tFollow,
      accel_clip[0],
      accel_clip[1],
      frogpilot_toggles,
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
        v_cruise,
        sm['modelV2'],
        sm['radarState'],
        x,
        v,
        a,
        j,
        sm['frogpilotPlan'].dangerFactor,
        sm['frogpilotPlan'].tFollow,
        acc_accel_clip[0],
        acc_accel_clip[1],
        frogpilot_toggles,
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
        sm['frogpilotCarState'].forceCoast,
        sm['radarState'].leadOne,
        v_ego,
        v_cruise,
        experimental_base_a_target,
        output_a_target_acc,
        output_a_target_e2e,
        getattr(frogpilot_toggles, "experimental_lead_boost_gain", EXPERIMENTAL_FREE_ROAD_LEAD_BOOST_GAIN_DEFAULT),
        getattr(frogpilot_toggles, "experimental_no_lead_boost_gain", EXPERIMENTAL_FREE_ROAD_NO_LEAD_BOOST_GAIN_DEFAULT),
        getattr(frogpilot_toggles, "experimental_boost_brake_cutoff", EXPERIMENTAL_FREE_ROAD_BRAKE_CUTOFF_DEFAULT),
      )
      output_a_target = get_experimental_boosted_accel(experimental_base_a_target, output_a_target_acc, self.experimental_free_road_boost)
      output_a_target = apply_experimental_force_coast_cap(output_a_target, output_a_target_acc, sm['frogpilotCarState'].forceCoast)
      if is_santa_fe_hev_2022(self.CP):
        output_a_target = apply_santa_fe_experimental_decelerating_lead_approach_cap(output_a_target, v_ego, sm['radarState'].leadOne)
        output_a_target = apply_santa_fe_experimental_lead_caution(output_a_target, v_ego, sm['radarState'].leadOne)
        output_a_target = apply_santa_fe_stopped_lead_smooth_approach_cap(
          output_a_target,
          v_ego,
          sm['radarState'].leadOne,
          increased_stopped_distance=sm['frogpilotPlan'].increasedStoppedDistance,
        )
      if experimental_base_a_target < output_a_target_mpc and output_a_target <= experimental_base_a_target:
        self.mpc.source = SOURCES[3]

    if sm['frogpilotCarState'].forceCoast:
      force_coast_target_accel = get_force_coast_target_from_toggles(v_ego, frogpilot_toggles)
      output_a_target = apply_force_coast_strength_brake_limit(output_a_target, force_coast_target_accel, True, v_ego,
                                                               sm['radarState'].leadOne, self.output_should_stop,
                                                               output_a_target_e2e)

    if sm['frogpilotCarState'].forceCoast and sm['carState'].standstill:
      self.output_should_stop = True
      output_a_target = min(output_a_target, 0.0)

    for idx in range(2):
      accel_clip[idx] = np.clip(accel_clip[idx], self.prev_accel_clip[idx] - 0.05, self.prev_accel_clip[idx] + 0.05)
    self.output_a_target = np.clip(output_a_target, accel_clip[0], accel_clip[1])
    self.prev_accel_clip = accel_clip

  def publish(self, sm, pm, frogpilot_toggles):
    plan_send = messaging.new_message('longitudinalPlan')

    plan_send.valid = sm.all_checks(service_list=['carState', 'controlsState', 'selfdriveState', 'radarState'])

    longitudinalPlan = plan_send.longitudinalPlan
    longitudinalPlan.modelMonoTime = sm.logMonoTime['modelV2']
    longitudinalPlan.processingDelay = (plan_send.logMonoTime / 1e9) - sm.logMonoTime['modelV2']
    longitudinalPlan.solverExecutionTime = self.mpc.solve_time

    longitudinalPlan.speeds = self.v_desired_trajectory.tolist()
    longitudinalPlan.accels = self.a_desired_trajectory.tolist()
    longitudinalPlan.jerks = self.j_desired_trajectory.tolist()

    longitudinalPlan.hasLead = sm['modelV2'].leadsV3[0].prob > frogpilot_toggles.lead_detection_probability
    longitudinalPlan.longitudinalPlanSource = self.mpc.source
    longitudinalPlan.fcw = self.fcw

    longitudinalPlan.leadTrajectoryX0 = self.mpc.lead_xv_0[:, 0].tolist()
    longitudinalPlan.leadTrajectoryV0 = self.mpc.lead_xv_0[:, 1].tolist()
    longitudinalPlan.leadTrajectoryX1 = self.mpc.lead_xv_1[:, 0].tolist()
    longitudinalPlan.leadTrajectoryV1 = self.mpc.lead_xv_1[:, 1].tolist()

    longitudinalPlan.aTarget = float(self.output_a_target)
    longitudinalPlan.shouldStop = bool(self.output_should_stop)
    longitudinalPlan.allowBrake = True
    longitudinalPlan.allowThrottle = bool(self.allow_throttle)
    longitudinalPlan.distanceToStopTarget = float(self.distance_to_stop_target_m)

    pm.send('longitudinalPlan', plan_send)
