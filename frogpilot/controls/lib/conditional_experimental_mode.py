#!/usr/bin/env python3
import math

from cereal import log
from openpilot.common.constants import CV
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.realtime import DT_MDL

from openpilot.frogpilot.common.frogpilot_variables import CRUISING_SPEED, THRESHOLD

CEStatus = {
  "OFF": 0,              # Off
  "USER_DISABLED": 1,    # "Experimental Mode" disabled by user
  "USER_OVERRIDDEN": 2,  # "Experimental Mode" enabled by user
  "CURVATURE": 3,        # Road curvature condition
  "LEAD": 4,             # Slower lead vehicle condition
  "SIGNAL": 5,           # Turn signal condition
  "SPEED": 6,            # Speed condition
  "SPEED_LIMIT": 7,      # Speed limit controller condition
  "STOP_LIGHT": 8,       # Stop light or sign condition
  "FORCE_COAST": 9       # Force Coast hold condition
}

THRESHOLD_0_25 = max(int(round(0.25 / DT_MDL)), 1)
LaneChangeState = log.LaneChangeState

class ConditionalExperimentalMode:
  def __init__(self, FrogPilotPlanner):
    self.frogpilot_planner = FrogPilotPlanner

    self.curvature_filter = FirstOrderFilter(0, 0.5, DT_MDL)
    self.slow_lead_filter = FirstOrderFilter(0, 1, DT_MDL)
    self.stop_light_filter = FirstOrderFilter(0, 0.5, DT_MDL)

    self.curve_detected = False
    self.experimental_mode = False
    self.slow_lead_detected = False
    self.lead_braking_detected = False
    self.lead_braking_active_count = 0
    self.stop_light_detected = False

  def update(self, v_ego, sm, v_lead, dRel_lead, aLeadK, frogpilot_toggles):
    v_ego_kph = v_ego * CV.MS_TO_KPH
    if frogpilot_toggles.experimental_mode_via_press:
      self.status_value = self.frogpilot_planner.params_memory.get("CEStatus")
    else:
      self.status_value = CEStatus["OFF"]

    if self.status_value not in (CEStatus["USER_DISABLED"], CEStatus["USER_OVERRIDDEN"]) and not sm["carState"].standstill:
      self.update_conditions(v_ego, sm, v_lead, dRel_lead, aLeadK, frogpilot_toggles)
      self.experimental_mode = self.check_conditions(v_ego, sm, v_ego_kph, frogpilot_toggles)
      self.frogpilot_planner.params_memory.put("CEStatus", self.status_value if self.experimental_mode else CEStatus["OFF"])
    else:
      self.experimental_mode &= sm["carState"].standstill and self.frogpilot_planner.model_stopped
      self.experimental_mode &= self.status_value != CEStatus["USER_DISABLED"]
      self.experimental_mode |= self.status_value == CEStatus["USER_OVERRIDDEN"]

      self.stop_light_detected &= self.status_value not in (CEStatus["USER_DISABLED"], CEStatus["USER_OVERRIDDEN"])
      self.stop_light_filter.x = 0

  def check_conditions(self, v_ego, sm, v_ego_kph, frogpilot_toggles):
    csc_curve_active = frogpilot_toggles.csc_curves and self.frogpilot_planner.frogpilot_vcruise.csc_controlling_speed
    csc_curve_active &= math.isfinite(self.frogpilot_planner.frogpilot_vcruise.csc_target)
    csc_curve_active &= self.frogpilot_planner.frogpilot_vcruise.csc_target < v_ego

    if (self.curve_detected and (not self.frogpilot_planner.frogpilot_following.following_lead or frogpilot_toggles.conditional_curves_lead) and frogpilot_toggles.conditional_curves) or csc_curve_active:
      self.status_value = CEStatus["CURVATURE"]
      return True

    if frogpilot_toggles.conditional_lead and (self.slow_lead_detected or (self.lead_braking_detected and v_ego_kph < 80.0)):
      self.status_value = CEStatus["LEAD"]
      return True

    if (sm["carState"].leftBlinker or sm["carState"].rightBlinker) and v_ego < frogpilot_toggles.conditional_signal:
      desired_lane = self.frogpilot_planner.lane_width_left if sm["carState"].leftBlinker else self.frogpilot_planner.lane_width_right
      if desired_lane < frogpilot_toggles.lane_detection_width or not frogpilot_toggles.conditional_signal_lane_detection:
        self.status_value = CEStatus["SIGNAL"]
        return True

    below_speed = 1 <= v_ego < (frogpilot_toggles.conditional_limit_lead if self.frogpilot_planner.frogpilot_following.following_lead else frogpilot_toggles.conditional_limit)
    below_urban_limit = self.frogpilot_planner.frogpilot_vcruise.slc_target != 0 and self.frogpilot_planner.frogpilot_vcruise.slc_target < 40.0 * CV.KPH_TO_MS
    if below_speed or below_urban_limit:
      self.status_value = CEStatus["SPEED"]
      return True

    if self.frogpilot_planner.frogpilot_vcruise.slc.experimental_mode:
      self.status_value = CEStatus["SPEED_LIMIT"]
      return True

    slc_active = self.frogpilot_planner.frogpilot_vcruise.slc_target != 0
    slc_override_target = max(self.frogpilot_planner.frogpilot_vcruise.slc.overridden_speed,
                              self.frogpilot_planner.frogpilot_vcruise.slc_target + self.frogpilot_planner.frogpilot_vcruise.slc_offset)
    aggressive_personality = sm["selfdriveState"].personality == log.LongitudinalPersonality.aggressive
    if slc_active and v_ego_kph < 50.0 and slc_override_target < v_ego and not aggressive_personality:
      self.status_value = CEStatus["SPEED"]
      return True

    force_coast_active = sm["frogpilotCarState"].forceCoast and any((
      frogpilot_toggles.force_coast_via_distance,
      frogpilot_toggles.force_coast_via_distance_long,
      frogpilot_toggles.force_coast_via_distance_very_long,
      frogpilot_toggles.force_coast_via_lkas,
    ))
    if force_coast_active:
      self.status_value = CEStatus["FORCE_COAST"]
      return True

    if self.stop_light_detected and frogpilot_toggles.conditional_model_stop_time != 0:
      self.status_value = CEStatus["STOP_LIGHT"]
      return True

    return False

  def update_conditions(self, v_ego, sm, v_lead, dRel_lead, aLeadK, frogpilot_toggles):
    not_changing_lanes = sm["modelV2"].meta.laneChangeState == LaneChangeState.off

    self.curve_detection(v_ego, frogpilot_toggles, not_changing_lanes)
    self.slow_lead(v_ego, v_lead, frogpilot_toggles, not_changing_lanes)
    self.stop_sign_and_light(v_ego, sm, frogpilot_toggles.conditional_model_stop_time)
    self.lead_braking(v_lead, dRel_lead, aLeadK, v_ego, not_changing_lanes, sm["selfdriveState"].personality)

  def curve_detection(self, v_ego, frogpilot_toggles, not_changing_lanes):
    self.curvature_filter.update(not_changing_lanes and (self.frogpilot_planner.driving_in_curve or self.frogpilot_planner.road_curvature_detected))
    self.curve_detected = self.curvature_filter.x >= THRESHOLD and v_ego > CRUISING_SPEED

  def slow_lead(self, v_ego, v_lead, frogpilot_toggles, not_changing_lanes):
    if self.frogpilot_planner.tracking_lead and not_changing_lanes:
      v_lead_kph = v_lead * CV.MS_TO_KPH
      slower_lead = frogpilot_toggles.conditional_slower_lead and (
        getattr(self.frogpilot_planner.frogpilot_following, "slower_lead", False) or (v_ego - v_lead) > CRUISING_SPEED
      )
      stopped_lead = self.frogpilot_planner.lead_one.vLead < 1 and frogpilot_toggles.conditional_stopped_lead

      self.slow_lead_filter.update((slower_lead and v_lead_kph < 80.0) or stopped_lead)
      self.slow_lead_detected = self.slow_lead_filter.x >= THRESHOLD
    else:
      self.slow_lead_filter.x = 0
      self.slow_lead_detected = False

  def stop_sign_and_light(self, v_ego, sm, model_time):
    self.stop_light_filter.update((self.frogpilot_planner.model_length < v_ego * model_time) or self.frogpilot_planner.model_stopped)
    self.stop_light_detected = self.stop_light_filter.x >= THRESHOLD and not self.frogpilot_planner.tracking_lead

  def lead_braking(self, v_lead, dRel_lead, aLeadK, v_ego, not_changing_lanes, personality):
    if not self.frogpilot_planner.tracking_lead or not not_changing_lanes:
      self.lead_braking_active_count = 0
      self.lead_braking_detected = False
      return

    dist_in_s = dRel_lead / v_ego if v_ego > 0 else 0.0
    aggressive_personality = personality == log.LongitudinalPersonality.aggressive
    lead_braking = (
      dist_in_s > 1.2
      and dist_in_s < 4.0
      and not aggressive_personality
      and aLeadK <= -0.5
      and v_lead < v_ego
    )

    if lead_braking or (self.lead_braking_detected and aLeadK <= -0.1 and v_lead < v_ego):
      self.lead_braking_active_count += 1
    else:
      self.lead_braking_active_count = max(0, self.lead_braking_active_count - 1)

    self.lead_braking_detected = self.lead_braking_active_count >= THRESHOLD_0_25
