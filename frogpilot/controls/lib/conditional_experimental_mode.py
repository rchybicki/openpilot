from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.realtime import DT_MDL

from openpilot.frogpilot.common.frogpilot_variables import CRUISING_SPEED, THRESHOLD, params_memory
from openpilot.common.conversions import Conversions as CV
from cereal import log

THRESHOLD_0_25 = 5  # 0.25s
THRESHOLD_0_5 = THRESHOLD_0_25 * 2  # 0.5s
THRESHOLD_1 = THRESHOLD_0_25 * 4  # 1s
THRESHOLD_1_5 = THRESHOLD_1 + THRESHOLD_0_5  # 1.5s
THRESHOLD_2 = THRESHOLD_1 * 2  # 2s
THRESHOLD_3 = THRESHOLD_1 * 3  # 3s

LaneChangeState = log.LaneChangeState

class ConditionalExperimentalMode:
  def __init__(self, FrogPilotPlanner):
    self.frogpilot_planner = FrogPilotPlanner

    self.curvature_filter = FirstOrderFilter(0, 1, DT_MDL)
    self.slow_lead_filter = FirstOrderFilter(0, 1, DT_MDL)
    self.stop_light_filter = FirstOrderFilter(0, 0.5, DT_MDL)

    self.curve_detected = False
    self.experimental_mode = False
    self.stop_light_detected = False
    self.slow_lead_detected = False
    self.lead_braking_detected = False
    self.lead_braking_active_count = 0

  def update(self, v_ego, sm, v_lead, dRel_lead, aLeadK, frogpilot_toggles):
    if frogpilot_toggles.experimental_mode_via_press:
      self.status_value = params_memory.get_int("CEStatus")
    else:
      self.status_value = 0

    if self.status_value not in {1, 2} and not sm["carState"].standstill:
      v_ego_kph = v_ego * CV.MS_TO_KPH
      self.update_conditions(
        self.frogpilot_planner.tracking_lead, v_ego, sm, v_ego_kph, v_lead, dRel_lead, aLeadK, frogpilot_toggles
      )

      # Normal operation - check conditions and set experimental mode
      self.experimental_mode = self.check_conditions(v_ego, sm, v_ego_kph, v_lead, frogpilot_toggles)
      params_memory.put_int("CEStatus", self.status_value if self.experimental_mode else 0)
    else:
      # Override mode
      self.experimental_mode = self.status_value == 2 or sm["carState"].standstill and self.experimental_mode and self.frogpilot_planner.model_stopped
      self.stop_light_detected &= self.status_value not in {1, 2}
      self.stop_light_filter.x = 0

    # For override disable mode, check if conditions would still require experimental mode
    if self.status_value == 1 and not sm["carState"].standstill:
      # Save current status value
      original_status = self.status_value

      # Temporarily check conditions to see if exp mode would be active
      would_be_exp_mode = self.check_conditions(v_ego, sm, v_ego_kph, v_lead, frogpilot_toggles)

      # Restore the original status value
      self.status_value = original_status

      # Clear override if conditions no longer require experimental mode
      if not would_be_exp_mode:
        params_memory.put_int("CEStatus", 0)

  def check_conditions(self, v_ego, sm, v_ego_kph, v_lead, frogpilot_toggles):
    below_speed = frogpilot_toggles.conditional_limit > v_ego >= 1 and not self.frogpilot_planner.frogpilot_following.following_lead
    below_speed_with_lead = frogpilot_toggles.conditional_limit_lead > v_ego >= 1 and self.frogpilot_planner.frogpilot_following.following_lead
    aggr_pers = sm["controlsState"].personality == log.LongitudinalPersonality.aggressive
    slc_active = self.frogpilot_planner.frogpilot_vcruise.slc_target != 0
    below_40 = slc_active and self.frogpilot_planner.frogpilot_vcruise.slc_target < 40.0 * CV.KPH_TO_MS
    if below_40 or below_speed or below_speed_with_lead:
      self.status_value = 3 if self.frogpilot_planner.frogpilot_following.following_lead else 4
      return True

    desired_lane = self.frogpilot_planner.lane_width_left if sm["carState"].leftBlinker else self.frogpilot_planner.lane_width_right
    lane_available = desired_lane >= frogpilot_toggles.lane_detection_width or not frogpilot_toggles.conditional_signal_lane_detection
    if v_ego < frogpilot_toggles.conditional_signal and (sm["carState"].leftBlinker or sm["carState"].rightBlinker) and not lane_available:
      self.status_value = 5
      return True

    approaching_maneuver = sm["frogpilotNavigation"].approachingIntersection or sm["frogpilotNavigation"].approachingTurn
    if (
      frogpilot_toggles.conditional_navigation
      and approaching_maneuver
      and (frogpilot_toggles.conditional_navigation_lead or not self.frogpilot_planner.frogpilot_following.following_lead)
    ):
      self.status_value = 6 if sm["frogpilotNavigation"].approachingIntersection else 7
      return True

    if frogpilot_toggles.conditional_curves and self.curve_detected and (
      frogpilot_toggles.conditional_curves_lead
      or not self.frogpilot_planner.frogpilot_following.following_lead
    ):
      self.status_value = 8
      return True

    if frogpilot_toggles.csc_curves:
      curve_ctrl_active = (
        self.frogpilot_planner.frogpilot_vcruise.csc_controlling_speed
        and self.frogpilot_planner.frogpilot_vcruise.csc_target < v_ego * 0.9
      )
      if curve_ctrl_active:
        self.status_value = 8
        return True

    if frogpilot_toggles.conditional_lead and (self.slow_lead_detected or (self.lead_braking_detected and v_ego_kph < 80.)):
      self.status_value = 9 if self.frogpilot_planner.lead_one.vLead < 1 else 10
      return True

    if frogpilot_toggles.conditional_model_stop_time != 0 and self.stop_light_detected:
      self.status_value = 11 if not self.frogpilot_planner.frogpilot_vcruise.forcing_stop else 12
      return True

    if self.frogpilot_planner.frogpilot_vcruise.slc.experimental_mode:
      self.status_value = 13
      return True

    if (
      slc_active
      and v_ego_kph < 50.0
      and max(
        self.frogpilot_planner.frogpilot_vcruise.slc.overridden_speed,
        self.frogpilot_planner.frogpilot_vcruise.slc_target + self.frogpilot_planner.frogpilot_vcruise.slc_offset,
      ) < v_ego
      and not aggr_pers
    ):
      self.status_value = 18
      return True

    if frogpilot_toggles.force_coast_via_distance and sm["frogpilotCarState"].forceCoast:
      self.status_value = 19
      return True

    return False

  def update_conditions(self, tracking_lead, v_ego, sm, v_ego_kph, v_lead, dRel_lead, aLeadK, frogpilot_toggles):
    not_changing_lanes = sm["modelV2"].meta.laneChangeState == LaneChangeState.off
    self.curve_detection(tracking_lead, v_ego, v_ego_kph, frogpilot_toggles, not_changing_lanes)
    self.slow_lead(tracking_lead, v_lead, frogpilot_toggles, not_changing_lanes)
    self.stop_sign_and_light(v_ego, sm, frogpilot_toggles.conditional_model_stop_time)
    self.lead_braking(tracking_lead, v_lead, dRel_lead, aLeadK, v_ego, not_changing_lanes, sm["controlsState"].personality)

  def curve_detection(self, tracking_lead, v_ego, v_ego_kph, frogpilot_toggles, not_changing_lanes):
    curve_bp = 1. if v_ego_kph < 120. and not_changing_lanes else 2.
    curve_detected = (curve_bp / abs(self.frogpilot_planner.road_curvature)) ** 0.5 < v_ego
    curve_active = (0.9 / abs(self.frogpilot_planner.road_curvature))**0.5 < v_ego and self.curve_detected

    self.curvature_filter.update(curve_detected or curve_active)
    self.curve_detected = self.curvature_filter.x >= THRESHOLD and v_ego > CRUISING_SPEED

  def slow_lead(self, tracking_lead, v_lead, frogpilot_toggles, not_changing_lanes):
    v_lead_kph = v_lead * CV.MS_TO_KPH
    if tracking_lead and not_changing_lanes:
      slower_lead = frogpilot_toggles.conditional_slower_lead and self.frogpilot_planner.frogpilot_following.slower_lead
      stopped_lead = frogpilot_toggles.conditional_stopped_lead and self.frogpilot_planner.lead_one.vLead < 1

      self.slow_lead_detected = self.slow_lead_filter.update(1 if (slower_lead and v_lead_kph < 80.0) or stopped_lead else 0) >= THRESHOLD
    else:
      self.slow_lead_filter.x = 0
      self.slow_lead_detected = False

  def stop_sign_and_light(self, v_ego, sm, model_time):
    if not sm["frogpilotCarState"].trafficModeEnabled:
      model_stopping = self.frogpilot_planner.model_length < v_ego * model_time

      self.stop_light_filter.update(self.frogpilot_planner.model_stopped or model_stopping)
      self.stop_light_detected = self.stop_light_filter.x >= THRESHOLD and not self.frogpilot_planner.tracking_lead
    else:
      self.stop_light_filter.x = 0
      self.stop_light_detected = False

  def lead_braking(self, tracking_lead, v_lead, dRel_lead, aLeadK, v_ego, not_changing_lanes, personality):
    dist_in_s = dRel_lead / v_ego if v_ego > 0.0 else 0.0
    aggr_pers = personality == log.LongitudinalPersonality.aggressive
    self.lead_braking_detected = self.lead_braking_active_count >= THRESHOLD_0_25
    lead_braking = (
      dist_in_s > 1.2
      and not aggr_pers
      and dist_in_s < 4.0
      and aLeadK <= -0.5
      and v_lead < v_ego
      or self.lead_braking_detected
      and aLeadK <= -0.1
      and v_lead < v_ego
    )
    if tracking_lead and lead_braking and not_changing_lanes:
      self.lead_braking_active_count = min(THRESHOLD_1_5, self.lead_braking_active_count + 1)
    else:
      self.lead_braking_active_count = max(0, self.lead_braking_active_count - 1) if not_changing_lanes else 0
