from openpilot.selfdrive.frogpilot.frogpilot_utilities import MovingAverageCalculator
from openpilot.selfdrive.frogpilot.frogpilot_variables import CITY_SPEED_LIMIT, CRUISING_SPEED, THRESHOLD, params_memory
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

    self.curvature_mac = MovingAverageCalculator()
    self.slow_lead_mac = MovingAverageCalculator()
    self.stop_light_mac = MovingAverageCalculator()

    self.curve_detected = False
    self.experimental_mode = False
    self.stop_light_detected = False
    self.slow_lead_detected = False
    self.lead_braking_detected = False
    self.lead_braking_active_count = 0

  def update(self, carState, frogpilotCarState, frogpilotNavigation, modelData, v_ego, v_lead, dRel_lead, aLeadK, frogpilot_toggles, personality):
    if frogpilot_toggles.experimental_mode_via_press:
      self.status_value = params_memory.get_int("CEStatus")
    else:
      self.status_value = 0

    if self.status_value not in {1, 2, 3, 4, 5, 6} and not carState.standstill:
      v_ego_kph = v_ego * CV.MS_TO_KPH
      self.update_conditions(frogpilotCarState, modelData, self.frogpilot_planner.tracking_lead, v_ego, v_ego_kph, v_lead, dRel_lead, aLeadK, frogpilot_toggles,
                             personality)
      self.experimental_mode = self.check_conditions(carState, frogpilotNavigation, modelData, self.frogpilot_planner.frogpilot_following.following_lead, v_ego,
                                                     v_ego_kph, v_lead, frogpilot_toggles, personality)
      params_memory.put_int("CEStatus", self.status_value if self.experimental_mode else 0)
    else:
      self.experimental_mode = self.status_value in {2, 4, 6} or carState.standstill and self.experimental_mode
      self.stop_light_detected &= self.status_value not in {1, 2, 3, 4, 5, 6}

  def check_conditions(self, carState, frogpilotNavigation, modelData, following_lead, v_ego, v_ego_kph, v_lead, frogpilot_toggles, personality):
    below_speed = frogpilot_toggles.conditional_limit > v_ego >= 1 and not following_lead
    below_speed_with_lead = frogpilot_toggles.conditional_limit_lead > v_ego >= 1 and following_lead
    aggr_pers = personality == log.LongitudinalPersonality.aggressive
    if below_speed or below_speed_with_lead:
      self.status_value = 7 if following_lead else 8
      return True

    desired_lane = self.frogpilot_planner.lane_width_left if carState.leftBlinker else self.frogpilot_planner.lane_width_right
    lane_available = desired_lane >= frogpilot_toggles.lane_detection_width or not frogpilot_toggles.conditional_signal_lane_detection \
             or not frogpilot_toggles.lane_detection
    if v_ego_kph < 80. and (carState.leftBlinker or carState.rightBlinker) and not lane_available:
      self.status_value = 9
      return True

    approaching_maneuver = modelData.navEnabled and (frogpilotNavigation.approachingIntersection or frogpilotNavigation.approachingTurn)
    if frogpilot_toggles.conditional_navigation and approaching_maneuver and (frogpilot_toggles.conditional_navigation_lead or not following_lead):
      self.status_value = 10 if frogpilotNavigation.approachingIntersection else 11
      return True

    if frogpilot_toggles.conditional_curves and self.curve_detected and (frogpilot_toggles.conditional_curves_lead or not following_lead):
      self.status_value = 12
      return True

    if frogpilot_toggles.conditional_lead and (self.slow_lead_detected or self.lead_braking_detected):
      self.status_value = 13 if v_lead < 1 else 14
      return True

    if frogpilot_toggles.conditional_model_stop_time != 0 and self.stop_light_detected and not aggr_pers:
      self.status_value = 15 if not self.frogpilot_planner.frogpilot_vcruise.forcing_stop else 16
      return True

    if self.frogpilot_planner.frogpilot_vcruise.slc.experimental_mode:
      self.status_value = 17
      return True

    slc_active = self.frogpilot_planner.frogpilot_vcruise.slc_target != 0
    if slc_active and v_ego_kph < 50. and \
       max(self.frogpilot_planner.frogpilot_vcruise.overridden_speed, self.frogpilot_planner.frogpilot_vcruise.slc_target) < v_ego \
        and not aggr_pers:
      self.status_value = 18
      return True

    if self.frogpilot_planner.frogpilot_vcruise.mtsc_active:
      self.status_value = 19
      return True

    return False

  def update_conditions(self, frogpilotCarState, modelData, tracking_lead, v_ego, v_ego_kph, v_lead, dRel_lead, aLeadK, frogpilot_toggles, personality):
    not_changing_lanes = modelData.meta.laneChangeState == LaneChangeState.off
    self.curve_detection(tracking_lead, v_ego, v_ego_kph, frogpilot_toggles, not_changing_lanes)
    self.slow_lead(tracking_lead, v_lead, frogpilot_toggles, not_changing_lanes)
    self.stop_sign_and_light(frogpilotCarState, tracking_lead, v_ego, frogpilot_toggles)
    self.lead_braking(tracking_lead, v_lead, dRel_lead, aLeadK, v_ego, not_changing_lanes, personality)

  def curve_detection(self, tracking_lead, v_ego, v_ego_kph, frogpilot_toggles, not_changing_lanes):
    if v_ego > CRUISING_SPEED:
      curve_bp = 1. if v_ego_kph < 120. and not_changing_lanes else 2.
      curve_detected = (curve_bp / self.frogpilot_planner.road_curvature) ** 0.5 < v_ego
      curve_active = (0.9 / self.frogpilot_planner.road_curvature)**0.5 < v_ego and self.curve_detected

      self.curvature_mac.add_data(curve_detected or curve_active)
      self.curve_detected = self.curvature_mac.get_moving_average() >= THRESHOLD
    else:
      self.curvature_mac.reset_data()
      self.curve_detected = False

  def slow_lead(self, tracking_lead, v_lead, frogpilot_toggles, not_changing_lanes):
    v_lead_kph = v_lead * CV.MS_TO_KPH
    if tracking_lead and not_changing_lanes:
      slower_lead = frogpilot_toggles.conditional_slower_lead and self.frogpilot_planner.frogpilot_following.slower_lead
      stopped_lead = frogpilot_toggles.conditional_stopped_lead and v_lead < 1

      self.slow_lead_mac.add_data((slower_lead and v_lead_kph < 80.0) or stopped_lead)
      self.slow_lead_detected = self.slow_lead_mac.get_moving_average() >= THRESHOLD
    else:
      self.slow_lead_mac.reset_data()
      self.slow_lead_detected = False

  def stop_sign_and_light(self, frogpilotCarState, tracking_lead, v_ego, frogpilot_toggles):
    if not (self.curve_detected or tracking_lead or frogpilotCarState.trafficModeActive):
      model_stopping = self.frogpilot_planner.model_length < v_ego * frogpilot_toggles.conditional_model_stop_time

      self.stop_light_mac.add_data(self.frogpilot_planner.model_stopped or model_stopping)
      self.stop_light_detected = self.stop_light_mac.get_moving_average() >= THRESHOLD
    else:
      self.stop_light_mac.reset_data()
      self.stop_light_detected = False

  def lead_braking(self, tracking_lead, v_lead, dRel_lead, aLeadK, v_ego, not_changing_lanes, personality):
    dist_in_s = dRel_lead / v_ego if v_ego > 0.0 else 0.0
    aggr_pers = personality == log.LongitudinalPersonality.aggressive
    self.lead_braking_detected = self.lead_braking_active_count >= THRESHOLD_0_25
    lead_braking = (
      dist_in_s > 1.2
      and not aggr_pers
      and dist_in_s < 4.0
      and aLeadK <= -0.3
      and v_lead < v_ego
      or self.lead_braking_detected
      and aLeadK <= -0.05
      and v_lead < v_ego
    )
    if tracking_lead and lead_braking and not_changing_lanes:
      self.lead_braking_active_count = min(THRESHOLD_1_5, self.lead_braking_active_count + 1)
    else:
      self.lead_braking_active_count = max(0, self.lead_braking_active_count - 1) if not_changing_lanes else 0
