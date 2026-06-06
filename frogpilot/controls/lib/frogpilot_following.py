#!/usr/bin/env python3
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import LEAD_DANGER_FACTOR, desired_follow_distance, get_jerk_factor, get_T_FOLLOW
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.stop_target_helpers import LEAD_STOP_DISTANCE_TARGET

from openpilot.frogpilot.common.frogpilot_variables import MAX_T_FOLLOW

class FrogPilotFollowing:
  def __init__(self, FrogPilotPlanner):
    self.frogpilot_planner = FrogPilotPlanner

    self.following_lead = False
    self.slower_lead = False

    self.acceleration_jerk = 0
    self.danger_jerk = 0
    self.desired_follow_distance = 0
    self.speed_jerk = 0
    self.t_follow = 0

  def update(self, long_control_active, v_ego, sm, frogpilot_toggles):
    if self.frogpilot_planner.frogpilot_traffic.active:
      self.base_acceleration_jerk = self.frogpilot_planner.frogpilot_traffic.acceleration_jerk
      self.base_danger_jerk = self.frogpilot_planner.frogpilot_traffic.danger_jerk
      self.base_speed_jerk = self.frogpilot_planner.frogpilot_traffic.speed_jerk
      self.t_follow = self.frogpilot_planner.frogpilot_traffic.t_follow
    elif long_control_active:
      if sm["carState"].aEgo >= 0:
        self.base_acceleration_jerk, self.base_danger_jerk, self.base_speed_jerk = get_jerk_factor(
          frogpilot_toggles.aggressive_jerk_acceleration, frogpilot_toggles.aggressive_jerk_danger, frogpilot_toggles.aggressive_jerk_speed,
          frogpilot_toggles.standard_jerk_acceleration, frogpilot_toggles.standard_jerk_danger, frogpilot_toggles.standard_jerk_speed,
          frogpilot_toggles.relaxed_jerk_acceleration, frogpilot_toggles.relaxed_jerk_danger, frogpilot_toggles.relaxed_jerk_speed,
          frogpilot_toggles.custom_personalities, sm["selfdriveState"].personality
        )
      else:
        self.base_acceleration_jerk, self.base_danger_jerk, self.base_speed_jerk = get_jerk_factor(
          frogpilot_toggles.aggressive_jerk_deceleration, frogpilot_toggles.aggressive_jerk_danger, frogpilot_toggles.aggressive_jerk_speed_decrease,
          frogpilot_toggles.standard_jerk_deceleration, frogpilot_toggles.standard_jerk_danger, frogpilot_toggles.standard_jerk_speed_decrease,
          frogpilot_toggles.relaxed_jerk_deceleration, frogpilot_toggles.relaxed_jerk_danger, frogpilot_toggles.relaxed_jerk_speed_decrease,
          frogpilot_toggles.custom_personalities, sm["selfdriveState"].personality
        )

      self.t_follow = get_T_FOLLOW(
        frogpilot_toggles.aggressive_follow,
        frogpilot_toggles.standard_follow,
        frogpilot_toggles.relaxed_follow,
        frogpilot_toggles.custom_personalities,
        sm["selfdriveState"].personality,
        v_ego,
        sm["selfdriveState"].experimentalMode,
        self.frogpilot_planner.not_leftmost_lane,
      )
    else:
      self.base_acceleration_jerk = 0
      self.base_danger_jerk = 0
      self.base_speed_jerk = 0
      self.t_follow = 0

    self.acceleration_jerk = self.base_acceleration_jerk
    if self.frogpilot_planner.frogpilot_traffic.active:
      self.danger_factor = self.frogpilot_planner.frogpilot_traffic.danger_factor
    else:
      self.danger_factor = LEAD_DANGER_FACTOR
    self.danger_jerk = self.base_danger_jerk
    self.speed_jerk = self.base_speed_jerk
    self.slower_lead = False

    self.following_lead = self.frogpilot_planner.tracking_lead and self.frogpilot_planner.lead_one.dRel < (self.t_follow * 2) * v_ego

    if self.frogpilot_planner.frogpilot_weather.weather_id != 0:
      self.t_follow = min(self.t_follow + self.frogpilot_planner.frogpilot_weather.increase_following_distance, MAX_T_FOLLOW)

    if long_control_active and self.frogpilot_planner.tracking_lead:
      desired_distance = desired_follow_distance(
        v_ego,
        self.frogpilot_planner.lead_one.vLead,
        self.frogpilot_planner.lead_one.dRel,
        t_follow=self.t_follow,
        lead_stop_distance_target=LEAD_STOP_DISTANCE_TARGET,
      )
      long_distance_factor = frogpilot_toggles.long_distance_factor if self.frogpilot_planner.not_leftmost_lane else 1.0
      distance_factor = frogpilot_toggles.short_distance_factor if self.frogpilot_planner.lead_one.dRel < desired_distance else long_distance_factor
      self.desired_follow_distance = int(desired_distance * distance_factor)
    else:
      self.desired_follow_distance = 0
