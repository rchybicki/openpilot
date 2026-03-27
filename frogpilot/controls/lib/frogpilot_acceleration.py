#!/usr/bin/env python3
import numpy as np

from openpilot.common.constants import CV
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.controls.lib.longitudinal_planner import ACCEL_MIN, get_max_accel

from openpilot.frogpilot.common.frogpilot_variables import CITY_SPEED_LIMIT, CRUISING_SPEED

A_CRUISE_MIN_ECO =   ACCEL_MIN / 2
A_CRUISE_MIN_SPORT = ACCEL_MIN * 2
CSC_FULL_BRAKING_FORCE_SPEED = 50 * CV.KPH_TO_MS
CSC_REDUCTION_END_SPEED = 100 * CV.KPH_TO_MS
FORCE_COAST_HIGH_SPEED_MIN_ACCEL = -1.2
FORCE_COAST_NEAR_STOP_MIN_ACCEL = -0.7
FORCE_COAST_RAMP_IN_S = 0.6

                  # MPH = [0.0,  11,  22,  34,  45,  56,  89]
A_CRUISE_MAX_BP_CUSTOM =  [0.0,  5., 10., 15., 20., 25., 40.]
A_CRUISE_MAX_VALS_ECO =   [2.5, 2.5, 1.8, 1.2, 1.0, 0.7, 0.6]
A_CRUISE_MAX_VALS_SPORT = [3.0, 2.5, 2.0, 1.5, 1.0, 0.8, 0.6]

ACCELERATION_PROFILES = {
  "STANDARD": 0,
  "ECO": 1,
  "SPORT": 2,
  "SPORT_PLUS": 3
}

DECELERATION_PROFILES = {
  "STANDARD": 0,
  "ECO": 1,
  "SPORT": 2
}

def get_max_accel_eco(v_ego):
  return np.interp(v_ego, A_CRUISE_MAX_BP_CUSTOM, A_CRUISE_MAX_VALS_ECO)

def get_max_accel_sport(v_ego):
  return np.interp(v_ego, A_CRUISE_MAX_BP_CUSTOM, A_CRUISE_MAX_VALS_SPORT)

def get_max_accel_low_speeds(max_accel, v_cruise):
  return np.interp(v_cruise, [0., CITY_SPEED_LIMIT / 2, CITY_SPEED_LIMIT], [max_accel / 4, max_accel / 2, max_accel])

def get_max_accel_ramp_off(max_accel, v_cruise, v_ego):
  return np.interp(v_cruise - v_ego, [0., 1., 5.], [0., 0.5, max_accel])

def get_max_allowed_accel(v_ego):
  return np.interp(v_ego, [0., 5., 20.], [4.0, 4.0, 2.0])  # ISO 15622:2018

def get_csc_braking_force_limit(v_ego, max_force, high_speed_reduction):
  reduction = float(np.clip(high_speed_reduction, 0.0, 1.0))
  force_scale = float(np.interp(v_ego,
                                [0.0, CSC_FULL_BRAKING_FORCE_SPEED, CSC_REDUCTION_END_SPEED],
                                [1.0, 1.0, 1.0 - reduction]))
  return max_force * force_scale

class FrogPilotAcceleration:
  def __init__(self, FrogPilotPlanner):
    self.frogpilot_planner = FrogPilotPlanner

    self.force_coast_blend = 0.0
    self.max_accel = 0
    self.min_accel = 0

  def get_normal_min_accel(self, eco_gear, sport_gear, frogpilot_toggles):
    if (eco_gear or sport_gear) and frogpilot_toggles.map_deceleration:
      return A_CRUISE_MIN_ECO if eco_gear else A_CRUISE_MIN_SPORT

    if frogpilot_toggles.deceleration_profile == DECELERATION_PROFILES["ECO"]:
      return A_CRUISE_MIN_ECO
    if frogpilot_toggles.deceleration_profile == DECELERATION_PROFILES["SPORT"]:
      return A_CRUISE_MIN_SPORT
    return ACCEL_MIN

  def update(self, v_ego, sm, frogpilot_toggles):
    eco_gear = sm["frogpilotCarState"].ecoGear
    sport_gear = sm["frogpilotCarState"].sportGear

    if sm["frogpilotCarState"].trafficModeEnabled:
      self.max_accel = get_max_accel(v_ego)
    elif (eco_gear or sport_gear) and frogpilot_toggles.map_acceleration:
      if eco_gear:
        self.max_accel = get_max_accel_eco(v_ego)
      else:
        if frogpilot_toggles.acceleration_profile == ACCELERATION_PROFILES["SPORT"]:
          self.max_accel = get_max_accel_sport(v_ego)
        else:
          self.max_accel = get_max_allowed_accel(v_ego)
    else:
      if frogpilot_toggles.acceleration_profile == ACCELERATION_PROFILES["ECO"]:
        self.max_accel = get_max_accel_eco(v_ego)
      elif frogpilot_toggles.acceleration_profile == ACCELERATION_PROFILES["SPORT"]:
        self.max_accel = get_max_accel_sport(v_ego)
      elif frogpilot_toggles.acceleration_profile == ACCELERATION_PROFILES["SPORT_PLUS"]:
        self.max_accel = get_max_allowed_accel(v_ego)
      else:
        self.max_accel = get_max_accel(v_ego)

    if frogpilot_toggles.human_acceleration:
      base_max_accel = self.max_accel
      self.max_accel = get_max_accel_low_speeds(self.max_accel, self.frogpilot_planner.v_cruise)
      self.max_accel = min(get_max_accel_ramp_off(self.max_accel, self.frogpilot_planner.v_cruise, v_ego), self.max_accel)

      lead = self.frogpilot_planner.lead_one
      lead_departing = lead.status and lead.vLead > max(v_ego + 0.2, 0.4) and lead.dRel > 2.5
      speed_scale = float(np.interp(v_ego, [4.2, 8.3], [0.0, 1.0]))
      if lead_departing and speed_scale > 0.0:
        speed_factor = float(np.interp(lead.vLead, [0.4, 1.5, 4.0], [0.35, 0.55, 0.8]))
        gap_factor = float(np.interp(lead.dRel, [2.5, 5.5, 12.0], [0.0, 0.6, 1.0]))
        launch_floor = base_max_accel * speed_factor * gap_factor * speed_scale
        self.max_accel = max(self.max_accel, min(launch_floor, base_max_accel))

    if self.frogpilot_planner.frogpilot_weather.weather_id != 0:
      self.max_accel -= self.max_accel * self.frogpilot_planner.frogpilot_weather.reduce_acceleration

    normal_min_accel = self.get_normal_min_accel(eco_gear, sport_gear, frogpilot_toggles)

    if self.frogpilot_planner.tracking_lead:
      self.force_coast_blend = 0.0
      self.min_accel = ACCEL_MIN
    elif sm["frogpilotCarState"].forceCoast:
      stop_gate = max(float(frogpilot_toggles.vEgoStopping), 0.2)
      force_coast_min_accel = float(np.interp(v_ego,
                                              [stop_gate, stop_gate + 0.8, stop_gate + 2.2],
                                              [FORCE_COAST_NEAR_STOP_MIN_ACCEL, -1.0, FORCE_COAST_HIGH_SPEED_MIN_ACCEL]))
      ramp_in_s = float(np.interp(v_ego, [stop_gate, stop_gate + 0.8, stop_gate + 2.2], [0.9, FORCE_COAST_RAMP_IN_S, 0.35]))
      self.force_coast_blend = min(self.force_coast_blend + (DT_MDL / max(ramp_in_s, DT_MDL)), 1.0)
      self.min_accel = float(((1.0 - self.force_coast_blend) * normal_min_accel) + (self.force_coast_blend * force_coast_min_accel))
    else:
      self.force_coast_blend = 0.0
      self.min_accel = normal_min_accel

    csc_active = frogpilot_toggles.curve_speed_controller and self.frogpilot_planner.road_curvature_detected and v_ego > CRUISING_SPEED
    if csc_active and self.min_accel < 0 and frogpilot_toggles.csc_braking_force > 0:
      desired_force = get_csc_braking_force_limit(v_ego, frogpilot_toggles.csc_braking_force,
                                                  frogpilot_toggles.csc_braking_force_high_speed_reduction)
      if desired_force > 0:
        desired_min = -desired_force
        self.min_accel = max(min(self.min_accel, desired_min), ACCEL_MIN)
