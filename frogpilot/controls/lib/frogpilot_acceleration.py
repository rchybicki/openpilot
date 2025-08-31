#!/usr/bin/env python3
import numpy as np

from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import CRUISE_MIN_ACCEL
from openpilot.selfdrive.controls.lib.longitudinal_planner import ACCEL_MIN, get_max_accel

from openpilot.frogpilot.common.frogpilot_variables import CITY_SPEED_LIMIT, CRUISING_SPEED

A_CRUISE_MIN_ECO = -0.5
A_CRUISE_MIN_SPORT = CRUISE_MIN_ACCEL * 2

                       # MPH = [ 0.,  11,  22,  34,  45,  56,  89]
                       # KPH = [ 0.,  18,  36,  56,  72,  90, 144]
A_CRUISE_MAX_BP_CUSTOM =       [ 0.,  5., 10., 15., 20., 25., 40.]
A_CRUISE_MAX_VALS_ECO =        [2.5, 2.5, 1.8, 1.2, 1.0, 0.7, 0.6]
A_CRUISE_MAX_VALS_SPORT =      [3.0, 2.5, 2.0, 1.5, 1.0, 0.8, 0.6]

def get_max_accel_eco(v_ego):
  return float(np.interp(v_ego, A_CRUISE_MAX_BP_CUSTOM, A_CRUISE_MAX_VALS_ECO))

def get_max_accel_sport(v_ego):
  return float(np.interp(v_ego, A_CRUISE_MAX_BP_CUSTOM, A_CRUISE_MAX_VALS_SPORT))

def get_max_accel_low_speeds(max_accel, v_cruise):
  return float(np.interp(v_cruise, [0., CITY_SPEED_LIMIT / 2, CITY_SPEED_LIMIT], [max_accel / 4, max_accel / 2, max_accel]))

def get_max_accel_ramp_off(max_accel, v_cruise, v_ego):
  return float(np.interp(v_cruise - v_ego, [0., 1., 5.], [0., 0.5, max_accel]))

def get_max_allowed_accel(v_ego):
  return float(np.interp(v_ego, [0., 5., 20.], [4.0, 4.0, 2.0]))  # ISO 15622:2018

class FrogPilotAcceleration:
  def __init__(self, FrogPilotPlanner):
    self.frogpilot_planner = FrogPilotPlanner

    self.max_accel = 0
    self.min_accel = 0

  def update(self, v_ego, sm, frogpilot_toggles):
    eco_gear = sm["frogpilotCarState"].ecoGear
    sport_gear = sm["frogpilotCarState"].sportGear

    if sm["frogpilotCarState"].trafficModeEnabled:
      self.max_accel = get_max_accel(v_ego)
    elif frogpilot_toggles.map_acceleration and (eco_gear or sport_gear):
      if eco_gear:
        self.max_accel = get_max_accel_eco(v_ego)
      else:
        if frogpilot_toggles.acceleration_profile == 2:
          self.max_accel = get_max_accel_sport(v_ego)
        else:
          self.max_accel = get_max_allowed_accel(v_ego)
    else:
      if frogpilot_toggles.acceleration_profile == 1:
        self.max_accel = get_max_accel_eco(v_ego)
      elif frogpilot_toggles.acceleration_profile == 2:
        self.max_accel = get_max_accel_sport(v_ego)
      elif frogpilot_toggles.acceleration_profile == 3:
        self.max_accel = get_max_allowed_accel(v_ego)
      else:
        self.max_accel = get_max_accel(v_ego)

    if frogpilot_toggles.human_acceleration:
      self.max_accel = min(get_max_accel_low_speeds(self.max_accel, self.frogpilot_planner.v_cruise), self.max_accel)
      self.max_accel = min(get_max_accel_ramp_off(self.max_accel, self.frogpilot_planner.v_cruise, v_ego), self.max_accel)

    if self.frogpilot_planner.frogpilot_weather.weather_id != 0:
      self.max_accel -= self.max_accel * self.frogpilot_planner.frogpilot_weather.reduce_acceleration

    if self.frogpilot_planner.tracking_lead:
      self.min_accel = ACCEL_MIN
    elif sm["frogpilotCarState"].forceCoast:
      self.min_accel = A_CRUISE_MIN_ECO
    elif frogpilot_toggles.map_deceleration and (eco_gear or sport_gear):
      if eco_gear:
        self.min_accel = A_CRUISE_MIN_ECO
      else:
        self.min_accel = A_CRUISE_MIN_SPORT
    else:
      if frogpilot_toggles.deceleration_profile == 1:
        self.min_accel = A_CRUISE_MIN_ECO
      elif frogpilot_toggles.deceleration_profile == 2:
        self.min_accel = A_CRUISE_MIN_SPORT
      else:
        self.min_accel = CRUISE_MIN_ACCEL

    # If CSC is actively controlling speed, allow stronger braking by scaling the min accel
    try:
      csc_active = (
        frogpilot_toggles.curve_speed_controller and
        sm["controlsState"].enabled and
        v_ego > CRUISING_SPEED and
        self.frogpilot_planner.road_curvature_detected
      )
      if csc_active and self.min_accel < 0:
        force = frogpilot_toggles.csc_braking_force if frogpilot_toggles.csc_braking_force else 0.0
        if force > 0.0:
          # Allow up to -force m/s^2 braking during CSC, respecting platform ACCEL_MIN and existing min_accel
          desired_min = -force
          self.min_accel = max(min(self.min_accel, desired_min), ACCEL_MIN)
    except Exception:
      # If any dependency is missing, fall back to computed min_accel
      pass
