import numpy as np
from openpilot.common.conversions import Conversions as CV
from cereal import log
from openpilot.selfdrive.controls.lib.lateral_planner import TRAJECTORY_SIZE
from openpilot.common.numpy_fast import clip, interp
from openpilot.common.params import Params, put_bool_nonblocking

# Time threshold for Conditional Experimental Mode (Code runs at 20hz, so: THRESHOLD / 20 = seconds)
THRESHOLD = 5 # 0.25s

# Lookup table for stop sign / stop light detection. Credit goes to the DragonPilot team!
STOP_SIGN_BREAKING_POINT = [0., 10., 20., 30., 40., 50., 55.]
STOP_SIGN_DISTANCE = [10, 30., 50., 70., 80., 90., 120.]

LaneChangeState = log.LateralPlan.LaneChangeState

class ExperimentalController():
  turn_speed_controller_exp_mode: bool = False
  
  def __init__(self):
    self.op_enabled = False
    self.gas_pressed = False
    self.last_params_update = 100
    self.v_ego = 0
    self.v_ego_kph = 0
    self.curve = False
    self.lead_braking_active = False
    self.lead_speed_diff_active = False
    self.lead_speed_active = False
    self.curvature_count = 0
    self.enabled_experimental = False
    self.lead_status_count = 0
    self.stop_light_count = 0
    self.mapd_force_count = 0
    self.previous_lead_status = False
    self.previous_lead_speed = 0
    self.engaged = 0
    self.params = Params()
    self.enabled = self.params.get_bool("ExperimentalControl")

  def update_params(self):
    self.last_params_update = self.last_params_update - 1
    if self.last_params_update <= 0:
      self.enabled = self.params.get_bool("ExperimentalControl")
      self.last_params_update = 100

  def road_curvature(self, lead, standstill):
    # Check if there's no lead vehicle if the toggle is on
    if (not lead or self.radarState.leadOne.dRel > 40)  and not standstill:
      predicted_lateral_accelerations = np.abs(np.array(self.modelData.acceleration.y))
      predicted_velocities = np.array(self.modelData.velocity.x)
      if len(predicted_lateral_accelerations) == len(predicted_velocities) != 0:
        curvature_ratios = predicted_lateral_accelerations / (predicted_velocities ** 2)
        predicted_lateral_accelerations = curvature_ratios * (self.v_ego ** 2)
        curvature = np.amax(predicted_lateral_accelerations)
        curvature_bp = 1.4 if self.v_ego_kph < 120. \
                      and self.lat_planner_data.laneChangeState == LaneChangeState.off else 1.8
        if curvature >= curvature_bp or self.turn_speed_controller_exp_mode or (self.curve and curvature > 1.1):
          # Setting the maximum to 10 lets it hold the status for 0.25s after it goes "false" to help prevent false negatives
          self.curvature_count = min(30, self.curvature_count + 1)
        else:
          self.curvature_count = max(0, self.curvature_count - 1)
        # Check if curve is detected for > 0.25s
        self.curve = self.curvature_count >= THRESHOLD
      else:
        self.curve = False
    else:
      self.curve = False
    return self.curve
      

  # Stop sign and stop light detection - Credit goes to the DragonPilot team!
  def stop_sign_and_light(self, lead, standstill):
    lead_speed = self.radarState.leadOne.vLead
    if abs(self.carState.steeringAngleDeg) <= 60 and (not standstill or self.stop_light_count >= THRESHOLD):
      # Check to make sure we don't have a lead that's stopping for the red light / stop sign
      if lead and not (self.previous_lead_speed >= lead_speed or self.radarState.leadOne.dRel <= 10 or lead_speed <= 1) or not lead:
        if len(self.modelData.orientation.x) == len(self.modelData.position.x) == TRAJECTORY_SIZE:
          if self.modelData.position.x[-1] < interp(self.v_ego_kph, STOP_SIGN_BREAKING_POINT, STOP_SIGN_DISTANCE):
            self.stop_light_count = min(10, self.stop_light_count + 1)
          else:
            self.stop_light_count = max(0, self.stop_light_count - 1)
        else:
          self.stop_light_count = max(0, self.stop_light_count - 1)
      else:
        self.stop_light_count = max(0, self.stop_light_count - 1)
    else:
      self.stop_light_count = max(0, self.stop_light_count - 1)
    self.previous_lead_speed = lead_speed
    # Check if stop sign / stop light is detected for > 0.25s
    return self.stop_light_count >= THRESHOLD
  
  def detect_lead(self):
    lead_status = self.radarState.leadOne.status
    self.lead_status_count = (self.lead_status_count + 1) if lead_status == self.previous_lead_status else 0
    self.previous_lead_status = lead_status
    # Check if lead is detected for > 0.25s
    return self.lead_status_count >= THRESHOLD and lead_status
  
  def mapd_force_exp(self):
    if self.params.get_bool("ExperimentalControl-MapdForce"):
      # Setting the maximum to 10 lets it hold the status for 0.25s after it goes "false" to help prevent false negatives
      self.mapd_force_count = min(10, self.mapd_force_count + 1)
    else:
      self.mapd_force_count = max(0, self.mapd_force_count - 1)
    # Check if active for > 0.25s
    return self.mapd_force_count >= THRESHOLD
  
  def lead_braking(self, lead):
    self.lead_braking_active = lead and (self.radarState.leadOne.aLeadK <= -0.75 or self.lead_braking_active and self.radarState.leadOne.aLeadK <= -0.25)
    return self.lead_braking_active
  
  def lead_speed_diff(self, lead, personality):
    self.lead_speed_diff_active = lead and personality!=log.LongitudinalPersonality.aggressive and self.v_ego_kph < 80. \
                                   and self.radarState.leadOne.vLead > 0.0 \
                                   and (self.v_ego / self.radarState.leadOne.vLead > 1.3 or self.lead_speed_diff_active and self.v_ego / self.radarState.leadOne.vLead > 1.1)
    return self.lead_speed_diff_active
  
  def lead_speed(self, lead, personality):
    self.lead_speed_active = lead and personality!=log.LongitudinalPersonality.aggressive \
                              and (self.radarState.leadOne.vLead < 8.0 or self.lead_speed_active and self.radarState.leadOne.vLead < 9.0)
    return self.lead_speed_active

  def update_calculations(self, slc_speed_limit, personality, vtsc_active):
    lead = self.detect_lead()
    standstill = self.carState.standstill
    signal = self.v_ego_kph < 50. and (self.carState.leftBlinker or self.carState.rightBlinker)
    curve = self.road_curvature(lead, standstill)
    stop_light_detected = self.stop_sign_and_light(lead, standstill)

    self.engaged = min(1000, self.engaged + 1) if self.op_enabled else 0
    engaged_active = self.v_ego_kph < 70. and self.engaged < 25

    exp_mode_speed_limit = 0.
    if personality==log.LongitudinalPersonality.relaxed:
        exp_mode_speed_limit = 35.
    elif personality==log.LongitudinalPersonality.standard:
      exp_mode_speed_limit = 27.
    elif personality==log.LongitudinalPersonality.aggressive:
      exp_mode_speed_limit = 5.
    else: #snow
      exp_mode_speed_limit = 45.
    
    speed = self.v_ego_kph <= exp_mode_speed_limit 

    lead_speed = self.lead_speed(lead, personality)
    lead_speed_diff = self.lead_speed_diff(lead, personality)


    exp_mode_lead_distance = 0.
    if personality==log.LongitudinalPersonality.relaxed:
        exp_mode_lead_distance = 20. #32.5
    elif personality==log.LongitudinalPersonality.standard:
      exp_mode_lead_distance = 15. #27.5
    elif personality==log.LongitudinalPersonality.aggressive:
      exp_mode_lead_distance = 10. #15.
    else: #snow
      exp_mode_lead_distance = 25. #40.
    lead_distance = lead and self.radarState.leadOne.dRel < exp_mode_lead_distance

    lead_braking = self.lead_braking(lead)
    mapd_force_exp_mode = self.mapd_force_exp()
    navd_upcoming_turn = self.params.get_bool("ExperimentalControl-NavdTurn")
    mapd_disable_exp_mode = self.params.get_bool("ExperimentalControl-MapdDisable")
    self.active = ((curve and not vtsc_active) or stop_light_detected or standstill or signal or speed or lead_speed or lead_speed_diff \
                    or lead_distance or lead_braking or slc_speed_limit == 0 or mapd_force_exp_mode \
                    or (navd_upcoming_turn and not mapd_disable_exp_mode) or engaged_active) \
                    and self.op_enabled
                    # and not self.gas_pressed 


  def update_experimental_mode(self):
    if not self.enabled:
      return
    experimental_mode = self.params.get_bool("ExperimentalMode")
    if self.active and not experimental_mode and not self.enabled_experimental:
      self.enabled_experimental = True
      put_bool_nonblocking("ExperimentalMode", True)
    elif not self.active and experimental_mode and self.enabled_experimental:
        put_bool_nonblocking("ExperimentalMode", False)
    elif not self.active and not experimental_mode and self.enabled_experimental:
      self.enabled_experimental = False


  def update(self, op_enabled, v_ego, sm, slc_speed_limit, vtsc_active, personality=log.LongitudinalPersonality.standard):
    self.op_enabled = op_enabled
    self.carState, self.modelData, self.radarState, self.lat_planner_data = (sm[key] for key in ['carState', 'modelV2', 'radarState', 'lateralPlan'])
    self.gas_pressed = self.carState.gasPressed
    self.v_ego = v_ego
    self.v_ego_kph = v_ego * 3.6

    self.update_params()
    self.update_calculations(slc_speed_limit, personality, vtsc_active)
    self.update_experimental_mode()
