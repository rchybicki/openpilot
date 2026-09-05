#!/usr/bin/env python3
import math
from numbers import Number

from cereal import car, custom, log
import cereal.messaging as messaging
from openpilot.common.constants import CV
from openpilot.common.params import Params
from openpilot.common.realtime import config_realtime_process, DT_CTRL, Priority, Ratekeeper
from openpilot.common.swaglog import cloudlog

from opendbc.car.car_helpers import interfaces
from opendbc.car.vehicle_model import VehicleModel
from opendbc.safety import ALTERNATIVE_EXPERIENCE
from openpilot.selfdrive.car.live_update_handoff import LIVE_UPDATE_HANDOFF_PARAM, PANDA_HANDOFF_STATES, state_name
from openpilot.selfdrive.controls.lib.drive_helpers import clip_curvature, longitudinal_accel_with_gas, longitudinal_control_active, longitudinal_control_override
from openpilot.selfdrive.controls.lib.latcontrol import LatControl
from openpilot.selfdrive.controls.lib.latcontrol_pid import LatControlPID
from openpilot.selfdrive.controls.lib.latcontrol_angle import LatControlAngle, STEER_ANGLE_SATURATION_THRESHOLD
from openpilot.selfdrive.controls.lib.latcontrol_curvature import LatControlCurvature
from openpilot.selfdrive.controls.lib.latcontrol_torque import LatControlTorque
from openpilot.selfdrive.controls.lib.longcontrol import LongControl, LongCtrlState
from openpilot.selfdrive.controls.lib import stopping_flags
from openpilot.selfdrive.controls.lib.identification_hook import HookInputs
from openpilot.selfdrive.modeld.modeld import LAT_SMOOTH_SECONDS
from openpilot.selfdrive.locationd.helpers import PoseCalibrator, Pose

from openpilot.frogpilot.common.frogpilot_variables import get_frogpilot_toggles
from openpilot.frogpilot.controls.lib.neural_network_feedforward import LatControlNNFF

State = log.SelfdriveState.OpenpilotState
LaneChangeState = log.LaneChangeState
LaneChangeDirection = log.LaneChangeDirection

ACTUATOR_FIELDS = tuple(car.CarControl.Actuators.schema.fields.keys())


class Controls:
  def __init__(self) -> None:
    self.params = Params()
    self.live_update_handoff_state = self.params.get(LIVE_UPDATE_HANDOFF_PARAM) or ""
    cloudlog.info("controlsd is waiting for CarParams")
    self.CP = messaging.log_from_bytes(self.params.get("CarParams", block=True), car.CarParams)
    self.FPCP = messaging.log_from_bytes(self.params.get("FrogPilotCarParams", block=True), custom.FrogPilotCarParams)
    cloudlog.info("controlsd got CarParams")

    self.CI = interfaces[self.CP.carFingerprint](self.CP, self.FPCP)

    self.sm = messaging.SubMaster(['liveDelay', 'liveParameters', 'liveTorqueParameters', 'modelV2', 'selfdriveState',
                                   'liveCalibration', 'livePose', 'longitudinalPlan', 'carState', 'carOutput', 'radarState',
                                   'driverMonitoringState', 'onroadEvents', 'driverAssistance'], poll='selfdriveState')
    self.pm = messaging.PubMaster(['carControl', 'controlsState'] + (['alertDebug'] if stopping_flags.IDENTIFICATION_HOOK else []))

    self.steer_limited_by_safety = False
    self.curvature = 0.0
    self.desired_curvature = 0.0

    self.pose_calibrator = PoseCalibrator()
    self.calibrated_pose: Pose | None = None
    self.longitudinal_active_with_gas = bool(
      self.FPCP.alternativeExperience & ALTERNATIVE_EXPERIENCE.LONGITUDINAL_ACTIVE_WITH_GAS
    )

    self.LoC = LongControl(self.CP)
    self.VM = VehicleModel(self.CP)
    self.LaC: LatControl
    if self.CP.steerControlType == car.CarParams.SteerControlType.angle:
      self.LaC = LatControlAngle(self.CP, self.CI, DT_CTRL)
    elif self.CP.steerControlType == car.CarParams.SteerControlType.curvature:
      self.LaC = LatControlCurvature(self.CP, self.CI, DT_CTRL)
    elif self.CP.lateralTuning.which() == 'pid':
      self.LaC = LatControlPID(self.CP, self.CI, DT_CTRL)
    elif self.CP.lateralTuning.which() == 'torque':
      self.LaC = LatControlTorque(self.CP, self.CI, DT_CTRL)

    # FrogPilot variables
    self.sm = self.sm.extend(['liveDelay', 'frogpilotCarState', 'frogpilotPlan'])

    self.frogpilot_toggles = get_frogpilot_toggles()

    if self.CP.steerControlType != car.CarParams.SteerControlType.curvature and self.CP.lateralTuning.which() == "torque" and \
       (self.frogpilot_toggles.nnff or self.frogpilot_toggles.nnff_lite):
      self.LaC = LatControlNNFF(self.CP, self.CI, DT_CTRL)

  def update(self):
    self.sm.update(15)
    if self.sm.frame % 10 == 0:
      self.live_update_handoff_state = self.params.get(LIVE_UPDATE_HANDOFF_PARAM) or ""
    if self.sm.updated["liveCalibration"]:
      self.pose_calibrator.feed_live_calib(self.sm['liveCalibration'])
    if self.sm.updated["livePose"]:
      device_pose = Pose.from_live_pose(self.sm['livePose'])
      self.calibrated_pose = self.pose_calibrator.build_calibrated_pose(device_pose)

    # FrogPilot variables
    if hasattr(self.LaC, "pid") and self.CP.lateralTuning.which() != "pid":
      self.LaC.pid._k_p = self.frogpilot_toggles.steerKp

    if self.sm.updated['liveDelay'] and hasattr(self.LaC, "update_live_delay"):
      self.LaC.update_live_delay(self.sm['liveDelay'].lateralDelay)

    self.frogpilot_toggles = get_frogpilot_toggles(self.sm)

  def state_control(self):
    CS = self.sm['carState']

    # Update VehicleModel
    lp = self.sm['liveParameters']
    x = max(lp.stiffnessFactor, 0.1)
    sr = max(lp.steerRatio, 0.1)
    self.VM.update_params(x, sr)

    steer_angle_without_offset = math.radians(CS.steeringAngleDeg - lp.angleOffsetDeg)
    self.curvature = -self.VM.calc_curvature(steer_angle_without_offset, CS.vEgo, lp.roll)

    # Update Torque Params
    if self.CP.lateralTuning.which() == 'torque':
      torque_params = self.sm['liveTorqueParameters']
      if self.sm.all_checks(['liveTorqueParameters']) and (torque_params.useParams or self.frogpilot_toggles.force_auto_tune):
        self.LaC.update_live_torque_params(torque_params.latAccelFactorFiltered, torque_params.latAccelOffsetFiltered,
                                           torque_params.frictionCoefficientFiltered)

    long_plan = self.sm['longitudinalPlan']
    model_v2 = self.sm['modelV2']

    CC = car.CarControl.new_message()
    live_update_handoff_active = state_name(self.live_update_handoff_state) in PANDA_HANDOFF_STATES
    CC.enabled = self.sm['selfdriveState'].enabled and not live_update_handoff_active

    # Check which actuators can be enabled
    standstill = abs(CS.vEgo) <= max(self.CP.minSteerSpeed, 0.3) or CS.standstill
    CC.latActive = not live_update_handoff_active and (self.sm['selfdriveState'].active or self.sm['frogpilotCarState'].alwaysOnLateralEnabled) and \
                   not CS.steerFaultTemporary and not CS.steerFaultPermanent and \
                   (not standstill or self.CP.steerAtStandstill) and self.sm['frogpilotPlan'].lateralCheck
    override_longitudinal = any(e.overrideLongitudinal for e in self.sm['onroadEvents'])
    gas_override = self.longitudinal_active_with_gas and CS.gasPressed
    CC.longActive = longitudinal_control_active(
      CC.enabled,
      self.CP.openpilotLongitudinalControl,
      self.sm['frogpilotCarState'].pauseLongitudinal,
      override_longitudinal,
      self.longitudinal_active_with_gas,
      CS.gasPressed,
    )

    actuators = CC.actuators
    actuators.longControlState = self.LoC.long_control_state

    # Enable blinkers while lane changing
    if model_v2.meta.laneChangeState != LaneChangeState.off:
      CC.leftBlinker = model_v2.meta.laneChangeDirection == LaneChangeDirection.left
      CC.rightBlinker = model_v2.meta.laneChangeDirection == LaneChangeDirection.right

    if not CC.latActive:
      self.LaC.reset()
    if not CC.longActive:
      self.LoC.reset()

    # accel PID loop
    pid_accel_limits = self.CI.get_pid_accel_limits(self.CP, CS.vEgo, CS.vCruise * CV.KPH_TO_MS)
    actuators.accel = float(min(
      self.LoC.update(
        CC.longActive,
        CS,
        long_plan.aTarget,
        long_plan.shouldStop,
        long_plan.distanceToStopTarget,
        pid_accel_limits,
        self.frogpilot_toggles,
        experimental_mode=self.sm["selfdriveState"].experimentalMode,
        lead_status=self.sm["radarState"].leadOne.status,
        lead_v=self.sm["radarState"].leadOne.vLead,
        lead_d_rel=self.sm["radarState"].leadOne.dRel,
        lead_a=self.sm["radarState"].leadOne.aLeadK,
        lead_track_id=self.sm["radarState"].leadOne.radarTrackId,
        lead_model_prob=self.sm["radarState"].leadOne.modelProb,
        lead2_status=self.sm["radarState"].leadTwo.status,
        lead2_v=self.sm["radarState"].leadTwo.vLead,
        lead2_d_rel=self.sm["radarState"].leadTwo.dRel,
        fcw=bool(long_plan.fcw),
        model_stop_d=float(long_plan.distanceToStopTargetModel),
        model_should_stop=model_v2.action.shouldStop,
        force_coast=self.sm["frogpilotCarState"].forceCoast,
        increased_stopped_distance=self.sm["frogpilotPlan"].increasedStoppedDistance,
        a_target_trajectory=(long_plan.aTargetTrajectory if long_plan.aTargetTrajectoryValid else None),
        freeze_integrator=gas_override,
        plan_valid=self.sm.valid['longitudinalPlan'],
        id_inputs=self._identification_inputs(CS, CC) if stopping_flags.IDENTIFICATION_HOOK else None,
      ),
      self.frogpilot_toggles.max_desired_acceleration,
    ))
    actuators.accel = longitudinal_accel_with_gas(actuators.accel, self.longitudinal_active_with_gas, CS.gasPressed)
    if stopping_flags.IDENTIFICATION_HOOK and self.LoC.id_hook_out is not None:
      # banner through the existing alertDebug -> "longitudinal maneuver" alert path (selfdrived); logged in the rlog
      hook = self.LoC.id_hook_out
      alert_msg = messaging.new_message('alertDebug')
      alert_msg.valid = True
      alert_msg.alertDebug.alertText1 = hook.text1
      alert_msg.alertDebug.alertText2 = hook.text2
      self.pm.send('alertDebug', alert_msg)

    # Steering PID loop and lateral MPC
    # Reset desired curvature to current to avoid violating the limits on engage
    new_desired_curvature = model_v2.action.desiredCurvature if CC.latActive else self.curvature
    self.desired_curvature, curvature_limited = clip_curvature(CS.vEgo, self.desired_curvature, new_desired_curvature, lp.roll)
    lat_delay = self.sm["liveDelay"].lateralDelay + LAT_SMOOTH_SECONDS

    actuators.curvature = self.desired_curvature
    steer, lateral_output, lac_log = self.LaC.update(CC.latActive, CS, self.VM, lp,
                                                     self.steer_limited_by_safety, self.desired_curvature,
                                                     curvature_limited, lat_delay,
                                                     self.calibrated_pose,
                                                     self.sm['modelV2'],
                                                     self.frogpilot_toggles)
    actuators.torque = float(steer)
    if self.CP.steerControlType == car.CarParams.SteerControlType.curvature:
      actuators.curvature = float(lateral_output)
    else:
      actuators.steeringAngleDeg = float(lateral_output)

    # OPGM variables
    if len(long_plan.speeds):
      actuators.speed = long_plan.speeds[-1]

    # Ensure no NaNs/Infs
    for p in ACTUATOR_FIELDS:
      attr = getattr(actuators, p)
      if not isinstance(attr, Number):
        continue

      if not math.isfinite(attr):
        cloudlog.error(f"actuators.{p} not finite {actuators.to_dict()}")
        setattr(actuators, p, 0.0)

    return CC, lac_log

  def _identification_inputs(self, CS, CC) -> HookInputs:
    """Validated envelope inputs for the TEMPORARY identification step hook (identification_hook.py)."""
    sm, t = self.sm, self.frogpilot_toggles
    fcs, lp, rs, mv = sm['frogpilotCarState'], sm['longitudinalPlan'], sm['radarState'], sm['modelV2']
    leads = list(mv.leadsV3)[:2]
    mapping_ok = not any(getattr(t, k, False) for k in (
      "experimental_mode_via_distance_long", "force_coast_via_distance_long", "pause_lateral_via_distance_long",
      "pause_longitudinal_via_distance_long", "personality_profile_via_distance_long", "traffic_mode_via_distance_long",
      "experimental_mode_via_distance_very_long", "force_coast_via_distance_very_long", "pause_lateral_via_distance_very_long",
      "pause_longitudinal_via_distance_very_long", "personality_profile_via_distance_very_long", "traffic_mode_via_distance_very_long"))
    valid = all(sm.valid[s] and sm.alive[s] for s in ('carState', 'radarState', 'modelV2', 'longitudinalPlan', 'livePose'))
    err = rs.radarErrors
    return HookInputs(
      valid=bool(valid), santa_fe=self.CP.carFingerprint == "HYUNDAI_SANTA_FE_HEV_2022",
      long_active=bool(CC.longActive and self.CP.openpilotLongitudinalControl), enabled=bool(sm['selfdriveState'].enabled),
      pid_state=self.LoC.long_control_state == LongCtrlState.pid, v_ego=float(CS.vEgo), gas=bool(CS.gasPressed), brake=bool(CS.brakePressed),
      force_coast=bool(fcs.forceCoast), pause_long=bool(fcs.pauseLongitudinal), standstill=bool(CS.standstill),
      steer_deg=float(CS.steeringAngleDeg), yaw_rate=float(CS.yawRate), blinker=bool(CS.leftBlinker or CS.rightBlinker),
      steer_fault=bool(CS.steerFaultTemporary or CS.steerFaultPermanent), esp_active=bool(CS.espActive), acc_faulted=bool(CS.accFaulted),
      can_valid=bool(CS.canValid), gear_drive=CS.gearShifter == car.CarState.GearShifter.drive, stock_aeb=bool(CS.stockAeb),
      stock_fcw=bool(CS.stockFcw), lead_status=bool(rs.leadOne.status or rs.leadTwo.status),
      radar_error=bool(err.canError or err.radarFault or err.wrongConfig or err.radarUnavailableTemporary),
      lead_prob=max([float(ld.prob) for ld in leads] or [0.0]), plan_has_lead=bool(lp.hasLead), plan_should_stop=bool(lp.shouldStop),
      plan_fcw=bool(lp.fcw), stop_target_m=float(lp.distanceToStopTarget), distance_pressed=bool(fcs.distancePressed), mapping_ok=mapping_ok)

  def publish(self, CC, lac_log):
    CS = self.sm['carState']

    # Orientation and angle rates can be useful for carcontroller
    # Only calibrated (car) frame is relevant for the carcontroller
    CC.currentCurvature = self.curvature
    if self.calibrated_pose is not None:
      CC.orientationNED = self.calibrated_pose.orientation.xyz.tolist()
      CC.angularVelocity = self.calibrated_pose.angular_velocity.xyz.tolist()

    CC.cruiseControl.override = longitudinal_control_override(CC.enabled, self.CP.openpilotLongitudinalControl, CC.longActive,
                                                             self.longitudinal_active_with_gas, CS.gasPressed)
    CC.cruiseControl.cancel = CS.cruiseState.enabled and (not CC.enabled or not self.CP.pcmCruise)
    CC.cruiseControl.resume = CC.enabled and CS.cruiseState.standstill and not self.sm['longitudinalPlan'].shouldStop

    hudControl = CC.hudControl
    hudControl.setSpeed = float(CS.vCruiseCluster * CV.KPH_TO_MS)
    hudControl.speedVisible = CC.enabled
    hudControl.lanesVisible = CC.enabled
    hudControl.leadVisible = self.sm['longitudinalPlan'].hasLead
    personality = self.sm['selfdriveState'].personality.raw
    if self.CP.brand == "hyundai" and self.CP.openpilotLongitudinalControl and self.frogpilot_toggles.personality_profile_via_distance_long and \
       self.sm['frogpilotCarState'].distanceLongPressed:
      personality = (personality - 1) % 3
    hudControl.leadDistanceBars = personality + 1
    hudControl.visualAlert = self.sm['selfdriveState'].alertHudVisual

    hudControl.rightLaneVisible = True
    hudControl.leftLaneVisible = True
    if self.sm.valid['driverAssistance']:
      hudControl.leftLaneDepart = self.sm['driverAssistance'].leftLaneDeparture
      hudControl.rightLaneDepart = self.sm['driverAssistance'].rightLaneDeparture

    if self.sm['selfdriveState'].active:
      CO = self.sm['carOutput']
      if self.CP.steerControlType == car.CarParams.SteerControlType.angle:
        self.steer_limited_by_safety = abs(CC.actuators.steeringAngleDeg - CO.actuatorsOutput.steeringAngleDeg) > \
                                              STEER_ANGLE_SATURATION_THRESHOLD
      else:
        self.steer_limited_by_safety = abs(CC.actuators.torque - CO.actuatorsOutput.torque) > 1e-2

    # TODO: both controlsState and carControl valids should be set by
    #       sm.all_checks(), but this creates a circular dependency

    # controlsState
    dat = messaging.new_message('controlsState')
    dat.valid = CS.canValid
    cs = dat.controlsState

    cs.curvature = self.curvature
    cs.longitudinalPlanMonoTime = self.sm.logMonoTime['longitudinalPlan']
    cs.lateralPlanMonoTime = self.sm.logMonoTime['modelV2']
    cs.desiredCurvature = self.desired_curvature
    cs.longControlState = self.LoC.long_control_state
    cs.upAccelCmd = float(self.LoC.pid.p)
    cs.uiAccelCmd = float(self.LoC.pid.i)
    cs.ufAccelCmd = float(self.LoC.pid.f)
    cs.forceDecel = bool((self.sm['driverMonitoringState'].awarenessStatus < 0.) or
                         (self.sm['selfdriveState'].state == State.softDisabling) or self.sm["frogpilotCarState"].forceCoast)

    # trigger the car's stock driver monitoring escalation
    CC.driverMonitoringEscalation = cs.forceDecel

    lat_tuning = self.CP.lateralTuning.which()
    if self.CP.steerControlType == car.CarParams.SteerControlType.angle:
      cs.lateralControlState.angleState = lac_log
    elif self.CP.steerControlType == car.CarParams.SteerControlType.curvature:
      cs.lateralControlState.curvatureState = lac_log
    elif lat_tuning == 'pid':
      cs.lateralControlState.pidState = lac_log
    elif lat_tuning == 'torque':
      cs.lateralControlState.torqueState = lac_log

    self.pm.send('controlsState', dat)

    # carControl
    cc_send = messaging.new_message('carControl')
    cc_send.valid = CS.canValid
    cc_send.carControl = CC
    self.pm.send('carControl', cc_send)

  def run(self):
    rk = Ratekeeper(100, print_delay_threshold=None)
    while True:
      self.update()
      CC, lac_log = self.state_control()
      self.publish(CC, lac_log)
      rk.monitor_time()


def main():
  config_realtime_process(4, Priority.CTRL_HIGH)
  controls = Controls()
  controls.run()


if __name__ == "__main__":
  main()
