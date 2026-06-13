import math
import numpy as np
from opendbc.can import CANPacker
from opendbc.car import Bus, DT_CTRL, make_tester_present_msg, structs
from opendbc.car.carlog import carlog
from opendbc.car.lateral import apply_driver_steer_torque_limits, common_fault_avoidance
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.hyundai import hyundaicanfd, hyundaican
from opendbc.car.hyundai.hyundaicanfd import CanBus
from opendbc.car.hyundai.values import HyundaiFlags, Buttons, CarControllerParams, CAR
from opendbc.car.interfaces import CarControllerBase

VisualAlert = structs.CarControl.HUDControl.VisualAlert
LongCtrlState = structs.CarControl.Actuators.LongControlState

# EPS faults if you apply torque while the steering angle is above 90 degrees for more than 1 second
# All slightly below EPS thresholds to avoid fault
MAX_ANGLE = 85
MAX_ANGLE_FRAMES = 89
MAX_ANGLE_CONSECUTIVE_FRAMES = 2

# Stopping-stack CAN-layer constants (stopping redesign spec §4.3-4.5). Defaults are byte-identical to
# legacy behavior; protocol stages (docs/stopping/on_vehicle_protocols.md) change one constant per session.
STOP_REQ_MAX_SPEED = 0.01     # m/s. Protocol knob. COMFORT EXPERIMENT == 0.01 (== legacy known-good): keeps
# the SCC OUT of the final stop so openpilot commands the terminal decel down to standstill (the controllable,
# IMU-measurable regime) instead of handing off to the SCC managed stop at 0.04. 0.04 was StopReq STAGE A
# (SCC owns the final stop, below the 0.104 m/s wheel-standstill threshold). 0.0001 REJECTED: with the latch
# the gate is the set-threshold and Kalman-filtered vEgo dithers near zero, so 0.0001 may never assert StopReq
# at all — leaving the standstill hold entirely to openpilot's command with no SCC managed stop (untested HKG
# behavior). Change ONLY via on_vehicle_protocols.md §1.
# STOPREQ_RELEASE_SPEED: latch speed-release, ALWAYS active at every protocol stage. Just below the
# 0.104 m/s wheel-speed standstill threshold: the latch may NEVER hold StopReq on a rolling car (F1).
STOPREQ_RELEASE_SPEED = 0.10  # m/s
STOPREQ_LATCH = True          # protocol STAGE A (on_vehicle_protocols.md §1). True enables the chatter-fix
# latch: set on stopping ∧ vEgo < gate; cleared on state-exit OR vEgo > STOPREQ_RELEASE_SPEED. False == legacy.
DYNAMIC_SCC14_JERK = False    # KILL SWITCH: False == legacy static 3.0/1.0/5.0
SCC14_JERK_MARGIN = 0.5       # m/s^3 headroom above observed command slew
SCC14_JERK_UPPER_PID = 3.0
SCC14_JERK_UPPER_STOPPING = 1.0
SCC14_JERK_LOWER = 5.0
SCC14_JERK_MAX = 12.7         # DBC ceiling; CANPacker WRAPS out-of-range, it does not clamp
REPORT_SENT_ACCEL = True      # KILL SWITCH: False restores legacy (pre-cap) telemetry
TELEMETRY_VERSION = 2         # carOutput.actuatorsOutput.accel reports the SENT accel (engagement cap included)


def scc14_jerk_floors(long_control_state):
  # Dynamic floors == the legacy static values: the dynamic path may only ADD jerk headroom over what
  # the car runs today, never advertise less, until protocol comparison data justifies lowering (§4.5).
  upper_floor = SCC14_JERK_UPPER_PID if long_control_state == LongCtrlState.pid else SCC14_JERK_UPPER_STOPPING
  return upper_floor, SCC14_JERK_LOWER


def process_hud_alert(enabled, fingerprint, hud_control):
  sys_warning = (hud_control.visualAlert in (VisualAlert.steerRequired, VisualAlert.ldw))

  # initialize to no line visible
  # TODO: this is not accurate for all cars
  sys_state = 1
  if hud_control.leftLaneVisible and hud_control.rightLaneVisible or sys_warning:  # HUD alert only display when LKAS status is active
    sys_state = 3 if enabled or sys_warning else 4
  elif hud_control.leftLaneVisible:
    sys_state = 5
  elif hud_control.rightLaneVisible:
    sys_state = 6

  # initialize to no warnings
  left_lane_warning = 0
  right_lane_warning = 0
  if hud_control.leftLaneDepart:
    left_lane_warning = 1 if fingerprint in (CAR.GENESIS_G90, CAR.GENESIS_G80) else 2
  if hud_control.rightLaneDepart:
    right_lane_warning = 1 if fingerprint in (CAR.GENESIS_G90, CAR.GENESIS_G80) else 2

  return sys_warning, sys_state, left_lane_warning, right_lane_warning


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.CAN = CanBus(CP)
    self.params = CarControllerParams(CP)
    self.packer = CANPacker(dbc_names[Bus.pt])
    self.angle_limit_counter = 0

    self.accel_last = 0
    self.apply_torque_last = 0
    self.car_fingerprint = CP.carFingerprint
    self.last_button_frame = 0
    self.engaged_frame = 0
    self.stopreq_latched = False
    self.accel_last_scc = 0.0

  def update(self, CC, CS, now_nanos, frogpilot_toggles):
    actuators = CC.actuators
    hud_control = CC.hudControl

    # engagement rising-edge bookkeeping, moved here from create_can_msgs so the hoisted cap below
    # sees it on the same frame it used to (cap window bit-identical incl. the rising-edge frame, §4.3)
    self.engaged_frame = self.frame if CC.longActive and self.engaged_frame == 0 else 0 if not CC.longActive else self.engaged_frame

    # steering torque
    new_torque = int(round(actuators.torque * self.params.STEER_MAX))
    apply_torque = apply_driver_steer_torque_limits(new_torque, self.apply_torque_last, CS.out.steeringTorque, self.params)

    # >90 degree steering fault prevention
    self.angle_limit_counter, apply_steer_req = common_fault_avoidance(abs(CS.out.steeringAngleDeg) >= MAX_ANGLE, CC.latActive,
                                                                       self.angle_limit_counter, MAX_ANGLE_FRAMES,
                                                                       MAX_ANGLE_CONSECUTIVE_FRAMES)

    if not CC.latActive:
      apply_torque = 0

    # Hold torque with induced temporary fault when cutting the actuation bit
    # FIXME: we don't use this with CAN FD?
    torque_fault = CC.latActive and not apply_steer_req

    self.apply_torque_last = apply_torque

    # accel + longitudinal
    # non-finite neutralization BEFORE the clip: np.clip propagates NaN, CANPacker raises on NaN/inf
    # and the 50 Hz SCC12 stream dies -> cruise fault with brakes released (spec binding principle #2)
    accel = float(actuators.accel)
    if not math.isfinite(accel):
      carlog.error("non-finite accel from controls; forcing 0.0")
      accel = 0.0
    accel = float(np.clip(accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX))

    # StopReq: legacy standstill gate, with an optional chatter-fix latch carrying an always-active
    # speed release — the latch may never hold StopReq on a rolling car (creep-push, §4.4/F1)
    stopreq_now = actuators.longControlState == LongCtrlState.stopping and CS.out.vEgo < STOP_REQ_MAX_SPEED
    if STOPREQ_LATCH:
      if actuators.longControlState != LongCtrlState.stopping:
        self.stopreq_latched = False
      elif CS.out.vEgo > STOPREQ_RELEASE_SPEED:
        self.stopreq_latched = False
      elif stopreq_now:
        self.stopreq_latched = True
      stopping = self.stopreq_latched
    else:
      stopping = stopreq_now

    set_speed_in_units = hud_control.setSpeed * (CV.MS_TO_KPH if CS.is_metric else CV.MS_TO_MPH)

    # fork post-engagement launch cap, hoisted from create_can_msgs so telemetry reports the accel
    # actually sent (§4.3); computed at 100 Hz with identical math — constant across the 2-frame SCC window
    if self.CP.flags & HyundaiFlags.CANFD:
      accel_sent = accel  # CAN-FD path keeps its own slew logic untouched (out of scope, classic-CAN car)
    else:
      engaged_active = CS.out.vEgo * CV.MS_TO_KPH < 50.0 and (self.frame - self.engaged_frame) * DT_CTRL < 4.0
      if engaged_active and accel > CS.out.aEgo and CS.out.aEgo < 0.5:
        accel_sent = min(accel, max(CS.out.aEgo * 1.3, 0.6))
      else:
        accel_sent = accel

    can_sends = []

    # *** common hyundai stuff ***

    # tester present - w/ no response (keeps relevant ECU disabled)
    if self.frame % 100 == 0 and not (self.CP.flags & HyundaiFlags.CANFD_CAMERA_SCC) and self.CP.openpilotLongitudinalControl:
      # for longitudinal control, either radar or ADAS driving ECU
      addr, bus = 0x7d0, self.CAN.ECAN if self.CP.flags & HyundaiFlags.CANFD else 0
      if self.CP.flags & HyundaiFlags.CANFD_LKA_STEERING.value:
        addr, bus = 0x730, self.CAN.ECAN
      can_sends.append(make_tester_present_msg(addr, bus, suppress_response=True))

      # for blinkers
      if self.CP.flags & HyundaiFlags.ENABLE_BLINKERS:
        can_sends.append(make_tester_present_msg(0x7b1, self.CAN.ECAN, suppress_response=True))

    # *** CAN/CAN FD specific ***
    if self.CP.flags & HyundaiFlags.CANFD:
      can_sends.extend(self.create_canfd_msgs(apply_steer_req, apply_torque, set_speed_in_units, accel,
                                              stopping, hud_control, CS, CC))
    else:
      can_sends.extend(self.create_can_msgs(apply_steer_req, apply_torque, torque_fault, set_speed_in_units, accel_sent,
                                            stopping, hud_control, actuators, CS, CC))

    new_actuators = actuators.as_builder()
    new_actuators.torque = apply_torque / self.params.STEER_MAX
    new_actuators.torqueOutputCan = apply_torque
    new_actuators.accel = accel_sent if REPORT_SENT_ACCEL else accel

    self.frame += 1
    return new_actuators, can_sends

  def create_can_msgs(self, apply_steer_req, apply_torque, torque_fault, set_speed_in_units, accel, stopping, hud_control, actuators, CS, CC):
    can_sends = []
    # Keep Hyundai lateral-facing status active when AOL keeps steering alive after long disengage.
    lat_enabled = CC.enabled or CC.latActive

    # HUD messages
    sys_warning, sys_state, left_lane_warning, right_lane_warning = process_hud_alert(lat_enabled, self.car_fingerprint,
                                                                                      hud_control)

    can_sends.append(hyundaican.create_lkas11(self.packer, self.frame, self.CP, apply_torque, apply_steer_req,
                                              torque_fault, CS.lkas11, sys_warning, sys_state, lat_enabled,
                                              hud_control.leftLaneVisible, hud_control.rightLaneVisible,
                                              left_lane_warning, right_lane_warning))

    # Button messages
    if not self.CP.openpilotLongitudinalControl:
      if CC.cruiseControl.cancel:
        can_sends.append(hyundaican.create_clu11(self.packer, self.frame, CS.clu11, Buttons.CANCEL, self.CP))
      elif CC.cruiseControl.resume:
        # send resume at a max freq of 10Hz
        if (self.frame - self.last_button_frame) * DT_CTRL > 0.1:
          # send 25 messages at a time to increases the likelihood of resume being accepted
          can_sends.extend([hyundaican.create_clu11(self.packer, self.frame, CS.clu11, Buttons.RES_ACCEL, self.CP)] * 25)
          if (self.frame - self.last_button_frame) * DT_CTRL >= 0.15:
            self.last_button_frame = self.frame

    if self.frame % 2 == 0 and self.CP.openpilotLongitudinalControl:
      # RATE PINNED (§4.5/F6): jerk is computed only inside this 50 Hz send block, so accel_last_scc is
      # updated ONLY on sent frames and the differencing interval really is 2*DT_CTRL
      if DYNAMIC_SCC14_JERK:
        cmd_jerk = (accel - self.accel_last_scc) / (2 * DT_CTRL)  # true 50 Hz command slope
        upper_floor, lower_floor = scc14_jerk_floors(actuators.longControlState)
        upper_jerk = float(np.clip(max(cmd_jerk, 0.0) + SCC14_JERK_MARGIN, upper_floor, SCC14_JERK_MAX))
        lower_jerk = float(np.clip(max(-cmd_jerk, 0.0) + SCC14_JERK_MARGIN, lower_floor, SCC14_JERK_MAX))
      else:
        upper_jerk = SCC14_JERK_UPPER_PID if actuators.longControlState == LongCtrlState.pid else SCC14_JERK_UPPER_STOPPING
        lower_jerk = SCC14_JERK_LOWER
      self.accel_last_scc = accel
      use_fca = self.CP.flags & HyundaiFlags.USE_FCA.value
      can_sends.extend(hyundaican.create_acc_commands(self.packer, CC.enabled, accel, upper_jerk, int(self.frame / 2),
                                                      hud_control, set_speed_in_units, stopping,
                                                      CC.cruiseControl.override, use_fca, CS.out.cruiseState.available, self.CP,
                                                      lower_jerk=lower_jerk))

    # 20 Hz LFA MFA message
    if self.frame % 5 == 0 and self.CP.flags & HyundaiFlags.SEND_LFA.value:
      can_sends.append(hyundaican.create_lfahda_mfc(self.packer, CC.enabled, CC.latActive))

    # 5 Hz ACC options
    if self.frame % 20 == 0 and self.CP.openpilotLongitudinalControl:
      can_sends.extend(hyundaican.create_acc_opt(self.packer, self.CP))

    # 2 Hz front radar options
    if self.frame % 50 == 0 and self.CP.openpilotLongitudinalControl:
      can_sends.append(hyundaican.create_frt_radar_opt(self.packer))

    return can_sends

  def create_canfd_msgs(self, apply_steer_req, apply_torque, set_speed_in_units, accel, stopping, hud_control, CS, CC):
    can_sends = []
    # Keep Hyundai lateral-facing status active when AOL keeps steering alive after long disengage.
    lat_enabled = CC.enabled or CC.latActive

    lka_steering = self.CP.flags & HyundaiFlags.CANFD_LKA_STEERING
    lka_steering_long = lka_steering and self.CP.openpilotLongitudinalControl

    # steering control
    can_sends.extend(hyundaicanfd.create_steering_messages(self.packer, self.CP, self.CAN, lat_enabled, apply_steer_req, apply_torque))

    # prevent LFA from activating on LKA steering cars by sending "no lane lines detected" to ADAS ECU
    if self.frame % 5 == 0 and lka_steering:
      can_sends.append(hyundaicanfd.create_suppress_lfa(self.packer, self.CAN, CS.lfa_block_msg,
                                                        self.CP.flags & HyundaiFlags.CANFD_LKA_STEERING_ALT))

    # LFA and HDA icons
    if self.frame % 5 == 0 and (not lka_steering or lka_steering_long):
      can_sends.append(hyundaicanfd.create_lfahda_cluster(self.packer, self.CAN, lat_enabled, CC.latActive))

    # blinkers
    if lka_steering and self.CP.flags & HyundaiFlags.ENABLE_BLINKERS:
      can_sends.extend(hyundaicanfd.create_spas_messages(self.packer, self.CAN, CC.leftBlinker, CC.rightBlinker))

    if self.CP.openpilotLongitudinalControl:
      if lka_steering:
        can_sends.extend(hyundaicanfd.create_adrv_messages(self.packer, self.CAN, self.frame))
      else:
        can_sends.extend(hyundaicanfd.create_fca_warning_light(self.packer, self.CAN, self.frame))
      if self.frame % 2 == 0:
        can_sends.append(hyundaicanfd.create_acc_control(self.packer, self.CAN, CC.enabled, self.accel_last, accel, stopping, CC.cruiseControl.override,
                                                         set_speed_in_units, hud_control))
        self.accel_last = accel
    else:
      # button presses
      if (self.frame - self.last_button_frame) * DT_CTRL > 0.25:
        # cruise cancel
        if CC.cruiseControl.cancel:
          if self.CP.flags & HyundaiFlags.CANFD_ALT_BUTTONS:
            can_sends.append(hyundaicanfd.create_acc_cancel(self.packer, self.CP, self.CAN, CS.cruise_info))
            self.last_button_frame = self.frame
          else:
            for _ in range(20):
              can_sends.append(hyundaicanfd.create_buttons(self.packer, self.CP, self.CAN, CS.buttons_counter + 1, Buttons.CANCEL))
            self.last_button_frame = self.frame

        # cruise standstill resume
        elif CC.cruiseControl.resume:
          if self.CP.flags & HyundaiFlags.CANFD_ALT_BUTTONS:
            # TODO: resume for alt button cars
            pass
          else:
            for _ in range(20):
              can_sends.append(hyundaicanfd.create_buttons(self.packer, self.CP, self.CAN, CS.buttons_counter + 1, Buttons.RES_ACCEL))
            self.last_button_frame = self.frame

    return can_sends
