#!/usr/bin/env python3
import os
import time
import threading

import cereal.messaging as messaging

from cereal import car, custom, log

from openpilot.common.params import Params
from openpilot.common.realtime import config_realtime_process, Priority, Ratekeeper
from openpilot.common.swaglog import cloudlog, ForwardingHandler

from opendbc.car import DT_CTRL, ButtonType, structs
from opendbc.car.can_definitions import CanData, CanRecvCallable, CanSendCallable
from opendbc.car.carlog import carlog
from opendbc.car.fw_versions import ObdCallback
from opendbc.car.car_helpers import get_car, interfaces
from opendbc.car.interfaces import CarInterfaceBase, RadarInterfaceBase
from opendbc.safety import ALTERNATIVE_EXPERIENCE
from openpilot.selfdrive.pandad import can_capnp_to_list, can_list_to_can_capnp
from openpilot.selfdrive.car.cruise import VCruiseHelper
from openpilot.selfdrive.car.car_specific import MockCarState
from openpilot.selfdrive.car.live_update_handoff import DIAGNOSTIC, DIAGNOSTIC_REQUESTED, FAILED, LIVE_UPDATE_HANDOFF_PARAM, \
                                                         PRECONDITION_SECONDS, RADAR_RESTORE_RETRY_SECONDS, \
                                                         READY, READY_REFRESH_SECONDS, REQUESTED, \
                                                         StockSccVerifier, UNSUPPORTED, VERIFYING, VERIFY_TIMEOUT_SECONDS, \
                                                         controls_disengagement_reasons, is_supported_car, should_suppress_always_on_lateral, \
                                                         state_name, state_timestamp, timestamped_state

from openpilot.frogpilot.common.frogpilot_variables import get_frogpilot_toggles, update_frogpilot_toggles
from openpilot.frogpilot.controls.frogpilot_card import FrogPilotCard

REPLAY = "REPLAY" in os.environ

EventName = log.OnroadEvent.EventName
HANDOFF_CRUISE_BUTTONS = (ButtonType.accelCruise, ButtonType.decelCruise, ButtonType.cancel,
                          ButtonType.resumeCruise, ButtonType.setCruise, ButtonType.mainCruise)

# forward
carlog.addHandler(ForwardingHandler(cloudlog))


def obd_callback(params: Params) -> ObdCallback:
  def set_obd_multiplexing(obd_multiplexing: bool):
    if params.get_bool("ObdMultiplexingEnabled") != obd_multiplexing:
      cloudlog.warning(f"Setting OBD multiplexing to {obd_multiplexing}")
      params.remove("ObdMultiplexingChanged")
      params.put_bool("ObdMultiplexingEnabled", obd_multiplexing)
      params.get_bool("ObdMultiplexingChanged", block=True)
      cloudlog.warning("OBD multiplexing set successfully")
  return set_obd_multiplexing


def can_comm_callbacks(logcan: messaging.SubSocket, sendcan: messaging.PubSocket) -> tuple[CanRecvCallable, CanSendCallable]:
  def can_recv(wait_for_one: bool = False) -> list[list[CanData]]:
    """
    wait_for_one: wait the normal logcan socket timeout for a CAN packet, may return empty list if nothing comes

    Returns: CAN packets comprised of CanData objects for easy access
    """
    ret = []
    for can in messaging.drain_sock(logcan, wait_for_one=wait_for_one):
      ret.append([CanData(msg.address, msg.dat, msg.src) for msg in can.can])
    return ret

  def can_send(msgs: list[CanData]) -> None:
    sendcan.send(can_list_to_can_capnp(msgs, msgtype='sendcan'))

  return can_recv, can_send


class Car:
  CI: CarInterfaceBase
  RI: RadarInterfaceBase
  CP: car.CarParams

  # FrogPilot variables
  FPCP: custom.FrogPilotCarParams

  def __init__(self, CI=None, RI=None) -> None:
    self.can_sock = messaging.sub_sock('can', timeout=20)
    self.sm = messaging.SubMaster(['pandaStates', 'carControl', 'onroadEvents'])
    self.pm = messaging.PubMaster(['sendcan', 'carState', 'carParams', 'carOutput', 'liveTracks'])

    self.can_rcv_cum_timeout_counter = 0

    self.CC_prev = car.CarControl.new_message()
    self.CS_prev = car.CarState.new_message()
    self.initialized_prev = False

    self.last_actuators_output = structs.CarControl.Actuators()

    self.params = Params()

    self.live_update_handoff_state = self.params.get(LIVE_UPDATE_HANDOFF_PARAM) or ""
    if state_name(self.live_update_handoff_state) == READY:
      self.live_update_handoff_state = timestamped_state(VERIFYING, time.monotonic())
      self.params.put(LIVE_UPDATE_HANDOFF_PARAM, self.live_update_handoff_state)
    self.live_update_handoff_last_read = time.monotonic()
    self.live_update_handoff_preconditions_since = None
    self.live_update_handoff_started_at = None
    self.live_update_handoff_verify_started_at = None
    self.live_update_handoff_radar_restore_last_attempt = None
    self.live_update_handoff_controls_active_since = None
    self.live_update_handoff_pressed_buttons = set()
    self.live_update_handoff_verifier = StockSccVerifier()

    self.can_callbacks = can_comm_callbacks(self.can_sock, self.pm.sock['sendcan'])

    is_release = False

    if CI is None:
      # wait for one pandaState and one CAN packet
      print("Waiting for CAN messages...")
      while True:
        can = messaging.recv_one_retry(self.can_sock)
        if len(can.can) > 0:
          break

      alpha_long_allowed = self.params.get_bool("AlphaLongitudinalEnabled")
      num_pandas = len(messaging.recv_one_retry(self.sm.sock['pandaStates']).pandaStates)

      cached_params = None
      cached_params_raw = self.params.get("CarParamsCache")
      if cached_params_raw is not None:
        with car.CarParams.from_bytes(cached_params_raw) as _cached_params:
          cached_params = _cached_params

      self.CI = get_car(*self.can_callbacks, obd_callback(self.params), alpha_long_allowed, is_release, self.params, num_pandas, cached_params, get_frogpilot_toggles())
      self.RI = interfaces[self.CI.CP.carFingerprint].RadarInterface(self.CI.CP)
      self.CP = self.CI.CP

      # continue onto next fingerprinting step in pandad
      self.params.put_bool("FirmwareQueryDone", True)

      # FrogPilot variables
      self.FPCP = self.CI.FPCP
    else:
      self.CI, self.CP, self.FPCP = CI, CI.CP, CI.FPCP
      self.RI = RI

    self.CP.alternativeExperience = 0
    openpilot_enabled_toggle = self.params.get_bool("OpenpilotEnabledToggle")
    controller_available = self.CI.CC is not None and openpilot_enabled_toggle
    self.CP.passive = not controller_available
    if self.CP.passive:
      safety_config = structs.CarParams.SafetyConfig()
      safety_config.safetyModel = structs.CarParams.SafetyModel.noOutput
      self.CP.safetyConfigs = [safety_config]

    if self.CP.secOcRequired and not is_release:
      # Copy user key if available
      try:
        with open("/cache/params/SecOCKey") as f:
          user_key = f.readline().strip()
          if len(user_key) == 32:
            self.params.put("SecOCKey", user_key)
      except Exception:
        pass

      secoc_key = self.params.get("SecOCKey")
      if secoc_key is not None:
        saved_secoc_key = bytes.fromhex(secoc_key.strip())
        if len(saved_secoc_key) == 16:
          self.CP.secOcKeyAvailable = True
          self.CI.CS.secoc_key = saved_secoc_key
          if controller_available:
            self.CI.CC.secoc_key = saved_secoc_key
        else:
          cloudlog.warning("Saved SecOC key is invalid")

    # Write previous route's CarParams
    prev_cp = self.params.get("CarParamsPersistent")
    if prev_cp is not None:
      self.params.put("CarParamsPrevRoute", prev_cp)

    # Write CarParams for controls and radard
    cp_bytes = self.CP.to_bytes()
    self.params.put("CarParams", cp_bytes)
    self.params.put_nonblocking("CarParamsCache", cp_bytes)
    self.params.put_nonblocking("CarParamsPersistent", cp_bytes)

    self.mock_carstate = MockCarState()
    self.v_cruise_helper = VCruiseHelper(self.CP)

    self.is_metric = self.params.get_bool("IsMetric")
    self.experimental_mode = self.params.get_bool("ExperimentalMode")

    # card is driven by can recv, expected at 100Hz
    self.rk = Ratekeeper(100, print_delay_threshold=None)

    # OPGM variables
    self.resume_prev_button = False

    # FrogPilot variables
    self.frogpilot_toggles = get_frogpilot_toggles()

    if self.frogpilot_toggles.always_on_lateral:
      self.FPCP.alternativeExperience |= ALTERNATIVE_EXPERIENCE.ALWAYS_ON_LATERAL

    fpcp_bytes = self.FPCP.to_bytes()
    self.params.put("FrogPilotCarParams", fpcp_bytes)
    self.params.put_nonblocking("FrogPilotCarParamsPersistent", fpcp_bytes)

    update_frogpilot_toggles()

    self.frogpilot_card = FrogPilotCard(self.CP, self.FPCP)

    self.sm = self.sm.extend(['frogpilotOnroadEvents', 'frogpilotPlan', 'frogpilotSelfdriveState', 'liveCalibration', 'selfdriveState'])
    self.pm = self.pm.extend(['frogpilotCarState'])

  def state_update(self) -> tuple[car.CarState, structs.RadarDataT | None]:
    """carState update loop, driven by can"""

    can_strs = messaging.drain_sock_raw(self.can_sock, wait_for_one=True)
    can_list = can_capnp_to_list(can_strs)
    self.can_list = can_list

    # Update carState from CAN
    CS, FPCS = self.CI.update(can_list, self.frogpilot_toggles)
    if self.CP.brand == 'mock':
      CS, FPCS = self.mock_carstate.update(CS, FPCS)

    # Update radar tracks from CAN
    RD: structs.RadarDataT | None = self.RI.update(can_list)

    self.sm.update(0)

    can_rcv_valid = len(can_strs) > 0

    # Check for CAN timeout
    if not can_rcv_valid:
      self.can_rcv_cum_timeout_counter += 1

    if can_rcv_valid and REPLAY:
      self.can_log_mono_time = messaging.log_from_bytes(can_strs[0]).logMonoTime

    self.v_cruise_helper.update_v_cruise(CS, self.sm['carControl'].enabled, self.is_metric, self.frogpilot_toggles)
    if self.sm['carControl'].enabled and not self.CC_prev.enabled:
      # Use CarState w/ buttons from the step selfdrived enables on
      self.v_cruise_helper.initialize_v_cruise(self.CS_prev, self.experimental_mode, self.resume_prev_button, self.frogpilot_toggles)

    # TODO: mirror the carState.cruiseState struct?
    CS.vCruise = float(self.v_cruise_helper.v_cruise_kph)
    CS.vCruiseCluster = float(self.v_cruise_helper.v_cruise_cluster_kph)

    # OPGM variables
    if any(be.type in (ButtonType.accelCruise, ButtonType.resumeCruise) for be in CS.buttonEvents):
      self.resume_prev_button = True
    elif any(be.type in (ButtonType.decelCruise, ButtonType.setCruise) for be in CS.buttonEvents):
      self.resume_prev_button = False

    # FrogPilot variables
    FPCS = self.frogpilot_card.update(CS, FPCS, self.sm, self.frogpilot_toggles)
    for button_event in CS.buttonEvents:
      for button_type in HANDOFF_CRUISE_BUTTONS:
        if button_event.type == button_type:
          if button_event.pressed:
            self.live_update_handoff_pressed_buttons.add(button_type)
          else:
            self.live_update_handoff_pressed_buttons.discard(button_type)
          break
    if should_suppress_always_on_lateral(self.live_update_handoff_state, CS.cruiseState.available):
      FPCS.alwaysOnLateralEnabled = False

    return CS, RD, FPCS

  def state_publish(self, CS: car.CarState, RD: structs.RadarDataT | None, FPCS: custom.FrogPilotCarState):
    """carState and carParams publish loop"""

    # carParams - logged every 50 seconds (> 1 per segment)
    if self.sm.frame % int(50. / DT_CTRL) == 0:
      cp_send = messaging.new_message('carParams')
      cp_send.valid = True
      cp_send.carParams = self.CP
      self.pm.send('carParams', cp_send)

    # publish new carOutput
    co_send = messaging.new_message('carOutput')
    co_send.valid = self.sm.all_checks(['carControl'])
    co_send.carOutput.actuatorsOutput = self.last_actuators_output
    self.pm.send('carOutput', co_send)

    # kick off controlsd step while we actuate the latest carControl packet
    cs_send = messaging.new_message('carState')
    cs_send.valid = CS.canValid
    cs_send.carState = CS
    cs_send.carState.canErrorCounter = self.can_rcv_cum_timeout_counter
    cs_send.carState.cumLagMs = -self.rk.remaining * 1000.
    self.pm.send('carState', cs_send)

    if RD is not None:
      tracks_msg = messaging.new_message('liveTracks')
      tracks_msg.valid = not any(RD.errors.to_dict().values())
      tracks_msg.liveTracks = RD
      self.pm.send('liveTracks', tracks_msg)

    # FrogPilot variables
    fpcs_send = messaging.new_message('frogpilotCarState')
    fpcs_send.valid = CS.canValid
    fpcs_send.frogpilotCarState = FPCS
    self.pm.send('frogpilotCarState', fpcs_send)

  def controls_update(self, CS: car.CarState, CC: car.CarControl, controls_quiesced=False):
    """control update loop, driven by carControl"""

    if not self.initialized_prev and not controls_quiesced:
      # Initialize CarInterface, once controls are ready
      # TODO: this can make us miss at least a few cycles when doing an ECU knockout
      self.CI.init(self.CP, *self.can_callbacks)
      # signal pandad to switch to car safety mode
      self.params.put_bool_nonblocking("ControlsReady", True)

    if self.sm.all_alive(['carControl']) and not controls_quiesced:
      # send car controls over can
      now_nanos = self.can_log_mono_time if REPLAY else int(time.monotonic() * 1e9)
      self.last_actuators_output, can_sends = self.CI.apply(CC, now_nanos, self.frogpilot_toggles)
      self.pm.send('sendcan', can_list_to_can_capnp(can_sends, msgtype='sendcan', valid=CS.canValid))

      self.CC_prev = CC

  def _read_live_update_handoff_state(self, now: float) -> str:
    if now - self.live_update_handoff_last_read >= 0.05:
      self.live_update_handoff_state = self.params.get(LIVE_UPDATE_HANDOFF_PARAM) or ""
      self.live_update_handoff_last_read = now
    return self.live_update_handoff_state

  def _set_live_update_handoff_state(self, state: str, nonblocking=False) -> None:
    if nonblocking:
      self.params.put_nonblocking(LIVE_UPDATE_HANDOFF_PARAM, state)
    else:
      self.params.put(LIVE_UPDATE_HANDOFF_PARAM, state)
    self.live_update_handoff_state = state
    self.live_update_handoff_last_read = time.monotonic()
    if state_name(state) != REQUESTED:
      self.live_update_handoff_preconditions_since = None

  def _pandas_in_handoff_mode(self) -> bool:
    panda_states = list(self.sm['pandaStates'])
    return (bool(panda_states) and self.sm.all_checks(['pandaStates']) and
            all(ps.safetyModel == car.CarParams.SafetyModel.elm327 and len(ps.faults) == 0 for ps in panda_states))

  def _handoff_controls_disengaged(self, CS, FPCS) -> bool:
    return not self._handoff_controls_active_reasons(CS, FPCS)

  def _handoff_controls_active_reasons(self, CS, FPCS, post_entry: bool = False) -> tuple[str, ...]:
    if not self.sm.all_checks(['carControl', 'selfdriveState']):
      return ("vehicle control state stale",)
    selfdrive_state = self.sm['selfdriveState']
    reasons = list(controls_disengagement_reasons(CS, self.sm['carControl'], FPCS, selfdrive_state.enabled, selfdrive_state.active,
                                                 bool(self.live_update_handoff_pressed_buttons), post_entry))
    if CS.gearShifter != car.CarState.GearShifter.drive:
      reasons.append("gear not drive")
    return tuple(reasons)

  def update_live_update_handoff(self, CS, FPCS, initialized: bool) -> bool:
    now = time.monotonic()
    state = self._read_live_update_handoff_state(now)
    state_value = state_name(state)
    panda_handoff_mode = self._pandas_in_handoff_mode()
    if state_value != REQUESTED:
      self.live_update_handoff_preconditions_since = None

    if state_value == REQUESTED:
      if not is_supported_car(self.CP) or self.CP.passive:
        self._set_live_update_handoff_state(UNSUPPORTED)
        cloudlog.error("live update handoff is unsupported for this car configuration")
        return False

      # A fresh boot can temporarily report ELM327 while fingerprinting. Keep controls active until
      # normal initialization has completed, but quiesce them when explicitly retrying a handoff whose
      # Panda ELM327 mode is already latched from an earlier failed attempt.
      request_quiesced = panda_handoff_mode and self.initialized_prev
      if initialized and self._handoff_controls_disengaged(CS, FPCS):
        if self.live_update_handoff_preconditions_since is None:
          self.live_update_handoff_preconditions_since = now
        elif now - self.live_update_handoff_preconditions_since >= PRECONDITION_SECONDS:
          self.live_update_handoff_started_at = now
          self._set_live_update_handoff_state(DIAGNOSTIC_REQUESTED, nonblocking=True)
          cloudlog.warning("live update handoff requesting Panda diagnostic mode; openpilot CAN output remains active until Panda switches")
          return request_quiesced
      else:
        self.live_update_handoff_preconditions_since = None
      return request_quiesced

    if state_value not in (DIAGNOSTIC_REQUESTED, DIAGNOSTIC, VERIFYING, READY, FAILED):
      return False

    if state_value == FAILED:
      if panda_handoff_mode:
        self.live_update_handoff_verifier.update(self.can_list, now)
        radar_restore_due = (not self.live_update_handoff_verifier.live(now) and
                             (self.live_update_handoff_radar_restore_last_attempt is None or
                              now - self.live_update_handoff_radar_restore_last_attempt >= RADAR_RESTORE_RETRY_SECONDS))
        if radar_restore_due:
          radar_enabled = self.CI.deinit(self.CP, *self.can_callbacks, retry=2)
          self.live_update_handoff_radar_restore_last_attempt = time.monotonic()
          if radar_enabled:
            self.params.remove("ControlsReady")
          cloudlog.error(f"live update handoff recovery radar communication enable returned {radar_enabled}; reboot remains blocked")
      return True

    active_reasons = self._handoff_controls_active_reasons(CS, FPCS, post_entry=True)
    if active_reasons:
      if state_value in (DIAGNOSTIC_REQUESTED, DIAGNOSTIC) and panda_handoff_mode:
        self.live_update_handoff_verifier = StockSccVerifier()
        radar_enabled = self.CI.deinit(self.CP, *self.can_callbacks)
        now = time.monotonic()
        if radar_enabled:
          self.params.remove("ControlsReady")
          self._set_live_update_handoff_state(timestamped_state(VERIFYING, now))
          cloudlog.warning("live update handoff restored radar while controls settle; stock SCC verification remains paused")
        else:
          self.live_update_handoff_radar_restore_last_attempt = now
          self._set_live_update_handoff_state(timestamped_state(FAILED, now))
          cloudlog.error("live update handoff could not restore radar while controls settled; reboot refused")
          return True
      elif state_value == READY:
        self._set_live_update_handoff_state(timestamped_state(VERIFYING, now))
      if self.live_update_handoff_controls_active_since is None:
        self.live_update_handoff_verifier = StockSccVerifier()
        self.live_update_handoff_started_at = None
        self.live_update_handoff_verify_started_at = None
        self.live_update_handoff_controls_active_since = now
        cloudlog.warning("live update handoff paused after commit while controls settle: " + ", ".join(active_reasons))
      return True
    if self.live_update_handoff_controls_active_since is not None:
      cloudlog.warning("live update handoff controls are off again; restarting stock SCC verification")
    self.live_update_handoff_controls_active_since = None

    if state_value in (DIAGNOSTIC_REQUESTED, DIAGNOSTIC):
      if self.live_update_handoff_started_at is None:
        self.live_update_handoff_started_at = now
      if panda_handoff_mode:
        self.live_update_handoff_verifier = StockSccVerifier()
        radar_enabled = self.CI.deinit(self.CP, *self.can_callbacks)
        now = time.monotonic()
        if radar_enabled:
          self.params.remove("ControlsReady")
        else:
          self.live_update_handoff_radar_restore_last_attempt = now
          self._set_live_update_handoff_state(timestamped_state(FAILED, now))
          cloudlog.error("live update handoff could not restore radar communication; reboot refused")
          return True
        self.live_update_handoff_verify_started_at = now
        self._set_live_update_handoff_state(timestamped_state(VERIFYING, now))
        cloudlog.warning(f"live update handoff radar communication enable returned {radar_enabled}; verifying stock SCC")
        return True
      if now - self.live_update_handoff_started_at > 3.0:
        self._set_live_update_handoff_state(timestamped_state(FAILED, now))
        cloudlog.error("live update handoff timed out waiting for Panda diagnostic mode")
        return True
      return False

    if state_value == VERIFYING:
      if self.live_update_handoff_verify_started_at is None:
        self.live_update_handoff_verify_started_at = now
      if not panda_handoff_mode:
        self._set_live_update_handoff_state(timestamped_state(FAILED, now))
        cloudlog.error("live update handoff lost Panda diagnostic mode during verification")
      else:
        self.live_update_handoff_verifier.update(self.can_list, now)
        if self.live_update_handoff_verifier.ready and self.live_update_handoff_verifier.live(now):
          self._set_live_update_handoff_state(timestamped_state(READY, now))
          cloudlog.warning("live update handoff verified stock SCC takeover")
        elif (self.live_update_handoff_verify_started_at is not None and
              now - self.live_update_handoff_verify_started_at > VERIFY_TIMEOUT_SECONDS):
          self._set_live_update_handoff_state(timestamped_state(FAILED, now))
          cloudlog.error("live update handoff did not verify stock SCC before timeout; refusing reboot")

    elif state_value == READY:
      self.live_update_handoff_verifier.update(self.can_list, now)
      ready_timestamp = state_timestamp(state)
      if not panda_handoff_mode:
        self._set_live_update_handoff_state(timestamped_state(FAILED, now))
        cloudlog.error("live update handoff lost Panda diagnostic mode after verification")
      elif not self.live_update_handoff_verifier.live(now):
        self._set_live_update_handoff_state(timestamped_state(FAILED, now))
        cloudlog.error("live update handoff lost live passive stock SCC after verification; reboot refused")
      elif ready_timestamp is None or now - ready_timestamp >= READY_REFRESH_SECONDS:
        self._set_live_update_handoff_state(timestamped_state(READY, now), nonblocking=True)

    return True

  def step(self):
    CS, RD, FPCS = self.state_update()

    self.state_publish(CS, RD, FPCS)

    initialized = (not any(e.name == EventName.selfdriveInitializing for e in self.sm['onroadEvents']) and
                   self.sm.seen['onroadEvents'])
    controls_quiesced = self.update_live_update_handoff(CS, FPCS, initialized)
    if not self.CP.passive and initialized:
      self.controls_update(CS, self.sm['carControl'], controls_quiesced)

    self.initialized_prev = initialized
    self.CS_prev = CS

    # FrogPilot variables
    self.CI.CS.CC = self.sm['carControl']

    self.frogpilot_toggles = get_frogpilot_toggles(self.sm)

  def params_thread(self, evt):
    while not evt.is_set():
      self.is_metric = self.params.get_bool("IsMetric")
      self.experimental_mode = self.params.get_bool("ExperimentalMode") and self.CP.openpilotLongitudinalControl
      time.sleep(0.1)

  def card_thread(self):
    e = threading.Event()
    t = threading.Thread(target=self.params_thread, args=(e, ))
    try:
      t.start()
      while True:
        self.step()
        self.rk.monitor_time()
    finally:
      e.set()
      t.join()


def main():
  config_realtime_process(4, Priority.CTRL_HIGH)
  car = Car()
  car.card_thread()


if __name__ == "__main__":
  main()
