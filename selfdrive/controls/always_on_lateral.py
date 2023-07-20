# PFEIFER - AOL

from cereal import log, car
from common.params import Params, put_bool_nonblocking
from selfdrive.controls.lib.latcontrol import MIN_LATERAL_CONTROL_SPEED
from panda import ALTERNATIVE_EXPERIENCE
from selfdrive.controls.lib.events import EngagementAlert, AudibleAlert
from enum import IntEnum

State = log.ControlsState.OpenpilotState
ACTIVE_STATES = (State.enabled, State.softDisabling, State.overriding)

# Speeds below this will deactivate lateral controls
STOP_SPEED = 0.5 # m/s

params = Params()
mem_params = Params("/dev/shm/params")
"""
A speed optimization, Params with a base path in /dev/shm/params.

/dev/shm is a memory mapped folder and does not persistently store across
reboots. The advantage to using a memory mapped folder is that it should be
very fast and consistent for both writes and reads.

If using the real filesystem placing data can, in extreme circumstances,
take over 1 second. put_bool_nonblocking helps with this by creating a
thread but this can create race conditions. If we need to block but don't
care about persistence across reboots the memory mapped location should
avoid random lag.
"""

class AlwaysOnLateralType(IntEnum):
  STOCK_OP = 0
  CRUISE_STATE = 1
  BUTTON = 2

class AlwaysOnLateral:
  def __init__(self, CI):
    self.CI = CI

    # UI Toggles
    self.enabled: bool = False
    self.main_enables: bool = False
    self.disengage_lat_on_brake: bool = False
    self.disengage_lat_on_blinker: bool = False

    # Car State
    self.braking: bool = False
    self.blinkers_active: bool = False
    self.standstill: bool = False
    self.steer_fault: bool = False
    self.invalid_gear: bool = False
    self.cruise_available: bool = False
    self.cruise_enabled: bool = False

    # OP/AOL states
    self.cruise_previously_engaged: bool = False
    self.last_lat_allowed: bool = False
    self.op_active: bool = False
    self.aol_type: AlwaysOnLateralType = AlwaysOnLateralType.STOCK_OP
    self.prev_lat_active: bool = False

    params.put_bool("AlwaysOnLateralEnabledLock", True) # locks toggle when onroad



  @staticmethod
  def set_always_on_lateral_type(typ: AlwaysOnLateralType):
    mem_params.put("AlwaysOnLateralType", str(int(typ)))

  @staticmethod
  def get_always_on_lateral_type() -> AlwaysOnLateralType:
    try:
      return AlwaysOnLateralType(int(mem_params.get("AlwaysOnLateralType")))
    except:
      return AlwaysOnLateralType.STOCK_OP

  @staticmethod
  def get_lateral_allowed() -> bool:
    return mem_params.get_bool("LateralAllowed");

  @staticmethod
  def set_lateral_allowed(lateral_allowed: bool) -> None:
    mem_params.put_bool("LateralAllowed", lateral_allowed);

  @staticmethod
  def toggle_lateral_allowed() -> None:
    lateral_allowed = AlwaysOnLateral.get_lateral_allowed()
    AlwaysOnLateral.set_lateral_allowed(not lateral_allowed)

  def handle_audible_alert(self, alert_manager, sm, lateral_allowed: bool):
    if self.last_lat_allowed != lateral_allowed:
      alert = None
      if lateral_allowed:
        alert = EngagementAlert(AudibleAlert.engage)
      else:
        alert = EngagementAlert(AudibleAlert.disengage)
      alert_manager.add_many(sm.frame, [alert])

  def update_cruise_previously_engaged(self):
    if not self.cruise_available:
      self.cruise_previously_engaged = False
    self.cruise_previously_engaged |= self.op_active

  def check_aol_state(self) -> bool:
    if not self.enabled:
      return False

    if self.aol_type == AlwaysOnLateralType.STOCK_OP:
      return self.op_active
    elif self.aol_type == AlwaysOnLateralType.CRUISE_STATE:
      if self.main_enables:
        return self.cruise_available
      return self.cruise_previously_engaged and self.cruise_available
    elif self.aol_type == AlwaysOnLateralType.BUTTON:
      if self.main_enables:
        return self.cruise_available and AlwaysOnLateral.get_lateral_allowed()
      return self.cruise_previously_engaged and self.cruise_available and AlwaysOnLateral.get_lateral_allowed()

    # Should be unreachable
    return False

  def update(self, car_state, op_state, car_params, sm, alert_manager):
    panda_states = sm['pandaStates']
    self.op_active = op_state in ACTIVE_STATES

    self.aol_type = AlwaysOnLateral.get_always_on_lateral_type()
    lateral_allowed = self.check_aol_state()
    self.handle_audible_alert(alert_manager, sm, lateral_allowed)

    self.enabled = params.get_bool('AlwaysOnLateralEnabled')
    self.main_enables = params.get_bool('AlwaysOnLateralMainEnables')
    self.disengage_lat_on_brake = params.get_bool('DisengageLatOnBrake')
    self.disengage_lat_on_blinker = params.get_bool('DisengageLatOnBlinker')

    self.cruise_available = car_state.cruiseState.available
    self.cruise_enabled = car_state.cruiseState.enabled

    self.update_cruise_previously_engaged()


    self.braking = car_state.brakePressed or car_state.regenBraking
    self.blinkers_active = car_state.leftBlinker or car_state.rightBlinker
    self.standstill = car_state.vEgo <= max(car_params.minSteerSpeed, MIN_LATERAL_CONTROL_SPEED) or car_state.standstill
    self.steer_fault = car_state.steerFaultTemporary or car_state.steerFaultPermanent
    self.invalid_gear = car_state.gearShifter not in [car.CarState.GearShifter.drive, car.CarState.GearShifter.sport, car.CarState.GearShifter.low, car.CarState.GearShifter.eco]

    # Always allow lateral when controls are allowed
    if any(ps.controlsAllowed for ps in panda_states) and not lateral_allowed:
      AlwaysOnLateral.set_lateral_allowed(True)

    self.last_lat_allowed = lateral_allowed

  @property
  def _lat_active(self):

    # cruise state must be reporting available to send lateral controls
    if not self.cruise_available:
      return False

    # Require engaging cruise control at least once before keeping lateral on
    # This helps to prevent cruise faults in some cars
    if not self.cruise_previously_engaged and not self.main_enables:
      return False

    # If car is in a gear that does not move forward do not engage lateral
    if self.invalid_gear:
      return False

    # If there is a steer fault lat is not available
    if self.steer_fault:
      return False

    # Disable lateral when vehicle is stopped to prevent "twitching" steering wheel
    if self.standstill:
      return False

    # Track the main button for BUTTON type aol
    if self.enabled and self.aol_type == AlwaysOnLateralType.BUTTON and not self.get_lateral_allowed():
      return False

    # lat is only active when openpilot is active if aol is disabled or we do not support AOL for the car
    if (not self.enabled or self.aol_type == AlwaysOnLateralType.STOCK_OP) and not self.op_active:
      return False

    # If DisengageLatOnBrake is enabled we disable lat when braking
    if self.disengage_lat_on_brake and self.braking:
      return False

    # If DisengageLatOnBlinker is enabled we disable lat when blinkers are on
    if self.disengage_lat_on_blinker and self.blinkers_active:
      return False

    # Lat is active if we pass all previous tests for disengagement
    return True

  @property
  def lat_active(self):
    lat_active = self._lat_active

    # push lat active to the ui
    if self.prev_lat_active != lat_active:
      put_bool_nonblocking("LateralActive", lat_active)

    self.prev_lat_active = lat_active
    return lat_active

  @property
  def alternative_experience(self):
    always_on_lateral_enabled = params.get_bool('AlwaysOnLateralEnabled')
    always_on_lateral_main_enables = params.get_bool('AlwaysOnLateralMainEnables')
    experience = 0
    if always_on_lateral_enabled:
      experience |= ALTERNATIVE_EXPERIENCE.ENABLE_ALWAYS_ON_LATERAL
    if not always_on_lateral_main_enables:
      experience |= ALTERNATIVE_EXPERIENCE.AOL_ENABLE_ON_MAIN
    return experience

