#!/usr/bin/env python3
import json

from openpilot.common.conversions import Conversions as CV
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.controls.controlsd import FrogPilotEventName, State
from openpilot.selfdrive.controls.lib.drive_helpers import V_CRUISE_MAX
from openpilot.selfdrive.ui.soundd import FrogPilotAudibleAlert

from openpilot.frogpilot.common.frogpilot_variables import params

RANDOM_EVENTS = {
  FrogPilotEventName.accel30: "accel30",
  FrogPilotEventName.accel35: "accel35",
  FrogPilotEventName.accel40: "accel40",
  FrogPilotEventName.dejaVuCurve: "dejaVuCurve",
  FrogPilotEventName.firefoxSteerSaturated: "firefoxSteerSaturated",
  FrogPilotEventName.hal9000: "hal9000",
  FrogPilotEventName.openpilotCrashedRandomEvent: "openpilotCrashedRandomEvent",
  FrogPilotEventName.thisIsFineSteerSaturated: "thisIsFineSteerSaturated",
  FrogPilotEventName.toBeContinued: "toBeContinued",
  FrogPilotEventName.vCruise69: "vCruise69",
  FrogPilotEventName.yourFrogTriedToKillMe: "yourFrogTriedToKillMe",
  FrogPilotEventName.youveGotMail: "youveGotMail",
}

class FrogPilotTracking:
  def __init__(self, frogpilot_planner):
    self.frogpilot_events = frogpilot_planner.frogpilot_events

    self.frogpilot_stats = json.loads(params.get("FrogPilotStats") or "{}")

    self.total_drives = params.get_int("FrogPilotDrives")
    self.total_kilometers = params.get_float("FrogPilotKilometers")
    self.total_minutes = params.get_float("FrogPilotMinutes")

    self.drive_added = False
    self.enabled = False

    self.distance_since_override = 0
    self.tracked_time = 0

    self.previous_random_events = set()

    self.sound = FrogPilotAudibleAlert.none
    self.state = State.disabled

  def update(self, sm, frogpilot_toggles):
    v_cruise = min(sm["controlsState"].vCruise, V_CRUISE_MAX) * CV.KPH_TO_MS
    v_ego = max(sm["carState"].vEgo, 0)

    self.enabled |= sm["controlsState"].enabled or sm["frogpilotCarState"].alwaysOnLateralEnabled

    self.total_kilometers += (v_ego * DT_MDL) / 1000
    self.tracked_time += DT_MDL

    self.frogpilot_stats["HighestAcceleration"] = max(self.frogpilot_events.max_acceleration, self.frogpilot_stats.get("HighestAcceleration", 0))

    if sm["frogpilotControlsState"].alertSound != self.sound:
      if sm["frogpilotControlsState"].alertSound == FrogPilotAudibleAlert.goat:
        self.frogpilot_stats["TotalGoatScreams"] = self.frogpilot_stats.get("TotalGoatScreams", 0) + 1

      self.sound = sm["frogpilotControlsState"].alertSound

    if sm["controlsState"].enabled:
      key = str(round(v_cruise, 2))
      total_cruise_speed_times = self.frogpilot_stats.get("TotalCruiseSpeedTimes", {})
      total_cruise_speed_times[key] = total_cruise_speed_times.get(key, 0) + DT_MDL
      self.frogpilot_stats["TotalCruiseSpeedTimes"] = total_cruise_speed_times

    if sm["carControl"].latActive:
      self.frogpilot_stats["TotalLateralTime"] = self.frogpilot_stats.get("TotalLateralTime", 0) + DT_MDL
    if sm["carControl"].longActive:
      self.frogpilot_stats["TotalLongitudinalTime"] = self.frogpilot_stats.get("TotalLongitudinalTime", 0) + DT_MDL
    elif sm["frogpilotCarState"].alwaysOnLateralEnabled:
      self.frogpilot_stats["TotalAOLTime"] = self.frogpilot_stats.get("TotalAOLTime", 0) + DT_MDL

    if sm["carState"].standstill:
      self.frogpilot_stats["TotalStandstillTime"] = self.frogpilot_stats.get("TotalStandstillTime", 0) + DT_MDL
      if self.frogpilot_events.stopped_for_light:
        self.frogpilot_stats["TotalStopLightTime"] = self.frogpilot_stats.get("TotalStopLightTime", 0) + DT_MDL

    if sm["controlsState"].state not in (State.disabled, State.overriding):
      self.distance_since_override += v_ego * DT_MDL
      self.longest_distance_no_override = max(self.distance_since_override, self.frogpilot_stats.get("LongestDistanceNoOverride", 0))

    if sm["controlsState"].state != self.state:
      if sm["controlsState"].state == State.disabled:
        self.distance_since_override = 0
        self.frogpilot_stats["TotalDisengages"] = self.frogpilot_stats.get("TotalDisengages", 0) + 1

        if frogpilot_toggles.sound_pack == "frog":
          self.frogpilot_stats["TotalFrogSqueaks"] = self.frogpilot_stats.get("TotalFrogSqueaks", 0) + 1
      elif sm["controlsState"].state == State.enabled:
        self.frogpilot_stats["TotalEngages"] = self.frogpilot_stats.get("TotalEngages", 0) + 1

        if frogpilot_toggles.sound_pack == "frog":
          self.frogpilot_stats["TotalFrogChirps"] = self.frogpilot_stats.get("TotalFrogChirps", 0) + 1
      elif sm["controlsState"].state == State.overriding:
        self.distance_since_override = 0
        self.frogpilot_stats["TotalOverrides"] = self.frogpilot_stats.get("TotalOverrides", 0) + 1

      self.state = sm["controlsState"].state

    if len(self.frogpilot_events.events) > 0:
      current_random_events = {event for event in self.frogpilot_events.events.names if event in RANDOM_EVENTS}
      new_events = current_random_events - self.previous_random_events

      if new_events:
        total_random_events = self.frogpilot_stats.get("TotalRandomEvents", {})
        for event in new_events:
          event_name = RANDOM_EVENTS[event]
          total_random_events[event_name] = total_random_events.get(event_name, 0) + 1
        self.frogpilot_stats["TotalRandomEvents"] = total_random_events

      self.previous_random_events = current_random_events

    if self.tracked_time > 60 and sm["carState"].standstill and self.enabled:
      params.put_float_nonblocking("FrogPilotKilometers", self.total_kilometers)

      self.total_minutes += self.tracked_time / 60
      params.put_float_nonblocking("FrogPilotMinutes", self.total_minutes)

      self.frogpilot_stats["TotalTrackedTime"] = self.frogpilot_stats.get("TotalTrackedTime", 0) + self.tracked_time

      current_model = frogpilot_toggles.model
      total_model_times = self.frogpilot_stats.get("TotalModelTimes", {})
      total_model_times[current_model] = total_model_times.get(current_model, 0) + self.tracked_time
      self.frogpilot_stats["TotalModelTimes"] = total_model_times

      params.put_nonblocking("FrogPilotStats", json.dumps(self.frogpilot_stats))

      self.tracked_time = 0

      if not self.drive_added:
        params.put_int_nonblocking("FrogPilotDrives", self.total_drives + 1)
        self.drive_added = True
