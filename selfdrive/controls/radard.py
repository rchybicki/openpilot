#!/usr/bin/env python3
import math
import numpy as np
from collections import deque
from types import SimpleNamespace
from typing import Any

import capnp
from cereal import messaging, log, car, custom
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL, Priority, config_realtime_process
from openpilot.common.swaglog import cloudlog
from openpilot.common.simple_kalman import KF1D
from openpilot.selfdrive.controls.lib.desire_helper import LaneChangeDirection, LaneChangeState
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.stop_target_helpers import get_published_lead_distance

from openpilot.frogpilot.common.frogpilot_variables import get_frogpilot_toggles


# Default lead acceleration decay set to 50% at 1s
_LEAD_ACCEL_TAU = 1.5

# radar tracks
SPEED, ACCEL = 0, 1     # Kalman filter states enum

# stationary qualification parameters
V_EGO_STATIONARY = 4.   # no stationary object flag below this speed

RADAR_TO_CENTER = 2.7   # (deprecated) RADAR is ~ 2.7m ahead from center of car
RADAR_TO_CAMERA = 1.52  # RADAR is ~ 1.5m ahead from center of mesh frame

SURROGATE_DREL_OFFSET = 40.0
SURROGATE_VLEAD_DELTA = 5.0
SURROGATE_YREL_EXEMPT = 0.3
SURROGATE_MIN_TARGET_LANE_WIDTH = 2.0
SURROGATE_TARGET_RELEASE_MAX_AGE = 3.0
SURROGATE_TARGET_RELEASE_DREL_TOL = 10.0
SURROGATE_TARGET_RELEASE_VLEAD_TOL = 4.0
SURROGATE_TARGET_DIVIDER_MARGIN = 0.2
SURROGATE_TARGET_DIVIDER_FRAMES = 3

DIVIDER_X_REF = 6.0
DIVIDER_MIN_PROB = 0.3
DIVIDER_CROSS_Y_HYST = 0.25
DIVIDER_CROSS_FRAMES = 3
SURROGATE_ACTIVE_STATES = (
  LaneChangeState.preLaneChange,
  LaneChangeState.laneChangeStarting,
)

SURROGATE_PHASE_OFF = 0
SURROGATE_PHASE_PREP = 1
SURROGATE_PHASE_EXEC = 2
SURROGATE_PHASE_DONE = 3


class KalmanParams:
  def __init__(self, dt: float):
    # Lead Kalman Filter params, calculating K from A, C, Q, R requires the control library.
    # hardcoding a lookup table to compute K for values of radar_ts between 0.01s and 0.2s
    assert dt > .01 and dt < .2, "Radar time step must be between .01s and 0.2s"
    self.A = [[1.0, dt], [0.0, 1.0]]
    self.C = [1.0, 0.0]
    #Q = np.matrix([[10., 0.0], [0.0, 100.]])
    #R = 1e3
    #K = np.matrix([[ 0.05705578], [ 0.03073241]])
    dts = [i * 0.01 for i in range(1, 21)]
    K0 = [0.12287673, 0.14556536, 0.16522756, 0.18281627, 0.1988689,  0.21372394,
          0.22761098, 0.24069424, 0.253096,   0.26491023, 0.27621103, 0.28705801,
          0.29750003, 0.30757767, 0.31732515, 0.32677158, 0.33594201, 0.34485814,
          0.35353899, 0.36200124]
    K1 = [0.29666309, 0.29330885, 0.29042818, 0.28787125, 0.28555364, 0.28342219,
          0.28144091, 0.27958406, 0.27783249, 0.27617149, 0.27458948, 0.27307714,
          0.27162685, 0.27023228, 0.26888809, 0.26758976, 0.26633338, 0.26511557,
          0.26393339, 0.26278425]
    self.K = [[np.interp(dt, dts, K0)], [np.interp(dt, dts, K1)]]


class Track:
  def __init__(self, identifier: int, v_lead: float, kalman_params: KalmanParams):
    self.identifier = identifier
    self.cnt = 0
    self.aLeadTau = FirstOrderFilter(_LEAD_ACCEL_TAU, 0.45, DT_MDL)
    self.K_A = kalman_params.A
    self.K_C = kalman_params.C
    self.K_K = kalman_params.K
    self.kf = KF1D([[v_lead], [0.0]], self.K_A, self.K_C, self.K_K)

    # FrogPilot variables
    self.leadLeft = False
    self.leadRight = False

    self.leadTrackID = 0

  def update(self, d_rel: float, y_rel: float, v_rel: float, v_lead: float, measured: float):
    # relative values, copy
    self.dRel = d_rel   # LONG_DIST
    self.yRel = y_rel   # -LAT_DIST
    self.vRel = v_rel   # REL_SPEED
    self.vLead = v_lead
    self.measured = measured   # measured or estimate

    # computed velocity and accelerations
    if self.cnt > 0:
      self.kf.update(self.vLead)

    self.vLeadK = float(self.kf.x[SPEED][0])
    self.aLeadK = float(self.kf.x[ACCEL][0])

    # Learn if constant acceleration
    if abs(self.aLeadK) < 0.5:
      self.aLeadTau.x = _LEAD_ACCEL_TAU
    else:
      self.aLeadTau.update(0.0)

    self.cnt += 1

  def get_RadarState(self, model_prob: float = 0.0):
    return {
      "dRel": float(self.dRel),
      "yRel": float(self.yRel),
      "vRel": float(self.vRel),
      "vLead": float(self.vLead),
      "vLeadK": float(self.vLeadK),
      "aLeadK": float(self.aLeadK),
      "aLeadTau": float(self.aLeadTau.x),
      "status": True,
      "fcw": self.is_potential_fcw(model_prob),
      "modelProb": model_prob,
      "radar": True,
      "radarTrackId": self.identifier,
    }

  def potential_low_speed_lead(self, v_ego: float):
    # stop for stuff in front of you and low speed, even without model confirmation
    # Radar points closer than 0.75, are almost always glitches on toyota radars
    return abs(self.yRel) < 1.0 and (v_ego < V_EGO_STATIONARY) and (0.75 < self.dRel < 25)

  def is_potential_fcw(self, model_prob: float):
    return model_prob > .9

  def __str__(self):
    ret = f"x: {self.dRel:4.1f}  y: {self.yRel:4.1f}  v: {self.vRel:4.1f}  a: {self.aLeadK:4.1f}"
    return ret

  # FrogPilot variables
  def potential_adjacent_lead(self, left: bool, model_data: capnp._DynamicStructReader):
    if self.vLeadK < 1 or self.leadTrackID == self.identifier:
      return False

    far_left_lane = np.interp(self.dRel, model_data.laneLines[0].x, model_data.laneLines[0].y)
    left_lane = np.interp(self.dRel, model_data.laneLines[1].x, model_data.laneLines[1].y)
    right_lane = np.interp(self.dRel, model_data.laneLines[2].x, model_data.laneLines[2].y)
    far_right_lane = np.interp(self.dRel, model_data.laneLines[3].x, model_data.laneLines[3].y)

    self.leadLeft = far_left_lane < -self.yRel < left_lane and self.dRel < model_data.position.x[-1]
    self.leadRight = right_lane < -self.yRel < far_right_lane and self.dRel < model_data.position.x[-1]

    if left:
      return self.leadLeft
    else:
      return self.leadRight

def laplacian_pdf(x: float, mu: float, b: float):
  b = max(b, 1e-4)
  return math.exp(-abs(x-mu)/b)


def match_vision_to_track(v_ego: float, lead: capnp._DynamicStructReader, model_data: capnp._DynamicStructReader, tracks: dict[int, Track], frogpilot_toggles: SimpleNamespace):
  offset_vision_dist = lead.x[0] - RADAR_TO_CAMERA

  def track_is_sane(track: Track):
    dist_sane = abs(track.dRel - offset_vision_dist) < max([(offset_vision_dist) * .25, 5.0])
    vel_sane = (abs(track.vRel + v_ego - lead.v[0]) < 10) or (v_ego + track.vRel > 3)
    return dist_sane and vel_sane

  def prob(c):
    prob_d = laplacian_pdf(c.dRel, offset_vision_dist, lead.xStd[0])
    prob_y = laplacian_pdf(c.yRel, -lead.y[0], lead.yStd[0])
    prob_v = laplacian_pdf(c.vRel + v_ego, lead.v[0], lead.vStd[0])

    # This isn't exactly right, but it's a good heuristic
    return prob_d * prob_y * prob_v

  track = max(tracks.values(), key=prob)

  # if no 'sane' match is found return -1
  # stationary radar points can be false positives
  if track_is_sane(track):
    return track
  else:
    return None


def get_RadarState_from_vision(lead_msg: capnp._DynamicStructReader, v_ego: float, model_v_ego: float):
  lead_v_rel_pred = lead_msg.v[0] - model_v_ego
  return {
    "dRel": float(lead_msg.x[0] - RADAR_TO_CAMERA),
    "yRel": float(-lead_msg.y[0]),
    "vRel": float(lead_v_rel_pred),
    "vLead": float(v_ego + lead_v_rel_pred),
    "vLeadK": float(v_ego + lead_v_rel_pred),
    "aLeadK": float(lead_msg.a[0]),
    "aLeadTau": 0.3,
    "fcw": False,
    "modelProb": float(lead_msg.prob),
    "status": True,
    "radar": False,
    "radarTrackId": -1,
  }


def get_lead(v_ego: float, ready: bool, tracks: dict[int, Track], lead_msg: capnp._DynamicStructReader,
             model_v_ego: float, model_data: capnp._DynamicStructReader,
             frogpilot_plan: capnp._DynamicStructReader, frogpilot_toggles: SimpleNamespace,
             low_speed_override: bool = True) -> dict[str, Any]:
  # Determine leads, this is where the essential logic happens
  if len(tracks) > 0 and ready and lead_msg.prob > frogpilot_toggles.lead_detection_probability:
    track = match_vision_to_track(v_ego, lead_msg, model_data, tracks, frogpilot_toggles)
  else:
    track = None

  lead_dict = {'status': False}
  if track is not None:
    lead_dict = track.get_RadarState(lead_msg.prob)
  elif (track is None) and ready and (lead_msg.prob > frogpilot_toggles.lead_detection_probability):
    lead_dict = get_RadarState_from_vision(lead_msg, v_ego, model_v_ego)

  if low_speed_override:
    low_speed_tracks = [c for c in tracks.values() if c.potential_low_speed_lead(v_ego)]
    if len(low_speed_tracks) > 0:
      closest_track = min(low_speed_tracks, key=lambda c: c.dRel)

      # Only choose new track if it is actually closer than the previous one
      if (not lead_dict['status']) or (closest_track.dRel < lead_dict['dRel']):
        lead_dict = closest_track.get_RadarState()

  # FrogPilot variables
  for track in tracks.values():
    track.leadTrackID = lead_dict.get('radarTrackId', -1)

  if 'dRel' in lead_dict:
    # §4.2.1: mutation active only while stopping_flags.PUBLISH_TRUE_LEAD_DISTANCE is False
    lead_dict['dRel'] = get_published_lead_distance(lead_dict['dRel'], frogpilot_plan.increasedStoppedDistance)

  return lead_dict


# FrogPilot variables
def get_adjacent_lead(tracks: dict[int, Track], model_data: capnp._DynamicStructReader, left: bool = True) -> dict[str, Any]:
  lead_dict = {'status': False}

  adjacent_tracks = [c for c in tracks.values() if c.potential_adjacent_lead(left, model_data)]
  if len(adjacent_tracks) > 0:
    closest_track = min(adjacent_tracks, key=lambda c: c.dRel)
    lead_dict = closest_track.get_RadarState()

  return lead_dict


class RadarD:
  def __init__(self, delay: float = 0.0):
    self.current_time = 0.0

    self.tracks: dict[int, Track] = {}
    self.kalman_params = KalmanParams(DT_MDL)

    self.v_ego = 0.0
    self.v_ego_hist = deque([0.0], maxlen=int(round(delay / DT_MDL))+1)
    self.last_v_ego_frame = -1

    self.radar_state: capnp._DynamicStructBuilder | None = None
    self.radar_state_valid = False

    self.ready = False

    # FrogPilot variables
    self.frogpilot_radar_state = custom.FrogPilotRadarState.new_message()

    self.frogpilot_toggles = get_frogpilot_toggles()

    self.surrogate_track_ids: set[int] = set()
    self.target_lane_released_track_ids: set[int] = set()
    self.target_lane_released_leads: deque[tuple[float, float, float]] = deque(maxlen=8)
    self.target_lane_crossing_counts: dict[int, int] = {}
    self.main_untracked_active = False
    self.main_untracked_sign = 0
    self.surrogate_untracked_side_signs: set[int] = set()
    self.prev_lane_change_state = LaneChangeState.off
    self.lc_direction_sign = 0
    self.center_surrogate_enabled = False

    self.divider_lane_line_idx = -1
    self.divider_initial_sign = 0
    self.divider_crossed_counter = 0
    self.divider_crossed = False
    self.surrogate_phase = SURROGATE_PHASE_OFF

  def _reset_lane_change_surrogates(self):
    self.surrogate_track_ids.clear()
    self.target_lane_released_track_ids.clear()
    self.target_lane_released_leads.clear()
    self.target_lane_crossing_counts.clear()
    self.main_untracked_active = False
    self.main_untracked_sign = 0
    self.surrogate_untracked_side_signs.clear()
    self.lc_direction_sign = 0
    self.center_surrogate_enabled = False

    self.divider_lane_line_idx = -1
    self.divider_initial_sign = 0
    self.divider_crossed_counter = 0
    self.divider_crossed = False
    self.surrogate_phase = SURROGATE_PHASE_OFF

  @staticmethod
  def _sign(x: float) -> int:
    return 1 if x > 0.0 else (-1 if x < 0.0 else 0)

  def _target_lane_exists_for_surrogate(self, sm: messaging.SubMaster) -> bool:
    if not sm.seen.get('frogpilotPlan', False):
      return True

    direction = sm['modelV2'].meta.laneChangeDirection
    if direction == LaneChangeDirection.left:
      lane_width = float(sm['frogpilotPlan'].laneWidthLeft)
    elif direction == LaneChangeDirection.right:
      lane_width = float(sm['frogpilotPlan'].laneWidthRight)
    else:
      return True

    min_lane_width = SURROGATE_MIN_TARGET_LANE_WIDTH
    lane_detection_width = float(getattr(self.frogpilot_toggles, "lane_detection_width", 0.0))
    if lane_detection_width > 0.0:
      min_lane_width = max(min_lane_width, lane_detection_width)

    return lane_width >= min_lane_width

  def _update_divider_crossing(self, sm: messaging.SubMaster, initialize: bool = False):
    if self.divider_crossed or self.divider_lane_line_idx < 0:
      return

    model_v2 = sm['modelV2']
    if len(model_v2.laneLines) <= self.divider_lane_line_idx or len(model_v2.laneLineProbs) <= self.divider_lane_line_idx:
      return

    divider_prob = float(model_v2.laneLineProbs[self.divider_lane_line_idx])
    if divider_prob < DIVIDER_MIN_PROB:
      return

    divider_line = model_v2.laneLines[self.divider_lane_line_idx]
    if len(divider_line.x) == 0 or len(divider_line.y) == 0:
      return

    divider_y = float(np.interp(DIVIDER_X_REF, divider_line.x, divider_line.y))

    if initialize:
      self.divider_initial_sign = 0
      self.divider_crossed_counter = 0

    if self.divider_initial_sign == 0:
      if abs(divider_y) > DIVIDER_CROSS_Y_HYST:
        self.divider_initial_sign = self._sign(divider_y)
      return

    crossed_candidate = (divider_y * self.divider_initial_sign) < -DIVIDER_CROSS_Y_HYST
    if crossed_candidate:
      self.divider_crossed_counter += 1
    else:
      self.divider_crossed_counter = 0

    if self.divider_crossed_counter >= DIVIDER_CROSS_FRAMES:
      self.divider_crossed = True

  def _release_lane_change_surrogate(self, track_id: int, side_sign: int):
    if track_id >= 0:
      self.surrogate_track_ids.discard(track_id)
      return

    if self.main_untracked_active:
      if self.main_untracked_sign == 0:
        self.center_surrogate_enabled = False
        self.main_untracked_active = False
        self.main_untracked_sign = 0
      elif self.main_untracked_sign == side_sign:
        self.main_untracked_active = False
        self.main_untracked_sign = 0
    self.surrogate_untracked_side_signs.discard(side_sign)

  def _update_lane_change_surrogates(self, sm: messaging.SubMaster, lead_main: dict[str, Any], lead_aux: dict[str, Any] | None = None):
    lane_change_state = sm['modelV2'].meta.laneChangeState

    if not (self.frogpilot_toggles.human_lane_changes and self.ready):
      self._reset_lane_change_surrogates()
      self.prev_lane_change_state = lane_change_state
      return

    if lane_change_state in SURROGATE_ACTIVE_STATES:
      newly_active = self.prev_lane_change_state not in SURROGATE_ACTIVE_STATES
      if newly_active:
        self._reset_lane_change_surrogates()
        direction = sm['modelV2'].meta.laneChangeDirection
        self.lc_direction_sign = 1 if direction == LaneChangeDirection.left else (-1 if direction == LaneChangeDirection.right else 0)
        self.divider_lane_line_idx = 1 if direction == LaneChangeDirection.left else (2 if direction == LaneChangeDirection.right else -1)
        self.surrogate_phase = SURROGATE_PHASE_PREP

      if lane_change_state == LaneChangeState.preLaneChange and self.surrogate_phase != SURROGATE_PHASE_DONE:
        self.surrogate_phase = SURROGATE_PHASE_PREP
      elif lane_change_state == LaneChangeState.laneChangeStarting and self.surrogate_phase in (SURROGATE_PHASE_OFF, SURROGATE_PHASE_PREP):
        self.surrogate_phase = SURROGATE_PHASE_EXEC

      self._update_divider_crossing(sm, initialize=newly_active)
      if self.divider_crossed:
        self.surrogate_phase = SURROGATE_PHASE_DONE
        self.prev_lane_change_state = lane_change_state
        return

      allow_registration = self.surrogate_phase == SURROGATE_PHASE_PREP
      allow_center_registration = lane_change_state == LaneChangeState.preLaneChange and allow_registration
      opposite_side_sign = -self.lc_direction_sign if self.lc_direction_sign != 0 else None
      divider_checked_track_ids: set[int] = set()

      for lead in (lead_main, lead_aux):
        if lead is None or not lead.get('status', False):
          continue

        track_id = lead.get('radarTrackId', -1)
        side_sign = self._lead_side_sign(lead)

        registered_surrogate = self._lead_is_registered_surrogate(lead)
        crossed_target_divider = False
        if registered_surrogate and (track_id < 0 or track_id not in divider_checked_track_ids):
          crossed_target_divider = self._registered_surrogate_crossed_target_divider(lead, sm)
          if track_id >= 0:
            divider_checked_track_ids.add(track_id)

        reached_target_lane = self._registered_surrogate_reached_target_lane(lead) or crossed_target_divider
        if crossed_target_divider or (
          self._lead_exempt_from_surrogate(lead) and (not registered_surrogate or reached_target_lane)
        ):
          if reached_target_lane:
            self._mark_target_lane_released_lead(lead)
          self._release_lane_change_surrogate(track_id, side_sign)
          continue

        if not allow_registration:
          continue

        if allow_center_registration and side_sign == 0:
          if track_id >= 0:
            self.surrogate_track_ids.add(track_id)
          else:
            if not self.main_untracked_active:
              self.main_untracked_active = True
              self.main_untracked_sign = 0
            elif self.main_untracked_sign != 0:
              self.surrogate_untracked_side_signs.add(0)
          self.center_surrogate_enabled = True
          continue

        if opposite_side_sign is not None and side_sign == opposite_side_sign:
          if track_id >= 0:
            self.surrogate_track_ids.add(track_id)
          else:
            if not self.main_untracked_active:
              self.main_untracked_active = True
              self.main_untracked_sign = side_sign
            elif self.main_untracked_sign != side_sign:
              self.surrogate_untracked_side_signs.add(side_sign)

      for track_id in tuple(self.target_lane_crossing_counts):
        if track_id not in divider_checked_track_ids:
          self.target_lane_crossing_counts.pop(track_id, None)

      if newly_active and allow_registration and opposite_side_sign is not None:
        self.surrogate_untracked_side_signs.add(opposite_side_sign)
    else:
      self._reset_lane_change_surrogates()

    self.prev_lane_change_state = lane_change_state

  def _lead_exempt_from_surrogate(self, lead: dict[str, Any]) -> bool:
    direction_sign = self.lc_direction_sign
    if direction_sign == 0:
      return False
    y_rel = lead.get('yRel', 0.0)
    if direction_sign == 1 and y_rel > SURROGATE_YREL_EXEMPT:
      return True
    if direction_sign == -1 and y_rel < -SURROGATE_YREL_EXEMPT:
      return True
    return False

  @staticmethod
  def _lead_side_sign(lead: dict[str, Any]) -> int:
    y_rel = lead.get('yRel', 0.0)
    if y_rel > 0.5:
      return 1
    if y_rel < -0.5:
      return -1
    return 0

  def _lead_is_registered_surrogate(self, lead: dict[str, Any]) -> bool:
    track_id = lead.get('radarTrackId', -1)
    side_sign = self._lead_side_sign(lead)
    if track_id >= 0:
      return track_id in self.surrogate_track_ids

    if self.main_untracked_active and (
      (self.main_untracked_sign == 0 and side_sign == 0 and self.center_surrogate_enabled) or
      (self.main_untracked_sign != 0 and side_sign == self.main_untracked_sign)
    ):
      return True
    return side_sign in self.surrogate_untracked_side_signs

  def _registered_surrogate_reached_target_lane(self, lead: dict[str, Any]) -> bool:
    return self._lead_side_sign(lead) == self.lc_direction_sign

  def _lead_target_divider_margin(self, lead: dict[str, Any], sm: messaging.SubMaster) -> float | None:
    if self.lc_direction_sign == 0 or self.divider_lane_line_idx < 0 or 'dRel' not in lead or 'yRel' not in lead:
      return None

    model_v2 = sm['modelV2']
    if len(model_v2.laneLines) <= self.divider_lane_line_idx or len(model_v2.laneLineProbs) <= self.divider_lane_line_idx:
      return None
    if float(model_v2.laneLineProbs[self.divider_lane_line_idx]) < DIVIDER_MIN_PROB:
      return None

    divider_line = model_v2.laneLines[self.divider_lane_line_idx]
    if len(divider_line.x) == 0 or len(divider_line.y) == 0:
      return None

    d_rel = float(lead['dRel'])
    if not math.isfinite(d_rel) or d_rel < float(divider_line.x[0]) or d_rel > float(divider_line.x[-1]):
      return None

    divider_y = float(np.interp(d_rel, divider_line.x, divider_line.y))
    lead_model_y = -float(lead['yRel'])
    return -self.lc_direction_sign * (lead_model_y - divider_y)

  def _registered_surrogate_crossed_target_divider(self, lead: dict[str, Any], sm: messaging.SubMaster) -> bool:
    track_id = lead.get('radarTrackId', -1)
    target_margin = self._lead_target_divider_margin(lead, sm)
    if target_margin is None or target_margin < SURROGATE_TARGET_DIVIDER_MARGIN:
      if track_id >= 0:
        self.target_lane_crossing_counts.pop(track_id, None)
      return False

    if track_id < 0:
      return True

    crossing_count = self.target_lane_crossing_counts.get(track_id, 0) + 1
    self.target_lane_crossing_counts[track_id] = crossing_count
    return crossing_count >= SURROGATE_TARGET_DIVIDER_FRAMES

  def _mark_target_lane_released_lead(self, lead: dict[str, Any]):
    track_id = lead.get('radarTrackId', -1)
    if track_id >= 0:
      self.target_lane_released_track_ids.add(track_id)
      self.target_lane_crossing_counts.pop(track_id, None)

    if 'dRel' in lead and 'vLead' in lead:
      self.target_lane_released_leads.append((self.current_time, lead['dRel'], lead['vLead']))

  def _lead_matches_target_lane_release(self, lead: dict[str, Any]) -> bool:
    track_id = lead.get('radarTrackId', -1)
    if track_id >= 0 and track_id in self.target_lane_released_track_ids:
      return True

    if 'dRel' not in lead or 'vLead' not in lead:
      return False

    for release_time, release_d_rel, release_v_lead in reversed(self.target_lane_released_leads):
      if self.current_time - release_time > SURROGATE_TARGET_RELEASE_MAX_AGE:
        continue
      if (
        abs(lead['dRel'] - release_d_rel) <= SURROGATE_TARGET_RELEASE_DREL_TOL and
        abs(lead['vLead'] - release_v_lead) <= SURROGATE_TARGET_RELEASE_VLEAD_TOL
      ):
        return True
    return False

  def _apply_overtake_surrogate(self, lead: dict[str, Any], sm: messaging.SubMaster, force: bool = False) -> tuple[dict[str, Any], bool]:
    if not lead.get('status', False):
      return lead, False

    if not self.frogpilot_toggles.human_lane_changes or self.divider_crossed:
      return lead, False

    lane_change_state = sm['modelV2'].meta.laneChangeState
    if lane_change_state not in SURROGATE_ACTIVE_STATES and not force:
      return lead, False

    if self.surrogate_phase in (SURROGATE_PHASE_OFF, SURROGATE_PHASE_DONE) and not force:
      return lead, False

    lane_change_direction = sm['modelV2'].meta.laneChangeDirection
    if not force and lane_change_state == LaneChangeState.preLaneChange and not self._target_lane_exists_for_surrogate(sm):
      return lead, False

    registered_surrogate = self._lead_is_registered_surrogate(lead)
    reached_target_lane = self._registered_surrogate_reached_target_lane(lead)
    if self._lead_exempt_from_surrogate(lead) and (
      not registered_surrogate or reached_target_lane
    ):
      track_id = lead.get('radarTrackId', -1)
      side_sign = self._lead_side_sign(lead)
      if reached_target_lane:
        self._mark_target_lane_released_lead(lead)
      self._release_lane_change_surrogate(track_id, side_sign)
      return lead, False

    lead_track_id = lead.get('radarTrackId', -1)
    side_sign = self._lead_side_sign(lead)
    target_lane_released_track = self._lead_matches_target_lane_release(lead)

    apply_surrogate = False

    if force and self.surrogate_phase == SURROGATE_PHASE_PREP:
      apply_surrogate = True
    elif registered_surrogate:
      apply_surrogate = True
    elif lead_track_id < 0:
      if self.main_untracked_active and (
        (self.main_untracked_sign == 0 and side_sign == 0 and self.center_surrogate_enabled) or
        (self.main_untracked_sign != 0 and side_sign == self.main_untracked_sign)
      ):
        apply_surrogate = True
      elif side_sign in self.surrogate_untracked_side_signs:
        apply_surrogate = True
    else:
      if self.surrogate_phase == SURROGATE_PHASE_PREP and lane_change_direction == LaneChangeDirection.left and side_sign == -1:
        apply_surrogate = True
      elif self.surrogate_phase == SURROGATE_PHASE_PREP and lane_change_direction == LaneChangeDirection.right and side_sign == 1:
        apply_surrogate = True

    if (
      not apply_surrogate and lane_change_state == LaneChangeState.laneChangeStarting and
      self.surrogate_phase == SURROGATE_PHASE_EXEC and side_sign != self.lc_direction_sign and
      not target_lane_released_track
    ):
      # Radar track IDs can churn after the lane change starts. Keep suppressing eligible
      # pre-divider leads instead of falling back to the close lead mid-maneuver.
      apply_surrogate = True

    if not apply_surrogate:
      return lead, False

    if lead_track_id >= 0:
      self.surrogate_track_ids.add(lead_track_id)
    else:
      if not self.main_untracked_active:
        self.main_untracked_active = True
        self.main_untracked_sign = side_sign
        if side_sign == 0:
          self.center_surrogate_enabled = True
      elif self.main_untracked_sign == 0 and side_sign != 0:
        self.surrogate_untracked_side_signs.add(side_sign)
      elif self.main_untracked_sign != 0 and side_sign not in (0, self.main_untracked_sign):
        self.surrogate_untracked_side_signs.add(side_sign)

    v_lead = lead.get('vLead', self.v_ego)

    new_lead = lead.copy()
    surrogate_v_lead = max(v_lead, self.v_ego + SURROGATE_VLEAD_DELTA)
    new_lead['vLead'] = surrogate_v_lead
    new_lead['vLeadK'] = max(new_lead.get('vLeadK', surrogate_v_lead), surrogate_v_lead)
    new_lead['vRel'] = surrogate_v_lead - self.v_ego
    new_lead['dRel'] = lead.get('dRel', 0.0) + SURROGATE_DREL_OFFSET
    new_lead['fcw'] = False
    new_lead['modelProb'] = max(new_lead.get('modelProb', 0.0), 0.01)

    return new_lead, True

  def update(self, sm: messaging.SubMaster, rr: car.RadarData):
    self.ready = sm.seen['modelV2']
    self.current_time = 1e-9*max(sm.logMonoTime.values())

    if sm.recv_frame['carState'] != self.last_v_ego_frame:
      self.v_ego = sm['carState'].vEgo
      self.v_ego_hist.append(self.v_ego)
      self.last_v_ego_frame = sm.recv_frame['carState']

    ar_pts = {pt.trackId: [pt.dRel, pt.yRel, pt.vRel, pt.measured] for pt in rr.points}

    # *** remove missing points from meta data ***
    for ids in list(self.tracks.keys()):
      if ids not in ar_pts:
        self.tracks.pop(ids, None)

    # *** compute the tracks ***
    for ids in ar_pts:
      rpt = ar_pts[ids]

      # align v_ego by a fixed time to align it with the radar measurement
      v_lead = rpt[2] + self.v_ego_hist[0]

      # create the track if it doesn't exist or it's a new track
      if ids not in self.tracks:
        self.tracks[ids] = Track(ids, v_lead, self.kalman_params)
      self.tracks[ids].update(rpt[0], rpt[1], rpt[2], v_lead, rpt[3])

    # *** publish radarState ***
    self.radar_state_valid = sm.all_checks()
    self.radar_state = log.RadarState.new_message()
    self.radar_state.mdMonoTime = sm.logMonoTime['modelV2']
    self.radar_state.radarErrors = rr.errors
    self.radar_state.carStateMonoTime = sm.logMonoTime['carState']

    if len(sm['modelV2'].velocity.x):
      model_v_ego = sm['modelV2'].velocity.x[0]
    else:
      model_v_ego = self.v_ego
    leads_v3 = sm['modelV2'].leadsV3
    if len(leads_v3) > 1:
      lead_one = get_lead(self.v_ego, self.ready, self.tracks, leads_v3[0], model_v_ego, sm['modelV2'], sm['frogpilotPlan'], self.frogpilot_toggles, low_speed_override=True)
      lead_two = get_lead(self.v_ego, self.ready, self.tracks, leads_v3[1], model_v_ego, sm['modelV2'], sm['frogpilotPlan'], self.frogpilot_toggles, low_speed_override=False)
      self._update_lane_change_surrogates(sm, lead_one, lead_two)

      lead_one, surrogate_applied = self._apply_overtake_surrogate(lead_one, sm)
      self.radar_state.leadOne = lead_one

      if lead_two.get('status', False):
        same_side = self._lead_side_sign(lead_two) == self._lead_side_sign(lead_one)
        if same_side and surrogate_applied:
          lead_two, _ = self._apply_overtake_surrogate(lead_two, sm, force=True)
        else:
          lead_two, _ = self._apply_overtake_surrogate(lead_two, sm)

      if surrogate_applied and lead_two.get('status', False):
        lead_one_track = lead_one.get('radarTrackId', -1)
        lead_two_track = lead_two.get('radarTrackId', -2)
        same_track = lead_one_track >= 0 and lead_one_track == lead_two_track
        same_side = self._lead_side_sign(lead_one) == self._lead_side_sign(lead_two)
        drel_diff = abs(lead_one.get('dRel', 0.0) - lead_two.get('dRel', 0.0))
        vlead_diff = abs(lead_one.get('vLead', self.v_ego) - lead_two.get('vLead', self.v_ego))
        close_untracked_same_side = lead_one_track < 0 and lead_two_track < 0 and same_side and drel_diff < 10.0 and vlead_diff < 5.0

        if same_track or close_untracked_same_side:
          lead_two, _ = self._apply_overtake_surrogate(lead_two, sm)

        hide_lead_two = False
        if lead_one_track >= 0 and lead_one_track == lead_two_track:
          hide_lead_two = True
        elif lead_one_track < 0 and lead_two_track < 0 and same_side:
          hide_lead_two = True

        if hide_lead_two:
          lead_two = lead_two.copy()
          lead_two['status'] = False

      self.radar_state.leadTwo = lead_two
    else:
      self._update_lane_change_surrogates(sm, {'status': False}, None)

    # FrogPilot variables
    if self.ready and (self.frogpilot_toggles.adjacent_lead_tracking or self.frogpilot_toggles.human_lane_changes):
      self.frogpilot_radar_state.leadLeft = get_adjacent_lead(self.tracks, sm['modelV2'], left=True)
      self.frogpilot_radar_state.leadRight = get_adjacent_lead(self.tracks, sm['modelV2'], left=False)

    self.frogpilot_toggles = get_frogpilot_toggles(sm)

  def publish(self, pm: messaging.PubMaster):
    assert self.radar_state is not None

    radar_msg = messaging.new_message("radarState")
    radar_msg.valid = self.radar_state_valid
    radar_msg.radarState = self.radar_state
    pm.send("radarState", radar_msg)

    # FrogPilot variables
    frogpilot_radar_msg = messaging.new_message("frogpilotRadarState")
    frogpilot_radar_msg.valid = self.radar_state_valid
    frogpilot_radar_msg.frogpilotRadarState = self.frogpilot_radar_state
    pm.send("frogpilotRadarState", frogpilot_radar_msg)


# fuses camera and radar data for best lead detection
def main() -> None:
  config_realtime_process(5, Priority.CTRL_LOW)

  # wait for stats about the car to come in from controls
  cloudlog.info("radard is waiting for CarParams")
  CP = messaging.log_from_bytes(Params().get("CarParams", block=True), car.CarParams)
  cloudlog.info("radard got CarParams")

  # *** setup messaging
  sm = messaging.SubMaster(['modelV2', 'carState', 'liveTracks'], poll='modelV2',
                           ignore_valid=['frogpilotPlan'])
  pm = messaging.PubMaster(['radarState'])

  RD = RadarD(CP.radarDelay)

  # FrogPilot variables
  sm = sm.extend(['frogpilotPlan'])
  pm = pm.extend(['frogpilotRadarState'])

  while 1:
    sm.update()

    RD.update(sm, sm['liveTracks'])
    RD.publish(pm)


if __name__ == "__main__":
  main()
