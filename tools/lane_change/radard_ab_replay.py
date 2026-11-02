#!/usr/bin/env python3
"""Replay recorded radar tracks through two radard implementations and compare the leadOne each publishes
during lane-change episodes.

  ours     = selfdrive/controls/radard.py at HEAD (lane-change surrogate)
  upstream = radard_upstream_13fa3b292a.py, FrogPilot-Testing source (cached adjacent-lane lead selection)

Both are fed the same liveTracks / modelV2 / carState / frogpilotPlan stream. Output: one JSON row per
laneChangeStarting episode with, per frame, the published lead of each implementation and the raw closest
in-path track (ground truth for 'was there something real ahead').
"""
import argparse
import glob
import importlib.util
import json
import os
import sys
from collections import deque
from types import SimpleNamespace

import numpy as np

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, REPO)
from cereal import custom, log
from openpilot.tools.lib.logreader import LogReader

DATA = os.path.expanduser('~/.route_sync/data/media/0/realdata')
LCS = log.LaneChangeState
LANE_CHANGE_STARTING = 2


def _ev(x):
  return x.raw if hasattr(x, 'raw') else int(x)


def load_radard(name, path):
  spec = importlib.util.spec_from_file_location(name, path)
  mod = importlib.util.module_from_spec(spec)
  spec.loader.exec_module(mod)
  return mod


class FakeSM:
  """Minimal SubMaster stand-in fed from an rlog."""

  def __init__(self):
    self.msgs = {}
    self.logMonoTime = {}
    self.recv_frame = {'carState': -1, 'modelV2': -1, 'liveTracks': -1, 'frogpilotPlan': -1}
    self.seen = {'modelV2': False, 'frogpilotPlan': False}
    self.frame = 0

  def feed(self, which, msg, t):
    self.msgs[which] = msg
    self.logMonoTime[which] = t
    self.recv_frame[which] = self.frame
    self.seen[which] = True
    self.frame += 1

  def __getitem__(self, k):
    return self.msgs[k]

  def all_checks(self):
    return True


def make_toggles():
  return SimpleNamespace(human_lane_changes=True, lead_detection_probability=0.35, adjacent_lead_tracking=True, lane_detection_width=0.0)


def raw_in_path_track(tracks, model):
  """Closest radar track inside the ego lane lines (model frame): what a driver would call 'the car ahead'."""
  best = None
  ll = model.laneLines
  if len(ll) < 3:
    return None
  for c in tracks.values():
    left = float(np.interp(c.dRel, ll[1].x, ll[1].y))
    right = float(np.interp(c.dRel, ll[2].x, ll[2].y))
    if left < -c.yRel < right and c.dRel < 120.0 and (best is None or c.dRel < best.dRel):
      best = c
  return best


def lead_row(lead):
  if lead is None or not lead.status:
    return None
  return dict(d=round(lead.dRel, 1), v=round(lead.vLead, 2), vr=round(lead.vRel, 2), y=round(lead.yRel, 2), tid=lead.radarTrackId)


def make_radard(mod, toggles, ours):
  """Build a RadarD without calling __init__ (it reads Params)."""
  rd = mod.RadarD.__new__(mod.RadarD)
  rd.current_time = 0.0
  rd.tracks = {}
  rd.kalman_params = mod.KalmanParams(mod.DT_MDL)
  rd.v_ego = 0.0
  rd.v_ego_hist = deque([0.0], maxlen=1)
  rd.last_v_ego_frame = -1
  rd.radar_state = None
  rd.radar_state_valid = False
  rd.ready = False
  rd.frogpilot_radar_state = custom.FrogPilotRadarState.new_message()
  rd.frogpilot_toggles = toggles
  if ours:
    rd.surrogate_track_ids = set()
    rd.target_lane_released_track_ids = set()
    rd.target_lane_released_leads = deque(maxlen=8)
    rd.target_lane_crossing_counts = {}
    rd.main_untracked_active = False
    rd.main_untracked_sign = 0
    rd.surrogate_untracked_side_signs = set()
    rd.prev_lane_change_state = LCS.off
    rd.lc_direction_sign = 0
    rd.center_surrogate_enabled = False
    rd.divider_lane_line_idx = -1
    rd.divider_initial_sign = 0
    rd.divider_crossed_counter = 0
    rd.divider_crossed = False
    rd.surrogate_phase = mod.SURROGATE_PHASE_OFF
    rd.surrogate_speed_gate_open = False
  mod.get_frogpilot_toggles = lambda sm=None, _t=toggles: _t  # neutralize the toggle refresh in update()
  return rd


def route_segments(route):
  segs = (p for p in glob.glob(f'{DATA}/{route}--*') if p.rsplit('--', 1)[1].isdigit())
  return sorted(segs, key=lambda p: int(p.rsplit('--', 1)[1]))


def rlog_path(seg):
  for name in ('rlog.zst', 'rlog'):
    p = os.path.join(seg, name)
    if os.path.exists(p):
      return p
  return None


def run_route(route, ours_mod, up_mod):
  toggles = make_toggles()
  impls = {'ours': make_radard(ours_mod, toggles, ours=True), 'up': make_radard(up_mod, toggles, ours=False)}
  sm = FakeSM()
  episodes = []
  ep = None
  state = 0
  cs = None
  for seg in route_segments(route):
    rl = rlog_path(seg)
    if rl is None:
      continue
    try:
      lr = LogReader(rl)
    except Exception as e:
      print(f'ERR open {seg}: {e}', file=sys.stderr)
      continue
    for m in lr:
      w = m.which()
      if w in ('carState', 'modelV2', 'frogpilotPlan'):
        sm.feed(w, getattr(m, w), m.logMonoTime)
        if w == 'carState':
          cs = m.carState
          if ep and cs.brakePressed:
            ep['brake'] = True
        elif w == 'modelV2':
          st = _ev(m.modelV2.meta.laneChangeState)
          if st == LANE_CHANGE_STARTING and state != LANE_CHANGE_STARTING:
            ep = dict(route=route, seg=os.path.basename(seg), t0=m.logMonoTime, dir=_ev(m.modelV2.meta.laneChangeDirection),
                      v0=round(cs.vEgo, 1) if cs else None, brake=False, frames=[])
          if st != LANE_CHANGE_STARTING and state == LANE_CHANGE_STARTING and ep:
            ep['dur_s'] = round((m.logMonoTime - ep['t0']) / 1e9, 2)
            episodes.append(ep)
            ep = None
          state = st
      elif w == 'liveTracks':
        if any(k not in sm.msgs for k in ('modelV2', 'carState', 'frogpilotPlan')):
          continue
        sm.feed('liveTracks', m.liveTracks, m.logMonoTime)
        rows = {}
        for name, rd in impls.items():
          try:
            rd.update(sm, m.liveTracks)
            rows[name] = lead_row(rd.radar_state.leadOne) if rd.radar_state is not None else None
          except Exception as e:
            rows[name] = f'ERR {type(e).__name__}: {e}'
        if ep is not None:
          raw = raw_in_path_track(impls['ours'].tracks, sm['modelV2'])
          ep['frames'].append(dict(t=round((m.logMonoTime - ep['t0']) / 1e9, 2), v=round(sm['carState'].vEgo, 1), ours=rows['ours'], up=rows['up'],
                                   raw=(dict(d=round(raw.dRel, 1), v=round(raw.vLeadK, 2), y=round(raw.yRel, 2), tid=raw.identifier) if raw else None),
                                   st=state))
  return episodes


if __name__ == '__main__':
  ap = argparse.ArgumentParser()
  ap.add_argument('routes', nargs='+')
  ap.add_argument('--out', default='/tmp/radard_ab_replay.json')
  a = ap.parse_args()
  ours = load_radard('radard_ours', os.path.join(REPO, 'selfdrive/controls/radard.py'))
  up = load_radard('radard_up', os.path.join(HERE, 'radard_upstream_13fa3b292a.py'))
  all_eps = []
  for r in a.routes:
    all_eps += run_route(r, ours, up)
  json.dump(all_eps, open(a.out, 'w'))
  print(f'{len(all_eps)} episodes -> {a.out}')
