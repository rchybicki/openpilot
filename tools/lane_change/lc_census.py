#!/usr/bin/env python3
"""Census of lane changes in synced rlogs.

For each laneChangeStarting episode: timing, whether a radar lead was published, whether the lane-change
surrogate signature (vRel == +5 m/s) appeared, and whether the driver pressed the brake.
Usage: lc_census.py [first-route-prefix]  -> JSON list on stdout
"""
import glob
import json
import os
import re
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))
from openpilot.tools.lib.logreader import LogReader

DATA = os.path.expanduser('~/.route_sync/data/media/0/realdata')
LANE_CHANGE_STARTING = 2


def _ev(x):
  return x.raw if hasattr(x, 'raw') else int(x)


def route_segments(route):
  segs = (p for p in glob.glob(f'{DATA}/{route}--*') if p.rsplit('--', 1)[1].isdigit())
  return sorted(segs, key=lambda p: int(p.rsplit('--', 1)[1]))


def rlog_path(seg):
  for name in ('rlog.zst', 'rlog'):
    p = os.path.join(seg, name)
    if os.path.exists(p):
      return p
  return None


def census(routes):
  out = []
  for route in routes:
    state = 0
    episode = None
    cs = None
    for seg in route_segments(route):
      rl = rlog_path(seg)
      if rl is None:
        continue
      try:
        for m in LogReader(rl):
          w = m.which()
          if w == 'carState':
            cs = m.carState
            if episode:
              episode['brake'] |= bool(cs.brakePressed)
              episode['gas'] |= bool(cs.gasPressed)
          elif w == 'modelV2':
            st = _ev(m.modelV2.meta.laneChangeState)
            if st == LANE_CHANGE_STARTING and state != LANE_CHANGE_STARTING:
              episode = dict(route=route, seg=os.path.basename(seg), t0=m.logMonoTime, dir=_ev(m.modelV2.meta.laneChangeDirection),
                             v0=cs.vEgo if cs else None, frames=0, brake=False, gas=False, lead_frames=0, surr_frames=0, min_drel=None)
            if st != LANE_CHANGE_STARTING and state == LANE_CHANGE_STARTING and episode:
              episode['dur_s'] = (m.logMonoTime - episode['t0']) / 1e9
              out.append(episode)
              episode = None
            state = st
            if episode:
              episode['frames'] += 1
          elif w == 'radarState' and episode:
            l1 = m.radarState.leadOne
            if l1.status:
              episode['lead_frames'] += 1
              episode['min_drel'] = l1.dRel if episode['min_drel'] is None else min(episode['min_drel'], l1.dRel)
              if abs(l1.vRel - 5.0) < 0.05 and not l1.fcw:
                episode['surr_frames'] += 1
      except Exception as e:
        print(f'ERR {seg}: {e}', file=sys.stderr)
  return out


if __name__ == '__main__':
  routes = sorted({re.sub(r'--\d+$', '', os.path.basename(p)) for p in glob.glob(f'{DATA}/*--*--*')})
  if len(sys.argv) > 1:
    routes = [r for r in routes if r >= sys.argv[1]]
  print(json.dumps(census(routes)))
