#!/usr/bin/env python3
"""Per-segment driving-mode census: experimentalMode share, forceCoast share, enabled frames. usage: mode_census.py <rlog.zst>"""
import sys
import json
sys.path.insert(0, "/Users/radoslawchybicki/Repos/openpilot-rch/tools/stopping/review")
from triage_one import read_events
path = sys.argv[1]
n = exp = fc = en = 0
src = {}
for ev in read_events(path):
  w = ev.which()
  if w == 'selfdriveState':
    n += 1
    exp += bool(ev.selfdriveState.experimentalMode)
    en += bool(ev.selfdriveState.enabled)
  elif w == 'frogpilotCarState':
    fc += bool(getattr(ev.frogpilotCarState, 'forceCoast', False))
  elif w == 'longitudinalPlan':
    s = str(ev.longitudinalPlan.longitudinalPlanSource)
    src[s] = src.get(s, 0) + 1
print(json.dumps({"path": path, "n": n, "exp_share": round(exp / n, 2) if n else None, "enabled": en, "forceCoast_frames": fc, "planSource": src}))
