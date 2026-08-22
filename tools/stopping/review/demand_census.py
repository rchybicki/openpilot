#!/usr/bin/env python3
"""Demand-tracking pairs for ONE rlog: (planner aTarget(t), sent(t), aEgo(t+0.40)) on engaged, pedal-free, v>2.5
braking frames. The trim's effect is aEgo vs DEMAND (aEgo vs sent is the plant gain, unchanged by design)."""
import sys
import json
import bisect
sys.path.insert(0, "/Users/radoslawchybicki/Repos/openpilot-rch/tools/stopping/review")
from triage_one import read_events

path = sys.argv[1]
cs = []
co = []
lp = []
for ev in read_events(path):
  w = ev.which()
  t = ev.logMonoTime * 1e-9
  if w == 'carState':
    c = ev.carState
    cs.append((t, c.vEgo, c.aEgo, c.brakePressed or c.gasPressed, c.cruiseState.enabled))
  elif w == 'carOutput':
    co.append((t, float(ev.carOutput.actuatorsOutput.accel)))
  elif w == 'longitudinalPlan':
    lp.append((t, float(ev.longitudinalPlan.aTarget)))
pairs = []
if cs and co and lp:
  ts_ = [c[0] for c in cs]
  tl = [x[0] for x in lp]
  for (t, a) in co:
    k = bisect.bisect_right(tl, t) - 1
    if k < 0:
      continue
    d = lp[k][1]
    if d > -0.75:
      continue
    i = bisect.bisect_left(ts_, t)
    j = bisect.bisect_left(ts_, t + 0.40)
    if i >= len(cs) or j >= len(cs):
      continue
    c0, c1 = cs[i], cs[j]
    if not c0[4] or c0[3] or c1[3] or c0[1] < 2.5:
      continue
    pairs.append((round(d, 2), round(a, 2), round(c1[2], 2)))
print(json.dumps({"path": path, "pairs": pairs}))
