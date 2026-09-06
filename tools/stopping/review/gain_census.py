#!/usr/bin/env python3
"""Plant tracking pairs for ONE rlog: (sent accel, aEgo 0.40 s later) on engaged, pedal-free, v>2 braking frames.
Aggregate with: bin by sent, ratio = mean(aEgo)/mean(sent)."""
import sys
import json
import bisect
sys.path.insert(0, "/Users/radoslawchybicki/Repos/openpilot-rch/tools/stopping/review")
from triage_one import read_events

path = sys.argv[1]
cs = []
co = []
for ev in read_events(path):
  w = ev.which()
  t = ev.logMonoTime * 1e-9
  if w == 'carState':
    c = ev.carState
    cs.append((t, c.vEgo, c.aEgo, c.brakePressed or c.gasPressed, c.cruiseState.enabled))
  elif w == 'carOutput':
    co.append((t, float(ev.carOutput.actuatorsOutput.accel)))
pairs = []
if cs and co:
  ts_ = [c[0] for c in cs]
  for (t, a) in co:
    if a > -0.5:
      continue
    i = bisect.bisect_left(ts_, t)
    j = bisect.bisect_left(ts_, t + 0.40)
    if i >= len(cs) or j >= len(cs):
      continue
    c0, c1 = cs[i], cs[j]
    if not c0[4] or c0[3] or c1[3] or c0[1] < 2.0:
      continue
    pairs.append((round(a, 2), round(c1[2], 2)))
print(json.dumps({"path": path, "pairs": pairs}))
