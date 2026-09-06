#!/usr/bin/env python3
"""Aim deficit on a lead-stop approach: at v = 3.0/2.5/2.0/1.5/1.0 m/s print gap, required decel to rest at
REST (4.3) lag-aware (gap - REST - v*0.25), the planner aTarget, the sent wire and realized aEgo.
usage: aim_deficit.py <rlog.zst> <t_settle route-s> <label> [REST]"""
import sys
import glob
sys.path.insert(0, "/Users/radoslawchybicki/Repos/openpilot-rch/tools/stopping/review")
from triage_one import read_events

path, t_settle, label = glob.glob(sys.argv[1])[0], float(sys.argv[2]), sys.argv[3]
REST = float(sys.argv[4]) if len(sys.argv) > 4 else 4.3
first = None
rows = []
cs = lp = rs = None
for ev in read_events(path):
  t = ev.logMonoTime * 1e-9
  if first is None:
    first = t
  tt = t - first
  w = ev.which()
  if tt < t_settle - 12 or tt > t_settle + 0.5:
    continue
  if w == 'carState':
    cs = ev.carState
  elif w == 'longitudinalPlan':
    lp = ev.longitudinalPlan
  elif w == 'radarState':
    rs = ev.radarState
  elif w == 'carOutput' and cs and lp and rs:
    rows.append((tt, cs.vEgo, cs.aEgo, float(ev.carOutput.actuatorsOutput.accel), float(lp.aTarget), rs.leadOne.dRel, rs.leadOne.vLead))
out = [label]
for vq in (3.0, 2.5, 2.0, 1.5, 1.0, 0.7):
  k = next((i for i in range(len(rows)) if rows[i][1] <= vq and rows[i][0] > t_settle - 12), None)
  if k is None:
    continue
  tt, v, ae, sent, atgt, gap, lv = rows[k]
  rem = gap - REST - v * 0.25
  req = v * v / (2 * rem) if rem > 0.1 else float('inf')
  head = f"v{vq:.1f}: gap {gap:4.1f} rem {rem:4.1f} REQ {req:4.2f} | aTgt {atgt:5.2f} sent {sent:5.2f} aEgo {ae:5.2f}"
  out.append(head + f" | deficit(req+aTgt) {req + atgt:+.2f}")
print("\n  ".join(out))
