#!/usr/bin/env python3
"""Tracking window: vEgo / aEgo / SENT accel / planner aTarget / lead, at ~0.25 s, plus a shortfall summary.
usage: track_win.py <rlog.zst glob> <t0 route-s> <t1 route-s>"""
import sys
import glob
import statistics
sys.path.insert(0, "/Users/radoslawchybicki/Repos/openpilot-rch/tools/stopping/review")
from triage_one import read_events

path, t0, t1 = glob.glob(sys.argv[1])[0], float(sys.argv[2]), float(sys.argv[3])
rows = []
cs = cc = lp = rs = None
base = None
for ev in read_events(path):
  t = ev.logMonoTime * 1e-9
  if base is None:
    base = t
  tt = t - base
  w = ev.which()
  if w == 'carState':
    cs = ev.carState
  elif w == 'carControl':
    cc = ev.carControl
  elif w == 'longitudinalPlan':
    lp = ev.longitudinalPlan
  elif w == 'radarState':
    rs = ev.radarState
  elif w == 'carOutput' and cs and cc and lp and rs and t0 <= tt <= t1:
    rows.append((tt, cs.vEgo, cs.aEgo, float(ev.carOutput.actuatorsOutput.accel), float(lp.aTarget),
                 rs.leadOne.dRel, rs.leadOne.vLead, cs.brakePressed, cs.gasPressed, cs.cruiseState.enabled))
last = -9
print(f"{'t':7} {'v':5} {'aEgo':6} {'sent':6} {'aTgt':6} {'err':6} {'dRel':5} {'vL':5} flags")
for r in rows:
  if r[0] - last >= 0.25:
    last = r[0]
    flags = ('BRK' if r[7] else '') + ('GAS' if r[8] else '') + ('' if r[9] else ' off')
    print(f"{r[0]:7.2f} {r[1]:5.2f} {r[2]:6.2f} {r[3]:6.2f} {r[4]:6.2f} {r[2]-r[3]:6.2f} {r[5]:5.1f} {r[6]:5.2f} {flags}")
sel = [r for r in rows if r[3] <= -1.0 and r[9] and not r[7] and not r[8]]
if sel:
  ms = statistics.mean(r[3] for r in sel)
  ma = statistics.mean(r[2] for r in sel)
  print(f"SUMMARY n={len(sel)} mean sent {ms:.2f} mean aEgo {ma:.2f} ratio {ma/ms:.2f} shortfall {ma-ms:+.2f}")
