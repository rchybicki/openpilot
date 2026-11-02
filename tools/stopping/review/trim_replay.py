#!/usr/bin/env python3
"""Replay the cycle-32 trim's PURE update on recorded (aTarget, aEgo, vEgo, enabled) streams at 100 Hz with
candidate parameter sets (ignores the cap-written/clean-frame gates -> an UPPER bound on activity).
usage: trim_replay.py <rlog.zst>... ; prints per parameter set: active share, mean/p90 trim on braking frames."""
import sys
import bisect
import collections
import statistics
sys.path.insert(0, "/Users/radoslawchybicki/Repos/openpilot-rch/tools/stopping/review")
from triage_one import read_events
DT = 0.01
TAU = 45
LAG = 0.5
SETS = {"shipped (db 0.05, guard 0.25, KI 1.0)": (0.05, 0.25, 1.0), "db 0.03 guard 0.10": (0.03, 0.10, 1.0),
        "db 0.02 guard 0.10 KI 1.5": (0.02, 0.10, 1.5), "db 0.03 guard 0.0": (0.03, 0.0, 1.0)}
streams = []
for path in sys.argv[1:]:
  lp = []
  cs = []
  for ev in read_events(path):
    w = ev.which()
    t = ev.logMonoTime * 1e-9
    if w == 'longitudinalPlan':
      lp.append((t, float(ev.longitudinalPlan.aTarget)))
    elif w == 'carState':
      cs.append((t, ev.carState.vEgo, ev.carState.aEgo, ev.carState.cruiseState.enabled, ev.carState.brakePressed or ev.carState.gasPressed))
  if lp and cs:
    streams.append((lp, cs))
for name, (DB, GUARD, KI) in SETS.items():
  act = 0
  n = 0
  trims = []
  for lp, cs in streams:
    tl = [x[0] for x in lp]
    buf = collections.deque(maxlen=TAU)
    filt = None
    trim = 0.0
    for (t, v, ae, en, pedal) in cs:
      k = bisect.bisect_right(tl, t) - 1
      d = lp[k][1] if k >= 0 else 0.0
      buf.append(d)
      if len(buf) < TAU:
        continue
      raw = buf[0]
      prev = filt
      filt = ae if filt is None else filt + (raw - filt) * DT / LAG
      rate = 0.0 if prev is None else (filt - prev) / DT
      learn = en and not pedal and 2.5 <= v <= 16.0 and d <= -0.75
      if learn:
        e = filt - ae
        band = DB + GUARD * abs(rate)
        if e < -band:
          target = trim + KI * (e + band) * DT
        elif e > 0.03:
          target = trim + 2.0 * (e - 0.03) * DT
        else:
          target = min(trim + 0.01 * DT, 0.0)
      else:
        target = min(trim + 1.0 * DT, 0.0)
      target = min(max(target, -0.40), 0.0)
      trim = min(max(target, trim - 0.01), trim + 0.01)
      if learn:
        n += 1
        trims.append(trim)
        act += trim < -0.05
  q = statistics.quantiles(trims, n=10)
  print(f"{name:38} braking frames {n:6d}: active(<-0.05) {act/n:5.1%}  mean trim {statistics.mean(trims):+.3f}  p10 {q[0]:+.3f}  p50 {q[4]:+.3f}")
