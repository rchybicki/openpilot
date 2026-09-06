#!/usr/bin/env python3
"""Felt/terminal score for ONE settle. usage: felt_one.py <rlog.zst> <t_settle route-s> <label>
(the first rlog event carries the ROUTE-start stamp, so t_mono = first + t_route)"""
import sys
import json
import glob
import math
sys.path.insert(0, "/Users/radoslawchybicki/Repos/openpilot-rch/tools/stopping/review")
from triage_one import read_events
import terminal_smoothness as ts

path, t_route, label = glob.glob(sys.argv[1])[0], float(sys.argv[2]), sys.argv[3]
first = None
T, V, A, W = [], [], [], []
wire = float('nan')
for ev in read_events(path):
  t = ev.logMonoTime * 1e-9
  if first is None:
    first = t
  w = ev.which()
  if w == "carOutput":
    wire = ev.carOutput.actuatorsOutput.accel
  elif w == "carState" and math.isfinite(wire):
    T.append(t)
    V.append(ev.carState.vEgo)
    A.append(ev.carState.aEgo)
    W.append(wire)
s = ts.score_terminal(T, V, W, t_target=first + t_route)
if s:
  k0, k1 = s.pop("k_window")
  s["felt_jerk_max"] = ts.felt_jerk(T, A, k0, k1)
  s["label"] = label
  print(json.dumps({k: s[k] for k in ("label", "felt_jerk_max", "wire_jerk_max", "wire_pump", "descent_count", "relaunched") if k in s}))
else:
  print(json.dumps({"label": label, "result": None}))
