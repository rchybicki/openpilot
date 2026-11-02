#!/usr/bin/env python3
"""Attributed-safety flip-gate tally from UNIQUE raw settle_summary events (never from matched index rows:
cycle 42 found repeated matches inflate them). usage: attr_gate_tally.py <rlog.zst>...  (or a dir glob)
Prints totals + per-route counts; the gate (program doc 2026-09-02): >=100 held-out stops over >=5 routes."""
import sys, json, glob, os, collections
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from stop_index import settle_events
paths = []
for a in sys.argv[1:]:
  paths += glob.glob(a) if any(c in a for c in "*?[") else [a]
tot = collections.Counter(); reasons = collections.Counter(); routes = collections.Counter(); seen = set(); n = 0; with_unexp = 0
for p in sorted(paths):
  seg = p.split("/realdata/")[-1].split("/")[0]
  for ev in settle_events(p):
    if not ev.get("attr_frames"):
      continue
    key = (seg, round(ev["t"], 1))
    if key in seen:
      continue
    seen.add(key); n += 1; routes[seg.split("--")[0]] += 1
    for k in ("attr_frames", "attr_ineligible", "attr_plan_bound", "attr_unexplained"):
      tot[k] += ev.get(k) or 0
    tot["released"] += ev.get("attr_released_sum") or 0.0
    for k, v in (ev.get("attr_reasons") or {}).items():
      reasons[k] += v
    if (ev.get("attr_unexplained") or 0) > 0:
      with_unexp += 1
el = tot["attr_frames"] - tot["attr_ineligible"]
print(f"unique raw settle events with attr data: {n} over {len(routes)} routes {dict(routes)}")
print(f"frames {tot['attr_frames']} eligible {el} ({el / max(tot['attr_frames'], 1) * 100:.1f}%) reasons {dict(reasons)}")
print(f"plan-bound {tot['attr_plan_bound']} ({tot['attr_plan_bound'] / max(el, 1) * 100:.1f}% of eligible); unexplained {tot['attr_unexplained']} "
      f"({tot['attr_unexplained'] / max(tot['attr_plan_bound'], 1) * 100:.1f}% of binds); released depth sum {tot['released']:.1f}; events with unexplained binds {with_unexp}/{n}")
