#!/usr/bin/env python3
"""Aggregate gain_census.py jsonl (stdin or file) into depth bins."""
import sys
import json
import statistics
import collections
BINS = [(-0.75, -0.5), (-1.0, -0.75), (-1.25, -1.0), (-1.5, -1.25), (-1.75, -1.5), (-2.0, -1.75), (-2.5, -2.0), (-9, -2.5)]
pool = collections.defaultdict(list)
src = open(sys.argv[1]) if len(sys.argv) > 1 else sys.stdin
for l in src:
  r = json.loads(l)
  for a, ae in r["pairs"]:
    for lo, hi in BINS:
      if lo <= a < hi:
        pool[(lo, hi)].append((a, ae))
        break
for b in BINS:
  p = pool.get(b, [])
  if len(p) < 20:
    continue
  ma = statistics.mean(x[0] for x in p)
  me = statistics.mean(x[1] for x in p)
  print(f"  sent [{b[0]},{b[1]}) n={len(p):6d} mean_sent {ma:6.2f} mean_aEgo {me:6.2f} ratio {me/ma:5.2f} shortfall {me-ma:+5.2f}")
