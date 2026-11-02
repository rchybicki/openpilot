#!/usr/bin/env python3
"""Classify each replayed lane-change episode by how ours vs upstream behaved.

Per frame we know: ours leadOne, upstream leadOne, the raw closest in-path radar track, ego speed.
Per episode we derive:
  surr_s          seconds ours published a surrogate (vRel == +5)
  up_swap_s       seconds upstream published a DIFFERENT track than ours' underlying track (= it selected a target-lane car)
  hidden_close_s  seconds ours was surrogated while the surrogated car itself was closing (vLead < v_ego - 1) within 45 m
  up_close_s      seconds upstream's lead was closing within 45 m (upstream would brake)
  brake           driver brake pressed during the episode
"""
import json
import sys
import collections

def closing(lead, v_ego, gap=45.0):
  return lead is not None and lead['d'] < gap and lead['v'] < v_ego - 1.0

def analyze(path):
  eps = json.load(open(path))
  rows = []
  for e in eps:
    fr = e['frames']
    if not fr:
      continue
    dt = e['dur_s'] / max(len(fr), 1)
    surr = up_swap = hidden = up_close = 0
    for f in fr:
      o, u, r, v = f['ours'], f['up'], f['raw'], f['v']
      if isinstance(o, str) or isinstance(u, str):
        continue
      is_surr = o is not None and abs(o['vr'] - 5.0) < 0.05
      if is_surr:
        surr += 1
        # the surrogated track's REAL motion: upstream publishes the same track unmodified when it did not swap
        real = u if (u is not None and u['tid'] == o['tid'] and o['tid'] >= 0) else r
        if closing(real, v):
          hidden += 1
      if o is not None and u is not None and o['tid'] != u['tid']:
        up_swap += 1
      elif o is None and u is not None:
        up_swap += 1
      if closing(u, v):
        up_close += 1
    rows.append(dict(route=e['route'], seg=e['seg'], dir=e['dir'], v0=e['v0'], dur=e['dur_s'], brake=e['brake'],
                     surr_s=round(surr * dt, 2), up_swap_s=round(up_swap * dt, 2), hidden_close_s=round(hidden * dt, 2),
                     up_close_s=round(up_close * dt, 2)))
  return rows

def fmt(r):
  seg = r['seg'].rsplit('--', 1)[1]
  head = f"  {r['route']} {seg:>3} dir={r['dir']} v0={r['v0']} dur={r['dur']} brake={int(r['brake'])} surr={r['surr_s']}"
  return head + f" hidden={r['hidden_close_s']} up_swap={r['up_swap_s']} up_close={r['up_close_s']}"


if __name__ == '__main__':
  rows = []
  for p in sys.argv[1:]:
    rows += analyze(p)
  print(f"episodes: {len(rows)}")
  c = collections.Counter()
  for r in rows:
    c['surrogated'] += r['surr_s'] > 0
    c['upstream picked other track'] += r['up_swap_s'] > 0
    c['ours hid a closing in-path car'] += r['hidden_close_s'] > 0.3
    c['upstream lead closing'] += r['up_close_s'] > 0.3
    c['driver braked'] += r['brake']
    c['braked AND ours hid closing car'] += r['brake'] and r['hidden_close_s'] > 0.3
    c['braked AND upstream lead closing'] += r['brake'] and r['up_close_s'] > 0.3
  for k, v in c.items():
    print(f"  {v:4d}  {k}")
  print("\nworst (ours hid a closing car, longest):")
  for r in sorted(rows, key=lambda r: -r['hidden_close_s'])[:12]:
    print(fmt(r))
  print("\nupstream picked a different track (target-lane car present):")
  for r in sorted(rows, key=lambda r: -r['up_swap_s'])[:12]:
    if r['up_swap_s'] > 0:
      print(fmt(r))
  json.dump(rows, open('/tmp/lc_outcomes.json', 'w'), indent=1)
