#!/usr/bin/env python3
"""Compare two equiv_replay result dirs (A = candidate, B = reference) per group, stage, tick and field. Exact float equality; prints the
differing ticks per field (count, max |A - B|, first route times) and, with --dump, writes them to <A dir>/diff_<group>.json."""
import json
import pickle
import sys
from pathlib import Path

import numpy as np

from equiv_replay import CTRL_OUT, FP_OUT

STAGES = {'radard': None, 'planner': ('at', 'ss', 'dts', 'dtsm', 'fcw', 'traj', 'trajv'),
          'loc': ('accel', 'lcs', 'phase', 'owning', 'svc_active'), 'chain': ('accel', 'lcs', 'phase', 'owning', 'svc_active'),
          'fp': FP_OUT + ('ceStatus',), 'controls': CTRL_OUT}   # fp / controls: runs with EQUIV_STORE_JSON only


def compare(a, b):
  out = {}
  o = b['origin']
  for stage, fields in STAGES.items():
    if stage not in b:
      continue
    A, B = a[stage], b[stage]
    fields = fields or tuple(k for k in B if k.startswith(('l1_', 'l2_')))
    same_ticks = np.array_equal(A['ns'], B['ns'])
    if not same_ticks:
      out[stage] = dict(ticks=(len(A['ns']), len(B['ns'])), same_ticks=False)
      continue
    rows = {}
    for f in fields:
      x, y = np.asarray(A[f], float), np.asarray(B[f], float)
      bad = ~((x == y) | (np.isnan(x) & np.isnan(y)))
      if bad.any():
        idx = np.where(bad)[0]
        rows[f] = dict(n=int(bad.sum()), max_abs=float(np.nanmax(np.abs(x[idx] - y[idx]))),
                       t=[round((int(A['ns'][i]) - o) / 1e9, 3) for i in idx[:12]],
                       a=[float(x[i]) for i in idx[:6]], b=[float(y[i]) for i in idx[:6]])
    out[stage] = dict(ticks=len(B['ns']), same_ticks=True, diff=rows)
  return out


def main(dir_a, dir_b, dump=False):
  total = 0
  for pb in sorted(Path(dir_b).glob('g*.pkl')):
    pa = Path(dir_a) / pb.name
    if not pa.exists():   # a partial run (mutations) covers some groups only
      continue
    a, b = pickle.loads(pa.read_bytes()), pickle.loads(pb.read_bytes())
    res = compare(a, b)
    print(f'== {pb.stem}  dropped(A)={ {s: len(v) for s, v in a["dropped"].items()} }  cp_dropped(A)={len(a["cp_dropped"])}')
    for stage, r in res.items():
      if not r['same_ticks']:
        print(f'  {stage}: TICKS DIFFER {r["ticks"]}')
        total += 1
        continue
      print(f'  {stage}: {r["ticks"]} ticks, fields differing: {len(r["diff"])}')
      for f, d in r['diff'].items():
        total += d['n']
        print(f'    {f}: n={d["n"]} max|A-B|={d["max_abs"]:.6g} t={d["t"]} A={d["a"]} B={d["b"]}')
    if dump:
      (Path(dir_a) / f'diff_{pb.stem}.json').write_text(json.dumps(res, indent=1))
  print('TOTAL differing field-ticks:', total)
  return total


if __name__ == "__main__":
  sys.exit(0 if main(sys.argv[1], sys.argv[2], '--dump' in sys.argv) == 0 else 1)
