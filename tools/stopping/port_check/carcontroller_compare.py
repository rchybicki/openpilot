#!/usr/bin/env python3
"""Compare two carcontroller_replay.py pickles (A, B) frame by frame, with coverage, and A against the recorded sendcan of each
segment. argv: A.pkl B.pkl. Run with PYTHONPATH=<tree>:<tree>/frogpilot/third_party of the tree that decodes the logs."""
import bisect
import pickle
import sys
from collections import Counter

from openpilot.tools.lib.logreader import LogReader

a, b = (pickle.load(open(p, 'rb')) for p in sys.argv[1:3])
tot = Counter()
addr_diff = Counter()
first = []
rec_match = Counter()
rec_total = Counter()
for seg in a:
  fa, fb = a[seg], b[seg]
  if fa is None or fb is None:
    tot['segments_without_carparams'] += 1
    continue
  tot['segments'] += 1
  assert len(fa) == len(fb), (seg, len(fa), len(fb))
  for x, y in zip(fa, fb, strict=True):
    tot['frames'] += 1
    for addr, _, dat in x[1]:
      if addr == 0x421 and dat[1] & 0x80:   # SCC12 StopReq (bit 15)
        tot['frames_stopreq_set'] += 1
      if addr == 0x340:
        tot['lkas11_frames'] += 1
    tot['frames_any_scc12'] += any(addr == 0x421 for addr, _, _ in x[1])
    if x[1] != y[1]:
      tot['frames_byte_diff'] += 1
      sx, sy = set(x[1]), set(y[1])
      for addr, _, _ in sx ^ sy:
        addr_diff[hex(addr)] += 1
      if len(first) < 5:
        first.append((seg.split('/')[-2], x[0], sorted(sx - sy)[:3], sorted(sy - sx)[:3]))
    tot['accel_diff'] += x[2] != y[2]
    tot['available_diff'] += x[3] != y[3]
  # recorded sendcan: the first sendcan published after each replayed CAN batch
  sent = [(m.logMonoTime, sorted((c.address, c.src - 128 if c.src >= 128 else c.src, bytes(c.dat)) for c in m.sendcan))
          for m in LogReader(seg) if m.which() == 'sendcan']
  times = [t for t, _ in sent]
  for t, sends, _, _ in fa:
    i = bisect.bisect_right(times, t)
    if i < len(sent):
      rec = {(addr, dat) for addr, _, dat in sent[i][1]}
      for addr, _, dat in sends:
        rec_total[hex(addr)] += 1
        rec_match[hex(addr)] += (addr, dat) in rec
print(dict(tot))
print('addresses in differing frames:', dict(addr_diff))
for f in first:
  print('first diffs:', f)
print('port vs recorded sendcan (exact frame bytes present in the next sendcan), per address:')
for k in sorted(rec_total):
  print(f'  {k}: {rec_match[k]}/{rec_total[k]} = {rec_match[k] / rec_total[k]:.4f}')
