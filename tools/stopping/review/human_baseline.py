#!/usr/bin/env python3
"""Offline last-ten-seconds stop census, metric_version=2 (not a passenger-feel score).

Usage: human_baseline.py rlog.zst; human_baseline.py --aggregate results.jsonl
Stops: v<.05 for >=.5s, no carState gap >.1s; observed peak >=3 in [stop-10s, stop].
Approach: final contiguous v<=9 band in those ten seconds, through the sample before the
final v<.45 descent. Terminal: that descent's first sample through stop+.5s. Endpoints inclusive.
Jerk: max |a(t)-interp(a,t-.300)|/.300 at recorded endpoints, with the FULL window inside
its interval. Missing context, invalid samples or gaps produce null and a reason, never zero.
No filtering, no cross-segment stitching, no historical <=.8 acceptance threshold on v2.
Aggregates use the sample median and nearest-rank p90 (rank ceil(.9*n), one-based).
"""
import bisect
import collections
import json
import math
from pathlib import Path
import re
import statistics
import sys

SECOND = 1_000_000_000
METRICS = ('jerk_300_term', 'jerk_300_approach', 'jerk_300_last10s', 'a_min_late', 'a_last_ge_010')
# jerk_300_last10s covers [stop-10s, stop+.5s]; a_last_ge_010 is the last pre-stop sample with v>=.10.


def window_reason(t, valid, start, end):
  if not t or start > end or start < t[0] or end > t[-1]:
    return 'missing_context'
  lo = max(0, bisect.bisect_right(t, start) - 1)
  hi = bisect.bisect_left(t, end)
  if not all(valid[lo:hi + 1]):
    return 'invalid_sample'
  if any(not 0 < t[k] - t[k - 1] <= SECOND // 10 for k in range(lo + 1, hi + 1)):
    return 'timestamp_gap'
  return None


def jerk_max(t, a, valid, start, end):
  if any(t[k] <= t[k - 1] for k in range(1, len(t))):
    return {'value': None, 'reason': 'timestamp_gap'}
  valid = [ok and math.isfinite(value) for ok, value in zip(valid, a, strict=True)]
  reason = window_reason(t, valid, start, end)
  if reason:
    return {'value': None, 'reason': reason}
  values = []
  for k in range(bisect.bisect_left(t, start + 3 * SECOND // 10), bisect.bisect_right(t, end)):
    target = t[k] - 3 * SECOND // 10
    j = bisect.bisect_right(t, target) - 1
    old = a[j] + (a[j + 1] - a[j]) * (target - t[j]) / (t[j + 1] - t[j])
    values.append(abs(a[k] - old) / .3)
  return {'value': max(values) if values else None, 'reason': None if values else 'short_window'}


def classify(enabled, brake, gas):
  if not enabled or any(e is None for e in enabled):
    return 'unknown'
  if all(enabled) and not any(brake) and not any(gas):
    return 'engaged'
  # Accelerator use while fully disengaged is still manual driving, not an openpilot override.
  if not any(enabled) and any(brake):
    return 'manual'
  return 'mixed'


def state_at(t, state_times, states):
  k = bisect.bisect_right(state_times, t) - 1
  return states[k] if k >= 0 and 0 <= t - state_times[k] <= SECOND // 2 else None


def score_stops(t, v, a, valid, enabled, brake, gas):
  if any(t[k] <= t[k - 1] for k in range(1, len(t))):
    raise ValueError('carState timestamps must be strictly increasing')
  valid = [ok and math.isfinite(speed) and math.isfinite(accel) for ok, speed, accel in zip(valid, v, a, strict=True)]
  rows, k = [], 0
  while k < len(t):
    if not valid[k] or v[k] >= .05:
      k += 1
      continue
    j = k
    while j + 1 < len(t) and valid[j + 1] and v[j + 1] < .05 and t[j + 1] - t[j] <= SECOND // 10:
      j += 1
    entry_known = k > 0 and valid[k - 1] and v[k - 1] >= .05 and t[k] - t[k - 1] <= SECOND // 10
    if entry_known and t[j] - t[k] >= SECOND // 2:
      stop = t[k]
      start, end = stop - 10 * SECOND, stop + SECOND // 2
      ka, ke = bisect.bisect_left(t, start), bisect.bisect_right(t, end)
      speeds = [v[i] for i in range(ka, k + 1) if valid[i]]
      if speeds and max(speeds) >= 3:
        kt = k
        while kt > ka and valid[kt - 1] and v[kt - 1] < .45 and t[kt] - t[kt - 1] <= SECOND // 10:
          kt -= 1
        kb = kt
        while kb > ka and valid[kb - 1] and v[kb - 1] <= 9 and t[kb] - t[kb - 1] <= SECOND // 10:
          kb -= 1
        context = window_reason(t, valid, start, end)
        metrics = {
          'jerk_300_term': jerk_max(t, a, valid, t[kt], end),
          'jerk_300_approach': jerk_max(t, a, valid, t[kb], t[max(kb, kt - 1)]),
          'jerk_300_last10s': jerk_max(t, a, valid, start, end),
        }
        if kt == ka or not valid[kt - 1] or t[kt] - t[kt - 1] > SECOND // 10:
          metrics['jerk_300_term'] = {'value': None, 'reason': 'missing_phase_start'}
        if context:
          metrics['jerk_300_approach'] = {'value': None, 'reason': context}
        late = [a[i] for i in range(ka, k + 1) if valid[i] and .45 <= v[i] <= 3]
        reason = window_reason(t, valid, start, stop) or (None if late else 'empty_speed_band')
        metrics['a_min_late'] = {'value': min(late) if not reason else None, 'reason': reason}
        kr = k - 1
        while kr >= ka and valid[kr] and v[kr] < .10:
          kr -= 1
        reason = window_reason(t, valid, t[max(0, kr)], stop) if kr >= ka else 'missing_rolling_sample'
        metrics['a_last_ge_010'] = {'value': a[kr] if not reason else None, 'reason': reason}
        rows.append({'stop_mono_ns': stop, 'v_appr': max(speeds), 'context_complete': context is None,
                     'cls': 'unknown' if context else classify(enabled[ka:ke], brake[ka:ke], gas[ka:ke]), 'metrics': metrics})
    k = j + 1
  return rows


def read_stops(path):
  # Reuse the battery decoder; failures must be visible to the caller.
  from openpilot.tools.stopping.review.triage_one import read_events

  t, v, a, valid, brake, gas, state_times, states = [], [], [], [], [], [], [], []
  origin, commit = None, None
  for ev in read_events(str(path)):
    ns = int(ev.logMonoTime)
    if origin is None:
      origin = ns
    if ev.which() == 'initData':
      commit = ev.initData.gitCommit
    elif ev.which() == 'carState':
      cs = ev.carState
      t.append(ns)
      v.append(cs.vEgo)
      a.append(cs.aEgo)
      valid.append(bool(ev.valid))
      brake.append(bool(cs.brakePressed))
      gas.append(bool(cs.gasPressed))
    elif ev.which() == 'selfdriveState':
      state_times.append(ns)
      states.append(bool(ev.selfdriveState.enabled) if ev.valid else None)
  if any(state_times[k] < state_times[k - 1] for k in range(1, len(state_times))):
    raise ValueError(f'{path}: selfdriveState timestamps out of order')
  enabled = [state_at(ns, state_times, states) for ns in t]
  for row in score_stops(t, v, a, valid, enabled, brake, gas):
    row.update(metric_version=2, seg=path.parent.name, event_id=f'{path.parent.name}@{row["stop_mono_ns"]}',
               t=(row['stop_mono_ns'] - origin) / SECOND, commit=commit)
    yield row


def aggregate(rows):
  unique = {}
  for r in rows:
    if r.get('metric_version') != 2 or r.get('cls') not in ('engaged', 'manual', 'mixed', 'unknown'):
      raise ValueError('expected metric_version=2 and a known class; do not mix historical scores')
    if (type(r.get('stop_mono_ns')) is not int or r['stop_mono_ns'] <= 0 or not isinstance(r.get('seg'), str)
        or re.fullmatch(r'[0-9a-f]{8}--[0-9a-f]{10}--(?:0|[1-9][0-9]*)', r['seg']) is None
        or r.get('event_id') != f'{r["seg"]}@{r["stop_mono_ns"]}' or type(r.get('v_appr')) not in (int, float)
        or not math.isfinite(r['v_appr']) or r['v_appr'] < 3 or not isinstance(r.get('context_complete'), bool)
        or (r['cls'] in ('engaged', 'manual') and not r['context_complete']) or set(r.get('metrics', {})) != set(METRICS)):
      raise ValueError('malformed v2 stop record')
    for key, m in r['metrics'].items():
      if (not isinstance(m, dict) or set(m) != {'value', 'reason'} or
          not ((m['value'] is None and isinstance(m['reason'], str) and m['reason']) or
               (type(m['value']) in (int, float) and math.isfinite(m['value']) and m['reason'] is None))):
        raise ValueError('malformed metric value/reason')
      if key.startswith('jerk_') and m['value'] is not None and m['value'] < 0:
        raise ValueError('jerk metrics must be non-negative')
    if r['event_id'] in unique and r != unique[r['event_id']]:
      raise ValueError(f'conflicting duplicate: {r["event_id"]}')
    unique[r['event_id']] = r
  groups = collections.defaultdict(list)
  for r in unique.values():
    band = '3-6' if r['v_appr'] < 6 else '6-9' if r['v_appr'] < 9 else '9+'
    groups[(r['cls'], 'all')].append(r)
    groups[(r['cls'], band)].append(r)
  out = []
  for (cls, band), group in sorted(groups.items()):
    metrics = {}
    for key in METRICS:
      values = sorted(r['metrics'][key]['value'] for r in group if r['metrics'][key]['value'] is not None)
      metrics[key] = {'n': len(values), 'missing': dict(collections.Counter(r['metrics'][key]['reason'] for r in group
                                                                         if r['metrics'][key]['value'] is None)),
                      'p50': statistics.median(values) if values else None,
                      'p90': values[math.ceil(.9 * len(values)) - 1] if values else None}
    out.append({'cls': cls, 'speed_band': band, 'stops': len(group),
                'complete_context': sum(r['context_complete'] for r in group), 'metrics': metrics})
  return {'metric_version': 2, 'unique_stops': len(unique), 'groups': out}


if __name__ == '__main__':
  if len(sys.argv) == 3 and sys.argv[1] == '--aggregate':
    with open(sys.argv[2]) as stream:
      print(json.dumps(aggregate([json.loads(line) for line in stream if line.strip()]), indent=2, allow_nan=False))
  elif len(sys.argv) == 2:
    for record in read_stops(Path(sys.argv[1])):
      print(json.dumps(record, allow_nan=False))
  else:
    sys.exit(__doc__)
