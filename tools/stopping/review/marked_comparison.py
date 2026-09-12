#!/usr/bin/env python3
"""Describe explicitly labelled stops and the frozen engaged-v2 comparator; never rank comfort.

Use existing bookmarked_baseline packets. Every speed-band cell has its own continuity and
control-mode check. Last crossings can hide earlier rebounds: retain the separate last-30s window.
vEgoRaw is still ECU wheel telemetry, not an independent physical-rest measurement.
"""
import argparse
import bisect
import hashlib
import json
import math
from pathlib import Path

from openpilot.tools.stopping.review.human_baseline import SECOND, classify, jerk_max, state_at, window_reason


def describe(data, stop):
  car = data['car']
  t, v, a = [[r[i] for r in car] for i in range(3)]
  valid = [r[3] and math.isfinite(r[1]) and math.isfinite(r[2]) for r in car]
  st, states = [[r[i] for r in data['state']] for i in range(2)]
  enabled = [state_at(ns, st, states) for ns in t]
  k = bisect.bisect_left(t, stop)
  if k == len(t) or t[k] != stop or any(y <= x for x, y in zip(t, t[1:], strict=False)):
    raise ValueError('stop must match a sample in a strictly increasing carState stream')
  if any(y <= x for x, y in zip(st, st[1:], strict=False)):
    raise ValueError('selfdriveState timestamps must be strictly increasing')
  end = stop + SECOND // 2
  hi = bisect.bisect_right(t, end)
  lookback_start = bisect.bisect_left(t, stop - 30 * SECOND)
  bands = {}
  for speed in (5., 2.5, 1., .5, .1):
    j = next((i for i in range(k - 1, lookback_start - 1, -1) if v[i] >= speed), None)
    reason = 'no_crossing_in_30s' if j is None else window_reason(t, valid, t[j], end)
    cell = {'start_ns': t[j] if j is not None else None, 'end_ns': end, 'reason': reason, 'class': 'unknown'}
    if not reason:
      minimum, recovery = v[j], 0.
      for value in v[j:k + 1]:
        minimum = min(minimum, value)
        recovery = max(recovery, value - minimum)
      cell.update({'class': classify(enabled[j:hi], [r[4] for r in car[j:hi]], [r[5] for r in car[j:hi]]),
                   'seconds_to_filtered_rest': (stop - t[j]) / SECOND,
                   'wheel_distance_m': sum((v[i] + v[i + 1]) / 2 * (t[i + 1] - t[i]) / SECOND for i in range(j, k)),
                   'max_speed_recovery_mps': recovery, 'a_entry': a[j],
                   'jerk_300': jerk_max(t, a, valid, t[j], end)})
    bands[str(speed)] = cell
  # Repeat the already stated easing descriptor with raw-speed anchors, keeping aEgo unchanged.
  # This tests anchor sensitivity only; the same wheel sensors supply both channels.
  raw_valid = [ok and math.isfinite(r[6]) for ok, r in zip(valid, car, strict=True)]
  raw_anchors = []
  for speed in (.5, .1):
    j = next((i for i in range(k - 1, lookback_start - 1, -1) if car[i][6] >= speed), None)
    reason = 'no_crossing_in_30s' if j is None else window_reason(t, raw_valid, t[j], end)
    raw_anchors.append({'start_ns': t[j] if j is not None else None, 'reason': reason, 'a': a[j] if not reason else None})
  easing = None if any(bands[s]['reason'] for s in ('0.5', '0.1')) else bands['0.1']['a_entry'] - bands['0.5']['a_entry']
  raw_easing = None if any(r['reason'] for r in raw_anchors) else raw_anchors[1]['a'] - raw_anchors[0]['a']
  start = stop - 30 * SECOND
  lo = lookback_start
  reason = window_reason(t, valid, start, end)
  return {'bands': bands, 'easing_delta_a': easing, 'raw_anchor_easing_delta_a': raw_easing, 'raw_anchors': raw_anchors,
          'raw_speed_at_filtered_rest': car[k][6] if raw_valid[k] else None,
          'last30s': {'start_ns': start, 'end_ns': end, 'reason': reason,
                      'class': 'unknown' if reason else classify(enabled[lo:hi], [r[4] for r in car[lo:hi]], [r[5] for r in car[lo:hi]]),
                      'jerk_300': jerk_max(t, a, valid, start, end)}}


def compare(labels_path, reference_packets, comparison_packets):
  labels = json.loads(labels_path.read_text())
  manual, bad = labels['manual_reference_ids'], labels['bad_openpilot_ids']
  if len(set(manual + bad)) != len(manual + bad):
    raise ValueError('duplicate or conflicting explicit labels')
  rows, inputs, found = [], [], set()
  for directory in reference_packets + comparison_packets:
    paths = [directory / name for name in ('baseline.json', 'signals.json')]
    baseline, data = [json.loads(p.read_text()) for p in paths]
    if baseline['files'] != data['files'] or baseline['route'] != data['route']:
      raise ValueError(f'{directory}: mismatched packet provenance')
    for source in baseline['files']:
      raw = Path(source['path']).read_bytes()
      if len(raw) != source['bytes'] or hashlib.sha256(raw).hexdigest() != source['sha256']:
        raise ValueError(f'{source["path"]}: source changed')
    inputs.extend({'path': str(p), 'sha256': hashlib.sha256(p.read_bytes()).hexdigest()} for p in paths)
    for stop in baseline['stops']:
      event_id = stop['event_id']
      if directory in reference_packets:
        label = 'Radek_manual' if event_id in manual else 'user_bad_openpilot' if event_id in bad else None
      else:
        label = 'observed_engaged_v2' if stop['v2'] and stop['v2']['cls'] == 'engaged' and stop['v2']['context_complete'] else None
      if label is None:
        continue
      if event_id in found:
        raise ValueError(f'duplicate selected event: {event_id}')
      found.add(event_id)
      rows.append({'event_id': event_id, 'label': label, 'segment': stop['segment'], 'stop_mono_ns': stop['stop_mono_ns'],
                   'lead_at_stop': stop['lead_at_stop'], 'pose_device_x_jerk_300': stop['pose_device_x_jerk_300'],
                   **describe(data, stop['stop_mono_ns'])})
  if set(manual + bad) - found:
    raise ValueError(f'missing labelled events: {sorted(set(manual + bad) - found)}')
  return {'version': 1, 'labels': labels, 'labels_sha256': hashlib.sha256(labels_path.read_bytes()).hexdigest(),
          'source_sha256': {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in
                            (Path(__file__), Path(__file__).with_name('human_baseline.py'))}, 'inputs': inputs, 'rows': rows}


if __name__ == '__main__':
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('--labels', type=Path, required=True)
  parser.add_argument('--reference-packets', type=Path, nargs='+', required=True)
  parser.add_argument('--comparison-packets', type=Path, nargs='+', required=True)
  parser.add_argument('--output', type=Path, required=True)
  args = parser.parse_args()
  result = compare(args.labels, args.reference_packets, args.comparison_packets)
  args.output.write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')
  print(f'{len(result["rows"])} retained events -> {args.output}')
