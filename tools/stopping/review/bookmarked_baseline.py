#!/usr/bin/env python3
"""Offline route packet: explicit bookmarks, continuous signals, and separate driver windows.

Usage: bookmarked_baseline.py --output DIR route-segment/rlog.zst ...
No driver identity is inferred. The caller must attach the user's attribution to selected events.
carState metrics reuse human_baseline v2. Device-axis livePose metrics are diagnostic proxies only.
"""
import argparse
import bisect
import hashlib
import json
import math
from pathlib import Path

import zstandard

from openpilot.tools.stopping.review.human_baseline import SECOND, classify, jerk_max, score_stops, state_at, window_reason
from openpilot.tools.stopping.review.triage_one import LOG


def rest_intervals(t, v, valid):
  """Include low-speed and boundary-crossing stops even if the v2 census omits them."""
  out, k = [], 0
  while k < len(t):
    if not valid[k] or v[k] >= .05:
      k += 1
      continue
    end = k
    while end + 1 < len(t) and valid[end + 1] and v[end + 1] < .05 and 0 < t[end + 1] - t[end] <= SECOND // 10:
      end += 1
    if k and valid[k - 1] and v[k - 1] >= .05 and 0 < t[k] - t[k - 1] <= SECOND // 10 and t[end] - t[k] >= SECOND // 2:
      out.append((k, end))
    k = end + 1
  return out


def bookmark_groups(raw):
  """Pair the UI button and userBookmark emitted for the same press; preserve both stamps."""
  groups = []
  for mark in sorted(raw, key=lambda m: m['mono_ns']):
    if (groups and mark['mono_ns'] - groups[-1][0]['mono_ns'] <= SECOND // 2
        and mark['type'] not in {m['type'] for m in groups[-1]}):
      groups[-1].append(mark)
    else:
      groups.append([mark])
  return groups


def read_route(paths):
  routes = {p.parent.name.rsplit('--', 1)[0] for p in paths}
  if len(routes) != 1:
    raise ValueError('one route per packet')
  data = {key: [] for key in ('car', 'state', 'pose', 'lead', 'wire', 'bookmarks', 'files', 'init')}
  for path in sorted(paths, key=lambda p: int(p.parent.name.rsplit('--', 1)[1])):
    raw = path.read_bytes()
    decoder = zstandard.ZstdDecompressor().decompressobj()
    decoded = decoder.decompress(raw)
    if not decoder.eof or decoder.unused_data:
      raise ValueError(f'{path}: incomplete or multi-frame log; refuse silent truncation')
    data['files'].append({'path': str(path), 'bytes': len(raw), 'sha256': hashlib.sha256(raw).hexdigest()})
    for ev in LOG.Event.read_multiple_bytes(decoded):
      ns, w, ok = int(ev.logMonoTime), ev.which(), bool(ev.valid)
      if w == 'initData':
        settings = {kv.key: bytes(kv.value).decode() for kv in ev.initData.params.entries
                    if kv.key in {'IncreasedStoppedDistance', 'CEForceCoastStrength', 'ExperimentalMode'}}
        data['init'].append({'segment': path.parent.name, 'mono_ns': ns, 'wall_ns': int(ev.initData.wallTimeNanos),
                             'commit': ev.initData.gitCommit, 'settings_at_init_only': settings})
      elif w == 'carState':
        c = ev.carState
        data['car'].append([ns, c.vEgo, c.aEgo, ok and c.canValid and math.isfinite(c.vEgo) and math.isfinite(c.aEgo),
                            bool(c.brakePressed), bool(c.gasPressed),
                            c.vEgoRaw, c.steeringAngleDeg, str(c.gearShifter), path.parent.name])
      elif w == 'selfdriveState':
        data['state'].append([ns, bool(ev.selfdriveState.enabled) if ok else None])
      elif w == 'livePose':
        p = ev.livePose
        data['pose'].append([ns, p.accelerationDevice.x, p.angularVelocityDevice.y,
                             ok and p.accelerationDevice.valid and math.isfinite(p.accelerationDevice.x),
                             ok and p.angularVelocityDevice.valid and math.isfinite(p.angularVelocityDevice.y)])
      elif w == 'radarState':
        p = ev.radarState.leadOne
        data['lead'].append([ns, bool(ok and p.status), p.dRel, p.vLead, p.modelProb, p.radarTrackId])
      elif w == 'carOutput':
        data['wire'].append([ns, ev.carOutput.actuatorsOutput.accel, ok])
      elif w in ('userBookmark', 'bookmarkButton'):
        data['bookmarks'].append({'mono_ns': ns, 'type': w, 'valid': ok, 'segment': path.parent.name})
  for key in ('car', 'state', 'pose', 'lead', 'wire'):
    # Never silently sort or deduplicate state streams: doing so can hide a broken join.
    if any(b[0] <= a[0] for a, b in zip(data[key], data[key][1:], strict=False)):
      raise ValueError(f'{key}: non-increasing timestamps')
  data['route'] = routes.pop()
  return data


def analyze(data):
  car = data['car']
  t, v, a, valid, brake, gas = [[r[i] for r in car] for i in range(6)]
  st, enabled_states = [[r[i] for r in data['state']] for i in range(2)]
  enabled = [state_at(ns, st, enabled_states) for ns in t]
  v2 = {r['stop_mono_ns']: r for r in score_stops(t, v, a, valid, enabled, brake, gas)}
  pose_t = [r[0] for r in data['pose']]
  lead_t = [r[0] for r in data['lead']]
  origin = data['init'][0]['mono_ns']
  stops = []
  for k, end in rest_intervals(t, v, valid):
    stop = t[k]
    kt = k
    while kt and valid[kt - 1] and v[kt - 1] < .45 and t[kt] - t[kt - 1] <= SECOND // 10:
      kt -= 1
    windows = {}
    for name, start, finish in [('last30s', stop - 30 * SECOND, stop + SECOND // 2),
                                ('last10s', stop - 10 * SECOND, stop + SECOND // 2),
                                ('terminal', t[kt], stop + SECOND // 2)]:
      lo, hi = bisect.bisect_left(t, start), bisect.bisect_right(t, finish)
      reason = window_reason(t, valid, start, finish)
      windows[name] = {'start_ns': start, 'end_ns': finish, 'reason': reason,
                       'class': 'unknown' if reason else classify(enabled[lo:hi], brake[lo:hi], gas[lo:hi]),
                       'jerk_300': jerk_max(t, a, valid, start, finish)}
    if kt == 0 or not valid[kt - 1] or t[kt] - t[kt - 1] > SECOND // 10:
      windows['terminal']['reason'] = 'missing_phase_start'
      windows['terminal']['class'] = 'unknown'
      windows['terminal']['jerk_300'] = {'value': None, 'reason': 'missing_phase_start'}
    lo = bisect.bisect_left(t, stop - 30 * SECOND)
    # Continuous manual duration, independently of how much of the approach the driver chose to do.
    km = k
    while km and enabled[km - 1] is False and valid[km - 1] and t[km] - t[km - 1] <= SECOND // 10:
      km -= 1
    manual = None
    if enabled[k] is False:
      manual = {'start_ns': t[km], 'seconds_before_stop': (stop - t[km]) / SECOND, 'v_start': v[km],
                'a_start': a[km], 'previous_a': a[km - 1] if km else None}
    samples = {}
    for speed in (3, 2.5, 1, .5, .2, .1):
      j = k - 1
      while j >= lo and v[j] < speed:
        j -= 1
      samples[str(speed)] = ({'v': v[j], 'a': a[j], 'before_stop_s': (stop - t[j]) / SECOND} if j >= lo else None)
    pj = max(0, bisect.bisect_left(pose_t, stop - 2 * SECOND) - 1)
    pe = bisect.bisect_right(pose_t, stop + SECOND) + 1
    pose = data['pose'][pj:pe]
    pt, px, pg, pv, gv = [[r[i] for r in pose] for i in range(5)]
    pitch_reason = window_reason(pt, gv, stop - 2 * SECOND, stop + SECOND)
    li = bisect.bisect_right(lead_t, stop) - 1
    lead = data['lead'][li] if li >= 0 and stop - lead_t[li] <= SECOND // 2 else None
    stops.append({'event_id': f'{data["route"]}@{stop}', 'stop_mono_ns': stop, 'route_s': (stop - origin) / SECOND,
                  'segment': car[k][-1], 'rest_end_ns': t[end], 'windows': windows, 'manual_prefix': manual,
                  'v_max_last30s': max(v[lo:k + 1]), 'v2': v2.get(stop), 'speed_samples': samples, 'lead_at_stop': lead,
                  'pitch_device_y_peak': {'value': max(abs(x) for ns, x in zip(pt, pg, strict=True)
                                                       if stop - 2 * SECOND <= ns <= stop + SECOND) if not pitch_reason else None,
                                          'reason': pitch_reason},
                  'pose_device_x_jerk_300': jerk_max(pt, px, pv, stop - 2 * SECOND, stop + SECOND)})
  bookmarks = []
  for group in bookmark_groups(data['bookmarks']):
    ns = group[0]['mono_ns']
    # Association only during observed rest (one-sample timing allowance); never pick an arbitrary nearest stop.
    candidates = [r for r in stops if r['stop_mono_ns'] - SECOND // 10 <= ns <= r['rest_end_ns'] + SECOND // 10]
    bookmarks.append({'events': group, 'route_s': (ns - origin) / SECOND,
                      'candidate_ids': [r['event_id'] for r in candidates],
                      'status': 'matched' if len(candidates) == 1 else 'ambiguous' if candidates else 'unmatched'})
  scorers = [Path(__file__), Path(__file__).with_name('human_baseline.py')]
  return {'packet_version': 1, 'route': data['route'], 'files': data['files'], 'init': data['init'],
          'scorer_sha256': {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in scorers},
          'bookmarks': bookmarks, 'stops': stops}


if __name__ == '__main__':
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('--output', type=Path, required=True)
  parser.add_argument('paths', type=Path, nargs='+')
  args = parser.parse_args()
  packet = read_route(args.paths)
  result = analyze(packet)
  args.output.mkdir(parents=True, exist_ok=True)
  (args.output / 'baseline.json').write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')
  (args.output / 'signals.json').write_text(json.dumps(packet, allow_nan=False) + '\n')
  print(json.dumps({'route': packet['route'], 'segments': len(packet['files']), 'stops': len(result['stops']),
                    'bookmarks': len(result['bookmarks']), 'output': str(args.output)}))
