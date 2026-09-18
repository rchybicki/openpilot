#!/usr/bin/env python3
"""Census every observed stop in frozen route packets, including slow and mixed-control stops.

Signed 300 ms jerk is a wheel-telemetry descriptor, not a comfort score. Keep explicit
driver references separate; bookmarks retain only the packet's observed-rest association.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path

from openpilot.tools.stopping.review.bookmarked_baseline import analyze
from openpilot.tools.stopping.review.human_baseline import jerk_extrema
from openpilot.tools.stopping.review.marked_comparison import describe


def review(packet_dirs):
  rows, packets, inputs, found = [], [], [], set()
  for directory in packet_dirs:
    paths = [directory / name for name in ('baseline.json', 'signals.json')]
    raw_packets = [p.read_bytes() for p in paths]
    baseline, data = [json.loads(raw) for raw in raw_packets]
    if baseline['packet_version'] != 1 or any(baseline[key] != data[key] for key in ('files', 'route', 'init')):
      raise ValueError(f'{directory}: mismatched packet provenance')
    for source in baseline['files']:
      with Path(source['path']).open('rb') as raw:
        digest = hashlib.file_digest(raw, 'sha256').hexdigest()
        size = raw.seek(0, 2)
      if size != source['bytes'] or digest != source['sha256']:
        raise ValueError(f'{source["path"]}: source changed')
    inputs.extend({'path': str(p), 'sha256': hashlib.sha256(raw).hexdigest()} for p, raw in zip(paths, raw_packets, strict=True))
    packets.append({key: baseline[key] for key in ('route', 'files', 'init', 'bookmarks', 'scorer_sha256')})
    car = data['car']
    t, a = [r[0] for r in car], [r[2] for r in car]
    valid = [r[3] and math.isfinite(r[1]) and math.isfinite(r[2]) for r in car]
    for stop in baseline['stops']:
      event_id = stop['event_id']
      if event_id != f'{baseline["route"]}@{stop["stop_mono_ns"]}':
        raise ValueError(f'{directory}: mismatched event identity')
      if event_id in found:
        raise ValueError(f'duplicate event: {event_id}')
      found.add(event_id)
      description = describe(data, stop['stop_mono_ns'])
      del description['last30s']  # The packet already owns all three independently checked windows.
      windows = {}
      for name in ('last30s', 'last10s', 'terminal'):
        window = stop['windows'][name]
        signed = (dict(min=None, max=None, min_end_ns=None, max_end_ns=None, reason=window['reason']) if window['reason'] else
                  jerk_extrema(t, a, valid, window['start_ns'], window['end_ns']))
        windows[name] = {**window, 'jerk_300_signed': signed}
      rows.append({**stop, **description, 'route': baseline['route'], 'windows': windows})
    if baseline != analyze(data):
      raise ValueError(f'{directory}: mismatched derived baseline or scorer')
  return {'version': 1, 'inputs': inputs, 'packets': packets, 'rows': rows,
          'source_sha256': {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in
                            (Path(__file__), Path(__file__).with_name('marked_comparison.py'),
                             Path(__file__).with_name('human_baseline.py'))}}


if __name__ == '__main__':
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('--packets', type=Path, nargs='+', required=True)
  parser.add_argument('--output', type=Path, required=True)
  args = parser.parse_args()
  result = review(args.packets)
  args.output.write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')
  print(f'{len(result["rows"])} observed stops -> {args.output}')
