#!/usr/bin/env python3
"""Audit context-reset replay windows; command differences are not predicted motion improvements.

Select explicit bad labels plus engaged-v2 stops from the supplied frozen packets. Entry means
first shipped-arm ownership in the last 10 seconds; pulse depth covers its following second.
"""
import argparse
import hashlib
import json
from pathlib import Path

from openpilot.tools.stopping.review.context_lifetime_replay import BASE


if __name__ == '__main__':
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('--labels', type=Path, required=True)
  parser.add_argument('--output', type=Path, required=True)
  parser.add_argument('packets', type=Path, nargs='+')
  args = parser.parse_args()
  bad_ids = json.loads(args.labels.read_text())['bad_openpilot_ids']
  output, seen = [], set()
  for packet in args.packets:
    baseline = json.loads((packet / 'baseline.json').read_text())
    replay = json.loads((packet / 'context_replay.json').read_text())
    if replay['comparison'] != 'context_lifetime' or replay['route'] != baseline['route'] or replay['sources'] != baseline['files']:
      raise ValueError(f'{packet}: mismatched replay/packet provenance')
    if replay['baseline_commit'] != BASE or any(hashlib.sha256(Path(p).read_bytes()).hexdigest() != sha for p, sha in replay['source_hashes'].items()):
      raise ValueError(f'{packet}: replay runtime source binding changed')
    for source in replay['sources']:
      raw = Path(source['path']).read_bytes()
      if len(raw) != source['bytes'] or hashlib.sha256(raw).hexdigest() != source['sha256']:
        raise ValueError(f'{source["path"]}: source changed')
    for stop in baseline['stops']:
      if stop['event_id'] not in bad_ids and not (stop['v2'] and stop['v2']['cls'] == 'engaged' and stop['v2']['context_complete']):
        continue
      if stop['event_id'] in seen:
        raise ValueError(f'duplicate event: {stop["event_id"]}')
      seen.add(stop['event_id'])
      ns = stop['stop_mono_ns']
      rows = [r for r in replay['rows'] if ns - 10e9 <= r['mono_ns'] <= ns + .5e9
              and r['active'] and r['valid'] and not (r['gas'] or r['brake'])]
      owned = [r for r in rows if r['off']['owning'] and r['v'] > .05]
      if not owned:
        raise ValueError(f'{stop["event_id"]}: no valid moving-owned frames')
      entry = next(r for r in rows if r['off']['owning'])
      pulse = [r for r in rows if entry['mono_ns'] <= r['mono_ns'] <= entry['mono_ns'] + 1e9]
      delta = [r['on']['wire'] - r['off']['wire'] for r in rows]
      error = [abs(r['off']['wire'] - r['recorded_wire']) for r in owned]
      output.append(dict(event_id=stop['event_id'], segment=stop['segment'], frames=len(rows),
        changed=sum(abs(x) > 1e-6 for x in delta), delta_min=min(delta), delta_max=max(delta),
        owned_mae=sum(error) / len(error), owned_maxerr=max(error), owned_frames=len(owned),
        ownership_differences=sum(r['off']['owning'] != r['on']['owning'] for r in rows),
        max_input_age_ms={k: max(r['ages_ms'][k] for r in owned) for k in ('carState', 'radarState', 'longitudinalPlan')},
        entry_t=(entry['mono_ns'] - ns) / 1e9, entry_raw_gap=entry['gap'],
        entry_gap_old=entry['off']['context_gap'], entry_gap_new=entry['on']['context_gap'], entry_v=entry['v'],
        entry_min_old=min(r['off']['wire'] for r in pulse), entry_min_new=min(r['on']['wire'] for r in pulse)))
  if set(bad_ids) - seen:
    raise ValueError(f'missing bad labels: {sorted(set(bad_ids) - seen)}')
  args.output.write_text(json.dumps(output, indent=2, allow_nan=False) + '\n')
  print(f'{len(output)} retained stops; {sum(bool(r["changed"]) for r in output)} with changed commands')
