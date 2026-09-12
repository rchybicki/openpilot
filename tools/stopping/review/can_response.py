#!/usr/bin/env python3
"""Extract classic Hyundai CAN requests and sensor proxies from a hashed route packet.

Input is bookmarked_baseline.py's signals.json. Each CAN frame is retained, including
frames batched at the same log timestamp. An echo proves transmission, not ECU response.
Sensor names/units come from the DBC; their physical calibration is not established here.
"""
import argparse
import hashlib
import inspect
import json
from pathlib import Path

import numpy as np
import zstandard
from opendbc.can import CANParser
from opendbc.can.dbc import DBC_PATH

from openpilot.tools.stopping.review.triage_one import LOG


FIELDS = {
  1057: ('aReqValue', 'aReqRaw', 'ACCMode', 'StopReq', 'CR_VSM_Alive'),
  905: ('JerkUpperLimit', 'JerkLowerLimit', 'ACCMode'),
  544: ('LONG_ACCEL', 'LONG_ACCEL_STAT', 'LONG_ACCEL_DIAG', 'CYL_PRES', 'CYL_PRES_STAT', 'CYL_PRESS_DIAG', 'ESP12_AliveCounter'),
  916: ('ACCEL_REF_ACC', 'DriverOverride'),
}
STREAMS = {'send': ('sendcan', 0, (1057, 905)), 'echo': ('can', 128, (1057, 905)), 'sensor': ('can', 0, (544, 916))}


def extract(packet_path, output):
  packet_bytes = packet_path.read_bytes()
  packet = json.loads(packet_bytes)
  sources = packet['files']
  paths = [Path(r['path']) for r in sources]
  if not paths or len(set(paths)) != len(paths) or {p.parent.name.rsplit('--', 1)[0] for p in paths} != {packet['route']}:
    raise ValueError('packet must contain unique files from one route')
  parsers = {name: CANParser('hyundai_kia_generic', [(addr, float('nan')) for addr in addresses], bus)
             for name, (_, bus, addresses) in STREAMS.items()}
  rows = {f'{name}_{addr}': [] for name, (_, _, addresses) in STREAMS.items() for addr in addresses}
  for record, path in zip(sources, paths, strict=True):
    raw = path.read_bytes()
    if len(raw) != record['bytes'] or hashlib.sha256(raw).hexdigest() != record['sha256']:
      raise ValueError(f'source mismatch: {path}')
    decoder = zstandard.ZstdDecompressor().decompressobj()
    payload = decoder.decompress(raw)
    if not decoder.eof or decoder.unused_data:
      raise ValueError(f'incomplete or multi-frame log: {path}')
    for ev in LOG.Event.read_multiple_bytes(payload):
      which = ev.which()
      if which not in ('can', 'sendcan'):
        continue
      for frame in getattr(ev, which):
        for name, (kind, bus, addresses) in STREAMS.items():
          if which != kind or frame.src != bus or frame.address not in addresses:
            continue
          address, ns = frame.address, int(ev.logMonoTime)
          dest = rows[f'{name}_{address}']
          if dest and ns < dest[-1][0]:
            raise ValueError(f'{name}_{address}: decreasing log timestamp')
          parser = parsers[name]
          # CANParser accepts short payloads, padding absent bits with zero.
          # Do not mistake a truncated frame for a valid brake request.
          updated = parser.update([[ns, [(address, bytes(frame.dat), bus)]]]) if len(frame.dat) == parser.message_states[address].size else set()
          valid = bool(ev.valid and address in updated)
          values = [float(parser.vl[address][key]) for key in FIELDS[address]] if valid else [float('nan')] * len(FIELDS[address])
          dest.append((ns, valid, values))

  arrays, summary = {}, {}
  for key, records in rows.items():
    width = len(FIELDS[int(key.rsplit('_', 1)[1])])
    ns = np.array([r[0] for r in records], dtype=np.int64)
    valid = np.array([r[1] for r in records], dtype=bool)
    arrays.update({f'{key}_ns': ns, f'{key}_valid': valid,
                   f'{key}_values': np.array([r[2] for r in records], dtype=float).reshape(-1, width)})
    summary[key] = {'rows': len(records), 'invalid': int(np.sum(~valid)),
                    'equal_timestamps': int(np.sum(np.diff(ns) == 0)),
                    'max_log_gap_s': float(np.max(np.diff(ns)) / 1e9) if len(ns) > 1 else None}
  manifest = {'route': packet['route'], 'packet': str(packet_path), 'packet_sha256': hashlib.sha256(packet_bytes).hexdigest(),
              'sources': sources, 'fields': FIELDS, 'streams': summary,
              'source_sha256': {str(p): hashlib.sha256(p.read_bytes()).hexdigest()
                                for p in (Path(__file__), Path(inspect.getfile(CANParser)), Path(DBC_PATH) / 'hyundai_kia_generic.dbc')},
              'limits': ['Log timestamps are batch timestamps; equal timestamps are preserved.',
                         'Valid means log validity, payload length and decoder acceptance, not independent ECU acknowledgment.',
                         'No independent checksum or alive-counter integrity check; the selected DBC messages have no such validators.',
                         'Segment gaps are retained; downstream episode admission must check continuity and source freshness.',
                         'DBC sensor status bits are retained without assigning unverified health or calibration semantics.']}
  # A frozen packet is never overwritten; validation finishes before creating the destination.
  output.mkdir(parents=True, exist_ok=False)
  np.savez_compressed(output / 'can.npz', **arrays)
  manifest['can_sha256'] = hashlib.sha256((output / 'can.npz').read_bytes()).hexdigest()
  (output / 'manifest.json').write_text(json.dumps(manifest, indent=2, allow_nan=False) + '\n')
  return manifest


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('packet', type=Path)
  parser.add_argument('--output', type=Path, required=True, help='New destination directory')
  args = parser.parse_args()
  manifest = extract(args.packet, args.output)
  print(json.dumps({'route': manifest['route'], 'streams': manifest['streams']}))


if __name__ == '__main__':
  main()
