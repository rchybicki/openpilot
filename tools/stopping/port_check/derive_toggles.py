#!/usr/bin/env python3
"""The tree's FrogPilotVariables toggles on today's device store (the initData params of route 00002310, initdata_params.json).

Run with cwd = the tree, PYTHONPATH=<tree>:<tree>/frogpilot/third_party and PARAMS_ROOT=<scratch dir> (OPENPILOT_PREFIX cannot
be used: macOS ZMQ refuses it). The tip's memory Params (on macOS under /tmp) are redirected to <PARAMS_ROOT>/memory. argv: out.json
store.json tip_tree rlog_segment_dir. The JSON store decoded binary values lossily: values with U+FFFD are skipped, and CarParams /
CarParamsPersistent are the segment's logged carParams copied by field name into this tree's schema (equiv_replay.fill). Writes the
JSON-serializable toggles, the skipped binary keys and the store keys this tree's registry does not have.
"""
import json
import os
import sys
from pathlib import Path


def logged_cp_bytes(tip, seg):
  import zstandard
  from cereal import car
  import equiv_replay
  tip_log = equiv_replay.tip_schema(tip)
  raw = zstandard.ZstdDecompressor().decompress((Path(seg) / 'rlog.zst').read_bytes(), max_output_size=int(1.5e9))
  cpm = next(e.carParams for e in tip_log.Event.read_multiple_bytes(raw) if e.which() == 'carParams')
  cp, dropped = car.CarParams.new_message(), set()
  equiv_replay.fill(cp, cpm.to_dict(), '', dropped)
  return cp.to_bytes(), sorted(dropped)


def fp_cp_bytes(tip, raw):
  """The store's FrogPilotCarParamsPersistent (tip schema) copied by field name into this tree's FrogPilotCarParams."""
  import capnp
  from cereal import custom
  import equiv_replay
  tip_custom = capnp.SchemaParser().load(str(Path(tip) / 'cereal/custom.capnp'))
  with tip_custom.FrogPilotCarParams.from_bytes(raw) as src:
    fp, dropped = custom.FrogPilotCarParams.new_message(), set()
    equiv_replay.fill(fp, src.to_dict(), '', dropped)
  return fp.to_bytes(), sorted(dropped)


def main(out, store_json, tip, seg):
  store = json.loads(Path(store_json).read_text())
  binary = sorted(k for k, v in store.items() if '\ufffd' in v)
  store = {k: v for k, v in store.items() if k not in binary}
  cp_bytes, cp_dropped = logged_cp_bytes(tip, seg)
  fp_bytes, fp_dropped = fp_cp_bytes(tip, store.pop('FrogPilotCarParamsPersistent').encode())
  d = Path(os.environ['PARAMS_ROOT']) / 'd'
  mem = Path(os.environ['PARAMS_ROOT']) / 'memory'
  d.mkdir(parents=True, exist_ok=True)
  from openpilot.common.params import Params, UnknownKeyName
  params = Params()
  unknown = []
  for k, v in store.items():
    try:
      params.check_key(k)
    except UnknownKeyName:
      unknown.append(k)
      continue
    (d / k).write_text(v)
  for k in ('CarParams', 'CarParamsPersistent'):
    (d / k).write_bytes(cp_bytes)
  for k in ('FrogPilotCarParams', 'FrogPilotCarParamsPersistent'):
    (d / k).write_bytes(fp_bytes)
  from openpilot.frogpilot.common import frogpilot_variables

  def scratch_params(*a, memory=False, **k):
    return Params(str(mem), return_defaults=True) if memory else Params(*a, **k)
  frogpilot_variables.Params = scratch_params
  fv = frogpilot_variables.FrogPilotVariables()
  fv.update(started=True)
  toggles = {}
  for k, v in vars(fv.frogpilot_toggles).items():
    try:
      json.dumps(v)
      toggles[k] = v
    except TypeError:
      toggles[k] = repr(v)
  Path(out).write_text(json.dumps(dict(toggles=toggles, unknown_keys=sorted(unknown), skipped_binary=binary, cp_dropped=cp_dropped,
                                       fp_cp_dropped=fp_dropped),
                                  indent=1, sort_keys=True))


if __name__ == "__main__":
  main(*sys.argv[1:5])
