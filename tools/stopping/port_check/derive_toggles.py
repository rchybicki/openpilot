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


def logged_cp_bytes(tip, seg, alternative_experience=0):
  import zstandard
  from cereal import car
  import equiv_replay
  tip_log = equiv_replay.tip_schema(tip)
  raw = zstandard.ZstdDecompressor().decompress((Path(seg) / 'rlog.zst').read_bytes(), max_output_size=int(1.5e9))
  cpm = next(e.carParams for e in tip_log.Event.read_multiple_bytes(raw) if e.which() == 'carParams')
  cp, dropped = car.CarParams.new_message(), set()
  equiv_replay.fill(cp, cpm.to_dict(), '', dropped)
  cp.alternativeExperience |= alternative_experience
  return cp.to_bytes(), sorted(dropped)


def fp_alternative_experience(tip, raw):
  """The alternative-experience bits of the store's FrogPilotCarParamsPersistent (tip schema). The new base keeps them in CarParams
  (its card sets them there: always-on lateral, longitudinal active with gas), so CarParams gets them for every tree."""
  import capnp
  tip_custom = capnp.SchemaParser().load(str(Path(tip) / 'cereal/custom.capnp'))
  with tip_custom.FrogPilotCarParams.from_bytes(raw) as src:
    return int(src.alternativeExperience)


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


def seed_store(store_json, tip, seg):
  """Write the device store into <PARAMS_ROOT>/d: the keys this tree's registry has, CarParams from the segment's carParams and
  FrogPilotCarParams from the store, both copied into this tree's schema. Returns the skipped binary keys, the keys the registry lacks
  and the fields the schema copies dropped."""
  store = json.loads(Path(store_json).read_text())
  binary = sorted(k for k, v in store.items() if '\ufffd' in v)
  store = {k: v for k, v in store.items() if k not in binary}
  fp_raw = store.pop('FrogPilotCarParamsPersistent').encode()
  alternative_experience = fp_alternative_experience(tip, fp_raw)
  cp_bytes, cp_dropped = logged_cp_bytes(tip, seg, alternative_experience)
  fp_bytes, fp_dropped = fp_cp_bytes(tip, fp_raw)
  d = Path(os.environ['PARAMS_ROOT']) / 'd'
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
  return binary, sorted(unknown), cp_dropped, fp_dropped, alternative_experience


def main(out, store_json, tip, seg):
  binary, unknown, cp_dropped, fp_dropped, _ = seed_store(store_json, tip, seg)
  mem = Path(os.environ['PARAMS_ROOT']) / 'memory'
  from openpilot.common.params import Params
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
  Path(out).write_text(json.dumps(dict(toggles=toggles, unknown_keys=unknown, skipped_binary=binary, cp_dropped=cp_dropped,
                                       fp_cp_dropped=fp_dropped),
                                  indent=1, sort_keys=True))


if __name__ == "__main__":
  main(*sys.argv[1:5])
