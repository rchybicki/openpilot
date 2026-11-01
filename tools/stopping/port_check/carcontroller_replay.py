#!/usr/bin/env python3
"""Byte-level CarInterface replay for one source tree.

Feeds the recorded CAN ('can') and carControl of rlog segments through the tree's own CarInterface in card's order (CarState update
from the CAN batch, then CarController.apply with the latest carControl) and pickles, per segment, every produced frame:
(logMonoTime, sorted [(address, bus, bytes)], reported accel, cruiseState.available). Run with cwd = the tree and
PYTHONPATH=<tree>:<tree>/frogpilot/third_party, one tree per process.
"""
import pickle
import sys
from types import SimpleNamespace


class Toggles(SimpleNamespace):
  def __getattr__(self, name):
    return False


def replay(paths):
  from opendbc.car import gen_empty_fingerprint, structs
  from opendbc.car import interfaces as car_interfaces
  from opendbc.car.car_helpers import interfaces
  from openpilot.tools.lib.logreader import LogReader

  # the old tree's CarInterfaceBase opens memory Params (/dev/shm, absent on macOS) for the onroad distance button only
  if hasattr(car_interfaces, 'Params'):
    real_params = car_interfaces.Params
    car_interfaces.Params = lambda *a, memory=False, **k: SimpleNamespace(get_bool=lambda *_: False) if memory else real_params(*a, **k)

  toggles = Toggles()
  results, cp_bytes = {}, None
  for path in paths:
    msgs = list(LogReader(path))
    cps = [m for m in msgs if m.which() == 'carParams']
    if cps:
      cp_bytes = cps[0].carParams.as_builder().to_bytes()
    if cp_bytes is None:
      results[path] = None
      continue
    with structs.CarParams.from_bytes(cp_bytes) as cp_reader:
      CP = cp_reader.as_builder()
    fingerprint = gen_empty_fingerprint()
    first_can = [m for m in msgs if m.which() == 'can'][:200]
    for m in first_can:
      for c in m.can:
        if c.src < 64:
          fingerprint[c.src][c.address] = len(c.dat)
    CI_cls = interfaces[CP.carFingerprint]
    FPCP = CI_cls.get_frogpilot_params(CP.carFingerprint, fingerprint, list(CP.carFw), CP, toggles)
    CI = CI_cls(CP, FPCP)

    frames, cc = [], None
    for m in msgs:
      w = m.which()
      if w == 'carControl':
        cc = m.carControl
      elif w == 'can':
        CS, _ = CI.update([(m.logMonoTime, [(c.address, c.dat, c.src) for c in m.can])], toggles)
        if cc is None:
          continue
        actuators, sends = CI.apply(cc, m.logMonoTime, toggles)
        frames.append((m.logMonoTime, sorted((addr, bus, bytes(dat)) for addr, dat, bus in sends), float(actuators.accel),
                       bool(CS.cruiseState.available)))
    results[path] = frames
  return results


if __name__ == "__main__":
  out = sys.argv[1]
  with open(out, 'wb') as f:
    pickle.dump(replay(sys.argv[2:]), f)
