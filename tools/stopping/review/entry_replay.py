#!/usr/bin/env python3
"""Compare entry correction with shipped 7f3bbaa; fixed motion cannot predict physical outcomes."""
import argparse
import hashlib
import json
import math
import subprocess
import sys
import types
from pathlib import Path

from openpilot.selfdrive.controls.lib.longcontrol import LongControl
from openpilot.selfdrive.controls.lib.stopping_service import StoppingService
from openpilot.tools.stopping.review.service_replay import replay

BASE = '7f3bbaa6a0a0b8d3c28ae9fab266d9200416909e'
SERVICE = 'selfdrive/controls/lib/stopping_service.py'


class Candidate(LongControl):
  service_type = StoppingService
  trace = []

  def __init__(self, cp):
    super().__init__(cp)
    self._service_shadow_svc = self.service_type()
    original = self._service_shadow_svc.update

    def capture(**kw):
      result = original(**kw)
      self.debug = {k: None if isinstance(v, float) and not math.isfinite(v) else v for k, v in result.debug.items()}
      return result

    self._service_shadow_svc.update = capture

  def update(self, *args, **kwargs):
    self.debug = {}
    wire = super().update(*args, **kwargs)
    self.trace.append(self.debug)
    return wire


if __name__ == '__main__':
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('--output', type=Path, required=True)
  parser.add_argument('rlogs', type=Path, nargs='+')
  args = parser.parse_args()
  runtime = ['selfdrive', 'common', 'opendbc', 'frogpilot', 'cereal', 'system']
  changed = subprocess.check_output(['git', 'diff', '--name-only', BASE, '--', *runtime], text=True).splitlines()
  changed += subprocess.check_output(['git', 'ls-files', '--others', '--exclude-standard', '--', *runtime], text=True).splitlines()
  unexpected = [p for p in changed if p != SERVICE and '/tests/' not in p]
  if unexpected:
    raise ValueError(f'runtime changes outside StoppingService: {unexpected}')
  source = subprocess.check_output(['git', 'show', f'{BASE}:{SERVICE}'])
  baseline = types.ModuleType('service_before_entry_correction')
  sys.modules[baseline.__name__] = baseline  # dataclasses resolve their module while loading
  exec(compile(source, f'{BASE}:{SERVICE}', 'exec'), baseline.__dict__)

  class Baseline(Candidate):
    service_type = baseline.StoppingService
    trace = []

  result = replay(args.rlogs, controller_types=(Baseline, Candidate), recovery_modes=(True, True))
  for row, old, new in zip(result['rows'], Baseline.trace, Candidate.trace, strict=True):
    row['off']['debug'], row['on']['debug'] = old, new
  result.update(comparison='entry_correction', baseline_commit=BASE, baseline_service_sha256=hashlib.sha256(source).hexdigest(),
                runner_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
  args.output.parent.mkdir(parents=True, exist_ok=True)
  args.output.write_text(json.dumps(result, allow_nan=False) + '\n')
  print(json.dumps(result['summary'], indent=2))
