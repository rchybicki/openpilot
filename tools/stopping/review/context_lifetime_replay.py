#!/usr/bin/env python3
"""Compare the shipped LongControl to the context-lifetime fix with recorded motion held fixed.

Both arms enable the existing recovery brake. Only longcontrol.py may differ in the runtime tree.
The off/on row names mean shipped/fixed here, not the recovery feature's off/on modes.
"""
import argparse
import hashlib
import json
import subprocess
import types
from pathlib import Path

from openpilot.selfdrive.controls.lib.longcontrol import LongControl
from openpilot.tools.stopping.review.service_replay import replay

BASE = 'ecd3fee0337f250acc075b4e53b48c8bddc46582'
LONGCONTROL = 'selfdrive/controls/lib/longcontrol.py'


if __name__ == '__main__':
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('--output', type=Path, required=True)
  parser.add_argument('rlogs', type=Path, nargs='+')
  args = parser.parse_args()
  # Loading one historical module is valid only when its runtime dependencies are unchanged.
  runtime = ['selfdrive', 'common', 'opendbc', 'frogpilot', 'cereal', 'system']
  changed = subprocess.check_output(['git', 'diff', '--name-only', BASE, '--', *runtime], text=True).splitlines()
  changed += subprocess.check_output(['git', 'ls-files', '--others', '--exclude-standard', '--', *runtime], text=True).splitlines()
  unexpected = [p for p in changed if p != LONGCONTROL and '/tests/' not in p]
  if unexpected:
    raise ValueError(f'runtime changes outside LongControl: {unexpected}')
  source = subprocess.check_output(['git', 'show', f'{BASE}:{LONGCONTROL}'])
  baseline = types.ModuleType('longcontrol_before_context_reset')
  exec(compile(source, f'{BASE}:{LONGCONTROL}', 'exec'), baseline.__dict__)
  result = replay(args.rlogs, controller_types=(baseline.LongControl, LongControl), recovery_modes=(True, True))
  result.update(comparison='context_lifetime', baseline_commit=BASE, baseline_longcontrol_sha256=hashlib.sha256(source).hexdigest(),
                runner_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
  args.output.parent.mkdir(parents=True, exist_ok=True)
  args.output.write_text(json.dumps(result, allow_nan=False) + '\n')
  print(json.dumps(result['summary'], indent=2))
