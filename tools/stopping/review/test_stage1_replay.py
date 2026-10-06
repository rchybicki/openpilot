"""Gate-2 replay regression against git HEAD, using the locally saved natural stops."""
import json

import pytest

from openpilot.tools.stopping.review.plant_data import OUTPUT, natural_entries
from openpilot.tools.stopping.review.stage1_replay import replay_stop


def test_saved_natural_inputs_are_bit_identical_with_flags_off():
  paths = sorted(OUTPUT.glob('inputs*.json'))
  if not paths:
    pytest.skip('local natural-stop corpus is not installed')
  inputs = {}
  for path in paths:
    for key, value in json.loads(path.read_text()).items():
      assert key not in inputs or inputs[key] == value
      inputs[key] = value
  entries = {entry['id']: entry for entry in natural_entries()}
  for key, value in inputs.items():
    result = replay_stop(entries[key], value)
    assert result['off_bit_identical'] and result['frames'] > 0
    head_landing = result['arms']['HEAD']['wheel_stop']['u']
    # FLAT_LANDING alone never lands deeper than HEAD. 'both' now includes the creep guard, which binds from 1.3 m/s: its
    # wheel-stop value after the first binding frame is open loop (recorded motion), not a prediction.
    assert result['arms']['flat']['wheel_stop']['u'] >= head_landing - 1e-9, key
