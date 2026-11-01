"""The DELIVERED_LANDING suite re-run with GAP_AWARE_TERMINAL on (the car with both flags). Two tests are left out: they build the guard with
test_stopping_stage1.arm() (150 frames at 1.2 m/s, gap 10 m) = the guard arm this flag delays by design; the both-flags guard / landing
order is pinned in test_gap_aware_terminal.py instead."""
import pytest

from openpilot.selfdrive.controls.lib import stopping_flags
from openpilot.selfdrive.controls.tests.test_delivered_landing import *  # noqa: F403  (the landing suite, collected here again)
from openpilot.selfdrive.controls.tests import test_delivered_landing as _landing

del test_creep_guard_shifts_by_the_ease_only_below_v_land, test_live_fcw_stops_the_landing_ease  # noqa: F821
assert hasattr(_landing, 'flags')


@pytest.fixture(autouse=True)
def gap_aware(monkeypatch):
  monkeypatch.setattr(stopping_flags, 'GAP_AWARE_TERMINAL', True)
