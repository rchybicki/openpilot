"""The DELIVERED_LANDING and LANDING_LEAD_TOLERANCE suites re-run with the tolerance on, in every GAP_AWARE_TERMINAL x ENTRY_COMFORT_CAP
combination (the car's live flags). With the strict latch confirmed the tolerance is never consulted, so the landing suite must pass
unchanged. The two landing tests that build the guard with test_stopping_stage1.arm() are skipped with GAT on (test_gap_aware_landing_combo.py)."""
import pytest

from openpilot.selfdrive.controls.lib import stopping_flags
from openpilot.selfdrive.controls.tests.test_delivered_landing import *  # noqa: F403  (the landing suite, collected here again)
from openpilot.selfdrive.controls.tests.test_landing_lead_tolerance import *  # noqa: F403  (the tolerance suite)
from openpilot.selfdrive.controls.tests import test_delivered_landing as _landing

GAT_ARM_TESTS = ('test_creep_guard_shifts_by_the_ease_only_below_v_land', 'test_live_fcw_stops_the_landing_ease')
assert all(hasattr(_landing, n) for n in GAT_ARM_TESTS)


@pytest.fixture(autouse=True, params=[(g, e) for g in (False, True) for e in (False, True)], ids=lambda p: f'gat{int(p[0])}-entry{int(p[1])}')
def combo(request, monkeypatch):
  gat, entry = request.param
  if gat and request.node.originalname in GAT_ARM_TESTS:
    pytest.skip('the guard arm this flag delays by design (pinned in test_gap_aware_terminal.py)')
  monkeypatch.setattr(stopping_flags, 'GAP_AWARE_TERMINAL', gat)
  monkeypatch.setattr(stopping_flags, 'ENTRY_COMFORT_CAP', entry)
  monkeypatch.setattr(stopping_flags, 'LANDING_LEAD_TOLERANCE', True)
