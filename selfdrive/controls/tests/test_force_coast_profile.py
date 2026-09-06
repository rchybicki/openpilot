"""cycle 52 (2026-09-05): the force-coast profile's comfort tail (frogpilot/controls/lib/force_coast.py)."""
import pytest

from openpilot.selfdrive.controls.lib import stopping_flags
from openpilot.frogpilot.controls.lib.force_coast import get_force_coast_target_accel, FORCE_COAST_ACCEL_MIN


def test_flag_off_is_todays_profile(monkeypatch):
  monkeypatch.setattr(stopping_flags, "FORCE_COAST_TERMINAL_TAPER", False)
  assert get_force_coast_target_accel(0.2, 0.2, 1.4) == pytest.approx(-0.98)
  assert get_force_coast_target_accel(1.0, 0.2, 1.4) == pytest.approx(-1.4)
  assert get_force_coast_target_accel(2.4, 0.2, 1.4) == pytest.approx(-1.68)
  assert get_force_coast_target_accel(10.0, 0.2, 1.0) == pytest.approx(-1.2)


def test_tail_bounds_a_strong_profile_only_below_the_fade(monkeypatch):
  monkeypatch.setattr(stopping_flags, "FORCE_COAST_TERMINAL_TAPER", True)
  assert get_force_coast_target_accel(0.2, 0.2, 1.4) == pytest.approx(-0.5)    # the wheel stop: was -0.98
  assert get_force_coast_target_accel(0.5, 0.2, 1.4) == pytest.approx(-0.6)    # was -1.14
  assert get_force_coast_target_accel(1.0, 0.2, 1.4) == pytest.approx(-0.8)    # was -1.4
  assert get_force_coast_target_accel(1.5, 0.2, 1.4) == pytest.approx(-1.0)    # was -1.5
  assert get_force_coast_target_accel(2.4, 0.2, 1.4) == pytest.approx(-1.68)   # the strength profile, untouched
  assert get_force_coast_target_accel(10.0, 0.2, 1.4) == pytest.approx(-1.68)
  # continuous and monotone through the fade-out (the max of two non-increasing functions)
  vs = [0.2 + 0.02 * k for k in range(150)]
  ts = [get_force_coast_target_accel(v, 0.2, 1.4) for v in vs]
  assert all(ts[k + 1] <= ts[k] + 1e-9 for k in range(len(ts) - 1))
  assert max(ts[k] - ts[k + 1] for k in range(len(ts) - 1)) < 0.06            # no step larger than the 0.02 m/s grid allows


def test_tail_never_deepens_a_weak_profile_and_scales_with_the_stop_gate(monkeypatch):
  monkeypatch.setattr(stopping_flags, "FORCE_COAST_TERMINAL_TAPER", True)
  assert get_force_coast_target_accel(0.2, 0.2, 0.5) == pytest.approx(-0.35)  # weak stays weak
  assert get_force_coast_target_accel(1.0, 0.2, 0.5) == pytest.approx(-0.5)
  assert get_force_coast_target_accel(1.0, 0.2, 2.0) == pytest.approx(-0.8)   # strong is bounded at low speed ...
  assert get_force_coast_target_accel(2.4, 0.2, 2.0) == pytest.approx(-2.4)   # ... and untouched above the fade
  assert get_force_coast_target_accel(0.5, 0.5, 1.4) == pytest.approx(-0.5)   # the gate follows vEgoStopping
  assert get_force_coast_target_accel(5.0, 0.2, 3.5) == FORCE_COAST_ACCEL_MIN
