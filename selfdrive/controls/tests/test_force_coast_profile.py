"""cycle 52 (2026-09-05): the force-coast profile's terminal taper (frogpilot/controls/lib/force_coast.py)."""
import pytest

from openpilot.selfdrive.controls.lib import stopping_flags
from openpilot.frogpilot.controls.lib.force_coast import get_force_coast_target_accel, FORCE_COAST_ACCEL_MIN


def test_flag_off_is_todays_profile(monkeypatch):
  monkeypatch.setattr(stopping_flags, "FORCE_COAST_TERMINAL_TAPER", False)
  assert get_force_coast_target_accel(0.2, 0.2, 1.4) == pytest.approx(-0.98)
  assert get_force_coast_target_accel(0.5, 0.2, 1.4) == pytest.approx(-0.7 * 1.4 + (-1.0 + 0.7) * 1.4 * (0.3 / 0.8))
  assert get_force_coast_target_accel(1.0, 0.2, 1.4) == pytest.approx(-1.4)
  assert get_force_coast_target_accel(2.4, 0.2, 1.4) == pytest.approx(-1.68)
  assert get_force_coast_target_accel(10.0, 0.2, 1.0) == pytest.approx(-1.2)


def test_taper_eases_only_below_one_metre_per_second(monkeypatch):
  monkeypatch.setattr(stopping_flags, "FORCE_COAST_TERMINAL_TAPER", True)
  assert get_force_coast_target_accel(0.2, 0.2, 1.4) == pytest.approx(-0.70)     # the wheel stop: was -0.98 (= the service hold)
  assert get_force_coast_target_accel(0.5, 0.2, 1.4) == pytest.approx(-0.84)     # the knee: was -1.14
  assert get_force_coast_target_accel(1.0, 0.2, 1.4) == pytest.approx(-1.4)      # unchanged from here up
  assert get_force_coast_target_accel(2.4, 0.2, 1.4) == pytest.approx(-1.68)
  assert get_force_coast_target_accel(10.0, 0.2, 1.4) == pytest.approx(-1.68)
  # monotone: the target only deepens with speed
  vs = [0.2 + 0.05 * k for k in range(60)]
  ts = [get_force_coast_target_accel(v, 0.2, 1.4) for v in vs]
  assert all(ts[k + 1] <= ts[k] + 1e-9 for k in range(len(ts) - 1))
  # the stop gate follows the toggle; strength scales; the floor clip stands
  assert get_force_coast_target_accel(0.3, 0.3, 1.0) == pytest.approx(-0.50)
  assert get_force_coast_target_accel(5.0, 0.2, 3.5) == FORCE_COAST_ACCEL_MIN
