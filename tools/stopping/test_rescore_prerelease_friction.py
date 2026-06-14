"""Tests for the DEVELOPMENT-ONLY pre-release friction re-score tool. The discipline guards matter
more than the numbers: the tool must never gate, must restore the live controller constants, and the
'OFF' setting must truly inert the pre-release lane."""

from __future__ import annotations

import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
  sys.path.insert(0, str(REPO_ROOT))

from openpilot.selfdrive.controls.lib import stopping_controller as scon
from openpilot.tools.stopping import rescore_prerelease_friction as rs
from openpilot.tools.stopping import sim_replay as sr

DT = 0.1


def test_prerelease_knobs_restores_constants():
  saved = (scon.A_TERMINAL_PRERELEASE, scon.J_TERMINAL_PRERELEASE, scon.TERMINAL_PRERELEASE_RELEASE_STEP)
  with rs.prerelease_knobs(0.35, 2.0):
    assert scon.A_TERMINAL_PRERELEASE == 0.35
    assert scon.J_TERMINAL_PRERELEASE == 2.0
    # the controller derives the per-frame step from J; the override must keep them consistent
    assert scon.TERMINAL_PRERELEASE_RELEASE_STEP == 2.0 / 100.0
  assert (scon.A_TERMINAL_PRERELEASE, scon.J_TERMINAL_PRERELEASE, scon.TERMINAL_PRERELEASE_RELEASE_STEP) == saved


def test_prerelease_off_inerts_the_lane():
  # With the OFF floor the terminal_prerelease trigger must never fire across the fixtures.
  scenarios = sr.fixture_scenarios()
  friction = sr.load_friction("default")
  with rs.prerelease_knobs(rs.PRERELEASE_OFF_FLOOR, rs.J_DEPLOYED):
    summ = rs.score(scenarios, friction, DT, sr.DEFAULT_EXTEND_S)
  assert summ.triggered == 0


def test_score_summary_is_diagnostic_only():
  # The summary carries predicted-IMU stats only; it has no pass/fail / is_harsh / gate field.
  scenarios = sr.fixture_scenarios()[:5]
  friction = sr.load_friction("default")
  summ = rs.score(scenarios, friction, DT, sr.DEFAULT_EXTEND_S)
  d = summ.as_dict()
  assert set(d) == {"label", "n_settles", "n_prerelease_triggered",
                    "predicted_settle_peak_imu_jerk", "predicted_settle_peak_imu_decel"}
  assert "is_harsh" not in d and "pass" not in d and "gate" not in d
