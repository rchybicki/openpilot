from __future__ import annotations

from openpilot.tools.stopping import verify_candidate as vc


def _row(i: int, plant: str, **updates):
  row = {
    "key": {"route": "r", "seg": 1, "hold_mono_ns": i},
    "plant": plant,
    "settled": True,
    "is_harsh": False,
    "is_leapfrog": False,
    "lead_contact": False,
    "speed_rebound_while_should_stop_mps": 0.0,
    "confirmed_lead_departure": False,
    "lead_distance_hold_m": 3.5,
    "minimum_lead_gap_m": 3.0,
    "end_stop_cmd_jerk_mps3": 0.5,
    "settle_peak_meas_jerk": 1.0,
    "hard_decel_duration_s": 0.0,
  }
  row.update(updates)
  return row


def _reports(n: int = 3):
  rows = [_row(i, plant) for plant in ("ref", "refit") for i in range(n)]
  return {"event_rows": rows}, {"event_rows": [dict(row) for row in rows]}


def test_identical_powered_reports_are_safe_to_road_test():
  baseline, candidate = _reports()
  report = vc.compare_reports(baseline, candidate, floor_n=2)
  assert report["verdict"] == "safe_to_road_test"
  assert report["hard_regressions"] == []


def test_new_leapfrog_blocks_even_below_power_floor():
  baseline, candidate = _reports(n=1)
  candidate["event_rows"][0]["is_leapfrog"] = True
  candidate["event_rows"][0]["speed_rebound_while_should_stop_mps"] = 0.1
  report = vc.compare_reports(baseline, candidate, floor_n=200)
  assert report["verdict"] == "blocked_regression"
  assert report["hard_regressions"][0]["reasons"] == ["new_leapfrog"]


def test_clean_small_deck_is_inconclusive():
  baseline, candidate = _reports(n=1)
  assert vc.compare_reports(baseline, candidate, floor_n=200)["verdict"] == "inconclusive"


def test_model_path_boundary_is_narrow():
  assert vc._is_model_path("frogpilot/tinygrad_modeld/models/driving.onnx")
  assert vc._is_model_path("frogpilot/tinygrad_modeld/parse_model_outputs.py")
  assert not vc._is_model_path("selfdrive/controls/lib/stopping_service.py")
