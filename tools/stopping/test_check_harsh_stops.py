import json
import subprocess
import sys
from pathlib import Path


SCRIPT_PATH = Path(__file__).resolve().with_name("check_harsh_stops.py")


def write_summary(path: Path, events: list[dict]) -> None:
  payload = {
    "host": "commawifi",
    "route": "test-route",
    "events": events,
  }
  path.write_text(json.dumps(payload, indent=2) + "\n")


def run_check(args: list[str]) -> subprocess.CompletedProcess[str]:
  return subprocess.run(
    [sys.executable, str(SCRIPT_PATH), *args],
    capture_output=True,
    text=True,
  )


def test_harsh_check_passes_for_smooth_events(tmp_path: Path):
  summary_path = tmp_path / "smooth_summary.json"
  write_summary(summary_path, [
    {
      "event_id": 1,
      "entry_speed_mps": 0.8,
      "end_stop_jerk_mps3": 0.35,
      "end_stop_cmd_jerk_mps3": 0.6,
      "end_stop_accel_step_mps2": 0.03,
      "min_a_ego_mps2": -0.7,
    },
    {
      "event_id": 2,
      "entry_speed_mps": 0.9,
      "end_stop_jerk_mps3": 0.42,
      "end_stop_cmd_jerk_mps3": 0.7,
      "end_stop_accel_step_mps2": 0.04,
      "min_a_ego_mps2": -0.8,
    },
  ])

  result = run_check(["--summary-json", str(summary_path), "--min-events", "1"])
  assert result.returncode == 0
  assert "status=pass" in result.stdout


def test_harsh_check_fails_for_harsh_events(tmp_path: Path):
  summary_path = tmp_path / "harsh_summary.json"
  write_summary(summary_path, [
    {
      "event_id": 1,
      "entry_speed_mps": 0.9,
      "end_stop_jerk_mps3": 1.8,
      "end_stop_cmd_jerk_mps3": 7.5,
      "end_stop_accel_step_mps2": 0.25,
      "min_a_ego_mps2": -1.6,
    },
    {
      "event_id": 2,
      "entry_speed_mps": 0.8,
      "end_stop_jerk_mps3": 0.4,
      "end_stop_cmd_jerk_mps3": 0.7,
      "end_stop_accel_step_mps2": 0.04,
      "min_a_ego_mps2": -0.8,
    },
  ])

  result = run_check(["--summary-json", str(summary_path), "--min-events", "1", "--max-harsh-rate", "0.20"])
  assert result.returncode == 1
  assert "status=fail" in result.stdout


def test_harsh_check_requires_minimum_event_count(tmp_path: Path):
  summary_path = tmp_path / "few_events_summary.json"
  write_summary(summary_path, [
    {
      "event_id": 1,
      "entry_speed_mps": 0.95,
      "end_stop_jerk_mps3": 0.45,
      "end_stop_cmd_jerk_mps3": 0.8,
      "end_stop_accel_step_mps2": 0.05,
      "min_a_ego_mps2": -0.75,
    },
  ])

  result = run_check(["--summary-json", str(summary_path), "--min-events", "3"])
  assert result.returncode == 2
  assert "status=insufficient_events" in result.stdout
