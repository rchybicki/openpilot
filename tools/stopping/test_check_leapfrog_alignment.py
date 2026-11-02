from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path


SCRIPT_PATH = Path(__file__).resolve().with_name("check_leapfrog_alignment.py")


def run_check(args: list[str]) -> subprocess.CompletedProcess[str]:
  return subprocess.run([sys.executable, str(SCRIPT_PATH), *args], capture_output=True, text=True)


def test_alignment_reports_exact_overlap(tmp_path: Path) -> None:
  measured_json = tmp_path / "measured.json"
  predicted_json = tmp_path / "predicted.json"
  output_json = tmp_path / "alignment.json"

  measured_json.write_text(json.dumps({
    "leapfrog_event_keys": [
      {"route": "route-a", "event_id": 2},
      {"route": "route-a", "event_id": 4},
    ],
  }))
  predicted_json.write_text(json.dumps({
    "event_rows": [
      {"route": "route-a", "event_id": 4, "is_leapfrog": True},
      {"route": "route-a", "event_id": 5, "is_leapfrog": True},
      {"route": "route-a", "event_id": 1, "is_leapfrog": False},
    ],
  }))

  result = run_check([
    "--measured-json",
    str(measured_json),
    "--predicted-json",
    str(predicted_json),
    "--output-json",
    str(output_json),
  ])
  assert result.returncode == 0
  assert "status=pass" in result.stdout
  assert "overlap_events=1" in result.stdout

  payload = json.loads(output_json.read_text())
  assert payload["measured_leapfrog_events"] == 2
  assert payload["predicted_leapfrog_events"] == 2
  assert payload["overlap_events"] == 1
  assert payload["near_match_count"] == 0


def test_alignment_reports_near_match_with_tolerance(tmp_path: Path) -> None:
  measured_json = tmp_path / "measured.json"
  predicted_json = tmp_path / "predicted.json"
  output_json = tmp_path / "alignment.json"

  measured_json.write_text(json.dumps({
    "leapfrog_event_examples": [
      {"route": "route-b", "event_id": 12},
      {"route": "route-b", "event_id": 14},
    ],
  }))
  predicted_json.write_text(json.dumps({
    "event_rows": [
      {"route": "route-b", "event_id": 15, "is_leapfrog": True},
    ],
  }))

  result = run_check([
    "--measured-json",
    str(measured_json),
    "--predicted-json",
    str(predicted_json),
    "--event-id-tolerance",
    "1",
    "--output-json",
    str(output_json),
  ])
  assert result.returncode == 0

  payload = json.loads(output_json.read_text())
  assert payload["overlap_events"] == 0
  assert payload["near_match_count"] == 1
  assert payload["near_matches"][0]["measured"]["event_id"] == 14
  assert payload["near_matches"][0]["predicted"]["event_id"] == 15


def test_alignment_can_fail_on_recall_and_count_delta(tmp_path: Path) -> None:
  measured_json = tmp_path / "measured.json"
  predicted_json = tmp_path / "predicted.json"
  output_json = tmp_path / "alignment.json"

  measured_json.write_text(json.dumps({
    "leapfrog_event_keys": [
      {"route": "route-c", "event_id": 1},
      {"route": "route-c", "event_id": 2},
    ],
  }))
  predicted_json.write_text(json.dumps({
    "event_rows": [],
  }))

  result = run_check([
    "--measured-json",
    str(measured_json),
    "--predicted-json",
    str(predicted_json),
    "--min-overlap-recall",
    "0.5",
    "--max-count-delta",
    "0",
    "--output-json",
    str(output_json),
  ])
  assert result.returncode == 1
  assert "status=fail" in result.stdout
  assert "reasons=" in result.stdout

  payload = json.loads(output_json.read_text())
  assert payload["status"] == "fail"
  assert payload["overlap_events"] == 0
  assert len(payload["reasons"]) == 2
