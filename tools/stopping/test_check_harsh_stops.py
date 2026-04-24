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


def test_harsh_check_filters_by_enabled_ratio(tmp_path: Path):
  summary_path = tmp_path / "filtered_enabled_summary.json"
  write_summary(summary_path, [
    {
      "event_id": 1,
      "entry_speed_mps": 0.9,
      "enabled_ratio": 0.0,
      "end_stop_jerk_mps3": 1.8,
      "end_stop_cmd_jerk_mps3": 7.5,
      "end_stop_accel_step_mps2": 0.25,
      "min_a_ego_mps2": -1.6,
    },
    {
      "event_id": 2,
      "entry_speed_mps": 0.8,
      "enabled_ratio": 1.0,
      "end_stop_jerk_mps3": 0.4,
      "end_stop_cmd_jerk_mps3": 0.7,
      "end_stop_accel_step_mps2": 0.04,
      "min_a_ego_mps2": -0.8,
    },
  ])

  result = run_check([
    "--summary-json",
    str(summary_path),
    "--min-events",
    "1",
    "--min-enabled-ratio",
    "0.8",
    "--max-harsh-rate",
    "0.20",
  ])
  assert result.returncode == 0
  assert "status=pass" in result.stdout


def test_harsh_check_filters_by_stop_signal_ratio(tmp_path: Path):
  summary_path = tmp_path / "filtered_stop_signal_summary.json"
  write_summary(summary_path, [
    {
      "event_id": 1,
      "entry_speed_mps": 0.9,
      "enabled_ratio": 1.0,
      "stop_signal_ratio": 0.0,
      "end_stop_jerk_mps3": 1.8,
      "end_stop_cmd_jerk_mps3": 7.5,
      "end_stop_accel_step_mps2": 0.25,
      "min_a_ego_mps2": -1.6,
    },
    {
      "event_id": 2,
      "entry_speed_mps": 0.8,
      "enabled_ratio": 1.0,
      "stop_signal_ratio": 1.0,
      "end_stop_jerk_mps3": 0.4,
      "end_stop_cmd_jerk_mps3": 0.7,
      "end_stop_accel_step_mps2": 0.04,
      "min_a_ego_mps2": -0.8,
    },
  ])

  result = run_check([
    "--summary-json",
    str(summary_path),
    "--min-events",
    "1",
    "--min-enabled-ratio",
    "0.8",
    "--min-stop-signal-ratio",
    "0.5",
    "--max-harsh-rate",
    "0.20",
  ])
  assert result.returncode == 0
  assert "status=pass" in result.stdout


def test_harsh_check_filters_by_should_stop_ratio_and_real_brake_command(tmp_path: Path):
  summary_path = tmp_path / "comfort_lane_filter_summary.json"
  write_summary(summary_path, [
    {
      "event_id": 1,
      "entry_speed_mps": 2.2,
      "should_stop_ratio": 0.05,
      "stopping_state_ratio": 0.30,
      "min_accel_cmd_mps2": -0.70,
      "entry_stop_jerk_mps3": 1.5,
      "entry_stop_accel_step_mps2": 0.20,
      "end_stop_jerk_mps3": 1.8,
      "end_stop_accel_step_mps2": 0.25,
      "min_a_ego_mps2": -1.4,
    },
    {
      "event_id": 2,
      "entry_speed_mps": 2.0,
      "should_stop_ratio": 0.28,
      "stopping_state_ratio": 0.31,
      "min_accel_cmd_mps2": -0.65,
      "entry_stop_jerk_mps3": 0.40,
      "entry_stop_accel_step_mps2": 0.04,
      "end_stop_jerk_mps3": 0.35,
      "end_stop_accel_step_mps2": 0.03,
      "min_a_ego_mps2": -0.7,
    },
    {
      "event_id": 3,
      "entry_speed_mps": 2.1,
      "should_stop_ratio": 0.32,
      "stopping_state_ratio": 0.34,
      "min_accel_cmd_mps2": -0.02,
      "entry_stop_jerk_mps3": 1.6,
      "entry_stop_accel_step_mps2": 0.24,
      "end_stop_jerk_mps3": 1.4,
      "end_stop_accel_step_mps2": 0.21,
      "min_a_ego_mps2": -1.3,
    },
  ])

  result = run_check([
    "--summary-json",
    str(summary_path),
    "--min-events",
    "1",
    "--min-should-stop-ratio",
    "0.15",
    "--require-brake-command-below",
    "-0.10",
  ])
  assert result.returncode == 0
  assert "status=pass" in result.stdout
  assert "filtered" in result.stdout


def test_harsh_check_detects_entry_harshness_in_comfort_lane(tmp_path: Path):
  summary_path = tmp_path / "comfort_lane_entry_harsh_summary.json"
  write_summary(summary_path, [
    {
      "event_id": 1,
      "entry_speed_mps": 2.4,
      "should_stop_ratio": 0.30,
      "stopping_state_ratio": 0.32,
      "min_accel_cmd_mps2": -0.72,
      "entry_stop_jerk_mps3": 1.10,
      "entry_stop_accel_step_mps2": 0.16,
      "end_stop_jerk_mps3": 0.35,
      "end_stop_accel_step_mps2": 0.04,
      "min_a_ego_mps2": -0.8,
    },
  ])

  result = run_check([
    "--summary-json",
    str(summary_path),
    "--min-events",
    "1",
    "--min-should-stop-ratio",
    "0.15",
    "--require-brake-command-below",
    "-0.10",
    "--max-entry-stop-jerk",
    "0.75",
    "--max-entry-stop-accel-step",
    "0.10",
    "--max-harsh-rate",
    "0.20",
  ])
  assert result.returncode == 1
  assert "status=fail" in result.stdout
  assert "harsh_events=1" in result.stdout
  assert "entryJerk=1.1" in result.stdout or "entryJerk=1.10" in result.stdout


def test_harsh_check_regression_seed_20260424_535_event9_sustained_op_decel(tmp_path: Path):
  # Seeded from route 00000535--74f739e0f4, event 9. This was a mediocre
  # fully OP-controlled stop: no driver brake/gas, but sustained approach force.
  summary_path = tmp_path / "route_535_event9_sustained_op_decel_20260424_summary.json"
  write_summary(summary_path, [
    {
      "event_id": 9,
      "event_source": "hybrid",
      "entry_speed_mps": 8.883646011352539,
      "enabled_ratio": 1.0,
      "brake_pressed_ratio": 0.0,
      "should_stop_ratio": 0.313953488372093,
      "stopping_state_ratio": 0.32558139534883723,
      "stop_signal_ratio": 0.32558139534883723,
      "min_accel_cmd_mps2": -1.8503408432006836,
      "entry_stop_jerk_mps3": 0.1774730221007182,
      "entry_stop_accel_step_mps2": 0.0016492009162902832,
      "end_stop_jerk_mps3": 0.7493111583519371,
      "end_stop_cmd_jerk_mps3": 2.126090233333974,
      "end_stop_accel_step_mps2": 0.10784532688558102,
      "hard_decel_duration_s": 2.6028305280001405,
      "min_a_ego_mps2": -2.0287983417510986,
      "speed_rebound_while_stop_signal_mps": 0.004705887287855148,
      "speed_rebound_while_should_stop_mps": 0.004705887287855148,
      "should_stop_unexpected_accel_mps2": 0.0017531926278024912,
    },
  ])

  result = run_check([
    "--summary-json",
    str(summary_path),
    "--event-source",
    "hybrid",
    "--min-events",
    "1",
    "--min-enabled-ratio",
    "0.8",
    "--min-should-stop-ratio",
    "0.15",
    "--require-brake-command-below",
    "-0.10",
    "--max-harsh-rate",
    "0.20",
  ])
  assert result.returncode == 1
  assert "status=fail" in result.stdout
  assert "harsh_events=1" in result.stdout
  assert "hardDecel=2.6028305280001405" in result.stdout
  assert "sustained_hard_decel" in result.stdout


def test_harsh_check_detects_leapfrog_rebound_and_unexpected_accel(tmp_path: Path):
  summary_path = tmp_path / "leapfrog_detection_summary.json"
  write_summary(summary_path, [
    {
      "event_id": 1,
      "event_source": "signal",
      "entry_speed_mps": 0.72,
      "enabled_ratio": 1.0,
      "stop_signal_ratio": 1.0,
      "end_stop_jerk_mps3": 0.20,
      "end_stop_cmd_jerk_mps3": 0.0,
      "end_stop_accel_step_mps2": 0.02,
      "min_a_ego_mps2": -0.50,
      "speed_rebound_while_stop_signal_mps": 0.11,
      "speed_rebound_while_should_stop_mps": 0.13,
      "should_stop_unexpected_accel_mps2": 0.26,
    },
  ])

  result = run_check([
    "--summary-json",
    str(summary_path),
    "--min-events",
    "1",
    "--min-entry-speed",
    "0.5",
    "--min-enabled-ratio",
    "0.8",
    "--min-stop-signal-ratio",
    "0.8",
  ])
  assert result.returncode == 0
  assert "status=pass" in result.stdout
  assert "harsh_events=0" in result.stdout
  assert "leapfrog_events=1" in result.stdout
  assert "leapfrog_sample#1" in result.stdout


def test_harsh_check_can_count_stop_signal_drop_and_exit_stop_as_leapfrog(tmp_path: Path):
  summary_path = tmp_path / "comfort_lane_dropout_leapfrog_summary.json"
  write_summary(summary_path, [
    {
      "event_id": 1,
      "entry_speed_mps": 2.1,
      "should_stop_ratio": 0.32,
      "stopping_state_ratio": 0.28,
      "min_accel_cmd_mps2": -0.55,
      "stop_signal_dropped_before_hold": True,
      "left_stopping_state_before_hold": True,
      "end_stop_jerk_mps3": 0.20,
      "end_stop_accel_step_mps2": 0.02,
      "min_a_ego_mps2": -0.5,
    },
  ])

  result = run_check([
    "--summary-json",
    str(summary_path),
    "--min-events",
    "1",
    "--min-should-stop-ratio",
    "0.15",
    "--require-brake-command-below",
    "-0.10",
    "--count-stop-signal-drop-as-leapfrog",
    "--count-exit-stop-as-leapfrog",
  ])
  assert result.returncode == 0
  assert "leapfrog_events=1" in result.stdout
  assert "stop_signal_drop" in result.stdout
  assert "exit_stopping_state" in result.stdout


def test_harsh_check_can_fail_specifically_on_leapfrog_rate(tmp_path: Path):
  summary_path = tmp_path / "leapfrog_rate_gate_summary.json"
  write_summary(summary_path, [
    {
      "event_id": 1,
      "event_source": "signal",
      "entry_speed_mps": 0.72,
      "enabled_ratio": 1.0,
      "stop_signal_ratio": 1.0,
      "end_stop_jerk_mps3": 0.20,
      "end_stop_cmd_jerk_mps3": 0.0,
      "end_stop_accel_step_mps2": 0.02,
      "min_a_ego_mps2": -0.50,
      "speed_rebound_while_stop_signal_mps": 0.11,
      "speed_rebound_while_should_stop_mps": 0.13,
      "should_stop_unexpected_accel_mps2": 0.26,
    },
  ])

  result = run_check([
    "--summary-json",
    str(summary_path),
    "--min-events",
    "1",
    "--min-entry-speed",
    "0.5",
    "--min-enabled-ratio",
    "0.8",
    "--min-stop-signal-ratio",
    "0.8",
    "--max-harsh-rate",
    "1.0",
    "--max-end-stop-jerk",
    "2.0",
    "--max-end-stop-accel-step",
    "0.3",
    "--min-a-ego-floor",
    "-2.5",
    "--max-leapfrog-rate",
    "0.20",
  ])
  assert result.returncode == 1
  assert "status=fail" in result.stdout
  assert "harsh_events=0" in result.stdout
  assert "leapfrog_events=1" in result.stdout
  assert "reasons=leapfrog_rate=" in result.stdout


def test_harsh_check_regression_seed_20260212_f1_event3_leapfrog_flags_comfort_gate_failure(tmp_path: Path):
  # Seeded from route_000006f1--1eeed096b0 (review batch 20260212T160050Z), event 3.
  # Intentionally expected to fail until leapfrogging is reduced.
  summary_path = tmp_path / "route_f1_event3_leapfrog_20260212_summary.json"
  write_summary(summary_path, [
    {
      "event_id": 3,
      "event_source": "signal",
      "entry_speed_mps": 1.13,
      "enabled_ratio": 1.0,
      "stop_signal_ratio": 1.0,
      "end_stop_jerk_mps3": 0.9719370595854989,
      "end_stop_cmd_jerk_mps3": 0.0,
      "end_stop_accel_step_mps2": 0.08139448426663876,
      "min_a_ego_mps2": -0.5896463990211487,
      "speed_rebound_while_stop_signal_mps": 0.116,
      "speed_rebound_while_should_stop_mps": 0.116,
      "should_stop_unexpected_accel_mps2": 0.3203718662261963,
    },
  ])

  result = run_check([
    "--summary-json",
    str(summary_path),
    "--event-source",
    "signal",
    "--min-events",
    "1",
    "--min-entry-speed",
    "0.5",
    "--min-enabled-ratio",
    "0.8",
    "--min-stop-signal-ratio",
    "0.8",
    "--max-harsh-rate",
    "0.20",
    "--max-end-stop-jerk",
    "2.0",
    "--max-end-stop-accel-step",
    "0.3",
    "--min-a-ego-floor",
    "-2.5",
    "--max-leapfrog-rate",
    "0.20",
  ])
  assert result.returncode == 1
  assert "status=fail" in result.stdout
  assert "leapfrog_events=1" in result.stdout
  assert "reasons=leapfrog_rate=" in result.stdout


def test_harsh_check_regression_seed_20260212_f2_event3_leapfrog_flags_comfort_gate_failure(tmp_path: Path):
  # Seeded from route_000006f2--ef82b286ad (review batch 20260212T160050Z), event 3.
  # Intentionally expected to fail until leapfrogging is reduced.
  summary_path = tmp_path / "route_f2_event3_leapfrog_20260212_summary.json"
  write_summary(summary_path, [
    {
      "event_id": 3,
      "event_source": "signal",
      "entry_speed_mps": 0.56,
      "enabled_ratio": 1.0,
      "stop_signal_ratio": 1.0,
      "end_stop_jerk_mps3": 0.48,
      "end_stop_cmd_jerk_mps3": 0.0,
      "end_stop_accel_step_mps2": 0.03,
      "min_a_ego_mps2": -0.74,
      "speed_rebound_while_stop_signal_mps": 0.102,
      "speed_rebound_while_should_stop_mps": 0.102,
      "should_stop_unexpected_accel_mps2": 0.33539628982543945,
    },
  ])

  result = run_check([
    "--summary-json",
    str(summary_path),
    "--event-source",
    "signal",
    "--min-events",
    "1",
    "--min-entry-speed",
    "0.5",
    "--min-enabled-ratio",
    "0.8",
    "--min-stop-signal-ratio",
    "0.8",
    "--max-harsh-rate",
    "0.20",
    "--max-end-stop-jerk",
    "2.0",
    "--max-end-stop-accel-step",
    "0.3",
    "--min-a-ego-floor",
    "-2.5",
    "--max-leapfrog-rate",
    "0.20",
  ])
  assert result.returncode == 1
  assert "status=fail" in result.stdout
  assert "leapfrog_events=1" in result.stdout
  assert "reasons=leapfrog_rate=" in result.stdout


def test_harsh_check_regression_seed_20260212_680_event5_leapfrog_flags_comfort_gate_failure(tmp_path: Path):
  # Seeded from route_00000680--76fa5738aa (review batch 20260212), event 5.
  # Intentionally expected to fail until leapfrogging is reduced.
  summary_path = tmp_path / "route_680_event5_leapfrog_20260212_summary.json"
  write_summary(summary_path, [
    {
      "event_id": 5,
      "event_source": "signal",
      "entry_speed_mps": 0.8933415412902832,
      "enabled_ratio": 1.0,
      "stop_signal_ratio": 1.0,
      "end_stop_jerk_mps3": 0.7376376240160287,
      "end_stop_cmd_jerk_mps3": 0.0,
      "end_stop_accel_step_mps2": 0.046877965331077576,
      "min_a_ego_mps2": -0.5646573305130005,
      "speed_rebound_while_stop_signal_mps": 0.1851629763841629,
      "speed_rebound_while_should_stop_mps": 0.1851629763841629,
      "should_stop_unexpected_accel_mps2": 0.5091063976287842,
    },
  ])

  result = run_check([
    "--summary-json",
    str(summary_path),
    "--event-source",
    "signal",
    "--min-events",
    "1",
    "--min-entry-speed",
    "0.5",
    "--min-enabled-ratio",
    "0.8",
    "--min-stop-signal-ratio",
    "0.8",
    "--max-harsh-rate",
    "0.20",
    "--max-end-stop-jerk",
    "2.0",
    "--max-end-stop-accel-step",
    "0.3",
    "--min-a-ego-floor",
    "-2.5",
    "--max-leapfrog-rate",
    "0.20",
  ])
  assert result.returncode == 1
  assert "status=fail" in result.stdout
  assert "leapfrog_events=1" in result.stdout
  assert "reasons=leapfrog_rate=" in result.stdout


def test_harsh_check_regression_seed_20260212_67d_event1_leapfrog_flags_comfort_gate_failure(tmp_path: Path):
  # Seeded from route_0000067d--071364d48b (review batch 20260212), event 1.
  # Intentionally expected to fail until leapfrogging is reduced.
  summary_path = tmp_path / "route_67d_event1_leapfrog_20260212_summary.json"
  write_summary(summary_path, [
    {
      "event_id": 1,
      "event_source": "signal",
      "entry_speed_mps": 0.8760672211647034,
      "enabled_ratio": 1.0,
      "stop_signal_ratio": 1.0,
      "end_stop_jerk_mps3": 0.563547429920325,
      "end_stop_cmd_jerk_mps3": 0.0,
      "end_stop_accel_step_mps2": 0.09040044496456781,
      "min_a_ego_mps2": -0.5531947016716003,
      "speed_rebound_while_stop_signal_mps": 0.24162538722157478,
      "speed_rebound_while_should_stop_mps": 0.24162538722157478,
      "should_stop_unexpected_accel_mps2": 0.6550189852714539,
    },
  ])

  result = run_check([
    "--summary-json",
    str(summary_path),
    "--event-source",
    "signal",
    "--min-events",
    "1",
    "--min-entry-speed",
    "0.5",
    "--min-enabled-ratio",
    "0.8",
    "--min-stop-signal-ratio",
    "0.8",
    "--max-harsh-rate",
    "0.20",
    "--max-end-stop-jerk",
    "2.0",
    "--max-end-stop-accel-step",
    "0.3",
    "--min-a-ego-floor",
    "-2.5",
    "--max-leapfrog-rate",
    "0.20",
  ])
  assert result.returncode == 1
  assert "status=fail" in result.stdout
  assert "leapfrog_events=1" in result.stdout
  assert "reasons=leapfrog_rate=" in result.stdout
