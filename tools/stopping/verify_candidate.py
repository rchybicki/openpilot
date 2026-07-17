#!/usr/bin/env python3
"""One-command offline stopping verification against a committed baseline.

The same current replay harness evaluates both code revisions. Exact fixture/integration checks
and the no-model-change boundary are kept separate from dual-plant predictions. A green result is
only permission for an on-road measurement drive; physical rest distance and felt jerk still need
the car.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import subprocess
import sys
import tempfile
from dataclasses import asdict
from datetime import UTC, datetime
from pathlib import Path
from typing import Any

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
  sys.path.insert(0, str(REPO_ROOT))

from openpilot.tools.stopping import paired_stats as ps
from openpilot.tools.stopping.sim_replay import DEFAULT_EVENT_STORE

DEFAULT_ANALYSIS_ROOT = Path.home() / ".comma" / "stopping_behavior" / "analysis" / "offline_verify"
TEST_PATHS = (
  "selfdrive/controls/lib/tests/test_stopping_service.py",
  "selfdrive/controls/lib/tests/test_longcontrol_service_live.py",
  "tools/stopping/test_sim_replay.py",
)
MODEL_PATH_MARKERS = (
  "frogpilot/tinygrad_modeld/models/",
  "selfdrive/modeld/models/",
  "frogpilot/tinygrad_modeld/tinygrad_modeld.py",
  "frogpilot/tinygrad_modeld/parse_model_outputs.py",
  "frogpilot/common/frogpilot_variables.py",
)
METRICS = (
  "rest_gap_band_error_m",
  "minimum_gap_deficit_m",
  "end_command_jerk_mps3",
  "plant_settle_jerk_mps3",
  "hard_decel_duration_s",
)
LEAPFROG_MODEL_DELTA_MPS = 0.02


def _run(command: list[str], *, cwd: Path = REPO_ROOT, env: dict[str, str] | None = None,
         output_path: Path | None = None) -> subprocess.CompletedProcess[str]:
  result = subprocess.run(command, cwd=cwd, env=env, capture_output=True, text=True)
  if output_path is not None:
    output_path.write_text(result.stdout + result.stderr)
  return result


def _git(*args: str) -> str:
  result = _run(["git", *args])
  if result.returncode:
    raise RuntimeError(result.stderr.strip() or "git command failed")
  return result.stdout.strip()


def _sha256(path: Path) -> str:
  digest = hashlib.sha256()
  with path.open("rb") as f:
    for chunk in iter(lambda: f.read(1024 * 1024), b""):
      digest.update(chunk)
  return digest.hexdigest()


def _event_key(row: dict[str, Any]) -> tuple[str, int, int, str]:
  key = row.get("key") or {}
  return (str(key.get("route", row.get("route", ""))), int(key.get("seg", -1)),
          int(key.get("hold_mono_ns", row.get("event_id", -1))), str(row.get("plant", "")))


def _gap_band_error(value: Any) -> float | None:
  try:
    gap = float(value)
  except (TypeError, ValueError):
    return None
  return max(2.5 - gap, 0.0, gap - 5.0)


def _normalized(row: dict[str, Any]) -> dict[str, Any]:
  gap = row.get("lead_distance_hold_m")
  min_gap = row.get("minimum_lead_gap_m")
  return {
    "key": row.get("key"),
    "rest_gap_band_error_m": _gap_band_error(gap),
    "minimum_gap_deficit_m": max(2.0 - float(min_gap), 0.0) if min_gap is not None else None,
    "end_command_jerk_mps3": row.get("end_stop_cmd_jerk_mps3"),
    "plant_settle_jerk_mps3": row.get("settle_peak_meas_jerk"),
    "hard_decel_duration_s": row.get("hard_decel_duration_s"),
  }


def _hard_regressions(baseline: list[dict[str, Any]], candidate: list[dict[str, Any]]) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
  by_key = {_event_key(row): row for row in baseline}
  regressions = []
  warnings = []
  for row in candidate:
    before = by_key.get(_event_key(row))
    if before is None:
      continue
    reasons = []
    if row.get("is_leapfrog") and not before.get("is_leapfrog"):
      rebound_before = float(before.get("speed_rebound_while_should_stop_mps") or 0.0)
      rebound_after = float(row.get("speed_rebound_while_should_stop_mps") or 0.0)
      structural = any(flag in set(row.get("leapfrog_flags") or []) - set(before.get("leapfrog_flags") or [])
                       for flag in ("stop_signal_drop", "exit_stopping_state", "pre_hold_reaccel"))
      if row.get("confirmed_lead_departure"):
        warnings.append({"key": row.get("key"), "plant": row.get("plant"),
                         "reason": "new_leapfrog_flag_during_confirmed_lead_departure"})
      elif structural or rebound_after - rebound_before >= LEAPFROG_MODEL_DELTA_MPS:
        reasons.append("new_leapfrog")
      else:
        warnings.append({"key": row.get("key"), "plant": row.get("plant"),
                         "reason": "sub_materiality_leapfrog_threshold_crossing",
                         "rebound_delta_mps": rebound_after - rebound_before})
    if row.get("is_harsh") and not before.get("is_harsh"):
      reasons.append("new_harsh")
    if row.get("lead_contact") and not before.get("lead_contact"):
      reasons.append("new_lead_contact")
    if before.get("settled") and not row.get("settled"):
      reasons.append("lost_settle")
    before_gap, after_gap = before.get("minimum_lead_gap_m"), row.get("minimum_lead_gap_m")
    if after_gap is not None and float(after_gap) < 2.0 and (before_gap is None or float(before_gap) >= 2.0):
      reasons.append("new_sub_2m_gap")
    if reasons:
      regressions.append({"key": row.get("key"), "plant": row.get("plant"), "reasons": reasons})
  return regressions, warnings


def compare_reports(baseline: dict[str, Any], candidate: dict[str, Any], floor_n: int) -> dict[str, Any]:
  baseline_rows = baseline["event_rows"]
  candidate_rows = candidate["event_rows"]
  base_keys = {_event_key(row) for row in baseline_rows}
  candidate_keys = {_event_key(row) for row in candidate_rows}
  aligned = base_keys == candidate_keys
  hard, warnings = _hard_regressions(baseline_rows, candidate_rows)
  plant_reports = []
  powered_metrics = 0
  statistical_regressions = []
  for plant in sorted({str(row["plant"]) for row in baseline_rows}):
    arm_a = [row for row in baseline_rows if row["plant"] == plant]
    arm_b = [row for row in candidate_rows if row["plant"] == plant]
    pairs = ps.join_paired([_normalized(row) for row in arm_a], [_normalized(row) for row in arm_b])
    verdicts = [ps.paired_continuous_verdict(metric, pairs, floor_n) for metric in METRICS]
    powered_metrics += sum(verdict.status != "refused_insufficient_power" for verdict in verdicts)
    statistical_regressions.extend(
      {"plant": plant, "metric": verdict.metric} for verdict in verdicts if verdict.status == "regressed"
    )
    plant_reports.append({"plant": plant, "paired_events": len(pairs),
                          "metrics": [asdict(verdict) for verdict in verdicts]})
  if not aligned:
    verdict = "inconclusive"
  elif hard or statistical_regressions:
    verdict = "blocked_regression"
  elif powered_metrics == 0:
    verdict = "inconclusive"
  else:
    verdict = "safe_to_road_test"
  return {
    "verdict": verdict,
    "scenario_alignment": {"passed": aligned, "baseline_rows": len(base_keys),
                           "candidate_rows": len(candidate_keys),
                           "missing_from_candidate": len(base_keys - candidate_keys),
                           "missing_from_baseline": len(candidate_keys - base_keys)},
    "hard_regressions": hard,
    "model_threshold_warnings": warnings,
    "statistical_regressions": statistical_regressions,
    "plant_reports": plant_reports,
    "power_floor_n": floor_n,
  }


def _changed_paths(baseline_ref: str) -> list[str]:
  changed = set(_git("diff", "--name-only", baseline_ref, "--").splitlines())
  changed.update(_git("ls-files", "--others", "--exclude-standard").splitlines())
  return sorted(path for path in changed if path)


def _is_model_path(path: str) -> bool:
  return any(marker in path for marker in MODEL_PATH_MARKERS)


def _render_markdown(report: dict[str, Any]) -> str:
  exact = report["exact_checks"]
  test_status = "SKIPPED" if not exact["tests_run"] else ("PASS" if exact["tests_passed"] else "FAIL")
  lines = ["# Offline stopping verification", "", f"Verdict: **{report['verdict']}**", "",
           "## Exact checks", "",
           f"- Candidate regression tests: {test_status}",
           f"- Same scenario keys: {'PASS' if report['scenario_alignment']['passed'] else 'FAIL'}",
           f"- No driving-model files changed: {'PASS' if exact['model_boundary_passed'] else 'FAIL'}",
           "", "## Dual-plant predictions", ""]
  for plant in report["plant_reports"]:
    lines.append(f"### {plant['plant']} ({plant['paired_events']} paired events)")
    lines.append("")
    for metric in plant["metrics"]:
      delta = metric.get("median_delta")
      delta_text = "n/a" if delta is None else f"{delta:+.4f}"
      lines.append(f"- `{metric['metric']}`: {metric['status']}; n={metric['n']}; median delta={delta_text}; MDE={metric['mde_at_n']:.4f}")
    lines.append("")
  if report["hard_regressions"]:
    lines.extend(["## Hard regressions", ""])
    for item in report["hard_regressions"]:
      lines.append(f"- `{item['key']}` / {item['plant']}: {', '.join(item['reasons'])}")
    lines.append("")
  if report["model_threshold_warnings"]:
    lines.extend(["## Model-threshold warnings", ""])
    for item in report["model_threshold_warnings"]:
      lines.append(f"- `{item['key']}` / {item['plant']}: {item['reason']}")
    lines.append("")
  lines.extend(["## What remains on-road", "",
                "- Actual resting gap and sub-100 ms felt brake grab are not promotable from this plant model.",
                "- A green offline verdict means safe to road-test, not safe to promote without measured stops.", ""])
  return "\n".join(lines)


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
  parser = argparse.ArgumentParser(description="Verify current stopping code against a committed baseline")
  parser.add_argument("--baseline-ref", default="HEAD")
  parser.add_argument("--event-store", default=str(DEFAULT_EVENT_STORE))
  parser.add_argument("--route", action="append", default=[])
  parser.add_argument("--refresh-store", action="store_true", help="Ingest the selected local routes before replay")
  parser.add_argument("--max-events", type=int, default=0)
  parser.add_argument("--floor-n", type=int, default=ps.SIM_AB_MIN_PAIRED_EVENTS)
  parser.add_argument("--skip-tests", action="store_true")
  parser.add_argument("--output-dir", default=None)
  return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
  args = parse_args(argv)
  event_store = Path(args.event_store).expanduser().resolve()
  if args.refresh_store and not args.route:
    print("[verify-candidate] --refresh-store requires at least one --route", file=sys.stderr)
    return 3
  stamp = datetime.now(UTC).strftime("%Y%m%dT%H%M%SZ")
  output_dir = Path(args.output_dir).expanduser().resolve() if args.output_dir else DEFAULT_ANALYSIS_ROOT / stamp
  output_dir.mkdir(parents=True, exist_ok=True)
  python = str(REPO_ROOT / ".venv" / "bin" / "python")

  try:
    baseline_commit = _git("rev-parse", args.baseline_ref)
    if args.refresh_store:
      command = [python, "tools/stopping/build_event_store.py", "--store-dir", str(event_store),
                 "--signals-version", "2", "--telemetry-version", "2", "--accel-cmd-source", "auto"]
      for route in args.route:
        command.extend(["--route", route])
      refreshed = _run(command, output_path=output_dir / "refresh_store.txt")
      if refreshed.returncode:
        raise RuntimeError("event-store refresh failed")
    events_index = event_store / "events.jsonl"
    if not events_index.is_file():
      raise RuntimeError(f"event store missing: {events_index}")

    tests_passed: bool | None = None
    if not args.skip_tests:
      tests = _run([python, "-m", "pytest", "-q", *TEST_PATHS], output_path=output_dir / "tests.txt")
      tests_passed = tests.returncode == 0

    sim_args = ["--event-store", str(event_store), "--controller", "service", "--plant", "both"]
    if args.max_events:
      sim_args.extend(["--max-events", str(args.max_events)])
    for route in args.route:
      sim_args.extend(["--route", route])
    baseline_json = output_dir / "baseline_sim.json"
    candidate_json = output_dir / "candidate_sim.json"
    harness = str(REPO_ROOT / "tools" / "stopping" / "sim_replay.py")

    with tempfile.TemporaryDirectory(prefix="stopping-baseline-") as temp_dir:
      baseline_root = Path(temp_dir) / "repo"
      added = _run(["git", "worktree", "add", "--detach", "--quiet", str(baseline_root), baseline_commit])
      if added.returncode:
        raise RuntimeError(added.stderr.strip() or "unable to create baseline worktree")
      try:
        baseline_env = os.environ.copy()
        baseline_env["OPENPILOT_REPO_ROOT"] = str(baseline_root)
        baseline_run = _run([python, harness, *sim_args, "--output-json", str(baseline_json)],
                            env=baseline_env, output_path=output_dir / "baseline_sim.txt")
      finally:
        _run(["git", "worktree", "remove", "--force", str(baseline_root)])
      if baseline_run.returncode:
        raise RuntimeError("baseline replay failed")

    candidate_env = os.environ.copy()
    candidate_env["OPENPILOT_REPO_ROOT"] = str(REPO_ROOT)
    candidate_run = _run([python, harness, *sim_args, "--output-json", str(candidate_json)],
                         env=candidate_env, output_path=output_dir / "candidate_sim.txt")
    if candidate_run.returncode:
      raise RuntimeError("candidate replay failed")

    baseline = json.loads(baseline_json.read_text())
    candidate = json.loads(candidate_json.read_text())
    comparison = compare_reports(baseline, candidate, args.floor_n)
    changed_paths = _changed_paths(args.baseline_ref)
    model_paths = [path for path in changed_paths if _is_model_path(path)]
    comparison.update({
      "generated_utc": datetime.now(UTC).replace(microsecond=0).isoformat(),
      "baseline_ref": args.baseline_ref,
      "baseline_commit": baseline_commit,
      "candidate_head": _git("rev-parse", "HEAD"),
      "event_store": str(event_store),
      "event_store_index_sha256": _sha256(events_index),
      "routes": args.route,
      "exact_checks": {"tests_passed": tests_passed, "model_boundary_passed": not model_paths,
                       "tests_run": not args.skip_tests, "model_paths_changed": model_paths},
      "offline_limit": "safe_to_road_test is not an on-road promotion; measured rest gap and felt jerk remain required",
    })
    if tests_passed is False or model_paths:
      comparison["verdict"] = "blocked_regression"
    (output_dir / "report.json").write_text(json.dumps(comparison, indent=2, sort_keys=True) + "\n")
    (output_dir / "report.md").write_text(_render_markdown(comparison))
    print(f"[verify-candidate] verdict={comparison['verdict']} output={output_dir}")
    print(f"[verify-candidate] hard_regressions={len(comparison['hard_regressions'])} "
          + f"statistical_regressions={len(comparison['statistical_regressions'])} "
          + f"model_warnings={len(comparison['model_threshold_warnings'])}")
    return {"safe_to_road_test": 0, "blocked_regression": 1, "inconclusive": 2}[comparison["verdict"]]
  except (OSError, RuntimeError, ValueError, json.JSONDecodeError) as exc:
    print(f"[verify-candidate] environment error: {exc}", file=sys.stderr)
    return 3


if __name__ == "__main__":
  raise SystemExit(main())
