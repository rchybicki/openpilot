#!/usr/bin/env python3
"""Append stopping cycle (model + gates + benchmarks) artifacts into the project worklog."""

from __future__ import annotations

import argparse
import json
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

DEFAULT_WORKLOG = Path("docs/stopping_behavior_worklog.md")


def format_path(path: Path) -> str:
  """Prefer home-relative paths in docs for portability."""
  try:
    resolved = path.expanduser().resolve()
    home = Path.home().resolve()
    return f"~/{resolved.relative_to(home)}"
  except Exception:
    return str(path)


def parse_args() -> argparse.Namespace:
  parser = argparse.ArgumentParser(description="Append a stopping cycle report to the markdown worklog")
  parser.add_argument("--worklog", default=str(DEFAULT_WORKLOG), help=f"Markdown worklog path. Default: {DEFAULT_WORKLOG}")
  parser.add_argument("--title", default=None, help="Optional section title override")
  parser.add_argument("--host", default=None, help="Optional host label for the cycle")
  parser.add_argument("--stamp", default=None, help="Optional UTC stamp for the cycle (used for traceability only)")
  parser.add_argument("--repo-branch", default=None, help="Optional local repo branch recorded for the cycle")
  parser.add_argument("--repo-commit", default=None, help="Optional local repo commit recorded for the cycle")

  parser.add_argument("--settings-json", default=None, help="Optional device stop settings snapshot JSON path")
  parser.add_argument("--sync-report-json", default=None, help="Optional sync report JSON path")
  parser.add_argument("--analysis-summary-json", default=None, help="Optional analysis summary JSON path for this cycle")
  parser.add_argument("--fit-summary-json", action="append", default=[], help="Optional fit summary.json inputs (repeatable)")
  parser.add_argument("--gate-summary-json", action="append", default=[], help="Optional gate summary.json inputs (repeatable)")

  parser.add_argument("--model-json", default=None, help="Optional fitted model JSON path")
  parser.add_argument("--measured-gate-json", default=None, help="Optional measured harsh/leapfrog gate JSON path")
  parser.add_argument("--model-gate-json", default=None, help="Optional model harsh/leapfrog gate JSON path")
  parser.add_argument("--leapfrog-alignment-json", default=None, help="Optional leapfrog alignment JSON path")
  parser.add_argument("--variant-benchmark-json", default=None, help="Optional controller variant benchmark JSON path")

  parser.add_argument("--note", action="append", default=[], help="Additional note line (repeatable)")
  parser.add_argument("--dry-run", action="store_true", help="Print markdown block without writing the file")
  return parser.parse_args()


def load_json(path: Path) -> dict[str, Any]:
  payload = json.loads(path.read_text())
  if not isinstance(payload, dict):
    raise ValueError(f"JSON root must be object: {path}")
  return payload


def parse_generated_date(payload: dict[str, Any]) -> datetime:
  generated = payload.get("generated_utc")
  if not generated:
    return datetime.now(timezone.utc)
  try:
    parsed = datetime.fromisoformat(str(generated).replace("Z", "+00:00"))
  except ValueError:
    return datetime.now(timezone.utc)
  if parsed.tzinfo is None:
    return parsed.replace(tzinfo=timezone.utc)
  return parsed.astimezone(timezone.utc)


def summarize_model(model_path: Path) -> list[str]:
  payload = load_json(model_path)
  model = payload.get("model", {})
  if not isinstance(model, dict):
    model = {}

  delay = model.get("delay_frames")
  rows = model.get("sample_count")
  rmse = model.get("rmse")
  mae = model.get("mae")
  r2 = model.get("r2")
  windows = payload.get("windows_used")
  event_source = payload.get("event_source_filter")
  summary_files = payload.get("summary_files", [])

  details: list[str] = []
  details.append(f"- Model JSON: `{format_path(model_path)}`")
  if event_source:
    details.append(f"- Model fit event_source: `{event_source}`")
  if isinstance(windows, int):
    details.append(f"- Model fit windows_used: {windows}")
  if delay is not None:
    details.append(f"- Model fit delay_frames: {delay}")
  if rows is not None:
    details.append(f"- Model fit rows: {rows}")
  if rmse is not None and mae is not None and r2 is not None:
    details.append(f"- Model fit rmse={rmse:.4f} mae={mae:.4f} r2={r2:.4f}")
  if isinstance(summary_files, list) and summary_files:
    details.append(f"- Model fit summary inputs: {len(summary_files)} file(s)")
  return details


def summarize_gate(gate_path: Path, *, label: str) -> list[str]:
  payload = load_json(gate_path)
  status = payload.get("status", "unknown")
  events = payload.get("events_considered")
  harsh = payload.get("harsh_events")
  harsh_rate = payload.get("harsh_rate")
  leapfrog = payload.get("leapfrog_events")
  leapfrog_rate = payload.get("leapfrog_rate")
  avg_score = payload.get("avg_event_score")

  line = f"- {label}: {status}"
  if isinstance(harsh, int) and isinstance(events, int) and events > 0:
    line += f" harsh={harsh}/{events}"
  if isinstance(harsh_rate, (int, float)):
    line += f" harsh_rate={float(harsh_rate):.3f}"
  if isinstance(leapfrog, int) and isinstance(events, int) and events > 0:
    line += f" leapfrog={leapfrog}/{events}"
  if isinstance(leapfrog_rate, (int, float)):
    line += f" leapfrog_rate={float(leapfrog_rate):.3f}"
  if isinstance(avg_score, (int, float)):
    line += f" avg_score={float(avg_score):.3f}"
  return [line, f"- {label} JSON: `{format_path(gate_path)}`"]


def summarize_alignment(alignment_path: Path) -> list[str]:
  payload = load_json(alignment_path)
  status = payload.get("status", "unknown")
  measured = payload.get("measured_leapfrog_events")
  predicted = payload.get("predicted_leapfrog_events")
  overlap = payload.get("overlap_events")
  recall = payload.get("overlap_recall")
  precision = payload.get("overlap_precision")

  line = f"- Leapfrog alignment: {status}"
  if isinstance(overlap, int) and isinstance(measured, int) and isinstance(predicted, int):
    line += f" overlap={overlap} measured={measured} predicted={predicted}"
  if isinstance(recall, (int, float)):
    line += f" recall={float(recall):.3f}"
  if isinstance(precision, (int, float)):
    line += f" precision={float(precision):.3f}"
  return [line, f"- Leapfrog alignment JSON: `{format_path(alignment_path)}`"]


def summarize_benchmark(benchmark_path: Path) -> list[str]:
  payload = load_json(benchmark_path)
  events = payload.get("events_considered", 0)
  lines: list[str] = []
  lines.append(f"- Variant benchmark events: {events}")
  tracked_variants = ("current", "horizon_v1", "profile_selector", "legacy_32b8be")

  for variant in tracked_variants:
    row = payload.get(variant, {})
    if not isinstance(row, dict):
      continue
    variant_events = row.get("events", events)
    harsh = row.get("harsh_events")
    harsh_rate = row.get("harsh_rate")
    leapfrog = row.get("leapfrog_events")
    leapfrog_rate = row.get("leapfrog_rate")
    avg = row.get("avg_event_score")
    perfect = row.get("perfect_events")
    good = row.get("good_or_better_events")
    event_total = int(variant_events) if isinstance(variant_events, int) and variant_events > 0 else None
    extra = ""
    if isinstance(harsh, int) and isinstance(harsh_rate, (int, float)):
      extra += f" harsh={harsh}/{event_total} rate={float(harsh_rate):.3f}" if event_total else f" harsh={harsh} rate={float(harsh_rate):.3f}"
    if isinstance(leapfrog, int) and isinstance(leapfrog_rate, (int, float)):
      extra += f" leapfrog={leapfrog}/{event_total} rate={float(leapfrog_rate):.3f}" if event_total else f" leapfrog={leapfrog} rate={float(leapfrog_rate):.3f}"
    if isinstance(avg, (int, float)):
      extra += f" avg_score={float(avg):.3f}"
    if isinstance(perfect, int) and event_total:
      extra += f" perfect={perfect}/{event_total}"
    if isinstance(good, int) and event_total:
      extra += f" good_or_better={good}/{event_total}"
    lines.append(f"- Variant `{variant}`:{extra}")
  comparison = payload.get("comparison", {})
  if isinstance(comparison, dict):
    improved = comparison.get("profile_selector_improved_events")
    worsened = comparison.get("profile_selector_worsened_events")
    if isinstance(improved, int) or isinstance(worsened, int):
      lines.append(f"- Profile selector comparison: improved={improved if isinstance(improved, int) else 'n/a'} worsened={worsened if isinstance(worsened, int) else 'n/a'}")
  lines.append(f"- Variant benchmark JSON: `{format_path(benchmark_path)}`")
  return lines


def build_block(args: argparse.Namespace) -> str:
  date_label = datetime.now(timezone.utc).strftime("%Y-%m-%d")
  heading = args.title or "Stopping cycle results"

  lines: list[str] = []
  lines.append(f"### {date_label}: {heading}")
  lines.append("")

  if args.host:
    lines.append(f"- Host: `{args.host}`")
  if args.stamp:
    lines.append(f"- Cycle stamp: `{args.stamp}`")
  if args.repo_branch:
    lines.append(f"- Repo branch: `{args.repo_branch}`")
  if args.repo_commit:
    lines.append(f"- Repo commit: `{args.repo_commit}`")

  def maybe_path(label: str, value: str | None) -> None:
    if not value:
      return
    path = Path(value).expanduser()
    if path.exists():
      lines.append(f"- {label}: `{format_path(path)}`")

  maybe_path("Settings JSON", args.settings_json)
  maybe_path("Sync report JSON", args.sync_report_json)
  maybe_path("Analysis summary JSON", args.analysis_summary_json)

  fit_inputs = [Path(item).expanduser() for item in args.fit_summary_json if item]
  fit_inputs = [path for path in fit_inputs if path.exists()]
  if fit_inputs:
    lines.append(f"- Fit summary inputs: {len(fit_inputs)} file(s)")

  gate_inputs = [Path(item).expanduser() for item in args.gate_summary_json if item]
  gate_inputs = [path for path in gate_inputs if path.exists()]
  if gate_inputs:
    lines.append(f"- Gate summary inputs: {len(gate_inputs)} file(s)")

  model_path = Path(args.model_json).expanduser() if args.model_json else None
  model_gate_path = Path(args.model_gate_json).expanduser() if args.model_gate_json else None
  measured_gate_path = Path(args.measured_gate_json).expanduser() if args.measured_gate_json else None
  alignment_path = Path(args.leapfrog_alignment_json).expanduser() if args.leapfrog_alignment_json else None
  benchmark_path = Path(args.variant_benchmark_json).expanduser() if args.variant_benchmark_json else None

  if model_path and model_path.exists():
    lines.extend(summarize_model(model_path))

  if measured_gate_path and measured_gate_path.exists():
    lines.extend(summarize_gate(measured_gate_path, label="Measured gate"))

  if model_gate_path and model_gate_path.exists():
    lines.extend(summarize_gate(model_gate_path, label="Model gate"))

  if alignment_path and alignment_path.exists():
    lines.extend(summarize_alignment(alignment_path))

  if benchmark_path and benchmark_path.exists():
    lines.extend(summarize_benchmark(benchmark_path))

  for note in args.note:
    lines.append(f"- Note: {note}")

  lines.append("")
  return "\n".join(lines)


def main() -> int:
  args = parse_args()
  worklog = Path(args.worklog).expanduser()
  block = build_block(args)

  if args.dry_run:
    print(block)
    return 0

  if worklog.exists():
    current = worklog.read_text()
    prefix = "" if current.endswith("\n\n") else ("\n" if current.endswith("\n") else "\n\n")
  else:
    current = ""
    prefix = ""

  worklog.parent.mkdir(parents=True, exist_ok=True)
  worklog.write_text(current + prefix + block)
  print(f"[append-cycle] updated worklog: {worklog}")
  return 0


if __name__ == "__main__":
  raise SystemExit(main())
