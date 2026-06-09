#!/usr/bin/env python3
"""Append stopping-analysis summary JSON data into the project worklog."""

from __future__ import annotations

import argparse
import json
from datetime import datetime, timezone
from pathlib import Path
from statistics import median
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
  parser = argparse.ArgumentParser(description="Append stopping analysis summary to markdown worklog")
  parser.add_argument("--summary-json", required=True, help="Path to summary.json from analyze_stopping_behavior.py")
  parser.add_argument("--worklog", default=str(DEFAULT_WORKLOG),
                      help=f"Markdown worklog path. Default: {DEFAULT_WORKLOG}")
  parser.add_argument("--title", default=None, help="Optional section title")
  parser.add_argument("--note", action="append", default=[], help="Additional note line (repeatable)")
  parser.add_argument("--dry-run", action="store_true", help="Print markdown block without writing file")
  return parser.parse_args()


def load_summary(path: Path) -> dict[str, Any]:
  data = json.loads(path.read_text())
  if not isinstance(data, dict):
    raise ValueError(f"Summary root must be JSON object: {path}")
  return data


def metric(events: list[dict[str, Any]], key: str) -> float | None:
  values = [item.get(key) for item in events]
  numbers = [float(value) for value in values if isinstance(value, (int, float))]
  if not numbers:
    return None
  return float(median(numbers))


def fmt(value: float | None, digits: int = 3) -> str:
  if value is None:
    return "n/a"
  return f"{value:.{digits}f}"


def _shadow_count(route_summary: dict[str, Any], key: str) -> Any:
  return route_summary.get(key, "n/a")


def _format_reason_counts(reason_counts: Any) -> str:
  if not isinstance(reason_counts, dict) or not reason_counts:
    return "none"
  return ", ".join(f"{reason}:{count}" for reason, count in sorted(reason_counts.items()))


def append_shadow_lines(lines: list[str], route_summary: dict[str, Any]) -> None:
  lines.append(f"- Shadow verdict: `{route_summary.get('verdict', 'unknown')}`")

  if "eligible_event_count" in route_summary:
    lines.append(
      "- Shadow eligible events covered: "
      f"`{_shadow_count(route_summary, 'eligible_events_with_shadow')}/{_shadow_count(route_summary, 'eligible_event_count')}`"
    )
    lines.append(
      "- Shadow eligible harsh events covered: "
      f"`{_shadow_count(route_summary, 'eligible_harsh_events_with_shadow')}/{_shadow_count(route_summary, 'eligible_harsh_events')}` "
      f"(`{_shadow_count(route_summary, 'eligible_harsh_events_missing_shadow')}` missing)"
    )
    lines.append(
      "- Shadow ineligible events: "
      f"`{_shadow_count(route_summary, 'ineligible_event_count')}/{_shadow_count(route_summary, 'event_count')}` "
      f"({_format_reason_counts(route_summary.get('ineligible_reason_counts'))})"
    )
    lines.append(
      "- Shadow safety/value events: "
      f"`unsafe={_shadow_count(route_summary, 'unsafe_shadow_candidate_events')}`, "
      f"`actionable={_shadow_count(route_summary, 'actionable_soften_candidates')}`, "
      f"`mixed={_shadow_count(route_summary, 'mixed_shadow_signal_events')}`, "
      f"`missed={_shadow_count(route_summary, 'missed_harsh_events')}`"
    )
    lines.append(
      "- Shadow all-event coverage: "
      f"`{_shadow_count(route_summary, 'events_with_shadow')}/{_shadow_count(route_summary, 'event_count')}`"
    )
    return

  events_with_shadow = route_summary.get("events_with_shadow", "n/a")
  shadow_event_count = route_summary.get("event_count", "n/a")
  lines.append(f"- Shadow events covered: `{events_with_shadow}/{shadow_event_count}`")
  lines.append(f"- Shadow harsh events covered: `{route_summary.get('harsh_events_with_shadow', 'n/a')}/{route_summary.get('actual_harsh_events', 'n/a')}`")
  lines.append(f"- Shadow unsafe-candidate events: `{route_summary.get('unsafe_shadow_candidate_events', 'n/a')}`")


def build_block(summary: dict[str, Any], summary_path: Path, title: str | None, notes: list[str]) -> str:
  generated = summary.get("generated_utc", datetime.now(timezone.utc).replace(microsecond=0).isoformat())
  dt = datetime.fromisoformat(str(generated).replace("Z", "+00:00"))
  date_label = dt.strftime("%Y-%m-%d")
  heading = title or f"Stopping analysis for route {summary.get('route', 'unknown')}"

  events = summary.get("events", []) if isinstance(summary.get("events", []), list) else []
  event_count = len(events)

  lines: list[str] = []
  lines.append(f"### {date_label}: {heading}")
  lines.append("")
  lines.append(f"- Host: `{summary.get('host', 'unknown')}`")
  lines.append(f"- Route: `{summary.get('route', 'unknown')}`")
  lines.append(f"- Segments analyzed: {summary.get('qlog_count', 'n/a')}")
  lines.append(f"- Detected stop events: {event_count}")
  lines.append(f"- Median duration to standstill hold: {fmt(metric(events, 'duration_s'), 2)} s")
  lines.append(f"- Median approach speed: {fmt(metric(events, 'approach_speed_mps'), 2)} m/s")
  lines.append(f"- Median entry speed: {fmt(metric(events, 'entry_speed_mps'), 2)} m/s")
  lines.append(f"- Median min aEgo: {fmt(metric(events, 'min_a_ego_mps2'), 2)} m/s²")
  lines.append(f"- Median min accel cmd: {fmt(metric(events, 'min_accel_cmd_mps2'), 2)} m/s²")
  lines.append(f"- Median shouldStop->stopping delay: {fmt(metric(events, 'should_stop_to_stopping_s'), 3)} s")
  lines.append(f"- Median creep after stop: {fmt(metric(events, 'creep_after_stop_mps'), 3)} m/s")

  settings_file = summary.get("settings_file")
  if settings_file:
    lines.append(f"- Settings snapshot: `{format_path(Path(str(settings_file)))}`")

  lines.append(f"- Analysis summary JSON: `{format_path(summary_path)}`")
  summary_md = summary_path.with_name("summary.md")
  if summary_md.exists():
    lines.append(f"- Analysis summary Markdown: `{format_path(summary_md)}`")

  shadow_summary_json = summary_path.with_name("shadow_summary.json")
  if shadow_summary_json.exists():
    try:
      shadow_summary = load_summary(shadow_summary_json)
    except Exception:
      shadow_summary = {}
    route_summary = shadow_summary.get("route_summary", {}) if isinstance(shadow_summary, dict) else {}
    if isinstance(route_summary, dict):
      append_shadow_lines(lines, route_summary)
    lines.append(f"- Shadow summary JSON: `{format_path(shadow_summary_json)}`")
    shadow_summary_md = shadow_summary_json.with_suffix(".md")
    if shadow_summary_md.exists():
      lines.append(f"- Shadow summary Markdown: `{format_path(shadow_summary_md)}`")

  if events:
    sample_event = events[0]
    graph_file = sample_event.get("graph_file")
    if isinstance(graph_file, str) and graph_file:
      lines.append(f"- Example event graph: `{format_path(summary_path.parent / graph_file)}`")

  if event_count < 3:
    lines.append("- Data quality note: low event count; collect more intentional stop scenarios for stronger comparisons.")

  for note in notes:
    lines.append(f"- Note: {note}")

  lines.append("")
  return "\n".join(lines)


def main() -> int:
  args = parse_args()
  summary_path = Path(args.summary_json).expanduser()
  if not summary_path.exists():
    raise FileNotFoundError(f"Summary JSON not found: {summary_path}")

  worklog = Path(args.worklog).expanduser()
  summary = load_summary(summary_path)
  block = build_block(summary, summary_path, args.title, args.note)

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
  print(f"[append-analysis] updated worklog: {worklog}")
  return 0


if __name__ == "__main__":
  raise SystemExit(main())
