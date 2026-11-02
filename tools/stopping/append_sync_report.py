#!/usr/bin/env python3
"""Append a route refresh report summary into the stopping behavior worklog."""

from __future__ import annotations

import argparse
import json
from collections import defaultdict
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

# New doc home (FINAL_SPEC commit 5b); run_stopping_cycle keeps passing the legacy
# worklog path explicitly until its cleanup-commit DEFAULT_WORKLOG flip.
DEFAULT_WORKLOG = Path("docs/stopping/worklog.md")
STOP_SETTING_KEYS = [
  "AdvancedLongitudinalTune",
  "LongitudinalTune",
  "HumanAcceleration",
  "ForceStops",
  "LongitudinalActuatorDelay",
  "MaxDesiredAcceleration",
]

def format_path(path: Path) -> str:
  """Prefer home-relative paths in docs for portability."""
  try:
    resolved = path.expanduser().resolve()
    home = Path.home().resolve()
    return f"~/{resolved.relative_to(home)}"
  except Exception:
    return str(path)


def parse_args() -> argparse.Namespace:
  parser = argparse.ArgumentParser(description="Append a route-refresh JSON report to the stopping markdown worklog")
  parser.add_argument("--report-file", required=True, help="Path to JSON report produced by tools/route_sync/refresh_routes.py")
  parser.add_argument("--worklog", default=str(DEFAULT_WORKLOG),
                      help=f"Path to markdown worklog. Default: {DEFAULT_WORKLOG}")
  parser.add_argument("--title", default=None, help="Optional custom section title")
  parser.add_argument("--note", action="append", default=[], help="Extra note to append (repeatable)")
  parser.add_argument("--settings-file", default=None, help="Optional JSON from device_stop_settings.py snapshot/set")
  parser.add_argument("--dry-run", action="store_true", help="Print markdown block without writing the file")
  parser.add_argument("--max-error-lines", type=int, default=3,
                      help="Maximum number of error lines to include in markdown")
  parser.add_argument("--max-list-items", type=int, default=3,
                      help="Maximum route/segment items shown per markdown line")
  return parser.parse_args()


def load_report(path: Path) -> dict[str, Any]:
  data = json.loads(path.read_text())
  if not isinstance(data, dict):
    raise ValueError("Report JSON root must be an object")
  return data


def load_optional_settings(path_raw: str | None) -> tuple[Path | None, dict[str, Any] | None]:
  if not path_raw:
    return None, None

  path = Path(path_raw).expanduser()
  if not path.exists():
    raise FileNotFoundError(f"Settings file not found: {path}")

  data = json.loads(path.read_text())
  if not isinstance(data, dict):
    raise ValueError(f"Settings JSON root must be an object: {path}")
  return path, data


def parse_timestamp(timestamp_raw: str | None) -> datetime:
  if not timestamp_raw:
    return datetime.now(timezone.utc)

  normalized = timestamp_raw.replace("Z", "+00:00")
  try:
    parsed = datetime.fromisoformat(normalized)
  except ValueError:
    return datetime.now(timezone.utc)

  if parsed.tzinfo is None:
    return parsed.replace(tzinfo=timezone.utc)
  return parsed.astimezone(timezone.utc)


def summarize_downloads(downloaded_files: list[dict[str, Any]]) -> tuple[list[str], list[str]]:
  route_to_segments: dict[str, set[str]] = defaultdict(set)
  for entry in downloaded_files:
    route = str(entry.get("route", "unknown"))
    segment = str(entry.get("segment", "unknown"))
    route_to_segments[route].add(segment)

  route_summaries: list[str] = []
  segment_summaries: list[str] = []
  for route in sorted(route_to_segments):
    segments = sorted(route_to_segments[route])
    route_summaries.append(f"`{route}` ({len(segments)} segments)")
    segment_summaries.extend(f"`{segment}`" for segment in segments)

  return route_summaries, segment_summaries


def summarize_markdown_items(items: list[str], max_items: int) -> str:
  values = sorted(set(map(str, items)))
  if not values:
    return "none"

  shown = values[:max_items]
  shown_text = ", ".join(f"`{value}`" for value in shown)
  hidden_count = len(values) - len(shown)
  if hidden_count > 0:
    return f"{len(values)} total; sample: {shown_text}; +{hidden_count} more"
  return f"{len(values)} total: {shown_text}"


def build_settings_summary(settings_data: dict[str, Any] | None) -> list[str]:
  if settings_data is None:
    return []

  values = settings_data.get("values", {})
  if not isinstance(values, dict):
    return []

  parts: list[str] = []
  for key in STOP_SETTING_KEYS:
    item = values.get(key)
    if not isinstance(item, dict):
      continue
    if "value" not in item:
      continue
    parts.append(f"{key}={item.get('value')}")

  return parts


def build_markdown_block(
  report: dict[str, Any],
  report_file: Path,
  settings_file: Path | None,
  settings_data: dict[str, Any] | None,
  title: str | None,
  notes: list[str],
  max_error_lines: int,
  max_list_items: int,
) -> str:
  timestamp = parse_timestamp(report.get("timestamp_utc"))
  date_label = timestamp.strftime("%Y-%m-%d")

  host = str(report.get("host", "unknown"))
  ssh_host = str(report.get("ssh_host", "")).strip()
  counts = report.get("counts", {}) if isinstance(report.get("counts", {}), dict) else {}
  downloaded_files = report.get("downloaded_files", []) if isinstance(report.get("downloaded_files", []), list) else []
  new_routes = report.get("new_routes", []) if isinstance(report.get("new_routes", []), list) else []
  new_segments = report.get("new_segments", []) if isinstance(report.get("new_segments", []), list) else []
  errors = report.get("errors", []) if isinstance(report.get("errors", []), list) else []

  heading = title or f"Route refresh from {host}"
  route_summaries, segment_summaries = summarize_downloads(downloaded_files)

  lines: list[str] = []
  lines.append(f"### {date_label}: {heading}")
  lines.append("")
  lines.append(f"- Host: `{host}`")
  if ssh_host and ssh_host != host:
    lines.append(f"- SSH host: `{ssh_host}` (fallback)")
  lines.append(
    f"- Sync counts: remote={counts.get('remote_files', 0)}, new={counts.get('new_files', 0)}, "
    f"changed={counts.get('changed_files', 0)}, downloaded={counts.get('downloaded', 0)}"
  )
  lines.append(
    f"- Additional counts: unchanged={counts.get('unchanged', 0)}, failures={counts.get('download_failures', 0)}, "
    f"skipped_limit={counts.get('skipped_due_to_limit', 0)}"
  )

  lines.append("- New routes detected: " + summarize_markdown_items(list(map(str, new_routes)), max_list_items))
  lines.append("- New segments detected: " + summarize_markdown_items(list(map(str, new_segments)), max_list_items))

  if route_summaries:
    visible_routes = route_summaries[:max_list_items]
    hidden_routes = len(route_summaries) - len(visible_routes)
    suffix = f" (+{hidden_routes} more)" if hidden_routes > 0 else ""
    lines.append("- Downloaded route summary: " + ", ".join(visible_routes) + suffix)
  else:
    lines.append("- Downloaded route summary: none")

  if segment_summaries:
    visible_segments = segment_summaries[:max_list_items]
    hidden_segments = len(segment_summaries) - len(visible_segments)
    suffix = f" (+{hidden_segments} more)" if hidden_segments > 0 else ""
    lines.append("- Downloaded segments: " + ", ".join(visible_segments) + suffix)

  lines.append(f"- Report JSON: `{format_path(report_file)}`")

  if settings_file is not None:
    lines.append(f"- Settings JSON: `{format_path(settings_file)}`")
    settings_summary = build_settings_summary(settings_data)
    if settings_summary:
      shown_settings = settings_summary[:max_list_items]
      hidden_settings = len(settings_summary) - len(shown_settings)
      suffix = f", ... (+{hidden_settings} more)" if hidden_settings > 0 else ""
      lines.append("- Stop settings snapshot: " + ", ".join(shown_settings) + suffix)

  if errors:
    trimmed_errors = [str(item) for item in errors[:max_error_lines]]
    lines.append("- Sync errors: " + " | ".join(trimmed_errors))

  for note in notes:
    lines.append(f"- Note: {note}")

  lines.append("")
  return "\n".join(lines)


def main() -> int:
  args = parse_args()
  report_file = Path(args.report_file).expanduser()
  worklog = Path(args.worklog).expanduser()

  if not report_file.exists():
    raise FileNotFoundError(f"Report file not found: {report_file}")

  report = load_report(report_file)
  settings_file, settings_data = load_optional_settings(args.settings_file)
  block = build_markdown_block(
    report=report,
    report_file=report_file,
    settings_file=settings_file,
    settings_data=settings_data,
    title=args.title,
    notes=args.note,
    max_error_lines=args.max_error_lines,
    max_list_items=args.max_list_items,
  )

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
  print(f"[append] updated worklog: {worklog}")
  return 0


if __name__ == "__main__":
  raise SystemExit(main())
