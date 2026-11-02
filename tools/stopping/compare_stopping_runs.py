#!/usr/bin/env python3
"""Compare two stopping analysis summaries to evaluate tuning/algorithm changes."""

from __future__ import annotations

import argparse
import json
from datetime import datetime, timezone
from pathlib import Path
from statistics import median
from typing import Any


def utc_now_iso() -> str:
  return datetime.now(timezone.utc).replace(microsecond=0).isoformat()


def parse_args() -> argparse.Namespace:
  parser = argparse.ArgumentParser(description="Compare two stopping analysis summary.json files")
  parser.add_argument("--before", required=True, help="Baseline summary.json")
  parser.add_argument("--after", required=True, help="Candidate summary.json")
  parser.add_argument("--output", default=None, help="Optional output markdown file")
  parser.add_argument("--title", default="Stopping Analysis Comparison", help="Markdown title")
  return parser.parse_args()


def load_summary(path: Path) -> dict[str, Any]:
  data = json.loads(path.read_text())
  if not isinstance(data, dict):
    raise ValueError(f"Expected JSON object in {path}")
  return data


def median_metric(events: list[dict[str, Any]], key: str) -> float | None:
  values = [item.get(key) for item in events]
  filtered = [float(value) for value in values if isinstance(value, (int, float))]
  if not filtered:
    return None
  return float(median(filtered))


def fmt(value: float | None, digits: int = 3) -> str:
  if value is None:
    return "n/a"
  return f"{value:.{digits}f}"


def delta(after: float | None, before: float | None) -> float | None:
  if after is None or before is None:
    return None
  return after - before


def delta_str(after: float | None, before: float | None, digits: int = 3) -> str:
  d = delta(after, before)
  if d is None:
    return "n/a"
  sign = "+" if d >= 0 else ""
  return f"{sign}{d:.{digits}f}"


def build_markdown(title: str, before_path: Path, after_path: Path, before: dict[str, Any], after: dict[str, Any]) -> str:
  before_events = before.get("events", []) if isinstance(before.get("events", []), list) else []
  after_events = after.get("events", []) if isinstance(after.get("events", []), list) else []

  metrics = [
    ("event_count", len(before_events), len(after_events), "count"),
    ("median_duration_s", median_metric(before_events, "duration_s"), median_metric(after_events, "duration_s"), "s"),
    ("median_approach_speed_mps", median_metric(before_events, "approach_speed_mps"), median_metric(after_events, "approach_speed_mps"), "m/s"),
    ("median_entry_speed_mps", median_metric(before_events, "entry_speed_mps"), median_metric(after_events, "entry_speed_mps"), "m/s"),
    ("median_min_a_ego_mps2", median_metric(before_events, "min_a_ego_mps2"), median_metric(after_events, "min_a_ego_mps2"), "m/s²"),
    ("median_min_accel_cmd_mps2", median_metric(before_events, "min_accel_cmd_mps2"), median_metric(after_events, "min_accel_cmd_mps2"), "m/s²"),
    ("median_should_to_stopping_s", median_metric(before_events, "should_stop_to_stopping_s"), median_metric(after_events, "should_stop_to_stopping_s"), "s"),
    ("median_creep_after_stop_mps", median_metric(before_events, "creep_after_stop_mps"), median_metric(after_events, "creep_after_stop_mps"), "m/s"),
    ("median_distance_traveled_m", median_metric(before_events, "distance_traveled_m"), median_metric(after_events, "distance_traveled_m"), "m"),
  ]

  lines: list[str] = []
  lines.append(f"# {title}")
  lines.append("")
  lines.append(f"- Generated (UTC): {utc_now_iso()}")
  lines.append(f"- Before: `{before_path}`")
  lines.append(f"- After: `{after_path}`")
  lines.append(f"- Before route: `{before.get('route', 'unknown')}` | After route: `{after.get('route', 'unknown')}`")
  lines.append("")
  lines.append("|Metric|Before|After|Delta (After-Before)|Unit|")
  lines.append("|---|---:|---:|---:|---|")

  for name, before_value, after_value, unit in metrics:
    if isinstance(before_value, int) and isinstance(after_value, int):
      delta_value = after_value - before_value
      delta_formatted = f"{delta_value:+d}"
      lines.append(f"|{name}|{before_value}|{after_value}|{delta_formatted}|{unit}|")
      continue

    lines.append(
      f"|{name}|{fmt(before_value)}|{fmt(after_value)}|{delta_str(after_value, before_value)}|{unit}|"
    )

  lines.append("")
  lines.append("## Interpretation Hints")
  lines.append("")
  lines.append("- `median_min_a_ego_mps2`: values closer to `0` are generally less harsh braking.")
  lines.append("- `median_creep_after_stop_mps`: lower is generally better for holding steady stops.")
  lines.append("- Compare event plots in each run for context before deciding better/worse.")
  lines.append("")
  return "\n".join(lines)


def main() -> int:
  args = parse_args()
  before_path = Path(args.before).expanduser()
  after_path = Path(args.after).expanduser()
  if not before_path.exists():
    raise FileNotFoundError(f"Before summary not found: {before_path}")
  if not after_path.exists():
    raise FileNotFoundError(f"After summary not found: {after_path}")

  before = load_summary(before_path)
  after = load_summary(after_path)
  markdown = build_markdown(args.title, before_path, after_path, before, after)

  if args.output:
    output_path = Path(args.output).expanduser()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(markdown + "\n")
    print(f"[compare] output={output_path}")
  else:
    print(markdown)

  return 0


if __name__ == "__main__":
  raise SystemExit(main())
