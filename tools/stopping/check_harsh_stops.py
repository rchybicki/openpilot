#!/usr/bin/env python3
"""Gate stopping comfort regressions from stop-event summary JSON files."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any


def parse_args() -> argparse.Namespace:
  parser = argparse.ArgumentParser(description="Check harsh-stop metrics from analyze_stopping_behavior summary.json files")
  parser.add_argument("--summary-json", action="append", required=True,
                      help="Path to summary.json from analyze_stopping_behavior.py (repeatable)")
  parser.add_argument("--min-events", type=int, default=4, help="Minimum event count required to evaluate")
  parser.add_argument("--min-entry-speed", type=float, default=0.20, help="Ignore events below this entry speed (m/s)")
  parser.add_argument("--max-harsh-rate", type=float, default=0.20, help="Maximum allowed harsh-event rate [0..1]")
  parser.add_argument("--max-harsh-count", type=int, default=0, help="Maximum allowed harsh-event count (0 = disabled)")
  parser.add_argument("--max-end-stop-jerk", type=float, default=0.75, help="Harsh threshold for end_stop_jerk_mps3")
  parser.add_argument("--max-end-stop-cmd-jerk", type=float, default=3.0, help="Harsh threshold for end_stop_cmd_jerk_mps3")
  parser.add_argument("--max-end-stop-accel-step", type=float, default=0.08, help="Harsh threshold for end_stop_accel_step_mps2")
  parser.add_argument("--min-a-ego-floor", type=float, default=-1.05, help="Harsh threshold for min_a_ego_mps2 (more negative is harsher)")
  parser.add_argument("--output-json", default=None, help="Optional path to write machine-readable check output")
  return parser.parse_args()


def as_float(value: Any) -> float | None:
  if value is None:
    return None
  try:
    return float(value)
  except (TypeError, ValueError):
    return None


def load_events(path: Path) -> list[dict[str, Any]]:
  data = json.loads(path.read_text())
  events = data.get("events", [])
  if not isinstance(events, list):
    return []
  route = str(data.get("route", "unknown"))
  tagged_events: list[dict[str, Any]] = []
  for event in events:
    if not isinstance(event, dict):
      continue
    item = dict(event)
    item["_route"] = route
    item["_summary_path"] = str(path)
    tagged_events.append(item)
  return tagged_events


def classify_event(event: dict[str, Any], args: argparse.Namespace) -> list[str]:
  flags: list[str] = []
  end_jerk = as_float(event.get("end_stop_jerk_mps3"))
  cmd_jerk = as_float(event.get("end_stop_cmd_jerk_mps3"))
  accel_step = as_float(event.get("end_stop_accel_step_mps2"))
  min_a_ego = as_float(event.get("min_a_ego_mps2"))

  if end_jerk is not None and end_jerk > args.max_end_stop_jerk:
    flags.append("end_stop_jerk")
  if cmd_jerk is not None and cmd_jerk > args.max_end_stop_cmd_jerk:
    flags.append("end_stop_cmd_jerk")
  if accel_step is not None and accel_step > args.max_end_stop_accel_step:
    flags.append("end_stop_accel_step")
  if min_a_ego is not None and min_a_ego < args.min_a_ego_floor:
    flags.append("hard_min_a_ego")

  return flags


def summarize(events: list[dict[str, Any]], args: argparse.Namespace) -> dict[str, Any]:
  considered: list[dict[str, Any]] = []
  harsh_rows: list[dict[str, Any]] = []

  for event in events:
    entry_speed = as_float(event.get("entry_speed_mps")) or 0.0
    if entry_speed < args.min_entry_speed:
      continue

    considered.append(event)
    flags = classify_event(event, args)
    if flags:
      harsh_rows.append({
        "route": event.get("_route"),
        "event_id": event.get("event_id"),
        "summary_json": event.get("_summary_path"),
        "entry_speed_mps": entry_speed,
        "end_stop_jerk_mps3": as_float(event.get("end_stop_jerk_mps3")),
        "end_stop_cmd_jerk_mps3": as_float(event.get("end_stop_cmd_jerk_mps3")),
        "end_stop_accel_step_mps2": as_float(event.get("end_stop_accel_step_mps2")),
        "min_a_ego_mps2": as_float(event.get("min_a_ego_mps2")),
        "flags": flags,
      })

  event_count = len(considered)
  harsh_count = len(harsh_rows)
  harsh_rate = (harsh_count / event_count) if event_count > 0 else 0.0

  status = "pass"
  reasons: list[str] = []
  if event_count < args.min_events:
    status = "insufficient_events"
    reasons.append(f"events={event_count} < min_events={args.min_events}")
  else:
    if harsh_rate > args.max_harsh_rate:
      status = "fail"
      reasons.append(f"harsh_rate={harsh_rate:.3f} > max_harsh_rate={args.max_harsh_rate:.3f}")
    if args.max_harsh_count > 0 and harsh_count > args.max_harsh_count:
      status = "fail"
      reasons.append(f"harsh_count={harsh_count} > max_harsh_count={args.max_harsh_count}")

  return {
    "status": status,
    "reasons": reasons,
    "events_considered": event_count,
    "harsh_events": harsh_count,
    "harsh_rate": harsh_rate,
    "thresholds": {
      "min_events": args.min_events,
      "min_entry_speed": args.min_entry_speed,
      "max_harsh_rate": args.max_harsh_rate,
      "max_harsh_count": args.max_harsh_count,
      "max_end_stop_jerk": args.max_end_stop_jerk,
      "max_end_stop_cmd_jerk": args.max_end_stop_cmd_jerk,
      "max_end_stop_accel_step": args.max_end_stop_accel_step,
      "min_a_ego_floor": args.min_a_ego_floor,
    },
    "harsh_event_examples": harsh_rows[:20],
  }


def main() -> int:
  args = parse_args()
  summary_paths = [Path(item).expanduser() for item in args.summary_json]
  missing = [str(path) for path in summary_paths if not path.exists()]
  if missing:
    for path in missing:
      print(f"[harsh-check] missing summary: {path}", file=sys.stderr)
    return 2

  all_events: list[dict[str, Any]] = []
  for path in summary_paths:
    all_events.extend(load_events(path))

  result = summarize(all_events, args)
  print(f"[harsh-check] status={result['status']}")
  print(f"[harsh-check] events_considered={result['events_considered']}")
  print(f"[harsh-check] harsh_events={result['harsh_events']}")
  print(f"[harsh-check] harsh_rate={result['harsh_rate']:.3f}")
  if result["reasons"]:
    print(f"[harsh-check] reasons={'; '.join(result['reasons'])}")

  for index, row in enumerate(result["harsh_event_examples"][:5], start=1):
    flags = ",".join(row["flags"])
    print(
      "[harsh-check] sample"
      f"#{index} route={row['route']} event={row['event_id']} entry={row['entry_speed_mps']:.2f}"
      f" endJerk={row['end_stop_jerk_mps3']} cmdJerk={row['end_stop_cmd_jerk_mps3']}"
      f" step={row['end_stop_accel_step_mps2']} minA={row['min_a_ego_mps2']} flags={flags}"
    )

  if args.output_json:
    output_path = Path(args.output_json).expanduser()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n")
    print(f"[harsh-check] output_json={output_path}")

  if result["status"] == "pass":
    return 0
  if result["status"] == "insufficient_events":
    return 2
  return 1


if __name__ == "__main__":
  raise SystemExit(main())
