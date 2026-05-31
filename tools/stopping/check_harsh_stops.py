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
  parser.add_argument("--event-source", default="all", choices=["all", "signal", "speed", "hybrid"],
                      help="Optional filter on event_source field (default: all)")
  parser.add_argument("--min-enabled-ratio", type=float, default=0.0,
                      help="Ignore events where enabled_ratio < this threshold (requires analyzer output with enabled_ratio)")
  parser.add_argument("--min-stop-signal-ratio", type=float, default=0.0,
                      help="Ignore events where stop_signal_ratio < this threshold (requires analyzer output with stop_signal_ratio)")
  parser.add_argument("--min-should-stop-ratio", type=float, default=0.0,
                      help="Ignore events where should_stop_ratio < this threshold")
  parser.add_argument("--min-stopping-state-ratio", type=float, default=0.0,
                      help="Ignore events where stopping_state_ratio < this threshold")
  parser.add_argument("--require-brake-command-below", type=float, default=None,
                      help="Ignore events unless min_accel_cmd_mps2 <= this threshold (for comfort lanes with real braking)")
  parser.add_argument("--min-events", type=int, default=4, help="Minimum event count required to evaluate")
  parser.add_argument("--min-entry-speed", type=float, default=0.20, help="Ignore events below this entry speed (m/s)")
  parser.add_argument("--max-harsh-rate", type=float, default=0.20, help="Maximum allowed harsh-event rate [0..1]")
  parser.add_argument("--max-harsh-count", type=int, default=0, help="Maximum allowed harsh-event count (0 = disabled)")
  parser.add_argument("--max-entry-stop-jerk", type=float, default=None, help="Optional harsh threshold for entry_stop_jerk_mps3")
  parser.add_argument("--max-entry-stop-cmd-jerk", type=float, default=None, help="Optional harsh threshold for entry_stop_cmd_jerk_mps3")
  parser.add_argument("--max-entry-stop-accel-step", type=float, default=None, help="Optional harsh threshold for entry_stop_accel_step_mps2")
  parser.add_argument("--max-end-stop-jerk", type=float, default=0.75, help="Harsh threshold for end_stop_jerk_mps3")
  parser.add_argument("--max-end-stop-cmd-jerk", type=float, default=3.0, help="Harsh threshold for end_stop_cmd_jerk_mps3")
  parser.add_argument("--max-end-stop-accel-step", type=float, default=0.08, help="Harsh threshold for end_stop_accel_step_mps2")
  parser.add_argument("--min-a-ego-floor", type=float, default=-1.05, help="Harsh threshold for min_a_ego_mps2 (more negative is harsher)")
  parser.add_argument("--max-hard-decel-duration", type=float, default=0.75,
                      help="Harsh threshold for hard_decel_duration_s from analyzer output")
  parser.add_argument("--min-lead-distance-hold", type=float, default=1.65,
                      help="Harsh threshold for lead_distance_hold_m; set <= 0 to disable")
  parser.add_argument("--min-far-lead-distance-hold", type=float, default=4.0,
                      help="Minimum lead_distance_hold_m for far-gap brake-spike classification")
  parser.add_argument("--min-far-lead-rollout", type=float, default=3.5,
                      help="Minimum rollout_distance_from_2mps_m for far-gap brake-spike classification")
  parser.add_argument("--max-far-lead-min-accel-cmd", type=float, default=-0.65,
                      help="Brake command threshold for far-gap brake-spike classification")
  parser.add_argument("--max-far-lead-min-a-ego", type=float, default=-0.85,
                      help="Actual decel threshold for far-gap brake-spike classification")
  parser.add_argument("--max-leapfrog-rate", type=float, default=1.0, help="Maximum allowed leapfrog-event rate [0..1] (1.0 disables gating)")
  parser.add_argument("--max-leapfrog-count", type=int, default=0, help="Maximum allowed leapfrog-event count (0 = disabled)")
  parser.add_argument("--max-speed-rebound-while-stop-signal", type=float, default=0.08,
                      help="Leapfrog threshold for speed_rebound_while_stop_signal_mps")
  parser.add_argument("--max-speed-rebound-while-should-stop", type=float, default=0.08,
                      help="Leapfrog threshold for speed_rebound_while_should_stop_mps")
  parser.add_argument("--max-should-stop-unexpected-accel", type=float, default=0.10,
                      help="Leapfrog threshold for should_stop_unexpected_accel_mps2")
  parser.add_argument("--count-stop-signal-drop-as-leapfrog", action="store_true",
                      help="Count stop_signal_dropped_before_hold as leapfrog in filtered comfort lanes")
  parser.add_argument("--count-exit-stop-as-leapfrog", action="store_true",
                      help="Count left_stopping_state_before_hold as leapfrog in filtered comfort lanes")
  parser.add_argument("--output-json", default=None, help="Optional path to write machine-readable check output")
  return parser.parse_args()


def as_float(value: Any) -> float | None:
  if value is None:
    return None
  try:
    return float(value)
  except (TypeError, ValueError):
    return None


def tag_event(event: dict[str, Any], route: str, path: Path) -> dict[str, Any]:
  item = dict(event)
  item["_route"] = route
  item["_summary_path"] = str(path)
  return item


def load_route_events(data: dict[str, Any], path: Path) -> list[dict[str, Any]]:
  tagged_events: list[dict[str, Any]] = []

  route = str(data.get("route", "unknown"))
  events = data.get("events", [])
  if isinstance(events, list):
    for event in events:
      if not isinstance(event, dict):
        continue
      tagged_events.append(tag_event(event, route, path))

  routes = data.get("routes", [])
  if isinstance(routes, list):
    for route_summary in routes:
      if not isinstance(route_summary, dict):
        continue
      route_name = str(route_summary.get("route", "unknown"))
      route_events = route_summary.get("events", [])
      if not isinstance(route_events, list):
        continue
      for event in route_events:
        if not isinstance(event, dict):
          continue
        tagged_events.append(tag_event(event, route_name, path))

  return tagged_events


def load_events(path: Path) -> list[dict[str, Any]]:
  data = json.loads(path.read_text())
  route = str(data.get("route", "unknown"))
  tagged_events = load_route_events(data, path)

  bookmark_matches = data.get("bookmark_matches", [])
  if isinstance(bookmark_matches, list):
    for match in bookmark_matches:
      if not isinstance(match, dict):
        continue
      event = match.get("review_event") or match.get("event")
      if not isinstance(event, dict):
        continue
      match_route = str(match.get("route", route))
      tagged_events.append(tag_event(event, match_route, path))

  return tagged_events


def classify_event(event: dict[str, Any], args: argparse.Namespace) -> tuple[list[str], list[str]]:
  harsh_flags: list[str] = []
  leapfrog_flags: list[str] = []
  entry_jerk = as_float(event.get("entry_stop_jerk_mps3"))
  entry_cmd_jerk = as_float(event.get("entry_stop_cmd_jerk_mps3"))
  entry_accel_step = as_float(event.get("entry_stop_accel_step_mps2"))
  end_jerk = as_float(event.get("end_stop_jerk_mps3"))
  cmd_jerk = as_float(event.get("end_stop_cmd_jerk_mps3"))
  accel_step = as_float(event.get("end_stop_accel_step_mps2"))
  min_a_ego = as_float(event.get("min_a_ego_mps2"))
  hard_decel_duration = as_float(event.get("hard_decel_duration_s"))
  lead_distance_hold = as_float(event.get("lead_distance_hold_m"))
  min_accel_cmd = as_float(event.get("min_accel_cmd_mps2"))
  rollout_2m = as_float(event.get("rollout_distance_from_2mps_m"))
  rebound_signal = as_float(event.get("speed_rebound_while_stop_signal_mps"))
  rebound_should_stop = as_float(event.get("speed_rebound_while_should_stop_mps"))
  should_stop_unexpected_accel = as_float(event.get("should_stop_unexpected_accel_mps2"))

  if args.max_entry_stop_jerk is not None and entry_jerk is not None and entry_jerk > args.max_entry_stop_jerk:
    harsh_flags.append("entry_stop_jerk")
  if args.max_entry_stop_cmd_jerk is not None and entry_cmd_jerk is not None and entry_cmd_jerk > args.max_entry_stop_cmd_jerk:
    harsh_flags.append("entry_stop_cmd_jerk")
  if args.max_entry_stop_accel_step is not None and entry_accel_step is not None and entry_accel_step > args.max_entry_stop_accel_step:
    harsh_flags.append("entry_stop_accel_step")
  if end_jerk is not None and end_jerk > args.max_end_stop_jerk:
    harsh_flags.append("end_stop_jerk")
  if cmd_jerk is not None and cmd_jerk > args.max_end_stop_cmd_jerk:
    harsh_flags.append("end_stop_cmd_jerk")
  if accel_step is not None and accel_step > args.max_end_stop_accel_step:
    harsh_flags.append("end_stop_accel_step")
  if min_a_ego is not None and min_a_ego < args.min_a_ego_floor:
    harsh_flags.append("hard_min_a_ego")
  if hard_decel_duration is not None and hard_decel_duration > args.max_hard_decel_duration:
    harsh_flags.append("sustained_hard_decel")
  if args.min_lead_distance_hold > 0.0 and lead_distance_hold is not None and 0.0 < lead_distance_hold < args.min_lead_distance_hold:
    harsh_flags.append("tight_lead_hold")
  if (
    lead_distance_hold is not None and lead_distance_hold >= args.min_far_lead_distance_hold
    and rollout_2m is not None and rollout_2m >= args.min_far_lead_rollout
    and min_accel_cmd is not None and min_accel_cmd < args.max_far_lead_min_accel_cmd
    and min_a_ego is not None and min_a_ego < args.max_far_lead_min_a_ego
  ):
    harsh_flags.append("far_lead_brake_spike")

  rebound_signal_flag = rebound_signal is not None and rebound_signal > args.max_speed_rebound_while_stop_signal
  rebound_should_stop_flag = rebound_should_stop is not None and rebound_should_stop > args.max_speed_rebound_while_should_stop
  unexpected_accel_flag = (
    should_stop_unexpected_accel is not None and should_stop_unexpected_accel > args.max_should_stop_unexpected_accel
  )
  if rebound_signal_flag:
    leapfrog_flags.append("leapfrog_rebound_signal")
  if rebound_should_stop_flag:
    leapfrog_flags.append("leapfrog_rebound_should_stop")
  if unexpected_accel_flag and (rebound_signal_flag or rebound_should_stop_flag):
    leapfrog_flags.append("leapfrog")
  if bool(event.get("reaccel_before_hold")):
    leapfrog_flags.append("pre_hold_reaccel")
  if args.count_stop_signal_drop_as_leapfrog and bool(event.get("stop_signal_dropped_before_hold")):
    leapfrog_flags.append("stop_signal_drop")
  if args.count_exit_stop_as_leapfrog and bool(event.get("left_stopping_state_before_hold")):
    leapfrog_flags.append("exit_stopping_state")

  return harsh_flags, leapfrog_flags


def summarize(events: list[dict[str, Any]], args: argparse.Namespace) -> dict[str, Any]:
  considered: list[dict[str, Any]] = []
  harsh_rows: list[dict[str, Any]] = []
  leapfrog_rows: list[dict[str, Any]] = []
  filtered_counts: dict[str, int] = {
    "event_source": 0,
    "min_enabled_ratio": 0,
    "min_stop_signal_ratio": 0,
    "min_should_stop_ratio": 0,
    "min_stopping_state_ratio": 0,
    "require_brake_command_below": 0,
    "min_entry_speed": 0,
  }

  for event in events:
    if args.event_source != "all":
      if str(event.get("event_source", "")) != args.event_source:
        filtered_counts["event_source"] += 1
        continue

    if args.min_enabled_ratio > 0.0:
      enabled_ratio = as_float(event.get("enabled_ratio"))
      if enabled_ratio is None or enabled_ratio < args.min_enabled_ratio:
        filtered_counts["min_enabled_ratio"] += 1
        continue
    if args.min_stop_signal_ratio > 0.0:
      stop_signal_ratio = as_float(event.get("stop_signal_ratio"))
      if stop_signal_ratio is None or stop_signal_ratio < args.min_stop_signal_ratio:
        filtered_counts["min_stop_signal_ratio"] += 1
        continue
    if args.min_should_stop_ratio > 0.0:
      should_stop_ratio = as_float(event.get("should_stop_ratio"))
      if should_stop_ratio is None or should_stop_ratio < args.min_should_stop_ratio:
        filtered_counts["min_should_stop_ratio"] += 1
        continue
    if args.min_stopping_state_ratio > 0.0:
      stopping_state_ratio = as_float(event.get("stopping_state_ratio"))
      if stopping_state_ratio is None or stopping_state_ratio < args.min_stopping_state_ratio:
        filtered_counts["min_stopping_state_ratio"] += 1
        continue
    if args.require_brake_command_below is not None:
      min_accel_cmd = as_float(event.get("min_accel_cmd_mps2"))
      if min_accel_cmd is None or min_accel_cmd > args.require_brake_command_below:
        filtered_counts["require_brake_command_below"] += 1
        continue

    entry_speed = as_float(event.get("entry_speed_mps")) or 0.0
    if entry_speed < args.min_entry_speed:
      filtered_counts["min_entry_speed"] += 1
      continue

    considered.append(event)
    harsh_flags, leapfrog_flags = classify_event(event, args)
    event_row = {
      "route": event.get("_route"),
      "event_id": event.get("event_id"),
      "summary_json": event.get("_summary_path"),
      "entry_speed_mps": entry_speed,
      "enabled_ratio": as_float(event.get("enabled_ratio")),
      "should_stop_ratio": as_float(event.get("should_stop_ratio")),
      "stopping_state_ratio": as_float(event.get("stopping_state_ratio")),
      "stop_signal_ratio": as_float(event.get("stop_signal_ratio")),
      "entry_stop_jerk_mps3": as_float(event.get("entry_stop_jerk_mps3")),
      "entry_stop_cmd_jerk_mps3": as_float(event.get("entry_stop_cmd_jerk_mps3")),
      "entry_stop_accel_step_mps2": as_float(event.get("entry_stop_accel_step_mps2")),
      "end_stop_jerk_mps3": as_float(event.get("end_stop_jerk_mps3")),
      "end_stop_cmd_jerk_mps3": as_float(event.get("end_stop_cmd_jerk_mps3")),
      "end_stop_accel_step_mps2": as_float(event.get("end_stop_accel_step_mps2")),
      "min_a_ego_mps2": as_float(event.get("min_a_ego_mps2")),
      "hard_decel_duration_s": as_float(event.get("hard_decel_duration_s")),
      "min_accel_cmd_mps2": as_float(event.get("min_accel_cmd_mps2")),
      "rollout_distance_from_2mps_m": as_float(event.get("rollout_distance_from_2mps_m")),
      "lead_distance_stop_entry_m": as_float(event.get("lead_distance_stop_entry_m")),
      "lead_distance_hold_m": as_float(event.get("lead_distance_hold_m")),
      "stop_signal_dropped_before_hold": bool(event.get("stop_signal_dropped_before_hold")),
      "left_stopping_state_before_hold": bool(event.get("left_stopping_state_before_hold")),
      "speed_rebound_while_stop_signal_mps": as_float(event.get("speed_rebound_while_stop_signal_mps")),
      "speed_rebound_while_should_stop_mps": as_float(event.get("speed_rebound_while_should_stop_mps")),
      "should_stop_unexpected_accel_mps2": as_float(event.get("should_stop_unexpected_accel_mps2")),
      "reaccel_before_hold": bool(event.get("reaccel_before_hold")),
    }
    if harsh_flags:
      harsh_rows.append({**event_row, "flags": harsh_flags})
    if leapfrog_flags:
      leapfrog_rows.append({**event_row, "flags": leapfrog_flags})

  event_count = len(considered)
  harsh_count = len(harsh_rows)
  leapfrog_count = len(leapfrog_rows)
  harsh_rate = (harsh_count / event_count) if event_count > 0 else 0.0
  leapfrog_rate = (leapfrog_count / event_count) if event_count > 0 else 0.0

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
    if leapfrog_rate > args.max_leapfrog_rate:
      status = "fail"
      reasons.append(f"leapfrog_rate={leapfrog_rate:.3f} > max_leapfrog_rate={args.max_leapfrog_rate:.3f}")
    if args.max_leapfrog_count > 0 and leapfrog_count > args.max_leapfrog_count:
      status = "fail"
      reasons.append(f"leapfrog_count={leapfrog_count} > max_leapfrog_count={args.max_leapfrog_count}")

  return {
    "status": status,
    "reasons": reasons,
    "events_considered": event_count,
    "harsh_events": harsh_count,
    "leapfrog_events": leapfrog_count,
    "harsh_rate": harsh_rate,
    "leapfrog_rate": leapfrog_rate,
    "filtered_counts": filtered_counts,
    "thresholds": {
      "event_source": args.event_source,
      "min_enabled_ratio": args.min_enabled_ratio,
      "min_stop_signal_ratio": args.min_stop_signal_ratio,
      "min_should_stop_ratio": args.min_should_stop_ratio,
      "min_stopping_state_ratio": args.min_stopping_state_ratio,
      "require_brake_command_below": args.require_brake_command_below,
      "min_events": args.min_events,
      "min_entry_speed": args.min_entry_speed,
      "max_harsh_rate": args.max_harsh_rate,
      "max_harsh_count": args.max_harsh_count,
      "max_leapfrog_rate": args.max_leapfrog_rate,
      "max_leapfrog_count": args.max_leapfrog_count,
      "max_entry_stop_jerk": args.max_entry_stop_jerk,
      "max_entry_stop_cmd_jerk": args.max_entry_stop_cmd_jerk,
      "max_entry_stop_accel_step": args.max_entry_stop_accel_step,
      "max_end_stop_jerk": args.max_end_stop_jerk,
      "max_end_stop_cmd_jerk": args.max_end_stop_cmd_jerk,
      "max_end_stop_accel_step": args.max_end_stop_accel_step,
      "min_a_ego_floor": args.min_a_ego_floor,
      "max_hard_decel_duration": args.max_hard_decel_duration,
      "min_lead_distance_hold": args.min_lead_distance_hold,
      "min_far_lead_distance_hold": args.min_far_lead_distance_hold,
      "min_far_lead_rollout": args.min_far_lead_rollout,
      "max_far_lead_min_accel_cmd": args.max_far_lead_min_accel_cmd,
      "max_far_lead_min_a_ego": args.max_far_lead_min_a_ego,
      "max_speed_rebound_while_stop_signal": args.max_speed_rebound_while_stop_signal,
      "max_speed_rebound_while_should_stop": args.max_speed_rebound_while_should_stop,
      "max_should_stop_unexpected_accel": args.max_should_stop_unexpected_accel,
      "count_stop_signal_drop_as_leapfrog": args.count_stop_signal_drop_as_leapfrog,
      "count_exit_stop_as_leapfrog": args.count_exit_stop_as_leapfrog,
    },
    "harsh_event_keys": [
      {
        "route": row.get("route"),
        "event_id": row.get("event_id"),
        "summary_json": row.get("summary_json"),
      }
      for row in harsh_rows
    ],
    "leapfrog_event_keys": [
      {
        "route": row.get("route"),
        "event_id": row.get("event_id"),
        "summary_json": row.get("summary_json"),
      }
      for row in leapfrog_rows
    ],
    "harsh_event_examples": harsh_rows[:20],
    "leapfrog_event_examples": leapfrog_rows[:20],
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
  print(f"[harsh-check] leapfrog_events={result['leapfrog_events']}")
  print(f"[harsh-check] leapfrog_rate={result['leapfrog_rate']:.3f}")
  if result["reasons"]:
    print(f"[harsh-check] reasons={'; '.join(result['reasons'])}")
  filtered_counts = result.get("filtered_counts", {})
  if isinstance(filtered_counts, dict) and any(filtered_counts.values()):
    message = (
      f"[harsh-check] filtered event_source={filtered_counts.get('event_source', 0)}"
      + f" min_enabled_ratio={filtered_counts.get('min_enabled_ratio', 0)}"
      + f" min_stop_signal_ratio={filtered_counts.get('min_stop_signal_ratio', 0)}"
      + f" min_should_stop_ratio={filtered_counts.get('min_should_stop_ratio', 0)}"
      + f" min_stopping_state_ratio={filtered_counts.get('min_stopping_state_ratio', 0)}"
      + f" require_brake_command_below={filtered_counts.get('require_brake_command_below', 0)}"
      + f" min_entry_speed={filtered_counts.get('min_entry_speed', 0)}"
    )
    print(message)

  for index, row in enumerate(result["harsh_event_examples"][:5], start=1):
    flags = ",".join(row["flags"])
    message = (
      f"[harsh-check] harsh_sample#{index} route={row['route']} event={row['event_id']} entry={row['entry_speed_mps']:.2f}"
      + f" enabled={row.get('enabled_ratio')} endJerk={row['end_stop_jerk_mps3']} cmdJerk={row['end_stop_cmd_jerk_mps3']}"
      + f" entryJerk={row.get('entry_stop_jerk_mps3')} entryCmdJerk={row.get('entry_stop_cmd_jerk_mps3')}"
      + f" entryStep={row.get('entry_stop_accel_step_mps2')}"
      + f" step={row['end_stop_accel_step_mps2']} minA={row['min_a_ego_mps2']} hardDecel={row.get('hard_decel_duration_s')}"
      + f" minCmd={row.get('min_accel_cmd_mps2')} rollout2m={row.get('rollout_distance_from_2mps_m')}"
      + f" shouldRatio={row.get('should_stop_ratio')} stopRatio={row.get('stopping_state_ratio')}"
      + f" leadEntry={row.get('lead_distance_stop_entry_m')} leadHold={row.get('lead_distance_hold_m')}"
      + f" reboundSig={row.get('speed_rebound_while_stop_signal_mps')}"
      + f" reboundShould={row.get('speed_rebound_while_should_stop_mps')}"
      + f" preHoldReaccel={row.get('reaccel_before_hold')}"
      + f" sigDrop={row.get('stop_signal_dropped_before_hold')} exitStop={row.get('left_stopping_state_before_hold')}"
      + f" shouldUnexpectedA={row.get('should_stop_unexpected_accel_mps2')} flags={flags}"
    )
    print(message)
  for index, row in enumerate(result["leapfrog_event_examples"][:5], start=1):
    flags = ",".join(row["flags"])
    message = (
      f"[harsh-check] leapfrog_sample#{index} route={row['route']} event={row['event_id']} entry={row['entry_speed_mps']:.2f}"
      + f" enabled={row.get('enabled_ratio')} reboundSig={row.get('speed_rebound_while_stop_signal_mps')}"
      + f" reboundShould={row.get('speed_rebound_while_should_stop_mps')}"
      + f" preHoldReaccel={row.get('reaccel_before_hold')}"
      + f" sigDrop={row.get('stop_signal_dropped_before_hold')} exitStop={row.get('left_stopping_state_before_hold')}"
      + f" shouldRatio={row.get('should_stop_ratio')} stopRatio={row.get('stopping_state_ratio')}"
      + f" shouldUnexpectedA={row.get('should_stop_unexpected_accel_mps2')} flags={flags}"
    )
    print(message)

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
