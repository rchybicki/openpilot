#!/usr/bin/env python3
"""Diagnose stopping failure modes from a corpus summary JSON."""

from __future__ import annotations

import argparse
import json
from collections import Counter, defaultdict
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from statistics import median
from typing import Any

FOCUS_CHOICES = ("all", "engaged", "signal", "hybrid", "speed")


@dataclass
class EventRow:
  route: str
  segment: int
  event_id: int
  source: str
  duration_s: float
  distance_m: float
  rollout_from_2m_m: float
  avg_speed_mps: float
  approach_speed_mps: float
  entry_speed_mps: float
  approach_entry_gap_mps: float
  min_a_ego_mps2: float
  min_accel_cmd_mps2: float | None
  should_to_stopping_s: float | None
  end_stop_jerk_mps3: float | None
  end_stop_accel_step_mps2: float | None
  end_stop_cmd_jerk_mps3: float | None
  end_stop_cmd_step_mps2: float | None
  wheel_stop_decel_mps2: float | None
  wheel_speed_drop_150ms_mps: float | None
  reaccel_before_hold: bool
  stop_signal_dropped_before_hold: bool
  left_stopping_state_before_hold: bool
  positive_accel_cmd_near_hold: bool
  positive_accel_cmd_with_stop_signal_near_hold: bool
  max_accel_cmd_near_hold_mps2: float | None
  speed_rebound_after_hold_mps: float
  speed_rebound_while_stop_signal_mps: float
  speed_rebound_while_should_stop_mps: float
  unexpected_accel_while_braking_mps2: float | None
  stable_cmd_accel_delta_mps2: float | None
  should_stop_unexpected_accel_mps2: float | None
  should_stop_decel_relief_spike_mps2: float | None
  low_speed_cmd_std_mps2: float | None
  creep_after_stop_mps: float
  brake_pressed_ratio: float
  forcing_stop_seen: bool
  red_light_seen: bool
  flags: list[str]
  severity_score: float


def utc_now_iso() -> str:
  return datetime.now(timezone.utc).replace(microsecond=0).isoformat()


def parse_args() -> argparse.Namespace:
  parser = argparse.ArgumentParser(description="Diagnose stop failure modes from corpus summary")
  parser.add_argument("--summary-json", required=True, help="Path to corpus summary.json")
  parser.add_argument("--output", default=None, help="Output markdown path (default: alongside summary)")
  parser.add_argument("--focus-source", default="engaged", choices=FOCUS_CHOICES,
                      help="Event source focus. engaged = signal + hybrid")
  parser.add_argument("--top-n", type=int, default=20, help="Number of events/routes to highlight")

  parser.add_argument("--long-duration-s", type=float, default=8.0, help="Long-stop threshold")
  parser.add_argument("--moving-duration-s", type=float, default=12.0, help="Long moving-stop threshold")
  parser.add_argument("--moving-distance-m", type=float, default=20.0, help="Long moving-stop distance threshold")
  parser.add_argument("--queue-speed-threshold-mps", type=float, default=1.2, help="Average speed threshold for queue-like events")
  parser.add_argument("--queue-entry-threshold-mps", type=float, default=2.0, help="Entry speed threshold for queue-like events")
  parser.add_argument("--creep-threshold-mps", type=float, default=0.15, help="Post-stop creep threshold")
  parser.add_argument("--positive-cmd-threshold-mps2", type=float, default=0.02,
                      help="Positive accel command threshold near stop hold")
  parser.add_argument("--hold-rebound-threshold-mps", type=float, default=0.15,
                      help="Speed rebound threshold within 1s after hold")
  parser.add_argument("--unexpected-accel-threshold-mps2", type=float, default=0.20,
                      help="Unexpected positive aEgo threshold while braking command is active near stop")
  parser.add_argument("--stable-cmd-accel-delta-threshold-mps2", type=float, default=0.35,
                      help="Max acceptable |delta aEgo| under near-constant accel command near stop")
  parser.add_argument("--should-stop-rebound-threshold-mps", type=float, default=0.08,
                      help="Rebound threshold while shouldStop remains true near hold")
  parser.add_argument("--should-stop-unexpected-accel-threshold-mps2", type=float, default=0.10,
                      help="Unexpected aEgo threshold while shouldStop and braking command are active")
  parser.add_argument("--should-stop-relief-spike-threshold-mps2", type=float, default=0.18,
                      help="Decel-relief spike threshold under shouldStop with stable braking command")
  parser.add_argument("--rollout-threshold-m", type=float, default=2.0, help="Max acceptable rollout from 2 m/s to stop hold")
  parser.add_argument("--end-stop-jerk-threshold-mps3", type=float, default=2.5, help="Max acceptable end-stop jerk proxy")
  parser.add_argument("--end-stop-accel-step-threshold-mps2", type=float, default=0.5, help="Max acceptable end-stop accel step")
  parser.add_argument("--cmd-jerk-threshold-mps3", type=float, default=3.0, help="Max acceptable command jerk proxy near stop")
  parser.add_argument("--cmd-step-threshold-mps2", type=float, default=0.5, help="Max acceptable command step near stop")
  parser.add_argument("--wheel-stop-decel-threshold-mps2", type=float, default=-4.0,
                      help="Most-negative acceptable wheel-speed decel proxy")
  parser.add_argument("--wheel-drop-threshold-mps", type=float, default=0.35,
                      help="Max acceptable wheel-speed drop within 150ms near stop")
  parser.add_argument("--hard-decel-threshold-mps2", type=float, default=-1.5, help="Harsh observed decel threshold")
  parser.add_argument("--hard-cmd-threshold-mps2", type=float, default=-1.2, help="Harsh commanded decel threshold")
  parser.add_argument("--late-signal-gap-mps", type=float, default=6.0, help="Approach-entry gap threshold for likely late stop-signal onset")
  parser.add_argument("--late-signal-entry-max-mps", type=float, default=2.0, help="Entry speed cap used with late-signal gap")
  return parser.parse_args()


def source_selected(source: str, focus: str) -> bool:
  if focus == "all":
    return True
  if focus == "engaged":
    return source in {"signal", "hybrid"}
  return source == focus


def format_float(value: float | None, digits: int = 2) -> str:
  if value is None:
    return "n/a"
  return f"{value:.{digits}f}"


def load_events(summary: dict[str, Any], args: argparse.Namespace) -> list[EventRow]:
  rows: list[EventRow] = []

  for route_summary in summary["routes"]:
    route = route_summary["route"]
    for event in route_summary["events"]:
      source = str(event.get("event_source", "unknown"))
      if not source_selected(source, args.focus_source):
        continue

      duration_s = float(event.get("duration_s", 0.0))
      distance_m = float(event.get("distance_traveled_m", 0.0))
      avg_speed = distance_m / duration_s if duration_s > 1e-3 else 0.0
      rollout_from_2m = float(event.get("rollout_distance_from_2mps_m", 0.0))
      approach = float(event.get("approach_speed_mps", 0.0))
      entry = float(event.get("entry_speed_mps", 0.0))
      gap = approach - entry
      min_a_ego = float(event.get("min_a_ego_mps2", 0.0))
      min_cmd_raw = event.get("min_accel_cmd_mps2")
      min_cmd = float(min_cmd_raw) if min_cmd_raw is not None else None
      delay_raw = event.get("should_stop_to_stopping_s")
      delay = float(delay_raw) if delay_raw is not None else None
      end_stop_jerk_raw = event.get("end_stop_jerk_mps3")
      end_stop_jerk = float(end_stop_jerk_raw) if end_stop_jerk_raw is not None else None
      end_stop_accel_step_raw = event.get("end_stop_accel_step_mps2")
      end_stop_accel_step = float(end_stop_accel_step_raw) if end_stop_accel_step_raw is not None else None
      end_stop_cmd_jerk_raw = event.get("end_stop_cmd_jerk_mps3")
      end_stop_cmd_jerk = float(end_stop_cmd_jerk_raw) if end_stop_cmd_jerk_raw is not None else None
      end_stop_cmd_step_raw = event.get("end_stop_cmd_step_mps2")
      end_stop_cmd_step = float(end_stop_cmd_step_raw) if end_stop_cmd_step_raw is not None else None
      wheel_stop_decel_raw = event.get("wheel_stop_decel_mps2")
      wheel_stop_decel = float(wheel_stop_decel_raw) if wheel_stop_decel_raw is not None else None
      wheel_drop_raw = event.get("wheel_speed_drop_150ms_mps")
      wheel_drop_150ms = float(wheel_drop_raw) if wheel_drop_raw is not None else None
      reaccel_before_hold = bool(event.get("reaccel_before_hold", False))
      stop_signal_drop_before_hold = bool(event.get("stop_signal_dropped_before_hold", False))
      left_stopping_state_before_hold = bool(event.get("left_stopping_state_before_hold", False))
      max_accel_cmd_near_hold_raw = event.get("max_accel_cmd_near_hold_mps2")
      max_accel_cmd_near_hold = float(max_accel_cmd_near_hold_raw) if max_accel_cmd_near_hold_raw is not None else None
      positive_cmd_near_hold = bool(event.get("positive_accel_cmd_near_hold", False))
      positive_cmd_with_signal = bool(event.get("positive_accel_cmd_with_stop_signal_near_hold", False))
      if max_accel_cmd_near_hold is not None:
        positive_cmd_near_hold = positive_cmd_near_hold or (max_accel_cmd_near_hold > args.positive_cmd_threshold_mps2)
      rebound_raw = event.get("speed_rebound_after_hold_mps")
      speed_rebound_after_hold = float(rebound_raw) if rebound_raw is not None else 0.0
      rebound_with_signal_raw = event.get("speed_rebound_while_stop_signal_mps")
      rebound_while_signal = float(rebound_with_signal_raw) if rebound_with_signal_raw is not None else 0.0
      rebound_with_should_stop_raw = event.get("speed_rebound_while_should_stop_mps")
      rebound_while_should_stop = float(rebound_with_should_stop_raw) if rebound_with_should_stop_raw is not None else 0.0
      unexpected_accel_raw = event.get("unexpected_accel_while_braking_mps2")
      unexpected_accel_while_braking = float(unexpected_accel_raw) if unexpected_accel_raw is not None else None
      stable_cmd_accel_delta_raw = event.get("stable_cmd_accel_delta_mps2")
      stable_cmd_accel_delta = float(stable_cmd_accel_delta_raw) if stable_cmd_accel_delta_raw is not None else None
      should_stop_unexpected_accel_raw = event.get("should_stop_unexpected_accel_mps2")
      should_stop_unexpected_accel = (
        float(should_stop_unexpected_accel_raw) if should_stop_unexpected_accel_raw is not None else None
      )
      should_stop_relief_spike_raw = event.get("should_stop_decel_relief_spike_mps2")
      should_stop_relief_spike = (
        float(should_stop_relief_spike_raw) if should_stop_relief_spike_raw is not None else None
      )
      low_speed_cmd_std_raw = event.get("low_speed_cmd_std_mps2")
      low_speed_cmd_std = float(low_speed_cmd_std_raw) if low_speed_cmd_std_raw is not None else None
      creep = float(event.get("creep_after_stop_mps", 0.0))
      brake_ratio = float(event.get("brake_pressed_ratio", 0.0))

      queue_like = (
        duration_s >= args.long_duration_s
        and avg_speed <= args.queue_speed_threshold_mps
        and entry <= args.queue_entry_threshold_mps
      )
      long_duration = duration_s >= args.long_duration_s and not queue_like
      long_moving = duration_s >= args.moving_duration_s and distance_m >= args.moving_distance_m
      creep_flag = creep >= args.creep_threshold_mps
      rollout_excess = rollout_from_2m > args.rollout_threshold_m
      jerk_high = end_stop_jerk is not None and end_stop_jerk > args.end_stop_jerk_threshold_mps3
      end_step_high = end_stop_accel_step is not None and end_stop_accel_step > args.end_stop_accel_step_threshold_mps2
      cmd_jerk_high = end_stop_cmd_jerk is not None and end_stop_cmd_jerk > args.cmd_jerk_threshold_mps3
      cmd_step_high = end_stop_cmd_step is not None and end_stop_cmd_step > args.cmd_step_threshold_mps2
      wheel_sharp = (
        (wheel_stop_decel is not None and wheel_stop_decel <= args.wheel_stop_decel_threshold_mps2)
        or (wheel_drop_150ms is not None and wheel_drop_150ms >= args.wheel_drop_threshold_mps)
      )
      hold_rebound = speed_rebound_after_hold >= args.hold_rebound_threshold_mps
      hold_rebound_while_signal = rebound_while_signal >= args.hold_rebound_threshold_mps
      hold_rebound_while_should_stop = rebound_while_should_stop >= args.should_stop_rebound_threshold_mps
      unexpected_accel_under_brake = (
        unexpected_accel_while_braking is not None and unexpected_accel_while_braking > args.unexpected_accel_threshold_mps2
      )
      stable_cmd_disturbance = (
        stable_cmd_accel_delta is not None and stable_cmd_accel_delta > args.stable_cmd_accel_delta_threshold_mps2
      )
      should_stop_unexpected_disturbance = (
        should_stop_unexpected_accel is not None
        and should_stop_unexpected_accel > args.should_stop_unexpected_accel_threshold_mps2
      )
      should_stop_relief_disturbance = (
        should_stop_relief_spike is not None
        and should_stop_relief_spike > args.should_stop_relief_spike_threshold_mps2
      )
      hard_brake = (
        min_a_ego <= args.hard_decel_threshold_mps2
        or (min_cmd is not None and min_cmd <= args.hard_cmd_threshold_mps2)
      )
      late_signal = gap >= args.late_signal_gap_mps and entry <= args.late_signal_entry_max_mps
      late_signal = late_signal and duration_s >= 1.5 and distance_m >= 0.5
      unexpected_release = positive_cmd_with_signal or (positive_cmd_near_hold and not stop_signal_drop_before_hold)

      flags: list[str] = []
      if queue_like:
        flags.append("queue_like")
      if long_duration:
        flags.append("excessive_duration")
      if long_moving:
        flags.append("long_moving_stop")
      if creep_flag:
        flags.append("post_stop_creep")
      if rollout_excess:
        flags.append("rollout_excess")
      if jerk_high:
        flags.append("end_stop_jerk_high")
      if end_step_high:
        flags.append("end_stop_accel_step_high")
      if cmd_jerk_high:
        flags.append("end_stop_cmd_jerk_high")
      if cmd_step_high:
        flags.append("end_stop_cmd_step_high")
      if wheel_sharp:
        flags.append("wheel_stop_sharpness_high")
      if reaccel_before_hold:
        flags.append("reaccel_before_hold")
      if stop_signal_drop_before_hold:
        flags.append("stop_signal_drop_before_hold")
      if left_stopping_state_before_hold:
        flags.append("left_stopping_state_before_hold")
      if positive_cmd_near_hold:
        flags.append("positive_cmd_near_hold_any")
      if unexpected_release:
        flags.append("positive_cmd_near_hold_unexpected")
      if hold_rebound:
        flags.append("speed_rebound_after_hold")
      if hold_rebound_while_signal:
        flags.append("speed_rebound_while_stop_signal")
      if hold_rebound_while_should_stop:
        flags.append("speed_rebound_while_should_stop")
      if unexpected_accel_under_brake:
        flags.append("unexpected_accel_under_brake_cmd")
      if stable_cmd_disturbance:
        flags.append("stable_cmd_accel_disturbance")
      if should_stop_unexpected_disturbance:
        flags.append("unexpected_accel_under_should_stop")
      if should_stop_relief_disturbance:
        flags.append("decel_relief_spike_under_should_stop")
      if hard_brake:
        flags.append("hard_brake")
      if late_signal:
        flags.append("late_signal_onset")

      severity = 0.0
      if hard_brake:
        severity += 3.0
      if rollout_excess:
        severity += 2.5
      if jerk_high:
        severity += 2.5
      if end_step_high:
        severity += 2.0
      if cmd_jerk_high:
        severity += 2.5
      if cmd_step_high:
        severity += 2.0
      if wheel_sharp:
        severity += 2.5
      if reaccel_before_hold:
        severity += 2.5
      if stop_signal_drop_before_hold:
        severity += 0.2
      if left_stopping_state_before_hold:
        severity += 0.2
      if unexpected_release:
        severity += 2.0
      if hold_rebound:
        severity += 2.0
      if hold_rebound_while_signal:
        severity += 2.0
      if unexpected_accel_under_brake:
        severity += 2.5
      if stable_cmd_disturbance:
        severity += 2.0
      if hold_rebound_while_should_stop:
        severity += 2.5
      if should_stop_unexpected_disturbance:
        severity += 3.0
      if should_stop_relief_disturbance:
        severity += 2.5
      if long_moving:
        severity += 2.0
      if long_duration:
        severity += 1.5
      if creep_flag:
        severity += 2.0
      if late_signal:
        severity += 1.0
      severity += min(max(duration_s / 20.0, 0.0), 1.0)
      severity += min(max(creep / 0.3, 0.0), 1.0)
      severity += min(max(rollout_from_2m / 5.0, 0.0), 1.0)

      rows.append(
        EventRow(
          route=route,
          segment=int(event["start_segment"]),
          event_id=int(event["event_id"]),
          source=source,
          duration_s=duration_s,
          distance_m=distance_m,
          rollout_from_2m_m=rollout_from_2m,
          avg_speed_mps=avg_speed,
          approach_speed_mps=approach,
          entry_speed_mps=entry,
          approach_entry_gap_mps=gap,
          min_a_ego_mps2=min_a_ego,
          min_accel_cmd_mps2=min_cmd,
          should_to_stopping_s=delay,
          end_stop_jerk_mps3=end_stop_jerk,
          end_stop_accel_step_mps2=end_stop_accel_step,
          end_stop_cmd_jerk_mps3=end_stop_cmd_jerk,
          end_stop_cmd_step_mps2=end_stop_cmd_step,
          wheel_stop_decel_mps2=wheel_stop_decel,
          wheel_speed_drop_150ms_mps=wheel_drop_150ms,
          reaccel_before_hold=reaccel_before_hold,
          stop_signal_dropped_before_hold=stop_signal_drop_before_hold,
          left_stopping_state_before_hold=left_stopping_state_before_hold,
          positive_accel_cmd_near_hold=positive_cmd_near_hold,
          positive_accel_cmd_with_stop_signal_near_hold=positive_cmd_with_signal,
          max_accel_cmd_near_hold_mps2=max_accel_cmd_near_hold,
          speed_rebound_after_hold_mps=speed_rebound_after_hold,
          speed_rebound_while_stop_signal_mps=rebound_while_signal,
          speed_rebound_while_should_stop_mps=rebound_while_should_stop,
          unexpected_accel_while_braking_mps2=unexpected_accel_while_braking,
          stable_cmd_accel_delta_mps2=stable_cmd_accel_delta,
          should_stop_unexpected_accel_mps2=should_stop_unexpected_accel,
          should_stop_decel_relief_spike_mps2=should_stop_relief_spike,
          low_speed_cmd_std_mps2=low_speed_cmd_std,
          creep_after_stop_mps=creep,
          brake_pressed_ratio=brake_ratio,
          forcing_stop_seen=bool(event.get("forcing_stop_seen", False)),
          red_light_seen=bool(event.get("red_light_seen", False)),
          flags=flags,
          severity_score=severity,
        )
      )

  return rows


def build_markdown(summary: dict[str, Any], rows: list[EventRow], args: argparse.Namespace) -> str:
  lines: list[str] = []
  lines.append("# Stopping Failure Diagnosis")
  lines.append("")
  lines.append(f"- Generated (UTC): {utc_now_iso()}")
  lines.append(f"- Source summary: `{Path(args.summary_json).expanduser()}`")
  lines.append(f"- Host: `{summary.get('host', 'unknown')}`")
  lines.append(f"- Corpus routes: {summary.get('routes_analyzed', 0)}")
  lines.append(f"- Corpus qlogs: {summary.get('total_qlogs', 0)}")
  lines.append(f"- Corpus events (all sources): {summary.get('total_stop_events', 0)}")
  lines.append(f"- Focus source: `{args.focus_source}`")
  lines.append(f"- Focused events analyzed: {len(rows)}")
  lines.append("")

  if not rows:
    lines.append("No events matched the selected source filter.")
    lines.append("")
    return "\n".join(lines)

  source_counts = Counter(row.source for row in rows)
  lines.append("## Source Mix")
  lines.append("")
  for source, count in sorted(source_counts.items()):
    lines.append(f"- `{source}`: {count}")
  lines.append("")

  durations = [row.duration_s for row in rows]
  approaches = [row.approach_speed_mps for row in rows]
  entries = [row.entry_speed_mps for row in rows]
  min_a = [row.min_a_ego_mps2 for row in rows]
  creep_vals = [row.creep_after_stop_mps for row in rows]
  rollout_vals = [row.rollout_from_2m_m for row in rows]
  end_jerk_vals = [row.end_stop_jerk_mps3 for row in rows if row.end_stop_jerk_mps3 is not None]
  end_step_vals = [row.end_stop_accel_step_mps2 for row in rows if row.end_stop_accel_step_mps2 is not None]
  cmd_jerk_vals = [row.end_stop_cmd_jerk_mps3 for row in rows if row.end_stop_cmd_jerk_mps3 is not None]
  cmd_step_vals = [row.end_stop_cmd_step_mps2 for row in rows if row.end_stop_cmd_step_mps2 is not None]
  wheel_decel_vals = [row.wheel_stop_decel_mps2 for row in rows if row.wheel_stop_decel_mps2 is not None]
  wheel_drop_vals = [row.wheel_speed_drop_150ms_mps for row in rows if row.wheel_speed_drop_150ms_mps is not None]
  cmd_near_hold_vals = [row.max_accel_cmd_near_hold_mps2 for row in rows if row.max_accel_cmd_near_hold_mps2 is not None]
  rebound_vals = [row.speed_rebound_after_hold_mps for row in rows]
  rebound_signal_vals = [row.speed_rebound_while_stop_signal_mps for row in rows]
  rebound_should_stop_vals = [row.speed_rebound_while_should_stop_mps for row in rows]
  unexpected_accel_vals = [
    row.unexpected_accel_while_braking_mps2 for row in rows if row.unexpected_accel_while_braking_mps2 is not None
  ]
  stable_cmd_delta_vals = [row.stable_cmd_accel_delta_mps2 for row in rows if row.stable_cmd_accel_delta_mps2 is not None]
  should_stop_unexpected_vals = [
    row.should_stop_unexpected_accel_mps2 for row in rows if row.should_stop_unexpected_accel_mps2 is not None
  ]
  should_stop_relief_vals = [
    row.should_stop_decel_relief_spike_mps2 for row in rows if row.should_stop_decel_relief_spike_mps2 is not None
  ]
  low_speed_cmd_std_vals = [row.low_speed_cmd_std_mps2 for row in rows if row.low_speed_cmd_std_mps2 is not None]
  cmd_vals = [row.min_accel_cmd_mps2 for row in rows if row.min_accel_cmd_mps2 is not None]

  lines.append("## Aggregate")
  lines.append("")
  lines.append(f"- Median duration: {format_float(median(durations), 2)} s")
  lines.append(f"- Median approach speed: {format_float(median(approaches), 2)} m/s")
  lines.append(f"- Median entry speed: {format_float(median(entries), 2)} m/s")
  lines.append(f"- Median min aEgo: {format_float(median(min_a), 2)} m/s²")
  lines.append(f"- Median min accel cmd: {format_float(median(cmd_vals), 2) if cmd_vals else 'n/a'} m/s²")
  lines.append(f"- Median rollout from 2 m/s: {format_float(median(rollout_vals), 2)} m")
  lines.append(f"- Median end-stop jerk proxy: {format_float(median(end_jerk_vals), 2) if end_jerk_vals else 'n/a'} m/s³")
  lines.append(f"- Median end-stop accel step: {format_float(median(end_step_vals), 2) if end_step_vals else 'n/a'} m/s²")
  lines.append(f"- Median end-stop command jerk proxy: {format_float(median(cmd_jerk_vals), 2) if cmd_jerk_vals else 'n/a'} m/s³")
  lines.append(f"- Median end-stop command step: {format_float(median(cmd_step_vals), 2) if cmd_step_vals else 'n/a'} m/s²")
  lines.append(f"- Median wheel-stop decel proxy: {format_float(median(wheel_decel_vals), 2) if wheel_decel_vals else 'n/a'} m/s²")
  lines.append(f"- Median wheel-speed drop (150ms): {format_float(median(wheel_drop_vals), 3) if wheel_drop_vals else 'n/a'} m/s")
  lines.append(f"- Median max accel cmd near hold: {format_float(median(cmd_near_hold_vals), 3) if cmd_near_hold_vals else 'n/a'} m/s²")
  lines.append(f"- Median speed rebound after hold: {format_float(median(rebound_vals), 3)} m/s")
  lines.append(f"- Median speed rebound while stop signal remains true: {format_float(median(rebound_signal_vals), 3)} m/s")
  lines.append(f"- Median speed rebound while shouldStop remains true: {format_float(median(rebound_should_stop_vals), 3)} m/s")
  lines.append(
    "- Median unexpected aEgo while braking cmd<=-0.1: "
    f"{format_float(median(unexpected_accel_vals), 3) if unexpected_accel_vals else 'n/a'} m/s²"
  )
  lines.append(
    "- Median |delta aEgo| under stable cmd (|delta cmd|<=0.02): "
    f"{format_float(median(stable_cmd_delta_vals), 3) if stable_cmd_delta_vals else 'n/a'} m/s²"
  )
  lines.append(
    "- Median unexpected aEgo while shouldStop + braking cmd<=-0.1: "
    f"{format_float(median(should_stop_unexpected_vals), 3) if should_stop_unexpected_vals else 'n/a'} m/s²"
  )
  lines.append(
    "- Median decel-relief spike under shouldStop (stable cmd): "
    f"{format_float(median(should_stop_relief_vals), 3) if should_stop_relief_vals else 'n/a'} m/s²"
  )
  lines.append(
    "- Median low-speed cmd std (v<=1.2, stop signal): "
    f"{format_float(median(low_speed_cmd_std_vals), 3) if low_speed_cmd_std_vals else 'n/a'} m/s²"
  )
  lines.append(f"- Median creep after stop: {format_float(median(creep_vals), 3)} m/s")
  rollout_ok = sum(1 for row in rows if row.rollout_from_2m_m <= args.rollout_threshold_m)
  jerk_ok = sum(1 for row in rows if row.end_stop_jerk_mps3 is None or row.end_stop_jerk_mps3 <= args.end_stop_jerk_threshold_mps3)
  step_ok = sum(1 for row in rows if row.end_stop_accel_step_mps2 is None or row.end_stop_accel_step_mps2 <= args.end_stop_accel_step_threshold_mps2)
  cmd_jerk_ok = sum(1 for row in rows if row.end_stop_cmd_jerk_mps3 is None or row.end_stop_cmd_jerk_mps3 <= args.cmd_jerk_threshold_mps3)
  cmd_step_ok = sum(1 for row in rows if row.end_stop_cmd_step_mps2 is None or row.end_stop_cmd_step_mps2 <= args.cmd_step_threshold_mps2)
  wheel_sharp_ok = sum(
    1
    for row in rows
    if (row.wheel_stop_decel_mps2 is None or row.wheel_stop_decel_mps2 > args.wheel_stop_decel_threshold_mps2)
    and (row.wheel_speed_drop_150ms_mps is None or row.wheel_speed_drop_150ms_mps < args.wheel_drop_threshold_mps)
  )
  no_reaccel = sum(1 for row in rows if not row.reaccel_before_hold)
  no_stop_signal_drop = sum(1 for row in rows if not row.stop_signal_dropped_before_hold)
  no_stopping_exit = sum(1 for row in rows if not row.left_stopping_state_before_hold)
  no_positive_cmd = sum(1 for row in rows if not row.positive_accel_cmd_near_hold)
  no_positive_cmd_with_signal = sum(1 for row in rows if not row.positive_accel_cmd_with_stop_signal_near_hold)
  rebound_ok = sum(1 for row in rows if row.speed_rebound_after_hold_mps < args.hold_rebound_threshold_mps)
  rebound_signal_ok = sum(1 for row in rows if row.speed_rebound_while_stop_signal_mps < args.hold_rebound_threshold_mps)
  rebound_should_stop_ok = sum(1 for row in rows if row.speed_rebound_while_should_stop_mps < args.should_stop_rebound_threshold_mps)
  unexpected_accel_ok = sum(
    1
    for row in rows
    if row.unexpected_accel_while_braking_mps2 is None or row.unexpected_accel_while_braking_mps2 <= args.unexpected_accel_threshold_mps2
  )
  stable_cmd_delta_ok = sum(
    1
    for row in rows
    if row.stable_cmd_accel_delta_mps2 is None or row.stable_cmd_accel_delta_mps2 <= args.stable_cmd_accel_delta_threshold_mps2
  )
  should_stop_unexpected_ok = sum(
    1
    for row in rows
    if row.should_stop_unexpected_accel_mps2 is None or row.should_stop_unexpected_accel_mps2 <= args.should_stop_unexpected_accel_threshold_mps2
  )
  should_stop_relief_ok = sum(
    1
    for row in rows
    if row.should_stop_decel_relief_spike_mps2 is None or row.should_stop_decel_relief_spike_mps2 <= args.should_stop_relief_spike_threshold_mps2
  )

  def unexpected_release(row: EventRow) -> bool:
    return row.positive_accel_cmd_with_stop_signal_near_hold or (
      row.positive_accel_cmd_near_hold and not row.stop_signal_dropped_before_hold
    )

  unexpected_release_ok = sum(1 for row in rows if not unexpected_release(row))
  strict_good = sum(
    1
    for row in rows
    if row.rollout_from_2m_m <= args.rollout_threshold_m
    and (row.end_stop_jerk_mps3 is None or row.end_stop_jerk_mps3 <= args.end_stop_jerk_threshold_mps3)
    and (row.end_stop_accel_step_mps2 is None or row.end_stop_accel_step_mps2 <= args.end_stop_accel_step_threshold_mps2)
    and (row.end_stop_cmd_jerk_mps3 is None or row.end_stop_cmd_jerk_mps3 <= args.cmd_jerk_threshold_mps3)
    and (row.end_stop_cmd_step_mps2 is None or row.end_stop_cmd_step_mps2 <= args.cmd_step_threshold_mps2)
    and (row.wheel_stop_decel_mps2 is None or row.wheel_stop_decel_mps2 > args.wheel_stop_decel_threshold_mps2)
    and (row.wheel_speed_drop_150ms_mps is None or row.wheel_speed_drop_150ms_mps < args.wheel_drop_threshold_mps)
    and not row.reaccel_before_hold
    and (row.unexpected_accel_while_braking_mps2 is None or row.unexpected_accel_while_braking_mps2 <= args.unexpected_accel_threshold_mps2)
    and (row.stable_cmd_accel_delta_mps2 is None or row.stable_cmd_accel_delta_mps2 <= args.stable_cmd_accel_delta_threshold_mps2)
    and (row.should_stop_unexpected_accel_mps2 is None or row.should_stop_unexpected_accel_mps2 <= args.should_stop_unexpected_accel_threshold_mps2)
    and (row.should_stop_decel_relief_spike_mps2 is None or row.should_stop_decel_relief_spike_mps2 <= args.should_stop_relief_spike_threshold_mps2)
    and not unexpected_release(row)
    and row.speed_rebound_while_stop_signal_mps < args.hold_rebound_threshold_mps
    and row.speed_rebound_while_should_stop_mps < args.should_stop_rebound_threshold_mps
    and row.creep_after_stop_mps < args.creep_threshold_mps
  )
  lines.append(f"- Rollout <= {args.rollout_threshold_m:.2f}m: {rollout_ok}/{len(rows)}")
  lines.append(f"- End-stop jerk <= {args.end_stop_jerk_threshold_mps3:.2f}: {jerk_ok}/{len(rows)}")
  lines.append(f"- End-stop accel-step <= {args.end_stop_accel_step_threshold_mps2:.2f}: {step_ok}/{len(rows)}")
  lines.append(f"- Command jerk <= {args.cmd_jerk_threshold_mps3:.2f}: {cmd_jerk_ok}/{len(rows)}")
  lines.append(f"- Command step <= {args.cmd_step_threshold_mps2:.2f}: {cmd_step_ok}/{len(rows)}")
  lines.append(f"- Wheel-stop sharpness within threshold: {wheel_sharp_ok}/{len(rows)}")
  lines.append(f"- No re-accel-before-hold: {no_reaccel}/{len(rows)}")
  lines.append(f"- No stop-signal drop before hold: {no_stop_signal_drop}/{len(rows)}")
  lines.append(f"- Stayed in stopping state through hold: {no_stopping_exit}/{len(rows)}")
  lines.append(f"- No positive accel command near hold (any): {no_positive_cmd}/{len(rows)}")
  lines.append(f"- No positive accel command near hold while stop signal is true: {no_positive_cmd_with_signal}/{len(rows)}")
  lines.append(f"- No unexpected positive-command release near hold: {unexpected_release_ok}/{len(rows)}")
  lines.append(f"- Rebound after hold < {args.hold_rebound_threshold_mps:.2f} m/s: {rebound_ok}/{len(rows)}")
  lines.append(f"- Rebound while stop signal true < {args.hold_rebound_threshold_mps:.2f} m/s: {rebound_signal_ok}/{len(rows)}")
  lines.append(
    f"- Rebound while shouldStop true < {args.should_stop_rebound_threshold_mps:.2f} m/s: "
    f"{rebound_should_stop_ok}/{len(rows)}"
  )
  lines.append(f"- Unexpected aEgo under braking <= {args.unexpected_accel_threshold_mps2:.2f}: {unexpected_accel_ok}/{len(rows)}")
  lines.append(
    f"- |delta aEgo| under stable cmd <= {args.stable_cmd_accel_delta_threshold_mps2:.2f}: "
    f"{stable_cmd_delta_ok}/{len(rows)}"
  )
  lines.append(
    f"- Unexpected aEgo under shouldStop <= {args.should_stop_unexpected_accel_threshold_mps2:.2f}: "
    f"{should_stop_unexpected_ok}/{len(rows)}"
  )
  lines.append(
    f"- Decel-relief under shouldStop <= {args.should_stop_relief_spike_threshold_mps2:.2f}: "
    f"{should_stop_relief_ok}/{len(rows)}"
  )
  lines.append(f"- Strict good-stop estimate: {strict_good}/{len(rows)} ({(100.0 * strict_good / len(rows)):.1f}%)")
  lines.append("")

  flag_counts = Counter(flag for row in rows for flag in row.flags)
  lines.append("## Failure Flags")
  lines.append("")
  if flag_counts:
    for flag, count in flag_counts.most_common():
      lines.append(f"- `{flag}`: {count}")
  else:
    lines.append("- No failure flags triggered by current thresholds.")
  lines.append("")

  lines.append("## Candidate Root Causes")
  lines.append("")
  if flag_counts.get("late_signal_onset", 0) > 0:
    lines.append("- Stop-signal onset is frequently late relative to approach speed (`approach-entry gap` high while `entry` is low).")
    lines.append(
      "- This suggests controller-side stop execution is concentrated near low speed; "
      "reduce end-stop sensitivity without relying on planner changes."
    )
  if flag_counts.get("long_moving_stop", 0) > 0:
    lines.append("- Some events stay in stopping behavior for long distance while still moving; this can feel hesitant or inconsistent.")
  if flag_counts.get("post_stop_creep", 0) > 0:
    lines.append("- Post-stop creep appears in outliers and can cause roll-forward behavior after standstill.")
  if flag_counts.get("hard_brake", 0) > 0:
    lines.append("- A subset of events shows harsh commanded/observed decel, indicating occasional comfort regressions.")
  if flag_counts.get("queue_like", 0) > 0:
    lines.append("- Many long events are queue-like (slow rolling before final standstill); treat these separately from true stop-finalization faults.")
  if flag_counts.get("stop_signal_drop_before_hold", 0) > 0 or flag_counts.get("left_stopping_state_before_hold", 0) > 0:
    lines.append(
      "- Some events drop stop intent/state right before hold; this can be normal at "
      "light/lead release, so treat as contextual unless paired with rebound."
    )
  if flag_counts.get("positive_cmd_near_hold_unexpected", 0) > 0 or flag_counts.get("speed_rebound_while_stop_signal", 0) > 0:
    lines.append("- Unexpected near-hold release while stop signal remains active appears in outliers and matches stop/release/re-stop discomfort.")
  if flag_counts.get("unexpected_accel_under_brake_cmd", 0) > 0 or flag_counts.get("stable_cmd_accel_disturbance", 0) > 0:
    lines.append("- Low-speed acceleration disturbances under stable braking command suggest drivetrain/clutch behavior dominating final-stop feel.")
  if flag_counts.get("unexpected_accel_under_should_stop", 0) > 0 or flag_counts.get("decel_relief_spike_under_should_stop", 0) > 0:
    lines.append("- Disturbances while `shouldStop` remains true are strong clutch-candidate events and should be prioritized.")
  lines.append("")

  ranked = sorted(rows, key=lambda row: row.severity_score, reverse=True)
  lines.append("## Top Problem Events")
  lines.append("")
  lines.append(
    "|Rank|Score|Route|Seg|Event|Source|Duration|Distance|Rollout2m|EndJerk|EndStep|CmdJerk|CmdStep|"
    "WheelDecel|WheelDrop150ms|ReAccel|SigDrop|ExitStop|PosCmd|PosCmdSig|MaxCmdNear|Rebound|ReboundSig|"
    "ReboundShould|UnexpA|StableDelta|UnexpShould|ReliefShould|Approach|Entry|Gap|Min aEgo|Min cmd|Creep|Flags|"
  )
  lines.append(
    "|---:|---:|---|---:|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|"
    "---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|"
  )
  for rank, row in enumerate(ranked[:args.top_n], start=1):
    lines.append(
      "|"
      f"{rank}|"
      f"{format_float(row.severity_score, 2)}|"
      f"{row.route}|"
      f"{row.segment}|"
      f"{row.event_id}|"
      f"{row.source}|"
      f"{format_float(row.duration_s, 2)}|"
      f"{format_float(row.distance_m, 2)}|"
      f"{format_float(row.rollout_from_2m_m, 2)}|"
      f"{format_float(row.end_stop_jerk_mps3, 2)}|"
      f"{format_float(row.end_stop_accel_step_mps2, 2)}|"
      f"{format_float(row.end_stop_cmd_jerk_mps3, 2)}|"
      f"{format_float(row.end_stop_cmd_step_mps2, 2)}|"
      f"{format_float(row.wheel_stop_decel_mps2, 2)}|"
      f"{format_float(row.wheel_speed_drop_150ms_mps, 3)}|"
      f"{'yes' if row.reaccel_before_hold else 'no'}|"
      f"{'yes' if row.stop_signal_dropped_before_hold else 'no'}|"
      f"{'yes' if row.left_stopping_state_before_hold else 'no'}|"
      f"{'yes' if row.positive_accel_cmd_near_hold else 'no'}|"
      f"{'yes' if row.positive_accel_cmd_with_stop_signal_near_hold else 'no'}|"
      f"{format_float(row.max_accel_cmd_near_hold_mps2, 2)}|"
      f"{format_float(row.speed_rebound_after_hold_mps, 3)}|"
      f"{format_float(row.speed_rebound_while_stop_signal_mps, 3)}|"
      f"{format_float(row.speed_rebound_while_should_stop_mps, 3)}|"
      f"{format_float(row.unexpected_accel_while_braking_mps2, 2)}|"
      f"{format_float(row.stable_cmd_accel_delta_mps2, 2)}|"
      f"{format_float(row.should_stop_unexpected_accel_mps2, 2)}|"
      f"{format_float(row.should_stop_decel_relief_spike_mps2, 2)}|"
      f"{format_float(row.approach_speed_mps, 2)}|"
      f"{format_float(row.entry_speed_mps, 2)}|"
      f"{format_float(row.approach_entry_gap_mps, 2)}|"
      f"{format_float(row.min_a_ego_mps2, 2)}|"
      f"{format_float(row.min_accel_cmd_mps2, 2)}|"
      f"{format_float(row.creep_after_stop_mps, 3)}|"
      f"{', '.join(row.flags) if row.flags else '-'}|"
    )
  lines.append("")

  route_flag_counts: defaultdict[str, int] = defaultdict(int)
  route_scores: defaultdict[str, float] = defaultdict(float)
  for row in rows:
    route_flag_counts[row.route] += len(row.flags)
    route_scores[row.route] += row.severity_score

  top_routes = sorted(route_scores.items(), key=lambda item: item[1], reverse=True)[:args.top_n]
  lines.append("## Priority Routes")
  lines.append("")
  lines.append("|Route|Severity score sum|Flag count|")
  lines.append("|---|---:|---:|")
  for route, score_sum in top_routes:
    lines.append(f"|{route}|{format_float(score_sum, 2)}|{route_flag_counts[route]}|")
  lines.append("")

  host = summary.get("host", "commawifi")
  event_mode = str(summary.get("event_mode", "engaged_signal"))
  min_entry_speed = float(summary.get("min_entry_speed", 2.0))
  require_enabled_speed_events = bool(summary.get("require_enabled_speed_events", False))
  lines.append("## Route Review Commands")
  lines.append("")
  lines.append("Use these to generate per-event graphs for top routes:")
  lines.append("")
  lines.append("```bash")
  for route, _score_sum in top_routes[:5]:
    cmd = (
      "python tools/stopping/analyze_stopping_behavior.py "
      f"--host {host} --route {route} --event-mode {event_mode} --min-entry-speed {min_entry_speed:.2f}"
    )
    if require_enabled_speed_events:
      cmd += " --require-enabled-speed-events"
    lines.append(
      cmd
    )
  lines.append("```")
  lines.append("")

  lines.append("## Thresholds")
  lines.append("")
  lines.append(f"- long_duration_s: {args.long_duration_s}")
  lines.append(f"- moving_duration_s: {args.moving_duration_s}")
  lines.append(f"- moving_distance_m: {args.moving_distance_m}")
  lines.append(f"- queue_speed_threshold_mps: {args.queue_speed_threshold_mps}")
  lines.append(f"- queue_entry_threshold_mps: {args.queue_entry_threshold_mps}")
  lines.append(f"- creep_threshold_mps: {args.creep_threshold_mps}")
  lines.append(f"- positive_cmd_threshold_mps2: {args.positive_cmd_threshold_mps2}")
  lines.append(f"- hold_rebound_threshold_mps: {args.hold_rebound_threshold_mps}")
  lines.append(f"- unexpected_accel_threshold_mps2: {args.unexpected_accel_threshold_mps2}")
  lines.append(f"- stable_cmd_accel_delta_threshold_mps2: {args.stable_cmd_accel_delta_threshold_mps2}")
  lines.append(f"- should_stop_rebound_threshold_mps: {args.should_stop_rebound_threshold_mps}")
  lines.append(f"- should_stop_unexpected_accel_threshold_mps2: {args.should_stop_unexpected_accel_threshold_mps2}")
  lines.append(f"- should_stop_relief_spike_threshold_mps2: {args.should_stop_relief_spike_threshold_mps2}")
  lines.append(f"- rollout_threshold_m: {args.rollout_threshold_m}")
  lines.append(f"- end_stop_jerk_threshold_mps3: {args.end_stop_jerk_threshold_mps3}")
  lines.append(f"- end_stop_accel_step_threshold_mps2: {args.end_stop_accel_step_threshold_mps2}")
  lines.append(f"- cmd_jerk_threshold_mps3: {args.cmd_jerk_threshold_mps3}")
  lines.append(f"- cmd_step_threshold_mps2: {args.cmd_step_threshold_mps2}")
  lines.append(f"- wheel_stop_decel_threshold_mps2: {args.wheel_stop_decel_threshold_mps2}")
  lines.append(f"- wheel_drop_threshold_mps: {args.wheel_drop_threshold_mps}")
  lines.append(f"- hard_decel_threshold_mps2: {args.hard_decel_threshold_mps2}")
  lines.append(f"- hard_cmd_threshold_mps2: {args.hard_cmd_threshold_mps2}")
  lines.append(f"- late_signal_gap_mps: {args.late_signal_gap_mps}")
  lines.append(f"- late_signal_entry_max_mps: {args.late_signal_entry_max_mps}")
  lines.append("")

  return "\n".join(lines)


def default_output_path(summary_json: Path) -> Path:
  return summary_json.parent / "failure_diagnosis.md"


def main() -> int:
  args = parse_args()
  summary_path = Path(args.summary_json).expanduser()
  summary = json.loads(summary_path.read_text())
  rows = load_events(summary, args)

  markdown = build_markdown(summary, rows, args)
  output_path = Path(args.output).expanduser() if args.output else default_output_path(summary_path)
  output_path.parent.mkdir(parents=True, exist_ok=True)
  output_path.write_text(markdown + "\n")

  print(f"[diagnose] focus_source={args.focus_source}")
  print(f"[diagnose] events_analyzed={len(rows)}")
  print(f"[diagnose] output={output_path}")
  return 0


if __name__ == "__main__":
  raise SystemExit(main())
