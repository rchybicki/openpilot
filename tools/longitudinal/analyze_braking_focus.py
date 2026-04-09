#!/usr/bin/env python3
"""Analyze braking comfort and over-correction on locally synced qlogs."""

from __future__ import annotations

import argparse
import json
import sys
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
  sys.path.insert(0, str(REPO_ROOT))

from openpilot.tools.longitudinal.analyze_longitudinal_tracking import SegmentFile, iter_qlog_files, pick_routes, read_bool_attr, read_events, safe_float_attr, step_sample
from openpilot.tools.route_sync.common import DEFAULT_DOWNLOAD_ROOT
from openpilot.tools.stopping.log_schema_helpers import controls_state_enabled, selfdrive_state_engaged

DEFAULT_BRAKING_ROOT = Path.home() / ".comma" / "longitudinal_tuning" / "braking"


@dataclass
class BrakingEvent:
  event_type: str
  route: str
  start_segment: int
  end_segment: int
  start_time_s: float
  end_time_s: float
  duration_s: float
  mean_speed_mps: float
  min_speed_mps: float
  model_brake_peak_mps2: float
  mean_a_target_mps2: float
  mean_accel_cmd_mps2: float
  mean_a_ego_mps2: float
  cmd_overbrake_peak_mps2: float
  cmd_overbrake_p95_mps2: float
  actual_overbrake_peak_mps2: float
  actual_overbrake_p95_mps2: float
  actual_underbrake_peak_mps2: float
  actual_underbrake_p95_mps2: float
  cmd_jerk_max_mps3: float
  cmd_jerk_p95_mps3: float
  actual_jerk_max_mps3: float
  actual_jerk_p95_mps3: float
  controller_deeper_than_model_ratio: float
  actual_deeper_than_model_ratio: float
  lead_present_ratio: float
  lead_distance_start_m: float | None
  lead_distance_end_m: float | None
  lead_v_rel_min_mps: float | None
  lead_a_lead_min_mps2: float | None
  score: float


@dataclass
class EventAggregate:
  event_type: str
  count: int
  total_duration_s: float
  median_cmd_overbrake_peak_mps2: float
  p95_cmd_overbrake_peak_mps2: float
  median_actual_overbrake_peak_mps2: float
  p95_actual_overbrake_peak_mps2: float
  median_actual_jerk_max_mps3: float
  p95_actual_jerk_max_mps3: float
  median_cmd_jerk_max_mps3: float
  p95_cmd_jerk_max_mps3: float


@dataclass
class RouteBrakeSummary:
  route: str
  car_fingerprint: str | None
  git_commit: str | None
  segment_count: int
  duration_s: float
  engaged_duration_s: float
  stop_final_events: list[BrakingEvent]
  lead_decel_events: list[BrakingEvent]


def utc_now_iso() -> str:
  return datetime.now(timezone.utc).replace(microsecond=0).isoformat()


def build_table(headers: list[str], rows: list[list[str]]) -> str:
  widths = [len(header) for header in headers]
  for row in rows:
    for index, cell in enumerate(row):
      widths[index] = max(widths[index], len(cell))

  def fmt(row: list[str]) -> str:
    return "| " + " | ".join(cell.ljust(widths[idx]) for idx, cell in enumerate(row)) + " |"

  separator = "| " + " | ".join("-" * width for width in widths) + " |"
  return "\n".join([fmt(headers), separator, *(fmt(row) for row in rows)])


def parse_args() -> argparse.Namespace:
  parser = argparse.ArgumentParser(description="Analyze braking comfort and over-correction from locally synced qlogs")
  parser.add_argument("--host", required=True, help="Host subfolder under download root, e.g. comma")
  parser.add_argument("--route", action="append", default=[], help="Explicit route ID to analyze (repeatable)")
  parser.add_argument("--download-root", default=str(DEFAULT_DOWNLOAD_ROOT), help=f"Local download root. Default: {DEFAULT_DOWNLOAD_ROOT}")
  parser.add_argument("--analysis-root", default=str(DEFAULT_BRAKING_ROOT),
                      help="Output root for braking analysis artifacts")
  parser.add_argument("--max-routes", type=int, default=5, help="If no explicit routes are passed, analyze this many newest routes")
  parser.add_argument("--min-route-segments", type=int, default=5, help="Minimum locally available segments for auto-selected routes")
  parser.add_argument("--max-segments", type=int, default=0, help="Limit newest segments per route (0 = all)")
  parser.add_argument("--min-speed", type=float, default=0.3, help="Minimum vEgo to keep a braking sample")
  parser.add_argument("--request-max-age", type=float, default=0.5, help="Max age for sampled request values in seconds")
  parser.add_argument("--stop-window", type=float, default=5.0, help="Seconds before standstill hold to score final-stop braking")
  parser.add_argument("--stop-hold-time", type=float, default=0.5, help="Required stopped hold time in seconds")
  parser.add_argument("--standstill-speed", type=float, default=0.12, help="Speed threshold for standstill hold")
  parser.add_argument("--lead-decel-threshold", type=float, default=-1.0, help="Minimum lead acceleration to treat as sudden lead braking")
  parser.add_argument("--lead-vrel-threshold", type=float, default=-0.5, help="Minimum relative speed to treat as closing on a decelerating lead")
  parser.add_argument("--lead-response-window", type=float, default=3.0, help="Seconds after lead-brake onset to score response")
  parser.add_argument("--max-lead-distance", type=float, default=80.0, help="Ignore lead-brake events beyond this distance")
  parser.add_argument("--max-events", type=int, default=12, help="Maximum worst events to keep per event type in the summary")
  parser.add_argument("--output-dir", default=None, help="Explicit output directory path")
  return parser.parse_args()


def percentile_or_zero(values: list[float], percentile: float) -> float:
  return float(np.percentile(values, percentile)) if values else 0.0


def jerk_stats(times: np.ndarray, values: np.ndarray) -> tuple[float, float]:
  if len(times) < 2 or len(values) < 2:
    return 0.0, 0.0
  dt = np.diff(times)
  dv = np.diff(values)
  valid = dt > 1e-6
  if not np.any(valid):
    return 0.0, 0.0
  jerks = np.abs(dv[valid] / dt[valid])
  return float(np.max(jerks)), float(np.percentile(jerks, 95))


def duration_from_mask(times: np.ndarray, mask: np.ndarray) -> float:
  if times.size < 2 or not np.any(mask):
    return 0.0
  dt = np.diff(times, append=times[-1])
  if dt.size >= 2:
    dt[-1] = float(np.median(dt[:-1]))
  return float(np.sum(dt[mask]))


def load_route_arrays(route_segments: list[SegmentFile], max_segments: int) -> dict[str, Any]:
  ordered_segments = sorted(route_segments, key=lambda segment: segment.segment)
  if max_segments > 0:
    ordered_segments = ordered_segments[-max_segments:]

  first_mono_time: float | None = None
  car_fingerprint: str | None = None
  git_commit: str | None = None

  state_times: list[float] = []
  state_segments: list[int] = []
  v_ego_values: list[float] = []
  a_ego_values: list[float] = []
  gas_pressed_values: list[int] = []
  brake_pressed_values: list[int] = []
  standstill_values: list[int] = []

  plan_times: list[float] = []
  a_target_values: list[float] = []
  should_stop_values: list[int] = []

  control_times: list[float] = []
  accel_cmd_values: list[float] = []
  long_active_values: list[int] = []

  controls_state_times: list[float] = []
  controls_enabled_values: list[int] = []
  selfdrive_state_times: list[float] = []
  selfdrive_enabled_values: list[int] = []

  radar_times: list[float] = []
  lead_status_values: list[int] = []
  lead_distance_values: list[float] = []
  lead_v_rel_values: list[float] = []
  lead_a_lead_values: list[float] = []

  for segment in ordered_segments:
    for msg in read_events(segment.path):
      mono_s = msg.logMonoTime * 1e-9
      if first_mono_time is None:
        first_mono_time = mono_s
      t_rel = mono_s - first_mono_time
      which = msg.which()

      if which == "carParams" and car_fingerprint is None:
        try:
          car_fingerprint = str(msg.carParams.carFingerprint)
        except Exception:
          car_fingerprint = None
      elif which == "initData" and git_commit is None:
        try:
          git_commit = str(msg.initData.gitCommit)
        except Exception:
          git_commit = None
      elif which == "carState":
        state_times.append(t_rel)
        state_segments.append(segment.segment)
        v_ego_values.append(float(msg.carState.vEgo))
        a_ego_values.append(float(msg.carState.aEgo))
        gas_pressed_values.append(1 if bool(msg.carState.gasPressed) else 0)
        brake_pressed_values.append(1 if bool(msg.carState.brakePressed) else 0)
        standstill_values.append(1 if bool(msg.carState.standstill) else 0)
      elif which == "longitudinalPlan":
        a_target = safe_float_attr(msg.longitudinalPlan, "aTarget")
        if a_target is not None:
          plan_times.append(t_rel)
          a_target_values.append(a_target)
          should_stop_values.append(1 if bool(msg.longitudinalPlan.shouldStop) else 0)
      elif which == "carControl":
        accel_cmd = safe_float_attr(msg.carControl.actuators, "accel")
        if accel_cmd is not None:
          control_times.append(t_rel)
          accel_cmd_values.append(accel_cmd)
          long_active = read_bool_attr(msg.carControl, "longActive", "enabled")
          long_active_values.append(1 if long_active else 0)
      elif which == "controlsState":
        enabled = controls_state_enabled(msg.controlsState)
        if enabled is not None:
          controls_state_times.append(t_rel)
          controls_enabled_values.append(1 if enabled else 0)
      elif which == "selfdriveState":
        enabled = selfdrive_state_engaged(msg.selfdriveState)
        if enabled is not None:
          selfdrive_state_times.append(t_rel)
          selfdrive_enabled_values.append(1 if enabled else 0)
      elif which == "radarState":
        radar_times.append(t_rel)
        lead_status = False
        lead_distance = np.nan
        lead_v_rel = np.nan
        lead_a_lead = np.nan
        try:
          lead_status = bool(msg.radarState.leadOne.status)
          if lead_status:
            lead_distance = float(msg.radarState.leadOne.dRel)
            lead_v_rel = float(msg.radarState.leadOne.vRel)
            lead_a_lead = float(msg.radarState.leadOne.aLeadK)
        except Exception:
          lead_status = False
        lead_status_values.append(1 if lead_status else 0)
        lead_distance_values.append(lead_distance)
        lead_v_rel_values.append(lead_v_rel)
        lead_a_lead_values.append(lead_a_lead)

  if not state_times:
    raise RuntimeError("No carState samples found")

  times = np.asarray(state_times, dtype=float)
  segments = np.asarray(state_segments, dtype=int)
  v_ego = np.asarray(v_ego_values, dtype=float)
  a_ego = np.asarray(a_ego_values, dtype=float)
  gas_pressed = np.asarray(gas_pressed_values, dtype=bool)
  brake_pressed = np.asarray(brake_pressed_values, dtype=bool)
  standstill = np.asarray(standstill_values, dtype=bool)

  plan_times_np = np.asarray(plan_times, dtype=float)
  a_target_np = np.asarray(a_target_values, dtype=float)
  should_stop_np = np.asarray(should_stop_values, dtype=float)

  control_times_np = np.asarray(control_times, dtype=float)
  accel_cmd_np = np.asarray(accel_cmd_values, dtype=float)
  long_active_np = np.asarray(long_active_values, dtype=float)

  controls_state_times_np = np.asarray(controls_state_times, dtype=float)
  controls_enabled_np = np.asarray(controls_enabled_values, dtype=float)
  selfdrive_state_times_np = np.asarray(selfdrive_state_times, dtype=float)
  selfdrive_enabled_np = np.asarray(selfdrive_enabled_values, dtype=float)

  radar_times_np = np.asarray(radar_times, dtype=float)
  lead_status_np = np.asarray(lead_status_values, dtype=float)
  lead_distance_np = np.asarray(lead_distance_values, dtype=float)
  lead_v_rel_np = np.asarray(lead_v_rel_values, dtype=float)
  lead_a_lead_np = np.asarray(lead_a_lead_values, dtype=float)

  data = {
    "route": ordered_segments[0].route,
    "segment_count": len(ordered_segments),
    "car_fingerprint": car_fingerprint,
    "git_commit": git_commit,
    "times": times,
    "segments": segments,
    "v_ego": v_ego,
    "a_ego": a_ego,
    "gas_pressed": gas_pressed,
    "brake_pressed": brake_pressed,
    "standstill": standstill,
    "a_target": step_sample(plan_times_np, a_target_np, times, max_age_s=0.5),
    "should_stop": step_sample(plan_times_np, should_stop_np, times, max_age_s=0.5, default_value=0.0) > 0.5,
    "accel_cmd": step_sample(control_times_np, accel_cmd_np, times, max_age_s=0.5),
    "long_active": step_sample(control_times_np, long_active_np, times, max_age_s=0.5, default_value=0.0) > 0.5,
    "controls_enabled": step_sample(controls_state_times_np, controls_enabled_np, times, max_age_s=0.5, default_value=0.0) > 0.5,
    "selfdrive_enabled": step_sample(selfdrive_state_times_np, selfdrive_enabled_np, times, max_age_s=0.5, default_value=0.0) > 0.5,
    "lead_status": step_sample(radar_times_np, lead_status_np, times, max_age_s=0.5, default_value=0.0) > 0.5,
    "lead_distance": step_sample(radar_times_np, lead_distance_np, times, max_age_s=0.5),
    "lead_v_rel": step_sample(radar_times_np, lead_v_rel_np, times, max_age_s=0.5),
    "lead_a_lead": step_sample(radar_times_np, lead_a_lead_np, times, max_age_s=0.5),
  }
  data["engaged"] = data["long_active"] | data["controls_enabled"] | data["selfdrive_enabled"]
  data["pedal_override"] = gas_pressed | brake_pressed
  return data


def make_braking_event(event_type: str, route_data: dict[str, Any], start_idx: int, end_idx: int) -> BrakingEvent | None:
  if end_idx <= start_idx:
    return None

  window = slice(start_idx, end_idx + 1)
  times = route_data["times"][window]
  v_ego = route_data["v_ego"][window]
  a_ego = route_data["a_ego"][window]
  a_target = route_data["a_target"][window]
  accel_cmd = route_data["accel_cmd"][window]
  lead_status = route_data["lead_status"][window]
  lead_distance = route_data["lead_distance"][window]
  lead_v_rel = route_data["lead_v_rel"][window]
  lead_a_lead = route_data["lead_a_lead"][window]

  valid = np.isfinite(a_target) & np.isfinite(accel_cmd) & np.isfinite(a_ego)
  if int(np.sum(valid)) < 5:
    return None

  braking_samples = valid & (a_target <= -0.05)
  if int(np.sum(braking_samples)) < 5:
    return None

  a_target_valid = a_target[braking_samples]
  accel_cmd_valid = accel_cmd[braking_samples]
  a_ego_valid = a_ego[braking_samples]

  cmd_overbrake = np.maximum(a_target_valid - accel_cmd_valid, 0.0)
  actual_overbrake = np.maximum(a_target_valid - a_ego_valid, 0.0)
  actual_underbrake = np.maximum(a_ego_valid - a_target_valid, 0.0)

  cmd_jerk_max, cmd_jerk_p95 = jerk_stats(times[valid & np.isfinite(accel_cmd)], accel_cmd[valid & np.isfinite(accel_cmd)])
  actual_jerk_max, actual_jerk_p95 = jerk_stats(times[valid], a_ego[valid])

  controller_deeper_ratio = float(np.mean((accel_cmd_valid < (a_target_valid - 0.05)).astype(float)))
  actual_deeper_ratio = float(np.mean((a_ego_valid < (a_target_valid - 0.05)).astype(float)))
  lead_present_ratio = float(np.mean(lead_status.astype(float)))

  lead_start = float(lead_distance[np.flatnonzero(np.isfinite(lead_distance))[0]]) if np.any(np.isfinite(lead_distance)) else None
  lead_end = float(lead_distance[np.flatnonzero(np.isfinite(lead_distance))[-1]]) if np.any(np.isfinite(lead_distance)) else None
  lead_v_rel_min = float(np.nanmin(lead_v_rel)) if np.any(np.isfinite(lead_v_rel)) else None
  lead_a_lead_min = float(np.nanmin(lead_a_lead)) if np.any(np.isfinite(lead_a_lead)) else None

  score = (
    actual_jerk_max
    + 0.8 * cmd_jerk_max
    + 2.0 * float(np.max(cmd_overbrake))
    + 1.5 * float(np.max(actual_overbrake))
  )

  return BrakingEvent(
    event_type=event_type,
    route=route_data["route"],
    start_segment=int(route_data["segments"][start_idx]),
    end_segment=int(route_data["segments"][end_idx]),
    start_time_s=float(route_data["times"][start_idx]),
    end_time_s=float(route_data["times"][end_idx]),
    duration_s=float(route_data["times"][end_idx] - route_data["times"][start_idx]),
    mean_speed_mps=float(np.mean(v_ego[braking_samples])),
    min_speed_mps=float(np.min(v_ego[braking_samples])),
    model_brake_peak_mps2=float(np.min(a_target_valid)),
    mean_a_target_mps2=float(np.mean(a_target_valid)),
    mean_accel_cmd_mps2=float(np.mean(accel_cmd_valid)),
    mean_a_ego_mps2=float(np.mean(a_ego_valid)),
    cmd_overbrake_peak_mps2=float(np.max(cmd_overbrake)),
    cmd_overbrake_p95_mps2=float(np.percentile(cmd_overbrake, 95)),
    actual_overbrake_peak_mps2=float(np.max(actual_overbrake)),
    actual_overbrake_p95_mps2=float(np.percentile(actual_overbrake, 95)),
    actual_underbrake_peak_mps2=float(np.max(actual_underbrake)),
    actual_underbrake_p95_mps2=float(np.percentile(actual_underbrake, 95)),
    cmd_jerk_max_mps3=cmd_jerk_max,
    cmd_jerk_p95_mps3=cmd_jerk_p95,
    actual_jerk_max_mps3=actual_jerk_max,
    actual_jerk_p95_mps3=actual_jerk_p95,
    controller_deeper_than_model_ratio=controller_deeper_ratio,
    actual_deeper_than_model_ratio=actual_deeper_ratio,
    lead_present_ratio=lead_present_ratio,
    lead_distance_start_m=lead_start,
    lead_distance_end_m=lead_end,
    lead_v_rel_min_mps=lead_v_rel_min,
    lead_a_lead_min_mps2=lead_a_lead_min,
    score=score,
  )


def find_stop_final_events(route_data: dict[str, Any], args: argparse.Namespace) -> list[BrakingEvent]:
  times = route_data["times"]
  engaged = route_data["engaged"]
  pedal_override = route_data["pedal_override"]
  standstill = route_data["standstill"]
  v_ego = route_data["v_ego"]
  should_stop = route_data["should_stop"]

  stopped = standstill | (v_ego <= args.standstill_speed)
  events: list[BrakingEvent] = []
  index = 1
  while index < len(times):
    if not (stopped[index] and not stopped[index - 1]):
      index += 1
      continue

    hold_start = index
    hold_end = hold_start
    while hold_end + 1 < len(times) and stopped[hold_end + 1]:
      hold_end += 1
      if (times[hold_end] - times[hold_start]) >= args.stop_hold_time:
        break

    if (times[hold_end] - times[hold_start]) < args.stop_hold_time:
      index += 1
      continue

    start_time = times[hold_start] - args.stop_window
    start_idx = int(np.searchsorted(times, start_time, side="left"))
    if start_idx >= hold_start:
      index = hold_end + 1
      continue

    valid_window = engaged[start_idx:hold_start + 1] & ~pedal_override[start_idx:hold_start + 1]
    if float(np.mean(valid_window.astype(float))) < 0.7:
      index = hold_end + 1
      continue

    if not np.any(should_stop[start_idx:hold_start + 1] | (route_data["a_target"][start_idx:hold_start + 1] <= -0.1)):
      index = hold_end + 1
      continue

    event = make_braking_event("stop_final_5s", route_data, start_idx, hold_start)
    if event is not None:
      events.append(event)
    index = hold_end + 1

  return events


def find_lead_decel_events(route_data: dict[str, Any], args: argparse.Namespace) -> list[BrakingEvent]:
  times = route_data["times"]
  engaged = route_data["engaged"]
  pedal_override = route_data["pedal_override"]
  v_ego = route_data["v_ego"]
  lead_status = route_data["lead_status"]
  lead_distance = route_data["lead_distance"]
  lead_v_rel = route_data["lead_v_rel"]
  lead_a_lead = route_data["lead_a_lead"]

  strong_lead_decel = (
    engaged
    & ~pedal_override
    & (v_ego >= 2.5)
    & lead_status
    & np.isfinite(lead_distance)
    & (lead_distance <= args.max_lead_distance)
    & np.isfinite(lead_v_rel)
    & np.isfinite(lead_a_lead)
    & (lead_v_rel <= args.lead_vrel_threshold)
    & (lead_a_lead <= args.lead_decel_threshold)
  )

  events: list[BrakingEvent] = []
  index = 1
  while index < len(times):
    if not (strong_lead_decel[index] and not strong_lead_decel[index - 1]):
      index += 1
      continue

    start_idx = index
    end_time = times[start_idx] + args.lead_response_window
    end_idx = int(min(len(times) - 1, np.searchsorted(times, end_time, side="right") - 1))
    event = make_braking_event("lead_decel_response", route_data, start_idx, end_idx)
    if event is not None:
      events.append(event)

    while index < len(times) and times[index] <= end_time:
      index += 1

  return events


def aggregate_events(event_type: str, events: list[BrakingEvent]) -> EventAggregate | None:
  if not events:
    return None
  cmd_overbrake_peaks = [event.cmd_overbrake_peak_mps2 for event in events]
  actual_overbrake_peaks = [event.actual_overbrake_peak_mps2 for event in events]
  actual_jerk_max = [event.actual_jerk_max_mps3 for event in events]
  cmd_jerk_max = [event.cmd_jerk_max_mps3 for event in events]
  return EventAggregate(
    event_type=event_type,
    count=len(events),
    total_duration_s=float(sum(event.duration_s for event in events)),
    median_cmd_overbrake_peak_mps2=float(np.median(cmd_overbrake_peaks)),
    p95_cmd_overbrake_peak_mps2=percentile_or_zero(cmd_overbrake_peaks, 95),
    median_actual_overbrake_peak_mps2=float(np.median(actual_overbrake_peaks)),
    p95_actual_overbrake_peak_mps2=percentile_or_zero(actual_overbrake_peaks, 95),
    median_actual_jerk_max_mps3=float(np.median(actual_jerk_max)),
    p95_actual_jerk_max_mps3=percentile_or_zero(actual_jerk_max, 95),
    median_cmd_jerk_max_mps3=float(np.median(cmd_jerk_max)),
    p95_cmd_jerk_max_mps3=percentile_or_zero(cmd_jerk_max, 95),
  )


def render_summary(summary: dict[str, Any], output_dir: Path) -> str:
  lines: list[str] = []
  lines.append("# Braking Focus Summary")
  lines.append("")
  lines.append(f"- Generated: `{summary['generated_at_utc']}`")
  lines.append(f"- Host: `{summary['host']}`")
  lines.append(f"- Car fingerprint(s): `{', '.join(summary['car_fingerprints'])}`")
  lines.append(f"- Routes analyzed: `{', '.join(summary['routes'])}`")
  lines.append(f"- JSON: `{output_dir / 'summary.json'}`")
  lines.append("")

  route_rows = [
    [
      route["route"],
      str(route["segment_count"]),
      f"{route['duration_s']:.1f}",
      f"{route['engaged_duration_s']:.1f}",
      str(len(route["stop_final_events"])),
      str(len(route["lead_decel_events"])),
    ]
    for route in summary["route_summaries"]
  ]
  lines.append("## Route Coverage")
  lines.append("")
  lines.append(build_table(["route", "segments", "duration_s", "engaged_s", "stop_events", "lead_events"], route_rows))
  lines.append("")

  aggregate_rows: list[list[str]] = []
  for aggregate in summary["aggregates"]:
    aggregate_rows.append([
      aggregate["event_type"],
      str(aggregate["count"]),
      f"{aggregate['total_duration_s']:.1f}",
      f"{aggregate['median_cmd_overbrake_peak_mps2']:.3f}",
      f"{aggregate['p95_cmd_overbrake_peak_mps2']:.3f}",
      f"{aggregate['median_actual_overbrake_peak_mps2']:.3f}",
      f"{aggregate['p95_actual_overbrake_peak_mps2']:.3f}",
      f"{aggregate['median_actual_jerk_max_mps3']:.3f}",
      f"{aggregate['p95_actual_jerk_max_mps3']:.3f}",
    ])
  if aggregate_rows:
    lines.append("## Aggregate Braking Metrics")
    lines.append("")
    lines.append(build_table(
      ["event_type", "count", "seconds", "cmd_over_med", "cmd_over_p95", "actual_over_med", "actual_over_p95", "jerk_med", "jerk_p95"],
      aggregate_rows,
    ))
    lines.append("")

  for key, title in (("worst_stop_events", "Worst Final-Stop Events"), ("worst_lead_events", "Worst Lead-Decel Response Events")):
    events = summary[key]
    if not events:
      continue
    rows: list[list[str]] = []
    for event in events:
      rows.append([
        event["route"],
        f"{event['start_segment']}->{event['end_segment']}",
        f"{event['start_time_s']:.1f}",
        f"{event['duration_s']:.2f}",
        f"{event['mean_speed_mps']:.2f}",
        f"{event['model_brake_peak_mps2']:.2f}",
        f"{event['cmd_overbrake_peak_mps2']:.2f}",
        f"{event['actual_overbrake_peak_mps2']:.2f}",
        f"{event['actual_underbrake_peak_mps2']:.2f}",
        f"{event['actual_jerk_max_mps3']:.2f}",
      ])
    lines.append(f"## {title}")
    lines.append("")
    lines.append(build_table(
      ["route", "segments", "start_s", "dur_s", "speed", "model_peak", "cmd_over", "actual_over", "actual_under", "jerk_max"],
      rows,
    ))
    lines.append("")

  return "\n".join(lines).rstrip() + "\n"


def main() -> int:
  args = parse_args()
  download_root = Path(args.download_root).expanduser()
  analysis_root = Path(args.analysis_root).expanduser()

  segments = iter_qlog_files(download_root, args.host)
  routes = pick_routes(segments, args.route, args.max_routes, args.min_route_segments)
  grouped: dict[str, list[SegmentFile]] = {}
  for segment in segments:
    grouped.setdefault(segment.route, []).append(segment)

  route_summaries: list[RouteBrakeSummary] = []
  for route in routes:
    route_data = load_route_arrays(grouped[route], args.max_segments)
    stop_events = find_stop_final_events(route_data, args)
    lead_events = find_lead_decel_events(route_data, args)
    route_summaries.append(RouteBrakeSummary(
      route=route,
      car_fingerprint=route_data["car_fingerprint"],
      git_commit=route_data["git_commit"],
      segment_count=route_data["segment_count"],
      duration_s=float(route_data["times"][-1] - route_data["times"][0]) if len(route_data["times"]) > 1 else 0.0,
      engaged_duration_s=duration_from_mask(route_data["times"], route_data["engaged"]),
      stop_final_events=stop_events,
      lead_decel_events=lead_events,
    ))

  if args.output_dir:
    output_dir = Path(args.output_dir).expanduser()
  else:
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    output_dir = analysis_root / args.host / stamp
  output_dir.mkdir(parents=True, exist_ok=True)

  all_stop_events = [event for route in route_summaries for event in route.stop_final_events]
  all_lead_events = [event for route in route_summaries for event in route.lead_decel_events]
  aggregates = [item for item in (
    aggregate_events("stop_final_5s", all_stop_events),
    aggregate_events("lead_decel_response", all_lead_events),
  ) if item is not None]

  summary_data: dict[str, Any] = {
    "generated_at_utc": utc_now_iso(),
    "host": args.host,
    "routes": [route.route for route in route_summaries],
    "car_fingerprints": sorted({route.car_fingerprint for route in route_summaries if route.car_fingerprint}) or ["unknown"],
    "aggregates": [asdict(item) for item in aggregates],
    "route_summaries": [
      {
        **asdict(route),
        "stop_final_events": [asdict(event) for event in route.stop_final_events],
        "lead_decel_events": [asdict(event) for event in route.lead_decel_events],
      }
      for route in route_summaries
    ],
    "worst_stop_events": [asdict(event) for event in sorted(all_stop_events, key=lambda item: item.score, reverse=True)[:args.max_events]],
    "worst_lead_events": [asdict(event) for event in sorted(all_lead_events, key=lambda item: item.score, reverse=True)[:args.max_events]],
  }

  summary_json = output_dir / "summary.json"
  summary_json.write_text(json.dumps(summary_data, indent=2, sort_keys=True) + "\n")

  summary_md = output_dir / "summary.md"
  summary_md.write_text(render_summary(summary_data, output_dir))

  print(f"[braking-focus] host={args.host}")
  print(f"[braking-focus] routes={', '.join(summary_data['routes'])}")
  print(f"[braking-focus] summary_json={summary_json}")
  print(f"[braking-focus] summary_md={summary_md}")
  return 0


if __name__ == "__main__":
  raise SystemExit(main())
