#!/usr/bin/env python3
"""Analyze stopping behavior from locally synced qlogs and generate review artifacts."""

from __future__ import annotations

import argparse
import bz2
import json
import sys
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from pathlib import Path
from statistics import median
from typing import Any

import numpy as np
import plotly.graph_objects as go
from plotly.subplots import make_subplots

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
  sys.path.insert(0, str(REPO_ROOT))

from cereal import log as capnp_log
from openpilot.tools.stopping.log_schema_helpers import controls_state_enabled, selfdrive_state_engaged

DEFAULT_DOWNLOAD_ROOT = Path.home() / ".comma" / "stopping_behavior" / "downloads"
DEFAULT_ANALYSIS_ROOT = Path.home() / ".comma" / "stopping_behavior" / "analysis"
DEFAULT_SETTINGS_DIR = Path.home() / ".comma" / "stopping_behavior" / "settings"

LONG_STATE_MAP = {"off": 0, "pid": 1, "stopping": 2, "starting": 3}
LONG_STATE_LABELS = {value: key for key, value in LONG_STATE_MAP.items()}
EVENT_MODES = ("engaged_signal", "speed_transition", "hybrid")


@dataclass
class SegmentFile:
  route: str
  segment: int
  path: Path
  mtime: float


@dataclass
class Sample:
  t: float
  segment: int
  v_ego: float
  v_wheel_avg: float | None
  a_ego: float
  standstill: bool
  brake_pressed: bool
  gas_pressed: bool
  enabled: bool
  long_state: str
  long_state_cmd: str
  should_stop: bool
  a_target: float | None
  distance_to_stop_target_m: float | None
  accel_cmd: float | None
  lead_status: bool
  lead_d_rel_m: float | None
  forcing_stop: bool
  red_light: bool

  @property
  def stop_signal(self) -> bool:
    return self.enabled and (self.should_stop or self.long_state == "stopping" or self.long_state_cmd == "stopping")


@dataclass
class StopEvent:
  event_id: int
  event_source: str
  start_index: int
  stop_index: int
  stop_hold_index: int
  start_segment: int
  stop_segment: int
  start_time_s: float
  stop_time_s: float
  stop_hold_time_s: float
  duration_s: float
  approach_speed_mps: float
  entry_speed_mps: float
  min_speed_mps: float
  min_a_ego_mps2: float
  min_accel_cmd_mps2: float | None
  min_a_target_mps2: float | None
  distance_traveled_m: float
  rollout_distance_from_2mps_m: float
  should_stop_to_stopping_s: float | None
  lead_distance_stop_entry_m: float | None
  lead_distance_hold_m: float | None
  entry_stop_jerk_mps3: float | None
  entry_stop_accel_step_mps2: float | None
  entry_stop_cmd_jerk_mps3: float | None
  entry_stop_cmd_step_mps2: float | None
  end_stop_jerk_mps3: float | None
  end_stop_accel_step_mps2: float | None
  end_stop_cmd_jerk_mps3: float | None
  end_stop_cmd_step_mps2: float | None
  wheel_stop_decel_mps2: float | None
  wheel_speed_drop_150ms_mps: float | None
  reaccel_before_hold: bool
  low_speed_oscillation_count: int
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
  forcing_stop_seen: bool
  red_light_seen: bool
  brake_pressed_ratio: float
  enabled_ratio: float
  stop_signal_ratio: float
  should_stop_ratio: float
  stopping_state_ratio: float
  stopping_state_cmd_ratio: float
  graph_file: str


def utc_now_iso() -> str:
  return datetime.now(timezone.utc).replace(microsecond=0).isoformat()


def short_exception(exc: Exception) -> str:
  text = str(exc).strip()
  return text.splitlines()[0] if text else exc.__class__.__name__


def parse_args() -> argparse.Namespace:
  parser = argparse.ArgumentParser(description="Analyze stopping behavior from locally synced qlogs")
  parser.add_argument("--host", required=True, help="Host subfolder under download root, e.g. commawifi")
  parser.add_argument("--route", default=None, help="Route ID to analyze. Default: newest route under host")
  parser.add_argument("--download-root", default=str(DEFAULT_DOWNLOAD_ROOT),
                      help=f"Local download root. Default: {DEFAULT_DOWNLOAD_ROOT}")
  parser.add_argument("--analysis-root", default=str(DEFAULT_ANALYSIS_ROOT),
                      help=f"Output root for analysis artifacts. Default: {DEFAULT_ANALYSIS_ROOT}")
  parser.add_argument("--settings-dir", default=str(DEFAULT_SETTINGS_DIR),
                      help=f"Directory for settings snapshots. Default: {DEFAULT_SETTINGS_DIR}")
  parser.add_argument("--settings-file", default=None, help="Optional explicit settings snapshot JSON file")
  parser.add_argument("--max-segments", type=int, default=0, help="Limit number of newest segments (0 = all)")
  parser.add_argument("--min-entry-speed", type=float, default=2.0, help="Minimum speed to treat as a stop approach")
  parser.add_argument("--standstill-speed", type=float, default=0.12, help="Speed threshold for standstill hold")
  parser.add_argument("--hold-time", type=float, default=0.5, help="Required standstill hold time in seconds")
  parser.add_argument("--max-stop-search", type=float, default=35.0,
                      help="Max seconds from stop-signal start to standstill hold")
  parser.add_argument("--event-mode", choices=EVENT_MODES, default="hybrid",
                      help="Event detector: engaged signal only, speed transition only, or hybrid")
  parser.add_argument("--require-enabled-speed-events", action="store_true",
                      help="In speed/hybrid mode, keep events only if at least one sample is enabled")
  parser.add_argument("--entry-lookback", type=float, default=8.0,
                      help="Seconds of history used to evaluate stop entry speed")
  parser.add_argument("--pre-window", type=float, default=8.0, help="Seconds before event start shown in graph")
  parser.add_argument("--post-window", type=float, default=6.0, help="Seconds after stop hold shown in graph")
  parser.add_argument("--max-events", type=int, default=0, help="Maximum number of detected events to plot (0 = all)")
  parser.add_argument("--output-dir", default=None, help="Explicit output directory path")
  return parser.parse_args()


def iter_qlog_files(download_root: Path, host: str) -> list[SegmentFile]:
  host_root = (download_root / host).expanduser()
  if not host_root.exists():
    raise FileNotFoundError(f"Host download directory not found: {host_root}")

  segments: list[SegmentFile] = []
  for qlog_path in host_root.rglob("qlog"):
    segment_name = qlog_path.parent.name
    if "--" not in segment_name:
      continue
    route, suffix = segment_name.rsplit("--", 1)
    try:
      segment = int(suffix)
    except ValueError:
      continue
    segments.append(SegmentFile(route=route, segment=segment, path=qlog_path, mtime=qlog_path.stat().st_mtime))

  if not segments:
    raise RuntimeError(f"No qlog files found under {host_root}")
  return segments


def pick_route(segments: list[SegmentFile], route_override: str | None) -> str:
  if route_override:
    return route_override

  newest_by_route: dict[str, float] = {}
  for seg in segments:
    newest_by_route[seg.route] = max(newest_by_route.get(seg.route, 0.0), seg.mtime)

  def route_prefix_key(route: str) -> tuple[int, int]:
    # Routes are often prefixed with an incrementing hex counter (e.g., 000006df--...).
    # Prefer that ordering to avoid local file mtime skew from repeated sync/copies.
    prefix = route.split("--", 1)[0] if "--" in route else route
    try:
      return 1, int(prefix, 16)
    except ValueError:
      return 0, 0

  return max(newest_by_route.items(), key=lambda item: (route_prefix_key(item[0]), item[1], item[0]))[0]


def read_events(path: Path):
  try:
    data = path.read_bytes()
  except Exception as exc:
    print(f"[analysis] warning: failed to read {path}: {short_exception(exc)}", file=sys.stderr)
    return

  try:
    if path.suffix == ".bz2" or data.startswith(b"BZh9"):
      data = bz2.decompress(data)
  except Exception as exc:
    print(f"[analysis] warning: failed to decompress {path}: {short_exception(exc)}", file=sys.stderr)
    return

  try:
    reader = capnp_log.Event.read_multiple_bytes(data)
  except Exception as exc:
    print(f"[analysis] warning: failed to decode {path}: {short_exception(exc)}", file=sys.stderr)
    return

  while True:
    try:
      yield next(reader)
    except StopIteration:
      break
    except Exception as exc:
      print(f"[analysis] warning: truncated/corrupt events in {path}: {short_exception(exc)}", file=sys.stderr)
      break


def load_samples(route_segments: list[SegmentFile]) -> list[Sample]:
  samples: list[Sample] = []
  first_mono_time: float | None = None

  enabled = False
  controls_enabled_signal_seen = False
  long_state = "off"
  long_state_cmd = "off"
  should_stop = False
  a_target: float | None = None
  distance_to_stop_target_m: float | None = None
  accel_cmd: float | None = None
  lead_status = False
  lead_d_rel_m: float | None = None
  forcing_stop = False
  red_light = False

  for seg in route_segments:
    for msg in read_events(seg.path):
      mono_s = msg.logMonoTime * 1e-9
      if first_mono_time is None:
        first_mono_time = mono_s
      t_rel = mono_s - first_mono_time

      which = msg.which()
      if which == "controlsState":
        state = msg.controlsState
        state_enabled = controls_state_enabled(state)
        if state_enabled is not None:
          enabled = state_enabled
          controls_enabled_signal_seen = True
        long_state = str(state.longControlState)
      elif which == "selfdriveState":
        state_enabled = selfdrive_state_engaged(msg.selfdriveState)
        if state_enabled is not None and not controls_enabled_signal_seen:
          enabled = state_enabled
      elif which == "longitudinalPlan":
        plan = msg.longitudinalPlan
        should_stop = bool(plan.shouldStop)
        a_target = float(plan.aTarget)
        try:
          distance_to_stop_target_m = float(plan.distanceToStopTarget)
        except (AttributeError, TypeError):
          distance_to_stop_target_m = None
      elif which == "frogpilotPlan":
        fp_plan = msg.frogpilotPlan
        forcing_stop = bool(fp_plan.forcingStop)
        red_light = bool(fp_plan.redLight)
      elif which == "carControl":
        control = msg.carControl
        accel_cmd = float(control.actuators.accel)
        long_state_cmd = str(control.actuators.longControlState)
      elif which == "radarState":
        radar = msg.radarState
        try:
          lead_status = bool(radar.leadOne.status)
          lead_d_rel_m = float(radar.leadOne.dRel) if lead_status else None
        except Exception:
          lead_status = False
          lead_d_rel_m = None
      elif which == "carState":
        car_state = msg.carState
        v_wheel_avg: float | None = None
        try:
          wheel_speeds = car_state.wheelSpeeds
          v_wheel_avg = float((wheel_speeds.fl + wheel_speeds.fr + wheel_speeds.rl + wheel_speeds.rr) * 0.25)
        except Exception:
          v_wheel_avg = None
        samples.append(
          Sample(
            t=t_rel,
            segment=seg.segment,
            v_ego=float(car_state.vEgo),
            v_wheel_avg=v_wheel_avg,
            a_ego=float(car_state.aEgo),
            standstill=bool(car_state.standstill),
            brake_pressed=bool(car_state.brakePressed),
            gas_pressed=bool(car_state.gasPressed),
            enabled=enabled,
            long_state=long_state,
            long_state_cmd=long_state_cmd,
            should_stop=should_stop,
            a_target=a_target,
            distance_to_stop_target_m=distance_to_stop_target_m,
            accel_cmd=accel_cmd,
            lead_status=lead_status,
            lead_d_rel_m=lead_d_rel_m,
            forcing_stop=forcing_stop,
            red_light=red_light,
          )
        )

  return samples


def find_signal_stop_events(
  samples: list[Sample],
  min_entry_speed: float,
  entry_lookback: float,
  standstill_speed: float,
  hold_time: float,
  max_stop_search: float,
) -> list[tuple[int, int, int, float]]:
  events: list[tuple[int, int, int, float]] = []
  if len(samples) < 3:
    return events

  index = 1
  while index < len(samples):
    start_candidate = samples[index].stop_signal and not samples[index - 1].stop_signal
    if not start_candidate:
      index += 1
      continue

    lookback_index = index
    while lookback_index > 0 and (samples[index].t - samples[lookback_index - 1].t) <= entry_lookback:
      lookback_index -= 1
    entry_speed = max(item.v_ego for item in samples[lookback_index:index + 1])
    if entry_speed < min_entry_speed:
      index += 1
      continue

    start_index = index
    stop_index: int | None = None
    hold_index: int | None = None
    scan = start_index
    while scan < len(samples):
      elapsed = samples[scan].t - samples[start_index].t
      if elapsed > max_stop_search:
        break

      current = samples[scan]
      if current.standstill and abs(current.v_ego) <= standstill_speed:
        hold_start_time = current.t
        hold_scan = scan
        while hold_scan < len(samples):
          held = samples[hold_scan].standstill and abs(samples[hold_scan].v_ego) <= standstill_speed
          if not held:
            break
          if samples[hold_scan].t - hold_start_time >= hold_time:
            stop_index = scan
            hold_index = hold_scan
            break
          hold_scan += 1
        if stop_index is not None:
          break
      scan += 1

    if stop_index is not None and hold_index is not None:
      events.append((start_index, stop_index, hold_index, entry_speed))
      index = hold_index + 1
    else:
      index = start_index + 1

  return events


def is_stopped(sample: Sample, standstill_speed: float) -> bool:
  return sample.standstill or abs(sample.v_ego) <= standstill_speed


def find_speed_stop_events(
  samples: list[Sample],
  min_entry_speed: float,
  entry_lookback: float,
  standstill_speed: float,
  hold_time: float,
  max_stop_search: float,
  require_enabled: bool,
) -> list[tuple[int, int, int, float]]:
  events: list[tuple[int, int, int, float]] = []
  if len(samples) < 3:
    return events

  index = 1
  while index < len(samples):
    current = samples[index]
    previous = samples[index - 1]
    if not (is_stopped(current, standstill_speed) and not is_stopped(previous, standstill_speed)):
      index += 1
      continue

    hold_start = index
    hold_index: int | None = None
    hold_scan = hold_start
    while hold_scan < len(samples):
      if not is_stopped(samples[hold_scan], standstill_speed):
        break
      if samples[hold_scan].t - samples[hold_start].t >= hold_time:
        hold_index = hold_scan
        break
      hold_scan += 1

    if hold_index is None:
      index += 1
      continue

    search_start = hold_start
    while search_start > 0 and (samples[hold_start].t - samples[search_start - 1].t) <= max_stop_search:
      search_start -= 1

    if require_enabled and not any(item.enabled for item in samples[search_start:hold_index + 1]):
      index = hold_index + 1
      continue

    lookback_start = hold_start
    while lookback_start > search_start and (samples[hold_start].t - samples[lookback_start - 1].t) <= entry_lookback:
      lookback_start -= 1

    approach_window = samples[lookback_start:hold_start + 1]
    approach_speed = max(item.v_ego for item in approach_window)
    if approach_speed < min_entry_speed:
      index = hold_index + 1
      continue

    relative_peak_idx = max(range(len(approach_window)), key=lambda offset: approach_window[offset].v_ego)
    start_index = lookback_start + relative_peak_idx
    stop_index = hold_start

    events.append((start_index, stop_index, hold_index, approach_speed))
    index = hold_index + 1

  return events


def merge_event_ranges(
  signal_events: list[tuple[int, int, int, float]],
  speed_events: list[tuple[int, int, int, float]],
  hold_merge_tolerance: int = 2,
) -> list[tuple[int, int, int, float, str]]:
  merged: list[tuple[int, int, int, float, str]] = [
    (start_idx, stop_idx, hold_idx, approach_speed, "signal")
    for (start_idx, stop_idx, hold_idx, approach_speed) in signal_events
  ]

  for start_idx, stop_idx, hold_idx, approach_speed in speed_events:
    matched = False
    for existing_idx, existing in enumerate(merged):
      if abs(hold_idx - existing[2]) <= hold_merge_tolerance:
        if start_idx < existing[0]:
          merged[existing_idx] = (start_idx, stop_idx, hold_idx, approach_speed, "hybrid")
        matched = True
        break
    if not matched:
      merged.append((start_idx, stop_idx, hold_idx, approach_speed, "speed"))

  merged.sort(key=lambda item: item[0])
  return merged


def find_stop_events_with_source(
  samples: list[Sample],
  min_entry_speed: float,
  entry_lookback: float,
  standstill_speed: float,
  hold_time: float,
  max_stop_search: float,
  event_mode: str,
  require_enabled_speed_events: bool,
) -> list[tuple[int, int, int, float, str]]:
  signal_events = find_signal_stop_events(
    samples=samples,
    min_entry_speed=min_entry_speed,
    entry_lookback=entry_lookback,
    standstill_speed=standstill_speed,
    hold_time=hold_time,
    max_stop_search=max_stop_search,
  )
  speed_events = find_speed_stop_events(
    samples=samples,
    min_entry_speed=min_entry_speed,
    entry_lookback=entry_lookback,
    standstill_speed=standstill_speed,
    hold_time=hold_time,
    max_stop_search=max_stop_search,
    require_enabled=require_enabled_speed_events,
  )

  if event_mode == "engaged_signal":
    return [(start_idx, stop_idx, hold_idx, approach_speed, "signal") for (start_idx, stop_idx, hold_idx, approach_speed) in signal_events]
  if event_mode == "speed_transition":
    return [(start_idx, stop_idx, hold_idx, approach_speed, "speed") for (start_idx, stop_idx, hold_idx, approach_speed) in speed_events]
  if event_mode == "hybrid":
    return merge_event_ranges(signal_events, speed_events)
  raise ValueError(f"Unsupported event mode: {event_mode}")


def find_stop_events(
  samples: list[Sample],
  min_entry_speed: float,
  entry_lookback: float,
  standstill_speed: float,
  hold_time: float,
  max_stop_search: float,
) -> list[tuple[int, int, int, float]]:
  # Backward compatibility for callers that import this function directly.
  return [
    (start_idx, stop_idx, hold_idx, approach_speed)
    for start_idx, stop_idx, hold_idx, approach_speed, _source in find_stop_events_with_source(
      samples=samples,
      min_entry_speed=min_entry_speed,
      entry_lookback=entry_lookback,
      standstill_speed=standstill_speed,
      hold_time=hold_time,
      max_stop_search=max_stop_search,
      event_mode="engaged_signal",
      require_enabled_speed_events=True,
    )
  ]


def integrate_distance(samples: list[Sample], start_idx: int, end_idx: int) -> float:
  if end_idx <= start_idx:
    return 0.0
  total = 0.0
  for idx in range(start_idx + 1, end_idx + 1):
    prev = samples[idx - 1]
    cur = samples[idx]
    dt = max(cur.t - prev.t, 0.0)
    total += max((prev.v_ego + cur.v_ego) * 0.5, 0.0) * dt
  return total


def rollout_distance_from_speed(samples: list[Sample], start_idx: int, hold_idx: int, speed_threshold: float = 2.0) -> float:
  rollout_start = hold_idx
  for idx in range(start_idx, hold_idx + 1):
    if samples[idx].v_ego <= speed_threshold:
      rollout_start = idx
      break
  return integrate_distance(samples, rollout_start, hold_idx)


def first_time(samples: list[Sample], start_idx: int, end_idx: int, predicate) -> float | None:
  for idx in range(start_idx, end_idx + 1):
    if predicate(samples[idx]):
      return samples[idx].t
  return None


def first_index(samples: list[Sample], start_idx: int, end_idx: int, predicate) -> int | None:
  for idx in range(start_idx, end_idx + 1):
    if predicate(samples[idx]):
      return idx
  return None


def lead_distance_metric(
  samples: list[Sample],
  start_idx: int,
  end_idx: int,
  *,
  anchor_idx: int,
  pre_window_s: float,
  post_window_s: float,
  require_stopped: bool = False,
) -> float | None:
  anchor_time_s = samples[anchor_idx].t
  values: list[float] = []
  for idx in range(start_idx, min(len(samples), end_idx + 1)):
    sample = samples[idx]
    if sample.t < (anchor_time_s - pre_window_s) or sample.t > (anchor_time_s + post_window_s):
      continue
    if require_stopped and not is_stopped(sample, standstill_speed=0.05):
      continue
    if sample.lead_status and sample.lead_d_rel_m is not None:
      values.append(sample.lead_d_rel_m)
  return float(median(values)) if values else None


def max_abs_jerk_in_window(samples: list[Sample], indices: list[int]) -> float | None:
  if len(indices) < 2:
    return None
  max_abs_jerk: float | None = None
  for prev_idx, cur_idx in zip(indices, indices[1:]):
    dt = samples[cur_idx].t - samples[prev_idx].t
    if dt <= 1e-6:
      continue
    jerk = abs((samples[cur_idx].a_ego - samples[prev_idx].a_ego) / dt)
    max_abs_jerk = jerk if max_abs_jerk is None else max(max_abs_jerk, jerk)
  return max_abs_jerk


def count_threshold_crossings(values: list[float], threshold: float) -> int:
  if len(values) < 2:
    return 0
  crossings = 0
  prev_above = values[0] >= threshold
  for value in values[1:]:
    current_above = value >= threshold
    if current_above != prev_above:
      crossings += 1
      prev_above = current_above
  return crossings


def wheel_stop_metrics(samples: list[Sample], indices: list[int]) -> tuple[float | None, float | None]:
  valid_indices = [idx for idx in indices if samples[idx].v_wheel_avg is not None]
  if len(valid_indices) < 2:
    return None, None

  min_wheel_decel: float | None = None
  for prev_idx, cur_idx in zip(valid_indices, valid_indices[1:]):
    dt = samples[cur_idx].t - samples[prev_idx].t
    if dt <= 1e-6:
      continue
    prev_speed = samples[prev_idx].v_wheel_avg
    cur_speed = samples[cur_idx].v_wheel_avg
    if prev_speed is None or cur_speed is None:
      continue
    wheel_decel = (cur_speed - prev_speed) / dt
    min_wheel_decel = wheel_decel if min_wheel_decel is None else min(min_wheel_decel, wheel_decel)

  max_drop_150ms = 0.0
  for base_idx in valid_indices:
    base_time = samples[base_idx].t
    base_speed = samples[base_idx].v_wheel_avg
    if base_speed is None:
      continue
    min_speed = base_speed
    for scan_idx in valid_indices:
      if scan_idx <= base_idx:
        continue
      dt = samples[scan_idx].t - base_time
      if dt > 0.15:
        break
      scan_speed = samples[scan_idx].v_wheel_avg
      if scan_speed is None:
        continue
      min_speed = min(min_speed, scan_speed)
    max_drop_150ms = max(max_drop_150ms, base_speed - min_speed)

  return min_wheel_decel, max_drop_150ms


def compute_transition_sharpness_metrics(
  samples: list[Sample],
  start_idx: int,
  end_idx: int,
  anchor_time_s: float | None,
) -> tuple[float | None, float | None, float | None, float | None]:
  if anchor_time_s is None:
    return None, None, None, None

  jerk_indices = [
    idx
    for idx in range(start_idx, min(len(samples), end_idx + 1))
    if (anchor_time_s - 0.10) <= samples[idx].t <= (anchor_time_s + 0.35)
  ]
  accel_jerk = max_abs_jerk_in_window(samples, jerk_indices)

  cmd_indices = [idx for idx in jerk_indices if samples[idx].accel_cmd is not None]
  cmd_jerk: float | None = None
  if len(cmd_indices) >= 2:
    for prev_idx, cur_idx in zip(cmd_indices, cmd_indices[1:]):
      dt = samples[cur_idx].t - samples[prev_idx].t
      if dt <= 1e-6:
        continue
      prev_cmd = samples[prev_idx].accel_cmd
      cur_cmd = samples[cur_idx].accel_cmd
      if prev_cmd is None or cur_cmd is None:
        continue
      slope = abs((cur_cmd - prev_cmd) / dt)
      cmd_jerk = slope if cmd_jerk is None else max(cmd_jerk, slope)

  pre_accel_values = [
    samples[idx].a_ego
    for idx in range(start_idx, min(len(samples), end_idx + 1))
    if (anchor_time_s - 0.30) <= samples[idx].t <= (anchor_time_s - 0.05)
  ]
  post_accel_values = [
    samples[idx].a_ego
    for idx in range(start_idx, min(len(samples), end_idx + 12))
    if anchor_time_s <= samples[idx].t <= (anchor_time_s + 0.15)
  ]
  accel_step: float | None = None
  if pre_accel_values and post_accel_values:
    accel_step = abs(float(np.mean(post_accel_values)) - float(np.mean(pre_accel_values)))

  pre_cmd_values = [
    samples[idx].accel_cmd
    for idx in range(start_idx, min(len(samples), end_idx + 1))
    if (anchor_time_s - 0.30) <= samples[idx].t <= (anchor_time_s - 0.05) and samples[idx].accel_cmd is not None
  ]
  post_cmd_values = [
    samples[idx].accel_cmd
    for idx in range(start_idx, min(len(samples), end_idx + 12))
    if anchor_time_s <= samples[idx].t <= (anchor_time_s + 0.15) and samples[idx].accel_cmd is not None
  ]
  cmd_step: float | None = None
  if pre_cmd_values and post_cmd_values:
    cmd_step = abs(float(np.mean(post_cmd_values)) - float(np.mean(pre_cmd_values)))

  return accel_jerk, accel_step, cmd_jerk, cmd_step


def compute_event(
  event_id: int,
  event_source: str,
  samples: list[Sample],
  start_idx: int,
  stop_idx: int,
  hold_idx: int,
  approach_speed: float,
  graph_file: str,
) -> StopEvent:
  window = samples[start_idx:hold_idx + 1]
  hold_time_s = samples[hold_idx].t

  accel_cmd_values = [item.accel_cmd for item in window if item.accel_cmd is not None]
  a_target_values = [item.a_target for item in window if item.a_target is not None]

  t_should_stop = first_time(window, 0, len(window) - 1, lambda item: item.should_stop)
  t_stopping_state = first_time(
    window,
    0,
    len(window) - 1,
    lambda item: item.long_state == "stopping" or item.long_state_cmd == "stopping",
  )
  should_to_stopping = None if t_should_stop is None or t_stopping_state is None else (t_stopping_state - t_should_stop)
  t_stopping_idx = first_index(
    window,
    0,
    len(window) - 1,
    lambda item: item.long_state == "stopping" or item.long_state_cmd == "stopping",
  )
  t_should_stop_idx = first_index(window, 0, len(window) - 1, lambda item: item.should_stop)
  stop_entry_offset_idx = t_stopping_idx if t_stopping_idx is not None else t_should_stop_idx
  stop_entry_index = start_idx + stop_entry_offset_idx if stop_entry_offset_idx is not None else start_idx
  lead_distance_stop_entry = lead_distance_metric(
    samples=samples,
    start_idx=start_idx,
    end_idx=hold_idx,
    anchor_idx=stop_entry_index,
    pre_window_s=0.0,
    post_window_s=0.15,
  )
  lead_distance_hold = lead_distance_metric(
    samples=samples,
    start_idx=max(start_idx, hold_idx - 20),
    end_idx=min(len(samples) - 1, hold_idx + 20),
    anchor_idx=hold_idx,
    pre_window_s=0.15,
    post_window_s=0.15,
    require_stopped=True,
  )
  stop_entry_anchor_s = t_stopping_state if t_stopping_state is not None else t_should_stop
  entry_stop_jerk, entry_stop_accel_step, entry_stop_cmd_jerk, entry_stop_cmd_step = compute_transition_sharpness_metrics(
    samples=samples,
    start_idx=start_idx,
    end_idx=hold_idx,
    anchor_time_s=stop_entry_anchor_s,
  )

  creep_window: list[Sample] = []
  for item in samples[hold_idx:min(hold_idx + 80, len(samples))]:
    if item.t - samples[hold_idx].t > 4.0:
      break
    if not (item.standstill or item.should_stop):
      break
    creep_window.append(item)
  creep_after_stop = max((abs(item.v_ego) for item in creep_window), default=0.0)

  jerk_indices = [
    idx
    for idx in range(start_idx, min(len(samples), hold_idx + 25))
    if (hold_time_s - 0.45) <= samples[idx].t <= (hold_time_s + 0.20)
  ]
  end_stop_jerk = max_abs_jerk_in_window(samples, jerk_indices)
  cmd_indices = [idx for idx in jerk_indices if samples[idx].accel_cmd is not None]
  end_stop_cmd_jerk: float | None = None
  if len(cmd_indices) >= 2:
    for prev_idx, cur_idx in zip(cmd_indices, cmd_indices[1:]):
      dt = samples[cur_idx].t - samples[prev_idx].t
      if dt <= 1e-6:
        continue
      prev_cmd = samples[prev_idx].accel_cmd
      cur_cmd = samples[cur_idx].accel_cmd
      if prev_cmd is None or cur_cmd is None:
        continue
      slope = abs((cur_cmd - prev_cmd) / dt)
      end_stop_cmd_jerk = slope if end_stop_cmd_jerk is None else max(end_stop_cmd_jerk, slope)
  wheel_stop_decel, wheel_speed_drop_150ms = wheel_stop_metrics(samples, jerk_indices)

  pre_accel_values = [
    samples[idx].a_ego
    for idx in range(start_idx, hold_idx + 1)
    if (hold_time_s - 0.50) <= samples[idx].t <= (hold_time_s - 0.10)
  ]
  hold_accel_values = [
    samples[idx].a_ego
    for idx in range(hold_idx, min(len(samples), hold_idx + 10))
    if hold_time_s <= samples[idx].t <= (hold_time_s + 0.15)
  ]
  end_stop_accel_step = None
  if pre_accel_values and hold_accel_values:
    end_stop_accel_step = abs(float(np.mean(hold_accel_values)) - float(np.mean(pre_accel_values)))

  pre_cmd_values = [
    samples[idx].accel_cmd
    for idx in range(start_idx, hold_idx + 1)
    if (hold_time_s - 0.50) <= samples[idx].t <= (hold_time_s - 0.10) and samples[idx].accel_cmd is not None
  ]
  hold_cmd_values = [
    samples[idx].accel_cmd
    for idx in range(hold_idx, min(len(samples), hold_idx + 10))
    if hold_time_s <= samples[idx].t <= (hold_time_s + 0.15) and samples[idx].accel_cmd is not None
  ]
  end_stop_cmd_step = None
  if pre_cmd_values and hold_cmd_values:
    end_stop_cmd_step = abs(float(np.mean(hold_cmd_values)) - float(np.mean(pre_cmd_values)))

  first_near_stop_idx: int | None = None
  for idx in range(start_idx, hold_idx + 1):
    if samples[idx].v_ego <= 0.25:
      first_near_stop_idx = idx
      break

  oscillation_count = 0
  reaccel_before_hold = False
  if first_near_stop_idx is not None:
    low_speed_values = [samples[idx].v_ego for idx in range(first_near_stop_idx, hold_idx + 1)]
    oscillation_count = count_threshold_crossings(low_speed_values, threshold=0.30)
    min_low_speed = min(low_speed_values) if low_speed_values else 0.0
    max_low_speed = max(low_speed_values) if low_speed_values else 0.0
    reaccel_before_hold = oscillation_count >= 2 and (max_low_speed - min_low_speed) >= 0.20 and max_low_speed >= 0.35

  late_hold_indices = [
    idx
    for idx in range(start_idx, hold_idx + 1)
    if (hold_time_s - 0.80) <= samples[idx].t <= hold_time_s
  ]
  stop_signal_dropped_before_hold = any(not samples[idx].stop_signal for idx in late_hold_indices)
  left_stopping_state_before_hold = any(
    not (samples[idx].long_state == "stopping" or samples[idx].long_state_cmd == "stopping")
    for idx in late_hold_indices
  )

  near_hold_cmd_values = [
    samples[idx].accel_cmd
    for idx in range(start_idx, min(len(samples), hold_idx + 12))
    if (hold_time_s - 0.35) <= samples[idx].t <= (hold_time_s + 0.20) and samples[idx].accel_cmd is not None
  ]
  near_hold_indices = [
    idx
    for idx in range(start_idx, min(len(samples), hold_idx + 12))
    if (hold_time_s - 0.35) <= samples[idx].t <= (hold_time_s + 0.20)
  ]
  max_accel_cmd_near_hold = max(near_hold_cmd_values) if near_hold_cmd_values else None
  positive_accel_cmd_near_hold = max_accel_cmd_near_hold is not None and max_accel_cmd_near_hold > 0.02
  positive_accel_cmd_with_stop_signal_near_hold = any(
    samples[idx].stop_signal and samples[idx].accel_cmd is not None and samples[idx].accel_cmd > 0.02
    for idx in near_hold_indices
  )

  baseline_speed_values = [
    samples[idx].v_ego
    for idx in range(start_idx, min(len(samples), hold_idx + 8))
    if (hold_time_s - 0.20) <= samples[idx].t <= (hold_time_s + 0.10)
  ]
  post_hold_speed_values = [
    samples[idx].v_ego
    for idx in range(hold_idx, min(len(samples), hold_idx + 35))
    if hold_time_s <= samples[idx].t <= (hold_time_s + 1.00)
  ]
  speed_rebound_after_hold = 0.0
  if baseline_speed_values and post_hold_speed_values:
    speed_rebound_after_hold = max(0.0, max(post_hold_speed_values) - min(baseline_speed_values))

  post_hold_signal_speed_values = [
    samples[idx].v_ego
    for idx in range(hold_idx, min(len(samples), hold_idx + 35))
    if hold_time_s <= samples[idx].t <= (hold_time_s + 1.00) and samples[idx].stop_signal
  ]
  speed_rebound_while_stop_signal = 0.0
  if baseline_speed_values and post_hold_signal_speed_values:
    speed_rebound_while_stop_signal = max(0.0, max(post_hold_signal_speed_values) - min(baseline_speed_values))

  should_stop_indices = [
    idx
    for idx in range(max(start_idx, hold_idx - 12), min(len(samples), hold_idx + 40))
    if (hold_time_s - 0.35) <= samples[idx].t <= (hold_time_s + 1.00)
    and samples[idx].should_stop
    and samples[idx].v_ego <= 1.2
  ]
  should_stop_baseline_speeds = [samples[idx].v_ego for idx in should_stop_indices if samples[idx].t <= (hold_time_s + 0.10)]
  should_stop_post_speeds = [samples[idx].v_ego for idx in should_stop_indices if samples[idx].t >= hold_time_s]
  speed_rebound_while_should_stop = 0.0
  if should_stop_baseline_speeds and should_stop_post_speeds:
    speed_rebound_while_should_stop = max(0.0, max(should_stop_post_speeds) - min(should_stop_baseline_speeds))

  should_stop_unexpected_accel: float | None = None
  braking_should_stop_indices = [
    idx for idx in should_stop_indices if samples[idx].accel_cmd is not None and samples[idx].accel_cmd <= -0.1
  ]
  if braking_should_stop_indices:
    should_stop_unexpected_accel = max(samples[idx].a_ego for idx in braking_should_stop_indices)

  should_stop_decel_relief_spike: float | None = None
  if len(should_stop_indices) >= 2:
    for prev_idx, cur_idx in zip(should_stop_indices, should_stop_indices[1:]):
      prev = samples[prev_idx]
      cur = samples[cur_idx]
      if prev.accel_cmd is None or cur.accel_cmd is None:
        continue
      if prev.accel_cmd > -0.1 or cur.accel_cmd > -0.1:
        continue
      if abs(cur.accel_cmd - prev.accel_cmd) > 0.02:
        continue
      relief = cur.a_ego - prev.a_ego
      should_stop_decel_relief_spike = relief if should_stop_decel_relief_spike is None else max(should_stop_decel_relief_spike, relief)

  low_speed_stop_indices = [
    idx
    for idx in range(start_idx, hold_idx + 1)
    if samples[idx].v_ego <= 1.2 and samples[idx].stop_signal and samples[idx].accel_cmd is not None
  ]
  low_speed_cmd_values = [samples[idx].accel_cmd for idx in low_speed_stop_indices if samples[idx].accel_cmd is not None]
  low_speed_cmd_std = float(np.std(low_speed_cmd_values)) if len(low_speed_cmd_values) >= 2 else (0.0 if low_speed_cmd_values else None)

  unexpected_accel_while_braking: float | None = None
  braking_indices = [idx for idx in low_speed_stop_indices if samples[idx].accel_cmd is not None and samples[idx].accel_cmd <= -0.1]
  if braking_indices:
    unexpected_accel_while_braking = max(samples[idx].a_ego for idx in braking_indices)

  stable_cmd_accel_delta: float | None = None
  if len(low_speed_stop_indices) >= 2:
    for prev_idx, cur_idx in zip(low_speed_stop_indices, low_speed_stop_indices[1:]):
      prev_cmd = samples[prev_idx].accel_cmd
      cur_cmd = samples[cur_idx].accel_cmd
      if prev_cmd is None or cur_cmd is None:
        continue
      if abs(cur_cmd - prev_cmd) > 0.02:
        continue
      accel_delta = abs(samples[cur_idx].a_ego - samples[prev_idx].a_ego)
      stable_cmd_accel_delta = accel_delta if stable_cmd_accel_delta is None else max(stable_cmd_accel_delta, accel_delta)

  brake_ratio = float(np.mean([1.0 if item.brake_pressed else 0.0 for item in window])) if window else 0.0
  enabled_ratio = float(np.mean([1.0 if item.enabled else 0.0 for item in window])) if window else 0.0
  stop_signal_ratio = float(np.mean([1.0 if item.stop_signal else 0.0 for item in window])) if window else 0.0
  should_stop_ratio = float(np.mean([1.0 if item.should_stop else 0.0 for item in window])) if window else 0.0
  stopping_state_ratio = float(np.mean([1.0 if item.long_state == "stopping" else 0.0 for item in window])) if window else 0.0
  stopping_state_cmd_ratio = float(np.mean([1.0 if item.long_state_cmd == "stopping" else 0.0 for item in window])) if window else 0.0

  return StopEvent(
    event_id=event_id,
    event_source=event_source,
    start_index=start_idx,
    stop_index=stop_idx,
    stop_hold_index=hold_idx,
    start_segment=samples[start_idx].segment,
    stop_segment=samples[hold_idx].segment,
    start_time_s=samples[start_idx].t,
    stop_time_s=samples[stop_idx].t,
    stop_hold_time_s=samples[hold_idx].t,
    duration_s=samples[hold_idx].t - samples[start_idx].t,
    approach_speed_mps=approach_speed,
    entry_speed_mps=samples[start_idx].v_ego,
    min_speed_mps=min(item.v_ego for item in window),
    min_a_ego_mps2=min(item.a_ego for item in window),
    min_accel_cmd_mps2=min(accel_cmd_values) if accel_cmd_values else None,
    min_a_target_mps2=min(a_target_values) if a_target_values else None,
    distance_traveled_m=integrate_distance(samples, start_idx, hold_idx),
    rollout_distance_from_2mps_m=rollout_distance_from_speed(samples, start_idx, hold_idx, speed_threshold=2.0),
    should_stop_to_stopping_s=should_to_stopping,
    lead_distance_stop_entry_m=lead_distance_stop_entry,
    lead_distance_hold_m=lead_distance_hold,
    entry_stop_jerk_mps3=entry_stop_jerk,
    entry_stop_accel_step_mps2=entry_stop_accel_step,
    entry_stop_cmd_jerk_mps3=entry_stop_cmd_jerk,
    entry_stop_cmd_step_mps2=entry_stop_cmd_step,
    end_stop_jerk_mps3=end_stop_jerk,
    end_stop_accel_step_mps2=end_stop_accel_step,
    end_stop_cmd_jerk_mps3=end_stop_cmd_jerk,
    end_stop_cmd_step_mps2=end_stop_cmd_step,
    wheel_stop_decel_mps2=wheel_stop_decel,
    wheel_speed_drop_150ms_mps=wheel_speed_drop_150ms,
    reaccel_before_hold=reaccel_before_hold,
    low_speed_oscillation_count=oscillation_count,
    stop_signal_dropped_before_hold=stop_signal_dropped_before_hold,
    left_stopping_state_before_hold=left_stopping_state_before_hold,
    positive_accel_cmd_near_hold=positive_accel_cmd_near_hold,
    positive_accel_cmd_with_stop_signal_near_hold=positive_accel_cmd_with_stop_signal_near_hold,
    max_accel_cmd_near_hold_mps2=max_accel_cmd_near_hold,
    speed_rebound_after_hold_mps=speed_rebound_after_hold,
    speed_rebound_while_stop_signal_mps=speed_rebound_while_stop_signal,
    speed_rebound_while_should_stop_mps=speed_rebound_while_should_stop,
    unexpected_accel_while_braking_mps2=unexpected_accel_while_braking,
    stable_cmd_accel_delta_mps2=stable_cmd_accel_delta,
    should_stop_unexpected_accel_mps2=should_stop_unexpected_accel,
    should_stop_decel_relief_spike_mps2=should_stop_decel_relief_spike,
    low_speed_cmd_std_mps2=low_speed_cmd_std,
    creep_after_stop_mps=creep_after_stop,
    forcing_stop_seen=any(item.forcing_stop for item in window),
    red_light_seen=any(item.red_light for item in window),
    brake_pressed_ratio=brake_ratio,
    enabled_ratio=enabled_ratio,
    stop_signal_ratio=stop_signal_ratio,
    should_stop_ratio=should_stop_ratio,
    stopping_state_ratio=stopping_state_ratio,
    stopping_state_cmd_ratio=stopping_state_cmd_ratio,
    graph_file=graph_file,
  )


def make_event_plot(
  event: StopEvent,
  samples: list[Sample],
  output_path: Path,
  pre_window: float,
  post_window: float,
) -> None:
  start_t = event.start_time_s - pre_window
  end_t = event.stop_hold_time_s + post_window
  points = [item for item in samples if start_t <= item.t <= end_t]
  if not points:
    return

  x = [item.t - event.start_time_s for item in points]
  v_ego = [item.v_ego for item in points]
  a_ego = [item.a_ego for item in points]
  a_target = [item.a_target if item.a_target is not None else np.nan for item in points]
  accel_cmd = [item.accel_cmd if item.accel_cmd is not None else np.nan for item in points]
  should_stop = [1.0 if item.should_stop else 0.0 for item in points]
  standstill = [1.0 if item.standstill else 0.0 for item in points]
  forcing_stop = [1.0 if item.forcing_stop else 0.0 for item in points]
  red_light = [1.0 if item.red_light else 0.0 for item in points]
  long_state = [LONG_STATE_MAP.get(item.long_state, np.nan) for item in points]

  fig = make_subplots(
    rows=3,
    cols=1,
    shared_xaxes=True,
    vertical_spacing=0.06,
    subplot_titles=(
      f"Event {event.event_id}: Speed",
      "Acceleration",
      "Stop Signals / States",
    ),
  )

  fig.add_trace(go.Scatter(x=x, y=v_ego, name="vEgo (m/s)", mode="lines"), row=1, col=1)
  fig.add_trace(go.Scatter(x=x, y=[0.5] * len(x), name="vEgoStopping ref", mode="lines"), row=1, col=1)

  fig.add_trace(go.Scatter(x=x, y=a_ego, name="aEgo", mode="lines"), row=2, col=1)
  fig.add_trace(go.Scatter(x=x, y=a_target, name="aTarget", mode="lines"), row=2, col=1)
  fig.add_trace(go.Scatter(x=x, y=accel_cmd, name="actuators.accel", mode="lines"), row=2, col=1)

  fig.add_trace(go.Scatter(x=x, y=long_state, name="longControlState", mode="lines"), row=3, col=1)
  fig.add_trace(go.Scatter(x=x, y=should_stop, name="shouldStop", mode="lines"), row=3, col=1)
  fig.add_trace(go.Scatter(x=x, y=standstill, name="standstill", mode="lines"), row=3, col=1)
  fig.add_trace(go.Scatter(x=x, y=forcing_stop, name="forcingStop", mode="lines"), row=3, col=1)
  fig.add_trace(go.Scatter(x=x, y=red_light, name="redLight", mode="lines"), row=3, col=1)

  fig.add_vline(x=0.0, line_width=1, line_dash="dash", line_color="gray")
  fig.add_vline(x=event.stop_hold_time_s - event.start_time_s, line_width=1, line_dash="dash", line_color="green")

  fig.update_layout(
    title=f"Stopping behavior event {event.event_id} (segment {event.start_segment})",
    height=960,
    legend=dict(orientation="h"),
  )
  fig.update_yaxes(title_text="m/s", row=1, col=1)
  fig.update_yaxes(title_text="m/s²", row=2, col=1)
  fig.update_yaxes(
    title_text="state/signal",
    tickmode="array",
    tickvals=[0, 1, 2, 3],
    ticktext=[LONG_STATE_LABELS.get(idx, str(idx)) for idx in [0, 1, 2, 3]],
    row=3,
    col=1,
  )
  fig.update_xaxes(title_text="Time from event start (s)", row=3, col=1)
  fig.write_html(str(output_path), include_plotlyjs="cdn")


def format_metric(value: float | None, digits: int = 3) -> str:
  if value is None:
    return "n/a"
  return f"{value:.{digits}f}"


def load_latest_settings(settings_dir: Path, host: str) -> Path | None:
  settings_files = sorted(settings_dir.expanduser().glob(f"stop_settings_{host}_*.json"))
  return settings_files[-1] if settings_files else None


def build_summary_markdown(
  route: str,
  qlog_count: int,
  event_mode: str,
  events: list[StopEvent],
  output_dir: Path,
  settings_file: Path | None,
) -> str:
  lines: list[str] = []
  lines.append(f"# Stopping Analysis: `{route}`")
  lines.append("")
  lines.append(f"- Generated (UTC): {utc_now_iso()}")
  lines.append(f"- Segments analyzed: {qlog_count}")
  lines.append(f"- Event mode: `{event_mode}`")
  lines.append(f"- Stop events detected: {len(events)}")
  if settings_file:
    lines.append(f"- Settings snapshot: `{settings_file}`")
  lines.append("")

  if events:
    durations = [item.duration_s for item in events]
    approaches = [item.approach_speed_mps for item in events]
    entries = [item.entry_speed_mps for item in events]
    min_decel = [item.min_a_ego_mps2 for item in events]
    rollouts = [item.rollout_distance_from_2mps_m for item in events]
    entry_jerk_values = [item.entry_stop_jerk_mps3 for item in events if item.entry_stop_jerk_mps3 is not None]
    entry_step_values = [item.entry_stop_accel_step_mps2 for item in events if item.entry_stop_accel_step_mps2 is not None]
    entry_cmd_jerk_values = [item.entry_stop_cmd_jerk_mps3 for item in events if item.entry_stop_cmd_jerk_mps3 is not None]
    entry_cmd_step_values = [item.entry_stop_cmd_step_mps2 for item in events if item.entry_stop_cmd_step_mps2 is not None]
    end_jerk_values = [item.end_stop_jerk_mps3 for item in events if item.end_stop_jerk_mps3 is not None]
    end_step_values = [item.end_stop_accel_step_mps2 for item in events if item.end_stop_accel_step_mps2 is not None]
    cmd_jerk_values = [item.end_stop_cmd_jerk_mps3 for item in events if item.end_stop_cmd_jerk_mps3 is not None]
    cmd_step_values = [item.end_stop_cmd_step_mps2 for item in events if item.end_stop_cmd_step_mps2 is not None]
    wheel_decel_values = [item.wheel_stop_decel_mps2 for item in events if item.wheel_stop_decel_mps2 is not None]
    wheel_drop_values = [item.wheel_speed_drop_150ms_mps for item in events if item.wheel_speed_drop_150ms_mps is not None]
    reaccel_count = sum(1 for item in events if item.reaccel_before_hold)
    stop_signal_drop_count = sum(1 for item in events if item.stop_signal_dropped_before_hold)
    stopping_exit_count = sum(1 for item in events if item.left_stopping_state_before_hold)
    positive_cmd_count = sum(1 for item in events if item.positive_accel_cmd_near_hold)
    positive_cmd_with_signal_count = sum(1 for item in events if item.positive_accel_cmd_with_stop_signal_near_hold)
    cmd_near_hold_values = [item.max_accel_cmd_near_hold_mps2 for item in events if item.max_accel_cmd_near_hold_mps2 is not None]
    rebound_values = [item.speed_rebound_after_hold_mps for item in events]
    rebound_with_signal_values = [item.speed_rebound_while_stop_signal_mps for item in events]
    rebound_with_should_stop_values = [item.speed_rebound_while_should_stop_mps for item in events]
    unexpected_accel_values = [
      item.unexpected_accel_while_braking_mps2 for item in events if item.unexpected_accel_while_braking_mps2 is not None
    ]
    should_stop_unexpected_accel_values = [
      item.should_stop_unexpected_accel_mps2 for item in events if item.should_stop_unexpected_accel_mps2 is not None
    ]
    stable_cmd_delta_values = [item.stable_cmd_accel_delta_mps2 for item in events if item.stable_cmd_accel_delta_mps2 is not None]
    should_stop_relief_values = [
      item.should_stop_decel_relief_spike_mps2 for item in events if item.should_stop_decel_relief_spike_mps2 is not None
    ]
    low_speed_cmd_std_values = [item.low_speed_cmd_std_mps2 for item in events if item.low_speed_cmd_std_mps2 is not None]
    delays = [item.should_stop_to_stopping_s for item in events if item.should_stop_to_stopping_s is not None]
    lead_entry_values = [item.lead_distance_stop_entry_m for item in events if item.lead_distance_stop_entry_m is not None]
    lead_hold_values = [item.lead_distance_hold_m for item in events if item.lead_distance_hold_m is not None]
    lines.append("## Aggregate Metrics")
    lines.append("")
    lines.append(f"- Median duration to standstill hold: {format_metric(median(durations), 2)} s")
    lines.append(f"- Median approach speed: {format_metric(median(approaches), 2)} m/s")
    lines.append(f"- Median entry speed: {format_metric(median(entries), 2)} m/s")
    lines.append(f"- Median min aEgo: {format_metric(median(min_decel), 2)} m/s²")
    lines.append(f"- Median rollout distance from 2 m/s to hold: {format_metric(median(rollouts), 2)} m")
    if lead_entry_values:
      lines.append(f"- Median lead distance when stopping starts: {format_metric(median(lead_entry_values), 2)} m")
    else:
      lines.append("- Median lead distance when stopping starts: n/a")
    if lead_hold_values:
      lines.append(f"- Median lead distance at final hold: {format_metric(median(lead_hold_values), 2)} m")
    else:
      lines.append("- Median lead distance at final hold: n/a")
    if entry_jerk_values:
      lines.append(f"- Median stop-entry jerk around first `stopping` transition: {format_metric(median(entry_jerk_values), 2)} m/s³")
    else:
      lines.append("- Median stop-entry jerk around first `stopping` transition: n/a")
    if entry_step_values:
      lines.append(f"- Median stop-entry accel step: {format_metric(median(entry_step_values), 2)} m/s²")
    else:
      lines.append("- Median stop-entry accel step: n/a")
    if entry_cmd_jerk_values:
      lines.append(f"- Median stop-entry command jerk: {format_metric(median(entry_cmd_jerk_values), 2)} m/s³")
    else:
      lines.append("- Median stop-entry command jerk: n/a")
    if entry_cmd_step_values:
      lines.append(f"- Median stop-entry command step: {format_metric(median(entry_cmd_step_values), 2)} m/s²")
    else:
      lines.append("- Median stop-entry command step: n/a")
    if end_jerk_values:
      lines.append(f"- Median end-stop jerk (|da/dt|): {format_metric(median(end_jerk_values), 2)} m/s³")
    else:
      lines.append("- Median end-stop jerk (|da/dt|): n/a")
    if end_step_values:
      lines.append(f"- Median end-stop accel step: {format_metric(median(end_step_values), 2)} m/s²")
    else:
      lines.append("- Median end-stop accel step: n/a")
    if cmd_jerk_values:
      lines.append(f"- Median end-stop command jerk (|dCmd/dt|): {format_metric(median(cmd_jerk_values), 2)} m/s³")
    else:
      lines.append("- Median end-stop command jerk (|dCmd/dt|): n/a")
    if cmd_step_values:
      lines.append(f"- Median end-stop command step: {format_metric(median(cmd_step_values), 2)} m/s²")
    else:
      lines.append("- Median end-stop command step: n/a")
    if wheel_decel_values:
      lines.append(f"- Median wheel-stop decel (dvWheel/dt): {format_metric(median(wheel_decel_values), 2)} m/s²")
    else:
      lines.append("- Median wheel-stop decel (dvWheel/dt): n/a")
    if wheel_drop_values:
      lines.append(f"- Median wheel-speed drop (150ms): {format_metric(median(wheel_drop_values), 3)} m/s")
    else:
      lines.append("- Median wheel-speed drop (150ms): n/a")
    if cmd_near_hold_values:
      lines.append(f"- Median max accel cmd near hold: {format_metric(median(cmd_near_hold_values), 3)} m/s²")
    else:
      lines.append("- Median max accel cmd near hold: n/a")
    lines.append(f"- Median speed rebound after hold: {format_metric(median(rebound_values), 3)} m/s")
    lines.append(f"- Median speed rebound while stop signal remains true: {format_metric(median(rebound_with_signal_values), 3)} m/s")
    lines.append(f"- Median speed rebound while shouldStop remains true: {format_metric(median(rebound_with_should_stop_values), 3)} m/s")
    if unexpected_accel_values:
      lines.append(f"- Median unexpected aEgo while braking cmd<=-0.1: {format_metric(median(unexpected_accel_values), 3)} m/s²")
    else:
      lines.append("- Unexpected aEgo while braking cmd<=-0.1: n/a")
    if should_stop_unexpected_accel_values:
      lines.append(f"- Median unexpected aEgo while shouldStop + braking cmd<=-0.1: {format_metric(median(should_stop_unexpected_accel_values), 3)} m/s²")
    else:
      lines.append("- Unexpected aEgo while shouldStop + braking cmd<=-0.1: n/a")
    if stable_cmd_delta_values:
      lines.append(f"- Median |delta aEgo| under stable cmd (|delta cmd|<=0.02): {format_metric(median(stable_cmd_delta_values), 3)} m/s²")
    else:
      lines.append("- |delta aEgo| under stable cmd (|delta cmd|<=0.02): n/a")
    if should_stop_relief_values:
      lines.append(f"- Median decel-relief spike under shouldStop (stable cmd): {format_metric(median(should_stop_relief_values), 3)} m/s²")
    else:
      lines.append("- Decel-relief spike under shouldStop (stable cmd): n/a")
    if low_speed_cmd_std_values:
      lines.append(f"- Median low-speed cmd std (v<=1.2, stop signal): {format_metric(median(low_speed_cmd_std_values), 3)} m/s²")
    else:
      lines.append("- Low-speed cmd std (v<=1.2, stop signal): n/a")
    if delays:
      lines.append(f"- Median shouldStop->stopping delay: {format_metric(median(delays), 3)} s")
    else:
      lines.append("- shouldStop->stopping delay: n/a")
    lines.append(f"- Re-accel before hold events: {reaccel_count}/{len(events)}")
    lines.append(f"- stopSignal dropped before hold (last 0.8s): {stop_signal_drop_count}/{len(events)}")
    lines.append(f"- Exited stopping state before hold (last 0.8s): {stopping_exit_count}/{len(events)}")
    lines.append(f"- Positive accel command near hold: {positive_cmd_count}/{len(events)}")
    lines.append(f"- Positive accel command near hold while stop signal is true: {positive_cmd_with_signal_count}/{len(events)}")
    lines.append("")

  lines.append("## Event Table")
  lines.append("")
  lines.append(
    "|Event|Source|Seg|Approach|Entry|LeadStart|LeadHold|EntryJerk|EntryStep|EntryCmdJerk|EntryCmdStep|Duration|Rollout2m|EndJerk|EndStep|CmdJerk|CmdStep|"
    "WheelDecel|WheelDrop150ms|ReAccel|SigDrop|ExitStop|PosCmd|PosCmdSig|MaxCmdNear|Rebound|ReboundSig|"
    "Min aEgo|Min cmd|should->stopping|ForceStop|RedLight|Graph|"
  )
  lines.append(
    "|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|"
  )

  for item in events:
    lines.append(
      "|"
      f"{item.event_id}|"
      f"{item.event_source}|"
      f"{item.start_segment}|"
      f"{format_metric(item.approach_speed_mps, 2)}|"
      f"{format_metric(item.entry_speed_mps, 2)}|"
      f"{format_metric(item.lead_distance_stop_entry_m, 2)}|"
      f"{format_metric(item.lead_distance_hold_m, 2)}|"
      f"{format_metric(item.entry_stop_jerk_mps3, 2)}|"
      f"{format_metric(item.entry_stop_accel_step_mps2, 2)}|"
      f"{format_metric(item.entry_stop_cmd_jerk_mps3, 2)}|"
      f"{format_metric(item.entry_stop_cmd_step_mps2, 2)}|"
      f"{format_metric(item.duration_s, 2)}|"
      f"{format_metric(item.rollout_distance_from_2mps_m, 2)}|"
      f"{format_metric(item.end_stop_jerk_mps3, 2)}|"
      f"{format_metric(item.end_stop_accel_step_mps2, 2)}|"
      f"{format_metric(item.end_stop_cmd_jerk_mps3, 2)}|"
      f"{format_metric(item.end_stop_cmd_step_mps2, 2)}|"
      f"{format_metric(item.wheel_stop_decel_mps2, 2)}|"
      f"{format_metric(item.wheel_speed_drop_150ms_mps, 3)}|"
      f"{'yes' if item.reaccel_before_hold else 'no'}|"
      f"{'yes' if item.stop_signal_dropped_before_hold else 'no'}|"
      f"{'yes' if item.left_stopping_state_before_hold else 'no'}|"
      f"{'yes' if item.positive_accel_cmd_near_hold else 'no'}|"
      f"{'yes' if item.positive_accel_cmd_with_stop_signal_near_hold else 'no'}|"
      f"{format_metric(item.max_accel_cmd_near_hold_mps2, 2)}|"
      f"{format_metric(item.speed_rebound_after_hold_mps, 3)}|"
      f"{format_metric(item.speed_rebound_while_stop_signal_mps, 3)}|"
      f"{format_metric(item.min_a_ego_mps2, 2)}|"
      f"{format_metric(item.min_accel_cmd_mps2, 2)}|"
      f"{format_metric(item.should_stop_to_stopping_s, 3)}|"
      f"{'yes' if item.forcing_stop_seen else 'no'}|"
      f"{'yes' if item.red_light_seen else 'no'}|"
      f"[plot]({item.graph_file})|"
    )

  lines.append("")
  lines.append("## Notes")
  lines.append("")
  lines.append("- `Duration` is from detected event start to standstill-hold confirmation.")
  lines.append("- `Rollout2m` is the no-lead rollout budget: distance traveled from first vEgo <= 2.0 m/s until standstill hold.")
  lines.append("- `LeadStart` is the lead distance near the first `stopping` transition (fallback: first `shouldStop`) when a lead exists.")
  lines.append("- `LeadHold` is the lead distance in the final standstill-hold window when a lead still exists; use this instead of `Rollout2m` for lead-follow stops.")
  lines.append("- `EntryJerk` / `EntryStep` measure the accel-side bite around the first `stopping` transition (fallback: first `shouldStop`).")
  lines.append("- `EntryCmdJerk` / `EntryCmdStep` measure the command-side bite around that same stop-entry transition.")
  lines.append("- `EndJerk` is max |daEgo/dt| in a narrow window around standstill hold.")
  lines.append("- `CmdJerk` and `CmdStep` are command-side stop sharpness proxies near standstill hold.")
  lines.append("- `WheelDecel` and `WheelDrop150ms` are wheel-speed based stop sharpness proxies around hold.")
  lines.append("- `ReAccel` marks low-speed oscillation before final hold (stop/re-accelerate/re-stop pattern).")
  lines.append("- `SigDrop`/`ExitStop`/`PosCmd`/`Rebound` highlight near-hold stop-intent dropouts and release behavior.")
  lines.append("- `PosCmdSig` and `ReboundSig` isolate release/rebound while stop signal is still true (higher confidence fault signal).")
  lines.append("- `Rebound while shouldStop` and `decel-relief under shouldStop` target clutch disturbance where stop intent remains active.")
  lines.append("- `Min cmd` comes from `carControl.actuators.accel` in qlog.")
  lines.append("- Use event plots for shape comparison before/after tuning changes.")
  lines.append("")

  return "\n".join(lines)


def build_output_dir(base: Path, host: str, route: str, override: str | None) -> Path:
  if override:
    return Path(override).expanduser()
  stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
  return base.expanduser() / host / route / stamp


def main() -> int:
  args = parse_args()

  all_segments = iter_qlog_files(Path(args.download_root), args.host)
  route = pick_route(all_segments, args.route)
  route_segments = sorted(
    [item for item in all_segments if item.route == route],
    key=lambda item: item.segment,
  )
  if not route_segments:
    raise RuntimeError(f"No qlogs found for route: {route}")

  if args.max_segments > 0 and len(route_segments) > args.max_segments:
    route_segments = route_segments[-args.max_segments:]

  samples = load_samples(route_segments)
  if not samples:
    raise RuntimeError("No carState samples found in selected qlogs")

  event_ranges = find_stop_events_with_source(
    samples=samples,
    min_entry_speed=args.min_entry_speed,
    entry_lookback=args.entry_lookback,
    standstill_speed=args.standstill_speed,
    hold_time=args.hold_time,
    max_stop_search=args.max_stop_search,
    event_mode=args.event_mode,
    require_enabled_speed_events=args.require_enabled_speed_events,
  )

  if args.max_events > 0:
    event_ranges = event_ranges[:args.max_events]

  output_dir = build_output_dir(Path(args.analysis_root), args.host, route, args.output_dir)
  output_dir.mkdir(parents=True, exist_ok=True)
  graph_dir = output_dir / "events"
  graph_dir.mkdir(parents=True, exist_ok=True)

  events: list[StopEvent] = []
  for idx, (start_idx, stop_idx, hold_idx, approach_speed, event_source) in enumerate(event_ranges, start=1):
    graph_file = f"events/event_{idx:03d}_seg_{samples[start_idx].segment:03d}.html"
    event = compute_event(idx, event_source, samples, start_idx, stop_idx, hold_idx, approach_speed, graph_file)
    make_event_plot(event, samples, output_dir / graph_file, args.pre_window, args.post_window)
    events.append(event)

  settings_file = Path(args.settings_file).expanduser() if args.settings_file else load_latest_settings(Path(args.settings_dir), args.host)
  summary_md = build_summary_markdown(route, len(route_segments), args.event_mode, events, output_dir, settings_file)

  summary_path = output_dir / "summary.md"
  summary_path.write_text(summary_md + "\n")

  json_payload: dict[str, Any] = {
    "generated_utc": utc_now_iso(),
    "host": args.host,
    "route": route,
    "segments_analyzed": [item.segment for item in route_segments],
    "qlog_count": len(route_segments),
    "sample_count": len(samples),
    "event_mode": args.event_mode,
    "require_enabled_speed_events": bool(args.require_enabled_speed_events),
    "event_count": len(events),
    "settings_file": str(settings_file) if settings_file else None,
    "events": [asdict(item) for item in events],
  }
  json_path = output_dir / "summary.json"
  json_path.write_text(json.dumps(json_payload, indent=2, sort_keys=True) + "\n")

  print(f"[analysis] host={args.host}")
  print(f"[analysis] route={route}")
  print(f"[analysis] qlogs={len(route_segments)} samples={len(samples)}")
  print(f"[analysis] detected_stop_events={len(events)}")
  print(f"[analysis] summary_markdown={summary_path}")
  print(f"[analysis] summary_json={json_path}")
  return 0


if __name__ == "__main__":
  raise SystemExit(main())
