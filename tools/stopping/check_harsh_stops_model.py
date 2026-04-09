#!/usr/bin/env python3
"""Run model-based harsh-stop regression checks from stop-event logs."""

from __future__ import annotations

import argparse
import json
import sys
from datetime import UTC, datetime
from pathlib import Path
from typing import Any

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
  sys.path.insert(0, str(REPO_ROOT))

from openpilot.common.numpy_fast import clip, interp
from openpilot.selfdrive.controls.lib.stopping_controller import StoppingController
from openpilot.tools.stopping.analyze_stopping_behavior import (  # pylint: disable=wrong-import-position
  DEFAULT_DOWNLOAD_ROOT,
  SegmentFile,
  iter_qlog_files,
  load_samples,
)
from openpilot.tools.stopping.stopping_model import FittedStoppingModel, simulate_event_with_model  # pylint: disable=wrong-import-position

STANDSTILL_SPEED_MPS = 0.05
STANDSTILL_CMD_JERK_TAU_S = 0.40
STOPPING_ACCEL_V_BP = [0.01, 0.20, 0.50]
STOPPING_ACCEL_MAX_BP = [-0.01, -0.10, -0.30]
DEFAULT_MIN_PRED_LEAD_HOLD_DISTANCE_M = 2.0
DEFAULT_MAX_PRED_LEAD_HOLD_DISTANCE_M = 4.0
RECORDED_LEAD_HOLD_LONG_SLACK_M = 0.15


def parse_args() -> argparse.Namespace:
  parser = argparse.ArgumentParser(description="Model-based harsh-stop gate from stop-event summary JSON files")
  parser.add_argument("--model-json", required=True, help="Path to model JSON produced by fit_stopping_model.py")
  parser.add_argument("--summary-json", action="append", required=True,
                      help="Path to analyze_stopping_behavior summary.json (repeatable)")
  parser.add_argument("--download-root", default=str(DEFAULT_DOWNLOAD_ROOT),
                      help=f"Local log root used by analyzer. Default: {DEFAULT_DOWNLOAD_ROOT}")
  parser.add_argument("--event-source", choices=["all", "signal", "speed", "hybrid"], default="all",
                      help="Event source filter from summary event_source")
  parser.add_argument("--command-source", choices=["recorded", "controller"], default="recorded",
                      help="Use recorded accel_cmd or replay controller outputs through fitted model")
  parser.add_argument("--stopping-speed-breakpoint", type=float, default=0.40,
                      help="Controller replay parameter for stopping speed breakpoint")
  parser.add_argument("--stop-accel", type=float, default=-2.0,
                      help="Controller replay stop accel floor")
  parser.add_argument("--controller-scope", choices=["all", "engaged", "engaged_stopping"], default="engaged_stopping",
                      help="Controller replay event scope: all events, engaged events, or engaged events with stopping state")
  parser.add_argument("--controller-min-enabled-ratio", type=float, default=0.80,
                      help="Minimum enabled ratio in replay window for controller scope that requires engagement")
  parser.add_argument("--controller-window-mode", choices=["event", "should_stop", "stopping_state"], default="stopping_state",
                      help="For controller replay: start at event start, first shouldStop sample, or first stopping-state sample")
  parser.add_argument("--controller-end-mode", choices=["hold", "last_should_stop", "last_stopping_state"], default="last_stopping_state",
                      help="For controller replay: end at event hold index, last shouldStop sample, or last stopping-state sample")
  parser.add_argument("--controller-should-stop-source", choices=["recorded", "constant_true"], default="recorded",
                      help="For controller replay: use recorded shouldStop samples, or force shouldStop=true across the replay window")
  parser.add_argument("--min-events", type=int, default=4, help="Minimum event count required to evaluate")
  parser.add_argument("--min-entry-speed", type=float, default=0.20, help="Ignore events below this entry speed")
  parser.add_argument("--max-harsh-rate", type=float, default=0.20, help="Maximum allowed harsh-event rate [0..1]")
  parser.add_argument("--max-leapfrog-rate", type=float, default=1.0, help="Maximum allowed leapfrog-event rate [0..1] (1.0 disables gating)")
  parser.add_argument("--max-leapfrog-count", type=int, default=0, help="Maximum allowed leapfrog-event count (0 = disabled)")
  parser.add_argument("--max-pred-end-jerk", type=float, default=0.80, help="Predicted harsh threshold for end-stop jerk")
  parser.add_argument("--max-pred-end-cmd-jerk", type=float, default=3.0, help="Predicted harsh threshold for end-stop command jerk")
  parser.add_argument("--max-pred-end-accel-step", type=float, default=0.08, help="Predicted harsh threshold for end-stop acceleration step")
  parser.add_argument("--min-pred-a-floor", type=float, default=-1.10, help="Predicted harsh threshold for minimum acceleration")
  parser.add_argument("--max-pred-rollout-m", type=float, default=2.0, help="Predicted rollout limit for ranking and harsh gating")
  parser.add_argument("--min-pred-lead-hold-distance-m", type=float, default=DEFAULT_MIN_PRED_LEAD_HOLD_DISTANCE_M,
                      help="For lead-follow stops, minimum acceptable predicted final hold gap to lead")
  parser.add_argument("--max-pred-lead-hold-distance-m", type=float, default=DEFAULT_MAX_PRED_LEAD_HOLD_DISTANCE_M,
                      help="For lead-follow stops, maximum acceptable predicted final hold gap to lead")
  parser.add_argument("--max-pred-speed-rebound-while-should-stop", type=float, default=0.08,
                      help="Predicted leapfrog threshold for speed rebound while shouldStop is active")
  parser.add_argument("--max-pred-should-stop-unexpected-accel", type=float, default=0.10,
                      help="Predicted leapfrog threshold for unexpected acceleration while shouldStop is active")
  parser.add_argument("--output-json", default=None, help="Optional path to write machine-readable check output")
  return parser.parse_args()


def load_json(path: Path) -> dict[str, Any]:
  data = json.loads(path.read_text())
  if not isinstance(data, dict):
    raise ValueError(f"JSON root must be object: {path}")
  return data


def nearest_index(times: np.ndarray, target: float) -> int:
  idx = int(np.searchsorted(times, target, side="left"))
  if idx <= 0:
    return 0
  if idx >= len(times):
    return len(times) - 1
  prev_idx = idx - 1
  return idx if abs(times[idx] - target) < abs(times[prev_idx] - target) else prev_idx


def route_samples(
  sample_cache: dict[tuple[str, str], list[Any]],
  segment_cache: dict[str, list[SegmentFile]],
  download_root: Path,
  host: str,
  route: str,
) -> list[Any]:
  key = (host, route)
  if key in sample_cache:
    return sample_cache[key]
  if host not in segment_cache:
    segment_cache[host] = iter_qlog_files(download_root, host)
  route_segments = sorted((seg for seg in segment_cache[host] if seg.route == route), key=lambda item: item.segment)
  if not route_segments:
    raise RuntimeError(f"No route segments found for host={host} route={route}")
  samples = load_samples(route_segments)
  sample_cache[key] = samples
  return samples


def build_result(status: str, reasons: list[str], event_rows: list[dict[str, Any]], args: argparse.Namespace) -> dict[str, Any]:
  harsh_count = sum(1 for row in event_rows if row["is_harsh"])
  leapfrog_count = sum(1 for row in event_rows if row.get("is_leapfrog"))
  total = len(event_rows)
  harsh_rate = (harsh_count / total) if total else 0.0
  leapfrog_rate = (leapfrog_count / total) if total else 0.0
  avg_score = (sum(float(row.get("event_score", 0.0)) for row in event_rows) / total) if total else 0.0
  return {
    "generated_utc": datetime.now(UTC).replace(microsecond=0).isoformat(),
    "status": status,
    "reasons": reasons,
    "events_considered": total,
    "harsh_events": harsh_count,
    "harsh_rate": harsh_rate,
    "leapfrog_events": leapfrog_count,
    "leapfrog_rate": leapfrog_rate,
    "avg_event_score": avg_score,
    "thresholds": {
      "min_events": args.min_events,
      "min_entry_speed": args.min_entry_speed,
      "max_harsh_rate": args.max_harsh_rate,
      "max_leapfrog_rate": args.max_leapfrog_rate,
      "max_leapfrog_count": args.max_leapfrog_count,
      "max_pred_end_jerk": args.max_pred_end_jerk,
      "max_pred_end_cmd_jerk": args.max_pred_end_cmd_jerk,
      "max_pred_end_accel_step": args.max_pred_end_accel_step,
      "min_pred_a_floor": args.min_pred_a_floor,
      "max_pred_rollout_m": args.max_pred_rollout_m,
      "min_pred_lead_hold_distance_m": args.min_pred_lead_hold_distance_m,
      "max_pred_lead_hold_distance_m": args.max_pred_lead_hold_distance_m,
      "max_pred_speed_rebound_while_should_stop": args.max_pred_speed_rebound_while_should_stop,
      "max_pred_should_stop_unexpected_accel": args.max_pred_should_stop_unexpected_accel,
      "controller_should_stop_source": getattr(args, "controller_should_stop_source", "constant_true"),
    },
    "command_source": args.command_source,
    "event_rows": event_rows,
  }


def jerk_window_metrics(
  times: list[float],
  predicted: list[float],
  hold_time_s: float,
  predicted_v: list[float] | None = None,
) -> tuple[float | None, float]:
  standstill_index: int | None = None
  if predicted_v is not None:
    for idx, speed in enumerate(predicted_v):
      if speed < STANDSTILL_SPEED_MPS:
        standstill_index = idx
        break

  pre_hold_indices = [
    idx for idx, t in enumerate(times)
    if (times[max(0, idx - 1)] if idx > 0 else t) >= (hold_time_s - 0.8)
  ]
  if standstill_index is not None:
    pre_hold_indices = [idx for idx in pre_hold_indices if idx <= standstill_index]
    if len(pre_hold_indices) < 2:
      window_start = max(0, standstill_index - 8)
      pre_hold_indices = list(range(window_start, standstill_index + 1))
  if len(pre_hold_indices) < 2:
    pre_hold_indices = list(range(len(times)))

  max_jerk: float | None = None
  for prev_i, cur_i in zip(pre_hold_indices, pre_hold_indices[1:], strict=False):
    if standstill_index is None and predicted_v is not None and max(predicted_v[prev_i], predicted_v[cur_i]) < STANDSTILL_SPEED_MPS:
      # Ignore post-standstill relaxation spikes; focus jerk metric on moving-speed stop phase.
      continue
    dt = times[cur_i] - times[prev_i]
    if dt <= 1e-6:
      continue
    jerk = abs((predicted[cur_i] - predicted[prev_i]) / dt)
    max_jerk = jerk if max_jerk is None else max(max_jerk, jerk)

  if max_jerk is None and predicted_v is not None:
    for prev_i, cur_i in zip(pre_hold_indices, pre_hold_indices[1:], strict=False):
      dt = times[cur_i] - times[prev_i]
      if dt <= 1e-6:
        continue
      jerk = abs((predicted[cur_i] - predicted[prev_i]) / dt)
      max_jerk = jerk if max_jerk is None else max(max_jerk, jerk)

  hold_index = len(times) - 1
  min_window_start = max(0, hold_index - 30)
  pred_min_a = min(predicted[min_window_start:hold_index + 1])
  return max_jerk, pred_min_a


def score_event_metrics(
  pred_jerk: float | None,
  pred_min_a: float,
  pred_rollout_m: float | None,
  max_rollout_m: float,
  *,
  pred_lead_hold_m: float | None = None,
  recorded_lead_hold_m: float | None = None,
  min_lead_hold_m: float | None = None,
  max_lead_hold_m: float | None = None,
  pred_cmd_jerk: float | None = None,
  max_cmd_jerk: float | None = None,
  pred_accel_step: float | None = None,
  max_accel_step: float | None = None,
) -> float:
  jerk_component = max(float(pred_jerk or 0.0), 0.0)
  floor_component = max(0.0, -1.0 - float(pred_min_a))
  rollout_component = max(0.0, float(pred_rollout_m or 0.0) - max_rollout_m)
  lead_gap_component = 0.0
  if (
    pred_lead_hold_m is not None
    and min_lead_hold_m is not None
    and max_lead_hold_m is not None
    and max_lead_hold_m > min_lead_hold_m
  ):
    effective_max_lead_hold_m = float(max_lead_hold_m)
    if recorded_lead_hold_m is not None:
      effective_max_lead_hold_m = max(effective_max_lead_hold_m, float(recorded_lead_hold_m) + RECORDED_LEAD_HOLD_LONG_SLACK_M)
    target_lead_hold_m = 0.5 * (float(min_lead_hold_m) + effective_max_lead_hold_m)
    half_band_m = 0.5 * (effective_max_lead_hold_m - float(min_lead_hold_m))
    lead_gap_error_m = abs(float(pred_lead_hold_m) - target_lead_hold_m)
    rollout_component = 0.0
    # Keep a mild preference for the middle of the acceptable band while strongly penalizing excursions.
    lead_gap_component = 0.15 * (lead_gap_error_m / max(half_band_m, 1e-6))
    lead_gap_component += 2.5 * max(0.0, lead_gap_error_m - half_band_m)
  cmd_component = 0.0
  accel_step_component = 0.0
  if pred_cmd_jerk is not None and max_cmd_jerk is not None and max_cmd_jerk > 1e-6:
    cmd_component = max(0.0, float(pred_cmd_jerk) - float(max_cmd_jerk)) / float(max_cmd_jerk)
  if pred_accel_step is not None and max_accel_step is not None and max_accel_step > 1e-6:
    accel_step_component = max(0.0, float(pred_accel_step) - float(max_accel_step)) / float(max_accel_step)
  return (
    jerk_component
    + (0.8 * floor_component)
    + (2.5 * rollout_component)
    + lead_gap_component
    + (0.7 * cmd_component)
    + (1.2 * accel_step_component)
  )


def classify_stop_distance(
  pred_rollout_m: float | None,
  pred_lead_hold_m: float | None,
  *,
  max_rollout_m: float,
  min_lead_hold_m: float,
  max_lead_hold_m: float,
  recorded_lead_hold_m: float | None = None,
) -> tuple[list[str], str, float | None]:
  if pred_lead_hold_m is not None:
    lead_hold = float(pred_lead_hold_m)
    effective_max_lead_hold_m = float(max_lead_hold_m)
    if recorded_lead_hold_m is not None:
      effective_max_lead_hold_m = max(effective_max_lead_hold_m, float(recorded_lead_hold_m) + RECORDED_LEAD_HOLD_LONG_SLACK_M)
    flags: list[str] = []
    if lead_hold < float(min_lead_hold_m):
      flags.append("pred_lead_distance_hold_short")
    if lead_hold > effective_max_lead_hold_m:
      flags.append("pred_lead_distance_hold_long")
    return flags, "lead_hold", lead_hold

  rollout = float(pred_rollout_m) if pred_rollout_m is not None else None
  flags = []
  if rollout is not None and rollout > float(max_rollout_m):
    flags.append("pred_rollout")
  return flags, "rollout_2mps", rollout


def stopping_accel_breakpoints(stopping_speed_breakpoint: float) -> tuple[list[float], list[float]]:
  mid_bp = clip(stopping_speed_breakpoint, STOPPING_ACCEL_V_BP[0] + 0.001, STOPPING_ACCEL_V_BP[-1] - 0.001)
  return [STOPPING_ACCEL_V_BP[0], mid_bp, STOPPING_ACCEL_V_BP[-1]], STOPPING_ACCEL_MAX_BP


def infer_hold_time_s(times: list[float], predicted_v: list[float] | None = None) -> float:
  if times:
    hold_time_s = float(times[-1])
  else:
    hold_time_s = 0.0
  if predicted_v is not None and times:
    for idx, speed in enumerate(predicted_v):
      if idx >= len(times):
        break
      if float(speed) < STANDSTILL_SPEED_MPS:
        hold_time_s = float(times[idx])
        break
  return hold_time_s


def compute_end_stop_sharpness_metrics(
  times: list[float],
  predicted_a: list[float],
  hold_time_s: float,
  predicted_cmd: list[float] | None = None,
) -> tuple[float | None, float | None]:
  jerk_indices = [
    idx
    for idx, t in enumerate(times)
    if (hold_time_s - 0.45) <= float(t) <= (hold_time_s + 0.20)
  ]

  cmd_jerk: float | None = None
  if predicted_cmd is not None and len(predicted_cmd) >= 2 and len(times) >= 2:
    valid_indices = [idx for idx in jerk_indices if idx < len(predicted_cmd)]
    if len(valid_indices) >= 2:
      for prev_idx, cur_idx in zip(valid_indices, valid_indices[1:], strict=False):
        dt = float(times[cur_idx]) - float(times[prev_idx])
        if dt <= 1e-6:
          continue
        slope = abs((float(predicted_cmd[cur_idx]) - float(predicted_cmd[prev_idx])) / dt)
        cmd_jerk = slope if cmd_jerk is None else max(cmd_jerk, slope)

  pre_accel_values = [
    float(predicted_a[idx])
    for idx, t in enumerate(times)
    if idx < len(predicted_a) and (hold_time_s - 0.50) <= float(t) <= (hold_time_s - 0.10)
  ]
  hold_accel_values = [
    float(predicted_a[idx])
    for idx, t in enumerate(times)
    if idx < len(predicted_a) and hold_time_s <= float(t) <= (hold_time_s + 0.15)
  ]
  accel_step: float | None = None
  if pre_accel_values and hold_accel_values:
    accel_step = abs(float(np.mean(hold_accel_values)) - float(np.mean(pre_accel_values)))

  return cmd_jerk, accel_step


def compute_entry_stop_sharpness_metrics(
  times: list[float],
  predicted_a: list[float],
  entry_time_s: float | None,
  predicted_cmd: list[float] | None = None,
) -> tuple[float | None, float | None, float | None, float | None]:
  if entry_time_s is None:
    return None, None, None, None

  jerk_indices = [
    idx
    for idx, t in enumerate(times)
    if (entry_time_s - 0.10) <= float(t) <= (entry_time_s + 0.35)
  ]
  accel_jerk = None
  if len(jerk_indices) >= 2:
    for prev_idx, cur_idx in zip(jerk_indices, jerk_indices[1:], strict=False):
      dt = float(times[cur_idx]) - float(times[prev_idx])
      if dt <= 1e-6:
        continue
      slope = abs((float(predicted_a[cur_idx]) - float(predicted_a[prev_idx])) / dt)
      accel_jerk = slope if accel_jerk is None else max(accel_jerk, slope)

  cmd_jerk: float | None = None
  if predicted_cmd is not None and len(predicted_cmd) >= 2:
    valid_indices = [idx for idx in jerk_indices if idx < len(predicted_cmd)]
    if len(valid_indices) >= 2:
      for prev_idx, cur_idx in zip(valid_indices, valid_indices[1:], strict=False):
        dt = float(times[cur_idx]) - float(times[prev_idx])
        if dt <= 1e-6:
          continue
        slope = abs((float(predicted_cmd[cur_idx]) - float(predicted_cmd[prev_idx])) / dt)
        cmd_jerk = slope if cmd_jerk is None else max(cmd_jerk, slope)

  pre_accel_values = [
    float(predicted_a[idx])
    for idx, t in enumerate(times)
    if idx < len(predicted_a) and (entry_time_s - 0.30) <= float(t) <= (entry_time_s - 0.05)
  ]
  post_accel_values = [
    float(predicted_a[idx])
    for idx, t in enumerate(times)
    if idx < len(predicted_a) and entry_time_s <= float(t) <= (entry_time_s + 0.15)
  ]
  accel_step: float | None = None
  if pre_accel_values and post_accel_values:
    accel_step = abs(float(np.mean(post_accel_values)) - float(np.mean(pre_accel_values)))

  cmd_step: float | None = None
  if predicted_cmd is not None:
    pre_cmd_values = [
      float(predicted_cmd[idx])
      for idx, t in enumerate(times)
      if idx < len(predicted_cmd) and (entry_time_s - 0.30) <= float(t) <= (entry_time_s - 0.05)
    ]
    post_cmd_values = [
      float(predicted_cmd[idx])
      for idx, t in enumerate(times)
      if idx < len(predicted_cmd) and entry_time_s <= float(t) <= (entry_time_s + 0.15)
    ]
    if pre_cmd_values and post_cmd_values:
      cmd_step = abs(float(np.mean(post_cmd_values)) - float(np.mean(pre_cmd_values)))

  return accel_jerk, accel_step, cmd_jerk, cmd_step


def sample_value(sample: Any, key: str, default: Any = None) -> Any:
  if isinstance(sample, dict):
    return sample.get(key, default)
  return getattr(sample, key, default)


def controller_should_stop_flags(samples: list[Any], start_idx: int, end_idx: int, source: str) -> list[bool]:
  start = max(0, int(start_idx))
  end = max(start, int(end_idx))
  count = end - start + 1
  if count <= 0:
    return []
  if source == "constant_true":
    return [True] * count

  has_should_stop_field = any(
    sample_value(samples[idx], "should_stop", None) is not None
    for idx in range(start, end + 1)
  )
  if not has_should_stop_field:
    # Preserve legacy behavior for old synthetic fixtures without should_stop.
    return [True] * count
  return [bool(sample_value(samples[idx], "should_stop", False)) for idx in range(start, end + 1)]


def time_window_median(
  times: list[float],
  values: list[float | None],
  anchor_time_s: float | None,
  *,
  pre_window_s: float,
  post_window_s: float,
  predicted_v: list[float] | None = None,
  require_stopped: bool = False,
) -> float | None:
  if anchor_time_s is None:
    return None
  window_values: list[float] = []
  for idx, t in enumerate(times):
    if idx >= len(values):
      break
    value = values[idx]
    if value is None:
      continue
    if float(t) < (anchor_time_s - pre_window_s) or float(t) > (anchor_time_s + post_window_s):
      continue
    if require_stopped:
      if predicted_v is None or idx >= len(predicted_v) or float(predicted_v[idx]) >= STANDSTILL_SPEED_MPS:
        continue
    window_values.append(float(value))
  return float(np.median(window_values)) if window_values else None


def compute_predicted_lead_distance_metrics(
  samples: list[Any],
  replay_sample_indices: list[int],
  times: list[float],
  predicted_v: list[float],
  predicted_distance_m: list[float],
  entry_time_s: float | None,
  hold_time_s: float | None,
) -> tuple[float | None, float | None, float | None]:
  if not replay_sample_indices or len(replay_sample_indices) != len(times) or len(predicted_distance_m) != len(times):
    return None, None, None

  recorded_distance_m = [0.0]
  for prev_idx, cur_idx in zip(replay_sample_indices, replay_sample_indices[1:], strict=False):
    prev_t = float(sample_value(samples[prev_idx], "t", 0.0) or 0.0)
    cur_t = float(sample_value(samples[cur_idx], "t", prev_t) or prev_t)
    dt = max(0.0, cur_t - prev_t)
    prev_v = max(0.0, float(sample_value(samples[prev_idx], "v_ego", 0.0) or 0.0))
    cur_v = max(0.0, float(sample_value(samples[cur_idx], "v_ego", prev_v) or prev_v))
    recorded_distance_m.append(recorded_distance_m[-1] + max(0.0, 0.5 * (prev_v + cur_v) * dt))

  predicted_gap_m: list[float | None] = []
  recorded_gap_m: list[float | None] = []
  recorded_v_ego: list[float] = []
  for idx, sample_idx in enumerate(replay_sample_indices):
    recorded_v_ego.append(max(0.0, float(sample_value(samples[sample_idx], "v_ego", 0.0) or 0.0)))
    lead_status = bool(sample_value(samples[sample_idx], "lead_status", False))
    lead_d_rel = sample_value(samples[sample_idx], "lead_d_rel_m", None)
    if not lead_status or lead_d_rel is None:
      predicted_gap_m.append(None)
      recorded_gap_m.append(None)
      continue
    recorded_gap_m.append(float(lead_d_rel))
    lead_abs_distance_m = recorded_distance_m[idx] + float(lead_d_rel)
    predicted_gap_m.append(lead_abs_distance_m - float(predicted_distance_m[idx]))

  hold_gap = time_window_median(
    times,
    predicted_gap_m,
    hold_time_s,
    pre_window_s=0.15,
    post_window_s=0.15,
    predicted_v=predicted_v,
    require_stopped=True,
  )
  if hold_gap is None:
    # When replay does not quite hit the standstill threshold inside a measured stop window,
    # still report the final-window lead gap instead of dropping back to rollout-only gating.
    hold_gap = time_window_median(times, predicted_gap_m, hold_time_s, pre_window_s=0.15, post_window_s=0.15)

  recorded_hold_gap = time_window_median(
    times,
    recorded_gap_m,
    hold_time_s,
    pre_window_s=0.15,
    post_window_s=0.15,
    predicted_v=recorded_v_ego,
    require_stopped=True,
  )
  if recorded_hold_gap is None:
    recorded_hold_gap = time_window_median(times, recorded_gap_m, hold_time_s, pre_window_s=0.15, post_window_s=0.15)

  return (
    time_window_median(times, predicted_gap_m, entry_time_s, pre_window_s=0.0, post_window_s=0.15),
    hold_gap,
    recorded_hold_gap,
  )


def compute_pred_leapfrog_metrics(
  predicted_v: list[float],
  predicted_a: list[float],
  *,
  max_accel_v_bp: list[float],
  max_accel_bp: list[float],
  should_stop_mask: list[bool] | None = None,
) -> tuple[float, float]:
  if not predicted_v or not predicted_a:
    return 0.0, 0.0
  max_len = min(len(predicted_v), len(predicted_a))
  if max_len <= 0:
    return 0.0, 0.0

  active_indices = list(range(max_len))
  if should_stop_mask is not None:
    active_indices = [idx for idx in active_indices if idx < len(should_stop_mask) and bool(should_stop_mask[idx])]
  if not active_indices:
    return 0.0, 0.0

  min_idx = min(active_indices, key=lambda idx: float(predicted_v[idx]))
  min_speed = float(predicted_v[min_idx])
  post_min_active = [idx for idx in active_indices if idx >= min_idx]
  if not post_min_active:
    post_min_active = [min_idx]
  rebound = max(0.0, max(float(predicted_v[idx]) for idx in post_min_active) - min_speed)

  unexpected_accel = 0.0
  for idx in active_indices:
    v_ego = predicted_v[idx]
    a_ego = predicted_a[idx]
    max_expected = interp(float(v_ego), max_accel_v_bp, max_accel_bp)
    unexpected_accel = max(unexpected_accel, float(a_ego) - float(max_expected))

  return rebound, max(unexpected_accel, 0.0)


def classify_pred_leapfrog(
  pred_rebound_while_should_stop: float | None,
  pred_should_stop_unexpected_accel: float | None,
  args: argparse.Namespace,
) -> list[str]:
  leapfrog_flags: list[str] = []
  rebound_flag = (
    pred_rebound_while_should_stop is not None
    and pred_rebound_while_should_stop > args.max_pred_speed_rebound_while_should_stop
  )
  unexpected_accel_flag = (
    pred_should_stop_unexpected_accel is not None
    and pred_should_stop_unexpected_accel > args.max_pred_should_stop_unexpected_accel
  )
  if rebound_flag:
    leapfrog_flags.append("pred_leapfrog_rebound_should_stop")
  if rebound_flag and unexpected_accel_flag:
    leapfrog_flags.append("pred_leapfrog")
  return leapfrog_flags


def first_index_in_range(samples: list[Any], start_idx: int, end_idx: int, predicate) -> int | None:
  for idx in range(start_idx, end_idx + 1):
    if predicate(samples[idx]):
      return idx
  return None


def last_index_in_range(samples: list[Any], start_idx: int, end_idx: int, predicate) -> int | None:
  for idx in range(end_idx, start_idx - 1, -1):
    if predicate(samples[idx]):
      return idx
  return None


def last_contiguous_index_span(samples: list[Any], start_idx: int, end_idx: int, predicate) -> tuple[int, int] | None:
  """Return the last contiguous active span within [start_idx, end_idx]."""
  idx = end_idx
  while idx >= start_idx:
    if predicate(samples[idx]):
      span_end = idx
      while idx >= start_idx and predicate(samples[idx]):
        idx -= 1
      span_start = idx + 1
      return span_start, span_end
    idx -= 1
  return None


def simulate_event_with_controller(
  samples: list[Any],
  start_idx: int,
  hold_idx: int,
  model: FittedStoppingModel,
  stopping_speed_breakpoint: float,
  stop_accel: float,
  controller_should_stop_source: str = "constant_true",
  return_trace: bool = False,
) -> dict[str, Any]:
  start = max(0, int(start_idx))
  hold = max(start + 1, min(int(hold_idx), len(samples) - 1))
  if hold <= start:
    raise ValueError("Event window too short for replay")

  v_bp, max_accel_bp = stopping_accel_breakpoints(stopping_speed_breakpoint)
  min_accel_bp = [-0.10, -0.50, -1.00]

  controller = StoppingController()
  dt = max(model.dt_s, 1e-3)
  v_ego = float(samples[start].v_ego)
  a_ego = float(samples[start].a_ego)
  history_start = max(0, start - model.delay_frames)
  command_trace: list[float] = []
  for idx in range(history_start, start + 1):
    cmd = samples[idx].accel_cmd
    if cmd is None:
      cmd = command_trace[-1] if command_trace else -0.12
    command_trace.append(float(cmd))

  last_output = command_trace[-1] if command_trace else (-0.12)
  controller.seed_command_history(command_trace)
  output_trace = [float(last_output)]
  times = [float(samples[start].t)]
  predicted = [a_ego]
  predicted_v = [v_ego]
  predicted_distance_m = [0.0]
  replay_sample_indices = [start]
  should_stop_flags = controller_should_stop_flags(samples, start, hold, controller_should_stop_source)
  should_stop_trace = [bool(should_stop_flags[0])] if should_stop_flags else [True]
  rollout_distance_m = 0.0
  rollout_from_2mps_m = 0.0
  standstill_steps = 0
  standstill_clamp_steps = max(1, int(round(0.6 / dt)))

  for step_offset, sample_idx in enumerate(range(start, hold)):
    should_stop_now = should_stop_flags[step_offset] if step_offset < len(should_stop_flags) else True
    if should_stop_now and v_ego <= 1e-4:
      standstill_steps += 1
    else:
      standstill_steps = 0
    if should_stop_now:
      output_seed = min(last_output, -0.1)
    else:
      sample_cmd = sample_value(samples[sample_idx], "accel_cmd", None)
      output_seed = float(sample_cmd) if sample_cmd is not None else float(last_output)
    max_expected = interp(v_ego, v_bp, max_accel_bp)
    min_expected = interp(v_ego, v_bp, min_accel_bp)
    result = controller.update(
      output_accel=output_seed,
      last_output_accel=last_output,
      should_stop=should_stop_now,
      v_ego=v_ego,
      a_ego=a_ego,
      max_expected_accel=max_expected,
      min_expected_accel=min_expected,
      stop_accel=stop_accel,
      dt=dt,
      distance_to_stop_target_m=sample_value(samples[sample_idx], "distance_to_stop_target_m", None),
    )
    output_cmd = float(result.output_accel)
    command_trace.append(output_cmd)
    output_trace.append(output_cmd)
    delayed_idx = max(0, len(command_trace) - 1 - model.delay_frames)
    delayed_cmd = float(command_trace[delayed_idx])
    a_next = model.predict_next(a_ego, delayed_cmd, v_ego)
    a_next = clip(a_next, -4.0, 3.0)
    if (
      controller_should_stop_source == "recorded"
      and should_stop_now
      and standstill_steps >= standstill_clamp_steps
      and v_ego < 0.01
      and output_cmd <= -0.20
      and a_next > 0.0
    ):
      # The fitted first-order model can drift into positive accel at standstill.
      # Clamp this narrow regime to avoid synthetic creep/leapfrog inflation in replay.
      a_next = 0.0
    prev_v_ego = v_ego
    v_ego = max(0.0, v_ego + (a_next * dt))
    step_distance_m = max(0.0, 0.5 * (prev_v_ego + v_ego) * dt)
    rollout_distance_m += step_distance_m
    if prev_v_ego <= 2.0 and v_ego <= 2.0:
      rollout_from_2mps_m += step_distance_m
    elif prev_v_ego > 2.0 >= v_ego:
      if abs(prev_v_ego - v_ego) <= 1e-6:
        rollout_from_2mps_m += step_distance_m
      else:
        # Linear crossing estimate to count only distance below 2 m/s.
        below_fraction = clip((2.0 - v_ego) / (prev_v_ego - v_ego), 0.0, 1.0)
        rollout_from_2mps_m += step_distance_m * below_fraction
    a_ego = float(a_next)
    last_output = output_cmd
    predicted.append(a_ego)
    predicted_v.append(v_ego)
    predicted_distance_m.append(predicted_distance_m[-1] + step_distance_m)
    next_should_stop = should_stop_flags[step_offset + 1] if step_offset + 1 < len(should_stop_flags) else should_stop_now
    should_stop_trace.append(bool(next_should_stop))
    times.append(times[-1] + dt)
    replay_sample_indices.append(min(sample_idx + 1, hold))

  hold_time_s = infer_hold_time_s(times, predicted_v)
  entry_time_s = None
  for idx, should_stop_now in enumerate(should_stop_trace):
    if should_stop_now:
      entry_time_s = float(times[idx])
      break
  pred_jerk, pred_min_a = jerk_window_metrics(times, predicted, hold_time_s, predicted_v=predicted_v)
  pred_entry_jerk, pred_entry_accel_step, pred_entry_cmd_jerk, pred_entry_cmd_step = compute_entry_stop_sharpness_metrics(
    times=times,
    predicted_a=predicted,
    entry_time_s=entry_time_s,
    predicted_cmd=output_trace,
  )
  pred_cmd_jerk, pred_accel_step = compute_end_stop_sharpness_metrics(
    times=times,
    predicted_a=predicted,
    hold_time_s=hold_time_s,
    predicted_cmd=output_trace,
  )
  pred_lead_entry, pred_lead_hold, recorded_lead_hold = compute_predicted_lead_distance_metrics(
    samples=samples,
    replay_sample_indices=replay_sample_indices,
    times=times,
    predicted_v=predicted_v,
    predicted_distance_m=predicted_distance_m,
    entry_time_s=entry_time_s,
    hold_time_s=hold_time_s,
  )

  stop_idx: int | None = None
  for idx, v in enumerate(predicted_v):
    if v < STANDSTILL_SPEED_MPS:
      stop_idx = idx
      break

  standstill_cmd_jerk: float | None = None
  if stop_idx is not None and stop_idx > 0 and STANDSTILL_CMD_JERK_TAU_S > 1e-6 and stop_idx - 1 < len(output_trace):
    standstill_cmd_jerk = abs(float(output_trace[stop_idx - 1])) / STANDSTILL_CMD_JERK_TAU_S
    pred_cmd_jerk = standstill_cmd_jerk if pred_cmd_jerk is None else max(pred_cmd_jerk, standstill_cmd_jerk)
    pred_jerk = standstill_cmd_jerk if pred_jerk is None else max(pred_jerk, standstill_cmd_jerk)

  pred_rebound, pred_unexpected_accel = compute_pred_leapfrog_metrics(
    predicted_v=predicted_v,
    predicted_a=predicted,
    max_accel_v_bp=v_bp,
    max_accel_bp=max_accel_bp,
    should_stop_mask=should_stop_trace,
  )

  result = {
    "times": times,
    "predicted_a_ego": predicted,
    "predicted_v_ego": predicted_v,
    "pred_rollout_distance_m": rollout_distance_m,
    "pred_rollout_from_2mps_m": rollout_from_2mps_m,
    "pred_lead_distance_stop_entry_m": pred_lead_entry,
    "pred_lead_distance_hold_m": pred_lead_hold,
    "recorded_lead_distance_hold_m": recorded_lead_hold,
    "pred_entry_stop_jerk_mps3": pred_entry_jerk,
    "pred_entry_stop_accel_step_mps2": pred_entry_accel_step,
    "pred_entry_stop_cmd_jerk_mps3": pred_entry_cmd_jerk,
    "pred_entry_stop_cmd_step_mps2": pred_entry_cmd_step,
    "pred_end_stop_jerk_mps3": pred_jerk,
    "pred_end_stop_cmd_jerk_mps3": pred_cmd_jerk,
    "pred_end_stop_accel_step_mps2": pred_accel_step,
    "pred_min_a_ego_mps2": pred_min_a,
    "pred_speed_rebound_while_should_stop_mps": pred_rebound,
    "pred_should_stop_unexpected_accel_mps2": pred_unexpected_accel,
    "controller_should_stop_source": controller_should_stop_source,
  }
  if return_trace:
    result["trace"] = {
      "dt_s": dt,
      "times": [float(value) for value in times],
      "predicted_a": [float(value) for value in predicted],
      "predicted_v": [float(value) for value in predicted_v],
      "predicted_distance_m": [float(value) for value in predicted_distance_m],
      "output_trace": [float(value) for value in output_trace],
      "command_trace": [float(value) for value in command_trace],
      "replay_sample_indices": [int(value) for value in replay_sample_indices],
      "should_stop_trace": [bool(value) for value in should_stop_trace],
      "entry_time_s": float(entry_time_s) if entry_time_s is not None else None,
      "hold_time_s": float(hold_time_s),
      "start_idx": int(start),
      "hold_idx": int(hold),
      "stopping_speed_breakpoint": float(stopping_speed_breakpoint),
      "stop_accel": float(stop_accel),
      "max_accel_v_bp": [float(value) for value in v_bp],
      "max_accel_bp": [float(value) for value in max_accel_bp],
      "min_accel_bp": [float(value) for value in min_accel_bp],
    }
  return result


def enabled_ratio(samples: list[Any], start_idx: int, end_idx: int) -> float:
  if end_idx < start_idx:
    return 0.0
  flags = [1.0 if bool(samples[idx].enabled) else 0.0 for idx in range(start_idx, end_idx + 1)]
  return float(sum(flags) / max(len(flags), 1))


def main() -> int:
  args = parse_args()
  model_path = Path(args.model_json).expanduser()
  if not model_path.exists():
    print(f"[model-harsh-check] missing model: {model_path}", file=sys.stderr)
    return 2

  summary_paths = [Path(item).expanduser() for item in args.summary_json]
  missing = [str(path) for path in summary_paths if not path.exists()]
  if missing:
    for path in missing:
      print(f"[model-harsh-check] missing summary: {path}", file=sys.stderr)
    return 2

  model_payload = load_json(model_path)
  model_data = model_payload["model"] if "model" in model_payload else model_payload
  model = FittedStoppingModel.from_json(model_data)

  download_root = Path(args.download_root).expanduser()
  sample_cache: dict[tuple[str, str], list[Any]] = {}
  segment_cache: dict[str, list[SegmentFile]] = {}
  rows: list[dict[str, Any]] = []

  for summary_path in summary_paths:
    summary = load_json(summary_path)
    host = str(summary.get("host", "commawifi"))
    route = str(summary.get("route", ""))
    if not route:
      continue
    samples = route_samples(sample_cache, segment_cache, download_root, host, route)
    times = np.array([float(item.t) for item in samples], dtype=float)

    for event in summary.get("events", []):
      if not isinstance(event, dict):
        continue
      source = str(event.get("event_source", ""))
      if args.event_source != "all" and source != args.event_source:
        continue

      entry_speed = float(event.get("entry_speed_mps", 0.0))
      if entry_speed < args.min_entry_speed:
        continue

      start_time = event.get("start_time_s")
      hold_time = event.get("stop_hold_time_s")
      if start_time is None or hold_time is None:
        continue

      start_idx = nearest_index(times, float(start_time))
      hold_idx = nearest_index(times, float(hold_time))
      if hold_idx <= start_idx:
        continue

      if args.command_source == "controller":
        sim_start_idx = start_idx
        sim_hold_idx = hold_idx
        should_stop_span = last_contiguous_index_span(samples, start_idx, hold_idx, lambda item: item.should_stop)
        stopping_span = last_contiguous_index_span(
          samples,
          start_idx,
          hold_idx,
          lambda item: item.long_state == "stopping" or item.long_state_cmd == "stopping",
        )
        should_stop_start = should_stop_span[0] if should_stop_span is not None else None
        should_stop_end = should_stop_span[1] if should_stop_span is not None else None
        stopping_start = stopping_span[0] if stopping_span is not None else None
        stopping_end = stopping_span[1] if stopping_span is not None else None

        if args.controller_window_mode == "should_stop":
          if should_stop_start is None:
            continue
          sim_start_idx = should_stop_start
        elif args.controller_window_mode == "stopping_state":
          if stopping_start is None:
            continue
          sim_start_idx = stopping_start

        if args.controller_end_mode == "last_should_stop":
          if should_stop_end is None:
            continue
          sim_hold_idx = min(sim_hold_idx, should_stop_end)
        elif args.controller_end_mode == "last_stopping_state":
          if stopping_end is None:
            continue
          sim_hold_idx = min(sim_hold_idx, stopping_end)

        if sim_hold_idx <= sim_start_idx:
          continue

        stopping_window_present = stopping_start is not None and stopping_end is not None
        replay_enabled_ratio = enabled_ratio(samples, sim_start_idx, sim_hold_idx)
        if args.controller_scope in ("engaged", "engaged_stopping") and replay_enabled_ratio < args.controller_min_enabled_ratio:
          continue
        if args.controller_scope == "engaged_stopping" and not stopping_window_present:
          continue

        simulation = simulate_event_with_controller(
          samples=samples,
          start_idx=sim_start_idx,
          hold_idx=sim_hold_idx,
          model=model,
          stopping_speed_breakpoint=args.stopping_speed_breakpoint,
          stop_accel=args.stop_accel,
          controller_should_stop_source=args.controller_should_stop_source,
        )
      else:
        simulation = simulate_event_with_model(samples, start_idx, hold_idx, hold_idx, model)

      pred_jerk = simulation["pred_end_stop_jerk_mps3"]
      pred_min_a = simulation["pred_min_a_ego_mps2"]
      pred_cmd_jerk = simulation.get("pred_end_stop_cmd_jerk_mps3")
      pred_accel_step = simulation.get("pred_end_stop_accel_step_mps2")
      pred_entry_jerk = simulation.get("pred_entry_stop_jerk_mps3")
      pred_entry_accel_step = simulation.get("pred_entry_stop_accel_step_mps2")
      pred_entry_cmd_jerk = simulation.get("pred_entry_stop_cmd_jerk_mps3")
      pred_entry_cmd_step = simulation.get("pred_entry_stop_cmd_step_mps2")
      pred_rollout_total = simulation.get("pred_rollout_distance_m")
      pred_rollout = simulation.get("pred_rollout_from_2mps_m", pred_rollout_total)
      pred_lead_entry = simulation.get("pred_lead_distance_stop_entry_m")
      pred_lead_hold = simulation.get("pred_lead_distance_hold_m")
      recorded_lead_hold = simulation.get("recorded_lead_distance_hold_m")
      pred_rebound = simulation.get("pred_speed_rebound_while_should_stop_mps")
      pred_unexpected_accel = simulation.get("pred_should_stop_unexpected_accel_mps2")
      if pred_cmd_jerk is None or pred_accel_step is None:
        replay_times = list(simulation.get("times", []))
        replay_predicted_a = list(simulation.get("predicted_a_ego", []))
        replay_predicted_v = list(simulation.get("predicted_v_ego", []))
        replay_hold_time_s = infer_hold_time_s(replay_times, replay_predicted_v)
        fallback_cmd_jerk, fallback_accel_step = compute_end_stop_sharpness_metrics(
          times=replay_times,
          predicted_a=replay_predicted_a,
          hold_time_s=replay_hold_time_s,
          predicted_cmd=None,
        )
        pred_cmd_jerk = fallback_cmd_jerk if pred_cmd_jerk is None else pred_cmd_jerk
        pred_accel_step = fallback_accel_step if pred_accel_step is None else pred_accel_step
      if pred_rebound is None or pred_unexpected_accel is None:
        rebound_v_bp, rebound_max_accel_bp = stopping_accel_breakpoints(args.stopping_speed_breakpoint)
        rebound_fallback, unexpected_fallback = compute_pred_leapfrog_metrics(
          predicted_v=list(simulation.get("predicted_v_ego", [])),
          predicted_a=list(simulation.get("predicted_a_ego", [])),
          max_accel_v_bp=rebound_v_bp,
          max_accel_bp=rebound_max_accel_bp,
        )
        pred_rebound = rebound_fallback if pred_rebound is None else pred_rebound
        pred_unexpected_accel = unexpected_fallback if pred_unexpected_accel is None else pred_unexpected_accel
      harsh_flags: list[str] = []
      if pred_jerk is not None and pred_jerk > args.max_pred_end_jerk:
        harsh_flags.append("pred_end_stop_jerk")
      if pred_cmd_jerk is not None and pred_cmd_jerk > args.max_pred_end_cmd_jerk:
        harsh_flags.append("pred_end_stop_cmd_jerk")
      if pred_accel_step is not None and pred_accel_step > args.max_pred_end_accel_step:
        harsh_flags.append("pred_end_stop_accel_step")
      if pred_min_a < args.min_pred_a_floor:
        harsh_flags.append("pred_min_a_ego")
      distance_flags, distance_gate_source, distance_metric_value = classify_stop_distance(
        pred_rollout,
        pred_lead_hold,
        max_rollout_m=args.max_pred_rollout_m,
        min_lead_hold_m=args.min_pred_lead_hold_distance_m,
        max_lead_hold_m=args.max_pred_lead_hold_distance_m,
        recorded_lead_hold_m=float(recorded_lead_hold) if recorded_lead_hold is not None else None,
      )
      harsh_flags.extend(distance_flags)
      leapfrog_flags = classify_pred_leapfrog(pred_rebound, pred_unexpected_accel, args)

      event_score = score_event_metrics(
        pred_jerk,
        pred_min_a,
        pred_rollout,
        args.max_pred_rollout_m,
        pred_lead_hold_m=pred_lead_hold,
        recorded_lead_hold_m=float(recorded_lead_hold) if recorded_lead_hold is not None else None,
        min_lead_hold_m=args.min_pred_lead_hold_distance_m,
        max_lead_hold_m=args.max_pred_lead_hold_distance_m,
        pred_cmd_jerk=pred_cmd_jerk,
        max_cmd_jerk=args.max_pred_end_cmd_jerk,
        pred_accel_step=pred_accel_step,
        max_accel_step=args.max_pred_end_accel_step,
      )
      rows.append({
        "summary_json": str(summary_path),
        "route": route,
        "event_id": event.get("event_id"),
        "event_source": source,
        "entry_speed_mps": entry_speed,
        "command_source": args.command_source,
        "enabled_ratio": replay_enabled_ratio if args.command_source == "controller" else None,
        "controller_should_stop_source": args.controller_should_stop_source if args.command_source == "controller" else None,
        "pred_lead_distance_stop_entry_m": pred_lead_entry,
        "pred_lead_distance_hold_m": pred_lead_hold,
        "recorded_lead_distance_hold_m": recorded_lead_hold,
        "pred_entry_stop_jerk_mps3": pred_entry_jerk,
        "pred_entry_stop_accel_step_mps2": pred_entry_accel_step,
        "pred_entry_stop_cmd_jerk_mps3": pred_entry_cmd_jerk,
        "pred_entry_stop_cmd_step_mps2": pred_entry_cmd_step,
        "pred_end_stop_jerk_mps3": pred_jerk,
        "pred_end_stop_cmd_jerk_mps3": pred_cmd_jerk,
        "pred_end_stop_accel_step_mps2": pred_accel_step,
        "pred_min_a_ego_mps2": pred_min_a,
        "pred_rollout_distance_m": pred_rollout,
        "pred_rollout_total_distance_m": pred_rollout_total,
        "distance_gate_source": distance_gate_source,
        "distance_metric_value_m": distance_metric_value,
        "pred_speed_rebound_while_should_stop_mps": pred_rebound,
        "pred_should_stop_unexpected_accel_mps2": pred_unexpected_accel,
        "event_score": event_score,
        "is_harsh": bool(harsh_flags),
        "is_leapfrog": bool(leapfrog_flags),
        "flags": harsh_flags,
        "leapfrog_flags": leapfrog_flags,
      })

  gate_rows = rows

  status = "pass"
  reasons: list[str] = []
  if len(gate_rows) < args.min_events:
    status = "insufficient_events"
    reasons.append(f"events={len(gate_rows)} < min_events={args.min_events}")
  else:
    harsh_rate = sum(1 for row in gate_rows if row["is_harsh"]) / max(len(gate_rows), 1)
    leapfrog_rate = sum(1 for row in gate_rows if row.get("is_leapfrog")) / max(len(gate_rows), 1)
    leapfrog_count = sum(1 for row in gate_rows if row.get("is_leapfrog"))
    if harsh_rate > args.max_harsh_rate:
      status = "fail"
      reasons.append(f"harsh_rate={harsh_rate:.3f} > max_harsh_rate={args.max_harsh_rate:.3f}")
    if leapfrog_rate > args.max_leapfrog_rate:
      status = "fail"
      reasons.append(f"leapfrog_rate={leapfrog_rate:.3f} > max_leapfrog_rate={args.max_leapfrog_rate:.3f}")
    if args.max_leapfrog_count > 0 and leapfrog_count > args.max_leapfrog_count:
      status = "fail"
      reasons.append(f"leapfrog_count={leapfrog_count} > max_leapfrog_count={args.max_leapfrog_count}")

  result = build_result(status=status, reasons=reasons, event_rows=gate_rows, args=args)

  print(f"[model-harsh-check] status={status}")
  print(f"[model-harsh-check] events_considered={result['events_considered']}")
  print(f"[model-harsh-check] harsh_events={result['harsh_events']}")
  print(f"[model-harsh-check] harsh_rate={result['harsh_rate']:.3f}")
  print(f"[model-harsh-check] leapfrog_events={result['leapfrog_events']}")
  print(f"[model-harsh-check] leapfrog_rate={result['leapfrog_rate']:.3f}")
  print(f"[model-harsh-check] avg_event_score={result['avg_event_score']:.3f}")
  if reasons:
    print(f"[model-harsh-check] reasons={'; '.join(reasons)}")

  for idx, row in enumerate([item for item in gate_rows if item["is_harsh"]][:5], start=1):
    distance_fragment = (
      f" predLeadHold={row.get('pred_lead_distance_hold_m')}"
      if row.get("distance_gate_source") == "lead_hold"
      else f" predRollout={row.get('pred_rollout_distance_m')}"
    )
    message = (
      f"[model-harsh-check] harsh_sample#{idx} route={row['route']} event={row['event_id']} entry={row['entry_speed_mps']:.2f}"
      + f" predJerk={row['pred_end_stop_jerk_mps3']} predCmdJerk={row.get('pred_end_stop_cmd_jerk_mps3')}"
      + f" predStep={row.get('pred_end_stop_accel_step_mps2')} predMinA={row['pred_min_a_ego_mps2']}"
      + distance_fragment
      + f" score={row.get('event_score')} flags={','.join(row['flags'])}"
    )
    print(message)
  for idx, row in enumerate([item for item in gate_rows if item.get("is_leapfrog")][:5], start=1):
    message = (
      f"[model-harsh-check] leapfrog_sample#{idx} route={row['route']} event={row['event_id']} entry={row['entry_speed_mps']:.2f}"
      + f" predRebound={row.get('pred_speed_rebound_while_should_stop_mps')}"
      + f" predUnexpectedA={row.get('pred_should_stop_unexpected_accel_mps2')}"
      + f" flags={','.join(row.get('leapfrog_flags', []))}"
    )
    print(message)

  if args.output_json:
    output_path = Path(args.output_json).expanduser()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n")
    print(f"[model-harsh-check] output_json={output_path}")

  if status == "pass":
    return 0
  if status == "insufficient_events":
    return 2
  return 1


if __name__ == "__main__":
  raise SystemExit(main())
