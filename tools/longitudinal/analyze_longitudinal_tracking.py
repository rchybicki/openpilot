#!/usr/bin/env python3
"""Analyze longitudinal requested-vs-actual acceleration on locally synced qlogs."""

from __future__ import annotations

import argparse
import bz2
import json
import shutil
import subprocess
import sys
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
  sys.path.insert(0, str(REPO_ROOT))

from cereal import log as capnp_log
from openpilot.tools.route_sync.common import DEFAULT_DOWNLOAD_ROOT
from openpilot.tools.stopping.log_schema_helpers import controls_state_enabled, selfdrive_state_engaged

DEFAULT_ANALYSIS_ROOT = Path.home() / ".comma" / "longitudinal_tuning" / "analysis"
QLOG_FILE_PATTERNS = ("qlog", "qlog.bz2", "qlog.zst")
ZSTD_MAGIC = b"\x28\xb5\x2f\xfd"
REQUEST_SIGNAL_LABELS = {
  "a_target": "longitudinalPlan.aTarget",
  "accel_cmd": "carControl.actuators.accel",
}


@dataclass
class SegmentFile:
  route: str
  segment: int
  path: Path
  mtime: float


@dataclass
class TrackingMetrics:
  request_signal: str
  regime: str
  sample_count: int
  duration_s: float
  best_delay_s: float
  rmse_mps2: float
  mae_mps2: float
  bias_mps2: float
  p95_abs_error_mps2: float
  corr: float | None
  mean_request_mps2: float
  mean_actual_mps2: float


@dataclass
class ErrorWindow:
  route: str
  start_segment: int
  end_segment: int
  start_time_s: float
  end_time_s: float
  duration_s: float
  regime: str
  classification: str
  mean_speed_mps: float
  mean_request_mps2: float
  mean_actual_mps2: float
  mean_error_mps2: float
  peak_abs_error_mps2: float
  score: float


@dataclass
class RouteSummary:
  route: str
  car_fingerprint: str | None
  git_commit: str | None
  segment_count: int
  duration_s: float
  engaged_duration_s: float
  pedal_override_duration_s: float
  metrics: list[TrackingMetrics]
  worst_windows: list[ErrorWindow]


def utc_now_iso() -> str:
  return datetime.now(timezone.utc).replace(microsecond=0).isoformat()


def short_exception(exc: Exception) -> str:
  text = str(exc).strip()
  return text.splitlines()[0] if text else exc.__class__.__name__


def read_bool_attr(message: object, *names: str) -> bool | None:
  for name in names:
    try:
      value = getattr(message, name)
    except AttributeError:
      continue
    except Exception:
      continue
    try:
      return bool(value)
    except Exception:
      continue
  return None


def safe_float_attr(message: object, name: str) -> float | None:
  try:
    value = getattr(message, name)
  except AttributeError:
    return None
  except Exception:
    return None

  try:
    return float(value)
  except Exception:
    return None


def parse_args() -> argparse.Namespace:
  parser = argparse.ArgumentParser(description="Analyze requested-vs-actual longitudinal acceleration from locally synced qlogs")
  parser.add_argument("--host", required=True, help="Host subfolder under download root, e.g. comma")
  parser.add_argument("--route", action="append", default=[], help="Explicit route ID to analyze (repeatable)")
  parser.add_argument("--download-root", default=str(DEFAULT_DOWNLOAD_ROOT), help=f"Local download root. Default: {DEFAULT_DOWNLOAD_ROOT}")
  parser.add_argument("--analysis-root", default=str(DEFAULT_ANALYSIS_ROOT), help=f"Output root for analysis artifacts. Default: {DEFAULT_ANALYSIS_ROOT}")
  parser.add_argument("--max-routes", type=int, default=2, help="If no explicit routes are passed, analyze this many newest routes")
  parser.add_argument("--min-route-segments", type=int, default=3, help="Minimum locally available segments for auto-selected routes")
  parser.add_argument("--max-segments", type=int, default=0, help="Limit newest segments per route (0 = all)")
  parser.add_argument("--min-speed", type=float, default=0.3, help="Minimum vEgo to score tracking")
  parser.add_argument("--dynamic-threshold", type=float, default=0.15, help="Minimum |requested accel| to treat as dynamic tracking")
  parser.add_argument("--delay-max", type=float, default=0.8, help="Maximum delay to scan in seconds")
  parser.add_argument("--delay-step", type=float, default=0.05, help="Delay scan step in seconds")
  parser.add_argument("--request-max-age", type=float, default=0.5, help="Max age for sampled request values in seconds")
  parser.add_argument("--window-threshold", type=float, default=0.35, help="Absolute error threshold for mismatch windows")
  parser.add_argument("--window-min-duration", type=float, default=0.75, help="Minimum duration for a mismatch window")
  parser.add_argument("--max-windows", type=int, default=8, help="Maximum mismatch windows to keep per route")
  parser.add_argument("--output-dir", default=None, help="Explicit output directory path")
  return parser.parse_args()


def qlog_path_priority(path: Path) -> int:
  if path.name == "qlog":
    return 0
  if path.name == "qlog.bz2":
    return 1
  if path.name == "qlog.zst":
    return 2
  return 99


def route_prefix_key(route: str) -> tuple[int, int]:
  prefix = route.split("--", 1)[0] if "--" in route else route
  try:
    return 1, int(prefix, 16)
  except ValueError:
    return 0, 0


def iter_qlog_files(download_root: Path, host: str) -> list[SegmentFile]:
  host_root = (download_root / host).expanduser()
  if not host_root.exists():
    raise FileNotFoundError(f"Host download directory not found: {host_root}")

  segments_by_key: dict[tuple[str, int], SegmentFile] = {}
  for pattern in QLOG_FILE_PATTERNS:
    for qlog_path in host_root.rglob(pattern):
      segment_name = qlog_path.parent.name
      if "--" not in segment_name:
        continue
      route, suffix = segment_name.rsplit("--", 1)
      try:
        segment = int(suffix)
      except ValueError:
        continue
      try:
        mtime = qlog_path.stat().st_mtime
      except OSError:
        continue

      segment_file = SegmentFile(route=route, segment=segment, path=qlog_path, mtime=mtime)
      key = (route, segment)
      existing = segments_by_key.get(key)
      if existing is None or qlog_path_priority(qlog_path) < qlog_path_priority(existing.path):
        segments_by_key[key] = segment_file

  if not segments_by_key:
    raise RuntimeError(f"No qlog files found under {host_root}")
  return list(segments_by_key.values())


def pick_routes(segments: list[SegmentFile], route_overrides: list[str], max_routes: int, min_route_segments: int) -> list[str]:
  grouped: dict[str, list[SegmentFile]] = {}
  for segment in segments:
    grouped.setdefault(segment.route, []).append(segment)

  if route_overrides:
    missing = [route for route in route_overrides if route not in grouped]
    if missing:
      raise RuntimeError(f"Requested routes not found locally: {', '.join(missing)}")
    return route_overrides

  candidates: list[tuple[str, int, float, bool]] = []
  for route, route_segments in grouped.items():
    newest_mtime = max(segment.mtime for segment in route_segments)
    segment_count = len(route_segments)
    has_segment_zero = any(segment.segment == 0 for segment in route_segments)
    if segment_count < min_route_segments or not has_segment_zero:
      continue
    candidates.append((route, segment_count, newest_mtime, has_segment_zero))

  if not candidates:
    raise RuntimeError("No routes met the auto-selection requirements")

  candidates.sort(key=lambda item: (route_prefix_key(item[0]), item[2], item[1], item[0]), reverse=True)
  return [route for route, _, _, _ in candidates[:max_routes]]


def read_log_bytes(path: Path) -> bytes:
  data = path.read_bytes()
  if path.suffix == ".bz2" or data.startswith(b"BZh9"):
    return bz2.decompress(data)
  if path.suffix == ".zst" or data.startswith(ZSTD_MAGIC):
    zstd = shutil.which("zstd")
    if not zstd:
      raise RuntimeError("zstd command not found for .zst qlog decode")
    result = subprocess.run([zstd, "-d", "-q", "-c", str(path)], capture_output=True, check=False)
    if result.returncode != 0:
      stderr = result.stderr.decode("utf-8", errors="ignore").strip() if result.stderr else "unknown zstd error"
      raise RuntimeError(stderr or "unknown zstd error")
    return result.stdout
  return data


def read_events(path: Path):
  try:
    data = read_log_bytes(path)
  except Exception as exc:
    print(f"[longitudinal-analysis] warning: failed to read {path}: {short_exception(exc)}", file=sys.stderr)
    return

  try:
    reader = capnp_log.Event.read_multiple_bytes(data)
  except Exception as exc:
    print(f"[longitudinal-analysis] warning: failed to decode {path}: {short_exception(exc)}", file=sys.stderr)
    return

  while True:
    try:
      yield next(reader)
    except StopIteration:
      break
    except Exception as exc:
      print(f"[longitudinal-analysis] warning: truncated/corrupt events in {path}: {short_exception(exc)}", file=sys.stderr)
      break


def step_sample(times: np.ndarray, values: np.ndarray, query_times: np.ndarray, max_age_s: float, default_value: float = np.nan) -> np.ndarray:
  output = np.full(query_times.shape, default_value, dtype=float)
  if times.size == 0 or values.size == 0 or query_times.size == 0:
    return output

  indices = np.searchsorted(times, query_times, side="right") - 1
  valid = indices >= 0
  if not np.any(valid):
    return output

  valid_indices = indices[valid]
  valid_query = query_times[valid]
  ages = valid_query - times[valid_indices]
  recent = ages <= max_age_s
  if not np.any(recent):
    return output

  out_indices = np.flatnonzero(valid)[recent]
  output[out_indices] = values[valid_indices[recent]]
  return output


def duration_from_mask(times: np.ndarray, mask: np.ndarray) -> float:
  if times.size == 0 or not np.any(mask):
    return 0.0
  if times.size == 1:
    return 0.0
  dt = np.diff(times, append=times[-1])
  if dt.size >= 2:
    dt[-1] = float(np.median(dt[:-1]))
  return float(np.sum(dt[mask]))


def compute_metrics(
  sample_times: np.ndarray,
  actual: np.ndarray,
  request_times: np.ndarray,
  request_values: np.ndarray,
  base_mask: np.ndarray,
  regime: str,
  dynamic_threshold: float,
  delay_max: float,
  delay_step: float,
  request_max_age: float,
  request_signal: str,
) -> TrackingMetrics | None:
  if request_times.size == 0 or request_values.size == 0:
    return None

  delays = np.arange(0.0, delay_max + 1e-9, delay_step)
  best_delay: float | None = None
  best_rmse = float("inf")
  best_mask: np.ndarray | None = None
  best_requested: np.ndarray | None = None

  for delay_s in delays:
    delayed_request = step_sample(request_times, request_values, sample_times - delay_s, request_max_age)
    mask = base_mask & np.isfinite(actual) & np.isfinite(delayed_request)
    if regime == "accel":
      mask &= delayed_request >= dynamic_threshold
    elif regime == "brake":
      mask &= delayed_request <= -dynamic_threshold
    else:
      mask &= np.abs(delayed_request) >= dynamic_threshold

    if int(np.sum(mask)) < 20:
      continue

    error = actual[mask] - delayed_request[mask]
    rmse = float(np.sqrt(np.mean(np.square(error))))
    if rmse < best_rmse:
      best_rmse = rmse
      best_delay = float(delay_s)
      best_mask = mask
      best_requested = delayed_request

  if best_delay is None or best_mask is None or best_requested is None:
    return None

  error = actual[best_mask] - best_requested[best_mask]
  corr: float | None = None
  if np.std(best_requested[best_mask]) > 1e-6 and np.std(actual[best_mask]) > 1e-6:
    corr = float(np.corrcoef(best_requested[best_mask], actual[best_mask])[0, 1])

  return TrackingMetrics(
    request_signal=request_signal,
    regime=regime,
    sample_count=int(np.sum(best_mask)),
    duration_s=duration_from_mask(sample_times, best_mask),
    best_delay_s=best_delay,
    rmse_mps2=best_rmse,
    mae_mps2=float(np.mean(np.abs(error))),
    bias_mps2=float(np.mean(error)),
    p95_abs_error_mps2=float(np.percentile(np.abs(error), 95)),
    corr=corr,
    mean_request_mps2=float(np.mean(best_requested[best_mask])),
    mean_actual_mps2=float(np.mean(actual[best_mask])),
  )


def classify_window(mean_request: float, mean_error: float) -> str:
  if mean_request < 0.0:
    return "brake under-response" if mean_error > 0.0 else "brake overshoot"
  return "accel under-response" if mean_error < 0.0 else "accel overshoot"


def build_error_windows(
  route: str,
  sample_times: np.ndarray,
  sample_segments: np.ndarray,
  speeds: np.ndarray,
  actual: np.ndarray,
  request_times: np.ndarray,
  request_values: np.ndarray,
  base_mask: np.ndarray,
  best_delay_s: float,
  request_max_age: float,
  dynamic_threshold: float,
  error_threshold: float,
  min_duration_s: float,
  max_windows: int,
) -> list[ErrorWindow]:
  delayed_request = step_sample(request_times, request_values, sample_times - best_delay_s, request_max_age)
  valid = base_mask & np.isfinite(actual) & np.isfinite(delayed_request) & (np.abs(delayed_request) >= dynamic_threshold)
  if not np.any(valid):
    return []

  error = actual - delayed_request
  hot = valid & (np.abs(error) >= error_threshold)
  windows: list[ErrorWindow] = []
  start_index: int | None = None

  for index, is_hot in enumerate(hot):
    if is_hot and start_index is None:
      start_index = index
      continue
    if is_hot or start_index is None:
      continue

    end_index = index - 1
    duration_s = float(sample_times[end_index] - sample_times[start_index]) if end_index > start_index else 0.0
    if duration_s >= min_duration_s:
      window_slice = slice(start_index, end_index + 1)
      mean_request = float(np.mean(delayed_request[window_slice]))
      mean_actual = float(np.mean(actual[window_slice]))
      mean_error = float(np.mean(error[window_slice]))
      peak_abs_error = float(np.max(np.abs(error[window_slice])))
      score = peak_abs_error * max(duration_s, 1e-3)
      windows.append(ErrorWindow(
        route=route,
        start_segment=int(sample_segments[start_index]),
        end_segment=int(sample_segments[end_index]),
        start_time_s=float(sample_times[start_index]),
        end_time_s=float(sample_times[end_index]),
        duration_s=duration_s,
        regime="brake" if mean_request < 0.0 else "accel",
        classification=classify_window(mean_request, mean_error),
        mean_speed_mps=float(np.mean(speeds[window_slice])),
        mean_request_mps2=mean_request,
        mean_actual_mps2=mean_actual,
        mean_error_mps2=mean_error,
        peak_abs_error_mps2=peak_abs_error,
        score=score,
      ))
    start_index = None

  if start_index is not None:
    end_index = len(hot) - 1
    duration_s = float(sample_times[end_index] - sample_times[start_index]) if end_index > start_index else 0.0
    if duration_s >= min_duration_s:
      window_slice = slice(start_index, end_index + 1)
      mean_request = float(np.mean(delayed_request[window_slice]))
      mean_actual = float(np.mean(actual[window_slice]))
      mean_error = float(np.mean(error[window_slice]))
      peak_abs_error = float(np.max(np.abs(error[window_slice])))
      score = peak_abs_error * max(duration_s, 1e-3)
      windows.append(ErrorWindow(
        route=route,
        start_segment=int(sample_segments[start_index]),
        end_segment=int(sample_segments[end_index]),
        start_time_s=float(sample_times[start_index]),
        end_time_s=float(sample_times[end_index]),
        duration_s=duration_s,
        regime="brake" if mean_request < 0.0 else "accel",
        classification=classify_window(mean_request, mean_error),
        mean_speed_mps=float(np.mean(speeds[window_slice])),
        mean_request_mps2=mean_request,
        mean_actual_mps2=mean_actual,
        mean_error_mps2=mean_error,
        peak_abs_error_mps2=peak_abs_error,
        score=score,
      ))

  windows.sort(key=lambda window: window.score, reverse=True)
  return windows[:max_windows]


def load_route_summary(route: str, route_segments: list[SegmentFile], args: argparse.Namespace) -> RouteSummary:
  ordered_segments = sorted(route_segments, key=lambda segment: segment.segment)
  if args.max_segments > 0:
    ordered_segments = ordered_segments[-args.max_segments:]

  first_mono_time: float | None = None
  car_fingerprint: str | None = None
  git_commit: str | None = None

  plan_times: list[float] = []
  a_target_values: list[float] = []
  control_times: list[float] = []
  accel_cmd_values: list[float] = []
  long_active_values: list[int] = []
  controls_state_times: list[float] = []
  controls_enabled_values: list[int] = []
  selfdrive_state_times: list[float] = []
  selfdrive_enabled_values: list[int] = []
  state_times: list[float] = []
  state_segments: list[int] = []
  a_ego_values: list[float] = []
  v_ego_values: list[float] = []
  gas_pressed_values: list[int] = []
  brake_pressed_values: list[int] = []
  standstill_values: list[int] = []
  live_pose_times: list[float] = []
  accel_device_values: list[float] = []

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
      elif which == "longitudinalPlan":
        a_target = safe_float_attr(msg.longitudinalPlan, "aTarget")
        if a_target is not None:
          plan_times.append(t_rel)
          a_target_values.append(a_target)
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
      elif which == "carState":
        state_times.append(t_rel)
        state_segments.append(segment.segment)
        a_ego_values.append(float(msg.carState.aEgo))
        v_ego_values.append(float(msg.carState.vEgo))
        gas_pressed_values.append(1 if bool(msg.carState.gasPressed) else 0)
        brake_pressed_values.append(1 if bool(msg.carState.brakePressed) else 0)
        standstill_values.append(1 if bool(msg.carState.standstill) else 0)
      elif which == "livePose":
        try:
          accel_x = float(msg.livePose.accelerationDevice.x)
        except Exception:
          continue
        live_pose_times.append(t_rel)
        accel_device_values.append(accel_x)

  if not state_times:
    raise RuntimeError(f"Route {route} has no carState samples")

  times = np.asarray(state_times, dtype=float)
  segments = np.asarray(state_segments, dtype=int)
  a_ego = np.asarray(a_ego_values, dtype=float)
  v_ego = np.asarray(v_ego_values, dtype=float)
  gas_pressed = np.asarray(gas_pressed_values, dtype=bool)
  brake_pressed = np.asarray(brake_pressed_values, dtype=bool)
  standstill = np.asarray(standstill_values, dtype=bool)
  plan_times_np = np.asarray(plan_times, dtype=float)
  a_target_np = np.asarray(a_target_values, dtype=float)
  control_times_np = np.asarray(control_times, dtype=float)
  accel_cmd_np = np.asarray(accel_cmd_values, dtype=float)
  long_active_np = np.asarray(long_active_values, dtype=float)
  controls_state_times_np = np.asarray(controls_state_times, dtype=float)
  controls_enabled_np = np.asarray(controls_enabled_values, dtype=float)
  selfdrive_state_times_np = np.asarray(selfdrive_state_times, dtype=float)
  selfdrive_enabled_np = np.asarray(selfdrive_enabled_values, dtype=float)

  long_active_state = step_sample(control_times_np, long_active_np, times, max_age_s=0.5, default_value=0.0) > 0.5
  controls_enabled_state = step_sample(controls_state_times_np, controls_enabled_np, times, max_age_s=0.5, default_value=0.0) > 0.5
  selfdrive_enabled_state = step_sample(selfdrive_state_times_np, selfdrive_enabled_np, times, max_age_s=0.5, default_value=0.0) > 0.5
  engaged = long_active_state | controls_enabled_state | selfdrive_enabled_state
  pedal_override = gas_pressed | brake_pressed

  live_pose_agreement: TrackingMetrics | None = None
  if len(live_pose_times) >= 10:
    live_pose_times_np = np.asarray(live_pose_times, dtype=float)
    accel_device_np = np.asarray(accel_device_values, dtype=float)
    accel_device_on_state = np.interp(times, live_pose_times_np, accel_device_np)
    actual_mask = engaged & ~pedal_override & ~standstill & (v_ego >= args.min_speed)
    if int(np.sum(actual_mask)) >= 20:
      error = accel_device_on_state[actual_mask] - a_ego[actual_mask]
      corr: float | None = None
      if np.std(accel_device_on_state[actual_mask]) > 1e-6 and np.std(a_ego[actual_mask]) > 1e-6:
        corr = float(np.corrcoef(accel_device_on_state[actual_mask], a_ego[actual_mask])[0, 1])
      live_pose_agreement = TrackingMetrics(
        request_signal="accel_device_vs_a_ego",
        regime="all",
        sample_count=int(np.sum(actual_mask)),
        duration_s=duration_from_mask(times, actual_mask),
        best_delay_s=0.0,
        rmse_mps2=float(np.sqrt(np.mean(np.square(error)))),
        mae_mps2=float(np.mean(np.abs(error))),
        bias_mps2=float(np.mean(error)),
        p95_abs_error_mps2=float(np.percentile(np.abs(error), 95)),
        corr=corr,
        mean_request_mps2=float(np.mean(a_ego[actual_mask])),
        mean_actual_mps2=float(np.mean(accel_device_on_state[actual_mask])),
      )

  base_mask = engaged & ~pedal_override & ~standstill & (v_ego >= args.min_speed)
  metrics: list[TrackingMetrics] = []
  request_signals = {
    "a_target": (plan_times_np, a_target_np),
    "accel_cmd": (control_times_np, accel_cmd_np),
  }
  metrics_by_signal: dict[str, dict[str, TrackingMetrics]] = {}
  for signal_name, (request_times, request_values) in request_signals.items():
    metrics_by_signal[signal_name] = {}
    for regime in ("all", "accel", "brake"):
      metric = compute_metrics(
        sample_times=times,
        actual=a_ego,
        request_times=request_times,
        request_values=request_values,
        base_mask=base_mask,
        regime=regime,
        dynamic_threshold=args.dynamic_threshold,
        delay_max=args.delay_max,
        delay_step=args.delay_step,
        request_max_age=args.request_max_age,
        request_signal=signal_name,
      )
      if metric is not None:
        metrics.append(metric)
        metrics_by_signal[signal_name][regime] = metric

  if live_pose_agreement is not None:
    metrics.append(live_pose_agreement)

  worst_windows: list[ErrorWindow] = []
  accel_cmd_overall = metrics_by_signal.get("accel_cmd", {}).get("all")
  if accel_cmd_overall is not None:
    worst_windows = build_error_windows(
      route=route,
      sample_times=times,
      sample_segments=segments,
      speeds=v_ego,
      actual=a_ego,
      request_times=control_times_np,
      request_values=accel_cmd_np,
      base_mask=base_mask,
      best_delay_s=accel_cmd_overall.best_delay_s,
      request_max_age=args.request_max_age,
      dynamic_threshold=args.dynamic_threshold,
      error_threshold=args.window_threshold,
      min_duration_s=args.window_min_duration,
      max_windows=args.max_windows,
    )

  return RouteSummary(
    route=route,
    car_fingerprint=car_fingerprint,
    git_commit=git_commit,
    segment_count=len(ordered_segments),
    duration_s=float(times[-1] - times[0]) if len(times) > 1 else 0.0,
    engaged_duration_s=duration_from_mask(times, engaged),
    pedal_override_duration_s=duration_from_mask(times, pedal_override),
    metrics=metrics,
    worst_windows=worst_windows,
  )


def build_table(headers: list[str], rows: list[list[str]]) -> str:
  widths = [len(header) for header in headers]
  for row in rows:
    for index, cell in enumerate(row):
      widths[index] = max(widths[index], len(cell))

  def format_row(row: list[str]) -> str:
    return "| " + " | ".join(cell.ljust(widths[index]) for index, cell in enumerate(row)) + " |"

  separator = "| " + " | ".join("-" * width for width in widths) + " |"
  return "\n".join([format_row(headers), separator, *(format_row(row) for row in rows)])


def render_summary(summary: dict[str, Any], output_dir: Path) -> str:
  lines: list[str] = []
  lines.append("# Longitudinal Tracking Summary")
  lines.append("")
  lines.append(f"- Generated: `{summary['generated_at_utc']}`")
  lines.append(f"- Host: `{summary['host']}`")
  lines.append(f"- Car fingerprint(s): `{', '.join(summary['car_fingerprints'])}`")
  lines.append(f"- Routes analyzed: `{', '.join(summary['routes'])}`")
  lines.append(f"- JSON: `{output_dir / 'summary.json'}`")
  lines.append("")
  lines.append("## Route Coverage")
  lines.append("")
  route_rows: list[list[str]] = []
  for route_summary in summary["route_summaries"]:
    route_rows.append([
      route_summary["route"],
      str(route_summary["segment_count"]),
      f"{route_summary['duration_s']:.1f}",
      f"{route_summary['engaged_duration_s']:.1f}",
      route_summary["car_fingerprint"] or "unknown",
    ])
  lines.append(build_table(
    ["route", "segments", "duration_s", "engaged_s", "car_fingerprint"],
    route_rows,
  ))
  lines.append("")

  aggregate_metrics = summary["aggregate_metrics"]
  metric_rows: list[list[str]] = []
  for metric in aggregate_metrics:
    signal_label = REQUEST_SIGNAL_LABELS.get(metric["request_signal"], metric["request_signal"])
    metric_rows.append([
      signal_label,
      metric["regime"],
      str(metric["sample_count"]),
      f"{metric['duration_s']:.1f}",
      f"{metric['best_delay_s']:.2f}",
      f"{metric['rmse_mps2']:.3f}",
      f"{metric['bias_mps2']:.3f}",
      f"{metric['mae_mps2']:.3f}",
      f"{metric['p95_abs_error_mps2']:.3f}",
      f"{metric['corr']:.3f}" if metric["corr"] is not None else "n/a",
    ])
  if metric_rows:
    lines.append("## Aggregate Tracking Metrics")
    lines.append("")
    lines.append(build_table(
      ["signal", "regime", "samples", "seconds", "delay_s", "rmse", "bias", "mae", "p95_abs", "corr"],
      metric_rows,
    ))
    lines.append("")

  agreement_metric = summary.get("actual_signal_agreement")
  if agreement_metric is not None:
    lines.append("## Actual-Signal Agreement")
    lines.append("")
    lines.append(build_table(
      ["comparison", "samples", "seconds", "rmse", "bias", "p95_abs", "corr"],
      [[
        "livePose.accelerationDevice.x vs carState.aEgo",
        str(agreement_metric["sample_count"]),
        f"{agreement_metric['duration_s']:.1f}",
        f"{agreement_metric['rmse_mps2']:.3f}",
        f"{agreement_metric['bias_mps2']:.3f}",
        f"{agreement_metric['p95_abs_error_mps2']:.3f}",
        f"{agreement_metric['corr']:.3f}" if agreement_metric["corr"] is not None else "n/a",
      ]],
    ))
    lines.append("")

  route_metric_rows: list[list[str]] = []
  for route_summary in summary["route_summaries"]:
    accel_cmd_all = next((metric for metric in route_summary["metrics"] if metric["request_signal"] == "accel_cmd" and metric["regime"] == "all"), None)
    a_target_all = next((metric for metric in route_summary["metrics"] if metric["request_signal"] == "a_target" and metric["regime"] == "all"), None)
    route_metric_rows.append([
      route_summary["route"],
      f"{accel_cmd_all['rmse_mps2']:.3f}" if accel_cmd_all else "n/a",
      f"{accel_cmd_all['bias_mps2']:.3f}" if accel_cmd_all else "n/a",
      f"{accel_cmd_all['best_delay_s']:.2f}" if accel_cmd_all else "n/a",
      f"{a_target_all['rmse_mps2']:.3f}" if a_target_all else "n/a",
      f"{a_target_all['bias_mps2']:.3f}" if a_target_all else "n/a",
    ])
  if route_metric_rows:
    lines.append("## Route Ranking")
    lines.append("")
    lines.append(build_table(
      ["route", "cmd_rmse", "cmd_bias", "cmd_delay_s", "plan_rmse", "plan_bias"],
      route_metric_rows,
    ))
    lines.append("")

  all_windows: list[dict[str, Any]] = []
  for route_summary in summary["route_summaries"]:
    all_windows.extend(route_summary["worst_windows"])
  all_windows.sort(key=lambda window: window["score"], reverse=True)
  if all_windows:
    lines.append("## Largest Tracking Mismatches")
    lines.append("")
    window_rows: list[list[str]] = []
    for window in all_windows[:12]:
      window_rows.append([
        window["route"],
        f"{window['start_segment']}->{window['end_segment']}",
        f"{window['start_time_s']:.1f}",
        f"{window['duration_s']:.2f}",
        window["classification"],
        f"{window['mean_speed_mps']:.2f}",
        f"{window['mean_request_mps2']:.2f}",
        f"{window['mean_actual_mps2']:.2f}",
        f"{window['mean_error_mps2']:.2f}",
        f"{window['peak_abs_error_mps2']:.2f}",
      ])
    lines.append(build_table(
      ["route", "segments", "start_s", "dur_s", "class", "speed", "request", "actual", "error", "peak_abs"],
      window_rows,
    ))
    lines.append("")

  return "\n".join(lines).rstrip() + "\n"


def main() -> int:
  args = parse_args()
  download_root = Path(args.download_root).expanduser()
  analysis_root = Path(args.analysis_root).expanduser()

  segments = iter_qlog_files(download_root, args.host)
  routes = pick_routes(segments, args.route, args.max_routes, args.min_route_segments)
  grouped_segments: dict[str, list[SegmentFile]] = {}
  for segment in segments:
    grouped_segments.setdefault(segment.route, []).append(segment)

  route_summaries: list[RouteSummary] = []
  for route in routes:
    summary = load_route_summary(route, grouped_segments[route], args)
    route_summaries.append(summary)

  if args.output_dir:
    output_dir = Path(args.output_dir).expanduser()
  else:
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    output_dir = analysis_root / args.host / stamp
  output_dir.mkdir(parents=True, exist_ok=True)

  car_fingerprints = sorted({summary.car_fingerprint for summary in route_summaries if summary.car_fingerprint})
  aggregate_metrics: list[TrackingMetrics] = []
  for request_signal in ("a_target", "accel_cmd"):
    for regime in ("all", "accel", "brake"):
      values = [
        metric for summary in route_summaries for metric in summary.metrics
        if metric.request_signal == request_signal and metric.regime == regime
      ]
      if not values:
        continue
      weight = np.asarray([metric.duration_s for metric in values], dtype=float)
      if np.sum(weight) <= 0:
        weight = np.ones_like(weight)
      rmse = float(np.sqrt(np.average(np.square([metric.rmse_mps2 for metric in values]), weights=weight)))
      mae = float(np.average([metric.mae_mps2 for metric in values], weights=weight))
      bias = float(np.average([metric.bias_mps2 for metric in values], weights=weight))
      p95 = float(np.average([metric.p95_abs_error_mps2 for metric in values], weights=weight))
      delay = float(np.average([metric.best_delay_s for metric in values], weights=weight))
      mean_request = float(np.average([metric.mean_request_mps2 for metric in values], weights=weight))
      mean_actual = float(np.average([metric.mean_actual_mps2 for metric in values], weights=weight))
      corr_values = [metric.corr for metric in values if metric.corr is not None]
      corr = float(np.average(corr_values)) if corr_values else None
      aggregate_metrics.append(TrackingMetrics(
        request_signal=request_signal,
        regime=regime,
        sample_count=int(sum(metric.sample_count for metric in values)),
        duration_s=float(sum(metric.duration_s for metric in values)),
        best_delay_s=delay,
        rmse_mps2=rmse,
        mae_mps2=mae,
        bias_mps2=bias,
        p95_abs_error_mps2=p95,
        corr=corr,
        mean_request_mps2=mean_request,
        mean_actual_mps2=mean_actual,
      ))

  agreement_metrics = [
    metric for summary in route_summaries for metric in summary.metrics
    if metric.request_signal == "accel_device_vs_a_ego"
  ]
  actual_signal_agreement: TrackingMetrics | None = None
  if agreement_metrics:
    weight = np.asarray([metric.duration_s for metric in agreement_metrics], dtype=float)
    if np.sum(weight) <= 0:
      weight = np.ones_like(weight)
    corr_values = [metric.corr for metric in agreement_metrics if metric.corr is not None]
    actual_signal_agreement = TrackingMetrics(
      request_signal="accel_device_vs_a_ego",
      regime="all",
      sample_count=int(sum(metric.sample_count for metric in agreement_metrics)),
      duration_s=float(sum(metric.duration_s for metric in agreement_metrics)),
      best_delay_s=0.0,
      rmse_mps2=float(np.sqrt(np.average(np.square([metric.rmse_mps2 for metric in agreement_metrics]), weights=weight))),
      mae_mps2=float(np.average([metric.mae_mps2 for metric in agreement_metrics], weights=weight)),
      bias_mps2=float(np.average([metric.bias_mps2 for metric in agreement_metrics], weights=weight)),
      p95_abs_error_mps2=float(np.average([metric.p95_abs_error_mps2 for metric in agreement_metrics], weights=weight)),
      corr=float(np.average(corr_values)) if corr_values else None,
      mean_request_mps2=float(np.average([metric.mean_request_mps2 for metric in agreement_metrics], weights=weight)),
      mean_actual_mps2=float(np.average([metric.mean_actual_mps2 for metric in agreement_metrics], weights=weight)),
    )

  summary_data: dict[str, Any] = {
    "generated_at_utc": utc_now_iso(),
    "host": args.host,
    "routes": [summary.route for summary in route_summaries],
    "car_fingerprints": car_fingerprints or ["unknown"],
    "aggregate_metrics": [asdict(metric) for metric in aggregate_metrics],
    "actual_signal_agreement": asdict(actual_signal_agreement) if actual_signal_agreement is not None else None,
    "route_summaries": [
      {
        **asdict(summary),
        "metrics": [asdict(metric) for metric in summary.metrics],
        "worst_windows": [asdict(window) for window in summary.worst_windows],
      }
      for summary in route_summaries
    ],
  }

  summary_json_path = output_dir / "summary.json"
  summary_json_path.write_text(json.dumps(summary_data, indent=2, sort_keys=True) + "\n")

  summary_md_path = output_dir / "summary.md"
  summary_md_path.write_text(render_summary(summary_data, output_dir))

  print(f"[longitudinal-analysis] host={args.host}")
  print(f"[longitudinal-analysis] routes={', '.join(summary_data['routes'])}")
  print(f"[longitudinal-analysis] summary_json={summary_json_path}")
  print(f"[longitudinal-analysis] summary_md={summary_md_path}")
  return 0


if __name__ == "__main__":
  raise SystemExit(main())
