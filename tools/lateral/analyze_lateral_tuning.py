#!/usr/bin/env python3
"""Analyze torque-controller lateral tracking from locally synced qlogs."""

from __future__ import annotations

import argparse
import bz2
import json
import math
import shutil
import subprocess
import sys
from collections import defaultdict
from datetime import UTC, datetime
from pathlib import Path
from statistics import fmean, median
from typing import Any

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
  sys.path.insert(0, str(REPO_ROOT))

from cereal import log as capnp_log
from openpilot.tools.route_sync.common import DEFAULT_DOWNLOAD_ROOT, host_download_root

DEFAULT_ANALYSIS_ROOT = Path.home() / ".comma" / "lateral_tuning" / "analysis"
QLOG_FILE_PATTERNS = ("qlog", "qlog.bz2", "qlog.zst")
ZSTD_MAGIC = b"\x28\xb5\x2f\xfd"
STEER_LIMIT_EPS = 1e-2
ISSUE_WINDOW_MAX_GAP_S = 0.25


def utc_now_iso() -> str:
  return datetime.now(UTC).replace(microsecond=0).isoformat()


def utc_now_stamp() -> str:
  return datetime.now(UTC).strftime("%Y%m%dT%H%M%SZ")


def format_path(path: Path) -> str:
  try:
    resolved = path.expanduser().resolve()
    home = Path.home().resolve()
    return f"~/{resolved.relative_to(home)}"
  except Exception:
    return str(path)


def parse_args() -> argparse.Namespace:
  parser = argparse.ArgumentParser(description="Analyze requested vs actual turning from locally synced qlogs")
  parser.add_argument("--host", required=True, help="Host subfolder under download root, e.g. commawifi")
  parser.add_argument("--route", action="append", default=[], help="Specific route ID to analyze (repeatable)")
  parser.add_argument("--car-fingerprint", default=None,
                      help="Optional exact carFingerprint filter, e.g. HYUNDAI_SANTA_FE_HEV_2022")
  parser.add_argument("--max-routes", type=int, default=3,
                      help="Maximum number of newest matching routes when --route is not used")
  parser.add_argument("--download-root", default=str(DEFAULT_DOWNLOAD_ROOT),
                      help=f"Local download root. Default: {DEFAULT_DOWNLOAD_ROOT}")
  parser.add_argument("--analysis-root", default=str(DEFAULT_ANALYSIS_ROOT),
                      help=f"Output root for analysis artifacts. Default: {DEFAULT_ANALYSIS_ROOT}")
  parser.add_argument("--output-dir", default=None, help="Explicit output directory path")
  parser.add_argument("--min-speed", type=float, default=2.5,
                      help="Minimum speed in m/s for active tracking samples")
  parser.add_argument("--turning-lat-accel", type=float, default=0.8,
                      help="Minimum |desired lateral accel| for turning metrics")
  parser.add_argument("--strong-lat-accel", type=float, default=1.5,
                      help="Minimum |desired lateral accel| for strong-turn metrics")
  parser.add_argument("--issue-min-lat-accel", type=float, default=1.0,
                      help="Minimum |desired lateral accel| for issue-window detection")
  parser.add_argument("--issue-max-ratio", type=float, default=0.8,
                      help="Flag an issue when |actual| / |desired| drops below this ratio")
  parser.add_argument("--issue-min-abs-error", type=float, default=0.4,
                      help="Flag an issue when |actual - desired| exceeds this value")
  parser.add_argument("--top-issues", type=int, default=10, help="Number of issue windows to include in the summary")
  return parser.parse_args()


def qlog_path_priority(path: Path) -> int:
  if path.name == "qlog":
    return 0
  if path.name == "qlog.bz2":
    return 1
  if path.name == "qlog.zst":
    return 2
  return 99


def iter_qlog_files(download_root: Path, host: str) -> list[dict[str, Any]]:
  host_root = host_download_root(download_root, host)
  if not host_root.exists():
    raise FileNotFoundError(f"Host download directory not found: {host_root}")

  segments_by_key: dict[tuple[str, int], dict[str, Any]] = {}
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
      entry = {"route": route, "segment": segment, "path": qlog_path, "mtime": mtime}
      key = (route, segment)
      existing = segments_by_key.get(key)
      if existing is None or qlog_path_priority(qlog_path) < qlog_path_priority(existing["path"]):
        segments_by_key[key] = entry

  if not segments_by_key:
    raise RuntimeError(f"No qlog files found under {host_root}")
  return list(segments_by_key.values())


def route_prefix_key(route: str) -> tuple[int, int]:
  prefix = route.split("--", 1)[0] if "--" in route else route
  try:
    return 1, int(prefix, 16)
  except ValueError:
    return 0, 0


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


def iter_events(path: Path):
  data = read_log_bytes(path)
  yield from capnp_log.Event.read_multiple_bytes(data)


def summarize_values(values: list[float]) -> dict[str, Any] | None:
  if not values:
    return None
  values = sorted(values)
  idx10 = int((len(values) - 1) * 0.10)
  idx90 = int((len(values) - 1) * 0.90)
  return {
    "median": float(median(values)),
    "p10": float(values[idx10]),
    "p90": float(values[idx90]),
    "count": len(values),
  }


def estimate_active_seconds(samples: list[dict[str, Any]]) -> float:
  if not samples:
    return 0.0
  samples = sorted(samples, key=lambda item: item["route_time_s"])
  deltas: list[float] = []
  for current, nxt in zip(samples, samples[1:], strict=False):
    delta = float(nxt["route_time_s"] - current["route_time_s"])
    if 0.0 < delta < 0.3:
      deltas.append(delta)
  nominal_dt = median(deltas) if deltas else 0.1
  return round(len(samples) * nominal_dt, 2)


def compute_error_metrics(samples: list[dict[str, Any]], desired_key: str, actual_key: str) -> dict[str, Any]:
  if not samples:
    return {
      "count": 0,
      "active_seconds_est": 0.0,
      "mean_abs_desired": None,
      "mae": None,
      "rmse": None,
      "p90_abs_error": None,
      "bias": None,
      "median_ratio": None,
      "under_ratio_below_0p8": None,
      "saturation_ratio": None,
      "steer_limited_ratio": None,
      "mean_speed_mps": None,
    }

  errors = [float(sample[actual_key] - sample[desired_key]) for sample in samples]
  abs_errors = [abs(error) for error in errors]
  ratios = [
    abs(float(sample[actual_key])) / abs(float(sample[desired_key]))
    for sample in samples
    if abs(float(sample[desired_key])) >= 0.5
  ]
  under = [
    sample for sample in samples
    if abs(float(sample[desired_key])) > 1.0 and abs(float(sample[actual_key])) / abs(float(sample[desired_key])) < 0.8
  ]
  sorted_abs_errors = sorted(abs_errors)
  idx90 = int((len(sorted_abs_errors) - 1) * 0.90)

  return {
    "count": len(samples),
    "active_seconds_est": estimate_active_seconds(samples),
    "mean_abs_desired": round(fmean(abs(float(sample[desired_key])) for sample in samples), 3),
    "mae": round(fmean(abs_errors), 3),
    "rmse": round(math.sqrt(fmean(error * error for error in errors)), 3),
    "p90_abs_error": round(sorted_abs_errors[idx90], 3),
    "bias": round(fmean(errors), 3),
    "median_ratio": round(median(ratios), 3) if ratios else None,
    "under_ratio_below_0p8": round(len(under) / len(samples), 4),
    "saturation_ratio": round(sum(bool(sample["saturated"]) for sample in samples) / len(samples), 4),
    "steer_limited_ratio": round(sum(bool(sample["steer_limited"]) for sample in samples) / len(samples), 4),
    "mean_speed_mps": round(fmean(float(sample["v_ego_mps"]) for sample in samples), 3),
  }


def build_issue_windows(samples: list[dict[str, Any]], issue_min_lat_accel: float, issue_max_ratio: float,
                        issue_min_abs_error: float, top_issues: int) -> list[dict[str, Any]]:
  issue_samples: list[dict[str, Any]] = []
  for sample in samples:
    desired = abs(float(sample["desired_lateral_accel_controller"]))
    actual = abs(float(sample["actual_lateral_accel_controller"]))
    ratio = actual / desired if desired > 0.5 else 1.0
    abs_error = abs(float(sample["actual_lateral_accel_controller"] - sample["desired_lateral_accel_controller"]))
    if desired < issue_min_lat_accel:
      continue
    if abs_error < issue_min_abs_error and ratio >= issue_max_ratio and not sample["saturated"]:
      continue
    issue_sample = dict(sample)
    issue_sample["tracking_ratio"] = ratio
    issue_sample["abs_error"] = abs_error
    issue_samples.append(issue_sample)

  if not issue_samples:
    return []

  issue_samples.sort(key=lambda item: (item["route"], item["segment"], item["route_time_s"]))
  windows: list[dict[str, Any]] = []
  current: dict[str, Any] | None = None

  for sample in issue_samples:
    key = (sample["route"], sample["segment"])
    if current is None or key != (current["route"], current["segment"]) or sample["route_time_s"] - current["end_route_time_s"] > ISSUE_WINDOW_MAX_GAP_S:
      if current is not None:
        windows.append(current)
      current = {
        "route": sample["route"],
        "segment": sample["segment"],
        "start_route_time_s": sample["route_time_s"],
        "end_route_time_s": sample["route_time_s"],
        "start_segment_time_s": sample["segment_time_s"],
        "end_segment_time_s": sample["segment_time_s"],
        "sample_count": 1,
        "worst_abs_error": sample["abs_error"],
        "worst_ratio": sample["tracking_ratio"],
        "saturated_any": bool(sample["saturated"]),
        "steer_limited_any": bool(sample["steer_limited"]),
        "v_ego_mps": sample["v_ego_mps"],
        "desired_lateral_accel_controller": sample["desired_lateral_accel_controller"],
        "actual_lateral_accel_controller": sample["actual_lateral_accel_controller"],
      }
      continue

    current["end_route_time_s"] = sample["route_time_s"]
    current["end_segment_time_s"] = sample["segment_time_s"]
    current["sample_count"] += 1
    current["saturated_any"] = current["saturated_any"] or bool(sample["saturated"])
    current["steer_limited_any"] = current["steer_limited_any"] or bool(sample["steer_limited"])
    if sample["abs_error"] > current["worst_abs_error"]:
      current["worst_abs_error"] = sample["abs_error"]
      current["worst_ratio"] = sample["tracking_ratio"]
      current["v_ego_mps"] = sample["v_ego_mps"]
      current["desired_lateral_accel_controller"] = sample["desired_lateral_accel_controller"]
      current["actual_lateral_accel_controller"] = sample["actual_lateral_accel_controller"]

  if current is not None:
    windows.append(current)

  windows.sort(key=lambda item: (item["worst_abs_error"], item["sample_count"]), reverse=True)
  return windows[:top_issues]


def probe_route_metadata(segment_files: list[dict[str, Any]]) -> dict[str, Any] | None:
  stock: dict[str, Any] = {}
  for segment in sorted(segment_files, key=lambda item: item["segment"]):
    try:
      for event in iter_events(segment["path"]):
        if event.which() != "carParams":
          continue
        car_params = event.carParams
        stock = {
          "car_fingerprint": car_params.carFingerprint or None,
          "steer_ratio": float(car_params.steerRatio),
          "steer_actuator_delay": float(car_params.steerActuatorDelay),
          "lat_accel_factor": float(car_params.lateralTuning.torque.latAccelFactor),
          "lat_accel_offset": float(car_params.lateralTuning.torque.latAccelOffset),
          "friction": float(car_params.lateralTuning.torque.friction),
        }
        break
    except Exception:
      continue
    if stock:
      break
  return stock or None


def select_routes(segments: list[dict[str, Any]], requested_routes: list[str], requested_car_fingerprint: str | None,
                  max_routes: int) -> tuple[list[str], dict[str, list[dict[str, Any]]], dict[str, dict[str, Any]]]:
  routes_to_segments: dict[str, list[dict[str, Any]]] = defaultdict(list)
  for segment in segments:
    routes_to_segments[segment["route"]].append(segment)

  route_meta_cache: dict[str, dict[str, Any]] = {}

  def get_meta(route: str) -> dict[str, Any]:
    if route not in route_meta_cache:
      route_meta_cache[route] = probe_route_metadata(routes_to_segments[route])
    return route_meta_cache[route]

  if requested_routes:
    missing = [route for route in requested_routes if route not in routes_to_segments]
    if missing:
      raise FileNotFoundError(f"Requested routes not found under host cache: {', '.join(missing)}")
    selected_routes = []
    unreadable: list[str] = []
    for route in dict.fromkeys(requested_routes):
      if get_meta(route) is None:
        unreadable.append(route)
        continue
      selected_routes.append(route)
    if unreadable:
      raise RuntimeError(f"Requested routes missing readable carParams: {', '.join(unreadable)}")
  else:
    ordered_routes = sorted(
      routes_to_segments,
      key=lambda route: (route_prefix_key(route), max(item["mtime"] for item in routes_to_segments[route]), route),
      reverse=True,
    )
    selected_routes = []
    for route in ordered_routes:
      meta = get_meta(route)
      if meta is None:
        continue
      if requested_car_fingerprint and meta["car_fingerprint"] != requested_car_fingerprint:
        continue
      selected_routes.append(route)
      if max_routes and len(selected_routes) >= max_routes:
        break

  if requested_car_fingerprint:
    mismatched = [route for route in selected_routes if get_meta(route)["car_fingerprint"] != requested_car_fingerprint]
    if mismatched:
      raise RuntimeError(
        f"Requested car fingerprint {requested_car_fingerprint} does not match routes: {', '.join(mismatched)}"
      )

  if not selected_routes:
    scope = requested_car_fingerprint or "available routes"
    raise RuntimeError(f"No routes selected for analysis from {scope}")

  selected_segments = {
    route: sorted(routes_to_segments[route], key=lambda item: item["segment"])
    for route in selected_routes
  }
  selected_meta = {route: get_meta(route) for route in selected_routes}
  return selected_routes, selected_segments, selected_meta


def analyze_route(route: str, segment_files: list[dict[str, Any]], stock_params: dict[str, Any], min_speed: float) -> dict[str, Any]:
  samples: list[dict[str, Any]] = []
  live_lat_accel_factors: list[float] = []
  live_frictions: list[float] = []
  live_lat_accel_offsets: list[float] = []
  live_steer_ratios: list[float] = []
  live_stiffness_factors: list[float] = []
  live_delays: list[float] = []
  live_use_params: list[bool] = []
  live_valid_flags: list[bool] = []

  for segment_file in segment_files:
    segment = int(segment_file["segment"])
    base_time_s: float | None = None
    car_state = None
    controls_state = None
    car_output = None

    try:
      for event in iter_events(segment_file["path"]):
        event_time_s = event.logMonoTime / 1e9
        if base_time_s is None:
          base_time_s = event_time_s
        segment_time_s = event_time_s - base_time_s

        which = event.which()
        if which == "carState":
          car_state = event.carState
        elif which == "controlsState":
          controls_state = event.controlsState
        elif which == "carOutput":
          car_output = event.carOutput
        elif which == "liveTorqueParameters":
          live = event.liveTorqueParameters
          live_use_params.append(bool(live.useParams))
          live_valid_flags.append(bool(live.liveValid))
          if live.liveValid or live.useParams:
            live_lat_accel_factors.append(float(live.latAccelFactorFiltered))
            live_frictions.append(float(live.frictionCoefficientFiltered))
            live_lat_accel_offsets.append(float(live.latAccelOffsetFiltered))
        elif which == "liveParameters":
          live = event.liveParameters
          if live.valid:
            live_steer_ratios.append(float(live.steerRatio))
            live_stiffness_factors.append(float(live.stiffnessFactor))
        elif which == "liveDelay":
          live_delays.append(float(event.liveDelay.lateralDelay))
        elif which == "carControl" and car_state is not None and controls_state is not None and car_output is not None:
          if controls_state.lateralControlState.which() != "torqueState":
            continue

          torque_state = controls_state.lateralControlState.torqueState
          v_ego = float(car_state.vEgo)
          left_blinker = bool(getattr(car_state, "leftBlinker", False))
          right_blinker = bool(getattr(car_state, "rightBlinker", False))
          if (
            not torque_state.active
            or bool(car_state.standstill)
            or bool(car_state.steeringPressed)
            or left_blinker
            or right_blinker
            or v_ego < min_speed
          ):
            continue

          desired_lateral_accel_controller = float(torque_state.desiredLateralAccel)
          actual_lateral_accel_controller = float(torque_state.actualLateralAccel)
          desired_curvature = float(controls_state.desiredCurvature)
          actual_curvature = float(controls_state.curvature)
          route_time_s = segment * 60.0 + segment_time_s
          samples.append({
            "route": route,
            "segment": segment,
            "route_time_s": route_time_s,
            "segment_time_s": round(segment_time_s, 3),
            "v_ego_mps": v_ego,
            "desired_lateral_accel_controller": desired_lateral_accel_controller,
            "actual_lateral_accel_controller": actual_lateral_accel_controller,
            "desired_lateral_accel_plan": desired_curvature * v_ego * v_ego,
            "actual_lateral_accel_plan": actual_curvature * v_ego * v_ego,
            "desired_curvature": desired_curvature,
            "actual_curvature": actual_curvature,
            "saturated": bool(torque_state.saturated),
            "steer_limited": abs(float(event.carControl.actuators.torque) - float(car_output.actuatorsOutput.torque)) > STEER_LIMIT_EPS,
          })
    except Exception:
      continue

  return {
    "route": route,
    "qlog_count": len(segment_files),
    "segment_count": len(segment_files),
    "segments": [item["segment"] for item in segment_files],
    "stock_params": stock_params,
    "samples": samples,
    "live_params": {
      "lat_accel_factor": summarize_values(live_lat_accel_factors),
      "friction": summarize_values(live_frictions),
      "lat_accel_offset": summarize_values(live_lat_accel_offsets),
      "steer_ratio": summarize_values(live_steer_ratios),
      "stiffness_factor": summarize_values(live_stiffness_factors),
      "lateral_delay": summarize_values(live_delays),
      "use_params_ratio": round(sum(live_use_params) / len(live_use_params), 4) if live_use_params else None,
      "live_valid_ratio": round(sum(live_valid_flags) / len(live_valid_flags), 4) if live_valid_flags else None,
    },
    "live_value_arrays": {
      "lat_accel_factor": live_lat_accel_factors,
      "friction": live_frictions,
      "lat_accel_offset": live_lat_accel_offsets,
      "steer_ratio": live_steer_ratios,
      "stiffness_factor": live_stiffness_factors,
      "lateral_delay": live_delays,
    },
  }


def build_param_comparison(stock_value: float | None, live_summary: dict[str, Any] | None) -> dict[str, Any] | None:
  if stock_value is None and live_summary is None:
    return None
  out: dict[str, Any] = {"stock": stock_value, "live": live_summary}
  if stock_value not in (None, 0.0) and live_summary is not None and live_summary.get("median") is not None:
    out["delta"] = round(float(live_summary["median"]) - float(stock_value), 5)
    out["ratio_live_over_stock"] = round(float(live_summary["median"]) / float(stock_value), 5)
  else:
    out["delta"] = None
    out["ratio_live_over_stock"] = None
  return out


def build_tracking_summary(samples: list[dict[str, Any]], turning_lat_accel: float, strong_lat_accel: float,
                           issue_min_lat_accel: float, issue_max_ratio: float, issue_min_abs_error: float,
                           top_issues: int) -> dict[str, Any]:
  turning_samples = [sample for sample in samples if abs(float(sample["desired_lateral_accel_controller"])) >= turning_lat_accel]
  strong_samples = [sample for sample in samples if abs(float(sample["desired_lateral_accel_controller"])) >= strong_lat_accel]

  speed_bins = [
    (2.5, 5.0, "2.5-5"),
    (5.0, 10.0, "5-10"),
    (10.0, 15.0, "10-15"),
    (15.0, 25.0, "15-25"),
    (25.0, 200.0, "25+"),
  ]

  speed_bin_metrics: list[dict[str, Any]] = []
  for low, high, label in speed_bins:
    bin_samples = [
      sample for sample in turning_samples
      if low <= float(sample["v_ego_mps"]) < high
    ]
    metrics = compute_error_metrics(bin_samples, "desired_lateral_accel_controller", "actual_lateral_accel_controller")
    speed_bin_metrics.append({
      "label": label,
      "min_speed_mps": low,
      "max_speed_mps": high,
      "metrics": metrics,
    })

  return {
    "active_all": {
      "controller": compute_error_metrics(samples, "desired_lateral_accel_controller", "actual_lateral_accel_controller"),
      "planner": compute_error_metrics(samples, "desired_lateral_accel_plan", "actual_lateral_accel_plan"),
    },
    "turning": {
      "controller": compute_error_metrics(turning_samples, "desired_lateral_accel_controller", "actual_lateral_accel_controller"),
      "planner": compute_error_metrics(turning_samples, "desired_lateral_accel_plan", "actual_lateral_accel_plan"),
    },
    "strong_turning": {
      "controller": compute_error_metrics(strong_samples, "desired_lateral_accel_controller", "actual_lateral_accel_controller"),
      "planner": compute_error_metrics(strong_samples, "desired_lateral_accel_plan", "actual_lateral_accel_plan"),
    },
    "speed_bins_turning": speed_bin_metrics,
    "issue_windows": build_issue_windows(
      samples,
      issue_min_lat_accel=issue_min_lat_accel,
      issue_max_ratio=issue_max_ratio,
      issue_min_abs_error=issue_min_abs_error,
      top_issues=top_issues,
    ),
  }


def build_markdown(summary: dict[str, Any], summary_path: Path) -> str:
  lines: list[str] = []
  lines.append("# Lateral Tuning Summary")
  lines.append("")
  lines.append(f"- Generated: `{summary['generated_utc']}`")
  lines.append(f"- Host cache: `{summary['host']}`")
  lines.append(f"- Car fingerprint: `{summary['car_fingerprint']}`")
  lines.append(f"- Routes analyzed: `{', '.join(summary['routes_analyzed'])}`")
  lines.append(f"- Qlogs analyzed: {summary['qlog_count']}")
  lines.append(f"- Active tracking samples: {summary['sample_counts']['all_active']}")
  lines.append(f"- Turning samples (`|desired lat accel| >= {summary['turning_threshold_mps2']:.1f}`): {summary['sample_counts']['turning']}")
  lines.append(f"- Strong-turn samples (`|desired lat accel| >= {summary['strong_turning_threshold_mps2']:.1f}`): {summary['sample_counts']['strong_turning']}")
  lines.append("")

  lines.append("## Stock Vs Live Params")
  lines.append("")
  for key, label in [
    ("lat_accel_factor", "LatAccelFactor"),
    ("friction", "Friction"),
    ("lat_accel_offset", "LatAccelOffset"),
    ("steer_ratio", "SteerRatio"),
    ("lateral_delay", "LateralDelay"),
  ]:
    comparison = summary["param_comparison"].get(key)
    if not comparison:
      continue
    stock = comparison.get("stock")
    live = comparison.get("live")
    if live is None:
      lines.append(f"- {label}: stock `{stock}`; no live samples")
      continue
    ratio = comparison.get("ratio_live_over_stock")
    ratio_text = f", live/stock `{ratio:.3f}`" if ratio is not None else ""
    lines.append("".join([
      f"- {label}: stock `{stock:.5f}`; live median `{live['median']:.5f}` ",
      f"(p10 `{live['p10']:.5f}`, p90 `{live['p90']:.5f}`){ratio_text}",
    ]))

  use_params_ratio = summary["live_param_health"].get("use_params_ratio")
  live_valid_ratio = summary["live_param_health"].get("live_valid_ratio")
  lines.append(
    f"- Live torque params health: `useParams={use_params_ratio}` `liveValid={live_valid_ratio}`"
  )
  lines.append("")

  lines.append("## Tracking Metrics")
  lines.append("")
  lines.append("| Slice | Controller MAE | Controller RMSE | Median Ratio | Under < 0.8 | Sat Ratio |")
  lines.append("| --- | ---: | ---: | ---: | ---: | ---: |")
  for key, label in [("active_all", "All active"), ("turning", "Turning"), ("strong_turning", "Strong turning")]:
    metrics = summary["tracking"][key]["controller"]
    lines.append("".join([
      f"| {label} | {metrics['mae']} | {metrics['rmse']} | {metrics['median_ratio']} | ",
      f"{metrics['under_ratio_below_0p8']} | {metrics['saturation_ratio']} |",
    ]))
  lines.append("")

  lines.append("## Per-Route Turning")
  lines.append("")
  lines.append("| Route | Turning Samples | MAE | Median Ratio | Sat Ratio |")
  lines.append("| --- | ---: | ---: | ---: | ---: |")
  for route_summary in summary["route_summaries"]:
    metrics = route_summary["tracking"]["turning"]["controller"]
    lines.append("".join([
      f"| {route_summary['route']} | {metrics['count']} | {metrics['mae']} | ",
      f"{metrics['median_ratio']} | {metrics['saturation_ratio']} |",
    ]))
  lines.append("")

  lines.append("## Speed Bins (Turning)")
  lines.append("")
  lines.append("| Speed Bin m/s | Samples | MAE | Median Ratio | Sat Ratio |")
  lines.append("| --- | ---: | ---: | ---: | ---: |")
  for speed_bin in summary["tracking"]["speed_bins_turning"]:
    metrics = speed_bin["metrics"]
    lines.append("".join([
      f"| {speed_bin['label']} | {metrics['count']} | {metrics['mae']} | ",
      f"{metrics['median_ratio']} | {metrics['saturation_ratio']} |",
    ]))
  lines.append("")

  issue_windows = summary["tracking"]["issue_windows"]
  lines.append("## Top Issue Windows")
  lines.append("")
  if not issue_windows:
    lines.append("- No issue windows matched the configured thresholds.")
  else:
    for window in issue_windows:
      lines.append("".join([
        "- ",
        f"`{window['route']}` seg `{window['segment']}` at ",
        f"`{window['start_segment_time_s']:.2f}-{window['end_segment_time_s']:.2f}s`: ",
        f"desired `{window['desired_lateral_accel_controller']:.3f}` vs actual ",
        f"`{window['actual_lateral_accel_controller']:.3f}` m/s², abs err ",
        f"`{window['worst_abs_error']:.3f}`, ratio `{window['worst_ratio']:.3f}`, ",
        f"saturated `{window['saturated_any']}`, steer-limited `{window['steer_limited_any']}`",
      ]))
  lines.append("")

  lines.append("## Artifact")
  lines.append("")
  lines.append(f"- Summary JSON: `{format_path(summary_path)}`")
  lines.append(f"- Summary Markdown: `{format_path(summary_path.with_suffix('.md'))}`")
  lines.append("")
  return "\n".join(lines)


def main() -> int:
  args = parse_args()
  download_root = Path(args.download_root).expanduser()
  analysis_root = Path(args.analysis_root).expanduser()

  segments = iter_qlog_files(download_root, args.host)
  selected_routes, selected_segments, selected_meta = select_routes(
    segments,
    requested_routes=args.route,
    requested_car_fingerprint=args.car_fingerprint,
    max_routes=args.max_routes,
  )

  route_analyses = [
    analyze_route(route, selected_segments[route], selected_meta[route], min_speed=args.min_speed)
    for route in selected_routes
  ]

  combined_samples = [sample for route_analysis in route_analyses for sample in route_analysis["samples"]]
  detected_fingerprints = sorted({route_analysis["stock_params"]["car_fingerprint"] for route_analysis in route_analyses})
  if len(detected_fingerprints) != 1:
    raise RuntimeError(f"Expected one car fingerprint in selected routes, found: {detected_fingerprints}")

  fingerprint = detected_fingerprints[0]
  car_label = args.car_fingerprint or fingerprint or "unknown_car"
  output_dir = Path(args.output_dir).expanduser() if args.output_dir else analysis_root / args.host / car_label / utc_now_stamp()
  output_dir.mkdir(parents=True, exist_ok=True)

  tracking_summary = build_tracking_summary(
    combined_samples,
    turning_lat_accel=args.turning_lat_accel,
    strong_lat_accel=args.strong_lat_accel,
    issue_min_lat_accel=args.issue_min_lat_accel,
    issue_max_ratio=args.issue_max_ratio,
    issue_min_abs_error=args.issue_min_abs_error,
    top_issues=args.top_issues,
  )

  route_summaries: list[dict[str, Any]] = []
  for route_analysis in route_analyses:
    route_tracking = build_tracking_summary(
      route_analysis["samples"],
      turning_lat_accel=args.turning_lat_accel,
      strong_lat_accel=args.strong_lat_accel,
      issue_min_lat_accel=args.issue_min_lat_accel,
      issue_max_ratio=args.issue_max_ratio,
      issue_min_abs_error=args.issue_min_abs_error,
      top_issues=args.top_issues,
    )
    route_summaries.append({
      "route": route_analysis["route"],
      "qlog_count": route_analysis["qlog_count"],
      "segment_count": route_analysis["segment_count"],
      "segments": route_analysis["segments"],
      "tracking": route_tracking,
      "live_params": route_analysis["live_params"],
    })

  stock_params = route_analyses[0]["stock_params"]
  live_lat_accel_factor = summarize_values([
    value for route_analysis in route_analyses for value in route_analysis["live_value_arrays"]["lat_accel_factor"]
  ])
  live_friction = summarize_values([
    value for route_analysis in route_analyses for value in route_analysis["live_value_arrays"]["friction"]
  ])
  live_lat_accel_offset = summarize_values([
    value for route_analysis in route_analyses for value in route_analysis["live_value_arrays"]["lat_accel_offset"]
  ])
  live_steer_ratio = summarize_values([
    value for route_analysis in route_analyses for value in route_analysis["live_value_arrays"]["steer_ratio"]
  ])
  live_lateral_delay = summarize_values([
    value for route_analysis in route_analyses for value in route_analysis["live_value_arrays"]["lateral_delay"]
  ])

  use_params_values = [
    route_analysis["live_params"]["use_params_ratio"]
    for route_analysis in route_analyses
    if route_analysis["live_params"]["use_params_ratio"] is not None
  ]
  live_valid_values = [
    route_analysis["live_params"]["live_valid_ratio"]
    for route_analysis in route_analyses
    if route_analysis["live_params"]["live_valid_ratio"] is not None
  ]

  summary: dict[str, Any] = {
    "generated_utc": utc_now_iso(),
    "host": args.host,
    "car_fingerprint": fingerprint,
    "routes_analyzed": selected_routes,
    "qlog_count": sum(route_analysis["qlog_count"] for route_analysis in route_analyses),
    "segment_count": sum(route_analysis["segment_count"] for route_analysis in route_analyses),
    "sample_counts": {
      "all_active": tracking_summary["active_all"]["controller"]["count"],
      "turning": tracking_summary["turning"]["controller"]["count"],
      "strong_turning": tracking_summary["strong_turning"]["controller"]["count"],
    },
    "turning_threshold_mps2": args.turning_lat_accel,
    "strong_turning_threshold_mps2": args.strong_lat_accel,
    "tracking": tracking_summary,
    "route_summaries": route_summaries,
    "stock_params": stock_params,
    "param_comparison": {
      "lat_accel_factor": build_param_comparison(stock_params["lat_accel_factor"], live_lat_accel_factor),
      "friction": build_param_comparison(stock_params["friction"], live_friction),
      "lat_accel_offset": build_param_comparison(stock_params["lat_accel_offset"], live_lat_accel_offset),
      "steer_ratio": build_param_comparison(stock_params["steer_ratio"], live_steer_ratio),
      "lateral_delay": build_param_comparison(stock_params["steer_actuator_delay"] + 0.2, live_lateral_delay),
    },
    "live_param_health": {
      "use_params_ratio": round(fmean(use_params_values), 4) if use_params_values else None,
      "live_valid_ratio": round(fmean(live_valid_values), 4) if live_valid_values else None,
    },
    "selection": {
      "requested_routes": args.route,
      "requested_car_fingerprint": args.car_fingerprint,
      "max_routes": args.max_routes,
      "min_speed_mps": args.min_speed,
      "blinker_filter_enabled": True,
      "steering_override_filter_enabled": True,
    },
    "artifact_dir": str(output_dir),
  }

  summary_path = output_dir / "summary.json"
  summary_path.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n")
  summary_md_path = output_dir / "summary.md"
  summary_md_path.write_text(build_markdown(summary, summary_path))

  print(f"[lateral-analysis] routes: {', '.join(selected_routes)}")
  print(f"[lateral-analysis] carFingerprint={fingerprint}")
  print("".join([
    "[lateral-analysis] samples ",
    f"active={summary['sample_counts']['all_active']} ",
    f"turning={summary['sample_counts']['turning']} ",
    f"strong={summary['sample_counts']['strong_turning']}",
  ]))
  print(f"[lateral-analysis] summary: {summary_path}")
  return 0


if __name__ == "__main__":
  raise SystemExit(main())
