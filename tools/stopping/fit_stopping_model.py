#!/usr/bin/env python3
"""Fit a simple braking response model from stop-event logs."""

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

from openpilot.tools.stopping.analyze_stopping_behavior import (  # pylint: disable=wrong-import-position
  DEFAULT_DOWNLOAD_ROOT,
  SegmentFile,
  iter_qlog_files,
  load_samples,
)
from openpilot.tools.stopping.stopping_model import fit_stopping_model  # pylint: disable=wrong-import-position


DEFAULT_MODEL_ROOT = Path.home() / ".comma" / "stopping_behavior" / "models"


def parse_args() -> argparse.Namespace:
  parser = argparse.ArgumentParser(description="Fit braking response model from stop-event summary JSON files")
  parser.add_argument("--summary-json", action="append", required=True,
                      help="Path to analyze_stopping_behavior summary.json (repeatable)")
  parser.add_argument("--download-root", default=str(DEFAULT_DOWNLOAD_ROOT),
                      help=f"Local log root used by analyzer. Default: {DEFAULT_DOWNLOAD_ROOT}")
  parser.add_argument("--event-source", choices=["all", "signal", "speed", "hybrid"], default="all",
                      help="Event source filter from summary event_source")
  parser.add_argument("--include-disabled", action="store_true",
                      help="Include samples where controls are not enabled (not recommended for command-response model)")
  parser.add_argument("--max-delay-frames", type=int, default=20, help="Maximum command delay (frames) to search")
  parser.add_argument("--min-speed", type=float, default=0.0, help="Minimum vEgo for training rows")
  parser.add_argument("--max-speed", type=float, default=2.0, help="Maximum vEgo for training rows")
  parser.add_argument("--relief-cmd-threshold", type=float, default=-0.25, help="Accel command threshold for clutch-relief feature")
  parser.add_argument("--low-speed-ref", type=float, default=1.2, help="Reference speed for low-speed feature scaling")
  parser.add_argument("--min-rows", type=int, default=50, help="Minimum training rows required")
  parser.add_argument("--delay-min-sample-ratio", type=float, default=0.40,
                      help="Minimum sample-count ratio (vs max-delay candidate) required during delay selection")
  parser.add_argument("--delay-rmse-tolerance", type=float, default=0.03,
                      help="Allow delays within this relative RMSE tolerance, then pick lower delay")
  parser.add_argument("--output", default=None, help="Output model JSON path")
  return parser.parse_args()


def utc_stamp() -> str:
  return datetime.now(UTC).strftime("%Y%m%dT%H%M%SZ")


def resolve_output_path(explicit: str | None) -> Path:
  if explicit:
    return Path(explicit).expanduser()
  DEFAULT_MODEL_ROOT.mkdir(parents=True, exist_ok=True)
  return DEFAULT_MODEL_ROOT / f"stopping_model_{utc_stamp()}.json"


def load_summary(path: Path) -> dict[str, Any]:
  data = json.loads(path.read_text())
  if not isinstance(data, dict):
    raise ValueError(f"Summary root must be object: {path}")
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
  cache: dict[tuple[str, str], list[Any]],
  segment_cache: dict[str, list[SegmentFile]],
  download_root: Path,
  host: str,
  route: str,
) -> list[Any]:
  cache_key = (host, route)
  if cache_key in cache:
    return cache[cache_key]

  if host not in segment_cache:
    segment_cache[host] = iter_qlog_files(download_root, host)
  route_segments = sorted((seg for seg in segment_cache[host] if seg.route == route), key=lambda item: item.segment)
  if not route_segments:
    raise RuntimeError(f"No route segments found for host={host} route={route}")
  samples = load_samples(route_segments)
  cache[cache_key] = samples
  return samples


def collect_windows(
  summaries: list[dict[str, Any]],
  download_root: Path,
  event_source: str,
) -> tuple[list[Any], list[tuple[int, int]], list[dict[str, Any]]]:
  sample_cache: dict[tuple[str, str], list[Any]] = {}
  segment_cache: dict[str, list[SegmentFile]] = {}
  all_windows: list[tuple[int, int]] = []
  combined_samples: list[Any] = []
  metadata: list[dict[str, Any]] = []

  for summary in summaries:
    host = str(summary.get("host", "commawifi"))
    route = str(summary.get("route", ""))
    if not route:
      continue
    samples = route_samples(sample_cache, segment_cache, download_root, host, route)
    times = np.array([float(item.t) for item in samples], dtype=float)
    base_offset = len(combined_samples)
    combined_samples.extend(samples)

    for event in summary.get("events", []):
      if not isinstance(event, dict):
        continue
      if event_source != "all":
        source = str(event.get("event_source", ""))
        if source != event_source:
          continue

      start_time = event.get("start_time_s")
      hold_time = event.get("stop_hold_time_s")
      if start_time is None or hold_time is None:
        continue

      start_idx = nearest_index(times, float(start_time))
      hold_idx = nearest_index(times, float(hold_time))
      if hold_idx <= start_idx:
        continue

      all_windows.append((base_offset + start_idx, base_offset + hold_idx))
      metadata.append({
        "host": host,
        "route": route,
        "event_id": event.get("event_id"),
        "event_source": event.get("event_source"),
        "start_time_s": float(start_time),
        "stop_hold_time_s": float(hold_time),
      })

  return combined_samples, all_windows, metadata


def main() -> int:
  args = parse_args()
  summary_paths = [Path(item).expanduser() for item in args.summary_json]
  missing = [str(path) for path in summary_paths if not path.exists()]
  if missing:
    for path in missing:
      print(f"[fit-model] missing summary: {path}", file=sys.stderr)
    return 2

  summaries = [load_summary(path) for path in summary_paths]
  download_root = Path(args.download_root).expanduser()
  samples, windows, metadata = collect_windows(
    summaries=summaries,
    download_root=download_root,
    event_source=args.event_source,
  )
  if not windows:
    print("[fit-model] no usable event windows found", file=sys.stderr)
    return 2

  model, delay_fits = fit_stopping_model(
    samples=samples,
    windows=windows,
    max_delay_frames=args.max_delay_frames,
    min_speed=args.min_speed,
    max_speed=args.max_speed,
    relief_cmd_threshold=args.relief_cmd_threshold,
    low_speed_ref=args.low_speed_ref,
    min_rows=args.min_rows,
    delay_min_sample_ratio=args.delay_min_sample_ratio,
    delay_rmse_tolerance=args.delay_rmse_tolerance,
    require_enabled=not args.include_disabled,
  )

  output_path = resolve_output_path(args.output)
  output_path.parent.mkdir(parents=True, exist_ok=True)
  output = {
    "generated_utc": datetime.now(UTC).replace(microsecond=0).isoformat(),
    "summary_files": [str(path) for path in summary_paths],
    "event_source_filter": args.event_source,
    "windows_used": len(windows),
    "window_metadata": metadata[:200],
    "model": model.as_json(),
    "delay_fit": [fit.__dict__ for fit in delay_fits],
  }
  output_path.write_text(json.dumps(output, indent=2, sort_keys=True) + "\n")

  print(f"[fit-model] windows={len(windows)}")
  print(f"[fit-model] best_delay_frames={model.delay_frames}")
  print(f"[fit-model] rows={model.sample_count}")
  print(f"[fit-model] rmse={model.rmse:.4f} mae={model.mae:.4f} r2={model.r2:.4f}")
  print(f"[fit-model] delay_selection=min_ratio={args.delay_min_sample_ratio:.2f} rmse_tol={args.delay_rmse_tolerance:.3f}")
  print(f"[fit-model] output={output_path}")
  return 0


if __name__ == "__main__":
  raise SystemExit(main())
