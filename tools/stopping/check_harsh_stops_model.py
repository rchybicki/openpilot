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

from openpilot.tools.stopping.analyze_stopping_behavior import (  # pylint: disable=wrong-import-position
  DEFAULT_DOWNLOAD_ROOT,
  SegmentFile,
  iter_qlog_files,
  load_samples,
)
from openpilot.tools.stopping.stopping_model import FittedStoppingModel, simulate_event_with_model  # pylint: disable=wrong-import-position


def parse_args() -> argparse.Namespace:
  parser = argparse.ArgumentParser(description="Model-based harsh-stop gate from stop-event summary JSON files")
  parser.add_argument("--model-json", required=True, help="Path to model JSON produced by fit_stopping_model.py")
  parser.add_argument("--summary-json", action="append", required=True,
                      help="Path to analyze_stopping_behavior summary.json (repeatable)")
  parser.add_argument("--download-root", default=str(DEFAULT_DOWNLOAD_ROOT),
                      help=f"Local log root used by analyzer. Default: {DEFAULT_DOWNLOAD_ROOT}")
  parser.add_argument("--event-source", choices=["all", "signal", "speed", "hybrid"], default="all",
                      help="Event source filter from summary event_source")
  parser.add_argument("--min-events", type=int, default=4, help="Minimum event count required to evaluate")
  parser.add_argument("--min-entry-speed", type=float, default=0.20, help="Ignore events below this entry speed")
  parser.add_argument("--max-harsh-rate", type=float, default=0.20, help="Maximum allowed harsh-event rate [0..1]")
  parser.add_argument("--max-pred-end-jerk", type=float, default=0.80, help="Predicted harsh threshold for end-stop jerk")
  parser.add_argument("--min-pred-a-floor", type=float, default=-1.10, help="Predicted harsh threshold for minimum acceleration")
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
  total = len(event_rows)
  harsh_rate = (harsh_count / total) if total else 0.0
  return {
    "generated_utc": datetime.now(UTC).replace(microsecond=0).isoformat(),
    "status": status,
    "reasons": reasons,
    "events_considered": total,
    "harsh_events": harsh_count,
    "harsh_rate": harsh_rate,
    "thresholds": {
      "min_events": args.min_events,
      "min_entry_speed": args.min_entry_speed,
      "max_harsh_rate": args.max_harsh_rate,
      "max_pred_end_jerk": args.max_pred_end_jerk,
      "min_pred_a_floor": args.min_pred_a_floor,
    },
    "event_rows": event_rows,
  }


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

      simulation = simulate_event_with_model(samples, start_idx, hold_idx, hold_idx, model)
      pred_jerk = simulation["pred_end_stop_jerk_mps3"]
      pred_min_a = simulation["pred_min_a_ego_mps2"]
      harsh_flags: list[str] = []
      if pred_jerk is not None and pred_jerk > args.max_pred_end_jerk:
        harsh_flags.append("pred_end_stop_jerk")
      if pred_min_a < args.min_pred_a_floor:
        harsh_flags.append("pred_min_a_ego")

      rows.append({
        "summary_json": str(summary_path),
        "route": route,
        "event_id": event.get("event_id"),
        "event_source": source,
        "entry_speed_mps": entry_speed,
        "pred_end_stop_jerk_mps3": pred_jerk,
        "pred_min_a_ego_mps2": pred_min_a,
        "is_harsh": bool(harsh_flags),
        "flags": harsh_flags,
      })

  status = "pass"
  reasons: list[str] = []
  if len(rows) < args.min_events:
    status = "insufficient_events"
    reasons.append(f"events={len(rows)} < min_events={args.min_events}")
  else:
    harsh_rate = sum(1 for row in rows if row["is_harsh"]) / max(len(rows), 1)
    if harsh_rate > args.max_harsh_rate:
      status = "fail"
      reasons.append(f"harsh_rate={harsh_rate:.3f} > max_harsh_rate={args.max_harsh_rate:.3f}")

  result = build_result(status=status, reasons=reasons, event_rows=rows, args=args)
  print(f"[model-harsh-check] status={status}")
  print(f"[model-harsh-check] events_considered={result['events_considered']}")
  print(f"[model-harsh-check] harsh_events={result['harsh_events']}")
  print(f"[model-harsh-check] harsh_rate={result['harsh_rate']:.3f}")
  if reasons:
    print(f"[model-harsh-check] reasons={'; '.join(reasons)}")

  for idx, row in enumerate([item for item in rows if item["is_harsh"]][:5], start=1):
    message = (
      f"[model-harsh-check] sample#{idx} route={row['route']} event={row['event_id']} entry={row['entry_speed_mps']:.2f}"
      + f" predJerk={row['pred_end_stop_jerk_mps3']} predMinA={row['pred_min_a_ego_mps2']} flags={','.join(row['flags'])}"
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
