#!/usr/bin/env python3
"""Scan all locally downloaded qlogs for stop events across routes."""

from __future__ import annotations

import argparse
import json
import sys
from dataclasses import asdict
from datetime import datetime, timezone
from pathlib import Path
from statistics import median
from typing import Any

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
  sys.path.insert(0, str(REPO_ROOT))

from openpilot.tools.route_sync.common import DEFAULT_DOWNLOAD_ROOT
from tools.stopping.analyze_stopping_behavior import (
  EVENT_MODES,
  SegmentFile,
  compute_event,
  find_stop_events_with_source,
  iter_qlog_files,
  load_samples,
)

DEFAULT_OUTPUT_ROOT = Path.home() / ".comma" / "stopping_behavior" / "analysis" / "corpus"


def utc_now_iso() -> str:
  return datetime.now(timezone.utc).replace(microsecond=0).isoformat()


def parse_args() -> argparse.Namespace:
  parser = argparse.ArgumentParser(description="Find stop events across all downloaded routes")
  parser.add_argument("--host", required=True, help="Host subfolder under download root, e.g. commawifi")
  parser.add_argument("--download-root", default=str(DEFAULT_DOWNLOAD_ROOT),
                      help=f"Local download root. Default: {DEFAULT_DOWNLOAD_ROOT}")
  parser.add_argument("--output-root", default=str(DEFAULT_OUTPUT_ROOT),
                      help=f"Output root for corpus reports. Default: {DEFAULT_OUTPUT_ROOT}")
  parser.add_argument("--route", action="append", default=[], help="Analyze only specific route(s) (repeatable)")
  parser.add_argument("--max-routes", type=int, default=0, help="Limit number of routes analyzed (0 = all)")
  parser.add_argument("--max-segments-per-route", type=int, default=0, help="Limit newest segments per route (0 = all)")
  parser.add_argument("--min-entry-speed", type=float, default=0.5, help="Minimum stop approach speed")
  parser.add_argument("--standstill-speed", type=float, default=0.12, help="Standstill speed threshold")
  parser.add_argument("--hold-time", type=float, default=0.5, help="Standstill hold time threshold (s)")
  parser.add_argument("--max-stop-search", type=float, default=35.0, help="Max seconds from stop signal to hold")
  parser.add_argument("--event-mode", choices=EVENT_MODES, default="hybrid", help="Stop event detector mode")
  parser.add_argument("--require-enabled-speed-events", action="store_true",
                      help="In speed/hybrid mode, keep only events with at least one enabled sample")
  parser.add_argument("--entry-lookback", type=float, default=8.0, help="Lookback window for approach speed")
  parser.add_argument("--verbose-routes", action="store_true", help="Print per-route scan output")
  parser.add_argument("--output-dir", default=None, help="Explicit output directory")
  return parser.parse_args()


def summarize_route(route: str, route_segments: list[SegmentFile], args: argparse.Namespace) -> dict[str, Any]:
  if args.max_segments_per_route > 0 and len(route_segments) > args.max_segments_per_route:
    route_segments = route_segments[-args.max_segments_per_route:]

  samples = load_samples(route_segments)
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

  events = [
    compute_event(idx + 1, event_source, samples, start_idx, stop_idx, hold_idx, approach_speed, "")
    for idx, (start_idx, stop_idx, hold_idx, approach_speed, event_source) in enumerate(event_ranges)
  ]
  event_payload = []
  for item in events:
    event_data = asdict(item)
    event_data["route_name"] = route
    event_payload.append(event_data)

  durations = [item.duration_s for item in events]
  approach_speeds = [item.approach_speed_mps for item in events]
  min_a_ego = [item.min_a_ego_mps2 for item in events]

  return {
    "route": route,
    "qlog_count": len(route_segments),
    "sample_count": len(samples),
    "event_count": len(events),
    "segments": [item.segment for item in route_segments],
    "median_duration_s": float(median(durations)) if durations else None,
    "median_approach_speed_mps": float(median(approach_speeds)) if approach_speeds else None,
    "median_min_a_ego_mps2": float(median(min_a_ego)) if min_a_ego else None,
    "events": event_payload,
  }


def build_output_dir(output_root: Path, host: str, override: str | None) -> Path:
  if override:
    return Path(override).expanduser()
  stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
  return output_root.expanduser() / host / stamp


def build_markdown(summary: dict[str, Any]) -> str:
  lines: list[str] = []
  lines.append(f"# Stop Event Corpus Scan ({summary['host']})")
  lines.append("")
  lines.append(f"- Generated (UTC): {summary['generated_utc']}")
  lines.append(f"- Routes analyzed: {summary['routes_analyzed']}")
  lines.append(f"- Routes with >=1 stop event: {summary['routes_with_stops']}")
  lines.append(f"- Total qlogs analyzed: {summary['total_qlogs']}")
  lines.append(f"- Event mode: `{summary['event_mode']}`")
  lines.append(f"- require_enabled_speed_events: `{summary['require_enabled_speed_events']}`")
  lines.append(f"- Total stop events found: {summary['total_stop_events']}")
  lines.append("")
  lines.append("|Route|Qlogs|Events|Median duration s|Median approach m/s|Median min aEgo|")
  lines.append("|---|---:|---:|---:|---:|---:|")

  for route_summary in summary["routes"]:
    lines.append(
      "|"
      f"{route_summary['route']}|"
      f"{route_summary['qlog_count']}|"
      f"{route_summary['event_count']}|"
      f"{format_metric(route_summary['median_duration_s'], 2)}|"
      f"{format_metric(route_summary['median_approach_speed_mps'], 2)}|"
      f"{format_metric(route_summary['median_min_a_ego_mps2'], 2)}|"
    )

  lines.append("")
  return "\n".join(lines)


def format_metric(value: float | None, digits: int = 3) -> str:
  if value is None:
    return "n/a"
  return f"{value:.{digits}f}"


def main() -> int:
  args = parse_args()
  output_dir = build_output_dir(Path(args.output_root), args.host, args.output_dir)
  output_dir.mkdir(parents=True, exist_ok=True)

  all_segments = iter_qlog_files(Path(args.download_root), args.host)
  route_names = sorted({item.route for item in all_segments})
  if args.route:
    selected = set(args.route)
    route_names = [name for name in route_names if name in selected]

  if args.max_routes > 0:
    route_names = route_names[:args.max_routes]

  route_summaries: list[dict[str, Any]] = []
  for route in route_names:
    route_segments = sorted(
      [item for item in all_segments if item.route == route],
      key=lambda item: item.segment,
    )
    if not route_segments:
      continue
    route_summary = summarize_route(route, route_segments, args)
    route_summaries.append(route_summary)
    if args.verbose_routes or route_summary["event_count"] > 0:
      print(
        f"[corpus] route={route} qlogs={route_summary['qlog_count']} "
        f"events={route_summary['event_count']}"
      )

  route_summaries.sort(key=lambda item: (item["event_count"], item["qlog_count"]), reverse=True)
  total_qlogs = sum(item["qlog_count"] for item in route_summaries)
  total_events = sum(item["event_count"] for item in route_summaries)
  routes_with_stops = sum(1 for item in route_summaries if item["event_count"] > 0)

  summary: dict[str, Any] = {
    "generated_utc": utc_now_iso(),
    "host": args.host,
    "routes_analyzed": len(route_summaries),
    "routes_with_stops": routes_with_stops,
    "total_qlogs": total_qlogs,
    "event_mode": args.event_mode,
    "require_enabled_speed_events": bool(args.require_enabled_speed_events),
    "min_entry_speed": args.min_entry_speed,
    "standstill_speed": args.standstill_speed,
    "hold_time": args.hold_time,
    "max_stop_search": args.max_stop_search,
    "entry_lookback": args.entry_lookback,
    "total_stop_events": total_events,
    "routes": route_summaries,
  }

  json_path = output_dir / "summary.json"
  md_path = output_dir / "summary.md"
  json_path.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n")
  md_path.write_text(build_markdown(summary) + "\n")

  print(f"[corpus] routes_analyzed={summary['routes_analyzed']}")
  print(f"[corpus] total_stop_events={summary['total_stop_events']}")
  print(f"[corpus] summary_json={json_path}")
  print(f"[corpus] summary_md={md_path}")
  return 0


if __name__ == "__main__":
  raise SystemExit(main())
