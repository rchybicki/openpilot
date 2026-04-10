#!/usr/bin/env python3
"""Match bookmark markers to nearby stop events for bad-stop triage."""

from __future__ import annotations

import argparse
import json
import sys
from dataclasses import asdict
from datetime import datetime, timezone
from pathlib import Path
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
  read_events,
)

DEFAULT_OUTPUT_ROOT = Path.home() / ".comma" / "stopping_behavior" / "analysis" / "bookmarks"


def utc_now_iso() -> str:
  return datetime.now(timezone.utc).replace(microsecond=0).isoformat()


def parse_args() -> argparse.Namespace:
  parser = argparse.ArgumentParser(description="Find bad-stop bookmarks and match them to stop events")
  parser.add_argument("--host", required=True, help="Host subfolder under download root, e.g. commawifi")
  parser.add_argument("--download-root", default=str(DEFAULT_DOWNLOAD_ROOT),
                      help=f"Local download root. Default: {DEFAULT_DOWNLOAD_ROOT}")
  parser.add_argument("--output-root", default=str(DEFAULT_OUTPUT_ROOT),
                      help=f"Output root for bookmark reports. Default: {DEFAULT_OUTPUT_ROOT}")
  parser.add_argument("--output-dir", default=None, help="Explicit output directory path")
  parser.add_argument("--route", action="append", default=[], help="Analyze only specific route(s)")
  parser.add_argument("--max-routes", type=int, default=0, help="Limit number of routes analyzed (0 = all)")
  parser.add_argument("--event-mode", choices=EVENT_MODES, default="engaged_signal")
  parser.add_argument("--min-entry-speed", type=float, default=0.5)
  parser.add_argument("--entry-lookback", type=float, default=8.0)
  parser.add_argument("--standstill-speed", type=float, default=0.12)
  parser.add_argument("--hold-time", type=float, default=0.5)
  parser.add_argument("--max-stop-search", type=float, default=35.0)
  parser.add_argument("--require-enabled-speed-events", action="store_true")
  parser.add_argument("--match-window-before", type=float, default=8.0,
                      help="Seconds before event start to allow bookmark->event match")
  parser.add_argument("--match-window-after", type=float, default=8.0,
                      help="Seconds after stop hold to allow bookmark->event match")
  parser.add_argument("--nearest-max-gap", type=float, default=25.0,
                      help="Max seconds from stop hold for nearest fallback match")
  parser.add_argument("--verbose-routes", action="store_true")
  return parser.parse_args()


def build_output_dir(base_root: Path, host: str, override: str | None) -> Path:
  if override:
    return Path(override).expanduser()
  stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
  return base_root.expanduser() / host / stamp


def load_user_flags(route_segments: list[SegmentFile]) -> list[dict[str, Any]]:
  user_flags: list[dict[str, Any]] = []
  first_mono_time: float | None = None

  for seg in route_segments:
    for msg in read_events(seg.path):
      mono_s = msg.logMonoTime * 1e-9
      if first_mono_time is None:
        first_mono_time = mono_s
      if msg.which() in {"userBookmark", "userFlag"}:
        user_flags.append({
          "segment": seg.segment,
          "mono_time_s": mono_s,
          "t_rel_s": mono_s - first_mono_time,
        })

  return user_flags


def match_flag_to_event(
  flag_t: float,
  events: list[Any],
  before_window: float,
  after_window: float,
  nearest_max_gap: float,
) -> tuple[Any | None, str]:
  in_window = [
    event for event in events
    if (event.start_time_s - before_window) <= flag_t <= (event.stop_hold_time_s + after_window)
  ]
  if in_window:
    return min(in_window, key=lambda event: abs(flag_t - event.stop_hold_time_s)), "window"

  if not events:
    return None, "none"

  nearest = min(events, key=lambda event: abs(flag_t - event.stop_hold_time_s))
  if abs(flag_t - nearest.stop_hold_time_s) <= nearest_max_gap:
    return nearest, "nearest"
  return None, "none"


def build_markdown(report: dict[str, Any]) -> str:
  lines: list[str] = []
  lines.append(f"# Bookmarked Bad Stops ({report['host']})")
  lines.append("")
  lines.append(f"- Generated (UTC): {report['generated_utc']}")
  lines.append(f"- Routes analyzed: {report['routes_analyzed']}")
  lines.append(f"- Routes with bookmarks: {report['routes_with_bookmarks']}")
  lines.append(f"- Total bookmarks: {report['total_bookmarks']}")
  lines.append(f"- Matched bookmarks: {report['matched_bookmarks']}")
  lines.append(f"- Unmatched bookmarks: {report['unmatched_bookmarks']}")
  lines.append(f"- Event mode: `{report['event_mode']}`")
  lines.append(f"- min-entry-speed: `{report['min_entry_speed']}`")
  lines.append("")

  lines.append("|Route|Bookmarks|Matched|Unmatched|Detected stop events|")
  lines.append("|---|---:|---:|---:|---:|")
  for route_summary in report["routes"]:
    lines.append(
      "|"
      f"{route_summary['route']}|"
      f"{route_summary['bookmark_count']}|"
      f"{route_summary['matched_count']}|"
      f"{route_summary['unmatched_count']}|"
      f"{route_summary['event_count']}|"
    )
  lines.append("")

  lines.append("## Bookmark Matches")
  lines.append("")
  lines.append("|Route|Seg|Bookmark t(s)|Match|Event|Delta to hold (s)|Rollout2m|EntryJerk|EntryCmdJerk|EndJerk|EndStep|CmdJerk|CmdStep|Creep|Flags|")
  lines.append("|---|---:|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|")
  for match in report["bookmark_matches"]:
    event = match.get("event")
    if event is None:
      lines.append(
        "|"
        f"{match['route']}|"
        f"{match['segment']}|"
        f"{match['bookmark_t_rel_s']:.3f}|"
        f"{match['match_type']}|"
        "n/a|n/a|n/a|n/a|n/a|n/a|n/a|n/a|n/a|-|"
      )
      continue

    flags = []
    if event.get("stop_signal_dropped_before_hold"):
      flags.append("sig_drop")
    if event.get("left_stopping_state_before_hold"):
      flags.append("exit_stop")
    if event.get("positive_accel_cmd_with_stop_signal_near_hold"):
      flags.append("pos_cmd_sig")
    if event.get("speed_rebound_while_stop_signal_mps", 0.0) >= 0.15:
      flags.append("rebound_sig")
    if event.get("speed_rebound_while_should_stop_mps", 0.0) >= 0.08:
      flags.append("rebound_should")
    if (event.get("should_stop_unexpected_accel_mps2") or -1.0) > 0.10:
      flags.append("unexp_should")
    if (event.get("should_stop_decel_relief_spike_mps2") or -1.0) > 0.18:
      flags.append("relief_should")
    if (event.get("unexpected_accel_while_braking_mps2") or -1.0) > 0.20:
      flags.append("unexp_accel")

    lines.append(
      "|"
      f"{match['route']}|"
      f"{match['segment']}|"
      f"{match['bookmark_t_rel_s']:.3f}|"
      f"{match['match_type']}|"
      f"{event['event_id']}|"
      f"{match['delta_to_hold_s']:.3f}|"
      f"{event.get('rollout_distance_from_2mps_m', 0.0):.2f}|"
      f"{event.get('entry_stop_jerk_mps3', 0.0):.2f}|"
      f"{event.get('entry_stop_cmd_jerk_mps3', 0.0):.2f}|"
      f"{event.get('end_stop_jerk_mps3', 0.0):.2f}|"
      f"{event.get('end_stop_accel_step_mps2', 0.0):.2f}|"
      f"{event.get('end_stop_cmd_jerk_mps3', 0.0):.2f}|"
      f"{event.get('end_stop_cmd_step_mps2', 0.0):.2f}|"
      f"{event.get('creep_after_stop_mps', 0.0):.3f}|"
      f"{', '.join(flags) if flags else '-'}|"
    )
  lines.append("")

  lines.append("## Next Step Commands")
  lines.append("")
  lines.append("```bash")
  for route_summary in report["routes"][:5]:
    route = route_summary["route"]
    lines.append(
      "python tools/stopping/analyze_stopping_behavior.py "
      f"--host {report['host']} --route {route} --event-mode {report['event_mode']} "
      f"--min-entry-speed {report['min_entry_speed']}"
    )
  lines.append("```")
  lines.append("")

  return "\n".join(lines)


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
  bookmark_matches: list[dict[str, Any]] = []
  total_bookmarks = 0
  matched_bookmarks = 0

  for route in route_names:
    route_segments = sorted(
      [item for item in all_segments if item.route == route],
      key=lambda item: item.segment,
    )
    if not route_segments:
      continue

    user_flags = load_user_flags(route_segments)
    if not user_flags and not args.verbose_routes:
      continue

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

    matched = 0
    for flag in user_flags:
      event, match_type = match_flag_to_event(
        flag_t=flag["t_rel_s"],
        events=events,
        before_window=args.match_window_before,
        after_window=args.match_window_after,
        nearest_max_gap=args.nearest_max_gap,
      )
      match_entry: dict[str, Any] = {
        "route": route,
        "segment": flag["segment"],
        "bookmark_t_rel_s": flag["t_rel_s"],
        "match_type": match_type,
        "event": asdict(event) if event is not None else None,
      }
      if event is not None:
        matched += 1
        matched_bookmarks += 1
        match_entry["delta_to_hold_s"] = flag["t_rel_s"] - event.stop_hold_time_s
      bookmark_matches.append(match_entry)

    total_bookmarks += len(user_flags)
    route_summaries.append({
      "route": route,
      "qlog_count": len(route_segments),
      "bookmark_count": len(user_flags),
      "matched_count": matched,
      "unmatched_count": len(user_flags) - matched,
      "event_count": len(events),
    })
    if args.verbose_routes or user_flags:
      print(
        f"[bookmarks] route={route} qlogs={len(route_segments)} "
        f"bookmarks={len(user_flags)} matched={matched} events={len(events)}"
      )

  route_summaries.sort(key=lambda item: (item["bookmark_count"], item["matched_count"]), reverse=True)

  report: dict[str, Any] = {
    "generated_utc": utc_now_iso(),
    "host": args.host,
    "routes_analyzed": len(route_names),
    "routes_with_bookmarks": sum(1 for route in route_summaries if route["bookmark_count"] > 0),
    "total_bookmarks": total_bookmarks,
    "matched_bookmarks": matched_bookmarks,
    "unmatched_bookmarks": total_bookmarks - matched_bookmarks,
    "event_mode": args.event_mode,
    "min_entry_speed": args.min_entry_speed,
    "match_window_before": args.match_window_before,
    "match_window_after": args.match_window_after,
    "nearest_max_gap": args.nearest_max_gap,
    "routes": route_summaries,
    "bookmark_matches": bookmark_matches,
  }

  json_path = output_dir / "summary.json"
  md_path = output_dir / "summary.md"
  json_path.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")
  md_path.write_text(build_markdown(report) + "\n")

  print(f"[bookmarks] routes_analyzed={report['routes_analyzed']}")
  print(f"[bookmarks] total_bookmarks={report['total_bookmarks']}")
  print(f"[bookmarks] matched_bookmarks={report['matched_bookmarks']}")
  print(f"[bookmarks] summary_json={json_path}")
  print(f"[bookmarks] summary_md={md_path}")
  return 0


if __name__ == "__main__":
  raise SystemExit(main())
