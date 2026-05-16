#!/usr/bin/env python3
"""Attach runtime stopping-shadow rlog decisions to detected stop events."""

from __future__ import annotations

import argparse
import json
import sys
from collections import Counter
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
  sys.path.insert(0, str(REPO_ROOT))

from openpilot.tools.route_sync.common import (
  CANONICAL_REMOTE_ROOT,
  DEFAULT_DOWNLOAD_ROOT,
  DEFAULT_HOST,
  FALLBACK_HOST,
  host_download_root,
  local_path_for,
  segment_has_active_lock,
)
from openpilot.tools.route_sync.refresh_routes import download_file
from openpilot.tools.stopping.analyze_stopping_behavior import (
  SegmentFile,
  load_samples,
  read_events,
  short_exception,
  iter_qlog_files,
)

RLOG_FILE_PATTERNS = ("rlog", "rlog.bz2", "rlog.zst")
SHADOW_EVENT_NAME = "stopping_shadow"
DEFAULT_PRE_WINDOW_S = 0.50
DEFAULT_POST_WINDOW_S = 1.00
DEFAULT_MIN_COMMAND_RELIEF_MPS2 = 0.03


@dataclass(frozen=True)
class ShadowDecision:
  mono_time_s: float
  segment: int
  observer_scope: str
  profile: str
  reason: str
  score_delta: float
  confidence: float
  first_output_accel: float | None
  actual_output_accel: float | None
  v_ego: float | None
  a_ego: float | None
  remaining_m: float | None
  rollout_m: float | None
  lead_status: bool
  lead_d_rel: float | None
  selected_rollout_m: float | None
  selected_min_a_ego: float | None
  selected_final_v_ego: float | None
  selected_harsh: bool
  selected_leapfrog: bool
  commit: str
  created: float | None

  @property
  def command_relief_mps2(self) -> float | None:
    if self.first_output_accel is None or self.actual_output_accel is None:
      return None
    return self.first_output_accel - self.actual_output_accel

  @property
  def accepted(self) -> bool:
    return self.reason == "accepted" and self.score_delta < 0.0

  @property
  def accepted_safe(self) -> bool:
    return self.accepted and not self.selected_harsh and not self.selected_leapfrog


@dataclass(frozen=True)
class EventShadowSummary:
  event_id: int
  segment: int
  actual_harsh: bool
  actual_leapfrog: bool
  shadow_count: int
  accepted_count: int
  accepted_safe_count: int
  accepted_unsafe_count: int
  accepted_relief_count: int
  observer_scope_counts: dict[str, int]
  profile_counts: dict[str, int]
  accepted_profile_counts: dict[str, int]
  best_score_delta: float | None
  max_command_relief_mps2: float | None
  first_shadow_time_offset_s: float | None
  last_shadow_time_offset_s: float | None
  predicted_harsh_count: int
  predicted_leapfrog_count: int
  verdict: str


def utc_now_iso() -> str:
  return datetime.now(timezone.utc).replace(microsecond=0).isoformat()


def _finite_float(value: Any) -> float | None:
  try:
    result = float(value)
  except (TypeError, ValueError):
    return None
  return result if result == result and result not in (float("inf"), float("-inf")) else None


def _float_or_default(value: Any, default: float = 0.0) -> float:
  result = _finite_float(value)
  return default if result is None else result


def parse_shadow_log_message(log_message: str, *, mono_time_s: float, segment: int) -> ShadowDecision | None:
  try:
    root = json.loads(log_message)
  except json.JSONDecodeError:
    return None

  payload = root.get("msg")
  if not isinstance(payload, dict) or payload.get("event") != SHADOW_EVENT_NAME:
    return None

  ctx = root.get("ctx")
  if not isinstance(ctx, dict):
    ctx = {}

  return ShadowDecision(
    mono_time_s=mono_time_s,
    segment=segment,
    observer_scope=str(payload.get("observer_scope", "")),
    profile=str(payload.get("profile", "")),
    reason=str(payload.get("reason", "")),
    score_delta=_float_or_default(payload.get("score_delta")),
    confidence=_float_or_default(payload.get("confidence")),
    first_output_accel=_finite_float(payload.get("first_output_accel")),
    actual_output_accel=_finite_float(payload.get("actual_output_accel")),
    v_ego=_finite_float(payload.get("v_ego")),
    a_ego=_finite_float(payload.get("a_ego")),
    remaining_m=_finite_float(payload.get("remaining_m")),
    rollout_m=_finite_float(payload.get("rollout_m")),
    lead_status=bool(payload.get("lead_status", False)),
    lead_d_rel=_finite_float(payload.get("lead_d_rel")),
    selected_rollout_m=_finite_float(payload.get("selected_rollout_m")),
    selected_min_a_ego=_finite_float(payload.get("selected_min_a_ego")),
    selected_final_v_ego=_finite_float(payload.get("selected_final_v_ego")),
    selected_harsh=bool(payload.get("selected_harsh", False)),
    selected_leapfrog=bool(payload.get("selected_leapfrog", False)),
    commit=str(ctx.get("commit", "")),
    created=_finite_float(root.get("created")),
  )


def rlog_path_priority(path: Path) -> int:
  name = path.name
  if name == "rlog":
    return 0
  if name == "rlog.bz2":
    return 1
  if name == "rlog.zst":
    return 2
  return 99


def iter_rlog_files(download_root: Path, host: str) -> list[SegmentFile]:
  host_root = host_download_root(download_root, host)
  if not host_root.exists():
    return []

  segments_by_key: dict[tuple[str, int], SegmentFile] = {}
  for pattern in RLOG_FILE_PATTERNS:
    for rlog_path in host_root.rglob(pattern):
      if segment_has_active_lock(rlog_path.parent):
        continue
      segment_name = rlog_path.parent.name
      if "--" not in segment_name:
        continue
      route, suffix = segment_name.rsplit("--", 1)
      try:
        segment = int(suffix)
        mtime = rlog_path.stat().st_mtime
      except (OSError, ValueError):
        continue
      key = (route, segment)
      existing = segments_by_key.get(key)
      if existing is None or rlog_path_priority(rlog_path) < rlog_path_priority(existing.path):
        segments_by_key[key] = SegmentFile(route=route, segment=segment, path=rlog_path, mtime=mtime)

  return list(segments_by_key.values())


def event_segments(events: list[dict[str, Any]]) -> list[int]:
  segments: set[int] = set()
  for event in events:
    try:
      start_segment = int(event["start_segment"])
      stop_segment = int(event.get("stop_segment", start_segment))
    except (KeyError, TypeError, ValueError):
      continue
    low_segment = min(start_segment, stop_segment)
    high_segment = max(start_segment, stop_segment)
    if high_segment - low_segment <= 3:
      segments.update(range(low_segment, high_segment + 1))
    else:
      segments.update((start_segment, stop_segment))
  return sorted(segments)


def local_rlog_zst_path(download_root: Path, host: str, route: str, segment: int) -> Path:
  remote_path = f"{CANONICAL_REMOTE_ROOT}/{route}--{segment}/rlog.zst"
  return local_path_for(download_root, host, remote_path)


def find_rlog_for_segment(rlogs: list[SegmentFile], route: str, segment: int) -> SegmentFile | None:
  for item in rlogs:
    if item.route == route and item.segment == segment:
      return item
  return None


def download_missing_rlogs(route: str, segments: list[int], *, host: str, download_root: Path, connect_timeout: int) -> dict[str, Any]:
  downloaded: list[str] = []
  skipped: list[int] = []
  failures: list[dict[str, Any]] = []
  rlogs = iter_rlog_files(download_root, host)
  fallback_host = FALLBACK_HOST if host == DEFAULT_HOST else DEFAULT_HOST if host == FALLBACK_HOST else None

  for segment in segments:
    if find_rlog_for_segment(rlogs, route, segment) is not None:
      skipped.append(segment)
      continue

    remote_path = f"{CANONICAL_REMOTE_ROOT}/{route}--{segment}/rlog.zst"
    local_path = local_rlog_zst_path(download_root, host, route, segment)
    try:
      download_file(host, remote_path, local_path, connect_timeout)
      downloaded.append(str(local_path))
      rlogs.append(SegmentFile(route=route, segment=segment, path=local_path, mtime=local_path.stat().st_mtime))
      continue
    except Exception as first_exc:
      if fallback_host is None:
        failures.append({"segment": segment, "error": short_exception(first_exc)})
        continue

    try:
      download_file(fallback_host, remote_path, local_path, connect_timeout)
      downloaded.append(str(local_path))
      rlogs.append(SegmentFile(route=route, segment=segment, path=local_path, mtime=local_path.stat().st_mtime))
    except Exception as second_exc:
      failures.append({"segment": segment, "error": short_exception(second_exc)})

  return {"downloaded": downloaded, "skipped_existing_segments": skipped, "failures": failures}


def load_shadow_decisions(rlog_segments: list[SegmentFile]) -> list[ShadowDecision]:
  decisions: list[ShadowDecision] = []
  for segment_file in sorted(rlog_segments, key=lambda item: (item.segment, str(item.path))):
    for msg in read_events(segment_file.path):
      if msg.which() != "logMessage":
        continue
      decision = parse_shadow_log_message(str(msg.logMessage), mono_time_s=msg.logMonoTime * 1e-9, segment=segment_file.segment)
      if decision is not None:
        decisions.append(decision)
  return decisions


def load_first_mono_time_s(download_root: Path, host: str, route: str) -> float:
  route_segments = sorted([item for item in iter_qlog_files(download_root, host) if item.route == route], key=lambda item: item.segment)
  if not route_segments:
    raise RuntimeError(f"No qlogs found for route {route}")
  samples = load_samples(route_segments)
  for sample in samples:
    if sample.mono_time_s is not None:
      return sample.mono_time_s - sample.t
  raise RuntimeError(f"No mono timestamps found in route {route}")


def actual_event_is_harsh(event: dict[str, Any]) -> bool:
  hard_duration = _float_or_default(event.get("hard_decel_duration_s"))
  min_a = _float_or_default(event.get("min_a_ego_mps2"))
  return hard_duration > 0.0 or min_a <= -1.50


def actual_event_is_leapfrog(event: dict[str, Any]) -> bool:
  return (
    bool(event.get("reaccel_before_hold", False))
    or bool(event.get("stop_signal_dropped_before_hold", False))
    or bool(event.get("left_stopping_state_before_hold", False))
    or _float_or_default(event.get("speed_rebound_after_hold_mps")) >= 0.15
    or _float_or_default(event.get("speed_rebound_while_should_stop_mps")) >= 0.05
  )


def shadow_decisions_for_event(
  event: dict[str, Any],
  decisions: list[ShadowDecision],
  *,
  first_mono_time_s: float,
  pre_window_s: float,
  post_window_s: float,
) -> list[ShadowDecision]:
  start_s = first_mono_time_s + _float_or_default(event.get("start_time_s"))
  hold_s = first_mono_time_s + _float_or_default(event.get("stop_hold_time_s"), _float_or_default(event.get("stop_time_s")))
  return [item for item in decisions if start_s - pre_window_s <= item.mono_time_s <= hold_s + post_window_s]


def summarize_event_shadow(
  event: dict[str, Any],
  decisions: list[ShadowDecision],
  *,
  first_mono_time_s: float,
  min_command_relief_mps2: float,
) -> EventShadowSummary:
  actual_harsh = actual_event_is_harsh(event)
  actual_leapfrog = actual_event_is_leapfrog(event)
  accepted = [item for item in decisions if item.accepted]
  accepted_safe = [item for item in accepted if item.accepted_safe]
  accepted_unsafe = [item for item in accepted if item.selected_harsh or item.selected_leapfrog]
  accepted_relief = [
    item for item in accepted_safe
    if item.command_relief_mps2 is not None and item.command_relief_mps2 >= min_command_relief_mps2
  ]
  best_score_delta = min((item.score_delta for item in decisions), default=None)
  max_command_relief = max((item.command_relief_mps2 for item in decisions if item.command_relief_mps2 is not None), default=None)
  start_s = first_mono_time_s + _float_or_default(event.get("start_time_s"))
  offsets = [item.mono_time_s - start_s for item in decisions]

  if not decisions:
    verdict = "missing_harsh_shadow_data" if actual_harsh else "missing_shadow_data"
  elif actual_harsh and accepted_relief and not accepted_unsafe:
    verdict = "actionable_soften_candidate"
  elif actual_harsh and accepted_relief and accepted_unsafe:
    verdict = "mixed_shadow_signal"
  elif accepted_unsafe:
    verdict = "unsafe_shadow_candidate"
  elif actual_harsh and not accepted_relief:
    verdict = "missed_harsh_stop"
  elif accepted_relief:
    verdict = "comfort_candidate"
  elif accepted:
    verdict = "accepted_without_relief"
  else:
    verdict = "no_change_signal"

  return EventShadowSummary(
    event_id=int(event.get("event_id", 0)),
    segment=int(event.get("start_segment", -1)),
    actual_harsh=actual_harsh,
    actual_leapfrog=actual_leapfrog,
    shadow_count=len(decisions),
    accepted_count=len(accepted),
    accepted_safe_count=len(accepted_safe),
    accepted_unsafe_count=len(accepted_unsafe),
    accepted_relief_count=len(accepted_relief),
    observer_scope_counts=dict(Counter(item.observer_scope for item in decisions)),
    profile_counts=dict(Counter(item.profile for item in decisions)),
    accepted_profile_counts=dict(Counter(item.profile for item in accepted)),
    best_score_delta=best_score_delta,
    max_command_relief_mps2=max_command_relief,
    first_shadow_time_offset_s=min(offsets) if offsets else None,
    last_shadow_time_offset_s=max(offsets) if offsets else None,
    predicted_harsh_count=sum(1 for item in decisions if item.selected_harsh),
    predicted_leapfrog_count=sum(1 for item in decisions if item.selected_leapfrog),
    verdict=verdict,
  )


def route_shadow_verdict(event_summaries: list[EventShadowSummary]) -> str:
  if not event_summaries:
    return "no_stop_events"
  if all(item.shadow_count == 0 for item in event_summaries):
    return "not_usable_no_shadow_data"
  actionable = sum(1 for item in event_summaries if item.verdict == "actionable_soften_candidate")
  mixed = sum(1 for item in event_summaries if item.verdict == "mixed_shadow_signal")
  unsafe = sum(1 for item in event_summaries if item.accepted_unsafe_count > 0)
  missing_harsh = sum(1 for item in event_summaries if item.verdict == "missing_harsh_shadow_data")
  missed_harsh = sum(1 for item in event_summaries if item.verdict == "missed_harsh_stop")
  if missing_harsh > 0 and unsafe > 0:
    return "not_ready_scope_and_safety_gaps"
  if missing_harsh > 0 and actionable == 0:
    return "low_value_missing_harsh_shadow_data"
  if actionable > 0 and unsafe == 0:
    return "valuable_promote_narrow_candidate"
  if actionable > 0 or mixed > 0:
    return "valuable_but_needs_stronger_guards"
  if unsafe > 0:
    return "not_ready_unsafe_predictions"
  if missed_harsh > 0:
    return "low_value_misses_harsh_stops"
  return "observability_only"


def format_metric(value: float | None, digits: int = 3) -> str:
  return "-" if value is None else f"{value:.{digits}f}"


def build_markdown_report(report: dict[str, Any]) -> str:
  route = report["route"]
  route_summary = report["route_summary"]
  lines = [
    f"# Stopping Shadow Review: `{route}`",
    "",
    f"- Generated: `{report['generated_at']}`",
    f"- Source stop summary: `{report['summary_json']}`",
    f"- Segments requested: `{', '.join(str(item) for item in report['segments_requested'])}`",
    f"- Rlog segments parsed: `{', '.join(str(item) for item in report['segments_with_rlogs'])}`",
    f"- Shadow decisions parsed: `{route_summary['shadow_decisions_total']}`",
    f"- Events with shadow data: `{route_summary['events_with_shadow']}/{route_summary['event_count']}`",
    f"- Route verdict: `{route_summary['verdict']}`",
    "",
    "## Event Shadow Table",
    "",
    "|Event|Seg|ActualHarsh|ActualLeapfrog|Shadow|Accepted|Safe|Unsafe|Relief|BestDelta|MaxRelief|PredLeapfrog|Verdict|Scopes|Profiles|",
    "|---:|---:|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---|---|---|",
  ]
  for item in report["event_summaries"]:
    scopes = ", ".join(f"{scope or 'unknown'}:{count}" for scope, count in sorted(item["observer_scope_counts"].items())) or "-"
    profiles = ", ".join(f"{profile}:{count}" for profile, count in sorted(item["profile_counts"].items())) or "-"
    lines.append(
      "|"
      f"{item['event_id']}|"
      f"{item['segment']}|"
      f"{'yes' if item['actual_harsh'] else 'no'}|"
      f"{'yes' if item['actual_leapfrog'] else 'no'}|"
      f"{item['shadow_count']}|"
      f"{item['accepted_count']}|"
      f"{item['accepted_safe_count']}|"
      f"{item['accepted_unsafe_count']}|"
      f"{item['accepted_relief_count']}|"
      f"{format_metric(item['best_score_delta'])}|"
      f"{format_metric(item['max_command_relief_mps2'])}|"
      f"{item['predicted_leapfrog_count']}|"
      f"{item['verdict']}|"
      f"{scopes}|"
      f"{profiles}|"
    )
  lines.append("")
  lines.append("## Interpretation")
  lines.append("")
  lines.append(
    "- `actionable_soften_candidate` means an actually harsh event had accepted, non-harsh, non-leapfrog shadow decisions that would have relaxed braking."
  )
  lines.append(
    "- `mixed_shadow_signal` means the oracle saw possible improvement but also accepted at least one candidate that predicted harsh or leapfrog behavior."
  )
  lines.append("- `missing_harsh_shadow_data` means the actual stop was harsh but no runtime shadow decision was logged for that event window.")
  lines.append("- `missed_harsh_stop` means the actual event was harsh but shadow mode did not produce a safe relief candidate in the event window.")
  lines.append("- `unsafe_shadow_candidate` means shadow accepted a candidate that its own rollout predicted as harsh or leapfrog.")
  return "\n".join(lines) + "\n"


def parse_args() -> argparse.Namespace:
  parser = argparse.ArgumentParser(description="Attach stopping-shadow rlog decisions to a stopping summary")
  parser.add_argument("--summary-json", required=True, help="summary.json from analyze_stopping_behavior.py")
  parser.add_argument("--host", default=DEFAULT_HOST, help=f"SSH/cache host. Default: {DEFAULT_HOST}")
  parser.add_argument("--download-root", default=str(DEFAULT_DOWNLOAD_ROOT), help=f"Local route-sync root. Default: {DEFAULT_DOWNLOAD_ROOT}")
  parser.add_argument("--download-missing-rlogs", action="store_true", help="Download rlog.zst for stop-event segments before analysis")
  parser.add_argument("--connect-timeout", type=int, default=8, help="SSH/SCP connect timeout in seconds")
  parser.add_argument("--pre-window-s", type=float, default=DEFAULT_PRE_WINDOW_S, help="Seconds before event start to attach shadow logs")
  parser.add_argument("--post-window-s", type=float, default=DEFAULT_POST_WINDOW_S, help="Seconds after hold to attach shadow logs")
  parser.add_argument("--min-command-relief", type=float, default=DEFAULT_MIN_COMMAND_RELIEF_MPS2,
                      help="Minimum first_output_accel - actual_output_accel to count as useful brake relief")
  parser.add_argument("--output-json", default=None, help="Output JSON path. Default: shadow_summary.json beside input summary")
  parser.add_argument("--output-md", default=None, help="Output Markdown path. Default: shadow_summary.md beside input summary")
  return parser.parse_args()


def main() -> int:
  args = parse_args()
  summary_path = Path(args.summary_json).expanduser()
  summary = json.loads(summary_path.read_text())
  route = str(summary["route"])
  events = list(summary.get("events", []))
  segments = event_segments(events)
  download_root = Path(args.download_root).expanduser()

  download_report: dict[str, Any] | None = None
  if args.download_missing_rlogs and segments:
    download_report = download_missing_rlogs(route, segments, host=args.host, download_root=download_root, connect_timeout=args.connect_timeout)

  rlogs = [item for item in iter_rlog_files(download_root, args.host) if item.route == route and item.segment in set(segments)]
  first_mono_time_s = load_first_mono_time_s(download_root, args.host, route)
  decisions = load_shadow_decisions(rlogs)

  event_summaries: list[EventShadowSummary] = []
  for event in events:
    event_decisions = shadow_decisions_for_event(
      event,
      decisions,
      first_mono_time_s=first_mono_time_s,
      pre_window_s=args.pre_window_s,
      post_window_s=args.post_window_s,
    )
    event_summaries.append(
      summarize_event_shadow(
        event,
        event_decisions,
        first_mono_time_s=first_mono_time_s,
        min_command_relief_mps2=args.min_command_relief,
      )
    )

  route_summary = {
    "event_count": len(events),
    "segments_requested_count": len(segments),
    "rlog_segments_found": len({item.segment for item in rlogs}),
    "shadow_decisions_total": len(decisions),
    "events_with_shadow": sum(1 for item in event_summaries if item.shadow_count > 0),
    "actual_harsh_events": sum(1 for item in event_summaries if item.actual_harsh),
    "actual_leapfrog_events": sum(1 for item in event_summaries if item.actual_leapfrog),
    "harsh_events_with_shadow": sum(1 for item in event_summaries if item.actual_harsh and item.shadow_count > 0),
    "harsh_events_missing_shadow": sum(1 for item in event_summaries if item.verdict == "missing_harsh_shadow_data"),
    "harsh_events_with_relief_candidate": sum(1 for item in event_summaries if item.actual_harsh and item.accepted_relief_count > 0),
    "actionable_soften_candidates": sum(1 for item in event_summaries if item.verdict == "actionable_soften_candidate"),
    "mixed_shadow_signal_events": sum(1 for item in event_summaries if item.verdict == "mixed_shadow_signal"),
    "unsafe_shadow_candidate_events": sum(1 for item in event_summaries if item.accepted_unsafe_count > 0),
    "missed_harsh_events": sum(1 for item in event_summaries if item.verdict == "missed_harsh_stop"),
    "verdict": route_shadow_verdict(event_summaries),
  }
  report = {
    "generated_at": utc_now_iso(),
    "summary_json": str(summary_path),
    "route": route,
    "host": args.host,
    "download_root": str(download_root),
    "download_report": download_report,
    "segments_requested": segments,
    "segments_with_rlogs": sorted({item.segment for item in rlogs}),
    "route_summary": route_summary,
    "event_summaries": [asdict(item) for item in event_summaries],
  }

  output_json = Path(args.output_json).expanduser() if args.output_json else summary_path.with_name("shadow_summary.json")
  output_md = Path(args.output_md).expanduser() if args.output_md else summary_path.with_name("shadow_summary.md")
  output_json.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")
  output_md.write_text(build_markdown_report(report))

  print(f"[shadow] route={route}")
  print(f"[shadow] events={len(events)} rlog_segments={route_summary['rlog_segments_found']} shadow_decisions={len(decisions)}")
  print(f"[shadow] events_with_shadow={route_summary['events_with_shadow']}/{len(events)} verdict={route_summary['verdict']}")
  print(f"[shadow] summary_json={output_json}")
  print(f"[shadow] summary_md={output_md}")
  return 0


if __name__ == "__main__":
  raise SystemExit(main())
