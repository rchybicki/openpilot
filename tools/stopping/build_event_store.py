#!/usr/bin/env python3
"""Full-corpus stop-event store builder (spec 7.1) -- rlog-first, qlog fallback, stable keys,
dual-rate metric blocks.

Source: locally synced logs under the route_sync download root, preferring rlog per segment
(100 Hz car signals + logMessage; the qlog plan stream is 2 Hz-aliased) with qlog fallback tagged
`rate_class='qlog10'`. Detection + metric definitions are REUSED from analyze_stopping_behavior
(hybrid detector, KEEP verdict honored, spec 7.1) -- the analyzer module imports capnp/plotly
lazily, so this module and its tests stay importable without them.

Record schema (one JSONL line per stop event, ~/.comma/stopping_behavior/event_store/events.jsonl):
  key: {route, seg, hold_mono_ns}    stable identity (fixes the positional-event-id fragility)
  detector, schema_version, signals_version, telemetry_version, controller_commit
  accel_cmd_source: 'carControl' | 'carOutput'   (spec 4.3 / F9; consumed by fit_plant_model.py)
  entry: {v_approach, lead_entry_gap_m, explicit_target, isd_m}
  metrics_100hz + metrics_10hz_compat            (dual-rate blocks are MANDATORY, spec 7.1: the
    10 Hz-compat block keeps the historical 2,097-event corpus and the 0-leapfrog baseline
    comparable forever; on qlog-sourced routes both blocks are 10 Hz and rate_class says so)
  analyzer_event: the full analyzer StopEvent row (native rate) for downstream tools
  trace_ref: events/<route>__<seg>__<hold_mono_ns>.npz with arrays t, v_ego, a_ego, accel_cmd
    (version-correct command stream), enabled, brake_pressed, should_stop (+ lead/target extras
    consumed by sim_replay.py)

Selective device pulls: --fetch-missing-rlogs downloads rlog.zst for event segments over SSH
(scp, comma -> commawifi fallback; machinery lifted from analyze_stopping_shadow.py:233-265
without editing that file -- it is WP8-deletable), bounded by --fetch-limit.
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
from dataclasses import asdict
from datetime import UTC, datetime
from pathlib import Path
from typing import Any

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
  sys.path.insert(0, str(REPO_ROOT))

from openpilot.tools.route_sync.common import CANONICAL_REMOTE_ROOT, DEFAULT_DOWNLOAD_ROOT, local_path_for
from openpilot.tools.stopping import analyze_stopping_behavior as asb
from openpilot.tools.stopping.scoring_config import SCORING_CONFIG

DEFAULT_STORE_DIR = Path.home() / ".comma" / "stopping_behavior" / "event_store"
DEFAULT_ANALYSIS_ROOT = Path.home() / ".comma" / "stopping_behavior" / "analysis"
SCHEMA_VERSION = 1
DETECTOR = "hybrid"
DEFAULT_MIN_ENTRY_SPEED = 0.5   # corpus-scan calibration (find_stop_events_corpus.py:46-49)
# Pre-window > the 4 s telemetry-v1 engagement exclusion (spec 7.5(i)): a trace that STARTS
# enabled is conservatively treated as an engagement rising edge by fit_plant_model, so the
# window must carry enough pre-event context that long-lived engagements keep usable rows.
TRACE_PRE_WINDOW_S = 8.0
TRACE_POST_WINDOW_S = 4.0
COMPAT_DT_S = 0.10
DEFAULT_HOST = "comma"
FALLBACK_HOST = "commawifi"


# --- metric blocks ----------------------------------------------------------------------------------

def metrics_block(event: Any, hold_acq_peak_cmd_jerk: float | None = None) -> dict[str, float | None]:
  """Spec-7.1 metric block from an analyzer StopEvent (same definitions, stable names).
  `hold_acq_peak_cmd_jerk` is the NON-gating diagnostic computed per rate by
  hold_acquisition_peak_cmd_jerk() (scoring_config.DiagnosticMetrics defines the window)."""
  rebounds = [event.speed_rebound_while_stop_signal_mps, event.speed_rebound_while_should_stop_mps]
  rebounds = [r for r in rebounds if r is not None]
  return {
    "end_stop_jerk": event.end_stop_jerk_mps3,
    "end_stop_accel_step": event.end_stop_accel_step_mps2,
    "min_a_ego": event.min_a_ego_mps2,
    "max_cmd_jerk": event.end_stop_cmd_jerk_mps3,
    "rollout_from_2mps_m": event.rollout_distance_from_2mps_m,
    "final_lead_gap_m": event.lead_distance_hold_m,
    "rebound_mps": max(rebounds) if rebounds else None,
    "unexpected_accel": event.should_stop_unexpected_accel_mps2,
    "hard_decel_duration_s": event.hard_decel_duration_s,
    "time_to_standstill_s": float(event.stop_time_s - event.start_time_s),
    "hold_acq_peak_cmd_jerk": hold_acq_peak_cmd_jerk,
  }


def hold_acquisition_peak_cmd_jerk(samples: list, start_idx: int, hold_idx: int) -> float | None:
  """NON-gating diagnostic (scoring_config.DiagnosticMetrics; this change, driveway route
  00001702--dcdc5c3eea--0): peak |d(accel_cmd)/dt| in the window [enabled rising edge with
  v_ego < hold_acq_edge_v_max, +hold_acq_window_s]. Low-speed engagement edges are
  engage-at-standstill / stop-and-go re-engage hold acquisitions; events without one (normal
  driving stops engage at speed) report None. Edges are scanned from the trace pre-window up to
  the hold sample; the jerk window itself may extend past the hold."""
  diag = SCORING_CONFIG.diagnostics
  if not samples:
    return None
  scan_t0 = samples[start_idx].t - TRACE_PRE_WINDOW_S
  peak: float | None = None
  for idx in range(1, min(hold_idx, len(samples) - 1) + 1):
    edge = samples[idx]
    if edge.t < scan_t0:
      continue
    if not (edge.enabled and not samples[idx - 1].enabled and edge.v_ego < diag.hold_acq_edge_v_max):
      continue
    window = [s for s in samples[idx:] if s.t <= edge.t + diag.hold_acq_window_s and s.accel_cmd is not None]
    for prev, cur in zip(window, window[1:], strict=False):
      dt = cur.t - prev.t
      if dt <= 1e-6:
        continue
      slope = abs((cur.accel_cmd - prev.accel_cmd) / dt)
      peak = slope if peak is None else max(peak, slope)
  return peak


def decimate_samples(samples: list, dt: float = COMPAT_DT_S) -> list:
  """Nearest-sample decimation onto a dt grid (10 Hz-compat block, spec 7.2: 10 Hz jerk
  systematically understates true jerk -- the compat block keeps history comparable)."""
  if not samples:
    return []
  out = [samples[0]]
  next_t = samples[0].t + dt
  for sample in samples[1:]:
    if sample.t >= next_t - 1e-9:
      out.append(sample)
      next_t += dt * max(1, int((sample.t - next_t) // dt) + 1)
  return out


def nearest_index(samples: list, t: float) -> int:
  times = [s.t for s in samples]
  idx = int(np.searchsorted(times, t))
  if idx <= 0:
    return 0
  if idx >= len(samples):
    return len(samples) - 1
  return idx if abs(times[idx] - t) < abs(times[idx - 1] - t) else idx - 1


# --- ingest -----------------------------------------------------------------------------------------

def trace_arrays(samples: list, start_idx: int, hold_idx: int) -> dict[str, np.ndarray]:
  t0 = samples[start_idx].t - TRACE_PRE_WINDOW_S
  t1 = samples[hold_idx].t + TRACE_POST_WINDOW_S
  window = [s for s in samples if t0 <= s.t <= t1]

  def arr(getter, fill=np.nan):
    return np.asarray([fill if getter(s) is None else float(getter(s)) for s in window], dtype=float)

  return {
    "t": np.asarray([s.mono_time_s if s.mono_time_s is not None else s.t for s in window], dtype=float),
    "v_ego": arr(lambda s: s.v_ego),
    "a_ego": arr(lambda s: s.a_ego),
    "accel_cmd": arr(lambda s: s.accel_cmd),
    "enabled": np.asarray([bool(s.enabled) for s in window], dtype=np.uint8),
    "brake_pressed": np.asarray([bool(s.brake_pressed) for s in window], dtype=np.uint8),
    "should_stop": np.asarray([bool(s.should_stop) for s in window], dtype=np.uint8),
    # extras consumed by sim_replay.py (beyond the fit_plant_model minimum contract)
    "lead_status": np.asarray([bool(s.lead_status) for s in window], dtype=np.uint8),
    "lead_v": arr(lambda s: s.lead_v, fill=0.0),
    "lead_d_rel_m": arr(lambda s: s.lead_d_rel_m),
    "distance_to_stop_target_m": arr(lambda s: s.distance_to_stop_target_m, fill=-1.0),
    "a_target": arr(lambda s: s.a_target),
  }


def ingest_route_samples(route: str, samples: list, *, rate_class: str = "qlog10",
                         signals_version: int = 1, telemetry_version: int = 1,
                         accel_cmd_source: str = "carControl", controller_commit: str | None = None,
                         min_entry_speed: float = DEFAULT_MIN_ENTRY_SPEED,
                         detector_kwargs: dict[str, Any] | None = None,
                         segment_log_kinds: dict[int, str] | None = None) -> list[dict[str, Any]]:
  """Detect stop events on a Sample stream and build store records + trace arrays.

  Pure function over Samples (no log decoding) so tests drive it with synthetic streams.
  Returns records with an extra '_trace' key holding the npz arrays (stripped on write).
  rate_class is the route-level default; segment_log_kinds (seg -> 'rlog'|'qlog') upgrades
  individual events to 'rlog100' when every segment covering the event window is rlog-sourced
  (targeted rlog pulls cover event segments first, spec 7.1).
  """
  # rate-aware hybrid merge window: the historical 2-sample tolerance was calibrated at 10 Hz
  # (0.2 s); on 100 Hz rlog streams the same TIME window is ~20 samples
  dts = [samples[i].t - samples[i - 1].t for i in range(1, min(len(samples), 200))]
  median_dt = float(np.median(dts)) if dts else 0.1
  tolerance = max(2, int(round(0.2 / max(median_dt, 1e-3))))
  kwargs = dict(min_entry_speed=min_entry_speed, entry_lookback=8.0, standstill_speed=0.12,
                hold_time=0.5, max_stop_search=35.0, event_mode=DETECTOR,
                require_enabled_speed_events=True, hold_merge_tolerance=tolerance)
  kwargs.update(detector_kwargs or {})
  event_ranges = asb.find_stop_events_with_source(samples=samples, **kwargs)
  compat = decimate_samples(samples, COMPAT_DT_S)

  records: list[dict[str, Any]] = []
  for event_id, (start_idx, stop_idx, hold_idx, approach_speed, event_source) in enumerate(event_ranges, start=1):
    native_event = asb.compute_event(event_id, event_source, samples, start_idx, stop_idx, hold_idx, approach_speed, "")
    # 10 Hz-compat block: identical definitions on the decimated stream (index-mapped by time)
    c_start = nearest_index(compat, samples[start_idx].t)
    c_stop = nearest_index(compat, samples[stop_idx].t)
    c_hold = nearest_index(compat, samples[hold_idx].t)
    hold_acq_native = hold_acquisition_peak_cmd_jerk(samples, start_idx, hold_idx)
    if c_hold <= c_start:
      compat_event = native_event
      hold_acq_compat = hold_acq_native
    else:
      compat_event = asb.compute_event(event_id, event_source, compat, c_start, max(c_stop, c_start + 1), c_hold, approach_speed, "")
      hold_acq_compat = hold_acquisition_peak_cmd_jerk(compat, c_start, c_hold)

    hold_sample = samples[hold_idx]
    hold_mono_ns = int(round((hold_sample.mono_time_s if hold_sample.mono_time_s is not None else hold_sample.t) * 1e9))
    window = samples[start_idx:hold_idx + 1]
    explicit_target = any(s.distance_to_stop_target_m is not None and s.distance_to_stop_target_m > 0.0 for s in window)
    isd_m = max((s.increased_stopped_distance_m for s in window), default=0.0)
    seg = int(hold_sample.segment)
    key = {"route": route, "seg": seg, "hold_mono_ns": hold_mono_ns}
    event_rate_class = rate_class
    if segment_log_kinds is not None:
      window_segments = {int(s.segment) for s in window}
      if window_segments and all(segment_log_kinds.get(s) == "rlog" for s in window_segments):
        event_rate_class = "rlog100"
    records.append({
      "key": key,
      "detector": DETECTOR,
      "schema_version": SCHEMA_VERSION,
      "signals_version": int(signals_version),
      "telemetry_version": int(telemetry_version),
      "accel_cmd_source": accel_cmd_source,
      "controller_commit": controller_commit,
      "rate_class": event_rate_class,
      "event_id": event_id,
      "event_source": event_source,
      "entry": {
        "v_approach": float(approach_speed),
        "lead_entry_gap_m": native_event.lead_distance_stop_entry_m,
        "explicit_target": bool(explicit_target),
        "isd_m": float(isd_m),
      },
      "metrics_100hz": metrics_block(native_event, hold_acq_peak_cmd_jerk=hold_acq_native),
      "metrics_10hz_compat": metrics_block(compat_event, hold_acq_peak_cmd_jerk=hold_acq_compat),
      "analyzer_event": asdict(native_event),
      "trace_ref": f"events/{route}__{seg}__{hold_mono_ns}.npz",
      "_trace": trace_arrays(samples, start_idx, hold_idx),
    })
  return records


def write_store(store_dir: Path, records: list[dict[str, Any]], merge: bool = True) -> dict[str, int]:
  """Write/merge records into events.jsonl + per-event npz traces; new records win on key clash."""
  store_dir.mkdir(parents=True, exist_ok=True)
  (store_dir / "events").mkdir(parents=True, exist_ok=True)
  events_path = store_dir / "events.jsonl"

  existing: dict[tuple, dict[str, Any]] = {}
  if merge and events_path.is_file():
    for line in events_path.read_text().splitlines():
      line = line.strip()
      if not line:
        continue
      record = json.loads(line)
      key = record.get("key", {})
      existing[(key.get("route"), key.get("seg"), key.get("hold_mono_ns"))] = record

  added = replaced = 0
  for record in records:
    trace = record.pop("_trace", None)
    key = record["key"]
    key_tuple = (key["route"], key["seg"], key["hold_mono_ns"])
    if key_tuple in existing:
      replaced += 1
    else:
      added += 1
    existing[key_tuple] = record
    if trace is not None:
      np.savez_compressed(store_dir / record["trace_ref"], **trace)

  ordered = sorted(existing.values(), key=lambda r: (r["key"]["route"], r["key"]["seg"], r["key"]["hold_mono_ns"]))
  with open(events_path, "w") as f:
    for record in ordered:
      f.write(json.dumps(record, sort_keys=True) + "\n")
  return {"total": len(ordered), "added": added, "replaced": replaced}


# --- device pulls (lifted from analyze_stopping_shadow.py:233-265 without editing the donor) --------

def event_segments_for_route(records: list[dict[str, Any]]) -> list[int]:
  segments: set[int] = set()
  for record in records:
    seg = int(record["key"]["seg"])
    segments.update((max(seg - 1, 0), seg))
  return sorted(segments)


def fetch_missing_rlogs(route: str, segments: list[int], *, host: str, download_root: Path,
                        connect_timeout: int = 10, limit: int = 0) -> dict[str, Any]:
  """scp rlog.zst for the given segments (skip ones already local); comma<->commawifi fallback."""
  from openpilot.tools.route_sync.refresh_routes import download_file  # lazy: ssh machinery

  downloaded: list[str] = []
  skipped: list[int] = []
  failures: list[dict[str, Any]] = []
  fallback = FALLBACK_HOST if host == DEFAULT_HOST else DEFAULT_HOST
  for segment in segments:
    if limit > 0 and len(downloaded) >= limit:
      break
    seg_dir = local_path_for(download_root, host, f"{CANONICAL_REMOTE_ROOT}/{route}--{segment}").parent / f"{route}--{segment}"
    if any((seg_dir / name).is_file() for name in asb.RLOG_FILE_PATTERNS):
      skipped.append(segment)
      continue
    remote_path = f"{CANONICAL_REMOTE_ROOT}/{route}--{segment}/rlog.zst"
    local_path = local_path_for(download_root, host, remote_path)
    try:
      download_file(host, remote_path, local_path, connect_timeout)
      downloaded.append(str(local_path))
      continue
    except Exception as first_exc:
      first_error = str(first_exc).splitlines()[0] if str(first_exc) else type(first_exc).__name__
    try:
      download_file(fallback, remote_path, local_path, connect_timeout)
      downloaded.append(str(local_path))
    except Exception as second_exc:
      failures.append({"segment": segment, "error": f"{first_error}; fallback: {second_exc}"})
  return {"downloaded": downloaded, "skipped_existing": skipped, "failures": failures}


# --- route discovery --------------------------------------------------------------------------------

def routes_from_summaries(analysis_root: Path, limit: int = 0) -> list[str]:
  """Routes referenced by local analysis summaries, newest first (ingest-from-summaries source)."""
  rows: list[tuple[float, str]] = []
  seen: set[str] = set()
  for summary_path in analysis_root.rglob("summary.json"):
    try:
      payload = json.loads(summary_path.read_text())
    except (OSError, ValueError):
      continue
    if not isinstance(payload, dict):
      continue
    candidates = [str(payload.get("route", ""))]
    if isinstance(payload.get("routes"), list):
      candidates.extend(str(r.get("route", "")) for r in payload["routes"] if isinstance(r, dict))
    mtime = summary_path.stat().st_mtime
    for route in candidates:
      if route and route not in seen:
        seen.add(route)
        rows.append((mtime, route))
  rows.sort(reverse=True)
  routes = [route for _, route in rows]
  return routes[:limit] if limit > 0 else routes


def git_short_head() -> str | None:
  try:
    out = subprocess.run(["git", "-C", str(REPO_ROOT), "rev-parse", "--short", "HEAD"],
                         capture_output=True, text=True, timeout=10)
    return out.stdout.strip() or None
  except Exception:
    return None


def ingest_route_from_logs(route: str, download_root: Path, host: str, args: argparse.Namespace) -> list[dict[str, Any]]:
  try:
    all_segments = asb.iter_qlog_files(download_root, host, prefer_rlog=True)
  except (FileNotFoundError, RuntimeError):
    return []
  route_segments = sorted([s for s in all_segments if s.route == route], key=lambda s: s.segment)
  if not route_segments:
    return []
  if args.max_segments > 0:
    route_segments = route_segments[-args.max_segments:]
  accel_cmd_source = asb.resolve_accel_cmd_source(args.accel_cmd_source, args.telemetry_version)
  stats: dict[str, Any] = {}
  samples = asb.load_samples(route_segments, accel_cmd_source=accel_cmd_source, stats=stats)
  if not samples:
    return []
  return ingest_route_samples(
    route, samples,
    rate_class=asb.rate_class_for_segments(route_segments),
    segment_log_kinds={int(s.segment): s.log_kind for s in route_segments},
    signals_version=args.signals_version,
    telemetry_version=args.telemetry_version,
    accel_cmd_source=accel_cmd_source,
    controller_commit=stats.get("git_commit") or git_short_head(),
    min_entry_speed=args.min_entry_speed,
  )


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
  parser = argparse.ArgumentParser(description="Build the stop-event store from locally synced logs (spec 7.1)")
  parser.add_argument("--download-root", default=str(DEFAULT_DOWNLOAD_ROOT))
  parser.add_argument("--host", default=DEFAULT_HOST)
  parser.add_argument("--store-dir", default=str(DEFAULT_STORE_DIR))
  parser.add_argument("--route", action="append", default=[], help="Route to ingest (repeatable)")
  parser.add_argument("--route-file", default=None, help="File with one route per line (# comments ok)")
  parser.add_argument("--from-summaries", action="store_true",
                      help=f"Ingest every route referenced by analysis summaries under {DEFAULT_ANALYSIS_ROOT}")
  parser.add_argument("--analysis-root", default=str(DEFAULT_ANALYSIS_ROOT))
  parser.add_argument("--max-routes", type=int, default=0, help="Cap routes ingested this run (0 = all)")
  parser.add_argument("--max-segments", type=int, default=0, help="Cap segments per route (newest kept; 0 = all)")
  parser.add_argument("--min-entry-speed", type=float, default=DEFAULT_MIN_ENTRY_SPEED)
  parser.add_argument("--signals-version", type=int, default=1)
  parser.add_argument("--telemetry-version", type=int, default=1)
  parser.add_argument("--accel-cmd-source", choices=asb.ACCEL_CMD_SOURCES, default="auto")
  parser.add_argument("--fetch-missing-rlogs", action="store_true",
                      help="scp rlog.zst for event segments missing local rlogs (comma->commawifi fallback), then re-ingest")
  parser.add_argument("--fetch-limit", type=int, default=0, help="Max rlog segments to download this run (0 = no cap)")
  parser.add_argument("--connect-timeout", type=int, default=10)
  parser.add_argument("--no-merge", action="store_true", help="Rewrite the store instead of merging by key")
  return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
  args = parse_args(argv)
  download_root = Path(args.download_root).expanduser()
  store_dir = Path(args.store_dir).expanduser()

  routes: list[str] = list(args.route)
  if args.route_file:
    routes.extend(line.strip() for line in Path(args.route_file).expanduser().read_text().splitlines()
                  if line.strip() and not line.strip().startswith("#"))
  if args.from_summaries:
    routes.extend(routes_from_summaries(Path(args.analysis_root).expanduser()))
  deduped: list[str] = []
  for route in routes:
    if route not in deduped:
      deduped.append(route)
  if args.max_routes > 0:
    deduped = deduped[:args.max_routes]
  if not deduped:
    print("[event-store] no routes selected (use --route/--route-file/--from-summaries)", file=sys.stderr)
    return 2

  all_records: list[dict[str, Any]] = []
  fetch_budget = args.fetch_limit
  fetch_report: list[dict[str, Any]] = []
  for route in deduped:
    records = ingest_route_from_logs(route, download_root, args.host, args)
    if args.fetch_missing_rlogs and records and (args.fetch_limit <= 0 or fetch_budget > 0):
      segments = event_segments_for_route(records)
      result = fetch_missing_rlogs(route, segments, host=args.host, download_root=download_root,
                                   connect_timeout=args.connect_timeout,
                                   limit=fetch_budget if args.fetch_limit > 0 else 0)
      fetch_report.append({"route": route, **result})
      if args.fetch_limit > 0:
        fetch_budget -= len(result["downloaded"])
      if result["downloaded"]:
        records = ingest_route_from_logs(route, download_root, args.host, args)  # re-ingest rlog-first
    print(f"[event-store] {route}: {len(records)} stop events", flush=True)
    all_records.extend(records)

  stats = write_store(store_dir, all_records, merge=not args.no_merge)
  manifest = {
    "generated_utc": datetime.now(UTC).replace(microsecond=0).isoformat(),
    "schema_version": SCHEMA_VERSION,
    "routes_ingested": deduped,
    "records_written": len(all_records),
    "store_totals": stats,
    "fetch_report": fetch_report,
  }
  (store_dir / "manifest.json").write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n")
  print(f"[event-store] store={store_dir} total_events={stats['total']} added={stats['added']} replaced={stats['replaced']}")
  if fetch_report:
    pulled = sum(len(r["downloaded"]) for r in fetch_report)
    failed = sum(len(r["failures"]) for r in fetch_report)
    print(f"[event-store] rlogs fetched={pulled} failures={failed}")
  return 0 if stats["total"] > 0 else 2


if __name__ == "__main__":
  raise SystemExit(main())
