#!/usr/bin/env python3
"""Run a full stopping-behavior data cycle: settings snapshot, route refresh, and worklog append."""

from __future__ import annotations

import argparse
import bz2
import json
import os
import shutil
import subprocess
import sys
from datetime import UTC, datetime
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
  sys.path.insert(0, str(REPO_ROOT))

from openpilot.tools.route_sync.common import DEFAULT_DOWNLOAD_ROOT, DEFAULT_REPORT_DIR, DEFAULT_STATE_FILE, host_download_root, segment_has_active_lock
from openpilot.tools.stopping.log_schema_helpers import controls_state_enabled, selfdrive_state_engaged

DEFAULT_SETTINGS_DIR = Path.home() / ".comma" / "stopping_behavior" / "settings"
DEFAULT_ANALYSIS_ROOT = Path.home() / ".comma" / "stopping_behavior" / "analysis"
DEFAULT_MODEL_DIR = Path.home() / ".comma" / "stopping_behavior" / "models"
DEFAULT_WORKLOG = Path("docs/stopping_behavior_worklog.md")
ROUTE_REFRESH_SCRIPT = REPO_ROOT / "tools" / "route_sync" / "refresh_routes.py"
REEXEC_ENV_VAR = "STOPPING_CYCLE_REEXEC_READY"
RC_INSUFFICIENT_INPUTS = 2
RC_ENVIRONMENT = 3
ZSTD_MAGIC = b"\x28\xb5\x2f\xfd"
QLOG_FILE_PATTERNS = ("qlog", "qlog.bz2", "qlog.zst")


def qlog_path_priority(path: Path) -> int:
  name = path.name
  if name == "qlog":
    return 0
  if name == "qlog.bz2":
    return 1
  if name == "qlog.zst":
    return 2
  return 99


def utc_stamp() -> str:
  return datetime.now(UTC).strftime("%Y%m%dT%H%M%SZ")


def run_cmd(cmd: list[str], label: str) -> int:
  print(f"[cycle] running {label}: {' '.join(cmd)}", flush=True)
  result = subprocess.run(cmd)
  return result.returncode


def merge_rc(current: int, new_rc: int) -> int:
  """Combine stage exit codes so we can still append docs even when gates fail.

  Preference order:
  - keep 0 if all stages pass
  - prefer gate failures (1) over insufficient events (2)
  - prefer environment failures (3) over insufficient events (2)
  - preserve other non-standard non-zero codes (first seen) for debugging
  """
  if new_rc == 0:
    return current
  if current == 0:
    return new_rc
  if current == 1 or new_rc == 1:
    return 1
  if current == RC_ENVIRONMENT or new_rc == RC_ENVIRONMENT:
    return RC_ENVIRONMENT
  if current == 2 or new_rc == 2:
    return 2
  return current


def required_modules(args: argparse.Namespace) -> list[str]:
  modules: list[str] = []
  if args.analyze:
    modules.extend(["capnp", "numpy", "plotly"])
  if args.fit_model or args.run_model_gate or args.run_variant_benchmark:
    modules.append("capnp")
  if args.fit_model or args.run_measured_gate or args.run_model_gate or args.run_leapfrog_alignment or args.run_variant_benchmark:
    modules.append("numpy")
  return sorted(set(modules))


def probe_python_modules(executable: str, modules: list[str]) -> tuple[bool, list[str]]:
  if not modules:
    return True, []

  probe = (
    "import importlib.util, json, sys; "
    "mods = sys.argv[1:]; "
    "missing = [m for m in mods if importlib.util.find_spec(m) is None]; "
    "print(json.dumps(missing))"
  )
  try:
    result = subprocess.run([executable, "-c", probe, *modules], capture_output=True, text=True, check=False)
  except OSError:
    return False, modules

  if result.returncode != 0:
    return False, modules

  try:
    payload = json.loads(result.stdout.strip() or "[]")
  except json.JSONDecodeError:
    return False, modules

  missing = [str(item) for item in payload if str(item)]
  return len(missing) == 0, missing


def ensure_dependency_ready_interpreter(args: argparse.Namespace) -> int:
  modules = required_modules(args)
  if not modules:
    return 0

  current_ok, missing = probe_python_modules(sys.executable, modules)
  if current_ok:
    return 0

  if os.environ.get(REEXEC_ENV_VAR) != "1":
    current_resolved = Path(sys.executable).resolve()
    for name in ("python", "python3.11", "python3"):
      candidate = shutil.which(name)
      if not candidate:
        continue
      try:
        if Path(candidate).resolve() == current_resolved:
          continue
      except OSError:
        continue
      candidate_ok, _ = probe_python_modules(candidate, modules)
      if candidate_ok:
        print(f"[cycle] re-executing with dependency-ready interpreter: {candidate}", flush=True)
        env = os.environ.copy()
        env[REEXEC_ENV_VAR] = "1"
        os.execve(candidate, [candidate, str(Path(__file__).resolve()), *sys.argv[1:]], env)

  missing_text = ", ".join(missing or modules)
  print(
    f"[cycle] missing required python modules under {sys.executable}: {missing_text}. "
    + "Install dependencies or run with an interpreter that has the repo tooling packages.",
    file=sys.stderr,
  )
  return RC_ENVIRONMENT


def has_local_qlogs(host_download_dir: Path) -> bool:
  if not host_download_dir.exists():
    return False
  for pattern in QLOG_FILE_PATTERNS:
    for qlog_path in host_download_dir.rglob(pattern):
      if segment_has_active_lock(qlog_path.parent):
        continue
      return True
  return False


def read_repo_identity(repo_root: Path) -> tuple[str | None, str | None]:
  try:
    branch = subprocess.run(
      ["git", "branch", "--show-current"],
      cwd=repo_root,
      capture_output=True,
      text=True,
      check=False,
    ).stdout.strip() or None
    commit = subprocess.run(
      ["git", "rev-parse", "--short", "HEAD"],
      cwd=repo_root,
      capture_output=True,
      text=True,
      check=False,
    ).stdout.strip() or None
  except OSError:
    return None, None
  return branch, commit


def load_sync_report(report_path: Path) -> dict:
  try:
    payload = json.loads(report_path.read_text())
  except (OSError, json.JSONDecodeError):
    return {}
  return payload if isinstance(payload, dict) else {}


def pick_newest_route_from_sync_report(report: dict) -> str | None:
  downloaded = report.get("downloaded_files", [])
  if not isinstance(downloaded, list) or not downloaded:
    return None

  new_routes = report.get("new_routes", [])
  candidate_routes = {str(route) for route in new_routes} if isinstance(new_routes, list) and new_routes else set()

  per_route_mtime: dict[str, int] = {}
  for entry in downloaded:
    if not isinstance(entry, dict):
      continue
    route = str(entry.get("route", "")).strip()
    if not route:
      continue
    if candidate_routes and route not in candidate_routes:
      continue
    try:
      mtime = int(entry.get("mtime", 0))
    except (TypeError, ValueError):
      mtime = 0
    per_route_mtime[route] = max(per_route_mtime.get(route, 0), mtime)

  if not per_route_mtime:
    return None
  return max(per_route_mtime.items(), key=lambda item: (item[1], item[0]))[0]


def short_exception(exc: Exception) -> str:
  text = str(exc).strip()
  return text.splitlines()[0] if text else exc.__class__.__name__


def read_qlog_bytes(qlog_path: Path) -> bytes:
  data = qlog_path.read_bytes()
  if qlog_path.suffix == ".bz2" or data.startswith(b"BZh"):
    return bz2.decompress(data)
  if qlog_path.suffix == ".zst" or data.startswith(ZSTD_MAGIC):
    zstd = shutil.which("zstd")
    if not zstd:
      raise RuntimeError("zstd command not found for .zst qlog decode")
    result = subprocess.run([zstd, "-d", "-q", "-c", str(qlog_path)], capture_output=True, check=False)
    if result.returncode != 0:
      stderr = result.stderr.decode("utf-8", errors="ignore").strip() if result.stderr else "unknown zstd error"
      raise RuntimeError(stderr or "unknown zstd error")
    return result.stdout
  return data


def route_prefix_sort_key(route: str) -> tuple[int, int]:
  prefix = route.split("--", 1)[0] if "--" in route else route
  try:
    return 1, int(prefix, 16)
  except ValueError:
    return 0, 0


def path_mtime(path: Path) -> float:
  try:
    return path.stat().st_mtime
  except OSError:
    return 0.0


def pick_recent_routes_from_sync_report(report: dict) -> list[str]:
  downloaded = report.get("downloaded_files", [])
  if not isinstance(downloaded, list) or not downloaded:
    return []

  per_route_mtime: dict[str, int] = {}
  for entry in downloaded:
    if not isinstance(entry, dict):
      continue
    route = str(entry.get("route", "")).strip()
    if not route:
      continue
    try:
      mtime = int(entry.get("mtime", 0))
    except (TypeError, ValueError):
      mtime = 0
    per_route_mtime[route] = max(per_route_mtime.get(route, 0), mtime)

  if not per_route_mtime:
    return []

  preferred: list[str] = []
  new_routes = report.get("new_routes", [])
  if isinstance(new_routes, list):
    for route in new_routes:
      route_id = str(route).strip()
      if route_id and route_id in per_route_mtime:
        preferred.append(route_id)

  preferred_set = set(preferred)
  remaining = [route for route in per_route_mtime.keys() if route not in preferred_set]

  def sort_key(route: str) -> tuple[tuple[int, int], int, str]:
    return route_prefix_sort_key(route), per_route_mtime.get(route, 0), route

  ordered = sorted(preferred_set, key=sort_key, reverse=True)
  ordered.extend(sorted(remaining, key=sort_key, reverse=True))
  return ordered


def scan_qlog_vmax_mps(
  qlog_path: Path,
  *,
  min_vmax_mps: float,
  max_carstate_samples: int = 900,
  max_duration_s: float = 90.0,
) -> float | None:
  try:
    from cereal import log as capnp_log
  except ImportError as exc:
    print(f"[cycle] warning: cannot import cereal.log for route scan: {short_exception(exc)}", file=sys.stderr)
    return None

  try:
    if qlog_path.stat().st_size == 0:
      return None
  except OSError:
    return None

  try:
    data = read_qlog_bytes(qlog_path)
    reader = capnp_log.Event.read_multiple_bytes(data)
    vmax_mps = 0.0
    first_mono_time: int | None = None
    carstate_samples = 0

    for event in reader:
      try:
        if event.which() != "carState":
          continue
      except Exception:
        continue

      try:
        v_ego = abs(float(event.carState.vEgo))
        mono_time = int(event.logMonoTime)
      except Exception:
        continue

      if v_ego > vmax_mps:
        vmax_mps = v_ego
        if vmax_mps >= min_vmax_mps:
          return vmax_mps

      carstate_samples += 1
      if first_mono_time is None:
        first_mono_time = mono_time
      elif (mono_time - first_mono_time) >= int(max_duration_s * 1e9):
        break
      if carstate_samples >= max_carstate_samples:
        break

    return vmax_mps
  except Exception as exc:
    print(f"[cycle] warning: failed to scan {qlog_path}: {short_exception(exc)}", file=sys.stderr)
    return None


def scan_qlog_stop_signal_seen(
  qlog_path: Path,
  *,
  max_events: int = 6000,
  max_duration_s: float = 120.0,
) -> bool | None:
  try:
    from cereal import log as capnp_log
  except ImportError as exc:
    print(f"[cycle] warning: cannot import cereal.log for stop-signal scan: {short_exception(exc)}", file=sys.stderr)
    return None

  try:
    if qlog_path.stat().st_size == 0:
      return None
  except OSError:
    return None

  try:
    data = read_qlog_bytes(qlog_path)
    reader = capnp_log.Event.read_multiple_bytes(data)
    first_mono_time: int | None = None

    enabled = False
    long_state = "off"
    long_state_cmd = "off"
    should_stop = False

    for index, event in enumerate(reader):
      if index >= max_events:
        break

      try:
        which = event.which()
        mono_time = int(event.logMonoTime)
      except Exception:
        continue

      if first_mono_time is None:
        first_mono_time = mono_time
      elif (mono_time - first_mono_time) >= int(max_duration_s * 1e9):
        break

      if which == "controlsState":
        try:
          state = event.controlsState
          state_enabled = controls_state_enabled(state)
          if state_enabled is not None:
            enabled = state_enabled
          long_state = str(state.longControlState)
        except Exception:
          continue
      elif which == "selfdriveState":
        try:
          state_enabled = selfdrive_state_engaged(event.selfdriveState)
          if state_enabled is not None:
            enabled = state_enabled
        except Exception:
          continue
      elif which == "longitudinalPlan":
        try:
          should_stop = bool(event.longitudinalPlan.shouldStop)
        except Exception:
          continue
      elif which == "carControl":
        try:
          long_state_cmd = str(event.carControl.actuators.longControlState)
        except Exception:
          continue

      if enabled and (should_stop or long_state == "stopping" or long_state_cmd == "stopping"):
        return True

    return False
  except Exception as exc:
    print(f"[cycle] warning: failed to scan {qlog_path}: {short_exception(exc)}", file=sys.stderr)
    return None


def index_qlog_paths_by_route(download_root: Path, host: str, candidate_routes: set[str]) -> dict[str, list[tuple[int, float, Path]]]:
  host_root = host_download_root(download_root, host)
  if not host_root.exists():
    return {}

  per_route: dict[str, list[tuple[int, float, Path]]] = {}
  per_segment: dict[tuple[str, int], tuple[int, float, Path]] = {}
  for pattern in QLOG_FILE_PATTERNS:
    for qlog_path in host_root.rglob(pattern):
      if segment_has_active_lock(qlog_path.parent):
        continue
      segment_name = qlog_path.parent.name
      if "--" not in segment_name:
        continue
      route, suffix = segment_name.rsplit("--", 1)
      if route not in candidate_routes:
        continue
      try:
        segment = int(suffix)
      except ValueError:
        continue
      try:
        mtime = qlog_path.stat().st_mtime
      except OSError:
        continue
      key = (route, segment)
      existing = per_segment.get(key)
      candidate = (segment, mtime, qlog_path)
      if existing is None or qlog_path_priority(qlog_path) < qlog_path_priority(existing[2]):
        per_segment[key] = candidate

  for (route, _segment), item in per_segment.items():
    per_route.setdefault(route, []).append(item)

  for route in per_route:
    per_route[route].sort(key=lambda item: (item[0], item[1], str(item[2])))
  return per_route


def select_route_scan_paths(entries: list[tuple[int, float, Path]]) -> list[Path]:
  if not entries:
    return []

  paths = [item[2] for item in entries]
  if len(paths) <= 3:
    return paths

  indices = (0, len(paths) // 2, len(paths) - 1)
  selected: list[Path] = []
  seen: set[str] = set()
  for idx in indices:
    path = paths[idx]
    key = str(path)
    if key in seen:
      continue
    seen.add(key)
    selected.append(path)
  return selected


def pick_moving_route_for_analysis(
  report: dict,
  *,
  download_root: Path,
  host: str,
  min_route_vmax_mps: float,
  require_stop_signal: bool,
) -> str | None:
  if min_route_vmax_mps <= 0.0:
    return pick_newest_route_from_sync_report(report)

  candidates = pick_recent_routes_from_sync_report(report)
  if not candidates:
    return None

  qlog_index = index_qlog_paths_by_route(download_root, host, set(candidates))

  for route in candidates:
    scan_paths = select_route_scan_paths(qlog_index.get(route, []))
    if not scan_paths:
      continue

    vmax_mps = 0.0
    stop_signal_seen = not require_stop_signal
    scanned_any = False
    for qlog_path in scan_paths:
      scanned = scan_qlog_vmax_mps(qlog_path, min_vmax_mps=min_route_vmax_mps)
      if scanned is None:
        continue
      scanned_any = True
      vmax_mps = max(vmax_mps, scanned)
      if require_stop_signal and not stop_signal_seen:
        signal_scanned = scan_qlog_stop_signal_seen(qlog_path)
        if signal_scanned:
          stop_signal_seen = True

      if vmax_mps >= min_route_vmax_mps and stop_signal_seen:
        return route

    if scanned_any and vmax_mps >= min_route_vmax_mps and stop_signal_seen:
      return route

  return candidates[0]


def summary_has_event_source(summary_path: Path, event_source: str) -> bool:
  payload = read_summary_payload(summary_path)
  if payload is None:
    return False
  events = payload.get("events", [])
  if not isinstance(events, list):
    return False
  if event_source == "all":
    return len(events) > 0
  return any(isinstance(event, dict) and str(event.get("event_source", "")) == event_source for event in events)


def read_summary_payload(summary_path: Path) -> dict[str, object] | None:
  try:
    payload = json.loads(summary_path.read_text())
  except (OSError, json.JSONDecodeError):
    return None
  if not isinstance(payload, dict):
    return None
  return payload


def read_summary_route_and_mode(summary_path: Path) -> tuple[str, str]:
  default_route = summary_path.parent.parent.name if summary_path.parent.parent.name else str(summary_path)
  default_mode = ""

  payload = read_summary_payload(summary_path)
  if payload is None:
    return default_route, default_mode

  route = str(payload.get("route", default_route))
  mode = str(payload.get("event_mode", default_mode))
  return route, mode


def summary_mode_priority(mode: str) -> int:
  priorities = {
    "hybrid": 3,
    "speed_transition": 2,
    "engaged_signal": 1,
  }
  return priorities.get(mode, 0)


def discover_recent_summaries(analysis_root: Path, host: str, event_source: str, limit: int) -> list[Path]:
  host_root = analysis_root / host
  if not host_root.exists():
    return []

  discovered: list[Path] = []
  for summary_path in sorted(host_root.rglob("summary.json"), key=path_mtime, reverse=True):
    if not summary_has_event_source(summary_path, event_source):
      continue
    discovered.append(summary_path)
    if event_source != "all" and limit > 0 and len(discovered) >= limit:
      break

  if event_source != "all":
    return discovered

  selected_by_route: dict[str, tuple[Path, int, float]] = {}
  for summary_path in discovered:
    route, mode = read_summary_route_and_mode(summary_path)
    priority = summary_mode_priority(mode)
    summary_mtime = path_mtime(summary_path)
    existing = selected_by_route.get(route)
    if existing is None or priority > existing[1] or (priority == existing[1] and summary_mtime > existing[2]):
      selected_by_route[route] = (summary_path, priority, summary_mtime)

  deduped = sorted((item[0] for item in selected_by_route.values()), key=path_mtime, reverse=True)
  if limit > 0:
    return deduped[:limit]
  return deduped


def dedupe_paths(paths: list[Path]) -> list[Path]:
  seen: set[str] = set()
  unique: list[Path] = []
  for path in paths:
    key = str(path)
    if key in seen:
      continue
    seen.add(key)
    unique.append(path)
  return unique


def select_fit_summaries(
  *,
  explicit_summaries: list[Path],
  analysis_summary_json: Path,
  analysis_root: Path,
  host: str,
  event_source: str,
  recent_limit: int,
  excluded_routes: set[str] | None = None,
) -> list[Path]:
  excluded_routes = excluded_routes or set()
  fit_summaries: list[Path] = []
  if explicit_summaries:
    fit_summaries.extend(explicit_summaries)
  else:
    if (
      analysis_summary_json.exists()
      and summary_has_event_source(analysis_summary_json, event_source)
      and summary_route_id(analysis_summary_json) not in excluded_routes
    ):
      fit_summaries.append(analysis_summary_json)
    discovered = discover_recent_summaries(
      analysis_root=analysis_root,
      host=host,
      event_source=event_source,
      limit=0 if excluded_routes else recent_limit,
    )
    for summary_path in discovered:
      if summary_route_id(summary_path) in excluded_routes:
        continue
      fit_summaries.append(summary_path)
      if recent_limit > 0 and len(fit_summaries) >= recent_limit + (1 if analysis_summary_json in fit_summaries else 0):
        break

  return dedupe_paths([path for path in fit_summaries if path.exists() and summary_route_id(path) not in excluded_routes])


def summary_route_id(summary_path: Path) -> str:
  try:
    route, _mode = read_summary_route_and_mode(summary_path)
  except Exception:
    route = ""
  return route or str(summary_path)


def parse_route_list_file(path: Path) -> list[str]:
  try:
    raw = path.read_text()
  except OSError:
    return []

  routes: list[str] = []
  for line in raw.splitlines():
    route = line.strip()
    if not route or route.startswith("#"):
      continue
    routes.append(route)
  return routes


def discover_route_summary(analysis_root: Path, host: str, route: str, event_source: str) -> Path | None:
  host_root = analysis_root / host
  if not host_root.exists():
    return None

  best_path: Path | None = None
  best_key: tuple[int, float, str] | None = None
  for summary_path in host_root.rglob("summary.json"):
    if not summary_has_event_source(summary_path, event_source):
      continue
    summary_route, mode = read_summary_route_and_mode(summary_path)
    if summary_route != route:
      continue
    key = (summary_mode_priority(mode), path_mtime(summary_path), str(summary_path))
    if best_key is None or key > best_key:
      best_key = key
      best_path = summary_path
  return best_path


def parse_args() -> argparse.Namespace:
  parser = argparse.ArgumentParser(description="Run settings snapshot + route refresh + worklog append")
  parser.add_argument("--host", default="comma",
                      help="SSH host alias label (default: comma). Underlying scripts fall back to commawifi if comma is unreachable.")

  parser.add_argument("--settings-dir", default=str(DEFAULT_SETTINGS_DIR),
                      help=f"Directory for settings snapshots. Default: {DEFAULT_SETTINGS_DIR}")
  parser.add_argument("--report-dir", default=str(DEFAULT_REPORT_DIR),
                      help=f"Directory for route refresh reports. Default: {DEFAULT_REPORT_DIR}")
  parser.add_argument("--state-file", default=str(DEFAULT_STATE_FILE),
                      help=f"State file used by route refresh. Default: {DEFAULT_STATE_FILE}")
  parser.add_argument("--download-root", default=str(DEFAULT_DOWNLOAD_ROOT),
                      help=f"Download root used by route refresh. Default: {DEFAULT_DOWNLOAD_ROOT}")
  parser.add_argument("--worklog", default=str(DEFAULT_WORKLOG),
                      help=f"Markdown worklog to append. Default: {DEFAULT_WORKLOG}")
  parser.add_argument("--analysis-root", default=str(DEFAULT_ANALYSIS_ROOT),
                      help=f"Output root for stopping analysis artifacts. Default: {DEFAULT_ANALYSIS_ROOT}")

  parser.add_argument("--params-dir", action="append", default=[],
                      help="Candidate remote params dir for settings snapshot (repeatable)")
  parser.add_argument("--remote-root", action="append", default=[],
                      help="Remote log root for route refresh (repeatable)")
  parser.add_argument("--file-name", action="append", default=[],
                      help="Remote file name filter for route refresh (repeatable)")

  parser.add_argument("--connect-timeout", type=int, default=8, help="SSH connect timeout in seconds")
  parser.add_argument("--include-rlog", action="store_true", help="Include rlog/rlog.bz2 in route refresh")
  parser.add_argument("--max-downloads", type=int, default=0, help="Cap downloads (0 = no limit)")
  parser.add_argument("--newest-first", action="store_true", default=True, help="Prefer newest files when capping downloads")
  parser.add_argument("--oldest-first", action="store_false", dest="newest_first",
                      help="Use path order for download candidates")
  parser.add_argument("--dry-run-sync", action="store_true", help="Run route refresh in discovery-only mode")
  parser.add_argument("--skip-settings", action="store_true", help="Skip settings snapshot stage")
  parser.add_argument("--settings-dry-run", action="store_true",
                      help="Validate/read requested setting writes without applying them")

  parser.add_argument("--title", default=None, help="Optional title override for appended worklog section")
  parser.add_argument("--note", action="append", default=[], help="Note line for worklog section (repeatable)")
  parser.add_argument("--max-list-items", type=int, default=3, help="Max route/segment items shown in worklog lines")
  parser.add_argument("--skip-append", action="store_true", help="Skip appending report to markdown worklog")

  parser.add_argument("--analyze", action="store_true", help="Run stop-event analysis after sync")
  parser.add_argument("--analysis-route", default=None, help="Optional route override for analysis")
  parser.add_argument("--analysis-min-route-vmax", type=float, default=0.5,
                      help=("When auto-selecting a route for --analyze, require the route to reach this max vEgo (m/s) "
                            + "to avoid standstill-only routes (0 disables scan)"))
  parser.add_argument("--analysis-max-segments", type=int, default=0,
                      help="Limit segments used by analyzer (0 = all)")
  parser.add_argument("--analysis-min-entry-speed", type=float, default=2.0,
                      help="Minimum stop entry speed for analyzer event detection")
  parser.add_argument("--analysis-event-mode", default="hybrid", choices=["engaged_signal", "speed_transition", "hybrid"],
                      help="Event detector mode for analyze_stopping_behavior.py")
  parser.add_argument("--analysis-require-enabled-speed-events", action="store_true",
                      help="In speed/hybrid mode, keep only events with at least one enabled sample")
  parser.add_argument("--skip-analysis-append", action="store_true",
                      help="When --analyze is used, do not append analysis summary to worklog")

  parser.add_argument("--fit-model", action="store_true",
                      help="Fit a fresh stopping model after sync/analysis")
  parser.add_argument("--fit-summary-json", action="append", default=[],
                      help="Explicit summary.json inputs for model fit (repeatable)")
  parser.add_argument("--fit-recent-summaries", type=int, default=12,
                      help="When --fit-summary-json is omitted, use this many newest summaries from analysis root")
  parser.add_argument("--fit-event-source", default="all", choices=["all", "signal", "speed", "hybrid"],
                      help="Event source filter for model fit and optional model gate")
  parser.add_argument("--gate-summary-json", action="append", default=[],
                      help=("Explicit summary.json inputs for gates (repeatable). "
                            + "If omitted, gates default to the same summaries used for model fit."))
  parser.add_argument("--gate-route", action="append", default=[],
                      help=("Route ID to include in gates (repeatable). "
                            + "Selects the newest matching summary.json under analysis_root/host/<route>/."))
  parser.add_argument("--gate-route-file", default=None,
                      help=("File containing one route ID per line for gates. "
                            + "Lines starting with # are ignored."))
  parser.add_argument("--fit-max-delay-frames", type=int, default=25,
                      help="Maximum command-delay frames searched by fit_stopping_model.py")
  parser.add_argument("--fit-min-speed", type=float, default=0.0,
                      help="Minimum vEgo used in model fit rows")
  parser.add_argument("--fit-max-speed", type=float, default=1.8,
                      help="Maximum vEgo used in model fit rows")
  parser.add_argument("--fit-relief-cmd-threshold", type=float, default=-0.25,
                      help="Accel-command threshold for clutch-relief feature in model fit")
  parser.add_argument("--fit-low-speed-ref", type=float, default=1.2,
                      help="Reference speed for low-speed feature scaling in model fit")
  parser.add_argument("--fit-min-rows", type=int, default=120,
                      help="Minimum rows required by fit_stopping_model.py")
  parser.add_argument("--fit-delay-min-sample-ratio", type=float, default=0.40,
                      help="Minimum sample-count ratio used by delay selection in fit_stopping_model.py")
  parser.add_argument("--fit-delay-rmse-tolerance", type=float, default=0.03,
                      help="Relative RMSE tolerance used to prefer lower delay in fit_stopping_model.py")
  parser.add_argument("--model-dir", default=str(DEFAULT_MODEL_DIR),
                      help=f"Directory for fitted models. Default: {DEFAULT_MODEL_DIR}")
  parser.add_argument("--fit-output", default=None,
                      help="Optional explicit model output path. Default: model_dir/stopping_model_<stamp>_<event_source>.json")

  parser.add_argument("--run-model-gate", action="store_true",
                      help="After fitting, run check_harsh_stops_model.py on the same summary inputs")
  parser.add_argument("--model-gate-command-source", default="controller", choices=["recorded", "controller"],
                      help="Command source for check_harsh_stops_model.py")
  parser.add_argument("--model-gate-controller-scope", default="engaged_stopping", choices=["all", "engaged", "engaged_stopping"],
                      help="Controller replay scope used by model gate when command source is controller")
  parser.add_argument("--model-gate-controller-should-stop-source", default="recorded", choices=["recorded", "constant_true"],
                      help="For controller replay in model gate: use recorded shouldStop or force shouldStop=true")
  parser.add_argument("--model-gate-controller-min-enabled-ratio", type=float, default=0.80,
                      help="Minimum enabled ratio used by controller-scope filters in model gate")
  parser.add_argument("--model-gate-min-events", type=int, default=6,
                      help="Minimum events required by model gate")
  parser.add_argument("--model-gate-min-entry-speed", type=float, default=0.20,
                      help="Minimum entry speed used by model gate")
  parser.add_argument("--model-gate-max-harsh-rate", type=float, default=0.50,
                      help="Maximum harsh rate accepted by model gate")
  parser.add_argument("--model-gate-max-leapfrog-rate", type=float, default=1.0,
                      help="Maximum leapfrog rate accepted by model gate (1.0 disables)")
  parser.add_argument("--model-gate-max-leapfrog-count", type=int, default=0,
                      help="Maximum leapfrog count accepted by model gate (0 disables)")
  parser.add_argument("--model-gate-max-pred-end-jerk", type=float, default=0.70,
                      help="Predicted end-stop jerk threshold used by model gate")
  parser.add_argument("--model-gate-max-pred-end-cmd-jerk", type=float, default=3.0,
                      help="Predicted end-stop command jerk threshold used by model gate")
  parser.add_argument("--model-gate-max-pred-end-accel-step", type=float, default=0.08,
                      help="Predicted end-stop acceleration-step threshold used by model gate")
  parser.add_argument("--model-gate-min-pred-a-floor", type=float, default=-1.10,
                      help="Predicted minimum acceleration floor used by model gate")
  parser.add_argument("--model-gate-max-pred-rollout-m", type=float, default=2.0,
                      help="Predicted rollout threshold used by model gate")
  parser.add_argument("--model-gate-min-pred-lead-hold-distance-m", type=float, default=2.0,
                      help="For lead-follow stops, minimum acceptable predicted final hold gap used by model gate")
  parser.add_argument("--model-gate-max-pred-lead-hold-distance-m", type=float, default=4.0,
                      help="For lead-follow stops, maximum acceptable predicted final hold gap used by model gate")
  parser.add_argument("--model-gate-max-pred-speed-rebound-while-should-stop", type=float, default=0.08,
                      help="Predicted speed-rebound threshold used by model gate leapfrog classification")
  parser.add_argument("--model-gate-max-pred-should-stop-unexpected-accel", type=float, default=0.10,
                      help="Predicted unexpected-accel threshold used by model gate leapfrog classification")
  parser.add_argument("--model-gate-output", default=None,
                      help="Optional explicit JSON output path for model gate")
  parser.add_argument("--run-leapfrog-alignment", action="store_true",
                      help="After model gate, compare measured vs predicted leapfrog event overlap")
  parser.add_argument("--alignment-min-enabled-ratio", type=float, default=None,
                      help="Enabled-ratio filter used by measured leapfrog check (default follows model-gate controller filter)")
  parser.add_argument("--alignment-min-stop-signal-ratio", type=float, default=0.0,
                      help="Stop-signal-ratio filter used by measured leapfrog check")
  parser.add_argument("--alignment-event-id-tolerance", type=int, default=1,
                      help="Event-id tolerance for near-match diagnostics in leapfrog alignment report")
  parser.add_argument("--alignment-min-overlap-recall", type=float, default=0.0,
                      help="Optional minimum measured-vs-predicted leapfrog overlap recall [0..1] (0 disables)")
  parser.add_argument("--alignment-max-count-delta", type=int, default=-1,
                      help="Optional max abs(measured_count - predicted_count) for alignment (negative disables)")
  parser.add_argument("--alignment-measured-output", default=None,
                      help="Optional explicit JSON output path for measured leapfrog check")
  parser.add_argument("--alignment-output", default=None,
                      help="Optional explicit JSON output path for leapfrog alignment report")

  parser.add_argument("--run-measured-gate", action="store_true",
                      help="Run check_harsh_stops.py on the same summary inputs (requires --fit-model in the same run)")
  parser.add_argument("--measured-gate-min-enabled-ratio", type=float, default=0.80,
                      help="Enabled-ratio filter used by measured gate")
  parser.add_argument("--measured-gate-min-stop-signal-ratio", type=float, default=0.0,
                      help="Stop-signal-ratio filter used by measured gate")
  parser.add_argument("--measured-gate-min-events", type=int, default=4,
                      help="Minimum events required by measured gate")
  parser.add_argument("--measured-gate-min-entry-speed", type=float, default=0.20,
                      help="Minimum entry speed used by measured gate")
  parser.add_argument("--measured-gate-max-harsh-rate", type=float, default=0.20,
                      help="Maximum harsh rate accepted by measured gate")
  parser.add_argument("--measured-gate-max-leapfrog-rate", type=float, default=1.0,
                      help="Maximum leapfrog rate accepted by measured gate (1.0 disables)")
  parser.add_argument("--measured-gate-max-leapfrog-count", type=int, default=0,
                      help="Maximum leapfrog count accepted by measured gate (0 disables)")
  parser.add_argument("--measured-gate-output", default=None,
                      help="Optional explicit JSON output path for measured gate")

  parser.add_argument("--run-variant-benchmark", action="store_true",
                      help="Run benchmark_controller_variants.py on a holdout summary (requires --fit-model in the same run)")
  parser.add_argument("--benchmark-summary-json", default=None,
                      help="Optional summary.json to benchmark (defaults to the analysis summary when --analyze is used)")
  parser.add_argument("--benchmark-output", default=None,
                      help="Optional explicit JSON output path for variant benchmark")

  parser.add_argument("--skip-cycle-summary-append", action="store_true",
                      help="Skip appending the cycle model/gate/benchmark summary to the worklog")

  return parser.parse_args()


def main() -> int:
  args = parse_args()
  dependency_rc = ensure_dependency_ready_interpreter(args)
  if dependency_rc != 0:
    return dependency_rc
  stamp = utc_stamp()
  script_dir = Path(__file__).resolve().parent
  repo_branch, repo_commit = read_repo_identity(REPO_ROOT)

  settings_dir = Path(args.settings_dir).expanduser()
  report_dir = Path(args.report_dir).expanduser()
  state_file = Path(args.state_file).expanduser()
  download_root = Path(args.download_root).expanduser()
  worklog = Path(args.worklog).expanduser()
  analysis_root = Path(args.analysis_root).expanduser()
  model_dir = Path(args.model_dir).expanduser()

  settings_path = settings_dir / f"stop_settings_{args.host}_{stamp}.json"
  report_path = report_dir / f"route_refresh_{args.host}_{stamp}.json"
  analysis_output_dir = analysis_root / args.host / f"cycle_{stamp}"
  analysis_summary_json = analysis_output_dir / "summary.json"
  settings_assignments: list[tuple[str, float]] = []

  settings_dir.mkdir(parents=True, exist_ok=True)
  report_dir.mkdir(parents=True, exist_ok=True)
  state_file.parent.mkdir(parents=True, exist_ok=True)
  download_root.mkdir(parents=True, exist_ok=True)
  analysis_root.mkdir(parents=True, exist_ok=True)
  model_dir.mkdir(parents=True, exist_ok=True)

  if args.skip_settings and settings_assignments:
    print("[cycle] --skip-settings cannot be combined with stop-setting write arguments", file=sys.stderr)
    return RC_INSUFFICIENT_INPUTS

  if not args.skip_settings:
    if settings_assignments:
      snapshot_cmd = [
        sys.executable,
        str(script_dir / "device_stop_settings.py"),
        "set",
        "--host",
        args.host,
        "--connect-timeout",
        str(args.connect_timeout),
        "--output",
        str(settings_path),
        "--settings-dir",
        str(settings_dir),
      ]
      for key, value in settings_assignments:
        snapshot_cmd.extend(["--set", f"{key}={value}"])
      for include_key in ("AdvancedLongitudinalTune", "LongitudinalTune"):
        snapshot_cmd.extend(["--include-key", include_key])
      if args.settings_dry_run:
        snapshot_cmd.append("--dry-run")
      settings_label = "settings set+snapshot"
    else:
      snapshot_cmd = [
        sys.executable,
        str(script_dir / "device_stop_settings.py"),
        "snapshot",
        "--host",
        args.host,
        "--connect-timeout",
        str(args.connect_timeout),
        "--output",
        str(settings_path),
        "--settings-dir",
        str(settings_dir),
      ]
      settings_label = "settings snapshot"
    for params_dir in args.params_dir:
      snapshot_cmd.extend(["--params-dir", params_dir])

    snapshot_rc = run_cmd(snapshot_cmd, settings_label)
    if snapshot_rc != 0:
      return snapshot_rc

  sync_cmd = [
    sys.executable,
    str(ROUTE_REFRESH_SCRIPT),
    "--host",
    args.host,
    "--connect-timeout",
    str(args.connect_timeout),
    "--state-file",
    str(state_file),
    "--download-root",
    str(download_root),
    "--report-file",
    str(report_path),
    "--report-dir",
    str(report_dir),
    "--max-downloads",
    str(args.max_downloads),
  ]
  if args.include_rlog:
    sync_cmd.append("--include-rlog")
  if args.newest_first:
    sync_cmd.append("--newest-first")
  if args.dry_run_sync:
    sync_cmd.append("--dry-run")
  for remote_root in args.remote_root:
    sync_cmd.extend(["--remote-root", remote_root])
  for file_name in args.file_name:
    sync_cmd.extend(["--file-name", file_name])

  sync_rc = run_cmd(sync_cmd, "route refresh")
  overall_rc = sync_rc

  if not report_path.exists():
    print(f"[cycle] route refresh report missing: {report_path}", file=sys.stderr)
    return sync_rc if sync_rc != 0 else RC_INSUFFICIENT_INPUTS

  if not args.skip_append:
    append_cmd = [
      sys.executable,
      str(script_dir / "append_sync_report.py"),
      "--report-file",
      str(report_path),
      "--worklog",
      str(worklog),
      "--max-list-items",
      str(args.max_list_items),
    ]
    if not args.skip_settings and settings_path.exists():
      append_cmd.extend(["--settings-file", str(settings_path)])
    if args.title:
      append_cmd.extend(["--title", args.title])
    for note in args.note:
      append_cmd.extend(["--note", note])

    append_rc = run_cmd(append_cmd, "worklog append")
    if append_rc != 0:
      return append_rc

  if args.analyze:
    selected_route: str | None = None
    if not args.analysis_route:
      selected_route = pick_moving_route_for_analysis(
        load_sync_report(report_path),
        download_root=download_root,
        host=args.host,
        min_route_vmax_mps=args.analysis_min_route_vmax,
        require_stop_signal=args.analysis_event_mode == "engaged_signal",
      )
      if selected_route:
        print(f"[cycle] selected analysis route: {selected_route}", flush=True)
    host_download_dir = host_download_root(download_root, args.host)
    if not has_local_qlogs(host_download_dir):
      print(f"[cycle] skipping analysis: no local qlog/qlog.bz2/qlog.zst files under {host_download_dir}")
      return sync_rc

    analyze_cmd = [
      sys.executable,
      str(script_dir / "analyze_stopping_behavior.py"),
      "--host",
      args.host,
      "--download-root",
      str(download_root),
      "--analysis-root",
      str(analysis_root),
      "--output-dir",
      str(analysis_output_dir),
      "--min-entry-speed",
      str(args.analysis_min_entry_speed),
      "--event-mode",
      str(args.analysis_event_mode),
      "--max-segments",
      str(args.analysis_max_segments),
    ]
    if args.analysis_require_enabled_speed_events:
      analyze_cmd.append("--require-enabled-speed-events")
    analysis_route = args.analysis_route or selected_route
    if analysis_route:
      analyze_cmd.extend(["--route", analysis_route])
    if settings_path.exists():
      analyze_cmd.extend(["--settings-file", str(settings_path)])

    analyze_rc = run_cmd(analyze_cmd, "stopping analysis")
    if analyze_rc != 0:
      return analyze_rc

    if not args.skip_analysis_append and analysis_summary_json.exists():
      append_analysis_cmd = [
        sys.executable,
        str(script_dir / "append_analysis_report.py"),
        "--summary-json",
        str(analysis_summary_json),
        "--worklog",
        str(worklog),
      ]
      append_analysis_rc = run_cmd(append_analysis_cmd, "analysis append")
      if append_analysis_rc != 0:
        return append_analysis_rc

  fit_summaries: list[Path] = []
  gate_summaries: list[Path] = []
  fitted_model_path: Path | None = None
  measured_gate_output_path: Path | None = None
  model_gate_output_path: Path | None = None
  alignment_output_path: Path | None = None
  benchmark_output_path: Path | None = None

  explicit_gate_requested = bool(args.gate_summary_json or args.gate_route or args.gate_route_file)

  if args.fit_model:
    explicit_fit_summaries = [Path(item).expanduser() for item in args.fit_summary_json]
    missing_fit_summaries = [path for path in explicit_fit_summaries if not path.exists()]
    if missing_fit_summaries:
      for missing in missing_fit_summaries:
        print(f"[cycle] missing fit summary: {missing}", file=sys.stderr)
      return RC_INSUFFICIENT_INPUTS
    fit_summaries = select_fit_summaries(
      explicit_summaries=explicit_fit_summaries,
      analysis_summary_json=analysis_summary_json,
      analysis_root=analysis_root,
      host=args.host,
      event_source=args.fit_event_source,
      recent_limit=args.fit_recent_summaries,
    )
    if not fit_summaries:
      print(f"[cycle] no fit summaries found for host={args.host} event_source={args.fit_event_source}", file=sys.stderr)
      return RC_INSUFFICIENT_INPUTS

    if explicit_gate_requested and not explicit_fit_summaries:
      gate_routes: list[str] = []
      if args.gate_route_file:
        gate_routes.extend(parse_route_list_file(Path(args.gate_route_file).expanduser()))
      gate_routes.extend([str(route).strip() for route in args.gate_route if str(route).strip()])

      holdout_routes = set(gate_routes)
      for item in args.gate_summary_json:
        if not item:
          continue
        holdout_routes.add(summary_route_id(Path(item).expanduser()))

      if holdout_routes:
        filtered = [path for path in fit_summaries if summary_route_id(path) not in holdout_routes]
        excluded = len(fit_summaries) - len(filtered)
        if excluded:
          print(f"[cycle] excluding {excluded} holdout summary input(s) from model fit", flush=True)
        fit_summaries = filtered
        if not fit_summaries:
          print("[cycle] no fit summaries remain after excluding holdout; increase --fit-recent-summaries or pass explicit --fit-summary-json", file=sys.stderr)
          return RC_INSUFFICIENT_INPUTS

    if args.fit_output:
      fitted_model_path = Path(args.fit_output).expanduser()
    else:
      fitted_model_path = model_dir / f"stopping_model_{stamp}_{args.fit_event_source}.json"
    fitted_model_path.parent.mkdir(parents=True, exist_ok=True)

    fit_cmd = [
      sys.executable,
      str(script_dir / "fit_stopping_model.py"),
      "--event-source",
      args.fit_event_source,
      "--max-delay-frames",
      str(args.fit_max_delay_frames),
      "--min-speed",
      str(args.fit_min_speed),
      "--max-speed",
      str(args.fit_max_speed),
      "--relief-cmd-threshold",
      str(args.fit_relief_cmd_threshold),
      "--low-speed-ref",
      str(args.fit_low_speed_ref),
      "--min-rows",
      str(args.fit_min_rows),
      "--delay-min-sample-ratio",
      str(args.fit_delay_min_sample_ratio),
      "--delay-rmse-tolerance",
      str(args.fit_delay_rmse_tolerance),
      "--output",
      str(fitted_model_path),
    ]
    for summary_path in fit_summaries:
      fit_cmd.extend(["--summary-json", str(summary_path)])

    fit_rc = run_cmd(fit_cmd, "fit stopping model")
    if fit_rc != 0:
      return fit_rc

    print(f"[cycle] fitted model: {fitted_model_path}", flush=True)

  if args.run_measured_gate or args.run_model_gate or args.run_leapfrog_alignment or explicit_gate_requested:
    explicit_gate_summaries = [Path(item).expanduser() for item in args.gate_summary_json if item]
    missing_gate_summaries = [path for path in explicit_gate_summaries if not path.exists()]
    if missing_gate_summaries:
      for missing in missing_gate_summaries:
        print(f"[cycle] missing gate summary: {missing}", file=sys.stderr)
      return RC_INSUFFICIENT_INPUTS

    gate_routes: list[str] = []
    if args.gate_route_file:
      gate_routes.extend(parse_route_list_file(Path(args.gate_route_file).expanduser()))
    gate_routes.extend([str(route).strip() for route in args.gate_route if str(route).strip()])

    if explicit_gate_summaries or gate_routes:
      gate_summaries.extend(explicit_gate_summaries)
      for route in gate_routes:
        discovered = discover_route_summary(analysis_root=analysis_root, host=args.host, route=route, event_source=args.fit_event_source)
        if discovered is None:
          print(f"[cycle] missing gate summary for route: {route}", file=sys.stderr)
          return RC_INSUFFICIENT_INPUTS
        gate_summaries.append(discovered)
      gate_summaries = dedupe_paths([path for path in gate_summaries if path.exists()])
    else:
      gate_summaries = fit_summaries or select_fit_summaries(
        explicit_summaries=[],
        analysis_summary_json=analysis_summary_json,
        analysis_root=analysis_root,
        host=args.host,
        event_source=args.fit_event_source,
        recent_limit=args.fit_recent_summaries,
      )

  if args.run_measured_gate:
    if not gate_summaries:
      print("[cycle] no summaries available for measured gate (pass --gate-summary-json/--gate-route* or run --analyze)", file=sys.stderr)
      return RC_INSUFFICIENT_INPUTS

    if args.measured_gate_output:
      measured_gate_output_path = Path(args.measured_gate_output).expanduser()
    else:
      measured_gate_output_path = analysis_root / f"measured_harsh_gate_{args.host}_{stamp}_{args.fit_event_source}.json"
    measured_gate_output_path.parent.mkdir(parents=True, exist_ok=True)

    measured_gate_cmd = [
      sys.executable,
      str(script_dir / "check_harsh_stops.py"),
      "--event-source",
      args.fit_event_source,
      "--min-enabled-ratio",
      str(args.measured_gate_min_enabled_ratio),
      "--min-stop-signal-ratio",
      str(args.measured_gate_min_stop_signal_ratio),
      "--min-events",
      str(args.measured_gate_min_events),
      "--min-entry-speed",
      str(args.measured_gate_min_entry_speed),
      "--max-harsh-rate",
      str(args.measured_gate_max_harsh_rate),
      "--max-leapfrog-rate",
      str(args.measured_gate_max_leapfrog_rate),
      "--max-leapfrog-count",
      str(args.measured_gate_max_leapfrog_count),
      "--output-json",
      str(measured_gate_output_path),
    ]
    for summary_path in gate_summaries:
      measured_gate_cmd.extend(["--summary-json", str(summary_path)])

    measured_gate_rc = run_cmd(measured_gate_cmd, "measured harsh gate")
    overall_rc = merge_rc(overall_rc, measured_gate_rc)

  if args.run_model_gate:
    if fitted_model_path is None:
      print("[cycle] --run-model-gate requires --fit-model in the same run", file=sys.stderr)
      return RC_INSUFFICIENT_INPUTS
    if not gate_summaries:
      print("[cycle] no summaries available for model gate (pass --gate-summary-json/--gate-route* or run --analyze)", file=sys.stderr)
      return RC_INSUFFICIENT_INPUTS

    if args.model_gate_output:
      model_gate_output_path = Path(args.model_gate_output).expanduser()
    else:
      model_gate_output_path = analysis_root / f"model_harsh_check_{args.host}_{stamp}_{args.fit_event_source}.json"
    model_gate_output_path.parent.mkdir(parents=True, exist_ok=True)

    gate_cmd = [
      sys.executable,
      str(script_dir / "check_harsh_stops_model.py"),
      "--model-json",
      str(fitted_model_path),
      "--event-source",
      args.fit_event_source,
      "--command-source",
      args.model_gate_command_source,
      "--controller-scope",
      args.model_gate_controller_scope,
      "--controller-should-stop-source",
      args.model_gate_controller_should_stop_source,
      "--controller-min-enabled-ratio",
      str(args.model_gate_controller_min_enabled_ratio),
      "--min-events",
      str(args.model_gate_min_events),
      "--min-entry-speed",
      str(args.model_gate_min_entry_speed),
      "--max-harsh-rate",
      str(args.model_gate_max_harsh_rate),
      "--max-leapfrog-rate",
      str(args.model_gate_max_leapfrog_rate),
      "--max-leapfrog-count",
      str(args.model_gate_max_leapfrog_count),
      "--max-pred-end-jerk",
      str(args.model_gate_max_pred_end_jerk),
      "--max-pred-end-cmd-jerk",
      str(args.model_gate_max_pred_end_cmd_jerk),
      "--max-pred-end-accel-step",
      str(args.model_gate_max_pred_end_accel_step),
      "--min-pred-a-floor",
      str(args.model_gate_min_pred_a_floor),
      "--max-pred-rollout-m",
      str(args.model_gate_max_pred_rollout_m),
      "--min-pred-lead-hold-distance-m",
      str(args.model_gate_min_pred_lead_hold_distance_m),
      "--max-pred-lead-hold-distance-m",
      str(args.model_gate_max_pred_lead_hold_distance_m),
      "--max-pred-speed-rebound-while-should-stop",
      str(args.model_gate_max_pred_speed_rebound_while_should_stop),
      "--max-pred-should-stop-unexpected-accel",
      str(args.model_gate_max_pred_should_stop_unexpected_accel),
      "--output-json",
      str(model_gate_output_path),
    ]
    for summary_path in gate_summaries:
      gate_cmd.extend(["--summary-json", str(summary_path)])

    gate_rc = run_cmd(gate_cmd, "model harsh gate")
    overall_rc = merge_rc(overall_rc, gate_rc)

  if args.run_leapfrog_alignment:
    if not args.run_model_gate:
      print("[cycle] --run-leapfrog-alignment requires --run-model-gate in the same run", file=sys.stderr)
      return RC_INSUFFICIENT_INPUTS
    if model_gate_output_path is None or not model_gate_output_path.exists():
      print("[cycle] leapfrog alignment requires a model gate output json", file=sys.stderr)
      return RC_INSUFFICIENT_INPUTS
    if not gate_summaries:
      print("[cycle] no summaries available for leapfrog alignment", file=sys.stderr)
      return RC_INSUFFICIENT_INPUTS

    if args.alignment_measured_output:
      measured_output_path = Path(args.alignment_measured_output).expanduser()
    else:
      measured_output_path = analysis_root / f"measured_harsh_gate_{args.host}_{stamp}_{args.fit_event_source}.json"
    measured_output_path.parent.mkdir(parents=True, exist_ok=True)

    alignment_enabled_ratio = args.alignment_min_enabled_ratio
    if alignment_enabled_ratio is None:
      if args.model_gate_command_source == "controller":
        alignment_enabled_ratio = args.model_gate_controller_min_enabled_ratio
      else:
        alignment_enabled_ratio = 0.0

    measured_cmd = [
      sys.executable,
      str(script_dir / "check_harsh_stops.py"),
      "--event-source",
      args.fit_event_source,
      "--min-events",
      "0",
      "--min-entry-speed",
      str(args.model_gate_min_entry_speed),
      "--min-enabled-ratio",
      str(alignment_enabled_ratio),
      "--min-stop-signal-ratio",
      str(args.alignment_min_stop_signal_ratio),
      "--max-harsh-rate",
      "1.0",
      "--max-leapfrog-rate",
      "1.0",
      "--output-json",
      str(measured_output_path),
    ]
    for summary_path in gate_summaries:
      measured_cmd.extend(["--summary-json", str(summary_path)])

    measured_rc = run_cmd(measured_cmd, "measured harsh/leapfrog check")
    overall_rc = merge_rc(overall_rc, measured_rc)

    if args.alignment_output:
      alignment_output_path = Path(args.alignment_output).expanduser()
    else:
      alignment_output_path = analysis_root / f"leapfrog_alignment_{args.host}_{stamp}_{args.fit_event_source}.json"
    alignment_output_path.parent.mkdir(parents=True, exist_ok=True)

    alignment_cmd = [
      sys.executable,
      str(script_dir / "check_leapfrog_alignment.py"),
      "--measured-json",
      str(measured_output_path),
      "--predicted-json",
      str(model_gate_output_path),
      "--event-id-tolerance",
      str(args.alignment_event_id_tolerance),
      "--min-overlap-recall",
      str(args.alignment_min_overlap_recall),
      "--max-count-delta",
      str(args.alignment_max_count_delta),
      "--output-json",
      str(alignment_output_path),
    ]
    alignment_rc = run_cmd(alignment_cmd, "leapfrog alignment check")
    overall_rc = merge_rc(overall_rc, alignment_rc)

  if args.run_variant_benchmark:
    if not args.fit_model:
      print("[cycle] --run-variant-benchmark requires --fit-model in the same run", file=sys.stderr)
      return RC_INSUFFICIENT_INPUTS
    if fitted_model_path is None:
      print("[cycle] variant benchmark requires a fitted model json", file=sys.stderr)
      return RC_INSUFFICIENT_INPUTS

    benchmark_summary_json = Path(args.benchmark_summary_json).expanduser() if args.benchmark_summary_json else None
    if benchmark_summary_json is None:
      if explicit_gate_requested and gate_summaries:
        benchmark_summary_json = gate_summaries[0]
      elif analysis_summary_json.exists():
        benchmark_summary_json = analysis_summary_json
      elif gate_summaries:
        benchmark_summary_json = gate_summaries[0]
      else:
        print(
          "[cycle] --run-variant-benchmark requires --benchmark-summary-json, --analyze, or an explicit gate summary set (--gate-summary-json/--gate-route*)",
          file=sys.stderr,
        )
        return RC_INSUFFICIENT_INPUTS
    if not benchmark_summary_json.exists():
      print(f"[cycle] benchmark summary missing: {benchmark_summary_json}", file=sys.stderr)
      return RC_INSUFFICIENT_INPUTS

    if args.benchmark_output:
      benchmark_output_path = Path(args.benchmark_output).expanduser()
    else:
      benchmark_output_path = analysis_root / f"controller_variant_benchmark_{args.host}_{stamp}_{args.fit_event_source}.json"
    benchmark_output_path.parent.mkdir(parents=True, exist_ok=True)

    benchmark_cmd = [
      sys.executable,
      str(script_dir / "benchmark_controller_variants.py"),
      "--model-json",
      str(fitted_model_path),
      "--summary-json",
      str(benchmark_summary_json),
      "--output-json",
      str(benchmark_output_path),
    ]
    benchmark_rc = run_cmd(benchmark_cmd, "variant benchmark")
    overall_rc = merge_rc(overall_rc, benchmark_rc)

  if not args.skip_append and not args.skip_cycle_summary_append:
    has_cycle_artifacts = any(
      path is not None and path.exists()
      for path in (
        fitted_model_path,
        measured_gate_output_path,
        model_gate_output_path,
        alignment_output_path,
        benchmark_output_path,
      )
    )
    if has_cycle_artifacts:
      append_cycle_cmd = [
        sys.executable,
        str(script_dir / "append_cycle_report.py"),
        "--worklog",
        str(worklog),
        "--host",
        args.host,
        "--stamp",
        stamp,
        "--settings-json",
        str(settings_path),
        "--sync-report-json",
        str(report_path),
      ]
      if analysis_summary_json.exists():
        append_cycle_cmd.extend(["--analysis-summary-json", str(analysis_summary_json)])
      for summary_path in fit_summaries:
        append_cycle_cmd.extend(["--fit-summary-json", str(summary_path)])
      for summary_path in gate_summaries:
        append_cycle_cmd.extend(["--gate-summary-json", str(summary_path)])
      if fitted_model_path is not None and fitted_model_path.exists():
        append_cycle_cmd.extend(["--model-json", str(fitted_model_path)])
      if measured_gate_output_path is not None and measured_gate_output_path.exists():
        append_cycle_cmd.extend(["--measured-gate-json", str(measured_gate_output_path)])
      if model_gate_output_path is not None and model_gate_output_path.exists():
        append_cycle_cmd.extend(["--model-gate-json", str(model_gate_output_path)])
      if alignment_output_path is not None and alignment_output_path.exists():
        append_cycle_cmd.extend(["--leapfrog-alignment-json", str(alignment_output_path)])
      if benchmark_output_path is not None and benchmark_output_path.exists():
        append_cycle_cmd.extend(["--variant-benchmark-json", str(benchmark_output_path)])
      if repo_branch:
        append_cycle_cmd.extend(["--repo-branch", repo_branch])
      if repo_commit:
        append_cycle_cmd.extend(["--repo-commit", repo_commit])

      append_cycle_rc = run_cmd(append_cycle_cmd, "cycle report append")
      if append_cycle_rc != 0:
        return append_cycle_rc

  return overall_rc


if __name__ == "__main__":
  raise SystemExit(main())
