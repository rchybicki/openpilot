#!/usr/bin/env python3
"""Run a full stopping-behavior data cycle: settings snapshot, log sync, and worklog append."""

from __future__ import annotations

import argparse
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path

DEFAULT_DOWNLOAD_ROOT = Path.home() / ".comma" / "stopping_behavior" / "downloads"
DEFAULT_REPORT_DIR = Path.home() / ".comma" / "stopping_behavior" / "reports"
DEFAULT_SETTINGS_DIR = Path.home() / ".comma" / "stopping_behavior" / "settings"
DEFAULT_STATE_FILE = Path.home() / ".comma" / "stopping_behavior" / "sync_state.json"
DEFAULT_ANALYSIS_ROOT = Path.home() / ".comma" / "stopping_behavior" / "analysis"
DEFAULT_WORKLOG = Path("docs/stopping_behavior_worklog.md")


def utc_stamp() -> str:
  return datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")


def run_cmd(cmd: list[str], label: str) -> int:
  print(f"[cycle] running {label}: {' '.join(cmd)}", flush=True)
  result = subprocess.run(cmd)
  return result.returncode


def parse_args() -> argparse.Namespace:
  parser = argparse.ArgumentParser(description="Run settings snapshot + log sync + worklog append")
  parser.add_argument("--host", required=True, help="SSH host alias, e.g. commawifi")

  parser.add_argument("--settings-dir", default=str(DEFAULT_SETTINGS_DIR),
                      help=f"Directory for settings snapshots. Default: {DEFAULT_SETTINGS_DIR}")
  parser.add_argument("--report-dir", default=str(DEFAULT_REPORT_DIR),
                      help=f"Directory for sync reports. Default: {DEFAULT_REPORT_DIR}")
  parser.add_argument("--state-file", default=str(DEFAULT_STATE_FILE),
                      help=f"State file used by sync_new_logs.py. Default: {DEFAULT_STATE_FILE}")
  parser.add_argument("--download-root", default=str(DEFAULT_DOWNLOAD_ROOT),
                      help=f"Download root used by sync_new_logs.py. Default: {DEFAULT_DOWNLOAD_ROOT}")
  parser.add_argument("--worklog", default=str(DEFAULT_WORKLOG),
                      help=f"Markdown worklog to append. Default: {DEFAULT_WORKLOG}")
  parser.add_argument("--analysis-root", default=str(DEFAULT_ANALYSIS_ROOT),
                      help=f"Output root for stopping analysis artifacts. Default: {DEFAULT_ANALYSIS_ROOT}")

  parser.add_argument("--params-dir", action="append", default=[],
                      help="Candidate remote params dir for settings snapshot (repeatable)")
  parser.add_argument("--remote-root", action="append", default=[],
                      help="Remote log root for sync (repeatable)")
  parser.add_argument("--file-name", action="append", default=[],
                      help="Remote file name filter for sync (repeatable)")

  parser.add_argument("--connect-timeout", type=int, default=8, help="SSH connect timeout in seconds")
  parser.add_argument("--include-rlog", action="store_true", help="Include rlog/rlog.bz2 in sync")
  parser.add_argument("--max-downloads", type=int, default=0, help="Cap downloads (0 = no limit)")
  parser.add_argument("--newest-first", action="store_true", default=True, help="Prefer newest files when capping downloads")
  parser.add_argument("--oldest-first", action="store_false", dest="newest_first",
                      help="Use path order for download candidates")
  parser.add_argument("--dry-run-sync", action="store_true", help="Run sync in discovery-only mode")
  parser.add_argument("--skip-settings", action="store_true", help="Skip settings snapshot stage")
  parser.add_argument("--set-stopping-speed-breakpoint", type=float, default=None,
                      help="If set, write StoppingSpeedBreakpoint before sync/analysis")
  parser.add_argument("--set-stopping-error-factor", type=float, default=None,
                      help="If set, write StoppingErrorFactor before sync/analysis")
  parser.add_argument("--settings-dry-run", action="store_true",
                      help="Validate/read requested setting writes without applying them")

  parser.add_argument("--title", default=None, help="Optional title override for appended worklog section")
  parser.add_argument("--note", action="append", default=[], help="Note line for worklog section (repeatable)")
  parser.add_argument("--max-list-items", type=int, default=3, help="Max route/segment items shown in worklog lines")
  parser.add_argument("--skip-append", action="store_true", help="Skip appending report to markdown worklog")

  parser.add_argument("--analyze", action="store_true", help="Run stop-event analysis after sync")
  parser.add_argument("--analysis-route", default=None, help="Optional route override for analysis")
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

  return parser.parse_args()


def main() -> int:
  args = parse_args()
  stamp = utc_stamp()
  script_dir = Path(__file__).resolve().parent

  settings_dir = Path(args.settings_dir).expanduser()
  report_dir = Path(args.report_dir).expanduser()
  state_file = Path(args.state_file).expanduser()
  download_root = Path(args.download_root).expanduser()
  worklog = Path(args.worklog).expanduser()
  analysis_root = Path(args.analysis_root).expanduser()

  settings_path = settings_dir / f"stop_settings_{args.host}_{stamp}.json"
  report_path = report_dir / f"sync_{args.host}_{stamp}.json"
  analysis_output_dir = analysis_root / args.host / f"cycle_{stamp}"
  analysis_summary_json = analysis_output_dir / "summary.json"
  settings_assignments: list[tuple[str, float]] = []
  if args.set_stopping_speed_breakpoint is not None:
    settings_assignments.append(("StoppingSpeedBreakpoint", float(args.set_stopping_speed_breakpoint)))
  if args.set_stopping_error_factor is not None:
    settings_assignments.append(("StoppingErrorFactor", float(args.set_stopping_error_factor)))

  settings_dir.mkdir(parents=True, exist_ok=True)
  report_dir.mkdir(parents=True, exist_ok=True)
  state_file.parent.mkdir(parents=True, exist_ok=True)
  download_root.mkdir(parents=True, exist_ok=True)
  analysis_root.mkdir(parents=True, exist_ok=True)

  if args.skip_settings and settings_assignments:
    print("[cycle] --skip-settings cannot be combined with stop-setting write arguments", file=sys.stderr)
    return 2

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
      for include_key in ("AdvancedLongitudinalTune", "LongitudinalTune", "StoppingSpeedBreakpoint", "StoppingErrorFactor"):
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
    str(script_dir / "sync_new_logs.py"),
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

  sync_rc = run_cmd(sync_cmd, "log sync")

  if not report_path.exists():
    print(f"[cycle] sync report missing: {report_path}", file=sys.stderr)
    return sync_rc if sync_rc != 0 else 2

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
    host_download_dir = download_root / args.host
    has_local_qlogs = host_download_dir.exists() and any(host_download_dir.rglob("qlog"))
    if not has_local_qlogs:
      print(f"[cycle] skipping analysis: no local qlogs under {host_download_dir}")
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
    if args.analysis_route:
      analyze_cmd.extend(["--route", args.analysis_route])
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

  return sync_rc


if __name__ == "__main__":
  raise SystemExit(main())
