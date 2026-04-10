#!/usr/bin/env python3
"""Refresh newly discovered route log files from a comma device via SSH."""

from __future__ import annotations

import argparse
import json
import os
import shlex
import shutil
import subprocess
import sys
import time
from collections import defaultdict
from dataclasses import dataclass
from pathlib import Path
from typing import Any

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
  sys.path.insert(0, str(REPO_ROOT))

from openpilot.tools.route_sync.common import (
  CANONICAL_REMOTE_ROOT,
  DEFAULT_DOWNLOAD_ROOT,
  DEFAULT_FILE_NAMES,
  DEFAULT_HOST,
  DEFAULT_REMOTE_ROOTS,
  DEFAULT_REPORT_DIR,
  DEFAULT_STATE_FILE,
  FALLBACK_HOST,
  RLOG_FILE_NAMES,
  build_report_path,
  cache_remote_path_aliases,
  cache_host_aliases,
  canonical_cache_host,
  local_path_for,
  raw_local_path_for,
  utc_now_iso,
)


@dataclass(frozen=True)
class RemoteFile:
  remote_path: str
  size: int
  mtime: int
  segment: str
  route: str


def derive_segment_and_route(remote_path: str) -> tuple[str, str]:
  segment = Path(remote_path).parent.name
  route = segment.rsplit("--", 1)[0] if "--" in segment else segment
  return segment, route


def build_remote_list_script(remote_roots: list[str], file_names: list[str]) -> str:
  quoted_roots = " ".join(shlex.quote(root) for root in remote_roots)
  find_expr = " -o ".join(f"-name {shlex.quote(name)}" for name in file_names)
  return f"""
for root in {quoted_roots}; do
  [ -d "$root" ] || continue
  find "$root" -maxdepth 2 -mindepth 2 -type f \\( {find_expr} \\) | while IFS= read -r f; do
    meta="$(stat -c '%s\t%Y' "$f" 2>/dev/null || stat -f '%z\t%m' "$f" 2>/dev/null || echo '-1\t-1')"
    printf '%s\t%s\n' "$f" "$meta"
  done
done
""".strip()


def list_remote_files(host: str, remote_roots: list[str], file_names: list[str], connect_timeout: int) -> list[RemoteFile]:
  script = build_remote_list_script(remote_roots, file_names)
  cmd = [
    "ssh",
    "-o",
    "BatchMode=yes",
    "-o",
    f"ConnectTimeout={connect_timeout}",
    host,
    "sh",
  ]
  result = subprocess.run(cmd, input=script, capture_output=True, text=True)
  if result.returncode != 0:
    stderr = result.stderr.strip() or result.stdout.strip() or "unknown ssh error"
    raise RuntimeError(f"Failed to list remote logs from {host}: {stderr}")

  files: list[RemoteFile] = []
  for raw_line in result.stdout.splitlines():
    line = raw_line.strip()
    if not line:
      continue

    parts = line.split("\t")
    if len(parts) < 3:
      continue

    remote_path, size_raw, mtime_raw = parts[0], parts[1], parts[2]
    try:
      size = int(size_raw)
      mtime = int(float(mtime_raw))
    except ValueError:
      continue

    segment, route = derive_segment_and_route(remote_path)
    files.append(RemoteFile(remote_path=remote_path, size=size, mtime=mtime, segment=segment, route=route))

  files.sort(key=lambda item: item.remote_path)
  return files


def load_state(state_file: Path) -> dict[str, Any]:
  if not state_file.exists():
    return {"version": 1, "hosts": {}}

  try:
    data = json.loads(state_file.read_text())
  except json.JSONDecodeError:
    return {"version": 1, "hosts": {}}

  if not isinstance(data, dict):
    return {"version": 1, "hosts": {}}

  data.setdefault("version", 1)
  data.setdefault("hosts", {})
  return data


def save_state(state_file: Path, state: dict[str, Any]) -> None:
  state_file.parent.mkdir(parents=True, exist_ok=True)
  state_file.write_text(json.dumps(state, indent=2, sort_keys=True) + "\n")


def merge_host_states(hosts: dict[str, Any], host: str) -> tuple[str, dict[str, Any]]:
  cache_host = canonical_cache_host(host)
  host_state = hosts.setdefault(cache_host, {})
  file_state: dict[str, Any] = host_state.setdefault("files", {})

  for alias in cache_host_aliases(host):
    if alias == cache_host:
      continue
    alias_state = hosts.pop(alias, None)
    if not isinstance(alias_state, dict):
      continue
    alias_files = alias_state.get("files", {})
    if not isinstance(alias_files, dict):
      continue

    for remote_path, alias_entry in alias_files.items():
      if remote_path not in file_state:
        file_state[remote_path] = alias_entry
        continue

      merged_entry = file_state[remote_path]
      if not isinstance(merged_entry, dict) or not isinstance(alias_entry, dict):
        continue

      for key in ("size", "mtime", "segment", "route"):
        if key not in merged_entry and key in alias_entry:
          merged_entry[key] = alias_entry[key]

      first_synced = alias_entry.get("first_synced_utc")
      current_first = merged_entry.get("first_synced_utc")
      if first_synced is not None and (current_first is None or str(first_synced) < str(current_first)):
        merged_entry["first_synced_utc"] = first_synced

      for key in ("last_synced_utc", "last_seen_utc", "local_path"):
        alias_value = alias_entry.get(key)
        current_value = merged_entry.get(key)
        if alias_value is not None and (current_value is None or str(alias_value) > str(current_value)):
          merged_entry[key] = alias_value

  return cache_host, host_state


def adopt_existing_alias_file(download_root: Path, host: str, remote_path: str) -> Path:
  cache_host = canonical_cache_host(host)
  canonical_path = local_path_for(download_root, cache_host, remote_path)
  if canonical_path.exists():
    return canonical_path

  for alias in cache_host_aliases(host):
    for alias_remote_path in cache_remote_path_aliases(remote_path):
      alias_path = raw_local_path_for(download_root, alias, alias_remote_path)
      if alias_path == canonical_path or not alias_path.exists():
        continue

      canonical_path.parent.mkdir(parents=True, exist_ok=True)
      try:
        alias_path.replace(canonical_path)
      except OSError:
        shutil.copy2(alias_path, canonical_path)
        alias_path.unlink()
      return canonical_path

  return canonical_path


def download_file(host: str, remote_path: str, local_path: Path, connect_timeout: int) -> None:
  local_path.parent.mkdir(parents=True, exist_ok=True)
  remote_spec = f"{host}:{remote_path}"
  tmp_path = local_path.with_name(f"{local_path.name}.partial_{os.getpid()}_{time.time_ns()}")
  cmd = [
    "scp",
    "-q",
    "-o",
    "BatchMode=yes",
    "-o",
    f"ConnectTimeout={connect_timeout}",
    remote_spec,
    str(tmp_path),
  ]
  try:
    result = subprocess.run(cmd, capture_output=True, text=True)
    if result.returncode != 0:
      stderr = result.stderr.strip() or result.stdout.strip() or "unknown scp error"
      raise RuntimeError(stderr)
    tmp_path.replace(local_path)
  finally:
    try:
      if tmp_path.exists():
        tmp_path.unlink()
    except OSError:
      pass


def interleave_by_route(candidates: list[RemoteFile], newest_first: bool) -> list[RemoteFile]:
  if not candidates:
    return []

  grouped: dict[str, list[RemoteFile]] = defaultdict(list)
  for item in candidates:
    grouped[item.route].append(item)

  for route in grouped:
    grouped[route].sort(key=lambda item: (item.mtime, item.remote_path), reverse=newest_first)

  route_order = sorted(grouped.keys(), key=lambda route: (grouped[route][0].mtime, route), reverse=newest_first)

  output: list[RemoteFile] = []
  index = 0
  while True:
    added = False
    for route in route_order:
      bucket = grouped[route]
      if index < len(bucket):
        output.append(bucket[index])
        added = True
    if not added:
      break
    index += 1

  return output


def parse_args() -> argparse.Namespace:
  parser = argparse.ArgumentParser(description="Refresh newly discovered route log files from comma/commawifi over SSH")
  parser.add_argument(
    "--host",
    default=DEFAULT_HOST,
    help=f"SSH host alias (defaults to {DEFAULT_HOST}; falls back to {FALLBACK_HOST} when {DEFAULT_HOST} is unreachable)",
  )
  parser.add_argument(
    "--remote-root",
    action="append",
    dest="remote_roots",
    default=[],
    help=f"Remote log root (repeatable). Default: {CANONICAL_REMOTE_ROOT}",
  )
  parser.add_argument(
    "--file-name",
    action="append",
    dest="file_names",
    default=[],
    help="File name to sync (repeatable). Defaults to qlog + qlog.bz2 + qlog.zst",
  )
  parser.add_argument("--include-rlog", action="store_true", help="Also sync rlog + rlog.bz2 + rlog.zst")
  parser.add_argument("--download-root", default=str(DEFAULT_DOWNLOAD_ROOT), help=f"Local download root. Default: {DEFAULT_DOWNLOAD_ROOT}")
  parser.add_argument("--state-file", default=str(DEFAULT_STATE_FILE), help=f"State JSON path. Default: {DEFAULT_STATE_FILE}")
  parser.add_argument("--report-file", default=None, help="Write JSON report to this path (default: timestamped file under report dir)")
  parser.add_argument("--report-dir", default=str(DEFAULT_REPORT_DIR), help=f"Directory for autogenerated reports. Default: {DEFAULT_REPORT_DIR}")
  parser.add_argument("--connect-timeout", type=int, default=8, help="SSH/SCP connect timeout in seconds")
  parser.add_argument("--max-downloads", type=int, default=0, help="Maximum number of files to download this run (0 = no limit)")
  parser.add_argument("--newest-first", action="store_true", help="Download candidates by newest mtime first (useful with --max-downloads)")
  parser.add_argument("--spread-routes", action="store_true", help="Interleave download candidates across routes (best used with --max-downloads)")
  parser.add_argument("--dry-run", action="store_true", help="Discover/compare only, do not download")
  parser.add_argument("--verbose", action="store_true", help="Print per-file actions")
  return parser.parse_args()


def main() -> int:
  args = parse_args()
  remote_roots = args.remote_roots or DEFAULT_REMOTE_ROOTS
  file_names = args.file_names or DEFAULT_FILE_NAMES
  if args.include_rlog:
    for name in RLOG_FILE_NAMES:
      if name not in file_names:
        file_names.append(name)

  download_root = Path(args.download_root).expanduser()
  state_file = Path(args.state_file).expanduser()
  report_dir = Path(args.report_dir).expanduser()
  report_path = build_report_path(args.report_file, report_dir, args.host)

  report: dict[str, Any] = {
    "timestamp_utc": utc_now_iso(),
    "host": args.host,
    "ssh_host": args.host,
    "cache_host": canonical_cache_host(args.host),
    "remote_roots": remote_roots,
    "file_names": file_names,
    "dry_run": bool(args.dry_run),
    "newest_first": bool(args.newest_first),
    "spread_routes": bool(args.spread_routes),
    "state_file": str(state_file),
    "download_root": str(download_root),
    "report_file": str(report_path),
    "counts": {
      "remote_files": 0,
      "new_files": 0,
      "changed_files": 0,
      "download_candidates": 0,
      "downloaded": 0,
      "download_failures": 0,
      "skipped_due_to_limit": 0,
      "unchanged": 0,
    },
    "new_routes": [],
    "new_segments": [],
    "downloaded_files": [],
    "errors": [],
  }

  ssh_host = args.host
  try:
    remote_files = list_remote_files(ssh_host, remote_roots, file_names, args.connect_timeout)
  except Exception as exc:  # explicit top-level error capture for reporting
    if args.host == DEFAULT_HOST:
      try:
        remote_files = list_remote_files(FALLBACK_HOST, remote_roots, file_names, args.connect_timeout)
        ssh_host = FALLBACK_HOST
        print(f"[route-refresh] {DEFAULT_HOST} unavailable, falling back to {FALLBACK_HOST}", file=sys.stderr)
      except Exception as fallback_exc:
        report["errors"].append(str(exc))
        report["errors"].append(f"fallback {FALLBACK_HOST}: {fallback_exc}")
        report_path.parent.mkdir(parents=True, exist_ok=True)
        report_path.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")
        print(f"[route-refresh] {exc}", file=sys.stderr)
        print(f"[route-refresh] report: {report_path}")
        return 2
    else:
      report["errors"].append(str(exc))
      report_path.parent.mkdir(parents=True, exist_ok=True)
      report_path.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")
      print(f"[route-refresh] {exc}", file=sys.stderr)
      print(f"[route-refresh] report: {report_path}")
      return 2

  report["ssh_host"] = ssh_host
  state = load_state(state_file)
  hosts = state.setdefault("hosts", {})
  cache_host, host_state = merge_host_states(hosts, args.host)
  file_state: dict[str, Any] = host_state.setdefault("files", {})
  report["cache_host"] = cache_host

  report["counts"]["remote_files"] = len(remote_files)

  to_download: list[RemoteFile] = []
  new_routes: set[str] = set()
  new_segments: set[str] = set()

  for remote_file in remote_files:
    prior = file_state.get(remote_file.remote_path)
    local_path = adopt_existing_alias_file(download_root, args.host, remote_file.remote_path)
    local_exists = local_path.exists()

    if prior is None:
      report["counts"]["new_files"] += 1
      to_download.append(remote_file)
      new_routes.add(remote_file.route)
      new_segments.add(remote_file.segment)
      continue

    changed = (
      int(prior.get("size", -1)) != remote_file.size
      or int(prior.get("mtime", -1)) != remote_file.mtime
      or not local_exists
    )
    if changed:
      report["counts"]["changed_files"] += 1
      to_download.append(remote_file)
      continue

    report["counts"]["unchanged"] += 1
    prior["last_seen_utc"] = report["timestamp_utc"]

  report["counts"]["download_candidates"] = len(to_download)
  if args.newest_first:
    to_download.sort(key=lambda item: (item.mtime, item.remote_path), reverse=True)
  if args.spread_routes:
    to_download = interleave_by_route(to_download, newest_first=args.newest_first)

  report["new_routes"] = sorted(new_routes)
  report["new_segments"] = sorted(new_segments)

  downloaded_entries: list[dict[str, Any]] = []
  max_downloads = args.max_downloads if args.max_downloads > 0 else None

  for index, remote_file in enumerate(to_download):
    if max_downloads is not None and index >= max_downloads:
      report["counts"]["skipped_due_to_limit"] += 1
      continue

    local_path = local_path_for(download_root, cache_host, remote_file.remote_path)
    if args.verbose:
      print(f"[route-refresh] downloading {remote_file.remote_path} -> {local_path}")

    if args.dry_run:
      continue

    try:
      download_file(ssh_host, remote_file.remote_path, local_path, args.connect_timeout)
    except Exception as exc:
      if args.host == DEFAULT_HOST and ssh_host == DEFAULT_HOST:
        try:
          download_file(FALLBACK_HOST, remote_file.remote_path, local_path, args.connect_timeout)
          ssh_host = FALLBACK_HOST
          report["ssh_host"] = ssh_host
          print(f"[route-refresh] {DEFAULT_HOST} download failed, switching to {FALLBACK_HOST}", file=sys.stderr)
        except Exception:
          report["counts"]["download_failures"] += 1
          report["errors"].append(f"{remote_file.remote_path}: {exc}")
          continue
      else:
        report["counts"]["download_failures"] += 1
        report["errors"].append(f"{remote_file.remote_path}: {exc}")
        continue

    entry = {
      "remote_path": remote_file.remote_path,
      "local_path": str(local_path),
      "size": remote_file.size,
      "mtime": remote_file.mtime,
      "segment": remote_file.segment,
      "route": remote_file.route,
      "synced_utc": report["timestamp_utc"],
    }
    downloaded_entries.append(entry)
    report["counts"]["downloaded"] += 1

    file_state[remote_file.remote_path] = {
      "size": remote_file.size,
      "mtime": remote_file.mtime,
      "local_path": str(local_path),
      "first_synced_utc": file_state.get(remote_file.remote_path, {}).get("first_synced_utc", report["timestamp_utc"]),
      "last_synced_utc": report["timestamp_utc"],
      "last_seen_utc": report["timestamp_utc"],
      "segment": remote_file.segment,
      "route": remote_file.route,
    }

  report["downloaded_files"] = downloaded_entries

  if not args.dry_run:
    save_state(state_file, state)
  report_path.parent.mkdir(parents=True, exist_ok=True)
  report_path.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")

  host_line = f"[route-refresh] host={args.host}"
  if ssh_host != args.host:
    host_line += f" (ssh={ssh_host})"
  print(host_line)
  print(f"[route-refresh] remote files: {report['counts']['remote_files']}")
  print(
    f"[route-refresh] new={report['counts']['new_files']} changed={report['counts']['changed_files']} "
    f"unchanged={report['counts']['unchanged']}"
  )
  print(
    f"[route-refresh] downloaded={report['counts']['downloaded']} failures={report['counts']['download_failures']} "
    f"skipped={report['counts']['skipped_due_to_limit']}"
  )
  print(f"[route-refresh] report: {report_path}")
  return 0 if report["counts"]["download_failures"] == 0 else 1


if __name__ == "__main__":
  raise SystemExit(main())
