#!/usr/bin/env python3
"""Refresh newly discovered route log files from a comma device via SSH."""

from __future__ import annotations

import argparse
import fcntl
import json
import math
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
  LEGACY_DOWNLOAD_ROOT,
  LEGACY_REPORT_DIR,
  LEGACY_STATE_FILE,
  RLOG_FILE_NAMES,
  build_report_path,
  cache_remote_path_aliases,
  cache_host_aliases,
  canonical_cache_host,
  canonicalize_remote_path_for_cache,
  legacy_host_local_path_for,
  local_path_for,
  raw_local_path_for,
  utc_now_iso,
)

PROTECTION_IDLE_SECONDS = 24 * 3600


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
    dir="${{f%/*}}"
    set -- "$dir"/*.lock
    [ -e "$1" ] && continue
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
      raise RuntimeError(f"Invalid remote log metadata from {host}: {line}")

    remote_path, size_raw, mtime_raw = parts[0], parts[1], parts[2]
    try:
      size = int(size_raw)
      mtime = int(float(mtime_raw))
    except ValueError as exc:
      raise RuntimeError(f"Invalid remote log metadata from {host}: {line}") from exc
    if (size_raw, mtime_raw) == ("-1", "-1"):
      continue  # the device deleter removed the file between find and stat
    if size < 0 or mtime < 0:
      raise RuntimeError(f"Invalid remote log metadata from {host}: {line}")

    segment, route = derive_segment_and_route(remote_path)
    files.append(RemoteFile(remote_path=remote_path, size=size, mtime=mtime, segment=segment, route=route))

  files.sort(key=lambda item: item.remote_path)
  return files


def load_state(state_file: Path) -> dict[str, Any]:
  if not state_file.exists():
    return {"version": 1, "hosts": {}}

  data = json.loads(state_file.read_text())
  if not isinstance(data, dict) or not isinstance(data.get("hosts", {}), dict):
    raise ValueError(f"Invalid route-sync state: {state_file}")

  data.setdefault("version", 1)
  data.setdefault("hosts", {})
  return data


def save_state(state_file: Path, state: dict[str, Any]) -> None:
  state_file.parent.mkdir(parents=True, exist_ok=True)
  temporary = state_file.with_name(f"{state_file.name}.tmp_{os.getpid()}")
  temporary.write_text(json.dumps(state, indent=2, sort_keys=True) + "\n")
  temporary.replace(state_file)


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
    host_state["retention_cutoff"] = max(host_state.get("retention_cutoff", [0, ""]), alias_state.get("retention_cutoff", [0, ""]))
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


def adopt_existing_alias_file(download_root: Path, host: str, remote_path: str, dry_run: bool = False) -> Path:
  cache_host = canonical_cache_host(host)
  canonical_path = local_path_for(download_root, cache_host, remote_path)
  if canonical_path.exists():
    return canonical_path

  roots_to_check = [download_root]
  if download_root.expanduser().resolve() == DEFAULT_DOWNLOAD_ROOT.resolve():
    roots_to_check.append(LEGACY_DOWNLOAD_ROOT)

  for root in roots_to_check:
    for alias in cache_host_aliases(host):
      for alias_remote_path in cache_remote_path_aliases(remote_path):
        candidate_paths = [
          raw_local_path_for(root, alias, alias_remote_path),
          legacy_host_local_path_for(root, alias, alias_remote_path),
        ]
        for alias_path in candidate_paths:
          if alias_path == canonical_path or not alias_path.exists():
            continue

          if dry_run:
            return alias_path
          canonical_path.parent.mkdir(parents=True, exist_ok=True)
          try:
            alias_path.replace(canonical_path)
          except OSError:
            shutil.copy2(alias_path, canonical_path)
            alias_path.unlink()
          return canonical_path

  return canonical_path


def _migrated_relative_path(relative_path: Path) -> Path:
  rel_str = str(relative_path).lstrip("/")
  if rel_str.startswith("data/media/0/"):
    canonical_remote = canonicalize_remote_path_for_cache(f"/{rel_str}")
    return Path(canonical_remote.lstrip("/"))
  return Path("data/media/0/realdata") / relative_path


def _migrate_tree_contents(src_root: Path, dst_root: Path) -> None:
  if not src_root.exists():
    return

  file_paths = sorted((path for path in src_root.rglob("*") if path.is_file()), key=lambda path: len(path.parts), reverse=True)
  for path in file_paths:
    relative_path = path.relative_to(src_root)
    target = dst_root / _migrated_relative_path(relative_path)
    if target.exists():
      continue
    target.parent.mkdir(parents=True, exist_ok=True)
    path.replace(target)

  for path in sorted((item for item in src_root.rglob("*") if item.is_dir()), key=lambda item: len(item.parts), reverse=True):
    try:
      path.rmdir()
    except OSError:
      pass
  try:
    src_root.rmdir()
  except OSError:
    pass


def _migrate_legacy_route_sync_layout(download_root: Path, state_file: Path, report_dir: Path) -> None:
  if download_root.resolve() != DEFAULT_DOWNLOAD_ROOT.resolve():
    return

  DEFAULT_DOWNLOAD_ROOT.mkdir(parents=True, exist_ok=True)
  DEFAULT_REPORT_DIR.mkdir(parents=True, exist_ok=True)

  if LEGACY_DOWNLOAD_ROOT.exists():
    for alias_root in cache_host_aliases(DEFAULT_HOST):
      _migrate_tree_contents(LEGACY_DOWNLOAD_ROOT / alias_root, DEFAULT_DOWNLOAD_ROOT)

  for alias_root in cache_host_aliases(DEFAULT_HOST):
    _migrate_tree_contents(DEFAULT_DOWNLOAD_ROOT / alias_root, DEFAULT_DOWNLOAD_ROOT)

  if state_file == DEFAULT_STATE_FILE and not state_file.exists() and LEGACY_STATE_FILE.exists():
    state_file.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(LEGACY_STATE_FILE, state_file)

  if state_file.exists():
    try:
      state = load_state(state_file)
      hosts = state.get("hosts", {})
      if isinstance(hosts, dict):
        updated = False
        for host_state in hosts.values():
          if not isinstance(host_state, dict):
            continue
          files = host_state.get("files", {})
          if not isinstance(files, dict):
            continue
          for remote_path, entry in files.items():
            if not isinstance(entry, dict):
              continue
            new_local_path = str(local_path_for(DEFAULT_DOWNLOAD_ROOT, DEFAULT_HOST, str(remote_path)))
            if entry.get("local_path") != new_local_path:
              entry["local_path"] = new_local_path
              updated = True
        if updated:
          save_state(state_file, state)
    except Exception:
      pass

  if report_dir == DEFAULT_REPORT_DIR and LEGACY_REPORT_DIR.exists():
    report_dir.mkdir(parents=True, exist_ok=True)
    for path in sorted(LEGACY_REPORT_DIR.iterdir()):
      target = report_dir / path.name
      if target.exists():
        continue
      if path.is_file():
        shutil.copy2(path, target)


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


def managed_log(remote_path: str) -> bool:
  path = Path(canonicalize_remote_path_for_cache(remote_path))
  route, separator, number = path.parent.name.rpartition("--")
  return (str(path.parent.parent) == CANONICAL_REMOTE_ROOT and path.name in DEFAULT_FILE_NAMES + RLOG_FILE_NAMES
          and bool(route and separator and number.isdigit()))


def plan_retention(download_root: Path, host_state: dict[str, Any], remote_files: list[RemoteFile],
                   keep_days: float, max_bytes: int, max_protected_bytes: int = 10 * 2**30,
                   listed: list[RemoteFile] = ()) -> dict[str, Any]:
  """Plan whole-route log eviction from disk plus prospective downloads (remote_files); never trust state local_path values.

  listed supplies source mtimes for cached logs without reserving space for them."""
  root = download_root / CANONICAL_REMOTE_ROOT.lstrip("/")
  if any(p.is_symlink() for p in (root, *root.parents)):
    raise ValueError(f"Refusing retention through a symlink: {root}")
  names = set(DEFAULT_FILE_NAMES + RLOG_FILE_NAMES)
  metadata: dict[str, dict[str, Any]] = {}
  for p, entry in host_state.get("files", {}).items():
    key = canonicalize_remote_path_for_cache(p)
    if entry.get("mtime", -1) >= metadata.get(key, {}).get("mtime", -1):
      metadata[key] = entry
  files: dict[str, tuple[int, float]] = {}
  local_files: dict[str, list[dict[str, Any]]] = defaultdict(list)
  markers: dict[str, list[dict[str, Any]]] = defaultdict(list)
  activity: dict[str, float] = defaultdict(float)
  unsafe: set[str] = set()
  for segment in root.iterdir() if root.exists() else []:
    route, separator, number = segment.name.rpartition("--")
    if not separator or not number.isdigit():
      continue
    if segment.is_symlink():
      unsafe.add(route)
      continue
    if not segment.is_dir():
      continue
    children = list(segment.iterdir())
    segment_stat = segment.stat()
    activity[route] = max(activity[route], segment_stat.st_mtime, segment_stat.st_ctime)
    if any(p.is_symlink() for p in children):
      unsafe.add(route)
    for path in children:
      if path.is_symlink() or not path.is_file():
        continue
      stat = path.stat()
      # ctime records local writes/copies even when a downloader preserves an old source mtime.
      activity[route] = max(activity[route], stat.st_mtime, stat.st_ctime)
      entry = {"path": str(path), "bytes": stat.st_size, "mtime_ns": stat.st_mtime_ns, "ctime_ns": stat.st_ctime_ns}
      partial = any(path.name.startswith(f"{name}.partial") for name in names)
      if partial or path.name.endswith(".lock"):
        markers[route].append(entry)
      if path.name not in names:
        continue
      remote_path = f"{CANONICAL_REMOTE_ROOT}/{segment.name}/{path.name}"
      mtime = metadata.get(remote_path, {}).get("mtime", stat.st_mtime)
      files[remote_path] = (stat.st_size, mtime)
      local_files[route].append(entry)

  # A file listed under several roots keeps its newest source mtime.
  listed_mtime: dict[str, float] = {}
  for item in listed:
    path = canonicalize_remote_path_for_cache(item.remote_path)
    listed_mtime[path] = max(listed_mtime.get(path, -1), item.mtime)
  for path, mtime in listed_mtime.items():
    if path in files:
      files[path] = (files[path][0], max(mtime, metadata.get(path, {}).get("mtime", -1)))
  for item in remote_files:
    path = Path(canonicalize_remote_path_for_cache(item.remote_path))
    if not managed_log(str(path)):
      continue
    # Reserve enough space for either the existing file or its replacement.
    size, mtime = files.get(str(path), (0, -1))
    files[str(path)] = (max(item.size, size), max(item.mtime, mtime))

  routes: dict[str, dict[str, Any]] = {}
  for path, (size, mtime) in files.items():
    _, route = derive_segment_and_route(path)
    row = routes.setdefault(route, {"route": route, "bytes": 0, "mtime": mtime})
    row["bytes"] += size
    row["mtime"] = max(row["mtime"], mtime)

  now = time.time()  # noqa: TID251 - filesystem activity and source times are Unix timestamps
  protected = {route for route in markers if activity[route] >= now - PROTECTION_IDLE_SECONDS}
  expired = set(markers) - protected
  for route in protected:
    entries = markers[route]
    # Partial files use space but must not make an old completed route look newer.
    row = routes.setdefault(route, {"route": route, "bytes": 0, "mtime": max(e["mtime_ns"] for e in entries) / 1e9})
    row["bytes"] += sum(entry["bytes"] for entry in entries)
  cutoff = tuple(host_state.get("retention_cutoff", [0, ""]))
  age_cutoff = now - keep_days * 86400
  excluded: set[str] = set()
  for route, row in routes.items():
    # The eviction cutoff only blocks re-downloads; cached logs follow the age and size limits.
    if route not in protected and (row["mtime"] < age_cutoff or (route not in local_files and (row["mtime"], route) <= cutoff)):
      excluded.add(route)
  protected_bytes = sum(row["bytes"] for route, row in routes.items() if route in protected)
  total = sum(row["bytes"] for route, row in routes.items() if route not in excluded and route not in protected)
  ordered = sorted((row for route, row in routes.items() if route not in excluded and route not in protected),
                   key=lambda row: (row["mtime"], row["route"]))
  required_bytes = ordered[-1]["bytes"] if ordered else 0
  # Protected data has its own budget; keep the newest ordinary route when it alone cannot fit.
  for row in ordered if required_bytes <= max_bytes else []:
    route = row["route"]
    if total <= max_bytes:
      break
    excluded.add(route)
    total -= row["bytes"]
    cutoff = max(cutoff, (row["mtime"], route))

  prune_files = ([entry for route in sorted(excluded) for entry in local_files[route]] +
                 [entry for route in sorted(expired) for entry in markers[route]])
  return {
    "keep_days": keep_days, "max_bytes": max_bytes, "max_protected_bytes": max_protected_bytes, "cutoff": list(cutoff),
    "excluded_routes": sorted(excluded), "protected_routes": sorted(protected), "unsafe_routes": sorted(unsafe),
    "protection_idle_hours": PROTECTION_IDLE_SECONDS / 3600, "expired_protection_routes": sorted(expired),
    "protected_bytes": protected_bytes, "ordinary_bytes": total,
    "files": prune_files, "prune_bytes": sum(entry["bytes"] for entry in prune_files),
    "local_bytes_before": (sum(entry["bytes"] for entries in local_files.values() for entry in entries) +
                           sum(entry["bytes"] for entries in markers.values() for entry in entries)),
    "projected_bytes": total + protected_bytes,
    "ordinary_over_budget_bytes": max(0, total - max_bytes),
    "protected_over_budget_bytes": max(0, protected_bytes - max_protected_bytes),
    "over_budget_bytes": max(0, total - max_bytes) + max(0, protected_bytes - max_protected_bytes),
    "pruned_files": 0, "pruned_bytes": 0,
  }


def apply_retention(plan: dict[str, Any], download_root: Path, host_state: dict[str, Any]) -> None:
  # Refresh holds the cache lock. Also refuse changed files and uncoordinated active writers.
  current = plan_retention(download_root, host_state, [], plan["keep_days"], plan["max_bytes"], plan["max_protected_bytes"])
  deleting_routes = {derive_segment_and_route(entry["path"])[1] for entry in plan["files"]}
  if deleting_routes.intersection(current["protected_routes"]):
    raise RuntimeError("Route became active during retention")
  for entry in plan["files"]:
    path = Path(entry["path"])
    if any(p.is_symlink() for p in (path, *path.parents)):
      raise ValueError(f"Refusing to prune a symlink: {path}")
    stat = path.stat()
    if (stat.st_size, stat.st_mtime_ns, stat.st_ctime_ns) != (entry["bytes"], entry["mtime_ns"], entry["ctime_ns"]):
      raise RuntimeError(f"Log changed during retention: {path}")
  for entry in plan["files"]:
    path = Path(entry["path"])
    path.unlink()
    plan["pruned_files"] += 1
    plan["pruned_bytes"] += entry["bytes"]
    if not any(path.parent.iterdir()):
      path.parent.rmdir()
  host_state["retention_cutoff"] = plan["cutoff"]
  file_state = host_state.setdefault("files", {})
  for remote_path in list(file_state):
    path = local_path_for(download_root, DEFAULT_HOST, remote_path)
    if managed_log(remote_path) and not path.exists():
      del file_state[remote_path]


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
    help="Remote log root (repeatable). Default: shared active roots under /data/media/0",
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
  parser.add_argument("--keep-days", type=float, default=30, help="Keep routes whose newest log is within this many days (default: 30)")
  parser.add_argument("--max-cache-gib", type=float, default=30, help="Ordinary route log budget in GiB, oldest routes first (default: 30)")
  parser.add_argument("--max-protected-gib", type=float, default=10, help="Separate protected route budget in GiB (default: 10)")
  parser.add_argument("--prune-only", action="store_true", help="Apply local retention without contacting the device")
  parser.add_argument("--dry-run", action="store_true", help="Write a plan report without downloading, pruning, migrating, or changing state")
  parser.add_argument("--verbose", action="store_true", help="Print per-file actions")
  args = parser.parse_args()
  if any(not math.isfinite(value) or value <= 0 for value in (args.keep_days, args.max_cache_gib, args.max_protected_gib)):
    parser.error("--keep-days, --max-cache-gib and --max-protected-gib must be finite and greater than zero")
  return args


def fallback_host_for(host: str) -> str | None:
  if host == DEFAULT_HOST:
    return FALLBACK_HOST
  if host == FALLBACK_HOST:
    return DEFAULT_HOST
  return None


def main() -> int:
  args = parse_args()
  download_root = Path(args.download_root).expanduser().resolve()
  args.download_root = str(download_root)
  download_root.mkdir(parents=True, exist_ok=True)
  with (download_root / ".refresh.lock").open("a") as lock:
    try:
      fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
      print("[route-refresh] another refresh/prune is running", file=sys.stderr)
      return 2
    try:
      # Check before migration can change either the cache or the state.
      load_state(Path(args.state_file).expanduser())
      return refresh(args)
    except (OSError, ValueError) as exc:
      print(f"[route-refresh] {exc}", file=sys.stderr)
      return 2


def refresh(args: argparse.Namespace) -> int:
  remote_roots = args.remote_roots or DEFAULT_REMOTE_ROOTS
  file_names = list(args.file_names or DEFAULT_FILE_NAMES)
  if args.include_rlog:
    for name in RLOG_FILE_NAMES:
      if name not in file_names:
        file_names.append(name)

  download_root = Path(args.download_root).expanduser()
  state_file = Path(args.state_file).expanduser()
  report_dir = Path(args.report_dir).expanduser()
  if not args.dry_run:
    _migrate_legacy_route_sync_layout(download_root, state_file, report_dir)
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
      "skipped_due_to_protected_limit": 0,
      "unchanged": 0,
    },
    "new_routes": [],
    "new_segments": [],
    "downloaded_files": [],
    "errors": [],
  }

  ssh_host = args.host
  try:
    remote_files = [] if args.prune_only else list_remote_files(ssh_host, remote_roots, file_names, args.connect_timeout)
  except Exception as exc:  # explicit top-level error capture for reporting
    fallback_host = fallback_host_for(args.host)
    if fallback_host is not None:
      try:
        remote_files = list_remote_files(fallback_host, remote_roots, file_names, args.connect_timeout)
        ssh_host = fallback_host
        print(f"[route-refresh] {args.host} unavailable, falling back to {fallback_host}", file=sys.stderr)
      except Exception as fallback_exc:
        report["errors"].append(str(exc))
        report["errors"].append(f"fallback {fallback_host}: {fallback_exc}")
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
  protected_limit = int(args.max_protected_gib * 2**30)
  try:
    # First filter age and past evictions; apply the size limit after --max-downloads selection.
    retention = plan_retention(download_root, host_state, remote_files, args.keep_days, sys.maxsize, protected_limit, listed=remote_files)
  except (OSError, ValueError) as exc:
    print(f"[route-refresh] retention failed: {exc}", file=sys.stderr)
    return 2
  report["retention"] = retention
  excluded_routes = set(retention["excluded_routes"] + retention["unsafe_routes"])
  eligible_files = [item for item in remote_files if not managed_log(item.remote_path) or item.route not in excluded_routes]
  report["counts"]["skipped_due_to_retention"] = len(remote_files) - len(eligible_files)

  to_download: list[RemoteFile] = []
  new_routes: set[str] = set()
  new_segments: set[str] = set()

  for remote_file in eligible_files:
    prior = file_state.get(remote_file.remote_path)
    local_path = adopt_existing_alias_file(download_root, args.host, remote_file.remote_path, args.dry_run)
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
    prior["size"] = remote_file.size
    prior["mtime"] = remote_file.mtime
    prior["segment"] = remote_file.segment
    prior["route"] = remote_file.route
    prior["local_path"] = str(local_path)
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
  local_retention = plan_retention(download_root, host_state, [], args.keep_days, sys.maxsize, protected_limit)
  protected_routes = set(local_retention["protected_routes"])
  reserved_protected_bytes = local_retention["protected_bytes"]
  reserved_sizes: dict[Path, int] = {}
  selected: list[RemoteFile] = []
  for remote_file in to_download:
    if max_downloads is not None and len(selected) >= max_downloads:
      report["counts"]["skipped_due_to_limit"] += 1
      continue
    if managed_log(remote_file.remote_path) and remote_file.route in protected_routes:
      path = local_path_for(download_root, cache_host, remote_file.remote_path)
      if path not in reserved_sizes:
        reserved_sizes[path] = path.stat().st_size if path.exists() else 0
      growth = max(0, remote_file.size - reserved_sizes[path])
      if growth > 0 and reserved_protected_bytes + growth > protected_limit:
        report["counts"]["skipped_due_to_protected_limit"] += 1
        continue
      reserved_protected_bytes += growth
      reserved_sizes[path] += growth
    selected.append(remote_file)
  retention = plan_retention(download_root, host_state, selected, args.keep_days, int(args.max_cache_gib * 2**30), protected_limit,
                             listed=remote_files)
  report["retention"] = retention
  excluded_routes = set(retention["excluded_routes"] + retention["unsafe_routes"])
  if retention["ordinary_over_budget_bytes"]:
    report["errors"].append("Ordinary retention limit cannot fit the newest route; no downloads or pruning applied")

  for remote_file in selected:
    if managed_log(remote_file.remote_path) and remote_file.route in excluded_routes:
      report["counts"]["skipped_due_to_retention"] += 1
      continue
    if retention["ordinary_over_budget_bytes"]:
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

    os.utime(local_path, (remote_file.mtime, remote_file.mtime))
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
    if not report["errors"]:
      try:
        # Persist the decision before unlinking so an interrupted prune does not re-download routes it evicted completely.
        host_state["retention_cutoff"] = retention["cutoff"]
        save_state(state_file, state)
        apply_retention(retention, download_root, host_state)
      except (OSError, ValueError, RuntimeError) as exc:
        report["errors"].append(f"retention: {exc}")
    save_state(state_file, state)
  report_path.parent.mkdir(parents=True, exist_ok=True)
  report_path.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")

  host_line = f"[route-refresh] host={args.host}"
  if ssh_host != args.host:
    host_line += f" (ssh={ssh_host})"
  print(host_line)
  print(f"[route-refresh] remote files: {report['counts']['remote_files']}")
  print(
    f"[route-refresh] new={report['counts']['new_files']} changed={report['counts']['changed_files']} " +
    f"unchanged={report['counts']['unchanged']}"
  )
  print(
    f"[route-refresh] downloaded={report['counts']['downloaded']} failures={report['counts']['download_failures']} " +
    f"skipped={report['counts']['skipped_due_to_limit']}"
  )
  print(f"[route-refresh] report: {report_path}")
  action = "would prune" if args.dry_run else "pruned"
  size = retention["prune_bytes"] if args.dry_run else retention["pruned_bytes"]
  print(f"[route-refresh] {action}: {size / 2**30:.2f} GiB; projected logs: {retention['projected_bytes'] / 2**30:.2f} GiB")
  print(f"[route-refresh] protected={len(retention['protected_routes'])} ({retention['protected_bytes'] / 2**30:.2f} GiB); " +
        f"expired protection={len(retention['expired_protection_routes'])}")
  print(f"[route-refresh] ordinary={retention['ordinary_bytes'] / 2**30:.2f}/{args.max_cache_gib:g} GiB; " +
        f"protected={retention['protected_bytes'] / 2**30:.2f}/{args.max_protected_gib:g} GiB; " +
        f"protected downloads deferred={report['counts']['skipped_due_to_protected_limit']}")
  if retention["ordinary_over_budget_bytes"]:
    print("[route-refresh] newest ordinary route exceeds its limit; no downloads or pruning applied", file=sys.stderr)
  if retention["protected_over_budget_bytes"]:
    print("[route-refresh] existing protected data exceeds its separate limit; preserved until idle, ordinary budget unchanged", file=sys.stderr)
  return 1 if report["errors"] or retention["over_budget_bytes"] else 0


if __name__ == "__main__":
  raise SystemExit(main())
