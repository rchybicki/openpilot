from __future__ import annotations

from datetime import datetime, timezone
from pathlib import Path

CANONICAL_REMOTE_ROOT = "/data/media/0/realdata"
LEGACY_REMOTE_ROOTS = [
  "/data/media/0/realdata_HD",
  "/data/media/0/realdata_konik",
]
DEFAULT_REMOTE_ROOTS = [CANONICAL_REMOTE_ROOT, *LEGACY_REMOTE_ROOTS]
DEFAULT_FILE_NAMES = ["qlog", "qlog.bz2", "qlog.zst"]
RLOG_FILE_NAMES = ["rlog", "rlog.bz2", "rlog.zst"]
DEFAULT_ROUTE_SYNC_ROOT = Path.home() / ".route_sync"
LEGACY_ROUTE_SYNC_ROOT = Path.home() / ".comma" / "route_sync"
DEFAULT_DOWNLOAD_ROOT = DEFAULT_ROUTE_SYNC_ROOT
DEFAULT_STATE_FILE = DEFAULT_ROUTE_SYNC_ROOT / "state.json"
DEFAULT_REPORT_DIR = DEFAULT_ROUTE_SYNC_ROOT / "reports"
LEGACY_DOWNLOAD_ROOT = LEGACY_ROUTE_SYNC_ROOT / "downloads"
LEGACY_STATE_FILE = LEGACY_ROUTE_SYNC_ROOT / "state.json"
LEGACY_REPORT_DIR = LEGACY_ROUTE_SYNC_ROOT / "reports"
DEFAULT_HOST = "commawifi"
FALLBACK_HOST = "comma"
HOST_CACHE_KEYS = {
  "comma": DEFAULT_HOST,
  DEFAULT_HOST: DEFAULT_HOST,
}


def utc_now_iso() -> str:
  return datetime.now(timezone.utc).replace(microsecond=0).isoformat()


def build_report_path(report_file: str | None, report_dir: Path, host: str) -> Path:
  if report_file:
    return Path(report_file).expanduser()

  report_dir.mkdir(parents=True, exist_ok=True)
  stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
  return report_dir / f"route_refresh_{host}_{stamp}.json"


def canonicalize_remote_path_for_cache(remote_path: str) -> str:
  for root in [CANONICAL_REMOTE_ROOT, *LEGACY_REMOTE_ROOTS]:
    if remote_path == root:
      return CANONICAL_REMOTE_ROOT
    root_prefix = f"{root}/"
    if remote_path.startswith(root_prefix):
      return f"{CANONICAL_REMOTE_ROOT}/{remote_path[len(root_prefix):]}"
  return remote_path


def cache_remote_path_aliases(remote_path: str) -> list[str]:
  canonical = canonicalize_remote_path_for_cache(remote_path)
  aliases = [canonical]

  if canonical == CANONICAL_REMOTE_ROOT:
    suffix = ""
  elif canonical.startswith(f"{CANONICAL_REMOTE_ROOT}/"):
    suffix = canonical[len(CANONICAL_REMOTE_ROOT) + 1:]
  else:
    return aliases

  for root in [CANONICAL_REMOTE_ROOT, *LEGACY_REMOTE_ROOTS]:
    alias = f"{root}/{suffix}" if suffix else root
    if alias not in aliases:
      aliases.append(alias)
  return aliases


def raw_local_path_for(download_root: Path, host: str, remote_path: str) -> Path:
  del host
  return download_root / remote_path.lstrip("/")


def legacy_host_local_path_for(download_root: Path, host: str, remote_path: str) -> Path:
  return download_root / host / remote_path.lstrip("/")


def local_path_for(download_root: Path, host: str, remote_path: str) -> Path:
  return raw_local_path_for(download_root, host, canonicalize_remote_path_for_cache(remote_path))


def host_download_root(download_root: Path, host: str) -> Path:
  root = download_root.expanduser()
  if (root / "data").exists():
    return root

  legacy_host_root = root / canonical_cache_host(host)
  if legacy_host_root.exists():
    return legacy_host_root
  return root


def segment_has_active_lock(segment_dir: Path) -> bool:
  if not segment_dir.is_dir():
    return False
  try:
    return any(segment_dir.glob("*.lock"))
  except OSError:
    return False


def canonical_cache_host(host: str) -> str:
  return HOST_CACHE_KEYS.get(host, host)


def cache_host_aliases(host: str) -> list[str]:
  canonical = canonical_cache_host(host)
  aliases = [canonical]
  for alias, cache_key in HOST_CACHE_KEYS.items():
    if cache_key == canonical and alias not in aliases:
      aliases.append(alias)
  if host not in aliases:
    aliases.append(host)
  return aliases
