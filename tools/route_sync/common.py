from __future__ import annotations

from datetime import datetime, timezone
from pathlib import Path

DEFAULT_REMOTE_ROOTS = [
  "/data/media/0/realdata",
  "/data/media/0/realdata_HD",
  "/data/media/0/realdata_konik",
]
DEFAULT_FILE_NAMES = ["qlog", "qlog.bz2", "qlog.zst"]
RLOG_FILE_NAMES = ["rlog", "rlog.bz2", "rlog.zst"]
DEFAULT_DOWNLOAD_ROOT = Path.home() / ".comma" / "route_sync" / "downloads"
DEFAULT_STATE_FILE = Path.home() / ".comma" / "route_sync" / "state.json"
DEFAULT_REPORT_DIR = Path.home() / ".comma" / "route_sync" / "reports"
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


def local_path_for(download_root: Path, host: str, remote_path: str) -> Path:
  return download_root / host / remote_path.lstrip("/")


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
