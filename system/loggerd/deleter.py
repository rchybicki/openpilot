#!/usr/bin/env python3
import os
import shutil
import threading

from openpilot.system.hardware.hw import Paths
from openpilot.common.swaglog import cloudlog
from openpilot.system.loggerd.config import get_available_bytes, get_available_percent
from openpilot.system.loggerd.xattr_cache import getxattr

MIN_BYTES = 5 * 1024 * 1024 * 1024
MIN_PERCENT = 10

DELETE_LAST = ['boot', 'crash']
LOG_ROOT_CANDIDATES = (
  "/data/media/0/realdata",
  "/data/media/0/realdata_konik",
  "/data/media/0/realdata_HD",
)

PRESERVE_ATTR_NAME = 'user.preserve'
PRESERVE_ATTR_VALUE = b'1'
PRESERVE_COUNT = 5


def get_managed_log_roots() -> list[str]:
  active_root = Paths.log_root().rstrip("/")
  if not os.path.isdir(active_root):
    return []

  try:
    active_dev = os.stat(active_root).st_dev
  except OSError:
    return []

  roots = []
  for root in (active_root, *LOG_ROOT_CANDIDATES):
    root = root.rstrip("/")
    if root in roots or not os.path.isdir(root):
      continue

    try:
      if os.stat(root).st_dev != active_dev:
        continue
    except OSError:
      continue

    roots.append(root)

  return roots


def list_log_dirs(roots: list[str]) -> list[str]:
  paths = []
  for root in roots:
    try:
      for entry in os.listdir(root):
        path = os.path.join(root, entry)
        if os.path.isdir(path):
          paths.append(path)
    except OSError:
      cloudlog.exception("list_log_dirs failed", root=root)

  return sorted(paths, key=lambda path: os.path.getmtime(path))


def has_preserve_xattr(path: str) -> bool:
  return getxattr(path, PRESERVE_ATTR_NAME) == PRESERVE_ATTR_VALUE


def get_preserved_segments(dirs_by_mtime: list[str]) -> set[str]:
  # skip deleting most recent N preserved segments (and their prior segment)
  preserved = set()
  for n, path in enumerate(filter(has_preserve_xattr, reversed(dirs_by_mtime))):
    if n == PRESERVE_COUNT:
      break

    d = os.path.basename(path)
    date_str, _, seg_str = d.rpartition("--")

    # ignore non-segment directories
    if not date_str:
      continue
    try:
      seg_num = int(seg_str)
    except ValueError:
      continue

    # preserve segment and two prior
    root = os.path.dirname(path)
    for _seg_num in range(max(0, seg_num - 2), seg_num + 1):
      preserved.add(os.path.join(root, f"{date_str}--{_seg_num}"))

  return preserved


def deleter_thread(exit_event: threading.Event):
  while not exit_event.is_set():
    out_of_bytes = get_available_bytes(default=MIN_BYTES + 1) < MIN_BYTES
    out_of_percent = get_available_percent(default=MIN_PERCENT + 1) < MIN_PERCENT

    if out_of_percent or out_of_bytes:
      roots = get_managed_log_roots()
      dirs = list_log_dirs(roots)
      preserved_dirs = get_preserved_segments(dirs)

      # remove the earliest directory we can
      for delete_path in sorted(dirs, key=lambda path: (os.path.basename(path) in DELETE_LAST, path in preserved_dirs, os.path.getmtime(path))):
        delete_dir = os.path.basename(delete_path)

        if any(name.endswith(".lock") for name in os.listdir(delete_path)):
          continue

        try:
          cloudlog.info(f"deleting {delete_path}")
          shutil.rmtree(delete_path)
          break
        except OSError:
          cloudlog.exception(f"issue deleting {delete_path}")
      exit_event.wait(.1)
    else:
      exit_event.wait(30)


def main():
  deleter_thread(threading.Event())


if __name__ == "__main__":
  main()
