from __future__ import annotations

import json
import os
from pathlib import Path
import subprocess
import sys

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
  sys.path.insert(0, str(REPO_ROOT))

from openpilot.tools.route_sync.common import CANONICAL_REMOTE_ROOT
from openpilot.tools.route_sync.refresh_routes import (
  RemoteFile, apply_retention, build_remote_list_script, interleave_by_route, list_remote_files, main, plan_retention,
)


@pytest.fixture(autouse=True)
def retention_clock(monkeypatch):
  # Existing sync fixtures use small Unix timestamps.
  monkeypatch.setattr("openpilot.tools.route_sync.refresh_routes.time.time", lambda: 100)


def test_interleave_by_route_spreads_newest_segments_first() -> None:
  candidates = [
    RemoteFile("/r/route_a--1/qlog", size=1, mtime=90, segment="route_a--1", route="route_a"),
    RemoteFile("/r/route_a--0/qlog", size=1, mtime=100, segment="route_a--0", route="route_a"),
    RemoteFile("/r/route_b--1/qlog", size=1, mtime=70, segment="route_b--1", route="route_b"),
    RemoteFile("/r/route_b--0/qlog", size=1, mtime=80, segment="route_b--0", route="route_b"),
  ]

  ordered = interleave_by_route(candidates, newest_first=True)

  assert [item.remote_path for item in ordered] == [
    "/r/route_a--0/qlog",
    "/r/route_b--0/qlog",
    "/r/route_a--1/qlog",
    "/r/route_b--1/qlog",
  ]


def test_remote_list_script_skips_locked_live_segments(tmp_path: Path) -> None:
  root = tmp_path / "realdata"
  finalized = root / "route--0"
  live = root / "route--1"
  finalized.mkdir(parents=True)
  live.mkdir(parents=True)
  (finalized / "qlog.zst").write_bytes(b"done")
  (live / "qlog.zst").write_bytes(b"partial")
  (live / "rlog.lock").write_bytes(b"")

  script = build_remote_list_script([str(root)], ["qlog.zst"])
  result = subprocess.run(["sh"], input=script, capture_output=True, text=True, check=False)

  assert result.returncode == 0
  assert str(finalized / "qlog.zst") in result.stdout
  assert str(live / "qlog.zst") not in result.stdout


def test_main_refreshes_new_and_missing_local_files(monkeypatch, tmp_path: Path) -> None:
  host = "commawifi"
  download_root = tmp_path / "downloads"
  state_file = tmp_path / "state.json"
  report_file = tmp_path / "report.json"
  existing_remote = "/data/media/0/realdata/route_existing--0/qlog"
  new_remote = "/data/media/0/realdata/route_new--0/qlog.zst"

  state_file.write_text(json.dumps({
    "version": 1,
    "hosts": {
      host: {
        "files": {
          existing_remote: {
            "size": 111,
            "mtime": 10,
            "local_path": str(download_root / existing_remote.lstrip("/")),
            "first_synced_utc": "2026-03-01T00:00:00+00:00",
          },
        },
      },
    },
  }))

  remote_files = [
    RemoteFile(remote_path=existing_remote, size=111, mtime=10, segment="route_existing--0", route="route_existing"),
    RemoteFile(remote_path=new_remote, size=222, mtime=20, segment="route_new--0", route="route_new"),
  ]

  downloads: list[tuple[str, str, Path]] = []

  def fake_list_remote_files(*_args, **_kwargs):
    return remote_files

  def fake_download_file(ssh_host: str, remote_path: str, local_path: Path, connect_timeout: int) -> None:
    assert connect_timeout == 8
    local_path.parent.mkdir(parents=True, exist_ok=True)
    local_path.write_bytes(b"log")
    downloads.append((ssh_host, remote_path, local_path))

  monkeypatch.setattr("openpilot.tools.route_sync.refresh_routes.list_remote_files", fake_list_remote_files)
  monkeypatch.setattr("openpilot.tools.route_sync.refresh_routes.download_file", fake_download_file)
  monkeypatch.setattr(sys, "argv", [
    "refresh_routes.py",
    "--host",
    host,
    "--download-root",
    str(download_root),
    "--state-file",
    str(state_file),
    "--report-file",
    str(report_file),
  ])

  rc = main()

  assert rc == 0
  assert [entry[1] for entry in downloads] == [existing_remote, new_remote]

  report = json.loads(report_file.read_text())
  assert report["host"] == host
  assert report["download_root"] == str(download_root)
  assert report["counts"]["changed_files"] == 1
  assert report["counts"]["new_files"] == 1
  assert report["counts"]["downloaded"] == 2
  assert report["new_routes"] == ["route_new"]
  assert [entry["route"] for entry in report["downloaded_files"]] == ["route_existing", "route_new"]


def test_main_uses_only_canonical_remote_root_by_default(monkeypatch, tmp_path: Path) -> None:
  download_root = tmp_path / "downloads"
  state_file = tmp_path / "state.json"
  report_file = tmp_path / "report.json"

  def fake_list_remote_files(_host: str, remote_roots: list[str], *_args, **_kwargs):
    assert remote_roots == [CANONICAL_REMOTE_ROOT]
    return []

  monkeypatch.setattr("openpilot.tools.route_sync.refresh_routes.list_remote_files", fake_list_remote_files)
  monkeypatch.setattr(sys, "argv", [
    "refresh_routes.py",
    "--host",
    "comma",
    "--download-root",
    str(download_root),
    "--state-file",
    str(state_file),
    "--report-file",
    str(report_file),
  ])

  rc = main()

  assert rc == 0
  report = json.loads(report_file.read_text())
  assert report["remote_roots"] == [CANONICAL_REMOTE_ROOT]


def test_main_falls_back_to_comma_when_commawifi_listing_fails(monkeypatch, tmp_path: Path) -> None:
  report_file = tmp_path / "report.json"
  state_file = tmp_path / "state.json"
  download_root = tmp_path / "downloads"
  remote = RemoteFile(
    remote_path="/data/media/0/realdata/route_fallback--0/qlog",
    size=100,
    mtime=50,
    segment="route_fallback--0",
    route="route_fallback",
  )

  def fake_list_remote_files(host: str, *_args, **_kwargs):
    if host == "commawifi":
      raise RuntimeError("wifi down")
    assert host == "comma"
    return [remote]

  def fake_download_file(ssh_host: str, remote_path: str, local_path: Path, _connect_timeout: int) -> None:
    assert ssh_host == "comma"
    assert remote_path == remote.remote_path
    local_path.parent.mkdir(parents=True, exist_ok=True)
    local_path.write_bytes(b"log")

  monkeypatch.setattr("openpilot.tools.route_sync.refresh_routes.list_remote_files", fake_list_remote_files)
  monkeypatch.setattr("openpilot.tools.route_sync.refresh_routes.download_file", fake_download_file)
  monkeypatch.setattr(sys, "argv", [
    "refresh_routes.py",
    "--host",
    "commawifi",
    "--download-root",
    str(download_root),
    "--state-file",
    str(state_file),
    "--report-file",
    str(report_file),
  ])

  rc = main()

  assert rc == 0
  report = json.loads(report_file.read_text())
  assert report["host"] == "commawifi"
  assert report["ssh_host"] == "comma"
  assert report["counts"]["downloaded"] == 1
  assert report["downloaded_files"][0]["route"] == "route_fallback"


def test_main_shares_cache_between_comma_aliases(monkeypatch, tmp_path: Path) -> None:
  download_root = tmp_path / "downloads"
  state_file = tmp_path / "state.json"
  report_file = tmp_path / "report.json"
  remote_path = "/data/media/0/realdata/route_shared--0/qlog.zst"
  old_local_path = download_root / "comma" / remote_path.lstrip("/")
  old_local_path.parent.mkdir(parents=True, exist_ok=True)
  old_local_path.write_bytes(b"log")

  state_file.write_text(json.dumps({
    "version": 1,
    "hosts": {
      "comma": {
        "files": {
          remote_path: {
            "size": 123,
            "mtime": 45,
            "local_path": str(old_local_path),
            "first_synced_utc": "2026-03-01T00:00:00+00:00",
            "last_synced_utc": "2026-03-01T00:00:00+00:00",
          },
        },
      },
    },
  }))

  remote_files = [
    RemoteFile(remote_path=remote_path, size=123, mtime=45, segment="route_shared--0", route="route_shared"),
  ]
  downloads: list[tuple[str, str, Path]] = []

  def fake_list_remote_files(*_args, **_kwargs):
    return remote_files

  def fake_download_file(ssh_host: str, remote_path_arg: str, local_path: Path, connect_timeout: int) -> None:
    downloads.append((ssh_host, remote_path_arg, local_path))
    raise AssertionError("alias-shared file should not be downloaded again")

  monkeypatch.setattr("openpilot.tools.route_sync.refresh_routes.list_remote_files", fake_list_remote_files)
  monkeypatch.setattr("openpilot.tools.route_sync.refresh_routes.download_file", fake_download_file)
  monkeypatch.setattr(sys, "argv", [
    "refresh_routes.py",
    "--host",
    "commawifi",
    "--download-root",
    str(download_root),
    "--state-file",
    str(state_file),
    "--report-file",
    str(report_file),
  ])

  rc = main()

  canonical_local_path = download_root / remote_path.lstrip("/")
  assert rc == 0
  assert downloads == []
  assert canonical_local_path.exists()
  assert not old_local_path.exists()

  report = json.loads(report_file.read_text())
  assert report["host"] == "commawifi"
  assert report["cache_host"] == "comma"
  assert report["counts"]["unchanged"] == 1
  assert report["counts"]["downloaded"] == 0

  state = json.loads(state_file.read_text())
  assert "comma" in state["hosts"]
  assert remote_path in state["hosts"]["comma"]["files"]
  assert state["hosts"]["comma"]["files"][remote_path]["local_path"] == str(canonical_local_path)


def test_retention_prunes_whole_routes_and_untracked_logs(tmp_path, monkeypatch):
  monkeypatch.setattr("openpilot.tools.route_sync.refresh_routes.time.time", lambda: 40 * 86400)
  root = tmp_path / "data/media/0/realdata"
  for segment, mtime in [("old--0", 1), ("old--1", 2), ("recent--0", 1), ("recent--1", 40 * 86400)]:
    path = root / segment / "qlog.zst"
    path.parent.mkdir(parents=True)
    path.write_bytes(b"log")
    os.utime(path, (mtime, mtime))
  note = root / "old--0/note.txt"
  note.write_text("keep")
  state = {"files": {f"{CANONICAL_REMOTE_ROOT}/old--0/qlog.zst": {"mtime": 1, "local_path": str(note)}}}
  plan = plan_retention(tmp_path, state, [], 30, 100)
  assert plan["excluded_routes"] == ["old"]
  assert plan["prune_bytes"] == 6
  apply_retention(plan, tmp_path, state)
  assert note.read_text() == "keep"
  assert not (root / "old--1").exists()
  assert (root / "recent--0/qlog.zst").exists()
  assert state["files"] == {}


def test_capacity_cutoff_prevents_redownload_across_file_selections(tmp_path):
  old_qlog = RemoteFile(f"{CANONICAL_REMOTE_ROOT}/old--0/qlog", 1, 80, "old--0", "old")
  old_rlog = RemoteFile(f"{CANONICAL_REMOTE_ROOT}/old--0/rlog", 10, 80, "old--0", "old")
  recent = RemoteFile(f"{CANONICAL_REMOTE_ROOT}/recent--0/qlog", 5, 90, "recent--0", "recent")
  state = {"files": {}}
  plan = plan_retention(tmp_path, state, [old_qlog, old_rlog, recent], 30, 10)
  assert plan["excluded_routes"] == ["old"]
  assert plan["projected_bytes"] == 5
  apply_retention(plan, tmp_path, state)
  second = plan_retention(tmp_path, state, [old_qlog, recent], 30, 10)
  assert second["excluded_routes"] == ["old"]
  assert second["projected_bytes"] == 5


@pytest.mark.parametrize("marker", ["rlog.lock", "qlog.partial", "symlink"])
def test_retention_protects_entire_active_or_symlinked_route(tmp_path, marker):
  for number in (0, 1):
    segment = tmp_path / f"data/media/0/realdata/route--{number}"
    segment.mkdir(parents=True)
    (segment / "qlog").write_bytes(b"log")
  protected = segment / marker
  if marker == "symlink":
    protected.symlink_to(tmp_path / "outside")
  else:
    protected.touch()
  plan = plan_retention(tmp_path, {}, [], 30, 1, max_protected_bytes=1)
  assert plan["files"] == []
  assert plan["protected_routes"] == ([] if marker == "symlink" else ["route"])
  assert plan["unsafe_routes"] == (["route"] if marker == "symlink" else [])
  assert plan["over_budget_bytes"] == 5


@pytest.mark.parametrize("marker", ["rlog.lock", "qlog.partial", "rlog.zst.partial_123_456"])
def test_stale_protection_expires_and_prunes_partial_bytes(tmp_path, monkeypatch, marker):
  path = tmp_path / "data/media/0/realdata/old--0/qlog"
  path.parent.mkdir(parents=True)
  path.write_bytes(b"log")
  os.utime(path, (1, 1))
  partial = path.parent / marker
  partial.write_bytes(b"leftover")
  now = max(p.stat().st_ctime for p in (path, partial, path.parent)) + 24 * 3600 + 1
  monkeypatch.setattr("openpilot.tools.route_sync.refresh_routes.time.time", lambda: now)
  plan = plan_retention(tmp_path, {}, [], 30, 100)
  assert plan["protected_routes"] == []
  assert plan["expired_protection_routes"] == ["old"]
  assert plan["local_bytes_before"] == plan["prune_bytes"] == 11
  apply_retention(plan, tmp_path, {})
  assert not path.parent.exists()


def test_recent_activity_in_another_segment_protects_whole_route(tmp_path, monkeypatch):
  old = tmp_path / "data/media/0/realdata/route--0/qlog"
  old.parent.mkdir(parents=True)
  old.write_bytes(b"log")
  os.utime(old, (1, 1))
  (old.parent / "rlog.lock").touch()
  now = old.parent.stat().st_ctime + 2 * 86400
  monkeypatch.setattr("openpilot.tools.route_sync.refresh_routes.time.time", lambda: now)
  plan = plan_retention(tmp_path, {}, [], 30, 100)
  assert plan["excluded_routes"] == ["route"]
  new = old.parent.parent / "route--1/qlog.partial"
  new.parent.mkdir()
  new.write_bytes(b"new")
  os.utime(new, (now, now))
  with pytest.raises(RuntimeError, match="became active"):
    apply_retention(plan, tmp_path, {})
  assert old.exists()
  current = plan_retention(tmp_path, {}, [], 30, 100)
  assert current["protected_routes"] == ["route"]
  assert current["protected_bytes"] == 6


def test_copy_with_preserved_mtime_renews_protection(tmp_path, monkeypatch):
  log = tmp_path / "data/media/0/realdata/route--0/qlog"
  log.parent.mkdir(parents=True)
  log.write_bytes(b"log")
  marker = log.parent / "qlog.partial"
  marker.write_bytes(b"pending")
  for path in (log, marker, log.parent):
    os.utime(path, (1, 1))
  monkeypatch.setattr("openpilot.tools.route_sync.refresh_routes.time.time", lambda: log.stat().st_ctime + 1)
  plan = plan_retention(tmp_path, {}, [], 30, 100)
  assert plan["protected_routes"] == ["route"]
  assert plan["protected_bytes"] == plan["local_bytes_before"] == 10
  assert plan["files"] == []


def test_oversized_stale_partial_does_not_block_newest_completed_logs(tmp_path, monkeypatch):
  log = tmp_path / "data/media/0/realdata/new--0/qlog"
  log.parent.mkdir(parents=True)
  log.write_bytes(b"ok")
  partial = log.parent / "rlog.zst.partial_123_456"
  partial.write_bytes(b"oversized")
  now = max(p.stat().st_ctime for p in (log, partial, log.parent)) + 86401
  monkeypatch.setattr("openpilot.tools.route_sync.refresh_routes.time.time", lambda: now)
  plan = plan_retention(tmp_path, {}, [], 30, 2)
  assert plan["over_budget_bytes"] == 0 and plan["projected_bytes"] == 2
  assert plan["prune_bytes"] == 9
  apply_retention(plan, tmp_path, {})
  assert log.read_bytes() == b"ok" and not partial.exists()
  assert plan_retention(tmp_path, {}, [], 30, 2)["files"] == []


def test_symlink_does_not_pin_regular_sibling_logs(tmp_path):
  log = tmp_path / "data/media/0/realdata/old--0/qlog"
  log.parent.mkdir(parents=True)
  log.write_bytes(b"log")
  os.utime(log, (-40 * 86400, -40 * 86400))
  target = tmp_path / "outside"
  target.write_bytes(b"keep")
  link = log.parent / "rlog"
  link.symlink_to(target)
  plan = plan_retention(tmp_path, {}, [], 30, 100)
  assert plan["protected_routes"] == [] and plan["unsafe_routes"] == ["old"]
  apply_retention(plan, tmp_path, {})
  assert not log.exists() and link.is_symlink() and target.read_bytes() == b"keep"


def test_retention_refuses_symlinked_root_and_changed_file(tmp_path):
  root = tmp_path / "data/media/0/realdata"
  root.parent.mkdir(parents=True)
  outside = tmp_path / "outside"
  outside.mkdir()
  root.symlink_to(outside, target_is_directory=True)
  with pytest.raises(ValueError, match="symlink"):
    plan_retention(tmp_path, {}, [], 30, 1)
  root.unlink()
  path = root / "old--0/qlog"
  path.parent.mkdir(parents=True)
  path.write_bytes(b"log")
  os.utime(path, (-40 * 86400, -40 * 86400))
  plan = plan_retention(tmp_path, {}, [], 30, 1)
  path.write_bytes(b"changed")
  with pytest.raises(RuntimeError, match="changed"):
    apply_retention(plan, tmp_path, {})
  assert path.read_bytes() == b"changed"


def test_prune_only_dry_run_then_apply_without_device(tmp_path, monkeypatch):
  path = tmp_path / "data/media/0/realdata/route--0/qlog"
  path.parent.mkdir(parents=True)
  path.write_bytes(b"too big")
  os.utime(path, (-40 * 86400, -40 * 86400))
  state_file = tmp_path / "state.json"
  state_file.write_text('{"hosts": {"comma": {"files": {}}}}')
  before = state_file.read_bytes()
  report_file = tmp_path / "report.json"
  argv = ["refresh_routes.py", "--prune-only", "--download-root", str(tmp_path), "--state-file", str(state_file),
          "--report-file", str(report_file), "--max-cache-gib", str(1 / 2**30)]
  def no_ssh(*_args):
    raise AssertionError("prune-only must not use SSH")
  monkeypatch.setattr("openpilot.tools.route_sync.refresh_routes.list_remote_files", no_ssh)
  monkeypatch.setattr(sys, "argv", argv + ["--dry-run"])
  assert main() == 0
  assert path.exists() and state_file.read_bytes() == before
  report = json.loads(report_file.read_text())
  assert report["retention"]["prune_bytes"] == 7
  assert report["retention"]["pruned_files"] == 0
  monkeypatch.setattr(sys, "argv", argv)
  assert main() == 0
  assert not path.exists()
  assert json.loads(report_file.read_text())["retention"]["pruned_bytes"] == 7


@pytest.mark.parametrize("option,value", [("--keep-days", "0"), ("--max-cache-gib", "-1"), ("--max-cache-gib", "nan"),
                                        ("--max-protected-gib", "0"), ("--max-protected-gib", "inf")])
def test_invalid_retention_limits(monkeypatch, option, value):
  monkeypatch.setattr(sys, "argv", ["refresh_routes.py", option, value])
  with pytest.raises(SystemExit) as exc:
    main()
  assert exc.value.code == 2


@pytest.mark.parametrize("failure", ["list", "download", "state", "unlink", "none"])
def test_refresh_retention_filters_downloads_and_preserves_cache_on_failure(tmp_path, monkeypatch, failure):
  old = tmp_path / "data/media/0/realdata/old--0/rlog"
  old.parent.mkdir(parents=True)
  old.write_bytes(b"old data")
  os.utime(old, (80, 80))
  state_file = tmp_path / "state.json"
  if failure == "state":
    state_file.write_text("{broken")
  report_file = tmp_path / "report.json"
  remote_files = [RemoteFile(f"{CANONICAL_REMOTE_ROOT}/old--0/rlog", 8, 80, "old--0", "old"),
                  RemoteFile(f"{CANONICAL_REMOTE_ROOT}/new--0/qlog", 3, 90, "new--0", "new"),
                  RemoteFile(f"{CANONICAL_REMOTE_ROOT}/ancient--0/qlog", 3, -40 * 86400, "ancient--0", "ancient")]
  downloads = []
  def listing(*_args):
    if failure == "list":
      raise RuntimeError("offline")
    return remote_files
  def download(_host, remote_path, local_path, _timeout):
    if failure == "download":
      raise RuntimeError("transfer failed")
    downloads.append(remote_path)
    local_path.parent.mkdir(parents=True, exist_ok=True)
    local_path.write_bytes(b"log")
  monkeypatch.setattr("openpilot.tools.route_sync.refresh_routes.list_remote_files", listing)
  monkeypatch.setattr("openpilot.tools.route_sync.refresh_routes.download_file", download)
  if failure == "unlink":
    def fail_unlink(_path):
      raise OSError("cannot delete")
    monkeypatch.setattr(Path, "unlink", fail_unlink)
  monkeypatch.setattr(sys, "argv", ["refresh_routes.py", "--download-root", str(tmp_path), "--state-file", str(state_file),
                                   "--report-file", str(report_file), "--max-cache-gib", str(5 / 2**30), "--include-rlog"])
  rc = main()
  if failure != "none":
    assert rc != 0 and old.exists()
    if failure == "unlink":
      assert json.loads(state_file.read_text())["hosts"]["comma"]["retention_cutoff"] == [80, "old"]
  else:
    assert rc == 0 and not old.exists()
    assert downloads == [remote_files[1].remote_path]
    assert main() == 0
    assert downloads == [remote_files[1].remote_path]
    assert json.loads(report_file.read_text())["counts"]["skipped_due_to_retention"] == 2


@pytest.mark.parametrize("protected", [False, True])
def test_impossible_budget_does_not_evict_newest_route(tmp_path, protected):
  root = tmp_path / "data/media/0/realdata"
  for route, size, mtime in [("old", 1, 80), ("new", 6, 90)]:
    path = root / f"{route}--0/qlog"
    path.parent.mkdir(parents=True)
    path.write_bytes(b"x" * size)
    os.utime(path, (mtime, mtime))
  if protected:
    (path.parent / "qlog.lock").touch()
  plan = plan_retention(tmp_path, {}, [], 30, 5, max_protected_bytes=5)
  assert plan["files"] == [] and plan["cutoff"] == [0, ""]
  assert plan["over_budget_bytes"] == (1 if protected else 2)


@pytest.mark.parametrize("protected_size", [10, 29])
def test_protected_bytes_do_not_reduce_ordinary_budget(tmp_path, protected_size):
  root = tmp_path / "data/media/0/realdata"
  for route, size, mtime in [("protected", protected_size, 70), ("old", 29, 80), ("new", 1, 90)]:
    path = root / f"{route}--0/qlog"
    path.parent.mkdir(parents=True)
    path.write_bytes(b"x" * size)
    os.utime(path, (mtime, mtime))
  (root / "protected--0/qlog.lock").touch()
  plan = plan_retention(tmp_path, {}, [], 30, 30, max_protected_bytes=10)
  assert plan["files"] == []
  assert plan["ordinary_bytes"] == 30 and plan["ordinary_over_budget_bytes"] == 0
  assert plan["protected_bytes"] == protected_size
  assert plan["protected_over_budget_bytes"] == max(0, protected_size - 10)
  assert plan["projected_bytes"] == 30 + protected_size
  smaller = plan_retention(tmp_path, {}, [], 30, 29, max_protected_bytes=10)
  assert smaller["excluded_routes"] == ["old"]
  assert smaller["ordinary_bytes"] == 1 and smaller["protected_bytes"] == protected_size


@pytest.mark.parametrize("protected_size", [8, 29])
def test_protected_download_limit_does_not_block_ordinary_refresh(tmp_path, monkeypatch, protected_size):
  root = tmp_path / "data/media/0/realdata"
  for route, size in [("active", protected_size), ("ordinary", 27)]:
    path = root / f"{route}--0/qlog"
    path.parent.mkdir(parents=True)
    path.write_bytes(b"x" * size)
    os.utime(path, (80, 80))
  (root / "active--0/rlog.lock").touch()
  remotes = [RemoteFile(f"{CANONICAL_REMOTE_ROOT}/{route}--{segment}/qlog", size, 90, f"{route}--{segment}", route)
             for route, segment, size in [("active", 1, 2), ("active", 2, 1), ("ordinary", 1, 3)]]
  monkeypatch.setattr("openpilot.tools.route_sync.refresh_routes.list_remote_files", lambda *_args: remotes)
  downloads = []
  def download(_host, remote, local, _timeout):
    downloads.append(remote)
    local.parent.mkdir(parents=True, exist_ok=True)
    local.write_bytes(b"x" * next(item.size for item in remotes if item.remote_path == remote))
  monkeypatch.setattr("openpilot.tools.route_sync.refresh_routes.download_file", download)
  report_file = tmp_path / "report.json"
  monkeypatch.setattr(sys, "argv", ["refresh_routes.py", "--download-root", str(tmp_path), "--state-file", str(tmp_path / "state.json"),
                                   "--report-file", str(report_file), "--max-cache-gib", str(30 / 2**30),
                                   "--max-protected-gib", str(10 / 2**30), "--max-downloads", "2"])
  assert main() == (0 if protected_size == 8 else 1)
  report = json.loads(report_file.read_text())
  assert report["counts"]["skipped_due_to_protected_limit"] == (1 if protected_size == 8 else 2)
  assert report["retention"]["ordinary_bytes"] == 30
  assert report["retention"]["protected_bytes"] == (10 if protected_size == 8 else 29)
  assert remotes[-1].remote_path in downloads
  assert remotes[1].remote_path not in downloads
  assert (root / "ordinary--0/qlog").stat().st_size == 27


@pytest.mark.parametrize("replacement_size", [7, 8])
def test_overfull_protection_allows_replacements_without_growth(tmp_path, monkeypatch, replacement_size):
  path = tmp_path / "data/media/0/realdata/active--0/qlog"
  path.parent.mkdir(parents=True)
  path.write_bytes(b"x" * 8)
  (path.parent / "rlog.lock").touch()
  remote = RemoteFile(f"{CANONICAL_REMOTE_ROOT}/active--0/qlog", replacement_size, 90, "active--0", "active")
  monkeypatch.setattr("openpilot.tools.route_sync.refresh_routes.list_remote_files", lambda *_args: [remote])
  def download(_host, _remote, local, _timeout):
    local.write_bytes(b"y" * replacement_size)
  monkeypatch.setattr("openpilot.tools.route_sync.refresh_routes.download_file", download)
  report_file = tmp_path / "report.json"
  monkeypatch.setattr(sys, "argv", ["refresh_routes.py", "--download-root", str(tmp_path), "--state-file", str(tmp_path / "state.json"),
                                   "--report-file", str(report_file), "--max-protected-gib", str(5 / 2**30)])
  assert main() == 1  # Existing protected data still exceeds its budget.
  assert path.read_bytes() == b"y" * replacement_size
  report = json.loads(report_file.read_text())
  assert report["counts"]["downloaded"] == 1
  assert report["counts"]["skipped_due_to_protected_limit"] == 0


def test_capped_refresh_reserves_only_attempted_downloads(tmp_path, monkeypatch):
  path = tmp_path / "data/media/0/realdata/old--0/qlog"
  path.parent.mkdir(parents=True)
  path.write_bytes(b"older")
  os.utime(path, (80, 80))
  remotes = [RemoteFile(f"{CANONICAL_REMOTE_ROOT}/{route}--0/qlog", 3, mtime, f"{route}--0", route)
             for route, mtime in [("new1", 90), ("new2", 95)]]
  monkeypatch.setattr("openpilot.tools.route_sync.refresh_routes.list_remote_files", lambda *_args: remotes)
  def download(_host, _remote, local, _timeout):
    local.parent.mkdir(parents=True, exist_ok=True)
    local.write_bytes(b"log")
  monkeypatch.setattr("openpilot.tools.route_sync.refresh_routes.download_file", download)
  report_file = tmp_path / "report.json"
  monkeypatch.setattr(sys, "argv", ["refresh_routes.py", "--download-root", str(tmp_path), "--state-file", str(tmp_path / "state.json"),
                                   "--report-file", str(report_file), "--max-cache-gib", str(8 / 2**30), "--max-downloads", "1", "--newest-first"])
  assert main() == 0 and path.exists()
  report = json.loads(report_file.read_text())
  assert report["retention"]["pruned_files"] == 0
  assert report["counts"]["downloaded"] == 1
  assert report["counts"]["skipped_due_to_limit"] == 1


@pytest.mark.parametrize("metadata", ["-2\t5", "broken", "NaN\t1"])
def test_invalid_remote_metadata_aborts_listing(monkeypatch, metadata):
  monkeypatch.setattr(subprocess, "run", lambda *_args, **_kwargs:
                      subprocess.CompletedProcess([], 0, f"{CANONICAL_REMOTE_ROOT}/route--0/qlog\t{metadata}\n"))
  with pytest.raises(RuntimeError, match="Invalid remote log metadata"):
    list_remote_files("comma", [CANONICAL_REMOTE_ROOT], ["qlog"], 8)


def test_dry_run_does_not_migrate_legacy_cache(tmp_path, monkeypatch):
  old = tmp_path / "comma/data/media/0/realdata/route--0/qlog"
  old.parent.mkdir(parents=True)
  old.write_bytes(b"log")
  state_file = tmp_path / "state.json"
  remote = f"{CANONICAL_REMOTE_ROOT}/route--0/qlog"
  state_file.write_text(json.dumps({"hosts": {"comma": {"files": {remote: {"size": 3, "mtime": 90}}}}}))
  original = state_file.read_bytes()
  monkeypatch.setattr("openpilot.tools.route_sync.refresh_routes.list_remote_files",
                      lambda *_args: [RemoteFile(remote, 3, 90, "route--0", "route")])
  report_file = tmp_path / "report.json"
  monkeypatch.setattr(sys, "argv", ["refresh_routes.py", "--dry-run", "--download-root", str(tmp_path), "--state-file", str(state_file),
                                   "--report-file", str(report_file)])
  assert main() == 0
  assert old.exists() and state_file.read_bytes() == original
  assert json.loads(report_file.read_text())["counts"]["unchanged"] == 1


def test_main_migrates_legacy_root_cache_into_canonical_realdata(monkeypatch, tmp_path: Path) -> None:
  download_root = tmp_path / "downloads"
  state_file = tmp_path / "state.json"
  report_file = tmp_path / "report.json"
  remote_path = "/data/media/0/realdata_konik/route_legacy--0/qlog.zst"
  legacy_local_path = download_root / "commawifi" / remote_path.lstrip("/")
  legacy_local_path.parent.mkdir(parents=True, exist_ok=True)
  legacy_local_path.write_bytes(b"log")

  state_file.write_text(json.dumps({
    "version": 1,
    "hosts": {
      "commawifi": {
        "files": {
          remote_path: {
            "size": 123,
            "mtime": 45,
            "local_path": str(legacy_local_path),
            "first_synced_utc": "2026-03-01T00:00:00+00:00",
            "last_synced_utc": "2026-03-01T00:00:00+00:00",
          },
        },
      },
    },
  }))

  remote_files = [
    RemoteFile(remote_path=remote_path, size=123, mtime=45, segment="route_legacy--0", route="route_legacy"),
  ]
  downloads: list[tuple[str, str, Path]] = []

  def fake_list_remote_files(*_args, **_kwargs):
    return remote_files

  def fake_download_file(ssh_host: str, remote_path_arg: str, local_path: Path, connect_timeout: int) -> None:
    downloads.append((ssh_host, remote_path_arg, local_path))
    raise AssertionError("legacy-root file should be migrated, not downloaded again")

  monkeypatch.setattr("openpilot.tools.route_sync.refresh_routes.list_remote_files", fake_list_remote_files)
  monkeypatch.setattr("openpilot.tools.route_sync.refresh_routes.download_file", fake_download_file)
  monkeypatch.setattr(sys, "argv", [
    "refresh_routes.py",
    "--host",
    "commawifi",
    "--download-root",
    str(download_root),
    "--state-file",
    str(state_file),
    "--report-file",
    str(report_file),
    "--remote-root",
    "/data/media/0/realdata_konik",
  ])

  rc = main()

  canonical_local_path = download_root / "data/media/0/realdata/route_legacy--0/qlog.zst"
  assert rc == 0
  assert downloads == []
  assert canonical_local_path.exists()
  assert not legacy_local_path.exists()

  report = json.loads(report_file.read_text())
  assert report["counts"]["unchanged"] == 1
  assert report["counts"]["downloaded"] == 0


def test_file_vanished_during_listing_is_skipped(monkeypatch):
  out = f"{CANONICAL_REMOTE_ROOT}/gone--0/qlog\t-1\t-1\n{CANONICAL_REMOTE_ROOT}/kept--0/qlog\t3\t90\n"
  monkeypatch.setattr(subprocess, "run", lambda *_args, **_kwargs: subprocess.CompletedProcess([], 0, out))
  assert [f.route for f in list_remote_files("comma", [CANONICAL_REMOTE_ROOT], ["qlog"], 8)] == ["kept"]


def _cached_qlog(tmp_path, segment, mtime):
  path = tmp_path / "data/media/0/realdata" / segment / "qlog"
  path.parent.mkdir(parents=True)
  path.write_bytes(b"log")
  os.utime(path, (mtime, mtime))
  return f"{CANONICAL_REMOTE_ROOT}/{segment}/qlog"


def test_fresher_duplicate_metadata_wins(tmp_path, monkeypatch):
  monkeypatch.setattr("openpilot.tools.route_sync.refresh_routes.time.time", lambda: 40 * 86400)
  remote = _cached_qlog(tmp_path, "route--0", 40 * 86400)
  legacy = remote.replace(CANONICAL_REMOTE_ROOT, "/data/media/0/realdata_HD")
  state = {"files": {remote: {"mtime": 40 * 86400}, legacy: {"mtime": 1}}}
  assert plan_retention(tmp_path, state, [], 30, 100)["excluded_routes"] == []


def test_listing_supplies_source_mtime_without_a_download(tmp_path, monkeypatch):
  monkeypatch.setattr("openpilot.tools.route_sync.refresh_routes.time.time", lambda: 40 * 86400)
  remote = _cached_qlog(tmp_path, "route--0", 1)
  listed = [RemoteFile(remote, 3, 40 * 86400 - 60, "route--0", "route")]
  assert plan_retention(tmp_path, {}, [], 30, 100)["excluded_routes"] == ["route"]
  plan = plan_retention(tmp_path, {}, [], 30, 100, listed=listed)
  assert plan["excluded_routes"] == [] and plan["projected_bytes"] == 3


def test_eviction_cutoff_blocks_downloads_but_not_cached_routes(tmp_path, monkeypatch):
  now = 40 * 86400
  monkeypatch.setattr("openpilot.tools.route_sync.refresh_routes.time.time", lambda: now)
  _cached_qlog(tmp_path, "cached--0", now - 3 * 86400)
  evicted = RemoteFile(f"{CANONICAL_REMOTE_ROOT}/evicted--0/qlog", 3, now - 2 * 86400, "evicted--0", "evicted")
  state = {"files": {}, "retention_cutoff": [now, "zzz"]}
  plan = plan_retention(tmp_path, state, [evicted], 3650, 4000 * 2**30)
  assert plan["excluded_routes"] == ["evicted"]
  assert plan["files"] == []


def test_stale_legacy_listing_does_not_age_a_cached_log(tmp_path, monkeypatch):
  now = 40 * 86400
  monkeypatch.setattr("openpilot.tools.route_sync.refresh_routes.time.time", lambda: now)
  remote = _cached_qlog(tmp_path, "route--0", now - 60)
  legacy = remote.replace(CANONICAL_REMOTE_ROOT, "/data/media/0/realdata_HD")
  canon, hd = RemoteFile(remote, 3, now - 60, "route--0", "route"), RemoteFile(legacy, 3, 1, "route--0", "route")
  state = {"files": {remote: {"mtime": now - 60}}}
  assert plan_retention(tmp_path, state, [], 30, 100, listed=[canon, hd])["excluded_routes"] == []
  assert plan_retention(tmp_path, state, [canon, hd], 30, 100)["excluded_routes"] == []


def test_aged_cached_log_does_not_take_a_download_slot(tmp_path, monkeypatch):
  now = 40 * 86400
  monkeypatch.setattr("openpilot.tools.route_sync.refresh_routes.time.time", lambda: now)
  old = _cached_qlog(tmp_path, "old--0", now - 10)
  remotes = [RemoteFile(old, 3, 1, "old--0", "old"), RemoteFile(f"{CANONICAL_REMOTE_ROOT}/zz--0/qlog", 3, now - 60, "zz--0", "zz")]
  monkeypatch.setattr("openpilot.tools.route_sync.refresh_routes.list_remote_files", lambda *_args: remotes)
  report_file = tmp_path / "report.json"
  monkeypatch.setattr(sys, "argv", ["refresh_routes.py", "--dry-run", "--download-root", str(tmp_path), "--state-file", str(tmp_path / "state.json"),
                                   "--report-file", str(report_file), "--keep-days", "30", "--max-downloads", "1"])
  assert main() == 0
  counts = json.loads(report_file.read_text())["counts"]
  assert counts["download_candidates"] == 1 and counts["skipped_due_to_limit"] == 0


def test_stale_legacy_listing_does_not_override_state_mtime(tmp_path, monkeypatch):
  now = 40 * 86400
  monkeypatch.setattr("openpilot.tools.route_sync.refresh_routes.time.time", lambda: now)
  remote = _cached_qlog(tmp_path, "route--0", 1)
  hd = RemoteFile(remote.replace(CANONICAL_REMOTE_ROOT, "/data/media/0/realdata_HD"), 3, 1, "route--0", "route")
  state = {"files": {remote: {"mtime": now - 60}}}
  assert plan_retention(tmp_path, state, [], 30, 100, listed=[hd])["excluded_routes"] == []
