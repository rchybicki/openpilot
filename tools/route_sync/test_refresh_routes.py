from __future__ import annotations

import json
from pathlib import Path
import subprocess
import sys

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
  sys.path.insert(0, str(REPO_ROOT))

from openpilot.tools.route_sync.common import CANONICAL_REMOTE_ROOT
from openpilot.tools.route_sync.refresh_routes import RemoteFile, build_remote_list_script, interleave_by_route, main


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
