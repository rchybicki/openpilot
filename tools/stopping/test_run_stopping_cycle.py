from __future__ import annotations

import json
import os
from pathlib import Path
import sys

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
  sys.path.insert(0, str(REPO_ROOT))

from openpilot.tools.stopping.run_stopping_cycle import discover_recent_summaries, select_fit_summaries
from openpilot.tools.stopping.run_stopping_cycle import pick_newest_route_from_sync_report
from openpilot.tools.stopping.run_stopping_cycle import pick_moving_route_for_analysis


def _write_summary(path: Path, *, route: str | None = None, event_mode: str = "speed_transition", event_sources: list[str] | None = None) -> None:
  path.parent.mkdir(parents=True, exist_ok=True)
  payload = {
    "route": route if route is not None else path.parent.parent.name,
    "event_mode": event_mode,
    "events": [{"event_source": source} for source in (event_sources or ["speed"])],
  }
  path.write_text(json.dumps(payload))


def _write_qlog(path: Path, v_ego_samples: list[float]) -> None:
  from cereal import log as capnp_log

  payload = bytearray()
  mono_time = 0
  for v_ego in v_ego_samples:
    mono_time += 1
    msg = capnp_log.Event.new_message()
    msg.logMonoTime = mono_time
    msg.init("carState")
    msg.carState.vEgo = float(v_ego)
    payload.extend(msg.to_bytes())

  path.parent.mkdir(parents=True, exist_ok=True)
  path.write_bytes(payload)


def test_select_fit_summaries_includes_recent_when_explicit_missing(tmp_path: Path) -> None:
  analysis_root = tmp_path / "analysis"
  host = "commawifi"

  current = analysis_root / host / "cycle_current" / "summary.json"
  older_a = analysis_root / host / "cycle_a" / "summary.json"
  older_b = analysis_root / host / "cycle_b" / "summary.json"
  _write_summary(current, route="route_current")
  _write_summary(older_a, route="route_a")
  _write_summary(older_b, route="route_b")

  # Ensure deterministic recency order: current > older_a > older_b
  base_ts = 1_700_000_000
  older_b.touch()
  older_a.touch()
  current.touch()
  older_b_stat = base_ts
  older_a_stat = base_ts + 10
  current_stat = base_ts + 20
  os.utime(older_b, (older_b_stat, older_b_stat))
  os.utime(older_a, (older_a_stat, older_a_stat))
  os.utime(current, (current_stat, current_stat))

  selected = select_fit_summaries(
    explicit_summaries=[],
    analysis_summary_json=current,
    analysis_root=analysis_root,
    host=host,
    event_source="all",
    recent_limit=2,
  )

  assert selected == [current, older_a]


def test_select_fit_summaries_prefers_explicit_inputs(tmp_path: Path) -> None:
  analysis_root = tmp_path / "analysis"
  host = "commawifi"
  explicit = tmp_path / "explicit.json"
  current = analysis_root / host / "cycle_current" / "summary.json"
  _write_summary(explicit)
  _write_summary(current)

  selected = select_fit_summaries(
    explicit_summaries=[explicit],
    analysis_summary_json=current,
    analysis_root=analysis_root,
    host=host,
    event_source="all",
    recent_limit=8,
  )

  assert selected == [explicit]


def test_discover_recent_summaries_all_prefers_hybrid_per_route(tmp_path: Path) -> None:
  analysis_root = tmp_path / "analysis"
  host = "commawifi"

  route_a_signal = analysis_root / host / "route_a" / "ts_signal" / "summary.json"
  route_a_speed = analysis_root / host / "route_a" / "ts_speed" / "summary.json"
  route_a_hybrid = analysis_root / host / "route_a" / "ts_hybrid" / "summary.json"
  route_b_speed = analysis_root / host / "route_b" / "ts_speed" / "summary.json"

  _write_summary(route_a_signal, route="route_a", event_mode="engaged_signal", event_sources=["signal"])
  _write_summary(route_a_speed, route="route_a", event_mode="speed_transition", event_sources=["speed"])
  _write_summary(route_a_hybrid, route="route_a", event_mode="hybrid", event_sources=["signal", "speed"])
  _write_summary(route_b_speed, route="route_b", event_mode="speed_transition", event_sources=["speed"])

  base_ts = 1_700_000_000
  os.utime(route_a_hybrid, (base_ts + 5, base_ts + 5))
  os.utime(route_a_speed, (base_ts + 15, base_ts + 15))
  os.utime(route_a_signal, (base_ts + 25, base_ts + 25))
  os.utime(route_b_speed, (base_ts + 20, base_ts + 20))

  discovered = discover_recent_summaries(analysis_root=analysis_root, host=host, event_source="all", limit=10)
  assert discovered == [route_b_speed, route_a_hybrid]


def test_pick_newest_route_from_sync_report_prefers_new_routes_and_mtime() -> None:
  report = {
    "new_routes": ["route_old", "route_new"],
    "downloaded_files": [
      {"route": "route_old", "mtime": 100, "remote_path": "/data/media/0/realdata/route_old--0/qlog"},
      {"route": "route_new", "mtime": 250, "remote_path": "/data/media/0/realdata/route_new--0/qlog"},
      {"route": "route_new", "mtime": 200, "remote_path": "/data/media/0/realdata/route_new--1/qlog"},
    ],
  }
  assert pick_newest_route_from_sync_report(report) == "route_new"


def test_pick_newest_route_from_sync_report_falls_back_to_downloaded_routes() -> None:
  report = {
    "downloaded_files": [
      {"route": "route_a", "mtime": 10},
      {"route": "route_b", "mtime": 20},
    ],
  }
  assert pick_newest_route_from_sync_report(report) == "route_b"


def test_pick_moving_route_for_analysis_skips_standstill_new_route(tmp_path: Path) -> None:
  download_root = tmp_path / "downloads"
  host = "commawifi"

  standstill_route = "00000002--standstill"
  moving_route = "00000001--moving"

  standstill_qlog = download_root / host / "data/media/0/realdata" / f"{standstill_route}--0" / "qlog"
  moving_qlog = download_root / host / "data/media/0/realdata" / f"{moving_route}--0" / "qlog"

  _write_qlog(standstill_qlog, [0.0] * 20)
  _write_qlog(moving_qlog, [0.0, 0.2, 1.2, 0.8])

  report = {
    "new_routes": [standstill_route],
    "downloaded_files": [
      {"route": standstill_route, "mtime": 200, "remote_path": f"/data/media/0/realdata/{standstill_route}--0/qlog"},
      {"route": moving_route, "mtime": 150, "remote_path": f"/data/media/0/realdata/{moving_route}--0/qlog"},
    ],
  }

  selected = pick_moving_route_for_analysis(
    report,
    download_root=download_root,
    host=host,
    min_route_vmax_mps=0.5,
    require_stop_signal=False,
  )
  assert selected == moving_route
