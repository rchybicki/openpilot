from __future__ import annotations

import json
import os
from pathlib import Path
import sys

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
  sys.path.insert(0, str(REPO_ROOT))

from openpilot.tools.stopping.run_stopping_cycle import build_shadow_analysis_cmd, discover_recent_summaries, parse_args, resolve_gate_summaries, select_fit_summaries
from openpilot.tools.stopping.run_stopping_cycle import has_local_qlogs
from openpilot.tools.stopping.run_stopping_cycle import pick_newest_route_from_sync_report
from openpilot.tools.stopping.run_stopping_cycle import pick_moving_route_for_analysis
from openpilot.tools.stopping.run_stopping_cycle import discover_route_summary, summary_route_id


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


def _write_qlog_zst_placeholder(path: Path) -> None:
  path.parent.mkdir(parents=True, exist_ok=True)
  path.write_bytes(b"\x28\xb5\x2f\xfdfake")


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


def test_select_fit_summaries_refills_after_excluding_holdout_routes(tmp_path: Path) -> None:
  analysis_root = tmp_path / "analysis"
  host = "commawifi"

  current = analysis_root / host / "cycle_current" / "summary.json"
  holdout_a = analysis_root / host / "review_20260307T000000Z_holdout_a_hybrid" / "summary.json"
  holdout_b = analysis_root / host / "review_20260307T000000Z_holdout_b_hybrid" / "summary.json"
  train_a = analysis_root / host / "review_20260307T000000Z_train_a_hybrid" / "summary.json"
  train_b = analysis_root / host / "review_20260307T000000Z_train_b_hybrid" / "summary.json"
  train_c = analysis_root / host / "review_20260307T000000Z_train_c_hybrid" / "summary.json"

  _write_summary(current, route="current")
  _write_summary(holdout_a, route="holdout_a", event_mode="hybrid", event_sources=["signal", "speed"])
  _write_summary(holdout_b, route="holdout_b", event_mode="hybrid", event_sources=["signal", "speed"])
  _write_summary(train_a, route="train_a", event_mode="hybrid", event_sources=["signal", "speed"])
  _write_summary(train_b, route="train_b", event_mode="hybrid", event_sources=["signal", "speed"])
  _write_summary(train_c, route="train_c", event_mode="hybrid", event_sources=["signal", "speed"])

  base_ts = 1_700_000_000
  ordered = [
    (train_c, base_ts + 10),
    (train_b, base_ts + 20),
    (train_a, base_ts + 30),
    (holdout_b, base_ts + 40),
    (holdout_a, base_ts + 50),
    (current, base_ts + 60),
  ]
  for path, ts in ordered:
    os.utime(path, (ts, ts))

  selected = select_fit_summaries(
    explicit_summaries=[],
    analysis_summary_json=current,
    analysis_root=analysis_root,
    host=host,
    event_source="all",
    recent_limit=4,
    excluded_routes={"holdout_a", "holdout_b"},
  )

  assert [summary_route_id(path) for path in selected] == ["current", "train_a", "train_b", "train_c"]


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


def test_discover_recent_summaries_all_skips_zero_event_summaries(tmp_path: Path) -> None:
  analysis_root = tmp_path / "analysis"
  host = "commawifi"

  empty_summary = analysis_root / host / "route_empty" / "ts_empty" / "summary.json"
  valid_summary = analysis_root / host / "route_valid" / "ts_valid" / "summary.json"

  empty_summary.parent.mkdir(parents=True, exist_ok=True)
  empty_summary.write_text(json.dumps({"route": "route_empty", "event_mode": "hybrid", "events": []}))
  _write_summary(valid_summary, route="route_valid", event_mode="hybrid", event_sources=["signal", "speed"])

  base_ts = 1_700_000_000
  os.utime(empty_summary, (base_ts + 20, base_ts + 20))
  os.utime(valid_summary, (base_ts + 10, base_ts + 10))

  discovered = discover_recent_summaries(analysis_root=analysis_root, host=host, event_source="all", limit=10)
  assert discovered == [valid_summary]


def test_discover_route_summary_finds_review_layout_by_route_payload(tmp_path: Path) -> None:
  analysis_root = tmp_path / "analysis"
  host = "commawifi"
  route = "0000071c--fb4cca0034"

  review_summary = analysis_root / host / f"review_20260307T000000Z_{route}_hybrid" / "summary.json"
  direct_summary = analysis_root / host / route / "20260306T000000Z" / "summary.json"

  _write_summary(review_summary, route=route, event_mode="hybrid", event_sources=["signal", "speed"])
  _write_summary(direct_summary, route=route, event_mode="speed_transition", event_sources=["speed"])

  base_ts = 1_700_000_000
  os.utime(direct_summary, (base_ts, base_ts))
  os.utime(review_summary, (base_ts + 10, base_ts + 10))

  discovered = discover_route_summary(analysis_root=analysis_root, host=host, route=route, event_source="all")
  assert discovered == review_summary


def test_resolve_gate_summaries_includes_current_analysis_and_skips_missing_routes(tmp_path: Path) -> None:
  analysis_root = tmp_path / "analysis"
  host = "commawifi"
  current_route = "000016a4--cc01502282"
  holdout_route = "00001688--e54e746812"

  current_summary = analysis_root / host / "cycle_current" / "summary.json"
  holdout_summary = analysis_root / host / f"review_current_{holdout_route}_hybrid" / "summary.json"
  _write_summary(current_summary, route=current_route, event_mode="hybrid", event_sources=["signal", "speed"])
  _write_summary(holdout_summary, route=holdout_route, event_mode="hybrid", event_sources=["signal", "speed"])

  summaries, missing_routes = resolve_gate_summaries(
    explicit_summaries=[],
    gate_routes=[holdout_route, "00000000--missing"],
    analysis_summary_json=current_summary,
    analysis_root=analysis_root,
    host=host,
    event_source="all",
    include_analysis_summary=True,
  )

  assert summaries == [current_summary, holdout_summary]
  assert missing_routes == ["00000000--missing"]


def test_resolve_gate_summaries_can_return_only_missing_routes(tmp_path: Path) -> None:
  summaries, missing_routes = resolve_gate_summaries(
    explicit_summaries=[],
    gate_routes=["00000000--missing"],
    analysis_summary_json=tmp_path / "missing_summary.json",
    analysis_root=tmp_path / "analysis",
    host="commawifi",
    event_source="all",
    include_analysis_summary=False,
  )

  assert summaries == []
  assert missing_routes == ["00000000--missing"]


def test_summary_route_id_prefers_payload_route_for_review_layout(tmp_path: Path) -> None:
  route = "00000721--2b37d8d4a9"
  summary = tmp_path / "analysis" / "commawifi" / f"review_20260307T000000Z_{route}_hybrid" / "summary.json"
  _write_summary(summary, route=route, event_mode="hybrid", event_sources=["signal", "speed"])

  assert summary_route_id(summary) == route


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


def test_has_local_qlogs_accepts_qlog_zst(tmp_path: Path) -> None:
  host_download_dir = tmp_path / "downloads" / "commawifi"
  _write_qlog_zst_placeholder(host_download_dir / "00000001--route--0" / "qlog.zst")

  assert has_local_qlogs(host_download_dir) is True


def test_has_local_qlogs_accepts_hostless_root(tmp_path: Path) -> None:
  host_download_dir = tmp_path / "downloads"
  _write_qlog_zst_placeholder(host_download_dir / "data/media/0/realdata" / "00000007--route--13" / "qlog.zst")

  assert has_local_qlogs(host_download_dir) is True


def test_has_local_qlogs_skips_live_segment_with_lock(tmp_path: Path) -> None:
  host_download_dir = tmp_path / "downloads" / "commawifi"
  live_segment = host_download_dir / "data/media/0/realdata" / "00000007--route--13"
  _write_qlog_zst_placeholder(live_segment / "qlog.zst")
  (live_segment / "rlog.lock").write_text("")

  assert has_local_qlogs(host_download_dir) is False


def test_pick_moving_route_for_analysis_prefers_plain_qlog_over_qlog_zst(monkeypatch, tmp_path: Path) -> None:
  download_root = tmp_path / "downloads"
  host = "commawifi"
  route = "00000001--moving"

  plain_qlog = download_root / host / "data/media/0/realdata" / f"{route}--0" / "qlog"
  qlog_zst = download_root / host / "data/media/0/realdata" / f"{route}--0" / "qlog.zst"
  _write_qlog(plain_qlog, [0.0, 0.4, 0.8])
  _write_qlog_zst_placeholder(qlog_zst)

  def fail_if_zst(path, *args, **kwargs):
    assert path == plain_qlog
    return 0.8

  monkeypatch.setattr("openpilot.tools.stopping.run_stopping_cycle.scan_qlog_vmax_mps", fail_if_zst)

  report = {
    "downloaded_files": [
      {"route": route, "mtime": 10, "remote_path": f"/data/media/0/realdata/{route}--0/qlog.zst"},
    ],
  }

  selected = pick_moving_route_for_analysis(
    report,
    download_root=download_root,
    host=host,
    min_route_vmax_mps=0.5,
    require_stop_signal=False,
  )

  assert selected == route


def test_parse_args_supports_leapfrog_alignment_flags(monkeypatch) -> None:
  monkeypatch.setattr(sys, "argv", [
    "run_stopping_cycle.py",
    "--fit-model",
    "--run-model-gate",
    "--run-leapfrog-alignment",
    "--model-gate-controller-should-stop-source",
    "constant_true",
    "--model-gate-min-entry-speed",
    "0.30",
    "--model-gate-max-leapfrog-rate",
    "0.20",
    "--model-gate-max-leapfrog-count",
    "2",
    "--model-gate-max-pred-end-cmd-jerk",
    "2.8",
    "--model-gate-max-pred-end-accel-step",
    "0.07",
    "--alignment-event-id-tolerance",
    "2",
    "--alignment-min-overlap-recall",
    "0.40",
    "--alignment-max-count-delta",
    "1",
  ])
  args = parse_args()
  assert args.run_leapfrog_alignment is True
  assert args.model_gate_controller_should_stop_source == "constant_true"
  assert args.model_gate_min_entry_speed == 0.30
  assert args.model_gate_max_leapfrog_rate == 0.20
  assert args.model_gate_max_leapfrog_count == 2
  assert args.model_gate_max_pred_end_cmd_jerk == 2.8
  assert args.model_gate_max_pred_end_accel_step == 0.07
  assert args.alignment_event_id_tolerance == 2
  assert args.alignment_min_overlap_recall == 0.40
  assert args.alignment_max_count_delta == 1
  assert args.alignment_min_enabled_ratio is None


def test_parse_args_runs_shadow_analysis_by_default(monkeypatch) -> None:
  monkeypatch.setattr(sys, "argv", [
    "run_stopping_cycle.py",
    "--analyze",
  ])

  args = parse_args()

  assert args.analyze is True
  assert args.skip_shadow_analysis is False


def test_build_shadow_analysis_cmd_downloads_targeted_rlogs(monkeypatch, tmp_path: Path) -> None:
  monkeypatch.setattr(sys, "argv", [
    "run_stopping_cycle.py",
    "--host",
    "commawifi",
    "--connect-timeout",
    "4",
  ])
  args = parse_args()
  summary_json = tmp_path / "analysis" / "summary.json"
  download_root = tmp_path / "downloads"

  cmd = build_shadow_analysis_cmd(
    script_dir=Path("/repo/tools/stopping"),
    args=args,
    summary_json=summary_json,
    download_root=download_root,
  )

  assert cmd[1:] == [
    "/repo/tools/stopping/analyze_stopping_shadow.py",
    "--host",
    "commawifi",
    "--download-root",
    str(download_root),
    "--summary-json",
    str(summary_json),
    "--connect-timeout",
    "4",
    "--download-missing-rlogs",
  ]


def test_parse_args_has_tightened_measured_comfort_defaults(monkeypatch) -> None:
  monkeypatch.setattr(sys, "argv", [
    "run_stopping_cycle.py",
  ])
  args = parse_args()
  assert args.measured_gate_min_should_stop_ratio == 0.15
  assert args.measured_gate_require_brake_command_below == -0.20
  assert args.measured_gate_min_events == 2
  assert args.measured_gate_min_entry_speed == 0.50
  assert args.measured_gate_max_entry_stop_jerk == 0.35
  assert args.measured_gate_max_entry_stop_cmd_jerk == 0.50
  assert args.measured_gate_max_end_stop_jerk == 0.35
  assert args.measured_gate_max_end_stop_cmd_jerk == 1.0
  assert args.measured_gate_max_leapfrog_rate == 0.20
