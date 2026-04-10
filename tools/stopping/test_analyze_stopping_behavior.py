from __future__ import annotations

from pathlib import Path
import subprocess

import pytest

from openpilot.tools.stopping.analyze_stopping_behavior import (
  SegmentFile,
  Sample,
  compute_event,
  compute_transition_sharpness_metrics,
  iter_qlog_files,
  pick_route,
  read_events,
)


def _sample(
  t: float,
  *,
  v_ego: float,
  a_ego: float,
  long_state: str = "pid",
  long_state_cmd: str = "pid",
  should_stop: bool = False,
  accel_cmd: float | None = None,
  standstill: bool = False,
  lead_status: bool = False,
  lead_d_rel_m: float | None = None,
) -> Sample:
  return Sample(
    t=t,
    segment=0,
    v_ego=v_ego,
    v_wheel_avg=v_ego,
    a_ego=a_ego,
    standstill=standstill,
    brake_pressed=False,
    gas_pressed=False,
    enabled=True,
    long_state=long_state,
    long_state_cmd=long_state_cmd,
    should_stop=should_stop,
    a_target=None,
    accel_cmd=accel_cmd,
    distance_to_stop_target_m=None,
    lead_status=lead_status,
    lead_d_rel_m=lead_d_rel_m,
    forcing_stop=False,
    red_light=False,
  )


def test_compute_transition_sharpness_metrics_uses_anchor_window():
  samples = [
    _sample(0.0, v_ego=1.20, a_ego=-0.05, accel_cmd=-0.05),
    _sample(0.1, v_ego=1.05, a_ego=-0.06, accel_cmd=-0.06),
    _sample(0.2, v_ego=0.90, a_ego=-0.10, accel_cmd=-0.10, should_stop=True, long_state="stopping", long_state_cmd="stopping"),
    _sample(0.3, v_ego=0.70, a_ego=-0.18, accel_cmd=-0.20, should_stop=True, long_state="stopping", long_state_cmd="stopping"),
    _sample(0.4, v_ego=0.50, a_ego=-0.22, accel_cmd=-0.22, should_stop=True, long_state="stopping", long_state_cmd="stopping"),
    _sample(0.5, v_ego=0.30, a_ego=-0.25, accel_cmd=-0.24, should_stop=True, long_state="stopping", long_state_cmd="stopping", standstill=True),
  ]

  accel_jerk, accel_step, cmd_jerk, cmd_step = compute_transition_sharpness_metrics(
    samples=samples,
    start_idx=0,
    end_idx=5,
    anchor_time_s=0.2,
  )

  assert accel_jerk == pytest.approx(0.8, abs=1e-6)
  assert accel_step == pytest.approx(0.085, abs=1e-6)
  assert cmd_jerk == pytest.approx(1.0, abs=1e-6)
  assert cmd_step == pytest.approx(0.095, abs=1e-6)


def test_compute_event_falls_back_to_should_stop_for_entry_metrics():
  samples = [
    _sample(0.0, v_ego=0.80, a_ego=-0.04, accel_cmd=-0.04),
    _sample(0.1, v_ego=0.60, a_ego=-0.05, accel_cmd=-0.05, should_stop=True),
    _sample(0.2, v_ego=0.40, a_ego=-0.12, accel_cmd=-0.14, should_stop=True),
    _sample(0.3, v_ego=0.20, a_ego=-0.20, accel_cmd=-0.18, should_stop=True),
    _sample(0.4, v_ego=0.04, a_ego=-0.18, accel_cmd=-0.16, should_stop=True, standstill=True),
    _sample(0.5, v_ego=0.00, a_ego=-0.16, accel_cmd=-0.15, should_stop=True, standstill=True),
  ]

  event = compute_event(
    event_id=1,
    event_source="speed",
    samples=samples,
    start_idx=0,
    stop_idx=4,
    hold_idx=4,
    approach_speed=1.2,
    graph_file="plot.html",
  )

  assert event.should_stop_to_stopping_s is None
  assert event.entry_stop_jerk_mps3 == pytest.approx(0.8, abs=1e-6)
  assert event.entry_stop_cmd_jerk_mps3 == pytest.approx(0.9, abs=1e-6)
  assert event.entry_stop_accel_step_mps2 == pytest.approx(0.045, abs=1e-6)
  assert event.entry_stop_cmd_step_mps2 == pytest.approx(0.055, abs=1e-6)


def test_compute_event_tracks_lead_distance_at_stop_entry_and_hold():
  samples = [
    _sample(0.0, v_ego=1.20, a_ego=-0.03, accel_cmd=-0.03, lead_status=True, lead_d_rel_m=16.0),
    _sample(0.1, v_ego=0.95, a_ego=-0.05, accel_cmd=-0.04, should_stop=True, lead_status=True, lead_d_rel_m=15.5),
    _sample(0.2, v_ego=0.70, a_ego=-0.08, accel_cmd=-0.08, should_stop=True, long_state="stopping", long_state_cmd="stopping", lead_status=True, lead_d_rel_m=15.0),
    _sample(0.3, v_ego=0.35, a_ego=-0.12, accel_cmd=-0.11, should_stop=True, long_state="stopping", long_state_cmd="stopping", lead_status=True, lead_d_rel_m=14.7),
    _sample(0.4, v_ego=0.03, a_ego=-0.10, accel_cmd=-0.10, should_stop=True, long_state="stopping", long_state_cmd="stopping", standstill=True, lead_status=True, lead_d_rel_m=5.2),
    _sample(0.5, v_ego=0.00, a_ego=-0.08, accel_cmd=-0.09, should_stop=True, long_state="stopping", long_state_cmd="stopping", standstill=True, lead_status=True, lead_d_rel_m=5.0),
    _sample(0.6, v_ego=0.00, a_ego=-0.08, accel_cmd=-0.09, should_stop=True, long_state="stopping", long_state_cmd="stopping", standstill=True, lead_status=True, lead_d_rel_m=4.9),
  ]

  event = compute_event(
    event_id=2,
    event_source="speed",
    samples=samples,
    start_idx=0,
    stop_idx=4,
    hold_idx=5,
    approach_speed=1.2,
    graph_file="plot.html",
  )

  assert event.lead_distance_stop_entry_m == pytest.approx(14.85, abs=1e-6)
  assert event.lead_distance_hold_m == pytest.approx(5.0, abs=1e-6)


def test_iter_qlog_files_finds_qlog_zst(tmp_path: Path):
  qlog_zst = tmp_path / "downloads" / "commawifi" / "00000001--route--0" / "qlog.zst"
  qlog_zst.parent.mkdir(parents=True, exist_ok=True)
  qlog_zst.write_bytes(b"\x28\xb5\x2f\xfdfake")

  discovered = iter_qlog_files(tmp_path / "downloads", "commawifi")

  assert len(discovered) == 1
  assert discovered[0].route == "00000001--route"
  assert discovered[0].segment == 0
  assert discovered[0].path == qlog_zst


def test_iter_qlog_files_finds_qlog_zst_in_hostless_root(tmp_path: Path):
  qlog_zst = tmp_path / "downloads" / "data/media/0/realdata" / "00000007--route--13" / "qlog.zst"
  qlog_zst.parent.mkdir(parents=True, exist_ok=True)
  qlog_zst.write_bytes(b"\x28\xb5\x2f\xfdfake")

  discovered = iter_qlog_files(tmp_path / "downloads", "commawifi")

  assert len(discovered) == 1
  assert discovered[0].route == "00000007--route"
  assert discovered[0].segment == 13
  assert discovered[0].path == qlog_zst


def test_iter_qlog_files_prefers_plain_qlog_over_zst_for_same_segment(tmp_path: Path):
  segment_dir = tmp_path / "downloads" / "commawifi" / "00000001--route--0"
  plain_qlog = segment_dir / "qlog"
  qlog_zst = segment_dir / "qlog.zst"
  plain_qlog.parent.mkdir(parents=True, exist_ok=True)
  plain_qlog.write_bytes(b"plain")
  qlog_zst.write_bytes(b"\x28\xb5\x2f\xfdfake")

  discovered = iter_qlog_files(tmp_path / "downloads", "commawifi")

  assert len(discovered) == 1
  assert discovered[0].path == plain_qlog


def test_iter_qlog_files_skips_live_segments_with_lock_files(tmp_path: Path):
  stable_qlog = tmp_path / "downloads" / "commawifi" / "00000001--stable--0" / "qlog.zst"
  live_qlog = tmp_path / "downloads" / "commawifi" / "00000007--live--13" / "qlog.zst"
  stable_qlog.parent.mkdir(parents=True, exist_ok=True)
  live_qlog.parent.mkdir(parents=True, exist_ok=True)
  stable_qlog.write_bytes(b"\x28\xb5\x2f\xfdfake")
  live_qlog.write_bytes(b"\x28\xb5\x2f\xfdfake")
  (live_qlog.parent / "rlog.lock").write_text("")

  discovered = iter_qlog_files(tmp_path / "downloads", "commawifi")

  assert len(discovered) == 1
  assert discovered[0].route == "00000001--stable"
  assert discovered[0].path == stable_qlog


def test_pick_route_prefers_newer_mtime_over_hex_like_prefix():
  older_high_prefix = SegmentFile(route="000009cc--older", segment=0, path=Path("/tmp/older"), mtime=100.0)
  newer_low_prefix = SegmentFile(route="00000007--newer", segment=0, path=Path("/tmp/newer"), mtime=200.0)

  selected = pick_route([older_high_prefix, newer_low_prefix], route_override=None)

  assert selected == "00000007--newer"


def test_read_events_decompresses_qlog_zst_via_zstd(monkeypatch, tmp_path: Path):
  from cereal import log as capnp_log

  qlog_zst = tmp_path / "00000001--route--0" / "qlog.zst"
  qlog_zst.parent.mkdir(parents=True, exist_ok=True)
  qlog_zst.write_bytes(b"\x28\xb5\x2f\xfdfake")

  msg = capnp_log.Event.new_message()
  msg.logMonoTime = 1
  msg.init("carState")
  msg.carState.vEgo = 1.23
  payload = msg.to_bytes()

  monkeypatch.setattr("openpilot.tools.stopping.analyze_stopping_behavior.shutil.which", lambda _: "/opt/homebrew/bin/zstd")

  def fake_run(cmd, capture_output, check):
    assert cmd[:3] == ["/opt/homebrew/bin/zstd", "-d", "-q"]
    assert cmd[-1] == str(qlog_zst)
    return subprocess.CompletedProcess(cmd, 0, stdout=payload, stderr=b"")

  monkeypatch.setattr("openpilot.tools.stopping.analyze_stopping_behavior.subprocess.run", fake_run)

  events = list(read_events(qlog_zst))

  assert len(events) == 1
  assert events[0].which() == "carState"
  assert events[0].carState.vEgo == pytest.approx(1.23)
