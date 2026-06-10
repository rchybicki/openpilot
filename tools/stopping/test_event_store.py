"""build_event_store tests (spec 7.1 / section 8): stable keys, schema/signals/telemetry versions,
dual-rate metric blocks, and the WP3 fit_plant_model event-store contract. Synthetic Sample
streams only -- no capnp/log decoding at any point."""

from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
  sys.path.insert(0, str(REPO_ROOT))

from openpilot.tools.stopping import build_event_store as bes
from openpilot.tools.stopping.analyze_stopping_behavior import Sample


def make_sample(t: float, segment: int, v: float, a: float, *, enabled=True, should_stop=False,
                accel_cmd=None, lead_status=False, lead_d_rel=None, lead_v=0.0, brake=False,
                isd=0.0) -> Sample:
  return Sample(
    t=t, segment=segment, v_ego=v, v_wheel_avg=v, a_ego=a,
    standstill=v < 0.05, brake_pressed=brake, gas_pressed=False,
    enabled=enabled, long_state="stopping" if (should_stop and v < 1.0) else "pid",
    long_state_cmd="off", should_stop=should_stop, a_target=accel_cmd,
    distance_to_stop_target_m=None, accel_cmd=accel_cmd,
    lead_status=lead_status, lead_d_rel_m=lead_d_rel, force_coast=False,
    forcing_stop=False, red_light=False, mono_time_s=1000.0 + t,
    lead_v=lead_v, accel_cmd_output=None, increased_stopped_distance_m=isd,
  )


def synthetic_stop_stream(dt: float = 0.01, segment: int = 7) -> list[Sample]:
  """3 m/s cruise -> 0.5 m/s^2 braking stop -> 6 s standstill hold; one clean stop event."""
  samples: list[Sample] = []
  t = 0.0
  v = 3.0
  while t < 2.0:                      # cruise
    samples.append(make_sample(t, segment, v, 0.0, accel_cmd=0.0))
    t += dt
  while v > 0.0:                      # braking
    a = -0.5
    samples.append(make_sample(t, segment, v, a, should_stop=v < 2.5, accel_cmd=-0.5))
    v = max(v + a * dt, 0.0)
    t += dt
  hold_end = t + 6.0
  while t < hold_end:                 # standstill hold
    samples.append(make_sample(t, segment, 0.0, 0.0, should_stop=True, accel_cmd=-0.2))
    t += dt
  return samples


class TestIngest:
  def test_one_event_detected_with_stable_key(self):
    samples = synthetic_stop_stream()
    records = bes.ingest_route_samples("routeA--abc", samples, rate_class="rlog100",
                                       signals_version=1, telemetry_version=1)
    assert len(records) == 1
    record = records[0]
    assert record["schema_version"] == bes.SCHEMA_VERSION
    assert record["detector"] == "hybrid"
    assert record["signals_version"] == 1
    assert record["telemetry_version"] == 1
    assert record["accel_cmd_source"] == "carControl"
    key = record["key"]
    assert key["route"] == "routeA--abc"
    assert key["seg"] == 7
    assert isinstance(key["hold_mono_ns"], int)
    assert key["hold_mono_ns"] > 10**11  # mono-time-based, not positional
    assert record["trace_ref"] == f"events/routeA--abc__7__{key['hold_mono_ns']}.npz"

  def test_stable_key_is_reproducible(self):
    samples = synthetic_stop_stream()
    r1 = bes.ingest_route_samples("r", samples)
    r2 = bes.ingest_route_samples("r", samples)
    assert r1[0]["key"] == r2[0]["key"]

  def test_dual_rate_metric_blocks(self):
    records = bes.ingest_route_samples("r", synthetic_stop_stream())
    record = records[0]
    expected_keys = {"end_stop_jerk", "end_stop_accel_step", "min_a_ego", "max_cmd_jerk",
                     "rollout_from_2mps_m", "final_lead_gap_m", "rebound_mps", "unexpected_accel",
                     "hard_decel_duration_s", "time_to_standstill_s"}
    for block in ("metrics_100hz", "metrics_10hz_compat"):
      assert set(record[block]) == expected_keys, block
    # same definitions, different rates: both report the same braking floor
    assert record["metrics_100hz"]["min_a_ego"] == record["metrics_10hz_compat"]["min_a_ego"]
    assert record["metrics_100hz"]["time_to_standstill_s"] > 0.0

  def test_entry_block(self):
    records = bes.ingest_route_samples("r", synthetic_stop_stream())
    entry = records[0]["entry"]
    assert entry["v_approach"] >= 2.5
    assert entry["explicit_target"] is False
    assert entry["isd_m"] == 0.0
    assert "lead_entry_gap_m" in entry

  def test_decimation_grid(self):
    samples = synthetic_stop_stream(dt=0.01)
    compat = bes.decimate_samples(samples, 0.10)
    deltas = np.diff([s.t for s in compat])
    assert abs(float(np.median(deltas)) - 0.10) < 0.011
    assert len(compat) < len(samples) / 5


class TestStoreWrite:
  def test_write_merge_and_npz(self, tmp_path: Path):
    store = tmp_path / "event_store"
    records = bes.ingest_route_samples("r1", synthetic_stop_stream())
    stats = bes.write_store(store, records)
    assert stats == {"total": 1, "added": 1, "replaced": 0}
    lines = (store / "events.jsonl").read_text().splitlines()
    assert len(lines) == 1
    record = json.loads(lines[0])
    assert "_trace" not in record
    trace_path = store / record["trace_ref"]
    assert trace_path.is_file()
    with np.load(trace_path) as npz:
      # WP3/fit_plant_model trace contract + sim_replay extras
      for name in ("t", "v_ego", "a_ego", "accel_cmd", "enabled", "brake_pressed", "should_stop",
                   "lead_status", "lead_v", "lead_d_rel_m", "distance_to_stop_target_m"):
        assert name in npz, name
      t = np.asarray(npz["t"])
      assert np.all(np.diff(t) > 0)

    # re-ingesting the same stream merges by stable key, not append
    stats2 = bes.write_store(store, bes.ingest_route_samples("r1", synthetic_stop_stream()))
    assert stats2 == {"total": 1, "added": 0, "replaced": 1}

  def test_fit_plant_model_can_load_the_store(self, tmp_path: Path):
    # the executable WP3 contract: tools/stopping/fit_plant_model.load_event_store reads our store
    from openpilot.tools.stopping import fit_plant_model as fpm
    store = tmp_path / "event_store"
    bes.write_store(store, bes.ingest_route_samples("r1", synthetic_stop_stream(),
                                                    signals_version=2, telemetry_version=2,
                                                    accel_cmd_source="carOutput"))
    traces = fpm.load_event_store(store)
    assert len(traces) == 1
    trace = traces[0]
    assert trace.route == "r1"
    assert trace.telemetry_version == 2
    assert trace.signals_version == 2
    assert trace.accel_cmd_source == "carOutput"
    assert trace.trusted_post_engagement() is True
    mask = fpm.usable_frame_mask(trace)
    assert int(np.count_nonzero(mask)) > 50
    resampled = fpm.resample_trace(trace, 0.10)
    assert len(resampled.t) > 10

  def test_v1_records_default_to_conservative_source(self, tmp_path: Path):
    from openpilot.tools.stopping import fit_plant_model as fpm
    store = tmp_path / "event_store"
    bes.write_store(store, bes.ingest_route_samples("r1", synthetic_stop_stream()))
    trace = fpm.load_event_store(store)[0]
    assert trace.accel_cmd_source == "carControl"
    assert trace.trusted_post_engagement() is False  # 4 s post-engagement exclusion applies


class TestRouteDiscovery:
  def test_routes_from_summaries(self, tmp_path: Path):
    root = tmp_path / "analysis"
    (root / "comma" / "r-new").mkdir(parents=True)
    (root / "comma" / "r-old").mkdir(parents=True)
    (root / "comma" / "r-old" / "summary.json").write_text(json.dumps({"route": "00000001--aaaa", "events": []}))
    (root / "comma" / "r-new" / "summary.json").write_text(json.dumps({"route": "00000002--bbbb", "events": []}))
    import os
    os.utime(root / "comma" / "r-old" / "summary.json", (1000, 1000))
    routes = bes.routes_from_summaries(root)
    assert routes[0] == "00000002--bbbb"
    assert set(routes) == {"00000001--aaaa", "00000002--bbbb"}
    assert bes.routes_from_summaries(root, limit=1) == ["00000002--bbbb"]
