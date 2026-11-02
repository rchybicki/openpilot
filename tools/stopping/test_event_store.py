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
from openpilot.tools.stopping import scoring_config as sc
from openpilot.tools.stopping.analyze_stopping_behavior import Sample


def make_sample(t: float, segment: int, v: float, a: float, *, enabled=True, should_stop=False,
                accel_cmd=None, lead_status=False, lead_d_rel=None, lead_v=0.0, brake=False,
                isd=0.0, long_state=None, a_long_imu=None, a_long_imu_raw=None) -> Sample:
  if long_state is None:
    long_state = "stopping" if (should_stop and v < 1.0) else "pid"
  return Sample(
    t=t, segment=segment, v_ego=v, v_wheel_avg=v, a_ego=a,
    standstill=v < 0.05, brake_pressed=brake, gas_pressed=False,
    enabled=enabled, long_state=long_state,
    long_state_cmd="off", should_stop=should_stop, a_target=accel_cmd,
    distance_to_stop_target_m=None, accel_cmd=accel_cmd,
    lead_status=lead_status, lead_d_rel_m=lead_d_rel, force_coast=False,
    forcing_stop=False, red_light=False, mono_time_s=1000.0 + t,
    lead_v=lead_v, accel_cmd_output=None, increased_stopped_distance_m=isd,
    a_long_imu=a_long_imu, a_long_imu_raw=a_long_imu_raw,
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
                     "hard_decel_duration_s", "time_to_standstill_s", "hold_acq_peak_cmd_jerk",
                     # cranked-requirement metrics (version 2, 2026-06-13)
                     "approach_peak_decel_over_gap2m", "approach_required_decel_to_2m",
                     "approach_necessary", "approach_worst_gap_m", "approach_worst_v_ego_mps",
                     "approach_worst_closing_mps", "approach_worst_meas_decel",
                     "settle_peak_meas_jerk", "settle_peak_sent_jerk", "settle_meas_minus_sent_jerk",
                     # IMU terminal-grab channels (eval.md §2.1): deprecated held-100Hz artifact jerk,
                     # honest 20Hz lower-bound jerk, trustworthy decel, and the raw filtered 100Hz channel
                     "settle_peak_imu_jerk", "settle_peak_imu_jerk_20hz", "settle_peak_imu_decel",
                     "settle_peak_imu_jerk_raw", "settle_peak_imu_decel_raw"}
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


class TestHoldAcquisitionDiagnostic:
  """NON-gating hold-acquisition peak command jerk (scoring_config.DiagnosticMetrics; driveway
  route 00001702--dcdc5c3eea--0): window = [enabled rising edge with v_ego < 0.3, +2 s]."""

  def test_standard_stream_reports_none(self):
    # synthetic_stop_stream engages at speed (enabled throughout): no low-speed engagement edge
    records = bes.ingest_route_samples("r", synthetic_stop_stream())
    for block in ("metrics_100hz", "metrics_10hz_compat"):
      assert records[0][block]["hold_acq_peak_cmd_jerk"] is None, block

  @staticmethod
  def _engage_at_standstill_samples(dt: float = 0.01) -> list:
    samples = []
    t = 0.0
    for _ in range(50):  # disengaged standstill creep
      samples.append(make_sample(t, 1, 0.04, 0.0, enabled=False, should_stop=False, accel_cmd=0.0))
      t += dt
    cmd = -0.10  # engage edge at v < 0.3, then a 3 m/s^3 ramp to the -1.05 hold
    for _ in range(250):
      samples.append(make_sample(t, 1, 0.03, -0.05, enabled=True, should_stop=True, accel_cmd=cmd))
      cmd = max(cmd - 0.03 * (dt / 0.01), -1.05)
      t += dt
    return samples

  def test_peak_cmd_jerk_measured_after_low_speed_engage_edge(self):
    samples = self._engage_at_standstill_samples()
    peak = bes.hold_acquisition_peak_cmd_jerk(samples, start_idx=50, hold_idx=len(samples) - 1)
    assert peak is not None
    assert abs(peak - 3.0) < 1e-6

  def test_steps_outside_the_2s_window_do_not_count(self):
    dt = 0.01
    samples = self._engage_at_standstill_samples(dt)
    # a disengage-release slam well past edge_t + 2 s must not inflate the metric
    t = samples[-1].t + dt
    samples.append(make_sample(t, 1, 0.0, 0.0, enabled=True, should_stop=True, accel_cmd=0.0))
    peak = bes.hold_acquisition_peak_cmd_jerk(samples, start_idx=50, hold_idx=len(samples) - 1)
    assert abs(peak - 3.0) < 1e-6

  def test_brake_takeover_inside_window_does_not_count_the_zeroing_step(self):
    # 2026-06-12 artifact case: driver takeover at edge +1.5 s (inside the 2 s window) zeroes the
    # command (-1.05 -> 0.0 in one 10 ms frame = 105 m/s^3); the active-only mask must truncate
    # the window at the first inactive frame so the metric keeps reporting the 3 m/s^3 ramp.
    dt = 0.01
    samples = self._engage_at_standstill_samples(dt)  # edge at idx 50 (t = 0.5 s)
    edge_t = samples[50].t
    takeover = [s for s in samples if s.t < edge_t + 1.5]
    t = takeover[-1].t + dt
    for _ in range(100):  # brake takeover: disengaged, long control off, command zeroed
      takeover.append(make_sample(t, 1, 0.0, 0.0, enabled=False, should_stop=False, accel_cmd=0.0,
                                  long_state="off", brake=True))
      t += dt
    peak = bes.hold_acquisition_peak_cmd_jerk(takeover, start_idx=50, hold_idx=len(takeover) - 1)
    assert peak is not None
    assert abs(peak - 3.0) < 1e-6  # NOT the ~105 m/s^3 zeroing step

  def test_gas_override_takeover_keeps_enabled_but_must_still_truncate(self):
    # The 00001707--18e50c5f8a shape: a gas press puts longitudinal in override -- `enabled`
    # STAYS true, longControlState goes 'off' one frame before the command zeroes. An
    # enabled-only mask misses this; the long_state mask must truncate before the zeroing step.
    dt = 0.01
    samples = self._engage_at_standstill_samples(dt)  # edge at idx 50 (t = 0.5 s)
    edge_t = samples[50].t
    takeover = [s for s in samples if s.t < edge_t + 1.5]
    t = takeover[-1].t + dt
    last_cmd = takeover[-1].accel_cmd
    # one frame: still enabled, long_state already 'off', command not yet zeroed (real log shape)
    takeover.append(make_sample(t, 1, 0.03, 0.0, enabled=True, should_stop=False,
                                accel_cmd=last_cmd, long_state="off"))
    t += dt
    for _ in range(100):  # override continues: enabled, long control off, command zeroed
      takeover.append(make_sample(t, 1, 0.03, 0.0, enabled=True, should_stop=False, accel_cmd=0.0,
                                  long_state="off"))
      t += dt
    peak = bes.hold_acquisition_peak_cmd_jerk(takeover, start_idx=50, hold_idx=len(takeover) - 1)
    assert peak is not None
    assert abs(peak - 3.0) < 1e-6  # NOT the ~105 m/s^3 zeroing step

  def test_edge_alignment_skew_does_not_empty_the_window(self):
    # The 00001713--979ec54e96 seg-11 shape: at the engage edge, `enabled` flips one frame
    # BEFORE longControlState does (edge frame ls='off', cmd still 0). Leading inactive frames
    # must be skipped, not truncated on, or the metric falsely reports None.
    dt = 0.01
    samples = self._engage_at_standstill_samples(dt)  # edge at idx 50
    skewed = list(samples)
    skewed[50] = make_sample(samples[50].t, 1, samples[50].v_ego, 0.0, enabled=True,
                             should_stop=False, accel_cmd=0.0, long_state="off")
    peak = bes.hold_acquisition_peak_cmd_jerk(skewed, start_idx=50, hold_idx=len(skewed) - 1)
    assert peak is not None
    assert abs(peak - 3.0) < 1e-6

  def test_engage_edge_at_speed_reports_none(self):
    dt = 0.01
    samples = []
    t = 0.0
    for _ in range(50):
      samples.append(make_sample(t, 1, 1.5, 0.0, enabled=False, should_stop=False, accel_cmd=0.0))
      t += dt
    for _ in range(100):  # engagement edge at 1.5 m/s: not a hold acquisition
      samples.append(make_sample(t, 1, 1.5, -0.3, enabled=True, should_stop=True, accel_cmd=-0.5))
      t += dt
    assert bes.hold_acquisition_peak_cmd_jerk(samples, start_idx=50, hold_idx=len(samples) - 1) is None


class TestCrankedApproachMetric:
  """Cranked-requirement P1 (2026-06-13): approach_decel_over_gap2m -- peak commanded decel while
  the lead gap is still > 2 m, with a kinematic-necessity exemption (decel required to bleed the
  closing speed to 0 before closing past the 2 m boundary)."""

  @staticmethod
  def _approach_stream(decel: float, gap: float, v0: float, lead_v: float, dt: float = 0.01) -> list:
    """Engaged stop with a constant lead gap and lead speed; brake at `decel` from v0 to a stop."""
    samples: list = []
    t = 0.0
    v = v0
    for _ in range(100):  # brief cruise
      samples.append(make_sample(t, 1, v, 0.0, accel_cmd=0.0, lead_status=True, lead_d_rel=gap, lead_v=lead_v))
      t += dt
    while v > 0.0:
      a = -decel
      samples.append(make_sample(t, 1, v, a, should_stop=v < max(v0 - 0.5, 0.5), accel_cmd=-decel,
                                 lead_status=True, lead_d_rel=gap, lead_v=lead_v))
      v = max(v + a * dt, 0.0)
      t += dt
    for _ in range(600):  # standstill hold
      samples.append(make_sample(t, 1, 0.0, 0.0, should_stop=True, accel_cmd=-0.2,
                                 lead_status=True, lead_d_rel=gap, lead_v=0.0))
      t += dt
    return samples

  def test_gentle_approach_under_cap_does_not_flag(self):
    # (a) gentle <= 0.5 m/s^2 approach: a far lead, slow closing -> peak <= cap, no violation
    samples = self._approach_stream(decel=0.4, gap=10.0, v0=3.0, lead_v=2.9)
    metric = bes.approach_decel_over_gap2m(samples, 0, len(samples) - 1)
    assert metric is not None
    assert metric["peak_decel"] <= 0.5 + 1e-9
    # cap not exceeded -> classify_event must not raise the flag
    harsh, _ = sc.classify_event({"approach_peak_decel_over_gap2m": metric["peak_decel"],
                                  "approach_required_decel_to_2m": metric["required_decel"]})
    assert "unnecessary_harsh_approach" not in harsh

  def test_unnecessary_harsh_approach_flags(self):
    # (b) UNNECESSARY > 0.5: big gap (4.5 m), slow closing (~0.1 m/s) yet the controller brakes 0.8
    samples = self._approach_stream(decel=0.8, gap=4.5, v0=1.0, lead_v=0.9)
    metric = bes.approach_decel_over_gap2m(samples, 0, len(samples) - 1)
    assert metric is not None
    assert metric["peak_decel"] > 0.5
    assert metric["required_decel"] <= 0.5
    assert metric["necessary"] is False
    harsh, _ = sc.classify_event({"approach_peak_decel_over_gap2m": metric["peak_decel"],
                                  "approach_required_decel_to_2m": metric["required_decel"]})
    assert "unnecessary_harsh_approach" in harsh

  def test_necessary_harsh_approach_is_exempt(self):
    # (c) NECESSARY > 0.5: close (2.5 m) fast (3 m/s) lead, high closing -> required >> cap, exempt
    samples = self._approach_stream(decel=1.7, gap=2.5, v0=3.0, lead_v=0.0)
    metric = bes.approach_decel_over_gap2m(samples, 0, len(samples) - 1)
    assert metric is not None
    assert metric["peak_decel"] > 0.5
    assert metric["required_decel"] > 0.5
    assert metric["necessary"] is True
    harsh, _ = sc.classify_event({"approach_peak_decel_over_gap2m": metric["peak_decel"],
                                  "approach_required_decel_to_2m": metric["required_decel"]})
    assert "unnecessary_harsh_approach" not in harsh

  def test_human_braked_unengaged_stop_reports_none(self):
    # ENGAGED MASK is mandatory: a 0%-engaged (human-braked) harsh stop must not produce a metric
    samples = self._approach_stream(decel=1.5, gap=4.5, v0=3.0, lead_v=0.0)
    for s in samples:
      s.enabled = False
      s.long_state = "off"
    assert bes.approach_decel_over_gap2m(samples, 0, len(samples) - 1) is None

  def test_close_lead_inside_gap_floor_is_ignored(self):
    # samples where the gap is already <= 2 m do not contribute (the requirement is gap > 2 m)
    samples = self._approach_stream(decel=1.2, gap=1.5, v0=2.0, lead_v=0.0)
    assert bes.approach_decel_over_gap2m(samples, 0, len(samples) - 1) is None


class TestCrankedTerminalGrab:
  """Cranked-requirement P2: settle_meas_jerk -- peak MEASURED jerk (WHEEL a_ego) at the first genuine
  standstill. P2 GATING is on the faithful IMU channels (settle_peak_imu_decel PRIMARY +
  settle_peak_imu_jerk_raw SECONDARY, re-wired v4 2026-06-14; see TestSettleImuJerk + test_scoring_config),
  NOT on the wheel settle_peak_meas_jerk. The WHEEL metric is still computed and recorded by
  build_event_store but stays a NON-gating diagnostic (a_ego is wheel-derived and blind to the v~=0
  grab, eval.md section 2.1), so a hard WHEEL grab must NOT set the harsh verdict."""

  @staticmethod
  def _settle_stream(grab: bool, dt: float = 0.01) -> list:
    samples: list = []
    t = 0.0
    v = 3.0
    for _ in range(100):
      samples.append(make_sample(t, 1, v, 0.0, accel_cmd=0.0))
      t += dt
    while v > 0.12:
      samples.append(make_sample(t, 1, v, -0.5, should_stop=v < 2.5, accel_cmd=-0.5))
      v = max(v - 0.5 * dt, 0.0)
      t += dt
    if grab:
      # disc grab: a_ego spikes -0.5 -> -1.6 in one 10 ms frame (110 m/s^3) as it settles to ~0.05
      samples.append(make_sample(t, 1, 0.08, -0.5, should_stop=True, accel_cmd=-0.5))
      t += dt
      samples.append(make_sample(t, 1, 0.055, -1.6, should_stop=True, accel_cmd=-0.55))
      t += dt
    else:
      # smooth settle: a_ego eases gently to 0
      for i in range(20):
        samples.append(make_sample(t, 1, max(0.10 - 0.0025 * i, 0.0), -0.5 * (1 - i / 20),
                                   should_stop=True, accel_cmd=-0.4))
        t += dt
    for _ in range(600):
      samples.append(make_sample(t, 1, 0.0, 0.0, should_stop=True, accel_cmd=-0.2))
      t += dt
    return samples

  def test_smooth_settle_metric_computed_and_does_not_gate(self):
    # (d) smooth settle: measured jerk well under the 3.0 diagnostic cap. The METRIC is computed
    # and recorded; classify_event raises no harsh flag.
    samples = self._settle_stream(grab=False)
    metric = bes.settle_meas_jerk(samples, 0, len(samples) - 1)
    assert metric is not None
    assert metric["peak_meas_jerk"] <= sc.SCORING_CONFIG.cranked.terminal_max_settle_meas_jerk
    harsh, _ = sc.classify_event({"settle_peak_meas_jerk": metric["peak_meas_jerk"]})
    assert "harsh_terminal_grab" not in harsh

  def test_grab_settle_metric_computed_but_diagnostic_does_not_gate(self):
    # (d) a hard disc grab: the MEASURED jerk metric is still computed and far over the diagnostic
    # cap -- but P2 is NON-gating (demoted 2026-06-13), so classify_event must NOT return harsh.
    samples = self._settle_stream(grab=True)
    metric = bes.settle_meas_jerk(samples, 0, len(samples) - 1)
    assert metric is not None
    assert metric["peak_meas_jerk"] > sc.SCORING_CONFIG.cranked.terminal_max_settle_meas_jerk
    harsh, _ = sc.classify_event({"settle_peak_meas_jerk": metric["peak_meas_jerk"]})
    assert "harsh_terminal_grab" not in harsh
    assert sc.is_harsh(harsh) is False

  def test_command_only_metric_underreports_the_grab(self):
    # the spec's central finding: MEASURED settle jerk exceeds the SENT-command jerk -> a
    # command-only gate is structurally blind, so the metric must read a_ego (it does)
    samples = self._settle_stream(grab=True)
    metric = bes.settle_meas_jerk(samples, 0, len(samples) - 1)
    assert metric["peak_meas_jerk"] > (metric["peak_sent_jerk"] or 0.0)
    assert metric["meas_minus_sent_jerk"] > 0.0


class TestSettleImuJerk:
  """Faithful terminal-grab channel (eval.md §2.1): settle_imu_jerk reads the device IMU
  longitudinal accel (Sample.a_long_imu = livePose.accelerationDevice.x) instead of wheel-derived
  a_ego. The point of the channel: at standstill a_ego quantizes/floors to ~0 so the v->0 disc-grab
  leaves no wheel signature, but the IMU channel catches it (locationd EKF, gravity-removed)."""

  @staticmethod
  def _imu_settle_stream(imu_grab: bool, dt: float = 0.01) -> list:
    """Stop where the WHEEL a_ego is FLOORED to ~0.03 m/s^2 through the whole final settle (the real
    quantization at v < 0.06 m/s -- no wheel motion to differentiate), so the wheel settle metric is
    blind. The IMU channel (a_long_imu) carries a sharp jolt as v crosses into standstill iff
    `imu_grab`. The settle window terminates at the FIRST v <= SETTLE_STANDSTILL_SPEED (0.06), so the
    grab lands in the final approach frames at/just-before that crossing.

    a_long_imu is HELD at ~20 Hz onto the 100 Hz clock (updates every 5th frame) -- the real wiring
    structure. a_long_imu_raw carries the same physical signal at native 100 Hz (no hold), so the
    deprecated held-100Hz diff manufactures a spike where the honest 20Hz/raw channels do not."""
    samples: list = []
    t = 0.0
    v = 3.0
    frame = 0
    held_imu = 0.0
    while t < 1.0:                                   # cruise (a_ego real, IMU matches)
      samples.append(make_sample(t, 1, v, 0.0, accel_cmd=0.0, a_long_imu=0.0, a_long_imu_raw=0.0))
      t += dt
      frame += 1
    # monotone decel from 3 m/s down through the dead zone to standstill. Through the wheel dead zone
    # (v < ~0.09) the wheel a_ego is FLOORED to a tiny quantized constant (0.03) with no real signal;
    # above it a_ego tracks the real -0.5 decel. The IMU channel (a_long_imu) carries the truth the
    # whole way, including a sharp grab in the last frame before the v <= 0.06 standstill crossing.
    floor_a = 0.03                                   # wheel-aEgo dead-zone quantization floor
    grab_frames = 0
    while v > 0.0:
      in_dead_zone = v <= 0.09
      a_wheel = floor_a if in_dead_zone else -0.5
      # raw IMU at native 100 Hz: real -0.5 brake, easing to ~ -0.05 near standstill; a sharp grab
      # spanning ~5 frames (~50 ms, a realistic sub-100 ms disc-grab) once inside the dead zone.
      a_imu_raw = -0.5 if v > 0.5 else -0.05
      if imu_grab and in_dead_zone and grab_frames < 5:
        a_imu_raw = -2.2                             # sharp grab ~ -2.2 m/s^2
        grab_frames += 1
      if frame % 5 == 0:                             # ~20 Hz livePose update; held in between
        held_imu = a_imu_raw
      samples.append(make_sample(t, 1, v, a_wheel, should_stop=v < 2.5, accel_cmd=-0.5,
                                 a_long_imu=held_imu, a_long_imu_raw=a_imu_raw))
      v = max(v - 0.5 * dt, 0.0)
      t += dt
      frame += 1
    for _ in range(600):                             # standstill hold; a_ego floored, IMU ~0
      samples.append(make_sample(t, 1, 0.0, floor_a, should_stop=True, accel_cmd=-0.2,
                                 a_long_imu=0.0, a_long_imu_raw=0.0))
      t += dt
      frame += 1
    return samples

  def test_imu_catches_grab_that_wheel_aego_misses(self):
    samples = self._imu_settle_stream(imu_grab=True)
    imu = bes.settle_imu_jerk(samples, 0, len(samples) - 1)
    wheel = bes.settle_meas_jerk(samples, 0, len(samples) - 1)
    assert imu is not None
    assert wheel is not None
    # the felt grab is a real -2.2 m/s^2 decel at v ~ 0.085 (the dead zone). The IMU resolves it:
    assert imu["peak_imu_decel"] > 1.5
    assert imu["peak_imu_jerk"] > 100.0
    # but the WHEEL a_ego in the dead zone is floored to the quantization constant (~0.03 m/s^2): it
    # carries no trace of the 2.2 m/s^2 grab. (Its peak jerk reflects only the unrelated -0.5 -> 0.03
    # quantization cliff at the dead-zone edge, an artifact -- not the grab the IMU caught.)
    dead_zone_wheel_decel = max(abs(s.a_ego) for s in samples if 0.0 < s.v_ego <= 0.085)
    assert dead_zone_wheel_decel < 0.10                          # wheel sees nothing -> ~floor
    assert imu["peak_imu_decel"] > 10.0 * dead_zone_wheel_decel  # IMU sees 20x+ the wheel floor

  def test_smooth_settle_reads_low_on_imu(self):
    samples = self._imu_settle_stream(imu_grab=False)
    imu = bes.settle_imu_jerk(samples, 0, len(samples) - 1)
    assert imu is not None
    # a subjectively gentle settle reads much lower than the grab case (monotone-with-harshness)
    grab = bes.settle_imu_jerk(self._imu_settle_stream(imu_grab=True), 0, len(samples) - 1)
    assert imu["peak_imu_jerk"] < grab["peak_imu_jerk"]
    assert imu["peak_imu_decel"] < grab["peak_imu_decel"]

  def test_absent_imu_returns_none(self):
    # qlog-only / pre-livePose route: a_long_imu is None everywhere -> graceful None (no crash)
    samples = self._imu_settle_stream(imu_grab=True)
    for s in samples:
      s.a_long_imu = None
    assert bes.settle_imu_jerk(samples, 0, len(samples) - 1) is None

  def test_recorded_in_both_metric_blocks(self):
    records = bes.ingest_route_samples("r", self._imu_settle_stream(imu_grab=True))
    assert len(records) == 1
    for block in ("metrics_100hz", "metrics_10hz_compat"):
      for key in ("settle_peak_imu_jerk", "settle_peak_imu_jerk_20hz", "settle_peak_imu_decel",
                  "settle_peak_imu_jerk_raw", "settle_peak_imu_decel_raw"):
        assert key in records[0][block], (block, key)
    # the 100 Hz block sees the full-rate grab on the trustworthy decel + the raw filtered jerk
    assert records[0]["metrics_100hz"]["settle_peak_imu_decel"] > 1.5
    assert records[0]["metrics_100hz"]["settle_peak_imu_jerk_raw"] > 0.0

  def test_standard_wheel_stream_has_no_imu_metric(self):
    # synthetic_stop_stream() carries no a_long_imu -> IMU settle is None, wheel settle still works
    records = bes.ingest_route_samples("r", synthetic_stop_stream())
    for block in ("metrics_100hz", "metrics_10hz_compat"):
      for key in ("settle_peak_imu_jerk", "settle_peak_imu_jerk_20hz", "settle_peak_imu_decel",
                  "settle_peak_imu_jerk_raw", "settle_peak_imu_decel_raw"):
        assert records[0][block][key] is None, (block, key)

  def test_deprecated_held100hz_jerk_overstates_honest_channels(self):
    # The deprecated held-100Hz settle_peak_imu_jerk is a RATE-ALIASING ARTIFACT: diffing a ~20 Hz
    # value held across the 0.01 s carState step manufactures a spike. On the grab stream it reads far
    # higher than the honest 20Hz lower bound and the raw filtered channel.
    samples = self._imu_settle_stream(imu_grab=True)
    imu = bes.settle_imu_jerk(samples, 0, len(samples) - 1)
    raw = bes.settle_imu_jerk_raw(samples, 0, len(samples) - 1)
    assert imu is not None and raw is not None
    # held-100Hz artifact >> honest 20Hz (the held repeats compress the real update into one 0.01 s step)
    assert imu["peak_imu_jerk"] > imu["peak_imu_jerk_20hz"]
    # the honest 20Hz channel is a LOWER BOUND -- it must be finite and well under the artifact
    assert imu["peak_imu_jerk_20hz"] is not None and imu["peak_imu_jerk_20hz"] > 0.0

  def test_raw_channel_separates_grab_from_noise(self):
    # The raw filtered channel reads the grab clearly above a gentle settle (faithful), and is finite
    # (filter suppresses the noise floor). Grab stream raw jerk > gentle stream raw jerk.
    grab_samples = self._imu_settle_stream(imu_grab=True)
    gentle_samples = self._imu_settle_stream(imu_grab=False)
    grab = bes.settle_imu_jerk_raw(grab_samples, 0, len(grab_samples) - 1)
    gentle = bes.settle_imu_jerk_raw(gentle_samples, 0, len(gentle_samples) - 1)
    assert grab is not None and gentle is not None
    assert grab["peak_imu_jerk_raw"] > gentle["peak_imu_jerk_raw"]
    assert grab["peak_imu_decel_raw"] > gentle["peak_imu_decel_raw"]

  def test_raw_channel_absent_returns_none(self):
    # No raw accelerometer (qlog-only / pre-livePose) -> graceful None, no crash
    samples = self._imu_settle_stream(imu_grab=True)
    for s in samples:
      s.a_long_imu_raw = None
    assert bes.settle_imu_jerk_raw(samples, 0, len(samples) - 1) is None


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
      for name in ("t", "v_ego", "a_ego", "a_long_imu", "a_long_imu_raw", "accel_cmd", "enabled",
                   "brake_pressed", "should_stop", "lead_status", "lead_v", "lead_d_rel_m",
                   "distance_to_stop_target_m"):
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
