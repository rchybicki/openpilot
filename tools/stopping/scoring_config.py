#!/usr/bin/env python3
"""Frozen, checked-in scoring/gate configuration for the stopping eval harness (spec 7.3 / F29).

GENERATED FROM THE OPERATIVE CODE, NOT TRANSCRIBED FROM PROSE: the values below serialize
  * check_harsh_stops.classify_event's flag-level logic verbatim (classify_event() here DELEGATES
    to that function -- there is exactly one implementation of the flags), and
  * the operative cycle invocation: the run_stopping_cycle measured-gate defaults
    (run_stopping_cycle.py:946-977), incl. the entry-side harsh flags (:960-966), and the two
    cycle-forced leapfrog flags --count-stop-signal-drop-as-leapfrog / --count-exit-stop-as-leapfrog
    (:1387-1388).

The OPERATIVE leapfrog predicate is `is_leapfrog = bool(leapfrog_flags)` -- an OR over ALL flags:
`leapfrog_rebound_signal` and `leapfrog_rebound_should_stop` (either rebound channel ALONE flags),
the rebound-AND-unexpected_accel combination (`leapfrog`, a third flag -- not the definition),
`pre_hold_reaccel`, and the two cycle-forced drop/exit flags. A rebound-only event IS a leapfrog,
exactly as the kept on-road tool counts it -- anything else would break comparability with the
736-event / 0-leapfrog baseline (spec R11).

test_scoring_config.py diffs this module against a recorded check_harsh_stops.classify_event run
and against the cycle parser defaults; any threshold change requires a `version` bump plus a
re-baseline note in docs/stopping/eval.md (spec 7.2/7.3).

Quality buckets and the headline event score are frozen from benchmark_controller_variants.py
:229-245 and check_harsh_stops_model.py:262-312 (both files die in the cleanup commit; the
cutoffs survive here). Both check_harsh_stops.py and run_stopping_cycle.py read their threshold
defaults from this module only.
"""

from __future__ import annotations

import json
from dataclasses import asdict, dataclass, field
from types import SimpleNamespace
from typing import Any

SCORING_CONFIG_VERSION = 4


@dataclass(frozen=True)
class EventFilters:
  """Operative comfort-lane event filters (run_stopping_cycle.py:946-957)."""
  event_source: str = "all"
  min_enabled_ratio: float = 0.80
  min_stop_signal_ratio: float = 0.0
  min_should_stop_ratio: float = 0.15
  min_stopping_state_ratio: float = 0.0
  require_brake_command_below: float = -0.20
  min_entry_speed: float = 0.50
  min_events: int = 2


@dataclass(frozen=True)
class HarshThresholds:
  """Harsh-flag thresholds consumed by check_harsh_stops.classify_event.

  Entry/end values are the operative cycle comfort lane (run_stopping_cycle.py:960-973);
  the remaining values are the check_harsh_stops defaults the cycle never overrides
  (check_harsh_stops.py:40-51), i.e. they are operative too.
  """
  max_entry_stop_jerk: float = 0.35
  max_entry_stop_cmd_jerk: float = 0.50
  max_entry_stop_accel_step: float = 0.08
  max_end_stop_jerk: float = 0.35
  max_end_stop_cmd_jerk: float = 1.0
  max_end_stop_accel_step: float = 0.08
  min_a_ego_floor: float = -1.05
  max_hard_decel_duration: float = 0.75
  min_lead_distance_hold: float = 1.65
  min_far_lead_distance_hold: float = 4.0
  min_far_lead_rollout: float = 3.5
  max_far_lead_min_accel_cmd: float = -0.65
  max_far_lead_min_a_ego: float = -0.85


@dataclass(frozen=True)
class CrankedComfortThresholds:
  """Cranked comfort thresholds (2026-06-13, version 2). Encode the user's two felt-per-stop
  forces directly, principled (clean requirement + cap) and robust across driving-model versions
  (the P1 necessity test keys on radar closing-speed + gap, not a fixed scenario). Read by
  check_harsh_stops.classify_event via the namespace.

  P1 -- unnecessary harsh approach (flag `unnecessary_harsh_approach`) is GATING:
    Requirement: peak commanded decel during the stopping phase WHILE the lead gap is still
    comfortable (> approach_gap_floor_m) must stay <= approach_max_decel, UNLESS kinematically
    necessary to avoid the lead. The exemption is a simple kinematic check at the worst sample:
        required_decel = closing^2 / (2 * max(gap - approach_gap_floor_m, eps))
        violation IFF peak_decel > approach_max_decel AND required_decel <= approach_max_decel
    (i.e. the gap/closing-speed did NOT require harder than the cap). approach_necessary_margin
    keeps borderline-kinematic events (e.g. required ~= cap) on the necessary side. The builder
    precomputes peak_decel / required_decel / necessary on the engaged + long-control-active,
    gap-gated stopping-phase window (build_event_store.approach_decel_over_gap2m). P1 is
    command-measurable and validated, so it stays GATING (contributes to the harsh verdict).

  P2 -- terminal disc-grab is GATING, RE-WIRED 2026-06-14 (version 4) off the FAITHFUL channels after
  the v3 gate metric was found to be a RATE-ALIASING ARTIFACT:
    v3 gated on `settle_peak_imu_jerk`, the peak |d(a_long_imu)/dt| over EVERY consecutive carState
    pair. But a_long_imu (livePose.accelerationDevice.x, ~20 Hz) is HELD onto the 100 Hz carState
    clock, so ~19 of every 20 pairs are repeats and the lone real update divided by the 0.01 s step
    MANUFACTURES a ~70 m/s^3 single-frame spike. On the 36-settle IMU baseline that artifact reads
    med 27 / max 70 m/s^3, but the HONEST native-20Hz update-to-update jerk is only med 6.9 / max 11,
    and the metric mis-ranks: 00001717-seg19 had the corpus-MAX artifact (70.2) yet every honest
    channel calls it gentle (20Hz 1.1, raw 4.8, decel 0.62) -- a pure rate-alias false alarm the v3
    gate would have flagged. So v3 gated the felt grab on an instrument artifact.

    The fidelity work (2026-06-14, eval.md §2.1) built the honest channels and validated which one to
    gate on (build_event_store.settle_imu_jerk / settle_imu_jerk_raw):
      * settle_peak_imu_decel  (PRIMARY) -- peak |a_long_imu| over the settle window. TRUSTWORTHY: it
        agrees with the independent raw-100 Hz pitch-compensated decel within ~3% rank-corr 0.97, is
        not noise-dominated, and ranks the SUBJECTIVELY-harshest stop (00001722 driveway grab) #1 at
        0.99 m/s^2. Baseline (n=36): med 0.67, p60 0.74, p75 0.80, p90 0.88, max 0.99 m/s^2.
      * settle_peak_imu_jerk_raw (SECONDARY) -- peak |d/dt| of the raw ~100 Hz pitch-compensated accel
        (Sample.a_long_imu_raw), MOVING-AVERAGE filtered (5-sample / ~50 ms) to drop the ~31 m/s^3
        parked sensor-noise floor to ~16 max before differentiating. The MOST FAITHFUL channel: it
        resolves the sub-100 ms grab the 20 Hz channel under-resolves (raw jerk is 1.7-5.1x the 20 Hz),
        but it is noisier. Baseline (n=36): med 9.3, p60 10.0, p75 12.2, p90 14.2, max 15.0 m/s^3.
    classify_event raises `harsh_terminal_grab` when EITHER faithful channel exceeds its cap. None-IMU
    events (qlog-only / pre-livePose routes) do NOT raise the flag -- graceful degradation.

    THRESHOLDS: decel cap 0.80 m/s^2 (just above p75 -> flags the rough ~25% tail incl. the felt
    driveway grab, spares the gentle-to-moderate half down to 0.22); raw-jerk cap 13.0 m/s^3 (just
    above p60 of the raw channel -> ~22% tail, and well clear of the ~16 filtered noise-floor max so a
    flag means a real grab, not noise). Both are CRANK KNOBS: tighten the decel toward 0.70 / the raw
    jerk toward 12 as the terminal decel softens.

  DEPRECATED: `terminal_max_settle_imu_jerk` (the v3 held-100Hz jerk cap) is RETAINED in the config for
  back-compat but is NO LONGER GATING (the metric it caps, settle_peak_imu_jerk, is a labeled artifact).
  `terminal_max_settle_meas_jerk` (the WHEEL-aEgo settle cap) likewise stays a NON-gating DIAGNOSTIC --
  a_ego floors to ~0 below ~0.03 m/s so it under-reports the grab. Re-wiring the gate is a THRESHOLD/
  GATING change, so it bumps SCORING_CONFIG_VERSION to 4 (module-header versioning rule)."""
  approach_max_decel: float = 0.5             # user's stated approach cap (m/s^2), GATING (P1)
  approach_gap_floor_m: float = 2.0           # "still comfortable" lead gap boundary (m)
  approach_necessary_margin: float = 0.12     # m/s^2 slack on required_decel to spare borderline-kinematic events
  terminal_max_settle_imu_decel: float = 0.80  # GATING (P2 PRIMARY): max FAITHFUL IMU settle decel (m/s^2); robust, crank knob, see P2
  terminal_max_settle_imu_jerk_raw: float = 13.0  # GATING (P2 SECONDARY): max filtered raw-100Hz settle jerk (m/s^3); faithful sub-100ms grab, see P2
  terminal_max_settle_imu_jerk: float = 30.0  # DEPRECATED (non-gating): v3 held-100Hz ARTIFACT jerk cap (m/s^3); the metric is rate-aliased, see P2
  terminal_max_settle_meas_jerk: float = 3.0  # DIAGNOSTIC (non-gating) WHEEL-aEgo settle jerk cap (m/s^3); under-reports the grab, see P2 above


@dataclass(frozen=True)
class LeapfrogThresholds:
  """Leapfrog-flag thresholds + the two cycle-forced flags (run_stopping_cycle.py:1387-1388)."""
  max_speed_rebound_while_stop_signal: float = 0.08
  max_speed_rebound_while_should_stop: float = 0.08
  max_should_stop_unexpected_accel: float = 0.10
  count_stop_signal_drop_as_leapfrog: bool = True
  count_exit_stop_as_leapfrog: bool = True


@dataclass(frozen=True)
class GateRates:
  """Pass/fail rates of the operative measured gate (run_stopping_cycle.py:958, :974-977)."""
  max_harsh_rate: float = 0.20
  max_harsh_count: int = 0
  max_leapfrog_rate: float = 0.20
  max_leapfrog_count: int = 0


@dataclass(frozen=True)
class QualityBucketRow:
  """One bucket row: ALL bounds must hold (benchmark_controller_variants.py:239-244)."""
  max_score: float
  max_end_jerk: float
  max_cmd_jerk: float
  max_accel_step: float
  min_a_floor: float


@dataclass(frozen=True)
class QualityBuckets:
  """perfect/good/mediocre cutoffs; poor = none matched; hard_fail = harsh or leapfrog."""
  perfect: QualityBucketRow = field(default_factory=lambda: QualityBucketRow(0.14, 0.14, 0.60, 0.025, -0.70))
  good: QualityBucketRow = field(default_factory=lambda: QualityBucketRow(0.24, 0.22, 1.00, 0.045, -0.80))
  mediocre: QualityBucketRow = field(default_factory=lambda: QualityBucketRow(0.40, 0.38, 1.60, 0.065, -0.95))


@dataclass(frozen=True)
class StopContract:
  """Product stop-distance contract (spec 7.3): rollout budgets per spec param #26; final hold
  gap band 2.5-5.0 m ABSOLUTE (no recorded-relative slack -- docs/stopping_behavior_status.md:78
  corrected band, replaces the 2.0-3.5 + recorded slack model-gate variant)."""
  rollout_budget_no_target_m: float = 2.0
  rollout_budget_explicit_m: float = 1.25
  hold_gap_min_m: float = 2.5
  hold_gap_max_m: float = 5.0
  lead_actionable_entry_gap_m: float = 8.0  # check_harsh_stops_model.py:45


@dataclass(frozen=True)
class EventScoreWeights:
  """Headline event-score formula weights, frozen from check_harsh_stops_model.score_event_metrics
  (check_harsh_stops_model.py:262-312): score = end_jerk + 0.8*max(0, -1.0 - min_a)
  + 2.5*max(0, rollout - budget) + lead_gap_term + 0.7*cmd_jerk_overrun + 1.2*accel_step_overrun,
  where the lead-gap term REPLACES the rollout term whenever a lead is actionable."""
  floor_weight: float = 0.8
  floor_ref: float = -1.0
  rollout_weight: float = 2.5
  lead_gap_band_weight: float = 0.15
  lead_gap_excursion_weight: float = 2.5
  cmd_jerk_weight: float = 0.7
  accel_step_weight: float = 1.2


@dataclass(frozen=True)
class DiagnosticMetrics:
  """NON-gating observational diagnostics (spec 7.2). Never read by classify_event or any gate
  predicate -- per the module-header versioning rule, only THRESHOLD changes require a version
  bump, so adding/defining diagnostics here does not.

  hold_acq_*: defines `hold_acq_peak_cmd_jerk` (build_event_store metric blocks) -- peak
  |d(accel_cmd)/dt| in the window [enabled rising edge with v_ego < hold_acq_edge_v_max,
  +hold_acq_window_s], masked to long-control-active samples (active = `enabled` AND
  longControlState != 'off' -- a gas-press override keeps `enabled` true): leading inactive
  frames after the edge are skipped (the enabled/longControlState flips can be a frame apart),
  and the window truncates at the first frame long control goes inactive after having been
  active, because a driver takeover inside the window zeroes the command and that step is a
  takeover artifact, not hold-acquisition ramp shape (mask added 2026-06-12 after the stage-1
  cycle; definition-only change, thresholds unchanged, so no version bump per the module
  rule). None when an event has no low-speed engagement edge
  (normal driving stops engage at speed). Added with the hold-acquisition soften change to
  measure engage-at-standstill / stop-and-go re-engage ramp shape (driveway route
  00001702--dcdc5c3eea--0, 2026-06-10)."""
  hold_acq_edge_v_max: float = 0.3
  hold_acq_window_s: float = 2.0


@dataclass(frozen=True)
class ScriptCliDefaults:
  """check_harsh_stops.py standalone-CLI defaults that DIFFER from the operative gate lane
  (kept byte-identical to the historical script defaults so ad-hoc invocations keep their
  meaning; the operative gate is the comfort lane above). Centralized here so the script has
  no threshold literal of its own (spec 1.3: thresholds read from scoring_config only)."""
  min_events: int = 4
  min_entry_speed: float = 0.20
  max_harsh_rate: float = 0.20
  max_end_stop_jerk: float = 0.75
  max_end_stop_cmd_jerk: float = 3.0
  max_leapfrog_rate: float = 1.0


@dataclass(frozen=True)
class ScoringConfig:
  version: int = SCORING_CONFIG_VERSION
  # 10 Hz provenance: the legacy harsh/leapfrog thresholds were calibrated on the qlog 10 Hz
  # corpus (2,097 events). 100 Hz re-baselining of THOSE happens via bucket-population matching
  # (spec 7.2) + a version bump; until then their 100 Hz verdicts use the same values and
  # historical comparisons MUST use the metrics_10hz_compat block.
  # The cranked comfort thresholds (CrankedComfortThresholds, 2026-06-13/14) are EXEMPT
  # from that 10 Hz provenance: they are scored on metrics_100hz (rlog100 primary) because 10 Hz
  # decimation systematically understates jerk (spec 7.2). BOTH cranked flags are GATING (v4):
  # P1 (unnecessary_harsh_approach) on the command, and P2 (harsh_terminal_grab) on the FAITHFUL IMU
  # channels -- settle_peak_imu_decel (PRIMARY, robust) and the filtered raw-100Hz settle_peak_imu_jerk_raw
  # (SECONDARY, faithful). v3 gated P2 on settle_peak_imu_jerk, found 2026-06-14 to be a rate-aliasing
  # ARTIFACT (held-20Hz value diffed across the 0.01 s carState step); that metric + terminal_max_settle_meas_jerk
  # (wheel-aEgo) are now NON-gating diagnostics. See CrankedComfortThresholds P2 + eval.md section 2.1.
  rate_basis: str = "10hz"
  leapfrog_definition: str = "or_of_flags"  # F29: bool(leapfrog_flags), rebound-only events count
  filters: EventFilters = field(default_factory=EventFilters)
  harsh: HarshThresholds = field(default_factory=HarshThresholds)
  cranked: CrankedComfortThresholds = field(default_factory=CrankedComfortThresholds)
  leapfrog: LeapfrogThresholds = field(default_factory=LeapfrogThresholds)
  gate: GateRates = field(default_factory=GateRates)
  buckets: QualityBuckets = field(default_factory=QualityBuckets)
  contract: StopContract = field(default_factory=StopContract)
  score: EventScoreWeights = field(default_factory=EventScoreWeights)
  script_cli: ScriptCliDefaults = field(default_factory=ScriptCliDefaults)
  diagnostics: DiagnosticMetrics = field(default_factory=DiagnosticMetrics)


SCORING_CONFIG = ScoringConfig()


def canonical_json(config: ScoringConfig = SCORING_CONFIG) -> str:
  """Deterministic serialization embedded into every gate artifact (spec 7.3)."""
  return json.dumps(asdict(config), indent=2, sort_keys=True)


def classify_event_namespace(config: ScoringConfig = SCORING_CONFIG) -> SimpleNamespace:
  """argparse-shaped namespace for check_harsh_stops.classify_event/summarize -- the operative
  gate invocation, attribute-for-attribute what run_stopping_cycle passes on the command line."""
  return SimpleNamespace(
    event_source=config.filters.event_source,
    min_enabled_ratio=config.filters.min_enabled_ratio,
    min_stop_signal_ratio=config.filters.min_stop_signal_ratio,
    min_should_stop_ratio=config.filters.min_should_stop_ratio,
    min_stopping_state_ratio=config.filters.min_stopping_state_ratio,
    require_brake_command_below=config.filters.require_brake_command_below,
    min_events=config.filters.min_events,
    min_entry_speed=config.filters.min_entry_speed,
    max_harsh_rate=config.gate.max_harsh_rate,
    max_harsh_count=config.gate.max_harsh_count,
    max_entry_stop_jerk=config.harsh.max_entry_stop_jerk,
    max_entry_stop_cmd_jerk=config.harsh.max_entry_stop_cmd_jerk,
    max_entry_stop_accel_step=config.harsh.max_entry_stop_accel_step,
    max_end_stop_jerk=config.harsh.max_end_stop_jerk,
    max_end_stop_cmd_jerk=config.harsh.max_end_stop_cmd_jerk,
    max_end_stop_accel_step=config.harsh.max_end_stop_accel_step,
    min_a_ego_floor=config.harsh.min_a_ego_floor,
    max_hard_decel_duration=config.harsh.max_hard_decel_duration,
    min_lead_distance_hold=config.harsh.min_lead_distance_hold,
    min_far_lead_distance_hold=config.harsh.min_far_lead_distance_hold,
    min_far_lead_rollout=config.harsh.min_far_lead_rollout,
    max_far_lead_min_accel_cmd=config.harsh.max_far_lead_min_accel_cmd,
    max_far_lead_min_a_ego=config.harsh.max_far_lead_min_a_ego,
    # cranked comfort thresholds (version 4, 2026-06-14): P1 GATING; P2 GATING via the FAITHFUL IMU
    # channels -- classify_event raises harsh_terminal_grab when settle_peak_imu_decel exceeds
    # terminal_max_settle_imu_decel (PRIMARY, robust) OR settle_peak_imu_jerk_raw exceeds
    # terminal_max_settle_imu_jerk_raw (SECONDARY, faithful). terminal_max_settle_imu_jerk (held-100Hz
    # artifact cap) and terminal_max_settle_meas_jerk (WHEEL-aEgo) ride in the namespace as DEPRECATED/
    # diagnostic reads only -- classify_event no longer gates on either.
    approach_max_decel=config.cranked.approach_max_decel,
    approach_gap_floor_m=config.cranked.approach_gap_floor_m,
    approach_necessary_margin=config.cranked.approach_necessary_margin,
    terminal_max_settle_imu_decel=config.cranked.terminal_max_settle_imu_decel,
    terminal_max_settle_imu_jerk_raw=config.cranked.terminal_max_settle_imu_jerk_raw,
    terminal_max_settle_imu_jerk=config.cranked.terminal_max_settle_imu_jerk,
    terminal_max_settle_meas_jerk=config.cranked.terminal_max_settle_meas_jerk,
    max_leapfrog_rate=config.gate.max_leapfrog_rate,
    max_leapfrog_count=config.gate.max_leapfrog_count,
    max_speed_rebound_while_stop_signal=config.leapfrog.max_speed_rebound_while_stop_signal,
    max_speed_rebound_while_should_stop=config.leapfrog.max_speed_rebound_while_should_stop,
    max_should_stop_unexpected_accel=config.leapfrog.max_should_stop_unexpected_accel,
    count_stop_signal_drop_as_leapfrog=config.leapfrog.count_stop_signal_drop_as_leapfrog,
    count_exit_stop_as_leapfrog=config.leapfrog.count_exit_stop_as_leapfrog,
  )


def classify_event(event: dict[str, Any], config: ScoringConfig = SCORING_CONFIG) -> tuple[list[str], list[str]]:
  """Flag an event with the OPERATIVE classifier: delegates VERBATIM to
  check_harsh_stops.classify_event (single implementation, F29)."""
  from openpilot.tools.stopping.check_harsh_stops import classify_event as _operative_classify_event  # lazy: avoid import cycle

  return _operative_classify_event(event, classify_event_namespace(config))


def is_harsh(harsh_flags: list[str]) -> bool:
  return bool(harsh_flags)


def is_leapfrog(leapfrog_flags: list[str]) -> bool:
  """The operative predicate: OR over ALL leapfrog flags (rebound-only events count, F29)."""
  return bool(leapfrog_flags)


def quality_bucket(score: float, end_jerk: float, cmd_jerk: float, accel_step: float, min_a: float,
                   harsh: bool, leapfrog: bool, config: ScoringConfig = SCORING_CONFIG) -> str:
  """benchmark_controller_variants.comfort_quality_bucket frozen (:229-245)."""
  if harsh or leapfrog:
    return "hard_fail"
  for name in ("perfect", "good", "mediocre"):
    row: QualityBucketRow = getattr(config.buckets, name)
    if (score <= row.max_score and end_jerk <= row.max_end_jerk and cmd_jerk <= row.max_cmd_jerk
        and accel_step <= row.max_accel_step and min_a >= row.min_a_floor):
      return name
  return "poor"


def event_score(*, end_jerk: float, min_a: float, rollout_m: float, lead_entry_gap_m: float | None,
                lead_hold_gap_m: float | None, cmd_jerk: float | None = None, accel_step: float | None = None,
                explicit_target: bool = False, config: ScoringConfig = SCORING_CONFIG) -> float:
  """Headline score frozen from check_harsh_stops_model.score_event_metrics (:262-312), on the
  ABSOLUTE hold-gap contract band (spec 7.3: no recorded-relative slack)."""
  w = config.score
  c = config.contract
  score = max(float(end_jerk or 0.0), 0.0)
  score += w.floor_weight * max(0.0, w.floor_ref - float(min_a))
  lead_actionable = (lead_entry_gap_m is not None and lead_hold_gap_m is not None
                     and 0.0 < lead_entry_gap_m <= c.lead_actionable_entry_gap_m)
  if lead_actionable:
    target = 0.5 * (c.hold_gap_min_m + c.hold_gap_max_m)
    half_band = 0.5 * (c.hold_gap_max_m - c.hold_gap_min_m)
    gap_error = abs(float(lead_hold_gap_m) - target)
    score += w.lead_gap_band_weight * (gap_error / max(half_band, 1e-6))
    score += w.lead_gap_excursion_weight * max(0.0, gap_error - half_band)
  else:
    budget = c.rollout_budget_explicit_m if explicit_target else c.rollout_budget_no_target_m
    score += w.rollout_weight * max(0.0, float(rollout_m or 0.0) - budget)
  if cmd_jerk is not None and config.harsh.max_end_stop_cmd_jerk > 1e-6:
    score += w.cmd_jerk_weight * max(0.0, float(cmd_jerk) - config.harsh.max_end_stop_cmd_jerk) / config.harsh.max_end_stop_cmd_jerk
  if accel_step is not None and config.harsh.max_end_stop_accel_step > 1e-6:
    score += w.accel_step_weight * max(0.0, float(accel_step) - config.harsh.max_end_stop_accel_step) / config.harsh.max_end_stop_accel_step
  return score


if __name__ == "__main__":
  print(canonical_json())
