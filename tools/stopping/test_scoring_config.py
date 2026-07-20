"""scoring_config diff tests (spec 7.3 / F29): the frozen config is GENERATED from the operative
code -- these tests diff the dataclass against a recorded check_harsh_stops.classify_event run and
against the operative cycle invocation defaults. Any threshold change must bump the version."""

from __future__ import annotations

import json
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
  sys.path.insert(0, str(REPO_ROOT))

from openpilot.tools.stopping import scoring_config as sc
from openpilot.tools.stopping.check_harsh_stops import classify_event as operative_classify_event
from openpilot.tools.stopping.scoring_config import SCORING_CONFIG

# A recorded battery covering EVERY flag classify_event can raise, plus clean controls.
# Baseline event: comfortably inside every threshold.
_CLEAN = {
  "entry_speed_mps": 1.5,
  "entry_stop_jerk_mps3": 0.10, "entry_stop_cmd_jerk_mps3": 0.20, "entry_stop_accel_step_mps2": 0.02,
  "end_stop_jerk_mps3": 0.10, "end_stop_cmd_jerk_mps3": 0.30, "end_stop_accel_step_mps2": 0.02,
  "min_a_ego_mps2": -0.60, "hard_decel_duration_s": 0.0, "lead_distance_hold_m": 3.0,
  "min_accel_cmd_mps2": -0.40, "rollout_distance_from_2mps_m": 1.0,
  "speed_rebound_while_stop_signal_mps": 0.0, "speed_rebound_while_should_stop_mps": 0.0,
  "should_stop_unexpected_accel_mps2": 0.0, "reaccel_before_hold": False,
  "stop_signal_dropped_before_hold": False, "left_stopping_state_before_hold": False,
  # cranked comfort metrics (version 4, 2026-06-14): clean = gentle approach + smooth settle.
  # The GATING faithful channels are settle_peak_imu_decel (PRIMARY, < 0.80 m/s^2 cap) and
  # settle_peak_imu_jerk_raw (SECONDARY, < 13.0 m/s^3 cap); both well under here. settle_peak_imu_jerk
  # (held-100Hz ARTIFACT) and the wheel-aEgo settle_peak_meas_jerk are non-gating diagnostics.
  "approach_peak_decel_over_gap2m": 0.30, "approach_required_decel_to_2m": 0.10,
  "settle_peak_imu_decel": 0.40, "settle_peak_imu_jerk_raw": 6.0,
  "settle_peak_imu_jerk": 24.0, "settle_peak_meas_jerk": 2.0,
}

RECORDED_BATTERY = [
  ("clean", {}, [], []),
  ("entry_jerk", {"entry_stop_jerk_mps3": 0.40}, ["entry_stop_jerk"], []),
  ("entry_cmd_jerk", {"entry_stop_cmd_jerk_mps3": 0.60}, ["entry_stop_cmd_jerk"], []),
  ("entry_accel_step", {"entry_stop_accel_step_mps2": 0.09}, ["entry_stop_accel_step"], []),
  ("end_jerk", {"end_stop_jerk_mps3": 0.40}, ["end_stop_jerk"], []),
  ("end_cmd_jerk", {"end_stop_cmd_jerk_mps3": 1.20}, ["end_stop_cmd_jerk"], []),
  ("end_accel_step", {"end_stop_accel_step_mps2": 0.09}, ["end_stop_accel_step"], []),
  ("hard_min_a", {"min_a_ego_mps2": -1.20}, ["hard_min_a_ego"], []),
  ("sustained_hard_decel", {"hard_decel_duration_s": 0.90}, ["sustained_hard_decel"], []),
  ("tight_lead_hold", {"lead_distance_hold_m": 1.20}, ["tight_lead_hold"], []),
  ("far_lead_brake_spike",
   {"lead_distance_hold_m": 4.5, "rollout_distance_from_2mps_m": 4.0, "min_accel_cmd_mps2": -0.80, "min_a_ego_mps2": -0.90},
   ["far_lead_brake_spike"], []),
  # cranked-requirement P1 (2026-06-13): UNNECESSARY harsh approach -- peak > 0.5 cap AND the
  # kinematics did not require it (required_decel <= cap)
  ("unnecessary_harsh_approach",
   {"approach_peak_decel_over_gap2m": 0.85, "approach_required_decel_to_2m": 0.10},
   ["unnecessary_harsh_approach"], []),
  # NECESSARY harsh approach (close fast lead, required >> cap) is EXEMPT -> no flag
  ("necessary_harsh_approach_exempt",
   {"approach_peak_decel_over_gap2m": 1.70, "approach_required_decel_to_2m": 1.22},
   [], []),
  # cranked-requirement P2 (2026-06-14, RE-WIRED off the FAITHFUL channels): harsh_terminal_grab fires
  # when EITHER the PRIMARY decel (settle_peak_imu_decel > 0.80 m/s^2, robust) OR the SECONDARY filtered
  # raw jerk (settle_peak_imu_jerk_raw > 13.0 m/s^3, faithful sub-100ms grab) exceeds its cap. The
  # held-100Hz settle_peak_imu_jerk (rate-aliasing ARTIFACT) and the wheel-aEgo settle_peak_meas_jerk
  # are NON-gating diagnostics -- a high value on either does NOT flag.
  ("terminal_grab_decel_is_harsh", {"settle_peak_imu_decel": 0.92}, ["harsh_terminal_grab"], []),
  ("terminal_grab_raw_jerk_is_harsh", {"settle_peak_imu_jerk_raw": 14.5}, ["harsh_terminal_grab"], []),
  ("terminal_grab_artifact_jerk_is_diagnostic_not_harsh", {"settle_peak_imu_jerk": 70.0}, [], []),
  ("terminal_grab_wheel_is_diagnostic_not_harsh", {"settle_peak_meas_jerk": 5.5}, [], []),
  # F29 pivot: EITHER rebound channel ALONE flags -- a rebound-only event IS a leapfrog
  ("rebound_signal_only", {"speed_rebound_while_stop_signal_mps": 0.12}, [], ["leapfrog_rebound_signal"]),
  ("rebound_should_stop_only", {"speed_rebound_while_should_stop_mps": 0.12}, [], ["leapfrog_rebound_should_stop"]),
  # the rebound AND unexpected_accel combination is a THIRD flag, not the definition
  ("rebound_plus_unexpected",
   {"speed_rebound_while_stop_signal_mps": 0.12, "should_stop_unexpected_accel_mps2": 0.20},
   [], ["leapfrog_rebound_signal", "leapfrog"]),
  ("unexpected_alone", {"should_stop_unexpected_accel_mps2": 0.20}, [], []),
  ("reaccel_before_hold", {"reaccel_before_hold": True}, [], ["pre_hold_reaccel"]),
  # the two cycle-forced flags (run_stopping_cycle.py:1387-1388) are ON in the operative config
  ("stop_signal_drop", {"stop_signal_dropped_before_hold": True}, [], ["stop_signal_drop"]),
  ("exit_stopping_state", {"left_stopping_state_before_hold": True}, [], ["exit_stopping_state"]),
]


def _event(overrides: dict) -> dict:
  return {**_CLEAN, **overrides}


class TestClassifyEventDiff:
  """The config classifier must be VERBATIM the operative classify_event (single implementation)."""

  @pytest.mark.parametrize("name,overrides,want_harsh,want_leapfrog", RECORDED_BATTERY)
  def test_recorded_run_matches(self, name, overrides, want_harsh, want_leapfrog):
    event = _event(overrides)
    harsh, leapfrog = sc.classify_event(event)
    harsh_op, leapfrog_op = operative_classify_event(event, sc.classify_event_namespace())
    assert (harsh, leapfrog) == (harsh_op, leapfrog_op), name
    assert harsh == want_harsh, name
    assert leapfrog == want_leapfrog, name

  def test_rebound_only_event_is_a_leapfrog(self):
    # F29: the operative predicate is OR-of-flags; rebound-alone counts
    _, leapfrog = sc.classify_event(_event({"speed_rebound_while_stop_signal_mps": 0.12}))
    assert sc.is_leapfrog(leapfrog) is True
    assert SCORING_CONFIG.leapfrog_definition == "or_of_flags"

  def test_is_harsh_is_leapfrog_are_or_of_flags(self):
    assert sc.is_harsh([]) is False
    assert sc.is_leapfrog([]) is False
    assert sc.is_harsh(["end_stop_jerk"]) is True
    assert sc.is_leapfrog(["pre_hold_reaccel"]) is True


class TestOperativeCycleInvocation:
  """The config must equal the operative cycle measured-gate defaults (run_stopping_cycle.py:946-977)."""

  @pytest.fixture()
  def cycle_args(self, monkeypatch):
    from openpilot.tools.stopping.run_stopping_cycle import parse_args
    monkeypatch.setattr(sys, "argv", ["run_stopping_cycle.py"])
    return parse_args()

  def test_measured_gate_defaults_match_config(self, cycle_args):
    cfg = SCORING_CONFIG
    assert cycle_args.measured_gate_min_enabled_ratio == cfg.filters.min_enabled_ratio
    assert cycle_args.measured_gate_min_stop_signal_ratio == cfg.filters.min_stop_signal_ratio
    assert cycle_args.measured_gate_min_should_stop_ratio == cfg.filters.min_should_stop_ratio
    assert cycle_args.measured_gate_require_brake_command_below == cfg.filters.require_brake_command_below
    assert cycle_args.measured_gate_min_events == cfg.filters.min_events
    assert cycle_args.measured_gate_min_entry_speed == cfg.filters.min_entry_speed
    assert cycle_args.measured_gate_max_harsh_rate == cfg.gate.max_harsh_rate
    assert cycle_args.measured_gate_max_entry_stop_jerk == cfg.harsh.max_entry_stop_jerk
    assert cycle_args.measured_gate_max_entry_stop_cmd_jerk == cfg.harsh.max_entry_stop_cmd_jerk
    assert cycle_args.measured_gate_max_entry_stop_accel_step == cfg.harsh.max_entry_stop_accel_step
    assert cycle_args.measured_gate_max_end_stop_jerk == cfg.harsh.max_end_stop_jerk
    assert cycle_args.measured_gate_max_end_stop_cmd_jerk == cfg.harsh.max_end_stop_cmd_jerk
    assert cycle_args.measured_gate_max_end_stop_accel_step == cfg.harsh.max_end_stop_accel_step
    assert cycle_args.measured_gate_min_a_ego_floor == cfg.harsh.min_a_ego_floor
    assert cycle_args.measured_gate_max_leapfrog_rate == cfg.gate.max_leapfrog_rate
    assert cycle_args.measured_gate_max_leapfrog_count == cfg.gate.max_leapfrog_count

  def test_cycle_forced_leapfrog_flags_are_on(self):
    # run_stopping_cycle.py:1387-1388 always passes both flags -- the frozen config says so
    assert SCORING_CONFIG.leapfrog.count_stop_signal_drop_as_leapfrog is True
    assert SCORING_CONFIG.leapfrog.count_exit_stop_as_leapfrog is True
    ns = sc.classify_event_namespace()
    assert ns.count_stop_signal_drop_as_leapfrog is True
    assert ns.count_exit_stop_as_leapfrog is True

  def test_script_cli_defaults_match_check_harsh_stops_parser(self, monkeypatch):
    from openpilot.tools.stopping.check_harsh_stops import parse_args as harsh_parse_args
    monkeypatch.setattr(sys, "argv", ["check_harsh_stops.py", "--summary-json", "x.json"])
    args = harsh_parse_args()
    cli = SCORING_CONFIG.script_cli
    assert args.min_events == cli.min_events
    assert args.min_entry_speed == cli.min_entry_speed
    assert args.max_harsh_rate == cli.max_harsh_rate
    assert args.max_end_stop_jerk == cli.max_end_stop_jerk
    assert args.max_end_stop_cmd_jerk == cli.max_end_stop_cmd_jerk
    assert args.max_leapfrog_rate == cli.max_leapfrog_rate
    # metric thresholds shared with the gate lane
    assert args.max_end_stop_accel_step == SCORING_CONFIG.harsh.max_end_stop_accel_step
    assert args.min_a_ego_floor == SCORING_CONFIG.harsh.min_a_ego_floor
    assert args.max_hard_decel_duration == SCORING_CONFIG.harsh.max_hard_decel_duration
    assert args.min_lead_distance_hold == SCORING_CONFIG.harsh.min_lead_distance_hold
    assert args.max_speed_rebound_while_stop_signal == SCORING_CONFIG.leapfrog.max_speed_rebound_while_stop_signal
    assert args.max_speed_rebound_while_should_stop == SCORING_CONFIG.leapfrog.max_speed_rebound_while_should_stop
    assert args.max_should_stop_unexpected_accel == SCORING_CONFIG.leapfrog.max_should_stop_unexpected_accel


class TestCanonicalJson:
  def test_deterministic_and_versioned(self):
    text1 = sc.canonical_json()
    text2 = sc.canonical_json()
    assert text1 == text2
    payload = json.loads(text1)
    assert payload["version"] == sc.SCORING_CONFIG_VERSION
    assert sc.SCORING_CONFIG_VERSION == 5  # v5: hold-gap floor 2.5 -> 3.0 (2026-07-20 band retune)
    assert payload["rate_basis"] == "10hz"
    assert payload["leapfrog"]["count_stop_signal_drop_as_leapfrog"] is True
    # the cranked block rides in the serialized config (spec 7.3)
    assert payload["cranked"]["approach_max_decel"] == 0.5
    assert payload["cranked"]["approach_gap_floor_m"] == 2.0
    assert payload["cranked"]["terminal_max_settle_imu_decel"] == 0.80    # GATING PRIMARY (v4)
    assert payload["cranked"]["terminal_max_settle_imu_jerk_raw"] == 13.0  # GATING SECONDARY (v4)
    assert payload["cranked"]["terminal_max_settle_imu_jerk"] == 30.0  # DEPRECATED artifact (non-gating)
    assert payload["cranked"]["terminal_max_settle_meas_jerk"] == 3.0  # diagnostic (non-gating)

  def test_config_is_frozen(self):
    import dataclasses
    with pytest.raises(dataclasses.FrozenInstanceError):
      SCORING_CONFIG.version = 99  # type: ignore[misc]


class TestCrankedComfortThresholds:
  """Cranked comfort thresholds (version 4, 2026-06-14): BOTH P1 and P2 are GATING harsh flags.
  P1 (unnecessary_harsh_approach) must EXEMPT kinematically necessary braking. P2 (harsh_terminal_grab)
  was RE-WIRED on 2026-06-14 off the FAITHFUL IMU channels after the v3 gate metric (settle_peak_imu_jerk)
  was found to be a rate-aliasing artifact: classify_event sets the harsh verdict when EITHER the PRIMARY
  decel (settle_peak_imu_decel > terminal_max_settle_imu_decel, robust) OR the SECONDARY filtered raw jerk
  (settle_peak_imu_jerk_raw > terminal_max_settle_imu_jerk_raw, faithful) exceeds its cap (eval.md section
  2.1). The held-100Hz settle cap (terminal_max_settle_imu_jerk) and the wheel-aEgo settle cap
  (terminal_max_settle_meas_jerk) are retained as NON-gating diagnostics only."""

  def test_thresholds_flow_into_the_namespace(self):
    ns = sc.classify_event_namespace()
    cfg = SCORING_CONFIG.cranked
    assert ns.approach_max_decel == cfg.approach_max_decel == 0.5
    assert ns.approach_gap_floor_m == cfg.approach_gap_floor_m == 2.0
    assert ns.approach_necessary_margin == cfg.approach_necessary_margin
    # P2 FAITHFUL gating thresholds flow into the namespace (classify_event gates on these).
    assert ns.terminal_max_settle_imu_decel == cfg.terminal_max_settle_imu_decel == 0.80
    assert ns.terminal_max_settle_imu_jerk_raw == cfg.terminal_max_settle_imu_jerk_raw == 13.0
    # P2 deprecated/diagnostic thresholds are retained for the diagnostic read (non-gating).
    assert ns.terminal_max_settle_imu_jerk == cfg.terminal_max_settle_imu_jerk == 30.0
    assert ns.terminal_max_settle_meas_jerk == cfg.terminal_max_settle_meas_jerk == 3.0

  def test_approach_exemption_boundary(self):
    # at the cap with required <= cap: not a violation (must EXCEED, strict)
    harsh, _ = sc.classify_event(_event({"approach_peak_decel_over_gap2m": 0.50,
                                         "approach_required_decel_to_2m": 0.10}))
    assert "unnecessary_harsh_approach" not in harsh
    # just over the cap, kinematics did not require it -> violation
    harsh, _ = sc.classify_event(_event({"approach_peak_decel_over_gap2m": 0.51,
                                         "approach_required_decel_to_2m": 0.10}))
    assert "unnecessary_harsh_approach" in harsh
    # just over the cap, but kinematics required more than the cap -> exempt
    harsh, _ = sc.classify_event(_event({"approach_peak_decel_over_gap2m": 0.90,
                                         "approach_required_decel_to_2m": 0.60}))
    assert "unnecessary_harsh_approach" not in harsh

  def test_missing_metrics_never_flag(self):
    # events with no engaged gap-gated approach / no engaged settle (metric None) must not flag
    harsh, _ = sc.classify_event({"entry_speed_mps": 1.5})
    assert "unnecessary_harsh_approach" not in harsh
    assert "harsh_terminal_grab" not in harsh

  def test_required_decel_none_treated_as_unnecessary(self):
    # no usable kinematic frame (required None) + a hard brake over the cap -> still flagged
    harsh, _ = sc.classify_event(_event({"approach_peak_decel_over_gap2m": 0.80,
                                         "approach_required_decel_to_2m": None}))
    assert "unnecessary_harsh_approach" in harsh

  def test_terminal_grab_primary_decel_is_gating(self):
    # RE-WIRED 2026-06-14: the PRIMARY faithful channel (settle_peak_imu_decel over the 0.80 m/s^2 cap)
    # makes classify_event return harsh via harsh_terminal_grab.
    harsh, leapfrog = sc.classify_event(_event({"settle_peak_imu_decel": 0.92}))
    assert "harsh_terminal_grab" in harsh
    assert sc.is_harsh(harsh) is True
    assert leapfrog == []
    # just over the gating threshold: flagged (strict >)
    harsh, _ = sc.classify_event(_event({"settle_peak_imu_decel": 0.801}))
    assert "harsh_terminal_grab" in harsh
    # at or below the cap: not flagged
    harsh, _ = sc.classify_event(_event({"settle_peak_imu_decel": 0.80}))
    assert "harsh_terminal_grab" not in harsh
    harsh, _ = sc.classify_event(_event({"settle_peak_imu_decel": 0.50}))
    assert "harsh_terminal_grab" not in harsh

  def test_terminal_grab_secondary_raw_jerk_is_gating(self):
    # the SECONDARY faithful channel (filtered raw-100Hz jerk over the 13.0 m/s^3 cap) ALSO flags
    harsh, _ = sc.classify_event(_event({"settle_peak_imu_jerk_raw": 14.5}))
    assert "harsh_terminal_grab" in harsh
    harsh, _ = sc.classify_event(_event({"settle_peak_imu_jerk_raw": 13.01}))
    assert "harsh_terminal_grab" in harsh
    harsh, _ = sc.classify_event(_event({"settle_peak_imu_jerk_raw": 13.0}))
    assert "harsh_terminal_grab" not in harsh

  def test_terminal_grab_artifact_jerk_stays_diagnostic(self):
    # the held-100Hz settle_peak_imu_jerk is a RATE-ALIASING ARTIFACT, now NON-gating: even at the
    # corpus-max 70 m/s^3 it must NOT by itself set the harsh verdict (the faithful channels are clean).
    harsh, leapfrog = sc.classify_event(_event({"settle_peak_imu_jerk": 70.0}))
    assert "harsh_terminal_grab" not in harsh
    assert sc.is_harsh(harsh) is False
    assert leapfrog == []

  def test_terminal_grab_wheel_aego_stays_diagnostic(self):
    # the WHEEL-aEgo settle jerk (settle_peak_meas_jerk) is a non-gating diagnostic: even far over
    # its (retained) 3.0 m/s^3 cap, it must NOT by itself set the harsh verdict.
    harsh, leapfrog = sc.classify_event(_event({"settle_peak_meas_jerk": 5.5}))
    assert "harsh_terminal_grab" not in harsh
    assert sc.is_harsh(harsh) is False  # clean IMU settle + only a high wheel jerk is NOT harsh
    assert leapfrog == []

  def test_terminal_grab_missing_imu_never_flags(self):
    # None IMU settle (qlog-only / pre-livePose route) must never raise harsh_terminal_grab even
    # if the diagnostics are high -- never gate on a missing signal (graceful degradation).
    ev = _event({"settle_peak_meas_jerk": 9.0, "settle_peak_imu_jerk": 70.0})
    ev["settle_peak_imu_decel"] = None
    ev["settle_peak_imu_jerk_raw"] = None
    harsh, _ = sc.classify_event(ev)
    assert "harsh_terminal_grab" not in harsh
    assert sc.is_harsh(harsh) is False


class TestQualityBuckets:
  def test_bucket_boundaries(self):
    qb = sc.quality_bucket
    assert qb(0.10, 0.10, 0.50, 0.02, -0.60, harsh=False, leapfrog=False) == "perfect"
    assert qb(0.20, 0.20, 0.90, 0.04, -0.75, harsh=False, leapfrog=False) == "good"
    assert qb(0.35, 0.35, 1.50, 0.06, -0.90, harsh=False, leapfrog=False) == "mediocre"
    assert qb(0.50, 0.50, 2.00, 0.10, -1.00, harsh=False, leapfrog=False) == "poor"
    assert qb(0.10, 0.10, 0.50, 0.02, -0.60, harsh=True, leapfrog=False) == "hard_fail"
    assert qb(0.10, 0.10, 0.50, 0.02, -0.60, harsh=False, leapfrog=True) == "hard_fail"

  def test_cutoffs_match_benchmark_controller_variants(self):
    # the donor file dies in the cleanup commit; while it exists, diff the frozen cutoffs
    # against its operative comfort_quality_bucket (skip cleanly post-deletion / clean checkout)
    bcv = pytest.importorskip("openpilot.tools.stopping.benchmark_controller_variants")
    cases = [
      (0.10, 0.10, 0.50, 0.020, -0.60), (0.14, 0.14, 0.60, 0.025, -0.70),
      (0.20, 0.20, 0.90, 0.040, -0.75), (0.24, 0.22, 1.00, 0.045, -0.80),
      (0.35, 0.35, 1.50, 0.060, -0.90), (0.40, 0.38, 1.60, 0.065, -0.95),
      (0.50, 0.50, 2.00, 0.100, -1.00), (0.141, 0.14, 0.60, 0.025, -0.70),
    ]
    for score, end_jerk, cmd_jerk, step, min_a in cases:
      metrics = bcv.VariantMetrics(
        pred_end_stop_jerk_mps3=end_jerk, pred_end_stop_cmd_jerk_mps3=cmd_jerk,
        pred_end_stop_accel_step_mps2=step, pred_min_a_ego_mps2=min_a,
        pred_rollout_distance_m=0.0, pred_lead_distance_stop_entry_m=None,
        pred_lead_distance_hold_m=None, recorded_lead_distance_hold_m=None,
        distance_gate_source="", pred_speed_rebound_while_should_stop_mps=None,
        pred_should_stop_unexpected_accel_mps2=None, event_score=score,
        is_harsh=False, flags=[], is_leapfrog=False, leapfrog_flags=[],
      )
      assert sc.quality_bucket(score, end_jerk, cmd_jerk, step, min_a, harsh=False, leapfrog=False) == bcv.comfort_quality_bucket(metrics)


class TestEventScore:
  def test_rollout_replaced_by_lead_gap_when_actionable(self):
    base = dict(end_jerk=0.2, min_a=-0.8, rollout_m=3.5, cmd_jerk=None, accel_step=None)
    no_lead = sc.event_score(lead_entry_gap_m=None, lead_hold_gap_m=None, **base)
    with_lead = sc.event_score(lead_entry_gap_m=6.0, lead_hold_gap_m=4.0, **base)  # band center (3.0-5.0)
    # rollout overrun penalized without a lead; centered lead-hold zeroes the rollout term
    assert no_lead > with_lead
    assert with_lead == pytest.approx(0.2, abs=1e-9)

  def test_hold_gap_contract_is_absolute(self):
    # spec 7.3: 3.0-5.0 m absolute (floor 3.0 since the 2026-07-20 band retune), no
    # recorded-relative slack -- mid-band 4.0 scores zero gap term
    mid = sc.event_score(end_jerk=0.0, min_a=0.0, rollout_m=0.0, lead_entry_gap_m=5.0, lead_hold_gap_m=4.0)
    edge = sc.event_score(end_jerk=0.0, min_a=0.0, rollout_m=0.0, lead_entry_gap_m=5.0, lead_hold_gap_m=5.0)
    out = sc.event_score(end_jerk=0.0, min_a=0.0, rollout_m=0.0, lead_entry_gap_m=5.0, lead_hold_gap_m=6.0)
    assert mid == pytest.approx(0.0)
    assert edge == pytest.approx(SCORING_CONFIG.score.lead_gap_band_weight)
    assert out > edge + 2.0  # excursion term kicks in past the band edge
