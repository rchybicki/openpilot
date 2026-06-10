"""similarity_gate tests (spec 7.6/7.7 / section 8): tier separation (Tier-2 trace divergence
never fails Tier-1 by itself); class-C blocks; triage table emitted; legacy-vs-legacy dry run
returns all-zero divergence."""

from __future__ import annotations

import json
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
  sys.path.insert(0, str(REPO_ROOT))

from openpilot.tools.stopping import sim_replay as sr
from openpilot.tools.stopping import similarity_gate as sg

DT = 0.1
ESTIMATOR_PASS = {"artifact": "estimator_equivalence", "status": "pass", "push_events": 12}


def _fixture_subset(n: int = 3):
  return sr.fixture_scenarios()[:n]


def _clean_pair(ref: str = "evt", **overrides):
  pair = {
    "event_ref": ref, "stratum": "fixture",
    "harsh_a": False, "harsh_b": False, "leapfrog_a": False, "leapfrog_b": False,
    "harsh_flags_a": [], "harsh_flags_b": [],
    "rollout_delta_m": 0.0, "hold_gap_delta_m": 0.0, "end_jerk_delta": 0.0, "tts_delta_s": 0.0,
    "trace_rms": 0.0, "trace_max_delta": 0.0, "settled_a": True, "settled_b": True,
  }
  pair.update(overrides)
  return pair


class TestLegacyVsLegacyDryRun:
  def test_all_zero_divergence(self):
    report = sg.run_gate(_fixture_subset(), [], {"ref_20260514": sr.PLANT_PARAMS_REF}, DT,
                         triage={}, estimator_report=ESTIMATOR_PASS,
                         controller_a="legacy", controller_b="legacy")
    plant_report = report["per_plant"]["ref_20260514"]
    for pair in plant_report["pairs"]:
      assert pair["trace_rms"] == 0.0, pair["event_ref"]
      assert pair["trace_max_delta"] == 0.0
      assert pair["end_jerk_delta"] in (0.0, None)
      assert pair["rollout_delta_m"] in (0.0, None)
    assert plant_report["tier1_pass"] is True
    assert plant_report["tier2"]["flagged"] == []
    assert report["verdict"] == "pass"


class TestTierSeparation:
  def test_tier2_divergence_alone_does_not_fail_tier1(self):
    pairs = [_clean_pair(f"e{i}") for i in range(9)] + [_clean_pair("e9", trace_rms=0.25, trace_max_delta=0.5)]
    tier1 = sg.tier1_checks(pairs)
    assert all(check["pass"] for check in tier1.values())
    tier2 = sg.tier2_diagnostics(pairs, triage={})
    assert len(tier2["flagged"]) == 1
    assert tier2["unclassified_events"] == ["e9"]

  def test_tier1_harsh_bound_catches_v2_only_harsh(self):
    pairs = [_clean_pair("e0"), _clean_pair("e1", harsh_b=True)]
    tier1 = sg.tier1_checks(pairs)
    assert tier1["harsh_no_v2_only"]["pass"] is False
    assert tier1["harsh_no_v2_only"]["v2_only_harsh_events"] == ["e1"]
    # harsh in BOTH controllers is not a V2 regression
    pairs2 = [_clean_pair("e0", harsh_a=True, harsh_b=True)]
    assert sg.tier1_checks(pairs2)["harsh_no_v2_only"]["pass"] is True

  def test_tier1_leapfrog_per_stratum(self):
    pairs = [_clean_pair("e0", stratum="s1", leapfrog_a=True),
             _clean_pair("e1", stratum="s2", leapfrog_b=True)]
    tier1 = sg.tier1_checks(pairs)
    assert tier1["leapfrog_per_stratum"]["pass"] is False
    rows = {r["stratum"]: r for r in tier1["leapfrog_per_stratum"]["strata"]}
    assert rows["s1"]["pass"] is True   # forest-only leapfrog is fine
    assert rows["s2"]["pass"] is False  # V2-only leapfrog fails its stratum

  def test_tier1_p95_bounds(self):
    pairs = [_clean_pair(f"e{i}", rollout_delta_m=0.05) for i in range(20)]
    assert sg.tier1_checks(pairs)["rollout_delta_p95"]["pass"] is True
    pairs = [_clean_pair(f"e{i}", rollout_delta_m=0.30) for i in range(20)]
    assert sg.tier1_checks(pairs)["rollout_delta_p95"]["pass"] is False
    pairs = [_clean_pair(f"e{i}", hold_gap_delta_m=-0.25) for i in range(20)]
    assert sg.tier1_checks(pairs)["hold_gap_delta_p95"]["pass"] is False
    pairs = [_clean_pair(f"e{i}", tts_delta_s=1.0) for i in range(20)]
    assert sg.tier1_checks(pairs)["time_to_standstill_delta_p95"]["pass"] is False
    # faster-than-forest stops are fine (the bound guards sluggishness only)
    pairs = [_clean_pair(f"e{i}", tts_delta_s=-1.0) for i in range(20)]
    assert sg.tier1_checks(pairs)["time_to_standstill_delta_p95"]["pass"] is True

  def test_tier1_end_jerk_ci_bound(self):
    pairs = [_clean_pair(f"e{i}", end_jerk_delta=0.10 + 0.001 * i) for i in range(30)]
    assert sg.tier1_checks(pairs)["end_jerk_median_ci"]["pass"] is False
    pairs = [_clean_pair(f"e{i}", end_jerk_delta=0.001 * (i % 3 - 1)) for i in range(30)]
    assert sg.tier1_checks(pairs)["end_jerk_median_ci"]["pass"] is True


class TestTriage:
  def test_class_c_blocks_verdict(self):
    pairs = [_clean_pair("bad", trace_rms=0.5, trace_max_delta=0.9)]
    tier2 = sg.tier2_diagnostics(pairs, triage={"bad": {"class": "C", "note": "unexplained"}})
    assert tier2["class_c_events"] == ["bad"]
    assert tier2["unclassified_events"] == []

  def test_class_a_and_b_do_not_block(self):
    pairs = [_clean_pair("a", trace_rms=0.5), _clean_pair("b", trace_max_delta=0.9)]
    tier2 = sg.tier2_diagnostics(pairs, triage={"a": {"class": "A", "note": "forest artifact"},
                                                "b": {"class": "B", "note": "param move"}})
    assert tier2["class_c_events"] == []
    assert tier2["unclassified_events"] == []

  def test_unclassified_flagged_event_blocks(self):
    report = sg.run_gate(_fixture_subset(1), [], {"p": sr.PLANT_PARAMS_REF}, DT,
                         triage={}, estimator_report=ESTIMATOR_PASS,
                         controller_a="legacy", controller_b="legacy")
    # legacy-vs-legacy has zero divergence -- inject a synthetic flagged pair instead
    tier2 = sg.tier2_diagnostics([_clean_pair("x", trace_rms=0.2)], triage={})
    assert tier2["unclassified_events"] == ["x"]
    assert report["verdict"] == "pass"  # the real run stays unaffected

  def test_estimator_row_required(self):
    report = sg.run_gate(_fixture_subset(1), [], {"p": sr.PLANT_PARAMS_REF}, DT,
                         triage={}, estimator_report=None,
                         controller_a="legacy", controller_b="legacy")
    tier1 = report["per_plant"]["p"]["tier1"]
    assert tier1["estimator_equivalence"]["pass"] is False
    assert tier1["estimator_equivalence"]["attached"] is False
    assert report["verdict"] == "fail"

  def test_integrated_dropout_rows_present(self):
    dropout = sg.dropout_fixture_scenarios()[:1]
    assert dropout, "no dropout-hold fixtures matched DROPOUT_FIXTURE_TOKENS"
    report = sg.run_gate(_fixture_subset(1), dropout, {"p": sr.PLANT_PARAMS_REF}, DT,
                         triage={}, estimator_report=ESTIMATOR_PASS,
                         controller_a="legacy", controller_b="legacy")
    rows = report["per_plant"]["p"]["tier1"]["integrated_dropout_hold"]["fixtures"]
    assert len(rows) == 1
    assert rows[0]["pass"] is True


class TestEmittedArtifacts:
  def test_triage_table_markdown(self):
    report = sg.run_gate(_fixture_subset(1), [], {"p": sr.PLANT_PARAMS_REF}, DT,
                         triage={}, estimator_report=ESTIMATOR_PASS,
                         controller_a="legacy", controller_b="legacy")
    table = sg.triage_table_markdown(report)
    assert "triage table" in table
    assert "| plant | event |" in table

  def test_cli_emits_verdict_json_and_triage_table(self, tmp_path: Path):
    estimator_path = tmp_path / "estimator.json"
    estimator_path.write_text(json.dumps(ESTIMATOR_PASS))
    out_path = tmp_path / "similarity.json"
    rc = sg.main(["--plant", "ref", "--b-controller", "legacy",
                  "--event-store", str(tmp_path / "missing_store"),
                  "--estimator-report-json", str(estimator_path),
                  "--output-json", str(out_path)])
    assert rc == 0
    payload = json.loads(out_path.read_text())
    assert payload["verdict"] == "pass"
    assert payload["controller_b"] == "legacy"
    assert out_path.with_suffix(".triage.md").is_file()

  def test_cli_insufficient_without_scenarios(self, tmp_path: Path):
    rc = sg.main(["--skip-fixtures", "--event-store", str(tmp_path / "missing"),
                  "--output-json", str(tmp_path / "out.json")])
    assert rc == 2


class TestStratifiedSample:
  def test_round_robin_cap(self):
    scenarios = []
    for i in range(10):
      scenarios.append(sr.Scenario(name=f"a{i}", samples=[], stratum="s1"))
      scenarios.append(sr.Scenario(name=f"b{i}", samples=[], stratum="s2"))
    picked = sg.stratified_sample(scenarios, 6)
    assert len(picked) == 6
    assert sum(1 for s in picked if s.stratum == "s1") == 3
    assert sum(1 for s in picked if s.stratum == "s2") == 3
