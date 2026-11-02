"""paired_stats tests (spec 7.4 / section 8): known-distribution fixtures, MDE field mandatory,
refusal below the power floor prints the required n. Pure python + numpy."""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import numpy as np
import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
  sys.path.insert(0, str(REPO_ROOT))

from openpilot.tools.stopping import paired_stats as ps


def _paired_events(deltas, base=0.30, harsh_b: int = 0, leapfrog_b: int = 0):
  """Build (events_a, events_b) sharing stable keys with end_jerk shifted by deltas."""
  events_a, events_b = [], []
  for i, d in enumerate(deltas):
    key = {"route": "r1", "seg": i // 10, "hold_mono_ns": 1000 + i}
    events_a.append({"key": key, "end_jerk": base, "min_a": -0.7, "rollout_m": 1.0,
                     "hold_gap_m": 3.0, "time_to_standstill_s": 4.0, "harsh": False, "leapfrog": False})
    events_b.append({"key": key, "end_jerk": base + d, "min_a": -0.7, "rollout_m": 1.0,
                     "hold_gap_m": 3.0, "time_to_standstill_s": 4.0,
                     "harsh": i < harsh_b, "leapfrog": i < leapfrog_b})
  return events_a, events_b


class TestPrimitives:
  def test_wilcoxon_detects_a_clear_shift(self):
    rng = np.random.default_rng(7)
    deltas = rng.normal(0.10, 0.05, size=300)
    result = ps.wilcoxon_signed_rank(deltas)
    assert result["p_two_sided"] < 1e-6

  def test_wilcoxon_null_is_insignificant(self):
    rng = np.random.default_rng(8)
    deltas = rng.normal(0.0, 0.05, size=300)
    assert ps.wilcoxon_signed_rank(deltas)["p_two_sided"] > 0.01

  def test_mcnemar_exact_known_value(self):
    # b=1, c=9: two-sided exact p = 2 * sum_{i<=1} C(10,i)/2^10 = 22/1024
    result = ps.mcnemar_exact(1, 9)
    assert result["p_two_sided"] == pytest.approx(22 / 1024)

  def test_mcnemar_no_discordant_pairs(self):
    assert ps.mcnemar_exact(0, 0)["p_two_sided"] == 1.0

  def test_mann_whitney_detects_separation(self):
    rng = np.random.default_rng(9)
    x = rng.normal(0.0, 1.0, 200)
    y = rng.normal(0.8, 1.0, 200)
    assert ps.mann_whitney_u(x, y)["p_two_sided"] < 1e-6

  def test_bca_ci_brackets_the_sample_mean(self):
    rng = np.random.default_rng(10)
    data = rng.normal(0.5, 0.1, 250)
    lo, hi = ps.bca_bootstrap_ci(data, lambda x: float(np.mean(x)), n_boot=800)
    assert lo < float(np.mean(data)) < hi
    assert hi - lo < 0.06  # ~4 standard errors at n=250, sd=0.1

  def test_mde_shrinks_with_n(self):
    rng = np.random.default_rng(11)
    small = rng.normal(0, 0.1, 50)
    large = rng.normal(0, 0.1, 500)
    assert ps.mde_paired(large) < ps.mde_paired(small)


class TestPairedCompare:
  def test_regression_detected(self):
    rng = np.random.default_rng(12)
    events_a, events_b = _paired_events(rng.normal(0.08, 0.02, 250))
    report = ps.compare_paired(events_a, events_b)
    end_jerk = next(m for m in report["metrics"] if m["metric"] == "end_jerk")
    assert end_jerk["status"] == "regressed"
    assert report["verdict"] == "regressed"

  def test_improvement_detected(self):
    rng = np.random.default_rng(13)
    events_a, events_b = _paired_events(rng.normal(-0.08, 0.02, 250))
    report = ps.compare_paired(events_a, events_b)
    end_jerk = next(m for m in report["metrics"] if m["metric"] == "end_jerk")
    assert end_jerk["status"] == "improved"
    assert report["verdict"] == "no_regression"

  def test_null_is_no_significant_change(self):
    rng = np.random.default_rng(14)
    events_a, events_b = _paired_events(rng.normal(0.0, 0.05, 250))
    report = ps.compare_paired(events_a, events_b)
    end_jerk = next(m for m in report["metrics"] if m["metric"] == "end_jerk")
    assert end_jerk["status"] == "no_significant_change"

  def test_mde_field_present_on_every_metric(self):
    rng = np.random.default_rng(15)
    events_a, events_b = _paired_events(rng.normal(0.0, 0.05, 250))
    report = ps.compare_paired(events_a, events_b)
    for row in report["metrics"]:
      assert "mde_at_n" in row, row["metric"]
      assert "n" in row

  def test_refusal_below_floor_prints_required_n(self):
    rng = np.random.default_rng(16)
    # a small observed delta vs its sd => the n required to resolve it exceeds the observed n
    events_a, events_b = _paired_events(rng.normal(0.005, 0.05, 40))  # 40 << 200 floor
    report = ps.compare_paired(events_a, events_b)
    assert report["verdict"] == "refused_insufficient_power"
    end_jerk = next(m for m in report["metrics"] if m["metric"] == "end_jerk")
    assert end_jerk["status"] == "refused_insufficient_power"
    assert end_jerk["required_n_for_observed_delta"] is not None
    assert end_jerk["required_n_for_observed_delta"] > 40

  def test_binary_mcnemar_regression(self):
    events_a, events_b = _paired_events(np.zeros(250), harsh_b=40)
    report = ps.compare_paired(events_a, events_b)
    harsh = next(m for m in report["metrics"] if m["metric"] == "harsh")
    assert harsh["status"] == "regressed"
    assert harsh["discordant_b_only"] == 40

  def test_join_uses_stable_keys(self):
    events_a, events_b = _paired_events(np.zeros(10))
    events_b = events_b[5:] + [{"key": {"route": "other", "seg": 0, "hold_mono_ns": 1}, "end_jerk": 1.0}]
    pairs = ps.join_paired(events_a, events_b)
    assert len(pairs) == 5

  def test_legacy_route_event_id_keys_still_join(self):
    a = [{"route": "r", "event_id": 3, "end_jerk": 0.2}]
    b = [{"route": "r", "event_id": 3, "end_jerk": 0.3}]
    assert len(ps.join_paired(a, b)) == 1


class TestOnroadCompare:
  def _arm(self, rng, n, jerk_mean, v_mean=1.5, lead=False, sv=1, isd=None):
    """isd=None omits entry.isd_m (legacy shape, fails the cross-era precondition);
    isd=0.0 is the event-store shape on this car (device runs ISD = 0.0)."""
    events = []
    for _ in range(n):
      entry = {"v_approach": float(rng.uniform(0.3, 3.0)), "lead_entry_gap_m": 5.0 if lead else None}
      if isd is not None:
        entry["isd_m"] = isd
      events.append({
        "end_jerk": float(rng.normal(jerk_mean, 0.05)),
        "entry": entry,
        "v_approach": float(rng.uniform(0.3, 3.0)),
        "signals_version": sv,
      })
    return events

  def test_stratified_regression_detected(self):
    rng = np.random.default_rng(17)
    before = self._arm(rng, 200, 0.30)
    after = self._arm(rng, 200, 0.40)
    report = ps.compare_onroad(before, after)
    assert report["verdict"] == "regressed"
    assert report["mde_at_n"] > 0.0

  def test_refusal_below_per_arm_floor(self):
    rng = np.random.default_rng(18)
    report = ps.compare_onroad(self._arm(rng, 60, 0.3), self._arm(rng, 60, 0.5))
    assert report["verdict"] == "refused_insufficient_power"
    assert report["required_n_per_arm"] == ps.ONROAD_MIN_STOPS_PER_ARM

  def test_cross_era_rule_engages_with_all_zero_isd(self):
    # eval.md section 3.1 (decided 2026-06-12): all-zero ISD in both arms drops signals_version
    # from the stratum key, so sv1-vs-sv2 arms pool and a real verdict comes out
    rng = np.random.default_rng(22)
    before = self._arm(rng, 200, 0.30, sv=1, isd=0.0)
    after = self._arm(rng, 200, 0.40, sv=2, isd=0.0)
    report = ps.compare_onroad(before, after)
    assert report["cross_era_rule"]["engaged"] is True
    assert not all(row["skipped"] for row in report["strata"])
    assert math.isfinite(report["pooled_median_delta"])
    assert report["verdict"] == "regressed"

  def test_cross_era_small_arm_refused_for_power_not_nan(self):
    # the 2026-06-12 shape: cross-era, all-zero ISD, after-arm far below the 150/arm floor --
    # the verdict must be a power refusal WITH real pooled numbers, not the all-strata-skipped NaN
    rng = np.random.default_rng(23)
    before = self._arm(rng, 160, 0.30, sv=1, isd=0.0)
    after = self._arm(rng, 29, 0.40, sv=2, isd=0.0)
    report = ps.compare_onroad(before, after)
    assert report["verdict"] == "refused_insufficient_power"
    assert report["cross_era_rule"]["engaged"] is True
    assert math.isfinite(report["pooled_median_delta"])
    assert all(math.isfinite(c) for c in report["pooled_ci"])

  def test_single_nonzero_isd_event_keeps_strict_behavior(self):
    rng = np.random.default_rng(24)
    before = self._arm(rng, 200, 0.30, sv=1, isd=0.0)
    after = self._arm(rng, 200, 0.40, sv=2, isd=0.0)
    after[0]["entry"]["isd_m"] = 1.2  # one nonzero-ISD event in one arm fails the precondition
    report = ps.compare_onroad(before, after)
    assert report["cross_era_rule"]["engaged"] is False
    assert all(row["skipped"] for row in report["strata"])
    assert math.isnan(report["pooled_median_delta"])

  def test_missing_isd_keeps_strict_behavior(self):
    rng = np.random.default_rng(25)
    before = self._arm(rng, 200, 0.30, sv=1)  # no entry.isd_m recorded -> comparability unproven
    after = self._arm(rng, 200, 0.40, sv=2, isd=0.0)
    report = ps.compare_onroad(before, after)
    assert report["cross_era_rule"]["engaged"] is False
    assert all(row["skipped"] for row in report["strata"])
    assert math.isnan(report["pooled_median_delta"])


class TestCli:
  def test_cli_refusal_exit_code_2(self, tmp_path: Path, capsys):
    rng = np.random.default_rng(19)
    events_a, events_b = _paired_events(rng.normal(0.05, 0.02, 20))
    a_path, b_path = tmp_path / "a.json", tmp_path / "b.json"
    a_path.write_text(json.dumps(events_a))
    b_path.write_text(json.dumps(events_b))
    out_path = tmp_path / "report.json"
    rc = ps.main(["--a-json", str(a_path), "--b-json", str(b_path), "--output-json", str(out_path)])
    assert rc == 2
    captured = capsys.readouterr()
    assert "required_n_for_observed_delta" in captured.out
    assert "VERDICT REFUSED" in captured.err
    payload = json.loads(out_path.read_text())
    assert payload["verdict"] == "refused_insufficient_power"

  def test_cli_regression_exit_code_1(self, tmp_path: Path):
    rng = np.random.default_rng(20)
    events_a, events_b = _paired_events(rng.normal(0.08, 0.02, 250))
    a_path, b_path = tmp_path / "a.json", tmp_path / "b.json"
    a_path.write_text(json.dumps(events_a))
    b_path.write_text(json.dumps(events_b))
    assert ps.main(["--a-json", str(a_path), "--b-json", str(b_path)]) == 1

  def test_cli_pass_exit_code_0(self, tmp_path: Path):
    rng = np.random.default_rng(21)
    events_a, events_b = _paired_events(rng.normal(0.0, 0.02, 250))
    a_path, b_path = tmp_path / "a.json", tmp_path / "b.json"
    a_path.write_text(json.dumps(events_a))
    b_path.write_text(json.dumps(events_b))
    assert ps.main(["--a-json", str(a_path), "--b-json", str(b_path)]) == 0


class TestStratumSpeedFromEntry:
  """Regression: store-shaped records carry approach speed ONLY under entry.v_approach
  (eval.md section 1). Before the 2026-06-12 fix, stratum_of read v=0.0 for every store
  record and on-road stratification degenerated to lead/no-lead x signals_version."""

  def _store_event(self, v_approach, lead=False, sv=2):
    # No top-level v_approach and no metrics dict: the real events.jsonl shape.
    return {
      "entry": {"v_approach": v_approach, "lead_entry_gap_m": 5.0 if lead else None, "isd_m": 0.0},
      "signals_version": sv,
    }

  def test_store_shaped_event_uses_entry_speed(self):
    assert ps.stratum_of(self._store_event(4.0, lead=True)) == "v>2|lead|sv2"
    assert ps.stratum_of(self._store_event(1.5)) == "v1-2|no_lead|sv2"
    assert ps.stratum_of(self._store_event(0.5)) == "v<1|no_lead|sv2"

  def test_top_level_speed_still_takes_precedence(self):
    event = self._store_event(4.0)
    event["v_approach"] = 0.4
    assert ps.stratum_of(event).startswith("v<1|")

  def test_entry_speed_composes_with_cross_era_rule(self):
    # The 3.1 cross-era kwarg must keep working with entry-only speed: sv token dropped, bin kept.
    assert ps.stratum_of(self._store_event(4.0, lead=True), ignore_signals_version=True) == "v>2|lead"
