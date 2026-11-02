"""stop_index conditioning semantics (flip-gate tooling): legacy 6-tuples unconditioned, new 7-tuples
use only stopped-lead frames, and a new trace with no stopped-lead samples is EXCLUDED."""
import sys

sys.path.insert(0, "/Users/radoslawchybicki/Repos/openpilot-rch/tools/stopping/review")
from stop_index import gov_approach_stats


def test_legacy_six_tuples_stay_unconditioned():
  tr = [(0.0, 2.0, 8.0, -0.8, -1.0, -0.2), (0.25, 1.8, 7.5, -0.8, -0.6, -0.2)]
  s = gov_approach_stats(tr)
  assert s["gov_appr_n"] == 2 and s["gov_appr_conditioned"] is False
  assert s["gov_appr_deeper"] == 0.5 and s["gov_appr_shallower"] == 0.5


def test_new_traces_use_only_stopped_lead_frames():
  tr = [(0.0, 2.0, 8.0, -0.8, -1.0, -0.2, 1.5),    # moving lead: ignored
        (0.25, 1.8, 7.5, -0.8, -1.2, -0.2, 0.1),   # stopped lead: used (deeper)
        (0.5, 1.6, 7.0, -0.8, -1.3, -0.2, 0.0)]    # stopped lead: used (deeper)
  s = gov_approach_stats(tr)
  assert s["gov_appr_n"] == 2 and s["gov_appr_conditioned"] is True
  assert s["gov_appr_deeper"] == 1.0 and s["gov_appr_shallower"] == 0.0


def test_new_trace_with_no_stopped_lead_samples_is_excluded():
  tr = [(0.0, 2.0, 8.0, -0.8, -0.2, -0.2, 1.5), (0.25, 1.8, 7.5, -0.8, -0.2, -0.2, 0.9)]
  s = gov_approach_stats(tr)
  assert "gov_appr_n" not in s, "moving-lead-only trace leaked into the aggregates"
  assert s["gov_appr_conditioned"] is True and s["gov_implausible"] is False


def test_implausible_flag_survives_exclusion():
  tr = [(0.0, 4.0, 0.8, -0.8, -2.5, -20.0, 1.5)]
  s = gov_approach_stats(tr)
  assert s["gov_implausible"] is True and "gov_appr_n" not in s
