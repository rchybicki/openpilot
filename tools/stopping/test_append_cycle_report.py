from __future__ import annotations

import json
from pathlib import Path
import sys

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
  sys.path.insert(0, str(REPO_ROOT))

from openpilot.tools.stopping.append_cycle_report import summarize_benchmark  # noqa: E402


def test_summarize_benchmark_reports_profile_selector_variant(tmp_path: Path) -> None:
  benchmark_json = tmp_path / "benchmark.json"
  benchmark_json.write_text(json.dumps({
    "events_considered": 8,
    "current": {
      "harsh_events": 3,
      "harsh_rate": 0.375,
      "leapfrog_events": 1,
      "leapfrog_rate": 0.125,
      "avg_event_score": 0.42,
      "perfect_events": 1,
      "good_or_better_events": 2,
    },
    "profile_selector": {
      "events": 6,
      "harsh_events": 1,
      "harsh_rate": 1.0 / 6.0,
      "leapfrog_events": 0,
      "leapfrog_rate": 0.0,
      "avg_event_score": 0.31,
      "perfect_events": 2,
      "good_or_better_events": 4,
    },
    "comparison": {
      "profile_selector_improved_events": 4,
      "profile_selector_worsened_events": 1,
    },
  }))

  lines = summarize_benchmark(benchmark_json)

  assert "- Variant benchmark events: 8" in lines
  assert "- Variant `profile_selector`: harsh=1/6 rate=0.167 leapfrog=0/6 rate=0.000 avg_score=0.310 perfect=2/6 good_or_better=4/6" in lines
  assert "- Profile selector comparison: improved=4 worsened=1" in lines
