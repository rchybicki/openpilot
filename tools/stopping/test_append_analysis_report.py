from __future__ import annotations

import json
from pathlib import Path
import sys

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
  sys.path.insert(0, str(REPO_ROOT))

from openpilot.tools.stopping.append_analysis_report import build_block  # noqa: E402


def _write_shadow_summary(summary_json: Path, route_summary: dict[str, object]) -> None:
  shadow_summary_json = summary_json.with_name("shadow_summary.json")
  shadow_summary_json.write_text(json.dumps({"route_summary": route_summary}))


def test_build_block_promotes_controller_owned_shadow_coverage(tmp_path: Path) -> None:
  summary_json = tmp_path / "analysis" / "summary.json"
  summary_json.parent.mkdir(parents=True)
  _write_shadow_summary(summary_json, {
    "event_count": 5,
    "eligible_event_count": 2,
    "ineligible_event_count": 3,
    "ineligible_reason_counts": {"manual_brake": 2, "openpilot_not_enabled": 1},
    "events_with_shadow": 2,
    "eligible_events_with_shadow": 2,
    "eligible_harsh_events": 1,
    "eligible_harsh_events_with_shadow": 1,
    "eligible_harsh_events_missing_shadow": 0,
    "unsafe_shadow_candidate_events": 0,
    "actionable_soften_candidates": 1,
    "mixed_shadow_signal_events": 0,
    "missed_harsh_events": 0,
    "verdict": "valuable_promote_narrow_candidate",
  })

  block = build_block(
    {
      "generated_utc": "2026-06-07T12:00:00+00:00",
      "host": "comma",
      "route": "000016eb--4b1dc029c9",
      "qlog_count": 3,
      "events": [],
    },
    summary_json,
    title=None,
    notes=[],
  )

  assert "- Shadow verdict: `valuable_promote_narrow_candidate`" in block
  assert "- Shadow eligible events covered: `2/2`" in block
  assert "- Shadow eligible harsh events covered: `1/1` (`0` missing)" in block
  assert "- Shadow ineligible events: `3/5` (manual_brake:2, openpilot_not_enabled:1)" in block
  assert "- Shadow safety/value events: `unsafe=0`, `actionable=1`, `mixed=0`, `missed=0`" in block
  assert "- Shadow all-event coverage: `2/5`" in block


def test_build_block_keeps_legacy_shadow_summary_fields(tmp_path: Path) -> None:
  summary_json = tmp_path / "analysis" / "summary.json"
  summary_json.parent.mkdir(parents=True)
  _write_shadow_summary(summary_json, {
    "event_count": 4,
    "events_with_shadow": 3,
    "actual_harsh_events": 2,
    "harsh_events_with_shadow": 1,
    "unsafe_shadow_candidate_events": 1,
    "verdict": "valuable_but_needs_stronger_guards",
  })

  block = build_block(
    {
      "generated_utc": "2026-06-07T12:00:00+00:00",
      "host": "comma",
      "route": "00001429--32d16f6f48",
      "qlog_count": 3,
      "events": [],
    },
    summary_json,
    title=None,
    notes=[],
  )

  assert "- Shadow verdict: `valuable_but_needs_stronger_guards`" in block
  assert "- Shadow events covered: `3/4`" in block
  assert "- Shadow harsh events covered: `1/2`" in block
  assert "- Shadow unsafe-candidate events: `1`" in block
