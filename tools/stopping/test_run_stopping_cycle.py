from __future__ import annotations

import json
import os
from pathlib import Path
import sys

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
  sys.path.insert(0, str(REPO_ROOT))

from openpilot.tools.stopping.run_stopping_cycle import discover_recent_summaries, select_fit_summaries


def _write_summary(path: Path, *, route: str | None = None, event_mode: str = "speed_transition", event_sources: list[str] | None = None) -> None:
  path.parent.mkdir(parents=True, exist_ok=True)
  payload = {
    "route": route if route is not None else path.parent.parent.name,
    "event_mode": event_mode,
    "events": [{"event_source": source} for source in (event_sources or ["speed"])],
  }
  path.write_text(json.dumps(payload))


def test_select_fit_summaries_includes_recent_when_explicit_missing(tmp_path: Path) -> None:
  analysis_root = tmp_path / "analysis"
  host = "commawifi"

  current = analysis_root / host / "cycle_current" / "summary.json"
  older_a = analysis_root / host / "cycle_a" / "summary.json"
  older_b = analysis_root / host / "cycle_b" / "summary.json"
  _write_summary(current, route="route_current")
  _write_summary(older_a, route="route_a")
  _write_summary(older_b, route="route_b")

  # Ensure deterministic recency order: current > older_a > older_b
  base_ts = 1_700_000_000
  older_b.touch()
  older_a.touch()
  current.touch()
  older_b_stat = base_ts
  older_a_stat = base_ts + 10
  current_stat = base_ts + 20
  os.utime(older_b, (older_b_stat, older_b_stat))
  os.utime(older_a, (older_a_stat, older_a_stat))
  os.utime(current, (current_stat, current_stat))

  selected = select_fit_summaries(
    explicit_summaries=[],
    analysis_summary_json=current,
    analysis_root=analysis_root,
    host=host,
    event_source="all",
    recent_limit=2,
  )

  assert selected == [current, older_a]


def test_select_fit_summaries_prefers_explicit_inputs(tmp_path: Path) -> None:
  analysis_root = tmp_path / "analysis"
  host = "commawifi"
  explicit = tmp_path / "explicit.json"
  current = analysis_root / host / "cycle_current" / "summary.json"
  _write_summary(explicit)
  _write_summary(current)

  selected = select_fit_summaries(
    explicit_summaries=[explicit],
    analysis_summary_json=current,
    analysis_root=analysis_root,
    host=host,
    event_source="all",
    recent_limit=8,
  )

  assert selected == [explicit]


def test_discover_recent_summaries_all_prefers_hybrid_per_route(tmp_path: Path) -> None:
  analysis_root = tmp_path / "analysis"
  host = "commawifi"

  route_a_signal = analysis_root / host / "route_a" / "ts_signal" / "summary.json"
  route_a_speed = analysis_root / host / "route_a" / "ts_speed" / "summary.json"
  route_a_hybrid = analysis_root / host / "route_a" / "ts_hybrid" / "summary.json"
  route_b_speed = analysis_root / host / "route_b" / "ts_speed" / "summary.json"

  _write_summary(route_a_signal, route="route_a", event_mode="engaged_signal", event_sources=["signal"])
  _write_summary(route_a_speed, route="route_a", event_mode="speed_transition", event_sources=["speed"])
  _write_summary(route_a_hybrid, route="route_a", event_mode="hybrid", event_sources=["signal", "speed"])
  _write_summary(route_b_speed, route="route_b", event_mode="speed_transition", event_sources=["speed"])

  base_ts = 1_700_000_000
  os.utime(route_a_hybrid, (base_ts + 5, base_ts + 5))
  os.utime(route_a_speed, (base_ts + 15, base_ts + 15))
  os.utime(route_a_signal, (base_ts + 25, base_ts + 25))
  os.utime(route_b_speed, (base_ts + 20, base_ts + 20))

  discovered = discover_recent_summaries(analysis_root=analysis_root, host=host, event_source="all", limit=10)
  assert discovered == [route_b_speed, route_a_hybrid]
