#!/usr/bin/env python3
"""Generate per-route stopping graph packs for top problematic routes."""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
from collections import defaultdict
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
  sys.path.insert(0, str(REPO_ROOT))

from tools.stopping.diagnose_stop_failures import FOCUS_CHOICES, load_events

DEFAULT_REVIEW_ROOT = Path.home() / ".comma" / "stopping_behavior" / "analysis" / "review_pack"


def parse_args() -> argparse.Namespace:
  parser = argparse.ArgumentParser(description="Build per-route graph packs from a corpus summary")
  parser.add_argument("--summary-json", required=True, help="Path to corpus summary.json")
  parser.add_argument("--top-routes", type=int, default=5, help="Number of top routes to process")
  parser.add_argument("--focus-source", default="engaged", choices=FOCUS_CHOICES,
                      help="Event source focus used for route ranking")
  parser.add_argument("--host", default=None, help="Override host alias for analysis runs")
  parser.add_argument("--event-mode", default=None, help="Override event mode for analysis runs")
  parser.add_argument("--min-entry-speed", type=float, default=None, help="Override min entry speed for analysis runs")
  parser.add_argument("--require-enabled-speed-events", action="store_true",
                      help="Force require-enabled speed events in analysis runs")
  parser.add_argument("--analysis-root", default=str(DEFAULT_REVIEW_ROOT),
                      help=f"Output root for route graph packs. Default: {DEFAULT_REVIEW_ROOT}")
  parser.add_argument("--output-dir", default=None, help="Explicit output directory")
  parser.add_argument("--dry-run", action="store_true", help="Print planned commands without executing them")

  # Mirror diagnosis thresholds for consistent route ranking.
  parser.add_argument("--long-duration-s", type=float, default=8.0)
  parser.add_argument("--moving-duration-s", type=float, default=12.0)
  parser.add_argument("--moving-distance-m", type=float, default=20.0)
  parser.add_argument("--queue-speed-threshold-mps", type=float, default=1.2)
  parser.add_argument("--queue-entry-threshold-mps", type=float, default=2.0)
  parser.add_argument("--creep-threshold-mps", type=float, default=0.15)
  parser.add_argument("--positive-cmd-threshold-mps2", type=float, default=0.02)
  parser.add_argument("--hold-rebound-threshold-mps", type=float, default=0.15)
  parser.add_argument("--unexpected-accel-threshold-mps2", type=float, default=0.20)
  parser.add_argument("--stable-cmd-accel-delta-threshold-mps2", type=float, default=0.35)
  parser.add_argument("--should-stop-rebound-threshold-mps", type=float, default=0.08)
  parser.add_argument("--should-stop-unexpected-accel-threshold-mps2", type=float, default=0.10)
  parser.add_argument("--should-stop-relief-spike-threshold-mps2", type=float, default=0.18)
  parser.add_argument("--rollout-threshold-m", type=float, default=2.0)
  parser.add_argument("--min-lead-hold-distance-m", type=float, default=2.0)
  parser.add_argument("--max-lead-hold-distance-m", type=float, default=4.0)
  parser.add_argument("--end-stop-jerk-threshold-mps3", type=float, default=2.5)
  parser.add_argument("--end-stop-accel-step-threshold-mps2", type=float, default=0.5)
  parser.add_argument("--cmd-jerk-threshold-mps3", type=float, default=3.0)
  parser.add_argument("--cmd-step-threshold-mps2", type=float, default=0.5)
  parser.add_argument("--wheel-stop-decel-threshold-mps2", type=float, default=-4.0)
  parser.add_argument("--wheel-drop-threshold-mps", type=float, default=0.35)
  parser.add_argument("--hard-decel-threshold-mps2", type=float, default=-1.5)
  parser.add_argument("--hard-cmd-threshold-mps2", type=float, default=-1.2)
  parser.add_argument("--late-signal-gap-mps", type=float, default=6.0)
  parser.add_argument("--late-signal-entry-max-mps", type=float, default=2.0)
  return parser.parse_args()


def build_output_dir(base_root: Path, override: str | None) -> Path:
  if override:
    return Path(override).expanduser()
  stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
  return base_root.expanduser() / stamp


def main() -> int:
  args = parse_args()
  summary_path = Path(args.summary_json).expanduser()
  summary: dict[str, Any] = json.loads(summary_path.read_text())

  rows = load_events(summary, args)
  if not rows:
    print("[review-pack] no events matched selected source filter")
    return 0

  route_scores: defaultdict[str, float] = defaultdict(float)
  route_event_counts: defaultdict[str, int] = defaultdict(int)
  for row in rows:
    route_scores[row.route] += row.severity_score
    route_event_counts[row.route] += 1

  top_routes = sorted(route_scores.items(), key=lambda item: item[1], reverse=True)[:args.top_routes]
  if not top_routes:
    print("[review-pack] no top routes selected")
    return 0

  host = args.host or str(summary.get("host", "commawifi"))
  event_mode = args.event_mode or str(summary.get("event_mode", "engaged_signal"))
  min_entry_speed = args.min_entry_speed if args.min_entry_speed is not None else float(summary.get("min_entry_speed", 2.0))
  require_enabled_speed_events = args.require_enabled_speed_events or bool(summary.get("require_enabled_speed_events", False))

  output_dir = build_output_dir(Path(args.analysis_root), args.output_dir)
  output_dir.mkdir(parents=True, exist_ok=True)
  analyze_script = REPO_ROOT / "tools" / "stopping" / "analyze_stopping_behavior.py"

  manifest: dict[str, Any] = {
    "generated_utc": datetime.now(timezone.utc).replace(microsecond=0).isoformat(),
    "summary_json": str(summary_path),
    "host": host,
    "event_mode": event_mode,
    "min_entry_speed": min_entry_speed,
    "require_enabled_speed_events": require_enabled_speed_events,
    "focus_source": args.focus_source,
    "top_routes": [],
  }

  failures = 0
  for route, score_sum in top_routes:
    route_output = output_dir / route
    cmd = [
      sys.executable,
      str(analyze_script),
      "--host", host,
      "--route", route,
      "--event-mode", event_mode,
      "--min-entry-speed", f"{min_entry_speed}",
      "--output-dir", str(route_output),
    ]
    if require_enabled_speed_events:
      cmd.append("--require-enabled-speed-events")

    manifest["top_routes"].append({
      "route": route,
      "severity_score_sum": score_sum,
      "event_count": route_event_counts[route],
      "command": cmd,
      "output_dir": str(route_output),
    })

    print(f"[review-pack] route={route} score={score_sum:.2f} events={route_event_counts[route]}")
    print(f"[review-pack] cmd={' '.join(cmd)}")
    if args.dry_run:
      continue

    result = subprocess.run(cmd, cwd=str(REPO_ROOT), check=False)
    if result.returncode != 0:
      failures += 1
      print(f"[review-pack] error: analyze command failed for route={route} (exit={result.returncode})")

  manifest_path = output_dir / "manifest.json"
  manifest_path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n")
  print(f"[review-pack] manifest={manifest_path}")

  if failures > 0:
    print(f"[review-pack] completed with failures={failures}")
    return 1

  print(f"[review-pack] completed routes={len(top_routes)}")
  return 0


if __name__ == "__main__":
  raise SystemExit(main())
