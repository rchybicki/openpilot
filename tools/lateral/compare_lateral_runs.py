#!/usr/bin/env python3
"""Compare two lateral tuning summary.json artifacts."""

from __future__ import annotations

import argparse
import json
from datetime import UTC, datetime
from pathlib import Path
from typing import Any

SLICE_PATHS = {
  "All active": ("tracking", "active_all", "controller"),
  "Turning": ("tracking", "turning", "controller"),
  "Strong turning": ("tracking", "strong_turning", "controller"),
}

SLICE_METRICS = [
  ("Samples", "count", "count", None),
  ("Active seconds", "active_seconds_est", "float", None),
  ("MAE", "mae", "float", "lower"),
  ("RMSE", "rmse", "float", "lower"),
  ("Median ratio", "median_ratio", "float", "target_1"),
  ("Under < 0.8", "under_ratio_below_0p8", "float", "lower"),
  ("Sat ratio", "saturation_ratio", "float", "lower"),
  ("Steer-limited ratio", "steer_limited_ratio", "float", "lower"),
]

SPEED_BIN_METRICS = [
  ("Samples", "count", "count", None),
  ("MAE", "mae", "float", "lower"),
  ("Median ratio", "median_ratio", "float", "target_1"),
  ("Under < 0.8", "under_ratio_below_0p8", "float", "lower"),
  ("Sat ratio", "saturation_ratio", "float", "lower"),
  ("Steer-limited ratio", "steer_limited_ratio", "float", "lower"),
]

PARAM_SPECS = [
  ("LatAccelFactor", "lat_accel_factor"),
  ("Friction", "friction"),
  ("SteerRatio", "steer_ratio"),
  ("LateralDelay", "lateral_delay"),
]


def utc_now_iso() -> str:
  return datetime.now(UTC).replace(microsecond=0).isoformat()


def parse_args() -> argparse.Namespace:
  parser = argparse.ArgumentParser(description="Compare two lateral tuning summary.json files")
  parser.add_argument("--before", required=True, help="Baseline summary.json")
  parser.add_argument("--after", required=True, help="Candidate summary.json")
  parser.add_argument("--output", default=None, help="Optional output markdown path")
  parser.add_argument("--title", default="Lateral Tuning Comparison", help="Markdown title")
  parser.add_argument("--top-issues", type=int, default=5, help="How many after-issue windows to include")
  return parser.parse_args()


def load_summary(path: Path) -> dict[str, Any]:
  data = json.loads(path.read_text())
  if not isinstance(data, dict):
    raise ValueError(f"Expected JSON object in {path}")
  return data


def get_nested(data: dict[str, Any], *keys: str) -> Any:
  current: Any = data
  for key in keys:
    if not isinstance(current, dict):
      return None
    current = current.get(key)
  return current


def fmt(value: float | int | None, digits: int = 3) -> str:
  if value is None:
    return "n/a"
  if isinstance(value, int):
    return str(value)
  return f"{value:.{digits}f}"


def delta(after: float | int | None, before: float | int | None) -> float | int | None:
  if after is None or before is None:
    return None
  return after - before


def delta_str(after: float | int | None, before: float | int | None, digits: int = 3) -> str:
  d = delta(after, before)
  if d is None:
    return "n/a"
  if isinstance(d, int) and isinstance(after, int) and isinstance(before, int):
    return f"{d:+d}"
  return f"{d:+.{digits}f}"


def target_error(value: float | None, target: float) -> float | None:
  if value is None:
    return None
  return abs(value - target)


def judgment(before: float | int | None, after: float | int | None, goal: str | None) -> str:
  if before is None or after is None or goal is None:
    return ""
  if goal == "lower":
    if after < before:
      return "better"
    if after > before:
      return "worse"
    return "same"
  if goal == "target_1":
    before_err = target_error(float(before), 1.0)
    after_err = target_error(float(after), 1.0)
    if before_err is None or after_err is None:
      return ""
    if after_err < before_err:
      return "better"
    if after_err > before_err:
      return "worse"
    return "same"
  return ""


def speed_bins_by_label(summary: dict[str, Any]) -> dict[str, dict[str, Any]]:
  items = get_nested(summary, "tracking", "speed_bins_turning")
  if not isinstance(items, list):
    return {}
  result: dict[str, dict[str, Any]] = {}
  for item in items:
    if not isinstance(item, dict):
      continue
    label = item.get("label")
    metrics = item.get("metrics")
    if isinstance(label, str) and isinstance(metrics, dict):
      result[label] = metrics
  return result


def build_metric_table(title: str, before_metrics: dict[str, Any] | None, after_metrics: dict[str, Any] | None,
                       metric_specs: list[tuple[str, str, str, str | None]]) -> list[str]:
  lines = [f"### {title}", "", "| Metric | Before | After | Delta | Judgment |", "| --- | ---: | ---: | ---: | --- |"]
  before_metrics = before_metrics or {}
  after_metrics = after_metrics or {}

  for label, key, kind, goal in metric_specs:
    before = before_metrics.get(key) if isinstance(before_metrics, dict) else None
    after = after_metrics.get(key) if isinstance(after_metrics, dict) else None
    digits = 0 if kind == "count" else 3
    lines.append(
      f"| {label} | {fmt(before, digits)} | {fmt(after, digits)} | {delta_str(after, before, digits)} | {judgment(before, after, goal)} |"
    )

  lines.append("")
  return lines


def build_param_table(before: dict[str, Any], after: dict[str, Any]) -> list[str]:
  lines = [
    "## Learned Vs Static Alignment",
    "",
    "| Parameter | Before live | After live | Before live/stock | After live/stock | Judgment |",
    "| --- | ---: | ---: | ---: | ---: | --- |",
  ]
  before_params = before.get("param_comparison", {}) if isinstance(before.get("param_comparison"), dict) else {}
  after_params = after.get("param_comparison", {}) if isinstance(after.get("param_comparison"), dict) else {}

  for label, key in PARAM_SPECS:
    before_entry = before_params.get(key, {}) if isinstance(before_params, dict) else {}
    after_entry = after_params.get(key, {}) if isinstance(after_params, dict) else {}
    before_live = get_nested(before_entry, "live", "median")
    after_live = get_nested(after_entry, "live", "median")
    before_ratio = before_entry.get("ratio_live_over_stock") if isinstance(before_entry, dict) else None
    after_ratio = after_entry.get("ratio_live_over_stock") if isinstance(after_entry, dict) else None
    lines.append(
      f"| {label} | {fmt(before_live)} | {fmt(after_live)} | {fmt(before_ratio)} | {fmt(after_ratio)} | {judgment(before_ratio, after_ratio, 'target_1')} |"
    )

  lines.append("")
  return lines


def build_after_issue_section(after: dict[str, Any], top_issues: int) -> list[str]:
  issues = get_nested(after, "tracking", "issue_windows")
  if not isinstance(issues, list) or not issues:
    return ["## Top After Issue Windows", "", "- none", ""]

  lines = ["## Top After Issue Windows", ""]
  for issue in issues[:top_issues]:
    if not isinstance(issue, dict):
      continue
    lines.append(
      "- "
      + f"`{issue.get('route', 'unknown')}` seg `{issue.get('segment', '?')}` "
      + f"{fmt(issue.get('start_segment_time_s'))}-{fmt(issue.get('end_segment_time_s'))} s, "
      + f"worst abs error `{fmt(issue.get('worst_abs_error'))}`, "
      + f"worst ratio `{fmt(issue.get('worst_ratio'))}`, "
      + f"steer-limited `{issue.get('steer_limited_any')}`, saturated `{issue.get('saturated_any')}`"
    )
  lines.append("")
  return lines


def build_markdown(title: str, before_path: Path, after_path: Path, before: dict[str, Any], after: dict[str, Any], top_issues: int) -> str:
  lines: list[str] = []
  lines.append(f"# {title}")
  lines.append("")
  lines.append(f"- Generated (UTC): {utc_now_iso()}")
  lines.append(f"- Before: `{before_path}`")
  lines.append(f"- After: `{after_path}`")
  lines.append(f"- Car fingerprint: `{after.get('car_fingerprint') or before.get('car_fingerprint') or 'unknown'}`")
  lines.append(
    f"- Routes analyzed: before `{len(before.get('routes_analyzed', []))}` | after `{len(after.get('routes_analyzed', []))}`"
  )
  lines.append(
    f"- Qlogs analyzed: before `{before.get('qlog_count', 'n/a')}` | after `{after.get('qlog_count', 'n/a')}`"
  )
  lines.append("")

  for slice_label, slice_path in SLICE_PATHS.items():
    lines.extend(build_metric_table(slice_label, get_nested(before, *slice_path), get_nested(after, *slice_path), SLICE_METRICS))

  lines.append("## Turning Speed Bins")
  lines.append("")
  before_bins = speed_bins_by_label(before)
  after_bins = speed_bins_by_label(after)
  for label in ["2.5-5", "5-10", "10-15", "15-25", "25+"]:
    lines.extend(build_metric_table(label, before_bins.get(label), after_bins.get(label), SPEED_BIN_METRICS))

  lines.extend(build_param_table(before, after))
  lines.extend(build_after_issue_section(after, top_issues))

  lines.append("## Interpretation Hints")
  lines.append("")
  lines.append("- Lower `MAE`, `RMSE`, `under < 0.8`, `sat ratio`, and `steer-limited ratio` are generally better.")
  lines.append("- `median ratio` closer to `1.0` is better.")
  lines.append("- `live/stock` closer to `1.0` means the static seed is better aligned with the learned state.")
  lines.append("")
  return "\n".join(lines)


def main() -> int:
  args = parse_args()
  before_path = Path(args.before).expanduser()
  after_path = Path(args.after).expanduser()
  if not before_path.exists():
    raise FileNotFoundError(f"Before summary not found: {before_path}")
  if not after_path.exists():
    raise FileNotFoundError(f"After summary not found: {after_path}")

  before = load_summary(before_path)
  after = load_summary(after_path)
  markdown = build_markdown(args.title, before_path, after_path, before, after, args.top_issues)

  if args.output:
    output_path = Path(args.output).expanduser()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(markdown + "\n")
    print(f"[compare] output={output_path}")
  else:
    print(markdown)

  return 0


if __name__ == "__main__":
  raise SystemExit(main())
