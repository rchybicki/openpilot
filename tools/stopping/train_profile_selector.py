#!/usr/bin/env python3
"""Train a small stop-tail profile selector from horizon-teacher benchmark output."""

from __future__ import annotations

import argparse
import json
import sys
from collections import Counter
from dataclasses import dataclass, field
from datetime import UTC, datetime
from math import sqrt
from pathlib import Path
from typing import Any

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
  sys.path.insert(0, str(REPO_ROOT))

from openpilot.selfdrive.controls.lib.stopping_profile_selector import (  # pylint: disable=wrong-import-position
  DEFAULT_FEATURE_SCALES,
  PROFILE_LABELS,
  SELECTOR_FEATURE_NAMES,
  PrototypeStoppingProfileSelector,
  StoppingProfileFeatures,
  profile_label_from_horizon_teacher,
)


@dataclass(frozen=True)
class SelectorTrainingRow:
  source: str
  route: str
  event_id: str
  label: str
  features: dict[str, float]
  residual_template_mps2: list[float] = field(default_factory=list)


def parse_args() -> argparse.Namespace:
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument("--benchmark-json", action="append", required=True, help="Path to benchmark_controller_variants.py output JSON")
  parser.add_argument("--output", required=True, help="Output selector model JSON")
  parser.add_argument("--min-improvement", type=float, default=0.01, help="Minimum horizon score improvement required for non-no_change labels")
  parser.add_argument("--min-profile-count", type=int, default=1, help="Drop learned profiles with fewer than this many rows")
  parser.add_argument("--selector-kind", choices=["prototype", "knn"], default="prototype")
  parser.add_argument("--knn-k", type=int, default=5, help="Neighbor count for --selector-kind knn")
  parser.add_argument("--include-route", action="append", default=[], help="Only train on this route; may be repeated")
  parser.add_argument("--exclude-route", action="append", default=[], help="Exclude this route from training; may be repeated")
  return parser.parse_args()


def _load_json(path: Path) -> dict[str, Any]:
  with path.open() as f:
    return json.load(f)


def load_training_rows(
  paths: list[Path],
  min_improvement: float,
  *,
  include_routes: set[str] | None = None,
  exclude_routes: set[str] | None = None,
) -> tuple[list[SelectorTrainingRow], dict[str, int]]:
  rows: list[SelectorTrainingRow] = []
  skipped = Counter()
  include_routes = include_routes or set()
  exclude_routes = exclude_routes or set()
  for path in paths:
    payload = _load_json(path)
    event_rows = payload.get("event_rows", [])
    if not isinstance(event_rows, list):
      skipped["missing_event_rows"] += 1
      continue
    for row in event_rows:
      if not isinstance(row, dict):
        skipped["non_dict_event_row"] += 1
        continue
      feature_payload = row.get("selector_features", {})
      if not isinstance(feature_payload, dict) or not feature_payload:
        skipped["missing_selector_features"] += 1
        continue
      teacher = row.get("horizon_teacher", {})
      if not isinstance(teacher, dict):
        skipped["missing_horizon_teacher"] += 1
        continue
      route = str(row.get("route", ""))
      if include_routes and route not in include_routes:
        skipped["route_not_in_include"] += 1
        continue
      if route in exclude_routes:
        skipped["route_excluded"] += 1
        continue
      label = profile_label_from_horizon_teacher(teacher, min_improvement=min_improvement)
      features = StoppingProfileFeatures.from_mapping(feature_payload).as_dict()
      residual_target = row.get("selector_residual_target", {})
      residual_template: list[float] = []
      if isinstance(residual_target, dict) and isinstance(residual_target.get("residual_template_mps2"), list):
        residual_template = [
          max(-0.20, min(0.20, float(value)))
          for value in residual_target["residual_template_mps2"]
        ]
      rows.append(SelectorTrainingRow(
        source=str(path),
        route=route,
        event_id=str(row.get("event_id", "")),
        label=label,
        features=features,
        residual_template_mps2=residual_template,
      ))
  return rows, dict(sorted(skipped.items()))


def _feature_mean(rows: list[SelectorTrainingRow], feature_name: str) -> float:
  return sum(row.features.get(feature_name, 0.0) for row in rows) / max(len(rows), 1)


def _feature_scale(rows: list[SelectorTrainingRow], feature_name: str) -> float:
  if len(rows) < 2:
    return DEFAULT_FEATURE_SCALES.get(feature_name, 1.0)
  mean = _feature_mean(rows, feature_name)
  variance = sum((row.features.get(feature_name, 0.0) - mean) ** 2 for row in rows) / max(len(rows) - 1, 1)
  learned_scale = sqrt(max(variance, 0.0))
  floor = DEFAULT_FEATURE_SCALES.get(feature_name, 1.0) * 0.25
  return max(learned_scale, floor, 1e-3)


def _residual_template_mean(rows: list[SelectorTrainingRow]) -> list[float]:
  templates = [row.residual_template_mps2 for row in rows if row.residual_template_mps2]
  if not templates:
    return [0.0, 0.0, 0.0]
  template_len = max(len(template) for template in templates)
  result: list[float] = []
  for idx in range(template_len):
    values = [template[min(idx, len(template) - 1)] for template in templates]
    mean = sum(values) / max(len(values), 1)
    result.append(max(-0.20, min(0.20, mean)))
  return result


def build_selector_model(
  rows: list[SelectorTrainingRow],
  *,
  min_profile_count: int,
  selector_kind: str = "prototype",
  knn_k: int = 5,
  source_paths: list[Path],
) -> dict[str, Any]:
  label_counts = Counter(row.label for row in rows)
  feature_scales = {
    name: _feature_scale(rows, name)
    for name in SELECTOR_FEATURE_NAMES
  }

  profiles: list[dict[str, Any]] = []
  for label in PROFILE_LABELS:
    label_rows = [row for row in rows if row.label == label]
    if len(label_rows) < min_profile_count:
      continue
    prototype = {
      name: _feature_mean(label_rows, name)
      for name in SELECTOR_FEATURE_NAMES
    }
    residual_template = [0.0, 0.0, 0.0] if label == "no_change" else _residual_template_mean(label_rows)
    profiles.append({
      "profile": label,
      "count": len(label_rows),
      "prototype": prototype,
      "residual_template_mps2": residual_template,
      "exemplars": [
        {
          "route": row.route,
          "event_id": row.event_id,
          "features": row.features,
          "residual_template_mps2": row.residual_template_mps2,
        }
        for row in label_rows
        if row.residual_template_mps2
      ],
    })

  return {
    "model_type": "nearest_profile_knn" if selector_kind == "knn" else "nearest_profile_prototype",
    "selector_kind": selector_kind,
    "knn_k": max(1, int(knn_k)),
    "generated_utc": datetime.now(UTC).isoformat(),
    "source_benchmarks": [str(path) for path in source_paths],
    "feature_names": list(SELECTOR_FEATURE_NAMES),
    "feature_scales": feature_scales,
    "profiles": profiles,
    "training": {
      "rows": len(rows),
      "label_counts": dict(sorted(label_counts.items())),
      "min_profile_count": int(min_profile_count),
      "selector_kind": selector_kind,
      "knn_k": max(1, int(knn_k)),
    },
  }


def evaluate_selector_model(rows: list[SelectorTrainingRow], model_payload: dict[str, Any]) -> dict[str, Any]:
  if not rows:
    return {"rows": 0, "accuracy": None, "confusion": {}}
  selector = PrototypeStoppingProfileSelector.from_json(model_payload)
  correct = 0
  confusion: dict[str, Counter[str]] = {}
  for row in rows:
    decision = selector.select(StoppingProfileFeatures.from_mapping(row.features))
    if decision.profile == row.label:
      correct += 1
    confusion.setdefault(row.label, Counter())[decision.profile] += 1
  return {
    "rows": len(rows),
    "accuracy": correct / len(rows),
    "confusion": {
      label: dict(sorted(counter.items()))
      for label, counter in sorted(confusion.items())
    },
  }


def main() -> int:
  args = parse_args()
  paths = [Path(item).expanduser() for item in args.benchmark_json]
  include_routes = {str(route) for route in args.include_route}
  exclude_routes = {str(route) for route in args.exclude_route}
  rows, skipped = load_training_rows(
    paths,
    min_improvement=args.min_improvement,
    include_routes=include_routes,
    exclude_routes=exclude_routes,
  )
  model_payload = build_selector_model(
    rows,
    min_profile_count=args.min_profile_count,
    selector_kind=args.selector_kind,
    knn_k=args.knn_k,
    source_paths=paths,
  )
  model_payload["training"]["skipped"] = skipped
  model_payload["training"]["route_filters"] = {
    "include_routes": sorted(include_routes),
    "exclude_routes": sorted(exclude_routes),
  }
  model_payload["training"]["resubstitution_eval"] = evaluate_selector_model(rows, model_payload)

  output_path = Path(args.output).expanduser()
  output_path.parent.mkdir(parents=True, exist_ok=True)
  with output_path.open("w") as f:
    json.dump(model_payload, f, indent=2, sort_keys=True)
    f.write("\n")

  print(f"[selector] rows={len(rows)} profiles={len(model_payload['profiles'])} output={output_path}")
  if skipped:
    print(f"[selector] skipped={skipped}")
  return 0


if __name__ == "__main__":
  raise SystemExit(main())
