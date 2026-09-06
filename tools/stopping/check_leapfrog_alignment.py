#!/usr/bin/env python3
"""Compare measured vs predicted leapfrog event sets.

Prediction source: sim_replay.py output (spec 7.8 -- the kept, ongoing model-truthfulness loop)
or the legacy check_harsh_stops_model.py output (accepted until its scheduled deletion). When
BOTH sides carry spec-7.1 stable keys (route, seg, hold_mono_ns) matching uses them; otherwise
the legacy positional (route, event_id) pairing applies.
"""

from __future__ import annotations

import argparse
import json
import sys
from dataclasses import dataclass
from datetime import UTC, datetime
from pathlib import Path
from typing import Any


@dataclass(frozen=True, order=True)
class EventRef:
  route: str
  event_id: int
  seg: int | None = None
  hold_mono_ns: int | None = None

  @property
  def has_stable_key(self) -> bool:
    return self.seg is not None and self.hold_mono_ns is not None

  def stable(self) -> EventRef:
    return EventRef(route=self.route, event_id=-1, seg=self.seg, hold_mono_ns=self.hold_mono_ns)

  def legacy(self) -> EventRef:
    return EventRef(route=self.route, event_id=self.event_id)

  def as_dict(self) -> dict[str, Any]:
    out: dict[str, Any] = {"route": self.route, "event_id": self.event_id}
    if self.has_stable_key:
      out["seg"] = self.seg
      out["hold_mono_ns"] = self.hold_mono_ns
    return out


def parse_args() -> argparse.Namespace:
  parser = argparse.ArgumentParser(description="Compare measured vs predicted leapfrog events")
  parser.add_argument("--measured-json", required=True, help="Output JSON from check_harsh_stops.py")
  parser.add_argument("--predicted-json", required=True,
                      help="Output JSON from sim_replay.py (or legacy check_harsh_stops_model.py)")
  parser.add_argument("--event-id-tolerance", type=int, default=0,
                      help="Optional event_id tolerance for near-match diagnostics")
  parser.add_argument("--min-overlap-recall", type=float, default=0.0,
                      help="Optional minimum required overlap recall [0..1] (0 disables)")
  parser.add_argument("--max-count-delta", type=int, default=-1,
                      help="Optional max abs(measured_count - predicted_count) (negative disables)")
  parser.add_argument("--output-json", default=None, help="Optional output path for alignment report JSON")
  return parser.parse_args()


def as_int(value: Any) -> int | None:
  try:
    return int(value)
  except (TypeError, ValueError):
    return None


def load_json(path: Path) -> dict[str, Any]:
  payload = json.loads(path.read_text())
  if not isinstance(payload, dict):
    raise ValueError(f"JSON root must be object: {path}")
  return payload


def parse_event_refs(rows: Any) -> list[EventRef]:
  refs: list[EventRef] = []
  if not isinstance(rows, list):
    return refs

  for row in rows:
    if not isinstance(row, dict):
      continue
    key = row.get("key") if isinstance(row.get("key"), dict) else {}
    route = str(row.get("route", key.get("route", "")) or "").strip()
    event_id = as_int(row.get("event_id"))
    seg = as_int(key.get("seg"))
    hold_mono_ns = as_int(key.get("hold_mono_ns"))
    if not route or (event_id is None and hold_mono_ns is None):
      continue
    refs.append(EventRef(route=route, event_id=event_id if event_id is not None else -1,
                         seg=seg, hold_mono_ns=hold_mono_ns))
  return refs


def align_ref_sets(measured: list[EventRef], predicted: list[EventRef]) -> tuple[list[EventRef], list[EventRef], str]:
  """Prefer spec-7.1 stable keys when BOTH sides carry them; legacy (route, event_id) otherwise."""
  if measured and predicted and all(r.has_stable_key for r in measured) and all(r.has_stable_key for r in predicted):
    return [r.stable() for r in measured], [r.stable() for r in predicted], "stable_key"
  return [r.legacy() for r in measured], [r.legacy() for r in predicted], "route_event_id"


def measured_leapfrog_refs(payload: dict[str, Any]) -> list[EventRef]:
  # Prefer explicit full-key output when present.
  refs = parse_event_refs(payload.get("leapfrog_event_keys"))
  if refs:
    return refs
  return parse_event_refs(payload.get("leapfrog_event_examples"))


def predicted_leapfrog_refs(payload: dict[str, Any]) -> list[EventRef]:
  rows = payload.get("event_rows")
  if isinstance(rows, list):
    refs: list[EventRef] = []
    for row in rows:
      if not isinstance(row, dict) or not bool(row.get("is_leapfrog")):
        continue
      route = str(row.get("route", "")).strip()
      event_id = as_int(row.get("event_id"))
      if not route or event_id is None:
        continue
      refs.append(EventRef(route=route, event_id=event_id))
    if refs:
      return refs

  refs = parse_event_refs(payload.get("leapfrog_event_keys"))
  if refs:
    return refs
  return parse_event_refs(payload.get("leapfrog_event_examples"))


def find_near_matches(
  measured_only: list[EventRef],
  predicted_only: list[EventRef],
  event_id_tolerance: int,
) -> list[dict[str, Any]]:
  if event_id_tolerance <= 0:
    return []
  if not measured_only or not predicted_only:
    return []

  candidates: list[tuple[int, EventRef, EventRef]] = []
  for measured in measured_only:
    for predicted in predicted_only:
      if measured.route != predicted.route:
        continue
      delta = abs(measured.event_id - predicted.event_id)
      if delta <= event_id_tolerance:
        candidates.append((delta, measured, predicted))

  if not candidates:
    return []

  candidates.sort(key=lambda item: (item[0], item[1].route, item[1].event_id, item[2].event_id))
  matched_measured: set[EventRef] = set()
  matched_predicted: set[EventRef] = set()
  pairs: list[dict[str, Any]] = []
  for delta, measured, predicted in candidates:
    if measured in matched_measured or predicted in matched_predicted:
      continue
    matched_measured.add(measured)
    matched_predicted.add(predicted)
    pairs.append({
      "measured": {"route": measured.route, "event_id": measured.event_id},
      "predicted": {"route": predicted.route, "event_id": predicted.event_id},
      "event_id_delta": delta,
    })
  return pairs


def summarize(args: argparse.Namespace) -> dict[str, Any]:
  measured_path = Path(args.measured_json).expanduser()
  predicted_path = Path(args.predicted_json).expanduser()
  measured_payload = load_json(measured_path)
  predicted_payload = load_json(predicted_path)

  measured_refs, predicted_refs, key_scheme = align_ref_sets(
    measured_leapfrog_refs(measured_payload), predicted_leapfrog_refs(predicted_payload))
  measured_set = sorted(set(measured_refs))
  predicted_set = sorted(set(predicted_refs))
  measured_lookup = set(measured_set)
  predicted_lookup = set(predicted_set)

  overlap = sorted(measured_lookup & predicted_lookup)
  measured_only = sorted(measured_lookup - predicted_lookup)
  predicted_only = sorted(predicted_lookup - measured_lookup)
  # event-id near-matching is a positional-id diagnostic; stable keys match exactly or not at all
  if key_scheme == "route_event_id":
    near_matches = find_near_matches(measured_only, predicted_only, max(int(args.event_id_tolerance), 0))
  else:
    near_matches = []

  measured_count = len(measured_set)
  predicted_count = len(predicted_set)
  overlap_count = len(overlap)
  recall = (overlap_count / measured_count) if measured_count > 0 else 1.0
  precision = (overlap_count / predicted_count) if predicted_count > 0 else 1.0
  count_delta = predicted_count - measured_count

  status = "pass"
  reasons: list[str] = []
  if args.min_overlap_recall > 0.0 and measured_count > 0 and recall < args.min_overlap_recall:
    status = "fail"
    reasons.append(f"overlap_recall={recall:.3f} < min_overlap_recall={args.min_overlap_recall:.3f}")
  if args.max_count_delta >= 0 and abs(count_delta) > args.max_count_delta:
    status = "fail"
    reasons.append(f"abs_count_delta={abs(count_delta)} > max_count_delta={args.max_count_delta}")

  return {
    "generated_utc": datetime.now(UTC).replace(microsecond=0).isoformat(),
    "status": status,
    "reasons": reasons,
    "measured_json": str(measured_path),
    "predicted_json": str(predicted_path),
    "measured_leapfrog_events": measured_count,
    "predicted_leapfrog_events": predicted_count,
    "overlap_events": overlap_count,
    "overlap_recall": recall,
    "overlap_precision": precision,
    "count_delta": count_delta,
    "event_id_tolerance": max(int(args.event_id_tolerance), 0),
    "key_scheme": key_scheme,
    "near_match_count": len(near_matches),
    "overlap_event_keys": [item.as_dict() for item in overlap],
    "measured_only_event_keys": [item.as_dict() for item in measured_only],
    "predicted_only_event_keys": [item.as_dict() for item in predicted_only],
    "near_matches": near_matches,
  }


def main() -> int:
  args = parse_args()
  measured_path = Path(args.measured_json).expanduser()
  predicted_path = Path(args.predicted_json).expanduser()
  if not measured_path.exists():
    print(f"[leapfrog-align] missing measured json: {measured_path}", file=sys.stderr)
    return 2
  if not predicted_path.exists():
    print(f"[leapfrog-align] missing predicted json: {predicted_path}", file=sys.stderr)
    return 2

  try:
    result = summarize(args)
  except (OSError, ValueError, json.JSONDecodeError) as exc:
    print(f"[leapfrog-align] failed: {exc}", file=sys.stderr)
    return 2

  print(f"[leapfrog-align] status={result['status']}")
  print(f"[leapfrog-align] measured_leapfrog_events={result['measured_leapfrog_events']}")
  print(f"[leapfrog-align] predicted_leapfrog_events={result['predicted_leapfrog_events']}")
  print(f"[leapfrog-align] overlap_events={result['overlap_events']}")
  print(f"[leapfrog-align] overlap_recall={result['overlap_recall']:.3f}")
  print(f"[leapfrog-align] overlap_precision={result['overlap_precision']:.3f}")
  print(f"[leapfrog-align] count_delta={result['count_delta']}")
  print(f"[leapfrog-align] near_match_count={result['near_match_count']}")
  if result["reasons"]:
    print(f"[leapfrog-align] reasons={'; '.join(result['reasons'])}")

  if args.output_json:
    output_path = Path(args.output_json).expanduser()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n")
    print(f"[leapfrog-align] output_json={output_path}")

  return 0 if result["status"] == "pass" else 1


if __name__ == "__main__":
  raise SystemExit(main())
