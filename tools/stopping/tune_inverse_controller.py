#!/usr/bin/env python3
"""Offline tuner for the inverse-model stop controller benchmark variant.

Searches over inverse-policy parameters using the same replay model and event filtering
logic as `benchmark_controller_variants.py`, but avoids re-loading qlogs for every
parameter set.
"""

from __future__ import annotations

import argparse
from collections.abc import Iterable
import json
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
  sys.path.insert(0, str(REPO_ROOT))

from openpilot.tools.stopping.benchmark_controller_variants import (  # pylint: disable=wrong-import-position
  simulate_event_with_inverse_controller,
  simulate_event_with_legacy_controller,
)
from openpilot.tools.stopping.check_harsh_stops_model import (  # pylint: disable=wrong-import-position
  DEFAULT_DOWNLOAD_ROOT,
  first_index_in_range,
  last_index_in_range,
  load_json,
  nearest_index,
  route_samples,
  score_event_metrics,
  simulate_event_with_controller,
)
from openpilot.tools.stopping.stopping_model import FittedStoppingModel  # pylint: disable=wrong-import-position


@dataclass(frozen=True)
class ParamSet:
  tau_s: float
  max_ref_decel: float
  hold_cmd_cap: float
  hold_cmd_speed: float
  kp: float
  ki: float
  step_scale: float
  brake_step_scale: float
  release_step_scale: float


@dataclass(frozen=True)
class VariantSummary:
  harsh_events: int
  harsh_rate: float
  leapfrog_events: int
  leapfrog_rate: float
  avg_score: float


def parse_csv_floats(text: str) -> list[float]:
  values: list[float] = []
  for raw in text.replace(";", ",").split(","):
    item = raw.strip()
    if not item:
      continue
    values.append(float(item))
  return values


def discover_summaries(analysis_root: Path, host: str, limit: int) -> list[Path]:
  root = (analysis_root / host).expanduser()
  if not root.exists():
    raise FileNotFoundError(f"Host analysis root not found: {root}")
  paths = sorted(root.rglob("summary.json"), key=lambda p: p.stat().st_mtime, reverse=True)
  if limit > 0:
    paths = paths[:limit]
  return paths


def enabled_ratio(samples: list[Any], start_idx: int, end_idx: int) -> float:
  if end_idx < start_idx:
    return 0.0
  flags = [1.0 if bool(samples[idx].enabled) else 0.0 for idx in range(start_idx, end_idx + 1)]
  return float(sum(flags) / max(len(flags), 1))


def iter_event_windows(
  summary_paths: list[Path],
  download_root: Path,
  event_source: str,
  min_entry_speed: float,
  controller_scope: str,
  min_enabled_ratio: float,
  window_mode: str,
  end_mode: str,
) -> tuple[list[dict[str, Any]], dict[tuple[str, str], list[Any]]]:
  sample_cache: dict[tuple[str, str], list[Any]] = {}
  segment_cache: dict[str, list[Any]] = {}
  windows: list[dict[str, Any]] = []

  for summary_path in summary_paths:
    summary = load_json(summary_path)
    host = str(summary.get("host", "commawifi"))
    route = str(summary.get("route", ""))
    if not route:
      continue
    samples = route_samples(sample_cache, segment_cache, download_root, host, route)
    times = [float(item.t) for item in samples]

    for event in summary.get("events", []):
      if not isinstance(event, dict):
        continue
      source = str(event.get("event_source", ""))
      if event_source != "all" and source != event_source:
        continue

      entry_speed = float(event.get("entry_speed_mps", 0.0))
      if entry_speed < min_entry_speed:
        continue

      start_time = event.get("start_time_s")
      hold_time = event.get("stop_hold_time_s")
      if start_time is None or hold_time is None:
        continue

      start_idx = nearest_index(times, float(start_time))
      hold_idx = nearest_index(times, float(hold_time))
      if hold_idx <= start_idx:
        continue

      sim_start_idx = start_idx
      sim_hold_idx = hold_idx
      should_stop_start = first_index_in_range(samples, start_idx, hold_idx, lambda item: item.should_stop)
      should_stop_end = last_index_in_range(samples, start_idx, hold_idx, lambda item: item.should_stop)
      stopping_start = first_index_in_range(
        samples,
        start_idx,
        hold_idx,
        lambda item: item.long_state == "stopping" or item.long_state_cmd == "stopping",
      )
      stopping_end = last_index_in_range(
        samples,
        start_idx,
        hold_idx,
        lambda item: item.long_state == "stopping" or item.long_state_cmd == "stopping",
      )

      if window_mode == "should_stop":
        if should_stop_start is None:
          continue
        sim_start_idx = should_stop_start
      elif window_mode == "stopping_state":
        if stopping_start is None:
          continue
        sim_start_idx = stopping_start

      if end_mode == "last_should_stop":
        if should_stop_end is None:
          continue
        sim_hold_idx = min(sim_hold_idx, should_stop_end)
      elif end_mode == "last_stopping_state":
        if stopping_end is None:
          continue
        sim_hold_idx = min(sim_hold_idx, stopping_end)

      if sim_hold_idx <= sim_start_idx:
        continue

      if controller_scope in ("engaged", "engaged_stopping"):
        en_ratio = enabled_ratio(samples, sim_start_idx, sim_hold_idx)
        if en_ratio < min_enabled_ratio:
          continue
        if controller_scope == "engaged_stopping" and (stopping_start is None or stopping_end is None):
          continue

      windows.append({
        "summary_json": str(summary_path),
        "host": host,
        "route": route,
        "event_id": event.get("event_id"),
        "event_source": source,
        "entry_speed_mps": entry_speed,
        "start_idx": sim_start_idx,
        "hold_idx": sim_hold_idx,
      })

  return windows, sample_cache


def summarize_variant(
  per_event: Iterable[dict[str, Any]],
  max_jerk: float,
  min_a_floor: float,
  max_rollout_m: float,
  max_rebound_should_stop: float,
  max_unexpected_accel_should_stop: float,
) -> VariantSummary:
  rows = list(per_event)
  harsh = 0
  leapfrog = 0
  score_sum = 0.0
  for item in rows:
    pred_jerk = item["pred_end_stop_jerk_mps3"]
    pred_min_a = float(item["pred_min_a_ego_mps2"])
    pred_rollout = float(item.get("pred_rollout_from_2mps_m", item.get("pred_rollout_distance_m", 0.0)))
    pred_rebound = item.get("pred_speed_rebound_while_should_stop_mps")
    pred_unexpected_accel = item.get("pred_should_stop_unexpected_accel_mps2")
    flags: list[str] = []
    if pred_jerk is not None and float(pred_jerk) > max_jerk:
      flags.append("pred_end_stop_jerk")
    if pred_min_a < min_a_floor:
      flags.append("pred_min_a_ego")
    if pred_rollout > max_rollout_m:
      flags.append("pred_rollout")
    if flags:
      harsh += 1

    rebound_flag = pred_rebound is not None and float(pred_rebound) > max_rebound_should_stop
    unexpected_accel_flag = (
      pred_unexpected_accel is not None and float(pred_unexpected_accel) > max_unexpected_accel_should_stop
    )
    if unexpected_accel_flag and not rebound_flag:
      # Keep parity with benchmark classification: unexpected accel alone does not define leapfrog.
      pass
    if rebound_flag:
      leapfrog += 1

    score_sum += score_event_metrics(pred_jerk, pred_min_a, pred_rollout, max_rollout_m)

  total = len(rows)
  return VariantSummary(
    harsh_events=harsh,
    harsh_rate=(harsh / total) if total else 0.0,
    leapfrog_events=leapfrog,
    leapfrog_rate=(leapfrog / total) if total else 0.0,
    avg_score=(score_sum / total) if total else 0.0,
  )


def parse_args() -> argparse.Namespace:
  parser = argparse.ArgumentParser(description="Tune inverse-model stop policy parameters offline")
  parser.add_argument("--model-json", required=True)
  parser.add_argument("--summary-json", action="append", default=[],
                      help="Repeatable. If omitted, discovers summaries under --analysis-root/--host.")
  parser.add_argument("--analysis-root", default=str(Path.home() / ".comma" / "stopping_behavior" / "analysis"))
  parser.add_argument("--host", default="commawifi")
  parser.add_argument("--max-summaries", type=int, default=0, help="Limit discovered summary.json files (0 = all)")
  parser.add_argument("--download-root", default=str(DEFAULT_DOWNLOAD_ROOT))
  parser.add_argument("--event-source", choices=["all", "signal", "speed", "hybrid"], default="all")
  parser.add_argument("--min-entry-speed", type=float, default=0.20)

  parser.add_argument("--controller-scope", choices=["all", "engaged", "engaged_stopping"], default="engaged_stopping")
  parser.add_argument("--controller-min-enabled-ratio", type=float, default=0.80)
  parser.add_argument("--controller-window-mode", choices=["event", "should_stop", "stopping_state"], default="stopping_state")
  parser.add_argument("--controller-end-mode", choices=["hold", "last_should_stop", "last_stopping_state"], default="last_stopping_state")

  parser.add_argument("--max-pred-end-jerk", type=float, default=0.70)
  parser.add_argument("--min-pred-a-floor", type=float, default=-1.10)
  parser.add_argument("--max-pred-rollout-m", type=float, default=2.0)
  parser.add_argument("--max-pred-speed-rebound-while-should-stop", type=float, default=0.08)
  parser.add_argument("--max-pred-should-stop-unexpected-accel", type=float, default=0.10)

  parser.add_argument("--tau-grid", default="0.80,0.85,0.90,0.95,1.00")
  parser.add_argument("--max-ref-grid", default="1.00")
  parser.add_argument("--hold-cap-grid", default="-0.20,-0.22,-0.25,-0.28")
  parser.add_argument("--hold-speed-grid", default="0.06,0.10,0.14")
  parser.add_argument("--kp-grid", default="0.08,0.10,0.12")
  parser.add_argument("--ki-grid", default="0.03,0.05,0.07")
  parser.add_argument("--step-grid", default="1.0")
  parser.add_argument("--brake-step-grid", default="1.0")
  parser.add_argument("--release-step-grid", default="1.0")

  parser.add_argument("--top-n", type=int, default=12)
  parser.add_argument("--output-json", default=None)
  return parser.parse_args()


def main() -> int:
  args = parse_args()
  model_payload = load_json(Path(args.model_json).expanduser())
  model_data = model_payload["model"] if "model" in model_payload else model_payload
  model = FittedStoppingModel.from_json(model_data)

  summary_paths = [Path(p).expanduser() for p in args.summary_json]
  if not summary_paths:
    summary_paths = discover_summaries(Path(args.analysis_root), args.host, args.max_summaries)
  summary_paths = [p for p in summary_paths if p.exists()]
  if not summary_paths:
    print("[tune-inverse] no summary.json inputs found", file=sys.stderr)
    return 2

  windows, sample_cache = iter_event_windows(
    summary_paths=summary_paths,
    download_root=Path(args.download_root).expanduser(),
    event_source=args.event_source,
    min_entry_speed=args.min_entry_speed,
    controller_scope=args.controller_scope,
    min_enabled_ratio=args.controller_min_enabled_ratio,
    window_mode=args.controller_window_mode,
    end_mode=args.controller_end_mode,
  )
  if not windows:
    print("[tune-inverse] no events after filtering", file=sys.stderr)
    return 2

  route_count = len({w["route"] for w in windows})
  print(f"[tune-inverse] events={len(windows)} summaries={len(summary_paths)} routes={route_count}")

  # Baselines (constant across parameter sets).
  current_rows: list[dict[str, Any]] = []
  legacy_rows: list[dict[str, Any]] = []
  for w in windows:
    samples = sample_cache[(w["host"], w["route"])]
    current_rows.append(
      simulate_event_with_controller(
        samples=samples,
        start_idx=int(w["start_idx"]),
        hold_idx=int(w["hold_idx"]),
        model=model,
        stopping_speed_breakpoint=0.40,
        stop_accel=-2.0,
      )
    )
    legacy_rows.append(
      simulate_event_with_legacy_controller(
        samples=samples,
        start_idx=int(w["start_idx"]),
        hold_idx=int(w["hold_idx"]),
        model=model,
        stop_accel=-2.0,
        stopping_speed_breakpoint=0.40,
        stopping_error_factor=1.3,
      )
    )

  current_summary = summarize_variant(
    current_rows,
    args.max_pred_end_jerk,
    args.min_pred_a_floor,
    args.max_pred_rollout_m,
    args.max_pred_speed_rebound_while_should_stop,
    args.max_pred_should_stop_unexpected_accel,
  )
  legacy_summary = summarize_variant(
    legacy_rows,
    args.max_pred_end_jerk,
    args.min_pred_a_floor,
    args.max_pred_rollout_m,
    args.max_pred_speed_rebound_while_should_stop,
    args.max_pred_should_stop_unexpected_accel,
  )
  print(
    f"[tune-inverse] current harsh={current_summary.harsh_events}/{len(windows)} "
    + f"leapfrog={current_summary.leapfrog_events}/{len(windows)} avg_score={current_summary.avg_score:.3f}"
  )
  print(
    f"[tune-inverse] legacy_32b8be harsh={legacy_summary.harsh_events}/{len(windows)} "
    + f"leapfrog={legacy_summary.leapfrog_events}/{len(windows)} avg_score={legacy_summary.avg_score:.3f}"
  )

  grids = {
    "tau_s": parse_csv_floats(args.tau_grid),
    "max_ref_decel": parse_csv_floats(args.max_ref_grid),
    "hold_cmd_cap": parse_csv_floats(args.hold_cap_grid),
    "hold_cmd_speed": parse_csv_floats(args.hold_speed_grid),
    "kp": parse_csv_floats(args.kp_grid),
    "ki": parse_csv_floats(args.ki_grid),
    "step_scale": parse_csv_floats(args.step_grid),
    "brake_step_scale": parse_csv_floats(args.brake_step_grid),
    "release_step_scale": parse_csv_floats(args.release_step_grid),
  }
  total = 1
  for name, values in grids.items():
    if not values:
      print(f"[tune-inverse] empty grid: {name}", file=sys.stderr)
      return 2
    total *= len(values)
  print(f"[tune-inverse] combinations={total}")

  results: list[dict[str, Any]] = []
  best: dict[str, Any] | None = None

  for tau in grids["tau_s"]:
    for max_ref in grids["max_ref_decel"]:
      for cap in grids["hold_cmd_cap"]:
        for hold_speed in grids["hold_cmd_speed"]:
          for kp in grids["kp"]:
            for ki in grids["ki"]:
              for step in grids["step_scale"]:
                for brake_scale in grids["brake_step_scale"]:
                  for release_scale in grids["release_step_scale"]:
                    params = ParamSet(
                      tau_s=tau,
                      max_ref_decel=max_ref,
                      hold_cmd_cap=cap,
                      hold_cmd_speed=hold_speed,
                      kp=kp,
                      ki=ki,
                      step_scale=step,
                      brake_step_scale=brake_scale,
                      release_step_scale=release_scale,
                    )

                    inv_rows: list[dict[str, Any]] = []
                    for w in windows:
                      samples = sample_cache[(w["host"], w["route"])]
                      inv_rows.append(
                        simulate_event_with_inverse_controller(
                          samples=samples,
                          start_idx=int(w["start_idx"]),
                          hold_idx=int(w["hold_idx"]),
                          model=model,
                          stop_accel=-2.0,
                          stopping_speed_breakpoint=0.40,
                          tau_s=params.tau_s,
                          max_ref_decel=params.max_ref_decel,
                          hold_cmd_cap=params.hold_cmd_cap,
                          hold_cmd_speed=params.hold_cmd_speed,
                          kp=params.kp,
                          ki=params.ki,
                          step_scale=params.step_scale,
                          brake_step_scale=params.brake_step_scale,
                          release_step_scale=params.release_step_scale,
                        )
                      )

                    summary = summarize_variant(
                      inv_rows,
                      args.max_pred_end_jerk,
                      args.min_pred_a_floor,
                      args.max_pred_rollout_m,
                      args.max_pred_speed_rebound_while_should_stop,
                      args.max_pred_should_stop_unexpected_accel,
                    )
                    payload = {
                      "params": params.__dict__,
                      "inverse": summary.__dict__,
                      "inverse_delta_vs_current": {
                        "harsh_events": summary.harsh_events - current_summary.harsh_events,
                        "leapfrog_events": summary.leapfrog_events - current_summary.leapfrog_events,
                        "avg_score": summary.avg_score - current_summary.avg_score,
                      },
                    }
                    results.append(payload)
                    if best is None:
                      best = payload
                    else:
                      b = best["inverse"]
                      if (summary.harsh_events, summary.leapfrog_events, summary.avg_score) < (
                        b["harsh_events"],
                        b["leapfrog_events"],
                        b["avg_score"],
                      ):
                        best = payload

  results.sort(key=lambda item: (item["inverse"]["harsh_events"], item["inverse"]["leapfrog_events"], item["inverse"]["avg_score"]))

  print("[tune-inverse] top:")
  limit = max(int(args.top_n), 1)
  for idx, item in enumerate(results[:limit], start=1):
    inv = item["inverse"]
    prm = item["params"]
    line = (
      f"[tune-inverse] #{idx} harsh={inv['harsh_events']}/{len(windows)} "
      + f"leapfrog={inv['leapfrog_events']}/{len(windows)} avg={inv['avg_score']:.3f} "
      + f"tau={prm['tau_s']:.2f} max_ref={prm['max_ref_decel']:.2f} cap={prm['hold_cmd_cap']:.2f} hold={prm['hold_cmd_speed']:.2f} "
      + f"kp={prm['kp']:.2f} ki={prm['ki']:.2f} step={prm['step_scale']:.2f} "
      + f"br={prm['brake_step_scale']:.2f} rel={prm['release_step_scale']:.2f}"
    )
    print(line)

  if best is not None:
    inv = best["inverse"]
    prm = best["params"]
    best_line = (
      "[tune-inverse] best: "
      + f"harsh={inv['harsh_events']}/{len(windows)} leapfrog={inv['leapfrog_events']}/{len(windows)} "
      + f"avg={inv['avg_score']:.3f} "
      + f"tau={prm['tau_s']:.2f} max_ref={prm['max_ref_decel']:.2f} cap={prm['hold_cmd_cap']:.2f} hold={prm['hold_cmd_speed']:.2f} "
      + f"kp={prm['kp']:.2f} ki={prm['ki']:.2f} step={prm['step_scale']:.2f} "
      + f"br={prm['brake_step_scale']:.2f} rel={prm['release_step_scale']:.2f}"
    )
    print(best_line)

  if args.output_json:
    out = Path(args.output_json).expanduser()
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(
      json.dumps(
        {
          "events": len(windows),
          "summaries": [str(p) for p in summary_paths],
          "baselines": {
            "current": current_summary.__dict__,
            "legacy_32b8be": legacy_summary.__dict__,
          },
          "results": results,
        },
        indent=2,
        sort_keys=True,
      )
      + "\n"
    )
    print(f"[tune-inverse] output_json={out}")

  return 0


if __name__ == "__main__":
  raise SystemExit(main())
