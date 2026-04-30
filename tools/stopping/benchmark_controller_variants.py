#!/usr/bin/env python3
"""Compare stop-controller replays using a fitted stopping-response model."""

from __future__ import annotations

import argparse
import json
import sys
from collections import Counter
from dataclasses import dataclass
from datetime import UTC, datetime
from pathlib import Path
from typing import Any, Iterable

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
  sys.path.insert(0, str(REPO_ROOT))

from openpilot.common.numpy_fast import clip, interp
from openpilot.selfdrive.controls.lib.stopping_profile_selector import (  # pylint: disable=wrong-import-position
  StoppingProfileFeatures,
  PrototypeStoppingProfileSelector,
  nearest_exemplar_distance_for_profile,
  profile_label_from_horizon_teacher,
  residual_template_for_profile,
)
from openpilot.tools.stopping.check_harsh_stops_model import (  # pylint: disable=wrong-import-position
  classify_stop_distance,
  classify_pred_leapfrog,
  compute_end_stop_sharpness_metrics,
  compute_predicted_lead_distance_metrics,
  compute_pred_leapfrog_metrics,
  DEFAULT_MAX_PRED_LEAD_HOLD_DISTANCE_M,
  DEFAULT_MIN_PRED_LEAD_HOLD_DISTANCE_M,
  DEFAULT_DOWNLOAD_ROOT,
  infer_hold_time_s,
  jerk_window_metrics,
  last_contiguous_index_span,
  load_json,
  nearest_index,
  route_samples,
  sample_value,
  score_event_metrics,
  simulate_event_with_controller,
)
from openpilot.tools.stopping.horizon_optimizer import (
  HorizonOptimizerConfig,
  simulate_event_with_horizon_v1_controller,
  simulate_event_with_residual_profile,
)
from openpilot.tools.stopping.stopping_model import FittedStoppingModel


@dataclass
class VariantMetrics:
  pred_end_stop_jerk_mps3: float | None
  pred_end_stop_cmd_jerk_mps3: float | None
  pred_end_stop_accel_step_mps2: float | None
  pred_min_a_ego_mps2: float
  pred_rollout_distance_m: float
  pred_lead_distance_hold_m: float | None
  recorded_lead_distance_hold_m: float | None
  distance_gate_source: str
  pred_speed_rebound_while_should_stop_mps: float | None
  pred_should_stop_unexpected_accel_mps2: float | None
  event_score: float
  is_harsh: bool
  flags: list[str]
  is_leapfrog: bool
  leapfrog_flags: list[str]


def iter_route_summaries(summary_path: Path) -> Iterable[dict[str, Any]]:
  payload = load_json(summary_path)
  route = str(payload.get("route", ""))
  if route:
    yield payload
    return

  host = str(payload.get("host", "commawifi"))
  routes = payload.get("routes", [])
  if not isinstance(routes, list):
    return
  for route_summary in routes:
    if not isinstance(route_summary, dict):
      continue
    summary = dict(route_summary)
    summary.setdefault("host", host)
    yield summary


def invert_command_with_model(model: FittedStoppingModel, *, a_prev: float, v_ego: float, a_next_des: float) -> float:
  coef = model.effective_coefficients(v_ego)
  low_speed = max(0.0, min(1.0, (model.low_speed_ref - v_ego) / max(model.low_speed_ref, 1e-6)))
  base = (
    coef["intercept"]
    + (coef["a_ego_prev"] * float(a_prev))
    + (coef["v_ego"] * float(v_ego))
    + (coef["low_speed"] * low_speed)
  )
  cmd_k0 = coef["accel_cmd_delayed"] + (coef["cmd_x_low_speed"] * low_speed)
  relief_k = coef["relief"]
  threshold = model.relief_cmd_threshold

  cmd_no_relief: float | None = None
  if abs(cmd_k0) > 1e-6:
    cmd_no_relief = (float(a_next_des) - base) / cmd_k0

  cmd_relief: float | None = None
  cmd_k1 = cmd_k0 + relief_k
  if abs(cmd_k1) > 1e-6:
    cmd_relief = (float(a_next_des) - (base - relief_k * threshold)) / cmd_k1

  if cmd_no_relief is not None and cmd_no_relief <= threshold:
    return float(cmd_no_relief)
  if cmd_relief is not None and cmd_relief > threshold:
    return float(cmd_relief)
  return float(threshold)


class _LegacyStoppingController32b8be:
  """Legacy stop logic snapshot from commit 32b8be... (old longcontrol.py stopping branch).

  This is only for offline benchmarking against recorded events/models.
  """

  def __init__(self) -> None:
    self.breakpoint_v = 1.0
    self.breakpoint_recorded = False

  def update(
    self,
    output_accel: float,
    v_ego: float,
    a_ego: float,
    max_expected_accel: float,
    min_expected_accel: float,
    stop_accel: float,
    stopping_speed_breakpoint: float,
    stopping_error_factor: float,
    dt: float,
  ) -> float:
    if not self.breakpoint_recorded and v_ego < 0.5:
      self.breakpoint_recorded = True
      self.breakpoint_v = interp(a_ego, [-1.0, -0.1], [1.0, 0.5])

    cmd = min(output_accel, -0.1)

    mid_bp = clip(stopping_speed_breakpoint, 0.011, 0.499)
    stopping_v_bp = [0.01, mid_bp, 0.50]
    stopping_v = [0.1, mid_bp, self.breakpoint_v]

    if a_ego > max_expected_accel or (v_ego < 1.0 and a_ego < min_expected_accel):
      release_step = interp(v_ego, stopping_v_bp, stopping_v)
      error_factor = 0.12 if a_ego > min_expected_accel else stopping_error_factor
      error = max_expected_accel - ((min_expected_accel - max_expected_accel) * error_factor) - a_ego
      step_factor = release_step if (a_ego < max_expected_accel or a_ego > 0.1) else 0.1
      cmd += error * step_factor * dt

    return float(clip(cmd, stop_accel, -0.05))


def parse_args() -> argparse.Namespace:
  parser = argparse.ArgumentParser(description="Compare active stopping-controller variants on replay")
  parser.add_argument("--model-json", required=True)
  parser.add_argument("--summary-json", action="append", required=True)
  parser.add_argument("--download-root", default=str(DEFAULT_DOWNLOAD_ROOT))
  parser.add_argument("--event-source", choices=["all", "signal", "speed", "hybrid"], default="all")
  parser.add_argument("--min-entry-speed", type=float, default=0.20)
  parser.add_argument("--stop-accel", type=float, default=-2.0)
  parser.add_argument("--stopping-speed-breakpoint", type=float, default=0.40)
  parser.add_argument("--stopping-error-factor", type=float, default=1.3)
  parser.add_argument("--horizon-v1-horizon-s", type=float, default=1.2, help="Tail horizon length for horizon_v1 sequence search")
  parser.add_argument("--horizon-v1-block-s", type=float, default=0.10, help="Piecewise-constant block length for horizon_v1 residual search")
  parser.add_argument("--horizon-v1-beam-width", type=int, default=24, help="Beam width for horizon_v1 residual search")
  parser.add_argument("--horizon-v1-residual-grid", default="-0.12,-0.06,0.0,0.06,0.12",
                      help="Comma-separated accel-command residual grid for horizon_v1")
  parser.add_argument("--controller-scope", choices=["all", "engaged", "engaged_stopping"], default="engaged_stopping")
  parser.add_argument("--controller-min-enabled-ratio", type=float, default=0.80)
  parser.add_argument("--controller-window-mode", choices=["event", "should_stop", "stopping_state"], default="stopping_state")
  parser.add_argument("--controller-end-mode", choices=["hold", "last_should_stop", "last_stopping_state"], default="last_stopping_state")
  parser.add_argument("--max-pred-end-jerk", type=float, default=0.70)
  parser.add_argument("--max-pred-end-cmd-jerk", type=float, default=3.0)
  parser.add_argument("--max-pred-end-accel-step", type=float, default=0.08)
  parser.add_argument("--min-pred-a-floor", type=float, default=-1.10)
  parser.add_argument("--max-pred-rollout-m", type=float, default=2.0)
  parser.add_argument("--min-pred-lead-hold-distance-m", type=float, default=DEFAULT_MIN_PRED_LEAD_HOLD_DISTANCE_M)
  parser.add_argument("--max-pred-lead-hold-distance-m", type=float, default=DEFAULT_MAX_PRED_LEAD_HOLD_DISTANCE_M)
  parser.add_argument("--max-pred-speed-rebound-while-should-stop", type=float, default=0.08)
  parser.add_argument("--max-pred-should-stop-unexpected-accel", type=float, default=0.10)
  parser.add_argument("--profile-selector-json", default=None, help="Optional selector JSON produced by train_profile_selector.py")
  parser.add_argument("--profile-selector-mode", choices=["select", "oracle"], default="select",
                      help="select uses the selector decision; oracle evaluates the learned profile library with the plant model")
  parser.add_argument("--profile-selector-min-confidence", type=float, default=0.0)
  parser.add_argument("--profile-selector-require-exemplar", action="store_true",
                      help="Fall back to current behavior unless the selected profile has a nearby training exemplar")
  parser.add_argument("--profile-selector-max-exemplar-distance", type=float, default=0.90)
  parser.add_argument("--include-route", action="append", default=[], help="Only benchmark this route; may be repeated")
  parser.add_argument("--exclude-route", action="append", default=[], help="Exclude this route from benchmarking; may be repeated")
  parser.add_argument("--output-json", default=None)
  args = parser.parse_args()
  args.horizon_v1_residual_grid = tuple(
    float(item.strip()) for item in str(args.horizon_v1_residual_grid).split(",") if item.strip()
  ) or HorizonOptimizerConfig().residual_grid_mps2
  return args


def enabled_ratio(samples: list[Any], start_idx: int, end_idx: int) -> float:
  if end_idx < start_idx:
    return 0.0
  count = end_idx - start_idx + 1
  return sum(1.0 if bool(samples[idx].enabled) else 0.0 for idx in range(start_idx, end_idx + 1)) / max(count, 1)


def mean_or_zero(values: list[float]) -> float:
  return float(sum(values) / len(values)) if values else 0.0


def top_counter_rows(counter: Counter[str], limit: int = 5) -> list[dict[str, Any]]:
  return [{"name": name, "count": int(count)} for name, count in counter.most_common(limit)]


def classify_horizon_delta_shape(
  first_avg: float,
  middle_avg: float,
  final_avg: float,
  avg_delta: float,
  max_soften: float,
  max_deepen: float,
) -> str:
  material_delta = 0.015
  has_soften = max_soften > material_delta
  has_deepen = max_deepen > material_delta
  if not has_soften and not has_deepen:
    return "match"
  if first_avg < -material_delta and final_avg > material_delta:
    return "deepen_then_soften"
  if first_avg > material_delta and final_avg < -material_delta:
    return "soften_then_deepen"
  if has_soften and has_deepen:
    if final_avg > material_delta and first_avg >= -material_delta:
      return "tail_soften"
    if final_avg < -material_delta and first_avg <= material_delta:
      return "tail_deepen"
    return "reshape"
  if avg_delta > material_delta or max_soften > (1.5 * max_deepen):
    return "soften"
  if avg_delta < -material_delta or max_deepen > (1.5 * max_soften):
    return "deepen"
  if middle_avg > material_delta or final_avg > material_delta:
    return "tail_soften"
  if middle_avg < -material_delta or final_avg < -material_delta:
    return "tail_deepen"
  return "reshape"


def summarize_horizon_teacher(current: dict[str, Any], horizon_v1: dict[str, Any], current_score: float, horizon_score: float) -> dict[str, Any]:
  current_trace = current.get("trace")
  horizon_trace = horizon_v1.get("trace")
  if not isinstance(current_trace, dict) or not isinstance(horizon_trace, dict):
    return {
      "status": "missing_trace",
      "score_delta": float(horizon_score - current_score),
    }

  current_outputs = [float(value) for value in current_trace.get("output_trace", [])]
  horizon_outputs = [float(value) for value in horizon_trace.get("output_trace", [])]
  trace_len = min(len(current_outputs), len(horizon_outputs))
  if trace_len < 2:
    return {
      "status": "too_short",
      "score_delta": float(horizon_score - current_score),
    }

  search_start_step = int(horizon_v1.get("optimizer_search_start_step", 0))
  first_output_idx = max(1, min(search_start_step + 1, trace_len - 1))
  deltas = [horizon_outputs[idx] - current_outputs[idx] for idx in range(first_output_idx, trace_len)]
  if not deltas:
    return {
      "status": "too_short",
      "score_delta": float(horizon_score - current_score),
      "optimizer_search_start_step": search_start_step,
    }

  third = max(1, len(deltas) // 3)
  first = deltas[:third]
  middle = deltas[third:2 * third]
  final = deltas[2 * third:]
  first_avg = mean_or_zero(first)
  middle_avg = mean_or_zero(middle)
  final_avg = mean_or_zero(final)
  avg_delta = mean_or_zero(deltas)
  max_soften = max(max(deltas), 0.0)
  max_deepen = max(-min(deltas), 0.0)
  intent = classify_horizon_delta_shape(
    first_avg=first_avg,
    middle_avg=middle_avg,
    final_avg=final_avg,
    avg_delta=avg_delta,
    max_soften=max_soften,
    max_deepen=max_deepen,
  )

  debug_trace = current_trace.get("debug_trace", [])
  trigger_counts: Counter[str] = Counter()
  phase_counts: Counter[str] = Counter()
  if isinstance(debug_trace, list):
    debug_start = max(0, first_output_idx - 1)
    debug_end = max(debug_start, min(len(debug_trace), trace_len - 1))
    for debug_step in debug_trace[debug_start:debug_end]:
      if not isinstance(debug_step, dict):
        continue
      for trigger in debug_step.get("triggers", []):
        trigger_counts[str(trigger)] += 1
      phase = debug_step.get("phase")
      if phase is not None:
        phase_counts[str(phase)] += 1

  return {
    "status": "ok",
    "intent": intent,
    "score_delta": float(horizon_score - current_score),
    "avg_cmd_delta_mps2": avg_delta,
    "first_third_avg_cmd_delta_mps2": first_avg,
    "middle_third_avg_cmd_delta_mps2": middle_avg,
    "final_third_avg_cmd_delta_mps2": final_avg,
    "max_soften_cmd_delta_mps2": max_soften,
    "max_deepen_cmd_delta_mps2": max_deepen,
    "optimizer_search_start_step": search_start_step,
    "optimizer_steps": len(deltas),
    "top_current_triggers": top_counter_rows(trigger_counts),
    "top_current_phases": top_counter_rows(phase_counts),
  }


def aggregate_horizon_teacher(rows: list[dict[str, Any]]) -> dict[str, Any]:
  intent_counts: Counter[str] = Counter()
  improved_intent_counts: Counter[str] = Counter()
  worsened_intent_counts: Counter[str] = Counter()
  improved_trigger_counts: Counter[str] = Counter()
  worsened_trigger_counts: Counter[str] = Counter()
  score_delta_by_intent: dict[str, list[float]] = {}
  missing_trace_events = 0

  for row in rows:
    teacher = row.get("horizon_teacher", {})
    if not isinstance(teacher, dict) or teacher.get("status") != "ok":
      missing_trace_events += 1
      continue

    intent = str(teacher.get("intent", "unknown"))
    score_delta = float(teacher.get("score_delta", 0.0))
    intent_counts[intent] += 1
    score_delta_by_intent.setdefault(intent, []).append(score_delta)

    top_triggers = teacher.get("top_current_triggers", [])
    if score_delta < -1e-6:
      improved_intent_counts[intent] += 1
      trigger_counter = improved_trigger_counts
    elif score_delta > 1e-6:
      worsened_intent_counts[intent] += 1
      trigger_counter = worsened_trigger_counts
    else:
      continue

    if isinstance(top_triggers, list):
      for item in top_triggers:
        if isinstance(item, dict) and "name" in item:
          trigger_counter[str(item["name"])] += int(item.get("count", 0))

  return {
    "intent_counts": dict(sorted(intent_counts.items())),
    "improved_intent_counts": dict(sorted(improved_intent_counts.items())),
    "worsened_intent_counts": dict(sorted(worsened_intent_counts.items())),
    "avg_score_delta_by_intent": {
      intent: mean_or_zero(values) for intent, values in sorted(score_delta_by_intent.items())
    },
    "top_improved_current_triggers": top_counter_rows(improved_trigger_counts, limit=10),
    "top_worsened_current_triggers": top_counter_rows(worsened_trigger_counts, limit=10),
    "missing_trace_events": missing_trace_events,
  }


def _trace_value(values: Any, idx: int, default: float = 0.0) -> float:
  if not isinstance(values, list) or not values:
    return default
  value_idx = max(0, min(idx, len(values) - 1))
  try:
    return float(values[value_idx])
  except (TypeError, ValueError):
    return default


def _trace_bool(values: Any, idx: int, default: bool = True) -> bool:
  if not isinstance(values, list) or not values:
    return default
  value_idx = max(0, min(idx, len(values) - 1))
  return bool(values[value_idx])


def _debug_value(debug_step: dict[str, Any], name: str, default: float | None = None) -> float | None:
  value = debug_step.get(name)
  if value is None:
    return default
  try:
    return float(value)
  except (TypeError, ValueError):
    return default


def selector_features_from_replay(samples: list[Any], current: dict[str, Any], horizon_v1: dict[str, Any]) -> dict[str, float]:
  trace = current.get("trace")
  if not isinstance(trace, dict):
    return {}

  output_trace = trace.get("output_trace", [])
  if not isinstance(output_trace, list) or not output_trace:
    return {}

  search_start_step = int(horizon_v1.get("optimizer_search_start_step", 0))
  step_idx = max(0, min(search_start_step, len(output_trace) - 1))
  debug_trace = trace.get("debug_trace", [])
  debug_step: dict[str, Any] = {}
  if isinstance(debug_trace, list) and debug_trace:
    debug_idx = max(0, min(step_idx - 1, len(debug_trace) - 1))
    maybe_debug = debug_trace[debug_idx]
    if isinstance(maybe_debug, dict):
      debug_step = maybe_debug

  replay_indices = trace.get("replay_sample_indices", [])
  sample_idx = int(_trace_value(replay_indices, step_idx, 0.0))
  sample_idx = max(0, min(sample_idx, len(samples) - 1))
  sample = samples[sample_idx]

  distance_to_stop_target_m = _debug_value(debug_step, "distance_to_stop_target_m")
  remaining_m = _debug_value(debug_step, "remaining_m")
  lead_status = bool(sample_value(sample, "lead_status", False))
  lead_d_rel_raw = sample_value(sample, "lead_d_rel_m", None)
  lead_d_rel_m = float(lead_d_rel_raw) if lead_status and lead_d_rel_raw is not None else None

  features = StoppingProfileFeatures(
    v_ego_mps=_trace_value(trace.get("predicted_v"), step_idx),
    a_ego_mps2=_trace_value(trace.get("predicted_a"), step_idx),
    last_output_accel_mps2=_trace_value(output_trace, step_idx),
    rollout_m=float(_debug_value(debug_step, "rollout_m", 0.0) or 0.0),
    remaining_m=remaining_m,
    lead_d_rel_m=lead_d_rel_m,
    lead_v_mps=float(sample_value(sample, "lead_v", 0.0) or 0.0),
    phase=int(_debug_value(debug_step, "phase", 0.0) or 0.0),
    release_lock_active=bool(debug_step.get("release_lock_active", False)),
    rebound_arrest_active=bool(debug_step.get("rebound_arrest_active", False)),
    explicit_target_available=distance_to_stop_target_m is not None and distance_to_stop_target_m >= 0.0,
    should_stop=_trace_bool(trace.get("should_stop_trace"), step_idx, True),
  )
  return features.as_dict()


def selector_residual_target_from_replay(current: dict[str, Any], horizon_v1: dict[str, Any], block_count: int = 12) -> dict[str, Any]:
  current_trace = current.get("trace")
  horizon_trace = horizon_v1.get("trace")
  if not isinstance(current_trace, dict) or not isinstance(horizon_trace, dict):
    return {}

  current_outputs = [float(value) for value in current_trace.get("output_trace", [])]
  horizon_outputs = [float(value) for value in horizon_trace.get("output_trace", [])]
  trace_len = min(len(current_outputs), len(horizon_outputs))
  if trace_len < 2:
    return {}

  search_start_step = int(horizon_v1.get("optimizer_search_start_step", 0))
  first_output_idx = max(1, min(search_start_step + 1, trace_len - 1))
  deltas = [horizon_outputs[idx] - current_outputs[idx] for idx in range(first_output_idx, trace_len)]
  if not deltas:
    return {}

  blocks: list[float] = []
  for block_idx in range(max(1, int(block_count))):
    start = (block_idx * len(deltas)) // max(1, int(block_count))
    end = ((block_idx + 1) * len(deltas)) // max(1, int(block_count))
    block_values = deltas[start:max(start + 1, end)]
    blocks.append(mean_or_zero(block_values))

  return {
    "residual_template_mps2": [float(clip(value, -0.20, 0.20)) for value in blocks],
    "avg_residual_mps2": mean_or_zero(deltas),
    "max_soften_mps2": max(max(deltas), 0.0),
    "max_deepen_mps2": max(-min(deltas), 0.0),
    "optimizer_search_start_step": search_start_step,
    "steps": len(deltas),
  }


def classify(metrics: dict[str, Any], args: argparse.Namespace) -> VariantMetrics:
  pred_jerk = metrics["pred_end_stop_jerk_mps3"]
  pred_cmd_jerk = metrics.get("pred_end_stop_cmd_jerk_mps3")
  pred_accel_step = metrics.get("pred_end_stop_accel_step_mps2")
  pred_min_a = float(metrics["pred_min_a_ego_mps2"])
  pred_rollout_raw = metrics.get("pred_rollout_from_2mps_m")
  if pred_rollout_raw is None:
    pred_rollout_raw = metrics.get("pred_rollout_distance_m", 0.0)
  pred_rollout = float(pred_rollout_raw)
  pred_lead_hold_raw = metrics.get("pred_lead_distance_hold_m")
  pred_lead_hold = float(pred_lead_hold_raw) if pred_lead_hold_raw is not None else None
  recorded_lead_hold_raw = metrics.get("recorded_lead_distance_hold_m")
  recorded_lead_hold = float(recorded_lead_hold_raw) if recorded_lead_hold_raw is not None else None
  pred_rebound = metrics.get("pred_speed_rebound_while_should_stop_mps")
  pred_unexpected_accel = metrics.get("pred_should_stop_unexpected_accel_mps2")
  flags: list[str] = []
  if pred_jerk is not None and pred_jerk > args.max_pred_end_jerk:
    flags.append("pred_end_stop_jerk")
  if pred_cmd_jerk is not None and pred_cmd_jerk > args.max_pred_end_cmd_jerk:
    flags.append("pred_end_stop_cmd_jerk")
  if pred_accel_step is not None and pred_accel_step > args.max_pred_end_accel_step:
    flags.append("pred_end_stop_accel_step")
  if pred_min_a < args.min_pred_a_floor:
    flags.append("pred_min_a_ego")
  distance_flags, distance_gate_source, _distance_metric_value = classify_stop_distance(
    pred_rollout,
    pred_lead_hold,
    max_rollout_m=args.max_pred_rollout_m,
    min_lead_hold_m=args.min_pred_lead_hold_distance_m,
    max_lead_hold_m=args.max_pred_lead_hold_distance_m,
    recorded_lead_hold_m=recorded_lead_hold,
  )
  flags.extend(distance_flags)
  leapfrog_flags = classify_pred_leapfrog(pred_rebound, pred_unexpected_accel, args)
  return VariantMetrics(
    pred_end_stop_jerk_mps3=pred_jerk,
    pred_end_stop_cmd_jerk_mps3=float(pred_cmd_jerk) if pred_cmd_jerk is not None else None,
    pred_end_stop_accel_step_mps2=float(pred_accel_step) if pred_accel_step is not None else None,
    pred_min_a_ego_mps2=pred_min_a,
    pred_rollout_distance_m=pred_rollout,
    pred_lead_distance_hold_m=pred_lead_hold,
    recorded_lead_distance_hold_m=recorded_lead_hold,
    distance_gate_source=distance_gate_source,
    pred_speed_rebound_while_should_stop_mps=float(pred_rebound) if pred_rebound is not None else None,
    pred_should_stop_unexpected_accel_mps2=float(pred_unexpected_accel) if pred_unexpected_accel is not None else None,
    event_score=score_event_metrics(
      pred_jerk,
      pred_min_a,
      pred_rollout,
      args.max_pred_rollout_m,
      pred_lead_hold_m=pred_lead_hold,
      recorded_lead_hold_m=recorded_lead_hold,
      min_lead_hold_m=args.min_pred_lead_hold_distance_m,
      max_lead_hold_m=args.max_pred_lead_hold_distance_m,
      pred_cmd_jerk=pred_cmd_jerk,
      max_cmd_jerk=args.max_pred_end_cmd_jerk,
      pred_accel_step=pred_accel_step,
      max_accel_step=args.max_pred_end_accel_step,
    ),
    is_harsh=bool(flags),
    flags=flags,
    is_leapfrog=bool(leapfrog_flags),
    leapfrog_flags=leapfrog_flags,
  )


def simulate_event_with_legacy_controller(
  samples: list[Any],
  start_idx: int,
  hold_idx: int,
  model: FittedStoppingModel,
  stop_accel: float,
  stopping_speed_breakpoint: float,
  stopping_error_factor: float,
) -> dict[str, Any]:
  start = max(0, int(start_idx))
  hold = max(start + 1, min(int(hold_idx), len(samples) - 1))
  if hold <= start:
    raise ValueError("Event window too short for legacy replay")

  mid_bp = clip(stopping_speed_breakpoint, 0.011, 0.499)
  v_bp = [0.01, mid_bp, 0.50]
  max_accel_bp = [-0.01, -0.10, -0.30]
  min_accel_bp = [-0.10, -0.50, -1.00]

  controller = _LegacyStoppingController32b8be()
  dt = max(model.dt_s, 1e-3)
  v_ego = float(samples[start].v_ego)
  a_ego = float(samples[start].a_ego)
  history_start = max(0, start - model.delay_frames)
  command_trace: list[float] = []
  for idx in range(history_start, start + 1):
    cmd = samples[idx].accel_cmd
    if cmd is None:
      cmd = command_trace[-1] if command_trace else -0.12
    command_trace.append(float(cmd))

  last_output = command_trace[-1] if command_trace else -0.12
  output_trace = [float(last_output)]
  times = [float(samples[start].t)]
  predicted = [a_ego]
  predicted_v = [v_ego]
  predicted_distance_m = [0.0]
  replay_sample_indices = [start]
  rollout_total_m = 0.0
  rollout_from_2mps_m = 0.0

  for _ in range(start, hold):
    output_seed = min(last_output, -0.1)
    max_expected = interp(v_ego, v_bp, max_accel_bp)
    min_expected = interp(v_ego, v_bp, min_accel_bp)
    output_cmd = controller.update(
      output_accel=output_seed,
      v_ego=v_ego,
      a_ego=a_ego,
      max_expected_accel=max_expected,
      min_expected_accel=min_expected,
      stop_accel=stop_accel,
      stopping_speed_breakpoint=stopping_speed_breakpoint,
      stopping_error_factor=stopping_error_factor,
      dt=dt,
    )
    command_trace.append(output_cmd)
    output_trace.append(output_cmd)
    delayed_idx = max(0, len(command_trace) - 1 - model.delay_frames)
    delayed_cmd = float(command_trace[delayed_idx])

    a_next = float(clip(model.predict_next(a_ego, delayed_cmd, v_ego), -4.0, 3.0))
    prev_v = v_ego
    v_ego = max(0.0, v_ego + (a_next * dt))
    step_dist = max(0.0, 0.5 * (prev_v + v_ego) * dt)
    rollout_total_m += step_dist
    if prev_v <= 2.0 and v_ego <= 2.0:
      rollout_from_2mps_m += step_dist
    elif prev_v > 2.0 >= v_ego:
      below_fraction = clip((2.0 - v_ego) / max(prev_v - v_ego, 1e-6), 0.0, 1.0)
      rollout_from_2mps_m += step_dist * below_fraction

    a_ego = a_next
    last_output = output_cmd
    predicted.append(a_ego)
    predicted_v.append(v_ego)
    predicted_distance_m.append(predicted_distance_m[-1] + step_dist)
    times.append(times[-1] + dt)
    replay_sample_indices.append(min(idx + 1, hold))

  hold_time_s = infer_hold_time_s(times, predicted_v)
  entry_time_s = float(times[0]) if times else None
  pred_jerk, pred_min_a = jerk_window_metrics(times, predicted, hold_time_s, predicted_v=predicted_v)
  pred_cmd_jerk, pred_accel_step = compute_end_stop_sharpness_metrics(
    times=times,
    predicted_a=predicted,
    hold_time_s=hold_time_s,
    predicted_cmd=output_trace,
  )
  pred_lead_entry, pred_lead_hold, recorded_lead_hold = compute_predicted_lead_distance_metrics(
    samples=samples,
    replay_sample_indices=replay_sample_indices,
    times=times,
    predicted_v=predicted_v,
    predicted_distance_m=predicted_distance_m,
    entry_time_s=entry_time_s,
    hold_time_s=hold_time_s,
  )
  stop_idx: int | None = None
  for idx, v in enumerate(predicted_v):
    if v < 0.05:
      stop_idx = idx
      break
  if stop_idx is not None and stop_idx > 0 and stop_idx - 1 < len(output_trace):
    standstill_cmd_jerk = abs(float(output_trace[stop_idx - 1])) / 0.40
    pred_cmd_jerk = standstill_cmd_jerk if pred_cmd_jerk is None else max(pred_cmd_jerk, standstill_cmd_jerk)
    pred_jerk = standstill_cmd_jerk if pred_jerk is None else max(pred_jerk, standstill_cmd_jerk)

  pred_rebound, pred_unexpected_accel = compute_pred_leapfrog_metrics(
    predicted_v=predicted_v,
    predicted_a=predicted,
    max_accel_v_bp=v_bp,
    max_accel_bp=max_accel_bp,
  )
  return {
    "pred_end_stop_jerk_mps3": pred_jerk,
    "pred_end_stop_cmd_jerk_mps3": pred_cmd_jerk,
    "pred_end_stop_accel_step_mps2": pred_accel_step,
    "pred_min_a_ego_mps2": pred_min_a,
    "pred_rollout_distance_m": rollout_total_m,
    "pred_rollout_from_2mps_m": rollout_from_2mps_m,
    "pred_lead_distance_stop_entry_m": pred_lead_entry,
    "pred_lead_distance_hold_m": pred_lead_hold,
    "recorded_lead_distance_hold_m": recorded_lead_hold,
    "pred_speed_rebound_while_should_stop_mps": pred_rebound,
    "pred_should_stop_unexpected_accel_mps2": pred_unexpected_accel,
  }


def main() -> int:
  args = parse_args()
  model_payload = load_json(Path(args.model_json).expanduser())
  model_data = model_payload["model"] if "model" in model_payload else model_payload
  model = FittedStoppingModel.from_json(model_data)
  selector_payload: dict[str, Any] | None = None
  selector: PrototypeStoppingProfileSelector | None = None
  if args.profile_selector_json:
    selector_payload = load_json(Path(args.profile_selector_json).expanduser())
    selector = PrototypeStoppingProfileSelector.from_json(selector_payload)

  summary_paths = [Path(item).expanduser() for item in args.summary_json]
  download_root = Path(args.download_root).expanduser()

  sample_cache: dict[tuple[str, str], list[Any]] = {}
  segment_cache: dict[str, list[Any]] = {}
  rows: list[dict[str, Any]] = []
  include_routes = {str(route) for route in args.include_route}
  exclude_routes = {str(route) for route in args.exclude_route}

  for summary_path in summary_paths:
    for summary in iter_route_summaries(summary_path):
      host = str(summary.get("host", "commawifi"))
      route = str(summary.get("route", ""))
      if not route:
        continue
      if include_routes and route not in include_routes:
        continue
      if route in exclude_routes:
        continue
      samples = route_samples(sample_cache, segment_cache, download_root, host, route)
      times = [float(item.t) for item in samples]

      for event in summary.get("events", []):
        if not isinstance(event, dict):
          continue
        source = str(event.get("event_source", ""))
        if args.event_source != "all" and source != args.event_source:
          continue
        entry_speed = float(event.get("entry_speed_mps", 0.0))
        if entry_speed < args.min_entry_speed:
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
        should_stop_span = last_contiguous_index_span(samples, start_idx, hold_idx, lambda item: item.should_stop)
        stopping_span = last_contiguous_index_span(
          samples,
          start_idx,
          hold_idx,
          lambda item: item.long_state == "stopping" or item.long_state_cmd == "stopping",
        )
        should_stop_start = should_stop_span[0] if should_stop_span is not None else None
        should_stop_end = should_stop_span[1] if should_stop_span is not None else None
        stopping_start = stopping_span[0] if stopping_span is not None else None
        stopping_end = stopping_span[1] if stopping_span is not None else None

        if args.controller_window_mode == "should_stop":
          if should_stop_start is None:
            continue
          sim_start_idx = should_stop_start
        elif args.controller_window_mode == "stopping_state":
          if stopping_start is None:
            continue
          sim_start_idx = stopping_start

        if args.controller_end_mode == "last_should_stop":
          if should_stop_end is None:
            continue
          sim_hold_idx = min(sim_hold_idx, should_stop_end)
        elif args.controller_end_mode == "last_stopping_state":
          if stopping_end is None:
            continue
          sim_hold_idx = min(sim_hold_idx, stopping_end)

        if sim_hold_idx <= sim_start_idx:
          continue

        en_ratio = enabled_ratio(samples, sim_start_idx, sim_hold_idx)
        stopping_present = stopping_start is not None and stopping_end is not None
        if args.controller_scope in ("engaged", "engaged_stopping") and en_ratio < args.controller_min_enabled_ratio:
          continue
        if args.controller_scope == "engaged_stopping" and not stopping_present:
          continue

        current = simulate_event_with_controller(
          samples=samples,
          start_idx=sim_start_idx,
          hold_idx=sim_hold_idx,
          model=model,
          stopping_speed_breakpoint=args.stopping_speed_breakpoint,
          stop_accel=args.stop_accel,
          return_trace=True,
        )
        optimizer_config = HorizonOptimizerConfig(
          horizon_s=args.horizon_v1_horizon_s,
          block_s=args.horizon_v1_block_s,
          beam_width=args.horizon_v1_beam_width,
          residual_grid_mps2=args.horizon_v1_residual_grid,
          max_pred_end_jerk=args.max_pred_end_jerk,
          max_pred_end_cmd_jerk=args.max_pred_end_cmd_jerk,
          max_pred_end_accel_step=args.max_pred_end_accel_step,
          max_pred_rollout_m=args.max_pred_rollout_m,
          min_pred_lead_hold_distance_m=args.min_pred_lead_hold_distance_m,
          max_pred_lead_hold_distance_m=args.max_pred_lead_hold_distance_m,
          max_pred_speed_rebound_while_should_stop=args.max_pred_speed_rebound_while_should_stop,
          max_pred_should_stop_unexpected_accel=args.max_pred_should_stop_unexpected_accel,
        )
        horizon_v1 = simulate_event_with_horizon_v1_controller(
          samples=samples,
          current_replay=current,
          model=model,
          config=optimizer_config,
          return_trace=True,
        )
        legacy = simulate_event_with_legacy_controller(
          samples=samples,
          start_idx=sim_start_idx,
          hold_idx=sim_hold_idx,
          model=model,
          stop_accel=args.stop_accel,
          stopping_speed_breakpoint=args.stopping_speed_breakpoint,
          stopping_error_factor=args.stopping_error_factor,
        )

        m_cur = classify(current, args)
        m_horizon = classify(horizon_v1, args)
        m_leg = classify(legacy, args)
        horizon_teacher = summarize_horizon_teacher(
          current=current,
          horizon_v1=horizon_v1,
          current_score=m_cur.event_score,
          horizon_score=m_horizon.event_score,
        )
        selector_features = selector_features_from_replay(samples, current, horizon_v1)
        selector_label = profile_label_from_horizon_teacher(horizon_teacher)
        selector_residual_target = selector_residual_target_from_replay(current, horizon_v1)
        profile_selector_row: dict[str, Any] | None = None
        if selector is not None and selector_payload is not None and selector_features:
          selector_decision = selector.select(StoppingProfileFeatures.from_mapping(selector_features))
          selected_profile = selector_decision.profile
          exemplar_distance = nearest_exemplar_distance_for_profile(selector_payload, selected_profile, selector_features)
          exemplar_supported = exemplar_distance is not None and exemplar_distance <= args.profile_selector_max_exemplar_distance
          evaluated_profile_count = 1
          if args.profile_selector_mode == "oracle":
            candidates: list[tuple[float, str, list[float], VariantMetrics, float | None, bool]] = [
              (m_cur.event_score, "no_change", [0.0, 0.0, 0.0], m_cur, None, True),
            ]
            for profile_item in selector_payload.get("profiles", []):
              if not isinstance(profile_item, dict):
                continue
              candidate_profile = str(profile_item.get("profile", ""))
              if candidate_profile == "no_change":
                continue
              candidate_exemplar_distance = nearest_exemplar_distance_for_profile(selector_payload, candidate_profile, selector_features)
              candidate_exemplar_supported = (
                candidate_exemplar_distance is not None
                and candidate_exemplar_distance <= args.profile_selector_max_exemplar_distance
              )
              if args.profile_selector_require_exemplar and not candidate_exemplar_supported:
                continue
              candidate_template = residual_template_for_profile(
                selector_payload,
                candidate_profile,
                selector_features,
                max_exemplar_distance=args.profile_selector_max_exemplar_distance,
              )
              candidate_replay = simulate_event_with_residual_profile(
                samples=samples,
                current_replay=current,
                model=model,
                residual_template_mps2=candidate_template,
                search_start_step=int(horizon_v1.get("optimizer_search_start_step", 0)),
                config=optimizer_config,
                return_trace=False,
              )
              candidate_metrics = classify(candidate_replay, args)
              if (candidate_metrics.is_harsh and not m_cur.is_harsh) or (candidate_metrics.is_leapfrog and not m_cur.is_leapfrog):
                continue
              candidates.append((
                candidate_metrics.event_score,
                candidate_profile,
                candidate_template,
                candidate_metrics,
                candidate_exemplar_distance,
                candidate_exemplar_supported,
              ))
            evaluated_profile_count = len(candidates)
            _score, selected_profile, residual_template, m_selector, exemplar_distance, exemplar_supported = min(candidates, key=lambda item: item[0])
          else:
            if selector_decision.confidence < args.profile_selector_min_confidence:
              selected_profile = "no_change"
            if args.profile_selector_require_exemplar and selected_profile != "no_change" and not exemplar_supported:
              selected_profile = "no_change"
            residual_template = residual_template_for_profile(
              selector_payload,
              selected_profile,
              selector_features,
              max_exemplar_distance=args.profile_selector_max_exemplar_distance,
            )
            profile_selector = simulate_event_with_residual_profile(
              samples=samples,
              current_replay=current,
              model=model,
              residual_template_mps2=residual_template,
              search_start_step=int(horizon_v1.get("optimizer_search_start_step", 0)),
              config=optimizer_config,
              return_trace=False,
            )
            m_selector = classify(profile_selector, args)
          profile_selector_row = {
            "profile": selected_profile,
            "raw_profile": selector_decision.profile,
            "mode": args.profile_selector_mode,
            "confidence": selector_decision.confidence,
            "distance": selector_decision.distance,
            "second_distance": selector_decision.second_distance,
            "exemplar_distance": exemplar_distance,
            "exemplar_supported": exemplar_supported,
            "evaluated_profile_count": evaluated_profile_count,
            "residual_template_mps2": residual_template,
            "harsh": m_selector.is_harsh,
            "flags": m_selector.flags,
            "leapfrog": m_selector.is_leapfrog,
            "leapfrog_flags": m_selector.leapfrog_flags,
            "score": m_selector.event_score,
            "pred_end_stop_jerk_mps3": m_selector.pred_end_stop_jerk_mps3,
            "pred_end_stop_cmd_jerk_mps3": m_selector.pred_end_stop_cmd_jerk_mps3,
            "pred_end_stop_accel_step_mps2": m_selector.pred_end_stop_accel_step_mps2,
            "pred_min_a_ego_mps2": m_selector.pred_min_a_ego_mps2,
            "pred_rollout_distance_m": m_selector.pred_rollout_distance_m,
            "pred_lead_distance_hold_m": m_selector.pred_lead_distance_hold_m,
            "recorded_lead_distance_hold_m": m_selector.recorded_lead_distance_hold_m,
            "distance_gate_source": m_selector.distance_gate_source,
            "pred_speed_rebound_while_should_stop_mps": m_selector.pred_speed_rebound_while_should_stop_mps,
            "pred_should_stop_unexpected_accel_mps2": m_selector.pred_should_stop_unexpected_accel_mps2,
          }

        row_payload = {
          "summary_json": str(summary_path),
          "route": route,
          "event_id": event.get("event_id"),
          "event_source": source,
          "entry_speed_mps": entry_speed,
          "enabled_ratio": en_ratio,
          "selector_features": selector_features,
          "selector_label": selector_label,
          "selector_residual_target": selector_residual_target,
          "current": {
            "harsh": m_cur.is_harsh,
            "flags": m_cur.flags,
            "leapfrog": m_cur.is_leapfrog,
            "leapfrog_flags": m_cur.leapfrog_flags,
            "score": m_cur.event_score,
            "pred_end_stop_jerk_mps3": m_cur.pred_end_stop_jerk_mps3,
            "pred_end_stop_cmd_jerk_mps3": m_cur.pred_end_stop_cmd_jerk_mps3,
            "pred_end_stop_accel_step_mps2": m_cur.pred_end_stop_accel_step_mps2,
            "pred_min_a_ego_mps2": m_cur.pred_min_a_ego_mps2,
            "pred_rollout_distance_m": m_cur.pred_rollout_distance_m,
            "pred_lead_distance_hold_m": m_cur.pred_lead_distance_hold_m,
            "recorded_lead_distance_hold_m": m_cur.recorded_lead_distance_hold_m,
            "distance_gate_source": m_cur.distance_gate_source,
            "pred_speed_rebound_while_should_stop_mps": m_cur.pred_speed_rebound_while_should_stop_mps,
            "pred_should_stop_unexpected_accel_mps2": m_cur.pred_should_stop_unexpected_accel_mps2,
          },
          "horizon_v1": {
            "harsh": m_horizon.is_harsh,
            "flags": m_horizon.flags,
            "leapfrog": m_horizon.is_leapfrog,
            "leapfrog_flags": m_horizon.leapfrog_flags,
            "score": m_horizon.event_score,
            "pred_end_stop_jerk_mps3": m_horizon.pred_end_stop_jerk_mps3,
            "pred_end_stop_cmd_jerk_mps3": m_horizon.pred_end_stop_cmd_jerk_mps3,
            "pred_end_stop_accel_step_mps2": m_horizon.pred_end_stop_accel_step_mps2,
            "pred_min_a_ego_mps2": m_horizon.pred_min_a_ego_mps2,
            "pred_rollout_distance_m": m_horizon.pred_rollout_distance_m,
            "pred_lead_distance_hold_m": m_horizon.pred_lead_distance_hold_m,
            "recorded_lead_distance_hold_m": m_horizon.recorded_lead_distance_hold_m,
            "distance_gate_source": m_horizon.distance_gate_source,
            "pred_speed_rebound_while_should_stop_mps": m_horizon.pred_speed_rebound_while_should_stop_mps,
            "pred_should_stop_unexpected_accel_mps2": m_horizon.pred_should_stop_unexpected_accel_mps2,
          },
          "horizon_teacher": horizon_teacher,
          "legacy_32b8be": {
            "harsh": m_leg.is_harsh,
            "flags": m_leg.flags,
            "leapfrog": m_leg.is_leapfrog,
            "leapfrog_flags": m_leg.leapfrog_flags,
            "score": m_leg.event_score,
            "pred_end_stop_jerk_mps3": m_leg.pred_end_stop_jerk_mps3,
            "pred_end_stop_cmd_jerk_mps3": m_leg.pred_end_stop_cmd_jerk_mps3,
            "pred_end_stop_accel_step_mps2": m_leg.pred_end_stop_accel_step_mps2,
            "pred_min_a_ego_mps2": m_leg.pred_min_a_ego_mps2,
            "pred_rollout_distance_m": m_leg.pred_rollout_distance_m,
            "pred_lead_distance_hold_m": m_leg.pred_lead_distance_hold_m,
            "recorded_lead_distance_hold_m": m_leg.recorded_lead_distance_hold_m,
            "distance_gate_source": m_leg.distance_gate_source,
            "pred_speed_rebound_while_should_stop_mps": m_leg.pred_speed_rebound_while_should_stop_mps,
            "pred_should_stop_unexpected_accel_mps2": m_leg.pred_should_stop_unexpected_accel_mps2,
          },
        }
        if profile_selector_row is not None:
          row_payload["profile_selector"] = profile_selector_row
        rows.append(row_payload)

  current_harsh = sum(1 for row in rows if row["current"]["harsh"])
  horizon_v1_harsh = sum(1 for row in rows if row["horizon_v1"]["harsh"])
  legacy_harsh = sum(1 for row in rows if row["legacy_32b8be"]["harsh"])
  current_leapfrog = sum(1 for row in rows if row["current"]["leapfrog"])
  horizon_v1_leapfrog = sum(1 for row in rows if row["horizon_v1"]["leapfrog"])
  legacy_leapfrog = sum(1 for row in rows if row["legacy_32b8be"]["leapfrog"])
  n = len(rows)
  current_rate = (current_harsh / n) if n else 0.0
  horizon_v1_rate = (horizon_v1_harsh / n) if n else 0.0
  legacy_rate = (legacy_harsh / n) if n else 0.0
  current_leapfrog_rate = (current_leapfrog / n) if n else 0.0
  horizon_v1_leapfrog_rate = (horizon_v1_leapfrog / n) if n else 0.0
  legacy_leapfrog_rate = (legacy_leapfrog / n) if n else 0.0
  current_avg = (sum(row["current"]["score"] for row in rows) / n) if n else 0.0
  horizon_v1_avg = (sum(row["horizon_v1"]["score"] for row in rows) / n) if n else 0.0
  legacy_avg = (sum(row["legacy_32b8be"]["score"] for row in rows) / n) if n else 0.0

  horizon_v1_improved = sum(1 for row in rows if row["horizon_v1"]["score"] < row["current"]["score"] - 1e-6)
  horizon_v1_worsened = sum(1 for row in rows if row["horizon_v1"]["score"] > row["current"]["score"] + 1e-6)
  selector_rows = [row for row in rows if isinstance(row.get("profile_selector"), dict)]
  selector_n = len(selector_rows)
  selector_harsh = sum(1 for row in selector_rows if row["profile_selector"]["harsh"])
  selector_leapfrog = sum(1 for row in selector_rows if row["profile_selector"]["leapfrog"])
  selector_rate = (selector_harsh / selector_n) if selector_n else 0.0
  selector_leapfrog_rate = (selector_leapfrog / selector_n) if selector_n else 0.0
  selector_avg = (sum(row["profile_selector"]["score"] for row in selector_rows) / selector_n) if selector_n else 0.0
  selector_improved = sum(
    1 for row in selector_rows
    if row["profile_selector"]["score"] < row["current"]["score"] - 1e-6
  )
  selector_worsened = sum(
    1 for row in selector_rows
    if row["profile_selector"]["score"] > row["current"]["score"] + 1e-6
  )

  result = {
    "generated_utc": datetime.now(UTC).replace(microsecond=0).isoformat(),
    "events_considered": n,
    "current": {
      "harsh_events": current_harsh,
      "harsh_rate": current_rate,
      "leapfrog_events": current_leapfrog,
      "leapfrog_rate": current_leapfrog_rate,
      "avg_event_score": current_avg,
    },
    "horizon_v1": {
      "harsh_events": horizon_v1_harsh,
      "harsh_rate": horizon_v1_rate,
      "leapfrog_events": horizon_v1_leapfrog,
      "leapfrog_rate": horizon_v1_leapfrog_rate,
      "avg_event_score": horizon_v1_avg,
    },
    "legacy_32b8be": {
      "harsh_events": legacy_harsh,
      "harsh_rate": legacy_rate,
      "leapfrog_events": legacy_leapfrog,
      "leapfrog_rate": legacy_leapfrog_rate,
      "avg_event_score": legacy_avg,
    },
    "comparison": {
      "improved_events": horizon_v1_improved,
      "worsened_events": horizon_v1_worsened,
      "horizon_v1_improved_events": horizon_v1_improved,
      "horizon_v1_worsened_events": horizon_v1_worsened,
      "profile_selector_improved_events": selector_improved,
      "profile_selector_worsened_events": selector_worsened,
    },
    "horizon_teacher_summary": aggregate_horizon_teacher(rows),
    "route_filters": {
      "include_routes": sorted(include_routes),
      "exclude_routes": sorted(exclude_routes),
    },
    "event_rows": rows,
  }
  if selector_n:
    result["profile_selector"] = {
      "events": selector_n,
      "harsh_events": selector_harsh,
      "harsh_rate": selector_rate,
      "leapfrog_events": selector_leapfrog,
      "leapfrog_rate": selector_leapfrog_rate,
      "avg_event_score": selector_avg,
    }

  print(f"[benchmark] events={n}")
  print(
    f"[benchmark] current harsh={current_harsh}/{n} rate={current_rate:.3f}"
    + f" leapfrog={current_leapfrog}/{n} leapfrog_rate={current_leapfrog_rate:.3f} avg_score={current_avg:.3f}"
  )
  print(
    f"[benchmark] horizon_v1 harsh={horizon_v1_harsh}/{n} rate={horizon_v1_rate:.3f}"
    + f" leapfrog={horizon_v1_leapfrog}/{n} leapfrog_rate={horizon_v1_leapfrog_rate:.3f} avg_score={horizon_v1_avg:.3f}"
  )
  print(
    f"[benchmark] legacy_32b8be harsh={legacy_harsh}/{n} rate={legacy_rate:.3f}"
    + f" leapfrog={legacy_leapfrog}/{n} leapfrog_rate={legacy_leapfrog_rate:.3f} avg_score={legacy_avg:.3f}"
  )
  if selector_n:
    print(
      f"[benchmark] profile_selector harsh={selector_harsh}/{selector_n} rate={selector_rate:.3f}"
      + f" leapfrog={selector_leapfrog}/{selector_n} leapfrog_rate={selector_leapfrog_rate:.3f} avg_score={selector_avg:.3f}"
    )
  print(
    f"[benchmark] improved={horizon_v1_improved} worsened={horizon_v1_worsened}"
    + f" horizon_v1_improved={horizon_v1_improved} horizon_v1_worsened={horizon_v1_worsened}"
    + f" profile_selector_improved={selector_improved} profile_selector_worsened={selector_worsened}"
  )

  if args.output_json:
    out = Path(args.output_json).expanduser()
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n")
    print(f"[benchmark] output_json={out}")

  return 0


if __name__ == "__main__":
  raise SystemExit(main())
