from __future__ import annotations

from dataclasses import dataclass
from math import ceil
from typing import Any

from openpilot.common.numpy_fast import clip
from openpilot.tools.stopping.check_harsh_stops_model import (
  DEFAULT_MAX_PRED_LEAD_HOLD_DISTANCE_M,
  DEFAULT_MIN_PRED_LEAD_HOLD_DISTANCE_M,
  STANDSTILL_CMD_JERK_TAU_S,
  STANDSTILL_SPEED_MPS,
  compute_end_stop_sharpness_metrics,
  compute_entry_stop_sharpness_metrics,
  compute_predicted_lead_distance_metrics,
  compute_pred_leapfrog_metrics,
  infer_hold_time_s,
  jerk_window_metrics,
  score_event_metrics,
)
from openpilot.tools.stopping.stopping_model import FittedStoppingModel


@dataclass(frozen=True)
class HorizonOptimizerConfig:
  horizon_s: float = 1.2
  block_s: float = 0.10
  beam_width: int = 24
  residual_grid_mps2: tuple[float, ...] = (-0.12, -0.06, 0.0, 0.06, 0.12)
  min_cmd: float = -3.0
  max_cmd: float = 1.0
  max_pred_end_jerk: float = 0.70
  max_pred_end_cmd_jerk: float = 3.0
  max_pred_end_accel_step: float = 0.08
  max_pred_rollout_m: float = 2.0
  min_pred_lead_hold_distance_m: float = DEFAULT_MIN_PRED_LEAD_HOLD_DISTANCE_M
  max_pred_lead_hold_distance_m: float = DEFAULT_MAX_PRED_LEAD_HOLD_DISTANCE_M
  max_pred_speed_rebound_while_should_stop: float = 0.08
  max_pred_should_stop_unexpected_accel: float = 0.10


def _rollout_from_2mps(predicted_v: list[float], dt_s: float) -> float:
  rollout_from_2mps_m = 0.0
  for prev_v, cur_v in zip(predicted_v, predicted_v[1:], strict=False):
    prev_v_ego = max(0.0, float(prev_v))
    cur_v_ego = max(0.0, float(cur_v))
    step_distance_m = max(0.0, 0.5 * (prev_v_ego + cur_v_ego) * dt_s)
    if prev_v_ego <= 2.0 and cur_v_ego <= 2.0:
      rollout_from_2mps_m += step_distance_m
    elif prev_v_ego > 2.0 >= cur_v_ego:
      below_fraction = clip((2.0 - cur_v_ego) / max(prev_v_ego - cur_v_ego, 1e-6), 0.0, 1.0)
      rollout_from_2mps_m += step_distance_m * below_fraction
  return float(rollout_from_2mps_m)


def _expanded_step_residuals(
  residual_blocks: list[float],
  search_steps: int,
  block_steps: int,
) -> list[float]:
  residuals: list[float] = []
  for residual in residual_blocks:
    residuals.extend([float(residual)] * block_steps)
  return residuals[:search_steps]


def expanded_profile_residuals(residual_template: list[float], search_steps: int) -> list[float]:
  if search_steps <= 0:
    return []
  if not residual_template:
    return [0.0] * search_steps
  return [
    float(residual_template[min(len(residual_template) - 1, (idx * len(residual_template)) // search_steps)])
    for idx in range(search_steps)
  ]


def _evaluate_rollout_cost(
  result: dict[str, Any],
  predicted_a: list[float],
  output_trace: list[float],
  entry_time_s: float | None,
  config: HorizonOptimizerConfig,
) -> float:
  pred_rollout_raw = result.get("pred_rollout_from_2mps_m")
  pred_rollout = float(pred_rollout_raw if pred_rollout_raw is not None else result.get("pred_rollout_distance_m", 0.0))
  pred_lead_hold = result.get("pred_lead_distance_hold_m")
  pred_lead_hold_value = float(pred_lead_hold) if pred_lead_hold is not None else None
  recorded_lead_hold = result.get("recorded_lead_distance_hold_m")
  recorded_lead_hold_value = float(recorded_lead_hold) if recorded_lead_hold is not None else None
  pred_cmd_jerk = result.get("pred_end_stop_cmd_jerk_mps3")
  pred_accel_step = result.get("pred_end_stop_accel_step_mps2")
  pred_rebound = result.get("pred_speed_rebound_while_should_stop_mps")
  pred_unexpected_accel = result.get("pred_should_stop_unexpected_accel_mps2")

  base_score = score_event_metrics(
    result.get("pred_end_stop_jerk_mps3"),
    float(result["pred_min_a_ego_mps2"]),
    pred_rollout,
    config.max_pred_rollout_m,
    pred_lead_hold_m=pred_lead_hold_value,
    recorded_lead_hold_m=recorded_lead_hold_value,
    min_lead_hold_m=config.min_pred_lead_hold_distance_m,
    max_lead_hold_m=config.max_pred_lead_hold_distance_m,
    pred_cmd_jerk=float(pred_cmd_jerk) if pred_cmd_jerk is not None else None,
    max_cmd_jerk=config.max_pred_end_cmd_jerk,
    pred_accel_step=float(pred_accel_step) if pred_accel_step is not None else None,
    max_accel_step=config.max_pred_end_accel_step,
  )

  entry_jerk, entry_accel_step, entry_cmd_jerk, _entry_cmd_step = compute_entry_stop_sharpness_metrics(
    times=[float(value) for value in result["times"]],
    predicted_a=predicted_a,
    entry_time_s=entry_time_s,
    predicted_cmd=output_trace,
  )
  entry_cost = 0.0
  if entry_jerk is not None:
    entry_cost += 0.35 * (float(entry_jerk) / max(config.max_pred_end_jerk, 1e-6))
  if entry_accel_step is not None:
    entry_cost += 0.45 * (float(entry_accel_step) / max(config.max_pred_end_accel_step, 1e-6))
  if entry_cmd_jerk is not None:
    entry_cost += 0.15 * (float(entry_cmd_jerk) / max(config.max_pred_end_cmd_jerk, 1e-6))

  leapfrog_cost = 0.0
  if pred_rebound is not None:
    leapfrog_cost += (
      3.0 * max(0.0, float(pred_rebound) - config.max_pred_speed_rebound_while_should_stop)
      / max(config.max_pred_speed_rebound_while_should_stop, 1e-6)
    )
  if pred_unexpected_accel is not None:
    leapfrog_cost += (
      2.0 * max(0.0, float(pred_unexpected_accel) - config.max_pred_should_stop_unexpected_accel)
      / max(config.max_pred_should_stop_unexpected_accel, 1e-6)
    )

  cmd_delta_avg = 0.0
  if len(output_trace) >= 2:
    cmd_delta_avg = sum(abs(float(cur) - float(prev)) for prev, cur in zip(output_trace, output_trace[1:], strict=False)) / max(len(output_trace) - 1, 1)

  return float(base_score + entry_cost + leapfrog_cost + (0.10 * cmd_delta_avg))


def _build_result(
  *,
  samples: list[Any],
  replay_sample_indices: list[int],
  times: list[float],
  predicted_a: list[float],
  predicted_v: list[float],
  predicted_distance_m: list[float],
  output_trace: list[float],
  should_stop_trace: list[bool],
  max_accel_v_bp: list[float],
  max_accel_bp: list[float],
  entry_time_s: float | None,
  include_trace: bool = False,
) -> dict[str, Any]:
  hold_time_s = infer_hold_time_s(times, predicted_v)
  pred_jerk, pred_min_a = jerk_window_metrics(times, predicted_a, hold_time_s, predicted_v=predicted_v)
  pred_cmd_jerk, pred_accel_step = compute_end_stop_sharpness_metrics(
    times=times,
    predicted_a=predicted_a,
    hold_time_s=hold_time_s,
    predicted_cmd=output_trace,
  )
  stop_idx: int | None = None
  for idx, v in enumerate(predicted_v):
    if float(v) < STANDSTILL_SPEED_MPS:
      stop_idx = idx
      break
  if stop_idx is not None and stop_idx > 0 and STANDSTILL_CMD_JERK_TAU_S > 1e-6 and stop_idx - 1 < len(output_trace):
    standstill_cmd_jerk = abs(float(output_trace[stop_idx - 1])) / STANDSTILL_CMD_JERK_TAU_S
    pred_cmd_jerk = standstill_cmd_jerk if pred_cmd_jerk is None else max(pred_cmd_jerk, standstill_cmd_jerk)
    pred_jerk = standstill_cmd_jerk if pred_jerk is None else max(pred_jerk, standstill_cmd_jerk)
  pred_lead_entry, pred_lead_hold, recorded_lead_hold = compute_predicted_lead_distance_metrics(
    samples=samples,
    replay_sample_indices=replay_sample_indices,
    times=times,
    predicted_v=predicted_v,
    predicted_distance_m=predicted_distance_m,
    entry_time_s=entry_time_s,
    hold_time_s=hold_time_s,
  )
  pred_rebound, pred_unexpected_accel = compute_pred_leapfrog_metrics(
    predicted_v=predicted_v,
    predicted_a=predicted_a,
    max_accel_v_bp=max_accel_v_bp,
    max_accel_bp=max_accel_bp,
    should_stop_mask=should_stop_trace,
  )

  result = {
    "times": [float(value) for value in times],
    "predicted_a_ego": [float(value) for value in predicted_a],
    "predicted_v_ego": [float(value) for value in predicted_v],
    "pred_rollout_distance_m": float(predicted_distance_m[-1]) if predicted_distance_m else 0.0,
    "pred_rollout_from_2mps_m": _rollout_from_2mps(predicted_v, float(times[1] - times[0]) if len(times) >= 2 else 0.0),
    "pred_lead_distance_stop_entry_m": pred_lead_entry,
    "pred_lead_distance_hold_m": pred_lead_hold,
    "recorded_lead_distance_hold_m": recorded_lead_hold,
    "pred_end_stop_jerk_mps3": pred_jerk,
    "pred_end_stop_cmd_jerk_mps3": pred_cmd_jerk,
    "pred_end_stop_accel_step_mps2": pred_accel_step,
    "pred_min_a_ego_mps2": pred_min_a,
    "pred_speed_rebound_while_should_stop_mps": pred_rebound,
    "pred_should_stop_unexpected_accel_mps2": pred_unexpected_accel,
  }
  if include_trace:
    result["trace"] = {
      "times": [float(value) for value in times],
      "predicted_a": [float(value) for value in predicted_a],
      "predicted_v": [float(value) for value in predicted_v],
      "predicted_distance_m": [float(value) for value in predicted_distance_m],
      "output_trace": [float(value) for value in output_trace],
      "replay_sample_indices": [int(value) for value in replay_sample_indices],
      "should_stop_trace": [bool(value) for value in should_stop_trace],
      "entry_time_s": float(entry_time_s) if entry_time_s is not None else None,
      "hold_time_s": float(infer_hold_time_s(times, predicted_v)),
    }
  return result


def simulate_event_with_horizon_v1_controller(
  *,
  samples: list[Any],
  current_replay: dict[str, Any],
  model: FittedStoppingModel,
  config: HorizonOptimizerConfig | None = None,
  return_trace: bool = False,
) -> dict[str, Any]:
  cfg = config or HorizonOptimizerConfig()
  trace = current_replay.get("trace")
  if not isinstance(trace, dict):
    raise ValueError("current_replay must include trace data from simulate_event_with_controller(return_trace=True)")

  output_trace_full = [float(value) for value in trace["output_trace"]]
  baseline_step_commands = output_trace_full[1:]
  step_count = len(baseline_step_commands)
  if step_count <= 0:
    return {key: value for key, value in current_replay.items() if key != "trace"}

  dt_s = max(float(trace["dt_s"]), 1e-3)
  search_steps = min(step_count, max(1, int(round(cfg.horizon_s / dt_s))))
  search_start_step = max(0, step_count - search_steps)
  block_steps = max(1, int(round(cfg.block_s / dt_s)))
  block_count = max(1, int(ceil(search_steps / block_steps)))
  residual_zero_tail = [0.0] * block_count

  command_trace_full = [float(value) for value in trace["command_trace"]]
  prefix_history_len = len(command_trace_full) - step_count
  prefix_command_trace = command_trace_full[:prefix_history_len + search_start_step]
  prefix_output_trace = output_trace_full[:search_start_step + 1]
  predicted_a_full = [float(value) for value in trace["predicted_a"]]
  predicted_v_full = [float(value) for value in trace["predicted_v"]]
  predicted_distance_full = [float(value) for value in trace["predicted_distance_m"]]
  times_full = [float(value) for value in trace["times"]]
  replay_sample_indices_full = [int(value) for value in trace["replay_sample_indices"]]
  should_stop_trace_full = [bool(value) for value in trace["should_stop_trace"]]
  max_accel_v_bp = [float(value) for value in trace["max_accel_v_bp"]]
  max_accel_bp = [float(value) for value in trace["max_accel_bp"]]
  entry_time_s = trace.get("entry_time_s")
  entry_time_value = float(entry_time_s) if entry_time_s is not None else None

  def evaluate_residual_blocks(partial_blocks: list[float]) -> tuple[float, dict[str, Any]]:
    residual_blocks = partial_blocks + residual_zero_tail[len(partial_blocks):]
    residual_steps = _expanded_step_residuals(residual_blocks, search_steps, block_steps)

    command_trace = list(prefix_command_trace)
    output_trace = list(prefix_output_trace)
    predicted_a = list(predicted_a_full[:search_start_step + 1])
    predicted_v = list(predicted_v_full[:search_start_step + 1])
    predicted_distance_m = list(predicted_distance_full[:search_start_step + 1])
    times = list(times_full[:search_start_step + 1])
    replay_sample_indices = list(replay_sample_indices_full[:search_start_step + 1])

    a_ego = float(predicted_a[-1])
    v_ego = float(predicted_v[-1])

    for step_offset, residual in enumerate(residual_steps):
      global_step = search_start_step + step_offset
      sample_idx = replay_sample_indices_full[global_step]
      baseline_cmd = baseline_step_commands[global_step]
      output_cmd = float(clip(float(baseline_cmd) + float(residual), cfg.min_cmd, cfg.max_cmd))
      command_trace.append(output_cmd)
      output_trace.append(output_cmd)
      delayed_idx = max(0, len(command_trace) - 1 - model.delay_frames)
      delayed_cmd = float(command_trace[delayed_idx])
      a_next = float(clip(model.predict_next(a_ego, delayed_cmd, v_ego), -4.0, 3.0))
      prev_v_ego = v_ego
      v_ego = max(0.0, v_ego + (a_next * dt_s))
      step_distance_m = max(0.0, 0.5 * (prev_v_ego + v_ego) * dt_s)
      a_ego = a_next
      predicted_a.append(a_ego)
      predicted_v.append(v_ego)
      predicted_distance_m.append(predicted_distance_m[-1] + step_distance_m)
      times.append(times[-1] + dt_s)
      replay_sample_indices.append(min(sample_idx + 1, int(trace["hold_idx"])))

    result = _build_result(
      samples=samples,
      replay_sample_indices=replay_sample_indices,
      times=times,
      predicted_a=predicted_a,
      predicted_v=predicted_v,
      predicted_distance_m=predicted_distance_m,
      output_trace=output_trace,
      should_stop_trace=should_stop_trace_full,
      max_accel_v_bp=max_accel_v_bp,
      max_accel_bp=max_accel_bp,
      entry_time_s=entry_time_value,
      include_trace=return_trace,
    )
    result["optimizer_cost"] = _evaluate_rollout_cost(result, predicted_a, output_trace, entry_time_value, cfg)
    result["optimizer_search_start_step"] = search_start_step
    result["optimizer_block_count"] = block_count
    return float(result["optimizer_cost"]), result

  beam: list[list[float]] = [[]]
  best_result: dict[str, Any] | None = None
  for _ in range(block_count):
    candidates: list[tuple[float, list[float], dict[str, Any]]] = []
    for partial_blocks in beam:
      for residual in cfg.residual_grid_mps2:
        candidate_blocks = partial_blocks + [float(residual)]
        candidate_cost, candidate_result = evaluate_residual_blocks(candidate_blocks)
        candidates.append((candidate_cost, candidate_blocks, candidate_result))
    candidates.sort(key=lambda item: item[0])
    beam = [candidate_blocks for _cost, candidate_blocks, _result in candidates[:cfg.beam_width]]
    if candidates:
      best_result = candidates[0][2]

  if best_result is None:
    return {key: value for key, value in current_replay.items() if key != "trace"}
  return best_result


def simulate_event_with_residual_profile(
  *,
  samples: list[Any],
  current_replay: dict[str, Any],
  model: FittedStoppingModel,
  residual_template_mps2: list[float],
  search_start_step: int,
  config: HorizonOptimizerConfig | None = None,
  return_trace: bool = False,
) -> dict[str, Any]:
  cfg = config or HorizonOptimizerConfig()
  trace = current_replay.get("trace")
  if not isinstance(trace, dict):
    raise ValueError("current_replay must include trace data from simulate_event_with_controller(return_trace=True)")

  output_trace_full = [float(value) for value in trace["output_trace"]]
  baseline_step_commands = output_trace_full[1:]
  step_count = len(baseline_step_commands)
  if step_count <= 0:
    return {key: value for key, value in current_replay.items() if key != "trace"}

  search_start = max(0, min(int(search_start_step), step_count - 1))
  search_steps = step_count - search_start
  residual_steps = expanded_profile_residuals([float(value) for value in residual_template_mps2], search_steps)

  command_trace_full = [float(value) for value in trace["command_trace"]]
  prefix_history_len = len(command_trace_full) - step_count
  prefix_command_trace = command_trace_full[:prefix_history_len + search_start]
  prefix_output_trace = output_trace_full[:search_start + 1]
  predicted_a_full = [float(value) for value in trace["predicted_a"]]
  predicted_v_full = [float(value) for value in trace["predicted_v"]]
  predicted_distance_full = [float(value) for value in trace["predicted_distance_m"]]
  times_full = [float(value) for value in trace["times"]]
  replay_sample_indices_full = [int(value) for value in trace["replay_sample_indices"]]
  should_stop_trace_full = [bool(value) for value in trace["should_stop_trace"]]
  max_accel_v_bp = [float(value) for value in trace["max_accel_v_bp"]]
  max_accel_bp = [float(value) for value in trace["max_accel_bp"]]
  entry_time_s = trace.get("entry_time_s")
  entry_time_value = float(entry_time_s) if entry_time_s is not None else None

  command_trace = list(prefix_command_trace)
  output_trace = list(prefix_output_trace)
  predicted_a = list(predicted_a_full[:search_start + 1])
  predicted_v = list(predicted_v_full[:search_start + 1])
  predicted_distance_m = list(predicted_distance_full[:search_start + 1])
  times = list(times_full[:search_start + 1])
  replay_sample_indices = list(replay_sample_indices_full[:search_start + 1])

  a_ego = float(predicted_a[-1])
  v_ego = float(predicted_v[-1])
  dt_s = max(float(trace["dt_s"]), 1e-3)

  for step_offset, residual in enumerate(residual_steps):
    global_step = search_start + step_offset
    sample_idx = replay_sample_indices_full[global_step]
    baseline_cmd = baseline_step_commands[global_step]
    output_cmd = float(clip(float(baseline_cmd) + float(residual), cfg.min_cmd, cfg.max_cmd))
    command_trace.append(output_cmd)
    output_trace.append(output_cmd)
    delayed_idx = max(0, len(command_trace) - 1 - model.delay_frames)
    delayed_cmd = float(command_trace[delayed_idx])
    a_next = float(clip(model.predict_next(a_ego, delayed_cmd, v_ego), -4.0, 3.0))
    prev_v_ego = v_ego
    v_ego = max(0.0, v_ego + (a_next * dt_s))
    step_distance_m = max(0.0, 0.5 * (prev_v_ego + v_ego) * dt_s)
    a_ego = a_next
    predicted_a.append(a_ego)
    predicted_v.append(v_ego)
    predicted_distance_m.append(predicted_distance_m[-1] + step_distance_m)
    times.append(times[-1] + dt_s)
    replay_sample_indices.append(min(sample_idx + 1, int(trace["hold_idx"])))

  result = _build_result(
    samples=samples,
    replay_sample_indices=replay_sample_indices,
    times=times,
    predicted_a=predicted_a,
    predicted_v=predicted_v,
    predicted_distance_m=predicted_distance_m,
    output_trace=output_trace,
    should_stop_trace=should_stop_trace_full,
    max_accel_v_bp=max_accel_v_bp,
    max_accel_bp=max_accel_bp,
    entry_time_s=entry_time_value,
    include_trace=return_trace,
  )
  result["profile_residual_template_mps2"] = [float(value) for value in residual_template_mps2]
  result["profile_search_start_step"] = int(search_start)
  return result
