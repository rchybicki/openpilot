#!/usr/bin/env python3
"""Compare stop-controller replays using a fitted stopping-response model."""

from __future__ import annotations

import argparse
import json
import sys
from dataclasses import dataclass
from datetime import UTC, datetime
from pathlib import Path
from typing import Any

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
  sys.path.insert(0, str(REPO_ROOT))

from openpilot.common.numpy_fast import clip, interp
from openpilot.tools.stopping.check_harsh_stops_model import (  # pylint: disable=wrong-import-position
  DEFAULT_DOWNLOAD_ROOT,
  first_index_in_range,
  last_index_in_range,
  jerk_window_metrics,
  load_json,
  nearest_index,
  route_samples,
  score_event_metrics,
  simulate_event_with_controller,
)
from openpilot.tools.stopping.stopping_model import FittedStoppingModel


@dataclass
class VariantMetrics:
  pred_end_stop_jerk_mps3: float | None
  pred_min_a_ego_mps2: float
  pred_rollout_distance_m: float
  event_score: float
  is_harsh: bool
  flags: list[str]


class AbstractStoppingController:
  """Simple model-aware stop controller with feedback + disturbance lock + rollout guard."""

  def __init__(self) -> None:
    self.release_lock_counter = 0
    self.low_speed_rollout_m = 0.0
    self.integral_error = 0.0

  def _update_rollout(self, v_ego: float, dt: float) -> None:
    if v_ego <= 0.02:
      self.low_speed_rollout_m = 0.0
      return
    if v_ego < 1.2:
      self.low_speed_rollout_m += v_ego * dt
    else:
      self.low_speed_rollout_m = max(self.low_speed_rollout_m - (v_ego * dt), 0.0)

  def _update_lock(self, v_ego: float, a_ego: float, last_cmd: float, max_expected_accel: float) -> None:
    disturbance = a_ego - max_expected_accel
    if v_ego < 1.4 and last_cmd < -0.08 and disturbance > 0.05:
      lock_frames = int(interp(v_ego, [0.0, 0.30, 0.80, 1.40], [108, 90, 66, 48]))
      self.release_lock_counter = max(self.release_lock_counter, lock_frames)
    elif self.release_lock_counter > 0:
      self.release_lock_counter -= 1

  def update(
    self,
    last_cmd: float,
    v_ego: float,
    a_ego: float,
    max_expected_accel: float,
    min_expected_accel: float,
    stop_accel: float,
    dt: float,
  ) -> float:
    self._update_rollout(v_ego, dt)
    self._update_lock(v_ego, a_ego, last_cmd, max_expected_accel)

    disturbance = clip(a_ego - max_expected_accel, 0.0, 1.2)

    # Decel reference profile with stronger braking as speed rises.
    a_ref = interp(v_ego, [0.00, 0.08, 0.25, 0.60, 1.20, 2.00], [-0.24, -0.28, -0.36, -0.50, -0.68, -0.86])
    if self.release_lock_counter > 0:
      lock_push = clip(disturbance / 0.45, 0.0, 1.0)
      a_ref -= lock_push * interp(v_ego, [0.00, 0.60, 1.20], [0.10, 0.14, 0.18])

    err = a_ref - a_ego
    self.integral_error = clip(self.integral_error + (err * dt), -1.3, 1.3)
    cmd_target = last_cmd + (0.10 * err) + (0.04 * self.integral_error)

    rollout_trigger = interp(v_ego, [0.02, 0.25, 0.55, 1.20], [0.10, 0.20, 0.35, 0.75])
    rollout_full = interp(v_ego, [0.02, 0.25, 0.55, 1.20], [0.35, 0.60, 0.95, 2.20])
    rollout_tighten = clip((self.low_speed_rollout_m - rollout_trigger) / max(rollout_full - rollout_trigger, 1e-3), 0.0, 1.0)
    if rollout_tighten > 0.0:
      cmd_target -= rollout_tighten * interp(v_ego, [0.02, 0.25, 0.55, 1.20], [0.04, 0.08, 0.12, 0.15])

    rebound = self.release_lock_counter > 0 and self.low_speed_rollout_m > 1.0 and v_ego < 0.9 and a_ego > 0.05
    if rebound:
      rebound_floor = interp(v_ego, [0.00, 0.25, 0.60, 0.90], [-0.70, -0.76, -0.84, -0.92])
      cmd_target = min(cmd_target, rebound_floor)

    if a_ego < (min_expected_accel - 0.10) and v_ego < 0.9:
      # Allow faster release if car is already decelerating harder than expected.
      cmd_target += interp(v_ego, [0.00, 0.20, 0.55, 0.90], [0.050, 0.043, 0.032, 0.020]) * dt

    hold_floor = interp(v_ego, [0.00, 0.03, 0.10, 0.25, 0.60], [-0.26, -0.30, -0.35, -0.44, -0.56])
    cmd_target = min(cmd_target, hold_floor)

    brake_step = interp(v_ego, [0.00, 0.35, 0.90, 1.40], [0.008, 0.012, 0.016, 0.020])
    release_step = interp(v_ego, [0.00, 0.35, 0.90, 1.40], [0.0012, 0.0030, 0.0050, 0.0070])

    if self.release_lock_counter > 0:
      release_step = min(release_step, interp(v_ego, [0.00, 0.35, 0.90, 1.40], [0.0008, 0.0016, 0.0028, 0.0040]))
    if a_ego < -0.95 and v_ego < 0.6:
      release_step = max(release_step, interp(v_ego, [0.00, 0.20, 0.60], [0.014, 0.012, 0.009]))

    dt_scale = clip(dt / 0.01, 0.5, 20.0)
    brake_step *= dt_scale
    release_step *= dt_scale

    cmd = clip(cmd_target, last_cmd - brake_step, last_cmd + release_step)
    return float(clip(cmd, stop_accel, -0.05))


class InverseStoppingController:
  """Model-inversion stop controller (offline-only).

  Uses the fitted response model to choose a command that should produce a desired acceleration profile.
  The intent is to prototype an "inverse model" policy offline and compare it to current + legacy.
  """

  def __init__(
    self,
    model: FittedStoppingModel,
    tau_s: float,
    max_ref_decel: float,
    hold_cmd_cap: float,
    hold_cmd_speed: float,
  ) -> None:
    self.model = model
    self.tau_s = max(float(tau_s), 1e-3)
    self.max_ref_decel = max(float(max_ref_decel), 0.05)
    self.hold_cmd_cap = float(hold_cmd_cap)
    self.hold_cmd_speed = max(float(hold_cmd_speed), 0.0)
    self.integral_error = 0.0

  def _desired_accel(self, v_ego: float) -> float:
    # Smooth reference that ramps decel down as speed approaches zero.
    # a_ref = -min(a_max, v/tau) is simple and stable in the replay model.
    return -min(self.max_ref_decel, max(0.0, float(v_ego)) / self.tau_s)

  def _invert_command(self, a_prev: float, v_ego: float, a_next_des: float) -> float:
    coef = self.model.coefficients
    low_speed = max(0.0, min(1.0, (self.model.low_speed_ref - v_ego) / max(self.model.low_speed_ref, 1e-6)))
    base = (
      coef["intercept"]
      + (coef["a_ego_prev"] * float(a_prev))
      + (coef["v_ego"] * float(v_ego))
      + (coef["low_speed"] * low_speed)
    )
    cmd_k0 = coef["accel_cmd_delayed"] + (coef["cmd_x_low_speed"] * low_speed)
    relief_k = coef["relief"]
    threshold = self.model.relief_cmd_threshold

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

  def update(self, last_cmd: float, v_ego: float, a_ego: float, stop_accel: float, dt: float) -> float:
    # Smith-style compensation: choose commands using a crude speed look-ahead over the fitted delay.
    v_delay = max(0.0, float(v_ego) + (float(a_ego) * float(dt) * float(self.model.delay_frames)))

    a_ref = self._desired_accel(v_delay)
    err = a_ref - float(a_ego)
    self.integral_error = clip(self.integral_error + (err * float(dt)), -2.0, 2.0)
    a_next_des = clip(a_ref + (0.10 * err) + (0.05 * self.integral_error), -3.0, 1.0)

    cmd_target = self._invert_command(a_prev=a_ego, v_ego=v_delay, a_next_des=a_next_des)
    if not (cmd_target == cmd_target):  # NaN
      cmd_target = float(last_cmd)

    # Near-hold cap: keep end-stop command magnitude reasonable to limit standstill command jerk.
    cap = interp(v_ego, [0.0, self.hold_cmd_speed, 0.60], [self.hold_cmd_cap, self.hold_cmd_cap, stop_accel])
    cmd_target = max(cmd_target, cap)

    cmd_target = float(clip(cmd_target, stop_accel, -0.05))

    brake_step = interp(v_ego, [0.00, 0.30, 1.00, 2.00], [0.006, 0.008, 0.010, 0.012])
    release_step = interp(v_ego, [0.00, 0.30, 1.00, 2.00], [0.0015, 0.0030, 0.0045, 0.0060])
    dt_scale = clip(float(dt) / 0.01, 0.5, 20.0)
    brake_step *= dt_scale
    release_step *= dt_scale

    cmd = clip(cmd_target, float(last_cmd) - brake_step, float(last_cmd) + release_step)
    return float(clip(cmd, stop_accel, -0.05))


class LegacyStoppingController32b8be:
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
  parser = argparse.ArgumentParser(description="Compare current replay against an abstract stop-controller replay")
  parser.add_argument("--model-json", required=True)
  parser.add_argument("--summary-json", action="append", required=True)
  parser.add_argument("--download-root", default=str(DEFAULT_DOWNLOAD_ROOT))
  parser.add_argument("--event-source", choices=["all", "signal", "speed", "hybrid"], default="all")
  parser.add_argument("--min-entry-speed", type=float, default=0.20)
  parser.add_argument("--stop-accel", type=float, default=-2.0)
  parser.add_argument("--stopping-speed-breakpoint", type=float, default=0.40)
  parser.add_argument("--stopping-error-factor", type=float, default=1.3)
  parser.add_argument("--inverse-tau-s", type=float, default=0.90, help="Inverse policy time constant for a_ref = -min(a_max, v/tau)")
  parser.add_argument("--inverse-max-ref-decel", type=float, default=1.00, help="Inverse policy max reference decel magnitude (m/s^2)")
  parser.add_argument("--inverse-hold-cmd-cap", type=float, default=-0.25, help="Inverse policy command cap near standstill (min allowed cmd)")
  parser.add_argument("--inverse-hold-cmd-speed", type=float, default=0.06, help="Inverse policy speed below which hold cap applies (m/s)")
  parser.add_argument("--controller-scope", choices=["all", "engaged", "engaged_stopping"], default="engaged_stopping")
  parser.add_argument("--controller-min-enabled-ratio", type=float, default=0.80)
  parser.add_argument("--controller-window-mode", choices=["event", "should_stop", "stopping_state"], default="stopping_state")
  parser.add_argument("--controller-end-mode", choices=["hold", "last_should_stop", "last_stopping_state"], default="last_stopping_state")
  parser.add_argument("--max-pred-end-jerk", type=float, default=0.70)
  parser.add_argument("--min-pred-a-floor", type=float, default=-1.10)
  parser.add_argument("--max-pred-rollout-m", type=float, default=2.0)
  parser.add_argument("--output-json", default=None)
  return parser.parse_args()


def enabled_ratio(samples: list[Any], start_idx: int, end_idx: int) -> float:
  if end_idx < start_idx:
    return 0.0
  count = end_idx - start_idx + 1
  return sum(1.0 if bool(samples[idx].enabled) else 0.0 for idx in range(start_idx, end_idx + 1)) / max(count, 1)


def classify(metrics: dict[str, Any], max_jerk: float, min_a_floor: float, max_rollout_m: float) -> VariantMetrics:
  pred_jerk = metrics["pred_end_stop_jerk_mps3"]
  pred_min_a = float(metrics["pred_min_a_ego_mps2"])
  pred_rollout = float(metrics.get("pred_rollout_from_2mps_m", metrics["pred_rollout_distance_m"]))
  flags: list[str] = []
  if pred_jerk is not None and pred_jerk > max_jerk:
    flags.append("pred_end_stop_jerk")
  if pred_min_a < min_a_floor:
    flags.append("pred_min_a_ego")
  if pred_rollout > max_rollout_m:
    flags.append("pred_rollout")
  return VariantMetrics(
    pred_end_stop_jerk_mps3=pred_jerk,
    pred_min_a_ego_mps2=pred_min_a,
    pred_rollout_distance_m=pred_rollout,
    event_score=score_event_metrics(pred_jerk, pred_min_a, pred_rollout, max_rollout_m),
    is_harsh=bool(flags),
    flags=flags,
  )


def simulate_event_with_abstract_controller(
  samples: list[Any],
  start_idx: int,
  hold_idx: int,
  model: FittedStoppingModel,
  stop_accel: float,
  stopping_speed_breakpoint: float,
) -> dict[str, Any]:
  start = max(0, int(start_idx))
  hold = max(start + 1, min(int(hold_idx), len(samples) - 1))
  if hold <= start:
    raise ValueError("Event window too short for abstract replay")

  mid_bp = clip(stopping_speed_breakpoint, 0.011, 0.499)
  v_bp = [0.01, mid_bp, 0.50]
  max_accel_bp = [-0.01, -0.10, -0.30]
  min_accel_bp = [-0.10, -0.50, -1.00]

  controller = AbstractStoppingController()
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
  rollout_total_m = 0.0
  rollout_from_2mps_m = 0.0

  for _ in range(start, hold):
    max_expected = interp(v_ego, v_bp, max_accel_bp)
    min_expected = interp(v_ego, v_bp, min_accel_bp)
    output_cmd = controller.update(last_output, v_ego, a_ego, max_expected, min_expected, stop_accel, dt)
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
    times.append(times[-1] + dt)

  hold_time_s = times[-1]
  for t, v in zip(times, predicted_v, strict=False):
    if v < 0.05:
      hold_time_s = t
      break
  pred_jerk, pred_min_a = jerk_window_metrics(times, predicted, hold_time_s, predicted_v=predicted_v)
  stop_idx: int | None = None
  for idx, v in enumerate(predicted_v):
    if v < 0.05:
      stop_idx = idx
      break
  if stop_idx is not None and stop_idx > 0 and stop_idx - 1 < len(output_trace):
    standstill_cmd_jerk = abs(float(output_trace[stop_idx - 1])) / 0.40
    pred_jerk = standstill_cmd_jerk if pred_jerk is None else max(pred_jerk, standstill_cmd_jerk)
  return {
    "pred_end_stop_jerk_mps3": pred_jerk,
    "pred_min_a_ego_mps2": pred_min_a,
    "pred_rollout_distance_m": rollout_total_m,
    "pred_rollout_from_2mps_m": rollout_from_2mps_m,
  }


def simulate_event_with_inverse_controller(
  samples: list[Any],
  start_idx: int,
  hold_idx: int,
  model: FittedStoppingModel,
  stop_accel: float,
  tau_s: float,
  max_ref_decel: float,
  hold_cmd_cap: float,
  hold_cmd_speed: float,
) -> dict[str, Any]:
  start = max(0, int(start_idx))
  hold = max(start + 1, min(int(hold_idx), len(samples) - 1))
  if hold <= start:
    raise ValueError("Event window too short for inverse replay")

  controller = InverseStoppingController(
    model=model,
    tau_s=tau_s,
    max_ref_decel=max_ref_decel,
    hold_cmd_cap=hold_cmd_cap,
    hold_cmd_speed=hold_cmd_speed,
  )
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
  rollout_total_m = 0.0
  rollout_from_2mps_m = 0.0

  for _ in range(start, hold):
    output_cmd = controller.update(last_output, v_ego, a_ego, stop_accel, dt)
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
    times.append(times[-1] + dt)

  hold_time_s = times[-1]
  for t, v in zip(times, predicted_v, strict=False):
    if v < 0.05:
      hold_time_s = t
      break
  pred_jerk, pred_min_a = jerk_window_metrics(times, predicted, hold_time_s, predicted_v=predicted_v)
  stop_idx: int | None = None
  for idx, v in enumerate(predicted_v):
    if v < 0.05:
      stop_idx = idx
      break
  if stop_idx is not None and stop_idx > 0 and stop_idx - 1 < len(output_trace):
    standstill_cmd_jerk = abs(float(output_trace[stop_idx - 1])) / 0.40
    pred_jerk = standstill_cmd_jerk if pred_jerk is None else max(pred_jerk, standstill_cmd_jerk)

  return {
    "pred_end_stop_jerk_mps3": pred_jerk,
    "pred_min_a_ego_mps2": pred_min_a,
    "pred_rollout_distance_m": rollout_total_m,
    "pred_rollout_from_2mps_m": rollout_from_2mps_m,
  }


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

  controller = LegacyStoppingController32b8be()
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
    times.append(times[-1] + dt)

  hold_time_s = times[-1]
  for t, v in zip(times, predicted_v, strict=False):
    if v < 0.05:
      hold_time_s = t
      break
  pred_jerk, pred_min_a = jerk_window_metrics(times, predicted, hold_time_s, predicted_v=predicted_v)
  stop_idx: int | None = None
  for idx, v in enumerate(predicted_v):
    if v < 0.05:
      stop_idx = idx
      break
  if stop_idx is not None and stop_idx > 0 and stop_idx - 1 < len(output_trace):
    standstill_cmd_jerk = abs(float(output_trace[stop_idx - 1])) / 0.40
    pred_jerk = standstill_cmd_jerk if pred_jerk is None else max(pred_jerk, standstill_cmd_jerk)
  return {
    "pred_end_stop_jerk_mps3": pred_jerk,
    "pred_min_a_ego_mps2": pred_min_a,
    "pred_rollout_distance_m": rollout_total_m,
    "pred_rollout_from_2mps_m": rollout_from_2mps_m,
  }


def main() -> int:
  args = parse_args()
  model_payload = load_json(Path(args.model_json).expanduser())
  model_data = model_payload["model"] if "model" in model_payload else model_payload
  model = FittedStoppingModel.from_json(model_data)

  summary_paths = [Path(item).expanduser() for item in args.summary_json]
  download_root = Path(args.download_root).expanduser()

  sample_cache: dict[tuple[str, str], list[Any]] = {}
  segment_cache: dict[str, list[Any]] = {}
  rows: list[dict[str, Any]] = []

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
      should_stop_start = first_index_in_range(samples, start_idx, hold_idx, lambda item: item.should_stop)
      should_stop_end = last_index_in_range(samples, start_idx, hold_idx, lambda item: item.should_stop)
      stopping_start = first_index_in_range(samples, start_idx, hold_idx, lambda item: item.long_state == "stopping" or item.long_state_cmd == "stopping")
      stopping_end = last_index_in_range(samples, start_idx, hold_idx, lambda item: item.long_state == "stopping" or item.long_state_cmd == "stopping")

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
      )
      abstract = simulate_event_with_abstract_controller(
        samples=samples,
        start_idx=sim_start_idx,
        hold_idx=sim_hold_idx,
        model=model,
        stop_accel=args.stop_accel,
        stopping_speed_breakpoint=args.stopping_speed_breakpoint,
      )
      inverse = simulate_event_with_inverse_controller(
        samples=samples,
        start_idx=sim_start_idx,
        hold_idx=sim_hold_idx,
        model=model,
        stop_accel=args.stop_accel,
        tau_s=args.inverse_tau_s,
        max_ref_decel=args.inverse_max_ref_decel,
        hold_cmd_cap=args.inverse_hold_cmd_cap,
        hold_cmd_speed=args.inverse_hold_cmd_speed,
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

      m_cur = classify(current, args.max_pred_end_jerk, args.min_pred_a_floor, args.max_pred_rollout_m)
      m_abs = classify(abstract, args.max_pred_end_jerk, args.min_pred_a_floor, args.max_pred_rollout_m)
      m_inv = classify(inverse, args.max_pred_end_jerk, args.min_pred_a_floor, args.max_pred_rollout_m)
      m_leg = classify(legacy, args.max_pred_end_jerk, args.min_pred_a_floor, args.max_pred_rollout_m)

      rows.append({
        "summary_json": str(summary_path),
        "route": route,
        "event_id": event.get("event_id"),
        "event_source": source,
        "entry_speed_mps": entry_speed,
        "enabled_ratio": en_ratio,
        "current": {
          "harsh": m_cur.is_harsh,
          "flags": m_cur.flags,
          "score": m_cur.event_score,
          "pred_end_stop_jerk_mps3": m_cur.pred_end_stop_jerk_mps3,
          "pred_min_a_ego_mps2": m_cur.pred_min_a_ego_mps2,
          "pred_rollout_distance_m": m_cur.pred_rollout_distance_m,
        },
        "abstract": {
          "harsh": m_abs.is_harsh,
          "flags": m_abs.flags,
          "score": m_abs.event_score,
          "pred_end_stop_jerk_mps3": m_abs.pred_end_stop_jerk_mps3,
          "pred_min_a_ego_mps2": m_abs.pred_min_a_ego_mps2,
          "pred_rollout_distance_m": m_abs.pred_rollout_distance_m,
        },
        "inverse": {
          "harsh": m_inv.is_harsh,
          "flags": m_inv.flags,
          "score": m_inv.event_score,
          "pred_end_stop_jerk_mps3": m_inv.pred_end_stop_jerk_mps3,
          "pred_min_a_ego_mps2": m_inv.pred_min_a_ego_mps2,
          "pred_rollout_distance_m": m_inv.pred_rollout_distance_m,
        },
        "legacy_32b8be": {
          "harsh": m_leg.is_harsh,
          "flags": m_leg.flags,
          "score": m_leg.event_score,
          "pred_end_stop_jerk_mps3": m_leg.pred_end_stop_jerk_mps3,
          "pred_min_a_ego_mps2": m_leg.pred_min_a_ego_mps2,
          "pred_rollout_distance_m": m_leg.pred_rollout_distance_m,
        },
      })

  current_harsh = sum(1 for row in rows if row["current"]["harsh"])
  abstract_harsh = sum(1 for row in rows if row["abstract"]["harsh"])
  inverse_harsh = sum(1 for row in rows if row["inverse"]["harsh"])
  legacy_harsh = sum(1 for row in rows if row["legacy_32b8be"]["harsh"])
  n = len(rows)
  current_rate = (current_harsh / n) if n else 0.0
  abstract_rate = (abstract_harsh / n) if n else 0.0
  inverse_rate = (inverse_harsh / n) if n else 0.0
  legacy_rate = (legacy_harsh / n) if n else 0.0
  current_avg = (sum(row["current"]["score"] for row in rows) / n) if n else 0.0
  abstract_avg = (sum(row["abstract"]["score"] for row in rows) / n) if n else 0.0
  inverse_avg = (sum(row["inverse"]["score"] for row in rows) / n) if n else 0.0
  legacy_avg = (sum(row["legacy_32b8be"]["score"] for row in rows) / n) if n else 0.0

  improved = sum(1 for row in rows if row["abstract"]["score"] < row["current"]["score"] - 1e-6)
  worsened = sum(1 for row in rows if row["abstract"]["score"] > row["current"]["score"] + 1e-6)
  inverse_improved = sum(1 for row in rows if row["inverse"]["score"] < row["current"]["score"] - 1e-6)
  inverse_worsened = sum(1 for row in rows if row["inverse"]["score"] > row["current"]["score"] + 1e-6)

  result = {
    "generated_utc": datetime.now(UTC).replace(microsecond=0).isoformat(),
    "events_considered": n,
    "current": {
      "harsh_events": current_harsh,
      "harsh_rate": current_rate,
      "avg_event_score": current_avg,
    },
    "abstract": {
      "harsh_events": abstract_harsh,
      "harsh_rate": abstract_rate,
      "avg_event_score": abstract_avg,
    },
    "inverse": {
      "harsh_events": inverse_harsh,
      "harsh_rate": inverse_rate,
      "avg_event_score": inverse_avg,
    },
    "legacy_32b8be": {
      "harsh_events": legacy_harsh,
      "harsh_rate": legacy_rate,
      "avg_event_score": legacy_avg,
    },
    "comparison": {
      "improved_events": improved,
      "worsened_events": worsened,
      "inverse_improved_events": inverse_improved,
      "inverse_worsened_events": inverse_worsened,
    },
    "event_rows": rows,
  }

  print(f"[benchmark] events={n}")
  print(f"[benchmark] current harsh={current_harsh}/{n} rate={current_rate:.3f} avg_score={current_avg:.3f}")
  print(f"[benchmark] abstract harsh={abstract_harsh}/{n} rate={abstract_rate:.3f} avg_score={abstract_avg:.3f}")
  print(f"[benchmark] inverse harsh={inverse_harsh}/{n} rate={inverse_rate:.3f} avg_score={inverse_avg:.3f}")
  print(f"[benchmark] legacy_32b8be harsh={legacy_harsh}/{n} rate={legacy_rate:.3f} avg_score={legacy_avg:.3f}")
  print(f"[benchmark] improved={improved} worsened={worsened} inverse_improved={inverse_improved} inverse_worsened={inverse_worsened}")

  if args.output_json:
    out = Path(args.output_json).expanduser()
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n")
    print(f"[benchmark] output_json={out}")

  return 0


if __name__ == "__main__":
  raise SystemExit(main())
