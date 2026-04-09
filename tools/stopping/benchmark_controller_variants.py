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
  score_event_metrics,
  simulate_event_with_controller,
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


class _InverseStoppingControllerV3:
  """Maintained inverse replay lane with explicit stop-intent latch and final-stop envelope."""

  def __init__(
    self,
    model: FittedStoppingModel,
    tau_s: float,
    max_ref_decel: float,
    hold_cmd_cap: float,
    hold_cmd_speed: float,
    risk_hold_cmd_cap: float,
    dropout_hold_cmd_cap: float,
    extra_decel_scale: float,
    rollout_floor_scale: float,
    kp: float,
    ki: float,
    step_scale: float,
    brake_step_scale: float,
    release_step_scale: float,
  ) -> None:
    self.model = model
    self.tau_s = max(float(tau_s), 1e-3)
    self.max_ref_decel = max(float(max_ref_decel), 0.05)
    self.hold_cmd_cap = float(hold_cmd_cap)
    self.hold_cmd_speed = max(float(hold_cmd_speed), 0.0)
    self.risk_hold_cmd_cap = float(risk_hold_cmd_cap)
    self.dropout_hold_cmd_cap = float(dropout_hold_cmd_cap)
    self.extra_decel_scale = max(float(extra_decel_scale), 0.0)
    self.rollout_floor_scale = max(float(rollout_floor_scale), 0.0)
    self.kp = float(kp)
    self.ki = float(ki)
    self.step_scale = float(step_scale)
    self.brake_step_scale = float(brake_step_scale)
    self.release_step_scale = float(release_step_scale)
    self.integral_error = 0.0
    self.release_lock_counter = 0
    self.rebound_arrest_counter = 0
    self.intent_hold_counter = 0
    self.prev_should_stop = False
    self.low_speed_rollout_m = 0.0

  def _desired_accel(self, v_ego: float) -> float:
    return -min(self.max_ref_decel, max(0.0, float(v_ego)) / self.tau_s)

  def _invert_command(self, a_prev: float, v_ego: float, a_next_des: float) -> float:
    return invert_command_with_model(self.model, a_prev=a_prev, v_ego=v_ego, a_next_des=a_next_des)

  def _update_rollout(self, v_ego: float, dt: float) -> None:
    if v_ego <= 0.02:
      self.low_speed_rollout_m = max(self.low_speed_rollout_m - (0.35 * dt), 0.0)
    elif v_ego < 1.2:
      self.low_speed_rollout_m += v_ego * dt
    else:
      self.low_speed_rollout_m = max(self.low_speed_rollout_m - (v_ego * dt), 0.0)

  def _remaining_distance_est_m(self, v_ego: float, a_ego: float) -> float:
    decel_mag = max(0.20, -float(a_ego))
    return float(clip((float(v_ego) ** 2) / (2.0 * decel_mag), 0.0, 3.0))

  def _update_release_lock(self, v_ego: float, a_ego: float, last_cmd: float, max_expected_accel: float, dt: float) -> None:
    if self.extra_decel_scale <= 0.0:
      self.release_lock_counter = 0
      return
    disturbance = a_ego - max_expected_accel
    threshold = 0.04 if v_ego < 0.08 else 0.03
    if v_ego > 0.002 and v_ego < 1.2 and last_cmd < -0.05 and disturbance >= threshold:
      lock_frames_100hz = int(interp(v_ego, [0.0, 0.20, 0.60, 1.20], [110, 95, 70, 50]))
      dt_scale = clip(dt / 0.01, 0.5, 20.0)
      lock_steps = max(1, int(lock_frames_100hz / dt_scale))
      self.release_lock_counter = max(self.release_lock_counter, lock_steps)
    elif self.release_lock_counter > 0:
      self.release_lock_counter -= 1

  def _rebound_risk(self, v_ego: float, a_ego: float, last_cmd: float, disturbance: float) -> float:
    if not (0.0 < v_ego < 0.25):
      return 0.0
    speed_factor = clip((0.25 - v_ego) / 0.25, 0.0, 1.0)
    decel_weakness = clip((a_ego + 0.45) / 0.45, 0.0, 1.0)
    disturbance_factor = clip((disturbance - 0.02) / 0.18, 0.0, 1.0)
    rollout_factor = clip((self.low_speed_rollout_m - 0.20) / 0.90, 0.0, 1.0)
    lock_factor = 1.0 if self.release_lock_counter > 0 else 0.0
    cmd_relief = clip((last_cmd + 0.45) / 0.25, 0.0, 1.0)

    risk = (
      (0.30 * speed_factor)
      + (0.26 * decel_weakness)
      + (0.18 * disturbance_factor)
      + (0.14 * rollout_factor)
      + (0.12 * lock_factor)
    )
    risk *= (0.82 + (0.18 * cmd_relief))
    if (a_ego > -0.30) or (disturbance > 0.12) or (self.release_lock_counter > 0 and disturbance > 0.05):
      return clip(risk, 0.0, 1.0)
    return 0.0

  def _update_rebound_arrest(self, v_ego: float, a_ego: float, last_cmd: float, disturbance: float, rebound_risk: float, dt: float) -> None:
    if self.extra_decel_scale <= 0.0:
      self.rebound_arrest_counter = 0
      return
    arrest_trigger = (
      0.0 < v_ego < 0.045
      and last_cmd < -0.22
      and a_ego > -0.24
      and rebound_risk > 0.12
      and self.release_lock_counter > 0
      and disturbance > 0.20
      and self.low_speed_rollout_m > 0.80
    )
    if arrest_trigger:
      base_frames_100hz = interp(v_ego, [0.00, 0.03, 0.08], [48, 40, 28])
      risk_frames_100hz = interp(rebound_risk, [0.38, 1.00], [0.0, 20.0])
      lock_bonus_100hz = 8 if self.release_lock_counter > 0 else 0
      dt_scale = clip(dt / 0.01, 0.5, 20.0)
      steps = max(1, int((base_frames_100hz + risk_frames_100hz + lock_bonus_100hz) / dt_scale))
      self.rebound_arrest_counter = max(self.rebound_arrest_counter, steps)
    elif self.rebound_arrest_counter > 0:
      self.rebound_arrest_counter -= 1

  def _update_intent_latch(self, should_stop: bool, v_ego: float, last_cmd: float, disturbance: float, dt: float) -> None:
    flicker_drop = (
      self.prev_should_stop
      and not should_stop
      and 0.0 < v_ego < 0.30
      and last_cmd < -0.12
    )
    threshold = interp(v_ego, [0.00, 0.08, 0.20], [0.025, 0.030, 0.040])
    disturbance_trigger = (
      0.0 < v_ego < 0.20
      and disturbance > threshold
      and last_cmd < -0.10
    )

    if flicker_drop or disturbance_trigger:
      if flicker_drop:
        latch_frames_100hz = interp(v_ego, [0.00, 0.05, 0.12, 0.30], [56, 46, 34, 22])
      else:
        latch_frames_100hz = interp(v_ego, [0.00, 0.05, 0.12, 0.30], [70, 58, 44, 28])
      dt_scale = clip(dt / 0.01, 0.5, 20.0)
      latch_steps = max(1, int(latch_frames_100hz / dt_scale))
      self.intent_hold_counter = max(self.intent_hold_counter, latch_steps)
    elif self.intent_hold_counter > 0:
      self.intent_hold_counter -= 1
    self.prev_should_stop = bool(should_stop)

  def _baseline_update(
    self,
    last_cmd: float,
    v_ego: float,
    a_ego: float,
    max_expected_accel: float,
    min_expected_accel: float,
    stop_accel: float,
    dt: float,
  ) -> float:
    self._update_release_lock(v_ego, a_ego, last_cmd, max_expected_accel, dt)
    disturbance = clip(a_ego - max_expected_accel, 0.0, 1.2)
    rebound_risk = self._rebound_risk(v_ego, a_ego, last_cmd, disturbance)
    self._update_rebound_arrest(v_ego, a_ego, last_cmd, disturbance, rebound_risk, dt)

    v_delay = max(0.0, float(v_ego) + (float(a_ego) * float(dt) * float(self.model.delay_frames)))
    base_ref = self._desired_accel(v_delay)
    lock_push = clip(float(self.release_lock_counter) / 40.0, 0.0, 1.0)
    disturbance_push = clip((disturbance - 0.01) / 0.30, 0.0, 1.0)
    extra_decel = (
      disturbance_push * interp(v_ego, [0.00, 0.20, 0.60, 1.20], [0.14, 0.12, 0.08, 0.04])
      + lock_push * interp(v_ego, [0.00, 0.20, 0.60, 1.20], [0.08, 0.07, 0.05, 0.03])
      + rebound_risk * interp(v_ego, [0.00, 0.08, 0.25, 0.60], [0.22, 0.19, 0.12, 0.06])
    )
    extra_decel *= self.extra_decel_scale
    a_ref = base_ref - extra_decel

    err = a_ref - float(a_ego)
    self.integral_error = clip(self.integral_error + (err * float(dt)), -2.0, 2.0)
    a_next_des = clip(a_ref + (self.kp * err) + (self.ki * self.integral_error), -3.0, 1.0)

    cmd_target = self._invert_command(a_prev=a_ego, v_ego=v_delay, a_next_des=a_next_des)
    if not (cmd_target == cmd_target):
      cmd_target = float(last_cmd)

    base_cap = interp(v_ego, [0.0, self.hold_cmd_speed, 0.30, 0.60], [self.hold_cmd_cap, self.hold_cmd_cap, -0.22, stop_accel])
    risk_floor = interp(v_ego, [0.00, 0.03, 0.08, 0.25], [self.risk_hold_cmd_cap, -0.40, -0.34, -0.26])
    cap = ((1.0 - rebound_risk) * base_cap) + (rebound_risk * risk_floor)
    if self.release_lock_counter > 0:
      lock_cap = interp(v_ego, [0.00, 0.12, 0.25, 0.50, 1.20], [-0.34, -0.31, -0.26, -0.18, -0.11])
      cap = min(cap, lock_cap)
    if self.rebound_arrest_counter > 0:
      arrest_cap = interp(v_ego, [0.00, 0.03, 0.06, 0.08], [-2.00, -1.60, -1.00, -0.60])
      cap = min(cap, arrest_cap)
    cmd_target = max(cmd_target, cap)

    rollout_trigger = interp(v_ego, [0.02, 0.25, 0.55, 1.20], [0.10, 0.20, 0.35, 0.70])
    rollout_full = interp(v_ego, [0.02, 0.25, 0.55, 1.20], [0.40, 0.65, 1.00, 2.20])
    rollout_tighten = clip((self.low_speed_rollout_m - rollout_trigger) / max(rollout_full - rollout_trigger, 1e-3), 0.0, 1.0)
    if rollout_tighten > 0.05 and a_ego > -0.30:
      push_floor = interp(v_ego, [0.06, 0.20, 0.50, 0.85, 1.20], [-0.50, -0.46, -0.40, -0.34, -0.28])
      cmd_target = min(cmd_target, push_floor)

    cmd_target = float(clip(cmd_target, stop_accel, -0.05))

    brake_step = interp(v_ego, [0.00, 0.30, 1.00, 2.00], [0.006, 0.008, 0.010, 0.012])
    release_step = interp(v_ego, [0.00, 0.30, 1.00, 2.00], [0.0015, 0.0030, 0.0045, 0.0060])
    if self.release_lock_counter > 0:
      release_step = min(release_step, interp(v_ego, [0.00, 0.20, 0.50, 1.20], [0.0010, 0.0015, 0.0030, 0.0060]))
    if rebound_risk > 0.0:
      brake_step = max(brake_step, interp(rebound_risk, [0.0, 1.0], [0.008, 0.016]))
      release_step = min(release_step, interp(rebound_risk, [0.0, 1.0], [0.0018, 0.0010]))
    if self.rebound_arrest_counter > 0:
      brake_step = max(brake_step, interp(v_ego, [0.00, 0.08], [0.060, 0.025]))
      release_step = min(release_step, interp(v_ego, [0.00, 0.08], [0.0008, 0.0014]))
    if a_ego < (min_expected_accel - 0.10) and v_ego < 0.9:
      release_step = max(release_step, interp(v_ego, [0.00, 0.20, 0.55, 0.90], [0.014, 0.012, 0.009, 0.006]))

    dt_scale = clip(float(dt) / 0.01, 0.5, 20.0)
    brake_step *= dt_scale
    release_step *= dt_scale
    if self.step_scale > 0.0:
      brake_step *= self.step_scale
      release_step *= self.step_scale
    if self.brake_step_scale > 0.0:
      brake_step *= self.brake_step_scale
    if self.release_step_scale > 0.0:
      release_step *= self.release_step_scale

    wants_release = cmd_target > (float(last_cmd) + 1e-6)
    deep_cmd = float(last_cmd) < (self.hold_cmd_cap - 0.25)
    if wants_release and deep_cmd and v_ego < 0.12:
      release_step = max(release_step, float(interp(v_ego, [0.00, 0.12], [0.090, 0.060])))

    cmd = clip(cmd_target, float(last_cmd) - brake_step, float(last_cmd) + release_step)
    return float(clip(cmd, stop_accel, -0.05))

  def update(
    self,
    last_cmd: float,
    v_ego: float,
    a_ego: float,
    max_expected_accel: float,
    min_expected_accel: float,
    stop_accel: float,
    dt: float,
    should_stop: bool,
  ) -> float:
    self._update_rollout(v_ego, dt)
    disturbance = clip(a_ego - max_expected_accel, 0.0, 1.2)
    self._update_intent_latch(should_stop, v_ego, last_cmd, disturbance, dt)
    intent_strength = clip(float(self.intent_hold_counter) / 40.0, 0.0, 1.0)
    remaining_m = self._remaining_distance_est_m(v_ego, a_ego)
    final_window = clip((0.30 - remaining_m) / 0.30, 0.0, 1.0)

    cmd_target = self._baseline_update(last_cmd, v_ego, a_ego, max_expected_accel, min_expected_accel, stop_accel, dt)

    if self.rollout_floor_scale > 0.0 and final_window > 0.0:
      low_disturbance = clip((0.10 - disturbance) / 0.10, 0.0, 1.0)
      strong_decel = clip(((-a_ego) - 0.45) / 0.45, 0.0, 1.0)
      low_latch = clip((0.35 - intent_strength) / 0.35, 0.0, 1.0)
      soften = clip(self.rollout_floor_scale * final_window * strong_decel * low_disturbance * low_latch, 0.0, 1.0)
      if soften > 0.0:
        soft_cap = interp(remaining_m, [0.00, 0.04, 0.12, 0.30], [self.hold_cmd_cap + 0.10, self.hold_cmd_cap + 0.08, self.hold_cmd_cap + 0.04, self.hold_cmd_cap])
        softened = max(cmd_target, soft_cap)
        cmd_target = ((1.0 - soften) * cmd_target) + (soften * softened)

    weak_decel = clip((a_ego + 0.30) / 0.35, 0.0, 1.0)
    rollout_risk = clip((self.low_speed_rollout_m - 0.30) / 0.90, 0.0, 1.0)
    disturbance_risk = clip((disturbance - 0.015) / 0.18, 0.0, 1.0)
    rebound_risk = clip((0.55 * disturbance_risk) + (0.45 * rollout_risk * weak_decel), 0.0, 1.0)
    flicker_active = self.intent_hold_counter > 0 and not should_stop

    if final_window > 0.0 and (flicker_active or rebound_risk > 0.20):
      fallback_gain = 1.0 if flicker_active else clip((rebound_risk - 0.20) / 0.80, 0.0, 1.0)
      safety_cap = interp(v_ego, [0.00, 0.05, 0.12, 0.22, 0.35], [self.dropout_hold_cmd_cap, self.risk_hold_cmd_cap, -0.42, -0.33, -0.25])
      guarded = min(cmd_target, safety_cap)
      cmd_target = ((1.0 - fallback_gain) * cmd_target) + (fallback_gain * guarded)

    if final_window > 0.0 and a_ego < -0.65 and disturbance < 0.04:
      settle_cap = interp(remaining_m, [0.00, 0.05, 0.15, 0.30], [self.hold_cmd_cap + 0.11, self.hold_cmd_cap + 0.08, self.hold_cmd_cap + 0.05, self.hold_cmd_cap + 0.02])
      cmd_target = max(cmd_target, settle_cap)

    cmd_target = float(clip(cmd_target, stop_accel, -0.05))
    cmd = cmd_target
    if flicker_active and v_ego < 0.15 and cmd > (float(last_cmd) + 1e-6):
      release_guard_step = interp(v_ego, [0.00, 0.05, 0.15], [0.0008, 0.0015, 0.0035])
      dt_scale = clip(float(dt) / 0.01, 0.5, 20.0)
      cmd = min(cmd, float(last_cmd) + (release_guard_step * dt_scale))
    return float(clip(cmd, stop_accel, -0.05))


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
  parser.add_argument("--inverse-tau-s", type=float, default=1.12, help="Inverse policy time constant for a_ref = -min(a_max, v/tau)")
  parser.add_argument("--inverse-max-ref-decel", type=float, default=1.46, help="Inverse policy max reference decel magnitude (m/s^2)")
  parser.add_argument("--inverse-hold-cmd-speed", type=float, default=0.05, help="Inverse policy speed below which hold cap applies (m/s)")
  parser.add_argument("--inverse-kp", type=float, default=0.12, help="Inverse policy accel-reference proportional gain")
  parser.add_argument("--inverse-ki", type=float, default=0.03, help="Inverse policy accel-reference integral gain")
  parser.add_argument("--inverse-step-scale", type=float, default=0.71, help="Scale inverse command slew limits (smaller = smoother)")
  parser.add_argument("--inverse-brake-step-scale", type=float, default=0.45, help="Additional scale for inverse braking slew (smaller = less ratcheting)")
  parser.add_argument("--inverse-release-step-scale", type=float, default=1.14, help="Additional scale for inverse release slew (larger = unwind faster)")
  parser.add_argument("--inverse-v3-hold-cmd-cap", type=float, default=-0.23, help="Inverse-v3 smooth hold cap near standstill (min allowed cmd)")
  parser.add_argument("--inverse-v3-risk-hold-cmd-cap", type=float, default=-0.59, help="Inverse-v3 stronger hold cap when stop-intent risk is high")
  parser.add_argument("--inverse-v3-dropout-hold-cmd-cap", type=float, default=-0.78, help="Inverse-v3 deep hold cap while intent latch is active")
  parser.add_argument("--inverse-v3-extra-decel-scale", type=float, default=0.02, help="Scale factor for inverse-v3 core low-speed decel shaping")
  parser.add_argument("--inverse-v3-rollout-floor-scale", type=float, default=0.60, help="Inverse-v3 late-stop softening blend factor")
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
  parser.add_argument("--output-json", default=None)
  return parser.parse_args()


def enabled_ratio(samples: list[Any], start_idx: int, end_idx: int) -> float:
  if end_idx < start_idx:
    return 0.0
  count = end_idx - start_idx + 1
  return sum(1.0 if bool(samples[idx].enabled) else 0.0 for idx in range(start_idx, end_idx + 1)) / max(count, 1)


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


def simulate_event_with_inverse_v3_controller(
  samples: list[Any],
  start_idx: int,
  hold_idx: int,
  model: FittedStoppingModel,
  stop_accel: float,
  stopping_speed_breakpoint: float,
  tau_s: float,
  max_ref_decel: float,
  hold_cmd_cap: float,
  hold_cmd_speed: float,
  risk_hold_cmd_cap: float,
  dropout_hold_cmd_cap: float,
  extra_decel_scale: float,
  rollout_floor_scale: float,
  kp: float,
  ki: float,
  step_scale: float,
  brake_step_scale: float,
  release_step_scale: float,
) -> dict[str, Any]:
  start = max(0, int(start_idx))
  hold = max(start + 1, min(int(hold_idx), len(samples) - 1))
  if hold <= start:
    raise ValueError("Event window too short for inverse-v3 replay")

  mid_bp = clip(stopping_speed_breakpoint, 0.011, 0.499)
  v_bp = [0.01, mid_bp, 0.50]
  max_accel_bp = [-0.01, -0.10, -0.30]
  min_accel_bp = [-0.10, -0.50, -1.00]

  controller = _InverseStoppingControllerV3(
    model=model,
    tau_s=tau_s,
    max_ref_decel=max_ref_decel,
    hold_cmd_cap=hold_cmd_cap,
    hold_cmd_speed=hold_cmd_speed,
    risk_hold_cmd_cap=risk_hold_cmd_cap,
    dropout_hold_cmd_cap=dropout_hold_cmd_cap,
    extra_decel_scale=extra_decel_scale,
    rollout_floor_scale=rollout_floor_scale,
    kp=kp,
    ki=ki,
    step_scale=step_scale,
    brake_step_scale=brake_step_scale,
    release_step_scale=release_step_scale,
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
  predicted_distance_m = [0.0]
  replay_sample_indices = [start]
  rollout_total_m = 0.0
  rollout_from_2mps_m = 0.0

  for idx in range(start, hold):
    max_expected = interp(v_ego, v_bp, max_accel_bp)
    min_expected = interp(v_ego, v_bp, min_accel_bp)
    should_stop = bool(samples[idx].should_stop)
    output_cmd = controller.update(last_output, v_ego, a_ego, max_expected, min_expected, stop_accel, dt, should_stop=should_stop)
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
      )
      inverse_v3 = simulate_event_with_inverse_v3_controller(
        samples=samples,
        start_idx=sim_start_idx,
        hold_idx=sim_hold_idx,
        model=model,
        stop_accel=args.stop_accel,
        stopping_speed_breakpoint=args.stopping_speed_breakpoint,
        tau_s=args.inverse_tau_s,
        max_ref_decel=args.inverse_max_ref_decel,
        hold_cmd_cap=args.inverse_v3_hold_cmd_cap,
        hold_cmd_speed=args.inverse_hold_cmd_speed,
        risk_hold_cmd_cap=args.inverse_v3_risk_hold_cmd_cap,
        dropout_hold_cmd_cap=args.inverse_v3_dropout_hold_cmd_cap,
        extra_decel_scale=args.inverse_v3_extra_decel_scale,
        rollout_floor_scale=args.inverse_v3_rollout_floor_scale,
        kp=args.inverse_kp,
        ki=args.inverse_ki,
        step_scale=args.inverse_step_scale,
        brake_step_scale=args.inverse_brake_step_scale,
        release_step_scale=args.inverse_release_step_scale,
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
      m_inv3 = classify(inverse_v3, args)
      m_leg = classify(legacy, args)

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
        "inverse_v3": {
          "harsh": m_inv3.is_harsh,
          "flags": m_inv3.flags,
          "leapfrog": m_inv3.is_leapfrog,
          "leapfrog_flags": m_inv3.leapfrog_flags,
          "score": m_inv3.event_score,
          "pred_end_stop_jerk_mps3": m_inv3.pred_end_stop_jerk_mps3,
          "pred_end_stop_cmd_jerk_mps3": m_inv3.pred_end_stop_cmd_jerk_mps3,
          "pred_end_stop_accel_step_mps2": m_inv3.pred_end_stop_accel_step_mps2,
          "pred_min_a_ego_mps2": m_inv3.pred_min_a_ego_mps2,
          "pred_rollout_distance_m": m_inv3.pred_rollout_distance_m,
          "pred_lead_distance_hold_m": m_inv3.pred_lead_distance_hold_m,
          "recorded_lead_distance_hold_m": m_inv3.recorded_lead_distance_hold_m,
          "distance_gate_source": m_inv3.distance_gate_source,
          "pred_speed_rebound_while_should_stop_mps": m_inv3.pred_speed_rebound_while_should_stop_mps,
          "pred_should_stop_unexpected_accel_mps2": m_inv3.pred_should_stop_unexpected_accel_mps2,
        },
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
      })

  current_harsh = sum(1 for row in rows if row["current"]["harsh"])
  inverse_v3_harsh = sum(1 for row in rows if row["inverse_v3"]["harsh"])
  legacy_harsh = sum(1 for row in rows if row["legacy_32b8be"]["harsh"])
  current_leapfrog = sum(1 for row in rows if row["current"]["leapfrog"])
  inverse_v3_leapfrog = sum(1 for row in rows if row["inverse_v3"]["leapfrog"])
  legacy_leapfrog = sum(1 for row in rows if row["legacy_32b8be"]["leapfrog"])
  n = len(rows)
  current_rate = (current_harsh / n) if n else 0.0
  inverse_v3_rate = (inverse_v3_harsh / n) if n else 0.0
  legacy_rate = (legacy_harsh / n) if n else 0.0
  current_leapfrog_rate = (current_leapfrog / n) if n else 0.0
  inverse_v3_leapfrog_rate = (inverse_v3_leapfrog / n) if n else 0.0
  legacy_leapfrog_rate = (legacy_leapfrog / n) if n else 0.0
  current_avg = (sum(row["current"]["score"] for row in rows) / n) if n else 0.0
  inverse_v3_avg = (sum(row["inverse_v3"]["score"] for row in rows) / n) if n else 0.0
  legacy_avg = (sum(row["legacy_32b8be"]["score"] for row in rows) / n) if n else 0.0

  inverse_v3_improved = sum(1 for row in rows if row["inverse_v3"]["score"] < row["current"]["score"] - 1e-6)
  inverse_v3_worsened = sum(1 for row in rows if row["inverse_v3"]["score"] > row["current"]["score"] + 1e-6)

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
    "inverse_v3": {
      "harsh_events": inverse_v3_harsh,
      "harsh_rate": inverse_v3_rate,
      "leapfrog_events": inverse_v3_leapfrog,
      "leapfrog_rate": inverse_v3_leapfrog_rate,
      "avg_event_score": inverse_v3_avg,
    },
    "legacy_32b8be": {
      "harsh_events": legacy_harsh,
      "harsh_rate": legacy_rate,
      "leapfrog_events": legacy_leapfrog,
      "leapfrog_rate": legacy_leapfrog_rate,
      "avg_event_score": legacy_avg,
    },
    "comparison": {
      "improved_events": inverse_v3_improved,
      "worsened_events": inverse_v3_worsened,
      "inverse_v3_improved_events": inverse_v3_improved,
      "inverse_v3_worsened_events": inverse_v3_worsened,
    },
    "event_rows": rows,
  }

  print(f"[benchmark] events={n}")
  print(
    f"[benchmark] current harsh={current_harsh}/{n} rate={current_rate:.3f}"
    + f" leapfrog={current_leapfrog}/{n} leapfrog_rate={current_leapfrog_rate:.3f} avg_score={current_avg:.3f}"
  )
  print(
    f"[benchmark] inverse_v3 harsh={inverse_v3_harsh}/{n} rate={inverse_v3_rate:.3f}"
    + f" leapfrog={inverse_v3_leapfrog}/{n} leapfrog_rate={inverse_v3_leapfrog_rate:.3f} avg_score={inverse_v3_avg:.3f}"
  )
  print(
    f"[benchmark] legacy_32b8be harsh={legacy_harsh}/{n} rate={legacy_rate:.3f}"
    + f" leapfrog={legacy_leapfrog}/{n} leapfrog_rate={legacy_leapfrog_rate:.3f} avg_score={legacy_avg:.3f}"
  )
  print(
    f"[benchmark] improved={inverse_v3_improved} worsened={inverse_v3_worsened}"
    + f" inverse_v3_improved={inverse_v3_improved} inverse_v3_worsened={inverse_v3_worsened}"
  )

  if args.output_json:
    out = Path(args.output_json).expanduser()
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n")
    print(f"[benchmark] output_json={out}")

  return 0


if __name__ == "__main__":
  raise SystemExit(main())
