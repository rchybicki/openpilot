from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum

from openpilot.common.numpy_fast import clip, interp


class StoppingPhase(IntEnum):
  APPROACH = 0
  NEAR_HOLD = 1
  HOLD = 2


@dataclass
class StoppingResult:
  output_accel: float
  release_lock_active: bool


class StoppingController:
  """Stateful stop controller with explicit low-speed phases and disturbance lock."""

  def __init__(self) -> None:
    self.phase = StoppingPhase.APPROACH
    self.release_lock_counter = 0
    self.low_speed_rollout_m = 0.0

  def reset(self) -> None:
    self.phase = StoppingPhase.APPROACH
    self.release_lock_counter = 0
    self.low_speed_rollout_m = 0.0

  def _phase_for_speed(self, v_ego: float) -> StoppingPhase:
    if v_ego <= 0.06:
      return StoppingPhase.HOLD
    if v_ego <= 0.85:
      return StoppingPhase.NEAR_HOLD
    return StoppingPhase.APPROACH

  def _update_release_lock(self, v_ego: float, a_ego: float, last_output_accel: float, max_expected_accel: float) -> None:
    disturbance = a_ego - max_expected_accel
    disturbance_detected = (
      v_ego < 1.2
      and last_output_accel < -0.1
      and disturbance > 0.03
    )
    if disturbance_detected:
      lock_frames = int(interp(v_ego, [0.0, 0.20, 0.60, 1.20], [110, 95, 70, 50]))
      self.release_lock_counter = max(self.release_lock_counter, lock_frames)
    elif self.release_lock_counter > 0:
      self.release_lock_counter -= 1

  def _update_low_speed_rollout(self, should_stop: bool, v_ego: float, dt: float) -> None:
    if not should_stop or v_ego <= 0.02:
      self.low_speed_rollout_m = 0.0
      return

    if v_ego < 1.2:
      self.low_speed_rollout_m += v_ego * dt
    else:
      self.low_speed_rollout_m = max(self.low_speed_rollout_m - (v_ego * dt), 0.0)

  def update(
    self,
    output_accel: float,
    last_output_accel: float,
    should_stop: bool,
    v_ego: float,
    a_ego: float,
    max_expected_accel: float,
    min_expected_accel: float,
    stop_accel: float,
    dt: float,
  ) -> StoppingResult:
    if not should_stop:
      self.reset()
      return StoppingResult(output_accel=output_accel, release_lock_active=False)

    self.phase = self._phase_for_speed(v_ego)
    self._update_release_lock(v_ego, a_ego, last_output_accel, max_expected_accel)
    self._update_low_speed_rollout(should_stop, v_ego, dt)
    release_lock_active = self.release_lock_counter > 0
    disturbance = clip(a_ego - max_expected_accel, 0.0, 1.0)

    rollout_trigger = interp(v_ego, [0.02, 0.25, 0.55, 1.20], [0.10, 0.20, 0.35, 0.70])
    rollout_full = interp(v_ego, [0.02, 0.25, 0.55, 1.20], [0.40, 0.65, 1.00, 2.20])
    rollout_tighten = clip(
      (self.low_speed_rollout_m - rollout_trigger) / max(rollout_full - rollout_trigger, 1e-3),
      0.0,
      1.0,
    )

    target = min(output_accel, -0.1)
    if self.phase == StoppingPhase.APPROACH:
      if disturbance > 0.0:
        target -= disturbance * interp(v_ego, [0.55, 1.20, 3.00], [0.10, 0.07, 0.05]) * dt
      over_brake = clip(min_expected_accel - a_ego, 0.0, 0.8)
      if over_brake > 0.0 and v_ego < 1.2:
        target += over_brake * 0.04 * dt
      brake_step = interp(v_ego, [0.55, 1.20], [0.010, 0.009])
      release_step = interp(v_ego, [0.55, 1.20], [0.005, 0.009])
    elif self.phase == StoppingPhase.NEAR_HOLD:
      hold_target = interp(v_ego, [0.06, 0.15, 0.30, 0.55, 0.85], [-0.34, -0.29, -0.24, -0.19, -0.15])
      target = min(target, hold_target)
      if disturbance > 0.0:
        target -= disturbance * interp(v_ego, [0.06, 0.55], [0.12, 0.07]) * dt
      brake_step = interp(v_ego, [0.06, 0.55, 0.85], [0.007, 0.009, 0.010])
      release_step = interp(v_ego, [0.06, 0.55, 0.85], [0.0010, 0.0028, 0.0038])
    else:
      hold_target = interp(v_ego, [0.00, 0.02, 0.06], [-0.26, -0.22, -0.18])
      target = min(target, hold_target)
      if disturbance > 0.0:
        target -= disturbance * 0.08 * dt
      brake_step = interp(v_ego, [0.00, 0.06], [0.006, 0.007])
      release_step = interp(v_ego, [0.00, 0.06], [0.0006, 0.0012])

    if rollout_tighten > 0.0:
      release_cap = interp(v_ego, [0.02, 0.25, 0.55, 1.20], [0.0010, 0.0018, 0.0030, 0.0050])
      release_step = min(release_step, release_cap)
      target -= rollout_tighten * interp(v_ego, [0.02, 0.25, 0.55, 1.20], [0.05, 0.08, 0.11, 0.12])
      rollout_floor = interp(v_ego, [0.02, 0.12, 0.25, 0.55, 1.20], [-0.30, -0.27, -0.24, -0.19, -0.13])
      target = min(target, rollout_floor + ((1.0 - rollout_tighten) * 0.05))

    if release_lock_active:
      release_step = min(release_step, interp(v_ego, [0.00, 0.20, 0.50, 1.20], [0.0010, 0.0015, 0.0030, 0.0060]))
      lock_floor = interp(v_ego, [0.00, 0.12, 0.25, 0.50, 1.20], [-0.34, -0.31, -0.26, -0.18, -0.11])
      target = min(target, lock_floor)

    limited_output = clip(target, last_output_accel - brake_step, last_output_accel + release_step)
    limited_output = clip(limited_output, stop_accel, -0.05)
    return StoppingResult(output_accel=limited_output, release_lock_active=release_lock_active)
