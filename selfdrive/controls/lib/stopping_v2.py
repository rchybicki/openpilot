from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum

from openpilot.common.numpy_fast import clip, interp


class StoppingV2Phase(IntEnum):
  APPROACH = 0
  NEAR_HOLD = 1
  HOLD = 2


@dataclass
class StoppingV2Result:
  output_accel: float
  release_lock_active: bool


class StoppingV2Controller:
  """Stateful stop controller with explicit low-speed phases and disturbance lock."""

  def __init__(self) -> None:
    self.phase = StoppingV2Phase.APPROACH
    self.release_lock_counter = 0

  def reset(self) -> None:
    self.phase = StoppingV2Phase.APPROACH
    self.release_lock_counter = 0

  def _phase_for_speed(self, v_ego: float) -> StoppingV2Phase:
    if v_ego <= 0.06:
      return StoppingV2Phase.HOLD
    if v_ego <= 0.55:
      return StoppingV2Phase.NEAR_HOLD
    return StoppingV2Phase.APPROACH

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
  ) -> StoppingV2Result:
    if not should_stop:
      self.reset()
      return StoppingV2Result(output_accel=output_accel, release_lock_active=False)

    self.phase = self._phase_for_speed(v_ego)
    self._update_release_lock(v_ego, a_ego, last_output_accel, max_expected_accel)
    release_lock_active = self.release_lock_counter > 0
    disturbance = clip(a_ego - max_expected_accel, 0.0, 1.0)

    target = min(output_accel, -0.1)
    if self.phase == StoppingV2Phase.APPROACH:
      if disturbance > 0.0:
        target -= disturbance * interp(v_ego, [0.55, 1.20, 3.00], [0.10, 0.07, 0.05]) * dt
      over_brake = clip(min_expected_accel - a_ego, 0.0, 0.8)
      if over_brake > 0.0 and v_ego < 1.2:
        target += over_brake * 0.04 * dt
      brake_step = interp(v_ego, [0.55, 1.20], [0.010, 0.009])
      release_step = interp(v_ego, [0.55, 1.20], [0.005, 0.009])
    elif self.phase == StoppingV2Phase.NEAR_HOLD:
      hold_target = interp(v_ego, [0.06, 0.15, 0.30, 0.55], [-0.30, -0.24, -0.18, -0.12])
      target = min(target, hold_target)
      if disturbance > 0.0:
        target -= disturbance * interp(v_ego, [0.06, 0.55], [0.12, 0.07]) * dt
      brake_step = interp(v_ego, [0.06, 0.55], [0.008, 0.010])
      release_step = interp(v_ego, [0.06, 0.55], [0.0015, 0.004])
    else:
      hold_target = interp(v_ego, [0.00, 0.02, 0.06], [-0.22, -0.18, -0.14])
      target = min(target, hold_target)
      if disturbance > 0.0:
        target -= disturbance * 0.08 * dt
      brake_step = interp(v_ego, [0.00, 0.06], [0.006, 0.007])
      release_step = interp(v_ego, [0.00, 0.06], [0.0008, 0.0015])

    if release_lock_active:
      release_step = min(release_step, interp(v_ego, [0.00, 0.20, 0.50, 1.20], [0.0010, 0.0015, 0.0030, 0.0060]))
      lock_floor = interp(v_ego, [0.00, 0.12, 0.25, 0.50, 1.20], [-0.34, -0.31, -0.26, -0.18, -0.11])
      target = min(target, lock_floor)

    limited_output = clip(target, last_output_accel - brake_step, last_output_accel + release_step)
    limited_output = clip(limited_output, stop_accel, -0.05)
    return StoppingV2Result(output_accel=limited_output, release_lock_active=release_lock_active)
