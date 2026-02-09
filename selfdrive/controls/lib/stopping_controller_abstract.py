from __future__ import annotations

from enum import IntEnum

from openpilot.common.numpy_fast import clip, interp
from openpilot.selfdrive.controls.lib.stopping_controller import StoppingResult


class AbstractStoppingPhase(IntEnum):
  APPROACH = 0
  SETTLE = 1
  HOLD = 2


class _AbstractStoppingControllerBase:
  """Simpler stop-controller base built around phase target + rollout feedback."""

  def __init__(self) -> None:
    self.phase = AbstractStoppingPhase.APPROACH
    self.low_speed_rollout_m = 0.0

  def reset(self) -> None:
    self.phase = AbstractStoppingPhase.APPROACH
    self.low_speed_rollout_m = 0.0

  def _phase_for_speed(self, v_ego: float) -> AbstractStoppingPhase:
    if v_ego <= 0.06:
      return AbstractStoppingPhase.HOLD
    if v_ego <= 0.90:
      return AbstractStoppingPhase.SETTLE
    return AbstractStoppingPhase.APPROACH

  def _update_rollout(self, should_stop: bool, v_ego: float, dt: float) -> None:
    if not should_stop or v_ego <= 0.02:
      self.low_speed_rollout_m = 0.0
      return
    if v_ego < 1.4:
      self.low_speed_rollout_m += v_ego * dt
    else:
      self.low_speed_rollout_m = max(0.0, self.low_speed_rollout_m - (0.35 * v_ego * dt))

  def _rollout_feedback(self, v_ego: float) -> float:
    trigger = interp(v_ego, [0.02, 0.20, 0.55, 1.20], [0.10, 0.22, 0.40, 0.90])
    full = interp(v_ego, [0.02, 0.20, 0.55, 1.20], [0.35, 0.62, 1.00, 2.20])
    scale = clip((self.low_speed_rollout_m - trigger) / max(full - trigger, 1e-3), 0.0, 1.0)
    base = scale * interp(v_ego, [0.02, 0.20, 0.55, 1.20], [0.05, 0.07, 0.09, 0.12])
    return base * self._rollout_scale()

  def _rollout_scale(self) -> float:
    return 1.0

  def _disturbance_feedback(self, v_ego: float, a_ego: float, max_expected_accel: float, min_expected_accel: float, dt: float) -> float:
    disturbance = clip(a_ego - max_expected_accel, 0.0, 1.0)
    over_brake = clip(min_expected_accel - a_ego, 0.0, 1.2)
    push = disturbance * interp(v_ego, [0.00, 0.55, 1.20], [0.11, 0.09, 0.07]) * dt
    relief = over_brake * interp(v_ego, [0.00, 0.55, 1.20], [0.07, 0.05, 0.03]) * dt
    return push - relief

  def _phase_target(self, v_ego: float, output_accel: float, a_ego: float) -> float:
    raise NotImplementedError

  def _slew_limits(self, v_ego: float, a_ego: float) -> tuple[float, float]:
    raise NotImplementedError

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
    self._update_rollout(should_stop=should_stop, v_ego=v_ego, dt=dt)
    target = min(output_accel, -0.1)
    target = min(target, self._phase_target(v_ego=v_ego, output_accel=output_accel, a_ego=a_ego))
    target -= self._disturbance_feedback(
      v_ego=v_ego,
      a_ego=a_ego,
      max_expected_accel=max_expected_accel,
      min_expected_accel=min_expected_accel,
      dt=dt,
    )
    target -= self._rollout_feedback(v_ego=v_ego)

    brake_step, release_step = self._slew_limits(v_ego=v_ego, a_ego=a_ego)
    output = clip(target, last_output_accel - brake_step, last_output_accel + release_step)
    output = clip(output, stop_accel, -0.05)
    return StoppingResult(output_accel=output, release_lock_active=False)


class AbstractStoppingControllerV2(_AbstractStoppingControllerBase):
  """Profile-based rewrite with small feedback terms and phase-specific slew limits."""

  def _rollout_scale(self) -> float:
    return 1.35

  def _phase_target(self, v_ego: float, output_accel: float, a_ego: float) -> float:
    if self.phase == AbstractStoppingPhase.APPROACH:
      return interp(v_ego, [0.90, 1.30, 1.80, 2.50], [-0.30, -0.38, -0.50, -0.62])
    if self.phase == AbstractStoppingPhase.SETTLE:
      settle = interp(v_ego, [0.06, 0.15, 0.35, 0.60, 0.90], [-0.38, -0.34, -0.29, -0.24, -0.20])
      if a_ego < -0.85:
        settle = max(settle, -0.36)
      return settle
    return interp(v_ego, [0.00, 0.02, 0.06], [-0.30, -0.25, -0.21])

  def _slew_limits(self, v_ego: float, a_ego: float) -> tuple[float, float]:
    if self.phase == AbstractStoppingPhase.APPROACH:
      brake_step = interp(v_ego, [0.90, 1.30, 1.80], [0.0080, 0.0075, 0.0070])
      release_step = interp(v_ego, [0.90, 1.30, 1.80], [0.0040, 0.0050, 0.0060])
    elif self.phase == AbstractStoppingPhase.SETTLE:
      brake_step = interp(v_ego, [0.06, 0.30, 0.90], [0.0072, 0.0084, 0.0095])
      release_step = interp(v_ego, [0.06, 0.30, 0.90], [0.0010, 0.0024, 0.0036])
      if a_ego < -0.90:
        release_step = max(release_step, 0.0085)
    else:
      brake_step = interp(v_ego, [0.00, 0.06], [0.0062, 0.0072])
      release_step = interp(v_ego, [0.00, 0.06], [0.0008, 0.0015])
    return brake_step, release_step


class AbstractStoppingControllerV3(_AbstractStoppingControllerBase):
  """Distance-to-hold rewrite: dynamic target from remaining rollout budget."""

  def _rollout_budget(self, v_ego: float) -> float:
    return interp(v_ego, [0.02, 0.20, 0.55, 1.20, 1.80], [0.10, 0.18, 0.35, 0.85, 1.60])

  def _phase_target(self, v_ego: float, output_accel: float, a_ego: float) -> float:
    budget = self._rollout_budget(v_ego)
    remaining = max(0.05, budget - (0.45 * self.low_speed_rollout_m))
    required_mag = (v_ego * v_ego) / max(2.0 * remaining, 0.1)
    required_target = -clip(required_mag, 0.08, 0.85)
    phase_floor = interp(v_ego, [0.00, 0.06, 0.20, 0.55, 1.20], [-0.24, -0.21, -0.23, -0.27, -0.33])
    target = min(required_target, phase_floor)
    if a_ego < -0.95 and self.phase != AbstractStoppingPhase.APPROACH:
      target = max(target, interp(v_ego, [0.00, 0.20, 0.55], [-0.34, -0.35, -0.37]))
    return target

  def _slew_limits(self, v_ego: float, a_ego: float) -> tuple[float, float]:
    brake_step = interp(v_ego, [0.00, 0.20, 0.55, 1.20], [0.0055, 0.0062, 0.0074, 0.0088])
    release_step = interp(v_ego, [0.00, 0.20, 0.55, 1.20], [0.0010, 0.0023, 0.0039, 0.0055])
    if self.low_speed_rollout_m > self._rollout_budget(v_ego):
      release_step = min(release_step, interp(v_ego, [0.00, 0.20, 0.55, 1.20], [0.0008, 0.0012, 0.0020, 0.0030]))
    if a_ego < -0.95:
      release_step = max(release_step, 0.0080)
    return brake_step, release_step
  def _rollout_scale(self) -> float:
    return 1.15
