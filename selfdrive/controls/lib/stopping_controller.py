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
    self.delay_frames = 5
    self._command_history: list[float] = []

  def reset(self) -> None:
    self.phase = StoppingPhase.APPROACH
    self.release_lock_counter = 0
    self.low_speed_rollout_m = 0.0
    self._command_history = []

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

  def _append_command(self, last_output_accel: float) -> None:
    self._command_history.append(float(last_output_accel))
    if len(self._command_history) > 48:
      self._command_history = self._command_history[-48:]

  def _delayed_command(self, fallback: float) -> float:
    if not self._command_history:
      return fallback
    delayed_index = len(self._command_history) - 1 - self.delay_frames
    if delayed_index < 0:
      return self._command_history[0]
    return self._command_history[delayed_index]

  def _delay_release_guard(self, v_ego: float, last_output_accel: float) -> float:
    delayed_cmd = self._delayed_command(last_output_accel)
    release_relief = clip(last_output_accel - delayed_cmd, 0.0, 0.35)
    relief_trigger = interp(v_ego, [0.00, 0.55, 1.20], [0.006, 0.014, 0.020])
    relief_scale = interp(v_ego, [0.00, 0.55, 1.20], [0.020, 0.040, 0.060])
    return clip((release_relief - relief_trigger) / max(relief_scale, 1e-3), 0.0, 1.0)

  def _apply_over_brake_damping(
    self,
    target: float,
    release_step: float,
    v_ego: float,
    a_ego: float,
    min_expected_accel: float,
    dt: float,
  ) -> tuple[float, float]:
    over_brake = clip(min_expected_accel - a_ego, 0.0, 1.2)
    if over_brake <= 0.0 or v_ego > 0.90:
      return target, release_step

    relax_gain = interp(v_ego, [0.00, 0.20, 0.55, 0.90], [0.16, 0.12, 0.08, 0.05])
    release_gain = interp(v_ego, [0.00, 0.20, 0.55, 0.90], [0.0018, 0.0014, 0.0009, 0.0005])
    target += over_brake * relax_gain * dt
    release_step += over_brake * release_gain
    return target, release_step

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

    self._append_command(last_output_accel)
    self.phase = self._phase_for_speed(v_ego)
    self._update_release_lock(v_ego, a_ego, last_output_accel, max_expected_accel)
    self._update_low_speed_rollout(should_stop, v_ego, dt)
    release_lock_active = self.release_lock_counter > 0
    disturbance = clip(a_ego - max_expected_accel, 0.0, 1.0)
    delay_release_guard = self._delay_release_guard(v_ego, last_output_accel)
    lock_overbrake_relief = a_ego < (min_expected_accel - 0.12)
    clutch_push_relief = (
      v_ego < 2.5
      and a_ego > 0.08
      and last_output_accel < -0.65
    )

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
      if v_ego > 0.90 and last_output_accel < -0.30:
        approach_floor = interp(v_ego, [0.90, 1.20, 1.60], [-0.40, -0.45, -0.50])
        target = min(target, approach_floor)
      over_brake = clip(min_expected_accel - a_ego, 0.0, 0.8)
      if over_brake > 0.0 and v_ego < 1.2:
        target += over_brake * 0.04 * dt
      brake_step = interp(v_ego, [0.55, 1.20], [0.008, 0.007])
      release_step = interp(v_ego, [0.55, 1.20], [0.004, 0.006])
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

    target, release_step = self._apply_over_brake_damping(
      target=target,
      release_step=release_step,
      v_ego=v_ego,
      a_ego=a_ego,
      min_expected_accel=min_expected_accel,
      dt=dt,
    )

    if clutch_push_relief:
      # Under heavy braking, some automatic gearboxes can still push the car forward.
      # Avoid ratcheting to very deep brake commands in this phase, which tends to increase end-stop jerk.
      relief_target = interp(v_ego, [0.00, 0.60, 1.20, 1.80, 2.50], [-0.44, -0.48, -0.52, -0.57, -0.62])
      target = max(target, relief_target)
      brake_step = min(brake_step, interp(v_ego, [0.00, 0.60, 1.20, 1.80, 2.50], [0.0015, 0.0020, 0.0026, 0.0034, 0.0042]))
      release_step = max(release_step, interp(v_ego, [0.00, 0.60, 1.20, 1.80, 2.50], [0.0215, 0.0235, 0.0255, 0.0275, 0.0295]))

    comfort_release = (
      self.phase == StoppingPhase.NEAR_HOLD
      and v_ego < 0.9
      and a_ego < -0.45
      and last_output_accel < -0.80
      and not clutch_push_relief
    )
    if comfort_release:
      # If the car is already decelerating strongly near hold, avoid adding more brake.
      # This limits end-stop jerk spikes while preserving short rollout.
      comfort_target = interp(v_ego, [0.06, 0.20, 0.55, 0.90], [-0.755, -0.775, -0.805, -0.845])
      target = max(target, comfort_target)
      brake_step = min(brake_step, interp(v_ego, [0.06, 0.20, 0.55, 0.90], [0.0022, 0.0026, 0.0032, 0.0042]))
      release_step = max(release_step, interp(v_ego, [0.06, 0.20, 0.55, 0.90], [0.0088, 0.0098, 0.0108, 0.0118]))

    hard_brake_hold_relief = (
      self.phase in (StoppingPhase.NEAR_HOLD, StoppingPhase.HOLD)
      and v_ego < 0.45
      and a_ego < -0.95
      and not clutch_push_relief
    )
    mild_command_deep_decel_relief = (
      self.phase == StoppingPhase.NEAR_HOLD
      and v_ego < 0.9
      and a_ego < -0.95
      and last_output_accel > -0.55
      and not clutch_push_relief
    )
    approach_deep_decel_relief = (
      self.phase == StoppingPhase.APPROACH
      and v_ego < 1.0
      and a_ego < -0.90
      and last_output_accel > -0.55
      and not clutch_push_relief
    )

    if rollout_tighten > 0.0:
      release_cap = interp(v_ego, [0.02, 0.25, 0.55, 1.20], [0.0010, 0.0018, 0.0030, 0.0050])
      release_step = min(release_step, release_cap)
      target -= rollout_tighten * interp(v_ego, [0.02, 0.25, 0.55, 1.20], [0.05, 0.08, 0.11, 0.12])
      rollout_floor = interp(v_ego, [0.02, 0.12, 0.25, 0.55, 1.20], [-0.30, -0.27, -0.24, -0.19, -0.13])
      target = min(target, rollout_floor + ((1.0 - rollout_tighten) * 0.05))

    if delay_release_guard > 0.0:
      delay_release_cap = interp(v_ego, [0.00, 0.20, 0.55, 1.20], [0.0004, 0.0008, 0.0015, 0.0024])
      release_step = min(release_step, delay_release_cap)
      target -= delay_release_guard * interp(v_ego, [0.00, 0.20, 0.55, 1.20], [0.05, 0.07, 0.10, 0.11])

    if hard_brake_hold_relief:
      # In deep near-standstill decel, keep hold braking from ratcheting further down.
      hold_relief_target = interp(v_ego, [0.00, 0.10, 0.25, 0.45], [-0.34, -0.36, -0.38, -0.42])
      target = max(target, hold_relief_target)
      brake_step = min(brake_step, interp(v_ego, [0.00, 0.10, 0.25, 0.45], [0.0015, 0.0018, 0.0023, 0.0028]))
      release_step = max(release_step, interp(v_ego, [0.00, 0.10, 0.25, 0.45], [0.0070, 0.0076, 0.0088, 0.0100]))

    if mild_command_deep_decel_relief:
      # If decel is already very strong under a modest brake command, allow earlier release.
      mild_relief_target = interp(v_ego, [0.06, 0.20, 0.50, 0.90], [-0.30, -0.32, -0.34, -0.36])
      target = max(target, mild_relief_target)
      brake_step = min(brake_step, interp(v_ego, [0.06, 0.20, 0.50, 0.90], [0.0012, 0.0015, 0.0019, 0.0024]))
      release_step = max(release_step, interp(v_ego, [0.06, 0.20, 0.50, 0.90], [0.0085, 0.0092, 0.0100, 0.0110]))

    if approach_deep_decel_relief:
      # Similar relief while still in approach, to prevent carry-over harshness into near-hold.
      approach_relief_target = interp(v_ego, [0.40, 0.70, 1.00], [-0.31, -0.33, -0.35])
      target = max(target, approach_relief_target)
      brake_step = min(brake_step, interp(v_ego, [0.40, 0.70, 1.00], [0.0012, 0.0017, 0.0022]))
      release_step = max(release_step, interp(v_ego, [0.40, 0.70, 1.00], [0.0080, 0.0090, 0.0100]))

    if release_lock_active:
      if lock_overbrake_relief:
        release_step = max(release_step, interp(v_ego, [0.00, 0.10, 0.30, 0.70, 1.20], [0.0070, 0.0078, 0.0090, 0.0105, 0.0120]))
      elif clutch_push_relief:
        release_step = min(release_step, interp(v_ego, [0.00, 0.60, 1.20, 1.80, 2.50], [0.0195, 0.0215, 0.0235, 0.0255, 0.0275]))
      else:
        release_step = min(release_step, interp(v_ego, [0.00, 0.20, 0.50, 1.20], [0.0010, 0.0015, 0.0030, 0.0060]))
        lock_floor = interp(v_ego, [0.00, 0.12, 0.25, 0.50, 1.20], [-0.34, -0.31, -0.26, -0.18, -0.11])
        target = min(target, lock_floor)

    limited_output = clip(target, last_output_accel - brake_step, last_output_accel + release_step)
    limited_output = clip(limited_output, stop_accel, -0.05)
    return StoppingResult(output_accel=limited_output, release_lock_active=release_lock_active)
