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
    self.rebound_arrest_counter = 0
    self.low_speed_recovery_i = 0.0
    self.low_speed_rollout_m = 0.0
    self.standstill_settled_time_s = 0.0
    self.delay_frames = 5
    self._command_history: list[float] = []

  def reset(self) -> None:
    self.phase = StoppingPhase.APPROACH
    self.release_lock_counter = 0
    self.rebound_arrest_counter = 0
    self.low_speed_recovery_i = 0.0
    self.low_speed_rollout_m = 0.0
    self.standstill_settled_time_s = 0.0
    self._command_history = []

  def _phase_for_speed(self, v_ego: float) -> StoppingPhase:
    if v_ego <= 0.06:
      return StoppingPhase.HOLD
    if v_ego <= 0.85:
      return StoppingPhase.NEAR_HOLD
    return StoppingPhase.APPROACH

  def _update_release_lock(self, v_ego: float, a_ego: float, last_output_accel: float, max_expected_accel: float, dt: float) -> None:
    disturbance = a_ego - max_expected_accel
    disturbance_threshold = 0.04 if v_ego < 0.08 else 0.03
    disturbance_detected = (
      v_ego > 0.002
      and v_ego < 1.2
      and last_output_accel < -0.05
      and disturbance >= disturbance_threshold
    )
    if disturbance_detected:
      lock_frames_100hz = int(interp(v_ego, [0.0, 0.20, 0.60, 1.20], [110, 95, 70, 50]))
      dt_scale = clip(dt / 0.01, 0.5, 20.0)
      lock_steps = max(1, int(lock_frames_100hz / dt_scale))
      self.release_lock_counter = max(self.release_lock_counter, lock_steps)
    elif self.release_lock_counter > 0:
      self.release_lock_counter -= 1

  def _update_low_speed_rollout(self, should_stop: bool, v_ego: float, dt: float) -> None:
    if not should_stop:
      self.low_speed_rollout_m = 0.0
      return

    if v_ego <= 0.02:
      self.low_speed_rollout_m = max(self.low_speed_rollout_m - (0.35 * dt), 0.0)
    elif v_ego < 1.2:
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

  def _low_speed_rebound_risk(
    self,
    should_stop: bool,
    v_ego: float,
    a_ego: float,
    last_output_accel: float,
    disturbance: float,
    release_lock_active: bool,
    clutch_push_relief: bool,
  ) -> float:
    if (
      not should_stop
      or clutch_push_relief
      or self.phase not in (StoppingPhase.NEAR_HOLD, StoppingPhase.HOLD)
      or not (0.0 < v_ego < 0.25)
    ):
      return 0.0

    speed_factor = clip((0.25 - v_ego) / 0.25, 0.0, 1.0)
    decel_weakness = clip((a_ego + 0.45) / 0.45, 0.0, 1.0)
    disturbance_factor = clip((disturbance - 0.02) / 0.18, 0.0, 1.0)
    rollout_factor = clip((self.low_speed_rollout_m - 0.20) / 0.90, 0.0, 1.0)
    lock_factor = 1.0 if release_lock_active else 0.0
    cmd_relief = clip((last_output_accel + 0.45) / 0.25, 0.0, 1.0)

    risk = (
      (0.30 * speed_factor)
      + (0.26 * decel_weakness)
      + (0.18 * disturbance_factor)
      + (0.14 * rollout_factor)
      + (0.12 * lock_factor)
    )
    if self.phase == StoppingPhase.HOLD:
      risk *= 1.08
    risk *= (0.82 + (0.18 * cmd_relief))

    if (a_ego > -0.30) or (disturbance > 0.12) or (release_lock_active and disturbance > 0.05):
      return clip(risk, 0.0, 1.0)
    return 0.0

  def _update_rebound_arrest(
    self,
    should_stop: bool,
    v_ego: float,
    a_ego: float,
    last_output_accel: float,
    disturbance: float,
    release_lock_active: bool,
    low_speed_rebound_risk: float,
    clutch_push_relief: bool,
    dt: float,
  ) -> None:
    if not should_stop or clutch_push_relief or self.phase != StoppingPhase.HOLD:
      self.rebound_arrest_counter = 0
      return

    arrest_trigger = (
      0.0 < v_ego < 0.045
      and last_output_accel < -0.22
      and a_ego > -0.24
      and low_speed_rebound_risk > 0.12
      and (release_lock_active or disturbance > 0.08 or self.low_speed_rollout_m > 0.30)
    )
    if arrest_trigger:
      base_frames_100hz = interp(v_ego, [0.00, 0.03, 0.08], [48, 40, 28])
      risk_frames_100hz = interp(low_speed_rebound_risk, [0.38, 1.00], [0.0, 20.0])
      lock_bonus_100hz = 8 if release_lock_active else 0
      frames_100hz = int(base_frames_100hz + risk_frames_100hz + lock_bonus_100hz)
      dt_scale = clip(dt / 0.01, 0.5, 20.0)
      steps = max(1, int(frames_100hz / dt_scale))
      self.rebound_arrest_counter = max(self.rebound_arrest_counter, steps)
    elif self.rebound_arrest_counter > 0:
      self.rebound_arrest_counter -= 1

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
    self.delay_frames = clip(int(round(0.05 / max(dt, 1e-3))), 1, 25)
    self.phase = self._phase_for_speed(v_ego)
    self._update_release_lock(v_ego, a_ego, last_output_accel, max_expected_accel, dt)
    self._update_low_speed_rollout(should_stop, v_ego, dt)
    if self.phase == StoppingPhase.HOLD and v_ego <= 0.02 and a_ego > -0.05:
      self.standstill_settled_time_s = min(self.standstill_settled_time_s + dt, 5.0)
    else:
      self.standstill_settled_time_s = 0.0
    release_lock_active = self.release_lock_counter > 0
    disturbance = clip(a_ego - max_expected_accel, 0.0, 1.0)
    delay_release_guard = self._delay_release_guard(v_ego, last_output_accel)
    low_speed_rebound_risk = self._low_speed_rebound_risk(
      should_stop=should_stop,
      v_ego=v_ego,
      a_ego=a_ego,
      last_output_accel=last_output_accel,
      disturbance=disturbance,
      release_lock_active=release_lock_active,
      clutch_push_relief=False,
    )
    lock_overbrake_relief = a_ego < (min_expected_accel - 0.12)
    clutch_push_relief = (
      0.12 < v_ego < 2.5
      and last_output_accel < -0.65
      and (
        a_ego > 0.08
        or (
          v_ego < 1.0
          and a_ego > -0.25
          and last_output_accel < -0.85
        )
      )
    )
    low_speed_rebound_risk = self._low_speed_rebound_risk(
      should_stop=should_stop,
      v_ego=v_ego,
      a_ego=a_ego,
      last_output_accel=last_output_accel,
      disturbance=disturbance,
      release_lock_active=release_lock_active,
      clutch_push_relief=clutch_push_relief,
    )
    if self.phase in (StoppingPhase.NEAR_HOLD, StoppingPhase.HOLD) and not clutch_push_relief:
      desired_low_speed_accel = interp(v_ego, [0.06, 0.20, 0.50, 0.85, 1.20], [-0.26, -0.32, -0.42, -0.52, -0.60])
      shortfall = clip(a_ego - desired_low_speed_accel, 0.0, 1.2)
      rollout_trigger_i = interp(v_ego, [0.06, 0.20, 0.50, 0.85, 1.20], [0.75, 0.90, 1.15, 1.45, 1.80])
      growth = shortfall * interp(v_ego, [0.06, 0.20, 0.50, 0.85, 1.20], [1.00, 0.85, 0.65, 0.45, 0.30]) * dt
      decay = interp(v_ego, [0.06, 0.20, 0.50, 0.85, 1.20], [0.30, 0.24, 0.18, 0.14, 0.10]) * dt
      if self.low_speed_rollout_m > rollout_trigger_i and shortfall > 0.0:
        self.low_speed_recovery_i = clip(self.low_speed_recovery_i + growth, 0.0, 0.90)
      else:
        self.low_speed_recovery_i = max(self.low_speed_recovery_i - decay, 0.0)
    else:
      self.low_speed_recovery_i = 0.0
    self._update_rebound_arrest(
      should_stop=should_stop,
      v_ego=v_ego,
      a_ego=a_ego,
      last_output_accel=last_output_accel,
      disturbance=disturbance,
      release_lock_active=release_lock_active,
      low_speed_rebound_risk=low_speed_rebound_risk,
      clutch_push_relief=clutch_push_relief,
      dt=dt,
    )
    rebound_arrest_active = self.rebound_arrest_counter > 0

    rollout_trigger = interp(v_ego, [0.02, 0.25, 0.55, 1.20], [0.10, 0.20, 0.35, 0.70])
    rollout_full = interp(v_ego, [0.02, 0.25, 0.55, 1.20], [0.40, 0.65, 1.00, 2.20])
    rollout_tighten = clip(
      (self.low_speed_rollout_m - rollout_trigger) / max(rollout_full - rollout_trigger, 1e-3),
      0.0,
      1.0,
    )

    target = min(output_accel, -0.05)
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
      hold_target = interp(v_ego, [0.06, 0.15, 0.30, 0.55, 0.85], [-0.14, -0.17, -0.21, -0.19, -0.15])
      target = min(target, hold_target)
      if disturbance > 0.0:
        target -= disturbance * interp(v_ego, [0.06, 0.55], [0.12, 0.07]) * dt
      brake_step = interp(v_ego, [0.06, 0.55, 0.85], [0.006, 0.008, 0.009])
      release_step = interp(v_ego, [0.06, 0.55, 0.85], [0.0010, 0.0028, 0.0038])
    else:
      hold_target = interp(v_ego, [0.00, 0.02, 0.06], [-0.16, -0.14, -0.12])
      target = min(target, hold_target)
      if disturbance > 0.0:
        target -= disturbance * 0.08 * dt
      brake_step = interp(v_ego, [0.00, 0.06], [0.006, 0.007])
      release_step = interp(v_ego, [0.00, 0.06], [0.0018, 0.0028])

    target, release_step = self._apply_over_brake_damping(
      target=target,
      release_step=release_step,
      v_ego=v_ego,
      a_ego=a_ego,
      min_expected_accel=min_expected_accel,
      dt=dt,
    )

    rollout_rebound_guard = (
      release_lock_active
      and self.low_speed_rollout_m > 1.05
      and v_ego < 0.95
      and a_ego > 0.02
      and disturbance > 0.12
      and not clutch_push_relief
    )
    if rollout_rebound_guard:
      # Once low-speed rollout has already grown, counter rebound quickly to avoid stop creep/retry.
      guard_floor = interp(v_ego, [0.00, 0.20, 0.55, 0.95], [-0.66, -0.72, -0.80, -0.88])
      target = min(target, guard_floor)
      brake_step = max(brake_step, interp(v_ego, [0.00, 0.20, 0.55, 0.95], [0.020, 0.028, 0.036, 0.045]))
      release_step = min(release_step, interp(v_ego, [0.00, 0.20, 0.55, 0.95], [0.0008, 0.0012, 0.0018, 0.0028]))

    severe_rebound_guard = (
      self.phase in (StoppingPhase.NEAR_HOLD, StoppingPhase.HOLD)
      and self.low_speed_rollout_m > 0.80
      and v_ego < 0.95
      and a_ego > -0.05
      and disturbance > 0.10
      and not clutch_push_relief
    )
    if severe_rebound_guard:
      # When rollout is already large and decel has collapsed, apply a temporary deeper floor.
      # This specifically targets leapfrog-like stop retries without changing nominal near-hold behavior.
      severe_floor = interp(v_ego, [0.00, 0.20, 0.55, 0.95], [-0.58, -0.64, -0.72, -0.84])
      target = min(target, severe_floor)
      brake_step = max(brake_step, interp(v_ego, [0.00, 0.20, 0.55, 0.95], [0.014, 0.020, 0.028, 0.036]))
      release_step = min(release_step, interp(v_ego, [0.00, 0.20, 0.55, 0.95], [0.0009, 0.0013, 0.0019, 0.0028]))

    if self.low_speed_recovery_i > 0.0 and not clutch_push_relief:
      recovery_gain = interp(v_ego, [0.06, 0.20, 0.50, 0.85, 1.20], [0.22, 0.20, 0.17, 0.13, 0.10])
      target -= self.low_speed_recovery_i * recovery_gain
      brake_step = max(brake_step, interp(v_ego, [0.06, 0.20, 0.50, 0.85, 1.20], [0.012, 0.016, 0.022, 0.029, 0.036]))
      release_step = min(release_step, interp(v_ego, [0.06, 0.20, 0.50, 0.85, 1.20], [0.0009, 0.0012, 0.0018, 0.0026, 0.0036]))

    if clutch_push_relief:
      # Under heavy braking, some automatic gearboxes can still push the car forward.
      # Avoid ratcheting to very deep brake commands in this phase, which tends to increase end-stop jerk.
      relief_rollout = clip((self.low_speed_rollout_m - 0.80) / 1.60, 0.0, 1.0)
      relief_speed_factor = clip((0.60 - v_ego) / 0.45, 0.0, 1.0)
      relief_bias = clip((0.70 * relief_rollout) + (0.30 if release_lock_active else 0.0), 0.0, 1.0) * relief_speed_factor
      relief_target_base = interp(v_ego, [0.00, 0.60, 1.20, 1.80, 2.50], [-0.30, -0.34, -0.38, -0.42, -0.46])
      relief_target_stabilize = interp(v_ego, [0.00, 0.60, 1.20, 1.80, 2.50], [-0.44, -0.50, -0.56, -0.62, -0.68])
      relief_target = ((1.0 - relief_bias) * relief_target_base) + (relief_bias * relief_target_stabilize)
      target = max(target, relief_target)
      brake_step = min(brake_step, interp(v_ego, [0.00, 0.60, 1.20, 1.80, 2.50], [0.0015, 0.0020, 0.0026, 0.0034, 0.0042]))
      release_step_base = interp(v_ego, [0.00, 0.60, 1.20, 1.80, 2.50], [0.0215, 0.0235, 0.0255, 0.0275, 0.0295])
      release_step_stabilize = interp(v_ego, [0.00, 0.60, 1.20, 1.80, 2.50], [0.0105, 0.0125, 0.0145, 0.0155, 0.0165])
      release_step = max(release_step, ((1.0 - relief_bias) * release_step_base) + (relief_bias * release_step_stabilize))

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

    medium_decel_relief = (
      self.phase == StoppingPhase.NEAR_HOLD
      and v_ego < 0.85
      and self.low_speed_rollout_m < 1.30
      and a_ego < -0.70
      and -0.85 < last_output_accel < -0.45
      and not clutch_push_relief
    )
    if medium_decel_relief:
      # For medium-deep commands near hold, stop ratcheting down once decel is already strong.
      medium_relief_target = interp(v_ego, [0.06, 0.20, 0.50, 0.85], [-0.40, -0.44, -0.50, -0.56])
      target = max(target, medium_relief_target)
      brake_step = min(brake_step, interp(v_ego, [0.06, 0.20, 0.50, 0.85], [0.0015, 0.0019, 0.0025, 0.0031]))
      release_step = max(release_step, interp(v_ego, [0.06, 0.20, 0.50, 0.85], [0.0090, 0.0100, 0.0112, 0.0124]))

    deep_command_jerk_relief = (
      self.phase in (StoppingPhase.NEAR_HOLD, StoppingPhase.HOLD)
      and self.low_speed_rollout_m < 0.60
      and v_ego < 0.75
      and a_ego < -0.90
      and last_output_accel < -0.95
      and not clutch_push_relief
    )
    if deep_command_jerk_relief:
      # Deep inherited brake commands can create harsh end-stop jerk; unwind earlier in this narrow case.
      deep_relief_target = interp(v_ego, [0.00, 0.20, 0.50, 0.75], [-0.76, -0.80, -0.88, -0.95])
      target = max(target, deep_relief_target)
      brake_step = min(brake_step, interp(v_ego, [0.00, 0.20, 0.50, 0.75], [0.0012, 0.0016, 0.0021, 0.0026]))
      release_step = max(release_step, interp(v_ego, [0.00, 0.20, 0.50, 0.75], [0.016, 0.018, 0.021, 0.024]))

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

    if rollout_tighten > 0.0 and not clutch_push_relief:
      release_cap = interp(v_ego, [0.02, 0.25, 0.55, 1.20], [0.0010, 0.0018, 0.0030, 0.0050])
      release_step = min(release_step, release_cap)
      # Only tighten target when decel is weak; otherwise allow a softer landing even if rollout is building.
      if a_ego > -0.35:
        target -= rollout_tighten * interp(v_ego, [0.02, 0.25, 0.55, 1.20], [0.05, 0.08, 0.11, 0.12])
        rollout_floor = interp(v_ego, [0.02, 0.12, 0.25, 0.55, 1.20], [-0.30, -0.27, -0.24, -0.19, -0.13])
        target = min(target, rollout_floor + ((1.0 - rollout_tighten) * 0.05))

    rollout_push = (
      rollout_tighten > 0.05
      and v_ego < 1.2
      and a_ego > -0.30
      and not clutch_push_relief
      and (
        v_ego > 0.18
        or self.low_speed_rollout_m > 1.00
        or disturbance > 0.05
        or release_lock_active
        or (self.low_speed_rollout_m > 1.70 and low_speed_rebound_risk > 0.20)
      )
    )
    if rollout_push:
      # If rollout is building while decel remains weak, enforce a firmer low-speed brake floor.
      push_floor = interp(v_ego, [0.06, 0.20, 0.50, 0.85, 1.20], [-0.50, -0.46, -0.40, -0.34, -0.28])
      target = min(target, push_floor)
      brake_step = max(brake_step, rollout_tighten * interp(v_ego, [0.06, 0.20, 0.50, 0.85, 1.20], [0.024, 0.020, 0.016, 0.013, 0.010]))
      release_step = min(release_step, interp(v_ego, [0.06, 0.20, 0.50, 0.85, 1.20], [0.0010, 0.0014, 0.0020, 0.0028, 0.0038]))

    rollout_near_limit_guard = (
      self.phase in (StoppingPhase.NEAR_HOLD, StoppingPhase.HOLD)
      and self.low_speed_rollout_m > 1.60
      and v_ego < 0.80
      and a_ego > -0.45
      and not clutch_push_relief
    )
    if rollout_near_limit_guard:
      # Once rollout is already near the 2m limit, apply a mild floor to avoid drifting over.
      near_limit_floor = interp(v_ego, [0.12, 0.30, 0.60, 0.80], [-0.40, -0.36, -0.30, -0.24])
      target = min(target, near_limit_floor)
      brake_step = max(brake_step, interp(v_ego, [0.12, 0.30, 0.60, 0.80], [0.012, 0.009, 0.006, 0.004]))
      release_step = min(release_step, interp(v_ego, [0.12, 0.30, 0.60, 0.80], [0.0012, 0.0018, 0.0030, 0.0040]))

    rollout_relief_guard = (
      self.phase in (StoppingPhase.NEAR_HOLD, StoppingPhase.HOLD)
      and self.low_speed_rollout_m > interp(v_ego, [0.06, 0.25, 0.60, 1.00], [0.75, 0.90, 1.20, 1.60])
      and v_ego < 1.0
      and a_ego > -0.25
      and not clutch_push_relief
    )
    if rollout_relief_guard:
      # Keep command away from the low-magnitude relief region while rollout is already elevated.
      relief_floor = interp(v_ego, [0.06, 0.25, 0.60, 1.00], [-0.38, -0.42, -0.48, -0.56])
      target = min(target, relief_floor)
      brake_step = max(brake_step, interp(v_ego, [0.06, 0.25, 0.60, 1.00], [0.010, 0.014, 0.020, 0.026]))
      release_step = min(release_step, interp(v_ego, [0.06, 0.25, 0.60, 1.00], [0.0009, 0.0012, 0.0018, 0.0028]))

    high_rollout_low_speed_unwind = (
      self.phase in (StoppingPhase.NEAR_HOLD, StoppingPhase.HOLD)
      and self.low_speed_rollout_m > 1.50
      and 0.12 < v_ego < 0.55
      and release_lock_active
      and a_ego > -0.20
      and disturbance < 0.20
      and not clutch_push_relief
    )
    if high_rollout_low_speed_unwind:
      # For sustained high-rollout rebound cycles, avoid staying at very deep low-speed command.
      # On current fitted dynamics, a milder command in this narrow window yields lower rebound risk.
      unwind_cap = interp(v_ego, [0.12, 0.25, 0.40, 0.55], [-0.30, -0.28, -0.27, -0.30])
      target = max(target, unwind_cap)
      brake_step = min(brake_step, interp(v_ego, [0.12, 0.25, 0.40, 0.55], [0.0015, 0.0019, 0.0023, 0.0028]))
      release_step = max(release_step, interp(v_ego, [0.12, 0.25, 0.40, 0.55], [0.010, 0.012, 0.014, 0.016]))

    comfortable_unwind = (
      self.phase in (StoppingPhase.NEAR_HOLD, StoppingPhase.HOLD)
      and v_ego < 0.30
      and a_ego < -0.45
      and not release_lock_active
      and not clutch_push_relief
    )
    if delay_release_guard > 0.0 and not comfortable_unwind and not clutch_push_relief:
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

    lock_soft_relax = (
      release_lock_active
      and self.phase in (StoppingPhase.NEAR_HOLD, StoppingPhase.HOLD)
      and v_ego < 0.18
      and a_ego < -0.55
      and self.low_speed_rollout_m < 1.10
      and low_speed_rebound_risk < 0.18
      and disturbance < 0.08
      and not clutch_push_relief
      and not lock_overbrake_relief
    )

    if release_lock_active:
      if lock_overbrake_relief:
        release_step = max(release_step, interp(v_ego, [0.00, 0.10, 0.30, 0.70, 1.20], [0.0100, 0.0115, 0.0135, 0.0160, 0.0180]))
      elif clutch_push_relief:
        release_step = min(release_step, interp(v_ego, [0.00, 0.60, 1.20, 1.80, 2.50], [0.0195, 0.0215, 0.0235, 0.0255, 0.0275]))
      else:
        if lock_soft_relax:
          release_step = max(release_step, interp(v_ego, [0.00, 0.08, 0.18], [0.016, 0.013, 0.010]))
          lock_floor = interp(v_ego, [0.00, 0.08, 0.18], [-0.27, -0.25, -0.22])
        else:
          release_step = min(release_step, interp(v_ego, [0.00, 0.20, 0.50, 1.20], [0.0010, 0.0015, 0.0030, 0.0060]))
          lock_floor = interp(v_ego, [0.00, 0.12, 0.25, 0.50, 1.20], [-0.34, -0.31, -0.26, -0.18, -0.11])
        target = min(target, lock_floor)

    soft_landing_release = (
      self.phase in (StoppingPhase.NEAR_HOLD, StoppingPhase.HOLD)
      and v_ego < 0.85
      and a_ego < -0.50
      and last_output_accel < -0.55
      and (not release_lock_active or lock_soft_relax)
      and not clutch_push_relief
    )
    if soft_landing_release:
      # If decel is already strong at very low speed, unwind toward a softer landing.
      # This limits the acceleration step at wheel-stop without disabling the disturbance/rollout guards.
      soft_target = interp(v_ego, [0.06, 0.20, 0.40, 0.85], [-0.12, -0.18, -0.26, -0.38])
      target = max(target, soft_target)
      brake_step = min(brake_step, interp(v_ego, [0.06, 0.85], [0.0022, 0.0032]))
      release_step = max(release_step, interp(v_ego, [0.06, 0.85], [0.010, 0.016]))

    creep_rebound_guard = (
      should_stop
      and self.phase in (StoppingPhase.NEAR_HOLD, StoppingPhase.HOLD)
      and 0.0 < v_ego < 0.25
      and (release_lock_active or a_ego > 0.04 or disturbance > 0.14)
      and not clutch_push_relief
    )
    if creep_rebound_guard:
      # If we rebound/creep while we still expect to stop, allow a slightly deeper low-speed brake cap.
      # This helps counter automatic clutch/drivetrain push without permanently increasing wheel-stop command magnitude.
      creep_cap = interp(v_ego, [0.02, 0.08, 0.25], [-0.32, -0.36, -0.48])
      target = min(target, creep_cap)
      brake_step = max(brake_step, interp(v_ego, [0.02, 0.08, 0.25], [0.010, 0.014, 0.020]))
      release_step = min(release_step, interp(v_ego, [0.02, 0.08, 0.25], [0.0012, 0.0016, 0.0024]))

    low_speed_rebound_cap: float | None = None
    low_speed_rebound_cap_active = (
      low_speed_rebound_risk > 0.0
      and not clutch_push_relief
      and (v_ego < 0.045 or a_ego > -0.08 or disturbance > 0.10 or release_lock_active)
    )
    if low_speed_rebound_cap_active:
      risk_floor = interp(v_ego, [0.00, 0.03, 0.08, 0.25], [-0.44, -0.40, -0.34, -0.28])
      low_speed_rebound_cap = (-0.275 * (1.0 - low_speed_rebound_risk)) + (risk_floor * low_speed_rebound_risk)
      target = min(target, low_speed_rebound_cap)
      brake_step = max(brake_step, interp(low_speed_rebound_risk, [0.0, 1.0], [0.008, 0.016]))
      release_step = min(release_step, interp(low_speed_rebound_risk, [0.0, 1.0], [0.0018, 0.0010]))

    rebound_arrest_cap: float | None = None
    if rebound_arrest_active and not clutch_push_relief:
      rebound_arrest_cap = interp(v_ego, [0.00, 0.03, 0.06, 0.08], [-1.40, -1.15, -0.85, -0.56])
      target = min(target, rebound_arrest_cap)
      brake_step = max(brake_step, interp(v_ego, [0.00, 0.08], [0.040, 0.022]))
      release_step = min(release_step, interp(v_ego, [0.00, 0.08], [0.0008, 0.0014]))

    end_stop_brake_cap = interp(v_ego, [0.00, 0.10, 0.15, 0.25, 0.60], [-0.255, -0.255, -0.38, -0.63, -1.08])
    low_speed_rebound_cap_relief = (
      self.phase in (StoppingPhase.NEAR_HOLD, StoppingPhase.HOLD)
      and v_ego < 0.12
      and a_ego > -0.45
      and self.low_speed_rollout_m < 0.90
      and low_speed_rebound_risk > 0.20
      and (release_lock_active or disturbance > 0.05 or self.low_speed_rollout_m > 1.20 or a_ego > -0.12)
      and not clutch_push_relief
    )
    if low_speed_rebound_cap_relief:
      # If near-standstill decel has become weak, avoid forcing an early unwind to the nominal -0.275 cap.
      # This keeps a little more brake authority while stop intent still holds and reduces rebound risk.
      rebound_relief_cap = interp(low_speed_rebound_risk, [0.20, 1.00], [-0.436, -0.536])
      end_stop_brake_cap = min(end_stop_brake_cap, rebound_relief_cap)
      release_step = min(release_step, interp(v_ego, [0.00, 0.04, 0.12], [0.0007, 0.0010, 0.0014]))
    low_rollout_soft_landing_cap = (
      self.phase in (StoppingPhase.NEAR_HOLD, StoppingPhase.HOLD)
      and v_ego < 0.22
      and self.low_speed_rollout_m < 1.50
      and low_speed_rebound_risk < 0.25
      and not rebound_arrest_active
      and not clutch_push_relief
      and (not release_lock_active or disturbance < 0.08)
    )
    if low_rollout_soft_landing_cap:
      # In low-rollout/low-rebound-risk stops, unwind deep near-hold command a bit earlier.
      # This targets end-stop jerk without weakening the high-rollout rebound guards.
      soft_landing_cap = interp(v_ego, [0.00, 0.08, 0.14, 0.22], [-0.225, -0.235, -0.28, -0.36])
      end_stop_brake_cap = max(end_stop_brake_cap, soft_landing_cap)
      release_step = max(release_step, interp(v_ego, [0.00, 0.08, 0.14, 0.22], [0.030, 0.024, 0.019, 0.014]))
    moderate_decel_soft_cap = (
      self.phase in (StoppingPhase.NEAR_HOLD, StoppingPhase.HOLD)
      and v_ego < 0.20
      and a_ego < -0.55
      and self.low_speed_rollout_m < 0.90
      and low_speed_rebound_risk < 0.15
      and not release_lock_active
      and not rebound_arrest_active
      and not clutch_push_relief
    )
    if moderate_decel_soft_cap:
      # For low-risk near-hold decel, keep end-stop cap slightly softer to reduce standstill cmd jerk.
      end_stop_brake_cap = max(end_stop_brake_cap, -0.275)
    strong_decel_soft_cap = (
      v_ego < 0.20
      and a_ego < -0.70
      and self.low_speed_rollout_m < 0.80
      and not release_lock_active
      and not clutch_push_relief
    )
    if strong_decel_soft_cap:
      end_stop_brake_cap = max(end_stop_brake_cap, -0.275)
    if creep_rebound_guard:
      end_stop_brake_cap = min(end_stop_brake_cap, creep_cap)
    if low_speed_rebound_cap is not None:
      end_stop_brake_cap = min(end_stop_brake_cap, low_speed_rebound_cap)
    if rebound_arrest_cap is not None:
      end_stop_brake_cap = min(end_stop_brake_cap, rebound_arrest_cap)
    end_stop_cap_active = (
      self.phase in (StoppingPhase.NEAR_HOLD, StoppingPhase.HOLD)
      and (v_ego < 0.60 or (v_ego < 0.65 and last_output_accel < -0.95))
      and not clutch_push_relief
      and (target < end_stop_brake_cap or last_output_accel < end_stop_brake_cap)
    )
    if end_stop_cap_active:
      # Clamp inherited deep brake commands near wheel-stop; use a higher release rate to reach the cap before standstill.
      # This is intended to reduce end-stop harshness while keeping enough authority to prevent large low-speed rollout.
      target = max(target, end_stop_brake_cap)
      suppress_fast_end_stop_release = (
        v_ego < 0.20
        and (
          release_lock_active
          or rebound_arrest_active
          or low_speed_rebound_risk > 0.45
        )
      )
      if not suppress_fast_end_stop_release:
        release_step = max(release_step, interp(v_ego, [0.00, 0.60], [0.020, 0.007]))

    standstill_relax = (
      self.phase == StoppingPhase.HOLD
      and v_ego <= 0.02
      and a_ego > -0.05
      and self.standstill_settled_time_s >= 0.8
      and not release_lock_active
      and not clutch_push_relief
    )
    if standstill_relax:
      # Once vEgo is essentially zero and accel is settled, relax toward a mild hold.
      # This reduces the acceleration step at wheel-stop while relying on disturbance lock to counter creep.
      relax_target = interp(v_ego, [0.00, 0.02], [-0.12, -0.10])
      target = max(target, relax_target)
      release_step = max(release_step, interp(v_ego, [0.00, 0.02], [0.0045, 0.0035]))

    ineffective_brake_guard = (
      self.phase in (StoppingPhase.NEAR_HOLD, StoppingPhase.HOLD)
      and v_ego < 0.35
      and a_ego > -0.35
      and last_output_accel < -1.05
      and not clutch_push_relief
    )
    if ineffective_brake_guard:
      # If we're already commanding deep braking but decel remains weak, avoid ratcheting further down.
      # This mitigates end-stop jerk spikes when drivetrain/clutch behavior flips near standstill.
      target = max(target, last_output_accel)
      brake_step = min(brake_step, interp(v_ego, [0.06, 0.20, 0.35], [0.0012, 0.0016, 0.0022]))
      release_step = max(release_step, interp(v_ego, [0.06, 0.20, 0.35], [0.0050, 0.0040, 0.0030]))

    rollout_oscillation_damping = (
      self.low_speed_rollout_m > interp(v_ego, [0.08, 0.25, 0.60, 1.00], [1.30, 1.60, 1.75, 1.80])
      and v_ego < 1.8
      and (release_lock_active or a_ego > -0.25 or disturbance > 0.10)
      and not clutch_push_relief
    )
    if rollout_oscillation_damping:
      # In high-rollout rebound cycles, prefer steadier command evolution over aggressive chase/relief swings.
      damping_floor = interp(v_ego, [0.00, 0.20, 0.55, 1.20], [-0.36, -0.42, -0.50, -0.60])
      target = min(target, damping_floor)
      brake_step = min(brake_step, interp(v_ego, [0.00, 0.20, 0.55, 1.20], [0.0023, 0.0032, 0.0042, 0.0051]))
      release_step = min(release_step, interp(v_ego, [0.00, 0.20, 0.55, 1.20], [0.0020, 0.0030, 0.0040, 0.0050]))

    brake_step = max(0.0004, brake_step)
    release_step = max(0.0004, release_step)

    # `brake_step`/`release_step` are tuned for the 100Hz control loop (dt ~= 0.01s).
    # Scale by dt so offline replays sampled at lower rates behave comparably.
    dt_scale = clip(dt / 0.01, 0.5, 20.0)
    brake_step *= dt_scale
    release_step *= dt_scale

    limited_output = clip(target, last_output_accel - brake_step, last_output_accel + release_step)
    limited_output = clip(limited_output, stop_accel, -0.05)
    return StoppingResult(output_accel=limited_output, release_lock_active=release_lock_active)
