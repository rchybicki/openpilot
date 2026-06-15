from __future__ import annotations

import numpy as np
from dataclasses import dataclass
from enum import IntEnum

from openpilot.selfdrive.controls.lib.stopping_shadow import StoppingShadowInput, StoppingShadowOracle

clip = np.clip
interp = np.interp

COMMAND_HISTORY_LEN = 48

# Hold-acquisition soften (2026-06-10, driveway route 00001702--dcdc5c3eea--0): on engage-at-
# standstill / creep re-stop the rebound-arrest lane ramped the command from ~-0.5..-0.78 down to
# the -1.05 hold at 3.0-3.2 m/s^3 while the car was rolling out at < 0.045 m/s -- the felt brake
# shock at wheel-stop. In the stationary-stable regime (ALL gates below hold) the deepening toward
# the full hill-hold depth is comfort-shaped instead. IMPORTANT sensing caveat: the v/a_ego/
# disturbance gates read QUIET during the sensor-blind window of a fresh grade re-roll (wheel-speed
# deadband ~0.08 m/s + ~0.1 s transport + accel-filter lag), so the gates alone CANNOT protect the
# hill-hold catch. Two additional safeguards keep worst-case 10%-grade rollback within ~4 cm of
# legacy (command-domain sim, arrest latched at -0.23, actuator tau 0.2 s):
#   1. the soften only arms once the command is already deep (last_output_accel < LAST_CMD_MAX);
#      the shallow catch -0.22..-0.55 always runs at the full arrest rate, and
#   2. while rebound_arrest_active (live memory of recent rebound/push evidence) the deepening is
#      floored at ARREST_BRAKE_STEP (2.0 m/s^3) instead of the 1.0 m/s^3 comfort rate.
# The felt driveway slam was the -0.5..-0.78 -> -1.05 segment, which both safeguards leave softened.
# Mirrored as documented params in stopping_params.py (row 40) so V2 inherits the numbers.
HOLD_ACQUISITION_SOFTEN_V_MAX = 0.05              # m/s; stationary band -- the lane is provably unreachable above this (and far below 0.5 m/s)
HOLD_ACQUISITION_SOFTEN_A_EGO_MAX = 0.30          # m/s^2; |a_ego| stability band (grade pull / creep surge reads as motion)
HOLD_ACQUISITION_SOFTEN_DISTURBANCE_MAX = 0.04    # m/s^2; live (a_ego - max_expected) gate = the v<0.08 release-lock threshold (:99)
HOLD_ACQUISITION_SOFTEN_LAST_CMD_MAX = -0.55      # m/s^2; soften only deep ramps -- preserves full arrest authority for the shallow blind-window catch
HOLD_ACQUISITION_SOFTEN_BRAKE_STEP = 0.010        # m/s^2 per 100 Hz frame = 1.0 m/s^3 comfort deepening rate
HOLD_ACQUISITION_SOFTEN_ARREST_BRAKE_STEP = 0.020  # m/s^2 per frame = 2.0 m/s^3 floor while rebound_arrest_active (hill-hold rollback bound)

# --- Cranked comfort requirement P2: terminal settle jerk cap (2026-06-13) -------------------
# The user feels the disc-grab as the pads bite / static friction grabs at the ~0.05 m/s settle.
# The MEASURED settle jerk exceeds the command jerk by a median ~+2..+3 m/s^3 (brake-pad stiction,
# beyond the command), but the COMMAND-side contribution is the per-frame deepening rate the
# stopping lane applies through the settle band -- which is the only part the controller can shape.
# This caps that deepening RATE across the whole terminal settle band (v_ego < TERMINAL_SETTLE_V_MAX,
# wider than the v<0.05 hold-acquisition band), bounding the commanded jerk to J_TERMINAL_SETTLE.
# Same two-regime discipline as hold-acquisition: it is a RATE cap only (targets/floors untouched,
# full hold force still reached), it never softens while a live disturbance / rebound arrest is
# active (those protect against creep/rollback and must keep their fast deepening), and the cap is
# floored at the arrest rate whenever the arrest is latched. The named jerk budget is the knob the
# user iterates downward; it is set just under the eval's terminal_max_settle_meas_jerk minus the
# measured-over-command stiction excess so a clean command produces a settle the gate accepts.
TERMINAL_SETTLE_V_MAX = 0.20               # m/s; terminal settle band where the felt grab lives (above the 0.05 hold-acq band)
J_TERMINAL_SETTLE = 1.5                     # m/s^3; commanded deepening-rate ceiling in the settle band (knob: crank downward as the user iterates)
TERMINAL_SETTLE_BRAKE_STEP = J_TERMINAL_SETTLE / 100.0          # m/s^2 per 100 Hz frame
TERMINAL_SETTLE_DISTURBANCE_MAX = 0.04     # m/s^2; live-push gate (= DIST_PUSH_THRESH_LOW) -- never soften a live creep/rollback push

# --- Cranked comfort requirement P1: gentle-approach decel cap (2026-06-13) -------------------
# Mirror of the longcontrol P1 cap so the ACTIVE stopping lane (the producer the sim exercises)
# enforces the SAME requirement the eval scores: while the lead gap is still comfortable
# (> APPROACH_DECEL_CAP_GAP_FLOOR_M) and the car is still rolling, the commanded decel stays
# <= APPROACH_DECEL_CAP_MPS2 UNLESS the lead kinematics require more (closing^2/(2*max(gap-floor,
# eps)) > cap), in which case the floor relaxes toward the required decel so a real closing threat
# is always braked in time. Same physics as the eval exemption (build_event_store.approach_decel_over_gap2m
# / scoring_config.CrankedComfortThresholds) and the longcontrol cap, so producer and gate agree.
# Applied as a final least-negative floor on limited_output, rate-limited so it injects no jerk.
APPROACH_DECEL_CAP_MPS2 = 0.5              # user's stated gentle-approach decel cap (m/s^2 magnitude)
APPROACH_DECEL_CAP_GAP_FLOOR_M = 2.0       # lead gap above which braking is expected to be gentle (m)
APPROACH_DECEL_CAP_V_EGO_MIN = 0.30        # below this the terminal settle/hold lanes own the command (m/s)
APPROACH_DECEL_CAP_RELEASE_MARGIN = 0.18   # m/s^2 slack on required_decel before fully releasing the cap (> eval's 0.12)
APPROACH_DECEL_CAP_RELEASE_STEP = 0.020    # m/s^2 per 100 Hz frame = 2.0 m/s^3 cap on how fast the floor relaxes toward the raw command
# SAFETY REVERT 2026-06-15: DISABLED (see longcontrol.py for the full incident note). The cap's
# kinematic release ignores lead deceleration and under-braked into a decelerating lead
# (route 00001725, near-collision takeover). Both producers are gated off until redesigned.
APPROACH_DECEL_CAP_ENABLED = False
# (releasing brake; matches HOLD_ACQUISITION_SOFTEN_ARREST_BRAKE_STEP -- the established gentle comfort/safety rate)

# --- Anti-stiction terminal pre-release (2026-06-14) -----------------------------------------
# The terminal "disc-grab" is the static-friction G-jolt as the car settles to rest (on-road IMU
# settle_peak_imu_jerk baseline ~24 m/s^3 at the 0.04 handoff, one gate-0.01 driveway settle hit
# 48). Data shows the SCC handoff is NOT the cause (grab pervasive whoever commands the stop) ->
# it is friction-transition physics, and gate-0.01 now puts openpilot in command of the terminal
# so we can SHAPE it. The mechanism: in the final approach to standstill -- ABOVE the gate-0.01
# StopReq handoff and ABOVE the standstill band, in openpilot's command domain -- EASE the brake
# command OFF the deep terminal hold toward a shallower floor (-A_TERMINAL_PRERELEASE, never to
# zero or positive), jerk-limited at J_TERMINAL_PRERELEASE so the ease itself injects no jolt, so
# the car eases THROUGH the static-friction transition instead of clamping hard into it. As v_ego
# crosses into the standstill band (<= V_LO) the pre-release disengages and the existing hold /
# end-stop / hold-acquisition stack re-applies the full hold depth so the car is firmly held.
#
# CRITICAL HONESTY ON SIM: the offline plant model maps command->modeled aEgo; it does NOT model
# brake-pad stiction or produce the livePose IMU signal, so the SIM CANNOT prove the felt-grab
# (settle_peak_imu_jerk) is reduced -- that is an ON-ROAD measurement for the next drive. The
# sim's job is only to verify (a) the command does the ease-then-rehold shape and (b) the SAFETY
# invariants via the plant (still reaches standstill promptly, full hold re-applied, no
# rollback/creep), and (c) no regression on existing comfort metrics / no under-braking.
#
# SAFETY -- two-regime gating, the SAME discipline as the hold-acquisition soften: the ease is a
# RELEASE-side FLOOR only (it can only ever make the command SHALLOWER toward the floor, never
# deeper, never below the floor), and it is fully DISABLED the instant any of: a live disturbance
# (a_ego vs expected) >= TERMINAL_PRERELEASE_DISTURBANCE_MAX, the release-inhibit lock,
# rebound-arrest, clutch-push relief, elevated low-speed rebound risk, OR insufficient decel
# (a_ego >= -TERMINAL_PRERELEASE_A_EGO_MAX, i.e. the car is not provably still decelerating toward
# the stop -- a creep/grade-pull reads here). On a downhill grade these flip the same frame the
# pull appears, restoring full brake; the ease NEVER reduces brake below what holds/stops the car
# because it floors at -A_TERMINAL_PRERELEASE (a real holding decel) and the deepening lanes below
# V_LO immediately re-apply the full hold. Mirrored as documented params in stopping_params.py
# (row 42) so the V2 facade inherits the numbers.
TERMINAL_PRERELEASE_V_LO = 0.06            # m/s; lower edge -- at/below this the standstill hold/end-stop stack re-applies the full hold
TERMINAL_PRERELEASE_V_HI = 0.30            # m/s; upper edge -- above this the approach/glide lanes own the command
A_TERMINAL_PRERELEASE = 0.30               # m/s^2; shallow ease floor (least-negative the command may take) -- never zero/positive, a real holding decel.
# Chosen at -0.30 (within the -0.25..-0.35 spec band): deep enough that the ease lands close to where
# the standstill re-grab settles (no double-shape notch -- the end_stop_accel_step on the weak-decel
# no-target 721 settle stays under the regression budget), yet shallow enough to ease a deep -0.4..-0.6
# terminal hold OFF through the friction-transition band. Just above the -0.275 end-stop nominal soft cap.
J_TERMINAL_PRERELEASE = 1.5                # m/s^3; release-side jerk ceiling on the ease so it injects no jolt
TERMINAL_PRERELEASE_RELEASE_STEP = J_TERMINAL_PRERELEASE / 100.0   # m/s^2 per 100 Hz frame
TERMINAL_PRERELEASE_DISTURBANCE_MAX = 0.04  # m/s^2; live-push gate (= DIST_PUSH_THRESH_LOW / HOLD_ACQUISITION_SOFTEN_DISTURBANCE_MAX)
TERMINAL_PRERELEASE_A_EGO_MAX = 0.10        # m/s^2; require a_ego <= -0.10 (provably decelerating); a creep/grade-pull reads weaker and disables the ease
TERMINAL_PRERELEASE_REBOUND_RISK_MAX = 0.08  # disable above this low-speed rebound risk (= the quiescent gate used by clean_settle/distance_carry)
# m; only ease in the GENUINE final settle (at the stop point) -- never while an explicit target is
# still materially ahead (that would under-brake / overshoot the target)
TERMINAL_PRERELEASE_REMAINING_MAX = 0.30


def approach_decel_cap_required_decel(v_ego: float, lead_v: float, lead_d_rel: float) -> float:
  """Kinematic decel required to bleed closing speed to zero before the gap reaches the floor;
  identical physics to build_event_store.approach_decel_over_gap2m and the longcontrol cap."""
  closing = max(float(v_ego) - float(lead_v), 0.0)
  return (closing * closing) / (2.0 * max(float(lead_d_rel) - APPROACH_DECEL_CAP_GAP_FLOOR_M, 0.1))


def stopping_phase_approach_decel_floor(v_ego: float, lead_status: bool, lead_v: float,
                                        lead_d_rel: float | None) -> float | None:
  """Negative accel floor (least-negative the command may take) for the gentle-approach cap, or
  None when it does not apply this frame (no/closer-than-floor lead, terminal band, or
  kinematically released)."""
  if not lead_status or lead_d_rel is None:
    return None
  if lead_d_rel <= APPROACH_DECEL_CAP_GAP_FLOOR_M or v_ego <= APPROACH_DECEL_CAP_V_EGO_MIN:
    return None
  required_decel = approach_decel_cap_required_decel(v_ego, lead_v, lead_d_rel)
  effective_cap_mag = max(APPROACH_DECEL_CAP_MPS2, required_decel + APPROACH_DECEL_CAP_RELEASE_MARGIN)
  return -float(effective_cap_mag)


class StoppingPhase(IntEnum):
  APPROACH = 0
  NEAR_HOLD = 1
  HOLD = 2


@dataclass
class StoppingResult:
  output_accel: float
  release_lock_active: bool


@dataclass
class StoppingCommandEnvelope:
  target: float
  brake_step: float
  release_step: float


@dataclass(frozen=True)
class StoppingControllerTuning:
  hold_speed_mps: float = 0.06
  near_hold_speed_mps: float = 0.85
  command_history_len: int = COMMAND_HISTORY_LEN
  standstill_rollout_decay_mps: float = 0.35
  standstill_settle_speed_mps: float = 0.02
  standstill_settle_accel_threshold_mps2: float = -0.05
  standstill_relax_time_s: float = 0.8


class StoppingController:
  """Stateful stop controller with explicit low-speed phases and disturbance lock."""

  def __init__(self, tuning: StoppingControllerTuning | None = None) -> None:
    self.tuning = tuning or StoppingControllerTuning()
    self.phase = StoppingPhase.APPROACH
    self.stop_entry_soften_counter = 0
    self.stop_reacquire_hold_counter = 0
    self.explicit_target_soft_entry_carry_counter = 0
    self.explicit_target_early_entry_capture_counter = 0
    self.late_no_target_stop_entry_capture_counter = 0
    self._last_stop_intent = False
    self._last_should_stop = False
    self.release_lock_counter = 0
    self.rebound_arrest_counter = 0
    self.tail_commit_counter = 0
    self.low_speed_recovery_i = 0.0
    self.low_speed_rollout_m = 0.0
    self.standstill_settled_time_s = 0.0
    self.delay_frames = 5
    self._command_history: list[float] = []
    self.shadow_oracle = StoppingShadowOracle()

  def reset(self) -> None:
    self.phase = StoppingPhase.APPROACH
    self.stop_entry_soften_counter = 0
    self.stop_reacquire_hold_counter = 0
    self.explicit_target_soft_entry_carry_counter = 0
    self.explicit_target_early_entry_capture_counter = 0
    self.late_no_target_stop_entry_capture_counter = 0
    self._last_stop_intent = False
    self._last_should_stop = False
    self.release_lock_counter = 0
    self.rebound_arrest_counter = 0
    self.tail_commit_counter = 0
    self.low_speed_recovery_i = 0.0
    self.low_speed_rollout_m = 0.0
    self.standstill_settled_time_s = 0.0
    self._command_history = []

  def seed_command_history(self, commands: list[float]) -> None:
    history = [float(cmd) for cmd in commands]
    max_len = int(self.tuning.command_history_len)
    self._command_history = history[-max_len:] if len(history) > max_len else history

  def _phase_for_speed(self, v_ego: float) -> StoppingPhase:
    if v_ego <= self.tuning.hold_speed_mps:
      return StoppingPhase.HOLD
    if v_ego <= self.tuning.near_hold_speed_mps:
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

  def _update_stop_entry_soften(
    self,
    raw_should_stop: bool,
    tail_commit_stop_latch: bool,
    v_ego: float,
    a_ego: float,
    last_output_accel: float,
    dt: float,
  ) -> tuple[bool, bool]:
    if not raw_should_stop:
      self._last_should_stop = False
      self.stop_entry_soften_counter = 0
      return False, False

    new_stop_entry = not self._last_should_stop
    self._last_should_stop = True
    if not new_stop_entry or tail_commit_stop_latch:
      return self.stop_entry_soften_counter > 0, new_stop_entry

    entry_soften_candidate = (
      0.12 < v_ego < 1.65
      and a_ego > -0.60
      and (
        last_output_accel > 0.02
        or last_output_accel < -0.28
      )
      and last_output_accel > -0.45
    )
    if not entry_soften_candidate:
      self.stop_entry_soften_counter = 0
      return False, new_stop_entry

    frames_100hz = int(interp(v_ego, [0.12, 0.30, 0.60, 1.00, 1.65], [28, 24, 20, 18, 16]))
    dt_scale = clip(dt / 0.01, 0.5, 20.0)
    self.stop_entry_soften_counter = max(1, int(frames_100hz / dt_scale))
    return True, new_stop_entry

  def _update_stop_reacquire_hold(
    self,
    should_stop: bool,
    new_stop_entry: bool,
    v_ego: float,
    a_ego: float,
    last_output_accel: float,
    dt: float,
  ) -> bool:
    if not should_stop:
      self.stop_reacquire_hold_counter = 0
      return False

    reacquire_candidate = (
      new_stop_entry
      and 0.05 < v_ego < 0.95
      and -1.10 < a_ego < -0.18
      and -1.10 < last_output_accel < -0.55
    )
    if reacquire_candidate:
      frames_100hz = int(interp(v_ego, [0.05, 0.15, 0.35, 0.65, 0.95], [60, 56, 48, 40, 32]))
      dt_scale = clip(dt / 0.01, 0.5, 20.0)
      self.stop_reacquire_hold_counter = max(1, int(frames_100hz / dt_scale))
    elif self.stop_reacquire_hold_counter > 0:
      self.stop_reacquire_hold_counter -= 1

    return self.stop_reacquire_hold_counter > 0

  def _update_late_no_target_stop_entry_capture(
    self,
    should_stop: bool,
    new_stop_entry: bool,
    v_ego: float,
    a_ego: float,
    last_output_accel: float,
    distance_to_stop_target_m: float | None,
    dt: float,
  ) -> bool:
    explicit_stop_target_available = distance_to_stop_target_m is not None and distance_to_stop_target_m >= 0.0
    if not should_stop or explicit_stop_target_available:
      self.late_no_target_stop_entry_capture_counter = 0
      return False

    late_entry_candidate = (
      new_stop_entry
      and 0.55 < v_ego < 0.95
      and -0.18 < a_ego < 0.10
      and -0.55 < last_output_accel < -0.22
    )
    if late_entry_candidate:
      frames_100hz = int(interp(v_ego, [0.55, 0.70, 0.85, 0.95], [32, 28, 24, 20]))
      dt_scale = clip(dt / 0.01, 0.5, 20.0)
      self.late_no_target_stop_entry_capture_counter = max(1, int(frames_100hz / dt_scale))
    elif self.late_no_target_stop_entry_capture_counter > 0:
      self.late_no_target_stop_entry_capture_counter -= 1

    return self.late_no_target_stop_entry_capture_counter > 0

  def _update_explicit_target_early_entry_capture(
    self,
    should_stop: bool,
    raw_should_stop: bool,
    v_ego: float,
    a_ego: float,
    last_output_accel: float,
    distance_to_stop_target_m: float | None,
    dt: float,
  ) -> bool:
    if not should_stop:
      self._last_stop_intent = False
      self.explicit_target_early_entry_capture_counter = 0
      return False

    new_stop_intent = not self._last_stop_intent
    self._last_stop_intent = True
    explicit_stop_target_available = distance_to_stop_target_m is not None and distance_to_stop_target_m >= 0.0
    if raw_should_stop or not explicit_stop_target_available:
      self.explicit_target_early_entry_capture_counter = 0
      return False

    early_entry_candidate = (
      (new_stop_intent or self.explicit_target_early_entry_capture_counter > 0)
      and 0.78 < v_ego < 1.08
      and 0.95 < float(distance_to_stop_target_m) < 2.40
      and -0.24 < a_ego < 0.08
      and -0.60 < last_output_accel < -0.42
    )
    if early_entry_candidate:
      frames_100hz = int(interp(v_ego, [0.82, 0.90, 1.02], [32, 28, 24]))
      dt_scale = clip(dt / 0.01, 0.5, 20.0)
      self.explicit_target_early_entry_capture_counter = max(1, int(frames_100hz / dt_scale))
    elif self.explicit_target_early_entry_capture_counter > 0:
      self.explicit_target_early_entry_capture_counter -= 1

    return self.explicit_target_early_entry_capture_counter > 0

  def _update_explicit_target_soft_entry_carry(
    self,
    should_stop: bool,
    new_stop_entry: bool,
    v_ego: float,
    a_ego: float,
    last_output_accel: float,
    distance_to_stop_target_m: float | None,
    dt: float,
  ) -> bool:
    explicit_stop_target_available = distance_to_stop_target_m is not None and distance_to_stop_target_m >= 0.0
    if not should_stop or not explicit_stop_target_available:
      self.explicit_target_soft_entry_carry_counter = 0
      return False

    carry_seed = (
      (new_stop_entry or self.explicit_target_soft_entry_carry_counter > 0)
      and 0.78 < v_ego < 1.06
      and 1.35 < float(distance_to_stop_target_m) < 2.45
      and -0.20 < a_ego < 0.10
      and -0.36 < last_output_accel < -0.18
    )
    if carry_seed:
      frames_100hz = int(interp(v_ego, [0.78, 0.90, 1.06], [120, 108, 96]))
      dt_scale = clip(dt / 0.01, 0.5, 20.0)
      self.explicit_target_soft_entry_carry_counter = max(1, int(frames_100hz / dt_scale))
    elif self.explicit_target_soft_entry_carry_counter > 0:
      carry_still_valid = (
        0.65 < v_ego < 1.08
        and 0.90 < float(distance_to_stop_target_m) < 2.45
        and -0.30 < a_ego < 0.24
        and -0.48 < last_output_accel < -0.18
      )
      if carry_still_valid:
        self.explicit_target_soft_entry_carry_counter -= 1
      else:
        self.explicit_target_soft_entry_carry_counter = 0

    return self.explicit_target_soft_entry_carry_counter > 0

  def _update_low_speed_rollout(self, should_stop: bool, v_ego: float, dt: float) -> None:
    if not should_stop:
      self.low_speed_rollout_m = 0.0
      return

    if v_ego <= 0.02:
      self.low_speed_rollout_m = max(self.low_speed_rollout_m - (self.tuning.standstill_rollout_decay_mps * dt), 0.0)
    elif v_ego < 1.2:
      self.low_speed_rollout_m += v_ego * dt
    else:
      self.low_speed_rollout_m = max(self.low_speed_rollout_m - (v_ego * dt), 0.0)

  def _remaining_distance_est_m(self, v_ego: float, a_ego: float) -> float:
    decel_mag = max(0.20, -float(a_ego))
    return float(clip((float(v_ego) ** 2) / (2.0 * decel_mag), 0.0, 3.0))

  def _remaining_distance_m(self, distance_to_stop_target_m: float | None, v_ego: float, a_ego: float) -> float:
    if distance_to_stop_target_m is not None and distance_to_stop_target_m >= 0.0:
      return float(clip(distance_to_stop_target_m, 0.0, 6.0))
    return self._remaining_distance_est_m(v_ego, a_ego)

  def _append_command(self, last_output_accel: float) -> None:
    self._command_history.append(float(last_output_accel))
    max_len = int(self.tuning.command_history_len)
    if len(self._command_history) > max_len:
      self._command_history = self._command_history[-max_len:]

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

  def _phase_base_envelope(
    self,
    output_accel: float,
    v_ego: float,
    a_ego: float,
    disturbance: float,
    last_output_accel: float,
    min_expected_accel: float,
    dt: float,
  ) -> StoppingCommandEnvelope:
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
      hold_target = interp(v_ego, [0.06, 0.15, 0.30, 0.55, 0.85], [-0.14, -0.17, -0.20, -0.18, -0.15])
      target = min(target, hold_target)
      if disturbance > 0.0:
        target -= disturbance * interp(v_ego, [0.06, 0.55], [0.12, 0.07]) * dt
      brake_step = interp(v_ego, [0.06, 0.55, 0.85], [0.006, 0.008, 0.009])
      release_step = interp(v_ego, [0.06, 0.55, 0.85], [0.0008, 0.0022, 0.0030])
    else:
      hold_target = interp(v_ego, [0.00, 0.02, 0.06], [-0.16, -0.15, -0.13])
      target = min(target, hold_target)
      if disturbance > 0.0:
        target -= disturbance * 0.08 * dt
      brake_step = interp(v_ego, [0.00, 0.06], [0.006, 0.007])
      release_step = interp(v_ego, [0.00, 0.06], [0.0014, 0.0022])

    target, release_step = self._apply_over_brake_damping(
      target=target,
      release_step=release_step,
      v_ego=v_ego,
      a_ego=a_ego,
      min_expected_accel=min_expected_accel,
      dt=dt,
    )
    return StoppingCommandEnvelope(target=target, brake_step=brake_step, release_step=release_step)

  def _tail_profile_planner(
    self,
    target: float,
    brake_step: float,
    release_step: float,
    v_ego: float,
    a_ego: float,
    remaining_m: float,
    low_speed_rebound_risk: float,
    disturbance: float,
  ) -> StoppingCommandEnvelope:
    rollout_factor = clip((self.low_speed_rollout_m - 0.70) / 0.60, 0.0, 1.0)
    rebound_factor = clip((low_speed_rebound_risk - 0.05) / 0.35, 0.0, 1.0)
    disturbance_factor = clip((disturbance - 0.02) / 0.10, 0.0, 1.0)
    tail_risk = clip(max(rebound_factor, 0.70 * rollout_factor, disturbance_factor), 0.0, 1.0)

    soft_cap = interp(
      remaining_m,
      [0.00, 0.05, 0.12, 0.22, 0.40, 0.65],
      [-0.12, -0.15, -0.19, -0.25, -0.34, -0.46],
    )
    speed_soft_cap = interp(v_ego, [0.00, 0.08, 0.15, 0.30, 0.50, 0.70], [-0.13, -0.16, -0.20, -0.28, -0.38, -0.48])
    soft_cap = min(soft_cap, speed_soft_cap)

    strong_decel_relax = clip(
      (
        -a_ego
        - interp(v_ego, [0.00, 0.15, 0.40, 0.70], [0.28, 0.36, 0.46, 0.56])
      ) / 0.25,
      0.0,
      1.0,
    )
    relaxed_soft_cap = interp(v_ego, [0.00, 0.10, 0.25, 0.50, 0.70], [-0.12, -0.15, -0.20, -0.30, -0.40])
    soft_cap = ((1.0 - strong_decel_relax) * soft_cap) + (strong_decel_relax * relaxed_soft_cap)

    risk_floor = interp(v_ego, [0.00, 0.08, 0.15, 0.30, 0.50, 0.70], [-0.22, -0.25, -0.29, -0.36, -0.44, -0.54])
    risk_floor -= tail_risk * interp(v_ego, [0.00, 0.15, 0.30, 0.70], [0.06, 0.07, 0.08, 0.10])
    if self.low_speed_rollout_m > 1.00:
      risk_floor = min(risk_floor, interp(v_ego, [0.00, 0.08, 0.15, 0.30], [-0.30, -0.32, -0.35, -0.40]))
    target = max(target, soft_cap)
    target = min(target, risk_floor)

    nominal_brake_step = interp(v_ego, [0.00, 0.15, 0.40, 0.70], [0.0020, 0.0025, 0.0032, 0.0040])
    brake_step = max(brake_step, nominal_brake_step + (tail_risk * 0.004))

    nominal_release_step = interp(remaining_m, [0.00, 0.08, 0.20, 0.40], [0.0045, 0.0060, 0.0075, 0.0090])
    nominal_release_step *= interp(tail_risk, [0.00, 1.00], [1.00, 0.45])
    release_step = max(release_step, nominal_release_step)
    if tail_risk > 0.60:
      release_step = min(release_step, interp(v_ego, [0.00, 0.15, 0.70], [0.0020, 0.0025, 0.0040]))

    return StoppingCommandEnvelope(target=target, brake_step=brake_step, release_step=release_step)

  def _glide_handoff_planner(
    self,
    target: float,
    brake_step: float,
    release_step: float,
    v_ego: float,
    a_ego: float,
    remaining_m: float,
    disturbance: float,
  ) -> StoppingCommandEnvelope:
    remaining_factor = clip((0.65 - remaining_m) / 0.65, 0.0, 1.0)
    strong_decel_relax = clip((-a_ego - 0.32) / 0.28, 0.0, 1.0)
    disturbance_factor = clip((disturbance - 0.02) / 0.10, 0.0, 1.0)

    glide_cap = interp(v_ego, [0.06, 0.12, 0.20, 0.35, 0.55, 0.78], [-0.18, -0.20, -0.24, -0.31, -0.39, -0.48])
    glide_cap = min(glide_cap, interp(remaining_m, [0.00, 0.06, 0.18, 0.35, 0.65], [-0.18, -0.20, -0.25, -0.34, -0.46]))
    relaxed_glide_cap = interp(v_ego, [0.06, 0.12, 0.20, 0.35, 0.55, 0.78], [-0.16, -0.18, -0.22, -0.29, -0.36, -0.44])
    glide_cap = ((1.0 - strong_decel_relax) * glide_cap) + (strong_decel_relax * relaxed_glide_cap)

    glide_floor = interp(v_ego, [0.06, 0.12, 0.20, 0.35, 0.55, 0.78], [-0.24, -0.27, -0.32, -0.38, -0.46, -0.54])
    glide_floor -= remaining_factor * interp(v_ego, [0.06, 0.20, 0.35, 0.55, 0.78], [0.01, 0.02, 0.03, 0.04, 0.05])
    glide_floor -= disturbance_factor * interp(v_ego, [0.06, 0.20, 0.35, 0.55, 0.78], [0.01, 0.02, 0.03, 0.04, 0.04])

    target = max(target, glide_cap)
    target = min(target, glide_floor)

    glide_brake_step = interp(v_ego, [0.06, 0.12, 0.20, 0.35, 0.55, 0.78], [0.0024, 0.0030, 0.0038, 0.0048, 0.0060, 0.0075])
    glide_brake_step += remaining_factor * interp(v_ego, [0.06, 0.20, 0.35, 0.55, 0.78], [0.0006, 0.0008, 0.0010, 0.0013, 0.0016])
    brake_step = max(brake_step, glide_brake_step)

    glide_release_floor = interp(v_ego, [0.06, 0.12, 0.20, 0.35, 0.55, 0.78], [0.0024, 0.0026, 0.0029, 0.0033, 0.0038, 0.0044])
    glide_release_floor += strong_decel_relax * interp(v_ego, [0.06, 0.20, 0.35, 0.55, 0.78], [0.0006, 0.0008, 0.0010, 0.0012, 0.0014])
    glide_release_cap = interp(v_ego, [0.06, 0.12, 0.20, 0.35, 0.55, 0.78], [0.0036, 0.0038, 0.0042, 0.0048, 0.0055, 0.0062])
    release_step = clip(max(release_step, glide_release_floor), glide_release_floor, glide_release_cap)

    return StoppingCommandEnvelope(target=target, brake_step=brake_step, release_step=release_step)

  def _update_tail_commit(
    self,
    should_stop: bool,
    v_ego: float,
    a_ego: float,
    last_output_accel: float,
    disturbance: float,
    remaining_m: float,
    release_lock_active: bool,
    rebound_arrest_active: bool,
    low_speed_rebound_risk: float,
    clutch_push_relief: bool,
    dt: float,
  ) -> None:
    if clutch_push_relief:
      self.tail_commit_counter = 0
      return

    commit_entry = (
      should_stop
      and self.phase == StoppingPhase.NEAR_HOLD
      and 0.40 < v_ego < 0.82
      and 0.82 < self.low_speed_rollout_m < 1.70
      and remaining_m < 0.70
      and a_ego > -0.52
      and a_ego < -0.28
      and last_output_accel < -0.20
      and disturbance < 0.08
      and low_speed_rebound_risk < 0.12
      and not release_lock_active
      and not rebound_arrest_active
    )
    if commit_entry:
      frames_100hz = int(interp(v_ego, [0.40, 0.60, 0.82], [68, 58, 46]))
      dt_scale = clip(dt / 0.01, 0.5, 20.0)
      steps = max(1, int(frames_100hz / dt_scale))
      self.tail_commit_counter = max(self.tail_commit_counter, steps)
      return

    commit_keep = (
      self.tail_commit_counter > 0
      and v_ego < 0.82
      and self.low_speed_rollout_m > 0.42
      and last_output_accel < -0.16
      and (should_stop or a_ego < -0.02)
      and not release_lock_active
      and not rebound_arrest_active
    )
    if commit_keep:
      self.tail_commit_counter -= 1
    else:
      self.tail_commit_counter = 0

  def _write_debug_state(
    self,
    debug: dict[str, object] | None,
    release_lock_active: bool,
    rebound_arrest_active: bool,
    clutch_push_relief: bool,
  ) -> None:
    if debug is None:
      return
    debug["phase"] = int(self.phase)
    debug["release_lock_active"] = bool(release_lock_active)
    debug["rebound_arrest_active"] = bool(rebound_arrest_active)
    debug["clutch_push_relief"] = bool(clutch_push_relief)
    debug["tail_commit_active"] = bool(self.tail_commit_counter > 0)
    debug["rollout_m"] = float(self.low_speed_rollout_m)
    debug["recovery_i"] = float(self.low_speed_recovery_i)
    debug["standstill_settled_time_s"] = float(self.standstill_settled_time_s)

  def _record_trigger(self, debug_triggers: list[str] | None, trigger: str) -> None:
    if debug_triggers is not None:
      debug_triggers.append(trigger)

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
    remaining_m: float,
    explicit_stop_target_available: bool,
    clutch_push_relief: bool,
    dt: float,
  ) -> None:
    if not should_stop or clutch_push_relief or self.phase != StoppingPhase.HOLD:
      self.rebound_arrest_counter = 0
      return

    explicit_target_tail_holdoff = (
      explicit_stop_target_available
      and 0.0 < v_ego < 0.06
      and remaining_m > 0.16
      and (
        a_ego > -0.12
        or (
          remaining_m > 0.24
          and a_ego > -0.24
          and last_output_accel > -0.50
        )
      )
      and disturbance < 0.08
    )
    if explicit_target_tail_holdoff:
      self.rebound_arrest_counter = 0
      return

    arrest_trigger = (
      0.0 < v_ego < 0.045
      and last_output_accel < -0.22
      and a_ego > -0.24
      and low_speed_rebound_risk > 0.12
      and (
        self.low_speed_rollout_m > 0.10
        or v_ego > 0.055
        or last_output_accel < -0.40
        or (
          release_lock_active
          and disturbance > 0.04
          and last_output_accel < -0.34
        )
      )
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
    distance_to_stop_target_m: float | None = None,
    raw_should_stop: bool | None = None,
    lead_status: bool = False,
    lead_v: float = 0.0,
    lead_d_rel: float | None = None,
    debug: dict[str, object] | None = None,
  ) -> StoppingResult:
    if raw_should_stop is None:
      raw_should_stop = should_stop
    lead_distance_m = float(lead_d_rel) if lead_d_rel is not None else None
    far_stopped_lead_release = (
      not should_stop
      and lead_status
      and lead_distance_m is not None
      and lead_distance_m > 5.00
      and 0.0 <= v_ego < 0.55
      and lead_v <= interp(v_ego, [0.00, 0.20, 0.55], [0.65, 0.45, 0.28])
    )
    tail_commit_stop_latch = (
      not should_stop
      and not far_stopped_lead_release
      and self.tail_commit_counter > 0
      and 0.0 <= v_ego < 0.70
      and 0.42 < self.low_speed_rollout_m < 1.65
      and last_output_accel < -0.20
      and a_ego < -0.02
    )
    stop_intent_active = should_stop or tail_commit_stop_latch
    stop_entry_soften_active, new_stop_entry = self._update_stop_entry_soften(
      raw_should_stop=raw_should_stop,
      tail_commit_stop_latch=tail_commit_stop_latch,
      v_ego=v_ego,
      a_ego=a_ego,
      last_output_accel=last_output_accel,
      dt=dt,
    )
    stop_reacquire_hold_active = self._update_stop_reacquire_hold(
      should_stop=stop_intent_active,
      new_stop_entry=new_stop_entry,
      v_ego=v_ego,
      a_ego=a_ego,
      last_output_accel=last_output_accel,
      dt=dt,
    )
    explicit_target_early_entry_capture_active = self._update_explicit_target_early_entry_capture(
      should_stop=stop_intent_active,
      raw_should_stop=raw_should_stop,
      v_ego=v_ego,
      a_ego=a_ego,
      last_output_accel=last_output_accel,
      distance_to_stop_target_m=distance_to_stop_target_m,
      dt=dt,
    )
    explicit_target_soft_entry_carry_active = self._update_explicit_target_soft_entry_carry(
      should_stop=stop_intent_active,
      new_stop_entry=new_stop_entry,
      v_ego=v_ego,
      a_ego=a_ego,
      last_output_accel=last_output_accel,
      distance_to_stop_target_m=distance_to_stop_target_m,
      dt=dt,
    )
    late_no_target_stop_entry_capture_active = self._update_late_no_target_stop_entry_capture(
      should_stop=stop_intent_active,
      new_stop_entry=new_stop_entry,
      v_ego=v_ego,
      a_ego=a_ego,
      last_output_accel=last_output_accel,
      distance_to_stop_target_m=distance_to_stop_target_m,
      dt=dt,
    )
    if not stop_intent_active:
      micro_dropout_hold_preserve = (
        0.0 < v_ego < 0.06
        and last_output_accel < -0.22
        and a_ego > -0.08
      )
      active_release_dropout_hold_preserve = (
        0.06 <= v_ego < 0.22
        and last_output_accel < -0.22
        and output_accel > (last_output_accel + 0.01)
        and self.low_speed_rollout_m < 0.70
      )
      late_dropout_hold_preserve = (
        0.06 <= v_ego < 0.20
        and last_output_accel < -0.32
        and a_ego > -0.08
      )
      low_speed_stop_dropout_hold = (
        0.0 < v_ego < 0.24
        and (
          (a_ego < -0.05 and output_accel > -0.16)
          or micro_dropout_hold_preserve
          or active_release_dropout_hold_preserve
          or late_dropout_hold_preserve
        )
      )
      if low_speed_stop_dropout_hold:
        # If stop intent drops near standstill while decel is still active, avoid an abrupt command release.
        # This keeps a mild braking envelope through brief shouldStop dropouts and reduces re-accel/re-stop jolts.
        dt_scale = clip(dt / 0.01, 0.5, 20.0)
        hold_floor = interp(v_ego, [0.00, 0.08, 0.24], [-0.24, -0.22, -0.16])
        if micro_dropout_hold_preserve:
          hold_floor = min(hold_floor, interp(v_ego, [0.00, 0.03, 0.06], [-0.30, -0.27, -0.24]))
        elif active_release_dropout_hold_preserve:
          hold_floor = min(hold_floor, interp(v_ego, [0.06, 0.12, 0.22], [-0.32, -0.29, -0.22]))
        elif late_dropout_hold_preserve:
          hold_floor = min(hold_floor, interp(v_ego, [0.06, 0.12, 0.20], [-0.34, -0.31, -0.24]))
        late_dropout_brake_onset = (
          0.03 < v_ego < 0.16
          and a_ego < -0.08
          and last_output_accel > -0.12
        )
        if late_dropout_brake_onset:
          brake_step = interp(v_ego, [0.03, 0.08, 0.16], [0.055, 0.040, 0.024]) * dt_scale
          release_step = interp(v_ego, [0.03, 0.08, 0.16], [0.0009, 0.0014, 0.0022]) * dt_scale
        elif micro_dropout_hold_preserve:
          brake_step = interp(v_ego, [0.00, 0.03, 0.06], [0.012, 0.010, 0.008]) * dt_scale
          release_step = interp(v_ego, [0.00, 0.03, 0.06], [0.0006, 0.0008, 0.0011]) * dt_scale
        elif active_release_dropout_hold_preserve:
          brake_step = interp(v_ego, [0.06, 0.12, 0.22], [0.012, 0.009, 0.007]) * dt_scale
          release_step = interp(v_ego, [0.06, 0.12, 0.22], [0.0004, 0.0006, 0.0010]) * dt_scale
        elif late_dropout_hold_preserve:
          brake_step = interp(v_ego, [0.06, 0.12, 0.20], [0.010, 0.008, 0.006]) * dt_scale
          release_step = interp(v_ego, [0.06, 0.12, 0.20], [0.0004, 0.0006, 0.0010]) * dt_scale
        else:
          brake_step = interp(v_ego, [0.00, 0.08, 0.24], [0.022, 0.018, 0.012]) * dt_scale
          release_step = interp(v_ego, [0.00, 0.08, 0.24], [0.0015, 0.0022, 0.0032]) * dt_scale
        target = min(output_accel, hold_floor)
        guarded_output = clip(target, last_output_accel - brake_step, last_output_accel + release_step)
        guarded_output = clip(guarded_output, stop_accel, -0.05)
        if debug is not None:
          debug["dropout_hold_active"] = True
        return StoppingResult(output_accel=guarded_output, release_lock_active=False)
      self.reset()
      return StoppingResult(output_accel=output_accel, release_lock_active=False)

    debug_triggers: list[str] | None = [] if debug is not None else None

    # Stage: update internal state and derived metrics.
    self._append_command(last_output_accel)
    self.delay_frames = clip(int(round(0.05 / max(dt, 1e-3))), 1, 25)
    self.phase = self._phase_for_speed(v_ego)
    self._update_release_lock(v_ego, a_ego, last_output_accel, max_expected_accel, dt)
    self._update_low_speed_rollout(stop_intent_active, v_ego, dt)
    if self.phase == StoppingPhase.HOLD and v_ego <= self.tuning.standstill_settle_speed_mps and a_ego > self.tuning.standstill_settle_accel_threshold_mps2:
      self.standstill_settled_time_s = min(self.standstill_settled_time_s + dt, 5.0)
    else:
      self.standstill_settled_time_s = 0.0
    release_lock_active = self.release_lock_counter > 0
    disturbance = clip(a_ego - max_expected_accel, 0.0, 1.0)
    delay_release_guard = self._delay_release_guard(v_ego, last_output_accel)
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
      should_stop=stop_intent_active,
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
    remaining_m = self._remaining_distance_m(distance_to_stop_target_m, v_ego, a_ego)
    explicit_stop_target_available = distance_to_stop_target_m is not None and distance_to_stop_target_m >= 0.0
    self._update_rebound_arrest(
      should_stop=stop_intent_active,
      v_ego=v_ego,
      a_ego=a_ego,
      last_output_accel=last_output_accel,
      disturbance=disturbance,
      release_lock_active=release_lock_active,
      low_speed_rebound_risk=low_speed_rebound_risk,
      remaining_m=remaining_m,
      explicit_stop_target_available=explicit_stop_target_available,
      clutch_push_relief=clutch_push_relief,
      dt=dt,
    )
    rebound_arrest_active = self.rebound_arrest_counter > 0
    if debug is not None:
      debug["distance_to_stop_target_m"] = None if distance_to_stop_target_m is None else float(distance_to_stop_target_m)
      debug["remaining_m"] = float(remaining_m)
    self._update_tail_commit(
      should_stop=stop_intent_active,
      v_ego=v_ego,
      a_ego=a_ego,
      last_output_accel=last_output_accel,
      disturbance=disturbance,
      remaining_m=remaining_m,
      release_lock_active=release_lock_active,
      rebound_arrest_active=rebound_arrest_active,
      low_speed_rebound_risk=low_speed_rebound_risk,
      clutch_push_relief=clutch_push_relief,
      dt=dt,
    )
    glide_handoff_active = (
      self.tail_commit_counter > 0
      and self.phase in (StoppingPhase.NEAR_HOLD, StoppingPhase.HOLD)
      and 0.28 < v_ego < 0.82
      and a_ego > -0.52
      and a_ego < -0.10
      and disturbance < 0.10
      and low_speed_rebound_risk < 0.18
      and not clutch_push_relief
      and not rebound_arrest_active
    )

    self._write_debug_state(
      debug=debug,
      release_lock_active=release_lock_active,
      rebound_arrest_active=rebound_arrest_active,
      clutch_push_relief=clutch_push_relief,
    )

    rollout_trigger = interp(v_ego, [0.02, 0.25, 0.55, 1.20], [0.10, 0.20, 0.35, 0.70])
    rollout_full = interp(v_ego, [0.02, 0.25, 0.55, 1.20], [0.40, 0.65, 1.00, 2.20])
    rollout_tighten = clip(
      (self.low_speed_rollout_m - rollout_trigger) / max(rollout_full - rollout_trigger, 1e-3),
      0.0,
      1.0,
    )

    # Stage: base target and nominal step sizes.
    base_envelope = self._phase_base_envelope(
      output_accel=output_accel,
      v_ego=v_ego,
      a_ego=a_ego,
      disturbance=disturbance,
      last_output_accel=last_output_accel,
      min_expected_accel=min_expected_accel,
      dt=dt,
    )
    target = base_envelope.target
    brake_step = base_envelope.brake_step
    release_step = base_envelope.release_step
    distance_carry_soft_cap: float | None = None

    explicit_target_gentle_entry_hold_active = (
      explicit_stop_target_available
      and 0.82 < v_ego < 1.08
      and 1.00 < remaining_m < 2.30
      and self.low_speed_rollout_m < 1.30
      and -0.35 < a_ego < -0.05
      and -0.62 < last_output_accel < -0.42
      and low_speed_rebound_risk < 0.30
      and disturbance < 0.20
      and not stop_reacquire_hold_active
      and not rebound_arrest_active
      and not clutch_push_relief
    )
    if explicit_target_gentle_entry_hold_active:
      # If braking is already gentle and coherent with the explicit stop target, keep that profile
      # instead of dropping into the deeper tail-catch lane a meter or more before the target.
      self._record_trigger(debug_triggers, "explicit_target_gentle_entry_hold")
      gentle_floor = interp(remaining_m, [1.00, 1.20, 1.50, 1.90, 2.30], [-0.58, -0.55, -0.51, -0.47, -0.45])
      gentle_floor = min(gentle_floor, interp(v_ego, [0.82, 0.94, 1.08], [-0.54, -0.57, -0.60]))
      target = min(target, gentle_floor)
      brake_step = min(max(brake_step, interp(v_ego, [0.82, 0.94, 1.08], [0.0012, 0.0015, 0.0018])), 0.0020)
      release_step = min(release_step, interp(v_ego, [0.82, 0.94, 1.08], [0.0011, 0.0014, 0.0018]))

    explicit_target_tail_catch_active = (
      explicit_stop_target_available
      and 0.70 < v_ego < 1.05
      and 0.55 < remaining_m < 1.30
      and self.low_speed_rollout_m < 1.30
      and -0.35 < a_ego < 0.12
      and last_output_accel < -0.45
      and low_speed_rebound_risk < 0.18
      and disturbance < 0.12
      and not explicit_target_gentle_entry_hold_active
      and not stop_reacquire_hold_active
      and not rebound_arrest_active
      and not clutch_push_relief
    )
    if explicit_target_tail_catch_active:
      # In explicit-target stops, catch a weak late entry with one coherent deeper brake profile
      # instead of letting rollout_push and the later soften stack fight each other.
      self._record_trigger(debug_triggers, "explicit_target_tail_catch")
      catch_floor = interp(remaining_m, [0.55, 0.80, 1.05, 1.30], [-0.72, -0.80, -0.88, -0.94])
      catch_floor = min(catch_floor, interp(v_ego, [0.55, 0.75, 0.90, 1.05], [-0.74, -0.82, -0.90, -0.96]))
      target = min(target, catch_floor)
      brake_step = max(brake_step, interp(v_ego, [0.55, 0.75, 0.90, 1.05], [0.016, 0.018, 0.020, 0.022]))
      release_step = min(release_step, interp(v_ego, [0.55, 0.75, 0.90, 1.05], [0.0012, 0.0015, 0.0018, 0.0022]))

    far_lead_for_teacher_profile = lead_distance_m is None or lead_distance_m > 3.20
    explicit_target_mid_tail_teacher_profile = (
      explicit_stop_target_available
      and self.phase == StoppingPhase.NEAR_HOLD
      and far_lead_for_teacher_profile
      and 0.20 < v_ego < 0.42
      and 0.78 < remaining_m < 1.70
      and 0.20 < self.low_speed_rollout_m < 0.85
      and -0.55 < a_ego < -0.20
      and -0.58 < last_output_accel < -0.32
      and low_speed_rebound_risk < 0.25
      and disturbance < 0.10
      and not stop_reacquire_hold_active
      and not release_lock_active
      and not rebound_arrest_active
      and not clutch_push_relief
    )
    if explicit_target_mid_tail_teacher_profile:
      # On the fresh hard routes, horizon_v1 repeatedly improves mediocre explicit-target lead
      # tails by releasing one control beat while the target is still ~1m ahead, then letting the
      # normal tail/hold logic rebuild brake if decel fades. Keep this scoped to far-lead,
      # already-decelerating stops so close-lead authority is unchanged.
      self._record_trigger(debug_triggers, "explicit_target_mid_tail_teacher_profile")
      speed_cap = interp(v_ego, [0.20, 0.28, 0.42], [-0.25, -0.29, -0.38])
      remaining_cap = interp(remaining_m, [0.78, 1.20, 1.70], [-0.38, -0.32, -0.26])
      lead_cap = interp(lead_distance_m if lead_distance_m is not None else 5.0, [3.20, 4.20, 5.00], [-0.39, -0.34, -0.30])
      teacher_cap = min(speed_cap, remaining_cap, lead_cap)
      target = max(target, teacher_cap)
      brake_step = min(brake_step, interp(v_ego, [0.20, 0.28, 0.42], [0.0012, 0.0015, 0.0022]))
      release_step = max(release_step, interp(remaining_m, [0.78, 1.20, 1.70], [0.0140, 0.0160, 0.0180]))
      distance_carry_soft_cap = teacher_cap

    explicit_target_broad_tail_teacher_soften = (
      explicit_stop_target_available
      and self.phase == StoppingPhase.NEAR_HOLD
      and lead_distance_m is not None
      and 3.35 < lead_distance_m < 5.65
      and 0.28 < v_ego < 0.72
      and 0.85 < remaining_m < 2.15
      and 0.10 < self.low_speed_rollout_m < 1.00
      and -0.76 < a_ego < -0.30
      and -0.64 < last_output_accel < -0.34
      and low_speed_rebound_risk < 0.28
      and disturbance < 0.14
      and not stop_reacquire_hold_active
      and not rebound_arrest_active
      and not clutch_push_relief
    )
    if explicit_target_broad_tail_teacher_soften:
      self._record_trigger(debug_triggers, "explicit_target_broad_tail_teacher_soften")
      speed_cap = interp(v_ego, [0.28, 0.40, 0.55, 0.72], [-0.30, -0.34, -0.40, -0.48])
      remaining_cap = interp(remaining_m, [0.85, 1.10, 1.50, 2.15], [-0.40, -0.36, -0.32, -0.30])
      lead_cap = interp(lead_distance_m, [3.35, 4.20, 5.00, 5.65], [-0.42, -0.36, -0.32, -0.30])
      teacher_cap = min(speed_cap, remaining_cap, lead_cap)
      target = max(target, teacher_cap)
      brake_step = min(brake_step, interp(v_ego, [0.28, 0.40, 0.55, 0.72], [0.0014, 0.0018, 0.0024, 0.0032]))
      release_step = max(release_step, interp(remaining_m, [0.85, 1.10, 1.50, 2.15], [0.0150, 0.0170, 0.0180, 0.0160]))
      distance_carry_soft_cap = teacher_cap if distance_carry_soft_cap is None else max(distance_carry_soft_cap, teacher_cap)

    explicit_target_mid_gap_teacher_release = (
      explicit_stop_target_available
      and self.phase == StoppingPhase.NEAR_HOLD
      and lead_distance_m is not None
      and 3.60 < lead_distance_m <= 5.00
      and lead_v <= 0.25
      and 0.28 < v_ego < 0.56
      and 1.05 < remaining_m < 2.20
      and self.low_speed_rollout_m < 0.78
      and -0.76 < a_ego < -0.42
      and -0.46 < last_output_accel < -0.28
      and low_speed_rebound_risk < 0.22
      and disturbance < 0.10
      and not stop_reacquire_hold_active
      and not release_lock_active
      and not rebound_arrest_active
      and not clutch_push_relief
    )
    if explicit_target_mid_gap_teacher_release:
      # With the lead already stopped at a healthy gap and the explicit target still well ahead,
      # the teacher repeatedly wins by releasing one beat earlier than the broad tail soften.
      self._record_trigger(debug_triggers, "explicit_target_mid_gap_teacher_release")
      speed_cap = interp(v_ego, [0.28, 0.38, 0.48, 0.56], [-0.25, -0.28, -0.32, -0.36])
      remaining_cap = interp(remaining_m, [1.05, 1.35, 1.75, 2.20], [-0.34, -0.31, -0.28, -0.26])
      lead_cap = interp(lead_distance_m, [3.60, 4.20, 5.00], [-0.34, -0.30, -0.27])
      decel_cap = interp(a_ego, [-0.76, -0.58, -0.42], [-0.31, -0.28, -0.25])
      teacher_cap = min(speed_cap, remaining_cap, lead_cap, decel_cap)
      target = max(target, teacher_cap)
      brake_step = min(brake_step, interp(v_ego, [0.28, 0.40, 0.56], [0.0010, 0.0014, 0.0020]))
      release_step = max(release_step, interp(remaining_m, [1.05, 1.35, 1.75, 2.20], [0.0180, 0.0200, 0.0200, 0.0170]))
      distance_carry_soft_cap = teacher_cap if distance_carry_soft_cap is None else max(distance_carry_soft_cap, teacher_cap)

    explicit_target_tail_settle_active = (
      explicit_stop_target_available
      and self.phase in (StoppingPhase.NEAR_HOLD, StoppingPhase.HOLD)
      and 0.03 < v_ego < 0.90
      and 0.15 < remaining_m < 1.05
      and self.low_speed_rollout_m < interp(v_ego, [0.03, 0.08, 0.16, 0.35, 0.60, 0.90], [1.70, 1.60, 1.42, 1.20, 0.98, 0.82])
      and -0.85 < a_ego < 0.08
      and low_speed_rebound_risk < 0.45
      and disturbance < 0.12
      and not stop_reacquire_hold_active
      and not rebound_arrest_active
      and not clutch_push_relief
    )
    if explicit_target_tail_settle_active:
      # When the explicit target is still ahead, prefer one monotonic tail envelope over the
      # generic rollout_push / comfort_release / end-stop-cap stack.
      self._record_trigger(debug_triggers, "explicit_target_tail_settle")
      settle_cap = interp(remaining_m, [0.15, 0.22, 0.30, 0.45, 0.70, 1.05], [-0.28, -0.31, -0.34, -0.40, -0.50, -0.62])
      settle_cap = min(settle_cap, interp(v_ego, [0.03, 0.08, 0.16, 0.30, 0.50, 0.70, 0.90], [-0.22, -0.24, -0.28, -0.36, -0.48, -0.60, -0.74]))
      settle_relax = interp(a_ego, [-0.75, -0.45, -0.20, 0.05], [0.0, 0.04, 0.10, 0.12])
      settle_cap = min(-0.18, settle_cap + settle_relax)
      target = max(target, settle_cap)
      brake_step = min(brake_step, interp(v_ego, [0.03, 0.08, 0.16, 0.30, 0.50, 0.70, 0.90], [0.0010, 0.0012, 0.0016, 0.0020, 0.0026, 0.0032, 0.0038]))
      release_step = max(release_step, interp(remaining_m, [0.15, 0.22, 0.30, 0.45, 0.70, 1.05], [0.013, 0.012, 0.010, 0.008, 0.006, 0.004]))
      distance_carry_soft_cap = settle_cap

    explicit_target_far_tail_teacher_profile = (
      explicit_target_tail_settle_active
      and self.phase == StoppingPhase.NEAR_HOLD
      and far_lead_for_teacher_profile
      and 0.14 < v_ego < 0.34
      and 0.50 < remaining_m < 1.05
      and self.low_speed_rollout_m < 1.35
      and -0.52 < a_ego < -0.12
      and last_output_accel < -0.38
      and low_speed_rebound_risk < 0.35
      and disturbance < 0.10
      and not stop_reacquire_hold_active
      and not release_lock_active
      and not rebound_arrest_active
      and not clutch_push_relief
    )
    if explicit_target_far_tail_teacher_profile:
      # In far-lead explicit-target tails, the teacher often releases one beat earlier while
      # the target is still 0.5-0.9m ahead, avoiding a final command spike without reducing
      # close-lead authority.
      self._record_trigger(debug_triggers, "explicit_target_far_tail_teacher_profile")
      far_tail_cap = interp(v_ego, [0.14, 0.18, 0.24, 0.34], [-0.34, -0.34, -0.37, -0.42])
      target = max(target, far_tail_cap)
      brake_step = min(brake_step, interp(v_ego, [0.14, 0.24, 0.34], [0.0014, 0.0020, 0.0028]))
      release_step = max(release_step, interp(v_ego, [0.14, 0.24, 0.34], [0.014, 0.010, 0.007]))
      distance_carry_soft_cap = far_tail_cap if distance_carry_soft_cap is None else max(distance_carry_soft_cap, far_tail_cap)

    explicit_target_rollout_relief_active = (
      explicit_stop_target_available
      and self.phase in (StoppingPhase.NEAR_HOLD, StoppingPhase.HOLD)
      and 0.18 < v_ego < 0.90
      and 0.45 < remaining_m < 0.85
      and self.low_speed_rollout_m > interp(v_ego, [0.18, 0.35, 0.55, 0.75, 0.90], [2.05, 1.95, 1.80, 1.55, 1.35])
      and -0.90 < a_ego < -0.20
      and last_output_accel < -0.92
      and low_speed_rebound_risk < 0.16
      and disturbance < 0.12
      and not stop_reacquire_hold_active
      and not rebound_arrest_active
      and not clutch_push_relief
    )
    if explicit_target_rollout_relief_active:
      # When the explicit target is still materially ahead, very large rollout is not a reason to
      # keep piling on brake. Prefer a shallower carry envelope over the generic high-rollout stack.
      self._record_trigger(debug_triggers, "explicit_target_rollout_relief")
      relief_cap = interp(remaining_m, [0.45, 0.70, 0.95, 1.25], [-0.62, -0.70, -0.80, -0.88])
      relief_cap = min(relief_cap, interp(v_ego, [0.18, 0.35, 0.55, 0.75, 0.90], [-0.58, -0.66, -0.74, -0.82, -0.88]))
      target = max(target, relief_cap)
      brake_step = min(brake_step, interp(v_ego, [0.18, 0.35, 0.55, 0.75, 0.90], [0.0018, 0.0022, 0.0028, 0.0034, 0.0040]))
      release_step = max(release_step, interp(remaining_m, [0.45, 0.70, 0.95, 1.25], [0.0080, 0.0060, 0.0045, 0.0035]))
      distance_carry_soft_cap = relief_cap if distance_carry_soft_cap is None else max(distance_carry_soft_cap, relief_cap)

    # Stage: rollout management + rebound/leapfrog guards.
    if stop_entry_soften_active and not clutch_push_relief:
      # Immediately after stop intent rises, prefer a shallow brake ramp before rollout/tail logic takes over.
      self._record_trigger(debug_triggers, "stop_entry_soften")
      entry_cap = interp(v_ego, [0.12, 0.20, 0.35, 0.60, 1.00, 1.65], [-0.14, -0.18, -0.24, -0.30, -0.38, -0.46])
      target = max(target, entry_cap)
      brake_step = min(brake_step, interp(v_ego, [0.12, 0.20, 0.35, 0.60, 1.00, 1.65], [0.0032, 0.0036, 0.0040, 0.0045, 0.0050, 0.0056]))
      release_step = min(release_step, interp(v_ego, [0.12, 0.20, 0.35, 0.60, 1.00, 1.65], [0.0010, 0.0012, 0.0015, 0.0020, 0.0026, 0.0032]))

    if late_no_target_stop_entry_capture_active and not clutch_push_relief:
      # When shouldStop arrives late without an explicit stop target, do not let the first stopping beat
      # unwind into a mild crawl before rebuilding brake a few frames later.
      self._record_trigger(debug_triggers, "late_no_target_stop_entry_capture")
      capture_floor = interp(v_ego, [0.55, 0.70, 0.85, 0.95], [-0.38, -0.41, -0.44, -0.46])
      target = min(target, capture_floor)
      brake_step = max(brake_step, interp(v_ego, [0.55, 0.70, 0.85, 0.95], [0.010, 0.012, 0.014, 0.016]))
      release_step = min(release_step, interp(v_ego, [0.55, 0.70, 0.85, 0.95], [0.0012, 0.0016, 0.0020, 0.0024]))

    if explicit_target_soft_entry_carry_active and not clutch_push_relief:
      # If stop mode enters on a mild inherited brake while the explicit target is still well ahead,
      # carry that gentler decel through the handoff instead of escalating into a later catch-up push.
      self._record_trigger(debug_triggers, "explicit_target_soft_entry_carry")
      speed_floor = interp(v_ego, [0.72, 0.86, 0.96, 1.08], [-0.26, -0.29, -0.32, -0.35])
      distance_floor = interp(remaining_m, [0.95, 1.20, 1.60, 2.00, 2.45], [-0.38, -0.36, -0.33, -0.30, -0.28])
      carry_floor = min(speed_floor, distance_floor)
      target = min(target, carry_floor)
      brake_step = min(max(brake_step, interp(v_ego, [0.72, 0.86, 0.96, 1.08], [0.0006, 0.0008, 0.0010, 0.0012])), 0.0014)
      release_step = min(release_step, interp(v_ego, [0.72, 0.86, 0.96, 1.08], [0.0010, 0.0012, 0.0015, 0.0018]))

    explicit_target_pre_should_stop_soft_entry_active = (
      explicit_stop_target_available
      and self.phase in (StoppingPhase.APPROACH, StoppingPhase.NEAR_HOLD)
      and not raw_should_stop
      and 0.78 < v_ego < 1.05
      and 1.35 < remaining_m < 2.35
      and self.low_speed_rollout_m < 1.10
      and -0.16 < a_ego < 0.14
      and -0.46 < last_output_accel < -0.16
      and output_accel < last_output_accel - 0.04
      and low_speed_rebound_risk < 0.30
      and disturbance < 0.35
      and not explicit_target_soft_entry_carry_active
      and not explicit_target_gentle_entry_hold_active
      and not stop_reacquire_hold_active
      and not rebound_arrest_active
      and not clutch_push_relief
    )
    if explicit_target_pre_should_stop_soft_entry_active:
      # LongControl can enter stopping before raw shouldStop while a real target is still ~2m ahead.
      # Cap that base-command step so the later raw shouldStop handoff can stay on the soft carry lane.
      self._record_trigger(debug_triggers, "explicit_target_pre_should_stop_soft_entry")
      pre_stop_cap = interp(remaining_m, [1.35, 1.70, 2.35], [-0.37, -0.34, -0.31])
      pre_stop_cap = min(pre_stop_cap, interp(v_ego, [0.78, 0.90, 1.05], [-0.31, -0.33, -0.36]))
      target = max(target, pre_stop_cap)
      brake_step = min(brake_step, interp(v_ego, [0.78, 0.90, 1.05], [0.0050, 0.0060, 0.0070]))
      release_step = min(release_step, interp(v_ego, [0.78, 0.90, 1.05], [0.0010, 0.0013, 0.0017]))

    explicit_target_weak_entry_shape_active = (
      explicit_stop_target_available
      and self.phase == StoppingPhase.NEAR_HOLD
      and 0.42 < v_ego < 0.72
      and 1.05 < remaining_m < 1.75
      and self.low_speed_rollout_m < 1.70
      and -0.22 < a_ego < 0.24
      and -0.58 < last_output_accel < -0.05
      and low_speed_rebound_risk < 0.45
      and disturbance < 0.60
      and not explicit_target_soft_entry_carry_active
      and not explicit_target_gentle_entry_hold_active
      and not stop_reacquire_hold_active
      and not rebound_arrest_active
      and not clutch_push_relief
    )
    if explicit_target_weak_entry_shape_active:
      # Use the remaining explicit-target distance before rollout accumulates enough to force a late catch-up jab.
      self._record_trigger(debug_triggers, "explicit_target_weak_entry_shape")
      shape_floor = interp(remaining_m, [1.05, 1.30, 1.55, 1.75], [-0.44, -0.39, -0.34, -0.30])
      speed_floor = interp(v_ego, [0.42, 0.55, 0.72], [-0.36, -0.40, -0.44])
      target = min(target, min(shape_floor, speed_floor))
      brake_step = max(brake_step, interp(v_ego, [0.42, 0.55, 0.72], [0.009, 0.011, 0.013]))
      release_step = min(release_step, interp(v_ego, [0.42, 0.55, 0.72], [0.0012, 0.0016, 0.0020]))

    if explicit_target_early_entry_capture_active and not explicit_target_gentle_entry_hold_active and not clutch_push_relief:
      # In explicit-target stops that enter stopping before raw shouldStop, avoid the shallow unwind
      # that later forces a sharper brake rebuild once the generic stop signal finally appears.
      self._record_trigger(debug_triggers, "explicit_target_early_entry_capture")
      distance_capture_floor = interp(remaining_m, [0.95, 1.30, 1.80, 2.40], [-0.64, -0.61, -0.58, -0.54])
      speed_capture_floor = interp(v_ego, [0.78, 0.90, 1.08], [-0.56, -0.60, -0.64])
      capture_floor = min(distance_capture_floor, speed_capture_floor)
      target = min(target, capture_floor)
      brake_step = max(brake_step, interp(v_ego, [0.78, 0.90, 1.08], [0.018, 0.020, 0.022]))
      release_step = min(release_step, interp(v_ego, [0.78, 0.90, 1.08], [0.0012, 0.0015, 0.0020]))

    if stop_reacquire_hold_active and not clutch_push_relief:
      # If stop intent comes back after brake has already built, avoid immediately unwinding into the
      # soft-landing/end-stop cap stack and then catching the stop again a few frames later.
      self._record_trigger(debug_triggers, "stop_reacquire_hold")
      reacquire_floor = interp(v_ego, [0.05, 0.10, 0.20, 0.40, 0.70, 0.95], [-0.58, -0.60, -0.63, -0.69, -0.75, -0.80])
      high_speed_reacquire_soften = (
        0.45 < v_ego < 0.95
        and a_ego < -0.55
        and -0.74 < last_output_accel < -0.50
      )
      if high_speed_reacquire_soften:
        # Late shouldStop reacquire at moderate speed still needs a hold floor, but the older
        # deeper profile creates the felt "jab" on today's 9cb-style routes.
        self._record_trigger(debug_triggers, "high_speed_reacquire_soften")
        reacquire_floor = max(reacquire_floor, interp(v_ego, [0.45, 0.70, 0.95], [-0.69, -0.72, -0.75]))
      target = min(target, reacquire_floor)
      brake_step = max(brake_step, interp(v_ego, [0.05, 0.10, 0.20, 0.40, 0.70, 0.95], [0.004, 0.005, 0.006, 0.008, 0.010, 0.012]))
      release_step = min(release_step, interp(v_ego, [0.05, 0.10, 0.20, 0.40, 0.70, 0.95], [0.0010, 0.0012, 0.0016, 0.0024, 0.0034, 0.0044]))

    micro_stopgo_soft_capture = (
      self.phase in (StoppingPhase.NEAR_HOLD, StoppingPhase.HOLD)
      and v_ego < 0.12
      and self.low_speed_rollout_m < 0.12
      and last_output_accel > -0.38
      and not clutch_push_relief
      and (a_ego > -0.06 or disturbance > 0.08 or release_lock_active)
    )
    if micro_stopgo_soft_capture:
      # In tiny-rollout stop-and-go re-entry, avoid the abrupt brake-onset that makes the stop feel grabby.
      self._record_trigger(debug_triggers, "micro_stopgo_soft_capture")
      capture_floor = interp(v_ego, [0.00, 0.04, 0.10, 0.18], [-0.22, -0.24, -0.28, -0.34])
      capture_step = interp(v_ego, [0.00, 0.04, 0.10, 0.18], [0.0050, 0.0055, 0.0065, 0.0080])
      capture_step += 0.004 * clip((disturbance - 0.08) / 0.20, 0.0, 1.0)
      target = min(target, capture_floor)
      brake_step = min(max(brake_step, capture_step), 0.010)
      release_step = min(release_step, interp(v_ego, [0.00, 0.04, 0.10, 0.18], [0.0008, 0.0010, 0.0012, 0.0016]))

    low_speed_disturbance_capture = (
      self.phase == StoppingPhase.HOLD
      and 0.0 < v_ego < 0.06
      and disturbance > 0.04
      and last_output_accel < -0.26
      and not clutch_push_relief
    )
    if low_speed_disturbance_capture:
      # If the car pushes forward right at hold, bias slightly deeper than the nominal settle path.
      self._record_trigger(debug_triggers, "low_speed_disturbance_capture")
      disturbance_floor = interp(v_ego, [0.00, 0.03, 0.06], [-0.31, -0.33, -0.35])
      target = min(target, disturbance_floor)
      brake_step = max(brake_step, interp(v_ego, [0.00, 0.03, 0.06], [0.028, 0.024, 0.018]))
      release_step = min(release_step, interp(v_ego, [0.00, 0.03, 0.06], [0.0008, 0.0010, 0.0013]))

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
      self._record_trigger(debug_triggers, "rollout_rebound_guard")
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
      and not explicit_target_soft_entry_carry_active
      and not clutch_push_relief
    )
    if severe_rebound_guard:
      # When rollout is already large and decel has collapsed, apply a temporary deeper floor.
      # This specifically targets leapfrog-like stop retries without changing nominal near-hold behavior.
      self._record_trigger(debug_triggers, "severe_rebound_guard")
      severe_floor = interp(v_ego, [0.00, 0.20, 0.55, 0.95], [-0.58, -0.64, -0.72, -0.84])
      target = min(target, severe_floor)
      brake_step = max(brake_step, interp(v_ego, [0.00, 0.20, 0.55, 0.95], [0.014, 0.020, 0.028, 0.036]))
      release_step = min(release_step, interp(v_ego, [0.00, 0.20, 0.55, 0.95], [0.0009, 0.0013, 0.0019, 0.0028]))

    low_risk_high_rollout_unwind = (
      self.phase in (StoppingPhase.NEAR_HOLD, StoppingPhase.HOLD)
      and 0.12 < v_ego < 0.22
      and 1.00 < self.low_speed_rollout_m < 1.50
      and -0.30 < a_ego < -0.15
      and -0.40 < last_output_accel < -0.32
      and low_speed_rebound_risk < 0.03
      and disturbance < 0.05
      and not release_lock_active
      and not clutch_push_relief
    )

    if self.low_speed_recovery_i > 0.0 and not explicit_target_tail_settle_active and not explicit_target_rollout_relief_active and not clutch_push_relief and not low_risk_high_rollout_unwind and not glide_handoff_active:
      self._record_trigger(debug_triggers, "low_speed_recovery")
      recovery_gain = interp(v_ego, [0.06, 0.20, 0.50, 0.85, 1.20], [0.22, 0.20, 0.17, 0.13, 0.10])
      target -= self.low_speed_recovery_i * recovery_gain
      brake_step = max(brake_step, interp(v_ego, [0.06, 0.20, 0.50, 0.85, 1.20], [0.012, 0.016, 0.022, 0.029, 0.036]))
      release_step = min(release_step, interp(v_ego, [0.06, 0.20, 0.50, 0.85, 1.20], [0.0009, 0.0012, 0.0018, 0.0026, 0.0036]))

    if clutch_push_relief:
      # Under heavy braking, some automatic gearboxes can still push the car forward.
      # Avoid ratcheting to very deep brake commands in this phase, which tends to increase end-stop jerk.
      self._record_trigger(debug_triggers, "clutch_push_relief")
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
      and not explicit_target_tail_settle_active
      and not explicit_target_rollout_relief_active
      and not clutch_push_relief
    )
    if comfort_release:
      # If the car is already decelerating strongly near hold, avoid adding more brake.
      # This limits end-stop jerk spikes while preserving short rollout.
      self._record_trigger(debug_triggers, "comfort_release")
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
      and not stop_reacquire_hold_active
      and not clutch_push_relief
    )
    if medium_decel_relief:
      # For medium-deep commands near hold, stop ratcheting down once decel is already strong.
      self._record_trigger(debug_triggers, "medium_decel_relief")
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
      self._record_trigger(debug_triggers, "deep_command_jerk_relief")
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

    if rollout_tighten > 0.0 and not explicit_target_soft_entry_carry_active and not explicit_target_pre_should_stop_soft_entry_active and not explicit_target_weak_entry_shape_active and not explicit_target_gentle_entry_hold_active and not explicit_target_tail_settle_active and not explicit_target_rollout_relief_active and not clutch_push_relief:
      self._record_trigger(debug_triggers, "rollout_tighten")
      release_cap = interp(v_ego, [0.02, 0.25, 0.55, 1.20], [0.0010, 0.0018, 0.0030, 0.0050])
      if glide_handoff_active:
        glide_release_cap = interp(v_ego, [0.06, 0.20, 0.45, 0.82], [0.0024, 0.0028, 0.0034, 0.0044])
        release_step = min(release_step, max(release_cap, glide_release_cap))
      else:
        release_step = min(release_step, release_cap)
      # Only tighten target when decel is weak; otherwise allow a softer landing even if rollout is building.
      if a_ego > -0.35 and not glide_handoff_active:
        target -= rollout_tighten * interp(v_ego, [0.02, 0.25, 0.55, 1.20], [0.05, 0.08, 0.11, 0.12])
        rollout_floor = interp(v_ego, [0.02, 0.12, 0.25, 0.55, 1.20], [-0.30, -0.27, -0.24, -0.19, -0.13])
        target = min(target, rollout_floor + ((1.0 - rollout_tighten) * 0.05))

    terminal_unwind_delay = (
      not explicit_stop_target_available
      and self.phase in (StoppingPhase.NEAR_HOLD, StoppingPhase.HOLD)
      and 0.12 < v_ego < 0.75
      and self.low_speed_rollout_m < interp(v_ego, [0.12, 0.40, 0.75], [0.42, 0.62, 0.82])
      and a_ego > interp(v_ego, [0.12, 0.20, 0.40, 0.75], [-0.48, -0.50, -0.62, -0.70])
      and a_ego < interp(v_ego, [0.12, 0.40, 0.75], [-0.30, -0.38, -0.46])
      and last_output_accel < interp(v_ego, [0.12, 0.40, 0.75], [-0.30, -0.44, -0.56])
      and disturbance < 0.08
      and not stop_reacquire_hold_active
      and not release_lock_active
      and not rebound_arrest_active
      and not clutch_push_relief
    )
    if terminal_unwind_delay:
      # Without an explicit planner stop target, avoid letting the terminal soften/cap stack own
      # a still-moving stop while brake is already built and rollout remains moderate.
      self._record_trigger(debug_triggers, "terminal_unwind_delay")
      target = max(target, last_output_accel)
      release_step = min(release_step, interp(v_ego, [0.12, 0.40, 0.75], [0.0016, 0.0026, 0.0034]))
    terminal_unwind_relief = (
      terminal_unwind_delay
      and 0.12 < v_ego < 0.78
      and 0.18 < self.low_speed_rollout_m < 0.65
      and -0.72 < a_ego < -0.42
      and -0.82 < last_output_accel < -0.48
      and disturbance < 0.06
      and low_speed_rebound_risk < 0.10
      and not glide_handoff_active
    )
    if terminal_unwind_relief:
      # The plain terminal_unwind_delay lane can stay too rigid after a late stop-mode reacquire.
      # In this moderate-rollout, already-decelerating corner, allow a small controlled unwind instead
      # of holding the inherited brake flat through the whole shouldStop span.
      self._record_trigger(debug_triggers, "terminal_unwind_relief")
      relief_delta = interp(v_ego, [0.28, 0.45, 0.60, 0.78], [0.018, 0.026, 0.034, 0.038])
      target = max(target, last_output_accel + relief_delta)
      release_step = max(release_step, interp(v_ego, [0.28, 0.45, 0.60, 0.78], [0.0020, 0.0024, 0.0028, 0.0032]))
      terminal_unwind_teacher_release = (
        v_ego < 0.30
        and remaining_m < 0.12
        and a_ego > -0.50
        and low_speed_rebound_risk < 0.06
      )
      if terminal_unwind_teacher_release:
        # The horizon teacher softens this late no-target glide before re-holding near standstill.
        # Let the unwind happen while the car is still moving instead of dumping it into the final cap.
        self._record_trigger(debug_triggers, "terminal_unwind_teacher_release")
        teacher_cap = interp(v_ego, [0.12, 0.20, 0.30], [-0.600, -0.610, -0.630])
        target = max(target, teacher_cap)
        release_step = max(release_step, interp(v_ego, [0.12, 0.20, 0.30], [0.0095, 0.0110, 0.0120]))
    no_target_low_rollout_terminal_teacher_release = (
      terminal_unwind_delay
      and not explicit_stop_target_available
      and not lead_status
      and 0.10 < v_ego < 0.24
      and self.low_speed_rollout_m < 0.18
      and -0.48 < a_ego < -0.24
      and last_output_accel < -0.48
      and low_speed_rebound_risk < 0.20
      and disturbance < 0.08
      and not stop_reacquire_hold_active
      and not release_lock_active
      and not rebound_arrest_active
      and not clutch_push_relief
    )
    if no_target_low_rollout_terminal_teacher_release:
      self._record_trigger(debug_triggers, "no_target_low_rollout_terminal_teacher_release")
      teacher_cap = interp(v_ego, [0.10, 0.16, 0.24], [-0.34, -0.38, -0.44])
      target = max(target, teacher_cap)
      brake_step = min(brake_step, interp(v_ego, [0.10, 0.16, 0.24], [0.0012, 0.0016, 0.0022]))
      release_step = max(release_step, interp(v_ego, [0.10, 0.16, 0.24], [0.018, 0.016, 0.012]))
    distance_carry_settle = (
      not glide_handoff_active
      and not terminal_unwind_delay
      and not explicit_target_tail_settle_active
      and self.phase in (StoppingPhase.NEAR_HOLD, StoppingPhase.HOLD)
      and 0.12 < v_ego < 0.65
      and 0.08 < remaining_m < 0.32
      and (
        explicit_stop_target_available
        or (v_ego < 0.42 and remaining_m < 0.18 and self.low_speed_rollout_m < 0.42)
      )
      and self.low_speed_rollout_m < interp(v_ego, [0.12, 0.30, 0.65], [0.78, 0.68, 0.58])
      and -0.45 < a_ego < 0.04
      and not stop_reacquire_hold_active
      and low_speed_rebound_risk < 0.08
      and disturbance < 0.06
      and not release_lock_active
      and not rebound_arrest_active
      and not clutch_push_relief
    )
    if distance_carry_settle:
      # When the planned stop target is still ahead and rollout is low, stay on a shallow carry profile
      # instead of committing to the terminal cap stack too early.
      self._record_trigger(debug_triggers, "distance_carry_settle")
      carry_cap = interp(remaining_m, [0.08, 0.16, 0.24, 0.32], [-0.17, -0.20, -0.26, -0.35])
      carry_cap = min(carry_cap, interp(v_ego, [0.12, 0.20, 0.35, 0.65], [-0.18, -0.22, -0.28, -0.38]))
      target = max(target, carry_cap)
      brake_step = min(brake_step, interp(v_ego, [0.12, 0.20, 0.35, 0.65], [0.0012, 0.0016, 0.0022, 0.0030]))
      release_step = max(release_step, interp(remaining_m, [0.08, 0.16, 0.24, 0.32], [0.0120, 0.0100, 0.0080, 0.0060]))
      distance_carry_soft_cap = carry_cap

    if glide_handoff_active:
      self._record_trigger(debug_triggers, "glide_handoff")
      glide_handoff = self._glide_handoff_planner(
        target=target,
        brake_step=brake_step,
        release_step=release_step,
        v_ego=v_ego,
        a_ego=a_ego,
        remaining_m=remaining_m,
        disturbance=disturbance,
      )
      target = glide_handoff.target
      brake_step = glide_handoff.brake_step
      release_step = glide_handoff.release_step

    rebound_settle_ramp = (
      not glide_handoff_active
      and
      self.phase == StoppingPhase.NEAR_HOLD
      and release_lock_active
      and 0.60 < v_ego < 0.90
      and 0.45 < self.low_speed_rollout_m < 0.95
      and a_ego > 0.04
      and disturbance > 0.10
      and last_output_accel > -0.32
      and not clutch_push_relief
    )
    if rebound_settle_ramp:
      # In stop-and-go, current rollout_push can turn on too abruptly after a short rebound.
      # Add a gentler intermediate brake ramp before the stronger high-rollout push takes over.
      self._record_trigger(debug_triggers, "rebound_settle_ramp")
      settle_floor = interp(v_ego, [0.60, 0.72, 0.82, 0.90], [-0.33, -0.35, -0.39, -0.43])
      target = min(target, settle_floor)
      brake_step = max(brake_step, interp(v_ego, [0.60, 0.72, 0.82, 0.90], [0.010, 0.012, 0.014, 0.016]))
      release_step = min(release_step, interp(v_ego, [0.60, 0.72, 0.82, 0.90], [0.0016, 0.0020, 0.0025, 0.0030]))

    micro_stopgo_soft_start = (
      self.phase in (StoppingPhase.NEAR_HOLD, StoppingPhase.HOLD)
      and 0.0 < v_ego < 0.25
      and self.low_speed_rollout_m < 0.14
      and a_ego > -0.10
      and last_output_accel > -0.45
      and not release_lock_active
      and not rebound_arrest_active
      and not clutch_push_relief
    )
    if micro_stopgo_soft_start:
      # For low-speed stop-and-go reacquire, avoid dropping into a fast brake staircase immediately.
      self._record_trigger(debug_triggers, "micro_stopgo_soft_start")
      onset_floor = interp(v_ego, [0.00, 0.05, 0.12, 0.25], [-0.20, -0.23, -0.28, -0.36])
      target = min(target, onset_floor)
      brake_step = min(brake_step, interp(v_ego, [0.00, 0.05, 0.12, 0.25], [0.006, 0.008, 0.010, 0.013]))
      release_step = min(release_step, interp(v_ego, [0.00, 0.05, 0.12, 0.25], [0.0008, 0.0010, 0.0014, 0.0022]))

    explicit_target_gentle_rollout_block = (
      explicit_stop_target_available
      and 0.82 < v_ego < 1.02
      and 1.15 < remaining_m < 1.90
      and -0.22 < a_ego < 0.02
      and -0.62 < last_output_accel < -0.44
      and disturbance < 0.28
      and not stop_reacquire_hold_active
      and not rebound_arrest_active
      and not clutch_push_relief
    )
    if explicit_target_gentle_rollout_block:
      # If the explicit target is still materially ahead and the carried brake is already coherent,
      # do not let rollout_push deepen the stop just because low-speed rollout is building.
      self._record_trigger(debug_triggers, "explicit_target_gentle_rollout_block")
      rollout_cap = interp(remaining_m, [1.15, 1.45, 1.90], [-0.58, -0.56, -0.53])
      rollout_cap = min(rollout_cap, interp(v_ego, [0.82, 0.92, 1.02], [-0.56, -0.58, -0.60]))
      target = max(target, rollout_cap)
      brake_step = min(brake_step, interp(v_ego, [0.82, 0.92, 1.02], [0.0012, 0.0015, 0.0018]))
      release_step = min(release_step, interp(v_ego, [0.82, 0.92, 1.02], [0.0011, 0.0014, 0.0018]))

    rollout_push = (
      not glide_handoff_active
      and
      rollout_tighten > 0.05
      and not explicit_target_soft_entry_carry_active
      and not explicit_target_pre_should_stop_soft_entry_active
      and not explicit_target_weak_entry_shape_active
      and not explicit_target_gentle_entry_hold_active
      and not explicit_target_gentle_rollout_block
      and not explicit_target_tail_settle_active
      and not explicit_target_rollout_relief_active
      and v_ego < 1.2
      and a_ego > -0.30
      and not distance_carry_settle
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
      self._record_trigger(debug_triggers, "rollout_push")
      push_floor = interp(v_ego, [0.06, 0.20, 0.50, 0.85, 1.20], [-0.50, -0.46, -0.40, -0.34, -0.28])
      target = min(target, push_floor)
      brake_step = max(brake_step, rollout_tighten * interp(v_ego, [0.06, 0.20, 0.50, 0.85, 1.20], [0.024, 0.020, 0.016, 0.013, 0.010]))
      release_step = min(release_step, interp(v_ego, [0.06, 0.20, 0.50, 0.85, 1.20], [0.0010, 0.0014, 0.0020, 0.0028, 0.0038]))
    if low_risk_high_rollout_unwind:
      self._record_trigger(debug_triggers, "low_risk_high_rollout_unwind")
      unwind_cap = interp(v_ego, [0.12, 0.20, 0.35], [-0.30, -0.32, -0.36])
      target = max(target, unwind_cap)
      brake_step = min(brake_step, interp(v_ego, [0.12, 0.20, 0.35], [0.0012, 0.0016, 0.0021]))
      release_step = max(release_step, interp(v_ego, [0.12, 0.20, 0.35], [0.010, 0.008, 0.006]))

    rollout_near_limit_guard = (
      self.phase in (StoppingPhase.NEAR_HOLD, StoppingPhase.HOLD)
      and not explicit_target_tail_settle_active
      and not explicit_target_rollout_relief_active
      and self.low_speed_rollout_m > 1.60
      and v_ego < 0.80
      and a_ego > -0.45
      and not clutch_push_relief
    )
    if rollout_near_limit_guard:
      # Once rollout is already near the 2m limit, apply a mild floor to avoid drifting over.
      self._record_trigger(debug_triggers, "rollout_near_limit_guard")
      near_limit_floor = interp(v_ego, [0.12, 0.30, 0.60, 0.80], [-0.40, -0.36, -0.30, -0.24])
      target = min(target, near_limit_floor)
      brake_step = max(brake_step, interp(v_ego, [0.12, 0.30, 0.60, 0.80], [0.012, 0.009, 0.006, 0.004]))
      release_step = min(release_step, interp(v_ego, [0.12, 0.30, 0.60, 0.80], [0.0012, 0.0018, 0.0030, 0.0040]))

    rollout_relief_guard = (
      self.phase in (StoppingPhase.NEAR_HOLD, StoppingPhase.HOLD)
      and not explicit_target_tail_settle_active
      and not explicit_target_rollout_relief_active
      and self.low_speed_rollout_m > interp(v_ego, [0.06, 0.25, 0.60, 1.00], [0.75, 0.90, 1.20, 1.60])
      and v_ego < 1.0
      and a_ego > -0.25
      and not clutch_push_relief
    )
    if rollout_relief_guard:
      # Keep command away from the low-magnitude relief region while rollout is already elevated.
      self._record_trigger(debug_triggers, "rollout_relief_guard")
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
      self._record_trigger(debug_triggers, "high_rollout_low_speed_unwind")
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

    # Stage: lock semantics and soft landing.
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

    # Stage: terminal stop shaping.
    tail_profile_planner_active = (
      not glide_handoff_active
      and
      not explicit_target_tail_settle_active
      and
      not explicit_target_rollout_relief_active
      and
      self.phase in (StoppingPhase.NEAR_HOLD, StoppingPhase.HOLD)
      and v_ego < 0.70
      and self.low_speed_rollout_m > 0.75
      and remaining_m < 0.50
      and not clutch_push_relief
    )
    if tail_profile_planner_active:
      self._record_trigger(debug_triggers, "tail_profile_planner")
      tail_profile = self._tail_profile_planner(
        target=target,
        brake_step=brake_step,
        release_step=release_step,
        v_ego=v_ego,
        a_ego=a_ego,
        remaining_m=remaining_m,
        low_speed_rebound_risk=low_speed_rebound_risk,
        disturbance=disturbance,
      )
      target = tail_profile.target
      brake_step = tail_profile.brake_step
      release_step = tail_profile.release_step

    teacher_rollout_soft_cap: float | None = None
    teacher_rollout_profile_lead_allowed = (not lead_status) or (lead_distance_m is not None and lead_distance_m > 3.20)
    teacher_rollout_profile_active = (
      not explicit_stop_target_available
      and teacher_rollout_profile_lead_allowed
      and self.phase in (StoppingPhase.NEAR_HOLD, StoppingPhase.HOLD)
      and 0.035 < v_ego < 0.48
      and 0.38 < self.low_speed_rollout_m < 1.55
      and remaining_m < 0.42
      and -0.72 < a_ego < -0.04
      and (
        low_speed_rebound_risk < 0.35
        or (
          not lead_status
          and v_ego < 0.10
          and self.low_speed_rollout_m < 0.75
          and disturbance < 0.08
        )
      )
      and disturbance < 0.08
      and not stop_reacquire_hold_active
      and not release_lock_active
      and not rebound_arrest_active
      and not clutch_push_relief
      and (
        tail_profile_planner_active
        or terminal_unwind_delay
        or rollout_tighten > 0.30
      )
    )
    if teacher_rollout_profile_active:
      # The horizon teacher repeatedly reshapes no-target terminal rollout by releasing earlier
      # while the car is still moving, then holding a mild floor if decel fades near wheel-stop.
      self._record_trigger(debug_triggers, "teacher_rollout_profile")
      teacher_release_cap = interp(v_ego, [0.035, 0.07, 0.10, 0.16, 0.24, 0.36, 0.48], [-0.255, -0.270, -0.315, -0.39, -0.46, -0.54, -0.61])
      teacher_rollout_soft_cap = teacher_release_cap
      target = max(target, teacher_release_cap)
      brake_step = min(brake_step, interp(v_ego, [0.035, 0.12, 0.24, 0.48], [0.0024, 0.0028, 0.0035, 0.0045]))
      release_step = max(release_step, interp(v_ego, [0.035, 0.12, 0.24, 0.48], [0.0050, 0.0065, 0.0075, 0.0085]))
      weak_terminal_decel = v_ego < 0.16 and a_ego > -0.22 and self.low_speed_rollout_m > 0.72
      if weak_terminal_decel:
        terminal_floor = interp(v_ego, [0.035, 0.07, 0.11, 0.16], [-0.45, -0.41, -0.38, -0.36])
        target = min(target, terminal_floor)
        brake_step = max(brake_step, interp(v_ego, [0.035, 0.07, 0.11, 0.16], [0.0048, 0.0054, 0.0060, 0.0068]))
        release_step = min(release_step, interp(v_ego, [0.035, 0.07, 0.11, 0.16], [0.0020, 0.0024, 0.0028, 0.0032]))

    soft_landing_release = (
      not tail_profile_planner_active
      and not terminal_unwind_delay
      and not explicit_target_tail_settle_active
      and not explicit_target_rollout_relief_active
      and
      self.phase in (StoppingPhase.NEAR_HOLD, StoppingPhase.HOLD)
      and v_ego < 1.05
      and a_ego < -0.50
      and last_output_accel < -0.55
      and not stop_reacquire_hold_active
      and not (v_ego < 0.55 and self.low_speed_rollout_m < 0.90 and remaining_m < 0.24)
      and (not release_lock_active or lock_soft_relax)
      and not clutch_push_relief
    )
    if soft_landing_release:
      # If decel is already strong at very low speed, unwind toward a softer landing.
      # This limits the acceleration step at wheel-stop without disabling the disturbance/rollout guards.
      self._record_trigger(debug_triggers, "soft_landing_release")
      soft_target = interp(v_ego, [0.06, 0.20, 0.40, 0.85, 1.05], [-0.12, -0.18, -0.26, -0.38, -0.44])
      target = max(target, soft_target)
      brake_step = min(brake_step, interp(v_ego, [0.06, 1.05], [0.0020, 0.0030]))
      release_step = max(release_step, interp(v_ego, [0.06, 1.05], [0.010, 0.015]))

    creep_rebound_guard = (
      not tail_profile_planner_active
      and
      should_stop
      and self.phase in (StoppingPhase.NEAR_HOLD, StoppingPhase.HOLD)
      and not micro_stopgo_soft_capture
      and 0.0 < v_ego < 0.25
      and (release_lock_active or a_ego > 0.04 or disturbance > 0.14)
      and not clutch_push_relief
    )
    if creep_rebound_guard:
      # If we rebound/creep while we still expect to stop, allow a slightly deeper low-speed brake cap.
      # This helps counter automatic clutch/drivetrain push without permanently increasing wheel-stop command magnitude.
      self._record_trigger(debug_triggers, "creep_rebound_guard")
      creep_cap = interp(v_ego, [0.02, 0.08, 0.25], [-0.32, -0.36, -0.48])
      target = min(target, creep_cap)
      brake_step = max(brake_step, interp(v_ego, [0.02, 0.08, 0.25], [0.010, 0.014, 0.020]))
      release_step = min(release_step, interp(v_ego, [0.02, 0.08, 0.25], [0.0012, 0.0016, 0.0024]))

    low_speed_rebound_cap: float | None = None
    high_rollout_rebound_cap_gate = (
      self.low_speed_rollout_m > 1.00
      and v_ego < 0.12
      and a_ego > -0.20
    )
    low_speed_rebound_cap_active = (
      low_speed_rebound_risk > 0.0
      and not clutch_push_relief
      and (v_ego < 0.10 or a_ego > -0.08 or disturbance > 0.10 or release_lock_active or high_rollout_rebound_cap_gate)
    )
    if low_speed_rebound_cap_active:
      self._record_trigger(debug_triggers, "low_speed_rebound_cap_active")
      risk_floor = interp(v_ego, [0.00, 0.03, 0.08, 0.25], [-0.44, -0.40, -0.34, -0.28])
      low_speed_rebound_cap = (-0.275 * (1.0 - low_speed_rebound_risk)) + (risk_floor * low_speed_rebound_risk)
      target = min(target, low_speed_rebound_cap)
      brake_step = max(brake_step, interp(low_speed_rebound_risk, [0.0, 1.0], [0.008, 0.016]))
      release_step = min(release_step, interp(low_speed_rebound_risk, [0.0, 1.0], [0.0018, 0.0010]))
      if v_ego < 0.08 and self.low_speed_rollout_m < 0.08:
        micro_rollout_floor = interp(v_ego, [0.00, 0.04, 0.08], [-0.26, -0.30, -0.34])
        target = min(target, micro_rollout_floor)
        brake_step = max(brake_step, interp(v_ego, [0.00, 0.04, 0.08], [0.024, 0.020, 0.016]))

    teacher_rollout_soft_landing = (
      teacher_rollout_soft_cap is not None
      and not lead_status
      and 0.035 < v_ego < 0.10
      and 0.38 < self.low_speed_rollout_m < 0.72
      and -0.25 < a_ego < -0.05
      and disturbance < 0.08
      and not release_lock_active
      and not rebound_arrest_active
      and not clutch_push_relief
    )
    if teacher_rollout_soft_landing:
      self._record_trigger(debug_triggers, "teacher_rollout_soft_landing")
      target = max(target, teacher_rollout_soft_cap)
      brake_step = min(brake_step, interp(v_ego, [0.035, 0.07, 0.10], [0.0014, 0.0018, 0.0022]))
      release_step = max(release_step, interp(v_ego, [0.035, 0.07, 0.10], [0.0075, 0.0065, 0.0055]))

    tail_profile_terminal_soften = (
      tail_profile_planner_active
      and 0.0 < v_ego < 0.09
      and 0.95 < self.low_speed_rollout_m < 1.10
      and remaining_m < 0.08
      and -0.14 < a_ego < -0.02
      and -0.36 < last_output_accel < -0.26
      and disturbance < 0.04
      and not release_lock_active
      and not rebound_arrest_active
      and not clutch_push_relief
    )
    if tail_profile_terminal_soften:
      # In the final low-rollout frames before wheel-stop, the terminal cap stack can hold more brake
      # than the replay model needs. Blend toward a shallower cap here instead of forcing another push.
      self._record_trigger(debug_triggers, "tail_profile_terminal_soften")
      soften_cap = interp(v_ego, [0.00, 0.03, 0.06, 0.09], [-0.235, -0.250, -0.272, -0.300])
      target = max(target, soften_cap)
      brake_step = min(brake_step, interp(v_ego, [0.00, 0.03, 0.06, 0.09], [0.0010, 0.0012, 0.0016, 0.0020]))
      release_step = max(release_step, interp(v_ego, [0.00, 0.03, 0.06, 0.09], [0.010, 0.008, 0.006, 0.004]))

    high_rollout_hold_preserve = (
      not tail_profile_planner_active
      and
      self.phase in (StoppingPhase.NEAR_HOLD, StoppingPhase.HOLD)
      and self.low_speed_rollout_m > 1.25
      and v_ego < 0.22
      and a_ego > -0.30
      and -0.42 < last_output_accel < -0.30
      and not release_lock_active
      and not rebound_arrest_active
      and not clutch_push_relief
    )
    if high_rollout_hold_preserve:
      # Preserve a mild inherited hold brake for the last high-rollout low-speed frames instead of unwinding immediately.
      # This is scoped to the weak-decel / no-rebound-risk corner where legacy replay stays flatter.
      self._record_trigger(debug_triggers, "high_rollout_hold_preserve")
      target = min(target, last_output_accel)
      brake_step = min(brake_step, interp(v_ego, [0.08, 0.14, 0.22], [0.0010, 0.0012, 0.0016]))
      release_step = min(release_step, interp(v_ego, [0.08, 0.14, 0.22], [0.0006, 0.0008, 0.0010]))

    pre_hold_rebound_preserve = (
      tail_profile_planner_active
      and self.phase in (StoppingPhase.NEAR_HOLD, StoppingPhase.HOLD)
      and self.low_speed_rollout_m > 1.15
      and 0.04 < v_ego < 0.13
      and a_ego > -0.10
      and -0.45 < last_output_accel < -0.30
      and low_speed_rebound_risk > 0.20
      and not release_lock_active
      and not rebound_arrest_active
      and not clutch_push_relief
    )
    if pre_hold_rebound_preserve:
      # If the car nearly reaches hold and then decel fades, do not unwind the inherited
      # hold brake into the pre-hold rebound. Preserve it until the rebound guards take over.
      self._record_trigger(debug_triggers, "pre_hold_rebound_preserve")
      target = min(target, last_output_accel)
      brake_step = min(brake_step, interp(v_ego, [0.04, 0.08, 0.13], [0.0010, 0.0012, 0.0015]))
      release_step = min(release_step, interp(v_ego, [0.04, 0.08, 0.13], [0.0005, 0.0007, 0.0010]))

    moderate_rollout_hold_preserve = (
      not tail_profile_planner_active
      and
      self.phase in (StoppingPhase.NEAR_HOLD, StoppingPhase.HOLD)
      and 0.88 < self.low_speed_rollout_m < 1.02
      and v_ego < 0.12
      and -0.28 < a_ego < -0.08
      and -0.34 < last_output_accel < -0.24
      and low_speed_rebound_risk < 0.03
      and disturbance < 0.05
      and not release_lock_active
      and not rebound_arrest_active
      and not clutch_push_relief
    )
    if moderate_rollout_hold_preserve:
      self._record_trigger(debug_triggers, "moderate_rollout_hold_preserve")
      target = min(target, last_output_accel)
      brake_step = min(brake_step, interp(v_ego, [0.05, 0.08, 0.12], [0.0010, 0.0013, 0.0018]))
      release_step = min(release_step, interp(v_ego, [0.05, 0.08, 0.12], [0.0006, 0.0009, 0.0013]))

    rebound_arrest_cap: float | None = None
    if rebound_arrest_active and not clutch_push_relief:
      self._record_trigger(debug_triggers, "rebound_arrest_active")
      rebound_arrest_cap = interp(v_ego, [0.00, 0.03, 0.06, 0.08], [-1.40, -1.15, -0.85, -0.56])
      micro_stop_rebound_soften = (
        self.low_speed_rollout_m < 0.12
        and v_ego < 0.05
        and disturbance > 0.04
      )
      if micro_stop_rebound_soften:
        # For tiny low-rollout stop-go retries, full rebound-arrest depth produces a visible jab.
        # Keep the arrest behavior but use a shallower cap and slower onset in this micro-stop regime.
        self._record_trigger(debug_triggers, "micro_stop_rebound_soften")
        rebound_arrest_cap = interp(v_ego, [0.00, 0.02, 0.05], [-0.78, -0.72, -0.62])
      moderate_rollout_rebound_soften = (
        not micro_stop_rebound_soften
        and 0.35 < self.low_speed_rollout_m < 0.90
        and v_ego < 0.055
        and a_ego > -0.10
        and low_speed_rebound_risk < 0.70
        and last_output_accel > -0.56
      )
      if moderate_rollout_rebound_soften:
        # Moderate-rollout low-speed stops should not jump from a mild hold brake straight into a deep arrest floor.
        # Only soften the first arrest beat while inherited brake is still mild; once the command is already deep,
        # fall back to the normal arrest lane instead of stretching the softer cap into a later bigger jab.
        self._record_trigger(debug_triggers, "moderate_rollout_rebound_soften")
        rebound_arrest_cap = interp(v_ego, [0.00, 0.02, 0.05], [-0.72, -0.66, -0.58])
      target = min(target, rebound_arrest_cap)
      rebound_brake_step = interp(v_ego, [0.00, 0.08], [0.040, 0.022])
      if micro_stop_rebound_soften:
        rebound_brake_step = min(rebound_brake_step, interp(v_ego, [0.00, 0.02, 0.05], [0.014, 0.012, 0.010]))
      elif moderate_rollout_rebound_soften:
        rebound_brake_step = min(rebound_brake_step, interp(v_ego, [0.00, 0.02, 0.05], [0.022, 0.020, 0.016]))
      brake_step = max(brake_step, rebound_brake_step)
      release_step = min(release_step, interp(v_ego, [0.00, 0.08], [0.0008, 0.0014]))

    end_stop_brake_cap = interp(v_ego, [0.00, 0.10, 0.15, 0.25, 0.60], [-0.255, -0.255, -0.30, -0.42, -0.68])
    explicit_target_micro_hold_active = (
      explicit_stop_target_available
      and self.phase == StoppingPhase.HOLD
      and 0.0 < v_ego < 0.06
      and 0.28 < remaining_m < 0.55
      and self.low_speed_rollout_m < 0.90
      and -0.05 < a_ego < 0.30
      and (a_ego > -0.01 or disturbance > 0.02 or last_output_accel > -0.20)
      and not stop_reacquire_hold_active
      and not release_lock_active
      and not rebound_arrest_active
      and not clutch_push_relief
    )
    if explicit_target_micro_hold_active:
      # If the explicit target is still materially ahead after we've already reached standstill,
      # do not unwind brake into a tiny creep-and-catch cycle. Hold one mild floor instead.
      self._record_trigger(debug_triggers, "explicit_target_micro_hold")
      micro_hold_cap = interp(remaining_m, [0.28, 0.39, 0.55], [-0.34, -0.38, -0.42])
      micro_hold_cap = min(micro_hold_cap, interp(v_ego, [0.00, 0.02, 0.04, 0.06], [-0.38, -0.37, -0.35, -0.33]))
      end_stop_brake_cap = min(end_stop_brake_cap, micro_hold_cap)
      target = min(target, micro_hold_cap)
      brake_step = max(brake_step, interp(v_ego, [0.00, 0.02, 0.04, 0.06], [0.010, 0.009, 0.008, 0.007]))
      release_step = min(release_step, interp(v_ego, [0.00, 0.02, 0.04, 0.06], [0.0006, 0.0008, 0.0010, 0.0012]))
    low_speed_rebound_cap_relief = (
      not tail_profile_planner_active
      and
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
      self._record_trigger(debug_triggers, "low_speed_rebound_cap_relief")
      rebound_relief_cap = interp(low_speed_rebound_risk, [0.20, 1.00], [-0.436, -0.536])
      end_stop_brake_cap = min(end_stop_brake_cap, rebound_relief_cap)
      release_step = min(release_step, interp(v_ego, [0.00, 0.04, 0.12], [0.0007, 0.0010, 0.0014]))
    if distance_carry_soft_cap is not None:
      end_stop_brake_cap = max(end_stop_brake_cap, distance_carry_soft_cap)
    no_target_micro_soft_landing = (
      not tail_profile_planner_active
      and not explicit_stop_target_available
      and self.phase in (StoppingPhase.NEAR_HOLD, StoppingPhase.HOLD)
      and low_speed_rebound_cap_active
      and last_output_accel < -0.16
      and v_ego < 0.08
      and self.low_speed_rollout_m < 0.12
      and a_ego > -0.22
      and disturbance < 0.06
      and not stop_reacquire_hold_active
      and not release_lock_active
      and not rebound_arrest_active
      and not clutch_push_relief
    )
    if no_target_micro_soft_landing:
      # In tiny no-target end-stop windows, the nominal rebound/end-stop cap stack can keep too much
      # brake and create a visible final jerk. Allow a shallower inherited-brake unwind here.
      self._record_trigger(debug_triggers, "no_target_micro_soft_landing")
      soften_cap = last_output_accel + interp(v_ego, [0.00, 0.03, 0.08], [0.14, 0.13, 0.10])
      soften_cap = max(soften_cap, interp(v_ego, [0.00, 0.03, 0.08], [-0.70, -0.58, -0.40]))
      if low_speed_rebound_cap_active:
        soften_cap = min(soften_cap, interp(v_ego, [0.00, 0.04, 0.08], [-0.34, -0.335, -0.33]))
      end_stop_brake_cap = max(end_stop_brake_cap, soften_cap)
      target = max(target, soften_cap)
      brake_step = min(brake_step, interp(v_ego, [0.00, 0.03, 0.08], [0.0010, 0.0012, 0.0018]))
      release_step = max(release_step, interp(v_ego, [0.00, 0.03, 0.08], [0.012, 0.010, 0.007]))
    no_target_pre_stop_soft_landing = (
      not tail_profile_planner_active
      and not explicit_stop_target_available
      and self.phase == StoppingPhase.NEAR_HOLD
      and low_speed_rebound_cap_active
      and 0.06 < v_ego < 0.16
      and self.low_speed_rollout_m < 0.18
      and -0.36 < a_ego < -0.12
      and last_output_accel < -0.34
      and disturbance < 0.08
      and not stop_reacquire_hold_active
      and not release_lock_active
      and not rebound_arrest_active
      and not clutch_push_relief
    )
    if no_target_pre_stop_soft_landing:
      self._record_trigger(debug_triggers, "no_target_pre_stop_soft_landing")
      soften_cap = max(
        last_output_accel + interp(v_ego, [0.06, 0.11, 0.16], [0.16, 0.18, 0.16]),
        interp(v_ego, [0.06, 0.11, 0.16], [-0.30, -0.325, -0.36]),
      )
      end_stop_brake_cap = max(end_stop_brake_cap, soften_cap)
      target = max(target, soften_cap)
      brake_step = min(brake_step, interp(v_ego, [0.06, 0.11, 0.16], [0.0010, 0.0014, 0.0019]))
      release_step = max(release_step, interp(v_ego, [0.06, 0.11, 0.16], [0.018, 0.021, 0.018]))
    clean_settle_profile_active = (
      not tail_profile_planner_active
      and self.phase in (StoppingPhase.NEAR_HOLD, StoppingPhase.HOLD)
      and 0.03 < v_ego < 0.12
      and 0.35 < self.low_speed_rollout_m < 0.90
      and -0.20 < a_ego < -0.06
      and disturbance < 0.06
      and low_speed_rebound_risk < 0.18
      and not low_speed_rebound_cap_relief
      and not creep_rebound_guard
      and not release_lock_active
      and not clutch_push_relief
    )
    if clean_settle_profile_active:
      # In a clean moderate-rollout settle, prefer one monotonic low-speed brake profile over the layered rebound/relief stack.
      self._record_trigger(debug_triggers, "clean_settle_profile")
      settle_cap = interp(v_ego, [0.03, 0.05, 0.08, 0.12], [-0.29, -0.30, -0.32, -0.35])
      target = max(target, settle_cap)
      brake_step = min(brake_step, interp(v_ego, [0.03, 0.05, 0.08, 0.12], [0.0010, 0.0012, 0.0016, 0.0022]))
      release_step = max(release_step, interp(v_ego, [0.03, 0.05, 0.08, 0.12], [0.0060, 0.0048, 0.0036, 0.0026]))
    low_rollout_soft_landing_cap = (
      not tail_profile_planner_active
      and not terminal_unwind_delay
      and not explicit_target_tail_settle_active
      and not explicit_target_rollout_relief_active
      and
      self.phase in (StoppingPhase.NEAR_HOLD, StoppingPhase.HOLD)
      and v_ego < 0.22
      and (distance_to_stop_target_m is None or remaining_m < 0.70)
      and self.low_speed_rollout_m < 1.50
      and not stop_reacquire_hold_active
      and low_speed_rebound_risk < 0.18
      and not low_speed_rebound_cap_active
      and not low_speed_rebound_cap_relief
      and not creep_rebound_guard
      and not rebound_arrest_active
      and not clutch_push_relief
      and (not release_lock_active or disturbance < 0.08)
    )
    if low_rollout_soft_landing_cap:
      # In low-rollout/low-rebound-risk stops, unwind deep near-hold command a bit earlier.
      # This targets end-stop jerk without weakening the high-rollout rebound guards.
      self._record_trigger(debug_triggers, "low_rollout_soft_landing_cap")
      soft_landing_cap = interp(v_ego, [0.00, 0.08, 0.14, 0.22], [-0.210, -0.225, -0.265, -0.340])
      end_stop_brake_cap = max(end_stop_brake_cap, soft_landing_cap)
      release_step = max(release_step, interp(v_ego, [0.00, 0.08, 0.14, 0.22], [0.028, 0.024, 0.022, 0.016]))
    moderate_decel_soft_cap = (
      not tail_profile_planner_active
      and
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
      self._record_trigger(debug_triggers, "moderate_decel_soft_cap")
      end_stop_brake_cap = max(end_stop_brake_cap, -0.275)
    strong_decel_soft_cap = (
      not tail_profile_planner_active
      and
      v_ego < 0.20
      and a_ego < -0.70
      and self.low_speed_rollout_m < 0.80
      and not release_lock_active
      and not clutch_push_relief
    )
    if strong_decel_soft_cap:
      self._record_trigger(debug_triggers, "strong_decel_soft_cap")
      end_stop_brake_cap = max(end_stop_brake_cap, -0.275)
    if creep_rebound_guard:
      end_stop_brake_cap = min(end_stop_brake_cap, creep_cap)
    if low_speed_rebound_cap is not None:
      end_stop_brake_cap = min(end_stop_brake_cap, low_speed_rebound_cap)
    if teacher_rollout_soft_landing and teacher_rollout_soft_cap is not None:
      end_stop_brake_cap = max(end_stop_brake_cap, teacher_rollout_soft_cap)
    if rebound_arrest_cap is not None:
      end_stop_brake_cap = min(end_stop_brake_cap, rebound_arrest_cap)
    final_high_rollout_hold_floor = (
      not tail_profile_planner_active
      and
      self.phase in (StoppingPhase.NEAR_HOLD, StoppingPhase.HOLD)
      and 1.10 < self.low_speed_rollout_m < 1.14
      and v_ego < 0.11
      and -0.18 < a_ego < -0.06
      and -0.32 < last_output_accel < -0.28
      and low_speed_rebound_risk > 0.35
      and disturbance < 0.05
      and not release_lock_active
      and not rebound_arrest_active
      and not clutch_push_relief
    )
    if final_high_rollout_hold_floor:
      self._record_trigger(debug_triggers, "final_high_rollout_hold_floor")
      end_stop_brake_cap = min(end_stop_brake_cap, interp(v_ego, [0.08, 0.11], [-0.314, -0.308]))
      release_step = min(release_step, interp(v_ego, [0.08, 0.11], [0.0010, 0.0014]))
    explicit_target_pre_hold_teacher_release = (
      explicit_stop_target_available
      and self.phase in (StoppingPhase.NEAR_HOLD, StoppingPhase.HOLD)
      and lead_status
      and lead_distance_m is not None
      and 3.20 < lead_distance_m <= 5.00
      and 0.0 <= v_ego < 0.16
      and 0.50 < remaining_m < 0.85
      and self.low_speed_rollout_m < 0.60
      and -0.34 < a_ego < -0.05
      and last_output_accel < -0.28
      and low_speed_rebound_risk < 0.55
      and disturbance < 0.12
      and not stop_reacquire_hold_active
      and not release_lock_active
      and not rebound_arrest_active
      and not clutch_push_relief
    )
    if explicit_target_pre_hold_teacher_release:
      self._record_trigger(debug_triggers, "explicit_target_pre_hold_teacher_release")
      teacher_cap = interp(v_ego, [0.05, 0.08, 0.12, 0.16], [-0.30, -0.31, -0.34, -0.40])
      teacher_cap = max(teacher_cap, interp(remaining_m, [0.50, 0.68, 0.85], [-0.31, -0.33, -0.38]))
      teacher_cap = max(teacher_cap, interp(lead_distance_m, [3.20, 4.20, 5.00], [-0.36, -0.34, -0.32]))
      end_stop_brake_cap = max(end_stop_brake_cap, teacher_cap)
      target = max(target, teacher_cap)
      brake_step = min(brake_step, interp(v_ego, [0.05, 0.08, 0.12, 0.16], [0.0010, 0.0012, 0.0016, 0.0020]))
      release_step = max(release_step, interp(v_ego, [0.05, 0.08, 0.12, 0.16], [0.018, 0.016, 0.013, 0.010]))

    explicit_target_wide_gap_pre_hold_teacher_release = (
      explicit_stop_target_available
      and self.phase in (StoppingPhase.NEAR_HOLD, StoppingPhase.HOLD)
      and lead_status
      and lead_distance_m is not None
      and 3.35 < lead_distance_m <= 5.00
      and 0.0 <= v_ego < 0.24
      and 0.48 < remaining_m < 1.75
      and self.low_speed_rollout_m < 1.08
      and -0.48 < a_ego < -0.08
      and last_output_accel < -0.22
      and low_speed_rebound_risk < 0.62
      and disturbance < 0.12
      and not stop_reacquire_hold_active
      and not release_lock_active
      and not rebound_arrest_active
      and not clutch_push_relief
    )
    if explicit_target_wide_gap_pre_hold_teacher_release:
      # With a healthy stopped-lead gap and the explicit target still ahead, the route teacher
      # consistently releases the last pre-hold brake beat instead of letting the generic
      # rebound/end-stop cap add a visible wheel-stop jab.
      self._record_trigger(debug_triggers, "explicit_target_wide_gap_pre_hold_teacher_release")
      speed_cap = interp(v_ego, [0.00, 0.05, 0.10, 0.16, 0.24], [-0.230, -0.240, -0.252, -0.268, -0.288])
      remaining_cap = interp(remaining_m, [0.48, 0.72, 0.95, 1.25, 1.75], [-0.300, -0.292, -0.282, -0.272, -0.258])
      lead_cap = interp(lead_distance_m, [3.35, 4.20, 5.00], [-0.320, -0.292, -0.270])
      teacher_cap = max(speed_cap, remaining_cap, lead_cap)
      end_stop_brake_cap = max(end_stop_brake_cap, teacher_cap)
      target = max(target, teacher_cap)
      brake_step = min(brake_step, interp(v_ego, [0.00, 0.08, 0.24], [0.0010, 0.0012, 0.0020]))
      release_step = max(release_step, interp(v_ego, [0.00, 0.08, 0.24], [0.0190, 0.0170, 0.0130]))

    explicit_target_low_rollout_strong_decel_teacher_release = (
      explicit_stop_target_available
      and self.phase in (StoppingPhase.NEAR_HOLD, StoppingPhase.HOLD)
      and lead_status
      and lead_distance_m is not None
      and 3.35 < lead_distance_m <= 5.00
      and 0.05 < v_ego < 0.23
      and 0.48 < remaining_m < 0.72
      and self.low_speed_rollout_m < 0.32
      and -0.76 < a_ego < -0.34
      and last_output_accel < -0.38
      and low_speed_rebound_risk < 0.55
      and disturbance < 0.12
      and not stop_reacquire_hold_active
      and not release_lock_active
      and not rebound_arrest_active
      and not clutch_push_relief
    )
    if explicit_target_low_rollout_strong_decel_teacher_release:
      self._record_trigger(debug_triggers, "explicit_target_low_rollout_strong_decel_teacher_release")
      speed_cap = interp(v_ego, [0.05, 0.12, 0.18, 0.23], [-0.30, -0.32, -0.35, -0.39])
      remaining_cap = interp(remaining_m, [0.48, 0.60, 0.72], [-0.31, -0.34, -0.37])
      decel_cap = interp(a_ego, [-0.76, -0.55, -0.34], [-0.32, -0.35, -0.39])
      teacher_cap = max(speed_cap, remaining_cap, decel_cap)
      end_stop_brake_cap = max(end_stop_brake_cap, teacher_cap)
      target = max(target, teacher_cap)
      brake_step = min(brake_step, interp(v_ego, [0.05, 0.12, 0.23], [0.0010, 0.0014, 0.0020]))
      release_step = max(release_step, interp(v_ego, [0.05, 0.12, 0.23], [0.0180, 0.0160, 0.0120]))

    explicit_target_terminal_teacher_soften = (
      explicit_stop_target_available
      and self.phase in (StoppingPhase.NEAR_HOLD, StoppingPhase.HOLD)
      and 0.0 < v_ego < 0.34
      and 0.24 < remaining_m < 0.48
      and self.low_speed_rollout_m < 1.55
      and -0.56 < a_ego < -0.06
      and low_speed_rebound_risk < 0.70
      and disturbance < 0.10
      and not stop_reacquire_hold_active
      and not release_lock_active
      and not rebound_arrest_active
      and not clutch_push_relief
    )
    if explicit_target_terminal_teacher_soften:
      # The horizon teacher repeatedly softens the last explicit-target tail before standstill,
      # then lets the hold logic reapply brake if needed. Keep that as a bounded terminal cap
      # instead of letting the generic tail/end-stop stack add a low-speed jab.
      self._record_trigger(debug_triggers, "explicit_target_terminal_teacher_soften")
      teacher_cap = interp(v_ego, [0.00, 0.05, 0.10, 0.18, 0.34], [-0.34, -0.34, -0.41, -0.50, -0.66])
      teacher_cap = min(teacher_cap, interp(remaining_m, [0.24, 0.32, 0.48], [-0.36, -0.46, -0.62]))
      end_stop_brake_cap = max(end_stop_brake_cap, teacher_cap)
      target = max(target, teacher_cap)
      brake_step = min(brake_step, interp(v_ego, [0.00, 0.08, 0.18, 0.34], [0.0010, 0.0012, 0.0018, 0.0026]))
      release_step = max(release_step, interp(v_ego, [0.00, 0.08, 0.18, 0.34], [0.0120, 0.0100, 0.0080, 0.0060]))
    end_stop_cap_active = (
      not tail_profile_planner_active
      and not terminal_unwind_delay
      and not explicit_target_tail_settle_active
      and
      self.phase in (StoppingPhase.NEAR_HOLD, StoppingPhase.HOLD)
      and (v_ego < 0.60 or (v_ego < 0.65 and last_output_accel < -0.95))
      and not high_rollout_hold_preserve
      and not pre_hold_rebound_preserve
      and not stop_reacquire_hold_active
      and not clutch_push_relief
      and (target < end_stop_brake_cap or last_output_accel < end_stop_brake_cap)
    )
    if end_stop_cap_active:
      # Clamp inherited deep brake commands near wheel-stop; use a higher release rate to reach the cap before standstill.
      # This is intended to reduce end-stop harshness while keeping enough authority to prevent large low-speed rollout.
      self._record_trigger(debug_triggers, "end_stop_cap_active")
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
        release_step = max(release_step, interp(v_ego, [0.00, 0.60], [0.009, 0.0045]))

    end_stop_high_rollout_release_hold = (
      not tail_profile_planner_active
      and
      end_stop_cap_active
      and 0.13 < v_ego < 0.45
      and self.low_speed_rollout_m > 1.00
      and a_ego < -0.28
      and last_output_accel < -0.38
      and low_speed_rebound_risk < 0.08
      and not release_lock_active
      and not rebound_arrest_active
      and not clutch_push_relief
    )
    if end_stop_high_rollout_release_hold:
      self._record_trigger(debug_triggers, "end_stop_high_rollout_release_hold")
      far_lead_high_rollout_teacher_release = (
        lead_status
        and lead_distance_m is not None
        and lead_distance_m > 4.00
        and remaining_m > 1.20
        and 0.22 < v_ego < 0.38
        and -0.56 < a_ego < -0.30
        and last_output_accel < -0.58
        and low_speed_rebound_risk < 0.08
        and disturbance < 0.08
      )
      if far_lead_high_rollout_teacher_release:
        self._record_trigger(debug_triggers, "far_lead_high_rollout_teacher_release")
        teacher_release_cap = interp(v_ego, [0.22, 0.30, 0.38], [-0.56, -0.58, -0.62])
        target = max(target, teacher_release_cap)
        brake_step = min(brake_step, interp(v_ego, [0.22, 0.30, 0.38], [0.0015, 0.0020, 0.0025]))
        release_step = max(release_step, interp(v_ego, [0.22, 0.30, 0.38], [0.0070, 0.0090, 0.0120]))
      else:
        release_step = min(release_step, interp(v_ego, [0.17, 0.45], [0.0018, 0.0030]))

    no_target_standstill_hold_relax = (
      should_stop
      and not explicit_stop_target_available
      and not lead_status
      and self.phase == StoppingPhase.HOLD
      and v_ego < 0.06
      and a_ego > -0.08
      and self.low_speed_rollout_m > 0.35
      and last_output_accel < -0.40
      and (not rebound_arrest_active or self.low_speed_rollout_m > 0.90)
      and not clutch_push_relief
    )
    if no_target_standstill_hold_relax:
      # After a no-target red-light stop has reached standstill, hold a mild brake instead of
      # ratcheting deeper and then bouncing through starting/stopping on brief signal dropouts.
      self._record_trigger(debug_triggers, "no_target_standstill_hold_relax")
      hold_cap = interp(v_ego, [0.00, 0.03, 0.06], [-0.34, -0.335, -0.32])
      target = max(target, hold_cap)
      brake_step = min(brake_step, interp(v_ego, [0.00, 0.03, 0.06], [0.0010, 0.0012, 0.0015]))
      release_step = max(release_step, interp(v_ego, [0.00, 0.03, 0.06], [0.020, 0.018, 0.014]))

    standstill_relax = (
      self.phase == StoppingPhase.HOLD
      and v_ego <= self.tuning.standstill_settle_speed_mps
      and a_ego > self.tuning.standstill_settle_accel_threshold_mps2
      and self.standstill_settled_time_s >= self.tuning.standstill_relax_time_s
      and not explicit_target_micro_hold_active
      and not release_lock_active
      and not clutch_push_relief
    )
    if standstill_relax:
      # Once vEgo is essentially zero and accel is settled, relax toward a mild hold.
      # This reduces the acceleration step at wheel-stop while relying on disturbance lock to counter creep.
      self._record_trigger(debug_triggers, "standstill_relax")
      relax_target = interp(v_ego, [0.00, 0.02], [-0.12, -0.10])
      target = max(target, relax_target)
      release_step = max(release_step, interp(v_ego, [0.00, 0.02], [0.0045, 0.0035]))

    ineffective_brake_guard = (
      not tail_profile_planner_active
      and
      self.phase in (StoppingPhase.NEAR_HOLD, StoppingPhase.HOLD)
      and v_ego < 0.35
      and a_ego > -0.35
      and last_output_accel < -1.05
      and not clutch_push_relief
    )
    if ineffective_brake_guard:
      # If we're already commanding deep braking but decel remains weak, avoid ratcheting further down.
      # This mitigates end-stop jerk spikes when drivetrain/clutch behavior flips near standstill.
      self._record_trigger(debug_triggers, "ineffective_brake_guard")
      target = max(target, last_output_accel)
      brake_step = min(brake_step, interp(v_ego, [0.06, 0.20, 0.35], [0.0012, 0.0016, 0.0022]))
      release_step = max(release_step, interp(v_ego, [0.06, 0.20, 0.35], [0.0050, 0.0040, 0.0030]))

    rollout_oscillation_damping = (
      not tail_profile_planner_active
      and
      self.low_speed_rollout_m > interp(v_ego, [0.08, 0.25, 0.60, 1.00], [1.30, 1.60, 1.75, 1.80])
      and v_ego < 1.8
      and (release_lock_active or a_ego > -0.25 or disturbance > 0.10)
      and not no_target_standstill_hold_relax
      and not clutch_push_relief
    )
    if rollout_oscillation_damping:
      # In high-rollout rebound cycles, prefer steadier command evolution over aggressive chase/relief swings.
      self._record_trigger(debug_triggers, "rollout_oscillation_damping")
      damping_floor = interp(v_ego, [0.00, 0.20, 0.55, 1.20], [-0.36, -0.42, -0.50, -0.60])
      target = min(target, damping_floor)
      brake_step = min(brake_step, interp(v_ego, [0.00, 0.20, 0.55, 1.20], [0.0023, 0.0032, 0.0042, 0.0051]))
      release_step = min(release_step, interp(v_ego, [0.00, 0.20, 0.55, 1.20], [0.0020, 0.0030, 0.0040, 0.0050]))

    late_stop_high_rollout_soften = (
      not tail_profile_planner_active
      and
      self.phase in (StoppingPhase.NEAR_HOLD, StoppingPhase.HOLD)
      and 0.75 < self.low_speed_rollout_m < 1.05
      and 0.45 < v_ego < 0.70
      and remaining_m < 0.40
      and a_ego < -0.40
      and -0.58 < last_output_accel < -0.46
      and disturbance < 0.04
      and low_speed_rebound_risk < 0.10
      and not release_lock_active
      and not rebound_arrest_active
      and not clutch_push_relief
    )
    if late_stop_high_rollout_soften:
      # Borrow inverse-v3-style softening only in the earlier moderate-rollout landing window.
      # Avoid the later deep-unwind tail where the 71c seeded regressions already release quickly enough.
      self._record_trigger(debug_triggers, "late_stop_high_rollout_soften")
      final_window = clip((0.40 - remaining_m) / 0.40, 0.0, 1.0)
      rollout_factor = clip((self.low_speed_rollout_m - 0.75) / 0.30, 0.0, 1.0)
      decel_factor = clip((-a_ego - 0.40) / 0.35, 0.0, 1.0)
      soften = final_window * max(rollout_factor, 0.5 * decel_factor)
      if soften > 0.0:
        soft_cap = interp(remaining_m, [0.00, 0.06, 0.16, 0.40], [-0.14, -0.16, -0.21, -0.29])
        softened_target = max(target, soft_cap)
        target = ((1.0 - soften) * target) + (soften * softened_target)
        brake_step = min(brake_step, interp(remaining_m, [0.00, 0.16, 0.40], [0.0018, 0.0024, 0.0032]))
        release_step = max(release_step, interp(remaining_m, [0.00, 0.16, 0.40], [0.012, 0.0095, 0.0065]))

    final_high_rollout_settle_guard = (
      not glide_handoff_active
      and self.phase in (StoppingPhase.NEAR_HOLD, StoppingPhase.HOLD)
      and self.low_speed_rollout_m > 1.15
      and 0.08 < v_ego < 0.22
      and a_ego > -0.12
      and not clutch_push_relief
    )
    if final_high_rollout_settle_guard:
      self._record_trigger(debug_triggers, "final_high_rollout_settle_guard")
      settle_floor = interp(v_ego, [0.08, 0.14, 0.22], [-0.40, -0.42, -0.44])
      target = min(target, settle_floor)
      brake_step = max(brake_step, interp(v_ego, [0.08, 0.14, 0.22], [0.012, 0.010, 0.008]))
      release_step = min(release_step, interp(v_ego, [0.08, 0.14, 0.22], [0.0010, 0.0014, 0.0019]))

    hold_acquisition_step_cap = (HOLD_ACQUISITION_SOFTEN_ARREST_BRAKE_STEP if rebound_arrest_active
                                 else HOLD_ACQUISITION_SOFTEN_BRAKE_STEP)
    hold_acquisition_soften = (
      0.0 <= v_ego < HOLD_ACQUISITION_SOFTEN_V_MAX
      and abs(a_ego) < HOLD_ACQUISITION_SOFTEN_A_EGO_MAX
      and disturbance < HOLD_ACQUISITION_SOFTEN_DISTURBANCE_MAX
      and last_output_accel < HOLD_ACQUISITION_SOFTEN_LAST_CMD_MAX
      and brake_step > hold_acquisition_step_cap
      and not clutch_push_relief
    )
    if hold_acquisition_soften:
      # Stationary-stable DEEP-ramp hold acquisition: cap the per-frame deepening RATE only --
      # every target (incl. rebound_arrest_cap) is untouched, so the full hold force is still
      # reached, just without the arrest slam. The v/a_ego/disturbance gates are blind to a fresh
      # grade re-roll (see constants block), hence the two safeguards: the shallow catch above
      # LAST_CMD_MAX never softens, and while rebound_arrest_active the rate is floored at
      # 2.0 m/s^3 instead of the 1.0 m/s^3 comfort rate. The latched release_lock is deliberately
      # NOT a disqualifier here: it outlives a dead creep push by ~1.1 s and would defeat the
      # soften in exactly the scenario it targets; a re-appearing push flips the live
      # `disturbance` (or the v/a_ego bands) and restores the fast lanes the same frame.
      self._record_trigger(debug_triggers, "hold_acquisition_soften")
      brake_step = hold_acquisition_step_cap

    # P2 terminal settle jerk cap (2026-06-13): bound the commanded deepening rate across the whole
    # terminal settle band so the command-side contribution to the felt disc-grab stays under the
    # cranked jerk budget. Rate cap only (targets/floors untouched -- full hold force still reached).
    # SAFETY: it is fully DISABLED whenever a live push (disturbance), the release-inhibit lock, the
    # clutch-push relief, OR the rebound arrest is active -- those lanes own creep/rollback and
    # hill-hold catch, and the arrest first-beat can legitimately want MORE deepening than this cap
    # (the hill-hold blind-window catch), so the cap must never reduce their authority.
    terminal_settle_jerk_cap = (
      0.0 <= v_ego < TERMINAL_SETTLE_V_MAX
      and disturbance < TERMINAL_SETTLE_DISTURBANCE_MAX
      and not release_lock_active
      and not rebound_arrest_active
      and not clutch_push_relief
      and brake_step > TERMINAL_SETTLE_BRAKE_STEP
    )
    if terminal_settle_jerk_cap:
      self._record_trigger(debug_triggers, "terminal_settle_jerk_cap")
      brake_step = TERMINAL_SETTLE_BRAKE_STEP

    brake_step = max(0.0004, brake_step)
    release_step = max(0.0004, release_step)

    # `brake_step`/`release_step` are tuned for the 100Hz control loop (dt ~= 0.01s).
    # Scale by dt so offline replays sampled at lower rates behave comparably.
    dt_scale = clip(dt / 0.01, 0.5, 20.0)
    brake_step *= dt_scale
    release_step *= dt_scale

    limited_output = clip(target, last_output_accel - brake_step, last_output_accel + release_step)
    stop_entry_output_cap = -0.05
    if stop_entry_soften_active and last_output_accel > -0.02:
      stop_entry_output_cap = interp(v_ego, [0.00, 0.12, 0.25, 0.40, 0.80], [0.00, -0.005, -0.015, -0.025, -0.05])
    limited_output = clip(limited_output, stop_accel, stop_entry_output_cap)
    if stop_intent_active and self.stop_entry_soften_counter > 0:
      self.stop_entry_soften_counter -= 1
    explicit_lead_gap_soften_base = (
      explicit_stop_target_available
      and lead_status
      and lead_distance_m is not None
      and 0.75 < remaining_m < 3.20
      and 0.08 < v_ego < 0.90
      and self.low_speed_rollout_m < 0.85
      and a_ego < -0.28
      and last_output_accel < -0.26
    )
    explicit_lead_glide_soften = explicit_lead_gap_soften_base and 3.20 < lead_distance_m <= 5.00
    explicit_lead_long_gap_glide = (
      explicit_stop_target_available
      and lead_status
      and lead_distance_m is not None
      and 5.00 < lead_distance_m <= 8.00
      and 0.75 < remaining_m < 4.80
      and 0.02 < v_ego < 0.90
      and self.low_speed_rollout_m < 1.20
      and a_ego < -0.06
      and lead_v <= 0.25
      and last_output_accel < -0.08
    )
    if explicit_lead_glide_soften:
      speed_soft_cap = interp(v_ego, [0.08, 0.20, 0.45, 0.70, 0.90], [-0.36, -0.40, -0.46, -0.50, -0.54])
      remaining_soft_cap = interp(remaining_m, [0.75, 1.00, 1.50, 2.40, 3.20], [-0.54, -0.50, -0.45, -0.40, -0.36])
      lead_soft_cap = interp(lead_distance_m, [3.20, 4.20, 5.00], [-0.52, -0.48, -0.44])
      glide_soft_cap = min(speed_soft_cap, remaining_soft_cap, lead_soft_cap)
      if limited_output < glide_soft_cap:
        limited_output = max(limited_output, glide_soft_cap)
        self._record_trigger(debug_triggers, "explicit_lead_glide_soften")
    elif explicit_lead_long_gap_glide:
      speed_soft_cap = interp(v_ego, [0.02, 0.20, 0.45, 0.70, 0.90], [-0.06, -0.09, -0.13, -0.19, -0.26])
      remaining_soft_cap = interp(remaining_m, [0.75, 1.00, 1.80, 3.00, 4.80], [-0.24, -0.20, -0.15, -0.11, -0.08])
      lead_soft_cap = interp(lead_distance_m, [5.00, 6.50, 8.00], [-0.20, -0.14, -0.10])
      glide_soft_cap = min(speed_soft_cap, remaining_soft_cap, lead_soft_cap)
      if limited_output < glide_soft_cap:
        limited_output = max(limited_output, glide_soft_cap)
        self._record_trigger(debug_triggers, "explicit_lead_long_gap_glide")
    explicit_lead_smooth_tail_release = (
      explicit_stop_target_available
      and lead_status
      and lead_distance_m is not None
      and 3.35 < lead_distance_m <= 5.00
      and lead_v <= 0.25
      and 0.16 < v_ego < 0.44
      and 0.72 < remaining_m < 1.70
      and 0.18 < self.low_speed_rollout_m < 0.56
      and -0.52 < a_ego < -0.02
      and -0.48 < last_output_accel < -0.16
      and low_speed_rebound_risk < 0.45
      and disturbance < 0.12
      and not stop_reacquire_hold_active
      and not release_lock_active
      and not rebound_arrest_active
      and not low_speed_rebound_cap_active
      and not low_speed_rebound_cap_relief
      and not clutch_push_relief
    )
    if explicit_lead_smooth_tail_release:
      # Route-backed smooth-stop lane: when a stopped lead is still at a healthy gap and the
      # explicit target remains ahead, release the low-speed tail before the final settle/deepen.
      speed_tail_cap = interp(v_ego, [0.16, 0.22, 0.30, 0.44], [-0.18, -0.20, -0.24, -0.32])
      remaining_tail_cap = interp(remaining_m, [0.72, 0.90, 1.20, 1.70], [-0.30, -0.24, -0.22, -0.26])
      lead_tail_cap = interp(lead_distance_m, [3.35, 3.70, 4.20, 5.00], [-0.30, -0.26, -0.22, -0.20])
      smooth_tail_cap = min(speed_tail_cap, remaining_tail_cap, lead_tail_cap)
      if limited_output < smooth_tail_cap:
        limited_output = max(limited_output, smooth_tail_cap)
        self._record_trigger(debug_triggers, "explicit_lead_smooth_tail_release")

    # P1 gentle-approach decel cap (2026-06-13): final least-negative floor while the lead gap is
    # still comfortable and the car is still rolling, released kinematically so it never
    # under-brakes a real closing threat. Rate-limited toward the floor (never shallower than the
    # output already was) so engaging/releasing it injects no jerk. This is the ACTIVE-lane
    # enforcement the eval can measure; the longcontrol cap covers the PID-lane approach origin.
    approach_decel_floor = stopping_phase_approach_decel_floor(v_ego, lead_status, lead_v, lead_distance_m) if APPROACH_DECEL_CAP_ENABLED else None
    if approach_decel_floor is not None and limited_output < approach_decel_floor:
      # Raise the command up toward the gentle floor. Easing OFF the brake is a release (inherently
      # comfortable), but rate-limit it so a deep inherited command unwinds smoothly instead of
      # snapping to the floor in one frame; never go shallower than the floor, never deeper than the
      # raw output. The upstream brake_step already governs the deepening side.
      target_release = min(approach_decel_floor, last_output_accel + APPROACH_DECEL_CAP_RELEASE_STEP * dt_scale)
      limited_output = max(limited_output, target_release)
      self._record_trigger(debug_triggers, "approach_decel_cap")

    # Anti-stiction terminal pre-release (2026-06-14): in the final approach to standstill -- ABOVE
    # the gate-0.01 StopReq handoff and ABOVE the standstill band -- ease the brake command OFF the
    # deep terminal hold toward the shallow -A_TERMINAL_PRERELEASE floor, jerk-limited, so the car
    # eases THROUGH the static-friction transition instead of clamping hard into it. Below V_LO the
    # ease disengages and the deepening stack (incl. hold-acquisition soften) re-applies the full
    # hold. This is the LAST terminal shaper and a RELEASE-side floor only: it can only raise the
    # command toward the floor (shallower), never deepen it, never go below the floor. Two-regime
    # safety (hold-acquisition discipline): fully disabled on any live disturbance / release lock /
    # rebound arrest / clutch push / elevated rebound risk / insufficient decel, so a downhill
    # grade-pull restores full brake the same frame. SIM CANNOT prove the felt-grab reduction (no
    # stiction in the plant); that is deferred to the on-road IMU settle_peak_imu_jerk measurement.
    terminal_prerelease_active = (
      stop_intent_active
      and self.phase in (StoppingPhase.NEAR_HOLD, StoppingPhase.HOLD)
      and TERMINAL_PRERELEASE_V_LO < v_ego < TERMINAL_PRERELEASE_V_HI
      and remaining_m < TERMINAL_PRERELEASE_REMAINING_MAX
      and a_ego <= -TERMINAL_PRERELEASE_A_EGO_MAX
      and disturbance < TERMINAL_PRERELEASE_DISTURBANCE_MAX
      and low_speed_rebound_risk < TERMINAL_PRERELEASE_REBOUND_RISK_MAX
      and not release_lock_active
      and not rebound_arrest_active
      and not clutch_push_relief
    )
    if terminal_prerelease_active and limited_output < -A_TERMINAL_PRERELEASE:
      # Ease UP toward the shallow floor at the jerk-limited release rate; never shallower than the
      # floor, never deeper than the raw output already was (the brake_step deepening side is
      # untouched -- if the raw command is still deepening this just bounds how shallow the ease
      # can reach this frame). Easing OFF the brake is a release, inherently comfortable.
      prerelease_floor = -A_TERMINAL_PRERELEASE
      target_release = min(prerelease_floor, last_output_accel + TERMINAL_PRERELEASE_RELEASE_STEP * dt_scale)
      if limited_output < target_release:
        limited_output = max(limited_output, target_release)
        self._record_trigger(debug_triggers, "terminal_prerelease")

    if debug is not None:
      shadow_decision = self.shadow_oracle.evaluate(
        StoppingShadowInput(
          output_accel=limited_output,
          last_output_accel=last_output_accel,
          should_stop=stop_intent_active,
          v_ego=v_ego,
          a_ego=a_ego,
          stop_accel=stop_accel,
          remaining_m=remaining_m,
          explicit_target_available=explicit_stop_target_available,
          rollout_m=self.low_speed_rollout_m,
          phase=int(self.phase),
          release_lock_active=release_lock_active,
          rebound_arrest_active=rebound_arrest_active,
          lead_status=lead_status,
          lead_v=lead_v,
          lead_d_rel=lead_d_rel,
        )
      )
      shadow_decision.write_debug(debug)
      debug["shadow_authority_active"] = False
    if debug is not None and debug_triggers is not None:
      debug["triggers"] = tuple(debug_triggers)
    return StoppingResult(output_accel=limited_output, release_lock_active=release_lock_active)
