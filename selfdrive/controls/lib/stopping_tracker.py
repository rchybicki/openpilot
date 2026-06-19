"""Stopping tracking law (stopping redesign spec sections 5.4-5.5, WP5).

Owns the persistent stopping-state estimate: command history (delay compensation against the
re-discretized PlantModel), the lumped disturbance estimate ``d_hat`` (LPF with the
``DIST_LPF_TAU_S = 0.0`` kill-switch bypass to legacy single-frame semantics), the
release-inhibit timer, the rollout odometer + recovery integrator (G9 verbatim), the arrest
mechanism (G8 authority, red-team F27) and the asymmetric jerk limiter.

Normative clamp order (spec 5.5.5, red-team F7/F26):
  (1) a_cmd_target from ref.a_ref + disturbance deepening + recovery deepening (+ delay guard)
  (2) TERMINAL end-stop ceiling re-clamp: quiescent A_END_STOP(v); deepened to
      min(A_END_STOP, A_DISTURBANCE_FLOOR) while push-active and to A_ARREST_MAX while
      arrest-active -- recovery extends braking DURATION, never terminal MAGNITUDE
  (3) slew with j_brake_eff / j_release_eff per the mechanism precedence below
  (4) SETTLE/HOLD absolute bound: deepening below A_ARREST_MAX(v) is never allowed

The FINAL authority clip (clip(u, stop_accel, -0.05) with the verbatim stop-entry-soften
exception) and the non-finite output fallback (spec 5.5.5 steps 4-5) are applied by the facade
(stopping_controller_v2.py), which owns the per-frame pipeline -- the split is purely
structural (the entry exception needs raw_should_stop edge state the spec tracker signature
does not carry); the order of operations matches the normative list.

This limiter runs on EVERY stopping frame (slew ownership, spec 6.4); the longcontrol global
low-speed slew is exempted for the whole stopping state (V2 is the only stopping controller).
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from openpilot.selfdrive.controls.lib.stop_target_arbiter import StopDecision, StopSource
from openpilot.selfdrive.controls.lib.stopping_params import STOPPING_PARAMS, StoppingParams
from openpilot.selfdrive.controls.lib.stopping_plant import PLANT_PARAMS_REF, PlantModel
from openpilot.selfdrive.controls.lib.stopping_trajectory import StopReference, TrajPhase, end_stop_ceiling

interp = np.interp

# --- G5 clutch/TC push-relief signature constants (verbatim, stopping_controller.py:828-839) ------
# Outer speed gate PUSH_RELIEF_V_MIN/MAX lives in stopping_params (red-team F31: below 0.12 m/s
# the disturbance-floor/arrest path owns the response exclusively).
RELIEF_CMD_DEEP = -0.65        # last_output_accel < -0.65 (:830)
RELIEF_A_EGO_PUSH = 0.08       # a_ego > 0.08 (:832)
RELIEF_V_INNER = 1.0           # second branch inner gate v_ego < 1.0 (:834) -- kept verbatim
RELIEF_A_EGO_MILD = -0.25      # a_ego > -0.25 (:835)
RELIEF_CMD_VERY_DEEP = -0.85   # last_output_accel < -0.85 (:836)

# Relief release budget: the spec mandates "cap target at A_PUSH_RELIEF_CAP and freeze deepening"
# (5.5.2) but names no release rate; without one an inherited -0.9 sheds to the cap at the locked
# 0.1-0.6 m/s^3 (seconds) where the forest does it in ~0.25 s. Verbatim provenance: the legacy
# lock-branch relief release cap x100 (stopping_controller.py:1768). Documented WP5 deviation.
PUSH_RELIEF_RELEASE_TABLE = ((0.00, 0.60, 1.20, 1.80, 2.50), (1.95, 2.15, 2.35, 2.55, 2.75))

ROLLOUT_INTEGRATE_V_MAX = 1.2  # rollout odometer integration bound (stopping_controller.py:294)
SETTLED_TIME_CAP_S = 5.0       # settled-time accumulator cap (stopping_controller.py:821)
SETTLED_A_EGO_ABS = 0.05       # |a_ego| settle gate (spec 5.3; legacy one-sided -0.05, :820)


@dataclass(frozen=True)
class TrackerResult:
  output_accel: float            # m/s^2 slewed stopping-state command (facade applies the final authority clip)
  release_inhibit_active: bool   # TELEMETRY ONLY -- does NOT feed stopping_guard (slew restructure, spec 6.4)
  disturbance: float             # m/s^2 filtered (a_ego - expected); > 0 = push (creep/grade)
  rollout_m: float               # low-speed rollout odometer


class StoppingTracker:
  def __init__(self, p: StoppingParams = STOPPING_PARAMS, plant: PlantModel | None = None):
    self.params = p
    # the injected plant (if any) supplies the PlantParams; the model is re-discretized to the
    # call-time dt (one code path for 100 Hz runtime and 10/20 Hz replay, spec 5.1)
    self._plant_params = plant.params if plant is not None else PLANT_PARAMS_REF
    self._plant = plant
    self._cmd_history: list[float] = []
    self.reset()

  def reset(self) -> None:
    """Full state reset. Called by the facade exactly where legacy resets (longcontrol :902-903
    intent loss / off state) and on driver-brake USER_DISABLE disengage (spec 5.5.6, F5):
    no stale d_hat / recovery_i / rollout may survive into a fresh engagement."""
    self._cmd_history = []
    self.d_hat = 0.0
    self.release_inhibit_timer_s = 0.0
    self.rollout_m = 0.0
    self.recovery_i = 0.0
    self.settled_time_s = 0.0
    self.arrest_active = False
    self._arrest_falling_time_s = 0.0
    self._a_ego_prev: float | None = None
    self._v_prev: float | None = None

  def seed_command_history(self, commands: list[float]) -> None:
    history = [float(cmd) for cmd in commands]
    max_len = int(self.params.CMD_HISTORY_LEN)
    self._cmd_history = history[-max_len:] if len(history) > max_len else history

  # --- internals ----------------------------------------------------------------------------------

  def _plant_for_dt(self, dt: float) -> PlantModel:
    if self._plant is None or abs(self._plant.dt - dt) > 1e-9:
      self._plant = PlantModel(self._plant_params, dt)
    return self._plant

  def _append_command(self, last_output_accel: float) -> None:
    self._cmd_history.append(float(last_output_accel))
    max_len = int(self.params.CMD_HISTORY_LEN)
    if len(self._cmd_history) > max_len:
      self._cmd_history = self._cmd_history[-max_len:]

  def _delayed_command(self, delay_frames: int, fallback: float) -> float:
    # verbatim semantics of stopping_controller.py:314-320
    if not self._cmd_history:
      return fallback
    idx = len(self._cmd_history) - 1 - int(delay_frames)
    return self._cmd_history[max(idx, 0)]

  def _delay_release_guard(self, v_ego: float, last_output_accel: float, dt: float) -> float:
    """G2 delay-release guard activation in [0, 1] (verbatim, stopping_controller.py:322-327).
    Retirement: spec 3.2 row 3 (Phase 2 -> PlantModel.rollforward)."""
    p = self.params
    delay_frames = int(np.clip(int(round(p.ACTUATOR_DELAY_S / max(dt, 1e-3))), 1, 25))  # :816
    delayed_cmd = self._delayed_command(delay_frames, fallback=last_output_accel)
    release_relief = float(np.clip(last_output_accel - delayed_cmd, 0.0, p.DELAY_RELIEF_CLIP_MAX))
    relief_trigger = float(interp(v_ego, p.DELAY_RELIEF_TRIGGER_TABLE[0], p.DELAY_RELIEF_TRIGGER_TABLE[1]))
    relief_scale = float(interp(v_ego, p.DELAY_RELIEF_SCALE_TABLE[0], p.DELAY_RELIEF_SCALE_TABLE[1]))
    return float(np.clip((release_relief - relief_trigger) / max(relief_scale, 1e-3), 0.0, 1.0))

  def _sanitize_state(self) -> None:
    # one adversarial NaN/inf frame must not latch into the estimator state (spec 5.5.5 step 5 /
    # F8 second layer; the facade output fallback is the first)
    if not math.isfinite(self.d_hat):
      self.d_hat = 0.0
    if not math.isfinite(self.release_inhibit_timer_s):
      self.release_inhibit_timer_s = 0.0
    if not math.isfinite(self.rollout_m):
      self.rollout_m = 0.0
    if not math.isfinite(self.recovery_i):
      self.recovery_i = 0.0
    if not math.isfinite(self.settled_time_s):
      self.settled_time_s = 0.0
    if self._cmd_history and not all(math.isfinite(c) for c in self._cmd_history):
      self._cmd_history = [c for c in self._cmd_history if math.isfinite(c)]

  # --- per-frame update (spec 5.5) ------------------------------------------------------------------

  def update(self, *, ref: StopReference, decision: StopDecision,
             v_ego: float, a_ego: float, last_output_accel: float,
             max_expected_accel: float, min_expected_accel: float,
             stop_accel: float, dt: float,
             debug: dict | None = None) -> TrackerResult:
    del stop_accel  # the facade applies the final authority clip (module docstring)
    p = self.params
    v = float(v_ego)
    plant = self._plant_for_dt(float(dt))

    # intent tiering (spec 5.2.5 / F32): longcontrol-forced holds are FULL stop intent; only the
    # non-forced timed dropout hold gates the deepening responses below
    tiered_intent = decision.stop_request_active and (decision.source != StopSource.DROPOUT_HOLD or decision.legacy_forced)

    # --- state stage (mirrors the forest's order, stopping_controller.py:814-860) ---
    self._append_command(last_output_accel)

    # expected response (5.5.1): plant prediction from the dead-time-delayed sent command,
    # sanity-clamped to the G3 expected envelope (param #30)
    a_prev = self._a_ego_prev if self._a_ego_prev is not None else float(a_ego)
    cmd_delayed = self._delayed_command(plant.delay_frames, fallback=float(last_output_accel))
    a_exp = plant.predict_next(a_prev, cmd_delayed, v)
    a_exp = min(max(a_exp, float(min_expected_accel)), float(max_expected_accel))

    # disturbance estimator (5.5.2); DIST_LPF_TAU_S = 0.0 is the KILL SWITCH bypass to the
    # legacy single-frame trigger semantics
    innovation = float(a_ego) - a_exp
    if p.DIST_LPF_TAU_S <= 0.0:
      self.d_hat = innovation
    else:
      alpha = min(float(dt) / p.DIST_LPF_TAU_S, 1.0)
      self.d_hat += alpha * (innovation - self.d_hat)

    push_thresh = p.DIST_PUSH_THRESH_LOW if v < p.DIST_PUSH_THRESH_V_SPLIT else p.DIST_PUSH_THRESH_HIGH
    braking = float(last_output_accel) < p.DIST_PUSH_MIN_BRAKE
    push_signature = self.d_hat >= push_thresh and braking
    push_detected = (push_signature and p.DIST_PUSH_V_MIN < v < p.DIST_PUSH_V_MAX and tiered_intent)
    if push_detected:
      self.release_inhibit_timer_s = max(self.release_inhibit_timer_s,
                                         float(interp(v, p.T_RELEASE_INHIBIT_TABLE[0], p.T_RELEASE_INHIBIT_TABLE[1])))
    else:
      self.release_inhibit_timer_s = max(self.release_inhibit_timer_s - float(dt), 0.0)
    push_active = self.release_inhibit_timer_s > 0.0

    overbrake = float(a_ego) < (float(min_expected_accel) - p.OVERBRAKE_TRIGGER_MARGIN)

    # G5 deep-command push signature, gated to the legacy 0.12 < v < 2.5 window (F31)
    relief = (p.PUSH_RELIEF_V_MIN < v < p.PUSH_RELIEF_V_MAX
              and float(last_output_accel) < RELIEF_CMD_DEEP
              and (float(a_ego) > RELIEF_A_EGO_PUSH
                   or (v < RELIEF_V_INNER and float(a_ego) > RELIEF_A_EGO_MILD
                       and float(last_output_accel) < RELIEF_CMD_VERY_DEEP)))

    # arrest mechanism (5.5.2 / F27): push fires below ARREST_V_MAX with rising v
    v_rising = self._v_prev is not None and v > self._v_prev
    v_falling = self._v_prev is not None and v < self._v_prev
    if not self.arrest_active:
      if push_detected and v < p.ARREST_V_MAX and v_rising:
        self.arrest_active = True
        self._arrest_falling_time_s = 0.0
    else:
      self._arrest_falling_time_s = self._arrest_falling_time_s + float(dt) if v_falling else 0.0
      if not push_signature or self._arrest_falling_time_s >= p.ARREST_EXIT_FALLING_T_S:
        self.arrest_active = False
        self._arrest_falling_time_s = 0.0

    # rollout odometer (G9 verbatim, stopping_controller.py:287-297; reset = facade reset on
    # intent loss, matching legacy _update_low_speed_rollout(should_stop=False))
    if v <= p.V_STANDSTILL_SETTLED:
      self.rollout_m = max(self.rollout_m - p.ROLLOUT_DECAY_MPS * float(dt), 0.0)
    elif v < ROLLOUT_INTEGRATE_V_MAX:
      self.rollout_m += v * float(dt)
    else:
      self.rollout_m = max(self.rollout_m - v * float(dt), 0.0)

    # settled time (spec 5.3)
    if v <= p.V_STANDSTILL_SETTLED and abs(float(a_ego)) <= SETTLED_A_EGO_ABS:
      self.settled_time_s = min(self.settled_time_s + float(dt), SETTLED_TIME_CAP_S)
    else:
      self.settled_time_s = 0.0

    # recovery integrator (G9 verbatim gains, stopping_controller.py:849-860; retirement 3.2 row 5)
    if ref.phase != TrajPhase.TRACK and not relief and tiered_intent:
      desired = float(interp(v, p.A_DESIRED_LOWSPEED_TABLE[0], p.A_DESIRED_LOWSPEED_TABLE[1]))
      shortfall = float(np.clip(float(a_ego) - desired, 0.0, 1.2))
      arm = float(interp(v, p.RECOVERY_ARM_TABLE[0], p.RECOVERY_ARM_TABLE[1]))
      if self.rollout_m > arm and shortfall > 0.0:
        growth = shortfall * float(interp(v, p.RECOVERY_GAIN_TABLE[0], p.RECOVERY_GAIN_TABLE[1])) * float(dt)
        self.recovery_i = float(np.clip(self.recovery_i + growth, 0.0, p.RECOVERY_CAP))
      else:
        decay = float(interp(v, p.RECOVERY_DECAY_TABLE[0], p.RECOVERY_DECAY_TABLE[1])) * float(dt)
        self.recovery_i = max(self.recovery_i - decay, 0.0)
    else:
      self.recovery_i = 0.0

    # --- (1) command target assembly ---
    a_cmd_target = float(ref.a_ref)
    j_brake = float(ref.j_brake_max)
    j_release = float(ref.j_release_max)

    # dropout edge case (5.5.6): a non-forced timed hold keeps the envelope, never deepens
    # beyond A_NEAR_HOLD(v)
    if decision.source == StopSource.DROPOUT_HOLD and not decision.legacy_forced:
      a_cmd_target = max(a_cmd_target, float(interp(v, p.A_NEAR_HOLD_TABLE[0], p.A_NEAR_HOLD_TABLE[1])))

    dist_floor = float(interp(v, p.A_DISTURBANCE_FLOOR_TABLE[0], p.A_DISTURBANCE_FLOOR_TABLE[1]))
    if push_active and tiered_intent and not overbrake and not relief:
      # push deepening toward the lock floor -- bypasses the TERMINAL ceiling (5.3 scope / F26)
      a_cmd_target = min(a_cmd_target, dist_floor)

    recovery_applied = self.recovery_i > 0.0 and not relief and tiered_intent
    if recovery_applied:
      a_cmd_target -= self.recovery_i * float(interp(v, p.RECOVERY_APPLY_GAIN_TABLE[0], p.RECOVERY_APPLY_GAIN_TABLE[1]))
      j_brake = max(j_brake, float(interp(v, p.RECOVERY_BRAKE_FLOOR_TABLE[0], p.RECOVERY_BRAKE_FLOOR_TABLE[1])))
      j_release = min(j_release, float(interp(v, p.RECOVERY_RELEASE_CAP_TABLE[0], p.RECOVERY_RELEASE_CAP_TABLE[1])))

    # delay-release guard (5.5.4, verbatim; retirement 3.2 row 3)
    guard = self._delay_release_guard(v, float(last_output_accel), float(dt))
    if guard > 0.0 and not relief:
      j_release = min(j_release, float(interp(v, p.DELAY_RELEASE_CAP_TABLE[0], p.DELAY_RELEASE_CAP_TABLE[1])))
      a_cmd_target -= guard * float(interp(v, p.DELAY_RELEASE_BIAS_TABLE[0], p.DELAY_RELEASE_BIAS_TABLE[1]))

    arrest_max = float(interp(v, p.A_ARREST_MAX_TABLE[0], p.A_ARREST_MAX_TABLE[1]))
    if self.arrest_active:
      # deepen toward an arrest target bounded by A_ARREST_MAX (G8 authority)
      a_cmd_target = min(a_cmd_target, arrest_max)

    if relief:
      # saturated brake authority: relieve toward the cap, never chase deeper (G5)
      a_cmd_target = max(a_cmd_target, float(interp(v, p.A_PUSH_RELIEF_CAP_TABLE[0], p.A_PUSH_RELIEF_CAP_TABLE[1])))

    # --- (2) TERMINAL end-stop ceiling re-clamp (after recovery: duration, not magnitude; F7) ---
    ceiling_binds = False
    if ref.phase != TrajPhase.TRACK and v < p.A_END_STOP_TABLE[0][-1] and not relief:
      if self.arrest_active:
        ceiling = arrest_max
      elif push_active and tiered_intent:
        ceiling = min(end_stop_ceiling(v, p), dist_floor)
      else:
        ceiling = end_stop_ceiling(v, p)
      ceiling_binds = a_cmd_target < ceiling or float(last_output_accel) < ceiling  # legacy :2376
      a_cmd_target = max(a_cmd_target, ceiling)

    # --- (3) slew: asymmetric jerk limiter, mechanism precedence per spec 5.5.5(3) ---
    if self.arrest_active:
      j_brake = float(interp(v, p.J_ARREST_TABLE[0], p.J_ARREST_TABLE[1]))  # F27: catch the surge
    if relief:
      j_brake = 0.0  # freeze deepening (5.5.2)
      j_release = max(j_release, float(interp(v, PUSH_RELIEF_RELEASE_TABLE[0], PUSH_RELIEF_RELEASE_TABLE[1])))
    elif push_active:
      if overbrake:
        # overbrake release floor replaces the locked cap (G4 verbatim structure, :1764-1766)
        j_release = max(j_release, float(interp(v, p.OVERBRAKE_RELEASE_FLOOR_TABLE[0], p.OVERBRAKE_RELEASE_FLOOR_TABLE[1])))
      else:
        j_release = min(j_release, float(interp(v, p.J_RELEASE_LOCKED_TABLE[0], p.J_RELEASE_LOCKED_TABLE[1])))
    elif (ceiling_binds and ref.phase == TrajPhase.TERMINAL
          and not (v < p.J_END_STOP_RELEASE_SUPPRESS_V and self.arrest_active)):
      # F30: shed an inherited deep brake to the binding ceiling BEFORE wheel-stop
      j_release = max(j_release, float(interp(v, p.J_END_STOP_RELEASE_TABLE[0], p.J_END_STOP_RELEASE_TABLE[1])))

    lo = float(last_output_accel) - j_brake * float(dt)
    hi = float(last_output_accel) + j_release * float(dt)
    u = min(max(a_cmd_target, lo), hi)

    # --- (4) SETTLE/HOLD absolute deepening bound (spec 5.5.5(4) second clause) ---
    if ref.phase in (TrajPhase.SETTLE, TrajPhase.HOLD):
      u = max(u, arrest_max)

    self._a_ego_prev = float(a_ego)
    self._v_prev = v
    self._sanitize_state()

    if debug is not None:
      debug["a_exp"] = a_exp
      debug["push_active"] = bool(push_active)
      debug["arrest_active"] = bool(self.arrest_active)
      debug["relief_active"] = bool(relief)
      debug["overbrake_active"] = bool(overbrake)
      debug["delay_release_guard"] = guard
      debug["ceiling_binds"] = bool(ceiling_binds)

    return TrackerResult(output_accel=u,
                         release_inhibit_active=push_active,
                         disturbance=self.d_hat,
                         rollout_m=self.rollout_m)
