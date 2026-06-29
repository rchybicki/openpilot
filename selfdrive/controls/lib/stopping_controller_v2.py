"""StoppingControllerV2 -- the V2 stopping facade (stopping redesign spec sections 2, 5.4, WP5).

Facade with the legacy seam (stopping_controller.py:664-681) EXTENDED by exactly one trailing
keyword: ``decision``. There is ONE stop-target arbiter in the system and it lives in
LongControl (spec section 6 Commit B); this facade NEVER instantiates or calls one -- it
consumes the StopDecision longcontrol already computed this frame (red-team F2; an AST guard in
test_stop_target_arbiter.py enforces that only the StopDecision/StopSource data types are
imported here). The legacy StoppingController never receives the kwarg (longcontrol passes it
only on the V2 branch).

Pipeline per stopping frame (spec 5.4):
  ref    = stop_reference(...)           # stopping_trajectory, uses decision.target_distance_m
  result = tracker.update(ref, decision, ...)   # stopping_tracker, spec 5.5 steps 1-3
  u      = final authority clip + verbatim stop-entry-soften exception (spec 5.5.5 step 4)
  u      = non-finite fallback to last_output_accel + 'nonfinite_fallback' flag (step 5, F8)
  return StoppingResult(u, release_lock_active=False)

``release_lock_active`` is ALWAYS False from the facade (seam compatibility only; under V2 no
lock wire feeds stopping_guard -- spec 6.4). ``low_speed_rollout_m`` and ``phase`` mirror the
legacy attributes longcontrol reads via getattr (longcontrol.py:637-644).

Driver brake/regen tap (spec 5.5.6, F5): a rising-edge press disengages openpilot entirely
(USER_DISABLE); longcontrol's off-state reset() path fully clears the tracker -- clean reset,
clean re-engage, no stale d_hat / recovery_i / rollout.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from openpilot.selfdrive.controls.lib.stop_target_arbiter import StopDecision
from openpilot.selfdrive.controls.lib.stopping_params import STOPPING_PARAMS, StoppingParams
from openpilot.selfdrive.controls.lib.stopping_tracker import StoppingTracker
from openpilot.selfdrive.controls.lib.stopping_trajectory import TrajPhase, stop_reference
from opendbc.car.hyundai.values import CAR as HYUNDAI_CAR

interp = np.interp

DEBUG_VERSION = "v2_tracker_1"

# Verbatim stop-entry-soften arming gates + window (stopping_controller.py:133-148); the cap
# tables live in stopping_params (param #35, STOP_ENTRY_OUTPUT_CAP_TABLE). The legacy 100 Hz
# frame window converts to seconds (frames / 100) -- no dt_scale (spec conventions).
ENTRY_SOFTEN_WINDOW_S_TABLE = ((0.12, 0.30, 0.60, 1.00, 1.65), (0.28, 0.24, 0.20, 0.18, 0.16))

# Legacy guarded mild-braking dropout envelope (verbatim tables, stopping_controller.py:746-810;
# per-frame steps x100 -> physical m/s^3 rates, consumed as rate * dt). Emitted for one hold
# period (T_STOP_INTENT_HOLD_S) when intent fully expires while 0 < v < 0.24 with active decel
# (spec 5.5.6 dropout edge case), then reset + passthrough.
_DROPOUT_HOLD_FLOOR = ((0.00, 0.08, 0.24), (-0.24, -0.22, -0.16))
_DROPOUT_MICRO_FLOOR = ((0.00, 0.03, 0.06), (-0.30, -0.27, -0.24))
_DROPOUT_ACTIVE_RELEASE_FLOOR = ((0.06, 0.12, 0.22), (-0.32, -0.29, -0.22))
_DROPOUT_LATE_FLOOR = ((0.06, 0.12, 0.20), (-0.34, -0.31, -0.24))
_DROPOUT_LATE_ONSET_BRAKE = ((0.03, 0.08, 0.16), (5.5, 4.0, 2.4))
_DROPOUT_LATE_ONSET_RELEASE = ((0.03, 0.08, 0.16), (0.09, 0.14, 0.22))
_DROPOUT_MICRO_BRAKE = ((0.00, 0.03, 0.06), (1.2, 1.0, 0.8))
_DROPOUT_MICRO_RELEASE = ((0.00, 0.03, 0.06), (0.06, 0.08, 0.11))
_DROPOUT_ACTIVE_RELEASE_BRAKE = ((0.06, 0.12, 0.22), (1.2, 0.9, 0.7))
_DROPOUT_ACTIVE_RELEASE_RELEASE = ((0.06, 0.12, 0.22), (0.04, 0.06, 0.10))
_DROPOUT_LATE_BRAKE = ((0.06, 0.12, 0.20), (1.0, 0.8, 0.6))
_DROPOUT_LATE_RELEASE = ((0.06, 0.12, 0.20), (0.04, 0.06, 0.10))
_DROPOUT_BASE_BRAKE = ((0.00, 0.08, 0.24), (2.2, 1.8, 1.2))
_DROPOUT_BASE_RELEASE = ((0.00, 0.08, 0.24), (0.15, 0.22, 0.32))


@dataclass
class StoppingResult:
  """Legacy result dataclass (stopping_controller.py:21-24), re-declared here so the facade
  carries no import of the forest module (which dies at the cleanup commit)."""
  output_accel: float
  release_lock_active: bool


class StoppingControllerV2:
  def __init__(self, CP=None, params: StoppingParams = STOPPING_PARAMS):
    self.CP = CP
    self.params = params
    # MAJOR 2 (adversarial verify): the firm terminal hold (stopping_trajectory A_HOLD_FIRM) is a
    # Santa-Fe-HEV creep-torque counter, so it must be fingerprint-scoped. CP is fixed per controller,
    # so resolve the Santa-Fe fingerprint once here; the per-frame kill-switch read happens in
    # stop_reference so the flag stays unit-flippable. Other vehicles never see the firm hold.
    self._is_santa_fe_hev_2022 = getattr(CP, "carFingerprint", None) == HYUNDAI_CAR.HYUNDAI_SANTA_FE_HEV_2022
    self.tracker = StoppingTracker(params)
    # legacy telemetry seam attributes (longcontrol getattr reads)
    self.phase = int(TrajPhase.TRACK)
    self.low_speed_rollout_m = 0.0
    self._entry_soften_time_s = 0.0
    self._last_raw_should_stop = False
    self._dropout_hold_elapsed_s = 0.0

  def reset(self) -> None:
    self.tracker.reset()
    self.phase = int(TrajPhase.TRACK)
    self.low_speed_rollout_m = 0.0
    self._entry_soften_time_s = 0.0
    self._last_raw_should_stop = False
    self._dropout_hold_elapsed_s = 0.0

  def seed_command_history(self, commands: list[float]) -> None:
    self.tracker.seed_command_history(commands)

  # --- legacy dropout envelope (verbatim port, stopping_controller.py:746-810) --------------------

  def _dropout_envelope_accel(self, output_accel: float, last_output_accel: float,
                              v_ego: float, a_ego: float, stop_accel: float, dt: float) -> float | None:
    v = float(v_ego)
    micro = 0.0 < v < 0.06 and last_output_accel < -0.22 and a_ego > -0.08
    active_release = (0.06 <= v < 0.22 and last_output_accel < -0.22
                      and output_accel > (last_output_accel + 0.01) and self.tracker.rollout_m < 0.70)
    late = 0.06 <= v < 0.20 and last_output_accel < -0.32 and a_ego > -0.08
    hold = 0.0 < v < 0.24 and ((a_ego < -0.05 and output_accel > -0.16) or micro or active_release or late)
    if not hold:
      return None
    hold_floor = float(interp(v, *_DROPOUT_HOLD_FLOOR))
    if micro:
      hold_floor = min(hold_floor, float(interp(v, *_DROPOUT_MICRO_FLOOR)))
    elif active_release:
      hold_floor = min(hold_floor, float(interp(v, *_DROPOUT_ACTIVE_RELEASE_FLOOR)))
    elif late:
      hold_floor = min(hold_floor, float(interp(v, *_DROPOUT_LATE_FLOOR)))
    late_onset = 0.03 < v < 0.16 and a_ego < -0.08 and last_output_accel > -0.12
    if late_onset:
      brake_rate, release_rate = float(interp(v, *_DROPOUT_LATE_ONSET_BRAKE)), float(interp(v, *_DROPOUT_LATE_ONSET_RELEASE))
    elif micro:
      brake_rate, release_rate = float(interp(v, *_DROPOUT_MICRO_BRAKE)), float(interp(v, *_DROPOUT_MICRO_RELEASE))
    elif active_release:
      brake_rate, release_rate = float(interp(v, *_DROPOUT_ACTIVE_RELEASE_BRAKE)), float(interp(v, *_DROPOUT_ACTIVE_RELEASE_RELEASE))
    elif late:
      brake_rate, release_rate = float(interp(v, *_DROPOUT_LATE_BRAKE)), float(interp(v, *_DROPOUT_LATE_RELEASE))
    else:
      brake_rate, release_rate = float(interp(v, *_DROPOUT_BASE_BRAKE)), float(interp(v, *_DROPOUT_BASE_RELEASE))
    target = min(output_accel, hold_floor)
    guarded = min(max(target, last_output_accel - brake_rate * dt), last_output_accel + release_rate * dt)
    return min(max(guarded, stop_accel), -0.05)

  # --- the legacy seam + the one trailing kwarg ----------------------------------------------------

  def update(self, output_accel, last_output_accel, should_stop, v_ego, a_ego,
             max_expected_accel, min_expected_accel, stop_accel, dt,
             distance_to_stop_target_m=None, raw_should_stop=None,
             lead_status=False, lead_v=0.0, lead_d_rel=None,
             debug: dict | None = None, decision: StopDecision | None = None) -> StoppingResult:
    del distance_to_stop_target_m, lead_status, lead_v, lead_d_rel  # seam-compat; V2 reads the decision
    assert decision is not None, "StoppingControllerV2 requires the longcontrol-computed StopDecision (spec section 2, F2)"
    if raw_should_stop is None:
      raw_should_stop = should_stop  # legacy fallback (stopping_controller.py:682-683)
    dt = float(dt)
    if not (math.isfinite(dt) and dt > 0.0):
      dt = 0.01  # F8: a poisoned dt must not raise out of the facade (PlantModel rejects it); DT_CTRL fallback

    # verbatim stop-entry-soften arming (stopping_controller.py:123-148), seconds-based window
    if not raw_should_stop:
      self._last_raw_should_stop = False
      self._entry_soften_time_s = 0.0
    else:
      new_entry = not self._last_raw_should_stop
      self._last_raw_should_stop = True
      if new_entry:
        candidate = (0.12 < v_ego < 1.65 and a_ego > -0.60
                     and (last_output_accel > 0.02 or last_output_accel < -0.28)
                     and last_output_accel > -0.45)
        self._entry_soften_time_s = float(interp(v_ego, *ENTRY_SOFTEN_WINDOW_S_TABLE)) if candidate else 0.0
    entry_soften_active = self._entry_soften_time_s > 0.0

    if not decision.stop_request_active:
      # spec 5.5.6 dropout edge case: intent fully expired -- legacy guarded mild-braking
      # envelope for one hold period, then reset and pass through. Both early returns carry the
      # spec 5.5.5 step (5) non-finite guard: the facade never emits NaN/inf (spec 5.7).
      envelope = self._dropout_envelope_accel(output_accel, last_output_accel, v_ego, a_ego, stop_accel, dt)
      if envelope is not None and self._dropout_hold_elapsed_s < self.params.T_STOP_INTENT_HOLD_S:
        self._dropout_hold_elapsed_s += dt
        nonfinite_fallback = False
        if not math.isfinite(envelope):
          envelope, nonfinite_fallback = self._finite_fallback(last_output_accel)
        self._fill_debug(debug, decision, a_ref=envelope, dropout_hold=True, nonfinite_fallback=nonfinite_fallback)
        return StoppingResult(output_accel=envelope, release_lock_active=False)
      self.reset()
      passthrough = float(output_accel)
      nonfinite_fallback = False
      if not math.isfinite(passthrough):
        passthrough, nonfinite_fallback = self._finite_fallback(last_output_accel)
      self._fill_debug(debug, decision, a_ref=passthrough, dropout_hold=False, nonfinite_fallback=nonfinite_fallback)
      return StoppingResult(output_accel=passthrough, release_lock_active=False)
    self._dropout_hold_elapsed_s = 0.0

    ref = stop_reference(v_ego=v_ego, a_ego=a_ego,
                         target_distance_m=decision.target_distance_m,
                         settled_time_s=self.tracker.settled_time_s,
                         rollout_m=self.tracker.rollout_m, p=self.params,
                         terminal_glide_firm_hold=self._is_santa_fe_hev_2022)
    result = self.tracker.update(ref=ref, decision=decision, v_ego=v_ego, a_ego=a_ego,
                                 last_output_accel=last_output_accel,
                                 max_expected_accel=max_expected_accel,
                                 min_expected_accel=min_expected_accel,
                                 stop_accel=stop_accel, dt=dt, debug=debug)
    if entry_soften_active:
      self._entry_soften_time_s = max(self._entry_soften_time_s - dt, 0.0)

    # spec 5.5.5 step (4): final authority clip with the verbatim entry-soften exception
    # (stopping_controller.py:2555-2559)
    output_cap = -0.05
    if entry_soften_active and last_output_accel > -0.02:
      output_cap = float(interp(v_ego, self.params.STOP_ENTRY_OUTPUT_CAP_TABLE[0], self.params.STOP_ENTRY_OUTPUT_CAP_TABLE[1]))
    nonfinite_fallback = False
    u = result.output_accel
    if not math.isfinite(u):
      u, nonfinite_fallback = self._finite_fallback(last_output_accel)
    else:
      u = min(u, output_cap)   # bound-second min/max keep u when a bound is non-finite
      u = max(u, stop_accel)
      if not math.isfinite(u):
        u, nonfinite_fallback = self._finite_fallback(last_output_accel)

    # legacy telemetry seam attributes
    self.phase = int(ref.phase)
    self.low_speed_rollout_m = result.rollout_m

    self._fill_debug(debug, decision, a_ref=ref.a_ref, dropout_hold=False,
                     remaining_m=ref.remaining_m, result=result, nonfinite_fallback=nonfinite_fallback)
    return StoppingResult(output_accel=u, release_lock_active=False)

  @staticmethod
  def _finite_fallback(last_output_accel: float) -> tuple[float, bool]:
    # spec 5.5.5 step (5) / F8: the facade never emits NaN/inf. last_output_accel is the
    # previous SENT command (in range by construction); the -0.1 second resort is the legacy
    # stopping-state seed (longcontrol.py:925) for the degenerate case where even it is poisoned.
    u = float(last_output_accel)
    if not math.isfinite(u):
      u = -0.1
    return u, True

  def _fill_debug(self, debug: dict | None, decision: StopDecision, *, a_ref: float,
                  dropout_hold: bool, remaining_m: float = -1.0, result=None,
                  nonfinite_fallback: bool = False) -> None:
    """Spec section-2 telemetry contract: v2 debug dict (the shadow_* oracle keys are
    intentionally retired with the oracle, F36). Filling the dict never changes the output."""
    if debug is None:
      return
    debug["version"] = DEBUG_VERSION
    debug["phase"] = int(self.phase)
    debug["a_ref"] = float(a_ref)
    debug["disturbance"] = float(result.disturbance) if result is not None else float(self.tracker.d_hat)
    debug["rollout_m"] = float(result.rollout_m) if result is not None else float(self.tracker.rollout_m)
    debug["remaining_m"] = float(remaining_m)
    debug["release_inhibit_active"] = bool(result.release_inhibit_active) if result is not None else (self.tracker.release_inhibit_timer_s > 0.0)
    debug["recovery_i"] = float(self.tracker.recovery_i)
    debug["settled_time_s"] = float(self.tracker.settled_time_s)
    debug["source"] = int(decision.source)
    debug["triggers"] = ()  # legacy key, always empty under V2
    debug["nonfinite_fallback"] = bool(nonfinite_fallback)
    debug["dropout_hold_active"] = bool(dropout_hold)
