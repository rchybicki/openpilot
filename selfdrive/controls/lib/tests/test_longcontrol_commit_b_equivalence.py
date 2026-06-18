"""Commit B replay-equivalence hard gate (FINAL_SPEC §6 Commit B, §11 WP7).

Replays LongControl old-vs-new on the stop_scenarios.py fixtures + scripted/randomized traces and
asserts identical ``stop_request_active`` / ``state_should_stop`` / target traces AND the full
``output_accel`` trace, plus the three independent release booleans (F14) per frame.

``LegacyLongControlOracle`` is a verbatim transcription of ``LongControl.update`` at base commit
3be25f5240 (the pre-arbiter inline arbitration block :729-868, timers :888-897, dispatch
:902-1130), minus the §4.6 dead code it never executed (``prep_stopping`` is always False, the
breakpoint bookkeeping is write-only, ``CP.stoppingVbp`` is never populated so the stopping
interp bps equal STOPPING_V_BP). The oracle was validated trace-equal against the UNMODIFIED
longcontrol.py over every scenario in this file before the Commit B edit landed (WP7 report);
the predicate helpers come from the longcontrol module itself (post-Commit-B these resolve to
the arbiter's AST-pinned verbatim ports), so the oracle exercises exactly the WIRING the
arbiter consolidation replaced.

The 5 pinned holdout-route replays are the on-device/CI half of the gate (spec §6) and cannot
run in this build-free environment (no recorded routes); the fixture half lives here.
"""

import random

import pytest

from opendbc.car.hyundai.values import CAR as HYUNDAI_CAR
from openpilot.common.pid import PIDController
from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.controls.lib import longcontrol as lcm
from openpilot.selfdrive.controls.lib import stopping_flags
from openpilot.selfdrive.controls.lib.longcontrol import (
  STOPPING_ACCEL_MAX,
  STOPPING_ACCEL_MIN,
  STOPPING_V_BP,
  LongControl,
  LongCtrlState,
  apply_experimental_close_lead_accel_cap,
  apply_pid_brake_model_alignment,
  experimental_close_lead_accel_cap,
  far_stopped_lead_brake_floor,
  far_stopped_lead_crawl_accel_cap,
  far_stopped_lead_settle_accel_cap,
  force_coast_no_target_pid_brake_cap,
  force_coast_no_target_pid_brake_step,
  long_control_state_trans,
  low_speed_close_lead_accel_cap,
  low_speed_close_lead_brake_step,
  low_speed_stopped_lead_glide_accel_cap,
  pid_integrator_enabled,
  should_apply_experimental_close_lead_accel_cap,
  should_apply_force_coast_no_target_pid_brake_cap,
  should_apply_low_speed_close_lead_accel_cap,
  should_apply_stopping_planner_floor,
  stopping_planner_floor_active,
  should_apply_low_speed_stopped_lead_glide_accel_cap,
  should_apply_pid_brake_model_alignment,
  should_apply_stop_entry_handoff_soften,
  should_apply_stopping_phase_approach_decel_cap,
  should_apply_stop_target_approach_mode,
  should_apply_stop_target_carry_mode,
  should_enter_stop_target_mode,
  should_hold_low_speed_stop_target_release,
  should_hold_no_target_standstill_dropout,
  should_hold_recent_close_stopped_lead_dropout,
  should_hold_stop_target_dropout,
  should_observe_pid_stopping_shadow,
  should_release_far_stopped_lead_gap,
  stop_entry_handoff_accel_cap,
  stop_target_approach_accel_cap,
  stop_target_carry_accel_floor,
  stopping_phase_approach_decel_cap,
)
from openpilot.selfdrive.controls.lib.longcontrol import APPROACH_DECEL_CAP_RELEASE_STEP
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.stop_target_helpers import get_stopped_lead_control_target
from openpilot.selfdrive.controls.lib.stop_and_go_helpers import should_release_stop_hold_for_departing_lead
from openpilot.selfdrive.controls.lib.stopping_controller import StoppingController
from openpilot.selfdrive.controls.lib.stopping_guard import apply_low_speed_output_slew
from openpilot.selfdrive.controls.lib.tests import stop_scenarios
from openpilot.frogpilot.controls.lib.force_coast import get_force_coast_target_from_toggles

clip = lcm.clip
interp = lcm.interp


class DummyCruiseState:
  def __init__(self, standstill: bool = False) -> None:
    self.standstill = standstill


class DummyCarState:
  def __init__(self, v_ego: float, a_ego: float, brake_pressed: bool = False,
               standstill: bool = False, cruise_standstill: bool = False) -> None:
    self.vEgo = v_ego
    self.aEgo = a_ego
    self.brakePressed = brake_pressed
    self.standstill = standstill
    self.cruiseState = DummyCruiseState(standstill=cruise_standstill)


class DummyLongitudinalTuning:
  def __init__(self) -> None:
    self.kpBP = [0.0]
    self.kpV = [1.0]
    self.kiBP = [0.0]
    self.kiV = [0.0]


class DummyCarParams:
  def __init__(self, car_fingerprint=HYUNDAI_CAR.HYUNDAI_SANTA_FE_HEV_2022, starting_state=False) -> None:
    self.longitudinalTuning = DummyLongitudinalTuning()
    self.carFingerprint = car_fingerprint
    self.enableGasInterceptor = False
    self.startingState = starting_state
    self.stopAccel = -1.0


class DummyFrogPilotToggles:
  def __init__(self, human_acceleration=False) -> None:
    self.vEgoStarting = 0.1
    self.vEgoStopping = 0.2
    self.force_coast_strength = 1.0
    self.human_acceleration = human_acceleration
    self.startAccel = 1.0


class Frame:
  """One LongControl.update input frame (controlsd-faithful: float sentinels, never None)."""

  def __init__(self, v_ego=0.0, a_ego=0.0, a_target=0.0, should_stop=False, distance=-1.0,
               lead_status=False, lead_v=0.0, lead_d_rel=0.0, brake_pressed=False,
               standstill=False, cruise_standstill=False, force_coast=False, active=True,
               experimental_mode=False):
    self.v_ego = v_ego
    self.a_ego = a_ego
    self.a_target = a_target
    self.should_stop = should_stop
    self.distance = distance
    self.lead_status = lead_status
    self.lead_v = lead_v
    self.lead_d_rel = lead_d_rel
    self.brake_pressed = brake_pressed
    self.standstill = standstill
    self.cruise_standstill = cruise_standstill
    self.force_coast = force_coast
    self.active = active
    self.experimental_mode = experimental_mode

  def car_state(self) -> DummyCarState:
    return DummyCarState(v_ego=self.v_ego, a_ego=self.a_ego, brake_pressed=self.brake_pressed,
                         standstill=self.standstill, cruise_standstill=self.cruise_standstill)


class LegacyLongControlOracle:
  """Verbatim transcription of LongControl.update @ 3be25f5240 (see module docstring)."""

  def __init__(self, CP, accel_limits=(-3.5, 2.0)):
    self.CP = CP
    self.long_control_state = LongCtrlState.off
    self.pid = PIDController((CP.longitudinalTuning.kpBP, CP.longitudinalTuning.kpV),
                             (CP.longitudinalTuning.kiBP, CP.longitudinalTuning.kiV),
                             rate=1 / DT_CTRL)
    self.accel_limits = accel_limits
    self.last_output_accel = 0.0
    self.stopping_controller = StoppingController()
    self.time_since_standstill_s = 10.0
    self.time_since_stop_intent_s = 10.0
    self.last_distance_to_stop_target_m = None
    self.stopping_shadow_frame = 0

  def reset(self):
    self.pid.reset()
    self.stopping_controller.reset()
    self.time_since_standstill_s = 10.0
    self.time_since_stop_intent_s = 10.0
    self.last_distance_to_stop_target_m = None
    self.stopping_shadow_frame = 0

  def _new_stopping_shadow_debug_if_due(self, observer_scope):
    # verbatim :618-624 (STOPPING_SHADOW_LOGGING_ENABLED is True)
    self.stopping_shadow_frame += 1
    if self.stopping_shadow_frame % 10 != 0:
      return None
    return {"shadow_observer_scope": observer_scope}

  def update(self, f: Frame, frogpilot_toggles) -> dict:
    CS = f.car_state()
    a_target = f.a_target
    should_stop = f.should_stop
    distance_to_stop_target_m = f.distance
    lead_status, lead_v, lead_d_rel, force_coast = f.lead_status, f.lead_v, f.lead_d_rel, f.force_coast
    accel_limits = self.accel_limits

    self.pid.neg_limit = accel_limits[0]
    self.pid.pos_limit = accel_limits[1]
    human_acceleration_active = frogpilot_toggles.human_acceleration and not f.experimental_mode

    output_accel = self.last_output_accel
    prev_distance_to_stop_target_m = self.last_distance_to_stop_target_m
    stopped_lead_control_target_m = (
      get_stopped_lead_control_target(v_ego=CS.vEgo, lead_v=float(lead_v), lead_d_rel=float(lead_d_rel))
      if bool(lead_status) and should_apply_low_speed_stopped_lead_glide_accel_cap(self.CP)
      else None
    )
    control_distance_to_stop_target_m = distance_to_stop_target_m
    if stopped_lead_control_target_m is not None and (
      control_distance_to_stop_target_m is None
      or control_distance_to_stop_target_m <= 0.0
      or stopped_lead_control_target_m < control_distance_to_stop_target_m
    ):
      control_distance_to_stop_target_m = stopped_lead_control_target_m

    release_lock_active = False
    max_expected_accel = interp(CS.vEgo, STOPPING_V_BP, STOPPING_ACCEL_MAX)
    stopped_lead_control_stop_active = stopped_lead_control_target_m is not None
    stop_target_request = should_enter_stop_target_mode(CS.vEgo, a_target, control_distance_to_stop_target_m)
    stop_request_active = should_stop or stop_target_request or stopped_lead_control_stop_active
    stop_target_approach_active = (
      not stop_request_active
      and should_apply_stop_target_approach_mode(CS.vEgo, a_target, control_distance_to_stop_target_m)
    )
    stop_target_carry_active = (
      not stop_request_active
      and not stop_target_approach_active
      and should_apply_stop_target_carry_mode(CS.vEgo, a_target, control_distance_to_stop_target_m)
    )
    standstill = bool(getattr(CS, "standstill", False)) or bool(CS.cruiseState.standstill)
    departing_lead_ready = should_release_stop_hold_for_departing_lead(
      human_acceleration=bool(frogpilot_toggles.human_acceleration),
      output_should_stop=True,
      force_coast=bool(force_coast),
      standstill=standstill,
      v_ego=float(CS.vEgo),
      v_ego_starting=float(frogpilot_toggles.vEgoStarting),
      lead_status=bool(lead_status),
      lead_v=float(lead_v),
      lead_d_rel=float(lead_d_rel),
    )
    departing_lead_release = bool(should_stop) and departing_lead_ready
    if departing_lead_release:
      stop_request_active = False
      stop_target_approach_active = False
    far_stopped_lead_gap_release = (
      should_apply_low_speed_stopped_lead_glide_accel_cap(self.CP)
      and not force_coast
      and should_release_far_stopped_lead_gap(
        v_ego=CS.vEgo,
        lead_status=bool(lead_status),
        lead_v=float(lead_v),
        lead_d_rel=float(lead_d_rel),
        distance_to_stop_target_m=control_distance_to_stop_target_m,
      )
    )
    if far_stopped_lead_gap_release:
      stop_request_active = False
      stop_target_approach_active = False
      stop_target_carry_active = False
    close_stopped_lead_dropout_hold_active = (
      should_apply_low_speed_stopped_lead_glide_accel_cap(self.CP)
      and not departing_lead_release
      and not far_stopped_lead_gap_release
      and should_hold_recent_close_stopped_lead_dropout(
        v_ego=CS.vEgo,
        v_ego_starting=float(frogpilot_toggles.vEgoStarting),
        standstill=standstill,
        time_since_standstill_s=self.time_since_standstill_s,
        lead_status=bool(lead_status),
        lead_v=float(lead_v),
        lead_d_rel=float(lead_d_rel),
        distance_to_stop_target_m=control_distance_to_stop_target_m,
        force_coast=bool(force_coast),
      )
    )
    if close_stopped_lead_dropout_hold_active:
      stop_request_active = True
      stop_target_approach_active = False
      stop_target_carry_active = False
    stop_target_release_hold_active = (
      not departing_lead_release
      and not far_stopped_lead_gap_release
      and not close_stopped_lead_dropout_hold_active
      and should_hold_low_speed_stop_target_release(
        v_ego=CS.vEgo,
        a_target=a_target,
        distance_to_stop_target_m=control_distance_to_stop_target_m,
        last_distance_to_stop_target_m=prev_distance_to_stop_target_m,
        last_output_accel=self.last_output_accel,
        time_since_stop_intent_s=self.time_since_stop_intent_s,
      )
    )
    if stop_target_release_hold_active:
      stop_request_active = True
      stop_target_approach_active = False
      stop_target_carry_active = False
    force_coast_standstill_hold = bool(force_coast) and standstill
    state_should_stop = (
      should_stop
      or stopped_lead_control_stop_active
      or close_stopped_lead_dropout_hold_active
      or stop_target_release_hold_active
      or force_coast_standstill_hold
    ) and not departing_lead_release and not far_stopped_lead_gap_release
    new_control_state = long_control_state_trans(self.CP, f.active, self.long_control_state, CS.vEgo,
                                                 state_should_stop, CS.brakePressed,
                                                 CS.cruiseState.standstill, frogpilot_toggles,
                                                 a_target=a_target,
                                                 distance_to_stop_target_m=control_distance_to_stop_target_m)
    state_dropout_hold = False
    if (
      self.long_control_state == LongCtrlState.stopping
      and not departing_lead_release
      and not far_stopped_lead_gap_release
    ):
      state_dropout_hold = (
        should_hold_stop_target_dropout(
          v_ego=CS.vEgo,
          a_target=a_target,
          distance_to_stop_target_m=control_distance_to_stop_target_m,
          last_distance_to_stop_target_m=prev_distance_to_stop_target_m,
          last_output_accel=self.last_output_accel,
          time_since_stop_intent_s=self.time_since_stop_intent_s,
        )
        or should_hold_no_target_standstill_dropout(
          v_ego=CS.vEgo,
          standstill=standstill,
          force_coast=bool(force_coast),
          a_target=a_target,
          distance_to_stop_target_m=control_distance_to_stop_target_m,
          last_output_accel=self.last_output_accel,
          time_since_stop_intent_s=self.time_since_stop_intent_s,
        )
      )
    if self.long_control_state == LongCtrlState.stopping and new_control_state != LongCtrlState.stopping and state_dropout_hold:
      new_control_state = LongCtrlState.stopping
    entered_stopping = self.long_control_state != LongCtrlState.stopping and new_control_state == LongCtrlState.stopping
    self.long_control_state = new_control_state

    if standstill:
      self.time_since_standstill_s = 0.0
    else:
      self.time_since_standstill_s = min(self.time_since_standstill_s + DT_CTRL, 10.0)

    stop_intent_active = (stop_request_active or stop_target_approach_active or stop_target_carry_active
                          or (self.long_control_state == LongCtrlState.stopping))
    if stop_intent_active:
      self.time_since_stop_intent_s = 0.0
    else:
      self.time_since_stop_intent_s = min(self.time_since_stop_intent_s + DT_CTRL, 10.0)

    standstill_recent = self.time_since_standstill_s < 0.5
    stop_intent_recent = self.time_since_stop_intent_s < 1.0

    if self.long_control_state == LongCtrlState.off or not stop_intent_active:
      self.stopping_controller.reset()

    stopping_shadow_debug = None

    if self.long_control_state == LongCtrlState.off:
      self.reset()
      output_accel = 0.

    elif self.long_control_state == LongCtrlState.stopping:
      handoff_soften_cap = None
      output_accel = min(output_accel, -0.1)
      if entered_stopping and should_apply_stop_entry_handoff_soften(CS.vEgo, CS.aEgo, a_target, self.last_output_accel, control_distance_to_stop_target_m):
        handoff_soften_cap = stop_entry_handoff_accel_cap(CS.vEgo, control_distance_to_stop_target_m)
        output_accel = max(output_accel, handoff_soften_cap)
      # CP.stoppingVbp is never populated -> stopping bps == STOPPING_V_BP (spec §4.6)
      max_expected_accel = interp(CS.vEgo, STOPPING_V_BP, STOPPING_ACCEL_MAX)
      min_expected_accel = interp(CS.vEgo, STOPPING_V_BP, STOPPING_ACCEL_MIN)

      stopping_shadow_debug = self._new_stopping_shadow_debug_if_due("stopping")

      stop_result = self.stopping_controller.update(
        output_accel=output_accel,
        last_output_accel=max(self.last_output_accel, handoff_soften_cap) if handoff_soften_cap is not None else self.last_output_accel,
        should_stop=stop_request_active,
        v_ego=CS.vEgo,
        a_ego=CS.aEgo,
        max_expected_accel=max_expected_accel,
        min_expected_accel=min_expected_accel,
        distance_to_stop_target_m=control_distance_to_stop_target_m,
        raw_should_stop=should_stop,
        stop_accel=self.CP.stopAccel,
        dt=DT_CTRL,
        lead_status=lead_status,
        lead_v=lead_v,
        lead_d_rel=lead_d_rel,
        debug=stopping_shadow_debug,
      )
      output_accel = stop_result.output_accel
      release_lock_active = stop_result.release_lock_active
      if should_apply_low_speed_close_lead_accel_cap(self.CP) and lead_status:
        close_lead_cap = low_speed_close_lead_accel_cap(CS.vEgo, lead_v, lead_d_rel)
        if close_lead_cap is not None and output_accel > close_lead_cap:
          close_lead_brake_step = low_speed_close_lead_brake_step(CS.vEgo, lead_d_rel)
          output_accel = max(close_lead_cap, output_accel - close_lead_brake_step)
      # mirror the stopping-phase planner-aTarget floor (longcontrol.py, incident 0000173c seg24) so
      # this oracle stays a verbatim transcription of LongControl.update -- the Commit B gate then
      # keeps asserting the ARBITER's per-frame equivalence, not regressing on a command-cap addition.
      if (
        lcm.STOPPING_PLANNER_FLOOR_ENABLED
        and should_apply_stopping_planner_floor(self.CP)
        and stopping_planner_floor_active(CS.vEgo, lead_status, lead_v, lead_d_rel, a_target, output_accel)
      ):
        output_accel = min(output_accel, a_target)

    elif self.long_control_state == LongCtrlState.starting:
      output_accel = (a_target if human_acceleration_active else frogpilot_toggles.startAccel)
      if human_acceleration_active and departing_lead_ready:
        lead_departure_speed = max(float(lead_v) - float(CS.vEgo), 0.0)
        departing_lead_accel_floor = interp(lead_departure_speed, [0.60, 1.20, 2.00, 3.00], [0.12, 0.22, 0.35, 0.45])
        output_accel = max(output_accel, min(float(frogpilot_toggles.startAccel), departing_lead_accel_floor))
      self.reset()

    else:  # LongCtrlState.pid
      error = a_target - CS.aEgo
      freeze_integrator = stop_target_approach_active or stop_target_carry_active
      output_accel = self.pid.update(error, speed=CS.vEgo,
                                     feedforward=a_target,
                                     freeze_integrator=freeze_integrator)
      integrator_enabled = pid_integrator_enabled(self.pid)
      if not integrator_enabled:
        self.pid.i = 0.0
      if stop_target_approach_active:
        output_accel = min(output_accel, stop_target_approach_accel_cap(CS.vEgo, control_distance_to_stop_target_m))
      if stop_target_carry_active:
        output_accel = max(output_accel, stop_target_carry_accel_floor(CS.vEgo, control_distance_to_stop_target_m))

    if self.long_control_state != LongCtrlState.off:
      allow_fast_release = (
        not force_coast
        and not stop_request_active and not stop_target_approach_active
        and self.long_control_state in (LongCtrlState.pid, LongCtrlState.starting)
        and a_target > 0.2
        and CS.vEgo > 0.12
      )
      if departing_lead_release and not force_coast:
        allow_fast_release = True
      if departing_lead_ready and self.long_control_state == LongCtrlState.starting and not force_coast:
        allow_fast_release = True
      if stop_intent_recent and not standstill_recent:
        allow_fast_release = False
      apply_global_low_speed_slew = not (self.long_control_state == LongCtrlState.stopping and stop_request_active)
      if apply_global_low_speed_slew:
        output_accel = apply_low_speed_output_slew(
          output_accel=output_accel,
          last_output_accel=self.last_output_accel,
          should_stop=(stop_request_active or stop_target_approach_active or stop_target_carry_active),
          v_ego=CS.vEgo,
          a_ego=CS.aEgo,
          max_expected_accel=max_expected_accel,
          allow_fast_release=allow_fast_release,
          release_lock_active=release_lock_active,
        )
      if far_stopped_lead_gap_release:
        output_accel = min(output_accel, far_stopped_lead_crawl_accel_cap(CS.vEgo, lead_d_rel))
        if should_stop or stop_intent_recent:
          settle_cap = far_stopped_lead_settle_accel_cap(CS.vEgo, lead_d_rel, control_distance_to_stop_target_m)
          if settle_cap is not None:
            settle_release_step = interp(CS.vEgo, [0.03, 0.20, 0.55], [0.006, 0.008, 0.011])
            output_accel = min(output_accel, settle_cap, self.last_output_accel + settle_release_step)
        far_lead_brake_floor = far_stopped_lead_brake_floor(CS.vEgo, lead_d_rel)
        if output_accel < far_lead_brake_floor:
          far_lead_release_step = interp(CS.vEgo, [0.00, 0.20, 0.55], [0.028, 0.024, 0.018])
          output_accel = min(far_lead_brake_floor, max(output_accel, self.last_output_accel + far_lead_release_step))

      stopped_lead_glide_cap = (
        low_speed_stopped_lead_glide_accel_cap(CS.vEgo, lead_v, lead_d_rel, control_distance_to_stop_target_m)
        if (
          should_apply_low_speed_stopped_lead_glide_accel_cap(self.CP)
          and lead_status
          and (stop_request_active or stop_target_approach_active or stop_target_carry_active or self.long_control_state == LongCtrlState.stopping)
        )
        else None
      )
      if stopped_lead_glide_cap is not None and output_accel > stopped_lead_glide_cap:
        stopped_lead_brake_step = interp(CS.vEgo, [0.02, 0.20, 0.35, 0.65, 0.95, 1.25], [0.004, 0.004, 0.004, 0.006, 0.008, 0.010])
        output_accel = max(stopped_lead_glide_cap, min(output_accel, self.last_output_accel) - stopped_lead_brake_step)

      if (
        should_apply_pid_brake_model_alignment(self.CP)
        and self.long_control_state == LongCtrlState.pid
        and not stop_request_active
        and not stop_target_approach_active
        and not far_stopped_lead_gap_release
      ):
        aligned_output = apply_pid_brake_model_alignment(output_accel, a_target, CS.aEgo, CS.vEgo)
        if aligned_output > output_accel:
          if integrator_enabled:
            self.pid.i = max(self.pid.i, aligned_output - (self.pid.p + self.pid.d + self.pid.f))
          output_accel = aligned_output
      if (
        should_apply_force_coast_no_target_pid_brake_cap(self.CP)
        and force_coast
        and self.long_control_state == LongCtrlState.pid
        and not stop_request_active
        and not stop_target_approach_active
        and not stop_target_carry_active
        and not lead_status
        and (control_distance_to_stop_target_m is None or control_distance_to_stop_target_m < 0.0)
      ):
        force_coast_target_accel = get_force_coast_target_from_toggles(CS.vEgo, frogpilot_toggles)
        if output_accel > force_coast_target_accel:
          force_coast_brake_step = force_coast_no_target_pid_brake_step(CS.vEgo)
          output_accel = max(force_coast_target_accel, min(output_accel, self.last_output_accel) - force_coast_brake_step)
          if integrator_enabled:
            self.pid.i = min(self.pid.i, output_accel - (self.pid.p + self.pid.d + self.pid.f))
        force_coast_brake_cap = force_coast_no_target_pid_brake_cap(CS.vEgo, force_coast_target_accel)
        if output_accel < force_coast_brake_cap:
          output_accel = force_coast_brake_cap
          if integrator_enabled:
            self.pid.i = max(self.pid.i, output_accel - (self.pid.p + self.pid.d + self.pid.f))
      if (
        should_apply_experimental_close_lead_accel_cap(self.CP, f.experimental_mode)
        and self.long_control_state == LongCtrlState.pid
        and not stop_request_active
        and not stop_target_approach_active
        and not stop_target_carry_active
        and lead_status
      ):
        close_lead_cap = experimental_close_lead_accel_cap(CS.vEgo, lead_v, lead_d_rel)
        if close_lead_cap is not None and output_accel > close_lead_cap:
          output_accel = apply_experimental_close_lead_accel_cap(output_accel, close_lead_cap)
          if integrator_enabled:
            self.pid.i = min(self.pid.i, output_accel - (self.pid.p + self.pid.d + self.pid.f))

      # P1 cranked-comfort cap (2026-06-13) -- mirrors longcontrol.update so this equivalence
      # oracle keeps verifying the arbiter consolidation, not freezing out the intentional cap.
      approach_decel_cap_context = (
        stop_request_active or stop_target_approach_active
        or self.long_control_state == LongCtrlState.stopping
        or (self.long_control_state == LongCtrlState.pid and lead_status)
      )
      if lcm.APPROACH_DECEL_CAP_ENABLED and should_apply_stopping_phase_approach_decel_cap(self.CP) and approach_decel_cap_context:
        approach_floor = stopping_phase_approach_decel_cap(CS.vEgo, lead_status, lead_v, lead_d_rel)
        if approach_floor is not None and output_accel < approach_floor:
          target_release = min(approach_floor, self.last_output_accel + APPROACH_DECEL_CAP_RELEASE_STEP)
          output_accel = max(output_accel, target_release)
          if self.long_control_state == LongCtrlState.pid and integrator_enabled:
            self.pid.i = max(self.pid.i, output_accel - (self.pid.p + self.pid.d + self.pid.f))

    if force_coast and standstill:
      output_accel = min(output_accel, 0.0)

    # shadow debug-dict cadence bookkeeping (:1089-1119) -- the dict content is telemetry-only,
    # but the frame counter feeds the stopping-controller debug cadence above, so it must match.
    if stopping_shadow_debug is None and self.long_control_state == LongCtrlState.pid:
      pid_shadow_active = should_observe_pid_stopping_shadow(
        v_ego=CS.vEgo,
        a_target=a_target,
        output_accel=output_accel,
        distance_to_stop_target_m=control_distance_to_stop_target_m,
        force_coast=force_coast,
        lead_status=lead_status,
        lead_v=lead_v,
        lead_d_rel=lead_d_rel,
        stop_request_active=stop_request_active,
        stop_target_approach_active=stop_target_approach_active,
        stop_target_carry_active=stop_target_carry_active,
      )
      if pid_shadow_active:
        self._new_stopping_shadow_debug_if_due("pid_stop_intent")
      else:
        self.stopping_shadow_frame = 0
    elif self.long_control_state not in (LongCtrlState.stopping,):
      self.stopping_shadow_frame = 0

    self.last_distance_to_stop_target_m = (
      float(control_distance_to_stop_target_m)
      if control_distance_to_stop_target_m is not None and control_distance_to_stop_target_m > 0.0
      else None
    )
    self.last_output_accel = clip(output_accel, accel_limits[0], accel_limits[1])

    return {
      "output_accel": float(self.last_output_accel),
      "long_control_state": self.long_control_state,
      "stop_request_active": bool(stop_request_active),
      "state_should_stop": bool(state_should_stop),
      "approach": bool(stop_target_approach_active),
      "carry": bool(stop_target_carry_active),
      "departing_lead_release": bool(departing_lead_release),
      "departing_lead_ready": bool(departing_lead_ready),
      "far_stopped_lead_release": bool(far_stopped_lead_gap_release),
      "legacy_forced": bool(close_stopped_lead_dropout_hold_active or stop_target_release_hold_active or force_coast_standstill_hold),
      "target_distance_m": (
        float(control_distance_to_stop_target_m)
        if control_distance_to_stop_target_m is not None and control_distance_to_stop_target_m > 0.0
        else -1.0
      ),
      "state_dropout_hold": bool(state_dropout_hold),
    }


# --- scenario decks --------------------------------------------------------------------------------


def frames_from_fixture(samples) -> list[Frame]:
  frames = []
  for s in samples:
    lead_d = s.lead_d_rel_m if s.lead_d_rel_m is not None else 0.0
    frames.append(Frame(
      v_ego=s.v_ego,
      a_ego=s.a_ego,
      a_target=s.accel_cmd if s.accel_cmd is not None else -0.2,
      should_stop=bool(s.should_stop),
      distance=s.distance_to_stop_target_m if s.distance_to_stop_target_m is not None else -1.0,
      lead_status=bool(s.lead_status),
      lead_v=s.lead_v,
      lead_d_rel=lead_d,
      standstill=s.v_ego < 0.05,
    ))
  return frames


def fixture_decks() -> list[tuple[str, list[Frame]]]:
  return [(name, frames_from_fixture(samples)) for name, samples in sorted(stop_scenarios.SCENARIOS.items())]


def random_frames(seed: int, n: int) -> list[Frame]:
  rng = random.Random(seed)
  v = rng.uniform(0.0, 2.0)
  a_target = -0.3
  target = -1.0
  lead = False
  lead_v = 0.0
  lead_d = 6.0
  should = False
  force_coast = False
  frames = []
  for _ in range(n):
    if rng.random() < 0.08:
      should = not should
    if rng.random() < 0.05:
      lead = not lead
    if rng.random() < 0.04:
      target = rng.choice([-1.0, rng.uniform(0.05, 4.5)])
    if rng.random() < 0.03:
      force_coast = not force_coast
    a_target += rng.uniform(-0.08, 0.08)
    a_target = max(-1.5, min(1.0, a_target))
    v += rng.uniform(-0.06, 0.05)
    v = max(0.0, min(2.5, v))
    if lead:
      lead_v = max(0.0, lead_v + rng.uniform(-0.15, 0.15))
      lead_d = max(0.3, min(12.0, lead_d + rng.uniform(-0.5, 0.5)))
    frames.append(Frame(
      v_ego=v,
      a_ego=rng.uniform(-1.2, 0.4),
      a_target=a_target,
      should_stop=should,
      distance=target,
      lead_status=lead,
      lead_v=lead_v,
      lead_d_rel=lead_d if lead else 0.0,
      brake_pressed=rng.random() < 0.01,
      standstill=v < 0.01 or rng.random() < 0.02,
      cruise_standstill=rng.random() < 0.01,
      force_coast=force_coast,
      active=rng.random() > 0.02,
      experimental_mode=rng.random() < 0.05,
    ))
  return frames


def random_decks() -> list[tuple[str, list[Frame]]]:
  return [(f"random_{seed}", random_frames(seed, 600)) for seed in range(6)]


def all_decks() -> list[tuple[str, list[Frame]]]:
  return fixture_decks() + random_decks()


# --- the gate --------------------------------------------------------------------------------------


@pytest.fixture(autouse=True)
def _pin_legacy_dispatch(monkeypatch):
  # Commit-B equivalence validates the LEGACY (revert-path) arbiter == the verbatim legacy oracle.
  # USE_STOPPING_V2 was FLIPPED TRUE 2026-06-18 (V2 live), so pin it False here to keep this gate
  # validating the legacy path that revert restores. test_v2_integrated_path_smoke_all_decks re-sets
  # it True itself (its monkeypatch runs after this fixture and wins for that test).
  monkeypatch.setattr(lcm, "USE_STOPPING_V2", False)


def run_commit_b_gate(frames, cp=None, toggles=None, isd: float = 0.0):
  """Drives the (post-Commit-B) LongControl and the verbatim legacy oracle in lockstep.

  Asserts per frame: identical long_control_state + output_accel (the spec's hard-gate pair) and
  the StopDecision fields against the legacy inline mutation chain (the three F14 release
  booleans, stop_request/state_should_stop, target, legacy_forced, state_dropout_hold).
  """
  cp = cp or DummyCarParams()
  toggles = toggles or DummyFrogPilotToggles()
  accel_limits = (-3.5, 2.0)
  oracle = LegacyLongControlOracle(cp, accel_limits=accel_limits)
  lc = LongControl(cp)

  captured: dict[str, object] = {}
  real_update = lc.arbiter.update

  def spy(**kwargs):
    decision = real_update(**kwargs)
    captured["decision"] = decision
    return decision

  lc.arbiter.update = spy

  for i, f in enumerate(frames):
    exp = oracle.update(f, toggles)
    got = lc.update(
      active=f.active,
      CS=f.car_state(),
      a_target=f.a_target,
      should_stop=f.should_stop,
      distance_to_stop_target_m=f.distance,
      accel_limits=accel_limits,
      frogpilot_toggles=toggles,
      experimental_mode=f.experimental_mode,
      lead_status=f.lead_status,
      lead_v=f.lead_v,
      lead_d_rel=f.lead_d_rel,
      force_coast=f.force_coast,
      increased_stopped_distance=isd,
    )
    d = captured["decision"]
    ctx = f"frame {i}: v={f.v_ego:.3f} should_stop={f.should_stop} target={f.distance} lead={f.lead_status}/{f.lead_v:.2f}/{f.lead_d_rel:.2f}"
    assert lc.long_control_state == exp["long_control_state"], ctx
    assert got == exp["output_accel"], f"{ctx}: output {got!r} != {exp['output_accel']!r}"
    assert d.stop_request_active == exp["stop_request_active"], ctx
    assert d.state_should_stop == exp["state_should_stop"], ctx
    assert d.approach_cap_active == exp["approach"], ctx
    assert d.carry_floor_active == exp["carry"], ctx
    assert d.departing_lead_release == exp["departing_lead_release"], ctx
    assert d.departing_lead_ready == exp["departing_lead_ready"], ctx
    assert d.far_stopped_lead_release == exp["far_stopped_lead_release"], ctx
    assert d.legacy_forced == exp["legacy_forced"], ctx
    assert d.target_distance_m == exp["target_distance_m"], ctx
    assert d.state_dropout_hold == exp["state_dropout_hold"], ctx


@pytest.mark.parametrize("name,frames", all_decks())
def test_commit_b_equivalence_default_config(name, frames):
  run_commit_b_gate(frames)


@pytest.mark.parametrize("seed", [0, 3])
def test_commit_b_equivalence_human_acceleration(seed):
  run_commit_b_gate(random_frames(seed, 600), toggles=DummyFrogPilotToggles(human_acceleration=True))


@pytest.mark.parametrize("seed", [1, 4])
def test_commit_b_equivalence_starting_state(seed):
  run_commit_b_gate(random_frames(seed, 600), cp=DummyCarParams(starting_state=True))


@pytest.mark.parametrize("seed", [2])
def test_commit_b_equivalence_non_santa_fe(seed):
  run_commit_b_gate(random_frames(seed, 600), cp=DummyCarParams(car_fingerprint=HYUNDAI_CAR.HYUNDAI_ELANTRA_2021))


def test_commit_b_isd_passthrough_while_flag_dark(monkeypatch):
  # §4.2.4: lead_d_rel_eff is a passthrough while PUBLISH_TRUE_LEAD_DISTANCE is False -- a nonzero
  # IncreasedStoppedDistance must not change a single frame (the flag-off no-op proof).
  # The shipped default flipped to True on 2026-06-10 (device ISD == 0.0, rollout plan stage 0);
  # the flag-off proof stays load-bearing for the revert path, so pin it via monkeypatch.
  monkeypatch.setattr(stopping_flags, "PUBLISH_TRUE_LEAD_DISTANCE", False)
  for _, frames in fixture_decks()[:6] + random_decks()[:2]:
    run_commit_b_gate(frames, isd=3.0)


# --- name-alias grep guard (spec §6 Commit B checklist, F24) ---------------------------------------


def test_tool_and_test_imported_longcontrol_names_still_resolve():
  import ast
  import pathlib

  repo_root = pathlib.Path(lcm.__file__).resolve().parents[3]
  consumers = sorted((repo_root / "tools" / "stopping").glob("*.py"))
  consumers += [repo_root / "selfdrive" / "controls" / "lib" / "tests" / "test_longcontrol_fast_release.py",
                repo_root / "selfdrive" / "controls" / "tests" / "test_longcontrol.py"]
  imported_names = set()
  for path in consumers:
    tree = ast.parse(path.read_text())
    for node in ast.walk(tree):
      if isinstance(node, ast.ImportFrom) and node.module and node.module.endswith("controls.lib.longcontrol"):
        imported_names.update(alias.name for alias in node.names)
  assert imported_names, "expected at least the check_harsh_stops_model.py imports"
  missing = [name for name in sorted(imported_names) if not hasattr(lcm, name)]
  assert not missing, f"longcontrol names imported by kept tools/tests no longer resolve: {missing}"


# --- integrated LongControl+V2 dark-path smoke (Commit C; the full §7.6 integrated sim_replay -----
# gate on the dropout-hold fixtures is WP6's similarity_gate / Commit D precondition) --------------


def test_v2_integrated_path_smoke_all_decks(monkeypatch):
  import math

  monkeypatch.setattr(lcm, "USE_STOPPING_V2", True)
  toggles = DummyFrogPilotToggles()
  accel_limits = (-3.5, 2.0)
  for name, frames in all_decks():
    lc = LongControl(DummyCarParams())
    for i, f in enumerate(frames):
      out = lc.update(
        active=f.active,
        CS=f.car_state(),
        a_target=f.a_target,
        should_stop=f.should_stop,
        distance_to_stop_target_m=f.distance,
        accel_limits=accel_limits,
        frogpilot_toggles=toggles,
        experimental_mode=f.experimental_mode,
        lead_status=f.lead_status,
        lead_v=f.lead_v,
        lead_d_rel=f.lead_d_rel,
        force_coast=f.force_coast,
      )
      assert math.isfinite(out), f"{name} frame {i}: non-finite output under V2"
      assert accel_limits[0] <= out <= accel_limits[1], f"{name} frame {i}: output outside accel limits"
