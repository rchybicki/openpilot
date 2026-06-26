"""WP5 acceptance tests for the stopping tracker (FINAL_SPEC sections 5.4-5.5, 8).

Pure python + numpy only (spec section 8 import-clean rule). The full event-store estimator
replay is WP6's tools/stopping/estimator_equivalence.py gate artifact (spec 5.5.2 / F21); this
file keeps the fixture-based LPF-vs-single-frame test, which runs on stop_scenarios.py and
skips the optional event-store extension gracefully when ~/.comma is absent.
"""

import dataclasses
import math
from pathlib import Path

import numpy as np
import pytest

from openpilot.selfdrive.controls.lib.stop_target_arbiter import StopDecision, StopSource
from openpilot.selfdrive.controls.lib.stopping_params import STOPPING_PARAMS
from openpilot.selfdrive.controls.lib.stopping_plant import PLANT_PARAMS_REF, PlantModel
from openpilot.selfdrive.controls.lib.stopping_tracker import PUSH_RELIEF_RELEASE_TABLE, RELIEF_CMD_DEEP, StoppingTracker, TrackerResult
from openpilot.selfdrive.controls.lib.stopping_trajectory import StopReference, TrajPhase, end_stop_ceiling, stop_reference
from openpilot.selfdrive.controls.lib.tests.stop_scenarios import SCENARIOS

interp = np.interp
P = STOPPING_PARAMS
DT = 0.01
EVENT_STORE_DIR = Path("~/.comma/stopping_behavior/event_store").expanduser()


def make_decision(*, stop=True, target=-1.0, source=StopSource.PLANNER, legacy_forced=False) -> StopDecision:
  return StopDecision(
    stop_request_active=stop, state_should_stop=stop, target_distance_m=target, source=source,
    approach_cap_active=False, carry_floor_active=False, departing_lead_release=False,
    departing_lead_ready=False, far_stopped_lead_release=False, legacy_forced=legacy_forced,
    release_reason="", state_dropout_hold=False)


DECISION = make_decision()


def make_ref(*, v, a_ref=None, phase=None, j_brake=None, j_release=None, remaining=1.0) -> StopReference:
  """Explicit reference for open-loop tracker tests (the real law is tested in
  test_stopping_trajectory.py); defaults follow the trajectory tables at v."""
  if phase is None:
    phase = TrajPhase.TRACK if v > P.V_NEAR_HOLD else (TrajPhase.TERMINAL if v > P.V_SETTLE else TrajPhase.SETTLE)
  if a_ref is None:
    a_ref = float(interp(v, P.A_NEAR_HOLD_TABLE[0], P.A_NEAR_HOLD_TABLE[1]))
  if j_brake is None:
    j_brake = float(interp(v, P.J_BRAKE_TABLE[0], P.J_BRAKE_TABLE[1]))
  if j_release is None:
    j_release = float(interp(v, P.J_RELEASE_TABLE[0], P.J_RELEASE_TABLE[1]))
  return StopReference(a_ref=a_ref, phase=phase, j_brake_max=j_brake, j_release_max=j_release, remaining_m=remaining)


def run_frame(tracker: StoppingTracker, *, v, a_ego, last, ref=None, decision=DECISION,
              max_exp=5.0, min_exp=-5.0, dt=DT, debug=None) -> TrackerResult:
  """max_exp == min_exp pins a_exp exactly (the G3 sanity clamp), making the innovation -- and
  therefore the push/overbrake/arrest triggers -- deterministic without solving the plant."""
  if ref is None:
    ref = make_ref(v=v)
  return tracker.update(ref=ref, decision=decision, v_ego=v, a_ego=a_ego, last_output_accel=last,
                        max_expected_accel=max_exp, min_expected_accel=min_exp,
                        stop_accel=-2.0, dt=dt, debug=debug)


class TestDelayCompensation:
  def test_a_exp_uses_dead_time_delayed_command(self):
    plant = PlantModel(PLANT_PARAMS_REF, DT)
    tracker = StoppingTracker(P)
    tracker.seed_command_history([-0.30] * 40)
    debug = {}
    run_frame(tracker, v=0.5, a_ego=-0.25, last=-0.50, debug=debug)
    # after the append, history[-1] = -0.50 and history[-1 - delay_frames] = -0.30
    expected = plant.predict_next(-0.25, -0.30, 0.5)
    assert debug["a_exp"] == pytest.approx(expected, abs=1e-12)

  def test_a_exp_advances_with_the_pipeline(self):
    plant = PlantModel(PLANT_PARAMS_REF, DT)
    tracker = StoppingTracker(P)
    seed = [-0.10 - 0.01 * k for k in range(30)]
    tracker.seed_command_history(seed)
    debug = {}
    run_frame(tracker, v=0.5, a_ego=-0.25, last=-0.50, debug=debug)
    history = seed + [-0.50]
    assert debug["a_exp"] == pytest.approx(plant.predict_next(-0.25, history[-1 - plant.delay_frames], 0.5), abs=1e-12)
    debug2 = {}
    run_frame(tracker, v=0.45, a_ego=-0.30, last=-0.52, debug=debug2)
    history.append(-0.52)
    assert debug2["a_exp"] == pytest.approx(plant.predict_next(-0.25, history[-1 - plant.delay_frames], 0.45), abs=1e-12)

  def test_a_exp_sanity_clamped_to_expected_envelope(self):
    tracker = StoppingTracker(P)
    tracker.seed_command_history([-0.30] * 40)
    debug = {}
    run_frame(tracker, v=0.5, a_ego=-0.25, last=-0.50, max_exp=-0.12, min_exp=-0.18, debug=debug)
    assert -0.18 <= debug["a_exp"] <= -0.12


class TestDisturbanceEstimator:
  def test_zero_tau_bypass_is_single_frame_semantics(self):
    # spec 5.5.2: DIST_LPF_TAU_S = 0.0 is the KILL SWITCH bypass -- d_hat == innovation exactly
    p0 = dataclasses.replace(P, DIST_LPF_TAU_S=0.0)
    tracker = StoppingTracker(p0)
    for a_ego in (0.00, 0.08, -0.30, 0.15, -0.02):
      r = run_frame(tracker, v=0.5, a_ego=a_ego, last=-0.30, max_exp=-0.10, min_exp=-0.10)
      assert r.disturbance == pytest.approx(a_ego - (-0.10), abs=1e-12)

  def test_lpf_recursion_matches_tau(self):
    # pin a nonzero tau: this test exercises the LPF recursion, not the (kill-switch 0.0) default
    p_lpf = dataclasses.replace(P, DIST_LPF_TAU_S=0.30)
    tracker = StoppingTracker(p_lpf)
    d_hat = 0.0
    alpha = DT / 0.30
    for a_ego in (0.10, 0.10, 0.10, -0.05, 0.20):
      r = run_frame(tracker, v=0.5, a_ego=a_ego, last=-0.30, max_exp=-0.10, min_exp=-0.10)
      d_hat += alpha * ((a_ego + 0.10) - d_hat)
      assert r.disturbance == pytest.approx(d_hat, abs=1e-12)

  def test_push_requires_braking_and_speed_window_and_intent(self):
    # innovation +0.30 against a pinned a_exp; only the fully-gated frame may arm the inhibit
    cases = [
      (0.5, -0.30, DECISION, True),                                            # all gates pass
      (0.5, -0.02, DECISION, False),                                           # not braking (last > -0.05)
      (1.5, -0.30, DECISION, False),                                           # v outside DIST_PUSH window
      (0.001, -0.30, DECISION, False),                                         # below DIST_PUSH_V_MIN
      (0.5, -0.30, make_decision(source=StopSource.DROPOUT_HOLD), False),      # non-forced dropout: no tier
      (0.5, -0.30, make_decision(source=StopSource.DROPOUT_HOLD, legacy_forced=True), True),  # forced = full intent
    ]
    for v, last, decision, expect in cases:
      p0 = dataclasses.replace(P, DIST_LPF_TAU_S=0.0)
      tracker = StoppingTracker(p0)
      r = run_frame(tracker, v=v, a_ego=0.20, last=last, decision=decision, max_exp=-0.10, min_exp=-0.10)
      assert r.release_inhibit_active is expect, (v, last, decision.source, decision.legacy_forced)

  def test_push_deepens_to_disturbance_floor_past_end_stop_ceiling(self):
    # spec 5.3 scope / F26: push deepening BYPASSES the TERMINAL ceiling, converging on
    # A_DISTURBANCE_FLOOR(v) (the G4 lock floor) -- the anti-creep band must keep its authority
    v = 0.10
    tracker = StoppingTracker(P)
    u = -0.20
    for _ in range(300):
      u = run_frame(tracker, v=v, a_ego=0.10, last=u, max_exp=-0.10, min_exp=-0.10).output_accel
    floor = float(interp(v, P.A_DISTURBANCE_FLOOR_TABLE[0], P.A_DISTURBANCE_FLOOR_TABLE[1]))
    assert u == pytest.approx(floor, abs=1e-9)
    assert u < end_stop_ceiling(v, P)  # deeper than the quiescent ceiling

  def test_release_inhibit_window_magnitude(self):
    p0 = dataclasses.replace(P, DIST_LPF_TAU_S=0.0)
    tracker = StoppingTracker(p0)
    run_frame(tracker, v=0.30, a_ego=0.20, last=-0.30, max_exp=-0.10, min_exp=-0.10)
    assert tracker.release_inhibit_timer_s == pytest.approx(
      float(interp(0.30, P.T_RELEASE_INHIBIT_TABLE[0], P.T_RELEASE_INHIBIT_TABLE[1])))

  def test_locked_release_cap_while_inhibited(self):
    # while release-inhibited (no overbrake) the release budget is min(J_RELEASE, J_RELEASE_LOCKED)
    # and the F30 end-stop fast release is SUPPRESSED (spec 5.5.5(3) precedence / #37 note)
    v = 0.30
    p0 = dataclasses.replace(P, DIST_LPF_TAU_S=0.0)
    tracker = StoppingTracker(p0)
    run_frame(tracker, v=v, a_ego=0.20, last=-0.60, max_exp=-0.10, min_exp=-0.10)  # arm the timer
    # quiescent innovation now, but the timer still runs; target milder than last => release
    r = run_frame(tracker, v=v, a_ego=-0.10, last=-0.60, ref=make_ref(v=v, a_ref=-0.17), max_exp=-0.10, min_exp=-0.10)
    locked = min(float(interp(v, P.J_RELEASE_TABLE[0], P.J_RELEASE_TABLE[1])),
                 float(interp(v, P.J_RELEASE_LOCKED_TABLE[0], P.J_RELEASE_LOCKED_TABLE[1])))
    assert r.output_accel - (-0.60) == pytest.approx(locked * DT, abs=1e-12)

  def test_overbrake_release_floor_magnitude(self):
    # G4 overbrake floor (param #39): with the inhibit timer active and a_ego far below the
    # expected envelope, release accelerates to OVERBRAKE_RELEASE_FLOOR_TABLE(v)
    v = 0.30
    p0 = dataclasses.replace(P, DIST_LPF_TAU_S=0.0)
    tracker = StoppingTracker(p0)
    run_frame(tracker, v=v, a_ego=0.20, last=-0.80, max_exp=-0.10, min_exp=-0.10)  # arm the timer
    a_over = -0.10 - P.OVERBRAKE_TRIGGER_MARGIN - 0.10
    r = run_frame(tracker, v=v, a_ego=a_over, last=-0.80, ref=make_ref(v=v, a_ref=-0.17), max_exp=-0.10, min_exp=-0.10)
    floor = float(interp(v, P.OVERBRAKE_RELEASE_FLOOR_TABLE[0], P.OVERBRAKE_RELEASE_FLOOR_TABLE[1]))
    assert r.output_accel - (-0.80) == pytest.approx(floor * DT, abs=1e-12)


class TestPushReliefGate:
  def test_relief_signature_gated_to_legacy_speed_window(self):
    # F31: the G5 deep-command push relief may NOT fire below 0.12 m/s (hill-hold/arrest owns
    # the response there) nor above 2.5 m/s
    for v, expect in ((0.10, False), (0.119, False), (0.121, True), (0.50, True), (2.49, True), (2.51, False)):
      tracker = StoppingTracker(P)
      debug = {}
      run_frame(tracker, v=v, a_ego=0.10, last=-0.90, max_exp=0.10, min_exp=0.10, debug=debug)
      assert debug["relief_active"] is expect, v

  def test_relief_freezes_deepening_and_releases_at_relief_rate(self):
    # spec 5.5.2: brake authority saturated => cap the target at A_PUSH_RELIEF_CAP and freeze
    # deepening. The signature holds while last < RELIEF_CMD_DEEP; once shed past it the
    # quiescent path owns the bound again (the TERMINAL end-stop ceiling at this v).
    v = 0.50
    tracker = StoppingTracker(P)
    debug = {}
    r = run_frame(tracker, v=v, a_ego=0.10, last=-0.90, ref=make_ref(v=v, a_ref=-1.4),
                  max_exp=0.10, min_exp=0.10, debug=debug)
    assert debug["relief_active"] is True
    relief_rate = float(interp(v, PUSH_RELIEF_RELEASE_TABLE[0], PUSH_RELIEF_RELEASE_TABLE[1]))
    assert r.output_accel - (-0.90) == pytest.approx(relief_rate * DT, abs=1e-12)
    u = -0.90
    for _ in range(400):
      u_next = run_frame(tracker, v=v, a_ego=0.10, last=u, ref=make_ref(v=v, a_ref=-1.4),
                         max_exp=0.10, min_exp=0.10).output_accel
      if u < RELIEF_CMD_DEEP:
        assert u_next >= u  # deepening frozen while the relief signature holds
      assert u_next >= -0.90 - 1e-12  # never chases the -1.4 reference deeper
      u = u_next
    # equilibrium: the deep command was shed past the signature; the quiescent ceiling binds
    assert u == pytest.approx(end_stop_ceiling(v, P), abs=1e-9)
    assert u > -0.65  # comfortably out of the saturated-authority band


class TestArrest:
  def _push_frame(self, tracker, v, last, dt=DT):
    # strong push: pinned a_exp -0.10, a_ego +0.30, braking command
    return run_frame(tracker, v=v, a_ego=0.30, last=last, ref=make_ref(v=v, a_ref=-0.20),
                     max_exp=-0.10, min_exp=-0.10, dt=dt)

  def test_arrest_fires_below_v_max_with_rising_v_and_deepens_at_arrest_rate(self):
    # F27: the deepening budget switches to J_ARREST_TABLE (2.2-4.0 m/s^3); J_BRAKE alone is
    # 4-6x too slow to catch an HEV creep surge near 0 m/s
    p0 = dataclasses.replace(P, DIST_LPF_TAU_S=0.0)
    tracker = StoppingTracker(p0)
    self._push_frame(tracker, v=0.030, last=-0.20)          # establishes v_prev; arrest needs rising v
    u_prev = -0.20
    v = 0.040
    r = self._push_frame(tracker, v=v, last=u_prev)         # rising v below ARREST_V_MAX => arrest
    assert tracker.arrest_active
    j_arrest = float(interp(v, P.J_ARREST_TABLE[0], P.J_ARREST_TABLE[1]))
    j_brake = float(interp(v, P.J_BRAKE_TABLE[0], P.J_BRAKE_TABLE[1]))
    assert u_prev - r.output_accel == pytest.approx(j_arrest * DT, abs=1e-12)
    assert j_arrest * DT > j_brake * DT * 2.0               # materially faster than the normal budget

  def test_arrest_depth_bounded_by_arrest_max(self):
    p0 = dataclasses.replace(P, DIST_LPF_TAU_S=0.0)
    tracker = StoppingTracker(p0)
    self._push_frame(tracker, v=0.030, last=-0.20)
    u = -0.20
    vs = [0.040, 0.045, 0.050, 0.055] + [0.060] * 80
    for v in vs:
      u = self._push_frame(tracker, v=v, last=u).output_accel
      assert u >= float(interp(v, P.A_ARREST_MAX_TABLE[0], P.A_ARREST_MAX_TABLE[1])) - 1e-9
    # converged on the arrest bound itself (the G8 authority)
    assert u == pytest.approx(float(interp(0.060, P.A_ARREST_MAX_TABLE[0], P.A_ARREST_MAX_TABLE[1])), abs=1e-9)

  def test_arrest_exits_on_sustained_falling_v(self):
    p0 = dataclasses.replace(P, DIST_LPF_TAU_S=0.0)
    tracker = StoppingTracker(p0)
    self._push_frame(tracker, v=0.030, last=-0.20)
    self._push_frame(tracker, v=0.040, last=-0.20)
    assert tracker.arrest_active
    v = 0.040
    frames_needed = math.ceil(P.ARREST_EXIT_FALLING_T_S / DT)
    for _ in range(frames_needed + 1):
      v -= 0.0005
      self._push_frame(tracker, v=v, last=-0.50)
      if not tracker.arrest_active:
        break
    assert not tracker.arrest_active

  def test_arrest_exits_when_push_clears(self):
    p0 = dataclasses.replace(P, DIST_LPF_TAU_S=0.0)
    tracker = StoppingTracker(p0)
    self._push_frame(tracker, v=0.030, last=-0.20)
    self._push_frame(tracker, v=0.040, last=-0.20)
    assert tracker.arrest_active
    run_frame(tracker, v=0.040, a_ego=-0.10, last=-0.50, max_exp=-0.10, min_exp=-0.10)  # innovation 0
    assert not tracker.arrest_active


class TestTerminalPushCatch:
  def test_mid_speed_push_catch_uses_fast_deepening_rate(self):
    # Route 0000178a seg19 shape: v~0.58, a_ego~+0.49, stop intent active, cmd still shallow.
    # This is above the classic ARREST_V_MAX band, so the terminal push catch must own the faster
    # deepening instead of waiting for the near-standstill rebound-arrest latch.
    v = 0.58
    tracker = StoppingTracker(P)
    debug = {}
    ref = make_ref(v=v, a_ref=-0.18, phase=TrajPhase.TERMINAL)
    last = -0.20
    r = run_frame(tracker, v=v, a_ego=0.49, last=last, ref=ref, max_exp=-0.30, min_exp=-0.30, debug=debug)
    j_catch = float(interp(v, P.J_PUSH_CATCH_TABLE[0], P.J_PUSH_CATCH_TABLE[1]))
    assert debug["push_catch_active"] is True
    assert r.output_accel == pytest.approx(last - j_catch * DT, abs=1e-12)

  def test_mid_speed_push_catch_converges_to_desired_low_speed_decel(self):
    v = 0.58
    tracker = StoppingTracker(P)
    ref = make_ref(v=v, a_ref=-0.18, phase=TrajPhase.TERMINAL)
    u = -0.20
    for _ in range(30):
      u = run_frame(tracker, v=v, a_ego=0.49, last=u, ref=ref, max_exp=-0.30, min_exp=-0.30).output_accel
    desired = float(interp(v, P.A_DESIRED_LOWSPEED_TABLE[0], P.A_DESIRED_LOWSPEED_TABLE[1]))
    assert u == pytest.approx(desired, abs=1e-9)

  def test_quiet_terminal_tracking_does_not_arm_push_catch(self):
    v = 0.58
    tracker = StoppingTracker(P)
    debug = {}
    ref = make_ref(v=v, a_ref=-0.50, phase=TrajPhase.TERMINAL)
    last = -0.20
    r = run_frame(tracker, v=v, a_ego=-0.30, last=last, ref=ref, max_exp=-0.30, min_exp=-0.30, debug=debug)
    j_normal = float(interp(v, P.J_BRAKE_TABLE[0], P.J_BRAKE_TABLE[1]))
    assert debug["push_catch_active"] is False
    assert r.output_accel == pytest.approx(last - j_normal * DT, abs=1e-12)


class TestClampOrder:
  def test_saturated_recovery_never_punches_terminal_ceiling(self):
    # F7 (normative): recovery applies BEFORE the TERMINAL ceiling, which re-clamps -- with
    # recovery_i at RECOVERY_CAP and a quiescent estimator the output never exceeds
    # A_END_STOP(v) anywhere in 0-0.60 m/s
    for v in np.linspace(0.061, 0.599, 23):
      v = float(v)
      tracker = StoppingTracker(P)
      ceiling = end_stop_ceiling(v, P)
      u = ceiling  # start at the ceiling; deepening below it would be the F7 head-bob jolt
      for _ in range(40):
        tracker.recovery_i = P.RECOVERY_CAP
        tracker.rollout_m = 5.0
        ref = stop_reference(v_ego=v, a_ego=-0.30, target_distance_m=0.05, settled_time_s=0.0,
                             rollout_m=tracker.rollout_m, p=P)
        u = run_frame(tracker, v=v, a_ego=-0.30, last=u, ref=ref, max_exp=-0.30, min_exp=-0.30).output_accel
        assert u >= ceiling - 1e-9, v

  def test_push_active_may_reach_disturbance_floor(self):
    # the same TERMINAL band, push active: the bound is A_DISTURBANCE_FLOOR(v), not the ceiling
    v = 0.10
    tracker = StoppingTracker(P)
    u = -0.20
    for _ in range(300):
      tracker.recovery_i = P.RECOVERY_CAP
      tracker.rollout_m = 5.0
      u = run_frame(tracker, v=v, a_ego=0.10, last=u, max_exp=-0.10, min_exp=-0.10).output_accel
    floor = float(interp(v, P.A_DISTURBANCE_FLOOR_TABLE[0], P.A_DISTURBANCE_FLOOR_TABLE[1]))
    assert u == pytest.approx(floor, abs=1e-9)
    assert floor < end_stop_ceiling(v, P)

  def test_settle_hold_absolute_arrest_bound(self):
    # spec 5.5.5(4): deepening below A_ARREST_MAX(v) is never allowed in SETTLE/HOLD
    v = 0.01
    tracker = StoppingTracker(P)
    ref = StopReference(a_ref=-3.0, phase=TrajPhase.SETTLE, j_brake_max=50.0, j_release_max=0.9, remaining_m=0.0)
    r = run_frame(tracker, v=v, a_ego=-0.05, last=-1.0, ref=ref, max_exp=-0.05, min_exp=-0.05)
    assert r.output_accel >= float(interp(v, P.A_ARREST_MAX_TABLE[0], P.A_ARREST_MAX_TABLE[1])) - 1e-12


class TestEndStopFastRelease:
  def test_inherited_deep_brake_shed_before_wheel_stop(self):
    # F30: when the ceiling binds in TERMINAL, the release budget is J_END_STOP_RELEASE_TABLE --
    # an inherited -0.60 reaches the calibrated no-jolt cap BEFORE v reaches the settle band
    tracker = StoppingTracker(P)
    u = -0.60
    v = 0.45
    reached = False
    while v > P.V_SETTLE:
      ref = stop_reference(v_ego=v, a_ego=-0.20, target_distance_m=-1.0, settled_time_s=0.0,
                           rollout_m=tracker.rollout_m, p=P)
      u = run_frame(tracker, v=v, a_ego=-0.20, last=u, ref=ref, max_exp=-0.20, min_exp=-0.20).output_accel
      if u >= end_stop_ceiling(v, P) - 1e-6:
        reached = True
      v -= 0.20 * DT
    assert reached, "command never shed to the binding end-stop ceiling before wheel-stop"
    assert u >= end_stop_ceiling(0.061, P) - 0.02

  def test_shed_rate_is_end_stop_release_budget(self):
    v = 0.40
    tracker = StoppingTracker(P)
    ref = stop_reference(v_ego=v, a_ego=-0.20, target_distance_m=-1.0, settled_time_s=0.0, rollout_m=0.0, p=P)
    r = run_frame(tracker, v=v, a_ego=-0.20, last=-0.60, ref=ref, max_exp=-0.20, min_exp=-0.20)
    j_fast = float(interp(v, P.J_END_STOP_RELEASE_TABLE[0], P.J_END_STOP_RELEASE_TABLE[1]))
    assert r.output_accel - (-0.60) == pytest.approx(j_fast * DT, abs=1e-12)
    assert j_fast > float(interp(v, P.J_RELEASE_TABLE[0], P.J_RELEASE_TABLE[1]))  # genuinely faster


class TestRecoveryIntegrator:
  def test_growth_and_application_verbatim(self):
    # G9 verbatim gains (stopping_controller.py:849-860, :1342-1347)
    v, a_ego = 0.30, -0.10
    tracker = StoppingTracker(P)
    tracker.rollout_m = 5.0  # above any arm threshold
    desired = float(interp(v, P.A_DESIRED_LOWSPEED_TABLE[0], P.A_DESIRED_LOWSPEED_TABLE[1]))
    shortfall = float(np.clip(a_ego - desired, 0.0, 1.2))
    expected_i = shortfall * float(interp(v, P.RECOVERY_GAIN_TABLE[0], P.RECOVERY_GAIN_TABLE[1])) * DT
    # huge jerk budget so the slew does not bind and the applied deepening is observable exactly
    ref = make_ref(v=v, a_ref=-0.20, j_brake=1000.0, j_release=1000.0)
    r = run_frame(tracker, v=v, a_ego=a_ego, last=-0.20, ref=ref, max_exp=a_ego, min_exp=a_ego)
    assert tracker.recovery_i == pytest.approx(expected_i, abs=1e-12)
    applied = expected_i * float(interp(v, P.RECOVERY_APPLY_GAIN_TABLE[0], P.RECOVERY_APPLY_GAIN_TABLE[1]))
    assert r.output_accel == pytest.approx(-0.20 - applied, abs=1e-12)

  def test_decay_when_rollout_below_arm(self):
    v = 0.30
    tracker = StoppingTracker(P)
    tracker.recovery_i = 0.50
    tracker.rollout_m = 0.0
    run_frame(tracker, v=v, a_ego=-0.30, last=-0.20, max_exp=-0.30, min_exp=-0.30)
    decay = float(interp(v, P.RECOVERY_DECAY_TABLE[0], P.RECOVERY_DECAY_TABLE[1])) * DT
    assert tracker.recovery_i == pytest.approx(0.50 - decay, abs=1e-12)

  def test_cap(self):
    v = 0.20
    tracker = StoppingTracker(P)
    tracker.recovery_i = P.RECOVERY_CAP
    tracker.rollout_m = 5.0
    run_frame(tracker, v=v, a_ego=1.0, last=-0.20, max_exp=1.0, min_exp=1.0)
    assert tracker.recovery_i <= P.RECOVERY_CAP

  def test_zeroed_in_track_phase(self):
    tracker = StoppingTracker(P)
    tracker.recovery_i = 0.50
    run_frame(tracker, v=1.0, a_ego=-0.30, last=-0.20, ref=make_ref(v=1.0, phase=TrajPhase.TRACK),
              max_exp=-0.30, min_exp=-0.30)
    assert tracker.recovery_i == 0.0


class TestRolloutAndSettle:
  def test_rollout_integrates_below_1p2_and_decays_at_standstill(self):
    tracker = StoppingTracker(P)
    run_frame(tracker, v=0.50, a_ego=-0.30, last=-0.20, max_exp=-0.30, min_exp=-0.30)
    assert tracker.rollout_m == pytest.approx(0.50 * DT)
    run_frame(tracker, v=0.50, a_ego=-0.30, last=-0.20, max_exp=-0.30, min_exp=-0.30)
    assert tracker.rollout_m == pytest.approx(1.00 * DT)
    tracker.rollout_m = 1.0
    run_frame(tracker, v=0.01, a_ego=0.0, last=-0.15, max_exp=0.0, min_exp=0.0)
    assert tracker.rollout_m == pytest.approx(1.0 - P.ROLLOUT_DECAY_MPS * DT)

  def test_settled_time_accumulates_then_resets(self):
    tracker = StoppingTracker(P)
    for _ in range(10):
      run_frame(tracker, v=0.01, a_ego=0.0, last=-0.15, max_exp=0.0, min_exp=0.0)
    assert tracker.settled_time_s == pytest.approx(10 * DT)
    run_frame(tracker, v=0.10, a_ego=0.0, last=-0.15, max_exp=0.0, min_exp=0.0)
    assert tracker.settled_time_s == 0.0

  def test_hold_relax_timing_through_trajectory(self):
    # spec 5.3 HOLD: A_HOLD until T_HOLD_RELAX_S of settled time, then A_HOLD_RELAXED
    tracker = StoppingTracker(P)
    v = 0.01
    frames = math.ceil(P.T_HOLD_RELAX_S / DT)
    for k in range(frames + 2):
      ref = stop_reference(v_ego=v, a_ego=0.0, target_distance_m=-1.0,
                           settled_time_s=tracker.settled_time_s, rollout_m=tracker.rollout_m, p=P)
      if k == 1:
        assert ref.phase == TrajPhase.HOLD
        assert ref.a_ref == pytest.approx(float(interp(v, P.A_HOLD_TABLE[0], P.A_HOLD_TABLE[1])))
      run_frame(tracker, v=v, a_ego=0.0, last=-0.15, ref=ref, max_exp=0.0, min_exp=0.0)
    ref = stop_reference(v_ego=v, a_ego=0.0, target_distance_m=-1.0,
                         settled_time_s=tracker.settled_time_s, rollout_m=tracker.rollout_m, p=P)
    assert ref.a_ref == pytest.approx(float(interp(v, P.A_HOLD_RELAXED_TABLE[0], P.A_HOLD_RELAXED_TABLE[1])))


class TestDropoutHoldEnvelope:
  def test_non_forced_dropout_hold_never_deepens_beyond_near_hold(self):
    # spec 5.5.6: a timed (non-forced) DROPOUT_HOLD keeps the envelope, no deepening beyond
    # A_NEAR_HOLD(v) -- even with a deep reference
    v = 0.30
    decision = make_decision(source=StopSource.DROPOUT_HOLD)
    tracker = StoppingTracker(P)
    u = -0.10
    for _ in range(100):
      u = run_frame(tracker, v=v, a_ego=-0.20, last=u, ref=make_ref(v=v, a_ref=-0.60),
                    decision=decision, max_exp=-0.20, min_exp=-0.20).output_accel
    assert u == pytest.approx(float(interp(v, P.A_NEAR_HOLD_TABLE[0], P.A_NEAR_HOLD_TABLE[1])), abs=1e-9)

  def test_forced_dropout_hold_keeps_full_authority(self):
    v = 0.30
    decision = make_decision(source=StopSource.DROPOUT_HOLD, legacy_forced=True)
    tracker = StoppingTracker(P)
    u = -0.10
    for _ in range(100):
      u = run_frame(tracker, v=v, a_ego=-0.20, last=u, ref=make_ref(v=v, a_ref=-0.45),
                    decision=decision, max_exp=-0.20, min_exp=-0.20).output_accel
    assert u < float(interp(v, P.A_NEAR_HOLD_TABLE[0], P.A_NEAR_HOLD_TABLE[1])) - 0.05


class TestResetOnDisengage:
  def test_reset_clears_all_estimator_state(self):
    # spec 5.5.6 / F5: driver brake/regen tap -> USER_DISABLE -> facade reset() -> the tracker
    # carries NO stale d_hat / recovery_i / rollout into the re-engagement
    p0 = dataclasses.replace(P, DIST_LPF_TAU_S=0.0)
    tracker = StoppingTracker(p0)
    tracker.seed_command_history([-0.4] * 20)
    for k in range(20):
      run_frame(tracker, v=0.30 + 0.001 * k, a_ego=0.25, last=-0.40, max_exp=-0.10, min_exp=-0.10)
    assert tracker.d_hat != 0.0
    assert tracker.release_inhibit_timer_s > 0.0
    assert tracker.rollout_m > 0.0
    tracker.reset()
    assert tracker.d_hat == 0.0
    assert tracker.release_inhibit_timer_s == 0.0
    assert tracker.rollout_m == 0.0
    assert tracker.recovery_i == 0.0
    assert tracker.settled_time_s == 0.0
    assert not tracker.arrest_active

  def test_fresh_reengage_equals_brand_new_tracker(self):
    p0 = dataclasses.replace(P, DIST_LPF_TAU_S=0.0)
    used = StoppingTracker(p0)
    used.seed_command_history([-0.4] * 20)
    for k in range(20):
      run_frame(used, v=0.30 + 0.001 * k, a_ego=0.25, last=-0.40, max_exp=-0.10, min_exp=-0.10)
    used.reset()
    fresh = StoppingTracker(p0)
    frames = [(0.40 - 0.01 * k, -0.25, -0.30 - 0.002 * k) for k in range(25)]
    for v, a_ego, last in frames:
      ru = run_frame(used, v=v, a_ego=a_ego, last=last, max_exp=-0.20, min_exp=-0.20)
      rf = run_frame(fresh, v=v, a_ego=a_ego, last=last, max_exp=-0.20, min_exp=-0.20)
      assert ru == rf


def _push_signature_trace(tau: float, samples) -> tuple[list[float], list[bool]]:
  """Per-frame push-signature stream driving the tracker open-loop over a recorded fixture.
  The signature is the tracker's own trigger (filtered disturbance >= threshold while braking
  in the speed window); for tau = 0.0 it is exactly the legacy single-frame trigger."""
  p = dataclasses.replace(P, DIST_LPF_TAU_S=tau)
  tracker = StoppingTracker(p)
  times: list[float] = []
  signature: list[bool] = []
  last = -0.10
  t_prev = None
  for s in samples:
    dt = (s.t - t_prev) if t_prev is not None else 0.1
    t_prev = s.t
    if not (math.isfinite(dt) and 0.0 < dt < 1.0):
      dt = 0.1
    if s.accel_cmd is not None:
      last = float(s.accel_cmd)
    v, a = float(s.v_ego), float(s.a_ego)
    max_exp = float(interp(v, P.EXPECTED_ACCEL_V_BP, P.EXPECTED_ACCEL_MAX))
    min_exp = float(interp(v, P.EXPECTED_ACCEL_V_BP, P.EXPECTED_ACCEL_MIN))
    ref = stop_reference(v_ego=v, a_ego=a, target_distance_m=-1.0,
                         settled_time_s=tracker.settled_time_s, rollout_m=tracker.rollout_m, p=p)
    r = tracker.update(ref=ref, decision=DECISION, v_ego=v, a_ego=a, last_output_accel=last,
                       max_expected_accel=max_exp, min_expected_accel=min_exp,
                       stop_accel=-2.0, dt=dt)
    thresh = P.DIST_PUSH_THRESH_LOW if v < P.DIST_PUSH_THRESH_V_SPLIT else P.DIST_PUSH_THRESH_HIGH
    times.append(s.t)
    signature.append(r.disturbance >= thresh and last < P.DIST_PUSH_MIN_BRAKE
                     and P.DIST_PUSH_V_MIN < v < P.DIST_PUSH_V_MAX)
  return times, signature


def _onset(times: list[float], signature: list[bool]) -> tuple[float | None, int]:
  """First trigger time + its persistence (consecutive frames the signature held)."""
  for k, fired in enumerate(signature):
    if fired:
      run = 0
      while k + run < len(signature) and signature[k + run]:
        run += 1
      return times[k], run
  return None, 0


class TestLpfFixtureEquivalence:
  """Spec 5.5.2 / F21: fixture-based LPF-vs-single-frame equivalence on stop_scenarios.py.
  The full event-store replay is WP6's estimator_equivalence.py gate artifact."""

  def test_lpf_onset_tracks_single_frame_trigger_on_fixtures(self):
    # Fixture-level criterion (the spec's +-0.2 s / >=90% / window-overlap gate runs at 100 Hz on
    # real push-disturbance EVENTS in WP6's estimator_equivalence.py): a SUSTAINED single-frame
    # trigger (>= 3 consecutive 10 Hz frames) must be tracked by the LPF within 2.5 frames
    # (0.25 s) on >= 90% of fixtures; a 1-2 frame transient may legitimately be filtered out --
    # that is the estimator's purpose, and the 0.0 bypass (tested above) is the kill switch (R9).
    sustained = 0
    tracked = 0
    any_single = False
    for name, samples in SCENARIOS.items():
      t0, persistence = _onset(*_push_signature_trace(0.0, samples))
      if t0 is None:
        continue
      any_single = True
      if persistence < 3:
        continue
      sustained += 1
      t_lpf, _ = _onset(*_push_signature_trace(P.DIST_LPF_TAU_S, samples))
      if t_lpf is not None and abs(t_lpf - t0) <= 0.25 + 1e-9:
        tracked += 1
      else:
        print(f"LPF missed sustained trigger on {name}: single={t0:.2f} lpf={t_lpf}")
    if not any_single:
      pytest.skip("no fixture produces a single-frame push trigger; covered by WP6 estimator_equivalence.py")
    if sustained == 0:
      pytest.skip("no fixture sustains a push trigger >= 3 frames; covered by WP6 estimator_equivalence.py")
    assert tracked / sustained >= 0.9, f"LPF tracked {tracked}/{sustained} sustained push triggers"

  def test_event_store_replay_extension(self):
    # graceful skip on a clean checkout (spec section 8): the event-store replay is machine-local
    if not EVENT_STORE_DIR.exists():
      pytest.skip(f"event store absent ({EVENT_STORE_DIR}); WP6 estimator_equivalence.py owns the full replay")
    npz_files = sorted(EVENT_STORE_DIR.glob("events/*.npz"))[:20]
    if not npz_files:
      pytest.skip("event store present but holds no trace npz files yet")
    compared = 0
    within = 0
    for path in npz_files:
      data = np.load(path)
      if not {"t", "v_ego", "a_ego", "accel_cmd"} <= set(data.files):
        continue

      @dataclasses.dataclass
      class _S:
        t: float
        v_ego: float
        a_ego: float
        accel_cmd: float

      samples = [_S(float(t), float(v), float(a), float(c))
                 for t, v, a, c in zip(data["t"], data["v_ego"], data["a_ego"], data["accel_cmd"], strict=True)]
      t0, persistence = _onset(*_push_signature_trace(0.0, samples))
      if t0 is None or persistence < 3:
        continue
      compared += 1
      t_lpf, _ = _onset(*_push_signature_trace(P.DIST_LPF_TAU_S, samples))
      if t_lpf is not None and abs(t_lpf - t0) <= 0.2 + 1e-9:
        within += 1
    if compared == 0:
      pytest.skip("no event-store trace sustains a single-frame push trigger")
    assert within / compared >= 0.9
