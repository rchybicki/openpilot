"""WP5 acceptance tests for the stateless trajectory reference law (FINAL_SPEC sections 5.3, 8).

Pure python + numpy only (spec section 8 import-clean rule).
"""

import dataclasses

import numpy as np
import pytest

from openpilot.selfdrive.controls.lib.stopping_params import STOPPING_PARAMS
from openpilot.selfdrive.controls.lib.stopping_trajectory import (
  StopReference,
  TrajPhase,
  end_stop_ceiling,
  remaining_distance_m,
  stop_reference,
)

interp = np.interp
P = STOPPING_PARAMS

V_GRID = [0.0, 0.01, 0.02, 0.05, 0.06, 0.08, 0.12, 0.20, 0.30, 0.45, 0.59, 0.60, 0.70, 0.85, 0.86, 1.0, 1.2, 1.6, 2.0, 3.0]
A_GRID = [-1.5, -0.8, -0.4, -0.2, -0.05, 0.0, 0.3]
TARGET_GRID = [-1.0, 0.05, 0.3, 0.8, 1.5, 3.0, 6.0, 9.0]


def ref(v, a=-0.3, target=-1.0, settled=0.0, rollout=0.0) -> StopReference:
  return stop_reference(v_ego=v, a_ego=a, target_distance_m=target, settled_time_s=settled, rollout_m=rollout, p=P)


class TestInvariants:
  def test_a_ref_in_stopping_authority_everywhere(self):
    # spec 5.7: no command outside [stop_accel, -0.05] in stopping authority; the reference
    # itself must always sit at or below -0.05
    for v in V_GRID:
      for a in A_GRID:
        for target in TARGET_GRID:
          for settled in (0.0, 0.4, 1.2):
            r = ref(v, a, target, settled)
            assert r.a_ref <= -0.05 + 1e-12, (v, a, target, settled)
            assert np.isfinite(r.a_ref)

  def test_jerk_budgets_positive_and_bounded(self):
    for v in V_GRID:
      for settled in (0.0, 1.0):
        r = ref(v, settled=settled)
        assert 0.0 < r.j_brake_max <= max(P.J_BRAKE_TABLE[1]) + 1e-12
        assert 0.0 < r.j_release_max <= max(max(P.J_RELEASE_TABLE[1]), P.J_SETTLE_RELEASE) + 1e-12

  def test_jerk_budget_continuity_across_phase_bounds(self):
    # the budget tables are continuous in v; crossing TRACK/TERMINAL must not step the brake budget
    for v0, v1 in ((P.V_NEAR_HOLD - 1e-6, P.V_NEAR_HOLD + 1e-6),):
      r0, r1 = ref(v0), ref(v1)
      assert abs(r0.j_brake_max - r1.j_brake_max) < 1e-3
      assert abs(r0.j_release_max - r1.j_release_max) < 1e-3


class TestPhases:
  def test_phase_mapping(self):
    assert ref(1.2).phase == TrajPhase.TRACK
    assert ref(P.V_NEAR_HOLD + 1e-6).phase == TrajPhase.TRACK
    assert ref(P.V_NEAR_HOLD).phase == TrajPhase.TERMINAL
    assert ref(0.3).phase == TrajPhase.TERMINAL
    assert ref(P.V_SETTLE + 1e-6).phase == TrajPhase.TERMINAL
    assert ref(P.V_SETTLE).phase == TrajPhase.SETTLE
    assert ref(0.01, settled=0.0).phase == TrajPhase.SETTLE
    assert ref(0.01, settled=0.2).phase == TrajPhase.HOLD

  def test_settle_uses_settle_release_budget(self):
    r = ref(0.02, settled=0.0)
    assert r.j_release_max == P.J_SETTLE_RELEASE

  def test_hold_relax_timing(self):
    v = 0.01
    hold = float(interp(v, P.A_HOLD_TABLE[0], P.A_HOLD_TABLE[1]))
    relaxed = float(interp(v, P.A_HOLD_RELAXED_TABLE[0], P.A_HOLD_RELAXED_TABLE[1]))
    assert ref(v, settled=P.T_HOLD_RELAX_S - 0.01).a_ref == pytest.approx(hold)
    assert ref(v, settled=P.T_HOLD_RELAX_S).a_ref == pytest.approx(relaxed)
    assert relaxed > hold  # relax means milder brake


class TestTerminalEnvelope:
  def test_end_stop_ceiling_binds_over_full_quiescent_domain(self):
    # spec 5.7 / F7 scope: never deeper than A_END_STOP(v) anywhere in 0-0.60 m/s on the
    # quiescent path -- even with a near-zero remaining distance demanding deep decel
    for v in np.linspace(0.061, 0.60, 60):
      for target in (0.05, 0.10, 0.30, -1.0):
        r = ref(float(v), a=-1.2, target=target, rollout=5.0)
        assert r.a_ref >= end_stop_ceiling(float(v), P) - 1e-12, (v, target)

  def test_terminal_envelope_within_tail_planner_band(self):
    # G16 oracle (spec 3.1 row 16): the preserved hold/desired tables bound the terminal
    # reference -- never deeper than A_DESIRED_LOWSPEED(v), never milder than A_NEAR_HOLD(v)
    for v in np.linspace(0.061, 0.85, 80):
      for a in A_GRID:
        for target in TARGET_GRID:
          r = ref(float(v), a, target)
          assert r.phase == TrajPhase.TERMINAL
          lo = float(interp(v, P.A_DESIRED_LOWSPEED_TABLE[0], P.A_DESIRED_LOWSPEED_TABLE[1]))
          hi = float(interp(v, P.A_NEAR_HOLD_TABLE[0], P.A_NEAR_HOLD_TABLE[1]))
          assert lo - 1e-12 <= r.a_ref <= max(hi, end_stop_ceiling(float(v), P)) + 1e-12

  def test_track_phase_respects_approach_floor(self):
    for v in (0.9, 1.2, 1.6, 2.5):
      floor = float(interp(v, P.A_APPROACH_FLOOR_TABLE[0], P.A_APPROACH_FLOOR_TABLE[1]))
      r = ref(v, a=-1.4, target=0.05)
      assert r.a_ref >= floor - 1e-12

  def test_track_rollout_arms_desired_low_speed_floor(self):
    v = 1.0
    arm = float(interp(v, P.RECOVERY_ARM_TABLE[0], P.RECOVERY_ARM_TABLE[1]))
    desired = float(interp(v, P.A_DESIRED_LOWSPEED_TABLE[0], P.A_DESIRED_LOWSPEED_TABLE[1]))
    gentle = ref(v, a=-0.2, target=6.0, rollout=0.0)
    armed = ref(v, a=-0.2, target=6.0, rollout=arm + 0.1)
    assert gentle.a_ref > desired  # without rollout debt the long target keeps the ref shallow
    assert armed.a_ref == pytest.approx(desired)


class TestDistanceFeedback:
  def test_remaining_distance_explicit_vs_kinematic(self):
    assert remaining_distance_m(v_ego=1.0, a_ego=-0.5, target_distance_m=2.5, p=P) == 2.5
    assert remaining_distance_m(v_ego=1.0, a_ego=-0.5, target_distance_m=9.0, p=P) == P.EXPLICIT_REMAINING_CLIP_M
    assert remaining_distance_m(v_ego=1.0, a_ego=-0.5, target_distance_m=0.01, p=P) == 0.05
    # kinematic fallback (param #29): v^2 / (2 * max(0.20, -a_ego)) clipped to 3.0
    assert remaining_distance_m(v_ego=1.0, a_ego=-0.5, target_distance_m=-1.0, p=P) == pytest.approx(1.0)
    assert remaining_distance_m(v_ego=1.0, a_ego=0.3, target_distance_m=-1.0, p=P) == pytest.approx(1.0 / (2 * 0.20))
    assert remaining_distance_m(v_ego=4.0, a_ego=-0.2, target_distance_m=-1.0, p=P) == P.KINEMATIC_REMAINING_CLIP_M

  def test_target_jump_monotonic_response(self):
    # a target jump DOWN (lead reacquired closer) must never make the reference milder
    for v in (0.3, 0.6, 1.2):
      targets = [6.0, 3.0, 1.5, 0.8, 0.4, 0.2, 0.05]
      refs = [ref(v, a=-0.3, target=t).a_ref for t in targets]
      for nearer, farther in zip(refs[1:], refs[:-1], strict=True):
        assert nearer <= farther + 1e-12, (v, refs)


class TestStateless:
  def test_dt_invariance_100hz_vs_10hz(self):
    # the reference law is memoryless and dt-free: the same kinematic state yields the same
    # reference whether visited on a 100 Hz or a 10 Hz grid (spec section 8 dt-invariance)
    def v_of_t(t):
      return max(0.0, 1.2 - 0.4 * t)

    for k in range(31):  # common sample points every 0.1 s
      t = k * 0.1
      state = dict(v_ego=v_of_t(t), a_ego=-0.4, target_distance_m=max(0.05, 1.8 - 0.5 * t),
                   settled_time_s=0.0, rollout_m=0.2)
      r100 = stop_reference(**state, p=P)   # the 100 Hz pass visits this exact state at frame 10k
      r10 = stop_reference(**state, p=P)
      assert r100.a_ref == pytest.approx(r10.a_ref, abs=1e-6)
      assert r100.phase == r10.phase
      assert r100.j_brake_max == pytest.approx(r10.j_brake_max, abs=1e-6)
      assert r100.j_release_max == pytest.approx(r10.j_release_max, abs=1e-6)

  def test_pure_function_no_hidden_state(self):
    a = [stop_reference(v_ego=0.4, a_ego=-0.3, target_distance_m=1.0, settled_time_s=0.0, rollout_m=0.5, p=P)
         for _ in range(5)]
    assert all(r == a[0] for r in a)

  def test_params_injectable(self):
    custom = dataclasses.replace(P, V_NEAR_HOLD=0.5)
    assert stop_reference(v_ego=0.6, a_ego=-0.3, target_distance_m=-1.0, settled_time_s=0.0,
                          rollout_m=0.0, p=custom).phase == TrajPhase.TRACK
