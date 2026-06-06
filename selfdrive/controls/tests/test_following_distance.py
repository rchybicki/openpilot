import pytest
import itertools
from parameterized import parameterized_class

from cereal import log
from openpilot.common.constants import CV

pytest.importorskip("casadi")
pytest.importorskip("openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.c_generated_code.acados_ocp_solver_pyx")

from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import (
  LEAD_STOP_DISTANCE_TARGET,
  STOP_DISTANCE,
  desired_follow_distance,
  get_T_FOLLOW,
  get_distance_to_stopped_lead_target,
)
from openpilot.selfdrive.test.longitudinal_maneuvers.maneuver import Maneuver


def run_following_distance_simulation(v_lead, t_end=100.0, e2e=False, personality=0):
  man = Maneuver(
    '',
    duration=t_end,
    initial_speed=float(v_lead),
    lead_relevancy=True,
    initial_distance_lead=100,
    speed_lead_values=[v_lead],
    breakpoints=[0.],
    e2e=e2e,
    personality=personality,
  )
  valid, output = man.evaluate()
  assert valid
  return output[-1,2] - output[-1,1]


@parameterized_class(("e2e", "personality", "speed"), itertools.product(
                      [True, False], # e2e
                      [log.LongitudinalPersonality.relaxed, # personality
                       log.LongitudinalPersonality.standard,
                       log.LongitudinalPersonality.aggressive],
                      [0,10,35])) # speed
class TestFollowingDistance:
  def test_following_distance(self):
    v_lead = float(self.speed)
    simulation_steady_state = run_following_distance_simulation(v_lead, e2e=self.e2e, personality=self.personality)
    correct_steady_state = desired_follow_distance(v_lead, v_lead, simulation_steady_state, t_follow=get_T_FOLLOW(personality=self.personality))
    err_ratio = 0.2 if self.e2e else 0.1
    assert simulation_steady_state == pytest.approx(correct_steady_state, abs=err_ratio * correct_steady_state + .5)


def test_desired_follow_distance_keeps_legacy_default_stopped_lead_gap():
  t_follow = get_T_FOLLOW(personality=log.LongitudinalPersonality.standard)
  desired_gap = desired_follow_distance(
    v_ego=0.0,
    v_lead=0.0,
    v_lead_distance=10.0,
    t_follow=t_follow,
  )
  assert desired_gap == pytest.approx(STOP_DISTANCE, abs=1e-6)


def test_high_speed_follow_reduction_requires_not_leftmost_lane():
  v_ego = 140.0 * CV.KPH_TO_MS

  leftmost_t_follow = get_T_FOLLOW(personality=log.LongitudinalPersonality.standard, v_ego=v_ego, not_leftmost_lane=False)
  not_leftmost_t_follow = get_T_FOLLOW(personality=log.LongitudinalPersonality.standard, v_ego=v_ego, not_leftmost_lane=True)

  assert leftmost_t_follow == pytest.approx(1.45)
  assert not_leftmost_t_follow == pytest.approx(1.05)


def test_desired_follow_distance_uses_explicit_stopped_lead_target():
  t_follow = get_T_FOLLOW(personality=log.LongitudinalPersonality.standard)
  desired_gap = desired_follow_distance(
    v_ego=0.0,
    v_lead=0.0,
    v_lead_distance=10.0,
    t_follow=t_follow,
    lead_stop_distance_target=LEAD_STOP_DISTANCE_TARGET,
  )
  assert desired_gap == pytest.approx(LEAD_STOP_DISTANCE_TARGET, abs=1e-6)


def test_distance_to_stopped_lead_target_only_activates_for_slow_leads():
  moving_gap = get_distance_to_stopped_lead_target(
    v_lead_raw=10.0,
    v_lead_distance_raw=20.0,
    lead_stop_distance_target=LEAD_STOP_DISTANCE_TARGET,
  )
  stopped_gap = get_distance_to_stopped_lead_target(
    v_lead_raw=0.0,
    v_lead_distance_raw=20.0,
    lead_stop_distance_target=LEAD_STOP_DISTANCE_TARGET,
  )

  assert moving_gap == pytest.approx(0.0, abs=1e-6)
  assert stopped_gap == pytest.approx(20.0 - LEAD_STOP_DISTANCE_TARGET, abs=1e-6)


def test_distance_to_stopped_lead_target_surfaces_for_close_creeping_lead_only():
  creeping_close_gap = get_distance_to_stopped_lead_target(
    v_lead_raw=1.6,
    v_lead_distance_raw=6.8,
    lead_stop_distance_target=LEAD_STOP_DISTANCE_TARGET,
  )
  creeping_far_gap = get_distance_to_stopped_lead_target(
    v_lead_raw=1.6,
    v_lead_distance_raw=10.0,
    lead_stop_distance_target=LEAD_STOP_DISTANCE_TARGET,
  )

  assert creeping_close_gap > 0.0
  assert creeping_far_gap == pytest.approx(0.0, abs=1e-6)
