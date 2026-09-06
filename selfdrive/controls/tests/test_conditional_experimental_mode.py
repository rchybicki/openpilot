from types import SimpleNamespace

import pytest

from openpilot.frogpilot.controls.lib import conditional_experimental_mode as cem_module
from openpilot.frogpilot.controls.lib.conditional_experimental_mode import ConditionalExperimentalMode, THRESHOLD


def make_cem(v_lead, d_rel, t_follow=1.45):
  planner = SimpleNamespace(
    tracking_lead=True,
    lead_one=SimpleNamespace(vLead=v_lead, dRel=d_rel),
    frogpilot_following=SimpleNamespace(t_follow=t_follow, slower_lead=False),
  )
  return ConditionalExperimentalMode(planner)


def make_sm(model_v, prob=0.9, traffic_mode=False):
  # model lead trajectory relative to its first point; x grows with the lead's speed
  x = [sum(model_v[:i]) * 2.0 for i in range(len(model_v))]
  lead = SimpleNamespace(prob=prob, x=x, v=list(model_v))
  return {
    "modelV2": SimpleNamespace(leadsV3=[lead], velocity=SimpleNamespace(x=[0.0] * 33)),
    "frogpilotCarState": SimpleNamespace(trafficModeEnabled=traffic_mode),
  }


TOGGLES = SimpleNamespace(conditional_slower_lead=True, conditional_stopped_lead=True, lead_detection_probability=0.35)


def settle(cem, v_ego, v_lead, sm, toggles=TOGGLES, frames=200):
  for _ in range(frames):
    cem.slow_lead(v_ego, v_lead, sm, toggles, True)
  return cem.slow_lead_detected


def test_kinematic_lead_triggers_when_required_deceleration_is_high():
  # 25 m/s ego, 15 m/s lead 30 m ahead: (625-225)/(2*(30-5.5)) = 8.2 m/s^2 >> 2.0
  cem = make_cem(v_lead=15.0, d_rel=30.0)
  assert settle(cem, 25.0, 15.0, make_sm([15.0] * 6)) is True
  assert cem.kinematic_lead is True


def test_kinematic_lead_has_hysteresis():
  cem = make_cem(v_lead=20.0, d_rel=60.0)
  cem.kinematic_lead = True
  # required decel (625-400)/(2*54.5) = 2.06: above the 1.5 hold threshold, below the 2.0 arm threshold once released
  cem.slow_lead(25.0, 20.0, make_sm([20.0] * 6), TOGGLES, True)
  assert cem.kinematic_lead is True
  cem.kinematic_lead = False
  cem.frogpilot_planner.lead_one.dRel = 65.0  # (225)/(2*59.5) = 1.89 < 2.0
  cem.slow_lead(25.0, 20.0, make_sm([20.0] * 6), TOGGLES, True)
  assert cem.kinematic_lead is False


# a lead matching ego speed at 20 m whose predicted trajectory brakes hard: its future obstacle (position + stopping
# distance) lands inside the safe follow distance + buffer while the lead has not slowed yet
BRAKING_LEAD = [20.0, 14.0, 8.0, 4.0, 2.0, 2.0]


def test_predicted_slower_lead_triggers_before_the_lead_has_slowed():
  cem = make_cem(v_lead=20.0, d_rel=20.0)
  assert settle(cem, 20.0, 20.0, make_sm(BRAKING_LEAD)) is True
  assert cem.kinematic_lead is False


def test_predicted_lead_ignored_when_model_prob_is_low():
  cem = make_cem(v_lead=20.0, d_rel=20.0)
  assert settle(cem, 20.0, 20.0, make_sm(BRAKING_LEAD, prob=0.1)) is False


def test_predicted_lead_ignored_when_its_future_obstacle_stays_far():
  # same braking profile but the lead is 200 m out: nothing to react to yet
  cem = make_cem(v_lead=20.0, d_rel=200.0)
  assert settle(cem, 20.0, 20.0, make_sm(BRAKING_LEAD)) is False


def test_predicted_stopped_lead_triggers():
  cem = make_cem(v_lead=8.0, d_rel=20.0)
  toggles = SimpleNamespace(conditional_slower_lead=False, conditional_stopped_lead=True, lead_detection_probability=0.35)
  assert settle(cem, 10.0, 8.0, make_sm([8.0, 6.0, 3.0, 0.5, 0.0, 0.0]), toggles) is True


def test_no_lead_resets_everything():
  cem = make_cem(v_lead=15.0, d_rel=30.0)
  settle(cem, 25.0, 15.0, make_sm([15.0] * 6))
  cem.frogpilot_planner.tracking_lead = False
  cem.slow_lead(25.0, 15.0, make_sm([15.0] * 6), TOGGLES, True)
  assert cem.slow_lead_detected is False
  assert cem.kinematic_lead is False
  assert cem.slow_lead_filter.x == 0


def test_stop_light_detection_is_off_in_traffic_mode():
  cem = make_cem(v_lead=0.0, d_rel=100.0)
  cem.frogpilot_planner.tracking_lead = False
  cem.frogpilot_planner.model_stopped = True
  cem.frogpilot_planner.model_length = 0.0
  cem.stop_light_filter.x = 1.0
  cem.stop_light_detected = True
  cem.stop_sign_and_light(10.0, make_sm([0.0] * 6, traffic_mode=True), 2.0)
  assert cem.stop_light_detected is False
  assert cem.stop_light_filter.x == 0


@pytest.mark.parametrize("frames", [1, 3])
def test_filter_needs_persistence(frames):
  cem = make_cem(v_lead=15.0, d_rel=30.0)
  assert settle(cem, 25.0, 15.0, make_sm([15.0] * 6), frames=frames) is False
  assert cem.slow_lead_filter.x < THRESHOLD
  assert cem_module.KINEMATIC_LEAD_DECELERATION > cem_module.KINEMATIC_LEAD_HOLD
