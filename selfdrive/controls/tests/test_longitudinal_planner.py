from types import SimpleNamespace

from openpilot.selfdrive.controls.lib.longitudinal_planner import (
  get_experimental_free_road_boost_target,
  rate_limit_value,
  update_experimental_free_road_boost,
)


def make_lead(status=False, d_rel=0.0):
  return SimpleNamespace(status=status, dRel=d_rel)


def test_experimental_free_road_boost_disabled_for_close_lead():
  lead = make_lead(status=True, d_rel=25.0)
  boost = get_experimental_free_road_boost_target(
    mode='blended',
    allow_throttle=True,
    should_stop=False,
    lead=lead,
    v_ego=20.0,
    v_cruise=24.0,
    mpc_accel=0.7,
    e2e_accel=0.0,
    boost_gain=1.0,
  )
  assert boost == 0.0


def test_experimental_free_road_boost_allows_small_nudge_from_slight_model_decel():
  boost = get_experimental_free_road_boost_target(
    mode='blended',
    allow_throttle=True,
    should_stop=False,
    lead=make_lead(),
    v_ego=20.0,
    v_cruise=24.0,
    mpc_accel=0.7,
    e2e_accel=-0.05,
    boost_gain=1.0,
  )
  assert 0.0 < boost < 0.3


def test_experimental_free_road_boost_disabled_when_allow_throttle_false():
  boost = get_experimental_free_road_boost_target(
    mode='blended',
    allow_throttle=False,
    should_stop=False,
    lead=make_lead(),
    v_ego=20.0,
    v_cruise=24.0,
    mpc_accel=0.7,
    e2e_accel=0.0,
    boost_gain=1.0,
  )
  assert boost == 0.0


def test_experimental_free_road_boost_gain_zero_disables_assist():
  boost = get_experimental_free_road_boost_target(
    mode='blended',
    allow_throttle=True,
    should_stop=False,
    lead=make_lead(),
    v_ego=20.0,
    v_cruise=24.0,
    mpc_accel=0.7,
    e2e_accel=0.0,
    boost_gain=0.0,
  )
  assert boost == 0.0


def test_rate_limit_value_uses_separate_up_and_down_steps():
  assert rate_limit_value(0.0, 0.3, 0.03, 0.08) == 0.03
  assert rate_limit_value(0.3, 0.0, 0.03, 0.08) == 0.22


def test_experimental_free_road_boost_clears_immediately_for_close_lead():
  boost = update_experimental_free_road_boost(
    current_boost=0.18,
    mode='blended',
    allow_throttle=True,
    should_stop=False,
    lead=make_lead(status=True, d_rel=20.0),
    v_ego=20.0,
    v_cruise=24.0,
    mpc_accel=0.7,
    e2e_accel=0.0,
  )
  assert boost == 0.0


def test_experimental_free_road_boost_clears_immediately_for_stop_request():
  boost = update_experimental_free_road_boost(
    current_boost=0.18,
    mode='blended',
    allow_throttle=True,
    should_stop=True,
    lead=make_lead(),
    v_ego=20.0,
    v_cruise=24.0,
    mpc_accel=0.7,
    e2e_accel=0.0,
  )
  assert boost == 0.0
