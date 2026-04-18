from types import SimpleNamespace

from openpilot.selfdrive.controls.lib.longitudinal_planner import (
  apply_santa_fe_experimental_lead_caution,
  apply_experimental_force_coast_cap,
  get_experimental_free_road_model_gate,
  get_experimental_free_road_boost_target,
  get_experimental_free_road_lead_speed_gate,
  get_experimental_free_road_lead_time_threshold,
  get_santa_fe_experimental_lead_caution_decel,
  get_experimental_boosted_accel,
  rate_limit_value,
  update_experimental_free_road_boost,
)


def make_lead(status=False, d_rel=0.0, v_rel=0.0):
  return SimpleNamespace(status=status, dRel=d_rel, vRel=v_rel)


def test_experimental_boost_caps_only_the_added_accel():
  assert get_experimental_boosted_accel(0.1, 0.4, 0.5) == 0.4


def test_experimental_boost_never_reduces_native_experimental_accel():
  assert get_experimental_boosted_accel(0.8, 0.4, 0.2) == 0.8


def test_experimental_force_coast_cap_preserves_stronger_native_braking():
  assert apply_experimental_force_coast_cap(-0.8, -0.4, True) == -0.8


def test_experimental_force_coast_cap_matches_acc_reference_when_needed():
  assert apply_experimental_force_coast_cap(-0.1, -0.6, True) == -0.6


def test_experimental_free_road_model_gate_weakens_slight_brake_boost():
  assert get_experimental_free_road_model_gate(-0.05, -0.2) < get_experimental_free_road_model_gate(0.0, -0.2)
  assert get_experimental_free_road_model_gate(-0.05, -0.2) < 0.5


def test_experimental_free_road_model_gate_respects_configured_cutoff():
  assert get_experimental_free_road_model_gate(-0.2, -0.2) == 0.0
  assert get_experimental_free_road_model_gate(-0.2, -0.35) > 0.0


def test_experimental_free_road_lead_time_threshold_relaxes_with_speed():
  assert get_experimental_free_road_lead_time_threshold(0.0) == 2.5
  assert get_experimental_free_road_lead_time_threshold(20.0) == 1.4


def test_experimental_free_road_lead_speed_gate_increases_with_speed():
  assert get_experimental_free_road_lead_speed_gate(0.0) == 0.25
  assert get_experimental_free_road_lead_speed_gate(20.0) == 1.0


def test_experimental_free_road_boost_disabled_for_close_lead():
  lead = make_lead(status=True, d_rel=25.0)
  boost = get_experimental_free_road_boost_target(
    mode='blended',
    allow_throttle=True,
    should_stop=False,
    force_coast=False,
    lead=lead,
    v_ego=20.0,
    v_cruise=24.0,
    experimental_base_accel=0.0,
    acc_reference_accel=0.7,
    e2e_accel=0.0,
    lead_boost_gain=1.0,
    no_lead_boost_gain=0.5,
  )
  assert boost == 0.0


def test_experimental_free_road_boost_disabled_for_low_speed_stop_and_go_lead():
  lead = make_lead(status=True, d_rel=5.0)
  boost = get_experimental_free_road_boost_target(
    mode='blended',
    allow_throttle=True,
    should_stop=False,
    force_coast=False,
    lead=lead,
    v_ego=3.0,
    v_cruise=9.0,
    experimental_base_accel=0.0,
    acc_reference_accel=0.7,
    e2e_accel=0.0,
    lead_boost_gain=1.0,
    no_lead_boost_gain=0.5,
  )
  assert boost == 0.0


def test_experimental_free_road_boost_allows_small_nudge_from_slight_model_decel():
  boost = get_experimental_free_road_boost_target(
    mode='blended',
    allow_throttle=True,
    should_stop=False,
    force_coast=False,
    lead=make_lead(),
    v_ego=20.0,
    v_cruise=24.0,
    experimental_base_accel=-0.1,
    acc_reference_accel=0.2,
    e2e_accel=-0.05,
    lead_boost_gain=1.0,
    no_lead_boost_gain=1.0,
  )
  assert 0.0 < boost < 1.0


def test_experimental_free_road_boost_uses_acc_reference_more_directly_for_far_lead():
  boost = get_experimental_free_road_boost_target(
    mode='blended',
    allow_throttle=True,
    should_stop=False,
    force_coast=False,
    lead=make_lead(status=True, d_rel=40.0),
    v_ego=20.0,
    v_cruise=20.3,
    experimental_base_accel=-0.05,
    acc_reference_accel=0.25,
    e2e_accel=0.0,
    lead_boost_gain=1.0,
    no_lead_boost_gain=0.5,
  )
  assert boost > 0.2


def test_experimental_free_road_lead_boost_is_weaker_at_stop_and_go_speed():
  low_speed_boost = get_experimental_free_road_boost_target(
    mode='blended',
    allow_throttle=True,
    should_stop=False,
    force_coast=False,
    lead=make_lead(status=True, d_rel=10.0),
    v_ego=2.0,
    v_cruise=10.0,
    experimental_base_accel=0.0,
    acc_reference_accel=1.0,
    e2e_accel=0.1,
    lead_boost_gain=1.0,
    no_lead_boost_gain=0.5,
  )
  high_speed_boost = get_experimental_free_road_boost_target(
    mode='blended',
    allow_throttle=True,
    should_stop=False,
    force_coast=False,
    lead=make_lead(status=True, d_rel=80.0),
    v_ego=20.0,
    v_cruise=28.0,
    experimental_base_accel=0.0,
    acc_reference_accel=1.0,
    e2e_accel=0.1,
    lead_boost_gain=1.0,
    no_lead_boost_gain=0.5,
  )
  assert 0.0 < low_speed_boost < high_speed_boost


def test_experimental_free_road_boost_uses_less_headroom_without_lead():
  no_lead_boost = get_experimental_free_road_boost_target(
    mode='blended',
    allow_throttle=True,
    should_stop=False,
    force_coast=False,
    lead=make_lead(),
    v_ego=20.0,
    v_cruise=30.0,
    experimental_base_accel=0.0,
    acc_reference_accel=2.0,
    e2e_accel=0.2,
    lead_boost_gain=1.0,
    no_lead_boost_gain=1.0,
  )
  lead_boost = get_experimental_free_road_boost_target(
    mode='blended',
    allow_throttle=True,
    should_stop=False,
    force_coast=False,
    lead=make_lead(status=True, d_rel=60.0),
    v_ego=20.0,
    v_cruise=30.0,
    experimental_base_accel=0.0,
    acc_reference_accel=2.0,
    e2e_accel=0.2,
    lead_boost_gain=1.0,
    no_lead_boost_gain=1.0,
  )
  assert no_lead_boost < lead_boost


def test_experimental_free_road_lead_boost_has_extra_headroom():
  boost = get_experimental_free_road_boost_target(
    mode='blended',
    allow_throttle=True,
    should_stop=False,
    force_coast=False,
    lead=make_lead(status=True, d_rel=60.0),
    v_ego=20.0,
    v_cruise=30.0,
    experimental_base_accel=0.0,
    acc_reference_accel=2.0,
    e2e_accel=0.0,
    lead_boost_gain=2.0,
    no_lead_boost_gain=0.5,
  )
  assert boost > 1.0


def test_experimental_free_road_boost_fades_out_for_stronger_model_brake_request():
  boost = get_experimental_free_road_boost_target(
    mode='blended',
    allow_throttle=True,
    should_stop=False,
    force_coast=False,
    lead=make_lead(),
    v_ego=20.0,
    v_cruise=24.0,
    experimental_base_accel=-0.2,
    acc_reference_accel=0.2,
    e2e_accel=-0.4,
    lead_boost_gain=2.0,
    no_lead_boost_gain=0.5,
  )
  assert boost == 0.0


def test_experimental_free_road_boost_uses_less_assist_for_slight_brake_with_higher_cutoff():
  lower_cutoff_boost = get_experimental_free_road_boost_target(
    mode='blended',
    allow_throttle=True,
    should_stop=False,
    force_coast=False,
    lead=make_lead(),
    v_ego=20.0,
    v_cruise=24.0,
    experimental_base_accel=-0.1,
    acc_reference_accel=0.4,
    e2e_accel=-0.05,
    lead_boost_gain=1.0,
    no_lead_boost_gain=1.0,
    brake_cutoff=-0.35,
  )
  higher_cutoff_boost = get_experimental_free_road_boost_target(
    mode='blended',
    allow_throttle=True,
    should_stop=False,
    force_coast=False,
    lead=make_lead(),
    v_ego=20.0,
    v_cruise=24.0,
    experimental_base_accel=-0.1,
    acc_reference_accel=0.4,
    e2e_accel=-0.05,
    lead_boost_gain=1.0,
    no_lead_boost_gain=1.0,
    brake_cutoff=-0.2,
  )
  assert higher_cutoff_boost < lower_cutoff_boost


def test_santa_fe_experimental_lead_caution_adds_only_gentle_extra_decel_for_fast_closing_lead():
  output_a_target = -1.90
  adjusted = apply_santa_fe_experimental_lead_caution(
    output_a_target,
    v_ego=5.44,
    lead=make_lead(status=True, d_rel=10.4, v_rel=-5.21),
  )
  assert adjusted < output_a_target
  assert adjusted > -2.30


def test_santa_fe_experimental_lead_caution_holds_some_brake_while_gap_is_still_marginal():
  output_a_target = -0.70
  adjusted = apply_santa_fe_experimental_lead_caution(
    output_a_target,
    v_ego=3.43,
    lead=make_lead(status=True, d_rel=5.50, v_rel=-0.76),
  )
  assert adjusted < output_a_target
  assert adjusted > -1.00


def test_santa_fe_experimental_lead_caution_fades_out_once_lead_is_clearly_recovering():
  output_a_target = 0.66
  adjusted = apply_santa_fe_experimental_lead_caution(
    output_a_target,
    v_ego=3.42,
    lead=make_lead(status=True, d_rel=6.19, v_rel=1.28),
  )
  assert adjusted == output_a_target


def test_santa_fe_experimental_lead_caution_is_zero_without_lead():
  assert get_santa_fe_experimental_lead_caution_decel(5.0, make_lead(), -1.0) == 0.0


def test_santa_fe_experimental_lead_caution_is_disabled_at_highway_speed():
  adjusted = apply_santa_fe_experimental_lead_caution(
    -1.90,
    v_ego=20.0,
    lead=make_lead(status=True, d_rel=30.0, v_rel=-6.0),
  )
  assert adjusted == -1.90


def test_experimental_free_road_boost_disabled_when_allow_throttle_false():
  boost = get_experimental_free_road_boost_target(
    mode='blended',
    allow_throttle=False,
    should_stop=False,
    force_coast=False,
    lead=make_lead(),
    v_ego=20.0,
    v_cruise=24.0,
    experimental_base_accel=0.0,
    acc_reference_accel=0.7,
    e2e_accel=0.0,
    lead_boost_gain=1.0,
    no_lead_boost_gain=0.5,
  )
  assert boost == 0.0


def test_experimental_free_road_boost_gain_zero_disables_assist():
  boost = get_experimental_free_road_boost_target(
    mode='blended',
    allow_throttle=True,
    should_stop=False,
    force_coast=False,
    lead=make_lead(),
    v_ego=20.0,
    v_cruise=24.0,
    experimental_base_accel=0.0,
    acc_reference_accel=0.7,
    e2e_accel=0.0,
    lead_boost_gain=0.0,
    no_lead_boost_gain=0.0,
  )
  assert boost == 0.0


def test_experimental_free_road_boost_disabled_when_force_coast_active():
  boost = get_experimental_free_road_boost_target(
    mode='blended',
    allow_throttle=True,
    should_stop=False,
    force_coast=True,
    lead=make_lead(),
    v_ego=20.0,
    v_cruise=24.0,
    experimental_base_accel=0.0,
    acc_reference_accel=0.7,
    e2e_accel=0.0,
    lead_boost_gain=1.0,
    no_lead_boost_gain=0.5,
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
    force_coast=False,
    lead=make_lead(status=True, d_rel=20.0),
    v_ego=20.0,
    v_cruise=24.0,
    experimental_base_accel=0.0,
    acc_reference_accel=0.7,
    e2e_accel=0.0,
    lead_boost_gain=1.0,
    no_lead_boost_gain=0.5,
  )
  assert boost == 0.0


def test_experimental_free_road_boost_clears_immediately_for_stop_request():
  boost = update_experimental_free_road_boost(
    current_boost=0.18,
    mode='blended',
    allow_throttle=True,
    should_stop=True,
    force_coast=False,
    lead=make_lead(),
    v_ego=20.0,
    v_cruise=24.0,
    experimental_base_accel=0.0,
    acc_reference_accel=0.7,
    e2e_accel=0.0,
    lead_boost_gain=1.0,
    no_lead_boost_gain=0.5,
  )
  assert boost == 0.0


def test_experimental_free_road_boost_clears_immediately_for_force_coast():
  boost = update_experimental_free_road_boost(
    current_boost=0.18,
    mode='blended',
    allow_throttle=True,
    should_stop=False,
    force_coast=True,
    lead=make_lead(),
    v_ego=20.0,
    v_cruise=24.0,
    experimental_base_accel=0.0,
    acc_reference_accel=0.7,
    e2e_accel=0.0,
    lead_boost_gain=1.0,
    no_lead_boost_gain=0.5,
  )
  assert boost == 0.0
