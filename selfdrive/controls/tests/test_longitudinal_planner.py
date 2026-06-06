from types import SimpleNamespace

from openpilot.selfdrive.controls.lib.longitudinal_planner import (
  apply_force_coast_strength_brake_limit,
  apply_santa_fe_experimental_decelerating_lead_approach_cap,
  apply_santa_fe_experimental_lead_caution,
  apply_santa_fe_slowing_lead_smooth_approach_cap,
  apply_santa_fe_stopped_lead_smooth_approach_cap,
  apply_experimental_force_coast_cap,
  get_active_long_distance_factor,
  get_experimental_free_road_model_gate,
  get_experimental_free_road_boost_target,
  get_experimental_free_road_lead_gap_gate,
  get_experimental_free_road_lead_pullaway_gate,
  get_experimental_free_road_lead_speed_gate,
  get_experimental_free_road_lead_time_threshold,
  get_experimental_free_road_no_lead_speed_gate,
  get_santa_fe_experimental_decelerating_lead_approach_cap,
  get_santa_fe_experimental_lead_caution_decel,
  get_santa_fe_slowing_lead_smooth_approach_cap,
  get_santa_fe_stopped_lead_smooth_approach_cap,
  get_experimental_boosted_accel,
  rate_limit_value,
  update_experimental_free_road_boost,
)


def make_lead(status=False, d_rel=0.0, v_rel=0.0, v_lead=0.0, a_lead_k=0.0):
  return SimpleNamespace(status=status, dRel=d_rel, vRel=v_rel, vLead=v_lead, aLeadK=a_lead_k)


def test_experimental_boost_caps_only_the_added_accel():
  assert get_experimental_boosted_accel(0.1, 0.4, 0.5) == 0.4


def test_experimental_boost_never_reduces_native_experimental_accel():
  assert get_experimental_boosted_accel(0.8, 0.4, 0.2) == 0.8


def test_experimental_force_coast_cap_preserves_stronger_native_braking():
  assert apply_experimental_force_coast_cap(-0.8, -0.4, True) == -0.8


def test_experimental_force_coast_cap_matches_acc_reference_when_needed():
  assert apply_experimental_force_coast_cap(-0.1, -0.6, True) == -0.6


def test_force_coast_strength_limits_far_lead_acc_spike_to_selected_target():
  lead = make_lead(status=True, d_rel=65.3, v_rel=-4.73, v_lead=3.26, a_lead_k=0.32)

  adjusted = apply_force_coast_strength_brake_limit(
    output_a_target=-3.338,
    force_coast_target_accel=-1.2,
    force_coast=True,
    v_ego=7.96,
    lead=lead,
    output_should_stop=False,
    model_accel=-0.367,
  )

  assert adjusted == -1.2


def test_force_coast_strength_allows_stronger_model_brake_but_not_acc_spike():
  lead = make_lead(status=True, d_rel=65.3, v_rel=-4.73, v_lead=3.26, a_lead_k=0.32)

  adjusted = apply_force_coast_strength_brake_limit(
    output_a_target=-3.338,
    force_coast_target_accel=-1.2,
    force_coast=True,
    v_ego=7.96,
    lead=lead,
    output_should_stop=False,
    model_accel=-1.8,
  )

  assert adjusted == -1.8


def test_force_coast_strength_allows_close_lead_safety_brake():
  lead = make_lead(status=True, d_rel=8.0, v_rel=-3.0, v_lead=4.96, a_lead_k=-0.2)

  adjusted = apply_force_coast_strength_brake_limit(
    output_a_target=-3.338,
    force_coast_target_accel=-1.2,
    force_coast=True,
    v_ego=7.96,
    lead=lead,
    output_should_stop=False,
    model_accel=-0.367,
  )

  assert adjusted == -3.338


def test_long_distance_factor_is_weaker_on_leftmost_lane():
  toggles = SimpleNamespace(long_distance_factor=1.5, lane_detection_width=3.0)

  assert get_active_long_distance_factor(0.0, toggles) == 0.75
  assert get_active_long_distance_factor(2.5, toggles) == 0.75
  assert get_active_long_distance_factor(3.2, toggles) == 1.5


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


def test_experimental_free_road_no_lead_speed_gate_is_25_percent_stronger():
  assert get_experimental_free_road_no_lead_speed_gate(0.5) == 0.5
  assert get_experimental_free_road_no_lead_speed_gate(1.25) == 0.875
  assert get_experimental_free_road_no_lead_speed_gate(2.0) == 1.0


def test_experimental_free_road_lead_gap_gate_blocks_inside_desired_gap():
  close_gate = get_experimental_free_road_lead_gap_gate(make_lead(status=True, d_rel=6.0), 2.0)
  far_gate = get_experimental_free_road_lead_gap_gate(make_lead(status=True, d_rel=16.0), 2.0)
  assert close_gate == 0.0
  assert far_gate == 1.0


def test_experimental_free_road_lead_gap_gate_opens_beyond_desired_gap():
  close_gap_gate = get_experimental_free_road_lead_gap_gate(make_lead(status=True, d_rel=6.0), 3.0)
  far_gap_gate = get_experimental_free_road_lead_gap_gate(make_lead(status=True, d_rel=40.0), 20.0)
  assert close_gap_gate == 0.0
  assert far_gap_gate == 1.0


def test_experimental_free_road_lead_pullaway_gate_weakens_when_lead_stops_pulling():
  strong_pullaway = get_experimental_free_road_lead_pullaway_gate(make_lead(status=True, v_lead=5.0, a_lead_k=1.0), 2.0)
  weak_pullaway = get_experimental_free_road_lead_pullaway_gate(make_lead(status=True, v_lead=2.6, a_lead_k=0.0), 2.0)
  assert 0.0 <= weak_pullaway < strong_pullaway <= 1.0


def test_experimental_free_road_lead_pullaway_gate_fades_out_at_higher_speed():
  low_speed_gate = get_experimental_free_road_lead_pullaway_gate(make_lead(status=True, v_lead=8.5, a_lead_k=0.0), 8.0)
  high_speed_gate = get_experimental_free_road_lead_pullaway_gate(make_lead(status=True, v_lead=19.6, a_lead_k=0.0), 19.0)
  assert low_speed_gate < high_speed_gate <= 1.0


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
    lead=make_lead(status=True, d_rel=40.0, v_lead=23.0, a_lead_k=1.0),
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
    lead=make_lead(status=True, d_rel=10.0, v_lead=3.5, a_lead_k=1.0),
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
    lead=make_lead(status=True, d_rel=80.0, v_lead=23.0, a_lead_k=1.0),
    v_ego=20.0,
    v_cruise=28.0,
    experimental_base_accel=0.0,
    acc_reference_accel=1.0,
    e2e_accel=0.1,
    lead_boost_gain=1.0,
    no_lead_boost_gain=0.5,
  )
  assert 0.0 < low_speed_boost < high_speed_boost


def test_experimental_free_road_lead_boost_fades_when_lead_stops_pulling_away():
  strong_pullaway_boost = get_experimental_free_road_boost_target(
    mode='blended',
    allow_throttle=True,
    should_stop=False,
    force_coast=False,
    lead=make_lead(status=True, d_rel=14.0, v_lead=8.0, a_lead_k=1.0),
    v_ego=6.0,
    v_cruise=12.0,
    experimental_base_accel=0.2,
    acc_reference_accel=1.2,
    e2e_accel=0.6,
    lead_boost_gain=1.0,
    no_lead_boost_gain=0.5,
  )
  weak_pullaway_boost = get_experimental_free_road_boost_target(
    mode='blended',
    allow_throttle=True,
    should_stop=False,
    force_coast=False,
    lead=make_lead(status=True, d_rel=14.0, v_lead=6.3, a_lead_k=0.0),
    v_ego=6.0,
    v_cruise=12.0,
    experimental_base_accel=0.2,
    acc_reference_accel=1.2,
    e2e_accel=0.6,
    lead_boost_gain=1.0,
    no_lead_boost_gain=0.5,
  )
  assert weak_pullaway_boost < strong_pullaway_boost


def test_experimental_free_road_lead_boost_uses_gap_gate_for_stop_and_go_lead():
  close_lead_boost = get_experimental_free_road_boost_target(
    mode='blended',
    allow_throttle=True,
    should_stop=False,
    force_coast=False,
    lead=make_lead(status=True, d_rel=6.0, v_lead=7.0, a_lead_k=1.0),
    v_ego=4.0,
    v_cruise=12.0,
    experimental_base_accel=0.2,
    acc_reference_accel=1.2,
    e2e_accel=0.6,
    lead_boost_gain=1.0,
    no_lead_boost_gain=0.5,
  )
  far_lead_boost = get_experimental_free_road_boost_target(
    mode='blended',
    allow_throttle=True,
    should_stop=False,
    force_coast=False,
    lead=make_lead(status=True, d_rel=16.0, v_lead=7.0, a_lead_k=1.0),
    v_ego=4.0,
    v_cruise=12.0,
    experimental_base_accel=0.2,
    acc_reference_accel=1.2,
    e2e_accel=0.6,
    lead_boost_gain=1.0,
    no_lead_boost_gain=0.5,
  )
  assert close_lead_boost == 0.0
  assert far_lead_boost > 0.0


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
    lead=make_lead(status=True, d_rel=60.0, v_lead=23.0, a_lead_k=1.0),
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
    lead=make_lead(status=True, d_rel=60.0, v_lead=23.0, a_lead_k=1.0),
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
    lead=make_lead(status=True, d_rel=10.4, v_rel=-5.21, v_lead=0.23),
  )
  extra_decel = output_a_target - adjusted
  assert adjusted < output_a_target
  assert 0.02 < extra_decel < 0.15


def test_santa_fe_decelerating_lead_approach_cap_bookmarked_segment_40_seed():
  output_a_target = -0.16
  lead = make_lead(status=True, d_rel=37.16, v_rel=-3.20, v_lead=10.63, a_lead_k=-0.45)

  cap = get_santa_fe_experimental_decelerating_lead_approach_cap(v_ego=13.82, lead=lead)
  adjusted = apply_santa_fe_experimental_decelerating_lead_approach_cap(output_a_target, v_ego=13.82, lead=lead)

  assert cap is not None
  assert -0.45 < cap < -0.20
  assert adjusted == cap


def test_santa_fe_decelerating_lead_approach_cap_does_not_deepen_existing_strong_brake():
  output_a_target = -1.89
  lead = make_lead(status=True, d_rel=21.80, v_rel=-3.57, v_lead=5.73, a_lead_k=-1.22)

  adjusted = apply_santa_fe_experimental_decelerating_lead_approach_cap(output_a_target, v_ego=9.36, lead=lead)

  assert adjusted == output_a_target


def test_santa_fe_stopped_lead_smooth_approach_cap_strengthens_latest_bookmark_early_approach():
  lead = make_lead(status=True, d_rel=21.60, v_rel=-9.21, v_lead=0.0, a_lead_k=0.0)

  cap = get_santa_fe_stopped_lead_smooth_approach_cap(v_ego=9.21, lead=lead)
  adjusted = apply_santa_fe_stopped_lead_smooth_approach_cap(-1.60, v_ego=9.21, lead=lead)

  assert cap is not None
  assert -2.30 < cap < -2.00
  assert adjusted == cap


def test_santa_fe_slowing_lead_smooth_approach_cap_brakes_earlier_for_decelerating_lead():
  lead = make_lead(status=True, d_rel=24.20, v_rel=-1.58, v_lead=10.01, a_lead_k=-1.21)

  cap = get_santa_fe_slowing_lead_smooth_approach_cap(v_ego=11.51, lead=lead)
  adjusted = apply_santa_fe_slowing_lead_smooth_approach_cap(-0.55, v_ego=11.51, lead=lead)

  assert cap is not None
  assert -1.20 < cap < -0.95
  assert adjusted == cap


def test_santa_fe_slowing_lead_smooth_approach_cap_brakes_earlier_for_high_speed_queue():
  lead = make_lead(status=True, d_rel=74.60, v_rel=-9.51, v_lead=3.88, a_lead_k=-1.85)

  cap = get_santa_fe_slowing_lead_smooth_approach_cap(v_ego=13.39, lead=lead)
  adjusted = apply_santa_fe_slowing_lead_smooth_approach_cap(-0.20, v_ego=13.39, lead=lead)

  assert cap is not None
  assert -1.40 < cap < -1.10
  assert adjusted == cap


def test_santa_fe_slowing_lead_smooth_approach_cap_ignores_far_slow_ttc_lead():
  lead = make_lead(status=True, d_rel=96.90, v_rel=-2.63, v_lead=8.83, a_lead_k=-1.66)

  cap = get_santa_fe_slowing_lead_smooth_approach_cap(v_ego=11.57, lead=lead)

  assert cap is None


def test_santa_fe_slowing_lead_smooth_approach_cap_ignores_steady_moving_lead():
  lead = make_lead(status=True, d_rel=24.20, v_rel=-1.58, v_lead=10.01, a_lead_k=-0.20)

  cap = get_santa_fe_slowing_lead_smooth_approach_cap(v_ego=11.51, lead=lead)
  adjusted = apply_santa_fe_slowing_lead_smooth_approach_cap(-0.55, v_ego=11.51, lead=lead)

  assert cap is None
  assert adjusted == -0.55


def test_santa_fe_slowing_lead_smooth_approach_cap_does_not_deepen_existing_strong_brake():
  lead = make_lead(status=True, d_rel=21.60, v_rel=-3.48, v_lead=5.73, a_lead_k=-1.22)

  adjusted = apply_santa_fe_slowing_lead_smooth_approach_cap(-1.60, v_ego=9.21, lead=lead)

  assert adjusted == -1.60


def test_santa_fe_stopped_lead_smooth_approach_cap_ignores_moving_lead():
  lead = make_lead(status=True, d_rel=21.60, v_rel=-3.48, v_lead=5.73, a_lead_k=-1.22)

  cap = get_santa_fe_stopped_lead_smooth_approach_cap(v_ego=9.21, lead=lead)
  adjusted = apply_santa_fe_stopped_lead_smooth_approach_cap(-1.60, v_ego=9.21, lead=lead)

  assert cap is None
  assert adjusted == -1.60


def test_santa_fe_stopped_lead_smooth_approach_cap_does_not_deepen_already_strong_brake():
  lead = make_lead(status=True, d_rel=21.60, v_rel=-9.21, v_lead=0.0, a_lead_k=0.0)

  adjusted = apply_santa_fe_stopped_lead_smooth_approach_cap(-2.40, v_ego=9.21, lead=lead)

  assert adjusted == -2.40


def test_santa_fe_decelerating_lead_approach_cap_cuts_wide_closing_gap_accel_to_coast():
  output_a_target = 0.43
  lead = make_lead(status=True, d_rel=44.78, v_rel=-2.01, v_lead=11.21, a_lead_k=0.00)

  cap = get_santa_fe_experimental_decelerating_lead_approach_cap(v_ego=13.25, lead=lead)
  adjusted = apply_santa_fe_experimental_decelerating_lead_approach_cap(output_a_target, v_ego=13.25, lead=lead)

  assert cap is not None
  assert -0.08 < cap < 0.02
  assert adjusted == cap


def test_santa_fe_decelerating_lead_approach_cap_ignores_far_steady_lead():
  lead = make_lead(status=True, d_rel=60.0, v_rel=-2.01, v_lead=11.21, a_lead_k=0.00)

  cap = get_santa_fe_experimental_decelerating_lead_approach_cap(v_ego=13.25, lead=lead)

  assert cap is None


def test_santa_fe_experimental_lead_caution_tapers_out_for_low_speed_moving_lead():
  output_a_target = -0.70
  adjusted = apply_santa_fe_experimental_lead_caution(
    output_a_target,
    v_ego=3.43,
    lead=make_lead(status=True, d_rel=5.50, v_rel=-0.76, v_lead=2.67),
  )
  assert adjusted == output_a_target


def test_santa_fe_experimental_lead_caution_tapers_out_for_moving_lead():
  output_a_target = -0.70
  adjusted = apply_santa_fe_experimental_lead_caution(
    output_a_target,
    v_ego=6.50,
    lead=make_lead(status=True, d_rel=11.0, v_rel=-1.0, v_lead=5.50),
  )
  assert adjusted == output_a_target


def test_santa_fe_experimental_lead_caution_tapers_out_for_lead_not_nearly_stopped():
  output_a_target = -1.00
  adjusted = apply_santa_fe_experimental_lead_caution(
    output_a_target,
    v_ego=6.00,
    lead=make_lead(status=True, d_rel=12.0, v_rel=-5.0, v_lead=1.00),
  )
  assert adjusted == output_a_target


def test_santa_fe_experimental_lead_caution_fades_out_once_lead_is_clearly_recovering():
  output_a_target = 0.66
  adjusted = apply_santa_fe_experimental_lead_caution(
    output_a_target,
    v_ego=3.42,
    lead=make_lead(status=True, d_rel=6.19, v_rel=1.28, v_lead=4.70),
  )
  assert adjusted == output_a_target


def test_santa_fe_experimental_lead_caution_is_zero_without_lead():
  assert get_santa_fe_experimental_lead_caution_decel(5.0, make_lead(), -1.0) == 0.0


def test_santa_fe_experimental_lead_caution_is_disabled_at_highway_speed():
  adjusted = apply_santa_fe_experimental_lead_caution(
    -1.90,
    v_ego=20.0,
    lead=make_lead(status=True, d_rel=30.0, v_rel=-6.0, v_lead=14.0),
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
