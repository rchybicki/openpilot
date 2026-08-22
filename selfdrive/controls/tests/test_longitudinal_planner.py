from types import SimpleNamespace

import math

import numpy as np
import pytest

from cereal import log

from openpilot.common.realtime import DT_MDL

from openpilot.selfdrive.controls.lib import stopping_flags
from openpilot.selfdrive.controls.lib.drive_helpers import update_should_stop_falling_edge_hold
from openpilot.selfdrive.controls.lib.longitudinal_planner import (
  SANTA_FE_STOP_AIM_CAP,
  apply_santa_fe_rest_close_reference_floor,
  get_santa_fe_rest_close_floor_v,
  update_santa_fe_rest_close_state,
  get_santa_fe_rest_close_ok,
  get_santa_fe_rest_close_epoch_evidence,
  update_santa_fe_rest_close_drive_frames,
  santa_fe_rest_close_drive_reopen,
  santa_fe_rest_close_consume_reopen,
  get_santa_fe_stop_aim_floor,
  get_santa_fe_stop_floor_demands,
  apply_force_coast_strength_brake_limit,
  apply_santa_fe_experimental_decelerating_lead_approach_cap,
  apply_santa_fe_experimental_lead_caution,
  apply_santa_fe_downhill_high_speed_stopped_lead_smooth_approach_cap,
  apply_santa_fe_slowing_lead_smooth_approach_cap,
  apply_santa_fe_stopped_lead_smooth_approach_cap,
  apply_experimental_force_coast_cap,
  get_active_long_distance_factor,
  get_experimental_free_road_model_gate,
  get_experimental_free_road_native_accel_gate,
  get_experimental_free_road_boost_target,
  get_experimental_free_road_lead_gap_gate,
  get_experimental_free_road_lead_pullaway_gate,
  get_experimental_free_road_lead_speed_gate,
  get_experimental_free_road_lead_time_threshold,
  get_experimental_free_road_no_lead_speed_gate,
  get_santa_fe_experimental_decelerating_lead_approach_cap,
  get_santa_fe_experimental_lead_caution_decel,
  get_santa_fe_downhill_high_speed_stopped_lead_smooth_approach_cap,
  get_santa_fe_downhill_queue_min_accel_clip_step,
  get_santa_fe_slowing_lead_smooth_approach_cap,
  get_santa_fe_stop_commit_floor,
  get_santa_fe_stop_commit_radar_min_acquire_d_rel,
  get_santa_fe_stop_commit_required_decel,
  get_santa_fe_stopped_lead_smooth_approach_cap,
  santa_fe_stop_commit_lead_state_ok,
  santa_fe_stop_commit_track_provenance_ok,
  SANTA_FE_EXPERIMENTAL_DECEL_LEAD_FEEDFORWARD_PERSIST_FRAMES,
  SANTA_FE_STOP_COMMIT_A_MAX,
  SANTA_FE_STOP_COMMIT_PERSIST_FRAMES,
  SANTA_FE_EXPERIMENTAL_DECEL_LEAD_FEEDFORWARD_SLEW_DOWN,
  SANTA_FE_EXPERIMENTAL_DECEL_LEAD_FEEDFORWARD_SLEW_UP,
  get_santa_fe_experimental_decelerating_lead_feedforward_target,
  slew_santa_fe_experimental_decelerating_lead_feedforward_authority,
  advance_santa_fe_experimental_decelerating_lead_feedforward_lane,
  update_santa_fe_experimental_decelerating_lead_persistence,
  update_santa_fe_stop_commit_track_certificate,
  update_santa_fe_stop_commit_persistence,
  get_experimental_boosted_accel,
  rate_limit_value,
  update_experimental_free_road_boost,
)


def make_lead(status=False, d_rel=0.0, v_rel=0.0, v_lead=0.0, a_lead_k=0.0, radar_track_id=-1, model_prob=1.0):
  return SimpleNamespace(status=status, dRel=d_rel, vRel=v_rel, vLead=v_lead, aLeadK=a_lead_k,
                         radarTrackId=radar_track_id, modelProb=model_prob)


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


def test_force_coast_strength_limits_far_fast_closing_lead_spike_to_selected_target():
  lead = make_lead(status=True, d_rel=41.7, v_rel=-10.3, v_lead=0.0, a_lead_k=0.0)

  adjusted = apply_force_coast_strength_brake_limit(
    output_a_target=-3.5,
    force_coast_target_accel=-1.2,
    force_coast=True,
    v_ego=9.93,
    lead=lead,
    output_should_stop=False,
    model_accel=-1.11,
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


def test_force_coast_strength_allows_urgent_closer_lead_brake():
  lead = make_lead(status=True, d_rel=22.0, v_rel=-8.0, v_lead=2.0, a_lead_k=-0.2)

  adjusted = apply_force_coast_strength_brake_limit(
    output_a_target=-3.338,
    force_coast_target_accel=-1.2,
    force_coast=True,
    v_ego=10.0,
    lead=lead,
    output_should_stop=False,
    model_accel=-0.367,
  )

  assert adjusted == -3.338


def test_force_coast_should_stop_does_not_bypass_cap_for_distant_slow_lead():
  # route 00001756 incident: a distant (12.3 m), slow (0.28 m/s closing) lead while ~stopped set
  # output_should_stop=True and drove a -1.70 spike through the old unconditional should_stop bypass.
  # The lead is NOT kinematically urgent, so the gentle force-coast cap (-0.7) must apply.
  lead = make_lead(status=True, d_rel=12.3, v_rel=-0.18, v_lead=0.23, a_lead_k=0.0)

  adjusted = apply_force_coast_strength_brake_limit(
    output_a_target=-1.70,
    force_coast_target_accel=-0.7,
    force_coast=True,
    v_ego=0.05,
    lead=lead,
    output_should_stop=True,
    model_accel=None,
  )

  assert adjusted == -0.7


def test_force_coast_should_stop_still_allows_close_stopped_lead_brake():
  # SAFETY (P1 no-under-braking): a genuinely close stopped lead while ~stopped must KEEP full MPC
  # brake authority even though output_should_stop=True -- the fix is never lead-blind.
  lead = make_lead(status=True, d_rel=3.0, v_rel=0.0, v_lead=0.0, a_lead_k=0.0)

  adjusted = apply_force_coast_strength_brake_limit(
    output_a_target=-2.0,
    force_coast_target_accel=-0.7,
    force_coast=True,
    v_ego=0.04,
    lead=lead,
    output_should_stop=True,
    model_accel=None,
  )

  assert adjusted == -2.0


def test_force_coast_should_stop_no_lead_capped_to_gentle_target():
  # no lead + force-coast: the harsh-stop-with-no-lead the driver bookmarked must fall back to the
  # gentle force-coast cap rather than passing a hard MPC demand through.
  lead = make_lead(status=False)

  adjusted = apply_force_coast_strength_brake_limit(
    output_a_target=-1.70,
    force_coast_target_accel=-0.7,
    force_coast=True,
    v_ego=0.05,
    lead=lead,
    output_should_stop=True,
    model_accel=None,
  )

  assert adjusted == -0.7


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


def test_experimental_free_road_native_accel_gate_fades_added_boost_for_strong_native_accel():
  assert get_experimental_free_road_native_accel_gate(0.2) == 1.0
  assert get_experimental_free_road_native_accel_gate(0.4) == 0.5
  assert get_experimental_free_road_native_accel_gate(0.6) == 0.0


def test_experimental_free_road_lead_time_threshold_relaxes_with_speed():
  assert get_experimental_free_road_lead_time_threshold(0.0) == 2.5
  assert get_experimental_free_road_lead_time_threshold(20.0) == 1.4


def test_experimental_free_road_lead_speed_gate_increases_with_speed():
  assert get_experimental_free_road_lead_speed_gate(0.0) == 0.25
  assert get_experimental_free_road_lead_speed_gate(20.0) == 1.0


def test_experimental_free_road_no_lead_speed_gate_is_50_percent_stronger():
  assert abs(get_experimental_free_road_no_lead_speed_gate(0.5) - 0.6) < 1e-9
  assert get_experimental_free_road_no_lead_speed_gate(1.25) == 1.0
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


def test_experimental_free_road_standard_lead_gap_window_is_30_percent_shorter_than_relaxed():
  v_ego = 20.0
  relaxed_start = 28.0
  relaxed_fully_open = 32.0
  standard_start = relaxed_start * 0.7
  standard_fully_open = relaxed_fully_open * 0.7

  assert abs(get_experimental_free_road_lead_gap_gate(make_lead(status=True, d_rel=relaxed_start), v_ego,
                                                      log.LongitudinalPersonality.relaxed)) < 1e-9
  assert abs(get_experimental_free_road_lead_gap_gate(make_lead(status=True, d_rel=relaxed_fully_open), v_ego,
                                                      log.LongitudinalPersonality.relaxed) - 1.0) < 1e-9
  assert abs(get_experimental_free_road_lead_gap_gate(make_lead(status=True, d_rel=standard_start), v_ego,
                                                      log.LongitudinalPersonality.standard)) < 1e-9
  assert abs(get_experimental_free_road_lead_gap_gate(make_lead(status=True, d_rel=standard_fully_open), v_ego,
                                                      log.LongitudinalPersonality.standard) - 1.0) < 1e-9


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


def test_experimental_free_road_one_x_no_lead_boost_exceeds_legacy_two_x_near_set_speed():
  boost = get_experimental_free_road_boost_target(
    mode='blended',
    allow_throttle=True,
    should_stop=False,
    force_coast=False,
    lead=make_lead(),
    v_ego=20.0,
    v_cruise=20.5,
    experimental_base_accel=-0.2,
    acc_reference_accel=0.2,
    e2e_accel=0.0,
    lead_boost_gain=1.0,
    no_lead_boost_gain=1.0,
    brake_cutoff=-0.3,
  )
  # Legacy 2x saturated the 0.6 m/s² cap here and used a 0.5 near-target speed gate.
  legacy_two_x_boost = 0.6 * get_experimental_free_road_model_gate(0.0, -0.3) * 0.5
  assert boost > legacy_two_x_boost
  assert -0.2 + boost > 0.0


def test_experimental_free_road_boost_stays_off_for_strong_native_acceleration():
  boost = get_experimental_free_road_boost_target(
    mode='blended',
    allow_throttle=True,
    should_stop=False,
    force_coast=False,
    lead=make_lead(),
    v_ego=20.0,
    v_cruise=24.0,
    experimental_base_accel=0.6,
    acc_reference_accel=1.0,
    e2e_accel=0.6,
    lead_boost_gain=1.0,
    no_lead_boost_gain=2.0,
    brake_cutoff=-0.3,
  )
  assert boost == 0.0


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
  assert boost > 0.15


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
    lead=make_lead(status=True, d_rel=20.0, v_lead=8.0, a_lead_k=1.0),
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
    lead=make_lead(status=True, d_rel=20.0, v_lead=6.3, a_lead_k=0.0),
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
    e2e_accel=0.2,
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


def test_santa_fe_decelerating_lead_feedforward_anticipates_route_00001f70_brake():
  # seg46 shape: ~1.9 s initial gap, filtered aLeadK ramps while the native request plateaus
  native_request = -0.55
  d_rel = 25.0
  state = (None, [])
  outputs = []
  closing_speeds = []

  authority = 0.0
  for frame in range(26):
    fraction = frame / 25.0
    v_ego = 13.05 - 1.15 * fraction
    v_lead = 12.9 - 2.1 * fraction
    a_lead_k = 0.0 if frame < 2 else -0.31 - 0.39 * ((frame - 2) / 23.0)
    lead = make_lead(status=True, d_rel=d_rel, v_rel=v_lead - v_ego, v_lead=v_lead, a_lead_k=a_lead_k,
                     radar_track_id=7001, model_prob=0.99)
    state = update_santa_fe_experimental_decelerating_lead_persistence(*state, v_ego, lead)
    ff_target = get_santa_fe_experimental_decelerating_lead_feedforward_target(v_ego, lead, state[1])
    authority = slew_santa_fe_experimental_decelerating_lead_feedforward_authority(authority, ff_target)
    outputs.append(min(native_request, authority) if authority < -1e-3 else native_request)
    closing_speeds.append(v_ego - v_lead)
    d_rel -= max(v_ego - v_lead, 0.0) * 0.05

  engaged_frames = [frame for frame, output in enumerate(outputs) if output < native_request]
  assert engaged_frames
  assert closing_speeds[engaged_frames[0]] < 1.20
  assert all(-1.0 <= output <= native_request for output in outputs)
  assert -0.78 < outputs[-1] < -0.70
  # a deeper native/MPC demand always wins: the lane only min()s on top
  assert min(-1.05, authority) == -1.05


def test_santa_fe_decelerating_lead_feedforward_requires_dwell_and_restarts_for_new_track():
  native_request = -0.55
  lead = make_lead(status=True, d_rel=25.0, v_rel=-0.7, v_lead=12.3, a_lead_k=-0.9,
                   radar_track_id=7001, model_prob=0.99)
  state = update_santa_fe_experimental_decelerating_lead_persistence(None, [], 13.0, lead)
  assert get_santa_fe_experimental_decelerating_lead_feedforward_target(13.0, lead, state[1]) is None

  recovered = make_lead(status=True, d_rel=25.0, v_rel=-0.7, v_lead=12.3, a_lead_k=0.0,
                        radar_track_id=7001, model_prob=0.99)
  assert update_santa_fe_experimental_decelerating_lead_persistence(*state, 13.0, recovered) == (None, [])

  state = (None, [])
  for _ in range(SANTA_FE_EXPERIMENTAL_DECEL_LEAD_FEEDFORWARD_PERSIST_FRAMES):
    state = update_santa_fe_experimental_decelerating_lead_persistence(*state, 13.0, lead)
  ff_target = get_santa_fe_experimental_decelerating_lead_feedforward_target(13.0, lead, state[1])
  assert ff_target is not None and ff_target < native_request
  authority = 0.0
  for _ in range(10):  # the slewed authority engages within 0.5 s of the dwell completing
    authority = slew_santa_fe_experimental_decelerating_lead_feedforward_authority(authority, ff_target)
  assert authority < native_request

  cut_in = make_lead(status=True, d_rel=25.0, v_rel=-0.7, v_lead=12.3, a_lead_k=-0.9,
                     radar_track_id=7002, model_prob=0.99)
  state = update_santa_fe_experimental_decelerating_lead_persistence(*state, 13.0, cut_in)
  assert len(state[1]) == 1
  assert get_santa_fe_experimental_decelerating_lead_feedforward_target(13.0, cut_in, state[1]) is None


def test_santa_fe_decelerating_lead_feedforward_no_change_gates():
  native_request = -0.55
  cases = (
    (13.0, make_lead(status=True, d_rel=25.0, v_rel=0.0, v_lead=13.0, a_lead_k=-0.7, radar_track_id=1, model_prob=0.99)),
    (13.0, make_lead(status=True, d_rel=25.0, v_rel=0.3, v_lead=13.3, a_lead_k=-0.7, radar_track_id=1, model_prob=0.99)),
    (13.0, make_lead(status=True, d_rel=25.0, v_rel=-0.7, v_lead=12.3, a_lead_k=0.4, radar_track_id=1, model_prob=0.99)),
    (13.0, make_lead(status=True, d_rel=25.0, v_rel=-0.7, v_lead=12.3, a_lead_k=-0.7, radar_track_id=-1, model_prob=0.99)),
    (13.0, make_lead(status=True, d_rel=25.0, v_rel=-0.7, v_lead=12.3, a_lead_k=-0.7, radar_track_id=1, model_prob=0.0)),
    (16.1, make_lead(status=True, d_rel=30.0, v_rel=-0.7, v_lead=15.4, a_lead_k=-0.7, radar_track_id=1, model_prob=0.99)),
    (4.6, make_lead(status=True, d_rel=16.0, v_rel=-4.2, v_lead=0.4, a_lead_k=-0.9, radar_track_id=1, model_prob=0.99)),
  )

  for v_ego, lead in cases:
    state = (None, [])
    for _ in range(SANTA_FE_EXPERIMENTAL_DECEL_LEAD_FEEDFORWARD_PERSIST_FRAMES):
      state = update_santa_fe_experimental_decelerating_lead_persistence(*state, v_ego, lead)
    assert get_santa_fe_experimental_decelerating_lead_feedforward_target(v_ego, lead, state[1]) is None
    assert apply_santa_fe_experimental_decelerating_lead_approach_cap(native_request, v_ego, lead) == native_request


def test_stop_commit_floor_can_deepen_past_decelerating_lead_feedforward_cap():
  lead = make_lead(status=True, d_rel=14.0, v_rel=-6.0, v_lead=4.0, a_lead_k=-1.0,
                   radar_track_id=7001, model_prob=0.99)
  state = (None, [])
  for _ in range(SANTA_FE_EXPERIMENTAL_DECEL_LEAD_FEEDFORWARD_PERSIST_FRAMES):
    state = update_santa_fe_experimental_decelerating_lead_persistence(*state, 10.0, lead)

  cap = get_santa_fe_experimental_decelerating_lead_feedforward_target(10.0, lead, state[1])
  floor, active = get_santa_fe_stop_commit_floor(10.0, lead, cap, state[1], False)
  assert cap == -1.0
  assert floor is not None and active
  assert floor < cap
  assert min(cap, floor) == floor


def test_santa_fe_decelerating_lead_feedforward_authority_is_continuous_across_eviction():
  # END-REVIEW round 1 (medium): with window [-0.31, -2.4 x5], the raw target steps from -0.37 to
  # -1.0 the frame the shallow sample ages out. The APPLIED authority must slew (<= SLEW_DOWN*DT
  # per frame); assigning the raw target directly (slew removed) fails this.
  lead = make_lead(status=True, d_rel=22.0, v_rel=-0.9, v_lead=11.5, a_lead_k=-2.4,
                   radar_track_id=9001, model_prob=0.99)
  shallow = make_lead(status=True, d_rel=22.0, v_rel=-0.9, v_lead=11.5, a_lead_k=-0.31,
                      radar_track_id=9001, model_prob=0.99)
  state = update_santa_fe_experimental_decelerating_lead_persistence(None, [], 11.5 + 0.9, shallow)
  for _ in range(SANTA_FE_EXPERIMENTAL_DECEL_LEAD_FEEDFORWARD_PERSIST_FRAMES + 3):
    state = update_santa_fe_experimental_decelerating_lead_persistence(*state, 11.5 + 0.9, lead)
    if len(state[1]) == SANTA_FE_EXPERIMENTAL_DECEL_LEAD_FEEDFORWARD_PERSIST_FRAMES and max(state[1]) == -0.31:
      break
  authority = 0.0
  prev = authority
  max_step = 0.0
  for _ in range(30):
    state = update_santa_fe_experimental_decelerating_lead_persistence(*state, 11.5 + 0.9, lead)
    ff_target = get_santa_fe_experimental_decelerating_lead_feedforward_target(11.5 + 0.9, lead, state[1])
    authority = slew_santa_fe_experimental_decelerating_lead_feedforward_authority(authority, ff_target)
    max_step = max(max_step, prev - authority)
    prev = authority
  assert authority == -1.0  # converged to the bounded deep target
  assert max_step <= SANTA_FE_EXPERIMENTAL_DECEL_LEAD_FEEDFORWARD_SLEW_DOWN * DT_MDL + 1e-9, (
    f"eviction stepped the authority {max_step:.3f} in one frame")


def test_santa_fe_decelerating_lead_feedforward_releases_at_slew_rate_not_instantly():
  # WITHIN the blended mode a one-frame eligibility loss releases at the slew rate, not to zero
  authority = -0.9
  released = slew_santa_fe_experimental_decelerating_lead_feedforward_authority(authority, None)
  assert released == -0.9 + SANTA_FE_EXPERIMENTAL_DECEL_LEAD_FEEDFORWARD_SLEW_UP * DT_MDL
  # and re-engagement afterwards deepens at the bounded rate
  redeepen = slew_santa_fe_experimental_decelerating_lead_feedforward_authority(released, -1.0)
  assert released - redeepen <= SANTA_FE_EXPERIMENTAL_DECEL_LEAD_FEEDFORWARD_SLEW_DOWN * DT_MDL + 1e-9


def test_santa_fe_decelerating_lead_feedforward_mode_reentry_requires_fresh_dwell():
  # END-REVIEW rounds 2+3 (HIGH, then a fixture finding): the PRODUCTION lane-advance function
  # (both call sites use it) must hard-clear authority AND window on an ineligible (mode-edge)
  # frame, and re-entry must re-earn the full dwell before ANY authority -- stale reactivation
  # from the pre-exit window was the hazard. This drives advance_...lane itself: deleting the
  # hard-clear inside it (returning the state unchanged on ineligible frames) fails below.
  lead = make_lead(status=True, d_rel=22.0, v_rel=-0.9, v_lead=11.5, a_lead_k=-2.4,
                   radar_track_id=9001, model_prob=0.99)
  track_id, window, authority = None, [], 0.0
  # phase 1: eligible frames build deep authority
  for _ in range(SANTA_FE_EXPERIMENTAL_DECEL_LEAD_FEEDFORWARD_PERSIST_FRAMES + 20):
    track_id, window, authority = advance_santa_fe_experimental_decelerating_lead_feedforward_lane(
      True, track_id, window, authority, 12.4, lead)
  assert authority <= -0.9  # deep and active
  # phase 2: ONE ineligible frame (mode exit)
  track_id, window, authority = advance_santa_fe_experimental_decelerating_lead_feedforward_lane(
    False, track_id, window, authority)
  assert track_id is None and window == [] and authority == 0.0, "mode edge did not hard-clear the lane"
  # phase 3: immediate re-entry -- no authority until the dwell is fully re-earned, then
  # slew-bounded re-engagement from zero (never a stale deep snap)
  for frame in range(SANTA_FE_EXPERIMENTAL_DECEL_LEAD_FEEDFORWARD_PERSIST_FRAMES + 2):
    track_id, window, authority = advance_santa_fe_experimental_decelerating_lead_feedforward_lane(
      True, track_id, window, authority, 12.4, lead)
    if frame < SANTA_FE_EXPERIMENTAL_DECEL_LEAD_FEEDFORWARD_PERSIST_FRAMES - 1:
      assert authority == 0.0, f"stale authority re-applied at re-entry frame {frame}"
  assert authority < 0.0
  assert authority >= -(SANTA_FE_EXPERIMENTAL_DECEL_LEAD_FEEDFORWARD_SLEW_DOWN * DT_MDL) * 3 - 1e-9


def test_santa_fe_stopped_lead_smooth_approach_cap_strengthens_latest_bookmark_early_approach():
  lead = make_lead(status=True, d_rel=21.60, v_rel=-9.21, v_lead=0.0, a_lead_k=0.0)

  cap = get_santa_fe_stopped_lead_smooth_approach_cap(v_ego=9.21, lead=lead)
  adjusted = apply_santa_fe_stopped_lead_smooth_approach_cap(-1.60, v_ego=9.21, lead=lead)

  assert cap is not None
  assert -2.90 < cap < -2.75
  assert adjusted == cap


def test_santa_fe_stopped_lead_smooth_approach_cap_spends_speed_earlier_for_late_acquired_stopped_lead():
  lead = make_lead(status=True, d_rel=33.84, v_rel=-10.32, v_lead=0.0, a_lead_k=0.0)

  cap = get_santa_fe_stopped_lead_smooth_approach_cap(v_ego=10.32, lead=lead)
  adjusted = apply_santa_fe_stopped_lead_smooth_approach_cap(-1.44, v_ego=10.32, lead=lead)

  assert cap is not None
  # range shifted 2026-07-29: late-approach firmness 1.06 + mid-band buffer raise (00001f65 seg13
  # harsh arrival -- spend more speed early so the terminal machinery inherits less)
  assert -2.30 < cap < -2.15
  assert adjusted == cap


def test_santa_fe_stopped_lead_smooth_approach_cap_catches_high_speed_far_stopped_lead_bookmark_seed():
  lead = make_lead(status=True, d_rel=81.0, v_rel=-14.24, v_lead=0.0, a_lead_k=0.0)

  cap = get_santa_fe_stopped_lead_smooth_approach_cap(v_ego=14.24, lead=lead)
  adjusted = apply_santa_fe_stopped_lead_smooth_approach_cap(-0.75, v_ego=14.24, lead=lead)

  assert cap is not None
  # range shifted 2026-07-29: late-approach firmness 1.06 (see the late_acquired fixture note)
  assert -1.70 < cap < -1.55
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
  assert -1.65 < cap < -1.45
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


def test_santa_fe_slowing_lead_smooth_approach_cap_adds_queue_reserve_for_live_takeover_seed():
  lead = make_lead(status=True, d_rel=20.69, v_rel=-4.13, v_lead=6.09, a_lead_k=-1.77)

  cap = get_santa_fe_slowing_lead_smooth_approach_cap(v_ego=9.83, lead=lead)
  adjusted = apply_santa_fe_slowing_lead_smooth_approach_cap(-1.97, v_ego=9.83, lead=lead)

  assert cap is not None
  assert -2.20 < cap < -2.05
  assert adjusted == cap


def test_santa_fe_downhill_high_speed_stopped_lead_cap_brakes_earlier_for_live_bookmark_seed():
  lead = make_lead(status=True, d_rel=91.72, v_rel=-17.99, v_lead=-0.25, a_lead_k=-0.03)

  cap = get_santa_fe_downhill_high_speed_stopped_lead_smooth_approach_cap(v_ego=17.71, lead=lead, accel_coast=-0.02)
  adjusted = apply_santa_fe_downhill_high_speed_stopped_lead_smooth_approach_cap(-0.63, v_ego=17.71, lead=lead, accel_coast=-0.02)

  assert cap is not None
  assert -1.90 < cap < -1.70
  assert adjusted == cap


def test_santa_fe_downhill_high_speed_stopped_lead_cap_catches_milder_downhill_followup_seed():
  lead = make_lead(status=True, d_rel=66.03, v_rel=-14.11, v_lead=-0.05, a_lead_k=-0.05)

  cap = get_santa_fe_downhill_high_speed_stopped_lead_smooth_approach_cap(v_ego=14.07, lead=lead, accel_coast=-0.11)
  adjusted = apply_santa_fe_downhill_high_speed_stopped_lead_smooth_approach_cap(-0.41, v_ego=14.07, lead=lead, accel_coast=-0.11)

  assert cap is not None
  assert -1.75 < cap < -1.50
  assert adjusted == cap


def test_santa_fe_downhill_high_speed_stopped_lead_cap_stays_off_on_flat_road():
  lead = make_lead(status=True, d_rel=91.72, v_rel=-17.99, v_lead=-0.25, a_lead_k=-0.03)

  cap = get_santa_fe_downhill_high_speed_stopped_lead_smooth_approach_cap(v_ego=17.71, lead=lead, accel_coast=-0.30)
  adjusted = apply_santa_fe_downhill_high_speed_stopped_lead_smooth_approach_cap(-0.63, v_ego=17.71, lead=lead, accel_coast=-0.30)

  assert cap is None
  assert adjusted == -0.63


def test_santa_fe_downhill_high_speed_stopped_lead_cap_stays_off_for_far_stopped_lead():
  lead = make_lead(status=True, d_rel=160.0, v_rel=-17.99, v_lead=-0.25, a_lead_k=-0.03)

  cap = get_santa_fe_downhill_high_speed_stopped_lead_smooth_approach_cap(v_ego=17.71, lead=lead, accel_coast=-0.02)
  adjusted = apply_santa_fe_downhill_high_speed_stopped_lead_smooth_approach_cap(-0.63, v_ego=17.71, lead=lead, accel_coast=-0.02)

  assert cap is None
  assert adjusted == -0.63


def test_santa_fe_downhill_high_speed_stopped_lead_cap_stays_off_for_moving_lead():
  lead = make_lead(status=True, d_rel=91.72, v_rel=-13.71, v_lead=4.0, a_lead_k=-0.03)

  cap = get_santa_fe_downhill_high_speed_stopped_lead_smooth_approach_cap(v_ego=17.71, lead=lead, accel_coast=-0.02)
  adjusted = apply_santa_fe_downhill_high_speed_stopped_lead_smooth_approach_cap(-0.63, v_ego=17.71, lead=lead, accel_coast=-0.02)

  assert cap is None
  assert adjusted == -0.63


def test_santa_fe_downhill_queue_relaxes_min_accel_clip_for_high_speed_stopped_queue_seed():
  lead = make_lead(status=True, d_rel=91.72, v_rel=-17.99, v_lead=-0.25, a_lead_k=-0.03)

  step = get_santa_fe_downhill_queue_min_accel_clip_step(
    v_ego=17.71,
    lead=lead,
    accel_coast=-0.02,
    output_a_target=-1.79,
    prev_min_accel_clip=-0.63,
  )

  assert step == 0.12


def test_santa_fe_downhill_queue_relaxes_min_accel_clip_for_milder_downhill_followup_seed():
  lead = make_lead(status=True, d_rel=66.03, v_rel=-14.11, v_lead=-0.05, a_lead_k=-0.05)

  step = get_santa_fe_downhill_queue_min_accel_clip_step(
    v_ego=14.07,
    lead=lead,
    accel_coast=-0.11,
    output_a_target=-1.60,
    prev_min_accel_clip=-0.41,
  )

  assert step == 0.12


def test_santa_fe_downhill_queue_relaxes_min_accel_clip_for_latest_takeover_seed():
  lead = make_lead(status=True, d_rel=42.77, v_rel=-7.09, v_lead=4.50, a_lead_k=-1.94)

  step = get_santa_fe_downhill_queue_min_accel_clip_step(
    v_ego=11.56,
    lead=lead,
    accel_coast=0.06,
    output_a_target=-1.99,
    prev_min_accel_clip=-1.54,
  )

  assert step == 0.12


def test_santa_fe_downhill_queue_clip_relax_stays_off_on_flat_road():
  lead = make_lead(status=True, d_rel=42.77, v_rel=-7.09, v_lead=4.50, a_lead_k=-1.94)

  step = get_santa_fe_downhill_queue_min_accel_clip_step(
    v_ego=11.56,
    lead=lead,
    accel_coast=-0.30,
    output_a_target=-1.99,
    prev_min_accel_clip=-1.54,
  )

  assert step == 0.05


def test_santa_fe_downhill_queue_clip_relax_stays_off_for_steady_lead():
  lead = make_lead(status=True, d_rel=42.77, v_rel=0.20, v_lead=11.76, a_lead_k=-0.10)

  step = get_santa_fe_downhill_queue_min_accel_clip_step(
    v_ego=11.56,
    lead=lead,
    accel_coast=0.06,
    output_a_target=-1.99,
    prev_min_accel_clip=-1.54,
  )

  assert step == 0.05


def test_santa_fe_stopped_lead_smooth_approach_cap_ignores_moving_lead():
  lead = make_lead(status=True, d_rel=21.60, v_rel=-3.48, v_lead=5.73, a_lead_k=-1.22)

  cap = get_santa_fe_stopped_lead_smooth_approach_cap(v_ego=9.21, lead=lead)
  adjusted = apply_santa_fe_stopped_lead_smooth_approach_cap(-1.60, v_ego=9.21, lead=lead)

  assert cap is None
  assert adjusted == -1.60


def test_santa_fe_stopped_lead_smooth_approach_cap_does_not_deepen_already_strong_brake():
  lead = make_lead(status=True, d_rel=21.60, v_rel=-9.21, v_lead=0.0, a_lead_k=0.0)

  adjusted = apply_santa_fe_stopped_lead_smooth_approach_cap(-3.00, v_ego=9.21, lead=lead)

  assert adjusted == -3.00


def test_santa_fe_stopped_lead_smooth_approach_cap_keeps_far_stopped_lead_on_normal_table():
  lead = make_lead(status=True, d_rel=46.0, v_rel=-10.0, v_lead=0.0, a_lead_k=0.0)

  cap = get_santa_fe_stopped_lead_smooth_approach_cap(v_ego=10.0, lead=lead)
  adjusted = apply_santa_fe_stopped_lead_smooth_approach_cap(-0.80, v_ego=10.0, lead=lead)

  assert cap is not None
  assert -1.30 < cap < -1.15
  assert adjusted == cap


def test_santa_fe_stopped_lead_smooth_approach_cap_ignores_far_high_speed_stopped_lead():
  lead = make_lead(status=True, d_rel=125.0, v_rel=-14.24, v_lead=0.0, a_lead_k=0.0)

  cap = get_santa_fe_stopped_lead_smooth_approach_cap(v_ego=14.24, lead=lead)
  adjusted = apply_santa_fe_stopped_lead_smooth_approach_cap(-0.75, v_ego=14.24, lead=lead)

  assert cap is None
  assert adjusted == -0.75


def test_santa_fe_stopped_lead_smooth_approach_cap_ignores_high_speed_moving_lead():
  lead = make_lead(status=True, d_rel=81.0, v_rel=-4.24, v_lead=10.0, a_lead_k=0.0)

  cap = get_santa_fe_stopped_lead_smooth_approach_cap(v_ego=14.24, lead=lead)
  adjusted = apply_santa_fe_stopped_lead_smooth_approach_cap(-0.75, v_ego=14.24, lead=lead)

  assert cap is None
  assert adjusted == -0.75


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
  assert abs(rate_limit_value(0.3, 0.0, 0.03, 0.08) - 0.22) < 1e-9


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


# --- shouldStop falling-edge hold (stopping redesign §4.1) ---


def _run_should_stop_hold(frames, hold_s=0.4, dt=0.05, v_ego_stopping=0.3):
  timer = 0.0
  outputs = []
  for raw, v_now, a_target in frames:
    held, timer = update_should_stop_falling_edge_hold(raw, v_now, a_target, v_ego_stopping, timer, hold_s, dt)
    outputs.append(held)
  return outputs


def test_should_stop_falling_edge_hold_bridges_brief_planner_dropout():
  # raw flicker near standstill: 0.15 s dropout is bridged, the stop never deasserts
  frames = [(True, 0.1, -0.3)] * 4 + [(False, 0.1, -0.3)] * 3 + [(True, 0.1, -0.3)] * 4
  assert all(_run_should_stop_hold(frames))


def test_should_stop_falling_edge_hold_expires_after_hold_window():
  frames = [(True, 0.1, -0.3)] + [(False, 0.1, -0.3)] * 12
  outputs = _run_should_stop_hold(frames)
  assert outputs[1:9] == [True] * 8  # 0.4 s at the planner's 20 Hz
  assert outputs[9:] == [False] * 4


def test_should_stop_falling_edge_hold_releases_on_go_signal():
  assert _run_should_stop_hold([(True, 0.1, -0.3), (False, 0.1, 0.25)]) == [True, False]
  assert _run_should_stop_hold([(True, 0.1, -0.3), (False, 0.5, -0.3)]) == [True, False]


def test_should_stop_falling_edge_hold_kill_switch_restores_raw_flag():
  frames = [(True, 0.1, -0.3), (False, 0.1, -0.3), (True, 0.1, -0.3), (False, 0.1, -0.3)]
  assert _run_should_stop_hold(frames, hold_s=0.0) == [True, False, True, False]


def test_should_stop_falling_edge_hold_cannot_create_stops():
  frames = [(False, 0.05, -0.5)] * 20
  assert not any(_run_should_stop_hold(frames))


def test_should_stop_lookahead_ships_off():
  assert stopping_flags.SHOULD_STOP_LOOKAHEAD_S == 0.0
  assert stopping_flags.SHOULD_STOP_FALLING_EDGE_HOLD_S == 0.4


# --- ISD compensation in the Santa Fe approach caps (stopping redesign §4.2) ---


def test_stopped_lead_cap_invariant_across_publish_flag_for_same_true_gap(monkeypatch):
  # the stopped-lead cap's only dRel use is the compensated hold-gap term, so for the same
  # TRUE gap the cap must be identical in both flag states
  isd = 1.5
  true_d_rel = 21.60
  monkeypatch.setattr(stopping_flags, "PUBLISH_TRUE_LEAD_DISTANCE", False)
  legacy = get_santa_fe_stopped_lead_smooth_approach_cap(
    v_ego=9.21, lead=make_lead(status=True, d_rel=true_d_rel - isd, v_lead=0.0), increased_stopped_distance=isd)
  monkeypatch.setattr(stopping_flags, "PUBLISH_TRUE_LEAD_DISTANCE", True)
  flipped = get_santa_fe_stopped_lead_smooth_approach_cap(
    v_ego=9.21, lead=make_lead(status=True, d_rel=true_d_rel, v_lead=0.0), increased_stopped_distance=isd)
  assert legacy is not None
  assert flipped == legacy


def test_slowing_lead_cap_flag_on_drops_only_the_compensation_term(monkeypatch):
  # flag on with ISD>0 must equal flag off with ISD==0 at the same published d_rel
  # (the uncompensated d_rel uses, e.g. projected TTC, shift by ISD by design -- R5 residual)
  lead = make_lead(status=True, d_rel=24.20, v_rel=-1.58, v_lead=10.01, a_lead_k=-1.21)
  monkeypatch.setattr(stopping_flags, "PUBLISH_TRUE_LEAD_DISTANCE", False)
  baseline = get_santa_fe_slowing_lead_smooth_approach_cap(v_ego=11.51, lead=lead, increased_stopped_distance=0.0)
  monkeypatch.setattr(stopping_flags, "PUBLISH_TRUE_LEAD_DISTANCE", True)
  flipped = get_santa_fe_slowing_lead_smooth_approach_cap(v_ego=11.51, lead=lead, increased_stopped_distance=1.5)
  assert baseline is not None
  assert flipped == baseline


# --- Stop-commitment necessity floor (route 00001f47 seg6 driver takeover) ---------------------

def test_stop_aim_envelope_kill_switch_is_live():
  assert stopping_flags.SANTA_FE_STOP_AIM_ENVELOPE is True


def test_stop_aim_floor_repairs_the_00001f90_ease():
  # Route 00001f90 seg22 (bookmarked, cycle-24): lead stopped, command EASED below the decel
  # required to rest at the 4.3 m aim; the deficit was repaid by the terminal stack at -2.46.
  # Recorded samples through the ease window -- the lane must commit and hold the necessity.
  # t-3.25s: v=3.99, gap 10.0, cmd -1.48; required-to-aim ~1.6 -> commits and binds.
  lead = make_lead(status=True, d_rel=10.0, v_lead=0.0, a_lead_k=-0.26)
  floor, committed = get_santa_fe_stop_aim_floor(3.99, lead, -1.48, [-0.26], False, 4.3)
  assert committed and floor is not None
  assert -1.75 < floor < -1.45
  assert floor < -1.48  # deepen-only: it actually binds against the eased command
  # t-2.75s: v=3.30, gap 7.8, cmd -1.43, still committed; necessity ~1.9 -> holds
  lead = make_lead(status=True, d_rel=7.8, v_lead=0.0, a_lead_k=0.0)
  floor, committed = get_santa_fe_stop_aim_floor(3.30, lead, -1.43, [0.0], True, 4.3)
  assert committed and floor is not None and floor < -1.43


def test_stop_aim_floor_stays_out_of_gentle_approaches():
  # f90 seg21 / f85 seg13 class (rest 4.09-4.20, felt at the human gate): requirement to the aim
  # never reaches ON while the command tracks it -- the lane must be silent end to end.
  lead = make_lead(status=True, d_rel=6.4, v_lead=0.0, a_lead_k=0.0)
  floor, committed = get_santa_fe_stop_aim_floor(3.61, lead, -1.10, [0.0], False, 4.3)
  # required = 3.61^2/(2*(6.4-0.72-4.3)) is far ABOVE cap at this tight geometry -> onset refuses
  assert floor is None and not committed
  lead = make_lead(status=True, d_rel=14.0, v_lead=0.0, a_lead_k=0.0)
  floor, committed = get_santa_fe_stop_aim_floor(4.0, lead, -1.10, [0.0], False, 4.3)
  # required ~0.9 < ON on an ordinary healthy approach -> silent
  assert floor is None and not committed


def test_stop_aim_onset_refuses_late_hot_but_committed_rides_to_cap():
  # f90 seg21's earlier episode: commitment onset while required decel ALREADY exceeds the cap
  # would step the target 0 -> -2.25 in one frame -- refuse; the floor/late-approach lanes own it.
  lead = make_lead(status=True, d_rel=8.0, v_lead=0.0, a_lead_k=0.0)
  floor, committed = get_santa_fe_stop_aim_floor(3.95, lead, 0.0, [0.0], False, 4.3)
  assert floor is None and not committed
  # ...but an ALREADY-COMMITTED stop whose necessity grows through the cap clips AT the cap
  floor, committed = get_santa_fe_stop_aim_floor(3.95, lead, 0.0, [0.0], True, 4.3)
  assert committed and floor == -SANTA_FE_STOP_AIM_CAP


def test_stop_aim_excludes_braking_waves_by_projected_stop_distance():
  # f86 seg41: lead braking hard from 14.7 m/s at 29 m -- projected stop point ~80 m out; the
  # lead releases long before resting. Commitment must refuse non-imminent stops.
  lead = make_lead(status=True, d_rel=29.4, v_lead=14.65, a_lead_k=-1.85)
  floor, committed = get_santa_fe_stop_aim_floor(16.0, lead, -1.43, [-1.85], False, 4.3)
  assert floor is None and not committed


def test_stop_aim_release_hysteresis_and_lead_departure():
  lead = make_lead(status=True, d_rel=10.0, v_lead=0.0, a_lead_k=0.0)
  _, committed = get_santa_fe_stop_aim_floor(3.99, lead, -1.48, [0.0], False, 4.3)
  assert committed
  # necessity decays below OFF (stop executed / lead moved off): release
  lead = make_lead(status=True, d_rel=30.0, v_lead=0.0, a_lead_k=0.0)
  floor, committed = get_santa_fe_stop_aim_floor(3.0, lead, -1.0, [0.0], True, 4.3)
  assert floor is None and not committed


def test_stop_aim_hardening_lead_rides_to_cap_without_release():
  # plan red-team: a lead that brakes HARDER mid-commitment shortens its projected stop point,
  # so the necessity GROWS -- the committed lane must ride it to the cap, never release.
  lead = make_lead(status=True, d_rel=12.0, v_lead=3.0, a_lead_k=-1.0)
  floor, committed = get_santa_fe_stop_aim_floor(6.5, lead, -1.30, [-1.0], False, 4.3)
  assert committed and floor is not None
  lead = make_lead(status=True, d_rel=9.5, v_lead=1.5, a_lead_k=-2.5)  # lead slams
  floor2, committed = get_santa_fe_stop_aim_floor(6.0, lead, -1.30, [-1.0, -2.5], True, 4.3)
  assert committed
  assert floor2 <= floor  # necessity monotone-deepens, bounded at the cap
  assert floor2 >= -SANTA_FE_STOP_AIM_CAP - 1e-9


def test_stop_aim_rest_target_tracks_isd():
  # rest_aim is LEAD_STOP_DISTANCE_TARGET + increasedStoppedDistance at the call site; a larger
  # reserve must produce a deeper necessity for the same geometry (ISD 0 vs 0.3).
  lead = make_lead(status=True, d_rel=10.0, v_lead=0.0, a_lead_k=0.0)
  f_isd0, c0 = get_santa_fe_stop_aim_floor(3.99, lead, -1.30, [0.0], False, 4.0)
  f_isd3, c3 = get_santa_fe_stop_aim_floor(3.99, lead, -1.30, [0.0], False, 4.3)
  assert c0 and c3
  assert f_isd3 < f_isd0


def test_stop_floor_lanes_are_evaluated_against_the_same_pre_lane_command():
  # END-REVIEW (HIGH): the reviewer's hardening sequence. Aim committed at its cap (-2.25) while
  # the 3 m band floor genuinely requires ~2.40 -- BETWEEN the aim cap and the floor's engage bar
  # measured off a post-aim command (2.25 + 0.30 = 2.55). Evaluated against the PRE-lane command
  # the floor engages and the merged output preserves the 3 m floor.
  lead = make_lead(status=True, d_rel=9.4, v_lead=1.4, a_lead_k=-1.0)
  pre_cmd = -1.63
  aim_floor, aim_c, commit_floor, floor_a = get_santa_fe_stop_floor_demands(
    5.5, lead, pre_cmd, [-1.0], True, False, 4.3, True)
  assert aim_c and aim_floor == -SANTA_FE_STOP_AIM_CAP
  assert floor_a and commit_floor is not None
  assert commit_floor < -SANTA_FE_STOP_AIM_CAP  # the floor's need is DEEPER than the aim cap
  merged = min(pre_cmd, *[f for f in (aim_floor, commit_floor) if f is not None])
  assert merged == commit_floor
  # the masked ordering (floor Schmitt reading the post-aim command) must NOT engage -- this is
  # what made the defect: same geometry, command already deepened to the aim cap
  masked_floor, masked_active = get_santa_fe_stop_commit_floor(5.5, lead, -SANTA_FE_STOP_AIM_CAP, [-1.0], False)
  assert masked_floor is None and not masked_active


def test_stop_aim_rollback_boundary_case_stays_committed_at_cap():
  # END-REVIEW (HIGH) exact boundary: v=2.17, gap=6.0, sustained lv=-0.20. The projection-
  # inflated necessity (3.11) exceeds CAP; onset must ride the BASELINE (2.03, commits) and the
  # floor value clips at CAP -- never a refusal in the class the projection serves.
  lead = make_lead(status=True, d_rel=6.0, v_lead=-0.20, a_lead_k=0.0)
  floor, committed = get_santa_fe_stop_aim_floor(2.17, lead, -0.92, [0.0], False, 4.3,
                                                 vlead_window=[-0.20] * 10)
  assert committed and floor == -SANTA_FE_STOP_AIM_CAP


def test_stop_aim_rollback_noise_burst_projects_nothing():
  # END-REVIEW (MEDIUM): the recorded stopped-lead Doppler bursts (-0.09..-0.20, <= 0.36 s)
  # always leave a clean frame inside the 0.5 s window -- least-negative-over-window projects
  # ZERO, so noise cannot commit or deepen; sustained rollback (every frame negative) projects.
  # geometry where the BASELINE commits (v 2.17, gap 6.0: baseline 2.03 in [ON, CAP])
  lead = make_lead(status=True, d_rel=6.0, v_lead=-0.20, a_lead_k=0.0)
  burst = [-0.20] * 7 + [0.0] + [-0.15] * 2          # 0.36 s burst + clean frame in-window
  f_burst, c_burst = get_santa_fe_stop_aim_floor(2.17, lead, -0.92, [0.0], False, 4.3,
                                                 vlead_window=burst)
  f_clean, c_clean = get_santa_fe_stop_aim_floor(2.17, lead, -0.92, [0.0], False, 4.3,
                                                 vlead_window=[0.0] * 10)
  # the burst projects NOTHING: byte-identical to the stopped-lead result
  assert c_burst and c_clean
  assert abs(f_burst - f_clean) < 1e-9
  # sustained rollback (every frame negative) deepens the committed floor (here to the cap)
  f_sus, c_sus = get_santa_fe_stop_aim_floor(2.17, lead, -0.92, [0.0], False, 4.3,
                                             vlead_window=[-0.20] * 10)
  assert c_sus and f_sus < f_clean


def test_stop_aim_rollback_projection_deepens_necessity():
  # cycle-27 (fc2 s6, felt 2.24, rest 3.2): a lead rolling BACK (-0.13..-0.20 sustained, ~1 m
  # over the approach) recedes the stop point; the clamped-at-zero necessity under-committed and
  # the floor defence paid -1.40 at gap 3.3. The projection shrinks the runway by
  # lv * ROLLBACK_HORIZON: same geometry, rolling-back lead -> deeper floor. HONEST NOTE: on the
  # recorded s6 the projection buys one earlier/deeper commitment beat; the class is majority
  # physics (each meter the lead rolls back is a meter of margin no comfort-band law recovers),
  # and the terminal floor-defence bill there is the floor working as designed.
  still = make_lead(status=True, d_rel=6.8, v_lead=0.0, a_lead_k=0.0)
  f_still, c_still = get_santa_fe_stop_aim_floor(2.17, still, -0.92, [0.0], False, 4.3,
                                                 vlead_window=[0.0] * 10)
  back = make_lead(status=True, d_rel=6.8, v_lead=-0.20, a_lead_k=0.0)
  f_back, c_back = get_santa_fe_stop_aim_floor(2.17, back, -0.92, [0.0], False, 4.3,
                                               vlead_window=[-0.20] * 10)
  # NOTE (end-review): commitment ONSET rides the baseline necessity, so with the baseline below
  # ON both cases refuse identically; where the baseline commits, sustained rollback DEEPENS the
  # floor value (see the boundary and noise fixtures above for the committed cases)
  assert c_back == c_still
  if f_back is not None and f_still is not None:
    assert f_back <= f_still
  # forward-creeping leads are untouched: the projection is deepen-only (min with 0)
  fwd = make_lead(status=True, d_rel=6.8, v_lead=0.20, a_lead_k=0.0)
  f_fwd, _ = get_santa_fe_stop_aim_floor(2.17, fwd, -0.92, [0.0], False, 4.3,
                                         vlead_window=[0.20] * 10)
  assert (f_fwd is None) == (f_still is None)


def test_stop_aim_reslam_extension_commits_below_the_floor():
  # cycle-28 (fd1 s4, felt 2.98, rest 2.8 = floor breach): mid-queue re-slam -- ego 1.3-1.8 m/s
  # behind a lead that launched then slams back to zero at 4 m. In 0.8-2.0 nobody committed
  # (aim floor 2.0, service refuses moving leads). Entry below the floor requires a
  # decisively-braking still-moving lead; recorded values commit at ~-1.7.
  lead = make_lead(status=True, d_rel=3.7, v_lead=0.56, a_lead_k=-0.64)
  floor, committed = get_santa_fe_stop_aim_floor(1.29, lead, -0.98, [-0.64, -0.70], False, 4.3,
                                                 vlead_window=[0.56] * 10)
  assert committed and floor is not None and floor < -1.3


def test_stop_aim_reslam_excludes_launches_and_steady_follow():
  # the f85 s4 launch-fight exclusion stands: a launching or steady lead keeps the 2.0 floor
  launching = make_lead(status=True, d_rel=3.7, v_lead=0.56, a_lead_k=+0.27)
  f1, c1 = get_santa_fe_stop_aim_floor(1.29, launching, -0.98, [+0.27, +0.22], False, 4.3,
                                       vlead_window=[0.56] * 10)
  assert f1 is None and not c1
  steady = make_lead(status=True, d_rel=3.7, v_lead=0.56, a_lead_k=-0.05)
  f2, c2 = get_santa_fe_stop_aim_floor(1.29, steady, -0.98, [-0.05, -0.10], False, 4.3,
                                       vlead_window=[0.56] * 10)
  assert f2 is None and not c2
  # a nearly-stopped lead (< 0.3) is the stopped-lead latch's territory, not the extension's
  slow = make_lead(status=True, d_rel=3.7, v_lead=0.10, a_lead_k=-0.64)
  f3, c3 = get_santa_fe_stop_aim_floor(1.29, slow, -0.98, [-0.64, -0.70], False, 4.3,
                                       vlead_window=[0.10] * 10)
  assert f3 is None and not c3
  # and below the extension's own floor the lane stays out entirely (NOTE: at these speeds the
  # runway clamp already makes ON unreachable -- the explicit RESLAM_V_MIN is belt over that
  # arithmetic; the paired mutation (floor + ON dropped together) proves this pin's coverage)
  f4, c4 = get_santa_fe_stop_aim_floor(0.70, make_lead(status=True, d_rel=3.7, v_lead=0.56,
                                                       a_lead_k=-0.64),
                                       -0.98, [-0.64], False, 4.3, vlead_window=[0.56] * 10)
  assert f4 is None and not c4


def test_stop_aim_reslam_launch_flip_releases_within_two_frames():
  # END-REVIEW (HIGH): after a rapid slam->launch flip, release must ride the LAST 0.1 s of alk
  # evidence, not the 6-frame window (which kept slam frames in scope for 0.3 s while the lane
  # commanded -1.3 against a +1.0 launch). Sequence: committed with a slam-heavy window, then
  # two launch frames arrive -> released.
  lead = make_lead(status=True, d_rel=3.5, v_lead=0.9, a_lead_k=+0.4)
  mixed = [-0.9, -1.0, -0.8, -0.6, +0.3, +0.4]   # slam history + 0.1 s of launch
  floor, committed = get_santa_fe_stop_aim_floor(1.14, lead, 1.0, mixed, True, 4.3,
                                                 vlead_window=[0.9] * 10)
  assert floor is None and not committed
  # one launch frame alone is NOT enough (noise immunity)
  one = [-0.9, -1.0, -0.8, -0.6, -0.5, +0.4]
  floor, committed = get_santa_fe_stop_aim_floor(1.14, lead, 1.0, one, True, 4.3,
                                                 vlead_window=[0.9] * 10)
  assert committed


def test_stop_aim_reslam_hold_band_covers_the_service_dwell():
  # END-REVIEW (HIGH): the runway clamp makes a_req <= v^2, so the old baseline release let
  # EVERY commitment self-release below 1.0 m/s -- exactly while the stopped-lead latch is
  # earning its 0.3 s dwell. Committed now holds through 1.0 and 0.8 down to RESLAM_EXIT_V
  # against a stopped, non-departing lead; below it the lane exits (HOLD/RAMP machinery owns).
  lead = make_lead(status=True, d_rel=3.2, v_lead=0.05, a_lead_k=-0.8)
  for v in (0.99, 0.85, 0.60, 0.35):
    floor, committed = get_santa_fe_stop_aim_floor(v, lead, -0.40, [-0.8, -0.9], True, 4.3,
                                                   vlead_window=[0.05] * 10)
    assert committed and floor is not None, f"hold band broke at v={v}"
  floor, committed = get_santa_fe_stop_aim_floor(0.25, lead, -0.40, [-0.8, -0.9], True, 4.3,
                                                 vlead_window=[0.05] * 10)
  assert not committed and floor is None


def test_stop_aim_reslam_launch_from_zero_releases_below_half_ms():
  # ROUND-3 (HIGH): a lead stopping during commitment then launching from zero must release
  # BEFORE reaching 0.5 m/s -- rising-speed evidence (+0.08 over 0.15 s with positive alk).
  lead = make_lead(status=True, d_rel=3.4, v_lead=0.22, a_lead_k=+0.35)
  vlw = [0.0] * 7 + [0.05, 0.14, 0.22]     # launching from zero, still under 0.5
  floor, committed = get_santa_fe_stop_aim_floor(1.14, lead, 1.0, [-0.6, -0.4, +0.30, +0.35],
                                                 True, 4.3, vlead_window=vlw)
  assert floor is None and not committed
  # ...but a stopped lead with negative-side Doppler noise does NOT read as launching
  noisy = make_lead(status=True, d_rel=3.4, v_lead=0.10, a_lead_k=+0.20)
  vlw_noise = [0.0, -0.12, 0.0, -0.09, 0.0, 0.0, -0.15, 0.0, 0.05, 0.10]
  floor, committed = get_santa_fe_stop_aim_floor(1.14, noisy, -0.40, [-0.6, -0.4, +0.18, +0.20],
                                                 True, 4.3, vlead_window=vlw_noise)
  assert committed, "noise-level readings released the hold band"


def test_stop_aim_release_semantics_above_the_hold_band():
  # boundary pins: above 1.5 the baseline-decay release applies (a dissolved stop releases);
  # ordinary above-2.0 commitments keep the same decay release
  far = make_lead(status=True, d_rel=12.0, v_lead=0.0, a_lead_k=0.0)
  floor, committed = get_santa_fe_stop_aim_floor(1.6, far, -0.40, [0.0], True, 4.3,
                                                 vlead_window=[0.0] * 10)
  assert not committed  # baseline ~0.24 < OFF at v 1.6: released (stop dissolved / gap opened)
  floor, committed = get_santa_fe_stop_aim_floor(2.5, far, -0.40, [0.0], True, 4.3,
                                                 vlead_window=[0.0] * 10)
  assert not committed  # same decay semantics above 2.0


def test_stop_aim_reslam_commitment_rides_through_lead_reaching_zero():
  # the re-slam's natural end: the lead reaches ~0 mid-commitment. Dropping the lane there
  # re-creates the dead zone at peak necessity (extension entry closed, latch not confirmed).
  lead = make_lead(status=True, d_rel=3.5, v_lead=0.13, a_lead_k=-0.99)
  floor, committed = get_santa_fe_stop_aim_floor(1.04, lead, -0.84, [-0.99, -1.0], True, 4.3,
                                                 vlead_window=[0.13] * 10)
  assert committed and floor is not None


def test_stop_aim_stays_out_below_service_entry():
  # below V_EGO_MIN the StoppingService owns stopped-lead work; a fresh crawl/launch can never
  # ENTER (f85 seg4: bind at v=1.14 against a +1.0 launch command in the ungated scan) -- the
  # extension's entry demands a decisively-braking MOVING lead
  lead = make_lead(status=True, d_rel=3.0, v_lead=0.0, a_lead_k=0.0)
  floor, committed = get_santa_fe_stop_aim_floor(1.14, lead, 1.0, [0.0], False, 4.3,
                                                 vlead_window=[0.0] * 10)
  assert floor is None and not committed
  # ...and a STALE commitment releases the moment the lead departs (launch dissolves the stop;
  # the baseline alone cannot decay fast at tight gaps because of the runway clamp)
  departing = make_lead(status=True, d_rel=3.5, v_lead=0.9, a_lead_k=+0.4)
  floor, committed = get_santa_fe_stop_aim_floor(1.14, departing, 1.0, [+0.3, +0.4], True, 4.3,
                                                 vlead_window=[0.9] * 10)
  assert floor is None and not committed


def test_rest_close_kill_switch_is_live():
  assert stopping_flags.SANTA_FE_REST_CLOSE_FLOOR is True


def test_rest_close_floor_matches_the_s22_closure_curve():
  # cycle-31 (route 00002011 s22, rest 6.47): the comfort closure toward 4.3 from the design
  # review's counterfactual table -- 0.8 m/s held through the mid-closure, tapering to zero.
  # gap 5.5, v_guard 0.7: d_eff = 5.5 - 4.3 - 0.175 = 1.025 -> floor = min(0.8, sqrt(1.025))
  f = get_santa_fe_rest_close_floor_v(5.5, 0.7, 4.3, 0.8)
  assert f == pytest.approx(0.8, abs=0.01)
  f = get_santa_fe_rest_close_floor_v(5.0, 0.5, 4.3, 0.8)   # d_eff 0.575 -> sqrt(0.575)=0.758
  assert f == pytest.approx(0.758, abs=0.01)
  # outside the active window: no floor (too close -- the terminal machinery owns; too far --
  # ordinary following owns)
  assert get_santa_fe_rest_close_floor_v(4.7, 0.5, 4.3, 0.8) is None   # d_eff 0.275 < 0.5
  assert get_santa_fe_rest_close_floor_v(7.5, 0.5, 4.3, 0.8) is None   # d_eff 3.075 > 2.5


def test_rest_close_reference_lift_is_raise_only_and_position_consistent():
  t = np.array([0.0, 0.5, 1.0, 1.5, 2.0])
  x = np.array([0.0, 0.10, 0.15, 0.17, 0.18])   # the e2e reference dying (the stall)
  v = np.array([0.25, 0.10, 0.03, 0.0, 0.0])
  a = np.zeros(5)
  x2, v2, a2, lifted = apply_santa_fe_rest_close_reference_floor(x, v, a, 0.8, t)
  assert lifted
  assert all(v2 >= v - 1e-9)                     # raise-only
  assert v2[0] == pytest.approx(0.8)             # the closure curve owns the dead reference
  assert v2[1] == pytest.approx(0.55)            # 0.8 - 0.5*0.5
  # position is the integral of the raised velocity (monotone, consistent)
  assert all(np.diff(x2) >= -1e-9)
  assert x2[1] == pytest.approx(x2[0] + 0.5 * (v2[0] + v2[1]) * 0.5, abs=1e-6)
  # a reference already above the floor is untouched
  v_hi = np.array([1.5, 1.3, 1.1, 0.9, 0.7])
  x3, v3, a3, lifted3 = apply_santa_fe_rest_close_reference_floor(x, v_hi, a, 0.8, t)
  assert not lifted3 and (v3 == v_hi).all()
  # CROSSING reference (alive early, dying late): the lift must never LOWER the early points --
  # raise-only means max(v, floor), not replacement
  v_x = np.array([1.2, 0.6, 0.1, 0.0, 0.0])
  x4, v4, a4, lifted4 = apply_santa_fe_rest_close_reference_floor(x, v_x, a, 0.8, t)
  assert lifted4
  assert v4[0] == pytest.approx(1.2), "the lift lowered a reference point above the floor"
  assert v4[2] == pytest.approx(0.3)  # 0.8 - 0.5*1.0


def test_rest_close_state_machine_one_shot_semantics():
  # arm on a healthy creep entry
  st = update_santa_fe_rest_close_state(False, False, 0.0, None, True, 0.9, 1.2, 7)
  assert st[0] and not st[1] and st[2] == pytest.approx(0.8) and st[3] == 7
  # ride while healthy
  st = update_santa_fe_rest_close_state(*st, rc_ok=True, v_ego=0.7, d_eff_arm=0.9, rc_tid=7)
  assert st[0]
  # lead replacement: permanent spend
  st = update_santa_fe_rest_close_state(*st, rc_ok=True, v_ego=0.7, d_eff_arm=0.9, rc_tid=9)
  assert not st[0] and st[1]
  # ...and no re-arm afterwards, however healthy
  st = update_santa_fe_rest_close_state(*st, rc_ok=True, v_ego=0.9, d_eff_arm=1.2, rc_tid=9)
  assert not st[0] and st[1]
  # entry refusals: above the arm band; below it; walking lead handled upstream via rc_ok
  assert not update_santa_fe_rest_close_state(False, False, 0.0, None, True, 2.0, 1.2, 7)[0]
  assert not update_santa_fe_rest_close_state(False, False, 0.0, None, True, 0.04, 1.2, 7)[0]
  # a 2 m/s lead-just-stopped approach: refused until inside the arm band (v gate)
  assert not update_santa_fe_rest_close_state(False, False, 0.0, None, False, 0.9, 1.2, 7)[0]
  # d_eff outside the window at arm time: refuse
  assert not update_santa_fe_rest_close_state(False, False, 0.0, None, True, 0.9, 0.3, 7)[0]
  assert not update_santa_fe_rest_close_state(False, False, 0.0, None, True, 0.9, 3.0, 7)[0]
  # v_cap: never accelerate above the captured entry speed
  st = update_santa_fe_rest_close_state(False, False, 0.0, None, True, 0.4, 1.2, 7)
  assert st[2] == pytest.approx(0.4)


def test_rest_close_approach_epoch_reopens_the_one_shot():
  # R1 HIGH: the spend is per-APPROACH, not per-process. Two consecutive stops in one plannerd
  # lifetime must each get the lane.
  # -- approach 1: arm at 0.8, ride to the rest; standstill cancels AND clears the spend
  st = update_santa_fe_rest_close_state(False, False, 0.0, None, True, 0.8, 1.2, 7)
  assert st[0] and st[2] == pytest.approx(0.8)
  st = update_santa_fe_rest_close_state(*st, rc_ok=False, v_ego=0.02, d_eff_arm=0.9, rc_tid=7,
                                        standstill=True)
  assert not st[0] and st[1], "cycle-33 R1: a completed rest stays SPENT until a new approach is evident"
  # at the rest itself nothing can arm, and a micro-roll behind a crawler (0.10 m/s, latch cleared)
  # with the gap still inside the window must NOT re-arm (E3)
  st = update_santa_fe_rest_close_state(*st, rc_ok=False, v_ego=0.0, d_eff_arm=0.9, rc_tid=7, standstill=True)
  assert not st[0] and st[1]
  for v_roll in (0.10, 0.11):
    st = update_santa_fe_rest_close_state(*st, rc_ok=True, v_ego=v_roll, d_eff_arm=0.9, rc_tid=7)
    assert not st[0] and st[1], f"micro-roll {v_roll} re-armed after a completed rest"
  # the gap merely reaching the window edge (2.5..3.0) is not departure evidence: still spent
  st = update_santa_fe_rest_close_state(*st, rc_ok=True, v_ego=0.3, d_eff_arm=2.7, rc_tid=7, d_eff_epoch=2.7)
  assert not st[0] and st[1]
  # R2 HIGH: an UNTRUSTED gap growing past the window (d_eff_arm 3.2 but no trusted epoch evidence) is not departure
  st = update_santa_fe_rest_close_state(*st, rc_ok=False, v_ego=0.3, d_eff_arm=3.2, rc_tid=7, d_eff_epoch=None)
  assert not st[0] and st[1]
  # the lead departs with TRUSTED geometry: the gap opens past the window + margin -> re-opens for the NEXT approach
  st = update_santa_fe_rest_close_state(*st, rc_ok=True, v_ego=0.3, d_eff_arm=3.2, rc_tid=7, d_eff_epoch=3.2)
  assert not st[0] and not st[1]
  # -- approach 2 (queue re-stop): arms FRESH with a NEW entry-speed cap once back in the window
  st = update_santa_fe_rest_close_state(*st, rc_ok=True, v_ego=0.5, d_eff_arm=1.0, rc_tid=11)
  assert st[0] and st[2] == pytest.approx(0.5) and st[3] == 11
  # -- mid-approach disqualifier spend HOLDS within the approach (no churn re-arm)...
  st = update_santa_fe_rest_close_state(*st, rc_ok=False, v_ego=0.5, d_eff_arm=1.0, rc_tid=11)
  assert not st[0] and st[1]
  st = update_santa_fe_rest_close_state(*st, rc_ok=True, v_ego=0.5, d_eff_arm=1.0, rc_tid=11)
  assert not st[0] and st[1]
  # ...and only leaving the regime entirely re-opens it
  st = update_santa_fe_rest_close_state(*st, rc_ok=False, v_ego=3.5, d_eff_arm=None, rc_tid=-1)
  assert not st[0] and not st[1]
  # boundary: 3.0 exactly does NOT clear (strict >)
  st2 = update_santa_fe_rest_close_state(False, True, 0.0, None, False, 3.0, None, -1)
  assert st2[1]
  # GAUNTLET NOTE (MR9, documented unkillable): dropping the `not armed` guard on the epoch
  # line is behavior-equivalent -- rc_ok requires `not standstill` and the cancel branch fires
  # at v > CANCEL_V (1.60) < EPOCH_RESET_V (3.0), so armed never survives to an epoch-true
  # frame. The guard is belt-and-braces against future threshold edits, not live logic.


class _Sig:
  """StopSignals stand-in for the pure gate (only the fields the gate reads)."""
  def __init__(self, d_gap=5.4, dropout=False, source="measured", outward=False, earned=True, wstop=False):
    self.d_gap, self.dropout_active, self.gap_source, self.gap_hold_outward = d_gap, dropout, source, outward
    self.lead_motion_earned, self.wheel_stop_latched = earned, wstop


def _ok(**kw):
  a = dict(blended=True, engaged=True, authorized=True, force_coast=False, sig=_Sig(), lead_v=0.5, standstill=False, deeper_lane=False)
  a.update(kw)
  return get_santa_fe_rest_close_ok(**a)


def test_rest_close_trusted_crawler_arms_without_stopped_confirmation():
  # cycle-33: drive the REAL authority -> StopContext chain with a lead at 0.5 m/s for 0.5 s: motion
  # trust earned, strict stopped confirmation FALSE, and the lane arms (s15: ego 1.22, lead 0.56, gap 5.44)
  from openpilot.selfdrive.controls.lib.stop_context import StopContext
  from openpilot.selfdrive.controls.lib.lead_provenance import StoppingLeadAuthority
  ctx, auth = StopContext(), StoppingLeadAuthority()
  sig = None
  for _ in range(10):
    authorized = auth.update(v_ego=1.22, lead_status=True, lead_d_rel=5.44, lead_track_id=42, model_prob=0.9)
    sig = ctx.update(v_ego=1.22, a_ego=-0.4, a_cmd=-0.5, lead_status=authorized, lead_v=0.5,
                     lead_d_rel=5.44 if authorized else None, lead_track_id=42 if authorized else None,
                     standstill=False, dt=DT_MDL)
  assert authorized and sig.lead_motion_earned and not sig.lead_confirmed_stopped
  assert get_santa_fe_rest_close_ok(True, True, authorized, False, sig, 0.5, False, False)
  d_eff = sig.d_gap - 4.3 - 0.25 * 1.22
  st = update_santa_fe_rest_close_state(False, False, 0.0, None, True, 1.22, d_eff, 42)
  assert st[0] and st[2] == pytest.approx(0.8)


def test_rest_close_keeps_absolute_curve_for_crawling_lead():
  # sol pin: gap 5.4, v_guard 0.5, target 4.3, v_cap 0.5 -> d_eff 0.975, v_floor 0.5 (NOT lead_v + sqrt)
  d_eff = 5.4 - 4.3 - 0.25 * 0.5
  assert d_eff == pytest.approx(0.975)
  assert get_santa_fe_rest_close_floor_v(5.4, 0.5, 4.3, 0.5) == pytest.approx(0.5)
  assert get_santa_fe_rest_close_floor_v(5.4, 0.5, 4.3, 0.5) != pytest.approx(0.5 + math.sqrt(2 * 0.5 * d_eff))
  # the horizon floor terminates at zero and the position is re-integrated
  t = np.array([0.0, 0.5, 1.0, 1.5, 2.0])
  x, v, a, lifted = apply_santa_fe_rest_close_reference_floor(np.zeros(5), np.zeros(5), np.zeros(5), 0.5, t)
  assert lifted and v[-1] == 0.0 and v[0] == pytest.approx(0.5)
  assert x[1] == pytest.approx(0.5 * (0.5 + 0.25) * 0.5)


def test_rest_close_s6_late_crawl_entry():
  # 201a s6: refuse while the lead rolls 1.15 m/s; arm when it reaches 0.70 with ego at 0.50 / gap 6.1;
  # the captured cap is 0.50 -- the lane never commands above the entry speed
  assert not _ok(lead_v=1.15)
  assert _ok(lead_v=0.70, sig=_Sig(d_gap=6.1))
  d_eff = 6.1 - 4.3 - 0.25 * max(0.5, 0.5 - 0.7, 0.0)
  st = update_santa_fe_rest_close_state(False, False, 0.0, None, True, 0.50, d_eff, 7)
  assert st[0] and st[2] == pytest.approx(0.50)
  assert get_santa_fe_rest_close_floor_v(6.1, 0.5, 4.3, st[2]) == pytest.approx(0.50)


def test_rest_close_s15_prevents_reference_death_then_converges_once_lead_stops():
  # a dying model reference behind a crawler is raised (raise-only) before wheel-stop; once the lead
  # is at zero the curve converges toward the anchor (v -> 0 exactly at d_eff = 0)
  t = np.array([0.0, 0.5, 1.0, 1.5, 2.0])
  v_dying = np.array([0.35, 0.15, 0.0, 0.0, 0.0])
  floor = get_santa_fe_rest_close_floor_v(5.13, 0.5, 4.3, 0.5)    # s15 geometry at ego ~0.5
  x, v, a, lifted = apply_santa_fe_rest_close_reference_floor(np.zeros(5), v_dying, np.zeros(5), floor, t)
  assert lifted and (v >= v_dying - 1e-9).all() and v[0] == pytest.approx(floor)
  for gap in (5.0, 4.7, 4.5, 4.35):      # lead at zero: the floor speed decays with the remaining closure
    f = get_santa_fe_rest_close_floor_v(gap, 0.3, 4.3, 0.5)
    assert f is None or f <= 0.5
  assert get_santa_fe_rest_close_floor_v(4.8, 0.0, 4.3, 0.8) == pytest.approx(math.sqrt(2 * 0.5 * 0.5))   # curve value
  assert get_santa_fe_rest_close_floor_v(4.8, 0.0, 4.3, 0.5) == pytest.approx(0.5)                         # entry cap binds
  assert get_santa_fe_rest_close_floor_v(4.7, 0.0, 4.3, 0.5) is None   # below the 0.5 m window: the lane is out


def test_rest_close_continuing_crawler_respects_the_mpc_gap_law():
  # the lane does not touch desired_follow_distance: pin the MPC gap behind a 0.5 m/s lead at 5.4 m
  # (ISD 0.3, t_follow 0.9): 5.674 at ego 1.0, 5.074 at ego 0.5 -- no 4.3 m promise while the lead crawls
  from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import desired_follow_distance
  from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.stop_target_helpers import LEAD_STOP_DISTANCE_TARGET
  for v_ego, expect in ((1.0, 5.674), (0.5, 5.074)):
    d = desired_follow_distance(np.array([v_ego]), np.array([0.5]), np.array([5.4]), t_follow=0.9,
                                lead_stop_distance_target=LEAD_STOP_DISTANCE_TARGET + 0.3)
    assert float(np.asarray(d).ravel()[0]) == pytest.approx(expect, abs=0.02)


def test_rest_close_crawl_exit_and_reversal_cancel_and_spend():
  st = update_santa_fe_rest_close_state(False, False, 0.0, None, True, 0.8, 1.2, 7)
  assert st[0]
  # crawl exit: lead_v 0.91 sustained (3 frames) -> cancel + spend; a later 0.5 cannot re-arm
  assert not _ok(lead_v=0.91)
  st = update_santa_fe_rest_close_state(*st, rc_ok=_ok(lead_v=0.91, armed=True, lead_out_frames=6), v_ego=0.8, d_eff_arm=1.2, rc_tid=7)
  assert not st[0] and st[1]
  st = update_santa_fe_rest_close_state(*st, rc_ok=_ok(lead_v=0.5), v_ego=0.8, d_eff_arm=1.2, rc_tid=7)
  assert not st[0] and st[1]
  # reversal guard: -0.11 cancels and spends; -0.10 exactly is still eligible
  assert not _ok(lead_v=-0.11) and _ok(lead_v=-0.10) and _ok(lead_v=0.90)
  st2 = update_santa_fe_rest_close_state(False, False, 0.0, None, True, 0.8, 1.2, 7)
  st2 = update_santa_fe_rest_close_state(*st2, rc_ok=_ok(lead_v=-0.11, armed=True, lead_out_frames=6), v_ego=0.8, d_eff_arm=1.2, rc_tid=7)
  assert not st2[0] and st2[1]


def test_rest_close_cancel_debounce_ignores_single_frame_doppler_blips():
  # 201a s6: the lead read 1.00 / 1.01 for two frames then returned to 0.6-0.9 -- the approach must
  # not be spent; 201a s4: one -0.13 reading likewise. A SUSTAINED excursion (3 frames) cancels.
  # 6-frame (0.30 s) debounce: s4's burst was 5 frames at -0.11..-0.17, s6's 3 frames; p90 burst 0.29 s
  for k in (1, 2, 5):
    assert _ok(lead_v=1.01, armed=True, lead_out_frames=k)
    assert _ok(lead_v=-0.17, armed=True, lead_out_frames=k)
  assert not _ok(lead_v=1.01, armed=True, lead_out_frames=6)
  assert not _ok(lead_v=-0.13, armed=True, lead_out_frames=6)
  # arming is instantaneous: out of band on the current frame never arms, whatever the counter
  assert not _ok(lead_v=1.01, armed=False, lead_out_frames=0)
  assert not _ok(lead_v=-0.13, armed=False, lead_out_frames=0)
  # through the state machine: a 5-frame burst keeps the lane armed; the 6th frame cancels and spends
  st = update_santa_fe_rest_close_state(False, False, 0.0, None, True, 0.8, 1.2, 7)
  for k in (1, 2, 3, 4, 5):
    st = update_santa_fe_rest_close_state(*st, rc_ok=_ok(lead_v=-0.14, armed=True, lead_out_frames=k), v_ego=0.8, d_eff_arm=1.2, rc_tid=7)
    assert st[0]
  st = update_santa_fe_rest_close_state(*st, rc_ok=_ok(lead_v=-0.14, armed=True, lead_out_frames=6), v_ego=0.8, d_eff_arm=1.2, rc_tid=7)
  assert not st[0] and st[1]


def test_rest_close_identity_rules_reject_identity_less_leads_and_handover():
  # cycle-33 R1 MEDIUM: a real track (42) handing over to an identity-less vision lead (-1) is a
  # REPLACEMENT: cancel + spend (it must not inherit the entry cap / earned trust); and an
  # identity-less lead never arms this lift in the first place
  st = update_santa_fe_rest_close_state(False, False, 0.0, None, True, 0.8, 1.2, 42)
  assert st[0] and st[3] == 42
  st = update_santa_fe_rest_close_state(*st, rc_ok=True, v_ego=0.8, d_eff_arm=1.2, rc_tid=-1)
  assert not st[0] and st[1], "42 -> -1 handover inherited the lift"
  assert not update_santa_fe_rest_close_state(False, False, 0.0, None, True, 0.8, 1.2, -1)[0]
  # a real replacement 42 -> 43 likewise cancels and spends
  st = update_santa_fe_rest_close_state(False, False, 0.0, None, True, 0.8, 1.2, 42)
  st = update_santa_fe_rest_close_state(*st, rc_ok=True, v_ego=0.8, d_eff_arm=1.2, rc_tid=43)
  assert not st[0] and st[1]


def test_rest_close_untrusted_held_gap_cannot_reopen_the_completed_rest(monkeypatch):
  # R2 HIGH regression with the REAL StopContext: completed rest at 6.5 m; the same track then reports
  # status=true with an INVALID dRel for 21 frames (the context's held prediction grows past the
  # window), valid 5.5 m readings return, a 0.10 m/s micro-roll follows -- spent must remain set.
  # Drives the wiring's trust expression: epoch evidence only from a trusted gap of the same identity.
  from openpilot.selfdrive.controls.lib.stop_context import StopContext
  from openpilot.selfdrive.controls.lib.lead_provenance import StoppingLeadAuthority
  ctx, auth = StopContext(), StoppingLeadAuthority()

  def frame(v_ego, lead_v, d_rel, standstill=False):
    authorized = auth.update(v_ego=v_ego, lead_status=True, lead_d_rel=d_rel if math.isfinite(d_rel) else 0.0,
                             lead_track_id=42, model_prob=0.9)
    sig = ctx.update(v_ego=v_ego, a_ego=0.0, a_cmd=-0.3, lead_status=authorized, lead_v=lead_v,
                     lead_d_rel=(d_rel if authorized else None), lead_track_id=42 if authorized else None,
                     standstill=standstill, dt=DT_MDL)
    d_eff = None if sig.d_gap is None else sig.d_gap - 4.3 - 0.25 * v_ego
    ok = get_santa_fe_rest_close_ok(True, True, authorized, False, sig, lead_v, standstill, False)
    return ok, d_eff, get_santa_fe_rest_close_epoch_evidence(authorized, sig, 42, 42, d_eff), sig

  st = (False, False, 0.0, None)
  for _ in range(10):                       # approach: arms
    ok, d_eff, d_ep, sig = frame(0.8, 0.5, 6.5)
    st = update_santa_fe_rest_close_state(*st, rc_ok=ok, v_ego=0.8, d_eff_arm=d_eff, rc_tid=42, d_eff_epoch=d_ep)
  assert st[0]
  for _ in range(10):                       # completed rest: cancels + spent
    ok, d_eff, d_ep, sig = frame(0.0, 0.0, 6.5, standstill=True)
    st = update_santa_fe_rest_close_state(*st, rc_ok=ok, v_ego=0.0, d_eff_arm=d_eff, rc_tid=42, standstill=True, d_eff_epoch=d_ep)
  assert not st[0] and st[1]
  grew = False
  for _ in range(21):                       # same track, status true, INVALID dRel: held prediction may grow
    ok, d_eff, d_ep, sig = frame(0.0, 0.9, float("nan"), standstill=True)
    grew = grew or (d_eff is not None and d_eff > 3.0)
    st = update_santa_fe_rest_close_state(*st, rc_ok=ok, v_ego=0.0, d_eff_arm=d_eff, rc_tid=42, standstill=True, d_eff_epoch=d_ep)
    assert st[1], "an untrusted held gap re-opened the completed rest"
  for _ in range(6):                        # valid readings return
    ok, d_eff, d_ep, sig = frame(0.0, 0.5, 5.5, standstill=True)
    st = update_santa_fe_rest_close_state(*st, rc_ok=ok, v_ego=0.0, d_eff_arm=d_eff, rc_tid=42, standstill=True, d_eff_epoch=d_ep)
  for v_roll in (0.10, 0.11, 0.12):         # micro-roll: must not arm
    ok, d_eff, d_ep, sig = frame(v_roll, 0.5, 5.5)
    st = update_santa_fe_rest_close_state(*st, rc_ok=ok, v_ego=v_roll, d_eff_arm=d_eff, rc_tid=42, d_eff_epoch=d_ep)
    assert not st[0] and st[1], f"micro-roll {v_roll} re-armed after the untrusted-gap episode"


def test_rest_close_outward_held_frame_cannot_reopen_the_completed_rest():
  # R3 HIGH regression with the REAL StopContext: completed rest at d_eff ~2.98; one same-track frame
  # with an OUTWARD dRel sample (held, outward provenance) crosses 3.0 before the 0.25 s persistence
  # completes; inward recovery; 0.10 m/s micro-roll -> spent must remain (epoch evidence = MEASURED only)
  from openpilot.selfdrive.controls.lib.stop_context import StopContext
  from openpilot.selfdrive.controls.lib.lead_provenance import StoppingLeadAuthority
  ctx, auth = StopContext(), StoppingLeadAuthority()

  def frame(v_ego, lead_v, d_rel, standstill=False):
    authorized = auth.update(v_ego=v_ego, lead_status=True, lead_d_rel=d_rel, lead_track_id=42, model_prob=0.9)
    sig = ctx.update(v_ego=v_ego, a_ego=0.0, a_cmd=-0.3, lead_status=authorized, lead_v=lead_v,
                     lead_d_rel=(d_rel if authorized else None), lead_track_id=42 if authorized else None,
                     standstill=standstill, dt=DT_MDL)
    d_eff = None if sig.d_gap is None else sig.d_gap - 4.3 - 0.25 * v_ego
    ok = get_santa_fe_rest_close_ok(True, True, authorized, False, sig, lead_v, standstill, False)
    return ok, d_eff, get_santa_fe_rest_close_epoch_evidence(authorized, sig, 42, 42, d_eff), sig

  st = (False, False, 0.0, None)
  for _ in range(10):                       # approach inside the window (gap 6.0 -> d_eff 1.5): arms
    ok, d_eff, d_ep, sig = frame(0.8, 0.5, 6.0)
    st = update_santa_fe_rest_close_state(*st, rc_ok=ok, v_ego=0.8, d_eff_arm=d_eff, rc_tid=42, d_eff_epoch=d_ep)
  assert st[0]
  for k in range(30):                       # completed rest while the lead crawls away: gap 6.0 -> 7.28 (d_eff 2.98)
    ok, d_eff, d_ep, sig = frame(0.0, 0.5, 6.0 + 1.28 * min(k / 20.0, 1.0), standstill=True)
    st = update_santa_fe_rest_close_state(*st, rc_ok=ok, v_ego=0.0, d_eff_arm=d_eff, rc_tid=42, standstill=True, d_eff_epoch=d_ep)
  assert not st[0] and st[1]
  assert sig.gap_source == "measured" and sig.d_gap == pytest.approx(7.28, abs=0.05)
  # ONE outward sample: the context emits a held/outward gap across the margin
  ok, d_eff, d_ep, sig = frame(0.0, 0.9, 7.40, standstill=True)
  st = update_santa_fe_rest_close_state(*st, rc_ok=ok, v_ego=0.0, d_eff_arm=d_eff, rc_tid=42, standstill=True, d_eff_epoch=d_ep)
  assert st[1], "one outward-held frame re-opened the completed rest"
  for _ in range(3):                        # inward recovery
    ok, d_eff, d_ep, sig = frame(0.0, 0.5, 6.5, standstill=True)
    st = update_santa_fe_rest_close_state(*st, rc_ok=ok, v_ego=0.0, d_eff_arm=d_eff, rc_tid=42, standstill=True, d_eff_epoch=d_ep)
  for v_roll in (0.10, 0.11):
    ok, d_eff, d_ep, sig = frame(v_roll, 0.5, 6.5)
    st = update_santa_fe_rest_close_state(*st, rc_ok=ok, v_ego=v_roll, d_eff_arm=d_eff, rc_tid=42, d_eff_epoch=d_ep)
    assert not st[0] and st[1]


class _Lane:
  """Drives the pure pieces exactly as the wiring does (pre-step, state machine, post-step)."""
  def __init__(self):
    self.st = (False, False, 0.0, None)
    self.rested, self.drive = False, 0

  def step(self, rc_ok, v_ego, d_eff_arm, rc_tid, standstill=False, d_eff_epoch=None):
    self.rested, self.drive, reopen = santa_fe_rest_close_drive_reopen(self.rested, self.drive, standstill, self.st[1], v_ego)
    before = self.st[1]
    self.st = update_santa_fe_rest_close_state(*self.st, rc_ok=rc_ok, v_ego=v_ego, d_eff_arm=d_eff_arm, rc_tid=rc_tid,
                                               standstill=standstill, d_eff_epoch=d_eff_epoch, drive_reopen=reopen)
    self.rested, self.drive = santa_fe_rest_close_consume_reopen(self.rested, self.drive, before, self.st[1])
    return self.st


def test_rest_close_driving_again_reopens_but_micro_rolls_never_do():
  # 201a s6: after the first queue stop ego drove 1.4-1.6 m/s for 4 s with the gap inside the window
  # (never > 3.0 m d_eff): ego clearly driving again re-opens the one-shot after 10 frames at >= 1.0;
  # 9 frames do not; a 0.12 m/s micro-roll for 100 frames never does
  lane = _Lane()
  for _ in range(5):
    lane.step(True, 0.8, 1.2, 7)
  assert lane.st[0]
  for _ in range(5):                                    # completed rest: cancel + spent + rested
    lane.step(False, 0.0, 1.0, 7, standstill=True)
  assert not lane.st[0] and lane.st[1] and lane.rested
  for _ in range(100):
    lane.step(True, 0.12, 1.2, 7)
    assert not lane.st[0] and lane.st[1]
  for _ in range(9):
    lane.step(True, 1.4, 1.8, 7)
    assert lane.st[1]
  lane.step(True, 1.4, 1.8, 7)                          # 10th frame: re-opens, evidence consumed
  assert not lane.st[1] and not lane.rested and lane.drive == 0
  lane.step(True, 1.4, 1.8, 7)
  assert lane.st[0] and lane.st[2] == pytest.approx(0.8)


def test_rest_close_drive_evidence_cannot_pre_earn_or_clear_a_new_spend():
  # R4 HIGH: (1) a FIRST approach at 1.4 m/s for 100 frames earns nothing (no completed rest yet);
  # (2) after a rest + 10 driving frames the lane re-arms, and then authority loss, an identity
  # change, and a 6-frame reversal each leave it SPENT -- the evidence was consumed at the re-open
  # and no re-open path acts on the frame of a cancel
  lane = _Lane()
  for _ in range(100):
    lane.step(False, 1.4, 3.0, 7)                       # driving, not eligible, never rested
  assert lane.drive == 0 and not lane.rested
  for _ in range(3):
    lane.step(True, 0.8, 1.2, 7)
  assert lane.st[0]
  lane.step(False, 1.2, 1.2, 9)                         # identity change at speed: cancel + spend
  assert not lane.st[0] and lane.st[1]
  for _ in range(30):
    lane.step(True, 1.4, 1.8, 9)                        # keeps driving: NOT a rest -> stays spent
    assert lane.st[1]
  for cancel in ("authority", "identity", "reversal"):
    lane = _Lane()
    for _ in range(3):
      lane.step(True, 0.8, 1.2, 7)
    for _ in range(5):
      lane.step(False, 0.0, 1.0, 7, standstill=True)    # completed rest
    for _ in range(10):
      lane.step(True, 1.4, 1.8, 7)                      # driving again: re-opens on the 10th frame
    assert not lane.st[1]
    lane.step(True, 1.2, 1.8, 7)                        # re-arms
    assert lane.st[0]
    if cancel == "authority":
      lane.step(False, 1.2, 1.8, 7)
    elif cancel == "identity":
      lane.step(True, 1.2, 1.8, 8)
    else:
      for k in range(1, 7):
        lane.step(_ok(lead_v=-0.14, armed=True, lead_out_frames=k), 1.2, 1.8, 7)
    assert not lane.st[0] and lane.st[1], f"{cancel}: the spend was cleared"
    for _ in range(40):                                 # keeps driving at 1.2-1.4: still spent (no rest)
      lane.step(True, 1.3, 1.8, 7)
      assert lane.st[1], f"{cancel}: old drive evidence cleared a new spend"


def test_rest_close_epoch_evidence_and_drive_counter_helpers():
  # epoch evidence: measured + authorized + earned + no dropout + same identity; held/outward is NOT evidence
  assert get_santa_fe_rest_close_epoch_evidence(True, _Sig(d_gap=8.0), 42, 42, 3.2) == 3.2
  assert get_santa_fe_rest_close_epoch_evidence(True, _Sig(d_gap=8.0), 42, None, 3.2) == 3.2
  assert get_santa_fe_rest_close_epoch_evidence(True, _Sig(source="held", outward=True), 42, 42, 3.2) is None
  assert get_santa_fe_rest_close_epoch_evidence(True, _Sig(dropout=True), 42, 42, 3.2) is None
  assert get_santa_fe_rest_close_epoch_evidence(True, _Sig(earned=False), 42, 42, 3.2) is None
  assert get_santa_fe_rest_close_epoch_evidence(False, _Sig(), 42, 42, 3.2) is None
  assert get_santa_fe_rest_close_epoch_evidence(True, _Sig(), 43, 42, 3.2) is None
  assert get_santa_fe_rest_close_epoch_evidence(True, _Sig(), -1, None, 3.2) is None
  # drive counter: counts only at/above REARM_V (1.0) and RESETS below it -- a 0.12 m/s micro-roll never counts
  n = 0
  for _ in range(100):
    n = update_santa_fe_rest_close_drive_frames(n, 0.12)
  assert n == 0
  for _ in range(7):
    n = update_santa_fe_rest_close_drive_frames(n, 1.4)
  assert n == 7
  assert update_santa_fe_rest_close_drive_frames(n, 0.9) == 0
  assert update_santa_fe_rest_close_drive_frames(3, 1.0) == 4


def test_rest_close_no_reopen_on_the_cancel_frame():
  # R4 invariant, gap path: a cancel and trusted departure evidence on the SAME frame leave the lane
  # spent on that frame (old/new evidence never clears a spend created in the current update); the
  # next frame with the same evidence re-opens it
  st = update_santa_fe_rest_close_state(False, False, 0.0, None, True, 0.8, 1.2, 7)
  assert st[0]
  st = update_santa_fe_rest_close_state(*st, rc_ok=False, v_ego=0.8, d_eff_arm=3.5, rc_tid=7, d_eff_epoch=3.5)
  assert not st[0] and st[1], "re-opened on the cancel frame"
  st = update_santa_fe_rest_close_state(*st, rc_ok=False, v_ego=0.8, d_eff_arm=3.5, rc_tid=7, d_eff_epoch=3.5)
  assert not st[1]


# GAUNTLET NOTE (G27, documented unkillable): dropping `rested` from the drive_reopen expression is
# behaviour-equivalent -- the counter is held at zero whenever rested is False (the scoped counter),
# so drive_frames >= REARM_FRAMES already implies rested. The term is belt-and-braces.


def test_rest_close_wheel_stop_latch_blocks_post_stop_motion():
  # E3 pinned independently of carState.standstill: the wheel-stop latch cancels, and a micro-roll
  # sequence (0.03 / 0.06 / 0.07 m/s) while latched cannot re-arm within the approach
  st = update_santa_fe_rest_close_state(False, False, 0.0, None, True, 0.5, 1.0, 7)
  assert st[0]
  st = update_santa_fe_rest_close_state(*st, rc_ok=_ok(sig=_Sig(wstop=True)), v_ego=0.03, d_eff_arm=1.0, rc_tid=7)
  assert not st[0] and st[1]
  for v in (0.06, 0.07):
    st = update_santa_fe_rest_close_state(*st, rc_ok=_ok(sig=_Sig(wstop=True)), v_ego=v, d_eff_arm=1.0, rc_tid=7)
    assert not st[0]
  assert not _ok(sig=_Sig(wstop=True))


@pytest.mark.parametrize("kw", [
  dict(authorized=False), dict(sig=_Sig(earned=False)), dict(sig=_Sig(dropout=True)),
  dict(sig=_Sig(source="held", outward=False)), dict(force_coast=True), dict(blended=False),
  dict(engaged=False), dict(deeper_lane=True), dict(standstill=True), dict(sig=_Sig(d_gap=None)),
  dict(lead_v=float("nan")),
])
def test_rest_close_disqualifiers(kw):
  assert _ok() and not _ok(**kw)


# GAUNTLET NOTE (G7, documented unkillable): dropping the explicit isfinite(lead_v) check is
# behaviour-equivalent -- a NaN fails the LEAD_V_MIN <= lead_v <= LEAD_V_MAX comparison by IEEE
# semantics, so the gate still returns False. The check is belt-and-braces for readability.


def test_rest_close_outward_held_gap_stays_eligible():
  assert _ok(sig=_Sig(source="held", outward=True))


def test_rest_close_unauthorized_lead_cannot_pre_earn_confirmation():
  # R2 MEDIUM regression: a REJECTED radar-only track (first sighting inside the acquisition
  # horizon at speed) must not pre-earn the stopped dwell or gap trust while unauthorized; when
  # modelProb later flips positive, confirmation must start from ZERO, not arm the same frame.
  # This drives the REAL classes through the exact gating expression the planner wiring uses.
  from openpilot.selfdrive.controls.lib.stop_context import StopContext
  from openpilot.selfdrive.controls.lib.lead_provenance import StoppingLeadAuthority
  ctx, auth = StopContext(), StoppingLeadAuthority()

  def step(v_ego, d_rel, prob):
    authorized = auth.update(v_ego=v_ego, lead_status=True, lead_d_rel=d_rel,
                             lead_track_id=42, model_prob=prob)
    lead_status = bool(True and authorized)  # the wiring's rc_lead_status mask
    sig = ctx.update(v_ego=v_ego, a_ego=0.0, a_cmd=0.0,
                     lead_status=lead_status, lead_v=0.0,
                     lead_d_rel=d_rel if lead_status else None,
                     lead_track_id=42 if lead_status else None,
                     standstill=False, dt=DT_MDL)
    return authorized, sig

  # first sighting: 10 m at 5.0 m/s -- inside get_radar_only_min_acquire_d_rel(5.0) ~ 14.8,
  # prob 0.0 -> REJECTED, and a rejected track stays rejected for its lifetime
  authorized, sig = step(5.0, 10.0, 0.0)
  assert not authorized
  # ego slows onto the return; 5 s of dwell opportunity while UNAUTHORIZED
  for _ in range(100):
    authorized, sig = step(0.8, 5.5, 0.0)
    assert not authorized
    assert not sig.lead_confirmed_stopped, "an unauthorized lead pre-earned confirmation"
  # modelProb flips positive: certificate certifies -- but nothing was pre-earned
  authorized, sig = step(0.8, 5.5, 0.6)
  assert authorized
  assert not sig.lead_confirmed_stopped, "the dwell must start at authorization, not before"
  # and with sustained vision association the dwell completes normally
  for _ in range(20):  # 1.0 s at DT_MDL >= the 0.3 s dwell + margin
    authorized, sig = step(0.8, 5.5, 0.6)
  assert sig.lead_confirmed_stopped


def test_stop_commit_envelope_kill_switch_is_live():
  assert stopping_flags.SANTA_FE_STOP_COMMIT_ENVELOPE is True


def test_stop_commit_floor_arrests_the_00001f47_terminal_softening():
  # t=6330.54: v=3.25, gap 5.4 m, lead fully stopped, command had softened to -1.80 while the
  # decel required to rest at the 3.0 m floor (on the actuation-delayed gap) was ~3.0.
  lead = make_lead(status=True, d_rel=5.4, v_lead=0.0, a_lead_k=-0.27)
  floor, active = get_santa_fe_stop_commit_floor(3.25, lead, -1.80, [-0.27], False)
  assert active
  assert floor is not None
  assert -3.10 < floor < -2.95

  # t=6331.04: v=2.52, gap 3.9 m, command -1.47; required 3.53 -> floor clamps at authority cap.
  lead = make_lead(status=True, d_rel=3.9, v_lead=0.0, a_lead_k=0.05)
  floor, active = get_santa_fe_stop_commit_floor(2.52, lead, -1.47, [0.05], True)
  assert active
  assert floor == -SANTA_FE_STOP_COMMIT_A_MAX


def test_stop_commit_floor_ignores_barely_slower_moving_lead():
  # The user-forbidden regression: a lead barely slower than us, not braking, must never gate in.
  assert not santa_fe_stop_commit_lead_state_ok(12.0, make_lead(status=True, d_rel=30.0, v_lead=10.5, a_lead_k=-0.30))
  # ... at any gap in range
  assert not santa_fe_stop_commit_lead_state_ok(12.0, make_lead(status=True, d_rel=12.0, v_lead=10.5, a_lead_k=-0.30))


def test_stop_commit_floor_stays_out_of_ordinary_stopped_lead_approaches():
  # July episode shape (00001ba3 t=6012): v=9.07 at 43.8 m to a stopped lead -- a normal
  # approach the comfort tables own; required-to-floor is ~1.0 so the lane must stay silent.
  lead = make_lead(status=True, d_rel=43.8, v_lead=0.0, a_lead_k=0.0)
  floor, active = get_santa_fe_stop_commit_floor(9.07, lead, -1.07, [0.0], False)
  assert floor is None and not active
  # low-speed big-gap stopped lead: no necessity
  lead = make_lead(status=True, d_rel=20.0, v_lead=0.0, a_lead_k=0.0)
  floor, active = get_santa_fe_stop_commit_floor(3.0, lead, -0.5, [0.0], False)
  assert floor is None and not active


def test_stop_commit_floor_respects_speed_ceiling_for_highway_brake_waves():
  # Corpus: 16/18 would-fires without the ceiling were 30-40 m/s highway braking waves.
  lead = make_lead(status=True, d_rel=45.4, v_lead=34.56, a_lead_k=-1.56)
  floor, active = get_santa_fe_stop_commit_floor(37.9, lead, -0.2, [-1.56], False)
  assert floor is None and not active
  lead = make_lead(status=True, d_rel=31.3, v_lead=15.45, a_lead_k=-3.28)
  floor, active = get_santa_fe_stop_commit_floor(18.73, lead, -2.3, [-3.28], False)
  assert floor is None and not active


def test_stop_commit_floor_is_deepen_only_and_schmitt_hysteretic():
  lead = make_lead(status=True, d_rel=5.4, v_lead=0.0, a_lead_k=0.0)
  # command already deeper than necessity (a_req ~3.02) -> no-op regardless of activation state
  floor, active = get_santa_fe_stop_commit_floor(3.25, lead, -3.10, [0.0], True)
  assert floor is None and not active
  # necessity between release (0.10) and activation (0.30) margins: engages only if already active
  # v=3.25, d_rel=6.05 -> delayed gap 5.40 -> a_req = 2.2005; command -2.00 -> a_req - |cmd| = 0.20
  lead = make_lead(status=True, d_rel=6.05, v_lead=0.0, a_lead_k=0.0)
  floor_inactive, active_inactive = get_santa_fe_stop_commit_floor(3.25, lead, -2.00, [0.0], False)
  floor_active, active_active = get_santa_fe_stop_commit_floor(3.25, lead, -2.00, [0.0], True)
  assert floor_inactive is None and not active_inactive
  assert floor_active is not None and active_active


def test_stop_commit_vision_lead_requires_high_model_confidence():
  weak = SimpleNamespace(status=True, dRel=8.0, vRel=0.0, vLead=0.0, aLeadK=0.0, radarTrackId=-1, modelProb=0.6)
  strong = SimpleNamespace(status=True, dRel=8.0, vRel=0.0, vLead=0.0, aLeadK=0.0, radarTrackId=-1, modelProb=0.97)
  radar_only = SimpleNamespace(status=True, dRel=8.0, vRel=0.0, vLead=0.0, aLeadK=0.0, radarTrackId=1234, modelProb=0.0)
  radar_confirmed = SimpleNamespace(status=True, dRel=8.0, vRel=0.0, vLead=0.0, aLeadK=0.0, radarTrackId=1234, modelProb=0.97)
  assert not santa_fe_stop_commit_lead_state_ok(5.0, weak)
  assert santa_fe_stop_commit_lead_state_ok(5.0, strong)
  assert santa_fe_stop_commit_lead_state_ok(2.83, radar_only)
  assert santa_fe_stop_commit_lead_state_ok(5.0, radar_confirmed)


def test_stop_commit_radar_only_track_requires_early_acquisition_or_model_confirmation():
  radar_only_close = make_lead(status=True, d_rel=6.02, v_rel=-2.83, v_lead=0.0, radar_track_id=102499, model_prob=0.0)
  minimum_acquire_d_rel = get_santa_fe_stop_commit_radar_min_acquire_d_rel(2.83)
  assert 7.5 < minimum_acquire_d_rel < 7.8
  assert not update_santa_fe_stop_commit_track_certificate(None, False, 2.83, radar_only_close, True)
  # A close track cannot self-certify later merely because ego slowed and the horizon shrank.
  assert not update_santa_fe_stop_commit_track_certificate(102499, False, 1.5, radar_only_close, True)

  radar_only_early = make_lead(status=True, d_rel=minimum_acquire_d_rel + 0.1, v_lead=0.0, radar_track_id=102500, model_prob=0.0)
  assert update_santa_fe_stop_commit_track_certificate(None, False, 2.83, radar_only_early, True)

  radar_model_matched = make_lead(status=True, d_rel=6.02, v_lead=0.0, radar_track_id=102499, model_prob=0.25)
  assert update_santa_fe_stop_commit_track_certificate(None, False, 2.83, radar_model_matched, True)


def test_stop_commit_radar_only_track_rejects_bookmarked_conflicting_lead_shape():
  # Route 00001f5c seg5: the manhole track appeared at 6.02 m while the real, strongly
  # model-confirmed car remained as leadTwo at 13.7 m.
  radar_only = make_lead(status=True, d_rel=6.02, v_rel=-2.83, v_lead=0.0, radar_track_id=102499, model_prob=0.0)
  confirmed_farther = make_lead(status=True, d_rel=13.7, v_lead=0.84, radar_track_id=50388, model_prob=0.999)
  assert not santa_fe_stop_commit_track_provenance_ok(radar_only, confirmed_farther, True)

  no_second_lead = make_lead(status=False)
  assert santa_fe_stop_commit_track_provenance_ok(radar_only, no_second_lead, True)

  weak_farther = make_lead(status=True, d_rel=13.7, radar_track_id=50388, model_prob=0.5)
  assert santa_fe_stop_commit_track_provenance_ok(radar_only, weak_farther, True)


def test_stop_commit_persistence_resets_on_track_switch_and_bad_frames():
  # the 00001b97 2-frame radar glitch class: a track flip restarts the 0.5 s count
  state = (None, 0, [], [])
  for _ in range(SANTA_FE_STOP_COMMIT_PERSIST_FRAMES):
    tid, frames, alk, vlw = state
    state = update_santa_fe_stop_commit_persistence(tid, frames, alk, lead_state_ok=True,
                                                    lead_track_id=100, a_lead_k=-1.0,
                                                    vlead_window=vlw, v_lead=0.0)
  assert state[1] >= SANTA_FE_STOP_COMMIT_PERSIST_FRAMES
  tid, frames, alk, vlw = state
  switched = update_santa_fe_stop_commit_persistence(tid, frames, alk, lead_state_ok=True,
                                                     lead_track_id=101, a_lead_k=-1.0,
                                                     vlead_window=vlw, v_lead=0.0)
  assert switched[1] == 1
  dropped = update_santa_fe_stop_commit_persistence(tid, frames, alk, lead_state_ok=False,
                                                    lead_track_id=100, a_lead_k=-1.0,
                                                    vlead_window=vlw, v_lead=0.0)
  assert dropped[1] == 0 and dropped[0] is None


def test_stop_commit_alk_window_uses_least_severe_decel():
  # one -3.0 spike inside a mild-braking window must not shorten the projected lead stop:
  # window max (least severe) is -0.9 -> lead stop distance uses 0.9, not 3.0
  lead = make_lead(status=True, d_rel=20.0, v_lead=6.0, a_lead_k=-3.0)
  window = [-0.9, -3.0, -0.9]
  floor_spiky, _ = get_santa_fe_stop_commit_floor(10.0, lead, -0.3, window, False)
  floor_severe, _ = get_santa_fe_stop_commit_floor(10.0, lead, -0.3, [-3.0], False)
  # least-severe windowing projects the lead rolling FARTHER -> lower necessity
  req_windowed = get_santa_fe_stop_commit_required_decel(10.0, 20.0, 6.0, 0.9)
  req_severe = get_santa_fe_stop_commit_required_decel(10.0, 20.0, 6.0, 3.0)
  assert req_windowed < req_severe
  if floor_spiky is not None and floor_severe is not None:
    assert floor_spiky >= floor_severe
