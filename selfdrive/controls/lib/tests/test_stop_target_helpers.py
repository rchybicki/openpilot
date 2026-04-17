import pytest

from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.stop_target_helpers import (
  STOP_TARGET_LATCH_DURATION_S,
  get_distance_to_stopped_lead_target,
  get_stop_target_factor,
  update_distance_to_stop_target_for_mode,
  update_distance_to_stop_target_with_latch,
)


def test_stop_target_latch_uses_min_positive_candidate() -> None:
  distance_to_stop_target_m, latch_timer_s = update_distance_to_stop_target_with_latch(
    current_distance_to_stop_target_m=-1.0,
    current_latch_timer_s=0.0,
    dt=0.05,
    candidates=(0.82, 0.31),
  )

  assert distance_to_stop_target_m == 0.31
  assert latch_timer_s == STOP_TARGET_LATCH_DURATION_S


def test_stop_target_latch_holds_last_positive_value_across_brief_dropout() -> None:
  distance_to_stop_target_m, latch_timer_s = update_distance_to_stop_target_with_latch(
    current_distance_to_stop_target_m=0.44,
    current_latch_timer_s=0.4,
    dt=0.05,
    candidates=(-1.0, -1.0),
  )

  assert distance_to_stop_target_m == 0.44
  assert latch_timer_s == pytest.approx(0.35, abs=1e-12)


def test_stop_target_latch_clears_after_timeout() -> None:
  distance_to_stop_target_m, latch_timer_s = update_distance_to_stop_target_with_latch(
    current_distance_to_stop_target_m=0.44,
    current_latch_timer_s=0.0,
    dt=0.05,
    candidates=(-1.0, -1.0),
  )

  assert distance_to_stop_target_m == -1.0
  assert latch_timer_s == 0.0


def test_stop_target_latch_stays_active_in_blended_mode() -> None:
  distance_to_stop_target_m, latch_timer_s = update_distance_to_stop_target_for_mode(
    mode="blended",
    current_distance_to_stop_target_m=-1.0,
    current_latch_timer_s=0.0,
    dt=0.05,
    candidates=(4.5, -1.0),
  )

  assert distance_to_stop_target_m == 4.5
  assert latch_timer_s == STOP_TARGET_LATCH_DURATION_S


def test_stop_target_latch_clears_for_unsupported_mode() -> None:
  distance_to_stop_target_m, latch_timer_s = update_distance_to_stop_target_for_mode(
    mode="unknown",
    current_distance_to_stop_target_m=0.44,
    current_latch_timer_s=0.4,
    dt=0.05,
    candidates=(0.82, 0.31),
  )

  assert distance_to_stop_target_m == -1.0
  assert latch_timer_s == 0.0


def test_stop_target_factor_keeps_creeping_lead_target_alive_longer() -> None:
  # Route 00000078 event 1 first surfaced the stop target with the lead already
  # down near 4.9 kph. Keep that explicit target alive earlier instead of
  # fading it out as if the lead were still genuinely moving.
  assert get_stop_target_factor(4.91) == pytest.approx(0.4795, abs=1e-4)
  assert get_stop_target_factor(2.66) == pytest.approx(0.7054, abs=1e-4)
  assert get_stop_target_factor(7.6) == pytest.approx(0.0, abs=1e-12)


def test_distance_to_stopped_lead_target_strengthens_recent_wide_route_case() -> None:
  distance_to_stop_target_m = get_distance_to_stopped_lead_target(
    v_lead_raw=1.364,
    v_lead_distance_raw=6.098,
    increased_stopped_distance=0.0,
    lead_stop_distance_target=2.5,
  )

  assert distance_to_stop_target_m == pytest.approx(1.725, abs=1e-3)


def test_distance_to_stopped_lead_target_preserves_good_slow_lead_behavior_inside_cap() -> None:
  distance_to_stop_target_m = get_distance_to_stopped_lead_target(
    v_lead_raw=0.739,
    v_lead_distance_raw=6.900,
    increased_stopped_distance=0.0,
    lead_stop_distance_target=2.5,
  )

  assert distance_to_stop_target_m == pytest.approx(3.104, abs=1e-3)


def test_distance_to_stopped_lead_target_stays_off_for_moving_lead() -> None:
  distance_to_stop_target_m = get_distance_to_stopped_lead_target(
    v_lead_raw=2.3,
    v_lead_distance_raw=6.5,
    increased_stopped_distance=0.0,
    lead_stop_distance_target=2.5,
  )

  assert distance_to_stop_target_m == pytest.approx(0.0, abs=1e-12)
