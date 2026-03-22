import pytest

from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.stop_target_helpers import (
  STOP_TARGET_LATCH_DURATION_S,
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
