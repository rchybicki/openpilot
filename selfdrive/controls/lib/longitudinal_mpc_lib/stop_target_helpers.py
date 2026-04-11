STOP_TARGET_LATCH_DURATION_S = 0.6


def update_distance_to_stop_target_with_latch(
  current_distance_to_stop_target_m: float,
  current_latch_timer_s: float,
  dt: float,
  candidates: tuple[float, ...],
) -> tuple[float, float]:
  positive_candidates = [candidate for candidate in candidates if candidate > 0.0]
  if positive_candidates:
    return min(positive_candidates), STOP_TARGET_LATCH_DURATION_S

  if current_distance_to_stop_target_m > 0.0 and current_latch_timer_s > 0.0:
    return current_distance_to_stop_target_m, max(0.0, current_latch_timer_s - dt)

  return -1.0, 0.0


def update_distance_to_stop_target_for_mode(
  mode: str,
  current_distance_to_stop_target_m: float,
  current_latch_timer_s: float,
  dt: float,
  candidates: tuple[float, ...],
) -> tuple[float, float]:
  if mode not in {"acc", "blended"}:
    return -1.0, 0.0

  return update_distance_to_stop_target_with_latch(
    current_distance_to_stop_target_m=current_distance_to_stop_target_m,
    current_latch_timer_s=current_latch_timer_s,
    dt=dt,
    candidates=candidates,
  )
