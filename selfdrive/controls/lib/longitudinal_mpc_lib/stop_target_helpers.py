import numpy as np

from openpilot.common.constants import CV

STOP_TARGET_LATCH_DURATION_S = 0.6
# Recent wide-stop review showed that the explicit stop target was still being
# crushed too aggressively once the lead crept into roughly the 5-6 kph band.
# Keep the truly stopped cases unchanged, but carry substantially more of the
# target through that creeping-lead regime before fading it out.
STOP_TARGET_SPEED_BP_KPH = [0.0, 1.5, 3.5, 5.5, 6.0, 6.5, 7.5]
STOP_TARGET_FACTOR_V = [1.0, 0.95, 0.78, 0.75, 0.65, 0.45, 0.0]
STOP_TARGET_MAX_DISTANCE_M = 4.5
LEAD_STOP_DISTANCE_TARGET = 4.0


def get_stop_target_factor(v_lead_kph: float) -> float:
  return float(np.interp(v_lead_kph, STOP_TARGET_SPEED_BP_KPH, STOP_TARGET_FACTOR_V))


def get_distance_to_stopped_lead_target(
  v_lead_raw,
  v_lead_distance_raw,
  increased_stopped_distance=0.0,
  lead_stop_distance_target=0.0,
):
  v_lead = np.mean(v_lead_raw)
  v_lead_kph = v_lead * CV.MS_TO_KPH
  v_lead_distance = np.mean(v_lead_distance_raw)
  distance_to_target = v_lead_distance + float(increased_stopped_distance) - float(lead_stop_distance_target)
  if distance_to_target <= 0.0 or distance_to_target > STOP_TARGET_MAX_DISTANCE_M:
    return 0.0

  # Keep the explicit stop target alive a bit longer for creeping leads only once the
  # stop is plausibly inside the remaining distance budget. This avoids leaking the
  # stopped-lead target into ordinary moving-following while still surfacing it early
  # enough for the soft approach / stop handoff logic to use.
  stopped_lead_factor = get_stop_target_factor(v_lead_kph)
  return max(0.0, distance_to_target * stopped_lead_factor)


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
