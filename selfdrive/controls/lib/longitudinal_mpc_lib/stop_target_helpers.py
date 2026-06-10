import numpy as np

from openpilot.common.constants import CV
from openpilot.selfdrive.controls.lib import stopping_flags

STOP_TARGET_LATCH_DURATION_S = 0.6
# Recent wide-stop review showed that the explicit stop target was still being
# crushed too aggressively once the lead crept into roughly the 5-6 kph band.
# Keep the truly stopped cases unchanged, but carry substantially more of the
# target through that creeping-lead regime before fading it out.
STOP_TARGET_SPEED_BP_KPH = [0.0, 1.5, 3.5, 5.5, 6.0, 6.5, 7.5]
STOP_TARGET_FACTOR_V = [1.0, 0.95, 0.78, 0.75, 0.65, 0.45, 0.0]
STOP_TARGET_MAX_DISTANCE_M = 4.5
STOP_TARGET_CLOSE_HOLD_REMAINING_M = 0.05
STOPPED_LEAD_MIN_CONTROL_GAP_M = 2.75
STOPPED_LEAD_CONTROL_MAX_GAP_M = 5.0
LEAD_STOP_DISTANCE_TARGET = 4.0


def get_stop_target_factor(v_lead_kph: float) -> float:
  return float(np.interp(v_lead_kph, STOP_TARGET_SPEED_BP_KPH, STOP_TARGET_FACTOR_V))


# --- ISD (increasedStoppedDistance) single-meaning helpers (stopping redesign §4.2) ---
# All sites read ONE constant: stopping_flags.PUBLISH_TRUE_LEAD_DISTANCE.
# Flag False (today): radard publishes dRel_true - ISD; the compensation terms below exactly
#   cancel the mutation and both stop paths rest at a TRUE gap of lead_stop_distance_target,
#   ISD-independent.
# Flag True: radard publishes the TRUE dRel and ISD's single meaning becomes
#   "rest ISD meters farther back" (rest true gap = lead_stop_distance_target + ISD).


def get_published_lead_distance(d_rel: float, increased_stopped_distance: float) -> float:
  """radard publish-side dRel mutation (§4.2.1). Flag off reproduces today's
  lead_dict['dRel'] -= increasedStoppedDistance bit-exactly; flag on publishes the true distance."""
  if stopping_flags.PUBLISH_TRUE_LEAD_DISTANCE:
    return d_rel
  return d_rel - float(increased_stopped_distance)


def get_published_lead_distance_compensation(increased_stopped_distance: float) -> float:
  """The term consumers tuned against TRUE gap add to a published dRel to recover it
  (planner Santa Fe hold-gap terms, longitudinal_planner.py). Becomes 0.0 once the
  publish-side mutation is gone."""
  if stopping_flags.PUBLISH_TRUE_LEAD_DISTANCE:
    return 0.0
  return float(increased_stopped_distance)


def get_effective_lead_distance(lead_d_rel: float, increased_stopped_distance: float) -> float:
  """Single-point ISD compensation for the longcontrol stopping layer (§4.2.4).
  LongControl.update computes lead_d_rel_eff = get_effective_lead_distance(...) ONCE at its top;
  the arbiter call and the kept-verbatim Santa Fe quirk-layer call sites consume it. Those
  consumers were tuned against the mutated published dRel, so once the true distance is
  published they receive lead_d_rel - ISD; while the mutation is active this is a passthrough.
  Invariant: for the same true gap, the returned value is identical in both flag states.
  # RETIRE: see FINAL_SPEC §3.2 row 6
  """
  if stopping_flags.PUBLISH_TRUE_LEAD_DISTANCE:
    return lead_d_rel - float(increased_stopped_distance)
  return lead_d_rel


def get_stopped_lead_obstacle_offset(
  stop_distance: float,
  increased_stopped_distance: float,
  lead_stop_distance_target: float,
  stopped_lead_factor: float,
) -> float:
  """MPC stopped-lead obstacle offset (§4.2.3(ii), red-team F25).

  Flag off: (stop_distance + ISD - target) * factor -- exactly cancels the radard mutation;
    ego rests at TRUE gap = lead_stop_distance_target, ISD-independent.
  Flag on: (stop_distance - ISD - target) * factor -- the MINUS sign makes the MPC obstacle
    appear ISD closer, so ego rests ISD farther from the true lead:
    rest TRUE gap = lead_stop_distance_target + ISD (matches get_distance_to_stopped_lead_target).
  """
  if stopping_flags.PUBLISH_TRUE_LEAD_DISTANCE:
    return (stop_distance - float(increased_stopped_distance) - float(lead_stop_distance_target)) * stopped_lead_factor
  return (stop_distance + float(increased_stopped_distance) - float(lead_stop_distance_target)) * stopped_lead_factor


def get_distance_to_stopped_lead_target(
  v_lead_raw,
  v_lead_distance_raw,
  increased_stopped_distance=0.0,
  lead_stop_distance_target=0.0,
):
  v_lead = np.mean(v_lead_raw)
  v_lead_kph = v_lead * CV.MS_TO_KPH
  v_lead_distance = np.mean(v_lead_distance_raw)
  if v_lead_distance <= 0.0:
    return 0.0
  if stopping_flags.PUBLISH_TRUE_LEAD_DISTANCE:
    # §4.2.3(i): v_lead_distance is the TRUE gap; rest ISD meters farther back.
    distance_to_target = v_lead_distance - (float(lead_stop_distance_target) + float(increased_stopped_distance))
  else:
    # today: v_lead_distance is the mutated published gap; the +ISD term cancels the mutation.
    distance_to_target = v_lead_distance + float(increased_stopped_distance) - float(lead_stop_distance_target)
  stopped_lead_factor = get_stop_target_factor(v_lead_kph)
  if distance_to_target <= 0.0:
    if stopped_lead_factor > 0.0:
      return STOP_TARGET_CLOSE_HOLD_REMAINING_M
    return 0.0
  if distance_to_target > STOP_TARGET_MAX_DISTANCE_M:
    return 0.0

  # Keep the explicit stop target alive a bit longer for creeping leads only once the
  # stop is plausibly inside the remaining distance budget. This avoids leaking the
  # stopped-lead target into ordinary moving-following while still surfacing it early
  # enough for the soft approach / stop handoff logic to use.
  return max(0.0, distance_to_target * stopped_lead_factor)


def get_stopped_lead_control_target(v_ego: float, lead_v: float, lead_d_rel: float) -> float | None:
  if lead_d_rel <= 0.0 or lead_d_rel > STOPPED_LEAD_CONTROL_MAX_GAP_M:
    return None
  if not (0.12 <= v_ego <= 1.90):
    return None

  closing_speed = v_ego - lead_v
  if closing_speed < np.interp(v_ego, [0.12, 0.75, 1.90], [0.04, 0.12, 0.18]):
    return None
  stopped_lead_v_limit = np.interp(v_ego, [0.12, 0.75, 1.90], [0.55, 0.45, 0.35])
  if lead_v > stopped_lead_v_limit:
    return None

  comfortable_decel = np.interp(v_ego, [0.12, 0.75, 1.90], [0.45, 0.62, 0.90])
  smooth_stop_distance = (v_ego * v_ego) / max(2.0 * comfortable_decel, 0.1)
  buffer_m = np.interp(v_ego, [0.12, 0.75, 1.90], [0.12, 0.20, 0.35])
  trigger_gap = float(np.clip(STOPPED_LEAD_MIN_CONTROL_GAP_M + smooth_stop_distance + buffer_m, 3.10, STOPPED_LEAD_CONTROL_MAX_GAP_M))
  if lead_d_rel > trigger_gap:
    return None

  return float(max(lead_d_rel - STOPPED_LEAD_MIN_CONTROL_GAP_M, STOP_TARGET_CLOSE_HOLD_REMAINING_M))


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
