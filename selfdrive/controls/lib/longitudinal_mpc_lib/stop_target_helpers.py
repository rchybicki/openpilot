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
# Cycle-25 (route 00001fb4 seg4, two bookmarks): NEAR-REST SPEED TIGHTENING. The speed-only
# factor curve above keeps a stop target 95% alive for a lead WALKING AWAY at 1.4 km/h, and a
# creeping queue's equilibrium gap equals the design rest gap -- so distanceToStopTarget glued
# at 0.05-0.2 m through entire creep-follow phases, longcontrol flapped into `stopping`, and the
# car executed full secure stops behind leads that never stopped (frame-verified: lead at
# +0.30..+0.58 m/s, gap growing, through a complete -0.70 landing). A stop target within
# NEAR_REST_BLEND_END of rest demands an ACTUALLY near-stopped lead: inside NEAR_REST_FULL_M
# the tolerance is NEAR_REST_SPEED_BP (full target only below ~0.5 km/h, gone by 2 km/h),
# blended back to the ordinary curve by NEAR_REST_BLEND_END. Far targets keep today's tolerant
# curve (approach shaping for decelerating leads is untouched). Frame-diffed over the corpus:
# the queue class loses its stop-mode entries (451 frames over the bookmarked pair's segments);
# every genuine stop's enter/hold decisions are UNCHANGED (lead speeds there are ~0 by the time
# the target is near); brief in-window dips of a decelerating lead cannot churn the published
# target because the arbiter's 0.6 s one-sided latch already requires CONTINUOUS absence.
STOP_TARGET_CLEAR_NOW = -2.0  # AFFIRMATIVE no-stop-here (near rule: lead moving at our rest
                              # point). Distinct from the callers' -1.0 no-candidate convention:
                              # -1.0 (lead absent/dropout) keeps the 0.6 s latch grace; -2.0
                              # clears the latch immediately.
NEAR_REST_FULL_M = 1.0
NEAR_REST_BLEND_END_M = 2.0
NEAR_REST_SPEED_BP_KPH = [0.5, 2.0]
NEAR_REST_SPEED_V = [1.0, 0.0]
NEAR_REST_CLEAR_V_MPS = 0.65  # AFFIRMATIVE clear only for a decisively-walking lead deep in the
                              # band; between 0.55 (factor 0) and here the target merely scales
                              # to ~0, so a lead DECELERATING through the band cannot flicker a
                              # one-frame clear between two live phases (churn fixture below)


def get_near_rest_speed_tightening(distance_to_target: float, v_lead_kph: float) -> float:
  """Multiplier in [0, 1]: how much of the stop-target factor survives this close to rest.
  1.0 whenever the target is far (> NEAR_REST_BLEND_END_M) or the lead is truly stopped."""
  near_factor = float(np.interp(v_lead_kph, NEAR_REST_SPEED_BP_KPH, NEAR_REST_SPEED_V))
  near_weight = float(np.interp(distance_to_target, [NEAR_REST_FULL_M, NEAR_REST_BLEND_END_M], [1.0, 0.0]))
  return (1.0 - near_weight) + near_weight * near_factor
STOP_TARGET_CLOSE_HOLD_REMAINING_M = 0.05
STOPPED_LEAD_MIN_CONTROL_GAP_M = 2.75
STOPPED_LEAD_CONTROL_MAX_GAP_M = 5.0
LEAD_STOP_DISTANCE_TARGET = 4.0
# Arrived-state early-return: once we have crept down to a STOPPED lead and are essentially
# at rest inside the hold gap, stop re-asserting control on bouncing radar (dRel jitter that
# re-arms the glide-cap near_hold_gap_cap and walks the car inward). See stopping_flags.
STOPPED_LEAD_REST_GAP_M = 4.0
STOPPED_LEAD_ARRIVED_V_EGO_MAX = 0.35
STOPPED_LEAD_ARRIVED_GAP_MARGIN_M = 0.30  # arrived ceiling = 4.30 m


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
  tightening = get_near_rest_speed_tightening(max(distance_to_target, 0.0), v_lead_kph)
  decisively_walking = v_lead >= NEAR_REST_CLEAR_V_MPS and distance_to_target <= NEAR_REST_FULL_M
  if distance_to_target <= 0.0:
    # the close-hold pin only survives for a truly near-stopped lead: any residual factor from
    # the tolerant far curve alone must NOT glue a 0.05 m target behind a walking lead. A
    # DECISIVELY walking lead at our rest point is an AFFIRMATIVE no-stop-here -- emit the clear
    # sentinel so the arbiter's dropout latch releases NOW instead of holding the pin 0.6 s.
    if stopped_lead_factor * tightening > 0.01:
      return STOP_TARGET_CLOSE_HOLD_REMAINING_M
    return STOP_TARGET_CLEAR_NOW if decisively_walking and stopped_lead_factor > 0.01 else 0.0
  if distance_to_target > STOP_TARGET_MAX_DISTANCE_M:
    return 0.0

  # Keep the explicit stop target alive a bit longer for creeping leads only once the
  # stop is plausibly inside the remaining distance budget. This avoids leaking the
  # stopped-lead target into ordinary moving-following while still surfacing it early
  # enough for the soft approach / stop handoff logic to use.
  tightened = distance_to_target * stopped_lead_factor * tightening
  if decisively_walking and tightened <= 0.01 and distance_to_target * stopped_lead_factor > 0.01:
    return STOP_TARGET_CLEAR_NOW  # near rule killed a target the ordinary curve kept: clear now
  return max(0.0, tightened)


def get_stopped_lead_control_target(v_ego: float, lead_v: float, lead_d_rel: float) -> float | None:
  if lead_d_rel <= 0.0 or lead_d_rel > STOPPED_LEAD_CONTROL_MAX_GAP_M:
    return None
  if not (0.12 <= v_ego <= 1.90):
    return None

  # TERMINAL-GLIDE PROFILE (correction 1): when the new flag is on, the synthetic target rests at
  # LEAD_STOP_DISTANCE_TARGET (4.0 m) instead of STOPPED_LEAD_MIN_CONTROL_GAP_M (2.75 m), so the
  # jerk-limited tracker lands v=0 at the intended 4.0 m hold gap in one monotonic glide. The
  # corrected stable target removes the synthetic jitter the arrived-gate patched, so the
  # STOPPED_LEAD_ARRIVED_GATE early-return is RETIRED (bypassed, not deleted) here -- it only ran
  # to suppress the re-grab walk that the 2.75 m rest produced.
  terminal_glide = stopping_flags.SANTA_FE_TERMINAL_GLIDE_PROFILE_ENABLED
  rest_gap_m = LEAD_STOP_DISTANCE_TARGET if terminal_glide else STOPPED_LEAD_MIN_CONTROL_GAP_M

  if (
    not terminal_glide
    and stopping_flags.STOPPED_LEAD_ARRIVED_GATE_ENABLED
    and v_ego <= STOPPED_LEAD_ARRIVED_V_EGO_MAX
    and lead_d_rel <= STOPPED_LEAD_REST_GAP_M + STOPPED_LEAD_ARRIVED_GAP_MARGIN_M
  ):
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
  # KEEP the trigger band on the 2.75 m gap so the control still FIRES at the same closure point;
  # only the RETURNED rest distance moves to rest_gap_m.
  trigger_gap = float(np.clip(STOPPED_LEAD_MIN_CONTROL_GAP_M + smooth_stop_distance + buffer_m, 3.10, STOPPED_LEAD_CONTROL_MAX_GAP_M))
  if lead_d_rel > trigger_gap:
    return None

  return float(max(lead_d_rel - rest_gap_m, STOP_TARGET_CLOSE_HOLD_REMAINING_M))


def update_distance_to_stop_target_with_latch(
  current_distance_to_stop_target_m: float,
  current_latch_timer_s: float,
  dt: float,
  candidates: tuple[float, ...],
) -> tuple[float, float]:
  positive_candidates = [candidate for candidate in candidates if candidate > 0.0]
  if positive_candidates:
    return min(positive_candidates), STOP_TARGET_LATCH_DURATION_S

  if any(candidate == STOP_TARGET_CLEAR_NOW for candidate in candidates):
    # affirmative clear (near-rest rule: the lead is MOVING at our rest point) -- the 0.6 s
    # grace below exists for radar DROPOUT flicker, not for a positively-classified non-stop
    return -1.0, 0.0

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
