"""Stop-target arbiter: the single owner of stop intent + stop target (FINAL_SPEC section 5.2, WP4).

v1 rules are VERBATIM ports of the longcontrol.py arbitration layer at base commit 3be25f5240
(line references cite that file/commit): the predicate helpers (:92-148, :298-358, :396-422,
:425-506), the per-frame arbitration block (:729-837), the post-transition dropout holds (:843-868)
and the intent/standstill timers (:888-897, :1124-1128). Consolidation itself is behavior-neutral:
LongControl (WP7, Commit B) replaces its inline block with one ``decision = self.arbiter.update(...)``
and the Commit B hard gate replays old-vs-new for identical traces.

THERE IS EXACTLY ONE StopTargetArbiter IN THE SYSTEM and it lives in LongControl (spec section 6,
red-team F2). The V2 facade receives the StopDecision via its trailing ``decision`` kwarg and never
instantiates or imports this class (AST-guarded in test_stop_target_arbiter.py).

ISD compensation (spec 4.2.4, red-team F4): the ``lead_d_rel`` kwarg receives the ALREADY-EFFECTIVE
distance -- LongControl.update computes ``lead_d_rel_eff = get_effective_lead_distance(lead_d_rel,
increased_stopped_distance)`` ONCE at its top and passes it here. The arbiter performs NO internal
ISD compensation; ``increased_stopped_distance_m`` is accepted for the spec interface and reserved
for the synthetic-target rest expression (spec 4.2.3(i)) -- the v1 verbatim synthetic target
(get_stopped_lead_control_target) carries no ISD term, so the kwarg is unused in v1 (unit-tested).

Dropout-hold authority (spec 5.2.5, 3.2 row 1, red-team F3): while ARBITER_LEGACY_DROPOUT_HOLDS is
True the verbatim legacy predicates are AUTHORITATIVE for StopDecision; the consolidated dual-window
holds (0.4 s rolling / 0.8 s explicit-target release / 1.4 s no-target standstill) plus the
unbounded STOPPED_LEAD_LATCH run shadow-only and feed the divergence counters
(``legacy_hold_fired`` / ``single_hold_covered`` / ``hold_divergence``). Retirement trigger:
>= 25 dropout events during the V2 soak with hold_divergence == 0.

WP7 wiring notes (Commit B):
  * ``last_output_accel`` is PREVIOUS-FRAME semantics (legacy reads self.last_output_accel before
    the frame's output is computed).
  * ``long_control_state`` is the CURRENT (pre-transition) state -- i.e. the previous frame's
    post-transition state, which is exactly what the legacy intent timer (:893-897) consumed.
  * reset() mirrors LongControl.reset() (:610-616): call it exactly where legacy calls
    self.reset() (off state :908, starting state :972). Do NOT call it on the
    ``not stop_intent_active`` branch (:902-903) -- that branch resets only the stopping
    controller; resetting the arbiter there would zero the dropout-hold timers mid-window and
    break Commit B replay equivalence.
  * ``time_since_standstill_s`` after update() is the frame-k value (read it directly for
    ``standstill_recent``, :899); ``time_since_stop_intent_s`` after update() is the frame-(k-1)
    value the NEXT frame will arbitrate with -- for ``stop_intent_recent`` (:900) use
    projected_time_since_stop_intent_s() with the post-transition state.
  * ``decision.state_dropout_hold`` ports :843-868: after long_control_state_trans, when the
    current state is stopping and the new state is not,
    ``if decision.state_dropout_hold: new_state = stopping``.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum

import numpy as np

from opendbc.car.hyundai.values import CAR as HYUNDAI_CAR
from openpilot.selfdrive.controls.lib.stop_and_go_helpers import should_release_stop_hold_for_departing_lead
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.stop_target_helpers import get_stopped_lead_control_target
from openpilot.selfdrive.controls.lib.stopping_params import STOPPING_PARAMS, StoppingParams

clip = np.clip
interp = np.interp

# AUTHORITY FLAG (spec 3.2 row 1; removed, not flipped, at the cleanup commit): while True the
# verbatim legacy dropout-hold predicates below are authoritative for StopDecision and the
# consolidated dual-window holds run shadow-only. Read at call time (module attribute) so tests
# can exercise both states.
ARBITER_LEGACY_DROPOUT_HOLDS = True

# opendbc car.capnp Actuators.LongControlState values. The arbiter must stay pure-python
# importable (spec section 8 import-clean rule), so cereal/capnp is not imported here.
LONG_CTRL_STATE_OFF = 0
LONG_CTRL_STATE_PID = 1
LONG_CTRL_STATE_STOPPING = 2
LONG_CTRL_STATE_STARTING = 3

# --- verbatim constants (longcontrol.py:39-49) ----------------------------------------------------
MIN_STOP_TARGET_MODE_DISTANCE_M = 0.2
MAX_STOP_TARGET_MODE_DISTANCE_M = 0.5
LEAD_FOLLOW_TARGET_HOLD_GAP_M = 3.75
FAR_STOPPED_LEAD_CRAWL_GAP_M = 5.0
FAR_STOPPED_LEAD_CLOSE_TARGET_HOLD_M = 1.8

# --- consolidated-hold constants (spec 5.2.5; shadow-only while ARBITER_LEGACY_DROPOUT_HOLDS) -----
# Early release everywhere: rule-4 releases, brake press, or v_ego >= 0.30 with a_target > 0.2.
CONSOLIDATED_EARLY_RELEASE_V = 0.30
CONSOLIDATED_EARLY_RELEASE_A_TARGET = 0.2
# Explicit-target release-hold window: 0.8 s (or while the target is still present), preserving
# should_hold_low_speed_stop_target_release (longcontrol.py:492); its speed bound is v < 0.22 (:490).
RELEASE_HOLD_WINDOW_S = 0.8
RELEASE_HOLD_V_MAX = 0.22

_LEGACY_TIMER_CAP_S = 10.0  # longcontrol.py:891, :897


# --- verbatim predicate ports (helpers move here; longcontrol re-imports at Commit B, spec 5.2.3) --
# test_stop_target_arbiter.py asserts each function body is AST-identical to the longcontrol.py
# definition for as long as longcontrol.py still defines it.


def has_explicit_stop_target(distance_to_stop_target_m: float | None) -> bool:
  return distance_to_stop_target_m is not None and distance_to_stop_target_m > 0.0


def should_enter_stop_target_mode(v_ego: float, a_target: float, distance_to_stop_target_m: float | None) -> bool:
  if distance_to_stop_target_m is None:
    return False

  min_meaningful_distance = clip(
    interp(v_ego, [0.0, 1.0, 2.3, 4.2, 6.0], [0.20, 0.22, 0.34, 0.44, 0.48]) - interp(-a_target, [0.2, 0.6, 1.2, 1.8], [0.0, 0.08, 0.22, 0.30]),
    MIN_STOP_TARGET_MODE_DISTANCE_M,
    MAX_STOP_TARGET_MODE_DISTANCE_M,
  )
  if distance_to_stop_target_m <= min_meaningful_distance:
    return False

  distance_to_target = float(clip(distance_to_stop_target_m, 0.0, 6.0))
  activation_limit = interp(v_ego, [0.0, 0.6, 1.5, 3.0, 5.0], [0.35, 0.65, 1.10, 1.70, 2.30])
  min_stop_approach_accel = interp(v_ego, [0.0, 0.6, 1.5, 3.0, 5.0], [-0.03, -0.06, -0.10, -0.16, -0.22])
  return distance_to_target < activation_limit and a_target < min_stop_approach_accel


def should_hold_stop_target_mode(v_ego: float, a_target: float, distance_to_stop_target_m: float | None) -> bool:
  if should_enter_stop_target_mode(v_ego, a_target, distance_to_stop_target_m):
    return True
  if distance_to_stop_target_m is None or distance_to_stop_target_m <= 0.0:
    return False

  hold_limit = interp(v_ego, [0.0, 0.8, 1.5, 2.4], [1.20, 1.40, 1.65, 1.90])
  hold_accel = interp(v_ego, [0.0, 0.8, 1.5, 2.4], [-0.04, -0.07, -0.10, -0.16])
  return v_ego < 2.5 and distance_to_stop_target_m < hold_limit and a_target < hold_accel


def should_apply_stop_target_approach_mode(v_ego: float, a_target: float, distance_to_stop_target_m: float | None) -> bool:
  if should_enter_stop_target_mode(v_ego, a_target, distance_to_stop_target_m):
    return False
  if distance_to_stop_target_m is None or distance_to_stop_target_m <= 0.0:
    return False

  distance_to_target = float(clip(distance_to_stop_target_m, 0.0, 6.0))
  activation_limit = interp(v_ego, [1.0, 2.8, 4.5, 7.0], [1.0, 3.0, 3.8, 4.8])
  min_stop_approach_accel = interp(v_ego, [1.0, 2.8, 4.5, 7.0], [-0.04, -0.07, -0.10, -0.14])
  return v_ego > 1.0 and distance_to_target < activation_limit and a_target < min_stop_approach_accel


def should_apply_stop_target_carry_mode(v_ego: float, a_target: float, distance_to_stop_target_m: float | None) -> bool:
  if should_enter_stop_target_mode(v_ego, a_target, distance_to_stop_target_m):
    return False
  if should_apply_stop_target_approach_mode(v_ego, a_target, distance_to_stop_target_m):
    return False
  if distance_to_stop_target_m is None or distance_to_stop_target_m <= 0.0:
    return False
  if not (0.55 < v_ego < 1.25):
    return False
  if a_target > -0.08:
    return False

  requested_decel = float(clip(-a_target, 0.12, 0.90))
  predicted_stop_distance = (v_ego * v_ego) / max(2.0 * requested_decel, 0.24)
  carry_margin = interp(v_ego, [0.55, 0.85, 1.25], [0.80, 1.05, 1.35])
  return distance_to_stop_target_m > (predicted_stop_distance + carry_margin)


def should_release_far_stopped_lead_gap(
  v_ego: float,
  lead_status: bool,
  lead_v: float,
  lead_d_rel: float,
  distance_to_stop_target_m: float | None,
) -> bool:
  if distance_to_stop_target_m is not None and 0.0 <= float(distance_to_stop_target_m) <= FAR_STOPPED_LEAD_CLOSE_TARGET_HOLD_M:
    return False
  if not lead_status or lead_d_rel <= FAR_STOPPED_LEAD_CRAWL_GAP_M:
    return False
  if not (0.0 <= v_ego < 0.55):
    return False

  stopped_lead_v_limit = interp(v_ego, [0.00, 0.20, 0.55], [0.65, 0.45, 0.28])
  return lead_v <= stopped_lead_v_limit


def should_hold_recent_close_stopped_lead_dropout(
  v_ego: float,
  v_ego_starting: float,
  standstill: bool,
  time_since_standstill_s: float,
  lead_status: bool,
  lead_v: float,
  lead_d_rel: float,
  distance_to_stop_target_m: float | None,
  force_coast: bool,
) -> bool:
  if distance_to_stop_target_m is not None and distance_to_stop_target_m >= MIN_STOP_TARGET_MODE_DISTANCE_M:
    return False
  if not lead_status or lead_d_rel <= 0.0 or lead_d_rel > FAR_STOPPED_LEAD_CRAWL_GAP_M:
    return False
  if v_ego >= 1.25:
    return False

  recently_stopped = standstill or time_since_standstill_s < 1.20
  close_inside_hold_band = lead_d_rel <= LEAD_FOLLOW_TARGET_HOLD_GAP_M
  if not recently_stopped and not close_inside_hold_band:
    return False

  if force_coast:
    return True

  if not standstill and v_ego >= 0.35:
    moving_lead_release_speed = interp(v_ego, [0.35, 0.65, 1.25], [0.35, 0.32, 0.28])
    if lead_v > moving_lead_release_speed:
      return False

  lead_departing = should_release_stop_hold_for_departing_lead(
    human_acceleration=True,
    output_should_stop=True,
    force_coast=False,
    standstill=standstill,
    v_ego=v_ego,
    v_ego_starting=v_ego_starting,
    lead_status=lead_status,
    lead_v=lead_v,
    lead_d_rel=lead_d_rel,
  )
  return not lead_departing


def should_apply_low_speed_stopped_lead_glide_accel_cap(cp) -> bool:
  return getattr(cp, "carFingerprint", None) == HYUNDAI_CAR.HYUNDAI_SANTA_FE_HEV_2022


def should_apply_stop_entry_handoff_soften(
  v_ego: float,
  a_ego: float,
  a_target: float,
  last_output_accel: float,
  distance_to_stop_target_m: float | None,
) -> bool:
  if not (0.35 < v_ego < 2.30):
    return False
  if not (-1.05 < a_ego < -0.42):
    return False
  if last_output_accel > -0.48 or last_output_accel < -0.88:
    return False
  target_floor = interp(v_ego, [0.35, 0.60, 1.00, 1.50, 2.30], [-0.20, -0.28, -0.38, -0.50, -0.60])
  if a_target < target_floor:
    return False
  if distance_to_stop_target_m is not None and 0.0 <= distance_to_stop_target_m < 0.22:
    return False
  return True


def stop_entry_handoff_accel_cap(v_ego: float, distance_to_stop_target_m: float | None) -> float:
  speed_cap = interp(v_ego, [0.35, 0.60, 1.00, 1.50, 2.30], [-0.44, -0.48, -0.56, -0.64, -0.74])
  if distance_to_stop_target_m is None or distance_to_stop_target_m <= 0.0:
    return speed_cap
  distance_cap = interp(distance_to_stop_target_m, [0.22, 0.40, 0.75, 1.20, 2.00], [-0.72, -0.66, -0.58, -0.52, -0.46])
  return min(speed_cap, distance_cap)


def should_hold_stop_target_dropout(
  v_ego: float,
  a_target: float | None,
  distance_to_stop_target_m: float | None,
  last_distance_to_stop_target_m: float | None,
  last_output_accel: float,
  time_since_stop_intent_s: float,
) -> bool:
  if a_target is None or distance_to_stop_target_m is None or distance_to_stop_target_m <= 0.0:
    return False
  if last_distance_to_stop_target_m is None or last_distance_to_stop_target_m <= 0.0:
    return False
  if not (0.0 < v_ego < 0.22):
    return False
  if last_output_accel > -0.12:
    return False
  if time_since_stop_intent_s > 0.35:
    return False

  hold_distance_limit = interp(v_ego, [0.00, 0.08, 0.16, 0.22], [1.05, 0.98, 0.92, 0.86])
  if distance_to_stop_target_m > hold_distance_limit:
    return False

  growth_allowance = interp(v_ego, [0.00, 0.08, 0.16, 0.22], [0.06, 0.08, 0.10, 0.12])
  if distance_to_stop_target_m > (last_distance_to_stop_target_m + growth_allowance):
    return False

  a_target_ceiling = interp(v_ego, [0.00, 0.08, 0.16, 0.22], [0.30, 0.26, 0.20, 0.14])
  return a_target <= (a_target_ceiling + 1e-6)


def should_hold_no_target_standstill_dropout(
  v_ego: float,
  standstill: bool,
  force_coast: bool,
  a_target: float | None,
  distance_to_stop_target_m: float | None,
  last_output_accel: float,
  time_since_stop_intent_s: float,
) -> bool:
  if not standstill and v_ego > 0.06:
    return False
  if force_coast and standstill:
    return True
  if distance_to_stop_target_m is not None and distance_to_stop_target_m >= 0.0:
    return False
  if last_output_accel > -0.08:
    return False
  if time_since_stop_intent_s > 1.40:
    return False
  return a_target is None or a_target <= 0.12


def should_hold_low_speed_stop_target_release(
  v_ego: float,
  a_target: float | None,
  distance_to_stop_target_m: float | None,
  last_distance_to_stop_target_m: float | None,
  last_output_accel: float,
  time_since_stop_intent_s: float,
) -> bool:
  if a_target is None or distance_to_stop_target_m is None or distance_to_stop_target_m <= 0.0:
    return False
  if last_distance_to_stop_target_m is None or last_distance_to_stop_target_m <= 0.0:
    return False
  if not (0.0 < v_ego < 0.22):
    return False
  if time_since_stop_intent_s > 0.8:
    return False
  if last_output_accel > -0.18:
    return False

  hold_distance_floor = interp(v_ego, [0.00, 0.04, 0.08, 0.12, 0.22], [0.56, 0.52, 0.46, 0.38, 0.30])
  if distance_to_stop_target_m < hold_distance_floor:
    return False

  growth_allowance = interp(v_ego, [0.00, 0.04, 0.08, 0.12, 0.22], [0.08, 0.10, 0.12, 0.15, 0.18])
  if distance_to_stop_target_m > (last_distance_to_stop_target_m + growth_allowance):
    return False

  release_accel_ceiling = interp(v_ego, [0.00, 0.04, 0.08, 0.12, 0.22], [0.95, 0.82, 0.62, 0.42, 0.18])
  return a_target <= (release_accel_ceiling + 1e-6)


# --- public decision interface (spec section 2) ---------------------------------------------------


class StopSource(IntEnum):
  NONE = 0
  PLANNER = 1            # raw plan shouldStop
  EXPLICIT_TARGET = 2    # planner distanceToStopTarget mode entry/hold
  STOPPED_LEAD = 3       # synthetic stopped-lead target
  DROPOUT_HOLD = 4       # falling-edge hold of a recently-active source (timed)
  FORCE_COAST_STANDSTILL = 5
  STOPPED_LEAD_LATCH = 6  # close-stopped-lead standstill latch -- UNBOUNDED while the lead stays (spec 5.2.5; port of longcontrol.py:316-358)


@dataclass(frozen=True)
class StopDecision:
  stop_request_active: bool      # == today's stop_request_active (drives controller should_stop)
  state_should_stop: bool        # == today's state_should_stop (drives long_control_state_trans)
  target_distance_m: float       # arbitrated control target; -1.0 = none
  source: StopSource
  approach_cap_active: bool      # pid-state approach cap mode (today's stop_target_approach_active)
  carry_floor_active: bool       # pid-state carry floor mode (today's stop_target_carry_active)
  # The three release/ready booleans are INDEPENDENT in legacy code (longcontrol.py:996/:998/:1014-1024)
  # and can be simultaneously true -- a single reason string is lossy (a slow-departing lead at gap > 5.8 m
  # satisfies both departing and far-stopped predicates on the same frame). Each downstream consumer
  # (allow_fast_release, starting-state fast release, crawl/settle cap + brake floor gating) reads its own bit.
  departing_lead_release: bool   # legacy departing_lead_release (:774) -- drives allow_fast_release
  departing_lead_ready: bool     # legacy departing_lead_ready (:763) -- starting-state fast release
  far_stopped_lead_release: bool  # legacy far_stopped_lead_gap_release (:778) -- gates crawl/settle caps + brake floor
  legacy_forced: bool            # True when intent comes from a longcontrol forced hold (close_stopped_lead_dropout_hold /
                                 # stop_target_release_hold / force_coast_standstill_hold, longcontrol.py:793-830) -- the
                                 # intent-tiering consumers (creep_rebound_guard / standstill-relax) treat these as FULL
                                 # stop intent (spec 5.2.5): gate on
                                 # `stop_request_active and (source != DROPOUT_HOLD or legacy_forced)`;
                                 # only the controller tail_commit latch is excluded.
  release_reason: str            # 'departing_lead' | 'far_stopped_lead' | '' (telemetry only; derived, never consumed by control)
  # Appended beyond the spec section-2 field list (documented WP4 deviation): port of the
  # post-transition dropout holds (longcontrol.py:843-868). These never set stop_request_active in
  # legacy code -- they only pin the state machine -- so folding them into state_should_stop would
  # break the Commit B trace-equality gate. WP7 applies, after long_control_state_trans:
  #   if current_state == stopping and new_state != stopping and decision.state_dropout_hold:
  #     new_state = stopping
  state_dropout_hold: bool = False


class StopTargetArbiter:
  """One owner of stop intent + stop target. See module docstring for wiring/authority rules.

  Cumulative shadow-divergence counters (spec 3.2 row 1; survive reset(), instrumented by WP7
  telemetry from Commit C):
    legacy_hold_fired   -- rising-edge count of load-bearing legacy dropout-hold episodes
    single_hold_covered -- episodes fully covered by the consolidated hold + STOPPED_LEAD_LATCH
    hold_divergence     -- legacy-hold frames NOT covered by the consolidated mechanism
                           (brake-pressed frames excluded: a brake press disengages openpilot,
                           spec 5.5.6, so legacy-vs-latch disagreement there is transitional)
  Retirement trigger: legacy_hold_fired == single_hold_covered and hold_divergence == 0 over
  >= 25 events during the soak.
  """

  def __init__(self, CP, params: StoppingParams = STOPPING_PARAMS):
    self.CP = CP
    self.params = params
    # Verbatim fingerprint gate (longcontrol.py:737, :779, :794): the synthetic stopped-lead
    # target, far-stopped-lead release and close-stopped-lead hold are Santa-Fe-quirk-layer gated.
    self._quirk_layer_enabled = should_apply_low_speed_stopped_lead_glide_accel_cap(CP)

    # cumulative shadow counters -- deliberately NOT cleared by reset()
    self.legacy_hold_fired = 0
    self.single_hold_covered = 0
    self.hold_divergence = 0
    self._legacy_episode_active = False
    self._legacy_episode_uncovered = False

    # consolidated-hold telemetry (refreshed every update)
    self.consolidated_hold_active = False
    self.consolidated_hold_source = StopSource.NONE
    self.consolidated_target_m = -1.0

    self.time_since_standstill_s = _LEGACY_TIMER_CAP_S
    self.time_since_stop_intent_s = _LEGACY_TIMER_CAP_S
    # dt of the PREVIOUS update() call: the deferred frame-(k-1) intent-timer accumulation must use
    # dt_{k-1} (legacy :893-897 accumulates at the END of frame k with dt_k); identical under
    # constant dt, divergent per accumulating frame under logged dt jitter.
    self._pending_dt: float | None = None
    self._last_target_distance_m: float | None = None
    self._prev_intent_flags = False
    self._drop_kind: str | None = None      # None | 'rolling' | 'release' | 'standstill'
    self._time_since_drop_s = 0.0
    self._latched_target_m = -1.0
    self._had_primary_intent = False
    self._prev_latch_active = False
    self._last_primary_source = StopSource.NONE

  def reset(self) -> None:
    """Mirrors the arbiter-relevant part of LongControl.reset() (longcontrol.py:610-616).

    Ordering note vs HEAD (3be25f5240): update() stores _last_target_distance_m BEFORE
    LongControl's off/starting branches call this reset, so such frames end with it None where
    HEAD's tail assignment (longcontrol.py:1124-1128) ran AFTER reset and kept the frame value.
    Non-load-bearing: both consumers (should_hold_stop_target_dropout, :268;
    should_hold_low_speed_stop_target_release, :321) require last_distance > 0 AND an intent
    timer <= 0.35/0.8 s that this same reset pins to the 10.0 cap; the timer can only re-arm via
    a later update() frame, which rewrites _last_target_distance_m first. A future consumer of
    _last_target_distance_m that is NOT intent-timer-gated would inherit the divergence —
    re-prime from LongControl after reset() in that case.
    """
    self._finalize_legacy_episode()
    self.time_since_standstill_s = _LEGACY_TIMER_CAP_S
    self.time_since_stop_intent_s = _LEGACY_TIMER_CAP_S
    self._pending_dt = None
    self._last_target_distance_m = None
    self._prev_intent_flags = False
    self._drop_kind = None
    self._time_since_drop_s = 0.0
    self._latched_target_m = -1.0
    self._had_primary_intent = False
    self._prev_latch_active = False
    self._last_primary_source = StopSource.NONE
    self.consolidated_hold_active = False
    self.consolidated_hold_source = StopSource.NONE
    self.consolidated_target_m = -1.0

  def projected_time_since_stop_intent_s(self, decision: StopDecision, post_transition_long_control_state: int, dt: float) -> float:
    """This frame's end-of-frame intent timer exactly as legacy computes it (longcontrol.py:893-897).

    The arbiter itself applies this update at the top of the NEXT update() call (identical values --
    the next call's long_control_state input is this frame's post-transition state). Legacy's
    same-frame consumer ``stop_intent_recent = time_since_stop_intent_s < 1.0`` (:900) needs the
    frame-k value, so WP7 reads it from here.
    """
    stop_intent_active = (decision.stop_request_active or decision.approach_cap_active or decision.carry_floor_active
                          or post_transition_long_control_state == LONG_CTRL_STATE_STOPPING)
    if stop_intent_active:
      return 0.0
    return min(self.time_since_stop_intent_s + dt, _LEGACY_TIMER_CAP_S)

  def _finalize_legacy_episode(self) -> None:
    if self._legacy_episode_active:
      if not self._legacy_episode_uncovered:
        self.single_hold_covered += 1
      self._legacy_episode_active = False
      self._legacy_episode_uncovered = False

  def update(self, *, v_ego: float, a_ego: float, a_target: float,
             raw_should_stop: bool, planner_target_m: float,
             lead_status: bool, lead_v: float, lead_d_rel: float,
             increased_stopped_distance_m: float,
             brake_pressed: bool, cruise_standstill: bool, standstill: bool,
             force_coast: bool, long_control_state: int,
             last_output_accel: float, dt: float,
             human_acceleration: bool, v_ego_starting: float) -> StopDecision:
    """Verbatim port of the longcontrol.py arbitration block (:729-868).

    ``planner_target_m`` keeps the planner's float sentinel (-1.0 = none, > 0.0 = explicit;
    longitudinalPlan.distanceToStopTarget arrives as a float, never None).
    ``standstill`` is the vehicle flag (CS.standstill); the legacy composite (:762) is rebuilt
    here as ``standstill or cruise_standstill``. ``a_ego`` and ``increased_stopped_distance_m``
    are spec-interface inputs unused by the v1 verbatim rules. ``human_acceleration`` /
    ``v_ego_starting`` are the frogpilot_toggles fields the legacy departing-lead and
    close-stopped-lead predicates consume (:764, :769, :799-801) -- required kwargs appended to
    the spec signature (documented WP4 deviation).
    """
    del a_ego, increased_stopped_distance_m  # spec interface; unused by the v1 verbatim port

    # Legacy updates the intent timer at the END of each frame using that frame's post-transition
    # state (:893-897); this frame's long_control_state input IS that state, so applying the
    # previous frame's update here first is value-identical.
    prev_stop_intent_active = self._prev_intent_flags or long_control_state == LONG_CTRL_STATE_STOPPING
    if prev_stop_intent_active:
      self.time_since_stop_intent_s = 0.0
    else:
      prev_dt = self._pending_dt if self._pending_dt is not None else dt
      self.time_since_stop_intent_s = min(self.time_since_stop_intent_s + prev_dt, _LEGACY_TIMER_CAP_S)

    standstill_eff = bool(standstill) or bool(cruise_standstill)  # :762
    prev_distance_to_stop_target_m = self._last_target_distance_m  # :730

    # synthetic stopped-lead target + min-merge (:731-746)
    stopped_lead_control_target_m = (
      get_stopped_lead_control_target(
        v_ego=v_ego,
        lead_v=float(lead_v),
        lead_d_rel=float(lead_d_rel),
      )
      if bool(lead_status) and self._quirk_layer_enabled
      else None
    )
    control_distance_to_stop_target_m = planner_target_m
    if stopped_lead_control_target_m is not None and (
      control_distance_to_stop_target_m is None
      or control_distance_to_stop_target_m <= 0.0
      or stopped_lead_control_target_m < control_distance_to_stop_target_m
    ):
      control_distance_to_stop_target_m = stopped_lead_control_target_m

    # primary intent + approach/carry modes (:750-761)
    stopped_lead_control_stop_active = stopped_lead_control_target_m is not None
    stop_target_request = should_enter_stop_target_mode(v_ego, a_target, control_distance_to_stop_target_m)
    stop_request_active = bool(raw_should_stop) or stop_target_request or stopped_lead_control_stop_active
    stop_target_approach_active = (
      not stop_request_active
      and should_apply_stop_target_approach_mode(v_ego, a_target, control_distance_to_stop_target_m)
    )
    stop_target_carry_active = (
      not stop_request_active
      and not stop_target_approach_active
      and should_apply_stop_target_carry_mode(v_ego, a_target, control_distance_to_stop_target_m)
    )

    # releases (:763-792)
    departing_lead_ready = should_release_stop_hold_for_departing_lead(
      human_acceleration=bool(human_acceleration),
      output_should_stop=True,
      force_coast=bool(force_coast),
      standstill=standstill_eff,
      v_ego=float(v_ego),
      v_ego_starting=float(v_ego_starting),
      lead_status=bool(lead_status),
      lead_v=float(lead_v),
      lead_d_rel=float(lead_d_rel),
    )
    departing_lead_release = bool(raw_should_stop) and departing_lead_ready
    if departing_lead_release:
      stop_request_active = False
      stop_target_approach_active = False
    far_stopped_lead_gap_release = (
      self._quirk_layer_enabled
      and not force_coast
      and should_release_far_stopped_lead_gap(
        v_ego=v_ego,
        lead_status=bool(lead_status),
        lead_v=float(lead_v),
        lead_d_rel=float(lead_d_rel),
        distance_to_stop_target_m=control_distance_to_stop_target_m,
      )
    )
    if far_stopped_lead_gap_release:
      stop_request_active = False
      stop_target_approach_active = False
      stop_target_carry_active = False

    # un-held snapshot (consumed by the promoted-consolidated branch + divergence counters)
    stop_request_before_holds = stop_request_active
    approach_before_holds = stop_target_approach_active
    carry_before_holds = stop_target_carry_active

    # legacy forced holds (:793-829)
    close_stopped_lead_dropout_hold_active = (
      self._quirk_layer_enabled
      and not departing_lead_release
      and not far_stopped_lead_gap_release
      and should_hold_recent_close_stopped_lead_dropout(
        v_ego=v_ego,
        v_ego_starting=float(v_ego_starting),
        standstill=standstill_eff,
        time_since_standstill_s=self.time_since_standstill_s,
        lead_status=bool(lead_status),
        lead_v=float(lead_v),
        lead_d_rel=float(lead_d_rel),
        distance_to_stop_target_m=control_distance_to_stop_target_m,
        force_coast=bool(force_coast),
      )
    )
    if close_stopped_lead_dropout_hold_active:
      stop_request_active = True
      stop_target_approach_active = False
      stop_target_carry_active = False
    stop_target_release_hold_active = (
      not departing_lead_release
      and not far_stopped_lead_gap_release
      and not close_stopped_lead_dropout_hold_active
      and should_hold_low_speed_stop_target_release(
        v_ego=v_ego,
        a_target=a_target,
        distance_to_stop_target_m=control_distance_to_stop_target_m,
        last_distance_to_stop_target_m=prev_distance_to_stop_target_m,
        last_output_accel=last_output_accel,
        time_since_stop_intent_s=self.time_since_stop_intent_s,
      )
    )
    if stop_target_release_hold_active:
      stop_request_active = True
      stop_target_approach_active = False
      stop_target_carry_active = False
    force_coast_standstill_hold = bool(force_coast) and standstill_eff  # :830
    state_should_stop = (
      bool(raw_should_stop)
      or stopped_lead_control_stop_active
      or close_stopped_lead_dropout_hold_active
      or stop_target_release_hold_active
      or force_coast_standstill_hold
    ) and not departing_lead_release and not far_stopped_lead_gap_release  # :831-837

    # post-transition dropout holds (:843-868) -- evaluated from the current (pre-transition)
    # state; WP7 applies state_dropout_hold only when the transition would leave stopping.
    state_dropout_hold = False
    if long_control_state == LONG_CTRL_STATE_STOPPING and not departing_lead_release and not far_stopped_lead_gap_release:
      state_dropout_hold = (
        should_hold_stop_target_dropout(
          v_ego=v_ego,
          a_target=a_target,
          distance_to_stop_target_m=control_distance_to_stop_target_m,
          last_distance_to_stop_target_m=prev_distance_to_stop_target_m,
          last_output_accel=last_output_accel,
          time_since_stop_intent_s=self.time_since_stop_intent_s,
        )
        or should_hold_no_target_standstill_dropout(
          v_ego=v_ego,
          standstill=standstill_eff,
          force_coast=bool(force_coast),
          a_target=a_target,
          distance_to_stop_target_m=control_distance_to_stop_target_m,
          last_output_accel=last_output_accel,
          time_since_stop_intent_s=self.time_since_stop_intent_s,
        )
      )

    # --- consolidated dual-window holds + STOPPED_LEAD_LATCH (spec 5.2.5; shadow-only while
    # ARBITER_LEGACY_DROPOUT_HOLDS is True) ---------------------------------------------------
    # Timer base (spec amendment, recorded in WP4 deviations): the legacy hold predicates compare
    # against time_since_stop_intent_s, which stays PINNED AT ZERO while the state machine remains
    # in stopping (stop_intent_active includes `long_control_state == stopping`, :893) -- the
    # legacy "1.4 s" standstill hold is therefore self-sustaining and bounded only by its escape
    # terms. The consolidated standstill window keeps that exact comparison (so the row-1
    # divergence trigger stays attainable); the NEW rolling window (0.4 s) and the explicit-target
    # release window (0.8 s) run on real time since the drop, with the release hold additionally
    # sustained while the target is still present (matching the legacy release-hold's
    # target-present requirement).
    # rule-4 releases are the LEGACY release booleans (the post-transition holds at :843 are gated
    # on exactly these); the latch additionally embeds its own ready-style departing check inside
    # the verbatim predicate, so a green-light departure still releases it during a dropout.
    released = departing_lead_release or far_stopped_lead_gap_release
    primary_intent = (bool(raw_should_stop) or stop_target_request or stopped_lead_control_stop_active) and not released
    early_release = (released or bool(brake_pressed)
                     or (v_ego >= CONSOLIDATED_EARLY_RELEASE_V and a_target > CONSOLIDATED_EARLY_RELEASE_A_TARGET))
    # The latch is the same verbatim predicate the legacy close hold uses -- coverage of that
    # legacy class is by construction. It additionally releases on brake press (spec 5.2.5).
    consolidated_latch_active = close_stopped_lead_dropout_hold_active and not bool(brake_pressed)
    latch_fell = self._prev_latch_active and not consolidated_latch_active

    if primary_intent:
      self._drop_kind = None
      self._time_since_drop_s = 0.0
      self._had_primary_intent = True
      if bool(raw_should_stop):
        self._last_primary_source = StopSource.PLANNER
      elif stop_target_request:
        self._last_primary_source = StopSource.EXPLICIT_TARGET
      else:
        self._last_primary_source = StopSource.STOPPED_LEAD
      self._latched_target_m = (
        float(control_distance_to_stop_target_m)
        if control_distance_to_stop_target_m is not None and control_distance_to_stop_target_m > 0.0
        else -1.0
      )
    elif early_release:
      self._drop_kind = None
      self._had_primary_intent = False
    elif self._had_primary_intent or latch_fell:
      # falling edge of a recently-active source -- primary intent, or the latch handing over
      # (e.g. the lead departs at standstill and the legacy no-target standstill hold takes over,
      # which is a stateless predicate with no arming edge of its own): classify per spec 5.2.5
      self._had_primary_intent = False
      self._time_since_drop_s = 0.0
      close_stopped_lead_present = bool(lead_status) and 0.0 < float(lead_d_rel) <= self.params.FAR_CRAWL_GAP_M
      target_present = control_distance_to_stop_target_m is not None and control_distance_to_stop_target_m > 0.0
      if (standstill_eff or v_ego <= self.params.V_SETTLE) and not target_present and not close_stopped_lead_present:
        self._drop_kind = 'standstill'
      elif (self._last_primary_source == StopSource.EXPLICIT_TARGET or self._latched_target_m > 0.0) and v_ego < RELEASE_HOLD_V_MAX:
        self._drop_kind = 'release'
      else:
        self._drop_kind = 'rolling'
    elif self._drop_kind is not None:
      self._time_since_drop_s += dt

    timed_hold_active = False
    if self._drop_kind is not None and not primary_intent and not early_release:
      target_present_now = control_distance_to_stop_target_m is not None and control_distance_to_stop_target_m > 0.0
      if self._drop_kind == 'standstill':
        # legacy validity + escape terms carried over (longcontrol.py:464-475); 0.06 is the
        # legacy rolling bound (:465), numerically == params.V_SETTLE
        timed_hold_active = (
          (standstill_eff or v_ego <= 0.06)
          and self.time_since_stop_intent_s <= self.params.T_STOP_INTENT_HOLD_STANDSTILL_S
          and a_target <= self.params.HOLD_ESCAPE_A_TARGET
          and last_output_accel <= self.params.HOLD_ESCAPE_LAST_OUTPUT
        )
      elif self._drop_kind == 'release':
        timed_hold_active = self._time_since_drop_s <= RELEASE_HOLD_WINDOW_S or target_present_now
      else:  # 'rolling'
        timed_hold_active = self._time_since_drop_s <= self.params.T_STOP_INTENT_HOLD_S
      if not timed_hold_active:
        self._drop_kind = None

    self._prev_latch_active = consolidated_latch_active
    self.consolidated_hold_active = (timed_hold_active or consolidated_latch_active) and not primary_intent
    if consolidated_latch_active:
      self.consolidated_hold_source = StopSource.STOPPED_LEAD_LATCH
    elif timed_hold_active:
      self.consolidated_hold_source = StopSource.DROPOUT_HOLD
    else:
      self.consolidated_hold_source = StopSource.NONE
    if self.consolidated_hold_active and timed_hold_active and not consolidated_latch_active:
      if control_distance_to_stop_target_m is not None and control_distance_to_stop_target_m > 0.0:
        self.consolidated_target_m = float(control_distance_to_stop_target_m)
      else:
        self.consolidated_target_m = self._latched_target_m
    else:
      self.consolidated_target_m = (
        float(control_distance_to_stop_target_m)
        if control_distance_to_stop_target_m is not None and control_distance_to_stop_target_m > 0.0
        else -1.0
      )

    # --- divergence counters (spec 3.2 row 1) -------------------------------------------------
    # Load-bearing legacy holds only: forced holds that flipped stop_request, and post-transition
    # holds on frames where state_should_stop alone would have let the state machine exit.
    legacy_hold_load_bearing = (
      ((close_stopped_lead_dropout_hold_active or stop_target_release_hold_active) and not stop_request_before_holds)
      or (state_dropout_hold and not state_should_stop)
    )
    if legacy_hold_load_bearing and not bool(brake_pressed):
      if not self._legacy_episode_active:
        self._legacy_episode_active = True
        self._legacy_episode_uncovered = False
        self.legacy_hold_fired += 1
      if not self.consolidated_hold_active:
        self.hold_divergence += 1
        self._legacy_episode_uncovered = True
    else:
      self._finalize_legacy_episode()

    # --- promoted consolidated holds (post-cleanup semantics; dark while the flag is True) ----
    if not ARBITER_LEGACY_DROPOUT_HOLDS:
      stop_request_active = stop_request_before_holds
      stop_target_approach_active = approach_before_holds
      stop_target_carry_active = carry_before_holds
      close_stopped_lead_dropout_hold_active = False
      stop_target_release_hold_active = False
      state_should_stop = (
        (bool(raw_should_stop) or stopped_lead_control_stop_active or force_coast_standstill_hold)
        and not departing_lead_release and not far_stopped_lead_gap_release
      )
      state_dropout_hold = False
      if self.consolidated_hold_active:
        stop_request_active = True
        stop_target_approach_active = False
        stop_target_carry_active = False
        state_should_stop = True
        state_dropout_hold = True

    # --- source + telemetry fields -------------------------------------------------------------
    legacy_forced = close_stopped_lead_dropout_hold_active or stop_target_release_hold_active or force_coast_standstill_hold
    if not ARBITER_LEGACY_DROPOUT_HOLDS and self.consolidated_hold_active:
      # promoted holds replace the longcontrol forced holds, so they carry full stop intent
      legacy_forced = True

    if stop_request_active or state_should_stop:
      if bool(raw_should_stop) and stop_request_active:
        source = StopSource.PLANNER
      elif stop_target_request and stop_request_active:
        source = StopSource.EXPLICIT_TARGET
      elif stopped_lead_control_stop_active and stop_request_active:
        source = StopSource.STOPPED_LEAD
      elif not ARBITER_LEGACY_DROPOUT_HOLDS and self.consolidated_hold_active:
        source = self.consolidated_hold_source
      elif close_stopped_lead_dropout_hold_active:
        source = StopSource.STOPPED_LEAD_LATCH
      elif stop_target_release_hold_active:
        source = StopSource.DROPOUT_HOLD
      elif force_coast_standstill_hold:
        source = StopSource.FORCE_COAST_STANDSTILL
      else:
        source = StopSource.NONE
    elif state_dropout_hold:
      source = StopSource.DROPOUT_HOLD
    else:
      source = StopSource.NONE

    if departing_lead_release:
      release_reason = 'departing_lead'
    elif far_stopped_lead_gap_release:
      release_reason = 'far_stopped_lead'
    else:
      release_reason = ''

    target_distance_m = (
      float(control_distance_to_stop_target_m)
      if control_distance_to_stop_target_m is not None and control_distance_to_stop_target_m > 0.0
      else -1.0
    )

    # --- end-of-frame state (legacy :888-891, :893-897 deferred to next call, :1124-1128) ------
    if standstill_eff:
      self.time_since_standstill_s = 0.0
    else:
      self.time_since_standstill_s = min(self.time_since_standstill_s + dt, _LEGACY_TIMER_CAP_S)
    self._prev_intent_flags = stop_request_active or stop_target_approach_active or stop_target_carry_active
    self._last_target_distance_m = (
      float(control_distance_to_stop_target_m)
      if control_distance_to_stop_target_m is not None and control_distance_to_stop_target_m > 0.0
      else None
    )
    self._pending_dt = dt

    return StopDecision(
      stop_request_active=stop_request_active,
      state_should_stop=state_should_stop,
      target_distance_m=target_distance_m,
      source=source,
      approach_cap_active=stop_target_approach_active,
      carry_floor_active=stop_target_carry_active,
      departing_lead_release=departing_lead_release,
      departing_lead_ready=departing_lead_ready,
      far_stopped_lead_release=far_stopped_lead_gap_release,
      legacy_forced=legacy_forced,
      release_reason=release_reason,
      state_dropout_hold=state_dropout_hold,
    )
