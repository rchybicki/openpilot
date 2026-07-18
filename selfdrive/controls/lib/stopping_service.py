"""StoppingService -- the Stopping Service V3 phase machine (docs/stopping/stopping_service_v3_plan.md).

STAGE 0/1 module: pure python + math, no cereal/capnp imports. In LIVE stages this is the SOLE
stopping-band writer (plan §2); in stage 1 it runs SHADOW-only (its output never reaches the wire).

Law -> plan §3 map (every constant is verbatim from the plan's constants block):

  ENTRY (INACTIVE -> APPROACH_GLIDE)   engaged AND scope AND v < V_ENTER AND
                                       (shouldStop OR (lead_confirmed_stopped AND d_rem < 15))
  d_rem (lead)                         d_gap - D_REST_eff; at entry D_REST_eff =
                                       min(D_REST_NOM, max(d_gap_entry - v_entry^2/(2*A_REST_FEAS), D_REST_MIN)),
                                       re-computed only if d_gap grows > 1.0 m (ledger D1-H2: rest re-zeroed
                                       for close entries, "remaining = 0 while resting normally" cannot occur)
  d_rem (no-lead)                      max(dts_planner, v^2/(2*A_SETTLE_REF)) continuous (ledger D3-H3 /
                                       D2's d_settle envelope; no stop-point latch, no staleness possible);
                                       both targets present => min
  APPROACH_GLIDE                       a_phase = clip(-v^2/(2*max(d_rem, 0.15)) - a_coast, planner_min_limit, -0.03);
                                       below v = 0.1 the HELD a_coast is consumed deepen-only (plan §3: a held
                                       negative drag value must never shallow the terminal demand)
  PRE_STOP_EASE                        gates: v <= V_EASE AND d_rem <= 0.8 (exit hysteresis: back to GLIDE only
                                       once d_rem > 0.95) AND d_gap > 2.6 AND v_lead >= -0.1;
                                       any gate fails => GLIDE law reached at J_SAFE (ledger D3-H2).
                                       a_phase = clip(clip(a_stop, -0.35, -0.10) - clip(a_coast, 0, 0.4), -0.35, -0.03)
                                       where a_stop is the pure kinematic stop demand (no a_coast term: creep/grade
                                       enters EASE once, via the deepen-only feedforward -- D1 graft)
  ANTI-HOVER/ANTI-ROLL MONITOR         (EASE/RAMP/HOLD, plus APPROACH_GLIDE once v <= V_EASE: at walking pace with
                                       stop intent, "v not decreasing" is a fault) v >= 0.03 not decreasing
                                       >= 0.02 m/s per 0.4 s, or v rising > 0.06 above the running min => deepen at
                                       J_SAFE to min(current, -0.35), escalate -0.15 per 0.5 s UNBOUNDED; the
                                       triggered floor is a ratchet cleared only on RELEASE/INACTIVE (never on
                                       EASE<->GLIDE flips), and post-latch, escapes are caught by a ROLL trigger
                                       (v >= 0.05 and rising) plus a DISPLACEMENT lane (trusted gap shrinking
                                       > 0.15 m below its latch reference -- catches sub-quantization crawls);
                                       in GLIDE/EASE the TRIGGER is suppressed while a present lead's conditioned
                                       gap is GROWING (> 0.03 m per 0.4 s window -- queue-following creep behind a
                                       departing lead is not a fault, route 00001b72); an armed floor still binds
  RAMP_TO_HOLD                         from the first wheel-stop-latched frame, preserve natural arrival <=0.5 s,
                                       then ramp toward A_HOLD_SECURE (-0.70) at J_HOLD
  HOLD                                 continue/hold -0.70; a_kin stays live; NO post-stop motion lanes exist
  RELEASE                              planner go (a_target > 0.2 AND lead pull/gap growth, or no lead),
                                       sustained observed lead pull (gap grew 0.3 m AND relative speed > 0.5
                                       for 0.5 s), or state exit: ramp to 0 at J_GO
  SAFETY LANE (every phase)            a_kin  = -max(v_ego - v_lead, 0)^2 / (2*max(d_gap - D_HARD, 0.30))
                                       a_plan normally keeps final planner aTarget; with trustworthy conditioned
                                       lead geometry, the trajectory demand remains unmodified while only extra
                                       direct/composite depth is bounded to stop by 2.5 m (phase still targets 4 m;
                                       a_kin still protects D_HARD)
                                       dropout floor: while decay-holding the command may not release above -0.25
  FINAL JERK LIMITER (sole writer)     a_cmd = jerk_limit(min(a_phase, a_kin, a_plan)); deepen J_DOWN 2.5 comfort /
                                       J_SAFE 8.0 when a safety lane binds, release J_UP 1.5, build-toward-hold
                                       J_HOLD 0.6, release-to-go J_GO 1.2

P1 STRUCTURAL RULE (plan §2/§5, non-negotiable): direct/composite model demand is advisory and is
position-bounded before admission; the trajectory demand is never shallowed. Every admitted lane
then modifies the phase command through min() (deepen-only). PRE_STOP_EASE is the only shallow
phase region, bounded on speed, gap and lead motion, and its deepen lanes never disarm. Non-finite
anything => safe fallback (hold last command, or A_HOLD at standstill); the service never emits a
non-finite command.
"""

from __future__ import annotations

import enum
import math
from dataclasses import dataclass, field

from openpilot.selfdrive.controls.lib.stop_context import A_COAST_HOLD_V, StopSignals

_INF = float("inf")


def _finite(x) -> bool:
  return x is not None and math.isfinite(x)


def _clip(x: float, lo: float, hi: float) -> float:
  return min(max(x, lo), hi)


class Phase(enum.IntEnum):
  INACTIVE = 0
  APPROACH_GLIDE = 1
  PRE_STOP_EASE = 2
  RAMP_TO_HOLD = 3
  HOLD = 4
  RELEASE = 5


@dataclass(frozen=True)
class ServiceParams:
  """Constants exactly per plan §3 (all physical units; TRUE meters everywhere)."""
  V_ENTER: float = 2.5
  V_EASE: float = 0.50
  D_REST_NOM_BASE: float = 4.0     # + ISD, clipped [D_REST_CLIP_MIN, D_REST_CLIP_MAX]
  D_REST_CLIP_MIN: float = 2.5
  D_REST_CLIP_MAX: float = 5.0
  D_HARD: float = 2.0
  D_REST_MIN: float = 2.4
  A_GLIDE_NOM: float = 0.5
  A_REST_FEAS: float = 1.2         # rest-anchor FEASIBILITY decel (route 00001b76 seg4/5: anchoring with the
                                   # 0.5 comfort glide re-zeroed a NORMAL stop-and-go entry (gap 5.4 @ 1.65 m/s)
                                   # to a 2.7 m rest -> car stopped at 2.1 m. The anchor must ask "can the car
                                   # firmly land at D_REST_NOM" (planner was already demanding 0.8-1.2 there),
                                   # not "can the gentlest glide reach it"; genuine close entries (gap ~3.0)
                                   # still re-zero to D_REST_MIN..2.85)
  A_REANCHOR_HYST: float = 0.15    # do not move the rest target for numerical/plant excursions around the comfort law
  REANCHOR_REMAINING_MAX_M: float = 0.6  # re-anchor only in the BLOW-UP region (remaining collapsing toward the
                                         # 0.15 floor): a transient 0.7-1.0 demand MID-glide (remaining >0.6) is a
                                         # normal firm stop-and-go arrival and must not erode the nominal rest
                                         # (comfort-trigger alone dropped a normal rest 4.0 -> 3.49 in fixtures)
  REANCHOR_GAP_MARGIN_M: float = 1.0  # never relax for comfort inside 1 m of the minimum rest band
  REANCHOR_TOTAL_MAX_M: float = 0.4   # total comfort relief budget per stop: the blow-up fix needs ~0.2 m
                                      # (route 00001f0c); unbounded repeated re-anchoring on sustained-push
                                      # grades surrendered >1 m of position (crawl fixtures eroded to 2.5)
  A_EASE_CAP: float = -0.10
  A_EASE_DEEP: float = -0.35
  A_HOLD: float = -0.45            # route 00001b87 segs 1/3 (cycle-4 review): -0.32 (the force-coast-proven
                                   # magnitude) is MARGINALLY insufficient against this HEV's creep torque on
                                   # some stops -- the car broke loose from -0.32..-0.43 holds and the monitor
                                   # re-arrested at -0.65 every time (6-16 cm felt nudges). Deeper resting hold
                                   # is felt-neutral (pressure builds after wheel-stop) and deep holds release
                                   # cleanly on this car (observed resumes from -0.65/-0.8 ratcheted holds)
  A_HOLD_SECURE: float = -0.70     # cycle-6: 2/5 stops still micro-escaped (5-7 cm) from -0.44..-0.62
                                   # holds, and across two cycles EVERY arrest at -0.70 held (~20+ events)
                                   # -- the empirically-always-holds level. Built silently after A_HOLD is
                                   # reached (car parked, zero felt cost); chosen by the arrest ledger.
  A_DROPOUT_MIN: float = -0.25
  A_SETTLE_REF: float = 0.40
  J_DOWN: float = 2.5
  J_UP: float = 1.5
  J_SAFE: float = 8.0
  J_HOLD: float = 0.6
  J_GO: float = 1.2
  # law-internal bounds from the §3 equations
  D_REM_FLOOR: float = 0.15        # glide floored denominator = the d_settle envelope bound (D2-H2)
  A_PHASE_MAX: float = -0.03       # phase command never shallower than -0.03 while active
  A_KIN_DEN_FLOOR_M: float = 0.30
  EASE_D_REM_MAX: float = 0.8
  EASE_D_REM_EXIT: float = 0.95    # d_rem gate exit hysteresis: EASE -> GLIDE only once d_rem > 0.95
  EASE_GAP_MIN: float = 2.6
  EASE_LEAD_V_MIN: float = -0.1
  ENTRY_LEAD_D_REM_MAX: float = 15.0
  REST_RECALC_GROW_M: float = 1.0
  MON_V_MIN: float = 0.03
  MON_DECREASE_MPS: float = 0.02   # required decrease per MON_WINDOW_S
  MON_WINDOW_S: float = 0.4
  MON_RISE_MPS: float = 0.06
  MON_ESCALATE_STEP: float = 0.15
  MON_ESCALATE_PERIOD_S: float = 0.5
  MON_POSTSTOP_ARREST_EXTRA: float = 0.25  # post-stop escape: first arrest floor = A_HOLD - this (deep,
                                           # applied at J_SAFE) instead of the A_EASE_DEEP ladder start
  MON_GAP_GROW_M: float = 0.03     # queue-creep gate: conditioned gap growth per MON_WINDOW_S that marks a departing lead
  MON_LEAD_RECEDE_MPS: float = 0.15  # ...AND the lead must be measurably MOVING (Doppler lead_v): a STOPPED lead
                                     # must never suppress the monitor -- radar gap quantization steps (~0.1 m
                                     # > MON_GAP_GROW_M) and slow outward radar walks masquerade as "departing"
                                     # (offline-gate event 000016dd: lead_v 0.017 + one +0.099 m notch cost 0.32 m
                                     # of rest; Codex review: sustained outward bias while truly closing)
  RELEASE_A_TARGET_MIN: float = 0.2
  RELEASE_LEAD_PULL_MPS: float = 0.5
  RELEASE_GAP_GROW_M: float = 0.3
  RELEASE_LEAD_CONFIRM_S: float = 0.5
  ENTRY_SEED_ACCEL: float = -0.10  # limiter seed when no wire value is available at entry
  PLANNER_MIN_FALLBACK: float = -3.5
  NATURAL_ARRIVAL_GRACE_S: float = 0.50  # preserve the rolling arrival briefly after wheel-stop latch,
                                        # then build the secure hold before a slow roll needs fast arrest


@dataclass(frozen=True)
class ServiceResult:
  accel: float
  phase: Phase
  active: bool
  debug: dict = field(default_factory=dict)


def service_holds_stopping_state(phase: Phase) -> bool:
  """The service is the sole settled-stop authority until its RELEASE ramp becomes inactive."""
  return phase in (Phase.RAMP_TO_HOLD, Phase.HOLD, Phase.RELEASE)


@dataclass(frozen=True)
class Evidence:
  """Per-frame motion-evidence snapshot. Orthogonal channels, NOT one exclusive state: a rise can
  end the arrival grace (finish_roll) without arming the monitor (rolling), and vice versa."""
  crawl: bool               # trusted-gap deficit > 0.15 m since the crawl reference (displacement lane)
  genuinely_stopped: bool   # wheel-stop latched, v below the quantization step, no crawl deficit
  hover: bool               # v not decreasing over the monitor window (entry-graced epoch)
  rolling: bool             # v risen above the epoch minimum past the monitor's arm gates
  decreasing: bool          # genuine deceleration over the window (pauses escalation, keeps the floor)
  gap_growing: bool         # queue-creep gate input: conditioned gap growing behind a receding lead
  suppressed: bool          # inside the queue-suppression deadline (fresh hover window after it lifts)
  windows_valid: bool       # False while evidence short-circuited (stopped / detection off this frame)


class StandstillEvidence:
  """THE single owner of every motion-evidence baseline, window, reference, and epoch (cycle-11
  consolidation, docs/stopping/motion_estimator_design.md REVISION). Three of the five previous
  terminal-band defect classes were scattered evidence baselines going stale (dither false-arrest,
  entry-grace poisoning, crawl blindness); every baseline MUTATION now lives in this one object,
  advanced by the service after its phase transitions and reset with the service episode.
  Two epochs are explicit and intentional (sol red-team finding 3):
    ENTRY-GRACED  -- monitor history (v_min, hover window): evidence accumulates only 0.5 s after
                     entry, so pre-control momentum (an aborted go) never becomes fault evidence.
    LATCH-IMMEDIATE -- finish roll (_ramp epoch) and the crawl/displacement reference: motion after
                     the wheel-stop latch is a fault from the very first frame, no grace.
  The monitor keeps its ratchet POLICY (arm/escalate/pause); this object owns only EVIDENCE."""

  def __init__(self, params: ServiceParams):
    self.p = params
    self.reset()

  def reset(self) -> None:
    self.entry_t = -10.0                # last INACTIVE->ACTIVE transition (roll-reference grace)
    self.mon_v_min = _INF               # entry-graced epoch minimum (monitor roll reference)
    self.suppress_until = 0.0           # queue-creep gate: fresh hover window after suppression lifts
    self.ramp_v_min = _INF              # LATCH-IMMEDIATE epoch: any rise above it ends arrival grace
    self.latch_gap: float | None = None  # trusted-gap crawl reference (re-bases, see advance())
    self.gap_trust_lost = False         # re-base the crawl reference on the first trusted frame after
    self.hold_entry_gap: float | None = None  # release gating: departure = growth beyond this
    self.lead_departure_s = 0.0         # confirmed physical-departure dwell (release gating)
    self.v_hist: list[tuple[float, float]] = []
    self.gap_hist: list[tuple[float, float]] = []

  # -- episode events (called by the service at its own transition points) -------------------------
  def on_entry(self, t: float, v: float) -> None:
    self.entry_t = t
    self.suppress_until = t + 0.5  # entry grace covers the hover test too: pre-control momentum
                                   # (an aborted go) is not hover evidence
    self.mon_v_min = v
    self.v_hist = []
    self.gap_hist = []

  def on_wheel_latch(self, v: float, d_gap: float | None) -> None:
    self.ramp_v_min = v
    self.latch_gap = d_gap        # post-latch crawl reference (displacement-based arrest)
    self.hold_entry_gap = d_gap

  def on_release(self, v: float) -> None:
    self.mon_v_min = v
    self.v_hist = []
    self.gap_hist = []

  def finish_roll(self, v: float) -> bool:
    """LATCH-IMMEDIATE epoch: v rise above the post-latch minimum ends the arrival grace (grace
    yields to evidence). Distinct channel and threshold (+0.02) from the monitor's arm gates."""
    self.ramp_v_min = min(self.ramp_v_min, v)
    return v > self.ramp_v_min + 0.02

  def track_departure(self, observed: bool, dt: float) -> float:
    self.lead_departure_s = self.lead_departure_s + dt if observed else 0.0
    return self.lead_departure_s

  # -- the per-frame evidence advance (ONE call; ordering owned here, mirrors the monitor law) -----
  def advance(self, *, t: float, v: float, phase: Phase, wheel_stop: bool, lead: bool,
              d_gap: float | None, lead_v: float, gap_trusted: bool, monitored: bool) -> Evidence:
    # entry-graced epoch: during the grace the roll reference re-seeds continuously, so roll
    # evidence only accumulates from motion that begins under service control (route 00001ba2)
    if t - self.entry_t < 0.5:
      self.mon_v_min = v
    else:
      self.mon_v_min = min(self.mon_v_min, v)
    # displacement-based crawl evidence: computed BEFORE the stopped short-circuit -- a
    # sub-quantization crawl (v reading 0.0, true v ~0.015) is invisible to every velocity trigger
    # but still consumes gap. Deficit only on TRUSTED frames (measured, no dropout decay-hold);
    # the reference re-bases on the first trusted frame after an untrusted stretch, and re-bases
    # UPWARD only on genuine departure (> RELEASE_GAP_GROW_M) so radar notch oscillation never
    # builds a false deficit.
    crawl = False
    if phase in (Phase.RAMP_TO_HOLD, Phase.HOLD) and lead and d_gap is not None:
      if not gap_trusted:
        self.gap_trust_lost = True
      else:
        if self.gap_trust_lost or self.latch_gap is None:
          self.latch_gap = d_gap
          self.gap_trust_lost = False
        elif d_gap > self.latch_gap + self.p.RELEASE_GAP_GROW_M:
          self.latch_gap = d_gap  # genuine lead departure: measure any later crawl from here
        crawl = (self.latch_gap - d_gap) > 0.15
    if wheel_stop and v < self.p.MON_V_MIN and not crawl:
      # genuinely stopped: window evidence does not accumulate this frame (mirrors the pre-ledger
      # early-out exactly -- hists untouched, suppression deadline unmoved)
      return Evidence(crawl=False, genuinely_stopped=True, hover=False, rolling=False,
                      decreasing=False, gap_growing=False, suppressed=False, windows_valid=False)
    if t - self.entry_t >= 0.5:  # entry grace: pre-control momentum never enters the hover window
      self.v_hist.append((t, v))
    else:
      self.v_hist = []
    while self.v_hist and self.v_hist[0][0] < t - 2.0 * self.p.MON_WINDOW_S:
      self.v_hist.pop(0)
    if lead and d_gap is not None:
      self.gap_hist.append((t, d_gap))
      while self.gap_hist and self.gap_hist[0][0] < t - 2.0 * self.p.MON_WINDOW_S:
        self.gap_hist.pop(0)
    else:
      self.gap_hist = []  # lead lost: stale growth evidence must never suppress the trigger
    if not monitored:
      return Evidence(crawl=crawl, genuinely_stopped=False, hover=False, rolling=False,
                      decreasing=False, gap_growing=False, suppressed=False, windows_valid=False)
    v_then = None
    for (t_i, v_i) in self.v_hist:
      if t_i <= t - self.p.MON_WINDOW_S:
        v_then = v_i
    decreasing = v_then is not None and (v_then - v) >= self.p.MON_DECREASE_MPS
    hover = v >= self.p.MON_V_MIN and v_then is not None and not decreasing
    rolling = v > self.mon_v_min + self.p.MON_RISE_MPS
    if phase in (Phase.RAMP_TO_HOLD, Phase.HOLD):
      # post-latch Kalman dither (v reading 0.03-0.05 while physically stopped, never truly
      # decreasing) satisfied the hover test on nearly EVERY stop (cycle-5 grab): post-stop, only
      # a genuine ROLL clear of the dither band -- or the displacement lane -- is an escape.
      hover = False
      rolling = (rolling and v >= 0.05) or crawl
    gap_then = None
    for (t_i, g_i) in self.gap_hist:
      if t_i <= t - self.p.MON_WINDOW_S:
        gap_then = g_i
    gap_growing = (phase in (Phase.APPROACH_GLIDE, Phase.PRE_STOP_EASE)
                   and lead and d_gap is not None and gap_then is not None
                   and (d_gap - gap_then) > self.p.MON_GAP_GROW_M
                   and lead_v > self.p.MON_LEAD_RECEDE_MPS)
    if gap_growing:
      # while legitimately following a receding queue, the constant-v history is not fault
      # evidence: after the gate lifts (lead stops again), the hover detector gets ONE fresh
      # window before it may arm -- the ego's own glide starts decelerating within that budget
      self.suppress_until = t + self.p.MON_WINDOW_S
    return Evidence(crawl=crawl, genuinely_stopped=False, hover=hover, rolling=rolling,
                    decreasing=decreasing, gap_growing=gap_growing,
                    suppressed=t <= self.suppress_until, windows_valid=True)


class StoppingService:
  def __init__(self, params: ServiceParams | None = None):
    self.p = params if params is not None else ServiceParams()
    self.reset()

  def reset(self) -> None:
    self.phase = Phase.INACTIVE
    self._last_cmd = 0.0
    self._t = 0.0
    self._v = 0.0
    self._d_rest_eff: float | None = None
    self._reanchor_ref: float | None = None    # anchor value the comfort-relief budget measures from
    self._d_rest_calc_gap: float | None = None
    self._fast_deepen = False           # EASE gate-fail revert: reach the glide law at J_SAFE
    self._mon_active = False            # escalation running
    self._mon_triggered = False         # floor lane armed (ratchet: never disarms inside EASE/RAMP/HOLD)
    self._mon_floor = 0.0
    self._mon_escalate_t = 0.0
    self._ramp_t = 0.0                  # time in RAMP_TO_HOLD (gentle-finish window)
    self.ev = StandstillEvidence(self.p)  # ALL motion-evidence baselines/windows/references live here
    self._isd = 0.0
    self._should_stop = False

  @property
  def _hold_entry_gap(self) -> float | None:  # legacy poke point (test_longcontrol_service_live)
    return self.ev.hold_entry_gap

  @_hold_entry_gap.setter
  def _hold_entry_gap(self, value: float | None) -> None:
    self.ev.hold_entry_gap = value

  # -- targets ------------------------------------------------------------------------------------
  def _d_rest_nom(self, isd: float) -> float:
    isd = float(isd) if _finite(isd) else 0.0
    return _clip(self.p.D_REST_NOM_BASE + isd, self.p.D_REST_CLIP_MIN, self.p.D_REST_CLIP_MAX)

  def _update_d_rest_eff(self, d_gap: float | None, v: float, isd: float, lead_v: float = 0.0) -> None:
    """D_REST_eff at entry = min(D_REST_NOM, max(d_gap_entry - v_entry^2/(2*A_REST_FEAS), D_REST_MIN));
    re-computed only if d_gap grows > 1.0 m (plan §3 / ledger D1-H2). A_REST_FEAS is a FIRM
    feasibility bound, not the comfort glide (route 00001b76: the 0.5 comfort bound re-zeroed a
    normal stop-and-go entry to a 2.7 m rest -> stopped at 2.1 m). While the LEAD IS STILL MOVING
    the geometry is not final (the lead keeps adding gap until it stops), so the anchor keeps
    re-computing and freezes only once the lead is stopped."""
    if d_gap is None:
      return
    lead_still_moving = lead_v > 0.30
    stale = self._d_rest_calc_gap is not None and d_gap > self._d_rest_calc_gap + self.p.REST_RECALC_GROW_M
    if self._d_rest_eff is None or stale or lead_still_moving:
      landing = d_gap - (v * v) / (2.0 * self.p.A_REST_FEAS)
      self._d_rest_eff = min(self._d_rest_nom(isd), max(landing, self.p.D_REST_MIN))
      self._d_rest_calc_gap = d_gap
      self._reanchor_ref = self._d_rest_eff  # comfort-relief budget baseline (REANCHOR_TOTAL_MAX_M)

  def _d_rem(self, d_gap: float | None, dts: float | None, v: float) -> float | None:
    """min of the lead target (TRUE meters) and the envelope-conditioned no-lead target (plan §3)."""
    candidates = []
    if d_gap is not None and self._d_rest_eff is not None:
      # TERMINAL ANTI-BLOWUP (cycle-5, route 00001ba3 seg28): arriving hot, d_rem collapses to its
      # 0.15 m floor and the -v^2/2d law explodes (-1.76 while the planner relaxed to -0.88, IMU
      # jerk 14.6). The COMFORT law must never brake harder than A_GLIDE_NOM to defend a rest
      # POSITION -- re-anchor the rest CLOSER (one-way, only when the resulting rest remains in the
      # intended >= D_REST_CLIP_MIN band) so the glide demand caps at the nominal glide decel and
      # the car lands nearer instead of slamming. A_REST_FEAS
      # remains the firmer ENTRY reachability test above; conflating the two made a normal terminal
      # glide use the entry feasibility limit as its comfort target. TRIGGER = comfort + hysteresis
      # (route 00001f0c seg0, cycle-10): triggering only above A_REST_FEAS left a dead band
      # (~0.65..1.35) where demands were neither re-anchored nor comfortable -- ISD 0.3 collapsed
      # d_rem to 0.2 m at v 0.6 -> demand -0.91, +0.24 creep ff -> -1.27 wire, jerk 11.3. The
      # trigger and the target now use the same comfort law. Genuine threats are unaffected:
      # a_kin (D_HARD) and a_plan keep unlimited depth.
      current_remaining = max(d_gap - self._d_rest_eff, self.p.D_REM_FLOOR)
      current_decel = (v * v) / (2.0 * current_remaining)
      comfort_landing = d_gap - (v * v) / (2.0 * self.p.A_GLIDE_NOM)
      if (v >= self.p.MON_V_MIN and d_gap >= self.p.D_REST_CLIP_MIN + self.p.REANCHOR_GAP_MARGIN_M
          and self._d_rest_eff > self.p.D_REST_MIN
          and current_remaining <= self.p.REANCHOR_REMAINING_MAX_M
          and current_decel > self.p.A_GLIDE_NOM + self.p.A_REANCHOR_HYST
          and comfort_landing >= self.p.D_REST_CLIP_MIN):
        if comfort_landing < self._d_rest_eff:
          relief_floor = (self._reanchor_ref - self.p.REANCHOR_TOTAL_MAX_M
                          if self._reanchor_ref is not None else self.p.D_REST_MIN)
          self._d_rest_eff = max(comfort_landing, self.p.D_REST_MIN, relief_floor)
      candidates.append(d_gap - self._d_rest_eff)
    envelope = (v * v) / (2.0 * self.p.A_SETTLE_REF)
    if dts is not None:
      candidates.append(max(dts, envelope))
    elif not candidates and self._should_stop:
      candidates.append(envelope)  # shouldStop with no target at all: settle on the envelope
    return min(candidates) if candidates else None

  # -- phase laws (plan §3) -------------------------------------------------------------------------
  def _glide_demand(self, v: float, d_rem: float, a_coast: float, planner_min: float) -> float:
    # Below the a_coast hold speed (0.1 m/s, == stop_context.A_COAST_HOLD_V) the HELD a_coast is
    # consumed DEEPEN-ONLY (plan §3: "below v = 0.1 hold last value (used deepen-only there)"): only
    # a positive (push) residual may deepen the demand -- a held/poisoned negative (drag) value must
    # never shallow the terminal demand toward the -0.03 clip. Above the hold speed the live
    # estimate stays symmetric (grade compensation, plan §1 uphill sweep).
    coast_ff = max(a_coast, 0.0) if v < A_COAST_HOLD_V else a_coast
    a = -(v * v) / (2.0 * max(d_rem, self.p.D_REM_FLOOR)) - coast_ff
    return _clip(a, planner_min, self.p.A_PHASE_MAX)

  def _ease_gates_pass(self, v: float, d_rem: float, d_gap: float | None, lead_status: bool, lead_v: float) -> bool:
    # d_rem gate with hysteresis (gating-only): enter EASE at d_rem <= 0.8, exit back to GLIDE only
    # once d_rem > 0.95 -- a d_rem dithering around 0.8 must not flip the phase (and chatter the
    # telemetry) every frame. Every other gate stays a hard per-frame bound.
    d_rem_max = self.p.EASE_D_REM_EXIT if self.phase == Phase.PRE_STOP_EASE else self.p.EASE_D_REM_MAX
    if v > self.p.V_EASE or d_rem > d_rem_max:
      return False
    if d_gap is not None and d_gap <= self.p.EASE_GAP_MIN:
      return False
    if lead_status and lead_v < self.p.EASE_LEAD_V_MIN:
      return False
    return True

  def _ease_demand(self, v: float, d_rem: float, a_coast: float, planner_min: float) -> float:
    # a_stop is the PURE kinematic stop demand -- no a_coast term here. Grade/creep enters EASE only
    # through the single deepen-only feedforward below (plan §3 / D1's A_PRESTOP construction):
    # a positive residual must not be counted twice (over-deepens back into the grab), and a negative
    # residual (uphill drag) must never shallow the demand (plan §1: "EASE stays deepen-only").
    a_stop = _clip(-(v * v) / (2.0 * max(d_rem, self.p.D_REM_FLOOR)), planner_min, self.p.A_PHASE_MAX)
    creep_ff = _clip(a_coast, 0.0, 0.4)                          # deepen-only creep feedforward (D1 graft)
    return _clip(_clip(a_stop, self.p.A_EASE_DEEP, self.p.A_EASE_CAP) - creep_ff, self.p.A_EASE_DEEP, self.p.A_PHASE_MAX)

  def _planner_safety_demand(self, a_target: float | None, a_target_trajectory: float | None,
                             v: float, a_coast: float,
                             signals: StopSignals, lead: bool, lead_v: float, wheel_stop: bool) -> tuple[float, bool]:
    """Bound only direct/composite excess depth; preserve the trajectory demand unchanged.

    The phase law targets the nominal rest and ``a_kin`` protects ``D_HARD``. With a trustworthy
    conditioned lead, direct-action depth beyond the constraint-resolved trajectory demand only
    needs enough authority to stop by the existing minimum rest distance. Relative speed handles
    moving and reversing leads uniformly; positive coast/grade residual deepens the bound. The
    trajectory demand is never shallowed. With no trustworthy gap (or no separately published
    trajectory demand), raw planner authority remains unchanged. At wheel stop the lane is disabled.
    """
    if wheel_stop:
      return _INF, False
    direct_demand = float(a_target) if a_target is not None and a_target <= -0.10 else _INF

    gap_trusted = signals.gap_source in ("measured", "held") and not signals.dropout_active
    if not lead or not gap_trusted or signals.d_gap is None:
      return direct_demand, False
    if a_target_trajectory is None or not _finite(a_target_trajectory):
      return direct_demand, False

    v_close = max(v - lead_v, 0.0)
    remaining = max(signals.d_gap - self.p.D_REST_CLIP_MIN, self.p.A_KIN_DEN_FLOOR_M)
    position_floor = -(v_close * v_close) / (2.0 * remaining) - max(a_coast, 0.0)
    bounded_direct = max(direct_demand, position_floor)
    trajectory_demand = float(a_target_trajectory) if a_target_trajectory <= -0.10 else _INF
    demand = min(bounded_direct, trajectory_demand)
    return demand, direct_demand != _INF and demand > direct_demand + 1e-9

  # -- anti-hover / anti-roll monitor (EASE/RAMP/HOLD + terminal GLIDE; plan §3) --------------------
  def _update_monitor(self, v: float, wheel_stop: bool, dt: float, lead: bool, d_gap: float | None, lead_v: float = 0.0,
                      gap_trusted: bool = True) -> float:
    """Once triggered, the floor is a RATCHET: "decreasing again / wheel-stop" pauses the ESCALATION
    only -- the floor lane itself never disarms (plan §2 P1 rule: the deepen lanes never disarm; and
    releasing the anti-roll floor at wheel-stop would perpetually re-roll the car on a 5 percent
    grade where a nominal hold is shallower than the push -- the deeper-held hold is plan §8-2's
    accepted "faintly perceptible" grade behavior). The ratchet clears ONLY on RELEASE (a genuine
    go) and on INACTIVE (reset()) -- never on EASE<->GLIDE phase flips, which is exactly when a
    closing-gap hover needs the armed floor most (R1 kill-shot: a floor dropped at the 2.6 m gap
    boundary released an anti-creep hold while the gap was still closing).
    Detection also runs in APPROACH_GLIDE once v <= V_EASE: at walking pace with stop intent,
    "v not decreasing" is a fault -- the close-gap corridor (d_gap <= 2.6) routes around the EASE
    gates into GLIDE, where a Stribeck-pushed crawl would otherwise never meet any monitor. Above
    V_EASE normal in-band following hovers legitimately, so detection stays off (an armed floor
    still binds). While wheel-stop is LATCHED but v samples sit at/above MON_V_MIN (0.03 == the
    vEgo quantization step, plan §8-4: the smallest nonzero reading), detection keeps running --
    a drivetrain push peaking below the 0.09 latch-reset speed can hold a perpetual crawl inside
    the latch band, which is the same creep-crawl fault, not a stop.
    QUEUE-CREEP GATE (route 00001b72 first stage-2 live drive): in GLIDE/EASE only, the hover/roll
    TRIGGER (arming + escalation) is suppressed while a lead is present and the CONDITIONED gap is
    GROWING (> MON_GAP_GROW_M over the same MON_WINDOW_S the hover detector uses): creeping behind a
    lead that is pulling away is legitimate queue-following, not a fault (the drive armed the
    monitor to -0.65 behind a creeping queue). Strictly bounded: RAMP_TO_HOLD/HOLD stay UNGATED
    (rollaway/creep-through at a genuine standstill must always be caught), and an ALREADY-ARMED
    floor still binds -- gap growth pauses the escalation exactly like 'decreasing again' does but
    NEVER disarms the ratchet (the floor releases only via RELEASE/INACTIVE, as everywhere else)."""
    if self.phase == Phase.RELEASE:
      self._mon_active = False
      self._mon_triggered = False
      self._mon_floor = 0.0
      self.ev.on_release(v)
      return _INF
    monitored = (self.phase in (Phase.PRE_STOP_EASE, Phase.RAMP_TO_HOLD, Phase.HOLD)
                 or (self.phase == Phase.APPROACH_GLIDE and v <= self.p.V_EASE))
    e = self.ev.advance(t=self._t, v=v, phase=self.phase, wheel_stop=wheel_stop, lead=lead,
                        d_gap=d_gap, lead_v=lead_v, gap_trusted=gap_trusted, monitored=monitored)
    if e.genuinely_stopped:
      self._mon_active = False  # genuinely stopped: escalation pauses; the achieved floor holds
      return self._mon_floor if self._mon_triggered else _INF
    if not monitored:
      self._mon_active = False  # detection off above V_EASE in GLIDE; an armed floor still binds
      return self._mon_floor if self._mon_triggered else _INF
    hover, rolling, decreasing, gap_growing = e.hover, e.rolling, e.decreasing, e.gap_growing
    if (hover or rolling) and not gap_growing and not e.suppressed:
      if not self._mon_active:
        self._mon_active = True
        self._mon_escalate_t = 0.0
        if not self._mon_triggered:
          # POST-STOP FAST ARREST (route 00001b87 cycle-4): motion after the wheel-stop latch is a
          # hold ESCAPE, not a hover -- climbing the 0.5 s escalation ladder from -0.35 costs 6-16 cm
          # of felt nudge. Arm the floor DEEP immediately (J_SAFE applies it); rolling approaches
          # keep the gentler A_EASE_DEEP first step.
          if self.phase in (Phase.RAMP_TO_HOLD, Phase.HOLD):
            self._mon_floor = min(self._last_cmd, self.p.A_HOLD - self.p.MON_POSTSTOP_ARREST_EXTRA)
          else:
            self._mon_floor = min(self._last_cmd, self.p.A_EASE_DEEP)
        self._mon_triggered = True
      else:
        self._mon_escalate_t += dt
        if self._mon_escalate_t >= self.p.MON_ESCALATE_PERIOD_S:
          self._mon_floor -= self.p.MON_ESCALATE_STEP  # unbounded escalation (plan §1 J3)
          self._mon_escalate_t = 0.0
    elif self._mon_active and (gap_growing or (decreasing and not rolling)):
      self._mon_active = False  # decreasing again / lead departing: escalation pauses; the floor lane persists
    return self._mon_floor if self._mon_triggered else _INF

  # -- final jerk limiter: THE only writer of the returned command (plan §3 / ledger D2-H1) ---------
  def _jerk_limit(self, target: float, safety_binding: bool, dt: float) -> float:
    if target < self._last_cmd:
      rate = self.p.J_SAFE if (safety_binding or self._fast_deepen) else (
        self.p.J_HOLD if self.phase in (Phase.RAMP_TO_HOLD, Phase.HOLD) else self.p.J_DOWN)
      cmd = max(target, self._last_cmd - rate * dt)
    else:
      rate = self.p.J_GO if self.phase == Phase.RELEASE else self.p.J_UP
      cmd = min(target, self._last_cmd + rate * dt)
    if self._fast_deepen and cmd <= target + 1e-9:
      self._fast_deepen = False  # caught up with the glide law: back to comfort rates
    return cmd

  def reseed_takeover(self, wire_accel: float | None, planner_min_limit: float) -> None:
    """Stage-2 LIVE takeover re-anchor (plan §6 stage 2: 'jerk-consistent takeover from live wire').
    While the service runs in OBSERVATION over the full band its jerk limiter tracks its OWN law,
    not the actuated wire; on the first frame it actually owns the wire, re-anchor _last_cmd on the
    live chain value so the first owned command moves from the ACTUATED trajectory by no more than
    the service's own jerk limits -- exactly the cold-entry seed semantics (update()'s entry path),
    applied warm. INACTIVE is a no-op (entry seeds itself from wire_accel). Deliberately touches
    ONLY the limiter anchor: the phase, the monitor ratchet/floor and the D_REST_eff entry anchor
    all keep their warm observation state (that warmth is the point of full-band observation)."""
    if self.phase == Phase.INACTIVE:
      return
    planner_min = float(planner_min_limit) if _finite(planner_min_limit) else self.p.PLANNER_MIN_FALLBACK
    seed = float(wire_accel) if _finite(wire_accel) else self.p.ENTRY_SEED_ACCEL
    self._last_cmd = _clip(seed, planner_min, self.p.A_PHASE_MAX)

  def _inactive(self) -> ServiceResult:
    return ServiceResult(accel=0.0, phase=Phase.INACTIVE, active=False, debug={"phase": "INACTIVE"})

  def _fallback(self, wheel_stop: bool, dt: float) -> ServiceResult:
    """Non-finite anything => hold last command, or A_HOLD at standstill (never emit non-finite)."""
    if not _finite(self._last_cmd):
      self._last_cmd = self.p.A_HOLD if wheel_stop else self.p.ENTRY_SEED_ACCEL
    if wheel_stop:
      self._last_cmd = min(self._last_cmd, max(self._last_cmd - self.p.J_HOLD * dt, self.p.A_HOLD))
    return ServiceResult(accel=self._last_cmd, phase=self.phase, active=True,
                         debug={"phase": self.phase.name, "nonfinite_fallback": True})

  # -- the service seam -----------------------------------------------------------------------------
  def update(self, *, engaged: bool, v_ego: float, a_ego: float, a_target: float | None,
             should_stop: bool, dts_planner: float | None, planner_min_limit: float,
             signals: StopSignals, lead_status: bool, lead_v: float,
             increased_stopped_distance: float = 0.0, dt: float = 0.01,
             wire_accel: float | None = None, scope_allowed: bool = True,
             a_target_trajectory: float | None = None) -> ServiceResult:
    if not engaged or not scope_allowed:
      self.reset()
      return self._inactive()
    if not (_finite(dt) and dt > 0.0):
      dt = 0.01
    self._t += dt
    wheel_stop = bool(signals.wheel_stop_latched)
    if not _finite(v_ego):
      return self._fallback(wheel_stop, dt) if self.phase != Phase.INACTIVE else self._inactive()
    v = float(v_ego)
    self._v = v
    self._isd = float(increased_stopped_distance) if _finite(increased_stopped_distance) else 0.0
    self._should_stop = bool(should_stop)
    a_tgt = float(a_target) if _finite(a_target) else None
    dts = float(dts_planner) if (_finite(dts_planner) and dts_planner >= 0.0) else None
    planner_min = float(planner_min_limit) if _finite(planner_min_limit) else self.p.PLANNER_MIN_FALLBACK
    d_gap = signals.d_gap if _finite(signals.d_gap) else None
    a_coast = signals.a_coast if _finite(signals.a_coast) else 0.0
    lv = float(lead_v) if _finite(lead_v) else 0.0
    lead = bool(lead_status)
    gap_trusted = signals.gap_source == "measured" and not signals.dropout_active

    self._update_d_rest_eff(d_gap, v, self._isd, lv if lead else 0.0)
    d_rem = self._d_rem(d_gap, dts, v)
    entry_ok = (v < self.p.V_ENTER
                and (self._should_stop
                     or (signals.lead_confirmed_stopped and d_rem is not None and d_rem < self.p.ENTRY_LEAD_D_REM_MAX)))

    # -- phase transitions ------------------------------------------------------------------------
    if self.phase == Phase.INACTIVE:
      if not entry_ok:
        return self._inactive()
      self.phase = Phase.APPROACH_GLIDE
      self.ev.on_entry(self._t, v)  # entry-graced epoch anchors (aborted-go re-entry, route 00001ba2)
      # D_REST_eff is defined AT ENTRY (plan §3 / ledger D1-H2): re-anchor it with the entry-frame
      # v and gap. The value computed on INACTIVE frames (needed for the entry d_rem check) may have
      # been anchored while the lead was first sighted at a different speed -- entry-inconsistent.
      self._d_rest_eff = None
      self._d_rest_calc_gap = None
      self._update_d_rest_eff(d_gap, v, self._isd, lv if lead else 0.0)
      d_rem = self._d_rem(d_gap, dts, v)
      seed = wire_accel if _finite(wire_accel) else self.p.ENTRY_SEED_ACCEL
      self._last_cmd = _clip(float(seed), planner_min, self.p.A_PHASE_MAX)  # jerk-consistent takeover
    if wheel_stop and self.phase in (Phase.APPROACH_GLIDE, Phase.PRE_STOP_EASE):
      self.phase = Phase.RAMP_TO_HOLD  # ramp starts at the FIRST qualifying wheel-stop frame (plan §3)
      self._ramp_t = 0.0
      self.ev.on_wheel_latch(v, d_gap)  # latch-immediate epoch anchors (finish roll + crawl reference)
    if self.phase in (Phase.APPROACH_GLIDE, Phase.PRE_STOP_EASE) and not entry_ok and not signals.dropout_active:
      self.phase = Phase.RELEASE  # state exit; NEVER while decay-holding (the glide keeps braking, D2-H3)
    if self.phase in (Phase.RAMP_TO_HOLD, Phase.HOLD):
      gap_grew = (self.ev.hold_entry_gap is not None and d_gap is not None
                  and d_gap > self.ev.hold_entry_gap + self.p.RELEASE_GAP_GROW_M)
      lead_receding = lead and lv - v > self.p.RELEASE_LEAD_PULL_MPS
      observed_departure = gap_grew and lead_receding and gap_trusted
      departure_s = self.ev.track_departure(observed_departure, dt)
      # 'not lead' extends the plan-§3 trigger to no-lead (stop-line) rests, where no gap/lead-pull
      # evidence can ever exist -- planner go (a_target > 0.2) is the only genuine signal there.
      # While decay-holding a dropped lead the A_DROPOUT_MIN floor below still bounds any release.
      planner_go = (a_tgt is not None and a_tgt > self.p.RELEASE_A_TARGET_MIN
                    and (lead_receding or gap_grew or not lead))
      # A stale negative planner stop cannot pin the car while a measured lead steadily drives
      # away (00001e7b/82, 00001efe/70). Confirmation rejects the brief Doppler/gap excursion that
      # caused 00001c90/142's false launch; no model class or route-specific threshold is involved.
      physical_go = departure_s >= self.p.RELEASE_LEAD_CONFIRM_S
      go = planner_go or physical_go
      if go:
        self.phase = Phase.RELEASE
    if self.phase == Phase.RELEASE and entry_ok and not wheel_stop:
      self.phase = Phase.APPROACH_GLIDE  # the stop re-asserted itself mid-release

    # -- phase command ------------------------------------------------------------------------------
    if self.phase == Phase.RELEASE:
      a_phase = 0.0
    elif self.phase in (Phase.RAMP_TO_HOLD, Phase.HOLD):
      # CYCLE-5 REGRESSION FIX (routes 00001b8f..00001ba3): the wheel-stop latch can fire while the
      # car is still finishing (CS.standstill asserts early / v<=0.06 window) -- building the full
      # A_HOLD (-0.45) there put -0.36..-0.45 on the wire at the true stop instant on MOST stops
      # (design band -0.35..-0.05) = the felt grab. Pressure builds ONLY once genuinely stopped.
      self._ramp_t += dt
      if self.ev.finish_roll(v):
        # GRACE YIELDS TO EVIDENCE (cycle-8 e65 class: a -0.31 natural arrival was held for the
        # whole grace while the car visibly rolled 0.14 m, forcing a harsh -1.0 monitor arrest).
        # The grace exists to preserve arrival FEEL, never to hold insufficient pressure against
        # observed motion: any v rise above the post-latch minimum ends it and the hold builds now.
        self._ramp_t = self.p.NATURAL_ARRIVAL_GRACE_S
        self._fast_deepen = True  # the existing safety-rate path arrests the evidenced roll now
      if (self.phase == Phase.RAMP_TO_HOLD and v >= self.p.MON_V_MIN
          and self._ramp_t < self.p.NATURAL_ARRIVAL_GRACE_S):
        # finish gently -- CRANK #1 (cycle-7, user: 'crank the smoothness requirement up slowly'):
        # HOLD the natural arrival command through the final rolling centimeters instead of building
        # to A_EASE_DEEP (which pinned wire@stop at exactly -0.35 on every stop). The stop instant
        # now carries the EASE arrival (-0.10..-0.25); pressure builds only once genuinely stopped.
        # Never releases an inherited deeper wire; worst case on a slight grade is <=0.5 s of cm-level
        # creep before the ramp window expires and the hold builds (monitor lanes stay live).
        a_phase = min(self._last_cmd, -0.05)
      else:
        # build to A_HOLD, then keep silently deepening to the SECURE hold (parked, felt-free):
        # closes the residual 5-7 cm micro-escape window without any moving-phase depth change
        a_phase = self.p.A_HOLD_SECURE
      if self.phase == Phase.RAMP_TO_HOLD and self._last_cmd <= self.p.A_HOLD + 1e-3:
        self.phase = Phase.HOLD
    else:
      d_rem_eff = d_rem if d_rem is not None else (v * v) / (2.0 * self.p.A_SETTLE_REF)
      in_ease = self._ease_gates_pass(v, d_rem_eff, d_gap, lead, lv)
      if self.phase == Phase.PRE_STOP_EASE and not in_ease:
        self.phase = Phase.APPROACH_GLIDE
        self._fast_deepen = True  # any gate fails => GLIDE law reached at J_SAFE (plan §3)
      elif self.phase == Phase.APPROACH_GLIDE and in_ease:
        self.phase = Phase.PRE_STOP_EASE  # monitor state is continuous across EASE<->GLIDE flips
      if self.phase == Phase.PRE_STOP_EASE:
        a_phase = self._ease_demand(v, d_rem_eff, a_coast, planner_min)
      else:
        a_phase = self._glide_demand(v, d_rem_eff, a_coast, planner_min)

    # -- safety lane: live in EVERY phase; all lanes are min() / deepen-only (plan §3, P1 rule) -----
    if d_gap is not None:
      v_close = max(v - (lv if lead else 0.0), 0.0)
      a_kin = -(v_close * v_close) / (2.0 * max(d_gap - self.p.D_HARD, self.p.A_KIN_DEN_FLOOR_M))
    else:
      a_kin = _INF
    a_plan, plan_position_bounded = self._planner_safety_demand(
      a_tgt, a_target_trajectory, v, a_coast, signals, lead, lv if lead else 0.0, wheel_stop)
    a_mon = self._update_monitor(v, wheel_stop, dt, lead, d_gap, lv if lead else 0.0, gap_trusted)
    target = min(a_phase, a_kin, a_plan, a_mon)
    if signals.dropout_active:
      target = min(target, self.p.A_DROPOUT_MIN)  # decay-hold: may deepen or hold, never release above -0.25
    safety_binding = target < a_phase - 1e-9
    if not _finite(target):
      return self._fallback(wheel_stop, dt)

    self._last_cmd = self._jerk_limit(target, safety_binding, dt)
    if self.phase == Phase.RELEASE and self._last_cmd >= -0.005:
      self.reset()
      return self._inactive()

    debug = {"phase": self.phase.name, "a_phase": a_phase, "a_kin": a_kin, "a_plan": a_plan,
             "a_plan_raw": a_tgt, "a_plan_trajectory": a_target_trajectory,
             "plan_position_bounded": plan_position_bounded,
             "a_monitor": a_mon, "d_rem": d_rem, "d_rest_eff": self._d_rest_eff, "d_gap": d_gap,
             "a_coast": a_coast, "safety_binding": safety_binding, "monitor_active": self._mon_triggered,
             "lead_departure_confirm_s": self.ev.lead_departure_s,
             "dropout_active": signals.dropout_active, "wheel_stop": wheel_stop}
    return ServiceResult(accel=self._last_cmd, phase=self.phase, active=True, debug=debug)
