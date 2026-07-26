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
                                       once d_rem > 0.95) AND d_gap > 3.1 AND v_lead >= -0.1;
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
                                       direct/composite depth is bounded to stop by 3.0 m (phase still targets 4 m;
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
  D_REST_CLIP_MIN: float = 3.0     # user directive 2026-07-20 (cycle-12): rests in the 3.5 m tail felt
                                   # too close -- the defended band minimum is 3.0 TRUE meters
  D_REST_CLIP_MAX: float = 5.0
  D_HARD: float = 2.0
  D_REST_MIN: float = 3.0          # hard rest floor == the band minimum (was 2.4/2.5 split); close
                                   # entries re-zero to exactly 3.0 instead of 2.4..2.85
  A_GLIDE_NOM: float = 0.5
  A_REST_FEAS: float = 1.2         # rest-anchor FEASIBILITY decel (route 00001b76 seg4/5: anchoring with the
                                   # 0.5 comfort glide re-zeroed a NORMAL stop-and-go entry (gap 5.4 @ 1.65 m/s)
                                   # to a 2.7 m rest -> car stopped at 2.1 m. The anchor must ask "can the car
                                   # firmly land at D_REST_NOM" (planner was already demanding 0.8-1.2 there),
                                   # not "can the gentlest glide reach it"; genuine close entries (gap ~3.0)
                                   # re-zero to the D_REST_MIN floor)
  A_REANCHOR_HYST: float = 0.15    # do not move the rest target for numerical/plant excursions around the comfort law
  REANCHOR_REMAINING_MAX_M: float = 0.6  # re-anchor only in the BLOW-UP region (remaining collapsing toward the
                                         # 0.15 floor): a transient 0.7-1.0 demand MID-glide (remaining >0.6) is a
                                         # normal firm stop-and-go arrival and must not erode the nominal rest
                                         # (comfort-trigger alone dropped a normal rest 4.0 -> 3.49 in fixtures)
  REANCHOR_GAP_MARGIN_M: float = 0.5  # never relax for comfort this close to the band minimum. 0.5 keeps
                                      # the ABSOLUTE admission edge at gap >= 3.5 (its pre-band-retune
                                      # value, when it was 2.5 + 1.0) so the cycle-10 anti-blowup keeps
                                      # covering hot arrivals in the 3.5-4.0 m window; only the landing
                                      # floor rose with the band (comfort_landing >= 3.0)
  REANCHOR_TOTAL_MAX_M: float = 0.4   # total comfort relief budget per stop: the blow-up fix needs ~0.2 m
                                      # (route 00001f0c); unbounded repeated re-anchoring on sustained-push
                                      # grades surrendered >1 m of position (crawl fixtures eroded to 2.5)
  REANCHOR_LANDING_MARGIN_M: float = 0.25  # relief admission needs comfort_landing >= CLIP_MIN + this:
                                           # a landing at exactly the floor leaves zero overshoot budget
                                           # (sol review: gap 3.5 @ v 0.7071 re-anchored to 3.000 and the
                                           # plant rested 2.84 -- through the floor). With the margin the
                                           # admitted branch targets >= 3.25 and rests >= 3.0; the
                                           # rejected near-boundary branch stays firm and also rests
                                           # >= 3.0. The remaining binary firm/gentle boundary is
                                           # structural (predates the band retune); continuous relief is
                                           # a ledgered candidate, not a hot-path addition
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
  J_PIN: float = 2.5               # stationary pressure build to the plant pin level (cycle-11,
                                   # route 00001f10 seg10: J_HOLD 0.6 lost the race against the
                                   # +0.43 Stribeck creep peak through ~0.2 s actuator lag -- 0.2 m
                                   # nudge + reactive -1.0 arrest. Built only once SECURELY stopped,
                                   # so it is felt-free; bounded by J_SAFE in the limiter.
  PIN_MARGIN: float = 0.25         # plant pin level = -(a_coast + this): covers the Stribeck peak
                                   # (+0.43 at v~0.066) above the coasting-EMA estimate (~0.2-0.3
                                   # at stop) -- friction-residual session, plan §8-1
  STOPPED_SECURE_V: float = 0.05   # == the post-latch roll escape bar: readings below it with a
                                   # flat trusted gap are the dither band, i.e. genuinely stopped
  STOPPED_SECURE_DWELL_S: float = 0.25  # == the wheel-latch dwell: sustained evidence, not one frame
  J_GO: float = 1.2
  # law-internal bounds from the §3 equations
  D_REM_FLOOR: float = 0.15        # glide floored denominator = the d_settle envelope bound (D2-H2)
  A_PHASE_MAX: float = -0.03       # phase command never shallower than -0.03 while active
  A_KIN_DEN_FLOOR_M: float = 0.30
  EASE_D_REM_MAX: float = 0.8
  EASE_D_REM_EXIT: float = 0.95    # d_rem gate exit hysteresis: EASE -> GLIDE only once d_rem > 0.95
  EASE_GAP_MIN: float = 3.1        # D_REST_MIN + 0.1: EASE (the only shallowing phase) never runs
                                   # inside the 3.0 m band minimum
  EASE_LEAD_V_MIN: float = -0.1
  ENTRY_LEAD_D_REM_MAX: float = 15.0
  REST_RECALC_GROW_M: float = 1.0
  MON_V_MIN: float = 0.03
  MON_DECREASE_MPS: float = 0.02   # required decrease per MON_WINDOW_S
  MON_WINDOW_S: float = 0.4
  MON_RISE_MPS: float = 0.06
  MON_ESCALATE_STEP: float = 0.15
  MON_ESCALATE_PERIOD_S: float = 0.5
  MON_UNWIND_DWELL_S: float = 2.0  # genuinely-stopped dwell before an over-escalated arrest floor is
                                   # given back toward A_HOLD_SECURE (cycle-13). Long on purpose: the
                                   # ladder only overshoots after a real escape, so there is no hurry,
                                   # and a slow unwind keeps the re-arm path (roll/displacement lanes,
                                   # still live) as the fast one.
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
    self.stopped_dwell_s = 0.0          # sustained sub-escape-bar, crawl-free readings (pin trigger)
    self.stopped_dwell_v: float | None = None  # dwell anchor reading: falling below it restarts the dwell
    self.v_hist: list[tuple[float, float]] = []
    self.gap_hist: list[tuple[float, float]] = []

  @property
  def stopped_secure(self) -> bool:
    """Dither-immune genuinely-stopped evidence (cycle-11 pin trigger): readings held below the
    post-latch escape bar with zero crawl deficit for the full dwell. Read by the RAMP law one
    frame behind advance() -- harmless lag on a 0.25 s dwell."""
    return self.stopped_dwell_s >= self.p.STOPPED_SECURE_DWELL_S

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
    self.stopped_dwell_s = 0.0    # secure-stop evidence starts fresh at each latch
    self.stopped_dwell_v = None

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
              d_gap: float | None, lead_v: float, gap_trusted: bool, monitored: bool,
              dt: float = 0.01) -> Evidence:
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
    # secure-stop dwell (cycle-11 pin trigger, route 00001f10): the dither band (readings 0.03-0.05
    # while physically stopped) IS a genuine stop once it sustains crawl-free for the full dwell --
    # the MON_V_MIN short-circuit below cannot see it (readings sit AT the quantization step).
    # The dwell is anchored to a reference reading: a SLOW DECAYING FINISH (v drifting 0.05 -> 0
    # under a light natural arrival) keeps falling below the anchor and restarts the dwell, so the
    # pin can never fire while the car is still genuinely finishing (the cycle-5 grab class);
    # dither bounces around a level and completes, a clean flat zero completes ~one dwell later.
    if wheel_stop and v < self.p.STOPPED_SECURE_V and not crawl:
      if self.stopped_dwell_v is None or v < self.stopped_dwell_v - 0.02:
        self.stopped_dwell_v = v       # still finishing: re-anchor, secure evidence restarts
        self.stopped_dwell_s = 0.0
      else:
        self.stopped_dwell_s += dt
    else:
      self.stopped_dwell_v = None
      self.stopped_dwell_s = 0.0
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
    self._mon_stopped_s = 0.0           # sustained genuinely-stopped dwell (arrest-floor unwind)
    self._ramp_t = 0.0                  # time in RAMP_TO_HOLD (gentle-finish window)
    self._pin_level: float | None = None  # secure-stop plant pin depth (J_PIN build target bound)
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
          and comfort_landing >= self.p.D_REST_CLIP_MIN + self.p.REANCHOR_LANDING_MARGIN_M):
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
                        d_gap=d_gap, lead_v=lead_v, gap_trusted=gap_trusted, monitored=monitored, dt=dt)
    if e.genuinely_stopped:
      self._mon_active = False  # genuinely stopped: escalation pauses; the achieved floor holds
      # CYCLE-13 UNWIND (ledger item a, routes 00001f44 seg3 / 00001f47 seg2): the arrest ladder
      # overshoots (it is time-stepped, not force-measured) and the floor is a RATCHET cleared only
      # on RELEASE -- so a transient breakaway left -1.00 on the wire for the WHOLE 27.4 s standstill
      # (ended only by driver gas) and -0.85 for 7.2 s. 8 of 24 corpus holds sat deeper than the
      # empirically-always-holds A_HOLD_SECURE. Once the car is GENUINELY stopped (sub-MON_V_MIN
      # readings, wheel-stop latched, zero crawl deficit) for a sustained dwell, the depth beyond
      # A_HOLD_SECURE is doing no work: give it back, slowly, and never past that level.
      # Safety: this can only ever relax TOWARD -0.70, which ~20+ recorded arrests across cycles
      # 6/11/12 (and seg3 again) show always holds this car; the roll and displacement lanes stay
      # live throughout, so if the relaxed floor were ever insufficient the ladder re-arms at J_SAFE
      # from wherever it has reached. A crawling car never qualifies (genuinely_stopped requires
      # not crawl), and the dwell restarts on any motion.
      if self._mon_triggered and self._mon_floor < self.p.A_HOLD_SECURE:
        self._mon_stopped_s += dt
        if self._mon_stopped_s >= self.p.MON_UNWIND_DWELL_S:
          self._mon_floor = min(self._mon_floor + self.p.J_HOLD * dt, self.p.A_HOLD_SECURE)
      return self._mon_floor if self._mon_triggered else _INF
    self._mon_stopped_s = 0.0  # any non-stopped frame restarts the unwind dwell
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
      elif decreasing and not e.crawl:
        # CYCLE-13 (routes 00001f44 seg3, 00001f47 seg2): PAUSE the ladder while the arrest is
        # demonstrably working. The pre-existing pause below is unreachable here -- it sits in the
        # elif, which requires (hover or rolling) to be FALSE, but ``rolling`` is measured against
        # the EPOCH MINIMUM, so during an arrest v stays far above it and rolling remains true all
        # the way down to standstill. The ladder therefore kept stepping every
        # MON_ESCALATE_PERIOD_S while the car was already decelerating under the floor it had just
        # armed: seg3 armed -0.70 at t=230.47, v peaked 0.25 and fell, and -0.85 / -1.00 were added
        # at 231.07 / 231.47 while v dropped 0.20 -> 0.08. Pure control overshoot -- and since the
        # floor is a ratchet cleared only on RELEASE, it became the hold for the ENTIRE standstill
        # (-1.00 for 27.4 s on seg3, -0.85 for 7.2 s on f47 seg2; 8 of 24 corpus holds sat deeper
        # than the empirically-always-holds -0.70).
        # Measured deceleration IS the evidence that the armed floor suffices, so it alone pauses
        # the ladder. Deliberately preserved: the achieved floor is NEVER released (deepen-only,
        # ratchet intact), and ``crawl`` still forces escalation -- a sub-quantization crawl reads
        # as flat/decreasing in velocity while genuinely consuming gap (R1 finding), and that case
        # must keep ratcheting. The timer does not accumulate while paused, so a stalled arrest
        # gets its next step one full period after the deceleration stops, not instantly.
        pass
      else:
        self._mon_escalate_t += dt
        if self._mon_escalate_t >= self.p.MON_ESCALATE_PERIOD_S:
          self._mon_floor -= self.p.MON_ESCALATE_STEP  # unbounded escalation (plan §1 J3)
          self._mon_escalate_t = 0.0
    elif self._mon_active and (gap_growing or (decreasing and not e.crawl)):
      # decreasing again / lead departing: escalation pauses; the floor lane persists.
      #
      # CYCLE-13 (routes 00001f44 seg3, 00001f47 seg2): the pause used to require
      # "decreasing AND NOT rolling", but ``rolling`` is measured against the EPOCH MINIMUM, so
      # during an arrest v stays far above it and rolling remains true all the way down. The ladder
      # therefore kept stepping every MON_ESCALATE_PERIOD_S while the car was ALREADY decelerating
      # under the floor it had just armed: seg3 armed -0.70 at t=230.47, v peaked 0.25 and began
      # falling, and -0.85 / -1.00 were then added at 231.07 / 231.47 while v dropped 0.20 -> 0.08.
      # Pure control overshoot -- and because the floor is a ratchet cleared only on RELEASE, that
      # overshoot became the hold for the ENTIRE standstill (-1.00 for 27.4 s on seg3, -0.85 for
      # 7.2 s on f47 seg2; 8 of 24 corpus holds sat deeper than the always-holds -0.70).
      # Genuine deceleration IS the evidence that the arrest is working, so it alone pauses the
      # ladder now. Two properties are preserved deliberately: the achieved floor is never released
      # (deepen-only), and the escalation TIMER is not reset, so if the deceleration stalls the next
      # step lands promptly. ``crawl`` still forces escalation regardless -- a sub-quantization crawl
      # reads as flat/decreasing in velocity while genuinely consuming gap (R1 finding), and that
      # case must keep ratcheting.
      self._mon_active = False
    return self._mon_floor if self._mon_triggered else _INF

  # -- final jerk limiter: THE only writer of the returned command (plan §3 / ledger D2-H1) ---------
  def _jerk_limit(self, target: float, safety_binding: bool, dt: float) -> float:
    if target < self._last_cmd:
      if safety_binding or self._fast_deepen:
        rate = self.p.J_SAFE
      elif self.phase in (Phase.RAMP_TO_HOLD, Phase.HOLD):
        # secure-stop plant pin: fast stationary build until the wire covers the measured push,
        # then the silent J_HOLD deepening to the secure hold continues as before
        rate = self.p.J_PIN if (self._pin_level is not None and self._last_cmd > self._pin_level) else self.p.J_HOLD
      else:
        rate = self.p.J_DOWN
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
    self._pin_level = None  # set only by the RAMP/HOLD branch below (secure-stop plant pin)
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
          and self._ramp_t < self.p.NATURAL_ARRIVAL_GRACE_S and not self.ev.stopped_secure):
        # finish gently -- CRANK #1 (cycle-7, user: 'crank the smoothness requirement up slowly'):
        # HOLD the natural arrival command through the final rolling centimeters instead of building
        # to A_EASE_DEEP (which pinned wire@stop at exactly -0.35 on every stop). The stop instant
        # now carries the EASE arrival (-0.10..-0.25); pressure builds only once genuinely stopped.
        # Never releases an inherited deeper wire. Once the stop is SECURE (dither-immune dwell) the
        # grace has nothing left to protect -- the pin build below starts even inside the window.
        a_phase = min(self._last_cmd, -0.05)
      else:
        # build to A_HOLD, then keep silently deepening to the SECURE hold (parked, felt-free):
        # closes the residual 5-7 cm micro-escape window without any moving-phase depth change
        a_phase = self.p.A_HOLD_SECURE
      # PLANT-AWARE PIN (cycle-11, route 00001f10 seg10): the measured coasting push tells us what
      # it takes to stay pinned; landing shallow (-0.13) and building at J_HOLD gave the +0.43
      # Stribeck creep peak a ~1.2 s window through the actuator lag -- 0.2 m nudge, reactive -1.0
      # arrest, ledger wound to -3.5. Once the stop is SECURE the wire builds at J_PIN until it
      # reaches -(a_coast + PIN_MARGIN); the reactive lanes remain as the safety net, no longer the
      # common case. A stationary pressure build is felt-free (the grab class was deep wire while
      # STILL FINISHING, which the secure dwell excludes by construction).
      self._pin_level = (_clip(-(max(a_coast, 0.0) + self.p.PIN_MARGIN), self.p.A_HOLD_SECURE, self.p.A_HOLD)
                         if self.ev.stopped_secure else None)
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
