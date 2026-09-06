"""StopContext -- conditioned signals for the Stopping Service V3 (plan: docs/stopping/stopping_service_v3_plan.md §3 "Signals").

STAGE 0/1 module: pure python + math, no cereal/capnp imports. Conditions exactly four signals
and nothing else (plan §2 role sentence); every law in stopping_service.py consumes THESE
outputs, never raw radar/CAN values.

1. d_gap -- asymmetric-persistence lead-gap filter (plan §3, ledger row D3-H1):
   - track-ID change => accept the raw reading immediately (a cut-in is real);
   - same-track INWARD step <= ego-closing-consistent (v_close*dt + 0.3 m) => immediate;
   - larger inward step => T_PERSIST_IN (0.15 s) persistence, meanwhile hold the
     ego-motion-propagated prediction (bad news is delayed only when physically inconsistent);
   - OUTWARD step => T_PERSIST_OUT (0.25 s) persistence, then rate-limited at
     R_OUT = max(v_lead, 0) + 0.5 m/s (good news is always slow);
   - dropout decay-hold bookkeeping (ledger row D2-H3): on lead loss the last-good gap decays
     INWARD at 0.5 m/s for T_DROPOUT (2.0 s), then expires to None.
2. a_coast = EMA_tau=1.0s(aEgo - a_cmd(t - tau_delay)), clipped [-0.5, +0.5]; held (not
   updated) below v = 0.1 where it is consumed deepen-only.
3. wheel_stop_latched: CS.standstill OR v <= 0.06 held 0.25 s; reset on any sample > 0.09
   (the 0.03/0.06 vEgo-quantization alternation latches cleanly, plan §1 J3 sweep).
4. lead_confirmed_stopped: lead_status AND v_lead in [-0.1, +0.3] held 0.3 s of ACCUMULATED
   dwell -- a brief small dip below the window (< 0.25 s aggregate per epoch, within 0.4 m/s
   below) PAUSES the dwell rather than resetting it (cycle-21: dips behind stationary leads are
   frequent but short, p50 0.13 s / p90 0.29 s over 226 runs, and an unbroken-dwell requirement
   made noisy stopped leads confirm late or never). Once confirmed, negative Doppler un-confirms
   only after 0.5 s persistence (cycle-15); a genuinely reversing lead sustains it -- and
   reversing safety lives in the deepen lanes/EASE gate, not this entry latch. Lead loss /
   drive-away un-confirm instantly (a reversing lead is never "stopped").

NaN robustness: any non-finite input holds the signal's last good state -- non-finite values
never propagate into an output.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

# Constants exactly per plan §3 (TRUE meters / m/s / s everywhere).
T_PERSIST_IN_S = 0.15
T_PERSIST_OUT_S = 0.25
T_MOTION_TRUST_S = 0.25       # round-3 review: a REPLACEMENT target's velocity is not departure
                              # evidence until the new identity persists this long (same clock as
                              # outward gap persistence -- motion that relieves must earn like
                              # geometry that relieves)
GAP_INWARD_SLACK_M = 0.3
R_OUT_BASE_MPS = 0.5
T_DROPOUT_S = 2.0
DROPOUT_DECAY_MPS = 0.5
TAU_COAST_S = 1.0
A_COAST_CLIP = 0.5
A_COAST_HOLD_V = 0.1
A_CMD_DELAY_S = 0.3   # tau_delay for a_cmd(t - tau_delay): nominal Santa Fe long actuation delay
V_WSTOP = 0.06
V_WSTOP_RESET = 0.09
T_WSTOP_S = 0.25
LEAD_STOPPED_V_MIN = -0.1
LEAD_ENTRY_V_MIN = -0.5       # cycle-22 (user policy, 2026-08-02, adjudicating the f82 3 m stop:
                              # "that will be super rare, but if we stopped at the desired 4-5 m,
                              # this wouldn't have been a problem"): a lead rolling back SLOWLY is
                              # still a stop the service should ENTER and manage -- the 4-5 m aim
                              # absorbs a ~25 cm rollback, and reversal safety lives in the
                              # deepen-only lanes (a_kin closes via v_close, EASE raw-rejects,
                              # the monitor arrests), never in this entry latch. The STRICT latch
                              # (window floor -0.1) is unchanged and still guards the cycle-17
                              # gentle-rate reversal disqualifier. Faster reversal (< -0.5) is a
                              # hazard approach, not a manageable stop: entry stays refused.
LEAD_STOPPED_V_MAX = 0.3
T_LEAD_STOPPED_S = 0.3
LEAD_STOPPED_DIP_V = 0.4      # how far below the stopped window a NOISE dip may reach (readings
                              # below LEAD_STOPPED_V_MIN - this are treated as real motion)
T_LEAD_STOPPED_DIP_S = 0.25   # cycle-21: a brief excursion out of the stopped window PAUSES the
                              # confirmation dwell instead of resetting it. Measured on 226
                              # negative-Doppler runs behind physically stationary leads (gap
                              # change < 0.3 m): p50 0.13 s, p90 0.29 s, p99 0.69 s, max 1.08 s --
                              # frequent but SHORT, and each one reset the unbroken-dwell
                              # requirement, so noisy stopped leads confirmed late (median
                              # handover 1.12 m/s across 45 corpus stops vs the 2.5 design;
                              # exemplar: route 00001f80 seg99 becomes eligible at 2.49 m/s /
                              # 7.8 m with this fix, was 2.26 / 7.0). PROVENANCE NOTE: route
                              # 00001f82 seg15 (the 3 m-rest bookmark that STARTED this cycle)
                              # turned out NOT to be this defect -- its 1.23 s negative run is
                              # corroborated by the geometry (implied lead movement -0.25 m vs
                              # -0.30 m predicted), i.e. that lead genuinely rolled back and the
                              # latch was RIGHT to withhold. A dip longer/deeper than this
                              # budget is likewise treated as real and resets the dwell.
T_LEAD_NEG_OFF_S = 0.5  # negative-Doppler must persist this long to un-confirm a stopped lead
                        # (cycle-15; recorded noise runs on a physically stopped lead were <= 0.36 s)


def _finite(x) -> bool:
  return x is not None and math.isfinite(x)


@dataclass(frozen=True)
class StopSignals:
  d_gap: float | None          # conditioned TRUE gap (None = no lead and no decay-hold)
  gap_source: str              # 'measured' | 'held' | 'decay' | 'none'
  gap_hold_outward: bool       # a 'held' gap whose provenance is OUTWARD persistence: the emitted
                               # value is min(prediction, raw), i.e. a LOWER BOUND on the true gap
                               # (safe to shallow a demand with). False for the inward-rejection
                               # and invalid-reading holds, where the held value can be LARGER than
                               # reality -- consumers that relieve braking must refuse those.
  dropout_active: bool         # True while the decay-hold window is running
  a_coast: float               # net external push (+) / drag (-) on the ego, m/s^2
  wheel_stop_latched: bool
  lead_confirmed_stopped: bool # STRICT window [-0.1, +0.3]: guards relief-side consumers
  lead_stopped_for_entry: bool # ENTRY window [-0.5, +0.3]: a slowly-rolling-back lead is a stop
                               # to manage (cycle-22); consumed by entry_ok ONLY
  track_age_s: float = 0.0         # seconds the CURRENT REAL lead identity has persisted; 0 while the lead is
                                   # identity-less (vision-only) -- the attributed-safety trust-in gate reads it
  lead_motion_earned: bool = True  # False while a REPLACEMENT identity is younger than
                                   # T_MOTION_TRUST_S: its velocity must not fire lead_receding
                                   # (round-3 review: one flap frame at +1.0 released a HOLD)


class _StoppedLatch:
  """One stopped-lead confirmation latch: dwell + aggregate dip budget + un-confirm rules,
  parameterised by the window floor so the STRICT (-0.1, relief-side) and ENTRY (-0.5,
  cycle-22 policy) latches share one implementation instead of drifting apart."""

  def __init__(self, v_min: float):
    self._v_min = v_min
    self.reset()

  def reset(self) -> None:
    self.stopped = False
    self._stop_t = 0.0
    self._neg_t = 0.0
    self._dip_t = 0.0
    self._earned = False   # THIS target accumulated its own full dwell (off-delay entitlement)

  def on_track_change(self, lead_v: float) -> None:
    """The accepted radar track was REPLACED (cycle-22 span review, HIGH): confirmation is
    evidence about ONE target, and the negative off-delay exists to survive same-target Doppler
    noise, not identity changes. A replacement reading inside this latch's own window keeps the
    state (queue scenes hand over between stopped objects every few hundred ms -- measured 9-11
    id changes per approach on 00001f7f seg129 / 00001f82 seg5, and a plain reset would starve
    confirmation exactly where stops happen); an out-of-window replacement resets NOW, off-delay
    included, so a fast-reversing new target can never ride an old target's confirmation."""
    if not (self._v_min <= lead_v <= LEAD_STOPPED_V_MAX):
      self.reset()
      return
    # In-window handover: confirmation carries over PROVISIONALLY, but the off-delay entitlement
    # and the dip budget do not (round-2 review, HIGH: transferring them let a chain of fresh ids
    # each present one in-window frame and then ride the 0.5 s off-delay -- unbounded laundering,
    # 395 m of reported reversal while entry-eligible). The new target keeps eligibility only
    # while it reads in-window and must re-earn its own dwell; its first out-of-window frame
    # before that resets instantly.
    self._stop_t = 0.0
    self._neg_t = 0.0
    self._dip_t = 0.0
    self._earned = False

  def update(self, lead_status: bool, lead_v: float, dt: float) -> None:
    if lead_status and self._v_min <= lead_v <= LEAD_STOPPED_V_MAX:
      self._stop_t += dt
      if self._stop_t >= T_LEAD_STOPPED_S:
        self.stopped = True   # monotone within the epoch: a provisional handover stays confirmed
        self._earned = True   # ...and re-earning here restores the off-delay entitlement
      self._neg_t = 0.0
    elif self.stopped and lead_status and lead_v < self._v_min:
      if not self._earned:
        # a provisionally-handed-over target gets NO off-delay grace: the off-delay is Doppler
        # tolerance for a target that earned its own dwell, never transferable evidence
        self.reset()
        return
      # negative-Doppler off-delay (cycle-15): un-confirm only after it persists
      self._neg_t += dt
      if self._neg_t >= T_LEAD_NEG_OFF_S:
        self.reset()
    elif (lead_status and not self.stopped and self._stop_t > 0.0
          and self._v_min - LEAD_STOPPED_DIP_V < lead_v < self._v_min
          and self._dip_t + dt < T_LEAD_STOPPED_DIP_S):
      # brief small dip while EARNING: pause the dwell (cycle-21, aggregate budget per epoch)
      self._dip_t += dt
    else:
      self.reset()


class StopContext:
  def __init__(self):
    self.reset()

  def reset(self) -> None:
    self._d_gap: float | None = None
    self._gap_source = "none"
    self._gap_hold_outward = False
    self._track_id = None
    self._in_t = 0.0
    self._out_t = 0.0
    self._dropout_t = 0.0
    self._dropout_active = False
    self._a_coast = 0.0
    self._cmd_buf: list[float] = []
    self._last_cmd = 0.0
    self._v = 0.0
    self._lead_v = 0.0
    self._wstop_t = 0.0
    self._wstop_latched = False
    self._latch_strict = _StoppedLatch(LEAD_STOPPED_V_MIN)  # relief-side consumers
    self._latch_entry = _StoppedLatch(LEAD_ENTRY_V_MIN)     # entry_ok only (cycle-22)
    self._latch_track_id = None                             # last REAL (>=0) id seen by the latches
    self._motion_trust_t = T_MOTION_TRUST_S                 # ids never seen -> legacy full trust
    self._track_age_t = 0.0                                 # grows ONLY while a REAL (>=0) identity persists; an
                                                            # identity-less (vision-only) lead is never mature (R1 HIGH)

  # -- signal 1: asymmetric-persistence gap filter + dropout decay-hold --------------------------
  def _accept(self, raw: float, track_id) -> None:
    self._d_gap = raw
    self._gap_source = "measured"
    self._gap_hold_outward = False
    self._in_t = 0.0
    self._out_t = 0.0
    if track_id is not None:
      self._track_id = track_id

  def _update_gap(self, v: float, lead_status: bool, lead_v: float, raw, track_id, dt: float) -> None:
    self._dropout_active = False
    if lead_status and _finite(raw) and raw > 0.0:
      self._dropout_t = 0.0
      if self._d_gap is None:
        self._accept(float(raw), track_id)
        return
      if track_id is not None and self._track_id is not None and track_id != self._track_id:
        # A new track is real -- but immediate acceptance is DEEPEN-ONLY (round-2 review, HIGH):
        # an OUTWARD replacement reading taken on one frame (real -> -1 -> other-real flap in the
        # recorded queue scenes) would otherwise become trusted "measured" geometry instantly,
        # bypassing outward persistence and releasing a HOLD via gap_grew. Inward (cut-in) still
        # accepts now; outward acknowledges the identity and earns through the outward
        # persistence / rate-limit path like any other relieving evidence.
        if float(raw) <= self._d_gap:
          self._accept(float(raw), track_id)
          return
        self._track_id = track_id
      v_close = max(v - lead_v, 0.0)
      pred = max(self._d_gap + (lead_v - v) * dt, 0.0)  # ego-motion-propagated prediction
      delta = float(raw) - self._d_gap
      if delta <= 0.0:  # inward
        self._out_t = 0.0
        if -delta <= v_close * dt + GAP_INWARD_SLACK_M:
          self._accept(float(raw), track_id)   # ego-closing-consistent: immediate
        else:
          self._in_t += dt
          if self._in_t >= T_PERSIST_IN_S:
            self._accept(float(raw), track_id)  # persisted collapse is real: full authority
          else:
            # INWARD rejection hold: the emitted prediction is LARGER than the raw reading, i.e.
            # possibly optimistic -- never usable to relieve a demand.
            self._d_gap, self._gap_source, self._gap_hold_outward = pred, "held", False
      else:  # outward
        self._in_t = 0.0
        self._out_t += dt
        if self._out_t >= T_PERSIST_OUT_S:
          r_out = max(lead_v, 0.0) + R_OUT_BASE_MPS
          self._d_gap = min(float(raw), self._d_gap + r_out * dt)
          self._gap_source, self._gap_hold_outward = "measured", False  # no stale hold provenance
        else:
          # OUTWARD persistence hold: min(prediction, raw) is a LOWER BOUND on the true gap.
          self._d_gap, self._gap_source, self._gap_hold_outward = min(pred, float(raw)), "held", True
    elif self._d_gap is not None:
      self._in_t = 0.0
      self._out_t = 0.0
      if lead_status:  # lead present but reading non-finite/invalid: hold ego-propagated prediction
        self._d_gap = max(self._d_gap + (self._lead_v - v) * dt, 0.0)
        self._gap_source, self._gap_hold_outward = "held", False  # unverified: never relieve on it
      else:  # dropout decay-hold (ledger D2-H3): decay inward, then expire
        self._gap_hold_outward = False
        self._dropout_t += dt
        if self._dropout_t <= T_DROPOUT_S + 1e-9:
          self._d_gap = max(self._d_gap - DROPOUT_DECAY_MPS * dt, 0.0)
          self._gap_source = "decay"
          self._dropout_active = True
        else:
          self._d_gap, self._gap_source, self._track_id = None, "none", None

  # -- signal 2: coast/creep residual ------------------------------------------------------------
  def _update_a_coast(self, v: float, a_ego: float, a_cmd: float, dt: float) -> None:
    self._last_cmd = float(a_cmd) if _finite(a_cmd) else self._last_cmd
    self._cmd_buf.append(self._last_cmd)
    delay_n = max(int(round(A_CMD_DELAY_S / dt)), 1)
    while len(self._cmd_buf) > delay_n + 1:
      self._cmd_buf.pop(0)
    if v >= A_COAST_HOLD_V and _finite(a_ego):  # held (deepen-only consumption) below 0.1 m/s
      alpha = min(dt / TAU_COAST_S, 1.0)
      self._a_coast += alpha * ((float(a_ego) - self._cmd_buf[0]) - self._a_coast)
      self._a_coast = min(max(self._a_coast, -A_COAST_CLIP), A_COAST_CLIP)

  # -- signals 3 + 4: latches --------------------------------------------------------------------
  def _update_latches(self, v: float, standstill: bool, lead_status: bool, lead_v: float, dt: float,
                      lv_valid: bool = True) -> None:
    if v > V_WSTOP_RESET:
      self._wstop_latched, self._wstop_t = False, 0.0
    else:
      if standstill:
        self._wstop_latched = True
      if v <= V_WSTOP:
        self._wstop_t += dt
        if self._wstop_t >= T_WSTOP_S:
          self._wstop_latched = True
      else:
        self._wstop_t = 0.0
    # Both latches share one implementation (_StoppedLatch): dwell 0.3 s with the cycle-21
    # aggregate dip budget, cycle-15's sustained-negative un-confirm, and instant un-confirm on
    # drive-away/lead-loss. They differ ONLY in the window floor: STRICT -0.1 (guards the
    # cycle-17 gentle-rate reversal disqualifier and any future relief consumer) vs ENTRY -0.5
    # (cycle-22 policy: a slowly-rolling-back lead is a stop to manage; the 4-5 m aim absorbs
    # the rollback and the deepen-only lanes own reversal safety).
    if lv_valid:
      self._latch_strict.update(lead_status, lead_v, dt)
      self._latch_entry.update(lead_status, lead_v, dt)
    elif not lead_status:
      self._latch_strict.reset()
      self._latch_entry.reset()
    # else: lead present, reading non-finite -- FREEZE both latches (round-3 review: the held
    # last-good velocity could otherwise accrue dwell and even the off-delay entitlement for a
    # target that never supplied a valid sample; retention-without-earning matches the gap
    # filter's unverified-hold behaviour on the same frames)

  def update(self, *, v_ego: float, a_ego: float, a_cmd: float, lead_status: bool, lead_v: float,
             lead_d_rel: float | None, lead_track_id=None, standstill: bool = False, dt: float = 0.01) -> StopSignals:
    if not (_finite(dt) and dt > 0.0):
      dt = 0.01
    v = float(v_ego) if _finite(v_ego) else self._v         # non-finite: hold last good, never propagate
    self._v = v
    lv_valid = _finite(lead_v)
    lv = float(lead_v) if lv_valid else self._lead_v
    lead_ok = bool(lead_status)
    if lead_track_id is not None and lead_track_id < 0:
      lead_track_id = None  # radard emits -1 for "no radar identity" (vision-promoted lead)
    self._update_gap(v, lead_ok, lv, lead_d_rel, lead_track_id, dt)
    if lead_ok and lead_track_id is not None:
      if self._latch_track_id != lead_track_id:
        self._track_age_t = 0.0     # first or replacement identity: age restarts (attributed-safety trust-in)
      if self._latch_track_id is not None and lead_track_id != self._latch_track_id:
        self._motion_trust_t = 0.0  # replacement identity: its motion must earn departure trust
        if lv_valid:
          self._latch_strict.on_track_change(lv)
          self._latch_entry.on_track_change(lv)
        else:
          # round-3 review (MEDIUM): a replacement arriving with a non-finite velocity would be
          # judged on the PREVIOUS target's held reading -- it has supplied no evidence at all
          self._latch_strict.reset()
          self._latch_entry.reset()
      self._latch_track_id = lead_track_id
    if lead_ok and lv_valid:
      self._motion_trust_t = min(self._motion_trust_t + dt, T_MOTION_TRUST_S)
    if lead_ok and lead_track_id is not None and lead_track_id == self._latch_track_id:
      self._track_age_t = min(self._track_age_t + dt, 1e9)   # a REAL identity persisting earns age
    else:
      self._track_age_t = 0.0       # no lead, or an identity-less (vision-only) lead: never mature
    if lead_ok:
      self._lead_v = lv
    self._update_a_coast(v, a_ego, a_cmd, dt)
    self._update_latches(v, bool(standstill), lead_ok, lv, dt, lv_valid=lv_valid)
    return StopSignals(d_gap=self._d_gap, gap_source=self._gap_source,
                       gap_hold_outward=self._gap_hold_outward, dropout_active=self._dropout_active,
                       a_coast=self._a_coast, wheel_stop_latched=self._wstop_latched,
                       lead_confirmed_stopped=self._latch_strict.stopped,
                       lead_stopped_for_entry=self._latch_entry.stopped,
                       track_age_s=self._track_age_t,
                       lead_motion_earned=self._motion_trust_t >= T_MOTION_TRUST_S)
