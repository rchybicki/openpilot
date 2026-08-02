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

  def update(self, lead_status: bool, lead_v: float, dt: float) -> None:
    if lead_status and self._v_min <= lead_v <= LEAD_STOPPED_V_MAX:
      self._stop_t += dt
      self.stopped = self._stop_t >= T_LEAD_STOPPED_S
      self._neg_t = 0.0
    elif self.stopped and lead_status and lead_v < self._v_min:
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
        self._accept(float(raw), track_id)  # cut-in: a new track is real, no persistence
        return
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
  def _update_latches(self, v: float, standstill: bool, lead_status: bool, lead_v: float, dt: float) -> None:
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
    self._latch_strict.update(lead_status, lead_v, dt)
    self._latch_entry.update(lead_status, lead_v, dt)

  def update(self, *, v_ego: float, a_ego: float, a_cmd: float, lead_status: bool, lead_v: float,
             lead_d_rel: float | None, lead_track_id=None, standstill: bool = False, dt: float = 0.01) -> StopSignals:
    if not (_finite(dt) and dt > 0.0):
      dt = 0.01
    v = float(v_ego) if _finite(v_ego) else self._v         # non-finite: hold last good, never propagate
    self._v = v
    lv = float(lead_v) if _finite(lead_v) else self._lead_v
    lead_ok = bool(lead_status)
    self._update_gap(v, lead_ok, lv, lead_d_rel, lead_track_id, dt)
    if lead_ok:
      self._lead_v = lv
    self._update_a_coast(v, a_ego, a_cmd, dt)
    self._update_latches(v, bool(standstill), lead_ok, lv, dt)
    return StopSignals(d_gap=self._d_gap, gap_source=self._gap_source,
                       gap_hold_outward=self._gap_hold_outward, dropout_active=self._dropout_active,
                       a_coast=self._a_coast, wheel_stop_latched=self._wstop_latched,
                       lead_confirmed_stopped=self._latch_strict.stopped,
                       lead_stopped_for_entry=self._latch_entry.stopped)
