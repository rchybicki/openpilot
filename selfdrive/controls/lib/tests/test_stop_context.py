"""StopContext unit tests (Stopping Service V3 plan §3 "Signals", §6 stage-0 fixtures)."""

import math

import pytest

from openpilot.selfdrive.controls.lib.stop_context import (
  DROPOUT_DECAY_MPS,
  StopContext,
  T_DROPOUT_S,
)

DT = 0.01


def _step(ctx, *, v=1.0, a=0.0, cmd=-0.1, lead=True, lv=0.0, gap=None, tid=None, standstill=False, dt=DT):
  return ctx.update(v_ego=v, a_ego=a, a_cmd=cmd, lead_status=lead, lead_v=lv, lead_d_rel=gap,
                    lead_track_id=tid, standstill=standstill, dt=dt)


# --- d_gap asymmetric persistence filter ---------------------------------------------------------

def test_first_reading_accepted_immediately() -> None:
  ctx = StopContext()
  sig = _step(ctx, gap=5.0)
  assert sig.d_gap == pytest.approx(5.0)
  assert sig.gap_source == "measured"


def test_ego_closing_consistent_inward_is_immediate() -> None:
  ctx = StopContext()
  _step(ctx, v=1.0, gap=5.0)
  sig = _step(ctx, v=1.0, gap=4.8)  # 0.2 m inward <= v_close*dt + 0.3
  assert sig.d_gap == pytest.approx(4.8)


def test_large_inward_step_needs_persistence_then_accepts() -> None:
  ctx = StopContext()
  _step(ctx, v=1.0, gap=5.0, dt=0.05)
  sig1 = _step(ctx, v=1.0, gap=3.0, dt=0.05)   # 2.0 m collapse: held on prediction
  assert sig1.d_gap == pytest.approx(5.0 - 1.0 * 0.05)
  assert sig1.gap_source == "held"
  sig2 = _step(ctx, v=1.0, gap=3.0, dt=0.05)
  assert sig2.gap_source == "held"
  sig3 = _step(ctx, v=1.0, gap=3.0, dt=0.05)   # 0.15 s persisted: real, full authority
  assert sig3.d_gap == pytest.approx(3.0)
  assert sig3.gap_source == "measured"


def test_alternating_bounce_never_survives() -> None:
  # plan §6 stage-0 fixture: alternating 2.0 <-> 3.9 radar bounce must never reach the laws
  ctx = StopContext()
  _step(ctx, v=0.5, gap=3.9, dt=0.05)
  lows = []
  for i in range(40):  # 2.0 s of alternation while ego closes at 0.5 m/s
    raw = 2.0 if i % 2 == 0 else 3.9
    sig = _step(ctx, v=0.5, gap=raw, dt=0.05)
    lows.append(sig.d_gap)
  assert min(lows) > 3.9 - 0.5 * 2.0 - 0.2  # never below the ego-propagated true track
  assert max(lows) <= 3.9 + 1e-9


def test_track_id_change_accepts_cut_in_immediately() -> None:
  ctx = StopContext()
  _step(ctx, gap=5.0, tid=1)
  sig = _step(ctx, gap=2.2, tid=2)
  assert sig.d_gap == pytest.approx(2.2)


def test_outward_step_persists_then_rate_limited() -> None:
  ctx = StopContext()
  _step(ctx, v=0.0, gap=3.0, dt=0.05)
  for _ in range(4):  # 0.20 s < T_PERSIST_OUT: held
    sig = _step(ctx, v=0.0, lv=0.5, gap=6.0, dt=0.05)
    assert sig.d_gap < 3.2
  sig = _step(ctx, v=0.0, lv=0.5, gap=6.0, dt=0.05)  # persisted: released at R_OUT = 0.5 + 0.5
  assert 3.1 < sig.d_gap < 3.25
  prev = sig.d_gap
  for _ in range(10):
    sig = _step(ctx, v=0.0, lv=0.5, gap=6.0, dt=0.05)
    assert sig.d_gap - prev <= (0.5 + 0.5) * 0.05 + 1e-9
    prev = sig.d_gap


# --- dropout decay-hold ---------------------------------------------------------------------------

def test_dropout_decays_inward_then_expires() -> None:
  ctx = StopContext()
  _step(ctx, gap=3.0)
  n_hold = int(round(T_DROPOUT_S / DT))
  for i in range(n_hold - 2):
    sig = _step(ctx, lead=False, gap=None)
    assert sig.dropout_active
    assert sig.d_gap == pytest.approx(3.0 - DROPOUT_DECAY_MPS * DT * (i + 1), abs=1e-6)
  for _ in range(4):  # window boundary (float accumulation tolerant): expires within a few frames
    sig = _step(ctx, lead=False, gap=None)
  assert sig.d_gap is None
  assert not sig.dropout_active
  assert sig.gap_source == "none"


def test_reacquire_after_dropout_accepts_fresh_reading() -> None:
  ctx = StopContext()
  _step(ctx, gap=3.0)
  for _ in range(5):
    _step(ctx, lead=False, gap=None)
  sig = _step(ctx, gap=2.8)  # within the closing-consistent slack of the decayed hold: immediate
  assert not sig.dropout_active
  assert sig.d_gap == pytest.approx(2.8, abs=1e-6)


# --- a_coast ---------------------------------------------------------------------------------------

def test_a_coast_converges_to_push_bias_and_clips() -> None:
  ctx = StopContext()
  for _ in range(500):  # 5 s: aEgo - cmd = +0.2 push
    sig = _step(ctx, v=1.0, a=-0.1, cmd=-0.3)
  assert sig.a_coast == pytest.approx(0.2, abs=0.02)
  for _ in range(800):  # residual +1.0 must clip at +0.5
    sig = _step(ctx, v=1.0, a=0.7, cmd=-0.3)
  assert sig.a_coast == pytest.approx(0.5, abs=1e-6)


def test_a_coast_held_below_v_threshold() -> None:
  ctx = StopContext()
  for _ in range(300):
    sig = _step(ctx, v=1.0, a=-0.1, cmd=-0.3)
  held = sig.a_coast
  for _ in range(200):  # below v = 0.1: hold, even with a wild residual
    sig = _step(ctx, v=0.05, a=2.0, cmd=-0.3)
  assert sig.a_coast == pytest.approx(held, abs=1e-9)


# --- wheel-stop latch ------------------------------------------------------------------------------

def test_wheel_stop_latches_after_quarter_second_and_resets_above_009() -> None:
  ctx = StopContext()
  sig = None
  for _ in range(24):
    sig = _step(ctx, v=0.05, lead=False)
    assert not sig.wheel_stop_latched
  sig = _step(ctx, v=0.05, lead=False)  # 0.25 s reached
  assert sig.wheel_stop_latched
  sig = _step(ctx, v=0.10, lead=False)  # any sample > 0.09 resets
  assert not sig.wheel_stop_latched


def test_wheel_stop_latches_on_quantized_003_006_alternation() -> None:
  ctx = StopContext()
  sig = None
  for i in range(30):
    sig = _step(ctx, v=0.03 if i % 2 == 0 else 0.06, lead=False)
  assert sig.wheel_stop_latched


def test_wheel_stop_latches_immediately_on_standstill_flag() -> None:
  ctx = StopContext()
  sig = _step(ctx, v=0.0, standstill=True, lead=False)
  assert sig.wheel_stop_latched


# --- lead_confirmed_stopped latch ------------------------------------------------------------------

def test_lead_stopped_latch_needs_hold_time() -> None:
  ctx = StopContext()
  sig = None
  for _ in range(29):
    sig = _step(ctx, gap=5.0, lv=0.2)
    assert not sig.lead_confirmed_stopped
  sig = _step(ctx, gap=5.0, lv=0.2)  # 0.3 s reached
  assert sig.lead_confirmed_stopped


def test_reversing_lead_is_never_confirmed_stopped() -> None:
  ctx = StopContext()
  for _ in range(100):
    sig = _step(ctx, gap=5.0, lv=-0.5)
    assert not sig.lead_confirmed_stopped
  # cycle-22: -0.5 sits ON the wide window's floor, so the ENTRY latch does confirm this lead --
  # by policy (a slow rollback is a stop to manage), not by accident.
  assert sig.lead_stopped_for_entry


# --- lead_stopped_for_entry latch (cycle-22: wide window for ENTRY only) ---------------------------

def test_slow_rollback_is_entry_eligible_but_never_strict() -> None:
  # USER POLICY (2026-08-02, f82 3 m stop): a lead rolling back slowly must not block entry --
  # refuse it and the takeover lands harsher and shorter than the 4-5 m aim. The STRICT latch
  # (relief-side consumers, cycle-17 reversing_hazard) must keep treating the same lead as
  # NOT-stopped so the gentle-rate machinery stays disqualified.
  ctx = StopContext()
  for _ in range(100):
    sig = _step(ctx, gap=5.0, lv=-0.3)
    assert not sig.lead_confirmed_stopped, "strict latch confirmed a reversing lead"
  assert sig.lead_stopped_for_entry, "slow rollback never became entry-eligible"


def test_fast_reversal_confirms_neither_latch() -> None:
  # below the wide floor (-0.5) the lead is a hazard approach, not a manageable stop
  ctx = StopContext()
  for _ in range(100):
    sig = _step(ctx, gap=5.0, lv=-0.6)
    assert not sig.lead_confirmed_stopped
    assert not sig.lead_stopped_for_entry


def test_wide_latch_unconfirms_when_rollback_accelerates() -> None:
  # a confirmed slow rollback that speeds past the wide floor un-confirms through the same
  # sustained-negative off-delay the strict latch uses (T_LEAD_NEG_OFF_S 0.5)
  ctx = StopContext()
  sig = None
  for _ in range(40):
    sig = _step(ctx, gap=5.0, lv=0.0)
  assert sig.lead_confirmed_stopped and sig.lead_stopped_for_entry
  for _ in range(60):  # 0.6 s at -0.6: past both off-delays
    sig = _step(ctx, gap=5.0, lv=-0.6)
  assert not sig.lead_confirmed_stopped
  assert not sig.lead_stopped_for_entry, "wide latch survived a sustained fast reversal"


def test_wide_latch_resists_the_alternating_dip_attack_in_its_own_band() -> None:
  # the cycle-21 attack, shifted into the wide latch's dip band [-0.9, -0.5): one in-window
  # frame, then 0.24 s just below the wide floor. The parameterised helper must carry the
  # AGGREGATE dip budget to the wide latch too -- this lead averages ~-0.67 m/s, genuinely
  # backing up fast, and must never become entry-eligible.
  ctx = StopContext()
  reversed_m = 0.0
  for k in range(2000):
    lead_v = -0.4 if k % 25 == 0 else -0.68
    sig = ctx.update(v_ego=1.0, a_ego=-0.3, a_cmd=-0.5, lead_status=True, lead_v=lead_v,
                     lead_d_rel=8.0, lead_track_id=1, standstill=False, dt=0.01)
    reversed_m += lead_v * 0.01
    assert not sig.lead_stopped_for_entry, (
      f"entry-confirmed a fast-reversing lead after {abs(reversed_m):.2f} m (frame {k})")
  assert abs(reversed_m) > 5.0, "fixture did not exercise a long reversal"


def test_moving_lead_resets_stop_latch() -> None:
  ctx = StopContext()
  for _ in range(40):
    sig = _step(ctx, gap=5.0, lv=0.0)
  assert sig.lead_confirmed_stopped
  sig = _step(ctx, gap=5.0, lv=0.6)
  assert not sig.lead_confirmed_stopped


def test_doppler_noise_does_not_unconfirm_a_stopped_lead() -> None:
  # ROUTE 00001f4c seg56 second stop (cycle-15): radar reported vLead -0.09..-0.20 for runs of up
  # to ~0.36 s on a PHYSICALLY STOPPED lead; the instantaneous un-confirm broke entry_ok mid-stop
  # -> spurious RELEASE -> re-entry pump -> the stale-anchor glide plunge behind the 0.89-carry
  # head bob. Negative Doppler must persist T_LEAD_NEG_OFF_S (0.5) to un-confirm.
  ctx = StopContext()
  sig = None
  for _ in range(40):
    sig = _step(ctx, gap=4.0, lv=0.02)
  assert sig.lead_confirmed_stopped
  for _ in range(36):  # the recorded worst noise run: 0.36 s below the -0.1 gate
    sig = _step(ctx, gap=4.0, lv=-0.15)
    assert sig.lead_confirmed_stopped, "recorded noise run un-confirmed the stopped lead"
  sig = _step(ctx, gap=4.0, lv=0.02)  # in-range frame resets the off-delay
  for _ in range(36):  # a second identical run must ALSO be survived (reset actually worked)
    sig = _step(ctx, gap=4.0, lv=-0.15)
    assert sig.lead_confirmed_stopped
  for _ in range(20):  # ...but sustained negative Doppler past 0.5 s DOES un-confirm
    sig = _step(ctx, gap=4.0, lv=-0.15)
  assert not sig.lead_confirmed_stopped, "a genuinely reversing lead must still un-confirm"


def test_lead_loss_and_drive_away_unconfirm_instantly() -> None:
  # The off-delay is scoped to negative Doppler ONLY: losing the lead or the lead driving away
  # (> +0.3) keep today's instantaneous un-confirm.
  ctx = StopContext()
  for _ in range(40):
    sig = _step(ctx, gap=4.0, lv=0.02)
  assert sig.lead_confirmed_stopped
  sig = _step(ctx, gap=4.0, lv=0.6)  # drive-away
  assert not sig.lead_confirmed_stopped
  ctx2 = StopContext()
  for _ in range(40):
    sig = _step(ctx2, gap=4.0, lv=0.02)
  sig = _step(ctx2, gap=None, lv=0.0, lead=False)  # lead loss
  assert not sig.lead_confirmed_stopped


# --- NaN robustness --------------------------------------------------------------------------------

def test_nan_inputs_hold_last_good_state() -> None:
  ctx = StopContext()
  for _ in range(200):
    _step(ctx, v=1.0, a=-0.1, cmd=-0.3, gap=5.0)
  before = _step(ctx, v=1.0, a=-0.1, cmd=-0.3, gap=4.99)
  nan = float("nan")
  for _ in range(20):
    sig = _step(ctx, v=nan, a=nan, cmd=nan, gap=nan, lv=nan)
  assert sig.d_gap is not None and math.isfinite(sig.d_gap)
  assert abs(sig.d_gap - before.d_gap) < 0.5
  assert math.isfinite(sig.a_coast)
  assert sig.a_coast == pytest.approx(before.a_coast, abs=0.05)
  sig = _step(ctx, v=1.0, a=-0.1, cmd=-0.3, gap=4.7)  # recovers cleanly
  assert sig.d_gap == pytest.approx(4.7, abs=0.3)


def test_alternating_dips_never_confirm_a_reversing_lead() -> None:
  # CYCLE-21 REVIEW (HIGH): the dwell-pause budget must be AGGREGATE over the confirmation epoch.
  # A per-excursion allowance that refreshed on every in-window frame let this pattern -- one
  # in-window frame, then 0.24 s just inside the dip band -- accumulate 0.30 s of dwell and
  # confirm a lead that had genuinely reversed 3.4 m.
  # CYCLE-22 NOTE: this pins the STRICT latch only. The wide ENTRY latch treats -0.49 as
  # in-window and confirms this lead BY POLICY (slow rollback = manageable stop); the shifted
  # attack against the wide latch's own dip band is pinned separately above.
  ctx = StopContext()
  reversed_m = 0.0
  for k in range(2000):
    lead_v = 0.0 if k % 25 == 0 else -0.49
    sig = ctx.update(v_ego=1.0, a_ego=-0.3, a_cmd=-0.5, lead_status=True, lead_v=lead_v,
                     lead_d_rel=8.0, lead_track_id=1, standstill=False, dt=0.01)
    reversed_m += lead_v * 0.01
    assert not sig.lead_confirmed_stopped, (
      f"confirmed a reversing lead after {abs(reversed_m):.2f} m of reversal (frame {k})")
  assert abs(reversed_m) > 5.0, "fixture did not exercise a long reversal"


def test_brief_dip_still_lets_a_noisy_stopped_lead_confirm() -> None:
  # ...and the fix still does its job: dips of the measured length (p50 0.13 s, p90 0.29 s over
  # 226 corpus runs behind stationary leads) inside a single epoch must not prevent confirmation.
  ctx = StopContext()
  confirmed_at = None
  for k in range(200):
    lead_v = -0.16 if 20 <= k < 33 else 0.0      # one 0.13 s dip mid-dwell
    sig = ctx.update(v_ego=1.0, a_ego=-0.3, a_cmd=-0.5, lead_status=True, lead_v=lead_v,
                     lead_d_rel=8.0, lead_track_id=1, standstill=False, dt=0.01)
    if sig.lead_confirmed_stopped and confirmed_at is None:
      confirmed_at = k
  assert confirmed_at is not None, "a stationary lead with one brief dip never confirmed"
  assert confirmed_at < 60, f"confirmation took {confirmed_at} frames"


def test_dip_budget_is_fresh_after_a_sustained_doppler_unconfirm() -> None:
  # CYCLE-21 REVIEW R2: the sustained-negative un-confirm reset the dwell but left the aggregate
  # dip budget spent, so the NEXT confirmation epoch started with no allowance and a normal
  # measured-length dip (0.13 s) reset it instead of confirming -- newly introduced leakage that
  # would delay entry after ordinary stationary-lead radar noise.
  ctx = StopContext()
  def run(frames, lead_v_fn):
    seen = False
    for k in range(frames):
      sig = ctx.update(v_ego=1.0, a_ego=-0.3, a_cmd=-0.5, lead_status=True,
                       lead_v=lead_v_fn(k), lead_d_rel=8.0, lead_track_id=1,
                       standstill=False, dt=0.01)
      seen = seen or sig.lead_confirmed_stopped
    return seen, sig
  # epoch 1: confirm while consuming the dip budget
  ok1, _ = run(200, lambda k: -0.16 if 20 <= k < 33 else 0.0)
  assert ok1, "epoch 1 never confirmed"
  # sustained negative Doppler un-confirms (T_LEAD_NEG_OFF_S)
  _, sig = run(80, lambda k: -0.30)
  assert not sig.lead_confirmed_stopped, "sustained Doppler did not un-confirm"
  # epoch 2: the same measured-length dip must confirm again, inside 0.6 s
  confirmed_at = None
  for k in range(60):
    lead_v = -0.16 if 10 <= k < 23 else 0.0
    sig = ctx.update(v_ego=1.0, a_ego=-0.3, a_cmd=-0.5, lead_status=True, lead_v=lead_v,
                     lead_d_rel=8.0, lead_track_id=1, standstill=False, dt=0.01)
    if sig.lead_confirmed_stopped and confirmed_at is None:
      confirmed_at = k
  # NOTE, measured: this pins that a second epoch confirms normally, but it does NOT discriminate
  # the budget clear added alongside it -- with and without that clear, confirmation lands on the
  # same frame (42). The sustained-negative reset leaves _lead_dip_t set for exactly one frame,
  # and the next out-of-window frame reaches the else-branch, which clears it anyway. The clear
  # is kept as hygiene (no reset path should leave epoch state behind), not as a behaviour fix,
  # and this test is honest about being a plain second-epoch regression.
  assert confirmed_at is not None, "the second epoch never confirmed"
  assert confirmed_at <= 60, f"second epoch confirmation was delayed to frame {confirmed_at}"
