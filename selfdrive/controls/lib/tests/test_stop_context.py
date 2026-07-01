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


def test_moving_lead_resets_stop_latch() -> None:
  ctx = StopContext()
  for _ in range(40):
    sig = _step(ctx, gap=5.0, lv=0.0)
  assert sig.lead_confirmed_stopped
  sig = _step(ctx, gap=5.0, lv=0.6)
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
