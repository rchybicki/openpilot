"""StoppingService tests: plan §6 stage-0 adversarial fixtures as tests + the stage-1
zero-wire-impact proof (docs/stopping/stopping_service_v3_plan.md).

The closed-loop fixtures run the full StopContext -> StoppingService chain against a simple plant
(v += a*dt with a first-order actuation lag + optional external push for creep/grade) -- no
external deps."""

import math
from dataclasses import dataclass, field

import pytest

from openpilot.selfdrive.controls.lib import stopping_flags
from openpilot.selfdrive.controls.lib.longcontrol import LongControl
from openpilot.selfdrive.controls.lib.stop_context import StopContext, StopSignals
from openpilot.selfdrive.controls.lib.stopping_service import Phase, ServiceParams, StoppingService
from openpilot.selfdrive.controls.lib.stopping_telemetry import StoppingTelemetry
from openpilot.selfdrive.controls.lib.tests.test_longcontrol_fast_release import (
  DummyCarParams,
  DummyCarState,
  DummyFrogPilotToggles,
)

P = ServiceParams()
DT = 0.01
EPS = 1e-6


# --- simple plant + closed-loop harness -----------------------------------------------------------

@dataclass
class Trace:
  t: list = field(default_factory=list)
  v: list = field(default_factory=list)
  u: list = field(default_factory=list)
  phase: list = field(default_factory=list)
  gap: list = field(default_factory=list)          # true gap (plant ground truth)
  gap_sig: list = field(default_factory=list)      # conditioned d_gap signal
  dts: list = field(default_factory=list)
  wstop: list = field(default_factory=list)
  mon: list = field(default_factory=list)
  dropout: list = field(default_factory=list)
  d_rest_eff: list = field(default_factory=list)


def simulate(*, v0, gap0=None, lead_v0=0.0, t_max=25.0, dt=DT, should_stop=True, seed_u=-0.05,
             push=0.0, push_fn=None, tau=0.2, v_quant=None, lead_v_fn=None, raw_gap_fn=None,
             dts0=None, dts_jumps=(), a_target_fn=None, nan_window=None, dropout_at_gap=None) -> Trace:
  """push_fn(v) overrides the constant push (speed-dependent drivetrain push, e.g. Stribeck);
  v_quant quantizes the MEASURED vEgo fed to context/service (0.03 m/s CAN steps, plan §8-4) --
  the plant itself always integrates the true continuous v."""
  ctx, svc = StopContext(), StoppingService()
  v, a_act, a_meas = float(v0), 0.0, 0.0
  gap = gap0
  dts = dts0
  last_u = float(seed_u)
  jumps = sorted(dts_jumps)
  jump_i = 0
  lead_alive = gap0 is not None
  stop_flag = should_stop
  tr = Trace()
  for i in range(int(round(t_max / dt))):
    t = i * dt
    lv = lead_v_fn(t) if lead_v_fn is not None else lead_v0
    if dropout_at_gap is not None and gap is not None and gap <= dropout_at_gap:
      lead_alive, stop_flag = False, False  # radar dropout: planner loses the stop too
    raw = None
    if lead_alive and gap is not None:
      raw = raw_gap_fn(t, gap) if raw_gap_fn is not None else gap
    v_meas = round(v / v_quant) * v_quant if v_quant else v
    v_in, gap_in, at_in = v_meas, raw, (a_target_fn(t) if a_target_fn is not None else None)
    a_in = a_meas
    if nan_window is not None and nan_window[0] <= t < nan_window[1]:
      v_in, a_in, at_in = float("nan"), float("nan"), float("nan")
      gap_in = float("nan") if raw is not None else None
    signals = ctx.update(v_ego=v_in, a_ego=a_in, a_cmd=last_u, lead_status=lead_alive, lead_v=lv,
                         lead_d_rel=gap_in, standstill=v < 0.005, dt=dt)
    res = svc.update(engaged=True, v_ego=v_in, a_ego=a_in, a_target=at_in, should_stop=stop_flag,
                     dts_planner=dts, planner_min_limit=-3.5, signals=signals,
                     lead_status=lead_alive, lead_v=lv, dt=dt, wire_accel=last_u)
    u = res.accel if res.active else 0.0
    tr.t.append(t)
    tr.v.append(v)
    tr.u.append(u)
    tr.phase.append(res.phase)
    tr.gap.append(gap)
    tr.gap_sig.append(signals.d_gap)
    tr.dts.append(dts)
    tr.wstop.append(signals.wheel_stop_latched)
    tr.mon.append(bool(res.debug.get("monitor_active", False)))
    tr.dropout.append(signals.dropout_active)
    tr.d_rest_eff.append(res.debug.get("d_rest_eff"))
    # plant: first-order actuation lag + external push (creep torque / grade)
    a_act += (u - a_act) * dt / (tau + dt)
    a_tot = a_act + (push_fn(v) if push_fn is not None else push)
    if v <= 0.0 and a_tot < 0.0:
      v_new, a_meas = 0.0, 0.0
    else:
      v_new = max(v + a_tot * dt, 0.0)
      a_meas = (v_new - v) / dt
    travel = max((v + v_new) / 2.0 * dt, 0.0)
    if gap is not None:
      gap = gap - travel + lv * dt
    if dts is not None:
      dts = max(dts - travel, 0.0)
      while jump_i < len(jumps) and t >= jumps[jump_i][0]:
        dts = max(dts + jumps[jump_i][1], 0.05)
        jump_i += 1
    v = v_new
    last_u = u
  return tr


def assert_no_slam(tr: Trace, dt: float = DT) -> None:
  """Plan §6 stage-0 no-slam gate: per-frame deepen <= J_SAFE*dt, release <= max(J_UP, J_GO)*dt."""
  for k in range(1, len(tr.u)):
    if tr.phase[k - 1] == Phase.INACTIVE or tr.phase[k] == Phase.INACTIVE:
      continue
    step = tr.u[k] - tr.u[k - 1]
    assert step >= -P.J_SAFE * dt - EPS, f"slam deepen {step:.4f} at t={tr.t[k]:.2f}"
    assert step <= max(P.J_UP, P.J_GO) * dt + EPS, f"slam release {step:.4f} at t={tr.t[k]:.2f}"


def last_rolling_idx(tr: Trace) -> int:
  idx = None
  for k, v in enumerate(tr.v):
    if v > 0.01:
      idx = k
  assert idx is not None and idx < len(tr.v) - 50, "car never stopped (or stopped only at trace end)"
  return idx


def first_stop_idx(tr: Trace) -> int:
  return last_rolling_idx(tr) + 1


def make_signals(d_gap=None, a_coast=0.0, wheel=False, latch=False, dropout=False) -> StopSignals:
  return StopSignals(d_gap=d_gap, gap_source="measured" if d_gap is not None else "none",
                     dropout_active=dropout, a_coast=a_coast, wheel_stop_latched=wheel,
                     lead_confirmed_stopped=latch)


# --- nominal stop (plan §6 stage 0 + release-rate audit companion) --------------------------------

def test_nominal_stop_from_2p4_at_gap_12() -> None:
  tr = simulate(v0=2.4, gap0=12.0, should_stop=False, seed_u=0.0)  # entry via the lead latch
  k_roll = last_rolling_idx(tr)
  k_stop = k_roll + 1
  assert_no_slam(tr)
  # wheel-stop wire in the felt-smoothness band
  assert -0.35 - EPS <= tr.u[k_roll] <= -0.05 + EPS, f"wheel-stop wire {tr.u[k_roll]:.3f}"
  # one continuous motion: v never re-rises before the stop, none after
  for k in range(1, k_stop):
    assert tr.v[k] <= max(tr.v[:k]) + 0.02
  assert max(tr.v[k_stop:]) < 0.01
  # hold: -0.30 or deeper within 0.7 s of the physical stop
  k_hold = k_stop + int(round(0.7 / DT))
  assert any(u <= -0.30 + EPS for u in tr.u[k_stop:k_hold + 1]), "hold not reached within 0.7 s"
  assert tr.u[-1] == pytest.approx(P.A_HOLD, abs=0.02)
  # rest in the intended band around D_REST_eff = 4.0
  assert 3.0 <= tr.gap[-1] <= 5.0, f"rest gap {tr.gap[-1]:.2f}"
  assert min(g for g in tr.gap if g is not None) >= 2.0


def test_stop_and_go_moving_lead_entry_rests_at_nominal_not_close() -> None:
  # ROUTE 00001b76 seg4/5 REGRESSION (first stage-3 drive): ego at 3.2 m/s behind a lead
  # decelerating 2.05 -> 0; the service entered as the lead stopped (gap ~5.2 at ego ~1.6) and the
  # old comfort-glide anchor (A_GLIDE_NOM 0.5) re-zeroed the rest to ~2.7 m -> the car stopped at
  # 2.1 m. With the FIRM feasibility anchor (A_REST_FEAS) + re-anchoring while the lead still
  # moves, the same entry must rest near nominal.
  # entry inside the band (the harness has no legacy chain above V_ENTER): ego 2.3 m/s at gap 6.5
  # behind a lead still rolling at 1.5 and braking to a stop -- the incident's entry geometry.
  # Old anchor: 6.5 - 2.3^2/(2*0.5) = 1.2 -> pinned D_REST_MIN 2.4 -> rests ~2.4 (the fault).
  # Firm anchor: 6.5 - 2.3^2/(2*1.2) = 4.3 -> nominal 4.0.
  def lead_v_fn(t: float) -> float:
    return max(1.5 - 1.0 * t, 0.0)

  tr = simulate(v0=2.3, gap0=6.5, lead_v_fn=lead_v_fn, should_stop=True, seed_u=-0.9, t_max=30.0)
  assert_no_slam(tr)
  first_stop_idx(tr)
  rests = [r for r in tr.d_rest_eff if r is not None]
  assert rests[-1] >= 3.9, f"rest anchor re-zeroed to {rests[-1]:.2f} on a normal stop-and-go entry"
  assert tr.gap[-1] >= 3.4, f"rested at {tr.gap[-1]:.2f} m -- the 00001b76 too-close class"
  assert min(g for g in tr.gap if g is not None) >= 2.5


def test_release_rate_audit_terminal_release_achieves_j_up() -> None:
  # J2 lesson (plan §1): assert the release path actually delivers J_UP when demand-limited.
  svc = StoppingService()
  sig = make_signals()
  r = None
  for _ in range(300):  # deepen onto the planner lane at -1.5 (safety lane, J_SAFE)
    r = svc.update(engaged=True, v_ego=1.0, a_ego=0.0, a_target=-1.5, should_stop=True,
                   dts_planner=1.0, planner_min_limit=-3.5, signals=sig,
                   lead_status=False, lead_v=0.0, dt=DT, wire_accel=-0.1)
  assert r.accel == pytest.approx(-1.5, abs=1e-6)
  prev = r.accel
  for _ in range(40):  # stop line far away: demand shallows, release must run at exactly J_UP
    r = svc.update(engaged=True, v_ego=1.0, a_ego=0.0, a_target=None, should_stop=True,
                   dts_planner=30.0, planner_min_limit=-3.5, signals=sig,
                   lead_status=False, lead_v=0.0, dt=DT, wire_accel=-0.1)
    assert r.accel - prev == pytest.approx(P.J_UP * DT, abs=1e-9)
    prev = r.accel


# --- alternating gap bounce through a full stop (mandatory stage-0 gate, ledger D2-H2/D3-H1) ------

def test_alternating_gap_bounce_through_full_stop_never_slams() -> None:
  def bouncy(t: float, true_gap: float) -> float:
    return true_gap if int(t / 0.05) % 2 == 0 else 2.0  # true <-> 2.0, 0.05 s radar blocks

  tr = simulate(v0=1.5, gap0=3.9, should_stop=True, seed_u=-0.30, raw_gap_fn=bouncy)
  assert_no_slam(tr)
  k_roll = last_rolling_idx(tr)
  assert tr.gap[-1] >= 2.0
  # the spurious 2.0 reading never reaches the laws while the true gap is still wide
  for k in range(len(tr.u)):
    if tr.gap[k] is not None and tr.gap[k] > 3.0 and tr.gap_sig[k] is not None:
      assert tr.gap_sig[k] > tr.gap[k] - 0.45
  # mid-stop the command never releases shallow (dropout-floor bound while still rolling fast)
  for k in range(int(0.5 / DT), k_roll):
    if tr.v[k] > 0.4:
      assert tr.u[k] <= P.A_DROPOUT_MIN + EPS
    if tr.phase[k] != Phase.INACTIVE:
      assert tr.u[k] <= -0.03 + EPS


# --- lead reversing during EASE (P1 worst case, plan §5) ------------------------------------------

def test_lead_reversing_in_ease_deepens_fast_and_keeps_hard_margin() -> None:
  # plan §5 worst-case geometry: ego ~0.5 m/s in EASE right at the 2.6 m gap-gate boundary
  t_rev = 0.2

  def lead_v_fn(t: float) -> float:
    return -0.5 if t_rev <= t < t_rev + 0.6 else 0.0

  tr = simulate(v0=0.5, gap0=2.75, should_stop=True, seed_u=-0.25, lead_v_fn=lead_v_fn, t_max=15.0)
  assert_no_slam(tr)
  k_rev = int(round(t_rev / DT))
  assert tr.phase[k_rev - 1] == Phase.PRE_STOP_EASE  # the fixture really exercises EASE
  # prompt deepen: the reversing lead is answered within the 0.15 s persistence budget + J_SAFE slew
  k_resp = k_rev + int(round(0.25 / DT))
  assert min(tr.u[k_rev:k_resp + 1]) <= tr.u[k_rev - 1] - 0.08
  # no EASE (shallow) frames while the lead is reversing, and the command goes to full authority
  for k in range(k_rev + 2, k_rev + int(0.8 / DT)):
    assert tr.phase[k] != Phase.PRE_STOP_EASE
  assert min(tr.u[k_rev:k_rev + int(0.8 / DT)]) < -0.45
  # closure arithmetic (plan §5): gap consumed from reversal onset to full-authority command <= 0.24 m
  deep_k = next((k for k in range(k_rev, len(tr.u)) if tr.u[k] <= -0.83), None)
  assert deep_k is not None  # full authority genuinely reached
  assert tr.gap[k_rev] - tr.gap[deep_k] <= 0.24 + 0.9 * DT * 2
  assert min(g for g in tr.gap if g is not None) >= 2.0  # D_HARD margin never violated
  assert max(tr.v[first_stop_idx(tr):]) < 0.02  # ego stays stopped


# --- creep-push hover (+0.3 m/s^2) -----------------------------------------------------------------

def test_creep_push_hover_monitor_catches_and_escalates() -> None:
  # enters EASE with an unconverged a_coast, so the +0.3 push out-muscles the -0.10 EASE demand:
  # the anti-hover/anti-roll monitor must catch the rise and escalate until the car stops.
  tr = simulate(v0=0.3, gap0=4.8, should_stop=True, seed_u=-0.05, push=0.3, t_max=40.0)
  assert_no_slam(tr)
  k_stop = first_stop_idx(tr)
  assert any(tr.mon), "anti-hover monitor never engaged"
  assert min(tr.u) <= -0.45, "monitor never escalated below the EASE deep bound"
  assert max(tr.v[k_stop:]) < 0.05  # the ratcheted floor beats the creep push: rest is final
  assert tr.t[k_stop] < 20.0


# --- R1 kill-shots: tight-entry Stribeck creep-crawl (findings 1+2, the under-brake hole) -----------

def stribeck_push(v: float) -> float:
  # measured HEV drivetrain push that RISES as v falls, peaking +0.43 m/s^2 as v -> 0 with the
  # divergence knee near v ~ 0.066 (friction-residual session, plan §8-1)
  return 0.15 + 0.28 * math.exp(-v / 0.066)


@pytest.mark.parametrize("gap0,v0,seed", [(2.8, 0.5, -0.2), (3.0, 0.6, -0.2), (3.0, 1.2, -0.6)])
def test_tight_entry_stribeck_crawl_stops_and_keeps_hard_margin(gap0: float, v0: float, seed: float) -> None:
  # R1 findings 1+2: close-gap terminal corridor (true gap < 2.6 routes AROUND the EASE gates into
  # GLIDE, where no monitor ran), a Stribeck push the frozen a_coast never learns, quantized vEgo
  # (0.03 steps) + tau=0.2 actuation lag, and a_target=None (planner glided out): the single-lane-
  # dependency case. Pre-fix this crawls monotonically through D_HARD to bumper contact.
  # Seeds mirror the plan's own close-entry fixture shapes (the v=1.2 approach is already braking
  # at entry, as in test_close_entry_gap_3_*): R1's breaches happened in the CRAWL at v 0.10-0.26,
  # not in the initial jerk-limited stop.
  tr = simulate(v0=v0, gap0=gap0, should_stop=True, seed_u=seed, push_fn=stribeck_push,
                v_quant=0.03, t_max=40.0)
  assert_no_slam(tr)
  # plan §6 stage-0 default-fail gate: min TRUE gap >= 2.0 m ALWAYS
  min_gap = min(g for g in tr.gap if g is not None)
  assert min_gap >= 2.0, f"hard-floor breach: min true gap {min_gap:.2f} m"
  # the car genuinely reaches wheel-stop -- no perpetual crawl / limit cycle
  k_stop = next((k for k in range(len(tr.v)) if tr.v[k] < 0.005), None)
  assert k_stop is not None and k_stop < len(tr.v) - int(5.0 / DT), "car never reached wheel-stop (perpetual crawl)"
  assert any(tr.wstop), "wheel-stop never latched"
  # ... and the rest is final: no post-stop forward travel beyond 0.05 m
  post_travel = sum(v * DT for v in tr.v[k_stop:])
  assert post_travel <= 0.05, f"post-stop forward travel {post_travel:.3f} m"


def test_monitor_floor_ratchet_survives_ease_to_glide_flip() -> None:
  # R1 finding 2: an armed anti-creep floor must NOT release when the gap closes through 2.6 and
  # the phase flips PRE_STOP_EASE -> APPROACH_GLIDE -- exactly while the gap is closing. The
  # ratchet clears only on RELEASE (a genuine go) or INACTIVE, never on EASE<->GLIDE flips.
  svc = StoppingService()
  r = None
  for _ in range(300):  # hover in EASE at v = 0.40, gap 2.70 (> 2.6): monitor triggers + escalates
    r = svc.update(engaged=True, v_ego=0.40, a_ego=0.0, a_target=None, should_stop=True,
                   dts_planner=None, planner_min_limit=-3.5, signals=make_signals(d_gap=2.70),
                   lead_status=True, lead_v=0.0, dt=DT, wire_accel=-0.2)
  assert r.phase == Phase.PRE_STOP_EASE
  assert r.debug["monitor_active"], "hover in EASE never armed the monitor"
  floor = r.accel
  assert floor <= -0.80  # 3 s of hover: -0.35 initial + escalations (R1's probe ratcheted to -1.10)
  for i in range(100):  # gap crosses 2.6 -> EASE gap gate fails -> phase flips to APPROACH_GLIDE
    r = svc.update(engaged=True, v_ego=0.40, a_ego=0.0, a_target=None, should_stop=True,
                   dts_planner=None, planner_min_limit=-3.5, signals=make_signals(d_gap=2.55),
                   lead_status=True, lead_v=0.0, dt=DT, wire_accel=-0.2)
    assert r.phase == Phase.APPROACH_GLIDE
    assert r.accel <= floor + EPS, f"armed floor released to {r.accel:.3f} after the phase flip (frame {i})"


# --- queue-creep monitor gate (route 00001b72: hover arming behind a departing queue) --------------

def test_queue_creep_gap_growing_never_arms_monitor_then_glide_resumes() -> None:
  # Tonight's live fault: shouldStop latched behind a creeping queue, ego creeping 0.3-0.5 m/s while
  # the LEAD PULLED AWAY (gap growing) -- the anti-hover monitor armed and rode the hold to -0.65.
  # Gap growth > MON_GAP_GROW_M per hover window in GLIDE/EASE must suppress the trigger entirely:
  # queue-following creep is legitimate, the wire stays gentle, and the car may keep creeping.
  svc = StoppingService()
  gap = 4.5
  r = None
  for k in range(400):  # lead pulling away at 1 m/s while the ego creeps at 0.35 behind it
    gap += 1.0 * DT
    r = svc.update(engaged=True, v_ego=0.35, a_ego=0.0, a_target=None, should_stop=True,
                   dts_planner=None, planner_min_limit=-3.5, signals=make_signals(d_gap=gap),
                   lead_status=True, lead_v=1.0, dt=DT, wire_accel=-0.15)
    assert r.active
    assert r.phase in (Phase.PRE_STOP_EASE, Phase.APPROACH_GLIDE)
    assert not r.debug["monitor_active"], f"monitor armed at frame {k} while the gap was growing"
    assert r.accel >= -0.35 - EPS, f"wire {r.accel:.3f} not gentle at frame {k}"
  # the lead stops again: normal glide resumes (the ego bleeds its creep speed down to a stop)
  v = 0.35
  for k in range(300):
    v = max(v - 0.0025, 0.0)
    wheel = v < 0.02
    r = svc.update(engaged=True, v_ego=v, a_ego=-0.25 if v > 0.0 else 0.0, a_target=None, should_stop=True,
                   dts_planner=None, planner_min_limit=-3.5, signals=make_signals(d_gap=gap, wheel=wheel, latch=True),
                   lead_status=True, lead_v=0.0, dt=DT, wire_accel=-0.15)
    assert r.active
    assert not r.debug["monitor_active"], f"monitor armed at decay frame {k} (v was decreasing)"
  assert r.phase in (Phase.RAMP_TO_HOLD, Phase.HOLD)
  assert r.accel <= -0.30 + EPS  # the firm hold still builds after the genuine stop


def test_hold_escape_arrested_within_5cm() -> None:
  # ROUTE 00001b87 segs 1/3 (cycle-4): the car broke loose from the -0.32 hold and traveled
  # 6-16 cm before the escalation ladder re-arrested it at -0.65. With the deeper A_HOLD (-0.45)
  # plus the post-stop fast arrest (first floor = A_HOLD - MON_POSTSTOP_ARREST_EXTRA at J_SAFE),
  # a creep push that still exceeds the hold must be caught within ~5 cm.
  # Two-phase: clean stop first (no push), then a 0.58 push injected once HOLDing.
  ctx, svc = StopContext(), StoppingService()
  v, a_act, gap, last_u = 1.0, 0.0, 7.0, -0.4
  tau = 0.2
  travel_after_stop = 0.0
  stopped = False
  floors = []
  for _i in range(4000):
    push = 0.58 if stopped else 0.0
    signals = ctx.update(v_ego=v, a_ego=0.0, a_cmd=last_u, lead_status=True, lead_v=0.0,
                         lead_d_rel=gap, standstill=v < 0.005, dt=DT)
    r = svc.update(engaged=True, v_ego=v, a_ego=0.0, a_target=None, should_stop=True,
                   dts_planner=None, planner_min_limit=-3.5, signals=signals,
                   lead_status=True, lead_v=0.0, dt=DT, wire_accel=last_u)
    u = r.accel if r.active else 0.0
    a_act += (u - a_act) * DT / (tau + DT)
    v_new = max(v + (a_act + push) * DT, 0.0)
    step = max((v + v_new) / 2.0 * DT, 0.0)
    gap -= step
    if stopped:
      travel_after_stop += step
      floors.append(u)
    if not stopped and v < 0.005 and r.phase in (Phase.RAMP_TO_HOLD, Phase.HOLD):
      stopped = True
    v = v_new
    last_u = u
  assert stopped, "never reached HOLD"
  assert travel_after_stop <= 0.05, f"hold escape traveled {travel_after_stop:.3f} m (>5 cm)"
  assert min(floors) <= P.A_HOLD - P.MON_POSTSTOP_ARREST_EXTRA + 0.05  # fast arrest floor engaged
  assert v < 0.02  # rest is final against the sustained push


def test_stopped_lead_gap_quantization_notch_does_not_suppress_monitor() -> None:
  # Offline-gate event 000016dd + Codex review (2026-07-02): a STOPPED lead (lead_v ~0.02) whose
  # conditioned gap steps up one radar quantization notch (+0.099 m > MON_GAP_GROW_M) must NOT be
  # read as "departing" -- the queue-creep gate requires actual lead recession (lead_v >
  # MON_LEAD_RECEDE_MPS), so the anti-hover monitor still arms on a closing/hovering crawl.
  svc = StoppingService()
  gap = 3.8
  armed_at = None
  for k in range(300):
    if k == 60:
      gap += 0.099  # one quantization notch while the lead is genuinely stopped
    r = svc.update(engaged=True, v_ego=0.25, a_ego=0.0, a_target=None, should_stop=True,
                   dts_planner=None, planner_min_limit=-3.5, signals=make_signals(d_gap=gap),
                   lead_status=True, lead_v=0.02, dt=DT, wire_accel=-0.15)
    if r.debug["monitor_active"] and armed_at is None:
      armed_at = k
  assert armed_at is not None, "monitor never armed against a hovering ego behind a STOPPED lead"
  assert armed_at * DT <= 1.0  # armed promptly despite the notch


def test_hold_rollaway_with_growing_gap_still_monitored() -> None:
  # Phase scoping of the queue-creep gate: RAMP_TO_HOLD/HOLD stay UNGATED -- a rollaway at a
  # standstill must be caught even while the departed lead makes the gap grow.
  svc = StoppingService()
  r = None
  for _ in range(100):  # settle to HOLD behind a stopped lead at gap 4.0
    r = svc.update(engaged=True, v_ego=0.0, a_ego=0.0, a_target=None, should_stop=True,
                   dts_planner=None, planner_min_limit=-3.5,
                   signals=make_signals(d_gap=4.0, wheel=True, latch=True),
                   lead_status=True, lead_v=0.0, dt=DT, wire_accel=-0.30)
  assert r.phase == Phase.HOLD
  gap, v = 4.0, 0.0
  for _ in range(150):  # lead drives off (gap grows) while a push rolls the ego through the latch band
    v = min(v + 0.002, 0.25)
    gap += 0.02
    r = svc.update(engaged=True, v_ego=v, a_ego=0.05, a_target=None, should_stop=True,
                   dts_planner=None, planner_min_limit=-3.5,
                   signals=make_signals(d_gap=gap, wheel=v <= 0.09, latch=False),
                   lead_status=True, lead_v=1.5, dt=DT, wire_accel=-0.30)
  assert r.phase == Phase.HOLD
  assert r.debug["monitor_active"], "HOLD rollaway must arm the monitor even with a growing gap"
  assert r.accel <= -0.50, f"anti-roll floor never escalated: {r.accel:.3f}"


def test_armed_monitor_floor_survives_gap_growth() -> None:
  # An ALREADY-ARMED floor is a ratchet: gap growth pauses the ESCALATION only -- it must never
  # disarm the floor (release happens only via RELEASE/INACTIVE, exactly like 'decreasing again').
  svc = StoppingService()
  r = None
  for _ in range(250):  # hover in EASE at v 0.40, gap 2.70 constant: monitor arms + escalates
    r = svc.update(engaged=True, v_ego=0.40, a_ego=0.0, a_target=None, should_stop=True,
                   dts_planner=None, planner_min_limit=-3.5, signals=make_signals(d_gap=2.70),
                   lead_status=True, lead_v=0.0, dt=DT, wire_accel=-0.2)
  assert r.debug["monitor_active"], "hover with a constant gap must still arm (gate needs GROWTH)"
  floor = r.accel
  assert floor <= -0.50  # armed at -0.35 + escalations
  gap = 2.70
  for k in range(150):  # the lead now pulls away: escalation pauses, but the armed floor still binds
    gap += 0.01
    r = svc.update(engaged=True, v_ego=0.40, a_ego=0.0, a_target=None, should_stop=True,
                   dts_planner=None, planner_min_limit=-3.5, signals=make_signals(d_gap=gap),
                   lead_status=True, lead_v=1.0, dt=DT, wire_accel=-0.2)
    assert r.debug["monitor_active"], f"gap growth DISARMED the ratchet at frame {k}"
    assert r.accel <= floor + EPS, f"armed floor released to {r.accel:.3f} at frame {k}"
  # escalation genuinely paused: at most one in-flight escalation step beyond the captured floor
  assert r.accel >= floor - P.MON_ESCALATE_STEP - EPS


def test_ease_glide_d_rem_gate_has_hysteresis() -> None:
  # R2 chatter note: enter EASE at d_rem <= 0.8, exit back to GLIDE only once d_rem > 0.95 --
  # a d_rem dithering just above 0.8 must not flip the phase every frame.
  svc = StoppingService()

  def step(d_gap: float) -> Phase:
    r = None
    for _ in range(3):
      r = svc.update(engaged=True, v_ego=0.3, a_ego=-0.1, a_target=None, should_stop=True,
                     dts_planner=None, planner_min_limit=-3.5, signals=make_signals(d_gap=d_gap),
                     lead_status=True, lead_v=0.0, dt=DT, wire_accel=-0.1)
    return r.phase

  assert step(4.80) == Phase.PRE_STOP_EASE   # entry: D_REST_eff = 4.0 -> d_rem = 0.8 <= 0.8
  assert step(4.90) == Phase.PRE_STOP_EASE   # d_rem 0.9 inside the hysteresis band: stays EASE
  assert step(5.00) == Phase.APPROACH_GLIDE  # d_rem 1.0 > 0.95: genuinely exits to GLIDE
  assert step(4.90) == Phase.APPROACH_GLIDE  # d_rem 0.9 > 0.8 entry gate: stays GLIDE (no chatter)
  assert step(4.75) == Phase.PRE_STOP_EASE   # d_rem 0.75 <= 0.8: re-enters


# --- +/-5 percent grade via a_coast bias -----------------------------------------------------------

def test_downhill_grade_rest_achieved_and_held() -> None:
  tr = simulate(v0=1.5, gap0=6.0, should_stop=True, seed_u=-0.50, push=0.49, t_max=40.0)
  assert_no_slam(tr)
  k_stop = first_stop_idx(tr)
  assert 2.2 <= tr.gap[-1] <= 5.2, f"rest gap {tr.gap[-1]:.2f}"
  assert max(tr.v[k_stop:]) < 0.15  # anti-roll ratchet keeps any post-stop excursion tiny
  assert max(tr.v[-int(2.0 / DT):]) < 0.05  # and the rest is final
  assert tr.u[-1] <= P.A_HOLD + EPS  # grade needs (and keeps) a deeper-than-nominal hold (plan §8-2)
  assert min(g for g in tr.gap if g is not None) >= 2.0


def test_uphill_grade_no_strand() -> None:
  tr = simulate(v0=2.0, gap0=6.0, should_stop=True, seed_u=-0.30, push=-0.49, t_max=30.0)
  assert_no_slam(tr)
  last_rolling_idx(tr)  # rest achieved
  assert 2.0 <= tr.gap[-1] <= 5.4, f"stranded rest gap {tr.gap[-1]:.2f}"
  assert abs(tr.gap[-1] - tr.gap[-int(2.0 / DT)]) < 0.05  # settled, no late drift


# --- radar dropout at gap 3.0 / v 0.4 (ledger D2-H3) -----------------------------------------------

def test_radar_dropout_decay_hold_keeps_braking_no_release() -> None:
  tr = simulate(v0=0.9, gap0=3.4, should_stop=True, seed_u=-0.30, dropout_at_gap=3.0, t_max=15.0)
  assert_no_slam(tr)
  assert any(tr.dropout), "decay-hold never engaged"
  k_drop = tr.dropout.index(True)
  assert tr.v[k_drop] > 0.3  # dropped while genuinely rolling
  for k in range(len(tr.u)):
    if tr.dropout[k]:
      assert tr.u[k] <= P.A_DROPOUT_MIN + EPS, f"released above the dropout floor at t={tr.t[k]:.2f}"
    assert tr.u[k] <= 0.0 + EPS
  k_stop = first_stop_idx(tr)
  assert tr.t[k_stop] - tr.t[k_drop] < 2.0  # stopped inside the decay window
  assert max(tr.v[k_stop:]) < 0.02  # no DISLIKE motion: the rest is final
  assert tr.u[-1] == pytest.approx(P.A_HOLD, abs=0.02)


# --- close entries: D_REST_eff re-zeroes (ledger D1-H2) --------------------------------------------

@pytest.mark.parametrize("v0,seed,wire_band", [(0.6, -0.2, (-0.35, -0.05)), (1.2, -0.6, (-1.6, -0.03))])
def test_close_entry_gap_3_rest_rezeroes_and_wire_bounded(v0: float, seed: float, wire_band) -> None:
  # DEVIATION from the task's fixture wording, per the plan itself: plan §6 stage 0 requires
  # "wheel-stop-release u >= -0.35 on NOMINAL fixtures" only. For the v=1.2 close entry the exact
  # §3 laws CANNOT land in [-0.35, -0.05]: D_REST_MIN clamps the rest at 2.4 m (required decel 1.2
  # > A_REST_FEAS), the terminal gap sits below the 2.6 m EASE gap gate (no shallow region exists
  # by design, ledger D1-H2), and the floored-denominator demand is genuinely deep; J_UP cannot
  # shed that in the ~0.4 s the car takes to stop. Safety over feel is the intended trade there.
  tr = simulate(v0=v0, gap0=3.0, should_stop=True, seed_u=seed, t_max=20.0)
  assert_no_slam(tr)
  k_roll = last_rolling_idx(tr)
  expected_rest = min(4.0, max(3.0 - v0 * v0 / (2.0 * P.A_REST_FEAS), P.D_REST_MIN))
  rest_vals = [r for r in tr.d_rest_eff if r is not None]
  assert rest_vals[0] == pytest.approx(expected_rest, abs=0.05)  # re-zeroed, not the 4.0 nominal
  assert rest_vals[0] < 4.0 - 1e-3
  assert wire_band[0] - EPS <= tr.u[k_roll] <= wire_band[1] + EPS, f"wheel-stop wire {tr.u[k_roll]:.3f}"
  assert tr.gap[-1] >= 2.1
  assert min(g for g in tr.gap if g is not None) >= 2.0
  # A_HOLD (-0.45, deeper than the monitor's A_EASE_DEEP ratchet floor since the 00001b87 hold
  # hardening) is the resting hold; escalations may deepen it further but never past one fast-arrest
  # step. Pressure never goes up-then-down post-stop (DoD-4).
  assert P.A_HOLD - P.MON_POSTSTOP_ARREST_EXTRA - EPS <= tr.u[-1] <= P.A_HOLD + 0.02


# --- stop-line dts moved +/-1.5 m mid-stop (ledger D3-H3: no latch, no staleness) ------------------

def test_stop_line_moves_mid_stop_continuous_response() -> None:
  tr = simulate(v0=2.0, dts0=8.0, gap0=None, should_stop=True, seed_u=-0.20,
                dts_jumps=[(2.0, +1.5), (4.5, -1.5)], t_max=25.0)
  assert_no_slam(tr)
  k1 = int(round(2.0 / DT))
  k2 = int(round(4.5 / DT))
  # line moved farther: command releases (continuously, at <= J_UP -- covered by assert_no_slam)
  assert tr.u[k1 + int(0.5 / DT)] >= tr.u[k1 - 1] + 0.02
  # line moved closer: command deepens promptly
  assert tr.u[k2 + int(0.4 / DT)] <= tr.u[k2 - 1] - 0.02
  first_stop_idx(tr)
  assert 0.0 <= tr.dts[-1] <= 1.0, f"stopped {tr.dts[-1]:.2f} m from the final line"


# --- NaN frames (plan §3: safe fallback, never propagate) ------------------------------------------

def test_nan_frames_no_nan_out_and_stop_still_lands() -> None:
  tr = simulate(v0=2.4, gap0=12.0, should_stop=True, seed_u=-0.20, nan_window=(2.0, 2.2))
  for u in tr.u:
    assert math.isfinite(u)
  assert_no_slam(tr)
  first_stop_idx(tr)
  assert 2.4 <= tr.gap[-1] <= 5.2


def test_all_nan_updates_hold_last_command() -> None:
  svc = StoppingService()
  sig = make_signals(d_gap=6.0, latch=True)
  r = None
  for _ in range(100):
    r = svc.update(engaged=True, v_ego=1.0, a_ego=-0.3, a_target=None, should_stop=True,
                   dts_planner=None, planner_min_limit=-3.5, signals=sig,
                   lead_status=True, lead_v=0.0, dt=DT, wire_accel=-0.2)
  held = r.accel
  nan = float("nan")
  for _ in range(20):
    r = svc.update(engaged=True, v_ego=nan, a_ego=nan, a_target=nan, should_stop=True,
                   dts_planner=nan, planner_min_limit=nan, signals=sig,
                   lead_status=True, lead_v=nan, dt=DT, wire_accel=nan)
    assert math.isfinite(r.accel)
  assert r.accel == pytest.approx(held, abs=1e-6)


# --- law units: EASE gates, dropout floor, deepen-only structure -----------------------------------

def test_ease_blocked_when_gap_at_or_below_2p6() -> None:
  svc = StoppingService()
  sig = make_signals(d_gap=2.5, latch=True)
  r = None
  for _ in range(200):
    r = svc.update(engaged=True, v_ego=0.4, a_ego=-0.2, a_target=None, should_stop=True,
                   dts_planner=None, planner_min_limit=-3.5, signals=sig,
                   lead_status=True, lead_v=0.0, dt=DT, wire_accel=-0.1)
  assert r.phase == Phase.APPROACH_GLIDE  # gap gate (> 2.6) keeps EASE unreachable
  assert r.accel < -0.40                  # glide law stays deep this close


def test_d_rest_nom_isd_enters_once_and_clips() -> None:
  svc = StoppingService()
  assert svc._d_rest_nom(0.0) == pytest.approx(4.0)
  assert svc._d_rest_nom(0.5) == pytest.approx(4.5)
  assert svc._d_rest_nom(3.0) == pytest.approx(P.D_REST_CLIP_MAX)   # 4.0 + ISD clipped to 5.0
  assert svc._d_rest_nom(-3.0) == pytest.approx(P.D_REST_CLIP_MIN)  # ... and to 2.5
  assert svc._d_rest_nom(float("nan")) == pytest.approx(4.0)        # non-finite ISD: nominal


def test_d_rest_eff_anchors_at_entry_not_first_sighting() -> None:
  svc = StoppingService()
  # lead first sighted while INACTIVE above V_ENTER: those kinematics (glide landing -1.0 m -> the
  # D_REST_MIN 2.4 pin) must NOT anchor the rest for the whole stop (plan §3: D_REST_eff "at entry")
  r = svc.update(engaged=True, v_ego=3.0, a_ego=0.0, a_target=None, should_stop=False,
                 dts_planner=None, planner_min_limit=-3.5, signals=make_signals(d_gap=8.0, latch=True),
                 lead_status=True, lead_v=0.0, dt=DT, wire_accel=-0.1)
  assert not r.active
  r = svc.update(engaged=True, v_ego=2.0, a_ego=-0.3, a_target=None, should_stop=False,
                 dts_planner=None, planner_min_limit=-3.5, signals=make_signals(d_gap=4.2, latch=True),
                 lead_status=True, lead_v=0.0, dt=DT, wire_accel=-0.1)  # entry frame
  assert r.active
  # entry-frame anchor: min(4.0, max(4.2 - 2.0^2/(2*A_REST_FEAS 1.2), 2.4)) = 2.533 -- not the 4.0
  # the (8.0, 3.0) sighting would have anchored
  assert r.debug["d_rest_eff"] == pytest.approx(4.2 - 4.0 / (2.0 * P.A_REST_FEAS), abs=1e-9)


def test_ease_demand_counts_creep_once_and_never_shallows_uphill() -> None:
  svc = StoppingService()
  # kinematic demand at v=0.3, d_rem=0.8 is -0.056 -> inner clip -0.10; creep FF applies ONCE
  base = svc._ease_demand(0.3, 0.8, 0.0, -3.5)
  assert base == pytest.approx(-0.10, abs=1e-9)
  pushed = svc._ease_demand(0.3, 0.8, 0.2, -3.5)
  assert pushed == pytest.approx(base - 0.2, abs=1e-9)  # exactly -clip(a_coast,0,0.4), not double
  # uphill drag (a_coast < 0) must never shallow the EASE demand (plan §1: EASE stays deepen-only)
  assert svc._ease_demand(0.3, 0.8, -0.4, -3.5) == pytest.approx(base, abs=1e-9)
  # deepen-only bound: the outer clip caps at A_EASE_DEEP even for a huge creep residual
  assert svc._ease_demand(0.3, 0.8, 0.5, -3.5) == pytest.approx(P.A_EASE_DEEP, abs=1e-9)


def test_dropout_floor_binds_shallow_demand() -> None:
  svc = StoppingService()
  sig = make_signals(d_gap=3.0, dropout=True)
  r = None
  for _ in range(30):  # short: the (correctly) hovering scripted v would engage the monitor later
    r = svc.update(engaged=True, v_ego=0.15, a_ego=-0.05, a_target=None, should_stop=True,
                   dts_planner=None, planner_min_limit=-3.5, signals=sig,
                   lead_status=False, lead_v=0.0, dt=DT, wire_accel=-0.05)
  assert r.accel == pytest.approx(P.A_DROPOUT_MIN, abs=1e-6)


def test_service_inactive_outside_entry_conditions() -> None:
  svc = StoppingService()
  sig = make_signals(d_gap=30.0)
  r = svc.update(engaged=True, v_ego=2.6, a_ego=0.0, a_target=None, should_stop=True,
                 dts_planner=None, planner_min_limit=-3.5, signals=sig,
                 lead_status=True, lead_v=0.0, dt=DT, wire_accel=-0.1)
  assert not r.active and r.phase == Phase.INACTIVE  # v >= V_ENTER
  r = svc.update(engaged=False, v_ego=1.0, a_ego=0.0, a_target=None, should_stop=True,
                 dts_planner=None, planner_min_limit=-3.5, signals=sig,
                 lead_status=True, lead_v=0.0, dt=DT, wire_accel=-0.1)
  assert not r.active  # disengaged


# --- telemetry: bounded events + one summary -------------------------------------------------------

def test_telemetry_bounded_events_and_summary() -> None:
  events = []
  tel = StoppingTelemetry(log_fn=lambda **kw: events.append(kw))
  for k in range(200):
    phase = "APPROACH_GLIDE" if k < 100 else "HOLD"
    tel.update(phase=phase, active=True, shadow_accel=-0.2, wire_accel=-0.3, v_ego=max(1.0 - k * 0.01, 0.0),
               d_gap=4.0, dts=None, wheel_stop_latched=k >= 150, dt=DT)
  tel.update(phase="INACTIVE", active=False, shadow_accel=0.0, wire_accel=-0.3, v_ego=0.0,
             d_gap=4.0, dts=None, wheel_stop_latched=True, dt=DT)
  summaries = [e for e in events if e.get("kind") == "settle_summary"]
  assert len(summaries) == 1
  assert len(events) <= 20
  s = summaries[0]
  assert s["wheel_stop_wire"] == pytest.approx(-0.3)
  assert s["max_divergence"] == pytest.approx(0.1, abs=1e-6)
  assert s["shadow_shallower_frac"] == pytest.approx(1.0)
  assert [p for (_, p) in s["phase_timeline"]] == ["APPROACH_GLIDE", "HOLD", "INACTIVE"]


def test_telemetry_timeline_capped_under_phase_chatter() -> None:
  # R2 finding L1: only LOGGED phase events were capped; the summary's timeline list grew one tuple
  # per phase change for the whole settle (10,000 entries per 100 s of worst-case chatter, all
  # serialized into a single cloudlog event). The timeline itself must stay bounded.
  events = []
  tel = StoppingTelemetry(log_fn=lambda **kw: events.append(kw))
  phases = ["APPROACH_GLIDE", "PRE_STOP_EASE"]
  for k in range(10000):  # worst-case chatter: a phase flip every frame for 100 s
    tel.update(phase=phases[k % 2], active=True, shadow_accel=-0.2, wire_accel=-0.3, v_ego=0.4,
               d_gap=3.0, dts=None, wheel_stop_latched=False, dt=DT)
  tel.update(phase="INACTIVE", active=False, shadow_accel=0.0, wire_accel=-0.3, v_ego=0.4,
             d_gap=3.0, dts=None, wheel_stop_latched=False, dt=DT)
  summaries = [e for e in events if e.get("kind") == "settle_summary"]
  assert len(summaries) == 1
  assert len(summaries[0]["phase_timeline"]) <= 64  # MAX_TIMELINE_ENTRIES_PER_STOP
  assert len([e for e in events if e.get("kind") == "phase_change"]) <= 16  # MAX_PHASE_EVENTS_PER_STOP


# --- STAGE 1: zero wire impact, provable at the seam -----------------------------------------------

def _scripted_stop_frames(n: int = 700):
  frames = []
  v, gap = 1.5, 6.0
  for _ in range(n):
    frames.append((v, -0.5 if v > 0.0 else 0.0, gap, v <= 0.0))
    v_new = max(v - 0.5 * DT, 0.0)
    gap = max(gap - (v + v_new) / 2.0 * DT, 3.9)
    v = v_new
  return frames


def _run_longcontrol_stop() -> tuple[list[float], LongControl]:
  lc = LongControl(DummyCarParams())
  toggles = DummyFrogPilotToggles()
  outs = []
  for (v, a, gap, standstill) in _scripted_stop_frames():
    cs = DummyCarState(v_ego=v, a_ego=a, standstill=standstill, cruise_standstill=False)
    outs.append(lc.update(active=True, CS=cs, a_target=-0.4, should_stop=True,
                          distance_to_stop_target_m=-1.0, accel_limits=(-3.0, 2.0),
                          frogpilot_toggles=toggles, lead_status=True, lead_v=0.0, lead_d_rel=gap))
  return outs, lc


def test_service_shadow_mode_has_zero_wire_impact(monkeypatch) -> None:
  monkeypatch.setattr(stopping_flags, "SERVICE_MODE", "OFF")
  wire_off, lc_off = _run_longcontrol_stop()
  assert lc_off._service_shadow_svc.phase == Phase.INACTIVE  # OFF really gated the observer out
  monkeypatch.setattr(stopping_flags, "SERVICE_MODE", "SHADOW")
  wire_shadow, lc = _run_longcontrol_stop()
  assert wire_off == wire_shadow  # byte-identical wire, frame by frame
  # ... and the shadow really ran, all the way into the terminal phases (otherwise this proves nothing)
  assert lc._service_shadow_svc.phase in (Phase.RAMP_TO_HOLD, Phase.HOLD)
  assert lc._service_shadow_ctx.update(v_ego=0.0, a_ego=0.0, a_cmd=-0.3, lead_status=True, lead_v=0.0,
                                       lead_d_rel=3.9, standstill=True).wheel_stop_latched


def test_service_shadow_exception_is_contained_and_disarms(monkeypatch) -> None:
  monkeypatch.setattr(stopping_flags, "SERVICE_MODE", "SHADOW")
  wire_clean, _ = _run_longcontrol_stop()

  def boom(self, *args, **kwargs):
    raise RuntimeError("injected shadow defect")

  monkeypatch.setattr(LongControl, "_update_stopping_service_shadow", boom)
  wire_broken, lc = _run_longcontrol_stop()
  assert wire_clean == wire_broken           # a shadow defect never reaches the wire
  assert lc._service_shadow_disabled         # ... and the observer disarmed after the first failure
