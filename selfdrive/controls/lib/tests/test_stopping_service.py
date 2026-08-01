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

# the terminal-smoothness scorer is the cycle-19 felt-quality gate (user directive: "make sure we
# can detect it's still wrong"); it lives with the review battery so route reviews and these
# fixtures score stops IDENTICALLY. tools/ is not a package -> load by path.
import importlib.util as _ilu
import pathlib as _pl
_sm_path = _pl.Path(__file__).resolve().parents[4] / "tools" / "stopping" / "review" / "terminal_smoothness.py"
_sm_spec = _ilu.spec_from_file_location("terminal_smoothness", _sm_path)
terminal_smoothness = _ilu.module_from_spec(_sm_spec)
_sm_spec.loader.exec_module(terminal_smoothness)
score_terminal = terminal_smoothness.score_terminal
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
  safety: list = field(default_factory=list)       # safety_binding per frame (scorer mask)


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
    tr.safety.append(bool(res.debug.get("safety_binding", False)))
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


def make_signals(d_gap=None, a_coast=0.0, wheel=False, latch=False, dropout=False,
                 gap_source=None, hold_outward=False) -> StopSignals:
  # NOTE (codex xhigh): a dropout in the REAL StopContext emits gap_source="decay" with
  # lead_status False (stop_context._update_gap), never "measured" -- a fixture that pairs
  # dropout=True with "measured" tests a signal combination the context cannot produce. Callers
  # exercising dropout should pass gap_source="decay" (or "held" for the lead-present variant).
  if gap_source is None:
    gap_source = "measured" if d_gap is not None else "none"
  # cycle-20: hold_outward marks the CONSERVATIVE hold (outward persistence, emits a lower bound);
  # inward-rejection and invalid-reading holds default False and may never relieve a demand.
  return StopSignals(d_gap=d_gap, gap_source=gap_source, gap_hold_outward=hold_outward,
                     dropout_active=dropout, a_coast=a_coast, wheel_stop_latched=wheel,
                     lead_confirmed_stopped=latch)


# --- nominal stop (plan §6 stage 0 + release-rate audit companion) --------------------------------

def test_nominal_stop_from_2p4_at_gap_12() -> None:
  tr = simulate(v0=2.4, gap0=12.0, should_stop=False, seed_u=0.0)  # entry via the lead latch
  k_roll = last_rolling_idx(tr)
  k_stop = k_roll + 1
  assert_no_slam(tr)
  # wheel-stop wire AT the terminal creep-hold floor (cycle-18, route 00001f6e): the old
  # [-0.35, -0.05] felt band sits below the HEV clutch's engagement push (+0.43 below 0.08 m/s)
  # and produced mid-stop relaunches; the felt metric is NET carry (bob = 0.0047 + 0.0138*carry),
  # and -0.70 wire against the engaged clutch is net ~0.27, inside the human clean band.
  assert tr.u[k_roll] == pytest.approx(P.A_HOLD_SECURE, abs=0.05), f"wheel-stop wire {tr.u[k_roll]:.3f}"
  # one continuous motion: v never re-rises before the stop, none after
  for k in range(1, k_stop):
    assert tr.v[k] <= max(tr.v[:k]) + 0.02
  assert max(tr.v[k_stop:]) < 0.01
  # hold: -0.30 or deeper within 0.7 s of the physical stop
  k_hold = k_stop + int(round(0.7 / DT))
  assert any(u <= -0.30 + EPS for u in tr.u[k_stop:k_hold + 1]), "hold not reached within 0.7 s"
  assert tr.u[-1] == pytest.approx(P.A_HOLD_SECURE, abs=0.02)
  # rest in the HEALTHY range (user rule 2026-08-01: "we still have to aim for something. 4-5
  # should be our healthy range. 3-4 for comfort and 5-6 is ok but probably shouldn't happen
  # because we can always use that for a comfortable 4-5 stop"). An UNCONSTRAINED nominal
  # approach must therefore land in 4-5, not merely inside the 3.0-5.0 band: dipping into 3-4 is
  # the comfort allowance for hot/close entries (see test_hot_arrival_*), and anything above 5
  # wasted room that a comfortable 4-5 stop could have used.
  assert 3.9 <= tr.gap[-1] <= 5.0, f"rest gap {tr.gap[-1]:.2f} outside the healthy 4-5 range"
  assert min(g for g in tr.gap if g is not None) >= 2.0


def test_crank1_arrival_holds_natural_value_not_ease_deep() -> None:
  # SMOOTHNESS CRANK #1 (cycle-7) RE-AIMED BY THE CLUTCH MECHANISM (cycle-18): the original gate
  # pinned wheel-stop wire in [-0.30, -0.05]. Route 00001f6e proved that band sits BELOW the
  # clutch engagement push (static +0.43 below 0.08 m/s, relaunch ~0.7): 3 of 6 stops
  # re-accelerated 0.24-0.38 m/s against active braking and were rescued by the monitor ladder.
  # New contract: arrive AT the terminal creep-hold floor (not deeper -- no slam past secure),
  # and pressure never goes up-then-down after the stop.
  tr = simulate(v0=2.4, gap0=12.0, should_stop=False, seed_u=0.0)
  k_roll = last_rolling_idx(tr)
  assert tr.u[k_roll] == pytest.approx(P.A_HOLD_SECURE, abs=0.05), f"wheel-stop wire {tr.u[k_roll]:.3f}"
  assert min(tr.u) >= P.A_HOLD_SECURE - EPS, "wire overshot past the secure level"
  assert tr.u[-1] == pytest.approx(P.A_HOLD_SECURE, abs=0.02)


def test_terminal_creep_hold_prevents_seg22_relaunch() -> None:
  # Route 00001f6e seg22 replay plant: the clutch push rises to +0.43 below 0.10 m/s, and any
  # positive net acceleration invokes the measured ~0.7 relaunch push until braking beats it.
  # Mutation check: setting TERMINAL_CREEP_HOLD_FLOOR=False makes the old EASE unload to ~-0.11;
  # this fixture then fails the no-relaunch assertion with a >0.2 m/s velocity rise.
  svc = StoppingService()
  v, gap, a_act, a_meas, last_u = 0.8, 5.0, 0.0, 0.0, -0.10
  below_020_v = None
  max_v_after_020 = 0.0
  latch_dwell = 0.0
  latch_net_decel = None
  monitor_armed = False
  for _ in range(2000):
    latch_dwell = latch_dwell + DT if v <= 0.06 else 0.0
    wheel_stop = latch_dwell >= 0.25
    sig = make_signals(d_gap=gap, wheel=wheel_stop, latch=True)
    r = svc.update(engaged=True, v_ego=v, a_ego=a_meas, a_target=None, should_stop=True,
                   dts_planner=None, planner_min_limit=-3.5, signals=sig,
                   lead_status=True, lead_v=0.0, dt=DT, wire_accel=last_u)
    u = r.accel
    monitor_armed |= bool(r.debug.get("monitor_active", False))

    a_act += (u - a_act) * DT / (0.25 + DT)
    engage = min(max((0.10 - v) / 0.02, 0.0), 1.0)
    static_push = 0.43 * engage
    push = 0.70 if a_meas > 0.0 else static_push
    a_net = a_act + push
    if wheel_stop and latch_net_decel is None:
      latch_net_decel = -a_net
    v_new = max(v + a_net * DT, 0.0)
    a_meas = (v_new - v) / DT
    gap -= max((v + v_new) / 2.0 * DT, 0.0)
    v = v_new
    last_u = u

    if below_020_v is None and v < 0.20:
      below_020_v = v
    if below_020_v is not None:
      max_v_after_020 = max(max_v_after_020, v)
    if wheel_stop and v == 0.0:
      break

  assert below_020_v is not None and max_v_after_020 - below_020_v <= 0.02, (
    f"terminal relaunch raised v by {max_v_after_020 - (below_020_v or 0.0):.3f} m/s")
  assert not monitor_armed, "anti-hover monitor armed instead of the terminal floor preventing the relaunch"
  assert latch_net_decel is not None and 0.10 <= latch_net_decel <= 0.45, (
    f"net decel at latch {latch_net_decel}")


def test_conditioned_lead_plan_depth_is_geometry_bounded() -> None:
  # With a trustworthy conditioned lead, relative-speed kinematics to the existing minimum
  # rest distance bound redundant planner depth. The nominal phase law still targets 4 m.
  svc = StoppingService()
  svc.phase = Phase.APPROACH_GLIDE
  svc._last_cmd = -0.20
  svc._d_rest_eff = 4.0
  svc._d_rest_calc_gap = 6.0
  sig = make_signals(d_gap=6.0, latch=True)
  r = svc.update(engaged=True, v_ego=0.7, a_ego=-0.5, a_target=-0.8, should_stop=True,
                 dts_planner=1.8, planner_min_limit=-3.5, signals=sig,
                 lead_status=True, lead_v=0.0, dt=DT, wire_accel=-0.20,
                 a_target_trajectory=-0.05)
  expected_floor = -(0.7 ** 2) / (2.0 * (6.0 - P.D_REST_CLIP_MIN))
  assert r.debug["plan_position_bounded"]
  assert r.debug["a_plan_raw"] == pytest.approx(-0.8)
  assert r.debug["a_plan"] == pytest.approx(expected_floor)
  assert r.accel > -0.8  # raw model depth did not become the wire

  held = StopSignals(d_gap=6.0, gap_source="held", gap_hold_outward=True, dropout_active=False, a_coast=0.0,
                     wheel_stop_latched=False, lead_confirmed_stopped=False)
  a_plan, bounded = svc._planner_safety_demand(-0.8, -0.05, 0.7, 0.0, held, True, 0.0, False)
  assert bounded
  assert a_plan == pytest.approx(expected_floor)  # conditioned-gap handoff does not chatter authority


def test_conditioned_lead_never_shallows_trajectory_demand() -> None:
  # The interface split is the architectural safety boundary: only direct/composite excess can be
  # position-bounded. A deeper constraint-resolved trajectory demand remains authoritative.
  svc = StoppingService()
  signals = make_signals(d_gap=6.0, latch=True)
  a_plan, bounded = svc._planner_safety_demand(-0.8, -0.55, 0.7, 0.0, signals, True, 0.0, False)
  assert bounded
  assert a_plan == pytest.approx(-0.55)

  same_plan, same_bounded = svc._planner_safety_demand(-0.55, -0.55, 0.7, 0.0, signals, True, 0.0, False)
  assert not same_bounded
  assert same_plan == pytest.approx(-0.55)  # ACC/trajectory-owned input is an identity


def test_missing_trajectory_split_fails_deep() -> None:
  # Mixed-version or direct callers without the split keep the old raw demand; absence can never
  # turn into a brake release.
  svc = StoppingService()
  signals = make_signals(d_gap=6.0, latch=True)
  a_plan, bounded = svc._planner_safety_demand(-0.8, None, 0.7, 0.0, signals, True, 0.0, False)
  assert not bounded
  assert a_plan == pytest.approx(-0.8)


@pytest.mark.parametrize("signals,lead_status", [
  (make_signals(d_gap=6.0, latch=True, dropout=True), True),  # decay-held gap
  (make_signals(), False),                                   # stop-line/no-lead
])
def test_plan_depth_remains_raw_without_trustworthy_lead_gap(signals, lead_status: bool) -> None:
  svc = StoppingService()
  svc.phase = Phase.APPROACH_GLIDE
  svc._last_cmd = -0.20
  svc._d_rest_eff = 4.0 if signals.d_gap is not None else None
  svc._d_rest_calc_gap = signals.d_gap
  r = svc.update(engaged=True, v_ego=0.7, a_ego=-0.5, a_target=-0.8, should_stop=True,
                 dts_planner=0.2, planner_min_limit=-3.5, signals=signals,
                 lead_status=lead_status, lead_v=0.0, dt=DT, wire_accel=-0.20,
                 a_target_trajectory=-0.20)
  assert not r.debug["plan_position_bounded"]
  assert r.debug["a_plan"] == pytest.approx(-0.8)
  assert r.accel == pytest.approx(-0.28)  # raw safety lane deepens at J_SAFE


def test_reversing_lead_deepens_generic_relative_speed_bound() -> None:
  # No reversing-lead branch is needed: relative closing speed automatically makes both the
  # 2.5 m planner bound and the 2.0 m hard-margin lane deeper.
  svc = StoppingService()
  svc.phase = Phase.APPROACH_GLIDE
  svc._last_cmd = -0.8
  svc._d_rest_eff = 4.0
  svc._d_rest_calc_gap = 8.3
  sig = make_signals(d_gap=8.3, latch=True)
  r = svc.update(engaged=True, v_ego=1.9, a_ego=-0.8, a_target=-0.9, should_stop=False,
                 dts_planner=3.5, planner_min_limit=-3.5, signals=sig,
                 lead_status=True, lead_v=-0.2, dt=DT, wire_accel=-0.8,
                 a_target_trajectory=-0.05)
  assert r.debug["plan_position_bounded"]
  expected_floor = -((1.9 - (-0.2)) ** 2) / (2.0 * (8.3 - P.D_REST_CLIP_MIN))
  assert r.debug["a_plan"] == pytest.approx(expected_floor)
  assert r.debug["a_kin"] > r.debug["a_plan"]  # 2.5 m bound brakes earlier than the 2 m hard lane


def test_new_model_shallow_arrival_builds_hold_before_roll_escape() -> None:
  # Route 00001e65: wheel-stop latched at ~0.09 m/s, -0.31 was held for 1 s, then
  # the car re-rolled 14 cm and the monitor had to arrest at -1.0. CYCLE-11 REVISION
  # (route 00001f10 pin law): flat 0.04 readings with a flat trusted gap ARE a secure
  # stop after the 0.25 s dwell -- waiting out the full fixed 0.5 s grace there served
  # no feel purpose and left exactly this re-roll window open. The arrival value is
  # preserved through the dwell, then the stationary pin build starts immediately.
  svc = StoppingService()
  svc.phase = Phase.RAMP_TO_HOLD
  svc._last_cmd = -0.31
  svc._d_rest_eff = 4.0
  svc._d_rest_calc_gap = 5.5
  sig = make_signals(d_gap=5.5, wheel=True, latch=True)
  cmds = []
  r = None
  for _ in range(70):
    r = svc.update(engaged=True, v_ego=0.04, a_ego=0.0, a_target=-0.5, should_stop=True,
                   dts_planner=1.2, planner_min_limit=-3.5, signals=sig,
                   lead_status=True, lead_v=0.0, dt=DT, wire_accel=-0.31)
    cmds.append(r.accel)
  def k(t: float) -> int:
    return int(round(t / DT)) - 1
  assert cmds[k(0.20)] == pytest.approx(-0.31)  # arrival preserved while the dwell accumulates
  assert cmds[k(0.50)] <= -0.44, "pin build must start at secure evidence, not the fixed grace end"
  assert min(cmds) >= P.A_HOLD_SECURE - EPS
  assert not r.debug["monitor_active"]  # proactive hold build, not a fast arrest


def test_arrest_floor_freezes_across_a_gap_dropout() -> None:
  # SOL ADVERSARIAL REVIEW: during an untrusted gap the crawl deficit is UNAVAILABLE, so `not crawl`
  # is silence rather than proof of stillness -- and on retrust the crawl reference re-bases, erasing
  # movement hidden by the dropout. Their probe unwound a -1.00 floor across a 9 s dropout and then
  # accepted a gap 0.30 m closer without re-deepening. The unwind must require affirmative trusted
  # stationary evidence and otherwise freeze.
  svc = StoppingService()
  svc.phase = Phase.HOLD
  svc._last_cmd = -1.00
  svc._mon_triggered = True
  svc._mon_floor = -1.00
  svc._d_rest_eff = 4.0
  svc._d_rest_calc_gap = 3.69
  for _ in range(900):  # 9 s parked, but the gap is untrusted the whole time
    svc.update(engaged=True, v_ego=0.0, a_ego=0.0, a_target=-0.05, should_stop=True,
               dts_planner=0.05, planner_min_limit=-3.5,
               signals=make_signals(d_gap=3.69, wheel=True, latch=True, dropout=True,
                                    gap_source="decay"),  # what StopContext really emits
               lead_status=False, lead_v=0.0, dt=DT, wire_accel=svc._last_cmd)
  assert svc._mon_floor == pytest.approx(-1.00, abs=1e-6), (
    f"floor unwound to {svc._mon_floor:.2f} on evidence that was never available")


def test_dropout_banked_dwell_does_not_cash_in_on_retrust() -> None:
  # SOL REVIEW round 2, finding 2: freezing the counter was not enough. Dwell banked BEFORE a blind
  # interval survived it, so the first retrusted frame resumed unwinding immediately -- their probe
  # banked 1.99 s, went blind for 9 s while the gap closed 0.30 m, then reached -0.70 within 0.5 s of
  # reacquisition. Reacquisition must earn a FRESH continuous dwell.
  svc = StoppingService()
  svc.phase = Phase.HOLD
  svc._last_cmd = -1.00
  svc._mon_triggered = True
  svc._mon_floor = -1.00
  svc._d_rest_eff = 4.0
  svc._d_rest_calc_gap = 3.69
  def step(gap, dropout):
    # dropout emits gap_source="decay" in the real StopContext, not "measured" (codex xhigh)
    svc.update(engaged=True, v_ego=0.0, a_ego=0.0, a_target=-0.05, should_stop=True,
               dts_planner=0.05, planner_min_limit=-3.5,
               signals=make_signals(d_gap=gap, wheel=True, latch=True, dropout=dropout,
                                    gap_source="decay" if dropout else "measured"),
               lead_status=not dropout, lead_v=0.0, dt=DT, wire_accel=svc._last_cmd)
  for _ in range(599):        # bank 5.99 s of trusted dwell -- just under the requirement
    step(3.69, False)
  assert svc._mon_floor <= -1.00 + EPS
  for _ in range(900):        # 9 s blind while the gap silently closes 0.30 m
    step(3.39, True)
  step(3.39, False)           # first retrusted frame
  assert svc._mon_floor <= -1.00 + EPS, "banked dwell cashed in on retrust"
  for _ in range(100):        # 1 s of fresh trusted dwell: still short of a full window
    step(3.39, False)
  assert svc._mon_floor <= -1.00 + EPS, "unwound on less than a fresh full dwell"


def test_outward_then_inward_walk_cannot_launder_the_dwell() -> None:
  # CODEX XHIGH attack: a lead that drifts outward and then returns must not be able to "bank" room to
  # move inward for free. MEASURED HONESTLY (mutation matrix): a 0.30 m return is already caught by the
  # PRE-EXISTING 0.15 m crawl lane, not by the new unwind-local baseline -- this fixture stays green
  # under removal of the baseline, of the unwind, and of the dropout gate. It is therefore a COMPOSITE
  # guard (it fails only if both the crawl lane and the baseline regress), not a baseline guard; the
  # baseline itself is isolated by the 0.021 m/s crawl fixture above. Kept deliberately, labelled
  # accurately, because a test that silently guards nothing is worse than one that says what it does.
  svc = StoppingService()
  svc.phase = Phase.HOLD
  svc._last_cmd = -0.85
  svc._mon_triggered = True
  svc._mon_floor = -0.85
  svc._d_rest_eff = 4.0
  svc._d_rest_calc_gap = 3.69
  # Timed to isolate the baseline: park just SHORT of a full dwell at the outward extreme, then
  # return inward. With the baseline the inward move restarts the dwell, so the following second
  # cannot unwind. Without it the banked dwell simply completes and the floor relaxes immediately.
  floors = []
  for i in range(790):
    gap = 3.99 if i < 590 else 3.69           # 5.9 s out (dwell not yet earned), then 0.30 m inward
    svc.update(engaged=True, v_ego=0.0, a_ego=0.0, a_target=-0.05, should_stop=True,
               dts_planner=0.05, planner_min_limit=-3.5,
               signals=make_signals(d_gap=gap, wheel=True, latch=True),
               lead_status=True, lead_v=0.0, dt=DT, wire_accel=svc._last_cmd)
    floors.append(svc._mon_floor)
  assert max(floors[590:]) <= -0.85 + EPS, (
    f"inward return laundered past the baseline (floor {max(floors[590:]):.2f})")


def test_over_escalated_arrest_floor_unwinds_to_the_secure_hold() -> None:
  # ROUTE 00001f44 seg3 (cycle-13): the car consumed 0.25 m of gap, so the displacement lane
  # legitimately ratcheted the floor to -1.00 -- and because the floor clears only on RELEASE, that
  # -1.00 then stayed on the wire for the ENTIRE 27.4 s standstill (it ended only on driver gas).
  # Once genuinely stopped, depth beyond A_HOLD_SECURE does no work and must be given back.
  svc = StoppingService()
  svc.phase = Phase.HOLD
  svc._last_cmd = -1.00
  svc._mon_triggered = True
  svc._mon_floor = -1.00
  svc._d_rest_eff = 4.0
  svc._d_rest_calc_gap = 3.69
  cmds = []
  for _ in range(1400):  # 14 s parked (dwell is 6 s)
    r = svc.update(engaged=True, v_ego=0.0, a_ego=0.0, a_target=-0.05, should_stop=True,
                   dts_planner=0.05, planner_min_limit=-3.5,
                   signals=make_signals(d_gap=3.69, wheel=True, latch=True),
                   lead_status=True, lead_v=0.0, dt=DT, wire_accel=svc._last_cmd)
    cmds.append(r.accel)
  # the dwell is respected before anything moves...
  assert cmds[int(5.0 / DT)] == pytest.approx(-1.00, abs=1e-6), "unwound before the dwell elapsed"
  # ...then it returns to exactly the secure hold and stops there -- never shallower
  assert svc._mon_floor == pytest.approx(P.A_HOLD_SECURE, abs=1e-6)
  assert cmds[-1] == pytest.approx(P.A_HOLD_SECURE, abs=0.02)
  assert min(cmds) >= -1.00 - EPS and max(cmds) <= P.A_HOLD_SECURE + EPS


def test_unwound_floor_re_arms_if_the_car_moves_again() -> None:
  # The unwind is only safe because the reactive lanes stay live. After giving depth back, a fresh
  # escape must re-arm and deepen at the safety rate from wherever the floor has reached.
  svc = StoppingService()
  svc.phase = Phase.HOLD
  svc._last_cmd = -1.00
  svc._mon_triggered = True
  svc._mon_floor = -1.00
  svc._d_rest_eff = 4.0
  svc._d_rest_calc_gap = 3.69
  for _ in range(1400):  # park long enough to fully unwind
    svc.update(engaged=True, v_ego=0.0, a_ego=0.0, a_target=-0.05, should_stop=True,
               dts_planner=0.05, planner_min_limit=-3.5,
               signals=make_signals(d_gap=3.69, wheel=True, latch=True),
               lead_status=True, lead_v=0.0, dt=DT, wire_accel=svc._last_cmd)
  assert svc._mon_floor == pytest.approx(P.A_HOLD_SECURE, abs=1e-6)
  floor_before = svc._mon_floor
  for _ in range(60):  # a fresh roll: readings rise past the post-latch escape bar
    svc.update(engaged=True, v_ego=0.12, a_ego=0.15, a_target=-0.05, should_stop=True,
               dts_planner=0.05, planner_min_limit=-3.5,
               signals=make_signals(d_gap=3.5, wheel=True, latch=True),
               lead_status=True, lead_v=0.0, dt=DT, wire_accel=svc._last_cmd)
  assert svc._mon_floor <= floor_before - P.MON_ESCALATE_STEP + EPS, (
    f"floor did not re-arm on a fresh escape ({svc._mon_floor:.2f} vs {floor_before:.2f})")


def test_crawl_faster_than_the_dwell_resets_the_unwind() -> None:
  # CODEX XHIGH found my previous version of this test worthless: it shrank the gap 0.0005 m per
  # frame, which only reaches the 0.15 m `crawl` deficit after ~3 s, so the floor had ALREADY
  # unwound to -0.70 by t=2.25 s -- and the test only inspected the final value at 6 s, by which
  # point the ladder had re-escalated. It passed while missing exactly the transient it claimed to
  # forbid. The unwind now carries its own displacement baseline with a data-derived slack
  # (MON_UNWIND_GAP_SLACK_M, from the measured one-quantum parked jitter), so a crawl that moves
  # more than that inside the dwell keeps resetting it. This asserts the TRANSIENT.
  svc = StoppingService()
  svc.phase = Phase.HOLD
  svc._last_cmd = -0.85
  svc._mon_triggered = True
  svc._mon_floor = -0.85
  svc._d_rest_eff = 4.0
  svc._d_rest_calc_gap = 3.69
  svc.ev.latch_gap = 3.69
  gap = 3.69
  floors = []
  # 0.021 m/s is chosen to ISOLATE the new baseline (codex xhigh): faster crawls reach the
  # pre-existing 0.15 m `crawl` threshold before the 6 s dwell even elapses, so they would keep this
  # test green even if the unwind-local baseline were deleted. At 0.021 m/s the deficit is only
  # ~0.13 m at 6 s, so ONLY the local baseline can hold the floor here.
  for _ in range(800):  # 8 s of crawl at 0.021 m/s -- just above slack/dwell = 0.020 m/s
    gap -= 0.021 * DT
    svc.update(engaged=True, v_ego=0.0, a_ego=0.0, a_target=-0.05, should_stop=True,
               dts_planner=0.05, planner_min_limit=-3.5,
               signals=make_signals(d_gap=gap, wheel=True, latch=True),
               lead_status=True, lead_v=0.0, dt=DT, wire_accel=svc._last_cmd)
    floors.append(svc._mon_floor)
  assert max(floors) <= -0.85 + EPS, (
    f"floor transiently unwound to {max(floors):.2f} while the car was crawling")


def test_parked_radar_jitter_still_permits_the_unwind() -> None:
  # The other side of that slack: one quantum of inward jitter is what a genuinely parked car
  # produces (measured across this corpus: n=200, p50 = p99 = max = 0.100 m), so it must NOT keep
  # resetting the dwell -- otherwise the unwind never fires in practice.
  svc = StoppingService()
  svc.phase = Phase.HOLD
  svc._last_cmd = -1.00
  svc._mon_triggered = True
  svc._mon_floor = -1.00
  svc._d_rest_eff = 4.0
  svc._d_rest_calc_gap = 3.69
  for i in range(1200):
    gap = 3.69 - (0.10 if (i // 40) % 2 else 0.0)  # oscillate by exactly one quantum
    svc.update(engaged=True, v_ego=0.0, a_ego=0.0, a_target=-0.05, should_stop=True,
               dts_planner=0.05, planner_min_limit=-3.5,
               signals=make_signals(d_gap=gap, wheel=True, latch=True),
               lead_status=True, lead_v=0.0, dt=DT, wire_accel=svc._last_cmd)
  assert svc._mon_floor == pytest.approx(P.A_HOLD_SECURE, abs=1e-6), (
    f"quantization jitter blocked the unwind entirely (floor {svc._mon_floor:.2f})")


def test_stop_and_go_moving_lead_entry_rests_at_nominal_not_close() -> None:
  # ROUTE 00001b76 seg4/5 REGRESSION (first stage-3 drive): ego at 3.2 m/s behind a lead
  # decelerating 2.05 -> 0; the service entered as the lead stopped (gap ~5.2 at ego ~1.6) and the
  # old comfort-glide anchor (A_GLIDE_NOM 0.5) re-zeroed the rest to ~2.7 m -> the car stopped at
  # 2.1 m. With the FIRM feasibility anchor (A_REST_FEAS) + re-anchoring while the lead still
  # moves, the same entry must rest near nominal.
  # entry inside the band (the harness has no legacy chain above V_ENTER): ego 2.3 m/s at gap 6.5
  # behind a lead still rolling at 1.5 and braking to a stop -- the incident's entry geometry.
  # Old anchor: 6.5 - 2.3^2/(2*0.5) = 1.2 -> pinned at the (then 2.4 m) rest floor (the fault).
  # Firm anchor: 6.5 - 2.3^2/(2*1.2) = 4.3 -> nominal 4.0.
  def lead_v_fn(t: float) -> float:
    return max(1.5 - 1.0 * t, 0.0)

  tr = simulate(v0=2.3, gap0=6.5, lead_v_fn=lead_v_fn, should_stop=True, seed_u=-0.9, t_max=30.0)
  assert_no_slam(tr)
  first_stop_idx(tr)
  rests = [r for r in tr.d_rest_eff if r is not None]
  assert rests[-1] >= 3.9, f"rest anchor re-zeroed to {rests[-1]:.2f} on a normal stop-and-go entry"
  assert tr.gap[-1] >= 3.4, f"rested at {tr.gap[-1]:.2f} m -- the 00001b76 too-close class"
  assert min(g for g in tr.gap if g is not None) >= 3.0  # band retune 2026-07-20: 3.0 m floor


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
  # plan §5 worst-case geometry: ego ~0.5 m/s in EASE right at the 3.1 m gap-gate boundary
  # (band retune 2026-07-20: EASE_GAP_MIN 2.6 -> 3.1 with the 3.0 m rest floor)
  t_rev = 0.2

  def lead_v_fn(t: float) -> float:
    return -0.5 if t_rev <= t < t_rev + 0.6 else 0.0

  tr = simulate(v0=0.5, gap0=3.25, should_stop=True, seed_u=-0.25, lead_v_fn=lead_v_fn, t_max=15.0)
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
  # closure arithmetic (plan §5): gap consumed from reversal onset to full-authority command <= 0.24 m.
  # At the 3.25 m boundary geometry the binding depth is the collapsed-d_rem glide law (~-0.83 at
  # v 0.5) jerk-limited while v falls -- the wire peaks ~-0.74; -0.70 marks genuine full authority
  deep_k = next((k for k in range(k_rev, len(tr.u)) if tr.u[k] <= -0.70), None)
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


def test_far_model_stop_shallow_arrival_pin_beats_creep() -> None:
  # ROUTE 00001f10 seg 10 (cycle-11) recorded-state probe: the model landed a stop far out (gap
  # 10.9) with its own arrival easing, so RAMP entered at wire -0.134. v then sat 0.03-0.05
  # (quantized) with a FLAT trusted gap for ~1.2 s -- genuinely stopped by every dither-immune
  # evidence rule -- while the J_HOLD 0.6 build crawled toward depth. Through the ~0.2 s actuator
  # lag the Stribeck creep peak (+0.43 as v -> 0) beat the still-shallow pressure at 625.31:
  # 0.2 m nudge (DISLIKE2), reactive -0.85/-1.0 arrest, ledger wound to -3.5. Contract: preserve
  # the early natural-arrival hold, but once GENUINELY stopped the wire must reach plant pin depth
  # -(a_coast + Stribeck margin) fast -- a stationary pressure build is felt-free -- so the creep
  # never escapes and the reactive lanes stay quiet.
  svc = StoppingService()
  svc.phase = Phase.RAMP_TO_HOLD
  svc._last_cmd = -0.134
  svc._d_rest_eff = 4.0
  svc._d_rest_calc_gap = 10.9
  sig = make_signals(d_gap=10.9, a_coast=0.30, wheel=True, latch=True)
  cmds = []
  r = None
  for _ in range(120):  # 2.4 s at DT
    r = svc.update(engaged=True, v_ego=0.04, a_ego=0.0, a_target=-0.15, should_stop=True,
                   dts_planner=None, planner_min_limit=-3.5, signals=sig,
                   lead_status=True, lead_v=0.0, dt=DT, wire_accel=-0.134)
    cmds.append(r.accel)
  def k(t: float) -> int:
    return int(round(t / DT)) - 1
  assert cmds[k(0.20)] == pytest.approx(-0.134), "natural-arrival hold must survive the first stationary frames"
  assert cmds[k(0.90)] <= -0.55, f"wire {cmds[k(0.90)]:.2f} at 0.9 s -- the pin lost the race the creep won at 1.2 s"
  assert min(cmds) >= P.A_HOLD_SECURE - EPS, "pin must never exceed the secure hold depth"
  assert all(cmds[i] - cmds[i + 1] <= P.J_SAFE * DT + EPS for i in range(len(cmds) - 1))
  assert not r.debug["monitor_active"], "the pin must be predictive, not a reactive arrest"


def test_finish_roll_fast_deepens_without_arming_the_ratchet() -> None:
  # Red-team finding 8 (cycle-11 ledger refactor): the two post-latch roll channels are DISTINCT and
  # must stay distinct. A rise crossing the finish-roll bar (+0.02 above the latch minimum) but
  # staying below the monitor's arm gates (v >= 0.05 and rise >= MON_RISE) must end the arrival
  # grace and deepen at the safety rate WITHOUT arming the persistent ratchet floor; a later rise
  # through the monitor gates must then arm the deep floor exactly once.
  svc = StoppingService()
  sig = make_signals(d_gap=4.2, wheel=True, latch=True)

  def step(v: float):
    return svc.update(engaged=True, v_ego=v, a_ego=0.0, a_target=-0.20, should_stop=True,
                      dts_planner=None, planner_min_limit=-3.5, signals=sig,
                      lead_status=True, lead_v=0.0, dt=DT, wire_accel=-0.15)

  r = step(0.04)                      # entry + wheel latch on the first frame, inside entry grace
  assert r.phase == Phase.RAMP_TO_HOLD
  for _ in range(2):                  # dip to 0.02: the latch-immediate minimum re-seeds to 0.02
    r = step(0.02)
  base = r.accel
  assert base >= -0.25, "the natural-arrival grace should still be holding the wire shallow here"
  for _ in range(int(0.12 / DT)):     # rise to 0.045: finish-roll (+0.025) fires, monitor gates
    r = step(0.045)                   # (v >= 0.05, rise >= MON_RISE) do NOT
  assert r.accel <= base - 0.25, "finish-roll evidence must fast-deepen past the J_HOLD build"
  assert not r.debug["monitor_active"], "sub-gate finish roll must NOT arm the ratchet"
  for _ in range(int(0.4 / DT)):      # settle again past the entry grace window
    r = step(0.04)
  assert not r.debug["monitor_active"]
  for _ in range(int(0.2 / DT)):      # rise through the monitor gates: rolling escape, arms once
    r = step(0.17)
  assert r.debug["monitor_active"], "a genuine post-latch escape must arm the deep floor"
  assert r.debug["a_monitor"] <= P.A_HOLD - P.MON_POSTSTOP_ARREST_EXTRA + EPS


def test_same_frame_retrust_rebases_before_deficit_evaluation() -> None:
  # Red-team finding 8, second half: when gap trust returns on the SAME frame as a reading far
  # below the stale crawl reference (dropout ended on a different radar notch), the reference must
  # re-base before the deficit is compared -- an apparent 0.30 m deficit on the retrust frame is
  # measurement discontinuity, not a crawl. A genuine post-rebase crawl must still be caught.
  svc = StoppingService()

  def step(sig):
    return svc.update(engaged=True, v_ego=0.04, a_ego=0.0, a_target=-0.20, should_stop=True,
                      dts_planner=None, planner_min_limit=-3.5, signals=sig,
                      lead_status=True, lead_v=0.0, dt=DT, wire_accel=-0.15)

  r = step(make_signals(d_gap=4.0, wheel=True, latch=True))   # entry + latch: crawl reference 4.0
  assert r.phase == Phase.RAMP_TO_HOLD
  for _ in range(20):
    r = step(make_signals(d_gap=4.0, wheel=True, latch=True))
  for _ in range(30):                                          # dropout stretch: trust lost
    r = step(make_signals(d_gap=4.0, wheel=True, latch=True, dropout=True))
  r = step(make_signals(d_gap=3.70, wheel=True, latch=True))   # retrust frame, reading 0.30 lower
  assert not r.debug["monitor_active"], "retrust-frame discontinuity must re-base, never arrest"
  for _ in range(20):
    r = step(make_signals(d_gap=3.70, wheel=True, latch=True))
  assert not r.debug["monitor_active"]
  for gap in (3.62, 3.58, 3.53, 3.50, 3.48):                   # genuine crawl from the NEW reference
    for _ in range(4):
      r = step(make_signals(d_gap=gap, wheel=True, latch=True))
  assert r.debug["monitor_active"], "the displacement lane must still catch a real post-rebase crawl"


def test_monitor_floor_ratchet_survives_ease_to_glide_flip() -> None:
  # R1 finding 2: an armed anti-creep floor must NOT release when the gap closes through the EASE
  # gap gate (3.1 since the 2026-07-20 band retune) and
  # the phase flips PRE_STOP_EASE -> APPROACH_GLIDE -- exactly while the gap is closing. The
  # ratchet clears only on RELEASE (a genuine go) or INACTIVE, never on EASE<->GLIDE flips.
  svc = StoppingService()
  r = None
  for _ in range(300):  # hover in EASE at v = 0.40, gap 3.20 (> 3.1): monitor triggers + escalates
    r = svc.update(engaged=True, v_ego=0.40, a_ego=0.0, a_target=None, should_stop=True,
                   dts_planner=None, planner_min_limit=-3.5, signals=make_signals(d_gap=3.20),
                   lead_status=True, lead_v=0.0, dt=DT, wire_accel=-0.2)
  assert r.phase == Phase.PRE_STOP_EASE
  assert r.debug["monitor_active"], "hover in EASE never armed the monitor"
  floor = r.accel
  assert floor <= -0.80  # 3 s of hover: -0.35 initial + escalations (R1's probe ratcheted to -1.10)
  for i in range(100):  # gap crosses 3.1 -> EASE gap gate fails -> phase flips to APPROACH_GLIDE
    r = svc.update(engaged=True, v_ego=0.40, a_ego=0.0, a_target=None, should_stop=True,
                   dts_planner=None, planner_min_limit=-3.5, signals=make_signals(d_gap=3.05),
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
  gap = 3.5
  r = None
  for k in range(100):  # enough for multiple monitor windows; terminal gap remains inside 5 m
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
  assert travel_after_stop <= 0.08, f"hold escape traveled {travel_after_stop:.3f} m (>8 cm)"  # roll-only trigger (cycle-5): +~1 cm vs hover, no false arrests
  assert min(floors) <= P.A_HOLD - P.MON_POSTSTOP_ARREST_EXTRA + 0.05  # fast arrest floor engaged
  assert v < 0.02  # rest is final against the sustained push


def test_poststop_kalman_dither_never_arms_the_arrest() -> None:
  # CYCLE-5 REGRESSION (routes 00001b8f..00001ba3): after a clean stop the Kalman vEgo dithers at
  # 0.03-0.05 while the car is physically stopped; the hover test read that as an escape and armed
  # the -0.70 fast-arrest on nearly EVERY stop (holds observed -0.70..-1.15) = the felt grab.
  # Post-latch, dither must NEVER arm the monitor; the hold rests at A_HOLD exactly.
  ctx, svc = StopContext(), StoppingService()
  v, a_act, gap, last_u = 1.5, 0.0, 8.0, -0.4
  tau = 0.2
  dither = [0.0, 0.03, 0.04, 0.03, 0.0, 0.04]
  k_dither = 0
  physically_stopped = False
  cmds_at_rest = []
  for _i in range(3500):
    if physically_stopped:
      v_meas = dither[k_dither % len(dither)]
      k_dither += 1
    else:
      v_meas = v
    signals = ctx.update(v_ego=v_meas, a_ego=0.0, a_cmd=last_u, lead_status=True, lead_v=0.0,
                         lead_d_rel=gap, standstill=physically_stopped, dt=DT)
    r = svc.update(engaged=True, v_ego=v_meas, a_ego=0.0, a_target=None, should_stop=True,
                   dts_planner=None, planner_min_limit=-3.5, signals=signals,
                   lead_status=True, lead_v=0.0, dt=DT, wire_accel=last_u)
    u = r.accel if r.active else 0.0
    a_act += (u - a_act) * DT / (tau + DT)
    if not physically_stopped:
      v = max(v + a_act * DT, 0.0)
      gap -= max(v * DT, 0.0)
      if v < 0.005:
        physically_stopped = True
    else:
      cmds_at_rest.append(u)
      assert not r.debug.get("monitor_active", False), "dither armed the post-stop monitor"
    last_u = u
  assert physically_stopped
  assert cmds_at_rest[-1] == pytest.approx(P.A_HOLD_SECURE, abs=0.02), f"hold rested at {cmds_at_rest[-1]:.2f}, not A_HOLD_SECURE"
  # no post-rest deepening beyond the inherited arrival value (a false arrest would dive to -0.70):
  arrival = cmds_at_rest[0]
  assert min(cmds_at_rest) >= min(arrival, P.A_HOLD_SECURE) - 0.02, "post-stop command deepened past the secure hold (false arrest)"


def test_floor_cap_relieves_the_f0c_slam_without_moving_the_anchor() -> None:
  # ROUTE 00001f0c seg0 (cycle-10 bookmark): at the RECORDED state (gap 4.50, v 0.6186, a_coast
  # +0.24, anchor 4.3) d_rem collapsed to 0.2 m and the glide demanded -1.16..-1.27 (jerk 11.3).
  # CYCLE-15 CONTRACT REWRITE (called out in the plan review): the anchor-move relief is replaced
  # by the stateless floor-defense cap. The anchor now NEVER moves (stays 4.3); the demand is
  # capped at max(lag-aware floor decel, A_GLIDE_NOM + hyst = 0.65, v^2/1.2) = 0.65 here, giving
  # a_phase = -(0.65 + coast 0.24) = -0.89 -- between the old relief's -0.74 and the -1.27 slam.
  # The 0.65 continuity term is load-bearing: without it, ordinary firm arrivals (demand 0.65-0.9)
  # would be relieved to the floor law broadly and erode nominal rests (the cycle-10 concern).
  # CYCLE-20 BOUNDARY SHARPENED (route 00001f80 seg99): the cap now also applies on a "held"
  # frame -- the gap filter's ego-propagated hold is a LOWER BOUND on the true gap, so defending
  # the floor with it is conservative, and requiring "measured" turned the cap off inside the
  # filter's 0.25 s outward-persistence window, which is exactly where that route's -1.93 spike
  # lived. A DROPOUT "decay" gap is invented, not measured, and is still refused: that is the
  # contrast case below (the raw anchor demand returns), and it is the cap's mutation pin.
  for gap_source, dropout, want_deep in (("decay", True, True), ("held", False, False),
                                         ("measured", False, False)):
    svc = StoppingService()
    svc.phase = Phase.APPROACH_GLIDE
    svc._d_rest_eff = 4.3
    svc._d_rest_calc_gap = 4.9
    svc._last_cmd = -0.70
    sig = StopSignals(d_gap=4.50, gap_source=gap_source, gap_hold_outward=(gap_source == "held"),
                      dropout_active=dropout, a_coast=0.24,
                      wheel_stop_latched=False, lead_confirmed_stopped=True)
    r = svc.update(engaged=True, v_ego=0.6186, a_ego=-0.5, a_target=-0.68, should_stop=True,
                   dts_planner=0.398, planner_min_limit=-3.5, signals=sig,
                   lead_status=bool(gap_source != "decay"), lead_v=0.0, dt=DT, wire_accel=-0.70,
                   increased_stopped_distance=0.3, a_target_trajectory=-0.25)
    assert r.debug["d_rest_eff"] == pytest.approx(4.3), "the anchor must never move"
    if want_deep:
      assert r.debug["a_phase"] < -1.1, (  # invented gap: no relief, the raw anchor demand stands
        f"{gap_source}: cap relieved an invented gap ({r.debug['a_phase']:.2f})")
    else:
      assert -0.95 < r.debug["a_phase"] < -0.80, f"{gap_source}: cap missed ({r.debug['a_phase']:.2f})"


def test_hot_arrival_glide_uses_comfort_not_entry_feasibility() -> None:
  # 00001efe seg10: entry reachability still uses A_REST_FEAS, but once the remaining distance
  # collapses at walking speed the terminal anti-blowup uses A_GLIDE_NOM as its COMFORT target.
  # Re-anchoring is admitted only when that comfort landing remains inside the intended >=2.5 m
  # rest band; tighter/hotter cases retain the firm target and the hard safety lanes.
  svc = StoppingService()
  svc._d_rest_eff = 4.0
  svc._d_rest_calc_gap = 4.0
  # CYCLE-15: the cap replaces the anchor-move. With a trusted gap, the effective remaining is
  # v^2/(2*cap) with cap = max(lag floor 0.436, 0.65, v^2/1.2 = 0.385) = 0.65; the anchor itself
  # never moves. Untrusted gap: raw (negative) remaining -- the cap requires displacement evidence.
  remaining = svc._d_rem(3.8, None, 0.68, gap_live=True)
  assert remaining == pytest.approx(0.68 ** 2 / (2.0 * (P.A_GLIDE_NOM + P.A_REANCHOR_HYST)))
  assert svc._d_rest_eff == pytest.approx(4.0), "the anchor must never move"
  assert svc._d_rem(3.8, None, 0.68, gap_live=False) == pytest.approx(3.8 - 4.0)
  assert svc._glide_demand(0.68, remaining, 0.0, -3.5) == pytest.approx(-(P.A_GLIDE_NOM + P.A_REANCHOR_HYST))

  # The older hot/close incident remains safety-owned and does not acquire a sub-band comfort
  # target merely to satisfy the nominal deceleration ceiling. Band retune 2026-07-20: with the
  # 3.0 m floor this arrival now rests IN-BAND (~3.45, was allowed down to 2.35) at the cost of a
  # firm wheel-stop wire -- hot arrivals into the floor stay firm by design (EASE never shallows
  # them); the felt-band wire contract applies to nominal-rest stops only.
  tr = simulate(v0=2.1, gap0=5.3, should_stop=True, seed_u=-1.05, t_max=25.0)
  assert_no_slam(tr)
  k_roll = last_rolling_idx(tr)
  assert tr.u[k_roll] <= -0.03, f"wheel-stop wire {tr.u[k_roll]:.2f} released early"
  assert tr.gap[-1] >= 3.0, f"rest {tr.gap[-1]:.2f} below the band floor"
  assert min(g for g in tr.gap if g is not None) >= 3.0


def test_reanchor_boundary_sweep_respects_band_floor_without_discontinuity() -> None:
  # SOL ADVERSARIAL REVIEW (band retune f9735655b1): with relief admitted at comfort_landing ==
  # exactly the 3.0 clip-min, gap 3.5 @ v 0.7071 re-anchored to 3.000 and the plant overshoot
  # rested 2.84 -- THROUGH the floor -- while v 0.7072 rejected relief and rested 3.04 at a much
  # firmer peak (-0.75 vs -1.17: a 0.0001 m/s cliff). REANCHOR_LANDING_MARGIN_M (0.25) closes
  # both: the admitted branch targets >= 3.25 (overshoot budget), and the whole boundary band
  # now takes the same firm branch -- the cliff is gone (0.7071 and 0.7072 rest within mm).
  rests = {}
  for v0 in (0.55, 0.65, 0.7071, 0.7072, 0.75):
    tr = simulate(v0=v0, gap0=3.5, should_stop=True, seed_u=-0.4, t_max=25.0)
    assert_no_slam(tr)
    rests[v0] = tr.gap[-1]
    assert tr.gap[-1] >= 3.0, f"v0={v0}: rest {tr.gap[-1]:.3f} through the 3.0 m floor"
  assert abs(rests[0.7071] - rests[0.7072]) < 0.02, "re-anchor admission cliff is back"
  # genuinely hot close entries physically overshoot the floor by ~0.15 m (deepen-only cannot
  # reclaim it; pre-retune this class rested ~2.4) -- bounded, no slam, D_HARD untouched
  for v0 in (0.85, 1.0):
    tr = simulate(v0=v0, gap0=3.5, should_stop=True, seed_u=-0.4, t_max=25.0)
    assert_no_slam(tr)
    assert tr.gap[-1] >= 2.85, f"v0={v0}: rest {tr.gap[-1]:.3f}"


def test_relief_entry_ramps_gently_and_safety_still_bypasses() -> None:
  # ROUTE 00001f62 seg25 (cycle-17, TWO user bookmarks): creeping at a flat 0.63 m/s, gap 4.6,
  # planner asking only -0.25, the wire stepped -0.39 -> -1.10 in 0.28 s at J_DOWN because late
  # EASE entry (Doppler noise under the EASE gate) left GLIDE approaching the relief-cap depth
  # (0.65 + creep ~0.45) from a shallow wire. The felt jolt IS that derivative. The gentle entry
  # rate spreads the same approved depth over ~0.71 s. Numbers pinned from the recorded state.
  svc = StoppingService()
  svc.phase = Phase.APPROACH_GLIDE
  svc._last_cmd = -0.39
  svc._d_rest_eff = 4.30
  svc._d_rest_calc_gap = 4.90
  sig = make_signals(d_gap=4.60, a_coast=0.45, latch=True)
  cmds = []
  for _ in range(80):  # 0.8 s
    r = svc.update(engaged=True, v_ego=0.63, a_ego=-0.05, a_target=-0.25, should_stop=True,
                   dts_planner=0.05, planner_min_limit=-3.5, signals=sig,
                   lead_status=True, lead_v=0.0, dt=DT, wire_accel=-0.39)
    cmds.append(r.accel)
  def k(t: float) -> int:
    return int(round(t / DT)) - 1
  # ordinary deepening never exceeds the gentle rate
  steps = [cmds[i] - cmds[i + 1] for i in range(len(cmds) - 1)]
  assert max(steps) <= P.J_RELIEF_ENTRY * DT + EPS, f"jolt step {max(steps):.4f}"
  # the recorded jolt profile is replaced: -0.85 +/- 0.03 at 0.45 s (was -1.077 at J_DOWN)...
  assert cmds[k(0.45)] == pytest.approx(-0.85, abs=0.03), f"0.45 s wire {cmds[k(0.45)]:.3f}"
  # ...and the full approved depth still arrives by 0.75 s (an inert/too-slow rate fails here)
  assert cmds[k(0.75)] <= -1.08, f"depth by 0.75 s only {cmds[k(0.75)]:.3f}"
  # SAFETY BYPASS: a binding deeper lane (a_plan via a deep trajectory-confirmed demand) must get
  # J_SAFE immediately, not the comfort rate
  r = svc.update(engaged=True, v_ego=0.63, a_ego=-0.05, a_target=-2.0, should_stop=True,
                 dts_planner=0.05, planner_min_limit=-3.5, signals=sig,
                 lead_status=True, lead_v=0.0, dt=DT, wire_accel=cmds[-1],
                 a_target_trajectory=-2.0)
  assert cmds[-1] - r.accel >= P.J_SAFE * DT - 0.02, (
    f"safety lane got the comfort rate: step {cmds[-1] - r.accel:.4f}")


def _relief_midramp_svc(v: float = 0.63, seed: float = -0.39, n: int = 30):
  # the bookmark-start state of test_relief_entry_ramps_gently... advanced n frames INTO the
  # gentle ramp (sol cycle-17 end-review: the hazard cases must begin mid-ramp in APPROACH_GLIDE,
  # not at entry -- the existing reversing test enters via PRE_STOP_EASE and misses this window)
  svc = StoppingService()
  svc.phase = Phase.APPROACH_GLIDE
  svc._last_cmd = seed
  svc._d_rest_eff = 4.30
  svc._d_rest_calc_gap = 4.90
  sig = make_signals(d_gap=4.60, a_coast=0.45, latch=True)
  r = None
  for _ in range(n):
    r = svc.update(engaged=True, v_ego=v, a_ego=-0.05, a_target=-0.25, should_stop=True,
                   dts_planner=0.05, planner_min_limit=-3.5, signals=sig,
                   lead_status=True, lead_v=0.0, dt=DT, wire_accel=seed)
  assert r is not None and r.debug["phase"] == "APPROACH_GLIDE"
  target = min(r.debug["a_phase"], r.debug["a_kin"], r.debug["a_plan"], r.debug["a_monitor"])
  assert target <= r.accel - 3 * P.J_SAFE * DT, "fixture inert: no catch-up depth left mid-ramp"
  return svc, r.accel


def test_relief_gentle_midramp_reversal_recovers_at_j_safe() -> None:
  # sol cycle-17 end-review (HIGH), transition 1/4: a lead GENUINELY reversing mid-gentle-ramp.
  # Genuine = the cycle-15 noise-hardened un-confirm has fired (lead_confirmed_stopped False
  # after T_LEAD_NEG_OFF_S of sustained negative Doppler) -- modeled by latch=False + lv=-0.5.
  # Pre-fix trace: gentle=true, safety_binding=false (a_phase -1.10 deeper than a_kin -0.25),
  # 0.01/frame. Required: J_SAFE recovery at once, deficit erased.
  svc, cmd0 = _relief_midramp_svc()
  sig = make_signals(d_gap=4.55, a_coast=0.45, latch=False)
  r = svc.update(engaged=True, v_ego=0.63, a_ego=-0.05, a_target=-0.25, should_stop=True,
                 dts_planner=0.05, planner_min_limit=-3.5, signals=sig,
                 lead_status=True, lead_v=-0.5, dt=DT, wire_accel=cmd0)
  assert cmd0 - r.accel >= P.J_SAFE * DT - EPS, (
    f"reversal kept a comfort rate: step {cmd0 - r.accel:.4f}")  # J_DOWN 0.025 / gentle 0.01 fail
  for _ in range(12):  # deficit fully erased well inside 0.13 s
    r = svc.update(engaged=True, v_ego=0.63, a_ego=-0.05, a_target=-0.25, should_stop=True,
                   dts_planner=0.05, planner_min_limit=-3.5, signals=sig,
                   lead_status=True, lead_v=-0.5, dt=DT, wire_accel=r.accel)
  assert r.accel <= -1.08, f"deficit not erased under reversal: wire {r.accel:.3f}"


def test_relief_gentle_midramp_monitor_arming_recovers_at_j_safe() -> None:
  # Transition 2/4: the hover monitor arming in an EXISTING gentle GLIDE ramp. Real mechanism, no
  # hand-set monitor state: v held at 0.45 (<= V_EASE so detection runs; the hover IS the fault
  # this monitor exists for) with lv=-0.15 Doppler noise (fails the raw EASE gate, so the phase
  # stays GLIDE -- the recorded bookmark condition -- while latch=True keeps the noise-hardened
  # reversal disqualifier correctly silent). The monitor floor (-0.35) is SHALLOWER than a_phase,
  # so safety_binding never asserts: pre-fix the ramp stayed at J_RELIEF_ENTRY through arming.
  svc = StoppingService()
  svc.phase = Phase.APPROACH_GLIDE
  svc._last_cmd = -0.05
  svc._d_rest_eff = 4.30
  svc._d_rest_calc_gap = 4.90
  sig = make_signals(d_gap=4.60, a_coast=0.45, latch=True)
  prev = -0.05
  armed_step = None
  for k in range(160):  # 1.6 s: entry grace 0.5 + MON_WINDOW_S 0.4 put arming ~0.9-1.0 s
    r = svc.update(engaged=True, v_ego=0.45, a_ego=0.0, a_target=-0.25, should_stop=True,
                   dts_planner=0.05, planner_min_limit=-3.5, signals=sig,
                   lead_status=True, lead_v=-0.15, dt=DT, wire_accel=prev)
    assert r.debug["phase"] == "APPROACH_GLIDE", f"left GLIDE at frame {k}"
    if r.debug["monitor_active"] and armed_step is None:
      armed_step = prev - r.accel
      assert prev - r.accel > 3 * P.J_RELIEF_ENTRY * DT, "fixture inert: armed with no depth left"
    prev = r.accel
  assert armed_step is not None, "monitor never armed: hover fixture broken"
  assert armed_step >= P.J_SAFE * DT - EPS, (
    f"monitor arming kept a comfort rate: step {armed_step:.4f}")


def test_relief_gentle_midramp_gap_collapse_recovers_at_j_safe() -> None:
  # Transition 3/4: an accepted inward gap collapse to 2.9 m (the real filter admits a persisted
  # collapse at full authority; 2.9 breaches the 3.0 m floor => lag-floor violation). Pre-fix the
  # flag cleared but recovery ran at J_DOWN, keeping the gentle ramp's accumulated deficit.
  svc, cmd0 = _relief_midramp_svc()
  sig = make_signals(d_gap=2.90, a_coast=0.45, latch=True)
  r = svc.update(engaged=True, v_ego=0.63, a_ego=-0.05, a_target=-0.25, should_stop=True,
                 dts_planner=0.05, planner_min_limit=-3.5, signals=sig,
                 lead_status=True, lead_v=0.0, dt=DT, wire_accel=cmd0)
  assert cmd0 - r.accel >= P.J_SAFE * DT - EPS, (
    f"gap collapse recovered at J_DOWN or slower: step {cmd0 - r.accel:.4f}")


def test_relief_gentle_midramp_dropout_recovers_at_j_safe() -> None:
  # Transition 4/4: lead dropout mid-gentle-ramp (decay-hold: the glide must KEEP braking, D2-H3,
  # and the deficit must not survive the blind interval). gap_source="decay" + lead_status=False
  # is the only combination the real StopContext emits here.
  svc, cmd0 = _relief_midramp_svc()
  sig = make_signals(d_gap=4.30, a_coast=0.45, latch=True, dropout=True, gap_source="decay")
  r = svc.update(engaged=True, v_ego=0.63, a_ego=-0.05, a_target=-0.25, should_stop=True,
                 dts_planner=0.05, planner_min_limit=-3.5, signals=sig,
                 lead_status=False, lead_v=0.0, dt=DT, wire_accel=cmd0)
  assert r.debug["phase"] == "APPROACH_GLIDE", "dropout must not exit the stop (D2-H3)"
  assert cmd0 - r.accel >= P.J_SAFE * DT - EPS, (
    f"dropout recovered at J_DOWN or slower: step {cmd0 - r.accel:.4f}")


def test_relief_gentle_track_churn_chatter_is_stable_until_caught_up() -> None:
  # sol round-2: a hazard FLICKERING at a gate boundary must neither oscillate gentle<->J_SAFE nor
  # release the catch-up early. REAL StopContext (per sol): radar churn loses the stopped lead 2
  # frames in every 8; loss frames emit decay/dropout (hazard reads true -> J_SAFE), reacquired
  # frames emit trusted measured (ego-closing-consistent inward raw -> immediate accept) where the
  # gentle gates pass again -- the exact chatter that must NOT resume the gentle rate mid-catch-up.
  ctx = StopContext()
  svc = StoppingService()
  svc.phase = Phase.APPROACH_GLIDE
  svc._last_cmd = -0.39
  svc._d_rest_eff = 4.30
  svc._d_rest_calc_gap = 4.90
  sig = None
  for _k in range(60):  # warm the context: gap measured, lead confirmed stopped
    sig = ctx.update(v_ego=0.63, a_ego=-0.05, a_cmd=-0.5, lead_status=True, lead_v=0.0,
                     lead_d_rel=4.60, lead_track_id=7, standstill=False, dt=DT)
  assert sig.gap_source == "measured" and sig.lead_confirmed_stopped
  prev = None
  rows = []  # (step_into_frame, dropout_active, accel)
  for k in range(110):
    lost = k >= 30 and (k - 30) % 8 < 2
    raw = 4.60 - 0.0063 * k  # ego closes at v*dt: reacquisition reads ego-closing-consistent
    sig = ctx.update(v_ego=0.63, a_ego=-0.05, a_cmd=prev if prev is not None else -0.4,
                     lead_status=not lost, lead_v=0.0, lead_d_rel=None if lost else raw,
                     lead_track_id=None if lost else 7, standstill=False, dt=DT)
    r = svc.update(engaged=True, v_ego=0.63, a_ego=-0.05, a_target=-0.25, should_stop=True,
                   dts_planner=0.05, planner_min_limit=-3.5, signals=sig,
                   lead_status=not lost, lead_v=0.0, dt=DT, wire_accel=prev)
    assert r.debug["phase"] == "APPROACH_GLIDE", f"left GLIDE at frame {k}"
    d = r.debug
    tgt = min(d["a_phase"], d["a_kin"], d["a_plan"], d["a_monitor"])
    if prev is not None:
      rows.append((prev - r.accel, sig.dropout_active, r.accel, tgt))
    prev = r.accel
  pre = [s for s, _, _, _ in rows[:29]]
  assert max(pre) <= P.J_RELIEF_ENTRY * DT + EPS, "clean gentle ramp broken before the churn"
  first_hazard = next(i for i, (_, d, _, _) in enumerate(rows) if d)
  # the ungentled-target snapshot for this state is ~-0.92 (the measured-frame arbitration
  # target the gentle ramp was descending to -- NOT the transient -1.6 dropout decay demand)
  caught = next(i for i, (_, _, a, _) in enumerate(rows) if i >= first_hazard and a <= -0.915)
  window = rows[first_hazard:caught + 1]
  # 1) the first hazard frame erases deficit at J_SAFE at once
  assert window[0][0] >= P.J_SAFE * DT - EPS, f"churn hazard frame stepped {window[0][0]:.4f}"
  # 2) NO gentle-rate frame inside the catch-up window: reacquired trusted frames (where the
  #    gentle gates pass again) must hold >= J_DOWN until the deficit is erased (the final frame
  #    is the clamped partial step that LANDS on the snapshot, so it is exempt)
  assert min(s for s, _, _, _ in window[:-1]) >= P.J_DOWN * DT - EPS, (
    f"gentle rate resumed mid-catch-up: min step {min(s for s, _, _, _ in window[:-1]):.4f}")
  # 3) the deficit is erased promptly despite the chatter (<= 0.15 s from the first hazard frame)
  assert caught - first_hazard <= 15, f"catch-up took {caught - first_hazard} frames"
  # 4) NO PUMP after catch-up (the churn-trace regression this test exists for): later churn
  #    cycles must never J_SAFE-slam again -- there is no deficit left -- and the wire must not
  #    ratchet below the CONCURRENT measured-frame demand by more than the ordinary 2-frame
  #    J_DOWN chase of the transient decay law (the pre-cycle-17 churn sawtooth, ~0.05)
  post = rows[caught + 1:]
  assert max(s for s, _, _, _ in post) <= P.J_DOWN * DT + EPS, (
    f"churn re-slammed after catch-up: step {max(s for s, _, _, _ in post):.4f}")
  last_measured_tgt = window[-1][3]
  for _s, drop_, a_, t_ in post:
    if not drop_:
      last_measured_tgt = t_
    assert a_ >= last_measured_tgt - 0.09, (
      f"churn ratcheted the wire to {a_:.3f} vs measured demand {last_measured_tgt:.3f}")


def test_relief_gentle_doppler_chatter_after_unconfirm_is_stable() -> None:
  # sol round-2, second chatter mode: post-unconfirm Doppler chatter. The lead has genuinely
  # begun reversing (un-confirm fired -> latch False) but raw Doppler chatters across the -0.1
  # boundary. The sticky latch must hold the catch-up through the benign-reading frames (J_DOWN,
  # never gentle) and slam only while the hazard reads true (J_SAFE).
  svc, cmd0 = _relief_midramp_svc()
  prev = cmd0
  rows = []
  for k in range(30):
    lv = -0.15 if (k // 3) % 2 == 0 else 0.02  # 3-frame chatter across the EASE_LEAD_V_MIN gate
    sig = make_signals(d_gap=4.55, a_coast=0.45, latch=False)
    r = svc.update(engaged=True, v_ego=0.63, a_ego=-0.05, a_target=-0.25, should_stop=True,
                   dts_planner=0.05, planner_min_limit=-3.5, signals=sig,
                   lead_status=True, lead_v=lv, dt=DT, wire_accel=prev)
    rows.append((prev - r.accel, lv, r.accel))
    prev = r.accel
  caught = next(i for i, (_, _, a) in enumerate(rows) if a <= -1.05)
  window = rows[:caught + 1]
  assert window[0][0] >= P.J_SAFE * DT - EPS, "reversal hazard frame did not slam"
  assert min(s for s, _, _ in window) >= P.J_DOWN * DT - EPS, (
    "gentle rate resumed between hazard reads")
  assert caught <= 20, f"catch-up took {caught} frames under Doppler chatter"


def test_relief_snapshot_immune_to_transient_safety_frame_then_hazard() -> None:
  # sol round-3 (medium): a ONE-FRAME trajectory-confirmed deep a_plan (-2.0) while the gentle
  # flag is true runs under the J_SAFE bypass -- it must NOT be captured as the ungentled-target
  # snapshot. Pre-fix trace: snapshot stored -2.0 while the wire moved one J_SAFE step; a
  # following reversal latched catch-up against the vanished -2.0, the wire parked at the real
  # -1.10 target, and the latch never cleared -- gentle disabled and J_DOWN for the rest of the
  # approach (the comfort-rate regression re-created from the inside).
  svc, cmd0 = _relief_midramp_svc()
  sig = make_signals(d_gap=4.60, a_coast=0.45, latch=True)
  r = svc.update(engaged=True, v_ego=0.63, a_ego=-0.05, a_target=-2.0, should_stop=True,
                 dts_planner=0.05, planner_min_limit=-3.5, signals=sig,
                 lead_status=True, lead_v=0.0, dt=DT, wire_accel=cmd0, a_target_trajectory=-2.0)
  assert cmd0 - r.accel >= P.J_SAFE * DT - EPS, "safety bypass lost"
  assert svc._relief_gentle_target is not None and svc._relief_gentle_target > -1.2, (
    f"snapshot poisoned by the transient safety frame: {svc._relief_gentle_target:.2f}")
  # the safety demand vanishes; a genuine reversal hazard latches catch-up for 3 frames...
  sig_rev = make_signals(d_gap=4.55, a_coast=0.45, latch=False)
  for _ in range(3):
    r = svc.update(engaged=True, v_ego=0.63, a_ego=-0.05, a_target=-0.25, should_stop=True,
                   dts_planner=0.05, planner_min_limit=-3.5, signals=sig_rev,
                   lead_status=True, lead_v=-0.5, dt=DT, wire_accel=r.accel)
  assert svc._relief_catchup, "hazard never latched: fixture broken"
  # ...then the hazard passes. The latch must clear against a REACHABLE snapshot and gentle resume.
  sig_ok = make_signals(d_gap=4.55, a_coast=0.45, latch=True)
  for _ in range(40):
    r = svc.update(engaged=True, v_ego=0.63, a_ego=-0.05, a_target=-0.25, should_stop=True,
                   dts_planner=0.05, planner_min_limit=-3.5, signals=sig_ok,
                   lead_status=True, lead_v=0.0, dt=DT, wire_accel=r.accel)
  assert not svc._relief_catchup, "catch-up latched forever on an unreachable snapshot"
  assert svc._relief_entry_gentle, "gentle never resumed after the hazard passed"


def test_relief_catchup_releases_when_target_shallows_past_snapshot() -> None:
  # The second unreachable-snapshot path (found extending sol round-3): the snapshot (-1.10) was
  # legitimate, but after the hazard the lead is reacquired FARTHER out -- the target shallows and
  # the wire can never reach the snapshot again. A deepen-only catch-up cannot progress past the
  # current target, so the latch must release there instead of pinning J_DOWN/no-gentle forever.
  svc, cmd0 = _relief_midramp_svc()
  sig_drop = make_signals(d_gap=4.30, a_coast=0.45, latch=True, dropout=True, gap_source="decay")
  r = svc.update(engaged=True, v_ego=0.63, a_ego=-0.05, a_target=-0.25, should_stop=True,
                 dts_planner=0.05, planner_min_limit=-3.5, signals=sig_drop,
                 lead_status=False, lead_v=0.0, dt=DT, wire_accel=cmd0)
  assert svc._relief_catchup, "dropout never latched: fixture broken"
  sig_far = make_signals(d_gap=5.60, a_coast=0.45, latch=True)  # reacquired farther: target shallows
  for _ in range(15):
    r = svc.update(engaged=True, v_ego=0.63, a_ego=-0.05, a_target=-0.25, should_stop=True,
                   dts_planner=0.05, planner_min_limit=-3.5, signals=sig_far,
                   lead_status=True, lead_v=0.0, dt=DT, wire_accel=r.accel)
  assert not svc._relief_catchup, "latch held past a shallowed target it can never catch"


def test_relief_gentle_benign_region_exit_keeps_comfort_rates() -> None:
  # GUARD ON THE FIX ITSELF: a BENIGN invalidation -- the lead creeps forward and the remaining
  # grows past the 0.6 m region -- must NOT latch J_SAFE catch-up. The target shallows with the
  # relief; slamming here would reintroduce the exact radar-step-noise jolt cycle-17 removed.
  svc, cmd0 = _relief_midramp_svc()
  sig = make_signals(d_gap=5.40, a_coast=0.45, latch=True)  # outward: remaining 1.1 > 0.6
  prev = cmd0
  for _ in range(10):
    r = svc.update(engaged=True, v_ego=0.63, a_ego=-0.05, a_target=-0.25, should_stop=True,
                   dts_planner=0.05, planner_min_limit=-3.5, signals=sig,
                   lead_status=True, lead_v=0.0, dt=DT, wire_accel=prev)
    assert prev - r.accel <= P.J_DOWN * DT + EPS, (
      f"benign region exit was slammed: step {prev - r.accel:.4f}")
    prev = r.accel


def _smoothness_gate(tr, label: str) -> None:
  sc = score_terminal(tr.t, tr.v, tr.u, safety_mask=tr.safety)
  assert sc is not None, f"{label}: stop never entered the terminal window"
  assert not sc["relaunched"], f"{label}: the stop relaunched after its first sub-0.05 dip"
  assert sc["descent_count"] == 1, f"{label}: {sc['descent_count']} descents (human template = 1)"
  # gate calibrated to the human template (manual re-engagements 0.6-1.0 m/s3); genuine
  # curve-following reads ~0.5, the stacked law's J_DOWN steps 2.5, the f7b flap 6.8-10
  assert sc["wire_jerk_max"] <= 0.80 + EPS, f"{label}: wire jerk {sc['wire_jerk_max']}"
  assert sc["wire_pump"] <= 0.06 + EPS, f"{label}: pump {sc['wire_pump']}"



# route 00001f80 seg99, the user-bookmarked harsh stop, decimated to 20 Hz. Columns:
# (t rel. wheel-stop, vEgo, lead dRel, planner distanceToStopTarget, planner aTarget). The
# recorded wire went to -1.93 here while the planner asked -0.85: the planner's STOP LINE
# collapsed (dts 2.7 -> 0.05 m) while the car still rolled at 1.6 -> 0.9 m/s, and the terminal
# law converted that position target into v^2/(2*dts) = up to 2.6 m/s2. The car rested 3.8 m
# from the lead -- always fine under the user's band rule (3.0 floor, comfort in between).
F80_SEQ = [
  (-2.598, 2.248, 6.92, 2.67, -0.969),
  (-2.551, 2.194, 6.92, 2.67, -0.969),
  (-2.5, 2.151, 6.49, 2.41, -0.969),
  (-2.45, 2.113, 6.3, 2.16, -1.004),
  (-2.4, 2.069, 6.3, 2.16, -1.004),
  (-2.351, 2.027, 6.31, 1.94, -0.999),
  (-2.299, 1.979, 6.29, 1.99, -0.96),
  (-2.25, 1.933, 6.29, 1.99, -0.96),
  (-2.201, 1.887, 6.16, 1.84, -0.908),
  (-2.15, 1.845, 6.03, 1.84, -0.874),
  (-2.1, 1.8, 5.73, 1.71, -0.859),
  (-2.051, 1.758, 5.59, 1.41, -0.888),
  (-2.0, 1.721, 5.55, 1.28, -0.907),
  (-1.95, 1.688, 5.42, 1.25, -0.908),
  (-1.9, 1.653, 5.42, 1.25, -0.908),
  (-1.849, 1.62, 5.24, 1.0, -0.893),
  (-1.801, 1.585, 5.19, 0.93, -0.875),
  (-1.749, 1.546, 5.19, 0.93, -0.875),
  (-1.7, 1.5, 4.91, 0.72, -0.877),
  (-1.65, 1.453, 4.86, 0.61, -0.868),
  (-1.601, 1.397, 4.8, 0.55, -0.838),
  (-1.55, 1.333, 4.64, 0.5, -0.826),
  (-1.5, 1.273, 4.57, 0.34, -0.848),
  (-1.45, 1.214, 4.58, 0.27, -0.847),
  (-1.4, 1.147, 4.58, 0.27, -0.847),
  (-1.351, 1.086, 4.52, 0.25, -0.81),
  (-1.299, 1.021, 4.45, 0.22, -0.79),
  (-1.25, 0.959, 4.45, 0.22, -0.79),
  (-1.201, 0.895, 4.27, 0.05, -0.732),
  (-1.151, 0.825, 4.26, 0.01, -0.697),
  (-1.1, 0.75, 4.26, 0.01, -0.697),
  (-1.051, 0.678, 4.04, 0.05, -0.63),
  (-1.001, 0.605, 3.96, 0.05, -0.611),
  (-0.95, 0.534, 3.96, 0.05, -0.611),
  (-0.899, 0.464, 3.94, 0.05, -0.538),
  (-0.851, 0.4, 3.87, 0.05, -0.484),
  (-0.8, 0.336, 3.75, 0.05, -0.412),
  (-0.751, 0.291, 3.75, 0.05, -0.345),
  (-0.701, 0.233, 3.72, 0.05, -0.279),
  (-0.65, 0.194, 3.72, 0.05, -0.279),
  (-0.599, 0.161, 3.63, 0.05, -0.166),
  (-0.551, 0.129, 3.56, 0.05, -0.12),
  (-0.5, 0.105, 3.62, 0.05, -0.081),
  (-0.45, 0.09, 3.62, 0.05, -0.053),
  (-0.401, 0.079, 3.5, 0.05, -0.039),
  (-0.35, 0.075, 3.5, 0.05, -0.039),
  (-0.301, 0.072, 3.73, 0.05, -0.017),
]


def test_planner_stop_line_collapse_does_not_out_brake_the_planner() -> None:
  # CYCLE-20 (user: "three is the minimum safety ... in between we just aim for comfort"): no
  # position target inside the band -- neither the nominal anchor nor the planner's own stop
  # line -- may demand more deceleration than resting at the 3.0 m floor requires. Replayed from
  # the recorded columns above, driving the REAL StopContext.
  ctx, svc = StopContext(), StoppingService()
  wire = -0.95
  worst_excess = 0.0
  worst_v = 0.0
  for i, (_t, v, gap, dts, plan) in enumerate(F80_SEQ):
    dt = 0.05 if i == 0 else F80_SEQ[i][0] - F80_SEQ[i - 1][0]
    sig = ctx.update(v_ego=v, a_ego=-0.9, a_cmd=wire, lead_status=True, lead_v=0.0,
                     lead_d_rel=gap, lead_track_id=4242, standstill=False, dt=dt)
    r = svc.update(engaged=True, v_ego=v, a_ego=-0.9, a_target=plan, should_stop=True,
                   dts_planner=dts, planner_min_limit=-3.5, signals=sig,
                   lead_status=True, lead_v=0.0, dt=dt, wire_accel=wire)
    wire = r.accel
    if v > 0.55 and not r.debug.get("safety_binding", False):
      excess = plan - wire
      if excess > worst_excess:
        worst_excess, worst_v = excess, v
  # measured ABOVE the terminal descent band (v > 0.55): below it the wire is deliberately deeper
  # than the planner to hold against the clutch (cycle-18/19 contract, pinned separately). The
  # recorded defect lived at v 1.8-0.9 and reached 1.22 m/s2 of excess (wire -1.93 vs -0.71).
  assert worst_excess <= 0.45, f"out-braked the planner by {worst_excess:.2f} at v={worst_v:.2f}"
  assert wire >= P.A_HOLD_SECURE - 0.02, f"wire ran past the secure hold: {wire:.2f}"


def test_inward_held_gap_never_relieves_the_floor_defence() -> None:
  # CYCLE-20 END-REVIEW (HIGH): StopContext emits gap_source="held" for THREE provenances --
  # outward persistence (min(prediction, raw) = a lower bound, safe to relieve on), inward-step
  # REJECTION (the larger prediction, while the raw reading says the gap collapsed), and an
  # invalid reading with the lead present. The floor-defence cap relieves braking, so it may only
  # consume the first. The reviewer's trace: an accepted 4.1 m reading followed by a persistent
  # same-track correction to 3.565 m produced held gaps of 4.05/4.00; capping on those shallowed
  # a_phase to -0.68 against a real -1.30/-1.42 requirement and the plant rested at 2.993 m --
  # through the floor. Driven here through the REAL StopContext with actuator lag.
  ctx, svc = StopContext(), StoppingService()
  v, a_act, wire = 1.0, -0.5, -0.60
  gap = 4.10
  raw = 4.10
  min_gap = gap
  for k in range(900):
    if k == 60:
      raw = 3.565           # the physical gap was always this: a persistent inward correction
    sig = ctx.update(v_ego=v, a_ego=a_act, a_cmd=wire, lead_status=True, lead_v=0.0,
                     lead_d_rel=raw, lead_track_id=77, standstill=False, dt=DT)
    r = svc.update(engaged=True, v_ego=v, a_ego=a_act, a_target=-0.55, should_stop=True,
                   dts_planner=None, planner_min_limit=-3.5, signals=sig,
                   lead_status=True, lead_v=0.0, dt=DT, wire_accel=wire)
    wire = r.accel
    a_act += (wire - a_act) * DT / (0.20 + DT)      # repository plant lag
    v = max(v + a_act * DT, 0.0)
    raw = max(raw - v * DT, 0.0)
    gap = raw
    min_gap = min(min_gap, gap)
    if v <= 0.0 and k > 100:
      break
  assert min_gap >= 3.0 - EPS, f"inward-held relief drove the gap to {min_gap:.3f} m, through the floor"

  # ...and the DECISION itself, which is what makes this mutation-sensitive: on an inward-rejection
  # hold the cap must not relieve at all, so a_phase must stay at the uncapped anchor demand.
  # (Admitting every hold -- the reviewed defect -- relieves it to ~-0.9 and fails here.)
  svc2 = StoppingService()
  svc2.phase = Phase.APPROACH_GLIDE
  svc2._d_rest_eff = 4.3
  svc2._d_rest_calc_gap = 4.9
  svc2._last_cmd = -0.70
  sig_in = make_signals(d_gap=4.50, a_coast=0.24, latch=True, gap_source="held", hold_outward=False)
  r2 = svc2.update(engaged=True, v_ego=0.6186, a_ego=-0.5, a_target=-0.68, should_stop=True,
                   dts_planner=0.398, planner_min_limit=-3.5, signals=sig_in,
                   lead_status=True, lead_v=0.0, dt=DT, wire_accel=-0.70,
                   increased_stopped_distance=0.3, a_target_trajectory=-0.25)
  assert r2.debug["a_phase"] < -1.1, (
    f"inward-rejection hold was relieved to {r2.debug['a_phase']:.2f} -- optimistic geometry")


def test_inward_held_gap_never_shallows_planner_authority() -> None:
  # CYCLE-20 END-REVIEW ROUND 2 (HIGH): the planner-safety lane POSITION-BOUNDS (shallows) the
  # planner's own demand, so it needs the same provenance test as the floor-defence cap. The
  # reviewer's replay: accepted 4.1 m then a same-track correction to 3.525 m physical, planner
  # asking -1.30 with trajectory -0.25, 0.20 s lag -- trusting the inward hold bounded a_plan to
  # -0.456 and rested at 2.995 m; refusing it kept the direct demand and rested at 3.045 m.
  ctx, svc = StopContext(), StoppingService()
  v, a_act, wire = 1.0, -0.5, -0.60
  # geometry chosen so the floor is DEFENDABLE with full authority and lost without it: from
  # 1.0 m/s the direct -1.30 demand needs ~0.39 m (plus lag) while the shallowed -0.46 needs
  # ~1.09 m. A correction arriving later than this leaves a physically unwinnable stop, which
  # would test the plant rather than the provenance rule.
  phys = 3.90                          # the TRUE gap; the radar over-reads it until the correction
  min_gap = phys
  bounded_during_hold = []
  for k in range(1200):
    raw = 4.40 if k < 20 else phys     # stale over-read, then a persistent same-track correction
    sig = ctx.update(v_ego=v, a_ego=a_act, a_cmd=wire, lead_status=True, lead_v=0.0,
                     lead_d_rel=raw, lead_track_id=91, standstill=False, dt=DT)
    r = svc.update(engaged=True, v_ego=v, a_ego=a_act, a_target=-1.30, should_stop=True,
                   dts_planner=None, planner_min_limit=-3.5, signals=sig,
                   lead_status=True, lead_v=0.0, dt=DT, wire_accel=wire,
                   a_target_trajectory=-0.25)
    if sig.gap_source == "held" and not sig.gap_hold_outward:
      bounded_during_hold.append(r.debug["a_plan"])
    wire = r.accel
    a_act += (wire - a_act) * DT / (0.20 + DT)
    v = max(v + a_act * DT, 0.0)
    phys = max(phys - v * DT, 0.0)
    min_gap = min(min_gap, phys)
    if v <= 0.0 and k > 100:
      break
  assert bounded_during_hold, "fixture never exercised an inward-rejection hold"
  # the direct demand must survive the hold un-shallowed (admitting all holds bounds it to ~-0.46)
  assert min(bounded_during_hold) <= -1.29, (
    f"planner authority shallowed to {min(bounded_during_hold):.2f} on an optimistic held gap")
  assert min_gap >= 3.0 - EPS, f"rested at {min_gap:.3f} m, through the floor"


def test_optimistic_gap_cannot_release_the_hold() -> None:
  # CYCLE-20 R3: gap_grew reaches RELEASE via planner_go WITHOUT the strict observed_departure
  # predicate. Enter the hold at 4.0, let the reading mature outward to ~4.5 (accepted), then a
  # same-track raw collapse to 3.5 is REJECTED and the filter emits the larger prediction --
  # gap_grew (4.5 > 4.0 + 0.3) then reads "the lead departed" on a lead that actually came
  # CLOSER, and a positive planner frame released the hold with lead_departure_confirm_s zero.
  ctx, svc = StopContext(), StoppingService()
  wire = -0.70
  r = None
  for _ in range(150):                       # settle into HOLD with the lead at 4.0
    sig = ctx.update(v_ego=0.0, a_ego=0.0, a_cmd=wire, lead_status=True, lead_v=0.0,
                     lead_d_rel=4.0, lead_track_id=31, standstill=True, dt=DT)
    r = svc.update(engaged=True, v_ego=0.0, a_ego=0.0, a_target=-0.3, should_stop=True,
                   dts_planner=None, planner_min_limit=-3.5, signals=sig,
                   lead_status=True, lead_v=0.0, dt=DT, wire_accel=wire)
    wire = r.accel
  assert r.phase in (Phase.RAMP_TO_HOLD, Phase.HOLD), f"fixture never held: {r.phase}"
  for _ in range(200):                       # the reading matures outward to ~4.5 (accepted)
    sig = ctx.update(v_ego=0.0, a_ego=0.0, a_cmd=wire, lead_status=True, lead_v=0.0,
                     lead_d_rel=4.5, lead_track_id=31, standstill=True, dt=DT)
    r = svc.update(engaged=True, v_ego=0.0, a_ego=0.0, a_target=-0.3, should_stop=True,
                   dts_planner=None, planner_min_limit=-3.5, signals=sig,
                   lead_status=True, lead_v=0.0, dt=DT, wire_accel=wire)
    wire = r.accel
  assert sig.d_gap > 4.3 and r.phase in (Phase.RAMP_TO_HOLD, Phase.HOLD)
  sig = ctx.update(v_ego=0.0, a_ego=0.0, a_cmd=wire, lead_status=True, lead_v=0.0,
                   lead_d_rel=3.5, lead_track_id=31, standstill=True, dt=DT)   # collapse, rejected
  assert sig.gap_source == "held" and not sig.gap_hold_outward, "fixture did not produce the hold"
  r = svc.update(engaged=True, v_ego=0.0, a_ego=0.0, a_target=+0.3, should_stop=False,
                 dts_planner=None, planner_min_limit=-3.5, signals=sig,
                 lead_status=True, lead_v=0.0, dt=DT, wire_accel=wire)
  assert r.phase != Phase.RELEASE, "optimistic gap released the hold without departure evidence"


def test_outward_held_gap_still_relieves_the_blow_up() -> None:
  # the other half of the boundary: an OUTWARD-persistence hold is a lower bound, so the cap must
  # still relieve there (that hold is where route 00001f80's -1.93 spike lived). Mutation: if the
  # cap refuses all holds, this demand returns to the raw anchor blow-up.
  svc = StoppingService()
  svc.phase = Phase.APPROACH_GLIDE
  svc._d_rest_eff = 4.3
  svc._d_rest_calc_gap = 4.9
  svc._last_cmd = -0.70
  sig = make_signals(d_gap=4.50, a_coast=0.24, latch=True, gap_source="held", hold_outward=True)
  r = svc.update(engaged=True, v_ego=0.6186, a_ego=-0.5, a_target=-0.68, should_stop=True,
                 dts_planner=0.398, planner_min_limit=-3.5, signals=sig,
                 lead_status=True, lead_v=0.0, dt=DT, wire_accel=-0.70,
                 increased_stopped_distance=0.3, a_target_trajectory=-0.25)
  assert -0.95 < r.debug["a_phase"] < -0.80, f"outward hold not relieved: {r.debug['a_phase']:.2f}"


def test_terminal_descent_smoothness_gates() -> None:
  # CYCLE-19 GATE (route 00001f7b seg3, user: "very gentle then unnecessarily increased suddenly;
  # make sure we can detect it's still wrong"): the recorded min()-stacked law scored wire jerk
  # 6.8-10 m/s3, pump 0.16-0.40, descents 3-4. The single-segment assigned descent must score the
  # human template on all three closed-loop shapes: nominal, clutch-plant, and Doppler-flap.
  # MUTATION KILL MAP (verified): descent off (TERMINAL_CREEP_HOLD_FLOOR=False) -> 3 fixtures
  # fail; min()-stacking restored -> the grade+flap f7b reconstruction fails (the old glide law,
  # deepened by grade a_coast, dips past the curve on flapped GLIDE frames); emission rate-bound
  # removed -> quantized fixtures fail at up to 2.5 m/s3. The u0 anchor alone is NOT separately
  # falsifiable under the emission clamp (a fixed anchor converges to the same smooth trajectory)
  # -- it is deliberate defense-in-depth keeping the RAW target continuous, so the clamp is not
  # the single load-bearing smoothness mechanism.
  _smoothness_gate(simulate(v0=2.4, gap0=12.0, should_stop=False, seed_u=0.0), "nominal")

  def clutch(v: float) -> float:
    return 0.43 * min(max((0.10 - v) / 0.02, 0.0), 1.0)
  _smoothness_gate(simulate(v0=1.2, gap0=8.0, should_stop=True, seed_u=-0.2, push_fn=clutch),
                   "clutch plant")

  def doppler_flicker(t: float) -> float:
    return -0.15 if int(t / 0.03) % 2 == 0 else 0.02
  _smoothness_gate(simulate(v0=1.2, gap0=8.0, should_stop=True, seed_u=-0.2, push_fn=clutch,
                            lead_v_fn=doppler_flicker), "doppler flap")

  # QUANTIZED variants (end-review HIGH: raw 0.03 vEgo quanta stepped the un-bounded curve target
  # at up to the 2.5 general limit -- the plunge re-created from inside; the rate-bounded monotone
  # emission must keep every variant under the same gates)
  _smoothness_gate(simulate(v0=2.4, gap0=12.0, should_stop=False, seed_u=0.0, v_quant=0.03),
                   "nominal quantized")
  _smoothness_gate(simulate(v0=1.2, gap0=8.0, should_stop=True, seed_u=-0.2, push_fn=clutch,
                            v_quant=0.03), "clutch plant quantized")

  # GRADE variant (mutation M2's killing fixture): sustained +0.25 push raises a_coast, which
  # makes the OLD glide relief-cap law ~-0.8 mid-window -- exactly the f7b -0.81 excursion. The
  # assigned descent ignores a_coast (the secure pin and monitor own grade at rest); min()-
  # stacking the curve back alongside the old laws re-admits the deep dip here and fails the
  # jerk/pump gates.
  def grade_clutch(v: float) -> float:
    # 0.25 grade: 0.25 + 0.43 clutch stall = 0.68 < the 0.70 secure hold, so the rest HOLDS --
    # a steeper grade overpowers the hold and the (correct) arrest ladder would add descents,
    # which is the cycle-5 grade-arrest class's contract, not this law's
    return 0.25 + clutch(v)
  _smoothness_gate(simulate(v0=1.2, gap0=8.0, should_stop=True, seed_u=-0.2, push_fn=grade_clutch),
                   "grade+clutch")
  # ...and the full f7b reconstruction: grade + Doppler flap. The flap pushes frames into GLIDE,
  # where the OLD relief-cap law (deepened by the grade's a_coast) reads ~-0.8 -- min()-stacking
  # the curve back alongside the old laws (mutation M2) re-admits that dip exactly here.
  _smoothness_gate(simulate(v0=1.2, gap0=8.0, should_stop=True, seed_u=-0.2, push_fn=grade_clutch,
                            lead_v_fn=doppler_flicker), "grade+flap (f7b reconstruction)")


def test_terminal_smoothness_scorer_rejects_dip_relaunch_slam() -> None:
  # end-review HIGH on the scorer itself: ending the window at the FIRST sub-0.05 sample cut a
  # dip -> relaunch -> harsh-arrest sequence out of the score entirely (it certified the known
  # relaunch class as good). Synthetic trace: clean descent to a 0.04 dip, relaunch to 0.30,
  # wire slams -0.85, second stop. Must flag relaunched (and thus not score as a clean stop).
  T, V, W = [], [], []
  t = 0.0
  def seg(v0, v1, w0, w1, dur):
    nonlocal t
    n = max(int(dur / 0.01), 1)
    for i in range(n):
      f = i / n
      T.append(t)
      V.append(v0 + f * (v1 - v0))
      W.append(w0 + f * (w1 - w0))
      t += 0.01
  seg(0.55, 0.04, -0.30, -0.45, 1.5)   # clean gentle descent into the dip
  seg(0.04, 0.30, -0.45, -0.10, 0.8)   # clutch relaunch, wire unloading
  seg(0.30, 0.00, -0.10, -0.85, 0.6)   # harsh arrest
  seg(0.00, 0.00, -0.85, -0.85, 0.6)   # final rest
  sc = score_terminal(T, V, W)
  assert sc is not None and sc["relaunched"], "scorer failed to flag the dip-relaunch-slam"


def test_terminal_smoothness_scorer_covers_slam_after_flicker_dip() -> None:
  # end-review round 2 (HIGH): a dip to 0.04, quantization flicker to 0.06/0.09 (below the 0.12
  # relaunch flag), then a wire slam -0.35 -> -0.90 and the real stop. The flicker must RESET the
  # standstill candidate so the slam is INSIDE the scored window (jerk caught), while flicker
  # alone must not flag relaunch.
  T, V, W = [], [], []
  t = 0.0
  def seg(v0, v1, w0, w1, dur):
    nonlocal t
    n = max(int(dur / 0.01), 1)
    for i in range(n):
      f = i / n
      T.append(t)
      V.append(v0 + f * (v1 - v0))
      W.append(w0 + f * (w1 - w0))
      t += 0.01
  seg(0.48, 0.04, -0.30, -0.35, 1.2)   # clean descent to the dip
  seg(0.06, 0.09, -0.35, -0.35, 0.3)   # sub-relaunch flicker: candidate must reset
  seg(0.09, 0.00, -0.35, -0.90, 0.15)  # the hidden slam (3.7 m/s3)
  seg(0.00, 0.00, -0.90, -0.90, 0.8)   # sustained final standstill
  sc = score_terminal(T, V, W)
  assert sc is not None
  assert not sc["relaunched"], "sub-0.12 flicker must not flag relaunch"
  assert sc["wire_jerk_max"] > 0.8, f"the post-flicker slam escaped the window: {sc['wire_jerk_max']}"


def test_terminal_smoothness_scorer_keeps_stop_and_go_stops_independent() -> None:
  # end-review round 3 (HIGH): two individually clean stops ~1.1 s apart must retain independent
  # scores -- the first stop's window ends AT its dwelled standstill; the second stop's launch is
  # a departure, not a relaunch, and its braking must not leak into the first score.
  T, V, W = [], [], []
  t = 0.0
  def seg(v0, v1, w0, w1, dur):
    nonlocal t
    n = max(int(dur / 0.01), 1)
    for i in range(n):
      f = i / n
      T.append(t)
      V.append(v0 + f * (v1 - v0))
      W.append(w0 + f * (w1 - w0))
      t += 0.01
  seg(0.48, 0.02, -0.30, -0.70, 1.4)   # clean stop 1
  seg(0.02, 0.02, -0.70, -0.70, 0.7)   # dwelled standstill (>= 0.5 s)
  seg(0.05, 0.60, -0.10, 0.10, 0.6)    # departure (the next stop-and-go launch)
  seg(0.60, 0.02, -0.30, -0.70, 0.5)   # clean stop 2 braking (steeper: must NOT leak in)
  seg(0.02, 0.02, -0.70, -0.70, 0.7)
  sc = score_terminal(T, V, W)
  assert sc is not None
  assert not sc["relaunched"], "the next departure was misread as a relaunch"
  assert sc["descent_count"] == 1, f"stop 2 leaked into stop 1's score: {sc['descent_count']} descents"
  k0, k1 = sc["k_window"]
  assert T[k1] < 2.2, f"window ran past stop 1's standstill (ended at t={T[k1]:.2f})"
  # end-review round 4 (HIGH): t_target must select the WANTED stop's episode -- requesting the
  # HARSH second stop must not return the clean first stop's score, and vice versa
  sc2 = score_terminal(T, V, W, t_target=T[-1])
  assert sc2 is not None
  assert sc2["wire_jerk_max"] > 0.5, f"stop-2 request returned stop 1's clean score: {sc2['wire_jerk_max']}"
  sc1 = score_terminal(T, V, W, t_target=1.8)
  assert sc1["wire_jerk_max"] <= 0.45, f"stop-1 request contaminated: {sc1['wire_jerk_max']}"


def test_terminal_smoothness_scorer_catches_jerk_at_window_entry() -> None:
  # end-review round 5 (HIGH): a wire step timed EXACTLY at the window crossing escaped both
  # metrics (the boundary sample was excluded). The retained boundary sample must put that
  # transition inside the score.
  T, V, W = [], [], []
  t, v, w = 0.0, 0.55, -0.30
  def frame(vv, ww):
    nonlocal t
    T.append(t)
    V.append(vv)
    W.append(ww)
    t += 0.01
  while v >= 0.45:
    frame(v, w)
    v -= 0.004
  while v > 0.02:
    w = max(w - 0.02, -0.70)  # 2.0 m/s3 from the very first sub-window frame
    frame(v, w)
    v -= 0.004
  for _ in range(70):
    frame(0.02, -0.70)
  sc = score_terminal(T, V, W)
  assert sc is not None
  assert sc["wire_jerk_max"] >= 1.9, f"entry-boundary jerk escaped: {sc['wire_jerk_max']}"


def test_terminal_smoothness_scorer_refuses_targeted_request_without_standstill() -> None:
  # end-review round 6: a targeted (CLI) request whose extract contains NO standstill episode --
  # or none within T_TARGET_MAX_DIST_S -- must REFUSE rather than score an unrelated stretch or
  # a neighbouring stop. Untargeted fixture calls keep scoring the first episode.
  T = [i * 0.01 for i in range(400)]
  V = [0.40] * 400              # a long crawl: never reaches standstill
  W = [-0.30 - 0.001 * i for i in range(400)]
  assert score_terminal(T, V, W, t_target=T[-1]) is None, "scored a trace with no standstill"
  assert score_terminal(T, V, W) is not None, "untargeted scoring must still work"
  # ...and a targeted request far from the only real standstill is refused too
  T2 = [i * 0.01 for i in range(400)]
  V2 = [0.40] * 200 + [0.02] * 200
  W2 = [-0.30] * 400
  assert score_terminal(T2, V2, W2, t_target=T2[-1] + 10.0) is None, "scored a distant stop"


def test_terminal_smoothness_scorer_handles_trace_edge_stop() -> None:
  # a trace that ends right at the dip (no dwell available) must score through the edge, not crash
  T = [i * 0.01 for i in range(60)]
  V = [max(0.45 - 0.008 * i, 0.03) for i in range(60)]
  W = [-0.30 - 0.005 * i for i in range(60)]
  sc = score_terminal(T, V, W)
  assert sc is not None and sc["descent_count"] >= 1


def test_slow_grade_crawl_below_roll_bar_is_arrested_by_displacement() -> None:
  # ADVERSARIAL PROBE (cycle-5): a Stribeck+grade push (~0.17) settles the post-latch crawl at an
  # equilibrium v ~0.04 -- below the 0.05 roll bar, invisible to velocity-based triggers, and it
  # walked through the lead in the probe. The displacement lane must arrest it: gap shrinking
  # > 0.15 m below the latch value arms the deep floor.
  def push_fn(v: float) -> float:
    return 0.24 + 0.28 * math.exp(-v / 0.066)  # sustained push beats A_HOLD -0.45 at rest: genuine crawl

  tr = simulate(v0=1.2, gap0=5.0, should_stop=True, seed_u=-0.5, push_fn=push_fn,
                v_quant=0.03, t_max=120.0)
  gaps = [g for g in tr.gap if g is not None]
  assert min(gaps) >= 3.0, f"crawl consumed the gap to {min(gaps):.2f} m"
  # the secure hold (-0.70) may arrest the crawl before the monitor needs to arm -- outcome-based
  tail_v = tr.v[-int(5.0 / DT):]
  assert max(tail_v) < 0.02, "crawl never fully arrested"


def test_aborted_go_reentry_coasts_down_without_monitor_slam() -> None:
  # ROUTE 00001ba2 seg11 (the felt leapfrog): lead crept off -> go-pulse launched the car -> lead
  # stopped again -> service re-entered MID-RISE (v 0.13 rising to 0.40 on launch momentum, gap ~6.3,
  # tiny glide demand). The old running-min roll reference read the rise as a rollaway and slammed
  # -1.0..-1.15. With the entry grace the car must coast down under the gentle glide, monitor silent.
  svc = StoppingService()
  v = 0.13
  r = None
  worst = 0.0
  for k in range(600):
    # launch momentum decays: v rises to ~0.40 over 0.5s then falls under gentle braking
    t = k * DT
    v = min(0.13 + 0.9 * t, 0.40) if t < 0.5 else max(0.40 - 0.25 * (t - 0.5), 0.0)
    r = svc.update(engaged=True, v_ego=v, a_ego=0.5 if t < 0.5 else -0.25, a_target=None,
                   should_stop=True, dts_planner=None, planner_min_limit=-3.5,
                   signals=make_signals(d_gap=6.3, latch=True),
                   lead_status=True, lead_v=0.2, dt=DT, wire_accel=-0.10)
    if t < 0.5:
      # END-REVIEW ROUND 1 (HIGH): the creep-hold floor must NEVER bind on the RISING launch leg
      # -- entering at 0.13 m/s with v rising on go momentum, an unarmed floor would drag the
      # wire toward -0.42..-0.52 and release it past 0.30 m/s: a brake pulse mid-launch. The
      # arming latch (v below peak-since-activation by FLOOR_ARM_DROP_MPS) forbids it; removing
      # the latch fails this bound.
      assert r.accel >= -0.36, f"floor bound on the rising launch leg: {r.accel:.2f} at t={t:.2f}"
    if v >= 0.15:
      # cycle-18: the slam bound applies to the launch-momentum rise and the coast-down; below
      # 0.15 m/s the terminal creep-hold floor (armed post-peak) deliberately deepens toward
      # A_HOLD_SECURE so the clutch engagement (+0.43 below 0.08 m/s) cannot relaunch the finish.
      worst = min(worst, r.accel if r.active else 0.0)
    assert not r.debug.get("monitor_active", False), f"monitor armed at t={t:.2f} on launch momentum"
  # cycle-19 re-derivation: the coast-down runs the single-segment descent (deeper mid-window,
  # -0.66 at 0.15 by design -- the 'brake a bit stronger' half of the human technique). The slam
  # this bound was built against was -1.0..-1.15; the descent never exceeds A_HOLD_SECURE.
  assert worst >= P.A_HOLD_SECURE - 0.01, f"aborted-go re-entry was slammed to {worst:.2f}"


def test_genuine_rollaway_after_entry_still_caught() -> None:
  # The entry grace must NOT blind the monitor to a real rollaway that begins under service control:
  # enter at rest-ish, stay quiet past the grace, then roll (v rising from 0.02 to 0.2 with the gap
  # closing) -> monitor arms.
  svc = StoppingService()
  armed = False
  gap = 4.0
  for k in range(800):
    t = k * DT
    if t < 1.0:
      v = 0.02
    else:
      v = min(0.02 + 0.35 * (t - 1.0), 0.25)  # rollaway
      gap = max(gap - v * DT, 2.5)
    r = svc.update(engaged=True, v_ego=v, a_ego=0.0, a_target=None, should_stop=True,
                   dts_planner=None, planner_min_limit=-3.5,
                   signals=make_signals(d_gap=gap, latch=True),
                   lead_status=True, lead_v=0.0, dt=DT, wire_accel=-0.20)
    if r.debug.get("monitor_active", False):
      armed = True
      break
  assert armed, "genuine post-entry rollaway never armed the monitor"


def test_arrival_grace_ends_immediately_on_observed_roll() -> None:
  # Cycle-8 e65 class: a shallow natural arrival (-0.20) that cannot hold the car must NOT be
  # preserved for the full grace while the car rolls -- any v rise above the post-latch minimum
  # ends the grace and the secure hold builds at once. Roll travel stays in the low centimeters.
  svc = StoppingService()
  v = 0.05
  travel = 0.0
  sig_gap = 5.0
  previous_accel = None
  fast_arrest_seen = False
  for k in range(200):
    t = k * DT
    # scripted: latch at rest-ish, then the car starts rolling at t=0.1 (insufficient arrival hold)
    if t < 0.1:
      v = 0.02
    else:
      v = min(0.02 + 0.30 * (t - 0.1), 0.30)
      travel += v * DT
      sig_gap = max(sig_gap - v * DT, 2.5)
    sig = make_signals(d_gap=sig_gap, latch=True, wheel=True)
    r = svc.update(engaged=True, v_ego=v, a_ego=0.0, a_target=None, should_stop=True,
                   dts_planner=None, planner_min_limit=-3.5, signals=sig,
                   lead_status=True, lead_v=0.0, dt=DT, wire_accel=-0.20)
    if previous_accel is not None and v > 0.04 and r.accel - previous_accel <= -0.75 * P.J_SAFE * DT:
      fast_arrest_seen = True
    previous_accel = r.accel
    if t >= 0.15 and r.phase in (Phase.RAMP_TO_HOLD, Phase.HOLD):
      # within 5 frames of the observed rise the phase command must be building, not holding -0.20
      if t >= 0.16:
        assert r.debug["a_phase"] <= -0.20 - EPS or r.accel <= -0.20 - EPS, \
          f"grace still holding the insufficient arrival at t={t:.2f}"
      if fast_arrest_seen:
        break
  else:
    raise AssertionError("never reached the post-latch roll scenario")
  assert fast_arrest_seen, "observed roll only used the parked J_HOLD rate instead of the existing fast-deepen path"


def test_sustained_observed_lead_departure_releases_stale_negative_plan() -> None:
  # 00001e7b/82 and 00001efe/70: the lead steadily drove away while shouldStop and negative
  # aTarget remained stale, so HOLD pinned the ego until the driver used gas at 6-16 m. Physical
  # departure is sufficient after confirmation; one short Doppler excursion (00001c90/142) is not.
  svc = StoppingService()
  svc.phase = Phase.HOLD
  svc._last_cmd = P.A_HOLD_SECURE
  svc._hold_entry_gap = 4.0
  sig = make_signals(d_gap=4.6, latch=True, wheel=True)

  for _ in range(int(P.RELEASE_LEAD_CONFIRM_S / DT) - 1):
    r = svc.update(engaged=True, v_ego=0.0, a_ego=0.0, a_target=-0.4, should_stop=True,
                   dts_planner=None, planner_min_limit=-3.5, signals=sig,
                   lead_status=True, lead_v=0.8, dt=DT, wire_accel=P.A_HOLD_SECURE)
    assert r.phase == Phase.HOLD

  for _ in range(3):
    r = svc.update(engaged=True, v_ego=0.0, a_ego=0.0, a_target=-0.4, should_stop=True,
                   dts_planner=None, planner_min_limit=-3.5, signals=sig,
                   lead_status=True, lead_v=0.8, dt=DT, wire_accel=P.A_HOLD_SECURE)
    if r.phase == Phase.RELEASE:
      break
  assert r.phase == Phase.RELEASE
  assert r.accel > P.A_HOLD_SECURE


def test_brief_observed_lead_departure_does_not_release_hold() -> None:
  svc = StoppingService()
  svc.phase = Phase.HOLD
  svc._last_cmd = P.A_HOLD_SECURE
  svc._hold_entry_gap = 5.6
  for _ in range(int(0.2 / DT)):
    r = svc.update(engaged=True, v_ego=0.0, a_ego=0.0, a_target=-0.4, should_stop=True,
                   dts_planner=None, planner_min_limit=-3.5,
                   signals=make_signals(d_gap=6.3, latch=True, wheel=True),
                   lead_status=True, lead_v=0.57, dt=DT, wire_accel=P.A_HOLD_SECURE)
    assert r.phase == Phase.HOLD

  r = svc.update(engaged=True, v_ego=0.0, a_ego=0.0, a_target=-0.4, should_stop=True,
                 dts_planner=None, planner_min_limit=-3.5,
                 signals=make_signals(d_gap=6.1, latch=True, wheel=True),
                 lead_status=True, lead_v=0.1, dt=DT, wire_accel=P.A_HOLD_SECURE)
  assert r.phase == Phase.HOLD
  assert r.debug["lead_departure_confirm_s"] == 0.0


def test_held_gap_prediction_does_not_confirm_physical_departure() -> None:
  svc = StoppingService()
  svc.phase = Phase.HOLD
  svc._last_cmd = P.A_HOLD_SECURE
  svc._hold_entry_gap = 4.0
  held = StopSignals(d_gap=5.0, gap_source="held", gap_hold_outward=True, dropout_active=False, a_coast=0.0,
                     wheel_stop_latched=True, lead_confirmed_stopped=False)
  for _ in range(int(2.0 * P.RELEASE_LEAD_CONFIRM_S / DT)):
    r = svc.update(engaged=True, v_ego=0.0, a_ego=0.0, a_target=-0.4, should_stop=True,
                   dts_planner=None, planner_min_limit=-3.5, signals=held,
                   lead_status=True, lead_v=0.8, dt=DT, wire_accel=P.A_HOLD_SECURE)
  assert r.phase == Phase.HOLD
  assert r.debug["lead_departure_confirm_s"] == 0.0


def test_far_stationary_settle_never_restarts_toward_an_unmoved_lead() -> None:
  # A bad far settle remains a measured defect, but automatically starting after wheel-stop creates
  # the exact stop/start/re-stop behavior the settled authority exists to prevent.
  svc = StoppingService()
  r = svc.update(engaged=True, v_ego=0.24, a_ego=-0.3, a_target=-0.5, should_stop=True,
                 dts_planner=None, planner_min_limit=-3.5,
                 signals=make_signals(d_gap=13.7, latch=True),
                 lead_status=True, lead_v=0.0, dt=DT, wire_accel=-0.3)
  assert r.phase in (Phase.APPROACH_GLIDE, Phase.PRE_STOP_EASE)

  r = svc.update(engaged=True, v_ego=0.04, a_ego=-0.1, a_target=-0.6, should_stop=True,
                 dts_planner=None, planner_min_limit=-3.5,
                 signals=make_signals(d_gap=13.6, wheel=True, latch=True),
                 lead_status=True, lead_v=0.0, dt=DT, wire_accel=-0.3)
  assert r.phase == Phase.RAMP_TO_HOLD


def test_far_stationary_gap_does_not_move_a_preexisting_standstill() -> None:
  # 00001efe/59 entered while already parked. Geometry from before this service engagement is not
  # authority to launch the car, even when it is outside the preferred lead-gap band.
  svc = StoppingService()
  r = svc.update(engaged=True, v_ego=0.0, a_ego=0.0, a_target=-0.4, should_stop=True,
                 dts_planner=None, planner_min_limit=-3.5,
                 signals=make_signals(d_gap=6.6, wheel=True, latch=True),
                 lead_status=True, lead_v=0.0, dt=DT, wire_accel=-0.4)
  assert r.phase == Phase.RAMP_TO_HOLD


def test_far_stationary_gap_does_not_override_explicit_stop_target() -> None:
  svc = StoppingService()
  svc.update(engaged=True, v_ego=0.24, a_ego=-0.3, a_target=-0.5, should_stop=True,
             dts_planner=0.8, planner_min_limit=-3.5,
             signals=make_signals(d_gap=13.7, latch=True),
             lead_status=True, lead_v=0.0, dt=DT, wire_accel=-0.3)
  r = svc.update(engaged=True, v_ego=0.04, a_ego=-0.1, a_target=-0.6, should_stop=True,
                 dts_planner=0.8, planner_min_limit=-3.5,
                 signals=make_signals(d_gap=13.6, wheel=True, latch=True),
                 lead_status=True, lead_v=0.0, dt=DT, wire_accel=-0.3)
  assert r.phase == Phase.RAMP_TO_HOLD


def test_radar_dropout_while_parked_does_not_false_arm_the_crawl_arrest() -> None:
  # Reviewer finding 4a (cycle-5): a stationary-target radar dropout while parked makes the
  # conditioned gap decay inward (decay-hold); on re-acquire the deficit vs the latch reference
  # read as a "crawl" and armed -0.70 permanently. Untrusted frames must freeze the lane and the
  # reference must re-base on the first trusted frame.
  ctx, svc = StopContext(), StoppingService()
  v, a_act, gap, last_u = 1.5, 0.0, 8.0, -0.4
  tau = 0.2
  stopped = False
  k_stop = None
  armed_after_stop = False
  for i in range(3000):
    lead_alive = not (stopped and k_stop is not None and (i - k_stop) in range(200, 260))  # 0.6s dropout while parked
    signals = ctx.update(v_ego=v if not stopped else [0.0, 0.03, 0.04][i % 3], a_ego=0.0, a_cmd=last_u,
                         lead_status=lead_alive, lead_v=0.0, lead_d_rel=gap if lead_alive else None,
                         standstill=stopped, dt=DT)
    r = svc.update(engaged=True, v_ego=v if not stopped else [0.0, 0.03, 0.04][i % 3], a_ego=0.0,
                   a_target=None, should_stop=True, dts_planner=None, planner_min_limit=-3.5,
                   signals=signals, lead_status=lead_alive, lead_v=0.0, dt=DT, wire_accel=last_u)
    u = r.accel if r.active else 0.0
    a_act += (u - a_act) * DT / (tau + DT)
    if not stopped:
      v = max(v + a_act * DT, 0.0)
      gap -= max(v * DT, 0.0)
      if v < 0.005:
        stopped = True
        k_stop = i
    elif k_stop is not None and i > k_stop + 50:
      if r.debug.get("monitor_active", False):
        armed_after_stop = True
    last_u = u
  assert stopped
  assert not armed_after_stop, "dropout-while-parked false-armed the crawl arrest"
  assert last_u == pytest.approx(P.A_HOLD_SECURE, abs=0.03), f"hold rested at {last_u:.2f}"


def test_subquantization_crawl_is_arrested_by_displacement() -> None:
  # Reviewer finding 2 (cycle-5): a crawl whose vEgo READING is 0.0 (true v ~0.015, below the 0.03
  # quantization step) bypassed every velocity trigger AND the old placement of the displacement
  # check (below the standstill early-out). With the check hoisted, gap consumption > 0.15 m must
  # arrest it even at reading 0.0.
  def push_fn(v: float) -> float:
    return 0.50  # sustained push; plant creeps while the quantized reading floors to 0.0

  tr = simulate(v0=1.0, gap0=6.0, should_stop=True, seed_u=-0.5, push_fn=push_fn,
                v_quant=0.20, t_max=90.0)  # coarse quantization: readings floor to 0 below 0.1 true
  gaps = [g for g in tr.gap if g is not None]
  assert min(gaps) >= 3.0, f"sub-quantization crawl consumed the gap to {min(gaps):.2f} m"
  tail_v2 = tr.v[-int(5.0 / DT):]
  assert max(tail_v2) < 0.02, "sub-quantization crawl never fully arrested"


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


def test_hold_with_sustained_departing_lead_releases_instead_of_arresting() -> None:
  # A confirmed departing lead is now a go source even with stale planner stop intent. The same
  # physical motion must not be classified as a rollaway and ratcheted deeper while the gap opens.
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
  assert r.debug["lead_departure_confirm_s"] >= P.RELEASE_LEAD_CONFIRM_S
  assert r.phase in (Phase.RELEASE, Phase.APPROACH_GLIDE)


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
  assert tr.u[-1] == pytest.approx(P.A_HOLD_SECURE, abs=0.02)


# --- close entries: D_REST_eff re-zeroes (ledger D1-H2) --------------------------------------------

@pytest.mark.parametrize("v0,seed,wire_band", [(0.6, -0.2, (-0.75, -0.05)), (1.2, -0.6, (-1.6, -0.03))])
def test_close_entry_gap_3_rest_rezeroes_and_wire_bounded(v0: float, seed: float, wire_band) -> None:
  # DEVIATION from the task's fixture wording, per the plan itself: plan §6 stage 0 requires
  # "wheel-stop-release u >= -0.35 on NOMINAL fixtures" only. For the v=1.2 close entry the exact
  # §3 laws CANNOT land in [-0.35, -0.05]: D_REST_MIN clamps the rest at 2.4 m (required decel 1.2
  # > A_REST_FEAS), the terminal gap sits below the 2.6 m EASE gap gate (no shallow region exists
  # by design, ledger D1-H2), and the floored-denominator demand is genuinely deep; J_UP cannot
  # shed that in the ~0.4 s the car takes to stop. Safety over feel is the intended trade there.
  # CYCLE-11 pin-law note (v=0.6 band widened -0.35 -> -0.60): this plant is friction-free and
  # unquantized, so the finish decays as a nonphysical 0.025 m/s^2 crawl tail that no measured
  # signal can distinguish from a secure stop within the dwell (a real Stribeck plant either
  # breaks such a crawl to a stop or hangs it into the monitor's hover/displacement lanes; the
  # AR(1) plant is invalid below 0.21 m/s). The secure pin firing into that tail is the correct
  # response to what the signals show. CYCLE-18: the v=0.6 band deepens to -0.75 -- the terminal
  # creep-hold floor lands arrivals at A_HOLD_SECURE by design (see test_crank1_arrival_*).
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
  # entry-frame anchor: min(4.0, max(4.2 - 2.0^2/(2*A_REST_FEAS 1.2) = 2.533, D_REST_MIN 3.0)) = 3.0
  # (band retune 2026-07-20: close-entry landings floor at 3.0) -- not the 4.0 the (8.0, 3.0)
  # sighting would have anchored
  assert r.debug["d_rest_eff"] == pytest.approx(P.D_REST_MIN, abs=1e-9)


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


def test_terminal_creep_hold_floor_schedule_pins_phase_demand() -> None:
  # cycle-19 anchored descent, driven on a PHYSICAL trajectory (0.31 m/s2 decel -- the rate-
  # bounded monotone emission tracks the curve exactly when slope*decel < J_TERMINAL_DESCENT;
  # a nonphysical fixture that teleports v cannot follow and pins nothing). Pins: (a) the first
  # armed frame's demand equals the wire at arming (continuity); (b) monotone; (c) secure depth
  # from v <= 0.10; (d) a tracked midpoint sits on the (v0,u0)->(0.10,-0.70) line.
  svc = StoppingService()
  sig = make_signals(d_gap=4.6, a_coast=0.45, latch=True)
  wire = -0.18
  decel = 0.31
  v = 0.52
  demands = []  # (v, a_phase)
  first_armed = None
  for _ in range(160):
    r = svc.update(engaged=True, v_ego=v, a_ego=-decel, a_target=None, should_stop=True,
                   dts_planner=None, planner_min_limit=-3.5, signals=sig,
                   lead_status=True, lead_v=0.0, dt=DT, wire_accel=wire)
    demands.append((v, r.debug["a_phase"]))
    if first_armed is None and svc._creep_floor_armed:
      first_armed = (v, r.debug["a_phase"], wire)
    wire = r.accel
    v = max(v - decel * DT, 0.05)
  assert first_armed is not None
  assert first_armed[1] == pytest.approx(svc._descent_u0, abs=1e-9)  # continuity at arming
  v0, u0 = svc._descent_v0, max(svc._descent_u0, P.A_HOLD_SECURE)
  armed_demands = [d for vv, d in demands if vv <= v0 + 1e-9]
  assert all(b <= a + 1e-9 for a, b in zip(armed_demands, armed_demands[1:], strict=False)), "not monotone"
  lows = [d for vv, d in demands if vv <= 0.10 + 1e-9]
  assert lows and lows[-1] == pytest.approx(P.A_HOLD_SECURE, abs=1e-3)
  v_mid = round(v0 - (v0 - 0.10) / 2.0, 4)
  d_mid = min(demands, key=lambda x: abs(x[0] - v_mid))[1]
  expect = u0 + (v0 - v_mid) / (v0 - P.V_CREEP_HOLD_SECURE) * (P.A_HOLD_SECURE - u0)
  assert d_mid == pytest.approx(expect, abs=0.02)


def test_dropout_floor_binds_shallow_demand() -> None:
  # cycle-18 note: run at v=0.35 / gap 4.5 -- ABOVE the terminal creep-hold window (v < 0.30),
  # where the phase demand is still genuinely shallow, so A_DROPOUT_MIN is the binding floor
  # exactly as before. Below 0.30 the creep-hold floor is deliberately deeper than -0.25.
  svc = StoppingService()
  sig = make_signals(d_gap=4.5, dropout=True)
  r = None
  for _ in range(30):  # short: the (correctly) hovering scripted v would engage the monitor later
    r = svc.update(engaged=True, v_ego=0.35, a_ego=-0.05, a_target=None, should_stop=True,
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
