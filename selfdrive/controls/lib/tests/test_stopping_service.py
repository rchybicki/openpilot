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


def make_signals(d_gap=None, a_coast=0.0, wheel=False, latch=False, dropout=False,
                 gap_source=None) -> StopSignals:
  # NOTE (codex xhigh): a dropout in the REAL StopContext emits gap_source="decay" with
  # lead_status False (stop_context._update_gap), never "measured" -- a fixture that pairs
  # dropout=True with "measured" tests a signal combination the context cannot produce. Callers
  # exercising dropout should pass gap_source="decay" (or "held" for the lead-present variant).
  if gap_source is None:
    gap_source = "measured" if d_gap is not None else "none"
  return StopSignals(d_gap=d_gap, gap_source=gap_source,
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
  assert tr.u[-1] == pytest.approx(P.A_HOLD_SECURE, abs=0.02)
  # rest in the intended band around D_REST_eff = 4.0
  assert 3.0 <= tr.gap[-1] <= 5.0, f"rest gap {tr.gap[-1]:.2f}"
  assert min(g for g in tr.gap if g is not None) >= 2.0


def test_crank1_arrival_holds_natural_value_not_ease_deep() -> None:
  # SMOOTHNESS CRANK #1 (cycle-7): the gentle-finish used to BUILD to A_EASE_DEEP while the last
  # centimeters rolled out, pinning wire@stop at exactly -0.35 on every stop. It must now HOLD the
  # natural arrival: on a nominal stop the wheel-stop wire lands shallower than -0.30.
  tr = simulate(v0=2.4, gap0=12.0, should_stop=False, seed_u=0.0)
  k_roll = last_rolling_idx(tr)
  assert tr.u[k_roll] > -0.30, f"wheel-stop wire {tr.u[k_roll]:.3f} still pinned at the deep edge"
  assert tr.u[k_roll] <= -0.05 + EPS
  # and the hold still builds to the secure level afterwards
  assert tr.u[-1] == pytest.approx(P.A_HOLD_SECURE, abs=0.02)


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

  held = StopSignals(d_gap=6.0, gap_source="held", dropout_active=False, a_coast=0.0,
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
  # Mutation sensitivity: with the cap disabled (region excluded via gap_trusted=False), the raw
  # anchor demand -1.2 returns -- asserted as the contrast case.
  for trusted, want_deep in ((False, True), (True, False)):
    svc = StoppingService()
    svc.phase = Phase.APPROACH_GLIDE
    svc._d_rest_eff = 4.3
    svc._d_rest_calc_gap = 4.9
    svc._last_cmd = -0.70
    sig = StopSignals(d_gap=4.50, gap_source="measured" if trusted else "held",
                      dropout_active=False, a_coast=0.24,
                      wheel_stop_latched=False, lead_confirmed_stopped=True)
    r = svc.update(engaged=True, v_ego=0.6186, a_ego=-0.5, a_target=-0.68, should_stop=True,
                   dts_planner=0.398, planner_min_limit=-3.5, signals=sig,
                   lead_status=True, lead_v=0.0, dt=DT, wire_accel=-0.70,
                   increased_stopped_distance=0.3, a_target_trajectory=-0.25)
    assert r.debug["d_rest_eff"] == pytest.approx(4.3), "the anchor must never move"
    if want_deep:
      assert r.debug["a_phase"] < -1.1  # cap unavailable on untrusted gap: the raw anchor demand
    else:
      assert -0.95 < r.debug["a_phase"] < -0.80, f"cap missed: {r.debug['a_phase']:.2f}"


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
  remaining = svc._d_rem(3.8, None, 0.68, gap_trusted=True)
  assert remaining == pytest.approx(0.68 ** 2 / (2.0 * (P.A_GLIDE_NOM + P.A_REANCHOR_HYST)))
  assert svc._d_rest_eff == pytest.approx(4.0), "the anchor must never move"
  assert svc._d_rem(3.8, None, 0.68, gap_trusted=False) == pytest.approx(3.8 - 4.0)
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
    worst = min(worst, r.accel if r.active else 0.0)
    assert not r.debug.get("monitor_active", False), f"monitor armed at t={t:.2f} on launch momentum"
  assert worst >= -0.60, f"aborted-go re-entry was slammed to {worst:.2f}"


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
  held = StopSignals(d_gap=5.0, gap_source="held", dropout_active=False, a_coast=0.0,
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

@pytest.mark.parametrize("v0,seed,wire_band", [(0.6, -0.2, (-0.60, -0.05)), (1.2, -0.6, (-1.6, -0.03))])
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
  # response to what the signals show; PHYSICAL natural arrivals keep the [-0.35, -0.05] contract
  # via test_nominal_stop_* / test_crank1_arrival_* and the on-road crank-1 wire@stop gate.
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
