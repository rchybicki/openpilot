"""LAUNCH_FOLLOW (stopping_flags, cycle 2026-10-17 WS LAUNCH-C; phase A = cycle 2026-10-13 RELEASE_AT_DECISION).

At rest behind a departing lead the service decides HOLD -> RELEASE on the planner go plus the lead's earned Doppler departure; today
the command then ramps -0.70 -> 0 at J_GO (0.6 s) under `stopping` (StopReq), and the arbiter's close-stopped-lead latch holds
`stopping` ~0.3 s more. With the flag the at-rest RELEASE steps to the release level on the first guarded frame (phase A) and the
decision is the arbiter's departing-lead release while LongControl keeps it armed, so `stopping` ends on the next frame; the planner's
standstill shouldStop does not take the launch back (the H5 flicker class), the model's stop bit and a stopped or braking lead do
(abort into the service's hold). From the step until the hand-over the command never exceeds the lead's pull-away L (the final
deepen-only ceiling after the low-speed slew), and the ceiling rises at J_GO at most."""
import math

import pytest

from openpilot.selfdrive.controls.lib import longcontrol, stopping_flags
from openpilot.selfdrive.controls.lib.longcontrol import LongControl, LongCtrlState
from openpilot.selfdrive.controls.lib.stop_and_go_helpers import departing_lead_release_gap
from openpilot.selfdrive.controls.lib.stopping_service import Phase, ServiceParams
from openpilot.selfdrive.controls.lib.tests.test_longcontrol_fast_release import DummyCarParams, DummyCarState, DummyFrogPilotToggles
from openpilot.selfdrive.controls.lib.tests.test_release_end_rehold import GO, LIMITS, Radar, travel

P = ServiceParams()
DT = 0.01
J_UP = P.J_GO * DT   # the ceiling's largest rise per frame
GAP0 = 4.4       # rest gap behind the lead (car launches: d0 p50 4.65 m)
T_DEP = 6.0      # the lead starts at 1 m/s^2 (up to 1.5 m/s)
T_GO = 6.8       # the planner's go (shouldStop 0, aTarget +0.42)
T_STEP = T_GO + P.RELEASE_STEP_GO_S
LIVE_FLAGS = ("GAP_AWARE_TERMINAL", "DELIVERED_LANDING", "LANDING_LEAD_TOLERANCE", "ENTRY_COMFORT_CAP", "RELEASE_GO_EVIDENCE_KEEP",
              "OWNED_STARTING_TRANSIT", "SERVICE_HOLD_ENTERS_STOPPING", "STOPREQ_SETTLED_SET", "HOLD_DEPARTURE_RELEASE")


@pytest.fixture(autouse=True)
def _flag_on(monkeypatch):
  monkeypatch.setattr(stopping_flags, "LAUNCH_FOLLOW", True)


def pull(lead_v, v_ego, lead_a=0.0):
  return max(lead_a, 0.0) + lead_v - v_ego - stopping_flags.LF_V_MARGIN - stopping_flags.LF_LAG_S * abs(lead_a)


def drive(seconds=9.0, gap=None, lead_v=None, lead_a=lambda t: 0.0, lead=lambda t: True, track=lambda t: 7,
          should_stop=lambda t: t < T_GO, a_target=lambda t: GO if t >= T_GO else -0.3, model_stop=lambda t: False,
          gas=lambda t: False):
  """LongControl (LIVE service ownership) at rest behind a lead that departs at T_DEP (recorded-input semantics: the ego stays
  at rest); rows per frame: (t, state, service phase, wire, launch armed, the service's step command, ceiling)."""
  cp = DummyCarParams()
  cp.startingState = True
  lc = LongControl(cp)
  toggles = DummyFrogPilotToggles()
  radar = Radar(gap or (lambda t: GAP0 + travel(t - T_DEP)), lead_v or (lambda t: min(max(t - T_DEP, 0.0), 1.5)))
  rows = []
  for i in range(int(seconds / DT)):
    t = i * DT
    d, lv = radar.read(t)
    out = lc.update(active=True, CS=DummyCarState(v_ego=0.0, a_ego=0.0, standstill=True), a_target=a_target(t),
                    should_stop=should_stop(t), distance_to_stop_target_m=0.3 if should_stop(t) else -1.0, accel_limits=LIMITS,
                    frogpilot_toggles=toggles, lead_status=lead(t), lead_v=lv, lead_d_rel=d, lead_a=lead_a(t), lead_track_id=track(t),
                    model_should_stop=model_stop(t), freeze_integrator=gas(t))
    rows.append((t, lc.long_control_state, lc._service_shadow_svc.phase, float(out), lc._launch_release,
                 lc._service_shadow_svc.launch_cmd, lc._lf_cap))
  return rows


class Follow:
  """Closed loop for the bound: the ego answers the wire through a 0.25 s lag (at rest only for a positive wire), the lead follows
  lead_v(t) from GAP0, radar at 20 Hz (0.1 m range quantisation). The step and the at-rest launch come from drive()'s timeline."""

  def __init__(self, lead_v, lead_a=lambda t: 0.0, lead=lambda t: True, track=lambda t: 7, gas=lambda t: False,
               active=lambda t: True, should_stop=lambda t: t < T_GO, a_target=lambda t: GO if t >= T_GO else -0.3):
    cp = DummyCarParams()
    cp.startingState = True
    self.lc, self.toggles = LongControl(cp), DummyFrogPilotToggles()
    self.lead_v, self.lead_a, self.lead, self.track, self.gas, self.active = lead_v, lead_a, lead, track, gas, active
    self.should_stop, self.a_target = should_stop, a_target
    self.t, self.x, self.v, self.a, self.xl, self.meas = 0.0, 0.0, 0.0, 0.0, GAP0, None
    self.rows = []

  def step(self):
    t = self.t
    if self.meas is None or round(t / DT) % 5 == 0:
      self.meas = (round((self.xl - self.x) / 0.1) * 0.1, self.lead_v(t))
    d, lv = self.meas
    lc = self.lc
    out = lc.update(active=self.active(t), CS=DummyCarState(v_ego=self.v, a_ego=self.a, standstill=self.v <= 0.0),
                    a_target=self.a_target(t), should_stop=self.should_stop(t), distance_to_stop_target_m=0.3 if self.should_stop(t) else -1.0,
                    accel_limits=LIMITS, frogpilot_toggles=self.toggles, lead_status=self.lead(t), lead_v=lv, lead_d_rel=d,
                    lead_a=self.lead_a(t), lead_track_id=self.track(t), model_should_stop=False, freeze_integrator=self.gas(t))
    self.rows.append(dict(t=t, state=lc.long_control_state, phase=lc._service_shadow_svc.phase, wire=float(out), cap=lc._lf_cap,
                          armed=lc._launch_release, v=self.v, gap=self.xl - self.x, lv=lv, d=d, lead_a=self.lead_a(t)))
    if not (self.v <= 0.0 and out <= 0.0):
      self.a += (out - self.a) * DT / 0.25
    self.v = max(self.v + self.a * DT, 0.0)
    if self.v <= 0.0:
      self.a = min(self.a, 0.0)
    self.x += self.v * DT
    self.xl += self.lead_v(t) * DT
    self.t += DT

  def run(self, seconds):
    for _ in range(int(round(seconds / DT))):
      self.step()
    return self.rows


def lead_ramp(a=1.0, v_max=1.5):
  return lambda t: min(max(t - T_DEP, 0.0) * a, v_max)


def k_step(rows):
  return next(k for k, r in enumerate(rows) if (r[5] if isinstance(r, tuple) else r['cap']) is not None)


def test_departure_leaves_stopping_on_the_frame_after_the_step():
  rows = drive()
  k_dec = next(k for k, r in enumerate(rows) if r[2] == Phase.RELEASE)
  k = k_step(rows)
  assert rows[k_dec][0] == pytest.approx(T_GO), "the decision itself is today's"
  assert (k - k_dec) * DT == pytest.approx(P.RELEASE_STEP_GO_S, abs=1.5 * DT), "the step waits for the planner's go to hold"
  assert rows[k - 1][2] == Phase.RELEASE and rows[k - 1][3] == pytest.approx(P.A_HOLD_SECURE + P.J_GO * (k - 1 - k_dec) * DT, abs=0.02)
  assert rows[k][2] == Phase.INACTIVE and rows[k][1] == LongCtrlState.stopping
  assert rows[k][3] == pytest.approx(0.0, abs=0.005), "the step frame carries the release level"
  assert rows[k + 1][1] == LongCtrlState.starting and rows[k + 1][4], "`stopping` (StopReq) must end on the next frame"
  launch = [r[3] for r in rows[k:k + 40]]
  assert all(b - a <= 0.030 + 1e-9 for a, b in zip(launch, launch[1:], strict=False)), "faster than the departing-lead fast release"
  assert launch[-1] > 0.5


def test_flag_off_keeps_the_ramp_and_the_latch(monkeypatch):
  on = drive()
  monkeypatch.setattr(stopping_flags, "LAUNCH_FOLLOW", False)
  off = drive()
  k_on = k_step(on)
  assert on[:k_on] == off[:k_on], "identical up to the step frame"
  assert all(r[4] is False and r[5] is None and r[6] is None for r in off)
  exit_on = next(k for k, r in enumerate(on) if r[0] >= T_GO and r[1] != LongCtrlState.stopping)
  exit_off = next(k for k, r in enumerate(off) if r[0] >= T_GO and r[1] != LongCtrlState.stopping)
  assert (exit_off - exit_on) * DT >= -P.A_HOLD_SECURE / P.J_GO - P.RELEASE_STEP_GO_S - 0.05
  assert all(a[3] >= b[3] - 1e-9 for a, b in zip(on, off, strict=True)), "the at-rest launch command is never behind today's"


@pytest.mark.parametrize("ticks", [1, 3])
def test_brief_planner_go_keeps_todays_ramp(monkeypatch, ticks):
  # the brief-go race (227e 241.2, 20c0 2458.5): the planner's go lasts 1-3 ticks and the stop request returns while the lead creeps
  t_end = T_GO + 0.05 * ticks
  kw = dict(should_stop=lambda t: not T_GO <= t < t_end, a_target=lambda t: GO if T_GO <= t < t_end else -0.3)
  on = drive(**kw)
  monkeypatch.setattr(stopping_flags, "LAUNCH_FOLLOW", False)
  assert on == drive(**kw)
  assert all(r[1] == LongCtrlState.stopping for r in on if r[0] >= T_GO)


def test_range_drift_go_keeps_todays_ramp_and_re_hold(monkeypatch):
  # the 4904 class: standstill range drift (Doppler 0) arms gap growth, the planner goes toward a stopped lead: no step, no early exit
  def scen():
    return drive(seconds=12.0, gap=lambda t: 4.6 + 0.1 * min(max(t - 3.0, 0.0), 5.0), lead_v=lambda t: 0.0,
                 should_stop=lambda t: t < 8.5, a_target=lambda t: GO if t >= 8.5 else -0.3)
  on = scen()
  monkeypatch.setattr(stopping_flags, "LAUNCH_FOLLOW", False)
  assert on == scen()
  assert all(r[1] == LongCtrlState.stopping for r in on if r[0] >= 8.5)


@pytest.mark.parametrize("veto", ["braking_lead", "no_stop_request_drop", "no_lead"])
def test_unguarded_decisions_keep_todays_release(monkeypatch, veto):
  kw = {}
  if veto == "braking_lead":
    kw["lead_a"] = lambda t: -0.5 if t >= T_DEP else 0.0                 # aLeadK below ATTR_LEAD_BRAKING at the decision
  elif veto == "no_stop_request_drop":
    kw["should_stop"] = lambda t: True                                    # the planner keeps asking to stop
  else:
    kw["lead"] = lambda t: t < T_DEP                                      # no lead at the decision: the no-lead go
  on = drive(**kw)
  monkeypatch.setattr(stopping_flags, "LAUNCH_FOLLOW", False)
  assert on == drive(**kw)
  assert all(r[5] is None and r[4] is False for r in on)


def test_planner_standstill_stop_after_the_step_keeps_the_launch():
  # runner syn_launch / 2232_s82 (phase A H5): the planner's standstill shouldStop returns 0.1 s after the step for 1.1 s while the
  # lead keeps pulling away; phase A aborted into a re-hold (+1.06 s). Today's departing-lead release ignores that stop; so does
  # the armed launch, under the bound; the service's departure mask keeps it out
  t_back = T_STEP + 0.1
  rows = drive(should_stop=lambda t: t < T_GO or t_back <= t < t_back + 1.1, a_target=lambda t: GO if T_GO <= t < t_back else 0.08)
  k = k_step(rows)
  after = [r for r in rows if t_back <= r[0] < t_back + 1.1]
  assert all(r[1] == LongCtrlState.starting and r[2] == Phase.INACTIVE and r[4] for r in after)
  assert all(r[3] <= r[6] + 1e-9 for r in rows[k + 1:] if r[6] is not None), "the ceiling binds the launch"
  assert max(r[3] for r in after) > 0.5


def test_model_stop_after_the_step_aborts_into_the_hold():
  # the departure mask keeps only the planner's standstill stop out: with the model's own stop bit the planner's stop re-enters the
  # service, the launch ends and the service holds
  t_m = T_STEP + 0.15
  rows = drive(model_stop=lambda t: t >= t_m, should_stop=lambda t: t < T_GO or t >= t_m,
               a_target=lambda t: GO if T_GO <= t < t_m else -0.3)
  late = [r for r in rows if r[0] >= t_m + 0.05]
  assert not any(r[4] for r in late)
  assert all(r[1] == LongCtrlState.stopping for r in late), "the model's stop takes the launch back"
  assert all(r[2] in (Phase.APPROACH_GLIDE, Phase.RAMP_TO_HOLD, Phase.HOLD) for r in late), "with the service's authority"
  assert rows[-1][3] == pytest.approx(P.A_HOLD_SECURE)


def test_lead_braking_to_a_stop_after_the_step_aborts_into_a_secure_hold():
  # the 2232 @4940 class: the lead brakes to a stop again 0.1 s after the step inside the latch's 5 m: the first braking frame ends
  # the launch, `stopping` returns on the next frame, the service holds at the safety rate, and no positive command follows
  gap0 = 4.0
  t_brk = T_STEP + 0.1
  v_brk = min(t_brk - T_DEP, 1.5)
  t_stop = t_brk + v_brk / 2.5

  def lv(t):
    return min(max(t - T_DEP, 0.0), 1.5) if t < t_brk else max(v_brk - 2.5 * (t - t_brk), 0.0)

  def gap(t):
    if t < t_brk:
      return gap0 + travel(t - T_DEP)
    tau = min(t, t_stop) - t_brk
    return gap0 + travel(t_brk - T_DEP) + v_brk * tau - 1.25 * tau * tau
  assert gap(t_stop) < 5.0
  rows = drive(gap=gap, lead_v=lv, lead_a=lambda t: -2.5 if t_brk <= t < t_stop else 0.0)
  k = k_step(rows)
  kb = next(k for k, r in enumerate(rows) if r[0] >= t_brk - 1e-9)
  assert rows[k + 1][1] == LongCtrlState.starting
  assert all(r[4] for r in rows[k:kb]) and not any(r[4] for r in rows[kb:]), "the launch must end on the first braking frame"
  assert all(r[1] == LongCtrlState.stopping for r in rows[kb + 1:]), "aborted into `stopping`"
  assert all(r[2] in (Phase.RAMP_TO_HOLD, Phase.HOLD) for r in rows[kb + 1:]), "with the service holding"
  assert all(b[3] <= a[3] + 1e-9 for a, b in zip(rows[kb:], rows[kb + 1:], strict=False)), "no positive pulse after the abort"
  assert rows[-1][3] == pytest.approx(P.A_HOLD_SECURE)


def test_track_switch_after_the_step_aborts_into_the_hold():
  t_v = T_STEP + 0.1
  rows = drive(track=lambda t: 7 if t < t_v else 8)
  assert any(r[5] is not None for r in rows)
  assert not any(r[4] for r in rows if t_v + DT <= r[0] < t_v + 0.15)   # (a moving replacement may earn its own step later)
  k = next(k for k, r in enumerate(rows) if r[0] >= t_v - 1e-9)
  assert rows[k][1] == LongCtrlState.stopping and rows[k][2] in (Phase.RAMP_TO_HOLD, Phase.HOLD), "the launch is aborted into the hold"


def test_replacement_stopped_lead_aborts_the_launch_into_the_hold():
  # Astra review P1 (phase A): 0.25 s after the step the departing track is replaced by a STOPPED one at 5.1 m (beyond the latch's
  # 5 m) while the planner keeps its go: the launch must not stay in an at-rest 'starting' without service authority
  t_sw = T_STEP + 0.25
  rows = drive(seconds=11.0, gap=lambda t: GAP0 + travel(t - T_DEP) if t < t_sw else 5.1,
               lead_v=lambda t: min(max(t - T_DEP, 0.0), 1.5) if t < t_sw else 0.0, track=lambda t: 7 if t < t_sw else 8)
  k = k_step(rows)
  assert rows[k + 1][1] == LongCtrlState.starting
  late = [r for r in rows if r[0] >= t_sw + 0.05]
  assert not any(r[4] for r in late)
  assert all(r[1] == LongCtrlState.stopping for r in late), "an aborted launch must leave 'starting'"
  assert all(r[2] in (Phase.RAMP_TO_HOLD, Phase.HOLD) for r in late), "the service holds the aborted launch"
  assert rows[-1][3] == pytest.approx(P.A_HOLD_SECURE)


def test_step_is_one_shot_and_service_scoped():
  rows = drive()
  assert len([r for r in rows if r[5] is not None]) == 1, "the stepped command is reported on its step frame only"
  assert all(r[2] == Phase.INACTIVE for r in rows[k_step(rows):]), "the departing lead never re-enters the service"


def test_ceiling_is_final_after_the_low_speed_slew(monkeypatch):
  # Astra plan review P1: a ceiling before the slew loses to max(request, prev - brake_step): prev 0.70, L 0.10 -> 0.69. The
  # ceiling is the last writer: the frame L drops to 0.10 carries 0.10, and its recovery rises at J_GO at most
  # (the lead rests 2 m further back than GAP0 and the far-stopped-lead crawl cap is stubbed: no legacy cap binds before the ceiling)
  monkeypatch.setattr(longcontrol, "far_stopped_lead_crawl_accel_cap", lambda *a, **k: 9.0)
  t_dip = T_STEP + 0.6
  lv = lambda t: 0.40 if t_dip <= t < t_dip + 0.3 else min(max(t - T_DEP, 0.0), 1.5)  # noqa: E731
  rows = drive(lead_v=lv, gap=lambda t: GAP0 + 2.0 + travel(t - T_DEP))
  kd = next(k for k, r in enumerate(rows) if r[0] >= t_dip - 1e-9 and r[3] < rows[k - 1][3] - 0.05)
  assert rows[kd - 1][3] >= 0.70 - 1e-9 and rows[kd - 1][1] == LongCtrlState.starting
  assert rows[kd][3] == pytest.approx(pull(0.40, 0.0), abs=1e-9), "the ceiling binds on the frame L drops"
  rec = [r[3] for r in rows[kd:] if r[0] < t_dip + 1.0]
  assert all(b - a <= J_UP + 1e-9 for a, b in zip(rec, rec[1:], strict=False)), "the recovery rises at J_GO at most"


def test_no_speed_only_hand_over():
  # Astra plan review P2: the prototype ended the bound at vEgo >= 1.25 whatever the gap (gap 3.4, lead 1.4 m/s: +0.8 passed)
  f = Follow(lead_v=lead_ramp(1.0, 1.4))
  while f.lc.long_control_state != LongCtrlState.pid or f.v < 0.2:
    f.step()
  assert f.lc._lf_cap is not None
  f.v, f.a, f.xl = 1.30, 0.8, f.x + 3.4   # (the isolated clause: ego 1.3 m/s at 3.4 m behind a 1.4 m/s lead)
  rows = f.run(1.0)[-100:]
  assert all(r['cap'] is not None for r in rows), "no hand-over without room"
  assert all(r['wire'] <= pull(r['lv'], r['v']) + 1e-9 for r in rows[1:])
  assert max(r['v'] for r in rows) > 1.25


def test_follow_hands_over_with_room_and_caught_up():
  f = Follow(lead_v=lead_ramp(0.4, 1.2))
  rows = f.run(25.0)
  k = k_step(rows)
  bound = [r for r in rows[k:] if r['cap'] is not None]
  end = len(bound) + k
  assert end < len(rows), "the bound ends"
  free = next(i for i in range(k, end) if rows[i]['lv'] - rows[i]['v'] <= stopping_flags.LF_CAUGHT_UP_MPS
              and rows[i]['d'] > departing_lead_release_gap(max(rows[i]['lv'] - rows[i]['v'], 0.0), False))
  assert all(r['wire'] <= pull(r['lv'], r['v']) + 1e-9 for r in rows[k + 1:free])
  assert min(r['gap'] for r in rows[k:end]) >= GAP0 - 0.05, "the bounded ego never closes on the departing lead"
  assert all(b['wire'] - a['wire'] <= J_UP + 1e-9 for a, b in zip(rows[free:end], rows[free + 1:end + 1], strict=False) if b['v'] >= 1.2)


def test_dropout_relaxes_the_ceiling_at_the_go_rate_and_the_track_rebinds():
  t_d = T_STEP + 2.0
  f = Follow(lead_v=lead_ramp(0.4, 1.2), lead=lambda t: not t_d <= t < t_d + 0.4)
  rows = f.run(T_STEP + 3.0)
  drop = [r for r in rows if t_d <= r['t'] < t_d + 0.4]
  assert all(r['cap'] is not None for r in drop), "a dropout is not a hand-over"
  assert all(b['cap'] - a['cap'] <= J_UP + 1e-9 for a, b in zip(drop, drop[1:], strict=False)), "relaxes at J_GO at most"
  back = next(r for r in rows if r['t'] >= t_d + 0.4)
  assert back['cap'] == pytest.approx(pull(back['lv'], back['v']), abs=1e-9), "the same track re-binds at once"


@pytest.mark.parametrize("end", ["gas", "disengage"])
def test_driver_and_disengage_end_the_bound_at_once(end):
  t_e = T_STEP + 1.0
  kw = {"gas": lambda t: t >= t_e} if end == "gas" else {"active": lambda t: t < t_e}
  f = Follow(lead_v=lead_ramp(0.4, 1.2), **kw)
  rows = f.run(T_STEP + 1.5)
  assert any(r['cap'] is not None for r in rows if r['t'] < t_e)
  assert all(r['cap'] is None for r in rows if r['t'] >= t_e)


def test_renewed_rest_ends_the_bound_and_a_second_go_re_arms():
  # stop-go-stop-go: the lead departs, stops again 2 s later, departs again 4 s after that; the first bound ends with the renewed
  # rest under the service's hold, and the second launch earns its own step and a fresh ceiling
  def lv(t):
    if t < T_DEP + 2.0:
      return min(max(t - T_DEP, 0.0) * 0.5, 1.0)
    if t < T_DEP + 6.0:
      return max(1.0 - 1.5 * (t - T_DEP - 2.0), 0.0)
    return min((t - T_DEP - 6.0) * 0.5, 1.0)
  t_go2 = T_DEP + 6.5
  f = Follow(lead_v=lv, should_stop=lambda t: t < T_GO or T_DEP + 2.3 <= t < t_go2,
             a_target=lambda t: GO if T_GO <= t < T_DEP + 2.3 or t >= t_go2 else -0.3)
  rows = f.run(T_DEP + 12.0)
  rest = [r for r in rows if T_DEP + 4.5 <= r['t'] < t_go2 and r['v'] <= 0.0 and r['state'] == LongCtrlState.stopping]
  assert rest and rest[-1]['cap'] is None, "the renewed rest under the hold ends the first bound"
  assert min(r['gap'] for r in rows) >= 3.0
  second = [r for r in rows if r['t'] >= t_go2]
  k2 = next(i for i, r in enumerate(second) if r['cap'] is not None)
  assert second[k2]['cap'] == pytest.approx(pull(second[k2]['lv'], second[k2]['v']), abs=1e-9), "a fresh ceiling"
  assert any(r['state'] == LongCtrlState.starting for r in second[k2:])


@pytest.mark.parametrize("live_flag", LIVE_FLAGS)
def test_launch_with_each_live_flag_off(monkeypatch, live_flag):
  monkeypatch.setattr(stopping_flags, live_flag, False)
  on = drive()
  monkeypatch.setattr(stopping_flags, "LAUNCH_FOLLOW", False)
  off = drive()
  k = k_step(on)
  assert on[:k] == off[:k]
  assert on[k + 1][1] == LongCtrlState.starting and on[k + 1][4]
  assert all(r[3] <= r[6] + 1e-9 for r in on[k + 1:] if r[6] is not None)
  assert all(a[3] >= b[3] - 1e-9 for a, b in zip(on, off, strict=True))


def test_lead_pull_matches_the_documented_law():
  assert pull(1.0, 0.2, 0.5) == pytest.approx(0.5 + 1.0 - 0.2 - 0.3 - 0.1)
  assert pull(1.0, 0.2, -0.5) == pytest.approx(1.0 - 0.2 - 0.3 - 0.1)
  assert math.isclose(pull(0.3, 0.0), 0.0, abs_tol=1e-12)
