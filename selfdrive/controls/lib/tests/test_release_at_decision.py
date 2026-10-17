"""RELEASE_AT_DECISION (stopping_flags, cycle 2026-10-13 LAUNCH phase A; cyc_1012/launch research, 114 holds).

At rest behind a departing lead the service decides HOLD -> RELEASE on the planner go plus the lead's earned Doppler departure; today
the command then ramps -0.70 -> 0 at J_GO (0.6 s) while LongControl stays pinned in `stopping` (StopReq: the ESC holds the car), and the
arbiter's close-stopped-lead latch holds `stopping` ~0.3 s more. With the flag the at-rest RELEASE steps the command to the release
level on the first frame that passes the guard (earned Doppler departure, live gap, no dropout, the planner's go held RELEASE_STEP_GO_S
without a stop request, lead neither braking nor confirmed stopped), and the latch counts it as its departing-lead release while the
same lead keeps moving: `stopping` ends on the next frame and `starting` rises at the departing-lead fast-release rate. Every other
release keeps today's ramp and arbiter."""
import pytest

from openpilot.selfdrive.controls.lib import stopping_flags
from openpilot.selfdrive.controls.lib.longcontrol import LongControl, LongCtrlState
from openpilot.selfdrive.controls.lib.stopping_service import Phase, ServiceParams
from openpilot.selfdrive.controls.lib.tests.test_longcontrol_fast_release import DummyCarParams, DummyCarState, DummyFrogPilotToggles
from openpilot.selfdrive.controls.lib.tests.test_release_end_rehold import GO, LIMITS, Radar, travel

P = ServiceParams()
DT = 0.01
GAP0 = 4.4       # rest gap behind the lead (car launches: d0 p50 4.65 m)
T_DEP = 6.0      # the lead starts at 1 m/s^2 (up to 1.5 m/s); the ego stays at rest (recorded-input replay semantics)
T_GO = 6.8       # the planner's go (shouldStop 0, aTarget +0.42): car launches decide p50 0.21 s after vLead > 0.3 held 0.2 s
T_STEP = T_GO + P.RELEASE_STEP_GO_S   # the go has held long enough: the step frame


@pytest.fixture(autouse=True)
def _flag_on(monkeypatch):
  monkeypatch.setattr(stopping_flags, "RELEASE_AT_DECISION", True)


def drive(seconds=9.0, gap=None, lead_v=None, lead_a=lambda t: 0.0, lead=lambda t: True, track=lambda t: 7,
          should_stop=lambda t: t < T_GO, a_target=lambda t: GO if t >= T_GO else -0.3):
  """LongControl (LIVE service ownership) at rest behind a lead that departs at T_DEP; rows per frame:
  (t, state, service phase, wire, latch released, the service's stepped command)."""
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
                    frogpilot_toggles=toggles, lead_status=lead(t), lead_v=lv, lead_d_rel=d, lead_a=lead_a(t), lead_track_id=track(t))
    rows.append((t, lc.long_control_state, lc._service_shadow_svc.phase, float(out), lc._launch_release,
                 lc._service_shadow_svc.launch_cmd))
  return rows


def first_exit(rows, after):
  return next(k for k, r in enumerate(rows) if r[0] >= after and r[1] != LongCtrlState.stopping)


def test_departure_leaves_stopping_on_the_frame_after_the_step():
  rows = drive()
  k_dec = next(k for k, r in enumerate(rows) if r[2] == Phase.RELEASE)
  k = next(k for k, r in enumerate(rows) if r[5] is not None)
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
  monkeypatch.setattr(stopping_flags, "RELEASE_AT_DECISION", False)
  off = drive()
  k_on = next(k for k, r in enumerate(on) if r[5] is not None)
  assert on[:k_on] == off[:k_on], "identical up to the step frame"
  assert all(r[4] is False and r[5] is None for r in off)
  gain = (first_exit(off, T_GO) - first_exit(on, T_GO)) * DT
  assert gain >= -P.A_HOLD_SECURE / P.J_GO - P.RELEASE_STEP_GO_S - 0.05, f"StopReq ends only {gain:.2f} s earlier"
  # the launch command is never behind today's: every frame's wire is at least the flag-off wire
  assert all(a[3] >= b[3] - 1e-9 for a, b in zip(on, off, strict=True))


@pytest.mark.parametrize("ticks", [1, 3])
def test_brief_planner_go_keeps_todays_ramp(monkeypatch, ticks):
  # the brief-go race (227e 241.2, 20c0 2458.5): the planner's go lasts 1-3 ticks (50 ms each) and the stop request returns while
  # the lead keeps creeping away; today's ramp absorbs it under StopReq, and so must the flag
  t_end = T_GO + 0.05 * ticks
  kw = dict(should_stop=lambda t: not T_GO <= t < t_end, a_target=lambda t: GO if T_GO <= t < t_end else -0.3)
  on = drive(**kw)
  monkeypatch.setattr(stopping_flags, "RELEASE_AT_DECISION", False)
  assert on == drive(**kw)
  assert all(r[1] == LongCtrlState.stopping for r in on if r[0] >= T_GO)


def test_range_drift_go_keeps_todays_ramp_and_re_hold(monkeypatch):
  # the 4904 class: standstill range drift (Doppler 0) arms gap growth, the planner goes toward a stopped lead: no step, no early exit
  def scen():
    return drive(seconds=12.0, gap=lambda t: 4.6 + 0.1 * min(max(t - 3.0, 0.0), 5.0), lead_v=lambda t: 0.0,
                 should_stop=lambda t: t < 8.5, a_target=lambda t: GO if t >= 8.5 else -0.3)
  on = scen()
  monkeypatch.setattr(stopping_flags, "RELEASE_AT_DECISION", False)
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
  monkeypatch.setattr(stopping_flags, "RELEASE_AT_DECISION", False)
  assert on == drive(**kw)
  assert all(r[5] is None and r[4] is False for r in on)


def test_latch_release_ends_when_the_lead_brakes_to_a_stop_again():
  # a queue lead that moves off and brakes to a stop again (-2.5 m/s^2) 0.1 s after the step, inside the latch's 5 m: the
  # first braking frame ends the latch release, `stopping` returns on the next frame and the service holds again
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
  k = next(k for k, r in enumerate(rows) if r[5] is not None)
  kb = next(k for k, r in enumerate(rows) if r[0] >= t_brk - 1e-9)
  assert rows[k + 1][1] == LongCtrlState.starting
  assert all(r[4] for r in rows[k:kb]) and not any(r[4] for r in rows[kb:]), "the latch release must end on the first braking frame"
  assert all(r[1] == LongCtrlState.stopping for r in rows[kb + 1:]), "the close-stopped-lead latch must hold again"
  assert max(r[3] for r in rows[k:]) <= 0.030 * (kb - k) + 1e-9
  assert rows[-1][3] == pytest.approx(P.A_HOLD_SECURE)


@pytest.mark.parametrize("veto", ["track_switch", "stop_request_returns"])
def test_latch_release_ends_on_a_track_switch_or_a_stop_request(veto):
  t_v = T_STEP + 0.1
  kw = {"track": lambda t: 7 if t < t_v else 8} if veto == "track_switch" else \
       {"should_stop": lambda t: t < T_GO or t >= t_v, "a_target": lambda t: GO if T_GO <= t < t_v else -0.3}
  rows = drive(**kw)
  assert any(r[5] is not None for r in rows)
  assert not any(r[4] for r in rows if t_v + DT <= r[0] < t_v + 0.15)   # (a moving replacement may earn its own step later)
  k = next(k for k, r in enumerate(rows) if r[0] >= t_v - 1e-9)
  assert rows[k][1] == LongCtrlState.stopping and rows[k][2] in (Phase.RAMP_TO_HOLD, Phase.HOLD), "the launch is aborted into the hold"


def test_replacement_stopped_lead_aborts_the_launch_into_the_hold():
  # Astra review P1: 0.25 s after the step the departing track is replaced by a STOPPED one at 5.1 m (beyond the latch's 5 m) while the
  # planner keeps its go: the launch must not stay in an at-rest 'starting' without service authority; it returns to 'stopping' with
  # the service holding
  t_sw = T_STEP + 0.25
  kw = dict(seconds=11.0, gap=lambda t: GAP0 + travel(t - T_DEP) if t < t_sw else 5.1,
            lead_v=lambda t: min(max(t - T_DEP, 0.0), 1.5) if t < t_sw else 0.0, track=lambda t: 7 if t < t_sw else 8)
  rows = drive(**kw)
  k = next(k for k, r in enumerate(rows) if r[5] is not None)
  assert rows[k + 1][1] == LongCtrlState.starting
  late = [r for r in rows if r[0] >= t_sw + 0.05]
  assert not any(r[4] for r in late)
  assert all(r[1] == LongCtrlState.stopping for r in late), "an aborted launch must leave 'starting'"
  assert all(r[2] in (Phase.RAMP_TO_HOLD, Phase.HOLD) for r in late), "the service holds the aborted launch"
  assert rows[-1][3] == pytest.approx(P.A_HOLD_SECURE)
  peak = max(r[3] for r in rows[k:])
  assert peak <= 0.030 * round((t_sw - rows[k][0]) / DT + 2) + 1e-9
  # (flag off with this timing: the RELEASE ends at 7.37 s before the replacement reads confirmed stopped, LongControl enters
  # 'starting' and the service's later HOLD has no authority either -- a latent base defect, reported with this change)


def test_step_is_one_shot_and_service_scoped():
  rows = drive()
  steps = [r for r in rows if r[5] is not None]
  assert len(steps) == 1, "the stepped command is reported on its step frame only"
  k = next(k for k, r in enumerate(rows) if r[5] is not None)
  assert all(r[2] == Phase.INACTIVE for r in rows[k:]), "the departing lead never re-enters the service"
