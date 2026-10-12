"""GAP_AWARE_TERMINAL (cycle 2026-10-12 WS REST): the creep guard arms below V_GUARD only once the service's sticky
terminal_ready latch is set (distance to the stop point within the guard's stopping distance, v <= V_DESCENT_START, a rise
while waiting, or a fail-safe line)."""
from dataclasses import replace

import pytest

from openpilot.selfdrive.controls.lib import stopping_flags
from openpilot.selfdrive.controls.lib.stopping_service import GOV_REST_BASE_M, Phase, StoppingService
from openpilot.selfdrive.controls.lib.tests.test_stopping_service import make_signals
from openpilot.selfdrive.controls.lib.tests.test_longcontrol_fast_release import DummyCarState, DummyFrogPilotToggles
from openpilot.selfdrive.controls.tests.test_stopping_stage1 import force, lc  # noqa: F401 (fixture)

ISD = 0.3


P = StoppingService().p


def d_arm(v):
  return P.GAT_T_S * v + v * v / (2.0 * -stopping_flags.A_GUARD)


def anchor(isd=ISD):
  return GOV_REST_BASE_M + max(isd, 0.0)


@pytest.fixture(autouse=True)
def flags(monkeypatch):
  monkeypatch.setattr(stopping_flags, 'GAP_AWARE_TERMINAL', True)
  monkeypatch.setattr(stopping_flags, 'SERVICE_APPROACH_LAW', 'governor')


def sstep(svc, v, gap, *, dts=-1., isd=ISD, track_age=5., a_coast=0., gap_source=None, hold_outward=False, dropout=False,
          should_stop=True, latch=True, lead_v=0., msd=-1.):
  sig = replace(make_signals(d_gap=gap, a_coast=a_coast, latch=latch, dropout=dropout, gap_source=gap_source,
                             hold_outward=hold_outward), track_age_s=track_age)
  return svc.update(engaged=True, v_ego=v, a_ego=-.25, a_target=-.25, should_stop=should_stop, dts_planner=dts,
                    planner_min_limit=-3.5, signals=sig, lead_status=True, lead_v=lead_v, increased_stopped_distance=isd,
                    wire_accel=svc._last_cmd, dt=.01, model_stop_d=msd)


def test_cold_approach_waits_then_arms_on_distance():
  svc = StoppingService()
  for v in (1.25, 1.15, 1.05, 1.0):
    sstep(svc, v, anchor() + d_arm(v) + .5)
    assert svc.phase == Phase.APPROACH_GLIDE and not svc.terminal_ready
  sstep(svc, .98, anchor() + d_arm(.98) - .005)
  assert svc.terminal_ready and svc.terminal_ready_reason == 'distance'
  sstep(svc, .97, 20.)   # sticky: a later far reading does not clear it
  assert svc.terminal_ready


@pytest.mark.parametrize('v', [1.29, 1.2, 1.0, .7, .55])
@pytest.mark.parametrize('inside', [.05, 1.0])
def test_hot_approach_arms_at_once(v, inside):
  # however deep inside the guard's distance: on the first frame below V_GUARD it is today's hot arrival (ramped arm), not an event
  svc = StoppingService()
  sstep(svc, v, anchor() + d_arm(v) - inside)
  assert svc.terminal_ready_reason == 'distance'


def test_nearer_planner_target_arms_earlier_and_no_target_uses_the_gap():
  svc = StoppingService()
  sstep(svc, 1.0, 9., dts=d_arm(1.0) - .01)
  assert svc.terminal_ready_reason == 'distance'
  svc = StoppingService()
  sstep(svc, 1.0, 9., dts=-1.)   # no planner target (228d_3425.02): the gap alone, far
  assert not svc.terminal_ready
  sstep(svc, 1.0, anchor() + d_arm(1.0), dts=-1.)
  assert svc.terminal_ready_reason == 'distance'


def test_a_target_appearing_inside_the_guard_distance_is_an_event():
  svc = StoppingService()
  sstep(svc, 1.0, 9.)
  sstep(svc, 1.0, 9., dts=d_arm(1.0) - .26)   # the planner's target jumps nearer
  assert svc.terminal_ready_reason == 'nearer'
  svc = StoppingService()
  sstep(svc, 1.0, 9.)
  sstep(svc, 1.0, 9., msd=d_arm(1.0) - .5)    # the e2e model's own stop point (distanceToStopTargetModel): arms, never an event
  assert svc.terminal_ready_reason == 'distance'
  svc = StoppingService()
  sstep(svc, 1.0, 9., msd=-1.)
  assert not svc.terminal_ready


def test_negative_isd_never_lowers_the_anchor_below_base():
  svc = StoppingService()
  sstep(svc, 1.0, GOV_REST_BASE_M + d_arm(1.0) - .01, isd=-1.0)
  assert svc.terminal_ready_reason == 'distance'


def test_low_speed_and_creep_rise_arm():
  svc = StoppingService()
  sstep(svc, .51, 9.)
  assert not svc.terminal_ready
  sstep(svc, .5, 9.)
  assert svc.terminal_ready_reason == 'v_low'
  svc = StoppingService()
  for v in (1.0, .9, .93):
    sstep(svc, v, 9.)
    assert not svc.terminal_ready   # a one-quantum flicker does not arm
  sstep(svc, .96, 9.)
  assert svc.terminal_ready_reason == 'v_rise'


def test_rise_above_v_guard_is_tracked_only_below_it():
  svc = StoppingService()
  for v in (1.6, 1.5, 1.4):  # above V_GUARD the guard cannot arm; no min is tracked
    sstep(svc, v, 12.)
  sstep(svc, 1.29, 12.)
  assert not svc.terminal_ready


@pytest.mark.parametrize('line,kw', [
  ('gap', dict(gap_source='held', hold_outward=False)),
  ('gap', dict(gap_source='held', hold_outward=False, dropout=True)),
  ('identity', dict(track_age=.2)),
  ('uphill', dict(a_coast=-.25)),
])
def test_fail_safe_lines_arm(line, kw):
  svc = StoppingService()
  sstep(svc, 1.0, 9., **kw)
  assert svc.terminal_ready and svc.terminal_ready_reason == line


def test_outward_hold_keeps_waiting():
  svc = StoppingService()
  sstep(svc, 1.0, 9., gap_source='held', hold_outward=True)
  assert not svc.terminal_ready


def test_legacy_law_and_monitor_arm(monkeypatch):
  svc = StoppingService()
  svc.phase = Phase.APPROACH_GLIDE
  svc._mon_triggered = True
  sstep(svc, 1.0, 9.)
  assert svc.terminal_ready_reason == 'monitor'
  monkeypatch.setattr(stopping_flags, 'SERVICE_APPROACH_LAW', 'glide')
  svc = StoppingService()
  sstep(svc, 1.0, 9.)
  assert svc.terminal_ready_reason == 'law'


def test_binding_floor_lane_arms():
  svc = StoppingService()
  # 1.25 m/s at 4.05 m: inside d_arm too, so check the order; a_kin/barrier bind deeper than a shallow planner
  sstep(svc, 1.25, 9.)
  assert not svc.terminal_ready
  svc._wait_v_min = 1.25
  r = sstep(svc, 1.25, 3.6, dts=9.)
  assert svc.terminal_ready and r.debug['a_barrier'] is not None
  assert svc.terminal_ready_reason == 'nearer'   # 3.6 m at 1.25 m/s: far inside d_arm = an event


def test_safety_line_is_set_when_only_a_floor_lane_binds(monkeypatch):
  svc = StoppingService()
  sstep(svc, 1.0, 9.)
  assert not svc.terminal_ready
  # make the distance line unreachable to isolate the end-of-frame safety line
  monkeypatch.setattr(svc, 'p', replace(svc.p, GAT_T_S=-100.))
  sstep(svc, 1.0, 3.3)   # barrier (3.1 m) binds far deeper than the planner's -0.25
  assert svc.terminal_ready_reason == 'safety'


def test_release_and_reset_clear_the_latch_and_it_is_re_earned():
  svc = StoppingService()
  sstep(svc, .5, 9.)
  assert svc.terminal_ready
  sstep(svc, 1.0, 9., should_stop=False, latch=False, lead_v=1.5)   # the stop is withdrawn, the lead leaves: RELEASE
  assert svc.phase == Phase.RELEASE and not svc.terminal_ready and svc.terminal_ready_reason is None
  sstep(svc, 1.0, 9.)                                    # re-asserted: waits again
  assert svc.phase == Phase.APPROACH_GLIDE and not svc.terminal_ready
  sstep(svc, .5, 9.)
  svc.reset()
  assert not svc.terminal_ready and svc._wait_v_min == float('inf')


def test_service_output_is_identical_with_the_flag_off(monkeypatch):
  def run():
    svc, out = StoppingService(), []
    for i in range(140):
      v = max(1.25 - i * .008, 0.)
      out.append(sstep(svc, v, 4.3 + 2.4 * v, dts=2.4 * v).accel)
    return out
  on = run()
  monkeypatch.setattr(stopping_flags, 'GAP_AWARE_TERMINAL', False)
  assert run() == on   # the latch is consumed only by the guard in LongControl


@pytest.mark.parametrize('isd', [0., .3, 1.])
def test_latch_is_never_unset_inside_the_reserve(isd):
  # property: on a clean approach the latch is set on the first frame at gap <= anchor + d_arm(v), whatever the planner target
  for v in (1.29, 1.1, .9, .7, .51):
    for slack in (-1e-6, -.2, -1.):
      for dts in (-1., 0., 1., 10.):
        svc = StoppingService()
        sstep(svc, v, anchor(isd) + d_arm(v) + slack, dts=dts, isd=isd)
        assert svc.terminal_ready, (v, slack, dts, isd)


# --- LongControl: the guard's arm waits for the latch ---------------------------------------------------------------------
def step(lc, *, v, gap, a_ego, lead=True):  # noqa: F811
  # test_stopping_stage1.step with the ego's measured decel matching the command (a_coast ~0: flat road, no creep)
  cs = DummyCarState(v_ego=v, a_ego=a_ego, brake_pressed=False, standstill=v == 0.)
  cs.gasPressed, cs.canValid, cs.canTimeout = False, True, False
  return lc.update(True, cs, -.6, False, -1., (-3.5, 2.), DummyFrogPilotToggles(),
                   lead_status=lead, lead_v=0., lead_d_rel=gap if lead else 0., lead_track_id=1 if lead else -1,
                   lead_model_prob=1. if lead else 0.)


def test_guard_waits_for_the_latch_then_arms_from_the_request(lc, monkeypatch):  # noqa: F811
  force(lc, monkeypatch, -.25)
  gap = 8.
  for _ in range(150):
    step(lc, v=1.5, gap=gap, a_ego=-.25)
  arm_gap = anchor(lc._service_shadow_svc._isd) + d_arm(1.0)   # the dummy toggles carry ISD 0
  waited = 0
  while True:
    wire = step(lc, v=1.0, gap=gap, a_ego=-.25)
    if lc._final_floor_armed:
      break
    assert wire == -.25   # below V_GUARD, short of the stop point: the planner's command stands
    gap -= .01   # the ego closes on the stopped lead at 1 m/s
    waited += 1
  assert waited > 100 and abs(gap - arm_gap) < .011
  assert lc._service_shadow_svc.terminal_ready_reason == 'distance'
  gap -= .01
  previous = wire
  assert wire == pytest.approx(-.26)   # the arming frame: the guard starts from the request
  for i in range(1, 40):
    wire = step(lc, v=1.0, gap=gap, a_ego=-.25)
    assert lc._final_floor_armed
    assert wire == pytest.approx(max(stopping_flags.A_GUARD, -.25 - (i + 1) * stopping_flags.J_GUARD * .01))
    assert wire <= previous
    previous = wire
    gap -= .01


def test_guard_arms_at_the_descent_speed_un_latched_by_distance(lc, monkeypatch):  # noqa: F811
  force(lc, monkeypatch, -.2)
  for _ in range(150):
    step(lc, v=1.5, gap=12., a_ego=-.2)
  gap = 12.
  for i in range(51):
    v = 1.0 - i * .01   # an ordinary slow-down, far from the stop point: never a rise
    step(lc, v=v, gap=gap, a_ego=-.2)
    assert lc._final_floor_armed == (v <= .5 + 1e-9)
    gap -= v * .01
  assert lc._service_shadow_svc.terminal_ready_reason == 'v_low'


def test_flag_off_guard_arms_at_v_guard_whatever_the_gap(lc, monkeypatch):  # noqa: F811
  monkeypatch.setattr(stopping_flags, 'GAP_AWARE_TERMINAL', False)
  force(lc, monkeypatch, -.25)
  for _ in range(150):
    step(lc, v=1.5, gap=12., a_ego=-.2)
  step(lc, v=1.25, gap=12., a_ego=-.25)
  assert lc._final_floor_armed and not lc._service_shadow_svc.terminal_ready


@pytest.mark.parametrize('flag', [True, False])
def test_lead_lost_while_waiting_gets_todays_ride_out_and_handover(lc, monkeypatch, flag):  # noqa: F811
  # round 2 (late-object grid 'lost'): today's guard is armed below V_GUARD and brakes a lost lead through its ride-out and the J_GO
  # handover; a waiting guard gets the same at once (A_GUARD), never more
  monkeypatch.setattr(stopping_flags, 'GAP_AWARE_TERMINAL', flag)
  force(lc, monkeypatch, -.25)
  for _ in range(150):
    step(lc, v=1.5, gap=12., a_ego=-.25)
  for _ in range(20):
    step(lc, v=1.0, gap=12., a_ego=-.25)
  assert lc._final_floor_armed == (not flag)
  wires = [step(lc, v=1.0, gap=12., a_ego=-.25, lead=False) for _ in range(150)]
  if flag:
    assert lc._service_shadow_svc.terminal_ready_reason in ('lead', 'gap')
  armed = [w for w in wires[:30] if w <= stopping_flags.A_GUARD + 1e-9]
  assert armed, 'the lost lead is braked at the guard level during the ride-out'
  assert not lc._final_floor_armed   # the ride-out ended: disarmed
  j_go = lc._service_shadow_svc.p.J_GO * .01
  assert all(b - a <= j_go + 1e-9 for a, b in zip(wires, wires[1:], strict=False))   # released no faster than the go rate


def test_an_event_while_waiting_arms_at_the_guard_level_at_once(lc, monkeypatch):  # noqa: F811
  force(lc, monkeypatch, -.25)
  for _ in range(150):
    step(lc, v=1.5, gap=12., a_ego=-.25)
  for _ in range(20):
    assert step(lc, v=1.0, gap=12., a_ego=-.25) == -.25
  # a confirmed car with a new track id cuts in nearer: the identity line, no J_GUARD ramp
  cs = DummyCarState(v_ego=1.0, a_ego=-.25, brake_pressed=False, standstill=False)
  cs.gasPressed, cs.canValid, cs.canTimeout = False, True, False
  wire = lc.update(True, cs, -.6, False, -1., (-3.5, 2.), DummyFrogPilotToggles(), lead_status=True, lead_v=0., lead_d_rel=6.,
                   lead_track_id=2, lead_model_prob=1.)
  assert lc._final_floor_armed and lc._service_shadow_svc.terminal_ready_reason in ('identity', 'gap', 'nearer')
  assert wire == pytest.approx(stopping_flags.A_GUARD)
