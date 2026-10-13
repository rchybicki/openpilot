"""GAP_AWARE_TERMINAL (cycle 2026-10-12 WS REST): the creep guard arms below V_GUARD only once the service's sticky
terminal_ready latch is set (distance to the stop point within the guard's stopping distance, v <= V_DESCENT_START, a rise
while waiting, or a fail-safe line)."""
from dataclasses import replace

import pytest

from openpilot.selfdrive.controls.lib import stopping_flags
from openpilot.selfdrive.controls.lib.stopping_service import GAT_EVENT_LINES, GOV_REST_BASE_M, Phase, StoppingService
from openpilot.selfdrive.controls.lib.tests.test_stopping_service import make_signals
from openpilot.selfdrive.controls.lib.tests.test_longcontrol_fast_release import DummyCarState, DummyFrogPilotToggles
from openpilot.selfdrive.controls.tests.test_stopping_stage1 import force, lc  # noqa: F401 (fixture)

ISD = 0.3


P = StoppingService().p


def d_arm(v):
  return P.GAT_T_S * v + v * v / (2.0 * -stopping_flags.A_GUARD)


def anchor(isd=ISD):
  return GOV_REST_BASE_M + max(isd, 0.0)


@pytest.fixture(autouse=True, params=[True, False], ids=['landing', 'no_landing'])
def flags(monkeypatch, request):
  # every test with DELIVERED_LANDING on (the car since 9e0b3c4e82) and off (its revert)
  monkeypatch.setattr(stopping_flags, 'GAP_AWARE_TERMINAL', True)
  monkeypatch.setattr(stopping_flags, 'SERVICE_APPROACH_LAW', 'governor')
  monkeypatch.setattr(stopping_flags, 'DELIVERED_LANDING', request.param)


def sstep(svc, v, gap, *, dts=-1., isd=ISD, track_age=5., a_coast=0., gap_source=None, hold_outward=False, dropout=False,
          should_stop=True, latch=True, lead_v=0., msd=-1., a_ego=-.25):
  sig = replace(make_signals(d_gap=gap, a_coast=a_coast, latch=latch, dropout=dropout, gap_source=gap_source,
                             hold_outward=hold_outward), track_age_s=track_age)
  return svc.update(engaged=True, v_ego=v, a_ego=a_ego, a_target=-.25, should_stop=should_stop, dts_planner=dts,
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


def test_a_creep_carried_wait_arms_on_the_stall_line():
  # 22a2_183.39: waiting at 0.71-0.80 m/s, the 2nd-gear creep held aEgo at -0.16..+0.04 for 1.2 s; arm after T_GAT_STALL of it
  n = round(stopping_flags.T_GAT_STALL / .01)
  svc = StoppingService()
  for _ in range(n - 1):
    sstep(svc, .75, 9., a_ego=-.10)
  assert not svc.terminal_ready
  sstep(svc, .75, 9., a_ego=-.20)     # one braking frame restarts the dwell
  for _ in range(n - 1):
    sstep(svc, .75, 9., a_ego=-.14)
  assert not svc.terminal_ready
  sstep(svc, .75, 9., a_ego=-.14)
  assert svc.terminal_ready_reason == 'stall' and 'stall' not in GAT_EVENT_LINES   # the guard's ordinary ramp, not a step
  svc = StoppingService()
  for _ in range(2 * n):
    sstep(svc, .75, 9., a_ego=stopping_flags.A_GAT_STALL)   # braking at the threshold is not a stall
  assert not svc.terminal_ready
  svc = StoppingService()
  for _ in range(2 * n):
    sstep(svc, 1.4, 9., a_ego=0.)     # above V_GUARD without a wait: nothing is tracked
  sstep(svc, 1.29, 12., a_ego=0.)
  assert not svc.terminal_ready


def test_the_stall_dwell_is_re_earned_after_a_release():
  n = round(stopping_flags.T_GAT_STALL / .01)
  svc = StoppingService()
  for _ in range(n):
    sstep(svc, .75, 9., a_ego=-.10)
  assert svc.terminal_ready_reason == 'stall'
  sstep(svc, 1.0, 9., should_stop=False, latch=False, lead_v=1.5)   # RELEASE clears the latch and the dwell
  assert svc.phase == Phase.RELEASE and svc._wait_stall_t == 0.
  sstep(svc, 1.0, 9., a_ego=-.10)
  assert svc.phase == Phase.APPROACH_GLIDE and not svc.terminal_ready


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


@pytest.mark.parametrize('a_ego', [-.25, -.05])   # -0.05: a creep-carried wait (the stall line)
def test_service_output_is_identical_with_the_flag_off(monkeypatch, a_ego):
  def run():
    svc, out = StoppingService(), []
    for i in range(140):
      v = max(1.25 - i * .008, 0.)
      out.append(sstep(svc, v, 4.3 + 2.4 * v, dts=2.4 * v, a_ego=a_ego).accel)
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


def test_the_guard_arms_before_the_landing_ease_and_only_then_takes_its_offset(lc, monkeypatch):  # noqa: F811
  # both flags (the car with this change): a far cold approach (no distance arm) arms at v <= V_DESCENT_START, above V_LAND, so the
  # landing ease (below V_LAND) can only ever shift an armed guard; before the arm the wire is the service's own command
  monkeypatch.setattr(stopping_flags, 'DELIVERED_LANDING', True)
  monkeypatch.setattr(stopping_flags, 'FLAT_LANDING', True)
  svc = lc._service_shadow_svc
  force(lc, monkeypatch, -.3)
  for _ in range(150):
    step(lc, v=1.5, gap=13.5, a_ego=-.3)
  rows, v, wire = [], 1.2, -.3
  while v > .12:
    wire = step(lc, v=v, gap=12. + v, a_ego=wire)   # the car delivers the previous wire (flat road, no creep: a_coast ~0)
    rows.append((v, wire, lc._final_floor_armed, svc.landing_up, svc.terminal_ready_reason))
    v -= .004
  armed = [i for i, r in enumerate(rows) if r[2]]
  eased = [i for i, r in enumerate(rows) if r[3] is not None]
  assert armed and eased, 'the approach reaches both the arm and the landing ease'
  assert rows[armed[0]][4] == 'v_low' and rows[armed[0]][0] <= svc.p.V_DESCENT_START + 1e-9
  assert armed[0] < eased[0] and rows[eased[0]][0] < stopping_flags.V_LAND
  assert all(rows[i][2] for i in eased)   # the offset only ever meets an armed guard


def test_a_lift_above_v_guard_while_waiting_arms_at_todays_level(lc, monkeypatch):  # noqa: F811
  # Astra review HIGH 1: waiting at 1.28 m/s behind a still-confirmed stopped lead, creep / a planner go lifts the car to 1.34 m/s;
  # today's guard (armed at 1.29) holds A_GUARD up to V_FLOOR, so the waiting one must arm there at that level, not lose the window
  force(lc, monkeypatch, -.25)
  gap = 12.
  for _ in range(150):
    step(lc, v=1.5, gap=gap, a_ego=-.25)
  for _ in range(30):
    assert step(lc, v=1.28, gap=gap, a_ego=-.25) == -.25 and not lc._final_floor_armed
    gap -= .0128
  wires = []
  for v in (1.30, 1.32, 1.34, 1.36):
    wires.append(step(lc, v=v, gap=gap, a_ego=.2))
    gap -= v * .01
  assert lc._final_floor_armed and lc._service_shadow_svc.terminal_ready_reason == 'v_rise'
  assert wires[-1] == pytest.approx(stopping_flags.A_GUARD)


def test_a_stall_while_waiting_arms_with_todays_ramp(lc, monkeypatch):  # noqa: F811
  # the stall line (22a2_183.39): a wait the creep carries arms the guard from the request at J_GUARD, never a step
  force(lc, monkeypatch, -.25)
  for _ in range(150):
    step(lc, v=1.5, gap=12., a_ego=-.25)
  for _ in range(20):
    assert step(lc, v=.75, gap=12., a_ego=-.25) == -.25
  wires = [step(lc, v=.75, gap=12., a_ego=-.05) for _ in range(round(stopping_flags.T_GAT_STALL / .01) + 40)]
  armed = [i for i, w in enumerate(wires) if w < -.25]
  assert armed and lc._final_floor_armed and lc._service_shadow_svc.terminal_ready_reason == 'stall'
  k = armed[0]
  assert k >= round(stopping_flags.T_GAT_STALL / .01) - 1
  assert all(b >= a - stopping_flags.J_GUARD * .01 - 1e-9 for a, b in zip(wires[k - 1:], wires[k:], strict=False))
  assert wires[-1] == pytest.approx(stopping_flags.A_GUARD)


def test_a_first_entry_event_arms_with_todays_ramp(lc, monkeypatch):  # noqa: F811
  # Astra review MEDIUM 3: the first eligible frame below V_GUARD never waited, so an event line there (a young track) arms like
  # today's guard: from the request at J_GUARD, never a step
  force(lc, monkeypatch, -.25)
  for _ in range(150):
    step(lc, v=1.5, gap=5.6, a_ego=-.25)
  svc = lc._service_shadow_svc
  monkeypatch.setattr(svc, '_wait_v_min', float('inf'))
  orig = svc.update

  def young(**kw):   # the lead's identity reads 0.4 s old on the first frame below V_GUARD
    kw['signals'] = replace(kw['signals'], track_age_s=.4)
    return orig(**kw)
  monkeypatch.setattr(svc, 'update', young)
  wire = step(lc, v=1.25, gap=5.6, a_ego=-.25)
  assert lc._final_floor_armed and svc.terminal_ready_reason == 'identity'
  assert wire == pytest.approx(-.25 - stopping_flags.J_GUARD * .01)
