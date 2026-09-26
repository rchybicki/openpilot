"""Stage-1 final arbitration, strict latch, landing, and real Hyundai/Panda wire contracts."""
from dataclasses import replace
from types import SimpleNamespace

import pytest

from opendbc.car.hyundai.tests.test_can_bounds_fork import SCC12_ADDR, get_signal, make_cc, make_controller, make_cs
from opendbc.safety.tests import test_hyundai
from opendbc.safety.tests.libsafety import libsafety_py
from openpilot.selfdrive.controls.lib import stopping_flags
from openpilot.selfdrive.controls.lib.longcontrol import LongControl, LongCtrlState
from openpilot.selfdrive.controls.lib.stopping_service import Phase, StoppingService
from openpilot.selfdrive.controls.lib.tests.test_longcontrol_fast_release import DummyCarParams, DummyCarState, DummyFrogPilotToggles
from openpilot.selfdrive.controls.lib.tests.test_stopping_service import make_signals


@pytest.fixture
def lc(monkeypatch):
  monkeypatch.setattr(stopping_flags, 'FINAL_FLOOR', True)
  monkeypatch.setattr(stopping_flags, 'IDENTIFICATION_HOOK', False)
  cp = DummyCarParams()
  cp.longitudinalTuning.kpV = [0.0]
  control = LongControl(cp)
  control.long_control_state = LongCtrlState.pid
  return control


def step(lc, *, v=1.5, lead_v=0., gap=10., target=-.6, active=True, gas=False, brake=False, **kwargs):
  cs = DummyCarState(v_ego=v, a_ego=-.5, brake_pressed=brake, standstill=v == 0.)
  cs.gasPressed, cs.canValid, cs.canTimeout = gas, True, False
  return lc.update(active, cs, target, False, -1., (-3.5, 2.), DummyFrogPilotToggles(),
                   lead_status=True, lead_v=lead_v, lead_d_rel=gap, lead_track_id=1, lead_model_prob=1., **kwargs)


def arm(lc):
  for _ in range(150):
    step(lc)
  assert lc._service_signals.lead_confirmed_stopped and lc._final_floor_armed


def test_final_floor_after_every_service_writer(lc, monkeypatch):
  arm(lc)
  update = lc._service_shadow_svc.update
  # Real context and full LongControl arbitration; force the last service result to isolate
  # the seam from every possible upstream release writer, including future ones.
  for demand in [-.9, -.5, -.49, -.35, 0., -.8]:
    monkeypatch.setattr(lc._service_shadow_svc, 'update', lambda demand=demand, **kw: replace(update(**kw), accel=demand))
    wire = step(lc)
    assert wire == min(demand, -.5) == lc.last_output_accel
  assert lc._final_floor_bound_frames >= 3


@pytest.mark.parametrize('condition', ['gas', 'brake', 'inactive', 'speed', 'lead', 'wheel', 'mode', 'scope', 'flag', 'not_run', 'fault', 'exception', 'reset'])
def test_disarm_and_reearn(lc, monkeypatch, condition):
  arm(lc)
  previous = lc.last_output_accel
  if condition == 'reset':
    lc.reset()
  elif condition == 'exception':
    def fail(**kw):
      raise RuntimeError('injected service fault')
    monkeypatch.setattr(lc._service_shadow_svc, 'update', fail)
    assert step(lc) <= previous
    assert lc._service_signals is None
  else:
    kw = {'gas': dict(gas=True), 'brake': dict(brake=True), 'inactive': dict(active=False),
          'speed': dict(v=2.5), 'lead': dict(lead_v=.31), 'wheel': dict(v=0.), 'fault': dict(plan_valid=False)}.get(condition, {})
    if condition == 'mode':
      monkeypatch.setattr(stopping_flags, 'SERVICE_MODE', 'SHADOW')
    if condition == 'scope':
      lc._service_shadow_scope = False
    if condition == 'flag':
      monkeypatch.setattr(stopping_flags, 'FINAL_FLOOR', False)
    if condition == 'not_run':
      lc._service_live_disabled = True
    wire = step(lc, **kw)
    if condition == 'fault':
      assert wire <= previous
  assert not lc._final_floor_armed
  assert lc._final_floor_bound_frames == 0


@pytest.mark.parametrize('lead_v', [.30001, .4, .6])
def test_crawling_lead_never_arms(lc, lead_v):
  for _ in range(200):
    step(lc, lead_v=lead_v)
    assert not lc._final_floor_armed


def test_latch_flicker_departure_and_new_request(lc, monkeypatch):
  # Positive Doppler outside the strict window resets accumulated confirmation.
  for i in range(150):
    step(lc, lead_v=.31 if i % 20 == 0 else .29)
    assert not lc._final_floor_armed
  arm(lc)
  step(lc, lead_v=.31)
  assert not lc._service_signals.lead_confirmed_stopped and not lc._final_floor_armed  # first departure frame
  update = lc._service_shadow_svc.update
  monkeypatch.setattr(lc._service_shadow_svc, 'update', lambda **kw: replace(update(**kw), accel=-.35))
  for _ in range(100):
    assert step(lc) == -.35
    assert not lc._final_floor_armed
  assert lc._service_signals.lead_confirmed_stopped
  monkeypatch.setattr(lc._service_shadow_svc, 'update', lambda **kw: replace(update(**kw), accel=-.5))
  assert step(lc) == -.5 and lc._final_floor_armed


@pytest.mark.parametrize('attr', ['off', 'live'])
@pytest.mark.parametrize('recovery', [False, True])
def test_real_planner_safety_pursuit_and_recovery_arbitration(lc, monkeypatch, attr, recovery):
  monkeypatch.setattr(stopping_flags, 'ATTRIBUTED_SAFETY', attr)
  monkeypatch.setattr(stopping_flags, 'GOVERNOR_RECOVERY_BRAKE', recovery)
  arm(lc)
  debug = []
  update = lc._service_shadow_svc.update

  def capture(**kw):
    result = update(**kw)
    debug.append(result.debug)
    return result

  monkeypatch.setattr(lc._service_shadow_svc, 'update', capture)
  bound = []
  for i in range(250):
    wire = step(lc, v=1.5 if i < 100 else .8, target=-1. if i % 50 < 10 else -.1,
                a_target_trajectory=-.8 if i % 50 < 10 else -.1, fcw=i % 70 < 5)
    assert wire <= -.5 and lc._final_floor_armed and lc._service_live_owning
    if lc._final_floor_added > 0.:
      bound.append(debug[-1])
  assert bound
  assert any(r['a_phase'] > -.5 for r in bound)  # a real upstream release is blocked
  assert any(r['a_plan'] < -.5 for r in debug)  # planner binding really changes
  if attr == 'live':
    assert {r['attr_eligible'] for r in debug} == {False, True}
    assert any(r['attr_live'] and r['attr_released'] > 0. for r in bound)


def service_step(svc, v, wheel=False, a_target=None, **kwargs):
  return svc.update(engaged=True, v_ego=v, a_ego=-.5 if v else 0., a_target=a_target,
                    should_stop=True, dts_planner=10., planner_min_limit=-3.5,
                    signals=make_signals(d_gap=14., latch=True, wheel=wheel), lead_status=True, lead_v=0.,
                    wire_accel=svc._last_cmd, dt=.01, **kwargs)


@pytest.mark.parametrize('seed', [-.3, -.5, -.9])
def test_flat_landing_and_immediate_hold(monkeypatch, seed):
  monkeypatch.setattr(stopping_flags, 'FLAT_LANDING', True)
  svc = StoppingService()
  svc._last_cmd = seed
  service_step(svc, .6)
  # Capture the inherited command; deeper captures release at the existing J_UP.
  previous = captured = svc._last_cmd
  for i in range(101):
    res = service_step(svc, .5 - i * .0044)
    assert res.accel <= previous + (svc.p.J_UP * .01 if captured < svc.p.A_HOLD_SECURE else 1e-9)
    previous = res.accel
  expected = min(-.5, max(captured, svc.p.A_HOLD_SECURE))
  assert previous == pytest.approx(expected, abs=.01)
  first = service_step(svc, .05, wheel=True).accel
  if seed >= -.5:
    assert first == pytest.approx(previous - .006)  # no grace even with nonzero v
    for _ in range(40):
      value = service_step(svc, 0., wheel=True).accel
      assert value == pytest.approx(max(-.7, first - .006))
      first = value
    assert first == pytest.approx(-.7)


def test_pin_level_cannot_accelerate_hold_without_roll(monkeypatch):
  monkeypatch.setattr(stopping_flags, 'FLAT_LANDING', True)
  svc = StoppingService()
  svc.phase, svc._last_cmd, svc._pin_level = Phase.HOLD, -.5, -.7
  assert svc._jerk_limit(-.7, False, .01) == pytest.approx(-.506)
  service_step(svc, 0., wheel=True)
  before = svc._last_cmd
  res = service_step(svc, .04, wheel=True)  # measured roll above the post-latch minimum
  assert svc.ev.finish_roll(.04)
  assert res.accel == pytest.approx(before - svc.p.J_SAFE * .01)


def test_floor_and_landing_reach_hyundai_and_panda(lc, monkeypatch):
  monkeypatch.setattr(stopping_flags, 'FLAT_LANDING', True)
  arm(lc)
  controller, _ = make_controller()
  controller.frame, controller.engaged_frame = 1000, 1
  panda = test_hyundai.TestHyundaiLongitudinalSafety('test_no_aeb_scc12')
  panda.setUp()
  panda.safety.set_controls_allowed(True)
  panda._rx(panda._speed_msg(1.5))
  sent = []
  for i in range(300):
    v = max(0., 1.5 - .01 * i)
    lagged = lc.long_control_state
    wire = step(lc, v=v, target=-.1)
    _, messages = controller.update(make_cc(accel=wire, state=lagged), make_cs(v_ego=v, a_ego=-.5), 0, SimpleNamespace())
    for addr, dat, bus in messages:
      assert panda.safety.safety_tx_hook(libsafety_py.make_CANPacket(addr, bus, dat))
      if addr == SCC12_ADDR:
        assert get_signal('SCC12', 'aReqValue', dat) == pytest.approx(wire, abs=.0051)
        sent.append(wire)
  assert len(sent) == 150 and min(sent) <= -.7 and any(abs(x + .5) < .01 for x in sent)


def test_departure_releases_on_first_frame(lc):
  arm(lc)
  assert step(lc, lead_v=2., target=.3) > stopping_flags.A_FLOOR
  assert not lc._final_floor_armed and not lc._service_signals.lead_confirmed_stopped


def test_transition_logs_include_approach_bound_count(lc, monkeypatch):
  from openpilot.common.swaglog import cloudlog
  logged = []
  monkeypatch.setattr(cloudlog, 'warning', lambda *args, **kwargs: logged.append(args))
  arm(lc)
  update = lc._service_shadow_svc.update
  monkeypatch.setattr(lc._service_shadow_svc, 'update', lambda **kw: replace(update(**kw), accel=-.35))
  count = lc._final_floor_bound_frames
  for _ in range(7):
    step(lc)
  assert lc._final_floor_bound_frames == count + 7
  lc.reset()
  events = [r for r in logged if r[0].startswith('stopping final_floor')]
  assert len(events) == 2 and 'arm reason=request' in events[0][0]
  assert events[1][1:] == ('reset', 1.5, 10., count + 7)


def test_service_exception_preserves_braking_while_owned(lc, monkeypatch):
  arm(lc)
  assert lc._service_live_owning
  previous = lc.last_output_accel

  def fail(**kwargs):
    raise RuntimeError('injected service failure while owned')

  monkeypatch.setattr(lc._service_shadow_svc, 'update', fail)
  assert step(lc, target=.3) <= previous
  assert not lc._final_floor_armed and lc._service_signals is None


@pytest.mark.parametrize('gap', [60., 100.])
def test_withdrawn_request_releases_before_service_entry(lc, gap):
  for _ in range(150):
    step(lc, gap=gap)
  assert lc._service_signals.lead_confirmed_stopped
  assert lc._service_shadow_svc.phase == Phase.INACTIVE
  assert not lc._service_live_owning and not lc._final_floor_armed
  assert step(lc, gap=gap, target=.5) == .5


def test_service_release_disarms_floor_while_still_owned(lc, monkeypatch):
  from openpilot.common.swaglog import cloudlog
  arm(lc)
  logged = []
  monkeypatch.setattr(cloudlog, 'warning', lambda *args, **kwargs: logged.append(args))
  update = lc._service_shadow_svc.update

  def release(**kwargs):
    result = update(**kwargs)
    lc._service_shadow_svc.phase = Phase.RELEASE
    return replace(result, accel=-.35)

  monkeypatch.setattr(lc._service_shadow_svc, 'update', release)
  assert step(lc) == -.35
  assert lc._service_live_owning and not lc._final_floor_armed
  assert lc._final_floor_added == 0.
  assert any(args[1] == 'not_owned' for args in logged if args[0].startswith('stopping final_floor disarm'))


@pytest.mark.parametrize('seed', [-.3, -.5, -.9, -1.1])
@pytest.mark.parametrize('transient', [False, True])
def test_flat_landing_never_deeper_than_head(monkeypatch, seed, transient):
  from openpilot.tools.stopping.review.stage1_replay import head_controller
  landings = []
  for flat in (False, True):
    monkeypatch.setattr(stopping_flags, 'FLAT_LANDING', flat)
    svc = StoppingService() if flat else head_controller()(DummyCarParams())._service_shadow_svc
    svc._last_cmd = seed
    service_step(svc, .6)
    for i in range(101):
      deep = transient and 30 <= i < 40
      service_step(svc, .5 - i * .0044, a_target=-1. if deep else None, a_target_trajectory=-1. if deep else None)
    landings.append(service_step(svc, .05, wheel=True).accel)
  assert landings[1] >= landings[0] - 1e-9


def test_transient_lane_does_not_move_flat_landing(monkeypatch):
  monkeypatch.setattr(stopping_flags, 'FLAT_LANDING', True)
  landings = []
  for transient in (False, True):
    svc = StoppingService()
    svc._last_cmd = -.3
    service_step(svc, .6)
    for i in range(101):
      deep = transient and 30 <= i < 40
      res = service_step(svc, .5 - i * .0044, a_target=-1. if deep else None, a_target_trajectory=-1. if deep else None)
      if deep:
        assert res.debug['a_plan'] < res.debug['a_phase']
    landings.append(service_step(svc, .05, wheel=True).accel)
  assert landings[0] == pytest.approx(landings[1])


def test_deep_capture_releases_at_head_j_up(monkeypatch):
  from openpilot.tools.stopping.review.stage1_replay import head_controller
  traces = []
  for flat in (False, True):
    monkeypatch.setattr(stopping_flags, 'FLAT_LANDING', flat)
    svc = StoppingService() if flat else head_controller()(DummyCarParams())._service_shadow_svc
    svc._last_cmd = -1.1
    service_step(svc, .6)
    svc._last_cmd = -1.1
    trace = [-1.1]
    for i in range(40):
      trace.append(service_step(svc, .5 - i * .01).accel)
      assert trace[-1] == pytest.approx(min(-.7, trace[-2] + svc.p.J_UP * .01))
    assert trace[-1] == pytest.approx(-.7)
    traces.append(trace)
  assert traces[0] == pytest.approx(traces[1])
