"""Gas override regressions, using the red-team inputs without external fixtures."""
import subprocess
import sys
from pathlib import Path
from types import ModuleType, SimpleNamespace as NS

import pytest
from cereal import car
from openpilot.selfdrive.controls.lib import stopping_flags, stopping_service
from openpilot.selfdrive.controls.lib.longcontrol import LongControl, LongCtrlState
from openpilot.selfdrive.controls.lib.stopping_service import Phase


class Session:
  def __init__(self, controller=LongControl, ki=0.0):
    cp = car.CarParams.new_message()
    cp.carFingerprint = 'HYUNDAI_SANTA_FE_HEV_2022'
    cp.startingState = True
    cp.vEgoStarting, cp.vEgoStopping, cp.startAccel, cp.stopAccel = 0.1, 0.5, 0.7, -2.0
    cp.openpilotLongitudinalControl = True
    cp.longitudinalTuning.kpBP, cp.longitudinalTuning.kpV = [0.0], [0.0]
    cp.longitudinalTuning.kiBP, cp.longitudinalTuning.kiV = [0.0], [ki]
    self.lc = controller(cp)
    self.toggles = NS(human_acceleration=False, vEgoStarting=0.1, startAccel=0.7, vEgoStopping=0.5,
                      force_coast_strength=1.4, max_desired_acceleration=2.0)
    self.frame = 0
    self.debug = {}
    original = self.lc._run_stopping_service

    def record(**kwargs):
      result = original(**kwargs)
      self.debug = result.debug if result else {}
      return result

    self.lc._run_stopping_service = record

  def step(self, *, v=0.0, gap=5.0, gas=False, active=True, freeze=None, target=-0.1, trajectory=None,
           stop=True, lead=True, valid=True, lead_v=0.0, a=0.0):
    cs = NS(vEgo=v, aEgo=a, gasPressed=gas, brakePressed=False, standstill=v < 0.01,
            cruiseState=NS(standstill=False, enabled=True), canValid=True, canTimeout=False, vCruise=50.0)
    if not active:
      self.lc.reset()
    wire = self.lc.update(active, cs, target, stop, max(gap - 3.0, 0.0) if lead else -1.0, (-3.5, 2.0), self.toggles,
                          lead_status=lead, lead_v=lead_v, lead_d_rel=gap if lead else 0.0, lead_track_id=7 if lead else -1,
                          model_should_stop=stop, model_stop_d=-1.0, a_target_trajectory=trajectory,
                          freeze_integrator=gas if freeze is None else freeze, plan_valid=valid, request_time=self.frame * 0.01)
    self.lc.observe_accel_request(wire, self.frame * 0.01 + 0.0001, authorized=active and not gas and valid)
    self.frame += 1
    return float(wire)


@pytest.fixture(autouse=True)
def live_service(monkeypatch):
  monkeypatch.setattr(stopping_flags, 'SERVICE_MODE', 'LIVE')
  monkeypatch.setattr(stopping_flags, 'SERVICE_APPROACH_LAW', 'governor')


def test_gas_discards_actuation_and_settle_but_preserves_context():
  s = Session()
  events = []
  s.lc._service_shadow_tel._log = lambda **kw: events.append(kw)
  for _ in range(500):
    s.step()
  assert s.lc._service_shadow_svc.phase == Phase.HOLD
  events.clear()
  for _ in range(60):
    assert s.step(gas=True) == 0.0
    assert s.lc.last_output_accel == 0.0
    assert not s.lc._service_live_owning
    assert s.lc._service_shadow_svc.phase == Phase.INACTIVE
    assert s.lc._service_signals.lead_confirmed_stopped
  assert s.lc._service_shadow_tel._frames == 0
  assert not any(e.get('kind') == 'settle_summary' for e in events)
  assert s.lc.long_control_state == LongCtrlState.stopping


@pytest.mark.parametrize('dropout', [False, True])
def test_pid_lift_owns_immediately_with_preserved_dropout(dropout):
  s = Session()
  for i in range(90):
    s.step(v=1.2, gap=4.0, gas=60 <= i, stop=False, lead=not (dropout and i >= 80))
  wire = s.step(v=1.2, gap=4.0, stop=False, lead=not dropout)
  assert s.lc._service_live_owning
  assert wire <= -0.25
  assert s.lc._service_signals.dropout_active == dropout


def test_hidden_planner_depth_does_not_seed_lift():
  s = Session()
  for i in range(230):
    s.step(v=1.147, gap=7.45, gas=i >= 200, target=-3.5 if i >= 200 else -0.1, trajectory=-0.1)
  for _ in range(50):
    wire = s.step(v=1.147, gap=7.45, target=-3.5, trajectory=-0.1)
    assert wire > -1.0
    assert s.debug['a_plan'] > -1.0
    assert s.debug['a_kin'] > -1.0


def test_lift_seeds_safety_not_comfort():
  s = Session()
  for i in range(100):
    s.step(v=1.2, gap=3.6, gas=i >= 60)
  wire = s.step(v=1.2, gap=3.6)
  assert wire == -3.5
  assert s.debug['a_barrier'] <= -3.5


def test_fault_frame_never_releases():
  s = Session()
  for _ in range(100):
    s.step(v=1.2, gap=3.6)
  prior = s.lc.last_output_accel
  assert s.step(v=1.2, gap=3.6, gas=True, valid=False) <= prior


@pytest.fixture
def head_controller(monkeypatch):
  modules = []
  for name in ('stopping_service', 'longcontrol'):
    module = ModuleType('_gas_release_head_' + name)
    monkeypatch.setitem(sys.modules, module.__name__, module)
    source = subprocess.check_output(['git', 'show', f'HEAD:selfdrive/controls/lib/{name}.py'],
                                     cwd=Path(__file__).resolve().parents[3], text=True)
    exec(compile(source, f'HEAD:{name}.py', 'exec'), module.__dict__)
    modules.append(module)

  class HeadService(modules[0].StoppingService, stopping_service.StoppingService):
    pass

  modules[1].StoppingService = HeadService
  return modules[1].LongControl


def test_without_active_with_gas_is_identical(head_controller):
  # Without the feature, controlsd disengages on gas and supplies freeze_integrator=False.
  # Gas metadata alone must not change the inactive or re-engagement trajectory.
  a, b = Session(), Session(controller=head_controller)
  for i in range(1000):
    gas = 500 <= i < 560
    assert a.step(gas=gas, active=not gas, freeze=False) == b.step(gas=gas, active=not gas, freeze=False)
    assert a.lc.long_control_state == b.lc.long_control_state
    assert a.lc._service_shadow_svc._last_cmd == b.lc._service_shadow_svc._last_cmd


def test_internal_clip_preserves_safety_rate_and_bounds_release(monkeypatch):
  monkeypatch.setattr(stopping_flags, 'SERVICE_APPROACH_LAW', 'legacy')
  s = Session()
  for i in range(200):
    wire = s.step(v=2.4, gap=2.1)
    if i == 30:
      assert wire == pytest.approx(-2.58)
      assert s.debug['safety_binding']
    assert s.lc._service_shadow_svc._last_cmd >= -3.5
  s.step(v=6.6 ** 0.5, gap=2.1)
  assert s.debug['a_kin'] == pytest.approx(-11.0)
  assert s.lc._service_shadow_svc._last_cmd == -3.5
  s.step(gap=2.1)
  for _ in range(300):
    s.step(lead=False, target=0.4, stop=False)
    if s.lc._service_shadow_svc.phase == Phase.INACTIVE:
      break
  assert s.lc._service_shadow_svc.phase == Phase.INACTIVE


@pytest.mark.parametrize('tap_frames', [30, 60, 100])
def test_a_gas_tap_in_a_standstill_hold_keeps_the_secure_hold(tap_frames):
  # the lift after a gas tap that did not move the car re-enters at the secure hold, not from the published zero
  s = Session()
  for _ in range(500):
    s.step()
  assert s.lc._service_shadow_svc.phase == Phase.HOLD
  for _ in range(tap_frames):
    s.step(gas=True)
  wires = [s.step() for _ in range(100)]
  assert max(wires) <= -0.69 and s.lc.long_control_state == LongCtrlState.stopping


def test_a_lift_while_still_rolling_slowly_keeps_the_gentle_finish():
  # host-edit review: the wheel-stop latch stays true up to 0.09 m/s; a lift at 0.06 m/s must not jump to the secure hold
  s = Session()
  for _ in range(500):
    s.step()
  assert s.lc._service_shadow_svc.phase == Phase.HOLD
  for _ in range(60):
    s.step(v=0.06, gas=True)
  assert s.step(v=0.06) > -0.1


@pytest.mark.parametrize('v,gap,lead_v', [(1.5, 5.0, 1.5), (2.4, 6.0, 2.4), (2.0, 10.0, 3.0)])
def test_moving_lead_dropout_during_gas_does_not_start_a_stop(v, gap, lead_v):
  s = Session()
  for i in range(550):
    wire = s.step(v=v, gap=gap, lead_v=lead_v, gas=200 <= i < 300, lead=i < 270,
                  stop=False, target=0.0, trajectory=0.0)
    if i >= 300:
      assert wire >= -0.05
      assert not s.lc._service_live_owning
      assert not s.lc._gas_episode


def test_dropout_episode_is_consumed_after_lift():
  s = Session()
  for i in range(90):
    s.step(v=1.2, gap=4.0, gas=i >= 60, stop=False, lead=i < 80)
  assert s.lc._gas_episode
  assert s.step(v=1.2, gap=4.0, stop=False, lead=False) <= -0.25
  assert not s.lc._gas_episode


def test_gas_freezes_integrator_without_cancelling_braking_feedforward(head_controller):
  fixed, head = Session(ki=0.5), Session(controller=head_controller, ki=0.5)
  for i in range(500):
    gas = 100 <= i < 300
    inputs = dict(v=20.0, a=0.5 if gas else -0.2, gap=40.0, lead_v=15.0,
                  gas=gas, target=-1.5, stop=False)
    wire = fixed.step(**inputs)
    baseline = head.step(**inputs)
    assert fixed.lc.pid.i == head.lc.pid.i
    if i >= 300:
      assert wire == baseline
      assert wire <= -1.49


@pytest.mark.parametrize('gas_after', [0.1, 0.6])
def test_a_green_light_launch_with_gas_does_not_brake_when_the_departed_lead_is_lost(gas_after):
  # re-review round 2: HOLD behind a lead, the lead departs, the driver follows with the gas, the lead turns away 0.5 s before
  # the lift; the episode latch must not survive a moving lead (HEAD: >= -0.42; the latched fix braked to -1.69)
  s, dt, v, x, xl, vl, hist = Session(), 0.01, 0.0, 0.0, 5.0, 0.0, [0.0] * 30
  t_dep = 5.0
  gas_on, gas_off = t_dep + gas_after, t_dep + gas_after + 1.5
  after_lift = []
  for i in range(int((gas_off + 3.0) / dt)):
    t = i * dt
    if t >= t_dep:
      vl = min(vl + 1.0 * dt, 4.0)
    gas = gas_on <= t < gas_off
    a = 1.0 if gas else hist[0]
    if v <= 0.0 and a < 0.0:
      a = 0.0
    v = max(v + a * dt, 0.0)
    x, xl = x + v * dt, xl + vl * dt
    stop = t < t_dep + 0.5
    target = -0.1 if stop else 0.5
    wire = s.step(v=v, a=a, gap=xl - x, lead_v=vl, gas=gas, lead=t < gas_off - 0.5, stop=stop, target=target, trajectory=target)
    hist = hist[1:] + [wire]
    if t >= gas_off:
      after_lift.append(wire)
  assert min(after_lift) >= -0.5


@pytest.mark.parametrize('noise', [False, True])
def test_a_re_confirmed_stopped_lead_lost_during_the_gas_still_re_enters_on_the_lift(noise):
  # green-light edit review: one +0.31 m/s lead-speed sample during the gas, then 0.5 s of stopped readings before the lead
  # drops out: the last sighting is a stopped lead, so the lift re-enters (the edit's first version lost the re-entry)
  s = Session()
  for i in range(140):
    s.step(v=1.2, gap=4.0, gas=i >= 60, stop=False, lead=i < 120, lead_v=0.31 if noise and i == 70 else 0.0)
  wire = s.step(v=1.2, gap=4.0, stop=False, lead=False)
  assert s.lc._service_live_owning and wire <= -0.3


@pytest.mark.parametrize('case', ['dropout_during_reconfirmation', 'noise_then_loss'])
def test_incomplete_re_confirmation_is_not_a_departure(case):
  # final edit review: a lead dropout (or one noisy sample) resets the stopped-lead latch; its re-confirmation in progress is
  # not evidence that the lead drove away, so the lift still re-enters behind the stopped lead
  s = Session()
  for i in range(140):
    if case == 'dropout_during_reconfirmation':
      s.step(v=1.2, gap=4.0, gas=i >= 60, stop=False, lead=i < 120 and i != 110)
    else:
      s.step(v=1.2, gap=4.0, gas=i >= 60, stop=False, lead=i < 80, lead_v=0.31 if i == 70 else 0.0)
  wire = s.step(v=1.2, gap=4.0, stop=False, lead=False)
  assert s.lc._service_live_owning and wire <= -0.3
