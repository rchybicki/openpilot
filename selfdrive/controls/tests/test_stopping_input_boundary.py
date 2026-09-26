"""Fault injection through the actual planner and LongControl entry points."""
import math
from types import SimpleNamespace

import numpy as np
import pytest
from cereal import log

from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.controls.lib import identification_hook as ih, longitudinal_planner as planner_module, stopping_flags
from openpilot.selfdrive.controls.lib.drive_helpers import longitudinal_accel_with_gas
from openpilot.selfdrive.controls.lib.lead_provenance import lead_values_finite
from openpilot.selfdrive.controls.lib.longcontrol import LongControl, LongCtrlState
from openpilot.selfdrive.controls.lib.tests.test_longcontrol_fast_release import DummyCarParams, DummyCarState, DummyFrogPilotToggles
from openpilot.selfdrive.controls.tests.test_identification_hook import T0, _press_schedule, good
from openpilot.selfdrive.controls.tests.test_whole_approach_governor import make_lead

BAD_VALUES = [math.nan, math.inf, -math.inf]
PLANNER_FIELDS = [('leadOne', key) for key in ('dRel', 'vRel', 'vLead', 'aLeadK', 'modelProb', 'radarTrackId')] + [
  ('leadTwo', key) for key in ('dRel', 'vLead', 'modelProb')]


@pytest.fixture
def planner(monkeypatch):
  class Solver:
    def __init__(self, **kwargs):
      self.mode, self.source, self.crash_cnt, self.solve_time = 'blended', 'e2e', 0, 0.0
      self.lead_xv_0 = self.lead_xv_1 = np.zeros((13, 2))
      self.calls = 0
      self.constraints = []

    def set_weights(self, *args, **kwargs):
      self.constraints.append(args[3])

    def set_cur_state(self, *args):
      pass

    def update(self, cruise, model, radar, x, v, a, j, *args, **kwargs):
      for lead in (radar.leadOne, radar.leadTwo):
        assert lead_values_finite(lead.status, lead.dRel, lead.vLead)
      self.calls += 1
      self.v_solution, self.a_solution, self.j_solution = v, a, j[:-1]

  monkeypatch.setattr(planner_module, 'LongitudinalMpc', Solver)
  cp = SimpleNamespace(carFingerprint=planner_module.HYUNDAI_CAR.HYUNDAI_SANTA_FE_HEV_2022,
                       openpilotLongitudinalControl=True, wheelbase=2.7, steerRatio=15.0)
  instance = planner_module.LongitudinalPlanner(cp)
  toggles = SimpleNamespace(taco_tune=False, long_distance_factor=1.0, short_distance_factor=1.0,
                            longitudinalActuatorDelay=.45, vEgoStopping=.25, lead_detection_probability=.5)

  class Messages(dict):
    updated = {'radarState': True}
    valid = {'radarState': True}
    alive = {'radarState': True}
    logMonoTime = {'modelV2': 0}

    def all_checks(self, **kwargs):
      return True

  times = np.array(planner_module.ModelConstants.T_IDXS)
  sm = Messages(
    carState=SimpleNamespace(vEgo=8.0, aEgo=-.6, vCruise=50, steeringAngleDeg=0.0, standstill=False,
                             gasPressed=False, brakePressed=False),
    controlsState=SimpleNamespace(longControlState=LongCtrlState.pid, forceDecel=False),
    selfdriveState=SimpleNamespace(experimentalMode=True, enabled=True, personality=log.LongitudinalPersonality.standard),
    carControl=SimpleNamespace(orientationNED=[0.0, 0.0, 0.0], actuators=SimpleNamespace(accel=-.6)),
    liveParameters=SimpleNamespace(angleOffsetDeg=0.0), frogpilotCarState=SimpleNamespace(forceCoast=False, trafficModeEnabled=False),
    frogpilotPlan=SimpleNamespace(vCruise=8.0, minAcceleration=-3.5, maxAcceleration=2.0, cscControllingSpeed=False,
                                  laneWidthLeft=3.5, accelerationJerk=1.0, dangerJerk=1.0, speedJerk=1.0,
                                  dangerFactor=1.0, tFollow=1.5, increasedStoppedDistance=.3),
    radarState=SimpleNamespace(leadOne=make_lead(), leadTwo=make_lead()),
    modelV2=SimpleNamespace(position=SimpleNamespace(x=(8.0 * times).tolist()),
                           velocity=SimpleNamespace(x=np.maximum(8.0 - .6 * times, 0.0).tolist()),
                           acceleration=SimpleNamespace(x=[-.6] * len(times)),
                           meta=SimpleNamespace(disengagePredictions=SimpleNamespace(gasPressProbs=[1.0, 1.0])),
                           action=SimpleNamespace(desiredAcceleration=-.6, shouldStop=False), leadsV3=[SimpleNamespace(prob=.9)]))
  for _ in range(12):
    instance.update(sm, toggles)
  return instance, sm, toggles


@pytest.mark.parametrize('bad', BAD_VALUES)
@pytest.mark.parametrize('lead_name,field', PLANNER_FIELDS)
def test_planner_rejects_before_solver_and_recovers(planner, lead_name, field, bad):
  instance, sm, toggles = planner
  lead = getattr(sm['radarState'], lead_name)
  original = getattr(lead, field)
  previous = instance.output_a_target
  instance.output_should_stop = True
  calls = [solver.calls for solver in (instance.mpc, instance.acc_mpc)]
  captured = []
  pm = SimpleNamespace(send=lambda name, msg: captured.append(msg))
  setattr(lead, field, bad)
  for _ in range(3):
    instance.update(sm, toggles)
    instance.publish(sm, pm, toggles)
    msg = captured[-1]
    assert not msg.valid and not msg.longitudinalPlan.aTargetTrajectoryValid
    assert not msg.longitudinalPlan.allowThrottle and msg.longitudinalPlan.shouldStop
    assert msg.longitudinalPlan.aTarget == pytest.approx(min(previous, 0.0))
    assert math.isfinite(msg.longitudinalPlan.aTargetTrajectory)
    assert np.isfinite(msg.longitudinalPlan.accels).all()
    assert not instance._sf_lead_auth.certified and not instance._wa_lead_auth.certified
    assert not instance.wa_committed and not instance.stop_commit_active
  assert [solver.calls for solver in (instance.mpc, instance.acc_mpc)] == calls
  setattr(lead, field, original)
  instance.update(sm, toggles)
  instance.publish(sm, pm, toggles)
  assert captured[-1].valid and captured[-1].longitudinalPlan.aTargetTrajectoryValid
  assert instance.output_a_target <= previous and instance.output_a_target_trajectory <= previous
  assert instance.mpc.calls == calls[0] + 1
  assert instance.acc_mpc.calls == calls[1] + 1
  assert not instance.mpc.constraints[-1] and not instance.acc_mpc.constraints[-1]


def test_absent_and_unused_lead_fields_do_not_fault(planner):
  instance, sm, toggles = planner
  sm['radarState'].leadOne.status = False
  for field in ('dRel', 'vRel', 'vLead', 'aLeadK', 'modelProb'):
    setattr(sm['radarState'].leadOne, field, math.nan)
  for field in ('vRel', 'aLeadK', 'radarTrackId'):
    setattr(sm['radarState'].leadTwo, field, math.nan)
  instance.update(sm, toggles)
  assert not instance.lead_input_fault


@pytest.mark.parametrize('lead_name,field', [('leadOne', 'vRel'), ('leadTwo', 'modelProb')])
def test_planner_only_fault_reaches_wire_boundary(planner, lead_name, field):
  instance, sm, toggles = planner
  lc = LongControl(DummyCarParams())
  lc.last_output_accel = -1.5
  lc.long_control_state = LongCtrlState.pid
  lc._service_live_owning = True
  setattr(getattr(sm['radarState'], lead_name), field, math.nan)
  instance.update(sm, toggles)
  captured = []
  instance.publish(sm, SimpleNamespace(send=lambda name, msg: captured.append(msg)), toggles)
  msg = captured[-1]
  result = run(lc, a_target=msg.longitudinalPlan.aTarget, plan_valid=msg.valid,
               a_target_trajectory=msg.longitudinalPlan.aTargetTrajectory if msg.longitudinalPlan.aTargetTrajectoryValid else None)
  assert result <= -1.5 and not lc._service_live_owning


def run(lc, *, v=1.2, a_ego=-.5, active=True, brake=False, force_coast=False, **kwargs):
  values = dict(experimental_mode=True, lead_status=True, lead_v=0.0, lead_d_rel=10.0, lead_a=0.0,
                lead_track_id=123, lead_model_prob=1.0, lead2_status=True, lead2_v=0.0, lead2_d_rel=20.0,
                a_target_trajectory=-1.0)
  a_target = kwargs.pop('a_target', -1.0)
  values.update(kwargs)
  cs = DummyCarState(v_ego=v, a_ego=a_ego, brake_pressed=brake, standstill=v == 0.0)
  return lc.update(active, cs, a_target, v < .25, -1.0, (-3.5, 2.0), DummyFrogPilotToggles(), force_coast=force_coast, **values)


@pytest.mark.parametrize('bad', BAD_VALUES)
@pytest.mark.parametrize('field', ['v', 'a_ego', 'a_target', 'lead_v', 'lead_d_rel', 'lead_a', 'lead_model_prob', 'lead_track_id',
                                  'lead2_v', 'lead2_d_rel', 'a_target_trajectory'])
def test_wire_fault_after_ownership_and_recovery(field, bad):
  lc = LongControl(DummyCarParams())
  for _ in range(60):
    run(lc)
  assert lc._service_live_owning
  previous = lc.last_output_accel
  for _ in range(3):
    result = run(lc, **{field: bad})
    assert math.isfinite(result) and result <= previous and result <= 0.0
    assert not lc._service_live_owning and not lc._service_lead_certificate.certified
    assert not lc._service_live_disabled
    previous = result
  assert run(lc) <= previous
  assert not lc._service_live_owning
  assert math.isfinite(run(lc))


@pytest.mark.parametrize('fault', [{'lead_d_rel': math.nan}, {'lead2_d_rel': math.nan}, {'plan_valid': False},
                                   {'v': math.nan}, {'v': math.inf}, {'a_ego': math.nan}, {'a_ego': -math.inf}])
@pytest.mark.parametrize('state', ['hold', 'force_coast', 'inactive', 'brake', 'gas'])
def test_fault_retains_hold_and_driver_authority(fault, state):
  lc = LongControl(DummyCarParams())
  lc.last_output_accel = -.8
  lc.long_control_state = LongCtrlState.stopping
  result = run(lc, **{'v': 0.0, **fault}, active=state != 'inactive', brake=state == 'brake', force_coast=state == 'force_coast')
  if state == 'gas':
    result = longitudinal_accel_with_gas(result, True, True)
  assert result == 0.0 if state in ('inactive', 'brake', 'gas') else result <= -.8


@pytest.mark.parametrize('fault', [{'lead2_v': math.nan}, {'lead_model_prob': math.nan}, {'plan_valid': False}])
def test_service_fault_keeps_deeper_valid_primary_lead_demand(fault):
  lc = LongControl(DummyCarParams())
  lc.last_output_accel = -.2
  lc.long_control_state = LongCtrlState.pid
  lc._service_live_owning = True
  result = run(lc, v=8.0, lead_d_rel=2.0, a_target=-3.0, **fault)
  assert result < -.2 and math.isfinite(result)
  assert not lc._service_live_owning


@pytest.fixture(autouse=True)
def _kcs1(monkeypatch):
  monkeypatch.setattr(ih, 'PLAN_ID', 'KCS1')   # the identification cases below drive KCS1 maneuver B
  monkeypatch.setattr(ih, 'MANEUVERS', ih.BLOCKS['KCS1'])


def _rep(monkeypatch, past_intent):
  """A KCS1 rep through the real LongControl: maneuver B (-1.0 to the stop) from the 20 km/h cruise; past_intent drives it
  (the command is the plant's deceleration) below V_INTENT, where the hook owns the wire and LongControl is stopping."""
  monkeypatch.setattr(stopping_flags, 'IDENTIFICATION_HOOK', True)
  lc = LongControl(DummyCarParams())
  lc.long_control_state = LongCtrlState.pid
  car = {'v_ego': 5.56, 'a_ego': 0.0}

  def step(inputs=None, active=True, a_target=0.0, **motion):
    cs = DummyCarState(**{**car, 'standstill': car['v_ego'] == 0.0, **motion})
    inputs = inputs or good(**car, standstill=car['v_ego'] == 0.0)
    return float(lc.update(active, cs, a_target, False, -1.0, (-3.5, 2.0), DummyFrogPilotToggles(), id_inputs=inputs))

  def drive(n):
    wires = []
    for _ in range(n):
      car['v_ego'] = max(0.0, car['v_ego'] + car['a_ego'] * DT_CTRL)
      wires.append(step())
      car['a_ego'] = wires[-1] if car['v_ego'] > 0.0 else 0.0
    return wires

  for k in range(T0 + 1):
    step(_press_schedule(k))
  assert lc._id_hook.state == 'ACTIVE' and lc.id_hook_out.maneuver == 'B' and lc.id_hook_out.floor == lc.last_output_accel == -1.0
  if past_intent:
    car['a_ego'] = -1.0
    while not lc.id_hook_out.stop_intent:
      drive(1)
    drive(2)
    assert lc.long_control_state == LongCtrlState.stopping and lc.id_hook_out.stop_intent and car['v_ego'] > 1.5
  return lc, step, drive, car


INTENT = pytest.mark.parametrize('past_intent', [False, True], ids=['before_intent', 'after_intent'])


@INTENT
@pytest.mark.parametrize('bad', BAD_VALUES)
@pytest.mark.parametrize('field', ['v_ego', 'a_ego'])
def test_motion_fault_during_a_rep_locks_and_keeps_its_braking(monkeypatch, field, bad, past_intent):
  lc, step, drive, car = _rep(monkeypatch, past_intent)
  for _ in range(3):
    # the reviewer's frame: a lead appears while the motion input is invalid
    assert step(good(**{**car, field: bad}, lead_prob=.5), **{field: bad}) == -1.0
    assert lc.lead_input_fault and lc._id_hook._locked == 'fault' and lc._id_hook.state == ('ACTIVE' if past_intent else 'HANDBACK')
  released = drive(30)
  assert not lc.lead_input_fault and released[0] == -1.0 and lc._id_hook.done == {m[0]: 0 for m in ih.MANEUVERS}
  if past_intent:                                                      # the floor is kept to the stop, never released
    assert released == [-1.0] * 30 and lc._id_hook.state == 'ACTIVE' and lc.id_hook_out.stop_intent and not lc.id_hook_out.own
  else:                                                                # a bounded release toward cruise
    assert released[-1] > -1.0 and lc._id_hook.state == 'HANDBACK'
    assert all(0.0 <= b - a <= ih.RELEASE_JERK * DT_CTRL + 1e-9 for a, b in zip(released, released[1:], strict=False))


@INTENT
def test_fault_publishes_the_lock_without_advancing_the_hook(monkeypatch, past_intent):
  lc, step, drive, car = _rep(monkeypatch, past_intent)
  hook = lc._id_hook
  progress = (hook._rep_t, hook._seg, hook._last_cmd)
  held = [step(a_target=math.nan) for _ in range(50)]
  out = lc.id_hook_out
  assert held == [-1.0] * 50 and (hook._rep_t, hook._seg, hook._last_cmd) == progress
  assert (out.state, out.maneuver, out.rep, out.seg, out.reason, out.floor, out.own, out.stop_intent, out.rep_done) == (
    'ACTIVE' if past_intent else 'HANDBACK', 'B', 1, 1, 'fault', -1.0, False, past_intent, '')
  if not past_intent:
    assert (out.text1, out.text2) == ('TEST B 1/6 ABORTED - fault', 'releasing; test mode LOCKED')
  assert step() == -1.0 and lc.id_hook_out == out                     # first valid frame: still held, still the fault banner
  if past_intent:
    kept = drive(400)                                                  # to the wheel stop and into the hold
    assert kept == [-1.0] * 400 and hook.state == 'HELD' and lc.long_control_state == LongCtrlState.stopping
    lc.reset()                                                         # the driver's brake ends the hold: not counted
    brake = good(**car, standstill=True, brake=True, enabled=False, long_active=False, plan_accel=.3)
    assert step(brake, active=False, a_target=.3, brake_pressed=True) == 0.0   # the planner wants the set speed back
  else:
    released = [step() for _ in range(150)]
    assert released[0] == pytest.approx(-1.0 + ih.RELEASE_JERK * DT_CTRL) and released[-1] == 0.0
    assert all(0.0 <= b - a <= ih.RELEASE_JERK * DT_CTRL + 1e-9 for a, b in zip(released, released[1:], strict=False))
  assert hook.state == 'LOCKED' and hook.done == {m[0]: 0 for m in ih.MANEUVERS} and lc.id_hook_out.floor is None


def test_fault_after_the_intent_keeps_the_finishing_banner_while_moving(monkeypatch):
  lc, step, drive, car = _rep(monkeypatch, True)
  texts = []
  for _ in range(3):
    step(a_target=math.nan)
    texts.append((lc.id_hook_out.state, lc.id_hook_out.text1, lc.id_hook_out.text2))
  assert texts == [('ACTIVE', 'TEST B 1/6 ABORTED - fault', 'finishing the stop; brake to end')] * 3


@INTENT
def test_disengaged_fault_frame_publishes_the_lock(monkeypatch, past_intent):
  lc, step, drive, car = _rep(monkeypatch, past_intent)
  lc.reset()                                                           # controlsd: longActive False
  off = good(**car, enabled=False, long_active=False)
  assert step(off, active=False, a_target=math.nan) == 0.0
  out = lc.id_hook_out
  # review 20260926-151117 finding 2: a disengaged fault frame ends the hook's authority at once (no floor survives)
  assert out.state == 'LOCKED' and out.reason == 'fault' and lc._id_hook._locked == 'fault' and out.floor is None
  for _ in range(2):                                                   # recovering, then recovered and disengaged
    assert step(off, active=False) == 0.0
  assert lc._id_hook.state == 'LOCKED' and lc.id_hook_out.floor is None and not lc.id_hook_out.stop_intent


def test_bad_conversion_is_fault_without_range_or_absent_policy():
  assert not lead_values_finite(True, 'bad')
  assert not lead_values_finite(True, 1.0, track_id='bad')
  assert lead_values_finite(False, math.nan, track_id=math.nan)
  assert lead_values_finite(True, -1.0, track_id=-1)
