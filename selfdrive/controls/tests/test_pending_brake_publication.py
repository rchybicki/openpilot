from types import SimpleNamespace

import pytest
from cereal import car, log
from cereal.services import SERVICE_LIST
from openpilot.selfdrive.controls import controlsd
from openpilot.selfdrive.controls.controlsd import Controls


@pytest.fixture
def controls(monkeypatch):
  module = controlsd
  now = 100.0
  monkeypatch.setattr(module.time, 'monotonic', lambda: now)
  original = module.messaging.new_message

  def message(name, *args, **kwargs):
    event = original(name, *args, **kwargs)
    event.logMonoTime = int(now * 1e9)
    return event

  monkeypatch.setattr(module.messaging, 'new_message', message)
  c = Controls.__new__(Controls)
  names = ['carState', 'selfdriveState', 'longitudinalPlan', 'radarState', 'modelV2', 'frogpilotCarState', 'frogpilotPlan',
           'liveParameters', 'liveDelay', 'carOutput', 'driverMonitoringState', 'driverAssistance']

  class Messages(dict):
    pass

  c.sm = Messages({name: getattr(message(name), name) for name in names})
  c.sm['onroadEvents'] = []
  c.sm.valid = dict.fromkeys(names, True)
  c.sm.valid['driverAssistance'] = False
  c.sm.alive = dict.fromkeys(names, True)
  c.sm.logMonoTime = dict.fromkeys(names, int(now * 1e9))
  c.sm.alive_timeout = {name: 10. / max(min(SERVICE_LIST[name].frequency, 100.), 1.) for name in names}
  c.sm['carState'].canValid = True
  c.sm['carState'].vEgo = 2.
  c.sm['carState'].aEgo = -.5
  c.sm['selfdriveState'].enabled = True
  c.sm['driverMonitoringState'].awarenessStatus = 1.
  c.CP = car.CarParams.new_message()
  c.CP.brand = 'hyundai'
  c.CP.openpilotLongitudinalControl = True
  c.CP.lateralTuning.init('pid')
  c.longitudinal_active_with_gas = True
  c.live_update_handoff_state = ''
  c.desired_curvature = c.curvature = 0.
  c.steer_limited_by_safety = False
  c.calibrated_pose = None
  c.frogpilot_toggles = SimpleNamespace(max_desired_acceleration=1., personality_profile_via_distance_long=False)
  c.CI = SimpleNamespace(get_pid_accel_limits=lambda *args: (-3.5, 2.))
  c.VM = SimpleNamespace(update_params=lambda *args: None, calc_curvature=lambda *args: 0.)
  lateral = log.ControlsState.new_message().lateralControlState.init('pidState')
  c.LaC = SimpleNamespace(reset=lambda: None, update=lambda *args: (0., 0., lateral))
  c.events = []
  c.requested = -.5

  def update(*args, **kwargs):
    c.events.append(('update_time', kwargs['request_time']))
    return c.requested

  c.LoC = SimpleNamespace(update=update, reset=lambda: None, long_control_state=car.CarControl.Actuators.LongControlState.pid,
                         pid=SimpleNamespace(p=0., i=0., f=0.), id_hook_out=None,
                         observe_accel_request=lambda value, stamp, **kw: c.events.append(('observed', value, stamp, kw['authorized'])))
  c.pm = SimpleNamespace(send=lambda name, data: c.events.append(('sent', name, data.to_bytes())))
  return c


@pytest.mark.parametrize('condition', [
  'normal', 'gas', 'brake', 'paused', 'disabled', 'handoff', 'nonfinite', 'stale', 'invalid', 'future', 'dead', 'acc_fault',
])
def test_final_request_observed_after_arbitration_and_publication(controls, condition):
  c = controls
  if condition == 'gas':
    c.sm['carState'].gasPressed = True
  elif condition == 'brake':
    c.sm['carState'].brakePressed = True
  elif condition == 'paused':
    c.sm['frogpilotCarState'].pauseLongitudinal = True
  elif condition == 'disabled':
    c.sm['selfdriveState'].enabled = False
  elif condition == 'handoff':
    c.live_update_handoff_state = next(iter(controlsd.PANDA_HANDOFF_STATES))
  elif condition == 'nonfinite':
    c.requested = float('nan')
  elif condition == 'stale':
    c.sm.logMonoTime['carState'] -= 200_000_000
  elif condition == 'future':
    c.sm.logMonoTime['carState'] += 1
  elif condition == 'invalid':
    c.sm.valid['longitudinalPlan'] = False
  elif condition == 'dead':
    c.sm.alive['radarState'] = False
  elif condition == 'acc_fault':
    c.sm['carState'].accFaulted = True
  cc, lateral = c.state_control()
  c.publish(cc, lateral)
  assert [r[0] for r in c.events] == ['update_time', 'sent', 'sent', 'observed']
  assert c.events[2][1] == 'carControl'
  observation = c.events[-1]
  assert observation[1] == cc.actuators.accel == (0. if condition in ('gas', 'nonfinite') else -.5)
  assert observation[2] == 100.
  assert observation[3] is (condition in ('normal', 'nonfinite'))
  assert (c.events[0][1] is None) is (condition in ('stale', 'invalid', 'future', 'dead', 'acc_fault'))


@pytest.mark.parametrize('source', ['carState', 'selfdriveState', 'longitudinalPlan', 'radarState', 'modelV2', 'frogpilotCarState', 'frogpilotPlan'])
def test_each_stale_source_disables_request_history_before_control(controls, source):
  c = controls
  c.sm.logMonoTime[source] -= int((c.sm.alive_timeout[source] + .01) * 1e9)
  c.state_control()
  assert c.events[0] == ('update_time', None)


def test_failed_publication_is_not_observed(controls):
  c = controls
  cc, lateral = c.state_control()

  def fail(name, data):
    if name == 'carControl':
      raise RuntimeError('failed publish')

  c.pm.send = fail
  with pytest.raises(RuntimeError, match='failed publish'):
    c.publish(cc, lateral)
  assert not any(event[0] == 'observed' for event in c.events)
