import pytest

from openpilot.selfdrive.controls.lib.longcontrol import LongControl
from openpilot.selfdrive.controls.lib.drive_helpers import longitudinal_accel_with_gas
from openpilot.selfdrive.controls.lib.stopping_telemetry import StoppingTelemetry
from openpilot.selfdrive.controls.lib.tests.test_longcontrol_fast_release import DummyCarParams, DummyCarState, DummyFrogPilotToggles


def test_complete_timed_history_matches_time_integral():
  lc = LongControl(DummyCarParams())
  cs = DummyCarState(v_ego=2., a_ego=0.)
  cs.canValid, cs.canTimeout, cs.gasPressed = True, False, False
  lc._service_shadow_tel = StoppingTelemetry(log_fn=lambda **kw: None)
  for k in range(40):
    lc.observe_accel_request(-k * .01, k * .01, authorized=True)
  lc.update(True, cs, -.3, False, -1., (-3.5, 2.), DummyFrogPilotToggles(), request_time=.4)
  assert lc._brake_requests[0][0] == pytest.approx(.1)
  assert lc._pending_brake_delta == pytest.approx(-.145)


@pytest.mark.parametrize('reason', ['gas', 'brake', 'inactive', 'freeze', 'plan', 'can', 'timeout', 'late', 'repeat', 'reset'])
def test_invalid_authority_or_time_requires_full_reentry_window(reason):
  lc = LongControl(DummyCarParams())
  lc._service_shadow_tel = StoppingTelemetry(log_fn=lambda **kw: None)
  cs = DummyCarState(v_ego=2., a_ego=0.)
  cs.canValid, cs.canTimeout, cs.gasPressed = True, False, False
  for k in range(40):
    lc.observe_accel_request(-k * .01, k * .01, authorized=True)
  active, now, kwargs = True, .4, {}
  if reason == 'gas':
    cs.gasPressed = True
  elif reason == 'brake':
    cs.brakePressed = True
  elif reason == 'inactive':
    active = False
  elif reason == 'freeze':
    kwargs['freeze_integrator'] = True
  elif reason == 'plan':
    kwargs['plan_valid'] = False
  elif reason == 'can':
    cs.canValid = False
  elif reason == 'timeout':
    cs.canTimeout = True
  elif reason == 'late':
    now = 1.
  elif reason == 'repeat':
    lc._brake_control_time = now
  elif reason == 'reset':
    lc.reset()
  lc.update(active, cs, -.3, False, -1., (-3.5, 2.), DummyFrogPilotToggles(), request_time=now, **kwargs)
  assert lc._pending_brake_delta == 0.
  assert not lc._brake_requests
  cs.gasPressed = cs.brakePressed = cs.canTimeout = False
  cs.canValid = True
  lc.observe_accel_request(-.5, now, authorized=True)
  for k in range(1, 30):
    lc.update(True, cs, -.3, False, -1., (-3.5, 2.), DummyFrogPilotToggles(), request_time=now + k * .01)
    assert lc._pending_brake_delta == 0.
    lc.observe_accel_request(-.5 - k * .01, now + k * .01, authorized=True)


@pytest.mark.parametrize('bad', ['override', 'positive', 'nan', 'late', 'repeat', 'backward'])
def test_invalid_post_arbitration_observation_cannot_keep_credit(bad):
  lc = LongControl(DummyCarParams())
  for k in range(40):
    lc.observe_accel_request(-k * .01, k * .01, authorized=True)
  lc._pending_brake_delta = -.1
  accel, stamp, authorized = -.5, .4, True
  if bad == 'override':
    authorized = False
  elif bad == 'positive':
    accel = .1
  elif bad == 'nan':
    accel = float('nan')
  elif bad == 'late':
    stamp = 1.
  elif bad == 'repeat':
    stamp = .39
  elif bad == 'backward':
    stamp = .1
  lc.observe_accel_request(accel, stamp, authorized=authorized)
  assert lc._pending_brake_delta == 0.
  assert len(lc._brake_requests) <= 1


def test_gas_override_then_reentry_has_no_phantom_credit_and_preserves_coast():
  controllers = [LongControl(DummyCarParams()), LongControl(DummyCarParams())]
  for lc in controllers:
    lc._service_shadow_tel = StoppingTelemetry(log_fn=lambda **kw: None)
  for frame in range(55):
    gas = frame < 25
    cs = DummyCarState(v_ego=2., a_ego=0.)
    cs.gasPressed, cs.canValid, cs.canTimeout = gas, True, False
    values = []
    for index, lc in enumerate(controllers):
      timing = {'request_time': frame * .01} if index == 1 else {}
      requested = lc.update(True, cs, -.3, True, -1., (-3.5, 2.), DummyFrogPilotToggles(),
        lead_status=True, lead_v=0., lead_d_rel=9. - .02 * frame, lead_track_id=7, lead_model_prob=.99,
        freeze_integrator=gas, model_should_stop=True, **timing)
      final_request = longitudinal_accel_with_gas(requested, True, gas)
      if index == 1:
        assert lc._pending_brake_delta == 0.
        lc.observe_accel_request(final_request, frame * .01, authorized=not gas)
      values.append((requested, final_request, lc._service_shadow_ctx._a_coast, list(lc._service_shadow_ctx._cmd_buf)))
    assert values[0] == values[1]


def test_missing_time_falls_back_exactly_without_changing_coast_input():
  old, new = LongControl(DummyCarParams()), LongControl(DummyCarParams())
  for lc in (old, new):
    lc._service_shadow_tel = StoppingTelemetry(log_fn=lambda **kw: None)
  for frame in range(40):
    new.observe_accel_request(-frame * .01, frame * .01, authorized=True)
  for frame in range(100):
    cs = DummyCarState(v_ego=2., a_ego=-.5)
    cs.gasPressed, cs.canValid, cs.canTimeout = False, True, False
    args = (True, cs, -.3, True, -1., (-3.5, 2.), DummyFrogPilotToggles())
    kwargs = dict(lead_status=True, lead_v=0., lead_d_rel=9. - .02 * frame, lead_track_id=7, lead_model_prob=.99)
    assert old.update(*args, **kwargs) == new.update(*args, **kwargs)
    assert old._service_shadow_ctx._a_coast == new._service_shadow_ctx._a_coast
    assert old._service_shadow_ctx._cmd_buf == new._service_shadow_ctx._cmd_buf
