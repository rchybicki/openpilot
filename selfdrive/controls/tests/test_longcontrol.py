from types import SimpleNamespace

from cereal import car, messaging
from openpilot.selfdrive.controls.lib.longcontrol import LongCtrlState, long_control_state_trans


FROGPILOT_TOGGLES = SimpleNamespace(vEgoStarting=0.5)


def car_params(starting_state=False, gas_interceptor=False):
  # serialized round trip: controlsd reads CarParams through log_from_bytes, so removed fields raise here too
  CP = car.CarParams.new_message(startingState=starting_state, enableGasInterceptorDEPRECATED=gas_interceptor)
  return messaging.log_from_bytes(CP.to_bytes(), car.CarParams)


def long_control_state_trans_test(CP, active, current_state, v_ego, should_stop, brake_pressed, cruise_standstill):
  return long_control_state_trans(
    CP,
    active,
    current_state,
    v_ego=v_ego,
    should_stop=should_stop,
    brake_pressed=brake_pressed,
    cruise_standstill=cruise_standstill,
    frogpilot_toggles=FROGPILOT_TOGGLES,
  )




class TestLongControlStateTransition:

  def test_stay_stopped(self):
    CP = car_params()
    active = True
    current_state = LongCtrlState.stopping
    next_state = long_control_state_trans_test(CP, active, current_state, v_ego=0.1,
                             should_stop=True, brake_pressed=False, cruise_standstill=False)
    assert next_state == LongCtrlState.stopping
    next_state = long_control_state_trans_test(CP, active, current_state, v_ego=0.1,
                             should_stop=False, brake_pressed=True, cruise_standstill=False)
    assert next_state == LongCtrlState.stopping
    next_state = long_control_state_trans_test(CP, active, current_state, v_ego=0.1,
                             should_stop=False, brake_pressed=False, cruise_standstill=True)
    assert next_state == LongCtrlState.stopping
    next_state = long_control_state_trans_test(CP, active, current_state, v_ego=1.0,
                             should_stop=False, brake_pressed=False, cruise_standstill=False)
    assert next_state == LongCtrlState.pid
    active = False
    next_state = long_control_state_trans_test(CP, active, current_state, v_ego=1.0,
                             should_stop=False, brake_pressed=False, cruise_standstill=False)
    assert next_state == LongCtrlState.off

def test_engage():
  CP = car_params()
  active = True
  current_state = LongCtrlState.off
  next_state = long_control_state_trans_test(CP, active, current_state, v_ego=0.1,
                             should_stop=True, brake_pressed=False, cruise_standstill=False)
  assert next_state == LongCtrlState.stopping
  next_state = long_control_state_trans_test(CP, active, current_state, v_ego=0.1,
                             should_stop=False, brake_pressed=True, cruise_standstill=False)
  assert next_state == LongCtrlState.stopping
  next_state = long_control_state_trans_test(CP, active, current_state, v_ego=0.1,
                             should_stop=False, brake_pressed=False, cruise_standstill=True)
  assert next_state == LongCtrlState.stopping
  next_state = long_control_state_trans_test(CP, active, current_state, v_ego=0.1,
                             should_stop=False, brake_pressed=False, cruise_standstill=False)
  assert next_state == LongCtrlState.pid

def test_starting():
  CP = car_params(starting_state=True)
  active = True
  current_state = LongCtrlState.starting
  next_state = long_control_state_trans_test(CP, active, current_state, v_ego=0.1,
                             should_stop=False, brake_pressed=False, cruise_standstill=False)
  assert next_state == LongCtrlState.starting
  next_state = long_control_state_trans_test(CP, active, current_state, v_ego=1.0,
                             should_stop=False, brake_pressed=False, cruise_standstill=False)
  assert next_state == LongCtrlState.pid


def test_stock_cruise_standstill_inactive_stays_off():
  CP = car_params()
  for current_state in (LongCtrlState.off, LongCtrlState.stopping):
    next_state = long_control_state_trans_test(CP, False, current_state, v_ego=0.0,
                                                should_stop=False, brake_pressed=False, cruise_standstill=True)
    assert next_state == LongCtrlState.off


def test_cruise_standstill_holds_until_release():
  CP = car_params(starting_state=True)
  next_state = long_control_state_trans_test(CP, True, LongCtrlState.stopping, v_ego=0.0,
                                             should_stop=False, brake_pressed=False, cruise_standstill=True)
  assert next_state == LongCtrlState.stopping
  next_state = long_control_state_trans_test(CP, True, LongCtrlState.stopping, v_ego=0.0,
                                             should_stop=False, brake_pressed=False, cruise_standstill=False)
  assert next_state == LongCtrlState.starting


def test_gas_interceptor_ignores_cruise_standstill():
  CP = car_params(starting_state=True, gas_interceptor=True)
  next_state = long_control_state_trans_test(CP, True, LongCtrlState.stopping, v_ego=0.0,
                                             should_stop=False, brake_pressed=False, cruise_standstill=True)
  assert next_state == LongCtrlState.starting
