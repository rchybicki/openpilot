from types import SimpleNamespace

from openpilot.selfdrive.controls.lib.longcontrol import LongCtrlState, long_control_state_trans


FROGPILOT_TOGGLES = SimpleNamespace(vEgoStarting=0.5)


def car_params(starting_state=False):
  return SimpleNamespace(enableGasInterceptor=False, startingState=starting_state)


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
