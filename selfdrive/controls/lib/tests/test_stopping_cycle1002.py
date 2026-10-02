"""Cycle 2026-10-02 (corpus cycle_20261002/PLAN.md): the creep-guard handover only after the guard bound, and the Santa Fe
trim hand-off below 2.5 m/s."""
from dataclasses import replace

import pytest

from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.controls.lib import stopping_flags
from openpilot.selfdrive.controls.lib.longcontrol import LongControl, LongCtrlState, SANTA_FE_TRIM_DECAY
from openpilot.selfdrive.controls.lib.tests.test_longcontrol_fast_release import DummyCarParams, DummyCarState, DummyFrogPilotToggles
from openpilot.selfdrive.controls.lib.tests.test_longcontrol_tracking_trim import Plant, _cp, run
from openpilot.selfdrive.controls.tests.test_stopping_stage1 import step

DT = DT_CTRL
EPS = 1e-9


@pytest.fixture
def flags(monkeypatch):
  monkeypatch.setattr(stopping_flags, 'SANTA_FE_TRIM_HANDOFF', True)


# --- B: the creep-guard handover belongs to a wire the guard deepened ------------------------------------------------

def test_guard_that_never_bound_hands_nothing_over(monkeypatch):
  # 0000222e s4: the guard armed under a -3.5 safety plunge and bound 0 frames; its J_GO handover then slowed the service's
  # own post-stop release and reseeded its limiter, so the monitor armed 0.16 deeper
  monkeypatch.setattr(stopping_flags, 'FINAL_FLOOR', True)
  cp = DummyCarParams()
  cp.longitudinalTuning.kpV = [0.0]
  lc = LongControl(cp)
  lc.long_control_state = LongCtrlState.pid
  update = lc._service_shadow_svc.update
  monkeypatch.setattr(lc._service_shadow_svc, 'update', lambda **kw: replace(update(**kw), accel=-2.0))
  for _ in range(150):
    step(lc, v=1.2)
  assert lc._final_floor_armed and lc._final_floor_bound_frames == 0 and lc.last_output_accel == pytest.approx(-2.0)
  monkeypatch.setattr(lc._service_shadow_svc, 'update', lambda **kw: replace(update(**kw), accel=-1.0))
  wire = step(lc, v=0.0)  # wheel stop: the guard disarms; the service's own shallower value passes at once
  assert not lc._final_floor_armed and not lc._final_floor_releasing
  assert wire == pytest.approx(-1.0)


# --- D: Santa Fe trim hand-off below 2.5 m/s -------------------------------------------------------------------------

def wound_controller():
  lc = LongControl(_cp())
  run(lc, n=int(8.0 / DT), demand=-1.80, gain=0.80, delay_s=0.45)  # a sustained shortfall (a downhill) winds the trim
  assert lc._trim_i <= -0.25
  return lc


def approach(lc, *, n, demand, v, lead_v, gas=False, should_stop=False, gain=0.80):
  plant = Plant(gain, 0.45, 0.5)
  out = []
  for i in range(n):
    t = i * DT
    vv = v(t) if callable(v) else v
    d = demand(t) if callable(demand) else demand
    lv = lead_v(t) if callable(lead_v) else lead_v
    cs = DummyCarState(v_ego=vv, a_ego=plant.a)
    cs.gasPressed = gas
    wire = float(lc.update(True, cs, d, should_stop, -1.0, (-3.5, 2.0), DummyFrogPilotToggles(), lead_status=True,
                           lead_v=lv, lead_d_rel=8.0, lead_track_id=7, lead_model_prob=1.0, freeze_integrator=gas))
    plant.step(wire)
    out.append((wire, float(lc._trim_i), lc.long_control_state))
  return out


def test_trim_is_held_below_v_min_behind_a_moving_lead(flags):
  lc = wound_controller()
  t0 = lc._trim_i
  rec = approach(lc, n=int(1.0 / DT), demand=-0.5, v=2.4, lead_v=1.0)
  assert all(trim == pytest.approx(t0) for _, trim, _ in rec)


def test_trim_decays_below_v_min_with_the_flag_off(monkeypatch):
  monkeypatch.setattr(stopping_flags, 'SANTA_FE_TRIM_HANDOFF', False)
  lc = wound_controller()
  rec = approach(lc, n=int(1.0 / DT), demand=-0.5, v=2.4, lead_v=1.0)
  assert rec[-1][1] == 0.0


@pytest.mark.parametrize('case', ['stopped_lead', 'gas', 'planner_eases'])
def test_no_hold_and_a_continuous_decay(flags, case):
  lc = wound_controller()
  t0 = lc._trim_i
  kw = dict(demand=-0.5, v=2.4, lead_v=1.0)
  if case == 'stopped_lead':
    kw['lead_v'] = 0.0  # behind an already stopped lead the service enters at the gate with its own a_coast
  elif case == 'gas':
    kw['gas'] = True
  else:
    kw['demand'] = -0.2
  rec = approach(lc, n=int(1.0 / DT), **kw)
  prev = t0
  for _, trim, _ in rec:
    assert trim >= prev - EPS  # never held deeper
    if case != 'stopped_lead':  # (behind a stopped lead the service takes over and zeroes the state: no wire effect)
      assert trim <= prev + SANTA_FE_TRIM_DECAY * DT + EPS  # decays, never steps
    prev = trim
  assert rec[-1][1] == 0.0


def test_held_trim_then_planner_release_never_deepens(flags):
  # red-team blocker: the low-speed slew must use the previous wire WITHOUT the trim, or a held trim integrates at a go
  lc = wound_controller()
  rec = approach(lc, n=int(1.5 / DT), demand=-0.5, v=lambda t: 2.4 - 0.9 * t, lead_v=0.9)
  assert rec[-1][1] < -0.2  # still held at 1.05 m/s
  start = rec[-1][0]
  go = approach(lc, n=int(1.0 / DT), demand=lambda t: min(-0.4 + 2.3 * t, 0.5), v=1.05, lead_v=lambda t: 0.9 + 2.0 * t)
  assert min(wire for wire, _, _ in go) >= start - EPS


def test_stopping_state_below_v_min_does_not_integrate_the_trim(flags):
  lc = wound_controller()
  approach(lc, n=int(0.5 / DT), demand=-0.5, v=2.4, lead_v=1.0)
  assert lc._trim_i < -0.2
  rec = approach(lc, n=int(0.5 / DT), demand=-0.5, v=2.3, lead_v=0.0, should_stop=True)
  stopping = [wire for wire, _, state in rec if state == LongCtrlState.stopping]
  assert stopping, 'the legacy state machine never entered stopping'
  assert min(stopping) > -2.0 + EPS  # HEAD integrates the trim every frame toward stopAccel
