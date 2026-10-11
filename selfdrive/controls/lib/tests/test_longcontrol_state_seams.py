"""LongControl state-machine seams under a service-owned stop (cycle 2026-10-10, WS-C3 / WS-C4; cycle_20261009/PLAN.md sections 18-23).

OWNED_STARTING_TRANSIT: shouldStop drops as the lead departs while the service owns a rolling approach; LongControl's one-frame
'starting' transit (stopping -> starting -> pid above vEgoStarting) handed the frame to the launch command (unslewed above 1.2 m/s)
and the next owned frame reseeded the service's limiter on it. With the flag the transit frame stays owned and the hand-off is the
service's own J_UP / RELEASE J_GO.

SERVICE_HOLD_ENTERS_STOPPING: a car resting > 5 m behind a stopped lead meets the arbiter's far-stopped-lead release, so the state
machine stayed in pid under the service's hold (no StopReq until vEgo dithered below 0: 1.9-10.9 s on the car, drives 2234_701.09,
225d_1229.52, 225f_1110.20). With the flag the owned hold enters stopping (StopReq from the Hyundai sender); the departure keeps the
service's RELEASE ramp and ends in today's pid (never the 'starting' launch command: runner ms_v*_far), and at rest while the planner
still stops the state waits for the service to re-hold (no StopReq toggle)."""
import numpy as np
import pytest

from openpilot.selfdrive.controls.lib import stopping_flags
from openpilot.selfdrive.controls.lib.longcontrol import LongControl, LongCtrlState
from openpilot.selfdrive.controls.lib.stopping_service import Phase, ServiceParams
from openpilot.selfdrive.controls.lib.stopping_telemetry import StoppingTelemetry
from openpilot.selfdrive.controls.lib.tests.test_longcontrol_fast_release import DummyCarParams, DummyCarState, DummyFrogPilotToggles
from opendbc.car.hyundai.tests.test_can_bounds_fork import get_signal, make_controller, run_frame

P = ServiceParams()
DT = 0.01


def _lc():
  cp = DummyCarParams()
  cp.startingState = True
  lc = LongControl(cp)
  lc._service_shadow_tel = StoppingTelemetry(log_fn=lambda **kw: None)
  return lc


@pytest.mark.parametrize("on", [False, True])
@pytest.mark.parametrize("v", [0.9, 1.25, 2.0])
def test_owned_approach_keeps_the_wire_through_the_starting_transit(monkeypatch, on, v):
  # q_2073_1098 (cycle 2026-10-09 v3 B) and the runner's go_v8/v12_t2.2_a2 (HEAD -1.15 -> +0.70 -> -0.04 at 1.21 m/s): the lead
  # departs while the service owns a rolling approach
  monkeypatch.setattr(stopping_flags, "SERVICE_MODE", "LIVE")
  monkeypatch.setattr(stopping_flags, "OWNED_STARTING_TRANSIT", on)
  lc = _lc()
  toggles = DummyFrogPilotToggles()
  lc.last_output_accel = -0.8
  wires, states, owned = [], [], []
  for k in range(140):
    stop = k < 100
    wires.append(lc.update(True, DummyCarState(v_ego=v, a_ego=-0.8), -0.8 if stop else 0.3, stop, 1.5 if stop else None, (-3.5, 2),
                           toggles, experimental_mode=True, lead_status=True, lead_v=0.0 if stop else 1.0, lead_d_rel=5.6, lead_a=0,
                           lead_track_id=7, lead_model_prob=0.99, increased_stopped_distance=0.3, a_target_trajectory=-0.8 if stop else 0.3))
    states.append(lc.long_control_state)
    owned.append(lc._service_live_owning)
  k_start = states.index(LongCtrlState.starting)
  assert k_start >= 100 and states[k_start + 1] == LongCtrlState.pid and all(owned[90:k_start])
  step = max(np.diff(wires[90:k_start + 10]))
  if on:
    assert all(owned[90:k_start + 3])
    assert step <= P.J_UP * DT + 1e-6
  elif v >= 1.2:
    assert wires[k_start] == pytest.approx(toggles.startAccel)   # the HEAD seam: the launch command, unslewed, for one frame
    assert not owned[k_start] and step > 1.0
  else:
    assert step <= 0.03 + 1e-6                                   # below 1.2 m/s the low-speed slew bounds the same frame


def test_owned_starting_transit_leaves_a_legacy_transit_alone(monkeypatch):
  # the flag keys on the previous frame's service ownership only: a stopping -> starting transit of the legacy chain (service not
  # owning) is today's
  monkeypatch.setattr(stopping_flags, "SERVICE_MODE", "LIVE")
  runs = {}
  for on in (False, True):
    monkeypatch.setattr(stopping_flags, "OWNED_STARTING_TRANSIT", on)
    lc = _lc()
    toggles = DummyFrogPilotToggles()
    lc.long_control_state = LongCtrlState.stopping
    lc.last_output_accel = -0.5
    out = []
    for k in range(20):
      stop = k < 5
      out.append(lc.update(True, DummyCarState(v_ego=3.0, a_ego=-0.5), -0.5 if stop else 0.3, stop, 1.5 if stop else None, (-3.5, 2),
                           toggles, experimental_mode=True, lead_status=False, a_target_trajectory=-0.5 if stop else 0.3))
      assert not lc._service_live_owning
    runs[on] = out
  assert runs[True] == runs[False] and any(w == pytest.approx(DummyFrogPilotToggles().startAccel) for w in runs[False])


def _far_hold(monkeypatch, on, n_hold=300, n=800, fault=None):
  """Owned service HOLD at rest 7.5 m behind a stopped lead (the far-stopped-lead release keeps the state machine out of stopping),
  then the lead departs at 1 m/s with the planner's go. fault: frame -> extra update kwargs (an input fault). Returns per frame
  (state, phase, StopReq, wire)."""
  monkeypatch.setattr(stopping_flags, "SERVICE_MODE", "LIVE")
  monkeypatch.setattr(stopping_flags, "SERVICE_HOLD_ENTERS_STOPPING", on)
  lc = _lc()
  lc.long_control_state = LongCtrlState.pid
  lc.last_output_accel = P.A_HOLD
  lc._service_live_owning = True
  svc = lc._service_shadow_svc
  svc.phase, svc._last_cmd, svc._hold_entry_gap = Phase.HOLD, P.A_HOLD, 7.5
  sender, _ = make_controller()
  toggles = DummyFrogPilotToggles()
  gap, stop_req, rows = 7.5, 0, []
  for k in range(n):
    go = k >= n_hold
    lead_v = 1.0 if go else 0.0
    gap += lead_v * DT
    state = lc.long_control_state
    kw = dict(experimental_mode=True, lead_status=True, lead_v=lead_v, lead_d_rel=gap, lead_a=0, lead_track_id=7, lead_model_prob=0.99,
              increased_stopped_distance=0.3, a_target_trajectory=0.35 if go else -0.02)
    a = float(lc.update(True, DummyCarState(v_ego=0.0, a_ego=0.0, standstill=True), 0.35 if go else -0.02, not go, None if go else 3.2,
                        (-3.5, 2.0), toggles, **(kw | (fault(k) if fault else {}))))
    _, msgs = run_frame(sender, dict(accel=a, state=state, v_ego=0.0, a_ego=0.0, long_active=True))
    if 0x421 in msgs:
      stop_req = int(get_signal('SCC12', 'StopReq', msgs[0x421]))
    rows.append((lc.long_control_state, svc.phase, stop_req, a))
  return rows


def test_service_hold_at_rest_requests_the_managed_hold(monkeypatch):
  head = _far_hold(monkeypatch, False)
  flag = _far_hold(monkeypatch, True)
  # HEAD: the hold runs in pid, no StopReq
  assert all(s == LongCtrlState.pid and r == 0 for s, ph, r, _ in head[:300])
  # flag: stopping from the first hold frame, StopReq from the next SCC frame, held through the hold
  assert all(s == LongCtrlState.stopping for s, ph, r, _ in flag[:300]) and all(r == 1 for s, ph, r, _ in flag[3:300])
  # the planner's go: the service's RELEASE runs in today's pid (StopReq ends with the hold), never 'starting'
  k_rel = next(k for k, (s, ph, r, _) in enumerate(flag) if ph == Phase.RELEASE)
  assert all(s == LongCtrlState.pid for s, *_ in flag[k_rel + 1:]) and all(r == 0 for s, ph, r, _ in flag[k_rel + 2:])
  # one StopReq episode: set once, cleared once at the departure (no chatter)
  sr = [r for _, _, r, _ in flag]
  assert sum(1 for a, b in zip(sr, sr[1:], strict=False) if a != b) == 2
  # departure unchanged: the same hold, RELEASE ramp and launch command
  assert [w for *_, w in flag] == [w for *_, w in head]


@pytest.mark.parametrize("fault", [dict(plan_valid=False), dict(lead_a=float("nan"))])
@pytest.mark.parametrize("frames", [1, 5])
def test_an_input_fault_inside_the_held_stop_never_launches(monkeypatch, fault, frames):
  # bundle-3 verifier (cycle 2026-10-11): an input-fault frame's reset() cleared the hold latch while the state stayed stopping, so
  # the far-stopped-lead release took stopping -> 'starting' (StopReq off, the launch command at rest behind the stopped lead). Flag
  # off: the hold runs in pid and keeps braking. Flag on: the stop stays stopping with StopReq, braking as deep as today
  def faulty(k):
    return fault if 150 <= k < 150 + frames else {}
  head = _far_hold(monkeypatch, False, n_hold=800, fault=faulty)
  flag = _far_hold(monkeypatch, True, n_hold=800, fault=faulty)
  assert all(s == LongCtrlState.pid for s, *_ in head) and max(w for *_, w in head[150:]) < -0.6
  assert all(s == LongCtrlState.stopping for s, *_ in flag) and all(r == 1 for _, _, r, _ in flag[3:])
  assert all(f <= h + 1e-9 for (*_, f), (*_, h) in zip(flag, head, strict=True))
  assert flag[-1][1] == Phase.HOLD


def test_service_hold_enters_stopping_only_from_an_owned_hold(monkeypatch):
  # not owned (observing) or under the driver's gas: today's state machine
  monkeypatch.setattr(stopping_flags, "SERVICE_MODE", "LIVE")
  monkeypatch.setattr(stopping_flags, "SERVICE_HOLD_ENTERS_STOPPING", True)
  for owning, gas in ((False, False), (True, True)):
    lc = _lc()
    lc.long_control_state = LongCtrlState.pid
    lc.last_output_accel = P.A_HOLD
    lc._service_live_owning = owning
    svc = lc._service_shadow_svc
    svc.phase, svc._last_cmd, svc._hold_entry_gap = Phase.HOLD, P.A_HOLD, 7.5
    cs = DummyCarState(v_ego=0.0, a_ego=0.0, standstill=True)
    cs.gasPressed = gas
    lc.update(True, cs, -0.02, True, 3.2, (-3.5, 2.0),
              DummyFrogPilotToggles(), experimental_mode=True, lead_status=True, lead_v=0.0, lead_d_rel=7.5, lead_a=0, lead_track_id=7,
              lead_model_prob=0.99, increased_stopped_distance=0.3, a_target_trajectory=-0.02, freeze_integrator=gas)
    assert lc.long_control_state == LongCtrlState.pid



@pytest.mark.parametrize("on", [False, True])
@pytest.mark.parametrize("gas", [False, True])
def test_held_stop_waits_for_the_service_or_ends_in_todays_pid(monkeypatch, on, gas):
  # after the service's RELEASE ended (INACTIVE) at rest 7.5 m behind the stopped lead while the planner still stops (runner
  # ms_v*_far: the service re-holds on the next frames): the held stop stays stopping, no StopReq toggle; under the driver's gas it
  # ends in today's pid. Flag off (a stopping state entered by the state machine): 'starting', the launch command at rest
  monkeypatch.setattr(stopping_flags, "SERVICE_MODE", "LIVE")
  monkeypatch.setattr(stopping_flags, "SERVICE_HOLD_ENTERS_STOPPING", on)
  lc = _lc()
  lc.long_control_state = LongCtrlState.stopping
  lc._hold_stopping = on
  lc.last_output_accel = -0.02
  cs = DummyCarState(v_ego=0.0, a_ego=0.0, standstill=True)
  cs.gasPressed = gas
  lc.update(True, cs, -0.02, True, 3.2, (-3.5, 2.0), DummyFrogPilotToggles(), experimental_mode=True, lead_status=True, lead_v=0.0,
            lead_d_rel=7.5, lead_a=0, lead_track_id=7, lead_model_prob=0.99, increased_stopped_distance=0.3, a_target_trajectory=-0.02,
            freeze_integrator=gas)
  assert lc.long_control_state == (LongCtrlState.starting if not on else LongCtrlState.pid if gas else LongCtrlState.stopping)


def _hold_then_launch(monkeypatch, on, n_hold=300, n=700):
  """Owned service HOLD at rest 7.5 m behind a stopped lead (state machine in pid: the far-stopped-lead release) while the planner no
  longer stops (shouldStop clear, aTarget +0.1 below the go threshold: syn_launch's re-hold), then the lead departs at 1.5 m/s with
  the planner's go; the car creeps off as the service releases (plant: wire + 0.4 creep). Returns per frame (state, phase, wire, v)."""
  monkeypatch.setattr(stopping_flags, "SERVICE_MODE", "LIVE")
  monkeypatch.setattr(stopping_flags, "SERVICE_HOLD_ENTERS_STOPPING", on)
  lc = _lc()
  lc.long_control_state = LongCtrlState.pid
  lc.last_output_accel = P.A_HOLD
  lc._service_live_owning = True
  svc = lc._service_shadow_svc
  svc.phase, svc._last_cmd, svc._hold_entry_gap = Phase.HOLD, P.A_HOLD, 7.5
  toggles = DummyFrogPilotToggles()
  gap, v, a_ego, rows = 7.5, 0.0, 0.0, []
  for k in range(n):
    go = k >= n_hold
    lead_v = 1.5 if go else 0.0
    gap += (lead_v - v) * DT
    a_tgt = min(0.3 + 0.01 * (k - n_hold), 1.5) if go else 0.1
    wire = float(lc.update(True, DummyCarState(v_ego=v, a_ego=a_ego, standstill=v < 0.01), a_tgt, False, None, (-3.5, 2.0), toggles,
                           experimental_mode=True, lead_status=True, lead_v=lead_v, lead_d_rel=gap, lead_a=0, lead_track_id=7,
                           lead_model_prob=0.99, increased_stopped_distance=0.3, a_target_trajectory=a_tgt))
    a_ego = wire + 0.4 if (v > 0.0 or wire + 0.4 > 0.0) else 0.0
    v = max(v + a_ego * DT, 0.0)
    rows.append((lc.long_control_state, svc.phase, wire, v))
  return rows


def test_service_hold_stopping_is_not_stop_intent_for_the_launch(monkeypatch):
  # runner syn_launch / 2232_s40 (cycle 2026-10-10 bundle): the hold's stopping frames restarted the arbiter's 1.0 s stop-intent
  # window, so the low-speed slew withheld the fast release once the car rolled (0.47 m/s slower within 4 s). The flag's stopping
  # episode is StopReq protocol only: the arbiter sees today's pid and the launch is today's, frame for frame
  head = _hold_then_launch(monkeypatch, False)
  flag = _hold_then_launch(monkeypatch, True)
  assert all(s == LongCtrlState.stopping for s, *_ in flag[:300]) and all(s == LongCtrlState.pid for s, *_ in head[:300])
  assert max(r[3] for r in head) > 1.0                     # the car really launched (moving, out of the standstill window)
  assert [r[2] for r in flag] == [r[2] for r in head]
