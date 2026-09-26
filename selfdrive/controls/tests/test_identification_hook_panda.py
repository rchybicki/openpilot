"""KCS1 brake-response test program, the whole command path on the host (no device, no files): the real LongControl (flag
on; LongControl builds the hook, the wheel button arms and starts it) -> the controlsd composition (min with
max_desired_acceleration, longitudinal_accel_with_gas, actuators.longControlState written one frame behind) -> the real
Hyundai CarController (SCC12/SCC14) -> Panda safety (TestHyundaiLongitudinalSafety). A pure-delay plant closes the loop."""
from types import SimpleNamespace

import numpy as np
import pytest

from opendbc.car.hyundai.tests.test_can_bounds_fork import SCC12_ADDR, SCC14_ADDR, get_signal, make_cc, make_controller, make_cs
from opendbc.car.hyundai.values import Buttons
from opendbc.safety import ALTERNATIVE_EXPERIENCE
from opendbc.safety.tests import test_hyundai   # module import: pytest must not collect its TestCase classes here
from opendbc.safety.tests.libsafety import libsafety_py
from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.controls.lib import identification_hook as ih, stopping_flags
from openpilot.selfdrive.controls.lib.drive_helpers import (longitudinal_accel_with_gas, longitudinal_control_active,
                                                             longitudinal_control_override)
from openpilot.selfdrive.controls.lib.longcontrol import LongControl, LongCtrlState
from openpilot.selfdrive.controls.lib.tests.test_longcontrol_fast_release import DummyCarParams, DummyCarState, DummyFrogPilotToggles
from openpilot.selfdrive.controls.tests.test_identification_hook import ARM, START, T0, good

MAX_DESIRED = 2.0                   # frogpilot_toggles.max_desired_acceleration (controlsd min): never binds while braking
STANDSTILL_V = 12 * 0.03125 / 3.6   # Hyundai CarState.standstill: every wheel speed at or below 0.104 m/s
DELAY = 45                          # frames: the plant's acceleration is the sent command 0.45 s earlier
GO = 0.3                            # the planner's aTarget below the set speed: it wants the 20 km/h cruise back
SEGS = {m[0]: m[2] for m in ih.MANEUVERS}


class Session:
  """controlsd, card and the Panda for one 100 Hz frame at a time, around one real LongControl and CarController."""

  def __init__(self, monkeypatch, maneuver="B", alt_exp=ALTERNATIVE_EXPERIENCE.DEFAULT):
    monkeypatch.setattr(stopping_flags, "IDENTIFICATION_HOOK", True)
    cp = DummyCarParams()
    cp.longitudinalTuning.kpV = [0.0]                                   # the Santa Fe HEV runs kp = ki = 0
    self.lc = LongControl(cp)
    self.lc.long_control_state = LongCtrlState.pid
    self.hook = self.lc._id_hook
    self.hook.load({"plan": ih.PLAN_ID, "done": {m[0]: int(m[0] != maneuver) for m in ih.MANEUVERS}})   # `maneuver` is next
    self.with_gas = alt_exp == ALTERNATIVE_EXPERIENCE.LONGITUDINAL_ACTIVE_WITH_GAS
    self.panda = test_hyundai.TestHyundaiLongitudinalSafety("test_no_aeb_scc12")
    self.panda.setUp()
    self.panda.safety.set_alternative_experience(alt_exp)
    self.panda.safety.set_controls_allowed(True)
    self.panda._rx(self.panda._speed_msg(5.56))
    self.controller, _ = make_controller()
    self.controller.frame, self.controller.engaged_frame = 1000, 1    # past the post-engagement launch cap
    self.v, self.a, self.sent = 5.56, 0.0, [0.0] * DELAY               # a steady 20 km/h cruise until the rep starts
    self.moving, self.rows = False, []

  def frame(self, *, enabled=True, gas=False, brake=False, pressed=False, a_target=0.0):
    lagged = self.lc.long_control_state                                 # actuators.longControlState is written before LoC.update
    long_active = longitudinal_control_active(enabled, True, False, gas, self.with_gas, gas)   # gas = the override event
    if not long_active:
      self.lc.reset()
    standstill = self.v <= STANDSTILL_V
    inputs = good(v_ego=self.v, a_ego=self.a, standstill=standstill, pid_state=lagged == LongCtrlState.pid, distance_pressed=pressed,
                  gas=gas, brake=brake, enabled=enabled, long_active=long_active, plan_accel=a_target)
    wire = float(self.lc.update(long_active, DummyCarState(v_ego=self.v, a_ego=self.a, brake_pressed=brake, standstill=standstill),
                                a_target, False, -1.0, (-3.5, 2.0), DummyFrogPilotToggles(), id_inputs=inputs,
                                freeze_integrator=self.with_gas and gas))
    accel = longitudinal_accel_with_gas(min(wire, MAX_DESIRED), self.with_gas, gas)
    override = longitudinal_control_override(enabled, True, long_active, self.with_gas, gas)
    _, sends = self.controller.update(make_cc(accel=accel, state=lagged, long_active=long_active, enabled=enabled, override=override),
                                      make_cs(v_ego=self.v, a_ego=self.a, gas_pressed=gas), 0, SimpleNamespace())
    row = SimpleNamespace(v=self.v, lagged=lagged, wire=wire, accel=accel, out=self.lc.id_hook_out, scc12=None, jerk=None,
                          rejected=[addr for addr, dat, bus in sends
                                    if not self.panda.safety.safety_tx_hook(libsafety_py.make_CANPacket(addr, bus, dat))])
    for addr, dat, _bus in sends:
      if addr == SCC12_ADDR:
        row.scc12 = {sig: get_signal("SCC12", sig, dat) for sig in ("aReqValue", "StopReq", "ACCMode")}
      elif addr == SCC14_ADDR:
        row.jerk = get_signal("SCC14", "JerkUpperLimit", dat)
    self.rows.append(row)
    self.moving = self.moving or row.out.state == "ACTIVE"             # the plant runs from the first ACTIVE frame
    if self.moving:
      self.sent.append(accel)
      a = self.sent.pop(0)
      self.v = max(0.0, self.v + a * DT_CTRL)
      self.a = a if self.v > 0.0 else 0.0
    return row

  def start(self):
    for k in range(T0 + 1):
      self.frame(pressed=k in ARM or k in START)
    assert self.hook.state == "ACTIVE" and not any(r.rejected for r in self.rows)
    return self.rows[-1]

  def rep(self, until, limit=3000):
    """ACTIVE frames (the planner wants the cruise back) until until(row); every row returned"""
    rows = []
    while not rows or not until(rows[-1]):
      rows.append(self.frame(a_target=GO if self.v < 5.5 else 0.0))
      assert len(rows) < limit
    return rows

  def hold(self, frames):
    """to HELD, then `frames` (+1) frames of the hold: the next frame (the driver's action) sends SCC"""
    rows = self.rep(lambda r: r.out.state == "HELD") + [self.frame(a_target=GO) for _ in range(frames)]
    return rows + ([self.frame(a_target=GO)] if self.controller.frame % 2 else [])


def _off(s, frames, **kw):
  return [s.frame(enabled=False, a_target=GO, **kw) for _ in range(frames)]


@pytest.mark.parametrize("maneuver", ["B", "E"])
def test_a_full_rep_sends_the_scripted_floor_and_ends_in_an_accepted_held_stop(monkeypatch, maneuver):
  s, segs = Session(monkeypatch, maneuver), SEGS[maneuver]
  assert segs[-1].accel <= ih.A_HOLD                                   # the hold keeps the last command (nothing to deepen)
  rep = [s.start()] + s.hold(150)
  assert all(r.out.state in ("ACTIVE", "HELD") for r in rep) and rep[-1].out.state == "HELD"
  script = [segs[r.out.seg - 1].accel if r.out.state == "ACTIVE" else segs[-1].accel for r in rep]
  assert list(dict.fromkeys(r.out.seg for r in rep)) == list(range(1, len(segs) + 1))   # every segment, in order, as scripted:
  for n, seg in enumerate(segs[:-1], start=1):
    frames = [r for r in rep if r.out.seg == n]
    assert len(frames) == round(seg.t_s / DT_CTRL) if seg.t_s else frames[-1].v > seg.v_end >= rep[rep.index(frames[-1]) + 1].v
  # the scripted floor is the wire and survives the controlsd composition; every SCC12 carries it (0.01 steps), accepted
  assert [r.out.floor for r in rep] == [r.wire for r in rep] == [r.accel for r in rep] == script
  sent = [(r, f) for r, f in zip(rep, script, strict=True) if r.scc12]
  assert sent and all(r.scc12["aReqValue"] == pytest.approx(round(f, 2), abs=1e-6) and r.scc12["ACCMode"] == 1 for r, f in sent)
  assert not any(r.rejected for r in s.rows)
  # stop intent -> LongControl stopping on the next frame -> actuators one frame later: SCC14 jerk 3.0 before, 1.0 after
  i = next(n for n, r in enumerate(rep) if r.out.stop_intent)
  assert all(r.lagged == (LongCtrlState.pid if n <= i + 1 else LongCtrlState.stopping) for n, r in enumerate(rep))
  assert all(r.jerk == pytest.approx(3.0 if n <= i + 1 else 1.0) for n, r in enumerate(rep) if r.jerk is not None)
  # StopReq: 0 while moving; latched in HELD from the first frame below 0.01 m/s (as the sender sees it: a float32 CarState)
  j = next(n for n, r in enumerate(rep) if float(np.float32(r.v)) < 0.01)
  assert rep[j].out.state == "HELD" and any(r.scc12 for r in rep[j:])
  assert all(r.scc12["StopReq"] == (n >= j) for n, r in enumerate(rep) if r.scc12)
  # the driver's brake ends the hold: disengaged, zero request, ACCMode 0 (the sender's disengaged value), accepted, counted
  s.panda._rx(s.panda._user_brake_msg(True))
  brake = s.frame(enabled=False, brake=True, a_target=GO)
  assert not s.panda.safety.get_controls_allowed()
  assert brake.wire == brake.accel == 0.0 and brake.scc12["aReqValue"] == pytest.approx(0.0, abs=1e-6) and brake.scc12["ACCMode"] == 0
  assert not brake.rejected and brake.out.rep_done == maneuver and s.hook.done[maneuver] == 1 and s.hook.state == "ARMED"
  assert all(r.accel == 0.0 and not r.rejected for r in _off(s, 50, brake=True))



# planner_holds: before 2026-09-26 LongControl stayed in stopping while inactive (dropout hold) and kept -0.7 after the brake
@pytest.mark.parametrize("plan", [GO, 0.0],
                         ids=["planner_go", "planner_holds"])
def test_the_brake_frame_after_a_hold_sends_only_accepted_zeros(monkeypatch, plan):
  s = Session(monkeypatch)
  s.start()
  s.hold(120)
  s.panda._rx(s.panda._user_brake_msg(True))
  off = [s.frame(enabled=False, brake=True, a_target=plan) for _ in range(50)]
  assert [r.wire for r in off] == [r.accel for r in off] == [0.0] * 50
  assert not any(r.rejected for r in off) and off[0].scc12["ACCMode"] == 0 and s.hook.done["B"] == 1


@pytest.mark.parametrize("alt_exp", [ALTERNATIVE_EXPERIENCE.DEFAULT, ALTERNATIVE_EXPERIENCE.LONGITUDINAL_ACTIVE_WITH_GAS],
                         ids=["default", "active_with_gas"])
def test_gas_in_the_hold_sends_no_braking_and_no_stop_request(monkeypatch, alt_exp):
  s = Session(monkeypatch, alt_exp=alt_exp)
  s.start()
  assert [r for r in s.hold(120) if r.scc12][-1].scc12["StopReq"] == 1     # latched before the pedal
  s.panda._rx(s.panda._user_gas_msg(1))
  gas = [s.frame(gas=True, a_target=GO) for _ in range(20)]
  frames = [r for r in gas if r.scc12]
  assert frames and all(r.scc12["aReqValue"] >= 0.0 and r.scc12["StopReq"] == 0 for r in frames)
  assert all(r.accel >= 0.0 and not r.rejected for r in gas)
  assert gas[0].out.floor is None and gas[0].out.reason == "pedal" and s.hook.state == "OFF" and s.hook.done["B"] == 0


@pytest.mark.parametrize("past_intent", [False, True], ids=["before_intent", "after_intent"])
def test_cruise_cancel_during_a_rep_sends_only_accepted_zeros(monkeypatch, past_intent):
  s = Session(monkeypatch)
  s.start()
  s.rep(lambda r: r.out.stop_intent if past_intent else s.hook._rep_t >= 1.0)
  assert s.hook.state == "ACTIVE" and s.rows[-1].accel == -1.0
  s.panda._rx(s.panda._button_msg(Buttons.CANCEL))
  off = _off(s, 200)
  assert not s.panda.safety.get_controls_allowed()
  assert [r.wire for r in off] == [r.accel for r in off] == [0.0] * 200 and not any(r.rejected for r in off)
  frames = [r.scc12 for r in off if r.scc12]
  assert frames and all(f["aReqValue"] == pytest.approx(0.0, abs=1e-6) and f["ACCMode"] == 0 and f["StopReq"] == 0 for f in frames)
  assert off[0].out.reason == "disengaged" and s.hook.state == "OFF" and s.hook.done["B"] == 0


@pytest.mark.parametrize("past_intent", [False, True], ids=["before_intent", "after_intent"])
def test_cancel_while_every_frame_raises_sends_only_accepted_zeros(monkeypatch, past_intent):
  # review 20260926-111812 finding 1: a persistent hook exception must not keep a request after cruise cancel
  s = Session(monkeypatch)
  s.start()
  s.rep(lambda r: r.out.stop_intent if past_intent else s.hook._rep_t >= 1.0)
  monkeypatch.setattr(ih, "precondition_failure", lambda *a, **k: (_ for _ in ()).throw(RuntimeError("boom")))
  first = s.frame(a_target=GO)
  assert first.accel == -1.0 and not first.rejected and s.hook._locked == "exception"
  assert s.hook.state == ("ACTIVE" if past_intent else "HANDBACK")   # after the intent the floor stays; before it, a release
  s.panda._rx(s.panda._button_msg(Buttons.CANCEL))
  off = _off(s, 200)
  assert [r.wire for r in off] == [r.accel for r in off] == [0.0] * 200 and not any(r.rejected for r in off)
  assert s.hook.state == "LOCKED" and all(r.out.floor is None for r in off)
