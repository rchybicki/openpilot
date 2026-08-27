"""Universal stop governor (SHADOW this cycle): law properties, barrier shape, telemetry bounds, and the
structural pin that the governor never reaches the wire (docs/stopping/universal_stop_program.md)."""
import math

import pytest

from openpilot.selfdrive.controls.lib import stopping_service as svc
from openpilot.selfdrive.controls.lib.stopping_service import (
  GOV_A_C, GOV_A_MAX, GOV_A_UP, GOV_LAG, GOV_TAU, barrier_demand, governor_demand,
)
from openpilot.selfdrive.controls.lib.stopping_telemetry import MAX_GOV_TRACE_ENTRIES, StoppingTelemetry


def test_governor_far_away_is_the_sqrt_closure_curve_and_near_is_linear():
  # far: v_ref ~ v_lead + sqrt(2 A_C d) (within the TAU shaping), a_ff -> -A_C
  a, v_ref, q_ref, d = governor_demand(v=8.0, v_lead=0.0, gap=60.0, isd=0.3)
  assert d == pytest.approx(60.0 - 4.3 - GOV_LAG * 8.0)
  assert q_ref == pytest.approx(math.sqrt(2 * GOV_A_C * d) - GOV_A_C * GOV_TAU, abs=0.05)
  a_ff = -GOV_A_C * q_ref / (q_ref + GOV_A_C * GOV_TAU)
  assert a_ff == pytest.approx(-GOV_A_C, abs=0.06)
  # near the anchor: q_ref -> d / TAU, a_ff -> 0, no singularity at d = 0
  _, v_ref0, q_ref0, d0 = governor_demand(v=0.3, v_lead=0.0, gap=4.3 + GOV_LAG * 0.3, isd=0.3)
  assert d0 == pytest.approx(0.0, abs=1e-9) and q_ref0 == pytest.approx(0.0, abs=1e-9) and v_ref0 == 0.0
  _, _, q_small, d_small = governor_demand(v=0.3, v_lead=0.0, gap=4.3 + GOV_LAG * 0.3 + 0.08, isd=0.3)
  assert q_small == pytest.approx(d_small / GOV_TAU, rel=0.12)
  # monotone in the gap
  prev = -1.0
  for gap in (4.5, 5.0, 6.0, 8.0, 12.0, 20.0):
    _, v_ref, _, _ = governor_demand(1.0, 0.0, gap, 0.3)
    assert v_ref > prev
    prev = v_ref


def test_governor_never_chases_and_is_bounded():
  # the cycle-33/today 's30' geometry: 1.3 m/s at gap 4.7 with the lead just stopped -- the planner lane slammed
  # -0.74 -> -1.99 in one frame; the governor asks a bounded, continuous ~-1.2
  a, *_ = governor_demand(1.3, 0.26, 4.7, 0.3)
  assert -1.6 <= a <= -0.9
  # authority bounds
  a_hot, *_ = governor_demand(12.0, 0.0, 8.0, 0.3)
  assert a_hot == -GOV_A_MAX
  a_up, *_ = governor_demand(0.2, 0.9, 8.0, 0.3)
  assert a_up == GOV_A_UP
  # crawling lead at the anchor: follow at its pace (v_ref = v_lead) and do nothing when already there
  a_f, v_ref, q_ref, _ = governor_demand(0.5, 0.5, 4.3 + 0.0, 0.3)
  assert q_ref == 0.0 and v_ref == 0.5 and a_f == pytest.approx(0.0)
  # continuity across a 0.10 m gap noise step at the anchor: bounded change
  a1, *_ = governor_demand(0.8, 0.0, 4.8, 0.3)
  a2, *_ = governor_demand(0.8, 0.0, 4.7, 0.3)
  assert abs(a1 - a2) < 0.25


def test_governor_reversal_uses_clamped_reference_only():
  # comfort reference clamps v_lead at 0 (the real negative speed belongs to a_kin / the barrier)
  _, v_ref, _, _ = governor_demand(1.0, -0.5, 6.0, 0.3)
  assert v_ref >= 0.0
  assert barrier_demand(1.0, -0.5, 6.0) < barrier_demand(1.0, 0.0, 6.0)  # reversal deepens the barrier


def test_barrier_shape():
  assert barrier_demand(0.0, 0.0, 10.0) == 0.0
  assert barrier_demand(1.0, 0.0, 10.0) == pytest.approx(-1.0 / (2 * (10.0 - 3.1 - 0.45)))
  assert barrier_demand(2.0, 0.0, 3.2) <= -2.0 / 0.1 * 0.5 / 1.0 * 0.1  # deep at the floor (den floor 0.10)
  assert barrier_demand(2.0, 0.0, 3.2) == pytest.approx(-(4.0) / (2 * 0.10))


@pytest.mark.parametrize("bad", [float("nan"), float("inf"), None, "x"])
def test_unusable_inputs_return_none(bad):
  assert governor_demand(bad, 0.0, 6.0, 0.3) is None
  assert governor_demand(1.0, 0.0, bad, 0.3) is None
  assert barrier_demand(1.0, bad, 6.0) is None


def test_telemetry_governor_shadow_is_bounded_and_summarized():
  events = []
  tel = StoppingTelemetry(log_fn=lambda **kw: events.append(kw))
  for k in range(3000):  # 30 s active
    tel.update(phase="APPROACH_GLIDE", active=True, shadow_accel=-0.5, wire_accel=-0.5, v_ego=1.0, d_gap=6.0,
               dts=None, wheel_stop_latched=False, dt=0.01, gov=(-0.9 if k % 2 else -0.1, -0.2))
  tel.update(phase="INACTIVE", active=False, shadow_accel=0.0, wire_accel=0.0, v_ego=0.0, d_gap=None,
             dts=None, wheel_stop_latched=False, dt=0.01)
  s = next(e for e in events if e["kind"] == "settle_summary")
  assert s["gov_frames"] == 3000 and len(s["gov_trace"]) <= MAX_GOV_TRACE_ENTRIES
  assert s["gov_deeper_frac"] == pytest.approx(0.5, abs=0.01) and s["gov_shallower_frac"] == pytest.approx(0.5, abs=0.01)
  assert s["gov_max_div"] == pytest.approx(0.4) and s["gov_min"] == pytest.approx(-0.9)
  # without governor data nothing is counted and the summary still emits
  events.clear()
  tel.update(phase="APPROACH_GLIDE", active=True, shadow_accel=-0.5, wire_accel=-0.5, v_ego=1.0, d_gap=6.0,
             dts=None, wheel_stop_latched=False, dt=0.01)
  tel.update(phase="INACTIVE", active=False, shadow_accel=0.0, wire_accel=0.0, v_ego=0.0, d_gap=None,
             dts=None, wheel_stop_latched=False, dt=0.01)
  assert next(e for e in events if e["kind"] == "settle_summary")["gov_frames"] == 0


def _run_service(monkeypatch, patched):
  from openpilot.selfdrive.controls.lib.stop_context import StopContext
  from openpilot.selfdrive.controls.lib.stopping_service import StoppingService
  if patched:
    monkeypatch.setattr(svc, "governor_demand", lambda *a, **k: (-9.0, 0.0, 0.0, 0.0))
    monkeypatch.setattr(svc, "barrier_demand", lambda *a, **k: -9.0)
  ctx, s = StopContext(), StoppingService()
  out = []
  v, gap = 2.0, 9.0
  for _ in range(600):
    sig = ctx.update(v_ego=v, a_ego=-0.6, a_cmd=-0.6, lead_status=True, lead_v=0.0, lead_d_rel=gap,
                     lead_track_id=7, standstill=v < 0.05, dt=0.01)
    r = s.update(engaged=True, v_ego=v, a_ego=-0.6, a_target=-0.6, should_stop=True, dts_planner=max(gap - 4.3, 0.05),
                 planner_min_limit=-3.5, signals=sig, lead_status=True, lead_v=0.0, increased_stopped_distance=0.3,
                 dt=0.01, wire_accel=-0.6, a_target_trajectory=-0.6)
    out.append((r.accel, r.active, r.debug.get("a_gov")))
    v = max(v - 0.6 * 0.01, 0.0)
    gap = max(gap - v * 0.01, 0.3)
  return out


def test_governor_shadow_never_reaches_the_wire(monkeypatch):
  # structural pin: with the governor patched to demand -9.0 on every frame, the service output is
  # frame-identical to the unpatched run (the value is telemetry only), and the real run reports it
  base = _run_service(monkeypatch, patched=False)
  assert any(x[2] is not None for x in base), "the shadow governor was never evaluated"
  patched = _run_service(monkeypatch, patched=True)
  assert [x[:2] for x in base] == [x[:2] for x in patched]
  assert all(x[2] == -9.0 for x in patched if x[2] is not None)


@pytest.mark.parametrize("mode", ["LIVE", "LIVE_TERMINAL"])
@pytest.mark.parametrize("helper", ["governor_demand", "barrier_demand"])
def test_shadow_helper_fault_never_reaches_the_live_fault_latch(monkeypatch, mode, helper):
  # R1 HIGH: a raise inside a shadow helper must not trip longcontrol's blanket fault latch: the wire is
  # frame-identical, ownership continues, _service_live_disabled stays False
  from openpilot.selfdrive.controls.lib import stopping_flags
  from openpilot.selfdrive.controls.lib.longcontrol import LongControl
  from openpilot.selfdrive.controls.lib.tests.test_longcontrol_fast_release import (
    DummyCarParams, DummyCarState, DummyFrogPilotToggles,
  )
  monkeypatch.setattr(stopping_flags, "SERVICE_MODE", mode)

  def run(raising):
    if raising:
      def boom(*a, **k):
        raise RuntimeError("injected shadow fault")
      monkeypatch.setattr(svc, helper, boom)
    else:
      monkeypatch.undo()
      monkeypatch.setattr(stopping_flags, "SERVICE_MODE", mode)
    cp = DummyCarParams()
    cp.longitudinalTuning.kpV = [0.0]
    lc = LongControl(cp)
    toggles = DummyFrogPilotToggles()
    v, gap = 2.2, 9.0
    wires, owned = [], []
    for _ in range(500):
      w = float(lc.update(active=True, CS=DummyCarState(v_ego=v, a_ego=-0.6, standstill=v < 0.05), a_target=-0.6,
                          should_stop=True, distance_to_stop_target_m=max(gap - 4.3, 0.05), accel_limits=(-3.5, 2.0),
                          frogpilot_toggles=toggles, lead_status=True, lead_v=0.0, lead_d_rel=gap, lead_model_prob=0.9,
                          lead_track_id=5))
      wires.append(w)
      owned.append(bool(lc._service_live_owning))
      v = max(v - 0.6 * 0.01, 0.0)
      gap = max(gap - v * 0.01, 0.3)
    return wires, owned, lc._service_live_disabled

  w0, o0, d0 = run(False)
  w1, o1, d1 = run(True)
  assert any(o0), "the service never owned the wire in the baseline"
  assert w1 == w0 and o1 == o0
  assert d0 is False and d1 is False


def test_telemetry_pre_entry_ring_flushes_into_the_trace_and_is_bounded():
  from openpilot.selfdrive.controls.lib.stopping_telemetry import PRE_ENTRY_RING
  events = []
  tel = StoppingTelemetry(log_fn=lambda **kw: events.append(kw))
  for k in range(600):   # 6 s of pre-band samples at 100 Hz -> ring keeps the last 3 s at 4 Hz
    tel.pre_entry_tick(0.01)
    tel.pre_entry_sample(v_ego=4.0 - k * 0.003, d_gap=20.0 - k * 0.02, wire_accel=-0.5, a_gov=-0.7, a_barrier=-0.1)
  assert len(tel._pre_ring) == PRE_ENTRY_RING
  for _ in range(10):
    tel.update(phase="APPROACH_GLIDE", active=True, shadow_accel=-0.5, wire_accel=-0.5, v_ego=2.2, d_gap=8.0,
               dts=None, wheel_stop_latched=False, dt=0.01, gov=(-0.9, -0.2))
  tel.update(phase="INACTIVE", active=False, shadow_accel=0.0, wire_accel=0.0, v_ego=0.0, d_gap=None,
             dts=None, wheel_stop_latched=False, dt=0.01)
  s = next(e for e in events if e["kind"] == "settle_summary")
  head = [x for x in s["gov_trace"] if x[0] < 0]
  assert len(head) == PRE_ENTRY_RING and -3.0 <= head[0][0] <= -2.5 and -0.5 <= head[-1][0] < 0.0
  assert len(tel._pre_ring) == 0
  # with no settle the ring is discarded, not logged
  events.clear()
  tel.pre_entry_tick(0.3)
  tel.pre_entry_sample(v_ego=3.0, d_gap=15.0, wire_accel=-0.4, a_gov=-0.6, a_barrier=None)
  assert not events


@pytest.mark.parametrize("mode", ["LIVE", "LIVE_TERMINAL"])
def test_pre_band_shadow_samples_before_entry_and_never_touches_the_wire(monkeypatch, mode):
  from openpilot.selfdrive.controls.lib import longcontrol as lc_mod
  from openpilot.selfdrive.controls.lib import stopping_flags
  from openpilot.selfdrive.controls.lib.longcontrol import LongControl
  from openpilot.selfdrive.controls.lib.tests.test_longcontrol_fast_release import (
    DummyCarParams, DummyCarState, DummyFrogPilotToggles,
  )
  monkeypatch.setattr(stopping_flags, "SERVICE_MODE", mode)

  def run(patched):
    monkeypatch.undo()
    monkeypatch.setattr(stopping_flags, "SERVICE_MODE", mode)
    if patched == "raise":
      def boom(*a, **k):
        raise RuntimeError("injected pre-band fault")
      monkeypatch.setattr(lc_mod, "governor_demand", boom)
    elif patched == "deep":
      monkeypatch.setattr(lc_mod, "governor_demand", lambda *a, **k: (-9.0, 0.0, 0.0, 0.0))
    cp = DummyCarParams()
    cp.longitudinalTuning.kpV = [0.0]
    lc = LongControl(cp)
    events = []
    lc._service_shadow_tel = lc_mod.StoppingTelemetry(log_fn=lambda **kw: events.append(kw))
    toggles = DummyFrogPilotToggles()
    v, gap = 4.4, 20.0
    wires = []
    for _ in range(900):
      w = float(lc.update(active=True, CS=DummyCarState(v_ego=v, a_ego=-0.6, standstill=v < 0.05), a_target=-0.6,
                          should_stop=v < 2.5, distance_to_stop_target_m=max(gap - 4.3, 0.05), accel_limits=(-3.5, 2.0),
                          frogpilot_toggles=toggles, lead_status=True, lead_v=0.0, lead_d_rel=gap, lead_model_prob=0.9,
                          lead_track_id=5))
      wires.append(w)
      v = max(v - 0.6 * 0.01, 0.0)
      gap = max(gap - v * 0.01, 0.3)
    # force the settle summary out
    lc.update(active=False, CS=DummyCarState(v_ego=0.0, a_ego=0.0, standstill=True), a_target=0.0, should_stop=False,
              distance_to_stop_target_m=-1.0, accel_limits=(-3.5, 2.0), frogpilot_toggles=toggles)
    return wires, events

  w0, ev0 = run(None)
  summaries = [e for e in ev0 if e["kind"] == "settle_summary"]
  assert summaries, "no settle summary emitted"
  head = [x for x in summaries[0]["gov_trace"] if x[0] < 0]
  assert head, "the pre-band shadow never sampled before entry"
  assert all(x[1] > 2.0 for x in head), "pre-band samples must come from above the service band"
  w1, _ = run("raise")
  w2, _ = run("deep")
  assert w1 == w0 and w2 == w0, "the pre-band shadow changed the wire"


def _settle(tel, n=10, gov=(-0.9, -0.2)):
  for _ in range(n):
    tel.update(phase="APPROACH_GLIDE", active=True, shadow_accel=-0.5, wire_accel=-0.5, v_ego=2.2, d_gap=8.0,
               dts=None, wheel_stop_latched=False, dt=0.01, gov=gov)
  tel.update(phase="INACTIVE", active=False, shadow_accel=0.0, wire_accel=0.0, v_ego=0.0, d_gap=None,
             dts=None, wheel_stop_latched=False, dt=0.01)


def test_pre_entry_ring_is_settle_bounded(monkeypatch):
  # R1 MEDIUM: (1) an ABORTED approach (samples, then 10 s without a settle) must not decorate a later
  # settle; (2) only samples within 3 s before entry attach; (3) samples while a settle is active are
  # ignored and cannot leak into the NEXT settle; (4) completion clears the ring
  events = []
  tel = StoppingTelemetry(log_fn=lambda **kw: events.append(kw))
  for _ in range(200):     # aborted approach: 2 s of samples
    tel.pre_entry_tick(0.01)
    tel.pre_entry_sample(v_ego=4.0, d_gap=15.0, wire_accel=-0.4, a_gov=-0.6, a_barrier=None)
  for _ in range(1000):    # 10 s idle: the sampler is NOT called (disengaged / above band) -- only the clock ticks
    tel.pre_entry_tick(0.01)
  assert len(tel._pre_ring) == 0, "an unfed ring must expire"
  _settle(tel)
  s1 = [e for e in events if e["kind"] == "settle_summary"][-1]
  assert not [x for x in s1["gov_trace"] if x[0] < 0], "stale pre-band samples attached to an unrelated settle"
  # fresh approach: 5 s of samples right before entry -> only the last 3 s attach, times end near -0.25
  for _ in range(500):
    tel.pre_entry_tick(0.01)
    tel.pre_entry_sample(v_ego=3.0, d_gap=12.0, wire_accel=-0.5, a_gov=-0.7, a_barrier=-0.1)
  _settle(tel)
  s2 = [e for e in events if e["kind"] == "settle_summary"][-1]
  head = [x for x in s2["gov_trace"] if x[0] < 0]
  assert head and min(x[0] for x in head) >= -3.0 - 1e-6 and max(x[0] for x in head) >= -0.3
  # samples during an active settle are ignored, and the completed settle leaves an empty ring
  for _ in range(5):
    tel.update(phase="APPROACH_GLIDE", active=True, shadow_accel=-0.5, wire_accel=-0.5, v_ego=2.0, d_gap=7.0,
               dts=None, wheel_stop_latched=False, dt=0.01)
    tel.pre_entry_tick(0.3)
    tel.pre_entry_sample(v_ego=2.0, d_gap=7.0, wire_accel=-0.5, a_gov=-0.8, a_barrier=None)
  assert len(tel._pre_ring) == 0
  tel.update(phase="INACTIVE", active=False, shadow_accel=0.0, wire_accel=0.0, v_ego=0.0, d_gap=None,
             dts=None, wheel_stop_latched=False, dt=0.01)
  assert len(tel._pre_ring) == 0
  _settle(tel)   # back-to-back: the next settle has no pre-band head
  s3 = [e for e in events if e["kind"] == "settle_summary"][-1]
  assert not [x for x in s3["gov_trace"] if x[0] < 0]


def test_pre_band_ring_expires_across_a_no_call_gap_in_longcontrol(monkeypatch):
  # R2: samples at 4 m/s, then 10 s DISENGAGED (no sampler calls, only frames), then an immediate low-speed
  # settle: the old samples must not attach (the ring expired through the per-frame tick / reset)
  from openpilot.selfdrive.controls.lib import longcontrol as lc_mod
  from openpilot.selfdrive.controls.lib import stopping_flags
  from openpilot.selfdrive.controls.lib.longcontrol import LongControl
  from openpilot.selfdrive.controls.lib.tests.test_longcontrol_fast_release import (
    DummyCarParams, DummyCarState, DummyFrogPilotToggles,
  )
  monkeypatch.setattr(stopping_flags, "SERVICE_MODE", "LIVE")
  cp = DummyCarParams()
  cp.longitudinalTuning.kpV = [0.0]
  lc = LongControl(cp)
  events = []
  lc._service_shadow_tel = lc_mod.StoppingTelemetry(log_fn=lambda **kw: events.append(kw))
  toggles = DummyFrogPilotToggles()
  for _ in range(200):     # pre-band samples at 4.0 m/s behind a lead (aborted: the lead then leaves)
    lc.update(active=True, CS=DummyCarState(v_ego=4.0, a_ego=-0.3), a_target=-0.3, should_stop=False,
              distance_to_stop_target_m=-1.0, accel_limits=(-3.5, 2.0), frogpilot_toggles=toggles,
              lead_status=True, lead_v=0.0, lead_d_rel=18.0, lead_model_prob=0.9, lead_track_id=5)
  assert len(lc._service_shadow_tel._pre_ring) > 0
  for _ in range(1000):    # 10 s disengaged: the sampler is never called
    lc.update(active=False, CS=DummyCarState(v_ego=9.0, a_ego=0.0), a_target=0.0, should_stop=False,
              distance_to_stop_target_m=-1.0, accel_limits=(-3.5, 2.0), frogpilot_toggles=toggles)
  assert len(lc._service_shadow_tel._pre_ring) == 0
  v, gap = 2.2, 8.0        # immediate re-engage into a stop
  for _ in range(700):
    lc.update(active=True, CS=DummyCarState(v_ego=v, a_ego=-0.6, standstill=v < 0.05), a_target=-0.6, should_stop=True,
              distance_to_stop_target_m=max(gap - 4.3, 0.05), accel_limits=(-3.5, 2.0), frogpilot_toggles=toggles,
              lead_status=True, lead_v=0.0, lead_d_rel=gap, lead_model_prob=0.9, lead_track_id=5)
    v = max(v - 0.6 * 0.01, 0.0)
    gap = max(gap - v * 0.01, 0.3)
  lc.update(active=False, CS=DummyCarState(v_ego=0.0, a_ego=0.0, standstill=True), a_target=0.0, should_stop=False,
            distance_to_stop_target_m=-1.0, accel_limits=(-3.5, 2.0), frogpilot_toggles=toggles)
  s = [e for e in events if e["kind"] == "settle_summary"]
  assert s, "no settle summary"
  stale = [x for x in s[-1]["gov_trace"] if x[0] < 0 and x[1] is not None and x[1] > 3.0]
  assert not stale, "stale 4 m/s samples attached to the later settle"


class _Sim:
  """Closed-loop service driver: perfect plant (accel executes), lead on its own velocity profile."""

  def __init__(self, v0=2.4, gap0=12.0, lead_v_fn=None, isd=0.3):
    from openpilot.selfdrive.controls.lib.stop_context import StopContext
    from openpilot.selfdrive.controls.lib.stopping_service import StoppingService
    self.ctx, self.svc = StopContext(), StoppingService()
    self.v, self.gap, self.isd = v0, gap0, isd
    self.lead_v_fn = lead_v_fn or (lambda t: 0.0)
    self.t = 0.0
    self.rec = {"v": [], "gap": [], "cmd": [], "phase": [], "active": []}

  def run(self, seconds=30.0, lead_status=True):
    while self.t < seconds:
      lv = self.lead_v_fn(self.t)
      sig = self.ctx.update(v_ego=self.v, a_ego=0.0, a_cmd=self.rec["cmd"][-1] if self.rec["cmd"] else -0.3,
                            lead_status=lead_status, lead_v=lv, lead_d_rel=self.gap if lead_status else None,
                            lead_track_id=7 if lead_status else None, standstill=self.v < 0.02, dt=0.01)
      r = self.svc.update(engaged=True, v_ego=self.v, a_ego=0.0, a_target=None, should_stop=True,
                          dts_planner=max(self.gap - (4.0 + self.isd), 0.05), planner_min_limit=-3.5, signals=sig,
                          lead_status=lead_status, lead_v=lv, increased_stopped_distance=self.isd, dt=0.01,
                          wire_accel=self.rec["cmd"][-1] if self.rec["cmd"] else -0.3)
      cmd = r.accel if r.active else -0.3
      self.rec["cmd"].append(cmd)
      self.rec["phase"].append(r.phase.name if r.active else "OFF")
      self.rec["active"].append(r.active)
      self.v = max(self.v + cmd * 0.01, 0.0)
      self.gap = max(self.gap + (lv - self.v) * 0.01, 0.0)
      self.rec["v"].append(self.v)
      self.rec["gap"].append(self.gap)
      self.t += 0.01
      if self.v <= 0.0 and self.t > 2.0 and abs(lv) < 0.01:
        break
    return self.rec


def _governor_flag(monkeypatch, value):
  from openpilot.selfdrive.controls.lib import stopping_flags
  monkeypatch.setattr(stopping_flags, "SERVICE_APPROACH_LAW", value)


def test_step3_flag_default_is_legacy():
  from openpilot.selfdrive.controls.lib import stopping_flags
  assert stopping_flags.SERVICE_APPROACH_LAW == "legacy"


def test_step3_governor_law_stops_in_the_band_with_one_descent(monkeypatch):
  _governor_flag(monkeypatch, "governor")
  rec = _Sim(v0=2.4, gap0=12.0).run()
  # perfect-plant band: the sim executes the -0.70 descent instantly at capture, so it never travels
  # the law's shaping tail (the real plant covers ~0.5 m through lag and creep); the 4.1-4.6 median
  # gate belongs to the real-plant corpus harness (program gate B), not to this structural sim
  assert rec["v"][-1] <= 0.0 and 4.0 <= rec["gap"][-1] <= 5.45, f"rest {rec['gap'][-1]:.2f}"
  assert "PRE_STOP_EASE" not in rec["phase"], "EASE must never engage under the governor law"
  steps = [abs(a - b) for a, b in zip(rec["cmd"][1:], rec["cmd"][:-1], strict=False) if a is not None]
  assert max(steps) <= 0.081, f"limiter bypassed: step {max(steps):.3f}"   # J_SAFE 8.0 * 0.01 + eps
  assert min(rec["gap"]) >= 3.0


def test_step3_governor_law_rides_a_stopping_crawler_to_the_anchor(monkeypatch):
  # the lead crawls at 0.5 for 2 s then stops (the service's job begins when the lead is stopping;
  # FOLLOWING a continuing crawler is the planner's -- the phase lane is braking-only by contract)
  _governor_flag(monkeypatch, "governor")
  rec = _Sim(v0=1.2, gap0=6.0, lead_v_fn=lambda t: 0.5 if t < 2.0 else 0.0).run(seconds=40.0)
  assert rec["v"][-1] <= 0.0 and 3.9 <= rec["gap"][-1] <= 5.0, f"rest {rec['gap'][-1]:.2f}"
  assert min(rec["gap"]) >= 3.0


def test_step3_barrier_holds_the_floor_on_a_short_aim(monkeypatch):
  _governor_flag(monkeypatch, "governor")
  rec = _Sim(v0=2.0, gap0=5.0).run()
  assert min(rec["gap"]) >= 2.95, f"floor breached: {min(rec['gap']):.2f}"


def test_step3_no_lead_keeps_the_legacy_law(monkeypatch):
  _governor_flag(monkeypatch, "governor")
  base = _Sim(v0=2.0, gap0=9.0).run(lead_status=False)
  monkeypatch.setattr(svc, "governor_demand", lambda *a, **k: (-9.0, 0.0, 0.0, 0.0))
  patched = _Sim(v0=2.0, gap0=9.0).run(lead_status=False)
  assert base["cmd"] == patched["cmd"], "the governor was consulted on a no-lead stop"


def test_step3_legacy_is_byte_identical_with_the_governor_patched(monkeypatch):
  _governor_flag(monkeypatch, "legacy")
  base = _Sim(v0=2.4, gap0=12.0).run()
  monkeypatch.setattr(svc, "governor_demand", lambda *a, **k: (-9.0, 0.0, 0.0, 0.0))
  monkeypatch.setattr(svc, "barrier_demand", lambda *a, **k: -9.0)
  patched = _Sim(v0=2.4, gap0=12.0).run()
  assert base["cmd"] == patched["cmd"]


def test_step3_glide_patch_lanes_never_run_under_the_governor(monkeypatch):
  _governor_flag(monkeypatch, "governor")
  sim = _Sim(v0=2.4, gap0=12.0)
  sim.run()
  assert sim.svc._norm_latched is False and sim.svc._norm_dwell == 0.0, "normalization engaged under the governor"
  assert sim.svc._late_seed_hold is False, "the late-entry corridor engaged under the governor"


def test_step3_norm_lift_never_engages_under_the_governor(monkeypatch):
  # the SAME fixture that engages the cycle-26 normalization under legacy must not engage it here
  from openpilot.selfdrive.controls.lib.tests.test_stopping_service import _deep_entry_svc, _norm_descend
  _governor_flag(monkeypatch, "legacy")
  _, lifted_legacy = _norm_descend(_deep_entry_svc(), 1.05, 0.45, 5.0, 4.1)
  assert lifted_legacy, "the fixture no longer engages under legacy -- the pin is vacuous"
  _governor_flag(monkeypatch, "governor")
  _, lifted_gov = _norm_descend(_deep_entry_svc(), 1.05, 0.45, 5.0, 4.1)
  assert not lifted_gov, "the cycle-26 normalization engaged under the governor law"


def test_step3_late_entry_corridor_never_engages_under_the_governor(monkeypatch):
  # the SAME ff3-s16 fixture that engages the cycle-29 corridor under legacy must not engage it here
  from openpilot.selfdrive.controls.lib.stopping_service import StoppingService
  from openpilot.selfdrive.controls.lib.tests.test_stopping_service import _late_run
  _governor_flag(monkeypatch, "legacy")
  _, held_legacy = _late_run(StoppingService())
  assert held_legacy, "the fixture no longer engages under legacy -- the pin is vacuous"
  _governor_flag(monkeypatch, "governor")
  _, held_gov = _late_run(StoppingService())
  assert not held_gov, "the late-entry corridor engaged under the governor law"


def test_step3_governor_law_keeps_the_glide_laws_grade_feedforward(monkeypatch):
  # a strong creep push (a_coast +0.40) must deepen the governed command like the glide law's
  # deepen-only feedforward does; without the coast term both runs would be equal
  from openpilot.selfdrive.controls.lib.stopping_service import StoppingService
  from openpilot.selfdrive.controls.lib.tests.test_stopping_service import make_signals
  _governor_flag(monkeypatch, "governor")

  def run(coast):
    s = StoppingService()
    r = None
    for _ in range(30):
      sig = make_signals(d_gap=8.0, a_coast=coast, latch=True)
      r = s.update(engaged=True, v_ego=1.5, a_ego=-0.4, a_target=None, should_stop=True, dts_planner=3.7,
                   planner_min_limit=-3.5, signals=sig, lead_status=True, lead_v=0.0, dt=0.02, wire_accel=None)
    return r.accel

  assert run(0.40) <= run(0.0) - 0.25, "the creep/grade feedforward is missing from the governed law"


def test_step3_barrier_binds_past_a_shallow_governor(monkeypatch):
  # a reversing lead at a small gap: the governor (comfort reference clamps lead at 0) is patched
  # shallow; the 3.1 m barrier is the lane that must deepen the wire
  from openpilot.selfdrive.controls.lib.stopping_service import StoppingService
  from openpilot.selfdrive.controls.lib.tests.test_stopping_service import make_signals
  _governor_flag(monkeypatch, "governor")
  monkeypatch.setattr(svc, "governor_demand", lambda *a, **k: (-0.10, 0.0, 0.0, 1.0))
  s = StoppingService()
  r = None
  for _ in range(40):
    sig = make_signals(d_gap=4.2, a_coast=0.0, latch=True)
    r = s.update(engaged=True, v_ego=0.8, a_ego=-0.2, a_target=None, should_stop=True, dts_planner=0.05,
                 planner_min_limit=-3.5, signals=sig, lead_status=True, lead_v=-0.5, dt=0.02, wire_accel=None)
  assert r.accel <= -1.1, f"the barrier did not bind: wire {r.accel:.2f}"


def test_step3_barrier_none_fails_closed(monkeypatch):
  # R1 HIGH: unusable barrier inputs must fall back to planner_min (fail-CLOSED), not remove the lane
  from openpilot.selfdrive.controls.lib.stopping_service import StoppingService
  from openpilot.selfdrive.controls.lib.tests.test_stopping_service import make_signals
  _governor_flag(monkeypatch, "governor")
  monkeypatch.setattr(svc, "barrier_demand", lambda *a, **k: None)
  s = StoppingService()
  r = None
  for _ in range(40):
    sig = make_signals(d_gap=4.2, a_coast=0.0, latch=True)
    r = s.update(engaged=True, v_ego=0.8, a_ego=-0.2, a_target=None, should_stop=True, dts_planner=0.05,
                 planner_min_limit=-3.5, signals=sig, lead_status=True, lead_v=-0.5, dt=0.02, wire_accel=None)
  assert r.accel <= -1.1, f"barrier None released the lane: wire {r.accel:.2f}"


def test_step3_barrier_raise_reaches_the_live_robustness_path(monkeypatch):
  # a raise in the LIVE barrier is the service faulting: longcontrol latches ownership off and the
  # legacy chain keeps the wire -- never shallower on the fault frame (the documented degradation)
  from openpilot.selfdrive.controls.lib import longcontrol as lc_mod
  from openpilot.selfdrive.controls.lib import stopping_flags
  from openpilot.selfdrive.controls.lib.longcontrol import LongControl
  from openpilot.selfdrive.controls.lib.tests.test_longcontrol_fast_release import (
    DummyCarParams, DummyCarState, DummyFrogPilotToggles,
  )
  monkeypatch.setattr(stopping_flags, "SERVICE_MODE", "LIVE")
  monkeypatch.setattr(stopping_flags, "SERVICE_APPROACH_LAW", "governor")

  def boom(*a, **k):
    raise RuntimeError("injected barrier fault")
  cp = DummyCarParams()
  cp.longitudinalTuning.kpV = [0.0]
  lc = LongControl(cp)
  toggles = DummyFrogPilotToggles()
  v, gap = 2.2, 9.0
  wires = []
  armed = False
  for _k in range(600):
    if not armed and lc._service_live_owning:
      monkeypatch.setattr(lc_mod, "barrier_demand", boom)   # fault on the frame after takeover
      monkeypatch.setattr(svc, "barrier_demand", boom)
      armed = True
    w = float(lc.update(active=True, CS=DummyCarState(v_ego=v, a_ego=-0.6, standstill=v < 0.05), a_target=-0.6,
                        should_stop=True, distance_to_stop_target_m=max(gap - 4.3, 0.05), accel_limits=(-3.5, 2.0),
                        frogpilot_toggles=toggles, lead_status=True, lead_v=0.0, lead_d_rel=gap, lead_model_prob=0.9,
                        lead_track_id=5))
    if armed and len(wires) and lc._service_live_disabled:
      # documented degradation: the fault frame keeps the legacy-chain value, which may sit a few
      # thousandths shallower for ONE frame (the caps re-pin at brake-step rate from the next frame)
      assert w <= wires[-1] + 0.02, "the fault frame released the wire beyond the one-frame semantics"
    wires.append(w)
    v = max(v - 0.6 * 0.01, 0.0)
    gap = max(gap - v * 0.01, 0.3)
  assert armed and lc._service_live_disabled, "the fault never reached the robustness path"


def _relief_fixture_run(law, monkeypatch):
  # the CYCLE-17 recorded fixture (00001f62 seg25): creeping 0.63 m/s, gap 4.6, shallow wire -0.39 --
  # the scenario the gentle entry rate exists for
  from openpilot.selfdrive.controls.lib.stopping_service import Phase, StoppingService
  from openpilot.selfdrive.controls.lib.tests.test_stopping_service import make_signals
  _governor_flag(monkeypatch, law)
  s = StoppingService()
  s.phase = Phase.APPROACH_GLIDE
  s._last_cmd = -0.39
  s._d_rest_eff = 4.30
  s._d_rest_calc_gap = 4.90
  sig = make_signals(d_gap=4.60, a_coast=0.45, latch=True)
  gentle = catchup = False
  for _ in range(80):
    s.update(engaged=True, v_ego=0.63, a_ego=-0.05, a_target=-0.25, should_stop=True,
             dts_planner=0.05, planner_min_limit=-3.5, signals=sig,
             lead_status=True, lead_v=0.0, dt=0.01, wire_accel=-0.39)
    gentle = gentle or s._relief_entry_gentle
    catchup = catchup or s._relief_catchup
  return s, gentle, catchup


def test_step3_relief_machinery_is_inert_under_the_governor(monkeypatch):
  # R1 HIGH: the cycle-17 gentling/catch-up shaped the limiter under the governor
  _, legacy_gentle, _ = _relief_fixture_run("legacy", monkeypatch)
  assert legacy_gentle, "the cycle-17 fixture no longer engages under legacy -- vacuous pin"
  s, gov_gentle, gov_catchup = _relief_fixture_run("governor", monkeypatch)
  assert not gov_gentle and not gov_catchup, "relief machinery ran under the governor"
  assert s._relief_gentle_target is None
