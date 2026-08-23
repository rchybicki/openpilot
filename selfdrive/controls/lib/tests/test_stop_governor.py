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
