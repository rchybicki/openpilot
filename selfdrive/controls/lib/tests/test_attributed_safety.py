"""Attributed-safety SHADOW step (2026-09-02, program doc): inside governor ownership the shadow computes the
candidate wire with the planner's comfort output removed from the min -- only attributed safety lanes may
deepen (a_kin, a_bar, a_mon, a_pred) -- and logs how often / how deep a_plan binds. Telemetry only.

Pins: the predictive lead-braking law; shadow containment (the wire is byte-identical with the flag off,
on, and with the law raising); fail-closed trust-in (fresh identity, unmeasured gap, dropout -> ineligible);
telemetry counters and the bounded ring; the flag default; and that "live" is not a recognised mode."""
import math

import pytest

from openpilot.selfdrive.controls.lib import stopping_flags
from openpilot.selfdrive.controls.lib import stopping_service as svc
from openpilot.selfdrive.controls.lib.stopping_service import predictive_lead_demand, ATTR_TRUST_IN_S
from openpilot.selfdrive.controls.lib.stopping_telemetry import StoppingTelemetry, ATTR_RING_MAX, ATTR_RING_STRIDE
from openpilot.selfdrive.controls.lib.tests.test_stop_governor import _Sim


def _law_flag(monkeypatch, value):
  monkeypatch.setattr(stopping_flags, "ATTRIBUTED_SAFETY", value)


# -- the predictive law ---------------------------------------------------------------------------
def test_pred_stopped_lead_is_the_lag_aware_kinematic_stop():
  # v=2, gap=8, stopped lead, no current braking: after rho=0.45 s the ego (still at 2 m/s) has
  # 8 - 2.0 - 0.9 = 5.1 m to stop -> -4/(2*5.1)
  a = predictive_lead_demand(2.0, 0.0, 8.0, 0.0, 2.0, rho=0.45, b_lead_max=2.5)
  assert a == pytest.approx(-4.0 / (2.0 * 5.1), abs=1e-6)


def test_pred_equal_speed_lead_binds_where_instantaneous_lanes_are_zero():
  # ego 2 m/s behind a lead at 2 m/s, 6 m: a_kin and the barrier are ZERO (no closing) -- the design
  # red-team's blocker. The predictive lane assumes the lead may brake at B_LEAD_MAX now.
  a = predictive_lead_demand(2.0, 2.0, 6.0, 0.0, 2.0, rho=0.45, b_lead_max=2.5)
  s_lead = 4.0 / (2.0 * 2.5)
  assert a == pytest.approx(-4.0 / (2.0 * (6.0 - 2.0 - 0.9 + s_lead)), abs=1e-6)
  assert a < -0.4


def test_pred_current_braking_reduces_the_demand_and_far_gap_vanishes():
  braking = predictive_lead_demand(2.0, 0.0, 8.0, -1.0, 2.0)
  coasting = predictive_lead_demand(2.0, 0.0, 8.0, 0.0, 2.0)
  assert braking > coasting
  assert predictive_lead_demand(2.0, 0.0, 60.0, 0.0, 2.0) > -0.05


def test_pred_unusable_inputs_return_none_and_depth_is_bounded():
  assert predictive_lead_demand(float("nan"), 0.0, 8.0, 0.0, 2.0) is None
  assert predictive_lead_demand(2.0, 0.0, 0.0, 0.0, 2.0) is None
  assert predictive_lead_demand(2.0, 0.0, -1.0, 0.0, 2.0) is None
  assert predictive_lead_demand(3.0, 0.0, 2.05, 0.0, 2.0) == -5.0   # clipped, finite


# -- shadow containment: the wire never depends on the shadow -------------------------------------
def _wire(monkeypatch, flag, raise_in_law=False):
  _law_flag(monkeypatch, flag)
  if raise_in_law:
    monkeypatch.setattr(svc, "predictive_lead_demand", lambda *a, **k: (_ for _ in ()).throw(RuntimeError("boom")))
  rec = _Sim(v0=2.4, gap0=12.0).run(seconds=25.0)
  return rec["cmd"], rec["gap"][-1]


def test_shadow_wire_is_byte_identical_with_the_flag_off_on_or_raising(monkeypatch):
  off, gap_off = _wire(monkeypatch, "off")
  on, gap_on = _wire(monkeypatch, "shadow")
  boom, gap_boom = _wire(monkeypatch, "shadow", raise_in_law=True)
  assert on == off and boom == off
  assert gap_on == gap_off == gap_boom
  assert 4.0 <= gap_off <= 5.5   # the governed stop still rests in band


def test_shadow_fields_appear_only_in_shadow_mode(monkeypatch):
  for flag, expect in (("off", False), ("shadow", True)):
    _law_flag(monkeypatch, flag)
    sim = _Sim(v0=2.4, gap0=12.0)
    sim.run(seconds=3.0)   # inside the band, identity mature; then inspect one more frame directly
    r = sim.svc.update(engaged=True, v_ego=sim.v, a_ego=0.0, a_target=-0.8, should_stop=True,
                       dts_planner=max(sim.gap - 4.3, 0.05), planner_min_limit=-3.5,
                       signals=sim.ctx.update(v_ego=sim.v, a_ego=0.0, a_cmd=-0.5, lead_status=True, lead_v=0.0,
                                              lead_d_rel=sim.gap, lead_track_id=7, standstill=False, dt=0.01),
                       lead_status=True, lead_v=0.0, increased_stopped_distance=0.3, dt=0.01, wire_accel=-0.5,
                       a_target_trajectory=-0.8)
    assert ("attr_eligible" in r.debug) == expect


# -- fail-closed trust-in ---------------------------------------------------------------------------
def _one_frame(monkeypatch, track_id=7, frames_before=120, dropout=False):
  from openpilot.selfdrive.controls.lib.stop_context import StopContext
  from openpilot.selfdrive.controls.lib.stopping_service import StoppingService
  _law_flag(monkeypatch, "shadow")
  ctx, s = StopContext(), StoppingService()
  v, gap = 2.0, 9.0
  r = None
  for i in range(frames_before):
    tid = track_id if i < frames_before - 1 or not dropout else None
    status = tid is not None
    sig = ctx.update(v_ego=v, a_ego=-0.6, a_cmd=-0.6, lead_status=status, lead_v=0.0, lead_d_rel=gap if status else None,
                     lead_track_id=tid, standstill=False, dt=0.01)
    r = s.update(engaged=True, v_ego=v, a_ego=-0.6, a_target=-1.2, should_stop=True, dts_planner=max(gap - 4.3, 0.05),
                 planner_min_limit=-3.5, signals=sig, lead_status=status, lead_v=0.0, increased_stopped_distance=0.3,
                 dt=0.01, wire_accel=-0.6, a_target_trajectory=-1.2)
  return r, sig


def test_mature_measured_lead_is_eligible_and_counts_a_plan_binding(monkeypatch):
  r, sig = _one_frame(monkeypatch)
  assert sig.track_age_s >= ATTR_TRUST_IN_S and sig.gap_source == "measured"
  assert r.debug["attr_eligible"] is True
  # a_plan = -1.2 (planner) is deeper than the governed phase and every attributed lane at 9 m / 2 m/s
  assert r.debug["attr_plan_bound"] is True and r.debug["attr_released"] > 0.10 and r.debug["attr_unexplained"] is True
  assert r.debug["attr_candidate"] > r.debug["a_plan"]


def test_fresh_identity_is_ineligible_until_trust_in(monkeypatch):
  r, sig = _one_frame(monkeypatch, frames_before=int(ATTR_TRUST_IN_S * 100) - 5)
  assert sig.track_age_s < ATTR_TRUST_IN_S
  assert r.debug["attr_eligible"] is False and r.debug["attr_plan_bound"] is False


def test_dropout_frame_is_ineligible(monkeypatch):
  r, sig = _one_frame(monkeypatch, dropout=True)
  assert sig.gap_source != "measured" or sig.dropout_active
  assert r.debug.get("attr_eligible", False) is False


def test_track_age_restarts_on_identity_replacement():
  from openpilot.selfdrive.controls.lib.stop_context import StopContext
  ctx = StopContext()
  for _ in range(80):
    sig = ctx.update(v_ego=2.0, a_ego=0.0, a_cmd=-0.3, lead_status=True, lead_v=0.0, lead_d_rel=9.0, lead_track_id=7, dt=0.01)
  assert sig.track_age_s == pytest.approx(0.8, abs=0.02)
  sig = ctx.update(v_ego=2.0, a_ego=0.0, a_cmd=-0.3, lead_status=True, lead_v=0.0, lead_d_rel=8.5, lead_track_id=9, dt=0.01)
  assert sig.track_age_s < 0.05


# -- telemetry ------------------------------------------------------------------------------------
def test_telemetry_counts_and_bounds_the_ring():
  tel = StoppingTelemetry()
  logs = []
  tel._log = lambda **kw: logs.append(kw)
  bound = {"attr_eligible": True, "attr_plan_bound": True, "attr_unexplained": True, "attr_released": 0.3,
           "attr_pred_bound": False, "attr_candidate": -0.5, "a_pred": -0.2, "a_plan": -0.8, "a_phase": -0.5,
           "a_kin": -0.1, "a_barrier": -0.1}
  n = ATTR_RING_MAX * ATTR_RING_STRIDE + 40
  for _ in range(n):
    tel.update(phase="APPROACH_GLIDE", active=True, shadow_accel=-0.5, wire_accel=-0.8, v_ego=1.5, d_gap=7.0, dts=3.0,
               wheel_stop_latched=False, dt=0.01, attr=bound)
  tel.update(phase="APPROACH_GLIDE", active=True, shadow_accel=-0.5, wire_accel=-0.8, v_ego=1.5, d_gap=7.0, dts=3.0,
             wheel_stop_latched=False, dt=0.01, attr={"attr_eligible": False})
  tel.update(phase="INACTIVE", active=False, shadow_accel=0.0, wire_accel=0.0, v_ego=0.0, d_gap=None, dts=None,
             wheel_stop_latched=False, dt=0.01)
  s = next(l for l in logs if l["kind"] == "settle_summary")
  assert s["attr_frames"] == n + 1 and s["attr_plan_bound"] == n and s["attr_unexplained"] == n
  assert s["attr_ineligible"] == 1 and s["attr_released_sum"] == pytest.approx(0.3 * n, abs=1e-6)
  assert len(s["attr_ring"]) == ATTR_RING_MAX and len(s["attr_ring"][0]) == 10


# -- flag pins -------------------------------------------------------------------------------------
def test_flag_default_is_shadow_and_live_is_not_a_mode():
  assert stopping_flags.ATTRIBUTED_SAFETY == "shadow"
  assert "live" not in ("off", "shadow")   # the service recognises only these two; "live" stays unimplemented
