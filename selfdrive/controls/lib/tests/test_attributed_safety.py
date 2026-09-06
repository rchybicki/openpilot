"""Attributed-safety SHADOW step (2026-09-02, program doc): inside governor ownership the shadow computes the
candidate wire with the planner's comfort output removed from the min -- only attributed safety lanes may
deepen (a_kin, a_bar, a_mon, a_pred) -- and logs how often / how deep a_plan binds. Telemetry only.

Pins: the predictive lead-braking law; shadow containment (the wire is byte-identical with the flag off,
on, and with the law raising); fail-closed trust-in (fresh identity, unmeasured gap, dropout -> ineligible);
telemetry counters and the bounded ring; the flag default; and the LIVE asymmetric switch (2026-09-05)."""
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
def _one_frame(monkeypatch, track_id=7, frames_before=120, dropout=False, lead_status_when_idless=False,
               lead2=None, fcw=False, lead_a=0.0, dts=None, switch_to_idless_at=None, model_stop_d=-1.0, a_ego=-0.6):
  from openpilot.selfdrive.controls.lib.stop_context import StopContext
  from openpilot.selfdrive.controls.lib.stopping_service import StoppingService
  _law_flag(monkeypatch, "shadow")
  ctx, s = StopContext(), StoppingService()
  v, gap = 2.0, 9.0
  r = sig = None
  for i in range(frames_before):
    tid = track_id
    if dropout and i == frames_before - 1:
      tid = None
    if switch_to_idless_at is not None and i >= switch_to_idless_at:
      tid = None
    status = tid is not None or (lead_status_when_idless and not dropout)
    sig = ctx.update(v_ego=v, a_ego=a_ego, a_cmd=-0.6, lead_status=status, lead_v=0.0, lead_d_rel=gap if status else None,
                     lead_track_id=tid, standstill=False, dt=0.01)
    r = s.update(engaged=True, v_ego=v, a_ego=a_ego, a_target=-1.2, should_stop=True,
                 dts_planner=max(gap - 4.3, 0.05) if dts is None else dts,
                 planner_min_limit=-3.5, signals=sig, lead_status=status, lead_v=0.0, increased_stopped_distance=0.3,
                 dt=0.01, wire_accel=-0.6, a_target_trajectory=-1.2, lead_a=lead_a, lead2=lead2, fcw=fcw, model_stop_d=model_stop_d)
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


def test_vision_only_lead_is_never_mature(monkeypatch):
  # R1 HIGH: a lead with NO radar identity (radarTrackId -1 -> None) must not inherit trust -- first
  # acquisition AND an identity-less replacement after a real identity both stay ineligible ("identity")
  r, sig = _one_frame(monkeypatch, track_id=None, lead_status_when_idless=True, frames_before=200)
  assert sig.track_age_s == 0.0 and r.debug["attr_eligible"] is False and r.debug["attr_reason"] == "identity"
  r, sig = _one_frame(monkeypatch, track_id=7, lead_status_when_idless=True, frames_before=200, switch_to_idless_at=150)
  assert sig.track_age_s == 0.0 and r.debug["attr_eligible"] is False and r.debug["attr_reason"] == "identity"


def test_fcw_and_braking_lead_are_ineligible_with_reasons(monkeypatch):
  r, _ = _one_frame(monkeypatch, fcw=True)
  assert r.debug["attr_eligible"] is False and r.debug["attr_reason"] == "fcw"
  r, _ = _one_frame(monkeypatch, lead_a=-0.5)
  assert r.debug["attr_eligible"] is False and r.debug["attr_reason"] == "lead_braking"
  r, _ = _one_frame(monkeypatch, lead_a=-0.2)
  assert r.debug["attr_eligible"] is True


def test_lead_two_limiting_explains_the_planner_depth(monkeypatch):
  # a second obstacle at 3.5 m (stopped) while leadOne rests at 9 m: a_other is deep, so the -1.2 planner
  # demand is NOT an unexplained bind (the flip gate must not count it as released depth)
  r, _ = _one_frame(monkeypatch, lead2=(True, 0.0, 3.5))
  assert r.debug["a_other"] is not None and r.debug["a_other"] < -1.2
  assert r.debug["attr_plan_bound"] is False and r.debug["attr_released"] == 0.0
  r, _ = _one_frame(monkeypatch, lead2=(True, 0.0, 40.0))
  assert r.debug["attr_plan_bound"] is True   # a far second lead explains nothing


def test_model_stop_closer_than_the_lead_explains_the_planner_depth(monkeypatch):
  # R2: the lead-derived distanceToStopTarget carries no e2e provenance -- the MODEL stop point
  # (longitudinalPlan.distanceToStopTargetModel) does. 1 m ahead vs the lead rest anchor 4.7 m ahead.
  r, _ = _one_frame(monkeypatch, model_stop_d=1.0)
  assert r.debug["a_other"] is not None and r.debug["a_other"] <= -2.0
  assert r.debug["attr_plan_bound"] is False
  r, _ = _one_frame(monkeypatch, dts=1.0)      # the lead-derived distance alone explains NOTHING now
  assert r.debug["attr_plan_bound"] is True
  r, _ = _one_frame(monkeypatch, model_stop_d=-1.0)   # -1 = no model stop: attributable, eligible
  assert r.debug["attr_eligible"] is True


def test_non_finite_hazard_inputs_fail_closed(monkeypatch):
  # R2 [medium]: NaN lead_a bypassed the braking veto; a status-valid leadTwo with NaN velocity was
  # fabricated as a stopped obstacle; a missing/NaN model-stop provenance must not be eligible
  for kw in (dict(lead_a=float("nan")), dict(lead2=(True, float("nan"), 3.5)), dict(lead2=(True, 0.0, float("inf"))),
             dict(model_stop_d=float("nan")), dict(model_stop_d=None), dict(a_ego=float("nan"))):
    r, _ = _one_frame(monkeypatch, **kw)
    assert r.debug["attr_eligible"] is False and r.debug["attr_reason"] == "unusable", kw
    assert r.debug["attr_plan_bound"] is False


def test_predictive_lane_uses_the_measured_ego_decel_not_the_shadow_prior(monkeypatch):
  # R2 [high]: the response-interval acceleration is what the car is doing NOW (a_ego), never the
  # service's own prior command (which is not the wire in observer frames)
  harder, _ = _one_frame(monkeypatch, a_ego=-1.5)
  softer, _ = _one_frame(monkeypatch, a_ego=0.0)
  assert harder.debug["a_pred"] > softer.debug["a_pred"]


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
  assert s["attr_reasons"] == {"?": 1}
  assert len(s["attr_ring"]) == ATTR_RING_MAX and len(s["attr_ring"][0]) == 10


# -- flag pins -------------------------------------------------------------------------------------
def test_flag_default_is_shadow():
  assert stopping_flags.ATTRIBUTED_SAFETY in ("shadow", "live")


# -- LIVE (2026-09-05): asymmetric switch ------------------------------------------------------------
def _frames(monkeypatch, flag, n, lead_a_fn=None, a_target=-1.2, v=2.0, gap=9.0, lead_v=0.0, v_fn=None, gap_fn=None):
  from openpilot.selfdrive.controls.lib.stop_context import StopContext
  from openpilot.selfdrive.controls.lib.stopping_service import StoppingService, ATTR_LIVE_DWELL_S, ATTR_LIVE_RELEASE_J
  _law_flag(monkeypatch, flag)
  ctx, s = StopContext(), StoppingService()
  wires, dbg = [], []
  for i in range(n):
    vi = v_fn(i) if v_fn else v
    gi = gap_fn(i) if gap_fn else gap
    sig = ctx.update(v_ego=vi, a_ego=-0.6, a_cmd=-0.6, lead_status=True, lead_v=lead_v, lead_d_rel=gi, lead_track_id=7, standstill=False, dt=0.01)
    r = s.update(engaged=True, v_ego=vi, a_ego=-0.6, a_target=a_target, should_stop=True, dts_planner=max(gi - 4.3, 0.05),
                 planner_min_limit=-3.5, signals=sig, lead_status=True, lead_v=lead_v, increased_stopped_distance=0.3, dt=0.01,
                 wire_accel=-0.6, a_target_trajectory=a_target, lead_a=(lead_a_fn(i) if lead_a_fn else 0.0), lead2=None, fcw=False,
                 model_stop_d=-1.0)
    wires.append(r.accel)
    dbg.append(r.debug)
  return wires, dbg, ATTR_LIVE_DWELL_S, ATTR_LIVE_RELEASE_J


def test_live_releases_the_planner_excess_only_after_the_dwell_and_at_the_release_ceiling(monkeypatch):
  shadow, _, dwell, jmax = _frames(monkeypatch, "shadow", 400)
  live, dbg, _, _ = _frames(monkeypatch, "live", 400)
  first_live = next(i for i, d in enumerate(dbg) if d.get("attr_live"))
  # identity matures at 0.5 s, then the 0.30 s dwell: nothing changes before that
  assert first_live >= int((0.5 + dwell) * 100) - 4 and live[:first_live] == shadow[:first_live]
  # from then on the wire rises above the a_plan-bound shadow wire, never faster than the release ceiling
  assert live[-1] > shadow[-1] + 0.10
  assert max(live[k + 1] - live[k] for k in range(first_live, 399)) <= jmax * 0.01 + 1e-9
  assert dbg[-1]["attr_candidate"] >= live[-1] - 1e-6            # never above the attributed candidate


def test_live_readmits_a_plan_at_once_on_an_ineligible_frame_and_restarts_the_dwell(monkeypatch):
  def lead_a(i):
    return -0.5 if i == 300 else 0.0                                   # one braking-lead frame mid-release
  live, dbg, dwell, _ = _frames(monkeypatch, "live", 420, lead_a_fn=lead_a)
  assert dbg[299]["attr_live"] and not dbg[300]["attr_live"] and dbg[300]["attr_reason"] == "lead_braking"
  assert live[301] < live[299] - 0.02                                 # deeper at once (a_plan re-admitted, J_SAFE)
  assert not any(d.get("attr_live") for d in dbg[301:301 + int(dwell * 100) - 1])   # dwell restarted
  assert dbg[300]["attr_reentry"] is True and dbg[300]["attr_flip"] is True


def test_live_never_changes_ineligible_or_off_frames(monkeypatch):
  shadow, _, _, _ = _frames(monkeypatch, "shadow", 200)
  live, dbg, _, _ = _frames(monkeypatch, "live", 200)               # identity < 0.5 s for the first 50 frames, then dwell
  assert live[:70] == shadow[:70]
  off, _, _, _ = _frames(monkeypatch, "off", 200)
  assert off == shadow


def test_live_exception_selects_the_current_target(monkeypatch):
  monkeypatch.setattr(svc, "predictive_lead_demand", lambda *a, **k: (_ for _ in ()).throw(RuntimeError("boom")))
  shadow_ref = _frames(monkeypatch, "shadow", 300)[0]
  live = _frames(monkeypatch, "live", 300)[0]
  assert live == shadow_ref


def test_live_is_release_only_it_never_deepens_below_the_current_target(monkeypatch):
  # R1 HIGH: equal-speed crawler at 6 m, planner not binding (a_target 0): the predictive lane is deeper than
  # today's target, but LIVE must not add braking -- wire identical to shadow
  shadow, _, _, _ = _frames(monkeypatch, "shadow", 300, a_target=0.0, v=2.0, gap=6.0, lead_v=2.0)
  live, dbg, _, _ = _frames(monkeypatch, "live", 300, a_target=0.0, v=2.0, gap=6.0, lead_v=2.0)
  assert live == shadow
  assert any(d.get("attr_live") for d in dbg[100:]) and all(d.get("attr_live_release", 0.0) == 0.0 for d in dbg)


def test_live_dwell_restarts_after_an_early_fallback_frame(monkeypatch):
  # R1 MEDIUM: a non-finite speed frame returns through _fallback() before the block; the completed dwell
  # must not survive it -- the next eligible frames need a fresh 0.30 s
  import math
  def v_fn(i):
    return math.nan if i == 200 else 2.0
  live, dbg, dwell, _ = _frames(monkeypatch, "live", 260, v_fn=v_fn)
  assert dbg[199].get("attr_live")
  assert not any(d.get("attr_live") for d in dbg[201:201 + int(dwell * 100) - 2])


# -- cycle 49 (2026-09-05): gap trust is the service's gap_live predicate ----------------------------------
def test_outward_hold_frames_stay_eligible_and_keep_the_live_release(monkeypatch):
  # the radar's 20 Hz reading sits a few centimetres OUTSIDE the ego-propagated prediction: the gap filter
  # answers with an OUTWARD persistence hold (min(prediction, raw), a lower bound). Before cycle 49 every
  # such frame was "gap"-ineligible and restarted the dwell; on-road that blocked all releases.
  def gap_fn(i):
    return 9.05 if 300 <= i < 306 else 9.0
  shadow, _, _, _ = _frames(monkeypatch, "shadow", 340, gap_fn=gap_fn)
  live, dbg, _, jmax = _frames(monkeypatch, "live", 340, gap_fn=gap_fn)
  from openpilot.selfdrive.controls.lib.stop_context import StopContext
  ctx = StopContext()
  for i in range(306):
    sig = ctx.update(v_ego=2.0, a_ego=-0.6, a_cmd=-0.6, lead_status=True, lead_v=0.0, lead_d_rel=gap_fn(i), lead_track_id=7,
                     standstill=False, dt=0.01)
  assert sig.gap_source == "held" and sig.gap_hold_outward   # the frame class under test really is an outward hold
  assert dbg[299]["attr_live"]
  for k in range(300, 306):
    assert dbg[k]["attr_eligible"] is True and dbg[k]["attr_reason"] is None and dbg[k]["attr_live"]
    assert dbg[k]["attr_reentry"] is False and dbg[k]["attr_flip"] is False
  # the release survives the hold: the live wire never drops to the a_plan-bound shadow wire (no re-admission),
  # and the held gap only propagates inward at v*dt (a gentle, bounded deepening -- not the J_SAFE snap of a
  # re-admitted a_plan); rises stay under the release ceiling
  assert all(live[k] >= shadow[k] - 1e-9 for k in range(300, 340))
  assert min(live[k] - shadow[k] for k in range(300, 306)) > 0.10
  assert min(live[k + 1] - live[k] for k in range(299, 306)) >= -0.012
  assert max(live[k + 1] - live[k] for k in range(299, 339)) <= jmax * 0.01 + 1e-9


def test_inward_rejection_hold_frame_is_ineligible(monkeypatch):
  # an inward step larger than the ego-closing slack is REJECTED by the filter: it emits the (larger)
  # prediction, an optimistic gap -- must stay "gap"-ineligible, re-admit a_plan and restart the dwell
  def gap_fn(i):
    return 8.5 if i == 300 else 9.0
  live, dbg, dwell, _ = _frames(monkeypatch, "live", 340, gap_fn=gap_fn)
  assert dbg[299]["attr_live"]
  assert dbg[300]["attr_eligible"] is False and dbg[300]["attr_reason"] == "gap" and not dbg[300]["attr_live"]
  assert dbg[300]["attr_reentry"] is True
  assert live[301] < live[299] - 0.02
  assert not any(d.get("attr_live") for d in dbg[301:301 + int(dwell * 100) - 2])

