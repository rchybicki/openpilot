"""Band-consistent governor profile (stopping_flags.GOVERNOR_BAND_PROFILE, cycle 2026-10-04).

Below V_GUARD the band executes at least the creep guard's A_GUARD (-0.60; 2nd gear creeps at -0.5 or shallower) and FLAT_LANDING
lands at A_FLOOR (-0.50) below the descent capture speed. The TAU-faded profile asked -0.47..0 between 1.3 and 0.5 m/s, so a car
riding it was held at -0.60 by the guard and rested ~0.9 m long; the 4.3-4.5 m rests depended on hot arrivals. The band profile
closes at those levels to the same anchor. Round 2 (eb2 red-team): the band profile owns a stop only when its service entry
needs no catch-up under it (an arrival the stop line or the driver already put on the governor's demand); a hot entry keeps the
TAU law for the whole stop, because the band profile brakes less in mid-band and moved a hot arrival's cost into the landing
(00002234 s3, 00002086 s8). Round 4 (Astra code review): the entry decision compares actuator commands -- the band law's
phase command g - coast_ff (the live governor's grade/creep feed-forward) against the entry command. Law and controller tests on
synthetic inputs, not a vehicle model."""
import math
import random

import pytest

from openpilot.selfdrive.controls.lib import stopping_flags
from openpilot.selfdrive.controls.lib.stop_context import StopSignals
from openpilot.selfdrive.controls.lib.stopping_service import (
  GOV_A_C, GOV_A_MAX, GOV_A_UP, GOV_BAND_ENTRY_TOL, GOV_LAG, GOV_REST_BASE_M, GOV_TAU, ServiceParams, StoppingService, governor_demand,
)

V_LAND = ServiceParams.V_DESCENT_START
A_LAND = -stopping_flags.A_FLOOR


def _tau_law(v, v_lead, gap, isd):
  """The flag-off law, written out (a_ego = 0)."""
  q = max(v - v_lead, 0.0)
  d = max(gap - (GOV_REST_BASE_M + isd) - GOV_LAG * q, 0.0)
  z = math.sqrt((GOV_A_C * GOV_TAU) ** 2 + 2.0 * GOV_A_C * d)
  q_ref = z - GOV_A_C * GOV_TAU
  vl = max(v_lead, 0.0)
  if stopping_flags.GOVERNOR_PROFILE_REFERENCE:
    v_ref, a_ff = q_ref, -GOV_A_C * (q_ref - vl) / max(q_ref + GOV_A_C * GOV_TAU, 1e-6)
  else:
    v_ref, a_ff = vl + q_ref, -GOV_A_C * q_ref / max(q_ref + GOV_A_C * GOV_TAU, 1e-6)
  return min(max(a_ff + (v_ref - v) / GOV_TAU, -GOV_A_MAX), GOV_A_UP), v_ref, q_ref, d


def _band_margin(v):
  """Remaining margin d at which the band profile's reference speed is v."""
  if v <= V_LAND:
    return v * v / (2.0 * A_LAND)
  return V_LAND * V_LAND / (2.0 * A_LAND) + (v * v - V_LAND * V_LAND) / (2.0 * GOV_A_C)


def _on_profile_gap(v, isd):
  return GOV_REST_BASE_M + isd + GOV_LAG * v + _band_margin(v)


def _tau_profile_gap(v, isd):
  return GOV_REST_BASE_M + isd + GOV_LAG * v + GOV_TAU * v + v * v / (2.0 * GOV_A_C)


def _random_args(rng):
  return rng.uniform(0.0, 5.0), rng.uniform(-1.0, 3.0), rng.uniform(0.0, 30.0), rng.uniform(0.0, 1.5)


def _band(*args, **kw):
  return governor_demand(*args, band=True, **kw)


@pytest.mark.parametrize("flag", [False, True])
@pytest.mark.parametrize("profile_ref", [False, True])
def test_default_is_the_tau_law_bit_for_bit(monkeypatch, profile_ref, flag):
  # the law takes the profile as an argument; the flag alone changes nothing in it
  monkeypatch.setattr(stopping_flags, "GOVERNOR_BAND_PROFILE", flag)
  monkeypatch.setattr(stopping_flags, "GOVERNOR_PROFILE_REFERENCE", profile_ref)
  rng = random.Random(1004)
  for _ in range(5000):
    args = _random_args(rng)
    assert governor_demand(*args) == _tau_law(*args) == governor_demand(*args, band=False)


@pytest.mark.parametrize("off_flag", ["FINAL_FLOOR", "FLAT_LANDING"])
def test_profile_follows_the_band_flags(monkeypatch, off_flag):
  # the profile is consistent with the band only while the band executes the guard and the flat landing
  monkeypatch.setattr(stopping_flags, off_flag, False)
  rng = random.Random(7)
  for _ in range(500):
    args = _random_args(rng)
    assert _band(*args) == _tau_law(*args)


def test_the_guard_and_the_profile_share_one_level():
  assert stopping_flags.A_GUARD == pytest.approx(-GOV_A_C)
  assert A_LAND * GOV_TAU < V_LAND <= stopping_flags.V_GUARD


@pytest.mark.parametrize("v", [0.45, 0.55, 0.8, 1.0, 1.3, 1.8, 2.5])
def test_on_profile_the_governor_asks_the_band_level(v):
  isd = 0.3
  gap = _on_profile_gap(v, isd)
  a, v_ref, _, d = _band(v, 0.0, gap, isd)
  assert v_ref == pytest.approx(v) and d == pytest.approx(_band_margin(v))
  assert a == pytest.approx(-GOV_A_C if v >= V_LAND else -A_LAND)
  # a car on the TAU profile is asked for the TAU fade, shallower than the guard's hold below V_GUARD (the conflict removed here)
  a_tau, v_ref_tau, *_ = governor_demand(v, 0.0, _tau_profile_gap(v, isd), isd)
  assert v_ref_tau == pytest.approx(v) and a_tau == pytest.approx(-GOV_A_C * v / (v + GOV_A_C * GOV_TAU))
  if v <= stopping_flags.V_GUARD:
    assert a_tau > stopping_flags.A_GUARD + 0.1


def test_no_position_chase_near_the_anchor():
  isd = 0.3
  for v in (0.1, 0.2, 0.3, 0.39):
    a, v_ref, _, _ = _band(v, 0.0, _on_profile_gap(v, isd), isd)
    assert v_ref == pytest.approx(v) and a == pytest.approx(-v / GOV_TAU)   # linear fade to 0 at the anchor
  for gap in (4.30, 4.35, 4.40):   # inside the fade the law is -v/tau whatever the gap: a 0.1 m radar step cannot move it
    a, *_ = _band(0.8, 0.0, gap + GOV_LAG * 0.8, isd)
    assert a == pytest.approx(-0.8 / GOV_TAU)
  a, v_ref, q_ref, d = _band(0.5, 0.5, 4.3, 0.3)   # the d = 0 fixed point of a crawler at its own pace
  assert d == 0.0 and q_ref == 0.0 and a == pytest.approx(0.0)


def test_anchor_lag_and_forecast_travel_are_unchanged():
  rng = random.Random(55)
  for _ in range(2000):
    args = (rng.uniform(0.0, 3.0), rng.uniform(-1.0, 2.0), rng.uniform(2.0, 25.0), rng.uniform(0.0, 1.0))
    a_ego = rng.uniform(-2.0, 0.5)
    on = _band(*args, a_ego=a_ego)
    off = governor_demand(*args, a_ego=a_ego)
    assert on[3] == off[3]                                       # the same remaining margin d (anchor, lag, forecast)
    assert _band_margin(on[2]) == pytest.approx(on[3], abs=1e-9)  # q_ref is the band profile's speed at that margin


def test_moving_lead_never_shallower_than_the_sum_reference(monkeypatch):
  rng = random.Random(53)
  for _ in range(3000):
    v, vl, gap, isd = rng.uniform(0.0, 3.0), rng.uniform(-1.0, 3.0), rng.uniform(3.0, 20.0), rng.uniform(0.0, 1.0)
    monkeypatch.setattr(stopping_flags, "GOVERNOR_PROFILE_REFERENCE", True)
    a_prof, *_ = _band(v, vl, gap, isd)
    monkeypatch.setattr(stopping_flags, "GOVERNOR_PROFILE_REFERENCE", False)
    a_sum, *_ = _band(v, vl, gap, isd)
    assert a_prof <= a_sum + 1e-12
    if vl <= 0.0:
      assert a_prof == a_sum


def test_band_profile_is_monotone_continuous_and_bounded():
  prev = -1.0
  for gap in (4.5, 5.0, 6.0, 8.0, 12.0, 20.0):
    _, v_ref, _, _ = _band(1.0, 0.0, gap, 0.3)
    assert v_ref > prev
    prev = v_ref
  assert _band(12.0, 0.0, 8.0, 0.3)[0] == -GOV_A_MAX
  assert _band(0.2, 0.9, 8.0, 0.3)[0] == GOV_A_UP
  rng = random.Random(9)
  for _ in range(3000):   # a 0.1 m gap step moves the demand by at most the TAU law's own pinned bound
    v, gap = rng.uniform(0.0, 2.5), rng.uniform(4.3, 15.0)
    a1, *_ = _band(v, 0.0, gap, 0.3)
    a2, *_ = _band(v, 0.0, gap + 0.1, 0.3)
    assert abs(a2 - a1) < 0.25


def _toy_stop(monkeypatch, band_profile, v0, gap0, lag=0.25, cmd0=-GOV_A_C, trace=None):
  """StoppingService on a synthetic first-order actuator with the creep guard emulated as min(cmd, guard level) below V_GUARD
  (longcontrol's FINAL_FLOOR, eased to A_FLOOR across V_GUARD_EASE). Not a vehicle model. Returns (rest gap, min command);
  trace: a list that receives every (v, command)."""
  monkeypatch.setattr(stopping_flags, "SERVICE_APPROACH_LAW", "governor")
  monkeypatch.setattr(stopping_flags, "GOVERNOR_BAND_PROFILE", band_profile)
  s = StoppingService()
  dt, v, gap, a, cmd, cmd_min = 0.01, v0, gap0, cmd0, cmd0, 0.0
  lo, hi = stopping_flags.V_GUARD_EASE
  for k in range(4000):
    sig = StopSignals(gap, "measured", False, False, 0.0, v < 0.02, True, True, 3.0 + k * dt, True)
    r = s.update(engaged=True, v_ego=v, a_ego=a, a_target=-0.3, should_stop=True, dts_planner=max(gap - 4.3, 0.0),
                 planner_min_limit=-3.5, signals=sig, lead_status=True, lead_v=0.0, increased_stopped_distance=0.3,
                 dt=dt, wire_accel=cmd, a_target_trajectory=-0.3)
    cmd = r.accel
    if v < stopping_flags.V_GUARD:
      w = min(max((v - lo) / (hi - lo), 0.0), 1.0)
      cmd = min(cmd, stopping_flags.A_FLOOR + w * (stopping_flags.A_GUARD - stopping_flags.A_FLOOR))
    cmd_min = min(cmd_min, cmd)
    if trace is not None:
      trace.append((v, cmd))
    a += (cmd - a) * dt / (lag + dt)
    v_new = max(v + a * dt, 0.0)
    gap -= (v + v_new) / 2.0 * dt
    v = v_new
    if v == 0.0:
      return gap, cmd_min
  raise AssertionError("no stop")


@pytest.mark.parametrize("v0", [1.0, 1.3, 2.0, 2.4])
def test_on_profile_arrival_rests_nearer_the_anchor_without_a_bite(monkeypatch, v0):
  isd = 0.3
  anchor = GOV_REST_BASE_M + isd
  rest_band, min_band = _toy_stop(monkeypatch, True, v0, _on_profile_gap(v0, isd))
  rest_tau, min_tau = _toy_stop(monkeypatch, False, v0, _tau_profile_gap(v0, isd))
  assert rest_tau >= rest_band + 0.6       # under the guard the TAU profile rests long; the band profile removes most of it
  # what remains is the lag reserve (GOV_LAG at constant speed is not consumed by a car already braking at the band level)
  assert anchor - 0.1 <= rest_band <= anchor + GOV_LAG * v0 + 0.1
  assert min_band >= stopping_flags.A_GUARD - 0.05 and min_tau >= stopping_flags.A_GUARD - 0.05   # no catch-up bite


def _hot_gap(v, isd, margin):
  """a gap `margin` m inside the band profile: the band governor asks deeper than the band level at entry"""
  return _on_profile_gap(v, isd) - margin


def _reassert(monkeypatch, release_frames, v0, gap, isd=0.3, coast=0.0):
  """A stop entered on the band demand at 2.0 m/s (band profile latched), released for release_frames (the lead lurches at
  1.5 m/s and the planner goes), then re-asserted at v0 / gap with a_coast `coast` ('the stop re-asserted itself
  mid-release'). Returns (service, re-assert frame result)."""
  monkeypatch.setattr(stopping_flags, "SERVICE_APPROACH_LAW", "governor")
  monkeypatch.setattr(stopping_flags, "GOVERNOR_BAND_PROFILE", True)

  def frame(k, v, g, lv, stopped, a_tgt, wire, a_coast=0.0):
    sig = StopSignals(g, "measured", False, False, a_coast, False, stopped, stopped, 3.0 + k * 0.01, True)
    return s.update(engaged=True, v_ego=v, a_ego=wire, a_target=a_tgt, should_stop=stopped, dts_planner=max(g - 4.3, 0.0),
                    planner_min_limit=-3.5, signals=sig, lead_status=True, lead_v=lv, increased_stopped_distance=isd, dt=0.01,
                    wire_accel=wire, a_target_trajectory=a_tgt)
  s = StoppingService()
  g = _on_profile_gap(2.0, isd)
  r = frame(0, 2.0, g, 0.0, True, -0.6, _band(2.0, 0.0, g, isd)[0])
  assert s._band
  for k in range(1, release_frames + 1):
    g += 0.015
    r = frame(k, 2.0, g, 1.5, False, 0.5, r.accel)
  assert r.phase.name == "RELEASE"
  r = frame(release_frames + 1, v0, gap, 0.0, True, -0.6, r.accel, coast)
  assert r.phase.name == "APPROACH_GLIDE"
  return s, r


@pytest.mark.parametrize("v0,margin", [(2.0, 1.0), (2.4, 1.5), (1.6, 0.6)])
def test_hot_entry_keeps_the_tau_law_for_the_whole_stop(monkeypatch, v0, margin):
  # eb2 red-team MEDIUM (00002234 s3, 1st-gear hot arrival the stop line did not shape): the band profile would brake less in
  # mid-band and move the cost into the landing (a_stop -0.42 -> -0.60). A hot entry keeps the TAU law: the flag changes nothing.
  isd = 0.3
  gap = _hot_gap(v0, isd, margin)
  assert _band(v0, 0.0, gap, isd)[0] < -GOV_A_C - GOV_BAND_ENTRY_TOL   # hot: the entry needs a catch-up under the band profile
  on, off = [], []
  rest_on, _ = _toy_stop(monkeypatch, True, v0, gap, trace=on)
  rest_off, _ = _toy_stop(monkeypatch, False, v0, gap, trace=off)
  assert on == off and rest_on == rest_off
  # eb3 verifier LOW: the same rule on every re-entry -- a stop that entered on the band demand, released mid-band and
  # re-asserted here hot runs the TAU law (it kept the first entry's band latch before)
  s, r = _reassert(monkeypatch, 30, v0, gap, isd)
  assert not s._band and r.debug["a_gov"] == pytest.approx(governor_demand(v0, 0.0, gap, isd)[0])


def test_entry_on_the_governor_demand_runs_the_band_profile(monkeypatch):
  # an arrival the stop line handed over on the governor's demand (the service's seed = the band law's ask) runs the band profile
  isd = 0.3
  v0 = 2.0
  gap = _on_profile_gap(v0, isd) - 0.4
  a_band = _band(v0, 0.0, gap, isd)[0]
  on, off = [], []
  _toy_stop(monkeypatch, True, v0, gap, cmd0=a_band, trace=on)
  _toy_stop(monkeypatch, False, v0, gap, cmd0=a_band, trace=off)
  assert on != off
  s = StoppingService()
  monkeypatch.setattr(stopping_flags, "GOVERNOR_BAND_PROFILE", True)
  sig = StopSignals(gap, "measured", False, False, 0.0, False, True, True, 3.0, True)
  r = s.update(engaged=True, v_ego=v0, a_ego=a_band, a_target=-0.3, should_stop=True, dts_planner=gap - 4.3, planner_min_limit=-3.5,
               signals=sig, lead_status=True, lead_v=0.0, increased_stopped_distance=isd, dt=0.01, wire_accel=a_band,
               a_target_trajectory=-0.3)
  assert s._band and r.debug["a_gov"] == pytest.approx(a_band)
  s2 = StoppingService()   # the same entry seeded shallower than the band law by more than the tolerance is hot: TAU
  s2.update(engaged=True, v_ego=v0, a_ego=a_band, a_target=-0.3, should_stop=True, dts_planner=gap - 4.3, planner_min_limit=-3.5,
            signals=sig, lead_status=True, lead_v=0.0, increased_stopped_distance=isd, dt=0.01, wire_accel=a_band + GOV_BAND_ENTRY_TOL + 0.05,
            a_target_trajectory=-0.3)
  assert not s2._band
  # a re-assert one frame into the release, still on the band demand, is decided by the same rule: the band profile again
  s3, r3 = _reassert(monkeypatch, 1, v0, _on_profile_gap(v0, isd))
  assert s3._band and r3.debug["a_gov"] == pytest.approx(_band(v0, 0.0, _on_profile_gap(v0, isd), isd)[0])


@pytest.mark.parametrize("coast", [0.0, 0.1, 0.2, 0.4])
def test_entry_decision_compares_actuator_commands(monkeypatch, coast):
  # Astra LFL review P2 (service_probe.py): fresh service, stopped lead, 2.0 m/s at the on-profile gap (the band law asks
  # -0.60), entry command -0.60, a_coast +0.4. The live band governor commands g - coast_ff = -1.00: a 0.40 catch-up, so the
  # stop is hot and keeps the TAU law; round 3 compared the net law (-0.60) and latched the band profile.
  monkeypatch.setattr(stopping_flags, "SERVICE_APPROACH_LAW", "governor")
  monkeypatch.setattr(stopping_flags, "GOVERNOR_BAND_PROFILE", True)
  isd, v0, seed = 0.3, 2.0, -GOV_A_C
  gap = _on_profile_gap(v0, isd)
  assert _band(v0, 0.0, gap, isd)[0] == pytest.approx(seed)

  def entry(wire):
    s = StoppingService()
    sig = StopSignals(gap, "measured", False, False, coast, False, True, True, 3.0, True)
    r = s.update(engaged=True, v_ego=v0, a_ego=wire + coast, a_target=-0.3, should_stop=True, dts_planner=gap - 4.3,
                 planner_min_limit=-3.5, signals=sig, lead_status=True, lead_v=0.0, increased_stopped_distance=isd, dt=0.01,
                 wire_accel=wire, a_target_trajectory=-0.3)
    return s, r
  s, r = entry(seed)
  assert s._band is (coast <= GOV_BAND_ENTRY_TOL)
  if not s._band:
    assert r.debug["a_gov"] == pytest.approx(governor_demand(v0, 0.0, gap, isd)[0])   # the TAU law, as on HEAD
  # an entry command that already carries the coast compensation is on the band governor's command: the band profile
  s, r = entry(seed - coast)
  assert s._band and r.debug["a_gov"] == pytest.approx(seed)


@pytest.mark.parametrize("coast", [0.0, 0.4])
def test_reentry_decision_compares_actuator_commands(monkeypatch, coast):
  # the same rule on a re-assert: one frame into a release, still on the band law's net demand -- with a +0.4 push the band
  # governor would command 0.40 deeper than the releasing command, so the re-entry is hot and runs the TAU law
  isd, v0 = 0.3, 2.0
  gap = _on_profile_gap(v0, isd)
  s, r = _reassert(monkeypatch, 1, v0, gap, isd, coast=coast)
  assert s._band is (coast == 0.0)
  law = _band(v0, 0.0, gap, isd) if coast == 0.0 else governor_demand(v0, 0.0, gap, isd)
  assert r.debug["a_gov"] == pytest.approx(law[0])
