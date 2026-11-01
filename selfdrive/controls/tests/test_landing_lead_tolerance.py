"""LANDING_LEAD_TOLERANCE (cycle 2026-10-16 WS CAPTURE): the DELIVERED_LANDING fail-safe also counts a lead as stopped while the entry latch
holds, the identity is aged and the raw vLead sits inside the radar's braking-bias band [-(K |aEgo|max0.3s + C), +C]; every other fail-safe
line stays. Hot arrivals (22be_1088.74, 22c1_434.92): the strict latch re-earns its dwell 0.03-0.3 s after the capture and the landing
latched today."""
import math
from dataclasses import replace

import pytest

from openpilot.selfdrive.controls.lib import stopping_flags
from openpilot.selfdrive.controls.lib.stopping_service import StoppingService
from openpilot.selfdrive.controls.lib.tests.test_stopping_service import make_signals
from openpilot.selfdrive.controls.tests.test_delivered_landing import APPROACH, DT, HAZARDS, run

# the hot-arrival capture: strict latch still re-earning, entry latch held, the radar reads the stopped lead -0.12 under braking
HOT = dict(cmd0=-.7, latch=False, entry=True, lead_v=-.12)


@pytest.fixture(autouse=True)
def flags(monkeypatch):
  monkeypatch.setattr(stopping_flags, 'FLAT_LANDING', True)
  monkeypatch.setattr(stopping_flags, 'DELIVERED_LANDING', True)
  monkeypatch.setattr(stopping_flags, 'LANDING_LEAD_TOLERANCE', True)


@pytest.mark.parametrize('on', [False, True])
def test_hot_capture_eases_only_with_the_flag(monkeypatch, on):
  monkeypatch.setattr(stopping_flags, 'LANDING_LEAD_TOLERANCE', on)
  svc, rows = run(**HOT)
  moving = [r for r in rows if r['phase'] in APPROACH]
  if on:
    assert not any(r['land_failsafe'] for r in rows) and max(r['cmd'] for r in moving) > -.45
    landing = [r['a'] for r in moving if 0. < r['v'] <= .15]
    assert sum(landing) / len(landing) > -.5
  else:
    assert rows[0]['land_failsafe'] and all(r['land'] is None for r in rows)      # today: latched on the capture frame, FLAT -0.70
    assert not svc._land_aegos                                                     # flag off: no new state


def test_strict_latch_alone_is_unchanged():
  # strict latch confirmed: the tolerance is never consulted (a reading outside the band changes nothing)
  _, rows = run(cmd0=-.7, lead_v=-.4)
  assert not any(r['land_failsafe'] for r in rows)


def capture(a_pre, a_cap, lead_v, *, pre_s=.25, age=10., entry=True):
  """Frames at 0.56..0.51 m/s with aEgo a_pre for pre_s, then the capture frame (0.50 m/s) with aEgo a_cap and the raw vLead lead_v;
  strict latch False. Returns the service after the capture frame."""
  svc = StoppingService()
  svc._last_cmd, svc._floor_v_peak = -.7, .6
  n = int(round(pre_s / DT))
  for i in range(n + 1):
    last = i == n
    v = .50 if last else .56 - .05 * i / max(n, 1)
    sig = replace(make_signals(d_gap=5., latch=False, entry_latch=entry), track_age_s=age)
    svc.update(engaged=True, v_ego=v, a_ego=a_cap if last else a_pre, a_target=None, should_stop=True, dts_planner=10., planner_min_limit=-3.5,
               signals=sig, lead_status=True, lead_v=lead_v if last else -.1, wire_accel=svc._last_cmd, dt=DT, prev_wire=-.7)
  assert svc._creep_floor_armed
  return svc


# the census p1 of the stopped-lead vLead in each aEgo band (data/an2.py, 23,031 frames) at the band's deep edge; the deepest band (< -1.2,
# p1 -0.46) does not reach the landing (landing frames: p1 -0.17)
@pytest.mark.parametrize('a, lead_v, ok', [(-.4, -.10, True), (-.8, -.15, True), (-1.2, -.27, True),
                                           (-.6, -.20, False), (-.6, .10, True), (-.6, .12, False)])
def test_band_at_the_census_p1(a, lead_v, ok):
  svc = capture(a, a, lead_v)
  assert svc._land_failsafe is not ok


def test_band_uses_the_deepest_decel_of_the_bias_lag():
  # the decel shallows -1.2 -> -0.3 at the capture: the radar still carries the bias of the deep frames 0.15 s back
  assert not capture(-1.2, -.3, -.25)._land_failsafe
  assert capture(-1.2, -.3, -.25, pre_s=0.)._land_failsafe          # the instantaneous bound (0.145) would reject it
  assert capture(-.3, -.3, -.25)._land_failsafe


@pytest.mark.parametrize('case', ['young_identity', 'entry_latch_off', 'nan_vlead'])
def test_unproven_lead_stays_latched(case):
  kw = dict(young_identity=dict(age=.3), entry_latch_off=dict(entry=False), nan_vlead={})[case]
  svc = capture(-.6, -.6, math.nan if case == 'nan_vlead' else -.1, **kw)
  assert svc._land_failsafe


def _departing(i, st):
  st.update(lead_v=.34, entry=False)


def _reversing(i, st):
  st.update(lead_v=-.4)


TOL_HAZARDS = {**HAZARDS, 'unconfirmed': lambda i, st: st.update(entry=False), 'dropout': lambda i, st: st.update(dropout=True),
               'departing': _departing, 'creeping_forward': lambda i, st: st.update(lead_v=.15), 'reversing': _reversing,
               'nan_vlead': lambda i, st: st.update(lead_v=math.nan)}


@pytest.mark.parametrize('hazard', list(TOL_HAZARDS))
def test_tolerated_landing_hazard_falls_back_to_today(hazard):
  # every fail-safe line stays live through a tolerance-held ease (22a6_1079.82: the identity blip + thin gap stay latched)
  _, rows = run(gap=5.0, hazard=TOL_HAZARDS[hazard], hazard_v=.38, **{**HOT, 'cmd0': -.5})
  k = next(i for i, r in enumerate(rows) if r['v'] <= .38)
  assert any(r['land'] is not None for r in rows[:k]) and not any(r['land_failsafe'] for r in rows[:k])
  assert rows[k]['land_failsafe'] or rows[k + 1]['land_failsafe']
  ups = [r['land_up'] for r in rows[k:]]
  assert all(b <= a + 1e-12 for a, b in zip(ups, ups[1:], strict=False))
  gone = next(i for i, u in enumerate(ups) if u == 0.)
  assert gone * DT <= rows[k]['land_up'] / StoppingService().p.J_TERMINAL_DESCENT + .02
