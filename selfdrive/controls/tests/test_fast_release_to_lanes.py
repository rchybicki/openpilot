"""FAST_RELEASE_TO_LANES (cycle 2026-10-18 WS RELEASE A): a deep inherited wire that the release limiter lags behind (22d4_4492.94,
22a6_1079.82: the service enters at 1.1-1.3 m/s with the wire near -2.0) rises at J_UP_FAST toward the target (the min of the law and every
lane) behind a lead the ENTRY latch confirmed stopped, below V_REL. Only the upward rate changes; the lanes still bind at their own rates."""
from collections import deque
from dataclasses import replace

import pytest

from openpilot.selfdrive.controls.lib import stopping_flags
from openpilot.selfdrive.controls.lib.stopping_service import Phase, StoppingService
from openpilot.selfdrive.controls.lib.tests.test_stopping_service import make_signals

DT = .01
LANES = ('a_phase', 'a_kin', 'a_plan', 'a_monitor', 'a_barrier')


def run(*, on, v0=1.3, cmd0=-2.0, gap=4.9, frames=300, hazard=None, **state):
  """Closed loop from a deep inherited wire: brake delay 0.15 s, aEgo = the delivered decel 0.06 shallower. hazard(i, state) may change the
  inputs per frame. Returns per-frame rows (debug + cmd, v, the lane min) until rest."""
  stopping_flags.FAST_RELEASE_TO_LANES = on
  svc = StoppingService()
  v, wire = v0, deque([cmd0] * 15, maxlen=15)
  st = dict(gap=gap, age=10., entry=True, lead=True, lead_v=-.1, dropout=False, a_target=None)
  st.update(state)
  rows = []
  for i in range(frames):
    if hazard is not None:
      hazard(i, st)
    sig = replace(make_signals(d_gap=st['gap'], latch=False, entry_latch=st['entry'], dropout=st['dropout'],
                               gap_source='decay' if st['dropout'] else None), track_age_s=st['age'])
    res = svc.update(engaged=True, v_ego=v, a_ego=wire[0] + .06, a_target=st['a_target'], should_stop=True, dts_planner=None,
                     planner_min_limit=-3.5, signals=sig, lead_status=st['lead'], lead_v=st['lead_v'], wire_accel=wire[-1], dt=DT,
                     a_target_trajectory=st['a_target'], prev_wire=wire[-1])
    d = res.debug
    lanes = [d[k] for k in LANES if isinstance(d.get(k), float)]
    rows.append({**d, 'v': v, 'cmd': res.accel, 'lane_min': min(lanes) if lanes else None, 'phase': res.phase})
    wire.append(res.accel)
    v = max(v + wire[0] * DT, 0.)
    st['gap'] -= v * DT
    st['age'] += DT if st['age'] > 0. else 0.
    if v == 0.:
      break
  return svc, rows


@pytest.fixture(autouse=True)
def restore(monkeypatch):
  monkeypatch.setattr(stopping_flags, 'FAST_RELEASE_TO_LANES', stopping_flags.FAST_RELEASE_TO_LANES)


def steps(rows):
  approach = [r for r in rows if r['phase'] == Phase.APPROACH_GLIDE]
  return [b['cmd'] - a['cmd'] for a, b in zip(approach, approach[1:], strict=False)]


def test_deep_inherited_wire_releases_at_the_fast_rate_and_never_past_the_lanes():
  _, off = run(on=False)
  _, on = run(on=True)
  assert 'rel_fast' not in off[0]                                         # flag off: no new debug, no new state read
  assert max(steps(off)) <= 1.5 * DT + 1e-9
  fast = [r for r in on if r['rel_fast']]
  assert fast and fast[0] is on[0]                                       # the entry frame is 0.3+ below the law already
  assert max(steps(on)) <= stopping_flags.J_UP_FAST * DT + 1e-9 and max(steps(on)) > 1.5 * DT + 1e-6
  assert all(r['cmd'] <= r['lane_min'] + 1e-9 for r in on if r['phase'] == Phase.APPROACH_GLIDE)   # never shallower than a lane
  k = next(i for i, r in enumerate(on) if not r['rel_fast'])
  assert on[k - 1]['lane_min'] - on[k - 1]['cmd'] < stopping_flags.D_REL_OFF + stopping_flags.J_UP_FAST * DT   # ends inside the band
  assert not any(r['rel_fast'] for r in on[k:])                         # no re-arm: the wire follows the law at J_UP from here
  at = lambda rows, v: next(r['cmd'] for r in rows if r['v'] <= v)
  assert at(on, .4) > at(off, .4) + .3                                   # the wire is much closer to the lanes before the stop


def test_small_lag_is_unchanged():
  # a release the limiter lags by at most 0.14 (below D_REL) never arms: byte-identical
  _, off = run(on=False, v0=1.0, gap=5.0, cmd0=-1.15)
  _, on = run(on=True, v0=1.0, gap=5.0, cmd0=-1.15)
  assert .1 < max(r['lane_min'] - r['cmd'] for r in off) < stopping_flags.D_REL
  assert not any(r['rel_fast'] for r in on)
  assert [r['cmd'] for r in on] == [r['cmd'] for r in off]


def _never_latched(i, st):
  st['entry'] = False


@pytest.mark.parametrize('case, kw', [
  ('moving_lead', dict(entry=False, lead_v=.3)),                        # the ENTRY latch never confirmed a stopped lead
  ('vision_only', dict(age=0.)),                                        # no radar identity (track_age_s 0 stays < ATTR_TRUST_IN_S)
  ('above_v_rel', dict(v0=1.7, gap=6.5)),                               # out of the late in-band region until the release is spent
])
def test_out_of_scope_is_unchanged(case, kw):
  _, off = run(on=False, **kw)
  svc, on = run(on=True, **kw)
  if case == 'vision_only':
    on_cmds = [r['cmd'] for r in on]
    assert not any(r.get('rel_fast') for r in on) and on_cmds == [r['cmd'] for r in off]
  elif case == 'moving_lead':
    assert not any(r.get('rel_fast') for r in on) and [r['cmd'] for r in on] == [r['cmd'] for r in off]
  else:
    above = [r for r in on if r['v'] >= stopping_flags.V_REL]
    assert above and not any(r['rel_fast'] for r in above)


def test_a_lead_leaving_the_entry_window_ends_it():
  # a departing lead (vLead above the ENTRY window) un-confirms the stop: the release keeps J_UP from that frame
  def depart(i, st):
    st['entry'] = i < 5
  _, on = run(on=True, hazard=depart)
  assert all(r['rel_fast'] for r in on[:5]) and not any(r['rel_fast'] for r in on[5:])


def test_one_fast_release_per_approach():
  # after the wire met its target, a lane that deepens and releases again (radar noise, a flapping target) keeps J_UP
  def dip(i, st):
    st['a_target'] = -2.0 if 90 <= i < 100 else None
  svc, on = run(on=True, hazard=dip)
  k = next(i for i, r in enumerate(on) if not r['rel_fast'])
  assert k < 90 and svc._rel_spent and not any(r['rel_fast'] for r in on[k:])
  assert max(steps(on[100:])) <= 1.5 * DT + 1e-9


@pytest.mark.parametrize('loss', ['lead_lost', 'dropout', 'new_identity'])
def test_losing_the_lead_ends_the_fast_release(loss):
  def hz(i, st):
    if i >= 5:
      st.update(dict(lead_lost=dict(lead=False), dropout=dict(dropout=True, lead=False), new_identity=dict(age=.1, entry=False))[loss])
  _, on = run(on=True, hazard=hz)
  assert on[4]['rel_fast'] and not any(r.get('rel_fast') for r in on[5:])


def test_a_deepening_lane_still_binds_at_its_own_rate():
  # mid-release the planner lane drops below the wire: the fast release ends and the wire deepens at J_SAFE (safety_binding) as today
  def plan(i, st):
    st['a_target'] = -2.5 if 8 <= i < 30 else None
  _, off = run(on=False, hazard=plan)
  _, on = run(on=True, hazard=plan)
  k = 8
  assert on[k - 1]['rel_fast'] and not on[k]['rel_fast'] and on[k]['safety_binding']
  assert on[k]['cmd'] < on[k - 1]['cmd']                                 # deepens on the same frame
  assert not any(r['rel_fast'] for r in on[k:])                          # spent: the lane met the wire
  deep = [b['cmd'] - a['cmd'] for a, b in zip(on[k - 1:30], on[k:30], strict=False)]
  assert min(deep) >= -StoppingService().p.J_SAFE * DT - 1e-9
  assert min(r['cmd'] for r in on[k:30]) <= min(r['cmd'] for r in off[k:30]) + .35   # reaches the lane like flag off (from a shallower start)


def test_live_flags_combo_stays_inside_the_lanes(monkeypatch):
  # the car's flags (DELIVERED_LANDING, GAP_AWARE_TERMINAL, ENTRY_COMFORT_CAP, LANDING_LEAD_TOLERANCE, LAUNCH_FOLLOW) as deployed, both
  # FLAT_LANDING settings: the release never passes a lane and never steps above J_UP_FAST
  for flat in (True, False):
    monkeypatch.setattr(stopping_flags, 'FLAT_LANDING', flat)
    _, on = run(on=True)
    assert max(steps(on)) <= stopping_flags.J_UP_FAST * DT + 1e-9
    assert all(r['cmd'] <= r['lane_min'] + 1e-9 for r in on if r['phase'] == Phase.APPROACH_GLIDE)
