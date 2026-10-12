"""DELIVERED_LANDING (cycle 2026-10-13 WS LANDING): the flat landing eases toward the delivered target only while the car still
brakes deeper than it, and every hazard falls back to today's landing."""
from collections import deque
from dataclasses import replace

import pytest

from openpilot.selfdrive.controls.lib import stopping_flags
from openpilot.selfdrive.controls.lib.stopping_service import Phase, StoppingService
from openpilot.selfdrive.controls.lib.tests.test_stopping_service import make_signals
from openpilot.selfdrive.controls.tests.test_stopping_stage1 import arm, lc, step  # noqa: F401  (lc is a fixture)

DT = .01


@pytest.fixture(autouse=True)
def flags(monkeypatch):
  monkeypatch.setattr(stopping_flags, 'FLAT_LANDING', True)
  monkeypatch.setattr(stopping_flags, 'DELIVERED_LANDING', True)


def run(*, gain=1., push=0., gap=5., cmd0=-.5, hazard=None, hazard_v=.35, loss_v=None, frames=400):
  """Closed loop: brake delay 0.15 s (gain, + push: creep / downhill), carState aEgo = the delivered decel 0.1 s later and 0.06
  shallower (the measured bias). hazard(i, state) may change the inputs from vEgo hazard_v on; loss_v: delivered decel lost
  (KCS2 release loss: -0.10) from that speed on. Returns per-frame rows until the wheel-stop latch."""
  svc = StoppingService()
  svc._last_cmd, svc._floor_v_peak = cmd0, .6   # the descent captures cmd0 on the first frame (0.5 m/s, coming down from 0.6)
  v, wire = .5, deque([cmd0] * 15, maxlen=15)
  real = deque([gain * cmd0 + push] * 10, maxlen=10)
  st = dict(gap=gap, age=10., a_target=None, a_coast=0., lead_v=0., dropout=False, latch=True)
  rows, still, hit = [], 0., False
  for i in range(frames):
    if hazard is not None and (hit or v <= hazard_v):
      hit = True
      hazard(i, st)
    st['age'] = st['age'] + DT if st['age'] > 0. else st['age']
    wheel = still >= .25
    sig = replace(make_signals(d_gap=st['gap'], a_coast=st['a_coast'], latch=st['latch'], wheel=wheel,
                               dropout=st['dropout'], gap_source='decay' if st['dropout'] else None), track_age_s=st['age'])
    res = svc.update(engaged=True, v_ego=v, a_ego=real[0] + .06, a_target=st['a_target'], should_stop=True, dts_planner=10.,
                     planner_min_limit=-3.5, signals=sig, lead_status=not st['dropout'], lead_v=st['lead_v'],
                     wire_accel=svc._last_cmd, dt=DT, a_target_trajectory=st['a_target'])
    rows.append({**res.debug, 'v': v, 'cmd': res.accel, 'a': real[-1], 'phase': res.phase, 'land': svc.landing_cmd})
    if wheel:
      break
    wire.append(res.accel)
    a = gain * wire[0] + push
    if loss_v is not None and v <= loss_v:
      a = -.10
    if v == 0.:
      a = 0.   # at rest the brake holds: no deceleration to measure
    real.append(a)
    v = max(v + a * DT, 0.)
    st['gap'] -= v * DT
    still = still + DT if v <= .06 else 0.
  return svc, rows


def test_eases_to_the_delivered_target_and_freezes():
  svc, rows = run()
  moving = [r for r in rows if r['phase'] != Phase.RAMP_TO_HOLD]
  cmds = [r['cmd'] for r in moving]
  assert max(cmds) > stopping_flags.A_FLOOR + .05                       # it eases
  assert max(cmds) <= stopping_flags.A_LAND_CMD_MIN + 1e-9              # never shallower than the command bound
  steps = [b - a for a, b in zip(cmds, cmds[1:], strict=False)]
  assert max(steps) <= svc.p.J_TERMINAL_DESCENT * DT + 1e-9             # at J_TERMINAL_DESCENT
  eased = [r for r in moving if r['land'] is not None]
  assert eased and all(b['cmd'] >= a['cmd'] - 1e-9 for a, b in zip(eased, eased[1:], strict=False))  # no self-made re-deepen
  assert not any(r['land_failsafe'] for r in rows)
  landing = [r['a'] for r in moving if 0. < r['v'] <= .15]
  assert -.45 <= sum(landing) / len(landing) <= -.33                   # delivered at the stop: Radek's golden band
  assert all(r['v'] < stopping_flags.V_LAND for r in eased)
  slow = [(a, b) for a, b in zip(rows, rows[1:], strict=False) if a['v'] <= svc.p.V_CREEP_HOLD_SECURE and b['phase'] != Phase.RAMP_TO_HOLD]
  assert slow and all(b['land_up'] == a['land_up'] for a, b in slow)   # an ease below 0.10 m/s would land after the stop


def test_hot_capture_eases_toward_the_target():
  _, rows = run(cmd0=-.7)
  moving = [r for r in rows if r['phase'] != Phase.RAMP_TO_HOLD]
  assert max(r['cmd'] for r in moving) > -.45 and not any(r['land_failsafe'] for r in rows)
  landing = [r['a'] for r in moving if 0. < r['v'] <= .15]
  assert sum(landing) / len(landing) > -.5                              # today: -0.70 held into the stop


def test_head_landing_without_the_flag(monkeypatch):
  monkeypatch.setattr(stopping_flags, 'DELIVERED_LANDING', False)
  svc, rows = run()
  assert all(r['land'] is None and 'land_up' not in r for r in rows)
  assert max(r['cmd'] for r in rows if r['v'] < .45) <= stopping_flags.A_FLOOR + 1e-9


@pytest.mark.parametrize('case', ['first_gear_creep', 'downhill'])
def test_under_delivering_car_is_not_eased(case):
  # 1st gear with creep on delivers ~60 % of the command; a downhill push takes ~0.2 off: already at the target, no ease
  _, rows = run(**(dict(gain=.6) if case == 'first_gear_creep' else dict(push=.2)))
  assert all(r['land'] is None for r in rows)
  assert max(r['cmd'] for r in rows if r['v'] < .45) <= stopping_flags.A_FLOOR + 1e-9


def test_release_loss_stall_guard_re_deepens():
  # KCS2 release loss: after the ease the delivered decel collapses to -0.10 at 0.3 m/s
  svc, rows = run(loss_v=.30)
  k = next(i for i, r in enumerate(rows) if r['land_failsafe'])
  assert rows[k - 1]['land'] is not None
  # reaction: aEgo sees the loss 0.1 s later; the guard trips on the first frame it reads shallower than A_LAND_STALL
  lost = next(i for i, r in enumerate(rows) if r['v'] <= .30)
  assert k - lost <= 12
  after = rows[k:]
  steps = [b['cmd'] - a['cmd'] for a, b in zip(after, after[1:], strict=False) if b['phase'] != Phase.RAMP_TO_HOLD]
  assert steps and max(steps) <= 1e-9                                   # never eases again
  assert min(steps) >= -svc.p.J_TERMINAL_DESCENT * DT - 1e-9            # re-deepens at J_TERMINAL_DESCENT
  back = next(i for i, r in enumerate(after) if r['land'] is None)
  assert after[back]['cmd'] <= stopping_flags.A_FLOOR + 1e-9            # today's landing level restored
  assert back * DT <= rows[k - 1]['land_up'] / svc.p.J_TERMINAL_DESCENT + .02


def _identity(i, st):
  st['age'] = 0. if st.get('_id') is None else st['age']
  st['_id'] = True


HAZARDS = {
  'identity': _identity,
  'nearer': lambda i, st: st.update(gap=min(st['gap'], 4.4)),
  'safety_lane': lambda i, st: st.update(a_target=-1.0),
  'uphill': lambda i, st: st.update(a_coast=-.3),
  'unconfirmed': lambda i, st: st.update(latch=False),
  'thin_gap': lambda i, st: st.update(gap=min(st['gap'], 3.6)),
}


@pytest.mark.parametrize('hazard', list(HAZARDS))
def test_hazard_falls_back_to_today(hazard):
  svc, rows = run(gap=5.0, hazard=HAZARDS[hazard], hazard_v=.38)
  k = next(i for i, r in enumerate(rows) if r['v'] <= .38)
  assert any(r['land'] is not None for r in rows[:k])                   # it was easing when the hazard came
  assert rows[k]['land_failsafe'] or rows[k + 1]['land_failsafe']
  ups = [r['land_up'] for r in rows[k:]]
  assert all(b <= a + 1e-12 for a, b in zip(ups, ups[1:], strict=False))   # the ease only unwinds (lanes keep their own shape)
  gone = next(i for i, u in enumerate(ups) if u == 0.)                     # back to today's landing at J_TERMINAL_DESCENT
  assert gone * DT <= rows[k]['land_up'] / svc.p.J_TERMINAL_DESCENT + .02


def test_legacy_law_never_eases(monkeypatch):
  monkeypatch.setattr(stopping_flags, 'SERVICE_APPROACH_LAW', 'legacy')
  _, rows = run()
  assert all(r['land'] is None for r in rows)


def test_post_stop_roll_before_the_hold_build():
  svc, rows = run()
  assert rows[-1]['phase'] == Phase.RAMP_TO_HOLD
  landed = svc._last_cmd
  assert landed > stopping_flags.A_FLOOR                               # the hold builds from the eased landing
  sig = make_signals(d_gap=4.5, latch=True, wheel=True)
  kw = dict(engaged=True, a_target=None, should_stop=True, dts_planner=10., planner_min_limit=-3.5, signals=sig,
            lead_status=True, lead_v=0., dt=DT)
  svc.update(v_ego=0., a_ego=0., wire_accel=svc._last_cmd, **kw)
  before = svc._last_cmd
  res = svc.update(v_ego=.04, a_ego=.1, wire_accel=svc._last_cmd, **kw)  # creep rolls the car before the hold is built
  assert svc.ev.finish_roll(.04)
  assert res.accel == pytest.approx(before - svc.p.J_SAFE * DT)         # the roll arrest builds at J_SAFE


def test_creep_guard_yields_only_below_v_land_while_easing(lc, monkeypatch):  # noqa: F811
  arm(lc)
  svc = lc._service_shadow_svc
  wires, cmds = [], []
  update = svc.update

  def capture(**kw):
    res = update(**kw)
    cmds.append((kw['v_ego'], svc.landing_cmd))
    return res

  monkeypatch.setattr(svc, 'update', capture)
  v = 1.2
  while v > .12:
    wires.append((v, step(lc, v=v, gap=5.0 + v, target=-.5)))
    v -= .004
  eased = [(v_, c) for v_, c in cmds if c is not None]
  assert eased and all(v_ < stopping_flags.V_LAND for v_, _ in eased)
  for (v_, wire), (_, c) in zip(wires, cmds, strict=False):
    if c is None:
      assert wire <= stopping_flags.A_FLOOR + 1e-9 or v_ >= stopping_flags.V_GUARD_EASE[1]
    else:
      assert wire == pytest.approx(c)                                   # the guard yields to this frame's eased landing


def test_unresponsive_car_stops_at_the_command_bound():
  # aEgo never shows the ease (a stale or wrong measurement): the command stops at A_LAND_CMD_MIN, above the ESC brake-off band
  svc = StoppingService()
  svc._last_cmd, svc._floor_v_peak = -.5, .6
  cmds = []
  for i in range(200):
    v = .5 - i * .0024
    res = svc.update(engaged=True, v_ego=v, a_ego=-.6, a_target=None, should_stop=True, dts_planner=10., planner_min_limit=-3.5,
                     signals=replace(make_signals(d_gap=5., latch=True), track_age_s=10. + i * DT), lead_status=True, lead_v=0.,
                     wire_accel=svc._last_cmd, dt=DT)
    cmds.append(res.accel)
  assert max(cmds) == pytest.approx(stopping_flags.A_LAND_CMD_MIN)


def test_lane_between_the_eased_and_todays_landing_holds_the_ease():
  # the planner lane at -0.44 (recorded landings: -0.30..-0.62) takes the wire through the min(); it is shallower than today's -0.50, so
  # it is no hazard: the ease holds where it is instead of re-deepening to -0.50 (a self-made re-grab in the last 0.4 s)
  svc, rows = run(hazard=lambda i, st: st.update(a_target=-.44), hazard_v=.38)
  k = next(i for i, r in enumerate(rows) if r['v'] <= .38)
  moving = [r for r in rows[k:] if r['phase'] != Phase.RAMP_TO_HOLD]
  assert not any(r['land_failsafe'] for r in rows)
  assert all(r['cmd'] >= stopping_flags.A_FLOOR - 1e-9 for r in moving)                     # never deeper than today's landing
  bound = [r for r in moving if r['safety_binding']]
  assert bound and max(r['land_up'] for r in bound) == pytest.approx(min(r['land_up'] for r in bound))   # the ease holds while bound


def test_no_ease_below_v_creep_hold_secure():
  # a late capture: the ease that starts at 0.14 m/s stops at 0.10 m/s (its brake response would arrive after the wheel stop)
  svc = StoppingService()
  svc._last_cmd, svc._floor_v_peak = -.5, .6
  ups = []
  for i in range(30):
    v = .14 - i * .004
    svc.update(engaged=True, v_ego=v, a_ego=-.6, a_target=None, should_stop=True, dts_planner=10., planner_min_limit=-3.5,
               signals=replace(make_signals(d_gap=5., latch=True), track_age_s=10. + i * DT), lead_status=True, lead_v=0.,
               wire_accel=svc._last_cmd, dt=DT)
    ups.append((v, svc._land_up))
  assert ups[-1][1] > 0. and all(u == ups[-1][1] for v, u in ups if v <= svc.p.V_CREEP_HOLD_SECURE)
