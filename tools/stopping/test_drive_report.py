import os
from pathlib import Path

import numpy as np
import pytest

from openpilot.tools.stopping import drive_replay as DR
from openpilot.tools.stopping import drive_report as D

E3 = 'RELEASE_END_STOPPED_LEAD_REHOLD'


def _route(v, t0=0.0, dt=0.01, long_active=None, gas=None, brake=None):
  """Stage-A style arrays for a speed trace (100 Hz carState/carControl, 20 Hz planner)."""
  n = len(v)
  t = t0 + dt * np.arange(n)
  z = np.zeros(n)
  la = np.ones(n) if long_active is None else np.asarray(long_active, float)
  cs = np.column_stack([t, v, z, (np.asarray(v) < 0.01).astype(float), z if gas is None else gas, z if brake is None else brake, v])
  cc = np.column_stack([t, la, la, z, z])
  lp = np.column_stack([t[::5], np.zeros(len(t[::5])), np.zeros(len(t[::5])), z[::5], z[::5], z[::5]])
  return dict(cs=cs, cc=cc, lp=lp, rs=np.zeros((0, 10)), ctl=np.column_stack([t, np.ones(n)]), logs=[])


def _stop_trace():
  # 10 m/s for 5 s, -1 m/s^2 to rest at 15 s, hold 10 s, launch
  v = np.r_[np.full(500, 10.0), np.linspace(10.0, 0.0, 1000), np.zeros(1000), np.linspace(0.0, 3.0, 300)]
  return v


def test_engaged_stop_census_rule():
  R = _route(_stop_trace())
  stops = D.engaged_stops(R)
  assert len(stops) == 1
  s = stops[0]
  assert s['kind'] == 'stop'
  first_below = R['cs'][np.flatnonzero(R['cs'][:, 1] < 0.10)[0], 0]
  assert s['t_ws'] == pytest.approx(first_below, abs=0.011)


def test_stop_needs_long_active_through_the_last_4_s():
  v = _stop_trace()
  la = np.ones(len(v))
  la[1300:1400] = 0.0   # disengaged 2-1 s before the wheel stop
  assert D.engaged_stops(_route(v, long_active=la)) == []


def test_rolling_stop():
  v = np.r_[np.full(300, 5.0), np.linspace(5.0, 0.3, 300), np.linspace(0.3, 2.0, 300)]
  stops = D.engaged_stops(_route(v))
  assert [s['kind'] for s in stops] == ['rolling']


def test_holds_and_spans_merge():
  R = _route(_stop_trace())
  hl = D.holds(R)
  assert len(hl) == 1 and hl[0]['began'] == 'engaged_stop' and hl[0]['t_end'] - hl[0]['t_start'] > 9.0
  spans = D.replay_spans('00009999--abcdef0123', R, D.engaged_stops(R), hl)
  assert len(spans) == 1   # the stop window and the hold window overlap
  assert spans[0]['lo'] == 0.5 and spans[0]['span'].startswith('9999_abcd_')


def test_zig_pumps_counts_dip_bite_release():
  t = np.arange(0, 3, 0.01)
  decel = np.interp(t, [0, 0.5, 1.0, 1.5, 2.0, 3.0], [0.3, 0.1, 0.6, 0.2, 0.2, 0.2])   # ease, bite, release
  assert D.zig_pumps(t, decel)[0] == 1
  assert D.zig_pumps(t, np.linspace(0.1, 0.8, len(t)))[0] == 0


def test_stopreq_pairs_at_rest_only_without_driver():
  R = _route(np.zeros(500))
  R['scc12'] = np.column_stack([np.arange(0, 5, 0.02), np.zeros(250), np.ones(250), np.zeros(250)])
  R['scc12'][100:120, 2] = 0.0   # clear at 2.0 s, set again at 2.4 s
  no_driver = lambda a, b: False  # noqa: E731
  assert D.stopreq_pairs(R, 0.0, 5.0, no_driver) == [(2.0, 2.4, 'set->clear')]
  assert D.stopreq_pairs(R, 0.0, 5.0, lambda a, b: True) == []


def test_match_bookmarks_window_nearest_hold():
  stops = [dict(id='a', t_ws=100.0, m=dict(t_lo=90.0)), dict(id='b', t_ws=300.0, m=dict(t_lo=290.0))]
  hl = [dict(t_start=500.0, t_end=560.0)]
  got = D.match_bookmarks([104.0, 125.0, 550.0, 900.0], stops, hl)
  assert [(g['stop'], g['how']) for g in got] == [('a', 'window'), ('a', 'nearest'), (None, 'hold'), (None, 'none')]


def _arm(n=300, **cols):
  a = dict(t=np.arange(n) * 0.01, wire=np.full(n, -0.7), phase=np.full(n, 4), stopreq=np.ones(n, int), lcs=np.full(n, 2),
           rehold=np.zeros(n, int), active=np.ones(n, int))
  for k, v in cols.items():
    a[k] = v
  return a


def test_diff_events_one_run_with_phase_sequences():
  on, off = _arm(), _arm()
  off['phase'][100:110] = 0
  off['lcs'][100:110] = 3
  off['wire'][100:130] = 0.1
  ev = D.diff_events(on, off)
  assert len(ev) == 1 and ev[0]['t0'] == 1.0 and ev[0]['phase_off'] == ['HOLD', 'INACTIVE', 'HOLD']
  assert ev[0]['lcs_off'] == [2, 3, 2]


def _inputs(n=300, v=None, gas=None):
  return dict(t=np.arange(n) * 0.01, v=np.zeros(n) if v is None else v, gas=np.zeros(n, int) if gas is None else gas,
              brake=np.zeros(n, int), lead=np.ones(n, int), gap=np.full(n, 5.0), vl=np.zeros(n))


def test_e3_checks_clean_rehold_and_each_violation():
  rh = np.zeros(300, int)
  rh[50:200] = 1
  on, off = _arm(rehold=rh), _arm()
  off['stopreq'][60:] = 0          # flag off clears StopReq at 0.6 s while the lead is still stopped (the race)
  on['stopreq'][200:] = 0
  c = D.e3_checks(on, off, _inputs(), live=True)
  assert len(c) == 1 and c[0]['violations'] == [] and c[0]['later_s'] == pytest.approx(1.4)
  assert 'race removed' in c[0]['notes'][0]
  bad = _arm(rehold=rh)
  bad['lcs'][120] = 3
  bad['stopreq'][130:140] = 0
  bad['active'][150] = 0
  viol = D.e3_checks(bad, off, _inputs(), live=True)[0]['violations']
  assert any("'starting'" in x for x in viol) and any('StopReq clear' in x for x in viol) and any('ownership' in x for x in viol)


def test_e3_motion_rule_only_on_a_live_drive():
  rh = np.zeros(300, int)
  rh[50:200] = 1
  v = np.zeros(300)
  v[60:120] = 0.5   # 0.3 m toward a stopped lead under the re-hold, no driver input (logged motion)
  on = _arm(rehold=rh)
  live = D.e3_checks(on, _arm(), _inputs(v=v), live=True)[0]['violations']
  assert any('moved 0.25 m' in x for x in live)
  assert D.e3_checks(on, _arm(), _inputs(v=v), live=False)[0]['violations'] == []


def test_release_end_race_detector():
  n = 600
  v = np.zeros(n)
  v[110:300] = 0.4   # creeps 0.76 m before the lead departs at 4.0 s
  R = _route(v)
  t = R['cs'][:, 0]
  R['rs'] = np.column_stack([t, np.ones(n), np.where(t < 4.0, 5.0, 8.0), np.where(t < 4.0, 0.0, 2.0), np.zeros((n, 6))])
  R['logs'] = [(1.0, 'INACTIVE', 'RELEASE'), (1.0, 'RAMP_TO_HOLD', 'INACTIVE')]
  r = D.release_end_races(R)
  assert len(r) == 1 and r[0]['race'] and r[0]['travel_before_lead'] == pytest.approx(0.76, abs=0.02) and r[0]['reentry'] == 1.0
  R['cs'][105:, 4] = 1.0   # the driver's gas before the motion: not a race
  assert not D.release_end_races(R)[0]['race']


def test_fidelity_ok_and_drift():
  R = dict(logs=[(1.5, 'HOLD', 'RAMP_TO_HOLD')], scc12=np.zeros((0, 4)))
  t = np.arange(0, 3, 0.01)
  ph = np.where(t >= 1.5, 4, 3)
  r = dict(t=t, wire=np.zeros(len(t)), rec=np.zeros(len(t)), stopreq=np.ones(len(t), int), phase=ph)
  spans = [dict(span='s')]
  assert D.fidelity(R, spans, {'s': r})['ok']
  r2 = dict(r, rec=np.where(t > 2.0, 0.2, 0.0))
  assert not D.fidelity(R, spans, {'s': r2})['ok']
  r3 = dict(r, phase=np.full(len(t), 3))
  assert D.fidelity(R, spans, {'s': r3})['phase_unmatched'] == 1


def test_flag_value_and_snapshot_override(tmp_path, monkeypatch):
  if DR.full_sha('2e39594627') is None or DR.full_sha('1e0327943d') is None:
    pytest.skip('car-build commits not in this clone')
  assert DR.flag_value('2e39594627', E3) is True
  assert DR.flag_value('1e0327943d', E3) is None
  monkeypatch.setattr(DR, 'WORK', tmp_path)
  files, shas, _ = DR.snapshot(DR.full_sha('2e39594627'), {E3: False})
  body = Path(files['openpilot.selfdrive.controls.lib.stopping_flags']).read_text()
  assert f'{E3} = False' in body and f'{E3} = True' not in body
  assert str(tmp_path) in files['openpilot.selfdrive.controls.lib.stopping_flags']


@pytest.mark.skipif(not (DR.WORK / 'scan' / '00002232--2a250f66e2--82.npz').exists(), reason='no stage-A scan of 00002232 cached')
def test_smoke_car_build_drive_2232():
  """The 9 engaged stops of the cycle-1003 census (R.NEW) on 00002232 from the cached scan."""
  R = D.load_route('00002232--2a250f66e2')
  got = [s['t_ws'] for s in D.engaged_stops(R)]
  assert got == [330.33, 2069.14, 2143.67, 2262.31, 2422.59, 4940.01, 5072.78, 5199.95, 5352.71]
  races = [r['t'] for r in D.release_end_races(R) if r['race']]
  assert races == [4904.04]


def test_hook_line(capsys):
  assert D.main(['--hook']) == 0
  assert 'drive_report.py' in capsys.readouterr().out
  assert os.path.basename(D.__file__) == 'drive_report.py'


# ---- SANTA_FE_STOP_LINE trial rules: a clean synthetic stop trips nothing; each injection trips its rule ---------------------
LINE = 'SANTA_FE_STOP_LINE'


def _line_case(n_p=400):
  """A stopped-lead approach (planner 20 Hz, replay 100 Hz): ego 5 -> 0 m/s by t = 15 s, lead stopped at a 4.5 m rest, the line
  armed 5-12 s at -0.6 (the flag-off plan -0.4), released at J to the plan, then the service holds; launch at 19 s."""
  t = np.arange(n_p) * 0.05
  v = np.clip(5.0 - t / 3.0, 0.0, None)
  s = np.r_[0.0, np.cumsum(v[1:] * 0.05)]
  d = s[-1] + 4.5 - s
  aoff = np.where(t < 15.0, -0.4, 0.0)
  lf = np.full(n_p, np.nan)
  armed = np.full(n_p, np.nan)
  arm = (t >= 5.0) & (t < 12.0)
  lf[arm], armed[arm] = -0.6, 1.0
  rel = np.flatnonzero(t >= 12.0)[:2]     # J release 0.125 per tick from -0.6 to the plan -0.4
  lf[rel], armed[rel] = (-0.475, -0.4 + 1e-3), 0.0
  aon = np.where(np.isfinite(lf), np.fmin(aoff, np.nan_to_num(lf, nan=9.0)), aoff)
  P = dict(t=t, ns=t * 1e9, v=v, d=d, vl=np.zeros(n_p), tid=np.full(n_p, 7.0), mp=np.ones(n_p), engaged=np.ones(n_p),
           override=np.zeros(n_p), at=aon, ss=np.zeros(n_p), lf=lf, armed=armed, cmd=np.where(np.isfinite(lf), aoff, np.nan),
           in_armed=np.r_[0.0, np.nan_to_num(armed[:-1])], cert=np.ones(n_p), pers=np.ones(n_p), prov=np.ones(n_p),
           cls=np.ones(n_p), log_at=aon, log_ss=np.zeros(n_p))
  Pf = dict(P, at=aoff.copy())
  n_l = n_p * 5
  tl = np.arange(n_l) * 0.01
  vl_ = np.interp(tl, t, v)
  wire_off = np.where(tl < 15.0, -0.4, np.where(tl < 19.0, -0.7, 0.5))
  wire_on = np.interp(tl, t, np.where(np.isfinite(lf), np.fmin(-0.4, np.nan_to_num(lf, nan=9.0)), -0.4))
  wire_on = np.where(tl < 15.0, wire_on, wire_off)
  phase = np.where(tl < 13.0, 0, np.where(tl < 15.0, 1, np.where(tl < 19.0, 4, 5)))
  base = dict(t=tl, stopreq=np.where((tl >= 15.0) & (tl < 19.0), 1, 0), lcs=np.where(tl < 15.0, 1, np.where(tl < 19.0, 2, 3)),
              owning=np.ones(n_l, int), band=np.zeros(n_l, int), active=np.ones(n_l, int), rehold=np.zeros(n_l, int))
  Ln = dict({k: x.copy() for k, x in base.items()}, wire=wire_on, phase=phase.copy())
  Lf = dict({k: x.copy() for k, x in base.items()}, wire=wire_off.copy(), phase=phase.copy())
  gap = np.interp(tl, t, d) + np.where(tl > 18.0, (tl - 18.0) * 1.0, 0.0)   # the lead departs at 18 s, 1 m/s
  inp = dict(t=tl, v=vl_, gas=np.zeros(n_l, int), brake=np.zeros(n_l, int), act=np.ones(n_l, int), lead=np.ones(n_l, int), gap=gap,
             vl=np.where(tl > 18.0, 1.0, 0.0), tid=np.full(n_l, 7))
  stop = dict(id='9999_15.00', span='sp', kind='stop', t_ws=15.0, rest=4.5, min_gap=4.5, a_stop=-0.45)
  return P, Pf, Ln, Lf, inp, stop


def _rules(P, Pf, Ln, Lf, inp, stop, R=None):
  inp['rec'] = Ln['wire']   # a live drive: the logged command is the flag-on arm's
  ev = D.line_span_events(P, Pf, Ln, Lf, inp, 'sp')
  row, ev2 = D.line_stop(stop, P, Pf, Ln, Lf, inp, R or {})
  return ev + ev2, row


def test_line_clean_stop_trips_nothing():
  ev, row = _rules(*_line_case())
  assert ev == [] and D.line_decide(ev) == []
  assert row['line'] and row['line_t'] == pytest.approx(5.0) and row['plan_extra'] == pytest.approx(0.2)
  assert row['go_on'] == row['go_off'] == pytest.approx(19.0, abs=0.02) and row['h5_dgap'] == 0.0


def _trips(ev):
  return [r for r, _ in D.line_decide(ev)]


@pytest.mark.parametrize('inject, rule', [
  ('release_step', 'R1'), ('positive_floor', 'R1'), ('positive_cap', 'R1'), ('burst_release', 'R1'), ('provenance', 'R1'),
  ('uncertified', 'R2'), ('pump', 'R4'), ('crawl_grab', 'R7'), ('creep_regrab', 'R9'), ('legacy_stopping', 'R10'),
  ('chatter', 'R10'), ('pid_hold', 'R10'), ('starting_hold', 'R10'), ('late_go', 'H5'), ('no_go', 'H5'), ('false_go', 'H5'),
  ('landing_close', 'R5'), ('landing_long', 'R5'),
  ('downhill', 'R8')])
def test_line_rule_injections_trip(inject, rule):
  P, Pf, Ln, Lf, inp, stop = _line_case()
  t, tl = P['t'], Ln['t']
  R = None
  k = int(np.searchsorted(t, 8.0))
  if inject == 'release_step':        # a binding line releases 0.3 in one tick (OFF flat)
    P['lf'][k:k + 3] = -0.3
    P['at'][k:k + 3] = -0.4
  elif inject == 'positive_floor':
    P['lf'][k:k + 3] = 0.2
  elif inject == 'positive_cap':      # Astra P2: a positive seed holds +0.56 below a +0.8 command
    P['cmd'][k:k + 8], P['lf'][k:k + 8], P['at'][k:k + 8] = 0.8, 0.56, 0.56
  elif inject == 'burst_release':     # a 4-tick level burst of a still lead releases the armed line
    P['vl'][k:k + 4] = 0.6
    P['armed'][k + 1:k + 4] = 0.0
  elif inject == 'provenance':        # an explicit provenance rejection while the line stays armed and deepens
    P['prov'][k:k + 2] = 0.0
    P['lf'][k + 1] = -0.7
  elif inject == 'uncertified':       # the certificate drops while the line holds 0.4 below OFF
    P['cert'][k:k + 10] = 0.0
    P['lf'][k:k + 10] = P['at'][k:k + 10] = -0.8
  elif inject == 'pump':              # release + re-arm within 1 s above 2.5 m/s; the wire gives and re-takes 0.2 vs OFF
    k = int(np.searchsorted(t, 6.0))
    P['armed'][k:k + 10] = 0.0
    j = int(np.searchsorted(tl, 6.0))
    Ln['wire'][j - 50:j + 50] = -0.6
    Ln['wire'][j + 50:j + 80] = -0.4
    Ln['wire'][j + 80:j + 200] = -0.6
  elif inject == 'crawl_grab':        # 1.5 s crawl at 0.1 m/s, then the flag-on wire deepens 0.3 before the wheel stop
    j0, j1 = int(np.searchsorted(tl, 13.0)), int(np.searchsorted(tl, 15.0))
    inp['v'][j0:j1] = 0.1
    Ln['wire'][j0:j1] = np.linspace(-0.3, -0.6, j1 - j0)
    Lf['wire'][j0:j1] = -0.3
    Ln['band'][j0:j1] = 1
  elif inject == 'creep_regrab':      # RELEASE -> APPROACH at 0.3 m/s behind a creeping lead, the flag-on wire re-grabs 0.4
    j = int(np.searchsorted(tl, 10.0))
    Ln['phase'][j - 10:j], Ln['phase'][j:j + 100] = 5, 1
    inp['v'][j - 10:j + 100], inp['vl'][j - 10:j + 100] = 0.3, 0.3
    Ln['wire'][j:j + 100] = -1.0   # 0.4 below the -0.6 before the re-entry
  elif inject == 'legacy_stopping':   # after the line: stopping wire -1.2 below 1.5 m/s without ownership, OFF owned
    j0, j1 = int(np.searchsorted(tl, 13.5)), int(np.searchsorted(tl, 14.2))
    Ln['lcs'][j0:j1], Ln['owning'][j0:j1], Ln['wire'][j0:j1] = 2, 0, -1.2
  elif inject == 'chatter':           # StopReq clear + set within 0.5 s at rest, flag on only
    j = int(np.searchsorted(tl, 16.5))
    Ln['stopreq'] = Ln['stopreq'].copy()
    Ln['stopreq'][j:j + 50] = 0
  elif inject in ('pid_hold', 'starting_hold'):   # the flag-on service holds at rest in pid / LongControl 'starting' under it
    j0, j1 = int(np.searchsorted(tl, 16.0)), int(np.searchsorted(tl, 16.5))
    Ln['lcs'][j0:j1] = 1 if inject == 'pid_hold' else 3
  elif inject == 'late_go':           # flag on launches 1.5 s after OFF: the lead has departed 1.5 m further
    j0, j1 = int(np.searchsorted(tl, 19.0)), int(np.searchsorted(tl, 20.5))
    Ln['wire'] = Ln['wire'].copy()
    Ln['wire'][j0:j1], Ln['stopreq'] = -0.7, np.where((tl >= 15.0) & (tl < 20.5), 1, 0)
  elif inject == 'no_go':             # flag on never commands the launch the flag-off arm commands
    Ln['wire'] = np.where(tl >= 15.0, -0.7, Ln['wire'])
    Ln['stopreq'] = np.where(tl >= 15.0, 1, 0)
  elif inject == 'false_go':          # flag on goes at 16 s toward the still-stopped lead
    Ln['wire'] = np.where((tl >= 16.0) & (tl < 19.0), 0.4, Ln['wire'])
    Ln['stopreq'] = np.where((tl >= 15.0) & (tl < 16.0), 1, 0)
  elif inject in ('landing_close', 'downhill'):   # the band brakes 0.2 less than OFF in the band: the flag-off rest is ~0.4 m longer
    j0, j1 = int(np.searchsorted(tl, 13.0)), int(np.searchsorted(tl, 15.0))
    Ln['wire'][j0:j1] = Lf['wire'][j0:j1] + 0.2
    Ln['band'][j0:j1] = 1
    stop['rest'] = stop['min_gap'] = 3.3 if inject == 'landing_close' else 3.7
    if inject == 'downhill':
      R = dict(cc=np.column_stack([tl, np.ones(len(tl)), np.ones(len(tl)), np.zeros(len(tl)), np.full(len(tl), -0.03)]))
  elif inject == 'landing_long':      # the line brakes 0.2 more for 5 s and the car rests 6.5 m back
    j0, j1 = int(np.searchsorted(tl, 10.0)), int(np.searchsorted(tl, 15.0))
    Ln['wire'][j0:j1] = Lf['wire'][j0:j1] - 0.2
    inp['gap'] = inp['gap'] + 2.0
  ev, _ = _rules(P, Pf, Ln, Lf, inp, stop, R=R)
  assert rule in {e['rule'] for e in ev}, ev
  assert rule in _trips(ev), (inject, ev)


def test_line_count_rules_need_two_events():
  """R3 (queue restart) and R6 (a_stop) revert on 2 events per drive; R4 on 2 pumps >= 0.10 or one >= 0.15."""
  def e(rule, value=1.0):
    return dict(rule=rule, span='s', t=0.0, v=0.0, what='', value=value)
  assert _trips([e('R3')]) == [] and _trips([e('R3'), e('R3')]) == ['R3']
  assert _trips([e('R6')]) == [] and _trips([e('R6'), e('R6')]) == ['R6']
  assert _trips([e('R4', 0.12)]) == [] and _trips([e('R4', 0.12), e('R4', 0.11)]) == ['R4'] and _trips([e('R4', 0.16)]) == ['R4']
  assert _trips([e('R4', 0.07), e('R4', 0.08)]) == []


def test_line_queue_restart_and_a_stop_inject():
  P, Pf, Ln, Lf, inp, stop = _line_case()
  t, tl = P['t'], Ln['t']
  for a in (5.2, 6.2):                # two launches of the lead above 2.5 m/s while the line binds 0.2 below OFF for 0.7 s
    k0, k1 = int(np.searchsorted(t, a)), int(np.searchsorted(t, a + 0.7))
    P['vl'][k0:k1] = 0.8
    P['d'][k0:k1] += np.arange(k1 - k0) * 0.04   # it moves (not a burst)
  ev, _ = _rules(P, Pf, Ln, Lf, inp, stop)
  assert [x['rule'] for x in ev].count('R3') == 2 and 'R3' in _trips(ev)
  P, Pf, Ln, Lf, inp, stop = _line_case()
  j0 = int(np.searchsorted(tl, 14.5))
  Ln['wire'][j0:int(np.searchsorted(tl, 15.0)) + 1] = -0.7   # the flag-on stop wire 0.3 deeper over the last 0.5 s
  stop['a_stop'] = -0.70
  ev, row = _rules(P, Pf, Ln, Lf, inp, stop)
  assert [x['rule'] for x in ev] == ['R6'] and row['a_off'] == pytest.approx(-0.4, abs=0.02)
  ev, _ = _rules(P, Pf, Ln, Lf, inp, dict(stop, a_stop=-0.60))   # not <= -0.65: no event
  assert ev == []


def test_plan_targets_delta_on_the_logged_plan():
  ns = np.arange(4) * 5e7
  P = dict(ns=ns, at=np.array([-0.5, -0.6, -0.7, -0.7]), ss=np.zeros(4), log_at=np.array([-0.4, -0.4, -0.5, -0.5]))
  ref = dict(ns=ns, at=np.array([-0.4, -0.4, -0.5, -0.5]), ss=np.zeros(4))
  frames = [dict(t=x * 1e-9 + 0.01, target=y, should_stop=False) for x, y in zip(ns, P['log_at'], strict=True)]
  tg, ss, matched = DR.plan_targets(frames, P, ref)
  assert matched == 4 and tg == pytest.approx([-0.5, -0.6, -0.7, -0.7]) and ss == [False] * 4
  same, _, _ = DR.plan_targets(frames, dict(P, at=ref['at']), ref)
  assert same == [f['target'] for f in frames]   # equal arms: bit-identical to the logged plan


def test_e3_grab_rule_is_relative_to_flag_off():
  rh = np.zeros(800, int)
  rh[50:200] = 1
  v = np.zeros(800)
  v[300:700] = 0.5
  on, off = _arm(800, rehold=rh), _arm(800)
  on['stopreq'][200:], off['stopreq'][200:] = 0, 0
  on['wire'][250:400] = off['wire'][250:400] = -0.6   # both arms grab after the re-hold: not E3's
  inp = dict(_inputs(800, v=v), gap=np.full(800, 5.0) + np.where(np.arange(800) > 210, 1.0, 0.0))
  assert not any('grab' in x for x in D.e3_checks(on, off, inp, live=True)[0]['violations'])
  on['wire'][250:400] = -0.9
  assert any('grab' in x for x in D.e3_checks(on, off, inp, live=True)[0]['violations'])


def test_markdown_renders_every_section_without_stops():
  rep = dict(route='00009999--abcdef0123', generated='now', segments=3, commit='56e1512892', dirty=False, engaged_s=0.0, stops=[],
             holds=[dict(t_start=1.0, t_end=5.0)], bookmark_matches=[dict(t=3.0, stop=None, hold=1.0, how='hold')], races=[], trials={},
             replay='no engaged driving', summary='s', scan_errors=[])
  md = D.markdown(rep)
  for head in ('## Stops', '## Bookmarks', '## Release-end races', '## Trial E3', '## Trial LINE', '## Trial BUNDLE', '## Trial LAUNCH'):
    assert head in md
  assert '3.00 -> hold 1.0 (hold)' in md


def test_rereport_only_when_rlogs_grow(tmp_path, monkeypatch):
  monkeypatch.setattr(D, 'WORK', tmp_path)
  monkeypatch.setattr(D, 'local_routes', lambda: ['00009999--aa', '0000999a--bb'])
  segs = {'00009999--aa': 5, '0000999a--bb': 3}
  monkeypatch.setattr(D, 'route_segments', lambda r: [f'{r}--{i}' for i in range(segs[r])])
  (tmp_path / 'scan').mkdir()
  for r, n in segs.items():   # stage A cached: no scan pool
    for i in range(n):
      (tmp_path / 'scan' / f'{r}--{i}.npz').touch()
  (tmp_path / 'state.json').write_text('{"00009999--aa": {"rlogs": 6}, "0000999a--bb": {"rlogs": 2}}')
  done = []
  monkeypatch.setattr(D, 'report_route', lambda r, log=print: done.append(r) or dict(summary=r, generated='now'))
  assert D.main([]) == 0
  assert done == ['0000999a--bb']   # 00009999 was pruned 6 -> 5 rlogs: its fuller report stays


def test_snapshot_override_derives_band_flag(tmp_path, monkeypatch):
  if DR.full_sha('56e1512892') is None:
    pytest.skip('trial build not in this clone')
  monkeypatch.setattr(DR, 'WORK', tmp_path)
  files, _, _ = DR.snapshot(DR.full_sha('56e1512892'), {LINE: False})
  ns: dict = {}
  exec(Path(files['openpilot.selfdrive.controls.lib.stopping_flags']).read_text(), ns)  # constants only
  assert ns[LINE] is False and ns['GOVERNOR_BAND_PROFILE'] is False and ns[E3] is True


# ---- Astra tooling review findings 7-10 ----------------------------------------------------------------------------------------
def test_cache_key_follows_the_replay_implementation(tmp_path, monkeypatch):
  """Finding 7: a replay / extraction code change (or an unpinned working-tree module) starts a new cache; the delta cache names both
  arms' keys."""
  head = DR.full_sha('HEAD')
  impl = tmp_path / 'impl.py'
  impl.write_text('A = 1\n')
  monkeypatch.setattr(DR, 'IMPL_FILES', [impl])
  k1 = D.cache_key(head, {})
  assert k1.startswith(DR.arm_key(head, {})) and D.cache_key(head, {}) == k1
  impl.write_text('A = 2\n')
  k2 = D.cache_key(head, {})
  assert k2 != k1 and D._cache_dir('delta', k2, 'ref') != D._cache_dir('delta', k1, 'ref')
  monkeypatch.setattr(DR, 'git', lambda *a: b'selfdrive/car/x.py\n' if a[0] == 'diff' else b'')   # an unpinned module differs ...
  monkeypatch.setattr(DR, 'REPO', tmp_path)
  (tmp_path / 'selfdrive/car').mkdir(parents=True)
  (tmp_path / 'selfdrive/car/x.py').write_text('B = 1\n')
  k3 = D.cache_key(head, {})
  (tmp_path / 'selfdrive/car/x.py').write_text('B = 2\n')   # ... and its working-tree content changes
  assert D.cache_key(head, {}) != k3


def test_interrupted_approach_and_takeover_get_trial_windows(tmp_path, monkeypatch):
  """Finding 8: an approach disengaged 2 s before its stop is not a census stop, but its span is replayed by the trials."""
  v = _stop_trace()
  la = np.ones(len(v))
  la[1300:] = 0.0   # disengaged at 13 s, wheel stop at 15 s
  R = _route(v, long_active=la)
  assert D.engaged_stops(R) == []
  rescued = D.engaged_stops(R, interrupted=True)
  assert [s['kind'] for s in rescued] == ['stop']
  brake = np.zeros(len(v))
  brake[1300:1400] = 1.0
  R2 = _route(v, long_active=la, brake=brake)
  assert D.takeovers(R2) == [13.0]
  monkeypatch.setattr(D, 'REPORTS', tmp_path)
  monkeypatch.setattr(D, 'load_route', lambda r: dict(R, logs=[], bookmarks=[], commits=['56e1512892'], dirty=False, errors=[], segments=['s']))
  monkeypatch.setattr(DR, 'full_sha', lambda c: 'f' * 40)
  monkeypatch.setattr(DR, 'flag_value', lambda c, f: True)
  monkeypatch.setattr(DR, 'snapshot', lambda c, o: ({}, {}, []))
  monkeypatch.setattr(D, 'run_arm', lambda *a, **k: {})
  got = {}

  def trial(flag, T, base, live, fv, spans, *a):
    got[flag] = spans
    return dict(flag=flag, name=T['name'], mode='live', base=base[:10], in_build=True, spans_failed=[], checks=[], stops=[], trips=[], events=[],
                plan_fidelity=[], revert=[], verdict='PASS')
  monkeypatch.setattr(D, 'trial_line', trial)
  monkeypatch.setattr(D, 'trial_e3', trial)
  rep = D.report_route('00009999--abcdef0123')
  assert rep['stops'] == [] and rep['interrupted'] == [dict(kind='stop', t_ws=rescued[0]['t_ws'])]
  assert all(len(s) == 1 and s[0]['lo'] <= 8.0 <= s[0]['hi'] for s in got.values())   # the R1 event at 8 s is inside a trial span


def test_failed_trial_replay_is_incomplete_everywhere(tmp_path, monkeypatch):
  """Finding 9: a span whose replay failed is never a PASS; the route is not marked reported, the exit status says so."""
  spans = [dict(span='sp', route='r', lo=0.0, hi=3.0)]
  ok = _arm()
  monkeypatch.setattr(D, 'run_arm', lambda *a, **k: {'sp': 'ERROR missing rlog'})
  tr = D.trial_e3(E3, D.TRIALS[E3], 'f' * 40, True, True, spans, {'sp': ok}, print)
  assert tr['spans_failed'] == ['sp'] and tr['verdict'] == 'INCOMPLETE' and tr['revert'] == []
  md = '\n'.join(D.markdown_e3(dict(commit='x'), tr))
  assert 'INCOMPLETE' in md and 'PASS' not in md
  monkeypatch.setattr(D, 'WORK', tmp_path)
  monkeypatch.setattr(D, 'local_routes', lambda: ['00009999--aa'])
  monkeypatch.setattr(D, 'route_segments', lambda r: [f'{r}--0'])
  (tmp_path / 'scan').mkdir()
  (tmp_path / 'scan' / '00009999--aa--0.npz').touch()
  monkeypatch.setattr(D, 'report_route', lambda r, log=print: dict(summary=r, generated='now', incomplete=['E3: 1 spans without a replay']))
  assert D.main(['--routes', '00009999']) == 3
  assert not (tmp_path / 'state.json').exists()   # retried on the next run


def test_r10_chatter_is_clear_then_set_not_a_late_set_and_launch_clear():
  """Finding 10: ON sets StopReq late (18.5 s) and clears it at the normal launch (19 s): one episode, no R10."""
  P, Pf, Ln, Lf, inp, stop = _line_case()
  tl = Ln['t']
  Ln['stopreq'] = np.where((tl >= 18.5) & (tl < 19.0), 1, 0)
  ev = D.line_span_events(P, Pf, Ln, Lf, inp, 'sp')
  assert not [e for e in ev if e['rule'] == 'R10']


# ---- the defect bundle (C2 C3 C4 SR C6; cycle_20261009/PLAN.md section 31) -----------------------------------------------------
def _barm(n=600, **cols):
  return _arm(n, **dict(dict(owning=np.zeros(n, int), target=np.zeros(n)), **cols))


def _car(n=600, **cols):
  c = dict(t=np.arange(n) * 0.01, v=np.zeros(n), vw=np.zeros(n), gas=np.zeros(n, int), brake=np.zeros(n, int), act=np.ones(n, int),
           lead=np.ones(n, int), gap=np.full(n, 5.0), vl=np.zeros(n), stopreq=np.zeros(n, int), lcs=np.full(n, 2), wire=np.full(n, -0.7),
           fault=None)
  c.update(cols)
  return c


def _verdicts(rows, flag, rule):
  return [r['verdict'] for r in rows if r['flag'] == flag and r['rule'] == rule]


def _c4_hold(n=600):
  """A service hold at rest: on enters stopping (StopReq 0.5 s after the vEgo gate at 1.0 s), off stays in pid without StopReq."""
  sr = (np.arange(n) >= 150).astype(int)
  on, off = _barm(n, stopreq=sr), _barm(n, lcs=np.full(n, 1), stopreq=np.zeros(n, int))
  car = _car(n, v=np.where(np.arange(n) < 100, 0.05, 0.0), stopreq=sr.copy())
  return on, off, car


def test_bundle_c4_hold_is_attributed_and_passes():
  on, off, car = _c4_hold()
  ev, rows = D.bundle_checks(on, off, car, live=True)
  assert [e['flags'] for e in ev] == [['C4']]
  assert _verdicts(rows, 'C4', 'StopReq') == ['PASS'] and _verdicts(rows, 'SR', 'StopReq set') == ['PASS']
  assert set(D.bundle_decide(rows).values()) == {'PASS'}


def test_bundle_c4_starting_under_hold_reverts_and_pid_hold_looks():
  on, off, car = _c4_hold()
  car['lcs'][300:302] = 3            # 'starting' at rest behind a stopped lead, flag off not
  car['lcs'][400:450] = 1            # 0.49 s in pid under the hold
  _, rows = D.bundle_checks(on, off, car, live=True)
  assert _verdicts(rows, 'C4', "'starting' under hold") == ['REVERT'] and _verdicts(rows, 'C4', 'pid hold') == ['LOOK']
  assert D.bundle_decide(rows)['C4'] == 'REVERT'
  car['vl'][:] = 1.0                 # behind a moving lead the 'starting' frame is a LOOK, not a revert
  _, rows = D.bundle_checks(on, off, car, live=True)
  assert _verdicts(rows, 'C4', "'starting' under hold") == ['LOOK']


def test_bundle_slower_launch_after_a_c4_hold_reverts_c4():
  """Live: the LOGGED command and sent StopReq are the car's launch, then its first wheel motion (review finding 4); the replay arms
  alone prove nothing about the car."""
  n = 600
  i = np.arange(n)
  on, off, car = _c4_hold(n)
  for arm in (on, off):
    arm['phase'] = np.where(i < 200, 4, 0)
    arm['wire'] = np.where(i < 200, -0.7, 0.5)
    arm['stopreq'] = np.where((i >= 150) & (i < 200), 1, 0)
  car['wire'] = np.where(i < 230, -0.7, 0.5)                    # the car's command launches 0.3 s after flag off's
  car['stopreq'] = np.where((i >= 150) & (i < 230), 1, 0)
  car['v'] = car['vw'] = np.where(i < 260, 0.0, 0.5)
  _, rows = D.bundle_checks(on, off, car, live=True)
  assert _verdicts(rows, 'C4', 'launch') == ['REVERT']
  car['wire'], car['stopreq'] = off['wire'].copy(), off['stopreq'].copy()   # the car launches with flag off: PASS ...
  _, rows = D.bundle_checks(on, off, car, live=True)
  assert _verdicts(rows, 'C4', 'launch') == ['PASS'] and _verdicts(rows, 'C4', 'first motion') == []
  car['v'] = car['vw'] = np.where(i < 350, 0.0, 0.5)            # ... the wheels move 1.5 s after it: no flag-off reference, LOOK
  _, rows = D.bundle_checks(on, off, car, live=True)
  assert _verdicts(rows, 'C4', 'first motion') == ['LOOK']


def _release(n=800, go_at=None, rehold_at=None):
  """on releases its HOLD at 2.0 s (off holds on); optional planner go pulse before it and a re-hold."""
  i = np.arange(n)
  ph = np.where(i < 200, 4, np.where(i < 300, 5, 0))
  if rehold_at is not None:
    ph[rehold_at:] = 4
  tg = np.full(n, 0.1)
  if go_at is not None:
    tg[go_at:go_at + 10] = 0.3
  rel = (i >= 200) & (ph != 4)
  on = _barm(n, phase=ph, target=tg, wire=np.where(rel, 0.3, -0.7), stopreq=np.where(i < 200, 1, 0), lcs=np.where(rel, 1, 2))
  return on, _barm(n), _car(n, vl=np.full(n, 0.8), lcs=on['lcs'].copy(), stopreq=on['stopreq'].copy())


def test_bundle_release_cause_c2_by_the_remembered_go_else_c6():
  on, off, car = _release(go_at=170)
  ev, rows = D.bundle_checks(on, off, car, live=True)
  assert ev[0]['flags'] == ['C2'] and _verdicts(rows, 'C2', 'release') == ['PASS']   # the lead keeps moving
  on, off, car = _release()
  ev, rows = D.bundle_checks(on, off, car, live=True)
  assert ev[0]['flags'] == ['C6'] and _verdicts(rows, 'C6', 'release') == ['PASS']
  assert _verdicts(rows, 'C6', 'no motion') == ['LOOK']   # live: no wheel motion within 2 s of a C6 release


def test_bundle_c6_rehold_toward_a_stopping_lead_with_motion_reverts():
  on, off, car = _release(rehold_at=300)
  car['vl'][250:] = 0.0              # the lead stops again for > 0.5 s
  car['v'][200:275] = car['vw'][200:275] = 0.4   # 0.3 m of motion before the re-hold
  _, rows = D.bundle_checks(on, off, car, live=True)
  assert _verdicts(rows, 'C6', 'lead stops again') == ['REVERT']
  _, rows = D.bundle_checks(on, off, car, live=False)   # counterfactual: the logged motion is the flag-off world
  assert _verdicts(rows, 'C6', 'lead stops again') == ['LOOK']
  car['v'][:] = car['vw'][:] = 0.0   # no motion: the accepted residual (PLAN 30, 1) -- LOOK once, REVERT when it repeats
  _, rows = D.bundle_checks(on, off, car, live=True)
  assert _verdicts(rows, 'C6', 'lead stops again') == ['LOOK'] and D.bundle_decide(rows)['C6'] == 'LOOK'
  assert D.bundle_decide(rows + rows)['C6'] == 'REVERT'


def test_bundle_release_toward_a_stopping_lead_reverts_without_a_rehold():
  """Review finding 2: the motion limits (C2 > 0.3 m, C6 > 0.1 m) apply whether or not the service re-holds; the motion runs from
  rest until the car rests again."""
  for f, go, mps in (('C2', 170, 0.25), ('C2', 170, 0.4), ('C6', None, 0.15)):
    on, off, car = _release(go_at=go)
    car['vl'][250:] = 0.0
    car['v'][200:300] = car['vw'][200:300] = mps   # mps metres in 1 s, then at rest; no re-hold
    _, rows = D.bundle_checks(on, off, car, live=True)
    assert _verdicts(rows, f, 'lead stops again') == (['REVERT'] if mps > D.BR['moved'][f] else ['LOOK']), (f, mps)


def test_bundle_driver_gas_after_a_c6_release_and_the_pedal_ends_its_window():
  on, off, car = _release(rehold_at=450)
  car['gas'][250:] = 1               # gas 0.5 s after the release; the re-hold at 4.5 s is the driver's, not the release's
  _, rows = D.bundle_checks(on, off, car, live=True)
  assert _verdicts(rows, 'C6', 'gas') == ['LOOK'] and _verdicts(rows, 'C6', 're-hold') == []
  assert _verdicts(rows, 'C6', 'no motion') == ['LOOK']   # the car did not move before the driver had to press the gas


def test_bundle_starting_at_rest_after_a_c6_release():
  on, off, car = _release()
  car['lcs'][220:230] = 3
  _, rows = D.bundle_checks(on, off, car, live=True)
  assert _verdicts(rows, 'C6', "'starting' after release") == ['LOOK']   # residual 3: LOOK unless it repeats or creeps
  car['lcs'][220:260] = 3            # a 'starting' episode from rest that accelerates out of the rest band (review finding 1)
  car['v'][220:260] = car['vw'][220:260] = np.linspace(0.0, 0.8, 40)   # 0.16 m
  _, rows = D.bundle_checks(on, off, car, live=True)
  assert _verdicts(rows, 'C6', "'starting' after release") == ['REVERT']


def test_bundle_gap_floor_only_where_flag_off_stays_above():
  on, off, car = _release()
  i = np.arange(800)
  car['vw'] = np.where((i >= 200) & (i < 300), 0.5, 0.0)            # 0.5 m toward the lead after the release
  car['gap'] = 3.3 - np.r_[0.0, np.cumsum(car['vw'][1:] * 0.01)]
  _, rows = D.bundle_checks(on, off, car, live=True)
  assert _verdicts(rows, 'C6', 'gap floor') == ['REVERT']
  car['gap'] = car['gap'] - 0.5     # flag off would be under 3.0 too: not the bundle's
  _, rows = D.bundle_checks(on, off, car, live=True)
  assert _verdicts(rows, 'C6', 'gap floor') == []


def test_bundle_c3_owned_transit_step():
  n = 300
  i = np.arange(n)
  lcs = np.where(i == 100, 3, 1)
  on = _barm(n, phase=np.full(n, 1), lcs=lcs, owning=np.ones(n, int), wire=np.full(n, -0.8))
  off = _barm(n, phase=np.full(n, 1), lcs=lcs, owning=np.where(i < 100, 1, 0), wire=np.where(i == 100, 0.7, -0.8))
  car = _car(n, v=np.full(n, 1.5), lcs=lcs, wire=on['wire'].copy())
  ev, rows = D.bundle_checks(on, off, car, live=True)
  assert [e['flags'] for e in ev] == [['C3']] and _verdicts(rows, 'C3', 'owned transit') == ['PASS']
  car['wire'][100] = 0.7             # a step above 0.125 reverts even when flag off steps the same (section 31; review finding 5)
  _, rows = D.bundle_checks(on, off, car, live=True)
  assert _verdicts(rows, 'C3', 'owned transit') == ['REVERT']
  car['gas'][100:] = 1               # the driver's gas override on the transit frame is not C3's
  _, rows = D.bundle_checks(on, off, car, live=True)
  assert _verdicts(rows, 'C3', 'owned transit') == []


def test_bundle_stopreq_settled_set_and_roll():
  n = 500
  i = np.arange(n)
  on = off = _barm(n, phase=np.zeros(n, int), stopreq=np.zeros(n, int))
  rest = np.where(i < 50, 0.05, 0.0)
  car = _car(n, v=rest.copy())       # stopping, at the vEgo gate from 0.5 s
  _, rows = D.bundle_checks(on, off, car, live=True)
  assert _verdicts(rows, 'SR', 'StopReq set') == ['LOOK']
  car['v'][100:] = car['vw'][100:] = 0.2   # the unset hold rolls out of the rest band: measured to its end (review finding 1)
  _, rows = D.bundle_checks(on, off, car, live=True)
  assert _verdicts(rows, 'SR', 'StopReq set') == ['REVERT']
  car['v'], car['vw'] = rest.copy(), np.zeros(n)
  car['stopreq'] = (i >= 150).astype(int)   # set 1.0 s after settling
  _, rows = D.bundle_checks(on, off, car, live=True)
  assert _verdicts(rows, 'SR', 'StopReq set') == ['PASS']


def test_bundle_acc_fault_in_a_c4_hold_reverts_c4():
  on, off, car = _c4_hold()
  car['fault'] = (np.arange(600) >= 300).astype(float)
  _, rows = D.bundle_checks(on, off, car, live=True)
  assert _verdicts(rows, 'C4', 'ACC fault') == ['REVERT']
  car['fault'] = np.ones(600)        # active from the span start: onset unknown, LOOK, never a fault-free PASS (review finding 9)
  _, rows = D.bundle_checks(on, off, car, live=True)
  assert _verdicts(rows, 'C4', 'ACC fault') == ['LOOK'] and D.bundle_decide(rows)['C4'] == 'LOOK'


def test_bundle_release_cause_at_the_go_memory_boundary_and_enabled_flags():
  """Review finding 6: a go 0.5 s before the release is C2's as in production (2.2 - 0.5 = 1.7000000000000002 must keep the go at
  1.70); a cause the build does not run is unresolved ('?', LOOK)."""
  n = 70
  i = np.arange(n)
  t = 1.7 + i * 0.01
  on = _barm(n, t=t, phase=np.where(i < 50, 4, 5), target=np.where(i == 0, 0.27, 0.1), lcs=np.where(i < 50, 2, 1),
             stopreq=np.where(i < 50, 1, 0))
  car = _car(n, t=t, vl=np.where(i >= 50, 0.8, 0.0), lcs=on['lcs'].copy(), stopreq=on['stopreq'].copy())
  ev, _ = D.bundle_checks(on, _barm(n, t=t), car, live=True)
  assert ev[0]['flags'] == ['C2']
  ev, rows = D.bundle_checks(on, _barm(n, t=t), car, live=True, enabled=('C3', 'C4', 'SR', 'C6'))
  assert ev[0]['flags'] == ['?'] and _verdicts(rows, '?', 'attribution') == ['LOOK']


def test_bundle_counterfactual_gap_is_an_open_loop_estimate():
  """Review finding 7: on a counterfactual drive the log is the flag-off car; a flag-on gap under 3.0 m is an estimate (LOOK)."""
  on, off, car = _release()
  car['gap'][:] = 3.1
  _, rows = D.bundle_checks(on, off, car, live=False)
  assert _verdicts(rows, 'C6', 'gap floor') == ['LOOK']


def test_bundle_driver_intervention_ends_the_causal_windows():
  """Review finding 8: gas, brake or loss of longitudinal control ends the gap and launch windows; the takeover stays LOOK."""
  on, off, car = _release()
  car['gas'][210:] = 1
  car['v'][220:] = car['vw'][220:] = 0.2   # manual travel toward the lead after the gas
  car['gap'] = 3.32 - np.r_[0.0, np.cumsum(car['vw'][1:] * 0.01)]
  _, rows = D.bundle_checks(on, off, car, live=True)
  assert _verdicts(rows, 'C6', 'gap floor') == [] and _verdicts(rows, 'C6', 'takeover') == ['LOOK']
  n = 600
  i = np.arange(n)
  on, off, car = _c4_hold(n)
  for arm in (on, off):
    arm['phase'] = np.where(i < 200, 4, 0)
    arm['stopreq'] = np.where((i >= 150) & (i < 200), 1, 0)
  on['wire'], off['wire'] = np.where(i < 230, -0.7, 0.5), np.where(i < 200, -0.7, 0.5)
  car['brake'][180:190] = 1
  car['act'][180:200] = 0
  _, rows = D.bundle_checks(on, off, car, live=True)
  assert _verdicts(rows, 'C4', 'launch') == [] and D.bundle_decide(rows)['C4'] == 'LOOK'


def test_bundle_windows_cover_rolling_transits_and_faults():
  """Review finding 10: a rolling 'starting' transit and an engaged fault onset outside any stop or hold get replay windows."""
  R = _route(np.r_[np.full(500, 5.0), np.linspace(5.0, 1.3, 200), np.linspace(1.3, 5.0, 200)])
  R['ctl'][700, 1] = 3
  assert D.engaged_stops(R) == [] and D.holds(R) == [] and D.takeovers(R) == []
  R['car'] = np.column_stack([R['cs'][:, 0], (R['cs'][:, 0] >= 3.0).astype(float)])
  assert D.bundle_windows(R) == pytest.approx([3.0, 7.0])
  sp = D.replay_spans('00009999--abcdef0123', R, [], [], D.bundle_windows(R))
  assert len(sp) == 1 and sp[0]['lo'] <= 3.0 and sp[0]['hi'] >= 7.0


def test_trial_bundle_live_off_arm_is_the_builds_flags_off(monkeypatch):
  """After a revert (C6 False in the build) the bundle stays live for the other four; the off arm sets only those False."""
  seen = []
  on, off, car = _c4_hold()
  monkeypatch.setattr(D, 'run_arm', lambda c, o, *a, **k: seen.append(o) or {'sp': off})
  monkeypatch.setattr(DR, 'frames_for', lambda s: None)
  monkeypatch.setattr(DR, 'inputs', lambda d: {k: car[k] for k in ('t', 'v', 'gas', 'brake', 'act', 'lead', 'gap', 'vl')} | dict(rec=car['wire']))
  t = car['t']
  R = dict(cs=np.column_stack([t, car['v'], *[np.zeros(len(t))] * 4, car['vw']]), scc12=np.column_stack([t, car['wire'], car['stopreq'], t * 0]),
           ctl=np.column_stack([t, car['lcs']]), car=np.column_stack([t, t * 0]))
  fv = {f: f != 'HOLD_DEPARTURE_RELEASE' for f in D.BUNDLE.values()}
  T = D.TRIALS['C2_C3_C4_SR_C6']
  tr = D.trial_bundle('C2_C3_C4_SR_C6', T, 'f' * 40, True, fv, [dict(span='sp')], {'sp': on}, R, print)
  assert seen == [{f: False for f in fv if fv[f]}] and 'HOLD_DEPARTURE_RELEASE' not in tr['off_flags']
  assert tr['missing'] == [] and tr['verdict'] == 'PASS' and [e['flags'] for e in tr['events']] == [['C4']]
  md = '\n'.join(D.markdown_bundle(dict(commit='x'), tr))
  assert '| C4 SERVICE_HOLD_ENTERS_STOPPING | 1 |' in md
  for k in ('scc12', 'ctl', 'car'):  # review finding 3: a live drive without the logged streams is INCOMPLETE, never a replay PASS
    R.pop(k)
  tr = D.trial_bundle('C2_C3_C4_SR_C6', T, 'f' * 40, True, fv, [dict(span='sp')], {'sp': on}, R, print)
  assert tr['missing'] == ['stopreq', 'lcs', 'fault'] and tr['verdict'] == 'INCOMPLETE'
  assert tr['decision']['C4'] == tr['decision']['SR'] == 'LOOK' and not any(r['verdict'] == 'PASS' and r['flag'] == 'C4' for r in tr['checks'])


# ---- adversarial verification of the bundle check (verify_d/RESULT.json, confirmed findings 0-12) -------------------------------
def test_bundle_lead_stop_is_observed_past_the_driver_and_the_window_end():
  """Findings 0, 4, 5: the lead's stop is a fact about the lead. A driver brake (or a pedal-free disengagement) 0.4 s after the lead
  stopped again does not hide the autonomous motion toward it (C2 > 0.3 m, C6 > 0.1 m reverts), and a stop that begins in the last
  0.5 s of the window (C2 2 s, C6 3 s) and lasts is seen, with or without a re-hold after it (later than 1.5 s: the queue creep LOOK)."""
  for f, go, mps in (('C6', None, 0.15), ('C2', 170, 0.4)):
    for brake in (1, 0):
      on, off, car = _release(go_at=go)
      car['vl'][250:] = 0.0                            # the lead stops again 0.5 s after the release
      car['v'][200:300] = car['vw'][200:300] = mps     # the car moves toward it ...
      car['brake'][290:], car['act'][290:] = brake, 0  # ... until the driver brakes / longitudinal control drops at 2.9 s
      _, rows = D.bundle_checks(on, off, car, live=True)
      assert _verdicts(rows, f, 'lead stops again') == ['REVERT'], (f, brake)
  for f, go, stop_at in (('C2', 170, 360), ('C6', None, 460)):
    for rehold in (None, stop_at + 10):
      on, off, car = _release(900, go_at=go, rehold_at=rehold)
      car['vl'][stop_at:] = 0.0                        # the lead stops 1.6 s (C2) / 2.6 s (C6) after the release and stays stopped
      car['v'][200:stop_at + 30] = car['vw'][200:stop_at + 30] = 0.4
      car['v'][stop_at + 30:stop_at + 60] = car['vw'][stop_at + 30:stop_at + 60] = np.linspace(0.4, 0.0, 30)   # stops at 1.3 m/s^2
      _, rows = D.bundle_checks(on, off, car, live=True)
      assert _verdicts(rows, f, 'lead stops later') == ['LOOK'] and _verdicts(rows, f, 'release') == [], (f, rehold)


def test_bundle_sr_judges_every_unset_episode_and_its_roll():
  """Findings 1, 6: an unset hold that rolls > 0.1 m reverts SR even when StopReq then comes within 1.5 s; a command rise >= 0.3 at rest
  (a release that re-holds inside one 'stopping' run) ends one episode, not the check of the rest of the run."""
  n = 600
  i = np.arange(n)
  t = i * 0.01
  arm = _barm(n, phase=np.zeros(n, int), stopreq=np.zeros(n, int))
  v = np.where((i >= 50) & (i < 120), 0.3, 0.0)      # settled at 0 s, rolls 0.21 m at 0.5-1.2 s, rests again
  _, rows = D.bundle_checks(arm, arm, _car(n, v=v.copy(), vw=v.copy(), stopreq=(i >= 140).astype(int)), live=True)
  assert _verdicts(rows, 'SR', 'StopReq set') == ['REVERT']   # StopReq 1.40 s after settling
  w = np.interp(t, [0, 0.4, 0.9, 1.9, 6], [-0.7, -0.7, -0.1, -0.7, -0.7])   # a release ramp at rest that re-holds (0.3 rise at 0.75 s)
  v = np.where(t < 0.5, 0.012, np.where((t >= 1.9) & (t < 5.0), 0.04, np.where(t < 5.0, 0.008, 0.0)))   # gate 0.5 s, crawl 1.9-5.0 s
  sr = (t >= 5.1).astype(int)                         # flag on sets after the crawl; flag off at the gate
  on, off = _barm(n, phase=np.zeros(n, int), stopreq=sr, wire=w), _barm(n, phase=np.zeros(n, int), stopreq=(t >= 0.5).astype(int), wire=w)
  _, rows = D.bundle_checks(on, off, _car(n, v=v, vw=np.where(v > 0.01, v, 0.0), stopreq=sr.copy(), wire=w.copy()), live=True)
  assert _verdicts(rows, 'SR', 'StopReq set') == ['REVERT']   # the unset hold crawled 0.12 m after the re-hold


def test_bundle_launch_cut_by_the_driver_within_late_s_is_not_slower():
  """Findings 2, 8, 10: the logged StopReq clears 1-2 frames after the logged command rises (sendcan follows carControl); a driver input
  on those frames cuts the launch window before the logged go: the on side is unseen but at most late_s behind, so LOOK (never REVERT);
  a StopReq still set when the driver acts 0.6 s later reverts C4."""
  n = 600
  i = np.arange(n)
  for lag, pedal, k, want in ((1, 'gas', 201, 'LOOK'), (2, 'gas', 201, 'LOOK'), (2, 'brake', 202, 'LOOK'), (n, 'gas', 260, 'REVERT')):
    on, off, car = _c4_hold(n)
    for arm in (on, off):
      arm['phase'] = np.where(i < 200, 4, 0)
      arm['wire'] = np.where(i < 200, -0.7, 0.5)
      arm['stopreq'] = np.where((i >= 150) & (i < 200), 1, 0)
    car['wire'] = off['wire'].copy()                   # the car's command rises with flag off's at 2.00 s
    car['stopreq'] = np.where((i >= 150) & (i < 200 + lag), 1, 0)
    car[pedal][k:] = 1
    car['v'] = car['vw'] = np.where(i < 240, 0.0, 0.5)
    _, rows = D.bundle_checks(on, off, car, live=True)
    assert _verdicts(rows, 'C4', 'launch') == [want], (lag, pedal)


def test_bundle_gap_floor_goes_to_the_mover_and_today_is_the_delayed_release():
  """Findings 7, 9: the gap floor is charged to the flag whose command moved the car (a C6 release out of a C4 hold reverts C6, not C4;
  the flag-off arm's 'starting' frame after a C2 release is not C3's); when flag off releases the same hold later, today's car is the
  logged car delayed by that much (a cut-in today's car would also have under 3.0 m is not the bundle's)."""
  n = 900
  t = np.arange(n) * 0.01

  def arm(t_rel, start=0):   # a HOLD released at t_rel (ramp -0.7 -> +0.5 at 1.2 m/s^3), 'starting' frames, then pid
    lcs = np.where(t < t_rel, 2, 1)
    lcs[int(round(t_rel * 100)):int(round(t_rel * 100)) + start] = 3
    return _barm(n, phase=np.where(t < t_rel, 4, np.where(t < t_rel + 1.0, 5, 0)), wire=np.where(t < t_rel, -0.7, np.minimum(-0.7 + 1.2 * (t - t_rel), 0.5)),
                 lcs=lcs, stopreq=np.where(t < t_rel, 1, 0), target=np.where((t >= 1.7) & (t < 1.8), 0.3, 0.1))

  def drive(on, off, t_rel, t_cut, g_cut):   # the car follows the on command, the lead pulls away 0.5 s before; a cut-in at g_cut m
    v = np.r_[0.0, np.cumsum(np.maximum(on['wire'][1:], 0.0) * 0.01)]
    gap = 6.0 + np.maximum(t - t_rel + 0.5, 0.0) - np.r_[0.0, np.cumsum(v[1:] * 0.01)]
    gap[(t >= t_cut) & (t < t_cut + 0.5)] = g_cut
    return D.bundle_checks(on, off, _car(n, v=v, vw=np.where(v > 0.104, v, 0.0), vl=np.where(t >= t_rel - 0.5, 1.0, 0.0), gap=gap,
                                         lcs=on['lcs'].copy(), stopreq=on['stopreq'].copy(), wire=on['wire'].copy()), live=True)
  ev, rows = drive(arm(5.0), _barm(n, lcs=np.ones(n, int), stopreq=np.zeros(n, int)), 5.0, 7.5, 2.8)   # C4 hold, C6 release, one event
  assert [e['flags'] for e in ev] == [['C4', 'C6']] and _verdicts(rows, 'C6', 'gap floor') == ['REVERT']
  assert _verdicts(rows, 'C4', 'gap floor') == [] and D.bundle_decide(rows)['C4'] == 'PASS'
  ev, rows = drive(arm(2.0, 30), arm(3.2, 1), 2.0, 4.5, 2.6)   # flag off releases at the next go 1.2 s later: one 'starting' frame at v 0.2
  assert [e['flags'] for e in ev] == [['C2']] and _verdicts(rows, 'C2', 'gap floor') == ['REVERT'] and _verdicts(rows, 'C3', 'gap floor') == []
  for g_cut, want in ((2.6, []), (2.9, ['REVERT'])):          # flag off 0.3 s later: today's car 2.84 m / 3.13 m from the cut-in
    _, rows = drive(arm(2.0), arm(2.3), 2.0, 4.5, g_cut)
    assert _verdicts(rows, 'C2', 'gap floor') == want, g_cut


def test_trial_bundle_a_stream_must_cover_each_span(monkeypatch):
  """Findings 3, 12: a stream logged elsewhere in the route but not over a span (a time gap read stale, or a segment scanned before
  accFaulted existed) is missing there: its rules are LOOK and the drive INCOMPLETE, never a PASS."""
  on, off, car = _c4_hold()
  monkeypatch.setattr(D, 'run_arm', lambda *a, **k: {'sp': off})
  monkeypatch.setattr(DR, 'frames_for', lambda s: None)
  monkeypatch.setattr(DR, 'inputs', lambda d: {k: car[k] for k in ('t', 'v', 'gas', 'brake', 'act', 'lead', 'gap', 'vl')} | dict(rec=car['wire']))
  t = car['t']
  sc = np.column_stack([t, car['wire'], car['stopreq'], t * 0])
  stale = np.column_stack([t, car['wire'], np.ones(len(t)), t * 0])[:50]   # scc12 rows for the first 0.5 s only, StopReq 1 carried on
  for m, scc12, ca in (('stopreq', stale, np.column_stack([t, t * 0])), ('fault', sc, np.column_stack([t + 100.0, t * 0]))):
    R = dict(cs=np.column_stack([t, car['v'], *[t * 0] * 4, car['vw']]), scc12=scc12, ctl=np.column_stack([t, car['lcs']]), car=ca)
    tr = D.trial_bundle('C2_C3_C4_SR_C6', D.TRIALS['C2_C3_C4_SR_C6'], 'f' * 40, True, dict.fromkeys(D.BUNDLE.values(), True),
                        [dict(span='sp')], {'sp': on}, R, print)
    assert tr['verdict'] == 'INCOMPLETE' and tr['decision']['C4'] == tr['decision']['SR'] == 'LOOK', m
    assert tr['missing'] == [] and tr['missing_spans'] == [dict(span='sp', streams=[m])]


def test_qlog_only_segments_keep_the_route_incomplete(tmp_path, monkeypatch, capsys):
  """Finding 11: a segment synced as a qlog only runs no rule; the report names it and the route is not marked reported; a route with
  qlogs only is named instead of silence."""
  route = '00009999--abcdef0123'
  for seg, f in ((f'{route}--0', 'rlog.zst'), (f'{route}--1', 'qlog.zst'), ('0000999a--bb--0', 'qlog.zst'), ('scan', f'{route}--0.npz')):
    (tmp_path / seg).mkdir(exist_ok=True)
    (tmp_path / seg / f).touch()
  monkeypatch.setattr(DR, 'RD', tmp_path)
  monkeypatch.setattr(D, 'WORK', tmp_path)
  monkeypatch.setattr(D, 'REPORTS', tmp_path / 'reports')
  monkeypatch.setattr(DR, 'full_sha', lambda c: 'f' * 40)
  R = _route(np.full(500, 5.0))   # engaged, no stop, hold or takeover: no replay span, nothing else incomplete
  monkeypatch.setattr(D, 'load_route', lambda r: dict(R, bookmarks=[], commits=['56e1512892'], dirty=False, errors=[], segments=[f'{route}--0']))
  rep = D.report_route(route)
  assert rep['summary'].startswith('INCOMPLETE') and f'{route}--1' in rep['incomplete'][0]
  assert D.main(['--routes', '00009999']) == 3 and not (tmp_path / 'state.json').exists()
  (tmp_path / 'state.json').write_text('{"00009999--abcdef0123": {"rlogs": 1}}')
  capsys.readouterr()
  assert D.main([]) == 0 and '0000999a--bb' in capsys.readouterr().out


# ---- recheck of those fixes (verify_d/RECHECK.json, problems 0-18) ----------------------------------------------------------------
def test_bundle_lead_stop_reads_radar_latency_and_the_driver_onset():
  """RECHECK 0, 1, 17: radard adds the current vEgo to a 0.15 s old relative speed, so braking behind a creeping lead reads vLead < 0.3
  for > 0.5 s: no 'lead stops again', autonomous or driver brake. A stop counts only if it begins before the driver's first input
  + 0.3 s: one that begins just before a brake reverts; a release the driver pre-empts with no motion is no re-hold residual (twice
  is LOOK) and keeps its 'no motion' row."""
  n = 800
  t = np.arange(n) * 0.01
  vt = np.interp(t, [0, 3.0, 3.5, 8], [0.8, 0.8, 0.45, 0.45])          # the lead creeps on at 0.45 m/s
  v = np.interp(t, [0, 2.4, 3.9, 4.45, 8], [0, 0, 1.5, 0.4, 0.4])      # the car follows it, braking at 2 m/s^2 from 3.9 s
  vl = np.interp(t - 0.15, t, vt) + v - np.interp(t - 0.15, t, v)       # the logged vLead
  for brake in (0, 1):
    on, off, car = _release(n)
    car.update(v=v.copy(), vw=v.copy(), vl=vl.copy())
    car['brake'][390:] = brake
    _, rows = D.bundle_checks(on, off, car, live=True)
    assert _verdicts(rows, 'C6', 'lead stops again') == [] and _verdicts(rows, 'C6', 'release') == ['PASS'], brake
  on, off, car = _release()
  car['vl'][290:] = 0.0                                                  # the lead stops 0.9 s after the release
  car['v'][200:300] = car['vw'][200:300] = 0.3                           # the car rolls 0.3 m toward it ...
  car['v'][300:330] = car['vw'][300:330] = np.linspace(0.3, 0.0, 30)
  car['brake'][300:] = 1                                                 # ... until the driver brakes 0.1 s after the stop
  _, rows = D.bundle_checks(on, off, car, live=True)
  assert _verdicts(rows, 'C6', 'lead stops again') == ['REVERT']
  on, off, car = _release()
  car['gas'][210:] = 1                                                   # the driver drives 0.1 s after the release, no motion before
  car['vl'][380:] = 0.0                                                  # the lead stops 1.8 s after the release
  _, rows = D.bundle_checks(on, off, car, live=True)
  assert _verdicts(rows, 'C6', 'lead stops again') == [] and _verdicts(rows, 'C6', 'no motion') == ['LOOK']
  assert D.bundle_decide(rows + rows)['C6'] == 'LOOK'


def test_bundle_c2_rehold_motion_counts_up_to_the_rehold():
  """RECHECK 2: C2's re-hold rule (motion > 0.3 m reverts) measures the travel up to the re-hold: a 0.3 s pre-stop blip before the car
  moved, then following the moving lead for metres, is the re-hold LOOK; a re-hold after 0.4 m of travel reverts."""
  for blip, start, want in ((250, 260, 'LOOK'), (300, 200, 'REVERT')):
    on, off, car = _release(go_at=170)
    on['phase'][blip:blip + 30] = D.PHASES.index('PRE_STOP_EASE')
    car['v'][start:] = car['vw'][start:] = 0.4
    _, rows = D.bundle_checks(on, off, car, live=True)
    assert _verdicts(rows, 'C2', 're-hold') == [want], blip


def test_bundle_release_flag_off_repeats_within_a_quarter_second_is_look():
  """RECHECK 3: a release the flag-off arm repeats < 0.25 s later is not the flag's effect (LOOK); 0.3 s later the motion toward a lead
  that stops again reverts C6."""
  for lag, want in ((1, 'LOOK'), (5, 'LOOK'), (30, 'REVERT')):
    on, off, car = _release()
    off.update({c: on[c].copy() for c in ('phase', 'target', 'wire', 'stopreq', 'lcs')})
    off['phase'][200:200 + lag], off['wire'][200:200 + lag], off['stopreq'][200:200 + lag], off['lcs'][200:200 + lag] = 4, -0.7, 1, 2
    car['vl'][250:] = 0.0
    car['v'][200:300] = car['vw'][200:300] = 0.15
    _, rows = D.bundle_checks(on, off, car, live=True)
    assert _verdicts(rows, 'C6', 'lead stops again') == [want], lag


def test_bundle_sr_skips_the_release_ramp_and_judges_the_hold_after_it():
  """RECHECK 6: no SR episode starts inside the on arm's RELEASE (a release's own creep is the release flag's); the hold after the
  re-hold is judged: StopReq 0.6 s after it is a PASS, an unset crawl of 0.12 m after it reverts."""
  n = 600
  t = np.arange(n) * 0.01
  w = np.interp(t, [0, 0.4, 0.9, 1.5, 2.0], [-0.7, -0.7, -0.1, -0.1, -0.7])   # a release ramp at rest (0.3 rise at 0.65 s), re-hold 2.0 s
  on = _barm(n, phase=np.where((t >= 0.4) & (t < 2.0), D.RELEASE, D.HOLD), wire=w, stopreq=np.zeros(n, int))
  off = _barm(n, stopreq=(t >= 0.3).astype(int))
  for v0, v1, mps, sr_at, want in ((0.75, 2.0, 0.09, 2.6, ['PASS']), (2.05, 5.0, 0.04, 5.1, ['REVERT'])):
    v = np.where(t < 0.2, 0.012, np.where((t >= v0) & (t < v1), mps, 0.0))   # at the vEgo gate from 0.2 s; a crawl under / after the release
    _, rows = D.bundle_checks(on, off, _car(n, v=v, vw=v.copy(), stopreq=(t >= sr_at).astype(int), wire=w.copy()), live=True)
    assert _verdicts(rows, 'SR', 'StopReq set') == want, v0


def test_bundle_gap_floor_mover_window_and_flag_off_travel():
  """RECHECK 8-11: a C4 hold (from 1.0 s, after a roll-in and an owned 'starting' frame that brakes 0.01 harder than flag off) merges
  with its C6 release (6.0 s) into one event. The gap floor goes to C6 only (a braking-side command difference moves no car), a gas
  tap during the hold does not end the release's window, flag off's travel counts from the release (not the shared roll-in), and a
  flag-off arm that leaves its hold only after the logged car moved (a replay of the on car) is today's car at rest."""
  n = 1200
  t = np.arange(n) * 0.01

  def drive(g_cut, tap=False, t_off=None):
    tr = (t >= 0.5) & (t < 0.65)
    on = _barm(n, phase=np.where(t < 1.0, 0, np.where(t < 6.0, D.HOLD, np.where(t < 7.0, D.RELEASE, 0))), owning=tr.astype(int),
               wire=np.where(t < 1.0, np.where(tr, -0.31, -0.3), np.where(t < 6.0, -0.7, np.minimum(-0.7 + 1.2 * (t - 6.0), 0.5))),
               lcs=np.where((t >= 0.6) & (t < 0.65), 3, np.where((t >= 1.0) & (t < 6.1), 2, 1)), stopreq=((t >= 1.2) & (t < 6.0)).astype(int),
               target=np.full(n, 0.05))
    off = _barm(n, phase=np.where(t < 1.0, 0, D.HOLD), wire=np.where(t < 1.0, -0.3, -0.7), lcs=np.ones(n, int), stopreq=np.zeros(n, int))
    if t_off is not None:   # flag off leaves its hold later with the same ramp
      off['phase'] = np.where(t < 1.0, 0, np.where(t < t_off, D.HOLD, np.where(t < t_off + 1.0, D.RELEASE, 0)))
      off['wire'] = np.where(t < 1.0, -0.3, np.where(t < t_off, -0.7, np.minimum(-0.7 + 1.2 * (t - t_off), 0.5)))
    v = np.r_[0.0, np.cumsum(np.maximum(on['wire'][1:], 0.0) * 0.01)] + np.interp(t, [0, 0.4, 1.0], [0.3, 0.3, 0.0])
    gap = 6.0 + np.maximum(t - 5.4, 0.0) - np.r_[0.0, np.cumsum(v[1:] * 0.01)] + np.r_[0.0, np.cumsum(v[1:] * 0.01)][100]
    gap[(t >= 8.5) & (t < 9.0)] = g_cut   # a car cuts in 2.5 s after the release (the on car has moved 0.73 m)
    car = _car(n, v=v, vw=v.copy(), vl=np.where(t >= 5.4, 1.0, 0.0), gap=gap, lcs=on['lcs'].copy(), stopreq=on['stopreq'].copy(),
               wire=on['wire'].copy())
    car['gas'][(t >= 3.0) & (t < 3.2)] = tap
    ev, rows = D.bundle_checks(on, off, car, live=True)
    return ev, [(r['flag'], r['verdict']) for r in rows if r['rule'] == 'gap floor']
  ev, gf = drive(2.5)
  assert [e['flags'] for e in ev] == [['C3', 'C4', 'C6']] and gf == [('C6', 'REVERT')]
  assert drive(2.5, tap=True)[1] == [('C6', 'REVERT')]
  assert drive(2.25)[1] == [] and drive(2.3)[1] == [('C6', 'REVERT')]   # today's car rests 2.98 / 3.03 m from the cut-in
  assert drive(2.4, t_off=6.81)[1] == [('C6', 'REVERT')]   # flag off leaves 0.1 s after the logged vEgo leaves rest: today's car 3.13 m


def test_trial_bundle_coverage_is_judged_on_engaged_frames(monkeypatch):
  """RECHECK 12: a stream that starts after the span start but before engagement (the SCC12 boot edge) covers the span; a gap inside
  the engaged frames does not."""
  on, off, car = _c4_hold()
  car['act'][:90] = 0
  monkeypatch.setattr(D, 'run_arm', lambda *a, **k: {'sp': off})
  monkeypatch.setattr(DR, 'frames_for', lambda s: None)
  monkeypatch.setattr(DR, 'inputs', lambda d: {k: car[k] for k in ('t', 'v', 'gas', 'brake', 'act', 'lead', 'gap', 'vl')} | dict(rec=car['wire']))
  t = car['t']
  sc = np.column_stack([t, car['wire'], car['stopreq'], t * 0])
  for rows, want, spans in (((t < 3.0) | (t >= 4.5), 'INCOMPLETE', [dict(span='sp', streams=['stopreq'])]), (t >= 0.8, 'PASS', [])):
    R = dict(cs=np.column_stack([t, car['v'], *[t * 0] * 4, car['vw']]), scc12=sc[rows], ctl=np.column_stack([t, car['lcs']]),
             car=np.column_stack([t, t * 0]))
    tr = D.trial_bundle('C2_C3_C4_SR_C6', D.TRIALS['C2_C3_C4_SR_C6'], 'f' * 40, True, dict.fromkeys(D.BUNDLE.values(), True),
                        [dict(span='sp')], {'sp': on}, R, print)
    assert tr['verdict'] == want and tr['missing_spans'] == spans, want


def test_absent_segments_keep_the_route_incomplete_and_index_has_one_line(tmp_path, monkeypatch):
  """RECHECK 14, 16: a segment number below the last rlog with neither an rlog nor a qlog keeps the route INCOMPLETE; INDEX.md keeps one
  line per route when an INCOMPLETE report is replaced."""
  route = '00009999--abcdef0123'
  for seg in (f'{route}--0', f'{route}--2'):
    (tmp_path / seg).mkdir()
    (tmp_path / seg / 'rlog.zst').touch()
  monkeypatch.setattr(DR, 'RD', tmp_path)
  monkeypatch.setattr(D, 'REPORTS', tmp_path / 'reports')
  R = _route(np.full(500, 5.0))
  monkeypatch.setattr(D, 'load_route', lambda r: dict(R, bookmarks=[], commits=['56e1512892'], dirty=False, errors=[],
                                                      segments=D.route_segments(r)))
  monkeypatch.setattr(DR, 'full_sha', lambda c: 'f' * 40)
  rep = D.report_route(route)
  assert rep['summary'].startswith('INCOMPLETE') and '[1]' in rep['incomplete'][0]
  (tmp_path / f'{route}--1').mkdir()
  (tmp_path / f'{route}--1' / 'rlog.zst').touch()
  rep = D.report_route(route)
  idx = (tmp_path / 'reports' / 'INDEX.md').read_text()
  assert not rep['summary'].startswith('INCOMPLETE') and [x for x in idx.splitlines() if route in x] == [f"- {rep['summary']}"]


# ---- orchestrator decisions after the recheck (re-hold phases, queue creep, SR after a re-hold) -------------------------------------
def test_bundle_glide_behind_a_moving_lead_is_no_rehold():
  """A glide after a release while the car follows the moving lead is normal following: no re-hold row (after 0.52 m it reverted C2);
  a pre-stop ease at the same frames is a re-hold and reverts."""
  for phase, rh, rel in (('APPROACH_GLIDE', [], ['PASS']), ('PRE_STOP_EASE', ['REVERT'], [])):
    on, off, car = _release(go_at=170)
    on['phase'][330:360] = D.PHASES.index(phase)
    car['v'][200:] = car['vw'][200:] = 0.4
    _, rows = D.bundle_checks(on, off, car, live=True)
    assert _verdicts(rows, 'C2', 're-hold') == rh and _verdicts(rows, 'C2', 'release') == rel, phase


def test_bundle_lead_stop_onset_splits_mis_release_from_queue_creep():
  """A lead stop that begins within 1.5 s of the release is the mis-release (car motion over the bar reverts; a car that stops hard
  splits the corrected reading, the onset is the uncorrected one); later is a queue creep: 'lead stops later' LOOK, no repeat class."""
  for f, go, stop_at, rule, want in (('C6', None, 340, 'lead stops again', 'REVERT'), ('C6', None, 460, 'lead stops later', 'LOOK'),
                                     ('C2', 170, 300, 'lead stops again', 'REVERT'), ('C2', 170, 380, 'lead stops later', 'LOOK')):
    on, off, car = _release(900, go_at=go)
    car['vl'][stop_at:] = 0.0
    car['v'][200:stop_at + 50] = car['vw'][200:stop_at + 50] = 0.4   # the car follows, then stops in one frame (a mis-release) ...
    if want == 'LOOK':   # ... or at 1.3 m/s^2
      car['v'][stop_at + 20:stop_at + 50] = car['vw'][stop_at + 20:stop_at + 50] = np.linspace(0.4, 0.0, 30)
    _, rows = D.bundle_checks(on, off, car, live=True)
    assert _verdicts(rows, f, rule) == [want] and len([r for r in rows if r['flag'] == f and r['rule'].startswith('lead')]) == 1, stop_at
    assert D.bundle_decide(rows + rows)[f] == want and not any(r.get('repeat') for r in rows if r['rule'] == 'lead stops later')


def test_bundle_sr_episode_starts_at_a_rehold_after_a_release():
  """RECHECK 5: an unset crawl at 0.04 m/s (never back under the vEgo gate) after a release that re-holds at rest reverts SR, also when
  the release command crosses 0 (the episode starts at the first non-positive re-hold command); the roll counts from the frame the
  re-hold command stops falling, so a creep under the falling command (the release's motion, wheel 0.15 m/s, vEgo lagging under 0.1)
  with StopReq 1.3 s later passes."""
  n = 600
  t = np.arange(n) * 0.01
  for hold, peak, v0, v1, mps, sr_at, want in ((-0.7, -0.1, 0.7, 5.0, 0.04, 5.1, 'REVERT'), (-0.45, 0.05, 0.7, 5.0, 0.04, 5.1, 'REVERT'),
                                              (-0.7, -0.1, 0.85, 1.85, 0.15, 2.2, 'PASS')):
    t_pk = 0.4 + (peak - hold) / 1.2
    w = np.interp(t, [0, 0.4, t_pk, t_pk + (peak - hold) / 0.6, 6], [hold, hold, peak, hold, hold])
    ph = np.where(t < 0.4, D.HOLD, np.where(t < t_pk, D.RELEASE, np.where(t < t_pk + (peak - hold) / 0.6, D.RAMP, D.HOLD)))
    on = _barm(n, phase=ph, wire=w, stopreq=(t >= sr_at).astype(int))
    off = _barm(n, phase=ph.copy(), wire=w.copy(), stopreq=(t >= 0.5).astype(int))
    v = np.where(t < 0.5, 0.012, np.where((t >= v0) & (t < v1), min(mps, 0.09), np.where(t < v1, 0.008, 0.0)))
    vw = np.where((t >= v0) & (t < v1), mps, 0.0)
    _, rows = D.bundle_checks(on, off, _car(n, v=v, vw=vw, stopreq=on['stopreq'].copy(), wire=w.copy()), live=True)
    assert D.bundle_decide(rows)['SR'] == want and want in _verdicts(rows, 'SR', 'StopReq set'), (hold, peak, v0)


def test_deepening_window_and_span_rule():
  t = np.arange(0.0, 2.0, 0.01)
  a = np.where(t < 1.0, 0.3, -0.7)   # a 1.0 drop at 1.0 s
  assert D.deepening(t, a, 0.0, 2.0, 0.3) == pytest.approx(-1.0)
  assert D.deepening(t, a, 1.0, 2.0, 0.3) == 0.0   # the drop is before the window
  assert D.deepening(t, a, 0.5, 0.7, 0.3) is None   # the window is shorter than the pair span
  gap = (t < 0.8) | (t >= 1.2)   # the drop falls in a 0.4 s sample gap: no pair spans 0.3 s within one frame across it
  assert D.deepening(t[gap], a[gap], 0.0, 2.0, 0.3) == 0.0
  assert D.deepening(t[gap], a[gap], 0.0, 2.0, 1.0) == pytest.approx(-1.0)
  a[150] = np.nan   # nan samples are skipped
  assert D.deepening(t, a, 0.0, 2.0, 0.3) == pytest.approx(-1.0)


def _plunge_route(t_plunge, entry=None):
  """_stop_trace (1 m/s at 14.0 s, wheel stop near 15 s) with a 100 Hz IMU: -0.3 body accel and a 1.0 drop over 0.2 s at t_plunge."""
  R = _route(_stop_trace())
  t = R['cs'][:, 0]
  body = -0.3 - np.clip((t - t_plunge) / 0.2, 0.0, 1.0)
  R['imu'] = np.column_stack([t, np.zeros(len(t)), np.zeros(len(t)), -body])   # zero calibration: body accel = -imu z
  R['calib'] = np.zeros((1, 4))
  R['co'] = np.column_stack([t, np.zeros(len(t))])
  R['logs'] = [] if entry is None else [(entry, 'APPROACH_GLIDE', 'INACTIVE')]
  return R


def test_plunge_from_entry_minus_1s_to_the_wheel_stop_and_attention():
  R = _plunge_route(14.3, entry=14.0)
  s = D.engaged_stops(R)[0]
  m = D.stop_metrics(R, s)
  assert m['deep300'] == pytest.approx(-1.0, abs=0.05) and m['deep1000'] == pytest.approx(-1.0, abs=0.01)
  m.update(rest=4.5, takeover_approach=False, takeover_hold=False)
  assert any(x.startswith('plunge') for x in D.attention(m, True))
  # entry at 14.0: a plunge before 13.0 is outside the window
  m = D.stop_metrics(_plunge_route(12.5, entry=14.0), s)
  assert m['deep300'] == 0.0 and m['deep1000'] == 0.0
  # no service entry: the window starts at the last 2.5 m/s crossing (12.5 s)
  m = D.stop_metrics(_plunge_route(12.7), s)
  assert m['t_entry'] is None and m['deep300'] == pytest.approx(-1.0, abs=0.05)
  assert D.stop_metrics(_plunge_route(12.0), s)['deep1000'] == 0.0


def test_plunge_attention_thresholds():
  m = dict(rest=4.5, a_stop=None, entry_bite=None, takeover_approach=False, takeover_hold=False, stopreq_pairs=[], hold_in_pid=[],
           starting_under_hold=[], deep300=-0.59, deep1000=-0.59)
  assert D.attention(m, True) == []
  assert D.attention(dict(m, deep300=-0.60), True) == ['plunge -0.6/-0.59']
  assert D.attention(dict(m, deep1000=-0.60), True) == ['plunge -0.59/-0.6']
  assert D.attention(dict(m, deep300=None, deep1000=None), True) == []


def test_starting_under_hold_ends_at_the_first_driver_input():
  # _stop_trace holds at rest 15-25 s under a logged HOLD; LongControl goes 'starting' at 20 s for 0.5 s
  gas = np.zeros(len(_stop_trace()))
  gas[1990:2100] = 1.0   # the driver's gas from 19.9 s: the override resets the service without a phase_change line
  tap = np.zeros(len(gas))
  tap[1800:1810] = 1.0   # a gas tap at 18 s ends the window: a later 'starting' is past it (the logged phase is stale)
  for g, want in ((None, [20.0]), (gas, []), (tap, [])):
    R = _route(_stop_trace(), gas=g)
    R['co'] = np.column_stack([R['cs'][:, 0], np.zeros(len(gas))])
    R['logs'] = [(14.0, 'HOLD', 'INACTIVE')]
    R['ctl'][2000:2050, 1] = D.LCS['starting']
    assert D.stop_metrics(R, D.engaged_stops(R)[0])['starting_under_hold'] == want


def _land_2235():
  R = D.load_route('00002235--991a0dec17')
  stops = D.engaged_stops(R)
  for s in stops:
    s['id'] = f"2235_{s['t_ws']:.2f}"
    s['m'] = D.stop_metrics(R, s)
  return R, stops


def test_land_baseline_matches_the_census_on_2235():
  R, stops = _land_2235()
  tr = D.trial_land('DELIVERED_LANDING', D.TRIALS['DELIVERED_LANDING'], False, False, stops, R)
  assert tr['mode'] == 'baseline' and tr['verdict'] == 'n/a' and not tr['checks'] and not tr['revert']
  assert tr['golden'] == (2, 12)
  row = next(r for r in tr['stops'] if r['stop'] == '2235_3106.94')
  assert row['swing'] == pytest.approx(0.606, abs=1e-3) and row['a_pre1'] == pytest.approx(-0.421, abs=1e-3)


def _land_live(R, stops):
  return D.trial_land('DELIVERED_LANDING', D.TRIALS['DELIVERED_LANDING'], True, True, stops, R)


def _ease(R, stops, idx):
  """The logged command eases -0.50 -> -0.38 below 0.45 m/s on the stops idx (the flag's landing)."""
  cs, cc = R['cs'], R['cc'].copy()
  for i in idx:
    w = (cc[:, 0] > stops[i]['t_ws'] - 2.0) & (cc[:, 0] <= stops[i]['t_ws']) & (D.at(cs, cc[:, 0], 1) < 0.45)
    cc[w, 3] = np.linspace(-0.50, -0.38, w.sum())
  return dict(R, cc=cc)


def test_land_live_rules_trip_on_a_stall_and_need_three_eased_stops(monkeypatch):
  R, stops = _land_2235()
  monkeypatch.setattr(D, 'stop_can', lambda R, lo, hi: np.zeros((0, 2)))
  tr = _land_live(R, stops)
  assert not any(r['eased'] for r in tr['stops']) and tr['verdict'] == 'PASS (too few eased stops for KEEP)'   # the guard's own ease
  R2 = _ease(R, stops, (0, 1, 2))
  cs = R2['cs'].copy()
  k = np.flatnonzero((cs[:, 0] > stops[2]['t_ws'] - 0.6) & (cs[:, 0] <= stops[2]['t_ws'] - 0.4))
  cs[k, 1] += 0.12   # the third stop re-accelerates before its wheel stop
  tr = _land_live(dict(R2, cs=cs), stops)
  assert sum(r['eased'] for r in tr['stops']) == 3
  assert [(c['rule'], c['stop']) for c in tr['checks'] if c['verdict'] == 'REVERT'] == [('stall', stops[2]['id'])]
  assert tr['revert'] == ['REVERT: DELIVERED_LANDING = False'] and tr['verdict'] == 'REVERT'


def test_land_safety_rules_run_on_every_lead_stop_and_missing_evidence_is_incomplete(monkeypatch):
  R, stops = _land_2235()
  stops[5]['m'] = dict(stops[5]['m'], min_gap=2.8)    # not eased: the floor still counts
  stops[6]['m'] = dict(stops[6]['m'], rest=None)      # no rest evidence: never a PASS
  tr = _land_live(R, stops)
  assert [(c['rule'], c['stop']) for c in tr['checks'] if c['verdict'] == 'REVERT'] == [('min gap', stops[5]['id'])]
  assert tr['missing'] == [f"{stops[6]['id']}: rest"] and tr['verdict'] == 'REVERT'
  stops[5]['m'] = dict(stops[5]['m'], min_gap=3.5)
  assert _land_live(R, stops)['verdict'] == 'INCOMPLETE'


@pytest.mark.parametrize('case', ['three_raw_pulses', 'counter_wrap', 'driver_launch'])
def test_land_roll_counts_raw_wrapped_pulses_until_a_driver_input(case):
  R, stops = _land_2235()
  s = stops[3]
  t_ws, base = next((r['t_ws'], r['roll_m']) for r in _land_live(R, stops)['stops'] if r['stop'] == s['id'])
  pul, cs = R['pul'].copy(), R['cs'].copy()
  w = np.flatnonzero((pul[:, 0] > t_ws + 0.3) & (pul[:, 0] <= t_ws + 3.0))
  if case == 'counter_wrap':   # the scaled 8-bit counter wraps 127.5 -> 0.0: two raw pulses
    pul[w[0]:, 1:5] = 127.0
    pul[w[10]:, 1:5] = 127.5
    pul[w[20]:, 1:5] = 0.0
    pul[:w[0], 1:5] = 127.0
  else:   # three raw pulses (0.5 each in DBC units) after the stop
    for j, n in ((w[10], 0.5), (w[20], 1.0), (w[30], 1.5)):
      pul[j:w[-1] + 1, 1:5] = R['pul'][j:w[-1] + 1, 1:5] + n
  if case == 'driver_launch':
    cs[(cs[:, 0] >= t_ws + 0.25) & (cs[:, 0] <= t_ws + 3.0), 4] = 1.0   # gas before the pulses: a departure, not a roll
  row = next(r for r in _land_live(dict(R, pul=pul, cs=cs), stops)['stops'] if r['stop'] == s['id'])
  mpp = __import__('openpilot.tools.stopping.sim.harness', fromlist=['_mpp'])._mpp()
  want = {'three_raw_pulses': base + 3 * mpp, 'counter_wrap': 2 * mpp, 'driver_launch': 0.0}[case]   # the counter_wrap trace replaces the log
  assert row['roll_m'] == pytest.approx(want, abs=1.5e-3)


# ---- GAP_AWARE_TERMINAL trial (Astra review findings 2, 4, 5, 6) -------------------------------------------------------------
GAT_FLAG = 'GAP_AWARE_TERMINAL'


def _gat_arm(n=400, phase=1, wire=-0.3):
  t = 10.0 + 0.01 * np.arange(n)
  ph = np.full(n, phase)
  ph[250:] = D.PHASES.index('RAMP_TO_HOLD')
  ph[260:] = D.PHASES.index('HOLD')
  return dict(t=t, wire=np.full(n, wire), phase=ph, stopreq=np.zeros(n), lcs=np.ones(n), rehold=np.zeros(n))


def _gat_cs(v_in=1.6, t_band=11.5):
  """carState over the _gat_arm span: v_in until t_band, then 0.9 m/s (the wait in the band)."""
  t = 10.0 + 0.01 * np.arange(400)
  cs = np.zeros((400, 6))
  cs[:, 0], cs[:, 1] = t, np.where(t < t_band, v_in, 0.9)
  return cs


def _gat_run(monkeypatch, on, off, stops, live=True):
  monkeypatch.setattr(D, 'run_arm', lambda base, flags, *a, **k: {'s': on if flags.get(GAT_FLAG) else off})
  R = dict(lp=np.array([[0.0, 0.0, 0.0, 0.2, -1.0, 1.0]]), cs=_gat_cs())
  return D.trial_gap(GAT_FLAG, dict(name='GAT'), 'f' * 40, live, True, [dict(span='s')], stops, {'s': on}, R, print)


def _gat_stop(**m):
  base = dict(rest=4.4, t_entry=11.0, a_stop=-0.5, felt=1.0, deep300=-0.3, deep1000=-0.3, lead_at_stop=True, vlead_at_stop=0.0)
  return dict(id='x', span='s', kind='stop', t_ws=12.5, m=base | m)


def test_gat_hold_difference_after_the_settle_reverts_even_when_it_started_in_the_approach(monkeypatch):
  on, off = _gat_arm(), _gat_arm()
  on['wire'][200:] = -0.25   # starts in the approach, carries into HOLD past the 1 s settle
  tr = _gat_run(monkeypatch, on, off, [_gat_stop()])
  assert tr['verdict'] == 'REVERT' and any(c['rule'] == 'difference in HOLD / RELEASE' for c in tr['checks'])
  on2 = _gat_arm()
  on2['wire'][200:300] = -0.25   # ends inside the hold's first second (the handover): a delayed stop, not a hold change
  tr = _gat_run(monkeypatch, on2, _gat_arm(), [_gat_stop()])
  assert tr['verdict'] == 'PASS' and tr['stops'][0]['delayed']


def test_gat_new_bite_is_a_deeper_entry_command_at_any_speed(monkeypatch):
  on = _gat_arm()
  on['wire'][105:110] = -0.6   # 0.05 s after the service entry (11.0 s): deeper than off
  tr = _gat_run(monkeypatch, on, _gat_arm(), [_gat_stop()])
  assert any(c['rule'].startswith('new bite') for c in tr['checks']) and tr['verdict'] == 'REVERT'


def test_gat_rest_rules_only_on_stopped_lead_stops(monkeypatch):
  rolling = dict(_gat_stop(rest=3.4), kind='rolling')
  moving = _gat_stop(rest=3.4, vlead_at_stop=1.2)
  tr = _gat_run(monkeypatch, _gat_arm(), _gat_arm(), [rolling, moving])
  assert tr['stops'] == [] and tr['verdict'] == 'PASS'
  tr = _gat_run(monkeypatch, _gat_arm(), _gat_arm(), [_gat_stop(rest=3.4)])
  assert tr['verdict'] == 'REVERT'


def test_gat_counterfactual_failure_is_not_a_pass(monkeypatch):
  on = _gat_arm()
  on['wire'][300:] = -0.25   # a hold difference only
  tr = _gat_run(monkeypatch, on, _gat_arm(), [_gat_stop()], live=False)
  assert tr['revert'] == [] and tr['verdict'] == 'FAIL'


def test_gat_missing_landing_metrics_on_a_delayed_stop_is_look(monkeypatch):
  on = _gat_arm()
  on['wire'][150:200] = -0.25
  tr = _gat_run(monkeypatch, on, _gat_arm(), [_gat_stop(a_stop=None, felt=None, deep300=None, deep1000=None)])
  assert tr['stops'][0]['verdict'] == 'LOOK' and tr['verdict'] == 'WATCH'


def test_gat_closer_rest_with_a_later_release_is_look_not_revert(monkeypatch):
  # runner H5 (2026-10-12): a delayed stop rests closer and its release comes > 0.3 s after the flag-off replay's -> LOOK; the hold /
  # release difference inside that window is explained by it
  on, off = _gat_arm(), _gat_arm()
  on['wire'][150:200] = -0.25                      # the delayed arm in the approach
  off['phase'][330:] = D.PHASES.index('RELEASE')   # off releases at 13.30 s
  on['phase'][370:] = D.PHASES.index('RELEASE')    # on 0.40 s later
  tr = _gat_run(monkeypatch, on, off, [_gat_stop()])
  assert tr['stops'][0]['verdict'] == 'LOOK' and tr['stops'][0]['late_release'] == pytest.approx(0.4)
  assert not any(c['verdict'] == 'REVERT' for c in tr['checks']) and tr['verdict'] == 'WATCH'


def test_gat_table_shows_the_logged_arm_line(monkeypatch):
  # WS TRIALFIX (22a2_183.39): the line that armed each stop's guard (the stall line is a creep-carried wait) is in the table
  monkeypatch.setattr(D, 'run_arm', lambda base, flags, *a, **k: {'s': _gat_arm()})
  stops = [_gat_stop()]
  R = dict(lp=np.array([[0.0, 0.0, 0.0, 0.2, -1.0, 1.0]]), gat=[(5.0, 'v_low'), (11.4, 'stall'), (13.0, 'distance')])
  tr = D.trial_gap(GAT_FLAG, dict(name='GAT'), 'f' * 40, True, True, [dict(span='s')], stops, {'s': _gat_arm()}, R, print)
  assert tr['stops'][0]['line'] == 'stall'
  assert '| stall |' in '\n'.join(D.markdown_gap({}, tr))
  R['gat'] = None   # segments scanned before the lines were read
  tr = D.trial_gap(GAT_FLAG, dict(name='GAT'), 'f' * 40, True, True, [dict(span='s')], stops, {'s': _gat_arm()}, R, print)
  assert tr['stops'][0]['line'] == '?'


@pytest.mark.parametrize('case', ['own', 'entry_only', 'within_tol'])
def test_gat_plunge_counts_from_the_wait_start_where_the_flag_changes_the_command(monkeypatch, case):
  # WS TERMINAL 2026-10-16 (review1016 section 2): 5 of 7 REVERT rows were entry take-overs with the flag-off command identical; the
  # trial's plunge window starts inside the wait (the first frame below 1.3 m/s after the entry, 11.5 s here) where the flag changes the
  # command (11.6 s)
  on, off = _gat_arm(), _gat_arm()
  if case == 'own':
    on['wire'][160:200] = -0.20          # the waiting command (11.6-12.0 s), 0.10 shallower than off
  elif case == 'entry_only':
    on['wire'][105:140] = -0.20          # the arms differ only before the wait start
  else:
    on['wire'][160:200] = -0.26          # inside the wait, within the 0.05 tolerance (shallower: no bite)
  seen = []
  monkeypatch.setattr(D, 'imu_deep', lambda R, lo, hi: seen.append((lo, hi)) or (-0.70, -0.75))
  monkeypatch.setattr(D, 'run_arm', lambda base, flags, *a, **k: {'s': on if flags.get(GAT_FLAG) else off})
  t = 10.0 + 0.01 * np.arange(400)
  cs = np.zeros((400, 6))
  cs[:, 0], cs[:, 1] = t, np.where(t < 11.5, 1.6, 0.9)
  R = dict(lp=np.array([[0.0, 0.0, 0.0, 0.2, -1.0, 1.0]]), cs=cs)
  tr = D.trial_gap(GAT_FLAG, dict(name='GAT'), 'f' * 40, True, True, [dict(span='s')], [_gat_stop(deep300=-0.9, deep1000=-0.9)], {'s': on}, R, print)
  row = tr['stops'][0]
  assert row['delayed']
  if case == 'own':
    assert seen == [(pytest.approx(11.6), 12.5)] and row['deep300'] == -0.70 and row['verdict'] == 'LOOK' and tr['verdict'] == 'WATCH'
  else:   # the stop table's -0.9 entry plunge is not the flag's
    assert not seen and row['deep300'] is None and row['verdict'] == 'PASS' and tr['verdict'] == 'PASS'


@pytest.mark.parametrize('felt,a_stop,verdict', [(0.66, -0.30, 'LOOK'),     # 22b5_356.25: the plunge is not felt
                                                  (1.59, -0.81, 'REVERT'),   # 22c1_1149.73
                                                  (1.40, -0.30, 'REVERT'), (0.66, -0.70, 'REVERT')])
def test_gat_own_plunge_reverts_only_on_an_uncomfortable_stop(monkeypatch, felt, a_stop, verdict):
  # orchestrator decision PLAN 62 (2026-10-17): a GAT-own plunge below -0.60 is a FAIL only with felt > 1.33 or a_stop < -0.66
  on, off = _gat_arm(), _gat_arm()
  on['wire'][160:200] = -0.20   # the flag-owned waiting command, shallower than off (no bite)
  monkeypatch.setattr(D, 'imu_deep', lambda R, lo, hi: (-0.40, -0.85))
  tr = _gat_run(monkeypatch, on, off, [_gat_stop(felt=felt, a_stop=a_stop)])
  row = tr['stops'][0]
  assert row['verdict'] == verdict and 'deep1000 -0.85' in ' '.join(row['why'])
  assert tr['verdict'] == ('WATCH' if verdict == 'LOOK' else 'REVERT') and bool(tr['revert']) == (verdict == 'REVERT')
  if verdict == 'LOOK':
    assert row['why'] == ['GAT creep-wait plunge deep1000 -0.85, not felt']


@pytest.mark.parametrize('deep,verdict', [((-0.30, None), 'PASS'), ((None, None), 'LOOK')])
def test_gat_short_owned_window_is_not_missing_evidence(monkeypatch, deep, verdict):
  # 22c6_298.97: the flag-owned window is shorter than 1 s, so no 1 s deepening exists; only no IMU at all is missing evidence
  on, off = _gat_arm(), _gat_arm()
  on['wire'][160:200] = -0.40
  monkeypatch.setattr(D, 'imu_deep', lambda R, lo, hi: deep)
  monkeypatch.setattr(D, 'run_arm', lambda base, flags, *a, **k: {'s': on if flags.get(GAT_FLAG) else off})
  t = 10.0 + 0.01 * np.arange(400)
  cs = np.zeros((400, 6))
  cs[:, 0], cs[:, 1] = t, np.where(t < 11.5, 1.6, 0.9)
  R = dict(lp=np.array([[0.0, 0.0, 0.0, 0.2, -1.0, 1.0]]), cs=cs)
  tr = D.trial_gap(GAT_FLAG, dict(name='GAT'), 'f' * 40, True, True, [dict(span='s')], [_gat_stop()], {'s': on}, R, print)
  assert tr['stops'][0]['verdict'] == verdict and tr['stops'][0]['t_own'] == pytest.approx(11.6)


@pytest.mark.parametrize('case', ['no_cs', 'cs_gap', 'nan_wire'])
def test_gat_unknown_attribution_is_look_not_a_pass(monkeypatch, case):
  # Astra P2-2: missing / gapped carState or a non-finite replay command must not read as "not the flag's"
  on, off = _gat_arm(), _gat_arm()
  on['wire'][160:200] = -0.20
  if case == 'nan_wire':
    on['wire'][230] = np.nan
  monkeypatch.setattr(D, 'imu_deep', lambda R, lo, hi: (-1.0, -1.0))
  monkeypatch.setattr(D, 'run_arm', lambda base, flags, *a, **k: {'s': on if flags.get(GAT_FLAG) else off})
  cs = _gat_cs()
  R = dict(lp=np.array([[0.0, 0.0, 0.0, 0.2, -1.0, 1.0]]))
  if case == 'cs_gap':
    R['cs'] = cs[(cs[:, 0] < 11.5) | (cs[:, 0] >= 12.1)]
  elif case == 'nan_wire':
    R['cs'] = cs
  tr = D.trial_gap(GAT_FLAG, dict(name='GAT'), 'f' * 40, True, True, [dict(span='s')], [_gat_stop()], {'s': on}, R, print)
  row = tr['stops'][0]
  assert row['own'] is None and row['verdict'] == 'LOOK' and 'attribution' in row['why'][0] and tr['verdict'] == 'WATCH'


def test_gat_in_band_entry_plunge_before_the_flag_changes_the_command_is_not_counted(monkeypatch):
  # Astra P2-3: an entry already below 1.3 m/s (wait start = entry 11.0); the arms are identical until 11.6: the deepening is measured
  # from 11.6 only
  on, off = _gat_arm(), _gat_arm()
  on['wire'][160:200] = -0.20
  seen = []
  monkeypatch.setattr(D, 'imu_deep', lambda R, lo, hi: seen.append(lo) or (-0.2, -0.3))
  monkeypatch.setattr(D, 'run_arm', lambda base, flags, *a, **k: {'s': on if flags.get(GAT_FLAG) else off})
  R = dict(lp=np.array([[0.0, 0.0, 0.0, 0.2, -1.0, 1.0]]), cs=_gat_cs(v_in=0.9))
  tr = D.trial_gap(GAT_FLAG, dict(name='GAT'), 'f' * 40, True, True, [dict(span='s')], [_gat_stop(deep300=-1.0, deep1000=-1.0)], {'s': on}, R, print)
  assert seen == [pytest.approx(11.6)] and tr['stops'][0]['verdict'] == 'PASS'


def test_land_roll_window_ends_at_the_release():
  # 22a7_521.24: the planner's go behind a departing lead released the hold 0.04 s after it and the release ramp under creep was
  # counted as a 0.12 m roll; the window now ends at the service's RELEASE
  R, stops = _land_2235()
  s = stops[3]
  t_ws, base = next((r['t_ws'], r['roll_m']) for r in _land_live(R, stops)['stops'] if r['stop'] == s['id'])
  pul = R['pul'].copy()
  w = np.flatnonzero((pul[:, 0] > t_ws + 0.3) & (pul[:, 0] <= t_ws + 3.0))
  for j, n in ((w[10], 0.5), (w[20], 1.0), (w[30], 1.5)):
    pul[j:w[-1] + 1, 1:5] = R['pul'][j:w[-1] + 1, 1:5] + n
  logs = sorted(R.get('logs', []) + [(t_ws + 0.25, 'RELEASE', 'HOLD')], key=lambda x: x[0])
  row = next(r for r in _land_live(dict(R, pul=pul, logs=logs), stops)['stops'] if r['stop'] == s['id'])
  assert row['roll_m'] == pytest.approx(0.0 if base is None else min(base, 0.0) + 0.0, abs=1.5e-3) or row['roll_m'] <= base + 1e-6
  assert row['t_end'] == pytest.approx(0.25, abs=0.02)


def _stall_regrab(R, stop, depth):
  """After the ease the command re-deepens by depth in the 0.3 s before the trial's wheel stop (the pulse edge) while aEgo reads -0.10
  (the stall guard's predicate)."""
  t_ws = next(r['t_ws'] for r in _land_live(R, [stop])['stops'])
  cc, cs = R['cc'].copy(), R['cs'].copy()
  w = (cc[:, 0] > t_ws - 0.3) & (cc[:, 0] < t_ws)
  cc[w, 3] = np.linspace(-0.38, -0.38 - depth, w.sum())
  cs[(cs[:, 0] > t_ws - 0.35) & (cs[:, 0] <= t_ws - 0.2), 2] = -0.10
  return dict(R, cc=cc, cs=cs)


def test_land_stall_regrabs_look_and_two_deep_ones_revert(monkeypatch):
  # WS TRIALFIX (J_LAND, car-first): any stall-guard re-grab is LOOK; >= 2 deeper than 0.20 in one drive REVERT
  R, stops = _land_2235()
  monkeypatch.setattr(D, 'stop_can', lambda R, lo, hi: np.zeros((0, 2)))
  R2 = _ease(R, stops, (0, 1, 2))
  one = _stall_regrab(R2, stops[0], 0.25)
  tr = _land_live(one, stops)
  row = next(r for r in tr['stops'] if r['stop'] == stops[0]['id'])
  assert row['stall_regrab'] and row['regrab'] > 0.2
  assert [c['stop'] for c in tr['checks'] if c['rule'] == 'stall re-grab' and c['verdict'] == 'LOOK'] == [stops[0]['id']]
  assert not any(c['rule'] == 'stall re-grabs' for c in tr['checks'])
  two = _stall_regrab(one, stops[1], 0.25)
  tr = _land_live(two, stops)
  assert [c['rule'] for c in tr['checks'] if c['verdict'] == 'REVERT'] == ['stall re-grabs'] and tr['verdict'] == 'REVERT'
  shallow = _stall_regrab(_stall_regrab(R2, stops[0], 0.15), stops[1], 0.15)   # two, but not deeper than 0.20: LOOK only
  assert not _land_live(shallow, stops)['revert']


def test_land_keep_needs_six_eased_stops(monkeypatch):
  R, stops = _land_2235()
  monkeypatch.setattr(D, 'stop_can', lambda R, lo, hi: np.zeros((0, 2)))
  tr = _land_live(_ease(R, stops, range(5)), stops)
  assert tr['golden'][1] == 5 and not tr['revert'] and tr['verdict'] in ('LOOK', 'PASS (too few eased stops for KEEP)')
  tr = _land_live(_ease(R, stops, range(6)), stops)
  g, n = tr['golden']
  assert n == 6 and bool(tr['revert']) == (g < D.LAND['keep'] * n)


# ---- ENTRY_COMFORT_CAP trial (PROPOSAL_v2 TRIALS; Astra code review P2-4 / 5 / 6) ---------------------------------------------
ENTRY_FLAG = 'ENTRY_COMFORT_CAP'
ENTRY_R = dict(cs=np.zeros((5, 8)), co=np.zeros((5, 3)))   # the logged car streams a live drive needs


def _entry_run(monkeypatch, on, off, stops, live=True, bookmarked=(), R=ENTRY_R):
  monkeypatch.setattr(D, 'run_arm', lambda base, flags, *a, **k: {'s': on if flags.get(ENTRY_FLAG) else off})
  return D.trial_entry(ENTRY_FLAG, dict(name='ENTRY'), 'f' * 40, live, True, [dict(span='s')], stops, {'s': on}, R, print,
                       bookmarked=bookmarked)


def _entry_arm(**kw):
  a = _gat_arm(wire=-0.6, **kw)
  a['ec'] = np.zeros(len(a['t']))
  return a


def _capped_pair(peak_on=-0.75, peak_off=-1.13, armed=True):
  on, off = _entry_arm(), _entry_arm()
  off['wire'][110:150] = peak_off   # today's catch-up after the service entry (11.0 s)
  on['wire'][110:150] = peak_on
  if armed:
    on['ec'][105:150] = 1
  return on, off


def test_entry_armed_stop_with_evidence_keeps(monkeypatch):
  tr = _entry_run(monkeypatch, *_capped_pair(), [_gat_stop(rest=3.8, a_stop=-0.5, felt=1.0)])
  assert tr['stops'][0]['armed'] and tr['stops'][0]['capped'] and tr['stops'][0]['peak'] == pytest.approx(-0.75) and tr['verdict'] == 'KEEP'


def test_entry_missing_evidence_is_not_judged(monkeypatch):
  tr = _entry_run(monkeypatch, *_capped_pair(), [_gat_stop()], R={})                    # live without the logged streams
  assert tr['verdict'] == 'NOT JUDGED'
  tr = _entry_run(monkeypatch, *_capped_pair(), [_gat_stop(), dict(_gat_stop(rest=None), id='y')])
  assert tr['verdict'] == 'NOT JUDGED' and tr['stops'][1]['verdict'] == 'NOT JUDGED'
  tr = _entry_run(monkeypatch, *_capped_pair(), [_gat_stop(a_stop=None)])
  assert tr['verdict'] == 'NOT JUDGED'
  tr = _entry_run(monkeypatch, _entry_arm(), _entry_arm(), [_gat_stop()])                # nothing armed
  assert tr['verdict'] == 'NOT JUDGED' and tr['revert'] == []


def test_entry_armed_cohort_includes_masked_stops(monkeypatch):
  # the planner masks the cap (equal wires) but the episode armed: its comfort rules apply
  on, off = _capped_pair(peak_on=-1.13)
  tr = _entry_run(monkeypatch, on, off, [_gat_stop(a_stop=-0.8, felt=2.0)])
  assert tr['stops'][0]['armed'] and not tr['stops'][0]['capped'] and tr['verdict'] == 'REVERT'


@pytest.mark.parametrize('stop,peak_on,want', [
  (dict(rest=2.9), -0.75, 'REVERT'),            # the floor on any stop
  (dict(a_stop=-0.7), -0.75, 'REVERT'),         # harder landing on an armed stop
  (dict(felt=1.4), -0.75, 'REVERT'),
  (dict(), -1.2, 'REVERT'),                     # a deeper peak than the flag-off replay
  (dict(rest=3.4), -0.75, 'LOOK'),              # the margin is used
  (dict(), -0.85, 'LOOK'),                      # a spend or a lane bind under the cap
  (dict(), -1.13, 'LOOK'),                      # armed, peak not shallower
])
def test_entry_rules(monkeypatch, stop, peak_on, want):
  on, off = _capped_pair(peak_on=peak_on)
  if peak_on < -1.13:
    on['wire'][110:120] = -0.7                  # the cap acted first
  tr = _entry_run(monkeypatch, on, off, [_gat_stop(**stop)])
  assert tr['verdict'] == want


@pytest.mark.parametrize('k_on', [280, None])
def test_entry_later_or_missing_release_is_look(monkeypatch, k_on):
  on, off = _capped_pair()
  off['phase'][270:] = D.PHASES.index('RELEASE')     # 12.70 s
  if k_on is not None:
    on['phase'][k_on:] = D.PHASES.index('RELEASE')   # 12.80 s
  tr = _entry_run(monkeypatch, on, off, [_gat_stop()])
  assert tr['verdict'] == 'LOOK' and any('release' in w for w in tr['stops'][0]['why'])


def test_entry_counterfactual_never_reverts_and_bookmark_is_look(monkeypatch):
  tr = _entry_run(monkeypatch, *_capped_pair(), [_gat_stop(felt=1.4)], live=False)
  assert tr['revert'] == [] and tr['verdict'] == 'FAIL'
  tr = _entry_run(monkeypatch, *_capped_pair(), [_gat_stop()], bookmarked={'x'})
  assert tr['verdict'] == 'LOOK' and 'bookmark' in tr['stops'][0]['why']
  assert '| x |' in '\n'.join(D.markdown_entry({}, tr))


# ---- LANDING_LEAD_TOLERANCE trial (cyc_1016/capture/PROPOSAL.md TRIALS) -------------------------------------------------------
CAPT_FLAG = 'LANDING_LEAD_TOLERANCE'


def _capt_run(monkeypatch, on, off, stops, live=True):
  monkeypatch.setattr(D, 'run_arm', lambda base, flags, *a, **k: {'s': on if flags.get(CAPT_FLAG) else off})
  return D.trial_capt(CAPT_FLAG, dict(name='CAPT'), 'f' * 40, live, True, [dict(span='s')], stops, {'s': on}, {}, print)


def _capt_pair(held=True, event=False, ease=0.35, k0=190):
  on, off = _gat_arm(wire=-0.7), _gat_arm(wire=-0.7)
  on['land'], off['land'] = np.zeros(400), np.full(400, 0.0)
  off['land'][k0:250] = 8                           # flag-off: the fail-safe latched on the capture frame (11.90 s)
  on['wire'][k0:250] = np.minimum(np.linspace(-0.7, -0.7 + ease * (250 - k0) / 60, 250 - k0), -0.7 + ease)
  on['land'][k0:250] = 2 if held else 1            # eased on the tolerance band alone (2) / with the strict latch (1)
  if event:
    on['land'][220] += 4
  return on, off


@pytest.mark.parametrize('pair,stop,want', [
  (dict(), dict(a_stop=-0.4), 'KEEP'),
  (dict(), dict(rest=2.9, a_stop=-0.4), 'REVERT'),                   # the floor
  (dict(event=True), dict(a_stop=-0.4), 'REVERT'),                   # a nearer target / identity change during the tolerance ease
  (dict(k0=60), dict(a_stop=-0.4), 'REVERT'),                       # a 1.9 s tolerance ease: estimated > 0.3 m closer than flag-off
  (dict(), dict(a_stop=-0.72), 'LOOK'),                              # a_stop not shallower than today's -0.70 landing
  (dict(), dict(a_stop=None), 'LOOK'),
  (dict(held=False), dict(a_stop=-0.4), 'NOT JUDGED'),               # the strict latch held the ease: not a tolerance stop
])
def test_capt_rules(monkeypatch, pair, stop, want):
  tr = _capt_run(monkeypatch, *_capt_pair(**pair), [_gat_stop(**stop)])
  assert tr['verdict'] == want
  assert tr['stops'][0]['tol'] == pair.get('held', True)


def test_capt_counterfactual_never_reverts_and_old_cache_is_incomplete(monkeypatch):
  tr = _capt_run(monkeypatch, *_capt_pair(event=True), [_gat_stop(a_stop=-0.4)], live=False)
  assert tr['revert'] == [] and tr['verdict'] == 'FAIL'
  assert '| x |' in '\n'.join(D.markdown_capt({}, tr))
  on, off = _capt_pair()
  del off['land']                                   # a replay without the landing column
  tr = _capt_run(monkeypatch, on, off, [_gat_stop(a_stop=-0.4)])
  assert tr['verdict'] == 'INCOMPLETE' and tr['spans_failed'] == ['s']


def test_capt_each_rule_reads_its_own_evidence(monkeypatch):
  # Astra review P2-2 / P2-3: a measured floor breach reverts without a replay; a replay event reverts without a measured rest
  on, off = _capt_pair()
  del off['land']
  tr = _capt_run(monkeypatch, on, off, [_gat_stop(rest=2.9, a_stop=-0.4)])
  assert tr['verdict'] == 'REVERT' and tr['spans_failed'] == ['s'] and tr['stops'][0]['why'] == ['rest 2.9 < 3.0']
  tr = _capt_run(monkeypatch, *_capt_pair(event=True), [_gat_stop(rest=None, a_stop=-0.4)])
  assert tr['verdict'] == 'REVERT' and tr['stops'][0]['tol']
  tr = _capt_run(monkeypatch, *_capt_pair(), [_gat_stop(rest=None, a_stop=-0.4)])
  assert tr['verdict'] == 'NOT JUDGED' and tr['stops'][0]['tol'] and tr['stops'][0]['why'] == ['no rest']


def _launch(vl_from=None, clr_on=100, clr_off=160, back_at=None, gas_at=None, n=400):
  """LAUNCH trial arms: HOLD -> RELEASE at 0.9 s; the flag-on arm clears StopReq at clr_on, the flag-off arm at clr_off (frames)."""
  on, off = _arm(n), _arm(n)
  on['phase'][90:clr_on] = 5
  on['phase'][clr_on:] = 0
  off['phase'][90:clr_off] = 5
  off['phase'][clr_off:] = 0
  on['stopreq'][clr_on + 1:] = 0
  on['lcs'][clr_on:] = 3
  off['stopreq'][clr_off + 1:] = 0
  off['lcs'][clr_off:] = 3
  if back_at is not None:
    on['lcs'][back_at:] = 2
  inp = _inputs(n, gas=None if gas_at is None else (np.arange(n) >= gas_at).astype(int))
  inp['act'] = np.ones(n, int)
  inp['vl'] = np.where(np.arange(n) >= (80 if vl_from is None else vl_from), 0.8, 0.0)
  inp['tid'] = np.full(n, 7)
  return on, off, inp


def _rs(inp, tid=None, step=5):
  """The logged radarState rows of the inputs at 20 Hz (t, status, dRel, vLead, aLeadK, radarTrackId)."""
  k = np.arange(0, len(inp['t']), step)
  return np.c_[inp['t'][k], inp['lead'][k], inp['gap'][k], inp['vl'][k], np.zeros(len(k)), inp['tid'][k] if tid is None else tid[k]]


def _lc(on, off, inp, live=False, R=None, car=None):
  R = dict(R or {})
  R.setdefault('rs', _rs(inp))
  return D.launch_checks(on, off, inp, live, R, car)


def test_launch_checks_clean_early_clear_and_each_violation():
  on, off, inp = _launch()
  c = _lc(on, off, inp)
  assert len(c) == 1 and c[0]['flag_only'] and c[0]['leaving'] and c[0]['violations'] == []
  assert c[0]['t_decision'] == pytest.approx(0.9) and c[0]['clear_off'] == pytest.approx(1.61)
  assert D.launch_decide(c, live=True) == []
  # L1: the lead never leaves; L3 (per drive): more returns to 'stopping' at rest after a launch than the flag-off arm; L5: later
  assert _lc(*_launch(vl_from=10**6))[0]['violations'] == ['L1']
  c = _lc(*_launch(back_at=150))
  assert (c[0]['reholds'], c[0]['reholds_off']) == (1, 0) and [x['rule'] for x in D.launch_decide(c, live=False)] == ['L3']
  assert 'L5' in _lc(*_launch(clr_on=170, clr_off=100))[0]['violations']
  # a back-to-stopping under the driver's gas is the driver's (the rule window ends at the first driver input)
  c = _lc(*_launch(back_at=150, gas_at=140))
  assert c[0]['violations'] == [] and c[0]['reholds'] == 0 and D.launch_decide(c, live=False) == []


def test_launch_decide_counts_per_drive():
  # L3: re-holds above BASE (the flag-on returns to 'stopping' against the flag-off arm's, per drive)
  one = [dict(t=1.0, flag_only=True, jerk=None, violations=[], reholds=1, reholds_off=1)]
  assert D.launch_decide(one, live=True) == []
  two = one + [dict(t=9.0, flag_only=True, jerk=None, violations=[], reholds=1, reholds_off=0)]
  assert [x['rule'] for x in D.launch_decide(two, live=True)] == ['L3']
  jerk = [dict(t=1.0, flag_only=True, jerk=5.0, violations=[], reholds=0, reholds_off=0),
          dict(t=2.0, flag_only=True, jerk=4.8, violations=[], reholds=0, reholds_off=0)]
  assert [x['rule'] for x in D.launch_decide(jerk, live=True)] == ['L4']
  assert D.launch_decide(jerk, live=False) == [] and D.launch_decide(jerk[:1], live=True) == []


def test_launch_review_cases():
  # Astra review 2026-10-12: a driver input in an EARLIER standstill does not exempt the next autonomous one (P1-3)
  on, off, inp = _launch(vl_from=10**6)
  inp['gas'][10:14] = 1
  inp['v'][15:40] = 0.5
  assert _lc(on, off, inp)[0]['violations'] == ['L1']
  # a much earlier flag-on clear (a skipped -3.5 ramp) pairs within its standstill, no L5 (P2-4)
  c = _lc(*_launch(clr_on=100, clr_off=270))
  assert len(c) == 1 and c[0]['violations'] == [] and c[0]['clear_off'] == pytest.approx(2.71)
  # live: the logged StopReq / state is the flag-on world (P1-2): a logged clear toward a stopped lead trips L1 although the replay
  # arms hold
  on, off, inp = _launch(vl_from=10**6, clr_on=399, clr_off=399)
  car = dict(stopreq=np.r_[np.ones(101, int), np.zeros(299, int)], lcs=np.r_[np.full(100, 2), np.full(300, 3)])
  rows = _lc(on, off, inp, live=True, R={}, car=car)
  assert rows[0]['violations'][:1] == ['L1'] and 'jerk unchecked' in rows[0]['notes'][0]
  # L2 measures ego travel only until the lead departed (P2-5): the lead departs at 0.8 s, the ego moves from 1.2 s
  on, off, inp = _launch(vl_from=80)
  inp['v'][120:] = 0.5
  inp['v'][100:120] = 0.0
  assert 'L2' not in _lc(on, off, inp, live=True)[0]['violations']
  # ...and trips when the ego travels 0.25 m before any departure
  on, off, inp = _launch(vl_from=10**6)
  inp['v'][103:] = 0.2
  assert 'L2' in _lc(on, off, inp, live=True)[0]['violations']


def test_launch_first_motion_per_live_launch():
  # decision -> StopReq clear -> first motion: live rows carry the first frame the ego moves (v > move_v) after the clear
  on, off, inp = _launch()
  inp['v'][180:] = 0.3
  row = _lc(on, off, inp, live=True)[0]
  assert row['t_decision'] == pytest.approx(0.9) and row['t'] == pytest.approx(1.01) and row['t_move'] == pytest.approx(1.8)
  assert 't_move' not in _lc(*_launch())[0]


def test_launch_rest_below_floor_short_hold_and_unknown_lead():
  # L6 (live): a settled rest below 3.0 m after real motion reverts
  on, off, inp = _launch()
  inp['v'][180:260] = 0.4
  inp['gap'][260:] = 2.9
  row = _lc(on, off, inp, live=True)[0]
  assert row['rest_min'] == pytest.approx(2.9) and 'L6' in row['violations']
  assert [x['rule'] for x in D.launch_decide([row], live=True)] == ['L6'] and D.launch_decide([row], live=False) == []
  # the short-hold class: StopReq never set before the flag-on launch (a real brake release on the launch frame)
  on, off, inp = _launch()
  on['stopreq'][:] = 0
  row = _lc(on, off, inp)[0]
  assert row['short'] and row['t'] == pytest.approx(1.0) and 'short hold' in row['notes'][0]


def test_launch_lead_classifier_needs_one_covered_track():
  # Astra code review 2026-10-17 P2-3: radarState rows with a 0.3 s hole across the purported departure are UNKNOWN (never joined)
  on, off, inp = _launch()
  rs = _rs(inp)
  rs = rs[(rs[:, 0] < 0.95) | (rs[:, 0] > 1.25)]
  row = _lc(on, off, inp, R=dict(rs=rs))[0]
  assert row['leaving'] is None and 'L1' not in row['violations'] and 'UNKNOWN' in row['notes'][0]
  # a new track id every 0.1 s: no one track holds the departure for 0.2 s -> not leaving (L1)
  row = _lc(on, off, inp, R=dict(rs=_rs(inp, tid=np.arange(len(inp['t'])) // 10)))[0]
  assert row['leaving'] is False and 'L1' in row['violations']
  # a span that ends before the classification window does: UNKNOWN, never L1 (the window is not clipped to the span)
  on, off, inp = _launch(vl_from=10**6, clr_off=999, n=121)
  row = _lc(on, off, inp)[0]
  assert row['leaving'] is None and row['violations'] == []
  assert D.lead_leaving(None, 1.0) is None and D.lead_leaving(_rs(_launch()[2]), 1.01) is True


def test_launch_pairs_every_release_and_counts_reholds():
  # P2-4: both arms clear at 1.01 s and re-hold at 1.40 s; the lead stops; ON alone clears again at 2.51 s and re-holds at 3.0 s
  on, off, inp = _launch(clr_on=100, clr_off=100, n=600)
  inp['vl'][120:] = 0.0
  for a in (on, off):
    a['stopreq'][140:] = 1
    a['lcs'][140:] = 2
  on['stopreq'][251:300] = 0
  on['lcs'][251:300] = 3
  c = _lc(on, off, inp)
  assert len(c) == 2 and c[1]['flag_only'] and c[1]['violations'] == ['L1'] and (c[0]['reholds'], c[0]['reholds_off']) == (2, 1)
  assert sorted(x['rule'] for x in D.launch_decide(c, live=False)) == ['L1', 'L3']


def test_launch_floor_rest_and_windows():
  # P2-5: the same launch time in both arms, then a settled rest at 2.9 m after real motion: L6 whatever the clear timing
  on, off, inp = _launch(clr_on=100, clr_off=100, n=600)
  inp['v'][180:260] = 0.4
  inp['gap'][260:] = 2.9
  assert 'L6' in _lc(on, off, inp, live=True)[0]['violations']
  # ...but the launch creep from a 2.9 m initial hold is not a later rest
  on, off, inp = _launch(n=600)
  inp['v'][180:220] = 0.06
  inp['v'][220:] = 0.4
  inp['gap'][:] = 2.9
  row = _lc(on, off, inp, live=True)[0]
  assert row['rest_min'] is None and 'L6' not in row['violations']
  # P2-6: a disengage at 2 s ends the window: the manual 2.9 m rest after it (and a later re-engage) is not the launch's
  on, off, inp = _launch(n=600)
  inp['v'][180:300] = 0.4
  inp['act'][200:400] = 0
  inp['gap'][300:] = 2.9
  assert 'L6' not in _lc(on, off, inp, live=True)[0]['violations']
  # ...and a takeover 20 ms after the flag-off clear does not prove the flag-on arm late (unjudged, not L5)
  on, off, inp = _launch(clr_on=399, clr_off=100, n=600)
  inp['gas'][103:] = 1
  c = _lc(on, off, inp)
  assert all('L5' not in x['violations'] for x in c) and any('unjudged' in n for x in c for n in x['notes'])


def test_launch_jerk_needs_imu_coverage(monkeypatch):
  # P2-7: a 0.9 s IMU hole is not a jerk breach: unchecked, not L4
  on, off, inp = _launch(n=600)
  inp['v'][150:] = 0.5
  ti = np.r_[np.arange(0.0, 1.10, 0.01), np.arange(2.0, 6.0, 0.01)]
  monkeypatch.setattr(D, 'body_accel', lambda R, t0: (ti, ti.copy()))
  row = _lc(on, off, inp, live=True, R=dict(imu=np.ones((2, 4)), calib=np.ones((1, 4))))[0]
  assert row['jerk'] is None and 'jerk unchecked (IMU gap)' in row['notes']
  ti = np.arange(0.0, 6.0, 0.01)
  monkeypatch.setattr(D, 'body_accel', lambda R, t0: (ti, ti.copy()))
  row = _lc(on, off, inp, live=True, R=dict(imu=np.ones((2, 4)), calib=np.ones((1, 4))))[0]
  assert row['jerk'] == pytest.approx(1.0, abs=0.1)
