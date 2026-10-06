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
  for head in ('## Stops', '## Bookmarks', '## Release-end races', '## Trial E3', '## Trial LINE'):
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
