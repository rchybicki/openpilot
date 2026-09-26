import json

import numpy as np
import pytest

from openpilot.tools.stopping.review import kcs1_reps as K
from openpilot.tools.stopping.review import kcs2_gates as G
from openpilot.tools.stopping.review.test_kcs1_reps import DT, simulate


class Lag:
  """gain x a first-order lag (tau) of the delayed command. simulate() computes `gain * command` once per 10 ms plant
  step, so this object stands in for its float gain."""
  def __init__(self, gain, tau):
    self.gain, self.k, self.y = gain, DT / tau, 0.0

  def __mul__(self, u):
    self.y += (self.gain * u - self.y) * self.k
    return self.y


def rep(man, lamp=None, delay=0.19, gain=0.95, tau=0.09, **kw):
  """One synthetic rep through the extractor on the measured engaged plant (dead time 0.19 s, first-order lag 0.09 s,
  gain 0.95; ANALYSIS_2). simulate() leaves the tcs13 brake light off; `lamp` holds it at a value."""
  streams, meta, _, _ = simulate(man, delay=delay, gain=Lag(gain, tau), **kw)
  if lamp is not None:
    streams['tcs13']['BrakeLight'] = np.full(len(streams['tcs13']['t']), lamp)
  (rec, series), = K.analyze(streams, meta)[0]
  return K.clean(rec), series


@pytest.fixture(scope='module')
def clean():
  return {man: rep(man) for man in G.ROLES}


def copy(rec):
  return json.loads(json.dumps(rec))


def write(d, reps):
  """An extractor output directory (reps.jsonl + series/) holding copies of synthetic reps as distinct reps."""
  (d / 'series').mkdir(parents=True)
  with open(d / 'reps.jsonl', 'w') as fh:
    for i, (rec, z) in enumerate(reps):
      rid = f"{rec['id']}_{d.name}{i}"
      rec = {**rec, 'id': rid, 'route': f"{rec['route']}_{d.name}{i}", 'series_file': f'series/{rid}.npz'}
      np.savez_compressed(d / rec['series_file'], **z)
      fh.write(json.dumps(rec) + '\n')
  return d


@pytest.mark.parametrize('man', G.GATED)
def test_clean_engaged_plant_passes_every_gate_with_the_brake_light_off(clean, man):
  rec, z = clean[man]
  assert rec['valid_for_fit'], rec['failed_checks']
  r = G.gate_rep(rec, z)
  seg, spec = rec['segments'][-1], K.MAN[man][2][-1]
  start = seg['t_start'] + G.SETTLE_S + K.SLOPE_S if spec.jerk is None else seg['ramp']['t_arrive'] + G.SETTLE_S
  assert r['measured'] and r['passed'], r
  assert r['lamp_on_frac'] == 0.0                                                   # the lamp is reported, never gated (C1)
  assert r['level'] == spec.accel and r['start'] == pytest.approx(start) and r['window'][0] >= start
  assert r['window'][1] <= rec['terminal']['t_stop'] and r['v_window'][0] <= G.V_BAND and r['v_window'][1] >= G.V_END
  assert r['nonneg_frames'] == 0 and r['a_stop_tol'] == pytest.approx(G.A_STOP_TOL)
  assert r['a_stop'] == rec['terminal']['decel_at_stop']['imu_0p1s'] == pytest.approx(0.95 * r['level'], abs=G.A_STOP_TOL)
  if man == 'N':   # the measured plant leaves N 4-5 frames (< A_MIN_S) at 0.15-0.2 m/s: (a) not applicable, judged on c, d, e
    assert not r['a_applicable'] and r['window_s'] < G.A_MIN_S and set(r['gates']) == {'c', 'd', 'e'}
  else:
    assert r['a_applicable'] and r['window_s'] >= G.A_MIN_S and set(r['gates']) == {'a1', 'a2', 'c', 'd', 'e'}
    assert r['a_mean'] <= G.LEVEL_FRAC * r['level'] and r['a_max'] <= G.WORST_FRAC * r['level']


@pytest.mark.parametrize('man', ['P', 'L'])
def test_a_step_from_cruise_is_read_after_the_brake_build(clean, man):
  """C4: with dead time 0.19 s and lag 0.09 s a trailing window that opens at the edge is still inside the build 0.5 s
  later; a step window starts 0.8 s after the edge, when that window has closed. (For held P/L, (a) reads only the
  floor band below V_BAND; the start also bounds the K and P fade windows.)"""
  rec, z = clean[man]
  thr = G.LEVEL_FRAC * K.MAN[man][2][-1].accel
  early, late = (G.realised(z, G.frames(rec, z, rec['segments'][-1]['t_start'] + dt)[0][:5]) for dt in (G.SETTLE_S, G.SETTLE_S + K.SLOPE_S))
  assert early.max() > thr + 0.03 and late.max() < thr
  assert G.gate_rep(rec, z)['start'] == pytest.approx(rec['segments'][-1]['t_start'] + 0.8)


def test_the_brake_light_does_not_gate(clean):
  """C1: TCS13 BrakeLight follows a command threshold near -0.55; on or off, the gates are the same."""
  rec, z = rep('I', lamp=1.0)
  on, off = G.gate_rep(rec, z), G.gate_rep(*clean['I'])
  assert on['lamp_on_frac'] == 1.0 and off['lamp_on_frac'] == 0.0 and on['gates'] == off['gates'] and on['passed']


def test_a_push_that_removes_the_braking_after_the_ease_fails_a_c_and_d():
  """+0.5 below 2.5 m/s: -1.0 still brakes (-0.4), the ease to -0.5 leaves +0.05; the stall rule then fires."""
  rec, z = rep('M', gain=0.9, push=0.5)
  assert rec['valid_for_fit'] and rec['label'] == 'stalled', rec['failed_checks']   # a counted rep: it fails, it is not excluded
  r = G.gate_rep(rec, z)
  assert r['measured'] and not r['passed']
  assert [r['gates'][k] for k in ('a1', 'a2', 'c', 'd')] == [False] * 4, r['gates']
  assert r['a_max'] >= 0.0 and r['nonneg_first']['v'] > G.V_END and r['stall_logged']


def test_a_stall_alone_fails_d():
  """-0.03 net after the ease: still slowing (no a >= 0) but by less than STALL_DV in STALL_T, so the stall rule fires."""
  rec, z = rep('M', gain=0.9, push=0.42)
  r = G.gate_rep(rec, z)
  assert rec['label'] == 'stalled' and r['measured'] and not r['gates']['d'] and r['gates']['c'] and not r['passed'], r['gates']


def test_a1_mean_and_a2_worst_frame_in_the_floor_band(clean):
  """(a) over the frames at <= V_BAND: a sustained 0.1 loss fails the mean (a1) only; a 0.5 s 0.2 dip fails the worst
  frame (a2) only; a 0.3 dip above V_BAND (held L at the cruise) is outside the band."""
  rec, z = clean['L']
  base = G.gate_rep(rec, z)
  first = z['whl__t'][np.nonzero((z['whl__t'] >= base['start']) & (z['whl__mean'] <= G.V_BAND))[0][0]]
  assert base['window'][0] == pytest.approx(first) and base['window'][0] > base['start'] + 1.0

  def shifted(dx, t_a, t_b):
    return G.gate_rep(rec, {**z, 'body__long': np.where((z['body__t'] >= t_a) & (z['body__t'] <= t_b), z['body__long'] + dx, z['body__long'])})
  r = shifted(0.1, base['window'][0] - 1.0, base['window'][1] + 1.0)
  assert (r['gates']['a1'], r['gates']['a2']) == (False, True) and r['a_mean'] > G.LEVEL_FRAC * r['level'], r
  mid = 0.5 * sum(base['window'])
  r = shifted(0.2, mid - 0.25, mid + 0.25)
  assert (r['gates']['a1'], r['gates']['a2']) == (True, False) and abs(r['a_max_t'] - mid) < 0.4, r
  r = shifted(0.3, base['start'], base['window'][0] - 0.5)
  assert r['gates'] == base['gates'] and r['a_max'] == base['a_max']


def test_a_and_the_fade_read_the_imu_not_the_abs_slope(clean):
  """C3/C5: the ABS wheel slope reads 0.1-0.26 shallow below ~0.5 m/s; a 0.3 m/s^2 under-read there changes neither (a)
  nor the K fade; the window end still follows the wheel speed."""
  rec, z = clean['L']
  base = G.gate_rep(rec, z)
  low = z['whl__mean'] < 0.5
  abs_band = {**z, 'whl__slope_0p3s': np.where(low, z['whl__slope_0p3s'] + 0.3, z['whl__slope_0p3s'])}
  r = G.gate_rep(rec, abs_band)
  assert r['passed'] and r['a_max'] == base['a_max'] and r['window'] == base['window'] and r['wheel_max'] > G.WORST_FRAC * r['level']
  t_b = z['whl__t'][np.nonzero((z['whl__t'] <= rec['terminal']['t_stop']) & (z['whl__mean'] >= G.V_END))[0][-1]]
  assert base['window'][1] == pytest.approx(t_b)

  k_rec, kz = clean['K']
  items = [(f'k{i}', k_rec, kz) for i in range(4)]
  kp = G.k_vs_p(items, [(f'p{i}', *clean['P']) for i in range(4)])
  k_abs = {**kz, 'whl__slope_0p3s': np.where(kz['whl__mean'] < 0.5, kz['whl__slope_0p3s'] + 0.3, kz['whl__slope_0p3s'])}
  assert kp['K_verdict'] == 'holds' and G.k_vs_p([('abs', k_rec, k_abs)] + items[1:], [])['K'][0]['fade'] == kp['K'][0]['fade']


def test_imu_gap_offset_or_stop_missing_is_unmeasured(clean):
  rec, z = clean['L']
  w = G.gate_rep(rec, z)['window']
  for t_hole in (0.5 * sum(w), rec['terminal']['t_stop'] - 0.12):                   # in the (a) window; before the stop ((e))
    keep = np.abs(z['body__t'] - t_hole) > 0.1                                      # a 0.2 s hole in the IMU
    r = G.gate_rep(rec, {**z, 'body__t': z['body__t'][keep], 'body__long': z['body__long'][keep]})
    assert not r['measured'] and 'IMU gap' in r['reason']
  no_offset = copy(rec)
  no_offset['grade']['imu_offset'] = None
  r = G.gate_rep(no_offset, z)
  assert not r['measured'] and 'IMU offset' in r['reason'] and G.a_stop(no_offset) is None
  no_stop = copy(rec)
  no_stop['terminal']['t_stop'] = None
  r = G.gate_rep(no_stop, z)
  assert not r['measured'] and set(r['unmeasured']) == {'a', 'e'} and 'no pulse stop' in r['reason'] and r['gates']['c']


def test_e_reads_the_imu_a_stop_within_max_of_0p07_and_10_percent(clean):
  """C2: (e) is the IMU a_stop (imu_0p1s); the ABS slope at the stop is reported only. The tolerance is 0.07 up to
  |level| 0.7 and 10 % of |level| beyond (0.10 at -1.0)."""
  rec, z = clean['P']
  rec = copy(rec)
  rec['terminal']['decel_at_stop']['wheel_slope_0p3s'] = -0.3
  r = G.gate_rep(rec, z)
  assert r['gates']['e'] and r['a_stop_wheel'] == -0.3 and r['a_stop_tol'] == pytest.approx(0.07)
  rec['terminal']['decel_at_stop']['imu_0p1s'] = -0.7 + 0.075
  assert G.gate_rep(rec, z)['gates']['e'] is False
  b, bz = rep('B')
  for x, ok in ((-1.095, True), (-0.905, True), (-1.105, False), (-0.895, False)):
    b = copy(b)
    b['terminal']['decel_at_stop']['imu_0p1s'] = x
    r = G.gate_rep(b, bz)
    assert r['a_stop_tol'] == pytest.approx(0.10) and r['gates']['e'] is ok, x


@pytest.mark.parametrize('delay', [0.19, 0.45])
def test_a_short_or_empty_a_window_makes_a_not_applicable(delay):
  """N on the measured plant keeps 4-5 frames (< A_MIN_S) after its ease; with a 0.45 s delay it is below 0.15 m/s
  before the window starts. Either way (a) is n/a (not unmeasured): the rep is measured and judged on c, d, e."""
  rec, z = rep('N', delay=delay)
  r = G.gate_rep(rec, z)
  assert rec['valid_for_fit'] and r['measured'] and not r['a_applicable'] and r['window_s'] < G.A_MIN_S
  assert set(r['gates']) == {'c', 'd', 'e'} and r['passed'] and (r['frames'] == 0) == (delay == 0.45) == (r['window'] is None)
  one = {'id': rec['id'], 'counted': True, **r}
  assert G.status([one] * 4)['status'] == 'PASS'
  res = {'roles': {m: m for m in G.ROLES}, 'reps': [{**one, 'role': 'N', 'label': rec['label']}], 'excluded': [], 'ignored_maneuvers': {},
         'status': {m: G.status([one] * 4) for m in G.GATED}, 'k_vs_p': G.k_vs_p([], []), 'day_control': G.day_control([], []),
         'branch': 'x', 'decision': []}
  assert ' n/a)' in G.report(res, {'script_sha256': 'x', 'inputs': [], 'kcs1': 'x'}).split(rec['id'])[1].split('\n')[0]
  bad = copy(rec)
  bad['terminal']['decel_at_stop']['imu_0p1s'] = -0.5 + 0.08
  assert G.gate_rep(bad, z)['passed'] is False


def test_nothing_after_the_pulse_stop_is_read(clean):
  """The fast-cycle build: no driver brake and no hold record, and the car drives off 0.3 s after the hold reaches -0.70.
  Every gate and the K/P values stop at the pulse stop, so a drive-off after it changes nothing."""
  for man in ('L', 'I', 'K'):
    rec, z = clean[man]
    t_stop = rec['terminal']['t_stop']
    fast = copy(rec)
    fast['hold'] = None
    fast['terminal'].update(flag_to_stopreq_s=None, displacement=None)
    off = dict(z)
    for s in ('whl', 'body', 'tcs13', 'esp12', 'car', 'pul'):
      after = z[f'{s}__t'] > t_stop
      for k in [k for k in z if k.startswith(f'{s}__') and not k.endswith('_t') and len(z[k]) == len(after)]:
        off[k] = np.where(after, 1.0 + np.abs(z[k]), z[k])                          # moving, accelerating, lamp on, no brake
    off['car__brake'] = np.zeros(len(z['car__t']))
    if man == 'K':
      assert G.fade_rep(fast, off) == G.fade_rep(rec, z)
    else:
      assert G.gate_rep(fast, off) == G.gate_rep(rec, z)


def test_nan_imu_is_never_interpolated(clean):
  """Review 20260926-203630 #1: NaN IMU samples at intact timestamps (series: body__long NaN 7-9 s; extractor: the
  accelerometer NaN 7-9 s after t0, which it still counts) are a gap, not a measurement: (a) unmeasured, the rep too."""
  rec, z = clean['L']
  nan = {**z, 'body__long': np.where((z['body__t'] > 7) & (z['body__t'] < 9), np.nan, z['body__long'])}
  streams, meta, _, _ = simulate('L', delay=0.19, gain=Lag(0.95, 0.09))
  streams['imu']['z'] = np.where((streams['imu']['t'] > 112) & (streams['imu']['t'] < 114), np.nan, streams['imu']['z'])
  (x_rec, x_z), = K.analyze(streams, meta)[0]
  x_rec = K.clean(x_rec)
  assert x_rec['valid_for_fit'] and np.isnan(x_z['body__long']).sum() == 200
  for r in (G.gate_rep(rec, nan), G.gate_rep(x_rec, x_z)):
    assert not r['measured'] and set(r['unmeasured']) == {'a'} and 'IMU gap' in r['reason'] and all(r['gates'].values()), r
  assert G.fade_rep(rec, nan)['measured'] is False


def test_a_cut_pulse_stream_is_not_a_stop():
  """Review 20260926-203630 #2: pulses cut 2 s before the real stop move the extractor's stop 2 s early (at ~1 m/s) and
  it still counts the rep; with no pulses the stop comes from the flag. Neither is a trusted stop: (a) and (e) are
  unmeasured, (c) runs to the standstill flag."""
  streams, meta, _, _ = simulate('L', delay=0.19, gain=Lag(0.95, 0.09))
  (rec, _), = K.analyze(streams, meta)[0]
  rec = K.clean(rec)
  cut = rec['t0_ns'] / 1e9 + rec['terminal']['t_stop'] - 2.0
  for pul, source in (({k: v[streams['pul']['t'] < cut] for k, v in streams['pul'].items()}, 'pulses'),
                      ({k: v[:0] for k, v in streams['pul'].items()}, 'flag')):
    (x_rec, z), = K.analyze({**streams, 'pul': pul}, meta)[0]
    x_rec = K.clean(x_rec)
    assert x_rec['valid_for_fit'] and x_rec['terminal']['stop_source'] == source
    if source == 'pulses':   # 2 s early, at ~1 m/s
      assert x_rec['terminal']['t_stop'] == pytest.approx(rec['terminal']['t_stop'] - 2.0, abs=0.05)
    r = G.gate_rep(x_rec, z)
    assert not r['measured'] and set(r['unmeasured']) == {'a', 'e'} and r['gates'] == {'c': True, 'd': True}, r
    assert r['c_end'] == x_rec['terminal']['t_flag'] and G.fade_rep(x_rec, z)['measured'] is False
    assert ('pulse gap' if len(pul['t']) else 'stop from the flag') in r['reason']


def test_an_imu_gap_keeps_the_failures_the_other_gates_prove(tmp_path, clean):
  """Review 20260926-203630 #3: a counted M that gains speed under the brake and stalls, with 0.3 s of its IMU missing
  mid-window: (a) is unmeasured, but (c) and (d) prove the loss, so the rep and M fail (J passes -> -0.60)."""
  rec, z = rep('M', gain=0.9, push=0.5)
  w = G.gate_rep(rec, z)['window']
  keep = np.abs(z['body__t'] - 0.5 * sum(w)) > 0.15
  bad = {**z, 'body__t': z['body__t'][keep], 'body__long': z['body__long'][keep]}
  r = G.gate_rep(rec, bad)
  assert r['measured'] and not r['passed'] and set(r['unmeasured']) == {'a'} and not r['gates']['c'] and not r['gates']['d'], r
  res = G.evaluate(G.load([write(tmp_path / 'gap', [clean[m] for m in G.ROLES for _ in range(4)] + [(rec, bad)])]), [])
  assert res['status']['M']['status'] == 'FAIL' and len(res['status']['M']['failed_reps']) == 1 and res['branch'] == 'A_FLOOR -0.60'


@pytest.mark.parametrize('man', ['L', 'N', 'M'])
def test_a_fast_cycle_rep_gates_like_a_brake_ended_rep(clean, man):
  """The KCS2 build counts 0.3 s after the full hold and drives off (no driver brake): same motion to the stop, so the
  same gate result as the brake-ended rep."""
  rec, z = rep(man, fast=True)
  assert rec['valid_for_fit'] and rec['hold']['end'] == 'launch' and rec['launch'] is not None, rec['failed_checks']
  assert G.gate_rep(rec, z) == G.gate_rep(*clean[man]) and G.gate_rep(rec, z)['passed']


def test_status_is_per_rep():
  ok = {'id': 'ok', 'counted': True, 'measured': True, 'passed': True}
  bad, unmeasured = {**ok, 'id': 'bad', 'passed': False}, {**ok, 'id': 'u', 'measured': False}
  assert G.status([ok] * 4)['status'] == 'PASS'
  assert G.status([ok] * 3 + [unmeasured] * 3) == {'status': 'INSUFFICIENT', 'counted': 6, 'measured': 3, 'passed': 3, 'failed_reps': []}
  assert G.status([ok] * 5 + [bad]) == {'status': 'FAIL', 'counted': 6, 'measured': 6, 'passed': 5, 'failed_reps': ['bad']}
  assert G.status([bad])['status'] == 'FAIL'                      # one failing rep decides, whatever the count
  assert G.status([ok] * 4, ['aborted']) == {'status': 'FAIL', 'counted': 4, 'measured': 4, 'passed': 4, 'failed_reps': ['aborted']}


PASS = dict.fromkeys(G.GATED, 'PASS')


@pytest.mark.parametrize('changes,branch', [
  ({}, 'A_FLOOR -0.50'),
  ({'M': 'FAIL'}, 'A_FLOOR -0.60'),
  ({'M': 'FAIL', 'N': 'FAIL', 'L': 'FAIL'}, 'A_FLOOR -0.60'),
  ({'M': 'FAIL', 'J': 'FAIL'}, 'ratchet'),
  ({'M': 'FAIL', 'J': 'INSUFFICIENT'}, 'INSUFFICIENT'),
  ({'I': 'FAIL'}, 'NOT COVERED'),
  ({'L': 'FAIL', 'M': 'INSUFFICIENT'}, 'NOT COVERED'),
  ({'N': 'INSUFFICIENT'}, 'INSUFFICIENT'),
  ({'J': 'FAIL'}, 'A_FLOOR -0.50'),
])
def test_decision_branch(changes, branch):
  b, lines = G.decide({**PASS, **changes})
  assert b == branch, lines
  if branch == 'A_FLOOR -0.60':
    assert any('UNVALIDATED' in x and '0.8 m/s' in x and 'held -0.60' in x for x in lines)
    assert any(x.startswith('M FAIL') for x in lines)
  if changes == {'J': 'FAIL'}:
    assert any(x.startswith('Warning: J') for x in lines)


@pytest.mark.parametrize('k,p,text,branch', [
  ('fades', 'holds', 'which the floor prevents', 'A_FLOOR -0.50'),
  ('fades', 'fades', 'level/speed/site effect at -0.7: stop and re-plan', 'RE-PLAN'),
  ('holds', 'holds', '2129 not reproduced', 'A_FLOOR -0.50'),
  ('holds', 'fades', 'P fades while K holds', 'RE-PLAN'),
  ('INSUFFICIENT', 'holds', 'K/P INSUFFICIENT', 'A_FLOOR -0.50'),
])
def test_k_p_reading(k, p, text, branch):
  """Review 20260926-203630 #4: 'stop and re-plan' overrides the floor reading (here I, M, N, L all pass)."""
  b, lines = G.decide(PASS, {'K_verdict': k, 'P_verdict': p})
  assert b == branch and sum(text in x for x in lines) == 1, lines
  assert (b == 'RE-PLAN') == lines[0].startswith('No A_FLOOR') and not any(x.startswith('A_FLOOR = ') for x in lines if b == 'RE-PLAN')


def test_k_fade_against_p(clean):
  (_, k_rec, kz), (_, p_rec, pz) = ('k', *clean['K']), ('p', *clean['P'])
  kp = G.k_vs_p([(f'k{i}', k_rec, kz) for i in range(4)], [(f'p{i}', p_rec, pz) for i in range(4)])
  assert (kp['K_verdict'], kp['P_verdict']) == ('holds', 'holds') and abs(kp['K_minus_P']) < 0.03
  k0 = kp['K'][0]
  assert k0['window'][0] == pytest.approx(k_rec['segments'][2]['t_start'] + G.SETTLE_S + K.SLOPE_S) and k0['fade'] < 0.05
  assert kp['P'][0]['v_start'] <= kp['v_top'] + 1e-9 and kp['P'][0]['window'][0] >= p_rec['segments'][0]['t_start'] + G.SETTLE_S + K.SLOPE_S
  # a 0.2 m/s^2 rise of the IMU in the second half of the K window, after its deepest value: the 2129 fade
  mid = np.mean(k0['window'])
  faded = {**kz, 'body__long': np.where(kz['body__t'] > mid, kz['body__long'] + 0.2, kz['body__long'])}
  kp = G.k_vs_p([('faded', k_rec, faded)] + [(f'k{i}', k_rec, kz) for i in range(3)], [(f'p{i}', p_rec, pz) for i in range(4)])
  assert kp['K_verdict'] == 'fades' and 0.2 - 0.01 <= kp['K'][0]['fade'] <= 0.2 + 0.05 and kp['K'][0]['fade_t'] > mid   # + the clean spread
  assert G.fade_verdict(kp['K'][1:]) == 'INSUFFICIENT'


def test_day_control_reads_the_imu_a_stop():
  """C2: the IMU medians decide; the wheel values (reported) point the other way here."""
  def recs(*xs, wheel=-0.4):
    return [{'terminal': {'decel_at_stop': {'imu_0p1s': x, 'wheel_slope_0p3s': wheel}}, 'grade': {'imu_offset': 0.0}} for x in xs]
  d = G.day_control(recs(-0.48, -0.47, -0.49, -0.50, wheel=-0.30), recs(-0.45, -0.44, -0.46, -0.45, wheel=-0.45))
  assert d['verdict'] == 'holds' and d['difference'] == pytest.approx(-0.035) and d['L_wheel_median'] - d['A_wheel_median'] > G.CONTROL_TOL
  d = G.day_control(recs(-0.48, -0.47, -0.49, -0.50), recs(-0.40, -0.41, -0.39, -0.40))
  assert d['verdict'] == 'FAILS' and d['difference'] == pytest.approx(-0.085) and d['L_wheel_median'] == d['A_wheel_median']
  assert G.day_control(recs(-0.48, -0.47, -0.49), recs(-0.45, -0.44, -0.46, -0.45))['verdict'] == 'INSUFFICIENT'
  no_offset = recs(-0.48, -0.47, -0.49, -0.50)
  no_offset[0]['grade']['imu_offset'] = None
  assert G.day_control(no_offset, recs(-0.45, -0.44, -0.46, -0.45))['verdict'] == 'INSUFFICIENT'


def test_load_rejects_a_repeated_rep(tmp_path, clean):
  """C6: the same directory twice, or one rep re-extracted under another id, would count twice."""
  d = write(tmp_path / 'a', [clean['M']] * 2)
  assert len(G.load([d])) == 2
  with pytest.raises(ValueError, match='repeated rep id'):
    G.load([d, d])
  rec = json.loads((d / 'reps.jsonl').read_text().splitlines()[0])
  (tmp_path / 'b').mkdir()
  (tmp_path / 'b' / 'reps.jsonl').write_text(json.dumps({**rec, 'id': rec['id'] + '_again'}) + '\n')
  with pytest.raises(ValueError, match='repeated rep route, t0_ns'):
    G.load([d, tmp_path / 'b'])


def test_an_uncounted_rep_that_loses_the_braking_before_its_abort_fails_its_maneuver(tmp_path, clean):
  """C7: the 'speed' abort fires when the car gains speed under a braking command; that rep is not counted, but its
  a >= 0 above 0.15 m/s before the abort fails the maneuver. A clean abort stays excluded only."""
  bad, bz = rep('M', gain=0.9, push=0.5)
  t_nn = G.gate_rep(bad, bz)['nonneg_first']['t']
  bad = {**copy(bad), 'label': 'aborted(speed)', 'device_label': 'aborted(speed)', 'valid_for_fit': False, 't_abort': t_nn + 1.0}
  good = {**copy(clean['M'][0]), 'label': 'aborted(lead)', 'valid_for_fit': False, 't_abort': 5.0}
  early = {**copy(bad), 't_abort': t_nn - 0.1}                                     # aborted before the push: clean
  kcs1 = write(tmp_path / 'kcs1', [rep('A')] * 4)
  base = [clean[m] for m in G.ROLES for _ in range(4)]
  res = G.evaluate(G.load([write(tmp_path / 'ok', base + [(good, clean['M'][1]), (early, bz)])]), G.load([kcs1]))
  assert res['status']['M']['status'] == 'PASS' and res['branch'] == 'A_FLOOR -0.50', res['decision']
  assert [x['c_failed'] for x in res['excluded']] == [False, False] and all('(c) clean' in x['reason'] for x in res['excluded'])
  res = G.evaluate(G.load([write(tmp_path / 'bad', base + [(bad, bz)])]), G.load([kcs1]))
  (x,) = res['excluded']
  assert x['c_failed'] and x['c_check']['c_end'] == pytest.approx(t_nn + 1.0) and '(c) FAILS' in x['reason']
  assert res['status']['M'] == {'status': 'FAIL', 'counted': 4, 'measured': 4, 'passed': 4, 'failed_reps': [x['id']]}
  assert res['branch'] == 'A_FLOOR -0.60'


def test_an_unknown_record_is_excluded(tmp_path, clean):
  """C9: measure() returns 'unknown' (rep starts before the log) without series_file, valid_for_fit or failed_checks."""
  d = write(tmp_path / 'kcs2', [clean[m] for m in G.ROLES for _ in range(4)])
  unk = {'id': 'x_M3_1', 'route': 'x', 'plan': 'KCS2', 'plan_source': 'table', 'commits': [], 'maneuver': 'M', 'rep': 3, 'attempt': 1,
         'device_label': 'counted', 'counted_on_device': True, 'label': 'unknown', 'times_from': 'first banner frame',
         'reason': 'rep starts before the log'}
  with open(d / 'reps.jsonl', 'a') as fh:
    fh.write(json.dumps(unk) + '\n')
  res = G.evaluate(G.load([d]), [])
  (x,) = res['excluded']
  assert x['id'] == 'x_M3_1' and x['label'] == 'unknown' and 'rep starts before the log' in x['reason'] and 'not checked' in x['reason']
  assert res['status']['M']['status'] == 'PASS'


def test_directories_end_to_end(tmp_path, clean):
  """4 clean reps of every role and one short hold -> -0.50, the short hold listed; an extra M push rep in a second
  directory -> M FAIL, J PASS -> -0.60. The KCS1 A reps are the day control."""
  reps = [clean[m] for m in G.ROLES for _ in range(4)] + [rep('L', hold_s=0.5)]
  kcs2 = write(tmp_path / 'kcs2', reps)
  kcs1 = write(tmp_path / 'kcs1', [rep('A')] * 4 + [rep('B')])
  res = G.main([str(kcs2), '--kcs1', str(kcs1), '--output', str(tmp_path / 'out')])
  assert res['branch'] == 'A_FLOOR -0.50', res['decision']
  assert all(res['status'][m] == {'status': 'PASS', 'counted': 4, 'measured': 4, 'passed': 4, 'failed_reps': []} for m in G.GATED)
  assert [x['label'] for x in res['excluded']] == ['short-hold'] and 'hold_min' in res['excluded'][0]['reason']
  assert (res['k_vs_p']['K_verdict'], res['k_vs_p']['P_verdict'], res['day_control']['verdict']) == ('holds', 'holds', 'holds')
  saved = json.loads((tmp_path / 'out' / 'gates.json').read_text())
  assert saved['script_sha256'] == G.sha256(G.__file__) and saved['branch'] == res['branch']
  assert 'short-hold' in (tmp_path / 'out' / 'GATES.md').read_text()
  with pytest.raises(FileExistsError):
    G.main([str(kcs2), '--kcs1', str(kcs1), '--output', str(tmp_path / 'out')])

  more = write(tmp_path / 'more', [rep('M', gain=0.9, push=0.5)])
  res = G.main([str(kcs2), str(more), '--kcs1', str(kcs1), '--output', str(tmp_path / 'out2')])
  assert res['branch'] == 'A_FLOOR -0.60' and res['status']['M']['status'] == 'FAIL' and len(res['status']['M']['failed_reps']) == 1

  # dry-run aliases: KCS1 A read as L (gated at its own -0.5), B as P
  res = G.main([str(kcs1), '--kcs1', str(kcs1), '--alias', 'L=A', '--alias', 'P=B', '--output', str(tmp_path / 'out3')])
  assert res['status']['L']['status'] == 'PASS' and res['status']['P']['counted'] == 1 and res['day_control']['difference'] == 0.0
  assert [r['level'] for r in res['reps'] if r['role'] == 'P'] == [-1.0]
