import json

import numpy as np
import pytest
import zstandard

from openpilot.selfdrive.controls.lib.identification_hook import STALL_DV, STALL_T
from openpilot.tools.stopping.review import kcs1_reps as K

DT = 0.01


def wheel_samples(t_end, v_of, rng, t0=100.0, noise=0.003):
  """50 Hz receive-batch times (0-8 ms jitter) and the 4-wheel mean of 0.03125 km/h-quantised noisy speeds (m/s)."""
  t = np.arange(t0 + 0.003, t_end, 0.02)
  t = t + rng.uniform(0.0, 0.008, len(t))
  kph = np.round((v_of(t)[None, :] + rng.normal(0.0, noise, (4, len(t)))) * 3.6 / 0.03125) * 0.03125
  return t, np.clip(kph, 0.0, None)


def test_trailing_slope_and_gain():
  rng = np.random.default_rng(1)
  t = np.sort(100.0 + np.arange(0, 5, 0.02) + rng.uniform(0, 0.008, 250))
  v = 5.0 - 0.8 * (t - 100.0)
  s = K.trailing(t, v, K.SLOPE_S, True)
  assert np.nanmax(np.abs(s[20:] + 0.8)) < 1e-9 and np.isnan(s[:3]).all()
  g = K.gain(t, v, 101.0, 104.0, -1.0)
  assert g['realized'] == pytest.approx(-0.8) and g['gain'] == pytest.approx(0.8) and g['window'] == [103.0, 104.0]
  assert K.gain(t, v, 101.0, 102.9, -1.0) is None                       # held < 2 s
  assert K.gain(t, v, 101.0, 102.99, -1.0) is not None                  # a 2 s segment one send period short on the wire
  assert K.gain(t, v, 101.0, 104.0, 0.0)['gain'] is None                # zero command: realized only


@pytest.mark.parametrize('delay,a0,a1', [(0.45, 0.0, -0.9), (0.30, -0.9, -0.27), (0.60, 0.0, -0.45)])
def test_onset_recovers_delay(delay, a0, a1):
  """Known delay on quantised, jittered, noisy 50 Hz wheel speeds: recovered within +0..60 ms (the trailing-window
  detection lag of the frozen rule), in the commanded direction only."""
  rng = np.random.default_rng(7)
  t_edge = 104.0

  def v_of(t):
    return 6.0 + a0 * (t - 100.0) + (a1 - a0) * np.clip(t - t_edge - delay, 0.0, None)

  t, kph = wheel_samples(108.0, v_of, rng)
  v = kph.mean(0) / 3.6
  s = K.trailing(t, v, K.SLOPE_S, True)
  sd = K.pre_sd(t, s, t_edge - 2.0, t_edge) if a0 == 0.0 else K.pre_sd(t, s, 100.0, 102.0)
  direction = 1.0 if a1 > a0 else -1.0
  r = K.onset(t, v, s, t_edge, 100.0, sd, direction, t_edge + K.MAX_DELAY_S)
  assert r['ref'] == pytest.approx(a0, abs=0.05)
  assert delay <= r['delay_s'] <= delay + 0.06
  assert K.onset(t, v, s, t_edge, 100.0, sd, -direction, t_edge + K.MAX_DELAY_S)['delay_s'] is None


def test_onset_none_without_response():
  rng = np.random.default_rng(3)
  t, kph = wheel_samples(106.0, lambda t: np.full(len(t), 5.56), rng)
  v = kph.mean(0) / 3.6
  s = K.trailing(t, v, K.SLOPE_S, True)
  assert K.onset(t, v, s, 104.0, 100.0, K.pre_sd(t, s, 102.0, 104.0), -1.0, 106.0)['delay_s'] is None


def test_parse_lines():
  d = K.parse_hook_line('identification hook ACTIVE man=C rep=2 seg=1 reason= floor=-1.0 intent=0 done= v=5.56')
  assert (d['state'], d['man'], d['rep'], d['seg'], d['reason'], d['floor'], d['intent'], d['done']) == ('ACTIVE', 'C', 2, 1, '', -1.0, False, '')
  d = K.parse_hook_line('identification hook ARMED man=C rep=2 seg=2 reason=complete floor=None intent=0 done=C v=0.00')
  assert d['floor'] is None and d['done'] == 'C' and d['reason'] == 'complete'
  d = K.parse_hook_line("identification hook progress saved: {'plan': 'KCS1', 'done': {'B': 1, 'A': 0}}")
  assert d == {'kind': 'progress_saved', 'record': {'plan': 'KCS1', 'done': {'B': 1, 'A': 0}}}
  assert K.parse_hook_line('identification hook constructed: OFF')['kind'] == 'constructed'
  assert K.parse_banner('TEST B 1/6 s1 -1.00')[0] == 'active'
  assert K.parse_banner('TEST E 3/6 s2 -0.30 - 1.4 m/s')[1] == {'man': 'E', 'rep': '3', 'seg': '2', 'cmd': '-0.30', 'v': '1.4'}
  assert K.parse_banner('TEST READY - press = start B 1/6')[0] == 'other'
  assert K.parse_banner('TEST B 1/6 STARTS IN 1.5 s')[0] == 'other'                 # the READY countdown opens no rep
  assert K.parse_banner('TEST B 1/6 STOPPED - hold 3.2 s - BRAKE NOW')[1]['hold'] == '3.2'


def banner_run(t, n, text1, text2=''):
  return [(t + DT * i, text1, text2) for i in range(n)]


def test_segment_reps():
  rows = (banner_run(0.0, 200, 'TEST B 1/6 STARTS IN 1.0 s', 'B: -1.0 to stop; brake = not now')
          + banner_run(2.0, 300, 'TEST B 1/6 s1 -1.00 - 4.0 m/s', 'press = cancel (releases, cruise resumes)')
          + banner_run(5.0, 150, 'TEST B 1/6 STOPPED - hold 0.5 s', 'brake to finish')
          + banner_run(6.5, 300, 'B 1/6 DONE', 'next A 1/6: -0.5 to stop')           # notice frames after the brake frame
          + banner_run(9.5, 100, 'TEST A 1/6 STARTS IN 0.5 s', 'x')
          + banner_run(10.5, 100, 'TEST A 1/6 s1 -0.50 - 5.0 m/s', 'x')
          + banner_run(11.5, 50, 'TEST A 1/6 ABORTED - lead', 'releasing; cruise resumes and can accelerate')
          + banner_run(12.0, 100, 'TEST ARMED - waiting: settling', 'x')
          + banner_run(13.0, 100, 'TEST A 1/6 s1 -0.50 - 2.2 m/s', 'x')
          + banner_run(14.0, 1, 'TEST A 1/6 ABORTED - press', 'finishing the stop; brake to end')
          + banner_run(14.01, 100, 'TEST A 1/6 s1 -0.50 - 1.8 m/s', 'finishing the stop; brake to end')
          + banner_run(15.01, 100, 'TEST LOCKED - fault - HELD', 'brake to end; restart the car')
          + banner_run(16.01, 1, 'A 1/6 NOT COUNTED - press', 'x')
          + banner_run(17.0, 50, 'TEST C 1/6 s1 -1.00 - 5.0 m/s', 'x')
          + banner_run(17.5, 300, 'TEST C 1/6 ABORTED - pedal', 'test mode off; long press distance to arm')
          + banner_run(21.0, 20, 'TEST D 1/6 s1 -1.00 - 5.0 m/s', 'x')
          + banner_run(22.0, 20, 'TEST D 1/6 s1 -1.00 - 4.0 m/s', 'x'))           # 0.8 s banner gap: a new rep
  reps = K.segment_reps(rows)
  assert [(r['man'], r['rep'], r['close']) for r in reps] == [('B', 1, 'done'), ('A', 1, 'banner'), ('A', 1, 'not_counted'),
                                                               ('C', 1, 'driver'), ('D', 1, 'gap'), ('D', 1, 'log_end')]
  facts = [K.banner_facts(r) for r in reps]
  assert facts[0]['start'] == 2.0 and facts[0]['held_t'] == 5.0 and facts[0]['end']['t'] == 6.5 and facts[0]['frames'] == 451
  assert facts[1]['aborted']['reason'] == 'lead' and facts[1]['end'] is None
  assert facts[2]['aborted']['t'] == 14.0 and facts[2]['locked_held'] and facts[2]['end']['reason'] == 'press'
  assert facts[3]['aborted']['reason'] == 'pedal' and facts[3]['frames'] == 51


# ---- synthetic end-to-end rep (the hook's segment walk on a pure-delay plant) ----------------------------------------
def simulate(man, delay=0.45, gain=0.9, hold_s=2.0, seed=0, push=0.0, flag_lag=0.2, override=None):
  """One rep of `man` from 20 km/h cruise; the plant is gain x (sent command `delay` earlier) + `push` below 2.5 m/s.
  The standstill flag (and the hook's HELD) comes `flag_lag` after the wheels stop, as the ABS speed does.
  override = (segment number, seconds, value): the normal chain's `value` passes min() at that segment's start."""
  segs = K.MAN[man][2]
  rng = np.random.default_rng(seed)
  t0, tb, v = 100.0, 105.0, 5.56
  rows, sent, banner, lines = [], [], [], []
  phase, k, seg_start, t_flag, t_brake, intent_n, j, floor, t_zero = 'pre', 0, 0.0, None, None, None, 0, 0.0, None
  hist, stalled, counted, reason = [], False, False, ''
  for n in range(int(40.0 / DT)):
    tn = t0 + n * DT
    first = False
    if phase == 'pre' and tn >= tb - 1e-9:
      phase, seg_start, first = 'active', tn, True
      lines.append((tn + 0.05, f'identification hook ACTIVE man={man} rep=1 seg=1 reason= floor={segs[0].accel} intent=0 done= v={v:.2f}'))
    if phase == 'active':
      t_zero = t_zero if t_zero is not None or v > 0.0 else tn
      if t_zero is not None and tn - t_zero >= flag_lag - 1e-9:
        phase, t_flag = 'held', tn
        lines.append((tn + 0.05, f'identification hook HELD man={man} rep=1 seg={k + 1} reason= floor={segs[k].accel} intent=1 done= v={v:.2f}'))
      else:
        seg = segs[k]
        if k + 1 < len(segs) and ((seg.v_end is not None and v <= seg.v_end) or (seg.t_s is not None and tn - seg_start >= seg.t_s - 1e-9)):
          k, seg_start, first, hist = k + 1, tn, True, []
          lines.append((tn + 0.05, f'identification hook ACTIVE man={man} rep=1 seg={k + 1} reason= floor={segs[k].accel} intent=0 done= v={v:.2f}'))
        if intent_n is None and segs[k].t_s is None and v <= K.INTENT_V[K.PLAN_OF[man]]:
          intent_n = n
    if phase == 'held' and tn >= t_flag + hold_s - 1e-9:
      phase, t_brake = 'done', tn
      counted = hold_s >= K.HOLD_MIN_S
      reason = ('stalled' if stalled else 'complete') if counted else 'short-hold'
      lines.append((tn + 0.05, f'identification hook ARMED man={man} rep=1 seg={k + 1} reason={reason} floor=None intent=0 ' +
                               f'done={man if counted else ""} v=0.00'))
    if phase == 'held' and floor > K.A_HOLD:                    # hold build: deepen at J_HOLD to A_HOLD
      floor = max(floor - K.J_HOLD * DT, K.A_HOLD)
    elif phase == 'active':                                     # the hook's stall rule: deepen at J_HOLD, sticky
      hist = [(t, x) for t, x in hist if t >= tn - STALL_T - 1e-9] + [(tn, v)]
      if (not stalled and segs[k].t_s is None and v < K.STALL_V and hist[0][0] <= tn - STALL_T + 1e-9
              and hist[0][1] - v < STALL_DV):
        stalled, intent_n = True, intent_n if intent_n is not None else n
        lines.append((tn + 0.05, f'identification hook ACTIVE man={man} rep=1 seg={k + 1} reason= floor={floor} intent=1 done= v={v:.2f}'))
      script = segs[k].accel if segs[k].jerk is None else min(max(segs[k].accel, floor - segs[k].jerk * DT), floor + segs[k].jerk * DT)
      floor = min(script, max(floor - K.J_HOLD * DT, K.A_HOLD) if floor > K.A_HOLD else floor) if stalled else script
    cmd = floor if phase in ('active', 'held') else 0.0
    if override and phase == 'active' and k + 1 == override[0] and tn - seg_start < override[1] - 1e-9:
      cmd = min(cmd, override[2])
    if n % 2 == 0:
      sent.append((tn + 0.005, cmd, phase == 'held' and tn >= t_flag + 0.1, 0.0 if phase == 'done' else 1.0, n))
    while j < len(sent) and sent[j][0] <= tn - delay + 1e-9:   # the plant sees the sent command `delay` later
      j += 1
    a = gain * (sent[j - 1][1] if j else 0.0) + (push if v < 2.5 else 0.0) if v > 0.0 else 0.0
    state = 1 if intent_n is None or n <= intent_n else (2 if phase != 'done' else 0)
    rows.append((tn, v, a, cmd, phase, state))
    text = {'pre': (f'TEST {man} 1/6 STARTS IN {max(tb - tn, 0.0):.1f} s', 'x') if tn >= tb - 2.0 else None,
            'active': (f'TEST {man} 1/6 s{k + 1} {cmd:+.2f}' + ('' if first else f' - {v:.1f} m/s'), 'x'),
            'held': (f'TEST {man} 1/6 STOPPED - hold {tn - (t_flag or tn):.1f} s', 'brake to finish'),
            'done': ((f'{man} 1/6 DONE' if counted else f'{man} 1/6 NOT COUNTED - {reason}'), 'x') if t_brake and tn < t_brake + 3.0
            else ('TEST ARMED - waiting: disengaged', 'x')}[phase]
    if text:
      banner.append((tn + 0.004, *text))
    v = max(v + a * DT, 0.0)
  tc, vc, ac, cmdc, ph, st = (np.array([r[i] for r in rows]) for i in range(6))
  zeros, T = np.zeros(len(tc)), tc[-1] + DT

  def can(name, t, **cols):
    return {'t': t, 'dat': np.array([None] * len(t), dtype=object),
            **{f: cols.get(f, np.zeros(len(t))) for f in K.FIELDS[K.CAN[name][2]]}}

  st_t, st_a, st_sr, st_mode, st_n = (np.array([s[i] for s in sent]) for i in range(5))
  sc = can('scc12', st_t, aReqValue=np.round(st_a * 100) / 100, aReqRaw=st_a, StopReq=st_sr.astype(float), ACCMode=st_mode)
  sc['dat'] = np.array([int(i).to_bytes(4, 'little') for i in st_n], dtype=object)
  echo = {**sc, 't': st_t + 0.003}
  upper = np.where(st[(st_n).astype(int)] == 2, 1.0, 3.0)
  tw, kph = wheel_samples(T, lambda t: np.interp(t, tc, vc), rng)
  dist = np.interp(tw, tc, np.concatenate(([0.0], np.cumsum(vc[:-1] * DT))))
  raw = np.floor((dist[None, :] + 0.005 * np.arange(4)[:, None]) / 0.02) % 256    # 0.02 m per count, wheel phases differ
  imu_t = tc + 0.001
  streams = {
    'car': {'t': tc, 'v': vc, 'a': ac, 'v_cruise': np.full(len(tc), 5.56), 'standstill': ((ph == 'held') | (ph == 'done')).astype(float),
            'brake': (ph == 'done').astype(float), 'gas': zeros, 'esp': zeros, 'acc_fault': zeros, 'valid': zeros + 1},
    'lcs': {'t': tc + 0.003, 'state': st.astype(float)},
    'cc': {'t': tc + 0.004, 'enabled': (ph != 'done').astype(float), 'long_active': (ph != 'done').astype(float), 'accel': cmdc,
           'pitch': zeros, 'override': zeros},
    'imu': {'t': imu_t, 'x': np.full(len(tc), K.G), 'y': zeros, 'z': -ac + rng.normal(0, 0.02, len(tc))},
    'gyro': {'t': imu_t, 'x': zeros, 'y': rng.normal(0, 0.002, len(tc)), 'z': zeros},
    'gps': {'t': tc[::10], 'bearing': np.full(len(tc[::10]), 359.0), 'speed': vc[::10], 'fix': np.ones(len(tc[::10])),
            'bearing_acc': np.ones(len(tc[::10]))},
    'calib': {'t': np.array([t0]), 'roll': np.zeros(1), 'pitch': np.zeros(1), 'yaw': np.zeros(1), 'calibrated': np.ones(1)},
    'scc12': sc, 'scc12_echo': echo, 'scc14': can('scc14', st_t, JerkUpperLimit=upper, JerkLowerLimit=np.full(len(st_t), 5.0)),
    'esp12': can('esp12', tc + 0.002, LONG_ACCEL=ac), 'tcs13': can('tcs13', tw), 'tcs15': can('tcs15', tc[::10] + 0.002),
    'whl': can('whl', tw, **{f'WHL_SPD_{w}': kph[i] for i, w in enumerate(K.WHEELS)}),
    'pul': can('pul', tw, **{f'WHL_PUL_{w}': raw[i] * K.PULSE_SCALE for i, w in enumerate(K.WHEELS)}),
  }
  meta = {'route': '00000001--synthetic', 'files': [], 'init': [{'segment': 'x', 'commit': 'c0ffee', 'branch': 'b', 'dirty': False}],
          'banner': banner, 'hook_lines': lines + [(t_brake + 0.06, f"identification hook progress saved: {{'plan': '{K.PLAN_OF[man]}', 'done': {{}}}}")]}
  return K.derive(streams), meta, t_flag, t_brake


@pytest.mark.parametrize('man', ['B', 'C', 'E'])
def test_synthetic_rep_end_to_end(man):
  streams, meta, t_flag, t_brake = simulate(man)
  (rec, series), = K.analyze(streams, meta)[0]
  rec = K.clean(rec)
  json.dumps(rec, allow_nan=False)
  assert (rec['label'], rec['device_label'], rec['label_agrees'], rec['valid_for_fit']) == ('complete', 'complete', True, True), rec['failed_checks']
  assert rec['counted_on_device'] and rec['pre_window']['ok'] and rec['echo_missing'] == 0
  segs = rec['segments']
  assert len(segs) == len(K.MAN[man][2]) and all(s['edge_source'] == 'scc12' and 0.0 <= s['edge_minus_banner_s'] <= 0.02 for s in segs)
  for s in segs:
    assert 0.45 <= s['onset_wheel']['delay_s'] <= 0.51, s                 # pure 0.45 s delay + the detection lag
    assert 0.40 <= s['onset_imu']['delay_s'] <= 0.51, s
    if s['gain']:
      assert s['gain']['gain'] == pytest.approx(0.9, abs=0.03)
  assert segs[0]['scc14_at_edge'] == {'upper': 3.0, 'lower': 5.0} and segs[0]['script']['mismatch'] == 0
  assert rec['hold']['hold_s'] == pytest.approx(2.0, abs=0.02) and not rec['hold']['escape'] and rec['hold']['wire_max_dev'] < K.HOLD_TOL
  t = rec['terminal']
  assert t['flag_to_stopreq_s'] == pytest.approx(0.1, abs=0.03) and t['displacement']['pulse_m'] < 0.05
  assert t['displacement']['m_per_pulse'] == pytest.approx(0.02, rel=0.02)
  assert t['decel_at_stop']['wheel_slope_0p3s'] == pytest.approx(0.9 * K.MAN[man][2][-1].accel, abs=0.15)
  assert rec['grade']['ok'] and rec['gps']['bearing_deg'] == pytest.approx(359.0) and rec['intent']['stopping_lag_s'] <= 0.02
  assert (rec['plan'], rec['plan_source']) == ('KCS1', 'log')
  assert series['whl__t'][0] == pytest.approx(-3.0, abs=0.03) and int(series['t0_ns']) == rec['t0_ns']
  if man == 'E':   # s1 -0.8 to 1.5 m/s, s2 -0.3 for 2 s, s3 -0.8 to stop
    assert rec['maneuver_checks'] == {'s1_edge_v': True, 's1_held_before_release': True, 's2_full': True, 's2_v_end': True}


@pytest.mark.parametrize('man', ['I', 'M', 'J', 'N'])
def test_a_ramped_ease_is_script_not_a_stall_and_its_gain_follows_the_arrival(man):
  """review 20260926-163704 P1: the first ramp values are deeper than the level; they are the script, not a stall."""
  streams, meta, _, _ = simulate(man)
  (rec, _), = K.analyze(streams, meta)[0]
  rec = K.clean(rec)
  assert (rec['label'], rec['plan'], rec['valid_for_fit']) == ('complete', 'KCS2', True), rec['failed_checks']
  ease = rec['segments'][1]
  seg = K.MAN[man][2][1]
  assert ease['edge_source'] == 'scc12' and ease['script']['mismatch'] == 0 and ease['script']['deeper'] == 0
  assert ease['ramp']['from'] == -1.0 and ease['ramp']['to'] == seg.accel and ease['ramp']['arrived']
  assert ease['ramp']['t_arrive'] - ease['t_start'] == pytest.approx((seg.accel + 1.0) / seg.jerk, abs=0.03)
  if ease['gain']:
    assert ease['gain']['window'][0] >= ease['ramp']['t_arrive'] - 1e-6 and ease['gain']['gain'] == pytest.approx(0.9, abs=0.03)
  assert rec['intent']['stopping_lag_s'] <= 0.02


def test_stalled_rep():
  """C's -0.3 against a +0.25 creep push stalls: the ramp to A_HOLD is not an override, the rep counts as stalled and
  leaves the terminal set; s2 is measured only up to the stall."""
  streams, meta, _, _ = simulate('C', push=0.25)
  (rec, _), = K.analyze(streams, meta)[0]
  assert (rec['label'], rec['label_agrees'], rec['valid_for_fit'], rec['terminal_use_ok']) == ('stalled', True, True, False), rec['failed_checks']
  assert rec['stall']['logged'] and rec['segments'][1]['script']['deeper'] == 0
  s2 = rec['segments'][1]
  assert s2['scripted_until'] == pytest.approx(rec['stall']['t']) and s2['t_end'] > s2['scripted_until'] + 1.0
  assert s2['gain']['realized'] == pytest.approx(0.9 * -0.3 + 0.25, abs=0.03)          # net acceleration against the push
  assert rec['hold']['wire_max_dev'] < K.HOLD_TOL and rec['intent']['t'] <= rec['stall']['t'] + 1e-9


def test_override_and_short_hold_labels():
  streams, meta, _, _ = simulate('B', hold_s=0.5)
  (rec, _), = K.analyze(streams, meta)[0]
  assert rec['label'] == rec['device_label'] == 'short-hold' and not rec['valid_for_fit'] and 'hold_min' in rec['failed_checks']
  assert rec['banner']['end']['kind'] == 'not_counted' and not rec['counted_on_device']
  streams, meta, _, _ = simulate('A')
  sc = streams['scc12']
  sc['aReqValue'] = np.where((sc['t'] > 107.0) & (sc['t'] < 107.2), -0.6, sc['aReqValue'])   # a deeper normal demand passed
  (rec, _), = K.analyze(streams, meta)[0]
  assert rec['label'] == 'overridden' and rec['segments'][0]['script']['deeper'] > 0 and not rec['label_agrees']


def test_echo_ignores_the_frame_blocked_at_the_brake():
  """(a) The Panda blocks the frame sent just before carState reports the brake: no echo, not a CAN gap."""
  streams, meta, _, t_brake = simulate('B')
  sc, echo = streams['scc12'], streams['scc12_echo']
  last = sc['t'][sc['t'] < t_brake][-1]
  assert t_brake - last <= K.PANDA_BLOCK_S

  def drop(*ts):
    keep = ~np.isin(echo['t'], [t + 0.003 for t in ts])
    streams['scc12_echo'] = {key: x[keep] for key, x in echo.items()}
    return K.analyze(streams, meta)[0][0][0]

  rec = drop(last)
  assert rec['echo_missing'] == 0 and rec['echo_blocked_at_brake'] and 'echo' not in rec['failed_checks'] and rec['valid_for_fit']
  rec = drop(last, sc['t'][sc['t'] < t_brake][-50])                    # a missing echo elsewhere still fails
  assert rec['echo_missing'] == 1 and 'echo' in rec['failed_checks']


def test_grade_gate_on_esp12_and_imu_offset():
  """(b) the gain-use gate reads the ESP12 grade, the biased orientationNED pitch is metadata; (g) a per-rep constant
  from the pre-window takes grade and IMU bias out of body__long."""
  streams, meta, _, _ = simulate('A')
  streams['cc']['pitch'] = np.full(len(streams['cc']['t']), -0.0245)   # the -2.45 % pitch offset of KCS1 drive 1
  streams['imu']['z'] = streams['imu']['z'] - 0.08                      # forward reading 0.08 m/s^2 high
  (rec, series), = K.analyze(streams, meta)[0]
  g = rec['grade']
  assert g['grade_pct_pitch'] == pytest.approx(-2.45, abs=0.01) and abs(g['grade_pct_esp12']) < 0.2 and g['ok'] and rec['gain_use_ok']
  assert g['imu_offset'] == pytest.approx(0.08, abs=0.02)
  s1 = rec['segments'][0]['gain']
  assert s1['imu_mean'] == pytest.approx(s1['realized'], abs=0.03)
  assert abs(np.mean(series['body__long'][(series['body__t'] > -2.0) & (series['body__t'] < 0.0)])) < 0.02


def test_stop_time_from_pulses():
  """(c) the flag trails the wheel stop: gains, bins and terminal features end at the last pulse; hold and StopReq keep
  the flag. A window ending at the flag would read the rest as braking (gain too soft)."""
  streams, meta, t_flag, t_brake = simulate('B', flag_lag=0.3)
  (rec, _), = K.analyze(streams, meta)[0]
  t = rec['terminal']
  assert t['stop_source'] == 'pulses' and 0.3 <= t['stop_to_flag_s'] <= 0.45 and t['t_flag'] == pytest.approx(t_flag - 105.0, abs=0.02)
  s1 = rec['segments'][0]
  assert s1['moving_until'] == pytest.approx(t['t_stop']) and s1['gain']['window'][1] == pytest.approx(t['t_stop'])
  assert s1['gain']['gain'] == pytest.approx(0.9, abs=0.03) and max(b['v_lo'] for b in s1['bins']) > 3.0
  t0 = rec['t0_ns'] * 1e-9
  w = K.sl(streams['whl'], t0 - 3.0, t_brake)
  assert K.gain(w['t'], w['mean'], t0, t_flag, -1.0)['gain'] < 0.85     # the old window, ending at the flag
  assert t['decel_at_stop']['wheel_slope_0p3s'] == pytest.approx(-0.9, abs=0.1) and t['displacement']['pulses'] == 0.0
  assert rec['hold']['hold_s'] == pytest.approx(2.0, abs=0.02) and t['flag_to_stopreq_s'] == pytest.approx(0.1, abs=0.03)


def test_stopreq_after_the_brake_is_ignored():
  """(d) a StopReq first sent after the driver's brake does not belong to the hold."""
  streams, meta, _, t_brake = simulate('B')
  sc = streams['scc12']
  sc['StopReq'] = ((sc['t'] > t_brake) & (sc['t'] < t_brake + 0.05)).astype(float)
  (rec, _), = K.analyze(streams, meta)[0]
  assert rec['terminal']['flag_to_stopreq_s'] is None and rec['hold']['stopreq_frac'] is None


@pytest.mark.parametrize('value,direction,lo,hi', [(-0.34, 1.0, 0.45, 0.51), (-1.2, -1.0, 0.45, 0.7)])
def test_overridden_start_onset_from_the_sent_step(value, direction, lo, hi):
  """(e) D s2 overridden at its start: the wire steps -1.0 -> the normal chain's value, not to the script's 0.0. The
  onset times the response to that sent step (known input), in its own direction."""
  streams, meta, _, _ = simulate('D', override=(2, 0.5, value))
  (rec, _), = K.analyze(streams, meta)[0]
  s2 = rec['segments'][1]
  assert rec['label'] == 'overridden' and s2['edge_source'] == 'wire' and 0.0 < s2['edge_minus_banner_s'] <= 0.02
  assert s2['wire_step'] == {'from': -1.0, 'to': value} and s2['direction'] == direction
  assert lo <= s2['onset_wheel']['delay_s'] <= hi and lo - 0.05 <= s2['onset_imu']['delay_s'] <= hi
  assert rec['segments'][2]['edge_source'] == 'scc12' and rec['segments'][2]['wire_step'] == {'from': 0.0, 'to': -0.8}


def test_label_from_the_wire():
  """(f) the device flags 'overridden' from 100 Hz carControl differences that never reach the wire: label from the
  sent frames, the device's verdict kept beside it."""
  streams, meta, _, _ = simulate('C', push=0.25)
  meta['hook_lines'] = [(t, x.replace('reason=stalled', 'reason=overridden').replace('done=C', 'done=')) for t, x in meta['hook_lines']]
  (rec, _), = K.analyze(streams, meta)[0]
  assert (rec['label'], rec['device_label'], rec['label_agrees'], rec['valid_for_fit']) == ('stalled', 'overridden', False, True)


def test_plan_from_the_log_or_the_table():
  """Maneuvers come from every block; the plan from the route's saved progress record, else the block with the id."""
  streams, meta, _, _ = simulate('K')
  (rec, _), = K.analyze(streams, meta)[0]
  assert (rec['label'], rec['plan'], rec['plan_source'], len(rec['segments'])) == ('complete', 'KCS2', 'log', 3), rec['failed_checks']
  assert rec['segments'][1]['wire_step'] == {'from': -0.7, 'to': -0.45}
  # a stale loaded record of another plan (the hook restarts its counts) does not relabel the rep
  meta['hook_lines'] = [(t, x.replace("'KCS2'", "'KCS1'").replace('saved', 'loaded')) for t, x in meta['hook_lines']]
  (rec, _), = K.analyze(streams, meta)[0]
  assert (rec['plan'], rec['plan_source']) == ('KCS2', 'table')
  assert K.plan_of('A', ['KCS2', 'KCS1']) == ('KCS1', 'log') and K.plan_of('Z', []) == (None, 'table')


def test_zero_rep_log(tmp_path):
  from cereal import log, messaging
  msgs = [messaging.new_message('initData'), messaging.new_message('carState'), messaging.new_message('alertDebug'), log.Event.new_message()]
  msgs[0].initData.gitCommit = 'deadbeef'
  msgs[2].alertDebug.alertText1 = 'Update Ready'
  msgs[3].logMessage = json.dumps({'msg': 'identification hook constructed: OFF'})
  seg = tmp_path / '00000001--0123456789--0'
  seg.mkdir()
  (seg / 'rlog.zst').write_bytes(zstandard.compress(b''.join(m.to_bytes() for m in msgs)))
  before = (seg / 'rlog.zst').read_bytes()
  records, infos = K.run([seg / 'rlog.zst'], tmp_path / 'out')
  assert records == [] and infos[0]['constructed'] == 1 and infos[0]['commits'] == ['deadbeef']
  assert (tmp_path / 'out' / 'reps.jsonl').read_text() == '' and 'No reps found.' in (tmp_path / 'out' / 'summary.md').read_text()
  assert (seg / 'rlog.zst').read_bytes() == before
  with pytest.raises(FileExistsError):
    K.run([seg / 'rlog.zst'], tmp_path / 'out')
