#!/usr/bin/env python3
"""KCS2 pre-registered gates and decision (~/.route_sync/corpus/stopping_decision_20260926/DECISION_v2.md section 4).

Written before any KCS2 data exists; the script's sha256 is printed with every result so a later run can show that the
rules were not changed after the data came in.

Usage (repository root, .venv active):
  python tools/stopping/review/kcs2_gates.py EXTRACTOR_DIR [EXTRACTOR_DIR ...] --output NEW_DIR [--kcs1 DIR] [--alias ROLE=MAN]

Input: kcs1_reps.py output directories (reps.jsonl + series/<id>.npz; every time in seconds from the rep's t0). A rep
id or a (route, t0_ns) seen twice across the inputs is an error. --alias reads another block's maneuver as a KCS2 role
(dry runs on KCS1 data only); the gate level is always the rep's own maneuver's last-segment command. Output (the
directory must not exist): gates.json and GATES.md (also printed).

Definitions (speed = whl__mean, the 4-wheel ABS mean; realised = the IMU: the 0.3 s trailing mean of body__long, the
extractor's calibrated forward specific force minus the rep's pre-window offset (grade and bias), over its finite
samples, interpolated to the wheel frames. The ABS wheel slope reads 0.1-0.26 m/s^2 shallow below ~0.5 m/s on this
car, so it gates only (c)):
- Counted rep: record valid_for_fit (the extractor's label complete or stalled and every extractor check passed).
  Other reps, and 'unknown' records without a series, are listed as excluded with their label and reason.
- Trusted stop: terminal.stop_source 'pulses' with pulse frames every <= GAP_MAX (0.1 s) from t0 through the
  standstill flag (a cut pulse stream moves the stop early). IMU data: the rep's IMU offset and finite samples every
  <= GAP_MAX over what a gate reads. (a) and (e) need both; (c) runs to the standstill flag instead of an untrusted stop.
- Each gate is judged on the data it needs. A failing gate fails the rep whatever else is missing; with no failure, a
  gate that cannot be read leaves the rep unmeasured (it counts neither way).
- Gated segment: the maneuver's last segment; level = its command (P -0.7, L -0.5, I/M/N -0.5, J -0.6).
- Window start: a step segment (P, L, K's rebuild) at its edge (t_start) + SETTLE_S + SLOPE_S, so the first trailing
  window opens SETTLE_S after the edge, past the brake build (dead time ~0.19 s, lag ~0.09 s); a ramped segment at
  SETTLE_S after its arrival on the wire (ramp.t_arrive; its early frames carry the deeper transition).
- Window end: the last wheel frame with speed >= V_END at or before the pulse stop (terminal.t_stop; after it the ABS
  speed is a decay curve, not motion).
- (a) window: the window's frames at speed <= V_BAND (the floor band; a held P/L starts at the cruise, an eased window
  starts below it). (a1) mean realised over it <= LEVEL_FRAC x level; (a2) its worst frame (max realised) <=
  WORST_FRAC x level; (a) holds when both hold. An (a) window shorter than A_MIN_S (last minus first frame; no frame =
  0 s: N on the measured plant) makes (a) NOT APPLICABLE (n/a): the rep is judged on (c), (d) and (e).
- (c) no frame with wheel slope (whl__slope_0p3s) >= 0 and speed > V_END from the first braking frame to the pulse stop.
  First braking frame = the first wheel frame at t >= 0 whose wheel slope is <= BRAKING_FRAC x the first segment's
  command; none = fail. Wheel speed and slope only.
- (d) the stall rule did not fire: stall.t is None and stall.logged is False.
- (e) |a_stop - level| <= max(A_STOP_TOL, A_STOP_REL x |level|), a_stop = terminal.decel_at_stop.imu_0p1s (the IMU
  0.1 s trailing mean at the pulse stop, same offset); needs a trusted stop and IMU data over [t_stop - SLOPE_S, t_stop].
- Grade: KCS2 is driven on a flat road; grade_pct_esp12 is reported per rep, no rep is excluded on grade.
- Reported, never gated: TCS13 BrakeLight on-fraction (it follows a command threshold near -0.55, not brake
  application: on in 1 % of KCS1 frames at -0.50), ESP12 LONG_ACCEL (max, median), the wheel-slope maximum, a_stop from
  the wheel slope and ESP12, the ESP12 grade.
- Uncounted reps with a series: (c) from the first braking frame to min(t_abort, (c)'s end) (the last segment's end
  when neither exists); a frame with a >= 0 there is a failing rep of the maneuver.
- Maneuver: FAIL if any counted measured rep fails a gate or any uncounted rep fails (c); PASS if none fails and
  >= MIN_VALID are counted and measured; else INSUFFICIENT.
- K vs P: K window = K's last segment (the -0.7 rebuild, a step) with the window rules above; v_top = median over K's
  counted reps of the wheel speed at that window's first frame. P window = from the step start after P's edge and from
  the first frame at speed <= v_top, to the same end. Level = median realised; deepest = min realised; fade = max
  realised at or after the deepest frame minus the deepest. A role fades if any counted rep has fade >= FADE_MIN; holds
  if >= MIN_VALID counted reps and none fades; else INSUFFICIENT.
- L vs KCS1 A (day control): median a_stop (IMU, as (e)) of counted L reps minus median of counted KCS1 A reps; holds
  if |difference| <= CONTROL_TOL; INSUFFICIENT below MIN_VALID reps on either side.
- Decision: the section-4 reading, evaluated in order: I, M, N, L all PASS -> A_FLOOR -0.50; M FAIL and J PASS ->
  -0.60 for releases at 1.5 m/s (0.8 m/s and held -0.60 UNVALIDATED); M and J FAIL -> ratchet; M FAIL and J
  INSUFFICIENT -> INSUFFICIENT; I, N or L FAIL without M failing -> NOT COVERED (no A_FLOOR, re-plan); else INSUFFICIENT.
  Then P fading (K fades or holds) -> RE-PLAN: no A_FLOOR, whatever the reading above gave.

The hold: no gate, window, a_stop or K/P value reads the hold or the driver's brake. The signals end at the pulse stop
(~10 ms of IMU after it); the pulse frame times and, for an untrusted stop, (c) run to the standstill flag, ~0.2 s
after the stop and before the hold matters. So the fast-cycle build (the rep counts HOLD_AUTO_S = 0.3 s after the hold
reaches -0.70, then drives off with no driver brake) changes no gated number. The one dependency is 'counted' = the
extractor's valid_for_fit, which must count the fast-cycle reps (kcs1_reps.py: hold.end 'launch', no HOLD_MIN_S);
a rep it does not count is excluded, (c) still runs on it, and it cannot make a role PASS.
"""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np

from openpilot.tools.stopping.review.kcs1_reps import GAP_MAX, MAN, SLOPE_S, clean, max_gap, trailing

SETTLE_S = 0.5            # the window starts this long after a ramp's arrival; a step's first trailing window opens this long after its edge
V_END = 0.15              # m/s: the window ends at the last wheel frame at or above this speed
V_BAND = 2.6              # m/s: (a) reads the window's frames at or below this speed (the floor band)
LEVEL_FRAC = 0.85         # (a1) window mean; KCS1 lost 0.15-0.63 of the command after a release
WORST_FRAC = 0.6          # (a2) worst frame; a known-good held -0.5 dips one 0.3 s frame to ~0.75 of it
A_MIN_S = 0.2             # an (a) window shorter than this: (a) not applicable
LAMP_ON = 0.5             # tcs13 BrakeLight (reported only)
BRAKING_FRAC = 0.5        # (c) the first braking frame: wheel slope at or below this fraction of the first command
A_STOP_TOL, A_STOP_REL = 0.07, 0.10   # (e) m/s^2, and the fraction of |level| when that is larger
MIN_VALID = 4
CONTROL_TOL = 0.05        # L vs KCS1 A: median a_stop difference, m/s^2
FADE_MIN = 0.15           # m/s^2: a K fade this large reproduces the 2129 fade
GATED = ('P', 'L', 'I', 'M', 'J', 'N')
ROLES = GATED + ('K',)
MAN_LEVEL = {m: MAN[m][2][-1].accel for m in GATED}
WHAT = {'I': 'releases at 2.5 m/s', 'M': 'releases at 1.5 m/s', 'N': 'releases at 0.8 m/s', 'L': 'a hold from the cruise'}
EPS = 1e-9
KCS1_DIR = Path.home() / '.route_sync/corpus/kcs1_drive1_20260926/analysis_block1_v2'


def load(dirs):
  """(record, series path or None) for every rep of the extractor output directories, in file order; a repeated rep id
  or (route, t0_ns) raises (the same rep passed twice would be counted twice)."""
  out, seen = [], {}
  for d in map(Path, dirs):
    for line in (d / 'reps.jsonl').read_text().splitlines():
      if line.strip():
        rec = json.loads(line)
        keys = [('id', rec['id'])] + ([('route, t0_ns', (rec['route'], rec['t0_ns']))] if rec.get('t0_ns') is not None else [])
        for k in keys:
          if k in seen:
            raise ValueError(f'repeated rep {k[0]} {k[1]} in {d} (first in {seen[k]}): pass every rep once')
          seen[k] = d
        out.append((rec, d / rec['series_file'] if rec.get('series_file') else None))
  return out


def a_stop(rec):
  """IMU a_stop (terminal.decel_at_stop.imu_0p1s); None without a terminal or the rep's IMU offset."""
  term = rec.get('terminal')
  return term['decel_at_stop']['imu_0p1s'] if term and rec['grade']['imu_offset'] is not None else None


def start(rec):
  """(arrival, window start) of the rep's last segment; (None, None) for a ramped segment without a ramp arrival."""
  spec, seg = MAN[rec['maneuver']][2][-1], rec['segments'][-1]
  if spec.jerk is None:
    return seg['t_start'], seg['t_start'] + SETTLE_S + SLOPE_S
  ramp = seg.get('ramp')   # absent in extractor output older than the KCS2 ramps
  return (ramp['t_arrive'], ramp['t_arrive'] + SETTLE_S) if ramp else (None, None)


def stop_problem(rec, z):
  """None when terminal.t_stop is a wheel-pulse stop with pulse frames every <= GAP_MAX from t0 through the standstill
  flag; else why the stop time is not trusted (a pulse stream cut before the real stop moves it early)."""
  term = rec.get('terminal') or {}
  if term.get('t_stop') is None:
    return 'no pulse stop'
  if term.get('stop_source') != 'pulses':
    return f"stop from the {term.get('stop_source')}, not the wheel pulses"
  gap = max_gap({'t': z['pul__t']}, 0.0, term['t_flag']) if 'pul__t' in z else None
  return None if gap is not None and gap <= GAP_MAX + EPS else f'pulse gap {gap} s from t0 to the standstill flag'


def c_end(rec, z):
  """(c)'s end: the pulse stop when it is trusted, else the standstill flag (the ABS speed only decays between them)."""
  term = rec.get('terminal') or {}
  return term['t_stop'] if stop_problem(rec, z) is None else term.get('t_flag')


def imu_problem(rec, z, t_a, t_b):
  """None when the rep has its IMU offset and finite IMU samples every <= GAP_MAX over [t_a, t_b]; else why not."""
  if rec['grade']['imu_offset'] is None or 'body__t' not in z:
    return 'no IMU offset (body__long is not referenced to the pre-window)'
  f = np.isfinite(z['body__t']) & np.isfinite(z['body__long'])
  gap = max_gap({'t': z['body__t'][f]}, t_a, t_b)
  return None if gap is not None and gap <= GAP_MAX + EPS else f'IMU gap {gap} s (finite samples) in {t_a:.2f}-{t_b:.2f} s'


def realised(z, t):
  """The IMU 0.3 s trailing mean of the finite body__long samples at times t."""
  f = np.isfinite(z['body__t']) & np.isfinite(z['body__long'])
  bt = z['body__t'][f]
  s = trailing(bt, z['body__long'][f], SLOPE_S, False)
  g = np.isfinite(s)
  return np.interp(t, bt[g], s[g], left=np.nan, right=np.nan)


def frames(rec, z, t_from):
  """(t, v, wheel slope) of the wheel frames from t_from to the last frame with speed >= V_END at or before the pulse
  stop (empty when there is none)."""
  t, v = z['whl__t'], z['whl__mean']
  ok = (t >= t_from - EPS) & (t <= rec['terminal']['t_stop'] + EPS)
  idx = np.nonzero(ok & (v >= V_END))[0]
  m = ok & (t <= (t[idx[-1]] if len(idx) else -np.inf))
  return t[m], v[m], z['whl__slope_0p3s'][m]


def in_window(t, t_a, t_b):
  return (t >= t_a - EPS) & (t <= t_b + EPS)


def gate_c(rec, z, t_end):
  """(c) from the first braking frame to t_end: wheel frames with slope >= 0 above V_END."""
  wt, wv, ws = z['whl__t'], z['whl__mean'], z['whl__slope_0p3s']
  fb = np.nonzero((wt >= 0.0) & (wt <= t_end + EPS) & (ws <= BRAKING_FRAC * rec['segments'][0]['cmd']))[0]
  t_brake = float(wt[fb[0]]) if len(fb) else None
  nonneg = np.nonzero((wt >= (t_brake if t_brake is not None else np.inf)) & (wt <= t_end + EPS) & (wv > V_END) & (ws >= 0.0))[0]
  return {'first_braking_t': t_brake, 'c_end': t_end, 'nonneg_frames': len(nonneg),
          'nonneg_first': None if not len(nonneg) else {'t': float(wt[nonneg[0]]), 'v': float(wv[nonneg[0]]), 'a': float(ws[nonneg[0]])}}


def gate_rep(rec, z):
  """The section-4 gates on one counted rep. Each gate is evaluated on the data it needs; one it cannot read is listed
  in `unmeasured`. A failing gate fails the rep whatever else is missing; with no failure, an unmeasured gate leaves
  the rep unmeasured (measured False, counts neither way). (a1)/(a2) are absent when (a) is n/a (window < A_MIN_S)."""
  level = MAN[rec['maneuver']][2][-1].accel
  arrival, t_a = start(rec)
  term = rec.get('terminal') or {}
  das = term.get('decel_at_stop') or {}
  stop, tol = a_stop(rec), max(A_STOP_TOL, A_STOP_REL * abs(level))
  out = {'level': level, 'arrival': arrival, 'start': t_a, 'a_mean_threshold': LEVEL_FRAC * level, 'a_max_threshold': WORST_FRAC * level,
         'stall_t': rec['stall']['t'], 'stall_logged': rec['stall']['logged'], 'grade_pct_esp12': rec['grade']['grade_pct_esp12'],
         'a_stop': stop, 'a_stop_tol': tol, 'a_stop_wheel': das.get('wheel_slope_0p3s'), 'a_stop_esp12': das.get('esp12')}
  gates, unmeasured = {'d': rec['stall']['t'] is None and not rec['stall']['logged']}, {}
  end = c_end(rec, z)
  if end is None:
    unmeasured['c'] = 'no standstill flag'
  else:
    out.update(gate_c(rec, z, end))
    gates['c'] = out['first_braking_t'] is not None and not out['nonneg_frames']
  bad_stop = stop_problem(rec, z)
  why = bad_stop or imu_problem(rec, z, term['t_stop'] - SLOPE_S, term['t_stop']) or (None if stop is not None else 'no IMU a_stop')
  if why:
    unmeasured['e'] = why
  else:
    gates['e'] = abs(stop - level) <= tol + EPS
  if bad_stop or t_a is None:
    unmeasured['a'] = bad_stop or 'ramped segment without a ramp arrival on the wire'
  else:
    t, v, ws = frames(rec, z, t_a)
    band = v <= V_BAND + EPS
    t, v, ws = t[band], v[band], ws[band]
    span = float(t[-1] - t[0]) if len(t) else 0.0
    out.update(a_applicable=span >= A_MIN_S - EPS, window=None, window_s=span, frames=len(t))
    if len(t):
      lamp = z['tcs13__BrakeLight'][in_window(z['tcs13__t'], t[0], t[-1])] > LAMP_ON
      esp = z['esp12__LONG_ACCEL'][in_window(z['esp12__t'], t[0], t[-1])]
      out.update(window=[float(t[0]), float(t[-1])], v_window=[float(v[0]), float(v[-1])],
                 wheel_max=float(np.nanmax(ws)) if np.isfinite(ws).any() else None,
                 esp12_max=float(esp.max()) if len(esp) else None, esp12_median=float(np.median(esp)) if len(esp) else None,
                 lamp_frames=len(lamp), lamp_on_frac=float(lamp.mean()) if len(lamp) else None)
    why = imu_problem(rec, z, t[0] - SLOPE_S, t[-1]) if out['a_applicable'] else None
    if why:
      unmeasured['a'] = why
    elif out['a_applicable']:
      a = realised(z, t)
      i = int(np.argmax(a))
      out.update(a_mean=float(a.mean()), a_max=float(a[i]), a_max_t=float(t[i]), a_max_v=float(v[i]),
                 a_frames_over=int(np.sum(a > LEVEL_FRAC * level + EPS)))
      gates = {'a1': out['a_mean'] <= LEVEL_FRAC * level + EPS, 'a2': out['a_max'] <= WORST_FRAC * level + EPS, **gates}
  failed = not all(gates.values())
  measured = failed or not unmeasured
  return {**out, 'gates': gates, 'unmeasured': unmeasured, 'measured': measured, 'passed': measured and not failed,
          **({} if measured else {'reason': '; '.join(f'({k}) {w}' for k, w in unmeasured.items())})}


def status(results, c_failed=()):
  """PASS / FAIL / INSUFFICIENT from the per-rep results of one role (per rep, never a median); c_failed: ids of
  uncounted reps that failed (c)."""
  measured = [r for r in results if r['counted'] and r['measured']]
  failed = [r['id'] for r in measured if not r['passed']] + list(c_failed)
  verdict = 'FAIL' if failed else ('PASS' if len(measured) >= MIN_VALID else 'INSUFFICIENT')
  return {'status': verdict, 'counted': sum(r['counted'] for r in results), 'measured': len(measured),
          'passed': sum(r['passed'] for r in measured), 'failed_reps': failed}


def fade_rep(rec, z, v_top=None):
  """Realised level, deepest value and fade (largest later rise above the deepest) in a K or P window."""
  _, t_from = start(rec)
  why = stop_problem(rec, z)
  if why:
    return {'measured': False, 'reason': why}
  if v_top is not None:
    t, v = z['whl__t'], z['whl__mean']
    below = np.nonzero((t >= t_from - EPS) & (t <= rec['terminal']['t_stop'] + EPS) & (v <= v_top))[0]
    if not len(below):
      return {'measured': False, 'reason': f'no frame at or below v_top {v_top:.2f} m/s before the pulse stop'}
    t_from = float(t[below[0]])
  t, v, _ = frames(rec, z, t_from)
  why = imu_problem(rec, z, t[0] - SLOPE_S, t[-1]) if len(t) else f'no wheel frame >= {V_END} m/s from {t_from:.2f} s to the pulse stop'
  if why:
    return {'measured': False, 'reason': why}
  a = realised(z, t)
  i = int(np.argmin(a))
  j = i + int(np.argmax(a[i:]))
  fade = float(a[j] - a[i])
  return {'measured': True, 'window': [t_from, float(t[-1])], 'frames': len(t), 'v_start': float(v[0]), 'level': float(np.median(a)),
          'deepest': float(a[i]), 'deepest_t': float(t[i]), 'deepest_v': float(v[i]), 'fade': fade, 'fade_t': float(t[j]),
          'fade_v': float(v[j]), 'fades': fade >= FADE_MIN - EPS}


def fade_verdict(results):
  m = [r for r in results if r['measured']]
  return 'fades' if any(r['fades'] for r in m) else ('holds' if len(m) >= MIN_VALID else 'INSUFFICIENT')


def k_vs_p(k_items, p_items):
  """K's rebuild window and P at the same speeds (items: (id, record, series) of counted reps)."""
  k = [{'id': i, **fade_rep(rec, z)} for i, rec, z in k_items]
  starts = [r['v_start'] for r in k if r['measured']]
  v_top = float(np.median(starts)) if starts else None
  p = [{'id': i, **(fade_rep(rec, z, v_top) if v_top else {'measured': False, 'reason': 'no measured K rep'})} for i, rec, z in p_items]

  def med(rs):
    xs = [r['level'] for r in rs if r['measured']]
    return float(np.median(xs)) if xs else None
  kl, pl = med(k), med(p)
  return {'v_top': v_top, 'K': k, 'P': p, 'K_verdict': fade_verdict(k), 'P_verdict': fade_verdict(p), 'K_level': kl, 'P_level': pl,
          'K_minus_P': None if kl is None or pl is None else kl - pl}


def day_control(l_recs, a_recs):
  """L vs KCS1 A: median IMU a_stop of the counted reps (the wheel-slope medians are reported beside)."""
  def median(recs, fn):
    xs = [x for x in map(fn, recs) if x is not None]
    return len(xs), float(np.median(xs)) if xs else None

  def wheel(rec):
    return rec['terminal']['decel_at_stop']['wheel_slope_0p3s']
  (ln, lm), (an, am) = median(l_recs, a_stop), median(a_recs, a_stop)
  out = {'L_n': ln, 'A_n': an, 'L_median': lm, 'A_median': am,
         'L_wheel_median': median(l_recs, wheel)[1], 'A_wheel_median': median(a_recs, wheel)[1]}
  if ln < MIN_VALID or an < MIN_VALID:
    return {**out, 'difference': None, 'verdict': 'INSUFFICIENT'}
  d = lm - am
  return {**out, 'difference': d, 'verdict': 'holds' if abs(d) <= CONTROL_TOL + EPS else 'FAILS'}


def decide(s, kp=None, day=None):
  """(branch, lines): the section-4 reading from the role statuses, the K/P verdicts and the day control."""
  st = {m: s[m]['status'] if isinstance(s[m], dict) else s[m] for m in GATED}
  if all(st[m] == 'PASS' for m in 'IMNL'):
    branch, lines = 'A_FLOOR -0.50', ['A_FLOOR = -0.50 for releases from 2.5 m/s down (I, M, N and L pass; tested conditions only).']
    if st['J'] == 'FAIL':
      lines.append('Warning: J (-0.6) fails where M (-0.5) passes; read the failing J reps before using this branch.')
  elif st['M'] == 'FAIL' and st['J'] == 'PASS':
    branch, lines = 'A_FLOOR -0.60', ['A_FLOOR = -0.60 for releases at 1.5 m/s (M fails, J passes).',
                                      'UNVALIDATED: a -0.60 release at 0.8 m/s (N speed) and a held -0.60; below 1.5 m/s -0.60 is a ' +
                                      'sensitivity arm only.']
  elif st['M'] == 'FAIL' and st['J'] == 'FAIL':
    branch, lines = 'ratchet', ['M and J fail: no floor above -0.7 is proven after a release; the candidate becomes a deepen-only ' +
                                'ratchet (DECISION.md "Profile").']
  elif st['M'] == 'FAIL':
    branch, lines = 'INSUFFICIENT', ['M fails and J has too few counted reps: drive J before A_FLOOR is set.']
  elif any(st[m] == 'FAIL' for m in 'INL'):
    failed = ', '.join(m for m in 'INL' if st[m] == 'FAIL')
    branch, lines = 'NOT COVERED', [f'{failed} fail(s) while M does not fail: outside the section-4 reading; no A_FLOOR, stop and re-plan.']
  else:
    short = ', '.join(m for m in 'IMNL' if st[m] == 'INSUFFICIENT')
    branch, lines = 'INSUFFICIENT', [f'Too few counted reps for {short}: no A_FLOOR yet.']
  if branch != 'INSUFFICIENT':
    lines += [f'{m} {st[m]}: {MAN_LEVEL[m]:+.2f} is not validated for {WHAT[m]}.' for m in 'IMNL' if st[m] != 'PASS']
  if kp is not None:
    k, p = kp['K_verdict'], kp['P_verdict']
    readings = {('fades', 'holds'): ['K fades and P holds: the fade follows the partial release and rebuild, which the floor prevents.'],
                ('fades', 'fades'): ['K and P both fade: a level/speed/site effect at -0.7: stop and re-plan.'],
                ('holds', 'holds'): ['K holds: 2129 not reproduced; its cause stays open.'],
                ('holds', 'fades'): ['K holds: 2129 not reproduced; its cause stays open.',
                                     'P fades while K holds: not in the section-4 reading: stop and re-plan.']}
    kp_lines = readings.get((k, p), [f'K/P INSUFFICIENT (K {k}, P {p}).'])
    if p == 'fades' and k in ('fades', 'holds'):   # both "stop and re-plan" readings: no floor, whatever I/M/N/L say
      branch, lines = 'RE-PLAN', [f'No A_FLOOR: P fades (K {k}): stop and re-plan; the I/M/N/L reading ({branch}) is not used.']
    lines += kp_lines
  if day is not None:
    lines.append({'holds': 'Day control (L vs KCS1 A a_stop) holds.',
                  'FAILS': f'Day control FAILS (L vs KCS1 A median a_stop differ by more than {CONTROL_TOL}): the KCS2 results describe this day only.',
                  'INSUFFICIENT': f'Day control INSUFFICIENT (L or KCS1 A below {MIN_VALID} counted reps).'}[day['verdict']])
  return branch, lines


def evaluate(items, kcs1_items, alias=None):
  """items: (record, series path) of the KCS2 reps; kcs1_items: the same for KCS1 (the A reps are the day control)."""
  roles = {**{r: r for r in ROLES}, **(alias or {})}
  role_of = {m: r for r, m in roles.items()}
  reps, excluded, ignored, kp_items = [], [], {}, {'K': [], 'P': []}
  for rec, path in items:
    role = role_of.get(rec['maneuver'])
    if role is None:
      ignored[rec['maneuver']] = ignored.get(rec['maneuver'], 0) + 1
      continue
    base = {'role': role, 'id': rec['id'], 'maneuver': rec['maneuver'], 'label': rec['label'], 'device_label': rec.get('device_label'),
            'counted_on_device': rec.get('counted_on_device'), 'counted': bool(rec.get('valid_for_fit', False))}
    series = path is not None and Path(path).exists()
    if not base['counted']:
      reason = f"label {rec['label']}" + (f" ({rec['reason']})" if rec.get('reason') else '') + \
               f"; failed checks: {', '.join(rec.get('failed_checks', [])) or '-'}"
      if not series or not rec.get('segments'):
        excluded.append({**base, 'reason': reason + '; (c) not checked: no series', 'c_failed': False})
        continue
      z = dict(np.load(path))
      ends = [x for x in (rec.get('t_abort'), c_end(rec, z)) if x is not None]
      c = gate_c(rec, z, min(ends) if ends else rec['segments'][-1]['t_end'])
      nn = c['nonneg_first']
      if nn:
        reason += f"; (c) FAILS to {c['c_end']:.2f} s: {c['nonneg_frames']} frames a >= 0 (first t {nn['t']:.2f}, v {nn['v']:.2f}): " + \
                  ('a failing rep' if role in GATED else 'reported only (K is not gated)')
      else:
        reason += f"; (c) clean to {c['c_end']:.2f} s" if c['first_braking_t'] is not None else f"; (c) no braking frame to {c['c_end']:.2f} s"
      excluded.append({**base, 'reason': reason, 'c_check': c, 'c_failed': nn is not None})
      continue
    if not series:
      excluded.append({**base, 'reason': f'series file missing: {path}'})
      continue
    z = dict(np.load(path))
    if role in kp_items:
      kp_items[role].append((rec['id'], rec, z))
    if role == 'K':
      continue
    res = {**base, **gate_rep(rec, z)}
    reps.append(res)
    if not res['measured']:
      excluded.append({**base, 'reason': 'unmeasured: ' + res['reason']})
  statuses = {m: status([r for r in reps if r['role'] == m], [x['id'] for x in excluded if x['role'] == m and x.get('c_failed')])
              for m in GATED}
  kp = k_vs_p(kp_items['K'], kp_items['P'])
  day = day_control([rec for rec, _ in items if role_of.get(rec['maneuver']) == 'L' and rec.get('valid_for_fit', False)],
                    [rec for rec, _ in kcs1_items if rec['maneuver'] == 'A' and rec.get('valid_for_fit', False)])
  branch, lines = decide(statuses, kp, day)
  return clean({'roles': roles, 'reps': reps, 'excluded': excluded, 'ignored_maneuvers': ignored, 'status': statuses, 'k_vs_p': kp,
                'day_control': day, 'branch': branch, 'decision': lines})


def f(x, nd=2):
  return '-' if x is None else f'{x:+.{nd}f}'


def row(*cells):
  return '| ' + ' | '.join(str(c) for c in cells) + ' |'


def report(res, meta):
  inputs = ', '.join(f"`{d}` (reps.jsonl sha256 {h[:12]})" for d, h in meta['inputs'])
  roles = ', '.join(f'{r}={m}' for r, m in res['roles'].items())
  out = ['# KCS2 gates (pre-registered, DECISION_v2 section 4)', '',
         f"Script sha256 `{meta['script_sha256']}`. Inputs: {inputs}. KCS1 control: `{meta['kcs1']}`. Roles: {roles}.", '',
         f"## Decision: {res['branch']}", '', *[f'- {x}' for x in res['decision']], '', '## Maneuvers', '',
         row('role', 'level', 'status', 'counted', 'measured', 'passed', 'failing reps'), row(*['---'] * 7)]
  for m in GATED:
    s = res['status'][m]
    out.append(row(f"{m} ({res['roles'][m]})", f"{MAN[res['roles'][m]][2][-1].accel:+.2f}", s['status'], s['counted'], s['measured'], s['passed'],
                   ', '.join(s['failed_reps']) or '-'))
  out += ['', f"## Gated reps (window from {SETTLE_S + SLOPE_S} s after a step's edge or {SETTLE_S} s after a ramp's arrival to the " +
          f"last frame >= {V_END} m/s before the pulse stop; (a) reads its frames at <= {V_BAND} m/s, n/a below {A_MIN_S} s; times in s " +
          f"from t0; realised = IMU {SLOPE_S} s trailing mean of body__long)", '',
          row('role', 'rep', 'label', 'level', 'grade %', '(a) window (s)', 'n', 'v', '(a1) mean / thr', '(a2) worst @t,v / thr',
              f'frames over {LEVEL_FRAC} x level', 'wheel max', 'ESP12 max / med', 'lamp on (not gated)', '(c) a >= 0 frames', '(d) stall t',
              '(e) a_stop IMU / tol (wheel / ESP12)', 'a1 a2 c d e (? unmeasured, - n/a)', 'result'), row(*['---'] * 19)]
  for r in res['reps']:
    if not r['measured']:
      continue
    nn = r.get('nonneg_first')
    c = '?' if 'c' in r['unmeasured'] else (f"{r['nonneg_frames']} (first t {nn['t']:.2f}, v {nn['v']:.2f}, a {nn['a']:+.2f})" if nn else
                                            ('0' if r['first_braking_t'] is not None else 'no braking frame'))
    w, am = r.get('window'), 'a_mean' in r
    span = '?' if 'window_s' not in r else ('-' if w is None else f"{w[0]:.2f}-{w[1]:.2f}") + \
        f" ({r['window_s']:.2f}{'' if r['a_applicable'] else ' n/a'})"
    out.append(row(r['role'], r['id'], r['label'], f"{r['level']:+.2f}", f(r['grade_pct_esp12']), span, r.get('frames', '?'),
                   '-' if w is None else f"{r['v_window'][0]:.2f}->{r['v_window'][1]:.2f}",
                   f"{r['a_mean']:+.3f} / {r['a_mean_threshold']:+.3f}" if am else '-',
                   f"{r['a_max']:+.3f} @{r['a_max_t']:.2f},{r['a_max_v']:.2f} / {r['a_max_threshold']:+.3f}" if am else '-',
                   r['a_frames_over'] if am else '-', f(r.get('wheel_max'), 3), f"{f(r.get('esp12_max'))} / {f(r.get('esp12_median'))}",
                   '-' if r.get('lamp_on_frac') is None else f"{r['lamp_on_frac']:.2f} of {r['lamp_frames']}", c,
                   '-' if r['stall_t'] is None and not r['stall_logged'] else f"{f(r['stall_t'])} (logged {r['stall_logged']})",
                   f"{f(r['a_stop'])} / {r['a_stop_tol']:.2f} ({f(r['a_stop_wheel'])} / {f(r['a_stop_esp12'])})",
                   ' '.join(('?' if k[0] in r['unmeasured'] else '-') if k not in r['gates'] else ('+' if r['gates'][k] else 'x')
                            for k in ('a1', 'a2', 'c', 'd', 'e')),
                   ('PASS' if r['passed'] else 'FAIL') + ('; unmeasured ' + '; '.join(f'({k}) {x}' for k, x in r['unmeasured'].items())
                                                          if r['unmeasured'] else '')))
  out += ['', '## Excluded (never silently; an uncounted rep that fails (c) before its abort or stop fails its maneuver)', '']
  out += [f"- {x['role']} `{x['id']}` (device {x['device_label']}, counted on device {x['counted_on_device']}): {x['reason']}"
          for x in res['excluded']] or ['- none']
  if res['ignored_maneuvers']:
    out.append('- not a KCS2 role here: ' + ', '.join(f'{m} ({n} reps)' for m, n in sorted(res['ignored_maneuvers'].items())))
  kp = res['k_vs_p']
  out += ['', f"## K vs P (K {kp['K_verdict']}, P {kp['P_verdict']}; a fade >= {FADE_MIN} fades; realised = IMU)", '',
          f"v_top (median K window start speed) {f(kp['v_top'])} m/s; level median K {f(kp['K_level'])}, P at the same speeds " +
          f"{f(kp['P_level'])}, K - P {f(kp['K_minus_P'])} m/s^2.", '',
          row('role', 'rep', 'window', 'n', 'v start', 'level', 'deepest @t,v', 'fade @t,v'), row(*['---'] * 8)]
  for role in ('K', 'P'):
    for r in kp[role]:
      if not r['measured']:
        out.append(row(role, r['id'], 'unmeasured: ' + r['reason'], '', '', '', '', ''))
        continue
      out.append(row(role, r['id'], f"{r['window'][0]:.2f}-{r['window'][1]:.2f}", r['frames'], f"{r['v_start']:.2f}", f"{r['level']:+.3f}",
                     f"{r['deepest']:+.3f} @{r['deepest_t']:.2f},{r['deepest_v']:.2f}",
                     f"{r['fade']:+.3f} @{r['fade_t']:.2f},{r['fade_v']:.2f}" + (' FADES' if r['fades'] else '')))
  d = res['day_control']
  out += ['', f"## L vs KCS1 A (day control): {d['verdict']}", '',
          f"a_stop IMU median L {f(d['L_median'])} (n {d['L_n']}), KCS1 A {f(d['A_median'])} (n {d['A_n']}), " +
          f"difference {f(d['difference'])} (tolerance {CONTROL_TOL}); wheel-slope medians (not gated) L {f(d['L_wheel_median'])}, " +
          f"A {f(d['A_wheel_median'])}."]
  return '\n'.join(out) + '\n'


def sha256(path):
  return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main(argv=None):
  ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
  ap.add_argument('reps', nargs='+', type=Path, help='kcs1_reps.py output directories')
  ap.add_argument('--output', required=True, type=Path, help='New destination directory')
  ap.add_argument('--kcs1', type=Path, default=KCS1_DIR, help='KCS1 extractor output for the L vs A day control')
  ap.add_argument('--alias', action='append', default=[], metavar='ROLE=MAN', help='dry runs: read maneuver MAN as KCS2 role ROLE')
  args = ap.parse_args(argv)
  alias = dict(a.split('=', 1) for a in args.alias)
  if any(r not in ROLES or m not in MAN for r, m in alias.items()):
    ap.error(f'--alias needs ROLE in {ROLES} and a maneuver of identification_hook.BLOCKS')
  if args.output.exists():
    raise FileExistsError(f'{args.output} exists; choose a new directory')
  res = evaluate(load(args.reps), load([args.kcs1]), alias)
  meta = {'script_sha256': sha256(__file__), 'inputs': [(str(d), sha256(Path(d) / 'reps.jsonl')) for d in args.reps], 'kcs1': str(args.kcs1)}
  text = report(res, meta)
  args.output.mkdir(parents=True)
  (args.output / 'gates.json').write_text(json.dumps({**meta, **res}, indent=1, allow_nan=False) + '\n')
  (args.output / 'GATES.md').write_text(text)
  print(text)
  return res


if __name__ == '__main__':
  main()
