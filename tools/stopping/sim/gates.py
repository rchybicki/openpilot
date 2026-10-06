"""The fixed gate list (PLAN section 72) on the arm rows of one run. Pure functions on rows (unit-tested in test_sim.py).

HARD: H1 flags off == HEAD (exact replay incl. the planner lockstep + closed loop), the car check and HEAD replay fidelity to the
logged command; H2 no new rest / minimum gap < 3.0 m; H3 model-only stops honoured; H4 StopReq / ownership; H5 launches (gap growth
since the rest, later first motion with a larger gap, false launch toward a stopped lead); H6 J-limited releases the change owns.
Gate definitions: PLAN sections 72 and 82 (HOST DECISIONS). A run with a failed or missing job is INCOMPLETE (no verdict).
COMFORT: six aggregates on the valid pairs of the NEW + history groups in the validated cells L42 + L42s (start v12): BETTER when
no aggregate is worse (> COMFORT_TOL and >= COMFORT_MIN_COUNT counts) and more aggregates are better than worse. The per-case
regression list is not gating. Merged from the cyc_1004 analyzers (an.py line(), analyze_cl.py, e3 cana2.py, report5.py)."""
import json
import os
import pickle
import re
import subprocess
import sys
from collections import Counter, defaultdict

import numpy as np

from openpilot.tools.stopping.sim.loader import REPO

LIMIT_GAP = 3.0           # m, H2
LAUNCH_LATE = 0.3         # s, H5: a first motion this much later than HEAD's (with a larger gap) FAILS
GAP_MOTION = 0.3          # m, H5: gap growth since the rest / gap at first motion larger than HEAD's by more than this FAILS (PLAN 82)
MOVE_V = 0.05             # m/s, H5: first motion = v_true above this, from 0.3 s after the rest (eb5 h5_gap.py)
STOP_PAST = 0.5           # m, H3: FAIL when the candidate stops further than this past HEAD's stop position
FID_SKIP_S = 1.0          # s, H1c: replay warm-up skipped at the start of each span (drive_report rule)
FALSE_MOVE = 0.25         # m, H5: ego travel from rest that counts as a launch
LEAD_STOPPED_V = 0.15     # m/s, H5: a launch is false only toward a lead slower than this through the launch (PLAN 82)
LEAD_V_HALF = 0.25        # s, H5: lead speed from the lead position (ego x + gap) over +-this where vl_true is missing
DESPIKE_S = 0.15          # s, H2/H5: gap median window (3 radar frames at 20 Hz: one-radar-frame glitches go, PLAN 86)
J_MAX = 2.5               # m/s^3, H6 (PLAN 82: 0.125 per 50 ms planner tick on the braking component of releases the change owns)
PLAN_DT = 0.05            # s, planner tick (J_MAX x PLAN_DT = 0.125 per tick, eb5 xr_an6.py)
LINE_BIND = 0.01          # m/s^2, H6: the line floor binds where the plan's aTarget is within this of it
HEAD_RISE_TOL = 0.01      # m/s^2, H6: an owned release counts only where it rises more than HEAD's command in the same window + this
CHATTER_S = 1.0           # s, H4: StopReq clear -> set at rest within this (PLAN 82)
PID_HOLD_S = 0.2          # s, H4: service hold at rest while LongControl is in pid
REVERSAL = 1e-3           # m/s^2, H4: a falling command frame (a reversal of the release / starting ramp)
BAND_CELLS = ('L42', 'L42s', 'L42F')   # H2: level-trigger cells without the reactive model, not valid in the 1st-gear brake-off band
CONFIRM_CELLS = ('L42P', 'H')          # H2: a band failure counts only if one of these agrees (PLAN 82)
BAND_FRAC = 0.5           # H2: share of the frames from the launch to the minimum gap in the brake-off band (plant off, 1st gear, braking)
COMFORT_TOL = 0.10        # COMFORT: an aggregate is worse only by more than 10 % ...
COMFORT_MIN_COUNT = 2     # ... and by at least 2 counts (pairs for the 4-5 m share; no count floor for the j300 median) (PLAN 82)
FID_MEAN = 0.01           # H1c: span mean |replayed - logged command| on active frames listed above this (informational)
GAP_TOL = 0.05            # m, valid pair: takeover plan gap <= HEAD's + GAP_TOL (analyze_cl)
RAMP, HOLD, RELEASE = 3, 4, 5
PID, STARTING = 1, 3
COMFORT_CELLS = ('L42', 'L42s')
H2_CELLS = ('L42', 'L42s', 'L42P', 'L42F')
PLAN_GAP = 0.25           # s, H1a / H6 replay rows: the planner lockstep must have a tick at least this often over the replayed frames
DIFF_COLS = ('wire', 'stopreq', 'phase', 'lcs', 'v_true', 'sent')
REPLAY_COLS = ('wire', 'sent', 'stopreq', 'phase', 'lcs', 'owning', 'active')
BUCKETS = ('<3.5', '3.5-4', '4-5', '5-6', '>6', 'none')


# ---- rows --------------------------------------------------------------------------------------------------------------------
def load(path):
  try:
    with open(path, 'rb') as fh:
      return pickle.load(fh)
  except FileNotFoundError:
    return []


def rkey(r):
  return (r['case'], r['cell'], str(r['start']), r['mode'])


def closed_rows(arm_dir, keys=None):
  """Closed rows of an arm (keys: only these job keys = the job set of this run)."""
  return {rkey(r): r for r in load(arm_dir / 'closed.pkl') if not r.get('err') and (keys is None or rkey(r) in keys)}


def replay_rows(arm_dir, corpus, keys=None):
  return {r['span']: r['res'] for r in load(arm_dir / f'replay_{corpus}.pkl') if not r.get('err') and (keys is None or (corpus, r['span']) in keys)}


def missing_jobs(arms, rarms, h_arms, expected, c_arms=None, f_arms=None):
  """[{kind, arm, keys, errors}] for every (kind, arm) whose expected jobs have no row (failed or never run): the run is INCOMPLETE."""
  out = []
  for (kind, name), keys in sorted(expected.items()):
    if kind == 'replay':
      d = rarms[name]
      have = {(c, s) for c in ('c1', 'c2') for s in replay_rows(d, c)}
    else:
      d = h_arms[name] if kind == 'cellH' else (c_arms or {})[name] if kind == 'confirm' else (f_arms or {})[name] if kind == 'f1' else arms[name]
      have = set(closed_rows(d))
    miss = sorted(keys - have)
    if miss:
      errs = {}
      for p in d.glob('errors_*.json'):
        for e in json.loads(p.read_text()):
          errs[tuple(e['key'])] = e['err'].strip().splitlines()[-1][:200]
      out.append(dict(kind=kind, arm=name, keys=[list(k) for k in miss], errors={'|'.join(map(str, k)): errs[k] for k in miss if k in errs}))
  return out


def M(r, k):
  for d in ('f', 'x', 'm'):
    if k in (r.get(d) or {}):
      return r[d][k]
  return None


def bucket(x):
  if x is None:
    return 'none'
  return '<3.5' if x < 3.5 else '3.5-4' if x < 4.0 else '4-5' if x < 5.0 else '5-6' if x < 6.0 else '>6'


def same_value(a, b):
  if isinstance(a, float) and isinstance(b, float) and np.isnan(a) and np.isnan(b):
    return True
  if isinstance(a, (list, tuple)) and isinstance(b, (list, tuple)):
    return len(a) == len(b) and all(same_value(x, y) for x, y in zip(a, b, strict=True))
  return a == b


def same_row(h, v):
  """Bit-identical closed run: the full-trace sha and every metric."""
  if h['trace_sha'] != v['trace_sha']:
    return False
  for d in ('m', 'x', 'f'):
    a, b = h.get(d) or {}, v.get(d) or {}
    if set(a) != set(b) or not all(same_value(a[k], b[k]) for k in a):
      return False
  return True


def runs(mask):
  d = np.diff(np.r_[0, np.asarray(mask, dtype=int), 0])
  return list(zip(np.flatnonzero(d == 1), np.flatnonzero(d == -1), strict=True))


# ---- per-trace measures (closed: the 100 Hz window 'w' of nodrv rows or the 50 Hz compact 'tr') ---------------------------
def trace(r):
  """(columns, k0, kd): the closed window from the takeover to the first driver input / logged disengagement (cana2.window)."""
  if r.get('w') is not None:
    x = r['w']
    k0 = r['meta'].get('k0', 0)
  else:
    x = dict(r['tr'])
    if 'phase_i' in x:
      x['phase'] = x['phase_i']
    cl = np.asarray(x.get('closed', np.ones(len(x['t']))), dtype=float) > 0
    k0 = int(np.argmax(cl)) if cl.any() else 0
  t = np.asarray(x['t'], dtype=float)
  drv = (np.asarray(x.get('gas', 0)) > 0) | (np.asarray(x.get('brake', 0)) > 0) | (np.asarray(x.get('active', 1)) < 1)
  drv = np.broadcast_to(drv, t.shape)
  kk = np.flatnonzero(drv[k0:])
  kd = k0 + int(kk[0]) if len(kk) else len(t)
  t_dis = (r.get('meta') or {}).get('t_dis')
  if t_dis is not None:
    kd = min(kd, int(np.searchsorted(t, t_dis)))
  return x, k0, kd


def on_grid(x, t):
  """x's per-frame columns at the timestamps t: the nearest sample within 0.6 of x's sample period, NaN where x has none ('_cov'
  False). Two arms are compared at the same instants, never by array index: their stored traces can start at different times
  (the compact drv trace starts 45 s before each arm's own stop) or sample a different 50 Hz phase."""
  tx, t = np.asarray(x['t'], dtype=float), np.asarray(t, dtype=float)
  n = len(tx)
  if n == 0:
    return dict(t=t, _cov=np.zeros(len(t), dtype=bool))
  i = np.clip(np.searchsorted(tx, t), 0, n - 1)
  il = np.clip(i - 1, 0, n - 1)
  j = np.where(np.abs(tx[il] - t) <= np.abs(tx[i] - t), il, i)
  cov = np.abs(tx[j] - t) <= 0.6 * (float(np.median(np.diff(tx))) if n > 1 else 0.0) + 1e-9
  out = {k: np.where(cov, np.asarray(c, dtype=float)[j], np.nan) for k, c in x.items() if np.ndim(c) == 1 and len(c) == n and k != 't'}
  return dict(out, t=t, _cov=cov)


def common_window(h, v):
  """The gate window both arms share, by time: from the later takeover to the earlier end (first driver input / logged
  disengagement, or the end of either stored trace). -> (xh, (k0, kd) of HEAD, xv, (k0, kd) of the candidate, uncovered), the index
  ranges of the same instants in each arm; uncovered = candidate window frames outside the stored HEAD trace (not compared: listed)."""
  xh, k0h, kdh = trace(h)
  xv, k0v, kdv = trace(v)
  th, tv = np.asarray(xh['t'], dtype=float), np.asarray(xv['t'], dtype=float)
  if not (len(th) and len(tv) and k0h < kdh and k0v < kdv):
    return xh, (k0h, k0h), xv, (k0v, k0v), 0
  end = lambda t, kd: t[kd] if kd < len(t) else t[-1] + 1e-6  # noqa: E731 -- exclusive end time of a window
  t0, t1 = max(th[k0h], tv[k0v]), min(end(th, kdh), end(tv, kdv))
  wv = tv[k0v:kdv]
  uncovered = int(np.sum((wv < th[0] - 1e-6) | (wv > th[-1] + 1e-6)))
  rng_ = lambda t: (int(np.searchsorted(t, t0)), max(int(np.searchsorted(t, t1)), int(np.searchsorted(t, t0))))  # noqa: E731
  return xh, rng_(th), xv, rng_(tv), uncovered


def chatter(x, k0, kd, rest_v=0.0):
  """StopReq clear -> set with the car at rest (v <= rest_v) at both transitions within CHATTER_S (a set-clear-set contains one). A
  set followed by the normal launch clear is one StopReq episode, not chatter (PLAN 82). -> [(t_clear, t_set)]"""
  sr, t, v = np.asarray(x['stopreq']), np.asarray(x['t'], dtype=float), np.asarray(x['v_true'], dtype=float)
  tr = [k for k in range(max(k0, 1), kd) if sr[k] != sr[k - 1] and sr[k] >= 0 and sr[k - 1] >= 0]
  return [(round(float(t[a]), 2), round(float(t[b]), 2)) for a, b in zip(tr, tr[1:], strict=False)
          if sr[a] == 0 and sr[b] == 1 and t[b] - t[a] <= CHATTER_S and v[a] <= rest_v and v[b] <= rest_v]


def stopreq_info(x, k0, kd):
  """H4 information (not gating, PLAN 81 events): StopReq sets while the service is in RELEASE, and StopReq clear -> set at rest within
  CHATTER_S in the 1 s after the gate window (a nodrv window ends at the logged disengagement). -> [(what, t)]"""
  sr, ph, t = np.asarray(x['stopreq']), np.asarray(x['phase']), np.asarray(x['t'], dtype=float)
  out = [('StopReq set in RELEASE', round(float(t[k]), 2)) for k in range(max(k0, 1), kd) if sr[k] == 1 and sr[k - 1] == 0 and ph[k] == RELEASE]
  late = set(chatter(x, k0, int(np.searchsorted(t, t[kd - 1] + 1.0)) if kd < len(t) else kd)) - set(chatter(x, k0, kd))
  return out + [('StopReq clear -> set at rest just after the window', c) for c in sorted(late)]


def pid_holds(x, k0, kd):
  """Service RAMP_TO_HOLD / HOLD at rest while LongControl is in pid, longer than PID_HOLD_S."""
  t = np.asarray(x['t'], dtype=float)
  m = np.isin(np.asarray(x['phase']), (RAMP, HOLD)) & (np.asarray(x['lcs']) == PID) & (np.asarray(x['v_true'], dtype=float) <= 0.0)
  m[:k0] = False
  m[kd:] = False
  return [(round(float(t[a]), 2), round(float(t[b - 1] - t[a]), 2)) for a, b in runs(m) if t[b - 1] - t[a] > PID_HOLD_S]


def start_under_hold(x, k0, kd):
  """Frames of LongControl 'starting' under RAMP_TO_HOLD / HOLD that the service owns or where the command falls (a reversal of the
  release / starting ramp). A non-owning hold label over a monotonic ramp is not a conflict (PLAN 82)."""
  w = np.asarray(x['wire'], dtype=float)
  m = np.isin(np.asarray(x['phase']), (RAMP, HOLD)) & (np.asarray(x['lcs']) == STARTING)
  own = np.asarray(x['owning']) > 0 if 'owning' in x else np.zeros(len(m), dtype=bool)
  m &= own | np.r_[False, np.diff(w) < -REVERSAL]
  m[:k0] = False
  m[kd:] = False
  return int(m.sum())


def first_stop(x, k0, kd):
  v = np.asarray(x['v_true'], dtype=float)
  rest = np.flatnonzero(v[k0:kd] <= 0.0)
  return k0 + int(rest[0]) if len(rest) else None


def positions(x):
  t, v = np.asarray(x['t'], dtype=float), np.asarray(x['v_true'], dtype=float)
  if 'x' in x:
    return np.asarray(x['x'], dtype=float)
  return np.r_[0.0, np.cumsum(0.5 * (v[1:] + v[:-1]) * np.diff(t))]


def despike(g, t):
  """Median of the gap over DESPIKE_S: the gap is held per 20 Hz radar frame (5 samples at 100 Hz, 2-3 at 50 Hz), so a one-radar-
  frame glitch is removed (PLAN 82, PLAN 86); NaN (no lead) stays NaN."""
  a = np.asarray(g, dtype=float)
  dt = float(np.median(np.diff(np.asarray(t, dtype=float)))) if len(a) > 1 else 0.0
  half = int(np.ceil(DESPIKE_S / 2 / dt)) if dt > 0 else 0
  if half < 1 or len(a) < 2 * half + 1:
    return a
  m = np.median(np.lib.stride_tricks.sliding_window_view(np.pad(np.where(np.isfinite(a), a, np.inf), half, mode='edge'), 2 * half + 1), axis=1)
  return np.where(np.isfinite(m), m, np.nan)


def lead_speed(x):
  """Lead speed per frame: vl_true where finite, else the lead position (ego x + gap) differenced over +-LEAD_V_HALF."""
  t = np.asarray(x['t'], dtype=float)
  p = positions(x) + np.asarray(x['gap'], dtype=float)
  lo = np.searchsorted(t, t - LEAD_V_HALF)
  hi = np.clip(np.searchsorted(t, t + LEAD_V_HALF, side='right') - 1, 0, len(t) - 1)
  dtt = t[hi] - t[lo]
  d = np.where(dtt > 0, (p[hi] - p[lo]) / np.where(dtt > 0, dtt, 1.0), np.nan)
  vl = np.asarray(x['vl_true'], dtype=float) if 'vl_true' in x else d
  return np.where(np.isfinite(vl), vl, d)


def _launch_runs(x, k0, kd):
  """Motion episodes from rest after the first stop with ego travel > FALSE_MOVE: [(k_onset, k_end, k_travel)], k_travel = the frame
  where the travel from the onset first exceeds FALSE_MOVE."""
  fs = first_stop(x, k0, kd)
  if fs is None:
    return []
  v, xx = np.asarray(x['v_true'], dtype=float), positions(x)
  out = []
  for a, b in runs(v[fs:kd] > 0.0):
    ka, kb = fs + a, fs + b - 1
    if xx[kb] - xx[ka] > FALSE_MOVE:
      out.append((ka, kb, ka + int(np.argmax(xx[ka:kb + 1] - xx[ka] > FALSE_MOVE))))
  return out


def launches(x, k0, kd):
  """Launches from rest after the first stop: (t_onset, ego distance, the lead's top speed from the onset until the ego has travelled
  FALSE_MOVE, false?). False = toward a lead slower than LEAD_STOPPED_V all through that (PLAN 82; a touch-and-go behind a crawler is
  not false); no lead = not false."""
  t, xx, vl = np.asarray(x['t'], dtype=float), positions(x), lead_speed(x)
  out = []
  for ka, kb, kt in _launch_runs(x, k0, kd):
    top = _lead_top(vl, ka, kt)
    out.append((round(float(t[ka]), 2), round(float(xx[kb] - xx[ka]), 2), top, top is not None and top < LEAD_STOPPED_V))
  return out


def _lead_top(vl, ka, kt):
  """The lead's top speed from the launch onset ka until the ego has travelled FALSE_MOVE (kt); None without a lead."""
  seg = vl[ka:kt + 1]
  seg = seg[np.isfinite(seg)]
  return round(float(seg.max()), 2) if len(seg) else None


def false_launch_at(x, k0, kd, k):
  """The lead's top speed through the launch from rest that contains frame k when that launch is false (launches()), else None."""
  vl = lead_speed(x)
  for ka, kb, kt in _launch_runs(x, k0, kd):
    if ka <= k <= kb:
      top = _lead_top(vl, ka, kt)
      return top if top is not None and top < LEAD_STOPPED_V else None
  return None


def rest_motion(x, k0, kend, t_stop=None):
  """(k_rest, k_move): the first rest (t_stop when given, else the first v_true <= 0 after k0) and the first motion after it (v_true >
  MOVE_V at least 0.3 s after the rest) before kend; None when absent (h5_gap.py)."""
  t, v = np.asarray(x['t'], dtype=float), np.asarray(x['v_true'], dtype=float)
  kr = first_stop(x, k0, kend) if t_stop is None else int(np.searchsorted(t, t_stop))
  if kr is None or kr >= min(kend, len(t)):
    return None, None
  mv = np.flatnonzero((t[kr:kend] > t[kr] + 0.3) & (v[kr:kend] > MOVE_V))
  return kr, (kr + int(mv[0])) if len(mv) else None


def _h5_ends(r):
  x, k0, kd = trace(r)
  ts = M(r, 't_stop')
  return x, despike(x['gap'], x['t']), rest_motion(x, k0, kd if r.get('mode') == 'nodrv' else len(x['t']), ts if ts is not None and np.isfinite(ts) else None)


def h5_gap(h, v):
  """H5 on one pair (PLAN 82) -> (fails, info). FAIL when HEAD moves after its rest and the candidate does not; when the candidate's gap
  growth since its rest (gap at first motion - gap at the rest) exceeds HEAD's by more than GAP_MOTION; or when its first motion is
  more than LAUNCH_LATE later at a gap more than GAP_MOTION larger. A larger gap at first motion from a rest further back (same growth,
  same time) passes. info['judged'] False (info['rest'] = why) when there is no comparable launch: no rest in an arm, rests that do
  not overlap, or no motion in either arm. Window: the whole closed trace of a drv run (the driver's own launch counts, as h5_gap.py;
  stored through the first launch), the takeover-to-logged-disengagement window of nodrv. Not judged either where HEAD's own first
  motion is a false launch (PLAN 86)."""
  out: list = []
  info: dict = dict(judged=False)
  (xh, gh, (rh, mh)), (xv, gv, (rv, mv)) = _h5_ends(h), _h5_ends(v)
  if rh is None or rv is None:
    info['rest'] = 'no rest' if rh is None and rv is None else 'HEAD only' if rv is None else 'candidate only'
    return out, info
  # the same rest: the two rests overlap in time (a candidate that rolls through the lead's lurch and rests only after HEAD has
  # moved off again, or the reverse, has no comparable launch; listed as information)
  th0, tv0 = float(xh['t'][rh]), float(xv['t'][rv])
  if (mh is not None and tv0 >= float(xh['t'][mh])) or (mv is not None and th0 >= float(xv['t'][mv])):
    info['rest'] = f'rests do not overlap (HEAD {th0:.2f}, candidate {tv0:.2f})'
    return out, info
  if mh is None and mv is None:
    info['rest'] = 'no motion in the window'
    return out, info
  # HEAD's own first motion is a false launch (toward a lead still stopped): it is no reference for the candidate's launch time (PLAN
  # 86); the candidate's own false launches are judged separately (launches())
  if mh is not None:
    _, k0h, kdh = trace(h)
    top = false_launch_at(xh, k0h, kdh if h.get('mode') == 'nodrv' else len(xh['t']), mh)
    if top is not None:
      info['rest'] = f"HEAD's first motion at {float(xh['t'][mh]):.2f} is a false launch (lead <= {top:.2f} m/s)"
      return out, info
  info['judged'] = True
  rnd = lambda y: None if y is None or not np.isfinite(y) else round(float(y), 2)  # noqa: E731
  info.update(head_move=None if mh is None else round(float(xh['t'][mh]), 2), cand_move=None if mv is None else round(float(xv['t'][mv]), 2),
              head_gap=rnd(gh[mh]) if mh is not None else None, cand_gap=rnd(gv[mv]) if mv is not None else None,
              head_rest_gap=rnd(gh[rh]), cand_rest_gap=rnd(gv[rv]))
  if mh is not None and mv is None:
    out.append(dict(check='candidate never moves (HEAD does)', head_move=info['head_move'], head_gap=info['head_gap']))
  elif mh is not None and mv is not None:
    info['dt'] = round(float(xv['t'][mv] - xh['t'][mh]), 2)
    fin = all(y is not None for y in (info['head_gap'], info['cand_gap'], info['head_rest_gap'], info['cand_rest_gap']))
    if fin:
      info['dgap'] = round(info['cand_gap'] - info['head_gap'], 2)
      info['dgrowth'] = round((float(gv[mv]) - float(gv[rv])) - (float(gh[mh]) - float(gh[rh])), 2)
      checks = []
      if info['dgrowth'] > GAP_MOTION:
        checks.append(f'gap growth since the rest > {GAP_MOTION} m more than HEAD')
      if info['dt'] > LAUNCH_LATE and info['dgap'] > GAP_MOTION:
        checks.append(f'first motion > {LAUNCH_LATE} s later at a gap > {GAP_MOTION} m larger')
      if checks:
        out.append(dict(check='; '.join(checks), dgrowth=info['dgrowth'], dgap=info['dgap'], dt=info['dt'], head_gap=info['head_gap'],
                        cand_gap=info['cand_gap'], head_rest_gap=info['head_rest_gap'], cand_rest_gap=info['cand_rest_gap']))
  return out, info


def stop_x(r):
  """Ego position at the wheel stop (closed trace 'x'), None without a stop or the column."""
  ts, tr = M(r, 't_stop'), r.get('tr') or {}
  if ts is None or 'x' not in tr:
    return None
  return float(np.interp(ts, np.asarray(tr['t'], dtype=float), np.asarray(tr['x'], dtype=float)))


def first_motion(x, k_from, kd, vmin=0.05):
  v = np.asarray(x['v_true'], dtype=float)
  k = np.flatnonzero(v[k_from:kd] >= vmin)
  return k_from + int(k[0]) if len(k) else None


def min_gap_at(x, k_from, kd):
  """(minimum of the despiked gap in [k_from, kd), its frame); (None, None) without a finite gap."""
  g = despike(x['gap'], x['t'])[k_from:kd]
  ok = np.isfinite(g)
  if not ok.any():
    return None, None
  k = int(np.argmin(np.where(ok, g, np.inf)))
  return float(g[k]), k_from + k


def min_gap_after_stop(x, k0, kd):
  fs = first_stop(x, k0, kd)
  return None if fs is None else min_gap_at(x, fs, kd)[0]


def band_after_launch(x, k0, kd, k_min):
  """True when the minimum gap (frame k_min) comes after a launch from rest and the plant brake was off in 1st gear on >= BAND_FRAC
  of the braking-command frames from the last launch onset before k_min to k_min (PLAN 81/82: the level plant of L42 / L42s gives
  ~0 decel for -0.2..-0.42 at 0.5-1.5 m/s in 1st gear; the log of 2072_2413.91 contradicts it). Rows without the plant columns: False."""
  if k_min is None or 'plant_off' not in x or 'gear' not in x:
    return False
  ons = [ka for ka, _, _ in _launch_runs(x, k0, kd) if ka < k_min]
  if not ons:
    return False
  seg = slice(ons[-1], k_min + 1)
  brk = np.asarray(x['wire'], dtype=float)[seg] < 0.0
  off = (np.asarray(x['plant_off'])[seg] > 0) & (np.asarray(x['gear'])[seg] == 1)
  return bool(brk.any() and (off & brk).sum() >= BAND_FRAC * brk.sum())


def owned_release_breaches(h, v, k0, kd):
  """H6 (PLAN 82): J_MAX x PLAN_DT (0.125) per 50 ms planner tick on the braking component of releases the change owns (LongControl
  'starting' frames exempt):
  (a) service: frames that v's service owns (or owned the frame before: the hand-back step is the service's) where v's command differs
      from h's by > 0.01 anywhere in the rise window (a release that ends exactly at h's value counts): the rise of min(command, 0)
      over the last PLAN_DT, summed over owned frames only (a planner step just before the service entry is not the service's; a step
      up to 0.125 or a 2.5 m/s^3 ramp passes);
  (b) line: planner ticks where v's aTarget differs from h's by > 0.01 (at the tick or the one before) and v's line floor bound the
      previous tick (aTarget within LINE_BIND of the floor; the recorded floor may lead aTarget by one sample): the rise of
      min(aTarget, 0) at the tick. A tick that rises from a deeper MPC value to the floor is the MPC's release capped by the line.
  Both only where the rise exceeds h's own rise in the same window by HEAD_RISE_TOL (PLAN 86: h's wire for (a), h's aTarget for (b)).
  h is the reference (HEAD; OFF on the replay), taken at v's timestamps (on_grid); k0 / kd index v; a frame without a HEAD sample
  counts as different with an unknown (no) HEAD rise. -> [(t, rise, 'service' | 'line')]"""
  n = min(len(v['wire']), kd)
  k0 = max(k0, 2)
  if n <= k0:
    return []
  sl = slice(k0, n)
  h = on_grid(h, np.asarray(v['t'], dtype=float)[:n])

  def col(x, c, dtype=float):
    return np.asarray(x[c], dtype=dtype)[:n] if c in x else None

  def brk(a):
    return np.minimum(np.nan_to_num(a, nan=0.0), 0.0)

  def svc(x):   # (owned at the frame or the one before, braking-component rise on owned frames over the last PLAN_DT)
    t = np.asarray(x['t'], dtype=float)[:n]
    o = col(x, 'owning') > 0 if 'owning' in x else np.zeros(n, dtype=bool)
    o = o | np.r_[False, o[:-1]]
    c = np.cumsum(np.where(o, np.r_[0.0, np.diff(brk(col(x, 'wire')))], 0.0))
    return o, c - c[np.searchsorted(t, t - PLAN_DT - 1e-6)]   # window (t - PLAN_DT, t]: 5 frames at 100 Hz, 2 samples at 50 Hz

  def line(x):   # (the floor bound the previous tick, braking-component rise at the tick)
    lf, at = col(x, 'line_floor'), col(x, 'a_target')
    if lf is None or at is None:
      return np.zeros(n, dtype=bool), np.zeros(n)
    b = brk(at)
    bind = np.isfinite(at) & ((np.abs(at - lf) <= LINE_BIND) | (np.abs(at - np.r_[np.nan, lf[:-1]]) <= LINE_BIND))
    return np.r_[False, bind[:-1]], np.r_[0.0, np.diff(b)]

  ov, cv = svc(v)
  bv, pv = line(v)
  start_v = col(v, 'lcs', int) == STARTING
  ath, atv = col(h, 'a_target'), col(v, 'a_target')
  d_plan = np.abs(np.nan_to_num(atv - ath, nan=0.0)) > 0.01 if ath is not None and atv is not None else np.zeros(n, dtype=bool)
  d_plan |= np.r_[False, d_plan[:-1]]
  lim = J_MAX * PLAN_DT + 1e-6
  # ... and faster than HEAD's own command in the same window (PLAN 86, as plan_release_breaches): a release HEAD makes as fast (its
  # planner's, unowned) is not one the change adds
  t = np.asarray(v['t'], dtype=float)[:n]
  lo = np.searchsorted(t, t - PLAN_DT - 1e-6)   # the rise window's first frame
  bh = np.minimum(col(h, 'wire'), 0.0)   # NaN = no HEAD sample
  hv = bh - bh[lo]
  nd = np.r_[0, np.cumsum(~(np.abs(col(v, 'wire') - col(h, 'wire')) <= 0.01))]   # frames that differ (or have no HEAD sample) so far
  div = nd[1:] - nd[lo] > 0   # ... anywhere in the rise window
  ph = np.r_[0.0, np.diff(brk(ath))] if ath is not None else np.zeros(n)
  s_ = div & ov & ~start_v & (cv > lim) & ~(cv <= hv + HEAD_RISE_TOL)
  l_ = ~s_ & d_plan & bv & ~start_v & (pv > lim) & (pv > ph + HEAD_RISE_TOL)
  out = [(k, cv[k], 'service') for k in np.flatnonzero(s_[sl]) + k0] + [(k, pv[k], 'line') for k in np.flatnonzero(l_[sl]) + k0]
  return [(round(float(t[k]), 2), round(float(r), 3), kind) for k, r, kind in sorted(out)]


def plan_release_breaches(x, y, frames):
  """H6 on the planner lockstep (eb5 xr_an6.py rule): plan ticks where the candidate's aTarget differs from the reference's by
  > 0.01 (at the tick or the one before) and its braking component min(aTarget, 0) rises more than J_MAX x PLAN_DT and more than the
  reference's own rise. Exempt: ticks outside the replayed frame window, or where the drive was not engaged or a pedal was pressed
  (frames = the span's replay input, replay.frames). x = reference row, y = candidate row. -> [(t, rise)]"""
  px, py = x.get('plan'), y.get('plan')
  if px is None or py is None or len(px['at']) != len(py['at']) or len(px['at']) < 2:
    return []
  a, b = np.asarray(py['at'], dtype=float), np.asarray(px['at'], dtype=float)
  diff = np.abs(a - b) > 0.01
  ra, rb = np.diff(np.minimum(a, 0.0)), np.diff(np.minimum(b, 0.0))
  ks = [k for k in range(1, len(a)) if (diff[k] or diff[k - 1]) and ra[k - 1] > J_MAX * PLAN_DT + 1e-6 and ra[k - 1] > rb[k - 1] + 1e-6]
  if not ks:
    return []
  d = frames()
  fr, o = d['frames'], d['origin']
  ft = np.array([f['t'] for f in fr])
  out = []
  for k in ks:
    ii = [int(np.searchsorted(ft, py['ns'][kk] * 1e-9, side='right')) - 1 for kk in (k - 1, k)]
    if min(ii) < 0 or any(not fr[i]['active'] or fr[i]['cs']['gasPressed'] or fr[i]['cs']['brakePressed'] for i in ii):
      continue
    out.append((round(float(py['ns'][k] * 1e-9 - o), 2), round(float(ra[k - 1]), 3)))
  return out


def first_div(h, v, k0, kd):
  """The first frame of v in [k0, kd) where a DIFF_COLS column differs from h at the same instant (on_grid; frames without an h
  sample are not compared). Indices are v's."""
  hh = on_grid(h, np.asarray(v['t'], dtype=float)[k0:kd])
  ks = []
  for c in DIFF_COLS:
    if c not in hh or c not in v:
      continue
    d = np.flatnonzero(hh['_cov'] & (np.nan_to_num(hh[c], nan=-99) != np.nan_to_num(np.asarray(v[c], dtype=float)[k0:kd], nan=-99)))
    if len(d):
      ks.append(k0 + int(d[0]))
  return min(ks) if ks else None


def window_same(h, v):
  """No divergence in the takeover-to-driver window both arms share (DIFF_COLS), the E3 closed-loop identity rule (cana2.py)."""
  xh, _, xv, (k0, kd), _ = common_window(h, v)
  return first_div(xh, xv, k0, kd) is None


def reholds(x, k0, kd):
  ph, t = np.asarray(x['phase']), np.asarray(x['t'], dtype=float)
  return [round(float(t[k]), 2) for k in range(max(k0, 1), kd) if ph[k] == RAMP and ph[k - 1] == RELEASE]


# ---- H2 (PLAN 72/82) ----------------------------------------------------------------------------------------------------------
def _h2_value(r, key):
  """(value, band) of one run: min_gap = the despiked minimum gap of the closed window (nodrv: from the first stop on), band = that
  minimum is in the 1st-gear brake-off band after a launch; rest_at_stop = the row metric (never in the band). A drv run whose stored
  trace does not cover the harness window (no driver input inside it, or it starts after the takeover) uses the harness min_gap
  where that is lower (the minimum lies outside the stored trace; not filtered)."""
  if key == 'rest_at_stop':
    return M(r, key), False
  x, k0, kd = trace(r)
  if r.get('mode') == 'nodrv':
    fs = first_stop(x, k0, kd)
    if fs is None:
      return None, False
    k0 = fs
  val, k = min_gap_at(x, k0, kd)
  hm, t = M(r, 'min_gap'), np.asarray(x['t'], dtype=float)
  if r.get('mode') != 'nodrv' and hm is not None and np.isfinite(hm):
    g = np.asarray(x['gap'], dtype=float)[k0:kd]
    raw = float(np.min(g[np.isfinite(g)])) if np.isfinite(g).any() else np.inf
    covered = kd < len(t) and (M(r, 't_lo') is None or M(r, 't_lo') >= t[0])
    if not covered and hm < raw - 0.05:
      return float(hm), False
  return val, val is not None and band_after_launch(x, k0, kd, k)


def h2(pairs, confirm):
  """H2: no new rest / minimum gap < LIMIT_GAP. drv runs: the worst cell per (case, start) over H2_CELLS; nodrv runs: the minimum gap
  after the stop. A minimum-gap failure whose failing cells are all BAND_CELLS runs in the 1st-gear brake-off band after a launch counts
  only if a CONFIRM_CELLS run of the same case agrees (a new < LIMIT_GAP there too) (PLAN 82; the drive's log check stays manual).
  confirm: {(case, cell, start, mode): (HEAD row, candidate row)} for the confirm cells not in pairs.
  -> (fails, cut-ins, band failures not confirmed, coverage, confirm job keys still needed)"""
  by: dict = defaultdict(dict)
  for k, h, v in pairs:
    if (k[3] == 'drv' and k[1] in H2_CELLS) or k[3] == 'nodrv':
      by[(k[0], k[2], k[3])][k[1]] = (h, v)
  rows = {(c, cell, st, md): hv for (c, st, md), cells in by.items() for cell, hv in cells.items()}
  fails, cut, unconfirmed, needs = [], [], [], set()
  for (case, st, mode), cells in sorted(by.items()):
    for key in (('min_gap', 'rest_at_stop') if mode == 'drv' else ('min_gap',)):
      vals = {c: (_h2_value(h, key)[0], _h2_value(v, key)) for c, (h, v) in cells.items()}
      hs = [x for x, _ in vals.values() if x is not None and np.isfinite(x)]
      cs = [y[0] for _, y in vals.values() if y[0] is not None and np.isfinite(y[0])]
      a, b = (min(hs) if hs else None), (min(cs) if cs else None)
      if b is None or b >= LIMIT_GAP or (a is not None and a < LIMIT_GAP):
        continue
      bad = sorted(c for c, (_, (y, _)) in vals.items() if y is not None and y < LIMIT_GAP)
      e = dict(case=case, start=st, mode=mode, metric=key if mode == 'drv' else 'min_gap_after_stop', head=None if a is None else round(a, 2),
               cand=round(b, 2), cells=bad)
      if case.startswith('cut_'):
        cut.append(e)
        continue
      if not all(c in BAND_CELLS and vals[c][1][1] for c in bad):
        fails.append(e)
        continue
      agree, seen = [], {}
      for c in CONFIRM_CELLS:
        hv = rows.get((case, c, st, mode)) or confirm.get((case, c, st, mode))
        if hv is None:
          needs.add((case, c, st, mode))
          continue
        x, (y, _) = _h2_value(hv[0], key)[0], _h2_value(hv[1], key)
        seen[c] = (None if x is None else round(x, 2), None if y is None else round(y, 2))
        if y is not None and y < LIMIT_GAP and not (x is not None and x < LIMIT_GAP):
          agree.append(c)
      e.update(band='1st-gear brake-off band after a launch', confirm=seen)
      if agree:
        fails.append(dict(e, confirmed_by=agree))
      elif not any(n[0] == case and n[2] == st for n in needs):
        unconfirmed.append(dict(e, note='not confirmed by ' + ' / '.join(CONFIRM_CELLS) + ' (not gating; check the drive log)'))
  n_drv = sum(1 for k in by if k[2] == 'drv')
  return fails, cut, unconfirmed, (n_drv, len(by) - n_drv), sorted(needs)


def h2_confirm_jobs(head_dir, cand_dir, keys):
  """The CONFIRM_CELLS jobs H2 needs on this run's rows (keys: the closed job keys of the run). -> [(case, cell, start, mode)]"""
  hd, cd = closed_rows(head_dir, keys), closed_rows(cand_dir, keys)
  return h2([(k, hd[k], cd[k]) for k in sorted(set(hd) & set(cd))], {})[4]


# ---- comfort (an.py line()) ------------------------------------------------------------------------------------------------
def zigzag(t, s, h):
  pts, ext_i, direction = [], 0, 0
  for k in range(1, len(s)):
    if direction >= 0 and s[k] > s[ext_i]:
      ext_i = k
      if direction == 0 and s[k] - s[0] >= h:
        direction = 1
    elif direction <= 0 and s[k] < s[ext_i]:
      ext_i = k
      if direction == 0 and s[0] - s[k] >= h:
        direction = -1
    if direction == 1 and s[ext_i] - s[k] >= h:
      pts.append((float(t[ext_i]), float(s[ext_i]), 'peak'))
      direction, ext_i = -1, k
    elif direction == -1 and s[k] - s[ext_i] >= h:
      pts.append((float(t[ext_i]), float(s[ext_i]), 'valley'))
      direction, ext_i = 1, k
  return pts


def zig_pumps(t, s, h=0.08):
  if len(t) < 3:
    return 0
  z = zigzag(t, s, h)
  return sum(1 for i, p in enumerate(z) if p[2] == 'peak' and i > 0)


def first_line(r):
  tr, c = r['tr'], []
  for k in ('line_floor', 'aim_floor'):
    if k in tr:
      x = np.asarray(tr[k], dtype=float)
      kk = np.flatnonzero(np.isfinite(x))
      if len(kk):
        c.append(float(tr['t'][kk[0]]))
  return min(c) if c else None


def full_pumps(r, t0, t1):
  tr = r['tr']
  t = np.asarray(tr['t'], dtype=float)
  k0, k1 = int(np.searchsorted(t, t0)), int(np.searchsorted(t, t1)) + 1
  if k1 - k0 < 3:
    return 0, 0
  w = -np.asarray(tr['wire'], dtype=float)[k0:k1]
  a = np.nan_to_num(np.asarray(tr['a_real'], dtype=float))
  a5 = -np.convolve(a, np.ones(5) / 5, mode='full')[:len(a)][k0:k1]
  return zig_pumps(t[k0:k1], w), zig_pumps(t[k0:k1], a5)


def fullp(h, v):
  c = [x for x in (first_line(h), first_line(v), M(h, 't3'), M(v, 't3')) if x is not None]
  if not c:
    return None
  t0 = min(c)
  te = lambda r: M(r, 't_stop') if M(r, 't_stop') is not None else M(r, 't_vmin')  # noqa: E731
  return [full_pumps(r, t0, te(r)) if te(r) is not None else (0, 0) for r in (h, v)]


def valid_pair(h, v):
  gh, gv = (h.get('m') or {}).get('pre_start_plan_gap'), (v.get('m') or {}).get('pre_start_plan_gap')
  return (gv or 0.0) <= (gh or 0.0) + GAP_TOL + 1e-12


def _median(xs):
  """Median of the finite values (nan without a RuntimeWarning when there are none)."""
  a = np.array([x for x in xs if x is not None], dtype=float)
  a = a[np.isfinite(a)]
  return float(np.median(a)) if len(a) else float('nan')


def comfort(pairs):
  """pairs: [(h, v)] valid pairs. -> {aggregate: (head, cand, worse_pct)} with the 'worse' direction applied."""
  if not pairs:
    return {}
  fp = [fullp(h, v) for h, v in pairs]
  rest = lambda r: M(r, 'rest_at_stop')  # noqa: E731
  agg = {
    'entry_bites<=-0.10': [sum(1 for p in pairs if (M(p[i], 'entry_bite') or 0) <= -0.10) for i in (0, 1)],
    'full_pumps_wire': [sum(f[i][0] for f in fp if f) for i in (0, 1)],
    'full_pumps_plant': [sum(f[i][1] for f in fp if f) for i in (0, 1)],
    'a_stop<=-0.6': [sum(1 for p in pairs if (M(p[i], 'a_stop') or 0) <= -0.6) for i in (0, 1)],
    'j300_median': [_median([M(p[i], 'j300') for p in pairs]) for i in (0, 1)],
    'rest_4_5_share': [sum(1 for p in pairs if bucket(rest(p[i])) == '4-5') / len(pairs) for i in (0, 1)],
  }
  out = {}
  for k, (a, b) in agg.items():
    if not (np.isfinite(a) and np.isfinite(b)):
      out[k] = dict(head=a, cand=b, worse_frac=None, better=False, worse=False, worse_counts=None)
      continue
    up_is_worse = k != 'rest_4_5_share'
    worse = (b - a) if up_is_worse else (a - b)
    pct = (worse / abs(a)) if a else (np.inf if worse > 0 else 0.0)
    # counts worse: the count difference (pairs for the 4-5 m share); None for the j300 median (no count floor)
    cnt = None if k == 'j300_median' else round(float(worse * (len(pairs) if k == 'rest_4_5_share' else 1)), 2)
    out[k] = dict(head=round(a, 3), cand=round(b, 3), worse_frac=round(float(pct), 3) if np.isfinite(pct) else 'inf', better=worse < 0, worse=worse > 0,
                  worse_counts=cnt)
  return out


def comfort_verdict(agg):
  """WORSE when an aggregate is worse by > COMFORT_TOL and by >= COMFORT_MIN_COUNT counts (PLAN 82); else BETTER when more aggregates
  are better than worse."""
  if not agg:
    return 'NO DATA'
  if all(not a['better'] and not a['worse'] for a in agg.values()):
    return 'SAME'
  big = [k for k, a in agg.items() if a['worse'] and (a['worse_frac'] == 'inf' or (a['worse_frac'] or 0) > COMFORT_TOL)
         and (a.get('worse_counts') is None or a['worse_counts'] >= COMFORT_MIN_COUNT - 1e-9)]
  nb, nw = sum(a['better'] for a in agg.values()), sum(a['worse'] for a in agg.values())
  if big:
    return 'WORSE (' + ', '.join(big) + f' worse by > {COMFORT_TOL:.0%}, and by >= {COMFORT_MIN_COUNT} counts where counted)'
  return 'BETTER' if nb > nw else 'NOT BETTER'


# ---- F1 following matrix (radar-input changes; f1.py; builder C) ---------------------------------------------------------------
# Per (case, cell) the candidate run vs HEAD's, on physical truth. Numbers: README.md "F1" (HEAD's own spread over the measured radar
# latency p10-p90 is <= 0.16 m clearance, 0.04 m/s closing, 0.06 s onset, 0.34 s release; the limits are about 3x that).
F1_FLOOR_GAP = 2.0        # m: absolute floor on the minimum clearance (a candidate below it where HEAD is not FAILS)
F1_FLOOR_TTC = 1.5        # s: absolute floor on the minimum time to collision
F1_GAP_M, F1_GAP_FRAC = 0.5, 0.05   # FAIL: min clearance smaller than HEAD's by more than max(0.5 m, 5 % of HEAD's)
F1_TTC_FRAC = 0.10        # FAIL: min TTC (where HEAD's is < F1_TTC_MAX) smaller than HEAD's by more than 10 %
F1_TTC_MAX = 10.0         # s
F1_CLOSE = 0.2            # m/s: FAIL: max closing speed (or impact speed) larger than HEAD's by more than this
F1_ONSET = 0.10           # s: FAIL: brake onset (sustained sent demand) later than HEAD's by more than this (or none where HEAD has one)
F1_DEFICIT = 0.5          # m/s: FAIL: integral of max(sent_cand - sent_HEAD, 0) from the event to HEAD's closest approach above this
F1_RELEASE = 0.5          # s: FAIL: release earlier than HEAD's by more than this


def f1_pair(h, v):
  """One F1 (case, cell) pair: {'case', 'cell', 'fails': [names], 'd': {measure deltas}}."""
  a, b = h['f1'], v['f1']
  fails, d = [], {}
  if b['collision'] and (not a['collision'] or b.get('impact_v', 0.0) > a.get('impact_v', 0.0) + F1_CLOSE
                         or (b.get('t_impact') is not None and a.get('t_impact') is not None and b['t_impact'] < a['t_impact'] - F1_ONSET)):
    fails.append('collision')
  if b['min_gap'] < F1_FLOOR_GAP <= a['min_gap']:
    fails.append('floor_gap')
  if b['min_ttc'] < F1_FLOOR_TTC <= a['min_ttc']:
    fails.append('floor_ttc')
  d['gap'] = b['min_gap'] - a['min_gap']
  if d['gap'] < -max(F1_GAP_M, F1_GAP_FRAC * a['min_gap']):
    fails.append('min_gap')
  if a['min_ttc'] < F1_TTC_MAX:
    d['ttc'] = min(b['min_ttc'], F1_TTC_MAX) - a['min_ttc']
    if d['ttc'] < -F1_TTC_FRAC * a['min_ttc']:
      fails.append('min_ttc')
  d['closing'] = b['max_closing'] - a['max_closing']
  if d['closing'] > F1_CLOSE:
    fails.append('closing')
  if a['onset'] is not None:
    d['onset'] = None if b['onset'] is None else b['onset'] - a['onset']
    if b['onset'] is None or d['onset'] > F1_ONSET:
      fails.append('onset')
  ta, sa, sb = np.asarray(a['t'], dtype=float), np.asarray(a['sent'], dtype=float), np.asarray(b['sent'], dtype=float)
  n = min(len(sa), len(sb), int(np.searchsorted(ta, a.get('t_min_gap', np.inf), side='right')))
  d['deficit'] = float(np.sum(np.maximum(sb[:n] - sa[:n], 0.0)) * (ta[1] - ta[0])) if n > 1 else 0.0
  if d['deficit'] > F1_DEFICIT:
    fails.append('deficit')
  if a.get('release') is not None and b.get('release') is not None:
    d['release'] = b['release'] - a['release']
    if d['release'] < -F1_RELEASE:
      fails.append('release')
  return dict(case=h['case'], cell=h['cell'], fails=fails, d={k: (round(x, 3) if x is not None else None) for k, x in d.items()},
              head=dict(min_gap=round(a['min_gap'], 2), min_ttc=round(a['min_ttc'], 2), collision=a['collision']))


def f1(head, cand, off=None):
  """The F1 gate rows: (fails, evidence, coverage, OFF != HEAD keys). HEAD collisions are listed (the case is beyond the system there:
  only a worse impact counts)."""
  pairs = [f1_pair(head[k], cand[k]) for k in sorted(set(head) & set(cand))]
  fails = [dict(case=p['case'], cell=p['cell'], fails=p['fails'], d=p['d'], head=p['head']) for p in pairs if p['fails']]
  ev = [dict(case=k[0], cell=k[1], note='HEAD collides', impact_v=round(r['f1'].get('impact_v', 0.0), 2)) for k, r in sorted(head.items())
        if r['f1']['collision']]
  off_diff = [list(k) for k in sorted(set(head) & set(off or {})) if not same_row(head[k], off[k])]
  return fails, ev, len(pairs), off_diff


# ---- evaluation --------------------------------------------------------------------------------------------------------------
def gate(name, fails, evidence=None, coverage=None, note=None):
  v = 'PASS' if not fails else 'FAIL'
  if coverage == 0 and not fails:
    v = 'NO COVERAGE'
  return dict(gate=name, verdict=v, n_fail=len(fails), fails=fails[:50], evidence=(evidence or [])[:50], coverage=coverage, note=note)


def plan_problem(x):
  """Why a replay row's planner lockstep cannot be used (None = usable): no ticks, a non-finite aTarget / shouldStop, or a gap of
  more than PLAN_GAP without a tick over the replayed frames (frame time = t + origin; plan tick = ns). Rows with such a plan
  prevent a verdict (the run is INCOMPLETE)."""
  p = x.get('plan')
  if p is None or not len(p.get('ns', ())):
    return 'no planner ticks'
  if not (np.isfinite(np.asarray(p['at'], dtype=float)).all() and np.isfinite(np.asarray(p['ss'], dtype=float)).all()):
    return 'non-finite planner output'
  if x.get('origin') is None or not len(x['t']):
    return 'no replayed frame times'
  ft, pt = np.asarray(x['t'], dtype=float) + x['origin'], np.asarray(p['ns'], dtype=float) * 1e-9
  gap = float(np.max(np.diff(np.r_[ft[0], pt[(pt > ft[0]) & (pt < ft[-1])], ft[-1]])))
  return f'{gap:.2f} s without a planner tick over the replayed frames' if gap > PLAN_GAP else None


def plan_diff(x, y):
  """Planner-lockstep ticks where two replay arms' plans differ (aTarget, shouldStop); None when a row has no plan. Plans on
  different ticks (ns) are a difference, never compared tick by tick."""
  px, py = x.get('plan'), y.get('plan')
  if px is None or py is None:
    return None
  if len(px['at']) != len(py['at']) or not np.array_equal(px['ns'], py['ns']):
    return dict(plan_ticks=(len(px['at']), len(py['at'])))
  out = dict(plan_at=int(np.sum(~(px['at'] == py['at']))), plan_ss=int(np.sum(~(px['ss'] == py['ss']))))   # NaN never equals
  out.update({f'plan_{k}': int(np.sum(~(px[k] == py[k]))) for k in ('fcw', 'dts', 'dtsm', 'traj', 'trajv') if k in px and k in py})
  return out


def radar_diff(x, y):
  """Scored radard ticks where two replay arms' re-run published leads differ (radar stage rows; {} without one)."""
  mx, my = (x.get('radar') or {}).get('m1'), (y.get('radar') or {}).get('m1')
  if mx is None or my is None:
    return {}
  if len(mx['t']) != len(my['t']):
    return dict(radar_ticks=(len(mx['t']), len(my['t'])))
  d = np.zeros(len(mx['t']), bool)
  for k in mx:
    if k[:3] in ('l1_', 'l2_') and k[3:] in ('status', 'radar', 'radarTrackId', 'vLead', 'vLeadK', 'aLeadK'):
      d |= ~(np.nan_to_num(mx[k], nan=-99) == np.nan_to_num(my[k], nan=-99))
  return dict(radar_ticks=int(d.sum()))


def replay_compare(a, b):
  """Spans compared, per-column frames (and plan ticks) that differ between two replay arms."""
  out, n = [], 0
  for s in sorted(set(a) & set(b)):
    x, y = a[s], b[s]
    n += len(x['wire'])
    d = {c: int(np.sum(np.nan_to_num(x[c], nan=-99) != np.nan_to_num(y[c], nan=-99))) for c in REPLAY_COLS if len(x[c]) == len(y[c])}
    d.update(plan_diff(x, y) or {})
    d.update(radar_diff(x, y))
    if any(v if isinstance(v, int) else True for v in d.values()) or len(x['wire']) != len(y['wire']):
      out.append(dict(span=s, route=str(x['route'])[:8], diff=d))
  return out, n, len(set(a) & set(b)), sorted(set(a) ^ set(b))


def replay_code(cache={}):  # noqa: B006 -- per-process memo
  """Repo files of the production modules the exact replay imports (H1c compares HEAD with the log only on drives whose commit
  has the same content in these files)."""
  if 'files' not in cache:
    code = ('import sys, openpilot.selfdrive.controls.lib.longcontrol, opendbc.car.hyundai.interface, opendbc.car.hyundai.tests.test_can_bounds_fork; '
            + 'print("\\n".join(sorted(m.__file__ for m in list(sys.modules.values()) if getattr(m, "__file__", None))))')
    out = subprocess.run([sys.executable, '-c', code], capture_output=True, text=True, check=True, env=dict(os.environ, STOP_SIM_TREE='')).stdout
    root = str(REPO) + '/'
    files = set()
    for f in out.split():
      f = os.path.realpath(f)
      if f.startswith(root) and f.endswith('.py'):
        rel = f[len(root):]
        files.add('opendbc_repo/' + rel if rel.startswith('opendbc/') else rel)
    cache['files'] = sorted(x for x in files if x.startswith(('selfdrive/', 'frogpilot/', 'common/', 'opendbc_repo/')))
  return cache['files']


def same_code(commit, tree, cache={}):  # noqa: B006 -- per-process memo
  k = (commit, tree)
  if k not in cache:
    r = subprocess.run(['git', '-C', str(REPO), 'diff', '--quiet', commit, tree, '--', *replay_code()], capture_output=True)
    cache[k] = r.returncode == 0 if r.returncode in (0, 1) else None
  return cache[k]


def evaluate(meta, arms, rarms, expected=None, missing=None, quick=False, h_arms=None, frames_fn=None, confirm_arms=None, f1_arms=None):
  """The gate list on the rows of this run's job set (expected: {(kind, arm): job keys}; None = every row of the arms). missing
  (missing_jobs) makes the run INCOMPLETE. quick: no verdict (every gate INFO). h_arms: the history-trigger cell H rows (HEAD, ON),
  reported in their own non-gating section. confirm_arms: the H2 confirm-cell rows (HEAD, ON; run_gates runs h2_confirm_jobs)."""
  exp = expected or {}
  if frames_fn is None:
    from openpilot.tools.stopping.sim.replay import frames as frames_fn
  C = {k: closed_rows(d, exp.get(('closed', k), set()) if expected is not None else None) for k, d in arms.items()}
  RP = {k: {c: replay_rows(d, c, exp.get(('replay', k), set()) if expected is not None else None) for c in ('c1', 'c2')} for k, d in rarms.items()}
  head, on, off = C['HEAD'], C['ON'], C['OFF']
  res = dict(meta=meta, gates=[], comfort={}, changed=[], replay={}, per_case=[], notes=[], missing=missing or [], cell_h=None, h5_info=[])
  same_arm = meta['arms']['ON']['key'] == meta['arms']['HEAD']['key']
  if same_arm:
    res['notes'].append('ON == HEAD (same production code and flag file): every gate compares an arm with itself')
  if meta.get('census_excluded'):
    res['notes'].append(f"holds / replay spans left out of the job set ('error' in census/holds.json or frames/<corpus>.json): {meta['census_excluded']}")
  car = meta.get('car')
  if car and car.get('same_as_candidate'):
    res['notes'].append(f"the car {car['car_sha'][:10]} runs the candidate code (base + diff, its own flag values)")
  elif car and car.get('car_arm'):
    res['notes'].append(f"the car {car['car_sha'][:10]} runs the candidate code with the flag values of arm {'/'.join(car['car_arm'])}")
  elif car and not car['same_code']:
    res['notes'].append(f"the base is NOT the car code ({car['n_differ']} production files differ from car {car['car_sha'][:10]}"
                        + (f"; {len(car['cand_differ'])} from the candidate" if 'cand_differ' in car else '')
                        + '): H1 checks flags off == the base, not the car')
  pairs = [(k, head[k], on[k]) for k in sorted(set(head) & set(on))]
  if meta.get('recheck'):
    rc = meta['recheck']
    res['gates'].append(gate('D determinism (fresh HEAD re-runs == cache)', rc['differ'], coverage=rc['n'], note=f"{rc['identical']}/{rc['n']} identical"))
  res['meta']['rows'] = dict(HEAD=len(head), ON=len(on), OFF=len(off), pairs=len(pairs), missing=sum(len(m['keys']) for m in res['missing']))

  # replay rows whose planner lockstep is missing, non-finite or does not cover the frames: no verdict (H1a / H6 read the plans)
  for k, by_c in RP.items():
    bad = {f'{c}|{s}': why for c, rows in by_c.items() for s, x in sorted(rows.items()) for why in [plan_problem(x)] if why}
    if bad:
      res['missing'].append(dict(kind='replay plan invalid', arm=k, keys=[x.split('|') for x in bad], errors=bad))
  # H1 (a) exact replay OFF vs HEAD (+ planner lockstep); (b) closed OFF vs HEAD; (c) HEAD replay vs logged on same-code drives + car
  diffs, nfr, nsp, unpaired, nticks = [], 0, 0, [], 0
  for c in ('c1', 'c2'):
    d, n, ns, un = replay_compare(RP['HEAD'][c], RP['OFF'][c])
    diffs += [dict(x, corpus=c) for x in d]
    nfr, nsp, unpaired = nfr + n, nsp + ns, unpaired + un
    nticks += sum(len((x.get('plan') or {}).get('at', ())) for x in RP['HEAD'][c].values())
  off_pairs = [(k, head[k], off[k]) for k in sorted(set(head) & set(off))]
  off_diff = [dict(case=k[0], cell=k[1], start=k[2], mode=k[3]) for k, h, v in off_pairs if not same_row(h, v)]
  fid, errs, fid_frames = [], [], 0
  for c in ('c1', 'c2'):
    for s, x in RP['HEAD'][c].items():
      if x.get('commit') and same_code(str(x['commit']), meta['arms']['HEAD']['tree']):
        t = np.asarray(x['t'], dtype=float)
        act = (x['act'] > 0) & (t >= t[0] + FID_SKIP_S)
        if act.sum() < 2:
          continue
        e = np.abs(x['wire'] - x['rec'])[act]
        fid_frames += int(act.sum())
        errs.append((float(np.mean(e)), float(np.max(e))))
        if np.mean(e) > FID_MEAN:
          fid.append(dict(corpus=c, span=s, active_frames=int(act.sum()), mean_abs=round(float(np.mean(e)), 4), max_abs=round(float(np.max(e)), 3)))
  me = np.array(errs) if errs else np.zeros((0, 2))
  res['gates'].append(gate('H1a replay OFF == HEAD', diffs, coverage=nsp,
                           note=f'{nsp} spans, {nfr} frames, columns {",".join(REPLAY_COLS)}; planner lockstep {nticks} ticks (aTarget, shouldStop); '
                                + f'unpaired spans {len(unpaired)}'))
  res['gates'].append(gate('H1b closed OFF == HEAD', off_diff, coverage=len(off_pairs), note=f'{len(off_pairs)} runs (trace sha + metrics)'))
  car_note = '' if not car else (f"CAR {car['car']} = {car['car_sha'][:10]}: production code " + ('== base' if car['same_code'] else
                                 f"!= base ({car['n_differ']} files, e.g. {', '.join(car['differ'][:3])})") + '; ')
  res['gates'].append(dict(gate('H1c HEAD replay vs logged command (same-code drives) + car check', [], evidence=fid, coverage=len(errs)), verdict='INFO',
                           note=car_note + f'{len(errs)} spans / {fid_frames} active frames (first {FID_SKIP_S:g} s of each span skipped) on drive commits '
                                + 'whose replayed modules equal the HEAD tree; '
                                + (f'span mean |err| median {np.median(me[:, 0]):.4f}, p90 {np.percentile(me[:, 0], 90):.4f}; max |err| median '
                                   + f'{np.median(me[:, 1]):.3f}, p90 {np.percentile(me[:, 1], 90):.3f}, worst {me[:, 1].max():.3f}; '
                                   if len(me) else '')
                                + f'{len(fid)} spans with mean |err| > {FID_MEAN} listed (tolerance not decided: not gating)'))

  # radar stage (cycle_20261006 builder R): R1 radard fidelity (INCOMPLETE on a scored mismatch), R3 FrogPilot fidelity (INFO), M1
  rbad, r1, r3 = radar_fidelity(RP['HEAD'], meta['arms']['HEAD']['tree'])
  if rbad:
    res['missing'].append(dict(kind='radar replay fidelity', arm='HEAD', keys=[k.split('|') for k in sorted(rbad)], errors=rbad))
  mv, mrows = m1(RP['HEAD'], RP['ON'])
  judged = [r for r in mrows if r['verdict'] not in ('EMPTY', 'INFO')]
  res['gates'] += [r1, r3, fp_compare(RP['HEAD'], RP['ON'])]
  res['gates'] += [dict(gate('M1 radar measurement (published lead vs truth, candidate vs HEAD)', [r for r in mrows if r['verdict'] == 'FAIL'],
                                     evidence=[r for r in mrows if r['verdict'] != 'FAIL'], coverage=len(judged)), verdict=mv,
                                note=f'{len(mrows)} classes ({len(judged)} judged with >= {M1_N_MIN} ticks in both arms; INFO rows not gating); '
                                     + 'truth: stationary = 0 '
                                     + '(geometry-qualified), moving = REL_SPEED shifted by the lag x scale envelope (9 corners) + ego speed; '
                                     + f'FAIL: candidate more optimistic than HEAD beyond v {M1_TOL_V}, a {M1_TOL_A}, stationary {M1_TOL_STATIC} '
                                     + f'at every corner, or stationary optimistic episodes (> {M1_STAT_V} m/s for >= {M1_STAT_S} s on one track): more '
                                     + "than HEAD's (count or seconds) or a new one longer / higher than HEAD's worst; UNCERTAIN (some corners) and EMPTY "
                                     + 'classes are not PASS (NO COVERAGE)')]

  # H2: worst cell per (case, start) over H2_CELLS; nodrv runs: minimum gap after the first stop; brake-off band rule (PLAN 82)
  cr = {k: closed_rows(d, exp.get(('confirm', k), set()) if expected is not None else None) for k, d in (confirm_arms or {}).items()}
  conf = {k: (cr['HEAD'][k], cr['ON'][k]) for k in set(cr.get('HEAD', {})) & set(cr.get('ON', {}))}
  f2, cut, unconf, (n_grp, n_nodrv), needs = h2(pairs, conf)
  if needs:
    res['missing'].append(dict(kind='confirm', arm='HEAD+ON', keys=[list(x) for x in needs], errors={}))
  res['gates'].append(gate('H2 no new rest / min gap < 3.0 m', f2, evidence=[dict(x, note='stated close cut-in (listed, not gating)') for x in cut] + unconf,
                           coverage=n_grp + n_nodrv,
                           note=f'{n_grp} (case, start) worst-cell groups + {n_nodrv} nodrv runs; min gap median over {DESPIKE_S:g} s; a failure only '
                                + f'in {"/".join(BAND_CELLS)} in the 1st-gear brake-off band after a launch counts if {" or ".join(CONFIRM_CELLS)} agrees '
                                + f'({len(conf)} confirm runs, {len(unconf)} not confirmed: listed)'))

  # H3: model-only stops (the ms family + every drv run where HEAD stops with no radar lead in the 2 s before the wheel stop): the
  # candidate stops too, at most STOP_PAST beyond HEAD's stop position
  h3, ev3, cov3, n_ms = [], [], 0, 0
  for k, h, v in pairs:
    if k[3] != 'drv':
      continue
    ms = h.get('group') == 'ms'
    ts = M(h, 't_stop')
    if ts is None:
      if ms:
        ev3.append(dict(case=k[0], cell=k[1], note='HEAD does not stop'))
      continue
    if not ms:
      if 'lead_status' not in h['tr']:
        continue
      t = np.asarray(h['tr']['t'], dtype=float)
      m = (t >= ts - 2.0) & (t <= ts)
      if not m.any() or (np.asarray(h['tr']['lead_status'], dtype=float)[m] > 0).any():
        continue
    cov3 += 1
    n_ms += ms
    if M(v, 't_stop') is None:
      h3.append(dict(case=k[0], cell=k[1], start=k[2], check='candidate does not stop', head_stop=round(ts, 2)))
      continue
    xh, xv = stop_x(h), stop_x(v)
    if xh is not None and xv is not None and xv - xh > STOP_PAST:
      h3.append(dict(case=k[0], cell=k[1], start=k[2], check=f'stops > {STOP_PAST} m past HEAD', past=round(xv - xh, 2)))
  res['gates'].append(gate('H3 model-only stops honoured', h3, evidence=ev3, coverage=cov3,
                           note=f'{n_ms} ms-family runs + {cov3 - n_ms} recorded / synthetic runs where HEAD stops without a radar lead in the last 2 s; '
                                + f'FAIL: no stop, or a stop > {STOP_PAST} m past HEAD'))

  # H4 / H5 / H6 on every pair; the changed-run list (E3-style) on the nodrv runs
  h4, h5, h6, ch, h5i, h4i, unc = [], [], [], [], [], [], []
  n5 = dict(judged=0, identical=0, not_judged=0)
  for k, h, v in pairs:
    # the window both arms share, by time (each arm's own indices of the same instants; PLAN 72 compares at the same time)
    xh, (k0h, kdh), xv, (k0, kd), nu = common_window(h, v)
    if nu:
      unc.append(dict(case=k[0], cell=k[1], start=k[2], frames=nu))
    for name, fn in (('StopReq chatter (clear -> set at rest within 1 s)', chatter), ('hold at rest in pid', pid_holds)):
      a, b = fn(xh, k0h, kdh), fn(xv, k0, kd)
      if len(b) > len(a):
        h4.append(dict(case=k[0], cell=k[1], start=k[2], check=name, head=a[:5], cand=b[:5]))
    a, b = start_under_hold(xh, k0h, kdh), start_under_hold(xv, k0, kd)
    if b > a:   # more conflict frames than HEAD (PLAN 86; was: only where HEAD has none)
      h4.append(dict(case=k[0], cell=k[1], start=k[2], check="'starting' under a hold (service-owned or command reversal)", head=a, cand=b))
    ia, ib = stopreq_info(xh, k0h, kdh), stopreq_info(xv, k0, kd)
    if len(ib) > len(ia):
      h4i.append(dict(case=k[0], cell=k[1], start=k[2], note='information (not gating)', head=ia[:3], cand=ib[:3]))
    fa, fb = [x for x in launches(xh, k0h, kdh) if x[3]], [x for x in launches(xv, k0, kd) if x[3]]
    if len(fb) > len(fa):
      h5.append(dict(case=k[0], cell=k[1], start=k[2], check=f'false launch (lead < {LEAD_STOPPED_V} m/s through the launch)', head=fa[:3], cand=fb[:3]))
    if same_row(h, v):
      rm = _h5_ends(h)[2]
      if rm[0] is not None and rm[1] is not None:
        n5['judged'] += 1
        n5['identical'] += 1
      continue
    fails, info = h5_gap(h, v)
    h5 += [dict(case=k[0], cell=k[1], start=k[2], **f) for f in fails]
    if not info['judged']:
      n5['not_judged'] += 1
      h5i.append(dict(case=k[0], cell=k[1], start=k[2], note=f"not judged: {info['rest']}"))
    else:
      n5['judged'] += 1
      if not fails and (info.get('dgap') or 0) > GAP_MOTION:
        h5i.append(dict(case=k[0], cell=k[1], start=k[2], note=f"rests further back, launches in time: gap at motion {info['dgap']:+.2f} m, growth "
                                                                + f"{info['dgrowth']:+.2f} m, first motion {info['dt']:+.2f} s (information)"))
      elif not fails and info.get('dt') is not None and info['dt'] > LAUNCH_LATE:
        h5i.append(dict(case=k[0], cell=k[1], start=k[2], note=f"first motion {info['dt']:+.2f} s later at gap {info.get('dgap')} m, growth "
                                                                + f"{info.get('dgrowth')} m vs HEAD (information)"))
    kdiv = first_div(xh, xv, k0, kd)   # a candidate frame; khd = HEAD's frame at the same instant
    if kdiv is not None:
      t_div = float(xv['t'][kdiv])
      khd = int(np.searchsorted(np.asarray(xh['t'], dtype=float), t_div - 1e-6))
      mh, mv = first_motion(xh, khd, kdh), first_motion(xv, kdiv, kd)
      dtm = round(float(xv['t'][mv] - xh['t'][mh]), 2) if mh is not None and mv is not None else None
      lh, lv = first_motion(xh, khd, kdh, 0.5), first_motion(xv, kdiv, kd, 0.5)
      dtl = round(float(xv['t'][lv] - xh['t'][lh]), 2) if lh is not None and lv is not None else None
      jb = owned_release_breaches(xh, xv, k0, kd)
      if jb:
        h6.append(dict(case=k[0], cell=k[1], start=k[2], frames=len(jb), first=jb[:3]))
      if k[3] == 'nodrv':
        ch.append(dict(case=k[0], cell=k[1], t_div=round(t_div, 2), reholds_head=reholds(xh, k0h, kdh), reholds_cand=reholds(xv, k0, kd),
                       dt_motion=dtm, dgap_motion=round(float(xv['gap'][mv] - xh['gap'][mh]), 2) if dtm is not None else None, dt_launch=dtl,
                       chatter=(len(chatter(xh, k0h, kdh)), len(chatter(xv, k0, kd))), start_under_hold=(a, b),
                       min_gap=(min_gap_after_stop(xh, khd, kdh), min_gap_after_stop(xv, kdiv, kd))))
  # recorded launch timing on drv pairs (x.t_launch): information only (PLAN 75: the gap at first motion decides)
  for k, h, v in pairs:
    if k[3] == 'drv' and M(h, 't_launch') is not None and M(v, 't_launch') is not None and M(v, 't_launch') - M(h, 't_launch') > LAUNCH_LATE \
       and not M(h, 'launch_by_driver'):
      h5i.append(dict(case=k[0], cell=k[1], start=k[2], note=f"launch {M(v, 't_launch') - M(h, 't_launch'):+.2f} s later (information)"))
  ev, rj, nrh, srd, pdiff, unpaired_on = [], [], [], 0, [], []
  for c in ('c1', 'c2'):
    for s in sorted(set(RP['ON'][c]) & set(RP['OFF'][c])):
      x, y = RP['OFF'][c][s], RP['ON'][c][s]
      pd = plan_diff(x, y)
      jb = []
      if pd and any(pd.values()):   # the planner change: the release bound on the plan's braking component per tick
        pdiff.append(dict(corpus=c, span=s, **pd))
        jb += [(t, r, 'plan braking component') for t, r in plan_release_breaches(x, y, lambda c=c, s=s: frames_fn(c, s))]
      if len(x['t']) != len(y['t']) or not np.array_equal(x['t'], y['t']):
        unpaired_on.append([c, s])
        continue
      d = (np.abs(x['wire'] - y['wire']) > 1e-9) | (x['phase'] != y['phase']) | (x['stopreq'] != y['stopreq']) | (x['lcs'] != y['lcs'])
      if d.any():
        srd += int(np.sum(x['stopreq'] != y['stopreq']) + np.sum(x['owning'] != y['owning']))
        ev.append(dict(corpus=c, span=s, route=str(y['route'])[:8], frames=int(d.sum()), t0=round(float(y['t'][np.argmax(d)]), 2),
                       max_dw=round(float(np.max(np.abs(x['wire'] - y['wire']))), 3)))
        # ... and, independently, the releases the service owns on the command (a planner change does not exempt them)
        jb += [(t, r, 'service-owned command') for t, r, _ in owned_release_breaches(x, y, 0, len(x['wire']))]
      if jb:
        rj.append(dict(corpus=c, span=s, check=', '.join(sorted({kind for *_, kind in jb})), frames=len(jb), first=sorted(jb)[:3]))
        rh_off = {round(float(x['t'][k]), 2) for k in range(1, len(x['phase'])) if x['phase'][k] == RAMP and x['phase'][k - 1] == RELEASE}
        for k in range(1, len(y['phase'])):
          if y['phase'][k] == RAMP and y['phase'][k - 1] == RELEASE and round(float(y['t'][k]), 2) not in rh_off:
            nrh.append((str(y['route'])[:8], round(float(y['t'][k]), 2), c, s))
  h6 += [dict(r, source='replay') for r in rj]
  if unpaired_on:   # the same span replayed on different frames: not comparable, no verdict
    res['missing'].append(dict(kind='replay ON / OFF frames differ', arm='ON+OFF', keys=unpaired_on, errors={}))
  res['gates'].append(gate('H4 StopReq / ownership', h4, evidence=h4i, coverage=len(pairs),
                           note=f'StopReq clear -> set at rest within {CHATTER_S:g} s; service hold at rest in pid > {PID_HOLD_S} s; '
                                + "'starting' under RAMP_TO_HOLD / HOLD on service-owned frames or a command reversal (PLAN 82); "
                                + f'listed (not gating): new StopReq sets in RELEASE and clear -> set just after the window ({len(h4i)} runs); '
                                + f'replay ON vs OFF: {srd} StopReq/ownership frame differences'))
  res['gates'].append(gate('H5 launches', h5, evidence=h5i, coverage=n5['judged'],
                           note=f"FAIL: false launch (ego > {FALSE_MOVE} m from rest toward a lead < {LEAD_STOPPED_V} m/s through the launch), "
                                + f'candidate never moves where HEAD does, gap growth since the rest > {GAP_MOTION} m more than HEAD, or first motion '
                                + f'> {LAUNCH_LATE} s later at a gap > {GAP_MOTION} m larger (PLAN 82); judged {n5["judged"]} pairs ({n5["identical"]} '
                                + f'identical), {n5["not_judged"]} changed pairs not judged (listed)'))
  res['gates'].append(gate('H6 J-limited releases', h6, coverage=len(pairs),
                           note=f'J_MAX {J_MAX} m/s^3 = {J_MAX * PLAN_DT:.3f} per planner tick on the braking component of releases the change owns '
                                + '(PLAN 82): closed traces: service-owned command frames and line-floor-binding plan ticks where the candidate '
                                + 'differs from HEAD (at the same instants); replay spans: service-owned command frames and, with a plan change, the plan '
                                + 'braking component per tick (engaged, no pedal); LongControl starting frames exempt; '
                                + f'{len(unc)} closed pairs with candidate frames outside the stored HEAD trace (not compared, meta.uncovered)'))
  holds_new = sorted({(r, round(t, 1)) for r, t, _, _ in nrh})
  res['replay'] = dict(events=ev[:100], n_event_spans=len(ev), new_reholds=nrh, new_rehold_holds=holds_new, plan_diff_spans=len(pdiff),
                       plan_diff=pdiff[:100])
  res['changed'] = ch
  nod = [p for p in pairs if p[0][3] == 'nodrv']
  res['meta']['nodrv'] = dict(pairs=len(nod), identical=sum(window_same(h, v) for _, h, v in nod))
  res['meta']['uncovered'] = unc

  # COMFORT on NEW + history, L42 + L42s, start v12, valid pairs (by group and together; the verdict on both groups together)
  cp = [(h, v) for k, h, v in pairs if k[3] == 'drv' and k[1] in COMFORT_CELLS and k[2] == 'v12' and h['group'] in ('new', 'hist') and valid_pair(h, v)]
  agg = comfort(cp)
  res['comfort'] = dict(n_valid=len(cp), aggregates=agg, verdict=comfort_verdict(agg), tol=COMFORT_TOL,
                        by_group={g: dict(n_valid=len(pp), aggregates=comfort(pp)) for g in ('new', 'hist')
                                  for pp in [[p for p in cp if p[0]['group'] == g]]})
  # per-case regressions (worst cell over the comfort cells; listed for the car trial, not gating)
  byc = defaultdict(list)
  for k, h, v in pairs:
    if k[3] == 'drv' and k[1] in COMFORT_CELLS and valid_pair(h, v):
      byc[(k[0], k[2])].append((h, v))
  for (case, st), lst in sorted(byc.items()):
    def w(fn, key, i, lst=lst):
      x = [M(p[i], key) for p in lst]
      x = [y for y in x if y is not None and np.isfinite(y)]
      return fn(x) if x else None
    reasons = []
    ah, av = w(min, 'a_stop', 0), w(min, 'a_stop', 1)
    if ah is not None and av is not None and av < ah - 0.1 and av <= -0.6:
      reasons.append(f'a_stop {ah:.2f}->{av:.2f}')
    jh, jv = w(max, 'j300', 0), w(max, 'j300', 1)
    if jh is not None and jv is not None and jv > jh + 0.5:
      reasons.append(f'j300 {jh:.2f}->{jv:.2f}')
    rh = [bucket(M(p[0], 'rest_at_stop')) for p in lst]
    rv = [bucket(M(p[1], 'rest_at_stop')) for p in lst]
    if sum(b == '4-5' for b in rv) < sum(b == '4-5' for b in rh):
      reasons.append(f'rest out of 4-5 m {Counter(rh)}->{Counter(rv)}')
    fp = [fullp(h, v) for h, v in lst]
    pw = (max([f[0][0] for f in fp if f], default=0), max([f[1][0] for f in fp if f], default=0))
    if pw[1] > pw[0]:
      reasons.append(f'full pumps wire {pw[0]}->{pw[1]}')
    if reasons:
      res['per_case'].append(dict(case=case, start=st, reasons=reasons))

  # F1 following matrix (f1_arms: HEAD, ON, OFF rows in arms/<key>/f1/; builder C)
  if f1_arms:
    fr = {k: closed_rows(d, exp.get(('f1', k), set()) if expected is not None else None) for k, d in f1_arms.items()}
    ff, fev, fcov, foff = f1(fr['HEAD'], fr['ON'], fr.get('OFF'))
    res['gates'].append(gate('F1 following (physical truth, 20-30 m/s)', ff, evidence=fev, coverage=fcov,
                             note=f'{fcov} (case, cell) pairs; floors clearance {F1_FLOOR_GAP:g} m / TTC {F1_FLOOR_TTC:g} s; vs HEAD: clearance '
                                  + f'-max({F1_GAP_M:g} m, {F1_GAP_FRAC:.0%}), TTC -{F1_TTC_FRAC:.0%}, closing +{F1_CLOSE:g} m/s, onset +{F1_ONSET:g} s, '
                                  + f'deficit {F1_DEFICIT:g} m/s, release -{F1_RELEASE:g} s; {len(fev)} HEAD collisions listed'))
    res['gates'].append(gate('H1b-F1 OFF == HEAD (F1 runs)', foff, coverage=len(set(fr['HEAD']) & set(fr.get('OFF') or {}))))

  # the history-trigger cell H (--with-h): its own rows, never gating
  if h_arms:
    hh, hv = (closed_rows(h_arms[k], exp.get(('cellH', k), set()) if expected is not None else None) for k in ('HEAD', 'ON'))
    hp = [(hh[k], hv[k]) for k in sorted(set(hh) & set(hv))]
    hc = [(h, v) for h, v in hp if h['group'] in ('new', 'hist') and h['start'] == 'v12' and valid_pair(h, v)]
    res['cell_h'] = dict(pairs=len(hp), changed=sum(not same_row(h, v) for h, v in hp), comfort_valid=len(hc), aggregates=comfort(hc))

  if quick:
    for g in res['gates']:
      g['verdict'] = 'INFO'
    res['comfort']['verdict'] = 'n/a (quick)'
    res['verdict'] = 'NO VERDICT (quick: iteration only)'
  elif res['missing']:
    res['verdict'] = 'INCOMPLETE'
  elif any(g['verdict'] == 'FAIL' for g in res['gates']):
    res['verdict'] = 'FAIL'
  else:
    nc = [g['gate'].split()[0] for g in res['gates'] if g['verdict'] == 'NO COVERAGE']
    res['verdict'] = f"NO COVERAGE ({', '.join(nc)})" if nc else 'PASS'
  res['h5_info'] = h5i
  return res


def render(res, short=False):
  m = res['meta']
  car = m.get('car') or {}
  L = [f"# Stopping sim gate: {m['run']}", '', f"**VERDICT: {res.get('verdict')}**" + (' (hard gates; COMFORT is reported separately)'
                                                                                         if res.get('verdict') in ('PASS', 'FAIL') else ''), '',
       f"base {m['base']} ({m['base_sha'][:10]}), diff {m.get('diff')} (sha1 {m.get('diff_sha1')}), touched {len(m.get('touched') or [])} files",
       (f"car {car.get('car')} ({str(car.get('car_sha'))[:10]}): production code {'== base' if car.get('same_code') else '!= base'}"
        + (f" ({car.get('n_differ')} files)" if car and not car.get('same_code') else '')
        + ('' if 'same_as_candidate' not in car else ', == candidate' if car['same_as_candidate'] else
           f", == candidate code with arm {'/'.join(car['car_arm'])} flags" if car.get('car_arm') else ', != candidate')) if car else 'car: not checked',
       f"flags ON {m['flags_on']} OFF {m['flags_off']}; arms " + ', '.join(f"{k} {v['key']} (flag file {str(v.get('flags_sha1'))[:10]})"
                                                                     for k, v in m['arms'].items()),
       f"engine {m['engine']} (replay {m['engine_replay']}); spec {m['spec']}; rows {m.get('rows')}; timing {m.get('timing_s')} s; "
       + f"wall {m.get('wall_s')} s", '']
  L += [f'- NOTE: {n}' for n in res['notes']]
  if res.get('missing'):
    L += ['', f"## INCOMPLETE: {sum(len(x['keys']) for x in res['missing'])} jobs failed or missing (the gates below are not a verdict)", '']
    for x in res['missing']:
      L.append(f"- {x['kind']} {x['arm']}: {len(x['keys'])} jobs, e.g. {x['keys'][:5]}")
      L += [f'  - {k}: {e}' for k, e in list(x['errors'].items())[:10]]
  L += ['', '| gate | verdict | fails | coverage | note |', '|---|---|---|---|---|']
  for g in res['gates']:
    L.append(f"| {g['gate']} | {g['verdict']} | {g['n_fail']} | {g['coverage']} | {g.get('note') or ''} |")
  c = res['comfort']
  L += ['', f"COMFORT ({c.get('n_valid')} valid pairs, NEW + history, L42 + L42s, v12): **{c.get('verdict')}**", '',
        '| aggregate | HEAD | candidate | worse by | NEW HEAD -> cand | history HEAD -> cand |', '|---|---|---|---|---|---|']
  bg = c.get('by_group') or {}
  for k, a in (c.get('aggregates') or {}).items():
    gs = [bg.get(g, {}).get('aggregates', {}).get(k) for g in ('new', 'hist')]
    L.append(f"| {k} | {a['head']} | {a['cand']} | {a['worse_frac']} ({a.get('worse_counts')} counts) | "
             + ' | '.join(f"{x['head']} -> {x['cand']}" if x else '-' for x in gs) + ' |')
  if bg:
    L.append(f"(valid pairs: NEW {bg.get('new', {}).get('n_valid')}, history {bg.get('hist', {}).get('n_valid')})")
  nd = m.get('nodrv') or {}
  rp = res['replay']
  L += ['', f"nodrv runs (census holds + R.NEW, driver removed): {nd.get('identical')}/{nd.get('pairs')} identical; "
        + f"replay: {rp.get('n_event_spans')} spans with ON != OFF ({rp.get('plan_diff_spans')} with a plan difference); new RELEASE -> "
        + f"RAMP_TO_HOLD re-holds on {len(rp.get('new_rehold_holds', []))} recorded holds: {rp.get('new_rehold_holds')}"]
  if res.get('cell_h'):
    h = res['cell_h']
    L += ['', f"cell H (history trigger, --with-h; NOT gating): {h['pairs']} pairs, {h['changed']} changed; comfort pairs {h['comfort_valid']}: "
          + '; '.join(f"{k} {a['head']} -> {a['cand']}" for k, a in h['aggregates'].items())]
  if short:
    return '\n'.join(L)
  L += ['', '## Changed nodrv runs', '']
  L += [f"- {x['case']} div {x['t_div']} reholds {x['reholds_head']}->{x['reholds_cand']} dt_motion {x['dt_motion']} dgap {x['dgap_motion']} "
        + f"dt_launch {x['dt_launch']} chatter {x['chatter']} starting-under-hold {x['start_under_hold']} min_gap {x['min_gap']}" for x in res['changed']]
  for g in res['gates']:
    if g['fails'] or g['evidence']:
      L += ['', f"## {g['gate']}: {g['verdict']}", '']
      L += [f'- FAIL {x}' for x in g['fails']] + [f'- {x}' for x in g['evidence']]
  L += ['', '## Replay events (ON vs OFF)', ''] + [f'- {x}' for x in res['replay'].get('events', [])[:60]]
  L += ['', '## Per-case regressions (not gating; for the car trial)', ''] + [f"- {x['case']} {x['start']}: {'; '.join(x['reasons'])}" for x in res['per_case']]
  return '\n'.join(L) + '\n'


# ---- radar stage gates (cycle_20261006 builder R): R1 radard replay fidelity, R3 FrogPilot fidelity (INFO), M1 measurement -----
RADAR_FILES = ('selfdrive/controls/radard.py', 'selfdrive/controls/lib/longitudinal_mpc_lib/stop_target_helpers.py', 'common/simple_kalman.py',
               'common/filter_simple.py', 'selfdrive/controls/lib/desire_helper.py')   # radard's publication path (R1 matching build)
FP_FILES = tuple(f'frogpilot/controls/{f}.py' for f in ('frogpilot_planner', 'lib/conditional_experimental_mode', 'lib/frogpilot_following',
                                                         'lib/frogpilot_traffic', 'lib/frogpilot_acceleration', 'lib/frogpilot_events'))
M1_N_MIN = 100            # ticks: a class with fewer (in either arm) is EMPTY (not PASS)
M1_TOL_V = dict(mean=0.05, p95=0.05, p99=0.08)          # m/s: candidate - HEAD signed error (pub - truth): more optimistic FAILS
M1_TOL_A = dict(mean=0.10, p95=0.15, p99=0.25)          # m/s^2, aLeadK
M1_TOL_STATIC = dict(mae=0.01, rms=0.01, p99abs=0.03)   # m/s, stationary truth 0 (vLead / vLeadK)
M1_EXC_V, M1_EXC_S = 0.1, 0.3   # sustained optimistic excursion: pub - truth > 0.1 m/s for >= 0.3 s on one track
M1_EXC_TOL = (2, 1.0, 0.10)     # FAIL: candidate excursions > HEAD + max(2, 10 %) or seconds > HEAD + max(1.0 s, 10 %)
# stationary leads (truth 0, geometry): an optimistic episode = published > M1_STAT_V for >= M1_STAT_S on one track; FAIL on more
# episodes or episode-seconds than HEAD, or a candidate episode that no HEAD episode of the same span and track overlaps and that is
# longer or higher than HEAD's worst (pooled MAE / RMS / p99 hide a short sustained error among ~250k stationary ticks: Astra tooling
# review finding 4)
M1_STAT_V, M1_STAT_S = 0.15, 0.3
M1_DT = 0.05              # s, radard tick
M1_EGO_BINS = ((-np.inf, -1.5), (-1.5, -0.5), (-0.5, np.inf))   # m/s^2, ego accel (lo, hi]: INFO split of the stationary leadOne


def same_build(commit, tree, files, cache={}):  # noqa: B006 -- per-process memo
  """The drive commit has the HEAD tree's content in files and the same value of every stopping flag they read (None: the commit is
  not in this repository). The radard / FrogPilot import closures (~180 files) differ on every drive in the corpora, mostly in
  stopping_flags.py and in frogpilot_variables.py's toggle defaults, which the replay takes from the log."""
  k = (commit, tree, files)
  if k not in cache:
    r = subprocess.run(['git', '-C', str(REPO), 'diff', '--quiet', commit, tree, '--', *files], capture_output=True)
    same = r.returncode == 0 if r.returncode in (0, 1) else None
    if same:
      from openpilot.tools.stopping.sim import loader
      names = sorted({n for f in files for n in re.findall(r'stopping_flags\.([A-Z][A-Z0-9_]*)', (REPO / f).read_text())})
      fv = [loader.flag_values(loader.git('show', f'{rev}:{loader.FLAGS_FILE}').decode()) for rev in (commit, tree)]
      same = all(fv[0].get(n) == fv[1].get(n) for n in names)
    cache[k] = same
  return cache[k]


def radar_fidelity(RP_head, tree, same=same_build):
  """R1 + R3 on the HEAD replay rows. R1: on drives whose commit has HEAD's radard code, a re-run radarState tick that differs from
  the logged one inside the scored window (the replayed frames; discrete fields exact, floats within radar_replay.FLOAT_TOL and finite,
  KF state within radar_replay.KF_TOL on race-affected ticks only; cold-start ticks are not exempt) -> the span is INCOMPLETE; on every
  span, a frame whose logged lead inputs do not equal the logged radarState it is mapped to -> INCOMPLETE. Returns (incomplete
  {corpus|span: why}, R1 gate, R3 gate)."""
  from openpilot.tools.stopping.sim.radar_replay import FLOAT_TOL, KF_TOL
  bad, other, cov, stats = {}, [], 0, dict(ticks=0, scored=0, race=0, cold=0, no_toggles=0, frames=0, tol_n=0, tol_max=0.0)
  fp_tot, fp_cov, fp_ticks, n_rows = Counter(), 0, 0, 0
  for c, rows in sorted(RP_head.items()):
    for s, x in sorted(rows.items()):
      if 'radar' not in x:   # a row without the radar stage (synthetic test rows): nothing to check
        continue
      n_rows += 1
      rd = x['radar']
      key = f'{c}|{s}'
      if rd is None:
        bad[key] = 'no radar stage (no rlog events)'
        continue
      f = rd['fid']
      n_fr = len(x['wire'])
      stats.update(ticks=stats['ticks'] + f['ticks'], scored=stats['scored'] + f['scored'], race=stats['race'] + f['n_race'],
                   cold=stats['cold'] + f['cold_kf_scored'], no_toggles=stats['no_toggles'] + f['n_no_toggles'], frames=stats['frames'] + n_fr,
                   tol_n=stats['tol_n'] + f['race_tol_scored'], tol_max=max(stats['tol_max'], f['race_tol_max']))
      if rd['kw_matched'] != n_fr:
        bad[key] = f"{n_fr - rd['kw_matched']} of {n_fr} frames: logged lead inputs != the mapped logged radarState"
        continue
      sb = same(str(x.get('commit')), tree, RADAR_FILES) if x.get('commit') else None
      if sb:
        cov += 1
        if f['bad_scored']:
          bad[key] = f"{f['bad_scored']} scored ticks differ from the logged radarState: {f['per_field']}"
      elif f['bad_scored']:
        other.append(dict(corpus=c, span=s, commit=str(x.get('commit'))[:10], bad_scored=f['bad_scored'], fields=sorted(f['per_field'])[:6]))
      if x.get('commit') and same(str(x['commit']), tree, FP_FILES):
        fp_cov += 1
        fp_ticks += rd['fp_fid']['ticks']
        fp_tot.update(rd['fp_fid']['per_field'])
  r1 = dict(gate('R1 radard replay fidelity (HEAD re-run vs logged radarState)', [dict(span=k, why=v) for k, v in sorted(bad.items())],
                 evidence=other, coverage=cov),
            note=f"{cov} spans on drives with HEAD's radard code ({stats['scored']} scored ticks of {stats['ticks']}; discrete exact, floats "
                 + f"<= {FLOAT_TOL:g} and finite; KF state (vLeadK / aLeadK / aLeadTau) <= {KF_TOL:g} on liveTracks-race-affected ticks only: "
                 + f"{stats['tol_n']} scored ticks used it, worst {stats['tol_max']:.4f}; mismatch -> INCOMPLETE); {len(other)} spans of other "
                 + f"builds with differences listed (not gating); liveTracks races settled by the logged leads {stats['race']}; cold-start KF "
                 + f"mismatches {stats['cold']} field-ticks (INCOMPLETE, not exempt); ticks before the first toggles {stats['no_toggles']}; {stats['frames']} "
                 + "frames mapped to their radarState")
  r1['verdict'] = 'INCOMPLETE' if bad else r1['verdict'] if n_rows else 'INFO'   # INFO: no row has the radar stage (synthetic rows)
  r3 = dict(gate('R3 FrogPilot lead-consumer replay vs logged frogpilotPlan (HEAD)', [], coverage=fp_cov), verdict='INFO',
            note=f'{fp_cov} spans on drives with HEAD\'s FrogPilot planner code, {fp_ticks} scored ticks; ticks that differ per field '
                 + f'(discrete exact, floats > 1e-4): {dict(fp_tot)}; the FrogPilot process input timing is not logged (its sample '
                 + 'choice follows the planner rule): not gating')
  return bad, r1, r3


def fp_compare(rp_head, rp_on):
  """R4 (INFO): the FrogPilot lead consumers, candidate vs HEAD on the same frogpilotPlan ticks: ticks that differ per field, those
  of them where HEAD's replay already differs from the logged frogpilotPlan (the candidate's planner reads log + (candidate - HEAD),
  the candidate's value for a discrete field: there its change rests on a replay mismatch, R3), the lead-departing alert onsets
  (rising edges) per arm; plus vision-only leads (no radar) whose published values differ."""
  diff, on_bad, onsets, vision, n = Counter(), Counter(), [0, 0], 0, 0
  for c in sorted(set(rp_head) & set(rp_on)):
    for s in sorted(set(rp_head[c]) & set(rp_on[c])):
      a, b = (rp_head[c][s].get('radar') or {}), (rp_on[c][s].get('radar') or {})
      fa, fb = a.get('fp'), b.get('fp')
      if fa is not None and fb is not None and np.array_equal(fa['ns'], fb['ns']):
        n += len(fa['ns'])
        for k in fa:
          if k.startswith('fp_'):
            d = fa[k] != fb[k]
            diff[k[3:]] += int(np.sum(d))
            if 'log_' + k in fa:
              on_bad[k[3:]] += int(np.sum(d & (fa[k] != fa['log_' + k])))
        for i, f in enumerate((fa, fb)):
          onsets[i] += len(runs(f['fp_leadDeparting'] > 0))
      ma, mb = a.get('m1'), b.get('m1')
      if ma is not None and mb is not None and len(ma['t']) == len(mb['t']):
        for w in ('l1', 'l2'):
          vis = (ma[f'{w}_status'] > 0) & (ma[f'{w}_radar'] == 0) & (mb[f'{w}_status'] > 0) & (mb[f'{w}_radar'] == 0)
          vision += int(np.sum(vis & (ma[f'{w}_vLead'] != mb[f'{w}_vLead'])))
  return dict(gate('R4 FrogPilot lead consumers + vision leads, candidate vs HEAD', [], coverage=n), verdict='INFO',
              note=f'{n} frogpilotPlan ticks; ticks that differ per field: { {k: v for k, v in diff.items() if v} } (of them on ticks where '
                   + f'HEAD\'s replay differs from the log: { {k: v for k, v in on_bad.items() if v} }); lead-departing '
                   + f'alert onsets HEAD {onsets[0]} -> candidate {onsets[1]}; vision-only lead ticks whose published vLead differs: {vision}')


def _q(x, p):
  return float(np.percentile(x, p)) if len(x) else float('nan')


def m1_samples(rows):
  """Pooled M1 arrays of replay rows ({corpus: {span: row}}): per lead the radar-lead ticks with their class flags, published fields
  and truth corners; per (span, lead) the tick runs for the excursion count."""
  out = {}
  for w in ('l1', 'l2'):
    parts = []
    for c, rs in sorted(rows.items()):
      for s, x in sorted(rs.items()):
        m = (x.get('radar') or {}).get('m1')
        if not m or not len(m['t']):
          continue
        ok = (m[f'{w}_status'] > 0) & (m[f'{w}_radar'] > 0) & np.isfinite(m[f'{w}_v11'])
        parts.append(dict(span=np.full(int(ok.sum()), f'{c}|{s}'), t=m['t'][ok], a_ego=m['a_ego'][ok],
                          **{k[len(w) + 1:]: v[ok] for k, v in m.items() if k.startswith(f'{w}_')}))
    out[w] = {k: np.concatenate([p[k] for p in parts]) for k in parts[0]} if parts else {}
  return out


def m1_classes(S, w):
  """{class: mask} of one lead's pooled samples (S = m1_samples(...)[w])."""
  if not S:
    return {}
  a, st, ae = S['a'], S['static'] > 0, S['a_ego']
  mv = ~st & np.isfinite(a)
  if w == 'l2':
    return {'stationary': st, 'moving braking (a <= -0.3)': mv & (a <= -0.3), 'moving other': mv & (a > -0.3)}
  return {'stationary': st,
          'braking lead (a <= -1.0), ego a <= -1.5': mv & (a <= -1.0) & (ae <= -1.5),
          'braking lead (a <= -1.0), ego -1.5..-0.5': mv & (a <= -1.0) & (ae > -1.5) & (ae <= -0.5),
          'braking lead (a <= -1.0), ego > -0.5': mv & (a <= -1.0) & (ae > -0.5),
          'mild braking (-1.0..-0.3)': mv & (a > -1.0) & (a <= -0.3), 'steady (|a| < 0.3)': mv & (np.abs(a) < 0.3), 'accelerating (a >= 0.3)': mv & (a >= 0.3),
          'braking onset (first 1 s)': ~st & (S['onset'] > 0), 'accel sign change (+-0.5 s)': ~st & (S['sign'] > 0),
          'fresh track (first 0.5 s)': ~st & (S['age'] < 0.5)}


def _excursions(S, field, corner):
  """(count, seconds) of sustained optimistic runs: pub - truth > M1_EXC_V for >= M1_EXC_S on consecutive ticks of one track."""
  if not S:
    return 0, 0.0
  e = S[field] - S[f'v{corner}'] > M1_EXC_V
  n, secs = 0, 0.0
  brk = np.r_[True, (S['span'][1:] != S['span'][:-1]) | (S['radarTrackId'][1:] != S['radarTrackId'][:-1]) | (np.diff(S['t']) > 1.5 * M1_DT)]
  for seg in np.split(np.arange(len(e)), np.flatnonzero(brk)[1:]):
    for a, b in runs(e[seg]):
      dur = (b - a) * M1_DT
      if dur >= M1_EXC_S:
        n, secs = n + 1, secs + dur
  return n, round(secs, 2)


def _stat_episodes(S, field):
  """[(span, track, t0, t1, peak)] of optimistic stationary-lead episodes: published field > M1_STAT_V for >= M1_STAT_S on consecutive
  stationary ticks of one track (truth 0)."""
  if not S:
    return []
  k = np.flatnonzero(S['static'] > 0)
  if not len(k):
    return []
  sp, tid, t, v = S['span'][k], S['radarTrackId'][k], S['t'][k], S[field][k]
  brk = np.r_[True, (sp[1:] != sp[:-1]) | (tid[1:] != tid[:-1]) | (np.diff(t) > 1.5 * M1_DT)]
  out = []
  for seg in np.split(np.arange(len(k)), np.flatnonzero(brk)[1:]):
    for a, b in runs(v[seg] > M1_STAT_V):
      if (b - a) * M1_DT >= M1_STAT_S - 1e-9:
        i = seg[a:b]
        out.append((str(sp[i[0]]), int(tid[i[0]]), float(t[i[0]]), float(t[i[-1]]), round(float(v[i].max()), 3)))
  return out


def m1(rp_head, rp_on):
  """M1: published vLead / vLeadK / aLeadK error vs truth, candidate vs HEAD, per lead class (tails, onsets, sign changes, fresh
  tracks, leadOne / leadTwo) at every corner of the truth envelope (radar_replay.LAGS x SCALES; stationary truth = 0). A class
  FAILS where the candidate is more optimistic than HEAD beyond the tolerance at every corner, is UNCERTAIN where only some corners
  fail, EMPTY below M1_N_MIN ticks in either arm. Returns (verdict PASS / FAIL / NO COVERAGE, rows)."""
  H, V = m1_samples(rp_head), m1_samples(rp_on)
  rows = []
  if not any(H.values()) and not any(V.values()) and not any('radar' in x for rs in rp_head.values() for x in rs.values()):
    return 'INFO', rows   # no row has the radar stage (synthetic rows): M1 not run
  corners = [f'{i}{j}' for i in range(3) for j in range(3)]
  for w in ('l1', 'l2'):
    ch, cv = m1_classes(H[w], w), m1_classes(V[w], w)
    for name in ch or cv:
      mh, mv = ch.get(name), cv.get(name)
      nh, nv = int(mh.sum()) if mh is not None else 0, int(mv.sum()) if mv is not None else 0
      row = dict(lead=w, cls=name, n=(nh, nv))
      if min(nh, nv) < M1_N_MIN:
        rows.append(dict(row, verdict='EMPTY'))
        continue
      checks = []
      if name == 'stationary':
        for f in ('vLead', 'vLeadK'):
          eh, ev = H[w][f][mh], V[w][f][mv]
          sh = dict(mae=float(np.mean(np.abs(eh))), rms=float(np.sqrt(np.mean(eh ** 2))), p99abs=_q(np.abs(eh), 99))
          sv = dict(mae=float(np.mean(np.abs(ev))), rms=float(np.sqrt(np.mean(ev ** 2))), p99abs=_q(np.abs(ev), 99))
          for k, tol in M1_TOL_STATIC.items():
            checks.append((f'{f} {k}', [sv[k] - sh[k] > tol] * len(corners), round(sh[k], 3), round(sv[k], 3)))
      else:
        for f, tol in (('vLead', M1_TOL_V), ('vLeadK', M1_TOL_V), ('aLeadK', M1_TOL_A)):
          for k, t_ in tol.items():
            fails, vals = [], []
            for c in corners:
              th, tv = (H[w]['a'], V[w]['a']) if f == 'aLeadK' else (H[w][f'v{c}'], V[w][f'v{c}'])
              eh, ev = (H[w][f] - th)[mh], (V[w][f] - tv)[mv]
              ok_h, ok_v = np.isfinite(eh), np.isfinite(ev)
              sh = float(np.mean(eh[ok_h])) if k == 'mean' else _q(eh[ok_h], int(k[1:]))
              sv = float(np.mean(ev[ok_v])) if k == 'mean' else _q(ev[ok_v], int(k[1:]))
              fails.append(sv - sh > t_)
              vals.append((sh, sv))
              if f == 'aLeadK':   # one acceleration truth (no lag / scale corners)
                fails, vals = fails * len(corners), vals * len(corners)
                break
            checks.append((f'{f} {k}', fails, round(vals[4][0], 3), round(vals[4][1], 3)))
      nf = [sum(x[1]) for x in checks]
      worst = [dict(check=x[0], head=x[2], cand=x[3], corners_failing=sum(x[1])) for x in checks if any(x[1])]
      verdict = 'PASS' if not any(nf) else 'FAIL' if any(n == len(corners) for n in nf) else 'UNCERTAIN'
      rows.append(dict(row, verdict=verdict, failing=worst[:6]))
  # sustained optimistic excursions on every moving radar-lead tick (leadOne and leadTwo), per corner
  for w in ('l1', 'l2'):
    sh_ = {k: v[H[w]['static'] == 0] for k, v in H[w].items()} if H[w] else {}
    sv_ = {k: v[V[w]['static'] == 0] for k, v in V[w].items()} if V[w] else {}
    for f in ('vLead', 'vLeadK'):
      fails, vals = [], []
      for c in corners:
        (nh, th), (nv, tv) = _excursions(sh_, f, c), _excursions(sv_, f, c)
        fails.append(nv > nh + max(M1_EXC_TOL[0], M1_EXC_TOL[2] * nh) or tv > th + max(M1_EXC_TOL[1], M1_EXC_TOL[2] * th))
        vals.append(((nh, th), (nv, tv)))
      verdict = 'PASS' if not any(fails) else 'FAIL' if all(fails) else 'UNCERTAIN'
      rows.append(dict(lead=w, cls=f'sustained optimistic excursions {f} (> {M1_EXC_V} m/s for >= {M1_EXC_S} s)', n=None, verdict=verdict,
                       failing=[dict(head=vals[4][0], cand=vals[4][1], corners_failing=sum(fails))] if any(fails) else []))
  # stationary leads: optimistic episodes vs zero truth, candidate vs HEAD (episode level; the pooled checks above stay)
  for w in ('l1', 'l2'):
    for f in ('vLead', 'vLeadK'):
      eh, ev = _stat_episodes(H[w], f), _stat_episodes(V[w], f)
      new = [e for e in ev if not any(h[:2] == e[:2] and h[2] <= e[3] and e[2] <= h[3] for h in eh)]
      sec = [round(sum(e[3] - e[2] + M1_DT for e in x), 2) for x in (eh, ev)]
      worst = (max((e[3] - e[2] for e in eh), default=0.0) + 1e-6, max((e[4] for e in eh), default=0.0))   # HEAD's longest / highest
      worse = [e for e in new if e[3] - e[2] > worst[0] or e[4] > worst[1]]
      bad = bool(worse) or len(ev) > len(eh) or sec[1] > sec[0] + 1e-6
      rows.append(dict(lead=w, cls=f'stationary optimistic episodes {f} (> {M1_STAT_V} m/s for >= {M1_STAT_S} s, truth 0)', n=(len(eh), len(ev)),
                       verdict='FAIL' if bad else 'PASS', seconds=sec, new=len(new), head_worst=(round(worst[0] + M1_DT, 2), worst[1]),
                       failing=[dict(span=e[0], track=e[1], t0=round(e[2], 2), t1=round(e[3], 2), peak=e[4]) for e in (worse or new or ev)[:6]] if bad else []))
  # INFO (not gating): the stationary leadOne per ego acceleration bin (the radar latency error is L x aEgo there: 10-02 replay
  # -0.27 m/s mean at ego <= -1.5), signed mean / MAE / p99 |error| of vLead and vLeadK, HEAD -> candidate
  for lo, hi in M1_EGO_BINS:
    sel = [(S['l1']['static'] > 0) & (S['l1']['a_ego'] > lo) & (S['l1']['a_ego'] <= hi) if S['l1'] else np.zeros(0, bool) for S in (H, V)]
    st = {f: [(round(float(np.mean(S['l1'][f][m])), 3), round(float(np.mean(np.abs(S['l1'][f][m]))), 3), round(_q(np.abs(S['l1'][f][m]), 99), 3))
              if m.any() else None for S, m in zip((H, V), sel, strict=True)] for f in ('vLead', 'vLeadK')}
    rows.append(dict(lead='l1', cls=f'stationary, ego a {lo:g}..{hi:g} (INFO: mean, MAE, p99 |err|, HEAD -> candidate)', n=tuple(int(m.sum()) for m in sel),
                     verdict='INFO', stats=st))
  v = 'FAIL' if any(r['verdict'] == 'FAIL' for r in rows) else 'NO COVERAGE' if any(r['verdict'] in ('EMPTY', 'UNCERTAIN') for r in rows) else 'PASS'
  return v, rows
