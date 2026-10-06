#!/usr/bin/env python3
"""Per-drive stopping report: run it after each route sync. Idempotent; no device access, no live readers.

  .venv/bin/python tools/stopping/drive_report.py                     # every new local route with rlogs
  .venv/bin/python tools/stopping/drive_report.py --routes 00002232,00002235 [--rebuild]

Stage A scans each rlog segment once into compact series (cached under WORK/scan). Stage B finds the engaged stops
(the cycle-1003 census rule behind the NEW/history case groups) and holds (engaged standstill >= 1 s), computes the stop
metrics on the LOGGED columns, replays every stop/hold span exactly (drive_replay.py: LongControl + service + sender on
the logged inputs, controller code pinned to the drive's commit) for the fidelity check and for every trial flag in
TRIALS (flag on vs off), and lists the trial events and the trial's revert rules. Output: REPORTS/<route>.md and .json,
one summary line per drive in REPORTS/INDEX.md. Repo docs are not written; the host copies the summary into the worklog.
Hook after a sync (printed by --hook):
  .venv/bin/python tools/route_sync/refresh_routes.py --include-rlog && .venv/bin/python tools/stopping/drive_report.py
"""
import argparse
import datetime as dt
import glob
import inspect
import json
import os
import sys
import time
from collections import defaultdict
from multiprocessing import get_context
from pathlib import Path

import numpy as np

if __package__ in (None, ''):
  sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from openpilot.tools.stopping import drive_replay as DR  # (no openpilot controls import at module level)
from openpilot.tools.stopping.sim import gates as SG

WORK = DR.WORK
REPORTS = Path(os.environ.get('DRIVE_REPORT_OUT', Path.home() / '.route_sync/reports/drives'))
PROCESSES = 2                 # Mac memory budget: at most 2 workers per agent
LCS = {'off': 0, 'pid': 1, 'stopping': 2, 'starting': 3}
PHASES = ('INACTIVE', 'APPROACH_GLIDE', 'PRE_STOP_EASE', 'RAMP_TO_HOLD', 'HOLD', 'RELEASE')   # stopping_service.Phase order
HOLD_PHASES = ('RAMP_TO_HOLD', 'HOLD')
G = 9.81
FIDELITY_SKIP_S = 1.0         # replay warm-up excluded from the fidelity check (frame 0 has no previous output)
# trial flag -> reference commit (first car build with the flag) for the counterfactual on drives without it
TRIALS = {'RELEASE_END_STOPPED_LEAD_REHOLD': dict(name='E3', ref='2e39594627', doc='cycle_20261003/PLAN.md sections 56-61'),
          'SANTA_FE_STOP_LINE': dict(name='LINE', ref='56e1512892', doc='cycle_20261003/PLAN.md sections 72-79, eb4 per-drive rule ' +
                                     'with the eb5 corrections and LFL_CODE_REVIEW_astra.md')}
ATTN = dict(rest_lo=3.5, rest_hi=6.0, a_stop=-0.6, bite=-0.10)


# ==== stage A: one pass per rlog segment ===================================================================================
def scan_segment(seg):
  """Compact series of one segment, times route-relative (s from the route's initData). Cached; returns (seg, status)."""
  out = WORK / 'scan' / f'{seg}.npz'
  if out.exists():
    return seg, 'cached'
  from openpilot.tools.stopping.review.triage_one import read_events
  from openpilot.tools.stopping.review import kcs1_reps
  path = DR.RD / seg / 'rlog.zst'
  rows = defaultdict(list)
  logs, bms = [], set()
  origin, commit, dirty, err = None, None, None, None
  try:
    for ev in read_events(str(path)):
      w = ev.which()
      if origin is None:
        origin = ev.logMonoTime   # every rlog starts with the route's initData
      t = (ev.logMonoTime - origin) * 1e-9
      if w == 'carState':
        c = ev.carState
        rows['cs'].append((t, c.vEgo, c.aEgo, c.standstill, c.gasPressed, c.brakePressed, c.vEgoRaw))
      elif w == 'carControl':
        c = ev.carControl
        o = list(c.orientationNED)
        rows['cc'].append((t, c.enabled, c.longActive, c.actuators.accel, o[1] if len(o) == 3 else np.nan))
      elif w == 'carOutput':
        rows['co'].append((t, ev.carOutput.actuatorsOutput.accel))
      elif w == 'controlsState':
        rows['ctl'].append((t, LCS.get(str(ev.controlsState.longControlState), -1)))
      elif w == 'longitudinalPlan':
        p = ev.longitudinalPlan
        rows['lp'].append((t, p.aTarget, p.shouldStop, p.distanceToStopTarget, p.distanceToStopTargetModel, p.hasLead))
      elif w == 'radarState':
        a, b = ev.radarState.leadOne, ev.radarState.leadTwo
        rows['rs'].append((t, a.status, a.dRel, a.vLead, a.aLeadK, a.radarTrackId, a.radar, b.status, b.dRel, b.vLead))
      elif w == 'selfdriveState':
        rows['sds'].append((t, ev.selfdriveState.enabled, ev.selfdriveState.experimentalMode))
      elif w == 'frogpilotPlan':
        rows['fpp'].append((t, ev.frogpilotPlan.experimentalMode, ev.frogpilotPlan.increasedStoppedDistance))
      elif w == 'logMessage':
        m = ev.logMessage
        if 'stopping_service' in m and 'phase_change' in m:
          try:
            d = json.loads(m)['msg']
            logs.append((round(t, 3), d.get('phase'), d.get('prev_phase')))
          except (ValueError, KeyError, TypeError, AttributeError):
            pass
      elif w in ('userBookmark', 'bookmarkButton'):
        bms.add(round(t, 2))
      elif w == 'initData' and commit is None:
        commit, dirty = ev.initData.gitCommit, bool(ev.initData.dirty)
  except Exception as exc:  # a truncated/corrupt rlog keeps what was read; reported per segment
    err = f'{type(exc).__name__}: {exc}'
  qlog = DR.RD / seg / 'qlog.zst'
  if qlog.is_file():   # bookmarks also from the qlog (as the census: qlog_bm.json)
    try:
      first = None
      for ev in read_events(str(qlog)):
        first = first or ev.logMonoTime
        if ev.which() in ('userBookmark', 'bookmarkButton'):
          bms.add(round((ev.logMonoTime - first) * 1e-9, 2))
    except Exception:  # bookmarks are best effort from a truncated qlog
      pass
  arrays = {}
  if origin is not None:
    try:
      streams, _ = kcs1_reps.read_route([str(path)])
      o = origin * 1e-9
      for name, cols in (('imu', ('x', 'y', 'z')), ('gyro', ('x', 'y', 'z')), ('calib', ('roll', 'pitch', 'yaw')),
                         ('scc12', ('aReqValue', 'StopReq', 'ACCMode')), ('whl', ('mean',)), ('esp12', ('LONG_ACCEL',)),
                         ('pul', ('WHL_PUL_FL', 'WHL_PUL_FR', 'WHL_PUL_RL', 'WHL_PUL_RR'))):
        s = streams[name]
        arrays[name] = np.column_stack([s['t'] - o] + [np.asarray(s[c], dtype=float) for c in cols]) if len(s['t']) else \
          np.zeros((0, len(cols) + 1))
    except Exception as exc:  # CAN/IMU streams missing: metrics that need them become None
      err = (err or '') + f' kcs1_reps: {type(exc).__name__}: {exc}'
  width = dict(cs=7, cc=5, co=2, ctl=2, lp=6, rs=10, sds=3, fpp=3)
  for k, n in width.items():
    arrays[k] = np.array(rows[k], dtype=np.float64).reshape(-1, n)
  out.parent.mkdir(parents=True, exist_ok=True)
  tmp = out.with_name(out.name + f'.tmp{os.getpid()}.npz')
  np.savez_compressed(tmp, **arrays)
  meta = dict(seg=seg, commit=commit, dirty=dirty, logs=logs, bookmarks=sorted(bms), error=err)
  out.with_suffix('.json').write_text(json.dumps(meta))
  os.replace(tmp, out)
  return seg, f"cs={len(rows['cs'])} err={err}"


def route_segments(route):
  segs = [os.path.basename(os.path.dirname(p)) for p in glob.glob(str(DR.RD / f'{route}--*' / 'rlog.zst'))]
  return sorted(segs, key=lambda s: int(s.rsplit('--', 1)[1]))


def load_route(route):
  """Concatenated stage-A series of a route (sorted by time) + meta (commits, phase-change logs, bookmarks)."""
  parts, metas = defaultdict(list), []
  for seg in route_segments(route):
    p = WORK / 'scan' / f'{seg}.npz'
    if not p.exists():
      continue
    d = np.load(p)
    for k in d.files:
      parts[k].append(d[k])
    m = json.loads(p.with_suffix('.json').read_text())
    m['t0'] = float(d['cs'][0, 0]) if len(d['cs']) else None
    metas.append(m)
  R = {}
  for k, v in parts.items():
    a = np.concatenate(v)
    R[k] = a[np.argsort(a[:, 0], kind='stable')] if len(a) else a
  R['logs'] = sorted(((t, ph, prev) for m in metas for t, ph, prev in m['logs']), key=lambda x: x[0])
  bms = sorted({b for m in metas for b in m['bookmarks']})
  R['bookmarks'] = [b for i, b in enumerate(bms) if i == 0 or b - bms[i - 1] > 0.1]   # rlog + qlog copies of one press
  R['commits'] = sorted({m['commit'] for m in metas if m['commit']})
  R['dirty'] = any(m.get('dirty') for m in metas)
  R['errors'] = [(m['seg'], m['error']) for m in metas if m['error']]
  R['segments'] = [m['seg'] for m in metas]
  return R


# ==== helpers ===============================================================================================================
def at(arr, t, col):
  """Value of col in the latest row at or before t (nan if none); t may be an array."""
  if not len(arr):
    return np.full(np.shape(t), np.nan) if np.ndim(t) else np.nan
  i = np.searchsorted(arr[:, 0], t, side='right') - 1
  v = arr[np.clip(i, 0, len(arr) - 1), col]
  return np.where(i >= 0, v, np.nan) if np.ndim(t) else (float(v) if i >= 0 else np.nan)


def rng(arr, a, b):
  return np.searchsorted(arr[:, 0], a, side='left'), np.searchsorted(arr[:, 0], b, side='right')


def runs(mask):
  """[(i0, i1)) runs of True."""
  d = np.diff(np.r_[0, np.asarray(mask, dtype=np.int8), 0])
  return list(zip(np.flatnonzero(d == 1), np.flatnonzero(d == -1), strict=True))


def trailing_mean(t, x, win, min_n=1):
  c = np.concatenate(([0.0], np.cumsum(x)))
  j = np.searchsorted(t, t - win, side='right')
  n = np.arange(1, len(t) + 1) - j
  out = (c[1:] - c[j]) / np.maximum(n, 1)
  out[n < min_n] = np.nan
  return out


def phase_at(logs, t):
  """Logged service phase (phase_change logs) at route time t."""
  ph = 'INACTIVE'
  for tl, p, _ in logs:
    if tl > t:
      break
    ph = p
  return ph


def zigzag(t, s, h):
  """rharness.zigzag: turning points of s with hysteresis h: [(t, value, 'peak'|'valley')]."""
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
  """eb2 an.zig_pumps: strict pump count (dip -> bite -> release: a peak preceded by a turning point) and max amplitude.
  s is the deceleration (-accel), so a peak is a bite."""
  if len(t) < 3:
    return 0, 0.0
  z = zigzag(t, s, h)
  n, amp = 0, 0.0
  for i, p in enumerate(z):
    if p[2] != 'peak' or i == 0:
      continue
    kp = int(np.searchsorted(t, p[0]))
    rise = p[1] - z[i - 1][1]
    drop = p[1] - (z[i + 1][1] if i + 1 < len(z) else float(np.min(s[kp:])))
    n += 1
    amp = max(amp, min(rise, drop))
  return n, amp


# ==== stops and holds ======================================================================================================
def engaged_stops(R, interrupted=False):
  """cycle-1003 census rule (history/census.py): a wheel stop (vEgo < 0.05 after >= 1.0) or a rolling stop (dip below
  0.5 and relaunch to >= 1.0 without a wheel stop), longActive through the last 4 s (>= 100 frames), planner present.
  interrupted: instead the stops this rule leaves out because the last 4 s were not all longActive although the approach (the 30 s
  before the stop) was engaged: a driver-rescued approach. Not in the stop table; the trials replay them."""
  cs, cc, lp = R['cs'], R['cc'], R['lp']
  if len(cs) < 2 or len(cc) < 100 or len(lp) < 5:
    return []
  t, v = cs[:, 0], cs[:, 1]
  cands, armed, low = [], False, None
  for i in range(1, len(v)):
    if v[i] >= 1.0:
      if low is not None:
        cands.append((low, 'rolling'))
      low, armed = None, True
    if armed and v[i] < 0.5 and (low is None or v[i] < v[low]):
      low = i
    if armed and v[i] < 0.05 and v[i - 1] >= 0.05:
      cands.append((i, 'stop'))
      armed, low = False, None
  out = []
  for i, kind in sorted(cands):
    ts = float(t[i])
    j0, j1 = rng(cc, ts - 4.0, ts)
    if (j1 - j0 < 100 or not np.all(cc[j0:j1, 2] > 0)) != interrupted:
      continue
    a0, a1 = rng(cc, ts - LR['approach_s'], ts)
    if interrupted and not np.any(cc[a0:a1, 2] > 0):
      continue
    k0, k1 = rng(cs, ts - 15.0, ts)
    vseg = cs[k0:k1 + 1, 1]
    hi = np.flatnonzero(vseg >= 0.10)
    tws = float(cs[k0 + hi[-1] + 1, 0]) if len(hi) and k0 + hi[-1] + 1 < len(cs) else ts
    if kind == 'rolling':
      tws = ts
    p0, p1 = rng(lp, tws - 15.0, tws + 0.01)
    if p1 - p0 < 5:
      continue
    g0, g1 = rng(cs, ts - 4.0, ts)
    out.append(dict(kind=kind, t_stop=round(ts, 2), t_ws=round(tws, 2), gas4=bool(np.any(cs[g0:g1, 4] > 0)),
                    brake4=bool(np.any(cs[g0:g1, 5] > 0))))
  return out


def holds(R):
  """cycle-1004 hold census rule (census/holds.py): maximal run with vEgo < 0.10 and longActive, >= 1.0 s."""
  cs, cc = R['cs'], R['cc']
  if len(cs) < 100 or len(cc) < 100:
    return []
  t = cs[:, 0]
  la = at(cc, t, 2) > 0.5
  stopped = cs[:, 1] < 0.10
  out = []
  for i0, i1 in runs(stopped & la):
    if t[i1 - 1] - t[i0] < 1.0:
      continue
    k = max(i0 - 1, 0)
    began = 'engaged_stop' if (la[k] and not stopped[k]) else ('engage_at_standstill' if not la[k] else 'other')
    out.append(dict(t_start=round(float(t[i0]), 2), t_end=round(float(t[i1 - 1]), 2), began=began,
                    end='log_end' if i1 >= len(t) else ('ego_moved' if la[min(i1, len(t) - 1)] else 'disengaged')))
  return out


def takeovers(R):
  """Pedal takeovers of engaged driving that need not end in a stop: a brake press while longActive, or a gas press while longActive
  with a braking command (the trial windows of a driver who overrides the car's braking). -> [route time]"""
  cs, cc = R['cs'], R['cc']
  if len(cs) < 2 or not len(cc):
    return []
  t = cs[:, 0]
  engaged = at(cc, t - 0.02, 2) > 0.5
  press = lambda c: np.r_[False, (cs[1:, c] > 0) & (cs[:-1, c] <= 0)]  # noqa: E731
  k = np.flatnonzero(engaged & (press(5) | (press(4) & (at(cc, t - 0.02, 3) < 0.0))))
  return [round(float(t[i]), 2) for i in k]


def launch_after(cs, t_ws):
  k = np.flatnonzero((cs[:, 0] > t_ws + 0.3) & (cs[:, 1] > 0.3))
  return float(cs[k[0], 0]) if len(k) else float(cs[-1, 0])


def last_above(cs, t, v, span):
  k = np.flatnonzero((cs[:, 0] < t) & (cs[:, 0] >= t - span) & (cs[:, 1] >= v))
  return float(cs[k[-1], 0]) if len(k) else None


def replay_spans(route, R, stops, hlds, tko=()):
  """Stop windows (last >= 8 m/s - 5 s, else wheel stop - 90 s, to launch + 8 s), hold windows (approach from the last
  >= 2.5 m/s - 15 s, to hold end + 8 s) and takeover windows (tko: the approach 30 s before to 8 s after), merged when they overlap or
  touch within 2 s (launch/spans.py)."""
  cs = R['cs']
  wins = [(t - LR['approach_s'], t + 8.0) for t in tko]
  for s in stops:
    t8 = last_above(cs, s['t_ws'], 8.0, 120.0)
    tl = s['t_ws'] if s['kind'] == 'rolling' else launch_after(cs, s['t_ws'])
    wins.append((t8 - 5.0 if t8 is not None else s['t_ws'] - 90.0, min(tl, s['t_ws'] + 150.0) + 8.0))
  for h in hlds:
    lo = h['t_start'] - 10.0
    if h['began'] == 'engaged_stop':
      t25 = last_above(cs, h['t_start'], 2.5, 120.0)
      lo = (t25 if t25 is not None else h['t_start'] - 20.0) - 15.0
    wins.append((lo, h['t_end'] + 8.0))
  t_end = float(cs[-1, 0])
  spans, cur = [], None
  for lo, hi in sorted(wins):
    lo, hi = max(lo, 0.5), min(hi, t_end)
    if cur is not None and lo <= cur['hi'] + 2.0:
      cur['hi'] = max(cur['hi'], hi)
    else:
      cur = dict(route=route, lo=lo, hi=hi)
      spans.append(cur)
  for s in spans:
    s.update(lo=round(s['lo'], 2), hi=round(s['hi'], 2))
    s['span'] = f"{route[4:8]}_{route[10:14]}_{s['lo']:.2f}_{s['hi']:.2f}"   # caches are keyed by the exact window
  return spans


# ==== per-stop metrics on the logged columns ================================================================================
def body_accel(R, t_ref):
  """kcs1 terminal.body: gravity-compensated forward specific force (raw accelerometer in the calibrated frame)."""
  from openpilot.common.transformations.orientation import rot_from_euler
  imu, cal, cc = R['imu'], R['calib'], R['cc']
  i = max(int(np.searchsorted(cal[:, 0], t_ref, side='right')) - 1, 0)
  rot = rot_from_euler([cal[i, 1], cal[i, 2], cal[i, 3]])
  f = np.stack([-imu[:, 3], -imu[:, 2], -imu[:, 1]], 1) @ rot
  ok = np.isfinite(cc[:, 4])
  pitch = np.interp(imu[:, 0], cc[ok, 0], cc[ok, 4]) if ok.any() else np.zeros(len(imu))
  return imu[:, 0], f[:, 0] - G * np.sin(pitch)


def terminal(R, t_flag, t_end):
  """a_stop / j300 as kcs1 terminal.metrics (arrive_rel = release level before the rebound peak - rest level, IMU 0.1 s
  mean; j300 = max 300 ms exact-difference jerk of that mean from the last 0.5 m/s to the stop + 0.8 s)."""
  wt, wv = R['whl'][:, 0], R['whl'][:, 1]
  i_flag = int(np.searchsorted(wt, t_flag, side='right'))
  above = np.flatnonzero(wv[:i_flag] > 0.5)
  if not len(above) or above[-1] + 1 >= len(wv):
    return {}
  i = above[-1]
  t05 = float(np.interp(0.5, [wv[i + 1], wv[i]], [wt[i + 1], wt[i]]))
  m = (R['imu'][:, 0] >= t05 - 6.0) & (R['imu'][:, 0] <= t_flag + 4.0)
  sub = dict(R, imu=R['imu'][m])
  bt, bl = body_accel(sub, t05 - 3.0)
  if len(bt) < 50:
    return {}
  m01 = trailing_mean(bt, bl, 0.1, min_n=5)
  hi = t_flag + 1.5 if t_end is None else min(t_flag + 1.5, t_end - 0.05)
  k = np.flatnonzero((bt >= t05) & (bt <= t_flag + 0.8))
  if len(k) < 5:
    return {}
  kp = k[np.nanargmax(m01[k])]
  kb = np.flatnonzero((bt >= bt[kp] - 0.4) & (bt <= bt[kp]))
  a_rel = float(m01[kb[np.nanargmin(m01[kb])]])
  q = np.arange(t05, t_flag + 0.8 - 0.3, 0.01)
  ok = np.isfinite(m01)
  j300 = float(np.max((np.interp(q + 0.3, bt[ok], m01[ok]) - np.interp(q, bt[ok], m01[ok])) / 0.3)) if len(q) else None
  rest = (bt >= t_flag + 0.7) & (bt <= hi)
  a_rest = float(np.nanmedian(m01[rest])) if rest.sum() >= 20 else None
  k2 = np.flatnonzero((bt >= t05) & (bt <= t_flag + 0.5))
  return dict(a_stop=None if a_rest is None else a_rel - a_rest, j300=j300, a_rest=a_rest,
              imu_min=float(np.nanmin(m01[k2])) if len(k2) else None, t05=t05)


def stopreq_pairs(R, a, b, cs_drv):
  """Autonomous StopReq toggles at rest in [a, b] whose next toggle follows within 1 s: [(t_toggle, t_next, kind)]."""
  sc = R.get('scc12')
  if sc is None or not len(sc):
    return []
  k0, k1 = rng(sc, a, b)
  x = sc[k0:k1]
  if len(x) < 2:
    return []
  tog = np.flatnonzero(np.diff(x[:, 2]) != 0) + 1
  out = []
  for i, j in zip(tog, tog[1:], strict=False):
    t1, t2 = float(x[i, 0]), float(x[j, 0])
    if t2 - t1 > 1.0:
      continue
    if at(R['cs'], t1, 1) >= 0.1 or at(R['cs'], t2, 1) >= 0.1 or cs_drv(t1 - 0.5, t2 + 0.5):
      continue
    out.append((round(t1, 2), round(t2, 2), 'set->clear' if x[i, 2] < 0.5 else 'clear->set'))
  return out


def stop_metrics(R, s):
  cs, cc, rs, ctl, logs = R['cs'], R['cc'], R['rs'], R['ctl'], R['logs']
  tws = s['t_ws']
  t_launch = tws if s['kind'] == 'rolling' else launch_after(cs, tws)

  def drv(a, b):   # any driver input (gas, brake) or not longActive in [a, b]
    k0, k1 = rng(cs, a, b)
    j0, j1 = rng(cc, a, b)
    return bool(np.any(cs[k0:k1, 4] > 0) or np.any(cs[k0:k1, 5] > 0) or (j1 > j0 and np.any(cc[j0:j1, 2] < 0.5)))
  # final engaged approach: after the last driver input within 12 s before the stop
  k0, k1 = rng(cs, tws - 12.0, tws)
  j0, j1 = rng(cc, tws - 12.0, tws)
  pedal = [float(cs[k0 + i, 0]) for i in np.flatnonzero((cs[k0:k1, 4] > 0) | (cs[k0:k1, 5] > 0))]
  off = np.flatnonzero(cc[j0:j1, 2] < 0.5)
  marks = pedal + [float(cc[j0 + i, 0]) for i in off]
  t_lo = max(marks) + 0.01 if marks else tws - 12.0
  # a takeover = a pedal or a disengagement (longActive 1 -> 0) inside the 12 s; an engagement there only shortens the approach
  drop = [i for i in off if i > 0 and cc[j0 + i - 1, 2] > 0.5]
  m = dict(kind=s['kind'], t_ws=tws, t_launch=round(t_launch, 2), t_lo=round(t_lo, 2), gas4=s['gas4'], brake4=s['brake4'])
  # radar gap
  g0, g1 = rng(rs, tws - 0.5, tws + 1.0)
  gl = rs[g0:g1][rs[g0:g1, 1] > 0]
  m['rest'] = round(float(gl[:, 2].min()), 2) if len(gl) else None          # census rest: min dRel [-0.5, +1.0] s
  g0, g1 = rng(rs, tws + 0.3, tws + 1.3)
  gl = rs[g0:g1][rs[g0:g1, 1] > 0]
  m['rest_med'] = round(float(np.median(gl[:, 2])), 2) if len(gl) else None   # rharness rest: median dRel [+0.3, +1.3] s
  g0, g1 = rng(rs, t_lo, tws + 1.3)
  gl = rs[g0:g1][rs[g0:g1, 1] > 0]
  m['min_gap'] = round(float(gl[:, 2].min()), 2) if len(gl) else None
  m['lead_at_stop'] = bool(at(rs, tws, 1) > 0)
  m['vlead_at_stop'] = round(float(at(rs, tws, 3)), 2) if m['lead_at_stop'] else None
  # command (carControl.actuators.accel = LongControl output)
  w0, w1 = rng(cc, t_lo, tws)
  ww = cc[w0:w1, 3]
  m['min_wire'] = round(float(ww.min()), 3) if len(ww) else None
  m['wire_at_stop'] = round(float(at(cc, tws, 3)), 3)
  # service entry: the last INACTIVE -> active phase change of the final approach
  ent = [(tl, ph) for tl, ph, prev in logs if t_lo < tl <= tws and prev == 'INACTIVE' and ph != 'INACTIVE']
  m.update(t_entry=None, entry_phase=None, v_entry=None, entry_bite=None)
  if ent:
    te, ph = ent[-1]
    e0, e1 = rng(cc, te, te + 0.6)
    if e1 > e0:
      m.update(t_entry=te, entry_phase=ph, v_entry=round(float(at(cs, te, 1)), 2),
               entry_bite=round(float(cc[e0:e1, 3].min() - at(cc, te - 1e-3, 3)), 3))
  # full-approach pumps (from the last 3 m/s crossing of the engaged approach to the stop) on the command and on the logged
  # aEgo (rharness bites_decel: the car's wheel-accel observation). The pulse-speed slope that the sim uses in recorded mode is
  # quantisation noise below 3 m/s on single drives (5-60 'pumps' per stop on 2232/2235), so it is not reported.
  t3 = last_above(cs, tws, 3.0, 30.0)
  t_full = max(t3, t_lo) if t3 is not None else t_lo
  f0, f1 = rng(cc, t_full, tws)
  a0, a1 = rng(cs, t_full, tws)
  m['t_full'] = round(t_full, 2)
  m['pumps_wire'], m['pump_amp_wire'] = zig_pumps(cc[f0:f1, 0], -cc[f0:f1, 3])
  m['pumps_aego'], m['pump_amp_aego'] = zig_pumps(cs[a0:a1, 0], -cs[a0:a1, 2])
  # terminal (IMU): a_stop / j300 around the standstill edge
  m.update(a_stop=None, j300=None)
  if s['kind'] == 'stop' and all(len(R.get(k, [])) for k in ('imu', 'calib', 'whl')):
    ss = cs[:, 3] > 0.5
    ct = cs[:, 0]
    edges = np.flatnonzero(ss[1:] & ~ss[:-1] & (ct[1:] > tws - 1.0) & (ct[1:] <= tws + 2.0)) + 1
    if len(edges):
      t_flag = float(ct[edges[0]])
      after = np.flatnonzero((ct > t_flag) & (~ss | (cs[:, 5] > 0.5) | (cs[:, 4] > 0.5)))
      t_end = min(float(ct[after[0]]) if len(after) else t_flag + 3.0, t_flag + 3.0)
      try:
        tm = terminal(R, t_flag, t_end)
      except (ValueError, IndexError) as exc:
        tm = dict(error=f'{type(exc).__name__}: {exc}')
      m.update(a_stop=None if tm.get('a_stop') is None else round(tm['a_stop'], 3),
               j300=None if tm.get('j300') is None else round(tm['j300'], 2), t_flag=round(t_flag, 2))
  # felt (census / stop_index definition)
  m['felt'] = felt_for(R, s['t_stop']) if s['kind'] == 'stop' else None
  # hold behaviour: StopReq chatter, hold in pid, 'starting' under a hold
  hold_hi = t_launch if s['kind'] == 'stop' else tws
  m['stopreq_pairs'] = stopreq_pairs(R, tws, hold_hi + 0.5, drv)
  c0, c1 = rng(ctl, tws, hold_hi)
  pid_hold, start_hold = [], []
  for i0, i1 in runs(np.array([(phase_at(logs, x) in HOLD_PHASES) and at(cs, x, 1) < 0.1 for x in ctl[c0:c1, 0]], dtype=bool)
                     & (ctl[c0:c1, 1] == LCS['pid'])):
    if ctl[c0 + i1 - 1, 0] - ctl[c0 + i0, 0] > 0.2:
      pid_hold.append((round(float(ctl[c0 + i0, 0]), 2), round(float(ctl[c0 + i1 - 1, 0] - ctl[c0 + i0, 0]), 2)))
  for i0, _ in runs(np.array([phase_at(logs, x) in HOLD_PHASES for x in ctl[c0:c1, 0]], dtype=bool) & (ctl[c0:c1, 1] == LCS['starting'])):
    start_hold.append(round(float(ctl[c0 + i0, 0]), 2))
  m['hold_in_pid'], m['starting_under_hold'] = pid_hold, start_hold
  # driver takeovers
  m['takeover_approach'] = bool(pedal or drop)
  m['engaged_from'] = round(t_lo, 2) if marks and not (pedal or drop) else None
  # hold: the driver's brake or a disengagement before the launch (a gas launch is the normal resume: launch_by)
  h0, h1 = rng(cs, tws + 0.05, hold_hi - 1.0)
  q0, q1 = rng(cc, tws + 0.05, hold_hi - 1.0)
  m['takeover_hold'] = bool(np.any(cs[h0:h1, 5] > 0) or (q1 > q0 and np.any(cc[q0:q1, 2] < 0.5)))
  k0, k1 = rng(cs, t_launch - 1.0, t_launch + 0.2)
  m['launch_by'] = None if s['kind'] == 'rolling' else ('driver' if np.any(cs[k0:k1, 4] > 0) else 'car')
  m['phases'] = [(tl, ph) for tl, ph, _ in logs if tws - 10.0 <= tl <= hold_hi + 1.0]
  return m


def felt_for(R, t_stop):
  """Census felt (terminal_smoothness.score_terminal window + felt_jerk on aEgo vs the 3-sample-median carOutput)."""
  from openpilot.tools.stopping.review import terminal_smoothness as ts
  cs, co = R['cs'], R['co']
  if len(co) < 3:
    return None
  c0, c1 = rng(co, t_stop - 9.0, t_stop + 2.0)
  x = co[c0:c1].copy()
  if len(x) >= 3:
    x[1:-1, 1] = np.median(np.stack([x[:-2, 1], x[1:-1, 1], x[2:, 1]]), axis=0)
  i0, i1 = rng(cs, t_stop - 8.0, t_stop + 1.2)
  T, V, A = cs[i0:i1, 0], cs[i0:i1, 1], cs[i0:i1, 2]
  j = np.searchsorted(x[:, 0], T, side='right') - 1
  ok = j >= 0
  T, V, A, W = T[ok], V[ok], A[ok], x[j[ok], 1]
  try:
    sc = ts.score_terminal(list(T), list(V), list(W), t_target=t_stop)
  except (ValueError, IndexError, ZeroDivisionError):
    return None
  if not sc:
    return None
  k0, k1 = sc.pop('k_window')
  return ts.felt_jerk(list(T), list(A), k0, k1)


def match_bookmarks(bms, stops, hlds):
  """find_bookmarked_bad_stops rule: a stop whose [approach start - 8 s, wheel stop + 8 s] holds the bookmark (nearest
  wheel stop wins), else the nearest stop within 25 s; else the hold that contains it; else unmatched."""
  out = []
  for b in bms:
    inw = [s for s in stops if s['m']['t_lo'] - 8.0 <= b <= s['t_ws'] + 8.0]
    if inw:
      s = min(inw, key=lambda s: abs(b - s['t_ws']))
      out.append(dict(t=b, stop=s['id'], how='window'))
      continue
    near = min(stops, key=lambda s: abs(b - s['t_ws'])) if stops else None
    if near is not None and abs(b - near['t_ws']) <= 25.0:
      out.append(dict(t=b, stop=near['id'], how='nearest'))
      continue
    h = next((h for h in hlds if h['t_start'] - 8.0 <= b <= h['t_end'] + 8.0), None)
    out.append(dict(t=b, stop=None, hold=None if h is None else h['t_start'], how='hold' if h else 'none'))
  return out


# ==== logged race class (E3's target) ======================================================================================
def release_end_races(R):
  """Logged RELEASE -> INACTIVE at rest behind a radar-stopped lead (|vLead| median < 0.25 m/s over 0.5 s, gap not grown).
  race = the car then moved >= 0.25 m before the lead departed (vLead > 0.5 or the gap grew 0.5 m) and without driver gas."""
  cs, rs, ctl, sc = R['cs'], R['rs'], R['ctl'], R.get('scc12')
  out = []
  for tl, ph, prev in R['logs']:
    if prev != 'RELEASE' or ph != 'INACTIVE' or at(cs, tl, 1) >= 0.1 or not at(rs, tl, 1) > 0:
      continue
    r0, r1 = rng(rs, tl - 0.5, tl)
    w = rs[r0:r1]
    if not len(w) or np.any(w[:, 1] <= 0) or abs(float(np.median(w[:, 3]))) >= 0.25:
      continue
    d0 = float(np.median(w[:, 2]))
    r0, r1 = rng(rs, tl, tl + 5.0)
    w = rs[r0:r1]
    dep = np.flatnonzero((w[:, 1] <= 0) | (w[:, 3] > 0.5) | (w[:, 2] > d0 + 0.5))
    t_dep = float(w[dep[0], 0]) if len(dep) else tl + 5.0
    k0, k1 = rng(cs, tl, t_dep)
    gas = np.flatnonzero(cs[k0:k1, 4] > 0)
    t_cut = float(cs[k0 + gas[0], 0]) if len(gas) else t_dep
    k1 = int(np.searchsorted(cs[:, 0], t_cut))
    x = float(np.sum(cs[k0 + 1:k1, 1] * np.diff(cs[k0:k1, 0]))) if k1 - k0 > 1 else 0.0
    c0, c1 = rng(ctl, tl, tl + 0.5)
    stq = None
    if sc is not None and len(sc):
      q0, q1 = rng(sc, tl - 0.05, tl + 1.0)
      clr = np.flatnonzero(sc[q0:q1, 2] < 0.5)
      stq = round(float(sc[q0 + clr[0], 0]), 2) if len(clr) else None
    reentry = next((round(t2, 2) for t2, p2, pv2 in R['logs'] if tl <= t2 <= tl + 0.5 and pv2 == 'INACTIVE' and p2 != 'INACTIVE'), None)
    out.append(dict(t=round(tl, 2), gap=round(d0, 2), travel_before_lead=round(x, 2), t_lead_departs=round(t_dep, 2),
                    driver_gas=round(t_cut, 2) if len(gas) else None, starting=bool(np.any(ctl[c0:c1, 1] == LCS['starting'])),
                    stopreq_clear=stq, reentry=reentry, race=x >= 0.25))
  return out


# ==== replay arms ==========================================================================================================
# Three job kinds per arm (one spawn pool per arm, PROCESSES workers, cached per span):
#   replay  - LongControl + service + sender on the logged plan (fidelity, E3 arms, the line trial's reference arm);
#   planner - the arm's LongitudinalPlanner in lockstep on the logged planner inputs, at every PLAN_BOUND_MS;
#   delta   - the replay with the plan = logged + (this arm's planner - the reference arm's planner) (the line trial's other arm).
def _cache_dir(kind, key, ref_key=None):
  return WORK / f'{kind}_v{DR.CACHE_V}' / (key if ref_key is None else f'{key}__vs__{ref_key}')


def cache_key(commit, overrides):
  """Cache key of one arm: the arm (commit + overrides) and its replay implementation (drive_replay.impl_key with this module's job
  code). A delta cache names both arms' keys, so a planner change invalidates the deltas built on it."""
  code = ''.join(inspect.getsource(f) for f in (_job, load_planner, pick_bound, plan_fidelity))
  return DR.arm_key(commit, overrides) + '_' + DR.impl_key(commit, code)


def _arm_init(commit, overrides):
  DR.load(commit, overrides)


def load_planner(key, span):
  """{bound_ms: columns} + 'origin_ns' of a cached planner job."""
  with np.load(_cache_dir('planner', key) / f'{span}.npz') as z:
    out = {b: {c: z[f'{b}__{c}'] for c in DR.ICOLS + DR.PCOLS} for b in DR.PLAN_BOUND_MS}
    out['origin_ns'] = int(z['origin_ns'])
  return out


def plan_fidelity(P, lo):
  """Reference planner replay vs the logged aTarget on the engaged ticks inside the span (the warm-up excluded)."""
  t = (P['ns'] - P['origin_ns']) * 1e-9 if 'origin_ns' in P else P['t']
  k = (t >= lo) & (P['engaged'] > 0.5)
  d = np.abs(P['at'][k] - P['log_at'][k])
  return dict(ticks=int(k.sum()), over_005=int((d > 0.05).sum()), max=round(float(d.max()), 3) if len(d) else None,
              ss_diff=int(np.sum(P['ss'][k] != P['log_ss'][k])))


def pick_bound(Q, lo):
  """The input bound whose reference planner replay matches the logged plan best inside the span (ties: the smaller)."""
  return min(DR.PLAN_BOUND_MS, key=lambda b: (plan_fidelity(dict(Q[b], origin_ns=Q['origin_ns']), lo)['over_005'], b))


def _job(job):
  kind, span, key, ref_key = job
  out = _cache_dir(kind, key, ref_key) / f"{span['span']}.npz"
  if out.exists():
    return span['span'], 'cached'
  try:
    if kind == 'planner':
      ev = DR.planner_events(span['route'], span['lo'], span['hi'])
      P = DR.planner_run(ev, DR.PLAN_BOUND_MS)
      r = {f'{b}__{c}': v for b, cols in P.items() for c, v in cols.items()}
      r['origin_ns'] = np.array(ev['origin'], dtype=np.int64)
    else:
      d = DR.frames_for(span)
      plan = None
      if kind == 'delta':
        P, Q = load_planner(key, span['span']), load_planner(ref_key, span['span'])
        b = pick_bound(Q, span['lo'])
        tg, ss, matched = DR.plan_targets(d['frames'], P[b], Q[b])
        plan = (tg, ss)
      r = DR.run(d, plan)
      if kind == 'delta':
        r.update(bound=np.array(b), matched=np.array(matched))
    out.parent.mkdir(parents=True, exist_ok=True)
    tmp = out.with_name(out.name + f'.tmp{os.getpid()}.npz')
    np.savez_compressed(tmp, **r)
    os.replace(tmp, out)
    return span['span'], 'ok'
  except Exception as exc:  # one span must not drop the run; reported per span
    import traceback
    out.parent.mkdir(parents=True, exist_ok=True)
    out.with_suffix('.err').write_text(traceback.format_exc())
    return span['span'], f'ERROR {type(exc).__name__}: {exc}'


def run_arm(commit, overrides, spans, log, kind='replay', ref_key=None):
  """Run one job kind on one arm. Returns {span: columns or error string} (planner: load_planner's dict)."""
  key = cache_key(commit, overrides)
  cdir = _cache_dir(kind, key, ref_key)
  todo = [s for s in spans if not (cdir / f"{s['span']}.npz").exists()]
  if todo:
    t0 = time.monotonic()
    with get_context('spawn').Pool(min(PROCESSES, len(todo)), initializer=_arm_init, initargs=(commit, overrides)) as pool:
      for name, msg in pool.imap_unordered(_job, [(kind, s, key, ref_key) for s in sorted(todo, key=lambda s: s['lo'] - s['hi'])],
                                           chunksize=1):
        if msg.startswith('ERROR'):
          log(f'  {kind} {cdir.name} {name}: {msg}')
    log(f'  {kind} {cdir.name}: {len(todo)} spans in {time.monotonic() - t0:.0f} s')
  res: dict = {}
  for s in spans:
    p = cdir / f"{s['span']}.npz"
    if p.exists():
      if kind == 'planner':
        res[s['span']] = load_planner(key, s['span'])
      else:
        with np.load(p) as z:
          res[s['span']] = {k: z[k] for k in z.files}
    else:
      err = p.with_suffix('.err')
      res[s['span']] = err.read_text().strip().splitlines()[-1] if err.exists() else 'missing'
  return res


def fidelity(R, spans, rep):
  """Replay of the drive's commit vs the log (warm-up excluded): commanded accel per frame, StopReq toggles (sendcan SCC12,
  matched within 50 ms) and every logged service phase change (matched within 50 ms; the replay's extra INACTIVE steps are
  resets, which the service does not log). OK = no unmatched StopReq toggle or phase change, command errors > 0.05 on at most
  0.1 % of the frames (RHARNESS 4.1: the recorded replay carries the controller state from the window start)."""
  n = big = 0
  abs_sum, worst = 0.0, (0.0, None)
  st_rep = st_log = st_miss = ph_log = ph_miss = 0
  errors = []
  for s in spans:
    r = rep.get(s['span'])
    if not isinstance(r, dict):
      errors.append((s['span'], r))
      continue
    k = r['t'] >= r['t'][0] + FIDELITY_SKIP_S
    t = r['t'][k]
    dw = np.abs(r['wire'][k] - r['rec'][k])
    n += len(dw)
    big += int((dw > 0.05).sum())
    abs_sum += float(dw.sum())
    if len(dw) and dw.max() > worst[0]:
      worst = (float(dw.max()), round(float(t[np.argmax(dw)]), 2))
    sc = R.get('scc12')
    if sc is not None and len(sc) and len(t):
      t_rep = t[np.flatnonzero(np.diff(r['stopreq'][k]) != 0) + 1]
      q0, q1 = rng(sc, t[0], t[-1])
      t_log = sc[q0:q1, 0][np.flatnonzero(np.diff(sc[q0:q1, 2]) != 0) + 1]
      st_rep += len(t_rep)
      st_log += len(t_log)
      st_miss += sum(not np.any(np.abs(t_log - x) <= 0.05) for x in t_rep) + sum(not np.any(np.abs(t_rep - x) <= 0.05) for x in t_log)
    if len(t):
      ph = r['phase'][k]
      ch = np.flatnonzero(np.diff(ph) != 0) + 1
      rep_ch = [(t[i], PHASES[ph[i]]) for i in ch]
      for tl, p, _ in R['logs']:
        if t[0] < tl <= t[-1]:
          ph_log += 1
          ph_miss += not any(abs(a - tl) <= 0.05 and q == p for a, q in rep_ch)
  ok = None if (n == 0 and not errors) else (big <= max(5, 1e-3 * n) and st_miss == 0 and ph_miss == 0 and not errors)   # None: no span
  return dict(frames=n, mean_abs=round(abs_sum / max(n, 1), 6), over_005=big, worst=worst, stopreq_toggles=(st_rep, st_log),
              stopreq_unmatched=st_miss, phase_changes=ph_log, phase_unmatched=ph_miss, errors=errors, ok=ok)


def diff_events(on, off, merge_s=0.5):
  """Runs of frames where command (> 1e-4), phase, StopReq or LongControl state differ between two arms of one span."""
  n = min(len(on['t']), len(off['t']))
  dif = (np.abs(on['wire'][:n] - off['wire'][:n]) > 1e-4) | (on['phase'][:n] != off['phase'][:n]) | \
        (on['stopreq'][:n] != off['stopreq'][:n]) | (on['lcs'][:n] != off['lcs'][:n])
  t = on['t'][:n]
  ev = []
  for i0, i1 in runs(dif):
    if ev and t[i0] - t[ev[-1][1] - 1] <= merge_s:
      ev[-1] = (ev[-1][0], i1)
    else:
      ev.append((i0, i1))

  def seq(x, a, b):
    out = []
    for v in x[a:b]:
      if not out or out[-1] != int(v):
        out.append(int(v))
    return out
  rows = []
  for i0, i1 in ev:
    a, b = max(i0 - 1, 0), min(i1 + 1, n)
    rows.append(dict(t0=round(float(t[i0]), 2), t1=round(float(t[i1 - 1]), 2), frames=int(dif[i0:i1].sum()),
                     max_dwire=round(float(np.max(on['wire'][i0:i1] - off['wire'][i0:i1])), 3),
                     min_dwire=round(float(np.min(on['wire'][i0:i1] - off['wire'][i0:i1])), 3),
                     phase_on=[PHASES[p] if 0 <= p < len(PHASES) else p for p in seq(on['phase'], a, b)],
                     phase_off=[PHASES[p] if 0 <= p < len(PHASES) else p for p in seq(off['phase'], a, b)],
                     lcs_on=seq(on['lcs'], a, b), lcs_off=seq(off['lcs'], a, b),
                     stopreq_on=seq(on['stopreq'], a, b), stopreq_off=seq(off['stopreq'], a, b),
                     rehold=bool(np.any(on['rehold'][i0:i1]))))
  return rows


def e3_checks(on, off, inp, live):
  """PLAN section 60 per-drive rules for every E3 re-hold (rising edge of the re-hold state in the ON arm).
  live: the drive ran the flag, so the logged motion is the ON world (motion/grab rules apply); else counterfactual."""
  out = []
  t = on['t']
  n = min(len(t), len(off['t']), len(inp['t']))
  for k in np.flatnonzero(np.diff(np.r_[0, on['rehold'][:n]]) > 0):
    e = k + int(np.argmax(on['rehold'][k:n] == 0)) if np.any(on['rehold'][k:n] == 0) else n
    rest = inp['v'][k:e] < 0.1
    viol, notes = [], []
    st = np.flatnonzero(on['lcs'][k:e] == LCS['starting'])
    if len(st):
      viol.append(f"'starting' under the re-hold at {t[k + st[0]]:.2f}")
    drop = np.flatnonzero(rest & (on['stopreq'][k:e] == 0))
    if len(drop) > 2:   # the re-hold frame and the next sender frame may still carry the previous request
      viol.append(f'StopReq clear at rest under the re-hold at {t[k + drop[0]]:.2f} ({len(drop)} frames)')
    esc = np.flatnonzero(rest & (on['active'][k:e] == 0))
    if len(esc):
      viol.append(f'service not active at rest under the re-hold at {t[k + esc[0]]:.2f} (ownership escape)')
    # launch timing: first StopReq clear after the firing, ON vs OFF
    def clear_after(arm, k=k):
      z = np.flatnonzero(arm['stopreq'][k:n] == 0)
      return float(arm['t'][k + z[0]]) if len(z) else None
    t_on, t_off = clear_after(on), clear_after(off)
    gap0 = inp['gap'][k]
    later = None if (t_on is None or t_off is None) else round(t_on - t_off, 2)
    if later is not None and later > 0.3:
      j = int(np.searchsorted(t, t_off))
      departed = inp['lead'][j] == 0 or inp['vl'][j] > 0.25 or inp['gap'][j] > gap0 + 0.3
      notes.append(f'StopReq clears {later:+.2f} s vs flag off' + (' (flag off cleared before the lead departed: race removed)'
                                                                   if not departed else ' (LATE: review with the rating)'))
    if live:
      x = np.r_[0.0, np.cumsum(inp['v'][k + 1:n] * np.diff(t[k:n]))]
      mv = np.flatnonzero(x > 0.25)
      if len(mv):
        j = k + mv[0]
        if inp['lead'][j] and inp['vl'][j] < 0.25 and inp['gap'][j] <= gap0 + 0.3 and not np.any(inp['gas'][k:j + 1]):
          viol.append(f'car moved 0.25 m at {t[j]:.2f} before the lead departed, no driver input')
      e5 = min(int(np.searchsorted(t, t[min(e, n - 1)] + 5.0)), n)
      w = on['wire'][e:e5]
      moving = inp['v'][e:e5] > 0.1
      # relative to the flag-off replay (PLAN section 77): a grab the flag-off arm also commands is not E3's
      grab = np.flatnonzero(moving & (w <= -0.5) & (w < off['wire'][e:e5] - 0.1) & (inp['brake'][e:e5] == 0))
      if len(grab):
        viol.append(f"grab {w[grab[0]]:.2f} (flag off {off['wire'][e + grab[0]]:.2f}) at {t[e + grab[0]]:.2f} after the re-hold")
    out.append(dict(t=round(float(t[k]), 2), t_end=round(float(t[min(e, n - 1)]), 2), gap=round(float(gap0), 2),
                    vlead=round(float(inp['vl'][k]), 2), lead=int(inp['lead'][k]), brake=bool(inp['brake'][k]),
                    stopreq_clear_on=t_on, stopreq_clear_off=t_off, later_s=later, violations=viol, notes=notes))
  return out


# ==== SANTA_FE_STOP_LINE trial (entry bite: planner stop line + band-consistent governor) ==================================
# The per-drive revert rules (cycle_20261003: eb4_lfl_final_build.json per_drive_rule, corrected by eb5_lfl_r4_check.json and
# LFL_CODE_REVIEW_astra.md): every rule compares the flag-on arm with the flag-off replay of the same drive (PLAN section 77).
# Neither the line nor the band latch is published: the planner lockstep replay reconstructs the line, the replay's service the
# band latch. Open loop: the logged motion is the same in both arms, so landing effects (rest, gaps, a_stop) of the arm that did
# not drive are estimated from the wire difference (double integral up to the wheel stop; mean over the last 0.5 s).
LR = dict(rel_tick=0.125, j_release=2.5, out_v=0.3, rise=0.05, burst_ticks=8, moved=0.3, uncert=0.3, crawl_vl=0.15, crawl_ticks=20,
          queue_vl=0.5, queue_v=2.5, queue_bind=0.1, queue_s=0.5, rearm_s=2.0, rearm_v=2.5, pump=0.10, pump_any=0.15, pump_info=0.05,
          rest=3.5, min_gap=3.2, ws_gap=6.0, worse_m=0.2, a_stop=-0.65, capture=-0.60, worse_a=0.05, crawl_lo=0.02, crawl_hi=0.2,
          crawl_s=0.8, crawl_deepen=0.1, downhill=-2.0, downhill_rest=3.8, hold_move=0.3, creep_grab=0.3, creep_v=0.5, own_wire=-1.0,
          own_v=1.5, own_after_s=10.0, own_s=0.2, h5_gap=0.3, approach_s=30.0)
# rule -> (what trips it, events per drive that revert); R4 also reverts on one event >= pump_any
LINE_RULES = {
  'R1': ('code defect: braking-component release > 0.125 per tick beyond OFF, positive floor, positive command capped, burst-hold ' +
         'failure, authority/deepening on a provenance rejection', 1),
  'R2': ('line episode > 0.3 deeper than OFF behind a lead not certified stopped (stop-commit certificate off, out of the stopped ' +
         'class beyond the burst hold, or a crawler: median radar vLead > 0.15 over the last 1 s of the episode)', 1),
  'R3': ('queue restart: the line binds > 0.1 below OFF for > 0.5 s after vLead >= 0.5 above 2.5 m/s', 2),
  'R4': ('line release/re-arm pump at the wire vs OFF (every re-arm within 2 s above 2.5 m/s): >= 0.10', 2),
  'R5': ('landing of an affected stop worse than OFF: rest < 3.5 m, min gap < 3.2 m or wheel-stop gap > 6 m', 1),
  'R6': ('a_stop <= -0.65 (or stop wire <= -0.65 after a capture < -0.60) worse than OFF', 2),
  'R7': ('crawl-then-grab: > 0.8 s at 0.02-0.2 m/s, then a wire deepening >= 0.1 (beyond OFF) before the wheel stop', 1),
  'R8': ('downhill (grade <= -2 %), affected stop: rest < 3.8 m worse than OFF, or > 0.3 m motion in HOLD with a shallower hold', 1),
  'R9': ('creeping lead: a service re-entry below 0.5 m/s re-grabs >= 0.3 (beyond OFF)', 1),
  'R10': ('ownership / StopReq beyond OFF: stopping wire <= -1.0 below 1.5 m/s without service ownership after a line episode, ' +
          'StopReq chatter at rest, a hold at rest in pid, starting under a hold', 1),
  'H5': ('gap at first motion > 0.3 m larger than OFF, no launch command where OFF launches, or a false launch', 1),
  'R11': ('Radek: an affected stop rated late / long, or an unwanted rolling arrival (s20 class) -- from the ratings', 0),
}


def _i(t, x):
  return int(np.clip(np.searchsorted(t, x), 0, max(len(t) - 1, 0)))


def _ev(rule, span, t, v, what, value, **kw):
  return dict(rule=rule, span=span, t=round(float(t), 2), v=None if v is None else round(float(v), 2), what=what,
              value=round(float(value), 3), **kw)


def line_state(Pn):
  """Line activity on the planner ticks: act (floor present), armed (not releasing), bind (the floor is the output)."""
  lf = Pn['lf']
  act = np.isfinite(lf)
  armed = act & (np.nan_to_num(Pn['armed']) > 0.5)
  bind = act & (np.abs(Pn['at'] - np.where(act, lf, 0.0)) < 1e-6)
  return act, armed, bind


def lead_ground(t, v, d):
  """Lead ground position: range + ego travel (good for a 0.3 m 'did not move' test over 1.5 s, not for a speed)."""
  return d + np.r_[0.0, np.cumsum(np.maximum(v[1:], 0.0) * np.diff(t))]


def line_span_events(Pn, Pf, Ln, Lf, inp, span):
  """R1-R4, R9, R10 on one span. Pn / Pf: flag-on / flag-off planner columns on the same ticks (+ route-relative 't');
  Ln / Lf: flag-on / flag-off replay columns; inp: the logged inputs (drive_replay.inputs)."""
  out = []
  t, v, vl, d = Pn['t'], Pn['v'], Pn['vl'], Pn['d']
  n = len(t)
  if n < 3:
    return out
  ok = (Pn['engaged'] > 0.5) & (Pn['override'] < 0.5)
  aon, aoff, lf, cmd = Pn['at'], Pf['at'], Pn['lf'], Pn['cmd']
  act, armed, bind = line_state(Pn)
  # R1 code defects
  for k in range(1, n):
    if bind[k - 1] and ok[k - 1] and ok[k]:
      dr = min(aon[k], 0.0) - min(aon[k - 1], 0.0)   # the braking component (the hand-back to a positive command is specified)
      if dr > max(LR['rel_tick'], LR['j_release'] * (t[k] - t[k - 1])) + 1e-6 and dr > min(aoff[k], 0.0) - min(aoff[k - 1], 0.0) + 1e-6:
        out.append(_ev('R1', span, t[k], v[k], 'braking-component release beyond OFF in one tick', dr))
  for i0, i1 in runs(act & (np.nan_to_num(lf) > 1e-9)):
    out.append(_ev('R1', span, t[i0], v[i0], f'positive line floor ({i1 - i0} ticks)', np.nanmax(lf[i0:i1])))
  pcap = bind & (np.nan_to_num(cmd) > 1e-9) & (aon > 1e-9) & (np.nan_to_num(lf) < np.nan_to_num(cmd) - 1e-9)
  for i0, i1 in runs(pcap):
    out.append(_ev('R1', span, t[i0], v[i0], f'positive command capped by the line ({i1 - i0} ticks)', cmd[i0] - aon[i0]))
  xl = lead_ground(t, v, d)
  tid = Pn['tid']
  for k0, k1 in runs(np.isfinite(vl) & (vl > LR['out_v']) & ok):
    if k0 == 0 or not armed[k0 - 1] or k1 - k0 > LR['burst_ticks'] or not np.all(tid[k0 - 1:k1] == tid[k0 - 1]):
      continue
    if np.nanmax(vl[k0:k1]) >= vl[k0] + LR['rise']:
      continue   # the speed rose: a launch, which the line may release on
    k15 = _i(t, t[k0] + 1.5)
    if np.isfinite(xl[k15] - xl[k0 - 1]) and xl[k15] - xl[k0 - 1] >= LR['moved']:
      continue   # the lead moved
    if np.any(~armed[k0:k1]):
      out.append(_ev('R1', span, t[k0], v[k0], f'burst-hold failure: released during a {k1 - k0}-tick level excursion of a lead ' +
                     'that did not move', float(np.nanmax(vl[k0:k1]))))
  lead = np.isfinite(tid)   # a dropped lead frame is held by design (no provenance or certificate on it)
  prov_ok = np.nan_to_num(Pn['prov'], nan=1.0) > 0.5
  in_armed = np.nan_to_num(Pn['in_armed']) > 0.5
  for k in np.flatnonzero(ok & lead & ~prov_ok & (in_armed | armed)):
    deeper = k > 0 and act[k] and act[k - 1] and lf[k] < lf[k - 1] - 1e-9
    if armed[k] or deeper:
      out.append(_ev('R1', span, t[k], v[k], 'line ' + ('armed' if not in_armed[k] else 'kept authority') + ' on a provenance rejection'
                     + (' and deepened' if deeper else ''), lf[k]))
  # R2 a lead not certified stopped while the line is armed (range + odometry ground speed is too noisy for this: 0.5-1.6 m/s
  # behind stopped leads on 00002235, so the planner's own certificate / class and the radar speed decide)
  cls = np.nan_to_num(Pn['cls'], nan=1.0) > 0.5
  cert = ~(lead & cls) | (np.nan_to_num(Pn['cert'], nan=1.0) > 0.5)   # judged on an in-class lead (excursions: outc)
  outc = np.zeros(n, bool)
  for i0, i1 in runs(armed & ~cls):
    outc[i0 + LR['burst_ticks']:i1] = True   # the code ends a line after the burst hold
  crawler = np.zeros(n, bool)
  vl0 = np.nan_to_num(vl)
  for i0, i1 in runs(armed):
    for k in range(i0 + LR['crawl_ticks'] - 1, i1):
      crawler[k] = np.median(vl0[k - LR['crawl_ticks'] + 1:k + 1]) > LR['crawl_vl']
  unc = armed & ok & (~cert | outc | crawler)
  for i0, i1 in runs(unc & (aoff - aon > LR['uncert'])):
    why = ', '.join(x for x, m in (('certificate off', ~cert), ('out of the stopped class', outc), ('crawler', crawler)) if m[i0:i1].any())
    out.append(_ev('R2', span, t[i0], v[i0], f'line {np.max(aoff[i0:i1] - aon[i0:i1]):.2f} deeper than OFF for {t[i1 - 1] - t[i0] + 0.05:.2f} s ' +
                   f'({why})', np.max(aoff[i0:i1] - aon[i0:i1])))
  # R3 queue restart
  for k0, k1 in runs(np.isfinite(vl) & (vl >= LR['queue_vl']) & (v > LR['queue_v']) & ok):
    b = act[k0:k1] & (aoff[k0:k1] - aon[k0:k1] > LR['queue_bind'])
    s = float(np.sum(np.diff(t[k0:k1 + 1] if k1 < n else np.r_[t[k0:k1], t[-1] + 0.05])[b]))
    if s > LR['queue_s']:
      out.append(_ev('R3', span, t[k0], v[k0], f'queue restart: line binds > 0.1 below OFF for {s:.2f} s after vLead >= 0.5', s))
  # R4 every release -> re-arm within 2 s above 2.5 m/s, at the wire (rearm_wire.py, relative to OFF)
  tl, dw = Ln['t'], Ln['wire'] - Lf['wire']
  k = 1
  while k < n:
    if armed[k - 1] and not armed[k] and v[k] > LR['rearm_v'] and ok[k]:
      j = k
      while j < n and not armed[j]:
        j += 1
      if j < n and t[j] - t[k] <= LR['rearm_s']:
        j0, je, jr = (int(np.searchsorted(tl, t[k])), min(int(np.searchsorted(tl, t[j] + 0.3)) + 1, len(dw)),
                      int(np.searchsorted(tl, t[j] + 1.5)) + 1)
        if 0 < j0 < je:
          jm = j0 + int(np.argmax(dw[j0:je]))
          give, red = float(dw[jm] - dw[j0 - 1]), float(dw[jm] - np.min(dw[jm:max(jr, jm + 1)]))
          if min(give, red) >= LR['pump_info']:
            vmax = np.nanmax(vl[k:j + 1]) if np.isfinite(vl[k:j + 1]).any() else np.nan
            out.append(_ev('R4', span, t[k], v[k], f're-arm after {t[j] - t[k]:.2f} s: wire gives {give:.3f}, re-deepens {red:.3f} vs OFF ' +
                           f'(vLead max {vmax:.2f})', min(give, red)))
      k = j
    k += 1
  # R9 creeping lead re-grab (service re-entry below 0.5 m/s while moving behind a moving lead)
  ph, vv = Ln['phase'], inp['v']
  for k in np.flatnonzero(np.isin(ph[1:], (1, 2)) & np.isin(ph[:-1], (0, 5))) + 1:
    if LR['crawl_lo'] < vv[k] < LR['creep_v'] and inp['lead'][k] and inp['vl'][k] > 0.05:
      kk = max(int(np.searchsorted(tl, tl[k] + 1.0)), k + 1)
      g_on, g_off = Ln['wire'][k - 1] - Ln['wire'][k:kk].min(), Lf['wire'][k - 1] - Lf['wire'][k:kk].min()
      if g_on >= LR['creep_grab'] and g_on > g_off + LR['worse_a']:
        out.append(_ev('R9', span, tl[k], vv[k], f're-entry re-grab {g_on:.2f} (OFF {g_off:.2f}) behind a lead at {inp["vl"][k]:.2f} m/s', g_on))
  # R10 ownership / StopReq / hold protocol, flag on beyond flag off
  t_act = t[act]
  last = np.searchsorted(t_act, tl, side='right') - 1
  recent = (last >= 0) & (tl - t_act[np.clip(last, 0, max(len(t_act) - 1, 0))] <= LR['own_after_s']) if len(t_act) else np.zeros(len(tl), bool)

  def legacy(L):
    return (vv < LR['own_v']) & (L['lcs'] == LCS['stopping']) & (L['owning'] == 0) & (L['wire'] <= LR['own_wire'])
  for i0, i1 in runs(legacy(Ln) & ~legacy(Lf) & recent):
    if tl[i1 - 1] - tl[i0] >= LR['own_s']:
      out.append(_ev('R10', span, tl[i0], vv[i0], f'stopping wire <= -1.0 without service ownership for {tl[i1 - 1] - tl[i0]:.2f} s after a ' +
                     'line episode (OFF not)', Ln['wire'][i0:i1].min()))
  drv = (inp['gas'] > 0) | (inp['brake'] > 0) | (inp['act'] == 0)

  def chatter(L):   # StopReq clear -> set at rest within 1 s (PLAN 82, sim.gates.chatter at the drive's 0.1 m/s rest), no driver near
    return [a for a, b in SG.chatter(dict(stopreq=L['stopreq'], t=tl, v_true=vv), 0, len(tl), rest_v=0.1)
            if not drv[_i(tl, a - 0.5):_i(tl, b + 0.5) + 1].any()]
  off_ch = chatter(Lf)
  for x in chatter(Ln):
    if not any(abs(x - y) <= 0.1 for y in off_ch):
      out.append(_ev('R10', span, x, 0.0, 'StopReq clear -> set at rest within 1 s (OFF not)', 1.0))
  hold = np.isin(Ln['phase'], (3, 4))
  for i0, i1 in runs(hold & (vv < 0.1) & (Ln['lcs'] == LCS['pid']) & ~(np.isin(Lf['phase'], (3, 4)) & (Lf['lcs'] == LCS['pid']))):
    if tl[i1 - 1] - tl[i0] > 0.2:
      out.append(_ev('R10', span, tl[i0], vv[i0], f'service hold at rest in pid for {tl[i1 - 1] - tl[i0]:.2f} s (OFF not)', tl[i1 - 1] - tl[i0]))
  for i0, _ in runs(hold & (Ln['lcs'] == LCS['starting']) & ~(np.isin(Lf['phase'], (3, 4)) & (Lf['lcs'] == LCS['starting']))):
    out.append(_ev('R10', span, tl[i0], vv[i0], "'starting' under a hold (OFF not)", 1.0))
  return out


def entry_bite(L, tl, a, b):
  """Service entry bite of one arm: min command in the 0.6 s after the last INACTIVE -> active phase change in [a, b] minus the
  command before it."""
  ph = L['phase']
  k = np.flatnonzero((ph[1:] != 0) & (ph[:-1] == 0) & (tl[1:] > a) & (tl[1:] <= b)) + 1
  if not len(k):
    return None
  e = int(k[-1])
  return round(float(L['wire'][e:_i(tl, tl[e] + 0.6) + 1].min() - L['wire'][e - 1]), 3)


def line_stop(st, Pn, Pf, Ln, Lf, inp, R):
  """Per-stop trial activity (for Radek's ratings) and R5-R8 / H5 on one stop. st: the report's stop dict (logged metrics)."""
  tws, span = st['t_ws'], st.get('span')
  t, tl = Pn['t'], Ln['t']
  act, armed, bind = line_state(Pn)
  kp = (t >= tws - LR['approach_s']) & (t <= tws + 0.5)
  kl = (tl >= tws - LR['approach_s']) & (tl <= tws + 1.0)
  dw = Ln['wire'] - Lf['wire']
  extra = Pf['at'] - Pn['at']
  line = bool(np.any(act & kp) and np.max(np.where(kp, extra, 0.0)) > 0.01)
  band = bool(np.any(Ln['band'][kl] > 0))
  ka = np.flatnonzero(armed & kp)
  row = dict(id=st['id'], span=span, kind=st['kind'], line=line, band=band, acted=line or band,
             line_t=round(float(t[ka[0]]), 2) if len(ka) else None, line_v=round(float(Pn['v'][ka[0]]), 2) if len(ka) else None,
             line_gap=round(float(Pn['d'][ka[0]]), 2) if len(ka) else None,
             line_s=round(float(np.sum(armed[kp]) * 0.05), 2), plan_extra=round(float(np.max(np.where(kp, extra, 0.0))), 3),
             wire_deeper=round(float(max(0.0, -dw[kl].min())), 3) if kl.any() else 0.0,
             wire_shallower=round(float(max(0.0, dw[kl].max())), 3) if kl.any() else 0.0,
             bite_on=entry_bite(Ln, tl, tws - LR['approach_s'], tws), bite_off=entry_bite(Lf, tl, tws - LR['approach_s'], tws), rules=[])
  # Landing (open loop: the logged motion is the same in both arms). The LOGGED landing and the LOGGED command stand for the flag-on
  # world -- exact on a live drive; on a counterfactual drive this is the 'as if live' reading -- and the flag-off value is the
  # logged one minus the trial's command effect dw = on - off: the travel difference x_on - x_off of its double integral up to the
  # wheel stop (dx; its size is an open-loop upper bound, its sign attributes the landing), the stop-level difference over the last
  # 0.5 s (da). Closer landings use the larger of the full-approach and the in-band travel difference (from the flag-on service
  # entry: upstream line braking would otherwise mask a band profile that brakes less in the band).
  k_ws = _i(tl, tws)
  ent = np.flatnonzero((Ln['phase'][1:] != 0) & (Ln['phase'][:-1] == 0) & (tl[1:] > tws - LR['approach_s']) & (tl[1:] <= tws))
  t_in = float(tl[ent[-1] + 1]) if len(ent) else None

  def travel(a):
    idx = np.flatnonzero((tl >= a) & (tl <= tws))
    if len(idx) < 2:
      return 0.0
    dtt = np.diff(tl[idx], prepend=tl[idx[0]])
    return float(np.sum(np.cumsum(dw[idx] * dtt) * dtt))
  dx = travel(tws - LR['approach_s'])
  dx_close = max(dx, travel(t_in)) if t_in is not None else dx
  da = float(np.mean(dw[_i(tl, tws - 0.5):k_ws + 1]))
  rec = inp['rec']
  ws = float(inp['gap'][k_ws]) if inp['lead'][k_ws] else None
  r2 = lambda x: None if x is None else round(float(x), 2)  # noqa: E731
  rest, mg, a_stop = st.get('rest'), st.get('min_gap'), st.get('a_stop')
  row.update(dx=round(dx, 2), dx_close=round(dx_close, 2), rest_on=rest, rest_off=r2(None if rest is None else max(rest + dx_close, 0.0)),
             ws_on=r2(ws), ws_off=r2(None if ws is None else max(ws + dx, 0.0)), a_on=a_stop, a_off=None if a_stop is None else round(a_stop - da, 3),
             wire_stop_on=round(float(rec[k_ws]), 3), wire_stop_off=round(float(rec[k_ws] - dw[k_ws]), 3))
  ev = []
  w = LR['worse_m']
  if row['acted'] and st['kind'] == 'stop':
    if rest is not None and rest < LR['rest'] and dx_close > w:
      ev.append(_ev('R5', span, tws, 0.0, f"rest {rest:.2f} m < 3.5 (OFF {row['rest_off']:.2f})", rest))
    if mg is not None and mg < LR['min_gap'] and dx_close > w:
      ev.append(_ev('R5', span, tws, 0.0, f'min gap {mg:.2f} m < 3.2 (OFF {mg + dx_close:.2f})', mg))
    if ws is not None and ws > LR['ws_gap'] and dx < -w:
      ev.append(_ev('R5', span, tws, 0.0, f"wheel-stop gap {ws:.2f} m > 6 (OFF {row['ws_off']:.2f})", ws))
  if st['kind'] == 'stop':
    if a_stop is not None and a_stop <= LR['a_stop'] and da < -LR['worse_a']:
      ev.append(_ev('R6', span, tws, 0.0, f"a_stop {a_stop:.2f} (OFF {row['a_off']:.2f})" + (' band-latched' if band else ''), a_stop))
    cap = float(rec[_i(tl, tws - 3.0):k_ws + 1].min())
    if (row['wire_stop_on'] <= LR['a_stop'] and cap < LR['capture'] and row['wire_stop_on'] < row['wire_stop_off'] - LR['worse_a']
            and not any(e['rule'] == 'R6' for e in ev)):
      ev.append(_ev('R6', span, tws, 0.0, f"stop wire {row['wire_stop_on']:.2f} after a capture {cap:.2f} (OFF {row['wire_stop_off']:.2f})"
                    + (' band-latched' if band else ''), row['wire_stop_on']))
  if row['acted']:
    k10 = np.flatnonzero((tl >= tws - 10.0) & (tl <= tws))
    if len(k10) > 1:
      crawl = (inp['v'][k10] >= LR['crawl_lo']) & (inp['v'][k10] <= LR['crawl_hi'])
      dtt = np.diff(tl[k10], prepend=tl[k10[0]])
      if float(np.sum(dtt[crawl])) > LR['crawl_s']:
        c0 = k10[np.flatnonzero(crawl)[0]]

        def deepen(x):
          return float(np.max(np.maximum.accumulate(x) - x)) if len(x) else 0.0
        g_on, g_off = deepen(rec[c0:k_ws]), deepen(rec[c0:k_ws] - dw[c0:k_ws])   # the hold build at the wheel stop excluded
        if g_on >= LR['crawl_deepen'] and g_on > g_off + LR['worse_a']:
          ev.append(_ev('R7', span, tws, 0.0, f'crawl {np.sum(dtt[crawl]):.1f} s at 0.02-0.2 m/s then a wire deepening {g_on:.2f} (OFF {g_off:.2f})', g_on))
    cc = R.get('cc')
    grade = None
    if cc is not None and len(cc):
      c0, c1 = rng(cc, tws - 2.0, tws)
      p = cc[c0:c1, 4]
      p = p[np.isfinite(p)]
      grade = round(float(100.0 * np.tan(np.mean(p))), 1) if len(p) else None
    row['grade'] = grade
    if grade is not None and grade <= LR['downhill'] and st['kind'] == 'stop':
      if rest is not None and rest < LR['downhill_rest'] and dx_close > w:
        ev.append(_ev('R8', span, tws, 0.0, f"downhill {grade} %: rest {rest:.2f} m < 3.8 (OFF {row['rest_off']:.2f})", rest))
      hk = np.flatnonzero((tl > tws) & (Ln['phase'] == 4))
      if len(hk) > 1:
        mv = float(np.sum(inp['v'][hk[1:]] * np.diff(tl[hk])))
        if mv > LR['hold_move'] and float(np.mean(dw[hk])) > LR['worse_a']:
          ev.append(_ev('R8', span, tws, 0.0, f'downhill {grade} %: {mv:.2f} m in HOLD with a hold {np.mean(dw[hk]):.2f} shallower than OFF', mv))
  # H5: gap at first motion (the first command to move: wire > 0 with StopReq clear) vs OFF, before the driver's gas
  if st['kind'] == 'stop':
    ka5 = tl > tws + 0.3
    gas = np.flatnonzero(ka5 & (inp['gas'] > 0))
    t_gas = float(tl[gas[0]]) if len(gas) else np.inf
    s = np.r_[0.0, np.cumsum(inp['v'][1:] * np.diff(tl))]
    gap_cf = inp['gap'] + (s - s[k_ws])   # the gap if the car had stayed at its rest position
    k_rest = _i(tl, tws + 0.3)

    def go(L):
      k = np.flatnonzero(ka5 & (L['wire'] > 0.0) & (L['stopreq'] == 0) & (tl < t_gas))
      return int(k[0]) if len(k) else None

    def false_go(k):
      return bool(inp['lead'][k] and inp['vl'][k] < LR['out_v'] and gap_cf[k] < gap_cf[k_rest] + LR['moved'])
    g_on, g_off = go(Ln), go(Lf)
    row.update(go_on=None if g_on is None else round(float(tl[g_on]), 2), go_off=None if g_off is None else round(float(tl[g_off]), 2),
               h5_dgap=None if g_on is None or g_off is None else round(float(gap_cf[g_on] - gap_cf[g_off]), 2))
    lead_gone = g_off is not None and (not inp['lead'][g_off] or gap_cf[g_off] > gap_cf[k_rest] + LR['moved'] or inp['vl'][g_off] >= LR['out_v'])
    if g_on is None and g_off is not None and lead_gone:
      ev.append(_ev('H5', span, tl[g_off], 0.0, f'no launch command where OFF launches at {tl[g_off]:.2f} (gap {gap_cf[g_off]:.2f} m)'
                    + (f'; the driver launched at {t_gas:.2f}' if np.isfinite(t_gas) else ''), gap_cf[g_off]))
    elif row['h5_dgap'] is not None and row['h5_dgap'] > LR['h5_gap']:
      ev.append(_ev('H5', span, tl[g_on], 0.0, f"gap at first motion {gap_cf[g_on]:.2f} m, {row['h5_dgap']:+.2f} m vs OFF", row['h5_dgap']))
    if g_on is not None and false_go(g_on) and not (g_off is not None and false_go(g_off) and abs(tl[g_off] - tl[g_on]) <= 0.3):
      ev.append(_ev('H5', span, tl[g_on], 0.0, f'false launch: move command toward a stopped lead (gap {gap_cf[g_on]:.2f} m)', gap_cf[g_on]))
  row['rules'] = sorted({e['rule'] for e in ev})
  return row, ev


def line_decide(events):
  """The trial's revert decision from all events of a drive: [(rule, evidence events)]."""
  by = defaultdict(list)
  for e in events:
    by[e['rule']].append(e)
  trips = []
  for r, (_, need) in LINE_RULES.items():
    e = by.get(r, [])
    if r == 'R4':
      big = [x for x in e if x['value'] >= LR['pump']]
      if len(big) >= need or any(x['value'] >= LR['pump_any'] for x in e):
        trips.append((r, big))
    elif need and len(e) >= need:
      trips.append((r, e))
  return trips


def trial_e3(flag, T, base, live, fv, spans, drive, log):
  """E3 (RELEASE_END_STOPPED_LEAD_REHOLD): flag on vs off on the logged plan, the re-hold checks of PLAN section 60."""
  on = drive if live else run_arm(base, {flag: True}, spans, log)
  off = run_arm(base, {flag: False}, spans, log)
  tr = dict(flag=flag, name=T['name'], in_build=fv, mode='live' if live else 'counterfactual', base=base[:10], events=[],
            checks=[], spans_failed=[])
  same_as_drive = True
  for s in spans:
    a, b, c = on.get(s['span']), off.get(s['span']), drive.get(s['span'])
    if not all(isinstance(x, dict) for x in (a, b, c)):
      tr['spans_failed'].append(s['span'])
      continue
    if not live:
      same_as_drive &= len(b['t']) == len(c['t']) and all(np.array_equal(b[k], c[k], equal_nan=True) for k in DR.COLS)
    for e in diff_events(a, b):
      e['span'] = s['span']
      tr['events'].append(e)
    tr['checks'] += [dict(c_, span=s['span']) for c_ in e3_checks(a, b, DR.inputs(DR.frames_for(s)), live)]
  if not live:
    tr['off_equals_drive'] = bool(same_as_drive) and not tr['spans_failed']
  tr['revert'] = [f"REVERT: {flag} = False" for c_ in tr['checks'] if c_['violations']][:1] if live else []
  tr['verdict'] = 'REVERT' if tr['revert'] else 'INCOMPLETE' if tr['spans_failed'] else 'PASS'   # unchecked spans are never a PASS
  return tr


def trial_line(flag, T, base, live, fv, spans, stops, drive, R, log):
  """SANTA_FE_STOP_LINE: planner lockstep flag on and off; the arm that drove (live: on, else: off) replays the logged plan, the
  other one the logged plan + its planner delta. Rules relative to the flag-off arm; per-stop line/band activity."""
  ref, oth = ((base, {}), (base, {flag: False})) if live else ((base, {flag: False}), (base, {flag: True}))
  ref_key = cache_key(*ref)
  p_ref = run_arm(*ref, spans, log, kind='planner')
  p_oth = run_arm(*oth, spans, log, kind='planner')
  l_ref = drive if live else run_arm(*ref, spans, log)
  l_oth = run_arm(*oth, spans, log, kind='delta', ref_key=ref_key)
  tr = dict(flag=flag, name=T['name'], in_build=fv, mode='live' if live else 'counterfactual', base=base[:10], ref_arm=ref_key,
            events=[], stops=[], spans_failed=[], plan_fidelity=[])
  for s in spans:
    pr, po, lr, lx = (x.get(s['span']) for x in (p_ref, p_oth, l_ref, l_oth))
    bad = [x for x in (pr, po, lr, lx) if not isinstance(x, dict)]
    if bad:
      tr['spans_failed'].append((s['span'], str(bad[0])))
      continue
    b = pick_bound(pr, s['lo'])
    Pr, Po = dict(pr[b]), dict(po[b])
    if not np.array_equal(Pr['ns'], Po['ns']):
      tr['spans_failed'].append((s['span'], 'planner ticks differ between the arms'))
      continue
    Pr['t'] = Po['t'] = (Pr['ns'] - pr['origin_ns']) * 1e-9
    tr['plan_fidelity'].append(dict(plan_fidelity(Pr, s['lo']), span=s['span'], bound_ms=b, delta_frames_matched=int(lx['matched']),
                                    frames=len(lx['t'])))
    Pn, Pf, Ln, Lf = (Pr, Po, lr, lx) if live else (Po, Pr, lx, lr)
    inp = DR.inputs(DR.frames_for(s))
    tr['events'] += line_span_events(Pn, Pf, Ln, Lf, inp, s['span'])
    for st in stops:
      if st.get('span') == s['span']:
        row, ev = line_stop(dict(st['m'], id=st['id'], span=s['span']), Pn, Pf, Ln, Lf, inp, R)
        tr['stops'].append(row)
        tr['events'] += ev
  tr['events'].sort(key=lambda e: (e['rule'], e['t']))
  trips = line_decide(tr['events'])
  tr['trips'] = [dict(rule=r, why=LINE_RULES[r][0], evidence=e[:10]) for r, e in trips]
  tr['revert'] = [f'REVERT: {flag} = False'] if live and trips else []
  tr['verdict'] = 'REVERT' if tr['revert'] else 'INCOMPLETE' if tr['spans_failed'] else 'PASS'   # unchecked spans are never a PASS
  return tr


# ==== report ===============================================================================================================
def attention(m, fid_ok):
  a = []
  if m.get('bookmark'):
    a.append('BOOKMARK')
  if m['rest'] is not None and (m['rest'] < ATTN['rest_lo'] or m['rest'] > ATTN['rest_hi']):
    a.append(f"rest {m['rest']}")
  if m['a_stop'] is not None and m['a_stop'] <= ATTN['a_stop']:
    a.append(f"a_stop {m['a_stop']}")
  if m['entry_bite'] is not None and m['entry_bite'] <= ATTN['bite']:
    a.append(f"bite {m['entry_bite']}")
  if m['takeover_approach'] or m['takeover_hold']:
    a.append('takeover')
  if m['stopreq_pairs']:
    a.append(f"StopReq chatter x{len(m['stopreq_pairs'])}")
  if m['hold_in_pid']:
    a.append('hold in pid')
  if m['starting_under_hold']:
    a.append("'starting' under hold")
  if not fid_ok:
    a.append('fidelity')
  return a


def fmt(x, nd=2):
  return '-' if x is None else (f'{x:.{nd}f}' if isinstance(x, float) else str(x))


def report_route(route, log=print):
  """Stage B for one scanned route. Returns the report dict (also written to REPORTS)."""
  t0 = time.monotonic()
  R = load_route(route)
  rep = dict(route=route, generated=dt.datetime.now().astimezone().isoformat(timespec='seconds'), segments=len(R['segments']),
             commits=R['commits'], dirty=R['dirty'], scan_errors=R['errors'], bookmarks=R['bookmarks'])
  engaged = len(R.get('cc', [])) and bool(np.any(R['cc'][:, 2] > 0))
  rep['engaged_s'] = round(float(np.sum(np.diff(R['cc'][:, 0])[R['cc'][1:, 2] > 0])), 1) if engaged else 0.0
  if not engaged:
    rep.update(stops=[], holds=[], bookmark_matches=match_bookmarks(R['bookmarks'], [], []), races=[], trials={},
               replay='no engaged driving', summary=f'{route}: no engaged driving ({len(R["segments"])} segments), ' +
                                                    f'bookmarks {len(R["bookmarks"])}')
    return write(rep)
  commit = R['commits'][0] if R['commits'] else None
  sha = DR.full_sha(commit) if commit else None
  rep['commit'] = commit[:10] if commit else None
  stops = engaged_stops(R)
  hlds = holds(R)
  for s in stops:
    s['id'] = f"{route[4:8]}_{s['t_ws']:.2f}"
    s['m'] = stop_metrics(R, s)
  bm = match_bookmarks(R['bookmarks'], stops, hlds)
  for b in bm:
    for s in stops:
      if b.get('stop') == s['id']:
        s['m'].setdefault('bookmark', []).append(b['t'])
  # driver-rescued approaches and pedal takeovers: trial replay windows only (the stop table keeps the census rule)
  rescued, tko = engaged_stops(R, interrupted=True), takeovers(R)
  spans = replay_spans(route, R, stops + rescued, hlds, tko)
  rep.update(spans=[dict(s) for s in spans], bookmark_matches=bm, holds=hlds, interrupted=[dict(kind=s['kind'], t_ws=s['t_ws']) for s in rescued],
             takeovers=tko)
  rep['races'] = release_end_races(R)
  # replays: the drive's own code (fidelity) + per trial flag on/off
  arms, fid, trials = {}, None, {}
  if len(R['commits']) > 1:
    rep['note_commits'] = 'several commits in one route: the replay uses the first'
  for s in stops:
    s['span'] = next((x['span'] for x in spans if x['lo'] <= s['t_ws'] <= x['hi']), None)
  if sha is None:
    rep['replay'] = f'skipped: commit {commit} not in the local repo'
  elif not spans:
    rep['replay'] = 'no stop or hold span'
  else:
    arms['drive'] = run_arm(sha, {}, spans, log)
    fid = fidelity(R, spans, arms['drive'])
    rep['fidelity'] = fid
    # differing python files outside drive_replay.MAPPED load from the working tree (tooling_check2 fix 8): named per replayed commit
    rep['unpinned'] = {sha[:10]: DR.snapshot(sha, {})[2]}
    for flag, T in TRIALS.items():
      fv = DR.flag_value(sha, flag)
      live = fv is True
      base = sha if live else DR.full_sha(T['ref'])
      if base is None:
        trials[flag] = dict(flag=flag, name=T['name'], in_build=fv, mode='skipped', note=f"reference {T['ref']} not in the local repo")
        continue
      if base[:10] not in rep['unpinned']:
        rep['unpinned'][base[:10]] = DR.snapshot(base, {})[2]
      if flag == 'SANTA_FE_STOP_LINE':
        trials[flag] = trial_line(flag, T, base, live, fv, spans, stops, arms['drive'], R, log)
      else:
        trials[flag] = trial_e3(flag, T, base, live, fv, spans, arms['drive'], log)
  rep['trials'] = trials
  # INCOMPLETE: a rule set that could not be checked (a failed span replay, a skipped trial, no replay of the drive's commit): never a
  # PASS; the route is not marked reported, so the next run retries (failed spans have no cache file)
  rep['incomplete'] = ([rep['replay']] if str(rep.get('replay', '')).startswith('skipped') else []) + \
    [f"{tr['name']}: {tr['note']}" if tr['mode'] == 'skipped' else f"{tr['name']}: {len(tr['spans_failed'])} spans without a replay"
     for tr in trials.values() if tr['mode'] == 'skipped' or tr['spans_failed']]
  fid_ok = fid is None or fid['ok'] is not False   # no replay / no span is not a drift
  for s in stops:
    s['m']['attention'] = attention(s['m'], fid_ok)
  rep['stops'] = [dict(id=s['id'], **s['m']) for s in stops]
  n_attn = sum(bool(s['attention']) for s in rep['stops'])
  tparts = []
  for tr in trials.values():
    if tr['mode'] == 'skipped':
      tparts.append(f"{tr['name']}: {tr['note']}")
    elif tr['name'] == 'LINE':
      acted = sum(r['acted'] for r in tr['stops'])
      tripped = ','.join(t_['rule'] for t_ in tr['trips']) or 'none'
      tparts.append(f"LINE live: {acted}/{len(tr['stops'])} stops line/band, " + (tr['revert'][0] if tr['revert'] else f"rules {tr['verdict']}")
                    if tr['mode'] == 'live' else
                    f"no LINE drive yet (counterfactual: {acted}/{len(tr['stops'])} stops line/band, rules that would trip: {tripped}"
                    + ('; INCOMPLETE' if tr['spans_failed'] else '') + ')')
    elif tr['mode'] == 'live':
      tparts.append(f"{tr['name']} live: {len(tr['checks'])} re-holds, " + ('REVERT' if tr['revert'] else f"rules {tr['verdict']}"))
    else:
      tparts.append(f"no {tr['name']} drive yet (counterfactual: {len(tr['checks'])} firings" + ('; INCOMPLETE' if tr['spans_failed'] else '') + ')')
  races = sum(r['race'] for r in rep['races'])
  rep['summary'] = (('INCOMPLETE ' if rep['incomplete'] else '') + f"{route} @{rep['commit']}: {rep['engaged_s'] / 60:.0f} min engaged, " +
                    f"{len(stops)} engaged stops, {len(hlds)} holds, {len(rescued)} interrupted approaches, {len(tko)} takeovers, {n_attn} ATTENTION, " +
                    f"bookmarks {len(R['bookmarks'])}, release-end races {races}, " +
                    f"fidelity {'n/a' if not fid or fid['ok'] is None else 'OK' if fid['ok'] else 'DRIFT'}; " + '; '.join(tparts))
  rep['runtime_s'] = round(time.monotonic() - t0, 1)
  return write(rep)


def write(rep):
  REPORTS.mkdir(parents=True, exist_ok=True)
  (REPORTS / f"{rep['route']}.json").write_text(json.dumps(rep, indent=1, default=lambda o: o.item() if hasattr(o, 'item') else str(o)))
  (REPORTS / f"{rep['route']}.md").write_text(markdown(rep))
  idx = REPORTS / 'INDEX.md'
  lines = [x for x in (idx.read_text().splitlines() if idx.exists() else []) if x and not x.startswith(f"- {rep['route']}")]
  lines = [x for x in lines if not x.startswith('#')]
  idx.write_text('# Drive reports (tools/stopping/drive_report.py)\n\n' + '\n'.join(sorted(lines + [f"- {rep['summary']}"])) + '\n')
  return rep


def markdown_e3(rep, tr):
  L = []
  if tr['mode'] == 'live':
    L.append("- LIVE in this build. Revert rules (PLAN section 60; the grab rule relative to the flag-off replay): " +
             (tr['revert'][0] if tr['revert'] else tr['verdict']))
  else:
    L.append(f"- no {tr['name']} drive yet: build {rep.get('commit')} " + ('has no ' + tr['flag'] + ' flag.' if tr['in_build'] is None
                                                                          else f"has {tr['flag']} = False."))
    L.append(f"- counterfactual: {tr['base']} with the flag on vs off on this drive's logged inputs (open loop; motion and grab " +
             f"rules need a live drive). Flag-off arm == this build's replay: {tr.get('off_equals_drive')}.")
  if tr['spans_failed']:
    L.append(f"- INCOMPLETE: spans without a replay (rules not checked; retried on the next run): {tr['spans_failed']}")
  L += ['', '| re-hold | gap | vLead | brake | StopReq clear on/off | later | violations | notes |', '|---|---|---|---|---|---|---|---|']
  for c in tr['checks']:
    L.append(f"| {c['t']:.2f} | {c['gap']} | {c['vlead']} | {c['brake']} | {fmt(c['stopreq_clear_on'])}/{fmt(c['stopreq_clear_off'])} | " +
             f"{fmt(c['later_s'])} | {'; '.join(c['violations']) or 'none'} | {'; '.join(c['notes'])} |")
  if not tr['checks']:
    L.append('| none | | | | | | | |')
  L += ['', f"Events (frames where command, phase, StopReq or LongControl state differ, flag on vs off): {len(tr['events'])}", '']
  for e in tr['events'][:40]:
    L.append(f"- {e['t0']:.2f}-{e['t1']:.2f} ({e['frames']} frames) dwire {e['min_dwire']:+.3f}..{e['max_dwire']:+.3f} " +
             f"phase on {e['phase_on']} / off {e['phase_off']}, lcs on {e['lcs_on']} / off {e['lcs_off']}, StopReq on " +
             f"{e['stopreq_on']} / off {e['stopreq_off']}" + (' [re-hold]' if e['rehold'] else ''))
  return L


def markdown_line(rep, tr):
  L = []
  if tr['mode'] == 'live':
    L.append(f"- LIVE in this build (flag-off reference: {tr['base']} with {tr['flag']} = False; the drive replays the logged plan, " +
             "the flag-off arm the logged plan + its planner delta).")
    L.append('- Revert rules (relative to the flag-off replay of this drive): ' + (f"**{tr['revert'][0]}**" if tr['revert'] else tr['verdict']))
  else:
    L.append(f"- no LINE drive yet: build {rep.get('commit')} " + ('has no ' + tr['flag'] + ' flag.' if tr['in_build'] is None
                                                                  else f"has {tr['flag']} = False."))
    L.append(f"- counterfactual: {tr['base']} flag on vs off as if the trial were live (the flag-off arm replays the logged plan, " +
             "the flag-on arm the logged plan + its planner delta). Rules that would trip: " +
             (', '.join(t['rule'] for t in tr['trips']) or 'none') + ('; INCOMPLETE (spans without a replay)' if tr['spans_failed'] else ''))
  for t in tr['trips']:
    L.append(f"  - {t['rule']} ({t['why']}):")
    L += [f"    - {e['span']} {e['t']:.2f} v {fmt(e['v'])}: {e['what']}" for e in t['evidence']]
  pf = tr['plan_fidelity']
  if pf:
    n = sum(x['ticks'] for x in pf)
    bad = sum(x['over_005'] for x in pf)
    L.append(f"- planner fidelity (reference arm vs the logged aTarget, engaged ticks in the spans): {bad}/{n} ticks > 0.05 " +
             f"(worst {max((x['max'] or 0.0) for x in pf):.3f}; bound ms per span {[x['bound_ms'] for x in pf]}); delta frames matched " +
             f"{sum(x['delta_frames_matched'] for x in pf)}/{sum(x['frames'] for x in pf)}")
  if tr['spans_failed']:
    L.append(f"- INCOMPLETE: spans without a replay (rules not checked; retried on the next run): {tr['spans_failed']}")
  who = ' (the trial drove)' if tr['mode'] == 'live' else ' (as if live: this drive stands in for the flag-on world)'
  L += ['', 'Per stop (for the ratings). Line, band, extra brake, entry bite and launch: replay arms, flag on / flag off. Rest, ' +
        f'wheel-stop gap, a_stop and stop wire: on = LOGGED{who}, off = logged minus the trial\'s command effect (open loop: an ' +
        'upper bound, gaps clipped at 0; only its sign and the 0.2 m / 0.05 thresholds enter the rules).', '',
        '| stop | line (t, v, gap, armed s) | extra brake plan/wire | band | entry bite on/off | rest on/off | wheel-stop gap on/off ' +
        '| a_stop on/off | wire@stop on/off | go on/off (dgap) | rules | rating |', '|---|---|---|---|---|---|---|---|---|---|---|---|']
  for r in tr['stops']:
    ln = f"{r['line_t']:.2f}, {r['line_v']}, {r['line_gap']}, {r['line_s']}" if r['line'] and r['line_t'] is not None else \
      ('yes' if r['line'] else '-')
    go = f"{fmt(r.get('go_on'))}/{fmt(r.get('go_off'))} ({fmt(r.get('h5_dgap'))})" if r['kind'] == 'stop' else 'rolling'
    L.append(f"| {r['id']} | {ln} | {r['plan_extra']:.2f}/{r['wire_deeper']:.2f} | {'yes' if r['band'] else '-'} | " +
             f"{fmt(r['bite_on'])}/{fmt(r['bite_off'])} | {fmt(r['rest_on'])}/{fmt(r['rest_off'])} | {fmt(r['ws_on'])}/{fmt(r['ws_off'])} | " +
             f"{fmt(r['a_on'])}/{fmt(r['a_off'])} | {fmt(r['wire_stop_on'])}/{fmt(r['wire_stop_off'])} | {go} | " +
             f"{', '.join(r['rules']) or '-'} | |")
  if not tr['stops']:
    L.append('| none | | | | | | | | | | | |')
  L += ['', f"Rule events (all, incl. below the revert counts): {len(tr['events'])}", '']
  L += [f"- {e['rule']} {e['span']} {e['t']:.2f} v {fmt(e['v'])}: {e['what']}" for e in tr['events'][:60]]
  return L


def markdown(rep):
  L = [f"# Drive {rep['route']}", '', rep['summary'], '',
       f"- commit {rep.get('commit')} (dirty {rep['dirty']}), segments {rep['segments']}, engaged {rep['engaged_s']} s, " +
       f"report {rep['generated']}"]
  if rep.get('incomplete'):
    L.append(f"- **INCOMPLETE** (rules not checked; the next run retries): {rep['incomplete']}")
  if rep.get('scan_errors'):
    L.append(f"- scan errors: {rep['scan_errors'][:5]}")
  fid = rep.get('fidelity')
  if fid:
    L.append(f"- fidelity (replay of the drive's commit vs the log, {fid['frames']} frames): command mean abs error {fid['mean_abs']}, "
             + f"{fid['over_005']} frames > 0.05 (worst {fid['worst'][0]:.3f} at {fid['worst'][1]}), StopReq toggles replay/log "
             + f"{fid['stopreq_toggles']} unmatched {fid['stopreq_unmatched']}, logged phase changes {fid['phase_changes']} unmatched "
             + f"{fid['phase_unmatched']}, span errors {len(fid['errors'])} -> " +
             {True: 'OK', False: 'SIM OUT OF SYNC WITH CAR', None: 'n/a (no stop or hold span)'}[fid['ok']])
  elif rep.get('replay'):
    L.append(f"- replay {rep['replay']}")
  for c, files in (rep.get('unpinned') or {}).items():
    if files:
      L.append(f"- replay of {c}: {len(files)} differing python files outside the pinned directories load from the working tree: "
               + ', '.join(files[:10]) + (' ...' if len(files) > 10 else ''))
  # every section renders on every drive (tooling_check fix 7: a drive with 0 census stops still has bookmarks, holds, races and
  # trial evidence)
  L += ['', '## Stops (ATTENTION first)', '']
  if rep.get('stops'):
    L += ['| stop | kind | rest | rest_med | min gap | entry bite | pumps wire/aEgo | a_stop | j300 | felt | wire@stop | launch | attention |',
          '|---|---|---|---|---|---|---|---|---|---|---|---|---|']
    for s in sorted(rep['stops'], key=lambda s: (not s['attention'], s['t_ws'])):
      L.append(f"| {s['id']} | {s['kind']} | {fmt(s['rest'])} | {fmt(s['rest_med'])} | {fmt(s['min_gap'])} | {fmt(s['entry_bite'], 3)} | " +
               f"{fmt(s.get('pumps_wire'))}/{fmt(s.get('pumps_aego'))} | {fmt(s['a_stop'])} | {fmt(s['j300'])} | {fmt(s['felt'])} | " +
               f"{fmt(s['wire_at_stop'])} | {fmt(s['launch_by'])} | {', '.join(s['attention'])} |")
  else:
    L.append(f"- no engaged census stop; holds (engaged standstill >= 1 s): {len(rep.get('holds') or [])}")
  L += ['', '## Interrupted approaches and takeovers (trial replay windows; not in the stop table)', '']
  L += [f"- interrupted {x['kind']} at {x['t_ws']:.2f}" for x in rep.get('interrupted') or []]
  L += [f"- pedal takeover at {t:.2f}" for t in rep.get('takeovers') or []] or (['- none'] if not rep.get('interrupted') else [])
  L += ['', '## Bookmarks', '']
  L += [f"- {b['t']:.2f} -> {b.get('stop') or ('hold ' + str(b.get('hold')) if b.get('hold') else 'unmatched')} ({b['how']})"
        for b in rep.get('bookmark_matches') or []] or ['- none']
  L += ['', '## Release-end races (logged; the class E3 removes)', '']
  races = rep.get('races') or []
  L += [f"- {r['t']:.2f} gap {r['gap']} m: travel {r['travel_before_lead']} m before the lead departed ({r['t_lead_departs']}), " +
        f"starting {r['starting']}, StopReq clear {r['stopreq_clear']}, re-entry {r['reentry']}, gas {r['driver_gas']}"
        + (' **RACE**' if r['race'] else '') for r in races] or ['- none']
  for flag, T in TRIALS.items():
    tr = (rep.get('trials') or {}).get(flag)
    L += ['', f"## Trial {T['name']} ({flag})", '']
    if tr is None:
      L.append(f"- no replay ({rep.get('replay') or 'n/a'})")
    elif tr['mode'] == 'skipped':
      L.append(f"- {tr['note']}")
    elif tr['name'] == 'LINE':
      L += markdown_line(rep, tr)
    else:
      L += markdown_e3(rep, tr)
  L += ['', '## Stop details', '']
  for s in rep['stops']:
    L.append(f"- {s['id']}: entry {fmt(s['t_entry'])} {s['entry_phase'] or ''} v {fmt(s['v_entry'])}, StopReq pairs {s['stopreq_pairs']}, " +
             f"hold in pid {s['hold_in_pid']}, starting under hold {s['starting_under_hold']}, takeover approach/hold " +
             f"{s['takeover_approach']}/{s['takeover_hold']}, phases {s['phases'][:8]}")
  return '\n'.join(L) + '\n'


# ==== CLI ==================================================================================================================
def local_routes():
  return sorted({os.path.basename(os.path.dirname(p)).rsplit('--', 1)[0] for p in glob.glob(str(DR.RD / '0000*--*--*' / 'rlog.zst'))})


def main(argv=None):
  ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
  ap.add_argument('--routes', help='comma-separated route names or prefixes (e.g. 00002232,00002235); default: new routes')
  ap.add_argument('--rebuild', action='store_true', help='report again even if the state has the route')
  ap.add_argument('--hook', action='store_true', help='print the post-sync hook line and exit')
  args = ap.parse_args(argv)
  if args.hook:
    print('.venv/bin/python tools/route_sync/refresh_routes.py --include-rlog && .venv/bin/python tools/stopping/drive_report.py')
    return 0
  state_p = WORK / 'state.json'
  state = json.loads(state_p.read_text()) if state_p.exists() else {}
  have = local_routes()
  if args.routes:
    keys = [k.strip() for k in args.routes.split(',') if k.strip()]
    routes = [r for r in have if any(r.startswith(k) for k in keys)]
    if not routes:
      print(f'no local route matches --routes {args.routes} ({len(have)} local routes with rlogs in {DR.RD})')
      return 2
  elif state:
    floor = min(state)
    routes = [r for r in have if r >= floor and (r not in state or state[r]['rlogs'] < len(route_segments(r)))]
  else:
    print('empty state: name the first routes with --routes (later runs report every newer route)')
    return 2
  if not args.rebuild and args.routes:
    routes = [r for r in routes if r not in state or state[r]['rlogs'] < len(route_segments(r))]
  if not routes:
    print('no new routes' + (' (every named route is reported; --rebuild reports it again)' if args.routes else ''))
    return 0
  t0 = time.monotonic()
  segs = [s for r in routes for s in route_segments(r) if not (WORK / 'scan' / f'{s}.npz').exists()]
  if segs:
    with get_context('spawn').Pool(PROCESSES) as pool:
      for i, (seg, msg) in enumerate(pool.imap_unordered(scan_segment, sorted(segs), chunksize=1)):
        if 'err=None' not in msg:
          print(f'  scan {seg}: {msg}', flush=True)
        if (i + 1) % 50 == 0:
          print(f'  scanned {i + 1}/{len(segs)} {time.monotonic() - t0:.0f} s', flush=True)
    print(f'stage A: {len(segs)} segments in {time.monotonic() - t0:.0f} s', flush=True)
  incomplete = []
  for r in routes:
    rep = report_route(r, log=lambda x: print(x, flush=True))
    print(rep['summary'], flush=True)
    for tr in (rep.get('trials') or {}).values():
      for line in tr.get('revert') or []:
        print(f"  {line}  ({tr['name']} evidence in {REPORTS / (r + '.md')})", flush=True)
        for tp in tr.get('trips') or []:
          print(f"    {tp['rule']}: " + '; '.join(f"{e['t']:.2f} {e['what']}" for e in tp['evidence'][:3]), flush=True)
    if rep.get('incomplete'):   # not marked reported: the next run retries the failed spans
      incomplete.append(r)
      continue
    # a route is reported again only when its rlog count grows (a pruned route keeps its fuller report)
    state[r] = dict(rlogs=max(len(route_segments(r)), state.get(r, {}).get('rlogs', 0)), reported=rep['generated'])
    state_p.parent.mkdir(parents=True, exist_ok=True)
    state_p.write_text(json.dumps(state, indent=1, sort_keys=True))
  print(f'done in {time.monotonic() - t0:.0f} s; reports in {REPORTS}' + (f'; INCOMPLETE (retried next run): {incomplete}' if incomplete else ''))
  return 3 if incomplete else 0


if __name__ == '__main__':
  sys.exit(main())
