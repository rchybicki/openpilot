#!/usr/bin/env python3
"""Brake-response test drives (plans KCS1, KCS2, ...) -> one measurement record per rep, for plant identification.

Usage (repository root, .venv active):
  python tools/stopping/review/kcs1_reps.py <rlog paths...> --output NEW_DIR

Rules: ~/.route_sync/corpus/test_program_plan_20260926/PLAN.md sections 2, 5 and 8, and
docs/stopping/brake_response_session_2026-09-26.md B6/B7. Maneuvers are looked up in every block of
identification_hook.BLOCKS (ids are unique across blocks); a rep's plan is the route's logged progress-record plan when
that block has the maneuver, else the block that has it. Reps are segmented from the alertDebug banner (sent on every
controlsd frame of a rep) and labelled from the sent SCC12 frames, with the device's cloudlog verdict kept beside it.
Every time comes from the sent SCC12 frames (sendcan bus 0), the CAN receive batches, carState/controlsState/carControl
and the IMU sensor timestamps, never from cloudlog. Output (the directory must not exist): reps.jsonl (one record per
rep; plain numbers, non-finite as null; times in seconds from t0 = send time of the first scripted SCC12 frame),
summary.md, and series/<rep>.npz (every signal of the rep window at its native rate as <stream>__<field>, times from t0;
lcs__state: off 0, pid 1, stopping 2, starting 3). Inputs are only read.

Stop time: the last WHL_PUL11 count increase (any wheel) before the standstill flag. The flag (ABS speed <= 0.104 m/s)
comes ~0.22 s later because the ABS speed decays after the pulses stop, so gains, speed bins and terminal features end
at the pulse stop; the hold, StopReq and escape checks stay on the flag (the device's own standstill).

Limits: CAN times are receive-batch times (0-10 ms after the bus frame); the sendcan time is the publish time before
the Panda transmits (the bus-128 echo time is kept per edge). ESP12 LONG_ACCEL is taken as a forward specific-force
sensor for the ESP12 grade (ESP12 minus the wheel slope; sign not verified against a known slope); the gain-use gate is
on it, and the carControl.orientationNED pitch grade (-2.45 % offset on KCS1 drive 1) is metadata only. WHL_PUL11
metres per count are not documented: they are scaled per rep from the pre-window wheel-speed integral. The IMU uses the
last liveCalibration before t0 and a per-rep constant offset (pre-window forward IMU mean minus the pre-window wheel
slope: grade and sensor bias together), not orientationNED; livePose is not used.
"""
import argparse
import ast
import hashlib
import json
import math
import re
from collections import Counter, defaultdict
from pathlib import Path

import numpy as np
from opendbc.can import CANParser

from openpilot.common.transformations.orientation import rot_from_euler
from openpilot.selfdrive.controls.lib.identification_hook import A_HOLD, BLOCKS, HOLD_MIN_S, INTENT_V, J_HOLD, STALL_V
from openpilot.tools.lib.logreader import LogReader
from openpilot.tools.stopping.review.can_response import FIELDS as RESPONSE_FIELDS

G = 9.81
KPH = 1 / 3.6
PRE_S = 2.0                                  # pre-window before t0 (PLAN section 2)
V_STEADY, A_STEADY, WIRE_STEADY = 0.3, 0.2, 0.15
SCRIPT_TOL = 0.005                           # sent aReqValue vs the 0.01-quantised script
HOLD_TOL = 0.02                              # hold build: 0.6 m/s^3 ramp, +-2 frames of phase and quantisation
GAP_MAX = 0.1
PANDA_BLOCK_S = 0.03                         # the Panda blocks (no echo) the frame sent 7-12 ms before carState's brake
EDGE_SEARCH_S = 0.3                          # a segment's first wire change is searched this long after its banner frame
EDGE_V_TOL = 0.1                             # a speed-ended segment switches at v_end +-0.1 m/s
SLOPE_S = 0.3                                # onset statistic window (wheel slope, IMU mean)
ONSET_N, ONSET_K = 3, 3.0
SD_FLOOR = 0.01                              # m/s^2: the 0.0022 m/s wheel-mean step over 0.3 s; also for the IMU
PRE_EDGE_S = 1.0                             # the pre-edge level: last 1.0 s before the edge (inside the previous segment)
MAX_DELAY_S = 2.0
HELD_S, GAIN_S = 2.0, 1.0                    # gain = mean wheel slope over the last 1.0 s of a segment held >= 2 s
SEND_S = 0.02                                # one SCC12 send period: wire edges are quantised to it
BIN_V, SETTLE_S = 0.1, 1.0                   # speed bins use slope windows starting >= 1.0 s after the edge
TERMINAL_V, TERMINAL_AFTER_S = 0.5, 0.5      # terminal window: last 0.5 m/s to 0.5 s after the standstill flag (the rebound)
IMU_MEAN_S = 0.1                             # IMU minimum on a 0.1 s trailing mean (raw 100 Hz is vibration)
DISP_S = 2.0
LCS_LAG_S = 0.035                            # stopping from the frame after the intent (+ publish jitter)
GRADE_MAX_PCT = 2.0
BANNER_GAP_S = 0.5
PAD_S = (3.0, 1.0)                           # series window: t0 - 3 s .. rep end + 1 s
PULSE_SCALE = 0.5                            # DBC factor of WHL_PUL_*: the raw 8-bit counter wraps at 256
WHEELS = ('FL', 'FR', 'RL', 'RR')
LCS = {'off': 0, 'pid': 1, 'stopping': 2, 'starting': 3}
MAN = {m[0]: m for block in BLOCKS.values() for m in block}
PLAN_OF = {m[0]: plan for plan, block in BLOCKS.items() for m in block}
MOVING = ('ACTIVE', 'HELD', 'HANDBACK')

FIELDS = {**RESPONSE_FIELDS,
          902: tuple(f'WHL_SPD_{w}' for w in WHEELS),
          903: tuple(f'WHL_PUL_{w}' for w in WHEELS),
          916: RESPONSE_FIELDS[916] + ('aBasis', 'BrakeLight', 'PBRAKE_ACT'),
          1287: ('AVH_LAMP',)}
CAN = {'scc12': ('sendcan', 0, 1057), 'scc14': ('sendcan', 0, 905), 'scc12_echo': ('can', 128, 1057), 'esp12': ('can', 0, 544),
       'tcs13': ('can', 0, 916), 'whl': ('can', 0, 902), 'pul': ('can', 0, 903), 'tcs15': ('can', 0, 1287)}
COLS = {'car': ('v', 'a', 'v_cruise', 'standstill', 'brake', 'gas', 'esp', 'acc_fault', 'valid'),
        'lcs': ('state',), 'cc': ('enabled', 'long_active', 'accel', 'pitch', 'override'),
        'imu': ('x', 'y', 'z'), 'gyro': ('x', 'y', 'z'), 'gps': ('bearing', 'speed', 'fix', 'bearing_acc'),
        'calib': ('roll', 'pitch', 'yaw', 'calibrated'),
        **{name: ('dat',) + FIELDS[addr] for name, (_, _, addr) in CAN.items()}}

BANNER = (
  ('active', re.compile(r'TEST (?P<man>[A-Z]) (?P<rep>\d+)/\d+ s(?P<seg>\d+) (?P<cmd>[+-]\d+\.\d+)(?: - (?P<v>-?[\d.]+) m/s)?')),
  ('stopped', re.compile(r'TEST (?P<man>[A-Z]) (?P<rep>\d+)/\d+ STOPPED - hold (?P<hold>[\d.]+) s(?: - BRAKE NOW)?')),
  ('aborted', re.compile(r'TEST (?P<man>[A-Z]) (?P<rep>\d+)/\d+ ABORTED - (?P<reason>.+)')),
  ('done', re.compile(r'(?P<man>[A-Z]) (?P<rep>\d+)/\d+ DONE')),
  ('not_counted', re.compile(r'(?P<man>[A-Z]) (?P<rep>\d+)/\d+ NOT COUNTED - (?P<reason>.+)')),
  ('locked_held', re.compile(r'TEST LOCKED - (?P<reason>.+) - HELD')),
)
HOOK_RX = re.compile(r'identification hook (?P<state>[A-Z]+) man=(?P<man>\S*) rep=(?P<rep>\d+) seg=(?P<seg>\d+) reason=(?P<reason>\S*) ' +
                     r'floor=(?P<floor>\S+) intent=(?P<intent>[01]) done=(?P<done>\S*) v=(?P<v>\S+)')
PROGRESS_RX = re.compile(r'identification hook progress (?P<what>loaded|saved): (?P<record>\{.*\})')


# ---- reading -------------------------------------------------------------------------------------------------------
def route_of(path):
  name = Path(path).parent.name
  route, _, seg = name.rpartition('--')
  return (route, int(seg)) if route and seg.isdigit() else (name, 0)


def derive(streams):
  """Wheel speeds in m/s and their mean; WHL_PUL11 as a cumulative mean raw count (wrap-corrected)."""
  w = streams['whl']
  speeds = np.stack([w[f'WHL_SPD_{x}'] for x in WHEELS]) * KPH
  w.update(dict(zip(WHEELS, speeds, strict=True)), mean=speeds.mean(0) if len(w['t']) else np.zeros(0))
  p = streams['pul']
  raw = np.stack([np.round(p[f'WHL_PUL_{x}'] / PULSE_SCALE) for x in WHEELS])
  p['count'] = np.concatenate(([0.0], np.cumsum((np.diff(raw, axis=1) % 256).mean(0)))) if len(p['t']) else np.zeros(0)
  return streams


def read_route(paths):
  """One pass over one route's rlogs in segment order: every stream the rep analysis needs (times in seconds)."""
  paths = sorted((Path(p) for p in paths), key=lambda p: route_of(p)[1])
  wanted = defaultdict(dict)
  for name, (svc, bus, addr) in CAN.items():
    wanted[(svc, bus)][addr] = name
  parsers = {key: CANParser('hyundai_kia_generic', [(a, float('nan')) for a in addrs], key[1]) for key, addrs in wanted.items()}
  rows = defaultdict(list)
  meta = {'route': route_of(paths[0])[0], 'files': [], 'init': [], 'banner': [], 'hook_lines': []}
  seen = set()
  for path in paths:
    raw = path.read_bytes()
    meta['files'].append({'path': str(path), 'bytes': len(raw), 'sha256': hashlib.sha256(raw).hexdigest()})
    for ev in LogReader(str(path)):
      w, ns = ev.which(), int(ev.logMonoTime)
      if w in ('can', 'sendcan'):
        for f in getattr(ev, w):
          name = wanted.get((w, f.src), {}).get(f.address)
          if name is None:
            continue
          cp, dat = parsers[(w, f.src)], bytes(f.dat)
          # as can_response.py: CANParser pads a short payload with zeros, so a truncated frame is dropped; a longer one
          # keeps every DBC bit (WHL_PUL11 is 8 bytes on this car, 6 in the DBC)
          if len(dat) < cp.message_states[f.address].size or f.address not in cp.update([[ns, [(f.address, dat, f.src)]]]):
            continue
          rows[name].append((ns, dat if name.startswith('scc12') else None, *(cp.vl[f.address][k] for k in FIELDS[f.address])))
      elif w == 'carState':
        c = ev.carState
        rows['car'].append((ns, c.vEgo, c.aEgo, c.vCruise * KPH, c.standstill, c.brakePressed, c.gasPressed, c.espActive,
                            c.accFaulted, ev.valid and c.canValid))
      elif w == 'controlsState':
        rows['lcs'].append((ns, LCS.get(str(ev.controlsState.longControlState), -1)))
      elif w == 'carControl':
        c = ev.carControl
        o = list(c.orientationNED)
        rows['cc'].append((ns, c.enabled, c.longActive, c.actuators.accel, o[1] if len(o) == 3 else math.nan, c.cruiseControl.override))
      elif w == 'accelerometer' and ev.accelerometer.which() == 'acceleration':
        rows['imu'].append((ev.accelerometer.timestamp, *list(ev.accelerometer.acceleration.v)[:3]))
      elif w == 'gyroscope' and ev.gyroscope.which() == 'gyroUncalibrated':
        rows['gyro'].append((ev.gyroscope.timestamp, *list(ev.gyroscope.gyroUncalibrated.v)[:3]))
      elif w == 'gpsLocationExternal':
        g = ev.gpsLocationExternal
        rows['gps'].append((ns, g.bearingDeg, g.speed, g.hasFix, g.bearingAccuracyDeg))
      elif w == 'liveCalibration' and len(ev.liveCalibration.rpyCalib) == 3:
        rows['calib'].append((ns, *list(ev.liveCalibration.rpyCalib), str(ev.liveCalibration.calStatus) == 'calibrated'))
      elif w == 'alertDebug':
        meta['banner'].append((ns * 1e-9, ev.alertDebug.alertText1, ev.alertDebug.alertText2))
      elif w in ('logMessage', 'errorLogMessage'):
        text = getattr(ev, w)
        if 'identification hook' in text and text not in seen:   # errors are forwarded on both services
          seen.add(text)
          try:
            msg = json.loads(text).get('msg', text)
          except (ValueError, AttributeError):
            msg = text
          meta['hook_lines'].append((ns * 1e-9, str(msg)))
      elif w == 'initData':
        i = ev.initData
        meta['init'].append({'segment': path.parent.name, 'commit': i.gitCommit, 'branch': i.gitBranch, 'dirty': bool(i.dirty)})
  streams = {}
  for name, cols in COLS.items():
    rs = sorted(rows[name], key=lambda r: r[0]) if name in ('imu', 'gyro') else rows[name]   # sensor time is the clock
    s = {'t': np.array([r[0] for r in rs], dtype=np.int64) * 1e-9}
    for k, col in enumerate(cols):
      s[col] = np.array([r[k + 1] for r in rs], dtype=object if col == 'dat' else float)
    streams[name] = s
  return derive(streams), meta


# ---- banner and cloudlog -------------------------------------------------------------------------------------------
def parse_banner(text1):
  for kind, rx in BANNER:
    m = rx.fullmatch(text1)
    if m:
      return kind, m.groupdict()
  return 'other', {}


def segment_reps(banner):
  """Banner rows (t, text1, text2) -> reps. A rep opens on an ACTIVE frame while none is open and keeps every frame
  tagged with its maneuver/rep (and LOCKED-HELD frames). It closes after the first DONE/NOT COUNTED frame (the brake
  frame) or driver-abort frame, on any other banner, or on a banner gap > BANNER_GAP_S."""
  reps, cur, last_t = [], None, -math.inf
  for t, text1, text2 in banner:
    kind, g = parse_banner(text1)
    if cur is not None:
      same = kind == 'locked_held' or (kind != 'other' and g['man'] == cur['man'] and int(g['rep']) == cur['rep'])
      if t - last_t > BANNER_GAP_S or not same:
        cur['close'] = 'gap' if t - last_t > BANNER_GAP_S else 'banner'
        reps.append(cur)
        cur = None
    if cur is None and kind == 'active':
      cur = {'man': g['man'], 'rep': int(g['rep']), 'frames': [], 'close': 'log_end'}
    if cur is not None:
      cur['frames'].append((t, kind, g, text2))
      if kind in ('done', 'not_counted') or (kind == 'aborted' and text2.startswith('test mode off')):
        cur['close'] = 'driver' if kind == 'aborted' else kind
        reps.append(cur)
        cur = None
    last_t = t
  if cur is not None:
    reps.append(cur)
  return reps


def banner_facts(rep):
  f = rep['frames']
  seg_t = {}
  for t, kind, g, _ in f:
    if kind == 'active':
      seg_t.setdefault(int(g['seg']), t)
  aborted = next(({'t': t, 'reason': g['reason'], 'text2': t2} for t, kind, g, t2 in f if kind == 'aborted'), None)
  stopped = [(t, float(g['hold'])) for t, kind, g, _ in f if kind == 'stopped']
  end = next(({'t': t, 'kind': kind, 'reason': g.get('reason', '')} for t, kind, g, _ in f if kind in ('done', 'not_counted')), None)
  return {'start': f[0][0], 'last': f[-1][0], 'seg_t': seg_t, 'aborted': aborted, 'end': end, 'close': rep['close'],
          'held_t': stopped[0][0] if stopped else None, 'hold_s_banner': stopped[-1][1] if stopped else None,
          'locked_held': any(kind == 'locked_held' for _, kind, _, _ in f), 'frames': len(f)}


def parse_hook_line(msg):
  m = HOOK_RX.fullmatch(msg.strip())
  if m:
    d = m.groupdict()
    return {'kind': 'state', 'state': d['state'], 'man': d['man'], 'rep': int(d['rep']), 'seg': int(d['seg']), 'reason': d['reason'],
            'floor': None if d['floor'] == 'None' else float(d['floor']), 'intent': d['intent'] == '1', 'done': d['done'], 'v': float(d['v'])}
  m = PROGRESS_RX.fullmatch(msg.strip())
  if m:
    try:
      record = ast.literal_eval(m['record'])
    except (ValueError, SyntaxError):
      record = None
    return {'kind': 'progress_' + m['what'], 'record': record}
  return {'kind': 'constructed' if 'constructed' in msg else 'other', 'text': msg}


def hook_group(lines, man, rep, t_from, t_to, used):
  """The rep's state lines: its start line (ACTIVE seg=1, no intent, no reason) up to the first resting state."""
  out = []
  for i, (t, d) in enumerate(lines):
    if i in used or d['kind'] != 'state' or not t_from <= t <= t_to or (d['man'], d['rep']) != (man, rep):
      continue
    if not out and not (d['state'] == 'ACTIVE' and d['seg'] == 1 and not d['intent'] and not d['reason']):
      continue
    out.append(i)
    if d['state'] not in MOVING:
      break
  used.update(out)
  return [lines[i] for i in out]


def device_label(group, b):
  """The device's own verdict: the resting cloudlog line, else the banner."""
  if group:
    last = group[-1][1]
    if last['state'] in MOVING:
      return 'incomplete'
    r = last['reason']
    return r if last['done'] or r in ('short-hold', 'overridden') else f'aborted({r or "unknown"})'
  if b['end'] and b['end']['kind'] == 'done':
    return 'counted'
  r = (b['end'] or b['aborted'] or {}).get('reason', '')
  return r if r in ('short-hold', 'overridden') else f'aborted({r or "unknown"})'


# ---- signal helpers ------------------------------------------------------------------------------------------------
def sl(s, a, b):
  i, j = np.searchsorted(s['t'], [a, b])
  return {k: v[i:j] for k, v in s.items()}


def at(s, key, t, stale=GAP_MAX):
  """Last sample at or before t (sample and hold); None if missing or older than `stale`."""
  i = np.searchsorted(s['t'], t, side='right') - 1
  return float(s[key][i]) if i >= 0 and t - s['t'][i] <= stale else None


def trailing(t, x, win, slope):
  """Least-squares slope (slope=True) or mean of x over each trailing window (t_i - win, t_i]; NaN below 5 samples."""
  out = np.full(len(t), np.nan)
  for i, j in enumerate(np.searchsorted(t, t - win, side='right')):
    if i + 1 - j >= 5:
      tt, xx = t[j:i + 1] - t[j:i + 1].mean(), x[j:i + 1]
      out[i] = np.dot(tt, xx - xx.mean()) / np.dot(tt, tt) if slope else xx.mean()
  return out


def ls_slope(t, x, a, b):
  m = (t >= a) & (t <= b)
  if m.sum() < 5:
    return None
  tt = t[m] - t[m].mean()
  return float(np.dot(tt, x[m] - x[m].mean()) / np.dot(tt, tt))


def mean_in(t, x, a, b):
  m = (t >= a) & (t <= b) & np.isfinite(x)
  return float(x[m].mean()) if m.any() else None


def pre_sd(t, s, w0, w1):
  """SD of the trailing statistic over the pre-window (windows fully inside it)."""
  v = s[(t >= w0 + SLOPE_S) & (t < w1) & np.isfinite(s)]
  return float(v.std()) if len(v) >= 5 else None


def departure(t, s, t_from, t_to, ref, sd, direction):
  """First sample time in [t_from, t_to] that starts ONSET_N consecutive samples whose statistic departs from ref by
  more than ONSET_K * sd in the commanded direction (sign of the command change); None if none."""
  dev = np.nan_to_num((s - ref) * direction, nan=-np.inf) > ONSET_K * sd
  for i in np.nonzero((t >= t_from) & (t <= t_to))[0]:
    if i + ONSET_N <= len(t) and dev[i:i + ONSET_N].all():
      return float(t[i])
  return None


def onset(t, x, s, t_edge, t_ref_from, sd_pre, direction, t_to, slope=True):
  """PLAN section 8 onset rule. s = trailing SLOPE_S statistic of x (wheel: LS slope of speed; IMU: mean). The
  pre-edge level is the same statistic over [max(t_edge - PRE_EDGE_S, t_ref_from), t_edge]; the threshold is ONSET_K
  pre-window SD (floored at SD_FLOOR). delay = onset sample time - t_edge (the send time of the new command)."""
  a = max(t_edge - PRE_EDGE_S, t_ref_from)
  ref = ls_slope(t, x, a, t_edge) if slope else mean_in(t, x, a, t_edge)
  sd = max(sd_pre if sd_pre is not None else 0.0, SD_FLOOR)
  t_on = None if ref is None else departure(t, s, t_edge, t_to, ref, sd, direction)
  return {'delay_s': None if t_on is None else t_on - t_edge, 'ref': ref, 'sd': sd, 'sd_floored': sd_pre is None or sd_pre < SD_FLOOR}


def gain(t, v, ts, te, cmd):
  """Realized/commanded over the last GAIN_S of a segment held >= HELD_S (LS wheel slope); None if shorter."""
  if te - ts < HELD_S - SEND_S:
    return None
  realized = ls_slope(t, v, te - GAIN_S, te)
  return {'realized': realized, 'gain': realized / cmd if realized is not None and cmd != 0.0 else None, 'window': [te - GAIN_S, te]}


def speed_bins(t, v, s, ts, te):
  """Net acceleration (trailing 0.3 s wheel slope) by 0.1 m/s bins of the window's mid speed, from SETTLE_S after the
  edge to the segment end."""
  m = (t - SLOPE_S >= ts + SETTLE_S) & (t <= te) & np.isfinite(s)
  if not m.any():
    return []
  vc = np.interp(t[m] - SLOPE_S / 2, t, v)
  out = []
  for b in np.unique(np.floor(vc / BIN_V + 1e-9)):
    a = s[m][np.floor(vc / BIN_V + 1e-9) == b]
    out.append({'v_lo': b * BIN_V, 'n': len(a), 'accel_mean': float(a.mean()), 'accel_sd': float(a.std())})
  return out


def q01(c):
  return round(c * 100) / 100


def body_frame(streams, t0, a, b):
  """Forward specific force and pitch rate in the calibrated frame (locationd device axes); the caller removes gravity
  (grade) and sensor bias with one per-rep offset."""
  imu, gyro, cal = sl(streams['imu'], a, b), sl(streams['gyro'], a, b), streams['calib']
  i = np.searchsorted(cal['t'], t0, side='right') - 1
  if i < 0 or not len(imu['t']):
    return None
  R = rot_from_euler([cal['roll'][i], cal['pitch'][i], cal['yaw'][i]])
  f = np.stack([-imu['z'], -imu['y'], -imu['x']], 1) @ R
  w = np.stack([-gyro['z'], -gyro['y'], -gyro['x']], 1) @ R if len(gyro['t']) else np.zeros((0, 3))
  return {'t': imu['t'], 'long': f[:, 0], 'gyro_t': gyro['t'], 'pitch_rate': w[:, 1],
          'calibrated': bool(cal['calibrated'][i]), 'rpy_calib': [cal['roll'][i], cal['pitch'][i], cal['yaw'][i]]}


def last_pulse(pul, a, b):
  """Time of the last WHL_PUL11 count increase (any wheel) in (a, b]: the wheel stop."""
  m = (pul['t'][1:] > a) & (pul['t'][1:] <= b) & (np.diff(pul['count']) > 0)
  idx = np.nonzero(m)[0]
  return float(pul['t'][idx[-1] + 1]) if len(idx) else None


def wire_edge(sc, tb):
  """Index of the first sent frame in [tb, tb + EDGE_SEARCH_S) whose value differs from the frame before it: where a
  segment starts on the wire (its script value, or the normal chain's value when that overrode the script); None if
  the wire did not change."""
  i0, i1 = np.searchsorted(sc['t'], [tb, tb + EDGE_SEARCH_S])
  if i0 == 0:
    return None
  ch = np.nonzero(np.abs(np.diff(sc['aReqValue'][i0 - 1:i1])) > SCRIPT_TOL)[0]
  return int(i0 + ch[0]) if len(ch) else None


def first_t(s, key, a, b, pred):
  x = sl(s, a, b)
  idx = np.nonzero(pred(x[key]))[0]
  return float(x['t'][idx[0]]) if len(idx) else None


def max_gap(s, a, b):
  t = sl(s, a, b)['t']
  if not len(t):
    return None
  return float(np.max(np.diff(np.concatenate(([a], t, [b])))))


# ---- one rep -------------------------------------------------------------------------------------------------------
def plan_of(man, logged):
  """(plan, source): the route's logged progress-record plan whose block has this maneuver, else the block that has it."""
  p = next((p for p in logged if man in {m[0] for m in BLOCKS.get(p, ())}), None)
  return (p, 'log') if p else (PLAN_OF.get(man), 'table')


def measure(rb, group, streams, meta, attempt, logged_plans=()):
  b = banner_facts(rb)
  man, rep = rb['man'], rb['rep']
  rid = f"{meta['route']}_{man}{rep}_{attempt}"
  commits = sorted({i['commit'] for i in meta['init']})
  dev_label = device_label(group, b)
  plan, plan_source = plan_of(man, logged_plans)
  base = {'id': rid, 'route': meta['route'], 'plan': plan, 'plan_source': plan_source, 'commits': commits, 'maneuver': man, 'rep': rep,
          'attempt': attempt, 'device_label': dev_label, 'counted_on_device': bool(group and group[-1][1]['done'] == man)}

  def times(t0):
    """Banner facts and cloudlog lines with times from t0 (cloudlog times are forwarding times: order only)."""
    def r(t):
      return None if t is None else t - t0
    return {'banner': {**b, 'start': r(b['start']), 'last': r(b['last']), 'held_t': r(b['held_t']),
                       'seg_t': {str(k): r(v) for k, v in b['seg_t'].items()},
                       'aborted': b['aborted'] and {**b['aborted'], 't': r(b['aborted']['t'])},
                       'end': b['end'] and {**b['end'], 't': r(b['end']['t'])}},
            'cloudlog': [{'t_logged': r(t), **d} for t, d in group]}

  spec = MAN.get(man)
  if spec is None or 1 not in b['seg_t']:
    return {**base, **times(b['start']), 'label': 'unknown', 'times_from': 'first banner frame',
            'reason': 'maneuver not in the table' if spec is None else 'rep starts before the log'}, None
  segs = spec[2]
  sc = streams['scc12']

  # command edges: the first sent SCC12 frame that changed the wire after each segment's first banner frame, with its
  # step from the previous sent frame (on an overridden start the step goes to the normal chain's value: known input)
  edges = []
  for k, seg in enumerate(segs):
    tb = b['seg_t'].get(k + 1)
    if tb is None:
      break
    i = wire_edge(sc, tb)
    if i is None:
      edges.append({'t': tb, 'source': 'banner', 'banner_t': tb, 'step': None})
      continue
    to = float(sc['aReqValue'][i])
    edges.append({'t': float(sc['t'][i]), 'source': 'scc12' if abs(to - q01(seg.accel)) <= SCRIPT_TOL + 1e-9 else 'wire', 'banner_t': tb,
                  'step': {'from': float(sc['aReqValue'][i - 1]), 'to': to}})
  t0 = edges[0]['t']
  w0 = t0 - PRE_S
  car = streams['car']
  t_last = b['last']
  t_abort = b['aborted']['t'] if b['aborted'] else math.inf
  t_flag = first_t(car, 'standstill', t0, t_last + 1.0, lambda x: x > 0.5)
  t_brake = first_t(car, 'brake', t_flag, t_last + 1.0, lambda x: x > 0.5) if t_flag is not None else None
  t_end = t_brake if t_brake is not None else (b['aborted']['t'] if b['close'] == 'driver' else t_last)
  n = len(edges)
  ends = [min(edges[k + 1]['t'] if k + 1 < n else math.inf, t_flag if t_flag is not None else math.inf, t_abort, t_end) for k in range(n)]
  t_stop = last_pulse(streams['pul'], t0, t_flag) if t_flag is not None else None
  stop_source = 'pulses' if t_stop is not None else ('flag' if t_flag is not None else None)
  t_stop = t_stop if t_stop is not None else t_flag

  win = (t0 - PAD_S[0], t_end + PAD_S[1])
  whl = sl(streams['whl'], *win)
  wt, wv = whl['t'], whl['mean']
  ws = trailing(wt, wv, SLOPE_S, True)
  body = body_frame(streams, t0, *win)
  imu_offset = None
  if body is not None:
    imu_pre, wheel_pre = mean_in(body['t'], body['long'], w0, t0), ls_slope(wt, wv, w0, t0)
    if imu_pre is not None and wheel_pre is not None:
      imu_offset = imu_pre - wheel_pre
      body['long'] = body['long'] - imu_offset
    body['s'] = trailing(body['t'], body['long'], SLOPE_S, False)
    body['m01'] = trailing(body['t'], body['long'], IMU_MEAN_S, False)
  sd_wheel = pre_sd(wt, ws, w0, t0)
  sd_imu = pre_sd(body['t'], body['s'], w0, t0) if body else None

  # stall: the floor deepens below a speed/standstill-ended segment's command below STALL_V (the device owns the wire)
  rc = sl(sc, win[0], t_end + 0.05)
  t_stall = math.inf
  for k, seg in enumerate(segs[:n]):
    if seg.t_s is None:
      m = (rc['t'] >= edges[k]['t']) & (rc['t'] < ends[k]) & (rc['aReqValue'] < q01(seg.accel) - SCRIPT_TOL)
      for t in rc['t'][m]:
        v = at(car, 'v', t)
        if v is not None and v < STALL_V:
          t_stall = min(t_stall, float(t))
          break
  stall_logged = any(d['state'] == 'ACTIVE' and d['intent'] and not d['reason'] and p['seg'] == d['seg']
                     for (_, p), (_, d) in zip(group, group[1:], strict=False))

  t_intent = t_stall
  for k, seg in enumerate(segs[:n]):
    if seg.t_s is None:
      t = first_t(car, 'v', edges[k]['t'] - 0.02, ends[k], lambda x: x <= INTENT_V[plan])
      t_intent = min(t_intent, t if t is not None else math.inf)

  # pre-window (PLAN section 2)
  pc, pw = sl(car, w0, t0), sl(sc, w0, t0)
  pre = {'car_gap_s': max_gap(car, w0, t0),
         'dv_max': float(np.max(np.abs(pc['v'] - pc['v_cruise']))) if len(pc['t']) else None,
         'a_max': float(np.max(np.abs(pc['a']))) if len(pc['t']) else None,
         'wire_max': float(np.max(np.abs(pw['aReqValue']))) if len(pw['t']) else None,
         'v_cruise': float(np.median(pc['v_cruise'])) if len(pc['t']) else None, 'sd_wheel_slope': sd_wheel, 'sd_imu': sd_imu}
  pre['ok'] = bool(pre['car_gap_s'] is not None and pre['car_gap_s'] <= GAP_MAX and pre['dv_max'] <= V_STEADY
                   and pre['a_max'] <= A_STEADY and pre['wire_max'] is not None and pre['wire_max'] <= WIRE_STEADY)

  s14, esp, tcs13, lcs, echo = streams['scc14'], streams['esp12'], streams['tcs13'], streams['lcs'], sl(streams['scc12_echo'], *win)
  echo_idx = defaultdict(list)
  for t, dat in zip(echo['t'], echo['dat'], strict=True):
    echo_idx[dat].append(t)

  def echo_t(t, dat):
    return next((e for e in echo_idx.get(dat, ()) if t <= e <= t + GAP_MAX), None)

  segments, overridden, script_ok = [], False, True
  for k, seg in enumerate(segs[:n]):
    ts, te = edges[k]['t'], ends[k]
    tm = max(ts, min(te, t_stall))   # the scripted part: from the stall on, the ramp deepens the floor by design
    tp = max(ts, min(tm, t_stop if t_stop is not None else math.inf))   # moving: gains and bins end at the wheel stop
    step = edges[k]['step']
    direction = 1.0 if (step['to'] > step['from'] if step else seg.accel > (segs[k - 1].accel if k else 0.0)) else -1.0
    t_ref_from = edges[k - 1]['t'] if k else w0
    t_to = min(ts + MAX_DELAY_S, tm)
    frames = sl(rc, ts, tm)
    dev = frames['aReqValue'] - q01(seg.accel)
    overridden |= bool(np.any(dev < -SCRIPT_TOL))
    script_ok &= bool(np.all(np.abs(dev) <= SCRIPT_TOL))
    i_edge = np.searchsorted(rc['t'], ts)
    t_echo = echo_t(ts, rc['dat'][i_edge]) if i_edge < len(rc['t']) else None
    f14 = sl(s14, ts, te)
    lim = Counter(zip(f14['JerkUpperLimit'].round(2), f14['JerkLowerLimit'].round(2), strict=True))
    g = gain(wt, wv, ts, tp, seg.accel)
    if g is not None and body is not None:
      g['imu_mean'] = mean_in(body['t'], body['long'], *g['window'])
      g['esp12_mean'] = mean_in(esp['t'], esp['LONG_ACCEL'], *g['window'])
      g['accel_ref_acc_mean'] = mean_in(tcs13['t'], tcs13['ACCEL_REF_ACC'], *g['window'])
      g['brake_light_frac'] = mean_in(tcs13['t'], tcs13['BrakeLight'], *g['window'])
    if g is not None:
      g['window'] = [x - t0 for x in g['window']]
    fl = sl(lcs, ts, te)
    segments.append({
      'seg': k + 1, 'cmd': seg.accel, 'v_end': seg.v_end, 't_s': seg.t_s, 'edge_source': edges[k]['source'], 'wire_step': step,
      't_start': ts - t0, 't_end': te - t0, 'scripted_until': tm - t0, 'moving_until': tp - t0, 'duration_s': te - ts,
      'edge_minus_banner_s': ts - edges[k]['banner_t'], 'edge_echo_lag_s': None if t_echo is None else t_echo - ts,
      'v_entry_wheel': float(np.interp(ts, wt, wv)) if len(wt) else None, 'v_entry_ego': at(car, 'v', ts),
      'v_exit_wheel': float(np.interp(te, wt, wv)) if len(wt) else None, 'v_exit_ego': at(car, 'v', te),
      'direction': direction,
      # onsets time the response to the sent step (a known-input step to the normal chain's value on an overridden start)
      'onset_wheel': onset(wt, wv, ws, ts, t_ref_from, sd_wheel, direction, t_to) if step else None,
      'onset_imu': onset(body['t'], body['long'], body['s'], ts, t_ref_from, sd_imu, direction, t_to, slope=False) if body and step else None,
      'gain': g, 'bins': speed_bins(wt, wv, ws, ts, tp),
      'script': {'frames': len(dev), 'max_abs_dev': float(np.max(np.abs(dev))) if len(dev) else None,
                 'mismatch': int(np.sum(np.abs(dev) > SCRIPT_TOL)), 'deeper': int(np.sum(dev < -SCRIPT_TOL))},
      'scc14_at_edge': {'upper': at(s14, 'JerkUpperLimit', ts + 0.02, 0.05), 'lower': at(s14, 'JerkLowerLimit', ts + 0.02, 0.05)},
      'scc14_limits': [{'upper': u, 'lower': lo, 'frames': c} for (u, lo), c in sorted(lim.items())],
      'long_control_state': {name: int(np.sum(fl['state'] == code)) for name, code in LCS.items()},
    })

  # terminal and hold
  terminal = hold = None
  mpc = None
  if len(wt):
    wp, pul = sl(whl, w0, t0), streams['pul']
    counts = np.interp([w0, t0], pul['t'], pul['count']) if len(pul['t']) > 1 else None
    if counts is not None and counts[1] > counts[0] and len(wp['t']) > 1:
      mpc = float(np.trapezoid(wp['mean'], wp['t']) / (counts[1] - counts[0]))
  if t_flag is not None and len(wt):
    i_flag = np.searchsorted(wt, t_flag, side='right')
    above = np.nonzero(wv[:i_flag] > TERMINAL_V)[0]
    t05 = None
    if len(above) and above[-1] + 1 < len(wt):
      i = above[-1]
      t05 = float(np.interp(TERMINAL_V, [wv[i + 1], wv[i]], [wt[i + 1], wt[i]])) if wv[i] != wv[i + 1] else float(wt[i])
    a, bnd = (t05 if t05 is not None else t_stop - 1.0), t_flag + TERMINAL_AFTER_S

    def extreme(t, x, fn, signed=None):
      """(value, time from the wheel stop) of the extreme of x in the terminal window; `signed` reports another array's value."""
      m = (t >= a) & (t <= bnd) & np.isfinite(x)
      if not m.any():
        return None, None
      i = np.nonzero(m)[0][fn(x[m])]
      return float((x if signed is None else signed)[i]), float(t[i] - t_stop)

    imu_min = extreme(body['t'], body['m01'], np.argmin) if body else (None, None)
    esp_min = extreme(esp['t'], esp['LONG_ACCEL'], np.argmin)
    peak = (None, None)
    if body is not None and len(body['gyro_t']):
      pr = body['pitch_rate'] - (mean_in(body['gyro_t'], body['pitch_rate'], w0, t0) or 0.0)   # gyroUncalibrated: remove bias
      peak = extreme(body['gyro_t'], np.abs(pr), np.argmax, signed=pr)
    t_sr = first_t(sc, 'StopReq', t_flag - 1.0, t_end, lambda x: x > 0.5)   # a StopReq after the driver's brake is not the hold's
    d_end = min(t_stop + DISP_S, t_brake if t_brake is not None else math.inf)
    pul = streams['pul']
    dc = float(np.diff(np.interp([t_stop, d_end], pul['t'], pul['count']))[0]) if len(pul['t']) > 1 else None
    fw = sl(whl, t_stop, d_end)
    terminal = {
      'window': [a - t0, bnd - t0], 't_stop': t_stop - t0, 'stop_source': stop_source, 't_flag': t_flag - t0, 'stop_to_flag_s': t_flag - t_stop,
      't05_to_stop_s': None if t05 is None else t_stop - t05,
      'imu_min_0p1s': imu_min[0], 'imu_min_t_from_stop': imu_min[1], 'esp12_min': esp_min[0], 'esp12_min_t_from_stop': esp_min[1],
      'pitch_rate_peak': peak[0], 'pitch_rate_peak_t_from_stop': peak[1],
      'decel_at_stop': {'wheel_slope_0p3s': ls_slope(wt, wv, t_stop - SLOPE_S, t_stop),
                        'imu_0p1s': float(np.interp(t_stop, body['t'], body['m01'])) if body else None,
                        'esp12': at(esp, 'LONG_ACCEL', t_stop), 'a_ego': at(car, 'a', t_stop)},
      'flag_to_stopreq_s': None if t_sr is None else t_sr - t_flag,
      # after the wheel stop the ABS speed is a decay curve: its integral bounds travel from above, the pulses measure it
      'displacement': {'window_s': d_end - t_stop, 'pulses': dc, 'm_per_pulse': mpc,
                       'pulse_m': dc * mpc if dc is not None and mpc is not None else None,
                       'wheel_integral_upper_m': float(np.trapezoid(fw['mean'], fw['t'])) if len(fw['t']) > 1 else None},
    }
    if t_brake is not None:
      t_held = b['held_t'] if b['held_t'] is not None else t_flag
      hf = sl(sc, t_held, t_brake)
      hold_dev = None
      if len(hf['t']):
        a0, ta = float(hf['aReqValue'][0]), float(hf['t'][0])
        exp = np.maximum(a0 - J_HOLD * (hf['t'] - ta), A_HOLD) if a0 > A_HOLD else np.full(len(hf['t']), a0)
        hold_dev = float(np.max(np.abs(hf['aReqValue'] - exp)))
      sr = sl(sc, t_sr, t_brake) if t_sr is not None else None
      after = sl(sc, t_brake, t_brake + 0.07)   # the first sent frames after the brake (card lags controlsd by a frame)
      cs = sl(car, t_flag, t_brake)
      esc_from = min(t_flag + DISP_S, t_brake)
      hold = {'hold_s': t_brake - t_flag, 'hold_s_banner': b['hold_s_banner'], 'brake_minus_banner_s':
              None if b['end'] is None else t_brake - b['end']['t'], 'wire_max_dev': hold_dev,
              'stopreq_frac': float(np.mean(sr['StopReq'])) if sr is not None and len(sr['t']) else None,
              'brake_frames': [{'t_from_brake': float(after['t'][i] - t_brake), **{k: float(after[k][i]) for k in ('aReqValue', 'StopReq', 'ACCMode')}}
                               for i in range(len(after['t']))],
              'escape': bool(np.any(cs['standstill'] < 0.5)),
              'escape_pulses': float(np.diff(np.interp([esc_from, t_brake], pul['t'], pul['count']))[0]) if len(pul['t']) > 1 else None,
              'avh_lamp_max': (lambda x: float(x.max()) if len(x) else None)(sl(streams['tcs15'], t_flag, t_brake + 0.5)['AVH_LAMP']),
              'pbrake_act_max': (lambda x: float(x.max()) if len(x) else None)(sl(tcs13, t_flag, t_brake + 0.5)['PBRAKE_ACT'])}

  # grade and bearing from the pre-window
  cp = sl(streams['cc'], w0, t0)
  pitch = mean_in(cp['t'], cp['pitch'], w0, t0)
  esp_pre, wheel_pre = mean_in(esp['t'], esp['LONG_ACCEL'], w0, t0), ls_slope(wt, wv, w0, t0)
  gps = sl(streams['gps'], w0, t0)
  fix = gps['fix'] > 0.5
  bearing = (math.degrees(math.atan2(np.mean(np.sin(np.radians(gps['bearing'][fix]))), np.mean(np.cos(np.radians(gps['bearing'][fix]))))) % 360
             if fix.any() else None)
  grade = {'pitch_rad': pitch, 'grade_pct_pitch': None if pitch is None else 100 * math.tan(pitch),
           'grade_pct_esp12': None if esp_pre is None or wheel_pre is None else 100 * (esp_pre - wheel_pre) / G,
           'imu_offset': imu_offset, 'imu_calibrated': body['calibrated'] if body else None, 'rpy_calib': body['rpy_calib'] if body else None}
  # gain use gates on the ESP12 grade; the orientationNED pitch grade is metadata (it read -2.45 % against ESP12 on KCS1 drive 1)
  grade['ok'] = grade['grade_pct_esp12'] is not None and abs(grade['grade_pct_esp12']) <= GRADE_MAX_PCT

  # checks (PLAN section 2 / runbook B7)
  t_moving_end = t_brake if t_brake is not None else t_end
  # a stall intent is timed by its first deeper sent frame, up to one send period after the controlsd frame
  pre_l, post_l = sl(lcs, t0, min(t_intent - 0.02, t_moving_end)), sl(lcs, t_intent + LCS_LAG_S, t_moving_end)
  first_stop = first_t(lcs, 'state', t0, t_moving_end, lambda x: x == LCS['stopping'])
  dc_, dcc = sl(car, w0, t_moving_end), sl(streams['cc'], w0, t_moving_end)
  sent = sl(sc, w0, t_end)
  miss = [t for t, d in zip(sent['t'], sent['dat'], strict=True) if echo_t(t, d) is None] if len(echo['t']) else None
  # the Panda sees the driver's brake before carState does and blocks the last frame sent before it (not a CAN gap)
  blocked = bool(miss and t_brake is not None and miss[-1] == sent['t'][-1] and t_brake - miss[-1] <= PANDA_BLOCK_S)
  missing = None if miss is None else len(miss) - blocked
  gaps = {name: max_gap(streams[name], w0, t_end) for name in ('car', 'whl', 'scc12', 'scc12_echo')}
  mc = {}
  for k, seg in enumerate(segs):
    name = f's{k + 1}'
    if k >= n:
      mc[f'{name}_ran'] = False
      continue
    if seg.v_end is not None and k + 1 < n:
      v_edge = at(car, 'v', edges[k + 1]['t'])
      mc[f'{name}_edge_v'] = v_edge is not None and abs(v_edge - seg.v_end) <= EDGE_V_TOL
    if k + 1 < len(segs) and segs[k + 1].accel > seg.accel:
      mc[f'{name}_held_before_release'] = ends[k] - edges[k]['t'] >= HELD_S - SEND_S
    if seg.t_s is not None:
      fc = sl(car, edges[k]['t'], ends[k])
      mc[f'{name}_full'] = k + 1 < n and ends[k] - edges[k]['t'] >= seg.t_s - 0.03
      mc[f'{name}_v_end'] = (at(car, 'v', ends[k]) or 0.0) >= 0.5
      if seg.accel == 0.0:
        mc[f'{name}_coast_band'] = bool(len(fc['v']) and np.all((fc['v'] >= 0.8) & (fc['v'] <= 3.3)))
  stalled = t_stall < math.inf or stall_logged
  checks = {
    'identity': bool(commits) and bool(group),
    'pre_window': pre['ok'],
    'script': script_ok and all(e['source'] == 'scc12' for e in edges),
    'hold_wire': None if hold is None or hold['wire_max_dev'] is None else hold['wire_max_dev'] <= HOLD_TOL,
    'echo': missing == 0,
    'long_control_state': bool(np.all(pre_l['state'] == LCS['pid']) and np.all(post_l['state'] == LCS['stopping'])),
    'gaps': all(g is not None and g <= GAP_MAX for g in gaps.values()),
    'driver': not any(np.any(dc_[k] > 0.5) for k in ('brake', 'gas', 'esp', 'acc_fault')) and bool(np.all(dc_['valid'] > 0.5))
              and not np.any(dcc['override'] > 0.5) and bool(np.all(dcc['enabled'] > 0.5) and np.all(dcc['long_active'] > 0.5)),
    'standstill_under_script': t_flag is not None and t_flag <= t_abort and n == len(segs),
    'hold_min': hold is not None and hold['hold_s'] >= HOLD_MIN_S - 1e-6,
    'maneuver': all(mc.values()),
  }
  abort_reason = b['aborted']['reason'] if b['aborted'] else None
  if abort_reason is None and dev_label.startswith('aborted(') and dev_label != 'aborted(unknown)':
    abort_reason = dev_label[8:-1]
  if abort_reason:
    label = f'aborted({abort_reason})'
  elif overridden:   # from the wire; the device flags 100 Hz carControl differences that never reach it (device_label)
    label = 'overridden'
  elif t_flag is None or t_brake is None:
    label = 'aborted(incomplete)'
  elif t_brake - t_flag < HOLD_MIN_S - 1e-6 or dev_label == 'short-hold':
    label = 'short-hold'
  else:
    label = 'stalled' if stalled else 'complete'
  failed = sorted(k for k, v in checks.items() if v is False)
  valid = label in ('complete', 'stalled') and not failed
  record = {
    **base, **times(t0), 'label': label, 'label_agrees': dev_label == label or (dev_label == 'counted' and label in ('complete', 'stalled')),
    'valid_for_fit': valid, 'gain_use_ok': valid and grade['ok'], 'terminal_use_ok': valid and not stalled, 'failed_checks': failed,
    'profile': spec[1], 't0_ns': int(round(t0 * 1e9)), 't_end': t_end - t0, 't_abort': None if b['aborted'] is None else t_abort - t0,
    'pre_window': pre, 'segments': segments,
    'stall': {'t': None if t_stall == math.inf else t_stall - t0, 'logged': stall_logged},
    'intent': {'t': None if t_intent == math.inf else t_intent - t0, 'first_stopping_t': None if first_stop is None else first_stop - t0,
               'stopping_lag_s': None if first_stop is None or t_intent == math.inf else first_stop - t_intent},
    'terminal': terminal, 'hold': hold, 'grade': grade,
    'gps': {'bearing_deg': bearing, 'speed': mean_in(gps['t'], gps['speed'], w0, t0), 'n': int(fix.sum())},
    'checks': checks, 'maneuver_checks': mc, 'echo_missing': missing, 'echo_blocked_at_brake': blocked, 'gaps_s': gaps,
    'series_file': f'series/{rid}.npz',
  }
  series = {f'{name}__{k}': (v - t0 if k == 't' else v) for name, s in streams.items() for k, v in sl(s, *win).items() if k != 'dat'}
  if body is not None:
    series.update({'body__t': body['t'] - t0, 'body__long': body['long'], 'body__gyro_t': body['gyro_t'] - t0,
                   'body__pitch_rate': body['pitch_rate']})
  series.update({'whl__slope_0p3s': ws, 't0_ns': np.int64(record['t0_ns'])})
  return record, series


# ---- route, output -------------------------------------------------------------------------------------------------
def analyze(streams, meta):
  lines = [(t, parse_hook_line(msg)) for t, msg in meta['hook_lines']]
  # the running build's plan is on its saved records; a loaded record can be another plan's counts (the hook restarts them)
  records = [d for kind in ('progress_saved', 'progress_loaded') for _, d in lines if d['kind'] == kind]
  plans = list(dict.fromkeys(d['record']['plan'] for d in records if isinstance(d['record'], dict) and d['record'].get('plan')))
  used, attempts, out = set(), Counter(), []
  for rb in segment_reps(meta['banner']):
    attempts[(rb['man'], rb['rep'])] += 1
    group = hook_group(lines, rb['man'], rb['rep'], rb['frames'][0][0] - 1.0, rb['frames'][-1][0] + 5.0, used)
    out.append(measure(rb, group, streams, meta, attempts[(rb['man'], rb['rep'])], plans))
  starts = [i for i, (_, d) in enumerate(lines) if d['kind'] == 'state' and d['state'] == 'ACTIVE' and d['seg'] == 1 and not d['intent']
            and not d['reason']]
  info = {'route': meta['route'], 'plans': plans, 'files': meta['files'], 'commits': sorted({i['commit'] for i in meta['init']}),
          'banner_frames': len(meta['banner']), 'hook_lines': len(lines),
          'constructed': sum(d['kind'] == 'constructed' for _, d in lines),
          'progress': [{'kind': d['kind'], 'record': d['record']} for _, d in lines if d['kind'].startswith('progress')],
          'unmatched_cloudlog_reps': len([i for i in starts if i not in used])}
  return out, info


def clean(x):
  if isinstance(x, dict):
    return {str(k): clean(v) for k, v in x.items()}
  if isinstance(x, (list, tuple)):
    return [clean(v) for v in x]
  if isinstance(x, (bool, np.bool_)):
    return bool(x)
  if isinstance(x, (int, np.integer)):
    return int(x)
  if isinstance(x, (float, np.floating)):
    return float(x) if math.isfinite(x) else None
  return x


def fmt(x, nd=2):
  return '-' if x is None else f'{x:+.{nd}f}'


def summary(records, infos):
  out = [f"# Identification reps ({', '.join(sorted({r['plan'] for r in records if r.get('plan')})) or '-'})", '']
  for i in infos:
    out.append(f"- route `{i['route']}` (logged plans {', '.join(i['plans']) or '-'}): {len(i['files'])} rlogs, " +
               f"commits {', '.join(c[:10] for c in i['commits']) or '-'}; " +
               f"banner frames {i['banner_frames']}, hook lines {i['hook_lines']} (constructed {i['constructed']}, " +
               f"unmatched cloudlog reps {i['unmatched_cloudlog_reps']})")
    out.extend(f"  - {p['kind'].replace('_', ' ')}: {p['record']}" for p in i['progress'])
  labels = Counter(r['label'] for r in records)
  out += ['', f"{len(records)} reps ({', '.join(f'{k} {v}' for k, v in sorted(labels.items())) or 'none'}); " +
              f"valid for fit {sum(bool(r.get('valid_for_fit')) for r in records)}.", '']
  if not records:
    out.append('No reps found.')
    return '\n'.join(out) + '\n'
  out += ['| maneuver | rep | try | label | device | fit | onset delay wheel/IMU (s) | gain (last 1 s) | terminal IMU / ESP12 / wheel at stop ' +
          '| hold s | failed checks |', '|' + '---|' * 11]
  for r in records:
    segs = r.get('segments', [])
    delays = '; '.join(f"s{s['seg']} {fmt((s['onset_wheel'] or {}).get('delay_s'))}/{fmt((s['onset_imu'] or {}).get('delay_s'))}" +
                       ('' if s['edge_source'] == 'scc12' else f" ({s['edge_source']})") for s in segs)
    gains = '; '.join(f"s{s['seg']} " + (fmt(s['gain']['gain']) if s['cmd'] else 'a=' + fmt(s['gain']['realized']))
                      for s in segs if s['gain'])
    t = r.get('terminal') or {}
    term = ' / '.join(fmt(x) for x in (t.get('imu_min_0p1s'), t.get('esp12_min'), (t.get('decel_at_stop') or {}).get('wheel_slope_0p3s')))
    hold = (r.get('hold') or {}).get('hold_s')
    out.append(f"| {r['maneuver']} | {r['rep']} | {r['attempt']} | {r['label']} | {r['device_label']} | {'yes' if r.get('valid_for_fit') else 'no'} " +
               f"| {delays or '-'} | {gains or '-'} | {term if t else '-'} | {'-' if hold is None else f'{hold:.1f}'} " +
               f"| {', '.join(r.get('failed_checks', [])) or '-'} |")
  return '\n'.join(out) + '\n'


def run(paths, output):
  output = Path(output)
  if output.exists():
    raise FileExistsError(f'{output} exists; choose a new directory')
  by_route = defaultdict(list)
  for p in paths:
    by_route[route_of(p)[0]].append(Path(p))
  results, infos = [], []
  for route in sorted(by_route):
    reps, info = analyze(*read_route(by_route[route]))
    results += reps
    infos.append(info)
  output.mkdir(parents=True)
  (output / 'series').mkdir()
  with open(output / 'reps.jsonl', 'w') as f:
    for record, series in results:
      if series is not None:
        np.savez_compressed(output / record['series_file'], **series)
      f.write(json.dumps(clean(record), allow_nan=False) + '\n')
  records = [clean(r) for r, _ in results]
  (output / 'summary.md').write_text(summary(records, infos))
  return records, infos


def main(argv=None):
  ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
  ap.add_argument('rlogs', nargs='+', type=Path)
  ap.add_argument('--output', required=True, type=Path, help='New destination directory')
  args = ap.parse_args(argv)
  records, infos = run(args.rlogs, args.output)
  print(json.dumps({'routes': [i['route'] for i in infos], 'rlogs': sum(len(i['files']) for i in infos), 'reps': len(records),
                    'labels': dict(Counter(r['label'] for r in records)), 'output': str(args.output)}))


if __name__ == '__main__':
  main()
