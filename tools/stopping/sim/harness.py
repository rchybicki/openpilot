# ruff: noqa: RUF100, ISC002, E501, C420, F401, UP034  (copied engine code kept verbatim; README.md)
"""Cycle-1002 stopping evaluation harness (read-only on the repo; usage, validation and limits in HARNESS.md).

  from openpilot.tools.stopping.sim import harness as H
  r = H.run('bk_222e_s4', 'recorded')                    # exact open-loop replay (variant-C input timing)
  r = H.run('bk_222e_s4', 'closed', cell='level0.47')     # KCS plant from the logged service entry
  r['trace'] -> dict of numpy columns, r['metrics'] -> dict, r['fidelity'] (recorded mode), r['info']

Cases: CASES (rlog), natural_cases() (28 saved stops), synthetic_case(). A variant is a zero-argument callable returning
a context manager that patches modules for the run (patched(), src_patch()). Closed-loop claims: report BOTH cells
'nominal' and 'level0.47' (the nominal cell's brake-off trigger turns the guarded landings into creeping stalls; the
level cells reproduce them). Times are route-relative seconds (logMonoTime - initData.logMonoTime, the trace_win base).
"""
import bisect
import contextlib
import inspect
import json
import multiprocessing as mp
import pickle
import sys
import textwrap
from pathlib import Path
from types import SimpleNamespace

sys.dont_write_bytecode = True  # the repo is read-only for this harness: no __pycache__ writes

import numpy as np  # noqa: E402

REPO = Path(__file__).resolve().parents[3]
RD = Path.home() / '.route_sync/data/media/0/realdata'
HERE = Path(__file__).resolve().parent
from openpilot.tools.stopping.sim import SIM_HOME  # noqa: E402   (sim package: case cache outside the repo)
CACHE = SIM_HOME / 'cases'
SERV = ('carState', 'radarState', 'longitudinalPlan', 'frogpilotCarState', 'frogpilotPlan', 'selfdriveState', 'modelV2')
DT = 0.01
G = 9.81
RADAR_VLEAD_PER_AEGO = 0.089  # s: stationary-lead vLead bias = 0.089 x aEgo (24,128 frames, host_latency/fit2.py)
STREAMS = {'car': ('v', 'a', 'standstill', 'brake', 'gas'), 'cc': ('accel', 'pitch', 'long_active'),
           'imu': ('x', 'y', 'z'), 'gyro': ('x', 'y', 'z'), 'calib': ('roll', 'pitch', 'yaw'),
           'scc12': ('aReqValue', 'StopReq', 'ACCMode'), 'scc14': ('JerkUpperLimit', 'JerkLowerLimit'),
           'esp12': ('LONG_ACCEL',), 'whl': ('mean',), 'pul': ('count', 'WHL_PUL_FL', 'WHL_PUL_FR', 'WHL_PUL_RL', 'WHL_PUL_RR'), 'tcs13': ('BrakeLight',)}

# ---- case specs: route, window (route-relative s), class, entry/stop hints (route-relative) -------------------------
_A = 'class A (stopped lead within 5.5 m at service entry; verify_history/vdetect.json)'
_N = 'near class A (stopped lead 5.5-7.6 m at entry, request <= -1.5 below 2 m/s; verify_history cand/results.jsonl)'
CASES = {
  'bk_222e_s4': dict(route='0000222e--f3c0c5bb86', lo=270.0, hi=300.5, cls='bookmark', entry=285.752, note='bookmark plunge + hold'),
  'good_2226_s3': dict(route='00002226--2f0fa69b0c', lo=185.0, hi=240.5, cls='good', note='post-deploy guarded stop'),
  'good_2226_s4': dict(route='00002226--2f0fa69b0c', lo=242.0, hi=292.5, cls='good', note='post-deploy guarded stop'),
  'good_222d_s3': dict(route='0000222d--aefebbe583', lo=215.0, hi=249.0, cls='good', note='post-deploy guarded stop'),
  'hold_20c1_690': dict(route='000020c1--8f82c447fd', lo=672.0, hi=736.0, cls='classB', note='crawl-lane deep hold -3.25 21.5 s'),
  'hold_20ff_387': dict(route='000020ff--375628a698', lo=370.0, hi=440.0, cls='classB', note='crawl-lane deep hold -1.90'),
}
for _route, _te, _ts in (('0000203a--41d916332f', 622.168, 624.307), ('0000203a--41d916332f', 948.123, 949.099),
                         ('00002045--b05f683895', 441.012, 442.732), ('00002045--b05f683895', 451.551, 453.972),
                         ('00002048--2eef08bee9', 208.665, 210.653), ('00002049--1da44dd6cd', 700.005, 701.928),
                         ('0000205d--b0e1709ece', 668.918, None), ('0000205d--b0e1709ece', 2981.255, 2983.162),
                         ('00002086--df4f36fce8', 1044.615, 1046.308), ('000020b7--b7f3994c55', 1296.979, 1298.121),
                         ('000020c0--418683bc8f', 361.856, 362.741), ('000020c0--418683bc8f', 2227.52, 2229.075),
                         ('000020c0--418683bc8f', 3668.892, 3670.798), ('000020f8--3495298f77', 2191.68, 2193.9),
                         ('000020f8--3495298f77', 2193.028, 2193.9)):
  CASES.setdefault(f'A_{_route[4:8]}_{int(_te)}', dict(route=_route, lo=_te - 16.0, hi=(_ts or _te + 4.0) + 8.0, cls='classA',
                                                         entry=_te, note=_A))
for _route, _te in (('00002048--2eef08bee9', 165.79), ('00002082--e7e32684a7', 413.94), ('000020c1--8f82c447fd', 24.80),
                    ('000020fd--6c49baf803', 876.29), ('00002082--e7e32684a7', 39.80), ('000020fa--32a67f8d0c', 315.59),
                    ('0000204a--4d869abb53', 391.67), ('00002100--1d4b9d86ff', 913.86)):
  CASES[f'N_{_route[4:8]}_{int(_te)}'] = dict(route=_route, lo=max(_te - 16.0, 1.0), hi=_te + 12.0, cls='nearA', entry=_te, note=_N)
CASES.update({
  'c31_s5': dict(route='00002231--73a1ec87ea', lo=285.0, hi=400.0, cls='c31', entry=327.728, note='2231 s5 stop, 58 s hold, gas launch 388.07'),
  'c31_s18': dict(route='00002231--73a1ec87ea', lo=1080.0, hi=1118.0, cls='c31', entry=1108.52, note='2231 s18 rolling stop, release at 0.39 m/s'),
  'c31_s20': dict(route='00002231--73a1ec87ea', lo=1200.0, hi=1252.0, cls='c31', entry=1240.79, note='2231 s20 bookmark: brake pump before stopping'),
  'c31_s22': dict(route='00002231--73a1ec87ea', lo=1335.0, hi=1392.0, cls='c31', entry=1376.65, note='2231 s22 stop'),
})
# 222e 285.765 is the bookmark itself (bk_222e_s4); the class-A duplicate key is dropped
CASES.pop('A_222e_285', None)


def _seg_paths(route, lo, hi):
  """rlog paths whose events can fall in [lo, hi] (route-relative). Segment s starts near 60 s."""
  out = []
  for s in range(max(int((lo - 12) // 60) - 1, 0), int((hi + 2) // 60) + 2):
    p = RD / f'{route}--{s}/rlog.zst'
    if p.is_file():
      out.append(p)
  return out


def _events(path):
  import zstandard
  from cereal import log
  raw = zstandard.ZstdDecompressor().decompress(Path(path).read_bytes(), max_output_size=int(9e8))
  return list(log.Event.read_multiple_bytes(raw))


def _mpp():
  """The frozen KCS1 reps 1-4 median metres per mean-wheel pulse count (plant_data.attach_pulse_truth)."""
  from openpilot.tools.stopping.review.plant_data import CORPUS
  rows = [json.loads(x) for x in (CORPUS / 'kcs1_drive1_20260926/analysis_block1_v2/reps.jsonl').read_text().splitlines()]
  return float(np.median([r['terminal']['displacement']['m_per_pulse'] for r in rows
                          if r['rep'] <= 4 and (r.get('terminal') or {}).get('displacement', {}).get('m_per_pulse')]))


def _small(stream, ev):
  """The fields LongControl and the replay need, as plain python values."""
  if stream == 'carState':
    c = ev.carState
    return dict(vEgo=c.vEgo, aEgo=c.aEgo, vEgoRaw=c.vEgoRaw, standstill=c.standstill, cruiseState={'standstill': c.cruiseState.standstill},
                gasPressed=c.gasPressed, brakePressed=c.brakePressed, canValid=c.canValid, canTimeout=c.canTimeout, vCruise=c.vCruise)
  if stream == 'radarState':
    a, b = ev.radarState.leadOne, ev.radarState.leadTwo
    return dict(lead_status=a.status, lead_v=a.vLead, lead_d_rel=a.dRel, lead_a=a.aLeadK, lead_track_id=a.radarTrackId,
                lead_model_prob=a.modelProb, lead2_status=b.status, lead2_v=b.vLead, lead2_d_rel=b.dRel)
  if stream == 'longitudinalPlan':
    lp = ev.longitudinalPlan
    return dict(aTarget=lp.aTarget, shouldStop=lp.shouldStop, dts=lp.distanceToStopTarget, fcw=bool(lp.fcw),
                dts_model=float(lp.distanceToStopTargetModel), traj=(lp.aTargetTrajectory if lp.aTargetTrajectoryValid else None),
                valid=ev.valid)
  if stream == 'frogpilotCarState':
    return dict(force_coast=ev.frogpilotCarState.forceCoast)
  if stream == 'frogpilotPlan':
    return dict(isd=ev.frogpilotPlan.increasedStoppedDistance)
  if stream == 'selfdriveState':
    return dict(experimental=ev.selfdriveState.experimentalMode)
  if stream == 'modelV2':
    return dict(model_should_stop=ev.modelV2.action.shouldStop)
  raise KeyError(stream)


def _frame(snap, cc_ev):
  cs, rs, lp = snap['carState'], snap['radarState'], snap['longitudinalPlan']
  cc = cc_ev.carControl
  t = cc_ev.logMonoTime * 1e-9
  kw = dict(experimental_mode=snap['selfdriveState']['experimental'], **rs, fcw=lp['fcw'], model_stop_d=lp['dts_model'],
            model_should_stop=snap['modelV2']['model_should_stop'], force_coast=snap['frogpilotCarState']['force_coast'],
            increased_stopped_distance=snap['frogpilotPlan']['isd'], a_target_trajectory=lp['traj'],
            freeze_integrator=cs['gasPressed'], plan_valid=lp['valid'])
  return dict(t=t, cs=cs, kw=kw, target=lp['aTarget'], should_stop=lp['shouldStop'], dts=lp['dts'], active=cc.longActive,
              recorded=float(cc.actuators.accel),
              authorized=bool(cc_ev.valid and cc.enabled and cc.longActive and not cc.cruiseControl.override
                              and not cs['gasPressed'] and not cs['brakePressed'] and cs['canValid'] and not cs['canTimeout']))


def _extract(cid, spec):
  """Variant-C inputs (latest message by logMonoTime <= the matching selfdriveState) + motion-truth streams."""
  from openpilot.tools.stopping.review import kcs1_reps
  paths = _seg_paths(spec['route'], spec['lo'], spec['hi'])
  if not paths:
    raise FileNotFoundError(f'{cid}: no local rlog for {spec["route"]} {spec["lo"]}-{spec["hi"]}')
  evs = [e for p in paths for e in _events(p)]
  init = next(e for e in evs if e.which() == 'initData')
  origin = init.logMonoTime * 1e-9
  lo, hi = origin + spec['lo'], origin + spec['hi']
  keep = ('HumanAcceleration', 'LongitudinalTune', 'CEForceCoastStrength', 'IncreasedStoppedDistance', 'ExperimentalMode')
  settings = {kv.key: bytes(kv.value).decode(errors='replace') for kv in init.initData.params.entries if kv.key in keep}
  cp = next(e.carParams.as_builder().to_bytes() for e in evs if e.which() == 'carParams')
  mono = {s: ([], []) for s in SERV}
  for e in evs:
    w = e.which()
    if w in SERV:
      mono[w][0].append(e.logMonoTime)
      mono[w][1].append(e)
  for s in SERV:
    order = sorted(range(len(mono[s][0])), key=mono[s][0].__getitem__)
    mono[s] = ([mono[s][0][i] for i in order], [mono[s][1][i] for i in order])
  memo, frames = {}, []
  for e in evs:
    if e.which() != 'carControl' or not lo <= e.logMonoTime * 1e-9 <= hi:
      continue
    i = bisect.bisect_right(mono['selfdriveState'][0], e.logMonoTime) - 1
    if i < 0:
      continue
    sds_ns = mono['selfdriveState'][0][i]
    snap = {}
    for s in SERV:
      j = bisect.bisect_right(mono[s][0], sds_ns) - 1
      if j < 0:
        break
      if (s, j) not in memo:
        memo[(s, j)] = _small(s, mono[s][1][j])
      snap[s] = memo[(s, j)]
    else:
      frames.append(_frame(snap, e))
  ft = np.array([f['t'] for f in frames])
  if len(ft) and ft[0] > lo + 0.5 and spec.get('trim_lo'):  # rharness: a window opening before the logged controller frames starts at them
    lo = ft[0]
  if len(ft) < 100 or ft[0] > lo + 0.5 or ft[-1] < hi - 0.5 or np.max(np.diff(ft)) > 0.1:
    raise ValueError(f'{cid}: incomplete controller frames n={len(ft)} span={ft[:1]}..{ft[-1:]} '
                     f'max_gap={np.max(np.diff(ft)) if len(ft) > 1 else None}')
  radar = [(e.logMonoTime * 1e-9, *[_small('radarState', e)[k] for k in ('lead_status', 'lead_d_rel', 'lead_v', 'lead_a', 'lead_track_id')])
           for e in mono['radarState'][1] if lo - 10 <= e.logMonoTime * 1e-9 <= hi + 5]
  streams, _ = kcs1_reps.read_route([str(p) for p in paths])
  S = {}
  for name, cols in STREAMS.items():
    s = streams[name]
    m = (s['t'] >= lo - 10) & (s['t'] <= hi + 5)
    S[f'{name}__t'] = s['t'][m]
    for c in cols:
      S[f'{name}__{c}'] = np.asarray(s[c], dtype=float)[m]
  return dict(id=cid, kind='rlog', route=spec['route'], cls=spec.get('cls'), note=spec.get('note', ''), origin=origin,
              lo=lo, hi=hi, entry_hint=origin + spec['entry'] if spec.get('entry') else None, paths=[str(p) for p in paths],
              commit=init.initData.gitCommit, settings=settings, cp=cp, frames=frames,
              radar=np.array(radar, dtype=float).reshape(-1, 6), S=S, mpp=_mpp(),
              t_ref=origin + spec['t_ref'] if spec.get('t_ref') else None)  # cyc_1003r: rolling-stop reference (no standstill)


def _held(t, x, q):
  t, x = np.asarray(t), np.asarray(x)
  return x[np.clip(np.searchsorted(t, q, side='right') - 1, 0, len(t) - 1)]


def _trailing_mean(t, x, win):
  c = np.concatenate(([0.0], np.cumsum(x)))
  j = np.searchsorted(t, t - win, side='right')
  n = np.arange(1, len(t) + 1) - j
  return (c[1:] - c[j]) / np.maximum(n, 1)


def _derive(c):
  """Truth signals on the controller frame grid: pulse distance/speed, wheel stop, 2.5 m/s crossing, grade, realised accel."""
  S, ft = c['S'], np.array([f['t'] for f in c['frames']])
  v_ego = np.array([f['cs']['vEgo'] for f in c['frames']])
  pt, px = S['pul__t'], S['pul__count'] * c['mpp']
  if len(pt) > 10:
    left, right = np.maximum(ft - 0.1, pt[0]), np.minimum(ft + 0.1, pt[-1])
    c['x_true'] = np.interp(ft, pt, px)
    c['v_true'] = (np.interp(right, pt, px) - np.interp(left, pt, px)) / np.maximum(right - left, 1e-3)
    c['truth'] = 'WHL_PUL11 mean-wheel counts x frozen KCS1 reps 1-4 scale'
  else:  # no pulse truth: vEgo integral (lags the wheels by 0.1-0.2 m/s in hard braking)
    c['x_true'] = np.r_[0.0, np.cumsum((v_ego[:-1] + v_ego[1:]) / 2 * np.diff(ft))]
    c['v_true'] = v_ego
    c['truth'] = 'vEgo integral (no pulses)'
  # the stop: first carState standstill rising edge after the entry hint (or after the first in-window approach)
  ct, ss = S['car__t'], S['car__standstill'] > 0.5
  ref = (c['entry_hint'] - 1.0) if c.get('entry_hint') else c['lo'] + 3.0
  edges = np.flatnonzero(ss[1:] & ~ss[:-1] & (ct[1:] > ref) & (ct[1:] <= c['hi'])) + 1
  c['t_flag'] = float(ct[edges[0]]) if len(edges) else None
  c['t_stop'] = None
  if c['t_flag'] is not None:
    inc = np.flatnonzero((np.diff(S['pul__count']) > 0) & (pt[1:] <= c['t_flag']) & (pt[1:] > c['t_flag'] - 1.0)) if len(pt) > 10 else []
    c['t_stop'] = float(pt[inc[-1] + 1]) if len(inc) else c['t_flag'] - 0.22
  if c['t_stop'] is None and c.get('t_ref'):  # cyc_1003r: a rolling stop has no standstill edge: its logged speed minimum
    c['t_stop'] = c['t_ref']
  # no logged stop: the approach is the one into the entry hint (the entry still yields wire metrics)
  stop_i = int(np.searchsorted(ft, c['t_stop'] if c['t_stop'] else (c['entry_hint'] or c['hi'])))
  above = np.flatnonzero(v_ego[:stop_i] >= 2.5)
  k = min(above[-1] + 1, len(ft) - 1) if len(above) else min(300, len(ft) - 1)
  # the final ENGAGED approach: after the last driver intervention (not longActive, gas, brake) before the stop
  drv = np.array([not f['active'] or f['cs']['gasPressed'] or f['cs']['brakePressed'] for f in c['frames'][k:stop_i]], dtype=bool)
  if drv.any():
    k = min(k + int(np.flatnonzero(drv)[-1]) + 1, len(ft) - 1)
  c['cross'] = float(ft[k])
  # grade (percent, + uphill): ESP12 LONG_ACCEL minus the WHL_SPD slope in 1 s windows before the stop (kcs1_reps definition)
  wt, wv, et, ea = S['whl__t'], S['whl__mean'], S['esp12__t'], S['esp12__LONG_ACCEL']
  g = []
  for a in np.arange(c['cross'] - 6.0, (c['t_stop'] or c['hi']) - 1.0, 1.0):
    mw, me = (wt >= a) & (wt <= a + 1), (et >= a) & (et <= a + 1)
    if mw.sum() >= 20 and me.sum() >= 20 and wv[mw].mean() > 1.0:
      g.append(100 * (ea[me].mean() - np.polyfit(wt[mw], wv[mw], 1)[0]) / G)
  c['grade'] = float(np.median(g)) if g else 0.0
  c['grade_n'] = len(g)
  # the same estimate as a road profile (by truth position, 1 s windows every 0.5 s, v > 1 m/s) for the closed loop
  gx, gv = [], []
  for a in np.arange(c['cross'] - 12.0, (c['t_stop'] or c['hi']) - 0.5, 0.5):
    mw, me = (wt >= a) & (wt <= a + 1), (et >= a) & (et <= a + 1)
    if mw.sum() >= 20 and me.sum() >= 20 and wv[mw].mean() > 1.0:
      gx.append(float(np.interp(a + 0.5, ft, c['x_true'])))
      gv.append(100 * (ea[me].mean() - np.polyfit(wt[mw], wv[mw], 1)[0]) / G)
  c['grade_profile'] = (np.array(gx), np.array(gv)) if len(gx) >= 2 else None
  # realised accel: gravity-compensated IMU body accel, 0.1 s trailing mean (kcs1 terminal.body); pulse fallback
  # realised WHEEL accel (the plant's quantity): gradient of the 0.2 s pulse speed, 0.2 s trailing mean. Pulse quantization
  # (0.0213 m/count) makes it noisy at walking pace (about +-0.3 near the stop): use a_stop_wheel / IMU for the terminal
  c['a_real'] = _trailing_mean(ft, np.gradient(c['v_true'], ft), 0.2)
  c['a_imu'], c['kcs'] = None, None
  if len(S.get('imu__t', [])) > 100 and len(S.get('calib__t', [])):
    from openpilot.tools.stopping.sim import terminal as term   # sim package: kcs1 terminal_hold/terminal.py body()/metrics()
    o = np.argsort(S['imu__t'], kind='stable')
    S2 = {k: (v[o] if k.startswith('imu__') else v) for k, v in S.items()}
    bt, bl, _, _ = term.body(S2, (c['t_stop'] or c['cross']) - 3.0)
    c['a_imu'] = np.interp(ft, bt, _trailing_mean(bt, bl, 0.1))  # gravity-compensated body accel (felt), 0.1 s mean
    if c['t_flag'] is not None:  # the established a_stop / j300 (kcs1 terminal.metrics, as stopmetrics.py)
      try:
        after = np.flatnonzero((ct > c['t_flag']) & (~ss | (S['car__brake'] > 0.5) | (S['car__gas'] > 0.5)))
        t_end = min(float(ct[after[0]]) if len(after) else c['t_flag'] + 3.0, c['t_flag'] + 3.0)
        km = term.metrics(S2, c['t_flag'], t_end)
        c['kcs'] = dict(a_stop=km['arrive_rel'], j300=km['j300_imu_max'], j300_esp=km['j300_esp_max'], imu_min=km['imu_min'],
                        esp_min=km['esp_min'], a_rest=km['a_rest'])
      except Exception as exc:  # noqa: BLE001 -- recorded-only reference metric; keep the case usable
        c['kcs'] = dict(error=f'{type(exc).__name__}: {exc}')
  return c


def case(cid, refresh=False):
  """A named case (CASES key, a natural-stop id from natural_cases(), or a case dict passed through). Cached."""
  if isinstance(cid, dict):
    return cid
  CACHE.mkdir(parents=True, exist_ok=True)
  path = CACHE / f'{cid}.pkl'
  if path.is_file() and not refresh:
    c = pickle.loads(path.read_bytes())
    if 'entry_rec' not in c:
      c['entry_rec'] = _logged_entry(c)
      path.write_bytes(pickle.dumps(c))
    return c
  if cid in CASES:
    c = _derive(_extract(cid, CASES[cid]))
  else:
    c = next(x for x in natural_cases(ids=[cid]))
  c['entry_rec'] = _logged_entry(c)
  path.write_bytes(pickle.dumps(c))
  return c


def _logged_entry(c):
  """The service entry of the final approach in the variant-free recorded replay (the closed loop's default takeover)."""
  tr = _run(c, 'recorded', 'nominal', {})['trace']
  sa = tr['svc_active'].astype(bool) & (tr['t'] + c['origin'] <= (c.get('t_stop') or c['hi']))
  on = np.flatnonzero(sa[1:] & ~sa[:-1]) + 1
  return float(tr['t'][on[-1]] + c['origin']) if len(on) else c['cross']


def natural_cases(ids=None):
  """The 28 saved natural stops (plant_data.natural_entries + sim/inputs.json; inputs extracted at the carControl
  publication in file order = ~1 frame early vs variant C, recorded on their historical commits)."""
  from openpilot.tools.stopping.review import plant_data
  out, entries = [], plant_data.natural_entries()
  todo = [e for e in entries if ids is None or e['id'] in ids]
  cached = {e['id']: CACHE / f'{e["id"]}.pkl' for e in todo}
  missing = [e for e in todo if not cached[e['id']].is_file()]
  if missing:
    inputs = json.loads((plant_data.OUTPUT / 'inputs.json').read_text())
    scale = plant_data.attach_pulse_truth(missing, inputs)
    for e in missing:
      inp = inputs[e['id']]
      origin = next(iter(_events(e['paths'][0]))).logMonoTime * 1e-9  # initData = route boot (trace_win base)
      keep = ('vEgo', 'aEgo', 'vEgoRaw', 'standstill', 'gasPressed', 'brakePressed', 'canValid', 'canTimeout', 'vCruise')
      frames = [dict(f, cs={**{k: f['cs'].get(k, 0.0) for k in keep}, 'cruiseState': {'standstill': f['cs']['cruiseState'].get('standstill', False)}})
                for f in inp['frames']]
      for f in frames:
        f['kw'] = {k: v for k, v in f['kw'].items() if k != 'request_time'}
      pul = np.asarray(inp['pulses'])
      raw = np.round(pul[:, 1:] / 0.5)
      counts = np.r_[0.0, np.cumsum((np.diff(raw, axis=0) % 256).mean(axis=1))]
      ft = np.array([f['t'] for f in frames])
      S = {'pul__t': pul[:, 0], 'pul__count': counts, 'car__t': ft,
           'car__standstill': np.array([float(f['cs']['standstill']) for f in frames]),
           'scc12__t': e['send_t'], 'scc12__aReqValue': e['send_u'], 'scc12__StopReq': _held(e['t'], e['stop_req'], e['send_t']),
           'scc14__t': e['jerk_t'], 'scc14__JerkUpperLimit': e['jerk_up'], 'scc14__JerkLowerLimit': e['jerk_lo'],
           'whl__t': np.zeros(0), 'whl__mean': np.zeros(0), 'esp12__t': np.zeros(0), 'esp12__LONG_ACCEL': np.zeros(0)}
      radar = []
      for f in frames:
        k = f['kw']
        row = (f['t'], float(k['lead_status']), k['lead_d_rel'], k['lead_v'], k['lead_a'], float(k['lead_track_id']))
        if not radar or row[1:] != radar[-1][1:]:
          radar.append(row)
      c = dict(id=e['id'], kind='natural', route=e['route'], cls='natural', note=e['truth'], origin=origin, lo=ft[0], hi=ft[-1],
               entry_hint=None, paths=e['paths'], commit=e['recorded_commit'], settings=inp['settings'],
               cp=bytes.fromhex(inp['cp']), frames=frames, radar=np.array(radar, dtype=float), S=S, mpp=scale,
               rest_gap_manifest=e['rest_gap'])
      c = _derive(c)
      c['cross'] = e['cross']  # plant_data's recorded vEgo 2.5 crossing (the frozen gate-A entry)
      c['t_stop'] = e['rest']  # last pulse edge before the standstill flag (attach_pulse_truth)
      cached[e['id']].write_bytes(pickle.dumps(c))
  for e in todo:
    out.append(pickle.loads(cached[e['id']].read_bytes()))
  return out


# ---- variants -------------------------------------------------------------------------------------------------------
@contextlib.contextmanager
def patched(*items):
  """Set (obj, attr, value) for the run and restore afterwards: `variant = lambda: patched((stopping_flags, 'FINAL_FLOOR', False))`."""
  saved = [(obj, attr, obj.__dict__.get(attr, getattr(obj, attr))) for obj, attr, _ in items]
  try:
    for obj, attr, value in items:
      setattr(obj, attr, value)
    yield
  finally:
    for obj, attr, value in reversed(saved):
      setattr(obj, attr, value)


def src_patch(owner, name, *pairs):
  """A modified copy of owner.name (function or method): each (old, new) source replacement must match exactly once.
  Compiled in the defining module's globals, so it is a drop-in value for patched((owner, name, src_patch(...)))."""
  fn = owner.__dict__[name] if isinstance(owner, type) else getattr(owner, name)
  src = textwrap.dedent(inspect.getsource(fn))
  for old, new in pairs:
    if src.count(old) != 1:
      raise ValueError(f'{owner.__name__}.{name}: {old!r} matches {src.count(old)} times')
    src = src.replace(old, new)
  ns = {}
  exec(compile(src, f'<variant {owner.__name__}.{name}>', 'exec'), fn.__globals__, ns)
  return ns[name]


# ---- lead model for the closed loop -----------------------------------------------------------------------------------
def lead_model(c, smooth=0.0, radar_lag=0.0, freeze_after=None):
  """Lead path in the truth-ego frame (metres; ego = c['x_true']). x = integral of the bias-corrected radar Doppler
  (vLead - 0.089 aEgo) + the geometric residual (ego position + dRel - integral) averaged over +-smooth s within the same
  radar track. The modelled radar therefore returns the recorded dRel/vLead when the plant reproduces the recorded motion,
  INCLUDING the recorded range artifacts (e.g. the bookmark's 0.3 m post-stop drift appears as lead motion).
  smooth=0: no residual smoothing -- x is the raw geometry (ego position + dRel) and the run samples it with
  sample-and-hold, i.e. the recorded dRel corrected only for the ego displacement difference (keeps the recorded radar
  step timing, which the post-stop crawl lane reacts to). freeze_after (route-relative s): the lead is stationary from
  then on (a clean stopped lead). Returns (t, x, v) arrays."""
  if 'lead' in c:
    return c['lead']
  R = c['radar']
  ok = (R[:, 1] > 0.5) & np.isfinite(R[:, 2]) & (R[:, 2] > 0)
  t, d, vl, tid = R[ok, 0], R[ok, 2], R[ok, 3], R[ok, 5]
  ft = np.array([f['t'] for f in c['frames']])
  S = c['S']
  a_e = np.interp(t, S['car__t'], S['car__a']) if len(S.get('car__a', [])) else np.interp(t, ft, [f['cs']['aEgo'] for f in c['frames']])
  v = vl - RADAR_VLEAD_PER_AEGO * a_e
  V = np.r_[0.0, np.cumsum((v[:-1] + v[1:]) / 2 * np.diff(t))]
  r = np.interp(t - radar_lag, ft, c['x_true']) + d - V
  rs = np.empty_like(r) if smooth > 0 else r
  for i in range(len(t) if smooth > 0 else 0):
    lo, hi = np.searchsorted(t, t[i] - smooth), np.searchsorted(t, t[i] + smooth, side='right')
    m = tid[lo:hi] == tid[i]
    rs[i] = r[lo:hi][m].mean()
  x = V + rs
  if freeze_after is not None:
    k = t >= c['origin'] + freeze_after
    if k.any():
      x[k] = np.interp(c['origin'] + freeze_after, t, x)
      v[k] = 0.0
  return t, x, v


def monotone_plant():
  """kcs_plant.Plant with a monotone post-stop ABS-tail observation floor (harness default, obs='monotone').
  The original floor 0.10/(1 + since/0.38) restarts at the wheel stop (since re-referenced from the v < 0.15 frame to
  the stop), so the observed vEgo rises 0.074 -> 0.099 m/s about 0.2 s after every stop; the logged vEgo decays
  monotonically on all four validation stops, and the bump armed the anti-hover monitor in 3/3 guarded stops (the
  logs never armed it). Motion is unchanged: only the controller's vEgo observation differs."""
  from openpilot.tools.stopping.review.kcs_plant import Plant

  class MonotonePlant(Plant):
    step = src_patch(Plant, 'step', ('    self.tail_t = self.t\n', '    self.tail_t, self._floor = self.t, .10\n'),
                     ('    raw = max(raw, .10 / (1 + since / .38), .0087)',
                      '    self._floor = min(self._floor, .10 / (1 + since / .38))\n    raw = max(raw, self._floor, .0087)'))
  return MonotonePlant


def _quantize(x, q):
  return round(x / q) * q if q else x


# ---- the run ----------------------------------------------------------------------------------------------------------
def _setup(c):
  from openpilot.common.swaglog import cloudlog, ipchandler
  with contextlib.suppress(ValueError):
    cloudlog.removeHandler(ipchandler)
  cloudlog.disabled = True
  from cereal import car
  from openpilot.selfdrive.controls.lib.longcontrol import LongControl
  from openpilot.selfdrive.controls.lib.stopping_telemetry import StoppingTelemetry
  from opendbc.car.hyundai.interface import CarInterface
  from opendbc.car.hyundai.tests.test_can_bounds_fork import make_controller, run_frame, get_signal
  with car.CarParams.from_bytes(c['cp']) as reader:
    cp = reader.as_builder()
  lc = LongControl(cp)
  lc._service_shadow_tel = StoppingTelemetry(log_fn=lambda **kw: None)
  st = c['settings']
  toggles = SimpleNamespace(vEgoStarting=cp.vEgoStarting, vEgoStopping=cp.vEgoStopping, startAccel=cp.startAccel,
                            human_acceleration=st.get('HumanAcceleration') == '1' and st.get('LongitudinalTune') == '1',
                            force_coast_strength=float(st.get('CEForceCoastStrength') or 1.0), max_desired_acceleration=4.0)
  sender, _ = make_controller(cp)
  return cp, lc, toggles, CarInterface, sender, run_frame, get_signal


def _cs(d):
  return SimpleNamespace(**{k: (SimpleNamespace(**v) if isinstance(v, dict) else v) for k, v in d.items()})


def run(c, mode='recorded', variant=None, cell='nominal', **opts):
  """Run one case. mode 'recorded' = open-loop replay of the logged inputs; 'closed' = KCS plant from the crossing.
  variant: zero-arg callable -> context manager (entered for the whole run, LongControl is built inside it).
  opts (closed): cross = 'entry' (default: the logged service entry of the final approach), 'v2.5' (the last vEgo 2.5 m/s
  crossing, the plant_data/gate-A convention) or a route-relative time; warm (2.1 s plant warm-up on the recorded SCC12);
  reseed (True: at the crossing the plant speed is reset to the pulse truth and its vEgo/aEgo observation to the logged
  values, removing warm-up drift; False = the KCS free-roll protocol); t_end (route-relative); gap_q (0.1 m);
  obs ('monotone' default | 'kcs': the plant's original post-stop vEgo observation, see monotone_plant());
  radar_hz (20); vlead_k (0.089 s); vlead_noise (0 m/s sigma); seed; lead_smooth (0 = raw recorded geometry, default; > 0 = residual smoothing
  window, s, see lead_model()); radar_lag (0 s);
  lead_freeze_after (route-relative s); grade (constant override, percent; default = the case's road grade profile).
  flags: dict of stopping_flags overrides (convenience). keep_trace (True)."""
  from openpilot.selfdrive.controls.lib import stopping_flags
  c = case(c)
  ctx = variant() if variant is not None else contextlib.nullcontext()
  fl = patched(*[(stopping_flags, k, v) for k, v in (opts.get('flags') or {}).items()])
  with ctx, fl, patched((stopping_flags, 'IDENTIFICATION_HOOK', False)):
    out = _run(c, mode, cell, opts)
  if not opts.get('keep_trace', True):
    out.pop('trace')
  return out


def _run(c, mode, cell, opts):
  from openpilot.tools.stopping.review.kcs_plant import GAIN, Plant, cells
  Plant = monotone_plant() if opts.get('obs', 'monotone') == 'monotone' else Plant  # noqa: N806
  cp, lc, toggles, CI, sender, run_frame, get_signal = _setup(c)
  frames, o = c['frames'], c['origin']
  ft = np.array([f['t'] for f in frames])
  closed = mode == 'closed'
  if mode not in ('recorded', 'closed'):
    raise ValueError(mode)
  svc = lc._service_shadow_svc
  inner, box = svc.update, {}

  def capture(**kw):
    r = inner(**kw)
    box['dbg'], box['svc_active'] = r.debug, r.active
    return r
  svc.update = capture
  S = c['S']
  sent_log = _held(S['scc12__t'], S['scc12__aReqValue'], ft) if len(S['scc12__t']) else np.full(len(ft), np.nan)
  cross = opts.get('cross', 'entry' if closed and c['kind'] != 'synthetic' else None)
  if cross == 'v2.5':
    cross = None
  if cross == 'entry':  # the LOGGED service entry of the final approach (variant-independent: recorded replay, no variant)
    if c.get('entry_rec') is None:
      c['entry_rec'] = _logged_entry(c)
    cross = c['entry_rec']
  elif cross is not None:
    cross = o + float(cross)  # route-relative input
  else:
    cross = c['cross']
  t_end = c['hi'] if opts.get('t_end') is None else o + opts['t_end']
  # metrics start at the 2.5 m/s crossing in both modes; the closed loop takes over at `cross` (info['closed_from'])
  info = dict(id=c['id'], mode=mode, cls=c['cls'], cross=min(cross, c['cross']) - o, closed_from=cross - o if closed else None,
              grade=None, cell=None, seed_speed_error=None)
  if closed:
    warm = 0.0 if c['kind'] == 'synthetic' else float(opts.get('warm', 2.1))
    seed_t = cross - warm
    if seed_t < ft[0] - 1e-6:
      raise ValueError(f'{c["id"]}: crossing {cross - o:.2f} leaves < {warm} s of warm-up in the window')
    cl = cells(True) if isinstance(cell, str) and cell not in cells() else cells()
    cobj = cl[cell] if isinstance(cell, str) else cell
    grade = float(opts.get('grade', c['grade']))
    profile = c.get('grade_profile') if opts.get('grade') is None else None
    info.update(grade=grade, cell=cell if isinstance(cell, str) else repr(cell))
    lt, lx, lv = lead_model(c, opts.get('lead_smooth', 0.0), opts.get('radar_lag', 0.0),
                            opts.get('lead_freeze_after'))
    lead_at = (lambda q_: float(_held(lt, lx, q_))) if opts.get('lead_smooth', 0.0) == 0 else (lambda q_: float(np.interp(q_, lt, lx)))
    pre = ft[ft < seed_t - 1e-9]
    grid = np.r_[pre, np.arange(seed_t if len(pre) == 0 else max(seed_t, pre[-1] + DT), t_end + 1e-9, DT)]
    rng = np.random.default_rng(opts.get('seed', 0))
    k_v, q, every = opts.get('vlead_k', RADAR_VLEAD_PER_AEGO), opts.get('gap_q', 0.1), max(int(round(100 / opts.get('radar_hz', 20))), 1)
  else:
    grid = ft[ft <= t_end + 1e-9]
  idx = np.clip(np.searchsorted(ft, grid, side='right') - 1, 0, len(ft) - 1)
  lc.last_output_accel = frames[idx[0]]['recorded']
  plant, radar, sent, upper, lower, stop_req, k_cross = None, None, 0.0, 3.0, 5.0, False, None
  rows = []
  for k, (now, i) in enumerate(zip(grid, idx, strict=True)):
    f = frames[i]
    cs = _cs(f['cs'])
    kw = dict(f['kw'])
    if closed and plant is None and now >= seed_t - 1e-9:
      v0 = float(np.interp(now, ft, c['v_true'])) if c['kind'] != 'synthetic' else f['cs']['vEgo']
      plant = Plant(max(v0, 0.0), f['cs']['aEgo'], float(_held(S['scc12__t'], S['scc12__aReqValue'], now)) if len(S['scc12__t']) else f['recorded'],
                    cobj, GAIN, grade)
    loop = closed and now >= cross - 1e-9
    if loop and k_cross is None:
      k_cross = k
      info['seed_speed_error'] = plant.v - float(np.interp(now, ft, c['v_true']))
      if opts.get('reseed', True) and c['kind'] != 'synthetic':
        plant.v = float(np.interp(now, ft, c['v_true']))
        plant.speed_history.extend(float(np.interp(now - DT * j, ft, c['v_true'])) for j in range(9, -1, -1))
        plant.v_ego, plant.a_ego, plant.raw = f['cs']['vEgo'], f['cs']['aEgo'], f['cs']['vEgo']
      x0 = float(np.interp(now, ft, c['x_true']))
      plant.x = 0.0
    if loop:
      cs.vEgo, cs.aEgo, cs.vEgoRaw, cs.standstill = plant.v_ego, plant.a_ego, plant.raw, plant.standstill
      cs.cruiseState.standstill = plant.standstill
      if radar is None or (k - k_cross) % every == 0:
        x_ego = x0 + plant.x
        lead_ok = bool(kw['lead_status']) and lt[0] <= now <= lt[-1]
        radar = dict(lead_d_rel=_quantize(lead_at(now) - x_ego, q) if lead_ok else kw['lead_d_rel'],
                     lead_v=(float(np.interp(now, lt, lv)) + k_v * plant.a_ego
                             + (rng.normal(0.0, opts['vlead_noise']) if opts.get('vlead_noise') else 0.0)) if lead_ok else kw['lead_v'],
                     lead2_d_rel=kw['lead2_d_rel'] + float(np.interp(now, ft, c['x_true'])) - x_ego,
                     gap_true=lead_at(now) - x_ego if lead_ok else np.nan,
                     lv_true=float(np.interp(now, lt, lv)) if lead_ok else np.nan)
      kw.update({key: radar[key] for key in ('lead_d_rel', 'lead_v', 'lead2_d_rel')})
    target, should_stop, dts = f['target'], f['should_stop'], f['dts']
    if c.get('planner') is not None:  # synthetic: a reactive planner proxy on what the controller sees this frame
      p = c['planner'](now - o, cs.vEgo, kw['lead_d_rel'], kw['lead_v'])
      target, should_stop, dts = p.get('a_target', 0.0), p.get('should_stop', False), p.get('dts', -1.0)
      kw['a_target_trajectory'] = p.get('a_traj')
    if not f['active']:
      lc.reset()
    prev_state = lc.long_control_state
    limits = CI.get_pid_accel_limits(cp, cs.vEgo, cs.vCruise / 3.6)
    box.clear()
    out = lc.update(f['active'], cs, target, should_stop, dts, limits, toggles, request_time=float(now), **kw)
    cmd = float(min(out, toggles.max_desired_acceleration))
    lc.observe_accel_request(cmd, float(now), authorized=f['authorized'])
    _, msgs = run_frame(sender, dict(accel=cmd, state=prev_state, v_ego=cs.vEgo, a_ego=cs.aEgo, long_active=f['active'],
                                     gas_pressed=cs.gasPressed))
    if 0x421 in msgs:
      sent = get_signal('SCC12', 'aReqValue', msgs[0x421])
      stop_req = bool(get_signal('SCC12', 'StopReq', msgs[0x421]))
    if 0x389 in msgs:
      upper, lower = get_signal('SCC14', 'JerkUpperLimit', msgs[0x389]), get_signal('SCC14', 'JerkLowerLimit', msgs[0x389])
    row = dict(t=now - o, v=cs.vEgo, a_ego=cs.aEgo, wire=cmd, logged=f['recorded'], sent=sent, active=f['active'],
               gas=f['cs']['gasPressed'], brake=f['cs']['brakePressed'], lead_status=bool(kw['lead_status']),
               gap_meas=kw['lead_d_rel'] if kw['lead_status'] else np.nan, lead_v=kw['lead_v'] if kw['lead_status'] else np.nan,
               a_target=target, should_stop=should_stop, phase=svc.phase.name, svc_active=bool(box.get('svc_active')),
               owning=lc._service_live_owning, lcs=int(lc.long_control_state), ff_armed=lc._final_floor_armed,
               ff_level=lc._final_floor_level, ff_added=lc._final_floor_added, ff_releasing=lc._final_floor_releasing,
               mon_trig=svc._mon_triggered, mon_floor=svc._mon_floor, mon_active=svc._mon_active, mon_stopped_s=svc._mon_stopped_s,
               latch_gap=svc.ev.latch_gap, closed=loop,
               trim_i=lc._trim_i, trim_hold=float(lc._trim_hold), trim_wire=lc._trim_wire, pid_i=float(lc.pid.i),
               lead_stopped=float(lc._service_signals.lead_confirmed_stopped) if lc._service_signals is not None else np.nan,
               wstop=float(lc._service_shadow_ctx._wstop_latched), ff_bound_frames=lc._final_floor_bound_frames)
    for key, val in (box.get('dbg') or {}).items():
      if key != 'phase':
        row['d_' + key] = val
    if plant is not None:
      u = sent if loop else float(_held(S['scc12__t'], S['scc12__aReqValue'], now))
      up = upper if loop else float(_held(S['scc14__t'], S['scc14__JerkUpperLimit'], now)) if len(S['scc14__t']) else 3.0
      lo_ = lower if loop else float(_held(S['scc14__t'], S['scc14__JerkLowerLimit'], now)) if len(S['scc14__t']) else 5.0
      sr = stop_req if loop else bool(_held(S['scc12__t'], S['scc12__StopReq'], now)) if len(S['scc12__t']) else False
      v_before = plant.v
      if profile is not None:
        plant.grade = float(np.interp(float(np.interp(now, ft, c['x_true'])) if not loop else x0 + plant.x, *profile)) + cobj.grade
      plant.step(float(u), float(up), float(lo_), sr)
      row.update(v_true=plant.v if loop else float(np.interp(now, ft, c['v_true'])), v_plant=plant.v, a_real=(plant.v - v_before) / DT,
                 x=plant.x if loop else np.nan, plant_off=plant.off, plant_a=plant.a,
                 gap=radar['gap_true'] if loop else row['gap_meas'], lead_v_true=radar['lv_true'] if loop else np.nan, sent_log=u)
    else:
      row.update(v_true=float(np.interp(now, ft, c['v_true'])), a_real=float(np.interp(now, ft, c['a_real'])),
                 x=float(np.interp(now, ft, c['x_true'])),
                 a_imu=float(np.interp(now, ft, c['a_imu'])) if c.get('a_imu') is not None else np.nan,
                 plant_off=False, plant_a=np.nan, gap=row['gap_meas'], lead_v_true=np.nan,
                 sent_log=float(sent_log[i]))
    rows.append(row)
  keys = list(dict.fromkeys(k for r in rows for k in r))
  trace = {}
  for key in keys:
    col = [r.get(key) for r in rows]
    try:
      trace[key] = np.array([np.nan if x is None else x for x in col], dtype=float)
    except (TypeError, ValueError):
      trace[key] = np.array(col, dtype=object)
  if closed:
    trace['a_real'] = np.where(np.asarray(trace['closed'], bool), trace['a_real'], np.interp(trace['t'] + o, ft, c['a_real']))
  info.update(t_stop_rec=(c['t_stop'] - o) if c.get('t_stop') else None, kcs_rec=c.get('kcs'),
              grade_profile=profile is not None if closed else None)
  res = dict(info=info, trace=trace, metrics=metrics(trace, info, closed))
  if not closed:
    res['fidelity'] = fidelity(trace, res['metrics'])
  return res


# ---- metrics ----------------------------------------------------------------------------------------------------------
def _first(mask, start=0):
  k = np.flatnonzero(mask[start:])
  return int(k[0]) + start if len(k) else None


def metrics(tr, info, closed):
  """Stop metrics from the crossing (info['cross']) to the first driver intervention or the window end.
  Motion (a_*, j300, gaps, times) = the plant in closed mode, the LOGGED motion in recorded mode; wire metrics = the run."""
  t, v, wire, sent = tr['t'], tr['v_true'], tr['wire'], tr['sent']
  act = tr['active'].astype(bool)
  drv = (~act) | tr['gas'].astype(bool) | tr['brake'].astype(bool)
  k0 = int(np.searchsorted(t, info['cross']))
  k_end = _first(drv, k0) or len(t)
  phase = tr['phase']
  if closed:
    k_stop = _first((v <= 0.0) & (np.arange(len(t)) < k_end), k0)
    a01 = _trailing_mean(t, tr['a_real'], 0.1)
  else:
    ts = info.get('t_stop_rec')
    k_stop = int(np.searchsorted(t, ts)) if ts is not None and t[k0] <= ts <= t[k_end - 1] else None  # engaged stops only
    a01 = tr['a_real']
  m = dict(t_cross=info['cross'], t_driver_end=float(t[k_end]) if k_end < len(t) else None, complete=k_stop is not None)
  band = (np.arange(len(t)) >= k0) & (np.arange(len(t)) < (k_stop if k_stop is not None else k_end)) & (v < 2.5) & act
  if band.any():
    j = np.flatnonzero(band)[np.argmin(wire[band])]
    m.update(min_wire_band=float(wire[j]), t_min_wire_band=float(t[j]), min_sent_band=float(np.min(sent[band])))
  g = tr['gap']
  rng = slice(k0, k_end)
  m['min_gap'] = float(np.nanmin(g[rng])) if np.isfinite(g[rng]).any() else None
  m['guard_bound_frames'] = int(np.sum(tr['ff_added'][k0:k_stop or k_end] > 0))
  if k_stop is None:
    return m
  t_stop = float(t[k_stop])
  win = (t >= info['cross']) & (t <= t_stop + 0.3)
  q = np.arange(t_stop - 0.5, t_stop + 0.5, 0.01)

  def rise(x):  # max 300 ms rise of a 0.1 s-mean signal around the stop
    ok = np.isfinite(x)
    return float(np.max((np.interp(q + 0.3, t[ok], x[ok]) - np.interp(q, t[ok], x[ok])) / 0.3))
  qm = (t >= t_stop - 0.5) & (t <= t_stop + 1e-6) & np.isfinite(tr['x'])
  a_stop_wheel = float(2 * np.polyfit(t[qm] - t_stop, tr['x'][qm], 2)[0]) if qm.sum() >= 5 else None
  peak_wheel = float(np.nanmin(a01[win]))
  kcs = info.get('kcs_rec') or {}
  imu = (not closed) and 'a_imu' in tr and np.isfinite(tr['a_imu'][win]).any()
  if closed:  # the plant's wheel: 0.1 s mean before the stop; j300 = the rise of the 0.1 s mean (no body rebound)
    a_stop, peak, j300 = float(np.mean(tr['a_real'][max(k_stop - 10, 0):k_stop])), peak_wheel, rise(a01)
  elif imu:   # the logged body (felt): kcs1 terminal.metrics a_stop/j300 when available, IMU 0.1 s mean otherwise
    ai = tr['a_imu']
    a_stop = kcs['a_stop'] if kcs.get('a_stop') is not None else float(np.interp(t_stop, t, ai))
    peak = float(np.nanmin(ai[win]))
    j300 = kcs['j300'] if kcs.get('j300') is not None else rise(ai)
  else:
    a_stop, peak, j300 = a_stop_wheel, peak_wheel, None
  above = np.flatnonzero(v[:k_stop] >= 2.5)
  settled = (t >= t_stop + 0.5) & (t <= t_stop + 1.5) & (np.arange(len(t)) < k_end)
  m.update(t_stop=t_stop, peak_decel=peak, peak_decel_wheel=peak_wheel, a_stop=a_stop, a_stop_wheel=a_stop_wheel, j300=j300,
           j300_law=4.88 * abs(a_stop) + 0.11 if a_stop is not None else None,
           wire_at_stop=float(wire[k_stop]), sent_at_stop=float(sent[k_stop]), rest_gap=float(g[k_stop]),
           rest_gap_meas=float(tr['gap_meas'][k_stop]), gap_settled=float(np.nanmedian(g[settled])) if np.isfinite(g[settled]).any() else None,
           t_2p5_to_stop=float(t_stop - t[above[-1]]) if len(above) else None)
  k_rel = _first((phase == 'RELEASE') & (np.arange(len(t)) < k_end), k_stop)
  k_hold_end = k_rel if k_rel is not None else k_end
  hold = slice(k_stop, k_hold_end)
  if k_hold_end > k_stop:
    m.update(hold_min=float(np.min(wire[hold])), hold_s=float(t[k_hold_end - 1] - t_stop),
             hold_below_1p0_s=float(np.sum(wire[hold] < -1.0) * DT), hold_below_1p5_s=float(np.sum(wire[hold] < -1.5) * DT))
  k_mon = _first(tr['mon_trig'].astype(bool), k_stop)
  m['t_monitor_armed'] = float(t[k_mon]) if k_mon is not None and k_mon < k_end else None
  if k_rel is not None:
    k_done = _first((wire >= -0.005) | (phase == 'INACTIVE'), k_rel)
    m.update(release_start=float(t[k_rel]), release_from=float(wire[max(k_rel - 1, 0)]),
             release_end=float(t[k_done]) if k_done is not None and k_done < k_end else None,
             release_cut_by_driver=k_done is None or k_done >= k_end)
  return m


def fidelity(tr, m):
  """Replayed command vs the logged carControl accel (active frames up to the driver's intervention)."""
  t, err = tr['t'], np.abs(tr['wire'] - tr['logged'])
  act = tr['active'].astype(bool)
  end = m['t_driver_end'] if m.get('t_driver_end') is not None else t[-1] + 1
  out = {}
  wins = dict(all=(t[0] + 1.0, end), approach=(m['t_cross'], m.get('t_stop', end)),
              hold=(m.get('t_stop', end), m.get('release_start', end)), release=(m.get('release_start', end), end))
  for name, (a, b) in wins.items():
    sel = act & (t >= a) & (t < b)
    if sel.any():
      j = np.flatnonzero(sel)[np.argmax(err[sel])]
      out[name] = dict(n=int(sel.sum()), mae=float(err[sel].mean()), max=float(err[j]), t_max=float(t[j]),
                       n_gt_0p05=int(np.sum(err[sel] > 0.05)), n_exact=int(np.sum(err[sel] < 1e-6)))
  return out


# ---- synthetic stopped-lead approaches --------------------------------------------------------------------------------
def kinematic_planner(t, v, gap, v_lead, d_stop=6.0, a_min=-2.0):
  """UNVALIDATED planner proxy for synthetic cases: the constant decel that closes on the lead's speed d_stop behind it
  (aTarget = trajectory demand; no shouldStop). Use functools.partial to change d_stop / a_min (picklable)."""
  q = max(v - max(v_lead, 0.0), 0.0)
  a = max(-q * q / (2.0 * max(gap - d_stop, 0.3)), a_min)
  return dict(a_target=a, a_traj=a, should_stop=False, dts=-1.0)


def synthetic_case(v0=2.4, gap0=8.0, a0=-0.5, grade=0.0, lead='stopped', v_lead0=0.0, a_lead=-0.7, t_lead=1.0,
                   v_rev=-0.3, rev_dist=0.6, duration=12.0, planner=None, isd=0.3, cid=None):
  """A closed-loop-only stopped-lead approach (t = 0 at the start; the loop is closed from the first frame).
  lead: 'stopped' (at rest at gap0), 'crawl_stop' (v_lead0 > 0 until t_lead, then a_lead to rest), 'reverse'
  (at rest until t_lead, then rolls back at a_lead toward v_rev < 0 and stops again after rev_dist metres), or a
  callable t -> (x_lead, v_lead) with x_lead measured from the ego start. planner: None = no planner demand
  (a_target 0, no shouldStop/trajectory: nothing brakes before the service enters, so a MOVING lead is closed on
  unbraked) or a callable (t, v, gap, v_lead) -> dict(a_target, should_stop, dts, a_traj), evaluated every frame
  on what the controller sees (e.g. kinematic_planner). The car is the bookmark's logged CarParams; experimental on."""
  base = case('bk_222e_s4')
  t = np.arange(0.0, duration + 1e-9, DT)
  if callable(lead):
    xl, vl = map(np.asarray, zip(*[lead(x) for x in t], strict=True))
  else:
    vl = np.full(len(t), float(v_lead0) if lead == 'crawl_stop' else 0.0)
    k = t >= t_lead
    if lead == 'crawl_stop':
      vl[k] = np.maximum(v_lead0 + a_lead * (t[k] - t_lead), 0.0)
    elif lead == 'reverse':
      vl[k] = np.maximum(-abs(a_lead) * (t[k] - t_lead), v_rev)
      moved = np.cumsum(-vl * DT)
      vl[moved >= rev_dist] = 0.0
    elif lead != 'stopped':
      raise ValueError(lead)
    xl = gap0 + np.r_[0.0, np.cumsum((vl[:-1] + vl[1:]) / 2 * DT)]
  frames = []
  for i, now in enumerate(t):
    frames.append(dict(t=now, cs=dict(vEgo=v0, aEgo=a0, vEgoRaw=v0, standstill=False, cruiseState={'standstill': False},
                                      gasPressed=False, brakePressed=False, canValid=True, canTimeout=False, vCruise=30.0),
                       kw=dict(experimental_mode=True, lead_status=True, lead_v=float(vl[i]), lead_d_rel=float(xl[i]), lead_a=0.0,
                               lead_track_id=7, lead_model_prob=1.0, lead2_status=False, lead2_v=0.0, lead2_d_rel=0.0, fcw=False,
                               model_stop_d=-1.0, model_should_stop=False, force_coast=False, increased_stopped_distance=isd,
                               a_target_trajectory=None, freeze_integrator=False, plan_valid=True),
                       target=0.0, should_stop=False, dts=-1.0, active=True, recorded=a0, authorized=True))
  S = {k: np.zeros(0) for k in ('scc12__t', 'scc12__aReqValue', 'scc12__StopReq', 'scc14__t', 'scc14__JerkUpperLimit',
                                'scc14__JerkLowerLimit', 'pul__t', 'pul__count', 'car__t', 'car__standstill')}
  return dict(id=cid or f'syn_v{v0}_g{gap0}_a{a0}_{lead if isinstance(lead, str) else "fn"}', kind='synthetic', route='synthetic',
              cls='synthetic', note='synthetic: no planner unless given; motion from the plant only', origin=0.0, lo=0.0, hi=duration,
              entry_hint=None, paths=[], commit=base['commit'], settings=base['settings'], cp=base['cp'], frames=frames,
              radar=np.zeros((0, 6)), S=S, mpp=base['mpp'], x_true=v0 * t, v_true=np.full(len(t), v0), a_real=np.zeros(len(t)),
              t_flag=None, t_stop=None, cross=0.0, grade=float(grade), grade_profile=None, lead=(t, xl, vl), kcs=None, a_imu=None,
              planner=planner, entry_rec=0.0)


# ---- sweeps -----------------------------------------------------------------------------------------------------------
def _job(job):
  c, mode, variant, cell, opts = job
  try:
    return run(c, mode, variant=variant, cell=cell, **opts)
  except Exception as exc:  # noqa: BLE001 -- one failed case must not drop the sweep; the error is reported per job
    return dict(info=dict(id=c if isinstance(c, str) else c.get('id'), mode=mode, cell=cell), error=f'{type(exc).__name__}: {exc}')


def _pool(processes):
  """Use spawn on macOS: native library initialization after fork can abort or hang workers.
  Variants must be picklable module-level functions on macOS."""
  return mp.get_context('spawn' if sys.platform == 'darwin' else 'fork').Pool(processes, maxtasksperchild=8)  # bound worker memory


def build_all(ids=None, processes=8):
  """Build (or load) every CASES entry in parallel; returns {id: 'ok' | error}. Natural cases: natural_cases()."""
  ids = list(ids or CASES)
  with _pool(processes) as pool:
    return dict(zip(ids, pool.map(_build_one, ids, chunksize=1), strict=True))


def _build_one(cid):
  try:
    case(cid)
    return 'ok'
  except Exception as exc:  # noqa: BLE001 -- report which cases cannot be built and why
    return f'{type(exc).__name__}: {exc}'


def sweep(jobs, processes=8):
  """jobs: (case_id_or_dict, mode, variant, cell, opts) tuples; traces are dropped unless opts['keep_trace'].
  Returns the results in job order. See _pool() for the variant pickling rule."""
  jobs = [(c, m, v, cell, {'keep_trace': False, **(o or {})}) for c, m, v, cell, o in jobs]
  for c in {j[0] for j in jobs if isinstance(j[0], str)}:
    case(c)  # build caches (and the logged entry time) in the parent once
  if processes <= 1:
    return [_job(j) for j in jobs]
  with _pool(processes) as pool:
    return pool.map(_job, jobs, chunksize=1)
