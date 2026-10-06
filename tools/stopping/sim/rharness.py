# ruff: noqa: RUF100, ISC002, E501, C420, F401, UP034  (copied engine code kept verbatim; README.md)
"""Reactive closed-loop stopping harness (cycle 2026-10-03d). Read-only on the repo. Usage, validation and limits: RHARNESS.md.

  from openpilot.tools.stopping.sim import rharness as R   # sim package copy of harness_g/hg3 (README.md); the loader must be installed first
  r = R.run('s20')                                   # HEAD closed loop, default cell/start
  r = R.run('s20', variant=my_variant, cell='level0.47', radar_delay=0.15, start=1235.0)
  r = R.run('s20', start=None)                       # recorded reference (no plant; planner replayed in the background)
  r['trace'] (numpy columns per 10 ms frame), r['metrics'], r['info'], r['plans'] (per planner tick)
  res = R.sweep([('s20', None, 'level0.47', {}), ...], processes=8)   # spawn pool on macOS

Loop per 10 ms frame (times are route-relative s; every event time is the LOGGED one):
  1. radard ticks (logged radarState times): the lead track measurement is the plant truth delayed by LAG_D (dRel, 0.1 m LONG_DIST
     quantization) and LAG_V (vRel = REL_SPEED, 0.01 m/s), referenced to the liveTracks message radard used minus 5 ms (CAN);
     radard's own Track / KF1D computes vLead = vRel + v_ego_hist[0] (deque length round(radar_delay / 0.05) + 1, filled with the
     controller-visible vEgo of the carState radard used), vLeadK, aLeadK, aLeadTau.
  2. planner ticks (logged longitudinalPlan times): the real LongitudinalPlanner; inputs = the latest message <= modelMonoTime + 3 ms
     (modelV2 <= modelMonoTime), with carState (vEgo, aEgo, vEgoRaw, standstill) from the plant, radarState from step 1 and
     carControl.actuators.accel from the controller once closed; modelV2, frogpilotPlan, selfdriveState, liveParameters,
     frogpilotCarState, controlsState are replayed by time.
  3. LongControl + StoppingService (variant-C input timing: latest plan / radarState at or before the frame's selfdriveState).
  4. Hyundai CarController -> SCC12/SCC14 -> KCS plant (measured grade fraction 0.5 above 2.5 m/s, 0.3 below).
     harness_g: the plant's gear follows the logged CAN 0x240 gear (gear.GearState) and pushes in 2nd gear only in the creep regime
     of gear.CREEP (GEAR.md); gear='legacy' restores the original 1st-gear-below-1.6 m/s plant.
Before `start` everything that reaches the car is the log (controller on logged inputs, plant warmed on the logged SCC12);
the planner and the radar model already run (warm state), so a variant acts from `start` on.
"""
import bisect
import contextlib
import functools
import json
import math
import sys
from collections import deque
from pathlib import Path
from types import SimpleNamespace

sys.dont_write_bytecode = True
HERE = Path(__file__).resolve().parent
from openpilot.tools.stopping.sim import harness as H  # noqa: E402
from openpilot.tools.stopping.sim import loader  # noqa: E402   sim package: the source tree under test (replaces head_pin.py)
PINNED = loader.install_from_env()
from openpilot.tools.stopping.sim import gear as G  # noqa: E402   harness_g: the gear model (GEAR.md)

import numpy as np  # noqa: E402

PSERV = ('carControl', 'carState', 'controlsState', 'liveParameters', 'radarState', 'modelV2', 'selfdriveState', 'frogpilotCarState',
         'frogpilotPlan')
LAG_V, LAG_D, CAN_TO_LT = 0.15, 0.13, 0.005   # verify_impact.json: REL_SPEED 0.150 s, LONG_DIST 0.130 s vs pulses; liveTracks 5 ms after CAN
GRADE_FRAC = (0.5, 0.3)                        # grade_study_20261003: wheel shortfall 0.5 g grade above 2.5 m/s, 0.3 below
OBS_SCALE = 1.010                              # logged vEgo / pulse speed: 1.004-1.027 by band, median 1.010 on the 6 core/good cases
PLAN_INPUT_BOUND = 0.003                       # planner inputs: latest message <= modelMonoTime + 3 ms (preplay.py, 5e-4 replay)
DEFAULT_CELL = 'fit'   # FIT[case] when fitted, else GLOBAL_CELL
TOGGLES_DEFAULT = json.loads((HERE / 'toggles_default.json').read_text())

# ---- cases ------------------------------------------------------------------------------------------------------------
def _spec(route, t_ws, cls, note, pre=90.0, post=7.0, entry=None):
  """A stop case window: 90 s before the wheel stop (the LongControl trim integrator and the planner need the warm-up: from
  50 s before, s20 differed by up to 0.06 at 1233; from 90 s, 0.025 at 1235-1239.5 and 0.006 from 1239.5) to 7 s after."""
  return dict(route=route, lo=max(t_ws - pre, 5.0), hi=t_ws + post, cls=cls, entry=entry or t_ws - 2.0, note=note, trim_lo=True)


H.CASES.update({
  's20': _spec('00002231--73a1ec87ea', 1242.72, 'core', 'bookmark 2026-10-03: brake pump before stopping', entry=1240.79),
  's5': _spec('00002231--73a1ec87ea', 329.79, 'core', '2231 s5 HARSH, aim commit 327.66', entry=327.73),
  's22': _spec('00002231--73a1ec87ea', 1379.48, 'core', '2231 s22 early commit, no bite', entry=1376.65),
  '222e_s4': _spec('0000222e--f3c0c5bb86', 286.72, 'core', 'previous bookmark: plunge + hold', post=14.0, entry=285.752),
  '2226_s3': _spec('00002226--2f0fa69b0c', 203.61, 'good', '2026-10-02 good guarded stop'),
  '2226_s4': _spec('00002226--2f0fa69b0c', 255.46, 'good', '2026-10-02 good guarded stop'),
  '222d_s3': _spec('0000222d--aefebbe583', 232.98, 'good', '2026-10-02 good guarded stop'),
  '2086_s8': _spec('00002086--df4f36fce8', 524.19, 'creep53', 'cycle-53 creeping lead, double stop'),
  '2086_s17': _spec('00002086--df4f36fce8', 1045.92, 'creep53', 'cycle-53 creeping lead bookmark', entry=1044.6),
})
CORE = ('s20', 's5', 's22', '222e_s4')

# ---- the 2026-10-03 afternoon drives (car at 1e0327943d = HEAD code): the 30 engaged stops of the census -------------------
# (id, route, t_ws wheel stop, t_launch = first vEgo > 0.3 m/s after it, census felt, census rest, kind); route-relative s.
# Window: 90 s before the wheel stop (warm-up; covers the approach from >= 8 m/s, or the engaged follow phase of the slow
# stops) to 2 s after the launch. Order: the 2 bookmarks, the 7 high-felt stops, the rest by route/time. NEW_CASES.md.
NEW_STOPS = (
  ('2235_s55', '00002235--991a0dec17', 3310.68, 3339.52, 1.205, 4.19, 'BOOKMARK 3315.06: crawler stop, release + re-grab, creeping lead'),
  ('2235_s71', '00002235--991a0dec17', 4301.64, 4304.17, 0.733, 4.2, 'BOOKMARK 4302.73: entry bite on a negative radar vLead'),
  ('2232_s40', '00002232--2a250f66e2', 2422.59, 2426.8, 3.934, 5.7, 'high felt: slow crawl stop (vmax15 3.4)'),
  ('2232_s34', '00002232--2a250f66e2', 2069.14, 2085.72, 2.16, 4.3, 'high felt'),
  ('2232_s35', '00002232--2a250f66e2', 2143.67, 2144.77, 2.123, 4.3, 'high felt'),
  ('2235_s47', '00002235--991a0dec17', 2875.79, 2877.59, 2.12, 4.4, 'high felt'),
  ('2235_s51', '00002235--991a0dec17', 3106.94, 3118.77, 2.205, 4.19, 'high felt'),
  ('2234_s30', '00002234--2fcbccc784', 1838.3, 1843.64, 1.715, 4.39, 'high felt'),
  ('2232_s89', '00002232--2a250f66e2', 5352.71, 5355.28, 1.727, 4.4, 'high felt'),
  ('2232_s5', '00002232--2a250f66e2', 330.33, 371.55, 1.075, 5.38, 'slow stop (vmax15 4.1), engaged from 318.35'),
  ('2232_s37', '00002232--2a250f66e2', 2262.31, 2295.34, 0.879, 4.2, ''),
  ('2232_s82', '00002232--2a250f66e2', 4940.01, 4958.42, 0.899, 4.0, ''),
  ('2232_s84', '00002232--2a250f66e2', 5072.78, 5132.31, 0.754, 5.3, ''),
  ('2232_s86', '00002232--2a250f66e2', 5199.95, 5246.11, 1.14, 4.85, ''),
  ('2234_s1', '00002234--2fcbccc784', 98.1, 122.31, 0.782, 4.29, 'slow stop (vmax15 5.7), engaged from 89.92'),
  ('2234_s3', '00002234--2fcbccc784', 240.65, 248.79, 1.214, 4.2, ''),
  ('2234_s4', '00002234--2fcbccc784', 269.82, 337.74, 0.714, 4.46, ''),
  ('2234_s9', '00002234--2fcbccc784', 573.46, 596.67, 1.023, 4.48, 'engaged from 569.46 only'),
  ('2234_s10', '00002234--2fcbccc784', 633.48, 661.94, 1.284, 4.29, ''),
  ('2234_s11', '00002234--2fcbccc784', 701.09, 720.54, 0.942, 7.49, 'slow stop (vmax15 5.4), rest 7.5'),
  ('2234_s18', '00002234--2fcbccc784', 1093.15, 1127.7, 0.7, 4.7, ''),
  ('2235_s46', '00002235--991a0dec17', 2761.07, 2763.4, 0.572, 4.46, ''),
  ('2235_s56', '00002235--991a0dec17', 3371.39, 3422.83, 0.932, 4.59, ''),
  ('2235_s59', '00002235--991a0dec17', 3552.72, 3606.16, 1.198, 5.05, ''),
  ('2235_s62', '00002235--991a0dec17', 3738.81, 3773.75, 1.025, 4.3, ''),
  ('2235_s63', '00002235--991a0dec17', 3805.58, 3814.64, 1.009, 4.49, ''),
  ('2235_s64', '00002235--991a0dec17', 3852.0, 3852.28, None, 5.68, 'ROLLING (min 0.21 m/s at 3852.0, no wheel stop)'),
  ('2235_s65', '00002235--991a0dec17', 3910.5, 3915.62, 0.775, 4.68, ''),
  ('2235_s67', '00002235--991a0dec17', 4050.71, 4077.58, 1.25, 4.99, ''),
  ('2235_s77', '00002235--991a0dec17', 4623.38, 4642.83, 0.761, 6.4, ''),
)
for _id, _route, _tws, _tl, _felt, _rest, _note in NEW_STOPS:
  H.CASES[_id] = dict(_spec(_route, _tws, 'new', f'{_note} (census felt {_felt}, rest {_rest})'.lstrip(), post=_tl + 2.0 - _tws),
                      t_launch=_tl, t_ref=_tws if 'ROLLING' in _note else None)
NEW = tuple(x[0] for x in NEW_STOPS)
# Planner input bound per case (NEW_CASES.md section 6.1): on routes 00002232 / 00002234 the HEAD planner replay matches the logged
# aTarget far better with inputs <= modelMonoTime + 10 ms (exact share median 0.80 -> 1.00 / 0.85 -> 0.98, scratch/boundsweep.json);
# 00002235 and the original cases keep PLAN_INPUT_BOUND (3 ms). run(..., plan_bound=x) overrides.
PLAN_BOUND = {cid: 0.010 for cid in NEW if cid.startswith(('2232_', '2234_'))}
NEW_BOOKMARKS, NEW_HIGH_FELT = NEW[:2], NEW[2:9]


def _c(trigger='level', gain=0.0):
  from openpilot.tools.stopping.review.kcs_plant import Cell
  return Cell(trigger=trigger, threshold=-0.47 if trigger == 'level' else -0.42, gain_delta=gain)


# validation fit (RHARNESS.md): (Cell, grade fraction). Delivery varies stop to stop (gain_delta -0.10..+0.035 best per case);
# GLOBAL_CELL is the best single cell over the HEAD-era flat cases. 222e_s4 (-3.3 % downhill plunge) needs the history trigger
# with the full grade while the brake is off: the measured average fraction does not reproduce its collapse.
GLOBAL_CELL = (('level', -0.035), (0.5, 0.3))
FIT = {'s20': (('level', -0.07), (0.5, 0.3)), 's5': (('level', -0.035), (0.5, 0.3)), 's22': (('level', -0.035), (0.5, 0.3)),
       '222e_s4': (('history', -0.10), (0.5, 0.3, 1.0)), '2226_s3': (('level', -0.035), (0.5, 0.3)),
       '2226_s4': (('level', 0.035), (0.5, 0.3)), '222d_s3': (('level', -0.10), (0.5, 0.3))}
SPREAD = {'g0': (('level', 0.0), (0.5, 0.3)), 'g-0.035': (('level', -0.035), (0.5, 0.3)), 'g-0.07': (('level', -0.07), (0.5, 0.3))}
GOOD = ('2226_s3', '2226_s4', '222d_s3')
# NEW delivery fit (NEW_CASES.md section 3; scratch/fit_new.py): best wire + speed RMSE from start='auto' over the 24-cell grid of
# RHARNESS 4.3 (exact ties -> the measured average fraction); the 2232/2234 stops refitted with PLAN_BOUND (fit_new_b010_2232_2234.pkl).
# The other 21 NEW stops use GLOBAL_CELL.
NEW_FIT = {'2235_s55': (('history', -0.035), (0.5, 0.3)), '2235_s71': (('level', 0.035), (0.5, 0.3)),
           '2232_s40': (('history', 0.0), (0.5, 0.3, 1.0)), '2232_s34': (('history', -0.035), (0.5, 0.3, 1.0)),
           '2232_s35': (('history', 0.035), (0.5, 0.3)), '2235_s47': (('history', -0.035), (0.5, 0.3, 1.0)),
           '2235_s51': (('level', 0.0), (0.5, 0.3)), '2234_s30': (('level', -0.07), (0.5, 0.3)),
           '2232_s89': (('level', -0.07), (0.5, 0.3))}
FIT.update(NEW_FIT)
HISTORY = Path.home() / '.route_sync/work/cyc_1003/history'  # persistent (/tmp is wiped at a restart)


def history_cases(kind='aim', n=30, seed=0):
  """tools/history band stops as case ids (registered in H.CASES): kind 'aim' = the 24 aim-bite stops (aim_bite_stops.csv),
  'nobite' = a seeded random sample of n band stops with neither an aim bite nor a recorded service-entry bite >= 0.2."""
  stops = json.loads((HISTORY / 'stops.json').read_text())
  test = ('0000212e', '0000212f', '000021ef', '000021f0', '000021f9', '000021fa')
  band = [s for s in stops if s['kind'] == 'stop' and s['lead_at_stop'] and s['rest'] is not None and s['rest'] < 15
          and s['vmax15'] >= 4.0 and not s['route'].startswith(test)]
  aim = {(r.split(',')[0].split()[0], round(float(r.split(',')[0].split()[2]), 1))
         for r in (HISTORY / 'aim_bite_stops.csv').read_text().splitlines()[1:]}
  key = lambda s: (s['route'][:8], round(s['t_ws'], 1))  # noqa: E731
  if kind == 'aim':
    sel = [s for s in band if key(s) in aim]
  else:
    pool = [s for s in band if key(s) not in aim and not any(x['step'] <= -0.2 for x in s.get('csteps', []))]
    sel = [pool[i] for i in sorted(np.random.default_rng(seed).choice(len(pool), size=min(n, len(pool)), replace=False))]
  ids = []
  for s in sel:
    cid = f"h_{s['route'][4:8]}_{s['t_ws']:.1f}"
    H.CASES.setdefault(cid, _spec(s['route'], s['t_ws'], 'hist_' + kind, f"history band stop felt {s['felt']} rest {s['rest']} commit {s['commit'][:1]}"))
    ids.append(cid)
  return ids


# ---- logged event streams (per process memo) ----------------------------------------------------------------------------
_EV: dict = {}
EV_KEEP = 3   # event streams kept per process (memory bound; sweep workers recycle the oldest)


def _stream(c):
  """Planner inputs, longitudinalPlan, radarState lead records and the liveTracks radard used, for the case window."""
  if c['id'] in _EV:
    return _EV[c['id']]
  from cereal import car
  o = c['origin']
  lo, hi = c['lo'] - 3.0, c['hi'] + 1.0
  want = set(PSERV) | {'longitudinalPlan', 'liveTracks', 'carParams'}
  by = {s: ([], []) for s in want}
  for p in c['paths']:
    for e in H._events(p):
      w = e.which()
      if w in want and (w == 'carParams' or lo <= e.logMonoTime * 1e-9 <= hi):
        by[w][0].append(e.logMonoTime)
        # Validate/copy once: repeated replay reads must not exhaust the serialized reader budget.
        by[w][1].append(e.as_builder().as_reader())
  for s in want:
    order = np.argsort(by[s][0], kind='stable')
    by[s] = (np.array(by[s][0], dtype=np.int64)[order], [by[s][1][i] for i in order])
  cp = by['carParams'][1][0].carParams if by['carParams'][1] else car.CarParams.from_bytes(c['cp']).__enter__()
  lt_ns, lt_ev = by['liveTracks']
  lt_pts = [{p.trackId: p.vRel for p in e.liveTracks.points} for e in lt_ev]
  rs = []
  for e in by['radarState'][1]:
    r = e.radarState
    a, b = r.leadOne, r.leadTwo
    lt = None
    if a.status and a.radar:  # the liveTracks message radard used for the lead: latest before, else one back (core.py rule)
      j = int(np.searchsorted(lt_ns, e.logMonoTime)) - 1
      for jj in (j, j - 1):
        if jj >= 0 and abs(lt_pts[jj].get(a.radarTrackId, 1e9) - a.vRel) < 1e-4:
          lt = lt_ns[jj] * 1e-9
          break
    rs.append(dict(t=e.logMonoTime * 1e-9, cs=r.carStateMonoTime * 1e-9, lt=lt, st=bool(a.status), radar=bool(a.radar),
                   tid=int(a.radarTrackId), d=float(a.dRel), vrel=float(a.vRel), vl=float(a.vLead), vlk=float(a.vLeadK),
                   alk=float(a.aLeadK), tau=float(a.aLeadTau), st2=bool(b.status), d2=float(b.dRel)))
  E = dict(by=by, cp=cp, rs=rs, t_rs=np.array([r['t'] for r in rs]),
           t_plan=by['longitudinalPlan'][0] * 1e-9, md_plan=np.array([e.longitudinalPlan.modelMonoTime for e in by['longitudinalPlan'][1]]) * 1e-9,
           cs_t=by['carState'][0] * 1e-9, cs_v=np.array([e.carState.vEgo for e in by['carState'][1]]),
           sds_t=by['selfdriveState'][0] * 1e-9)
  # lead truth (route clock): the radar measurement taken at lt - CAN_TO_LT - LAG is the truth there
  ft = np.array([f['t'] for f in c['frames']])
  for r in rs:
    if r['lt'] is not None:
      td, tv = r['lt'] - CAN_TO_LT - LAG_D, r['lt'] - CAN_TO_LT - LAG_V
      r['xl'] = float(np.interp(td, ft, c['x_true'])) + r['d']
      r['vlt'] = r['vrel'] + float(np.interp(tv, ft, c['v_true']))
  E['o'] = o
  while len(_EV) >= EV_KEEP:  # ~0.3 GB per case: an unbounded cache took 3 parallel sweeps past 64 GB (2 kernel panics, 2026-10-03)
    _EV.pop(next(iter(_EV)))
  _EV[c['id']] = E
  return E


def _toggles(js, memo={}):  # noqa: B006 -- per-process cache, as process_frogpilot_toggles
  if js not in memo:
    d = dict(TOGGLES_DEFAULT)
    d.update(json.loads(js) if js else {})
    memo[js] = SimpleNamespace(**d)
  return memo[js]


class _SM:
  """SubMaster stand-in for LongitudinalPlanner.update/publish."""
  def __init__(self, msgs, prev):
    self.msgs = msgs
    self.logMonoTime = {s: msgs[s].logMonoTime for s in msgs}
    self.updated = {s: prev is None or prev.get(s) != msgs[s].logMonoTime for s in msgs}
    self.valid = {s: msgs[s].valid for s in msgs}
    self.alive = {s: True for s in msgs}

  def __getitem__(self, s):
    return getattr(self.msgs[s], s)

  def all_checks(self, service_list=None):
    return all(self.valid[s] for s in (service_list or self.msgs))


class _PM:
  last: object = None

  def send(self, s, msg):
    self.last = msg


# planner instrumentation: the stop-aim floor value and commit flag of each tick (pure pass-through)
_AIM: list = []


def _instrument():
  import openpilot.selfdrive.controls.lib.longitudinal_planner as LP
  if getattr(LP.get_santa_fe_stop_floor_demands, '_rh', False):
    return LP
  orig = LP.get_santa_fe_stop_floor_demands

  def wrapped(*a, **k):
    out = orig(*a, **k)
    _AIM.append((out[0], bool(out[1]), float(a[2])))
    return out
  wrapped._rh = True
  LP.get_santa_fe_stop_floor_demands = wrapped
  return LP


@functools.cache
def plant_class(frac=GRADE_FRAC, fade=None, creep=None):
  """kcs_plant.Plant + the harness monotone post-stop observation floor + the measured grade fraction (frac[0] at v >= 2.5,
  frac[1] below) + optional regen fade: fade = (v_lo, v_hi, depth) adds +depth m/s^2 of lost decel, a half-sine bump over
  v_lo..v_hi, while the command is <= -0.3 (UNVALIDATED option, see RHARNESS.md).
  harness_g: creep = sorted gear.CREEP items -> the gear model (GEAR.md): the 1st-gear state follows plant.gear_now (set by the run
  loop from gear.GearState each frame) instead of v < 1.6; the KCS shift dip only on a downshift at >= 1.0 m/s (the 3->1 hot
  arrival the plant was identified on); in 2nd gear the brake-off push is creep2() instead of .12. creep=None: the original plant
  (bit-identical)."""
  from openpilot.tools.stopping.review import kcs_plant
  from openpilot.tools.stopping.review.kcs_plant import Plant
  kcs_plant.OBS_SCALE = OBS_SCALE   # read by the patched step (compiled in kcs_plant's globals)
  grade = '- 9.81 * self.grade / 100 * (self.frac[2] if self.off and len(self.frac) > 2 else self.frac[0 if self.v >= 2.5 else 1])'
  if fade:
    grade += (' + (self.fade[2] * math.sin(math.pi * (self.v - self.fade[0]) / (self.fade[1] - self.fade[0]))'
              ' if r <= -.3 and self.fade[0] < self.v < self.fade[1] else 0.)')
  step = H.src_patch(Plant, 'step', ('    self.tail_t = self.t\n', '    self.tail_t, self._floor = self.t, .10\n'),
                     ('    raw = max(raw, .10 / (1 + since / .38), .0087)',
                      '    self._floor = min(self._floor, .10 / (1 + since / .38))\n    raw = max(raw, self._floor, .0087)'),
                     ('- 9.81 * self.grade / 100', grade),
                     ('raw = self.speed_history[-delay_frames - 1]\n', 'raw = self.speed_history[-delay_frames - 1] * OBS_SCALE\n'),
                     *((('  if not self.gear1 and self.v < 1.6:\n    self.gear1 = True\n    if self.off:\n',
                         '  if self.gear1 and self.gear_now != 1:\n    self.gear1 = False\n'
                         '  if not self.gear1 and self.gear_now == 1:\n    self.gear1 = True\n    if self.off and self.v >= 1.0:\n'),
                        ('.30 + .42 * (1.27 - self.v))) if self.gear1 else .12', '.30 + .42 * (1.27 - self.v))) if self.gear1 else self.creep2(wire)'))
                       if creep else ()))
  if not creep:
    return type('RPlant', (Plant,), dict(step=step, frac=frac, fade=fade))
  K = dict(creep)
  n_lag = int(round(K['lag'] / kcs_plant.DT)) + 1

  def __init__(self, v, a, wire, *args, **kw):
    Plant.__init__(self, v, a, wire, *args, **kw)
    self.gear_now = 1 if v < G.V21 else 2      # the run loop sets it from GearState before every step
    self.gear1 = self.gear_now == 1
    self._lagq = deque([wire] * n_lag, maxlen=n_lag)
    self._dwell, self._eng, self._q = 0.0, False, 0.0

  def creep2(self, wire):
    """2nd-gear push (m/s^2): the KCS plant's brake-off loss target in 2nd gear (the plant applies it only in its brake-off regime,
    with the KCS onset and lag): max(creep, p2_hi at v >= v_p2 (the KCS 'P=.12 above shift') else p2_lo) plus off_grade x the grade
    share the case fraction leaves uncompensated (a released brake compensates no grade), within +-p_max (the KCS push cap). creep =
    k_q x the 0x472 creep torque model of gear.CREEP. A case frac with its own brake-off value (len 3) keeps it and gets no extra.
    Called every step (keeps the creep state)."""
    self._lagq.append(wire)
    lag = self._lagq[0]
    if self.gear1:
      self._dwell, self._eng = 0.0, False
    else:
      self._dwell = self._dwell + kcs_plant.DT if lag >= K['thr'] and self.v <= K['v_hi'] else 0.0
      if not self._eng and self._dwell >= K['dwell'] - 1e-9 and K['v_lo'] <= self.v <= K['v_hi']:
        self._eng = True
      elif self._eng and (lag < K['thr'] or self.v < K['v_cut']):
        self._eng = False
    tgt = max(K['qc'] * (1.0 + K['vslope'] * (1.0 - self.v)), 0.0) if self._eng else 0.0
    self._q += (tgt - self._q) * -math.expm1(-kcs_plant.DT / (K['tau_r'] if tgt > self._q else K['tau_d']))
    push = max(K['k_q'] * self._q, K['p2_hi'] if self.v >= K['v_p2'] else K['p2_lo'])
    if K['off_grade'] and len(self.frac) == 2:
      push -= K['off_grade'] * (1.0 - self.frac[0 if self.v >= 2.5 else 1]) * 9.81 * self.grade / 100
    return max(-self.cell.p_max, min(self.cell.p_max, push))
  return type('RPlantG', (Plant,), dict(step=step, frac=frac, fade=fade, __init__=__init__, creep2=creep2))


def _cell(cell):
  from openpilot.tools.stopping.review.kcs_plant import cells
  if not isinstance(cell, str):
    return cell
  return cells()[cell] if cell in cells() else cells(True)[cell]


def resolve_start(c, start):
  """'auto' = the last logged vEgo >= 6 m/s before the stop, clipped to [stop - 10 s, 2.5 m/s crossing - 1 s] and >= lo + 12 s
  (planner warm-up); 'vX' = the last vEgo >= X before the stop; a number = route-relative time; None = no takeover."""
  if start is None or c['kind'] == 'synthetic':
    return None if start is None else 0.0
  if not isinstance(start, str):
    return float(start)
  o = c['origin']
  ft = np.array([f['t'] for f in c['frames']])
  v = np.array([f['cs']['vEgo'] for f in c['frames']])
  ts = c.get('t_stop') or c.get('entry_hint') or c['hi'] - 4.0
  k_stop = int(np.searchsorted(ft, ts))
  vx = 6.0 if start == 'auto' else float(start[1:])
  above = np.flatnonzero(v[:k_stop] >= vx)
  t = ft[above[-1]] if len(above) else ft[0]
  if start == 'auto':
    t = min(max(t, ts - 10.0), c['cross'] - 1.0)
  drv = [k for k in range(k_stop) if not c['frames'][k]['active'] or c['frames'][k]['cs']['gasPressed'] or c['frames'][k]['cs']['brakePressed']]
  if drv:  # the final engaged approach: after the last driver intervention before the stop
    t = max(t, ft[drv[-1]] + 0.5)
  return float(max(t, c['lo'] + 12.0)) - o


# ---- synthetic scenarios (real planner on donor messages + a proxy e2e model; UNVALIDATED proxy, see RHARNESS.md) ----------
SYNTH = {
  'syn_stop8': dict(v0=8.0, gap0=40.0, lead='stopped'),
  'syn_stop12': dict(v0=12.0, gap0=65.0, lead='stopped'),
  'syn_crawl_stop': dict(v0=6.0, gap0=25.0, lead='crawl_stop', v_lead0=1.0, t_lead=5.0, a_lead=-0.5),
  'syn_crawl': dict(v0=6.0, gap0=25.0, lead='crawl', v_lead0=0.8, duration=30.0),
  'syn_reverse': dict(v0=8.0, gap0=40.0, lead='reverse', v_trig=1.6, v_rev=-0.3, rev_dist=0.9),
  'syn_launch': dict(v0=8.0, gap0=40.0, lead='launch', t_go=3.0, a_go=1.5, v_go=8.0, duration=30.0),
}
DONOR = ('s20', 1236.0)   # planner message templates: 2231 s20 approach (experimental, CEM on, stopped lead ahead)
SYN_T0 = 1000.0           # synthetic clock origin (logMonoTime is unsigned); route-relative time = scenario time


class Lead:
  """Synthetic lead (absolute position, ego starts at x = 0), stepped with the ego each 10 ms frame. Modes: stopped,
  crawl (v_lead0 for ever), crawl_stop (v_lead0 until t_lead, then a_lead to rest), reverse (stopped; when the ego reads
  <= v_trig it rolls back toward v_rev at 1 m/s^2 and stops after rev_dist, the 2026-10-02 plan reviewer's case),
  launch (stopped; t_go s after the ego stops it departs at a_go up to v_go)."""
  def __init__(self, gap0, lead='stopped', v_lead0=0.0, t_lead=5.0, a_lead=-0.5, v_trig=1.6, v_rev=-0.3, rev_dist=0.9, t_go=3.0,
               a_go=1.5, v_go=8.0, **_):
    self.mode, self.v_lead0, self.t_lead, self.a_lead, self.v_trig, self.v_rev, self.rev_dist = lead, v_lead0, t_lead, a_lead, v_trig, v_rev, rev_dist
    self.t_go, self.a_go, self.v_go = t_go, a_go, v_go
    v = v_lead0 if lead in ('crawl', 'crawl_stop') else 0.0
    self.t, self.x, self.v, self.a = [SYN_T0 - 1.0], [gap0 - v], [v], [0.0]
    self.trig, self.x_trig, self.t_stopped = None, None, None

  def step(self, t, ego_v):
    dt, v, x = t - self.t[-1], self.v[-1], self.x[-1]
    if dt <= 0:
      return
    m = self.mode
    if m == 'crawl_stop' and t - SYN_T0 >= self.t_lead:
      v = max(v + self.a_lead * dt, 0.0)
    elif m == 'reverse':
      if self.trig is None and ego_v <= self.v_trig:
        self.trig, self.x_trig = t, x
      if self.trig is not None:
        back = self.x_trig - x
        v = max(v - dt, self.v_rev) if back < self.rev_dist else min(v + dt, 0.0)
    elif m == 'launch':
      self.t_stopped = (self.t_stopped or t) if ego_v <= 0.01 else None
      if self.trig is None and self.t_stopped is not None and t - self.t_stopped >= self.t_go:
        self.trig = t
      if self.trig is not None:
        v = min(v + self.a_go * dt, self.v_go)
    a = (v - self.v[-1]) / dt
    self.t.append(t)
    self.x.append(x + (v + self.v[-1]) / 2 * dt)
    self.v.append(v)
    self.a.append(a)

  def x_at(self, t):
    return float(np.interp(t, self.t, self.x))

  def v_at(self, t):
    return float(np.interp(t, self.t, self.v))

  def a_at(self, t):
    return float(np.interp(t, self.t, self.a))


def e2e_proxy(v, d, vl, al):
  """UNVALIDATED stand-in for the e2e model's desiredAcceleration: rest 3.5 m behind a stopped/stopping lead (the logged
  e2e rested ~3.4-3.5 m behind, cycle-1003 pump analysis), follow a moving one at 3.5 + 1.0 vLead, go when it departs."""
  vl = max(vl, 0.0)
  stopping = vl < 0.3 or al < -0.1
  if stopping:
    d_rem = d + (vl * vl / (2.0 * max(-al, 0.3)) if vl >= 0.3 else 0.0) - 3.5
    a = -v * v / (2.0 * max(d_rem, 0.3))
  else:
    d_rem = d - (3.5 + 1.0 * vl)
    dv = v - vl
    a = (-dv * dv / (2.0 * max(d_rem, 0.3)) if dv > 0 else min(0.4 * d_rem, 1.5)) + 0.5 * min(d_rem, 0.0)   # inside: + 0.5 per metre
  return float(np.clip(a, -3.0, 1.5))


_DONOR: dict = {}


def _donor():
  if not _DONOR:
    c = H.case(DONOR[0])
    E = _stream(c)
    t = c['origin'] + DONOR[1]
    for s in PSERV + ('longitudinalPlan',):
      ns, evs = E['by'][s]
      _DONOR[s] = evs[int(np.searchsorted(ns, int(t * 1e9))) - 1]
    _DONOR['cp'] = E['cp']
    _DONOR['case'] = c
  return _DONOR


def synthetic_case(name=None, v0=8.0, gap0=40.0, a0=0.0, grade=0.0, duration=20.0, **lead_kw):
  """A closed-loop-only scenario (closed from t = 0; route clock = scenario clock). Timing as logged: model 20 Hz, radarState
  14 ms after the model, the planner 11.5 ms after it, liveTracks 27 ms before the radarState, controller 100 Hz."""
  from openpilot.selfdrive.modeld.constants import ModelConstants
  params = dict(SYNTH[name]) if name in SYNTH else {}
  params.update(dict(v0=v0, gap0=gap0, a0=a0, grade=grade, duration=duration, **lead_kw) if name not in SYNTH else {})
  v0, gap0, a0, grade, duration = (params.pop(k, d) for k, d in (('v0', 8.0), ('gap0', 40.0), ('a0', 0.0), ('grade', 0.0), ('duration', 20.0)))
  D = _donor()
  base = D['case']
  lead = Lead(gap0, **params)
  t = SYN_T0 + np.arange(0.0, duration + 1e-9, H.DT)
  cs = dict(vEgo=v0, aEgo=a0, vEgoRaw=v0, standstill=False, cruiseState={'standstill': False}, gasPressed=False, brakePressed=False,
            canValid=True, canTimeout=False, vCruise=float(D['carState'].carState.vCruise))
  kw = dict(experimental_mode=True, lead_status=True, lead_v=0.0, lead_d_rel=gap0, lead_a=0.0, lead_track_id=7, lead_model_prob=1.0,
            lead2_status=False, lead2_v=0.0, lead2_d_rel=0.0, fcw=False, model_stop_d=-1.0, model_should_stop=False, force_coast=False,
            increased_stopped_distance=0.3, a_target_trajectory=None, freeze_integrator=False, plan_valid=True)
  frames = [dict(t=float(x), cs=cs, kw=kw, target=0.0, should_stop=False, dts=-1.0, active=True, recorded=a0, authorized=True) for x in t]
  md = SYN_T0 + np.arange(-0.15, duration + 0.1, 0.05)
  t_rs = md + 0.014
  rs = [dict(t=float(x), cs=float(x) - 0.010, lt=float(x) - 0.027, st=True, radar=True, tid=7, d=gap0, vrel=-v0, vl=0.0, vlk=0.0, alk=0.0,
             tau=1.5, st2=False, d2=0.0, v_cs=v0) for x in t_rs]
  by = {s: (np.array([-10**12], dtype=np.int64), [D[s]]) for s in PSERV}

  def synth_msgs(md_t, msgs, ego_v, ego_a, gap):
    ns = int(round(md_t * 1e9))
    out = {}
    for s, m in msgs.items():
      b = m.as_builder()
      b.logMonoTime = ns - (0 if s == 'modelV2' else 5_000_000)
      out[s] = b
    c_ = out['carState'].carState
    c_.vEgo, c_.aEgo, c_.vEgoRaw, c_.standstill = float(ego_v), float(ego_a), float(ego_v), bool(ego_v <= 0.0)
    c_.gasPressed, c_.brakePressed = False, False
    vl, al = lead.v_at(md_t), lead.a_at(md_t)
    a = e2e_proxy(ego_v, gap, vl, al)
    T = np.asarray(ModelConstants.T_IDXS)
    vv = np.maximum(ego_v + a * T, 0.0)
    xx = np.r_[0.0, np.cumsum((vv[1:] + vv[:-1]) / 2 * np.diff(T))]
    m = out['modelV2'].modelV2
    m.position.x, m.velocity.x, m.acceleration.x = xx.tolist(), vv.tolist(), np.where(vv > 0, a, 0.0).tolist()
    m.action.desiredAcceleration, m.action.shouldStop = a, False
    n_gp = len(m.meta.disengagePredictions.gasPressProbs)   # the planner allows throttle only on a confident 'go' (logged launch: 0.7-0.98)
    m.meta.disengagePredictions.gasPressProbs = [0.9 if a > 0.2 else 0.02] * n_gp
    LT = np.asarray(ModelConstants.LEAD_T_IDXS)
    lv = vl + al * LT if al >= 0 or vl <= 0 else np.maximum(vl + al * LT, 0.0)
    lx = gap + 1.52 + np.r_[0.0, np.cumsum((lv[1:] + lv[:-1]) / 2 * np.diff(LT))]
    for i, L in enumerate(m.leadsV3):
      L.prob = 0.99 if i == 0 else 0.0
      if i == 0:
        L.x, L.v, L.a = lx.tolist(), lv.tolist(), [al] * len(LT)
    out['radarState'].radarState.leadTwo.status = False
    out['frogpilotCarState'].frogpilotCarState.forceCoast = False
    return out

  E = dict(by=by, cp=D['cp'], rs=rs, t_rs=t_rs, t_plan=md + 0.0115, md_plan=md, cs_t=t, cs_v=np.full(len(t), v0), sds_t=t - 0.001,
           o=SYN_T0, synth_msgs=synth_msgs)
  S = {k: np.zeros(0) for k in ('scc12__t', 'scc12__aReqValue', 'scc12__StopReq', 'scc14__t', 'scc14__JerkUpperLimit', 'scc14__JerkLowerLimit')}
  return dict(id=name or 'syn', kind='synthetic', route='synthetic', cls='synthetic', note=str(SYNTH.get(name, params)), origin=SYN_T0,
              lo=SYN_T0, hi=SYN_T0 + duration, entry_hint=None, paths=[], commit=base['commit'], settings=base['settings'], cp=base['cp'], frames=frames, S=S,
              x_true=v0 * (t - SYN_T0), v_true=np.full(len(t), v0), a_real=np.zeros(len(t)), t_stop=None, cross=0.0, grade=float(grade),
              grade_profile=None, kcs=None, a_imu=None, stream=E, frame_sds=t - 0.001, lead=lead, planner_start=-0.09)


# ---- the run ----------------------------------------------------------------------------------------------------------
def run(case, variant=None, cell=DEFAULT_CELL, radar_delay=0.0, start='auto', **opts):
  """One run. case: id or case dict (synthetic_case()). variant: zero-arg callable -> context manager (H.patched / H.src_patch),
  entered for the whole run. cell: KCS cell name or Cell. radar_delay: radard CP.radarDelay (s). start: takeover (see resolve_start).
  opts: lag_v, lag_d (radar lags, s), gap_q (0.1), frac (grade fraction pair), fade (regen-fade tuple), grade (constant override, %),
  t_end (route-relative), flags (stopping_flags overrides), keep_trace (True), warm (2.1 s plant warm-up),
  cruise_standstill ('plant' default = the original harness; 'car' = False as this car publishes it: use for hold/launch claims),
  plan_bound (planner input bound, s; default PLAN_BOUND[case] else PLAN_INPUT_BOUND).
  harness_g (GEAR.md): gear ('log' default for rlog cases, 'model' default for synthetic, 'log_time', '1st', a shift speed in m/s,
  or 'legacy' = the original plant), creep (dict overrides of gear.CREEP, e.g. dict(k_q=0) = no 2nd-gear creep push),
  (gear.CREEP['off_grade'] = the share of the uncompensated grade added to the 2nd-gear brake-off push, default 1.0; 0 = none).
  cyc_1004 standstill (sim/gated.py): standstill None (the original plant, UNVALIDATED at rest), 'gate' (the
  measured StopReq / positive-command release gate; required for any hold, release or creep-follow claim), 'gate+creep1' (gate +
  the 1st-gear creep push under a held brake; re-stop sensitivity cell), 'creep_at_rest' (the gate opens on the StopReq drop
  alone: the unmeasured StopReq=0 / command<=0 rest regime as creep; worst-case cell)."""
  from openpilot.selfdrive.controls.lib import stopping_flags
  _register(case)
  c = synthetic_case(case) if isinstance(case, str) and case in SYNTH else H.case(case)
  if isinstance(cell, str) and (cell == 'fit' or cell in SPREAD):
    (trig, gain), frac = SPREAD[cell] if cell in SPREAD else FIT.get(c['id'], GLOBAL_CELL)
    cell = _c(trig, gain)
    opts.setdefault('frac', frac)
  ctx = variant() if variant is not None else contextlib.nullcontext()
  fl = H.patched(*[(stopping_flags, k, v) for k, v in (opts.get('flags') or {}).items()])
  with ctx, fl, H.patched((stopping_flags, 'IDENTIFICATION_HOOK', False)):
    out = _run(c, cell, radar_delay, resolve_start(c, start), opts)
  out['metrics'] = metrics(out['trace'], out['info'])
  if not opts.get('keep_trace', True):
    out.pop('trace')
    out.pop('plans')
  return out


def _hist_at(ts, xs, t):
  i = bisect.bisect_right(ts, t + 1e-9) - 1
  return xs[max(i, 0)]


def _run(c, cell, radar_delay, start, opts):
  from openpilot.selfdrive.controls import radard
  from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.stop_target_helpers import get_published_lead_distance
  from openpilot.tools.stopping.review.kcs_plant import GAIN
  LP = _instrument()
  synthetic = c['kind'] == 'synthetic'
  E = c['stream'] if synthetic else _stream(c)
  o = c['origin']
  frames = c['frames']
  ft = np.array([f['t'] for f in frames])
  cp, lc, toggles_lc, CI, sender, run_frame, get_signal = H._setup(c)
  planner = LP.LongitudinalPlanner(E['cp'])
  svc = lc._service_shadow_svc
  inner, box = svc.update, {}

  def capture(**kw):
    r = inner(**kw)
    box['dbg'], box['svc_active'] = r.debug, r.active
    return r
  svc.update = capture
  S = c['S']
  closed = start is not None
  t0 = o + start if closed else np.inf
  t_end = o + opts['t_end'] if opts.get('t_end') is not None else c['hi']
  lag_v, lag_d, gap_q = opts.get('lag_v', LAG_V), opts.get('lag_d', LAG_D), opts.get('gap_q', 0.1)
  frac, fade = tuple(opts.get('frac', GRADE_FRAC)), opts.get('fade')
  gmode = opts.get('gear', 'model' if synthetic else 'log')   # harness_g: the gear model (GEAR.md); 'legacy' = the original plant
  creep = None if gmode in (None, 'legacy') else tuple(sorted({**G.CREEP, **(opts.get('creep') or {})}.items()))
  if closed:
    warm = 0.0 if synthetic else float(opts.get('warm', 2.1))
    seed_t = t0 - warm
    if seed_t < ft[0] - 1e-6:
      raise ValueError(f'{c["id"]}: start {start:.2f} leaves < {warm} s of plant warm-up in the window')
    pre = ft[ft < seed_t - 1e-9]
    grid = np.r_[pre, np.arange(seed_t if len(pre) == 0 else max(seed_t, pre[-1] + H.DT), t_end + 1e-9, H.DT)]
    cobj = _cell(cell)
    grade = float(opts.get('grade', c['grade']))
    profile = c.get('grade_profile') if opts.get('grade') is None else None
  else:
    grid = ft[ft <= t_end + 1e-9]
  idx = np.clip(np.searchsorted(ft, grid, side='right') - 1, 0, len(ft) - 1)
  frame_sds = c['frame_sds'] if synthetic else _frame_sds(c, E)

  # motion / observation history (route clock): controller-visible carState and the plant truth
  hist = dict(t=[], v_obs=[], a_obs=[], raw=[], ss=[], x=[], v=[], out=[])
  x_truth = lambda t: float(np.interp(t, ft, c['x_true']))  # noqa: E731
  v_truth = lambda t: float(np.interp(t, ft, c['v_true']))  # noqa: E731

  def ego_x(t):
    return x_truth(t) if t < t0 or not hist['t'] else x0 + float(np.interp(t, hist['t'], hist['x']))

  def ego_v(t):
    return v_truth(t) if t < t0 or not hist['t'] else float(np.interp(t, hist['t'], hist['v']))

  def obs(t):  # the carState published at t (plant observation once closed)
    if t < t0 or not hist['t']:
      return None
    i = max(bisect.bisect_right(hist['t'], t + 1e-9) - 1, 0)
    return hist['v_obs'][i], hist['a_obs'][i], hist['raw'][i], hist['ss'][i]

  # ---- radar model (radard Track/KF on the modelled lead measurement) ----
  rs_log, t_rs = E['rs'], E['t_rs']
  rs_out = []          # synthesized radarState records, in logged order
  rad = dict(k=0, hist=deque([0.0], maxlen=int(round(radar_delay / 0.05)) + 1), last_cs=None, track=None, tid=None, prev_radar=False)
  kp = radard.KalmanParams(0.05)

  def radar_tick(k):
    r = rs_log[k]
    ob = obs(r['cs'])
    v_used = ob[0] if ob is not None else (r['v_cs'] if synthetic else float(np.interp(r['cs'], E['cs_t'], E['cs_v'])))
    if r['cs'] != rad['last_cs']:
      rad['hist'].append(v_used)
      rad['last_cs'] = r['cs']
    out = dict(t=r['t'], st=r['st'], d=r['d'], vrel=r['vrel'], vl=r['vl'], vlk=r['vlk'], alk=r['alk'], tau=r['tau'], d2=r['d2'],
               model=False, gap_true=np.nan, vl_true=np.nan)
    if r['st'] and r['radar'] and r['lt'] is not None:
      td, tv = r['lt'] - CAN_TO_LT - lag_d, r['lt'] - CAN_TO_LT - lag_v
      xl, vlt = (c['lead'].x_at(td), c['lead'].v_at(tv)) if synthetic else (r['xl'], r['vlt'])
      if td >= t0 or synthetic:   # measured after the takeover: the plant geometry, quantized as LONG_DIST / REL_SPEED
        d_rel = H._quantize(xl - ego_x(td), gap_q)
        v_rel = float(np.float32(round(vlt - ego_v(tv), 2)))
      else:
        d_rel, v_rel = r['d'], r['vrel']
      v_lead = v_rel + rad['hist'][0]
      new_in_log = (not synthetic) and r['alk'] == 0.0 and r['vlk'] == r['vl']   # radard created the track on this frame
      if rad['track'] is None or r['tid'] != rad['tid'] or not rad['prev_radar'] or new_in_log:
        tr = rad['track'] = radard.Track(r['tid'], v_lead, kp)
        rad['tid'] = r['tid']
        tr.update(d_rel, 0.0, v_rel, v_lead, True)
        if not new_in_log and not synthetic:   # the lead switched to a track radard already filtered: its logged KF state
          tr.kf.set_x([[r['vlk']], [r['alk']]])
          tr.vLeadK, tr.aLeadK, tr.aLeadTau.x, tr.cnt = r['vlk'], r['alk'], r['tau'], 1
      else:
        rad['track'].update(d_rel, 0.0, v_rel, v_lead, True)
      s = rad['track'].get_RadarState()
      out.update(d=float(get_published_lead_distance(s['dRel'], c.get('isd', 0.3))), vrel=s['vRel'], vl=s['vLead'], vlk=s['vLeadK'],
                 alk=s['aLeadK'], tau=s['aLeadTau'], model=True, gap_true=xl - ego_x(td), vl_true=vlt)
      out['gap_true'] = (c['lead'].x_at(r['t']) if synthetic else _lead_x(c, E, r['t'])) - ego_x(r['t'])
    elif r['st']:  # vision lead / unmatched: logged values, dRel corrected for the ego displacement difference
      out['d'] = r['d'] + (x_truth(r['t']) - ego_x(r['t'])) if not synthetic else r['d']
    rad['prev_radar'] = bool(r['st'] and r['radar'] and r['lt'] is not None)
    if r['st2'] and not synthetic:
      out['d2'] = r['d2'] + x_truth(r['t'] - CAN_TO_LT - lag_d) - ego_x(r['t'] - CAN_TO_LT - lag_d)
    rs_out.append(out)

  def rs_msg(i, ev):
    """The logged radarState event at index i with the modelled lead fields."""
    s = rs_out[i]
    r = rs_log[i]
    if not (s['model'] or s['d'] != r['d'] or s['d2'] != r['d2']) and not synthetic:
      return ev
    b = ev if synthetic else ev.as_builder()
    L = b.radarState.leadOne
    if s['st']:
      L.dRel, L.vRel, L.vLead, L.vLeadK, L.aLeadK, L.aLeadTau = s['d'], s['vrel'], s['vl'], s['vlk'], s['alk'], s['tau']
    if synthetic:
      L.status, L.radar, L.modelProb, L.radarTrackId, L.yRel = True, True, 1.0, r['tid'], 0.0
    if r['st2']:
      b.radarState.leadTwo.dRel = s['d2']
    return b

  # ---- planner ----
  by = E['by']
  plans, ctl = {}, dict(j=-1, prev=None)
  lps_t, lps_md = E['t_plan'], E['md_plan']
  t_plan_start = o + c.get('planner_start', c['lo'] - o)
  ctl['j'] = int(np.searchsorted(lps_t, t_plan_start)) - 1

  def latest(s, t):
    ns = by[s][0]
    i = int(np.searchsorted(ns, int(round(t * 1e9)), side='right')) - 1
    return (by[s][1][i], i) if i >= 0 else (None, -1)

  pib = opts.get('plan_bound', PLAN_BOUND.get(c['id'], PLAN_INPUT_BOUND))   # cyc_1003r: per-case planner input bound

  def planner_tick(j):
    md = lps_md[j]
    msgs = {}
    for s in PSERV:
      m, _ = latest(s, md if s == 'modelV2' else md + pib)
      if m is None:
        return
      msgs[s] = m
    if synthetic:
      ob = obs(md - 0.005)
      v_e, a_e = (ob[0], ob[1]) if ob is not None else (c['frames'][0]['cs']['vEgo'], c['frames'][0]['cs']['aEgo'])
      msgs = E['synth_msgs'](md, msgs, v_e, a_e, c['lead'].x_at(md) - ego_x(md))
    i_rs = bisect.bisect_right(t_rs, md + pib) - 1
    if 0 <= i_rs < len(rs_out):
      msgs['radarState'] = rs_msg(i_rs, by['radarState'][1][i_rs] if not synthetic else msgs['radarState'])
    loop = md >= t0
    if loop and not synthetic and opts.get('e2e', 'replay') == 'proxy':
      # cyc_1004 line_floor: the reactive e2e stand-in (e2e_proxy, the synthetic cases' model) on the plant truth instead of the
      # time-replayed model (a candidate that moves the car metres back would otherwise inherit HEAD's position-dependent braking)
      from openpilot.selfdrive.modeld.constants import ModelConstants
      ob = obs(md - 0.005)
      v_e = ob[0] if ob is not None else float(np.interp(md, ft, c['v_true']))
      rk = [x for x in rs_out[-3:] if x.get('model') and np.isfinite(x.get('vl_true', np.nan))]
      gap = _lead_x(c, E, md) - ego_x(md)
      if rk and np.isfinite(gap):
        vl = float(rk[-1]['vl_true'])
        al = 0.0 if len(rk) < 2 else float(np.clip((rk[-1]['vl_true'] - rk[0]['vl_true']) / max(rk[-1]['t'] - rk[0]['t'], 0.05), -4.0, 2.0))
        a = e2e_proxy(v_e, gap, vl, al)
        T = np.asarray(ModelConstants.T_IDXS)
        vv = np.maximum(v_e + a * T, 0.0)
        xx = np.r_[0.0, np.cumsum((vv[1:] + vv[:-1]) / 2 * np.diff(T))]
        b = msgs['modelV2'].as_builder()
        mm = b.modelV2
        mm.position.x, mm.velocity.x, mm.acceleration.x = xx.tolist(), vv.tolist(), np.where(vv > 0, a, 0.0).tolist()
        mm.action.desiredAcceleration, mm.action.shouldStop = a, False
        msgs['modelV2'] = b
    if loop:
      t_cs = msgs['carState'].logMonoTime * 1e-9
      ob = obs(t_cs)
      if ob is not None:
        b = msgs['carState'] if synthetic else msgs['carState'].as_builder()
        b.carState.vEgo, b.carState.aEgo, b.carState.vEgoRaw, b.carState.standstill = float(ob[0]), float(ob[1]), float(ob[2]), bool(ob[3])
        msgs['carState'] = b
        b = msgs['carControl'] if synthetic else msgs['carControl'].as_builder()
        b.carControl.actuators.accel = float(_hist_at(hist['t'], hist['out'], msgs['carControl'].logMonoTime * 1e-9))
        msgs['carControl'] = b
    sm = _SM(msgs, ctl['prev'])
    ctl['prev'] = {s: msgs[s].logMonoTime for s in msgs}
    toggles = _toggles(sm['frogpilotPlan'].frogpilotToggles)
    _AIM.clear()
    planner.update(sm, toggles)
    pm = _PM()
    planner.publish(sm, pm, toggles)
    p = pm.last.longitudinalPlan
    aim = _AIM[-1] if _AIM else (None, False, np.nan)
    rsl = sm['radarState'].leadOne
    plans[j] = dict(t=lps_t[j] - o, aTarget=float(p.aTarget), shouldStop=bool(p.shouldStop), dts=float(p.distanceToStopTarget),
                    fcw=bool(p.fcw), dts_model=float(p.distanceToStopTargetModel),
                    traj=float(p.aTargetTrajectory) if p.aTargetTrajectoryValid else None, valid=bool(pm.last.valid), closed=loop,
                    aim_floor=aim[0], aim=bool(planner.stop_aim_committed), pre_aim=aim[2], v=float(sm['carState'].vEgo),
                    dRel=float(rsl.dRel), vLead=float(rsl.vLead), aLeadK=float(rsl.aLeadK),
                    log_a=float(by['longitudinalPlan'][1][j].longitudinalPlan.aTarget) if not synthetic else np.nan)

  # ---- main loop ----
  Plant = plant_class(frac, tuple(fade) if fade else None, creep)  # noqa: N806
  gs = G.GearState(gmode, c, t0 if closed else None) if creep is not None and closed else None
  k_q = dict(creep)['k_q'] if creep else 0.0
  plant, sent, upper, lower, stop_req, x0, k_cross = None, 0.0, 3.0, 5.0, False, 0.0, None
  rows = []
  for k, (now, i) in enumerate(zip(grid, idx, strict=True)):
    f = frames[i]
    cs = H._cs(f['cs'])
    kw = dict(f['kw'])
    sds = frame_sds[i]
    if synthetic:
      c['lead'].step(now, plant.v if plant is not None else f['cs']['vEgo'])
    while rad['k'] < len(rs_log) and t_rs[rad['k']] <= sds + 1e-9:
      radar_tick(rad['k'])
      rad['k'] += 1
    while ctl['j'] + 1 < len(lps_t) and lps_t[ctl['j'] + 1] <= sds + 1e-9:
      ctl['j'] += 1
      if lps_t[ctl['j']] >= t_plan_start:
        planner_tick(ctl['j'])
    if closed and plant is None and now >= seed_t - 1e-9:
      v0 = float(np.interp(now, ft, c['v_true']))
      plant = Plant(max(v0, 0.0), f['cs']['aEgo'], float(H._held(S['scc12__t'], S['scc12__aReqValue'], now)) if len(S['scc12__t']) else f['recorded'],
                    cobj, GAIN, grade)
      if opts.get('standstill'):   # cyc_1004: standstill / release gate (gated.py)
        from openpilot.tools.stopping.sim.gated import Creep1, Gated
        ss = opts['standstill']
        plant = Gated(Creep1(plant) if ss == 'gate+creep1' else plant, **({'C_GO': -1e9} if ss == 'creep_at_rest' else {}))
      if gs is not None:
        plant.gear_now = gs(now, plant.v, False)
        plant.gear1 = plant.gear_now == 1
    loop = closed and now >= t0 - 1e-9
    if loop and k_cross is None:
      k_cross = k
      if not synthetic:
        plant.v = float(np.interp(now, ft, c['v_true']))
        plant.speed_history.extend(float(np.interp(now - H.DT * jj, ft, c['v_true'])) for jj in range(9, -1, -1))
        plant.v_ego, plant.a_ego, plant.raw = f['cs']['vEgo'], f['cs']['aEgo'], f['cs']['vEgo']
      x0 = x_truth(now)
      plant.x = 0.0
    i_rs = bisect.bisect_right(t_rs, sds + 1e-9) - 1
    rsx = rs_out[i_rs] if 0 <= i_rs < len(rs_out) else None
    j = ctl['j']
    pj = plans.get(j)
    if loop or synthetic:
      cs.vEgo, cs.aEgo, cs.vEgoRaw, cs.standstill = plant.v_ego, plant.a_ego, plant.raw, plant.standstill
      # cyc_1003r: opts cruise_standstill='plant' (default, as the original harness) or 'car' (False: the Hyundai carstate with
      # openpilot long always publishes cruiseState.standstill False; 'plant' blocks stopping -> starting until the plant moves)
      cs.cruiseState.standstill = plant.standstill if opts.get('cruise_standstill', 'plant') == 'plant' else False
      if rsx is not None:
        kw.update(lead_d_rel=rsx['d'], lead_v=rsx['vl'], lead_a=rsx['alk'], lead2_d_rel=rsx['d2'])
      if pj is None:
        raise RuntimeError(f'{c["id"]}: no replanned plan at {now - o:.2f}')
      target, should_stop, dts = pj['aTarget'], pj['shouldStop'], pj['dts']
      kw.update(fcw=pj['fcw'], model_stop_d=pj['dts_model'], a_target_trajectory=pj['traj'], plan_valid=pj['valid'])
    else:
      target, should_stop, dts = f['target'], f['should_stop'], f['dts']
    if not f['active']:
      lc.reset()
    prev_state = lc.long_control_state
    limits = CI.get_pid_accel_limits(cp, cs.vEgo, cs.vCruise / 3.6)
    box.clear()
    out = lc.update(f['active'], cs, target, should_stop, dts, limits, toggles_lc, request_time=float(now), **kw)
    cmd = float(min(out, toggles_lc.max_desired_acceleration))
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
               lead_a=kw['lead_a'] if kw['lead_status'] else np.nan, a_target=target, a_target_log=f['target'],
               a_target_rep=pj['aTarget'] if pj else np.nan, should_stop=should_stop,
               dts=dts, plan_t=pj['t'] if pj else np.nan, aim=bool(pj['aim']) if pj else False,
               aim_floor=pj['aim_floor'] if pj and pj['aim_floor'] is not None else np.nan, pre_aim=pj['pre_aim'] if pj else np.nan,
               phase=svc.phase.name, svc_active=bool(box.get('svc_active')), owning=lc._service_live_owning, lcs=int(lc.long_control_state),
               ff_added=lc._final_floor_added, mon_trig=svc._mon_triggered, closed=loop,
               gap_true=rsx['gap_true'] if rsx else np.nan, vl_true=rsx['vl_true'] if rsx else np.nan)
    for key, val in (box.get('dbg') or {}).items():
      if key != 'phase':
        row['d_' + key] = val
    if plant is not None:
      u = sent if loop else float(H._held(S['scc12__t'], S['scc12__aReqValue'], now))
      up = upper if loop else (float(H._held(S['scc14__t'], S['scc14__JerkUpperLimit'], now)) if len(S['scc14__t']) else 3.0)
      lo_ = lower if loop else (float(H._held(S['scc14__t'], S['scc14__JerkLowerLimit'], now)) if len(S['scc14__t']) else 5.0)
      sr = stop_req if loop else (bool(H._held(S['scc12__t'], S['scc12__StopReq'], now)) if len(S['scc12__t']) else False)
      if loop:
        hist['t'].append(now)
        for key, val in (('v_obs', plant.v_ego), ('a_obs', plant.a_ego), ('raw', plant.raw), ('ss', plant.standstill),
                         ('x', plant.x), ('v', plant.v), ('out', cmd)):
          hist[key].append(val)
      v_before = plant.v
      if profile is not None:
        plant.grade = float(np.interp(x_truth(now) if not loop else x0 + plant.x, *profile)) + cobj.grade
      if gs is not None:
        plant.gear_now = gs(now, plant.v, loop)
      plant.step(float(u), float(up), float(lo_), sr)
      row.update(v_true=plant.v if loop else v_truth(now), a_real=(plant.v - v_before) / H.DT if loop else np.nan,
                 x=x0 + plant.x if loop else x_truth(now), plant_off=plant.off, gap=row['gap_true'] if loop else row['gap_meas'])
      if gs is not None:   # harness_g: plant gear (1 / 2) and the 2nd-gear creep push (m/s^2)
        row.update(gear=2 - int(plant.gear1), creep=plant._q * k_q)
    else:
      row.update(v_true=v_truth(now), a_real=np.nan, x=x_truth(now), plant_off=False, gap=row['gap_meas'])
    rows.append(row)
  trace = _columns(rows)
  a_log = np.interp(trace['t'] + o, ft, c['a_real'])
  trace['a_real'] = np.where(trace['closed'].astype(bool), trace['a_real'], a_log)
  if c.get('a_imu') is not None:
    trace['a_imu'] = np.interp(trace['t'] + o, ft, c['a_imu'])
  info = dict(gear=gmode if creep is not None else 'legacy', creep=dict(creep) if creep else None, id=c['id'], cls=c['cls'], start=start, cell=cell if isinstance(cell, str) or cell is None else repr(cell), radar_delay=radar_delay,
              closed=closed, grade=grade if closed else None, t_stop_rec=(c['t_stop'] - o) if c.get('t_stop') else None, kcs_rec=c.get('kcs'),
              cross=(c['cross'] - o) if not synthetic else 0.0, frac=frac, fade=fade, lag_v=lag_v, lag_d=lag_d)
  return dict(trace=trace, plans=plans, info=info)


def _lead_x(c, E, t):
  """Lead truth position (route clock) at t: sample-and-hold of the reconstructed lead track positions."""
  if '_lx' not in E:
    pts = sorted((r['lt'] - CAN_TO_LT - LAG_D, r['xl']) for r in E['rs'] if r.get('xl') is not None)
    E['_lx'] = (np.array([p[0] for p in pts]), np.array([p[1] for p in pts]))
  lt, lx = E['_lx']
  return float(H._held(lt, lx, t)) if len(lt) else np.nan


def _frame_sds(c, E):
  """The selfdriveState time matching each logged controller frame (variant C: latest at or before the carControl)."""
  if '_sds' not in E:
    E['_sds'] = E['sds_t'][np.clip(np.searchsorted(E['sds_t'], np.array([f['t'] for f in c['frames']]) + 1e-9) - 1, 0, None)]
  return E['_sds']


def _columns(rows):
  keys = list(dict.fromkeys(k for r in rows for k in r))
  out = {}
  for key in keys:
    col = [r.get(key) for r in rows]
    try:
      out[key] = np.array([np.nan if x is None else x for x in col], dtype=float)
    except (TypeError, ValueError):
      out[key] = np.array(col, dtype=object)
  return out


# ---- metrics ----------------------------------------------------------------------------------------------------------
def zigzag(t, s, h):
  """Turning points of s with hysteresis h: [(t, value, 'peak'|'valley')]."""
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


def felt(t, a, mask):
  """stop_index felt: max |a_k - a_j| / (t_k - t_j) with j the first sample within 0.3 s of k, on the masked samples.
  Returns (value, t_j, t_k)."""
  idx = np.flatnonzero(mask)
  T, A = t[idx], a[idx]
  best, j = (0.0, None, None), 0
  for k in range(len(T)):
    while T[k] - T[j] > 0.30:
      j += 1
    if k > j and abs(A[k] - A[j]) / (T[k] - T[j]) > best[0]:
      best = (abs(A[k] - A[j]) / (T[k] - T[j]), float(T[j]), float(T[k]))
  return best


def _steps(t, x, lo, hi, thr=0.2):
  """Deepening steps: x falls >= thr within 0.3 s from a non-falling frame (history steps_on rule, no speed band)."""
  out = []
  for k in range(2, len(t)):
    if not (lo <= t[k] <= hi) or not (x[k] < x[k - 1] - 0.03 and x[k - 1] >= x[k - 2] - 0.03):
      continue
    kk = np.searchsorted(t, t[k] + 0.3, side='right')
    jj = k + int(np.argmin(x[k:kk]))
    d = float(x[jj] - x[k - 1])
    if d <= -thr and (not out or t[k] - out[-1][0] > 0.3):
      out.append((round(float(t[k]), 3), round(d, 3)))
  return out


def metrics(tr, info):
  """Shape features from the takeover (or the recorded window's 12 s before the stop) to the stop, plus the rest phase.
  Motion = the plant once closed, the logged truth (pulses, logged aEgo, logged dRel) otherwise."""
  t, v, wire = tr['t'], tr['v_true'], tr['wire']
  closed = bool(info['closed'])
  act = tr['active'].astype(bool)
  drv = (~act) | tr['gas'].astype(bool) | tr['brake'].astype(bool)
  if closed:
    st = np.flatnonzero((v <= 0.0) & (t >= info['start']))
    t_stop = float(t[st[0]]) if len(st) else None
  else:
    t_stop = info.get('t_stop_rec')
  lo = info['start'] if closed else (t_stop - 12.0 if t_stop is not None else t[0])
  k_lo = int(np.searchsorted(t, lo))
  before = np.flatnonzero(drv[:int(np.searchsorted(t, t_stop))]) if t_stop is not None else []
  if len(before) and before[-1] + 1 > k_lo:   # the final engaged approach only
    k_lo = int(before[-1]) + 1
    lo = float(t[k_lo])
  k_end = next((k for k in np.flatnonzero(drv) if k >= k_lo), len(t))
  m = dict(t_lo=lo, t_stop=t_stop, complete=t_stop is not None and np.searchsorted(t, t_stop) < k_end)
  if closed:  # a variant must not change the plan before the takeover: max |replayed - logged aTarget| in the 3 s before start
    pre = (t >= info['start'] - 3.0) & (t < info['start']) & np.isfinite(tr['a_target_rep'])
    m['pre_start_plan_gap'] = float(np.max(np.abs(tr['a_target_rep'][pre] - tr['a_target_log'][pre]))) if pre.any() else None
  k_stop = int(np.searchsorted(t, t_stop)) if t_stop is not None else k_end
  w = slice(k_lo, min(k_stop, k_end))
  if w.stop - w.start < 5:
    return m
  tw, ww = t[w], wire[w]
  m['min_wire'] = float(ww.min())
  m['t_min_wire'] = float(tw[np.argmin(ww)])
  band = (v[w] < 2.5)
  m['min_wire_band'] = float(ww[band].min()) if band.any() else None
  m['max_step_10ms'] = float(np.min(np.diff(ww)))
  m['wire_steps'] = _steps(tw, ww, lo, t_stop or tw[-1])
  # decel signal: the plant wheel accel (closed) / logged pulse accel (recorded), 0.1 s mean; aEgo as the controller sees it
  a01 = H._trailing_mean(t, np.nan_to_num(tr['a_real']), 0.1)
  zw = slice(k_lo, max(int(np.searchsorted(t, (t_stop or t[w.stop - 1]) - 0.25)), k_lo + 2))
  g = tr['gap']
  m['min_gap'] = float(np.nanmin(g[k_lo:k_end])) if np.isfinite(g[k_lo:k_end]).any() else None
  ae = tr['a_ego']
  zz_w, zz_a = zigzag(t[zw], -wire[zw], 0.08), zigzag(t[zw], -ae[zw], 0.08)
  m['bites_wire'] = sum(p[2] == 'peak' for p in zz_w)
  m['bites_decel'] = sum(p[2] == 'peak' for p in zz_a)   # on aEgo (native KF wheel accel: the car's and the plant's observation)
  m['bites_wheel'] = sum(p[2] == 'peak' for p in zigzag(t[zw], -a01[zw], 0.08)) if closed else None   # plant wheel, 0.1 s mean
  m['zigzag_wire'] = [(round(p[0], 2), round(-p[1], 3), p[2]) for p in zz_w]
  m['zigzag_decel'] = [(round(p[0], 2), round(-p[1], 3), p[2]) for p in zz_a]
  m['peak_decel'] = float(np.min(a01[w])) if closed else float(np.min(ae[w]))
  if t_stop is not None:
    ts = t_stop
    term_lo = t[np.flatnonzero((t < ts) & (v >= 0.45))[-1]] if ((t < ts) & (v >= 0.45)).any() else ts - 1.0
    m['felt'], m['t_felt'], _ = felt(t, ae, (t >= term_lo) & (t <= ts + 0.4))
    m['felt_appr'], m['t_felt_appr'], _ = felt(t, ae, (t >= max(ts - 12.0, lo + (0.5 if closed else 0.0))) & (t < term_lo) & (tr['v'] <= 9.0))
    q = np.arange(ts - 0.5, ts + 0.5, 0.01)
    m['j300'] = float(np.max((np.interp(q + 0.3, t, a01) - np.interp(q, t, a01)) / 0.3))
    m['a_stop'] = float(np.mean(tr['a_real'][max(k_stop - 10, 0):k_stop])) if closed else _a_stop_rec(info, t, tr)
    m['j300_law'] = 4.88 * abs(m['a_stop']) + 0.11 if m['a_stop'] is not None else None
    m['wire_at_stop'] = float(wire[k_stop]) if k_stop < len(t) else None
    m['rest'] = float(np.nanmedian(g[(t >= ts + 0.3) & (t <= ts + 1.3) & (np.arange(len(t)) < k_end)])) if k_stop < k_end else None
    m['rest_at_stop'] = float(g[k_stop]) if k_stop < len(t) else None
  # aim commit: first planner frame whose plan is committed (after lo), with the aTarget and wire steps around it
  # planner steps (aTarget of the HEAD planner: live once closed, the background replay in recorded mode) at 1.5-4.5 m/s,
  # and those the stop-aim floor makes (the floor binds at the step's end)
  a_pl = tr['a_target'] if closed else tr['a_target_rep']
  band_v = (v >= 1.5) & (v <= 4.5)
  steps = [(ts_, d_) for ts_, d_ in _steps(t, np.nan_to_num(a_pl), lo, t_stop or t[w.stop - 1]) if band_v[int(np.searchsorted(t, ts_))]]
  aimb = []
  for ts_, d_ in steps:
    k = int(np.searchsorted(t, ts_ + 0.3))
    k = min(k, len(t) - 1)
    if abs(a_pl[k] - tr['aim_floor'][k]) < 0.02 or abs(a_pl[min(k + 5, len(t) - 1)] - tr['aim_floor'][min(k + 5, len(t) - 1)]) < 0.02:
      aimb.append((ts_, d_))
  m.update(plan_steps=steps, aim_bites=aimb, n_aim_bites=len(aimb), aim_bite_max=min((d_ for _, d_ in aimb), default=0.0))
  aim = tr['aim'].astype(bool)
  m['aim_at_start'] = bool(aim[k_lo])
  k_aim = next((k for k in range(max(k_lo, 1), w.stop) if aim[k] and not aim[k - 1]), None)
  if k_aim is not None:
    pre = tr['a_target'][max(k_aim - 1, 0)]
    m.update(t_aim=float(t[k_aim]), v_aim=float(v[k_aim]), aim_step=float(tr['a_target'][k_aim] - pre),
             aim_wire_step=float(np.min(wire[k_aim:k_aim + 30]) - wire[k_aim - 1]))
  else:
    m.update(t_aim=None, v_aim=None, aim_step=None, aim_wire_step=None)
  # service entry (the last APPROACH entry before the stop): entry wire, bite (min wire 0.6 s after - entry), trough, release
  sa = tr['svc_active'].astype(bool)
  ent = [k for k in range(max(k_lo, 1), w.stop) if sa[k] and not sa[k - 1]]
  if ent:
    ke = ent[-1]
    k6 = int(np.searchsorted(t, t[ke] + 0.6))
    kt = ke + int(np.argmin(wire[ke:w.stop])) if w.stop > ke else ke
    m.update(t_entry=float(t[ke]), v_entry=float(v[ke]), wire_entry=float(wire[ke - 1]),
             entry_bite=float(np.min(wire[ke:max(k6, ke + 1)]) - wire[ke - 1]), trough=float(wire[kt]), t_trough=float(t[kt]),
             release=float(np.max(wire[kt:w.stop]) - wire[kt]) if w.stop > kt else 0.0)
  else:
    m.update(t_entry=None, v_entry=None, wire_entry=None, entry_bite=None, trough=None, t_trough=None, release=None)
  return m


def _a_stop_rec(info, t, tr):
  kcs = info.get('kcs_rec') or {}
  if kcs.get('a_stop') is not None:
    return float(kcs['a_stop'])
  return None


# ---- sweeps -----------------------------------------------------------------------------------------------------------
def _register(cid):
  if isinstance(cid, str) and cid.startswith('h_') and cid not in H.CASES:
    history_cases('aim')
    history_cases('nobite')


def _build_one(cid):
  try:
    _register(cid)
    H.case(cid)
    return 'ok'
  except Exception as exc:  # noqa: BLE001 -- report which cases cannot be built and why
    return f'{type(exc).__name__}: {exc}'


def build(ids, processes=8):
  """Build the case caches in parallel (spawn); returns {id: 'ok' | error}."""
  ids = [i for i in ids if i not in SYNTH]
  with H._pool(processes) as pool:
    return dict(zip(ids, pool.map(_build_one, ids, chunksize=1), strict=True))


def _job(job):
  c, variant, cell, opts = job
  try:
    return run(c, variant=variant, cell=cell, **opts)
  except Exception as exc:  # noqa: BLE001 -- one failed case must not drop the sweep; reported per job
    return dict(info=dict(id=c if isinstance(c, str) else c.get('id'), cell=cell), error=f'{type(exc).__name__}: {exc}')


def sweep(jobs, processes=4):
  """jobs: (case, variant, cell, opts) tuples; opts may hold radar_delay/start and run() opts; traces dropped unless
  opts['keep_trace']. spawn pool on macOS (variants must be picklable module-level callables)."""
  jobs = [(c, v, cell, {'keep_trace': False, **(o or {})}) for c, v, cell, o in jobs]
  todo = [c for c in {j[0] for j in jobs if isinstance(j[0], str)} if c not in SYNTH and not (H.CACHE / f'{c}.pkl').is_file()]
  if todo:
    build(todo, processes)
  if processes <= 1:
    return [_job(j) for j in jobs]
  with H._pool(processes) as pool:
    return pool.map(_job, jobs, chunksize=1)
