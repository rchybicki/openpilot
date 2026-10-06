"""The fixed case set, the plant cells and the case builders of the standard runner (merged from the cyc_1004 runners cl3.py,
vr4.py, ev.py and ccl2.py; the definitions are unchanged). STANDARD is part of the fixed gate list: changing it changes the gates.

Case ids: recorded stops (R.NEW, history 'h_*', named, downhill, queue 'q_*', '<case>@hold' = the lead held where it stands
from 0.5 s before its launch), census holds 'sv_<route8>_<t_start>' (run with the driver's inputs removed, 'nodrv'), synthetic
families (syn, rb, rl, cut, slow, go, cr, lt, burst, sgs, ms).
"""
import contextlib
import copy
import json

import numpy as np

from openpilot.tools.stopping.sim import SIM_HOME
from openpilot.tools.stopping.sim import harness as H
from openpilot.tools.stopping.sim import rharness as R
from openpilot.tools.stopping.sim import gear as GEAR

# cell -> ((trigger, threshold, gain_delta), grade fraction); 'P' = the reactive e2e proxy (rharness e2e='proxy')
CELLS = {'L42': (('level', -0.42, -0.035), (0.5, 0.3)), 'H': (('history', -0.42, -0.035), (0.5, 0.3)),
         'L42s': (('level', -0.42, -0.035), (0.75, 0.5)), 'L42F': (('level', -0.42, -0.035), (0.5, 0.3, 1.0))}
CELLS.update({k + 'P': v for k, v in list(CELLS.items()) if k in ('L42', 'H', 'L42s')})

# ---- synthetic families (cl3.py) -------------------------------------------------------------------------------------------
RT_SYNTH = {}
for _v in (10.0, 13.0, 16.0):          # (a) a lead brakes to a stop from speed
  for _hw in (1.2, 1.8):
    for _a in (-1.5, -2.5, -3.5):
      RT_SYNTH[f'rb_v{_v:g}_h{_hw:g}_a{-_a:g}'] = dict(v0=_v, gap0=round(_hw * _v + 3.0, 1), lead='crawl_stop', v_lead0=_v, t_lead=1.0,
                                                    a_lead=_a, duration=round(1.0 + _v / -_a + 9.0, 1))
for _v in (6.0, 8.0, 10.0):            # (b) a stopped lead first seen late
  for _req in (1.35, 1.6, 1.9):
    _d = _v * _v / (2.0 * _req) + 4.3 + 0.45 * _v + 0.5 * _v
    RT_SYNTH[f'rl_v{_v:g}_r{_req:g}'] = dict(v0=_v, gap0=round(_d, 1), lead='stopped', duration=round(_v / 1.0 + 8.0, 1))
R.SYNTH.update(RT_SYNTH)
# (c) cut-in: a far stopped object (track 7); at t_cut a car (track 8) cuts in at gap gc moving at vc and brakes to a stop at ac
CUT = {}
for _v0, _gc, _vc, _ac in ((8.0, 10.0, 3.0, -1.5), (8.0, 14.0, 4.0, -2.0), (6.0, 8.0, 2.0, -1.0), (6.0, 6.0, 1.5, -1.5),
                           (10.0, 16.0, 5.0, -2.5), (4.0, 7.0, 1.0, -1.0)):
  CUT[f'cut_v{_v0:g}_g{_gc:g}_v{_vc:g}_a{-_ac:g}'] = dict(v0=_v0, gc=_gc, vc=_vc, ac=_ac, t_cut=2.0, far=60.0)
# (d) non-stop slowdowns: the lead brakes to v_min (or a brief stop), holds, then accelerates back
SLOW = {'slow_v10_a2_to2': dict(v0=10.0, g0=22.0, a=-2.0, v_min=2.0, hold=2.0, a_up=1.0),
        'slow_v8_a15_to1': dict(v0=8.0, g0=16.0, a=-1.5, v_min=1.0, hold=1.5, a_up=1.2),
        'slow_v6_a1_to05': dict(v0=6.0, g0=12.0, a=-1.0, v_min=0.5, hold=1.0, a_up=1.0),
        'sg_v8_stop05': dict(v0=8.0, g0=16.0, a=-2.0, v_min=0.0, hold=0.5, a_up=1.5),
        'slow_v12_a25_to3': dict(v0=12.0, g0=26.0, a=-2.5, v_min=3.0, hold=1.0, a_up=1.0)}


class _SlowLead(R.Lead):
  def step(self, t, ego_v):
    dt, v, x = t - self.t[-1], self.v[-1], self.x[-1]
    if dt <= 0:
      return
    ts = t - R.SYN_T0
    p = self.slow
    if ts >= p['t_b']:
      if self.t_min is None:
        v = max(v + p['a'] * dt, p['v_min'])
        if v <= p['v_min'] + 1e-9:
          self.t_min = t
      elif t - self.t_min >= p['hold']:
        v = min(v + p['a_up'] * dt, p['v0'])
    self.t.append(t)
    self.x.append(x + (v + self.v[-1]) / 2 * dt)
    self.v.append(v)
    self.a.append((v - self.v[-2]) / dt)


def slow_case(name):
  p = dict(SLOW[name], t_b=2.0)
  dur = 2.0 + (p['v0'] - p['v_min']) / -p['a'] + p['hold'] + (p['v0'] - p['v_min']) / p['a_up'] + 6.0
  c = R.synthetic_case(None, v0=p['v0'], gap0=p['g0'], duration=round(dur, 1), lead='crawl', v_lead0=p['v0'])
  c['id'] = name
  c['lead'].__class__ = _SlowLead
  c['lead'].slow, c['lead'].t_min = p, None
  c['note'] = str(p)
  return c


SYN_BASE = ('syn_stop8', 'syn_stop12', 'syn_crawl_stop', 'syn_crawl', 'syn_reverse', 'syn_launch')
NAMED = ('s20', 's5', 's22', '222e_s4', '2226_s3', '2226_s4', '222d_s3', '2086_s8', '2086_s17')
DOWN = (('00002082--e7e32684a7', 416.15), ('000020b8--a458067950', 288.46), ('000020fa--32a67f8d0c', 318.66))
DOWN_IDS = []
for _route, _tws in DOWN:
  _cid = f"h_{_route[4:8]}_{_tws:.1f}"
  H.CASES.setdefault(_cid, R._spec(_route, _tws, 'downhill', 'steep downhill intersection stop (section 23)'))
  DOWN_IDS.append(_cid)
DOWN_IDS += ['222e_s4', '2234_s18', '2235_s55']


class _CutLead:
  """Two objects: a far stationary object (track 7) until t_cut; then a car (track 8) cuts in gc ahead of the ego, moving at vc,
  and brakes at ac to rest 0.5 s later. The ego position is integrated from the speeds the run loop passes to step()."""

  def __init__(self, gc, vc, ac, t_cut, far):
    self.gc, self.vc, self.ac, self.far = gc, vc, ac, far
    self.t_cut = R.SYN_T0 + t_cut
    self.t_prev, self.ego_x = None, 0.0
    self.t, self.x, self.v, self.a = [R.SYN_T0 - 1.0], [far], [0.0], [0.0]
    self.cut = False

  def step(self, t, ego_v):
    if self.t_prev is not None:
      self.ego_x += ego_v * (t - self.t_prev)
    self.t_prev = t
    dt = t - self.t[-1]
    if dt <= 0:
      return
    if t < self.t_cut:
      x, v = self.far, 0.0
    elif not self.cut:
      self.cut = True
      x, v = self.ego_x + self.gc, self.vc
    else:
      v0 = self.v[-1]
      v = max(v0 + self.ac * dt, 0.0) if t >= self.t_cut + 0.5 else v0
      x = self.x[-1] + (v + v0) / 2 * dt
    self.a.append((v - self.v[-1]) / dt if self.cut and len(self.v) and t > self.t_cut else 0.0)
    self.t.append(t)
    self.x.append(x)
    self.v.append(v)

  def x_at(self, t):
    return float(np.interp(t, self.t, self.x)) if t < self.t_cut or not self.cut else float(np.interp(t, self.t, self.x, left=self.far))

  def v_at(self, t):
    return float(np.interp(t, self.t, self.v))

  def a_at(self, t):
    return float(np.interp(t, self.t, self.a))


def cut_case(name):
  p = CUT[name]
  dur = p['t_cut'] + 0.5 + p['vc'] / -p['ac'] + 12.0
  c = R.synthetic_case(None, v0=p['v0'], gap0=p['far'], duration=dur, lead='stopped')
  c['id'] = name
  c['lead'] = _CutLead(p['gc'], p['vc'], p['ac'], p['t_cut'], p['far'])
  for r in c['stream']['rs']:
    if r['t'] >= R.SYN_T0 + p['t_cut']:
      r['tid'] = 8
  c['note'] = str(p)
  return c


# ---- synthetic families (vr4.py) -------------------------------------------------------------------------------------------
class _GoLead(R.Lead):
  """stopped lead that launches (a_go up to v_go) once the ego speed drops to v_trig (queue restart during the approach)"""
  def step(self, t, ego_v):
    dt, v, x = t - self.t[-1], self.v[-1], self.x[-1]
    if dt <= 0:
      return
    if self.trig is None and t - R.SYN_T0 > 1.0 and ego_v <= self.go['v_trig']:
      self.trig = t
    if self.trig is not None and t - self.trig >= self.go.get('delay', 0.0):
      v = min(v + self.go['a_go'] * dt, self.go['v_go'])
    self.t.append(t)
    self.x.append(x + (v + self.v[-1]) / 2 * dt)
    self.v.append(v)
    self.a.append((v - self.v[-2]) / dt)


GO = {}
for _v0 in (8.0, 12.0):
  for _vt in (4.0, 3.0, 2.2):
    for _ag in (1.0, 2.0):
      GO[f'go_v{_v0:g}_t{_vt:g}_a{_ag:g}'] = dict(v0=_v0, gap0=round(_v0 * _v0 / 2.4 + 4.3 + 0.95 * _v0, 1), v_trig=_vt, a_go=_ag, v_go=10.0)
CR = {f'cr_v{_v0:g}_l{_vl:g}': dict(v0=_v0, gap0=_g0, vl=_vl) for _v0, _g0 in ((6.0, 30.0), (10.0, 45.0)) for _vl in (0.2, 0.35, 0.45)}
LT = {f'lt_v{_v0:g}_g{_g:g}': dict(v0=_v0, gap0=_g) for _v0 in (2.8, 3.3, 4.0) for _g in (8.0, 10.0, 13.0)}
QR = {'q_2073_1098': ('00002073--b2443011ed', 1101.0), 'q_20c0_3486': ('000020c0--418683bc8f', 3489.0),
      'q_20f8_1267': ('000020f8--3495298f77', 1271.0), 'q_203a_455': ('0000203a--41d916332f', 458.5),
      'q_2231_1106': ('00002231--73a1ec87ea', 1111.0)}
for _k, (_r, _t) in QR.items():
  H.CASES.setdefault(_k, R._spec(_r, _t, 'queue', 'recorded stopped lead that launched during the approach'))


def go_case(name):
  p = GO[name]
  c = R.synthetic_case(None, v0=p['v0'], gap0=p['gap0'], duration=round(p['v0'] / 1.0 + 14.0, 1), lead='stopped')
  c['id'] = name
  c['lead'].__class__ = _GoLead
  c['lead'].go = p
  c['note'] = str(p)
  return c


def cr_case(name):
  p = CR[name]
  c = R.synthetic_case(None, v0=p['v0'], gap0=p['gap0'], duration=40.0, lead='crawl', v_lead0=p['vl'])
  c['id'] = name
  c['note'] = str(p)
  return c


def lt_case(name):
  p = LT[name]
  c = R.synthetic_case(None, v0=p['v0'], gap0=p['gap0'], duration=16.0, lead='stopped')
  c['id'] = name
  c['note'] = str(p)
  return c


class _BurstLead(R.Lead):
  """A stopped lead whose RADAR SPEED reads a Doppler burst (amp m/s for dur s) when the ego speed first drops to v_burst; the
  position never moves (the burst is a measurement artefact). Optional second burst."""
  def v_at(self, t):
    b = self.burst
    for t0 in self.bt:
      if t0 <= t < t0 + b['dur']:
        return b['amp']
    return 0.0

  def step(self, t, ego_v):
    dt = t - self.t[-1]
    if dt <= 0:
      return
    b = self.burst
    if len(self.bt) < b.get('n', 1) and t - R.SYN_T0 > 1.0 and ego_v <= b['v_burst'] - 0.6 * len(self.bt):
      self.bt.append(t)
    self.t.append(t)
    self.x.append(self.x[-1])
    self.v.append(0.0)
    self.a.append(0.0)


BURST = {}
for _v0 in (8.0, 12.0):
  for _vb in (5.0, 3.5):
    for _dur in (0.15, 0.25, 0.36):
      for _amp in (0.4, 0.6, 1.2):
        BURST[f'bu_v{_v0:g}_b{_vb:g}_d{_dur:g}_a{_amp:g}'] = dict(v0=_v0, gap0=round(_v0 * _v0 / 2.4 + 4.3 + 0.95 * _v0, 1), v_burst=_vb,
                                                                 dur=_dur, amp=_amp)


def burst_case(name):
  p = BURST[name]
  c = R.synthetic_case(None, v0=p['v0'], gap0=p['gap0'], duration=round(p['v0'] / 1.0 + 12.0, 1), lead='stopped')
  c['id'] = name
  c['lead'].__class__ = _BurstLead
  c['lead'].burst, c['lead'].bt = p, []
  return c


class _SGSLead(R.Lead):
  """stop-go-stop: a stopped lead; when the ego first drops to v_trig the lead moves off at a_go up to v_go, travels dist m, then
  brakes at a_stop back to rest (a queue that lurches forward and stops again)."""
  def step(self, t, ego_v):
    dt, v, x = t - self.t[-1], self.v[-1], self.x[-1]
    if dt <= 0:
      return
    p = self.p
    if self.trig is None and t - R.SYN_T0 > 1.0 and ego_v <= p['v_trig']:
      self.trig, self.x_trig = t, x
    if self.trig is not None:
      if x - self.x_trig < p['dist']:
        v = min(v + p['a_go'] * dt, p['v_go'])
      else:
        v = max(v + p['a_stop'] * dt, 0.0)
    self.t.append(t)
    self.x.append(x + (v + self.v[-1]) / 2 * dt)
    self.v.append(v)
    self.a.append((v - self.v[-2]) / dt)


SGS = {}
for _v0 in (8.0, 12.0):
  for _vt in (1.8, 1.2, 0.6):
    for _vg, _dist in ((1.5, 2.5), (2.5, 5.0), (3.5, 8.0)):
      SGS[f'sgs_v{_v0:g}_t{_vt:g}_g{_vg:g}'] = dict(v0=_v0, gap0=round(_v0 * _v0 / 2.4 + 4.3 + 0.95 * _v0, 1), v_trig=_vt, a_go=1.5,
                                                    v_go=_vg, dist=_dist, a_stop=-2.5)


def sgs_case(name):
  p = SGS[name]
  c = R.synthetic_case(None, v0=p['v0'], gap0=p['gap0'], duration=round(p['v0'] / 1.0 + 22.0, 1), lead='stopped')
  c['id'] = name
  c['lead'].__class__ = _SGSLead
  c['lead'].p = p
  return c


def hold_case(cid):
  """'<case>@hold': the recorded case with its lead held where it stands from 0.5 s before its launch."""
  base = cid.split('@')[0]
  R._register(base)
  c = dict(H.case(base))
  c['id'] = cid
  if cid not in R._EV:
    E = copy.copy(R._stream(H.case(base)))
    rs = [dict(r) for r in E['rs']]
    t_ref = (c.get('t_stop') or c['hi']) - 3.0
    go = [r['t'] for r in rs if r.get('vlt') is not None and r['t'] >= t_ref and r['vlt'] > 0.25]
    t_hold = (go[0] if go else c['hi']) - 0.5
    xh = next((r['xl'] for r in reversed(rs) if r.get('xl') is not None and r['t'] <= t_hold), None)
    for r in rs:
      if r['t'] >= t_hold and r.get('xl') is not None:
        r['xl'], r['vlt'] = xh, 0.0
    E['rs'] = rs
    E.pop('_lx', None)
    E['hold_t'] = t_hold
    R._EV[cid] = E
  return c


_gear_load = GEAR.load
GEAR.load = lambda cid, build=True: _gear_load(cid.split('@')[0], build)


# ---- model-only stops (H3): the model stops at a point (red light / pedestrian) with no radar lead ------------------------------
# ms_v<v0>_none: no radar lead and no model lead; ms_v<v0>_far: a radar + model lead 40 m ahead driving on at v0 (it passes the
# light). The model (cq3/qf qv.py 'kin' stop point, without the creeping queue): coast until reaching the point needs MS_A0, then
# the constant deceleration to MS_AIM short of the point (at least MS_A_MIN, also at rest); the trajectory follows that
# deceleration; shouldStop by modeld's rule (vEgo < 0.3 and desiredAcceleration < 0.1), passed to LongControl as controlsd does
# (ms_model_bit). Point = 2 s of cruise + v0^2 / (2 MS_A0) + MS_AIM ahead. UNVALIDATED model stand-in (as e2e_proxy).
MS = {f'ms_v{_v:g}_{_l}': dict(v0=_v, lead=_l) for _v in (6.0, 8.0, 10.0, 13.0) for _l in ('none', 'far')}
MS_A0, MS_A_MIN, MS_AIM, MS_FAR = 0.9, 0.3, 1.0, 40.0
MS_STATE = {'ss': False}


def ms_case(name):
  from openpilot.selfdrive.modeld.constants import ModelConstants
  p = MS[name]
  v0 = p['v0']
  x_stop = round(2.0 * v0 + v0 * v0 / (2.0 * MS_A0) + MS_AIM, 2)
  far = p['lead'] == 'far'
  c = R.synthetic_case(None, v0=v0, gap0=MS_FAR if far else x_stop, duration=round(2.0 + v0 / MS_A0 + 12.0, 1), lead='crawl' if far else 'stopped',
                       v_lead0=v0 if far else 0.0)
  c['id'], c['note'], c['ms'] = name, f'model-only stop {p}, point {x_stop} m', dict(x_stop=x_stop, brake=False)
  if not far:   # no radar lead: no radarState lead records (the harness forces status on every record it models)
    E = c['stream']
    E['rs'], E['t_rs'] = [], np.zeros(0)
    kw = dict(c['frames'][0]['kw'], lead_status=False, lead_d_rel=0.0, lead_v=0.0, lead_a=0.0, lead_model_prob=0.0)
    c['frames'] = [dict(f, kw=kw) for f in c['frames']]
  T = np.asarray(ModelConstants.T_IDXS)
  orig, st, lead = c['stream']['synth_msgs'], c['ms'], c['lead']
  MS_STATE['ss'] = False

  def synth_msgs(md_t, msgs, ego_v, ego_a, gap):
    out = orig(md_t, msgs, ego_v, ego_a, gap)
    d_rem = st['x_stop'] - (lead.x_at(md_t) - gap) - MS_AIM
    a_req = ego_v * ego_v / (2.0 * max(d_rem, 0.3))
    st['brake'] = st['brake'] or a_req >= MS_A0
    m = out['modelV2'].modelV2
    a = -max(min(a_req, 3.0), MS_A_MIN) if st['brake'] else 0.0
    if far:
      a = min(a, float(m.action.desiredAcceleration))
    ss = bool(ego_v < 0.3 and a < 0.1)
    vv = np.maximum(ego_v + a * T, 0.0)
    xx = np.r_[0.0, np.cumsum((vv[1:] + vv[:-1]) / 2 * np.diff(T))]
    m.position.x, m.velocity.x, m.acceleration.x = xx.tolist(), vv.tolist(), np.where(vv > 0, a, 0.0).tolist()
    m.action.desiredAcceleration, m.action.shouldStop = a, ss
    n_gp = len(m.meta.disengagePredictions.gasPressProbs)
    m.meta.disengagePredictions.gasPressProbs = [0.02] * n_gp
    if not far:
      for L in m.leadsV3:
        L.prob = 0.0
      out['radarState'].radarState.leadOne.status = False
    MS_STATE['ss'] = ss
    return out
  c['stream']['synth_msgs'] = synth_msgs
  return c


@contextlib.contextmanager
def ms_model_bit():
  """controlsd passes the model's shouldStop bit to LongControl (model_should_stop); the synthetic frames carry a constant False."""
  from openpilot.selfdrive.controls.lib.longcontrol import LongControl
  base = LongControl.update

  def update(self, *a, **k):
    k['model_should_stop'] = bool(MS_STATE['ss'])
    return base(self, *a, **k)
  with H.patched((LongControl, 'update', update)):
    yield


def is_ms(cid):
  return cid in MS


# ---- census holds (ev.py register_holds; ccl2.py strip_driver / first_disengage) --------------------------------------------
_HOLDS: dict = {}


def holds():
  """Census holds (SIM_HOME/census/holds.json, cyc_1004/census) keyed '<route8>_<t_start>'."""
  if not _HOLDS:
    for h in json.loads((SIM_HOME / 'census' / 'holds.json').read_text()):
      if 'error' not in h:
        _HOLDS[f"{h['route'][:8]}_{h['t_start']:.2f}"] = h
  return _HOLDS


def register_hold(key):
  """Register 'sv_<key>' in H.CASES (window: the hold start to its end + 12 s, cut 0.5 s before the next hold on the route)."""
  hs = holds()
  h = hs[key]
  cid = 'sv_' + key
  nxt = [g['t_start'] for g in hs.values() if g['route'] == h['route'] and g['t_start'] > h['t_start'] + 0.5]
  hi = min(h['t_end'] + 12.0, (min(nxt) - 0.5) if nxt else 1e9)
  hi = max(hi, h['t_end'] + 2.0)
  H.CASES.setdefault(cid, R._spec(h['route'], h['t_start'], 'svc_hold', f"census hold {key} ({h['began']}, end {h['end']})",
                                  post=hi - h['t_start']))
  return cid


def hold_ids():
  """Every census hold (the census order), whether or not its case is cached: a hold whose case cannot be built is a failed job
  (INCOMPLETE) until the census marks it with an 'error' (holds() skips those)."""
  return [f'sv_{k}' for k in holds()]


def strip_driver(c, t_strip):
  """Driver-free counterfactual: logged gas / brake / disengagement after t_strip (route-relative) removed."""
  c = dict(c)
  fr = list(c['frames'])
  for k, f in enumerate(fr):
    if f['t'] - c['origin'] >= t_strip and (f['cs']['gasPressed'] or f['cs']['brakePressed'] or not f['active']):
      g = copy.deepcopy(f)
      g['cs']['gasPressed'], g['cs']['brakePressed'], g['active'], g['authorized'] = False, False, True, True
      g['kw']['freeze_integrator'] = False
      fr[k] = g
  c['frames'] = fr
  return c


def first_disengage(c, t_from):
  for f in c['frames']:
    if f['t'] - c['origin'] >= t_from and not f['active']:
      return float(f['t'] - c['origin'])
  return None


# ---- groups ------------------------------------------------------------------------------------------------------------------
SYN_FAMILIES = {'rb': [k for k in RT_SYNTH if k.startswith('rb_')], 'rl': [k for k in RT_SYNTH if k.startswith('rl_')], 'cut': list(CUT),
                'slow': list(SLOW), 'go': list(GO), 'cr': list(CR), 'lt': list(LT), 'burst': list(BURST), 'sgs': list(SGS),
                'syn': list(SYN_BASE), 'ms': list(MS)}


def is_syn(cid):
  return cid in R.SYNTH or any(cid in v for v in SYN_FAMILIES.values())


def group_ids(g):
  if g in SYN_FAMILIES:
    return list(SYN_FAMILIES[g])
  if g == 'bm':
    return ['2235_s55', '2235_s71', 's20']
  if g == 'new':
    return list(R.NEW)
  if g == 'hist':
    return R.history_cases('aim') + R.history_cases('nobite', n=30, seed=0)
  if g == 'named':
    return list(NAMED)
  if g == 'down':
    return list(DOWN_IDS)
  if g == 'qr':
    return list(QR)
  if g == 'holds':
    return hold_ids()
  return g.split('+')


def group_of(cid):
  """Report group of a case id (the comfort aggregate groups are 'new' and 'hist')."""
  if cid.startswith('sv_'):
    return 'holds'
  if '@hold' in cid:
    return 'hold@'
  if cid in ('2235_s55', '2235_s71', 's20'):
    return 'bm'
  if cid in R.NEW:
    return 'new'
  if cid in NAMED:
    return 'named'
  if cid in DOWN_IDS:
    return 'down'
  if cid in QR:
    return 'qr'
  for g, ids in SYN_FAMILIES.items():
    if cid in ids:
      return g
  if cid.startswith('h_'):
    return 'hist'
  return 'other'


# STANDARD: (groups, cells, starts, mode). Recorded starts 'v12' (+ 'auto' for queue / @hold); synthetic always 'auto'; nodrv runs
# 'hold' (census hold: its own start rule; R.NEW: 'auto' with the driver stripped from the takeover).
REC_CELLS = ('L42', 'L42s', 'L42P')
STANDARD = (
  (('holds', 'new'), ('L42',), ('hold',), 'nodrv'),   # census holds + R.NEW without the driver (the E3 closed-loop set, 327 runs)
  (('bm', 'new', 'named', 'down', 'hist', 'qr', 's20@hold'), REC_CELLS, ('v12',), 'drv'),
  (('qr', 's20@hold'), REC_CELLS, ('auto',), 'drv'),
  (('down', '2235_s55'), ('L42F',), ('v12',), 'drv'),
  (('syn', 'rb', 'rl', 'slow', 'go', 'burst', 'cut', 'cr', 'lt', 'sgs', 'ms'), REC_CELLS, ('auto',), 'drv'),
)
QUICK = (
  (('bm', 'named'), ('L42',), ('v12',), 'drv'),
  (('syn',), ('L42',), ('auto',), 'drv'),
)


def jobs(spec=STANDARD, cells_only=None):
  """[(case, cell, start, mode)] for a case-set spec; cells_only restricts the cells (the OFF identity arm runs L42 only)."""
  out = []
  for groups, cells, starts, mode in spec:
    ids = []
    for g in groups:
      ids += group_ids(g)
    for cid in dict.fromkeys(ids):
      for cell in cells:
        if cells_only and cell not in cells_only:
          continue
        for s in (['auto'] if is_syn(cid) and starts != ('hold',) else starts):
          out.append((cid, cell, s, mode))
  return list(dict.fromkeys(out))


def make(cid, start, mode):
  """(case object or id, resolved start, meta) for R.run."""
  meta = {}
  if cid.startswith('sv_'):
    key = cid[3:]
    register_hold(key)
    h = holds()[key]
    c0 = H.case(cid)
    t_strip = h['t_start'] + 0.5
    c = strip_driver(c0, t_strip) if mode == 'nodrv' else c0
    st = h['t_start'] + 1.0 if h['began'] == 'engage_at_standstill' else R.resolve_start(c, 'auto')
    meta.update(t_strip=t_strip, t_dis=first_disengage(c0, t_strip) if mode == 'nodrv' else None, began=h['began'], t_hold=h['t_start'])
    return c, st, meta
  if cid in CUT:
    return cut_case(cid), start, meta
  if cid in MS:
    return ms_case(cid), start, meta
  if cid in SLOW:
    return slow_case(cid), start, meta
  for fam, fn in ((GO, go_case), (CR, cr_case), (LT, lt_case), (BURST, burst_case), (SGS, sgs_case)):
    if cid in fam:
      return fn(cid), start, meta
  if '@hold' in cid:
    return hold_case(cid), start, meta
  R._register(cid)
  return cid, start, meta
