"""F1 following matrix (radar-input changes): synthetic closed-loop following at 20-30 m/s, scored on PHYSICAL truth.

Every case runs the tree's production RadarD (rharness) on synthesized tracks of known objects: the measurement is the truth
delayed by LAG_D / LAG_V (the `lag` cells vary LAG_V over the measured p10-p90). Objects never react to the ego. Families:
  hb   steady following at 1.5 s, the lead brakes at a (-2, -3, -4) to half speed (or to rest from 20 m/s) and holds
  ab   ego and lead accelerate together at +1 for 3 s, then the lead brakes at -3 (stale positive aLeadK at the transition)
  cut  a car cuts in already braking (new radar track: cold Kalman filter)
  sw   the lead brakes at -3; during the braking its radar track id changes: 'cold' (a new id) or 'warm' (to a second track of
       the same car that radard has filtered since the start)
  l2   leadTwo becomes limiting: the lead changes lanes out while the car ahead of it brakes at -3 (it was leadTwo)
  rv   radar / vision transition: the braking lead's radar track drops out for 1 s (vision lead), then returns as a new track
Configurations (the planner's modes): B = Experimental + CEM + human_following (the car's setting: CEM decides experimentalMode),
A = ACC (CEM and Experimental off) + human_following, K = ACC with human_following off (the radar-only lead extrapolation path).
The tree's FrogPilotPlanner runs on the simulated state every model tick (rharness fp_tick: following t_follow / jerks / danger factor,
CEM, acceleration limits, traffic controller, events) and its frogpilotPlan feeds the planner and radard; follow_distance only sets
the starting gap. The model is the rharness e2e proxy (UNVALIDATED; it
sees the truth without lag, so blended rows are less sensitive to radar errors than ACC rows).

Physical measures (per run, from the event start t_ev to the first contact): min clearance to the nearest in-lane object, min TTC,
max closing speed, collision with the impact (closing) speed and time; brake onset = first sustained sent SCC12 demand
(<= ONSET_A for ONSET_S); release = first sent >= RELEASE_A after the onset. gates.f1 compares each candidate run with
HEAD's run of the same case and cell.
"""
import json

import numpy as np

from openpilot.tools.stopping.sim import rharness as R

LANE_Y = 1.5          # m: an object with |y| below this is in the ego lane (clearance truth)
ONSET_A, ONSET_S = -0.5, 0.3     # m/s^2, s: brake onset = sent SCC12 <= ONSET_A held for ONSET_S
RELEASE_A = -0.3      # m/s^2: release = the first sent >= this after the onset
DUR_TAIL = 8.0        # s after the lead reaches its end speed
CONFIGS = {'B': dict(experimental=True, human_following=True), 'A': dict(experimental=False, human_following=True),
           'K': dict(experimental=False, human_following=False)}
LAGS = (0.10, 0.21)   # s: LAG_V cells (verify_impact.json REL_SPEED p10-p90); the base cells use rharness.LAG_V 0.15
# the F1 plant: the L42 cell with gain_delta refitted at speed (README "Plant at speed": 57 logged follow-brake events at 10-38 m/s;
# -0.035 brakes 0.13 m/s^2 more than the car on sent <= -1 frames, -0.135 has a median bias of 0.00); F1 only, the stop cells keep L42
PLANT = (('level', -0.42, -0.135), (0.5, 0.3))


class Obj:
  """A scripted object: speed profile from (t, accel, v_end) segments (scenario seconds), lateral y(t) ramp, radar track ids."""
  def __init__(self, x0, v0, segs=(), y_ramp=None, anchor=None, visible=None):
    t = np.arange(-1.0, 120.0, 0.01)
    v, a = np.empty_like(t), np.zeros_like(t)
    vv = v0
    for k, tt in enumerate(t):
      acc = 0.0
      for t_s, a_s, v_end in segs:   # the last started segment rules; it ends at its end speed
        if tt >= t_s:
          acc = a_s if (a_s < 0 and vv > v_end) or (a_s > 0 and vv < v_end) else 0.0
      vv = max(vv + acc * 0.01, 0.0) if k else v0
      v[k], a[k] = vv, acc
    self.t = R.SYN_T0 + t
    self.v, self.a = v, a
    cum = np.r_[0.0, np.cumsum((v[1:] + v[:-1]) / 2 * 0.01)]
    self.x = x0 + cum - float(np.interp(R.SYN_T0, self.t, cum))   # x0 at the scenario start (the ego starts at x = 0)
    self.y_ramp = y_ramp     # (t0, t1, y_end) scenario seconds, linear
    self.anchor = anchor     # (t, gap): placed `gap` ahead of the ego at scenario time t (a cut-in relative to the actual ego)
    self.visible = visible   # scenario-clock predicate (None = always): radar and clearance truth
    self.ego_x, self.t_prev = 0.0, None

  def step(self, t, ego_v):
    if self.t_prev is not None:
      self.ego_x += ego_v * (t - self.t_prev)
    self.t_prev = t
    if self.anchor is not None and t - R.SYN_T0 >= self.anchor[0] - 1e-9:
      self.x = self.x + (self.ego_x + self.anchor[1] - self.x_at(t))
      self.anchor = None

  def seen(self, t):
    return self.visible is None or self.visible(t - R.SYN_T0)

  def x_at(self, t):
    return float(np.interp(t, self.t, self.x))

  def v_at(self, t):
    return float(np.interp(t, self.t, self.v))

  def a_at(self, t):
    return float(np.interp(t, self.t, self.a))

  def y_at(self, t):
    if self.y_ramp is None:
      return 0.0
    t0, t1, y1 = self.y_ramp
    return float(np.interp(t - R.SYN_T0, (t0, t1), (0.0, y1)))


class Nearest:
  """The nearest in-lane object (the model's lead 0 and the clearance truth)."""
  def __init__(self, objs):
    self.objs = objs

  def step(self, t, ego_v):
    for o in self.objs:
      o.step(t, ego_v)

  def pick(self, t):
    inl = [o for o in self.objs if abs(o.y_at(t)) < LANE_Y and o.seen(t)]
    return min(inl, key=lambda o: o.x_at(t)) if inl else self.objs[-1]

  def x_at(self, t):
    return self.pick(t).x_at(t)

  def v_at(self, t):
    return self.pick(t).v_at(t)

  def a_at(self, t):
    return self.pick(t).a_at(t)


def follow_distance(v, experimental):
  """The starting gap: the MPC's steady follow distance at v behind a lead at v for the car's toggles and personality (get_T_FOLLOW
  with not_leftmost_lane True; the run's following comes from FrogPilotPlanner, the T_EV settle absorbs a difference)."""
  from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import desired_follow_distance, get_T_FOLLOW
  from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.stop_target_helpers import LEAD_STOP_DISTANCE_TARGET
  D = R._donor()
  T = R._toggles(D['frogpilotPlan'].frogpilotPlan.frogpilotToggles)
  tf = get_T_FOLLOW(T.aggressive_follow, T.standard_follow, T.relaxed_follow, T.custom_personalities, D['selfdriveState'].selfdriveState.personality,
                    v, experimental, True)
  return float(desired_follow_distance(v, v, 0.0, t_follow=tf, lead_stop_distance_target=LEAD_STOP_DISTANCE_TARGET))


def _family():
  """F1 case id -> (scenario params). The matrix: every id runs in every configuration (cell suffix) and lag cell."""
  out = {}
  for v in (20.0, 25.0, 30.0):
    for a in (2.0, 3.0, 4.0):
      out[f'f1hb_v{v:g}_a{a:g}'] = dict(kind='hb', v0=v, a=-a, v_end=v / 2)
  out['f1hb_v20_a3_stop'] = dict(kind='hb', v0=20.0, a=-3.0, v_end=0.0)
  for v in (20.0, 25.0):
    out[f'f1ab_v{v:g}_a3'] = dict(kind='ab', v0=v, a=-3.0, v_end=v / 2)
  for v in (20.0, 25.0):
    for g in (1.0, 1.5):
      out[f'f1cut_v{v:g}_h{g:g}_a3'] = dict(kind='cut', v0=v, h=g, vc=v - 3.0, a=-3.0, v_end=v / 2)
  for sw in ('cold', 'warm'):
    out[f'f1sw_v25_a3_{sw}'] = dict(kind='sw', v0=25.0, a=-3.0, v_end=12.5, sw=sw)
  out['f1l2_v25_a3'] = dict(kind='l2', v0=25.0, a=-3.0, v_end=10.0)
  out['f1rv_v25_a3'] = dict(kind='rv', v0=25.0, a=-3.0, v_end=12.5)
  return out


FAMILY = _family()
T_EV = 3.0   # s: the event (brake / cut-in) starts here; the planner and radard settle on the steady follow before it


def case(name, config='A'):
  """The synthetic case of F1 id `name` in configuration `config` (B / A / K)."""
  p = FAMILY[name]
  cf = CONFIGS[config]
  v0, kind = p['v0'], p['kind']
  gap0 = round(follow_distance(v0, cf['experimental']), 1)   # the planner's own steady follow distance at v0
  dur = round(T_EV + (3.0 if kind == 'ab' else 0.0) + (v0 + 3.0 - p['v_end']) / -p['a'] + DUR_TAIL, 1)
  t_ev = T_EV + 3.0 if kind == 'ab' else T_EV
  segs = [(t_ev, p['a'], p['v_end'])]
  sc = lambda f: (lambda t: f(t - R.SYN_T0))  # noqa: E731 -- track predicates on the route clock
  if kind == 'ab':   # both accelerate +1 for 3 s from T_EV (the ego follows: cruise allows v0 + 4), then the lead brakes
    lead = Obj(gap0, v0, [(T_EV, 1.0, v0 + 3.0), (t_ev, p['a'], p['v_end'])])
    objs, tracks = [lead], [(lead, lambda t: 7, None, 0.0)]
  elif kind == 'cut':   # a far lead (track 7) at v0; at T_EV a car cuts in h x v0 ahead of the ego at vc, already braking at a (track 8)
    far = Obj(gap0 + 60.0, v0)
    cut = Obj(0.0, p['vc'], [(T_EV - 0.5, p['a'], p['v_end'])], anchor=(T_EV, p['h'] * v0),
              visible=lambda ts: ts >= T_EV)
    objs = [far, cut]
    tracks = [(far, lambda t: 7, None, 0.0), (cut, lambda t: 8, sc(cut.visible), 0.0)]
  elif kind == 'sw':
    lead = Obj(gap0, v0, segs)
    t_sw = T_EV + 1.0
    if p['sw'] == 'cold':
      tracks = [(lead, sc(lambda ts: 7 if ts < t_sw else 11), None, 0.0)]
    else:   # a second return of the same car (track 9, 0.4 m further) filtered from the start; track 7 drops at t_sw
      tracks = [(lead, lambda t: 7, sc(lambda ts: ts < t_sw), 0.0), (lead, lambda t: 9, None, 0.4)]
    objs = [lead]
  elif kind == 'l2':   # lead 1 (track 7) leaves the lane from T_EV - 0.5 (y to 3.6 m over 2 s); lead 2 (track 8) 35 m ahead of it brakes
    lead1 = Obj(gap0, v0, [], y_ramp=(T_EV - 0.5, T_EV + 1.5, 3.6))
    lead2 = Obj(gap0 + 35.0, v0, segs)
    objs = [lead1, lead2]
    tracks = [(lead1, lambda t: 7, None, 0.0), (lead2, lambda t: 8, None, 0.0)]
  elif kind == 'rv':   # the radar loses the braking lead for 1 s (vision only), then a new track
    lead = Obj(gap0, v0, segs)
    t0, t1 = T_EV + 0.5, T_EV + 1.5
    tracks = [(lead, sc(lambda ts: 7 if ts < t0 else 12), sc(lambda ts: not t0 <= ts < t1), 0.0)]
    objs = [lead]
  else:
    lead = Obj(gap0, v0, segs)
    objs, tracks = [lead], [(lead, lambda t: 7, None, 0.0)]
  near = Nearest(objs)
  c = R.synthetic_case(None, v0=v0, gap0=gap0, duration=dur, lead_obj=near)
  c['id'], c['note'] = name, f'F1 {p} config {config} {cf}'
  c['f1'] = dict(objs=objs, t_ev=R.SYN_T0 + t_ev, config=config)
  c['fp_live'] = True   # rharness runs the tree's FrogPilotPlanner on the simulated state (following, CEM, limits, traffic)

  def radar_objects(td, tv, r):
    out = []
    for o, tid, vis, off in tracks:
      if vis is None or vis(td):
        out.append((tid(td), o.x_at(td) + off, -o.y_at(td), o.v_at(tv)))
    return out
  c['radar_objects'] = radar_objects
  v_set = v0 + (4.0 if kind == 'ab' else 1.0)   # cruise just above the follow speed (the lead, not the set speed, limits)
  for f in c['frames']:   # LongControl's mode and cruise speed (km/h)
    f['kw'] = dict(f['kw'], experimental_mode=cf['experimental'])
    f['cs'] = dict(f['cs'], vCruise=v_set * 3.6)
  E = c['stream']
  orig = E['synth_msgs']

  def synth_msgs(md_t, msgs, ego_v, ego_a, gap):
    from openpilot.selfdrive.modeld.constants import ModelConstants
    out = orig(md_t, msgs, ego_v, ego_a, gap)
    m = out['modelV2'].modelV2
    nb = [o for o in objs if abs(o.y_at(md_t)) < LANE_Y and o.x_at(md_t) > near.x_at(md_t) + 1.0]
    for i, L in enumerate(m.leadsV3):
      L.y = [0.0] * len(L.y)
      if i == 1 and nb:   # the second in-lane car (radard's leadTwo needs a model lead to match)
        o = min(nb, key=lambda q: q.x_at(md_t))
        L.prob = 0.9
        L.x = [o.x_at(md_t) - (near.x_at(md_t) - gap) + 1.52 + o.v_at(md_t) * tt for tt in ModelConstants.LEAD_T_IDXS]
        L.v = [o.v_at(md_t)] * len(L.v)
    # the configuration's inputs; FrogPilotPlanner (rharness fp_tick) computes following, CEM and the limits from them
    out['selfdriveState'].selfdriveState.experimentalMode = cf['experimental']   # the ExperimentalMode setting (CEM off)
    fp = out['frogpilotPlan'].frogpilotPlan
    fp.vCruise = v_set
    tg = json.loads(fp.frogpilotToggles or '{}')
    tg['human_following'] = cf['human_following']
    tg['conditional_experimental_mode'] = cf['experimental']   # B = the car: Experimental + CEM (CEM decides the mode)
    fp.frogpilotToggles = json.dumps(tg, sort_keys=True)
    out['frogpilotCarState'].frogpilotCarState.trafficModeEnabled = False
    out['carState'].carState.vCruise = v_set * 3.6
    return out
  E['synth_msgs'] = synth_msgs
  return c


def is_f1(cid):
  return cid in FAMILY


def jobs():
  """(case, cell, start, mode): every F1 id x config at the base lags, + the lag cells on configuration A."""
  out = [(cid, f'F1{cf}', 'auto', 'drv') for cid in FAMILY for cf in CONFIGS]
  out += [(cid, f'F1A_lv{lag:g}', 'auto', 'drv') for cid in FAMILY for lag in LAGS]
  return out


def cell_opts(cell):
  """(configuration, run opts) of an F1 cell name 'F1<config>[_lv<lag_v>]'."""
  cf, _, lag = cell[2:].partition('_lv')
  return cf, ({'lag_v': float(lag)} if lag else {})


def measures(tr, c):
  """Physical-truth measures of one F1 run (trace columns at 100 Hz from rharness)."""
  t = np.asarray(tr['t'], dtype=float) + R.SYN_T0
  x = np.asarray(tr['x'], dtype=float)
  v = np.asarray(tr['v_true'], dtype=float)
  sent = np.asarray(tr['sent'], dtype=float)
  objs = c['f1']['objs']
  gaps, vl = [], []
  for o in objs:
    xo = np.interp(t, o.t, o.x)
    inl = np.array([abs(o.y_at(q)) < LANE_Y and o.seen(q) for q in t])
    gaps.append(np.where(inl, xo - x, np.inf))
    vl.append(np.interp(t, o.t, o.v))
  G = np.vstack(gaps)
  k_near = np.argmin(G, axis=0)
  gap = G[k_near, np.arange(len(t))]
  v_lead = np.vstack(vl)[k_near, np.arange(len(t))]
  closing = v - v_lead
  ev = t >= c['f1']['t_ev']
  ttc = np.where((closing > 0.05) & np.isfinite(gap), gap / np.maximum(closing, 1e-3), np.inf)
  k_ev = int(np.argmax(ev))
  on = None
  hold = int(round(ONSET_S / 0.01))
  below = sent <= ONSET_A
  for k in range(k_ev, len(t) - hold):
    if below[k:k + hold].all():
      on = k
      break
  rel = None
  if on is not None:
    up = np.flatnonzero(sent[on:] >= RELEASE_A)
    rel = on + int(up[0]) if len(up) else None
  hit = np.flatnonzero(ev & (gap <= 0.0))
  k_hit = int(hit[0]) if len(hit) else len(t)   # objects pass through the ego: nothing after the contact is physical
  sc = ev & (np.arange(len(t)) <= k_hit)
  g_ev = gap[sc]
  return dict(min_gap=float(max(np.min(g_ev), 0.0)), t_min_gap=float(t[sc][np.argmin(g_ev)] - R.SYN_T0), min_ttc=float(max(np.min(ttc[sc]), 0.0)),
              max_closing=float(np.max(closing[sc])), collision=bool(len(hit)), impact_v=float(closing[k_hit]) if len(hit) else 0.0,
              t_impact=float(t[k_hit] - c['f1']['t_ev']) if len(hit) else None,
              onset=None if on is None else float(t[on] - c['f1']['t_ev']), release=None if rel is None else float(t[rel] - c['f1']['t_ev']),
              min_sent=float(np.min(sent[ev])), t=(t[ev] - R.SYN_T0).astype(np.float32)[::2], sent=sent[ev].astype(np.float32)[::2],
              gap=g_ev.astype(np.float32)[::2], v=v[ev].astype(np.float32)[::2],
              lead_v=np.asarray(tr['lead_v'], dtype=float)[ev].astype(np.float32)[::2], v_lead=v_lead[ev].astype(np.float32)[::2])
