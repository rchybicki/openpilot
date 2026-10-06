"""Radar stage of the exact replay (cycle_20261006 PLAN section 5, builder R): the tree's radard (RadarD.update, production code) and
the FrogPilot lead consumers (FrogPilotPlanner.update / publish with the non-lead parts frozen to the log) re-run on the logged
inputs of a span, plus the M1 truth of the published leads.

radard (T1): one RadarD(CP.radarDelay of the arm's tree: CarInterface.get_non_essential_params(<logged fingerprint>)) per span,
started RADAR_WARM s before the planner history so the track filters, v_ego history and lane-change surrogate state are warm (the
track KF error shrinks x0.895 per 50 ms tick). One update per logged radarState (radard's own cadence); its inputs are the ones that
radarState names (modelV2 = mdMonoTime, carState = carStateMonoTime), liveTracks = the latest one card sent no later than that
carState's cycle (card sends liveTracks right after carState) and published before the radarState, frogpilotPlan = the latest
published before the radarState. The re-run radarState keeps the logged logMonoTime and valid bit (SubMaster alive / frequency
checks are not replayable). Fidelity: every re-run tick vs the logged radarState (discrete fields exact, floats within FLOAT_TOL;
KF state within KF_TOL on race-affected ticks only).

FrogPilot (T3): per logged frogpilotPlan tick, FrogPilotPlanner.update + publish (the tree's code) on an instance whose lead
consumers are real (CEM, following, traffic controller, acceleration limits, events incl. the lead-departing alert, tracking lead)
and whose speed-limit / curve / weather / GPS / params parts replay the logged frogpilotPlan values. Inputs: the arm's re-run
radarState and the logged other services (latest published before the modelV2 that triggered the tick + LAG_NS)."""
import bisect
import inspect
from pathlib import Path
from types import SimpleNamespace

import numpy as np

RADAR_WARM = 10.0          # s of radard history before the planner pass (KF error x0.895 per tick: 1e-9 after 10 s)
FP_WARM = 5.0              # s of FrogPilot planner history before the planner pass (filters RC 0.5 s; traffic t_follow rate)
FLOAT_TOL = 1e-4           # fidelity: |re-run - logged| per float field (m, m/s, m/s^2) on a matching build
LEAD_F = ('status', 'radar', 'radarTrackId', 'fcw', 'dRel', 'yRel', 'vRel', 'vLead', 'vLeadK', 'aLeadK', 'aLeadTau', 'modelProb')
LEAD_DISCRETE = ('status', 'radar', 'radarTrackId', 'fcw')
KF_F = ('vLeadK', 'aLeadK', 'aLeadTau')   # track filter state (memory of the pre-log history)
# KF state of a track that became the lead after a liveTracks race on a background track (no logged lead can settle it: the 0.2 ms
# carState / liveTracks race of card): its filter carries the other sample for ~2 s (x0.895 per tick). Declared tolerance for KF_F,
# only on race-affected ticks: the lead's track (re-run or logged) had differing values among the race's candidate liveTracks
# messages within the last KF_RACE_S; every other tick keeps FLOAT_TOL.
KF_TOL = 0.05              # m/s, m/s^2, s
KF_RACE_S = 2.0            # s
SURR_F = ('leadOneSurrogate', 'leadTwoSurrogate')
# frogpilotPlan fields the lead consumers write (the planner / selfdrived read them); the rest of the message is the log's
FP_F = ('trackingLead', 'experimentalMode', 'redLight', 'tFollow', 'desiredFollowDistance', 'dangerFactor', 'accelerationJerk',
        'dangerJerk', 'speedJerk', 'maxAcceleration', 'minAcceleration')
FP_DISCRETE = ('trackingLead', 'experimentalMode', 'redLight', 'desiredFollowDistance')
FP_SERV = ('carControl', 'carState', 'controlsState', 'liveParameters', 'modelV2', 'selfdriveState', 'frogpilotCarState',
           'frogpilotSelfdriveState', 'frogpilotModelV2')
LAG_NS = 3_000_000         # FrogPilot pass: services published up to 3 ms after the triggering modelV2 (the planner lockstep rule)


def radar_delay(fingerprint):
  """CP.radarDelay of the process's tree for the logged fingerprint (where radard reads it: CarParams from get_params, whose
  non-essential part is get_non_essential_params)."""
  from opendbc.car.car_helpers import interfaces
  return float(interfaces[fingerprint].get_non_essential_params(fingerprint).radarDelay)


def radard_kw(radard, fingerprint):
  """RadarD's keyword arguments as radard.main passes them in the arm's tree: the car fingerprint where RadarD takes it (the Santa Fe
  HEV radar time alignment, cycle_20261006 phase D; older trees construct RadarD(delay) only)."""
  return {'car_fingerprint': fingerprint} if 'car_fingerprint' in inspect.signature(radard.RadarD).parameters else {}


def latest(by, s, tns):
  i = bisect.bisect_right(by[s][0], tns) - 1
  return by[s][1][i] if i >= 0 else None


def _at(by, s, tns):
  """The event of stream s with exactly this logMonoTime (None if absent)."""
  i = bisect.bisect_left(by[s][0], tns)
  return by[s][1][i] if i < len(by[s][0]) and by[s][0][i] == tns else None


class _RSM:
  """radard's SubMaster on one replayed tick (modelV2 poll; carState, liveTracks, frogpilotPlan as received)."""
  def __init__(self, msgs, seen, recv_frame, valid):
    self.msgs = msgs
    self.logMonoTime = {s: e.logMonoTime for s, e in msgs.items()}
    self.seen = seen
    self.recv_frame = recv_frame
    self._valid = valid

  def __getitem__(self, s):
    return getattr(self.msgs[s], s)

  def all_checks(self, service_list=None):
    return self._valid


def _votes(leads, tracks, isd):
  """How well a liveTracks message reproduces the logged radar leads (leadOne / leadTwo + the adjacent leads of frogpilotRadarState):
  per lead track present, one vote each for the same lateral position, relative speed and (published) distance."""
  from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.stop_target_helpers import get_published_lead_distance
  pts = {p.trackId: (p.yRel, p.vRel, p.dRel) for p in tracks.points}
  n = 0
  for lead, published in leads:   # each field that matches counts (a lane-change surrogate keeps only yRel)
    p = pts.get(lead.radarTrackId) if lead.status and lead.radar else None
    if p is not None:
      d = get_published_lead_distance(p[2], isd) if published else p[2]
      n += (abs(p[0] - lead.yRel) < 1e-4) + (abs(p[1] - lead.vRel) < 1e-4) + (abs(d - lead.dRel) < 1e-4)
  return n


def logged_frs(by, R):
  """The frogpilotRadarState radard sent right after the logged radarState event R (None if absent)."""
  frs_ns = by['frogpilotRadarState'][0]
  i = bisect.bisect_left(frs_ns, R.logMonoTime)
  return by['frogpilotRadarState'][1][i].frogpilotRadarState if i < len(frs_ns) and frs_ns[i] - R.logMonoTime < 10_000_000 else None


def live_tracks_index(by, R, cs_ns):
  """Index into by['liveTracks'] of the message radard read for the logged radarState event R (-1: none) whose carState has
  logMonoTime cs_ns, and whether the logged leads settled a race: the latest one card sent no later than that carState's cycle
  (card sends liveTracks right after carState) and published before R; where several messages are possible (sent around the
  carState radard read, 0.2 ms apart, or between two socket reads of one drain) the one whose tracks reproduce more of the logged
  leads (leadOne / leadTwo + the adjacent leads of frogpilotRadarState; _votes) wins. The exact replay and the closed loop share it.
  Returns (index, race settled by the logged leads, the track ids whose values differ among the candidate messages)."""
  cs_t, lt_ns = by['carState'][0], by['liveTracks'][0]
  k = bisect.bisect_right(cs_t, cs_ns)
  cycle_end = cs_t[k] if k < len(cs_t) else R.logMonoTime
  j = bisect.bisect_right(lt_ns, min(cycle_end - 1, R.logMonoTime)) - 1
  i0, i1 = max(bisect.bisect_right(lt_ns, cs_ns) - 1, 0), bisect.bisect_right(lt_ns, R.logMonoTime)
  if i1 - i0 <= 1:
    return j, False, frozenset()
  cand = [{p.trackId: (p.dRel, p.yRel, p.vRel) for p in by['liveTracks'][1][q].liveTracks.points} for q in range(i0, i1)]
  amb = frozenset(t for t in set().union(*cand) if len({c.get(t) for c in cand}) > 1)
  rs = R.radarState
  lf = logged_frs(by, R)
  leads = [(rs.leadOne, True), (rs.leadTwo, True)] + ([(lf.leadLeft, False), (lf.leadRight, False)] if lf is not None else [])
  fp = latest(by, 'frogpilotPlan', R.logMonoTime)
  isd = fp.frogpilotPlan.increasedStoppedDistance if fp is not None else 0.0
  votes = [_votes(leads, by['liveTracks'][1][q].liveTracks, isd) for q in range(i0, i1)]
  best = i0 + max(range(len(votes)), key=lambda q: votes[q])
  if best != j and votes[best - i0] > (_votes(leads, by['liveTracks'][1][j].liveTracks, isd) if j >= 0 else 0):
    return best, True, amb
  return j, False, amb


def _lead_vals(lead):
  return [float(getattr(lead, f)) for f in LEAD_F]


def radard_pass(by, t_lo, t_hi, fingerprint):
  """The tree's RadarD on the logged inputs of every logged radarState in [t_lo - RADAR_WARM, t_hi] (ns). Returns (msgs, out):
  msgs = {logged radarState logMonoTime: re-run radarState event (owned reader)}, out = arrays per tick: ns, the re-run and the
  logged leads (rs_<lead>_<field>, log_<lead>_<field>), surrogate flags, input-selection counts and the delay used."""
  from cereal import log, messaging
  from openpilot.selfdrive.controls import radard
  from openpilot.frogpilot.common.frogpilot_variables import get_frogpilot_toggles
  delay = radar_delay(fingerprint)
  rd = None
  seen = dict.fromkeys(('modelV2', 'carState', 'liveTracks', 'frogpilotPlan'), False)
  empty_lt = messaging.new_message('liveTracks').as_reader()
  first_fp = next((e for e in by['frogpilotPlan'][1] if e.frogpilotPlan.frogpilotToggles), None)
  if first_fp is None:
    raise ValueError('no logged frogpilotPlan with toggles: radard cannot be replayed')
  rows, msgs, pts, ambs, n_skip, n_lt_none, n_race, n_no_toggles = [], {}, [], [], 0, 0, 0, 0
  w_lo = t_lo - int(RADAR_WARM * 1e9)
  for R in by['radarState'][1]:
    if not w_lo <= R.logMonoTime <= t_hi:
      continue
    rs = R.radarState
    md, cs = _at(by, 'modelV2', rs.mdMonoTime), _at(by, 'carState', rs.carStateMonoTime)
    if md is None or cs is None:   # an input the log does not have: the tick cannot be replayed
      n_skip += 1
      continue
    j, raced, amb = live_tracks_index(by, R, cs.logMonoTime)
    lt = by['liveTracks'][1][j] if j >= 0 else None
    fp = latest(by, 'frogpilotPlan', R.logMonoTime)
    lf = logged_frs(by, R)
    n_race += raced
    n_lt_none += lt is None
    seen.update(modelV2=True, carState=True, liveTracks=seen['liveTracks'] or lt is not None, frogpilotPlan=seen['frogpilotPlan'] or fp is not None)
    if fp is None or not fp.frogpilotPlan.frogpilotToggles:   # before the first toggles: the first logged ones (the device would
      fp = first_fp                                            # read its Params through FrogPilotVariables)
      n_no_toggles += 1
    sm_msgs = dict(modelV2=md, carState=cs, liveTracks=lt if lt is not None else empty_lt, frogpilotPlan=fp)
    if rd is None:   # radard's constructor reads the toggles from an empty SubMaster (FrogPilotVariables: device Params); give it
      toggles = get_frogpilot_toggles(_RSM(sm_msgs, seen, {}, R.valid))   # the first logged ones (update reads them every tick)
      orig, radard.get_frogpilot_toggles = radard.get_frogpilot_toggles, lambda *a, _t=toggles: _t
      try:
        rd = radard.RadarD(delay, **radard_kw(radard, fingerprint))
      finally:
        radard.get_frogpilot_toggles = orig
    sm = _RSM(sm_msgs, dict(seen), dict(carState=cs.logMonoTime), R.valid)
    rd.update(sm, sm['liveTracks'])
    ev = log.Event.new_message(radarState=rd.radar_state, valid=R.valid, logMonoTime=R.logMonoTime)
    out_ev = ev.as_reader()
    msgs[R.logMonoTime] = out_ev
    rows.append([R.logMonoTime, *_lead_vals(out_ev.radarState.leadOne), *_lead_vals(out_ev.radarState.leadTwo),
                 *_lead_vals(rs.leadOne), *_lead_vals(rs.leadTwo),
                 float(rd.frogpilot_radar_state.leadOneSurrogate), float(rd.frogpilot_radar_state.leadTwoSurrogate),
                 *(float(getattr(lf, f)) if lf is not None else np.nan for f in SURR_F), cs.carState.vEgo, cs.carState.aEgo])
    pts.append({p.trackId: (p.dRel, p.yRel, p.vRel) for p in sm['liveTracks'].points})
    ambs.append(amb)
  cols = (['ns'] + [f'rs_{w}_{f}' for w in ('l1', 'l2') for f in LEAD_F] + [f'log_{w}_{f}' for w in ('l1', 'l2') for f in LEAD_F]
          + [f'rs_{f}' for f in SURR_F] + [f'log_{f}' for f in SURR_F] + ['v_ego', 'a_ego'])
  arr = np.array(rows, dtype=np.float64).reshape(-1, len(cols))
  out = {c: arr[:, i] for i, c in enumerate(cols)}
  out['ns'] = arr[:, 0].astype(np.int64)
  out.update(truth(out, pts))
  ns, k_amb = out['ns'], [k for k, a in enumerate(ambs) if a]
  for w in ('l1', 'l2'):   # race-affected ticks per lead (KF_TOL applies there only)
    race = np.zeros(len(ns))
    for k in k_amb:
      for q in range(k, bisect.bisect_right(ns, ns[k] + int(KF_RACE_S * 1e9))):
        race[q] = race[q] or bool({int(out[f'rs_{w}_radarTrackId'][q]), int(out[f'log_{w}_radarTrackId'][q])} & ambs[k])
    out[f'race_{w}'] = race
  out['delay'] = np.float64(delay)
  out['first_ns'] = np.int64(out['ns'][0] if len(out['ns']) else 0)   # the re-run's first tick (its filters start cold there)
  out['n_skip'] = np.int64(n_skip)
  out['n_lt_none'] = np.int64(n_lt_none)
  out['n_race'] = np.int64(n_race)
  out['n_no_toggles'] = np.int64(n_no_toggles)
  return msgs, out


# ---- M1 truth (T6) --------------------------------------------------------------------------------------------------------
# Lead ground speed at a radard tick t = scale x the lead track's REL_SPEED at t + L (the radar reports it L late) / cos(azimuth)
# + the ego speed radard used at t (cycle_20261002 pass2.py: radar right but late; AZIMUTH reads ~2x the true angle). L and scale
# are uncertain: the envelope spans the measured REL_SPEED latency p10..p90 (0.10..0.21 s on liveTracks, + 0.026 s radard
# staleness) and the radar-vs-wheel scale (~1 %). Stationary leads: geometry (the lead's world position from the range, aligned by
# the range lag, + the ego travel) stays within STATIC_SPAN over +-1 s -> truth 0 exactly, no lag model.
LAGS = (0.13, 0.18, 0.24)       # s, REL_SPEED lag at radarState time (p10, median, p90)
SCALES = (0.99, 1.0, 1.01)
LAG_D = 0.13                    # s, LONG_DIST lag (static geometry)
STATIC_SPAN, STATIC_HALF = 0.3, 1.0   # m, s
A_HALF = 0.25                   # s, truth acceleration: centred difference of the L=0.18 truth speed
ONSET_S, SIGN_S = 1.0, 0.5      # s: braking onset window; accel sign-change window (+-)


def truth(out, pts):
  """Per lead (l1, l2) per tick the truth envelope of the re-run lead's track: tr_<w>_v<i><j> (LAGS[i], SCALES[j]; NaN where the
  track ends before t + L), tr_<w>_a (acceleration, NaN where the window leaves the track), tr_<w>_static (geometry-qualified
  stationary), tr_<w>_age (s since the track appeared), tr_<w>_onset (within ONSET_S of a braking onset: a > -0.3 -> <= -0.5),
  tr_<w>_sign (an accel sign change >= +0.3 <-> <= -0.3 within +-SIGN_S). Vision leads and surrogates get NaN / False."""
  t = np.asarray(out['ns'], dtype=np.float64) * 1e-9
  ve = out['v_ego']
  s_ego = np.r_[0.0, np.cumsum(np.diff(t) * 0.5 * (ve[1:] + ve[:-1]))] if len(t) else np.zeros(0)
  series: dict = {}
  for k, p in enumerate(pts):
    for tid, v in p.items():
      series.setdefault(tid, []).append((k, *v))
  runs = {}
  for tid, lst in series.items():
    a = np.array(lst, dtype=np.float64)
    for seg in np.split(a, np.flatnonzero(np.diff(a[:, 0]) > 1) + 1):
      ks = seg[:, 0].astype(int)
      tt, d, y, vr = t[ks], seg[:, 1], seg[:, 2], seg[:, 3]
      vc = vr / (d / np.sqrt(d ** 2 + 4 * y ** 2))
      vl = np.where(tt + LAGS[1] <= tt[-1], np.interp(tt + LAGS[1], tt, vc) + ve[ks], np.nan)
      ok = (tt - A_HALF >= tt[0]) & (tt + A_HALF + LAGS[1] <= tt[-1])
      al = np.where(ok, (np.interp(tt + A_HALF, tt, np.nan_to_num(vl)) - np.interp(tt - A_HALF, tt, np.nan_to_num(vl))) / (2 * A_HALF), np.nan)
      xw = np.interp(tt + LAG_D, tt, d) + s_ego[ks]
      for n, k in enumerate(ks):
        runs[(tid, int(k))] = (tt, vc, al, xw, n, ks)
  res = {}
  for w in ('l1', 'l2'):
    n = len(t)
    v = np.full((len(LAGS), len(SCALES), n), np.nan)
    acc, age, static, onset, sign = np.full(n, np.nan), np.full(n, np.nan), np.zeros(n, bool), np.zeros(n, bool), np.zeros(n, bool)
    for k in range(n):
      if not (out[f'rs_{w}_status'][k] > 0 and out[f'rs_{w}_radar'][k] > 0):
        continue
      hit = runs.get((int(out[f'rs_{w}_radarTrackId'][k]), k))
      if hit is None:   # a surrogate / filtered lead whose id is not a live point this tick
        continue
      tt, vc, al, xw, j, ks = hit
      for i, L in enumerate(LAGS):
        if t[k] + L <= tt[-1]:
          v[i, :, k] = np.asarray(SCALES) * np.interp(t[k] + L, tt, vc) + ve[k]
      acc[k], age[k] = al[j], t[k] - tt[0]
      win = (tt >= t[k] - STATIC_HALF) & (tt <= t[k] + STATIC_HALF) & (tt + LAG_D <= tt[-1])
      static[k] = t[k] - STATIC_HALF >= tt[0] and t[k] + STATIC_HALF + LAG_D <= tt[-1] and np.ptp(xw[win]) < STATIC_SPAN
      back = (tt >= t[k] - ONSET_S) & (tt <= t[k])
      onset[k] = al[j] <= -0.5 and bool(np.any(al[back] > -0.3))
      near = (tt >= t[k] - SIGN_S) & (tt <= t[k] + SIGN_S)
      sign[k] = bool(np.any(al[near] >= 0.3)) and bool(np.any(al[near] <= -0.3))
    for i in range(len(LAGS)):
      for jj in range(len(SCALES)):
        res[f'tr_{w}_v{i}{jj}'] = v[i, jj]
    res.update({f'tr_{w}_a': acc, f'tr_{w}_age': age, f'tr_{w}_static': static.astype(np.float64), f'tr_{w}_onset': onset.astype(np.float64),
                f'tr_{w}_sign': sign.astype(np.float64)})
  return res


M1_KEEP = ('status', 'radar', 'radarTrackId', 'vLead', 'vLeadK', 'aLeadK')


def m1_rows(R, lo_ns, hi_ns, o0):
  """The scored window's published leads + truth (float32) for the M1 gate: t (s from the log origin), a_ego and per lead the
  published M1_KEEP fields and the tr_ arrays."""
  sc = (np.asarray(R['ns']) >= lo_ns) & (np.asarray(R['ns']) <= hi_ns)
  out = dict(t=((np.asarray(R['ns'])[sc] - o0) * 1e-9).astype(np.float64), a_ego=R['a_ego'][sc].astype(np.float32))
  for w in ('l1', 'l2'):
    out.update({f'{w}_{f}': R[f'rs_{w}_{f}'][sc].astype(np.float32) for f in M1_KEEP})
    out.update({k[3:]: v[sc].astype(np.float32) for k, v in R.items() if k.startswith(f'tr_{w}_')})
  return out


def arm_subst(by, F, FB):
  """The logged frogpilotPlan / selfdriveState events an arm with a reference reads instead (planner_pass subst): on ticks where the
  arm's replayed lead-consumer fields differ from the reference arm's (FB), the floats move by (arm - reference) and the discrete
  fields take the arm's value; selfdriveState.experimentalMode follows the frogpilotPlan selfdrived read before it."""
  assert np.array_equal(F['ns'], FB['ns']), 'frogpilotPlan ticks differ from the reference arm'
  diff = np.zeros(len(F['ns']), bool)
  for f in FP_F:
    diff |= F[f'fp_{f}'] != FB[f'fp_{f}']
  idx = {int(n): i for i, n in enumerate(F['ns'])}
  fp, sds = {}, {}
  for e in by['frogpilotPlan'][1]:
    i = idx.get(e.logMonoTime)
    if i is None or not diff[i]:
      continue
    b = e.as_builder()
    for f in FP_F:
      a, r = F[f'fp_{f}'][i], FB[f'fp_{f}'][i]
      if a != r:
        setattr(b.frogpilotPlan, f, (int(a) if f == 'desiredFollowDistance' else bool(a)) if f in FP_DISCRETE
                else float(getattr(e.frogpilotPlan, f) + (a - r)))
    fp[e.logMonoTime] = b.as_reader()
  em = F['fp_experimentalMode'] != FB['fp_experimentalMode']
  if em.any():
    fpn = np.asarray(F['ns'])
    for e in by['selfdriveState'][1]:
      q = int(np.searchsorted(fpn, e.logMonoTime, side='right')) - 1
      if q >= 0 and em[q]:
        b = e.as_builder()
        b.selfdriveState.experimentalMode = bool(F['fp_experimentalMode'][q])
        sds[e.logMonoTime] = b.as_reader()
  return {'frogpilotPlan': fp, 'selfdriveState': sds}


def fidelity(R, pre_ns, lo_ns, hi_ns):
  """Re-run vs logged radarState: per field the ticks that differ (discrete: any; float: > FLOAT_TOL, or not finite) inside the scored
  window [lo_ns, hi_ns] (the replayed frames) and in the planner history before it [pre_ns, lo_ns) (the radard warm-up before pre_ns is
  not compared), the worst float error, the first scored mismatch. KF-state fields: KF_TOL on race-affected ticks (race_<lead>; the
  ticks it tolerates and their worst error are reported), FLOAT_TOL elsewhere. A KF mismatch less than RADAR_WARM after the re-run's
  first tick (the log starts late: the filters' earlier state is unknown) is a mismatch like any other, also on a race-affected tick
  (cold_kf_scored counts them)."""
  ns = np.asarray(R['ns'])
  sc = (ns >= lo_ns) & (ns <= hi_ns)
  bad = np.zeros(len(ns), bool)
  ns_ok = ns >= pre_ns
  cold = ns < int(R['first_ns']) + int(RADAR_WARM * 1e9)
  per, worst, n_cold, tol_n, tol_max = {}, {}, 0, 0, 0.0
  for w in ('l1', 'l2'):
    race = np.asarray(R.get(f'race_{w}', np.zeros(len(ns)))) > 0
    for f in LEAD_F:
      a, b = R[f'rs_{w}_{f}'], R[f'log_{w}_{f}']
      on = (R[f'rs_{w}_status'] > 0) | (R[f'log_{w}_status'] > 0)   # a lead's values matter only where one of the two has it
      if f in LEAD_DISCRETE:
        d = a != b
      else:
        e = np.where(on, np.abs(a - b), 0.0)
        d = ~(e <= FLOAT_TOL)   # NaN fails
        if f in KF_F:   # not on cold ticks: there the filters' earlier state is unknown, so a mismatch is not attributable to the race
          ok = race & ~cold & (e <= KF_TOL)
          tol = ok & d & on & ns_ok & sc
          tol_n, tol_max = tol_n + int(tol.sum()), max(tol_max, float(np.max(e[tol])) if tol.any() else 0.0)
          d &= ~ok
        worst[f'{w}_{f}'] = float(np.max(e[sc])) if sc.any() else 0.0
      d &= on if f not in ('status',) else True
      d &= ns_ok
      if f in KF_F:
        n_cold += int(np.sum(d & cold & sc))
      per[f'{w}_{f}'] = (int(np.sum(d & sc)), int(np.sum(d & ~sc)))
      bad |= d
  for f in SURR_F:
    lg = R[f'log_{f}']
    d = np.isfinite(lg) & (R[f'rs_{f}'] != lg) & ns_ok
    per[f] = (int(np.sum(d & sc)), int(np.sum(d & ~sc)))
    bad |= d
  k = np.flatnonzero(bad & sc)
  return dict(ticks=int(ns_ok.sum()), scored=int(sc.sum()), cold_kf_scored=n_cold, bad_scored=int(np.sum(bad & sc)), bad_pre=int(np.sum(bad & ~sc)),
              race_tol_scored=tol_n, race_tol_max=round(tol_max, 6),
              per_field={k_: v for k_, v in per.items() if v[0] or v[1]}, worst={k_: round(v, 6) for k_, v in worst.items()},
              first_bad=int(ns[k[0]]) if len(k) else None, delay=float(R['delay']), n_skip=int(R['n_skip']), n_lt_none=int(R['n_lt_none']),
              n_race=int(R['n_race']), n_no_toggles=int(R['n_no_toggles']))


class _Store:
  """Params / params_memory of the replayed FrogPilot planner (nothing persists; the log has no CEStatus history)."""
  def __init__(self):
    self.d = {}

  def get(self, k, *a, **kw):
    return self.d.get(k)

  def get_bool(self, k, *a, **kw):
    return bool(self.d.get(k))

  def put(self, k, v, *a, **kw):
    self.d[k] = v

  put_bool = put
  put_nonblocking = put
  put_bool_nonblocking = put


class _VCruise:
  """FrogPilotVCruise frozen to the log: each update takes the values the logged frogpilotPlan of this tick published (speed-limit
  and curve controllers read map / GPS data the replay does not have). CEM reads them before update, i.e. the previous tick's."""
  def __init__(self):
    self.csc_controlling_speed, self.csc_target, self.forcing_stop, self.tracked_model_length = False, 0.0, False, 0.0
    self.slc_target, self.slc_offset = 0.0, 0.0
    self.slc = SimpleNamespace(experimental_mode=False, overridden_speed=0.0, map_speed_limit=0.0, mapbox_limit=0.0, next_speed_limit=0.0,
                               source='', speed_limit_changed_timer=0.0, unconfirmed_speed_limit=0.0)
    self.csc = SimpleNamespace(enable_training=False)
    self.logged = None

  def update(self, *a):
    p = self.logged
    self.csc_controlling_speed, self.csc_target = p.cscControllingSpeed, p.cscSpeed
    self.forcing_stop, self.tracked_model_length = p.forcingStop, p.forcingStopLength
    self.slc_target, self.slc_offset = p.slcSpeedLimit, p.slcSpeedLimitOffset
    self.slc.overridden_speed, self.slc.map_speed_limit, self.slc.mapbox_limit = p.slcOverriddenSpeed, p.slcMapSpeedLimit, p.slcMapboxSpeedLimit
    self.slc.next_speed_limit, self.slc.source, self.slc.unconfirmed_speed_limit = p.slcNextSpeedLimit, p.slcSpeedLimitSource, p.unconfirmedSlcSpeedLimit
    self.slc.speed_limit_changed_timer = 0.0
    self.csc.enable_training = p.cscTraining
    return p.vCruise


class _FSM:
  """The FrogPilot process's SubMaster on one replayed tick (a stream the log lacks reads as an empty message)."""
  def __init__(self, msgs, gps):
    self.msgs = msgs
    self.gps = gps

  def __getitem__(self, s):
    if s == self.gps:
      return SimpleNamespace(latitude=0.0, longitude=0.0, bearingDeg=0.0)
    return getattr(self.msgs[s], s)

  def all_checks(self, service_list=None):
    return all(self.msgs[s].valid for s in (service_list or self.msgs))


def _fp_vals(p):
  return [float(getattr(p, f)) for f in FP_F] + [float(any(e.name.raw == LEAD_DEPARTING for e in p.frogpilotEvents))]


LEAD_DEPARTING = 7   # custom.FrogPilotEventName.leadDeparting


def fp_planner():
  """The tree's FrogPilotPlanner (FrogPilotPlanner.__init__ without device Params / GPS / theme): lead consumers real (CEM, following,
  traffic controller, acceleration limits, events), speed-limit / curve controllers frozen to a given frogpilotPlan (_VCruise), no
  weather. Used by the exact replay (frogpilot_pass) and the synthetic closed loop (rharness, F1)."""
  from openpilot.common.filter_simple import FirstOrderFilter
  from openpilot.common.realtime import DT_MDL
  from openpilot.frogpilot.controls import frogpilot_planner as FPM
  from openpilot.frogpilot.controls.lib.conditional_experimental_mode import ConditionalExperimentalMode
  from openpilot.frogpilot.controls.lib.frogpilot_acceleration import FrogPilotAcceleration
  from openpilot.frogpilot.controls.lib.frogpilot_events import FrogPilotEvents
  from openpilot.frogpilot.controls.lib.frogpilot_following import FrogPilotFollowing
  from openpilot.frogpilot.controls.lib.frogpilot_traffic import FrogPilotTraffic
  P = FPM.FrogPilotPlanner.__new__(FPM.FrogPilotPlanner)   # FrogPilotPlanner.__init__ without device Params / GPS / theme
  P.params, P.params_memory = _Store(), _Store()
  P.frogpilot_acceleration = FrogPilotAcceleration(P)
  P.frogpilot_cem = ConditionalExperimentalMode(P)
  P.frogpilot_events = FrogPilotEvents(P, Path('/nonexistent/frogpilot_replay/error.txt'), None)
  P.frogpilot_following = FrogPilotFollowing(P)
  P.frogpilot_traffic = FrogPilotTraffic()
  P.frogpilot_vcruise = _VCruise()
  P.frogpilot_weather = SimpleNamespace(weather_id=0, is_daytime=False, increase_following_distance=0.0, increase_stopped_distance=0.0,
                                        reduce_acceleration=0.0, update_weather=lambda *a: None)
  P.driving_in_curve = P.gps_valid = P.lateral_check = P.model_stopped = P.id_test_armed = False
  P.not_leftmost_lane = P.road_curvature_detected = P.tracking_lead = False
  P.lane_width_left = P.lane_width_right = P.lateral_acceleration = P.model_length = P.road_curvature = P.time_to_curve = P.v_cruise = 0
  P.gps_position = None
  P.gps_location_service = 'gpsLocationExternal'
  P.tracking_lead_filter = FirstOrderFilter(0, 0.5, DT_MDL)
  return P


def fp_step(P, msgs, plan):
  """One FrogPilotPlanner.update + publish on msgs (FP_SERV + radarState), the frozen parts read from frogpilotPlan `plan` (its toggles,
  vCruise, speed-limit / curve values, weather id). Returns the published frogpilotPlan event (builder)."""
  from openpilot.frogpilot.common.frogpilot_variables import process_frogpilot_toggles
  toggles = process_frogpilot_toggles(plan.frogpilotToggles)
  P.frogpilot_vcruise.logged = plan
  P.frogpilot_weather.weather_id = plan.weatherId
  sm = _FSM(msgs, P.gps_location_service)
  P.update(None, False, sm, toggles)
  pm = SimpleNamespace(last=None)
  pm.send = lambda s, m: setattr(pm, 'last', m)
  P.publish(plan.themeUpdated, sm, pm, toggles)
  return pm.last


def frogpilot_pass(by, t_lo, t_hi, rs_msgs):
  """FrogPilotPlanner.update + publish (fp_planner / fp_step) on every logged frogpilotPlan tick in [t_lo - FP_WARM, t_hi], with the
  radarState replaced by the re-run one (rs_msgs). Returns arrays: ns (logged frogpilotPlan logMonoTime), fp_<field> (replayed),
  log_fp_<field> (logged), FP_F + leadDeparting."""
  from cereal import messaging
  P = fp_planner()
  empty = {s: messaging.new_message(s).as_reader() for s in FP_SERV}
  rs_ns = by['radarState'][0]
  rows = []
  for F in by['frogpilotPlan'][1]:
    if not t_lo - int(FP_WARM * 1e9) <= F.logMonoTime <= t_hi or not F.frogpilotPlan.frogpilotToggles:
      continue
    md = latest(by, 'modelV2', F.logMonoTime)
    if md is None:
      continue
    lim = md.logMonoTime + LAG_NS
    msgs = {s: latest(by, s, lim) for s in FP_SERV}
    msgs = {s: empty[s] if e is None else e for s, e in msgs.items()}
    msgs['modelV2'] = md
    i = bisect.bisect_right(rs_ns, lim) - 1
    if i < 0 or rs_ns[i] not in rs_msgs:
      continue
    msgs['radarState'] = rs_msgs[rs_ns[i]]
    rows.append([F.logMonoTime, *_fp_vals(fp_step(P, msgs, F.frogpilotPlan).frogpilotPlan), *_fp_vals(F.frogpilotPlan)])
  names = [*FP_F, 'leadDeparting']
  cols = ['ns'] + [f'fp_{f}' for f in names] + [f'log_fp_{f}' for f in names]
  arr = np.array(rows, dtype=np.float64).reshape(-1, len(cols))
  out = {c: arr[:, i] for i, c in enumerate(cols)}
  out['ns'] = arr[:, 0].astype(np.int64)
  return out


def fp_fidelity(F, lo_ns, hi_ns):
  """Replayed vs logged frogpilotPlan on the scored window: per lead-consumer field the ticks that differ (discrete exact, floats >
  FLOAT_TOL) and the worst float error."""
  ns = np.asarray(F['ns'])
  sc = (ns >= lo_ns) & (ns <= hi_ns)
  per, worst = {}, {}
  for f in (*FP_F, 'leadDeparting'):
    a, b = F[f'fp_{f}'][sc], F[f'log_fp_{f}'][sc]
    d = a != b if f in (*FP_DISCRETE, 'leadDeparting') else np.abs(a - b) > FLOAT_TOL
    per[f] = int(d.sum())
    if f not in FP_DISCRETE and f != 'leadDeparting' and len(a):
      worst[f] = round(float(np.max(np.abs(a - b))), 6)
  return dict(ticks=int(sc.sum()), per_field={k: v for k, v in per.items() if v}, worst=worst)
