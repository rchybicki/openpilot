"""Exact recorded replay (the cyc_1004 lrcore / e3 rcore recipe): LongControl + StopContext + StoppingService + the Hyundai
CarController sender on the logged 100 Hz controller frames (variant-C input timing). The modules come from the process's tree
(loader); the flags are set process-wide by the worker. No code patch: the service call is wrapped only to read its kwargs.

Corpora (SIM_HOME/frames/<corpus>/<span>.pkl, span lists <corpus>.json): c1 = the launch / hold spans (cyc_1004/launch), c2 = the
stop windows (cyc_1003v/A). Per span: the command (wire), the sent SCC12 aReqValue / StopReq / ACCMode, the service phase and
activity, LongControl's state and ownership, the hold anchor, plus the logged inputs the gates read (t, v, gas, brake, active,
recorded command, lead, gap, vLead).

Planner lockstep (eb5 pp6.py): the process's planner replays every recorded longitudinalPlan tick of the span (from 20 s before it)
on the logged planner inputs (latest message <= modelMonoTime + 3 ms, 10 ms on 2232/2234; modelV2 <= modelMonoTime). Its aTarget and
shouldStop per tick are the arm's plan ('plan'). An arm with a reference (job 'ref_plan_dir' = the HEAD arm's plans) feeds
LongControl the recorded target plus (arm aTarget - reference aTarget) and the arm's shouldStop where they differ, at the tick
whose logged aTarget the recorded frame used; the reference arm (HEAD) keeps the recorded inputs. So a planner change reaches the
replayed command, and flags-off identity covers the planner (H1a compares the plans too).

Radar stage (radar_replay.py, cycle_20261006 builder R): before the planner, the arm's radard re-runs on the logged radard inputs and
the FrogPilot lead consumers re-run on its radarState. The planner reads the re-run radarState (every arm, HEAD too: same logged
logMonoTime, so its older-sample timing is the log's) and, in an arm with a reference, the logged frogpilotPlan / selfdriveState with
the lead-consumer fields moved by (arm - reference) (experimental mode: the arm's where they differ). LongControl gets the complete
planner output as deltas (aTarget, distanceToStopTarget, distanceToStopTargetModel, aTargetTrajectory; shouldStop, FCW and the
trajectory's validity where they differ) and the lead inputs of the radarState its frame read (latest radarState at or before the
frame's selfdriveState): float fields moved by (arm - reference), the arm's values where status or track id differ."""
import bisect
import contextlib
import json
import os
import pickle
from pathlib import Path
from types import SimpleNamespace

import numpy as np

from openpilot.tools.stopping.sim import SIM_HOME, radar_replay

FRAMES = SIM_HOME / 'frames'
OUT_I8 = ('stopreq', 'accmode', 'phase', 'active', 'owning', 'lcs')
RD = Path.home() / '.route_sync/data/media/0/realdata'
PSERV = ('carControl', 'carState', 'controlsState', 'liveParameters', 'radarState', 'modelV2', 'selfdriveState', 'frogpilotCarState',
         'frogpilotPlan')
LSERV = tuple(dict.fromkeys(PSERV + ('liveTracks', 'frogpilotRadarState') + radar_replay.FP_SERV))   # streams the span job reads
PLAN_F = ('fcw', 'dts', 'dtsm', 'traj', 'trajv')   # planner outputs forwarded to LongControl besides aTarget / shouldStop
TOGGLES = Path(__file__).resolve().parent / 'toggles_default.json'
PLAN_PRE = 20.0   # s of planner history before the span


def frames(corpus, span):
  return pickle.loads((FRAMES / corpus / f'{span}.pkl').read_bytes())


def _quiet():
  from openpilot.common.swaglog import cloudlog, ipchandler
  with contextlib.suppress(ValueError):
    cloudlog.removeHandler(ipchandler)
  cloudlog.disabled = True


def run(d):
  from cereal import car
  from openpilot.selfdrive.controls.lib.longcontrol import LongControl
  from openpilot.selfdrive.controls.lib.stopping_telemetry import StoppingTelemetry
  from opendbc.car.hyundai.interface import CarInterface as CI
  from opendbc.car.hyundai.tests.test_can_bounds_fork import make_controller, run_frame, get_signal
  _quiet()
  with car.CarParams.from_bytes(d['cp']) as reader:
    cp = reader.as_builder()
  lc = LongControl(cp)
  lc._service_shadow_tel = StoppingTelemetry(log_fn=lambda **kw: None)
  st = d['settings']
  toggles = SimpleNamespace(vEgoStarting=cp.vEgoStarting, vEgoStopping=cp.vEgoStopping, startAccel=cp.startAccel,
                            human_acceleration=st.get('HumanAcceleration') == '1' and st.get('LongitudinalTune') == '1',
                            force_coast_strength=float(st.get('CEForceCoastStrength') or 1.0), max_desired_acceleration=4.0)
  sender, _ = make_controller(cp)
  svc = lc._service_shadow_svc
  box = {}
  inner = svc.update

  def cap(**kw):
    r = inner(**kw)
    box['r'] = r
    return r
  svc.update = cap
  fr = d['frames']
  o = d['origin']
  lc.last_output_accel = fr[0]['recorded']
  out = {c: [] for c in ('wire', 'sent', *OUT_I8, 'hold_gap')}
  sent, stop_req, accmode = 0.0, False, 0
  for f in fr:
    cs = SimpleNamespace(**{k: (SimpleNamespace(**v) if isinstance(v, dict) else v) for k, v in f['cs'].items()})
    if not f['active']:
      lc.reset()
    prev_state = lc.long_control_state
    limits = CI.get_pid_accel_limits(cp, cs.vEgo, cs.vCruise / 3.6)
    box.clear()
    a = float(min(lc.update(f['active'], cs, f['target'], f['should_stop'], f['dts'], limits, toggles, request_time=float(f['t']),
                            **f['kw']), toggles.max_desired_acceleration))
    lc.observe_accel_request(a, float(f['t']), authorized=f['authorized'])
    _, msgs = run_frame(sender, dict(accel=a, state=prev_state, v_ego=cs.vEgo, a_ego=cs.aEgo, long_active=f['active'], gas_pressed=cs.gasPressed))
    if 0x421 in msgs:
      sent = get_signal('SCC12', 'aReqValue', msgs[0x421])
      stop_req = bool(get_signal('SCC12', 'StopReq', msgs[0x421]))
      accmode = int(get_signal('SCC12', 'ACCMode', msgs[0x421]))
    r = box.get('r')
    hg = getattr(getattr(svc, 'ev', None), 'hold_entry_gap', None)
    out['wire'].append(a)
    out['sent'].append(float(sent))
    out['stopreq'].append(int(stop_req))
    out['accmode'].append(accmode)
    out['phase'].append(int(svc.phase))
    out['active'].append(int(bool(r.active)) if r is not None else -1)
    out['owning'].append(int(bool(getattr(lc, '_service_live_owning', False))))
    out['lcs'].append(int(lc.long_control_state))
    out['hold_gap'].append(float(hg) if hg is not None else np.nan)
  R = {k: np.asarray(v, dtype=np.int8 if k in OUT_I8 else np.float64) for k, v in out.items()}
  g = lambda fn, dt=np.float32: np.asarray([fn(x) for x in fr], dtype=dt)  # noqa: E731
  R.update(t=g(lambda x: x['t'] - o, np.float64), v=g(lambda x: x['cs']['vEgo']), gas=g(lambda x: x['cs']['gasPressed'], np.int8),
           brake=g(lambda x: x['cs']['brakePressed'], np.int8), act=g(lambda x: x['active'], np.int8), rec=g(lambda x: x['recorded'], np.float64),
           lead=g(lambda x: bool(x['kw']['lead_status']), np.int8), gap=g(lambda x: x['kw']['lead_d_rel']), vl=g(lambda x: x['kw']['lead_v']))
  return R


def _seg_paths(route, lo, hi):
  """rlog paths whose events can fall in [lo, hi] (route-relative; harness._seg_paths)."""
  out = []
  for s in range(max(int((lo - 12) // 60) - 1, 0), int((hi + 2) // 60) + 2):
    p = RD / f'{route}--{s}/rlog.zst'
    if p.is_file():
      out.append(p)
  return out


class _SM:
  def __init__(self, msgs, prev):
    self.msgs = msgs
    self.logMonoTime = {s: msgs[s].logMonoTime for s in msgs}
    self.updated = {s: prev is None or prev.get(s) != msgs[s].logMonoTime for s in msgs}
    self.valid = {s: msgs[s].valid for s in msgs}
    self.alive = dict.fromkeys(msgs, True)

  def __getitem__(self, s):
    return getattr(self.msgs[s], s)

  def all_checks(self, service_list=None):
    return all(self.valid[s] for s in (service_list or self.msgs))


class _PM:
  def __init__(self):
    self.last = None

  def send(self, s, msg):
    self.last = msg


def span_events(route, lo, hi):
  """The rlog events of the span's segments (from RADAR_WARM + PLAN_PRE before lo): (initData, carParams, {stream: (sorted
  logMonoTimes, events)} of LSERV, longitudinalPlans sorted); None when no rlog / initData / carParams is left for the span."""
  import zstandard
  from cereal import log
  evs = []
  for p in _seg_paths(route, lo - PLAN_PRE - radar_replay.RADAR_WARM, hi):
    evs += list(log.Event.read_multiple_bytes(zstandard.ZstdDecompressor().decompress(p.read_bytes(), max_output_size=int(1.5e9))))
  init = next((e for e in evs if e.which() == 'initData'), None)
  cpm = next((e.carParams for e in evs if e.which() == 'carParams'), None)
  if init is None or cpm is None:
    return None
  by: dict = {s: ([], []) for s in LSERV}
  lps = []
  for e in evs:
    w = e.which()
    if w in by:
      by[w][0].append(e.logMonoTime)
      by[w][1].append(e)
    elif w == 'longitudinalPlan':
      lps.append(e)
  del evs
  for s in by:
    o = np.argsort(by[s][0], kind='stable')
    by[s] = ([by[s][0][i] for i in o], [by[s][1][i] for i in o])
  lps.sort(key=lambda e: e.logMonoTime)
  return init, cpm, by, lps


def planner_pass(E, route, lo, hi, subst=None):
  """The process's planner on the recorded inputs of [lo - PLAN_PRE, hi + 0.5] (pp6.planner_pass, one arm; E = span_events). subst:
  {stream: {logged logMonoTime: event}} read instead of the logged event (the re-run radarState, the arm's frogpilotPlan /
  selfdriveState). Returns arrays ns (tick logMonoTime), log_at / log_ss (logged plan), at / ss and PLAN_F (replayed: fcw,
  distanceToStopTarget, distanceToStopTargetModel, aTargetTrajectory and its validity); empty arrays without events."""
  import openpilot.selfdrive.controls.lib.longitudinal_planner as LP
  cols = ('ns', 'log_at', 'log_ss', 'at', 'ss') + PLAN_F
  if E is None:
    return {c: np.zeros(0) for c in cols}
  init, cpm, by, lps = E
  subst = subst or {}
  fp = [e.frogpilotPlan.frogpilotToggles for e in by['frogpilotPlan'][1] if e.frogpilotPlan.frogpilotToggles]
  tog = dict(json.loads(TOGGLES.read_text()))
  tog.update(json.loads(fp[0]) if fp else {})
  toggles = SimpleNamespace(**tog)
  bound = 10_000_000 if route.startswith(('00002232', '00002234')) else 3_000_000
  o0 = init.logMonoTime

  def latest(s, tns):
    i = bisect.bisect_right(by[s][0], tns) - 1
    if i < 0:
      return None
    return subst.get(s, {}).get(by[s][0][i], by[s][1][i])

  planner = LP.LongitudinalPlanner(cpm)
  rows, prev = [], None
  t_lo, t_hi = o0 + int((lo - PLAN_PRE) * 1e9), o0 + int((hi + 0.5) * 1e9)
  for lp in lps:
    if not t_lo <= lp.logMonoTime <= t_hi:
      continue
    md = lp.longitudinalPlan.modelMonoTime
    msgs = {s: latest(s, md + bound) for s in PSERV}
    msgs['modelV2'] = latest('modelV2', md)
    if any(x is None for x in msgs.values()):
      continue
    sm = _SM(msgs, prev)
    prev = {s: msgs[s].logMonoTime for s in msgs}
    planner.update(sm, toggles)
    pm = _PM()
    planner.publish(sm, pm, toggles)
    lpo = pm.last.longitudinalPlan
    rows.append((lp.logMonoTime, float(lp.longitudinalPlan.aTarget), float(lp.longitudinalPlan.shouldStop), float(lpo.aTarget), float(lpo.shouldStop),
                 float(lpo.fcw), float(lpo.distanceToStopTarget), float(lpo.distanceToStopTargetModel), float(lpo.aTargetTrajectory),
                 float(lpo.aTargetTrajectoryValid)))
  arr = np.array(rows, dtype=np.float64).reshape(-1, len(cols))
  out = {c: arr[:, i] for i, c in enumerate(cols)}
  out['ns'] = arr[:, 0].astype(np.int64)
  return out


KW_LEAD = {'l1': (('lead_status', 'status'), ('lead_v', 'vLead'), ('lead_d_rel', 'dRel'), ('lead_a', 'aLeadK'), ('lead_track_id', 'radarTrackId'),
                  ('lead_model_prob', 'modelProb')),
           'l2': (('lead2_status', 'status'), ('lead2_v', 'vLead'), ('lead2_d_rel', 'dRel'))}   # LongControl kwargs <- radarState lead


def _lead_kw(kw, A, B, i):
  """LongControl lead kwargs of a frame that read re-run radarState tick i: arm A vs reference B (R arrays). Unchanged where the two
  agree (or neither has the lead); the arm's values where status or track id differ; else the logged floats + (A - B)."""
  out = {}
  for w, pairs in KW_LEAD.items():
    a = {f: float(A[f'rs_{w}_{f}'][i]) for f in LEAD_KEYS}
    b = {f: float(B[f'rs_{w}_{f}'][i]) for f in LEAD_KEYS}
    if a == b or not (a['status'] or b['status']):
      continue
    whole = a['status'] != b['status'] or a['radarTrackId'] != b['radarTrackId']
    for k, f in pairs:
      if f == 'status':
        out[k] = bool(a[f])
      elif f == 'radarTrackId':
        out[k] = int(a[f])
      else:
        out[k] = a[f] if whole else float(kw[k] + (a[f] - b[f]))
  return out


LEAD_KEYS = ('status', 'radarTrackId', 'vLead', 'dRel', 'aLeadK', 'modelProb')


def plan_inputs(d, P, ref, lead=None):
  """Span data d with the arm's plan delta applied (pp6.lc_pass): per frame the plan tick whose logged aTarget the frame used
  (latest tick at or before the frame, up to 5 ticks back); target / distanceToStopTarget / distanceToStopTargetModel /
  aTargetTrajectory += arm - reference, shouldStop / FCW / trajectory validity = the arm's where they differ. lead (radar stage):
  dict(R, F, rs_i, fp_i) = the arm's re-run radarState and FrogPilot arrays and per frame the radarState tick and frogpilotPlan
  tick its selfdriveState read (-1: none); ref holds the reference arm's as R_* / F_*: the lead kwargs (_lead_kw) and experimental
  mode follow the arm. Returns (d2, matched frames, frames changed)."""
  assert np.array_equal(P['ns'], ref['ns']) and np.array_equal(P['log_at'], ref['log_at']), 'plan ticks differ from the reference arm'
  fr = d['frames']
  delta, ssd = P['at'] - ref['at'], P['ss'] != ref['ss']
  dd = {k: P[k] - ref[k] for k in ('dts', 'dtsm', 'traj') if k in P and k in ref}
  dfcw = P['fcw'] != ref['fcw'] if 'fcw' in P and 'fcw' in ref else np.zeros(len(delta), bool)
  dtv = P['trajv'] != ref['trajv'] if 'trajv' in P and 'trajv' in ref else np.zeros(len(delta), bool)
  plan_changed = (delta != 0.0) | ssd | dfcw | dtv
  for v in dd.values():
    plan_changed |= v != 0.0
  if lead is not None:
    RB = {k[2:]: v for k, v in ref.items() if k.startswith('R_')}
    FB = {k[2:]: v for k, v in ref.items() if k.startswith('F_')}
    assert np.array_equal(lead['R']['ns'], RB['ns']) and np.array_equal(lead['F']['ns'], FB['ns']), 'radar ticks differ from the reference arm'
    em_diff = lead['F']['fp_experimentalMode'] != FB['fp_experimentalMode']
  out, matched, changed = [], 0, 0
  for n, f in enumerate(fr):
    j = int(np.searchsorted(P['ns'], f['t'] * 1e9, side='right')) - 1
    jj = j
    for back in range(5):
      if j - back >= 0 and P['log_at'][j - back] == f['target']:
        jj = j - back
        break
    ok = jj >= 0 and P['log_at'][jj] == f['target']
    matched += int(ok)
    kw = {}
    if lead is not None:
      i, k = int(lead['rs_i'][n]), int(lead['fp_i'][n])
      if i >= 0:
        kw.update(_lead_kw(f['kw'], lead['R'], RB, i))
      if k >= 0 and em_diff[k]:
        kw['experimental_mode'] = bool(lead['F']['fp_experimentalMode'][k])
    if jj >= 0 and plan_changed[jj]:
      if dfcw[jj]:
        kw['fcw'] = bool(P['fcw'][jj])
      if 'dtsm' in dd and dd['dtsm'][jj] != 0.0:
        kw['model_stop_d'] = float(f['kw']['model_stop_d'] + dd['dtsm'][jj])
      if 'traj' in dd and (dd['traj'][jj] != 0.0 or dtv[jj]):
        rec = f['kw']['a_target_trajectory']
        kw['a_target_trajectory'] = (None if not P['trajv'][jj] else float(P['traj'][jj]) if dtv[jj] or rec is None
                                     else float(rec + dd['traj'][jj]))
    if kw or (jj >= 0 and plan_changed[jj]):
      g = dict(f, kw=dict(f['kw'], **kw))
      if jj >= 0 and plan_changed[jj]:
        g['target'] = float(f['target'] + delta[jj])
        if ssd[jj]:
          g['should_stop'] = bool(P['ss'][jj])
        if 'dts' in dd:
          g['dts'] = float(f['dts'] + dd['dts'][jj])
      out.append(g)
      changed += 1
    else:
      out.append(f)
  return dict(d, frames=out), matched, changed


def _save_plan(path, P):
  path.parent.mkdir(parents=True, exist_ok=True)
  tmp = path.with_name(f'{path.stem}.{os.getpid()}.tmp.npz')
  np.savez(tmp, **P)
  os.replace(tmp, path)


def frame_ticks(d, by, R, F):
  """Per frame the re-run radarState tick (index into R) and the replayed frogpilotPlan tick (index into F) that its selfdriveState
  read: the frame's selfdriveState = the latest at or before its carControl (variant C), its radarState = the latest at or before
  that selfdriveState, its experimental mode = the latest frogpilotPlan at or before it (selfdrived); -1 where absent. Also the
  frames whose logged lead kwargs equal that logged radarState's leadOne (the mapping check)."""
  sds, rsn, fpn = by['selfdriveState'][0], np.asarray(R['ns']), np.asarray(F['ns'])
  rs_i, fp_i, ok = [], [], 0
  for f in d['frames']:
    k = bisect.bisect_right(sds, int(round(f['t'] * 1e9))) - 1
    s_ns = sds[k] if k >= 0 else -1
    i = int(np.searchsorted(rsn, s_ns, side='right')) - 1 if k >= 0 else -1
    q = int(np.searchsorted(fpn, s_ns, side='right')) - 1 if k >= 0 else -1
    rs_i.append(i)
    fp_i.append(q)
    if i >= 0:
      ok += (bool(R['log_l1_status'][i]) == bool(f['kw']['lead_status']) and np.float32(R['log_l1_vLead'][i]) == np.float32(f['kw']['lead_v'])
             and np.float32(R['log_l1_dRel'][i]) == np.float32(f['kw']['lead_d_rel']))
  return np.array(rs_i), np.array(fp_i), ok


def span_job(j):
  """One span: the radar stage (radard + FrogPilot lead consumers re-run), the planner pass on its outputs (plan + radar arrays
  saved to j['plan_dir']/<span>.npz for the arms that use this one as reference), then LongControl / service / sender on the
  recorded frames with the plan and lead deltas against j['ref_plan_dir'] (None = this is the reference arm)."""
  d = frames(j['corpus'], j['span'])
  span = next(s for s in json.loads((FRAMES / f"{j['corpus']}.json").read_text())['spans'] if s['span'] == j['span'])
  E = span_events(span['route'], span['lo'], span['hi'])
  ref = None
  if j.get('ref_plan_dir'):
    with np.load(Path(j['ref_plan_dir']) / f"{j['span']}.npz") as z:
      ref = {k: z[k] for k in z.files}
  radar, lead = None, None
  if E is not None:
    init, cpm, by, _ = E
    o0 = init.logMonoTime
    t_lo, t_hi = o0 + int((span['lo'] - PLAN_PRE) * 1e9), o0 + int((span['hi'] + 0.5) * 1e9)
    s_lo, s_hi = int(round(d['frames'][0]['t'] * 1e9)), int(round(d['frames'][-1]['t'] * 1e9))
    rs_msgs, R = radar_replay.radard_pass(by, t_lo, t_hi, cpm.carFingerprint)
    F = radar_replay.frogpilot_pass(by, t_lo, t_hi, rs_msgs)
    subst = {'radarState': rs_msgs}
    if ref is not None:
      subst.update(radar_replay.arm_subst(by, F, {k[2:]: v for k, v in ref.items() if k.startswith('F_')}))
    P = planner_pass(E, span['route'], span['lo'], span['hi'], subst)
    rs_i, fp_i, kw_ok = frame_ticks(d, by, R, F)
    lead = dict(R=R, F=F, rs_i=rs_i, fp_i=fp_i)
    radar = dict(fid=radar_replay.fidelity(R, t_lo, s_lo, s_hi), fp_fid=radar_replay.fp_fidelity(F, s_lo, s_hi), kw_matched=kw_ok,
                 m1=radar_replay.m1_rows(R, s_lo, s_hi, o0), subst={k: len(v) for k, v in subst.items()},
                 fp={k: v[(F['ns'] >= s_lo) & (F['ns'] <= s_hi)].astype(np.int64 if k == 'ns' else np.float32) for k, v in F.items()
                     if k == 'ns' or k.startswith(('fp_', 'log_fp_'))})   # R4: the arm's lead-consumer outputs (+ the logged) on the scored window
    del rs_msgs, subst
  else:
    P = planner_pass(None, span['route'], span['lo'], span['hi'])
  _save_plan(Path(j['plan_dir']) / f"{j['span']}.npz", dict(P, **({f'R_{k}': v for k, v in lead['R'].items() if k != 'm1'} if lead else {}),
                                                           **({f'F_{k}': v for k, v in lead['F'].items()} if lead else {})))
  info = dict(plan_ticks=len(P['ns']), plan_ref=ref is not None, frames_matched=None, frames_changed=0)
  if ref is not None:
    d, info['frames_matched'], info['frames_changed'] = plan_inputs(d, P, ref, lead if lead is not None and 'R_ns' in ref else None)
  r = run(d)
  r['route'], r['commit'], r['origin'] = d['route'], d.get('commit'), d['origin']   # origin: frame times of the plan coverage check
  r['plan'] = P
  r['plan_info'] = info
  r['radar'] = radar
  return r
