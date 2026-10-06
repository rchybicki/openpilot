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
replayed command, and flags-off identity covers the planner (H1a compares the plans too)."""
import bisect
import contextlib
import json
import os
import pickle
from pathlib import Path
from types import SimpleNamespace

import numpy as np

from openpilot.tools.stopping.sim import SIM_HOME

FRAMES = SIM_HOME / 'frames'
OUT_I8 = ('stopreq', 'accmode', 'phase', 'active', 'owning', 'lcs')
RD = Path.home() / '.route_sync/data/media/0/realdata'
PSERV = ('carControl', 'carState', 'controlsState', 'liveParameters', 'radarState', 'modelV2', 'selfdriveState', 'frogpilotCarState',
         'frogpilotPlan')
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


def planner_pass(route, lo, hi):
  """The process's planner on the recorded inputs of [lo - PLAN_PRE, hi + 0.5] (pp6.planner_pass, one arm). Returns arrays ns (tick
  logMonoTime), log_at / log_ss (logged plan), at / ss (replayed); empty arrays when no rlog is left for the span."""
  import zstandard
  from cereal import log
  import openpilot.selfdrive.controls.lib.longitudinal_planner as LP
  evs = []
  for p in _seg_paths(route, lo - PLAN_PRE, hi):
    evs += list(log.Event.read_multiple_bytes(zstandard.ZstdDecompressor().decompress(p.read_bytes(), max_output_size=int(1.5e9))))
  cols = ('ns', 'log_at', 'log_ss', 'at', 'ss')
  init = next((e for e in evs if e.which() == 'initData'), None)
  cpm = next((e.carParams for e in evs if e.which() == 'carParams'), None)
  if init is None or cpm is None:
    return {c: np.zeros(0) for c in cols}
  by: dict = {s: ([], []) for s in PSERV}
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
  fp = [e.frogpilotPlan.frogpilotToggles for e in by['frogpilotPlan'][1] if e.frogpilotPlan.frogpilotToggles]
  tog = dict(json.loads(TOGGLES.read_text()))
  tog.update(json.loads(fp[0]) if fp else {})
  toggles = SimpleNamespace(**tog)
  bound = 10_000_000 if route.startswith(('00002232', '00002234')) else 3_000_000
  o0 = init.logMonoTime

  def latest(s, tns):
    i = bisect.bisect_right(by[s][0], tns) - 1
    return by[s][1][i] if i >= 0 else None

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
    rows.append((lp.logMonoTime, float(lp.longitudinalPlan.aTarget), float(lp.longitudinalPlan.shouldStop), float(lpo.aTarget), float(lpo.shouldStop)))
  arr = np.array(rows, dtype=np.float64).reshape(-1, len(cols))
  out = {c: arr[:, i] for i, c in enumerate(cols)}
  out['ns'] = arr[:, 0].astype(np.int64)
  return out


def plan_inputs(d, P, ref):
  """Span data d with the arm's plan delta applied (pp6.lc_pass): per frame the plan tick whose logged aTarget the frame used
  (latest tick at or before the frame, up to 5 ticks back); target += arm aTarget - reference aTarget, shouldStop = the arm's where
  they differ. Returns (d2, matched frames, frames changed)."""
  assert np.array_equal(P['ns'], ref['ns']) and np.array_equal(P['log_at'], ref['log_at']), 'plan ticks differ from the reference arm'
  fr = d['frames']
  delta, ssd = P['at'] - ref['at'], P['ss'] != ref['ss']
  out, matched, changed = [], 0, 0
  for f in fr:
    j = int(np.searchsorted(P['ns'], f['t'] * 1e9, side='right')) - 1
    jj = j
    for back in range(5):
      if j - back >= 0 and P['log_at'][j - back] == f['target']:
        jj = j - back
        break
    ok = jj >= 0 and P['log_at'][jj] == f['target']
    matched += int(ok)
    if jj >= 0 and (delta[jj] != 0.0 or ssd[jj]):
      g = dict(f)
      g['target'] = float(f['target'] + delta[jj])
      if ssd[jj]:
        g['should_stop'] = bool(P['ss'][jj])
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


def span_job(j):
  """One span: the planner pass (saved to j['plan_dir']/<span>.npz for the arms that use this one as reference), then LongControl /
  service / sender on the recorded frames with the plan delta against j['ref_plan_dir'] (None = this is the reference arm)."""
  d = frames(j['corpus'], j['span'])
  span = next(s for s in json.loads((FRAMES / f"{j['corpus']}.json").read_text())['spans'] if s['span'] == j['span'])
  P = planner_pass(span['route'], span['lo'], span['hi'])
  _save_plan(Path(j['plan_dir']) / f"{j['span']}.npz", P)
  info = dict(plan_ticks=len(P['ns']), plan_ref=bool(j.get('ref_plan_dir')), frames_matched=None, frames_changed=0)
  if j.get('ref_plan_dir'):
    with np.load(Path(j['ref_plan_dir']) / f"{j['span']}.npz") as z:
      ref = {k: z[k] for k in z.files}
    d, info['frames_matched'], info['frames_changed'] = plan_inputs(d, P, ref)
  r = run(d)
  r['route'], r['commit'], r['origin'] = d['route'], d.get('commit'), d['origin']   # origin: frame times of the plan coverage check
  r['plan'] = P
  r['plan_info'] = info
  return r
