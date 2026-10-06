"""Exact recorded replay for the per-drive report (drive_report.py).

LongControl + StopContext + StoppingService + the Hyundai CarController sender run on the logged inputs of one span
(variant-C timing: the latest message of each service at or before the selfdriveState that the carControl frame used).
This is the recipe of the cycle-1003/1004 replays (replay_core.extract, e3/replay2/rcore.run), moved into the repo.

Source pinning: the controller modules come from `git show <commit>` (never from the working tree), so flags or edits in
the working tree cannot leak into a replay. Every .py under the mapped directories that differs between the working tree
and <commit> is served from a snapshot directory by a meta-path finder; a flag override rewrites one `NAME = value` line
of stopping_flags.py in the snapshot, so derived flags (GOVERNOR_BAND_PROFILE = SANTA_FE_STOP_LINE) follow at import, and
check() asserts every flag against the snapshot file evaluated as Python. One arm (commit + overrides) per process: call
load() once, before any openpilot controls import. Compiled extensions and every unmapped module still come from the
working tree build. (tools/stopping/sim/loader.py does the same job for the sim runner but still sets flags with setattr,
tooling_check fix 1; merge the two loaders once that is fixed.)

Planner lockstep (planner_run, the eb5 pp6.py planner arm): the arm's LongitudinalPlanner on the logged planner inputs of a
span (+ PLAN_WARMUP_S), one tick per logged longitudinalPlan; it records the published aTarget / shouldStop and the stop-line
state (floor, armed, the command the line saw, the stop-commit certificate / persistence / provenance it was given). run()
takes the plan of another arm as a delta on the logged target (logged + arm - reference, pp6 lc_pass), so frames where the
arms agree stay bit-identical to the reference replay.
"""
import bisect
import contextlib
import hashlib
import importlib
import importlib.abc
import importlib.util
import os
import pickle
import re
import subprocess
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np

REPO = Path(__file__).resolve().parents[2]
RD = Path.home() / '.route_sync/data/media/0/realdata'
WORK = Path(os.environ.get('DRIVE_REPORT_HOME', Path.home() / '.route_sync/work/drive_report'))
# repo directory -> module prefix: every .py there that differs from the working tree is pinned to the commit
MAPPED = {'selfdrive/controls/lib': 'openpilot.selfdrive.controls.lib', 'opendbc_repo/opendbc/car/hyundai': 'opendbc.car.hyundai',
          'selfdrive/controls': 'openpilot.selfdrive.controls'}
FLAGS_FILE = 'selfdrive/controls/lib/stopping_flags.py'
SERV = ('carState', 'radarState', 'longitudinalPlan', 'frogpilotCarState', 'frogpilotPlan', 'selfdriveState', 'modelV2')
SETTINGS = ('HumanAcceleration', 'LongitudinalTune', 'CEForceCoastStrength', 'IncreasedStoppedDistance', 'ExperimentalMode')
COLS = ('wire', 'stopreq', 'accmode', 'phase', 'active', 'owning', 'lcs', 'rehold', 'strict', 'd_gap', 'band', 'target')
CACHE_V = 2   # replay cache layout (COLS); bump when the columns change
ARM: dict | None = None
# the replay / extraction code, the toggle defaults and the sender test helper: never pinned to the commit (tools and tests load from
# the working tree), so they are part of every cache identity (impl_key)
IMPL_FILES = [Path(__file__).resolve(), REPO / 'tools/stopping/review/triage_one.py', REPO / 'tools/stopping/sim/toggles_default.json',
              REPO / 'opendbc_repo/opendbc/car/hyundai/tests/test_can_bounds_fork.py']
UNPINNED_ROOTS = ('selfdrive', 'opendbc_repo', 'frogpilot', 'common', 'cereal', 'system')


def git(*args):
  return subprocess.run(['git', '-C', str(REPO), *args], capture_output=True, check=True).stdout


def full_sha(commit):
  """The full sha of a commit in the local repo, or None when it is not there."""
  r = subprocess.run(['git', '-C', str(REPO), 'rev-parse', '--verify', '--quiet', f'{commit}^{{commit}}'], capture_output=True)
  return r.stdout.decode().strip() or None


def impl_key(commit, extra=''):
  """Cache identity of a replay of commit beyond (commit, overrides): IMPL_FILES, `extra` (the caller's own replay code) and the
  working-tree content of every python file under UNPINNED_ROOTS that differs from commit outside MAPPED (it loads from the working
  tree, not from the commit). A change to any of them starts a new cache."""
  h = hashlib.sha1(extra.encode())
  unpinned = [p for p in git('diff', '--name-only', commit, '--', *UNPINNED_ROOTS).decode().split()
              if p.endswith('.py') and p.rsplit('/', 1)[0] not in MAPPED]
  for f in IMPL_FILES + [REPO / p for p in sorted(unpinned)]:
    h.update(str(f).encode() + b'\0' + (f.read_bytes() if f.is_file() else b'<absent>'))
  return h.hexdigest()[:10]


def flag_value(commit, name):
  """The literal value of a stopping flag at a commit: True / False / None (the flag does not exist there)."""
  try:
    text = git('show', f'{commit}:{FLAGS_FILE}').decode()
  except subprocess.CalledProcessError:
    return None
  m = re.search(rf'^{name}\s*=\s*(True|False)\s*(#.*)?$', text, re.M)
  return None if m is None else m.group(1) == 'True'


def arm_key(commit, overrides):
  return commit[:10] + ''.join(f'__{k}-{int(v)}' for k, v in sorted(overrides.items()))


def snapshot(commit, overrides):
  """Write the pinned files of one arm; returns ({module: file}, {module: sha1}, outside) where outside lists the differing
  python files outside MAPPED (they still load from the working tree; the report names them)."""
  changed = sorted(set(git('diff', '--name-only', commit, '--', 'selfdrive', 'opendbc_repo/opendbc/car', 'frogpilot').decode().split())
                   | {FLAGS_FILE})   # the flags are always pinned (overrides, and the asserted file content)
  src = WORK / 'src' / arm_key(commit, overrides)
  files, shas, outside = {}, {}, []
  for rel in changed:
    if not rel.endswith('.py') or '/tests/' in rel:
      continue
    d, name = rel.rsplit('/', 1)
    if d not in MAPPED:
      outside.append(rel)
      continue
    try:
      body = git('show', f'{commit}:{rel}').decode()
    except subprocess.CalledProcessError:
      continue   # added after the commit: the commit's modules cannot import it
    if rel == FLAGS_FILE:
      for k, v in overrides.items():
        body, n = re.subn(rf'^{k}\s*=\s*(True|False)', f'{k} = {v}', body, flags=re.M)
        assert n == 1, f'{k}: {n} definitions in {commit}:{rel}'
    p = src / rel
    p.parent.mkdir(parents=True, exist_ok=True)
    if not p.exists() or p.read_text() != body:
      tmp = p.with_name(p.name + f'.tmp{os.getpid()}')
      tmp.write_text(body)
      os.replace(tmp, p)
    mod = f'{MAPPED[d]}.{name[:-3]}'
    files[mod], shas[mod] = str(p), hashlib.sha1(body.encode()).hexdigest()
  return files, shas, outside


class _Finder(importlib.abc.MetaPathFinder):
  def __init__(self, files):
    self.files = files

  def find_spec(self, fullname, path=None, target=None):
    f = self.files.get(fullname)
    return importlib.util.spec_from_file_location(fullname, f) if f else None


def load(commit, overrides=None):
  """Pin this process to one arm. Overrides force stopping flags, e.g. {'RELEASE_END_STOPPED_LEAD_REHOLD': False}."""
  global ARM
  overrides = dict(overrides or {})
  if ARM is not None:
    assert ARM['key'] == arm_key(commit, overrides), (ARM['key'], commit, overrides)
    return ARM
  if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))
  files, shas, outside = snapshot(commit, overrides)
  early = [m for m in files if m in sys.modules]
  assert not early, f'imported before the pin: {early}'
  sys.meta_path.insert(0, _Finder(files))
  ARM = dict(key=arm_key(commit, overrides), commit=commit, overrides=overrides, files=files, shas=shas, outside=outside)
  check()
  from openpilot.common.swaglog import cloudlog, ipchandler
  with contextlib.suppress(ValueError):
    cloudlog.removeHandler(ipchandler)
  cloudlog.disabled = True
  return ARM


def check():
  """Every pinned module that is imported is the snapshot file with the expected content; flags are as requested, and every
  module-level flag (derived ones included) equals the snapshot flags file evaluated as Python."""
  for mod in ('openpilot.selfdrive.controls.lib.stopping_flags', 'openpilot.selfdrive.controls.lib.longcontrol'):
    importlib.import_module(mod)
  for mod, f in ARM['files'].items():
    m = sys.modules.get(mod)
    if m is None:
      continue
    assert os.path.realpath(m.__file__) == os.path.realpath(f), (mod, m.__file__)
    assert hashlib.sha1(Path(f).read_bytes()).hexdigest() == ARM['shas'][mod], f'snapshot changed: {f}'
  sf = sys.modules['openpilot.selfdrive.controls.lib.stopping_flags']
  for k, v in ARM['overrides'].items():
    assert getattr(sf, k) is v, (k, getattr(sf, k, None))
  ns: dict = {}
  exec(compile(Path(sf.__file__).read_text(), FLAGS_FILE, 'exec'), ns)  # the pinned constants-only flags file
  for k, v in ns.items():
    if k.isupper():
      assert getattr(sf, k) == v, f'flag {k}: module {getattr(sf, k, None)!r} != file {v!r}'


# ---- frame extraction (variant C, streaming per segment) -------------------------------------------------------------------
def seg_paths(route, lo, hi):
  """rlog paths whose events can fall in [lo, hi] (route-relative s). Segment s starts near 60 s."""
  out = []
  for s in range(max(int((lo - 12) // 60) - 1, 0), int((hi + 2) // 60) + 2):
    p = RD / f'{route}--{s}/rlog.zst'
    if p.is_file():
      out.append(p)
  return out


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
  kw = dict(experimental_mode=snap['selfdriveState']['experimental'], **rs, fcw=lp['fcw'], model_stop_d=lp['dts_model'],
            model_should_stop=snap['modelV2']['model_should_stop'], force_coast=snap['frogpilotCarState']['force_coast'],
            increased_stopped_distance=snap['frogpilotPlan']['isd'], a_target_trajectory=lp['traj'],
            freeze_integrator=cs['gasPressed'], plan_valid=lp['valid'])
  return dict(t=cc_ev.logMonoTime * 1e-9, cs=cs, kw=kw, target=lp['aTarget'], should_stop=lp['shouldStop'], dts=lp['dts'],
              active=cc.longActive, recorded=float(cc.actuators.accel),
              authorized=bool(cc_ev.valid and cc.enabled and cc.longActive and not cc.cruiseControl.override
                              and not cs['gasPressed'] and not cs['brakePressed'] and cs['canValid'] and not cs['canTimeout']))


def extract(route, lo_rel, hi_rel):
  """Replay inputs for [lo_rel, hi_rel] (route-relative s; the origin is the route's initData)."""
  from openpilot.tools.stopping.review.triage_one import read_events
  paths = seg_paths(route, lo_rel, hi_rel)
  if not paths:
    raise FileNotFoundError(f'no rlog {route} {lo_rel}-{hi_rel}')
  mono = {s: ([], []) for s in SERV}
  ccs = []
  origin = cp = settings = commit = None
  for p in paths:
    for e in read_events(str(p)):
      w = e.which()
      if w in mono:
        mono[w][0].append(e.logMonoTime)
        mono[w][1].append(_small(w, e))
      elif w == 'carControl':
        c = e.carControl
        ccs.append(SimpleNamespace(logMonoTime=e.logMonoTime, valid=e.valid, carControl=SimpleNamespace(
          longActive=c.longActive, enabled=c.enabled, actuators=SimpleNamespace(accel=c.actuators.accel),
          cruiseControl=SimpleNamespace(override=c.cruiseControl.override))))
      elif w == 'initData' and origin is None:
        origin = e.logMonoTime * 1e-9
        commit = e.initData.gitCommit
        settings = {kv.key: bytes(kv.value).decode(errors='replace') for kv in e.initData.params.entries if kv.key in SETTINGS}
      elif w == 'carParams' and cp is None:
        cp = e.carParams.as_builder().to_bytes()
  for s in SERV:
    order = sorted(range(len(mono[s][0])), key=mono[s][0].__getitem__)
    mono[s] = ([mono[s][0][i] for i in order], [mono[s][1][i] for i in order])
  lo, hi = origin + lo_rel, origin + hi_rel
  frames = []
  for ev in ccs:
    if not lo <= ev.logMonoTime * 1e-9 <= hi:
      continue
    i = bisect.bisect_right(mono['selfdriveState'][0], ev.logMonoTime) - 1
    if i < 0:
      continue
    sds_ns = mono['selfdriveState'][0][i]
    snap = {}
    for s in SERV:
      j = bisect.bisect_right(mono[s][0], sds_ns) - 1
      if j < 0:
        break
      snap[s] = mono[s][1][j]
    else:
      frames.append(_frame(snap, ev))
  ft = [f['t'] for f in frames]
  return dict(route=route, lo_rel=lo_rel, hi_rel=hi_rel, origin=origin, commit=commit, settings=settings, cp=cp, frames=frames,
              max_gap=max((b - a for a, b in zip(ft, ft[1:], strict=False)), default=None))


def frames_for(span):
  """Cached replay inputs of a span dict (route, lo, hi, span), keyed by the extraction code (this file + triage_one.py)."""
  key = hashlib.sha1(b''.join(f.read_bytes() for f in IMPL_FILES[:2])).hexdigest()[:10]
  p = WORK / f'frames_{key}' / f"{span['span']}.pkl"
  p.parent.mkdir(parents=True, exist_ok=True)
  if p.exists():
    return pickle.loads(p.read_bytes())
  d = extract(span['route'], span['lo'], span['hi'])
  tmp = p.with_name(p.name + f'.tmp{os.getpid()}')
  tmp.write_bytes(pickle.dumps(d, protocol=pickle.HIGHEST_PROTOCOL))
  os.replace(tmp, p)
  return d


# ---- replay (rcore.run without the frozen-ego option) --------------------------------------------------------------------
def plan_targets(frames, P, ref):
  """Per-frame (target, should_stop) of an arm whose planner replay is P, as a delta on the logged plan against the reference
  planner replay ref (pp6 lc_pass): the frame's logged plan tick is the latest tick at or before the frame whose logged aTarget
  equals the frame's target (searched back 5 ticks). Frames without a matched tick keep the logged plan."""
  ns = P['ns']
  delta = P['at'] - ref['at']
  ssd = P['ss'] != ref['ss']
  tg, ss, matched = [], [], 0
  for f in frames:
    j = int(np.searchsorted(ns, f['t'] * 1e9, side='right')) - 1
    jj = next((j - b for b in range(5) if j - b >= 0 and P['log_at'][j - b] == f['target']), -1)
    matched += jj >= 0
    tg.append(float(f['target'] + delta[jj]) if jj >= 0 and delta[jj] != 0.0 else f['target'])
    ss.append(bool(P['ss'][jj]) if jj >= 0 and ssd[jj] else f['should_stop'])
  return tg, ss, matched


def run(d, plan=None):
  """Replay one span on the loaded arm. Returns numpy columns COLS + t (route-relative) + rec (logged command).
  plan = (targets, should_stops) per frame (plan_targets) replaces the logged plan."""
  from cereal import car
  from openpilot.selfdrive.controls.lib.longcontrol import LongControl
  from openpilot.selfdrive.controls.lib.stopping_telemetry import StoppingTelemetry
  from opendbc.car.hyundai.interface import CarInterface as CI
  from opendbc.car.hyundai.tests.test_can_bounds_fork import make_controller, run_frame, get_signal
  check()
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
    box['kw'] = kw
    box['r'] = inner(**kw)
    return box['r']
  svc.update = cap
  fr = d['frames']
  lc.last_output_accel = fr[0]['recorded'] if fr else 0.0
  out: dict = {c: [] for c in COLS}
  stop_req, accmode = False, 0
  for i, f in enumerate(fr):
    cs = SimpleNamespace(**{k: (SimpleNamespace(**v) if isinstance(v, dict) else v) for k, v in f['cs'].items()})
    if not f['active']:
      lc.reset()
    prev_state = lc.long_control_state
    limits = CI.get_pid_accel_limits(cp, cs.vEgo, cs.vCruise / 3.6)
    box.clear()
    target, should_stop = (f['target'], f['should_stop']) if plan is None else (plan[0][i], plan[1][i])
    a = float(min(lc.update(f['active'], cs, target, should_stop, f['dts'], limits, toggles, request_time=float(f['t']),
                            **f['kw']), toggles.max_desired_acceleration))
    lc.observe_accel_request(a, float(f['t']), authorized=f['authorized'])
    _, msgs = run_frame(sender, dict(accel=a, state=prev_state, v_ego=cs.vEgo, a_ego=cs.aEgo, long_active=f['active'],
                                     gas_pressed=cs.gasPressed))
    if 0x421 in msgs:
      stop_req = bool(get_signal('SCC12', 'StopReq', msgs[0x421]))
      accmode = int(get_signal('SCC12', 'ACCMode', msgs[0x421]))
    sig = box['kw']['signals'] if 'kw' in box else None
    r = box.get('r')
    out['wire'].append(a)
    out['stopreq'].append(int(stop_req))
    out['accmode'].append(accmode)
    out['phase'].append(int(svc.phase))
    out['active'].append(int(bool(r.active)) if r is not None else -1)
    out['owning'].append(int(bool(getattr(lc, '_service_live_owning', False))))
    out['lcs'].append(int(lc.long_control_state))
    out['rehold'].append(int(getattr(svc.ev, 'rehold_gap', None) is not None))
    out['strict'].append(int(bool(sig.lead_confirmed_stopped)) if sig is not None else -1)
    out['d_gap'].append(float(sig.d_gap) if (sig is not None and sig.d_gap is not None) else np.nan)
    out['band'].append(int(bool(getattr(svc, '_band', False))))   # GOVERNOR_BAND_PROFILE latch (not published)
    out['target'].append(float(target))
  R = {k: np.asarray(v) for k, v in out.items()}
  R['t'] = np.array([f['t'] - d['origin'] for f in fr])
  R['rec'] = np.array([f['recorded'] for f in fr])
  return R


# ---- planner lockstep (eb5 pp6.planner_pass, one arm per process) --------------------------------------------------------
PSERV = ('carControl', 'carState', 'controlsState', 'liveParameters', 'radarState', 'modelV2', 'selfdriveState', 'frogpilotCarState',
         'frogpilotPlan')
PLAN_WARMUP_S = 20.0
PLAN_BOUND_MS = (3, 10)   # planner inputs: the latest message <= modelMonoTime + bound (pp6: 10 ms fitted on 00002232/34, else 3)
TOGGLES = REPO / 'tools/stopping/sim/toggles_default.json'   # FrogPilot toggle defaults under the logged frogpilotToggles
PCOLS = ('at', 'ss', 'lf', 'armed', 'cmd', 'in_armed', 'cert', 'pers', 'prov', 'cls')   # per arm (line columns NaN without a line)
ICOLS = ('ns', 'log_at', 'log_ss', 'v', 'a', 'd', 'vl', 'tid', 'mp', 'engaged', 'override')


class _SM:
  """SubMaster stand-in on logged events (pp6 FakeSM)."""
  def __init__(self, msgs, prev):
    self.msgs = msgs
    self.logMonoTime = {s: m.logMonoTime for s, m in msgs.items()}
    self.updated = {s: prev is None or prev.get(s) != m.logMonoTime for s, m in msgs.items()}
    self.valid = {s: m.valid for s, m in msgs.items()}
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


def planner_events(route, lo_rel, hi_rel):
  """Logged planner inputs (event readers by service, sorted), the logged plans, carParams and toggles of [lo_rel, hi_rel]."""
  import json
  from openpilot.tools.stopping.review.triage_one import read_events
  by: dict = {s: ([], []) for s in PSERV}
  lps, origin, cp, fpt = [], None, None, None
  paths = seg_paths(route, lo_rel - PLAN_WARMUP_S, hi_rel)
  if not paths:
    raise FileNotFoundError(f'no rlog {route} {lo_rel}-{hi_rel}')
  for p in paths:
    for e in read_events(str(p)):
      w = e.which()
      if w in by:
        by[w][0].append(e.logMonoTime)
        by[w][1].append(e)
        if w == 'frogpilotPlan' and fpt is None and e.frogpilotPlan.frogpilotToggles:
          fpt = e.frogpilotPlan.frogpilotToggles
      elif w == 'longitudinalPlan':
        lps.append(e)
      elif w == 'initData' and origin is None:
        origin = e.logMonoTime
      elif w == 'carParams' and cp is None:
        cp = e.carParams
  for s in by:
    o = sorted(range(len(by[s][0])), key=by[s][0].__getitem__)
    by[s] = ([by[s][0][i] for i in o], [by[s][1][i] for i in o])
  lps.sort(key=lambda e: e.logMonoTime)
  tog = json.loads(TOGGLES.read_text())
  tog.update(json.loads(fpt) if fpt else {})
  return dict(by=by, lps=lps, origin=origin, cp=cp, toggles=SimpleNamespace(**tog), lo=lo_rel, hi=hi_rel)


_LINE: dict = {}


def _wrap_line(LP):
  """Record what update_santa_fe_stop_line sees each tick (the planner calls the module global): the command, the incoming
  armed state, the stop-commit certificate / persistence it is given, the provenance check the code applies to that lead
  (santa_fe_stop_commit_track_provenance_ok with the armed track's own certification) and the stopped class."""
  if _LINE.get('mod') is LP or not hasattr(LP, 'update_santa_fe_stop_line'):
    return
  import inspect
  inner = LP.update_santa_fe_stop_line
  sig = inspect.signature(inner)

  def rec(*a, **k):
    b = sig.bind(*a, **k).arguments
    line, lead = b['line'], b['lead']
    tid_in = line[1] if line is not None else None
    lead_tid = int(getattr(lead, 'radarTrackId', -1)) if lead.status else -1
    cert = bool(b['track_certified'])
    _LINE['seen'] = dict(cmd=float(b['cmd']), in_armed=float(tid_in is not None), cert=float(cert), pers=float(bool(b['persisted'])),
                         prov=float(lead_tid < 0 or LP.santa_fe_stop_commit_track_provenance_ok(
                           lead, b['lead_two'], cert or (tid_in is not None and lead_tid == tid_in))),
                         cls=float(lead_tid >= 0 and float(getattr(lead, 'vLead', 0.0)) <= LP.SANTA_FE_STOP_LINE_STOPPED_V
                                   and float(lead.dRel) <= LP.SANTA_FE_STOP_AIM_STOP_WITHIN_M))
    return inner(*a, **k)
  LP.update_santa_fe_stop_line = rec
  _LINE['mod'] = LP


def planner_run(ev, bounds_ms=(PLAN_BOUND_MS[0],)):
  """Lockstep planner replay of the loaded arm on planner_events ev, one planner per input bound (same arm). Returns
  {bound_ms: columns ICOLS + PCOLS}; ticks with a missing input are skipped."""
  check()
  import openpilot.selfdrive.controls.lib.longitudinal_planner as LP
  _wrap_line(LP)
  by, o0 = ev['by'], ev['origin']

  def latest(s, tns):
    i = bisect.bisect_right(by[s][0], tns) - 1
    return by[s][1][i] if i >= 0 else None
  t_lo, t_hi = o0 + int((ev['lo'] - PLAN_WARMUP_S) * 1e9), o0 + int((ev['hi'] + 0.5) * 1e9)
  out = {}
  for bound in bounds_ms:
    pl = LP.LongitudinalPlanner(ev['cp'])
    rows: dict = {c: [] for c in ICOLS + PCOLS}
    prev = None
    for lp in ev['lps']:
      if not t_lo <= lp.logMonoTime <= t_hi:
        continue
      md = lp.longitudinalPlan.modelMonoTime
      msgs = {s: latest(s, md + bound * 1_000_000) for s in PSERV}
      msgs['modelV2'] = latest('modelV2', md)
      if any(x is None for x in msgs.values()):
        continue
      sm = _SM(msgs, prev)
      prev = {s: m.logMonoTime for s, m in msgs.items()}
      _LINE.pop('seen', None)
      pl.update(sm, ev['toggles'])
      pm = _PM()
      pl.publish(sm, pm, ev['toggles'])
      po = pm.last.longitudinalPlan
      sl = getattr(pl, 'stop_line', None)
      seen = _LINE.get('seen', {})
      cs, ld = sm['carState'], sm['radarState'].leadOne
      st = bool(ld.status)
      for k, x in (('ns', lp.logMonoTime), ('log_at', lp.longitudinalPlan.aTarget), ('log_ss', lp.longitudinalPlan.shouldStop),
                   ('v', cs.vEgo), ('a', cs.aEgo), ('d', ld.dRel if st else np.nan), ('vl', ld.vLead if st else np.nan),
                   ('tid', ld.radarTrackId if st else np.nan), ('mp', ld.modelProb if st else np.nan),
                   ('engaged', sm['controlsState'].longControlState != 0 and sm['selfdriveState'].enabled),
                   ('override', cs.gasPressed or cs.brakePressed), ('at', po.aTarget), ('ss', po.shouldStop),
                   ('lf', np.nan if sl is None else sl[0]), ('armed', np.nan if sl is None else float(sl[1] is not None))):
        rows[k].append(float(x))
      for k in ('cmd', 'in_armed', 'cert', 'pers', 'prov', 'cls'):
        rows[k].append(seen.get(k, np.nan))
    out[bound] = {k: np.asarray(v, dtype=np.float64) for k, v in rows.items()}
  return out


def inputs(d):
  """Logged per-frame inputs of a span (route-relative t)."""
  fr, o = d['frames'], d['origin']
  g = lambda fn: np.asarray([fn(f) for f in fr])  # noqa: E731
  return dict(t=g(lambda f: f['t'] - o), v=g(lambda f: f['cs']['vEgo']), gas=g(lambda f: int(f['cs']['gasPressed'])),
              brake=g(lambda f: int(f['cs']['brakePressed'])), act=g(lambda f: int(f['active'])), rec=g(lambda f: f['recorded']),
              lead=g(lambda f: int(bool(f['kw']['lead_status']))), gap=g(lambda f: f['kw']['lead_d_rel']), vl=g(lambda f: f['kw']['lead_v']),
              tid=g(lambda f: f['kw']['lead_track_id']))

