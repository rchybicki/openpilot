"""One arm of a gate run (one source tree + one flag file per process tree): closed-loop jobs into ARM/closed.pkl, or exact-replay
spans into ARM/replay_<corpus>.pkl (+ the span plans in ARM/plan/<corpus>/<span>.npz).

  STOP_SIM_TREE=<arm manifest.json> python -m openpilot.tools.stopping.sim.worker ARM_DIR JOBS.json

The loader is installed at import (spawn children re-import this module before they unpickle a job). The flags are the arm's
stopping_flags.py snapshot (loader.arm_manifest: the overrides are written into the file, so derived flags follow); nothing sets
module attributes. Every job asserts the tree (mapped module files and sha1), every module-level flag against the flag file
evaluated as Python, the engine files (sha1 recorded at import) and the package location of the engine. Failed jobs are written
to ARM_DIR/errors_<jobs file>.json and make the exit status 3. At most 2 spawn workers (SIM_PROCS)."""
import hashlib
import json
import os
import pickle
import sys
import time
import traceback
from pathlib import Path

from openpilot.tools.stopping.sim import loader

loader.install_from_env()
import numpy as np

HERE = Path(__file__).resolve().parent
REVIEW = loader.REPO / 'tools/stopping/review'
# executed by both engines and never mapped from the tree under test (tools and test helpers import from the working tree): the
# loader, this worker, the package files and the Hyundai sender test helper (run_frame) with its package files
COMMON_FILES = [HERE / f for f in ('__init__.py', 'loader.py', 'worker.py')] + [loader.REPO / 'tools/__init__.py'] + \
               [loader.REPO / 'opendbc_repo/opendbc/car' / f for f in ('tests/__init__.py', 'hyundai/tests/__init__.py', 'hyundai/tests/conftest.py',
                                                                       'hyundai/tests/test_can_bounds_fork.py')]
ENGINE_FILES = [HERE / f for f in ('harness.py', 'rharness.py', 'gear.py', 'gated.py', 'terminal.py', 'cases.py', 'metrics.py',
                                   'toggles_default.json', 'f1.py', 'radar_replay.py')] + \
               [REVIEW / f for f in ('kcs_plant.py', 'plant_data.py', 'plant_sim.py', 'kcs1_reps.py', 'can_response.py', 'stop_harness.py')] + COMMON_FILES
REPLAY_FILES = [HERE / 'replay.py', HERE / 'radar_replay.py', HERE / 'toggles_default.json'] + COMMON_FILES   # planner pass: toggle defaults


def sha_files(files):
  out = {}
  for f in files:
    if not f.is_file():
      raise FileNotFoundError(f'engine file missing: {f} (the untracked tools/stopping/review plant files are required)')
    out[str(f.relative_to(loader.REPO))] = hashlib.sha1(f.read_bytes()).hexdigest()
  return out


def engine_sha(files=None):
  d = sha_files(files or ENGINE_FILES)
  return hashlib.sha1(json.dumps(d, sort_keys=True).encode()).hexdigest()[:12], d


ENGINE_SHA, ENGINE = engine_sha()
FLAGS = dict(loader.MANIFEST.get('overrides') or {})
_STATE: dict = {}


def unhashed(files):
  """Imported repo files outside the fingerprint (files) that always load from the working tree: tools/stopping and the test
  helpers under loader.ROOTS. Each one would change the rows without changing the arm key."""
  root, have = str(loader.REPO) + '/', {str(f) for f in files}
  out = set()
  for m in list(sys.modules.values()):
    f = os.path.realpath(getattr(m, '__file__', None) or '')
    rel = f[len(root):] if f.startswith(root) else ''
    if (rel.startswith('tools/stopping/') or (rel.startswith(loader.ROOTS) and loader.is_test(rel))) and f not in have:
      out.add(rel)
  return sorted(out)


def check_env(replay=False):
  assert os.environ.get(loader.ENV) and loader.MANIFEST, 'no source tree installed (STOP_SIM_TREE)'
  loader.check()
  assert engine_sha()[0] == ENGINE_SHA, 'engine files changed during the run'
  if replay:
    assert sha_files(REPLAY_FILES) == _STATE.setdefault('replay_sha', sha_files(REPLAY_FILES)), 'replay files changed during the run'
  bad = unhashed(REPLAY_FILES if replay else ENGINE_FILES)
  assert not bad, f'executed files outside the engine fingerprint (add them to worker.py): {bad}'
  from openpilot.selfdrive.controls.lib import stopping_flags as SF
  ent = loader.MANIFEST['mapped'].get(loader.FLAGS_MODULE)
  assert ent and os.path.realpath(SF.__file__) == os.path.realpath(ent['file']), ('flags not from the arm flag file', SF.__file__)
  want = _STATE.setdefault('flags', loader.flag_values(Path(ent['file']).read_text()))
  have = {k: v for k, v in vars(SF).items() if k.isupper() and not k.startswith('_')}
  bad = sorted(k for k in set(want) | set(have) if k not in want or k not in have or type(want[k]) is not type(have[k]) or want[k] != have[k])
  assert not bad, ('module flags differ from the arm flag file', {k: (have.get(k), want.get(k)) for k in bad})
  assert {k: repr(v) for k, v in want.items()} == loader.MANIFEST['flag_values'], 'flag file differs from the manifest'
  for k, v in FLAGS.items():
    assert want[k] is v, ('override not in effect', k, want[k], v)
  if not replay:
    from openpilot.tools.stopping.sim import rharness as R
    assert os.path.realpath(R.__file__) == str(HERE / 'rharness.py'), R.__file__
    assert R.PINNED is loader.PINNED




# ---- closed loop -------------------------------------------------------------------------------------------------------------
SR: dict = {'v': None, 'rec': []}
LINE: list = []
WIN_COLS = (('t', np.float64), ('v', np.float32), ('v_true', np.float32), ('x', np.float32), ('wire', np.float32), ('sent', np.float32),
            ('lcs', np.int8), ('gap', np.float32), ('gap_meas', np.float32), ('lead_v', np.float32), ('a_target', np.float32),
            ('should_stop', np.int8), ('gas', np.int8), ('brake', np.int8), ('active', np.int8), ('svc_active', np.int8), ('owning', np.int8),
            ('a_real', np.float32), ('closed', np.int8), ('lead_status', np.int8), ('vl_true', np.float32), ('line_floor', np.float32),
            ('plant_off', np.int8), ('gear', np.int8))   # last 4: H5 lead speed, H6 line ownership, H2 brake-off band (PLAN 82)
PH = {'INACTIVE': 0, 'APPROACH_GLIDE': 1, 'PRE_STOP_EASE': 2, 'RAMP_TO_HOLD': 3, 'HOLD': 4, 'RELEASE': 5, 'CREEP': 6}
SHA_COLS = ('wire', 'sent', 'v_true', 'lcs')
LAUNCH_V, LAUNCH_TAIL = 0.5, 2.0   # drv compact trace: through the first launch after the stop (v_true > LAUNCH_V) + LAUNCH_TAIL s


def _install_sr_capture(H):
  """SCC12 StopReq per frame (the plant gate input; the trace does not store it), as the E3 closed-loop runner (ccl2.py)."""
  if getattr(H, '_sim_sr', False):
    return
  orig_setup = H._setup

  def setup(c):
    cp, lc, tg, CI, sender, rf, gs = orig_setup(c)
    SR['v'], SR['rec'] = None, []

    def rf2(snd, d, _rf=rf, _gs=gs):
      r = _rf(snd, d)
      if 0x421 in r[1]:
        SR['v'] = bool(_gs('SCC12', 'StopReq', r[1][0x421]))
      SR['rec'].append(-1 if SR['v'] is None else int(SR['v']))
      return r
    return cp, lc, tg, CI, sender, rf2, gs
  H._setup = setup
  H._sim_sr = True


def _line_variant():
  """Records the planner's stop line (getattr stop_line; None on trees without one) per planner tick (vr4.py)."""
  import contextlib
  from openpilot.tools.stopping.sim import harness as H
  import openpilot.selfdrive.controls.lib.longitudinal_planner as LP

  @contextlib.contextmanager
  def ctx():
    base_upd = LP.LongitudinalPlanner.update

    def upd(self, sm, toggles):
      base_upd(self, sm, toggles)
      sl = getattr(self, 'stop_line', None)
      LINE.append((np.nan if sl is None else float(sl[0]), -1 if sl is None or sl[1] is None else int(sl[1])))
    with H.patched((LP.LongitudinalPlanner, 'update', upd)):
      yield
  return ctx()


def _variant(ms):
  """The run variant: the stop-line recorder, plus the model's shouldStop bit into LongControl on the model-only stop family."""
  import contextlib
  from openpilot.tools.stopping.sim import cases as C

  @contextlib.contextmanager
  def ctx():
    with _line_variant(), (C.ms_model_bit() if ms else contextlib.nullcontext()):
      yield
  return ctx


def closed_job(j):
  from openpilot.tools.stopping.sim import harness as H
  from openpilot.tools.stopping.sim import rharness as R
  from openpilot.tools.stopping.sim import cases as C
  from openpilot.tools.stopping.sim import metrics as MT
  from openpilot.tools.stopping.review.kcs_plant import Cell
  cid, cell, start, mode = j['case'], j['cell'], j['start'], j['mode']
  _install_sr_capture(H)
  # ---- F1 following matrix (radar-input changes, f1.py): cell 'F1<config>[_lv<lag>]', the higher-speed plant F1.PLANT ----
  f1 = None
  if cell.startswith('F1'):
    from openpilot.tools.stopping.sim import f1 as F1
    cfg, ropts = F1.cell_opts(cell)
    (trig, thr, gd), frac = F1.PLANT
    case, st, meta = F1.case(cid, cfg), start, {}
    f1 = F1
  else:
    (trig, thr, gd), frac = C.CELLS[cell]
    case, st, meta = C.make(cid, start, mode)
    ropts = {}
  if mode == 'nodrv' and not cid.startswith('sv_'):   # recorded stop without the driver: strip from the takeover (ccl2 R.NEW rule)
    c0 = H.case(cid)
    st = R.resolve_start(c0, 'auto')
    case = C.strip_driver(c0, st)
    meta.update(t_strip=st, t_dis=C.first_disengage(c0, st))
  LINE.clear()
  R._instrument()
  r = R.run(case, variant=_variant(C.is_ms(cid)), cell=Cell(trigger=trig, threshold=thr, gain_delta=gd), start=st, frac=frac, creep=dict(off_grade=0.0),
            cruise_standstill='car', standstill='gate', keep_trace=True, e2e='proxy' if cell.endswith('P') else 'replay', **ropts)
  tr, m = r['trace'], r['metrics']
  n = len(tr['t'])
  sr = np.array(SR['rec'], dtype=np.int8)
  if len(sr) != n:
    raise RuntimeError(f'StopReq capture length {len(sr)} != trace {n}')
  tr['stopreq'] = sr
  tr['phase_i'] = np.array([PH.get(str(p), -1) for p in tr['phase']], dtype=np.int8)
  if isinstance(case, dict) and hasattr(case.get('lead'), 'true_x'):   # synthetic: the true gap from the lead object (ccl2)
    tr['gap_true'] = np.array([case['lead'].true_x(tt + R.SYN_T0) for tt in tr['t']]) - np.asarray(tr['x'])
  pl = [r['plans'][k] for k in sorted(r['plans'])]
  if pl and len(LINE) == len(pl):
    pt = np.array([p['t'] for p in pl])
    ln = np.array([x[0] for x in LINE], np.float64)
    ix = np.clip(np.searchsorted(pt, tr['t'], side='right') - 1, 0, len(pt) - 1)
    tr['line_floor'] = np.where(tr['t'] >= pt[0], ln[ix], np.nan)
  closed = np.asarray(tr['closed']).astype(bool)
  k0 = int(np.argmax(closed)) if closed.any() else 0
  h = hashlib.sha1()
  for k in SHA_COLS:
    h.update(np.ascontiguousarray(np.nan_to_num(np.asarray(tr[k], dtype=np.float64)[k0:], nan=-99.0)).tobytes())
  h.update(tr['phase_i'][k0:].tobytes())
  h.update(sr[k0:].tobytes())
  lo = r['info']['start'] if r['info'].get('start') is not None else m.get('t_lo')
  row = dict(case=cid, cell=cell, start=start, mode=mode, group=C.group_of(cid), meta=meta,
             info={k: r['info'].get(k) for k in ('start', 'grade', 'gear', 'radar_delay', 'radar_fid', 'radar_warm')},
             m=MT._flat(m), trace_sha=h.hexdigest(), err=None)
  if f1 is not None:
    row['f1'] = f1.measures(tr, case)
  try:
    row['x'] = MT.new_extra(tr, m, lo) if lo is not None else {}
  except Exception as exc:   # a metric failure keeps the row (as vr4.py)
    row['x'] = dict(x_err=repr(exc))
  try:
    row['f'] = MT.full_metrics(tr, m)
  except Exception as exc:
    row['f'] = dict(f_err=repr(exc))
  ts = m.get('t_stop') or tr['t'][-1]
  hi = 6.0
  if mode != 'nodrv' and m.get('t_stop') is not None:   # H5 needs v / gap / lead speed through the first launch (nodrv rows keep 'w')
    la = np.flatnonzero((tr['t'] > ts + 0.3) & (np.asarray(tr['v_true'], dtype=float) > LAUNCH_V))
    hi = max(hi, (float(tr['t'][la[0]]) + LAUNCH_TAIL if len(la) else float(tr['t'][-1])) - ts)
  ctr = MT.compact(tr, m, hi_pad=hi)
  idx = np.flatnonzero((tr['t'] >= ts - 45.0) & (tr['t'] <= ts + hi) & (np.arange(n) % 2 == 0))   # MT.compact's frames
  for k in ('stopreq', 'phase_i', 'lead_v', 'vl_true', 'sent', 'owning', 'gas', 'brake', 'active', 'gap_meas', 'gap_true', 'closed', 'lead_status',
            'x'):
    if k in tr and k not in ctr:
      a = np.asarray(tr[k])[idx]
      ctr[k] = a.astype(np.float32) if a.dtype.kind in 'fiub' else a.astype(str)
  row['tr'] = ctr
  if mode == 'nodrv':   # the full 100 Hz closed window (hold / launch gates, as ccl2.py)
    sl = slice(max(k0 - 50, 0), n)
    w = {k: np.nan_to_num(np.asarray(tr[k], dtype=float), nan=-1).astype(dt) if dt == np.int8 else np.asarray(tr[k], dtype=float).astype(dt)
         for k, dt in WIN_COLS if k in tr}
    w = {k: v[sl] for k, v in w.items()}
    if 'gap_true' in tr:
      w['gap'] = np.asarray(tr['gap_true'])[sl].astype(np.float32)
    w['phase'] = tr['phase_i'][sl]
    w['stopreq'] = sr[sl]
    row['w'] = w
    row['meta']['k0'] = int(k0 - sl.start)
  return row


def job(j):
  t0 = time.monotonic()
  try:
    if j['kind'] == 'replay':
      check_env(replay=True)
      from openpilot.tools.stopping.sim import replay
      out = replay.span_job(j)
      check_env(replay=True)
      return dict(j, err=None, res=out, secs=round(time.monotonic() - t0, 1))
    check_env()
    row = closed_job(j)
    check_env()
    row['secs'] = round(time.monotonic() - t0, 1)
    return row
  except Exception as exc:   # one failed job must not drop the sweep; reported per job
    return dict(j, err=f'{type(exc).__name__}: {exc}\n{traceback.format_exc()[-1500:]}')


def key(r):
  return (r['case'], r['cell'], str(r['start']), r['mode']) if r.get('kind', 'closed') == 'closed' else (r['corpus'], r['span'])


def load_rows(p):
  try:
    with open(p, 'rb') as fh:
      return pickle.load(fh)
  except FileNotFoundError:
    return []


def save_rows(p, rows):
  tmp = f'{p}.{os.getpid()}.tmp'
  with open(tmp, 'wb') as fh:
    pickle.dump(rows, fh)
  os.replace(tmp, p)


def main(argv):
  arm_dir, jobs_path = Path(argv[1]), Path(argv[2])
  procs = max(1, min(int(os.environ.get('SIM_PROCS', '2')), 2))
  jobs = json.loads(jobs_path.read_text())
  kind = jobs[0]['kind'] if jobs else 'closed'
  out = arm_dir / ('closed.pkl' if kind == 'closed' else f"replay_{jobs[0]['corpus']}.pkl")
  rows = load_rows(out)
  have = {key(r) for r in rows if not r.get('err')}
  todo = [j for j in jobs if key(j) not in have]
  tree = loader.MANIFEST.get('tree', '')[:12]
  print(f'arm {arm_dir.name} {kind}: {len(todo)}/{len(jobs)} jobs, engine {ENGINE_SHA}, tree {tree}, flag file '
        + f"{loader.MANIFEST.get('flags_sha1', '')[:12]} overrides {FLAGS}", flush=True)
  err_path = arm_dir / f'errors_{jobs_path.stem}.json'
  err_path.unlink(missing_ok=True)
  if not todo:
    return 0
  from openpilot.tools.stopping.sim import harness as H
  t0 = time.monotonic()
  buf, done, errs = [], 0, []
  pool = H._pool(procs) if procs > 1 else None
  it = pool.imap_unordered(job, todo, chunksize=1) if pool else map(job, todo)
  try:
    for r in it:
      done += 1
      if r.get('err'):
        errs.append(dict(key=list(key(r)), err=r['err'][-1500:]))
        print('ERR', key(r), r['err'][-600:], flush=True)
      else:
        buf.append(r)
      if len(buf) >= 16 or done == len(todo):
        rows = [x for x in load_rows(out) if not x.get('err')] + buf
        save_rows(out, rows)
        buf = []
        print(f'{done}/{len(todo)} {time.monotonic() - t0:.0f}s errors {len(errs)}', flush=True)
  finally:
    if pool is not None:
      pool.close()
      pool.join()
    if errs:
      err_path.write_text(json.dumps(errs, indent=1))
  print('done', len(todo), round(time.monotonic() - t0), 'errors', len(errs), flush=True)
  return 3 if errs else 0


if __name__ == '__main__':
  sys.exit(main(sys.argv))
