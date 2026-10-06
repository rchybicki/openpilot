"""Standard stopping simulator runner: one command runs the fixed gate list for HEAD or a candidate diff (README.md).

  .venv/bin/python -m openpilot.tools.stopping.sim.run_gates [--base REV] [--diff FILE] [--on K=V,...] [--off K=V,...] [--car REV]
                                                            [--label X] [--quick] [--with-h] [--compute-only]

Arms (one source tree + flag file each, content-addressed under SIM_HOME/arms/, so a repeated run reuses every finished job):
  HEAD = the base tree; ON = base + diff with the candidate flags on; OFF = base + diff with the candidate flags off (must equal HEAD).
  Candidate flags = stopping_flags.py `NAME = True|False` switches that the diff adds or whose default it changes: ON uses True for a
  new flag and the diff's value for a changed one, OFF the base value (False for a new flag); --on / --off override (any module-level
  flag, derived ones too). The values are written into the arm's copy of stopping_flags.py (derived flags follow). An arm's key is
  (production code hash, flag file sha1, engine sha[, reference replay arm]), so two commits that differ only in docs share rows.
  Order: closed holds (HEAD, ON), exact replay (HEAD, OFF, ON; the HEAD plans are the reference of the others), standard closed set
  (HEAD, ON), OFF identity set, then the H2 confirm cells (L42P + H of a brake-off-band failure, HEAD and ON, arms/<key>/confirm/).
  --with-h adds the history-trigger cell H in its own arm subdirectory (never gating, never mixed into the arm rows). --quick runs
  bm + named + syn at L42 and prints no verdict. --prune deletes the arms of other engines.
Outputs: SIM_HOME/runs/<YYYYmmdd-HHMM>_<label>/gate.md + gate.json. One sweep at a time per machine (SWEEP_LOCK), 2 workers.
Exit status: 0 = every hard gate PASS (or --quick / a complete --compute-only), 1 = a gate FAIL, 2 = no valid verdict (INCOMPLETE:
a job failed or is missing; or NO COVERAGE: a hard gate had nothing to check), 64 = bad input.
"""
import argparse
import fcntl
import hashlib
import json
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path

from openpilot.tools.stopping.sim import SIM_HOME, loader

PY = sys.executable
CORPORA = ('c1', 'c2')
OFF_HOLD_STRIDE = 4   # the OFF identity arm runs every 4th census hold (the exact replay covers every hold frame flags-off)
CAR_REF = 'origin/!my-fp-new'   # default car build: the pushed branch the device deploys (PLAN: deploy = push + Full Update)
BOOL = {'true': True, 'false': False, '1': True, '0': False}
# one sweep at a time on this machine, whatever STOP_SIM_HOME is (a scratch home must not run beside the main sweep: 2 workers)
SWEEP_LOCK = Path.home() / '.route_sync' / 'work' / 'sim_sweep.lock'


class InputError(Exception):
  pass


def parse_kv(s):
  out = {}
  for kv in filter(None, (s or '').split(',')):
    k, sep, v = kv.partition('=')
    if not sep or not k or v.lower() not in BOOL:
      raise InputError(f'bad flag assignment {kv!r}: expected NAME=true|false (or 1|0), comma separated')
    out[k] = BOOL[v.lower()]
  return out


def candidate_flags(m, on=None, off=None):
  """(ON flags, OFF flags) as full assignments of the candidate flags."""
  new = {k: v for k, v in m['flags'].items() if k not in m['base_flags']}
  changed = {k: v for k, v in m['flags'].items() if k in m['base_flags'] and m['base_flags'][k] != v}
  f_on = dict.fromkeys(new, True) | changed | (on or {})
  f_off = dict.fromkeys(new, False) | {k: m['base_flags'][k] for k in changed} | (off or {})
  return f_on, f_off


def engine_sha():
  code = ('from openpilot.tools.stopping.sim import worker as W; import json; '
          + 'print(json.dumps([W.ENGINE_SHA, W.engine_sha(W.REPLAY_FILES)[0] if W.REPLAY_FILES[0].is_file() else None, W.ENGINE]))')
  r = subprocess.run([PY, '-c', code],
                     capture_output=True, text=True, check=True, env=dict(os.environ, STOP_SIM_TREE=''))
  return json.loads(r.stdout.strip().splitlines()[-1])


def arm(m, flags, eng, ref=None):
  """Arm descriptor. The flag values that differ from the tree's file (evaluated) are written into the arm's flag file; its sha1 keys
  the arm. ref: the reference replay arm (the HEAD plans) unless this arm has the same code and flag file."""
  defaults = loader.flag_values(loader.git('show', f"{m['tree']}:{loader.FLAGS_FILE}").decode())
  ov = {k: v for k, v in sorted(flags.items()) if k not in defaults or defaults[k] is not v}
  am = loader.arm_manifest(m, ov)
  if ref is not None and (ref['code_hash'], ref['flags_sha1']) == (m['code_hash'], am['flags_sha1']):
    ref = None
  key = hashlib.sha1(json.dumps([m['code_hash'], am['flags_sha1'], eng] + ([ref['key']] if ref else [])).encode()).hexdigest()[:16]
  d = SIM_HOME / 'arms' / key
  d.mkdir(parents=True, exist_ok=True)
  info = dict(key=key, code_hash=m['code_hash'], tree=m['tree'], base_sha=m['base_sha'], diff_sha1=m['diff_sha1'], overrides=ov,
              flags_sha1=am['flags_sha1'], engine=eng, ref=ref['key'] if ref else None)
  if not (d / 'arm.json').is_file():
    (d / 'arm.json').write_text(json.dumps(info, indent=1))
  return dict(info, dir=d, manifest=am['path'], ref_dir=ref['dir'] if ref else None, flag_values=am['flag_values'])


def run_worker(a, jobs, log):
  """Runs one arm's jobs; returns (seconds, exit status). A non-zero status (failed jobs) does not stop the run: the job check marks
  the run INCOMPLETE."""
  if not jobs:
    return 0.0, 0
  jp = a['dir'] / f'jobs_{jobs[0]["kind"]}_{hashlib.sha1(json.dumps(jobs).encode()).hexdigest()[:8]}.json'
  jp.write_text(json.dumps(jobs))
  env = dict(os.environ, STOP_SIM_TREE=a['manifest'])
  env.pop('STOP_SIM_FLAGS', None)
  t0 = time.monotonic()
  with open(log, 'a') as fh:
    fh.write(f'\n== {time.strftime("%H:%M:%S")} arm {a["key"]} {len(jobs)} {jobs[0]["kind"]} jobs\n')
    fh.flush()
    r = subprocess.run([PY, '-m', 'openpilot.tools.stopping.sim.worker', str(a['dir']), str(jp)], stdout=fh, stderr=subprocess.STDOUT, env=env)
  return time.monotonic() - t0, r.returncode


def closed_jobs(spec, cells_only=None):
  code = 'import json; from openpilot.tools.stopping.sim import cases as C; ' + f'print(json.dumps(C.jobs(C.{spec}, {cells_only!r})))'
  r = subprocess.run([PY, '-c', code], capture_output=True, text=True, check=True,
                     env=dict(os.environ, STOP_SIM_TREE=''))
  return [dict(kind='closed', case=c, cell=cell, start=s, mode=mode) for c, cell, s, mode in json.loads(r.stdout.strip().splitlines()[-1])]


def f1_jobs():
  """The F1 following matrix (f1.py; builder C): closed jobs, rows in arms/<key>/f1/ (never mixed into the stop gates' rows)."""
  code = 'import json; from openpilot.tools.stopping.sim import f1; print(json.dumps(f1.jobs()))'
  r = subprocess.run([PY, '-c', code], capture_output=True, text=True, check=True, env=dict(os.environ, STOP_SIM_TREE=''))
  return [dict(kind='closed', case=c, cell=cell, start=s, mode=mode) for c, cell, s, mode in json.loads(r.stdout.strip().splitlines()[-1])]


def replay_jobs(corpus, a=None):
  """The spans of a corpus (longest first); with an arm: its plan directory and the reference arm's (planner lockstep)."""
  spans = [s for s in json.loads((SIM_HOME / 'frames' / f'{corpus}.json').read_text())['spans'] if 'error' not in s]   # 'error': left out, noted
  extra = {} if a is None else dict(plan_dir=str(a['dir'] / 'plan' / corpus), ref_plan_dir=str(a['ref_dir'] / 'plan' / corpus) if a['ref_dir'] else None)
  return [dict(kind='replay', corpus=corpus, span=s['span'], **extra) for s in sorted(spans, key=lambda s: -(s['hi'] - s['lo']))]


def job_key(j):
  return (j['case'], j['cell'], str(j['start']), j['mode']) if j['kind'] == 'closed' else (j['corpus'], j['span'])


def car_check(car, base_sha, cand_tree=None, arm_flags=None):
  """The car build vs the base and vs the candidate tree: production .py / compiled files (ROOTS, tests excluded) that differ. When the
  car differs from the candidate tree only in stopping_flags.py, car_arm names the arms whose flag values the car has (a deployed
  trial = base + diff, on or off)."""
  car_sha = loader.git('rev-parse', f'{car}^{{commit}}').decode().strip()

  def differ(rev):
    return [p for p in loader.git('diff', '--name-only', car_sha, rev, '--', *loader.ROOTS).decode().split() if not loader.is_test(p)
            and p.endswith(('.py',) + loader.COMPILED)]
  files = differ(base_sha)
  out = dict(car=car, car_sha=car_sha, same_code=not files, differ=files[:20], n_differ=len(files))
  if cand_tree is not None:
    cf = differ(cand_tree)
    out.update(same_as_candidate=not cf, cand_differ=cf[:20])
    if cf == [loader.FLAGS_FILE] and arm_flags:
      fl = {k: repr(v) for k, v in loader.flag_values(loader.git('show', f'{car_sha}:{loader.FLAGS_FILE}').decode()).items()}
      out['car_arm'] = [name for name, v in arm_flags.items() if v == fl]
  return out


def prune():
  """Delete the arms of other engines (their rows can never be reused: the engine sha is part of the arm key). -> (arms, bytes)"""
  eng_closed, eng_replay, _ = engine_sha()
  n = size = 0
  for d in sorted((SIM_HOME / 'arms').glob('*/')):
    info = json.loads((d / 'arm.json').read_text()) if (d / 'arm.json').is_file() else None
    if info is None or info.get('engine') in (eng_closed, eng_replay):
      continue
    size += sum(f.stat().st_size for f in d.rglob('*') if f.is_file())
    shutil.rmtree(d)
    n += 1
    print(f'pruned arm {d.name} (engine {info.get("engine")})', flush=True)
  return n, size


def check_inputs(a):
  """Validated inputs (clean errors before anything is written): base, diff, flag assignments, car."""
  try:
    loader.git('rev-parse', '--verify', '--quiet', f'{a.base}^{{commit}}')
  except RuntimeError:
    raise InputError(f'--base {a.base!r} is not a commit of this repository') from None
  if a.diff:
    p = Path(a.diff)
    if not p.is_file():
      raise InputError(f'--diff {a.diff}: no such file')
    if not p.read_bytes().strip():
      raise InputError(f'--diff {a.diff}: empty diff (run without --diff for a HEAD self-check)')
  on, off = parse_kv(a.on), parse_kv(a.off)
  try:
    loader.git('rev-parse', '--verify', '--quiet', f'{a.car}^{{commit}}')
  except RuntimeError:
    raise InputError(f'--car {a.car!r} is not a commit of this repository (pass the deployed car build)') from None
  return on, off


def stage_jobs(spec, with_h):
  std = closed_jobs(spec)
  holds = [j for j in std if j['mode'] == 'nodrv']
  rest = [j for j in std if j['mode'] != 'nodrv']
  hjobs = [dict(j, cell='H') for j in rest if j['cell'] == 'L42'] if with_h else []
  return holds, rest, hjobs


def main(argv=None):
  ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  ap.add_argument('--base', default='HEAD')
  ap.add_argument('--diff')
  ap.add_argument('--on', default='')
  ap.add_argument('--off', default='')
  ap.add_argument('--car', default=CAR_REF, help=f'the car build for the H1 car check (default {CAR_REF})')
  ap.add_argument('--label', default=None)
  ap.add_argument('--quick', action='store_true', help='bm + named + syn at L42 only (iteration; no verdict)')
  ap.add_argument('--with-h', action='store_true', help='also the rejected history-trigger cell H (separate, non-gating section)')
  ap.add_argument('--compute-only', action='store_true')
  ap.add_argument('--stages', default='holds,replay,standard,off,f1')
  ap.add_argument('--recheck', type=int, default=0, help='re-run N cached HEAD closed jobs fresh and require identical rows (determinism)')
  ap.add_argument('--prune', action='store_true', help='delete the arms of other engines (stale rows) and exit')
  a = ap.parse_args(argv)
  if a.prune:
    SWEEP_LOCK.parent.mkdir(parents=True, exist_ok=True)
    with open(SWEEP_LOCK, 'w') as lk:
      try:
        fcntl.flock(lk, fcntl.LOCK_EX | fcntl.LOCK_NB)
      except BlockingIOError:
        print(f'run_gates: another sim sweep holds {SWEEP_LOCK}', file=sys.stderr)
        return 64
      n, size = prune()
    print(f'pruned {n} arms, {size / 1e9:.1f} GB')
    return 0
  try:
    on, off = check_inputs(a)
    stages = a.stages.split(',')
    if not set(stages) <= {'holds', 'replay', 'standard', 'off', 'f1'}:
      raise InputError(f'--stages {a.stages!r}: choose from holds,replay,standard,off,f1')
    base_m = loader.build(a.base)
    try:
      cand_m = loader.build(a.base, a.diff) if a.diff else base_m
    except RuntimeError as exc:
      raise InputError(f'--diff {a.diff} does not apply to {a.base}: ' + ' | '.join(str(exc).splitlines())) from None
    except ValueError as exc:
      raise InputError(f'--diff {a.diff}: {exc}') from None
    unknown = sorted(k for k in {**on, **off} if k not in cand_m['flag_names'])
    if unknown:
      raise InputError(f"unknown flags {unknown} for the tree under test (stopping_flags.py defines {', '.join(cand_m['flag_names'])})")
    f_on, f_off = candidate_flags(cand_m, on, off)
  except InputError as exc:
    print(f'run_gates: {exc}', file=sys.stderr)
    return 64
  (SIM_HOME / 'runs').mkdir(parents=True, exist_ok=True)
  SWEEP_LOCK.parent.mkdir(parents=True, exist_ok=True)
  lock = open(SWEEP_LOCK, 'w')
  try:
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
  except BlockingIOError:
    lock.close()
    print(f'run_gates: another sim sweep holds {SWEEP_LOCK} (one sweep at a time on this machine)', file=sys.stderr)
    return 64
  t_start = time.monotonic()
  eng_closed, eng_replay, eng_files = engine_sha()
  try:
    arms = {'HEAD': arm(base_m, {}, eng_closed), 'ON': arm(cand_m, f_on, eng_closed), 'OFF': arm(cand_m, f_off, eng_closed)}
    rh = arm(base_m, {}, eng_replay)
    rarms = {'HEAD': rh, 'ON': arm(cand_m, f_on, eng_replay, ref=rh), 'OFF': arm(cand_m, f_off, eng_replay, ref=rh)}
  except ValueError as exc:   # unknown flag / not exactly one definition
    print(f'run_gates: {exc}', file=sys.stderr)
    lock.close()
    return 64
  car = car_check(a.car, base_m['base_sha'], cand_m['tree'] if a.diff else None, {k: arms[k]['flag_values'] for k in ('ON', 'OFF')})
  label = a.label or (Path(a.diff).stem if a.diff else 'head')
  run_dir = SIM_HOME / 'runs' / f'{time.strftime("%Y%m%d-%H%M")}_{label}'
  run_dir.mkdir(parents=True, exist_ok=True)
  log = run_dir / 'run.log'
  timing = {}
  spec = 'QUICK' if a.quick else 'STANDARD'
  if a.quick:
    stages = ['standard']
  arm_keys = ', '.join(f'{k}: {v["key"]}' for k, v in arms.items())
  print(f'run {run_dir.name}: base {base_m["base_sha"][:10]} diff {cand_m["diff_sha1"]} flags ON {f_on} OFF {f_off}; arms {arm_keys}; '
        + f"car {car['car_sha'][:10]} same code {car['same_code']}; log {log}", flush=True)
  holds, rest, hjobs = stage_jobs(spec, a.with_h)
  h_arms = {k: dict(arms[k], dir=arms[k]['dir'] / 'cellH', key=arms[k]['key'] + '/cellH') for k in ('HEAD', 'ON')} if hjobs else {}
  # the required job set is the whole spec (every stage, every census hold), whatever --stages computes and whatever is cached: a
  # stage that is not computed here must already have its rows, else the run is INCOMPLETE
  todo = [('holds', 'HEAD', holds), ('holds', 'ON', holds)]
  todo += [(f'replay_{c}', k, replay_jobs(c, rarms[k])) for k in ('HEAD', 'OFF', 'ON') for c in CORPORA] if not a.quick else []
  todo += [('standard', 'HEAD', rest), ('standard', 'ON', rest)]
  todo += [('off', 'OFF', [j for j in rest if j['cell'] == 'L42'] + holds[::OFF_HOLD_STRIDE])] if not a.quick else []
  if hjobs:
    todo += [('cellH', 'HEAD', hjobs), ('cellH', 'ON', hjobs)]
  # F1 following matrix (builder C): HEAD, ON and the OFF identity, in arms/<key>/f1/
  f_arms = {k: dict(arms[k], dir=arms[k]['dir'] / 'f1', key=arms[k]['key'] + '/f1') for k in ('HEAD', 'ON', 'OFF')} if not a.quick else {}
  if f_arms:
    fj = f1_jobs()
    todo += [('f1', k, fj) for k in ('HEAD', 'ON', 'OFF')]
  expected: dict = {}   # (kind, arm name) -> job keys this run needs
  done_keys, worker_status = set(), []
  for stage, name, jobs in todo:
    A = h_arms[name] if stage == 'cellH' else f_arms[name] if stage == 'f1' else rarms[name] if stage.startswith('replay') else arms[name]
    kind = 'cellH' if stage == 'cellH' else 'f1' if stage == 'f1' else 'replay' if stage.startswith('replay') else 'closed'
    expected.setdefault((kind, name), set()).update(job_key(j) for j in jobs)
    if stage != 'cellH' and stage.split('_')[0] not in stages:
      continue
    k = (A['key'], stage, len(jobs))
    if k in done_keys:   # HEAD == ON (no diff) or OFF == HEAD: the same arm already ran these jobs
      continue
    done_keys.add(k)
    A['dir'].mkdir(parents=True, exist_ok=True)
    dt, rc = run_worker(A, jobs, log)
    timing[f'{stage}_{name}'] = round(dt, 1)
    if rc:
      worker_status.append(dict(stage=stage, arm=name, key=A['key'], status=rc))
    print(f'  {stage} {name} ({A["key"]}): {len(jobs)} jobs {dt:.0f} s' + (f' WORKER STATUS {rc}' if rc else ''), flush=True)
  from openpilot.tools.stopping.sim import gates
  # H2 confirm cells (PLAN 82): a band failure in the level plant counts only if L42P or H agrees -> run those cells for HEAD and ON
  c_arms = {k: dict(arms[k], dir=arms[k]['dir'] / 'confirm', key=arms[k]['key'] + '/confirm') for k in ('HEAD', 'ON')}
  if not a.quick:
    keys = expected.get(('closed', 'HEAD'), set()) & expected.get(('closed', 'ON'), set())
    cj = [dict(kind='closed', case=c, cell=cell, start=st, mode=md) for c, cell, st, md in
          gates.h2_confirm_jobs(arms['HEAD']['dir'], arms['ON']['dir'], keys)]
    for name in ('HEAD', 'ON'):
      expected[('confirm', name)] = {job_key(j) for j in cj}
      k = (c_arms[name]['key'], 'confirm', len(cj))
      if cj and k not in done_keys:
        done_keys.add(k)
        c_arms[name]['dir'].mkdir(parents=True, exist_ok=True)
        dt, rc = run_worker(c_arms[name], cj, log)
        timing[f'confirm_{name}'] = round(dt, 1)
        if rc:
          worker_status.append(dict(stage='confirm', arm=name, key=c_arms[name]['key'], status=rc))
        print(f'  confirm {name} ({c_arms[name]["key"]}): {len(cj)} jobs {dt:.0f} s' + (f' WORKER STATUS {rc}' if rc else ''), flush=True)
  recheck = None
  if a.recheck:
    cached = sorted(gates.closed_rows(arms['HEAD']['dir']).items())
    sample = cached[::max(len(cached) // a.recheck, 1)][:a.recheck]
    rd = arms['HEAD']['dir'] / 'recheck' / run_dir.name
    rd.mkdir(parents=True, exist_ok=True)
    jobs = [dict(kind='closed', case=r['case'], cell=r['cell'], start=r['start'], mode=r['mode']) for _, r in sample]
    dt, _ = run_worker(dict(arms['HEAD'], dir=rd), jobs, log)
    fresh = gates.closed_rows(rd)
    bad = [list(k) for k, r in sample if k not in fresh or not gates.same_row(r, fresh[k])]
    recheck = dict(n=len(sample), identical=len(sample) - len(bad), differ=bad, secs=round(dt, 1))
    print(f'  recheck: {recheck["identical"]}/{recheck["n"]} fresh HEAD runs identical to the cache ({dt:.0f} s)', flush=True)
  census = SIM_HOME / 'census' / 'holds.json'   # census holds the job set leaves out (entries with an 'error'), with the reason
  excluded = [f"{h['route'][:8]}_{h['t_start']:.2f}: {h['error']}" for h in (json.loads(census.read_text()) if census.is_file() else []) if 'error' in h]
  excluded += [f"{c}/{sp['span']}: {sp['error']}" for c in CORPORA for sp in json.loads((SIM_HOME / 'frames' / f'{c}.json').read_text())['spans']
               if 'error' in sp]   # replay spans the job set leaves out (frames/<corpus>.json entries with an 'error')
  meta = dict(run=run_dir.name, recheck=recheck, base=a.base, base_sha=base_m['base_sha'], diff=a.diff, diff_sha1=cand_m['diff_sha1'],
              touched=cand_m['touched'], car=car, quick=a.quick, with_h=a.with_h,
              flags_on=f_on, flags_off=f_off,
              arms={k: {kk: v[kk] for kk in ('key', 'code_hash', 'tree', 'overrides', 'flags_sha1')} for k, v in arms.items()},
              replay_arms={k: v['key'] for k, v in rarms.items()}, engine=eng_closed, engine_replay=eng_replay, engine_files=eng_files,
              spec=spec, stages=stages, timing_s=timing, worker_status=worker_status, census_excluded=excluded,
              n_jobs=dict(holds=len(holds), standard=len(rest), cellH=len(hjobs), replay=sum(len(replay_jobs(c)) for c in CORPORA),
                          f1=len(fj) if f_arms else 0))
  (run_dir / 'meta.json').write_text(json.dumps(meta, indent=1, default=str))
  missing = gates.missing_jobs({k: v['dir'] for k, v in arms.items()}, {k: v['dir'] for k, v in rarms.items()},
                               {k: v['dir'] for k, v in h_arms.items()}, expected, {k: v['dir'] for k, v in c_arms.items()},
                               {k: v['dir'] for k, v in f_arms.items()})
  if a.compute_only:
    n_miss = sum(len(v['keys']) for v in missing)
    print('compute only:', run_dir, f'{time.monotonic() - t_start:.0f} s', f'INCOMPLETE: {n_miss} jobs missing' if n_miss else 'all jobs present')
    lock.close()
    return 2 if n_miss else 0
  res = gates.evaluate(meta, {k: v['dir'] for k, v in arms.items()}, {k: v['dir'] for k, v in rarms.items()}, expected=expected,
                       missing=missing, quick=a.quick, h_arms={k: v['dir'] for k, v in h_arms.items()},
                       confirm_arms={k: v['dir'] for k, v in c_arms.items()}, f1_arms={k: v['dir'] for k, v in f_arms.items()})
  res['meta']['wall_s'] = round(time.monotonic() - t_start, 1)
  (run_dir / 'gate.json').write_text(json.dumps(res, indent=1, default=str))
  (run_dir / 'gate.md').write_text(gates.render(res))
  print(gates.render(res, short=True))
  print('wrote', run_dir / 'gate.md')
  lock.close()   # releases the sweep lock (an in-process caller keeps running)
  return 0 if res['verdict'] == 'PASS' or a.quick else 1 if res['verdict'] == 'FAIL' else 2


if __name__ == '__main__':
  sys.exit(main())
