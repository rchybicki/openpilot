import json
import os
import pickle
import shutil
import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest

from openpilot.tools.stopping.sim import gates as G
from openpilot.tools.stopping.sim import loader as L
from openpilot.tools.stopping.sim import run_gates as RG


def _dump(path, rows):
  with open(path, 'wb') as fh:
    pickle.dump(rows, fh)


def _load(path):
  with open(path, 'rb') as fh:
    return pickle.load(fh)


# ---- loader ------------------------------------------------------------------------------------------------------------------
def _git(repo, *a):
  return subprocess.run(['git', '-C', str(repo), *a], capture_output=True, text=True, check=True).stdout.strip()


FLAGS_SRC = 'OLD_FLAG = True\nOTHER = 3\nLINE = False\n# derived: follows LINE\nBAND = LINE\n'


@pytest.fixture
def repo(tmp_path, monkeypatch):
  r = tmp_path / 'repo'
  (r / 'selfdrive/controls/lib').mkdir(parents=True)
  (r / 'docs').mkdir()
  (r / 'selfdrive/controls/lib/stopping_flags.py').write_text(FLAGS_SRC)
  (r / 'selfdrive/controls/lib/mod.py').write_text('X = 1\n')
  (r / 'docs/a.md').write_text('a\n')
  _git(tmp_path, 'init', '-q', str(r))
  _git(r, 'add', '.')
  _git(r, '-c', 'user.name=t', '-c', 'user.email=t@t', 'commit', '-qm', 'base')
  monkeypatch.setattr(L, 'REPO', r)
  monkeypatch.setattr(L, 'SIM_HOME', tmp_path / 'sim')
  return r


def _diff(repo, tmp_path, edits):
  for p, txt in edits.items():
    (repo / p).write_text(txt)
  d = subprocess.run(['git', '-C', str(repo), 'diff'], capture_output=True, check=True).stdout
  _git(repo, 'checkout', '--', '.')
  f = tmp_path / 'cand.diff'
  f.write_bytes(d)
  return f


def test_loader_applies_diff_without_touching_the_working_tree(repo, tmp_path):
  f = _diff(repo, tmp_path, {'selfdrive/controls/lib/mod.py': 'X = 2\n',
                             'selfdrive/controls/lib/stopping_flags.py': FLAGS_SRC.replace('OLD_FLAG = True', 'OLD_FLAG = False') + 'NEW_FLAG = True\n'})
  m = L.build('HEAD', f)
  name = 'openpilot.selfdrive.controls.lib.mod'
  assert set(m['mapped']) == {name, L.FLAGS_MODULE}
  with open(m['mapped'][name]['file']) as fh:
    assert fh.read() == 'X = 2\n'
  assert (repo / 'selfdrive/controls/lib/mod.py').read_text() == 'X = 1\n'   # working tree untouched
  assert m['flags'] == {'OLD_FLAG': False, 'LINE': False, 'NEW_FLAG': True} and m['base_flags'] == {'OLD_FLAG': True, 'LINE': False}
  assert m['flag_names'] == ['BAND', 'LINE', 'NEW_FLAG', 'OLD_FLAG', 'OTHER']
  on, off = RG.candidate_flags(m)
  assert on == {'NEW_FLAG': True, 'OLD_FLAG': False} and off == {'NEW_FLAG': False, 'OLD_FLAG': True}
  assert RG.candidate_flags(m, on={'NEW_FLAG': False})[0]['NEW_FLAG'] is False


def test_flag_override_rewrites_the_file_and_derived_flags_follow(repo, tmp_path):
  m = L.build('HEAD')
  on = L.arm_manifest(m, {'LINE': True})
  off = L.arm_manifest(m, {})
  assert on['flag_values']['LINE'] == 'True' and on['flag_values']['BAND'] == 'True'   # derived from LINE (setattr would leave False)
  assert off['flag_values']['BAND'] == 'False' and on['flags_sha1'] != off['flags_sha1']
  assert on['mapped'][L.FLAGS_MODULE]['file'] != off['mapped'][L.FLAGS_MODULE]['file']   # the flag file is always mapped, per arm
  src = (repo / 'selfdrive/controls/lib/stopping_flags.py').read_text()
  assert L.flag_values(L.rewrite_flags(src, {'BAND': True, 'OTHER': 4})) == {'OLD_FLAG': True, 'OTHER': 4, 'LINE': False, 'BAND': True}
  with pytest.raises(ValueError, match='unknown flags'):
    L.arm_manifest(m, {'NOPE': True})
  with pytest.raises(ValueError, match='top-level definitions'):
    L.rewrite_flags('A = True\nA = False\n', {'A': True})


def test_arm_flag_file_is_what_the_process_imports(repo, tmp_path):
  """A process on an arm manifest imports the rewritten flag file (the worker's check_env compares every flag with it)."""
  m = L.arm_manifest(L.build('HEAD'), {'LINE': True})
  code = ('import json; from openpilot.tools.stopping.sim import loader; loader.install_from_env(); '
          + 'from openpilot.selfdrive.controls.lib import stopping_flags as SF; '
          + 'want = loader.flag_values(open(SF.__file__).read()); '
          + 'print(json.dumps([SF.LINE, SF.BAND, want == {k: v for k, v in vars(SF).items() if k.isupper()}]))')
  out = subprocess.run([sys.executable, '-c', code], capture_output=True, text=True, env=dict(os.environ, STOP_SIM_TREE=m['path']))
  assert json.loads(out.stdout.strip().splitlines()[-1]) == [True, True, True], out.stderr


def test_loader_base_only_maps_nothing_and_code_hash_ignores_docs(repo, tmp_path):
  m0 = L.build('HEAD')
  assert m0['mapped'] == {}
  (repo / 'docs/a.md').write_text('b\n')
  _git(repo, '-c', 'user.name=t', '-c', 'user.email=t@t', 'commit', '-qam', 'docs')
  m1 = L.build('HEAD')
  assert m1['code_hash'] == m0['code_hash'] and m1['tree'] != m0['tree']
  m2 = L.build('HEAD~1')   # older base, same code: nothing to map
  assert m2['mapped'] == {}


def test_loader_maps_working_tree_drift_to_the_base(repo, tmp_path):
  (repo / 'selfdrive/controls/lib/mod.py').write_text('X = 99  # uncommitted\n')
  m = L.build('HEAD')
  with open(m['mapped']['openpilot.selfdrive.controls.lib.mod']['file']) as fh:
    assert fh.read() == 'X = 1\n'


def test_loader_refuses_compiled_source_changes(repo, tmp_path):
  (repo / 'selfdrive/controls/lib/x.h').write_text('int a;\n')
  _git(repo, 'add', '.')
  _git(repo, '-c', 'user.name=t', '-c', 'user.email=t@t', 'commit', '-qm', 'h')
  (repo / 'selfdrive/controls/lib/x.h').write_text('int b;\n')
  with pytest.raises(RuntimeError, match='compiled sources'):
    L.build('HEAD')


GEN = 'selfdrive/controls/lib/longitudinal_mpc_lib/long_mpc.py'
GEN_SRC = """import numpy as np
from openpilot.selfdrive.modeld.constants import T
if __name__ == '__main__':
  from openpilot.third_party.acados.acados_template import AcadosOcp
COMFORT_BRAKE = 2.5
NODES = np.array([0.0, 1.0])
RUNTIME_ONLY = 1.0
def gen_ocp():
  Tf = NODES[-1]
  return COMFORT_BRAKE * T * Tf
def runtime_helper(x):
  return x * RUNTIME_ONLY
class LongitudinalMpc:
  def update(self, dt=RUNTIME_ONLY):
    return runtime_helper(dt) + COMFORT_BRAKE
if __name__ == '__main__':
  gen_ocp()
"""


@pytest.fixture
def gen_repo(repo):
  for p, txt in {GEN: GEN_SRC, 'selfdrive/modeld/__init__.py': '', 'selfdrive/modeld/constants.py': 'T = 1\n', 'selfdrive/controls/lib/__init__.py': '',
                 'third_party/acados/solver.in.c': 'int a;\n'}.items():
    (repo / p).parent.mkdir(parents=True, exist_ok=True)
    (repo / p).write_text(txt)
  _git(repo, 'add', '.')
  _git(repo, '-c', 'user.name=t', '-c', 'user.email=t@t', 'commit', '-qm', 'mpc')
  return repo


def _gen(old, new):
  return {GEN: GEN_SRC.replace(old, new)}


@pytest.mark.parametrize('edits, why', [
  (_gen('COMFORT_BRAKE = 2.5', 'COMFORT_BRAKE = 1.2'), 'COMFORT_BRAKE'),                         # a constant the OCP uses
  (_gen('* T *', '* T * 2 *'), 'def gen_ocp'),                                                   # the OCP function
  (_gen('class Long', '_init = NODES.__setitem__(-1, 2.0)\nclass Long'), '_init'),               # Astra P1: assignment mutation
  (_gen('class Long', '_n = NODES\n_n[-1] = 2.0\nclass Long'), '_n = NODES'),                    # Astra P1: alias mutation
  (_gen('def runtime_helper(x):', 'def runtime_helper(x, _i=NODES.__setitem__(-1, 2.0)):'), 'def runtime_helper'),   # P1: default
  (_gen('RUNTIME_ONLY = 1.0', 'RUNTIME_ONLY = 1.0\nRUNTIME_ONLY += 1'), 'RUNTIME_ONLY += 1'),    # not a plain literal assignment
  (_gen('import numpy as np\n', 'import numpy as np\nimport json\n'), 'import json'),            # a non-inert import
  (_gen('    return runtime_helper', '    globals()\n    return runtime_helper'), 'dynamic name access'),
  ({'selfdrive/modeld/constants.py': 'T = 2\n'}, 'selfdrive/modeld/constants.py'),              # read by the code generation
  ({'selfdrive/modeld/__init__.py': 'from . import constants\nconstants.T = 2\n'}, 'selfdrive/modeld/__init__.py'),   # P1: parent package
  ({'third_party/acados/solver.in.c': 'int b;\n'}, 'acados template'),
])
def test_loader_refuses_changes_to_generated_solver_code(gen_repo, tmp_path, monkeypatch, capsys, edits, why):
  """The sim always loads the working tree's compiled solver: a change that may enter its generated code must not run silently."""
  f = _diff(gen_repo, tmp_path, edits)
  with pytest.raises(ValueError, match='compiled acados solver') as e:
    L.build('HEAD', f)
  assert why in str(e.value) and 'scratch worktree' in str(e.value)
  monkeypatch.setattr(RG, 'SIM_HOME', tmp_path / 'sim')
  assert RG.main(['--diff', str(f), '--car', 'HEAD']) == 64
  assert 'compiled acados solver' in capsys.readouterr().err and not (tmp_path / 'sim' / 'runs').exists()


def test_loader_serves_runtime_only_generator_changes(gen_repo, tmp_path):
  """Runtime-only (a method or unreached def body, a literal only runtime code reads, a module Tf next to the generator's local Tf,
  an inert import, a comment) is served."""
  src = GEN_SRC.replace('RUNTIME_ONLY = 1.0', 'RUNTIME_ONLY = -2.0\nTf = 123.0\nNEW = (3.0, "x")').replace('x * RUNTIME_ONLY', 'x * NEW[0]')
  src = src.replace('import numpy as np\n', 'import numpy as np\nfrom openpilot.selfdrive.controls.lib import stopping_flags\n')
  src = src.replace('runtime_helper(dt) + COMFORT_BRAKE', 'runtime_helper(dt) * stopping_flags.OTHER').replace('def gen_ocp():', 'def gen_ocp():  # note')
  m = L.build('HEAD', _diff(gen_repo, tmp_path, {GEN: src}))
  with open(m['mapped']['openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc']['file']) as fh:
    assert fh.read() == src
  assert L.build('HEAD', _diff(gen_repo, tmp_path, {'selfdrive/controls/lib/mod.py': 'X = 2\n'}))['mapped']   # not read by codegen


def test_generator_inventory_of_the_real_scripts():
  """Pinned: the defs that importing the real generator scripts and their code generation reach (solver_reads; every other def or
  method body there is runtime-only; a new helper of the OCP must show up here) and their whole-file dependencies."""
  reached = {g: L.solver_reads(L.ast.parse((L.REPO / g).read_text()).body)[1] for g in L.SOLVER_GENERATORS}
  deps: list[str] = []
  L.solver_source_changes('HEAD', type('Seen', (list,), {'__contains__': lambda self, p: deps.append(p) or False})())
  assert {'cereal/__init__.py', 'common/constants.py', 'selfdrive/modeld/__init__.py', 'selfdrive/modeld/constants.py'} <= set(deps)
  long, lat = L.SOLVER_GENERATORS
  assert reached == {long: {'gen_long_ocp', 'gen_long_model', 'get_safe_obstacle_distance', 'get_T_FOLLOW'}, lat: {'gen_lat_ocp', 'gen_lat_model'}}
  for g, cls in L.SOLVER_GENERATORS.items():
    src = (L.REPO / g).read_text()
    assert f'class {cls}:' in src and L.generator_change(g, src, src, 'HEAD') is None   # no dynamic name access


def test_loader_refuses_a_base_whose_solver_code_differs_from_the_working_tree(gen_repo, tmp_path, monkeypatch, capsys):
  (gen_repo / GEN).write_text(GEN_SRC.replace('COMFORT_BRAKE = 2.5', 'COMFORT_BRAKE = 1.2'))   # the working tree's solver = 1.2
  with pytest.raises(ValueError, match='compiled acados solver'):
    L.build('HEAD')
  monkeypatch.setattr(RG, 'SIM_HOME', tmp_path / 'sim')
  assert RG.main(['--car', 'HEAD']) == 64
  assert '--base HEAD' in capsys.readouterr().err


@pytest.mark.skipif(sys.platform != 'darwin' or not shutil.which('clang'), reason='builds Mach-O libraries with clang')
def test_code_hash_includes_the_solver_libraries_and_what_they_load(gen_repo, tmp_path, monkeypatch):
  """Astra's linked-binary case: rebuilding only a library the solver links (@rpath) changes the key and fails an old manifest."""
  gen, lib = gen_repo / 'selfdrive/controls/lib/longitudinal_mpc_lib/c_generated_code', gen_repo / 'third_party/acados/Darwin/lib'
  gen.mkdir()
  lib.mkdir(parents=True)

  def build_dep(v):
    (tmp_path / 'dep.c').write_text(f'int dep_value(void) {{ return {v}; }}\n')
    subprocess.run(['clang', '-dynamiclib', str(tmp_path / 'dep.c'), '-o', str(lib / 'libdep.dylib'), '-Wl,-install_name,@rpath/libdep.dylib'], check=True)
  build_dep(1)
  (tmp_path / 'solver.c').write_text('extern int dep_value(void); int solver_value(void) { return dep_value(); }\n')
  subprocess.run(['clang', '-dynamiclib', str(tmp_path / 'solver.c'), '-o', str(gen / 'libacados_ocp_solver_long.dylib'), f'-L{lib}', '-ldep',
                  f'-Wl,-rpath,{lib}'], check=True)
  m0 = L.build('HEAD')
  assert set(m0['solver']) == {'selfdrive/controls/lib/longitudinal_mpc_lib/c_generated_code/libacados_ocp_solver_long.dylib',
                               'third_party/acados/Darwin/lib/libdep.dylib'}
  monkeypatch.setattr(L, 'MANIFEST', dict(m0, mapped={}))
  L.check()
  build_dep(2)   # rebuilt dependency: new arms, and a job on the old manifest fails instead of writing its rows
  assert L.build('HEAD')['code_hash'] != m0['code_hash']
  with pytest.raises(AssertionError, match='compiled solver or a library it loads changed'):
    L.check()


def test_finder_imports_the_snapshot(tmp_path):
  snap = tmp_path / 'snap.py'
  snap.write_text('VALUE = "snapshot"\n')
  man = tmp_path / 'm.json'
  man.write_text(json.dumps(dict(tree='t', mapped={'openpilot.sim_test_probe_mod': dict(file=str(snap), sha1='x')})))
  code = ('from openpilot.tools.stopping.sim import loader; loader.install_from_env(); '
          + 'import openpilot.sim_test_probe_mod as m; print(m.VALUE)')
  out = subprocess.run([sys.executable, '-c', code], capture_output=True, text=True, env=dict(os.environ, STOP_SIM_TREE=str(man)))
  assert out.stdout.strip() == 'snapshot', out.stderr


# ---- CLI input errors (clean message, status 64, nothing written) -------------------------------------------------------------
@pytest.mark.parametrize('argv, msg', [(['--diff', '/nonexistent/cand.diff'], 'no such file'), (['--on', 'LINE=yes'], 'bad flag assignment'),
                                       (['--on', 'LINE'], 'bad flag assignment'), (['--base', 'no_such_rev'], 'not a commit'),
                                       (['--on', 'NOPE=true', '--car', 'HEAD'], 'unknown flags'), (['--car', 'no_such_rev'], 'not a commit'),
                                       (['--stages', 'holds,bogus', '--car', 'HEAD'], '--stages')])
def test_cli_rejects_bad_input_cleanly(repo, tmp_path, monkeypatch, capsys, argv, msg):
  monkeypatch.setattr(RG, 'SIM_HOME', tmp_path / 'sim')
  assert RG.main(argv) == 64
  assert msg in capsys.readouterr().err
  assert not (tmp_path / 'sim' / 'runs').exists()


def test_cli_rejects_a_diff_that_does_not_apply(repo, tmp_path, monkeypatch, capsys):
  monkeypatch.setattr(RG, 'SIM_HOME', tmp_path / 'sim')
  bad = tmp_path / 'bad.diff'
  bad.write_text('--- a/selfdrive/controls/lib/mod.py\n+++ b/selfdrive/controls/lib/mod.py\n@@ -1 +1 @@\n-X = 7\n+X = 8\n')
  assert RG.main(['--diff', str(bad), '--car', 'HEAD']) == 64
  assert 'does not apply' in capsys.readouterr().err


def test_loader_refuses_a_diff_to_a_production_non_py_file(repo, tmp_path, monkeypatch, capsys):
  (repo / 'selfdrive/controls/lib/colors.json').write_text('{"a": 1}\n')
  _git(repo, 'add', '.')
  _git(repo, '-c', 'user.name=t', '-c', 'user.email=t@t', 'commit', '-qm', 'json')
  f = _diff(repo, tmp_path, {'selfdrive/controls/lib/colors.json': '{"a": 2}\n', 'docs/a.md': 'b\n'})
  with pytest.raises(ValueError, match='cannot serve'):
    L.build('HEAD', f)
  monkeypatch.setattr(RG, 'SIM_HOME', tmp_path / 'sim')
  assert RG.main(['--diff', str(f), '--car', 'HEAD']) == 64
  assert 'colors.json' in capsys.readouterr().err
  assert L.build('HEAD', _diff(repo, tmp_path, {'docs/a.md': 'c\n'}))['mapped'] == {}   # docs outside the production roots: fine


def test_unappliable_diff_error_is_one_line(repo, tmp_path, monkeypatch, capsys):
  monkeypatch.setattr(RG, 'SIM_HOME', tmp_path / 'sim')
  bad = tmp_path / 'bad.diff'
  bad.write_text('--- a/selfdrive/controls/lib/mod.py\n+++ b/selfdrive/controls/lib/mod.py\n@@ -1 +1 @@\n-X = 7\n+X = 8\n')
  assert RG.main(['--diff', str(bad), '--car', 'HEAD']) == 64
  err = capsys.readouterr().err
  assert err.count('\n') == 1 and 'does not apply' in err


def test_car_check_against_the_candidate_tree(repo, tmp_path):
  base = _git(repo, 'rev-parse', 'HEAD')
  f = _diff(repo, tmp_path, {'selfdrive/controls/lib/mod.py': 'X = 2\n', 'selfdrive/controls/lib/stopping_flags.py': FLAGS_SRC + 'NEW = True\n'})
  m = L.build('HEAD', f)
  _git(repo, 'apply', str(f))
  _git(repo, '-c', 'user.name=t', '-c', 'user.email=t@t', 'commit', '-qam', 'trial')   # the car = base + diff (a deployed trial)
  c = RG.car_check('HEAD', base, m['tree'])
  assert not c['same_code'] and c['same_as_candidate']
  (repo / 'selfdrive/controls/lib/stopping_flags.py').write_text(FLAGS_SRC + 'NEW = False\n')
  _git(repo, '-c', 'user.name=t', '-c', 'user.email=t@t', 'commit', '-qam', 'revert flag')   # the trial reverted by its flag
  arm_flags = {k: L.arm_manifest(m, {'NEW': val})['flag_values'] for k, val in (('ON', True), ('OFF', False))}
  c = RG.car_check('HEAD', base, m['tree'], arm_flags)
  assert not c['same_as_candidate'] and c['cand_differ'] == [L.FLAGS_FILE] and c['car_arm'] == ['OFF']


def test_one_sweep_per_machine_whatever_the_sim_home(repo, tmp_path, monkeypatch, capsys):
  import fcntl
  monkeypatch.setattr(RG, 'SIM_HOME', tmp_path / 'scratch_home')
  monkeypatch.setattr(RG, 'SWEEP_LOCK', tmp_path / 'sweep.lock')
  with open(tmp_path / 'sweep.lock', 'w') as held:
    fcntl.flock(held, fcntl.LOCK_EX | fcntl.LOCK_NB)   # the main sweep
    assert RG.main(['--car', 'HEAD']) == 64
    assert RG.main(['--prune']) == 64
  assert 'another sim sweep' in capsys.readouterr().err


def test_prune_deletes_only_the_arms_of_other_engines(tmp_path, monkeypatch):
  monkeypatch.setattr(RG, 'SIM_HOME', tmp_path)
  monkeypatch.setattr(RG, 'SWEEP_LOCK', tmp_path / 'sweep.lock')
  monkeypatch.setattr(RG, 'engine_sha', lambda: ('closed1', 'replay1', {}, 'judge1'))
  for key, eng in (('a', 'closed0'), ('b', 'closed1'), ('c', 'replay1'), ('d', 'replay0'), ('f', 'judge1'), ('g', 'judge0')):
    (tmp_path / 'arms' / key).mkdir(parents=True)
    (tmp_path / 'arms' / key / 'arm.json').write_text(json.dumps(dict(engine=eng)))
    (tmp_path / 'arms' / key / 'closed.pkl').write_bytes(b'x' * 10)
  (tmp_path / 'arms' / 'e').mkdir()   # no arm.json: kept
  assert RG.main(['--prune']) == 0
  assert sorted(p.name for p in (tmp_path / 'arms').iterdir()) == ['b', 'c', 'e', 'f']


def test_car_check(repo):
  base = _git(repo, 'rev-parse', 'HEAD')
  assert RG.car_check('HEAD', base)['same_code']
  (repo / 'selfdrive/controls/lib/mod.py').write_text('X = 5\n')
  (repo / 'docs/a.md').write_text('c\n')
  _git(repo, '-c', 'user.name=t', '-c', 'user.email=t@t', 'commit', '-qam', 'car')
  c = RG.car_check('HEAD', base)
  assert not c['same_code'] and c['differ'] == ['selfdrive/controls/lib/mod.py']


# ---- gate rules on small traces ----------------------------------------------------------------------------------------------
def _x(n=400, dt=0.01, **cols):
  t = np.arange(n) * dt
  x = dict(t=t, v_true=np.zeros(n), wire=np.full(n, -0.5), stopreq=np.ones(n, dtype=np.int8), phase=np.full(n, 4, dtype=np.int8),
           lcs=np.full(n, 2, dtype=np.int8), gap=np.full(n, 4.5), sent=np.zeros(n), gas=np.zeros(n), brake=np.zeros(n), active=np.ones(n))
  x.update(cols)
  return x


def test_chatter_is_a_clear_then_set_at_rest_within_one_second():
  x = _x()
  assert G.chatter(x, 0, 400) == []
  x['stopreq'][100:150] = 0     # clear at 1.0, set again at 1.5 at rest (set-clear-set / clear-set): chatter
  assert G.chatter(x, 0, 400) == [(1.0, 1.5)]
  x['v_true'][150] = 0.2    # moving at the second transition: not chatter
  assert G.chatter(x, 0, 400) == []
  y = _x(stopreq=np.zeros(400, dtype=np.int8))
  y['stopreq'][100:150] = 1     # set at 1.0, the normal launch clear 0.5 s later: one StopReq episode (PLAN 82), not chatter
  assert G.chatter(y, 0, 400) == []


def test_stopreq_information_lists_a_set_in_release_and_a_re_set_after_the_window():
  x = _x(stopreq=np.zeros(400, dtype=np.int8))
  x['phase'][:] = G.RELEASE
  x['stopreq'][100:] = 1
  assert G.stopreq_info(x, 0, 400) == [('StopReq set in RELEASE', 1.0)]
  y = _x()
  y['stopreq'][195:220] = 0     # cleared at 1.95, set again at 2.2: after a window that ends at 2.0
  assert G.stopreq_info(y, 0, 200) == [('StopReq clear -> set at rest just after the window', (1.95, 2.2))]
  assert G.chatter(y, 0, 200) == []


def test_pid_hold_and_starting_under_hold():
  x = _x()
  assert G.pid_holds(x, 0, 400) == [] and G.start_under_hold(x, 0, 400) == 0
  x['lcs'][50:80] = G.PID
  assert len(G.pid_holds(x, 0, 400)) == 1
  x['lcs'][50:80] = G.STARTING
  assert G.start_under_hold(x, 0, 400) == 0     # a non-owning hold label over a flat command: not a conflict (PLAN 82)
  x['wire'][50:80] = np.linspace(-0.5, 0.2, 30)
  assert G.start_under_hold(x, 0, 400) == 0     # ... nor over the rising starting ramp
  x['wire'][70:80] = -0.6
  assert G.start_under_hold(x, 0, 400) == 1     # the ramp reverses (a re-grab) at frame 70
  x['owning'] = np.where((np.arange(400) >= 50) & (np.arange(400) < 80), 1, 0)
  assert G.start_under_hold(x, 0, 400) == 30    # the service owns every 'starting' frame


def _launch(lead_v=None, rest_frames=100, derive=False):
  """Rest from frame 0 (or a 0.28 s touch at frame 100), then 0.5 m/s from frame 200; the lead stands still or crawls at lead_v."""
  n = 400
  t = np.arange(n) * 0.01
  v = np.zeros(n)
  v[200:300] = 0.5
  if rest_frames < 100:   # a brief touch: rolling at 0.5 m/s, at rest for rest_frames, then moving again
    v[:200 - rest_frames] = 0.5
  xx = np.r_[0.0, np.cumsum(0.5 * (v[1:] + v[:-1]) * 0.01)]
  vl = 0.0 if lead_v is None else lead_v
  x = _x(v_true=v, x=xx, gap=6.0 + vl * t - xx)
  if not derive:
    x['vl_true'] = np.full(n, vl)
  return x


@pytest.mark.parametrize('lead_v, derive, false', [(None, False, True), (None, True, True), (0.6, False, False), (0.6, True, False),
                                                   (0.1, False, True)])
def test_false_launch_needs_a_stopped_lead_through_the_launch(lead_v, derive, false):
  la = G.launches(_launch(lead_v, derive=derive), 0, 400)
  assert len(la) == 1 and la[0][3] is false   # vl_true, or the lead position (ego x + gap) where vl_true is missing


def test_a_brief_rest_behind_a_crawler_is_not_a_false_launch():
  """sv_000020c0_1449.83 (tooling_check2 fix 2): a 0.28 s touch-and-go behind a 0.6 m/s crawler; the lead travels only 0.16 m during
  that rest, but it is moving through the launch."""
  la = G.launches(_launch(0.6, rest_frames=28), 0, 400)
  assert len(la) == 1 and la[0][2] == 0.6 and la[0][3] is False


def test_h6_service_release_bound_is_per_50_ms_on_owned_frames():
  h, v = _x(), _x()
  v['wire'][100:] = np.minimum(-0.5 + 0.05 * np.arange(300), 0.0)   # +0.05 per 10 ms = 5 m/s^3
  assert G.owned_release_breaches(h, v, 0, 400) == []   # nobody owns it (e.g. the MPC's own release): not the change's release
  v['owning'] = np.ones(400)
  out = G.owned_release_breaches(h, v, 0, 400)
  assert out and all(kind == 'service' for _, _, kind in out) and out[0][1] == pytest.approx(0.15)   # 3 owned rises in the window
  assert G.owned_release_breaches(h, h, 0, 400) == []   # the same command as HEAD
  for step, n_out in ((0.12, 0), (0.14, 1)):   # one step: 0.125 per 50 ms planner tick (PLAN 82)
    w = _x(owning=np.ones(400))
    w['wire'][200:] = -0.5 + step
    assert len(G.owned_release_breaches(h, w, 0, 400)) == n_out * 5, step   # the step stays in the 50 ms window for 5 frames
  r = _x(owning=np.ones(400))
  r['wire'][100:] = np.minimum(-0.5 + 0.02 * np.arange(300), 0.0)   # 2 m/s^3: within J_MAX
  assert G.owned_release_breaches(h, r, 0, 400) == []
  v['lcs'][:] = G.STARTING   # LongControl's starting step is exempt
  assert G.owned_release_breaches(h, v, 0, 400) == []
  pre = _x(owning=np.where(np.arange(400) >= 201, 1, 0))
  pre['wire'][200:] = -0.3   # a 0.2 planner step one frame before the service entry: not the service's
  assert G.owned_release_breaches(h, pre, 0, 400) == []


def test_h6_owned_release_no_faster_than_heads_own_is_not_the_changes():
  """PLAN 86 (check_r3 item 3, sv_2049_462.37): the service owns the entry and rises 0.127 in 50 ms where HEAD's own (planner)
  command rises 0.154 in the same window."""
  h = _x()
  h['wire'][200:] = -0.5 + 0.154
  v = _x(owning=np.ones(400))
  v['wire'][200:] = -0.5 + 0.140
  v['wire'][300:] = -0.2      # keeps v different from h after the step
  assert [r for r in G.owned_release_breaches(h, v, 0, 400) if r[0] < 2.5] == []
  v['wire'][200:300] = -0.5 + 0.30   # faster than HEAD's: counts
  assert G.owned_release_breaches(h, v, 0, 400)[0][:2] == (2.0, 0.3)


def _plan(at, lf):
  x = _x()
  x['a_target'] = np.repeat(np.asarray(at, dtype=float), 5)[:400]   # held per 50 ms planner tick
  x['line_floor'] = np.repeat(np.asarray(lf, dtype=float), 5)[:400]
  return x


def test_h6_line_release_needs_the_floor_binding_on_the_previous_tick():
  h = _plan(np.full(80, -0.3), np.full(80, np.nan))
  end = _plan([-1.0] * 40 + [-0.5] * 40, [-1.0] * 40 + [np.nan] * 40)   # the floor ends while binding: a 0.5 step
  assert G.owned_release_breaches(h, end, 0, 400) == [(2.0, 0.5, 'line')]
  ramp = _plan([-1.0] * 40 + [min(-1.0 + 0.1 * i, -0.5) for i in range(40)], [-1.0] * 40 + [-1.0 + 0.1 * i for i in range(40)])
  assert G.owned_release_breaches(h, ramp, 0, 400) == []   # the floor ramps out at 2 m/s^3 (0.1 per tick)
  capped = _plan([-0.8, -0.6] * 40, [-0.6] * 80)   # the MPC's own release to the floor (the line caps it): not the line's
  assert G.owned_release_breaches(h, capped, 0, 400) == []


def test_plan_release_bound_on_the_braking_component():
  n = 40
  ns = (100.0 + np.arange(n) * 0.05) * 1e9
  ref = dict(plan=dict(ns=ns, at=np.full(n, -1.0), ss=np.zeros(n)))
  at = np.full(n, -2.0)
  at[20:] = np.minimum(-2.0 + 0.1 * np.arange(n - 20), -1.0)     # 0.1 per tick = 2 m/s^3: J-limited
  ok = dict(plan=dict(ns=ns, at=at, ss=np.zeros(n)))
  jump = dict(plan=dict(ns=ns, at=np.where(np.arange(n) >= 20, -1.0, -2.0), ss=np.zeros(n)))   # 1.0 in one tick
  pos = dict(plan=dict(ns=ns, at=np.where(np.arange(n) >= 20, 0.5, -0.05), ss=np.zeros(n)))   # hand-back past zero: braking part 0.05

  def frames(gas=False):
    fr = [dict(t=100.0 + k * 0.01, active=True, cs=dict(gasPressed=gas, brakePressed=False)) for k in range(n * 5)]
    return dict(frames=fr, origin=0.0)
  assert G.plan_release_breaches(ref, ok, frames) == []
  assert G.plan_release_breaches(ref, jump, frames) == [(101.0, 1.0)]
  assert G.plan_release_breaches(ref, jump, lambda: frames(gas=True)) == []   # the driver's pedal: exempt
  assert G.plan_release_breaches(ref, pos, frames) == []


def test_comfort_ten_percent_rule():
  base = dict(head=10, cand=10, worse_frac=0.0, better=False, worse=False)
  agg = {k: dict(base) for k in ('a', 'b', 'c')}
  assert G.comfort_verdict(agg) == 'SAME'
  agg['a'] = dict(head=10, cand=8, worse_frac=-0.2, better=True, worse=False)
  assert G.comfort_verdict(agg) == 'BETTER'
  agg['b'] = dict(head=10, cand=11.5, worse_frac=0.15, better=False, worse=True)
  assert G.comfort_verdict(agg).startswith('WORSE')
  agg['b'] = dict(head=10, cand=10.5, worse_frac=0.05, better=False, worse=True)
  assert G.comfort_verdict(agg) == 'NOT BETTER'   # one better, one worse (within 10 %)
  agg['b'] = dict(head=1, cand=2, worse_frac=1.0, better=False, worse=True, worse_counts=1.0)
  assert G.comfort_verdict(agg) == 'NOT BETTER'   # 100 % worse but 1 count: below the small-count floor (PLAN 82)
  agg['b'] = dict(head=10, cand=12, worse_frac=0.2, better=False, worse=True, worse_counts=2.0)
  assert G.comfort_verdict(agg).startswith('WORSE')
  agg['b'] = dict(head=1.5, cand=1.8, worse_frac=0.2, better=False, worse=True, worse_counts=None)
  assert G.comfort_verdict(agg).startswith('WORSE')   # the j300 median has no count floor


def test_comfort_counts_worse():
  pairs = [(_row(f'c{i}', 4.5), _row(f'c{i}', 4.5 if i else 3.8, 'b')) for i in range(4)]
  agg = G.comfort(pairs)
  assert agg['rest_4_5_share']['worse_counts'] == 1.0 and agg['j300_median']['worse_counts'] is None


def _row(case, gap, sha='a', cell='L42', mode='drv', start='v12', group='new', v0=5.0, t_go=None, gap_go=None, x_shift=0.0, stops=True):
  """A drv row: decelerates from v0 to rest at t = v0 (1 m/s^2), optionally moves off at t_go (0.5 m/s) with the gap gap_go then."""
  t = np.arange(0, 12, 0.02)
  v = np.maximum(v0 - t, 0.0) if stops else np.full(len(t), 1.0)
  g = np.full(len(t), gap)
  if t_go is not None:
    v = np.where(t >= t_go, 0.5, v)
    g = np.where(t >= t_go, gap_go, g)
  x = np.r_[0.0, np.cumsum(0.5 * (v[1:] + v[:-1]) * np.diff(t))] + x_shift
  tr = dict(t=t, v_true=v, wire=np.full(len(t), -0.5), a_real=np.full(len(t), -0.5), stopreq=np.zeros(len(t)), phase_i=np.zeros(len(t)),
            lcs=np.full(len(t), 1), gap=g, closed=np.ones(len(t)), gas=np.zeros(len(t)), brake=np.zeros(len(t)), active=np.ones(len(t)),
            lead_status=np.ones(len(t)), x=x)
  m = dict(t_stop=v0 if stops else None, t_lo=0.0, min_gap=gap, rest_at_stop=gap if stops else None)
  return dict(case=case, cell=cell, start=start, mode=mode, group=group, meta={}, trace_sha=sha, err=None, m=m, x={}, f={}, tr=tr)


def _evaluate(tmp_path, rows, **kw):
  arms = {k: tmp_path / k for k in ('HEAD', 'ON', 'OFF')}
  for k, d in arms.items():
    d.mkdir(exist_ok=True)
    _dump(d / 'closed.pkl', rows[k])
  meta = dict(run='t', base='b', base_sha='0' * 40, flags_on={}, flags_off={}, engine='e', engine_replay='r', spec='S',
              arms={k: dict(key=k, code_hash=k, tree='0' * 40, overrides={}, flags_sha1=k) for k in arms})
  res = G.evaluate(meta, arms, arms, **kw)
  return res, {x['gate']: x for x in res['gates']}


def test_evaluate_flags_a_new_close_rest_and_lists_cut_ins(tmp_path):
  rows = {'HEAD': [_row('2232_s1', 4.0), _row('cut_v8', 3.5)], 'ON': [_row('2232_s1', 2.8, 'b'), _row('cut_v8', 2.5, 'b')],
          'OFF': [_row('2232_s1', 4.0), _row('cut_v8', 3.5)]}
  res, g = _evaluate(tmp_path, rows)
  assert g['H2 no new rest / min gap < 3.0 m']['verdict'] == 'FAIL' and res['verdict'] == 'FAIL'
  assert {f['case'] for f in g['H2 no new rest / min gap < 3.0 m']['fails']} == {'2232_s1'}
  assert {e['case'] for e in g['H2 no new rest / min gap < 3.0 m']['evidence']} == {'cut_v8'}
  assert g['H1b closed OFF == HEAD']['verdict'] == 'PASS'
  assert 'H2' in G.render(res)


@pytest.mark.parametrize('cand, fails', [
  (dict(t_go=None), ['candidate never moves (HEAD does)']),                   # HEAD moves, the candidate stays
  (dict(t_go=8.0, gap_go=5.0), ['gap growth since the rest > 0.3 m more than HEAD']),   # same time, the lead got 0.4 m further
  (dict(t_go=9.0, gap_go=4.7), []),                                            # 1 s later, growth +0.1, gap +0.2: passes (listed)
  (dict(t_go=7.0, gap_go=4.4), []),                                            # earlier
  (dict(gap=5.0, t_go=8.0, gap_go=5.0), []),                                   # rests 0.5 m further back, same launch: passes (listed)
  (dict(gap=4.9, t_go=8.5, gap_go=5.1), ['first motion > 0.3 s later at a gap > 0.3 m larger']),   # growth +0.2 only
])
def test_h5_gap_growth_and_later_motion(tmp_path, cand, fails):
  h = _row('2234_s1', 4.5, t_go=8.0, gap_go=4.5)
  v = _row('2234_s1', cand.pop('gap', 4.6), 'b', **cand)
  out, info = G.h5_gap(h, v)
  assert [f['check'] for f in out] == fails and info['judged']
  res, g = _evaluate(tmp_path, {'HEAD': [h], 'ON': [v], 'OFF': [h]})
  assert g['H5 launches']['verdict'] == ('FAIL' if fails else 'PASS') and g['H5 launches']['coverage'] == 1
  if cand.get('t_go') == 9.0:
    assert any('later' in e['note'] for e in g['H5 launches']['evidence'])   # the clock delta is listed as information
  if cand.get('gap_go') == 5.0 and not fails:
    assert any('rests further back' in e['note'] for e in g['H5 launches']['evidence'])


def test_h5_counts_only_judged_pairs(tmp_path):
  """Changed pairs with no motion in the window are listed as not judged and are not coverage (tooling_check2 fix 1)."""
  h, v = _row('2234_s1', 4.5), _row('2234_s1', 4.6, 'b')
  out, info = G.h5_gap(h, v)
  assert out == [] and not info['judged'] and info['rest'] == 'no motion in the window'
  res, g = _evaluate(tmp_path, {'HEAD': [h], 'ON': [v], 'OFF': [h]})
  assert g['H5 launches']['coverage'] == 0 and g['H5 launches']['verdict'] == 'NO COVERAGE'
  assert [e['note'] for e in g['H5 launches']['evidence']] == ['not judged: no motion in the window']


def _nodrv(case, gaps, band=True, sha='a', cell='L42'):
  """A nodrv row: at rest for 2 s, a launch to 1 m/s, then a follow on a braking command; the gap goes linearly through gaps."""
  n = 1000
  t = np.arange(n) * 0.01
  v = np.clip((t - 2.0), 0.0, 1.0)
  x = _x(n=n, v_true=v, x=np.r_[0.0, np.cumsum(0.5 * (v[1:] + v[:-1]) * 0.01)], gap=np.interp(t, np.linspace(0, 9.99, len(gaps)), gaps),
         wire=np.where(t > 3.0, -0.3, 0.5), plant_off=np.full(n, int(band), dtype=np.int8), gear=np.ones(n, dtype=np.int8))
  return dict(case=case, cell=cell, start='hold', mode='nodrv', group='holds', meta=dict(k0=0), trace_sha=sha, err=None, m={}, x={}, f={}, w=x)


def test_h2_min_gap_ignores_a_one_frame_radar_glitch(tmp_path):
  h = _nodrv('sv_1', [5.0, 4.0], band=False)
  v = _nodrv('sv_1', [5.0, 4.0], band=False, sha='b')
  v['w']['gap'][500:505] = 2.5      # one 20 Hz radar frame, held 5 samples at 100 Hz (check_r3: a 3-sample median kept it)
  _, g = _evaluate(tmp_path, {'HEAD': [h], 'ON': [v], 'OFF': [h]})
  assert g['H2 no new rest / min gap < 3.0 m']['verdict'] == 'PASS'
  v['w']['gap'][500:530] = 2.5      # 0.3 s: a real minimum
  _, g = _evaluate(tmp_path, {'HEAD': [h], 'ON': [v], 'OFF': [h]})
  assert g['H2 no new rest / min gap < 3.0 m']['verdict'] == 'FAIL'


def test_despike_removes_one_radar_frame_at_50_hz_too():
  t = np.arange(300) * 0.02
  g = np.full(300, 4.0)
  g[100:103] = 2.5                  # one radar frame on a 50 Hz drv trace (2-3 samples)
  assert G.despike(g, t).min() == 4.0
  g[100:115] = 2.5                  # 0.3 s
  assert G.despike(g, t).min() == 2.5


def test_starting_under_hold_fails_on_more_frames_than_head(tmp_path):
  h = _nodrv('sv_1', [5.0, 4.5], band=False)
  v = _nodrv('sv_1', [5.0, 4.5], band=False, sha='b')
  for r, n in ((h, 5), (v, 10)):
    r['w']['lcs'][100:100 + n] = G.STARTING
    r['w']['owning'] = np.where((np.arange(1000) >= 100) & (np.arange(1000) < 100 + n), 1, 0)
  _, g = _evaluate(tmp_path, {'HEAD': [h], 'ON': [v], 'OFF': [h]})
  assert g['H4 StopReq / ownership']['verdict'] == 'FAIL'   # HEAD has some, the candidate more (check_r3 item 5)
  v['w']['lcs'][105:110] = 2
  _, g = _evaluate(tmp_path, {'HEAD': [h], 'ON': [v], 'OFF': [h]})
  assert g['H4 StopReq / ownership']['verdict'] == 'PASS'


def test_h5_is_void_where_heads_own_first_motion_is_a_false_launch():
  """PLAN 86 (205d / 20b7 class): HEAD releases on standstill drift toward a lead that is still stopped; the candidate holds and
  launches when the lead leaves. HEAD's launch is no reference; the candidate's own launches are judged by the false-launch rule."""
  h = _row('205d_1', 4.5, t_go=8.0, gap_go=4.5)
  tr = h['tr']
  go = tr['t'] >= 8.0
  tr['gap'] = np.where(go, 4.5 - (tr['x'] - np.interp(8.0, tr['t'], tr['x'])), tr['gap'])   # the lead stands: the gap closes
  v = _row('205d_1', 4.6, 'b', t_go=9.0, gap_go=5.5)
  out, info = G.h5_gap(h, v)
  assert out == [] and not info['judged'] and 'false launch' in info['rest']
  h2 = _row('205d_1', 4.5, t_go=8.0, gap_go=4.5)   # HEAD launches behind a moving lead: judged as before
  out, info = G.h5_gap(h2, v)
  assert info['judged'] and out


def test_h2_drv_min_gap_outside_the_stored_trace_uses_the_harness_metric(tmp_path):
  h = _row('2232_s1', 4.0)
  v = _row('2232_s1', 4.0, 'b')
  v['m']['min_gap'] = 2.5   # the harness saw 2.5 m after the stored trace ends (no driver input inside it): counts
  _, g = _evaluate(tmp_path, {'HEAD': [h], 'ON': [v], 'OFF': [h]})
  assert g['H2 no new rest / min gap < 3.0 m']['verdict'] == 'FAIL'
  v['tr']['gas'] = np.where(np.arange(len(v['tr']['t'])) >= 550, 1.0, 0.0)   # the trace covers the harness window: not in it = 1 frame
  _, g = _evaluate(tmp_path, {'HEAD': [h], 'ON': [v], 'OFF': [h]})
  assert g['H2 no new rest / min gap < 3.0 m']['verdict'] == 'PASS'


@pytest.mark.parametrize('confirm, verdict', [(None, 'INCOMPLETE'), ({'L42P': (3.5, 3.5), 'H': (4.5, 4.4)}, 'PASS'),
                                              ({'L42P': (3.5, 2.8), 'H': (4.5, 4.4)}, 'FAIL')])
def test_h2_brake_off_band_failure_needs_l42p_or_h(tmp_path, confirm, verdict):
  """PLAN 82: a new < 3.0 m minimum in the 1st-gear brake-off band after a launch (L42) counts only if L42P or H agrees."""
  rows = {'HEAD': [_nodrv('sv_2072', [6.0, 3.1])], 'ON': [_nodrv('sv_2072', [6.0, 2.1], sha='b')], 'OFF': [_nodrv('sv_2072', [6.0, 3.1])]}
  ca = None
  if confirm:
    ca = {k: tmp_path / 'confirm' / k for k in ('HEAD', 'ON')}
    for i, k in enumerate(('HEAD', 'ON')):
      ca[k].mkdir(parents=True)
      _dump(ca[k] / 'closed.pkl', [_nodrv('sv_2072', [6.0, end[i]], sha=k, cell=c) for c, end in confirm.items()])
  res, g = _evaluate(tmp_path, rows, confirm_arms=ca)
  h2 = g['H2 no new rest / min gap < 3.0 m']
  assert res['verdict'] == verdict or (verdict == 'PASS' and h2['verdict'] == 'PASS')
  if verdict == 'INCOMPLETE':
    assert res['missing'][-1]['kind'] == 'confirm' and len(res['missing'][-1]['keys']) == 2
  elif verdict == 'PASS':
    assert h2['n_fail'] == 0 and any('not confirmed' in e.get('note', '') for e in h2['evidence'])
  else:
    assert h2['fails'][0]['confirmed_by'] == ['L42P']
  rows['ON'] = [_nodrv('sv_2072', [6.0, 2.1], band=False, sha='b')]   # outside the band: counts without confirmation
  _, g = _evaluate(tmp_path, rows, confirm_arms=ca)
  assert g['H2 no new rest / min gap < 3.0 m']['verdict'] == 'FAIL'


def test_h3_model_only_stop_must_stop_near_head(tmp_path):
  h = _row('ms_v8_none', 50.0, group='ms')
  cases = {'stops': (_row('ms_v8_none', 50.0, 'b', group='ms', x_shift=0.3), 'PASS'),
           'past': (_row('ms_v8_none', 50.0, 'b', group='ms', x_shift=0.8), 'FAIL'),
           'never': (_row('ms_v8_none', 50.0, 'b', group='ms', stops=False), 'FAIL')}
  for name, (v, verdict) in cases.items():
    d = tmp_path / name
    d.mkdir()
    _, g = _evaluate(d, {'HEAD': [h], 'ON': [v], 'OFF': [h]})
    assert (g['H3 model-only stops honoured']['verdict'], g['H3 model-only stops honoured']['coverage']) == (verdict, 1), name


def test_rows_outside_the_job_set_are_ignored_and_missing_jobs_make_the_run_incomplete(tmp_path):
  key = ('2232_s1', 'L42', 'v12', 'drv')
  stale = _row('2232_s1', 2.0, 'z', cell='L42s')   # a cached row from another run's job set (e.g. --with-h used to mix them in)
  rows = {'HEAD': [_row('2232_s1', 4.0), _row('2232_s1', 4.0, cell='L42s')], 'ON': [_row('2232_s1', 4.0), stale], 'OFF': [_row('2232_s1', 4.0)]}
  expected = {('closed', k): {key} for k in ('HEAD', 'ON', 'OFF')}
  res, g = _evaluate(tmp_path, rows, expected=expected)
  assert g['H2 no new rest / min gap < 3.0 m']['verdict'] == 'PASS' and g['H2 no new rest / min gap < 3.0 m']['coverage'] == 1
  assert res['verdict'] == 'NO COVERAGE (H1a, H3, H5)'   # no replay rows, no model-only stop, no launch: not a PASS
  rows['ON'] = [stale]   # the ON job of this run failed
  arms = {k: tmp_path / k for k in ('HEAD', 'ON', 'OFF')}
  _dump(arms['ON'] / 'closed.pkl', rows['ON'])
  (arms['ON'] / 'errors_jobs_closed_x.json').write_text(json.dumps([dict(key=list(key), err='Traceback\nAssertionError: snapshot file changed')]))
  missing = G.missing_jobs(arms, arms, {}, expected)
  assert missing == [dict(kind='closed', arm='ON', keys=[list(key)], errors={'2232_s1|L42|v12|drv': 'AssertionError: snapshot file changed'})]
  res, _ = _evaluate(tmp_path, rows, expected=expected, missing=missing)
  assert res['verdict'] == 'INCOMPLETE' and 'INCOMPLETE' in G.render(res)


def test_quick_prints_no_verdict(tmp_path):
  rows = {'HEAD': [_row('s20', 4.0)], 'ON': [_row('s20', 2.5, 'b')], 'OFF': [_row('s20', 4.0)]}
  res, g = _evaluate(tmp_path, rows, quick=True)
  assert res['verdict'].startswith('NO VERDICT') and {x['verdict'] for x in res['gates']} == {'INFO'}
  out = G.render(res, short=True)
  assert '| PASS |' not in out and '| FAIL |' not in out and 'NO VERDICT' in out


def test_cell_h_rows_stay_out_of_the_gates(tmp_path):
  rows = {'HEAD': [_row('2232_s1', 4.0)], 'ON': [_row('2232_s1', 4.0)], 'OFF': [_row('2232_s1', 4.0)]}
  hd = {k: tmp_path / 'h' / k for k in ('HEAD', 'ON')}
  for k, d in hd.items():
    d.mkdir(parents=True)
    _dump(d / 'closed.pkl', [_row('2232_s1', 4.0 if k == 'HEAD' else 2.0, k, cell='H')])
  res, g = _evaluate(tmp_path, rows, h_arms=hd)
  assert 'FAIL' not in {x['verdict'] for x in res['gates']} and res['cell_h']['pairs'] == 1 and res['cell_h']['changed'] == 1


def test_comfort_without_j300_has_no_warning():
  h, v = _row('2232_s1', 4.5), _row('2232_s1', 4.5, 'b')
  agg = G.comfort([(h, v)])   # no j300 at all: the repo's -Werror would turn an all-NaN median warning into a failure
  assert agg['j300_median']['worse_frac'] is None and not agg['j300_median']['worse']


def test_h1c_skips_the_replay_warm_up(tmp_path, monkeypatch):
  monkeypatch.setattr(G, 'same_code', lambda commit, tree: True)
  n = 300
  rec = np.full(n, -0.5)
  wire = rec.copy()
  wire[:50] = -2.0          # warm-up error inside the first second only
  span = dict(_span(n, wire=wire), rec=rec, commit='c', route='00002232')
  rows = {'HEAD': [_row('s20', 4.0)], 'ON': [_row('s20', 4.0)], 'OFF': [_row('s20', 4.0)]}
  for k in rows:
    (tmp_path / k).mkdir()
    _dump(tmp_path / k / 'replay_c1.pkl', [dict(kind='replay', corpus='c1', span='2232_0000', err=None, res=span)])
  _, g = _evaluate(tmp_path, rows)
  h1c = g['H1c HEAD replay vs logged command (same-code drives) + car check']
  assert h1c['coverage'] == 1 and h1c['evidence'] == [] and 'worst 0.000' in h1c['note']


def test_h1c_without_same_code_spans_reports_zero_coverage(tmp_path, monkeypatch):
  """No replay span on a drive with the HEAD controls code (e.g. c1/c2 only hold older builds): H1c has no samples and must not crash
  on the empty summary."""
  monkeypatch.setattr(G, 'same_code', lambda commit, tree: False)
  span = dict(_span(300), commit='c', route='00002232')
  rows = {'HEAD': [_row('s20', 4.0)], 'ON': [_row('s20', 4.0)], 'OFF': [_row('s20', 4.0)]}
  for k in rows:
    (tmp_path / k).mkdir()
    _dump(tmp_path / k / 'replay_c1.pkl', [dict(kind='replay', corpus='c1', span='2232_0000', err=None, res=span)])
  _, g = _evaluate(tmp_path, rows)
  h1c = g['H1c HEAD replay vs logged command (same-code drives) + car check']
  assert h1c['coverage'] == 0 and h1c['verdict'] == 'INFO' and 'median' not in h1c['note']


def _span(n=400, wire=None, owning=None, at=None):
  """A replay row (100 Hz frames from route time 100 s) with a 20 Hz planner lockstep over its frames."""
  ns = (100.0 + np.arange(n // 5) * 0.05) * 1e9
  return dict(t=np.arange(n) * 0.01, origin=100.0, wire=np.full(n, -0.5) if wire is None else wire, sent=np.zeros(n), rec=np.full(n, -0.5),
              stopreq=np.zeros(n, dtype=np.int8), phase=np.zeros(n, dtype=np.int8), lcs=np.full(n, 2, dtype=np.int8), active=np.ones(n, dtype=np.int8),
              owning=np.zeros(n, dtype=np.int8) if owning is None else owning, act=np.ones(n, dtype=np.int8), v=np.zeros(n), route='00009999', commit=None,
              plan=dict(ns=ns, at=np.full(len(ns), -0.5) if at is None else at, ss=np.zeros(len(ns))))


def _replay_eval(tmp_path, spans, **kw):
  """evaluate() on identical safe closed rows and the replay rows spans = {arm: row} (span c1/9999_0001)."""
  rows = {k: [_row('2232_s1', 4.0)] for k in ('HEAD', 'ON', 'OFF')}
  for k, x in spans.items():
    (tmp_path / k).mkdir(exist_ok=True)
    _dump(tmp_path / k / 'replay_c1.pkl', [dict(kind='replay', corpus='c1', span='9999_0001', err=None, res=x)])

  def frames(c, s):
    return dict(origin=100.0, frames=[dict(t=100.0 + k * 0.01, active=True, cs=dict(gasPressed=False, brakePressed=False)) for k in range(400)])
  return _evaluate(tmp_path, rows, frames_fn=frames, **kw)


def test_required_jobs_come_from_the_whole_spec(tmp_path, monkeypatch, capsys):
  """Astra tooling finding 1: --stages selects what is computed, never what the verdict needs; a census hold without a cached case is
  a required job (INCOMPLETE), not a smaller census."""
  from openpilot.tools.stopping.sim import cases as C
  monkeypatch.setattr(C, '_HOLDS', {'0000aaaa_1.00': {}, '0000bbbb_2.00': {}})
  monkeypatch.setattr(C.H, 'CACHE', tmp_path / 'cases')   # neither case is cached
  assert C.hold_ids() == ['sv_0000aaaa_1.00', 'sv_0000bbbb_2.00']
  sim = tmp_path / 'sim'
  (sim / 'frames').mkdir(parents=True)
  (sim / 'frames' / 'c1.json').write_text(json.dumps(dict(spans=[dict(span='9999_0001', lo=0.0, hi=1.0)])))
  (sim / 'frames' / 'c2.json').write_text(json.dumps(dict(spans=[])))
  monkeypatch.setattr(RG, 'SIM_HOME', sim)
  monkeypatch.setattr(RG, 'SWEEP_LOCK', tmp_path / 'lock')
  m = dict(base_sha='0' * 40, tree='1' * 40, code_hash='c', diff_sha1=None, touched=[], flag_names=[], flags={}, base_flags={})
  monkeypatch.setattr(RG.loader, 'build', lambda base, diff=None: m)
  monkeypatch.setattr(RG, 'check_inputs', lambda a: ({}, {}))
  monkeypatch.setattr(RG, 'engine_sha', lambda: ('e', 'r', {}, 'j'))
  monkeypatch.setattr(RG, 'car_check', lambda *a: dict(car='c', car_sha='0' * 40, same_code=True, differ=[], n_differ=0))

  def arm(mm, flags, eng, ref=None):
    d = sim / 'arms' / eng
    d.mkdir(parents=True, exist_ok=True)
    return dict(key=eng, code_hash='c', tree=mm['tree'], base_sha=mm['base_sha'], diff_sha1=None, overrides={}, flags_sha1='f', engine=eng,
                ref=None, dir=d, manifest='', ref_dir=None, flag_values={})
  monkeypatch.setattr(RG, 'arm', arm)
  monkeypatch.setattr(RG, 'closed_jobs', lambda spec, cells_only=None: [dict(kind='closed', case=c, cell='L42', start=s, mode=md)
                                                                       for c, s, md in (('sv_0000aaaa_1.00', 'hold', 'nodrv'), ('s20', 'v12', 'drv'))])
  for eng in ('e', 'r'):   # the computed stages' rows are cached; the holds stage's are not
    (sim / 'arms' / eng).mkdir(parents=True)
  _dump(sim / 'arms' / 'e' / 'closed.pkl', [_row('s20', 4.0)])
  _dump(sim / 'arms' / 'r' / 'replay_c1.pkl', [dict(kind='replay', corpus='c1', span='9999_0001', err=None, res=_span())])
  ran = []
  monkeypatch.setattr(RG, 'run_worker', lambda a, jobs, log, module='worker': ran.extend(j['kind'] for j in jobs[:1]) or (0.0, 0))
  assert RG.main(['--stages', 'standard,replay,off', '--label', 't']) == 2   # the holds stage has no rows: INCOMPLETE, never PASS
  res = json.loads(next((sim / 'runs').glob('*_t/gate.json')).read_text())
  assert res['verdict'] == 'INCOMPLETE' and ['sv_0000aaaa_1.00', 'L42', 'hold', 'nodrv'] in [k for x in res['missing'] for k in x['keys']]
  assert 'replay' in ran and 'closed' in ran


def test_h6_compares_the_arms_at_the_same_instants(tmp_path):
  """Astra tooling finding 2: the candidate releases 0.3 at 2 s, HEAD at 3 s (the same array index of a trace that starts 1 s later)."""
  v = _x(owning=np.ones(400))
  v['wire'][200:] = -0.2
  h = _x()
  h['t'] = h['t'] + 1.0
  h['wire'][200:] = -0.2
  assert G.owned_release_breaches(h, v, 100, 400)[0] == (2.0, 0.3, 'service')
  hh = G.on_grid(h, v['t'])
  assert not hh['_cov'][:99].any() and hh['_cov'][100:].all() and hh['wire'][150] == -0.5
  v['wire'][250:] = -0.5   # back to HEAD's command 0.5 s later: the gate windows are the same instants in both arms
  a, b = _row('2232_s1', 4.0), _row('2232_s1', 4.0, 'b')
  b['tr'] = {k: (c[25:] if np.ndim(c) else c) for k, c in b['tr'].items()}   # the candidate's stored trace starts 0.5 s later
  xh, (k0h, kdh), xv, (k0, kd), nu = G.common_window(a, b)
  assert xh['t'][k0h] == xv['t'][k0] == pytest.approx(0.5) and nu == 0 and kd - k0 == kdh - k0h
  assert G.window_same(a, b)


def test_h6_release_back_to_heads_command_counts():
  """Astra tooling finding 3: ON releases 0.5 in one frame onto HEAD's own value."""
  h = _x()
  v = _x(owning=np.ones(400))
  v['wire'][:200] = -1.0
  assert G.owned_release_breaches(h, v, 0, 400)[0] == (2.0, 0.5, 'service')


def test_replay_h6_checks_the_service_with_a_plan_difference(tmp_path):
  """Astra tooling finding 4: a constant planner difference (-0.5 vs -0.6) must not hide a 0.4 service-owned release."""
  wire = np.where(np.arange(400) < 200, -1.0, -0.6)
  off = _span()
  on = _span(wire=wire, owning=np.ones(400, dtype=np.int8), at=np.full(80, -0.6))
  res, g = _replay_eval(tmp_path, {'HEAD': off, 'OFF': off, 'ON': on})
  h6 = g['H6 J-limited releases']
  assert h6['verdict'] == 'FAIL' and h6['fails'][0]['source'] == 'replay' and 'service-owned command' in h6['fails'][0]['check']
  assert h6['fails'][0]['first'][0][:2] == (2.0, 0.4)


@pytest.mark.parametrize('plan, why', [(dict(ns=np.zeros(0), at=np.zeros(0), ss=np.zeros(0)), 'no planner ticks'),
                                       ('nan', 'non-finite'), ('gap', 'without a planner tick')])
def test_invalid_replay_plans_prevent_a_verdict(tmp_path, plan, why):
  """Astra tooling finding 5: empty, non-finite or gapped planner lockstep rows are not a PASS of H1a."""
  x = _span()
  if plan == 'nan':
    x['plan']['at'][3] = np.nan
  elif plan == 'gap':
    x['plan']['ns'] = x['plan']['ns'][:40]   # the ticks stop at 2 s of a 4 s span
    x['plan']['at'], x['plan']['ss'] = x['plan']['at'][:40], x['plan']['ss'][:40]
  else:
    x['plan'] = plan
  assert why in G.plan_problem(x) and G.plan_problem(_span()) is None
  res, g = _replay_eval(tmp_path, {'HEAD': x, 'OFF': x, 'ON': x})
  assert res['verdict'] == 'INCOMPLETE' and any(m['kind'] == 'replay plan invalid' for m in res['missing'])
  nan = _span()
  nan['plan']['at'][:] = np.nan
  assert G.plan_diff(_span(), nan)['plan_at'] == 80   # a NaN plan never equals a finite one


def test_engine_fingerprints_cover_every_executed_file(monkeypatch):
  """Astra tooling finding 6: a change of the sender test helper (run_frame) or the loader changes both engine fingerprints, and an
  executed tools/stopping or test file outside them is reported."""
  from openpilot.tools.stopping.sim import worker as W
  helper = L.REPO / 'opendbc_repo/opendbc/car/hyundai/tests/test_can_bounds_fork.py'
  for files in (W.ENGINE_FILES, W.REPLAY_FILES):
    assert helper in files and Path(L.__file__).resolve() in files
  assert Path(W.HERE / 'radar_replay.py') in W.REPLAY_FILES   # the radar stage (cycle_20261006) keys the replay arms
  before = (W.engine_sha()[0], W.engine_sha(W.REPLAY_FILES)[0])
  orig = Path.read_bytes
  monkeypatch.setattr(Path, 'read_bytes', lambda p: orig(p).replace(b"v_ego=cmd.get('v_ego', 0.0)", b'v_ego=0.0') if p == helper else orig(p))
  after = (W.engine_sha()[0], W.engine_sha(W.REPLAY_FILES)[0])
  assert before[0] != after[0] and before[1] != after[1]
  assert 'tools/stopping/sim/gates.py' in W.unhashed(W.ENGINE_FILES)   # imported by this test, not by the engine: would be refused


def test_temporary_index_is_under_sim_home(repo, tmp_path, monkeypatch):
  """Astra tooling finding 11."""
  seen = []
  orig = L.git
  monkeypatch.setattr(L, 'git', lambda *a, env=None, inp=None: seen.append((env or {}).get('GIT_INDEX_FILE')) or orig(*a, env=env, inp=inp))
  L.build('HEAD', _diff(repo, tmp_path, {'selfdrive/controls/lib/mod.py': 'X = 2\n'}))
  idx = [p for p in seen if p]
  assert idx and all(p.startswith(str(tmp_path / 'sim')) for p in idx)


@pytest.mark.skipif(not (L.SIM_HOME / 'cases' / '2235_s71.pkl').is_file(), reason='needs SIM_HOME case cache (README.md)')
def test_smoke_one_closed_run_is_deterministic(tmp_path):
  m = L.arm_manifest(L.build('HEAD'), {})
  jobs = tmp_path / 'j.json'
  jobs.write_text(json.dumps([dict(kind='closed', case='ms_v8_none', cell='L42', start='auto', mode='drv')]))
  env = {k: v for k, v in os.environ.items() if k != 'OPENPILOT_PREFIX'} | dict(STOP_SIM_TREE=m['path'], SIM_PROCS='1')   # conftest prefix: no ZMQ
  shas = []
  for i in range(2):
    d = tmp_path / f'a{i}'
    d.mkdir()
    out = subprocess.run([sys.executable, '-m', 'openpilot.tools.stopping.sim.worker', str(d), str(jobs)], check=True, env=env, capture_output=True,
                         text=True)
    rows = _load(d / 'closed.pkl')
    assert len(rows) == 1, out.stdout[-2000:]
    assert rows[0]['m']['t_stop'] is not None   # the model-only stop family stops (no radar lead)
    shas.append(rows[0]['trace_sha'])
  assert shas[0] == shas[1]


# ---- radar stage (cycle_20261006 builder R) ------------------------------------------------------------------------------------
def _radar_row(n=400, bad_scored=0, kw_matched=None, commit='abc'):
  r = _span(n)
  r['commit'] = commit
  fid = dict(ticks=200, scored=80, cold_kf_scored=0, bad_scored=bad_scored, bad_pre=0, race_tol_scored=0, race_tol_max=0.0,
             per_field={'l1_vLead': (bad_scored, 0)} if bad_scored else {},
             worst={}, first_bad=None, delay=0.0, n_skip=0, n_lt_none=0, n_race=0, n_no_toggles=0)
  r['radar'] = dict(fid=fid, fp_fid=dict(ticks=80, per_field={}, worst={}), kw_matched=n if kw_matched is None else kw_matched, m1=None, subst={})
  return r


@pytest.mark.parametrize('row, same, incomplete', [
  (dict(), True, False), (dict(bad_scored=3), True, True), (dict(bad_scored=3), False, False), (dict(kw_matched=390), False, True)])
def test_radar_fidelity_mismatch_on_a_matching_build_is_incomplete(row, same, incomplete):
  """T1: a re-run tick that differs from the logged radarState inside the scored window of a drive with HEAD's radard code makes the
  run INCOMPLETE; another build's differences are listed; a frame not mapped to its logged radarState is INCOMPLETE on any build."""
  bad, r1, r3 = G.radar_fidelity({'c1': {'9999_0001': _radar_row(**row)}}, 'tree', same=lambda *a: same)
  assert bool(bad) == incomplete and (r1['verdict'] == 'INCOMPLETE') == incomplete
  assert r3['verdict'] == 'INFO'
  if row.get('bad_scored') and not same:
    assert r1['evidence'] and r1['evidence'][0]['bad_scored'] == 3


def _fid_R(n=400):
  """radard_pass arrays of n identical re-run / logged ticks (50 ms; a radar leadOne, no leadTwo) for radar_replay.fidelity."""
  from openpilot.tools.stopping.sim import radar_replay as RR
  R = dict(ns=np.arange(n, dtype=np.int64) * 50_000_000, first_ns=np.int64(0), delay=0.0, n_skip=0, n_lt_none=0, n_race=0, n_no_toggles=0)
  for w, st in (('l1', 1.0), ('l2', 0.0)):
    for f in RR.LEAD_F:
      v = np.full(n, st if f in ('status', 'radar') else 7.0 if f == 'radarTrackId' else 0.0 if f == 'fcw' else 1.5)
      R[f'rs_{w}_{f}'], R[f'log_{w}_{f}'] = v.copy(), v.copy()
    R[f'race_{w}'] = np.zeros(n)
  for f in RR.SURR_F:
    R[f'rs_{f}'], R[f'log_{f}'] = np.zeros(n), np.zeros(n)
  return R


@pytest.mark.parametrize('inject, bad', [('none', 0), ('cold_10', 20), ('cold_10_race', 20), ('nan_warm', 1), ('kf_0.03', 1), ('kf_0.03_race', 0),
                                         ('kf_0.06_race', 1)])
def test_radar_fidelity_scores_cold_nan_and_race_tolerance(inject, bad):
  """Astra tooling review finding 3: a scored KF error in the cold first seconds counts (no exemption: 10 m/s^2 aLeadK fails), a NaN
  after the warm-up counts, KF_TOL (0.05) applies only on race-affected warm ticks and is reported."""
  from openpilot.tools.stopping.sim import radar_replay as RR
  R = _fid_R()
  lo = 0 if inject.startswith('cold_10') else 250 * 50_000_000   # scored window: 20 ticks from the re-run start (cold) or after 12.5 s (warm)
  hi = lo + 19 * 50_000_000
  k = int(lo // 50_000_000)
  if inject.startswith('cold_10'):
    R['rs_l1_aLeadK'][k:k + 20] += 0.03 if inject.endswith('race') else 10.0
    R['race_l1'][k:k + 20] = 1.0 if inject.endswith('race') else 0.0
  elif inject == 'nan_warm':
    R['rs_l1_aLeadK'][k + 5] = np.nan
  elif inject.startswith('kf_'):
    R['rs_l1_vLeadK'][k + 5] += float(inject.split('_')[1])
    if inject.endswith('race'):
      R['race_l1'][k:k + 20] = 1.0
  f = RR.fidelity(R, 0, lo, hi)
  assert f['scored'] == 20 and f['bad_scored'] == bad
  assert f['cold_kf_scored'] == (20 if inject.startswith('cold_10') else 0)
  assert (f['race_tol_scored'], f['race_tol_max']) == ((1, 0.03) if inject == 'kf_0.03_race' else (0, 0.0))


def test_live_tracks_index_takes_the_cycle_message_and_settles_races_by_the_logged_leads():
  """The liveTracks message radard read (shared by the exact replay and the closed loop): the latest one sent no later than the
  carState's card cycle and before the radarState; where two were sent around the carState radard read, the one that reproduces the
  logged lead (track id + yRel / vRel / published dRel) wins."""
  from types import SimpleNamespace as NS
  from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.stop_target_helpers import get_published_lead_distance
  from openpilot.tools.stopping.sim import radar_replay as RR
  off = NS(status=False, radar=False, radarTrackId=-1, yRel=0.0, vRel=0.0, dRel=0.0)
  R = NS(logMonoTime=1000, radarState=NS(leadOne=NS(status=True, radar=True, radarTrackId=7, yRel=0.5, vRel=-1.0,
                                                    dRel=get_published_lead_distance(20.0, 0.0)), leadTwo=off))
  lt = lambda v: NS(liveTracks=NS(points=[NS(trackId=7, yRel=0.5, vRel=v, dRel=20.0)]))  # noqa: E731
  by = lambda ns, evs: {'carState': ([900, 1900], [None, None]), 'liveTracks': (ns, evs), 'frogpilotRadarState': ([], []), 'frogpilotPlan': ([], [])}  # noqa: E731
  assert RR.live_tracks_index(by([800, 901], [lt(-1.0), lt(-1.0)]), R, 900) == (1, False, frozenset())  # one message after the carState
  assert RR.live_tracks_index(by([800, 901, 950], [lt(-2.0), lt(-1.0), lt(-3.0)]), R, 900) == (1, True, {7})   # a race: the logged lead decides
  assert RR.live_tracks_index(by([800, 901, 950], [lt(-2.0), lt(-3.0), lt(-1.0)]), R, 900) == (2, False, {7})  # the latest already matches
  assert RR.live_tracks_index(by([1200], [lt(-1.0)]), R, 900) == (-1, False, frozenset())                 # none sent before the radarState


def test_r4_counts_candidate_changes_that_rest_on_a_head_replay_mismatch():
  """R3 propagation: the candidate's planner reads the log + (candidate - HEAD); R4 counts the ticks where the candidate's
  lead-consumer field differs from HEAD's and HEAD's replay differs from the logged frogpilotPlan (the change rests on a mismatch)."""
  ns = np.arange(6, dtype=np.int64)
  em_log, em_head, em_on = (np.array(x, np.float32) for x in ([1, 1, 0, 0, 1, 1], [1, 0, 0, 0, 1, 1], [0, 1, 1, 0, 1, 1]))
  row = lambda em: dict(radar=dict(fp=dict(ns=ns, fp_experimentalMode=em, log_fp_experimentalMode=em_log, fp_leadDeparting=np.zeros(6, np.float32))))  # noqa: E731
  g = G.fp_compare({'c1': {'s': row(em_head)}}, {'c1': {'s': row(em_on)}})
  assert g['coverage'] == 6 and "{'experimentalMode': 3}" in g['note'] and "the log: {'experimentalMode': 1}" in g['note']


def _m1_rows(n_spans=3, n=400, opt=0.0, cls='braking'):
  """Replay rows whose radar stage carries M1 samples: one radar leadOne track per span; HEAD publishes the truth (+ a small noise),
  the candidate adds opt m/s where the lead brakes (truth a <= -1, ego braking)."""
  rng = np.random.default_rng(0)
  out = {}
  for k in range(n_spans):
    t = 100.0 + np.arange(n) * 0.05
    a = np.where(np.arange(n) < n // 2, -2.0, 0.0) if cls == 'braking' else np.zeros(n)
    v = 10.0 + np.cumsum(a) * 0.05
    m = dict(t=t, a_ego=np.full(n, -2.0, np.float32))
    for w in ('l1', 'l2'):
      pub = v + rng.normal(0, 0.02, n) + np.where(a <= -1.0, opt, 0.0)
      m.update({f'{w}_status': np.ones(n, np.float32), f'{w}_radar': np.ones(n, np.float32), f'{w}_radarTrackId': np.full(n, 7, np.float32),
                f'{w}_vLead': pub.astype(np.float32), f'{w}_vLeadK': pub.astype(np.float32), f'{w}_aLeadK': a.astype(np.float32),
                f'{w}_a': a.astype(np.float32), f'{w}_age': (np.arange(n) * 0.05 + 5).astype(np.float32), f'{w}_static': np.zeros(n, np.float32),
                f'{w}_onset': np.zeros(n, np.float32), f'{w}_sign': np.zeros(n, np.float32)})
      m.update({f'{w}_v{i}{j}': v.astype(np.float32) for i in range(3) for j in range(3)})
    r = _radar_row()
    r['radar']['m1'] = m
    out[f'9999_{k:04d}'] = r
  return {'c1': out}


def test_m1_fails_a_braking_lead_read_too_fast_and_passes_identity():
  """T6: a candidate that reads a braking lead 0.25 m/s faster than HEAD at every truth corner FAILS M1; HEAD vs itself has no
  failing class; classes without samples are EMPTY, so the identity run is NO COVERAGE (not PASS) on this braking-only corpus."""
  head, cand = _m1_rows(), _m1_rows(opt=0.25)
  v, rows = G.m1(head, cand)
  assert v == 'FAIL'
  bad = [r for r in rows if r['verdict'] == 'FAIL']
  assert any(r['lead'] == 'l1' and r['cls'].startswith('braking lead (a <= -1.0), ego a <= -1.5') for r in bad)
  assert any(r['cls'].startswith('sustained optimistic excursions vLead') for r in bad)
  v0, rows0 = G.m1(head, head)
  assert v0 == 'NO COVERAGE' and not any(r['verdict'] in ('FAIL', 'UNCERTAIN') for r in rows0)
  assert any(r['verdict'] == 'EMPTY' for r in rows0)


def test_m1_small_optimism_within_tolerance_passes_its_class():
  v, rows = G.m1(_m1_rows(), _m1_rows(opt=0.02))
  r = next(r for r in rows if r['cls'].startswith('braking lead (a <= -1.0), ego a <= -1.5') and r['lead'] == 'l1')
  assert r['verdict'] == 'PASS'


def _m1_static(n_spans=30, n=4000):
  rows = _m1_rows(n_spans=n_spans, n=n, cls='steady')
  for r in rows['c1'].values():
    m = r['radar']['m1']
    for w in ('l1', 'l2'):
      m[f'{w}_static'] = np.ones(n, np.float32)
      m[f'{w}_vLead'] = m[f'{w}_vLeadK'] = np.random.default_rng(1).normal(0, 0.02, n).astype(np.float32)
  return rows


@pytest.mark.parametrize('ticks, v, fails', [(20, 0.8, True), (5, 0.8, False), (20, 0.12, False), (0, 0.0, False), ('moved', 0.2, False),
                                              ('moved_higher', 0.4, True)])
def test_m1_stationary_episode_is_not_pooled_away(ticks, v, fails):
  """Astra tooling review finding 4: 20 ticks (1 s) at +0.8 m/s on one stationary track among 120k stationary ticks pass every pooled
  check but FAIL the episode check (> 0.15 m/s for >= 0.3 s); a 0.25 s or a 0.12 m/s excursion is no episode. HEAD has two 0.4 s
  episodes at 0.2 m/s: a candidate with one of them moved elsewhere (fewer, no worse) passes; a moved one at 0.4 m/s FAILS."""
  head, cand = _m1_static(), _m1_static()
  for rows in (head, cand):
    for sp in ('9999_0003', '9999_0004'):
      m = rows['c1'][sp]['radar']['m1']
      m['l1_vLead'] = m['l1_vLead'].copy()
      m['l1_vLead'][500:508] = 0.2
  m = cand['c1']['9999_0007']['radar']['m1']
  m['l1_vLead'] = m['l1_vLead'].copy()
  if isinstance(ticks, str):   # HEAD's episode on span 3 is gone in the candidate, a new one appears on span 7
    m3 = cand['c1']['9999_0003']['radar']['m1']
    m3['l1_vLead'] = np.zeros_like(m3['l1_vLead'])
    m['l1_vLead'][2000:2008] = v
    ticks = 0
  m['l1_vLead'][1000:1000 + ticks] = v
  verdict, rows = G.m1(head, cand)
  ep = next(r for r in rows if r['lead'] == 'l1' and r['cls'].startswith('stationary optimistic episodes vLead '))
  pooled = next(r for r in rows if r['lead'] == 'l1' and r['cls'] == 'stationary')
  assert pooled['verdict'] == 'PASS'
  assert (ep['verdict'] == 'FAIL') is fails and (verdict == 'FAIL') is fails
  if fails:
    assert ep['failing'][0]['span'] == 'c1|9999_0007' and ep['failing'][0]['peak'] == v


HEAD_RP = L.SIM_HOME / 'arms/f1e8c49444bf2b87/replay_c1.pkl'   # run 20261006-1227_e_c2fev, HEAD replay arm


@pytest.mark.skipif(not HEAD_RP.is_file(), reason='needs the 20261006-1227_e_c2fev HEAD replay rows')
def test_m1_fails_astra_stationary_injection_on_saved_head_rows():
  """Astra's reproduction on the saved HEAD rows: c1/20bf_0108 leadOne vLead at 2088.015-2088.970 s (20 stationary ticks, ego braking)
  set to +0.8 m/s -> M1 FAIL (the HEAD copy against itself is not FAIL)."""
  import copy
  head = {'c1': G.replay_rows(HEAD_RP.parent, 'c1')}
  cand = {'c1': dict(head['c1'])}
  span = next(s for s in head['c1'] if s.endswith('20bf_0108') or s == '20bf_0108')
  x = cand['c1'][span] = copy.copy(head['c1'][span])
  x['radar'] = dict(x['radar'], m1=dict(x['radar']['m1']))
  m = x['radar']['m1']
  k = (m['t'] >= 2088.0) & (m['t'] <= 2088.98) & (m['l1_static'] > 0)
  assert k.sum() == 20
  m['l1_vLead'] = np.where(k, 0.8, m['l1_vLead']).astype(np.float32)
  v, rows = G.m1(head, cand)
  assert v == 'FAIL'
  assert [r['cls'] for r in rows if r['verdict'] == 'FAIL'] == [f'stationary optimistic episodes vLead (> {G.M1_STAT_V} m/s for >= {G.M1_STAT_S} s, truth 0)']
  assert G.m1(head, head)[0] != 'FAIL'


def test_m1_stationary_lead_info_rows_split_by_ego_accel():
  """M1 INFO: the stationary leadOne per ego accel bin (signed mean, MAE, p99 |err| HEAD -> candidate); never part of the verdict."""
  def rows(err_hard):
    out = _m1_rows(cls='steady')
    for r in out['c1'].values():
      m = r['radar']['m1']
      n = len(m['t'])
      m['a_ego'] = np.repeat(np.array([-2.0, -1.0, 0.0], np.float32), -(-n // 3))[:n]
      m['l1_static'] = np.ones(n, np.float32)
      m['l1_vLead'] = m['l1_vLeadK'] = np.where(m['a_ego'] <= -1.5, err_hard, 0.0).astype(np.float32)
    return out
  v_same, rows_same = G.m1(rows(-0.27), rows(-0.27))
  v, rows_ = G.m1(rows(-0.27), rows(0.0))
  info = [r for r in rows_ if r['verdict'] == 'INFO']
  assert [r['cls'].split(' (')[0] for r in info] == ['stationary, ego a -inf..-1.5', 'stationary, ego a -1.5..-0.5', 'stationary, ego a -0.5..inf']
  assert info[0]['stats']['vLead'][0] == (-0.27, 0.27, 0.27) and info[0]['stats']['vLead'][1] == (0.0, 0.0, 0.0)
  assert info[1]['stats']['vLeadK'] == [(0.0, 0.0, 0.0)] * 2 and info[0]['n'][0] == info[0]['n'][1] > 0
  assert v == v_same   # the INFO rows do not move the verdict (the pooled stationary class judges the candidate)
  v0, rows0 = G.m1({'c1': {}}, {'c1': {}})
  assert all(r['n'] == (0, 0) and r['stats']['vLead'] == [None, None] for r in rows0 if r['verdict'] == 'INFO')


def test_plan_inputs_forward_the_complete_plan_and_the_arm_leads():
  """T2: FCW, distance to stop (planner and model), the trajectory and its validity reach LongControl as the arm's change; the lead
  inputs follow the arm's re-run radarState (float delta; the arm's lead where the track differs) and its experimental mode."""
  from openpilot.tools.stopping.sim import replay as RP
  kw = dict(experimental_mode=True, lead_status=True, lead_v=5.0, lead_d_rel=20.0, lead_a=-1.0, lead_track_id=7, lead_model_prob=0.9,
            lead2_status=False, lead2_v=0.0, lead2_d_rel=0.0, fcw=False, model_stop_d=30.0, a_target_trajectory=-0.5)
  fr = [dict(t=100.0 + k * 0.01, target=-0.5, should_stop=False, dts=40.0, kw=dict(kw)) for k in range(10)]
  ns = np.array([int(100.0e9), int(100.05e9)])
  base = dict(ns=ns, log_at=np.full(2, -0.5), log_ss=np.zeros(2), at=np.full(2, -0.6), ss=np.zeros(2), fcw=np.zeros(2), dts=np.full(2, 35.0),
              dtsm=np.full(2, 25.0), traj=np.full(2, -0.6), trajv=np.ones(2))
  arm = dict(base, at=np.full(2, -0.9), fcw=np.ones(2), dts=np.full(2, 33.0), dtsm=np.full(2, 24.0), traj=np.full(2, -1.0))
  lead_f = ('status', 'radarTrackId', 'vLead', 'dRel', 'aLeadK', 'modelProb')
  RB = {f'rs_{w}_{f}': np.array([1.0, 1.0]) * v for w in ('l1', 'l2') for f, v in zip(lead_f, (1, 7, 5.2, 20.5, -1.0, 0.9), strict=True)}
  RA = dict(RB, rs_l1_vLead=np.array([5.5, 5.5]), ns=np.array([1, 2]))
  RA['rs_l2_status'] = RB['rs_l2_status'] = np.zeros(2)
  RB['ns'] = RA['ns']
  FA, FB = dict(ns=np.array([1, 2]), fp_experimentalMode=np.zeros(2)), dict(ns=np.array([1, 2]), fp_experimentalMode=np.ones(2))
  ref = dict(base, **{f'R_{k}': v for k, v in RB.items()}, **{f'F_{k}': v for k, v in FB.items()})
  lead = dict(R=RA, F=FA, rs_i=np.ones(10, int), fp_i=np.ones(10, int))
  d2, matched, changed = RP.plan_inputs(dict(frames=fr), arm, ref, lead)
  g = d2['frames'][0]
  assert matched == 10 and changed == 10
  assert g['target'] == pytest.approx(-0.8) and g['dts'] == pytest.approx(38.0) and g['kw']['model_stop_d'] == pytest.approx(29.0)
  assert g['kw']['fcw'] is True and g['kw']['a_target_trajectory'] == pytest.approx(-0.9)
  assert g['kw']['lead_v'] == pytest.approx(5.3) and g['kw']['lead_d_rel'] == 20.0 and g['kw']['experimental_mode'] is False
  same, _, n0 = RP.plan_inputs(dict(frames=fr), base, ref, dict(lead, R=RB, F=FB))
  assert n0 == 0 and same['frames'][0] is fr[0]   # the reference arm's own plan and leads: the recorded frames, untouched


SPAN = ('c2', '2072_0045')


@pytest.mark.skipif(not (L.SIM_HOME / 'frames' / SPAN[0] / f'{SPAN[1]}.pkl').is_file(), reason='needs SIM_HOME frames + the route rlogs (README.md)')
def test_radar_stage_is_exact_at_head_and_a_publication_change_alters_the_command(tmp_path, monkeypatch):
  """T1 + T2 on a recorded span: the re-run radarState equals the logged one (fidelity), the HEAD-referenced replay of HEAD is the
  identity, and a radard change that only alters the published lead speed changes the replayed plan and command."""
  monkeypatch.delenv('OPENPILOT_PREFIX', raising=False)   # conftest prefix: the macOS ZMQ backend refuses it (radard's SubMaster default)
  from openpilot.selfdrive.controls import radard
  from openpilot.tools.stopping.sim import replay as RP
  job = dict(corpus=SPAN[0], span=SPAN[1], plan_dir=str(tmp_path / 'head'), ref_plan_dir=None)
  h = RP.span_job(job)
  f = h['radar']['fid']
  assert f['scored'] > 0 and f['bad_scored'] == 0 and h['radar']['kw_matched'] == len(h['wire'])
  same = RP.span_job(dict(job, plan_dir=str(tmp_path / 'off'), ref_plan_dir=str(tmp_path / 'head')))
  assert same['plan_info']['frames_changed'] == 0 and np.array_equal(same['wire'], h['wire'])
  orig = radard.Track.get_RadarState

  def published(self, model_prob=0.0):
    d = orig(self, model_prob)
    d['vLead'] += 0.5
    return d
  monkeypatch.setattr(radard.Track, 'get_RadarState', published)
  on = RP.span_job(dict(job, plan_dir=str(tmp_path / 'on'), ref_plan_dir=str(tmp_path / 'head')))
  assert np.abs(on['plan']['at'] - h['plan']['at']).max() > 0.05
  assert np.abs(on['wire'] - h['wire']).max() > 0.05
  assert (on['radar']['m1']['l1_vLead'] - h['radar']['m1']['l1_vLead']).max() == pytest.approx(0.5, abs=1e-3)


# ---- INFO stages: the judges' grids (judges.py) and the launch delays (never gating) -------------------------------------------
def _jrow(i, min_gap, sha='a', **kw):
  """A judges.py row (adv_lib metrics) for configuration i."""
  r = dict(i=i, cfg=dict(v0=2.0, gap0=8.0 + i), min_gap=min_gap, rest=min_gap, min_wire=-1.0, regrab=0.0, pid_hold_s=0.0, rest_nosr_s=0.0,
           rest_s=3.0, wire_sha=sha)
  r.update(kw)
  return r


def test_judge_grids_keep_the_judges_sizes_and_stay_out_of_the_closed_and_replay_keys():
  """The folded grids are the judges' grids (jstress 288 runs, adv.py's 13 grids, Astra's pair), and the judge engine is its own
  fingerprint: judges.py is not in the closed / replay engine files, so the INFO stages never change a cached gate row's arm key."""
  from openpilot.tools.stopping.sim import judges as J
  from openpilot.tools.stopping.sim import worker as W
  sizes = {g: len(J.grid(g)) for gs in J.STAGES.values() for g in gs}
  assert sizes == dict(stress=288, cutin=384, move=648, noise=648, far=405, depart=192, sgs=432, flap=320, regap=384, push=240, crawl=288,
                       spike=400, fault=480, gas=320, cutfix=2)
  assert set(J.STAGES) == set(RG.INFO_STAGES) and not set(RG.INFO_STAGES) & set(RG.STAGES)
  me = Path(J.__file__).resolve()
  assert me in J.FILES and me not in W.ENGINE_FILES and me not in W.REPLAY_FILES and set(W.COMMON_FILES) <= set(J.FILES)
  assert J.ENGINE_SHA != W.ENGINE_SHA


def test_smoke_judge_worker_checks_the_arm_writes_rows_and_reuses_them(tmp_path):
  """The judge worker in a real arm process: its per-job checks pass (no executed file outside its fingerprint, flags from the arm
  file), Astra's pair is written to judges.pkl, and a repeated run reuses the finished grid."""
  m = L.arm_manifest(L.build('HEAD'), {})
  jobs = tmp_path / 'jobs_judge.json'
  jobs.write_text(json.dumps([dict(kind='judge', grid='cutfix')]))
  env = {k: v for k, v in os.environ.items() if k != 'OPENPILOT_PREFIX'} | dict(STOP_SIM_TREE=m['path'], SIM_PROCS='1')
  outs = [subprocess.run([sys.executable, '-m', 'openpilot.tools.stopping.sim.judges', str(tmp_path), str(jobs)], env=env, capture_output=True,
                         text=True) for _ in range(2)]
  assert [o.returncode for o in outs] == [0, 0], outs[0].stdout[-2000:] + outs[0].stderr[-2000:]
  rows = _load(tmp_path / 'judges.pkl')
  assert [r['grid'] for r in rows] == ['cutfix'] and len(rows[0]['rows']) == 2 and all('err' not in x and x['cutin_t'] == 1.32 for x in rows[0]['rows'])
  assert '0/1 grids' in outs[1].stdout


def _die_in_worker(j):
  os._exit(23)   # a judge worker killed mid-grid (signal, OOM, os._exit)


def test_judge_worker_death_is_a_reported_grid_error_not_a_hang(tmp_path):
  """Astra tooling review P2: a dead judge worker used to leave multiprocessing.Pool waiting forever for its lost task, so the gate run
  never finished. Every unfinished grid must become a reported error and the worker must return (exit 3) in bounded time."""
  jobs = tmp_path / 'jobs_judge.json'
  jobs.write_text(json.dumps([dict(kind='judge', grid='cutfix'), dict(kind='judge', grid='stress')]))
  code = ('import sys; from openpilot.tools.stopping.sim import judges as J, test_sim as T; J.job = T._die_in_worker; ' +
          f'sys.exit(J.main(["judges", {str(tmp_path)!r}, {str(jobs)!r}]))')
  env = {k: v for k, v in os.environ.items() if k != 'OPENPILOT_PREFIX'} | dict(SIM_PROCS='2')
  out = subprocess.run([sys.executable, '-c', code], env=env, capture_output=True, text=True, timeout=120)
  assert out.returncode == 3, out.stdout[-2000:] + out.stderr[-2000:]
  errs = json.loads((tmp_path / 'errors_jobs_judge.json').read_text())
  assert sorted(e['key'][0] for e in errs) == ['cutfix', 'stress'] and all('BrokenProcessPool' in e['err'] for e in errs)
  assert not (tmp_path / 'judges.pkl').exists()


def _instant_grid(j):
  return dict(j, err=None, rows=[], secs=0.0)


def test_judge_workers_finish_more_grids_than_a_worker_lifetime(tmp_path):
  """The gate run hung after 8 grids twice on 2026-10-10 (Python 3.11's ProcessPoolExecutor max_tasks_per_child=4 deadlocks once the
  first workers retire): an arm with more grids than two worker lifetimes must finish."""
  jobs = tmp_path / 'jobs_judge.json'
  jobs.write_text(json.dumps([dict(kind='judge', grid=f'g{i}') for i in range(14)]))
  code = ('import sys; from openpilot.tools.stopping.sim import judges as J, test_sim as T; J.job = T._instant_grid; ' +
          f'sys.exit(J.main(["judges", {str(tmp_path)!r}, {str(jobs)!r}]))')
  env = {k: v for k, v in os.environ.items() if k != 'OPENPILOT_PREFIX'} | dict(SIM_PROCS='2')
  out = subprocess.run([sys.executable, '-c', code], env=env, capture_output=True, text=True, timeout=120)
  assert out.returncode == 0, out.stdout[-2000:] + out.stderr[-2000:]
  from openpilot.tools.stopping.sim import worker as W
  assert sorted(r['grid'] for r in W.load_rows(tmp_path / 'judges.pkl')) == sorted(f'g{i}' for i in range(14))


def test_judge_cut_in_at_a_fixed_time_and_by_speed():
  """Astra's pair: the stopped target appears 3 m nearer at 1.32 s whatever the speed (the cutin grid triggers below a speed)."""
  from openpilot.tools.stopping.sim import judges as J
  for c in J.grid('cutfix'):
    m = J.run(c, sender=False)
    assert m['cutin_t'] == 1.32 and abs(m['cutin_prior_gap'] - m['cutin_gap'] - 3.0) < 0.05 and 'rest_nosr_s' not in m
  c = J.grid('cutin')[0]
  m = J.run(c)
  assert m['cutin_v'] < c['cutin'][0] and m['cutin_t'] > 0.0 and 'rest_nosr_s' in m and len(m['wire_sha']) == 12


def test_judge_grid_rows_new_close_holds_handoffs_and_launches():
  H = [_jrow(0, 3.5), _jrow(1, 2.5), _jrow(2, 4.0, launch_dt=0.5, gap_at_launch=4.0), _jrow(3, 4.0, depart_rise_5f=0.05, depart_rise_1f=0.01),
       _jrow(4, 4.0), _jrow(5, 4.0, err='boom')]
  C = [_jrow(0, 2.9, 'b'), _jrow(1, 2.4, 'b'), _jrow(2, 4.0, 'b', launch_dt=0.9, gap_at_launch=4.5),
       _jrow(3, 4.0, 'b', depart_rise_5f=0.2, depart_rise_1f=0.1), _jrow(4, 4.0, pid_hold_s=1.0, rest_nosr_s=1.0, rest=6.5), _jrow(5, 4.0)]
  x = G.judge_grid(H, C)
  assert [len(x[k]) for k in ('new_lt3', 'closer_lt3', 'gap_worse', 'rest_gt6', 'pid_hold', 'nosr', 'handoff', 'launch')] == [1, 1, 1, 1, 1, 1, 1, 1]
  assert x['new_lt3'][0]['cand'] == 2.9 and x['worst'] == 2.4 and x['head_lt3'] == 1 and x['identical'] == 1 and len(x['errors']) == 1
  assert x['dgap']['min'] == -0.6 and x['launch'][0]['cand'] == 0.9
  C[2]['launch_dt'] = None   # never launches
  C[4].pop('rest_nosr_s')    # a grid without the sender: no StopReq measure
  x = G.judge_grid(H, C)
  assert x['launch'][0]['cand'] is None and x['nosr'] == []


def test_launch_delays_count_identical_pairs_as_zero_and_classify_the_later_ones():
  info = lambda **kw: dict(dict(head_move=10.0, cand_move=10.0, dt=0.0, dgap=0.0, head_rest_gap=4.0, cand_rest_gap=4.0, head_gap=4.0, cand_gap=4.0), **kw)  # noqa: E731
  e = [(('a',), None), (('b',), info(dt=0.1)), (('c',), info(dt=1.0, dgap=0.5)), (('d',), info(dt=0.8, cand_rest_gap=3.5)),
       (('e',), info(dt=0.6)), (('f',), info(cand_move=None, dt=None)), (('g',), info(head_move=None, dt=None))]
  r = G.launch_delays(e)
  assert r['n_judged'] == 7 and r['n_identical'] == 1 and r['n_dt'] == 5 and r['max'] == 1.0 and r['n_later'] == 3
  assert r['classes'] == {'on time': 2, 'defect (later at a larger gap)': 1, 'closer-rest': 1, 'other later': 1, 'defect (never moves)': 1,
                          'no HEAD motion': 1}
  assert [x['key'] for x in r['later']] == ['f', 'c', 'd', 'e']   # never moves first, then by delay
  assert G.launch_delays([])['median'] is None


def test_judges_are_reported_but_never_change_the_verdict(tmp_path):
  rows = {'HEAD': [_row('2232_s1', 4.0)], 'ON': [_row('2232_s1', 4.0)], 'OFF': [_row('2232_s1', 4.0)]}
  base, _ = _evaluate(tmp_path, rows)
  ja = {k: tmp_path / 'j' / k for k in ('HEAD', 'ON', 'OFF')}
  for k, d in ja.items():
    d.mkdir(parents=True)
    cut = [_jrow(0, 3.5 if k != 'ON' else 2.8, 'b' if k == 'ON' else 'a')]
    _dump(d / 'judges.pkl', [dict(kind='judge', grid='cutin', err=None, rows=cut),
                             dict(kind='judge', grid='cutfix', err=None, rows=[dict(_jrow(0, 3.1 if k != 'ON' else 2.8), cutin_t=1.32)])])
  (ja['ON'] / 'errors_jobs_judge_x.json').write_text(json.dumps([dict(key=['sgs'], err='Traceback\nAssertionError: x')]))
  res, _ = _evaluate(tmp_path, rows, judge_arms=ja, judge_grids=['cutin', 'cutfix', 'sgs'])
  assert res['verdict'] == base['verdict'] and [x['verdict'] for x in res['gates']] == [x['verdict'] for x in base['gates']]
  J = res['judges']
  assert len(J['grids']['cutin']['new_lt3']) == 1 and J['not_run'] == ['sgs'] and J['errors'][0]['grid'] == 'sgs'
  assert J['off'] == {'cutin': dict(n=1, differ=0, first=[]), 'cutfix': dict(n=1, differ=0, first=[])}
  assert J['pair'][0]['new_lt3'] and J['pair'][0]['track'] == 'same'
  line = G.render(res, short=True).splitlines()[-1]
  assert line.startswith('INFO (not gating)') and 'adversarial 1 grids 1 runs: NEW < 3.0 m 1 (worst 2.80 m)' in line and 'not run: sgs' in line
  assert "## INFO (not gating): judges' grids and launch delays" in G.render(res) and '| cutin | 1 | 0 | 0 | 2.8 | 1 |' in G.render(res)


@pytest.fixture
def fake_runner(tmp_path, monkeypatch):
  """run_gates.main on stubbed builds / arms / workers (no compute): one cached closed row; returns (SIM_HOME, the worker calls)."""
  sim = tmp_path / 'sim'
  (sim / 'frames').mkdir(parents=True)
  for c in ('c1', 'c2'):
    (sim / 'frames' / f'{c}.json').write_text(json.dumps(dict(spans=[])))
  monkeypatch.setattr(RG, 'SIM_HOME', sim)
  monkeypatch.setattr(RG, 'SWEEP_LOCK', tmp_path / 'lock')
  m = dict(base_sha='0' * 40, tree='1' * 40, code_hash='c', diff_sha1=None, touched=[], flag_names=[], flags={}, base_flags={})
  monkeypatch.setattr(RG.loader, 'build', lambda base, diff=None: m)
  monkeypatch.setattr(RG, 'check_inputs', lambda a: ({}, {}))
  monkeypatch.setattr(RG, 'engine_sha', lambda: ('e', 'r', {}, 'j'))
  monkeypatch.setattr(RG, 'car_check', lambda *a: dict(car='c', car_sha='0' * 40, same_code=True, differ=[], n_differ=0))

  def arm(mm, flags, eng, ref=None):
    d = sim / 'arms' / eng
    d.mkdir(parents=True, exist_ok=True)
    return dict(key=eng, code_hash='c', tree=mm['tree'], base_sha=mm['base_sha'], diff_sha1=None, overrides={}, flags_sha1='f', engine=eng,
                ref=None, dir=d, manifest='', ref_dir=None, flag_values={})
  monkeypatch.setattr(RG, 'arm', arm)
  monkeypatch.setattr(RG, 'closed_jobs', lambda spec, cells_only=None: [dict(kind='closed', case='s20', cell='L42', start='v12', mode='drv')])
  monkeypatch.setattr(RG, 'judge_jobs', lambda stages: {s: [dict(kind='judge', grid=s + '_g')] for s in stages})
  (sim / 'arms' / 'e').mkdir(parents=True)
  _dump(sim / 'arms' / 'e' / 'closed.pkl', [_row('s20', 4.0)])
  ran = []
  monkeypatch.setattr(RG, 'run_worker', lambda a, jobs, log, module='worker': ran.append((module, a['key'], jobs[0].get('grid'))) or (0.0, 0))
  return sim, ran


def test_info_stages_run_in_their_own_arms_and_never_enter_the_required_job_set(fake_runner):
  sim, ran = fake_runner
  assert RG.main(['--quick', '--label', 'q']) == 0   # the judge workers wrote no rows: listed as not run, no INCOMPLETE
  res = json.loads(next((sim / 'runs').glob('*_q/gate.json')).read_text())
  assert res['judges']['not_run'] == ['stress_g', 'adv_g'] and not res['missing'] and res['meta']['judge_arms'] == dict(HEAD='j', ON='j')
  assert ('judges', 'j', 'stress_g') in ran and ('judges', 'j', 'adv_g') in ran and all(mod == 'worker' for mod, k, _ in ran if k != 'j')
  ran.clear()
  assert RG.main(['--quick', '--stages', 'standard', '--label', 'n']) == 0   # deselected: not computed, not reported
  res = json.loads(next((sim / 'runs').glob('*_n/gate.json')).read_text())
  assert 'judges' not in res and not [r for r in ran if r[0] == 'judges'] and 'launch' in res


def test_wait_queues_behind_a_running_sweep(fake_runner, tmp_path):
  import fcntl
  import threading
  sim, _ = fake_runner
  with open(tmp_path / 'lock', 'w') as held:
    fcntl.flock(held, fcntl.LOCK_EX | fcntl.LOCK_NB)   # another sweep
    assert RG.main(['--quick', '--label', 'x']) == 64
    threading.Timer(0.5, lambda: fcntl.flock(held, fcntl.LOCK_UN)).start()
    assert RG.main(['--quick', '--wait', '--label', 'w']) == 0   # runs once the other sweep releases the lock
  assert next((sim / 'runs').glob('*_w/gate.json')).is_file()


def test_rharness_felt_ignores_pairs_shorter_than_the_minimum_span():
  """cyc_1012 landing: a one-sample aEgo step over a 4 ms carState gap at the mask start must not set felt (was 5.0 here)."""
  from openpilot.tools.stopping.sim import rharness as R
  t = np.concatenate([[0.0, 0.004], 0.01 * np.arange(1, 121)])
  a = -0.5 + 0.5 * t
  a[1] += 0.02
  value, t_j, t_k = R.felt(t, a, t >= 0.0)
  assert abs(value - 0.5) < 0.1 and t_k - t_j >= 0.25
  assert R.felt(t, a, t > 99.0) == (0.0, None, None)
