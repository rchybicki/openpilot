"""The source tree under test: a base commit plus an optional diff, imported through a meta-path finder.

build(base, diff) applies the diff to the base in a temporary git index (git read-tree + git apply --cached + git write-tree; the
working tree is never touched) and writes a manifest. Every production .py file (ROOTS, tests excluded) that differs between the
working tree and that tree is written to SIM_HOME/trees/<tree>/ and mapped by module name; everything else imports from the
working tree, which then has the same content. Compiled code always comes from the working-tree build, so a tree whose
compiled sources differ from the working tree is refused, and so is a diff that changes any production file other than .py.

Flag values are part of the source: arm_manifest(m, overrides) rewrites the flag definitions (`NAME = value`) in a copy of the
tree's stopping_flags.py, maps that file and records its sha1, so derived flags (GOVERNOR_BAND_PROFILE = SANTA_FE_STOP_LINE) follow
the override exactly as on the car. Nothing sets module attributes at run time.

A process selects its tree with the STOP_SIM_TREE environment variable (the arm manifest path) and must call install_from_env()
before it imports any openpilot production module (one tree per process tree; spawn workers inherit the variable).
"""
import ast
import hashlib
import importlib.abc
import importlib.util
import json
import os
import subprocess
import sys
import tempfile
from pathlib import Path

from openpilot.tools.stopping.sim import SIM_HOME

REPO = Path(__file__).resolve().parents[3]
ROOTS = ('selfdrive', 'frogpilot', 'common', 'system', 'cereal', 'opendbc_repo')
COMPILED = ('.c', '.cc', '.cpp', '.h', '.hpp', '.pyx', '.pxd', '.capnp', '.dbc', '.so')
ENV = 'STOP_SIM_TREE'
FLAGS_FILE = 'selfdrive/controls/lib/stopping_flags.py'
FLAGS_MODULE = 'openpilot.selfdrive.controls.lib.stopping_flags'
PINNED: dict[str, str | None] = {}   # module name -> file in the tree snapshot
MANIFEST: dict = {}


def git(*args, env=None, inp=None):
  r = subprocess.run(['git', '-C', str(REPO), *args], capture_output=True, input=inp, env=env)
  if r.returncode != 0:
    raise RuntimeError(f"git {' '.join(args)}: {r.stderr.decode(errors='replace').strip()}")
  return r.stdout


def is_test(path):
  return '/tests/' in path or path.rsplit('/', 1)[-1].startswith('test_')


def module_name(path):
  if path.startswith('opendbc_repo/'):
    path = path[len('opendbc_repo/'):]
  else:
    path = 'openpilot/' + path
  name = path[:-3].replace('/', '.')
  return name[:-len('.__init__')] if name.endswith('.__init__') else name


def apply_tree(base_sha, diff):
  """Tree sha of base + diff (bytes, may be empty), built in a temporary index."""
  if not diff.strip():
    return git('rev-parse', f'{base_sha}^{{tree}}').decode().strip()
  SIM_HOME.mkdir(parents=True, exist_ok=True)
  with tempfile.TemporaryDirectory(dir=SIM_HOME) as td:   # the temporary index stays under SIM_HOME
    env = dict(os.environ, GIT_INDEX_FILE=str(Path(td) / 'index'))
    git('read-tree', base_sha, env=env)
    git('apply', '--cached', '--check', env=env, inp=diff)
    git('apply', '--cached', env=env, inp=diff)
    return git('write-tree', env=env).decode().strip()


def flag_defaults(tree):
  """The flag switches of stopping_flags.py in a tree: module-level `NAME = True|False` literals (name -> value). Derived flags
  (`NAME = OTHER`) are not switches; they follow their source through the file rewrite."""
  src = git('show', f'{tree}:{FLAGS_FILE}').decode()
  out = {}
  for n in ast.parse(src).body:
    if isinstance(n, ast.Assign) and len(n.targets) == 1 and isinstance(n.targets[0], ast.Name) and isinstance(n.value, ast.Constant) \
       and isinstance(n.value.value, bool):
      out[n.targets[0].id] = n.value.value
  return out


def flag_values(src):
  """Every module-level flag of a stopping_flags.py source, evaluated as Python (name -> value; derived flags included)."""
  ns: dict = {}
  exec(compile(src, FLAGS_FILE, 'exec'), ns)
  return {k: v for k, v in ns.items() if k.isupper() and not k.startswith('_')}


def rewrite_flags(src, overrides):
  """src with each overridden flag's single top-level definition replaced by `NAME = value` (the whole statement, so a derived
  definition can be overridden too). Raises ValueError for a name without exactly one top-level definition."""
  if not overrides:
    return src
  lines = src.splitlines(keepends=True)
  defs: dict[str, list] = {}
  for n in ast.parse(src).body:
    if isinstance(n, ast.Assign) and len(n.targets) == 1 and isinstance(n.targets[0], ast.Name):
      defs.setdefault(n.targets[0].id, []).append(n)
  for k in sorted(overrides, key=lambda k: -defs[k][0].lineno if len(defs.get(k, ())) == 1 else 0):
    if len(defs.get(k, ())) != 1:
      raise ValueError(f'flag {k}: {len(defs.get(k, ()))} top-level definitions in {FLAGS_FILE} (need exactly 1)')
    n = defs[k][0]
    assert n.end_lineno is not None
    lines[n.lineno - 1:n.end_lineno] = [f'{k} = {overrides[k]!r}  # sim override\n']
  return ''.join(lines)


def code_hash(tree):
  """sha1 over the production .py blobs of a tree (equal for two commits that differ only outside the production code)."""
  ls = git('ls-tree', '-r', tree, '--', *ROOTS).decode().splitlines()
  keep = [x for x in ls if x.endswith('.py') and not is_test(x.split('\t', 1)[1])]
  return hashlib.sha1('\n'.join(keep).encode()).hexdigest()


def build(base, diff_path=None):
  """Manifest dict for base (+ diff file). Writes the snapshot files and SIM_HOME/trees/<tree>/manifest.json."""
  base_sha = git('rev-parse', f'{base}^{{commit}}').decode().strip()
  diff = Path(diff_path).read_bytes() if diff_path else b''
  tree = apply_tree(base_sha, diff)
  touched = sorted({ln.split('\t')[-1] for ln in git('apply', '--numstat', inp=diff).decode().splitlines()}) if diff.strip() else []
  served = [p for p in touched if p.startswith(tuple(r + '/' for r in ROOTS)) and not is_test(p) and not p.endswith('.py')]
  if served:   # only .py is mapped: a production .json / .cc / ... change would be silently ignored (tooling_check2 fix 5)
    raise ValueError(f'the diff changes production files the sim cannot serve (only .py is mapped): {served[:10]}')
  root = SIM_HOME / 'trees' / tree[:12]
  changed = git('diff', '--name-only', tree, '--', *ROOTS).decode().split()
  bad = [p for p in changed if p.endswith(COMPILED) and not is_test(p)]
  if bad:
    raise RuntimeError(f'compiled sources differ between the working tree and the tree under test (rebuild needed): {bad[:10]}')
  mapped = {}
  for p in changed:
    if not p.endswith('.py') or is_test(p):
      continue
    r = subprocess.run(['git', '-C', str(REPO), 'show', f'{tree}:{p}'], capture_output=True)
    dst = root / p
    if r.returncode != 0:   # only in the working tree: the tree under test has no such module
      mapped[module_name(p)] = None
      continue
    if not dst.is_file() or dst.read_bytes() != r.stdout:
      dst.parent.mkdir(parents=True, exist_ok=True)
      tmp = dst.with_name(dst.name + f'.{os.getpid()}.tmp')
      tmp.write_bytes(r.stdout)
      tmp.replace(dst)
    mapped[module_name(p)] = dict(file=str(dst), sha1=hashlib.sha1(r.stdout).hexdigest())
  m = dict(base=base, base_sha=base_sha, diff=str(diff_path) if diff_path else None, diff_sha1=hashlib.sha1(diff).hexdigest() if diff else None,
           tree=tree, code_hash=code_hash(tree), mapped=mapped, touched=touched, flags=flag_defaults(tree), base_flags=flag_defaults(base_sha),
           flag_names=sorted(flag_values(git('show', f'{tree}:{FLAGS_FILE}').decode())))
  root.mkdir(parents=True, exist_ok=True)
  (root / 'manifest.json').write_text(json.dumps(m, indent=1, sort_keys=True))
  m['path'] = str(root / 'manifest.json')
  return m


def arm_manifest(m, overrides):
  """The manifest of one arm: the tree m with its stopping_flags.py rewritten for overrides (name -> bool), always mapped from the
  snapshot (flags_sha1 = sha1 of that file; part of the arm key). Written next to the tree manifest."""
  src = git('show', f"{m['tree']}:{FLAGS_FILE}").decode()
  unknown = sorted(k for k in overrides if k not in m['flag_names'])
  if unknown:
    raise ValueError(f"unknown flags for tree {m['tree'][:12]}: {unknown} (known: {', '.join(m['flag_names'])})")
  body = rewrite_flags(src, overrides).encode()
  sha = hashlib.sha1(body).hexdigest()
  root = SIM_HOME / 'trees' / m['tree'][:12]
  dst = root / 'flags' / sha[:12] / FLAGS_FILE
  if not dst.is_file() or dst.read_bytes() != body:
    dst.parent.mkdir(parents=True, exist_ok=True)
    tmp = dst.with_name(dst.name + f'.{os.getpid()}.tmp')
    tmp.write_bytes(body)
    tmp.replace(dst)
  a = {k: v for k, v in m.items() if k != 'path'}
  a.update(mapped=dict(m['mapped'], **{FLAGS_MODULE: dict(file=str(dst), sha1=sha)}), overrides=dict(sorted(overrides.items())), flags_sha1=sha,
           flag_values={k: repr(v) for k, v in flag_values(body.decode()).items()})
  p = root / f'manifest_{sha[:12]}.json'
  p.write_text(json.dumps(a, indent=1, sort_keys=True))
  a['path'] = str(p)
  return a


class _Finder(importlib.abc.MetaPathFinder):
  def find_spec(self, fullname, path=None, target=None):
    if fullname not in PINNED:
      return None
    f = PINNED[fullname]
    if f is None:
      raise ModuleNotFoundError(f'{fullname} does not exist in the tree under test')
    return importlib.util.spec_from_file_location(fullname, f, submodule_search_locations=[str(Path(f).parent)] if f.endswith('__init__.py') else None)


def install(manifest):
  """Idempotent per process. Refuses a mapped module that is already imported from elsewhere."""
  if MANIFEST:
    assert (MANIFEST['tree'], MANIFEST.get('flags_sha1')) == (manifest['tree'], manifest.get('flags_sha1')), \
      ('one tree and flag file per process', MANIFEST['tree'], manifest['tree'])
    return PINNED
  for name, ent in manifest['mapped'].items():
    f = ent['file'] if ent else None
    mod = sys.modules.get(name)
    if mod is not None and getattr(mod, '__file__', None) != f:
      raise RuntimeError(f'loader: {name} was imported before the tree was installed')
    PINNED[name] = f
  MANIFEST.update(manifest)
  sys.meta_path.insert(0, _Finder())
  return PINNED


def install_from_env():
  p = os.environ.get(ENV)
  if not p:
    return PINNED
  return install(json.loads(Path(p).read_text()))


def check():
  """Every mapped module that is imported comes from its snapshot file with the recorded sha1."""
  for name, ent in MANIFEST.get('mapped', {}).items():
    mod = sys.modules.get(name)
    if mod is None or ent is None:
      continue
    assert os.path.realpath(mod.__file__) == os.path.realpath(ent['file']), (name, mod.__file__)
    assert hashlib.sha1(Path(mod.__file__).read_bytes()).hexdigest() == ent['sha1'], f'{name}: snapshot file changed'
