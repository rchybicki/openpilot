"""The source tree under test: a base commit plus an optional diff, imported through a meta-path finder.

build(base, diff) applies the diff to the base in a temporary git index (git read-tree + git apply --cached + git write-tree; the
working tree is never touched) and writes a manifest. Every production .py file (ROOTS, tests excluded) that differs between the
working tree and that tree is written to SIM_HOME/trees/<tree>/ and mapped by module name; everything else imports from the
working tree, which then has the same content. Compiled code always comes from the working-tree build, so a tree whose
compiled sources differ from the working tree is refused, and so is a diff that changes any production file other than .py.
The same holds for the acados MPC solvers (SOLVER_GENERATORS): a change to a generator script must be runtime-only
(generator_change, fail closed), and the modules its code generation reads names from (whole files with their parent packages)
and third_party/acados must equal the working tree's. code_hash includes the sha1 of the solver libraries and of the libraries
they load (solver_files), so a rebuilt solver or dependency gets new arms; check() fails a job when they changed.

Flag values are part of the source: arm_manifest(m, overrides) rewrites the flag definitions (`NAME = value`) in a copy of the
tree's stopping_flags.py, maps that file and records its sha1, so derived flags (GOVERNOR_BAND_PROFILE = SANTA_FE_STOP_LINE) follow
the override exactly as on the car. Nothing sets module attributes at run time.

A process selects its tree with the STOP_SIM_TREE environment variable (the arm manifest path) and must call install_from_env()
before it imports any openpilot production module (one tree per process tree; spawn workers inherit the variable).
"""
import ast
import difflib
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
SOLVER_GENERATORS = {'selfdrive/controls/lib/longitudinal_mpc_lib/long_mpc.py': 'LongitudinalMpc',   # generator script -> runtime class
                     'selfdrive/controls/lib/lateral_mpc_lib/lat_mpc.py': 'LateralMpc'}
SOLVER_THIRD_PARTY = 'third_party/acados'   # the acados templates and headers (SConscript source_list)
DYNAMIC = {'globals', 'vars', 'exec', 'eval', 'setattr', 'delattr', '__dict__', 'modules', 'importlib', '__import__'}
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


def code_hash(tree, solver):
  """sha1 over the production .py blobs of a tree and the content of the solver libraries (solver_files: content only, so two
  checkouts with the same binaries share arms; equal for two commits that differ only outside the production code)."""
  ls = git('ls-tree', '-r', tree, '--', *ROOTS).decode().splitlines()
  keep = [x for x in ls if x.endswith('.py') and not is_test(x.split('\t', 1)[1])]
  return hashlib.sha1(('\n'.join(keep) + json.dumps(sorted(solver.values()))).encode()).hexdigest()


# Lists the dynamic libraries that loading the files in argv adds to this process (what the sim loads with the solver).
LOADED_LIBS = """import ctypes, json, sys
def images():
  if sys.platform == 'darwin':
    d = ctypes.CDLL(None)
    d._dyld_get_image_name.restype = ctypes.c_char_p
    return {d._dyld_get_image_name(i).decode() for i in range(d._dyld_image_count())}
  return {ln.split(None, 5)[5].strip() for ln in open('/proc/self/maps') if len(ln.split(None, 5)) == 6 and ln.split(None, 5)[5].startswith('/')}
before = images()
for f in sys.argv[1:]:
  ctypes.CDLL(f)
print(json.dumps(sorted(images() - before)))
"""


def solver_files():
  """Path -> sha1 of the compiled solver libraries in the working tree's c_generated_code dirs and of every library they load
  (found by loading them in a subprocess, so @rpath, versioned names and symlinks resolve as in the sim; the macOS system
  libraries are in the shared cache, not files, and drop out). Paths under REPO are relative."""
  libs = [f for g in SOLVER_GENERATORS for d in [REPO / Path(g).parent / 'c_generated_code'] for f in sorted([*d.glob('*.so'), *d.glob('*.dylib')])]
  if not libs:
    return {}
  r = subprocess.run([sys.executable, '-c', LOADED_LIBS, *map(str, libs)], capture_output=True, text=True)
  if r.returncode != 0:
    raise RuntimeError(f'loading the compiled solver libraries failed: {r.stderr.strip()[-500:]}')
  own = {os.path.realpath(f) for f in libs}
  deps = [Path(x) for x in json.loads(r.stdout) if os.path.realpath(x) not in own]
  return {str(f.relative_to(REPO) if f.is_relative_to(REPO) else f): hashlib.sha1(f.read_bytes()).hexdigest() for f in libs + deps if f.is_file()}


def module_path(name):
  """Repository path of an openpilot / opendbc / cereal module or package in the working tree (None for other packages or no such file)."""
  if name.startswith('openpilot.'):
    base = name[len('openpilot.'):].replace('.', '/')
  elif name == 'opendbc' or name.startswith('opendbc.'):
    base = 'opendbc_repo/' + name.replace('.', '/')
  elif name == 'cereal' or name.startswith('cereal.'):
    base = name.replace('.', '/')
  else:
    return None
  return next((p for p in (base + '.py', base + '/__init__.py') if (REPO / p).is_file()), None)


def module_files(name):
  """The repository files that importing module `name` executes: each parent package __init__.py and the module itself."""
  parts = name.split('.')
  return [p for i in range(1, len(parts) + 1) if (p := module_path('.'.join(parts[:i])))]


def imports(stmt, path):
  """Module names that an import statement in file `path` loads (relative imports resolved; `from M import N` gives M and M.N:
  N can be a submodule), or [] for any other statement."""
  if isinstance(stmt, ast.Import):
    return [a.name for a in stmt.names]
  if not isinstance(stmt, ast.ImportFrom):
    return []
  base = stmt.module or ''
  if stmt.level:
    pkg = module_name(path).split('.')[:None if path.endswith('__init__.py') else -1]
    base = '.'.join(pkg[:len(pkg) - stmt.level + 1] + ([stmt.module] if stmt.module else []))
  return [base] + [f'{base}.{a.name}' for a in stmt.names]


def is_literal(node):
  """A number, string, signed number or tuple of these: evaluating it runs no code and reads no name."""
  if isinstance(node, ast.UnaryOp) and isinstance(node.op, (ast.USub, ast.UAdd)):
    node = node.operand
  if isinstance(node, ast.Tuple):
    return all(is_literal(x) for x in node.elts)
  return isinstance(node, ast.Constant) and not isinstance(node.value, bytes)


def is_inert(src):
  """A module whose import runs no code: a docstring and `NAME = literal | NAME` statements only (stopping_flags.py)."""
  return all((isinstance(n, ast.Expr) and isinstance(n.value, ast.Constant)) or (isinstance(n, ast.Assign) and len(n.targets) == 1 and
             isinstance(n.targets[0], ast.Name) and (is_literal(n.value) or isinstance(n.value, ast.Name))) for n in ast.parse(src).body)


def _head(n):
  """The parts of a def or class that run when it is defined (decorators, defaults, annotations, bases)."""
  if isinstance(n, ast.ClassDef):
    return [*n.decorator_list, *n.bases, *n.keywords]
  a = n.args
  return [*n.decorator_list, *a.defaults, *(d for d in a.kw_defaults if d), *([n.returns] if n.returns else []),
          *(x.annotation for x in [*a.posonlyargs, *a.args, *a.kwonlyargs, a.vararg, a.kwarg] if x and x.annotation)]


def load_names(stmts):
  """Names that executing these module or class body statements reads: def bodies do not run, def heads and class bodies do.
  A default or annotation that is a bare name or a literal runs no code, so it counts only when the def is reached (free_names)."""
  out = set()
  for n in stmts:
    if isinstance(n, (ast.FunctionDef, ast.AsyncFunctionDef)):
      head = [h for h in _head(n) if h in n.decorator_list or not (isinstance(h, ast.Name) or is_literal(h))]
    else:
      head = _head(n) if isinstance(n, ast.ClassDef) else [n]
    out |= {x.id for h in head for x in ast.walk(h) if isinstance(x, ast.Name) and not isinstance(x.ctx, ast.Store)}
    if isinstance(n, ast.ClassDef):
      out |= load_names(n.body)
  return out


def free_names(n):
  """Module names that a def reads when it is defined or runs: its head and the names its body loads minus its own locals
  (arguments and names it binds outside nested scopes, unless declared global). A class counts every name in it."""
  if isinstance(n, ast.ClassDef):
    return {x.id for x in ast.walk(n) if isinstance(x, ast.Name)}
  scope, own = [], list(n.body)
  while own:
    x = own.pop()
    scope.append(x)
    own += [c for c in ast.iter_child_nodes(x) if not isinstance(c, (ast.FunctionDef, ast.AsyncFunctionDef, ast.Lambda, ast.ClassDef, ast.ListComp,
                                                                      ast.SetComp, ast.DictComp, ast.GeneratorExp))]
  a = n.args
  bound = {x.arg for x in [*a.posonlyargs, *a.args, *a.kwonlyargs, a.vararg, a.kwarg] if x} | \
          {x.id for x in scope if isinstance(x, ast.Name) and not isinstance(x.ctx, ast.Load)} | \
          {(al.asname or al.name).split('.')[0] for x in scope if isinstance(x, (ast.Import, ast.ImportFrom)) for al in x.names}
  bound -= {g for x in scope if isinstance(x, (ast.Global, ast.Nonlocal)) for g in x.names}
  head = {x.id for h in _head(n) for x in ast.walk(h) if isinstance(x, ast.Name)}
  return ({x.id for b in n.body for x in ast.walk(b) if isinstance(x, ast.Name)} - bound) | head


def solver_reads(body):
  """(names, defs) that importing a generator script or running its code generation reads: the load-time code (every top-level
  statement except def bodies; the `__main__` block runs the generation) and the free names of every def or class it reaches."""
  defs: dict[str, list] = {}
  for n in body:
    if isinstance(n, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)):
      defs.setdefault(n.name, []).append(n)
  names, reached = load_names(body), set()
  while todo := (names & defs.keys()) - reached:
    reached |= todo
    names |= {x for k in todo for d in defs[k] for x in free_names(d)}
  return names, reached


def _changes(old, new):
  """(old statement | None, new statement | None) per statement that differs; a def or class is paired with the same-named one
  of the same changed block."""
  out = []
  ops = difflib.SequenceMatcher(None, [ast.dump(n) for n in old], [ast.dump(n) for n in new], autojunk=False).get_opcodes()
  for tag, i1, i2, j1, j2 in ops:
    if tag == 'equal':
      continue
    rest = list(new[j1:j2])
    for o in old[i1:i2]:
      n = next((x for x in rest if type(x) is type(o) and isinstance(o, (ast.FunctionDef, ast.ClassDef)) and x.name == o.name), None)
      if n is not None:
        rest.remove(n)
      out.append((o, n))
    out += [(None, n) for n in rest]
  return out


def _body_only(o, n):
  return o is not None and n is not None and isinstance(o, (ast.FunctionDef, ast.AsyncFunctionDef)) and \
    ast.dump(ast.Module(_head(o), [])) == ast.dump(ast.Module(_head(n), [])) and ast.dump(o.args) == ast.dump(n.args)


def generator_change(path, old_src, new_src, tree):
  """None when a change to solver generator script `path` is runtime-only, else the first statement that may enter the
  compiled solver. Fail closed: a changed statement is runtime-only only when it is (a) the body of a def, or of a method of
  the runtime class, that importing the script and its code generation never reach, (b) `NAME = <number / string / tuple of
  literals>` for a NAME they never read, or (c) an import of inert modules (is_inert) binding only such names."""
  if DYNAMIC & {x.id if isinstance(x, ast.Name) else x.attr for src in (old_src, new_src) for x in ast.walk(ast.parse(src))
                if isinstance(x, (ast.Name, ast.Attribute))}:
    return 'it uses dynamic name access (globals / setattr / exec / ...), so the whole file counts as solver code'
  old, new = ast.parse(old_src).body, ast.parse(new_src).body
  (r_old, d_old), (r_new, d_new) = solver_reads(old), solver_reads(new)
  reads, reached = r_old | r_new, d_old | d_new

  def ok(x):
    if isinstance(x, ast.Assign):
      return len(x.targets) == 1 and isinstance(x.targets[0], ast.Name) and x.targets[0].id not in reads and is_literal(x.value)
    if isinstance(x, (ast.Import, ast.ImportFrom)) and not (isinstance(x, ast.ImportFrom) and x.level):
      if {(a.asname or a.name).split('.')[0] for a in x.names} & reads:
        return False
      mods = imports(x, path)
      files = {f for m in mods for f in module_files(m)} - set(module_files(module_name(path)))
      return all(module_path(m) for m in (mods if isinstance(x, ast.Import) else mods[:1])) and all(
        (r := subprocess.run(['git', '-C', str(REPO), 'show', f'{tree}:{f}'], capture_output=True)).returncode == 0 and is_inert(r.stdout.decode())
        for f in files)
    return False

  for o, n in _changes(old, new):
    if _body_only(o, n) and o.name not in reached:
      continue
    if isinstance(o, ast.ClassDef) and isinstance(n, ast.ClassDef) and o.name == SOLVER_GENERATORS[path] and o.name not in reached and \
       ast.dump(ast.Module(_head(o), [])) == ast.dump(ast.Module(_head(n), [])) and all(_body_only(a, b) for a, b in _changes(o.body, n.body)):
      continue
    if all(x is None or ok(x) for x in (o, n)):
      continue
    x = n or o
    return f"{'candidate' if n else 'working-tree'} line {x.lineno}: `{ast.unparse(x).splitlines()[0][:100]}` is not a runtime-only change"
  return None


def solver_source_changes(tree, changed):
  """Why the tree's acados solvers would differ from the working tree's build (empty when the compiled solvers serve the tree):
  a generator script change that is not runtime-only (generator_change), a change to a module whose names the import or code
  generation of a generator reads (the whole file, its parent packages and, transitively, everything they import), or a
  changed third_party/acados file."""
  why = [f'{p} (acados template / header)' for p in git('diff', '--name-only', tree, '--', SOLVER_THIRD_PARTY).decode().split()]
  deps: set[str] = set()
  for g in SOLVER_GENERATORS:
    r = subprocess.run(['git', '-C', str(REPO), 'show', f'{tree}:{g}'], capture_output=True)
    srcs = [x for x in (r.stdout.decode() if r.returncode == 0 else None, (REPO / g).read_text() if (REPO / g).is_file() else None) if x is not None]
    if g in changed:
      w = generator_change(g, srcs[1], srcs[0], tree) if len(srcs) == 2 else 'was added or removed'
      if w:
        why.append(f'{g} {w}')
    deps |= set(module_files(module_name(g))[:-1])   # the script's own packages
    for src in srcs:
      mod = ast.parse(src)
      reads = solver_reads(mod.body)[0]
      deps |= {f for x in ast.walk(mod) for m in imports(x, g) if {(a.asname or a.name).split('.')[0] for a in x.names} & reads for f in module_files(m)}
  deps = {p for p in deps if p.startswith(tuple(r + '/' for r in ROOTS))}   # third_party/acados: checked as a whole above
  seen: set[str] = set()
  while deps - seen:
    p = sorted(deps - seen)[0]
    seen.add(p)
    if p in changed:
      why.append(f'{p} (the solver code generation reads names from it; the whole file and its parent packages count)')
    deps |= {f for x in ast.walk(ast.parse((REPO / p).read_text())) for m in imports(x, p) for f in module_files(m)
             if f.startswith(tuple(r + '/' for r in ROOTS))}
  return why


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
  why = solver_source_changes(tree, changed)
  if why:   # the sim always loads the working tree's compiled solvers: a candidate solver would run with the old one
    raise ValueError('the tree under test changes the generated code of a compiled acados solver, but the sim always loads the '
                     + f'working tree\'s c_generated_code build: {"; ".join(why)}. Way out: check the candidate out in a scratch worktree, '
                     + 'build its solver there (scons -u -j2 <the MPC lib dir>; first remove the solver links of a mkwt.sh worktree) and run '
                     + 'run_gates from that worktree with --base <that commit> and no --diff (its arms are keyed by the solver binaries); '
                     + 'the HEAD arms come from a checkout built from HEAD.')
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
  solver = solver_files()
  m = dict(base=base, base_sha=base_sha, diff=str(diff_path) if diff_path else None, diff_sha1=hashlib.sha1(diff).hexdigest() if diff else None,
           tree=tree, code_hash=code_hash(tree, solver), solver=solver, mapped=mapped, touched=touched, flags=flag_defaults(tree),
           base_flags=flag_defaults(base_sha),
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
  """Every mapped module that is imported comes from its snapshot file with the recorded sha1, and the solver binaries are the ones
  in the manifest (a rebuild during a sweep fails the job instead of storing its rows under the old solver's arm)."""
  if 'solver' in MANIFEST:
    now = {p: hashlib.sha1((REPO / p).read_bytes()).hexdigest() for p in MANIFEST['solver']}
    assert now == MANIFEST['solver'], 'loader: the compiled solver or a library it loads changed since the manifest was built'
  for name, ent in MANIFEST.get('mapped', {}).items():
    mod = sys.modules.get(name)
    if mod is None or ent is None:
      continue
    assert os.path.realpath(mod.__file__) == os.path.realpath(ent['file']), (name, mod.__file__)
    assert hashlib.sha1(Path(mod.__file__).read_bytes()).hexdigest() == ent['sha1'], f'{name}: snapshot file changed'
