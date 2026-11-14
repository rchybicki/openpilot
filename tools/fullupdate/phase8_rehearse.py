#!/usr/bin/env python3
"""Host rehearsals of the phase 8 device-move procedure (docs/frogpilot_refresh_20261031/PHASE8_PROCEDURE.md).

Each rehearsal runs the procedure's own host blocks, from any git revision of the document, in zsh against a sandboxed device: a
fake root (params stores, /data/openpilot as a real git repo with an origin, /VERSION, /proc, cgroups, overlay staging), a process
table, and shims for ssh, systemctl, sudo, pgrep, ps, tmux, flock, mountpoint, umount, rm, reboot and sleep. The ssh shim runs the
remote command in the sandbox with every device path rewritten under the sandbox root; it never opens a network connection, and
the harness refuses to run when `ssh` does not resolve to the shim. A rehearsal reports SAFE or UNSAFE for one Act-on item of a
review or one defect of a device run; the revision that had it must be UNSAFE and the revised text SAFE (test_phase8_rehearse.py).

  python tools/fullupdate/phase8_rehearse.py                       # working-tree text, every item
  python tools/fullupdate/phase8_rehearse.py --rev f3a2cadec7      # the reviewed text
  python tools/fullupdate/phase8_rehearse.py --item 4 --verbose    # one item with its outputs

Needs macOS or Linux with zsh, bash >= 5 (BASH5), GNU coreutils (GNU_BIN, the directory of GNU stat/readlink/sha256sum/mktemp/df/
timeout) and git >= 2.36.
"""
import argparse
import contextlib
import hashlib
import json
import os
import re
import shutil
import signal
import subprocess
import sys
import tempfile
import textwrap
from dataclasses import dataclass, field
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
DOC = 'docs/frogpilot_refresh_20261031/PHASE8_PROCEDURE.md'
BASH5 = os.environ.get('BASH5', shutil.which('bash', path='/opt/homebrew/bin:/usr/local/bin:/usr/bin:/bin') or 'bash')
GNU_BIN = os.environ.get('GNU_BIN', '/opt/homebrew/opt/coreutils/libexec/gnubin')
DEVICE_PREFIXES = ('/data/', '/cache/', '/VERSION', '/sys/', '/proc/', '/persist/', '/tmp/', '/dev/shm/')
UNIT = 'comma.service'
OLD_REV = '6272ccdbaa'   # the device branch: its launch_env.sh and agnos.json
OLD_MAPD_BYTES = b'mapd of !my-fp-new: gomsgq v0.1.10, 15 reader slots\n'
NEW_MAPD_BYTES = b'mapd v2.4.0 with our speed logic: reads NUM_READERS from /data/openpilot/msgq_repo/msgq/msgq.h\n'
TILE = '50/16/51.000000_16.750000_51.250000_17.000000'   # Domaslawska, Wroclaw
BORDER = '44/12/45.250000_13.250000_45.500000_13.500000'  # in the main set and in the Slovenia overlay
TILE_FORMATS = ((b'v240 tile', 'v240'), (b'old-format tile', 'v206'))   # the first bytes of a stand-in tile: the format mapd sees
OLD_TILES = 'data/media/0/osm/offline'       # the old mapd's tiles (mapd ours-v206 params.GetBaseOpPath())
NEW_TILES = 'data/media/0/osm-v240/offline'  # the rebased mapd's tiles (mapd ours-v240 110ab08 params.deviceMapsPath)
MAPD_TILES = {NEW_MAPD_BYTES: ('v240', NEW_TILES), OLD_MAPD_BYTES: ('v206', OLD_TILES)}


def prerequisites() -> list[str]:
  missing = []
  if not shutil.which('zsh'):
    missing.append('zsh')
  if subprocess.run([BASH5, '-c', '[ "${BASH_VERSINFO[0]}" -ge 5 ]']).returncode:
    missing.append(f'bash >= 5 ({BASH5})')
  if not (Path(GNU_BIN) / 'stat').exists() and not sys.platform.startswith('linux'):
    missing.append(f'GNU coreutils in {GNU_BIN}')
  return missing


# ------------------------------------------------------------------ procedure text

def doc_text(rev: str | None) -> str:
  if rev is None:
    return (REPO / DOC).read_text()
  return subprocess.run(['git', '-C', str(REPO), 'show', f'{rev}:{DOC}'], capture_output=True, text=True, check=True).stdout


def marked_block(text: str, name: str) -> str:
  """The ```sh block right after `<!-- p8:<name> -->` (revised text)."""
  m = re.search(rf'<!-- p8:{re.escape(name)} -->\s*\n```(?:sh|python)\n(.*?)```', text, re.S)
  if not m:
    raise KeyError(f'no block p8:{name}')
  return m.group(1)


def block_after(text: str, anchor: str, index: int = 0, lang: str = 'sh') -> str:
  """The index-th ```<lang> block after the first line containing anchor, list indentation removed (the reviewed text)."""
  start = text.index(anchor)
  blocks = re.findall(rf'```{lang}\n(.*?)```', text[start:], re.S)
  return textwrap.dedent(blocks[index])


def block(text: str, name: str, anchor: str, index: int = 0, lang: str = 'sh') -> str:
  return marked_block(text, name) if f'<!-- p8:{name} -->' in text else block_after(text, anchor, index, lang)


# ------------------------------------------------------------------ sandbox

def run_group(cmd, *, env, cwd, timeout, stdin_text=None) -> subprocess.CompletedProcess:
  """Run cmd in its own session and kill the whole process group when it ends or times out: a stalled shim, a hung stop or a
  detached probe never outlives its rehearsal."""
  proc = subprocess.Popen(cmd, env=env, cwd=cwd, text=True, start_new_session=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                          stdin=subprocess.PIPE if stdin_text is not None else subprocess.DEVNULL)
  try:
    out, err = proc.communicate(stdin_text, timeout=timeout)
  except subprocess.TimeoutExpired:
    with contextlib.suppress(ProcessLookupError):
      os.killpg(proc.pid, signal.SIGKILL)
    proc.communicate()
    raise
  with contextlib.suppress(ProcessLookupError):
    os.killpg(proc.pid, signal.SIGKILL)
  return subprocess.CompletedProcess(cmd, proc.returncode, out, err)


SHIM_COMMON = r'''
import json, os, re, sys
SB = os.environ['P8SB']
STATE = os.path.join(SB, 'state')
def load(name, default):
  p = os.path.join(STATE, name)
  return json.load(open(p)) if os.path.exists(p) else default
def save(name, value):
  json.dump(value, open(os.path.join(STATE, name), 'w'), indent=1)
def log(*words):
  with open(os.path.join(STATE, 'calls.log'), 'a') as f:
    f.write(' '.join(words) + '\n')
def conf(key, default=None):
  return load('config.json', {}).get(key, default)
def mkproc(p, unit_cgroup=None):
  d = os.path.join(SB, 'proc', str(p['pid'])); os.makedirs(d, exist_ok=True)
  open(os.path.join(d, 'status'), 'w').write(f"Name:\t{p.get('comm', '')}\nPid:\t{p['pid']}\nPPid:\t{p.get('ppid', 1)}\n")
  open(os.path.join(d, 'cgroup'), 'w').write(f"0::{unit_cgroup or '/user.slice'}\n")
  open(os.path.join(d, 'cmdline'), 'wb').write(p['args'].replace(' ', '\0').encode() + b'\0')
  open(os.path.join(d, 'environ'), 'wb').write(b''.join(f'{k}={v}'.encode() + b'\0' for k, v in p.get('environ', {}).items()))
  if p.get('exe') and not os.path.lexists(os.path.join(d, 'exe')):
    os.symlink(SB + p['exe'], os.path.join(d, 'exe'))
'''

SHIMS = {
  'ssh': r'''
args = sys.argv[1:]
log('ssh', *args)
if conf('ssh_down'):
  sys.stderr.write('ssh: connect to host comma port 22: Operation timed out\n'); sys.exit(255)
if conf('ssh_stall'):   # connected, then nothing: only a per-call bound ends this
  import time
  time.sleep(3600)
with_value = {'-o', '-p', '-i', '-F', '-l', '-J', '-E', '-c', '-m', '-b', '-D', '-L', '-R', '-W', '-S', '-Q', '-w', '-e'}
i = 0
while i < len(args) and args[i].startswith('-'):
  i += 2 if args[i] in with_value else 1
host, words = args[i], args[i + 1:]
if not words:
  sys.stderr.write('interactive ssh is not rehearsed\n'); sys.exit(255)
cmd = ' '.join(words)
def rewrite(s):
  s = re.sub(r'(?<![\w.~-])(' + '|'.join(re.escape(p) for p in ''' + repr(DEVICE_PREFIXES) + r''') + ')', lambda m: SB + m.group(1), s)
  s = re.sub(r'(?<![\w.~/-])(/data|/cache)(?=[\s\'"]|$)', lambda m: SB + m.group(1), s, flags=re.M)
  return re.sub(r'(?<=cd )/(?=[;\s&|])|(?<=-C )/(?=\s)', SB + '/', s)
env = json.load(open(os.path.join(STATE, 'device_env.json')))
if 'P8_REHEARSAL_TIME_SCALE' in os.environ:
  env['env']['P8_REHEARSAL_TIME_SCALE'] = os.environ['P8_REHEARSAL_TIME_SCALE']
stdin = None
run = rewrite(cmd)
if cmd.startswith('bash -s'):
  stdin = rewrite(sys.stdin.read()).encode()
  # as on the device, the script has a /proc/<pid>/cmdline of its own (revision 5 at eb52f87ff7 reads it); exec keeps the PID
  run = 'p8self() { mkdir -p "$P8SB/proc/$$" && printf "%s\\0" "$@" > "$P8SB/proc/$$/cmdline"; exec "$@"; }; p8self ' + run
import subprocess
r = subprocess.run([env['BASH5'], '-c', run], input=stdin, stdin=None if stdin is not None else sys.stdin, env=env['env'], cwd=SB + '/home')
sys.exit(r.returncode)
''',
  'sudo': r'''
args = [a for a in sys.argv[1:] if a not in ('-n',)]
log('sudo', *args)
os.execvp(args[0], args)
''',
  'systemctl': r'''
args = sys.argv[1:]
log('systemctl', *args)
units = load('units.json', {})
verb = args[0]
words, skip = [], False
for a in args[1:]:   # the unit is the argument that is neither an option nor an option's value (-p Id)
  if skip:
    skip = False
  elif a == '-p':
    skip = True
  elif not a.startswith('-'):
    words.append(a)
unit = words[-1] if words else ''
if unit and '.' not in unit:
  unit += '.service'
if verb == 'stop':
  rc = conf('stop_rc', 0)
  if rc:
    sys.stderr.write(f'Failed to stop {unit}: Unit {unit} not loaded.\n'); sys.exit(rc)
  if conf('stop_hang'):
    import time
    time.sleep(3600)
  if unit in units:
    units[unit] = 'inactive'; save('units.json', units)
    if not conf('stop_leaves_processes'):
      survivors = conf('stop_survivors', [])
      procs = [p for p in load('procs.json', []) if p.get('unit') != unit] + survivors
      save('procs.json', procs)
      import shutil as sh
      for d in os.listdir(os.path.join(SB, 'proc')):
        if d.isdigit() and int(d) not in {p['pid'] for p in procs}:
          sh.rmtree(os.path.join(SB, 'proc', d))
      for root in conf('cgroup_roots', ['sys/fs/cgroup']):
        cg_dir = os.path.join(SB, root, 'system.slice', unit)
        sh.rmtree(cg_dir, ignore_errors=True)
        if survivors:   # left in a child cgroup of the unit, as a process that ignored the stop
          os.makedirs(os.path.join(cg_dir, 'payload'))
          open(os.path.join(cg_dir, 'cgroup.procs'), 'w').close()
          child = os.path.join(cg_dir, 'payload', 'cgroup.procs')
          open(child, 'w').write(''.join(f"{p['pid']}\n" for p in survivors))
          if conf('cgroup_unreadable'):
            os.chmod(child, 0)
      for p in survivors:
        mkproc(p, f'/system.slice/{unit}/payload')
  sys.exit(0)
if verb == 'start':
  units[unit] = 'active'; save('units.json', units)
  procs = load('procs.json', [])
  have = {p['pid'] for p in procs}
  for p in load('procs_on_start.json', []):
    if p.get('unit') == unit and p['pid'] not in have:
      procs.append(p)
      mkproc(p, f'/system.slice/{unit}')
  for root in conf('cgroup_roots', ['sys/fs/cgroup']):
    cg_dir = os.path.join(SB, root, 'system.slice', unit)
    os.makedirs(cg_dir, exist_ok=True)
    open(os.path.join(cg_dir, 'cgroup.procs'), 'w').write(''.join(f"{p['pid']}\n" for p in procs if p.get('unit') == unit))
  save('procs.json', procs); sys.exit(0)
if verb == 'is-active':
  s = units.get(unit, 'inactive'); print(s); sys.exit(0 if s == 'active' else 3)
if verb == 'show':
  props = []
  for i, a in enumerate(args):
    if a == '-p' and i + 1 < len(args):
      props.append(args[i + 1])
    elif a.startswith('--property='):
      props.append(a.split('=', 1)[1])
    elif a.startswith('-p') and len(a) > 2:
      props.append(a[2:])
  props = sum([p.split(',') for p in props], []) or ['']
  for p in props:
    val = {'Id': unit, 'ActiveState': units.get(unit, 'inactive'), 'Environment': conf('unit_environment', ''),
           'EnvironmentFiles': '', 'Restart': 'always', 'MainPID': '100', 'ExecStart': '{ path=/usr/comma/comma.sh }',
           'ControlGroup': f'/system.slice/{unit}' if units.get(unit) == 'active' else ''}.get(p, '')
    print(val if '--value' in args else f'{p}={val}')
  sys.exit(0)
if verb == 'cat':
  print(f'# /etc/systemd/system/{unit}'); print('[Service]'); print('ExecStart=/usr/comma/comma.sh')
  print(conf('unit_cat_extra', '')); sys.exit(0)
sys.exit(0)
''',
  'pgrep': r'''
args = sys.argv[1:]
full = exact = show = False
pat = None
for a in args:
  if a.startswith('-') and pat is None and a != '--':
    full |= 'f' in a; exact |= 'x' in a; show |= 'a' in a or 'l' in a
  elif a != '--':
    pat = a
procs = load('procs.json', [])
if conf('pgrep_sees_script'):   # as on the device: the calling device script and every shell it forked are in the process table
  import subprocess                # (real PIDs): the main shell, the shell that runs the check, and the other pipeline stages
  table = {}
  for row in subprocess.run(['/bin/ps', '-axo', 'pid=,ppid=,command='], capture_output=True, text=True).stdout.splitlines():
    f = row.split(None, 2)
    if len(f) == 3:
      table[int(f[0])] = (int(f[1]), f[2].strip())
  script = lambda command: re.match(r'(\S*/)?bash -s( |$)', command) is not None
  def chain(pid):
    out = []
    while pid in table and pid > 1 and len(out) < 40:
      out.append(pid); pid = table[pid][0]
    return out
  mine = [q for q in chain(os.getppid()) if script(table[q][1])]
  main = mine[-1] if mine else None   # the outermost shell of this script
  for q, (_, command) in sorted(table.items()):
    if main is not None and script(command) and main in chain(q):
      procs = procs + [{'pid': q, 'args': re.sub(r'^\S*/(?=bash -s)', '', command), 'comm': 'bash'}]
hits = []
for p in procs:
  if exact:
    ok = re.fullmatch(pat, p.get('comm', '')) is not None
  else:
    ok = re.search(pat, p['args'] if full else p.get('comm', '')) is not None
  if ok:
    hits.append(p)
for p in hits:
  print(f"{p['pid']} {p['args']}" if show else p['pid'])
sys.exit(0 if hits else 1)
''',
  'ps': r'''
procs = load('procs.json', [])
fmt = sys.argv[sys.argv.index('-eo') + 1] if '-eo' in sys.argv else 'pid,args'
cols = fmt.split(',')
print(' '.join(c.upper() for c in cols))
for p in procs:
  vals = {'pid': str(p['pid']), 'args': p['args'], 'etimes': str(p.get('etimes', 600)), 'comm': p.get('comm', '')}
  print(' '.join(vals[c] for c in cols))
''',
  'tmux': r'''
args = sys.argv[1:]
log('tmux', *args)
panes = conf('tmux_panes', [])
if conf('tmux_rc'):
  sys.stderr.write('no server running on /tmp/tmux-1000/default\n'); sys.exit(conf('tmux_rc'))
def fmt(f, pane):
  session, rest = pane['target'].split(':')
  window, index = rest.split('.')
  for k, v in (('#{session_name}', session), ('#{pane_pid}', str(pane['pid'])), ('#{window_index}', window), ('#{pane_index}', index), ('#S', session)):
    f = f.replace(k, v)
  return f
if args[0] in ('ls', 'list-sessions'):
  f = args[args.index('-F') + 1] if '-F' in args else '#{session_name}'
  seen = []
  for p in panes:
    s = p['target'].split(':')[0]
    if s not in seen:
      seen.append(s); print(fmt(f, p))
  sys.exit(0 if panes else 1)
if args[0] == 'list-panes':
  f = args[args.index('-F') + 1]
  for p in panes:
    print(fmt(f, p))
  sys.exit(0 if panes else 1)
if args[0] == 'capture-pane':
  t = args[args.index('-t') + 1] if '-t' in args else (panes[0]['target'] if panes else '')
  for p in panes:
    if p['target'] == t or p['target'].split(':')[0] == t or p['target'].startswith(t + ':'):
      sys.stdout.write(p['content']); sys.exit(0)
  sys.stderr.write(f"can't find pane: {t}\n"); sys.exit(1)
sys.exit(0)
''',
  'flock': r'''
import fcntl
args = sys.argv[1:]
nb = '-n' in args
rest = [a for a in args if a != '-n']
mode = fcntl.LOCK_EX | (fcntl.LOCK_NB if nb else 0)
held = conf('locks_held', [])
target = rest[0]
if target.isdigit():
  try:
    path = os.readlink(f'/dev/fd/{target}')
  except OSError:
    path = ''
  if any(path.endswith(h) for h in held):
    sys.exit(1)
  try:
    fcntl.flock(int(target), mode)
  except OSError:
    sys.exit(1)
  sys.exit(0)
if any(target.endswith(h) for h in held):
  sys.exit(1)
fd = os.open(target, os.O_CREAT | os.O_RDWR)
try:
  fcntl.flock(fd, mode)
except OSError:
  sys.exit(1)
import subprocess
sys.exit(subprocess.run(rest[1:]).returncode if len(rest) > 1 else 0)
''',
  'mountpoint': r'''
args = [a for a in sys.argv[1:] if a != '-q']
quiet = '-q' in sys.argv
path = os.path.realpath(args[0])
mounts = [os.path.realpath(os.path.join(SB, m.lstrip('/'))) for m in conf('mounts', []) + list(conf('binds', {}))]
ok = path in mounts
if not quiet:
  print(f"{args[0]} is {'a' if ok else 'not a'} mountpoint")
sys.exit(0 if ok else 32)
''',
  'umount': r'''
log('umount', *sys.argv[1:])
mounts = conf('mounts', [])
c = load('config.json', {})
target = os.path.realpath(sys.argv[-1])
c['mounts'] = [m for m in mounts if os.path.realpath(os.path.join(SB, m.lstrip('/'))) != target]
save('config.json', c)
''',
  'rm': r'''
# a bind mount (config binds: mount point -> source) as the kernel treats it: rm -r of the mount point, or of a tree that holds it,
# deletes the source's contents and then fails on the busy mount point; a path inside the mount deletes the same path in the source
import shutil, subprocess
log('rm', *sys.argv[1:])
binds = {os.path.realpath(os.path.join(SB, m.lstrip('/'))): os.path.realpath(os.path.join(SB, s.lstrip('/'))) for m, s in conf('binds', {}).items()}
busy = []
for target in [os.path.realpath(a) for a in sys.argv[1:] if not a.startswith('-')]:
  for point, source in binds.items():
    if target == point or point.startswith(target + '/'):
      for child in os.listdir(source):
        c = os.path.join(source, child)
        shutil.rmtree(c) if os.path.isdir(c) and not os.path.islink(c) else os.unlink(c)
      busy.append(point)
    elif target.startswith(point + '/'):
      inner = os.path.join(source, os.path.relpath(target, point))
      if os.path.isdir(inner) and not os.path.islink(inner):
        shutil.rmtree(inner)
      elif os.path.lexists(inner):
        os.unlink(inner)
rc = subprocess.run(['/bin/rm', *sys.argv[1:]]).returncode
for point in busy:
  os.makedirs(point, exist_ok=True)
  sys.stderr.write(f"rm: cannot remove '{point}': Device or resource busy\n")
sys.exit(1 if busy else rc)
''',
  'reboot': r'''
log('reboot')
if conf('reboot_changes_boot_id', True):
  open(os.path.join(SB, 'proc/sys/kernel/random/boot_id'), 'w').write('rebooted-0000-1111-2222-333333333333\n')
c = load('config.json', {}); c['rebooted'] = c.get('rebooted', 0) + 1; save('config.json', c)
''',
  'sleep': r'''
import time
time.sleep(min(float(sys.argv[1]) if len(sys.argv) > 1 else 0, 0.01))
''',
  'mktemp': r'''
import time
if conf('mktemp_hang') and any('/data/media' in a for a in sys.argv[1:]):
  time.sleep(30)
os.execv(os.environ['GNU_MKTEMP'], [os.environ['GNU_MKTEMP'], *sys.argv[1:]])
''',
  'df': r'''
args = sys.argv[1:]
media = conf('media_df')
if media and args and args[-1].rstrip('/').endswith('/data/media'):
  fields = [f for a in args if a.startswith('--output=') for f in a.split('=', 1)[1].split(',')] or ['size', 'avail']
  print(' '.join(f.capitalize() for f in fields))
  print(' '.join(str(media[f]) for f in fields))
  sys.exit(0)
os.execv(os.environ['GNU_DF'], [os.environ['GNU_DF'], *args])
''',
  'ss': r'''
print('State Recv-Q Send-Q Local Address:Port Peer Address:Port')
''',
  'timeout': r'''
import subprocess
args = sys.argv[1:]
i = 0
while args[i].startswith('-'):
  i += 2 if args[i] in ('-k', '-s') else 1
duration = float(args[i].rstrip('s')) * float(os.environ.get('P8_REHEARSAL_TIME_SCALE', '1'))
try:
  sys.exit(subprocess.run(args[i + 1:], timeout=duration).returncode)
except subprocess.TimeoutExpired:
  sys.exit(124)
''',
}


@dataclass
class Proc:
  pid: int
  args: str
  comm: str = ''
  ppid: int = 1
  unit: str = UNIT
  environ: dict = field(default_factory=dict)
  exe: str = ''
  etimes: int = 600


def default_procs(launcher_env=None) -> list[Proc]:
  env = {'PATH': '/usr/local/bin:/usr/bin:/bin', 'HOME': '/home/comma', 'USER': 'comma'}
  env.update(launcher_env or {})
  return [
    Proc(100, '/bin/bash /usr/comma/comma.sh', 'bash'),
    Proc(110, 'tmux new-session -s comma -d /usr/comma/comma.sh', 'tmux: server'),
    Proc(120, '/bin/bash ./launch_chffrplus.sh', 'bash', 110, environ=env),
    Proc(130, '/usr/local/venv/bin/python3 ./manager.py', 'python3', 120),
    Proc(140, 'selfdrive.pandad.pandad', 'pandad', 130),
    Proc(141, 'frogpilot.frogpilot_process', 'frogpilot_proc', 130),
    Proc(142, 'system.updated.updated', 'updated', 130),
    Proc(143, 'system.hardware.hardwared', 'hardwared', 130),
    Proc(144, './ui', 'ui', 130),
    Proc(145, './mapd', 'mapd', 130, exe='/data/openpilot/frogpilot/navigation/mapd'),
    Proc(146, 'system.athena.manage_athenad', 'manage_athenad', 130),
  ]


class Sandbox:
  def __init__(self, base: Path, launcher_env=None, config=None, procs=None):
    self.root = Path(tempfile.mkdtemp(prefix='p8sb_', dir=base))
    for p in DEVICE_PREFIXES:
      assert p not in str(self.root) + '/', f'sandbox path {self.root} contains {p}'
    r = self.root
    for d in ('state', 'shims', 'hostshims', 'home', 'tmp', 'host/P8', 'data/media/0/realdata', 'persist', 'proc/sys/kernel/random',
              'sys/firmware/devicetree/base', 'data/safe_staging/finalized', 'data/error_logs',
              'data/toggle_backups/2026-11-01_10-00-00_auto', 'data/ssh', 'dev/shm'):
      (r / d).mkdir(parents=True, exist_ok=True)
    with open(r / 'dev/shm/msgq_mapdOut', 'wb') as f:
      f.truncate(2 * 1024 * 1024 + 504)   # a mapdOut queue with 20 reader slots
    for rel, data in ((TILE, b'old-format tile, Wroclaw\n'), (BORDER, b'old-format tile, border\n'),
                      ('52/20/52.000000_21.000000_52.250000_21.250000', b'old-format tile, Warsaw\n')):
      (r / OLD_TILES / rel).parent.mkdir(parents=True, exist_ok=True)
      (r / OLD_TILES / rel).write_bytes(data)
    for rel, data in (('tiles_main/' + TILE, b'v240 tile, Wroclaw\n'), ('tiles_main/' + BORDER, b'v240 tile, border without Slovenia\n'),
                      ('tiles_si/' + BORDER, b'v240 tile, border with Slovenia\n')):
      (r / 'host' / rel).parent.mkdir(parents=True, exist_ok=True)
      (r / 'host' / rel).write_bytes(data)
    self.extra_env = {}
    self.old_tiles = tile_files(r / OLD_TILES)
    (r / 'data/toggle_backups/2026-11-01_10-00-00_auto/LaneCentering').write_text('1')
    (r / 'data/ssh/id_ed25519').write_text('stand-in key\n')
    (r / 'data/ssh').chmod(0o700)
    (r / 'data/ssh/id_ed25519').chmod(0o600)
    (r / 'data/fullupdate.log').write_text('earlier deploy output\n')
    (r / 'data/fullupdate_reboot.log').write_text('earlier supervisor output\n')
    (r / 'VERSION').write_text('12.8\n')
    (r / 'sys/firmware/devicetree/base/model').write_bytes(b'comma tici\x00')
    (r / 'proc/sys/kernel/random/boot_id').write_text('before-0000-1111-2222-333333333333\n')
    (r / 'proc/uptime').write_text('3600.00 7000.00\n')
    self.config = {'mounts': ['/data/media'], 'tmux_panes': [{'target': 'comma:0.0', 'pid': 120, 'content': 'manager started\nall good\n'}]}
    self.config.update(config or {})
    self._save('config.json', self.config)
    self._save('units.json', {UNIT: 'active'})
    self.set_procs(procs if procs is not None else default_procs(launcher_env))
    self._save('procs_on_start.json', [p.__dict__ for p in default_procs(launcher_env)])
    self._params()
    self._repo()
    self._shims()

  # state
  def _save(self, name, value):
    (self.root / 'state' / name).write_text(json.dumps(value, indent=1))

  def update_config(self, **kw):
    c = json.loads((self.root / 'state/config.json').read_text())
    c.update(kw)
    self._save('config.json', c)

  def calls(self) -> str:
    p = self.root / 'state/calls.log'
    return p.read_text() if p.exists() else ''

  def set_procs(self, procs: list[Proc]):
    self._save('procs.json', [p.__dict__ for p in procs])
    proc = self.root / 'proc'
    for d in proc.iterdir():
      if d.name.isdigit():
        shutil.rmtree(d)
    for p in procs:
      d = proc / str(p.pid)
      d.mkdir()
      (d / 'cmdline').write_bytes(p.args.replace(' ', '\0').encode() + b'\0')
      (d / 'environ').write_bytes(b''.join(f'{k}={v}'.encode() + b'\0' for k, v in p.environ.items()))
      (d / 'status').write_text(f'Name:\t{p.comm}\nPid:\t{p.pid}\nPPid:\t{p.ppid}\n')
      (d / 'cgroup').write_text(f'0::/system.slice/{p.unit}\n' if p.unit else '0::/user.slice\n')
      if p.exe:
        (d / 'exe').symlink_to(str(self.root) + p.exe)
    roots = json.loads((self.root / 'state/config.json').read_text()).get('cgroup_roots', ['sys/fs/cgroup'])
    for root in roots:
      d = self.root / root / 'system.slice' / UNIT
      d.mkdir(parents=True, exist_ok=True)
      (d / 'cgroup.procs').write_text(''.join(f'{p.pid}\n' for p in procs if p.unit == UNIT))

  def _params(self):
    for store in ('data/params', 'cache/params'):
      tmp = self.root / store / '.tmp_abc123'
      tmp.mkdir(parents=True)
      (self.root / store / 'd').symlink_to('.tmp_abc123')
      (tmp / '.lock').write_text('')
      for k, v in {'IsOnroad': '0', 'IsEngaged': '0', 'AutomaticUpdates': '1', 'LaneCentering': '1', 'LaneCenteringE2EAuthority': '0.000000',
                   'LaneCenterOffset': '0.0', 'UpdaterState': 'idle', 'ExperimentalMode': '1', 'SLCPriority1': 'Map Data',
                   'SLCPriority2': 'Dashboard', 'SLCPriority3': 'None', 'SpeedLimitFiller': '0', 'MapsSelected': '', 'MapdSettings': '{}'}.items():
        (tmp / k).write_text(v)

  def rewrite(self, s: str) -> str:
    """Device paths under the sandbox root, in one pass (a prefix inside another path, /sys/ in /proc/sys/, stays as it is)."""
    root = str(self.root)
    s = re.sub(r'(?<![\w.~-])(' + '|'.join(re.escape(p) for p in DEVICE_PREFIXES) + ')', lambda m: root + m.group(1), s)
    s = re.sub(r'(?<![\w.~/-])(/data|/cache)(?=[\s\'"]|$)', lambda m: root + m.group(1), s, flags=re.M)
    return re.sub(r'(?<=cd )/(?=[;\s&|])|(?<=-C )/(?=\s)', root + '/', s)

  def git(self, *args, cwd=None, check=True):
    env = {**os.environ, 'HOME': str(self.root / 'home'), 'GIT_CONFIG_NOSYSTEM': '1', 'GIT_AUTHOR_DATE': '2026-11-02T10:00:00',
           'GIT_COMMITTER_DATE': '2026-11-02T10:00:00'}
    return subprocess.run(['git', *args], cwd=cwd or self.root / 'data/openpilot', env=env, capture_output=True, text=True, check=check)

  def _repo(self):
    r = self.root
    (r / 'home/.gitconfig').write_text('[user]\n\tname = p8\n\temail = p8@example.invalid\n[init]\n\tdefaultBranch = main\n[advice]\n\tdetachedHead = false\n')
    wt = r / 'data/openpilot'
    wt.mkdir(parents=True)
    self.git('init', '-q')

    def real(rev, path):
      return subprocess.run(['git', '-C', str(REPO), 'show', f'{rev}:{path}'], capture_output=True, check=True).stdout

    def write_tree(files):
      for f in [p for p in wt.rglob('*') if '.git' not in p.parts and p.is_file()]:
        f.unlink()
      for path, data in files.items():
        p = wt / path
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_bytes(data)
        if path.endswith(('.sh', 'environment_variables')):
          p.chmod(0o755)
    stub = self.rewrite(textwrap.dedent('''\
      #!/usr/bin/env bash
      # rehearsal stand-in for fullupdate.sh's parked path: fetch the upstream branch, reset, delete prebuilt, reboot
      set -e
      cd /data/openpilot
      b=$(git symbolic-ref --short HEAD)
      git fetch -q origin "refs/heads/$b"
      git reset -q --hard FETCH_HEAD
      echo "HEAD is now at $(git rev-parse --short HEAD)"
      rm -f /data/openpilot/prebuilt
      sudo reboot
      ''')).encode()
    old = {'launch_env.sh': self.rewrite(real(OLD_REV, 'launch_env.sh').decode()).encode(),
           'launch_chffrplus.sh': real(OLD_REV, 'launch_chffrplus.sh'),
           'frogpilot/system/environment_variables': b'#!/bin/sh\necho "export FROGPILOT_STAND_IN=1"\n',
           'system/hardware/tici/agnos.json': real(OLD_REV, 'system/hardware/tici/agnos.json'),
           'selfdrive/modeld/models/driving_supercombo.onnx': b'old model bytes\n', 'fullupdate.sh': stub,
           'frogpilot/navigation/mapd': OLD_MAPD_BYTES, 'msgq_repo/msgq/msgq.h': b'#define NUM_READERS 15\n'}
    new = {'launch_env.sh': self.rewrite(real('HEAD', 'launch_env.sh').decode()).encode(),
           'launch_chffrplus.sh': real('HEAD', 'launch_chffrplus.sh'),
           'system/hardware/tici/agnos.json': real('HEAD', 'system/hardware/tici/agnos.json'),
           'system/hardware/tici/c3_agnos.json': real('HEAD', 'system/hardware/tici/c3_agnos.json'),
           'selfdrive/modeld/models/driving_supercombo.onnx': b'tsfdo model bytes\n', 'fullupdate.sh': stub,
           'frogpilot/navigation/mapd': NEW_MAPD_BYTES, 'msgq_repo/msgq/msgq.h': b'#define NUM_READERS 20\n'}
    write_tree(old)
    self.git('add', '-A')
    self.git('commit', '-qm', 'old tree')
    self.old_sha = self.git('rev-parse', 'HEAD').stdout.strip()
    self.git('checkout', '-q', '-b', 'cand')
    write_tree(new)
    self.git('add', '-A')
    self.git('commit', '-qm', 'candidate tree')
    self.cand_sha = self.git('rev-parse', 'HEAD').stdout.strip()
    self.git('checkout', '-q', '-B', '!my-fp-new', self.old_sha)
    self.git('branch', '-q', '-D', 'cand')
    remote = r / 'remote.git'
    subprocess.run(['git', 'init', '-q', '--bare', str(remote)], check=True, env={**os.environ, 'HOME': str(r / 'home')})
    self.git('push', '-q', str(remote), f'{self.old_sha}:refs/heads/!my-fp-new', f'{self.cand_sha}:refs/heads/!my-fp-1031')
    self.git('remote', 'add', 'origin', str(remote))
    self.git('fetch', '-q', 'origin')
    self.git('branch', '-q', '--set-upstream-to=origin/!my-fp-new', '!my-fp-new')
    for f in ('.overlay_init',):
      (wt / f).write_text('')
    (wt / 'prebuilt').write_text('')
    (r / 'data/safe_staging/finalized/.overlay_consistent').write_text('')

  def _shims(self):
    gnu_mktemp = str(Path(GNU_BIN) / 'mktemp') if (Path(GNU_BIN) / 'mktemp').exists() else shutil.which('mktemp')
    for name, body in SHIMS.items():
      for d in (['hostshims'] if name == 'ssh' else ['shims', 'hostshims'] if name in ('sleep', 'timeout') else ['shims']):
        p = self.root / d / name
        p.write_text(f'#!{sys.executable}\n' + SHIM_COMMON + body)
        p.chmod(0o755)
    path = os.pathsep.join([str(self.root / 'shims'), GNU_BIN, '/opt/homebrew/bin', '/usr/local/bin', '/usr/bin', '/bin', '/usr/sbin', '/sbin'])
    gnu_df = str(Path(GNU_BIN) / 'df') if (Path(GNU_BIN) / 'df').exists() else shutil.which('df')
    env = {'PATH': path, 'P8SB': str(self.root), 'HOME': str(self.root / 'home'), 'TMPDIR': str(self.root / 'tmp'), 'LC_ALL': 'C',
           'GIT_CONFIG_NOSYSTEM': '1', 'GNU_MKTEMP': gnu_mktemp, 'GNU_DF': gnu_df}
    self._save('device_env.json', {'BASH5': BASH5, 'env': env})

  # running
  def host(self, script: str, env=None, timeout=60) -> subprocess.CompletedProcess:
    """Run a host block in zsh, with the ssh shim first in PATH and the procedure's session variables pointing into the sandbox."""
    hostpath = os.pathsep.join([str(self.root / 'hostshims'), os.environ['PATH']])
    e = {**os.environ, 'PATH': hostpath, 'P8SB': str(self.root), 'DEV': 'comma', 'P8': str(self.root / 'host/P8'),
         'REPO': str(self.root / 'remote.git'), 'PORT': str(REPO), 'CAND': self.cand_sha, 'ORIGIN_NEW': self.old_sha,
         'DEVICE_HEAD': self.old_sha, 'OLD_SHA': self.old_sha, 'UNIT': UNIT, 'HOME': str(self.root / 'home'),
         'TARGET_SHA': self.cand_sha, 'TARGET_BRANCH': '!my-fp-1031', 'TARGET_MODE': 'switch', 'MIN_FREE': '5000000000', 'TIP': str(REPO),
         'UNIT_CGROUP': f'/system.slice/{UNIT}', 'MAPD_PROVEN': self.cand_mapd, 'CAND_MAPD': self.cand_mapd, 'OLD_MAPD': self.old_mapd,
         'TARGET_MAPD': self.cand_mapd, 'TILES_MAIN': str(self.root / 'host/tiles_main'), 'TILES_SI': str(self.root / 'host/tiles_si'),
         'GIT_CONFIG_NOSYSTEM': '1', **self.extra_env, **(env or {})}
    resolved = subprocess.run(['zsh', '-fc', 'whence -p ssh'], env=e, capture_output=True, text=True).stdout.strip()
    if resolved != str(self.root / 'hostshims/ssh'):
      raise RuntimeError(f'ssh resolves to {resolved!r}, not the sandbox shim: refusing to run')
    return run_group(['zsh', '-f', '-c', script], env=e, cwd=self.root / 'home', timeout=timeout)

  def device(self, script: str, *args, timeout=60) -> subprocess.CompletedProcess:
    """Run a device script directly in the sandbox (as `ssh $DEV bash -s -- args <<'DEV'` would)."""
    env = json.loads((self.root / 'state/device_env.json').read_text())
    return run_group([BASH5, '-s', '--', *args], stdin_text=self.rewrite(script), env=env['env'], cwd=self.root / 'home', timeout=timeout)

  def store(self, which='data') -> dict:
    d = (self.root / f'{which}/params/d').resolve()
    return {p.name: p.read_bytes() for p in sorted(d.iterdir()) if p.is_file() and p.name != '.lock'}

  def head(self) -> str:
    return self.git('rev-parse', 'HEAD').stdout.strip()

  @property
  def cand_mapd(self) -> str:
    return hashlib.sha256(NEW_MAPD_BYTES).hexdigest()

  @property
  def old_mapd(self) -> str:
    return hashlib.sha256(OLD_MAPD_BYTES).hexdigest()

  def osm(self, name='') -> Path:
    return self.root / 'data/media/0/osm' / name

  def pair(self) -> str:
    """The checked-out mapd against the bytes of every tile in the directory that binary reads (each binary has its own): 'match' when
    it reads them all, 'no map tiles' when there is none (no map data on either binary: safe, degraded), else MISMATCH."""
    tree = (self.root / 'data/openpilot/frogpilot/navigation/mapd').read_bytes()
    binary, tiles = MAPD_TILES.get(tree, ('unknown', None))
    live = sorted({next((f for prefix, f in TILE_FORMATS if data.startswith(prefix)), 'unknown')
                   for data in tile_files(self.root / tiles).values()}) if tiles else []
    state = 'no map tiles' if not live else 'match' if live == [binary] else 'MISMATCH'
    return f'{state} (mapd {binary}, its tiles {"+".join(live) or "none"})'

  def old_tiles_kept(self) -> bool:
    return tile_files(self.root / OLD_TILES) == self.old_tiles

  def overlay_would_install(self) -> bool:
    """launch_chffrplus.sh:52-58 (both branches): .overlay_init exists, .git is not newer than it, a consistent finalized overlay
    exists and no old_openpilot backup."""
    wt = self.root / 'data/openpilot'
    init = wt / '.overlay_init'
    if not init.exists():
      return False
    newer = subprocess.run(['find', str(wt / '.git'), '-newer', str(init)], capture_output=True, text=True).stdout.strip()
    return (not newer and (self.root / 'data/safe_staging/finalized/.overlay_consistent').exists()
            and not (self.root / 'data/safe_staging/old_openpilot').is_dir())

  def updated_tick(self):
    """What a still-running `updated` does after the .git change (system/updated/updated.py init_overlay, then a finalize that
    completes): it touches .overlay_init and leaves a consistent finalized overlay."""
    procs = json.loads((self.root / 'state/procs.json').read_text())
    if any('system.updated.updated' in p['args'] for p in procs):
      (self.root / 'data/safe_staging/finalized').mkdir(parents=True, exist_ok=True)
      (self.root / 'data/safe_staging/finalized/.overlay_consistent').write_text('')
      os.utime(self.root / 'data/openpilot/.git', None)
      (self.root / 'data/openpilot/.overlay_init').write_text('')
      return True
    return False


# ------------------------------------------------------------------ helpers for the rehearsals

def make_backup(sb: Sandbox, text: str):
  """Boxes 3.2 to 3.4 and 3.7 of the given text: archives on the device, copy and listing on the host."""
  for name, anchor in (('backup', '3.2 Make the archives'), ('copy', '3.3 Copy the archives'), ('reference', '3.4 Extract the reference')):
    r = sb.host(block(text, name, anchor))
    assert r.returncode == 0, (name, r.stdout, r.stderr)
  if '<!-- p8:device-listing -->' in text:
    r = sb.host(block(text, 'device-listing', '3.7 Rehearse'))
    assert r.returncode == 0 and 'DEVICE_TAR_PASS' in r.stdout, (r.stdout, r.stderr)


STAGE_STEPS = (('tiles-list', 'TILES_LIST'), ('stage-prepare', 'STAGE_READY'), ('stage-copy', 'COPIED'), ('stage-verify', 'STAGED v240'),
               ('tiles-backup', 'OLD_TILES_SAVED'))


def staged_env(sb: Sandbox) -> dict:
  """The session variables boxes 2a.1 and 2a.4 export, read back from their record files."""
  p8 = sb.root / 'host/P8'
  listing = p8 / 'tiles_v240.sha256'
  env = {'TILES_DIGEST': hashlib.sha256(listing.read_bytes()).hexdigest()} if listing.exists() else {}
  for line in (p8 / '2a_tiles_list.txt').read_text().splitlines() if (p8 / '2a_tiles_list.txt').exists() else []:
    if line.startswith('TILES_LIST '):
      env['TILES_COUNT'], env['TILES_BYTES'] = line.split()[1:3]
  olds = [line.split()[2] for line in ((p8 / '2a_stage.txt').read_text().splitlines() if (p8 / '2a_stage.txt').exists() else [])
          if line.startswith('OLD_TILES ')]
  if olds:
    env['OLD_TILES_DIGEST'] = olds[-1]
  for line in (p8 / '2a_old_tiles.txt').read_text().splitlines() if (p8 / '2a_old_tiles.txt').exists() else []:
    if line.startswith('OLD_TILES_SAVED '):
      env['OLD_TILES_COUNT'], env['OLD_TILES_DIGEST'] = line.split()[1:3]
  return env


def stage(sb: Sandbox, text: str) -> bool:
  """Section 2a of the given text (revision 7 on): the new tiles staged and, from revision 9, today's tiles copied to the host. False
  when the text has no staging."""
  if '<!-- p8:stage-prepare -->' not in text:
    return False
  for name, want in STAGE_STEPS:
    if f'<!-- p8:{name} -->' not in text:
      continue
    r = sb.host(marked_block(text, name), timeout=120)
    assert r.returncode == 0 and want in r.stdout, (name, r.stdout, r.stderr)
    sb.extra_env.update(staged_env(sb))
  return True


def install_candidate(sb: Sandbox, text: str):
  """The device after a completed switch: the tiles staged where the text stages them, the candidate checked out."""
  stage(sb, text)
  sb.git('checkout', '-q', '-B', '!my-fp-1031', sb.cand_sha)


def rollback_env(sb: Sandbox) -> dict:
  return {'TARGET_SHA': sb.old_sha, 'TARGET_BRANCH': '!my-fp-new', 'TARGET_MODE': 'rollback', 'TARGET_MAPD': sb.old_mapd}


@dataclass
class Result:
  item: int
  title: str
  verdict: str            # SAFE / UNSAFE
  evidence: list[str]
  output: str


def run_item(n: int, text: str, base: Path) -> Result:
  return ITEMS[n](text, base)


def item1(text, base) -> Result:
  """R2: a failed stop (systemctl rc 5, manager alive) must stop the restore before any delete or extract and must not start the
  unit; a successful run verifies both stores before the start."""
  ev, out = [], []
  sb = Sandbox(base)
  make_backup(sb, text)
  before = sb.store()
  # the post-switch store differs from the backup
  d = (sb.root / 'data/params/d').resolve()
  (d / 'LaneCenterOffset').write_text('9.9')
  (d / 'NewKey').write_text('x')
  changed = sb.store()
  sb.update_config(stop_rc=5)
  r = sb.host(block(text, 'r2', '### R2. Full settings restore'))
  out.append(f'[stop rc 5] rc={r.returncode}\n{r.stdout}{r.stderr}')
  mutated = sb.store() != changed
  started = 'systemctl start' in sb.calls()
  ev.append(f'stop rc 5: store mutated={mutated}, unit started={started}, rc={r.returncode}')
  unsafe = mutated or started or r.returncode == 0
  # second case: the stop succeeds but a writer stays alive
  sb2 = Sandbox(base, config={'stop_leaves_processes': True})
  make_backup(sb2, text)
  d2 = (sb2.root / 'data/params/d').resolve()
  (d2 / 'LaneCenterOffset').write_text('9.9')
  changed2 = sb2.store()
  r2 = sb2.host(block(text, 'r2', '### R2. Full settings restore'))
  out.append(f'[stop ok, manager alive] rc={r2.returncode}\n{r2.stdout}{r2.stderr}')
  mutated2 = sb2.store() != changed2
  ev.append(f'writer alive after stop: store mutated={mutated2}, unit started={"systemctl start" in sb2.calls()}, rc={r2.returncode}')
  unsafe |= mutated2 or 'systemctl start' in sb2.calls()
  # third case: everything works; both stores must equal the backup, verified before the start
  sb3 = Sandbox(base)
  make_backup(sb3, text)
  ref, ref_cache = sb3.store(), sb3.store('cache')
  d3 = (sb3.root / 'data/params/d').resolve()
  (d3 / 'LaneCenterOffset').write_text('9.9')
  (d3 / 'NewKey').write_text('x')
  (sb3.root / 'cache/params/d').resolve().joinpath('LaneCenterOffset').write_text('7.7')
  r3 = sb3.host(block(text, 'r2', '### R2. Full settings restore'))
  out.append(f'[normal] rc={r3.returncode}\n{r3.stdout}{r3.stderr}')
  calls = sb3.calls()
  ok3 = sb3.store() == ref and sb3.store('cache') == ref_cache and 'systemctl start' in calls
  verified_cache = 'CACHE_STORE_VERIFIED' in r3.stdout
  ev.append(f'normal run: data store restored={sb3.store() == ref}, cache store restored={sb3.store("cache") == ref_cache}, ' +
            f'cache verified before start={verified_cache}, started={"systemctl start" in calls}')
  unsafe |= not (ok3 and verified_cache)
  del before
  return Result(1, 'R2 restore stops on a failed stop and verifies both stores', 'UNSAFE' if unsafe else 'SAFE', ev, '\n'.join(out))


def item2(text, base) -> Result:
  """AGNOS: tici, /VERSION 12.8 and an inherited AGNOS_VERSION=16 (or one from /data/launch_env.sh) must stop the switch."""
  ev, out, unsafe = [], [], False
  cases = [('inherited AGNOS_VERSION=16', {'AGNOS_VERSION': '16'}, None),
           ('/data/launch_env.sh exports AGNOS_VERSION=16', {}, 'export WEATHER_TOKEN=secret-value\nexport AGNOS_VERSION=16\n'),
           ('clean device', {}, None)]
  for title, env, override in cases:
    sb = Sandbox(base, launcher_env=env)
    if override:
      (sb.root / 'data/launch_env.sh').write_text(override)
    truth = sb.device(textwrap.dedent('''\
      set -euo pipefail
      mapfile -d '' -t E < /proc/120/environ
      for sha in "$@"; do git -C /data/openpilot show "$sha:launch_env.sh" > /tmp/le.sh
        env -i "${E[@]}" bash -c 'source /tmp/le.sh >/dev/null 2>&1; printf "%s " "$AGNOS_VERSION"'; done'''), sb.cand_sha, sb.old_sha).stdout.split()
    flash = any(v != '12.8' for v in truth)
    if '<!-- p8:agnos -->' in text:
      r = sb.host(block(text, 'agnos', ''))
      stopped = r.returncode != 0 and 'AGNOS_CHECK STOP' in r.stdout
      leaked = 'secret-value' in r.stdout + r.stderr
      out.append(f'[{title}] effective (candidate, rollback) = {truth}\nrc={r.returncode}\n{r.stdout}{r.stderr}')
    else:
      r = sb.host(block(text, 'state', '1.3 Record the device state'))
      model_ok = re.search(r'model .*comma (tici|tizi)', r.stdout) is not None
      agnos_ok = re.search(r'^agnos 12\.8$', r.stdout, re.M) is not None
      stopped, leaked = not (model_ok and agnos_ok), False
      out.append(f'[{title}] effective (candidate, rollback) = {truth}\nbox 1.3 predicate: model ok={model_ok}, agnos 12.8={agnos_ok}\n{r.stdout}')
    ev.append(f'{title}: a boot would flash={flash}; the procedure stops={stopped}; secret printed={leaked}')
    unsafe |= (flash and not stopped) or (stopped and not flash) or leaked
  return Result(2, 'effective AGNOS target of both trees', 'UNSAFE' if unsafe else 'SAFE', ev, '\n'.join(out))


def item3(text, base) -> Result:
  """Overlay: a running `updated` that refreshes .overlay_init after the .git change must not leave an installable overlay."""
  sb = Sandbox(base)
  r0 = sb.host(block(text, 'fetch', '2.6 The device downloads the objects'))
  assert r0.returncode == 0, r0.stderr
  stage(sb, text)
  if '<!-- p8:switch -->' in text:
    r = sb.host(block(text, 'switch', ''), env={'TARGET_SHA': sb.cand_sha, 'TARGET_BRANCH': '!my-fp-1031'})
  else:
    r1 = sb.host(block(text, 'branch', '4.3 Create the local branch'))
    r = sb.host(block(text, 'switch', '4.4 Point `HEAD` at the new branch'))
    r = subprocess.CompletedProcess(r.args, r.returncode, r1.stdout + r.stdout, r1.stderr + r.stderr)
  ticked = sb.updated_tick()
  installs = sb.overlay_would_install()
  verified = 'OVERLAY_INELIGIBLE' in r.stdout
  ev = [f'switch rc={r.returncode}, HEAD=candidate {sb.head() == sb.cand_sha}, reboot called={"reboot" in sb.calls()}',
        f'updated still running after the switch (touched .overlay_init)={ticked}', f'overlay ineligible verified before reboot={verified}',
        f'boot would install the finalized overlay={installs}']
  unsafe = installs or not verified
  return Result(3, 'overlay updater quiesced and the overlay ineligible before the reboot', 'UNSAFE' if unsafe else 'SAFE', ev,
                f'rc={r.returncode}\n{r.stdout}{r.stderr}')


def item4(text, base) -> Result:
  """Rollback: offline (no fetch) to the pinned old SHA; bounded waits; the reboot proven by a changed boot_id."""
  ev, out, unsafe = [], [], False
  sb = Sandbox(base)
  r0 = sb.host(block(text, 'fetch', '2.6 The device downloads the objects'))
  assert r0.returncode == 0, r0.stderr
  # the device runs the candidate (as after a switch); the network is gone
  install_candidate(sb, text)
  shutil.rmtree(sb.root / 'remote.git')
  if '<!-- p8:switch -->' in text:
    r = sb.host(block(text, 'switch', ''), env=rollback_env(sb))
  else:
    r = sb.host(block(text, 'r1b', '**R1b, when'))
  rolled = (sb.head() == sb.old_sha and not sb.git('status', '--porcelain', '--untracked-files=no').stdout.strip()
            and (sb.root / 'data/openpilot/selfdrive/modeld/models/driving_supercombo.onnx').read_bytes() == b'old model bytes\n')
  out.append(f'[rollback, network down] rc={r.returncode}\n{r.stdout}{r.stderr}')
  ev.append(f'rollback with the network down: HEAD is the old SHA={rolled}, reboot called={"reboot" in sb.calls()}')
  unsafe |= not rolled
  # bounded wait for the device
  new = '<!-- p8:wait -->' in text
  wait = accelerate(block(text, 'wait', '4.5 Wait for the device'))
  if not new:
    wait = wait.splitlines()[0]
  sb2 = Sandbox(base, config={'ssh_down': True})
  (sb2.root / 'host/P8/boot_id_before.txt').write_text('before-0000-1111-2222-333333333333\n')
  try:
    r = sb2.host(wait, timeout=20, env=FAST)
    bounded, msg = True, f'rc={r.returncode}\n{r.stdout}{r.stderr}'
  except subprocess.TimeoutExpired:
    bounded, msg = False, 'still waiting after 20 s (killed)'
  out.append(f'[device unreachable] {msg}')
  ev.append(f'wait for an unreachable device ends by itself={bounded}' + (' and names the physical recovery' if bounded and 'physical' in msg else ''))
  unsafe |= not bounded or 'physical' not in msg
  # a reboot that did not happen: the device answers with the old boot_id
  sb3 = Sandbox(base, config={'reboot_changes_boot_id': False})
  (sb3.root / 'host/P8/boot_id_before.txt').write_text('before-0000-1111-2222-333333333333\n')
  if new:
    r = sb3.host(wait, timeout=20, env=FAST)
    detected = r.returncode != 0 and 'NOT_REBOOTED' in r.stdout
    out.append(f'[boot_id unchanged] rc={r.returncode}\n{r.stdout}{r.stderr}')
  else:
    try:
      r = sb3.host(wait, timeout=20)
      detected = False
      out.append(f'[boot_id unchanged] the reviewed text waits for any answer and accepts it: rc={r.returncode}\n{r.stdout}{r.stderr}')
    except subprocess.TimeoutExpired:
      detected = False
      out.append('[boot_id unchanged] killed after 20 s')
  ev.append(f'a reboot that did not happen is detected={detected}')
  unsafe |= not detected
  return Result(4, 'offline rollback, bounded waits, reboot proven by boot_id', 'UNSAFE' if unsafe else 'SAFE', ev, '\n'.join(out))


def item5(text, base) -> Result:
  """Storage: a mounted but dead NVMe (writes fail or hang) must stop the switch; Offroad_StorageMissing stops even if the backup has it."""
  ev, out, unsafe = [], [], False
  for title, cfg, breakit in (('writes fail', {}, True), ('writes hang', {'mktemp_hang': True}, False)):
    sb = Sandbox(base, config=cfg)
    media = sb.root / 'data/media'
    if breakit:
      media.chmod(0o555)
    try:
      if '<!-- p8:storage -->' in text:
        r = sb.host(block(text, 'storage', ''), timeout=40)
        stopped = r.returncode != 0 and 'STORAGE_CHECK STOP' in r.stdout
      else:
        r = sb.host(block(text, 'state', '1.3 Record the device state'), timeout=40)
        stopped = 'is a mountpoint' not in r.stdout
      out.append(f'[{title}] rc={r.returncode}\n{r.stdout[-600:]}{r.stderr[-300:]}')
    except subprocess.TimeoutExpired:
      stopped = False
      out.append(f'[{title}] hung past 40 s')
    finally:
      media.chmod(0o755)
    ev.append(f'NVMe mounted, {title}: the procedure stops={stopped}')
    unsafe |= not stopped
  sb = Sandbox(base)
  d = (sb.root / 'data/params/d').resolve()
  (d / 'Offroad_StorageMissing').write_text('{"text": "Storage missing"}')
  (sb.root / 'host/P8/backup/params_d').mkdir(parents=True)
  (sb.root / 'host/P8/backup/params_d/Offroad_StorageMissing').write_text('{"text": "Storage missing"}')
  r = sb.host(block(text, 'offroad-alerts', '5a.6 Offroad alerts'))
  if '<!-- p8:offroad-alerts -->' in text:
    stopped = r.returncode != 0 and 'STOP' in r.stdout
  else:
    stopped = False   # the reviewed rule: "only alerts that the backup also had" passes it
  out.append(f'[Offroad_StorageMissing in backup and now] rc={r.returncode}\n{r.stdout}{r.stderr}')
  ev.append(f'Offroad_StorageMissing present before and after: the procedure stops={stopped}')
  unsafe |= not stopped
  return Result(5, 'storage health probe and Offroad_StorageMissing', 'UNSAFE' if unsafe else 'SAFE', ev, '\n'.join(out))


def item6(text, base) -> Result:
  """Log evidence: a failed or wrong tmux capture must not pass; a process failure in the middle of a segment must be reported."""
  ev, out, unsafe = [], [], False
  for title, cfg, want in (('tmux capture fails', {'tmux_rc': 1}, 'missing'),
                           ('first session is not openpilot', {'tmux_panes': [
                             {'target': 'scratch:0.0', 'pid': 999, 'content': 'an unrelated shell\n'},
                             {'target': 'comma:0.0', 'pid': 120, 'content': 'Traceback (most recent call last):\n  File "x.py"\nValueError\n'}]},
                            'traceback')):
    sb = Sandbox(base, config=cfg)
    r = sb.host(block(text, 'logs', '5a.3 No error log and no tracebacks'))
    if want == 'missing':
      caught = r.returncode != 0 and 'LOG_EVIDENCE_MISSING' in r.stdout
    else:
      caught = re.search(r'tracebacks [1-9]', r.stdout) is not None
    out.append(f'[{title}] rc={r.returncode}\n{r.stdout[-500:]}{r.stderr[-300:]}')
    ev.append(f'{title}: caught={caught} (rc {r.returncode})')
    unsafe |= not caught
  seg = base / 'p8_segment_modeld_crash.zst'
  synthetic_segment(seg)
  r = run_a3(text, base, seg)
  caught = 'modeld' in r.stdout and '-11' in r.stdout
  out.append(f'[modeld exits -11 at t=20 s, restarts at t=21 s] rc={r.returncode}\n{r.stdout}{r.stderr[-300:]}')
  ev.append(f'A3 reports the modeld exit -11 in the middle of the segment={caught}')
  unsafe |= not caught
  return Result(6, 'log source proven and process failures accumulated', 'UNSAFE' if unsafe else 'SAFE', ev, '\n'.join(out))


def synthetic_segment(path: Path, manager=True, crash=True, pid_change=False, model=False, init_early_s=0.0, events=(), mapd=0):
  """A 60 s segment. managerState at 2 Hz (120 samples) with modeld (pid 100) and card (pid 200): crash makes modeld exit with -11
  at 20 s and run again from 21 s (the last sample is healthy); pid_change gives modeld pid 300 from 20 s on, with every sample
  healthy (a crash and restart between samples). model adds 1200 modelV2 messages (20 Hz), or that many when it is a number.
  init_early_s stamps initData that much before the segment, as a route's later segment does (initData carries the route start).
  events adds one onroadEvents message per (time s, names). mapd adds that many mapdOut messages (20 Hz) on a loaded tile at 50 kph."""
  import zstandard
  from cereal import log
  evs = []
  t0 = 1_000_000_000_000
  init = log.Event.new_message(logMonoTime=t0 - int(init_early_s * 1e9))
  init.initData.gitCommit = 'f' * 40
  init.initData.gitBranch = '!my-fp-1031'
  evs.append(init)
  if model:
    for i in range(1200 if model is True else model):
      evs.append(log.Event.new_message(logMonoTime=t0 + 1 + i * 50_000_000, modelV2={'frameId': i}))
  for when, names in events:
    e = log.Event.new_message(logMonoTime=t0 + 3 + int(when * 1e9))
    for slot, name in zip(e.init('onroadEvents', len(names)), names, strict=True):
      slot.name = name
    evs.append(e)
  for i in range(mapd):
    evs.append(log.Event.new_message(logMonoTime=t0 + 4 + i * 50_000_000, mapdOut={'tileLoaded': True, 'speedLimit': 50 / 3.6, 'roadName': 'Domaslawska'}))
  if manager:
    for i in range(120):
      m = log.Event.new_message(logMonoTime=t0 + 2 + i * 500_000_000)
      ps = m.init('managerState').init('processes', 2)
      crashed = crash and 40 <= i < 42
      ps[0].name, ps[0].shouldBeRunning, ps[0].running, ps[0].exitCode = 'modeld', True, not crashed, -11 if crashed else 0
      ps[0].pid = 300 if pid_change and i >= 40 else 100
      ps[1].name, ps[1].shouldBeRunning, ps[1].running, ps[1].exitCode, ps[1].pid = 'card', True, True, 0, 200
      evs.append(m)
  evs.sort(key=lambda e: e.logMonoTime)
  path.write_bytes(zstandard.ZstdCompressor().compress(b''.join(e.to_bytes() for e in evs)))


def run_a3(text: str, base: Path, seg: Path, segment: int = 1) -> subprocess.CompletedProcess:
  a3_path = base / 'p8_a3.py'
  a3_path.write_text(block(text, 'a3', '### A3.py', 0, 'python'))
  return subprocess.run([sys.executable, str(a3_path), str(seg), str(segment)], capture_output=True, text=True,
                        env={**os.environ, 'PYTHONPATH': f'{REPO}:{REPO}/frogpilot/third_party'})


def item7(text, base) -> Result:
  """Model digest: `"$CAND:path"` in zsh is a substitution modifier; the box must use `${CAND}:` and check the producer's status."""
  sb = Sandbox(base)
  r0 = sb.host(block(text, 'fetch', '2.6 The device downloads the objects'))
  assert r0.returncode == 0, r0.stderr
  sb.git('checkout', '-q', '-B', '!my-fp-1031', sb.cand_sha)
  r = sb.host(block(text, 'model', '5a.9 The built model'))
  empty = 'e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855' in r.stdout
  good = subprocess.run(['shasum', '-a', '256'], input=b'tsfdo model bytes\n', capture_output=True).stdout.split()[0].decode()
  matched = 'MODEL_MATCH' in r.stdout and good in r.stdout
  # a wrong candidate must fail loudly
  rbad = sb.host(block(text, 'model', '5a.9 The built model'), env={'CAND': '0' * 40})
  loud = rbad.returncode != 0 and 'MODEL_MATCH' not in rbad.stdout
  host_blocks = re.findall(r'```sh\n(.*?)```', text, re.S)
  colon = sorted({m for b in host_blocks for m in re.findall(r'\$[A-Za-z_][A-Za-z0-9_]*:[A-Za-z&]', b)})
  no_pipefail = sum(1 for b in host_blocks if not b.lstrip().startswith('setopt pipefail'))
  ev = [f'digest of the empty input printed={empty}; MODEL_MATCH with the real digest={matched}; wrong CAND fails (rc {rbad.returncode})={loud}',
        f'`$VAR:<modifier>` patterns in host blocks: {colon or "none"}', f'host blocks without `setopt pipefail` first: {no_pipefail} of {len(host_blocks)}']
  unsafe = empty or not matched or not loud or bool(colon) or no_pipefail > 0
  return Result(7, 'zsh modifiers, pipefail and checked producers', 'UNSAFE' if unsafe else 'SAFE', ev,
                f'rc={r.returncode}\n{r.stdout}{r.stderr}\n--- wrong CAND rc={rbad.returncode}\n{rbad.stdout}{rbad.stderr}')


def flow(text, base) -> Result:
  """The whole revised procedure in one sandbox, every device script once and in order: preconditions, backup, switch, reboot,
  build, verification, rollback, restores and resume. Each step must print its PASS marker. Push and route sync are left out
  (they act on remote hosts)."""
  if '<!-- p8:switch -->' not in text:
    return Result(0, 'whole-procedure flow (revised text only)', 'UNSAFE', ['the text has no p8 block markers'], '')
  sb = Sandbox(base, config={'pgrep_sees_script': True})
  p8 = sb.root / 'host/P8'
  real_home = {'HOME': str(Path.home())}
  for name, fname in (('a1', 'A1.py'), ('a2', 'A2.py'), ('a3', 'A3.py'), ('a4', 'A4.sh')):
    m = re.search(rf'<!-- p8:{name} -->\s*\n```(?:python|bash)\n(.*?)```', text, re.S)
    (p8 / fname).write_text(m.group(1))
  seg_root = sb.root / 'data/media/0/realdata'
  steps = [
    ('state', {}, ['unit comma.service', 'update_lock free', 'branch_1031 absent']),
    ('storage', {}, ['STORAGE_CHECK PASS']),
    ('offline-check', {}, ['OFFLINE_ROLLBACK READY']),
    ('fetch', {}, [f'fetched {sb.cand_sha}']),
    ('agnos', {}, ['AGNOS_CHECK PASS']),
    ('mapd-check', {}, ['MAPD_CHECK PASS']),
    ('tiles-list', {}, ['TILES_LIST 2 ']),
    ('stage-prepare', {}, ['STAGE_READY']),
    ('stage-copy', {}, ['COPIED']),
    ('stage-verify', {}, ['STAGED v240 ']),
    ('tiles-backup', {}, ['OLD_TILES_SAVED 3 ']),
    ('owners', {}, ['/data/params/d ->']),
    ('backup', {}, ['params_d.tar']),
    ('copy', {}, ['params_d.tar: OK']),
    ('reference', {}, ['params_d ']),
    ('record', real_home, ['## lane centering']),
    ('a4-run', real_home, ['REHEARSAL_PASS']),
    ('device-listing', {}, ['DEVICE_TAR_PASS']),
    ('slc-gate', {}, ['SLC_GATE PASS']),
    ('stage-verify', {}, ['STAGED v240 ']),   # box 4.3
    ('switch', {}, ['QUIESCED', 'OVERLAY_INELIGIBLE', 'SWITCH_ISSUED']),
    ('wait', {}, ['REBOOTED']),
    ('boot', {}, []),
    ('build-wait', {}, ['BUILD_DONE']),
    ('sha-check', {}, ['SHA_CHECK PASS']),
    ('processes', {}, ['run ui 1', 'off port_8082 0']),
    ('restarts', {}, ['NO_RESTARTS']),
    ('logs', {}, ['log_source tmux pane comma:0.0', 'tracebacks 0', 'go_panics 0']),
    ('telemetry', {}, ['FrogPilotTelemetryConfirmed=']),
    ('settings', real_home, ['MISSING, NOT EXPECTED (restore these): []']),
    ('offroad-alerts', {}, ['alerts 0']),
    ('flags', {}, ['/cache/use_HD absent']),
    ('updater', {}, [f'head {sb.cand_sha} remote {sb.cand_sha}', 'manager_routing']),
    ('model', {}, ['MODEL_MATCH']),
    ('mapd-run', {}, ['MAPD_RUN PASS']),
    ('slc-gate', {}, ['SLC_GATE PASS']),
    ('onroad', {}, ['loggerd_LOG_ROOT']),
    ('segments', {}, ['05b_rlog_1.zst']),
    ('decode', {}, ['PROCESS_CHECK FAIL', 'EVENTS_CHECK PASS', 'MODEL_CHECK PASS']),   # the synthetic segment has the modeld crash
    ('offroad-again', {}, ['MANAGER_UP']),
    ('r2-key', {}, ['KEY_VERIFIED LaneCenterOffset', 'RESTORED_AND_STARTED']),
    ('switch', rollback_env(sb), ['QUIESCED', 'SWITCH_ISSUED']),
    ('wait', {}, ['REBOOTED']),
    ('boot', {}, []),
    ('sha-check', {'TARGET_SHA': sb.old_sha, 'TARGET_BRANCH': '!my-fp-new'}, ['SHA_CHECK PASS']),
    ('remove-branch', {}, [sb.old_sha]),
    ('r2', {}, ['DATA_STORE_VERIFIED', 'CACHE_STORE_VERIFIED', 'RESTORED_AND_STARTED']),
    ('owners-r2', {}, ['OWNERS_MATCH']),
    ('decode-old', {'TIP': str(REPO)}, ['PROCESS_CHECK']),
    ('resume', {}, ['RESUMED']),
    ('tiles-restore', {}, ['RESTORE_READY', 'TILES_RESTORED']),
  ]
  ev, out, ok = [], [], True
  for i, (name, env, want) in enumerate(steps):
    if name == 'boot':   # what the reboot does: the unit starts again with a new boot_id
      sb.device('sudo systemctl start comma.service')
      (sb.root / 'proc/sys/kernel/random/boot_id').write_text(f'boot-{i:04d}-1111-2222-333333333333\n')
      continue
    if name == 'segments':
      for k in range(3):
        d = seg_root / f'00000400--abcdef--{k}'
        d.mkdir(parents=True, exist_ok=True)
        synthetic_segment(d / 'rlog.zst')
    if name == 'r2-key':   # a changed key to restore
      (sb.root / 'data/params/d').resolve().joinpath('LaneCenterOffset').write_text('9.9')
    if name == 'r2':
      (sb.root / 'data/params/d').resolve().joinpath('NewKeyFromTheNewBase').write_text('x')
    try:
      script = marked_block(text, name)
    except KeyError:
      ok = False
      ev.append(f'{name}: FAIL, this revision has no such block')
      continue
    r = sb.host(script, env=env, timeout=120)
    sb.extra_env.update(staged_env(sb))
    got = r.stdout + r.stderr
    missing = [w for w in want if w not in got]
    good = not missing
    ok &= good
    ev.append(f'{name}: {"PASS" if good else "FAIL missing " + repr(missing)} (rc {r.returncode})')
    out.append(f'[{name}] rc={r.returncode}\n{got[-1500:]}')
  restored = sb.store()
  listed = sorted(line.split()[1][2:] for line in (p8 / 'backup/params_d.sha256').read_text().splitlines())
  ev.append(f'after R2: the data store equals the backup listing={sorted(restored) == listed}')
  ev.append(f'after R1: the checked-out mapd and its tiles: {sb.pair()}; today\'s tiles unchanged={sb.old_tiles_kept()}')
  ok &= sb.pair().startswith('match') and sb.old_tiles_kept()
  return Result(0, 'whole-procedure flow', 'SAFE' if ok else 'UNSAFE', ev, '\n'.join(out))


FAST = {'P8_REHEARSAL_TIME_SCALE': '0.005'}   # sandbox only: the timeout shim's scale; revision 3's wait loops also read it
SPEEDUP = 0.005


def accelerate(script: str, factor: float = SPEEDUP) -> str:
  """The harness's own copy of a wait loop with its literal limits scaled (900 s become 4.5 s). The procedure text has no setting
  that changes its timing (round 3, item 2); only this extracted copy runs faster."""
  def scale(m):
    return m.group(1) + ', '.join(f'{int(v) * factor:g}' for v in m.group(2).split(', '))
  return re.sub(r'^(LIMIT, CALL, PAUSE = )(\d+, \d+, \d+)$', scale, script, flags=re.M)


def survivor(pid, args, comm):
  return {'pid': pid, 'args': args, 'comm': comm, 'ppid': 1, 'unit': UNIT, 'environ': {}}


def item8(text, base) -> Result:
  """Round 2, item 1: quiescence is the unit inactive and its whole cgroup tree empty, before any reset or restore; the stop is bounded."""
  ev, out, unsafe = [], [], False
  # a. switch: PID 777 (git reset --hard) stays in a child cgroup of comma.service after a successful stop
  sb = Sandbox(base, config={'stop_survivors': [survivor(777, 'git reset --hard', 'git')]})
  sb.host(block(text, 'fetch', '2.6 The device downloads the objects'))
  stage(sb, text)
  r = sb.host(block(text, 'switch', ''))
  reset = sb.head() == sb.cand_sha
  out.append(f'[switch, PID 777 left in the cgroup] rc={r.returncode}\n{r.stdout}{r.stderr}')
  ev.append(f'switch with PID 777 in the cgroup: reset ran={reset}, SWITCH_ISSUED={"SWITCH_ISSUED" in r.stdout}, reboot={"reboot" in sb.calls()}')
  unsafe |= reset or 'reboot' in sb.calls()
  # b. R2: ./loggerd (a params writer) survives the stop
  sb = Sandbox(base, config={'stop_survivors': [survivor(778, './loggerd', 'loggerd')]})
  make_backup(sb, text)
  d = (sb.root / 'data/params/d').resolve()
  (d / 'LaneCenterOffset').write_text('9.9')
  changed = sb.store()
  r = sb.host(block(text, 'r2', '### R2. Full settings restore'))
  mutated, started = sb.store() != changed, 'systemctl start' in sb.calls()
  out.append(f'[R2, ./loggerd survives] rc={r.returncode}\n{r.stdout}{r.stderr}')
  ev.append(f'R2 with ./loggerd alive: store changed={mutated}, unit started={started}, rc={r.returncode}')
  unsafe |= mutated or started
  # c. R2-key: PID 777 in the cgroup
  sb = Sandbox(base, config={'stop_survivors': [survivor(777, 'git reset --hard', 'git')]})
  make_backup(sb, text)
  d = (sb.root / 'data/params/d').resolve()
  (d / 'LaneCenterOffset').write_text('9.9')
  changed = sb.store()
  r = sb.host(block(text, 'r2-key', '### R2-key. Restore one key'))
  mutated, started = sb.store() != changed, 'systemctl start' in sb.calls()
  out.append(f'[R2-key, PID 777 in the cgroup] rc={r.returncode}\n{r.stdout}{r.stderr}')
  ev.append(f'R2-key with PID 777 in the cgroup: store changed={mutated}, unit started={started}, rc={r.returncode}')
  unsafe |= mutated or started
  # d. the stop itself hangs
  sb = Sandbox(base, config={'stop_hang': True})
  sb.host(block(text, 'fetch', '2.6 The device downloads the objects'))
  stage(sb, text)
  try:
    r = sb.host(block(text, 'switch', ''), env=FAST, timeout=30)
    bounded = True
    out.append(f'[systemctl stop hangs] rc={r.returncode}\n{r.stdout}{r.stderr}')
  except subprocess.TimeoutExpired:
    bounded = False
    out.append('[systemctl stop hangs] still waiting after 30 s (killed)')
  ev.append(f'a hanging systemctl stop ends by itself={bounded}, tree unchanged={sb.head() == sb.old_sha}')
  unsafe |= not bounded or sb.head() != sb.old_sha
  return Result(8, 'quiescence = unit inactive and cgroup tree empty (round 2, item 1)', 'UNSAFE' if unsafe else 'SAFE', ev, '\n'.join(out))


def item9(text, base) -> Result:
  """Round 2, item 2: a rollback must not depend on a stale UpdaterState; actual AGNOS flashing still blocks it."""
  ev, out, unsafe = [], [], False
  for title, param, extra, must_roll in (('stale UpdaterState=checking..., no updater process', 'checking...', None, True),
                                         ('stale Offroad_NeosUpdate, no flashing process', None, 'neos', True),
                                         ('AGNOS flashing process runs', None, 'agnos', False)):
    procs = [p for p in default_procs() if 'updated' not in p.args]
    if extra == 'agnos':
      procs.append(Proc(150, 'python3 system/hardware/tici/agnos.py --swap manifest.json', 'python3', 1, unit=''))
    sb = Sandbox(base, procs=procs)
    sb.host(block(text, 'fetch', '2.6 The device downloads the objects'))
    install_candidate(sb, text)
    d = (sb.root / 'data/params/d').resolve()
    if param:
      (d / 'UpdaterState').write_text(param)
    if extra == 'neos':
      (d / 'Offroad_NeosUpdate').write_text('{"text": "AGNOS update"}')
    r = sb.host(block(text, 'switch', ''), env=rollback_env(sb))
    rolled = sb.head() == sb.old_sha and not sb.git('status', '--porcelain', '--untracked-files=no').stdout.strip()
    out.append(f'[{title}] rc={r.returncode}\n{r.stdout}{r.stderr}')
    ev.append(f'{title}: rolled back={rolled} (expected {must_roll})')
    unsafe |= rolled != must_roll
  return Result(9, 'rollback independent of the progress params, AGNOS flashing still blocks (round 2, item 2)',
                'UNSAFE' if unsafe else 'SAFE', ev, '\n'.join(out))


def item10(text, base) -> Result:
  """Round 2, item 3: A3 fails on missing or thin managerState and on a PID change of a continuously required process."""
  ev, out, unsafe = [], [], False
  for title, kw, want_fail in (('1200 modelV2, no managerState', {'manager': False, 'crash': False, 'model': True}, True),
                               ('120 healthy samples, modeld pid 100 -> 300 at 20 s', {'crash': False, 'pid_change': True}, True),
                               ('120 healthy samples, no change', {'crash': False}, False)):
    seg = base / f'p8_seg_{abs(hash(title))}.zst'
    synthetic_segment(seg, **kw)
    r = run_a3(text, base, seg)
    failed = 'PROCESS_CHECK FAIL' in r.stdout
    out.append(f'[{title}] rc={r.returncode}\n{r.stdout}{r.stderr[-300:]}')
    ev.append(f'{title}: PROCESS_CHECK FAIL={failed} (expected {want_fail})')
    unsafe |= failed != want_fail
  return Result(10, 'A3 managerState coverage and PID changes (round 2, item 3)', 'UNSAFE' if unsafe else 'SAFE', ev, '\n'.join(out))


def item11(text, base) -> Result:
  """Round 2, item 4: every ssh call has a hard per-call bound and every wait loop a monotonic deadline; a connection that stalls
  after connecting must not block."""
  ev, out, unsafe = [], [], False
  for name, anchor, limit in (('wait', '4.5 Wait for the device', 15), ('build-wait', '4.6 Wait for the build', 30),
                              ('state', '1.3 Record the device state', 15)):
    try:
      script = accelerate(block(text, name, anchor))
    except (KeyError, ValueError):
      ev.append(f'{name}: this revision has no such box')
      unsafe = True
      continue
    sb = Sandbox(base, config={'ssh_stall': True})
    (sb.root / 'host/P8/boot_id_before.txt').write_text('before-0000-1111-2222-333333333333\n')
    t = __import__('time').monotonic()
    try:
      r = sb.host(script, env=FAST, timeout=limit)
      took = __import__('time').monotonic() - t
      bounded = True
      out.append(f'[{name}, ssh stalls after connecting] rc={r.returncode}, {took:.1f} s\n{r.stdout[-400:]}{r.stderr[-200:]}')
    except subprocess.TimeoutExpired:
      bounded = False
      out.append(f'[{name}, ssh stalls after connecting] still waiting after {limit} s (killed)')
    ev.append(f'{name} with a stalled connection ends by itself={bounded}')
    unsafe |= not bounded
  host_blocks = re.findall(r'```sh\n(.*?)```', text, re.S)
  bare = []
  for b in host_blocks:
    b = re.sub(r"<<'(DEV|PY)'[^\n]*\n.*?\n\1\n", '\n', b, flags=re.S)
    for line in b.splitlines():
      for m in re.finditer(r'(?<![\w\'"-])ssh (?=[-$])', line):
        before = line[:m.start()]
        if not re.search(r'timeout -k \S+ \S+ $', before) or 'ServerAliveInterval' not in line:
          bare.append(line.strip()[:90])
  python_waits = re.findall(r"<<'PY'[^\n]*\n(.*?)\nPY\n", text, re.S)
  unbounded_py = [w for w in python_waits if "'ssh'" in w and ('timeout=' not in w or 'monotonic' not in w or 'ServerAliveInterval' not in w)]
  ev.append(f'ssh calls in host blocks without `timeout -k` and ServerAliveInterval: {len(bare)}' + (f' (first: {bare[0]})' if bare else ''))
  ev.append(f'python wait loops without a per-call timeout, a monotonic deadline and ServerAliveInterval: {len(unbounded_py)}')
  unsafe |= bool(bare) or bool(unbounded_py)
  return Result(11, 'hard per-call bounds and monotonic deadlines (round 2, item 4)', 'UNSAFE' if unsafe else 'SAFE', ev, '\n'.join(out))


LAYOUTS = {'v2': ['sys/fs/cgroup'], 'v1': ['sys/fs/cgroup/systemd'], 'hybrid': ['sys/fs/cgroup/unified', 'sys/fs/cgroup/systemd']}


def item12(text, base) -> Result:
  """Round 3, item 1: an unreadable cgroup.procs (PID 777 in a child cgroup, mode 000) must count as not empty on every supported
  cgroup layout, in the switch, R2 and R2-key; a readable, empty tree still lets the switch run."""
  ev, out, unsafe = [], [], False
  for layout, roots in LAYOUTS.items():
    survivor_cfg = {'cgroup_roots': roots, 'stop_survivors': [survivor(777, 'git reset --hard', 'git')], 'cgroup_unreadable': True}
    sb = Sandbox(base, config=survivor_cfg)
    sb.host(block(text, 'fetch', '2.6 The device downloads the objects'))
    stage(sb, text)
    r = sb.host(block(text, 'switch', ''))
    bad = sb.head() != sb.old_sha or 'reboot' in sb.calls()
    out.append(f'[{layout}: switch, unreadable child cgroup.procs] rc={r.returncode}\n{r.stdout}{r.stderr[-300:]}')
    ev.append(f'{layout} switch: QUIESCED printed={"QUIESCED" in r.stdout}, reset ran={sb.head() != sb.old_sha}, reboot={"reboot" in sb.calls()}')
    unsafe |= bad
    for name, anchor in (('r2', '### R2. Full settings restore'), ('r2-key', '### R2-key. Restore one key')):
      sb = Sandbox(base, config=survivor_cfg)
      make_backup(sb, text)
      (sb.root / 'data/params/d').resolve().joinpath('LaneCenterOffset').write_text('9.9')
      changed = sb.store()
      r = sb.host(block(text, name, anchor))
      mutated, started = sb.store() != changed, 'systemctl start' in sb.calls()
      out.append(f'[{layout}: {name}, unreadable child cgroup.procs] rc={r.returncode}\n{r.stdout}{r.stderr[-300:]}')
      ev.append(f'{layout} {name}: QUIESCED printed={"QUIESCED" in r.stdout}, store changed={mutated}, unit started={started}')
      unsafe |= mutated or started
    sb = Sandbox(base, config={'cgroup_roots': roots})
    sb.host(block(text, 'fetch', '2.6 The device downloads the objects'))
    stage(sb, text)
    r = sb.host(block(text, 'switch', ''))
    ok = 'SWITCH_ISSUED' in r.stdout and sb.head() == sb.cand_sha
    out.append(f'[{layout}: switch, readable and empty] rc={r.returncode}\n{r.stdout}{r.stderr[-300:]}')
    ev.append(f'{layout} switch with a readable, empty tree: issued={ok}')
    unsafe |= not ok
  return Result(12, 'cgroup read errors count as not empty, v1/v2/hybrid (round 3, item 1)', 'UNSAFE' if unsafe else 'SAFE', ev, '\n'.join(out))


def item13(text, base) -> Result:
  """Round 3, item 2: no environment variable may change the procedure's timing. A shell that inherited
  P8_REHEARSAL_TIME_SCALE=0.005 must still wait the full 900 s (4.5) and 3600 s (4.6); the harness runs the text unaccelerated here."""
  import time
  ev, out, unsafe = [], [], False
  for name, anchor, bound in (('wait', '4.5 Wait for the device', 10), ('build-wait', '4.6 Wait for the build', 25)):
    try:
      script = block(text, name, anchor)
    except (KeyError, ValueError):
      ev.append(f'{name}: this revision has no such box')
      unsafe = True
      continue
    sb = Sandbox(base, config={'ssh_down': True})
    (sb.root / 'host/P8/boot_id_before.txt').write_text('before-0000-1111-2222-333333333333\n')
    t = time.monotonic()
    try:
      r = sb.host(script, env={'P8_REHEARSAL_TIME_SCALE': '0.005'}, timeout=bound)
      took = time.monotonic() - t
      out.append(f'[{name}, inherited P8_REHEARSAL_TIME_SCALE=0.005, device unreachable] ended after {took:.2f} s, rc={r.returncode}\n' +
                 r.stdout[-300:])
      ev.append(f'{name}: the inherited variable shortened the wait to {took:.2f} s')
      unsafe = True
    except subprocess.TimeoutExpired:
      out.append(f'[{name}, inherited P8_REHEARSAL_TIME_SCALE=0.005, device unreachable] still waiting after {bound} s (killed), as with no variable')
      ev.append(f'{name}: still waiting after {bound} s; the inherited variable has no effect')
  blocks = re.findall(r'```(?:sh|python|bash)\n(.*?)```', text, re.S)
  hits = sorted({m.group(0) for b in blocks for m in re.finditer(r'P8_REHEARSAL\w*|os\.environ\.get\([^)]*\)|os\.getenv\([^)]*\)', b)})
  ev.append(f'environment-controlled timing in the procedure blocks: {hits or "none"}')
  unsafe |= bool(hits)
  return Result(13, 'no environment-controlled timing in the operating blocks (round 3, item 2)', 'UNSAFE' if unsafe else 'SAFE', ev, '\n'.join(out))


def item14(text, base) -> Result:
  """Run of 2026-11-14, box 4.4: on the device pgrep also lists the device script itself and its subshells, and the script's
  arguments hold the cgroup path (/system.slice/comma.service), which the writers pattern matches. After a clean stop the switch,
  the rollback, R2 and R2-key must run. A real process outside the cgroup must still stop them, and so must an independent run of
  a device script, with other arguments or with exactly this script's arguments (review of revision 5: a filter by command line
  hid it)."""
  ev, out, unsafe = [], [], False
  see = {'pgrep_sees_script': True}
  fetch = block(text, 'fetch', '2.6 The device downloads the objects')
  # a. clean device: the switch, then the rollback
  sb = Sandbox(base, config=see)
  sb.host(fetch)
  stage(sb, text)
  r = sb.host(block(text, 'switch', ''))
  issued = 'SWITCH_ISSUED' in r.stdout and sb.head() == sb.cand_sha
  out.append(f'[clean device, switch] rc={r.returncode}\n{r.stdout[-600:]}{r.stderr[-300:]}')
  ev.append(f'clean device, the script visible to pgrep: switch issued={issued}')
  unsafe |= not issued
  sb.device('sudo systemctl start comma.service')
  r = sb.host(block(text, 'switch', ''), env=rollback_env(sb))
  rolled = 'SWITCH_ISSUED' in r.stdout and sb.head() == sb.old_sha
  out.append(f'[clean device, rollback] rc={r.returncode}\n{r.stdout[-600:]}{r.stderr[-300:]}')
  ev.append(f'clean device: rollback issued={rolled} (false also when the switch above did not run)')
  unsafe |= not rolled
  # b. clean device: R2 and R2-key
  for name, anchor in (('r2', '### R2. Full settings restore'), ('r2-key', '### R2-key. Restore one key')):
    sb = Sandbox(base, config=see)
    make_backup(sb, text)
    key = (sb.root / 'data/params/d').resolve() / 'LaneCenterOffset'
    before = key.read_bytes()
    key.write_text('9.9')
    r = sb.host(block(text, name, anchor))
    ok = 'RESTORED_AND_STARTED' in r.stdout and key.read_bytes() == before
    out.append(f'[clean device, {name}] rc={r.returncode}\n{r.stdout[-400:]}{r.stderr[-300:]}')
    ev.append(f'clean device: {name} restored and started={ok}')
    unsafe |= not ok
  # c. processes that are not this script still stop the switch
  args = f'bash -s -- {UNIT} /system.slice/{UNIT}'
  cases = {
    'an updater outside the cgroup': lambda sb: Proc(900, 'system.updated.updated', 'updated', 1, unit=''),
    'another device script, other arguments': lambda sb: Proc(901, f'{args} {sb.old_sha} !my-fp-new rollback', 'bash', 1, unit=''),
    'an independent run with identical arguments': lambda sb: Proc(901, f'{args} {sb.cand_sha} !my-fp-1031 switch', 'bash', 1, unit=''),
  }
  for title, extra in cases.items():
    sb = Sandbox(base, config=see)
    sb.set_procs(default_procs() + [extra(sb)])
    sb.host(fetch)
    stage(sb, text)
    r = sb.host(block(text, 'switch', ''))
    blocked = sb.head() == sb.old_sha and 'reboot' not in sb.calls()
    out.append(f'[{title}] rc={r.returncode}\n{r.stdout[-400:]}{r.stderr[-300:]}')
    ev.append(f'{title}: switch blocked={blocked}')
    unsafe |= not blocked
  # d. an independent run of the same restore, with identical arguments, still stops R2 and R2-key (a retry after a host timeout)
  for name, anchor, argv in (('r2', '### R2. Full settings restore', args), ('r2-key', '### R2-key. Restore one key', f'{args} LaneCenterOffset')):
    sb = Sandbox(base, config=see)
    make_backup(sb, text)
    sb.set_procs(default_procs() + [Proc(901, argv, 'bash', 1, unit='')])
    key = (sb.root / 'data/params/d').resolve() / 'LaneCenterOffset'
    key.write_text('9.9')
    r = sb.host(block(text, name, anchor))
    blocked = key.read_text() == '9.9' and 'systemctl start' not in sb.calls()
    out.append(f'[{name}, an independent run with identical arguments] rc={r.returncode}\n{r.stdout[-400:]}{r.stderr[-300:]}')
    ev.append(f'{name} with an independent run of identical arguments alive: blocked={blocked}')
    unsafe |= not blocked
  return Result(14, 'the process check leaves out only this script (run of 2026-11-14)', 'UNSAFE' if unsafe else 'SAFE', ev, '\n'.join(out))


def candidate_with_mapd(sb: Sandbox, data: bytes) -> str:
  """A candidate commit whose mapd is `data`, on the sandbox origin."""
  sb.git('checkout', '-q', '-B', 'p8-variant', sb.cand_sha)
  (sb.root / 'data/openpilot/frogpilot/navigation/mapd').write_bytes(data)
  sb.git('commit', '-qam', 'candidate with another mapd')
  sha = sb.head()
  sb.git('push', '-q', str(sb.root / 'remote.git'), f'{sha}:refs/heads/p8-variant')
  sb.git('checkout', '-q', '!my-fp-new')
  sb.git('branch', '-q', '-D', 'p8-variant')
  return sha


def item15(text, base) -> Result:
  """Run of 2026-11-14, fix 4: the candidate's mapd must fit the base's msgq reader slots. That day's candidate carried the 15-slot
  mapd into the 20-slot base, box 2.1 passed it, and the first boot crash-looped."""
  title = 'the candidate mapd fits NUM_READERS (run of 2026-11-14, fix 4)'
  if '<!-- p8:mapd-check -->' not in text:
    return Result(15, title, 'UNSAFE', ['no box checks the candidate mapd against NUM_READERS: box 2.1 passed the 2026-11-14 candidate'], '')
  ev, out, unsafe = [], [], False
  for case, data, want in (('the candidate as built (v2.4.0 mapd, 20 slots)', None, 'MAPD_CHECK PASS'),
                           ('the 2026-11-14 candidate (the old 15-slot mapd)', OLD_MAPD_BYTES, 'MAPD_CHECK STOP'),
                           ('a rebuilt mapd that the end-to-end proof did not run', NEW_MAPD_BYTES + b'rebuilt\n', 'MAPD_CHECK STOP')):
    sb = Sandbox(base)
    r = sb.host(marked_block(text, 'mapd-check'), env={'CAND': candidate_with_mapd(sb, data)} if data else None)
    got = want in r.stdout and ('MAPD_CHECK PASS' in r.stdout) == (want == 'MAPD_CHECK PASS')
    out.append(f'[{case}] rc={r.returncode}\n{r.stdout}{r.stderr[-300:]}')
    ev.append(f'{case}: {want}={got}')
    unsafe |= not got
  return Result(15, title, 'UNSAFE' if unsafe else 'SAFE', ev, '\n'.join(out))


def tile_files(d: Path) -> dict:
  return {p.relative_to(d).as_posix(): p.read_bytes() for p in d.rglob('*') if p.is_file() and p.name != '.p8_tiles'} if d.is_dir() else {}


def item16(text, base) -> Result:
  """Section 2a: the new tiles staged complete and verified in the directory of the rebased mapd (/data/media/0/osm-v240/offline), the
  Slovenia overlay last, only while parked and with free space on the NVMe; today's tiles never change; 2a.4 only reads, so box 4.3 can
  run it again; an incomplete or extended copy fails 2a.4, and a second run of 2a.2 to 2a.4 deletes it and copies again."""
  title = 'tile staging into the directory of the rebased mapd (section 2a)'
  if '<!-- p8:stage-prepare -->' not in text:
    return Result(16, title, 'UNSAFE', ['no staging: the new mapd boots with no tiles in its directory (no map data)'], '')
  ev, out, unsafe = [], [], False
  want = {TILE: b'v240 tile, Wroclaw\n', BORDER: b'v240 tile, border with Slovenia\n'}

  def run(sb, names, env=None):
    rs = []
    for name in names:
      r = sb.host(marked_block(text, name), env=env, timeout=120)
      sb.extra_env.update(staged_env(sb))
      out.append(f'[{name}] rc={r.returncode}\n{r.stdout[-400:]}{r.stderr[-300:]}')
      rs.append(r)
    return rs
  # a. normal, then 2a.4 again (box 4.3)
  sb = Sandbox(base)
  rs = run(sb, ['tiles-list', 'stage-prepare', 'stage-copy', 'stage-verify'])
  ok = 'STAGED v240' in rs[3].stdout and tile_files(sb.root / NEW_TILES) == want and sb.old_tiles_kept()
  ev.append(f"normal: 2 tiles in {NEW_TILES}, the Slovenia tile last, STAGED v240; today's tiles unchanged: {ok}")
  unsafe |= not ok
  if not (sb.root / NEW_TILES).is_dir():
    return Result(16, title, 'UNSAFE', ev + [f'nothing is staged in {NEW_TILES}, the directory the new mapd reads'], '\n'.join(out))
  r = run(sb, ['stage-verify'])[0]
  again = 'STAGED v240' in r.stdout and tile_files(sb.root / NEW_TILES) == want
  ev.append(f'2a.4 again (box 4.3): STAGED v240, the set unchanged: {again}')
  unsafe |= not again
  # b. stops before any copy
  nearly_full = {'media_df': {'size': 250_000_000_000, 'avail': 26_000_000_000}}
  for case, cfg, params in (('NVMe nearly full (250 GB, 26 GB free: the deleter keeps 25 GB)', nearly_full, {}),
                            ('the car is on the road', {}, {'IsOnroad': '1'})):
    sb = Sandbox(base, config=cfg)
    for k, v in params.items():
      (sb.root / 'data/params/d').resolve().joinpath(k).write_text(v)
    rs = run(sb, ['tiles-list', 'stage-prepare'])
    stopped = 'STAGE STOP' in rs[1].stdout and rs[1].returncode != 0 and not (sb.root / NEW_TILES).exists()
    ev.append(f'{case}: stopped before any copy: {stopped}')
    unsafe |= not stopped
  # c. a wrong copy fails 2a.4, and a second run of 2a.2 to 2a.4 deletes it and copies again
  sb = Sandbox(base)
  empty = sb.root / 'host/empty'
  empty.mkdir()
  run(sb, ['tiles-list', 'stage-prepare'])
  run(sb, ['stage-copy'], env={'TILES_SI': str(empty)})
  r = run(sb, ['stage-verify'])[0]
  no_overlay = 'STAGE FAIL' in r.stdout and r.returncode != 0
  ev.append(f'the Slovenia overlay not copied: STAGE FAIL: {no_overlay}')
  run(sb, ['stage-prepare', 'stage-copy'])
  (sb.root / NEW_TILES / '._50').write_bytes(b'AppleDouble\n')
  r = run(sb, ['stage-verify'])[0]
  extra = 'STAGE FAIL' in r.stdout and r.returncode != 0
  ev.append(f'an extra file in the staged set: STAGE FAIL: {extra}')
  rs = run(sb, ['stage-prepare', 'stage-copy', 'stage-verify'])
  repaired = 'STAGED v240' in rs[2].stdout and tile_files(sb.root / NEW_TILES) == want and sb.old_tiles_kept()
  ev.append(f"a second run after the failures: the earlier copy deleted, staged again, today's tiles unchanged: {repaired}")
  unsafe |= not (no_overlay and extra and repaired)
  return Result(16, title, 'UNSAFE' if unsafe else 'SAFE', ev, '\n'.join(out))

def item18(text, base) -> Result:
  """Box 3.9: nothing may replace our map limits on day one. The car's values pass; camera first, the filler, a map selection, mapd
  conditional limits or fixed mapd reader slots stop the switch."""
  title = 'speed-limit and map settings gate (box 3.9)'
  if '<!-- p8:slc-gate -->' not in text:
    return Result(18, title, 'UNSAFE', ['no gate: camera first, the filler, a map selection or mapd conditional limits reach the switch'], '')
  ev, out, unsafe = [], [], False
  car = {'SLCPriority1': 'Map Data', 'SLCPriority2': 'Dashboard', 'SLCPriority3': 'None', 'SpeedLimitFiller': '0', 'MapsSelected': '', 'MapdSettings': '{}'}
  sb = Sandbox(base)
  d = (sb.root / 'data/params/d').resolve()
  for case, keys, want_pass in (('the car on 2026-11-14', {}, True),
                                ('MapdSettings after the new manager', {'MapdSettings': '{"settings_version": 2}'}, True),
                                ('camera first', {'SLCPriority1': 'Camera'}, False), ('the filler on', {'SpeedLimitFiller': '1'}, False),
                                ('a map selection', {'MapsSelected': 'nation.PL'}, False),
                                ('mapd conditional limits',
                                 {'MapdSettings': '{"settings_version": 2, "conditional_speed_limit_control_enabled": true}'}, False),
                                ('mapd reader slots fixed at 15', {'MapdSettings': '{"msgq_queue_size": 15}'}, False),
                                ('SLCPriority3 missing (the new default is Dashboard)', {'SLCPriority3': None}, False)):
    for k, v in {**car, **keys}.items():
      (d / k).unlink(missing_ok=True)
      if v is not None:
        (d / k).write_text(v)
    r = sb.host(marked_block(text, 'slc-gate'))
    passed = r.returncode == 0 and 'SLC_GATE PASS' in r.stdout
    out.append(f'[{case}] rc={r.returncode}\n{r.stdout[-400:]}{r.stderr[-200:]}')
    ev.append(f'{case}: passed={passed} (expected {want_pass})')
    unsafe |= passed != want_pass
  return Result(18, title, 'UNSAFE' if unsafe else 'SAFE', ev, '\n'.join(out))


def item19(text, base) -> Result:
  """Box 5a.10: after the boot mapd must run the candidate binary, started with the manager and with no panic in the log (2026-11-14: a
  panic loop). Its own directory must hold every staged tile, and today's tiles must be unchanged, because the rollback needs them."""
  title = 'the running mapd and both tile directories after the boot (box 5a.10)'
  if '<!-- p8:mapd-run -->' not in text:
    return Result(19, title, 'UNSAFE', ['no check of the running mapd after the boot'], '')
  ev, out, unsafe = [], [], False
  see = {'pgrep_sees_script': True}
  panic = 'manager started\npanic: runtime error: index out of range [15] with length 15\n\ngoroutine 23 [running]:\n'
  for case, exe, etimes, pane, gone, want_pass in (("the candidate on its tiles, today's tiles unchanged", None, 600, None, None, True),
                                                   ('the old binary still runs', OLD_MAPD_BYTES, 600, None, None, False),
                                                   ('mapd restarted 20 s ago', None, 20, None, None, False),
                                                   ('a Go panic in the log', None, 600, panic, None, False),
                                                   ('a staged tile missing', None, 600, None, f'{NEW_TILES}/{TILE}', False),
                                                   ("one of today's tiles missing", None, 600, None, f'{OLD_TILES}/{BORDER}', False)):
    cfg = dict(see, **({'tmux_panes': [{'target': 'comma:0.0', 'pid': 120, 'content': pane}]} if pane else {}))
    sb = Sandbox(base, config=cfg)
    install_candidate(sb, text)
    procs = default_procs()
    for p in procs:
      if p.comm == 'mapd':
        p.etimes = etimes
    sb.set_procs(procs)
    if exe:
      (sb.root / 'old_mapd').write_bytes(exe)
      (sb.root / 'proc/145/exe').unlink()
      (sb.root / 'proc/145/exe').symlink_to(sb.root / 'old_mapd')
    if gone:
      (sb.root / gone).unlink(missing_ok=True)
    sb.host(marked_block(text, 'logs'))
    r = sb.host(marked_block(text, 'mapd-run'))
    passed = r.returncode == 0 and 'MAPD_RUN PASS' in r.stdout
    out.append(f'[{case}] rc={r.returncode}\n{r.stdout[-700:]}{r.stderr[-200:]}')
    ev.append(f'{case}: MAPD_RUN PASS={passed} (expected {want_pass})')
    unsafe |= passed != want_pass
  return Result(19, title, 'UNSAFE' if unsafe else 'SAFE', ev, '\n'.join(out))

def item20(text, base) -> Result:
  """Run of 2026-11-14, fixes 1 and 2: A3 starts a segment's span after initData (it carries the route start: segment 1 read a 61 s
  gap); segment 0 allows one start-up canError frame, and the modelV2 minimum applies to segment 1 only. It also summarises mapdOut."""
  ev, out, unsafe = [], [], False
  boot = [(1.0, ['selfdriveInitializing', 'wrongGear']), (8.4, ['commIssue']), (8.9, ['canError']), (9.0, ['wrongGear'])]
  for case, kw, segment, want in (
      ('segment 1, initData stamped 61 s early (route 000026e7)', {'crash': False, 'init_early_s': 61.3, 'model': True}, 1,
       ['PROCESS_CHECK PASS', 'MODEL_CHECK PASS']),
      ('segment 0, one canError frame at 8.9 s, 1072 modelV2 (route 000026e7)', {'crash': False, 'model': 1072, 'events': boot}, 0,
       ['PROCESS_CHECK PASS', 'EVENTS_CHECK PASS', 'MODEL_CHECK PASS']),
      ('segment 1 with the same frames', {'crash': False, 'model': 1072, 'events': boot}, 1, ['EVENTS_CHECK FAIL', 'MODEL_CHECK FAIL']),
      ('segment 0, a second canError frame at 40 s', {'crash': False, 'model': 1072, 'events': boot + [(40.0, ['canError'])]}, 0, ['EVENTS_CHECK FAIL']),
      ('segment 1, 1200 mapdOut on a loaded tile', {'crash': False, 'model': True, 'mapd': 1200}, 1, ['mapdOut 1200 messages, tileLoaded 1200'])):
    seg = base / f'p8_seg20_{abs(hash(case))}.zst'
    synthetic_segment(seg, **kw)
    r = run_a3(text, base, seg, segment)
    missing = [w for w in want if w not in r.stdout]
    out.append(f'[{case}] rc={r.returncode}\n{r.stdout}{r.stderr[-300:]}')
    ev.append(f'{case}: ' + ('as expected' if not missing else f'MISSING {missing}'))
    unsafe |= bool(missing)
  return Result(20, 'A3 span, segment event rules and mapdOut (run of 2026-11-14, fixes 1 and 2)', 'UNSAFE' if unsafe else 'SAFE', ev, '\n'.join(out))


def item21(text, base) -> Result:
  """Run of 2026-11-14, fix 3: the 3.1 owners record must survive a second run after R2, and the R2 comparison must find a changed
  mode."""
  ev, out, unsafe = [], [], False
  sb = Sandbox(base)
  p8 = sb.root / 'host/P8'
  owners = block(text, 'owners', '3.1 Record owners')
  sb.host(owners)
  record = (p8 / '03_owners.txt').read_text()
  d = (sb.root / 'data/params/d').resolve()
  for k in ('IsOnroad', 'IsEngaged'):   # after R2 the old manager deletes its CLEAR_ON keys (444 files instead of 454 on 2026-11-14)
    (d / k).unlink()
  r = sb.host(owners)   # revision 6, R2: "compare the owner and mode counts with 3.1 (run the 3.1 block)"
  kept = (p8 / '03_owners.txt').read_text() == record
  out.append(f'[3.1 block after R2] rc={r.returncode}\n{r.stdout}{r.stderr[-200:]}')
  ev.append(f'the 3.1 record after the block runs again after R2: unchanged={kept}')
  unsafe |= not kept
  if '<!-- p8:owners-r2 -->' not in text:
    ev.append('no R2 owners block: the comparison has no record of its own')
    return Result(21, 'the 3.1 owners record survives R2 (run of 2026-11-14, fix 3)', 'UNSAFE', ev, '\n'.join(out))
  r = sb.host(marked_block(text, 'owners-r2'))
  match = 'OWNERS_MATCH' in r.stdout and (p8 / '06_owners_after_r2.txt').is_file()
  out.append(f'[R2 owners block] rc={r.returncode}\n{r.stdout}{r.stderr[-200:]}')
  next(p for p in d.iterdir() if p.is_file() and p.name != '.lock').chmod(0o600)
  r = sb.host(marked_block(text, 'owners-r2'))
  differ = 'OWNERS_DIFFER' in r.stdout
  out.append(f'[R2 owners block, one mode changed] rc={r.returncode}\n{r.stdout}{r.stderr[-200:]}')
  ev.append(f'R2 owners block: OWNERS_MATCH with fewer files={match}; a changed mode shows OWNERS_DIFFER={differ}')
  unsafe |= not (match and differ)
  return Result(21, 'the 3.1 owners record survives R2 (run of 2026-11-14, fix 3)', 'UNSAFE' if unsafe else 'SAFE', ev, '\n'.join(out))


CUT = r'''p8n=0 p8re='^(quiesce$|exec [89]>|sudo (rm|umount|reboot)|rm -f|mv -T|git (branch|config|symbolic-ref|reset))'
trap '[[ $BASH_COMMAND =~ $p8re ]] && { (( p8n++ < %d )) || { echo "P8CUT before: $BASH_COMMAND"; %skill -9 $$; }; }; true' DEBUG
'''
GIT_LOCK = r'''case $BASH_COMMAND in
  "git branch"*) : > ".git/refs/heads/${branch}.lock";; "git config"*) : > .git/config.lock;;
  "git symbolic-ref"*) : > .git/HEAD.lock;; "git reset"*) : > .git/index.lock;;
esac; '''
GIT_WRITE = re.compile(r'^git (branch|config|symbolic-ref|reset)')


def cut_at(script: str, n: int, inside_git: bool = False) -> str:
  """The harness's copy of a device script that dies (kill -9, as a power cut or a lost session leaves it) just before its (n+1)-th
  command that changes the device; n = 0 cuts before the first. A failed step stops at the same places: fail() exits at once. With
  inside_git, a git write dies after it took its lock file, as one killed half way does (review of revision 8, item 3: .git/HEAD.lock)."""
  return script.replace('\nset -euo pipefail\n', '\nset -euo pipefail\n' + CUT % (n, GIT_LOCK if inside_git else ''), 1)


def item22(text, base) -> Result:
  """Review of revision 7, item 1, and of revision 8, items 2 to 5: the switch and R1, each cut before every command that changes the
  device, and each git write also cut half way (its lock file left: the reviewer's .git/HEAD.lock), then the unit started as the next
  boot starts it. That boot must find today's tiles unchanged and the checked-out mapd on its own format or on no tiles, never on the
  other format. The documented next step, R1 (once more when it stopped), must then bring the old system back: OLD_SHA on !my-fp-new, a
  clean tree, the old mapd on today's unchanged tiles."""
  title = "a stop, a power cut or a killed git command in the switch or R1 leaves today's tiles unchanged, and R1 recovers"
  ev, out, unsafe = [], [], False
  sw, fetch = block(text, 'switch', ''), block(text, 'fetch', '2.6 The device downloads the objects')

  def attempt(mode, n, inside_git):
    sb = Sandbox(base, config={'pgrep_sees_script': True})
    sb.host(fetch)
    if mode == 'rollback':
      install_candidate(sb, text)
    else:
      stage(sb, text)
    r = sb.host(cut_at(sw, n, inside_git), env=rollback_env(sb) if mode == 'rollback' else None)
    cut = re.search(r'^P8CUT before: (.*)$', r.stdout, re.M)
    sb.device('sudo systemctl start comma.service')   # the next boot (a power cut), or resume
    at_boot, kept = sb.pair(), sb.old_tiles_kept()
    r1s = []
    while len(r1s) < 2 and not (r1s and 'SWITCH_ISSUED' in r1s[-1].stdout):   # R1, and once more when it stopped
      r1s.append(sb.host(sw, env=rollback_env(sb)))
    sb.device('sudo systemctl start comma.service')   # the boot after R1
    branch = sb.git('symbolic-ref', '--short', 'HEAD', check=False).stdout.strip()
    clean = not sb.git('status', '--porcelain', '--untracked-files=no', check=False).stdout.strip()
    back = ('SWITCH_ISSUED' in r1s[-1].stdout and sb.head() == sb.old_sha and branch == '!my-fp-new' and clean and sb.pair().startswith('match')
            and sb.old_tiles_kept())
    where = f'cut {"inside" if inside_git else "before"} `{cut.group(1).replace(str(sb.root), "")}`' if cut else 'no cut'
    out.append(f'[{mode}, {where}] rc={r.returncode}\n{r.stdout[-500:]}\n' +
               ''.join(f'[R1 run {i + 1}] rc={x.returncode}\n{x.stdout[-300:]}{x.stderr[-200:]}\n' for i, x in enumerate(r1s)))
    ev.append(f"{mode}, {where}: at boot {at_boot}, today's tiles unchanged={kept}; R1 runs {len(r1s)}, old system back={back}")
    return cut, at_boot, kept, back
  for mode in ('switch', 'rollback'):
    n, counts, git_cuts = 0, {}, []
    while n < 40:
      cut, at_boot, kept, back = attempt(mode, n, False)
      state = at_boot.split(' (')[0]
      counts[state] = counts.get(state, 0) + 1
      unsafe |= state == 'MISMATCH' or not kept or not back
      if not cut:
        break
      if GIT_WRITE.match(cut.group(1)):
        git_cuts.append(n)
      n += 1
    for g in git_cuts:
      cut, at_boot, kept, back = attempt(mode, g, True)
      unsafe |= at_boot.startswith('MISMATCH') or not kept or not back
    ev.append(f'{mode}: {n} cuts, {len(git_cuts)} of them also inside the git write, and the full run; at boot {counts}')
  return Result(22, title, 'UNSAFE' if unsafe else 'SAFE', ev, '\n'.join(out))

def item25(text, base) -> Result:
  """Review of revision 7, item 4: box 3.9 must judge MapdSettings as mapd decodes it. mapd uses Go's encoding/json: a key matches a
  field without regard to case (Unicode simple folding: the long s and the Kelvin sign too), the last matching key wins, and null keeps
  the default. The hazard column applies those rules; json.Unmarshal into settings.MapdSettings (mapd ours-v240) gave the same values."""
  title = "box 3.9 reads MapdSettings keys as mapd's Go decoder does (review of revision 7, item 4)"
  ev, out, unsafe = [], [], False
  c, q = 'conditional_speed_limit_control_enabled', 'msgq_queue_size'
  sb = Sandbox(base)
  d = (sb.root / 'data/params/d').resolve()
  for case, value, hazard in (('upper-case keys (the reviewer)',
                               '{"settings_version":2,"CONDITIONAL_SPEED_LIMIT_CONTROL_ENABLED":true,"MSGQ_QUEUE_SIZE":15}', True),
                              ('the exact key false, then a mixed-case key true', f'{{"{c}":false,"Conditional_Speed_Limit_Control_Enabled":true}}', True),
                              ('a long s in the key', '{"conditional_ſpeed_limit_control_enabled":true}', True),
                              ('reader slots under a mixed-case key', '{"Msgq_Queue_Size":15}', True),
                              ('the exact key twice, the last true', f'{{"{c}":false,"{c}":true}}', True),
                              ('the exact key twice, the last false', f'{{"{c}":true,"{c}":false}}', False),
                              ('null for both keys', f'{{"{c}":null,"{q}":null}}', False),
                              ('the value after the new manager', '{"settings_version": 2}', False)):
    (d / 'MapdSettings').write_text(value, encoding='utf-8')
    r = sb.host(marked_block(text, 'slc-gate'))
    passed = r.returncode == 0 and 'SLC_GATE PASS' in r.stdout
    out.append(f'[{case}] rc={r.returncode}\n{r.stdout[-500:]}{r.stderr[-200:]}')
    ev.append(f'{case}: mapd decodes a hazard={hazard}; the gate passed={passed}')
    unsafe |= (hazard and passed) or (value == '{"settings_version": 2}' and not passed)
  return Result(25, title, 'UNSAFE' if unsafe else 'SAFE', ev, '\n'.join(out))


def item26(text, base) -> Result:
  """Review of revision 7, item 5: staging and the switch must never write through a symbolic link: not the checksum listing (linked to
  another file or to one of today's tiles), not through a link in the staging directory into today's tiles (tar), and not an update
  lock."""
  title = 'staging and the switch never write through a symbolic link (review of revision 7, item 5)'
  ev, out, unsafe = [], [], False

  def run(sb, names, env=None):
    for name in names:
      r = sb.host(marked_block(text, name), env=env, timeout=120)
      sb.extra_env.update(staged_env(sb))
      out.append(f'[{name}] rc={r.returncode}\n{r.stdout[-300:]}{r.stderr[-300:]}')
  listing = 'data/media/0/osm-v240/p8_v240.sha256'
  staging = (('the listing linked to another file (2a.3)', listing, 'data/decoy.txt'),
             ("the listing linked to one of today's tiles (2a.3)", listing, f'{OLD_TILES}/{TILE}'),
             ("a link in the staging directory into today's tiles (2a.3)", f'{NEW_TILES}/50', f'{OLD_TILES}/50'))
  for case, link, target in staging if '<!-- p8:stage-prepare -->' in text else ():
    sb = Sandbox(base)
    (sb.root / 'data/decoy.txt').write_text('another file\n')
    run(sb, ['tiles-list', 'stage-prepare'])
    (sb.root / link).parent.mkdir(parents=True, exist_ok=True)
    (sb.root / link).symlink_to(sb.root / target)
    run(sb, ['stage-copy'])
    kept = sb.old_tiles_kept() and (sb.root / 'data/decoy.txt').read_text() == 'another file\n'
    ev.append(f"{case}: today's tiles and the other file unchanged={kept}")
    unsafe |= not kept
  sb = Sandbox(base, config={'pgrep_sees_script': True})
  sb.host(block(text, 'fetch', '2.6 The device downloads the objects'))
  stage(sb, text)
  (sb.root / 'data/decoy.txt').write_text('another file\n')
  (sb.root / 'data/fullupdate.lock').symlink_to(sb.root / 'data/decoy.txt')
  r = sb.host(block(text, 'switch', ''))
  out.append(f'[the switch, /data/fullupdate.lock linked to another file] rc={r.returncode}\n{r.stdout[-400:]}')
  kept = (sb.root / 'data/decoy.txt').read_text() == 'another file\n'
  ev.append(f'the switch with /data/fullupdate.lock linked to another file: that file unchanged={kept}, rebooted={"reboot" in sb.calls()}')
  unsafe |= not kept
  return Result(26, title, 'UNSAFE' if unsafe else 'SAFE', ev, '\n'.join(out))


def item27(text, base) -> Result:
  """Review of revision 8, item 1: with today's tiles bind-mounted at the staging path, 2a.2 deleted their contents (rm -rf) and then
  failed on the mount point. Staging must refuse a mount at or below the paths it deletes, a nested one too, before any delete. The
  sandbox models a bind mount as the kernel treats it: a mount point (mountpoint shim) that shows the source's files, whose rm -r deletes
  the source's contents and then fails on the busy point, and in which a deleted path is deleted in the source (rm shim)."""
  title = 'staging never deletes through a mount (review of revision 8, item 1)'
  if '<!-- p8:stage-prepare -->' not in text:
    return Result(27, title, 'SAFE', ['the text has no staging, so nothing deletes there'], '')
  ev, out, unsafe = [], [], False
  for case, point, source in (("today's tiles bind-mounted at revision 8's staging path", 'data/media/0/osm/offline-v240', OLD_TILES),
                              ("today's tiles bind-mounted at the new tile directory", NEW_TILES, OLD_TILES),
                              ("today's osm directory bind-mounted at osm-v240", 'data/media/0/osm-v240', 'data/media/0/osm'),
                              ("today's directory 50 bind-mounted inside the new tile directory", f'{NEW_TILES}/50', f'{OLD_TILES}/50')):
    sb = Sandbox(base, config={'binds': {f'/{point}': f'/{source}'}})
    shutil.copytree(sb.root / source, sb.root / point, symlinks=True)
    for name in ('tiles-list', 'stage-prepare', 'stage-copy', 'stage-verify'):
      r = sb.host(marked_block(text, name), timeout=120)
      sb.extra_env.update(staged_env(sb))
      out.append(f'[{case}: {name}] rc={r.returncode}\n{r.stdout[-300:]}{r.stderr[-300:]}')
      if name == 'stage-prepare' and 'STAGE_READY' not in r.stdout:
        break
    kept = sb.old_tiles_kept()
    ev.append(f"{case}: today's tiles unchanged={kept}{'' if kept else ' (deleted through the mount)'}")
    unsafe |= not kept
  return Result(27, title, 'UNSAFE' if unsafe else 'SAFE', ev, '\n'.join(out))


def item28(text, base) -> Result:
  """Review of revision 8, item 6: box 3.9 passed {"settings_version":3}, with which the new mapd panics at start (interface conversion:
  interface {} is nil, not settings.MapdSettings). The hazard column is the Docker probe of the real binary (654b96d8, 2026-11-14):
  versions 0, 2.5 and 3 panic; absent, 1 and 2 run."""
  title = 'box 3.9 stops a settings_version the new mapd cannot load (review of revision 8, item 6)'
  if '<!-- p8:slc-gate -->' not in text:
    return Result(28, title, 'UNSAFE', ['no gate'], '')
  ev, out, unsafe = [], [], False
  sb = Sandbox(base)
  d = (sb.root / 'data/params/d').resolve()
  for case, value, panics in (("the reviewer's value", '{"settings_version":3}', True), ('version 0', '{"settings_version":0}', True),
                              ('version 2.5', '{"settings_version":2.5}', True), ('the car before the move', '{}', False),
                              ('the value after the new manager', '{"settings_version": 2}', False),
                              ('version 1 (mapd migrates it)', '{"settings_version":1}', False)):
    (d / 'MapdSettings').write_text(value)
    r = sb.host(marked_block(text, 'slc-gate'))
    passed = r.returncode == 0 and 'SLC_GATE PASS' in r.stdout
    out.append(f'[{case}] rc={r.returncode}\n{r.stdout[-400:]}{r.stderr[-200:]}')
    ev.append(f'{case}: mapd panics at start={panics}; the gate passed={passed}')
    unsafe |= passed == panics
  return Result(28, title, 'UNSAFE' if unsafe else 'SAFE', ev, '\n'.join(out))


def item29(text, base) -> Result:
  """Review of revision 8, item 4: the rollback depends on today's tiles, and they existed only on the device. Before the switch, box 2a.5
  must copy them to the host and check the copy against a listing made on the device, and never write that copy again; R4 must put
  them back from it, file for file, after they changed on the device; and R4 must delete nothing when the host copy no longer matches
  its record."""
  title = "today's tiles copied to the host before the switch, and R4 restores them (review of revision 8, item 4)"
  if '<!-- p8:tiles-backup -->' not in text or '<!-- p8:tiles-restore -->' not in text:
    return Result(29, title, 'UNSAFE', ["today's tiles exist only on the device: no box copies them to the host, and none restores them"], '')
  ev, out, unsafe = [], [], False
  sb = Sandbox(base, config={'pgrep_sees_script': True})
  copy = sb.root / 'host/P8/tiles_v206'

  def run(name):
    r = sb.host(marked_block(text, name), timeout=120)
    sb.extra_env.update(staged_env(sb))
    out.append(f'[{name}] rc={r.returncode}\n{r.stdout[-400:]}{r.stderr[-300:]}')
    return r
  r = run('tiles-backup')
  ok = 'OLD_TILES_SAVED 3 ' in r.stdout and tile_files(copy) == sb.old_tiles
  ev.append(f"2a.5: OLD_TILES_SAVED 3, the host copy equals today's tiles: {ok}")
  unsafe |= not ok
  (sb.root / OLD_TILES / TILE).write_bytes(b'v240 tile, Wroclaw\n')   # today's set changed on the device: a tile in the new format
  (sb.root / OLD_TILES / BORDER).unlink()                            # and a tile gone
  r = run('tiles-backup')
  kept = 'OLD_TILES_KEPT' in r.stdout and tile_files(copy) == sb.old_tiles
  ev.append(f'2a.5 again after the change: the host copy kept, not overwritten with the changed set: {kept}')
  unsafe |= not kept
  r = run('tiles-restore')
  restored = 'TILES_RESTORED' in r.stdout and sb.old_tiles_kept()
  ev.append(f"R4: TILES_RESTORED, today's tiles equal the 2a.5 copy again: {restored}")
  unsafe |= not restored
  (copy / TILE).write_bytes(b'damaged on the host\n')
  (sb.root / OLD_TILES / BORDER).unlink()
  before = tile_files(sb.root / OLD_TILES)
  r = run('tiles-restore')
  refused = 'TILES_RESTORE STOP' in r.stdout and tile_files(sb.root / OLD_TILES) == before and 'RESTORE_READY' not in r.stdout
  ev.append(f'R4 with a damaged host copy: TILES_RESTORE STOP, nothing deleted on the device: {refused}')
  unsafe |= not refused
  return Result(29, title, 'UNSAFE' if unsafe else 'SAFE', ev, '\n'.join(out))

ITEMS = {0: flow, 1: item1, 2: item2, 3: item3, 4: item4, 5: item5, 6: item6, 7: item7, 8: item8, 9: item9, 10: item10, 11: item11, 12: item12,
         13: item13, 14: item14, 15: item15, 16: item16, 18: item18, 19: item19, 20: item20, 21: item21, 22: item22, 25: item25, 26: item26,
         27: item27, 28: item28, 29: item29}   # 17, 23 and 24 tested the tile renames of revisions 7 and 8, deleted with them


def syntax(text: str) -> tuple[dict, list[str]]:
  """zsh -n for every host block, bash -n for every device script (`bash -s` heredoc body) and every bash block, and a Python
  compile of every python block. Returns the counts and the errors."""
  errors, counts = [], {'zsh': 0, 'bash': 0, 'python': 0}
  for i, b in enumerate(re.findall(r'```sh\n(.*?)```', text, re.S)):
    counts['zsh'] += 1
    r = subprocess.run(['zsh', '-n', '-c', 'setopt no_bang_hist\n' + b], capture_output=True, text=True)
    if r.returncode:
      errors.append(f'host block {i}: {r.stderr.strip()}')
    for body in re.findall(r"<<'DEV'[^\n]*\n(.*?)\nDEV\n", b, re.S):
      counts['bash'] += 1
      r = subprocess.run([BASH5, '-n'], input=body, capture_output=True, text=True)
      if r.returncode:
        errors.append(f'device script in host block {i}: {r.stderr.strip()}')
  for i, b in enumerate(re.findall(r'```bash\n(.*?)```', text, re.S)):
    counts['bash'] += 1
    r = subprocess.run([BASH5, '-n'], input=b, capture_output=True, text=True)
    if r.returncode:
      errors.append(f'bash block {i}: {r.stderr.strip()}')
  for i, b in enumerate(re.findall(r'```python\n(.*?)```', text, re.S)):
    counts['python'] += 1
    try:
      compile(b, f'python block {i}', 'exec')
    except SyntaxError as e:
      errors.append(f'python block {i}: {e}')
  return counts, errors


def main():
  ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  ap.add_argument('--rev', help='git revision of the procedure (default: the working tree)')
  ap.add_argument('--item', type=int, action='append', help='item number (repeatable; default: all)')
  ap.add_argument('--verbose', action='store_true', help='print each rehearsal output')
  ap.add_argument('--scratch', default=str(Path.home() / '.route_sync/work/fp1031/port-scratch'), help='sandbox parent directory')
  args = ap.parse_args()
  missing = prerequisites()
  if missing:
    sys.exit(f'missing prerequisites: {", ".join(missing)}')
  text = doc_text(args.rev)
  base = Path(tempfile.mkdtemp(prefix='p8_rehearse_', dir=args.scratch))
  print(f'procedure: {args.rev or "working tree"}; sandboxes under {base}')
  counts, errors = syntax(text)
  print(f'syntax: {counts["zsh"]} host blocks (zsh -n), {counts["bash"]} device scripts and bash blocks (bash -n), ' +
        f'{counts["python"]} python blocks (compile): {len(errors)} errors')
  for e in errors:
    print(f'  {e}')
  worst = 1 if errors else 0
  for n in args.item or sorted(ITEMS):
    res = run_item(n, text, base)
    print(f'item {n} {res.verdict}: {res.title}')
    for e in res.evidence:
      print(f'    {e}')
    if args.verbose:
      print(textwrap.indent(res.output, '    | '))
    worst |= res.verdict != 'SAFE'
  with contextlib.suppress(OSError):
    shutil.rmtree(base)
  sys.exit(worst)


if __name__ == '__main__':
  main()
