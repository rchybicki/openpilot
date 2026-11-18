#!/usr/bin/env python3
"""Check the modeld build path of a commit on the parked car, before the commit is pushed.

The car builds COMMIT (default HEAD) the way its boot build does, in a copy of its own checkout and build outputs:
  1. scons --minimal compiles the QCOM driving model bundle and splits it into chunks
  2. selfdrive/modeld/tests/test_model_load.py loads the built bundle
  3. 8 stateful policy steps of the bundle are compared with onnxruntime on the same ONNX: the desired lateral and
     longitudinal accelerations to CONTROL_TOLERANCE m/s^2, all outputs to TOLERANCE of their range
The gate covers the modeld build path (compile, chunk, load, numerics). It does not replay all of system/manager/build.py:
the panda version string differs, build.py's Params artifact cleanup is skipped, and a failed build is retried at -j1 only.

Without --device the Mac builds a CPU bundle and runs the same tests: a first check, not the gate.

  tools/modeld/build_check.py --device comma [COMMIT]    # the parked car (commawifi when comma fails)
  tools/modeld/build_check.py [COMMIT]                   # this Mac
"""
import argparse
import base64
import json
import os
import pickle
import shlex
import shutil
import signal
import subprocess
import sys
import tarfile
import time
from pathlib import Path

import numpy as np

WORK = Path.home() / '.route_sync' / 'work' / 'model_build_check'
DEVICE_WORK = '/data/model_build_check'
ORT_PYTHON = ['uv', 'run', '--no-project', '--python', '3.11', '--with', 'onnxruntime==1.32.0', '--with', 'numpy==2.4.6', 'python']
SSH = ['ssh', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=8', '-o', 'ServerAliveInterval=15']
STEPS = 8
# max|bundle - onnxruntime| / max|onnxruntime| per step. Cheddar: the QCOM bundle 0.0034; one recurrent state dropped 0.039 to 0.090
TOLERANCE = 0.01
# max|bundle - onnxruntime| of the desired accelerations, m/s^2. Cheddar's QCOM bundle: 0.020 lateral, 0.006 longitudinal
CONTROL_TOLERANCE = 0.05
# seconds for the whole job on the car; it takes about 10 minutes when the build passes and 16 when it fails
TIMEOUT = 40 * 60

# The job on the car. `receive` takes the payload from stdin and starts `run` detached. `run` copies the car's checkout and
# build outputs, applies the commit's changes, builds, tests, and deletes all it wrote. Each heavy command is a step: its own
# process group (set -m), idle CPU and I/O priority (from the launch), stopped when the car goes onroad, the Mac writes `stop`,
# or the deadline passes. The job and its steps hold the flock on `lock` until the last of them ends.
DEVICE_SCRIPT = r'''
set -e -o pipefail -m
mode=$1 sha=$2 base=$3
run=$PWD
cleaning= current=

parked() { [ "$(cat /data/params/d/IsOnroad 2>/dev/null)" = 0 ]; }
uptime_s() { local up; read -r up _ < /proc/uptime; echo "${up%.*}"; }

stop_reason() {
  if ! parked; then echo 'the car is onroad'
  elif [ -n "$cleaning" ]; then return 1
  elif [ -e "$run/stop" ]; then echo 'the Mac stopped the check'
  elif [ "$(uptime_s)" -ge "$deadline" ]; then echo 'the check timed out'
  else return 1
  fi
}

end_group() {
  # TERM the step's process group, KILL what ignores TERM, and return when none of its processes is left
  local signal i
  for signal in TERM KILL; do
    kill "-$signal" -- "-$1" 2>/dev/null || return 0
    for i in $(seq 50); do
      kill -0 -- "-$1" 2>/dev/null || return 0
      sleep 0.2
    done
  done
  echo "build_check: process group $1 outlived SIGKILL" >&2
  return 1
}

step() {
  local reason rc
  if reason=$(stop_reason); then echo "build_check: $reason, not starting: $*" >&2; return 3; fi
  "$@" &
  current=$!
  while kill -0 "$current" 2>/dev/null; do
    if reason=$(stop_reason); then
      echo "build_check: $reason, stopping: $*" >&2
      end_group "$current"
      wait "$current" 2>/dev/null || true
      current=
      return 3
    fi
    sleep 2
  done
  wait "$current" && rc=0 || rc=$?
  end_group "$current"
  current=
  return "$rc"
}

remove() {
  # delete only while parked; when the car goes onroad, wait until it is parked again
  local rc
  while :; do
    step rm -rf "$@" && return 0 || rc=$?
    [ "$rc" = 3 ] || return "$rc"
    until parked; do sleep 10; done
  done
}

finish() {
  local rc=$?
  set +e
  [ -z "$current" ] || end_group "$current"
  cd "$run"
  echo "$sha $rc" > status.part && mv status.part status
  cleaning=1
  remove tree home tmp pycache params
  # the Mac reads the log and the outputs, then writes fetched; stop means it will not
  for _ in $(seq 300); do [ -e fetched ] || [ -e stop ] && break; sleep 2; done
  remove "$run"
}

case $mode in
receive)
  exec 9>>lock
  flock -n 9
  deadline=$(( $(uptime_s) + $4 ))
  # until `run` starts, the run holds only the payload, one file
  trap 'rm -rf "$run"' EXIT
  # room for the copy of /data/openpilot without .git and 2 GiB for the build, leaving free what openpilot's deleter keeps free
  step du -sk --exclude=.git /data/openpilot > du.out
  read -r tree_kb _ < du.out
  read -r size_kb free_kb < <(df -k --output=size,avail /data | tail -n 1)
  need_kb=$(( tree_kb + $5 / 1024 + 2 * 1024 * 1024 ))
  keep_kb=$(( size_kb / 10 > 5 * 1024 * 1024 ? size_kb / 10 : 5 * 1024 * 1024 ))
  if [ $(( free_kb - need_kb )) -lt "$keep_kb" ]; then
    echo "build_check: /data has $free_kb KiB free; the check needs $need_kb KiB and must leave $keep_kb KiB" >&2
    exit 5
  fi
  step tar -x <&0
  exec </dev/null
  trap - EXIT
  setsid nohup bash job.sh run "$sha" "$base" "$deadline" > log 2>&1 &
  ;;
run)
  deadline=$4
  trap finish EXIT
  trap 'exit 143' TERM HUP INT
  # runs of a car that powered off mid-check: older than an hour, and no process holds their lock
  for old in "${run%.*}".*; do
    if [ "$old" != "$run" ] && [ -n "$(find "$old" -maxdepth 0 -mmin +60)" ] && flock -n "$old/lock" true; then
      step rm -rf "$old"
    fi
  done
  mkdir tree home tmp
  # the car's build outputs (its untracked files) make the build incremental, as the boot build after an update is
  step bash -o pipefail -c 'git -C /data/openpilot ls-files -z --others | rsync -a --from0 --files-from=- /data/openpilot/ tree/'
  step bash -o pipefail -c 'git -C /data/openpilot archive "$0" | tar -x -C tree' "$base"
  step git -C tree apply --allow-empty ../changes.patch
  # a tree without .git takes its build metadata from build.json; channel "unknown" sets the same flags as the car's branch
  printf '{"openpilot": {"git_commit": "%s"}}\n' "$sha" > tree/build.json
  cd tree
  export PATH=/usr/local/venv/bin:$PATH
  source ./launch_env.sh
  # the build and the tests write only in this run: tinygrad's cache (under HOME), temporary files, bytecode, Params
  export PYTHONPATH="$PWD/frogpilot/third_party:$PWD" HOME="$run/home" TMPDIR="$run/tmp" PYTHONPYCACHEPREFIX="$run/pycache" \
         PARAMS_ROOT="$run/params" OPENPILOT_PREFIX=build_check
  # no SCons cache: even --cache-readonly refreshes the timestamps of the shared cache's entries.
  # build.py retries with fewer jobs when clang runs out of memory
  step scons -j4 --minimal --cache-disable || { rc=$?; [ "$rc" != 3 ] || exit 3; step scons -j1 --minimal --cache-disable; }
  # the repository conftest is not loaded: its fixture writes to /dev/shm
  step python3 -m pytest -q -n 0 --noconftest selfdrive/modeld/tests/test_model_load.py
  step python3 ../build_check.py policy-steps ../reference.npz ../outputs.npz "$sha"
  ;;
esac
'''


def reference(onnx_path, out_path):
  import onnxruntime as ort
  from openpilot.common.file_chunker import read_file_chunked
  session = ort.InferenceSession(read_file_chunked(onnx_path), providers=['CPUExecutionProvider'])
  inputs = {i.name: i for i in session.get_inputs()}
  output_names = [o.name for o in session.get_outputs()]
  state = {k: np.zeros(i.shape, np.uint8 if i.type == 'tensor(uint8)' else np.float32) for k, i in inputs.items() if f'next_{k}' in output_names}
  # the action output holds the desired lateral and longitudinal accelerations, then their log std (Parser.parse_mdn)
  action = pickle.loads(base64.b64decode(session.get_modelmeta().custom_metadata_map['output_slices']))['action']

  rng = np.random.default_rng(0)
  steps = []
  for i in range(STEPS):
    desire = np.zeros(inputs['desire'].shape, np.float32)
    desire[..., 2] = i == 3
    feeds = {
      'new_img': rng.integers(0, 256, inputs['new_img'].shape, dtype=np.uint8),
      'desire': desire,
      'traffic_convention': np.array([[1, 0]], np.float32),
      'action_t': np.array([[0.3 + 0.01 * i, 0.5]], np.float32),
    }
    assert feeds.keys() | state.keys() == inputs.keys(), f"the model's inputs are {sorted(inputs)}; update build_check.py"
    res = dict(zip(output_names, session.run(None, feeds | state), strict=True))
    state = {k: res[f'next_{k}'] for k in state}
    steps.append(feeds | {'outputs': res['outputs']})
  np.savez(out_path, controls=np.arange(action.start, (action.start + action.stop) // 2), **{k: np.stack([s[k] for s in steps]) for k in steps[0]})


def policy_steps(inputs_path, out_path, sha):
  from tinygrad.tensor import Tensor
  from openpilot.common.file_chunker import open_file_chunked
  from openpilot.selfdrive.modeld.compile_modeld import POLICY_INPUTS, make_input_queues
  from openpilot.selfdrive.modeld.helpers import MODELD_PKL_PATH, get_tg_input_devices, load_oob

  devices = get_tg_input_devices('selfdrive.modeld.modeld')
  with open_file_chunked(MODELD_PKL_PATH) as f:
    jits = load_oob(f)
  queues, npy = make_input_queues(jits['metadata']['input_shapes'], devices['QUEUE_DEV'])
  steps = np.load(inputs_path)
  outputs = []
  for i in range(len(steps['outputs'])):
    for k in npy.keys() & set(steps.files):
      npy[k][:] = steps[k][i]
    warped = Tensor(steps['new_img'][i], device=devices['WARP_DEV']).realize()
    out, = jits['run_policy'](**{k: queues[k] for k in POLICY_INPUTS}, warped=warped)
    outputs.append(out.numpy().copy())
  np.savez(out_path, outputs=np.stack(outputs), sha=sha)


def matches_reference(reference_path, outputs_path):
  ref = np.load(reference_path)
  controls = ref['controls']
  passed = True
  for i, (r, g) in enumerate(zip(ref['outputs'], np.load(outputs_path)['outputs'], strict=True)):
    error = float(np.abs(r - g).max() / np.abs(r).max())
    control_error = float(np.abs(r[..., controls] - g[..., controls]).max())
    print(f'step {i}: outputs differ from onnxruntime by {error:.4f} of their range, desired accelerations by {control_error:.4f} m/s^2')
    passed &= error <= TOLERANCE and control_error <= CONTROL_TOLERANCE
  return passed


def check_on_device(host, sha, work):
  ssh = [*SSH, host]
  run = f'{DEVICE_WORK}.{work.name}'
  base = subprocess.check_output([*ssh, 'git -C /data/openpilot rev-parse HEAD'], text=True).strip()
  if subprocess.run(['git', 'cat-file', '-e', f'{base}^{{commit}}']).returncode:
    sys.exit(f'build_check: the car runs {base[:12]}, which this checkout does not have; fetch it first')
  # the car copies its own checkout, so only the commit's changes cross the network
  with open(work / 'changes.patch', 'wb') as f:
    subprocess.run(['git', 'diff', '--binary', '--full-index', '--no-color', '--no-ext-diff', '--no-textconv', base, sha], stdout=f, check=True)
  with tarfile.open(work / 'payload.tar', 'w') as tar:
    for path in (work / 'changes.patch', work / 'reference.npz', Path(__file__)):
      tar.add(path, arcname=path.name)
  job = f'nice -n 19 ionice -c 3 bash job.sh receive {sha} {base} {TIMEOUT} {os.path.getsize(work / "payload.tar")}'
  launch = f'mkdir {run} && cd {run} && printf %s {shlex.quote(DEVICE_SCRIPT)} > job.sh && exec {job}'
  # liveness first: the job writes its status before it releases the lock
  poll = f'cd {run} 2>/dev/null || exit 4; {{ flock -n 8 && echo ended; }} 8<lock; cat status 2>/dev/null; true'
  status = None
  try:
    with open(work / 'payload.tar', 'rb') as payload:
      sent = subprocess.run([*ssh, launch], stdin=payload)
    # 255: ssh failed, and the job may have started anyway
    if sent.returncode not in (0, 255):
      raise subprocess.CalledProcessError(sent.returncode, [f'the job on {host}'])
    print(f'building {sha[:12]} on {host}; follow it with: ssh {host} tail -f {run}/log', flush=True)
    deadline = time.monotonic() + TIMEOUT + 300
    while status is None:
      if time.monotonic() > deadline:
        raise subprocess.CalledProcessError(124, [f'the job on {host} (no result after {TIMEOUT + 300} s)'])
      time.sleep(15)
      res = subprocess.run([*ssh, poll], capture_output=True, text=True)
      if res.returncode == 4:
        raise subprocess.CalledProcessError(4, [f'the job on {host} (its directory is gone)'])
      ended = res.stdout.startswith('ended')
      status = res.stdout.removeprefix('ended').split() or None
      if ended and status is None:
        raise subprocess.CalledProcessError(4, [f'the job on {host} (it ended without a result)'])
    (work / 'log').write_bytes(subprocess.run([*ssh, f'cat {run}/log'], capture_output=True).stdout)
    print(*(work / 'log').read_text(errors='replace').splitlines()[-40:], f'(full log: {work / "log"})', sep='\n')
    if status[0] != sha:
      raise subprocess.CalledProcessError(4, [f'the job on {host} (it built {status[0]})'])
    if status[1] != '0':
      raise subprocess.CalledProcessError(int(status[1]), [f'the job on {host} (log above)'])
    (work / 'outputs.npz').write_bytes(subprocess.run([*ssh, f'cat {run}/outputs.npz'], capture_output=True, check=True).stdout)
    if str(np.load(work / 'outputs.npz')['sha']) != sha:
      raise subprocess.CalledProcessError(4, [f'the job on {host} (outputs of another commit)'])
  finally:
    # the job deletes its copy itself, only while the car is parked
    subprocess.run([*ssh, f'touch {run}/{"fetched" if status else "stop"}'], check=False, capture_output=True)
    for _ in range(30):
      gone = subprocess.run([*ssh, f'[ ! -e {run} ]'], capture_output=True).returncode
      if gone != 1:
        break
      time.sleep(10)
    if gone == 0:
      print(f'{host} deleted {run}')
    else:
      print(f'build_check: {run} may still be on {host}; its job, or the next check if the job died, deletes it while the car is parked')


def check_on_mac(tree, sha):
  env = os.environ | {'PYTHONPATH': f'{tree}/frogpilot/third_party:{tree}'}
  # a tree without .git takes its build metadata from build.json
  (tree / 'build.json').write_text(json.dumps({'openpilot': {'git_commit': sha}}))
  # a Mac has no Panda firmware toolchain, so it builds the model and what the tests import, not the whole boot build
  for cmd in (['scons', '-j4', '--cache-readonly', 'selfdrive/modeld', 'selfdrive/pandad', 'msgq_repo', 'test-dependencies'],
              [sys.executable, '-m', 'pytest', '-q', '-n', '0', 'selfdrive/modeld/tests/test_model_load.py'],
              [sys.executable, __file__, 'policy-steps', '../reference.npz', '../outputs.npz', sha]):
    subprocess.run(cmd, cwd=tree, env=env, check=True)


def main():
  p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  p.add_argument('commit', nargs='?', default='HEAD')
  p.add_argument('--device', metavar='HOST', help='ssh host of the parked car; without it the check runs on this Mac')
  args = p.parse_args()
  # a closed terminal or a kill still runs the cleanup, which stops the job on the car
  for sig in (signal.SIGTERM, signal.SIGHUP, signal.SIGINT):
    signal.signal(sig, lambda *_: sys.exit('build_check: stopped'))

  sha = subprocess.check_output(['git', 'rev-parse', '--verify', f'{args.commit}^{{commit}}'], text=True).strip()
  work = WORK / f'{sha[:12]}-{time.strftime("%Y%m%d-%H%M%S")}-{os.getpid()}'
  tree = work / 'tree'
  tree.mkdir(parents=True)
  try:
    subprocess.run(['bash', '-o', 'pipefail', '-c', f'git archive {sha} | tar -x -C {shlex.quote(str(tree))}'], check=True)
    subprocess.run([*ORT_PYTHON, __file__, 'reference', str(tree / 'selfdrive/modeld/models/driving_supercombo.onnx'), str(work / 'reference.npz')],
                   env=os.environ | {'PYTHONPATH': str(tree)}, check=True)
    if args.device:
      check_on_device(args.device, sha, work)
    else:
      check_on_mac(tree, sha)
    passed = matches_reference(work / 'reference.npz', work / 'outputs.npz')
  except subprocess.CalledProcessError as e:
    sys.exit(f'FAIL {sha[:12]} on {args.device or "this Mac"}: exit {e.returncode} from {shlex.join(map(str, e.cmd)).splitlines()[0]}')
  finally:
    shutil.rmtree(tree, ignore_errors=True)
    for name in ('changes.patch', 'payload.tar'):
      (work / name).unlink(missing_ok=True)
  if not passed:
    sys.exit(f'FAIL {sha[:12]} on {args.device or "this Mac"}: the bundle differs from onnxruntime ({TOLERANCE} of the range, {CONTROL_TOLERANCE} m/s^2)')
  print(f'PASS {sha[:12]} on {args.device or "this Mac"}: scons build, test_model_load, and onnxruntime match')


if __name__ == '__main__':
  if sys.argv[1:2] == ['reference']:
    reference(*sys.argv[2:])
  elif sys.argv[1:2] == ['policy-steps']:
    policy_steps(*sys.argv[2:])
  else:
    main()
