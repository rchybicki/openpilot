#!/usr/bin/env python3
"""Build a commit the way the car's boot build does, before the commit is pushed.

The check exports COMMIT (default HEAD) to a scratch tree and runs there:
  1. scons, which compiles the driving model bundle and splits it into chunks. On the car this is the boot build
     (`scons --minimal`) over a copy of the car's own build outputs; on a Mac, the modeld targets and what the tests import
  2. selfdrive/modeld/tests/test_model_load.py on the built bundle
  3. 8 stateful policy steps of the bundle, compared with onnxruntime on the same ONNX

The car compiles a QCOM bundle (FLOAT16, IMAGE, a different size) that no Mac build produces. Run --device on the
parked car before pushing a change to the driving model, tinygrad_repo, or the modeld build.

  tools/modeld/build_check.py [COMMIT]                   # this Mac
  tools/modeld/build_check.py --device comma [COMMIT]    # the parked car (commawifi when comma fails)
"""
import argparse
import json
import os
import shlex
import shutil
import signal
import subprocess
import sys
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

PARKED = '[ "$(cat /data/params/d/IsOnroad 2>/dev/null)" = 0 ]'
DEVICE_SCRIPT = f"""
set -e
trap 'echo $? > {DEVICE_WORK}/exit' EXIT
trap 'kill -TERM -- "-$pid" 2>/dev/null || true; exit 143' TERM
cd {DEVICE_WORK}/tree
export PATH=/usr/local/venv/bin:$PATH
source ./launch_env.sh
export PYTHONPATH="$PWD/frogpilot/third_party:$PWD"
step() {{
  {PARKED} || {{ echo "build_check: the car is onroad, not starting: $*" >&2; exit 3; }}
  setsid "$@" &
  pid=$!
  while kill -0 -- "-$pid" 2>/dev/null; do
    if ! {PARKED}; then
      kill -TERM -- "-$pid" 2>/dev/null || true
      wait "$pid" || true
      echo "build_check: the car went onroad, stopped: $*" >&2
      exit 3
    fi
    sleep 2
  done
  wait "$pid"
}}
# build.py retries with fewer jobs when clang runs out of memory
step scons -j4 --minimal --cache-readonly || step scons -j1 --minimal --cache-readonly
step python3 -m pytest -q -n 0 selfdrive/modeld/tests/test_model_load.py
step python3 ../build_check.py policy-steps ../reference.npz ../outputs.npz
"""


def reference(onnx_path, out_path):
  import onnxruntime as ort
  session = ort.InferenceSession(onnx_path, providers=['CPUExecutionProvider'])
  inputs = {i.name: i for i in session.get_inputs()}
  output_names = [o.name for o in session.get_outputs()]
  state = {k: np.zeros(i.shape, np.uint8 if i.type == 'tensor(uint8)' else np.float32) for k, i in inputs.items() if f'next_{k}' in output_names}

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
  np.savez(out_path, **{k: np.stack([s[k] for s in steps]) for k in steps[0]})


def policy_steps(inputs_path, out_path):
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
  np.savez(out_path, outputs=np.stack(outputs))


def matches_reference(reference_path, outputs_path):
  ref, got = np.load(reference_path)['outputs'], np.load(outputs_path)['outputs']
  errors = [float(np.abs(r - g).max() / np.abs(r).max()) for r, g in zip(ref, got, strict=True)]
  for i, e in enumerate(errors):
    print(f'step {i}: max|bundle - onnxruntime| = {e:.4f} of max|onnxruntime|')
  return all(e <= TOLERANCE for e in errors)


def check_on_device(host, work):
  ssh = [*SSH, host]
  # the car's untracked build outputs make the scratch build incremental, as the boot build after an update is
  seed = f'cd /data/openpilot && git ls-files -z --others | rsync -a --from0 --files-from=- ./ {DEVICE_WORK}/tree/'
  subprocess.run([*ssh, f'{PARKED} || {{ echo "build_check: the car is onroad" >&2; exit 3; }}; rm -rf {DEVICE_WORK} && mkdir -p {DEVICE_WORK}/tree && {seed}'],
                 check=True)
  job = None
  try:
    # files that match the car's own checkout are copied on the car, so only the commit's changes cross the network
    subprocess.run(['rsync', '-a', '--checksum', '--copy-dest=/data/openpilot/', f'{work}/tree/', f'{host}:{DEVICE_WORK}/tree/'], check=True)
    subprocess.run(['scp', '-q', __file__, f'{work}/reference.npz', f'{host}:{DEVICE_WORK}/'], check=True)
    # the job and its onroad guard run on the car, so a dropped connection only delays the result
    launch = f'setsid nohup bash -c {shlex.quote(DEVICE_SCRIPT)} > {DEVICE_WORK}/log 2>&1 < /dev/null & echo $!'
    job = subprocess.check_output([*ssh, launch], text=True).strip()
    print(f'building on {host}; follow it with: ssh {host} tail -f {DEVICE_WORK}/log', flush=True)
    while True:
      time.sleep(15)
      poll = subprocess.run([*ssh, f'cat {DEVICE_WORK}/exit 2>/dev/null || kill -0 {job}'], capture_output=True, text=True)
      if poll.returncode != 255 and (poll.stdout or poll.returncode != 0):
        break
    subprocess.run([*ssh, f'cat {DEVICE_WORK}/log'], check=False)
    status = int(poll.stdout) if poll.stdout else -1
    if status != 0:
      raise subprocess.CalledProcessError(status, [f'the job on {host} (log above)'])
    subprocess.run(['scp', '-q', f'{host}:{DEVICE_WORK}/outputs.npz', f'{work}/'], check=True)
  finally:
    stop = f'[ -f {DEVICE_WORK}/exit ] || kill -TERM {job}; sleep 3; ' if job else ''
    # never delete gigabytes while the car goes onroad; the next check removes what is left
    subprocess.run([*ssh, f'{PARKED} && {{ {stop}rm -rf {DEVICE_WORK}; }} || echo "build_check: left {DEVICE_WORK} on the car"'], check=False)


def check_on_mac(work):
  tree = work / 'tree'
  env = os.environ | {'PYTHONPATH': f'{tree}/frogpilot/third_party:{tree}'}
  # a Mac has no Panda firmware toolchain, so it builds the model and what the tests import, not the whole boot build
  for cmd in (['scons', '-j4', '--cache-readonly', 'selfdrive/modeld', 'selfdrive/pandad', 'msgq_repo', 'test-dependencies'],
              [sys.executable, '-m', 'pytest', '-q', '-n', '0', 'selfdrive/modeld/tests/test_model_load.py'],
              [sys.executable, __file__, 'policy-steps', '../reference.npz', '../outputs.npz']):
    subprocess.run(cmd, cwd=tree, env=env, check=True)


def main():
  p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  p.add_argument('commit', nargs='?', default='HEAD')
  p.add_argument('--device', metavar='HOST', help='ssh host of the parked car; without it the check runs on this Mac')
  args = p.parse_args()
  # a closed terminal or a kill still runs the cleanup below, which stops the job on the car
  for sig in (signal.SIGTERM, signal.SIGHUP):
    signal.signal(sig, lambda *_: sys.exit('build_check: stopped'))

  sha = subprocess.check_output(['git', 'rev-parse', '--verify', f'{args.commit}^{{commit}}'], text=True).strip()
  work = WORK / sha[:12]
  shutil.rmtree(work, ignore_errors=True)
  (work / 'tree').mkdir(parents=True)
  try:
    subprocess.run(['bash', '-o', 'pipefail', '-c', f'git archive {sha} | tar -x -C {shlex.quote(str(work / "tree"))}'], check=True)
    # the tests read build metadata, which a tree without .git takes from build.json; channel "unknown" sets the same flags as the car's branch
    (work / 'tree' / 'build.json').write_text(json.dumps({'openpilot': {'git_commit': sha}}))
    subprocess.run([*ORT_PYTHON, __file__, 'reference', str(work / 'tree/selfdrive/modeld/models/driving_supercombo.onnx'), str(work / 'reference.npz')],
                   check=True)
    if args.device:
      check_on_device(args.device, work)
    else:
      check_on_mac(work)
    passed = matches_reference(work / 'reference.npz', work / 'outputs.npz')
  except subprocess.CalledProcessError as e:
    sys.exit(f'FAIL {sha[:12]} on {args.device or "this Mac"}: exit {e.returncode} from {shlex.join(map(str, e.cmd)).splitlines()[0]}')
  finally:
    shutil.rmtree(work / 'tree', ignore_errors=True)
  if not passed:
    sys.exit(f'FAIL {sha[:12]} on {args.device or "this Mac"}: the bundle differs from onnxruntime by more than {TOLERANCE}')
  print(f'PASS {sha[:12]} on {args.device or "this Mac"}: scons build, test_model_load, and onnxruntime match')


if __name__ == '__main__':
  if sys.argv[1:2] == ['reference']:
    reference(*sys.argv[2:])
  elif sys.argv[1:2] == ['policy-steps']:
    policy_steps(*sys.argv[2:])
  else:
    main()
