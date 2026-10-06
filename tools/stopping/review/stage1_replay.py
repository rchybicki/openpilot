"""Stage-1 recorded-motion comparison, using plant_sim's gate-2 controller/sender loop.

Run with .venv: python -m tools.stopping.review.stage1_replay. No device access.
Motion after the first differing command is NOT a counterfactual prediction.
"""
import argparse
import contextlib
from functools import cache
from multiprocessing import Pool
import os
import hashlib
import json
import subprocess
import sys
from pathlib import Path
from types import ModuleType

from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.controls.lib import longcontrol, stopping_flags, stopping_service
from openpilot.tools.stopping.review import plant_data, plant_sim
from openpilot.tools.stopping.review.kcs_plant import GAIN, cells
from openpilot.tools.stopping.review.plant_data import DECISION, OUTPUT, natural_entries

DEST = DECISION / 'stage1'


@cache
def head_controller():
  """Load pristine HEAD source without changing the checkout or its import bindings."""
  modules = []
  for name in ('stopping_service', 'longcontrol'):
    module = ModuleType('_stage1_head_' + name)
    sys.modules[module.__name__] = module
    source = subprocess.check_output(['git', 'show', f'HEAD:selfdrive/controls/lib/{name}.py'], text=True)
    exec(compile(source, f'HEAD:{name}.py', 'exec'), module.__dict__)
    modules.append(module)
  # Preserve the gate-2 helper's service type assertion while executing HEAD methods.
  class HeadService(modules[0].StoppingService, stopping_service.StoppingService):
    pass
  modules[1].StoppingService = HeadService
  return modules[1].LongControl


def replay_stop(entry, inputs):
  rows = {}
  originals = stopping_flags.FINAL_FLOOR, stopping_flags.FLAT_LANDING
  cloudlog.disabled = True
  try:
    for name, floor, flat, cls in [('HEAD', False, False, head_controller()),
                                 ('off', False, False, longcontrol.LongControl),
                                 ('floor', True, False, longcontrol.LongControl),
                                 ('flat', False, True, longcontrol.LongControl),
                                 ('both', True, True, longcontrol.LongControl)]:
      stopping_flags.FINAL_FLOOR, stopping_flags.FLAT_LANDING = floor, flat
      trace = []

      def capture(lc, frame, command, sent, trace=trace):
        # HEAD has no retained signals: context state still supplies its identical wheel latch.
        trace.append(dict(t=frame['t'], v=frame['cs']['vEgo'], gap=frame['kw']['lead_d_rel'], u=command, scc12=sent,
                          armed=getattr(lc, '_final_floor_armed', False), added=getattr(lc, '_final_floor_added', 0.),
                          wheel=lc._service_shadow_ctx._wstop_latched))

      plant_sim.simulate(entry, inputs, cells()['nominal'], GAIN, recorded=True, controller_type=cls, frame_observer=capture)
      rows[name] = trace
  finally:
    stopping_flags.FINAL_FLOOR, stopping_flags.FLAT_LANDING = originals
    cloudlog.disabled = False
  identical = all(a['u'].hex() == b['u'].hex() and a['scc12'].hex() == b['scc12'].hex()
                  for a, b in zip(rows['HEAD'], rows['off'], strict=True))
  assert identical, entry['id']
  result = dict(id=entry['id'], frames=len(rows['HEAD']), off_bit_identical=identical, arms={})
  for name, trace in rows.items():
    binding = next((i for i, r in enumerate(trace) if r['added'] > 0.), None)
    difference = next((i for i, (a, b) in enumerate(zip(trace, rows['HEAD'], strict=True)) if a['u'] != b['u']), None)
    wheel_from = entry['rest'] - .5 if entry['split'] == 'census' else entry['cross']
    wheel = next((r for r in trace if r['wheel'] and r['t'] >= wheel_from), None)
    result['arms'][name] = dict(armed=any(r['armed'] for r in trace),
      first_binding=trace[binding] if binding is not None else None,
      bound_frames=sum(r['added'] > 0. for r in trace),
      max_added_before_binding=max((r['added'] for r in trace[:binding]), default=0.) if binding is not None else 0.,
      max_added_open_loop=max((r['added'] for r in trace), default=0.),
      first_difference=trace[difference] if difference is not None else None, wheel_stop=wheel)
  return result


def census_stop(job):
  route, stop, start = job
  key = f"{route['route']}_{stop['rest']:.9f}"
  result_path = DEST / 'census' / f'{key}.json'
  if result_path.exists():
    return json.loads(result_path.read_text())
  entry = dict(id=key, route=route['route'], settings={}, paths=route['paths'], t=[start, stop['rest'] + 1.],
               cross=start + 2.1, rest=stop['rest'], split='census', recorded_commit='see source rlogs', grade=0.)
  scratch = DEST / f'inputs_worker_{os.getpid()}.json'
  try:
    from openpilot.tools.lib.logreader import LogReader
    first = min(route['paths'], key=lambda p: int(Path(p).parent.name.rsplit('--', 1)[1]))
    init = next(e.initData for e in LogReader(first) if e.which() == 'initData')
    entry['settings'] = {kv.key: bytes(kv.value).decode() for kv in init.params.entries
                         if kv.key in {'HumanAcceleration', 'LongitudinalTune', 'CEForceCoastStrength'}}
    # Existing gate-2 extraction, redirected into this report directory. No sim inputs are changed.
    original_output = plant_data.output_path
    try:
      plant_data.output_path = lambda name: scratch
      with contextlib.redirect_stdout(sys.stderr):
        inputs = plant_data.extract_inputs([entry])[key]
    finally:
      plant_data.output_path = original_output
    frames = inputs['frames']
    times = [f['t'] for f in frames]
    entry.update(send_t=times, send_u=[f['recorded'] for f in frames], jerk_t=times,
                 jerk_up=[3.] * len(times), jerk_lo=[5.] * len(times))
    # The plant is unused by recorded=True. These warm-up placeholders never feed LongControl.
    result = replay_stop(entry, inputs)
    result['source_paths'] = route['paths']
    result['census_eligible'], result['census_reasons'] = stop['eligible'], stop['reasons']
  except (OSError, ValueError, RuntimeError, StopIteration, KeyError, IndexError, TypeError) as exc:
    result = dict(id=key, error=f'{type(exc).__name__}: {exc}', census_eligible=stop['eligible'])
  finally:
    scratch.unlink(missing_ok=True)
  result_path.write_text(json.dumps(result, indent=2))
  return result


def main():
  parser = argparse.ArgumentParser()
  parser.add_argument('--natural', action='store_true')
  parser.add_argument('--census', action='store_true')
  parser.add_argument('--workers', type=int, default=8)
  args = parser.parse_args()
  DEST.mkdir(exist_ok=True)
  if args.census:
    (DEST / 'census').mkdir(exist_ok=True)
    routes = json.loads((OUTPUT / 'v2_census.json').read_text())['routes']
    jobs = []
    for route in routes:
      previous = -float('inf')
      for stop in route['stops']:
        jobs.append((route, stop, max(stop['rest'] - 60., previous + .1)))
        previous = stop['rest']
    with Pool(args.workers) as pool:
      for i, result in enumerate(pool.imap_unordered(census_stop, jobs, chunksize=1)):
        print(i + 1, '/', len(jobs), result['id'], result.get('error', 'PASS'), flush=True)
  if args.natural:
    sources, inputs = {}, {}
    for path in sorted(OUTPUT.glob('inputs*.json')):
      sources[path.name] = hashlib.sha256(path.read_bytes()).hexdigest()
      for key, value in json.loads(path.read_text()).items():
        if key in inputs and inputs[key] != value:
          raise ValueError(f'conflicting input copies: {key}')
        inputs[key] = value
    entries = {e['id']: e for e in natural_entries()}
    results = []
    for key, value in inputs.items():
      result = replay_stop(entries[key], value)
      results.append(result)
      (DEST / 'natural.json').write_text(json.dumps(dict(sources=sources, stops=results), indent=2))
      print(key, result['off_bit_identical'], flush=True)


if __name__ == '__main__':
  main()
