#!/usr/bin/env python3
"""Paired controller stress tests on hypothetical plants, not predictions of vehicle performance."""
import argparse
import hashlib
import itertools
import json
import math
import subprocess
import sys
import types
from collections import deque
from pathlib import Path

from openpilot.selfdrive.controls.lib import stopping_flags
from openpilot.selfdrive.controls.lib.longcontrol import LongControl
from openpilot.selfdrive.controls.lib.stopping_service import StoppingService
from openpilot.selfdrive.controls.lib.stopping_telemetry import StoppingTelemetry
from openpilot.selfdrive.controls.lib.tests.test_longcontrol_fast_release import DummyCarParams, DummyCarState, DummyFrogPilotToggles
from openpilot.tools.stopping.review.entry_replay import SERVICE


def simulate(service_type, entry, plant, lead, wheel_observation):
  v, gap, wire = entry
  gain, lag, delay, push = plant
  lead_v, lead_brake = lead
  control = LongControl(DummyCarParams())
  control._service_shadow_svc = service_type()
  control._service_shadow_tel = StoppingTelemetry(log_fn=lambda **kw: None)
  control.last_output_accel = wire
  toggles = DummyFrogPilotToggles()
  a = gain * wire + push
  vk, ak, raw = v, a, v
  history = deque([wire] * round(delay / 0.01))
  stopped, creep = None, False
  min_gap = gap
  trace, head, tail = [], [], []
  for k in range(2500):
    if wheel_observation:
      # Known native KF recurrence; held 50 Hz input. Rounding the mean wheel grid is a
      # hypothetical quantizer, not an identified wheel-ECU observation model.
      if k % 2 == 0:
        raw = round(v * 460.8) / 460.8
      if abs(raw - vk) > 2.0:
        vk, ak = raw, 0.0
      vk, ak = .825939610864816 * vk + .01 * ak + .17406038913518396 * raw, ak + 1.6592563982783999 * (raw - vk)
    else:
      vk, ak, raw = v, a, v
    # Both arms receive a position-dependent planner proxy, not the production MPC.
    plan = -min(max(vk - lead_v, 0) ** 2 / (2 * max(gap - 4.3, 0.3)), 1.5)
    wire = float(control.update(True, DummyCarState(v_ego=vk, a_ego=ak, standstill=raw < .005), plan,
      True, max(gap - 4.3, 0), (-3.5, 2), toggles, experimental_mode=True,
      lead_status=True, lead_v=lead_v, lead_d_rel=gap, lead_a=-lead_brake if lead_v > 0 else 0,
      lead_track_id=7, lead_model_prob=.99, increased_stopped_distance=.3, a_target_trajectory=plan))
    history.append(wire)
    a += (gain * history.popleft() + push - a) * -math.expm1(-.01 / lag)
    next_v = max(v + a * .01, 0)
    next_lead_v = max(lead_v - lead_brake * .01, 0)
    gap += (lead_v + next_lead_v - v - next_v) * .005
    min_gap = min(min_gap, gap)
    lead_v = next_lead_v
    a, v = (next_v - v) / .01, next_v
    trace.append(a)
    if v > .5:
      head.append(a)
    elif v > .05:
      tail.append(a)
    if stopped is not None:
      creep |= v > .05
      if k - stopped >= 100:
        break
    elif v == 0:
      stopped = k
    if gap < 1:
      break
  return dict(gap=gap, min_gap=min_gap, time_s=k * .01, stopped=stopped is not None, creep=creep,
              head_min=min(head, default=None), tail_min=min(tail, default=None),
              jerk_300=max(abs(trace[i] - trace[i - 30]) / .3 for i in range(30, len(trace))))


if __name__ == '__main__':
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('--base', required=True)
  parser.add_argument('--wheel-observation', action='store_true')
  parser.add_argument('--output', type=Path, required=True)
  args = parser.parse_args()
  if args.output.exists():
    raise FileExistsError(args.output)
  runtime = ['selfdrive', 'common', 'opendbc', 'frogpilot', 'cereal', 'system']
  changed = subprocess.check_output(['git', 'diff', '--name-only', args.base, '--', *runtime], text=True).splitlines()
  changed += subprocess.check_output(['git', 'ls-files', '--others', '--exclude-standard', '--', *runtime], text=True).splitlines()
  if any(p != SERVICE and '/tests/' not in p for p in changed):
    raise ValueError('This comparison requires identical runtime outside StoppingService')
  source = subprocess.check_output(['git', 'show', f'{args.base}:{SERVICE}'])
  baseline = types.ModuleType('baseline_stopping_service')
  sys.modules[baseline.__name__] = baseline
  exec(compile(source, f'{args.base}:{SERVICE}', 'exec'), baseline.__dict__)
  assert stopping_flags.SERVICE_MODE == 'LIVE' and stopping_flags.SERVICE_APPROACH_LAW == 'governor'
  assert stopping_flags.GOVERNOR_RECOVERY_BRAKE and stopping_flags.GOVERNOR_PROFILE_REFERENCE
  assert stopping_flags.ATTRIBUTED_SAFETY == 'live'
  entries = [(2.4, 11., -.3), (2.22, 7.2, -1.), (2.15, 7., -1.3), (2.4, 5.5, -.1),
             (1.1, 5., -.3), (2.4, 5.5, -2.), (2.4, 7., -2.), (2.4, 5.5, -2.5)]
  cases = [(entry, plant, (0., 0.)) for entry in entries for plant in
           itertools.product([.7, 1., 1.3], [.15, .4, .7], [0., .3, .6], [-.45, -.1, 0., .2, .45])]
  cases += [((2.4, gap, wire), plant, (lv, brake)) for gap, wire, lv, brake in
            itertools.product([5.5, 8., 11.], [-.3, -1.5], [.5, 1., 2.], [1., 2.5, 4.]) for plant in
            itertools.product([.8, 1.2], [.15, .6], [0., .6], [-.45, .45])]
  rows = []
  for entry, plant, lead in cases:
    rows.append(dict(entry=entry, plant=plant, lead=lead,
      baseline=simulate(baseline.StoppingService, entry, plant, lead, args.wheel_observation),
      candidate=simulate(StoppingService, entry, plant, lead, args.wheel_observation)))
  paths = [Path(__file__), Path(SERVICE), Path('selfdrive/controls/lib/longcontrol.py'),
           Path('selfdrive/controls/lib/stop_context.py'), Path('selfdrive/controls/lib/stopping_flags.py'),
           Path('selfdrive/controls/lib/tests/test_longcontrol_fast_release.py')]
  result = dict(baseline_commit=args.base, baseline_service_sha256=hashlib.sha256(source).hexdigest(),
    hashes={str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}, wheel_observation=args.wheel_observation,
    entry_fields=['speed', 'gap', 'wire'], plant_fields=['gain', 'lag_s', 'delay_s', 'push'], lead_fields=['speed', 'braking'], rows=rows)
  args.output.parent.mkdir(parents=True, exist_ok=True)
  args.output.write_text(json.dumps(result, allow_nan=False) + '\n')
  print(json.dumps(dict(cases=len(rows),
    new_floor_crossing=sum(r['candidate']['min_gap'] < 3 <= r['baseline']['min_gap'] for r in rows),
    new_incomplete=sum(not r['candidate']['stopped'] and r['baseline']['stopped'] for r in rows),
    new_creep=sum(r['candidate']['creep'] and not r['baseline']['creep'] for r in rows)), indent=2))
