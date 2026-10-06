#!/usr/bin/env python3
"""Offline measured-plant gate A and HEAD replay/simulation. Run as a module.

Examples (activate .venv first; set PYTHONDONTWRITEBYTECODE=1):
  python -m tools.stopping.review.plant_sim --prepare
  python -m tools.stopping.review.plant_sim --cells nominal --mode all
  python -m tools.stopping.review.plant_sim --cells screening --mode all
  python -m tools.stopping.review.plant_sim --cells grid --mode gate

All outputs are confined to stopping_decision_20260926/sim. No device access.
Planner: recorded aTarget, trajectory, shouldStop, dts, FCW and model-stop inputs
are replayed by time (NOT a validated counterfactual planner). Ego motion and
lead-relative gap are simulated; absolute lead path is exogenous/reconstruct.
Radar updates at 20 Hz; controller at 100 Hz; Hyundai sender at native 50 Hz.
"""
import argparse
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import pickle
import subprocess
from types import SimpleNamespace

import numpy as np

from openpilot.tools.stopping.review.kcs_plant import DT, Plant, cells, fit_gain, held, parameter_manifest
from openpilot.tools.stopping.review.plant_data import DECISION, OUTPUT, attach_pulse_truth, extract_inputs, kcs_entries, natural_entries, output_path
from openpilot.tools.stopping.review.stop_harness import reconstruct


def reversals(t, values, threshold=.1, include_open=False):
  """Hysteretic release >=threshold followed by rebuild >=threshold, including endpoints."""
  if len(values) == 0:
    return []
  if not np.all(np.isfinite(values)):
    raise ValueError('nonfinite reversal input')
  low, peak, released, events = 0, 0, False, []
  for i, value in enumerate(values):
    if not released:
      if value < values[low]:
        low = i
      if value - values[low] >= threshold - 1e-9:
        released, peak = True, i
    else:
      if value > values[peak]:
        peak = i
      if values[peak] - value >= threshold - 1e-9:
        events.append(dict(start=float(t[low]), peak=float(t[peak]), rebuild=float(t[i]),
                           release=float(values[peak] - values[low]), depth=float(values[peak] - value)))
        low, released = i, False
  if include_open and released:
    events.append(dict(start=float(t[low]), peak=float(t[peak]), rebuild=None,
                       release=float(values[peak] - values[low]), depth=None))
  return events


def metrics(trace):
  t, v, a, u, gap, off = (np.asarray(trace[k]) for k in ('t', 'v', 'a', 'u', 'gap', 'off'))
  if not len(t) or not all(np.all(np.isfinite(x)) for x in (t, v, a, u)):
    raise ValueError('empty/nonfinite trace')
  if np.any(np.diff(t) <= 0):
    raise ValueError('non-increasing metric times')
  stop = np.flatnonzero(v <= 1e-6)
  rest = int(stop[0]) if len(stop) else None
  band = (v <= 2.5) & (v >= .15)
  if rest is not None:
    band[rest:] = False
  moving = np.flatnonzero(v[:rest] > 0) if rest is not None else np.array([], dtype=int)
  last = int(moving[-1]) if len(moving) else None
  tail = v[rest + 1:] if rest is not None else []
  command_events, accel_events = reversals(t[band], u[band]), reversals(t[band], a[band])
  return dict(command_reversals=len(command_events), accel_reversals=len(accel_events),
    command_events=command_events, accel_events=accel_events, brake_off_s=float(np.sum(off & (v > 0)) * DT),
    rest_gap=float(gap[rest]) if rest is not None and np.isfinite(gap[rest]) else None,
    min_gap=float(np.nanmin(gap)) if np.any(np.isfinite(gap)) else None,
    a_stop=float(a[last]) if last is not None else None, jerk_proxy=5 * abs(float(a[last])) if last is not None else None,
    stop_time=float(t[rest] - t[0]) if rest is not None else None, incomplete=rest is None,
    stationary_validation='UNVALIDATED: rollback, breakaway and hold security are not identified',
    creep=bool(np.any(np.asarray(tail) > .05)), relaunch=bool(np.any(np.asarray(tail) >= .5)),
    a_plan_bind_frames=int(sum(trace.get('plan_bind', []))), barrier_bind_frames=int(sum(trace.get('bar_bind', []))),
    blocked_release_frames=int(sum(trace.get('blocked', []))))


def summarize_metrics(rows):
  rest = [r['rest_gap'] for r in rows if r['rest_gap'] is not None]
  return dict(stationary_validation='UNVALIDATED', n=len(rows), completed=sum(not r['incomplete'] for r in rows), rest_gap_count=len(rest),
    rest_p10_median_p90=np.percentile(rest, [10, 50, 90]).tolist() if rest else None,
    below3=sum(x < 3 for x in rest), below3p5=sum(x < 3.5 for x in rest), above5p5=sum(x > 5.5 for x in rest),
    incomplete=sum(r['incomplete'] for r in rows), creep=sum(r['creep'] for r in rows), relaunch=sum(r['relaunch'] for r in rows),
    zero_command_reversals=sum(r['command_reversals'] == 0 for r in rows))


def free_roll(entry, cell, gain, traces=None):
  e = entry
  t = np.arange(e['t'][0], e['t'][-1], DT)
  u = held(e['send_t'], e['send_u'], t)
  up, lo = held(e['jerk_t'], e['jerk_up'], t), held(e['jerk_t'], e['jerk_lo'], t)
  vrec = np.interp(t, e['t'], e['v'])
  stops = held(e['t'], e['stop_req'], t)
  plant = Plant(vrec[0], e['a'][0], u[0], cell, gain, e['grade'])
  entry_i = int(np.searchsorted(t, e['cross']))
  vv, aa, xx, off, physical = [], [], [], [], []
  if e['cross'] - t[0] < 2. or e['send_t'][0] > t[0]:
    raise ValueError('insufficient >=2 s SCC12 warm-up')
  # One motion seed at the START of warm-up. Regime and speed evolve freely.
  for i, _now in enumerate(t):
    if i == entry_i:
      plant.x = 0.
    plant.step(float(u[i]), float(up[i]), float(lo[i]), bool(stops[i]))
    vv.append(plant.v)
    aa.append(plant.a_ego)
    xx.append(plant.x)
    off.append(plant.off)
    physical.append(plant.a)
  vv, aa, xx = map(np.asarray, (vv, aa, xx))
  rest_i = np.flatnonzero((np.arange(len(t)) >= entry_i) & (vv == 0))
  predicted_rest = int(rest_i[0]) if len(rest_i) else None
  mask = (t >= e['cross']) & (t <= e['rest'])
  if 'distance' in e:
    truth_distance = e['distance']
  else:
    truth_distance = float(np.trapezoid(np.interp(t[mask], e['t'], e['v']), t[mask]))
  distance_error = float(xx[predicted_rest] - truth_distance) if predicted_rest is not None else None
  rest_error = -distance_error if distance_error is not None else None
  if 'rows' in e and not any(r['lead'] for r in e['rows']):
    rest_error = None
  if rest_error is not None and 'rows' in e:
    if '_lead_path' not in e:
      frames = [dict(t=now, v=v, lv=r['lv'], ld=r['gap'], ls=r['lead'])
                for now, v, r in zip(e['t'], e['v'], e['rows'], strict=True)]
      e['_lead_path'] = reconstruct(dict(seg=e['id'], frames=frames))
    lt, _, lead, _ = e['_lead_path']
    rest_error += float(np.interp(t[predicted_rest], lt, lead) - np.interp(e['rest'], lt, lead))
  if traces is not None:
    traces.update(t=t, v=vv, a=aa, x=xx, off=np.asarray(off), physical=np.asarray(physical), u=u)
  return dict(id=e['id'], split=e['split'], truth=e['truth'], complete=predicted_rest is not None,
    stationary_validation='UNVALIDATED', distance_error=distance_error, rest_error=rest_error,
    time_error=float(t[predicted_rest] - e['rest']) if predicted_rest is not None else None,
    aego_rmse=float(np.sqrt(np.mean((aa[mask] - np.interp(t[mask], e['t'], e['a'])) ** 2))),
    end_speed=float(vv[-1]), distance_pass=distance_error is not None and abs(distance_error) <= .2,
    floor_pass=distance_error is not None and abs(distance_error) <= .075)


def regime_diagnosis(entry, trace):
  """Time-local error accounting, not a causal attribution: carried speed error persists after rebuild."""
  t = trace['t']
  moving = (t >= entry['cross']) & (t <= entry['rest'])
  vrec = np.interp(t, entry['t'], entry['v'])
  arec = np.interp(t, entry['t'], entry['a'])
  # Wire-history proxy is independent of the simulated speed/regime.
  reference, released = float(trace['u'][0]), False
  wire_off = []
  for u, v in zip(trace['u'], vrec, strict=True):
    if released:
      reference = max(reference, u)
      if u <= reference - .08:
        released, reference = False, u
    else:
      reference = min(reference, u)
      if u >= reference + .10 and v < 2.6:
        released, reference = True, u
    wire_off.append(released)
  result = {}
  for source, off in dict(plant=trace['off'], lights=held(entry['light_t'], entry['light'], t) < .5,
                          wire=np.asarray(wire_off)).items():
    result[source] = {}
    for regime, mask in dict(engaged=moving & ~off, brake_off=moving & off).items():
      error = trace['a'][mask] - arec[mask]
      result[source][regime] = dict(seconds=float(sum(mask) * DT),
        distance_error_m=float(sum((trace['v'] - vrec)[mask]) * DT),
        aego_signed_integral=float(sum(error) * DT), aego_absolute_integral=float(sum(abs(error)) * DT),
        aego_p50=float(np.median(abs(error))) if len(error) else None,
        aego_p90=float(np.percentile(abs(error), 90)) if len(error) else None)
  stop = np.flatnonzero((t >= entry['cross']) & (trace['v'] == 0))
  end = t[stop[0]] if len(stop) else t[-1]
  result['tail_distance_m'] = float(sum(trace['v'][(t > entry['rest']) & (t <= end)]) * DT)
  return result


def gate_summary(rows):
  result = {}
  for split in sorted({r['split'] for r in rows}):
    rr = [r for r in rows if r['split'] == split]
    de = [abs(r['distance_error']) for r in rr if r['distance_error'] is not None]
    re = [abs(r['rest_error']) for r in rr if r['rest_error'] is not None]
    te = [abs(r['time_error']) for r in rr if r['time_error'] is not None]
    ar = [r['aego_rmse'] for r in rr]
    p90, a90 = float(np.percentile(de, 90)) if de else None, float(np.percentile(ar, 90))
    result[split] = dict(n=len(rr), incomplete=sum(not r['complete'] for r in rr),
      distance_median=float(np.median(de)) if de else None, distance_max=max(de, default=None),
      distance_p90=p90, rest_gap_count=len(re), rest_p90=float(np.percentile(re, 90)) if re else None,
      time_median=float(np.median(te)) if te else None, time_p90=float(np.percentile(te, 90)) if te else None,
      aego_rmse_p90=a90, pass_02=sum(r['distance_pass'] for r in rr), pass_0075=sum(r['floor_pass'] for r in rr),
      gate_pass=len(de) == len(rr) and (all(x <= .2 for x in de) if split == 'kcs_heldout' else float(np.percentile(re, 90)) <= .35 and a90 <= .20))
  return result


def namespace(data):
  return SimpleNamespace(**{k: namespace(v) if isinstance(v, dict) else v for k, v in data.items()})


def simulate(entry, inputs, cell, gain, recorded=False, controller_type=None, frame_observer=None):
  from cereal import car
  from openpilot.common.swaglog import cloudlog, ipchandler
  cloudlog.removeHandler(ipchandler)  # offline run: no IPC log writes outside the output tree
  from openpilot.selfdrive.controls.lib import stopping_flags
  from openpilot.selfdrive.controls.lib.longcontrol import LongControl
  from openpilot.selfdrive.controls.lib.stopping_service import StoppingService
  from openpilot.selfdrive.controls.lib.stop_context import StopContext
  from openpilot.selfdrive.controls.lib.stopping_telemetry import StoppingTelemetry
  from opendbc.car.hyundai.tests.test_can_bounds_fork import make_controller, run_frame, get_signal
  from opendbc.car.hyundai.interface import CarInterface

  original = stopping_flags.IDENTIFICATION_HOOK
  stopping_flags.IDENTIFICATION_HOOK = False
  try:
    with car.CarParams.from_bytes(bytes.fromhex(inputs['cp'])) as reader:
      cp = reader.as_builder()
    if any(cp.longitudinalTuning.kpV) or any(cp.longitudinalTuning.kiV):
      raise ValueError('expected logged kp=ki=0')
    lc = (controller_type or LongControl)(cp)
    assert isinstance(lc._service_shadow_svc, StoppingService) and isinstance(lc._service_shadow_ctx, StopContext)
    lc._service_shadow_tel = StoppingTelemetry(log_fn=lambda **kw: None)
    settings = inputs['settings']
    toggles = SimpleNamespace(vEgoStarting=cp.vEgoStarting, vEgoStopping=cp.vEgoStopping, startAccel=cp.startAccel,
      human_acceleration=settings.get('HumanAcceleration') == '1' and settings.get('LongitudinalTune') == '1',
      force_coast_strength=float(settings.get('CEForceCoastStrength', '1')))
    sender, _ = make_controller(cp)
    debug = {}
    update = lc._service_shadow_svc.update

    def capture(**kwargs):
      result = update(**kwargs)
      debug.clear()
      debug.update(result.debug)
      return result

    lc._service_shadow_svc.update = capture
    frames = inputs['frames']
    ft = np.array([f['t'] for f in frames])
    t = ft if recorded else np.arange(ft[0], ft[-1], DT)
    indices = np.arange(len(ft)) if recorded else np.clip(np.searchsorted(ft, t, side='right') - 1, 0, len(ft) - 1)
    # Reuse the established recorded ego + radar reconstruction; no residual plant correction.
    ep = dict(seg=entry['id'], frames=[dict(t=f['t'], v=f['cs']['vEgo'], lv=f['kw']['lead_v'],
      ld=f['kw']['lead_d_rel'], ls=f['kw']['lead_status']) for f in frames])
    if any(f['ls'] for f in ep['frames']):
      if '_lead_path' not in inputs:
        inputs['_lead_path'] = reconstruct(ep)
      lt, ego, lead, jitter = inputs['_lead_path']
    else:
      lt = ft
      vv = np.array([f['v'] for f in ep['frames']])
      ego = np.r_[0., np.cumsum((vv[:-1] + vv[1:]) / 2 * np.diff(ft))]
      lead, jitter = np.full(len(ft), np.nan), None
    if entry['cross'] - t[0] < 2.:
      raise ValueError('insufficient controller warm-up')
    crossing = int(np.searchsorted(t, entry['cross']))
    ego_origin = float(np.interp(t[crossing], lt, ego))
    sent = float(held(entry['send_t'], entry['send_u'], t[0]))
    plant = Plant(frames[0]['cs']['vEgo'], frames[0]['cs']['aEgo'], sent, cell, gain, entry['grade'])
    lc.last_output_accel = frames[0]['recorded']
    trace = {k: [] for k in ('t', 'v', 'a', 'u', 'gap', 'off', 'plan_bind', 'bar_bind', 'blocked')}
    twin, rec, twin_t, twin_v, owned = [], [], [], [], []
    upper, lower, stop_req = 3., 5., False
    radar = None
    warm_u = held(entry['send_t'], entry['send_u'], t)
    warm_up = held(entry['jerk_t'], entry['jerk_up'], t)
    warm_lo = held(entry['jerk_t'], entry['jerk_lo'], t)
    for k, (now, idx) in enumerate(zip(t, indices, strict=True)):
      f = frames[idx]
      cs = namespace(f['cs'])
      kw = dict(f['kw'])
      if k == crossing:
        plant.x = 0.
      if k >= crossing and not recorded:
        cs.vEgo, cs.aEgo, cs.vEgoRaw, cs.standstill = plant.v_ego, plant.a_ego, plant.raw, plant.standstill
        cs.cruiseState.standstill = plant.standstill
        if k % 5 == 0 or radar is None:
          radar = {key: kw[key] for key in ('lead_status', 'lead_v', 'lead_a', 'lead_track_id', 'lead_model_prob',
                                           'lead2_status', 'lead2_v', 'lead2_d_rel')}
          radar['lead_d_rel'] = float(np.interp(now, lt, lead) - ego_origin - plant.x) if kw['lead_status'] else 0.
          # Same ego displacement correction for second lead, whose recorded path is also exogenous.
          radar['lead2_d_rel'] += float(np.interp(now, lt, ego) - ego_origin - plant.x)
        kw.update(radar)
      previous_state = lc.long_control_state
      if not f['active']:
        lc.reset()
      kw['request_time'] = float(now)
      debug.clear()
      limits = CarInterface.get_pid_accel_limits(cp, cs.vEgo, cs.vCruise / 3.6)
      previous_command = lc.last_output_accel
      command = float(lc.update(f['active'], cs, f['target'], f['should_stop'], f['dts'], limits, toggles, **kw))
      lc.observe_accel_request(command, float(now), authorized=f['authorized'])
      if lc._service_live_disabled or lc._service_shadow_disabled:
        raise RuntimeError(f'{entry["id"]}: controller service disabled during replay')
      _, messages = run_frame(sender, dict(accel=command, state=previous_state, v_ego=cs.vEgo,
                                          a_ego=cs.aEgo, long_active=f['active'], gas_pressed=cs.gasPressed))
      if 0x421 in messages:
        sent = get_signal('SCC12', 'aReqValue', messages[0x421])
        stop_req = bool(get_signal('SCC12', 'StopReq', messages[0x421]))
      if 0x389 in messages:
        upper = get_signal('SCC14', 'JerkUpperLimit', messages[0x389])
        lower = get_signal('SCC14', 'JerkLowerLimit', messages[0x389])
      if frame_observer is not None:
        frame_observer(lc, f, command, sent)
      if k < crossing or recorded:
        plant.step(float(warm_u[k]), float(warm_up[k]), float(warm_lo[k]))
      else:
        plant.step(sent, upper, lower, stop_req)
      if k >= crossing:
        twin.append(command)
        rec.append(f['recorded'])
        twin_t.append(float(now - t[crossing]))
        twin_v.append(f['cs']['vEgo'])
        owned.append(lc._service_live_owning)
        values = (float(now - t[crossing]), plant.v, plant.a, sent,
                  float(np.interp(now + DT, lt, lead) - ego_origin - plant.x), plant.off)
        for key, value in zip(('t', 'v', 'a', 'u', 'gap', 'off'), values, strict=True):
          trace[key].append(value)
        demands = [debug.get(key) for key in ('a_phase', 'a_kin', 'a_plan', 'a_monitor', 'a_barrier')]
        minimum = min((x for x in demands if x is not None), default=0.)
        trace['plan_bind'].append(debug.get('a_plan') is not None and debug['a_plan'] <= minimum + 1e-9
                                  and debug.get('attr_live_release', 0.) == 0.)
        trace['bar_bind'].append(debug.get('a_barrier') is not None and debug['a_barrier'] <= minimum + 1e-9)
        trace['blocked'].append(debug.get('a_phase', previous_command) > previous_command + .001
                                and command <= previous_command + 1e-9 and bool(debug.get('safety_binding')))
    twin_v, twin_t, twin, rec = map(np.asarray, (twin_v, twin_t, twin, rec))
    band = (twin_v >= .15) & (twin_v <= 2.5) & (twin_t <= entry['rest'] - t[crossing])
    own = band & np.asarray(owned)
    result = dict(id=entry['id'], split=entry['split'], lead_path_jitter=jitter,
      recorded_commit=entry['recorded_commit'], command_mae=float(np.mean(abs(twin[band] - rec[band]))),
      owned_command_mae=float(np.mean(abs(twin[own] - rec[own]))) if np.any(own) else None,
      recorded_events=reversals(twin_t[band], rec[band], include_open=True),
      head_events=reversals(twin_t[band], twin[band], include_open=True),
      stationary_validation='UNVALIDATED', hold_unknown=plant.hold_unknown, cp_delay=float(cp.longitudinalActuatorDelay))
    if not recorded:
      result.update(metrics(trace))
    return result
  finally:
    stopping_flags.IDENTIFICATION_HOOK = original


def run_v2_gate():
  """Execute the frozen V2 protocol. Deliberately has no candidate or closed-loop branch."""
  spec = OUTPUT / 'v2_GATE_SPEC.md'
  if not spec.is_file():
    raise ValueError('preregister v2_GATE_SPEC.md first')
  gain, provenance = fit_gain(pickle.loads((DECISION / 'stopimp_plant/dg/gain_rows.pkl').read_bytes()))
  training, train_excluded = kcs_entries(training=True)
  scores = {}
  for name, cell in cells().items():
    rows = [free_roll(e, cell, gain) for e in training]
    scores[name] = float(np.mean([abs(r['distance_error']) + .5 * r['aego_rmse'] + .2 * abs(r['time_error'])
                                  for r in rows])) if all(r['complete'] for r in rows) else None
  usable = {n: x for n, x in scores.items() if x is not None}
  if not usable:
    raise ValueError('no complete training cell: do not evaluate held-out data')
  selected = min(usable, key=usable.get)
  cell = cells()[selected]
  result = dict(head=subprocess.check_output(['git', 'rev-parse', 'HEAD'], text=True).strip(),
                source_hashes={name: hashlib.sha256(Path(__file__).with_name(name).read_bytes()).hexdigest()
                               for name in ('plant_sim.py', 'plant_data.py', 'kcs_plant.py')},
                spec_sha256=hashlib.sha256(spec.read_bytes()).hexdigest(), selected=selected,
                cell=asdict(cell), fit_scores=scores, training_ids=[e['id'] for e in training],
                train_excluded=train_excluded, fitted_gain=gain, gain_provenance=provenance)
  output_path(OUTPUT / 'v2_selection.json').write_text(json.dumps(result, indent=2))
  inputs = json.loads((OUTPUT / 'inputs.json').read_text())
  entries = natural_entries()
  attach_pulse_truth(entries, inputs)
  result['reserved_entries'] = [e['id'] for e in entries if e['route'] not in ('2086', '2129')]
  all_entries = entries
  entries = [e for e in entries if e['route'] in ('2086', '2129')]
  for e in entries:
    e['split'] = 'natural_validation' if e['route'] == '2086' else 'holdout_2129'
  kcs, result['excluded_kcs'] = kcs_entries()
  rows, acceleration = [], {}
  for e in kcs + entries:
    trace = {}
    r = free_roll(e, cell, gain, trace)
    t, u = trace['t'], trace['u']
    vrec = np.interp(t, e['t'], e['v'])
    braking = np.flatnonzero((vrec < 2.6) & (u <= -.45 + 1e-9) & (t <= e['rest']))
    start = int(braking[0]) if len(braking) else None
    r['regime'] = ('engaged' if np.all(u[start:][t[start:] <= e['rest']] <= -.45 + 1e-9) else 'brake_off') if start is not None else 'unclassified'
    mask = (t >= e['cross']) & (t <= e['rest'])
    errors = abs(trace['a'][mask] - np.interp(t[mask], e['t'], e['a']))
    acceleration[r['id']] = errors.tolist()
    r['aego_p50'], r['aego_p90'] = np.percentile(errors, [50, 90]).tolist()
    r['diagnosis'] = regime_diagnosis(e, trace)
    recorded_off = held(e['light_t'], e['light'], t) < .5
    events = {}
    for name, values in dict(recorded=recorded_off, plant=trace['off']).items():
      indices = np.flatnonzero(mask)
      changes = np.diff(values[indices].astype(int))
      events[name] = dict(release=t[indices[1:][changes == 1]].tolist(), rebuild=t[indices[1:][changes == -1]].tolist(),
                          initial_off=bool(values[indices[0]]))
    r['events'] = events
    r['event_count_match'] = all(len(events['recorded'][k]) == len(events['plant'][k]) for k in ('release', 'rebuild'))
    r['event_timing_errors'] = [abs(a - b) for k in ('release', 'rebuild')
                              for a, b in zip(events['recorded'][k], events['plant'][k], strict=True)] if r['event_count_match'] else []
    r['warmup_s'] = float(e['cross'] - t[0])
    r['seed_speed_error'] = float(trace['v'][np.searchsorted(t, e['cross'])] - np.interp(e['cross'], e['t'], e['v']))
    rows.append(r)
  result['rows'], result['summaries'] = rows, {}
  for split in sorted({r['split'] for r in rows}):
    result['summaries'][split] = {}
    for regime in ('engaged', 'brake_off', 'unclassified'):
      rr = [r for r in rows if r['split'] == split and r['regime'] == regime]
      if not rr:
        result['summaries'][split][regime] = dict(n=0, gate_pass=False, reason='no evidence')
        continue
      dist = [abs(r['distance_error']) for r in rr if r['distance_error'] is not None]
      rest = [r['rest_error'] for r in rr if r['rest_error'] is not None]
      times = [abs(r['time_error']) for r in rr if r['time_error'] is not None]
      ae = [x for r in rr for x in acceleration[r['id']]]
      timing = [x for r in rr for x in r['event_timing_errors']]
      summary = dict(n=len(rr), incomplete=sum(not r['complete'] for r in rr),
        aego_p50=float(np.percentile(ae, 50)), aego_p90=float(np.percentile(ae, 90)),
        distance_p90=float(np.percentile(dist, 90)) if dist else None, distance_max=max(dist, default=None),
        rest_signed_median=float(np.median(rest)) if rest else None,
        rest_p90=float(np.percentile(np.abs(rest), 90)) if rest else None,
        rest_p99=float(np.percentile(np.abs(rest), 99)) if rest else None,
        rest_time_p90=float(np.percentile(times, 90)) if times else None,
        event_count_matches=sum(r['event_count_match'] for r in rr),
        event_timing_p90=float(np.percentile(timing, 90)) if timing else None,
        kcs_within_02=sum(d <= .2 for d in dist))
      complete = len(dist) == len(rr) and len(rest) == len(rr)
      summary['numerical_pass'] = bool(complete and summary['aego_p50'] <= .08 and summary['aego_p90'] <= .20
        and summary['distance_p90'] <= .35 and summary['distance_max'] <= .60
        and abs(summary['rest_signed_median']) <= .10 and summary['rest_p90'] <= .35 and summary['rest_p99'] <= .60
        and summary['rest_time_p90'] <= .40 and summary['event_count_matches'] == len(rr)
        and (not timing or summary['event_timing_p90'] <= .20)
        and (split != 'kcs_heldout' or all(d <= .2 for d in dist)))
      summary['gate_pass'] = False  # >=100 route-grouped held-out stops are not in this frozen cohort.
      summary['count_gate'] = 'FAIL: fewer than 100 held-out stops; census entries are not validated rollouts'
      result['summaries'][split][regime] = summary
  result['twin'] = [simulate(e, inputs[e['id']], cell, gain, recorded=True) for e in all_entries]
  for r in result['twin']:
    r['event_count_match'] = (len(r['recorded_events']) == len(r['head_events'])
                              and sum(e['rebuild'] is not None for e in r['recorded_events'])
                              == sum(e['rebuild'] is not None for e in r['head_events']))
    r['gate2_pass'] = r['command_mae'] <= .01 and r['event_count_match']
  result['status'] = 'STOP after step 3: engaged gate A fails; no checker, generator, HEAD or candidate simulation'
  output_path(OUTPUT / 'v2_gate.json').write_text(json.dumps(result, indent=2, allow_nan=False))
  print(json.dumps(dict(selected=selected, summaries=result['summaries']), indent=2), flush=True)


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('--census', action='store_true')
  parser.add_argument('--v2-gate', action='store_true')
  parser.add_argument('--prepare', action='store_true')
  parser.add_argument('--cells', default='screening', help='nominal, screening, grid, or comma-separated named cells')
  parser.add_argument('--mode', choices=('gate', 'twin', 'sim', 'all'), default='all')
  parser.add_argument('--output', type=Path)
  args = parser.parse_args()
  if args.census:
    from concurrent.futures import ProcessPoolExecutor, as_completed
    from openpilot.tools.stopping.review.plant_data import census_route
    root = Path.home() / '.route_sync/data/media/0/realdata'
    routes = {}
    for path in sorted(root.glob('*/rlog.zst')):
      if 0x2031 <= int(path.parent.name.split('--')[0], 16) <= 0x2128:
        routes.setdefault(path.parent.name.rsplit('--', 1)[0], []).append(path)
    rows = []
    with ProcessPoolExecutor(max_workers=4) as pool:
      jobs = [pool.submit(census_route, sorted(paths, key=lambda p: int(p.parent.name.rsplit('--', 1)[1])))
              for paths in routes.values()]
      for job in as_completed(jobs):
        row = job.result()
        rows.append(row)
        output_path(OUTPUT / f'v2_census_{row["route"]}.json').write_text(json.dumps(row, indent=2))
        print(row['route'], row['eligible'], 'eligible;', len(row['failures']), 'failed files', flush=True)
    output_path(OUTPUT / 'v2_census.json').write_text(json.dumps(dict(routes=rows, eligible=sum(r['eligible'] for r in rows)), indent=2))
    return
  if args.v2_gate:
    run_v2_gate()
    return
  entries = natural_entries()
  if args.prepare:
    target = output_path(OUTPUT / 'v2_inputs.json')
    target.write_text(json.dumps(extract_inputs(entries), allow_nan=False))
    return
  inputs = json.loads((OUTPUT / 'inputs.json').read_text())
  pulse_scale = attach_pulse_truth(entries, inputs)
  available = {**cells(), **cells(True)}
  selected = cells(True) if args.cells == 'grid' else cells() if args.cells == 'screening' else {n: available[n] for n in args.cells.split(',')}
  gain_rows = pickle.loads((DECISION / 'stopimp_plant/dg/gain_rows.pkl').read_bytes())
  gain, provenance = fit_gain(gain_rows)
  output_path(OUTPUT / 'v2_parameters.json').write_text(json.dumps(dict(**parameter_manifest(), fitted_gain=gain, windows=provenance), indent=2))
  result = dict(pulse_scale=pulse_scale, head=subprocess.check_output(['git', 'rev-parse', 'HEAD'], text=True).strip(),
    cells={k: asdict(v) for k, v in selected.items()},
    sources={str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in
             (Path(__file__), Path(__file__).with_name('kcs_plant.py'), Path(__file__).with_name('plant_data.py'),
              *[Path('selfdrive/controls/lib') / f'{name}.py' for name in
                ('longcontrol', 'stopping_service', 'stop_context', 'stopping_flags')],
              Path('opendbc_repo/opendbc/car/hyundai/carcontroller.py'))},
    manifest=[{k: e[k] for k in ('id', 'split', 'source', 'source_sha256', 'recorded_commit')} for e in entries],
    cohort_note='Saved 28 evidence episodes = 26 natural + 2 heldout 2129; section 3 route/count text disagrees.',
    limits=[__doc__, 'Plant gain fit uses reps 1-4 only. No offset fit. Unknown holds/positive actuation remain assumptions.',
            'Natural pulse scale is the global reps 1-4 median; no per-stop scale or residual fit.',
            'HEAD differs from historical route commits; command agreement is measured, never asserted.',
            'Simulation ends at recorded window end: incomplete episodes retained; no invented planner tail.'])
  if args.mode in ('gate', 'all'):
    kcs, excluded = kcs_entries()
    result['excluded_kcs'] = excluded
    result['gate'] = {}
    for name, cell in selected.items():
      rows = [free_roll(e, cell, gain) for e in kcs + entries]
      result['gate'][name] = dict(summary=gate_summary(rows), rows=rows)
      print('gate', name, flush=True)
  if args.mode in ('twin', 'sim', 'all'):
    if args.mode in ('twin', 'all'):
      result['twin'] = [simulate(e, inputs[e['id']], cells()['nominal'], gain, recorded=True) for e in entries]
      print('twin', len(result['twin']), flush=True)
    if args.mode in ('sim', 'all'):
      result['sim'] = {}
      for name, cell in selected.items():
        rows = [simulate(e, inputs[e['id']], cell, gain) for e in entries]
        result['sim'][name] = dict(summary=summarize_metrics(rows), rows=rows)
        print('sim', name, flush=True)
  target = output_path(args.output or OUTPUT / f'v2_{args.mode}_{args.cells}.json')
  target.write_text(json.dumps(result, allow_nan=False, indent=2))
  print(target)


if __name__ == '__main__':
  main()
