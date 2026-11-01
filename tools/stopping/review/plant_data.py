"""Read-only adapters for the frozen KCS1 split and the decision's 28-stop evidence manifest."""
import hashlib
import json
from pathlib import Path
import pickle

import numpy as np

from openpilot.tools.stopping.review.kcs_plant import held

CORPUS = Path.home() / '.route_sync/corpus'
DECISION = CORPUS / 'stopping_decision_20260926'
OUTPUT = DECISION / 'sim'


def output_path(name):
  path = Path(name).expanduser().resolve()
  if not path.is_relative_to(OUTPUT.resolve()):
    raise ValueError(f'run outputs must be under {OUTPUT}')
  if not path.name.startswith('v2_'):
    raise ValueError('new outputs must have a v2_ prefix; first-run files are read-only')
  path.parent.mkdir(parents=True, exist_ok=True)
  return path


def natural_entries():
  """Saved list is authoritative; section 3 counts disagree. Keep 2129 separate."""
  manifest = json.loads((DECISION / 'stopimp_evidence/episodes.json').read_text())
  entries = []
  for route in sorted({m['route'] for m in manifest}):
    path = DECISION / 'stopimp_evidence' / ('rFF.pkl' if route == '20ff' else f'r{route}.pkl')
    # Trusted local analysis artifact, never accept an arbitrary external pickle.
    data = pickle.loads(path.read_bytes())
    rows, can = data['rows'], data['can']
    t = np.array([r['ns'] * 1e-9 for r in rows])
    v = np.array([r['v'] for r in rows])
    for m in (m for m in manifest if m['route'] == route):
      rest = int(np.argmin(abs(t - m['rest_ns'] * 1e-9)))
      cross = rest
      while cross > 0 and v[cross] < 2.5:
        cross -= 1
      cross += 1
      start = int(np.searchsorted(t, t[cross] - 2.1))
      end = int(np.searchsorted(t, t[rest] + 3.))
      if t[cross] - t[start] < 2 or np.max(np.diff(t[start:end])) > .1:
        raise ValueError(f'{route}/{m["rest_ns"]}: insufficient continuous history')
      rr = rows[start:end]
      tt = t[start:end]
      ss = np.array([r['standstill'] for r in rr])
      flags = np.flatnonzero(ss & (tt > t[cross]))
      # Natural packets have no pulse truth. Label this approximation, do not hide its uncertainty.
      truth_stop = float(tt[flags[0]] - .22) if len(flags) else float(t[rest])
      entries.append(dict(id=f'{route}_{m["rest_ns"]}', route=route, split='holdout_2129' if route == '2129' else 'natural',
        source=str(path), source_sha256=hashlib.sha256(path.read_bytes()).hexdigest(), paths=data['meta']['files'],
        settings=data['meta']['init'][0]['settings'], recorded_commit=data['meta']['init'][0]['commit'],
        rows=rr, t=tt, v=v[start:end], a=np.array([r['a'] for r in rr]), cross=float(t[cross]), rest=truth_stop,
        rest_gap=m['gap_rest'], truth='natural: vEgo integral; wheel stop estimated as standstill minus .22 s; no pulse truth',
        wire=held(can['scc12'][:, 0] * 1e-9, can['scc12'][:, 1], tt),
        upper=held(can['scc14'][:, 0] * 1e-9, can['scc14'][:, 1], tt),
        lower=held(can['scc14'][:, 0] * 1e-9, can['scc14'][:, 2], tt),
        stop_req=held(can['scc12'][:, 0] * 1e-9, can['scc12'][:, 4], tt),
        send_t=can['scc12'][:, 0] * 1e-9, send_u=can['scc12'][:, 1],
        jerk_t=can['scc14'][:, 0] * 1e-9, jerk_up=can['scc14'][:, 1], jerk_lo=can['scc14'][:, 2],
        light_t=can['tcs13'][:, 0] * 1e-9, light=can['tcs13'][:, 1], grade=0.))
  return entries


def kcs_entries(training=False):
  base = CORPUS / 'kcs1_drive1_20260926/analysis_block1_v2'
  entries, excluded = [], []
  for line in (base / 'reps.jsonl').read_text().splitlines():
    r = json.loads(line)
    if (r['rep'] < 5) != training:
      continue
    if training and not r['valid_for_fit']:
      excluded.append(dict(id=r['id'], reason='not valid_for_fit', label=r['label'], split='train'))
      continue
    stop = (r.get('terminal') or {}).get('t_stop')
    if stop is None or r['label'] not in ('complete', 'stalled', 'overridden'):
      excluded.append(dict(id=r['id'], reason='incomplete/aborted attempt', label=r['label'], split='heldout'))
      continue
    with np.load(base / r['series_file']) as z:
      s = dict(z)
    scale = r['terminal']['displacement']['m_per_pulse']
    pt, px = s['pul__t'], s['pul__count'] * scale
    t = np.arange(max(-2., pt[0] + .1), stop + 3., .01)
    v = (np.interp(t + .1, pt, px) - np.interp(t - .1, pt, px)) / .2
    cross = np.flatnonzero((t < stop) & (v >= 2.5))
    if not len(cross):
      excluded.append(dict(id=r['id'], reason='no 2.5 crossing', label=r['label'], split='heldout'))
      continue
    crossing = float(t[cross[-1]])
    # Carry the plant from >=2 s before entry; never reseed at entry or thereafter.
    start = max(0, int(np.searchsorted(t, crossing - 2.1)))
    tt = t[start:]
    entries.append(dict(id=r['id'], split='kcs_train' if training else 'kcs_heldout', label=r['label'], t=tt, v=v[start:],
      a=np.interp(tt, s['car__t'], s['car__a']), cross=crossing, rest=stop,
      distance=float(np.interp(stop, pt, px) - np.interp(crossing, pt, px)),
      truth='pulse distance scaled by pre-window wheel calibration; no acceleration offset',
      light_t=s['tcs13__t'], light=s['tcs13__BrakeLight'],
      grade=r['grade']['grade_pct_esp12'], wire=held(s['scc12__t'], s['scc12__aReqValue'], tt),
      upper=held(s['scc14__t'], s['scc14__JerkUpperLimit'], tt),
      lower=held(s['scc14__t'], s['scc14__JerkLowerLimit'], tt),
      stop_req=held(s['scc12__t'], s['scc12__StopReq'], tt),
      send_t=s['scc12__t'], send_u=s['scc12__aReqValue'], jerk_t=s['scc14__t'],
      jerk_up=s['scc14__JerkUpperLimit'], jerk_lo=s['scc14__JerkLowerLimit']))
  return entries, excluded


def extract_inputs(entries):
  """Capture complete CS plus every LongControl input directly from local rlogs.

  No planner synthesis. Latest publication at carControl is an approximation of
  controlsd subscription timing. Source gaps are rejected by each entry validator.
  """
  from openpilot.tools.lib.logreader import LogReader
  result = {}
  for route in sorted({e['route'] for e in entries}):
    selected = [e for e in entries if e['route'] == route]
    latest, settings = {}, selected[0]['settings']
    first_path = min(selected[0]['paths'], key=lambda p: int(Path(p).parent.name.rsplit('--', 1)[1]))
    cp = next(e.carParams.as_builder().to_bytes().hex() for e in LogReader(first_path) if e.which() == 'carParams')
    frames = {e['id']: [] for e in selected}
    pulses = {e['id']: [] for e in selected}
    from opendbc.can import CANParser
    parser = CANParser('hyundai_kia_generic', [(903, float('nan'))], 0)
    # Parse only logs whose monotonic interval overlaps a required seed/episode.
    for path in sorted(selected[0]['paths'], key=lambda p: int(Path(p).parent.name.rsplit('--', 1)[1])):
      reader = LogReader(path)
      first = next(e for e in reader if e.which() != 'initData')
      lo = first.logMonoTime * 1e-9
      if not any(e['t'][0] - 1 <= lo + 61 and e['t'][-1] + 1 >= lo for e in selected):
        continue
      latest.clear()
      for event in reader:
        w, now = event.which(), event.logMonoTime * 1e-9
        if w == 'can':
          targets = [e for e in selected if e['t'][0] <= now <= e['t'][-1]]
          if targets:
            for message in event.can:
              if message.src == 0 and message.address == 903:
                dat = bytes(message.dat)
                if len(dat) >= parser.message_states[903].size and 903 in parser.update([[event.logMonoTime, [(903, dat, 0)]]]):
                  row = [now] + [parser.vl[903][f'WHL_PUL_{w}'] for w in ('FL', 'FR', 'RL', 'RR')]
                  for e in targets:
                    pulses[e['id']].append(row)
        elif w == 'carParams':
          cp = event.carParams.as_builder().to_bytes().hex()
        elif w in ('carState', 'radarState', 'longitudinalPlan', 'frogpilotCarState', 'frogpilotPlan', 'selfdriveState', 'modelV2'):
          latest[w] = getattr(event, w).to_dict()
          if w == 'longitudinalPlan':
            latest['plan_valid'] = bool(event.valid)
        elif w == 'carControl':
          targets = [e for e in selected if e['t'][0] <= now <= e['t'][-1]]
          if not targets or len(latest) != 8 or cp is None:
            continue
          cs, lp, rs = latest['carState'], latest['longitudinalPlan'], latest['radarState']
          lead, lead2, cc = rs['leadOne'], rs['leadTwo'], event.carControl
          kw = dict(experimental_mode=latest['selfdriveState']['experimentalMode'], lead_status=lead['status'],
            lead_v=lead['vLead'], lead_d_rel=lead['dRel'], lead_a=lead['aLeadK'], lead_track_id=lead['radarTrackId'],
            lead_model_prob=lead['modelProb'], lead2_status=lead2['status'], lead2_v=lead2['vLead'], lead2_d_rel=lead2['dRel'],
            fcw=lp['fcw'], model_stop_d=lp['distanceToStopTargetModel'], model_should_stop=latest['modelV2']['action']['shouldStop'],
            force_coast=latest['frogpilotCarState']['forceCoast'],
            increased_stopped_distance=latest['frogpilotPlan']['increasedStoppedDistance'],
            a_target_trajectory=lp['aTargetTrajectory'] if lp['aTargetTrajectoryValid'] else None,
            freeze_integrator=cs['gasPressed'], plan_valid=latest['plan_valid'])
          frame = dict(t=now, cs=cs, kw=kw, target=lp['aTarget'], should_stop=lp['shouldStop'],
                       dts=lp['distanceToStopTarget'], active=cc.longActive, recorded=float(cc.actuators.accel),
                       authorized=bool(event.valid and cc.enabled and cc.longActive and not cc.cruiseControl.override
                                       and not cs['gasPressed'] and not cs['brakePressed'] and cs['canValid'] and not cs['canTimeout']))
          for e in targets:
            frames[e['id']].append(frame)
      print(f'extracted {Path(path).parent.name}', flush=True)
    for e in selected:
      ff = frames[e['id']]
      if not ff or ff[0]['t'] > e['cross'] - 2 or max(np.diff([f['t'] for f in ff])) > .1:
        raise ValueError(f'{e["id"]}: missing continuous >=2 s raw controller history: n={len(ff)}, '
                         + f'first={ff[0]["t"] if ff else None}, cross={e["cross"]}, '
                         + f'max_gap={max(np.diff([f["t"] for f in ff])) if len(ff)>1 else None}')
      result[e['id']] = dict(cp=cp, settings=settings, frames=ff, pulses=pulses[e['id']])
    output_path(OUTPUT / f'v2_inputs_{route}.json').write_text(json.dumps({e['id']: result[e['id']] for e in selected}, allow_nan=False))
  return result


def attach_pulse_truth(entries, inputs):
  """One frozen pulse scale from fit reps; natural stops cannot alter calibration."""
  base = CORPUS / 'kcs1_drive1_20260926/analysis_block1_v2/reps.jsonl'
  fit = [json.loads(line) for line in base.read_text().splitlines()]
  scales = [(r.get('terminal') or {}).get('displacement', {}).get('m_per_pulse') for r in fit if r['rep'] <= 4]
  scale = float(np.median([x for x in scales if x is not None]))
  for e in entries:
    pulses = np.asarray(inputs[e['id']]['pulses'])
    if len(pulses) < 10 or np.max(np.diff(pulses[:, 0])) > .1:
      raise ValueError(f'{e["id"]}: missing pulse truth')
    raw = np.round(pulses[:, 1:] / .5)
    counts = np.r_[0., np.cumsum((np.diff(raw, axis=0) % 256).mean(axis=1))]
    pt, px = pulses[:, 0], counts * scale
    frames = inputs[e['id']]['frames']
    flags = [f['t'] for f in frames if f['t'] > e['cross'] and f['cs']['standstill']]
    if not flags:
      raise ValueError(f'{e["id"]}: no standstill flag')
    edges = np.flatnonzero((np.diff(counts) > 0) & (pt[1:] <= flags[0]) & (pt[1:] > flags[0] - 1.))
    if not len(edges):
      raise ValueError(f'{e["id"]}: no terminal pulse edge')
    e['rest'] = float(pt[edges[-1] + 1])
    # At cache edges use the actual available interval, not half a stencil / .2.
    left, right = np.maximum(e['t'] - .1, pt[0]), np.minimum(e['t'] + .1, pt[-1])
    if np.any(right <= left):
      raise ValueError(f'{e["id"]}: pulse truth does not cover episode')
    speed = (np.interp(right, pt, px) - np.interp(left, pt, px)) / (right - left)
    # Keep the recorded vEgo 2.5 crossing as the controller entry, but seed physical speed with pulses.
    e['v'] = speed
    e['distance'] = float(np.interp(e['rest'], pt, px) - np.interp(e['cross'], pt, px))
    e['truth'] = f'pulse distance; fixed scale {scale:.9f} m/count from reps 1-4; last edge before standstill'
  return scale


def census_route(paths):
  """Local rlog-only census. Missing segments and stale input cannot qualify a stop."""
  from collections import deque
  import capnp
  import zstandard
  from cereal import log
  from opendbc.can import CANParser

  parser = CANParser('hyundai_kia_generic', [(1057, float('nan'))], 0)
  history, stops, failures = deque(), [], []
  radar_t = cc_t = wire_t = -float('inf')
  lead, active, wire = False, False, 0.
  was_still, last_stop = True, -float('inf')
  for path in paths:
    try:
      raw = zstandard.ZstdDecompressor().decompress(Path(path).read_bytes(), max_output_size=900_000_000)
      events = [e for e in log.Event.read_multiple_bytes(raw) if e.which() in ('radarState', 'carControl', 'sendcan', 'carState')]
      for event in sorted(events, key=lambda e: e.logMonoTime):
        kind, now = event.which(), event.logMonoTime * 1e-9
        if kind == 'radarState':
          l = event.radarState.leadOne
          radar_t, lead = now, bool(event.valid and l.status and abs(l.vLead) <= .3 and l.dRel > 0)
        elif kind == 'carControl':
          cc = event.carControl
          cc_t, active = now, bool(event.valid and cc.enabled and cc.longActive and not cc.cruiseControl.override)
        elif kind == 'sendcan':
          for msg in event.sendcan:
            if msg.src == 0 and msg.address == 1057 and len(msg.dat) >= parser.message_states[1057].size:
              if 1057 in parser.update([[event.logMonoTime, [(1057, bytes(msg.dat), 0)]]]):
                wire_t, wire = now, float(parser.vl[1057]['aReqValue'])
        elif kind == 'carState':
          cs = event.carState
          fresh = 0 <= now - radar_t <= .2 and 0 <= now - cc_t <= .1 and 0 <= now - wire_t <= .1
          clean = bool(event.valid and cs.canValid and not cs.canTimeout and not cs.brakePressed and not cs.gasPressed
                       and not cs.espActive and not cs.accFaulted and active)
          history.append((now, float(cs.vEgo), bool(cs.standstill), clean, lead, fresh, wire))
          while history and now - history[0][0] > 65:
            history.popleft()
          if cs.standstill and not was_still:
            rows = list(history)
            crosses = [i for i, r in enumerate(rows[:-1]) if r[1] >= 2.6 and now - r[0] <= 60 and r[0] > last_stop]
            reasons = []
            if not crosses:
              reasons.append('no_2p6_crossing')
            else:
              cross = crosses[-1] + 1
              start = next((i for i, r in enumerate(rows) if r[0] >= rows[cross][0] - 2.1), cross)
              warm, moving = rows[start:], rows[cross:]
              if rows[cross][0] - rows[start][0] < 2 or any(b[0] - a[0] > .1 for a, b in zip(warm, warm[1:], strict=False)):
                reasons.append('missing_history_or_gap')
              if not all(r[3] for r in warm):
                reasons.append('driver_or_inactive_or_fault')
              if not all(r[5] for r in warm):
                reasons.append('stale_input')
              if not all(r[4] for r in moving):
                reasons.append('not_stopped_lead')
              braking = next((i for i, r in enumerate(moving) if r[6] <= -.45 + 1e-9), None)
              if braking is None or any(r[6] > -.45 + 1e-9 for r in moving[braking:]):
                reasons.append('wire_released_or_never_braked')
            last_stop = now
            stops.append(dict(rest=now, path=str(path), eligible=not reasons, reasons=reasons))
          was_still = bool(cs.standstill)
    except (OSError, zstandard.ZstdError, capnp.KjException) as exc:
      failures.append(dict(path=str(path), error=str(exc)))
      history.clear()
      radar_t = cc_t = wire_t = -float('inf')
  return dict(route=Path(paths[0]).parent.name.rsplit('--', 1)[0], paths=list(map(str, paths)),
              eligible=sum(s['eligible'] for s in stops), stops=stops, failures=failures)
