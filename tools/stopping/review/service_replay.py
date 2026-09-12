#!/usr/bin/env python3
"""Recorded-input LongControl comparison. This does not predict motion after a changed command.

Replay complete rlogs from one route, in segment order, using the latest logged inputs at each
carControl publication. Compare GOVERNOR_RECOVERY_BRAKE off/on. Preserve actual carState and radar
geometry in both arms; command agreement validates input assembly, not a counterfactual plant.
"""
import argparse
import hashlib
import json
import math
import subprocess
from pathlib import Path
from types import SimpleNamespace

import zstandard
from cereal import log

from openpilot.selfdrive.controls.lib import stopping_flags
from openpilot.selfdrive.controls.lib.longcontrol import LongControl
from openpilot.selfdrive.controls.lib.stopping_telemetry import StoppingTelemetry


def replay(paths, *, controller_types=None, recovery_modes=(False, True)):
  routes = {p.parent.name.rsplit('--', 1)[0] for p in paths}
  if len(routes) != 1:
    raise ValueError('one route per replay')
  latest, stamps, sources, controllers, rows, init = {}, {}, [], [], [], []
  settings = {}
  previous_segment = None
  discontinuities = []
  skipped = 0
  flags = {k: v for k, v in vars(stopping_flags).items() if k.isupper()}
  original = stopping_flags.GOVERNOR_RECOVERY_BRAKE
  try:
    for path in sorted(paths, key=lambda p: int(p.parent.name.rsplit('--', 1)[1])):
      segment = int(path.parent.name.rsplit('--', 1)[1])
      if previous_segment is not None and segment != previous_segment + 1:
        discontinuities.append(path.parent.name)
        latest.clear()
        stamps.clear()
        controllers = []  # an absent interval cannot carry controller state into the next block
      previous_segment = segment
      raw = path.read_bytes()
      dec = zstandard.ZstdDecompressor().decompressobj()
      decoded = dec.decompress(raw)
      if not dec.eof or dec.unused_data:
        raise ValueError(f'{path}: incomplete or multi-frame log')
      sources.append({'path': str(path), 'bytes': len(raw), 'sha256': hashlib.sha256(raw).hexdigest()})
      for e in log.Event.read_multiple_bytes(decoded):
        w, ns = e.which(), int(e.logMonoTime)
        if w == 'initData':
          settings = {kv.key: bytes(kv.value).decode() for kv in e.initData.params.entries
                      if kv.key in {'HumanAcceleration', 'LongitudinalTune', 'CEForceCoastStrength'}}
          init.append({'segment': path.parent.name, 'commit': e.initData.gitCommit, 'mono_ns': ns, 'settings': settings})
        elif w == 'carParams' and not controllers:
          cp = e.carParams.as_builder()
          controllers = [cls(cp) for cls in (controller_types or (LongControl, LongControl))]
          for lc in controllers:
            lc._service_shadow_tel = StoppingTelemetry(log_fn=lambda **kw: None)
          toggles = SimpleNamespace(vEgoStarting=cp.vEgoStarting, vEgoStopping=cp.vEgoStopping, startAccel=cp.startAccel,
            human_acceleration=settings.get('HumanAcceleration') == '1' and settings.get('LongitudinalTune') == '1',
            force_coast_strength=float(settings['CEForceCoastStrength']))
        elif w in ('carState', 'radarState', 'longitudinalPlan', 'frogpilotCarState', 'frogpilotPlan', 'selfdriveState', 'modelV2'):
          latest[w] = getattr(e, w).as_builder()
          stamps[w] = ns
          if w == 'longitudinalPlan':
            latest['plan_valid'] = bool(e.valid)
          elif w == 'carState':
            latest['car_valid'] = bool(e.valid and e.carState.canValid)
        elif w == 'carControl':
          if len(latest) != 9 or not controllers:
            skipped += 1
            continue
          cs, lp, rs = latest['carState'], latest['longitudinalPlan'], latest['radarState']
          lead, lead2 = rs.leadOne, rs.leadTwo
          cc = e.carControl
          values = []
          for enabled, lc in zip(recovery_modes, controllers, strict=True):
            stopping_flags.GOVERNOR_RECOVERY_BRAKE = enabled
            if not cc.longActive:
              lc.reset()
            wire = lc.update(cc.longActive, cs, lp.aTarget, lp.shouldStop, lp.distanceToStopTarget, (-3.5, 2.0), toggles,
              experimental_mode=latest['selfdriveState'].experimentalMode, lead_status=lead.status, lead_v=lead.vLead,
              lead_d_rel=lead.dRel, lead_a=lead.aLeadK, lead_track_id=lead.radarTrackId, lead_model_prob=lead.modelProb,
              lead2_status=lead2.status, lead2_v=lead2.vLead, lead2_d_rel=lead2.dRel, fcw=lp.fcw,
              model_stop_d=lp.distanceToStopTargetModel, model_should_stop=latest['modelV2'].action.shouldStop,
              force_coast=latest['frogpilotCarState'].forceCoast,
              increased_stopped_distance=latest['frogpilotPlan'].increasedStoppedDistance,
              a_target_trajectory=lp.aTargetTrajectory if lp.aTargetTrajectoryValid else None,
              freeze_integrator=cs.gasPressed, plan_valid=latest['plan_valid'])
            values.append({'wire': float(wire), 'owning': lc._service_live_owning,
                           'phase': lc._service_shadow_svc.phase.name, 'coast': lc._service_shadow_ctx._a_coast,
                           'context_gap': lc._service_shadow_ctx._d_gap, 'gap_source': lc._service_shadow_ctx._gap_source})
          rows.append({'mono_ns': ns, 'v': cs.vEgo, 'a': cs.aEgo, 'lead': bool(lead.status), 'lv': lead.vLead,
                       'gap': lead.dRel, 'active': bool(cc.longActive), 'brake': cs.brakePressed, 'gas': cs.gasPressed,
                       'valid': bool(e.valid and latest['car_valid'] and latest['plan_valid']),
                       'recorded_wire': cc.actuators.accel, 'off': values[0], 'on': values[1],
                       'ages_ms': {k: (ns - stamps[k]) / 1e6 for k in ('carState', 'radarState', 'longitudinalPlan')}})
  finally:
    stopping_flags.GOVERNOR_RECOVERY_BRAKE = original
  if any(b['mono_ns'] <= a['mono_ns'] for a, b in zip(rows, rows[1:], strict=False)):
    raise ValueError('non-increasing control timestamps')
  if not rows:
    raise ValueError('no complete control frames')
  owned = [r for r in rows if (r['off']['owning'] or r['on']['owning']) and r['valid'] and r['v'] > 0.05]
  errors = [abs(r['off']['wire'] - r['recorded_wire']) for r in owned if r['off']['owning']]
  changed = [r for r in owned if abs(r['on']['wire'] - r['off']['wire']) > 1e-6]
  return {'route': routes.pop(), 'sources': sources, 'init': init, 'skipped_startup_frames': skipped,
          'reset_at_source_gaps': discontinuities,
          'replay_commit': subprocess.check_output(['git', 'rev-parse', 'HEAD'], text=True).strip(),
          'source_hashes': {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in
                            [Path(__file__), Path('selfdrive/controls/lib/longcontrol.py'),
                             Path('selfdrive/controls/lib/stop_context.py'), Path('selfdrive/controls/lib/stopping_service.py'),
                             Path('selfdrive/controls/lib/stopping_flags.py')]},
          'flags': flags, 'recovery_modes': list(recovery_modes), 'summary': {'owned_moving_frames': len(owned), 'changed_frames': len(changed),
            'ownership_differences': sum(r['off']['owning'] != r['on']['owning'] for r in rows),
            'baseline_mae': sum(errors) / len(errors) if errors else None, 'baseline_max_error': max(errors, default=None),
            'max_deepen': max((r['off']['wire'] - r['on']['wire'] for r in owned), default=None),
            'max_release': max((r['on']['wire'] - r['off']['wire'] for r in owned), default=None)},
          'limits': ['Recorded inputs are held fixed; candidate wire does not predict speed, jerk or rest gap.',
                     'Latest logged inputs at carControl publication approximate controlsd subscription timing.',
                     'Initial toggle snapshot and standard Hyundai acceleration limits; no live parameter changes reconstructed.'],
          'rows': rows}


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('--output', type=Path, required=True)
  parser.add_argument('rlogs', type=Path, nargs='+')
  args = parser.parse_args()
  result = replay(args.rlogs)
  if not all(math.isfinite(r['on']['wire']) and math.isfinite(r['off']['wire']) for r in result['rows']):
    raise ValueError('non-finite replay output')
  args.output.parent.mkdir(parents=True, exist_ok=True)
  args.output.write_text(json.dumps(result, allow_nan=False))
  print(json.dumps(result['summary'], indent=2))


if __name__ == '__main__':
  main()
