#!/usr/bin/env python3
"""Replay the whole-approach SHADOW on recorded planner inputs; never a plant or LIVE acceptance test.

Run from the repository root in the managed venv. The output records each indexed stop, excluded
commitments, capture checks, identity coverage and raw (not index-matched) attributed-safety totals.
The replay explicitly enables shadow in this process only. It never writes vehicle commands.
"""
import argparse
import collections
import json
import math
import pathlib
import sys


from openpilot.selfdrive.controls.lib import stopping_flags
from openpilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlanner, reset_whole_approach_certificate
from openpilot.selfdrive.controls.lib.lead_provenance import StoppingLeadAuthority
from openpilot.selfdrive.controls.lib.stop_context import StopContext
from openpilot.selfdrive.controls.lib.stopping_governor import gap_ref, capture_reserve
from openpilot.common.swaglog import cloudlog
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from triage_one import read_events

def main():
  DATA = pathlib.Path.home() / '.route_sync/data/media/0/realdata'
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('--routes', help='Comma-separated route counters; defaults to routes in the stop index')
  parser.add_argument('--output', required=True, type=pathlib.Path)
  args = parser.parse_args()
  rows = [json.loads(l) for l in open(pathlib.Path.home() / '.route_sync/corpus/stop_index.jsonl')]
  routes = args.routes.split(',') if args.routes else sorted({r['seg'].split('--')[0] for r in rows})
  stopping_flags.WHOLE_APPROACH_GOVERNOR = 'shadow'
  cloudlog.event = lambda *args, **kw: None
  out = {'routes': [], 'stops': [], 'limitations': ['Recorded geometry is not a counterfactual vehicle rollout.',
         'Published aTargetTrajectory is clipped: a proxy for the unpublished raw MPC input.',
         'Stop windows use the existing candidate index; they are not an exhaustive held-out stop census.'],
         'frames': collections.Counter(), 'attr': collections.Counter(), 'attr_reasons': collections.Counter()}
  for route in routes:
    planner = LongitudinalPlanner.__new__(LongitudinalPlanner)
    reset_whole_approach_certificate(planner)
    planner.dt, planner.wa_stats = .05, None
    planner._wa_lead_auth, planner._wa_stop_ctx = StoppingLeadAuthority(), StopContext()
    latest, stamps, frames = {}, {}, []
    origin = None
    previous_t = last_radar_t = None
    for path in sorted(DATA.glob(route + '--*/rlog.zst'), key=lambda p: int(p.parent.name.split('--')[-1])):
      for ev in read_events(str(path)):
        w, t = ev.which(), ev.logMonoTime * 1e-9
        if origin is None:
          origin = t
        if w == 'logMessage' and 'settle_summary' in ev.logMessage:
          payload = json.loads(ev.logMessage)
          payload = payload.get('msg', payload)
          for k in ('attr_frames', 'attr_ineligible', 'attr_plan_bound', 'attr_unexplained', 'attr_released_sum', 'attr_pred_bound'):
            out['attr'][k] += payload.get(k, 0) or 0
          out['attr_reasons'].update(payload.get('attr_reasons') or {})
          if payload.get('attr_frames'):
            out['attr']['settles'] += 1
        if w in ('carState', 'selfdriveState', 'controlsState', 'frogpilotCarState', 'frogpilotPlan', 'radarState'):
          latest[w], stamps[w] = getattr(ev, w), t
        if w != 'longitudinalPlan':
          continue
        out['frames']['all_plan'] += 1
        required = ('carState', 'selfdriveState', 'controlsState', 'frogpilotCarState', 'frogpilotPlan', 'radarState')
        if not all(k in latest for k in required) or any(t - stamps[k] > .5 for k in required):
          out['frames']['missing_or_stale'] += 1
          continue
        if previous_t is not None and not 0 < t - previous_t < .15:
          reset_whole_approach_certificate(planner)
          planner._wa_stop_ctx.reset()
          planner._wa_lead_auth.reset()
          planner.wa_stats = None
          out['frames']['cadence_reset'] += 1
        previous_t = t
        cs, ss, ctrl, fc, fp, rs = [latest[k] for k in required]
        lp = ev.longitudinalPlan
        engaged = bool(ss.enabled and str(ctrl.longControlState) != 'off' and cs.vCruise != 255
                        and not cs.gasPressed and not cs.brakePressed)
        isd = float(fp.increasedStoppedDistance)
        kw = dict(santa_fe=True, blended=ss.experimentalMode, engaged=engaged,
                  force_coast=fc.forceCoast, v_ego=cs.vEgo, a_ego=cs.aEgo, actual_a_target=lp.aTarget,
                  a_target_mpc=lp.aTargetTrajectory, lead=rs.leadOne, lead_two=rs.leadTwo,
                  fcw=lp.fcw, isd=isd, standstill=cs.standstill,
                  radar_fresh=stamps['radarState'] != last_radar_t and 0 <= t-stamps['radarState'] <= .1)
        last_radar_t = stamps['radarState']
        prev_commit = planner.wa_committed
        planner._update_whole_approach_shadow(**kw)
        lead = rs.leadOne
        out['frames']['replayed'] += 1
        if lead.status:
          out['frames']['lead_status'] += 1
          out['frames']['identityless'] += lead.radarTrackId < 0
        excluded = (not ss.enabled or not engaged or cs.brakePressed or cs.gasPressed or fc.forceCoast or
                    not ss.experimentalMode or not lead.status or lead.vLead < -.25 or cs.standstill)
        if planner.wa_committed:
          out['frames']['committed'] += 1
          out['frames']['excluded_committed'] += excluded
          for name, condition in [('standstill', cs.standstill), ('gas', cs.gasPressed), ('brake', cs.brakePressed),
                                  ('disengaged', not ss.enabled or not engaged), ('force_coast', fc.forceCoast),
                                  ('no_lead', not lead.status), ('reversing', lead.vLead < -.25)]:
            out['frames'][name + '_committed'] += condition
        if not prev_commit and planner.wa_committed:
          out['frames']['entries'] += 1
        if prev_commit and not planner.wa_committed:
          out['frames']['releases'] += 1
        dcurve = gap_ref(max(cs.vEgo, 0), isd)
        capture = capture_reserve(max(cs.vEgo, 0))
        frame = dict(t=t-origin, v=cs.vEgo, lv=lead.vLead, gap=lead.dRel, lead=lead.status,
                     track=lead.radarTrackId, isd=isd, enabled=ss.enabled, gas=cs.gasPressed, brake=cs.brakePressed,
                     committed=planner.wa_committed, entry=not prev_commit and planner.wa_committed,
                     reason=planner.whole_approach_reason, candidate=planner.whole_approach_demand,
                     actual=lp.aTarget, mpc=lp.aTargetTrajectory, safety=planner.whole_approach_safety_min,
                     boundary_margin=lead.dRel-dcurve-capture-.5*cs.vEgo if dcurve is not None and capture is not None else None)
        frames.append(frame)
    for row in [r for r in rows if r['seg'].split('--')[0] == route]:
      win = [f for f in frames if row['t']-30 <= f['t'] <= row['t']]
      # Do not mix an earlier stop/departure into this approach.
      for i in range(len(win)-1, -1, -1):
        if win[i]['v'] < .05 and win[i]['t'] < row['t']-.5:
          win = win[i+1:]
          break
      finite = [f for f in win if math.isfinite(f['candidate'])]
      entries = [f for f in win if f['entry']]
      early = [f['actual']-f['candidate'] for f in finite if f['v'] > 4]
      late = [f['candidate']-f['actual'] for f in finite if .8 <= f['v'] <= 3]
      high = [f for f in win if f['v'] >= 6 and f['lead'] and abs(f['lv'])<=.3]
      crossing = next((f for f in high if f['boundary_margin'] <= 0), None)
      rec = dict(seg=row['seg'], t=row['t'], lead_v=row.get('lead_v'), v_appr=row.get('v_appr'),
                 replay_max_v=max([f['v'] for f in win], default=0), frames=len(win), entries=len(entries),
                 stopped_lead=bool(row.get('lead_v') is not None and abs(row['lead_v'])<=.3),
                 moving_to_stop=any(f['lv']>1 for f in win if f['lead'] and win and f['track']==win[-1]['track']),
                 driver_free=bool(win) and all(f['enabled'] and not f['gas'] and not f['brake'] for f in win),
                 early_deeper=max(early, default=None), late_shallower=max(late, default=None),
                 capture_certified=crossing['committed'] if crossing else None,
                 first_entry=entries[0] if entries else None,
                 below_2_5_samples=sum(f['v']<2.5 for f in finite))
      out['stops'].append(rec)
    out['routes'].append(dict(route=route, frames=len(frames)))
    print(route, len(frames), 'frames', flush=True)
  selected=[r for r in out['stops'] if r['stopped_lead'] and r['replay_max_v']>=3]
  out['summary'] = dict(stopped_lead_stops=len(selected), starts_ge_8=sum(r['replay_max_v']>=8 for r in selected),
                        moving_to_stop=sum(r['moving_to_stop'] for r in selected),
                        driver_free=sum(r['driver_free'] for r in selected),
                        high_capture_n=sum(r['capture_certified'] is not None for r in selected),
                        high_capture_ok=sum(r['capture_certified'] is True for r in selected),
                        demand_earlier=sum((r['early_deeper'] or 0)>=.15 and (r['late_shallower'] or 0)>=.15 for r in selected),
                        multiple_entries=sum(r['entries']>1 for r in selected))
  args.output.write_text(json.dumps(out, indent=2))
  print(json.dumps({k:v for k,v in out.items() if k not in ('stops','routes')}, indent=2))


if __name__ == "__main__":
  main()
