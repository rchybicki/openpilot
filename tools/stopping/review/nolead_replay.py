#!/usr/bin/env python3
"""Recorded-input replay of the stopping service over one no-lead stop (gate 2b of the cycle-50 stage-A design).

Feeds the rlog's 100 Hz frames (carState, held longitudinalPlan / radarState / modelV2 / carControl) through a fresh
StopContext + StoppingService with (a) the deployed stop flag (NOLEAD_STOP_INTENT off) or (b) the cycle-50 implementation:
the TRAJECTORY stop intent (planned v < 0.5 m/s within 0.5-HORIZON_S) through longcontrol's entry derivation, any_lead
and the model stop into the service, whose no-lead branch releases the planner's demand release-only. Prints per 0.1 s:
recorded wire, the service's phase and a_phase, and the stage-A wire (= the service output on owned frames). Open loop: the car's recorded speed does
not respond, so the drift is bounded
analytically from the takeover speed: v^2/(2*|a_stageA|) - v^2/(2*|a_recorded|).
Usage: nolead_replay.py <segment-dir-name> <t_lo> <t_hi> [horizon_s=4.0]   (segment-local seconds)
The context is fed a_cmd = a_ego (neutral coast residual): open loop, the recorded plant brakes harder than the recorded
command and the residual would otherwise shallow a_phase to its clip."""
import sys
from pathlib import Path

from openpilot.tools.lib.logreader import LogReader
from openpilot.selfdrive.controls.lib.stop_context import A_CMD_DELAY_S, StopContext
from openpilot.selfdrive.controls.lib import stopping_flags
from openpilot.selfdrive.controls.lib.stopping_service import StoppingService

DATA = Path.home() / ".route_sync/data/media/0/realdata"
T_IDXS = [((i / 32) ** 2) * 10.0 for i in range(33)]
MODEL_STOP_V = 0.5
PLANNER_MIN = -3.5


def trajectory_stop(vel_x, pos_x, horizon_s):
  """(intent, distance) from the e2e plan: the first point under MODEL_STOP_V inside the horizon."""
  for i, t in enumerate(T_IDXS):
    if t > horizon_s:
      break
    if i < len(vel_x) and vel_x[i] < MODEL_STOP_V:
      return True, max(float(pos_x[i]), 0.0)
  return False, -1.0


def replay(seg, t_lo, t_hi, horizon_s, use_intent, neutral_coast=True):
  seg_idx = int(seg.split("--")[-1])
  stopping_flags.ATTRIBUTED_SAFETY = "live"
  stopping_flags.NOLEAD_ATTRIBUTED_SAFETY = "live" if use_intent else "off"
  ctx, svc = StopContext(), StoppingService()
  t0 = None
  plan = None
  lead = (False, 0.0, 0.0, -1, 0.0)
  lead_any = False
  lead2 = (False, 0.0, 0.0)
  intent_s = 0.0
  intent_hold = 0.0
  wire = 0.0
  intent, msd_traj = False, -1.0
  fc = False
  isd = 0.0
  rows = []
  msgs = list(LogReader(str(DATA / seg / "rlog.zst")))
  a_ego_series = [(m.logMonoTime / 1e9, m.carState.aEgo) for m in msgs if m.which() == "carState"]
  delay_n = max(int(round(A_CMD_DELAY_S / 0.01)), 1)
  cs_index = 0
  for m in msgs:
    t = m.logMonoTime / 1e9
    if t0 is None:
      t0 = t
    rel = t - t0 - 60.0 * seg_idx
    w = m.which()
    if w == "longitudinalPlan":
      lp = m.longitudinalPlan
      plan = (lp.aTarget, lp.shouldStop, lp.distanceToStopTarget, lp.distanceToStopTargetModel, bool(lp.fcw),
              lp.aTargetTrajectory if lp.aTargetTrajectoryValid else None)
    elif w == "radarState":
      lo, l2 = m.radarState.leadOne, m.radarState.leadTwo
      lead = (bool(lo.status and lo.radarTrackId >= 0), lo.vLead, lo.dRel, lo.radarTrackId, lo.aLeadK)
      lead_any = bool(lo.status)
      lead2 = (bool(l2.status), l2.vLead, l2.dRel)
    elif w == "modelV2":
      intent, msd_traj = trajectory_stop(list(m.modelV2.velocity.x), list(m.modelV2.position.x), horizon_s)
    elif w == "carControl":
      wire = m.carControl.actuators.accel
    elif w == "frogpilotCarState":
      fc = m.frogpilotCarState.forceCoast
    elif w == "frogpilotPlan":
      isd = float(getattr(m.frogpilotPlan, "increasedStoppedDistance", 0.0))
    elif w == "carState":
      cs = m.carState
      dt = 0.01   # the car feeds the context and the service DT_CTRL regardless of log jitter
      # the context delays a_cmd by A_CMD_DELAY_S before differencing it with a_ego: feed the FUTURE a_ego so the
      # delayed command equals the measured response (residual ~0), as the service's own closed loop would see
      future_a = a_ego_series[min(cs_index + delay_n, len(a_ego_series) - 1)][1]
      cs_index += 1
      if plan is None or rel < t_lo:
        continue
      if rel > t_hi:
        break
      a_target, ss_plan, dts_plan, msd_plan, fcw, a_traj = plan
      any_lead = bool(lead_any or lead2[0])
      if use_intent:
        msd = msd_traj if intent else msd_plan
        # longcontrol's cycle-50 entry derivation: persisted intent, no lead of any kind, in band, planner braking
        intent_now = msd >= 0.0 and not any_lead and cs.vEgo < 2.5 and a_target <= -0.30
        if intent_now:
          intent_s = min(intent_s + dt, 0.20)
          if intent_s >= 0.20 - 1e-9:
            intent_hold = 0.40
        else:
          intent_s = max(intent_s - 2.0 * dt, 0.0)
          intent_hold = max(intent_hold - dt, 0.0)
        nolead_intent = intent_hold > 0.0 and not any_lead
        should_stop = ss_plan or nolead_intent
        nolead_only = nolead_intent and not ss_plan
      else:
        msd = msd_plan
        should_stop = ss_plan
        nolead_only = False
      lead_status = lead[0]
      sig = ctx.update(v_ego=cs.vEgo, a_ego=cs.aEgo, a_cmd=future_a if neutral_coast else wire, lead_status=lead_status, lead_v=lead[1],
                       lead_d_rel=lead[2] if lead_status else None, lead_track_id=lead[3] if lead_status else None,
                       standstill=cs.standstill, dt=dt)
      r = svc.update(engaged=True, v_ego=cs.vEgo, a_ego=cs.aEgo, a_target=a_target, should_stop=should_stop,
                     dts_planner=dts_plan if dts_plan >= 0 else None, planner_min_limit=PLANNER_MIN, signals=sig,
                     lead_status=lead_status, lead_v=lead[1], increased_stopped_distance=isd, dt=dt, wire_accel=wire,
                     a_target_trajectory=a_traj, lead_a=lead[4], lead2=lead2, fcw=fcw, model_stop_d=msd, any_lead=any_lead,
                     nolead_intent=nolead_only, force_coast_accel=(-0.7 if fc else None))
      a_phase = r.debug.get("a_phase") if r.active else None
      # the stage-A wire IS the service's output on owned frames (its no-lead branch releases the planner's demand
      # release-only inside); off-band / inactive frames keep the recorded wire
      stage = r.accel if r.active else wire
      rows.append((rel, cs.vEgo, a_target, wire, r.phase.name if r.active else "-", a_phase, stage, should_stop, msd, fc, sig.a_coast, cs.aEgo,
                   r.debug.get("d_rem"), dts_plan, isd, lead2))
  return rows


def main():
  seg, t_lo, t_hi = sys.argv[1], float(sys.argv[2]), float(sys.argv[3])
  horizon_s = float(sys.argv[4]) if len(sys.argv) > 4 else 4.0
  base = replay(seg, t_lo, t_hi, horizon_s, use_intent=False)
  stage = replay(seg, t_lo, t_hi, horizon_s, use_intent=True)
  entry_base = next((r[0] for r in base if r[4] != "-"), None)
  entry_stage = next((r[0] for r in stage if r[4] != "-"), None)
  print(f"{seg} {t_lo}-{t_hi}: service entry deployed at {entry_base}, with the trajectory intent at {entry_stage}")
  print(f"{'t':7} {'v':5} {'aTgt':6} {'wire':6} {'aEgo':6} {'coast':6} | {'phase(dep)':12} | {'phase(A)':12} {'a_phase':8} {'wireA':6} ss msd fc")
  last = -1.0
  for rb, ra in zip(base, stage, strict=False):
    if ra[0] - last < 0.1:
      continue
    last = ra[0]
    ap = "-" if ra[5] is None else f"{ra[5]:.2f}"
    left = f"{ra[0]:7.2f} {ra[1]:5.2f} {ra[2]:6.2f} {ra[3]:6.2f} {ra[11]:6.2f} {ra[10]:6.2f} | {rb[4]:12} | "
    print(left + f"{ra[4]:12} {ap:8} {ra[6]:6.2f} {int(ra[7])} {ra[8]:5.1f} {int(ra[9])}  d_rem={ra[12]} dts={ra[13]:.1f} isd={ra[14]:.2f} lead2={ra[15]}")
  owned = [r for r in stage if r[4] != "-"]
  under = min((r[6] - r[3]) for r in owned) if owned else 0.0
  # NEW braking = the stage-A wire going below BOTH the recorded wire and its own previous value (a hand-back that lags
  # a recorded step-up at the jerk limit is not new braking)
  new_brake = min((owned[k][6] - min(owned[k - 1][6], owned[k][3]) for k in range(1, len(owned))), default=0.0)
  print(f"stage-A wire vs recorded on owned frames: lag-inclusive excursion {under:.3f}, NEW braking below baseline {new_brake:.3f} m/s^2 (=> ~0)")
  # drift bound at the first frame where the stage-A wire departs from the recorded wire
  dep = next((r for r in stage if r[6] > r[3] + 0.02), None)
  if dep:
    v, a_rec, a_st = dep[1], dep[3], dep[6]
    a_rec_end = min(r[3] for r in stage if r[0] >= dep[0])
    a_st_end = min(max(r[5], r[6]) if r[5] is not None else r[6] for r in stage if r[0] >= dep[0])
    d_rec = v * v / (2.0 * max(-a_rec_end, 0.05))
    d_st = v * v / (2.0 * max(-a_st_end, 0.05))
    print(f"release starts at t={dep[0]:.2f} v={v:.2f}: recorded wire {a_rec:.2f} -> stage-A {a_st:.2f}")
    print(f"stop-distance bound from here: recorded {d_rec:.1f} m (deepest {a_rec_end:.2f}) vs stage-A {d_st:.1f} m "
          + f"(deepest {a_st_end:.2f}); drift <= {d_st - d_rec:.1f} m")
  else:
    print("stage-A wire never departs from the recorded wire in this window")


if __name__ == "__main__":
  main()
