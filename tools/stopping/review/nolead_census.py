#!/usr/bin/env python3
"""Census of engaged NO-LEAD stops (no radar lead in the last 1.5 s before the wheel stop): when the model's stop flag
fired relative to the wheel stop, when the model trajectory first planned v -> 0 within 2.5 s, the planner's deepest
demand, the wire at the wheel stop and the stopping service's entry time. Stage 1 scans qlogs for candidates, stage 2
reads the rlog of each candidate segment. Usage: nolead_census.py <route counter hex lower bound inclusive> [upper]."""
import glob
import json
import os
import sys

from openpilot.tools.lib.logreader import LogReader

DATA = os.path.expanduser("~/.route_sync/data/media/0/realdata")


def candidates(seg_dir):
  """(local t of wheel stop) for engaged stops from >= 1.5 m/s with no radar lead 1 s before the stop (qlog)."""
  qlog = os.path.join(seg_dir, "qlog.zst")
  if not os.path.exists(qlog):
    return []
  t0 = None
  v_hist = []      # (t, v)
  lead_hist = []   # (t, status, track)
  enabled = False
  out = []
  last_stop = -10.0
  for m in LogReader(qlog):
    t = m.logMonoTime / 1e9
    if t0 is None:
      t0 = t
    rel = t - t0
    w = m.which()
    if w == "selfdriveState":
      enabled = m.selfdriveState.enabled
    elif w == "radarState":
      lead_hist.append((rel, m.radarState.leadOne.status, m.radarState.leadOne.radarTrackId))
    elif w == "carState":
      v = m.carState.vEgo
      v_hist.append((rel, v))
      if enabled and v < 0.1 and rel - last_stop > 5.0 and any(vv > 1.5 for tt, vv in v_hist if rel - 6.0 <= tt <= rel):
        leads = [(s, tid) for tt, s, tid in lead_hist if rel - 1.5 <= tt <= rel - 0.3]
        if leads and not any(s and tid >= 0 for s, tid in leads):
          out.append(rel)
          last_stop = rel
  return out


def measure(seg_dir, t_stop_hint):
  rlog = os.path.join(seg_dir, "rlog.zst")
  if not os.path.exists(rlog):
    return None
  t0 = None
  rows = []   # (rel, v, a_ego, a_tgt, wire, model_ss, plan_ss, msd, v_plan_2p5, force_coast, svc_phase)
  v = a_ego = 0.0
  a_tgt = wire = 0.0
  model_ss = plan_ss = False
  msd = -1.0
  v25 = None
  fc = False
  for m in LogReader(rlog):
    t = m.logMonoTime / 1e9
    if t0 is None:
      t0 = t
    rel = t - t0
    if rel < t_stop_hint - 6.0 or rel > t_stop_hint + 1.0:
      continue
    w = m.which()
    if w == "carState":
      v, a_ego = m.carState.vEgo, m.carState.aEgo
      rows.append((rel, v, a_ego, a_tgt, wire, model_ss, plan_ss, msd, v25, fc))
    elif w == "longitudinalPlan":
      a_tgt, plan_ss, msd = m.longitudinalPlan.aTarget, m.longitudinalPlan.shouldStop, m.longitudinalPlan.distanceToStopTargetModel
    elif w == "carControl":
      wire = m.carControl.actuators.accel
    elif w == "modelV2":
      model_ss = m.modelV2.action.shouldStop
      vs = list(m.modelV2.velocity.x)
      v25 = vs[16] if len(vs) > 16 else None   # T_IDXS[16] ~ 2.5 s
    elif w == "frogpilotCarState":
      fc = m.frogpilotCarState.forceCoast
  if not rows:
    return None
  # wheel stop = first frame v < 0.1 after the hint window start
  stop = next((r for r in rows if r[1] < 0.1 and r[0] >= t_stop_hint - 1.0), None)
  if stop is None:
    return None
  ts = stop[0]
  appr = [r for r in rows if ts - 5.0 <= r[0] <= ts]
  v_appr = max(r[1] for r in appr)
  t_model_ss = next((r[0] for r in appr if r[5]), None)
  t_plan_ss = next((r[0] for r in appr if r[6]), None)
  t_traj = next((r[0] for r in appr if r[8] is not None and r[8] < 0.5 and r[1] > 0.5), None)
  t_deep = next((r[0] for r in appr if r[3] < -0.5), None)
  a_min = min(r[3] for r in appr)
  wire_min = min(r[4] for r in appr)
  v_at_model_ss = next((r[1] for r in appr if r[5]), None)
  last = [r for r in appr if ts - 0.35 <= r[0] <= ts]
  a_wstop = min(r[2] for r in last) if last else None
  jerk = max(abs(last[i + 1][2] - last[i][2]) / max(last[i + 1][0] - last[i][0], 1e-3) for i in range(len(last) - 1)) if len(last) > 1 else None
  fc_frac = sum(1 for r in appr if r[9]) / max(len(appr), 1)
  return {"t_stop": round(ts, 1), "v_appr": round(v_appr, 2), "a_tgt_min": round(a_min, 2), "wire_min": round(wire_min, 2),
          "model_ss_lead_s": None if t_model_ss is None else round(ts - t_model_ss, 2),
          "v_at_model_ss": None if v_at_model_ss is None else round(v_at_model_ss, 2),
          "plan_ss_lead_s": None if t_plan_ss is None else round(ts - t_plan_ss, 2),
          "traj_stop_lead_s": None if t_traj is None else round(ts - t_traj, 2),
          "deep_lead_s": None if t_deep is None else round(ts - t_deep, 2),
          "a_wheelstop": None if a_wstop is None else round(a_wstop, 2), "jerk_wstop": None if jerk is None else round(jerk, 1),
          "fc_frac": round(fc_frac, 2)}


def main():
  lo = sys.argv[1]
  hi = sys.argv[2] if len(sys.argv) > 2 else "ffffffff"
  segs = sorted(d for d in glob.glob(os.path.join(DATA, "0000*--*--*")) if lo <= os.path.basename(d)[:8] <= hi)
  for seg_dir in segs:
    for t_hint in candidates(seg_dir):
      r = measure(seg_dir, t_hint)
      if r:
        print(json.dumps({"seg": os.path.basename(seg_dir), **r}), flush=True)


if __name__ == "__main__":
  main()
