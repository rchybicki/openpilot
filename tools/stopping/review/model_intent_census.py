#!/usr/bin/env python3
"""Gate-2 census for the trajectory-based model stop intent: over every engaged, in-band (v <= 2.5 m/s), no-radar-lead
frame of the given routes, count intent EPISODES (planned v < 0.5 m/s within the horizon, from the rlog modelV2 plan)
and whether a wheel stop followed within 6 s -- versus the model's own action.shouldStop bit.
Usage: model_intent_census.py <route lo hex> [route hi hex] [horizon_s=4.0]"""
import glob
import json
import os
import sys

from openpilot.tools.lib.logreader import LogReader

DATA = os.path.expanduser("~/.route_sync/data/media/0/realdata")
T_IDXS = [((i / 32) ** 2) * 10.0 for i in range(33)]
MODEL_STOP_V = 0.5
FOLLOW_S = 6.0


def trajectory_intent(vel_x, horizon_s):
  for i, t in enumerate(T_IDXS):
    if t > horizon_s:
      break
    if i < len(vel_x) and vel_x[i] < MODEL_STOP_V:
      return True
  return False


def scan(seg_dir, horizon_s):
  rlog = os.path.join(seg_dir, "rlog.zst")
  if not os.path.exists(rlog):
    return []
  t0 = None
  v = 0.0
  enabled = False
  lead_radar = False
  frames = []   # (t, v, in_scope, intent_traj, intent_bit, a_target, force_coast)
  intent_traj = intent_bit = False
  a_target = 0.0
  force_coast = False
  for m in LogReader(rlog):
    t = m.logMonoTime / 1e9
    if t0 is None:
      t0 = t
    rel = t - t0
    w = m.which()
    if w == "selfdriveState":
      enabled = m.selfdriveState.enabled
    elif w == "radarState":
      lo = m.radarState.leadOne
      lead_radar = bool(lo.status and lo.radarTrackId >= 0)
    elif w == "modelV2":
      intent_traj = trajectory_intent(list(m.modelV2.velocity.x), horizon_s)
      intent_bit = bool(m.modelV2.action.shouldStop)
    elif w == "longitudinalPlan":
      a_target = m.longitudinalPlan.aTarget
    elif w == "frogpilotCarState":
      force_coast = bool(m.frogpilotCarState.forceCoast)
    elif w == "carState":
      v = m.carState.vEgo
      frames.append((rel, v, enabled and v <= 2.5 and not lead_radar, intent_traj, intent_bit, a_target, force_coast))
  episodes = []
  for key, idx in (("traj", 3), ("bit", 4)):
    active = False
    start = v_start = 0.0
    for f in frames:
      on = f[2] and f[idx]
      if on and not active:
        if f[1] < 0.5:
          continue                       # an intent while already (nearly) stopped is not an entry
        active = True
        start, v_start = f[0], f[1]
      elif active and not on:
        active = False
        if episodes and episodes[-1]["kind"] == key and start - episodes[-1]["t_end"] < 0.5:
          episodes[-1]["t_end"] = round(f[0], 2)   # flicker: merge into the previous episode
          episodes[-1]["dur_s"] = round(episodes[-1]["t_end"] - episodes[-1]["t"], 2)
          continue
        stop_followed = any(ff[1] < 0.1 for ff in frames if start <= ff[0] <= start + FOLLOW_S)
        stop_within_10 = any(ff[1] < 0.1 for ff in frames if start <= ff[0] <= start + 10.0)
        window = [ff for ff in frames if start <= ff[0] <= f[0]]
        a_min = min(ff[5] for ff in window) if window else None
        fc_any = any(ff[6] for ff in window)
        episodes.append({"seg": os.path.basename(seg_dir), "kind": key, "t": round(start, 1), "t_end": round(f[0], 2),
                         "v": round(v_start, 2), "dur_s": round(f[0] - start, 2), "stop_within_6s": stop_followed,
                         "stop_within_10s": stop_within_10, "a_target_min": None if a_min is None else round(a_min, 2),
                         "force_coast": fc_any})
  return episodes


def main():
  lo = sys.argv[1]
  hi = sys.argv[2] if len(sys.argv) > 2 else "ffffffff"
  horizon = float(sys.argv[3]) if len(sys.argv) > 3 else 4.0
  segs = sorted(d for d in glob.glob(os.path.join(DATA, "0000*--*--*")) if lo <= os.path.basename(d)[:8] <= hi)
  for seg_dir in segs:
    for ep in scan(seg_dir, horizon):
      print(json.dumps(ep), flush=True)


if __name__ == "__main__":
  main()
