#!/usr/bin/env python3
"""Census of the frames the force-coast cap deletion would touch: engaged, blended, force coast ON, no lead of any kind
(radarState leadOne/leadTwo status False), model stop intent (planned v < 0.5 m/s within 0.5-4 s). Per episode: duration,
speed range, the planner's excess below min(e2e, MPC), and whether a stop followed within 10 s.
Usage: fc_cap_census.py <route lo hex> [route hi hex]"""
import glob
import json
import os
import sys

from openpilot.tools.lib.logreader import LogReader

DATA = os.path.expanduser("~/.route_sync/data/media/0/realdata")
T_IDXS = [((i / 32) ** 2) * 10.0 for i in range(33)]


def intent(vel_x):
  for i, t in enumerate(T_IDXS):
    if t < 0.5:
      continue
    if t > 4.0:
      break
    if i < len(vel_x) and vel_x[i] < 0.5:
      return True
  return False


def scan(seg_dir):
  rlog = os.path.join(seg_dir, "rlog.zst")
  if not os.path.exists(rlog):
    return []
  t0 = None
  v = 0.0
  enabled = exp = False
  fc = False
  any_lead = False
  it = False
  e2e = 0.0
  frames = []
  for m in LogReader(rlog):
    t = m.logMonoTime / 1e9
    if t0 is None:
      t0 = t
    rel = t - t0
    w = m.which()
    if w == "selfdriveState":
      enabled, exp = m.selfdriveState.enabled, m.selfdriveState.experimentalMode
    elif w == "frogpilotCarState":
      fc = bool(m.frogpilotCarState.forceCoast)
    elif w == "radarState":
      any_lead = bool(m.radarState.leadOne.status or m.radarState.leadTwo.status)
    elif w == "modelV2":
      it = intent(list(m.modelV2.velocity.x))
      e2e = m.modelV2.action.desiredAcceleration
    elif w == "longitudinalPlan":
      lp = m.longitudinalPlan
      both = min(e2e, lp.aTargetTrajectory) if lp.aTargetTrajectoryValid else e2e
      frames.append((rel, v, enabled and exp and fc and not any_lead and it, both - lp.aTarget))
    elif w == "carState":
      v = m.carState.vEgo
  eps = []
  cur = None
  for f in frames:
    if f[2] and cur is None:
      cur = [f[0], f[0], f[1], f[1], f[3], 1]
    elif f[2]:
      cur[1] = f[0]
      cur[2] = max(cur[2], f[1])
      cur[3] = min(cur[3], f[1])
      cur[4] = max(cur[4], f[3])
      cur[5] += 1
    elif cur is not None:
      stop = any(ff[1] < 0.1 for ff in frames if cur[0] <= ff[0] <= cur[0] + 10.0)
      eps.append({"seg": os.path.basename(seg_dir), "t": round(cur[0], 1), "dur_s": round(cur[1] - cur[0], 2), "v_max": round(cur[2], 2),
                  "v_min": round(cur[3], 2), "excess_max": round(cur[4], 2), "frames": cur[5], "stop_within_10s": stop})
      cur = None
  return eps


def main():
  lo = sys.argv[1]
  hi = sys.argv[2] if len(sys.argv) > 2 else "ffffffff"
  for seg_dir in sorted(d for d in glob.glob(os.path.join(DATA, "0000*--*--*")) if lo <= os.path.basename(d)[:8] <= hi):
    for ep in scan(seg_dir):
      print(json.dumps(ep), flush=True)


if __name__ == "__main__":
  main()
