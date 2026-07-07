#!/usr/bin/env python3
"""Deep stop-quality analysis of ONE rlog segment. Emits JSON.
Per settle: rest gap, dts, terminal IMU jerk (honest 20Hz), settle peak decel,
wheel-stop wire (cmd at last rolling frame), post-stop rebound, hold cmd, taxonomy."""
import json
import sys

import capnp
import zstandard

REPO = "/Users/radoslawchybicki/Repos/openpilot-rch"
capnp.remove_import_hook()
LOG = capnp.load(f"{REPO}/cereal/log.capnp")


def main(path):
  vs, cmds, imu, leads, plans, states = [], [], [], [], [], []
  gyro = []
  enabled = False
  t0 = None
  raw = zstandard.ZstdDecompressor().decompress(open(path, "rb").read(), max_output_size=int(9e8))
  for ev in LOG.Event.read_multiple_bytes(raw):
    w = ev.which()
    t = ev.logMonoTime * 1e-9
    if t0 is None:
      t0 = t
    t -= t0
    if w == "carState":
      vs.append((t, ev.carState.vEgo, ev.carState.aEgo))
    elif w == "carOutput":
      cmds.append((t, ev.carOutput.actuatorsOutput.accel))
    elif w == "livePose":
      a = ev.livePose.accelerationDevice
      if a.valid:
        imu.append((t, a.x))
      g = ev.livePose.angularVelocityDevice
      if g.valid:
        gyro.append((t, g.y))  # pitch rate: the felt stop 'nod' (body rotation on brake grab/release)
    elif w == "radarState":
      l1 = ev.radarState.leadOne
      leads.append((t, bool(l1.status), l1.dRel, l1.vLead))
    elif w == "longitudinalPlan":
      lp = ev.longitudinalPlan
      plans.append((t, lp.aTarget, getattr(lp, "distanceToStopTarget", -1.0)))
    elif w == "selfdriveState":
      enabled = ev.selfdriveState.enabled
    elif w == "controlsState":
      states.append((t, str(ev.controlsState.longControlState), enabled))

  def at(arr, t, idx=1):
    if not arr:
      return None
    lo, hi = 0, len(arr) - 1
    while lo < hi:
      mid = (lo + hi) // 2
      if arr[mid][0] < t:
        lo = mid + 1
      else:
        hi = mid
    best = lo
    if lo > 0 and abs(arr[lo - 1][0] - t) < abs(arr[lo][0] - t):
      best = lo - 1
    return arr[best][idx]

  V_STOP = 0.05
  settles = []
  in_stop = None
  stop_start = None
  for (t, v, _a) in vs:
    en = at(states, t, 2)
    if v < V_STOP and en:
      if stop_start is None:
        stop_start = t
      if in_stop is None and t - stop_start >= 0.5:
        in_stop = {"t_settle": stop_start}
    else:
      if in_stop is not None:
        in_stop["t_release"] = t
        settles.append(in_stop)
        in_stop = None
      stop_start = None
  if in_stop is not None:
    in_stop["t_release"] = None
    settles.append(in_stop)

  events = []
  for i, s in enumerate(settles):
    ts, tr = s["t_settle"], s["t_release"]
    e = {"t_settle": round(ts, 1), "held_s": round(tr - ts, 1) if tr else "to_end"}
    if at(leads, ts + 0.3, 1):
      e["rest_gap"] = round(at(leads, ts + 0.3, 2), 2)
      e["lead_v"] = round(at(leads, ts + 0.3, 3), 2)
    d = at(plans, ts + 0.3, 2)
    e["dts"] = round(d, 2) if d is not None else None
    a_win = [(t, v) for (t, v, _) in vs if ts - 10.0 <= t < ts and v >= V_STOP]
    if a_win:
      e["v_appr"] = round(max(v for _, v in a_win), 2)
      c_win = [c for (t, c) in cmds if ts - 10.0 <= t < ts]
      if c_win:
        e["cmd_min"] = round(min(c_win), 2)
    roll = [t for (t, v, _) in vs if t < ts and v >= V_STOP]
    if roll:
      e["wire_at_stop"] = round(at(cmds, roll[-1]), 2)
    iw = [(t, x) for (t, x) in imu if ts - 2.0 <= t <= ts + 1.5]
    if len(iw) >= 3:
      jerks = [abs(iw[k][1] - iw[k - 1][1]) / (iw[k][0] - iw[k - 1][0])
               for k in range(1, len(iw)) if 0.01 < iw[k][0] - iw[k - 1][0] < 0.2]
      if jerks:
        e["jerk"] = round(max(jerks), 1)
      e["pdec"] = round(-min(x for _, x in iw), 2)
      e["rebound"] = round(max(x for (t, x) in iw if t >= ts), 2) if any(t >= ts for t, _ in iw) else None
    gw = [(t, x) for (t, x) in gyro if ts - 2.0 <= t <= ts + 1.5]
    if len(gw) >= 3:
      e["pitch_rate_peak"] = round(max(abs(x) for _, x in gw), 3)
    ch = [c for (t, c) in cmds if ts + 0.5 <= t <= ts + 2.0]
    if ch:
      e["hold"] = round(sum(ch) / len(ch), 2)
    ch2 = [c for (t, c) in cmds if ts + 3.0 <= t <= ts + 5.0]
    if ch2:
      e["hold_5s"] = round(sum(ch2) / len(ch2), 2)
    if tr is not None:
      nxt = settles[i + 1]["t_settle"] if i + 1 < len(settles) else None
      end = nxt if nxt is not None else (vs[-1][0] if vs else tr)
      seg_v = [(t, v) for (t, v, _) in vs if tr <= t <= end]
      if seg_v:
        dist = sum(seg_v[k][1] * (seg_v[k][0] - seg_v[k - 1][0])
                   for k in range(1, len(seg_v)) if seg_v[k][0] - seg_v[k - 1][0] < 0.2)
        lm = at(leads, tr + 0.5, 3)
        lt = at(leads, tr + 0.5, 1)
        if nxt is not None and lt and (lm or 0) < 0.3 and dist < 3.0:
          e["taxonomy"] = "DISLIKE1_stop_go_stop" if dist > 0.25 else "DISLIKE2_nudge"
        else:
          e["taxonomy"] = "normal_resume"
    events.append(e)
  print(json.dumps({"path": path.split("/realdata/")[-1], "events": events}))


if __name__ == "__main__":
  main(sys.argv[1])
