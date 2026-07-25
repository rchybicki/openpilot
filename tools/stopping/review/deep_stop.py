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

  # ARRIVAL/ESCAPE SCAN (cycle-13): the settle detector above needs 0.5 s below V_STOP, so a stop
  # whose wheel never reads < 0.05 before it creeps forward (route 00001f44 seg3: v bottomed at
  # 0.082, crept 0.28 m, arrested at -1.00) produces NO settle at its arrival -- the class this
  # battery exists to catch was being labelled normal_resume. This scan is independent of the
  # settle machinery (historical settle numbers stay comparable): inside each engaged low-speed
  # episode it finds the natural arrival (velocity minimum) and measures any forward excursion
  # after it. Reports the ARRIVAL wire, not a wire sampled inside the reactive arrest.
  escapes = []
  V_EPISODE, MIN_EPISODE_S, ESCAPE_RISE = 0.30, 1.0, 0.02
  k = 0
  while k < len(vs):
    if vs[k][1] >= V_EPISODE or not at(states, vs[k][0], 2):
      k += 1
      continue
    j = k
    while j + 1 < len(vs) and vs[j + 1][1] < V_EPISODE and at(states, vs[j + 1][0], 2):
      j += 1
    if vs[j][0] - vs[k][0] >= MIN_EPISODE_S:
      # walk forward from a RUNNING minimum: the arrival is the running min at the moment the car
      # first rises off it (the global episode min is the parked floor, which is not the arrival)
      n_min, n = k, k + 1
      while n <= j:
        if vs[n][1] < vs[n_min][1]:
          n_min = n
        elif vs[n][1] > vs[n_min][1] + ESCAPE_RISE:
          m = n
          while m + 1 <= j and vs[m + 1][1] > vs[n_min][1] + 0.01:
            m += 1
          if m >= j and not any(vs[q][1] <= vs[n_min][1] + 0.01 for q in range(n, j + 1)):
            n += 1  # never came back down inside the episode: this is a genuine launch, not an escape
            continue
          travel = sum(vs[q][1] * (vs[q][0] - vs[q - 1][0])
                       for q in range(n_min + 1, m + 1) if 0.0 < vs[q][0] - vs[q - 1][0] < 0.5)
          escapes.append({
            "t_arrival": round(vs[n_min][0], 2),
            "v_arrival": round(vs[n_min][1], 3),
            "wire_at_arrival": round(at(cmds, vs[n_min][0]), 2) if cmds else None,
            "escape_peak_v": round(max(r[1] for r in vs[n_min:m + 1]), 3),
            "escape_travel_m": round(travel, 3),
            "wire_min_after": round(min((c for (t, c) in cmds if vs[n_min][0] <= t <= vs[m][0]), default=0.0), 2),
            "gap_at_arrival": round(at(leads, vs[n_min][0], 2), 2) if at(leads, vs[n_min][0], 1) else None,
          })
          n_min = m  # re-base: a later breakaway is a separate escape
          n = m
        n += 1
    k = j + 1

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
  print(json.dumps({"path": path.split("/realdata/")[-1], "events": events, "escapes": escapes}))


if __name__ == "__main__":
  main(sys.argv[1])
