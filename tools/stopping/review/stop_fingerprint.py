#!/usr/bin/env python3
"""Full-rate fingerprint of ONE stop: the shape of the final seconds, manual or autonomous.

Cycle-14 (user directive): the reference for a perfect stop is the HUMAN technique -- brake, ease
off near the end, re-apply gently right at rest -- and the question is how the autonomous profile
differs. deep_stop.py's 20 Hz livePose channels cannot resolve that; rlogs carry carState/carOutput
at ~101 Hz and raw accelerometer/gyroscope at ~104 Hz, which can.

Usage: stop_fingerprint.py <rlog.zst> <t_stop> [label]
  t_stop = approximate time (s, log-relative) the wheel speed first reads < 0.05.

Emits one JSON object:
  meta:     label, t_stop refined at 100 Hz, v_appr, mode (auto/manual from brakePressed/enabled)
  profile:  100 Hz arrays over [t_stop - 6 s, t_stop + 2 s]: t (rel to stop), v (vEgo), a (aEgo),
            wire (carOutput accel; NaN while disengaged), brake (brakePressed), pitch (gyro y, rad/s)
            downsampled x4 for JSON size (25 Hz effective) -- metrics below use the full rate.
  metrics:
    peak_decel        deepest 100 Hz aEgo in the approach window [-6, -0.5]
    release_min_decel shallowest |aEgo| between the peak and the stop (the human "let go")
    release_frac      release_min_decel / peak_decel  (0 = full release; 1 = no release at all)
    t_release         when that shallowest point occurs, s before the stop
    decel_at_stop     aEgo at the last rolling frame (the head-bob driver: suspension unloads this)
    reapply           decel_at_stop - release_min_decel (the human "brake gently again")
    bob_pitch_peak    peak |gyro.y| in [t_stop - 0.3, t_stop + 1.2] at 104 Hz (THE felt nod)
    bob_pitch_settle  time from t_stop until |gyro.y| stays < 0.01 rad/s for 0.3 s
    jerk_100          peak |d(aEgo)/dt| over [-1.5, +1.0], 100 Hz, 5-sample MA
"""
import json
import math
import sys

import capnp
import zstandard

REPO = "/Users/radoslawchybicki/Repos/openpilot-rch"
capnp.remove_import_hook()
LOG = capnp.load(f"{REPO}/cereal/log.capnp")


def main(path, t_stop_hint, label=""):
  raw = zstandard.ZstdDecompressor().decompress(open(path, "rb").read(), max_output_size=int(9e8))
  cs, wire, gyro, en = [], [], [], []
  t0 = None
  for ev in LOG.Event.read_multiple_bytes(raw):
    w = ev.which()
    t = ev.logMonoTime * 1e-9
    if t0 is None:
      t0 = t
    t -= t0
    if abs(t - t_stop_hint) > 12.0:
      continue
    if w == "carState":
      c = ev.carState
      cs.append((t, c.vEgo, c.aEgo, bool(c.brakePressed)))
    elif w == "carOutput":
      wire.append((t, ev.carOutput.actuatorsOutput.accel))
    elif w == "gyroscope":
      g = ev.gyroscope
      which = g.which()
      v = g.gyroUncalibrated.v if which == "gyroUncalibrated" else g.gyro.v
      if len(v) >= 2:
        gyro.append((t, v[1]))  # device y = pitch rate
    elif w == "selfdriveState":
      en.append((t, bool(ev.selfdriveState.enabled)))

  if not cs:
    print(json.dumps({"error": "no carState near hint", "path": path}))
    return

  # refine t_stop at 100 Hz: first frame < 0.05 after having been > 0.5 within 5 s before the hint
  t_stop = None
  seen_moving = False
  for (t, v, _a, _b) in cs:
    if v > 0.5:
      seen_moving = True
    if seen_moving and v < 0.05 and t >= t_stop_hint - 3.0:
      t_stop = t
      break
  if t_stop is None:
    print(json.dumps({"error": "no stop found near hint", "path": path}))
    return

  def near(arr, t, idx=1):
    return min(arr, key=lambda r: abs(r[0] - t))[idx] if arr else None

  win = [r for r in cs if t_stop - 6.0 <= r[0] <= t_stop + 2.0]
  v_appr = max(v for (_, v, _, _) in win)
  brake_used = any(b for (t, _, _, b) in win if t <= t_stop)
  def enab(t):
    x = [e for e in en if e[0] <= t]
    return x[-1][1] if x else False
  mode = "auto" if (enab(t_stop - 1.0) and not brake_used) else ("manual" if brake_used else "mixed")

  appr = [r for r in win if t_stop - 6.0 <= r[0] <= t_stop - 0.5]
  peak_decel = -min(a for (_, _, a, _) in appr) if appr else float("nan")
  t_peak = next((t for (t, _, a, _) in appr if -a == peak_decel), t_stop - 2.0)
  tail = [r for r in win if t_peak <= r[0] <= t_stop]
  rel = min(tail, key=lambda r: abs(r[2])) if tail else None
  release_min = abs(rel[2]) if rel else float("nan")
  t_release = t_stop - rel[0] if rel else float("nan")
  rolling = [r for r in win if r[1] >= 0.05 and r[0] <= t_stop]
  decel_at_stop = -rolling[-1][2] if rolling else float("nan")

  gw = [(t, y) for (t, y) in gyro if t_stop - 0.3 <= t <= t_stop + 1.2]
  bob_peak = max((abs(y) for (_, y) in gw), default=float("nan"))
  settle = float("nan")
  after = [(t, y) for (t, y) in gyro if t >= t_stop]
  for i, (t, _y) in enumerate(after):
    if all(abs(y2) < 0.01 for (t2, y2) in after[i:] if t2 <= t + 0.3):
      settle = t - t_stop
      break

  jw = [r for r in win if t_stop - 1.5 <= r[0] <= t_stop + 1.0]
  ma = [(jw[i][0], sum(x[2] for x in jw[max(0, i - 2):i + 3]) / len(jw[max(0, i - 2):i + 3]))
        for i in range(len(jw))]
  jerk = max((abs((b[1] - a[1]) / (b[0] - a[0])) for a, b in zip(ma, ma[1:], strict=False) if b[0] > a[0]),
             default=float("nan"))

  ds = [r for i, r in enumerate(win) if i % 4 == 0]
  profile = {
    "t": [round(t - t_stop, 3) for (t, _, _, _) in ds],
    "v": [round(v, 3) for (_, v, _, _) in ds],
    "a": [round(a, 3) for (_, _, a, _) in ds],
    "wire": [round(near(wire, t), 3) if near(wire, t) is not None else None for (t, _, _, _) in ds],
    "brake": [int(b) for (_, _, _, b) in ds],
    "pitch": [round(near(gyro, t), 4) if near(gyro, t) is not None else None for (t, _, _, _) in ds],
  }
  def r2(x):
    return round(x, 3) if isinstance(x, float) and math.isfinite(x) else None
  print(json.dumps({
    "path": path.split("/realdata/")[-1], "label": label,
    "meta": {"t_stop": round(t_stop, 2), "v_appr": round(v_appr, 2), "mode": mode},
    "metrics": {
      "peak_decel": r2(peak_decel), "release_min_decel": r2(release_min),
      "release_frac": r2(release_min / peak_decel) if peak_decel else None,
      "t_release": r2(t_release), "decel_at_stop": r2(decel_at_stop),
      "reapply": r2(decel_at_stop - release_min),
      "bob_pitch_peak": r2(bob_peak), "bob_pitch_settle_s": r2(settle),
      "jerk_100": r2(jerk),
    },
    "profile": profile,
  }))


if __name__ == "__main__":
  main(sys.argv[1], float(sys.argv[2]), sys.argv[3] if len(sys.argv) > 3 else "")
