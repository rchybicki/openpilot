#!/usr/bin/env python3
"""Verify which raw gyro axis is pitch rate and which raw accel axis is longitudinal.

Method: around a hard manual brake (peak aEgo < -1.4), correlate each raw-sensor axis
(bias-removed) against the calibrated 20 Hz references:
  - livePose.angularVelocityDevice.y  (device-frame pitch rate)
  - livePose.accelerationDevice.x     (device-frame longitudinal accel)
  - carState.aEgo                     (wheel-derived longitudinal accel)
The raw axis with the dominant |corr| wins; sign of corr gives the sign convention.
Also print the raw pitch trace around the decel edge to see dive/rebound time-locking.
"""
import sys
import numpy as np
import capnp
import zstandard

REPO = "/Users/radoslawchybicki/Repos/openpilot-rch"
capnp.remove_import_hook()
LOG = capnp.load(f"{REPO}/cereal/log.capnp")
BASE = "/Users/radoslawchybicki/.route_sync/data/media/0/realdata"


def load(seg, t_lo, t_hi):
  raw = zstandard.ZstdDecompressor().decompress(open(f"{BASE}/{seg}/rlog.zst", "rb").read(), max_output_size=int(9e8))
  cs, gyro, acc, lp_av, lp_ad = [], [], [], [], []
  t0 = None
  for ev in LOG.Event.read_multiple_bytes(raw):
    t = ev.logMonoTime * 1e-9
    if t0 is None:
      t0 = t
    t -= t0
    if t < t_lo or t > t_hi:
      continue
    w = ev.which()
    if w == "carState":
      c = ev.carState
      cs.append((t, c.vEgo, c.aEgo))
    elif w == "gyroscope":
      g = ev.gyroscope
      v = g.gyroUncalibrated.v if g.which() == "gyroUncalibrated" else g.gyro.v
      if len(v) >= 3:
        gyro.append((t, v[0], v[1], v[2]))
    elif w == "accelerometer":
      a = ev.accelerometer
      v = a.acceleration.v if a.which() == "acceleration" else None
      if v is not None and len(v) >= 3:
        acc.append((t, v[0], v[1], v[2]))
    elif w == "livePose":
      p = ev.livePose
      lp_av.append((t, p.angularVelocityDevice.x, p.angularVelocityDevice.y, p.angularVelocityDevice.z))
      lp_ad.append((t, p.accelerationDevice.x, p.accelerationDevice.y, p.accelerationDevice.z))
  return (np.array(cs), np.array(gyro), np.array(acc), np.array(lp_av), np.array(lp_ad))


def corr(a, b):
  a = a - a.mean(); b = b - b.mean()
  d = np.sqrt((a * a).sum() * (b * b).sum())
  return float((a * b).sum() / d) if d > 0 else 0.0


def main(seg, t_hint):
  cs, gyro, acc, lp_av, lp_ad = load(seg, t_hint - 20, t_hint + 6)
  tc, v, a = cs[:, 0], cs[:, 1], cs[:, 2]
  # hard brake event: deepest aEgo before the stop
  i_pk = int(np.argmin(a))
  print(f"== {seg}  t_stop_hint={t_hint}  peak aEgo={a[i_pk]:.2f} at t={tc[i_pk]:.2f} v={v[i_pk]:.2f}")
  # analysis window around the brake event
  w_lo, w_hi = tc[i_pk] - 4.0, tc[i_pk] + 4.0
  gm = (gyro[:, 0] >= w_lo) & (gyro[:, 0] <= w_hi)
  am = (acc[:, 0] >= w_lo) & (acc[:, 0] <= w_hi)
  g, ac = gyro[gm], acc[am]
  print(f"   gyro rate ~{(len(g)-1)/(g[-1,0]-g[0,0]):.1f} Hz, accel ~{(len(ac)-1)/(ac[-1,0]-ac[0,0]):.1f} Hz, livePose n={len(lp_av)}")

  # --- gyro axes vs livePose.angularVelocityDevice.y ---
  lm = (lp_av[:, 0] >= w_lo) & (lp_av[:, 0] <= w_hi)
  lp = lp_av[lm]
  for ax, name in [(1, "gx"), (2, "gy"), (3, "gz")]:
    gi = np.interp(lp[:, 0], g[:, 0], g[:, ax])
    print(f"   corr(raw {name}, livePose.angVel.y) = {corr(gi, lp[:, 2]):+.3f}   "
          f"corr with angVel.x = {corr(gi, lp[:, 1]):+.3f}   angVel.z = {corr(gi, lp[:, 3]):+.3f}")

  # --- accel axes vs aEgo and livePose.accelerationDevice.x ---
  lmA = (lp_ad[:, 0] >= w_lo) & (lp_ad[:, 0] <= w_hi)
  lpa = lp_ad[lmA]
  csm = (tc >= w_lo) & (tc <= w_hi)
  for ax, name in [(1, "ax"), (2, "ay"), (3, "az")]:
    ai_cs = np.interp(tc[csm], ac[:, 0], ac[:, ax])
    ai_lp = np.interp(lpa[:, 0], ac[:, 0], ac[:, ax])
    print(f"   corr(raw {name}, aEgo) = {corr(ai_cs, a[csm]):+.3f}   corr(raw {name}, livePose.accDev.x) = {corr(ai_lp, lpa[:, 1]):+.3f}"
          f"   mean={ac[:, ax].mean():+.3f}")

  # --- time-locked dive/rebound print: decel edge vs candidate pitch axes ---
  print("   t_rel_to_peak |   aEgo |  raw gx  raw gy  raw gz | lp.avY")
  for dt in np.arange(-1.5, 3.01, 0.25):
    t = tc[i_pk] + dt
    ai = np.interp(t, tc, a)
    gx = np.interp(t, g[:, 0], g[:, 1]); gy = np.interp(t, g[:, 0], g[:, 2]); gz = np.interp(t, g[:, 0], g[:, 3])
    ly = np.interp(t, lp_av[:, 0], lp_av[:, 2])
    print(f"   {dt:+6.2f}       | {ai:+6.2f} | {gx:+7.3f} {gy:+7.3f} {gz:+7.3f} | {ly:+7.3f}")


if __name__ == "__main__":
  main(sys.argv[1], float(sys.argv[2]))
