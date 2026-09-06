#!/usr/bin/env python3
"""Cycle-14 honest 100 Hz per-stop metrics for the 18 stops in stoplist.tsv.  v3

Verified axes (axis_verify.py, two hard manual brakes, corr vs livePose calibrated):
  pitch rate   = raw gyroscope.gyroUncalibrated.v[1]  (= -angularVelocityDevice.y)
  longitudinal = -raw accelerometer.acceleration.v[2] (= accelerationDevice.x); offset-corrected
                 vs parked baseline it tracks aEgo with slope ~1.0.
carState.wheelSpeeds are ZERO in these logs; vEgo(<0.05) lags the true wheel stop by
0.15-0.5 s, so t_phys = midpoint of the max positive unload edge in 0.15 s-smoothed raw
forward accel (sign-flipped when gearShifter=reverse) within [t_kalman-1.5, t_kalman+0.3].
Gyro bias: per-stop parked median [t_phys+3,+6] when available, else the route-level
median of parked biases (parked biases agree to ~0.001 rad/s within a route).
"""
import json
import numpy as np
import capnp
import zstandard

REPO = "/Users/radoslawchybicki/Repos/openpilot-rch"
capnp.remove_import_hook()
LOG = capnp.load(f"{REPO}/cereal/log.capnp")
BASE = "/Users/radoslawchybicki/.route_sync/data/media/0/realdata"

STOPS = [
  ("00001f49--2a6fa30502--3", 208.13, "manual", 15.68),
  ("00001f49--2a6fa30502--61", 3675.09, "manual", 1.75),
  ("00001f4a--d32dd70102--12", 742.55, "manual", 16.05),
  ("00001f4a--d32dd70102--4", 254.25, "manual", 6.52),
  ("00001f4b--f5d57a7f89--0", 17.75, "manual", 1.86),
  ("00001f4b--f5d57a7f89--5", 347.05, "manual", 13.66),
  ("00001f4c--e8e297ccb6--16", 1002.64, "manual", 15.39),
  ("00001f4c--e8e297ccb6--55", 3314.36, "manual", 3.72),
  ("00001f4c--e8e297ccb6--56", 3393.17, "auto", 3.89),
  ("00001f4c--e8e297ccb6--56", 3418.17, "auto", 4.54),
  ("00001f4c--e8e297ccb6--57", 3466.17, "manual", 3.42),
  ("00001f4c--e8e297ccb6--57", 3479.47, "manual", 2.91),
  ("00001f4c--e8e297ccb6--58", 3521.47, "manual", 4.88),
  ("00001f4c--e8e297ccb6--60", 3606.97, "manual", 2.23),
  ("00001f4c--e8e297ccb6--60", 3632.87, "manual", 5.78),
  ("00001f4c--e8e297ccb6--61", 3691.17, "manual", 3.98),
  ("00001f4c--e8e297ccb6--82", 4929.88, "manual", 11.09),
  ("00001f4d--51907b5a56--9", 586.17, "manual(ex-mixed)", 13.21),
]

V_STAND = 0.09  # Kalman vEgo resting junk runs up to ~0.08; stationarity gate

_cache = {}

def load(seg):
  if seg in _cache:
    return _cache[seg]
  raw = zstandard.ZstdDecompressor().decompress(open(f"{BASE}/{seg}/rlog.zst", "rb").read(), max_output_size=int(9e8))
  cs, wire, gy, ax, lp, gear_rev = [], [], [], [], [], []
  t0 = None
  for ev in LOG.Event.read_multiple_bytes(raw):
    t = ev.logMonoTime * 1e-9
    if t0 is None:
      t0 = t
    t -= t0
    w = ev.which()
    if w == "carState":
      c = ev.carState
      cs.append((t, c.vEgo, c.aEgo, float(c.brakePressed)))
      gear_rev.append((t, 1.0 if c.gearShifter == "reverse" else 0.0))
    elif w == "carOutput":
      wire.append((t, ev.carOutput.actuatorsOutput.accel))
    elif w == "gyroscope":
      g = ev.gyroscope
      v = g.gyroUncalibrated.v if g.which() == "gyroUncalibrated" else g.gyro.v
      if len(v) >= 3:
        gy.append((t, v[1]))
    elif w == "accelerometer":
      a = ev.accelerometer
      if a.which() == "acceleration" and len(a.acceleration.v) >= 3:
        ax.append((t, -a.acceleration.v[2]))  # forward accel (verified axis+sign)
    elif w == "livePose":
      lp.append((t, ev.livePose.accelerationDevice.x))
  out = tuple(np.array(x) for x in (cs, wire, gy, ax, lp, gear_rev))
  _cache[seg] = out
  return out


def smooth(t, y, half=0.075):
  j0 = np.searchsorted(t, t - half)
  j1 = np.searchsorted(t, t + half)
  c = np.cumsum(np.insert(y, 0, 0.0))
  return (c[j1] - c[j0]) / np.maximum(j1 - j0, 1)


def one(seg, hint, mode, v_scan, bias_override=None):
  flags = []
  cs, wire, gy, ax, lp, gear_rev = load(seg)
  tc, v, a = cs[:, 0], cs[:, 1], cs[:, 2]

  # kalman stop (matches first-pass convention)
  t_k = None
  for gate in (0.5, 0.25):
    seen = False
    for t, vv in zip(tc, v):
      if vv > gate:
        seen = True
      if seen and vv < 0.05 and t >= hint - 3.0:
        t_k = float(t)
        break
    if t_k is not None:
      break

  def vat(t):
    return float(np.interp(t, tc, v))

  # reverse maneuver?
  rev = float(np.interp(t_k - 1.5, gear_rev[:, 0], gear_rev[:, 1])) > 0.5
  if rev:
    flags.append("reverse_gear_parking_maneuver")
  ta = ax[:, 0]
  af = -ax[:, 1] if rev else ax[:, 1]   # sign-flip so braking-to-stop is always negative
  afs = smooth(ta, af)

  # ---- physical stop: max positive unload edge ----
  m_e = (ta >= t_k - 1.5) & (ta <= t_k + 0.3)
  te = ta[m_e]
  d = np.interp(te + 0.12, ta, afs) - np.interp(te - 0.12, ta, afs)
  ok = np.array([0.03 <= vat(t) <= 0.6 for t in te])
  i = int(np.argmax(np.where(ok, d, -np.inf))) if ok.any() else int(np.argmax(d))
  if not ok.any():
    flags.append("edge_outside_v_band")
  t_phys = float(te[i])
  edge_amp = float(d[i])
  if edge_amp < 0.15:
    flags.append("weak_unload_edge_t_phys_uncertain")

  # ---- stationary span after the stop ----
  m_after = (tc >= t_phys) & (tc <= t_phys + 8.0)
  t_aft, v_aft = tc[m_after], v[m_after]
  still = np.where(v_aft < V_STAND)[0]  # Kalman vEgo still decays for up to ~1 s after t_phys
  i0 = still[0] if len(still) else 0
  moving = np.where(v_aft[i0:] >= V_STAND)[0]
  span_end = float(t_aft[i0 + moving[0]]) if len(moving) else float(t_aft[-1])
  stop_dur = span_end - t_phys
  if stop_dur < 2.0:
    flags.append(f"brief_stop_{stop_dur:.1f}s_then_moves")

  # ---- parked baseline of raw fwd accel ----
  b_lo, b_hi = t_phys + 0.7, min(span_end - 0.05, t_phys + 3.0)
  if b_hi - b_lo < 0.4:
    b_lo, b_hi = t_phys + 0.25, span_end - 0.05
    flags.append("baseline_from_brief_span")
  m_base = (ta >= b_lo) & (ta <= b_hi)
  base = float(np.median(af[m_base])) if m_base.sum() >= 10 else float("nan")
  if np.isnan(base):
    flags.append("no_baseline")
  a_true = afs - base

  # ---- approach / peak / release ----
  v_appr6 = float(np.max(v[(tc >= t_phys - 6.0) & (tc <= t_phys)]))
  m_ap = (ta >= t_phys - 6.0) & (ta <= t_phys - 0.3)
  peak_decel = float(-a_true[m_ap].min())
  t_peak = float(ta[m_ap][np.argmin(a_true[m_ap])])
  m_ap_cs = (tc >= t_phys - 6.0) & (tc <= t_phys - 0.5) & (v > 0.3)
  peak_decel_aego = float(-a[m_ap_cs].min()) if m_ap_cs.any() else float("nan")

  m_rel = (ta >= t_peak) & (ta <= t_phys - 0.25)
  if m_rel.any():
    i_r = int(np.argmax(a_true[m_rel]))
    release_min = float(-a_true[m_rel][i_r])
    t_release = float(t_phys - ta[m_rel][i_r])
    v_at_release = float(vat(ta[m_rel][i_r]))
  else:
    release_min = t_release = v_at_release = float("nan")
    flags.append("no_release_window")
  release_frac = release_min / peak_decel if peak_decel > 0 else float("nan")
  m_rel_cs = (tc >= t_peak) & (tc <= t_phys) & (v > 0.3)
  release_min_aego = float(-a[m_rel_cs].max()) if m_rel_cs.any() else float("nan")

  # ---- honest decel at the stop instant ----
  m_bef = (ta >= t_phys - 0.40) & (ta <= t_phys - 0.15)
  decel_at_stop = float(-a_true[m_bef].mean())
  if len(lp) and not rev:
    tl, lx = lp[:, 0], lp[:, 1]
    l_bef = (tl >= t_phys - 0.45) & (tl <= t_phys - 0.10)
    l_aft = (tl >= b_lo) & (tl <= b_hi)
    decel_at_stop_lp = float(lx[l_aft].mean() - lx[l_bef].mean()) if (l_aft.any() and l_bef.any()) else float("nan")
  else:
    decel_at_stop_lp = float("nan")
  reapply = decel_at_stop - release_min

  m_sane = (tc >= t_phys - 6.0) & (tc <= t_phys - 0.5) & (v > 0.5)
  slope = float(np.polyfit(a[m_sane], np.interp(tc[m_sane], ta, a_true), 1)[0]) if m_sane.sum() > 50 else float("nan")
  if not np.isnan(slope) and abs(slope - 1.0) > 0.25:
    flags.append("approach_grade_drift_slope_%.2f" % slope)

  # ---- gyro bias + bob (keyed to t_phys) ----
  tg, gv = gy[:, 0], gy[:, 1]
  m_park = (tg >= t_phys + 3.0) & (tg <= t_phys + 6.0)
  parked_ok = m_park.sum() > 100 and all(vat(t) < V_STAND for t in tg[m_park])
  bias_parked = float(np.median(gv[m_park])) if parked_ok else None
  sigma_parked = float(1.4826 * np.median(np.abs(gv[m_park] - bias_parked))) if parked_ok else None
  m_roll = (tg >= t_phys - 8.0) & (tg <= t_phys - 6.0)
  bias_rolling = float(np.median(gv[m_roll])) if m_roll.any() else None
  if bias_parked is not None:
    bias, bias_source = bias_parked, "parked[+3,+6]"
  elif bias_override is not None:
    bias, bias_source = bias_override, "route_parked_median"
  else:
    bias, bias_source = bias_rolling, "rolling[-8,-6]"
  gcorr = gv - bias
  m_bob = (tg >= t_phys - 0.3) & (tg <= min(t_phys + 1.2, span_end))
  bob_peak = float(np.abs(gcorr[m_bob]).max())
  t_bob = float(tg[m_bob][np.argmax(np.abs(gcorr[m_bob]))] - t_phys)

  # ---- wire / brake at the physical stop ----
  tw, wv = wire[:, 0], wire[:, 1]
  wire_at_stop = float(wv[np.argmin(np.abs(tw - t_phys))])
  brake_at_stop = bool(np.interp(t_phys, tc, cs[:, 3]) > 0.5)

  return {
    "seg": seg, "mode": mode, "t_kalman_stop": round(t_k, 3), "t_phys_stop": round(t_phys, 3),
    "kalman_lag_s": round(t_k - t_phys, 2), "unload_edge_amp": round(edge_amp, 3),
    "stop_duration_s": round(stop_dur, 2) if stop_dur < 7.9 else ">8",
    "v_scan": v_scan, "v_appr6": round(v_appr6, 2),
    "peak_decel": round(peak_decel, 3), "peak_decel_aego": round(peak_decel_aego, 3),
    "release_min": round(release_min, 3), "release_min_aego_v>0.3": round(release_min_aego, 3),
    "release_frac": round(release_frac, 3),
    "t_release_before_stop": round(t_release, 2), "v_at_release": round(v_at_release, 2),
    "decel_at_stop": round(decel_at_stop, 3), "decel_at_stop_livepose": round(decel_at_stop_lp, 3),
    "reapply": round(reapply, 3),
    "bob_peak": round(bob_peak, 4), "t_bob_peak": round(t_bob, 2),
    "gyro_bias": round(bias, 4), "bias_source": bias_source,
    "gyro_bias_rolling_ref": round(bias_rolling, 4) if bias_rolling is not None else None,
    "gyro_sigma_parked": round(sigma_parked, 4) if sigma_parked is not None else None,
    "accel_slope_sanity": None if np.isnan(slope) else round(slope, 3),
    "wire_at_stop": round(wire_at_stop, 3), "brake_at_stop": brake_at_stop,
    "flags": flags,
    "_span_end": span_end,
  }


def main():
  # pass 1
  rows = [one(*s) for s in STOPS]
  # route-level parked bias
  route_bias = {}
  for r in rows:
    if r["bias_source"] == "parked[+3,+6]":
      route_bias.setdefault(r["seg"].split("--")[1], []).append(r["gyro_bias"])
  route_bias = {k: float(np.median(v)) for k, v in route_bias.items()}
  global_bias = float(np.median([b for v in route_bias.values() for b in [v]]))
  # pass 2 for stops without a parked window
  for i, (r, s) in enumerate(zip(rows, STOPS)):
    if r["bias_source"] != "parked[+3,+6]":
      ov = route_bias.get(s[0].split("--")[1], global_bias)
      rows[i] = one(*s, bias_override=ov)

  sig_parked = [r["gyro_sigma_parked"] for r in rows if r["gyro_sigma_parked"] is not None]
  sigma_g = float(np.median(sig_parked))
  thr = round(max(4.0 * sigma_g, 0.01), 4)
  for r, s in zip(rows, STOPS):
    _cs, _w, gy, _a, _l, _g = load(s[0])
    t_phys, span_end = r["t_phys_stop"], r.pop("_span_end")
    tg, gv = gy[:, 0], gy[:, 1]
    gc = np.abs(gv - r["gyro_bias"])
    t_hi = min(t_phys + 3.0, span_end)
    m = (tg >= t_phys) & (tg <= t_hi)
    ts, gs = tg[m], gc[m]
    if t_hi - t_phys < 0.5:
      r["bob_settle_s"] = None
      r["flags"].append("settle_window_lt_0.5s")
      continue
    exc = np.where((gs[:-1] >= thr) & (gs[1:] >= thr))[0]  # 2 consecutive samples
    r["bob_settle_s"] = round(float(ts[exc[-1] + 1] - t_phys), 2) if len(exc) else 0.0

  out = {
    "axis_findings": {
      "pitch_rate_raw_axis": "gyroscope.gyroUncalibrated.v[1] (= -livePose.angularVelocityDevice.y; corr -0.67/-0.91 on the two hard brakes; first-pass axis guess y was RIGHT, sign inverted vs device frame)",
      "yaw_raw_axis": "v[0] (corr -0.98/-1.00 vs angularVelocityDevice.z)",
      "roll_raw_axis": "v[2] (corr -0.81/-0.94 vs angularVelocityDevice.x)",
      "longitudinal_accel": "-accelerometer.acceleration.v[2] (raw v[2] corr -0.92/-0.96 vs aEgo, -0.95/-0.98 vs livePose.accelerationDevice.x); raw v[0] carries gravity (+9.6); offset-corrected slope vs aEgo ~1.0-1.06 on clean stops",
      "wheelSpeeds": "carState.wheelSpeeds is ALL ZEROS in these rlogs - unusable for the stop instant",
      "t_phys_method": "max positive unload edge in 0.15s-smoothed raw fwd accel; the Kalman vEgo<0.05 instant LAGS it by kalman_lag_s (0.15-0.5 s) - all first-pass stop-instant metrics were windowed on the wrong instant",
    },
    "noise_floor": {
      "gyro_sigma_parked_median_rad_s": round(sigma_g, 4),
      "per_stop_parked_sigmas": sig_parked,
      "settle_threshold_rad_s": thr,
      "settle_def": "last time in [t_phys, min(t_phys+3, span_end)] with |pitch-bias| >= thr for 2 consecutive samples",
      "route_parked_bias_rad_s": route_bias,
    },
    "caveats": [
      "decel_at_stop = parked-baseline-corrected raw accel averaged over [t_phys-0.40,-0.15]; suspension-pitch gravity leakage across the unload puts ~+-0.1 m/s^2 uncertainty on it; livePose cross-check agrees within 0.03-0.09 on all non-reverse stops",
      "negative release_min = the car briefly ACCELERATED during the release (full brake release + creep)",
      "3 stops are not representative technique stops: 00001f4b--0 is a reverse-gear parking maneuver, 00001f4c--57@3466 is a 0.6 s inch-forward dip in stop-and-go, 00001f4d--9 is a brake-free creep dip 3 s before the final park",
      "00001f4a--12 parked gyro sigma is 0.0045 (vs 0.0026 fleet median): its bob_settle_s=2.96 likely reflects elevated idle vibration, not body bob",
      "wire_at_stop is 0.0 on manual stops (disengaged, actuatorsOutput zeroed); meaningful only on the 2 auto stops",
      "release/peak metrics are computed from the offset-corrected raw accel channel (valid below v=0.3 where aEgo is not); *_aego columns are the v>0.3-gated reference",
    ],
    "stops": rows,
  }
  with open("/tmp/sc14/corrected_metrics.json", "w") as f:
    json.dump(out, f, indent=1)
  print(json.dumps(out, indent=1))


if __name__ == "__main__":
  main()
