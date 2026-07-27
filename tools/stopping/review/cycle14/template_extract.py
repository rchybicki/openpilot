#!/usr/bin/env python3
"""Cycle-14 human perfect-stop TEMPLATE extraction.

Aligns all 18 stops at the PHYSICAL wheel-stop instant t_phys (from corrected_metrics.json,
same code path as honest_metrics.py), overlays 100 Hz decel / velocity / bias-removed pitch
rate, computes per-stop technique parameters, and fits the parametric recipe vs approach speed.

Channels & conventions (verified in axis_verify.py):
  decel d(t)  = -(smoothed(-accelerometer.v[2]) - parked_baseline)   [positive = braking]
  pitch rate  = gyroscope.gyroUncalibrated.v[1] - per-stop bias
  v_recon(t)  = integral_t^{t_phys} d(s) ds  (honest 100 Hz velocity for the last 2.5 s,
                anchored at v=0 at t_phys; does not lag/floor like Kalman vEgo)
"""
import sys, json
import numpy as np

sys.path.insert(0, "/tmp/sc14")
import honest_metrics as hm

ROWS = json.load(open("/tmp/sc14/corrected_metrics.json"))["stops"]
STOPS = hm.STOPS
V_STAND = hm.V_STAND

NONREP = {4: "reverse-gear parking maneuver", 10: "0.6s inch-forward dip in stop-and-go",
          17: "brake-free creep dip, final park 3s later"}
AUTO = {8, 9}

GRID_T0, GRID_T1, GRID_DT = -6.0, 2.0, 0.02
grid = np.round(np.arange(GRID_T0, GRID_T1 + 1e-9, GRID_DT), 3)


def cumtrapz(y, t):
  return np.concatenate([[0.0], np.cumsum(0.5 * (y[1:] + y[:-1]) * np.diff(t))])


def analyze(i):
  seg, hint, mode, v_scan = STOPS[i]
  row = ROWS[i]
  cs, wire, gy, ax, lp, gear_rev = hm.load(seg)
  tc, v, a, brk = cs[:, 0], cs[:, 1], cs[:, 2], cs[:, 3]
  t_phys = row["t_phys_stop"]
  rev = any("reverse" in f for f in row["flags"])
  ta = ax[:, 0]
  af = -ax[:, 1] if rev else ax[:, 1]
  afs = hm.smooth(ta, af)

  def vat(t):
    return float(np.interp(t, tc, v))

  # stationary span after stop (copy of honest_metrics logic)
  m_after = (tc >= t_phys) & (tc <= t_phys + 8.0)
  t_aft, v_aft = tc[m_after], v[m_after]
  still = np.where(v_aft < V_STAND)[0]
  i0 = still[0] if len(still) else 0
  moving = np.where(v_aft[i0:] >= V_STAND)[0]
  span_end = float(t_aft[i0 + moving[0]]) if len(moving) else float(t_aft[-1])

  # parked baseline
  b_lo, b_hi = t_phys + 0.7, min(span_end - 0.05, t_phys + 3.0)
  if b_hi - b_lo < 0.4:
    b_lo, b_hi = t_phys + 0.25, span_end - 0.05
  m_base = (ta >= b_lo) & (ta <= b_hi)
  base = float(np.median(af[m_base])) if m_base.sum() >= 10 else 0.0
  d_true = -(afs - base)  # decel, positive while braking

  # cross-check against corrected_metrics
  m_ap = (ta >= t_phys - 6.0) & (ta <= t_phys - 0.3)
  i_pk = int(np.argmax(d_true[m_ap]))
  d_peak, t_peak = float(d_true[m_ap][i_pk]), float(ta[m_ap][i_pk])
  assert abs(d_peak - row["peak_decel"]) < 0.06, (seg, d_peak, row["peak_decel"])

  m_rel = (ta >= t_peak) & (ta <= t_phys - 0.25)
  i_rm = int(np.argmin(d_true[m_rel]))
  d_rmin, t_rmin = float(d_true[m_rel][i_rm]), float(ta[m_rel][i_rm])

  m_carry = (ta >= t_phys - 0.40) & (ta <= t_phys - 0.15)
  carry = float(d_true[m_carry].mean())
  reapply = carry - d_rmin

  # release start: last time in [t_peak, t_rmin] decel still >= rmin + 85% of the drop
  m_rs = (ta >= t_peak) & (ta <= t_rmin)
  thr85 = d_rmin + 0.85 * (d_peak - d_rmin)
  above = np.where(d_true[m_rs] >= thr85)[0]
  t_rel_start = float(ta[m_rs][above[-1]]) if len(above) else t_peak
  rel_drop_rate = (d_peak - d_rmin) / max(t_rmin - t_rel_start, 0.05)  # m/s^3, decel falling

  # reapply onset + rebuild rate (only if a distinct rebuild exists)
  t_reapp = rebuild_rate = None
  if reapply >= 0.15:
    m_rb = (ta >= t_rmin) & (ta <= t_phys - 0.15)
    trb, drb = ta[m_rb], d_true[m_rb]
    c25 = np.where(drb >= d_rmin + 0.25 * reapply)[0]
    c75 = np.where(drb >= d_rmin + 0.75 * reapply)[0]
    if len(c25) and len(c75):
      t25, t75 = float(trb[c25[0]]), float(trb[c75[0]])
      t_reapp = t25
      if t75 > t25 + 0.02:
        rebuild_rate = 0.5 * reapply / (t75 - t25)

  # v_recon: honest 100 Hz velocity over the last 2.5 s (v=0 at t_phys)
  m_vr = (ta >= t_phys - 2.5) & (ta <= t_phys)
  tvr = ta[m_vr]
  C = cumtrapz(d_true[m_vr], tvr)
  v_recon = C[-1] - C  # decreasing to 0 at t_phys
  # sanity: v_recon vs Kalman at v~0.8 (Kalman reliable there)
  chk = None
  hi = np.where(v[(tc >= t_phys - 2.5) & (tc <= t_phys)] >= 0.8)[0]
  tt = tc[(tc >= t_phys - 2.5) & (tc <= t_phys)]
  if len(hi):
    t8 = float(tt[hi[-1]])
    if t8 > tvr[0]:
      chk = round(float(np.interp(t8, tvr, v_recon)) - vat(t8), 2)

  def v_of(t):
    """blended speed: Kalman above 0.6 (reliable), v_recon below (Kalman lags/floors)."""
    vk = vat(t)
    if vk >= 0.6 or t < tvr[0] or t > t_phys:
      return vk
    return max(float(np.interp(t, tvr, v_recon)), 0.0)

  # distance-to-stop via blended v on a 100 Hz grid
  tg100 = np.arange(t_phys - 6.0, t_phys, 0.01)
  vb = np.array([v_of(t) for t in tg100])
  D = cumtrapz(vb, tg100)
  def dist_of(t):
    return float(D[-1] - np.interp(t, tg100, D))

  # terminal decel three ways
  m_kal = (tc >= t_phys - 3.0) & (tc <= t_phys) & (v >= 0.2) & (v <= 0.8)
  term_kal = float(-np.polyfit(tc[m_kal], v[m_kal], 1)[0]) if m_kal.sum() > 10 else None
  m_t4 = tvr >= t_phys - 0.4
  term_recon = float(-np.polyfit(tvr[m_t4], v_recon[m_t4], 1)[0]) if m_t4.sum() > 10 else None
  d_m10_m07 = float(d_true[(ta >= t_phys - 1.0) & (ta <= t_phys - 0.7)].mean())
  d_m07_m04 = float(d_true[(ta >= t_phys - 0.7) & (ta <= t_phys - 0.4)].mean())

  v_ap = row["v_appr6"]
  p = {
    "seg": seg, "idx": i, "mode": mode, "t_phys": t_phys,
    "representative": i not in NONREP and i not in AUTO,
    "excluded_reason": NONREP.get(i),
    "v_appr6": v_ap,
    "d_peak": round(d_peak, 3), "t_peak_before_stop": round(t_phys - t_peak, 2),
    "v_at_peak": round(v_of(t_peak), 2), "dist_at_peak": round(dist_of(t_peak), 2),
    "t_rel_start_before_stop": round(t_phys - t_rel_start, 2),
    "v_at_rel_start": round(v_of(t_rel_start), 2), "dist_at_rel_start": round(dist_of(t_rel_start), 2),
    "rel_drop_rate": round(rel_drop_rate, 2),
    "d_release_min": round(d_rmin, 3), "release_frac": round(d_rmin / d_peak, 3) if d_peak > 0 else None,
    "t_rel_min_before_stop": round(t_phys - t_rmin, 2), "v_at_rel_min": round(v_of(t_rmin), 2),
    "dist_at_rel_min": round(dist_of(t_rmin), 2),
    "carry_decel_at_stop": round(carry, 3), "reapply": round(reapply, 3),
    "t_reapply_before_stop": round(t_phys - t_reapp, 2) if t_reapp else None,
    "v_at_reapply": round(v_of(t_reapp), 2) if t_reapp else None,
    "rebuild_rate": round(rebuild_rate, 2) if rebuild_rate else None,
    "shape": "release-dwell-reapply" if reapply >= 0.15 else "taper-to-touch",
    "terminal_decel_recon_last0.4s": round(term_recon, 3) if term_recon is not None else None,
    "terminal_decel_kalman_v0.2-0.8": round(term_kal, 3) if term_kal is not None else None,
    "d_at_[-1.0,-0.7]": round(d_m10_m07, 3), "d_at_[-0.7,-0.4]": round(d_m07_m04, 3),
    "bob_peak": row["bob_peak"], "bob_settle_s": row.get("bob_settle_s"),
    "v_recon_vs_kalman_at_v0.8": chk,
    "flags": row["flags"],
  }

  # ---- overlay on common grid ----
  tg = t_phys + grid
  gz = hm.smooth(gy[:, 0], gy[:, 1] - row["gyro_bias"], half=0.05)
  ov = {
    "v_ego": np.round(np.interp(tg, tc, v), 3).tolist(),
    "a_ego": np.round(np.interp(tg, tc, a), 3).tolist(),
    "decel_true": np.round(np.interp(tg, ta, d_true), 3).tolist(),
    "pitch_rate": np.round(np.interp(tg, gy[:, 0], gz), 4).tolist(),
    "brake_pressed": [int(x > 0.5) for x in np.interp(tg, tc, brk)],
    "v_recon": [round(float(np.interp(t, tvr, v_recon)), 3) if tvr[0] <= t <= t_phys else None for t in tg],
  }
  return p, ov


params, overlays = [], []
for i in range(len(STOPS)):
  p, ov = analyze(i)
  params.append(p)
  overlays.append({"seg": p["seg"], "idx": i, "mode": p["mode"], "t_phys": p["t_phys"],
                   "representative": p["representative"], **ov})
  print(f"done {i} {p['seg']}", file=sys.stderr)

rep = [p for p in params if p["representative"]]

def arr(k):
  return np.array([p[k] for p in rep], dtype=float)

v_ap, d_pk, carry, bob = arr("v_appr6"), arr("d_peak"), arr("carry_decel_at_stop"), arr("bob_peak")
rel_fr = arr("release_frac")

def fit(x, y):
  b, a = np.polyfit(x, y, 1)
  r = float(np.corrcoef(x, y)[0, 1])
  return {"slope": round(float(b), 3), "intercept": round(float(a), 3), "r": round(r, 2)}

fits = {
  "n_representative": len(rep),
  "d_peak_vs_v_appr6": fit(v_ap, d_pk),
  "v_at_rel_start_vs_v_appr6": fit(v_ap, arr("v_at_rel_start")),
  "dist_at_rel_start_vs_v_appr6": fit(v_ap, arr("dist_at_rel_start")),
  "t_rel_start_before_stop_vs_v_appr6": fit(v_ap, arr("t_rel_start_before_stop")),
  "release_frac_vs_v_appr6": fit(v_ap, rel_fr),
  "carry_vs_v_appr6": fit(v_ap, carry),
  "bob_vs_carry": fit(carry, bob),
  "bob_vs_d_peak": fit(d_pk, bob),
  "bob_vs_v_appr6": fit(v_ap, bob),
  "bob_vs_terminal_recon": fit(arr("terminal_decel_recon_last0.4s"), bob),
}

def med_iqr(k, sub=rep):
  x = np.array([p[k] for p in sub if p.get(k) is not None], dtype=float)
  return {"median": round(float(np.median(x)), 3), "p25": round(float(np.percentile(x, 25)), 3),
          "p75": round(float(np.percentile(x, 75)), 3), "n": len(x)}

summary = {k: med_iqr(k) for k in
           ["d_peak", "release_frac", "v_at_rel_start", "t_rel_start_before_stop", "dist_at_rel_start",
            "rel_drop_rate", "carry_decel_at_stop", "reapply", "terminal_decel_recon_last0.4s",
            "terminal_decel_kalman_v0.2-0.8", "d_at_[-1.0,-0.7]", "d_at_[-0.7,-0.4]", "bob_peak"]}
shapeA = [p for p in rep if p["shape"] == "release-dwell-reapply"]
shapeB = [p for p in rep if p["shape"] == "taper-to-touch"]

out = {
  "meta": {
    "alignment": "t=0 is t_phys (physical wheel stop, max positive unload edge in 0.15s-smoothed raw fwd accel)",
    "grid": {"t0": GRID_T0, "t1": GRID_T1, "dt": GRID_DT, "n": len(grid)},
    "channels": {
      "v_ego": "Kalman vEgo (lags ~0.33s near stop, floors near 0)",
      "a_ego": "carState.aEgo (floors below ~0.03 m/s)",
      "decel_true": "positive-braking decel = -(0.15s-smoothed(-accel.v[2]) - parked baseline)",
      "pitch_rate": "gyroUncalibrated.v[1] - per-stop parked bias, 0.1s smoothed; noise sigma 0.0026",
      "v_recon": "integral of decel_true back from v=0 at t_phys; honest 100Hz terminal velocity",
    },
  },
  "per_stop_params": params,
  "fits_representative_manual": fits,
  "summary_representative_manual": summary,
  "shape_split": {
    "release_dwell_reapply": {"n": len(shapeA), "segs": [p["seg"] for p in shapeA],
                              "reapply": med_iqr("reapply", shapeA) if shapeA else None,
                              "carry": med_iqr("carry_decel_at_stop", shapeA) if shapeA else None,
                              "bob": med_iqr("bob_peak", shapeA) if shapeA else None},
    "taper_to_touch": {"n": len(shapeB), "segs": [p["seg"] for p in shapeB],
                       "carry": med_iqr("carry_decel_at_stop", shapeB) if shapeB else None,
                       "bob": med_iqr("bob_peak", shapeB) if shapeB else None},
  },
  "overlays": overlays,
}
with open("/tmp/sc14/template.json", "w") as f:
  json.dump(out, f)

del out["overlays"], out["per_stop_params"]
print(json.dumps(out, indent=1))
print("\nPER-STOP TABLE (representative manual first):")
cols = ["seg", "v_appr6", "d_peak", "t_rel_start_before_stop", "v_at_rel_start", "dist_at_rel_start",
        "release_frac", "d_release_min", "t_rel_min_before_stop", "v_at_rel_min", "carry_decel_at_stop",
        "reapply", "t_reapply_before_stop", "v_at_reapply", "rebuild_rate",
        "terminal_decel_recon_last0.4s", "d_at_[-1.0,-0.7]", "d_at_[-0.7,-0.4]", "bob_peak", "shape",
        "v_recon_vs_kalman_at_v0.8"]
for p in sorted(params, key=lambda x: (not x["representative"], x["seg"])):
  print(" | ".join(str(p.get(c)) for c in cols) + (" NONREP:" + str(p["excluded_reason"]) if p["excluded_reason"] else "") + (" AUTO" if "auto" in p["mode"] else ""))
