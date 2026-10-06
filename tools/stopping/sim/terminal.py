"""(sim package copy of ~/.route_sync/corpus/kcs1_drive1_20260926/analysis2/terminal_hold/terminal.py: body() and metrics() only)
Terminal + hold metrics for KCS1 reps and Radek's marked manual stops, one definition for both.

Frames: series times in seconds (KCS1: from t0; Radek: from the baseline rest time). Body = gravity-compensated forward
specific force from the raw accelerometer in the calibrated frame (as kcs1_reps.body_frame); pitch rate from
gyroUncalibrated with an at-rest bias (flag+0.7 .. flag+1.5 s, before any brake-end). Positive pitch rate = nose up.
"""
import numpy as np
from openpilot.common.transformations.orientation import rot_from_euler

G = 9.81


def trailing_mean(t, x, win):
  c = np.concatenate(([0.0], np.cumsum(x)))
  j = np.searchsorted(t, t - win, side='right')
  n = np.arange(1, len(t) + 1) - j
  out = (c[1:] - c[j]) / np.maximum(n, 1)
  out[n < 5] = np.nan
  return out


def ls_slope(t, x, a, b):
  m = (t >= a) & (t <= b)
  if m.sum() < 5:
    return None
  tt = t[m] - t[m].mean()
  return float(np.dot(tt, x[m] - x[m].mean()) / np.dot(tt, tt))


def at(t, x, q, stale=0.1):
  i = np.searchsorted(t, q, side='right') - 1
  return float(x[i]) if i >= 0 and q - t[i] <= stale else None


def body(S, t_ref):
  ct = S['calib__t']
  i = max(np.searchsorted(ct, t_ref, side='right') - 1, 0)
  R = rot_from_euler([S['calib__roll'][i], S['calib__pitch'][i], S['calib__yaw'][i]])
  f = np.stack([-S['imu__z'], -S['imu__y'], -S['imu__x']], 1) @ R
  ok = np.isfinite(S['cc__pitch'])
  pitch = np.interp(S['imu__t'], S['cc__t'][ok], S['cc__pitch'][ok])
  w = np.stack([-S['gyro__z'], -S['gyro__y'], -S['gyro__x']], 1) @ R
  return S['imu__t'], f[:, 0] - G * np.sin(pitch), S['gyro__t'], w[:, 1]


def metrics(S, t_flag, t_end):
  """t_end: end of the undisturbed hold (driver's brake for KCS1; None = open)."""
  wt, wv = S['whl__t'], S['whl__mean']
  i_flag = np.searchsorted(wt, t_flag, side='right')
  above = np.nonzero(wv[:i_flag] > 0.5)[0]
  i = above[-1]
  t05 = float(np.interp(0.5, [wv[i + 1], wv[i]], [wt[i + 1], wt[i]]))
  bt, bl, gt, pr = body(S, t05 - 3.0)
  m01 = trailing_mean(bt, bl, 0.1)
  hi = t_flag + 1.5 if t_end is None else min(t_flag + 1.5, t_end - 0.05)
  bm = (gt >= t_flag + 0.7) & (gt <= hi)
  bias = float(np.median(pr[bm])) if bm.sum() >= 30 else float(np.median(pr[(gt >= t05 - 4.0) & (gt <= t05 - 2.0)]))
  prc = pr - bias
  pr05 = trailing_mean(gt, prc, 0.05)
  # pitch angle relative to t05 (deg)
  m = gt >= t05 - 0.5
  ang = np.full(len(gt), np.nan)
  ang[m] = np.degrees(np.concatenate(([0.0], np.cumsum(0.5 * (prc[m][1:] + prc[m][:-1]) * np.diff(gt[m])))))
  ang -= np.interp(t05, gt, ang)
  w_term = (bt >= t05) & (bt <= t_flag + 0.8)
  # approach level: IMU mean over the first half of the last 0.5 m/s (before any terminal feature)
  a_app = float(np.nanmean(m01[(bt >= t05) & (bt <= t05 + 0.5 * (t_flag - t05))]))
  # rebound peak (IMU 0.1 s mean max after t05) and the level it released from (min over the 0.4 s before the peak)
  k = np.nonzero(w_term)[0]
  kp = k[np.nanargmax(m01[k])]
  t_peak, a_peak = float(bt[kp]), float(m01[kp])
  kb = np.nonzero((bt >= t_peak - 0.4) & (bt <= t_peak))[0]
  kmin = kb[np.nanargmin(m01[kb])]
  a_rel, t_rel = float(m01[kmin]), float(bt[kmin])
  # IMU minimum over the terminal window [t05, flag+0.5]
  k2 = np.nonzero((bt >= t05) & (bt <= t_flag + 0.5))[0]
  kmn = k2[np.nanargmin(m01[k2])]
  # 300 ms exact-difference jerk on the 0.1 s IMU mean and on ESP12
  def j300(t, x, a, b):
    q = np.arange(a, b - 0.3, 0.01)
    xi = np.interp(q, t[np.isfinite(x)], x[np.isfinite(x)])
    xj = np.interp(q + 0.3, t[np.isfinite(x)], x[np.isfinite(x)])
    d = (xj - xi) / 0.3
    return float(d.max()), float(d.min())
  jimu = j300(bt, m01, t05, t_flag + 0.8)
  et, ea = S['esp12__t'], S['esp12__LONG_ACCEL']
  e = (et >= t05) & (et <= t_flag + 0.5)
  ke = np.nonzero(e)[0]
  kem = ke[np.argmin(ea[ke])]
  ep = (et >= t05) & (et <= t_flag + 0.8)
  jesp = j300(et, ea, t05, t_flag + 0.8)
  gp = (gt >= t05) & (gt <= t_flag + 0.8)
  kpr = np.nonzero(gp)[0]
  angw = ang[(gt >= t_flag - 0.6) & (gt <= t_flag + 0.8)]
  # wheel pulses: last increment before flag+2 s and counts after the rebound peak
  pt, pc = S['pul__t'], S['pul__count']
  inc = np.nonzero(np.diff(pc) > 0)[0]
  last_inc = [float(pt[j + 1] - t_flag) for j in inc if t_flag - 1.0 <= pt[j + 1] <= t_flag + 2.0]
  d_end = t_flag + 2.0 if t_end is None else min(t_flag + 2.0, t_end)
  pulses_flag_2s = float(np.diff(np.interp([t_flag, d_end], pt, pc))[0])
  pulses_after_peak = float(np.diff(np.interp([t_peak + 0.3, t_end if t_end is not None else t_flag + 3.0], pt, pc))[0])
  rest = (bt >= t_flag + 0.7) & (bt <= hi)
  a_rest = float(np.nanmedian(m01[rest])) if rest.sum() >= 20 else None
  er = (et >= t_flag + 0.7) & (et <= hi)
  e_rest = float(np.median(ea[er])) if er.sum() >= 10 else None
  e_pre = float(np.median(ea[(et >= t05) & (et <= t_flag - 0.35)]))
  e_grab = float(ea[(et >= t_flag - 0.35) & (et <= t_flag + 0.1)].min()) - e_pre
  raw = (bt >= t05) & (bt <= t_flag - 0.35)
  i_pre = float(np.median(bl[raw]))
  r5 = trailing_mean(bt, bl, 0.06)
  i_grab = float(np.nanmin(r5[(bt >= t_flag - 0.35) & (bt <= t_flag + 0.1)])) - i_pre
  gp5 = (gt >= t05) & (gt <= t_flag + 0.5)
  k5 = np.nonzero(gp5)[0]
  ticks = {w: [round(float(pt[j + 1] - t_flag), 2) for j in np.nonzero(np.diff(np.round(S[f'pul__WHL_PUL_{w}'] / 0.5)) % 256 > 0)[0]
               if t_flag < pt[j + 1] <= (t_end if t_end is not None else t_flag + 3.0)] for w in ('FL', 'FR', 'RL', 'RR')}
  w0 = np.nonzero(wv[i_flag:] <= 0.0)[0]
  vt, vv = S['car__t'], S['car__v']
  v01 = np.nonzero((vt > t_flag) & (vv < 0.01))[0]
  mm = (wt >= t05 - 5.0) & (wt <= t05)
  cnt = np.diff(np.interp([t05 - 5.0, t05], pt, pc))[0]
  mpp = float(np.trapezoid(wv[mm], wt[mm]) / cnt) if cnt > 0 else None
  d05 = float(np.diff(np.interp([t05, t_flag + 1.0], pt, pc))[0]) * mpp if mpp else None
  incs = pt[1:][(np.diff(pc) > 0) & (pt[1:] > t05) & (pt[1:] <= t_flag)]
  t_stop = float(incs[-1]) if len(incs) else t_flag
  out = {
    't_stop_from_flag': t_stop - t_flag, 't05_to_stop': t_stop - t05,
    'imu_t05': float(np.interp(t05, bt, m01)), 'imu_mid': float(np.interp(0.5 * (t05 + t_rel), bt, m01)), 'mpp': mpp, 'dist_05_m': d05,
    'a_rest': a_rest, 'arrive_rel': None if a_rest is None else a_rel - a_rest, 'overshoot': None if a_rest is None else a_peak - a_rest,
    'esp_rest': e_rest, 'esp_arrive_rel': None if e_rest is None else float(ea[(et >= t_rel - 0.05) & (et <= t_rel + 0.05)].mean()) - e_rest,
    'esp_overshoot': None if e_rest is None else float(ea[ep].max()) - e_rest,
    'esp_grab': e_grab, 'imu30_grab': i_grab,
    'pr_pos_05': float(np.nanmax(pr05[k5])), 'pr_pos_05_t': float(gt[k5][np.nanargmax(pr05[k5])] - t_flag),
    'pr_neg_05': float(np.nanmin(pr05[k5])), 'pr_neg_05_t': float(gt[k5][np.nanargmin(pr05[k5])] - t_flag),
    'ticks_after_flag': ticks, 'wheel_zero_t': float(wt[i_flag + w0[0]] - t_flag) if len(w0) else None,
    'vego_lt_0p01_t': float(vt[v01[0]] - t_flag) if len(v01) else None, 't_rel_from_t05': t_rel - t05,
    't05': t05, 't05_to_flag': t_flag - t05,
    'at_flag': {'wheel': ls_slope(wt, wv, t_flag - 0.3, t_flag), 'imu': float(np.interp(t_flag, bt, m01)),
                'esp12': at(et, ea, t_flag), 'aEgo': at(S['car__t'], S['car__a'], t_flag)},
    'a_app_imu': a_app,
    'imu_min': float(m01[kmn]), 'imu_min_t': float(bt[kmn] - t_flag),
    'esp_min': float(ea[kem]), 'esp_min_t': float(et[kem] - t_flag),
    'grab_imu': float(m01[kmn]) - a_app,
    'rel_from': a_rel, 'rel_from_t': t_rel - t_flag, 'peak': a_peak, 'peak_t': t_peak - t_flag, 'p2p': a_peak - a_rel,
    'rel_rate': (a_peak - a_rel) / max(t_peak - t_rel, 0.01),
    'esp_max': float(ea[ep].max()), 'esp_p2p': float(ea[ep].max() - ea[(et >= t05) & (et <= t_flag + 0.8)].min()),
    'j300_imu_max': jimu[0], 'j300_imu_min': jimu[1], 'j300_esp_max': jesp[0], 'j300_esp_min': jesp[1],
    'pr_pos': float(np.nanmax(pr05[kpr])), 'pr_pos_t': float(gt[kpr][np.nanargmax(pr05[kpr])] - t_flag),
    'pr_neg': float(np.nanmin(pr05[kpr])), 'pr_neg_t': float(gt[kpr][np.nanargmin(pr05[kpr])] - t_flag),
    'pr_bias': bias, 'pitch_swing_deg': float(np.nanmax(angw) - np.nanmin(angw)),
    'pitch_rise_deg': float(np.nanmax(ang[(gt >= t_flag - 0.6) & (gt <= t_flag + 0.8)]) - np.interp(t_flag - 0.6, gt, ang)),
    'pulse_last_inc_t': last_inc[-1] if last_inc else None, 'pulse_incs_after_flag': [x for x in last_inc if x > 0],
    'pulses_flag_2s': pulses_flag_2s, 'pulses_after_rebound': pulses_after_peak,
  }
  return out
