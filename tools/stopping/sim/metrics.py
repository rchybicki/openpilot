# ruff: noqa: RUF100, ISC002, E501, C420, F401, UP034  (copied engine code kept verbatim; README.md)
"""Per-run row metrics (copied unchanged from the cyc_1003r validate.py and the cyc_1004 cl3.py runners):
new_extra (release / re-grab, felt03, hold and launch times), full_metrics (pumps from the 3 m/s crossing, entry rise),
compact (the 50 Hz trace window around the stop that the gate analyzers read)."""
import numpy as np

from openpilot.tools.stopping.sim import harness as H
from openpilot.tools.stopping.sim import rharness as R


def _flat(m):
  return {k: v for k, v in (m or {}).items() if not isinstance(v, list)} | {k: (m or {}).get(k) for k in ('aim_bites', 'plan_steps')}


def span03(t, a, lo, hi):
  """0.3 s-span felt (the census feltloc.json 'robust' value): max |a(q + 0.3) - a(q)| / 0.3 over q in [lo, hi - 0.3] on a 10 ms
  grid, linear interpolation. The stop_index pair maximum ('felt') peaks on sub-0.1 s pairs at the window start (artifact)."""
  ok = np.isfinite(a)
  if hi - lo < 0.3 or ok.sum() < 2:
    return None, None
  q = np.arange(lo, hi - 0.3 + 1e-9, 0.01)
  d = np.abs(np.interp(q + 0.3, t[ok], a[ok]) - np.interp(q, t[ok], a[ok])) / 0.3
  k = int(np.argmax(d))
  return float(d[k]), float(q[k])


def new_extra(tr, m, lo):
  """NEW-only features of one run (closed: plant; log: the logged wire and motion) from lo (the closed takeover; the log rows use
  the same 'auto' start) to the stop, plus the hold:
  - release / re-grab: approach minimum of the 5-frame median wire (drops the one-frame +0.70 'starting' swap), the release peak
    after it, the deepest wire after that peak (regrab = re-grab wire - peak);
  - felt03 / felt03_appr: 0.3 s-span felt on aEgo (logged aEgo / plant observation): terminal window [last v >= 0.45, stop + 0.4] and
    approach window [max(stop - 12, lo + 0.5), last v >= 0.45];
  - hold: t_go (first wire > 0 after the stop: the controller's launch decision), t_launch (first true speed > 0.3 m/s after the
    stop + 0.5 s), launch_by_driver (a driver input at or before the launch), hold wire min/max, gap at the stop / at the launch,
    lead_creep (gap at launch - gap at stop), vl_hold_max (lead truth speed);
  - v_min / t_vmin over [lo, stop or window end] (rolling stops);
  - service entry: gov_entry (deepest governor demand d_a_gov in the 0.6 s after), gov_vref_entry, coast_entry (d_a_coast),
    pre_entry_ease (wire before the entry minus its minimum in the 0.6 s before); n_plant_off (closed: brake-off frames)."""
  t, v = tr['t'], tr['v_true']
  lo = max(lo, m.get('t_lo') or lo)
  k_lo = int(np.searchsorted(t, lo))
  ts = m.get('t_stop')
  act = tr['active'].astype(bool)
  drv = (~act) | tr['gas'].astype(bool) | tr['brake'].astype(bool)
  k_end = next((k for k in np.flatnonzero(drv) if k >= k_lo), len(t))
  k_s = int(np.searchsorted(t, ts)) if ts is not None else k_end
  out = {}
  if k_s - k_lo > 5:
    kv = k_lo + int(np.argmin(v[k_lo:k_s]))
    out.update(v_min=float(v[kv]), t_vmin=float(t[kv]))
  if ts is None or k_s - k_lo <= 5:
    return out
  wire = np.median(np.lib.stride_tricks.sliding_window_view(np.pad(tr['wire'], 2, mode='edge'), 5), axis=1)
  k_min = k_lo + int(np.argmin(wire[k_lo:k_s]))
  k_pk = k_min + int(np.argmax(wire[k_min:k_s + 1]))
  k_rg = k_pk + int(np.argmin(wire[k_pk:k_s + 1]))
  out.update(t_min=float(t[k_min]), min_wire5=float(wire[k_min]), rel_peak=float(wire[k_pk]), t_rel_peak=float(t[k_pk]),
             regrab=float(wire[k_rg] - wire[k_pk]), t_regrab=float(t[k_rg]), regrab_wire=float(wire[k_rg]), v_regrab=float(v[k_rg]))
  te = m.get('t_entry')
  if te is not None:   # service entry: governor demand (deepest in the 0.6 s after), its reference speed, the coast compensation,
    ke = int(np.searchsorted(t, te))   # and the wire ease in the 0.6 s before the entry
    k6 = int(np.searchsorted(t, te + 0.6))
    gov = tr['d_a_gov'][ke:max(k6, ke + 1)]
    out.update(gov_entry=float(np.nanmin(gov)) if np.isfinite(gov).any() else None,
               gov_vref_entry=float(tr['d_gov_v_ref'][ke]) if np.isfinite(tr['d_gov_v_ref'][ke]) else None,
               coast_entry=float(tr['d_a_coast'][ke]) if np.isfinite(tr['d_a_coast'][ke]) else None,
               pre_entry_ease=float(tr['wire'][ke - 1] - np.min(tr['wire'][max(ke - 60, k_lo):ke])) if ke > k_lo else None)
  off = tr['plant_off'][k_lo:k_s]
  out['n_plant_off'] = int(np.nansum(off)) if np.isfinite(off).any() else None   # closed only: plant brake-off frames (creep push)
  above = np.flatnonzero((t < ts) & (v >= 0.45))
  term_lo = float(t[above[-1]]) if len(above) else ts - 1.0
  out['felt03'], out['t_felt03'] = span03(t, tr['a_ego'], term_lo, ts + 0.4)   # the metrics' felt window (census: to t_stop = wheel stop + ~0.4)
  out['felt03_appr'], out['t_felt03_appr'] = span03(t, tr['a_ego'], max(ts - 12.0, lo + 0.5), term_lo)
  go = np.flatnonzero((t > ts) & (tr['wire'] > 0.0))
  out['t_go'] = float(t[go[0]]) if len(go) else None
  la = np.flatnonzero((t > ts + 0.5) & (v > 0.3))
  out['t_launch'] = float(t[la[0]]) if len(la) else None
  k_l = int(la[0]) if len(la) else len(t) - 1
  dr = np.flatnonzero(drv & (t > ts))
  out['t_drv_after'] = float(t[dr[0]]) if len(dr) else None
  out['launch_by_driver'] = bool(len(dr) and t[dr[0]] <= t[k_l] + 0.05)
  g = tr['gap']
  k_h = int(go[0]) if len(go) else k_l   # the hold: stop to the launch decision
  if k_h > k_s:
    hold = slice(k_s, k_h)
    fin = np.isfinite(g[k_l]) and np.isfinite(g[k_s])
    out.update(hold_wire_min=float(np.min(wire[hold])), hold_wire_max=float(np.max(wire[hold])), gap_stop=float(g[k_s]) if np.isfinite(g[k_s]) else None,
               gap_launch=float(g[k_l]) if np.isfinite(g[k_l]) else None, lead_creep=float(g[k_l] - g[k_s]) if fin else None,
               gap_go=float(g[k_h]) if np.isfinite(g[k_h]) else None,
               vl_hold_max=float(np.nanmax(tr['vl_true'][hold])) if np.isfinite(tr['vl_true'][hold]).any() else None)
  return out


def pumps(t, s, k0, k1, h=0.08):
  """aimnoL vn.pumps (copied): s = decel magnitude; strict pumps = dip -> bite -> release with hysteresis h in [k0, k1)."""
  if k1 - k0 < 3:
    return 0, 0.0, 0, 0.0, []
  tt, ss = t[k0:k1], s[k0:k1]
  z = R.zigzag(tt, ss, h)
  ev = []
  for i, p in enumerate(z):
    if p[2] != 'peak':
      continue
    kp = int(np.searchsorted(tt, p[0]))
    rise = p[1] - (z[i - 1][1] if i > 0 else float(np.min(ss[:kp + 1])))
    drop = p[1] - (z[i + 1][1] if i + 1 < len(z) else float(np.min(ss[kp:])))
    ev.append((round(p[0], 2), round(p[1], 3), round(rise, 3), round(drop, 3), i > 0))
  strict = [e for e in ev if e[4]]
  br = [e for e in ev if e[2] >= h]
  return (len(strict), float(max((min(e[2], e[3]) for e in strict), default=0.0)), len(br),
          float(max((min(e[2], e[3]) for e in br), default=0.0)), ev)


def full_metrics(tr, m):
  out = {}
  ts = m.get('t_stop')
  if ts is None or m.get('t_lo') is None:
    return out
  t, v = tr['t'], tr['v_true']
  k_lo = int(np.searchsorted(t, m['t_lo']))
  k_st = int(np.searchsorted(t, ts))
  above = np.flatnonzero(v[:k_st] >= 3.0)
  k0 = max(k_lo, int(above[-1]) if len(above) else k_lo)
  out['t3'], out['v3'] = float(t[k0]), float(v[k0])
  a01 = H._trailing_mean(t, np.nan_to_num(tr['a_real']), 0.1)
  for key, sig in (('w', -tr['wire']), ('a', -a01)):
    n, amp, nb, bamp, ev = pumps(t, sig, k0, k_st + 1)
    out['pump_' + key], out['pamp_' + key], out['br_' + key], out['bamp_' + key], out['pev_' + key] = n, amp, nb, bamp, ev
  out['minw3'] = float(np.min(tr['wire'][k0:k_st + 1]))
  out['mina3'] = float(np.min(a01[k0:k_st + 1]))
  # creep exposure (the 2nd-gear push column) from the takeover to the wheel stop
  if 'creep' in tr:
    cr = np.nan_to_num(tr['creep'][k_lo:k_st + 1])
    out['creep_s'] = float(np.sum(cr > 0.02) * H.DT)
    out['creep_max'] = float(cr.max()) if len(cr) else 0.0
  # entry: the last service entry -> deepening step in the 0.6 s after (bite) and release step (rise of the wire) in the 0.6 s after
  sa = tr['svc_active'].astype(bool)
  ent = [k for k in range(max(k_lo, 1), k_st) if sa[k] and not sa[k - 1]]
  if ent:
    ke = ent[-1]
    k6 = int(np.searchsorted(t, t[ke] + 0.6))
    w = tr['wire']
    out['entry_rise'] = float(np.max(w[ke:max(k6, ke + 1)]) - w[ke - 1])
  # descent capture (wire at 0.5 m/s on the way down) and wire at 1.3 m/s
  for vv, key in ((0.5, 'w05'), (1.3, 'w13')):
    kk = np.flatnonzero((v[:k_st] >= vv))
    if len(kk):
      out[key] = float(tr['wire'][kk[-1]])
  return out


def compact(tr, m, lo_pad=45.0, hi_pad=6.0):
  ts = m.get('t_stop') or tr['t'][-1]
  t = tr['t']
  idx = np.flatnonzero((t >= ts - lo_pad) & (t <= ts + hi_pad) & (np.arange(len(t)) % 2 == 0))   # every other frame of the trace: one 50 Hz grid for all arms
  keys = ('t', 'v_true', 'wire', 'a_real', 'a_ego', 'gap', 'a_target', 'aim_floor', 'svc_active', 'phase', 'plant_off', 'lead_v', 'creep',
          'gear', 'd_a_coast', 'd_a_plan_trajectory', 'd_a_phase', 'd_a_plan', 'ff_added', 'd_safety_binding', 'line_floor',
          'a_target_log', 'd_a_gov', 'd_a_kin', 'd_a_barrier', 'lcs', 'gap_meas')
  out = {}
  for k in keys:
    if k in tr:
      a = np.asarray(tr[k])[idx]
      out[k] = a.astype(np.float32) if a.dtype.kind in 'fiub' else a.astype(str)
  return out
