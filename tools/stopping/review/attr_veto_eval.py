#!/usr/bin/env python3
"""Attributed-safety LIVE evaluation on recorded rlogs (program doc 2026-09-05 revised gate): for every UNEXPLAINED
plan-bound ring row of every unique settle_summary, (1) MATERIALITY per stop: sustained releases (> 0.10 m/s^2 for
>= 0.30 s = 6 consecutive ring rows at the 20 Hz ring rate) and released impulse sum(max(candidate - a_plan - 0.10,
0))*0.05; (2) LATE-RESCUE veto: in the 2.0 s after the row, does a_kin / a_bar / a_pred / aTargetTrajectory
(recomputed from the rlog with the REAL pure functions) become >= 0.10 deeper than the released candidate, or FCW,
or driver braking, or lead loss? usage: attr_veto_eval.py <rlog globs>"""
import sys, os, glob, json, math, collections
import capnp, zstandard
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, "/Users/radoslawchybicki/Repos/openpilot-rch")
from stop_index import settle_events
from openpilot.selfdrive.controls.lib.stopping_service import barrier_demand, predictive_lead_demand
capnp.remove_import_hook()
LOG = capnp.load("/Users/radoslawchybicki/Repos/openpilot-rch/cereal/log.capnp")
D_HARD, EPS = 2.0, 0.30

def frames(path):
  raw = zstandard.ZstdDecompressor().decompress(open(path, "rb").read(), max_output_size=int(9e8))
  t0 = None; v = a = 0.0; lead = (False, 0.0, 0.0, 0.0, -1); traj = None; fcw = False; brake = False; out = []
  for ev in LOG.Event.read_multiple_bytes(raw):
    t = ev.logMonoTime * 1e-9
    if t0 is None:
      t0 = t
    w = ev.which()
    if w == "carState":
      cs = ev.carState; v, a, brake = cs.vEgo, cs.aEgo, bool(cs.brakePressed)
      out.append((t - t0, v, a, brake, lead, traj, fcw))
    elif w == "radarState":
      l = ev.radarState.leadOne; lead = (bool(l.status), float(l.dRel), float(l.vLead), float(l.aLeadK), int(l.radarTrackId))
    elif w == "longitudinalPlan":
      lp = ev.longitudinalPlan; traj = float(lp.aTargetTrajectory) if lp.aTargetTrajectoryValid else None; fcw = bool(lp.fcw)
  return out

paths = []
for arg in sys.argv[1:]:
  paths += glob.glob(arg) if any(c in arg for c in "*?[") else [arg]
seen = set(); stops = []; viol = collections.Counter(); checked = 0; rows_unexp = 0
for p in sorted(paths):
  seg = p.split("/realdata/")[-1].split("/")[0]
  evs = [e for e in settle_events(p) if e.get("attr_frames")]
  if not evs:
    continue
  fr = None
  for e in evs:
    key = (seg, round(e["t"], 1))
    if key in seen:
      continue
    seen.add(key)
    start = e["t"] - (e["frames"] or 0) / 100.0
    ring = e.get("attr_ring") or []
    unexp = [r for r in ring if r[8] is not None and r[3] is not None and (r[8] - r[3]) > 0.10]
    rows_unexp += len(unexp)
    # materiality
    impulse = sum(max(r[8] - r[3] - 0.10, 0.0) for r in ring if r[8] is not None and r[3] is not None) * 0.05
    run = best = 0
    for r in ring:
      ok = r[8] is not None and r[3] is not None and (r[8] - r[3]) > 0.10
      run = run + 1 if ok else 0; best = max(best, run)
    stops.append({"seg": seg, "t": round(e["t"], 1), "rest": e.get("rest_gap"), "unexp_rows": len(unexp), "sustained": best >= 6, "impulse": round(impulse, 3)})
    if not unexp:
      continue
    if fr is None:
      fr = frames(p)
    for r in unexp:
      t_abs = start + r[0]; cand = r[8]; checked += 1
      win = [f for f in fr if t_abs < f[0] <= t_abs + 2.0]
      tid0 = next((f[4][4] for f in fr if f[0] >= t_abs), None)
      for (t, v, a, brake, lead, traj, fcw) in win:
        status, d, lv, alk, tid = lead
        if not status or tid != tid0:
          viol["lead_lost_or_replaced"] += 1; break
        if brake:
          viol["driver_brake"] += 1; break
        if fcw:
          viol["fcw"] += 1; break
        vc = max(v - max(lv, 0.0), 0.0)
        a_kin = -(vc * vc) / (2.0 * max(d - D_HARD, EPS))
        a_bar = barrier_demand(v, lv, d)
        a_pred = predictive_lead_demand(v, lv, d, a, D_HARD, eps=EPS)
        deeper = [("a_kin", a_kin), ("a_bar", a_bar), ("a_pred", a_pred), ("traj", traj)]
        hit = [n for n, x in deeper if x is not None and math.isfinite(x) and x <= cand - 0.10]
        if hit:
          viol["rescue:" + "+".join(hit)] += 1; break
        if alk < -0.3:
          viol["lead_braking"] += 1; break
n = len(stops)
print(f"unique settles with attr data: {n}; unexplained ring rows: {rows_unexp}; rows checked for late rescue: {checked}")
print(f"MATERIALITY: stops with any unexplained release {sum(s['unexp_rows'] > 0 for s in stops)}/{n}; sustained (>= 0.30 s) {sum(s['sustained'] for s in stops)}/{n}; "
      f"impulse >= 0.05 m/s: {sum(s['impulse'] >= 0.05 for s in stops)}/{n}; impulse p50 {sorted(s['impulse'] for s in stops)[n // 2]:.3f} max {max(s['impulse'] for s in stops):.3f}")
print(f"LATE-RESCUE VETO violations (per released row, first cause in the next 2 s): {dict(viol)}")
for s in sorted(stops, key=lambda s: -s["impulse"])[:8]:
  print("  ", s)
