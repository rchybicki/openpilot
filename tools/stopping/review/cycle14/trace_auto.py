#!/usr/bin/env python3
"""Cycle-14 controller-gap trace: the two AUTO stops in 00001f4c seg 56 (t_phys 3392.743 / 3417.704).

100 Hz trace (wire vs planner vs actual decel vs v) + REAL StopContext+StoppingService replay
driven from recorded carState (per the replay rule: never hand-set wheel_stop / hold v).
Route time base = 56*60 + (mono - t0(seg56)); matches honest_metrics.py convention.
"""
import sys
import json
import numpy as np
import capnp
import zstandard

REPO = "/Users/radoslawchybicki/Repos/openpilot-rch"
sys.path.insert(0, REPO)
capnp.remove_import_hook()
LOG = capnp.load(f"{REPO}/cereal/log.capnp")
BASE = "/Users/radoslawchybicki/.route_sync/data/media/0/realdata"
ROUTE = "00001f4c--e8e297ccb6"

from openpilot.selfdrive.controls.lib.stopping_service import StoppingService, Phase
from openpilot.selfdrive.controls.lib.stop_context import StopContext

def load_seg(idx):
  raw = zstandard.ZstdDecompressor().decompress(
    open(f"{BASE}/{ROUTE}--{idx}/rlog.zst", "rb").read(), max_output_size=int(9e8))
  return list(LOG.Event.read_multiple_bytes(raw))

evs56 = load_seg(56)
evs57 = load_seg(57)
t0 = evs56[0].logMonoTime * 1e-9

cs, cc, co, lp, rs, fp, ax = [], [], [], [], [], [], []
for ev in evs56 + evs57:
  t = ev.logMonoTime * 1e-9 - t0
  w = ev.which()
  if w == "carState":
    c = ev.carState
    cs.append((t, c.vEgo, c.aEgo, float(c.standstill), float(c.brakePressed), float(c.gasPressed)))
  elif w == "carControl":
    c = ev.carControl
    cc.append((t, float(c.longActive), c.actuators.accel, int(c.actuators.longControlState.raw)))
  elif w == "carOutput":
    co.append((t, ev.carOutput.actuatorsOutput.accel))
  elif w == "longitudinalPlan":
    p = ev.longitudinalPlan
    att = p.aTargetTrajectory if p.aTargetTrajectoryValid else float("nan")
    lp.append((t, p.aTarget, att, float(p.shouldStop), p.distanceToStopTarget))
  elif w == "radarState":
    l = ev.radarState.leadOne
    rs.append((t, float(l.status), l.dRel, l.vLead))
  elif w == "frogpilotPlan":
    fp.append((t, ev.frogpilotPlan.increasedStoppedDistance))
  elif w == "accelerometer":
    a = ev.accelerometer
    if a.which() == "acceleration" and len(a.acceleration.v) >= 3:
      ax.append((t, -a.acceleration.v[2]))

cs = np.array(cs); cc = np.array(cc); co = np.array(co)
lp = np.array(lp); rs = np.array(rs); fp = np.array(fp); ax = np.array(ax)

def zoh(tq, tab, col):
  """zero-order hold of tab[:,col] at times tq (latest message <= t, like controlsd's sm)."""
  i = np.searchsorted(tab[:, 0], tq, side="right") - 1
  i = np.clip(i, 0, len(tab) - 1)
  return tab[i, col]

def smooth(t, y, half=0.075):
  j0 = np.searchsorted(t, t - half); j1 = np.searchsorted(t, t + half)
  c = np.cumsum(np.insert(y, 0, 0.0))
  return (c[j1] - c[j0]) / np.maximum(j1 - j0, 1)

ta = ax[:, 0]
afs = smooth(ta, ax[:, 1])

STOPS = [("56a", 3392.743), ("56b", 3417.704)]

# ---------- replay: REAL StopContext + StoppingService over the whole double-stop window ----------
t_run0, t_run1 = 3380.0, 3424.0
m = (cs[:, 0] >= t_run0) & (cs[:, 0] <= t_run1)
tq = cs[m, 0]
v_q = cs[m, 1]; aego_q = cs[m, 2]; stand_q = cs[m, 3]
wire_q = zoh(tq, co, 1)
longact_q = zoh(tq, cc, 1); lcs_q = zoh(tq, cc, 3)
atgt_q = zoh(tq, lp, 1); atraj_q = zoh(tq, lp, 2); sstop_q = zoh(tq, lp, 3); dts_q = zoh(tq, lp, 4)
lead_st_q = zoh(tq, rs, 1); drel_q = zoh(tq, rs, 2); vlead_q = zoh(tq, rs, 3)
isd_q = zoh(tq, fp, 1) if len(fp) else np.zeros_like(tq)

ctx = StopContext()
svc = StoppingService()
rep = []
prev_t = tq[0] - 0.01
LCS_STOPPING = 3  # cereal LongCtrlState: off=0, pid=1, stopping=2, starting=3 -- verify below
# pull actual enum values
LCS_names = {int(v.raw): str(v) for v in []}
for i in range(len(tq)):
  t = tq[i]; dt = max(t - prev_t, 1e-3); prev_t = t
  active = longact_q[i] > 0.5
  stopping = lcs_q[i] == 2
  run = active and (v_q[i] < 2.5 or stopping)
  wire = float(wire_q[i])
  if not run:
    ctx.reset(); svc.reset()
    rep.append((t, 0.0, 0, {}, 0.0))
    continue
  sig = ctx.update(v_ego=float(v_q[i]), a_ego=float(aego_q[i]), a_cmd=wire,
                   lead_status=bool(lead_st_q[i] > 0.5), lead_v=float(vlead_q[i]),
                   lead_d_rel=float(drel_q[i]) if lead_st_q[i] > 0.5 else None,
                   standstill=bool(stand_q[i] > 0.5), dt=dt)
  dts = float(dts_q[i]) if dts_q[i] >= 0.0 else None
  atraj = float(atraj_q[i]) if np.isfinite(atraj_q[i]) else None
  res = svc.update(engaged=True, v_ego=float(v_q[i]), a_ego=float(aego_q[i]),
                   a_target=float(atgt_q[i]), should_stop=bool(sstop_q[i] > 0.5),
                   dts_planner=dts, planner_min_limit=-3.5, signals=sig,
                   lead_status=bool(lead_st_q[i] > 0.5), lead_v=float(vlead_q[i]),
                   increased_stopped_distance=float(isd_q[i]), dt=dt, wire_accel=wire,
                   a_target_trajectory=atraj)
  rep.append((t, res.accel, int(res.phase), dict(res.debug), sig.a_coast))

# replay fidelity: service accel vs recorded wire on frames where service active
acc = np.array([r[1] for r in rep]); ph = np.array([r[2] for r in rep])
act_m = ph > 0
err = acc[act_m] - wire_q[act_m] if act_m.any() else np.array([0.0])
print(f"REPLAY FIDELITY: n_active={act_m.sum()} median|err|={np.median(np.abs(err)):.3f} p95|err|={np.percentile(np.abs(err),95):.3f} max|err|={np.abs(err).max():.3f}")

# ---------- print traces ----------
for name, t_phys in STOPS:
  base_m = (ta >= t_phys + 0.8) & (ta <= t_phys + 2.5)
  base = float(np.median(ax[base_m, 1]))
  print(f"\n===== STOP {name}  t_phys={t_phys}  raw-accel parked baseline={base:+.3f} =====")
  print("t-tp | v | aEgo | a_raw | wire | aTgt | aTraj | dts | sstp | dRel | vLead | phase | a_phase | a_kin | a_plan | a_mon | d_rem | d_rest | a_coast | bind")
  for dtq in np.arange(-6.0, 3.01, 0.1):
    t = t_phys + dtq
    j = np.searchsorted(tq, t)
    if j >= len(tq):
      break
    j = min(j, len(tq) - 1)
    r = rep[j]; d = r[3]
    araw = float(np.interp(t, ta, afs)) - base
    pv = lambda x, f="{:+.2f}": (f.format(x) if (x is not None and np.isfinite(x)) else "  -  ")
    print(f"{dtq:+5.1f} | {v_q[j]:4.2f} | {aego_q[j]:+5.2f} | {araw:+5.2f} | {wire_q[j]:+5.2f} | "
          f"{atgt_q[j]:+5.2f} | {pv(atraj_q[j])} | {dts_q[j]:5.2f} | {int(sstop_q[j])} | "
          f"{drel_q[j]:5.2f} | {vlead_q[j]:+5.2f} | {Phase(r[2]).name[:9]:9s} | "
          f"{pv(d.get('a_phase'))} | {pv(d.get('a_kin'))} | {pv(d.get('a_plan'))} | {pv(d.get('a_monitor'))} | "
          f"{pv(d.get('d_rem'),'{:+.2f}')} | {pv(d.get('d_rest_eff'),'{:.2f}')} | {r[4]:+.2f} | {int(bool(d.get('safety_binding',False)))}")

# dump replay for further analysis
out = []
for i, r in enumerate(rep):
  d = r[3]
  out.append({"t": round(r[0], 3), "svc": round(r[1], 3), "phase": int(r[2]),
              "wire": round(float(wire_q[i]), 3), "v": round(float(v_q[i]), 3),
              "a_phase": d.get("a_phase"), "a_kin": d.get("a_kin"), "a_plan": d.get("a_plan"),
              "a_mon": d.get("a_monitor"), "d_rem": d.get("d_rem"), "d_rest": d.get("d_rest_eff"),
              "d_gap": d.get("d_gap"), "a_coast": r[4],
              "atgt": round(float(atgt_q[i]), 3), "atraj": (round(float(atraj_q[i]), 3) if np.isfinite(atraj_q[i]) else None),
              "dts": round(float(dts_q[i]), 3), "sstop": int(sstop_q[i]),
              "wheel_stop": d.get("wheel_stop")})
json.dump(out, open("/tmp/sc14/replay_56.json", "w"))
print("\nwrote /tmp/sc14/replay_56.json")
