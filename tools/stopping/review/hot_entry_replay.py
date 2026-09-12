#!/usr/bin/env python3
"""Hot band entry (cycle 54): closed-loop replay of the REAL StoppingService + StopContext on the recorded stopped-lead
approaches. The lead's absolute position comes from the recording (ego displacement + dRel); the ego is simulated
(first-order actuation lag 0.45 s, a creep push +0.25 m/s^2 cut by brake depth); the recorded planner wire drives the
ego until the service takes over; shouldStop / aTarget come from the recording. The plant model under-reads recorded
arrivals by ~30 %: compare arms, not absolutes.

Arms: "today" = the shipped law; "taup" = the two-sided decoupled pursuit constant (a law-experiment wrapper, REJECTED in
cycle 54: it breaks the moving-lead cancellation at the crawler-follow fixed point). The service calls governor_demand
TWICE per frame (live and shadow), so a stateful wrapper must not advance on the shadow call.

Usage: python tools/stopping/review/hot_entry_replay.py [route ...]   (default: routes 2075-2086)"""
import bisect
import glob
import json
import os
import statistics
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from triage_one import read_events

from openpilot.selfdrive.controls.lib import stopping_flags
from openpilot.selfdrive.controls.lib import stopping_service as svc_mod
from openpilot.selfdrive.controls.lib.stop_context import StopContext
from openpilot.selfdrive.controls.lib.stopping_service import (
  GOV_A_C, GOV_A_MAX, GOV_A_UP, GOV_TAU, StoppingService,
)

BASE = os.path.expanduser("~/.route_sync/data/media/0/realdata/")
ORIG = svc_mod.governor_demand
DEFAULT_ROUTES = ["00002075", "00002076", "0000207e", "00002081", "00002082", "00002085", "00002086"]


def make_law(kind, tau_p=1.5):
  """A law-experiment wrapper (stateless): kind "today" or "taup" (the pursuit constant decoupled, two-sided)."""
  def law(v, v_lead, gap, isd):
    g = ORIG(v, v_lead, gap, isd)
    if g is None or kind == "today":
      return g
    a_gov, v_ref, q_ref, d = g
    a_ff = -GOV_A_C * q_ref / max(q_ref + GOV_A_C * GOV_TAU, 1e-6)
    a = min(max(a_ff + (v_ref - v) / tau_p, -GOV_A_MAX), GOV_A_UP)
    return a, v_ref, q_ref, d
  return law


def load(seg):
  rows = []
  t0 = None
  lead = None
  wire = None
  ss = False
  a_tgt = 0.0
  summaries = []
  for m in read_events(BASE + seg + "/rlog.zst"):
    t = m.logMonoTime / 1e9
    if t0 is None:
      t0 = t
    rel = t - t0
    w = m.which()
    if w == "radarState":
      lead_one = m.radarState.leadOne
      lead = (bool(lead_one.status), lead_one.dRel, lead_one.vLead, lead_one.radarTrackId)
    elif w == "carControl":
      wire = m.carControl.actuators.accel
    elif w == "longitudinalPlan":
      ss = m.longitudinalPlan.shouldStop
      a_tgt = m.longitudinalPlan.aTarget
    elif w == "logMessage" and "settle_summary" in m.logMessage:
      try:
        summaries.append((rel, json.loads(m.logMessage).get("msg")))
      except (ValueError, AttributeError):
        pass
    elif w == "carState" and lead is not None and wire is not None:
      rows.append((rel, m.carState.vEgo, m.carState.aEgo, lead, wire, ss, a_tgt))
  return rows, summaries


def run(rows, t_start, t_end, law, push=0.25, lag=0.45):
  stopping_flags.SERVICE_APPROACH_LAW = "governor"
  svc_mod.governor_demand = law
  xs = []
  x = 0.0
  for i, r in enumerate(rows):
    if i:
      x += rows[i - 1][1] * (r[0] - rows[i - 1][0])
    xs.append(x)
  i0 = bisect.bisect_left([r[0] for r in rows], t_start)
  ctx, svc = StopContext(), StoppingService()
  v, a_act, x_sim, cmd = rows[i0][1], rows[i0][2], xs[i0], rows[i0][4]
  out = []
  for i in range(i0, len(rows) - 1):
    t, v_rec, a_rec, lead, w_rec, ss, a_tgt = rows[i]
    if t > t_end:
      break
    step = rows[i + 1][0] - t
    status, d_rel, lv, tid = lead
    gap = xs[i] + d_rel - x_sim if status else None
    # the measured acceleration fed back is the NET one the plant integrates (actuation + creep push); feeding the
    # actuation alone made the coast estimator read the push as drag (review 20260912-103236)
    a_meas = a_act + push * min(max((a_act + 0.70) / 0.35, 0.0), 1.0)
    sig = ctx.update(v_ego=v, a_ego=a_meas, a_cmd=cmd, lead_status=status, lead_v=lv, lead_d_rel=gap,
                     lead_track_id=tid if status else None, standstill=v < 0.02, dt=step)
    r = svc.update(engaged=True, v_ego=v, a_ego=a_meas, a_target=a_tgt, should_stop=ss,
                   dts_planner=max((gap or 30.0) - 4.3, 0.05), planner_min_limit=-3.5, signals=sig, lead_status=status,
                   lead_v=lv, increased_stopped_distance=0.3, dt=step, wire_accel=cmd, a_target_trajectory=a_tgt)
    cmd = r.accel if r.active else w_rec      # not owning: the recorded planner wire (the approach before the takeover)
    a_act += (cmd - a_act) * step / lag
    f = min(max((a_act + 0.70) / 0.35, 0.0), 1.0)
    a_net = a_act + push * f
    v = max(v + a_net * step, 0.0)
    x_sim += v * step
    dbg = r.debug or {}
    out.append({"t": t, "v": v, "a_net": a_net, "gap": gap, "cmd": cmd, "lv": lv,
                "phase": r.phase.name if r.active else "OFF", "a_bar": dbg.get("a_barrier"), "a_phase": dbg.get("a_phase"),
                "v_rec": v_rec, "a_rec": a_rec, "w_rec": w_rec})
    if out[-1]["phase"] in ("HOLD", "RAMP_TO_HOLD") and v <= 0.02:
      break
  svc_mod.governor_demand = ORIG
  return out


def metrics(out):
  own = [o for o in out if o["phase"] in ("APPROACH_GLIDE", "PRE_STOP_EASE")]
  if not own or out[-1]["phase"] not in ("HOLD", "RAMP_TO_HOLD"):
    return None
  t_in = own[0]["t"]
  head = [o for o in out if o["t"] >= t_in and o["v"] > 0.5]
  term = [o for o in out if o["t"] >= t_in and 0.12 < o["v"] <= 0.5]
  bar_bind = sum(1 for o in own if o["a_bar"] is not None and o["a_phase"] is not None
                 and o["a_bar"] < o["a_phase"] - 1e-6 and o["a_bar"] < o["cmd"] + 0.05)
  gaps = [o["gap"] for o in out if o["gap"] is not None]
  return {"t_in": t_in, "v_in": own[0]["v"], "gap_in": own[0]["gap"],
          "peak_head": min(o["a_net"] for o in head) if head else 0.0,
          "peak_cmd_head": min(o["cmd"] for o in head) if head else 0.0,
          "peak_term": min(o["a_net"] for o in term) if term else 0.0,
          "min_gap": min(gaps) if gaps else None, "rest": out[-1]["gap"], "t_stop": out[-1]["t"] - t_in,
          "bar_bind": bar_bind, "v_end": out[-1]["v"]}


def main(routes):
  segs = sorted(d for d in glob.glob(BASE + "0000*--*--*")
                if os.path.basename(d)[:8] in routes and os.path.exists(d + "/rlog.zst"))
  cands = [("today", make_law("today")), ("taup", make_law("taup", tau_p=1.2))]
  results = []
  for sd in segs:
    seg = os.path.basename(sd)
    try:
      rows, summaries = load(seg)
    except (OSError, ValueError):
      continue
    for t_settle, msg in summaries:
      tr = msg.get("gov_trace") or []
      entry = next((r for r in tr if r[0] >= 0.0), None)
      if not entry or len(entry) < 7 or entry[1] < 1.5 or abs(entry[6]) > 0.3:   # stopped-lead entries only
        continue
      t_entry = next((r[0] for r in reversed(rows) if r[0] < t_settle and r[1] >= 2.5), None)
      if t_entry is None or t_settle - t_entry > 15.0:
        continue
      rec_head = min((r[2] for r in rows if t_entry <= r[0] <= t_entry + 3.0 and r[1] > 0.5), default=None)
      line = {"seg": seg, "t": round(t_settle, 1)}
      rec_txt = "None" if rec_head is None else f"{rec_head:.2f}"
      print(f"{seg} t={t_settle:.1f} REC v_in {entry[1]:.2f} gap {entry[2]:.1f} a_gov {entry[4]:.2f} peak_head {rec_txt} rest {msg.get('rest_gap')}")
      for name, law in cands:
        m = metrics(run(rows, t_entry - 1.5, t_settle + 3.0, law))
        line[name] = m
        if m is None:
          print(f"   {name:8} no ownership")
          continue
        head = f"   {name:8} v_in {m['v_in']:.2f} gap_in {m['gap_in']:.2f} | peak_head {m['peak_head']:5.2f} (cmd {m['peak_cmd_head']:5.2f})"
        tail = f"peak_term {m['peak_term']:5.2f} | min_gap {m['min_gap']:.2f} rest {m['rest']:.2f} t_stop {m['t_stop']:.1f}"
        tail += f" bar {m['bar_bind']} v_end {m['v_end']:.2f}"
        print(head + " " + tail)
      results.append(line)
  print("\nSUMMARY (median over stops):")
  for name, _ in cands:
    ms = [ln[name] for ln in results if ln.get(name)]
    if not ms:
      continue
    med = {k: statistics.median(m[k] for m in ms) for k in ("peak_head", "peak_term", "min_gap", "rest", "t_stop")}
    bar_stops = sum(1 for m in ms if m["bar_bind"] > 0)
    short = sum(1 for m in ms if m["min_gap"] < 3.6)
    print(f"  {name:8} n={len(ms)} peak_head {med['peak_head']:5.2f} peak_term {med['peak_term']:5.2f} min_gap {med['min_gap']:.2f} "
          + f"rest {med['rest']:.2f} t_stop {med['t_stop']:.1f} bar_bind_stops {bar_stops} min_gap<3.6 {short}")


if __name__ == "__main__":
  main(sys.argv[1:] or DEFAULT_ROUTES)
