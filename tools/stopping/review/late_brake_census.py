#!/usr/bin/env python3
"""LATE-BRAKE census for ONE rlog (user feedback 2026-09-04: "brakes gently, then unnecessarily harshly to do the
stop"). For every ENGAGED stop from >= 3 m/s: the decel profile of the approach split by SPEED band, the late peak,
and who owned the wire when the peak happened. Emits one JSON line per stop.
  a_early : mean aEgo while v in [v_appr*0.6, v_appr*0.9]   (the "gentle" part)
  a_late  : min (deepest) aEgo while v in [0.45, 3.0]        (the "harsh" part, before the terminal window)
  v_at_late_peak, wire_at_late_peak, owner: 'service' (v<2.5) or 'planner'
  late_ratio = a_late / a_early  (>= 2 = the pattern; a smooth stop tapers, ratio <= 1)
Aggregate: late_brake_census.py --aggregate <jsonl>"""
import capnp, zstandard, sys, json, os
if len(sys.argv) > 2 and sys.argv[1] == "--aggregate":
  import statistics as st
  rows = [json.loads(l) for l in open(sys.argv[2]) if l.strip()]
  rows = [r for r in rows if r.get("a_early") is not None and r["a_early"] < -0.15]
  ratios = [r["late_ratio"] for r in rows]
  print(f"engaged stops n={len(rows)} late_ratio p50={st.median(ratios):.2f} p90={sorted(ratios)[int(0.9*len(ratios))-1]:.2f}  "
        f"share ratio>=2: {sum(x >= 2 for x in ratios)/len(rows)*100:.0f}%  share a_late<=-1.2: {sum(r['a_late'] <= -1.2 for r in rows)/len(rows)*100:.0f}%")
  for owner in ("planner", "service"):
    v = [r for r in rows if r["owner"] == owner]
    if v:
      print(f"  late peak owned by {owner:8}: n={len(v)} a_late p50={st.median([r['a_late'] for r in v]):.2f} v_at_peak p50={st.median([r['v_at_late_peak'] for r in v]):.2f}")
  worst = sorted(rows, key=lambda r: r["late_ratio"], reverse=True)[:8]
  for r in worst:
    print("   ", {k: r[k] for k in ("seg", "t", "v_appr", "a_early", "a_late", "v_at_late_peak", "owner", "late_ratio")})
  sys.exit()
capnp.remove_import_hook()
LOG = capnp.load("/Users/radoslawchybicki/Repos/openpilot-rch/cereal/log.capnp")
path = sys.argv[1]
try:
  raw = zstandard.ZstdDecompressor().decompress(open(path, "rb").read(), max_output_size=int(9e8))
except Exception:
  sys.exit()
T, V, A, B, W = [], [], [], [], []
EN = []
t0 = None
wire = 0.0
for ev in LOG.Event.read_multiple_bytes(raw):
  w = ev.which()
  if w == "carOutput":
    wire = ev.carOutput.actuatorsOutput.accel
  elif w == "carState":
    t = ev.logMonoTime * 1e-9
    if t0 is None:
      t0 = t
    cs = ev.carState
    T.append(t - t0); V.append(cs.vEgo); A.append(cs.aEgo); B.append(bool(cs.brakePressed)); W.append(wire)
  elif w == "selfdriveState":
    EN.append((ev.logMonoTime * 1e-9 - (t0 or 0), bool(ev.selfdriveState.enabled)))
def enabled_at(t):
  e = False
  for (tt, en) in EN:
    if tt <= t:
      e = en
    else:
      break
  return e
n = len(T); k = 0
while k < n:
  if V[k] < 0.05:
    j = k
    while j + 1 < n and V[j + 1] < 0.05:
      j += 1
    if T[j] - T[k] >= 0.5:
      ts = T[k]; ka = k
      while ka > 0 and ts - T[ka] < 15.0:
        ka -= 1
      v_appr = max(V[ka:k + 1])
      if v_appr >= 3.0 and enabled_at(ts - 0.5) and not any(B[ka:k + 1]):
        kb = k
        while kb > ka and V[kb - 1] <= v_appr:
          kb -= 1
        early = [A[i] for i in range(kb, k + 1) if 0.6 * v_appr <= V[i] <= 0.9 * v_appr]
        late = [(A[i], V[i], W[i]) for i in range(kb, k + 1) if 0.45 <= V[i] <= 3.0]
        if len(early) >= 5 and len(late) >= 5:
          a_early = sum(early) / len(early)
          a_late, v_pk, w_pk = min(late, key=lambda x: x[0])
          print(json.dumps({"seg": path.split("/realdata/")[-1].split("/")[0], "t": round(ts, 1), "v_appr": round(v_appr, 1),
                            "a_early": round(a_early, 2), "a_late": round(a_late, 2), "v_at_late_peak": round(v_pk, 2),
                            "wire_at_late_peak": round(w_pk, 2), "owner": "service" if v_pk < 2.5 else "planner",
                            "late_ratio": round(a_late / a_early, 2) if a_early < -0.05 else None}))
    k = j + 1
  else:
    k += 1
