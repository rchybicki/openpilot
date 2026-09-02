#!/usr/bin/env python3
"""HUMAN BASELINE census for ONE rlog: every real stop (from >= 3 m/s to a >= 0.5 s standstill), classified
engaged (openpilot, no brake press on approach) / manual (driver braking, disengaged) / mixed, scored with the
SAME whole-stop metrics for both: felt_term (aEgo jerk, 0.3 s window, terminal v<0.45), felt_appr (same jerk
over the whole approach v<=9), v_at_appr_peak (where the approach jerk peaks), a_wheelstop (aEgo at the last
rolling frame = the nod driver), a_min_term. Emits one JSON line per stop.
Aggregate: tools/stopping/review/human_baseline.py --aggregate <jsonl>   (p50/p90 per class)
Retrospective 2026-09-02 (docs/stopping/retrospective_2026-09-02.md): across routes 00002xxx every one of
these metrics scores engaged stops ~2x smoother than the driver's own stops -- the program's acceptance
metric does not track the driver's perception; calibrate against per-stop ratings before optimising further."""
import capnp, zstandard, sys, json
if len(sys.argv) > 2 and sys.argv[1] == "--aggregate":
    import statistics as st, collections
    rows=[json.loads(l) for l in open(sys.argv[2]) if l.strip()]
    by=collections.defaultdict(list)
    for r in rows: by[r["cls"]].append(r)
    def q(xs,p): xs=sorted(xs); return xs[min(len(xs)-1,int(p*len(xs)))]
    print(f"{'class':8} {'n':>4} | felt_term p50 p90 | <=0.8 | felt_appr p50 p90 | a_wheelstop p50 | a_min_term p50")
    for c in ("manual","engaged","mixed"):
        v=by.get(c,[])
        if not v: continue
        ft=[r["felt_term"] for r in v]; fa=[r["felt_appr"] for r in v]; aw=[r["a_wheelstop"] for r in v]; am=[r["a_min_term"] for r in v]
        print(f"{c:8} {len(v):4d} | {st.median(ft):5.2f} {q(ft,0.9):5.2f} | {sum(x<=0.8 for x in ft)/len(v)*100:4.0f}% | {st.median(fa):5.2f} {q(fa,0.9):5.2f} | {st.median(aw):6.2f} | {st.median(am):6.2f}")
    sys.exit()
capnp.remove_import_hook()
LOG = capnp.load("/Users/radoslawchybicki/Repos/openpilot-rch/cereal/log.capnp")
path = sys.argv[1]
try:
    raw = zstandard.ZstdDecompressor().decompress(open(path,"rb").read(), max_output_size=int(9e8))
except Exception:
    sys.exit()
T,V,A,B = [],[],[],[]
EN = []  # (t, enabled)
t0=None
for ev in LOG.Event.read_multiple_bytes(raw):
    w=ev.which()
    if w=="carState":
        t=ev.logMonoTime*1e-9
        if t0 is None: t0=t
        cs=ev.carState
        T.append(t-t0); V.append(cs.vEgo); A.append(cs.aEgo); B.append(bool(cs.brakePressed))
    elif w=="selfdriveState":
        EN.append((ev.logMonoTime*1e-9-(t0 or ev.logMonoTime*1e-9), bool(ev.selfdriveState.enabled)))
def enabled_at(t):
    e=False
    for (tt,en) in EN:
        if tt<=t: e=en
        else: break
    return e
def jerk_max(k0,k1,win=0.30):
    worst=0.0; j=k0; kw=k0
    for k in range(k0,k1+1):
        while T[k]-T[j]>win: j+=1
        if k>j:
            x=abs(A[k]-A[j])/(T[k]-T[j])
            if x>worst: worst=x; kw=k
    return worst,kw
n=len(T); k=0; out=[]
while k<n:
    if V[k]<0.05:
        j=k
        while j+1<n and V[j+1]<0.05: j+=1
        if T[j]-T[k]>=0.5:
            ts=T[k]
            # approach: last 10 s; require a real stop from >= 3 m/s
            ka=k
            while ka>0 and ts-T[ka]<10.0: ka-=1
            vmax=max(V[ka:k+1])
            if vmax>=3.0:
                k0=k
                while k0>ka and V[k0-1]<0.45: k0-=1   # first sub-0.45 frame of this descent
                # approach band k_ap: v in [0.45, 3.0] contiguous before k0
                kb=k0
                while kb>ka and V[kb-1]<=9.0: kb-=1
                k1=k
                while k1+1<n and T[k1+1]-ts<=0.5: k1+=1
                eng=enabled_at(ts-0.5) and not any(B[kb:k+1])
                man=(not enabled_at(ts-0.5)) and (not enabled_at(ts-2.0)) and any(B[kb:k+1])
                cls="engaged" if eng else ("manual" if man else "mixed")
                ft,kft=jerk_max(k0,k1)
                fa,kfa=jerk_max(kb,max(kb,k0-1)) if k0-1>kb else (0.0,kb)
                kr=k
                while kr>0 and V[kr]<0.10: kr-=1
                out.append({"seg":path.split("/realdata/")[-1].split("/")[0],"t":round(ts,1),"cls":cls,"v_appr":round(vmax,1),
                            "felt_term":round(ft,3),"felt_appr":round(fa,3),"v_at_appr_peak":round(V[kfa],2),
                            "a_wheelstop":round(A[kr],3),"a_min_term":round(min(A[k0:k+1]),3)})
        k=j+1
    else: k+=1
for o in out: print(json.dumps(o))
