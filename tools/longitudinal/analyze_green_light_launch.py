#!/usr/bin/env python3
"""Green-light launch analysis: how does the engaged car pull away behind a slowly accelerating lead?

Usage (repo root, venv active): python tools/longitudinal/analyze_green_light_launch.py <route-id> [<route-id> ...]

For every launch (ego at standstill while engaged with a lead, then moving) collect a 20 Hz trace of
ego speed/accel, lead state, e2e model accel, planner aTarget, actuator accel and gas override, and
reconstruct the experimental free-road boost gates from selfdrive/controls/lib/longitudinal_planner.py.
"""
from __future__ import annotations

import json
import sys
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path


from openpilot.tools.stopping.analyze_stopping_behavior import read_events
from openpilot.selfdrive.controls.lib import longitudinal_planner as lp
from openpilot.selfdrive.controls.lib import longcontrol as lc

ROOT = Path.home() / ".route_sync/data/media/0/realdata"  # shared route cache, see docs/route_refresh_process.md


class Lead:
  def __init__(self, d):
    self.__dict__.update(d)


def segment_samples(seg_dir: Path):
  rlog = seg_dir / "rlog.zst"
  if not rlog.exists():
    rlog = seg_dir / "rlog"
  if not rlog.exists():
    return seg_dir.name, []
  cs = None
  lead = None
  e2e = None
  ss = None
  fpcs = None
  fpp = None
  cc = None
  co = None
  out = []
  for ev in read_events(rlog):
    w = ev.which()
    if w == "carState":
      cs = ev.carState
    elif w == "radarState":
      l1 = ev.radarState.leadOne
      lead = dict(status=bool(l1.status), dRel=float(l1.dRel), vRel=float(l1.vRel), vLead=float(l1.vLead),
                  aLeadK=float(l1.aLeadK), modelProb=float(l1.modelProb), radarTrackId=int(l1.radarTrackId))
    elif w == "modelV2":
      e2e = float(ev.modelV2.action.desiredAcceleration)
    elif w == "selfdriveState":
      s = ev.selfdriveState
      ss = dict(enabled=bool(s.enabled), active=bool(s.active), experimentalMode=bool(s.experimentalMode), personality=s.personality.raw)
    elif w == "frogpilotCarState":
      fpcs = dict(forceCoast=bool(ev.frogpilotCarState.forceCoast))
    elif w == "frogpilotPlan":
      fpp = dict(experimentalMode=bool(ev.frogpilotPlan.experimentalMode), tFollow=float(ev.frogpilotPlan.tFollow),
                 maxA=float(ev.frogpilotPlan.maxAcceleration))
    elif w == "carOutput":
      co = float(ev.carOutput.actuatorsOutput.accel)
    elif w == "carControl":
      cc = float(ev.carControl.actuators.accel)
    elif w == "longitudinalPlan":
      if cs is None or lead is None or ss is None:
        continue
      p = ev.longitudinalPlan
      out.append(dict(
        t=ev.logMonoTime / 1e9,
        vEgo=float(cs.vEgo), aEgo=float(cs.aEgo), gas=bool(cs.gasPressed), brake=bool(cs.brakePressed),
        vCruise=float(cs.cruiseState.speed),
        lead=lead, e2e=e2e, aTarget=float(p.aTarget), allowThrottle=bool(p.allowThrottle), shouldStop=bool(p.shouldStop),
        hasLead=bool(p.hasLead), ss=ss, forceCoast=(fpcs or {}).get("forceCoast", False),
        fpExp=(fpp or {}).get("experimentalMode"), tFollow=(fpp or {}).get("tFollow"), maxA=(fpp or {}).get("maxA"), cc=cc, co=co,
      ))
  return seg_dir.name, out


def gates(s, brake_cutoff=-0.25, dts=-1.0):
  lead = Lead(s["lead"])
  v = s["vEgo"]
  e2e = s["e2e"] if s["e2e"] is not None else 0.0
  personality = s["ss"]["personality"]  # raw enum int
  g = dict(
    speed=lp.get_experimental_free_road_lead_speed_gate(v),
    gap=lp.get_experimental_free_road_lead_gap_gate(lead, v, personality) if lead.status else None,
    pull=lp.get_experimental_free_road_lead_pullaway_gate(lead, v) if lead.status else None,
    model=lp.get_experimental_free_road_model_gate(e2e, brake_cutoff),
    native=lp.get_experimental_free_road_native_accel_gate(e2e),
    allowed=lp.experimental_free_road_boost_allowed('blended', s["allowThrottle"], s["shouldStop"], s["forceCoast"], lead, v, personality),
  )
  if lead.status:
    confirmed = (dts < 0.0 and v < lp.EXPERIMENTAL_FREE_ROAD_DEPARTING_LEAD_MAX_EGO_SPEED
                 and lead.vRel >= lp.EXPERIMENTAL_FREE_ROAD_DEPARTING_LEAD_MIN_REL_SPEED
                 and lead.aLeadK >= lp.EXPERIMENTAL_FREE_ROAD_DEPARTING_LEAD_MIN_ACCEL
                 and lead.radarTrackId >= 0 and lead.modelProb >= lp.EXPERIMENTAL_FREE_ROAD_DEPARTING_LEAD_MIN_MODEL_PROB)
    sg = g["speed"] * g["gap"] * g["pull"]
    if confirmed:
      sg = max(sg, g["gap"] * g["pull"])
    g["confirmed"] = confirmed
    g["total"] = (g["model"] * g["native"] * sg) if g["allowed"] else 0.0
    g["boost_ub"] = lp.EXPERIMENTAL_FREE_ROAD_LEAD_BOOST_MAX * g["total"]
  else:
    g["total"] = None
    g["boost_ub"] = None
  return g


def find_launches(samples):
  """standstill (v<0.3 for >=1s) while enabled+lead -> moving. Ends at v>=9 m/s, 14 s, or disengage."""
  launches = []
  i = 0
  n = len(samples)
  while i < n:
    s = samples[i]
    if s["vEgo"] < 0.3 and s["ss"]["enabled"] and s["lead"]["status"]:
      j = i
      while j < n and samples[j]["vEgo"] < 0.3 and samples[j]["ss"]["enabled"]:
        j += 1
      still_dur = samples[j - 1]["t"] - s["t"] if j > i else 0
      if j < n and still_dur >= 1.0 and samples[j]["ss"]["enabled"]:
        k = j
        t0 = samples[j]["t"]
        while k < n and samples[k]["vEgo"] < 9.0 and samples[k]["t"] - t0 < 14.0 and (samples[k]["ss"]["enabled"] or samples[k]["gas"]):
          k += 1
        trace = samples[max(i, j - 20):k]
        if k - j >= 40:
          launches.append(dict(t0=t0, still=still_dur, trace=trace))
        i = k
        continue
      i = max(j, i + 1)
    else:
      i += 1
  return launches


def fmt(x, p=2):
  return "  -  " if x is None else f"{x:5.{p}f}"


def summarize(route, seg, L):
  tr = L["trace"]
  t0 = L["t0"]
  mov = [s for s in tr if s["t"] >= t0]
  exp = sum(1 for s in mov if s["ss"]["experimentalMode"]) / max(len(mov), 1)
  gas_t = next((s["t"] - t0 for s in mov if s["gas"]), None)
  active_lost = next((s["t"] - t0 for s in mov if not s["ss"]["active"]), None)
  def at(tt):
    return next((s for s in mov if s["t"] - t0 >= tt), None)
  rows = []
  for tt in (0.5, 1, 2, 3, 4, 5, 6, 8):
    s = at(tt)
    if s is None:
      break
    ld = s["lead"]
    g = gates(s)
    clc = lc.experimental_close_lead_accel_cap(s["vEgo"], ld["vLead"], ld["dRel"], lead_a=ld["aLeadK"]) if ld["status"] else None
    rows.append(dict(t=tt, v=s["vEgo"], a=s["aEgo"], e2e=s["e2e"], aT=s["aTarget"], cc=s["cc"], co=s["co"], maxA=s["maxA"], tF=s["tFollow"],
                     clc=clc, dRel=ld["dRel"], vLead=ld["vLead"],
                     aLeadK=ld["aLeadK"], tgap=(ld["dRel"] / s["vEgo"]) if s["vEgo"] > 0.5 else None, gas=s["gas"],
                     allow=s["allowThrottle"], gate=g["total"], sp=g["speed"], gp=g["gap"], pl=g["pull"], md=g["model"], nt=g["native"],
                     conf=g.get("confirmed"), lead=ld["status"]))
  return dict(route=route, seg=seg, t0=t0, still=L["still"], exp_frac=exp, gas_t=gas_t, active_lost=active_lost,
              personality=mov[0]["ss"]["personality"], rows=rows, n=len(mov))


def main():
  routes = sys.argv[1:]
  segs = []
  for r in routes:
    segs += sorted(ROOT.glob(f"{r}--*"), key=lambda p: int(p.name.rsplit("--", 1)[1]))
  results = []
  with ProcessPoolExecutor(8) as ex:
    for seg, samples in ex.map(segment_samples, segs):
      route = seg.rsplit("--", 1)[0]
      for L in find_launches(samples):
        results.append(summarize(route, seg, L))
  json.dump(results, open("/tmp/launches.json", "w"), indent=1)  # per-launch rows for follow-up scripting
  print(f"{len(results)} launches in {len(segs)} segments")
  for r in results:
    header = f"\n== {r['seg']} t0={r['t0']:.1f} still={r['still']:.1f}s exp={r['exp_frac']:.2f} pers={r['personality']}"
    print(f"{header} gas@{r['gas_t']} activeLost@{r['active_lost']}")
    print("   t    v     aEgo  e2e   aTgt  cc    sent  maxA  tF   clcap dRel  vLead aLdK  tgap  gas allow gate  sp   gp   pl   md   nt  conf lead")
    for w in r["rows"]:
      cols = [(w['v'], 2), (w['a'], 2), (w['e2e'], 2), (w['aT'], 2), (w['cc'], 2), (w['co'], 2), (w['maxA'], 2), (w['tF'], 2), (w['clc'], 2),
              (w['dRel'], 1), (w['vLead'], 2), (w['aLeadK'], 2), (w['tgap'], 1)]
      gates_cols = [(w['gate'], 2), (w['sp'], 2), (w['gp'], 2), (w['pl'], 2), (w['md'], 2), (w['nt'], 2)]
      print(f"  {w['t']:3.1f} " + " ".join(fmt(x, p) for x, p in cols)
            + f"  {int(w['gas'])}   {int(w['allow'])}  " + " ".join(fmt(x, p) for x, p in gates_cols)
            + f"  {w['conf']}  {int(w['lead'])}")


if __name__ == "__main__":
  main()
