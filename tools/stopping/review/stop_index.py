#!/usr/bin/env python3
"""Persistent STOP INDEX -- the single entry point of a review cycle (context-lean by design).

Stage A (cheap): triage EVERY synced segment's qlog (stop runs, bookmarks, build, engaged frames).
Stage B (heavy, only for candidate segments = stop runs / bookmarks / neighbours): rlog deep-stop census,
felt-jerk score, the service's own `settle_summary` events (incl. the shadow-governor gov_* fields), and a
DETECTOR AUDIT (service settles the heuristic missed, heuristic settles the service never owned).
Rows accumulate in ~/.route_sync/corpus/stop_index.jsonl (deduped)
processed segments are remembered, so a
re-run only touches new data. Output: per-route one-liners + one line per stop, ATTENTION rows first
`--all` prints everything, `--quiet` only the summary.

usage: stop_index.py [--since 00002024] [--routes 00002029,...] [--all] [--quiet] [--rebuild]
"""
import argparse
import collections
import json
import os
import pathlib
import statistics
import subprocess
import sys

REPO = "/Users/radoslawchybicki/Repos/openpilot-rch"
REV = f"{REPO}/tools/stopping/review"
DATA = pathlib.Path.home() / ".route_sync/data/media/0/realdata"
CORPUS = pathlib.Path.home() / ".route_sync/corpus"
INDEX = CORPUS / "stop_index.jsonl"
STATE = CORPUS / "stop_index_state.json"
PY = "/opt/homebrew/bin/python3.11"
ENV = dict(os.environ, PYTHONPATH=f"{REPO}:{REPO}/.venv/lib/python3.11/site-packages")


def run_many(script, paths, workers=8):
  """run one review script per path in parallel
  returns {path: stdout} (stderr dropped)."""
  out = {}
  procs = []
  for p in paths:
    procs.append((p, subprocess.Popen([PY, f"{REV}/{script}", *([p] if isinstance(p, str) else p)],
                                      stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True, env=ENV, cwd=DATA)))
    if len(procs) >= workers:
      p0, pr = procs.pop(0)
      out[p0 if isinstance(p0, str) else p0[0]] = pr.communicate()[0]
  for p0, pr in procs:
    out[p0 if isinstance(p0, str) else p0[0]] = pr.communicate()[0]
  return out


def settle_events(rlog):
  """service settle_summary events from one rlog (times relative to the rlog's first event, which is the
  route-start initData replayed at the head of EVERY segment -- i.e. route-relative; settle_events.first_mono
  records the origin so a neighbour's events can be shifted by the exact origin delta, normally 0)."""
  sys.path.insert(0, REV)
  from triage_one import read_events
  first = None
  rows = []
  for ev in read_events(rlog):
    t = ev.logMonoTime * 1e-9
    if first is None:
      first = t
      settle_events.first_mono[rlog] = t   # segment clock origin (every rlog replays initData at ROUTE start)
    if ev.which() != "logMessage":
      continue
    m = ev.logMessage
    if "settle_summary" not in m:
      continue
    try:
      d = json.loads(m)
      msg = d.get("msg", d)
    except Exception:
      continue
    rows.append({"t": round(t - first, 1), "frames": msg.get("frames"), "rest_gap": msg.get("rest_gap"), "min_gap": msg.get("min_gap"),
                 "wheel_stop_wire": msg.get("wheel_stop_wire"), "post_stop_v_max": msg.get("post_stop_v_max"),
                 "gov_frames": msg.get("gov_frames"), "gov_max_div": msg.get("gov_max_div"),
                 "gov_deeper_frac": msg.get("gov_deeper_frac"), "gov_shallower_frac": msg.get("gov_shallower_frac"),
                 "gov_min": msg.get("gov_min"), "phase_timeline": msg.get("phase_timeline"),
                 "gov_trace": msg.get("gov_trace"),
                 # attributed-safety shadow (2026-09-02): the flip gate's per-settle inputs
                 "attr_frames": msg.get("attr_frames"), "attr_ineligible": msg.get("attr_ineligible"),
                 "attr_plan_bound": msg.get("attr_plan_bound"), "attr_unexplained": msg.get("attr_unexplained"),
                 "attr_released_sum": msg.get("attr_released_sum"), "attr_pred_bound": msg.get("attr_pred_bound"),
                 "attr_reasons": msg.get("attr_reasons"), "attr_ring": msg.get("attr_ring")})
  # approach-only shadow stats (v >= 0.5 m/s: the law does not model the clutch hold, so hold frames
  # would dominate the whole-settle fractions)
  for r in rows:
    stats = gov_approach_stats(r.get("gov_trace"))
    if stats:
      r.update(stats)
  return rows


settle_events.first_mono = {}


def gov_approach_stats(trace):
  """Approach-only shadow stats. A NEW trace (any entry carries lead_v) uses ONLY stopped-lead frames
  (|lead_v| <= 0.3): moving-lead frames belong to the planner safety lane by design, and a new trace
  with no stopped-lead samples is EXCLUDED from the aggregates (R1: falling back to moving-lead
  frames biases the flip gate). Legacy 6-tuple traces stay unconditioned and are marked as such."""
  tr = [x for x in (trace or []) if x[1] is not None and x[1] >= 0.5 and x[3] is not None and x[4] is not None]
  if not tr:
    return None
  out = {"gov_implausible": any(x[2] is not None and x[2] < 1.5 and x[1] > 3.0 for x in tr)}
  new_style = any(len(x) > 6 for x in tr)
  use = [x for x in tr if len(x) > 6 and x[6] is not None and abs(x[6]) <= 0.3] if new_style else tr
  out["gov_appr_conditioned"] = new_style
  if not use:
    return out    # new trace, no stopped-lead samples: implausibility flag only, no aggregates
  out["gov_appr_n"] = len(use)
  out["gov_appr_div"] = round(max(abs(x[4] - x[3]) for x in use), 2)
  out["gov_appr_deeper"] = round(sum(x[4] < x[3] - 0.05 for x in use) / len(use), 2)
  out["gov_appr_shallower"] = round(sum(x[4] > x[3] + 0.05 for x in use) / len(use), 2)
  return out


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument("--since", default=None, help="route counter (hex) exclusive lower bound")
  ap.add_argument("--routes", default=None, help="comma-separated route counters")
  ap.add_argument("--all", action="store_true")
  ap.add_argument("--quiet", action="store_true")
  ap.add_argument("--rebuild", action="store_true", help="ignore the processed-segment memory")
  a = ap.parse_args()
  CORPUS.mkdir(exist_ok=True)
  state = json.load(open(STATE)) if STATE.exists() and not a.rebuild else {"segments": []}
  done = set(state["segments"])
  segs = sorted(p for p in os.listdir(DATA) if p.startswith("0000") and "--" in p)
  if a.routes:
    want = set(a.routes.split(","))
    segs = [s for s in segs if s.split("--")[0] in want]
  elif a.since:
    segs = [s for s in segs if int(s.split("--")[0], 16) > int(a.since, 16)]
  if a.routes:   # explicit routes are REPROCESSED: forget their segments and drop their old rows (tooling fixes re-index)
    done = {s for s in done if s.split("--")[0] not in want}
    if INDEX.exists():
      keep = [l for l in open(INDEX) if not any(l.startswith(f'{{"seg": "{w}--') for w in want)]
      open(INDEX, "w").writelines(keep)
  new = [s for s in segs if s not in done and (DATA / s / "qlog.zst").exists()]
  # --- stage A: qlog triage over every new segment
  tri = {}
  for s, txt in run_many("triage_one.py", [f"{s}/qlog.zst" for s in new]).items():
    try:
      tri[s.split("/")[0]] = json.loads(txt)
    except Exception:
      pass
  by_route = collections.defaultdict(lambda: {"segs": 0, "commits": set(), "bm": 0, "runs": 0, "en": 0, "cand": 0})
  cand = set()
  for s, r in tri.items():
    b = by_route[s.split("--")[0]]
    b["segs"] += 1
    b["commits"].add(r.get("gitCommit"))
    b["bm"] += r.get("bookmarks", 0)
    b["runs"] += len(r.get("stop_runs", []))
    b["en"] += r.get("enabled_frames", 0)
    if r.get("stop_runs") or r.get("bookmarks"):
      cand.add(s)
      route, rid, idx = s.split("--")   # neighbours: a stop can straddle a segment boundary
      for k in (int(idx) - 1, int(idx) + 1):
        n = f"{route}--{rid}--{k}"
        if (DATA / n / "rlog.zst").exists():
          cand.add(n)
  cand = sorted(c for c in cand if (DATA / c / "rlog.zst").exists())
  for s in tri:
    by_route[s.split("--")[0]]["cand"] += s in cand
  # --- stage B: rlog analysis only for candidates
  deep = {}
  for s, txt in run_many("deep_stop.py", [f"{s}/rlog.zst" for s in cand]).items():
    try:
      deep[s.split("/")[0]] = json.loads(txt)
    except Exception:
      pass
  jobs = []
  for s, d in deep.items():
    for e in d.get("events", []):
      jobs.append((f"{s}/rlog.zst", str(e["t_settle"]), f"{s}|{e['t_settle']}"))
  felt = {}
  procs = [(job, subprocess.Popen([PY, f"{REV}/felt_one.py", *job], stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                                  text=True, env=ENV, cwd=DATA)) for job in jobs]
  for job, pr in procs:
    try:
      felt[job[2]] = json.loads(pr.communicate()[0]).get("felt_jerk_max")
    except Exception:
      felt[job[2]] = None
  rows = []
  audit = []
  svc_cache = {}

  def svc_events(seg):
    if seg not in svc_cache:
      p = DATA / seg / "rlog.zst"
      svc_cache[seg] = settle_events(str(p)) if p.exists() else []
    return svc_cache[seg]

  for s in cand:
    d = deep.get(s, {"events": []})
    ev_service = list(svc_events(s))
    # a hold that runs past the segment boundary emits its settle_summary in the NEXT rlog (2026-09-02:
    # seven stopped-lead stops were flagged NO_SERVICE_EVENT although the service owned them) -- borrow the
    # neighbour's events shifted by the clock-origin delta (0: both rlogs replay the route-start initData); they are matched here but never counted as MISSES here
    route, rid, idx = s.split("--")
    nxt = f"{route}--{rid}--{int(idx) + 1}"
    nxt_rows = svc_events(nxt)
    fm = settle_events.first_mono
    shift = fm.get(str(DATA / nxt / "rlog.zst"), 0.0) - fm.get(str(DATA / s / "rlog.zst"), 0.0)
    ev_service += [dict(x, t=x["t"] + shift, from_next=True) for x in nxt_rows]
    settles = d.get("events", [])
    def covers(x, t_settle):
      # settle_summary is emitted when the service goes INACTIVE (end of the hold); its window started
      # `frames` x 10 ms earlier -- a heuristic settle inside [start - 3 s, end + 1 s] belongs to it
      start = x["t"] - (x["frames"] or 0) / 100.0
      return start - 3.0 <= t_settle <= x["t"] + 1.0
    for e in settles:
      svc = next((x for x in ev_service if covers(x, e["t_settle"])), None)
      lv = e.get("lead_v")
      rg = e.get("rest_gap")
      fj = felt.get(f"{s}|{e['t_settle']}")
      att = []
      if rg is not None and rg < 15:
        if rg < 3.5:
          att.append("SHORT")
        if rg > 5.0:
          att.append("LONG")
      if fj is not None and fj > 1.0:
        att.append("HARSH")
      if e.get("wire_at_stop") == 0.0 or e.get("taxonomy", "").startswith("DISLIKE"):
        att.append("TAKEOVER?")
      if svc is None:
        att.append("NO_SERVICE_EVENT")
      if svc is not None and svc.get("gov_implausible"):
        att.append("IMPLAUSIBLE_TRACE")
      if (e.get("fc") or 0) >= 0.5:
        att.append("FORCE_COAST")   # the no-lead/force-coast class censuses separately (user directive 2026-08-29)
      rows.append({"seg": s, "t": e["t_settle"], "rest_gap": rg, "lead_v": lv, "fc": e.get("fc"), "v_appr": e.get("v_appr"), "cmd_min": e.get("cmd_min"),
                   "felt_appr": e.get("felt_appr"), "a_wstop": e.get("a_wheelstop"),
                   "wire_at_stop": e.get("wire_at_stop"), "pdec": e.get("pdec"), "felt": fj, "taxonomy": e.get("taxonomy"),
                   "bookmark": bool(tri.get(s, {}).get("bookmarks")), "commit": tri.get(s, {}).get("gitCommit"),
                   "svc": svc, "attention": att})
    for x in ev_service:   # service settles the heuristic did not detect: they COUNT (the long class hid here)
      if x.get("from_next"):
        continue           # the neighbour's own pass audits its events
      if not any(covers(x, e["t_settle"]) for e in settles):
        rg = x.get("rest_gap")
        att = ["HEURISTIC_MISS"] + (["LONG"] if rg is not None and rg > 5.0 else []) + (["SHORT"] if rg is not None and rg < 3.5 else [])
        rows.append({"seg": s, "t": round(x["t"] - (x["frames"] or 0) / 100.0, 1), "rest_gap": None if rg is None else round(rg, 2),
                     "lead_v": None, "v_appr": None, "cmd_min": None, "wire_at_stop": x.get("wheel_stop_wire"), "pdec": None,
                     "felt": None, "taxonomy": "service_only", "bookmark": bool(tri.get(s, {}).get("bookmarks")),
                     "commit": tri.get(s, {}).get("gitCommit"), "svc": x, "attention": att})
        audit.append({"seg": s, "t": x["t"], "service_rest_gap": rg, "note": "service settle without a detected stop"})
  # --- persist
  seen = set()
  if INDEX.exists() and not a.rebuild:
    for l in open(INDEX):
      try:
        r = json.loads(l)
        seen.add((r["seg"], round(float(r["t"]), 1)))
      except Exception:
        pass
  with open(INDEX, "a" if not a.rebuild else "w") as f:
    for r in rows:
      if (r["seg"], round(float(r["t"]), 1)) not in seen:
        f.write(json.dumps(r) + "\n")
  state["segments"] = sorted(done | set(tri.keys()))
  json.dump(state, open(STATE, "w"))
  # --- report (compact)
  for k in sorted(by_route):
    b = by_route[k]
    builds = sorted(c for c in b['commits'] if c)
    print(f"{k}: segs {b['segs']:3d} cand {b['cand']:3d} stop_runs {b['runs']:2d} bookmarks {b['bm']} enabled {b['en']:6d} build {builds}")
  if a.quiet:
    return
  rows.sort(key=lambda r: (not r["attention"], r["seg"], r["t"]))
  print(f"\n{'seg':26} {'t':8} {'rest':5} {'lv':5} {'vappr':6} {'cmdmin':7} {'felt':5} {'feltA':5} {'govdivA':7} {'govdeepA':8} {'unexp/bnd/n':9} attention")
  shown = 0
  for r in rows:
    if not a.all and not r["attention"] and shown >= 12:
      continue
    svc = r["svc"] or {}
    head = f"{r['seg']:26} {r['t']:8.1f} {str(r['rest_gap']):5} {str(r['lead_v']):5} {str(r['v_appr']):6} {str(r['cmd_min']):7} {str(r['felt']):5} {str(r.get('felt_appr')):5}"
    attr = (f"{svc['attr_unexplained']}/{svc['attr_plan_bound']}/{svc['attr_frames']}"
            if svc.get("attr_frames") else "-")
    tail = f"{str(svc.get('gov_appr_div')):7} {str(svc.get('gov_appr_deeper')):8} {attr:9} {','.join(r['attention'])}{' BM' if r['bookmark'] else ''}"
    print(head + " " + tail)
    shown += 1
  hidden = len(rows) - shown
  if hidden > 0:
    print(f"... {hidden} unflagged stops not shown (--all)")
  lead_stops = [r["rest_gap"] for r in rows if r["rest_gap"] is not None and r["rest_gap"] < 15 and (r["lead_v"] or 9) < 0.3]
  if lead_stops:
    n_short = sum(g < 3.5 for g in lead_stops)
    n_long = sum(g > 5.0 for g in lead_stops)
    print(f"stopped-lead rests n={len(lead_stops)} median {statistics.median(lead_stops):.2f} <3.5 {n_short} >5.0 {n_long}")
  gov = [r["svc"] for r in rows if r["svc"] and r["svc"].get("gov_appr_n") and not r["svc"].get("gov_implausible")]
  n_cond = sum(1 for g in gov if g.get("gov_appr_conditioned"))
  if gov:
    def med(key):
      return statistics.median(g[key] for g in gov)
    print(f"shadow governor (approach v>=0.5; {n_cond}/{len(gov)} conditioned on a STOPPED lead): "
          + f"max|gov-wire| p50 {med('gov_appr_div'):.2f}; deeper p50 {med('gov_appr_deeper'):.2f}; "
          + f"shallower p50 {med('gov_appr_shallower'):.2f}")
  for x in audit[:8]:
    print("AUDIT", x)
  if len(audit) > 8:
    print(f"AUDIT ... {len(audit)-8} more")


if __name__ == "__main__":
  main()
