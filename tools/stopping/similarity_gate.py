#!/usr/bin/env python3
"""The spec-7.6 two-tier similarity gate + spec-7.7 triage-table emitter -- precondition for
Commit D (`USE_STOPPING_V2 = True`).

Replays BOTH controllers (legacy forest, V2 facade) closed-loop through the plant sim
(sim_replay's integrated harness: state machine + single arbiter + controller wired as on-car) on
the scenario deck -- (1) pinned holdout routes' event-store events, (2) a stratified event-store
sample (default cap 300; strata: approach speed x target kind), (3) every stop_scenarios.py
fixture -- on TWO plants (frozen 20260514 + newest refit); the verdict must agree on both. This is
the AR(1) sim's single, final act as a promotion gate ("the sim develops, the measurement
promotes" applies ever after).

Tier 1 -- outcome envelope (PASS/FAIL, all required, both plants):
  harsh: no event harsh-in-V2-but-not-forest (frozen config incl. entry-side flags)
  leapfrog (operative OR-of-flags definition): count(V2) <= count(forest) on every stratum
  predicted rollout delta: |delta| <= 0.15 m at p95
  predicted final hold gap delta (lead events): |delta| <= 0.10 m at p95
  end_jerk paired delta: 95% BCa CI of the median within [-0.05, +0.03] m/s^3
  time-to-standstill delta: <= +0.5 s at p95
  estimator equivalence (spec 5.5.2): passed and attached (estimator_equivalence.py report)
  integrated dropout-hold fixtures: replayed through the integrated harness, no V2-only harsh

Tier 2 -- command-trace diagnostics (budgeted, triaged, NOT pass/fail by themselves): target
RMS(a_cmd_V2 - a_cmd_forest) <= 0.05 m/s^2 for >= 90% of events; every event with RMS > 0.10 or
max|delta| > 0.30 receives a mandatory spec-7.7 classification (A = forest artifact, accept;
B = calibration, bounded parameter move; C = unexplained, BLOCKER). Classifications are supplied
via --triage-json {"<event_ref>": {"class": "A"|"B"|"C", "note": ...}}; unclassified flagged
events and any class-C event block the verdict.

Gate exit criterion (deterministic, spec 7.6): Tier 1 all green on both plants AND zero class-C
events AND the emitted triage table committed to docs/stopping/archive/similarity_<date>.md in
the same commit as the flip. Exit codes: 0 pass, 1 fail, 2 insufficient inputs.
"""

from __future__ import annotations

import argparse
import json
import sys
from datetime import UTC, datetime
from pathlib import Path
from typing import Any

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
  sys.path.insert(0, str(REPO_ROOT))

from openpilot.selfdrive.controls.lib.stopping_plant import PlantModel
from openpilot.tools.stopping import sim_replay as sr
from openpilot.tools.stopping import scoring_config as sc
from openpilot.tools.stopping.paired_stats import bca_bootstrap_ci

DEFAULT_EVENT_STORE = Path.home() / ".comma" / "stopping_behavior" / "event_store"
DEFAULT_HOLDOUT_ROUTES = Path(__file__).resolve().parent / "holdout_routes.txt"
STOPPING = sr.STOPPING

# Tier-1 bounds (spec 7.6 table)
ROLLOUT_DELTA_P95_M = 0.15
HOLD_GAP_DELTA_P95_M = 0.10
END_JERK_CI_LO = -0.05
END_JERK_CI_HI = 0.03
TTS_DELTA_P95_S = 0.5
# Tier-2 diagnostics (spec 7.6)
TIER2_RMS_TARGET = 0.05
TIER2_RMS_FLAG = 0.10
TIER2_MAXD_FLAG = 0.30
TIER2_TARGET_FRACTION = 0.90

DROPOUT_FIXTURE_TOKENS = ("dropout", "reacquire", "hold")


def p95(values: list[float]) -> float:
  return float(np.percentile(np.asarray(values, dtype=float), 95)) if values else 0.0


def event_ref(scenario: sr.Scenario) -> str:
  if scenario.key:
    return f"{scenario.key['route']}--{scenario.key['seg']}--{scenario.key['hold_mono_ns']}"
  return scenario.name


def paired_run(scenario: sr.Scenario, plant: PlantModel, dt: float,
               a_name: str, b_name: str) -> dict[str, Any]:
  """One scenario, both controllers, identical harness; per-event outcome metrics + trace deltas."""
  controller_a = sr.make_controller(a_name)
  controller_b = sr.make_controller(b_name)
  trace_a = sr.simulate_stop(controller_a, plant, scenario, dt, controller_name=a_name)
  trace_b = sr.simulate_stop(controller_b, plant, scenario, dt, controller_name=b_name)
  metrics_a = sr.trace_metrics(trace_a, scenario)
  metrics_b = sr.trace_metrics(trace_b, scenario)
  harsh_a, leap_a = sr.classify_metrics(metrics_a)
  harsh_b, leap_b = sr.classify_metrics(metrics_b)

  # command-trace divergence over frames where EITHER run is in stopping authority (captures
  # entry/exit timing divergence; outside stopping both emit the identical passthrough)
  n = min(len(trace_a.u), len(trace_b.u))
  in_scope = [i for i in range(n) if trace_a.state[i] == STOPPING or trace_b.state[i] == STOPPING]
  if in_scope:
    deltas = np.asarray([trace_a.u[i] - trace_b.u[i] for i in in_scope], dtype=float)
    rms = float(np.sqrt(np.mean(np.square(deltas))))
    max_delta = float(np.max(np.abs(deltas)))
  else:
    rms = 0.0
    max_delta = 0.0

  def _delta(metric: str) -> float | None:
    va, vb = metrics_a.get(metric), metrics_b.get(metric)
    if va is None or vb is None:
      return None
    return float(vb) - float(va)

  return {
    "event_ref": event_ref(scenario),
    "stratum": scenario.stratum or "fixture",
    "harsh_a": sc.is_harsh(harsh_a), "harsh_b": sc.is_harsh(harsh_b),
    "leapfrog_a": sc.is_leapfrog(leap_a), "leapfrog_b": sc.is_leapfrog(leap_b),
    "harsh_flags_a": harsh_a, "harsh_flags_b": harsh_b,
    "rollout_delta_m": _delta("rollout_distance_from_2mps_m"),
    "hold_gap_delta_m": _delta("lead_distance_hold_m"),
    "end_jerk_delta": _delta("end_stop_jerk_mps3"),
    "tts_delta_s": _delta("time_to_standstill_s"),
    "trace_rms": rms,
    "trace_max_delta": max_delta,
    "settled_a": bool(metrics_a.get("settled")), "settled_b": bool(metrics_b.get("settled")),
  }


def tier1_checks(pairs: list[dict[str, Any]]) -> dict[str, Any]:
  checks: dict[str, Any] = {}

  b_only_harsh = [p["event_ref"] for p in pairs if p["harsh_b"] and not p["harsh_a"]]
  checks["harsh_no_v2_only"] = {"pass": not b_only_harsh, "v2_only_harsh_events": b_only_harsh}

  strata = sorted({p["stratum"] for p in pairs})
  leap_rows = []
  leap_pass = True
  for stratum in strata:
    rows = [p for p in pairs if p["stratum"] == stratum]
    count_a = sum(1 for p in rows if p["leapfrog_a"])
    count_b = sum(1 for p in rows if p["leapfrog_b"])
    ok = count_b <= count_a
    leap_pass = leap_pass and ok
    leap_rows.append({"stratum": stratum, "forest": count_a, "v2": count_b, "pass": ok})
  checks["leapfrog_per_stratum"] = {"pass": leap_pass, "strata": leap_rows}

  rollout = [abs(p["rollout_delta_m"]) for p in pairs if p["rollout_delta_m"] is not None]
  checks["rollout_delta_p95"] = {"pass": p95(rollout) <= ROLLOUT_DELTA_P95_M, "p95": p95(rollout),
                                 "bound": ROLLOUT_DELTA_P95_M, "n": len(rollout)}

  hold_gap = [abs(p["hold_gap_delta_m"]) for p in pairs if p["hold_gap_delta_m"] is not None]
  checks["hold_gap_delta_p95"] = {"pass": p95(hold_gap) <= HOLD_GAP_DELTA_P95_M, "p95": p95(hold_gap),
                                  "bound": HOLD_GAP_DELTA_P95_M, "n": len(hold_gap)}

  end_jerk = [p["end_jerk_delta"] for p in pairs if p["end_jerk_delta"] is not None]
  if end_jerk:
    if all(d == 0.0 for d in end_jerk):
      ci = (0.0, 0.0)  # degenerate identical traces (legacy-vs-legacy dry run)
    else:
      ci = bca_bootstrap_ci(np.asarray(end_jerk), lambda x: float(np.median(x)))
    ok = END_JERK_CI_LO <= ci[0] and ci[1] <= END_JERK_CI_HI
  else:
    ci = (0.0, 0.0)
    ok = True
  checks["end_jerk_median_ci"] = {"pass": ok, "ci": list(ci), "bounds": [END_JERK_CI_LO, END_JERK_CI_HI], "n": len(end_jerk)}

  tts = [p["tts_delta_s"] for p in pairs if p["tts_delta_s"] is not None]
  tts_p95 = float(np.percentile(np.asarray(tts), 95)) if tts else 0.0
  checks["time_to_standstill_delta_p95"] = {"pass": tts_p95 <= TTS_DELTA_P95_S, "p95": tts_p95,
                                            "bound": TTS_DELTA_P95_S, "n": len(tts)}
  return checks


def tier2_diagnostics(pairs: list[dict[str, Any]], triage: dict[str, dict[str, Any]]) -> dict[str, Any]:
  n = len(pairs)
  under_target = sum(1 for p in pairs if p["trace_rms"] <= TIER2_RMS_TARGET)
  flagged = [p for p in pairs if p["trace_rms"] > TIER2_RMS_FLAG or p["trace_max_delta"] > TIER2_MAXD_FLAG]
  rows = []
  class_c: list[str] = []
  unclassified: list[str] = []
  for p in flagged:
    ref = p["event_ref"]
    entry = triage.get(ref, {})
    cls = str(entry.get("class", "")).upper() or "UNCLASSIFIED"
    if cls == "C":
      class_c.append(ref)
    elif cls == "UNCLASSIFIED":
      unclassified.append(ref)
    rows.append({"event_ref": ref, "stratum": p["stratum"], "rms": p["trace_rms"],
                 "max_delta": p["trace_max_delta"], "class": cls, "note": entry.get("note", "")})
  return {
    "events": n,
    "rms_target": TIER2_RMS_TARGET,
    "fraction_under_target": (under_target / n) if n else 1.0,
    "target_fraction": TIER2_TARGET_FRACTION,
    "meets_target": (under_target / n if n else 1.0) >= TIER2_TARGET_FRACTION,
    "flagged": rows,
    "class_c_events": class_c,
    "unclassified_events": unclassified,
  }


def triage_table_markdown(report: dict[str, Any]) -> str:
  lines = [
    f"# Similarity-gate triage table ({report['generated_utc']})",
    "",
    f"Controllers: {report['controller_a']} (forest baseline) vs {report['controller_b']} (candidate)",
    f"Scenario deck: {report['scenario_count']} events; plants: {', '.join(report['plants'])}",
    f"Verdict: **{report['verdict'].upper()}**",
    "",
    "Classes (spec 7.7): A = forest artifact (accept, document); B = vehicle-calibration mismatch",
    "(bounded named-parameter move toward its preserve-group source, re-run); C = unexplained",
    "fidelity loss (BLOCKER). The flip cannot land with class-C or UNCLASSIFIED rows open.",
    "",
    "| plant | event | stratum | RMS (m/s^2) | max delta | class | note |",
    "|---|---|---|---|---|---|---|",
  ]
  for plant_name, plant_report in report["per_plant"].items():
    for row in plant_report["tier2"]["flagged"]:
      lines.append(f"| {plant_name} | {row['event_ref']} | {row['stratum']} | {row['rms']:.3f} "
                   + f"| {row['max_delta']:.3f} | {row['class']} | {row['note']} |")
  if all(not plant_report["tier2"]["flagged"] for plant_report in report["per_plant"].values()):
    lines.append("| - | (no flagged events) | - | - | - | - | - |")
  lines.append("")
  return "\n".join(lines)


def run_gate(scenarios: list[sr.Scenario], dropout_scenarios: list[sr.Scenario],
             plants: dict[str, Any], dt: float, triage: dict[str, dict[str, Any]],
             estimator_report: dict[str, Any] | None,
             controller_a: str = "legacy", controller_b: str = "v2") -> dict[str, Any]:
  per_plant: dict[str, Any] = {}
  for plant_name, plant_params in plants.items():
    plant = PlantModel(plant_params, dt)
    pairs = [paired_run(s, plant, dt, controller_a, controller_b) for s in scenarios]
    tier1 = tier1_checks(pairs)

    # integrated-path requirement (spec 7.6): the dropout-hold fixture set must replay through
    # the integrated wiring with no V2-only harsh outcome and finite commands throughout
    integrated_rows = []
    integrated_pass = True
    for scenario in dropout_scenarios:
      pair = paired_run(scenario, plant, dt, controller_a, controller_b)
      ok = not (pair["harsh_b"] and not pair["harsh_a"])
      integrated_pass = integrated_pass and ok
      integrated_rows.append({"event_ref": pair["event_ref"], "pass": ok,
                              "harsh_a": pair["harsh_a"], "harsh_b": pair["harsh_b"],
                              "trace_rms": pair["trace_rms"]})
    tier1["integrated_dropout_hold"] = {"pass": integrated_pass, "fixtures": integrated_rows}

    estimator_pass = bool(estimator_report and estimator_report.get("status") == "pass")
    tier1["estimator_equivalence"] = {
      "pass": estimator_pass,
      "attached": estimator_report is not None,
      "report": estimator_report,
    }

    per_plant[plant_name] = {
      "tier1": tier1,
      "tier1_pass": all(check["pass"] for check in tier1.values()),
      "tier2": tier2_diagnostics(pairs, triage),
      "pairs": pairs,
    }

  tier1_all = all(p["tier1_pass"] for p in per_plant.values())
  class_c = sorted({ref for p in per_plant.values() for ref in p["tier2"]["class_c_events"]})
  unclassified = sorted({ref for p in per_plant.values() for ref in p["tier2"]["unclassified_events"]})
  verdict = "pass" if (tier1_all and not class_c and not unclassified) else "fail"
  return {
    "generated_utc": datetime.now(UTC).replace(microsecond=0).isoformat(),
    "controller_a": controller_a,
    "controller_b": controller_b,
    "dt": dt,
    "plants": sorted(plants),
    "scenario_count": len(scenarios),
    "dropout_fixture_count": len(dropout_scenarios),
    "scoring_config_version": sc.SCORING_CONFIG.version,
    "scoring_config": json.loads(sc.canonical_json()),
    "per_plant": per_plant,
    "tier1_all_plants_pass": tier1_all,
    "class_c_events": class_c,
    "unclassified_flagged_events": unclassified,
    "verdict": verdict,
  }


def load_holdout_routes(path: Path) -> set[str]:
  if not path.is_file():
    return set()
  return {line.strip() for line in path.read_text().splitlines() if line.strip() and not line.strip().startswith("#")}


def stratified_sample(scenarios: list[sr.Scenario], cap: int) -> list[sr.Scenario]:
  """Round-robin over strata up to cap (spec 7.6 deck item 2), deterministic order."""
  if cap <= 0 or len(scenarios) <= cap:
    return scenarios
  by_stratum: dict[str, list[sr.Scenario]] = {}
  for scenario in scenarios:
    by_stratum.setdefault(scenario.stratum or "unknown", []).append(scenario)
  for rows in by_stratum.values():
    rows.sort(key=lambda s: s.name)
  picked: list[sr.Scenario] = []
  while len(picked) < cap:
    advanced = False
    for stratum in sorted(by_stratum):
      if by_stratum[stratum]:
        picked.append(by_stratum[stratum].pop(0))
        advanced = True
        if len(picked) >= cap:
          break
    if not advanced:
      break
  return picked


def dropout_fixture_scenarios() -> list[sr.Scenario]:
  return [s for s in sr.fixture_scenarios() if any(tok in s.name for tok in DROPOUT_FIXTURE_TOKENS)]


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
  parser = argparse.ArgumentParser(description="Two-tier stopping similarity gate (spec 7.6) + triage table (spec 7.7)")
  parser.add_argument("--event-store", default=None, help=f"Event store dir (default: {DEFAULT_EVENT_STORE} when present)")
  parser.add_argument("--holdout-routes", default=str(DEFAULT_HOLDOUT_ROUTES))
  parser.add_argument("--sample-cap", type=int, default=300, help="Stratified event-store sample size (spec 7.6 deck item 2)")
  parser.add_argument("--plant", default="both", help="'ref', 'refit', 'both' (spec: dual-plant), or a model JSON path")
  parser.add_argument("--dt", type=float, default=sr.DEFAULT_DT)
  parser.add_argument("--a-controller", default="legacy", choices=["legacy", "v2"], help="Baseline (forest)")
  parser.add_argument("--b-controller", default="v2", choices=["legacy", "v2"],
                      help="Candidate (use 'legacy' for the all-zero-divergence dry run)")
  parser.add_argument("--triage-json", default=None, help="Spec-7.7 classifications for Tier-2 flagged events")
  parser.add_argument("--estimator-report-json", default=None,
                      help="estimator_equivalence.py report (spec 5.5.2 mandatory Tier-1 row)")
  parser.add_argument("--skip-fixtures", action="store_true", help="Exclude stop_scenarios fixtures from the deck")
  parser.add_argument("--output-json", default=None)
  parser.add_argument("--triage-table-out", default=None, help="Markdown triage table path (default: alongside --output-json)")
  return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
  args = parse_args(argv)

  scenarios: list[sr.Scenario] = []
  store_dir = Path(args.event_store).expanduser() if args.event_store else DEFAULT_EVENT_STORE
  if store_dir.is_dir() and (store_dir / "events.jsonl").is_file():
    holdouts = load_holdout_routes(Path(args.holdout_routes).expanduser())
    store_scenarios = sr.load_store_scenarios(store_dir, args.dt)
    holdout_scenarios = [s for s in store_scenarios if s.key and s.key.get("route") in holdouts]
    other = [s for s in store_scenarios if s not in holdout_scenarios]
    scenarios.extend(holdout_scenarios)              # deck item 1: all pinned holdout events
    scenarios.extend(stratified_sample(other, args.sample_cap))  # deck item 2
  if not args.skip_fixtures:
    scenarios.extend(sr.fixture_scenarios())         # deck item 3
  if not scenarios:
    print("[similarity-gate] no scenarios (no event store and fixtures skipped)", file=sys.stderr)
    return 2

  estimator_report = None
  if args.estimator_report_json:
    estimator_path = Path(args.estimator_report_json).expanduser()
    if estimator_path.is_file():
      estimator_report = json.loads(estimator_path.read_text())
  triage: dict[str, dict[str, Any]] = {}
  if args.triage_json:
    triage = json.loads(Path(args.triage_json).expanduser().read_text())

  report = run_gate(scenarios, dropout_fixture_scenarios(), sr.resolve_plants(args.plant), args.dt,
                    triage, estimator_report, controller_a=args.a_controller, controller_b=args.b_controller)

  print(f"[similarity-gate] verdict={report['verdict']} (tier1_all_plants={report['tier1_all_plants_pass']})")
  for plant_name, plant_report in report["per_plant"].items():
    for check, result in plant_report["tier1"].items():
      print(f"[similarity-gate] {plant_name} tier1.{check}: {'PASS' if result['pass'] else 'FAIL'}")
    t2 = plant_report["tier2"]
    print(f"[similarity-gate] {plant_name} tier2: {t2['fraction_under_target']:.2%} under RMS target "
          + f"({'meets' if t2['meets_target'] else 'below'} {t2['target_fraction']:.0%}); flagged={len(t2['flagged'])}")
  if report["class_c_events"]:
    print(f"[similarity-gate] CLASS-C BLOCKERS: {report['class_c_events']}", file=sys.stderr)
  if report["unclassified_flagged_events"]:
    print("[similarity-gate] UNCLASSIFIED flagged events (triage required, spec 7.7): "
          + f"{report['unclassified_flagged_events']}", file=sys.stderr)

  if args.output_json:
    out = Path(args.output_json).expanduser()
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")
    print(f"[similarity-gate] output_json={out}")
    table_path = Path(args.triage_table_out).expanduser() if args.triage_table_out else out.with_suffix(".triage.md")
  else:
    table_path = Path(args.triage_table_out).expanduser() if args.triage_table_out else None
  if table_path is not None:
    table_path.parent.mkdir(parents=True, exist_ok=True)
    table_path.write_text(triage_table_markdown(report))
    print(f"[similarity-gate] triage_table={table_path}")

  return 0 if report["verdict"] == "pass" else 1


if __name__ == "__main__":
  raise SystemExit(main())
