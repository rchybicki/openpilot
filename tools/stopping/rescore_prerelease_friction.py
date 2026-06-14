#!/usr/bin/env python3
"""Re-score the deployed anti-stiction terminal pre-release through the friction-augmented plant.

DEVELOPMENT TOOL ONLY -- NOT A GATE. Everything here is a MODEL PREDICTION to be confirmed on-road.
The on-road IMU settle metric (settle_peak_imu_jerk, threshold 30 m/s^3 in scoring_config) remains the
promoter; this tool never participates in any pass/fail decision. It exists to give an OFFLINE READ on
whether the pre-release plausibly reduces the predicted terminal disc-grab, and which direction the
(coarse-provisional) friction model says to tune the pre-release knobs for the NEXT iteration.

WHAT IT DOES
  * Drives the LEGACY StoppingController (the live controller; V2 is dark and has no pre-release lane)
    through the friction-augmented plant (tools/stopping/sim_replay.simulate_stop with --friction),
    scoring the predicted-IMU settle jerk/decel the wheel plant is blind to.
  * Compares the deployed pre-release (HEAD module constants: A_TERMINAL_PRERELEASE=0.30,
    J_TERMINAL_PRERELEASE=1.5, ...) vs pre-release-OFF over the IMU-bearing event-store settles +
    stop_scenarios fixtures, reporting predicted settle_peak_imu_jerk/decel before/after.
  * Sweeps the two iteration knobs -- A_TERMINAL_PRERELEASE floor {0.25, 0.30, 0.35} x
    J_TERMINAL_PRERELEASE {1.0, 1.5, 2.0} -- against the friction-plant predicted grab and reports
    the direction the model wants.

HOW THE PRE-RELEASE IS TOGGLED / SWEPT (no controller edit): the pre-release reads module-level
constants in selfdrive.controls.lib.stopping_controller at call time. We monkeypatch those constants
around each run. "OFF" = set the floor so deep the release lane can never raise the command
(`limited_output < -A_TERMINAL_PRERELEASE` is never true) -- the controller is otherwise byte-identical.

HONEST CAVEATS (load-bearing):
  * The closed-loop drives off WHEEL a_ego (unchanged from the gated sim); the friction residual only
    augments the SCORED IMU channel. So the pre-release's *on-car* trigger gate `a_ego <= -0.10` sees
    the wheel channel exactly as on-road -- realistic -- but the AR(1) wheel plant under-resolves the
    sub-0.06 m/s band, so the predicted-IMU grab is a structure/ranking read, not a calibrated number.
  * The friction curve is coarse-provisional (n=26 stops/9 routes, c0/c1 on their physical-prior rails;
    only the velocity SHAPE v0 is well-constrained). Trust the SIGN/DIRECTION of the before/after and
    knob deltas, not the absolute m/s^3.
  * The friction residual is velocity-only and does NOT itself model stiction relief from a gentler
    command. The pre-release effect the sim sees is the kinematic one: a shallower terminal command
    settles the wheel trajectory differently, which shifts where/how fast v sweeps the grab band, which
    moves the predicted-IMU jerk. The TRUE stiction relief (gentler pad engagement) is unmodeled and is
    exactly what the on-road IMU must confirm.
"""

from __future__ import annotations

import argparse
import json
import sys
from contextlib import contextmanager
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
  sys.path.insert(0, str(REPO_ROOT))

from openpilot.selfdrive.controls.lib import stopping_controller as scon
from openpilot.tools.stopping import sim_replay as sr

# Knob sweep grid (development guidance for the next iteration; explicitly model-predicted).
A_FLOOR_SWEEP = (0.25, 0.30, 0.35)   # |A_TERMINAL_PRERELEASE| floor (m/s^2); deployed = 0.30
J_SWEEP = (1.0, 1.5, 2.0)            # J_TERMINAL_PRERELEASE release-side jerk ceiling (m/s^3); deployed = 1.5
PRERELEASE_OFF_FLOOR = 100.0         # so deep the release lane can never raise the command -> pre-release inert
A_FLOOR_DEPLOYED = 0.30              # HEAD A_TERMINAL_PRERELEASE
J_DEPLOYED = 1.5                     # HEAD J_TERMINAL_PRERELEASE


@contextmanager
def prerelease_knobs(a_floor: float, j: float):
  """Monkeypatch the live pre-release module constants for one run, restoring them after. Mirrors the
  controller's own derivation TERMINAL_PRERELEASE_RELEASE_STEP = J / 100.0 so J actually takes effect."""
  saved = (scon.A_TERMINAL_PRERELEASE, scon.J_TERMINAL_PRERELEASE, scon.TERMINAL_PRERELEASE_RELEASE_STEP)
  try:
    scon.A_TERMINAL_PRERELEASE = float(a_floor)
    scon.J_TERMINAL_PRERELEASE = float(j)
    scon.TERMINAL_PRERELEASE_RELEASE_STEP = float(j) / 100.0
    yield
  finally:
    scon.A_TERMINAL_PRERELEASE, scon.J_TERMINAL_PRERELEASE, scon.TERMINAL_PRERELEASE_RELEASE_STEP = saved


@dataclass
class ScoreSummary:
  label: str
  n_settles: int
  jerk_pred: list[float] = field(default_factory=list)
  decel_pred: list[float] = field(default_factory=list)
  triggered: int = 0   # # of settles where the pre-release lane fired at least once

  def _stats(self, xs: list[float]) -> dict[str, float | None]:
    if not xs:
      return {"median": None, "mean": None, "p90": None, "max": None}
    a = np.asarray(xs, dtype=float)
    return {"median": float(np.median(a)), "mean": float(np.mean(a)),
            "p90": float(np.percentile(a, 90)), "max": float(np.max(a))}

  def as_dict(self) -> dict[str, Any]:
    return {"label": self.label, "n_settles": self.n_settles, "n_prerelease_triggered": self.triggered,
            "predicted_settle_peak_imu_jerk": self._stats(self.jerk_pred),
            "predicted_settle_peak_imu_decel": self._stats(self.decel_pred)}


def score(scenarios, friction, dt: float, extend_s: float, collect_triggers: bool = True) -> ScoreSummary:
  """Run the legacy controller through the friction plant over all scenarios; collect predicted-IMU
  settle jerk/decel (and whether the pre-release lane fired). Returns a ScoreSummary."""
  plant = sr.PlantModel(sr.PLANT_PARAMS_REF, dt)
  summ = ScoreSummary(label="", n_settles=0)
  for scenario in scenarios:
    controller = sr.make_controller("legacy")
    trace = sr.simulate_stop(controller, plant, scenario, dt, extend_s=extend_s,
                             controller_name="legacy", plant_name="ref_20260514",
                             collect_debug=collect_triggers, friction=friction)
    metrics = sr.trace_metrics(trace, scenario)
    jp = metrics.get("settle_peak_imu_jerk_pred")
    dp = metrics.get("settle_peak_imu_decel_pred")
    if jp is None:
      continue
    summ.n_settles += 1
    summ.jerk_pred.append(float(jp))
    if dp is not None:
      summ.decel_pred.append(float(dp))
    if collect_triggers and any("terminal_prerelease" in (f.get("triggers") or ()) for f in trace.debug_frames):
      summ.triggered += 1
  return summ


def build_scenarios(args) -> list:
  scenarios: list = []
  store_dir = Path(args.event_store).expanduser()
  if store_dir.is_dir():
    scenarios.extend(sr.load_store_scenarios(store_dir, args.dt, max_events=args.max_events))
  if args.include_fixtures or not scenarios:
    scenarios.extend(sr.fixture_scenarios())
  return scenarios


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
  p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  p.add_argument("--event-store", default=str(sr.DEFAULT_EVENT_STORE))
  p.add_argument("--friction", default="default", help="friction fit: 'default' or a JSON path")
  p.add_argument("--include-fixtures", action="store_true", help="also score the stop_scenarios fixtures")
  p.add_argument("--max-events", type=int, default=0, help="cap event-store scenarios (0 = all)")
  p.add_argument("--dt", type=float, default=sr.DEFAULT_DT)
  p.add_argument("--extend-s", type=float, default=sr.DEFAULT_EXTEND_S)
  p.add_argument("--output-json", default=None)
  return p.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
  args = parse_args(argv)
  friction = sr.load_friction(args.friction)
  if friction is None:
    print("error: a friction fit is required for the predicted-IMU re-score", file=sys.stderr)
    return 2
  scenarios = build_scenarios(args)
  if not scenarios:
    print("error: no scenarios", file=sys.stderr)
    return 2

  c = friction.params
  print("=" * 92)
  print("DEVELOPMENT-ONLY pre-release re-score through the FRICTION PLANT -- MODEL PREDICTION, not proof.")
  print("On-road IMU settle_peak_imu_jerk (threshold 30 m/s^3) remains the promoter; this never gates.")
  print(f"friction resid(v) = {c.c0:+.4f}{c.c1:+.4f}*exp(-v/{c.v0:.4f})  (coarse-provisional, 2026-06-14)")
  print(f"scenarios={len(scenarios)}  controller=legacy (live)  plant=ref_20260514")
  print("=" * 92)

  # 1) Deployed pre-release (HEAD) vs pre-release OFF.
  with prerelease_knobs(scon.A_TERMINAL_PRERELEASE, scon.J_TERMINAL_PRERELEASE):
    on = score(scenarios, friction, args.dt, args.extend_s)
  on.label = f"prerelease ON (HEAD: A={A_FLOOR_DEPLOYED:.2f}, J={J_DEPLOYED:.1f})"
  with prerelease_knobs(PRERELEASE_OFF_FLOOR, scon.J_TERMINAL_PRERELEASE):
    off = score(scenarios, friction, args.dt, args.extend_s)
  off.label = "prerelease OFF"

  def jstat(s: ScoreSummary, k: str) -> float | None:
    return s.as_dict()["predicted_settle_peak_imu_jerk"][k]

  print("\n[1] DEPLOYED PRE-RELEASE vs OFF -- predicted settle_peak_imu_jerk (m/s^3):")
  for s in (off, on):
    js = s.as_dict()["predicted_settle_peak_imu_jerk"]
    print(f"  {s.label:42s}  n={s.n_settles:3d}  triggered={s.triggered:3d}  "
          + f"median={_f(js['median'])}  p90={_f(js['p90'])}  max={_f(js['max'])}")
  d_med = _delta(jstat(on, "median"), jstat(off, "median"))
  d_max = _delta(jstat(on, "max"), jstat(off, "max"))
  verdict = ("REDUCES" if (d_med is not None and d_med < -1e-6) else
             "INCREASES" if (d_med is not None and d_med > 1e-6) else "no change in")
  print(f"  -> pre-release {verdict} the predicted grab: median {_f(d_med)}, max {_f(d_max)} m/s^3 (MODEL prediction)")

  # 2) Knob sweep against the predicted grab.
  print("\n[2] KNOB SWEEP -- predicted settle_peak_imu_jerk median (m/s^3), A_TERMINAL_PRERELEASE x J:")
  sweep: dict[str, dict[str, Any]] = {}
  header = "        " + "  ".join(f"J={j:<4.1f}" for j in J_SWEEP)
  print(header)
  best = None  # (median, a, j)
  for a_floor in A_FLOOR_SWEEP:
    cells = []
    for j in J_SWEEP:
      with prerelease_knobs(a_floor, j):
        s = score(scenarios, friction, args.dt, args.extend_s, collect_triggers=False)
      med = s.as_dict()["predicted_settle_peak_imu_jerk"]["median"]
      sweep[f"A{a_floor:.2f}_J{j:.1f}"] = s.as_dict()
      cells.append(_f(med))
      if med is not None and (best is None or med < best[0]):
        best = (med, a_floor, j)
    print(f"  A={a_floor:.2f}  " + "  ".join(f"{c:>7s}" for c in cells))
  if best is not None:
    print(f"  -> model-lowest predicted grab at A={best[1]:.2f}, J={best[2]:.1f} (median {best[0]:.2f} m/s^3)")
    print("     direction vs deployed (A=0.30,J=1.5): " + _sweep_direction(sweep))

  report = {
    "_banner": "DEVELOPMENT TOOL -- NOT A GATE. MODEL PREDICTION through the coarse-provisional friction "
               + "plant; the on-road IMU settle_peak_imu_jerk remains the promoter. Confirm on-road.",
    "friction_residual": friction.as_dict(),
    "n_scenarios": len(scenarios),
    "controller": "legacy",
    "plant": "ref_20260514",
    "prerelease_on_vs_off": {"on": on.as_dict(), "off": off.as_dict(),
                             "predicted_jerk_median_delta_on_minus_off": d_med,
                             "predicted_jerk_max_delta_on_minus_off": d_max,
                             "verdict": verdict},
    "knob_sweep": sweep,
  }
  if args.output_json:
    out = Path(args.output_json).expanduser()
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")
    print(f"\nwrote {out}")
  return 0


def _f(x: float | None) -> str:
  return "  n/a" if x is None else f"{x:.2f}"


def _delta(a: float | None, b: float | None) -> float | None:
  return None if (a is None or b is None) else (a - b)


def _sweep_direction(sweep: dict[str, dict[str, Any]]) -> str:
  """Plain-English read of which way the model wants each knob, anchored at the deployed cell."""
  def med(a: float, j: float) -> float | None:
    return sweep.get(f"A{a:.2f}_J{j:.1f}", {}).get("predicted_settle_peak_imu_jerk", {}).get("median")
  base = med(0.30, 1.5)
  if base is None:
    return "indeterminate (deployed cell produced no predicted settle)"
  parts = []
  # A direction at deployed J
  a_lo, a_hi = med(0.25, 1.5), med(0.35, 1.5)
  if a_lo is not None and a_hi is not None:
    if a_hi < base - 1e-6 and a_hi <= a_lo:
      parts.append("deeper floor (A=0.35) lowers predicted grab")
    elif a_lo < base - 1e-6 and a_lo <= a_hi:
      parts.append("shallower floor (A=0.25) lowers predicted grab")
    else:
      parts.append("floor change ~neutral on predicted grab")
  # J direction at deployed A
  j_lo, j_hi = med(0.30, 1.0), med(0.30, 2.0)
  if j_lo is not None and j_hi is not None:
    if j_hi < base - 1e-6 and j_hi <= j_lo:
      parts.append("faster release (J=2.0) lowers predicted grab")
    elif j_lo < base - 1e-6 and j_lo <= j_hi:
      parts.append("slower release (J=1.0) lowers predicted grab")
    else:
      parts.append("release-rate change ~neutral on predicted grab")
  return "; ".join(parts) if parts else "indeterminate"


if __name__ == "__main__":
  sys.exit(main())
