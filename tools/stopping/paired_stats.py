#!/usr/bin/env python3
"""Paired statistics for stopping-stack A/B verdicts -- MDE-stating, refusal-capable (spec 7.4).

Two modes:
  * sim A/B (controller vs controller on the SAME events): per-event paired deltas on the
    pre-registered metrics; Wilcoxon signed-rank + BCa bootstrap 95% CI of the mean/median delta;
    binary metrics (harsh/leapfrog flags) via exact McNemar. Pre-registered floor:
    n >= 200 paired events.
  * on-road before/after (DIFFERENT events): stratified by approach-speed bin x lead/no-lead
    x signals_version; Mann-Whitney per stratum + stratified bootstrap of the pooled difference.
    Pre-registered power rule: >= 150 stops per arm (20% relative median-end-jerk change at 80%
    power for this corpus' dispersion). Cross-era rule (eval.md section 3.1, decided 2026-06-12):
    signals_version is dropped from the stratum key IFF every event in both arms records
    entry.isd_m == 0; otherwise cross-era arms stay refused-by-construction.

MANDATORY VERDICT FIELDS: every metric report carries `n` and `mde_at_n` (minimum detectable
effect at the observed n, alpha=0.05, power=0.80). Below the pre-registered floor the verdict is
REFUSED (status 'refused_insufficient_power') and the report prints the n required to resolve the
observed delta -- a gate that cannot detect a 10-20% effect at its n SAYS SO instead of passing.

Exit-code protocol (kept for cycle automation): 0 = no regression, 1 = regression detected,
2 = insufficient data / verdict refused, 3 = environment error.

Implementation note: tests are normal-approximation forms (Wilcoxon/Mann-Whitney with tie
corrections, exact binomial McNemar) on stdlib + numpy only -- exactness at tiny n is irrelevant
by construction because verdicts below the power floor are refused.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from dataclasses import asdict, dataclass, field
from pathlib import Path
from statistics import NormalDist
from typing import Any
from collections.abc import Callable, Sequence

import numpy as np

ALPHA = 0.05
POWER = 0.80
SIM_AB_MIN_PAIRED_EVENTS = 200    # pre-registered floor (spec 7.4)
ONROAD_MIN_STOPS_PER_ARM = 150    # pre-registered power rule (spec 7.4)
BOOTSTRAP_RESAMPLES = 4000
BOOTSTRAP_SEED = 20260609

# Pre-registered metric set (spec 7.4); continuous deltas are (B - A): negative = B better for
# the "smaller is better" metrics, which is every continuous metric except min_a (closer to 0 =
# gentler) -- min_a is compared on magnitude via the `transform` hook below.
CONTINUOUS_METRICS: tuple[str, ...] = ("end_jerk", "min_a", "rollout_m", "hold_gap_m", "time_to_standstill_s")
BINARY_METRICS: tuple[str, ...] = ("harsh", "leapfrog")

_NORMAL = NormalDist()


def _z(p: float) -> float:
  return _NORMAL.inv_cdf(p)


def _two_sided_p_from_z(z: float) -> float:
  return 2.0 * (1.0 - _NORMAL.cdf(abs(z)))


def wilcoxon_signed_rank(deltas: Sequence[float]) -> dict[str, float]:
  """Wilcoxon signed-rank, normal approximation with tie correction; zero deltas dropped
  (standard practice). Returns w_plus, z, p_two_sided, n_nonzero."""
  d = np.asarray([x for x in deltas if x != 0.0], dtype=float)
  n = len(d)
  if n == 0:
    return {"w_plus": 0.0, "z": 0.0, "p_two_sided": 1.0, "n_nonzero": 0}
  ranks = _rank_with_ties(np.abs(d))
  w_plus = float(np.sum(ranks[d > 0]))
  mean_w = n * (n + 1) / 4.0
  # tie correction on the variance
  _, counts = np.unique(np.abs(d), return_counts=True)
  tie_term = float(np.sum(counts ** 3 - counts)) / 48.0
  var_w = n * (n + 1) * (2 * n + 1) / 24.0 - tie_term
  if var_w <= 0.0:
    return {"w_plus": w_plus, "z": 0.0, "p_two_sided": 1.0, "n_nonzero": n}
  z = (w_plus - mean_w) / math.sqrt(var_w)
  return {"w_plus": w_plus, "z": z, "p_two_sided": _two_sided_p_from_z(z), "n_nonzero": n}


def _rank_with_ties(values: np.ndarray) -> np.ndarray:
  order = np.argsort(values, kind="mergesort")
  ranks = np.empty(len(values), dtype=float)
  ranks[order] = np.arange(1, len(values) + 1, dtype=float)
  # average ranks over ties
  unique, inverse = np.unique(values, return_inverse=True)
  for idx in range(len(unique)):
    members = inverse == idx
    if np.count_nonzero(members) > 1:
      ranks[members] = float(np.mean(ranks[members]))
  return ranks


def mann_whitney_u(x: Sequence[float], y: Sequence[float]) -> dict[str, float]:
  """Mann-Whitney U, normal approximation with tie correction."""
  x = np.asarray(x, dtype=float)
  y = np.asarray(y, dtype=float)
  n1, n2 = len(x), len(y)
  if n1 == 0 or n2 == 0:
    return {"u": 0.0, "z": 0.0, "p_two_sided": 1.0}
  combined = np.concatenate([x, y])
  ranks = _rank_with_ties(combined)
  r1 = float(np.sum(ranks[:n1]))
  u1 = r1 - n1 * (n1 + 1) / 2.0
  mean_u = n1 * n2 / 2.0
  n = n1 + n2
  _, counts = np.unique(combined, return_counts=True)
  tie_term = float(np.sum(counts ** 3 - counts)) / (n * (n - 1)) if n > 1 else 0.0
  var_u = n1 * n2 / 12.0 * ((n + 1) - tie_term)
  if var_u <= 0.0:
    return {"u": u1, "z": 0.0, "p_two_sided": 1.0}
  z = (u1 - mean_u) / math.sqrt(var_u)
  return {"u": u1, "z": z, "p_two_sided": _two_sided_p_from_z(z)}


def mcnemar_exact(b01: int, b10: int) -> dict[str, float]:
  """Exact two-sided McNemar on the discordant pairs (binomial p = 0.5)."""
  n = b01 + b10
  if n == 0:
    return {"discordant": 0, "p_two_sided": 1.0}
  k = min(b01, b10)
  tail = sum(math.comb(n, i) for i in range(k + 1)) / (2 ** n)
  p = min(1.0, 2.0 * tail)
  return {"discordant": n, "p_two_sided": p}


def bca_bootstrap_ci(values: Sequence[float], stat_fn: Callable[[np.ndarray], float],
                     n_boot: int = BOOTSTRAP_RESAMPLES, alpha: float = ALPHA,
                     seed: int = BOOTSTRAP_SEED) -> tuple[float, float]:
  """Bias-corrected and accelerated bootstrap CI of stat_fn over values."""
  data = np.asarray(values, dtype=float)
  n = len(data)
  if n == 0:
    return (float("nan"), float("nan"))
  if n == 1:
    return (float(data[0]), float(data[0]))
  rng = np.random.default_rng(seed)
  theta_hat = float(stat_fn(data))
  boot = np.empty(n_boot, dtype=float)
  idx = rng.integers(0, n, size=(n_boot, n))
  for i in range(n_boot):
    boot[i] = stat_fn(data[idx[i]])
  # bias correction
  prop = float(np.mean(boot < theta_hat))
  prop = min(max(prop, 1.0 / (n_boot + 1)), 1.0 - 1.0 / (n_boot + 1))
  z0 = _z(prop)
  # acceleration via jackknife
  jack = np.empty(n, dtype=float)
  for i in range(n):
    jack[i] = stat_fn(np.delete(data, i))
  jack_mean = float(np.mean(jack))
  num = float(np.sum((jack_mean - jack) ** 3))
  den = 6.0 * float(np.sum((jack_mean - jack) ** 2)) ** 1.5
  a = num / den if den > 0.0 else 0.0
  lo_q = _NORMAL.cdf(z0 + (z0 + _z(alpha / 2.0)) / (1.0 - a * (z0 + _z(alpha / 2.0))))
  hi_q = _NORMAL.cdf(z0 + (z0 + _z(1.0 - alpha / 2.0)) / (1.0 - a * (z0 + _z(1.0 - alpha / 2.0))))
  lo_q = min(max(lo_q, 0.0), 1.0)
  hi_q = min(max(hi_q, 0.0), 1.0)
  return (float(np.quantile(boot, lo_q)), float(np.quantile(boot, hi_q)))


def mde_paired(deltas: Sequence[float], alpha: float = ALPHA, power: float = POWER) -> float:
  """Minimum detectable mean paired delta at the observed n (normal/paired-t approximation):
  MDE = (z_{1-a/2} + z_{power}) * sd(delta) / sqrt(n)."""
  d = np.asarray(deltas, dtype=float)
  n = len(d)
  if n < 2:
    return float("inf")
  sd = float(np.std(d, ddof=1))
  if sd == 0.0:
    return 0.0
  return (_z(1.0 - alpha / 2.0) + _z(power)) * sd / math.sqrt(n)


def required_n_paired(observed_delta: float, deltas: Sequence[float],
                      alpha: float = ALPHA, power: float = POWER) -> int | None:
  """n required for the OBSERVED delta to be detectable at the pre-registered power."""
  d = np.asarray(deltas, dtype=float)
  if len(d) < 2 or observed_delta == 0.0:
    return None
  sd = float(np.std(d, ddof=1))
  if sd == 0.0:
    return len(d)
  n = ((_z(1.0 - alpha / 2.0) + _z(power)) * sd / abs(observed_delta)) ** 2
  return int(math.ceil(n))


def mde_mcnemar(n_pairs: int, discordant_rate: float = 0.10,
                alpha: float = ALPHA, power: float = POWER) -> float:
  """MDE on the binary-rate difference for McNemar at n pairs, given the observed (or assumed)
  discordant-pair rate: delta = (z_{1-a/2} + z_{power}) * sqrt(psi / n) where psi is the
  discordant rate (Connor's approximation)."""
  if n_pairs <= 0 or discordant_rate <= 0.0:
    return float("inf")
  return (_z(1.0 - alpha / 2.0) + _z(power)) * math.sqrt(discordant_rate / n_pairs)


@dataclass
class MetricVerdict:
  metric: str
  kind: str                 # 'continuous' | 'binary'
  n: int
  mde_at_n: float           # MANDATORY (spec 7.4)
  status: str               # 'improved' | 'regressed' | 'no_significant_change' | 'refused_insufficient_power'
  mean_delta: float | None = None
  median_delta: float | None = None
  ci_mean: tuple[float, float] | None = None
  ci_median: tuple[float, float] | None = None
  p_value: float | None = None
  rate_a: float | None = None
  rate_b: float | None = None
  discordant_a_only: int | None = None
  discordant_b_only: int | None = None
  required_n_for_observed_delta: int | None = None
  notes: list[str] = field(default_factory=list)


def event_key(record: dict[str, Any]) -> tuple:
  """Stable event identity: the spec-7.1 (route, seg, hold_mono_ns) key when present, the legacy
  (route, event_id) pair otherwise."""
  key = record.get("key")
  if isinstance(key, dict) and "route" in key:
    return (str(key.get("route")), int(key.get("seg", -1)), int(key.get("hold_mono_ns", -1)))
  return (str(record.get("route", "")), int(record.get("event_id", -1)))


def join_paired(events_a: list[dict[str, Any]], events_b: list[dict[str, Any]]) -> list[tuple[dict, dict]]:
  by_key_b = {event_key(e): e for e in events_b}
  pairs = []
  for ea in events_a:
    eb = by_key_b.get(event_key(ea))
    if eb is not None:
      pairs.append((ea, eb))
  return pairs


def _metric_value(event: dict[str, Any], metric: str) -> float | None:
  value = event.get(metric)
  if value is None:
    metrics = event.get("metrics") or event.get("metrics_10hz_compat") or {}
    if isinstance(metrics, dict):
      value = metrics.get(metric)
  if value is None:
    return None
  try:
    return float(value)
  except (TypeError, ValueError):
    return None


def paired_continuous_verdict(metric: str, pairs: list[tuple[dict, dict]], floor_n: int) -> MetricVerdict:
  deltas = []
  for ea, eb in pairs:
    va, vb = _metric_value(ea, metric), _metric_value(eb, metric)
    if va is None or vb is None or not (math.isfinite(va) and math.isfinite(vb)):
      continue
    if metric == "min_a":
      va, vb = abs(va), abs(vb)  # compare braking magnitude (deeper = worse)
    deltas.append(vb - va)
  n = len(deltas)
  mde = mde_paired(deltas)
  if n < floor_n:
    observed = float(np.mean(deltas)) if deltas else 0.0
    return MetricVerdict(
      metric=metric, kind="continuous", n=n, mde_at_n=mde,
      status="refused_insufficient_power",
      mean_delta=observed if deltas else None,
      median_delta=float(np.median(deltas)) if deltas else None,
      required_n_for_observed_delta=required_n_paired(observed, deltas),
      notes=[f"n={n} below pre-registered floor {floor_n}; verdict refused (spec 7.4)"],
    )
  arr = np.asarray(deltas, dtype=float)
  wil = wilcoxon_signed_rank(deltas)
  ci_mean = bca_bootstrap_ci(arr, lambda x: float(np.mean(x)))
  ci_median = bca_bootstrap_ci(arr, lambda x: float(np.median(x)))
  significant = wil["p_two_sided"] < ALPHA and not (ci_median[0] <= 0.0 <= ci_median[1])
  if significant:
    status = "improved" if float(np.median(arr)) < 0.0 else "regressed"
  else:
    status = "no_significant_change"
  return MetricVerdict(
    metric=metric, kind="continuous", n=n, mde_at_n=mde, status=status,
    mean_delta=float(np.mean(arr)), median_delta=float(np.median(arr)),
    ci_mean=ci_mean, ci_median=ci_median, p_value=wil["p_two_sided"],
  )


def paired_binary_verdict(metric: str, pairs: list[tuple[dict, dict]], floor_n: int) -> MetricVerdict:
  b_only = a_only = 0
  count_a = count_b = 0
  n = 0
  for ea, eb in pairs:
    va, vb = ea.get(metric), eb.get(metric)
    if va is None or vb is None:
      continue
    va, vb = bool(va), bool(vb)
    n += 1
    count_a += int(va)
    count_b += int(vb)
    if vb and not va:
      b_only += 1
    elif va and not vb:
      a_only += 1
  discordant_rate = (a_only + b_only) / n if n else 0.0
  mde = mde_mcnemar(n, max(discordant_rate, 0.02))
  rate_a = count_a / n if n else 0.0
  rate_b = count_b / n if n else 0.0
  if n < floor_n:
    return MetricVerdict(
      metric=metric, kind="binary", n=n, mde_at_n=mde,
      status="refused_insufficient_power", rate_a=rate_a, rate_b=rate_b,
      discordant_a_only=a_only, discordant_b_only=b_only,
      notes=[f"n={n} below pre-registered floor {floor_n}; verdict refused (spec 7.4)"],
    )
  test = mcnemar_exact(a_only, b_only)
  if test["p_two_sided"] < ALPHA:
    status = "regressed" if b_only > a_only else "improved"
  else:
    status = "no_significant_change"
  return MetricVerdict(
    metric=metric, kind="binary", n=n, mde_at_n=mde, status=status,
    rate_a=rate_a, rate_b=rate_b, discordant_a_only=a_only, discordant_b_only=b_only,
    p_value=test["p_two_sided"],
  )


def compare_paired(events_a: list[dict[str, Any]], events_b: list[dict[str, Any]],
                   floor_n: int = SIM_AB_MIN_PAIRED_EVENTS) -> dict[str, Any]:
  """Sim A/B comparison (spec 7.4): A = baseline (e.g. legacy forest), B = candidate (e.g. V2)."""
  pairs = join_paired(events_a, events_b)
  verdicts = [paired_continuous_verdict(m, pairs, floor_n) for m in CONTINUOUS_METRICS]
  verdicts += [paired_binary_verdict(m, pairs, floor_n) for m in BINARY_METRICS]
  any_refused = any(v.status == "refused_insufficient_power" for v in verdicts)
  any_regressed = any(v.status == "regressed" for v in verdicts)
  return {
    "mode": "sim_ab_paired",
    "paired_events": len(pairs),
    "floor_n": floor_n,
    "alpha": ALPHA,
    "power": POWER,
    "metrics": [asdict(v) for v in verdicts],
    "verdict": ("refused_insufficient_power" if any_refused
                else "regressed" if any_regressed else "no_regression"),
  }


def all_zero_isd(events: list[dict[str, Any]]) -> bool:
  """Cross-era comparability precondition (eval.md section 3.1, decided 2026-06-12): True iff
  EVERY event records entry.isd_m == 0. Missing or non-numeric isd_m fails the precondition
  (strict by default -- comparability must be proven, not assumed). Empty arms fail it too."""
  if not events:
    return False
  for event in events:
    entry = event.get("entry")
    isd = entry.get("isd_m") if isinstance(entry, dict) else None
    try:
      if isd is None or float(isd) != 0.0:
        return False
    except (TypeError, ValueError):
      return False
  return True


def stratum_of(event: dict[str, Any], ignore_signals_version: bool = False) -> str:
  """On-road stratification (spec 7.4): approach-speed bin x lead/no-lead x signals_version.

  `ignore_signals_version` implements the cross-era comparison rule (eval.md section 3.1,
  decided 2026-06-12): signals_version only changes published lead-gap semantics through a
  nonzero IncreasedStoppedDistance, so when every event in BOTH arms records entry.isd_m == 0
  the eras are physically comparable and signals_version is dropped from the key. compare_onroad
  sets this from the arm-level all_zero_isd() precondition -- never per event. Gates do not
  consume these strata and remain same-era only."""
  entry = event.get("entry") if isinstance(event.get("entry"), dict) else event
  # Store-shaped records (eval.md section 1) carry approach speed only under entry.v_approach;
  # without this lookup every store record read v=0.0 and stratification degenerated to
  # lead/no-lead x sv (fixed 2026-06-12, eval.md section 3 note on archived-report comparability).
  v = (_metric_value(event, "v_approach") or _metric_value(event, "entry_speed_mps")
       or _metric_value(entry, "v_approach") or 0.0)
  speed_bin = "<1" if v < 1.0 else "1-2" if v <= 2.0 else ">2"
  lead = "lead" if (entry.get("lead_entry_gap_m") not in (None, 0.0) or event.get("lead_distance_stop_entry_m")) else "no_lead"
  if ignore_signals_version:
    return f"v{speed_bin}|{lead}"
  sv = int(event.get("signals_version", 1) or 1)
  return f"v{speed_bin}|{lead}|sv{sv}"


def compare_onroad(events_before: list[dict[str, Any]], events_after: list[dict[str, Any]],
                   metric: str = "end_jerk", floor_per_arm: int = ONROAD_MIN_STOPS_PER_ARM) -> dict[str, Any]:
  """Stratified before/after comparison on DIFFERENT events (spec 7.4).

  Cross-era rule (eval.md section 3.1, decided 2026-06-12): signals_version is dropped from the
  stratum key IFF every event in BOTH arms has entry.isd_m == 0 (single arm-level precondition).
  Any nonzero or missing isd_m in either arm keeps the strict behavior, under which cross-era
  arms occupy disjoint strata and the comparison is refused-by-construction (pooled delta NaN)."""
  n_before, n_after = len(events_before), len(events_after)
  ignore_sv = all_zero_isd(events_before) and all_zero_isd(events_after)
  sv_before = sorted({int(e.get("signals_version", 1) or 1) for e in events_before})
  sv_after = sorted({int(e.get("signals_version", 1) or 1) for e in events_after})
  if ignore_sv:
    rule_note = ("CROSS-ERA RULE ENGAGED: every event in both arms has entry.isd_m == 0, so signals_version is dropped "
                 + "from the stratum key and eras pool (eval.md section 3.1, decided 2026-06-12). Power floors are unchanged.")
  else:
    rule_note = ("strict stratification: signals_version retained in the stratum key (a nonzero or missing entry.isd_m is present); "
                 + "cross-era arms occupy disjoint strata")
  cross_era_rule = {
    "engaged": ignore_sv,
    "signals_versions_before": sv_before,
    "signals_versions_after": sv_after,
    "note": rule_note,
  }
  strata: dict[str, dict[str, list[float]]] = {}
  for arm, events in (("before", events_before), ("after", events_after)):
    for event in events:
      value = _metric_value(event, metric)
      if value is None or not math.isfinite(value):
        continue
      strata.setdefault(stratum_of(event, ignore_signals_version=ignore_sv), {"before": [], "after": []})[arm].append(value)

  stratum_rows = []
  pooled_deltas: list[float] = []
  pooled_weights: list[int] = []
  rng = np.random.default_rng(BOOTSTRAP_SEED)
  boot_pool = np.zeros(BOOTSTRAP_RESAMPLES)
  total_weight = 0
  for name, arms in sorted(strata.items()):
    xb, xa = arms["before"], arms["after"]
    if not xb or not xa:
      stratum_rows.append({"stratum": name, "n_before": len(xb), "n_after": len(xa), "skipped": True})
      continue
    mw = mann_whitney_u(xb, xa)
    delta = float(np.median(xa) - np.median(xb))
    weight = min(len(xb), len(xa))
    stratum_rows.append({
      "stratum": name, "n_before": len(xb), "n_after": len(xa),
      "median_delta": delta, "p_two_sided": mw["p_two_sided"], "skipped": False,
    })
    pooled_deltas.append(delta)
    pooled_weights.append(weight)
    xb_arr, xa_arr = np.asarray(xb), np.asarray(xa)
    for i in range(BOOTSTRAP_RESAMPLES):
      rb = xb_arr[rng.integers(0, len(xb_arr), len(xb_arr))]
      ra = xa_arr[rng.integers(0, len(xa_arr), len(xa_arr))]
      boot_pool[i] += weight * float(np.median(ra) - np.median(rb))
    total_weight += weight

  if total_weight > 0:
    boot_pool /= total_weight
    pooled = float(np.average(pooled_deltas, weights=pooled_weights))
    ci = (float(np.quantile(boot_pool, ALPHA / 2)), float(np.quantile(boot_pool, 1 - ALPHA / 2)))
  else:
    pooled, ci = float("nan"), (float("nan"), float("nan"))

  before_vals = [v for arms in strata.values() for v in arms["before"]]
  baseline_median = float(np.median(before_vals)) if before_vals else float("nan")
  # pre-registered power rule: >= 150/arm resolves a 20% relative median change at 80% power
  mde_relative = 0.20 * baseline_median if math.isfinite(baseline_median) else float("inf")
  refused = min(n_before, n_after) < floor_per_arm
  if refused:
    status = "refused_insufficient_power"
  elif math.isfinite(ci[0]) and (ci[0] > 0.0 or ci[1] < 0.0):
    status = "regressed" if pooled > 0.0 else "improved"
  else:
    status = "no_significant_change"
  return {
    "mode": "onroad_stratified",
    "metric": metric,
    "n_before": n_before,
    "n_after": n_after,
    "floor_per_arm": floor_per_arm,
    "alpha": ALPHA,
    "power": POWER,
    "mde_at_n": mde_relative,
    "pooled_median_delta": pooled,
    "pooled_ci": ci,
    "strata": stratum_rows,
    "cross_era_rule": cross_era_rule,
    "verdict": status,
    "required_n_per_arm": floor_per_arm if refused else None,
  }


def _load_events(path: Path) -> list[dict[str, Any]]:
  if path.suffix == ".jsonl":
    return [json.loads(line) for line in path.read_text().splitlines() if line.strip()]
  payload = json.loads(path.read_text())
  if isinstance(payload, list):
    return payload
  for key in ("events", "event_rows"):
    if isinstance(payload.get(key), list):
      return payload[key]
  raise ValueError(f"no event list found in {path}")


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
  parser = argparse.ArgumentParser(description="Paired/stratified stopping statistics with mandatory MDE + refusal (spec 7.4)")
  parser.add_argument("--mode", choices=["sim-ab", "onroad"], default="sim-ab")
  parser.add_argument("--a-json", required=True, help="Baseline events (sim-ab: controller A; onroad: before)")
  parser.add_argument("--b-json", required=True, help="Candidate events (sim-ab: controller B; onroad: after)")
  parser.add_argument("--metric", default="end_jerk", help="Primary metric for onroad mode")
  parser.add_argument("--floor-n", type=int, default=None, help="Override the pre-registered floor (testing only)")
  parser.add_argument("--output-json", default=None)
  return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
  args = parse_args(argv)
  try:
    events_a = _load_events(Path(args.a_json).expanduser())
    events_b = _load_events(Path(args.b_json).expanduser())
  except (OSError, ValueError) as exc:
    print(f"[paired-stats] failed to load inputs: {exc}", file=sys.stderr)
    return 3

  if args.mode == "sim-ab":
    floor = args.floor_n if args.floor_n is not None else SIM_AB_MIN_PAIRED_EVENTS
    report = compare_paired(events_a, events_b, floor_n=floor)
  else:
    floor = args.floor_n if args.floor_n is not None else ONROAD_MIN_STOPS_PER_ARM
    report = compare_onroad(events_a, events_b, metric=args.metric, floor_per_arm=floor)

  print(f"[paired-stats] mode={report['mode']} verdict={report['verdict']}")
  for row in report.get("metrics", []):
    line = (f"[paired-stats] {row['metric']}: n={row['n']} mde_at_n={row['mde_at_n']:.4f} status={row['status']}")
    if row.get("required_n_for_observed_delta"):
      line += f" required_n_for_observed_delta={row['required_n_for_observed_delta']}"
    print(line)
  if report["mode"] == "onroad_stratified":
    print(f"[paired-stats] {report['metric']}: n_before={report['n_before']} n_after={report['n_after']} "
          + f"mde_at_n={report['mde_at_n']:.4f} pooled_delta={report['pooled_median_delta']:.4f}")
    rule = report["cross_era_rule"]
    if rule["engaged"]:
      print(f"[paired-stats] NOTE: {rule['note']} "
            + f"(signals_versions before={rule['signals_versions_before']} after={rule['signals_versions_after']})")
  if report["verdict"] == "refused_insufficient_power":
    print("[paired-stats] VERDICT REFUSED: below the pre-registered power floor (see required n above)", file=sys.stderr)

  if args.output_json:
    out = Path(args.output_json).expanduser()
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")
    print(f"[paired-stats] output_json={out}")

  if report["verdict"] == "refused_insufficient_power":
    return 2
  if report["verdict"] == "regressed":
    return 1
  return 0


if __name__ == "__main__":
  raise SystemExit(main())
