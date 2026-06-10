# Stopping Evaluation Methodology

How stopping behavior is measured, scored, compared, and gated. Tools live in `tools/stopping/`
(commands: `tools/stopping/README.md`). Principle: **the sim develops, the measurement promotes** —
the AR(1) plant sim acts as a promotion gate exactly once (the section-5 similarity gate, dual
plants); afterwards it is demoted to development smoke and every tuning commit promotes on measured
paired statistics only.

## 1. Event store (`build_event_store.py`)

Machine-local, JSONL + npz: `~/.comma/stopping_behavior/event_store/events.jsonl` with per-event
traces at `events/<key>.npz` and a `manifest.json` (device fetch report). Source order: rlogs
first (`logMessage` + 20 Hz plan + 100 Hz car signals; qlog plan is 2 Hz-aliased — useless for
dropout analysis), qlog fallback tagged `rate_class='qlog10'`. Detection reuses the calibrated
hybrid detector + metric definitions from `analyze_stopping_behavior.py` (rate-aware merge
window). Engagement truth comes from `selfdriveState` (the `controlsState.enabled` relic decodes
always-False on this fork); only engaged stop events are stored.

Record shape:

```json
{"key": {"route": "...", "seg": 12, "hold_mono_ns": 1234567890},
 "detector": "hybrid", "schema_version": 1,
 "signals_version": 1, "telemetry_version": 1,
 "controller_commit": "abc123", "accel_cmd_source": "carControl",
 "entry": {"v_approach": 3.1, "lead_entry_gap_m": 7.2, "explicit_target": true, "isd_m": 0.0},
 "metrics_100hz": {"end_stop_jerk": 0.0, "end_stop_accel_step": 0.0, "min_a_ego": 0.0,
                   "max_cmd_jerk": 0.0, "rollout_from_2mps_m": 0.0, "final_lead_gap_m": 0.0,
                   "rebound_mps": 0.0, "unexpected_accel": 0.0, "hard_decel_duration_s": 0.0,
                   "time_to_standstill_s": 0.0},
 "metrics_10hz_compat": {"same definitions, decimated to 10 Hz"},
 "trace_ref": "events/<key>.npz"}
```

- **Stable keys** `(route, seg, hold_mono_ns)` replace positional event ids — paired comparisons
  survive re-scans.
- **Dual-rate metric blocks are mandatory.** `metrics_10hz_compat` keeps the historical
  2,097-event corpus and the 736-event / 0-leapfrog baseline comparable forever. 10 Hz jerk
  systematically understates true jerk, so 100 Hz and 10 Hz values are never mixed in one
  comparison.
- **Era flags:** `signals_version` ≥ 2 marks post-dRel-flip routes; `telemetry_version` ≥ 2 marks
  post-telemetry-fix routes (the builder then sources `accel_cmd` from
  `carOutput.actuatorsOutput.accel` — the sent value — and records `accel_cmd_source`; v1 routes
  keep `carControl.actuators.accel` with the 4 s post-engagement exclusion). Both flags are
  CLI-declared at build time (pre-flip logs carry no in-log marker); `controller_commit` and the
  auto-detected `controller_version` (legacy/v2 from `stopping_shadow` payloads) support forensic
  reconstruction of mixed eras.

## 2. Frozen scoring (`scoring_config.py`)

Single checked-in dataclass, JSON-serialized into every gate artifact. **Generated from the
operative code, not transcribed from prose**: it freezes `check_harsh_stops.classify_event`'s
flag-level logic plus the operative cycle invocation, and `test_scoring_config.py` diffs it
against a recorded `classify_event` run.

- **Leapfrog is the operative OR-of-flags predicate**: `leapfrog_rebound_signal` or
  `leapfrog_rebound_should_stop` alone flag (a rebound-only event IS a leapfrog), plus the
  rebound∧unexpected_accel combination, `reaccel_before_hold`, and the two cycle-forced flags
  (stop-signal-drop / exit-stop). Anything else would make the historical 0-leapfrog baseline
  incomparable.
- **Harsh seeds** include the entry-side flags from the operative cycle config (entry_stop_jerk
  0.35, entry_cmd_jerk 0.50, entry_accel_step 0.08) alongside end_jerk > 0.35 ∨ cmd_jerk > 1.0 ∨
  accel_step > 0.08 ∨ min_a < −1.05 ∨ hard-decel > 0.75 s ∨ tight-lead-hold < 1.65 m ∨ far-lead
  spike.
- Quality buckets preserve the `benchmark_controller_variants.py` cutoffs; rollout budgets
  2.0 m (no target) / 1.25 m (explicit target); hold-gap contract 2.5–5.0 m absolute.
- Any threshold change = config version bump + re-baseline note here.
- `check_harsh_stops.py` and the cycle read defaults from this config (CLI flags are explicit
  overrides only, retained until the cleanup commit reworks the test files).

### 2.1 100 Hz threshold re-baselining (procedure; not yet executed)

When the first sufficiently large 100 Hz corpus scan lands: on the calibration set of events
having BOTH metric blocks, choose 100 Hz harsh/jerk thresholds such that bucket populations
(perfect/good/mediocre/poor/hard_fail + harsh flags) match the 10 Hz assignments on the same
events within ±2% per bucket. Document the chosen values + matching report in this file and
version-bump `scoring_config`. Until then, historical comparisons stay on `metrics_10hz_compat`.

## 3. Statistics (`paired_stats.py`) — MDE-stating, refusal-capable

- **Sim A/B (controller vs controller, same events):** per-event paired deltas on pre-registered
  metrics (end_jerk, min_a, rollout, hold_gap, harsh flag, leapfrog flag); Wilcoxon signed-rank +
  BCa bootstrap 95% CI of the mean/median delta; binary metrics via McNemar. Pre-registered
  floor: n ≥ 200 paired events.
- **On-road before/after (different events):** stratified by approach-speed bin × lead/no-lead ×
  signals_version; Mann–Whitney per stratum + stratified bootstrap of the pooled difference.
  Power rule: ≥ 150 stops/arm to detect a 20% relative median-end-jerk change at 80% power; rare
  binary rates per McNemar/Fisher power formula.
- **Mandatory verdict fields:** every report prints `n` and `mde_at_n`; below the floor it
  REFUSES the verdict and prints the n required for the observed delta (exit code 2
  "insufficient data"). Exit-code protocol 0/1/2/3 preserved for automation.

## 4. Estimator-equivalence artifact (`estimator_equivalence.py`)

Pre-gate requirement for the V2 disturbance estimator (risk R9): on the event store, (i) LPF
threshold-crossing onset within ±0.2 s of the legacy single-frame trigger on ≥ 90% of
push-disturbance events, and (ii) ≥ 90% of push events with ≥ 80% Jaccard release-inhibit window
overlap. Emitted as a mandatory row of the similarity report. Replay is gated to
enabled-AND-should_stop spans (the estimator only runs in stopping authority).

Measured (129-event store, 23 push events): `DIST_LPF_TAU_S = 0.30` fails badly (13.0% / 13.0%);
no tau in {0.02..0.30} passes; **tau = 0.0 (the documented kill-switch bypass == legacy
single-frame semantics) passes 100% / 100% and is the deployed value.** Re-tuning a nonzero tau
is future work that must re-pass this artifact before any gate re-run.

## 5. The similarity gate (`similarity_gate.py`) — precondition for the V2 flip

Replays BOTH controllers (legacy forest, V2 facade) closed-loop through the plant sim on:
(1) all 5 pinned holdout routes' events (`tools/stopping/holdout_routes.txt`), (2) a stratified
event-store sample (strata: approach speed {<1, 1–2, >2 m/s} × {explicit target, stopped lead,
no target} × {clean, push-disturbance}), (3) every `stop_scenarios.py` fixture. Run on **two
plants** (frozen 20260514 + newest refit, both archived in `docs/stopping/archive/`); verdicts
must agree on both. **Integrated-path requirement:** the gate also replays LongControl-with-V2
(state machine + the single arbiter + facade wired exactly as on-car) on the dropout-hold fixture
set — integration wiring must never debut on the road.

**Tier 1 — outcome envelope (PASS/FAIL, all required, both plants):**

| Metric | Bound |
|---|---|
| harsh (frozen config, incl. entry-side flags) | no event harsh-in-V2-but-not-forest |
| leapfrog (operative OR-of-flags definition) | count(V2) ≤ count(forest) on every stratum |
| predicted rollout delta | \|Δ\| ≤ 0.15 m at p95 |
| predicted final hold-gap delta (lead events) | \|Δ\| ≤ 0.10 m at p95 |
| end_jerk paired delta | 95% CI of median ⊆ [−0.05, +0.03] m/s³ |
| time-to-standstill delta | ≤ +0.5 s at p95 |
| estimator equivalence (section 4) | passed and attached |

**Tier 2 — command-trace diagnostics (budgeted, NOT pass/fail):** target RMS(a_cmd_V2 −
a_cmd_forest) ≤ 0.05 m/s² for ≥ 90% of events; every event with RMS > 0.10 or max|Δ| > 0.30 gets
a mandatory written classification. Trace divergence with clean Tier-1 outcomes is expected.

**Triage taxonomy (mandatory per flagged event).** Root causes RC1 shouldStop flicker / RC2
published-dRel lie / RC3 no reference trajectory / RC4 no disturbance estimator. Classes:
**A** — forest artifact whose root cause Phase 0/V2 legitimately fixed (accept + document;
quirk-cap-induced divergence downstream of the facade is pre-registered Class A);
**B** — vehicle-calibration mismatch (move the named parameter toward its documented
preserve-group source value only — no free tuning against the sim; re-run the gate);
**C** — unexplained fidelity loss (BLOCKER: fix the mechanism or escalate; the flip cannot land
with open class-C events).

**Exit criterion (deterministic):** Tier 1 all green on both plants AND zero class-C events
remaining AND the triage table committed in `docs/stopping/archive/similarity_<date>.md` in the
same commit as the flip.

## 6. Gate status: 2026-06-10 run — NOT PASSED, V2 remains dark

Deck: 160 paired scenarios (129 event-store events incl. all 5 holdout routes + 31
`stop_scenarios` fixtures), dual plants, legacy-vs-V2, plus 4 integrated
LongControl-with-V2 dropout-hold fixtures. One bounded class-B parameter move was made first:
`DIST_LPF_TAU_S` 0.30 → 0.0 (section 4), flipping the estimator row to PASS on both plants.

- **Frozen plant (ref_20260514):** harsh PASS, end_jerk CI PASS, hold_gap p95 0.070 m PASS,
  integrated dropout-hold 4/4 PASS, estimator PASS. **FAIL** leapfrog (one stratum: forest 14 vs
  V2 15), **FAIL** rollout p95 0.551 m, **FAIL** time-to-standstill p95 2.94 s.
- **Refit plant (refit_20260531):** harsh, leapfrog, rollout (p95 0.022 m), tts (p95 0.000 s),
  end_jerk, integrated 4/4, estimator all PASS. **FAIL** hold_gap p95 2.528 m (25/111 lead
  events, event-specific stop-position divergences up to ±9.5 m).
- **Tier 2:** 46.9% / 45.0% of events under the 0.05 RMS target (need 90%); 81 / 69 flagged
  events, all awaiting per-event written classification (class-C count 0, but unclassified events
  block the verdict by themselves).
- **Root-cause reading:** the frozen-plant failures are authority-collapse stall artifacts — the
  plant's documented gain inversion below ~0.21 m/s makes deeper braking RAISE its steady-state
  accel, and V2's tracker shares the sim's plant model so its closed-loop innovation is zero by
  construction (push/arrest machinery cannot fire in-sim), while the forest's RC3/RC4 lanes
  (keyed to the mismatched interp envelope) escape the band. No provenance-legal parameter move
  fixes this (a deepening move is provably counterproductive on the inverted plant). The refit
  hold_gap failure maps to no named parameter. **Escalated to the user per the class-C/R2 rule**:
  either re-scope the frozen-plant Tier-1 rows or supply per-event triage.
- **Integrated-path requirement itself PASSES** (wiring is not the blocker; envelope similarity
  is).
- Artifacts (machine-local; committed to `docs/stopping/archive/` only with the flip commit):
  `~/.comma/stopping_behavior/analysis/similarity_gate_postC_tau000.{json,triage.md}`,
  `~/.comma/stopping_behavior/event_store/estimator_equivalence_report.json`.

## 7. Post-flip measurement loop

After the flip and soak: `check_leapfrog_alignment.py` (re-pointed at `sim_replay` predictions,
stable event keys) is the only ongoing model-truthfulness loop. Every improvement commit changes
one named parameter and carries a `paired_stats` report (with MDE field) in the commit message.
The plant sim runs as development smoke only.
