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
                   "time_to_standstill_s": 0.0, "hold_acq_peak_cmd_jerk": null,
                   "approach_peak_decel_over_gap2m": null, "approach_required_decel_to_2m": null,
                   "approach_necessary": null, "settle_peak_meas_jerk": null,
                   "settle_meas_minus_sent_jerk": null},
 "metrics_10hz_compat": {"same definitions, decimated to 10 Hz"},
 "trace_ref": "events/<key>.npz"}
```

- **Stable keys** `(route, seg, hold_mono_ns)` replace positional event ids — paired comparisons
  survive re-scans.
- **Dual-rate metric blocks are mandatory.** `metrics_10hz_compat` keeps the historical
  2,097-event corpus and the 736-event / 0-leapfrog baseline comparable forever. 10 Hz jerk
  systematically understates true jerk, so 100 Hz and 10 Hz values are never mixed in one
  comparison.
- **`hold_acq_peak_cmd_jerk` is a NON-gating diagnostic** (`scoring_config.DiagnosticMetrics`,
  no version bump — only threshold changes require one): peak |d(accel_cmd)/dt| in the window
  [`enabled` rising edge with v_ego < 0.3 m/s, +2 s]. Measures engage-at-standstill / stop-and-go
  re-engage hold-acquisition ramp shape (hold-acquisition soften change, driveway route
  `00001702--dcdc5c3eea--0`, 2026-06-10); `null` for events without a low-speed engagement edge.
  Never read by `classify_event` or any gate predicate.
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
- **Cranked comfort criteria (version 2, 2026-06-13):** `unnecessary_harsh_approach` (gap-gated
  approach decel cap 0.5 m/s² with a kinematic-necessity exemption) is the GATING harsh flag;
  `harsh_terminal_grab` (measured settle-jerk cap 3.0 m/s³) was DEMOTED on 2026-06-13 to a
  PROVISIONAL NON-gating diagnostic (the metric is computed/recorded but does not contribute to
  the harsh verdict) — see §2.1.
- Any threshold change = config version bump + re-baseline note here (the version 2 bump is §2.1).
- `check_harsh_stops.py` and the cycle read defaults from this config (CLI flags are explicit
  overrides only, retained until the cleanup commit reworks the test files).

### 2.1 Cranked comfort requirement (version 2, 2026-06-13)

The user feels **two distinct harsh events per stop** and wants each measured, then the requirement
cranked. `scoring_config` version 1 → **2** (`CrankedComfortThresholds`) adds **one gating
harsh-classification criterion (P1, `unnecessary_harsh_approach`)** and **one provisional NON-gating
diagnostic (P2, `terminal_max_settle_meas_jerk`)**. Both metrics are computed by `build_event_store`
on the engaged + long-control-active Sample stream and scored on the **`metrics_100hz`** block
(rlog100 primary — 10 Hz decimation systematically understates jerk, spec 7.2, so the terminal
settle-jerk diagnostic in particular is only faithful at 100 Hz). The legacy 10 Hz-provenance rule
(§2 / §2.2) does **not** apply to these two; their thresholds belong to the 100 Hz block by
construction.

**P1 — unnecessary harsh approach (`unnecessary_harsh_approach`).** *Requirement:* during the
stopping phase, **while the lead gap is still comfortable (> 2.0 m), peak commanded decel must stay
≤ 0.5 m/s²** — UNLESS kinematically necessary to avoid the lead. The complaint is *unnecessary*
harsh braking: the controller brakes hard when gentle would have sufficed. The metric
(`approach_peak_decel_over_gap2m`, `build_event_store.approach_decel_over_gap2m`) is the peak
|accel_cmd| (most-negative command) over the masked stopping-phase window — samples that are
engaged + long-control-active AND have a present lead with `lead_d_rel_m > 2.0 m` AND
`v_ego > standstill`. The **engaged mask is mandatory**: without it the 13 human-braked
(0%-engaged) speed-detected stops dominate and pollute the metric. The command is the right signal
— ground truth shows it tracks measured aEgo within ~0.1 m/s² on all 7 engaged P1 stops, and it is
the controllable quantity; measured aEgo rides alongside as a diagnostic
(`approach_worst_meas_decel`).

  *Kinematic-exemption definition (encodes "no UNNECESSARY harsh approach braking"):* at the worst
  (most harsh) sample, with `closing = max(v_ego − lead_v, 0)`,

  ```
  required_decel = closing² / (2 · max(gap − 2.0, 0.1))
  necessary      = (peak_decel ≤ required_decel + 0.12)        # 0.12 m/s² margin spares borderline-kinematic events
  violation      ⟺ peak_decel > 0.5  AND  required_decel ≤ 0.5  # (required None ⇒ treated as 0 = unnecessary)
  ```

  `required_decel` is the decel needed to bleed the closing speed to zero before closing past the
  2 m boundary. The test keys on radar **closing-speed + available gap**, NOT a fixed
  speed/scenario, so it is principled and **robust across driving-model versions**. Verified on the
  two new StopReq-A routes: flags the 4 unnecessary events (db31#4, b3a0#5, b3a0#8, b3a0#12) and
  leaves the 2 necessary high-closing approaches (db31#6 reqDecel 1.22, b3a0#13 reqDecel 0.76)
  untouched.

**P2 — terminal disc-grab (`terminal_max_settle_meas_jerk`): PROVISIONAL NON-gating diagnostic
(demoted 2026-06-13).** *Intended requirement:* peak **MEASURED** settle jerk (`a_ego`) at the
first genuine standstill should stay ≤ **3.0 m/s³** (named constant `terminal_max_settle_meas_jerk`).
The metric (`settle_peak_meas_jerk`, `build_event_store.settle_meas_jerk`) is **still computed and
recorded** — peak |d(a_ego)/dt| over the settle window from ~0.6 s before the first sample reaching
the genuine-standstill band (`SETTLE_STANDSTILL_SPEED = 0.06` m/s) up to that first-standstill
sample, masked to engaged + long-control-active and **truncated at the first inactive frame after
being active** (same takeover-artifact guard as the hold-acquisition diagnostic). A companion
`settle_meas_minus_sent_jerk` records the stiction excess. **But `classify_event` no longer raises
`harsh_terminal_grab`**: P2 does **not** contribute to the harsh verdict or the quality bucket, the
same way `hold_acq_peak_cmd_jerk` is a non-gating diagnostic.

*Why non-gating (the metric is not trustworthy yet).* The felt grab is the brake pads biting at
v ≈ 0, but neither available channel can see it: (i) `a_ego` is **wheel-speed-derived** and
**quantizes/floors to ~0 at standstill** — the static-friction grab that the user feels leaves no
wheel signature, so the measured-jerk metric understates (often misses) it; (ii) under **StopReq-A
the SCC owns the final stop**, so the openpilot **command is also blind** to the actuator behavior
below the 0.04 m/s gate. Gating on a structurally blind metric is the exact anti-pattern this
project avoids, so P2 is a diagnostic until it can be measured faithfully. (P1, by contrast, is
command-measurable and validated, so it stays gating.)

*Concrete next step to make P2 measurable.* Wire an **IMU longitudinal-accel channel** into the
eval — either the **raw accelerometer (~101 Hz)** or **`livePose.accelerationDevice` (~20 Hz,
gravity-removed)**; neither is currently consumed by `analyze_stopping_behavior.load_samples`. An
inertial accel channel sees the static-friction grab at standstill that wheel-derived `a_ego`
floors away. **Then** re-derive the P2 metric off that channel, crank/iterate the cap, and only
then re-promote P2 to gating. *Data finding bounding the eventual fix:* the command-side P2 lever
is **largely exhausted** — commanded settle jerk is already ≤ 1.5 m/s³, and the felt excess is
actuator stiction beyond the command (median ~+2 to +3 m/s³, worst +3.12, +3.11). So the eventual
P2 fix likely leans on the **StopReq-A SCC handoff** (shaping/handing the terminal bite to the SCC
tail, which probes smooth at 0.6–0.9 m/s³ below 0.04 m/s), **not** deeper command caps. The P2
controller command-deepening cap stays (it measurably bounds command jerk and is safety-cleared);
only the eval gating flag was demoted.

**Corpus re-score (218-event store, scored on `metrics_100hz`, engaged @ `enabled_ratio ≥ 0.80`).**
P1 is the gating cranked requirement; P2 no longer contributes to the harsh verdict. P1 is
intentionally strict — the failure rate quantifies the gap to close. (P2 is shown for diagnostic
context only and is no longer a gate input.)

| Criterion | Gating? | Engaged failures | All-events failures |
|---|---|---|---|
| `unnecessary_harsh_approach` (P1) | **yes** | 68 / 131 (**51.9%**) | 78 / 218 (35.8%) |
| `terminal_max_settle_meas_jerk` (P2) | no (diagnostic) | 30 / 131 (22.9%) | 31 / 218 (14.2%) |
| P1 only (harsh verdict) | **yes** | 68 / 131 (**51.9%**) | 78 / 218 (35.8%) |

On the two new StopReq-A routes specifically (`gitCommit 390054594e`): `0000171e--5c66f4db31`
(6 events, 4 engaged) → 2 approach; `0000171f--45bcc6b3a0` (18 events, 9 engaged) → 7 approach.
P1 is a THRESHOLD change → `scoring_config` stays at version **2**. The P2 demotion is a
gating/labeling change (P2 was never a released gate), so it is a refinement **within** version 2,
not a version bump. Re-tuning P1 downward as the user iterates is a 100 Hz threshold change.

### 2.2 100 Hz threshold re-baselining (procedure; not yet executed)

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
  binary rates per McNemar/Fisher power formula. Cross-era pooling: §3.1.
  **Stratum-definition fix (2026-06-12):** `stratum_of` previously never read
  `entry.v_approach` (the only place store records carry approach speed), so every store-shaped
  event landed in the `v<1` bin and stratification degenerated to lead/no-lead × sv. On-road
  reports archived before this fix (including `paired_onroad_end_stop_jerk_crossera_20260612`)
  used the degenerate strata and are **not stratum-comparable** with later reports — rerun the
  comparison rather than diffing report files across the fix.
- **Mandatory verdict fields:** every report prints `n` and `mde_at_n`; below the floor it
  REFUSES the verdict and prints the n required for the observed delta (exit code 2
  "insufficient data"). Exit-code protocol 0/1/2/3 preserved for automation.

### 3.1 Cross-era comparison rule (decided 2026-06-12)

**Default (strict):** `signals_version` is part of the on-road stratum key, so telemetry-era arms
(v1 vs v2) occupy disjoint strata and every cross-era comparison is refused-by-construction —
all strata skipped, pooled delta NaN (this is what the 2026-06-12 old-vs-new run hit).

**Physical rationale (this car):** `signals_version` ≥ 2 marks the dRel-honesty flip
(`PUBLISH_TRUE_LEAD_DISTANCE`), which changes published lead-gap semantics ONLY through a nonzero
`IncreasedStoppedDistance` — published gap and true gap differ by exactly ISD. Every event store
record carries `entry.isd_m`, and the device runs ISD = 0.0. Therefore when every event in both
arms has `isd_m == 0`, the v1 and v2 lead-gap semantics coincide bit-for-bit and the eras are
physically comparable.

**Operative rule (`paired_stats.compare_onroad`):** `signals_version` is dropped from the stratum
key IFF every event in BOTH arms records `entry.isd_m == 0` — a single arm-level precondition
(`all_zero_isd`), never per-event. Missing or non-numeric `isd_m` fails the precondition: any
nonzero-ISD event in either arm keeps the strict refused-by-construction behavior. The report
carries a `cross_era_rule` block and the CLI prints a loud NOTE whenever the rule engages.

**Scope and non-effects:**
- Measurement comparisons only. **Gates remain same-era**: the similarity gate's strata come from
  `sim_replay.stratum_for_entry` (controller vs controller on one deck) and do not consume
  `stratum_of`; an explicit comment marks this at its `paired_stats` import.
- Power floors are unchanged — the rule lets strata pool; it does not relax the ≥ 150/arm floor.
- Comparability/diagnostics rule, not a threshold change: no `scoring_config` version bump.

**First application (2026-06-12 re-run, end_stop_jerk, 129 sv1 vs 29 sv2, all `isd_m == 0`):**
rule engages, strata pool, verdict still `refused_insufficient_power` at n_after = 29 < 150/arm —
the correct outcome — but now with real numbers instead of NaN: pooled median delta
+0.260 m/s³ (new arm higher, descriptive only), 95% CI [+0.126, +0.549], `mde_at_n` 0.152 m/s³;
the `v<1|lead` stratum pools 116 vs 29 (per-stratum Mann–Whitney p = 0.0067, descriptive — the
power refusal stands), `v<1|no_lead` skipped (no sv2 events yet). Exit code 2.

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
