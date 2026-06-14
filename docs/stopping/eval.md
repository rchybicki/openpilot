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
                   "settle_meas_minus_sent_jerk": null,
                   "settle_peak_imu_jerk": null, "settle_peak_imu_decel": null},
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
- **Cranked comfort criteria (version 3, 2026-06-13):** BOTH cranked flags are now GATING.
  `unnecessary_harsh_approach` (gap-gated approach decel cap 0.5 m/s² with a kinematic-necessity
  exemption) gates on the command; `harsh_terminal_grab` was PROMOTED back to GATING on 2026-06-13
  via the **faithful IMU channel** (`settle_peak_imu_jerk` cap **30.0 m/s³**, named
  `terminal_max_settle_imu_jerk`). The old WHEEL-aEgo settle cap (`terminal_max_settle_meas_jerk`,
  3.0 m/s³) is retained as a NON-gating diagnostic only — see §2.1.
- Any threshold change = config version bump + re-baseline note here (the version 3 bump is §2.1).
- `check_harsh_stops.py` and the cycle read defaults from this config (CLI flags are explicit
  overrides only, retained until the cleanup commit reworks the test files).

### 2.1 Cranked comfort requirement (version 3, 2026-06-13)

The user feels **two distinct harsh events per stop** and wants each measured, then the requirement
cranked. `scoring_config` now at version **3** (`CrankedComfortThresholds`) carries **two GATING
harsh-classification criteria**: P1 (`unnecessary_harsh_approach`, command-side) and P2
(`harsh_terminal_grab`), the latter PROMOTED back to gating on 2026-06-13 once the felt terminal
grab became faithfully measurable via the device IMU (`settle_peak_imu_jerk`). The history: P2 was
first added as a gating flag, DEMOTED to a non-gating diagnostic on 2026-06-13 because the only
channel then available (wheel-derived `a_ego`) is blind to the v≈0 grab, then RE-PROMOTED the same
day after the IMU channel was wired in and a baseline measured (this section). Both metrics are
computed by `build_event_store` on the engaged + long-control-active Sample stream and scored on the
**`metrics_100hz`** block (rlog100 primary — 10 Hz decimation systematically understates jerk,
spec 7.2, so the terminal settle-jerk gate in particular is only faithful at 100 Hz). The legacy
10 Hz-provenance rule (§2 / §2.2) does **not** apply to these two; their thresholds belong to the
100 Hz block by construction.

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

**P2 — terminal disc-grab (`terminal_max_settle_imu_jerk`): GATING via the device IMU
(re-promoted 2026-06-13).** *Requirement:* peak **FAITHFUL** settle jerk on the device IMU
longitudinal channel at the first genuine standstill must stay ≤ **30.0 m/s³** (named constant
`terminal_max_settle_imu_jerk`). The gating metric is `settle_peak_imu_jerk`
(`build_event_store.settle_imu_jerk`) — peak |d(a_long_imu)/dt|, where
`a_long_imu = livePose.accelerationDevice.x` is locationd's gravity-removed, pitch-compensated EKF
longitudinal accel (device frame [Forward,Right,Down], ~20 Hz). It is read over the same settle
window as the wheel metric: from ~0.6 s before the first sample reaching the genuine-standstill band
(`SETTLE_STANDSTILL_SPEED = 0.06` m/s) up to that first-standstill sample, masked to engaged +
long-control-active and **truncated at the first inactive frame after being active** (same
takeover-artifact guard as the hold-acquisition diagnostic). `classify_event` raises
`harsh_terminal_grab` when `settle_peak_imu_jerk > 30.0`, contributing to the harsh verdict + quality
bucket. **None-IMU events do not raise the flag** (qlog-only / pre-livePose routes have no channel —
graceful degradation, never gate on a missing signal).

*Why the IMU, and why this reverses the earlier demotion.* The felt grab is the brake pads biting at
v ≈ 0. The wheel-derived `a_ego` is **blind** there: it quantizes/floors to ~0.04 m/s² below
~0.03 m/s (no wheel motion to differentiate), so the static-friction grab leaves no wheel signature,
and under StopReq-A the SCC owns the stop below the 0.04 m/s gate so the openpilot command is blind
too. That blindness is exactly why P2 was demoted on 2026-06-13. The fix was an instrument, not a
requirement change: `livePose.accelerationDevice.x` reads ~0 parked (mean ≈ −0.02, std ≈ 0.02 m/s²,
gravity already removed) where `a_ego` floors out, yet still resolves a real settle decel of
0.2–1.7 m/s² and jerk up to ~70 m/s³, and a subjectively harder stop reads higher (cross-validated
against the 20 Hz-decimated pitch-compensated raw accelerometer within ~10–20%). The wheel-aEgo
settle metric (`settle_peak_meas_jerk`, 3.0 m/s³ cap) is **retained as a NON-gating diagnostic** and
under-reports the grab by a median ~9× (IMU jerk − wheel jerk median +21 m/s³ on the StopReq-A
baseline) — kept only as a fallback channel label for routes with no IMU.

*Threshold calibration (the crank knob).* 30.0 m/s³ is set from the 2026-06-13 IMU baseline over
**33 measurable engaged settles** (StopReq-A routes 0000171e/171f at gate 0.04 + the pre-StopReq-A
sv2 routes where openpilot commanded to near-standstill): combined median 24.9, p60 30.5, p75 35.8,
p90 40.7, max 70.2 m/s³; companion `settle_peak_imu_decel` ranges 0.22–0.89 m/s². 30.0 sits just
above the p60 — it spares the gentle-to-moderate half (smooth settles down to 5.4 m/s³) and flags
the **rough tail only** (~42% of measurable settles). It is deliberately the **crank knob**: tighten
toward 25 / 22 m/s³ as the terminal decel softens.

*SCC-handoff vs legacy (the first evidence on the culprit, reported honestly).* Per-arm IMU
baseline: StopReq-A (gate 0.04, SCC owns final stop) settle jerk **n=10, median 23.7, p90 36.3,
max 39.6 m/s³**, decel median 0.59; pre-StopReq-A sv2 (openpilot-to-near-standstill) **n=23, median
26.7, p90 40.7, max 70.2 m/s³**, decel median 0.66. The SCC-handoff stops are **NOT** harsher than
the legacy stops — if anything marginally gentler, but **inconclusively** (Mann-Whitney jerk
p ≈ 0.62, decel p ≈ 0.77; n is small). The terminal grab is **pervasive in both arms** (~40% over
30 m/s³ in each), so on this evidence the SCC handoff is **not** the obvious culprit and the grab
predates it. The gate-reduction experiment (drop the StopReq gate so openpilot commands the terminal
decel) is still required to attribute it causally.

**Corpus re-score (218-event store, scored on `metrics_100hz` with the engaged comfort filter:
`enabled_ratio ≥ 0.80`, `should_stop_ratio ≥ 0.15`, `v_approach ≥ 0.50` → 111 considered).** Both P1
and P2 are gating. The IMU gate can only fire on the **23 IMU-measurable engaged settles** (events
on the routes re-ingested with the livePose channel — only those carry `settle_peak_imu_jerk`);
None-IMU events never raise it.

| Criterion | Gating? | Failures (basis) |
|---|---|---|
| `unnecessary_harsh_approach` (P1) | **yes** | 15 / 111 considered (13.5%) |
| `harsh_terminal_grab` (P2, IMU) | **yes** | **9 / 23 IMU-measurable settles (39.1%)** = 9 / 111 considered (8.1%) |

(P1's 13.5% here is lower than the earlier §2.1-v2 figure because the store was re-ingested at the
faithful 100 Hz with the livePose channel and the considered set/filter basis shifted; the absolute
P1 count on this basis is 15.)

The nine flagged terminal-grab events: `00001717#19` (70.2), `0000171d#7` (40.7), `0000171c#4`
(40.2), `0000171f#28` (39.6), `0000171f#21` (35.9), `00001717#3` (35.6), `0000171f#28` (31.3),
`0000171f#42` (31.1), `00001713#16` (30.2 m/s³). Promoting the IMU gate is a THRESHOLD/GATING change
→ `scoring_config` bumps to version **3**. Re-tuning the 30.0 cap downward as the user iterates is a
100 Hz threshold change (another version bump each time).

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

### 5.1 Friction-augmented plant (`fit_friction_residual.py` → `sim_replay --friction`) — DEVELOPMENT ONLY, NEVER A GATE

The linear AR(1) plant is identified on **wheel** `aEgo`, which quantizes/floors to ~0 below
~0.03 m/s, so it is structurally **blind** to the terminal brake-friction "disc-grab" the driver
feels as the car settles. The device IMU longitudinal channel (`a_long_imu` =
`livePose.accelerationDevice.x`, gravity-removed/pitch-compensated, ~20 Hz) **sees** it. The grab is
therefore exactly the residual the wheel plant misses:

```
a_imu(v) ≈ linear_plant_response(command, state) + friction_residual(v_ego)
friction_residual(v) = c0 + c1·exp(−v/v0)          (3-param Stribeck-like net-decel curve)
```

`fit_friction_residual.py` fits that velocity curve on the **decel** channel (`a_long_imu − a_ego`)
over the engaged settle window (same window as `build_event_store.settle_imu_jerk`).
`stopping_plant.FrictionPlant` wraps a `PlantModel`: `predict_next` is **bit-identical** to the
linear plant (wheel path untouched) and `predict_next_imu` adds the residual. `sim_replay.py
--friction <fit>` is the **opt-in** consumer: the closed loop still drives off the wheel channel
exactly as the gated sim, but the trace additionally records a predicted-IMU channel and reports
`settle_peak_imu_jerk_pred` / `settle_peak_imu_decel_pred` (named `_pred` precisely so they can
**never** be confused with the on-road gating `settle_peak_imu_jerk`).

**Fit quality (archived `docs/stopping/archive/friction_residual_20260614.json`, "coarse-provisional"):**
n = 26 distinct stops / 9 routes / 1573 residual frames, leave-one-route-out hold-out. Per-stop
peak-decel MAE 0.065 m/s² (in-sample ≈ hold-out, no generalization gap), Spearman 0.87 (ranks a
harsh driveway grab above a gentle one). **Only the velocity *shape* `v0` is well-constrained**
(0.041–0.045 across all folds); `c0`/`c1` sit on their physical-prior rails in every fold — the
absolute offset/amplitude split is **not** identified by this thin, low-speed/driveway-skewed,
route-correlated data. The 20 Hz **jerk** channel is aliased for the sub-100 ms grab peak and is
deliberately **not** fitted. HEV confound: regen vs friction not separable. Trust the curve for
**relative ranking** and reproducing the wheel-blind grab decel; do **not** trust absolute physical
friction, jerk magnitude, rolling-traffic stops, or any gating decision.

**Discipline (the project's founding lesson):** this plant is a **development accelerator only**. It
**must not** become a promotion gate, is **never** wired into `similarity_gate` (the gate's plant
selection is untouched), and the on-road IMU `settle_peak_imu_jerk` (§2.1, threshold 30 m/s³) stays
the sole P2 promoter. Every read from it is a **model prediction to confirm on-road**. The fit
**grows with engaged drives** — each new IMU-bearing engaged settle in the event store tightens it
(and may eventually pull `c0`/`c1` off their rails); re-fit and re-archive as the corpus grows.

**Pre-release re-score (`rescore_prerelease_friction.py`, 2026-06-14, model prediction).** Driving
the live legacy controller through the friction plant over the IMU-bearing settles + fixtures, the
deployed anti-stiction pre-release (HEAD: A=0.30, J=1.5) vs pre-release-OFF shows **no material
change** in the predicted grab (median predicted `settle_peak_imu_jerk` 3.90 → 3.90 m/s³, max
16.20 → 16.20; robust across dt = 0.10/0.05/0.02, even though the lane fires in ~45–53 of ~80
settles). The knob sweep (A ∈ {0.25, 0.30, 0.35} × J ∈ {1.0, 1.5, 2.0}) is **flat** (all medians
3.90–3.93 m/s³). **This is expected and honest, not a defect:** the friction residual is
**velocity-only**, so a shallower terminal *command* only moves the predicted grab to the extent it
moves the *velocity sweep* through the 0.06→0.30 m/s band — which it barely does — and the residual
does **not** model the actual stiction-relief mechanism (gentler pad engagement) the pre-release
targets. **The sim therefore cannot prove the pre-release helps; the on-road IMU must.** This is
the correct division of labor: the friction plant tells us where the grab lives and ranks settles;
the felt-grab reduction is an on-road `settle_peak_imu_jerk` measurement.

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
