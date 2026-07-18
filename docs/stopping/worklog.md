# Stopping Worklog

Dated evidence log for the stopping stack (post-redesign home). Entries are appended by
`tools/stopping/append_analysis_report.py`, `append_cycle_report.py`, and `append_sync_report.py`
(this file is their default `--worklog`), or by hand.

- History through 2026 H1 (pre-redesign forest era): `docs/stopping/archive/worklog_2026H1.md`
- Methodology and gate protocol: `docs/stopping/eval.md`
- Entry format: `### YYYY-MM-DD: <title>` followed by bullet lines (commands, artifact paths,
  before/after metrics, keep/reject decision).

### 2026-06-10: Redesign docs scaffold

- New documentation home created (`docs/stopping/`); legacy worklog archived to `docs/stopping/archive/worklog_2026H1.md`.
- Similarity gate (spec 7.6) ran on the dual-plant deck and did NOT pass; `USE_STOPPING_V2` remains `False` (legacy forest active). Details: `docs/stopping/eval.md`, gate-status section.

### 2026-06-10: Driveway review, hold-acquisition soften, rollout plan

- Driveway/hill-hold blind-window review (route `00001702--dcdc5c3eea--0`): felt hold-acquisition slam traced to the deep ramp (-0.5..-0.78 -> -1.05).
- Hold-acquisition soften shipped (commit 9b4a07de78): parameters.md row 40 (`HOLD_ACQ_SOFTEN_*`, `J_HOLD_ACQUISITION` 1.0 m/s^3, arrest floor `J_HOLD_ACQUISITION_ARREST` 2.0 m/s^3); `hold_acq_peak_cmd_jerk` added to the event store as a NON-gating diagnostic (eval.md section 1).
- dRel-honesty flip prepared: `PUBLISH_TRUE_LEAD_DISTANCE = True` at device ISD = 0.0 (all weather variants 0; bit-identical today, later ISD raises compensated by design).
- Living rollout plan adopted: `docs/stopping/rollout_plan.md` (status table, stage 1 baseline-corpus procedure, decision log). Sequencing re-scoped: StopReq/dynamic-jerk stages are independent of the V2 flip; `on_vehicle_protocols.md` cross-linked.
- Decision: keep; stage 1 (baseline corpus, no flag changes) is now active.

### 2026-06-12: Stage-1 cycle report — 12 routes ingested, stage 1 continues (29/40 honest events)

- Ingest: 12 routes (00001703..00001714, 225 segments) → +29 telemetry-v2/signals-v2 events (all rlog100, accel_cmd_source=carOutput); store total 158 (129 sv1 baseline + 29 sv2). Manifest: added=29, replaced=0.
- Error sweep: NONE attributable to the stopping stack — 0 tracebacks / 0 CRITICAL across all 225 segments; no daemon crashes; no mid-drive cruise faults (all accFaulted/controlsMismatch/canError within 0.3 s of ignition-off, plus documented t<9 s startup instances); 17 engaged stopping intervals, all clean. Watch items (pre-existing, non-stopping): kernel-stall severity worsening (24.6 s sysfs read on 00001714; 12.0/11.9 s on 00001713; mid-drive softDisable at ~140 km/h on 00001714, plus encoder gap at t=723 re-triggering the loggerd desync); deep_rl3 (route 00001714) gentle unexplained decel at 31–35 m/s (accelCmd −0.45..−0.78, t=496–498 and 533–536) — model-regression candidate, not stopping code.
- USER BOOKMARK (00001714--a9b5750c03 seg 8, +539.5 s into the segment qlog — qlog spans 541 s due to the pre-b04a617ee3 rotation desync; seg-8 rlog absent): highway ~30 m/s, engaged, longitudinal in override (long_state 'off', cmd 0, no brake), no stop event nearby — likely flags deep_rl3 highway behavior; needs separate review (video streams may be dead per the desync bug).
- Hold-acq diagnostic (eval.md §1, non-gating): (1) metric flaw — `hold_acq_peak_cmd_jerk` (tools/stopping/build_event_store.py:88) uses an unmasked [edge, +2 s] window, so driver-takeover command-zeroing pollutes it; every 100+ value in both arms is an artifact (background fix task filed: mask to long-control-active + regression test). (2) Artifact-free comparison: NO improvement signal post-soften (driveway 5.8–6.1 vs 7.1–8.7 m/s³ n=3 vs 3; all-edge active-only median 5.11 n=18 vs 7.96 n=5) — too small to conclude, point estimates slightly higher post-soften. (3) Stop-tail: small suggestive median worsening on both proxies (p=0.024 unpaired diagnostic), better worst case — monitor only. Artifacts: /tmp/hold_acq_edges_final.json, /tmp/hold_acq_extract.py, /tmp/hold_acq_final.py.
- Plant refit (documented CLI, --baseline-json docs/stopping/archive/plant_model_20260531T0751…): φ(a_ego_prev)=0.918085 ∈ (0,1) at dt=0.1 s; acceptance PASS — holdout RMSE 0.03759 (n=2626) vs baseline 0.04187 (n=3060), ratio 0.898 ≤ 1.1, same 5 holdout routes (tools/stopping/holdout_routes.txt). estimator_equivalence PASS (100%/100%).
- Similarity gate re-run 2026-06-12T18:27:17Z (legacy forest vs v2, deck 189 = 158 store events incl. 29 honest + fixtures + 4 integrated dropout fixtures, scoring v1; dual plants {ref_20260514 frozen, refit_20260612} driven via a /tmp driver mirroring main() exactly because --plant both hardcodes the archived 20260531 refit — no repo changes): **NOT passed**. Frozen plant tier 1 FAIL 4/8: leapfrog_per_stratum (v>2|explicit: v2=21 > forest=20, other 7 strata pass), rollout_delta_p95 (0.886 m vs 0.15, n=124), hold_gap_delta_p95 (2.221 m vs 0.10, n=138 — NEW, was 0.070 on Jun-10), time_to_standstill_delta_p95 (2.68 s vs 0.5, n=123). PASS rows: harsh_no_v2_only, end_jerk_median_ci ([0.0, 0.0] within [−0.05, +0.03]), integrated_dropout_hold.
- paired_stats compare_onroad (end_stop_jerk, A=129 old / B=29 new): VERDICT REFUSED (refused_insufficient_power, exit 2) — n_after=29 < 150/arm floor (mde_at_n 0.152 m/s³ vs old median ~0.76 at α=0.05, power 0.80); all cross-arm deltas descriptive only. Structural: stratum_of() includes signals_version, so sv1/sv2 arms occupy disjoint strata — cross-era pairing is refused-by-construction; a cross-era rule needs an eval.md decision before any gate verdict from this pairing. Within-era sv2-vs-sv2 accumulation is the usable path.
- DECISION (path C per rollout_plan.md): stage 1 exit NOT met — sole blocker is the honest-event count (29/40; need 11+ more) — **no flips, stage 1 continues**. `USE_STOPPING_V2` stays False (gate honestly failed again); StopReq stage 3 not entered (requires stage 1 exit). Status table + decision log updated.
- Cross-era comparison rule decided and implemented (eval.md §3.1, decided 2026-06-12): `paired_stats.compare_onroad` drops `signals_version` from the stratum key IFF every event in BOTH arms records `entry.isd_m == 0` (arm-level `all_zero_isd` precondition; missing/nonzero ISD anywhere keeps the strict refused-by-construction behavior). Rationale: the dRel-honesty flip changes lead-gap semantics only through a nonzero IncreasedStoppedDistance, and this device runs ISD = 0.0 — so v1/v2 eras are physically comparable when all-zero ISD is proven in the data. Loud CLI NOTE + `cross_era_rule` report block when engaged; gates stay same-era (similarity_gate uses `sim_replay.stratum_for_entry`, guard comment added at its `paired_stats` import); floors unchanged; non-gating diagnostics rule → no scoring_config bump. Re-run of the old-vs-new end_stop_jerk comparison (129 sv1 vs 29 sv2, all isd_m=0): rule ENGAGED, verdict still refused_insufficient_power at n_after=29 < 150/arm (exit 2, the correct outcome) — but now with real numbers instead of NaN: pooled Δmedian +0.260 m/s³ (new higher, descriptive), 95% CI [+0.126, +0.549], mde_at_n 0.152 m/s³; `v<1|lead` pools 116-vs-29 (MW p=0.0067, descriptive), `v<1|no_lead` skipped (no sv2 events yet). Tests: 4 new cross-era cases in test_paired_stats.py (engage-and-verdict, power-refusal-not-NaN at the 2026-06-12 shape, single nonzero-ISD strictness, missing-ISD strictness).

## 2026-06-12 (evening) — post-cycle tooling fixes

The cycle's own measurements exposed three defects in the measurement layer; all fixed, tested, and re-derived the same evening:

- **hold_acq_peak_cmd_jerk takeover artifact**: the diagnostic window was not masked to long-control-active frames, so driver takeovers within 2 s of engaging contributed the command→0 zeroing step (~93–135 m/s³ readings in BOTH arms of the before/after comparison). Fixed (mask + truncate at first inactive frame; 3 regression tests); affected routes re-ingested (manifest: replaced=27, store total 158 unchanged). Corrected values: 5.98–9.07 m/s³. Note: the corrected peak is dominated by the benign single-frame engage-seed step, so this peak metric still cannot resolve the hold-acquisition soften's deep-ramp shaping — windowed/segmented jerk would be needed if we want to measure that specifically.
- **fit_plant_model hardening**: unstable-pole candidate guard added (fired on real data: delays 6–7 had poles >1) and default --max-delay-frames raised 8→15 with a ceiling warning. Corrected refit: delay 9 (0.9 s dead time), φ=0.791, holdout ratio 1.086 ≤ 1.1 PASS — archived as docs/stopping/archive/plant_model_20260612_refit.json (supersedes the same-day ceiling-pinned delay-8 run; both pass acceptance, stage-1 verdict unchanged).
- **cross-era stats rule** (eval.md §3.1): signals_version is dropped from the paired-stats stratum key iff every event in both arms has isd_m == 0 (physically identical eras on this car). Jun-12 comparison re-run: still refused for power (n=29 < 150/arm) but now with real numbers — end_stop_jerk pooled Δ +0.26 m/s³ [CI +0.13, +0.55], descriptive only. Gates remain same-era.
- Log pulling: use tools/route_sync/refresh_routes.py (or build_event_store --fetch-missing-rlogs) — logs only, never whole segment dirs (video). Locally cached video files from this cycle's manual rsync were purged.

### 2026-06-13: Stage-1 EXIT + Path B — StopReq stage A staged (gate failed, corpus established)

- Ingest: +9 routes (00001715..0000171d, 122 segments) → +36 honest tv2/sv2 stop events. Store now **194 total** (129 sv1 + 65 sv2). Honest events **65/40** — the 29/40 event-count blocker from 2026-06-12 is **cleared**. All honest events are simultaneously telemetry_version≥2 ∧ signals_version≥2 ∧ accel_cmd_source=='carOutput', rate_class rlog100. Honest controller_commit spread: 2ee311b407 ×29, 5076fcb9b5 ×24, d08f2033ac ×5, d19a7abc8d ×5, 8a19a13925 ×2 (all post-29219fc51d, same sv2 era).
- **Stage 1 EXIT MET** (all three criteria, rollout_plan.md line 19/77-86): (1) honest events 65 ≥ 40; (2) plant refit_20260613 stable pole φ(a_ego_prev) ∈ (0,1) at an interior optimum (not a sweep-ceiling pin — hardened fitter held); (3) estimator_equivalence PASS.
- Similarity gate re-run on dual plants → **NOT passed on EITHER plant** → `USE_STOPPING_V2` stays False (V2 dark). ref_20260514: leapfrog/rollout_p95(0.863)/hold_gap_p95(2.335)/tts_p95(3.02) FAIL; harsh/end_jerk/integrated/estimator PASS. refit_20260613: harsh/leapfrog/rollout_p95(0.362)/hold_gap_p95(2.664)/integrated FAIL; end_jerk/tts_p95(0.0)/estimator PASS. 163 tier-2 events unclassified — the V2 flip is blocked on triage, not on data volume.
- Error sweep (all 9 routes): NO new stopping-code-attributable issues — no traceback/CRITICAL/cruise-fault/daemon-restart correlated with longControlState==stopping or any new path. **Downhill-clip relax (523cd29fce) NOT exercised** — all 9 routes ran commits before it (latest 2ee311b407 @09:57; downhill-clip @11:40), so its alert correlation could not be tested this cycle; needs routes on ≥ 523cd29fce next cycle. Data corrections: 00001719 commit recovered as 8a19a13925; 00001719 + 0000171a are PRE-deviceState-tolerance (only 0000171b/c/d carry the mitigation).
- commIssue/kernel-stall trend: the two selfdrived deviceState stale-tolerance commits (2ee311b407, 21bdfe9567) materially reduced the kernel-stall signature (symptom mitigation, not a kernel fix). Per engaged-minute: PRE 1.39 thermal/min + 1.39 commIssue/min → POST 0.06 thermal/min (~23×↓) + 0.29 commIssue/min (~5×↓); absolute thermal-stall 28→2 despite more engaged time. softDisable-while-engaged occurred ONCE in 9 routes (00001718 seg8 @506.8s, ~1s commIssue dropout at vEgo~13 m/s, pre-mitigation, far from any stop). Caveat: confounded (worst pre-fix routes are heaviest pre-mitigation drives; AGNOS kernel root cause unchanged). commIssue residual is DM/camera-service dominated, non-engaged, non-stopping-correlated.
- hold-acq diagnostic (non-gating, tooling corrected per the 2026-06-12 mask-to-active fix): no new hold-acq regression. Quality-by-model: honest events span 5 controller commits, all on the same sv2 actuator path; no per-model stopping-quality outlier in the error sweep. No new USER BOOKMARKs in this batch's brief.
- **DECISION — PATH B** (decision tree: refit.exit_met=true ∧ gate.passed=false): StopReq stage A is the next deployable stage (corpus/plant established → the actuator-path-identification ordering constraint is satisfied; V2 flip blocked on tier-2 triage; StopReq is the independent carcontroller track).

#### Staged change (one behavioral variable: the StopReq assert semantics)

- `opendbc_repo/opendbc/car/hyundai/carcontroller.py`: `STOPREQ_LATCH` False → **True**; `STOP_REQ_MAX_SPEED` 0.01 → **0.04**; `STOPREQ_RELEASE_SPEED` **0.10 unchanged** (always-active speed release, F1). This is on_vehicle_protocols.md §1 **stage A** verbatim. Protocol stage 0 (latch on @ gate 0.01) was skipped: stage 0 and stage A differ only in the assert gate, and 0.04 is still below the 0.104 m/s wheel-standstill threshold (wheels provably stopped at the assert point), so the conservative standstill-only property of stage 0 is preserved while covering the full Kalman-dither band in one stage.
- `opendbc_repo/opendbc/car/hyundai/tests/test_can_bounds_fork.py`: updated to the documented staged-flip procedure. `TestZeroCanDelta` split into (a) `test_scc_bytes_identical_to_legacy_reference_in_dark_state` — constants monkeypatched back to legacy, byte-for-byte identical to the pre-change oracle (the rollback-target proof); and (b) `test_stage_a_diverges_from_legacy_only_in_stopreq_bit` — with the LIVE stage-A constants, SCC11/SCC14 stay byte-identical and SCC12 diverges from legacy ONLY in the StopReq bit (every other payload field byte-equal; CR_VSM_ChkSum excluded as a dependent field; divergence direction pinned StopReq 0→1, never the reverse). `test_legacy_gate_default` → `test_legacy_gate_dark_state` (monkeypatched to legacy); new `test_stage_a_live_gate_and_latch` pins the shipped gate 0.04 + latch + speed-release behavior. `test_default_constants_are_normative` now pins the LIVE stage-A values with the legacy revert targets documented inline.
- Tests: **23/23 test_can_bounds_fork.py PASS** (was 21; +2 net), **13 passed + 2 skipped + 343 subtests passed** in test_hyundai.py. ruff (isolated E/F/W, line-length 160) clean on both changed .py (repo pyproject ruff-config selector `TRY203` errors under the installed ruff version — pre-existing env mismatch, not introduced here).
- NOT committed, NOT deployed (human commits + deploys after review). Parking-lot session required first per protocol §1: 3 stops + 60 s holds + 1 deliberate creep-push, watching TCS15 `AVH_LAMP`, TCS13 `PBRAKE_ACT`/`ACCEnable`, cruise faults.

#### Adversarial CAN safety review of the staged diff — verdict: NO fatal/major findings

The diff changes exactly one CAN signal's assert logic (SCC12 `StopReq`). Reviewed against EPB/auto-hold engagement, StopReq-while-moving ESC behavior, panda frame-drop/wrap, and fault-latch paths. Empirically simulated the stage-A latch on the Santa Fe HEV CP (latch set/hold/clear sweep).

- **EPB / auto-hold engagement on prolonged holds — LOW.** StopReq is asserted only when `vEgo < 0.04` (wheels provably stopped; below the 0.104 m/s wheel-standstill threshold) and is the standard ESC standstill-hold request the stock SCC already uses. Stage A does NOT change WHAT StopReq means to the EPB/ESC, only WHEN it asserts (0.04 vs 0.01) and that it latches through Kalman dither instead of chattering at 50 Hz. The chatter-fix is strictly *less* likely to surprise the EPB than the legacy 50 Hz toggle. The promotion criteria still require an explicit ≥ 60 s standstill hold watching TCS15 `AVH_LAMP`/TCS13 `PBRAKE_ACT` before promoting — residual risk is observational, not latent in the diff. Stage A is NOT the EPB-handoff stage (that is stage C, gated separately with the explicit auto-hold watch).
- **StopReq-while-moving ESC behavior at the new 0.04 gate — LOW (the central change, designed for).** The assert gate rises 0.01 → 0.04, still 2.6× below wheel-standstill (0.104 m/s) — the wheels are provably stopped at the assert point, so this is NOT a StopReq-while-rolling probe (that is deliberately deferred to stage C). The always-active speed release (`vEgo > STOPREQ_RELEASE_SPEED = 0.10`) is checked BEFORE the latch-set branch, so a creep-push/hill-roll that carries the car past 0.10 m/s clears the latch on the same frame even while `longControlState` stays `stopping` (F1: the latch may NEVER hold StopReq on a rolling car). Verified empirically: dwelling in the 0.04–0.10 band never SETS the latch (set requires vEgo < 0.04); once set, the latch holds through dither to 0.09 and clears at ≥ 0.10; re-stop re-latches only below 0.04 (clean hysteresis); state-exit (starting) clears even at vEgo = 0.01 (launch path intact).
- **Panda frame-drop / wrap on any changed value — NONE.** No numeric range or scaling changed. `StopReq` is a 1-bit signal (0/1); 0.04 and 0.10 are control-side comparison thresholds in `carcontroller.py`, never packed onto CAN. The SCC12 payload range, the unconditional accel clip [ACCEL_MIN, ACCEL_MAX], and the SCC14 jerk clip [0, 12.7] are all untouched. No CANPacker wrap exposure is added. `CR_VSM_ChkSum` recomputes correctly over the new StopReq bit (it is computed from the packed payload in `create_acc_commands`, unchanged path).
- **Fault-latch paths (accFaulted / cruise fault) — NONE added.** The non-finite-accel-before-clip guard, the 50 Hz SCC12 cadence, and the ACCMode field are all unchanged. StopReq does not gate `ACCEnable`/`ACCMode`; the diff cannot drive TCS13 `ACCEnable != 0`. The stage-A latch only ever asserts a value the stock contract already supports at standstill.
- **One residual to watch on-vehicle (not a diff defect):** the latch now holds StopReq continuously for the entire standstill instead of toggling at 50 Hz. If the specific Santa Fe HEV EPB interprets a *sustained* StopReq differently from the legacy chatter (e.g. arms auto-hold sooner), that would surface as TCS15 `AVH_LAMP` during the mandatory ≥ 60 s parking-lot hold — which is exactly what the promotion criteria gate on. No code change can pre-empt that observation; it is correctly deferred to the on-vehicle session.

Verdict: **NO fatal/major findings; deploy is not blocked by the diff.** All identified risks are LOW or observational and are already gated by the protocol's parking-lot promotion criteria. Deploy alone (one variable), parking-lot session first, revert = flip the three constants back to legacy (False / 0.01 / 0.10) — the dark-state byte-identity test is the rollback proof.

## 2026-06-13 (comfort cycle) — user-felt forces: measure, crank, iterate

Routes 0000171e--5c66f4db31 + 0000171f--45bcc6b3a0 (both on 390054594e = StopReq-A; 24 honest sv2 stop events, store 194→218; zero video pulled; no userBookmarks).

**StopReq-A on-road validation (the parking-lot-equivalent, from real driving): CLEAN.** 13 engaged standstill holds — AVH_LAMP + PBRAKE_ACT False every frame (no auto-hold/EPB), 0 faults during holds, latch never held on a rolling car. StopReq A locked in.

**P1 — unnecessary harsh approach: measured, cranked, fixed.** Ground truth: 4 of 6 over-0.5-m/s² approach-brake events were unnecessary (required-decel 0.02–0.34 vs commanded 0.59–0.85); 2 were necessary (fast approaches). Command tracks aEgo within ~0.1 on approach → command-measurable. Eval: new gating `unnecessary_harsh_approach` (gap>2m, peak |cmd| ≤0.5 unless kinematically required; engaged-masked), SCORING_CONFIG_VERSION 1→2, baseline 68/131 (51.9%) engaged-fail. Controller: one kinematic cap = max(0.5, closing²/(2·max(gap−2,ε))) with a +0.18 release margin, rate-limited at 2.0 m/s³, in both the stopping_controller final clamp and longcontrol post-cap (covers PID-lane planner-aTarget origins). Sim: approach-harsh ~halved both plants; adversarial fast-close fixture reached full −2.0 authority (no under-braking); hill-hold + StopReq-A unaffected. Deployed.

**P2 — terminal disc-grab: HELD, not measurable yet.** wheel-aEgo quantizes to ~0 at standstill (the felt v≈0 grab has no wheel signature); command is SCC-blind under StopReq-A. `harsh_terminal_grab` demoted to non-gating diagnostic — do not gate on a blind metric. The command-side P2 lever is ~exhausted (commanded settle jerk ≤1.5 m/s³; felt excess is stiction). NEXT: wire an IMU long-accel channel (raw accelerometer ~101 Hz / livePose.accelerationDevice ~20 Hz; neither consumed by load_samples today) into the eval, THEN crank/iterate P2 — likely via the StopReq-A SCC handoff rather than deeper command caps.

Tests: 615 passed / 1 pre-existing fail (test_stopping_controller.py:1792) / 18 skipped; ruff zero-new. The V2 similarity gate still fails (163 tier-2 events unclassified) — V2 stays dark; triage remains the V2-flip long pole.

## 2026-06-13 (IMU + gate experiment) — measuring and chasing the terminal grab

**The grab is now measurable — and ~9x harsher than we thought.** Wheel-derived aEgo floors to ~0 below 0.03 m/s, so the v→0 static-friction grab left no signature; the earlier "SCC tail is smooth" conclusion used that blind sensor and is retracted. Wired `livePose.accelerationDevice.x` (locationd EKF, gravity-removed, ~20 Hz) into load_samples as `a_long_imu` (raw accelerometer rejected — gravity-dominated + noise-swamped; ESP12 CAN LONG_ACCEL exists at msg 544 but is unparsed on this fork). The IMU settle metric resolves real jolts (jerk 20–40 m/s³, decel 0.24–0.87) where aEgo read ~0. `harsh_terminal_grab` promoted back to GATING on the IMU channel, threshold 30 m/s³ (just above the combined p60); scoring v2→3.

**Baseline (the "how harsh, really" number): StopReq-A arm (gate 0.04) settle jerk median 23.7 / p90 36.3 / max 39.6 m/s³; pre-StopReq-A median 26.7 / p90 40.7 / max 70.2.** The grab is genuinely harsh and PERVASIVE (~40% of settles >30 m/s³ in both arms).

**Key finding — the SCC handoff is NOT the obvious culprit.** StopReq-A (SCC owns the stop) is statistically indistinguishable from legacy (openpilot to near-standstill): Mann-Whitney p=0.62 jerk, p=0.77 decel, n=10 vs 23. The grab predates StopReq-A and appears regardless of who commands the final stop — consistent with a friction-transition physical phenomenon, not a command/handoff artifact.

**Action: StopReq gate 0.04→0.01** (reverts StopReq stage A toward legacy). Rationale: (1) the causal A/B — does the grab change when openpilot owns the terminal? — and (2) the prerequisite for a command-side **anti-stiction pre-release** (ease the brake just before the wheels stop), which the SCC-owned 0.04 handoff precludes. The user's instinct (take the terminal back from the SCC) is right as an enabler even though the "SCC causes the grab" framing is unconfirmed. Safety: no fatal/major; latch + 0.10 release unchanged (never holds on a rolling car). 0.0001 rejected (dither may never assert StopReq → no SCC managed hold). Parking-lot watch: clean standstill hold, no creep, no ACC dropout. Deployed.

**Next:** drive on gate 0.01 → IMU metric gives the causal answer + the new P1-cap on-road confirmation. Then add the openpilot anti-stiction terminal pre-release (the likely real P2 fix) and crank the 30 m/s³ threshold down as it improves.

### 2026-06-14: Gate-0.01 review cycle — standstill hold CLEAN, handoff A/B against the hypothesis, P1 not yet confirmed → build the anti-stiction pre-release next

Three new routes reviewed (00001720--3d68d7af81, 00001721--292d134c01, 00001722--35371c3e9e). Logs only; gate 0.01 live on HEAD 4c1230691b. Analysis only — no code change, no commit, no deploy.

**Commit/gate audit (each route's controller_commit verified against the store, not assumed):**
- 00001722--35371c3e9e: commit **4c1230691b** (gate 0.01; confirmed ≥ the 57def50ac8 gate-0.01 cutoff by `git merge-base --is-ancestor`). 5 records, the only engaged route this cycle.
- 00001721--292d134c01: commit **57def50ac8** (valid gate-0.01 commit) but **0 events** in the store — route ran fully unengaged (no `stopping` longState).
- 00001720--3d68d7af81: commit **5061019182**, which **PREDATES the gate-0.01 rollback** (`git merge-base --is-ancestor 57def50ac8 5061019182` → NO; `git show 5061019182:.../carcontroller.py` → `STOP_REQ_MAX_SPEED = 0.04`). So **this route is gate 0.04, not 0.01**, and both its events are ~0% engaged (human-braked). Flagged: it does not contribute to the gate-0.01 arm.

So the entire new gate-0.01 *engaged* population is route 00001722, and within it exactly **n=1 distinct engaged settle** (the store holds 5 records for 722 seg0; ev1/ev2 carry no settle, and ev3/ev4/ev5 share `settle_peak_imu_jerk = 48.03` / `settle_peak_imu_decel = 0.99` — one physical standstill captured under three overlapping detector windows; counting n=3 triple-counts). Verified in the store directly.

**(1) Standstill hold @ gate 0.01 — CLEAN → KEEP GATE 0.01.** Across all gate-0.01 stops: `accFaulted` (TCS13 `ACCEnable != 0`) = 0 on every segment (722, 720-0, 720-1) — the SCC never reported temp/permanent fault. No creep away from an established hold: the ~0.058–0.067 m/s vEgo wander on the no-gas crawl plateaus (722 t=18.56, t=38.68) is sub-0.1 m/s Kalman/wheel-band dither in the terminal of a low-speed crawl the driver then re-launches, NOT creep-from-hold; the initial naive stop-window pass that reported ~0.31 m/s "creep" was a measurement artifact (it captured the legitimate re-launch ramp, window exited at vEgo ≥ 0.30). The only "ACC.enabled==False while engaged" frames are ~40 ms engagement rising-edge transients (722 t=15.42–15.45 and t=29.30–29.33; 720-0 t=58.11 while rolling), not standstill dropouts — of 646 engaged+standstill frames on 722, 642 ACC-ok, 4 rising-edge. **The inverse risk of lowering the gate (car fails to get the SCC managed hold) did not materialize.** Caveat: 722 is a driveway test where the car re-launched at each crawl, so there was no long SCC-managed standstill to creep out of on that route — the managed-hold-cleanliness evidence rests on the rolling stops + prior StopReq-A validation.

**(2) Terminal-grab causal A/B — handoff hypothesis NOT supported (inconclusive-leaning-against).** The lone gate-0.01 engaged stop: `settle_peak_imu_jerk = 48.03` m/s³, `settle_peak_imu_decel = 0.99` (100 Hz; meas-wheel jerk only 3.29, confirming the IMU sees a grab the wheel channel is blind to). The gate-0.04 StopReq-A baseline was **re-derived from the store and reproduced exactly** (validates the harness): IMU jerk n=10, median 23.70 / p90 36.29 / max 39.57; IMU decel median 0.59. The single gate-0.01 stop (48.03) is ~24 m/s³ above the StopReq-A median, **above its p90 AND max**, above the legacy median (26.7), and above the 30 m/s³ `harsh_terminal_grab` gate (would flag P2). **Giving openpilot the terminal decel down to 0.01 did NOT reduce the grab — it read WORSE, not better — confirming the prior baseline finding that the SCC handoff is not the culprit; the grab is friction-transition physics.** Severe caveat: n=1 vs n=10, and population-mismatched (gate-0.01 datum is a driveway standstill-engagement crawl entry_v 0.86 / lead 3.2 m; baseline is rolling traffic, only one baseline event is low-speed-comparable at jerk 20.5). The 48.03 is a single-frame 100 Hz held-IMU spike (10 Hz-compat 6.05); the baseline was measured the same way so the 100 Hz-vs-100 Hz comparison is fair. Directional red flag, NOT a statistical verdict.

**(3) P1 on-road approach-cap confirmation — NOT CONFIRMED, insufficient data (population artifact).** Engaged-masked P1 rate: pre-cap (sv2 corpus) 22/30 = 0.73 (approach_peak_decel median 0.85, p90 1.93, max 2.19; 28/30 over the 0.5 cap), post-cap 1/3 = 0.33. **The 0.73 → 0.33 "drop" is NOT a cap effect — it is a population artifact:** all 3 post-cap engaged events are sub-1-m/s driveway crawls on 00001722 (entry_speed 0.86, worst_gap pinned 2.1–2.3 m at the floor, worst_closing 0.22–0.49 m/s); the two distributions do not overlap on any axis (pre-cap min worst_gap 3.24 m > post-cap max 2.30 m). The two big routes that should hold real >2 m approaches contributed zero usable engaged windows (00001720 gate-0.04 + unengaged; 00001721 zero events). Narrowly positive on the 3 driveway stops: **no under-braking** (hard_decel_duration 0.00 s, gentle min_a_ego −0.63/−0.71/−0.71 inside the −1.05 floor, rollout 0.92–1.52 m under the 2.0 m budget, no cap-induced min-gap reduction — the close 1.8–2.3 m terminal gaps are driveway geometry, lead was only 3.2 m at entry; necessity exemption correctly let ev4/ev5's required-1.205 commands through, flagged ev3's near-zero-closing case), and the cap mechanism behaves as designed. **A longer engaged rolling-traffic drive is required for the on-road P1 confirmation.**

**DECISION — next step: build the anti-stiction terminal pre-release (P2 step 2) now; the next on-road drive must be a longer engaged rolling-traffic drive.** Gate 0.01 holds cleanly (keep it, no revert) and the causal A/B closed against the handoff hypothesis, so the grab is command-shapeable and gate 0.01 is the enabler (openpilot now owns the terminal). The pre-release is sim-developed against `settle_peak_imu_jerk` + the hill-hold/StopReq-A fixtures, so it is NOT blocked on the thin gate-0.01 corpus and can start immediately. Design + adversarial invariants recorded in rollout_plan.md Stage 3.C P2 step 2: a bounded, rate-limited, double-edged shaped notch in the terminal ~0.25 m/s (ease accel toward ~−0.2…−0.3 m/s², never to zero/positive, then re-apply the hold at wheel-standstill), with a kinematic-necessity override (a closing lead cancels it) and invariants that it never creeps/rolls on grade, never delays the stop, never fails to reach the hold. In parallel the next drive — which P1 also needs — grows the gate-0.01 engaged-settle population beyond n=1 (ideally rolling-traffic stops to match the StopReq-A baseline) so the pre-release's on-road effect is measurable. P1 cap stays deployed; gate stays 0.01.

Files: event store `~/.comma/stopping_behavior/event_store/events.jsonl`; IMU settle metric computed by `tools/stopping/build_event_store.py:settle_imu_jerk`; gate threshold `terminal_max_settle_imu_jerk = 30.0` at `tools/stopping/scoring_config.py:124`; StopReq gate `STOP_REQ_MAX_SPEED = 0.01` at `opendbc_repo/opendbc/car/hyundai/carcontroller.py:24`.

## 2026-06-14 (anti-stiction pre-release) — deployed 44371049fc

The 2026-06-14 driveway routes were mostly UNENGAGED (00001720 gate-0.04 ~0% engaged; 00001721 gate-0.01 fully unengaged, 0 events; only 00001722 engaged = sub-1 m/s driveway crawls with the driver riding the gas). So: gate-0.01 standstill hold not positively exercised (no failure, but vEgo never dropped <0.01 while engaged); IMU grab A/B n=1 (one driveway settle 48 m/s³, harsher than the 0.04 baseline 23.7 — leans further against the SCC-handoff being the cause, but population-mismatched); P1 on-road NOT confirmed (no engaged approaches). Keep gate 0.01. **Need a real engaged rolling-traffic drive.**

Built + deployed the **anti-stiction terminal pre-release** (P2 step 2) since it's sim-developed and not blocked on the corpus: ease the brake off the deep hold toward a -0.30 m/s³ floor (jerk-limited 1.5 m/s³, release-side only) in 0.06-0.30 m/s, then re-apply full hold below 0.06; gated off instantly on disturbance/grade-pull/rebound-arrest/release-lock/insufficient-decel. Composes with the hold-acq soften (deepening-rate cap) + P2 terminal cap without double-shaping (release floor vs deepening rate). Sim: safety invariants all hold (no creep/rollback, reaches standstill, full re-hold, no under-braking, lead overshoot ~0.3 cm); the FELT grab reduction is unproven by sim (plant has no stiction/IMU) — on-road IMU next drive. Safety review: safe-with-watch, no fatal; one major (NEAR_HOLD recovery slower than HOLD-phase rebound_arrest → ~1 cm extra creep if a grade-pull hits mid-ease) downgraded to a first-drive watch item (within the "few cm" bar; floor stays a real holding decel). 199 tests pass + the known :1792 pre-existing fail; zero new ruff. **First-drive watch:** grade-stop creep, settle not floaty, and settle_peak_imu_jerk vs the 24/48 baseline. Revert = drop the pre-release lane.

## 2026-06-15 (P1 closed) — approach harshness is PLANNER-OWNED; v1 cap reverted, no v2 built

**Outcome: NO controller change.** The reverted P1 approach-decel cap stays OFF
(`APPROACH_DECEL_CAP_ENABLED=False` in longcontrol.py:99 + stopping_controller.py), and a
safe-by-construction v2 (trim DEEPER toward aTarget, never SHALLOWER) is **not viable** because the
unnecessary harshness lives in the planner, not downstream. The precondition the task set for
building v2 ("harshness is downstream-added") is not met.

**Why the v1 cap was reverted (recap + safety sweep).** On route 00001725 seg8, following a
decelerating lead at ~15 m/s, the planner asked aTarget=−1.78 m/s² but the cap pinned the command at
−0.50 for 8.3 s (gap 27→7.5 m, still closing) → driver took over (near-collision). Two root flaws:
(1) the release formula `required_decel = closing²/(2·max(gap−2,ε))` IGNORES the lead's own
deceleration (aLeadK), so against a braking lead it never released; (2) the longcontrol gate
(longcontrol.py:828) has no upper-speed bound, so the cap was live during 15 m/s following, not just
low-speed stop approaches. A sweep reconstructing the exact reverted floor on both cap-live routes
(1725 + 1726, both commit 329b1926a1) found this was NOT a one-off bookmark hazard: a SECOND
essentially-identical high-speed takeover on **1726 seg15** (~933.5–936.0 s: lead decelerated
12.1→8.6 m/s, planner aTarget deepened correctly to −2.08, cap pinned the command at exactly −0.50
for 2.5 s, PID state, v ~12.8 m/s, gap 24→17.5 m, withheld up to 1.58 m/s² → driver brake+disengage)
plus 3 more driver-brake corrections (1726 seg15 ~967.8 s planner −1.33; 1726 seg14 ~863.4 s; 1726
seg3 ~191 s). The revert was justified by field evidence, not just the one bookmark.

**Why no v2 — the corpus says the planner owns it.** The June P1 diagnosis ("4/6 over-0.5-m/s²
approaches unnecessary, required-decel 0.02–0.34 vs commanded 0.59–0.85") had only command vs
kinematic-required — NOT the planner aTarget. Re-ran the classification over the full 233-event
store with `accel_cmd` AND `a_target` per frame (npz traces in
`~/.comma/stopping_behavior/event_store/events/`). For every engaged, moving (v > 0.30), lead-present,
gap > 2 m frame whose commanded decel exceeds the 0.5 cap AND the kinematic-required + 0.12 margin
(the "unnecessary-harsh" signature), compared command to aTarget and inferred the long-control state:

- **Total unnecessary-harsh candidate frames: 95,803.**
- **command == aTarget within 0.10 (planner-owned): 83,305 = 87.0%.** The planner asked for the deep
  decel; the PID command tracked it. June's specific events re-checked WITH aTarget have aTarget
  within ~0.00 of the command (e.g. the planner asked modestly harder than strict gap-2 kinematics
  need — not a downstream stage adding braking).
- command DEEPER than aTarget (the only bucket a never-shallower cap could trim): 3,754 = 3.9%. Of
  these, only **31** sit below even the Santa Fe `pid_brake_model_alignment` floor; the other 3,723
  are accounted for by: (a) message-interleave timing skew — 79–100% of deeper frames match an
  aTarget value within an ±80 ms window (median |window-delta| 0.02–0.11), i.e. command tracks aTarget
  through a fast transient sampled a few frames off; (b) the Santa Fe brake-alignment margin
  (0.03–0.18 m/s², longcontrol.py:152–170); (c) the low-speed terminal lane (the 00001688 seg10
  "residual" deeper frames are all at v 0.54–0.64 m/s, where the StoppingController / low-speed glide
  caps own the command and aTarget is intentionally not the reference). Genuine sustained PID
  free-following downstream over-braking ≈ 0.
- command SHALLOWER than aTarget (cap actively under-braking on the cap-live routes): 2.5%.
- low-speed terminal stopping regime: 6.6%.

**Conclusion.** Approach braking belongs to the planner. A v2 that may only trim braking DEEPER
toward aTarget and never shallower would have essentially NOTHING to trim — the planner is the source
of the deep demand. Making the planner-demanded approaches gentler requires commanding SHALLOWER than
aTarget, which is exactly the unsafe under-braking that caused the two takeovers. An effective
gentleness change must happen IN/BEFORE the planner (longitudinal MPC comfort/jerk-accel cost
weighting, or lead/stop-distance shaping), where it is just as exposed to the closing-lead threat
that broke the cap and is NOT safe-by-construction — out of the stopping-stack scope. **No controller
change made.** v1 cap stays OFF. The comfort program's command-shapeable work is the terminal
pre-release (P2), already deployed and awaiting an engaged rolling-traffic drive.

**No tests added / no code changed** — analysis + docs only (this is the "planner owns it" branch).
Files touched: `docs/stopping/rollout_plan.md` (Stage 3.C1 row, Stage 3.C P1 narrative, decision log),
`docs/stopping/worklog.md` (this entry). Verified the existing tests still pass and ruff is clean
(no new findings) since the controller code is unchanged. **Deploy recommendation: keep the cap OFF
as it is now (no deploy needed for this branch — the docs change carries no runtime effect).**

### 2026-06-15: Post-P1 adversarial audit of the 3 OTHER live comfort changes — 2 KEPT, terminal pre-release GUARDED OFF

After the P1 cap revert, swept the legacy-StoppingController / longcontrol comfort changes that were
each sim-reviewed but never adversarially FIELD-swept for the P1-class failure mode (command brakes
LESS than the planner aTarget while a lead is closing, OR creep/rollback/under-hold at a stop). The
P1 lesson carried in: P1's own review checked under-braking but on only 3 driveway stops and missed
both the no-upper-speed-bound and the lead-decel-blind release — so for each change I asked what its
original review similarly under-tested, and re-read the gate + effect frame-by-frame.

**Verdicts:**

1. **gate-0.01 StopReq latch** (`opendbc_repo/opendbc/car/hyundai/carcontroller.py:128-140`,
   `STOPREQ_LATCH=True`, gate `STOP_REQ_MAX_SPEED=0.01`, release `STOPREQ_RELEASE_SPEED=0.10`) →
   **CLEAN, KEEP.** 134 engaged settled holds examined (14 on gate-0.01-live commits): zero rollaway,
   zero cruise/ACC fault at standstill, latch never held StopReq on a rolling car (the 0.10 release
   fires correctly everywhere — it IS the upper bound P1 lacked). Code-confirmed it is a HANDOFF
   signal (does the SCC own the managed stop vs openpilot's command), not a brake floor, so the P1
   root flaws do not translate; release only hands the hold back to openpilot's stopping command
   which keeps braking. **Coverage caveat:** gate-live full-stop data thin (14 holds, mostly the 1725
   corpus); no grade-stop landing in the [0.01,0.10] dither band captured (in-band settle leaves the
   hold to openpilot's command = legacy behavior, field-shown to hold). No revert.

2. **hold-acquisition soften** (`selfdrive/controls/lib/stopping_controller.py:2676-2695`,
   `HOLD_ACQUISITION_SOFTEN_*`) → **CLEAN, KEEP.** 705 traces / ~22.8k candidate frames: zero
   rollback/under-hold attributable to the soften. Code-confirmed it caps `brake_step` (the per-frame
   DEEPENING rate) ONLY — never `target`, never `release_step`; in
   `clip(target, last−brake_step, last+release_step)` a smaller brake_step only RAISES the lower
   bound, so it structurally cannot release brake or push the command below the target. Hard v<0.05
   upper bound; no floor and no release path → neither P1 root flaw (lead-decel-blind release, no
   upper bound) applies. The single net-creep run (171c, max 1.13 cm/s) was should_stop=False with
   the command DEEPENING into a departing lead — the opposite of under-hold. No revert.

3. **anti-stiction terminal pre-release** (`selfdrive/controls/lib/stopping_controller.py:2828-2849`,
   `A_TERMINAL_PRERELEASE` etc.) → **GUARDED OFF (`TERMINAL_PRERELEASE_ENABLED=False`).** This is the
   one change that shares P1's EXACT structural blind-spot class:
   - the firing gate (2828-2849) is LEAD-BLIND and aTARGET-BLIND — no lead_v / lead_d_rel / closing /
     planner-aTarget term anywhere in it;
   - it RAISES the command toward the −0.30 floor (reduces braking);
   - it runs AFTER the static `clip(limited_output, stop_accel=−2.0, …)` at line 2729 with NO re-clamp
     against the LIVE planner demand (stop_accel is the static −2.0 platform floor, not aTarget);
   - the only lead-aware downstream net, `low_speed_close_lead_accel_cap`
     (`longcontrol.py:221`, applied 680-684), is gated `0.12 <= v_ego` — so the band
     **v ∈ (0.06, 0.12) is lead-blind AND has the cap OFF**.
   The field counterfactual found 0 P1-signature frames, but the audit itself called this "a
   structural argument, not observed data": safety rests on an EMPIRICAL regularity (planner terminal
   demand always relaxes shallower than −0.30 by the final settle; the slow +0.015/frame ease never
   reaches the floor while a lead is still closing), NOT an explicit guard. That is the identical
   "clean field sweep over an unobserved structural hole" that let P1 ship and caused two
   near-collision under-brakes. No prerelease unit test exercises a closing lead at all.
   **DECISION:** on supervised L2 we do not keep a lead-blind brake-REDUCING terminal shaper live on
   empirical safety alone. The felt disc-grab it targeted is actuator stiction (the command lever is
   ~exhausted — already established 2026-06-13/-14), so the upside is small and the risk is P1-class.
   Staged kill-switch `TERMINAL_PRERELEASE_ENABLED=False` (default-off, mirrors the P1 revert's
   `APPROACH_DECEL_CAP_ENABLED=False`), wired into the gate at line 2829. Re-enabling requires either
   an explicit live-aTarget re-clamp INSIDE the controller OR extending the lead-aware cap down to
   cover (0.06,0.12) — i.e. close the structural hole, do not rely on the regularity.

**Tests / lint:**

- Staged guard: `selfdrive/controls/lib/stopping_controller.py` (`TERMINAL_PRERELEASE_ENABLED=False`
  constant + gate term). Tests that assert the pre-release MECHANISM fires now force-enable the flag
  locally (helper `_run_terminal_prerelease_frame` toggles it; the seed test `…_9cb_event3` and the
  multi-frame convergence test use `monkeypatch`) so the mechanism stays verified for an eventual
  re-enable. The `…_disabled_*` tests are unaffected (they also patch the band empty).
- Targeted suite `test_stopping_controller.py` + `test_longcontrol_commit_b_equivalence.py` +
  `test_longcontrol_fast_release.py`: **251 pass, 1 fail.** The 1 failure
  (`test_stopping_controller_low_rollout_soft_landing_release_step_not_too_aggressive`) is
  **PRE-EXISTING and UNRELATED** to all three audited changes: it is already failing on the clean
  HEAD working tree, originates from the foundational commit `3204868077`, fires
  `low_rollout_soft_landing_cap` / `end_stop_cap_active` / `ineffective_brake_guard` (none of the
  three changes), and toggling the pre-release off does NOT change its output (verified). Out of
  scope for this audit and not introduced by the guard — flagged separately.
- ruff: **zero new** (176==176 on the two changed files vs clean HEAD; all pre-existing E501 on the
  long npz seed-data lines, none on added lines).
- **NOT committed / NOT deployed** — the orchestrator owns deploy.

Files touched: `selfdrive/controls/lib/stopping_controller.py` (kill-switch),
`selfdrive/controls/lib/tests/test_stopping_controller.py` (force-enable flag in the 3 mechanism
tests), `docs/stopping/rollout_plan.md` (3.C2 guarded-off row + decision-log audit entry),
`docs/stopping/worklog.md` (this entry).

### 2026-06-16: FIRST engaged drive on the consolidated SAFE stack — under-braking RESOLVED, comfort ACCEPTABLE, V2 triage is now the only real work

First engaged session on the consolidated safe stopping stack (P1 approach-cap OFF
`APPROACH_DECEL_CAP_ENABLED=False`; terminal pre-release OFF `TERMINAL_PRERELEASE_ENABLED=False`;
arbiter behavior-neutral; shouldStop falling-edge hold; honest telemetry v2 = carOutput sent accel;
`PUBLISH_TRUE_LEAD_DISTANCE=True`; gate-0.01 StopReq + hold-acq soften; planner owns approach braking;
V2 DARK). 6 new routes 00001727..0000172c. Read-only review — no code/flag/deploy change.

**Commit/era audit (each route verified in the store, not assumed): CLEAN.** Every event on the two
routes that produced events (0000172a, 0000172c) carries `controller_commit = 80e84c1f21…` =
the expected device HEAD (P1 off, pre-release off). NO event on any earlier commit where P1 or the
pre-release were still live → the entire engaged population reflects the consolidated safe stack.
(The brief flagged f11ff331a1 as the other behaviorally-identical commit; none of these events landed
on it, and none on a pre-revert commit.)

**Engaged-data shape (the binding constraint).** Only route 0000172c was driven engaged (868 s,
~100% enabled on the captured segments); the other five are effectively manual (engaged_s ~0.2,
gasPressed-dominated). 5 store events on 172c + 1 on 172a; IMU settle metrics compute only on
rlog-fetched segments, so honest terminal-grab data exists for exactly **2 distinct physical engaged
stops** — 172c seg3 and seg18. Verified in the store: the 5 settle records collapse to two values
(seg3 decel 0.404 ×3 under overlapping detector windows; seg18 decel 0.476 ×2). This is a 2-sample
probe, not a distribution.

**(1) UNDER-BRAKING — RESOLVED (user's belief confirmed with evidence).** This was the expected
outcome of turning P1 off, and the engaged window proves it directly. NO under-braking takeover
anywhere across all 6 routes' qlogs (88 segs) + the 5 fetched engaged rlogs. The P1 clamp-below-aTarget
signature is GONE: comparing carOutput accel_cmd (the sent value, telemetry v2) vs longitudinalPlan
aTarget at 100 Hz over every engaged approach (v > 2 m/s, planner commanding decel), `|cmd − tgt| < 0.10`
for 98–100% of samples on all 5 windows, and cmd median == tgt median to within 0.001–0.01 m/s². The
command faithfully TRACKS the planner — exactly as predicted with the downstream cap removed. The old
"car wouldn't slow down" failure does not reproduce; the planner owns approach braking and there is no
clamp under-braking it.

**(2) COMFORT — ACCEPTABLE; no new real issue surfaced (one honest caveat).**
- *Approach (now planner-owned, P1 gone):* comfortable, NOT felt-harsh. `approach_necessary=False`
  on all stops, zero sustained hard braking (`hard_decel_duration_s = 0.0` everywhere), peak approach
  command −0.48…−0.90 m/s². The user's original harsh-approach complaint is not reproduced — and
  removing the cap did not reintroduce harshness. (Caveat: this engaged route had gentle approaches;
  it is not a hard-braking-into-traffic stress sample.)
- *Standstill hold @ gate 0.01:* CLEAN on both true engaged holds. seg18 (the solid 5.1 s hold):
  creep_after_stop 0.042 m/s, speed_rebound 0.0017 m/s (negligible). No creep failure, no ACC dropout,
  no failure-to-hold, no reaccel-before-hold on either. The inverse risk of gate 0.01 (failing to get
  the managed hold) again did not materialize, now on genuine rolling-traffic engaged stops.
- *Terminal grab (honest IMU, pre-release OFF):* PRIMARY `settle_peak_imu_decel` (gate > 0.80) =
  [0.404, 0.476], med 0.44, max 0.476 → **0/2 over the primary gate**, well under. The grab is
  materially gentler than the historical baselines (StopReq-A median decel 0.59; the felt 00001722
  driveway grab 0.99). HONEST NUANCE the gate logic requires surfacing: the SECONDARY channel
  `settle_peak_imu_jerk_raw` (gate > 13.0) splits — seg18 = 5.5 (clean), seg3 = **41.1 (over)**. Under
  `classify_event`'s OR logic, seg3 WOULD raise `harsh_terminal_grab` on the secondary channel even
  though its primary decel is gentle. Read honestly: seg3 had a sharp but SHALLOW settle transient (a
  brief sub-100 ms jerk at a low overall decel), not a hard grab — the kind of single-event secondary
  spike the n=2 sample cannot distinguish from noise, and the pre-release that targeted exactly this is
  OFF by design (guarded for sharing P1's lead-blind blind-spot). NOT a blocker, NOT a regression vs
  baseline; logged as the one thing a larger engaged corpus should re-check. Net: comfort is acceptable
  to rest on; no harsh-terminal pattern, no approach harshness, clean holds.

**(3) Bookmarks:** NONE. Zero userBookmark/bookmarkButton events across all 6 routes (88 qlog segs +
5 full-rate rlogs). The user pressed no bookmark this session.

**DECISION — the consolidated safe stack is in a good, safe, acceptable state; STOP adding stopping
behavior. The remaining real work is V2 triage, and this session does NOT unblock it.** Under-braking
is resolved (P1 off, confirmed on-road); comfort is acceptable (planner approach comfortable, holds
clean, terminal grab gentle on the primary gate). There is no comfort defect worth a new controller
change — manufacturing one (re-arming a downstream cap or the pre-release) would re-open exactly the
P1-class lead-blind risk we just closed. This is a fine place to rest the comfort program.

The V2 flip is blocked on **163 unclassified tier-2 similarity-gate events** (decision log 2026-06-13),
NOT on data volume — and this engaged session does not change that. It adds engaged data (good for the
honest corpus and any future paired_stats), but V2 triage is an OFFLINE classification task against the
existing failing gate (eval.md §5 taxonomy / spec-7.7), not something that needs more drives. So: no
new stopping deploy; the orchestrator's next move, if it wants to advance the stack, is to start the
tier-2 triage (classify the 163 events, decide class-B parameter moves vs Tier-1 re-scope per
rollout_plan Stage 2). If the orchestrator instead wants to soak, the stack is safe to soak as-is.

Files touched: `docs/stopping/rollout_plan.md` (status table + decision log),
`docs/stopping/worklog.md` (this entry). No code/test/flag/deploy change (read-only review).

### 2026-06-18: seg24 close-lead coast-in fix — stopping-phase planner-aTarget floor (DEPLOYED)

- **Incident (bookmarked):** route `0000173c` seg24, near-collision driver takeover *during stopping*.
  Verified provenance: `0000173c` is the most recent local route (drive start 2026-06-18 10:58 CEST);
  the userBookmark is in seg24 at 11:27:03, pressed at vEgo≈0.02 m/s (right after the takeover). The
  car coasted to a **1.36 m min gap** behind a STOPPED lead before the driver braked (aEgo to -2.06).
- **⚠️ GRADE CORRECTION (2026-06-18, user-flagged):** the original diagnosis called this a "-6.2%
  downhill." **It was NOT a slope — the road was essentially flat (~0.2% grade).** The "downhill" came
  from reading `livePose.orientationNED.y` (≈-0.066 rad) as road grade when it is the **camera mount
  pitch**: seg24's median orientationNED.y (-0.066) is *identical* to the route-wide median (-0.066,
  8 segments sampled), and the mount-independent (aEgo - cmd) bias over 1657 rolling-coast frames was
  only **+0.015 m/s²** (a -6% downhill would show ~+0.6). Grade was a mislabel; the real defect stands.
- **Root cause (rlog-traced, grade-independent):** the planner stop target `distanceToStopTarget`
  correctly collapsed to its 0.05 m close-hold pin once dRel reached `LEAD_STOP_DISTANCE_TARGET` (4.0 m
  — the intended rest gap *behind* the lead). That handed command to the legacy `StoppingController`'s
  terminal glide/hold lane ~4 m early, which **flat-floored the command at -0.12 m/s²**. At the takeover
  the car was at **v≈1.25 m/s with only ~2.2 m to the stopped lead** — kinematics required ≥ **-0.36
  m/s²** just to stop in the gap, and the planner aTarget correctly demanded **-0.52** — but it was
  **discarded** at `output_accel = stop_result.output_accel`, so -0.12 under-braked **on flat ground**
  and coasted to 1.36 m. The dts "collapse" is CORRECT semantics, NOT a bug — dts is remaining distance
  to the intended stop point, 4.0 m behind the lead. The 2026-06-12 downhill commit `2aff68d02a` was
  IRRELEVANT here (it only acts >12.5 m/s; this was 1–2 m/s).
- **Fix (`0ecb745f2c`, safe-by-construction — the INVERSE of the reverted P1 cap):** in
  `LongCtrlState.stopping`, with a close lead present and the planner still demanding decel deeper than
  the controller command, take the DEEPER of the two: `output_accel = min(output_accel, a_target)`. A
  one-way DEEPEN — can only ADD braking, never reduce it, so it cannot under-brake and cannot repeat the
  P1 failure. Gated to Santa Fe HEV, v_ego > 0.30 m/s (clear of the terminal hold), stop-relevant lead
  within 12 m, a_target ≤ -0.10. Grade-agnostic by construction: the planner aTarget already reflects
  whatever net decel the situation needs (flat or graded), so no separate grade term. Kill switch:
  `STOPPING_PLANNER_FLOOR_ENABLED` (`longcontrol.py:132`, set False to revert). New helpers
  `stopping_planner_floor_active` / `should_apply_stopping_planner_floor`.
- **Proof:** open-loop rlog replay of seg24 through the real `LongControl.update` (flag off vs on,
  13,638 frames) — baseline coasts to contact (kinematic gap-hold reaches min gap ~0 m, matches the real
  incident min dRel 1.36 m); flag-on tracks the planner (-0.50..-0.68) and holds ~3.0 m (> 2.5 m bar).
  One-way-deepen proven: 0 frames shallower than baseline across the seg23/seg24 corpus (734 deepened);
  synthetic stress 47,040 frames 0 violations; broad fuzz 71,622 frames found only 82 sub-0.006 m/s²
  downstream rate-limiter path-divergence transients (0 > 0.01 m/s², net braking never reduced on any of
  800 trajectories) — NOT the floor reducing braking, and below actuator/IMU resolution.
- **Verify:** `selfdrive/controls/lib/tests/` → **509 passed, 19 skipped**; ruff clean. Commit-B arbiter
  per-frame equivalence preserved (oracle mirrors the floor; AST allowlist names
  `stopping_planner_floor_active` as a lead_d_rel_eff consumer).
- **Adversarial safety verdict: `safe-with-watch`.** Two minor findings, both deliberately NOT
  code-fixed (the path-divergence transient is structural to any rate-limiter downstream of a
  state-carrying command change — forcing a strict floor-last would bypass jerk-limiting; the unused
  `lead_v` gate param is harmless signature symmetry).
- **Deployed:** pushed `!my-fp-new` → device `fullupdate.sh` → device HEAD `0ecb745f`, flag confirmed
  `True` on-device. **First-drive watch list:** (1) close stopped-lead approach at 1–2 m/s now brakes per
  the planner (~-0.4..-0.7, may feel firmer than the old coast-in glide); (2) standstill-hold crossing
  (v through 0.30 m/s, floor disarms) — watch for any firmness→gentle snap; (3) stop-relevant leads in
  the 4–12 m band while rolling >0.30 m/s may now deepen toward the planner; (4) lead-free / far-lead
  (>12 m) stops are provably bit-identical — any change there is NOT this fix.
- **SEPARATE seg26 recommendation (flagged to user, NOT changed here):** a high-speed under-brake at
  v~11.6 m/s sits just BELOW the `2aff68d02a` downhill cap's lower gate of 12.50 m/s
  (`SANTA_FE_DOWNHILL_STOPPED_LEAD_HIGH_SPEED_BP[0]`). The stopping-phase floor does NOT cover it (PID/
  approach state, not stopping). The hard 12.50 m/s gate is brittle — consider extending the downhill
  cap's lower breakpoint to ~10–11 m/s or grade-blending across the 10–13 m/s seam. This is a
  `longitudinal_planner.py` tuning decision for the user.

Files touched: `selfdrive/controls/lib/longcontrol.py` (+84), `test_longcontrol_fast_release.py` (+11
tests), `test_longcontrol_commit_b_equivalence.py` (oracle mirror), `test_stop_target_arbiter.py` (AST
allowlist), `docs/stopping/worklog.md` (this entry).

### 2026-06-18: V2-unblock track — gate failure is a PLANT artifact, V2 verified safe (DECISION: document+soak)

User redirected the program away from adding downstream exceptions ("ethology") toward the principled
de-sprawl: **finish V2 (planner-owned stopping) and delete the forest.** A design workflow + adversarial
critique rejected "invert ownership inside longcontrol now" (amputates live tuned forest lanes, P1-class
risk on a thin proof) in favor of finishing V2. Then an offline-only investigation of the V2 gate:

- **Plant decision (re-fit vs re-scope) is MOOT** (independently verified — re-ran the fit sweep + re-derived
  DC gains). RE-FIT not viable: stable poles exist only at delays 0–3 but ALL have wrong-sign standstill
  gain K(0)<0; the only correct-sign stable plant IS the near-integrator artifact (φ≈0.974). RE-SCOPE
  off-target: the failures aren't in the sub-0.21 entry band.
- **Quick-win:** the existing `estimator_equivalence_20260613.json` report was failing the gate only because
  it wasn't attached. Attaching it flips that Tier-1 row to PASS on both plants.
- **hold_gap (the last red Tier-1 row) is a PLANT artifact, not a V2 deficiency** — trajectory-verified.
  The refit/frozen plant's DC gain inverts below ~0.21 m/s (deeper command → *higher* predicted velocity),
  producing a non-physical 0.18 m/s plateau that traps V2's correct, *firmer* terminal command while
  rewarding the forest's −0.12 flat-floor. 58/67 hold_gap divergences are command-equivalent where the
  plant is valid; the split is purely terminal. Real car stopped in **147/147** divergence events.
- **Full Tier-2 triage: ZERO class-C.** All 152 flagged events classified (committed:
  `docs/stopping/tier2_triage_20260618.json`, 191 refs across both plants, all class-A with evidence
  notes). With it + the estimator attached, the gate's Tier-2 deterministic exit is satisfied (0 class-C,
  0 unclassified both plants) and Tier-1 on the honest refit plant passes everything EXCEPT the documented
  `hold_gap` artifact (leapfrog/rollout/tts/end_jerk/estimator/harsh_no_v2_only/dropout all PASS).
- **Independent adversarial verification (4 attackers + skeptical judge): zero class-C SURVIVES (medium
  confidence).** Linchpin honest-plant stop test: V2 reaches a safe standstill on EVERY physically-realistic
  plant (drag≥0.03 / actuator lag / friction) and is firmer-or-equal to the forest on approach (66 firmer /
  19 softer / 47 equal). No safety under-capture. Two **non-blocking** "needs-human" items, both retired by
  an on-road hold trace during soak: (1) a standstill ±0.025 m/s² command sawtooth (V2 `d_hat` single-frame
  estimator tracking quantized wheel-aEgo at `DIST_LPF_TAU_S=0.0`; felt-neutral in sim) — candidate small
  pre-flip deadband fix in `stopping_tracker.py`; (2) `00001720 seg1` V2 brake-release at v≈0.5 m/s on an
  idealized frictionless plant only (cured by any drag/lag; real car stopped).

**DECISION (mine, per user delegation): document + soak — do NOT massage the gate metric.** Leave `hold_gap`
honestly red as a truthful "the sim cannot adjudicate the terminal below 0.21 m/s" signal; the flip is a
documented, evidence-backed decision gated on a supervised on-road soak (spec's "sim develops, road
promotes"). Path to flip: (1) [done] commit the triage table + this record; (2) optional pre-flip d_hat
deadband hardening; (3) flip `USE_STOPPING_V2=True` behind the kill switch + deploy; (4) supervised soak,
capture one on-road hold trace to retire the two watch items; (5) cleanup-delete the forest + all stopping
caps + the seg24 floor (the de-sprawl payoff). Artifacts: /tmp/gate_triaged.json, /tmp/honest_*.json,
docs/stopping/tier2_triage_20260618.json.

### 2026-06-19: FOREST DELETION — the de-sprawl payoff (deployed)

After V2's soak (stop-and-go + a high-speed 14→0 approach + a 73s hold, all within spec, no
sawtooth/coast-in; settle nod accepted as physics-bounded), deleted the dead legacy stopping stack.

- **SCOPE CORRECTION (verified in source):** the seg24 planner floor + close-lead/far-lead/glide caps are
  NOT dead under V2 — they post-process the V2 facade output in longcontrol's stopping branch, gated on
  Santa-Fe-HEV + live arbiter `decision.*` fields, NOT on `USE_STOPPING_V2`. They are KEPT (deleting them
  would change V2 and re-open the seg24 coast-in). Only the legacy forest + kill-switch forks + the two
  already-disabled families were removable. (An early mapping agent mislabeled the caps as dead; caught
  and corrected before any edit.)
- **Deleted (8,430 lines):** `stopping_controller.py` (2,892-line legacy forest); the `USE_STOPPING_V2`
  kill switch + its 3 forks (instantiation/dispatch/slew collapsed to the V2-only branch); the dead
  `APPROACH_DECEL_CAP_*` family in longcontrol (ENABLED=False); legacy tests
  (`test_stopping_controller.py`, `test_longcontrol_commit_b_equivalence.py`, the flag-pinned cases in
  `test_longcontrol_fast_release.py`). Necessarily also the now-obsolete legacy-vs-V2 eval tools that
  only existed to compare the forest to V2 (`tools/stopping/similarity_gate.py`,
  `rescore_prerelease_friction.py`, + their tests) and the `run_stopping_cycle` gate stage; `sim_replay.py`
  repointed to V2-only. The reusable eval primitives (stopping_plant, scoring_config, paired_stats,
  build_event_store, sim_replay) remain.
- **V2 runtime BYTE-IDENTICAL (proof):** `test_stopping_v2_replay.py` + arbiter trio = 112 passed/18
  skipped, identical to the pre-deletion baseline. Full controls/lib/tests 358/19 (−156 = the deleted
  legacy tests, zero failures). tools/stopping green. ruff clean on the changed runtime + tools.
- **Revert** is now `git revert` of the cleanup commit (the in-place one-bool kill switch is gone).
- Executed in an isolated worktree by a focused agent, then the runtime diff was human-audited (V2 path
  unchanged, live caps untouched) and re-verified on the main tree before adopting + deploying.

Files: `stopping_controller.py` (del), `longcontrol.py` (−142, forks collapsed), `stopping_tracker.py`
(stale comment), test/tool deletions per above, `docs/stopping/worklog.md` (this entry).

---

## 2026-06-20 — Force-coast no-lead standstill fault: re-diagnosed + properly fixed (db889cd638, deployed-staged)

The cruise fault RECURRED after the 2026-06-19 StopReq-on-override frame fix (c06b9ea4db) — user:
"that part didn't work." That fix was based on a **wrong mechanism**. Re-diagnosed and **proven**:

**Root cause** — `longitudinal_planner.py:899` forced `output_should_stop=True` for
`forceCoast and standstill`, which holds via the SCC **managed stop (StopReq)**. With **no lead** the
car's TCS treats the managed stop as invalid and **disables ACC (`accFaulted`)** the instant the driver
gas-overrides it. The frame fix couldn't help — the fault is the car reacting to *being in* a no-lead
managed stop (override at t=51.5 → ACC off → `accFaulted` t=52.3, StopReq already 0).

**Corpus proof** (25 gas-resume-from-stop events across 00001744–00001759):
- no-lead force-coast resumes: **2/2 faulted**; lead-backed resumes: **0/23 faulted**.
- `distanceToStopTarget = -1` at both faults → `should_stop` was force-coast-forced, **not** the MPC.

**Fix** — `apply_force_coast_standstill_hold()`: only force the managed stop when an actionable lead
(<8 m) backs it; otherwise hold via a firm openpilot brake command (`FORCE_COAST_NO_LEAD_HOLD_ACCEL`
= −1.0, **no** `should_stop` → **no** StopReq) so the resume is a plain override the SCC accepts. This
also lifts force-coast's firm braking off the gentle no-lead stop (user's 2nd point: smoother no-lead
stop). 4 new CI regression tests; logic verified standalone + ruff/compile.

**Trade-off accepted by user** ("ship the fix, you test"): the firm hold IS the StopReq we removed, so
the no-lead hold now rides the −1.0 brake command — firmness/feel is the on-road variable.
**On-road watch**: (1) no-lead force-coast stop holds without creep, esp. on a slope; (2) cruise fault
gone on gas-resume; tune `FORCE_COAST_NO_LEAD_HOLD_ACCEL` from feel. Deploy is **staged** (car was
on-road); live after next off-road reboot. Review cursor → 0000175a (0000175a clean: no engaged stops).

Files: `longitudinal_planner.py` (+helper +2 consts, line-899 block gated), `test_longitudinal_planner.py`
(+4 tests), `docs/stopping/review_cursor.json`, `docs/stopping/worklog.md` (this entry).

---

## 2026-06-20 (cycle 2) — Force-coast fault was a V2 HOLD-MAGNITUDE regression (fix 2a105acc2d; db889cd6 reverted)

The cruise fault RECURRED on route 0000175f while running db889cd6 (my prior "fix"). User: "compare to an
older my-fp-new / backup branch — it's a V2 regression." **Correct.** Branch-diff vs baseline a02630ba23
(backup/!my-fp-new_06-01, 2026-05-31, last known-good) + fault-log replay found the REAL root:

**The no-lead force-coast standstill HOLD MAGNITUDE — not StopReq, not should_stop.** Baseline held this
case FIRM (~-0.32..-0.34 m/s², legacy StoppingController no-target branch). V2 holds the uniform gentle
A_HOLD ~-0.13 relaxing to -0.10. The car's TCS rejects unwinding from the shallow -0.12 hold on a gas
tip-in → accFaulted; it did NOT reject the firm baseline hold.

**Fault-log proof (0000175f seg3, db889cd6):** held command was **-0.12** (NOT my -1.0 → my fix never
reached the wire); accFaulted fired AFTER StopReq was already 0 (StopReq is not the discriminator — why
Fix2 + db889cd6 both failed). The -1.0 was discarded because the stopping branch ignores a_target (uses
the V2 tracker A_HOLD) and the longcontrol force_coast+standstill clamp was only `min(output_accel, 0.0)`.

**Fix:** firm that clamp to `min(output_accel, FORCE_COAST_STANDSTILL_HOLD_ACCEL=-0.32)` — on output_accel,
which reaches the wire. Force-coast standstills only; normal lead stops keep the gentle V2 hold (0/23
faulted). Reverted db889cd6 (9ea41849df). StopReq latch (9be7dd361e) exonerated.

**Meta-lesson (3 wrong fixes):** replay the fault log to see the actual wire command before theorizing,
and diff against the last known-good branch EARLY — the regression was a behavioral magnitude change,
invisible to frame-level reasoning. **ON-ROAD TEST pending:** no fault on force-coast no-lead gas-resume;
hold feels firmer (baseline). Deployed detached while on-road (reboots when parked).

---

## 2026-06-20 (cycle 3) — Rest-gap too close on lead stops: carry-to-target + ISD setting (e8e70f5bab)

User bookmark: V2 lead stops smooth but rest too close (~2 m; wants >=2.5 m) without losing smoothness;
suggested handing off to the stopping code earlier. MEASURED (00001751 seg11, rested 2.99 m behind a 4.0 m
target; new bookmark not yet synced -- last routes 00001761/62 had no engaged stops): the planner aTarget
RELAXES early (-1.0 @ dToStop 1.7 m -> -0.7 @ target -> -0.26 @ rest), so the car reaches the stop point
still rolling ~0.9 m/s and coasts ~1 m past. The terminal is already deeper than the relaxed planner, so
STOPPING_PLANNER_FLOOR (deepen-to-aTarget) can't help. NOTE: the literal "hand off earlier" would backfire
-- the stopping terminal is the GENTLE part; handing to it sooner brakes LESS. The lever is arriving slower.

Fix = "both modest" (user chose): (1) rest-gap TARGET via the existing user setting IncreasedStoppedDistance
(rest gap = LEAD_STOP_DISTANCE_TARGET 4.0 + ISD; PUBLISH_TRUE_LEAD_DISTANCE=True). User sets ~0.7 m, live,
no smoothness change. Did NOT bump LEAD_STOP_DISTANCE_TARGET (wide ripple: MPC/approach caps/follow/traffic).
(2) code carry-to-target in longcontrol stopping branch: while ROLLING toward a close lead-backed target,
hold kinematic v^2/2d brake (cap -1.05) so it lands nearer the target. One-way DEEPEN, gated v in (0.45,2.2)
+ remaining in (0.15,2.5) + lead-backed -> never fights the gentle final hold or the no-lead force-coast
hold. Verified lint/compile + simulated on the measured frames. Deployed e8e70f5b (device rebooted into it).
ON-ROAD: rest >=2.5 m, approach as smooth; raise ISD if still close, lower STOP_TARGET_CARRY_CAP if firmer.

---

## 2026-06-20 (cycle 4) — Lead-stop rest gap bounded [2.5,5.0]: ISD + close-gap creep (a24d76d15c)

Arc: lead stops were too CLOSE (~2 m) -> raised target via IncreasedStoppedDistance (ISD) setting + a
"carry" (reverted as inert) -> then too FAR in stop-and-go (5.7-6 m, abrupt). User relaxed bound to
[2.5,5.0]. ROOT of the far-stops (measured route 00001764 seg27): behind a CONFIRMED STOPPED lead the car
braked to a near-stop ~1 m SHORT of target (rested 5.7 m TRUE, dts~1.0) and HELD -- the glide cap
(low_speed_stopped_lead_glide_accel_cap) is a gentle brake with no stop-position target, the arbiter
far-stopped crawl is gated off when dts<=1.8, V2 can't command positive accel. StopReq=0 at the hold
(Kalman vEgo dithers >0.01) so the car is held by the soft command -> a tiny positive aReq can move it.

FIX (a24d76d15c): stateful latched slew-limited FORWARD creep in longcontrol, applied as the LAST writer
of the stopping-state output_accel (after all caps, before the force_coast hold) so the glide brake can't
clobber it. Arms at standstill behind a confirmed stopped lead with eff gap clearly above target; disarms
by gap (ISD-aware hard floor)/lead-departure/force_coast/overspeed. POSITIVE-ONLY (no lower-bound lane).
Santa-Fe-gated + STOPPING_CLOSE_GAP_CREEP_ENABLED kill switch. Eff-space gap math (lead_d_rel_eff=true-ISD);
eff target+floor ISD-aware clamped so TRUE rest in [2.5,5.0] for all ISD.

PROCESS (the discipline that was missing on the earlier blind fixes): TWO adversarial-verify workflow rounds.
Round 1 REJECTED the first design (a wrong-sign lower-bound relax lane that eased braking toward a close
lead -> coasted CLOSER; + an inert creep clobbered by the glide cap) -> did NOT deploy. Round 2 produced the
corrected design (0 blockers), fixed 2 majors (ISD double-count, state scoping). Then verified the 2
make-or-break items the workflow couldn't run: StopReq=0 at the hold + accel_limits clip [-3.5,2.0] (creep
reaches the wire). Integration sim: 5.7->4.6 m eff, peak 0.22 m/s, disarms, no oscillation. AST-guard test
+ 6 new creep unit tests pass; ruff/compile clean. Deployed (device a24d76d1). ISD=0.3 (target 4.3) live.
ON-ROAD: stop-and-go creeps to ~4.3-4.6 m not 6 m; never <2.5 m; tune CREEP_ACCEL_MAX if too slow.

## 2026-06-26 — Roll-in floor: far near-stop fix (deployed 42f6a3f7bd)
Bookmark (model deep_rl3): behind a confirmed creeping-then-STOPPING radar lead the car near-stopped
~8.5 m back then slow-crawled ~1.5 km to ~4 m. Root: MPC OVER-brakes (-0.4..-0.64) to match the slowing
lead, near-stopping at the follow gap before any stop target exists. The deployed smooth-approach CAP
(min/deepen) can't fix an over-brake. Fix = its MIRROR: santa_fe_stopping_lead_roll_in, a max()/RAISE FLOOR
(longitudinal_planner.py) that lifts the over-brake up to the same gentle stop-at-hold-gap decel (shared
get_santa_fe_stopped_lead_hold_gap_required_decel), so the car rolls in continuously. Measured frame
(v=1.3,dRel=9.7,lead->0): floor=-0.157, raises -0.50->-0.157; rolls 8.5->~5.6 m then hands off to the
arbiter stopped-lead control + the close-gap creep (a24d76d1) -> ~4 m. Safety (user: "don't slam"): gated
OFF whenever longcontrol is/could be stopping -- mirrors output_should_stop OR should_enter OR should_hold
AND the arbiter's synthetic stopped-lead target on the ISD-EFFECTIVE gap (convention-exact w/ longcontrol
lead_d_rel_eff); off under force-coast; latched off 0.8 s on a lead hard-stop; v in [0.30,2.50), lead_v<=0.55,
closing<=2.3, TTC>=4 s; carry-past guard (hard MPC brake passes through, never carries past the hold gap).
Santa-Fe-gated, kill switch SANTA_FE_STOPPING_LEAD_ROLL_IN. ~5 adversarial-verify rounds (under-braking is
entangled with the anti-collision nets; verify caught 4 holes). Final: under-brake SAFE + regressions SAFE,
17 unit tests (test_santa_fe_stopping_lead_roll_in.py), eff-gap re-confirm 0 hazard frames on a grid scan.
ON-ROAD (unexercised, needs engaged stop-and-go behind a stopped lead): continuous roll-in to ~4 m, NO
~8.5 m near-stop + crawl, no harshness/slam. Revert: SANTA_FE_STOPPING_LEAD_ROLL_IN=False.

## 2026-06-28 — Arrived-state gate: stop a stopped lead churning the car in to ~2m (deployed 0be14f05f4)
Review of new routes >00001766 (68 routes, tar-streamed qlogs+rlogs). Healthy: roll-in-floor routes
(00001ae8/00001aea, commit 2cb25d66) rest 3.8-5.1m, terminal jerk 0.6-1.1 (good). DEFECT (route 00001786 seg5,
measured full-rate): low-speed creep-up to a STOPPED lead rests ~2m + harsh terminal grab (IMU jerk 10-11). Root:
radar dRel bounces 2.0<->3.9m at close range; the synthetic stopped-lead target get_stopped_lead_control_target
re-asserts on every low read, re-arming low_speed_stopped_lead_glide_accel_cap's near_hold_gap_cap (gated by
stop_request_active OR stopping) -> repeated firm -0.74..-0.81 re-grabs; the bounce-driven brake/release cycle walks
the car inward 3.1->2.0m. FIX (arrived-state early-return in the producer, kill switch
STOPPED_LEAD_ARRIVED_GATE_ENABLED): once v_ego<=0.35 AND dRel<=4.30 return None -> synthetic stops re-asserting ->
clean dropout -> should_hold_recent_close_stopped_lead_dropout holds the car in ONE stable hold instead of the
off<->stopping alternation. Trigger anchor (2.75) + rest expr unchanged (no AST-pin break); roll-in floor gate
(keys on is-not-None) preserved; close-gap creep (arms >4.7m) unaffected. Adversarial verify: dont-slam SAFE
(dropout-hold holds the car, no creep into lead), regressions SAFE; 109 tests (3 new), logic-replay suppresses
40/40 bounce-grab frames. Downstream rest code-traced (local env can't full-replay). ON-ROAD WATCH: low-speed
creep-up to a stopped lead holds in one smooth settle, no repeated re-grabs, rests ~3-4.5m not ~2m.

## 2026-06-28 — Leapfrog diagnosis (NOT yet fixed; terminal-glide design in progress)
User bookmarks pinpointed: 00001adb seg30 (jerky-lead chase, NOT the leapfrog), 0000178a seg19 (THE leapfrog),
00001788 seg10 (highway oscillation, separate). User clarified leapfrog = car fully stops then makes a small forward
creep with the lead NOT moving. Measured (0000178a, engaged, with carControl OUT): car brakes -0.54 (3x the -0.16
kinematic ideal) -> near-stops ~0.8m SHORT of its 4.3m target at 5.1m -> brake eases near standstill (OUT -0.54 ->
-0.08) -> HEV creep torque rolls it fwd to 4.3m -> re-brakes. Still present on roll-in build (00001aea engaged:
stop@4.87->creep+0.60->4.27) -- roll-in floor SHRANK it (0.90->0.2-0.6m) but gives up below ~5m (synthetic) and
below 0.30 m/s. INSIGHT: leapfrog == terminal OVER-brake (glide imperfection); a perfect glide (decel reaches 0 AT
the target) is both smoother AND creep-free -- they are the SAME goal, the tension is only for symptom-hacks. User
bar: "something very good", smoother not traded, LESS tree not more. Design: one unifying terminal glide law that
owns the final approach (judge panel + adversarial verify in progress).

## 2026-06-29 — Terminal glide: stopped-lead leapfrog (deployed 30676dcdc3)
User-clarified leapfrog = car fully stops then small forward creep with the lead NOT moving. Measured (0000178a
engaged, carControl OUT): brake -0.54 (3x the -0.16 kinematic ideal) -> near-stop ~0.8m SHORT at 5.1m -> brake eases
near standstill -> HEV creep torque rolls fwd to 4.3m -> re-brake. Persisted on the roll-in build (00001aea engaged:
stop@4.87->creep+0.60->4.27). Root: low_speed_stopped_lead_glide_accel_cap is a speed/gap brake table with NO
stop-position kinematics; the existing jerk-limited tracker (stop_reference a=-v^2/2d) was fed the wrong target
(2.75m) and overridden by that cap. INSIGHT: leapfrog == terminal over-brake; a perfect glide (decel reaches 0 AT the
target) is both smoother AND creep-free -- same goal, the tension is only for symptom-hacks. FIX (flag
SANTA_FE_TERMINAL_GLIDE_PROFILE_ENABLED, Santa-Fe fingerprint): (1) target->4.0m in get_stopped_lead_control_target
so the tracker glides v=0 at the gap (all producers agree on one target; arrived-gate retired when flag on); (2) firm
hold A_HOLD_FIRM=-0.32 (== FORCE_COAST_STANDSTILL_HOLD_ACCEL, pinned) counters HEV creep at standstill; (3) V-GATED
bypass of the glide cap + close-lead cap ONLY above STOPPING_PLANNER_FLOOR_V_EGO_MIN (0.30) -- below 0.30 byte-
identical to legacy so anti-collision is unchanged (no new under-brake hole by construction). Staged: close-gap creep
KEPT on. TWO PRIOR sub-0.30 attempts FAILED the default-fail under-brake check (retire-everything left a hole;
slow re-armed floor reached CONTACT at creep 0.35) -> replaced by the v-gate. Verify: sub-0.30 anti-collision SAFE
(byte-identical-or-deeper vs HEAD), regress SAFE, works minors-only; 405 tests. ON-ROAD WATCH: stop behind a stopped
lead = ONE smooth glide to ~4m (no near-stop-short, no small forward creep/leapfrog), firmer standstill hold; confirm
landing distribution <=~4.6m before retiring the staged close-gap creep. Revert: SANTA_FE_TERMINAL_GLIDE_PROFILE_
ENABLED=False.

## 2026-06-30 — Terminal glide VALIDATED on-road + distance-gated settle (committed 1fe3bd7003, NOT deployed)
Reviewed 12 new routes >00001aef. 00001af8/af9 on 05e164fd (= terminal glide + arrived-gate + roll-in floor + user's
gas-override fix) = FIRST on-road exercise of the terminal glide. **VALIDATED: terminal glide works** -- 00001af9 (14
engaged stops) terminal IMU jerk 0.3-1.8 m/s3 (was 10-11 on the leapfrog), held stops land cleanly at 3.7-4.4m with
dToStop=0.05 (committed in ONE motion, no stop-short+creep). The leapfrog is GONE and stops are smooth. NEW finding:
a few stops SETTLE ~1.1m SHORT of their 4.0m target (rest 5.37-5.40m, distanceToStopTarget ~1.1) and hold; the after-
stop close-gap creep commands +0.02 but can't break static friction (car doesn't move, dRel bounces 4.65-5.48 on radar
noise). Root: the SETTLE declaration (stopping_tracker.py) accumulates settled_time_s on VELOCITY ALONE (v<=0.02) with
NO remaining-distance guard -> freezes into HOLD ~1.1m short; the distance-blind TERMINAL near_hold comfort floor bleeds
the speed off early. USER LEAPFROG TAXONOMY (decisive): OK = ONE continuous motion to a SINGLE stop (incl. a slow
creep-in to close); DISLIKE#1 = full settle then a second go; DISLIKE#2 = full settle then a pointless ~10cm nudge. So
the after-stop close-gap creep IS Dislike#1. FIX (committed, behind SANTA_FE_TERMINAL_GLIDE_PROFILE_ENABLED + Santa-Fe
fingerprint, new tracker.update kwarg default False -> other cars bit-identical): distance-gated settle -- don't
accumulate settled_time_s while remaining_m > NO_SETTLE_REMAINING_M(0.50) so the car glides the residual gap to ~4.0m
as ONE motion then settles once (firm A_HOLD_FIRM=-0.32 re-arms at target); DWELL ESCAPE (anti-hang) settles anyway
after 1.20s at v<=0.06. Creep KEPT ON (staged backstop). Verify: no-overshoot SAFE (facade min(u,-0.05) clamp -> never
carries past the rest point), no-hang SAFE (dwell escape fires 1.19s, traced vs the real 109s v=0 case), regress
minors-only; 411 tests. HONEST CAVEAT: fixes the arrives-MOVING case (one continuous glide); an already-fully-stopped-
short stop just holds (no second-go, no hang). COMMIT-ONLY per user -- NOT deployed; on-road is the proof. 2.50m close
stop (stop#11, high-speed 2.2m/s late-commit) and the 00001af0 highway/disengaged bookmark = separate/open.

## 2026-07-01 — Full-stack review (user: "are we optimal? rewrite OK") — verdict + V3 plan (cursor->00001b71, NO deploy)
Reviewed 22 new routes >00001afb (engaged stopping in 00001b05/09/0a/6c/6e/6f; 00001b70/71 on ca47fc7118 ~fully
disengaged). 41 engaged settles deep-analyzed with a consistent honest method (20Hz livePose IMU jerk, taxonomy,
rest gaps, frame traces). HOLDS: zero DISLIKE#1 leapfrogs, clean holds, no sawtooth, rests mostly in [2.5,5.0].
**DOES NOT HOLD: the felt wheel-stop grab persists on most stops** — terminal IMU jerk median ~5.5 m/s3 (2-10.4),
settle peak decel 0.6-1.2 (8/29 above the 0.80 harsh gate). Frame trace (00001b6c seg4): planner aTarget glides
-0.86 -> -0.12 as dts -> 0 (correct!) but the WIRE stays pinned -0.60..-0.81 through wheel-stop, releasing ~1.2s
AFTER standstill; IMU shows -0.80 held to v=0.08 then +0.27 rebound = the felt grab. ROOT: (a) sub-0.30 legacy-cap
re-enable (longcontrol.py:446-458) + low_speed_stopped_lead_glide_accel_cap clip floor -0.60 whose activation gaps
(<=4.15-4.2m) COVER the normal resting zone -> the "anti-collision net" binds on every nominal stop; (b) synthetic
stop target pins remaining=0.05m while still rolling (stop_target_helpers.py:160, trigger_gap ~= rest_gap) and
clobbers the planner's honest distance via the arbiter min(). METHOD CORRECTION: the 06-30 "jerk 0.3-1.8 validated"
figure was a measurement artifact; same-method A/B on 00001af9 gives median ~5.2 -> settle-gate era = no regression,
no terminal-feel improvement. Close-gap creep now measurably causes DISLIKE#2 nudges (2/41). High-speed approaches
rest close (2.1-2.9m). Bookmark 00001b09 seg4 = driver brake takeover on a slowing-queue approach -> ca47fc7118
fixed it at the WRONG layer (48-constant table cap under the PID; planner/MPC is the principled home). PROCESS
VERDICT: post-forest-deletion the stack regrew to ~31 writers / ~1,050 tuned values / the glide law duplicated at
8 sites in 12 days — the forest is regrowing around V2's blind spots. OUTPUT: docs/stopping/stopping_service_v3_plan.md
(judge-panel synthesis, clean-slate single-writer stopping service, 5-phase lifecycle, 3 always-live deepen-only
safety lanes on 3 sensor families, ~25 physical constants, net ~-3,800 lines, staged SHADOW->LIVE_TERMINAL->LIVE->
delete, per-stage one-drive gates, honest DoD: jerk median<=2.5/p90<=4.0, wheel-stop wire in [-0.35,-0.05], rebound
<=0.10 — Stribeck floor ~2 m/s3 makes lower targets unfalsifiable). NO runtime change this review.

## 2026-07-01/02 — Leapfrog re-diagnosed (HOLD ESCAPE, not creep) + fix deployed + V3 stage 0/1 built
User: "leapfrogs still happen (last two days)". Dedicated detector over all 35 fresh-route rlogs found 3
FULL_SETTLE_LEAPFROGs (00001b05 seg3, 00001b6c seg2, 00001b6e seg14) with ONE shared signature: settle 1.0-1.3s
-> 5-13cm forward escape -> re-brake. Fine trace: the car settles at v~0.03-0.05 (never <0.01, no StopReq), the
hold sits at -0.25 for ~1s while IMU shows HEV creep torque building, then the car BREAKS FREE against the -0.25
brake. ROOT (seam gap): the trajectory's firm hold A_HOLD_FIRM=-0.32 was re-clamped back to the quiescent
end-stop ceiling (-0.255 near v=0) by the tracker's step-2 re-clamp, which applies to SETTLE/HOLD too — the firm
hold NEVER reached the wire on a quiescent stop (tests asserted a_ref only, never through-tracker u). The current
stack couples: gentle glide-out => escape risk; deep cap-pinned stops (-0.60) => grab. FIX 75fde63ad4 (deployed):
release the ceiling to A_HOLD_FIRM in SETTLE/HOLD under the firm-hold scope only — wire ramps -0.255->-0.32 in
0.12s (~0.6 m/s3) AFTER wheels stop (silent); TERMINAL glide-out unchanged; other cars bit-identical; through-
tracker regression tests added. Also RETIRED the close-gap creep (kill switch False, post-stop motion is disliked
by construction; innocent in these 3 traces but same class). V3 STAGE 0+1 BUILT + DEPLOYED DARK (4ca981babf):
stopping_service.py + stop_context.py + stopping_telemetry.py + 40 tests + sim_replay adapter; SERVICE_MODE=SHADOW
(zero wire impact, proven byte-identical + diff-audited). Adversarial rounds fixed: Stribeck creep-crawl through
D_HARD (monitor in GLIDE v<=0.5, ratchet survives phase flips), a_coast deepen-only <0.1, D_REST_eff entry anchor,
EASE ff once, hysteresis, telemetry caps — each with fail-before/pass-after proof. NEXT DRIVE WATCH: (1) NO
settle-escape leapfrogs (the 3 routes' signature gone); (2) hold firmness at -0.32 not felt as a grab (ramps
after wheel-stop); (3) rests land in-band WITHOUT the creep; (4) shadow telemetry present in rlogs
(stopping_service cloudlog events) + zero shadow-observer disarms. Stage-2 gates in the 4ca981babf message.

## 2026-07-02 — STAGE 2 LIVE: service owns the terminal band (v<=0.85), shadow-drive gate waived
User waived the shadow-drive gate (L2, driver supervises — same basis as every prior live terminal change);
offline default-fail under-brake gate RETAINED and run on the deploy SHA (0bbccc235a), both plants, 274 scenarios
x 2 = 1,096 paired sims: **ZERO persistent under-brake on deployable frames**; 5 transient 0.1s frames attributed
(2 = budgeted persistence-filter cost <0.002m closure, 3 = sim takeover-seed artifact impossible on-car). The
script's raw verdict prints FAIL on 11 events/plant — I verified the worst cases MYSELF frame-by-frame: (a) ref
plant pushes a HELD car forward with positive accel against -0.32..-9 commands from standstill (the documented
sub-0.21 inverted-DC-gain pathology; deeper hold = more phantom creep — the plant punishes exactly what fixes the
real escape-leapfrog); (b) refit zeros are phantom-lead extension tails (recording ends with a real go at v~1.5,
sim freezes the departed lead, service INACTIVE placeholder frames drive into it); (c) remaining deltas are the
designed closer rests, all >2.0m. Same harness limitation as the V2 flip; same resolution (on-road is the
terminal-band judge). Wiring (adversarially reviewed SAFE, cold-context finding fixed): takeover keys on
prev-frame ownership (takeover frame legacy-capped), jerk reseed from live wire, handback v>0.95 w/ continuity,
exception -> pre-takeover value + drive-scoped latch; service+context observe the full v<2.5 band (warm filters);
sub-0.30 cap family bypassed ONLY on service-owned frames; seg24 floor + force-coast hold stay. 752 tests green.
**REVERT = stopping_flags.SERVICE_MODE = "SHADOW" (one line).** ON-ROAD PROTOCOL: parking lot first — 3 stops +
gas tip-ins from held stops (TCS watch: accFaulted/AVH), then normal stops. Watch: wheel-stop wire gentle (no
-0.60 grab), hold firm ~0.5s after stop, no escape-leapfrog, no coast-in, rests 2.4-5.2m. Any DISLIKE/TCS
fault/takeover -> flip the flag back before analysis.

## 2026-07-02 (later) — First LIVE drive reviewed mid-drive + STAGE 3 (full-band ownership) built, gated, deployed
Route 00001b72 (bookmark seg7): first stop from 9.85 m/s VERY SMOOTH (fix confirmed); the go-hesitation +
brake-to-stop at 40m with a receding lead = PLANNER/MODEL (cmd==aTgt in pid/starting; go-abort-go; radar flipped
to a slow 2nd target; deep_rl3 family) — NOT stopping code; second stop harsh = planner aTgt slam -0.32->-0.81
in ONE frame at v 0.92 in PID state, where stage-2 couldn't own (stopping state entered at v 0.15). STAGE 3
(8921b4e438): SERVICE_MODE=LIVE — service owns every service-active frame <2.5 m/s in pid+stopping states
(slam absorbed at J_SAFE; pid integrator frozen+reseeded per owned frame, stepless handback; C4/C5 pid.i gated
off on owned frames). Monitor queue-creep gate: arming suppressed only when the lead is GENUINELY receding
(gap growth >0.03m/0.4s AND lead_v>0.15) + one fresh hover window after suppression lifts — the offline gate
caught gap-quantization notches from a STOPPED lead masquerading as departure (event 000016dd, -0.32m rest
regression, now back to baseline parity), and Codex's second-opinion review caught the same class via outward
radar walks PLUS an exception-fallback release (owned-frame exception now never releases: min(chain, prev wire)).
Codex's "CRITICAL" (service converges to deep planner demands at J_SAFE rather than instantly) REJECTED with
numbers: <=5 frames on-car, adversarial-inverse verified zero frames stalled above a deeper planner demand —
designed jerk-limited convergence, not P1's sustained pin. Gates on final tree: zero persistent under-brake both
plants; refit fail-set == stage-2 baseline; ref +1 = intended queue-creep on the invalid plant band (attributed
by two independent reviewers). 764 tests, ruff clean. REVERT tiers: LIVE_TERMINAL / SHADOW (one word).
ON-ROAD WATCH (next drive): approach stops now glide through the planner-slam class (no jerk-8 stops); queue
creep no longer arms the monitor (hold lands -0.32 on flat stops); no under-brake on genuine lead hard-stops
(a_plan passes deep demands at J_SAFE); rests 2.4-5.2m.

## 2026-07-02 (stage-3 live drive) — TOO-CLOSE STOP (2.1 m) root-caused + fixed same-session (097af23f40, staged on-road)
Route 00001b76 seg4/5 bookmark: stop-and-go queue, lead decelerated to a stop at gap ~5.2 while ego did ~1.6.
The service entered as the lead stopped and the D_REST_eff entry anchor -- feasibility = A_GLIDE_NOM 0.5 comfort
glide -- re-zeroed the rest to ~2.7 m (formula: 5.4 - 1.65^2/1.0). The service then gilded PRECISELY to its wrong
anchor (cmd -0.68 vs planner -0.43, service-owned, working as built), the shallow tail let creep push 0.3 m more
(monitor caught it, ratcheted -0.35->-0.80), rest = 2.1 m radar. FIX: A_REST_FEAS=1.2 m/s2 for the anchor ("can
the car FIRMLY land at nominal", matching the 0.8-1.2 the planner was already demanding -- not "can the gentlest
glide reach it"); anchor keeps re-computing while the lead is still moving (entry geometry not final). Genuine
close entries (gap ~3.0) still re-zero to 2.4-2.85. Incident fixture reproduces 2.67 m under the old anchor.
Gate: 0 persistent under-brake both plants, 3 baseline refit min-gap failures RESOLVED (farther rests), 0 new.
765 tests. Staged on-road; applies at next parked reboot. WATCH: stop-and-go rests land 3.5-4.5 m.

## 2026-07-03 — Stage-3 HOLD escape through legacy far-stopped close-gap release (00001b82 seg40)
Route `00001b82--fc9c2370e8--40` bookmark at route t=2436.04 s: the felt stop-go/leapfrog began about 10 s before
the bookmark. Evidence: raw planner `shouldStop=True` through the first stop, lead was still effectively stopped
(`vLeadK` ~0.16-0.25 m/s, gap ~6.0-6.5 m), and `stopping_service` telemetry showed `APPROACH_GLIDE -> RAMP_TO_HOLD
-> HOLD` by t=2425.90. The bug was the wire, not the model: immediately after service HOLD, the old far-stopped-lead
close-gap release suppressed `state_should_stop`, Hyundai `starting` escaped underneath the active service hold, and
the wire went `stopping -> starting -> pid`, producing a small go pulse and then legacy re-stop commands down to
~-0.95 m/s2. Fix: in `SERVICE_MODE="LIVE"`, when the service is already in `RAMP_TO_HOLD/HOLD`, block only the
legacy far-stopped-lead close-gap release from leaving `stopping` unless a real departing-lead release is present.
This preserves genuine lead-departure release while enforcing the stage-3 architecture: settled service stops are not
allowed to fall through retired post-stop close-gap motion lanes. Regression pinned in
`test_live_hold_blocks_far_stopped_lead_starting_escape`.

## 2026-07-04 — Cycle-4 review (00001b73-00001b89) + hold hardening deployed (5bf274af72)
Full-corpus detector sweep (takeover/leapfrog/harsh/bookmark) + rlog traces. TAKEOVER CLASS (user bookmark
00001b88 seg14 + unbookmarked seg9/00001b82 seg6 + 3 harsh stops from 14.5-22 m/s): NOT the wire — seg14: engaged
at 11 m/s onto a stopped lead ~45m out, planner wound up ~1s then tracked to -3.3 (near full authority), still
arrived 2.3 m/s at 3.5m; driver braked, rest 1.6m. Hot-approach envelope = PLANNER/MODEL domain (cmd==aTgt, above
the service band) — DECLARED NEXT PROJECT; not fixable safely downstream (P1 lesson). LEAPFROGS: (a) pre-06e6f5ed
starting-escapes (b82 segs 39/40/44) — fixed by 06e6f5ed, validated zero on e6008984; (b) HOLD ESCAPES on today's
code (b87 segs 1/3): car breaks loose from -0.32..-0.43 holds, 6-16cm nudge, monitor re-arrests -0.65 every time
→ FIXED: A_HOLD -0.45 + post-stop fast arrest (floor A_HOLD-0.25 at J_SAFE on post-latch motion; fixture ≤5cm).
VALIDATED from logs: A_REST_FEAS anchor (rests 3.3-4.4m) + far-lead-release fix. Wheel-stop wires -0.25..-0.35 ✓;
first-stop jerks 3-7 → felt-smoothness (DoD median ≤2.5) = target after the approach-envelope project. Gate script
lost with scratchpad; change deepen-only (no under-brake surface); task chip to rebuild gate in-repo. Device 5bf274af.

## 2026-07-06 — Cycle-5 REGRESSION review + terminal un-regression deployed (269379c80f)
User: "most stops harsh; smooth while the model controls it, then stopping mode turns on -> harsh." Quantified over
26 new routes: jerk median 5.5/p90 9.5/max 14.6, wire@stop -0.36..-0.45 (band -0.35..-0.05), holds -0.70..-1.15.
THREE terminal regressions, two self-inflicted by the cycle-4 hold hardening: (1) post-stop Kalman dither (v reads
0.03-0.05 while parked) false-armed the fast arrest -> -0.70 on nearly EVERY stop = the felt grab; (2) RAMP built
the -0.45 hold while still rolling (early latch) = deep arrival; (3) glide-law blowup on hot arrivals (d_rem pinned
at its 0.15m floor -> -1.76 while the planner relaxed to -0.88; 00001ba3 seg28 jerk 14.6) — the new planner
brake-earlier commits (9e3ae640/ffbbadad) deliver firm arrivals into exactly this. FIXES: post-latch detection =
velocity-roll (v>=0.05 rising) + TRUSTED-gap DISPLACEMENT lane (>0.15m below the latch reference; frozen on
dropout/unmeasured frames, upward re-base only on genuine departure >0.3m) — displacement separates dither (zero
net travel) from crawls (steady gap consumption) and catches sub-quantization crawls no velocity bar can see;
gentle-finish window (max -0.35 while motion remains, never releasing an inherited deeper wire; hold builds once
genuinely stopped or 1s latched); terminal anti-blowup (comfort glide re-anchors the rest closer, one-way, floored
at D_REST_MIN, when its demand would exceed A_REST_FEAS — land nearer instead of slamming; a_kin/a_plan retain
unlimited depth). ADVERSARIAL REVIEW verdict: initial 3-fix set UNSAFE (post-latch crawl blindness -> probed
CONTACT at Stribeck+grade 0.10-0.20, 2.7-20m blind travel); the displacement lane + reviewer must-fixes (hoist
above the standstill early-out; dropout/notch false-arm immunity) close it. F3 attacked across a hot-entry grid:
zero new D_HARD breaches. No-lead stop-line grade crawl = accepted residual (documented; driver present, no
collision object). Planner commits left in place (mid-approach firmness = intended takeover-catch; their harsh
arrivals were regression 3, now absorbed). 772 tests, 6 new fixtures. Deployed + device-verified 269379c8.
WATCH next drive: wire@stop back in -0.35..-0.05, holds resting -0.45 (not -0.70), hot arrivals land ~1.2-capped
with no terminal slam, no crawl-through on grades.

## 2026-07-06 — Stop-and-go early-brake complaint review: planner brake-earlier commits EXONERATED (no deploy)
User hypothesis: the planner commits 9e3ae640a8 ("brake earlier for late stopped leads") + ffbbadadf1 ("catch high
speed stopped lead approaches") over-rotated the other way — braking much earlier in normal stop-and-go and keeping
huge gaps behind leads that are moving and barely slower than us. Method: offline 3-generation recompute of the
stopped-lead smooth-approach cap (OLD pre-9e3ae640 / MID 9e3ae640 / NEW ffbbadad) + the slowing-lead cap from qlog
radarState across three build groups: BASE=Jul-3 00001b7e-00001b88 (39f2b988/e6008984), MID=00001b8b-00001b95,
NEW=00001b96-00001ba3 (Jul-5 eve + Jul-6). Reimplementation verified bit-exact vs the real functions (1.66M-point
grid diff = 0.00000, reproduces all 6 commit-test assertions); 4-agent adversarial verify pass: 4x CONFIRMED.
FINDINGS: (1) the new code paths were essentially INACTIVE — on the NEW build ~1h engaged, new-only firing = 2
radar frames (~0.5s), max deepening vs OLD tables 0.12 m/s2; the gates (lead vLead<=0.35-0.55, closing>=4.5-9.5,
TTC<=4.2-6.4, d_rel caps) structurally exclude moving-lead following, and the caps only run in blended mode anyway.
(2) Follow time-gaps did NOT widen: banded + personality-conditioned p50 is EQUAL-OR-TIGHTER on NEW in every speed
band (pooled p50 2.40->2.31s, frac>4s halved); calm-lead early-brake onsets 4.7/min BASE vs 2.0/min NEW — the
complained-about behavior was MORE common in the Jul-3 baseline. Confound caught: personality mix (BASE 27%/MID 53%
relaxed vs NEW 100% standard) inflates baseline gaps — always condition cross-day gap comparisons on
selfdriveState.personality. (3) The felt at-speed braking = the PRE-EXISTING slowing-lead queue cap (b3a7989604,
06-06) + MPC responding to genuinely braking leads — deepest run today (-2.27, 00001ba3 t~6168) had a single stable
radar track braking at aLeadK -2.9 from 13 m/s to a full stop (justified; slow-cap tracked within 0.03). A
"phantom" candidate (t~5945) was also real: the lead fully stopped ~3s before resuming. (4) ONE REAL DEFECT: the
only new-only fire (00001b97 t~3926.6) was a 2-frame radar track-association glitch (vLead 10.6 -> -0.03 at
v_ego=14) that the 12.5->16 m/s band extension made cap-eligible -> -2.0 m/s2 actuator pulse ~0.35s (no takeover,
braking independently warranted; 1-frame repeat t~3930). Hardening candidate (task chip spawned): require
stopped-reading persistence (>=3 radar frames) or vLead<->vRel consistency before the late-approach path fires
above 12.5 m/s. Also noted: separate longcontrol-side PID_STOPPED_LEAD_APPROACH_* early-brake (37ab992566, 06-19)
exists on BOTH sides of the comparison (not a delta; blind spot of the cap-only scan). NO runtime change deployed —
the commits do what they were designed to do and rarely fire; the complaint's mechanism is pre-existing queue-stop
behavior responding to real lead braking.

## 2026-07-06 (addendum) — The remaining felt leapfrog = ABORTED-GO RE-ENTRY slam; entry grace deployed (389ea287b0)
Both-variant leapfrog sweep over the cycle-5 corpus found ONE instrumented leapfrog (00001ba2 seg11, the user's
bookmark). Full anatomy: smooth stop -> dither false-arrest to -0.85 (fixed by 269379c80f) -> lead crept off ->
go-pulse (+0.70, starting) -> lead stopped again -> go ABORTED; the service re-entered MID-RISE (v 0.13 rising to
0.40 on launch momentum, gap 6.3, glide demand ~-0.04) and the monitor read the momentum as roll/hover evidence
(running-min latched at the 0.13 entry reading; hover window full of the rise) -> slammed -1.00..-1.15 at v 0.40 =
stop, lurch, harsh yank = THE felt leapfrog. FIX: 0.5s ENTRY GRACE (roll reference re-seeds continuously; hover
suppressed; hover window starts clean at grace end) so motion evidence only accumulates under service control;
genuine post-entry rollaways still arm from the fresh reference (fixture). WATCH next drive: queue go-aborts coast
back down gently (no yank), plus the 269379c80f watch items (wire@stop in band, holds -0.45, no grade crawl).

## 2026-07-07 — Cycle-6 validation PASSED + secure-hold close-out (e8b96c3f48)
First drive on the cycle-5 fixes (00001ba4-6): watch list PASSED — wire@stop -0.35 on every stop, holds -0.44/-0.45
(dither false-arrest gone), rests 3.3-4.85m, jerks 4.4-6.7 (no slams, was up to 14.6), no takeovers, no bookmarks.
Residual: 2/5 stops micro-escaped the -0.45 hold (5-7cm, arrested cleanly at -0.70). Close-out: A_HOLD_SECURE -0.70
silent build after A_HOLD — the empirically-always-holds level from the arrest ledger (~20+ events, all held),
applied only while parked (felt-free; deep holds proven to release cleanly). Review battery moved into
tools/stopping/review/ (scratchpad-loss lesson). NEXT: felt-smoothness polish (jerk median ~6 vs DoD 2.5),
planner hot-approach envelope project, stage-4 deletion after a clean drive on e8b96c3f.

## 2026-07-10 — Cycle-7 regression: restore the validated model + close HOLD authority leak (not deployed)
User reported the whole stop-and-go drive as harsh, with multiple autonomous rests around 8-10 m instead of the
2.5-5.0 m acceptance band. All three physical bookmarks were treated exactly like the rest of the corpus: two are
in route `00001c90` segment 134 and one is in segment 142.

CORPUS COMPLETION: exact device-size reconciliation covered every completed route counter after `00001ba6` through
`00001c91`: 1,140 valid finalized qlogs across the 30 completed route counters. Three old segment directories with
`rlog.lock` (`00001bb0/118`, `00001bb1/4`, `00001c8d/3`) were excluded under the normal route-sync policy; the only
qlog in one-segment route `00001bb9` is corrupt on the device itself and was also recorded as unusable. A full
engaged-stop sweep found no autonomous full stop outside `00001c90` except one in `00001c8e` (gap 4.70 m, jerk
3.3 m/s3, wheel-stop wire -0.44). The newer route `00001c92` was still live at segment 142 during final inventory,
so it remains pending and the review cursor advances only through `00001c91`.

FULL `00001c90` REVIEW: all 211 qlogs and 21 selected event/boundary rlogs were validated. The qlog sweep proposed
23 candidates; 100 Hz hold validation removed four micro/near-stops that never formed a distinct 0.5 s hold, leaving
19 physical autonomous stops. Rest-gap median was 6.6 m, p90 10.2 m; 17/19 were above 5.0 m and only 2/19 were in
the 2.5-5.0 m band. Honest 20 Hz terminal jerk was median 6.8 / p90 12.9 / max 15.8 m/s3. Only 4/19 wheel-stop
wires met crank-1's intended natural-arrival band. This decisively FAILS the positioning, smoothness, and crank-1
gates; all three bookmarks lie inside the same failed population rather than being privileged evidence.

BUILD STRATIFICATION found no stopping-code change after `c29f878673`; `7b7f8b9817` changed only the default
`driving_supercombo.onnx`. On the last validated model, cycle 6 had five real stops at 3.3-4.85 m with wheel-stop
planner/model demand only -0.08..-0.13 and jerk 4.4-6.7. On the new model, far settles carried roughly
-0.31..-0.53 published `aTarget` at the last rolling frame (matching model desired acceleration in the traced
events), forcing the service's unconditional `a_plan` safety lane deep. The three recent stopping commits are
exonerated by direction and phase: entry grace only suppresses an aborted-go re-entry arrest; secure hold builds
after the car is parked; crank 1 holds the natural arrival instead of deepening it. ACTION: restore the last
validated `deep_rl3` blob (`19a5354c8e524a76cf05970d67247957980f8691`) and retain crank 1 for a clean-model
validation drive. Do not weaken/gate `a_plan` downstream to hide a model regression.

INDEPENDENT CORRECTNESS DEFECT from bookmark/segment 142: after a harsh first settle (gap 5.19 m, jerk 12.9), the
same stable radar track briefly reported ~0.57 m/s while the physical lead remained stopped. Legacy
`departing_lead_release` moved `LongControl` to `starting` even though `shouldStop` stayed true, `aTarget` stayed
negative, and Stopping Service remained in `HOLD`; the wire released, ego travelled 0.43 m, then re-stopped at
6.09 m with jerk 12.9. FIX: in LIVE mode, both legacy release booleans are blocked from leaving `stopping` while
the service is in `RAMP_TO_HOLD/HOLD/RELEASE`; a real departure first enters the service's jerk-limited `RELEASE`,
then may enter `starting` only after that ramp completes and the service resets to `INACTIVE`.

VALIDATION: 202 focused controller/service tests passed; targeted ruff and text-diff checks passed; the restored
ONNX passed `onnx.checker` (IR 10, 481 nodes). No device deploy in this cycle yet. Next-drive gates
are rests 2.5-5.0 m, crank-1 wire/jerk recovery on the validated model, and zero start/re-stop motion while the
service remains in HOLD.

## 2026-07-12 — Cycle-8: adapt stopping code to newest model; model untouched (not deployed)
The live device and local checkout were both `82bfd61d`; the deployed/default driving model was `bb669698aa`
`deep_rl3` with git blob `82b3250ee490aaa886e1d588bd78474174c9a415` on both sides. Per the user boundary for this
thread, the model artifact and model selection were read-only throughout this cycle.

CORPUS COMPLETION: reconciled every completed route above cursor `00001c91`: counters `00001c92-00001c9b` and
`00001e5f-00001e65`, 16 routes / 606 exact-size-matched qlogs. Every qlog passed `zstd -t`; `00001c97/10` was
excluded under the normal policy because `rlog.lock` remains. The complete qlog triage found autonomous stop
activity in only five routes and no `userBookmark`/`userFlag` events in the new corpus. Bookmarks therefore had
no privileged weight (there were none); all stop candidates were processed identically, with selected rlogs for
every candidate/boundary segment.

NEWEST-MODEL RESULT: only `00001e63/8` and `00001e65/22-23` contained autonomous physical stopping episodes.
They produced three settles across two episodes. Rest gaps were `5.8 m` and `5.5 -> 5.2 m`, all beyond the 5 m
maximum. Honest 20 Hz IMU terminal jerk was `6.6` and `8.1 m/s3`. The e65 episode stopped, rolled `0.14 m` while
the lead stayed effectively stationary, then re-stopped; the re-arrest wire reached `-1.0`.

ROOT CAUSE: in both traces, an explicit planner stop target plus measured lead already resolved an implied rest
around `4.7-4.8 m`, but the composite new-model `aTarget` remained roughly `-0.7..-1.2 m/s2` at walking speed. The
service's unconditional `a_plan` lane treated that redundant depth as independent safety authority, overriding
the nominal 4 m phase geometry and stopping early. In e65, wheel-stop then latched on a `-0.31` natural arrival;
the 1.0 s gentle-finish grace held that insufficient pressure until the car rolled, so the monitor correctly but
harshly arrested at `-1.0`.

CODE-ONLY CANDIDATE (simplified after the complexity audit): publish the constraint-resolved trajectory demand as
`aTargetTrajectory` instead of giving the already-merged `aTarget` indivisible safety authority. Whenever a
trustworthy conditioned lead gap exists, preserve `aTargetTrajectory` unmodified and bound only additional
composite/direct-model depth with one relative-speed kinematic law that still stops by the existing `2.5 m`
minimum. The normal phase law remains the 4 m writer; `a_kin` remains live to `D_HARD=2.0 m`; positive coast/grade
deepens the bound. Moving and reversing leads use the same `v_close` equation, with no classification threshold.
No-lead, dropout/decay-held, otherwise untrusted gaps, and missing split data retain raw `aTarget` (fail deep).
Filter-`held` gaps use the same law as `measured` gaps because the phase and `a_kin` lanes already trust that
conditioned signal; this prevents an authority seam at filter transitions. Separately, shorten the natural-arrival
grace from `1.0` to `0.5 s`, then build the silent secure hold at `J_HOLD` before a slow roll needs the fast monitor.

AUTHORITY AUDIT: blended planning has two behavior paths: MPC produces a constraint-resolved trajectory demand,
then `min(output_a_target_mpc, output_a_target_e2e)` lets the direct model action deepen it. In the two newest-model
traces, final `aTarget` matched the direct action on `77/80` and `129/136` terminal plan frames while MPC was already
relaxing. The first geometry-only candidate was therefore incomplete: bounding the merged target could also hide a
legitimate MPC response. The split above resolves that without changing general planner behavior, adding a model
condition, or adding a writer. Global removal of the direct-action path and a second terminal MPC/controller were
rejected. After a clean validation drive, stage 4 should delete the duplicated legacy/V2 terminal writers and state
authority; keeping them indefinitely as a reference/fallback would preserve the very tree this service replaces.

RECORDED-STATE REPLAY (counterfactual arbitration only; not a closed-loop rest-distance claim): old logs do not
contain the new field, so `aTargetTrajectory` was reconstructed from each logged MPC speed/acceleration curve at the
recorded 0.55 s action horizon. Across the full active in-band windows, the split relieved composite depth on
`185/574` frames in e63 (mean `+0.134 m/s2` over all active frames, max `+0.485`; trajectory demand bound 175 of
those frames) and `437/1025` in e65 (mean `+0.156`, max `+0.776`; trajectory bound 232). On the last validated-model
routes 1ba5/10 and 1ba6/3, mean relief over all active frames was only `+0.008` and `+0.012 m/s2`. Safety fallbacks
remained raw. Validation: `262 passed` across the service, context, live-mode, controller and replay suites; the
Cap'n Proto/C++ cereal target builds with the new field and validity bit; targeted ruff, `py_compile`, JSON parse and
`git diff --check` pass. The candidate is not committed, pushed, or deployed. Next on-road gates: rests in
`[2.5,5.0] m`, no stop-roll-re-stop behind a stationary lead, terminal jerk lower than 6.6-8.1, and no regression in
reversing/dropout/no-lead stops.

## 2026-07-13 — Review of cycle-7/8 commits (c0d0d2bf, 6711daf0) + grace-yields-to-evidence
Reviewed the two stopping commits from the parallel threads. VERDICT: AGREE with both. (1) c0d0d2bf (hold through
release): extends the starting-escape blocker to both lead-release predicates + through RELEASE, one-frame wire
continuity on RELEASE completion — consistent with "the service owns the settled stop". WATCH ITEM: launch latency
— a genuine go now waits for the RELEASE ramp from the -0.70 secure hold (~0.6 s at J_GO 1.2); if queue launches
feel sluggish, add a faster hold-release rate (own cycle, own measurement). (2) 6711daf0 (terminal trajectory
authority): the aTarget/aTargetTrajectory split is the RIGHT generalization — direct/composite model depth is
advisory and position-bounded (stop-by-2.5 m at relative speed, creep-deepened), the constraint-resolved MPC
demand is never shallowed, every doubt path fails deep. The authority audit (direct-action bound 77/80 & 129/136
terminal frames while MPC relaxed) justified splitting rather than bounding the merged target; counterfactual
replay quantified relief; fixtures cover never-shallow/fail-deep/reversing/held-gap. This honors the P1 lesson's
substance (never shallow below a demand you can't model — here the bound IS a kinematic model and the MPC demand
is preserved verbatim). IMPROVEMENT SHIPPED: GRACE YIELDS TO EVIDENCE — the e65 roll happened DURING the arrival
grace (insufficient -0.31 arrival held while the car visibly rolled); shortening the window (their 1.0->0.5 s)
reduces exposure, terminating it on observed motion (any v rise >0.02 above the post-latch minimum) closes the
class for any duration. Fixture: shallow arrival + roll at t=0.1 -> hold builds within frames, cm-level travel.
CRANK LADDER: still at crank-1, gate NOT yet evaluated on a validated-model drive (cycle-8 stops were new-model +
a_plan-defect affected) — next review must score wire@stop/jerk/pitch-rate on post-6711daf routes.

## 2026-07-17 — Cycle-9: one settled-stop authority and revision-paired offline verification
The device and checkout were both `bf50f90`; intervening commits after the last stopping build were unrelated live-
update changes. The default `driving_supercombo.onnx` was read-only and identical locally/device-side (file SHA-1
`253497401f3d5e55cd9c91a2dd30a9b9284eef08`, git blob `82b3250ee490aaa886e1d588bd78474174c9a415`).
No model artifact, selection, parser, or model runtime file changed.

CORPUS COMPLETION: processed every counter above cursor `00001e65` through completed `00001eff`, plus all 23
finalized segments (`0-22`) of still-live `00001f00`; active `f00/23` and incomplete locked `eff/16` were excluded.
The reconciled union is 37 routes / 1,159 readable finalized qlogs (36 completed routes / 1,136 qlogs plus the 23
live-route finalized segments). The hybrid sweep found 57 candidate transitions, including duplicate windows and
manual/takeover stops; every candidate was triaged. Five actual qlog bookmark markers existed: three matched stops
(`efe/10`, `efe/59`, `efe/66`) and two marked ordinary non-stop driving (`efc/35`, `ef9/22`). The separately reported
fresh event without a serialized marker was still found by the full sweep (`efe/70`) and received the same review.
Artifacts: `/tmp/stopping_corpus_complete_1784304997/summary.json`,
`/tmp/stopping_corpus_refresh_1784305572/summary.json`, `/tmp/stopping_corpus_f00_1784306528/summary.json`, and their
matching bookmark summaries. All selected high-rate rlogs and finalized qlogs used for decisions passed `zstd -t`.

EVIDENCE: `efe/59` reproduced the stationary-lead false start/re-stop: the service remained in HOLD, but an
unclassified legacy state transition entered `starting`; the ego reached 0.47 m/s and was re-stopped harshly even
though the lead did not move. `e7b/82` and `efe/70` exposed the inverse authority failure: the lead physically drove
away while stale negative `aTarget`/`shouldStop` pinned the service hold; gaps opened past 6 m and to 15.7 m until
driver gas. `efe/66` was a distinct bookmarked 13.6 m stationary-lead rest: ego and lead decelerated together,
service entered from motion near 0.25 m/s, then accepted the far wheel-stop as terminal. `efe/10` showed the comfort
law using the firmer entry-feasibility decel as its terminal ceiling. `efe/7` and the new `f00/15` first settle showed
the natural-arrival hold losing a few centimeters before the parked-rate pressure build; `f00/15` otherwise rested
in band at 4.4-4.7 m, but high-rate IMU jerk was 8.7 and its later 4.3 m settle reached 13.8, so crank-1 still FAILS.

GENERIC CORRECTION: the service phase itself now blocks every legacy `stopping -> starting` escape while settled,
independent of a growing list of release-reason booleans. The same single `RELEASE` phase owns only valid exits:
planner go or 0.5 s of measured physical lead recession with >0.3 m gap growth. The initially considered far-rest
release was rejected: restarting toward an unmoved lead recreates the user's disliked start/re-stop signature. A
stationary far rest is therefore held and fails the rest-gap gate; its cause must be fixed before standstill. No new
phase, controller, positive-motion writer, model branch, or route special case was added. Separately, entry
feasibility stays at 1.2 m/s2 while terminal comfort can re-anchor toward the existing 0.5 m/s2 glide law only when
the resulting rest remains >=2.5 m and the live gap retains an additional 1.0 m margin. Observed post-latch roll
uses the existing `J_SAFE` path instead of waiting on parked `J_HOLD`.

OFFLINE VERIFIER INVESTMENT: `verify_candidate.py` now evaluates a baseline worktree and candidate with the exact
same current replay harness. The replay consumes recorded planner `aTarget`, warms up on recorded state, then
free-runs the simulated ego against exogenous lead/target world paths through the full stage-3 service band. It
measures first-standstill gap, bounded +1.0 s rebound, minimum gap/contact, and confirmed lead departure. The
verifier enforces the no-driving-model-change boundary, stable-key alignment, dual reference/refit plants, paired
MDE-aware statistics, and hard new-contact/sub-2m/lost-settle/harsh/leapfrog regression gates.

FINAL OFFLINE RESULT: 250 event-store stops aligned exactly across both revisions and both plants (500 paired
rows). Verdict `safe_to_road_test`: zero hard regressions and zero statistical regressions. Two model-only warnings
remain: one +0.0082 m/s rebound threshold crossing (below the 0.02 m/s materiality floor), and one leapfrog flag
during a confirmed physical lead departure. Exact verifier tests passed and no model path changed. This verdict is
permission for an on-road test, not proof of actual rest distance or felt jerk. Cursor advances only through
completed `00001eff`; live `00001f00` remains intentionally pending for the next cycle.

## 2026-07-18 — Cycle-10: bookmarked stop-and-go slam = re-anchor DEAD BAND; fixed + staged (b76e5739a2)
Live-route bookmark (00001f0c seg0, michael-rl): service-owned APPROACH_GLIDE slam -1.16..-1.27 (jerk 11.3,
pdec 1.30) at v 0.62, gap 4.5. Sol's trace (first cycle under the delegation rules) + my verification: ISD 0.3
anchored rest at 4.3; d_rem collapsed to 0.2; demand -0.91 + creep ff 0.24. Cycle-9's refined anti-blowup
re-anchors to the COMFORT landing but triggered only above A_REST_FEAS+hyst=1.35 -> the 0.65..1.35 band was
neither re-anchored nor comfortable. NOTE: the 6711daf0 trajectory split WORKED here (michael-rl composite
-0.68 bounded to a_plan -0.35) - the model did not write this slam; our glide law did. FIX (b76e5739a2, staged):
trigger = comfort+hyst (0.65) matching the target; scoped to the blow-up region (remaining <= 0.6 m) so normal
firm mid-glide demands don't erode rests (comfort-trigger alone dropped a nominal rest 4.0->3.49 in fixtures);
total relief budgeted 0.4 m/stop so sustained-push grades keep position (unbounded eroded crawl fixtures to 2.5).
Recorded-state probe: a_phase -1.20 -> -0.74, anchor 4.3 -> 4.12. 798 tests. Full f0a-f0c review + crank-gate
scoring pends the next complete cycle (route was live during this one).
