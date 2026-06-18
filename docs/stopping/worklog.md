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
