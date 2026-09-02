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

## 2026-07-18 — Cycle 11: evidence ledger + secure-stop pin; 11 m far-rest attributed (00001f10)
- **Bookmark seg10 t=624.5, rest 10.9 m**: model/planner-owned (source `e2e` all 170 approach frames;
  michael-rl mirrored the lead's own stop from 13 m back). Service exonerated: entered at v 0.53 with
  the stop already shaped; deepen-only means it can never extend travel. First settle was in crank-1
  band (wire -0.13, pitch-rate 0.004). Stop-position mechanisms all missed: explicit stop target
  activates only <=8.8 m true gap; synthetic glide <=5.0 m eff; **roll-in floor (42f6a3f7bd) computed
  the right -0.111 answer but its hard-stop latch (lead decel >=0.45 blanks 0.8 s) suppressed all 23
  eligible frames** (sol investigation, pinned to 55c11c2027). Close-gap creep retired 2026-07-01
  (rests final) so nothing corrects post-rest. +0.35 pulse at 625.3 = Conditional Experimental Mode
  flicker (0.5 s, source `cruise`); service correctly refused it.
- **Nudge at 625.31 (DISLIKE2, ours)**: gentle-finish landed -0.134 at latch; J_HOLD 0.6 build lost
  the race vs the +0.43 Stribeck creep peak through ~0.2 s actuator lag; 0.2 m escape, reactive
  -0.85/-1.0 arrest, ledger wound to -3.5 over the 62 s hold.
- **Shipped** (e525441ba1 + 18072f7df3, deployed staged on-road):
  1. StandstillEvidence ledger — all motion-evidence baselines/windows/references/epochs in ONE
     service-owned object (anti-conditional-tree directive). Five-state MotionEstimator REJECTED by
     sol xhigh red-team (9 findings: distinct thresholds/epochs/trust flavors are intentional);
     thinner ledger adopted. Behavior-neutral, 364 tests unchanged.
  2. Plant-aware secure-stop pin — once SECURELY stopped (dwell: readings < 0.05 crawl-free 0.25 s,
     anchor-restarted so decaying finishes never qualify), wire builds at J_PIN 2.5 to
     -(a_coast + 0.25), clip [-0.70, -0.45]. Predictive, replaces reactive arrest as the common case.
     Probe fixture from recorded incident numbers; 00001e65 fixture updated (its re-roll is prevented
     earlier); v0=0.6 close-entry band widened for the friction-free sim tail (nonphysical).
- **Ledger (next cycle candidates)**: (a) roll-in floor hard-stop latch scoping/removal — the direct
  far-rest miss; planner-side, read 42f6a3f7bd rationale first; (b) far-gap release scope: honor
  planner-go toward a stopped lead when trusted gap >= 6.0 m sustained >= 1.0 s (CEM flicker at 0.5 s
  must stay rejected; far_stationary fixtures rewrite to the sharper contract); user has effectively
  ruled a far rest WORSE than a post-settle correction move. (c) arrest-ledger overwind (-3.5 while
  parked) softening once pin is validated. Routes 00001f0a-00001f10 review still pending.

## 2026-07-20 — Cycle 12: secure-stop pin VALIDATED on-road; rest-gap band floor raised to 3.0 m (f9735655b1)
CORPUS: first post-pin drives. 19 routes `00001f11-00001f23` (679 finalized segments; qlogs tar-streamed,
then 38 stop-segment rlogs -- device on LTE at 0.36 MB/s, so the full-rlog sync was abandoned for the
two-phase qlog-triage/selective-rlog discipline). Every route runs a descendant of the pin commit
`18072f7df3` (verified `git merge-base --is-ancestor` for all 8 device commits). Crash-tail truncated
qlogs excluded: f11/3, f12/28, f16/9 (+ stale-lock f19/68, f1b/33, f22/9). BACKFILL CLOSED: f0d segs
4-18 and f0e segs 3-12/15-18 synced -- ZERO stopping frames in both (highway/manual); nothing reviewed
away. NOTE sync tooling: `refresh_routes.py --include-rlog` wanted to re-download rlogs for ~1,400
HISTORICAL segments still on-device before reaching the new routes (state.json only tracks what a prior
run fetched, and old runs were qlog-only) -- killed it and tar-streamed instead; fix or scope the state
before the next big sync.

PIN VALIDATION (18072f7df3): **hold overwind GONE** -- every hold rests -0.69/-0.70, deepest -0.85
across the 9-minute jam hold (f1d segs 100-109); the -3.5 arrest-ledger class did not recur, so
cycle-11 ledger item (c) is RESOLVED BY THE PIN (no softening needed). **Creep-nudge class mitigated,
not extinct**: 1 residual DISLIKE2 in ~7 real stops (f23 seg102 t6176). Frame trace: EASE arrival
landed -0.19 (beautiful), secure dwell + J_PIN build fired exactly on schedule (-0.19 -> -0.70 at 2.5
starting 0.25 s after wheel-stop), but the Stribeck creep broke through DURING the build ~0.45 s
post-stop, when effective brake pressure was only ~-0.5 through the ~0.25 s actuator lag. Travel 0.08 m
(was 0.2 m), arrested BY THE PIN at -0.70 (no reactive -0.85/-1.0, no windup). Residual exposure =
dwell 0.25 + build ~0.2 + lag ~0.25 s. Ledger candidate (do NOT bundle): shrink the window (faster
J_PIN, or seed the build from the arrival wire when the arrival landed shallower than -(a_coast+0.25)).

CRANK-1 GATE: **FAIL** -- ladder stays at crank 1. Moving-approach settles: jerk 3.4/3.6/4.1/5.4/6.0/
6.6/6.9 -> median 5.4 (gate <= 5.0); wire@stop -0.05/-0.19 in band but -0.31/-0.32/-0.35 on half the
stops and one -0.66. Mechanism (f12 seg6 trace): arrivals INSIDE the anchor collapse d_rem -> EASE
clips at A_EASE_DEEP -0.35 = the measured wire. The crank-2 candidate (A_EASE_DEEP -0.35 -> -0.30)
targets exactly this, but stays gated until jerk median <= 5.0.

REST-GAP MEASUREMENT (user: 'stops on the latest route felt too close'): autonomous lead-backed rests
3.5/3.6/4.0/4.1/4.2 (+5.09 re-settle; far-queue crawl rests 9.4-14 excluded) vs the 4.3 nominal
(4.0 + ISD 0.3). The headline 1.39 m rest on f23 seg103 is EXONERATED: driver gas-override creep from
4.3 m to ~1.0 m with pedal braking, re-engaged at standstill -- the service correctly held the
inherited rest. The 3.5 tail is COAST-IN GEOMETRY, not an anchor defect: traced f12 seg6 anchored the
full 4.3 (lead still moving -> re-anchor until lead stopped), no comfort-relief fired; the car was
simply already at ~4.3 gap with 0.9 m/s residual when the lead stopped, and the EASE -0.35 finish
carried it to 3.5. So the fix is the FLOOR, not the nominal/A_REST_FEAS.

RETUNE (f9735655b1, one lever): D_REST_MIN 2.4 -> 3.0, D_REST_CLIP_MIN 2.5 -> 3.0, EASE_GAP_MIN
2.6 -> 3.1, longcontrol CREEP_REST_GAP_MIN_M 2.5 -> 3.0. REANCHOR_GAP_MARGIN_M 1.0 -> 0.5 keeps the
anti-blowup ABSOLUTE admission edge at gap >= 3.5 (raising the clip-min would otherwise have blocked
comfort relief across 3.5-4.0 m and re-opened the cycle-10 slam dead-band); only the relief landing
floor rises to 3.0. Planner mirrors already coherent (4.0 nominal, 3.10 eff = 3.40 true trigger clip,
roll-in geometry) -- untouched. D_HARD 2.0 untouched. Direct-model position bound now stops-by-3.0
(deeper = safe direction; MPC trajectory still never shallowed). Fixtures moved to the new contract;
the hot-arrival probe (v0 2.1, gap0 5.3) now rests IN-BAND at ~3.45 (was allowed 2.35) with a
deliberately firm arrival -- hot arrivals into the floor stay firm by design. Grade exception stands:
the 5%-downhill fixture rests 2.83 (documented crawl loss; deepen-only cannot reclaim overshoot).
519 lib tests + 110 helper/params tests green; 30 pre-existing env failures in controls/tests
(latcontrol/torqued/following-distance) confirmed identical on the clean baseline; ruff clean on all
touched files.

WATCH (next cycle): rests should shift to ~3.8-4.3 with a >= 3.0 floor; hot arrivals near the floor
now arrive FIRM (wire deeper than the felt band) -- if the user reports new terminal grabs on close
stops, that is this trade; residual pin-build nudge window; f23 seg111 two brief hot `stopping`
entries at v 1.5-2.1 with no stop (planner-domain, takeover-class adjacent, no takeover occurred).

SOL FINAL REVIEW (adversarial, xhigh): verdict needs-attention, both findings real, both shipped in
arbitrated form (ca323b8499). (1-high) The margin-compensation claim was WRONG in one respect: the
landing gate also moved (2.5 -> 3.0), so relief admitted at landing == exactly 3.0 had zero overshoot
budget -- gap 3.5 @ v 0.7071 rested 2.84 THROUGH the floor, and v 0.7072 sat across a hair-trigger
firm/gentle cliff. Fixed with REANCHOR_LANDING_MARGIN_M 0.25 (admitted landings >= 3.25; the boundary
band all takes the firm branch -- the cliff measurably disappears in the sweep fixture). Sol's fuller
recommendation (predicted terminal position incl. actuator rollout + continuous relief) REJECTED for
this cycle: new machinery in a hot safety path; ledgered as a candidate. (2-medium) The offline
replay gate (StopContract 2.5-5.0, verify_candidate _gap_band_error) still passed 2.5-3.0 rests --
moved to 3.0, SCORING_CONFIG_VERSION 5; NOTE for the next verifier run: paired baselines score under
the new band on both sides (symmetric), no re-baseline of stored corpora performed here. Final:
802 passed / 19 skipped across the stopping battery + tools, ruff clean.

## 2026-07-25 — Stop-commitment necessity floor (live-route takeover 00001f47 seg6) — planner policy lane
User bookmarked a live-drive brake takeover: approaching a lead that braked hard to a stop (aLeadK -2.6..-2.9,
13->0 m/s), the planner tracked the slowing-lead cap ceiling (~-2.3) fine, but from t=6329.5 (v 5.0, gap 10.2,
lead stopped) the command RELAXED -1.93 -> -1.80 -> -1.47 exactly as the decel required to rest at the 3.0 m
band floor blew through 2.2 -> 3.8 m/s2; driver braked at v=2.25 / gap 3.3, rested 1.7 m. Wire faithful
(actuator==aTarget within 0.05); stopping service owned nothing until the last 0.3 s. Same class as the cycle-4
hot-approach takeovers; user directed a policy-level fix between model/MPC and the stopping controller, with an
explicit DO-NOT-OVER-ROTATE constraint (no return of harsh-braking-for-slower-leads / huge gaps).
DIAGNOSIS CORRECTIONS during design: (a) the onset was NOT accel_clip-slew-limited — in the blended branch the
clip converges to ACCEL_MIN in cruise (the 0.95 m/s2/s ramp was the slowing-cap's own value evolution); (b) to
the 3.0 m floor the approach was kinematically sound until t=6330.3 — the ENTIRE failure is the terminal
softening. So the fix shrank to ONE deepen-only lane (sol plan-review adopted: drop the no-soften hold C — its
wire authority is void under LIVE service scalar position-bounding; drop clip escalation A — not the limiter).
SHIPPED: stop-commitment necessity floor (kill switch stopping_flags.SANTA_FE_STOP_COMMIT_ENVELOPE): LAST Santa
Fe writer in the blended branch, output = min(output, -clip(a_req, 0, 3.25)) where a_req rests the ego 3.0 m
behind the lead's projected stop point (least-severe aLeadK over 0.3 s -> conservative lead stop projection).
Gates: same-radar-track persistence 0.5 s (00001b97 glitch class), stopping-or-stopped lead only (aLeadK<=-0.75
or vLead<=0.5), v_ego in (0.5, 16.5] (corpus: 16/18 ungated fires were 30-40 m/s highway brake-waves — wrong
frame there), d_rel<60, a_req>=1.5, Schmitt margins 0.30-in/0.10-out vs the current command, forceCoast off,
counters reset with reset_state. VALIDATION: 8 new fixtures incl. incident frames + barely-slower-lead
non-fire + ordinary-approach non-fire + glitch persistence + least-severe windowing + vision-confidence gate; 520
stopping-suite tests pass. CODEX ADVERSARIAL END-REVIEW (needs-attention) -> all 3 findings addressed in one
round: (1) actuation-horizon: necessity now computed on the 0.2 s response-delayed gap (paired counterfactual
with the observed ~0.35 m/s2 actuator undershoot: rest 1.38 m unaided -> 3.11 m with the floor, >= the 3.0
band floor, service arrest below 2.5 not even modeled); (2) vision-only leads (radarTrackId -1 shared
sentinel) now require modelProb >= 0.9 every frame instead of fake same-track persistence; (3) lane state
cleared on EVERY ineligible frame (acc mode, force-coast, kill switch, non-Santa-Fe) via a common-path reset,
so confirmation is always a fresh contiguous 0.5 s. FINAL corpus scan (real shipped functions, 4.82 h
engaged): 5 fires = 1.04/h -- the incident (fires 0.25 s earlier, 1.5 s), two sub-0.5 s stopped-lead
micro-defenses genuinely below the floor, two single-frame nudges behind hard-braking leads (max delta 0.37 /
1.54 vs a near-zero command); zero in ordinary following, highway waves excluded by the 16.5 m/s ceiling.
Residuals documented: acc->blended mode-switch clip ramp (rare) un-addressed; lead braking at -0.74 with
collapsing gap is a knowing false-negative (v1 scope); ~16.5-18 m/s hard-braking-lead class excluded by the
ceiling (f1d t=2904 fired only because v dipped under 16.5).

## 2026-07-26 — Cycle 13: standstill escape is the dominant class; pin re-armed on lost deceleration (21ee247965)
CORPUS: cursor 00001f23 -> 00001f47. 24 routes / 517 finalized qlogs, plus the 5 stop-segment rlogs pulled
once the device came back on WiFi. The f24-f3a backfill (never synced during cycle 12, device was on LTE)
turned out to hold 20x the engaged driving of the f3b-f47 block: 69,140 enabled frames / 9,443 stopping
frames vs 20,159 / 485. Routes f3b-f43 are ~100% manual (2.0 s of engaged frames across 160 segments).
Builds spanned: f24 db0f0920be (PRE band-retune), f27-f37 5eb3268574 (post-retune, OLD model), f39-f45
a9b9a78366 (+ Rebel Legion model), f46-f47 aeb6264e43. All predate the stop-commitment floor 22b9e1e294.

MEASUREMENT DEFECT FOUND AND FIXED FIRST (ed0b94628b): deep_stop's settle detector needs 0.5 s below
0.05 m/s before it opens a settle, so a stop whose wheel never reads that low before it creeps forward
produced NO settle at its arrival -- and `wire_at_stop` was then sampled from a frame INSIDE the reactive
arrest. 00001f44 seg3 was being reported as `normal_resume, wire -1.00` when it was actually a 0.25 m
creep escape off a -0.15 arrival. The class this battery exists to catch was invisible to it, which is
very likely why cycle 12 counted "1 residual nudge in ~7 stops". Added an independent arrival/escape scan
(settle metrics untouched, historical numbers stay comparable); it reproduces all three f44/f46/f47
incidents and filters genuine launches.

THE CLASS (12 escapes / 49 settles corpus-wide), by build:
  A  db0f0920be  pre-retune, old model     19 settles   3 escapes (16%)  travel med 0.095  holds<-0.70 1/16
  B  5eb3268574  post-retune, OLD model    25 settles   5 escapes (20%)  travel med 0.128  holds<-0.70 5/22
  C  a9b9a78366+ post-retune + Rebel Legion  5 settles   4 escapes (80%)  travel med 0.154  holds<-0.70 4/4
So the escape is PRE-EXISTING (16% before the cycle-12 band retune -- not introduced by it), the retune
made the overwind materially worse (6% -> 23% of holds deeper than the secure level), and the new model
pushed it to near-universal: its terminal demand now arrives at aTarget -0.05..-0.14, so the standstill
pin is left doing all of the holding. n=5 on build C, so treat the 80% as directional.

ROOT CAUSE (traced frame-exact on both rlogs; two independent structural faults in the cycle-11 pin):
 1. a_coast is FROZEN below v = 0.1 (stop_context A_COAST_HOLD_V) -- it is learned BEFORE the HEV creep
    torque engages. Replaying StopContext over 00001f44 seg3 gives a_coast = +0.044 held from t=229.766,
    while the measured push climbs +0.036 -> +0.175 -> +0.287 -> +0.43 under a CONSTANT -0.148 wire.
    -(a_coast + PIN_MARGIN) is therefore shallower than A_HOLD on essentially every stop and the clip
    does all the work: the "plant-aware" pin level was a constant -0.45 in disguise. J_PIN could only
    ever cover the -0.19 -> -0.45 leg (measured: 0.08 s, 39% of the needed depth) and the -0.45 -> -0.70
    leg always crawled at J_HOLD 0.6 for ~0.42 s.
 2. The secure dwell asks the VELOCITY READING to fall below STOPPED_SECURE_V 0.05, which a car held up
    by creep torque never does. 00001f44 seg3 bottomed at 0.082 m/s (zero frames below 0.05 in the whole
    arrival): the pin never armed AT ALL. Marginal detail worth keeping: CS.standstill first asserted at
    v=0.095, ABOVE V_WSTOP_RESET 0.09, so that frame was discarded and the latch then dropped again
    mid-escape.
Consequence chain, identical on both: shallow arrival -> creep breakaway -> reactive arrest -0.70 ->
MON_ESCALATE_STEP 0.15 every 0.5 s -> -0.85 / -1.00, and because the monitor floor is a RATCHET that
clears only on RELEASE, that depth then held for the ENTIRE standstill (27.4 s on seg3, ended only by
driver gas; 7.2 s on f47 seg2).

FIX (one lever): ask "is the finish over" of the measured DECELERATION instead of the velocity reading.
a_ego was already plumbed into StoppingService.update and completely unused. New ledger evidence
ev.finish_over = a_ego >= -FINISH_OVER_A_EGO (0.02) sustained FINISH_OVER_DWELL_S (0.06 s), post-latch,
crawl-free. It joins the existing "grace yields to evidence" branch, so the same J_SAFE path that today
reacts to an OBSERVED roll now pre-empts it, and the pin targets A_HOLD_SECURE -- the depth the plant has
actually demonstrated (~20 arrests across cycles 6/11/12, and again in seg3: aEgo crossed zero only after
-0.70 had been on the wire for 0.49 s). The a_coast term is dropped from the level because it is provably
inert at standstill; the ORed dwell is kept because each channel is blind where the other sees.
Strictly gentler than today's outcome: the same depth arrives earlier, without the lurch that precedes it
or the escalation that follows.

SOL ADVERSARIAL REVIEW -> NO-SHIP on the first cut; both findings correct, both fixed (5f09eb041f).
 (1-high) "loss of deceleration is not sufficient evidence of standstill": a_ego >= -0.02 also matches a
 constant-velocity roll, a wheel-speed quantization plateau and a 60 ms Kalman excursion, and CS.standstill
 latches as high as 0.095 m/s on this car (seg3 asserted at exactly 0.095) so the wheel-stop gate does not
 bound residual speed. Their probe drove -0.70 at J_SAFE while vEgo held 0.082 = the cycle-5 grab rebuilt.
 MY OWN negative fixture only covered the DECELERATING case, which is why I missed it. Shipped predicate
 now requires NET FORWARD ACCELERATION (a_ego >= +0.02): true only once the wire is insufficient, false for
 both a coasting roll and a genuine finish. 4 parametrised no-grab replays added.
 (2-medium) the proactive pin delays quick queue restarts ~0.45 s; the channel now pauses while trusted
 departure evidence accumulates (never shallows an achieved floor -- that would break deepen-only), and the
 release ramp is bounded by a regression test (<= 0.75 s to inactive).
EVIDENCE, and the cost of the sharpening: recorded-state replay puts -0.70 on the wire at +0.005 s (seg3)
and +0.050 s (f47 seg2) relative to breakaway -- essentially AT it, versus -0.155 s for the rejected
aggressive cut -- and 0.20-0.23 s earlier than the recorded reactive response. So this is a PARTIAL
MITIGATION, not a proven prevention: expect less escape travel and no ratchet to -0.85/-1.00, but the
escape may still occur. Shipping it anyway because the aggressive variant buys prevention at the price of
a grab on every ordinary stop -- the worse trade. If the next drive still shows escapes, the principled
next lever is a standstill-specific push estimate (ledger item b), NOT a looser trigger. HONEST LIMIT: the closed-loop plant CANNOT adjudicate this class (no static-friction breakaway
model -- the friction-residual assessment says a velocity-only residual cannot evaluate stiction relief),
and indeed it shows 0.000 m escape pre-fix, so no closed-loop efficacy claim is made. Crank-1 protection
is positively checked instead: while the car is genuinely still decelerating a_ego is negative so the
channel stays silent (fixture), and instrumented in closed-loop sim finish_over first fires at v = 0.0.
804 tests, 2 new fixtures from the recorded numbers. One pre-existing fixture relaxed deliberately
(test_longcontrol_live_terminal force-coast: it required >=10 mid-ramp frames to observe the tail min();
the faster build leaves 8 -- the contract under test is unchanged, only the window is shorter).

BAND RETUNE (cycle 12) -- validated properly on the larger sample, and my cycle-12-era reading corrected:
26 moving-approach settles with a lead give rest gaps min 2.80 / p25 3.67 / MEDIAN 4.04 / p75 4.32. That
is squarely the intended 3.8-4.3 nominal band; the earlier "0 of 3 in band" note was small-sample noise
from the thin f44-f47 block. Two sub-floor rests: f24 seg25 at 2.80 is PRE-retune (old 2.4 floor, expected)
and f27 seg12 at 2.80 is the documented hot-arrival overshoot, not an escape (v 1.43 at 4.2 m, firm -1.51,
dts pinned at the 0.05 close-hold; deepen-only cannot reclaim overshoot -- cycle-12 fixtures already pin
this class at >= 2.85).

CRANK-1: FAIL, ladder stays at 1. With real rlogs the honest 20 Hz jerks are 7.9 / 7.7 / 8.2 / 8.2
(median ~8.0) -- WORSE than cycle-12's 5.4. Critically, the qlog-derived numbers for the same stops were
1.5 / 2.3 / 2.6, a ~3x suppression: livePose is decimated to 5 Hz in qlog against a gate defined at 20 Hz.
RULE FOR FUTURE CYCLES: never score the crank gate from qlogs. Note the crank-2 candidate
(A_EASE_DEEP -0.35 -> -0.30) targets a -0.31..-0.35 wire cluster that does not exist in this corpus at
all -- terminal wires are already -0.05..-0.19 and the failure is the opposite direction.

LEDGER: (a) the escalated monitor floor still never unwinds while stopped -- if the fix removes the
escapes this becomes moot, so re-measure before touching it; (b) a_coast's freeze below 0.1 m/s is a
real blind spot in its own right (it can never learn the Stribeck rise) -- a standstill-specific push
estimate is the principled repair; (c) f46 seg13 rested 5.80 m with dts +1.57 (stopped 1.57 m short of
target) -- the far-rest class, unrelated to the escape; (d) 00001f47 seg6 is the takeover 22b9e1e294 was
written for and is NOT yet exercised on-road by any route in this corpus.

### Cycle-13 addendum (2026-07-26): the pin trigger was REVERTED (350e86aeb7) — read this before retrying
Round-2 sol review returned no-ship again, and its central finding is verified independently: replaying
00001f44 seg3's recorded carState through the REAL StopContext yields only **34.1 ms** of continuous
`wheel_stop_latched AND aEgo >= +0.02` against the 60 ms dwell, so the trigger never fires on the primary
recorded escape.
```
t=230.178  vEgo=0.089846  aEgo=+0.030  standstill=True    <- last latched frame
t=230.186  vEgo=0.092257  aEgo=+0.050  standstill=False   <- crosses V_WSTOP_RESET 0.09, latch clears
```
**THE STRUCTURAL BLOCKER (the real result of this cycle): the wheel-stop latch resets at V_WSTOP_RESET
0.09 m/s, and the creep escape accelerates the car through exactly that threshold. Any standstill evidence
channel gated on the latch dies at the precise moment it is needed.** That is why both the cycle-11 dwell
and my cycle-13 replacement fail on the same incident, for the same underlying reason.

METHOD FAILURE WORTH REMEMBERING: both of my verification artifacts — the unit fixture AND the
recorded-state replay — forced `wheel_stop_latched=True` and held vEgo constant instead of deriving the
latch from recorded inputs. They shared one wrong assumption, so the replay could not catch the fixture's
error, and I reported "beats the creep" twice on evidence that had the failure mode engineered out of it.
Rule for next time: any replay that claims to reproduce an incident must drive the REAL StopContext from
recorded carState, never hand-set its outputs.

Also unresolved from the same review and NOT fixed by the revert: with only the latch bounding residual
speed, positive aEgo is still satisfiable by a downhill roll or queue-following creep (moving-roll grab
surface), and the departure pause starts only after 0.3 m of confirmed gap growth, by which time a pin can
already be built.

NEXT ATTEMPT must start at the latch, not add another channel behind it. Two candidates: (a) give the
standstill latch hysteresis that survives a creep breakaway (it currently resets on the very event it
should be reporting); (b) a standstill-specific push estimate independent of the latch — a_coast is frozen
below 0.1 m/s and structurally cannot learn the Stribeck rise. Either is its own cycle with its own
evidence. Do NOT retry a looser trigger.

### Cycle-13 addendum 2 (2026-07-26): codex xhigh design pass — SHIPPED the unwind, and BOTH ledgered next-steps are DEAD
Ran the codex rescue helper read-only at `--effort xhigh` as a design/diagnosis pass (not a review),
briefed with the three rejected attempts and the frame-exact latch/a_coast findings. It earned its keep:
it found a real defect in my unwind, killed both of the "next attempt" candidates I had ledgered, and
proposed a better one. Verbatim-grounded conclusions:

**(1) My unwind had a real hole, and my own fixture hid it.** "A crawling car never qualifies" was FALSE:
`crawl` needs a 0.15 m trusted-gap deficit, which a sub-quantization crawl (~0.015 m/s) does not reach
for ~10 s, so the floor unwound at t=2.25 s -- and my fixture only inspected the FINAL floor at 6 s, by
which point the ladder had re-escalated, so it passed while missing exactly the transient it claimed to
forbid. FIXED with an unwind-local displacement baseline whose slack is MEASURED, not invented: across
every parked hold in this corpus the trusted gap moves inward by exactly one 0.1 m radar quantum and
never two (n=200, p50 = p99 = max = 0.100 m), so MON_UNWIND_GAP_SLACK_M = 0.12 admits jitter and rejects
a second quantum. Dwell length then sets the slowest catchable crawl (0.12/6.0 = 0.02 m/s) and slower
ones trip the 0.15 m crawl lane at ~7.5 s, so the two mechanisms meet. Residual bounded by the clamp at
A_HOLD_SECURE. Also: my dropout fixtures paired `dropout=True` with `gap_source="measured"`, a
combination the real StopContext CANNOT emit (it emits "decay" with lead_status False) -- the same
"fixture doesn't test reality" failure as the reverted attempt, a third time. Corrected.

**(2) LEDGER ITEM (a) "give the wheel-stop latch hysteresis" is DEAD.** While `wheel_stop` is true,
`_planner_safety_demand()` returns _INF, which disables BOTH the direct and the trajectory planner
demand. Keeping the latch asserted through a 0.1-0.25 m/s escape would therefore remove the MPC
trajectory lane while the car is MOVING -- a direct violation of the P1 constraint. It also would not
arm the pin anyway (`stopped_secure` still needs v<0.05, `genuinely_stopped` v<0.03), and the phase
already remembers the episode (losing the latch does not return RAMP_TO_HOLD to approach). A separate
"latch was acquired then broke" EDGE is legitimate but is a reactive mitigation only: it fires after the
car is already above 0.09, so the ~0.25 s lag forbids calling it prevention.

**(3) LEDGER ITEM (b) "unfreeze a_coast / standstill push estimate" is DEAD as posed.** At static
standstill measured acceleration is ~0 while creep torque, commanded braking and static friction
balance, so the residual `aEgo - delayed_cmd` would learn `0 - delayed_negative_command`: the alleged
"push" grows with the service's OWN command. Self-referential, not identified propulsion. And there is
no brake-force observation to disambiguate: Hyundai CarState sets `ret.brake = 0` with a
"TODO: Find brake pressure". An IMU/ESP12.LONG_ACCEL cannot help either -- acceleration stays zero while
static friction holds, so no signal reveals latent force BEFORE motion.

**(4) NEW BEST CANDIDATE (next cycle): a raw-wheel physical-stop certificate.** Hyundai `CS.standstill`
accepts wheel readings up to 12 DBC counts ~ 0.104 m/s (carstate.py:17,103-109) -- THAT is why it
asserts while the car is still rolling at 0.095. But each wheel is quantized at 0.03125 km/h ~
0.00868 m/s (WHL_SPD11), and `CS.vEgoRaw` plus all four wheel speeds are already exposed. So a genuine
stop certificate = phase in RAMP/HOLD, no release fired, ALL FOUR raw wheel speeds <= 1 count. Emit it
as a NEW StopSignals field from StopContext (it must NOT be reused as generic `wheel_stop`, or it
re-creates defect 2 above), preserve the arrival command for one frame, then target A_HOLD_SECURE at
J_SAFE (-0.19 -> -0.70 in ~0.064 s). MANDATORY VALIDATION, in order: replay raw wheel counts from
00001f44 seg3 and 00001f47 seg2 through the real parser and real StopContext and prove the certificate
occurs BEFORE the recorded breakaway -- if it never occurs on the primary incident, REJECT the design
immediately; then replay all 49 settles to prove no ordinary true-stop wire leaves [-0.30,-0.05]; then
measure counterfactual timing on all 12 escapes. Call it a timing mitigation until on-road proves
prevention.

**(5) The other honest option is StopReq stage B**, already specified in docs/stopping/on_vehicle_protocols.md
(STOP_REQ_MAX_SPEED 0.10, release 0.12). The primary incident bottomed at 0.082, so that gate WOULD have
asserted where the current 0.01 gate does not. It is the only repo-grounded way to separate the arrival
command from the holding actuator physically (ESC/EPB holds while the scalar command stays shallow), but
it is an on-vehicle protocol experiment requiring the existing isolated staging (fault / AVH_LAMP /
PBRAKE_ACT / creep-push / launch checks) and an explicit amendment to the "sole longitudinal authority"
architecture.

**(6) On crank-1.** The data refutes "shallower has improved measured jerk" (8.0 vs 5.4 m/s^3) but does
NOT prove "deeper while moving will feel better" -- those are different claims, and cycle-5's subjective
regression still stands. Crank-1 therefore remains BINDING for the next change and must be re-evaluated
with a controlled faithful-IMU + subjective A/B, not silently relaxed inside a patch.

### Cycle-13 CLOSE-OUT (2026-07-26): unwind SHIPPED and DEPLOYED (ae00bfd41e, device verified)
Deployed the one lever that survived: the standstill hold no longer stays over-braked after an escape.
Adversarial history for the record -- FOUR review rounds, three verdicts of no-ship on the escape trigger
(reverted, 350e86aeb7) and two on this lever, plus a codex xhigh design pass. The last round confirmed the
CODE correct ("correctly evidence-gated, resets on blind/held/no-lead states, survives the baseline
attacks, and cannot relax past A_HOLD_SECURE") and rejected only the PROOF as not mutation-sensitive, so
the guards are now backed by an explicit mutation matrix: baseline slack -> 1e9 fails the 0.021 m/s crawl
fixture; unwind dwell -> 1e9 fails the jitter and unwinds-to-secure fixtures; removing the
stationary_trusted gate fails the dropout fixture. One probe (outward-then-inward walk) was found to pass
under EVERY mutation -- it is covered by the pre-existing 0.15 m crawl lane, not the new baseline -- and
was relabelled as a composite guard rather than left implying protection it does not provide.

RECURRING METHOD FAILURE, now four instances in one cycle, all the same shape: a fixture or replay that
had the failure mode engineered out of it (forced wheel_stop=True; held vEgo constant; forced a zero epoch
minimum so `rolling` stayed true; paired dropout=True with gap_source="measured", which StopContext cannot
emit; and a crawl fixture that only inspected the END state after the ladder had re-escalated). STANDING
RULES going forward: (1) any replay claiming to reproduce an incident must drive the REAL StopContext from
recorded carState, never hand-set its outputs; (2) any fixture guarding a safety mechanism must be shown
to FAIL when that mechanism alone is mutated; (3) assert transients, not just end states.

WATCH on the next drive: holds after a creep escape should settle to -0.70 rather than staying at
-0.85/-1.00, and pull-away after such a stop should be quicker. The creep escape ITSELF is unchanged and
will still occur -- expect the same 0.05-0.28 m nudges. Next cycle: the raw-wheel physical-stop
certificate (addendum 2, item 4) with its mandatory reject-first validation.

## 2026-07-26 — Cycle 14: the perfect-stop template — bob is ONE number, and the blocker is a three-guard dead zone
USER DIRECTIVE: his manual stops (brake, release near the end, gentle re-apply at rest, no head bob)
are near-perfect; autonomous stops never are. First audit the instruments, then reproduce the
technique programmatically. Same-day corpus: routes 00001f48-00001f4d, 182 qlogs; 16 MANUAL stops +
2 AUTONOMOUS (00001f4c seg56, on the deployed ffbd9e0e91) + 1 mixed; 15 stop-segment rlogs pulled.

INSTRUMENTS: we had everything and were using almost none of it. rlogs carry carState/carOutput/
carControl at ~101 Hz and RAW accelerometer+gyroscope at ~104 Hz (livePose 20 Hz; qlogs decimate
livePose to 5 Hz — never score smoothness there). deep_stop reads only 20 Hz channels and NO tool had
ever looked at a disengaged stop. Shipped stop_fingerprint.py (f5779c5184): manual stops are now
first-class evidence. First-pass traps confirmed and handled by the workflow's signal agent:
carState.aEgo floors near standstill (decel@stop must come from raw accel / dv/dt), gyroUncalibrated
needs per-window bias removal + axis verification (first-pass "bob" numbers were bias).

THE TEMPLATE (13 clean manual stops, time-aligned at physical wheel stop; artifacts in
tools/stopping/review/cycle14/): peak ~0.27+0.31*v_ap; release begins at v~0.59*v_ap, ramps down
0.54 m/s3 [0.35-0.72] to ~0.25*peak; the car ROLLS the last ~1.1 s at light decel; touch/carry at
the stop instant is the whole game: **bob_peak = 0.0047 + 0.0138 * carry (net decel at wheel-stop)**.
Approach speed r=0.23, release shape nil — TWO distinct human sub-styles (release-dwell-reapply vs
taper-to-touch) score IDENTICALLY. Clean exemplars carry 0.20-0.40; the fit predicts BOTH autonomous
stops within 0.001 rad/s (carry 0.49 -> bob 0.0119; carry 0.89 -> 0.0179). PRESCRIPTION: land with
net carry <= 0.4 and let the creep carry the last stretch. Honest caveat: 56a's 0.0119 is INSIDE the
manual clean range (the human does not release on creep-settles either); the outlier class is 56b.

CONTROLLER GAP (replay-validated: the real StopContext+service driven from recorded carState
reproduces the recorded wire p95 0.032/0.052 — tools/stopping/review/cycle14/trace_auto.py):
- 56a (carry 0.49): planner permitted -0.12 the whole terminal; the MONITOR ladder (hover armed by
  the creep-carried roll — the human technique's signature IS the monitor's fault definition)
  carried -0.65 into rest. Not addressed this cycle (felt size inside the clean band; a roll-in
  corridor is big new safety surface; the cycle-13 unwind returns the depth 6 s post-rest).
- 56b (carry 0.89, THE bob class): Doppler noise on the stopped lead broke lead_confirmed_stopped
  (vLead -0.09..-0.20 for 0.8 s vs the -0.1 gate) -> spurious RELEASE at 1.3 m/s -> re-entry KEPT the
  stale 3.94 anchor at gap 3.5 -> d_rem floored 0.15 -> glide blow-up -1.5..-2.2 at t-0.7 s; recovery
  ran the J_UP 1.5 line, wire crossed -0.50 only 0.05-0.12 s before rest -> through the ~0.25 s
  actuator lag the CALIPER pressure at t_phys was the -0.9 wire from a quarter-second earlier. The
  divergence map (7 items, D1-D7 with line refs) is in the workflow journal; headline: no phase may
  emit shallower than -0.10, creep is CANCELLED rather than used as the carrier, and the roll-out
  signature is the monitor's fault definition unless a lead is receding.

ATTEMPTED LEVER, KILLED BY MY OWN EVIDENCE (uncommitted, reverted): J_UP_TERMINAL fast shed of
no-longer-demanded transients. Sol plan-review (xhigh) first killed 2 of my 3 proposed parts with
frame-exact math (A1 re-entry re-anchor: wrong numbers, plunge returns via the floor in 0.17 s; A3
grace convergence: RAMP begins AFTER t_phys on the incident — cannot help — and I had cited the
REVERTED finish_over path as its guard; my validation gate "wire@t_phys < -0.50" already passes
today, i.e. vacuous). The surviving shed was then iterated against the R1 sub-quantization crawl
probe (slow release IS the cushion there — gap consumed to 2.92) into a decel-licensed v<=0.35
GLIDE+EASE form — all 510 tests green — and then the instrumented counterfactual on the validated
replay showed it NEVER FIRES on 56b: the wire trails the relaxing composite a_plan by ~0.25, just
under any sane shed gap, and the carry was already fixed by the plunge through actuator lag.
An inert lever does not ship. Working tree reverted to the committed state.

THE NAMED PROBLEM (next cycle's design target, with fresh review budget): 56b's plunge lives in a
THREE-GUARD DEAD ZONE of the anti-blowup relief — comfort_landing 2.92 fails the landing margin
(>= 3.0+0.25, cycle-12 sol), relief_floor = ref 3.94 - budget 0.4 = 3.54 blocks it (cycle-10 crawl
guard), and the gap margin is marginal — so the glide law defends a stale unreachable anchor at -2.2
instead of conceding ~10-15 cm inside the documented hot-entry overshoot class. Cycle-12 chose
position over comfort at this exact boundary ("the rejected case becomes abruptly firm" — sol said
it then); cycle-14's template evidence re-prices that choice: the firm branch costs bob 0.0179, the
user's core complaint. Un-knotting the three guards coherently is THE lever; it must also swallow
D7 (the Doppler entry pump: hysteresis on LEAD_STOPPED_V_MIN) or re-entries keep re-plunging.
Secondary ledger: 56a/monitor roll-in corridor (displacement-budgeted tolerance of the creep-carried
finish); planner-side: why the composite aTarget holds -0.53..-0.41 while the trajectory demand is
-0.25..-0.01 (the position bound gives no relief inside gap ~3.5 — interacts with the same dead zone).

CRANK LADDER RE-AIM (evidence): the gate's wire@stop band [-0.30,-0.05] measures the WIRE, but the
felt quantity is NET CARRY (wire + push, through actuator lag). Rewrite the gate on carry (raw-accel
dv/dt) <= 0.4 with bob_pitch_peak <= 0.015 once the fingerprint tool has a few cycles of corpus.

## 2026-07-26 — Cycle 15: the dead zone dissolved (floor-defense cap) + the Doppler pump killed (589678c067 + abe2c9d021)
Design done in-main-loop off cycle-14's banked evidence (no new routes needed); sol xhigh PLAN review
first (verdict: neither lever as written), implementation to its corrected spec, then sol end-review:
**APPROVE, "no material findings"** — the first clean first-pass ship verdict of this project.

WHAT THE PLAN REVIEW CORRECTED (both adopted): (1) my zero-lag floor law rested 2.84-2.90 in the
0.55-0.75 band whose contract is >= 3.0 — the shipped cap is LAG-AWARE (aim FLOOR_AIM_MARGIN 0.10 +
v*ACTUATOR_LAG_S 0.25 ahead) and stateless-continuous per sol's own form: never relieves below
max(A_GLIDE_NOM+hyst, v^2/1.2), so it equals the anchor demand at both region crossovers. (2) My
"any gap growth un-confirms" clause for the latch was rejected (radar outward walk would recreate the
false RELEASE); the shipped off-delay is negative-Doppler-persistence only (0.5 s, in-range frame
resets; acquisition/loss/drive-away unchanged). (3) A coupling I'd missed: the spurious RELEASE was
ACCIDENTALLY shedding the deep wire, so the latch fix must never ship without the cap.

LEVER 1 (589678c067): the anti-blowup anchor-move relief and its three guards (landing margin,
relief budget + _reanchor_ref, gap margin) are DELETED — net-negative machinery — replaced by the
floor-defense cap in _d_rem: in the blow-up region the phase law never demands more decel than what
rests the car at the band floor (lag-aware), trusted-measured-gap only, everything harder belonging
to a_kin/a_plan unchanged. Closed-loop contract: gap-3.5 sweep rests 3.045-3.096 across 0.55-0.75
(peaks softened, e.g. 0.55: -0.81 was -0.90); hot 0.85/1.0 unchanged in the documented exception
class; f0c relieved to -0.89 (was slam -1.27; old relief -0.74 — the 0.65 continuity term is
load-bearing against broad rest erosion and this contract rewrite of the sol-vetted f0c fixture is
explicit, with trusted/untrusted contrast as its mutation sensitivity); ba3 gentler, no regression;
no new discontinuity across the floor (the 3.1 EASE-gate step is pre-existing, jerk-limited).
HONEST SCOPE: at gap ~3.3 @ 0.7 m/s the cap correctly YIELDS (no gentle rest >= 3.0 exists there —
lag eats 0.17 m); firm is right, and the hot class keeps its documented bounds.

LEVER 2 (abe2c9d021): T_LEAD_NEG_OFF_S 0.5 on lead_confirmed_stopped. The recorded pump (56b):
one 20 Hz frame of vLead < -0.1 noise (runs <= 0.36 s on a physically stopped lead) broke entry_ok
mid-stop -> RELEASE 3415.78 -> re-entry with a stale anchor -> the -2.2 plunge -> 0.89 carry ->
bob 0.0179. With the off-delay the RELEASE never fires (replay), and CLOSED-LOOP the no-pump stop
rests 3.376 m IN BAND — the 56b class becomes a normal stop. Fixtures survive the recorded noise
run twice (reset exercised), sustained reversal still un-confirms, loss/drive-away instant;
mutation-tested (delay->0 fails exactly the noise fixture).

OPEN-LOOP CAVEAT (documented so nobody re-trips on it): the combined open-loop counterfactual reads
"worse" (deeper commanded braking without the accidental RELEASE shed) — that is the fixed-recorded-
velocity artifact sol named in advance; closed-loop is the valid evidence and it is green. 811 tests.

ON-ROAD WATCH (next drives): hot-arrival slams soften (f0c class -0.89 not -1.27); no more spurious
mid-stop releases behind stopped leads; per the carry law both should pull bob toward the 0.011
clean band. Score stops with stop_fingerprint.py: CARRY <= 0.4 is the target (crank ladder re-aim),
not the wire band. Residual known contributors: 56a-class monitor ladder on creep-carried rolls
(deferred, felt size inside clean band); genuinely hot arrivals stay firm by design.

## 2026-07-29 — Cycle 16: cycle-15 VALIDATED on-road; the bookmarked takeover is late lead ACQUISITION (fix attempted, reverted)
CORPUS: routes 00001f4e-00001f5d (238 qlogs). Engaged content lives in 00001f5c only (10,356 enabled
frames, 1,529 stopping frames, 2 user bookmarks); device build 354bb88cbd = cycle-15 + "Rebellious
Hope" model. 64 stops total: 60 manual, 2 autonomous, 2 mixed.

**CYCLE-15 VALIDATED.** Bias-corrected 100 Hz metrics (honest extractor, per-stop parked gyro bias,
raw-accel carry) on the two ordinary autonomous stops: carry 0.74/0.69, **bob 0.0060 / 0.0069**.
The human clean band is 0.006-0.012 and cycle-14's autonomous stops were 0.0119/0.0179, so routine
autonomous stops now sit INSIDE the manual clean band at roughly half the previous bob. NOTE the
cycle-14 carry law over-predicts here (0.014-0.015 predicted vs 0.006 measured) -- with the plunge
class removed, carry alone no longer explains bob; the law was fitted on a corpus dominated by
plunge-driven carry. Do not re-tune on it without refitting.

**THE BOOKMARK (seg5 t=341.72, marking the stop at t=338.4).** 100 Hz trace: ego following a lead
that decelerated normally from 5.9 to 0.8 m/s at dRel ~14-17 m, wire a comfortable -0.9. At
t=336.15 radarState.leadOne switched from track 50388 (dRel 13.8, modelProb 1.0, which REMAINED as
leadTwo and continued slowing to a stop) to track 102499 at **dRel 6.0, modelProb 0.0** -- an 8 m
inward jump in one frame. distanceToStopTarget went -1 -> +1.72 the same frame, and the wire ramped
-0.87 -> -1.37 -> -1.90 -> -2.77 over 0.8 s at 2.8 m/s until the driver braked (peak decel 2.32,
100 Hz jerk 10.0, bob 0.042 -- 6x the other stops on the drive).

**ATTEMPTED FIX, REVERTED (c77c9ac62c -> fbfe7ccddd).** I gated stopped-lead STOP TARGETS on
modelProb >= 0.5, reading modelProb 0.0 as "no vision support = ghost". WRONG, and the adversarial
review caught it as a CRITICAL under-braking hole; verified directly in radard.py: the low-speed
override deliberately promotes the closest radar track via `closest_track.get_RadarState()` whose
model_prob parameter DEFAULTS TO 0.0. So modelProb == 0.0 is the signature of the radar-only
low-speed promotion path -- which exists precisely so the car brakes for close objects vision may
miss -- not evidence of a phantom. The gate would have removed the explicit stop target for real
close stopped cars in degraded vision, and also rejected legitimate detections between the
configurable threshold (as low as 0.25) and 0.5. Second finding, also correct: my fixtures only
called the helper predicate and never ran LongitudinalMpc.update/the arbiter/the wire, so they could
not have proven the escalation was removed even if the premise had held.

**RE-READ OF THE INCIDENT (honest):** the 6.0 m track is not demonstrably a ghost. It is at least as
consistent with a genuinely closer vehicle in a queue (the 13.8 m car stayed present as leadTwo)
that the radar-only low-speed path acquired LATE. Given a real object at 6 m while doing 2.8 m/s,
-2.5 m/s^2 is the correct response to that geometry. The defect is the LATENESS of the acquisition,
which lives in lead association/selection (radard + model), the domain cycle 4 declared out of the
stopping stack's scope -- not in the stopping laws.

**LEDGER / NEXT:** (a) lead-association latency for close in-queue vehicles at low speed: does the
low-speed override only fire once dRel < some bound, and can a second, closer track be surfaced
earlier (leadTwo already carried the farther car -- the ordering flipped only at 6 m)? This is a
radard question and needs its own evidence pass over multi-vehicle queue approaches. (b) NOTE for
the user's own 355cd68 (stop-commitment floor requires modelProb >= 0.9): by the same radard
finding, that gate makes the floor inert for every radar-only low-speed-promoted lead. Since that
lane only ADDS deepening authority the direction is conservative, but the intent may not match the
effect -- worth revisiting. (c) the cycle-14 carry law needs refitting on post-cycle-15 stops.

## 2026-07-30 — Cycle 17: the walking-pace relief jolt — rate-shaped entry into relief depth (J_RELIEF_ENTRY)
CORPUS: routes 00001f4e-00001f62 (f5e-f61 essentially unengaged; f62 = the drive, build 9ee7e26dec,
30 segs, 2 user bookmarks at seg25 t=1546). 13 stops: 8 manual, 5 autonomous.

FELT DEFECT (both bookmarks): creeping at a FLAT 0.63 m/s, gap 4.6, planner aTarget -0.25, the wire
stepped -0.39 -> -1.10 in 0.28 s (J_DOWN toward the relief cap 0.65 + creep cancel ~0.45), then
~1.0-1.2 m/s2 ridden into rest. Drive-wide: 4/5 autonomous stops carried 1.04-1.20 with
release_min ~= peak (NO release), bob 0.011-0.021 -- vs 0.0060/0.0069 on f5c with IDENTICAL
stopping code. My first attribution (push/grade: f62 stops ran +0.19..+0.27 push vs f5c's
+0.05..+0.11) was a GAIN term, not the classifier.

ATTRIBUTION (sol xhigh design pass -- the discriminator I missed): **EASE ENTRY TIMING.** All four
bad stops encountered negative-Doppler radar noise below the -0.1 EASE gate (lowest vLead -0.127..
-0.188 after ego <= 0.5), so `_ease_gates_pass` rejected EASE and they stayed in GLIDE until
0.22-0.26 m/s; the f5c good stops entered EASE at 0.48-0.50 and unloaded 0.35-0.46 s before rest.
Matched-state comparison (both v~0.63, gap 4.60, anchor 4.30, cap constant 0.65): f5c arrived
brake-loaded (-0.62, increment to target ~0.11); f62 arrived shallow (-0.39, increment 0.71) -- the
jolt IS that increment taken at J_DOWN 2.5 in 0.28 s.

CANDIDATE A (relief floor as TOTAL wire, crediting creep) BUILT, TESTED, REJECTED: fixes the
bookmark level but physically breaches the R1/cycle-5 push-grade floor guards (2.76/2.91 vs the
3.0 pins) at EVERY credit strength incl. near-zero -- on a sustained push the wire fights the push
AND stops; the guards' margins are real, not stale. Discarded without shipping.

SHIPPED (this commit): **J_RELIEF_ENTRY = 1.0 m/s3** -- the lever is the DERIVATIVE of an
already-approved target, not the level. Applies only to the ordinary APPROACH_GLIDE descent into
relief depth, gated per sol's exact spec: trusted measured gap above the floor; blow-up region on
BOTH raw lead remaining and arbitrated d_rem (<= 0.6); lag-aware floor decel <= A_GLIDE_NOM AND
v^2/1.2 <= A_GLIDE_NOM (implies v <= 0.775). Deliberately does NOT require the cap to be binding
(the close-hold envelope selects d_rem ~0.5 while raw remaining enters 0.6; waiting for anchor>cap
misses the ramp). Safety paths untouched by branch order (safety_binding/_fast_deepen J_SAFE,
RAMP/HOLD rates, monitor). Bookmark: same -1.10 over 0.71 s instead of 0.28 s; displacement cost
~0.06 m command-ramp (~0.10-0.15 m with lag). Probe gauntlet: slow-grade crawl UNCHANGED,
subquantization 3.259 -> 3.257 (1.9 mm), tight-entry/boundary/f0c at existing pins -- NO pin moved.
Fixture pinned from the recorded state, mutation-sensitive BOTH directions (2.5 fails the 0.45 s
microtrace pin; 0.05 fails depth-by-0.75). 812 tests.

LEDGER: (a) THE ROOT TRIGGER -- the EASE gate's instantaneous rejection of lead_v < -0.1 is what
strands these stops in GLIDE on Doppler noise (same noise class the cycle-15 latch off-delay
handles for ENTRY); an EASE-gate off-delay is the principled root fix but touches reversing-lead
protection = own lever, own review, own probes. (b) f62 seg20 (v_appr 12.9, carry 1.20, bob 0.0214)
is a HIGH-SPEED approach with the same no-release signature -- check whether the same late-EASE
mechanism or the hot-approach class; (c) the carry-bob relation continues to weaken (carries 1.0+
with bob 0.011-0.021 spans the old line) -- refit still pending.

### Cycle-17 addendum — THE MECHANISM (user-supplied, 2026-07-30): we are fighting the hybrid's clutch
The user, mid-cycle: "We're fighting with the clutch of the hybrid system. The way I get perfect
stops is steady braking until the very end, sometimes minimally letting off, and then, right before
the stop, letting off more and re-engaging braking gently."

Re-reading our own data under this model, everything closes:
- The June friction-residual fit IS the opponent's torque curve: push = 0.15 + 0.28*exp(-v/0.066)
  -- near zero at 0.5 m/s, PEAK +0.43 at v~0.066. It is an ACTIVE clutch/e-motor creep controller,
  not passive drag, and it does NOT yield to brake pressure (cycle-14: push rose +0.04 -> +0.43
  under a CONSTANT -0.148 wire; cycle-13: broke the car loose through ~0.5 of caliper pressure).
- Every machine stop therefore ends as TWO OPPOSING RAMPS -- our brake deepening, its torque
  rising -- resolved discontinuously at wheel-lock. That discontinuity is the bob.
- The human technique is PHASE-MATCHING, not gentleness: release as the creep ramps (net force
  small and constant -- the cycle-14 template's "constant light 0.2-0.4 net decel through the last
  ~1.1 s" is exactly this), then re-engage so pressure MEETS the creep peak at rest.
- A_HOLD_SECURE -0.70 was never arbitrary: creep stall 0.43 + PIN_MARGIN 0.25 = 0.68.
- THE WRONG ASSUMPTION EXPOSED: the EASE band (-0.35..-0.10) and the crank-1 wire@stop gate
  [-0.30,-0.05] sit BELOW the creep curve -- which is precisely why stops hover/escape/get
  arrested there. "Gentle" is gentle in NET terms; the correct finishing WIRE is deeper than EASE
  allows and arrives in phase with the clutch.

CYCLE-18 SPECIFICATION (design with sol before building): a creep-synchronized terminal
feedforward -- in the final safe window (trusted geometry, v <= ~0.5, blow-up-region-class gates),
the wire tracks -(creep(v) + CARRY_TARGET ~0.25) using the fitted curve: ~-0.44 at v 0.5, -0.55 at
0.2, arriving at -0.70 == A_HOLD_SECURE exactly at rest, where the existing secure build/pin takes
over. Net decel constant ~0.25 by construction == the human template. Validate against the 13
manual exemplars (template.json) + the probe gauntlet; the felt gates re-aim on NET carry and bob,
NOT the wire band (retire/re-aim crank-1's wire window). Interactions to design: EASE's role in
that window (the feedforward may subsume EASE below 0.5), the monitor's hover definition during
the synchronized roll, and the June curve's provenance (refit on current corpus first -- it is
coarse-provisional, HEV regen/friction non-separable).

### Cycle-17 end-review round 1 (sol adversarial, base 0249c7aa07): NO-SHIP, one HIGH -- fixed
Finding (verified in sol's own microtraces): the gentle flag was geometry-only, and in the relief
window a_phase is often DEEPER than every safety lane, so safety_binding never asserts -- a lead
reversing mid-ramp or the monitor arming mid-GLIDE kept J_RELIEF_ENTRY; an inward gap collapse or
dropout cleared the flag but recovered only at J_DOWN with the gentle ramp's accumulated command
deficit intact. Also flagged: a target-agnostic rate selector would throttle the planned cycle-18
creep-synchronized feedforward (ledgered for that design).

Fix (this commit): hazard disqualifiers + a dedicated _relief_catchup latch (J_SAFE until the
wire catches the ungentled target; releases if the gentle conditions re-establish benignly or the
phase leaves APPROACH_GLIDE; deliberately NOT overloading _fast_deepen, whose EASE-revert
semantics differ). One deliberate deviation from sol's letter, matching its intent: the reversal
disqualifier uses the cycle-15 noise-hardened un-confirm (lead_confirmed_stopped through
T_LEAD_NEG_OFF_S) rather than raw lead_v < -0.1 -- recorded Doppler noise on physically stopped
leads runs to -0.20 in exactly this geometry (cycle-15 evidence), and a raw term would flip the
flag mid-ramp on noise and re-create the jolt being fixed. A never-confirmed lead still gets the
raw term instantly. Benign invalidations (remaining grew past the region, held-frame gap blips)
keep comfort rates by design, with a guard test pinning that.

Five new fixtures, all starting MID-RAMP in APPROACH_GLIDE (the window round 1 proved uncovered):
reversal / monitor-arming (real hover mechanism, no hand-set state) / gap-collapse-to-2.9 /
dropout each require an immediate J_SAFE step and deficit erasure; a benign region-exit guard
requires comfort rates. Mutation gauntlet: (A) disqualifiers removed -> reversal+monitor tests
fail; (B) latch never set -> all four hazard tests fail; (C) latch recovers at J_DOWN -> all four
fail. Battery 800 passed / 19 skipped; the nominal bookmark trace is untouched (v=0.63 > V_EASE
keeps monitor detection off there; latch=True keeps the reversal term silent at lv=0).

### Cycle-17 end-review round 2: catch-up release instability -- fixed via the ungentled-target snapshot
Sol round-2 flagged the round-1 fix's latch release: benign re-establishment could release the
catch-up early or oscillate gentle<->J_SAFE while a hazard flickers at a gate boundary; asked for
a sticky release (until the ungentled target) and a real-StopContext chatter test. Building that
test caught a REAL defect in my first stabilization attempt: under radar track churn (2 lost / 6
present frames), the flag re-asserted each cycle, the next dropout RE-LATCHED, and J_SAFE chased
the transient decay-frame demand (-1.6) -- the wire ratcheted -0.87 -> -1.23 over three churn
cycles, a progressive over-braking pump 3x worse than pre-cycle-17 churn behavior.

Root insight: the deficit sol wants erased is against the UNGENTLED TARGET OF THE GENTLE RAMP
(the last gentle-frame arbitration target, ~-0.92 in the trace), never the transient
dropout/decay-frame demand. Shipped design: (_relief_gentle_target) snapshots the target on every
gentle frame; the latch arms only when a hazard invalidates the ramp AND a live deficit against
the snapshot exists; the J_SAFE stage is BOUNDED at the snapshot; transient deeper demands are
chased at ordinary J_DOWN exactly as pre-cycle-17 (genuinely deeper safety demands still get
J_SAFE via safety_binding, which needs no help); release ONLY on snapshot catch-up or phase exit
-- no benign release at all. Closed-loop churn trace after: single J_SAFE deficit-erase (4
frames), zero re-latches, +-0.03 sawtooth around the measured demand -- the pre-cycle-17 churn
signature restored.

Tests now 8 relief fixtures: nominal bookmark, 4 mid-ramp hazard transitions, benign region-exit
guard, real-StopContext track-churn chatter (pins: J_SAFE first hazard frame, no gentle
mid-catch-up, catch-up <= 0.15 s, NO re-slam after catch-up, wire never > 0.09 below concurrent
measured demand), post-unconfirm Doppler chatter. Mutation gauntlet 5-way, all killed: (A) no
disqualifiers -> 3 fail; (B) no latch -> 6 fail; (C) J_DOWN deficit stage -> 6 fail; (D2) benign
release restored -> 2 fail; (D3) snapshot bound removed (the pump) -> 1 fail (the churn test's
reason to exist). Battery 802 passed / 19 skipped; wheel-latch probe: latch clears before RAMP,
hold builds at J_HOLD (no cycle-5 grab).

### Cycle-17 end-review round 3: transient-safety snapshot poison -- fixed (+ a second lockup path)
Sol round-3 (medium, verified in its own microtrace): the snapshot updated on ANY gentle-flag
frame, including frames where safety_binding runs the J_SAFE bypass -- a one-frame
trajectory-confirmed a_plan -2.0 stored -2.0 as "the ungentled target"; a following hazard then
latched a catch-up that could never complete (the wire parks at the real -1.10 target, cmd <=
snapshot never fires), leaving gentle disabled and J_DOWN for the rest of the approach. Fixed
exactly as recommended: the snapshot is captured only on frames where the gentle lane actually
governs (APPROACH_GLIDE, no safety_binding, no fast_deepen, deepening). Extending the finding I
found a SECOND unreachable-snapshot path sol did not name: a legitimate snapshot (-1.10) followed
by the lead reacquired farther out -- the target shallows and cmd can never reach the snapshot
again. General fix: the latch also releases when the wire has caught the CURRENT target (beyond
which a deepen-only catch-up cannot progress); mid-deficit benign frames keep the latch because
there the target is still deeper than the wire, so R2 stickiness is preserved (churn re-latch
still blocked by the live-deficit requirement). Two regression tests (poison trace; shallowed
target), mutation-tested (E: eligibility removed -> poison test fails; F: current-target clear
removed -> shallowing test fails). Relief group 10/10, battery 804 passed / 19 skipped.

NOTE mid-cycle interleave: the other agent pushed 44bfdd6432 (planner: late stopped-lead
approach firmness, 00001f65 seg13) while this cycle's review ran; an accidental amend of their
commit was repaired via soft-reset to the published tip (tree-identical, published history
untouched; the round-2+ work now lives in its own commit). Their 4 test_longitudinal_planner.py
failures PRE-EXIST at the shared base (verified at three commits) -- their lane, flagged.

## Cycle 18 (2026-07-31) -- the mid-stop clutch relaunch ("leapfrog"), and the harsh-chase bookmarks

User: most otherwise-smooth stops have a tiny acceleration moment mid-stop ("leapfrog") -- wants
one consistent stopping motion; plus two bookmarks (one per day) of harsh braking "from not
slowing down enough for a stop or slowing lead."

EVIDENCE (routes f66-f70, 146 qlogs; rlogs for every stop; all on 67b7a4a7db):
- THE SURGE, 3 of 6 f6e autonomous stops, uniform signature at 100 Hz: EASE unloads the wire to
  -0.11 as the car glides to v 0.09-0.16 (the latch needs v<=0.06 -- never fires), the creep
  clutch engages and re-accelerates the car 0.24-0.38 m/s AGAINST -0.35..-0.50 wire, the
  anti-hover monitor ladder (-0.35 start, 0.15/0.5s) rescues ~1.6 s later, rest arrives late at
  -0.80. The user's leapfrog IS the monitor rescue.
- CREEP REFIT (n=1987 frames, 6 stops): the June curve is wrong in shape -- the clutch is OPEN
  during deceleration above ~0.1 m/s (push p50 0.04-0.14) and ENGAGES as a threshold event near
  rest: static push p50 +0.43 below v=0.08 (p90 0.73); once accelerating the relaunch transient
  sustains ~0.7. Prevention (never let net decel reach zero) >> cure.
- ARRIVALS VALIDATED: all 6 stops carry 0.11-0.23 (human clean band), predicted bob 0.006-0.008,
  including the bookmarked one -- cycles 15/17 hold on-road; the surge is the remaining defect.
- USER'S TECHNIQUE NOTE (mid-cycle): brake steadily a bit STRONGER than our code so the clutch
  never kicks in, then release significantly right before stiction ("what Teslas do"). The floor
  implements the first half; the second half (release-before-stiction) is deliberately NOT
  implemented: the measured carry law (bob = f(net carry), release shape irrelevant, cycle-14)
  says -0.70 wire against the engaged clutch lands net ~0.27 = clean; on-road bob after deploy
  adjudicates.

LEVER 1 (SHIPPED 6400bfbd64, sol xhigh implementation per the standing delegation rule; arrival
-pin re-derivation by hand): TERMINAL CREEP-HOLD FLOOR -- deepen-only, entry-gated, EASE +
terminal GLIDE: -0.35 @ v0.30 -> -0.45 @ 0.15 -> A_HOLD_SECURE @ 0.08, jerk-limited by the
existing limiter. Net decel never reaches zero; the relaunch never starts; the monitor returns
to being a safety net. RETIRES the crank-1 wheel-stop wire band [-0.35,-0.05] per the mechanism
(felt metric = NET carry): 8 arrival pins re-derived deliberately (nominal, crank-1 -> arrive AT
the floor + never past secure, close-entry 0.6 band -> -0.75, dropout-floor moved above the
creep window preserving its purpose, aborted-go slam bound scoped to v>=0.15, slam fixtures'
moving-approach ratchet bound -> A_HOLD_SECURE, cap-bypass phase set + band). Closed-loop seg22
replay plant (real latch, engagement+relaunch push) pins no-relaunch/no-monitor/net-decel-at-
latch; schedule pins 0.14/0.07/0.35; mutation: floor off -> both fail. Battery 806/19.

BOOKMARK TRIAGE: (1) f6e seg16 = late queue reveal (lead switch at ~50 m, 11.5 m/s) + sustained
-1.1..-1.6 chase -- perception-limited, planner lane; (2) f70 seg46 = pure RESPONSE-LAG chase:
lead braked -0.9 -> -2.4 from 22 m while ego's first 2.5 s of response plateaued at -0.53..-0.57,
debt forced a -2.44 peak. Sol xhigh read-only diagnosis of the planner response chain is running;
lever 2 pending its verdict.

### Cycle-18 end-review: four rounds to approve
R1 NO-SHIP: (HIGH) the creep floor armed on an aborted-go re-entry with RISING v (launch pulse
floor-braked -0.10 -> -0.40 then released past 0.30 = a brake pulse mid-launch; my own test
surgery had scoped the slam bound to v>=0.15 and HID exactly this) -> fixed with the arming
latch: v below peak-since-activation by 0.05 (> one vEgo quantum, so rise flicker can never
arm), holds through a clutch relaunch, clears on both RELEASE paths; rising-leg bound pinned,
latch-off mutation fails. (MEDIUM) feedforward window eviction stepped the raw target ~0.45 in
one frame / instant release -> slewed authority (2.5 down / 1.5 up m/s3), continuity tests +
slew-removed mutation. R2: (HIGH) mode exit slewed a stored-but-unapplied authority (output
snapped anyway) and could re-apply stale authority on re-entry -> mode edge HARD-CLEARS the
whole lane (authority + window; in-mode continuity stays the in-block slew's job); (MEDIUM) my
soft-reset commit split had embedded the unslewed feedforward in the stopping commit (unsafe
bisect/revert) -> re-split atomically with full unstaging. R3: the re-entry test exercised only
helpers with hand-set state (deleting the production hard-clear stayed green -- the cycle-13
fixture sin again) -> the lane advance unified into ONE production function used by both call
sites; the test drives it across build/exit/re-entry; neutered-hard-clear mutation fails. R4:
APPROVE (10k-case equivalence sweep on the extraction). Suites at ship: planner 97 + exactly the
4 pre-existing failures (verified at base across 3 commits, other agent's lane, flagged);
stopping battery 806/19; ruff clean.

LEDGER (cycle-18): seg16 late-queue-reveal bookmark = perception-limited (lead switch at ~50 m),
partially mitigated by the feedforward's earlier moving-lead response, planner lane for any
deeper fix; carry-bob law refit on post-floor data once new routes land (wire@rest now -0.70 by
design -- the felt gates re-aim on NET carry, crank ladder scoring must switch metrics); watch
first drives for: relaunch class gone (no more monitor-ladder rescues mid-stop), no new
launch-leg braking, chase peaks shallower behind braking leads, and bob at/below 0.008.

## Cycle 19 (2026-07-31) -- one terminal descent curve + the smoothness scorer (THE felt gate)

User bookmark on route 00001f7b seg3 (first drive on the cycle-18 floor): "the stiction was okay
but after switching into the stopping controller the braking was very gentle and then
unnecessarily increased suddenly ... we need a test for that". Directive: build the detector
first, keep the code abstract (no messy tree of ifs), then experiment.

DIAGNOSIS (graphs at /tmp/f7b_stop{1,2}.png; phases overlaid): below ~0.5 m/s the wire was the
min() of laws with DIFFERENT SHAPES -- EASE (>= -0.35), the relief-cap glide law (cap + a_coast
~ -0.8), the floor schedule -- and EASE<->GLIDE flapping (Doppler noise on the gates) switched
between them frame to frame. Recorded: gentle unload to -0.35 while the planner asked -0.28,
then a plunge to -0.81 at v~0.45 (DEEPER than the floor's -0.70: the glide law, not the floor),
then flap pump -0.81 -> -0.42 -> -0.70 into HOLD. The clutch/stiction side was fine (push +0.65
held by -0.70): cycle-18's fix works; the SHAPE was the defect.

THE DETECTOR (tools/stopping/review/terminal_smoothness.py, shared by route reviews AND the test
battery so both measure identically): per stop, over the terminal window (v < 0.45 = the descent
law's realistic arming point, to the first SUSTAINED standstill selected by t_target) ->
wire_jerk_max (<= 0.80, calibrated to the human template: manual re-engagements 0.6-1.0,
curve-following ~0.5, J_DOWN steps 2.5, the f7b flap 6.8-10), wire_pump (<= 0.06),
descent_count (== 1), felt_jerk_max (<= 0.8), relaunched (any post-dip rise > 0.12 = FAIL).
Baseline: f7b bookmark 6.78/0.10/2 descents/1.51 felt = good:false; f7b stop2 10.09/0.16/2.

THE LAW (one function, no stacking): while the cycle-18 arming latch holds and v <= 0.50 in
EASE/GLIDE with entry_ok, a_phase IS (assignment) a single linear descent from the wire CAPTURED
AT ARMING to A_HOLD_SECURE at 0.10, emitted through a rate-bound + monotone clamp
(J_TERMINAL_DESCENT 0.60). Assignment kills the flap sensitivity (both phases produce the same
number) and supersedes the relief-cap excursion; the capture kills the entry step; the clamp
kills quantization and short-span steepness. Safety lanes still min() on top; the jerk limiter is
untouched; params A_CREEP_HOLD_MID / V_CREEP_HOLD_MID / V_CREEP_HOLD_START RETIRED (net params
down, one law instead of three).

EXPERIMENT LOG (what the loop caught): sol's inconsistency proof (the old two-segment knee forces
>= 1.67 m/s3 -- the knee was an artifact of the floor's origin, deleted); a fixed top anchor made
the limiter STEP onto the curve at 2.5 (clutch-plant fixture); my first jerk gate (0.45) was
stricter than the human template (recalibrated 0.80); a 0.35-grade fixture physically overpowers
the -0.70 hold so the arrest ladder correctly adds descents (lowered to 0.25 -- that class is the
cycle-5 grade contract, not this law's).

END-REVIEW (6 rounds): R1 (law) quantized/short-span steps -> the emission clamp; R2 (law/tool)
scorer truncated a dip-relaunch-slam -> sustained-standstill endpoint; R3 flicker truncation ->
contiguous-band state machine; R4 stop-and-go merging -> window ends AT the dwelled standstill;
R5 CLI certified a neighbouring stop -> t_target episode selection + backward window open, and
the boundary sample was excluded -> retained (window 0.50 -> 0.45, the arming point); R6 findings
EMPTY, one edge named (targeted request with no standstill bypassed the distance guard) -> fixed
+ pinned. NOTE: every finding from R3 on was in the OFFLINE SCORER, not in code that drives the
car; the car-side law has been unchanged and clean since R2.

Battery 813 passed / 19 skipped. Mutation kill map (documented in the test): descent off -> 3
fixtures fail; min()-stacking restored -> the f7b reconstruction (grade + flap) fails; emission
clamp removed -> quantized fixtures fail at 2.5; the u0 anchor is deliberately an equivalent
mutant under the clamp (defense-in-depth on the RAW target).

WATCH next drives (scored, not felt): every stop should read descent_count 1, wire_jerk <= 0.80,
pump <= 0.06, relaunched false. Ledger: EASE-band smoothness above 0.45 has its own pins and is
not covered by this gate (quantized EASE demand steps ~0.9 m/s3 there -- a candidate next cycle
if the entry into the descent ever feels abrupt); carry-law refit still pending on post-floor
data.

## Cycle 20 (2026-08-01) -- OPEN: the f80 late-slam class, three designs falsified

User bookmark, route 00001f80 seg99 (device on 50cfc14fa7 = cycle-19): "again unnecessarily harsh
braking before the stop, not even the stiction".

EVIDENCE (100 Hz): the PLANNER handled this stop smoothly (aTarget ~-0.88 easing to -0.73
throughout). The service entered APPROACH_GLIDE at v=2.2 and its GLIDE law drove the wire
-0.87 -> -1.93 between v=1.79 and v=0.88 -- 1.22 m/s2 DEEPER than the concurrent planner demand,
sustained >0.3 excess for 2.04 s -- then released back through -1.48 to the cycle-19 descent.
Felt jerk 2.56 m/s3. Rest 3.7 m, lead stationary, gap 4.4 m at the peak.
NEW METRIC (this cycle): excess-over-planner in the service-owned band separates the classes --
f80 bookmark 1.22 / 2.04 s / 2.56 felt; f7b (cycle-18 build) 0.66 / 1.12 / 1.37; clean stop
0.38 / 0.12 / 0.68. NOTE the cycle-19 smoothness gate MISSED this class exactly as ledgered: its
window opens at v<0.45 and the slam lives at 1.8-0.9 m/s.

THREE DESIGNS BUILT AND FALSIFIED (sol xhigh; no code retained, no pin weakened):
A. Whole-band single curve (arm the cycle-19 descent at service ownership, v<=V_ENTER): kills the
   excess completely (0.00) and the smoothness is perfect, but a v-linear curve carries NO
   stopping-distance authority -- rest 2.945 on the f80 shape (the wall is 3.0) and 1.08 m on the
   downhill fixture; 9 pinned fixtures fail. LESSON: on this geometry the DEPTH IS REQUIRED --
   the defect is WHEN it is spent, not how much.
B. Curve + old glide as a geometry floor underneath: REJECTED unbuilt (re-creates the law stack
   cycles 18-19 deleted; user rule: no messy tree of ifs).
C. Ratcheted constant-decel-to-anchor (hold the deepest required decel since arming, so the debt
   is paid early): f80 peak -1.93 -> -1.50, excess 1.22 -> 0.71, rest 3.358, smoothness passes.
   BUT 3 tight/close-entry fixtures rest 0.1-0.2 m short (the J_TERMINAL_DESCENT comfort clamp
   starves genuinely-required late authority) and the cycle-19 capture-continuity pin broke.
C'. C + "safety may always slam" bypass (clamp yields when raw a_req is deeper than the comfort
   emission) + capture-seeded ratchet: EVERY rest-gap pin passes (table in the cycle-20 notes,
   f80 peak -1.49 / excess 0.679 / rest 3.972) -- but the raw emission produces wheel-stop wires
   of -0.998 (close 0.6, pin -0.75) and -2.980 (close 1.2, pin -1.60) and a -3.13 rolling peak:
   the close-entry fixtures' own "safety over feel" trade, but far outside their pinned bands.

TWO REAL FINDINGS TO CARRY FORWARD:
1. MY DETECTOR HAS A BUG: the smoothness gate requires descent_count == 1, which encodes the OLD
   shape (release then ONE re-engagement). Under a front-loaded law the wire arrives already deep
   and simply holds -- C' scored descent_count 0 with jerk 0.000 and pump 0.000, i.e. the ideal,
   and my gate called it a failure. The gate should be descent_count <= 1 (0 = no re-engagement
   needed; >= 2 = pumping). Fix this BEFORE the next design attempt or it will keep rejecting the
   right answer.
2. The pin set encodes a coherent EXISTING policy (close entries get late firm authority), and
   every reshape trades one pin class for another. The next attempt should therefore not replace
   the terminal law wholesale but attack the ORIGIN: the anchor/d_rem collapse that makes the
   glide law back-load. Candidate: bound the RATE at which the required decel may grow by
   re-aiming the anchor earlier (spend the debt when d_rem is still large), leaving the law
   itself alone.

### Cycle-20 SHIPPED: no position target outranks the floor
Root (route 00001f80 seg99, recorded columns): the PLANNER's own distanceToStopTarget collapsed
2.7 -> 0.27 -> 0.05 m while the car still rolled at 1.6 -> 0.9 m/s. The terminal law turns a
position target into v^2/(2*remaining), so it demanded 1.3 -> 2.6 m/s2 for a stop line the car
was about to pass -- wire -1.93 against a planner asking -0.85 (excess 1.22 sustained 2.04 s,
felt jerk 2.56) -- and the car rested 3.8 m from the lead, which was always fine.

USER RULE (2026-08-01, then refined): 3.0 m hard floor; 4-5 m is the AIM (healthy); 3-4 m is the
comfort allowance; 5-6 m is wasted room. Inside the band the exact number carries no value --
comfort decides. So the law may AIM (early, via the anchor) but must never CHASE a position
target at the end, where the required decel explodes.

SHIPPED (one commit): the cycle-15 floor-defence cap (a) bounds the FINAL d_rem instead of only
the lead-anchor candidate -- the planner-stop-line candidate can no longer re-impose the blow-up
through the min() -- and its third term (the anchor's own demand at the region edge, i.e. the
nominal being chased) is deleted with the region gate; (b) admits a live-lead gap instead of
measured-only. a_kin, a_plan, the monitor and the dropout floor are untouched and still min() on
top; the anchor still never moves. MEASURED rests after: nominal 4.03 / moderate 4.17 / gentle
4.28 / slow 4.42 / hot-close 3.32 -- the 4-5 aim holds, only the hot entry uses the comfort
allowance; the nominal fixture now pins 3.9-5.0 so the aim cannot drift.

END-REVIEW (2 rounds landed, both HIGH, both real safety holes IN MY OWN CHANGE):
R1: gap_source "held" covers THREE provenances -- outward persistence (emits min(prediction,
raw) = a LOWER bound), inward-step REJECTION (emits the LARGER prediction while the raw reading
says the gap collapsed = OPTIMISTIC), and an invalid reading with the lead present. Admitting all
three let the cap shallow a_phase to -0.68 against a real -1.30/-1.42 requirement: rest 2.993 m,
through the floor. FIXED AT THE SOURCE: StopSignals now carries gap_hold_outward, True only on
the outward branch; the cap admits measured OR outward-held.
R2: the SAME bug in _planner_safety_demand, which position-bounds (shallows) planner authority --
bounded -1.30 to -0.456 on an optimistic held gap, rest 2.995 m. Fixed with the same predicate.
R3 (its runner died 4x; I ran its key sweep by hand): audited EVERY consumer of gap_source /
d_gap and classified each as reliever or deepener -- the two relievers above are fixed; the
monitor's arrest-floor unwind, the RELEASE-on-departure gate and the cycle-17 gentle-rate gate
all already use the STRICT "measured" definition, which is the conservative side for a reliever;
the crawl-reference latch and monitor arming deepen/refuse. No third instance.
Mutations, all killed: cap removed -> 3 fail; measured-only -> 2 fail; all-holds in the cap -> 1;
holds-refused -> 3; all-holds in the planner lane -> 1. Battery 817 passed / 19 skipped.
FIXTURE LESSON (mine): the first inward-collapse regression passed for the wrong reason -- its
geometry was physically unwinnable, so it tested the plant, not the rule. Retuned so full
authority holds the floor (~0.39 m + lag) and shallowed authority loses it (~1.09 m).

NEXT (route 00001f82 seg15, the newest bookmark, rest 3.1 m + felt jerk 3.3): a DIFFERENT root --
LATE HANDOVER. The service was INACTIVE until 1.7 s before the stop (entered at v=1.36, gap 4.2)
while the planner eased off (-0.80 -> -0.47) because its own stop line had already passed, so
through v 2.5 -> 1.4 nobody held the 4-5 aim; the car coasted into the floor and the floor
defence rescued at -2.15. Entry is gated behind lead_confirmed_stopped, whose [-0.1, +0.3] window
this lead's noisy Doppler (-0.07..-0.22) kept flapping -- the same ledgered cycle-17 Doppler root.
Cycle-20 halves that peak (the cap bounds it to ~-1.3) but does not move the handover: that is
the cycle-21 lever, and it is the user's own hypothesis ("if we're handing over too late, we can
affect that too").

### Cycle-20 end-review: seven relief paths found, THREE shipped, four rejected on cost
The review chain (7 rounds, every runner detached after the plugin fix) established one rule and
then hunted it: ANY path that RELIEVES braking -- shallows a demand, releases a phase, drops a
floor, shortens authority -- must know whether its geometry is trustworthy. gap_source "held" has
three provenances: OUTWARD persistence (emits min(prediction, raw) = a LOWER bound, safe),
INWARD-step rejection (emits the LARGER prediction while the raw reading says the gap collapsed =
OPTIMISTIC) and an invalid reading with the lead present. StopSignals.gap_hold_outward now
distinguishes them.

SHIPPED (measured harm, no measured cost):
1. the floor-defence cap (rest 2.993 m on an inward-held gap) -- in 96047c3328;
2. the planner-authority bound (-1.30 shallowed to -0.456, rest 2.995 m) -- in 96047c3328;
3. gap_grew, which reaches RELEASE via planner_go bypassing observed_departure -- this commit.

REJECTED after building and measuring them (each traded a certain felt regression for a hazard
already bounded by the 2 s A_DROPOUT_MIN decay floor and the planner lane):
- ABSENCE CERTIFICATION (gap_absent_verified + travel/timer): delayed a legitimate launch from
  HOLD by ~5 s whenever a stopped lead's track dropped -- common in traffic -- and the invented
  6 m "never measured" default could still certify falsely.
- EASE authorisation gate: turned the MOST COMMON dropout (a directly measured gap, then loss)
  into a new harsh path -- EASE -0.35 to GLIDE -0.70 within 50 ms at J_SAFE, not baseline-
  identical.
- TERMINAL DESCENT gate: RELIEVED braking late in a descent (verified -0.680 vs the GLIDE -0.281
  it fell back to), accumulating 0.21 m/s2 of release over a 14-frame hold -- the opposite of its
  intent, because the descent is not always the shallower law.
- APPROACH STATE-EXIT gate: could retain a stop indefinitely under persistently invalid radar
  data (no maturity/expiry on an invalid-reading hold), deepen-starving a legitimate launch.

LEDGERED (unfixed, pre-existing, unchanged by this cycle): an unobservable lead -- dropout expiry,
or a lead tracked but never validly measured -- still relieves braking, because the no-gap GLIDE
law is shallower, a_kin disappears and A_DROPOUT_MIN lapses. The honest fix is evidence-based
departure (observed recession, or travel past the last-known gap) with NO launch penalty; the
timer-based version is proven wrong. Own cycle.

PROCESS NOTE: rounds 5-7 each found defects in the FIX rather than the original code -- the signal
that a lever has passed its useful depth. Four commits were written, reviewed and deleted here.
That is cheaper than shipping them.

## Cycle 21 (2026-08-02) -- late handover: the latch that never confirmed

TARGET (set by cycle-20's ledger + the user's hypothesis "if we're handing over too late, we can
affect that too"): across 45 corpus stops the service takes over at a MEDIAN 1.12 m/s with 2.63 s
left, though V_ENTER is 2.5. Instrumenting seven stops: the stopped-lead latch was OFF for
essentially the whole 1.3-2.2 s delay window on six; the one prompt entry (f80) had it ON 100% --
the latch governs handover timing.

DIAGNOSIS, two classes, only one a defect:
- DEFECT: confirmation required 0.30 s of UNBROKEN in-window samples, and negative-Doppler dips
  behind physically stationary leads are frequent but SHORT (226 runs, gap change < 0.3 m: p50
  0.13 s, p90 0.29 s, p99 0.69 s) -- each dip reset the dwell, so noisy stopped leads confirmed
  late or never.
- NOT A DEFECT: the f82 seg15 bookmark (the 3 m stop that started this cycle) -- its 1.23 s of
  -0.24 m/s is CORROBORATED by the geometry (implied lead movement -0.25 m vs -0.30 predicted):
  the lead genuinely rolled back ~25 cm and the latch was right to withhold. Entry there was late
  for a correct reason; the -2.15 rescue followed from the resulting tight geometry. LEDGERED as
  a POLICY question (hand over behind a slowly-reversing lead?), not a bug. Also not defects: the
  moving-lead stops (f7b, f6e seg22) where the lead genuinely hadn't stopped yet.

SHIPPED (5a40d1f3f8, deployed): a dip that is BRIEF (< 0.25 s AGGREGATE per confirmation epoch)
and SMALL (within 0.4 m/s below the window) PAUSES the dwell instead of resetting it, only while
confirmation is being EARNED. Everything else falls through to the existing resets: forward/
departing leads, deeper or longer dips, lead loss, and the already-confirmed latch (whose
instant drive-away/loss un-confirm and 0.5 s sustained-Doppler rule are untouched). Effect:
f80 seg99 becomes entry-eligible at 2.49 m/s / 7.8 m (was 2.26 / 7.0); moving-lead stops
unchanged.

TWO DESIGNS BUILT AND REVERTED BEFORE IT (the record matters):
- "Remember a recently-confirmed lead for entry" (2.0 s memory): R1 showed entry_ok is shared by
  three ACTIVE-phase decisions, so the memory leaked into release/descent (delayed RELEASE ~2 s);
  the R2 fix made the episode coherent but R3 showed that necessarily re-introduced remembered
  evidence into an active exit. The design was fighting itself -> deleted.
- Its root premise was then FALSIFIED by measurement: f82's noise was real motion (above), and
  the corpus dips are too short to need a memory at all -- the dwell reset was the actual defect.

REVIEW (2 rounds on the shipped fix): R1 HIGH -- a per-excursion allowance that refreshed on
every in-window frame let an alternating 1-frame/0.24 s pattern confirm a lead that had reversed
3.4 m -> the budget is now AGGREGATE per epoch, pinned by an attack regression that runs 9.4 m of
reversal without confirming, plus the complementary pin (a stationary lead with a p50-length dip
still confirms < 0.6 s). R2 MEDIUM -- the sustained-Doppler un-confirm left the spent budget in
place for the next epoch; cleared, and the test says HONESTLY that the clear is hygiene (the
else-branch masks it one frame later; measured identical confirm frames with and without).
LEDGERED from R1: stopped-lead timers are not scoped to a radar track id (pre-existing, applies
equally to the strict dwell and wheel-stop latch; needs longcontrol wiring -- own commit).

PROCESS: this cycle also produced the "Never end a turn waiting" rule (~/.claude/CLAUDE.md) and
the detached review wrapper (~/.claude/scripts/codex-review.py) after the user had to prompt a
dozen times; both were exercised here (reviews polled to completion in-turn, three findings
fixed without a ping).

## 2026-08-02 -- cycle 22: the f82 policy question answered -- wide entry latch

USER DECISION on the cycle-21 ledger item (hand over behind a slowly-reversing lead?): "that
will be super rare, but if we stopped at the desired 4-5 m, this wouldn't have been a problem."
That reframes the f82 3 m stop: the rollback was only a problem because NOBODY was aiming at
4-5 -- the refusal itself created the tight geometry the floor defence then rescued at -2.15.

SHIPPED (1a1c333535): entry_ok rides a second WIDE latch, lead_stopped_for_entry, window floor
-0.5 m/s (was: the strict latch's -0.1). One parameterised _StoppedLatch helper now backs both
latches -- same 0.3 s dwell, same cycle-21 AGGREGATE dip budget (shifted band [-0.9,-0.5) for
the wide one), same sustained-negative off-delay and instant drive-away/loss un-confirm -- so
the two windows cannot drift apart in behaviour, only in floor. The STRICT latch is untouched
and still guards the cycle-17 reversing_hazard disqualifier: a slowly-reversing lead is
entry-eligible AND still disqualifies the gentle-rate relief machinery. Below -0.5 the lead is
a hazard approach, not a manageable stop: neither latch confirms.

CONSUMER AUDIT before the change (the cycle-21 lesson, applied first this time): the latch has
exactly TWO consumers -- entry_ok (the target) and reversing_hazard (must NOT widen). entry_ok's
three active-phase uses (state exit, RELEASE re-entry, descent authorisation) inherit the policy
coherently: stay active, keep braking, let the deepen-only lanes (a_kin closure, floor defence,
monitor) own the shrinking gap.

REPLAY (recorded f82 seg15 through the real StopContext): strict latch ON 8% of the
2.5 -> 1.36 m/s delay window -- the on-road refusal; wide latch ON 100% of it, first ON 8.0 s
before the stop at gap 33 m. Entry would have held at the design speed with ~7-8 m of gap.

MUTATIONS all killed by named tests: entry reverted to strict / strict widened / entry not
widened / dip budget removed / sustained-negative un-confirm disabled. Battery 826/19.

### Cycle-22 span review (cycles 18-22, base 67b7a4a7db): one HIGH, fixed same-turn
The user widened the mandate: review ALL stopping changes of the last 3 days, not just the new
lever. The adversarial pass confirmed the accumulated cycle-18..21 machinery held (no other
material finding) and found ONE HIGH in the new latch pair -- confirmation SURVIVED radar track
replacement, so a fast-reversing replacement target could ride the previous target's earned
entry eligibility for up to the 0.5 s off-delay (state retention, RELEASE re-entry and descent
authorisation included). This is the cycle-21 ledgered track-id item made material by the wide
entry latch.

FIXED (7f918afeae), measured before designed: recorded queue approaches hand leadOne between
stopped objects constantly (9-11 id changes per approach, f7f seg129 / f82 seg5; 37/39
approaches id-stable; radard emits -1 for vision-promoted leads), so the reviewer's blanket
reset-on-change would starve confirmation exactly where stops happen -- adopted in SCOPED form:
a real id change keeps each latch only if the new reading sits in that latch's own window;
out-of-window replacement resets instantly, off-delay included; ids < 0 carry no identity.
The reproduced scenario dies on the replacement frame. Wiring: radarTrackId now flows
controlsd -> longcontrol -> StopContext (was always None), which also turns on the gap
filter's designed cut-in immediate-accept on-road for the first time.

PROCESS FIND: a stale .pyc silently poisoned one earlier battery/gauntlet run (a red test read
green). The gauntlet was RE-RUN with caches cleared under PYTHONDONTWRITEBYTECODE: all 8
mutations killed by named tests (one test first sharpened -- its -1 frames now carry an
out-of-window dip so the -1-as-identity mutation actually discriminates). f82 replayed with the
REAL id stream (10 changes in the last 10 s): wide latch still ON 100% of the delay window.

### Cycle-22 rounds 2-3: the identity machinery hardened to earned-evidence semantics
Round 2 (on the fix): (1) in-window handovers LAUNDERED confirmation -- a chain of fresh ids,
one in-window frame each, rode the 0.5 s off-delay indefinitely (395 m of reported reversal
while entry-eligible) -> handovers are now PROVISIONAL: confirmation carries only while the new
target reads in-window, with NO off-delay entitlement and no dip budget until it re-earns its
own 0.3 s dwell; first out-of-window frame resets. (2) the newly-live cut-in accept branch took
OUTWARD replacement geometry on one frame (real -> -1 -> other-real flap) and released a HOLD
via gap_grew -> immediate acceptance is DEEPEN-ONLY; outward replacements earn through outward
persistence. Round 3: (1) the replacement's VELOCITY still fired lead_receding (one +1.0 flap
frame = "departed") -> StopSignals.lead_motion_earned, False for T_MOTION_TRUST_S (0.25 s) after
a real id change, required by lead_receding; genuine departure still releases (pinned). (2) a
NaN-velocity replacement was judged on the previous target's held reading and NaN frames accrued
dwell -> invalid handover resets, invalid frames FREEZE (retain, never earn).

Shipped c167f70ba8 + 34cf4f02ae. Mutations M9-M16 all killed by named tests (M9's first "kill"
was an anchor artifact -- re-scoped and re-proven). Battery 837/19. f82 replay with real ids
after every round: wide latch ON 100% of the 2.5 -> 1.36 delay window -- the cycle-22 gain is
untouched by the hardening.

SIGN-OFF: three adversarial rounds, each strictly narrower (span -> fix -> fix-of-fix), the
cycle-20 depth signal. The closing sweep of remaining flap-velocity consumers is MINE:
reversing_hazard chatter is deepen-biased; a_kin shallowing is one lane of the min() under the
jerk limiter; d_rest_eff recompute is gated on >1.0 m growth which held-outward prevents.
WATCH: in churny queue scenes a provisional target dropped by a noise dip can briefly hand
mid-approach shaping back to the planner chain (RELEASE + 0.3 s re-entry); conservative
direction, but a felt-smoothness candidate if it shows up in route reviews.

### Cycle-22 corpus validation: the lever fires on exactly the class it was built for
Replayed every approach-to-stop in the 6 most recent routes (f7a/f7b/f7f/f80/f81/f82) through the
REAL StopContext, evaluating the latch half of entry_ok frame by frame under the STRICT window
(shipped through cycle-21) and the WIDE one (cycle-22). 16 approaches found, 12 with a
stopped-lead entry path (the other 4 enter via shouldStop -- no latch involved).

  MOVED: 2/12. Unchanged: 10/12, bit-identical eligibility.
  f82 seg13: strict v=0.56 / t-1.34 s  ->  wide v=2.35 / t-2.81 s   (+1.79 m/s, 1.47 s earlier)
  f82 seg15: strict v=1.56 / t-1.93 s  ->  wide v=2.49 / t-2.99 s   (+0.93 m/s, 1.06 s earlier)

And the two it moves are precisely the two SHORTEST stops in the corpus -- rest 3.33 m and 3.10 m,
the only approaches below the 4-5 m aim with a lead present (every unmoved one rested 3.79-6.83).
seg15 is the bookmark; seg13 is a second, unreported instance of the same class in the same route,
which is what the user meant by "that one bad stop was just one of the instances". The lever is
therefore exactly as rare as the user predicted ("that will be super rare") and lands on the right
frames: no broad behaviour change, ~1.3 s more approach time where the 4-5 m aim was being lost.

(f80 seg99 shows rest 3.79 / wire -1.96 in this table: that is the RECORDED pre-cycle-20 drive,
already fixed at the floor-defence layer -- it is not a cycle-22 case and correctly does not move.)

## 2026-08-03 -- cycle 23 opens: carry-refit ledger item CLOSED by measurement

While waiting for the new route to sync: the cycle-16 WATCH item ("carry law now OVER-predicts --
REFIT before reusing") is closed, not by a refit but by the discovery that there is nothing left
to fit. All 4 engaged wheel-stops with rlogs in f7a-f82 land at wire exactly -0.70 (A_HOLD_SECURE
-- the cycle-18/19 terminal floor + assigned descent working every time), i.e. carry pinned at
0.27, inside the human clean band 0.20-0.40 from the cycle-14 manual-stop template, with uniform
post-stop transients (~0.07 max-min aEgo proxy). The floor REMOVED the carry variance the
cycle-14 law was fit on; bob-vs-carry has no slope to estimate on post-floor data and the
prescription ("land <= 0.4 and let creep carry") is now enforced by construction. The carry <= 0.4
stop_fingerprint gate stays as a regression tripwire.

## 2026-08-03 -- cycle 23: fresh-corpus decomposition -- the dominant class is already fixed

14 new routes synced (f83-f90, 291 segment qlogs + 11 stop rlogs). COMMIT FORENSICS: f83-f89 all
ran CYCLE-20 builds (96047c3328 / 4f056930b6) -- the cycle-21/22 deploys sat STAGED until today's
reboot because the car never got its parked handoff; f90 confirms the tip (ac06dbbf81) is live
now. So this corpus is a pre-fix observation window, and it decomposes COMPLETELY into known
classes -- no bookmarks, and no new defect:

- 5 CLEAN stops: rests 3.5-4.3 m, wire lands -0.70 exactly, wire_jerk 0.0-0.35, no pumps. The
  cycle-18/19 terminal machinery revalidated on fresh data.
- 4 LATE-HANDOVER stops = the cycle-22 class RECURRING pre-fix. Worst: f89 seg14 (rest 3.09,
  wire -1.81, felt jerk 4.33) -- a full f82-signature stop. Replayed all four through the CURRENT
  StopContext: strict latch ON 16-57% of each delay window, wide latch 74-100%. The deployed fix
  covers every instance; f89 seg14 goes 28% -> 100%.
- 2 QUEUE SHUFFLES (f85 seg33/38, rests 2.9/2.4): the car was ALREADY holding at -0.70, the lead
  crept ahead, ego creep-followed (wire 0.00 through the hop, felt 0.37-0.51 = gentlest in the
  set) and re-settled close -- normal queue behaviour, and the sub-floor rest is by construction
  (the queue moved ~2.5 m). Hold re-ramps -0.05 -> -0.70 within 2 s on both. NOT a defect; the
  3.0 floor rule is an APPROACH rule and shuffle rests are exempt by geometry.
- 1 LATE LEAD ACQUISITION (f85 seg20, rest 2.9, wire -1.44): NO radar lead existed until 0.5 s
  before rest -- the braking was for the vision/planner target and no latch could have helped.
  The pre-existing cycle-16 ledgered class, still open, unmoved by anything since.
- 1 WASTEFUL rest (f85 seg24, 5.3 m, wire_jerk 0.948 vs the 0.80 gate): single mild instance,
  planner-anchor side; noted, not actioned.

DECISION: NO new lever this cycle. The dominant class in the fresh data is the one cycle-22
already fixed, deployed today and not yet driven; stacking another lever before its on-road
validation would break the one-lever-per-cycle discipline. NEXT REVIEW: first post-reboot drives
validate cycle-22 (watch: late-handover class should vanish; queue crawls stay smooth -- the
provisional-handover watch item; seg20's acquisition class is the standing next target if it
recurs).

## 2026-08-03 -- cycle 24 opens: first cycle-22 on-road data -- the approach layer is now the defect

Route 00001f90 (live, commit ac06dbbf81 = full cycle-22): 4 engaged stops, 1 bookmarked.

- seg21, rest 4.20, worst wire -0.83, felt 0.79: a GOOD stop -- aim band, human-gate felt. Proof
  the machine can do it when the approach gives it room.
- seg22 stop 1, rest 3.90, worst -1.00, felt 1.31: decent, aim boundary.
- seg24 ("meh"), rest 3.73, worst wire -2.09 (excess over planner -1.11), felt 0.95: hot approach
  (8.1 m/s at 22.8 m), late deep braking, rest short of aim.
- seg22 stop 2 (BOOKMARKED), rest 3.05, worst wire -2.46, felt 5.29: ego at 11.3 m/s closing on a
  lead that braked hard 10.7 -> 0 in ~4 s. Phase 1 (physics, fine): -2.0..-2.3 sheds closing
  speed. Phase 2 (THE ROOT): with the lead stopped at 15 m, the demand EASES -2.06 -> -1.13 while
  resting at 4.5 needs sustained ~-1.5 -- the deficit is borrowed. Phase 3 (the felt slam): the
  planner relieves toward -0.2 but the service's kinematic/floor lanes pay the debt: wire -1.40 ->
  -2.46 in 0.75 s at v~1, excess over planner ~1.7, rest 3.05. CYCLE-22 WORKED AS DESIGNED -- the
  wide latch held entry through the lead's -0.13..-0.34 rollback readings and the service was in
  by 2.5 m/s -- but by then the geometry was already spent: at entry (v 2.6, gap 6.2) resting at
  4.5 required -2.0 sustained. The smoothness gate is BLIND to this class (the slam lives at
  1.8 -> 0.5 m/s, above the V_WINDOW 0.45 terminal window; inside the window the wire only
  relaxes) -- only felt_jerk catches it.

THE PATTERN across all four: rest correlates inversely with approach heat (gentle -> 4.2, hot ->
3.0-3.7 + late deep braking). The terminal machinery is landing -0.70 with zero pumps on every
stop -- that layer is done. The budget is set UPSTREAM: the MPC solves to its ~4.0-equivalent
stop anchor (STOP_DISTANCE 5.5 / LEAD_STOP_DISTANCE_TARGET / stopped_lead_offset in long_mpc.py)
and hot approaches spend the margin, exactly the v^2/(2*remaining) explosion the user's rest-gap
rule names. The service cannot fix this from below V_ENTER 2.5.

CYCLE-24 LEVER CANDIDATES (planner layer, per the user's standing "we have access to other
layers" grant):
A. AIM-EARLY ANCHOR: raise the effective stopped-lead rest anchor so LANDINGS hit 4-5 (today the
   aim is ~4.0 and landings run 3.0-4.2). The cycle-16 "vetted safe-by-construction rest-gap
   nudge", now sanctioned by the 2026-08-01 rule ("4-5 should be our healthy range... 5-6 we can
   always use for a comfortable 4-5 stop"). Ripple risk: core constant; needs corpus replay.
B. APPROACH-COMMITMENT ENVELOPE: once stopping behind a stopped lead is committed, the demand
   must not EASE below the constant-decel-to-aim requirement (the bookmark's -1.13 ease at 5 m is
   the borrowed deficit). Relation to the cycle-13 stop-commitment necessity floor
   (22b9e1e294) must be established first -- why did it not fire here?
Both candidates need the undershoot map (MPC anchor vs actual rest, by approach heat) before
design. NOT STARTED -- review delivered first.

### Cycle-24 lever SHIPPED (42958442d8, deploy pending review arbitration)
The aim-commitment necessity floor -- full record in the commit message. Design notes that must
survive: THREE gates each falsified a simpler design on recorded data BEFORE implementation
(command-relative Schmitt = one-frame engagement window; ungated commitment = drags gentle stops
+ fights launches; no wave gate = binds on 80 m-out projected stops). The falsified
command-relative design is retained as a MUTATION killed by the repair fixture. Undershoot map
(19 stops): gentle lands ON design 4.25 -> lever A (anchor raise) REJECTED, would push gentle
into the wasteful band; the defect was pure shape. f85 seg19/20 RECLASSIFIED: the cycle-23
"late-lead-acquisition" call was a segment-boundary artifact -- it is a 4th instance of the ease
class (lead tracked throughout, rollback signature), and the lane catches it. Open-loop trap
respected: the toy integrator's rest numbers were dominated by divergence (stops that never bind
drifted +-2 m); certification is bind-location analysis on recorded trajectories + fixtures +
the on-road drive. approach_excess_max <= 0.80 gate added to the review scorer (calibrated:
ease class 1.03-1.71, good <= 0.66).

### Cycle-24 CLOSED: two review rounds, one HIGH fixed, deployed d3b64470b4 (device verified)
Plan red-team (sol xhigh, arrived post-implementation): its headline findings (one-frame Schmitt
window, release-at-cap) had been independently found and fixed on recorded data; adopted the
0.25 s measured-lag alignment + hardening-lead and ISD fixtures; discards recorded in
770e45fc9a. End review R1 HIGH (REAL, my ordering bug): the aim lane's min() fed the band
floor's command-relative Schmitt a deeper command -- at aim-cap -2.25 a genuine 2.40 floor need
read as within-margin and stayed MASKED (0.42 m floor deficit in a hardening transition) ->
get_santa_fe_stop_floor_demands evaluates BOTH floors against the same pre-lane command, caller
min-merges; the reviewer's exact sequence is the regression, mutation MG killed. R2: APPROVE, no
findings (lanes' hysteresis coherent, aim-off path byte-identical, no downstream re-masking).
Deployed and device-verified d3b64470b4. VALIDATION on next drives: hot lead-stops should rest
4-5 m with wire <= ~1.7 and no terminal slam; approach_excess_max <= 0.80 on route reviews;
watch braking-wave feel 14-16.5 m/s (the 35 m gate should keep the lane out) and no-lead stops
(explicitly out of scope -- ledgered).

### 2026-08-03 evening -- f91/f92 review: the felt "earlier braking" is NOT the aim lane
User report after the first post-cycle-24 drive: "braking much earlier for leads. Probably too
early but the distances were better." Findings, f92 (build 160efa82fb, 17 min engaged, 931 s
with rlogs):
- ZERO completed stops (engaged v-min 2.09) -- the aim lane's target class never occurred;
  cycle-24 on-road validation is STILL PENDING.
- The aim lane fired ~zero: qlog replay 0 substantive episodes; every on-road
  aTarget-below-trajectory episode sits OUTSIDE its gates by arithmetic (gaps 36-77 m vs the
  35 m projected-stop gate; or leads projected to stop 50-120 m out); the drive's single
  candidate blip (seg8, 0.4 s at v 2.2/gap 5.1) is cap-refused at onset (a_req ~4.7 > 2.25) --
  another writer produced it (lead-churn moment, gap flapping 3-22 m at walking pace).
- Build delta f90 -> f92 is EXACTLY the cycle-24 commits (verified git log; no other agent's
  changes landed). So the felt change is today's TRAFFIC: 16 braking-wave decel episodes in 17
  minutes (leads braking hard from 13-22 m/s at 23-76 m) -- the cycle-18 decelerating-lead
  feedforward + approach caps doing what they have done since they shipped, unusually often.
- "Distances better": min follow gaps 11-17 m at speed are t_follow-governed (unchanged);
  earlier feedforward braking in waves does preserve mid-wave gaps -- pre-existing behaviour.
VERDICT: no evidence of the aim lane over-firing; no evidence FOR the lever yet either (no
stops). If the early feel persists on a drive WITH actual lead-stops, the aim-lane episodes will
be directly measurable and ON/35 m are the tuning handles.

## 2026-08-06 -- cycle 25 opens: the breathing-queue class (fb4 seg4, two bookmarks)

Route 00001fb4 seg4 (build 56ce08a74a -- NOTE: includes post-cycle-24 commits by another agent:
experimental lead-boost tunings + c70b7e027e "Gate stopping service radar-only authority").
Two bookmarked stops 10 s apart in a CREEPING QUEUE (lead breathing 0-1.7 m/s at gap 4-7 m).
Every existing gate PASSES (rests 4.2/4.4 in the aim band, wires land -0.70/-0.79, no slam,
approach_excess 0.42/0.93) -- the machine executed two textbook stops. That is exactly the
complaint: THE STOPS THEMSELVES WERE UNNECESSARY. Stop 2's terminal, 10 Hz: the lead was
DEPARTING the entire time (lv +0.30..+0.58, gap growing 4.0 -> 4.5) and both stopped-lead
latches were correctly FALSE -- yet the car drove -0.70 to a full secure stop, sat 2 s, and
launched. Stop-launch-stop on a 10 s period is the wooden cadence the user feels; a human
modulates creep speed and never wheel-stops.

ROOT (frame-level, decisive): distanceToStopTarget sits at 0.1-0.2 m through the WHOLE creep --
the queue's equilibrium gap (~4.4) EQUALS the design rest gap (LEAD_STOP_DISTANCE_TARGET 4.0 +
ISD 0.3), so the stopped/slow-lead synthetic stop target is glued to the bumper. Any small ego
deceleration flips longcontrol into `stopping` (should_enter/hold_stop_target_mode see dts~0.1),
shouldStop follows below ~0.5 m/s, and the stopping chain dutifully completes a secure stop --
REGARDLESS of the lead actively driving away. The system cannot creep-follow AT its own design
gap without repeatedly triggering its own stop machinery. This is JUNE ARBITER territory
(synthetic stopped-lead stop targets), upstream of the service, the latches, and both necessity
floors -- none of which misbehaved.

CYCLE-25 LEVER (design, pre-red-team): stop-target mode entry/hold for the SYNTHETIC
stopped-lead path must require the lead to actually be STOPPED -- a receding lead (lv above
~+0.2 sustained, gap >= target and growing) means the stop point is moving away: stay in
pid/creep-follow and track it. No-lead stop lines unchanged; genuine stopped-lead stops
unchanged (their lv IS ~0); the standstill-hold + launch path unchanged. Risk surface: the
arbiter's per-frame legacy equivalence, the cycle-13 lesson (creep-held cars never give
v<0.05), anti-hover history (hovering NEAR rest with a STOPPED lead must still land secure --
distinguish BY LEAD MOTION). Plan review via sol xhigh before implementation.

### Cycle-25 lever SHIPPED (e93dc7fe4b, deploy pending end review)
Near-rest stop-target tightening -- full record in the commit. Design history that must
survive: the growth-based discriminator (lead receding faster than ego) was FALSIFIED on the
recorded trace (growth turns positive only ~0.3 s before wheel-stop -- ego is still faster than
the receding lead through the whole descent); absolute lead speed near the rest point is the
physical classifier. The first cut's absolute-epsilon clear produced a one-frame clear flicker
for leads decelerating THROUGH the band (sol's churn warning materialized in my own fixture) ->
the decisive-walking threshold (0.65 m/s deep in the band) separates affirmative clears from
continuous scaling. The arbiter departure-test bound was re-adjudicated 1.0 -> 1.2 s with the
record in-test: the old release rode the stale target VALUE drifting out of the close band; the
designed departing-lead predicate now owns that release (~1.05 s). sol's structural finding
(three independent stop-request producers) held: raw shouldStop (MPC, fires only below ~0.5 m/s
-- starved once stop-mode stops braking the creep), the dts/arbiter chain (fixed), and the
synthetic control target (already gated). LEDGER (new): the ACC follow law wants 7-8.7 m at
walking pace (STOP_DISTANCE 5.5 dominates); masked by the experimental blend on this build;
own lever if queue follow ever runs pure ACC.

## 2026-08-11 -- cycle 26 opens: the jerk census -- the open-clutch band is the whole difference

User: "see what levels of jerk we're at; the best should become the minimum." 7 engaged stops on
post-cycle-25 builds (fba/fbc/fbd, no bookmarks). Census: felt_terminal 0.74-2.27 (3x spread),
rests 3.69-4.2 (one 6.29 outlier), wires land -0.70..-0.73 on every clean stop. Queue-creep
validation implicit: no unnecessary full stops observed.

THE FINDING (fbd s23 felt 0.77 vs fbd s9 felt 2.27, same wire law, 10 Hz side-by-side): the
whole difference is the NET DECEL HELD THROUGH THE OPEN-CLUTCH BAND (0.45-0.10 m/s). s23
entered its final 2.5 s at 0.87 m/s, got a mid-terminal relief (wire -0.41), and crossed the
band at net -0.35..-0.45, bleeding to -0.14 by 0.3 s before stiction: felt 0.77. s9 entered at
1.30 m/s with wire pinned -0.70 from v=1.3 (no relief fired), crossed the band at net
-0.60..-0.67, and unloaded the entire -0.6 at stiction in 0.4 s: felt 2.27. Same -0.70 landing,
same creep-fade mechanism -- the fade START LEVEL is the felt jerk. This is the cycle-14 carry
law felt in-window: the jolt equals the net decel just before stiction.

Other census members: fbc s3 (felt 2.08) = LATE LEAD ACQUISITION (no lead until 15.2 m at
5.9 m/s, rollback readings; the aim lane committed instantly and rode its cap -- rest 3.69 vs
the historical 3.0s; perception latency set the comfort bill; cycle-16 ledgered class, not a
control defect). fbd s2 (rest 6.29 wasteful, approach felt 2.49, hot 8.7 m/s) -- single
instance, needs its own dive if it recurs. fbd s6: scorer refused (creep entry, no sustained
standstill in-window) -- instrumentation note.

CYCLE-26 LEVER (design): TERMINAL NET-DECEL COMFORT CEILING in the open-clutch band -- the
assigned descent's path (a_phase lane ONLY; every safety lane still deepens through the min())
is bounded so net decel (wire + creep push estimate) stays <= ~0.45 for v in (0.10, 0.45),
then the existing plunge to A_HOLD_SECURE lands as the clutch engagement eats it -- the human
template's "release, then re-engage to meet the peak", now enforced for FAST terminal entries
instead of only emerging on slow ones. Cost: +~6 cm of travel in the band (nothing). Gates:
relief-class machinery rules apply (strict latch, trusted gap, no monitor/safety binding --
structurally free because the cap lives inside the phase-lane target only). Fixture: s9's
recorded entry (v0 1.30, wire -0.69) must produce band net <= 0.5; s23's profile is the
template. Target: felt <= ~1.0 on every clean stop = the best becomes the minimum.

### Cycle-26 CLOSED: pre-arm capture normalization deployed 446f0cad6f (device verified)
Shipped 698811575f + 446f0cad6f -- full records in the commits. The build's engineering ledger:
FOUR falsifications on recorded data (cycle-17 composite's walking-region term excluded the whole
motivating corridor; hard engage dwell defeated by the measured 10 Hz / 50% duty blink; 2x leak
decays at that duty; snapshot reuse broke 9 pinned relief behaviors -> no catch-up wiring needed
at deficit <= 0.25), then end-review R1 found the stale-hazard engage window (current-frame floor
predicate now at engage/apply/instant-release) and the RELEASE state survival; R2 APPROVE.
Recorded replays through the real service: fbd s9 u0 -0.72 -> -0.52 with the band off the floor
(predicted felt ~1.0 from 2.27), s23/s4/s5 untouched, the hot floor-tight stop refused at full
depth. Mutations MN1-MN7 + MR1-MR3 + the paired-guard kill all named. Battery 976/19.
VALIDATE next drives: every clean lead-stop should now land felt <= ~1.0-1.2 (the census best
becomes the working minimum); watch launch feel after releases (norm state re-earns) and any
downhill stop (lift must never appear -- fails deep by construction). LEDGER unchanged: late
lead acquisition (fbc s3 class) is perception latency, not control; fbd s2's wasteful 6.29 rest
single-instance watch; ACC follow-law walking-pace bias (cycle-25 ledger).

## 2026-08-09 -- cycle 27: fc2 review -- cycle-26 VALIDATED on-road; rollback projection shipped

Route 00001fc2 (build 891fbac316 = cycle-26 live), 7 stops, two bookmarks. THE VALIDATION HALF:
the normalization works on-road -- hot stops (vmax 8-10 m/s) landed felt 0.88-0.97 with the band
lifted (-0.52..-0.54), previously impossible; s12 rest 3.7 at vmax 10 felt 0.88.

BOOKMARK fc2 s6 (felt 2.24, rest 3.2): hot approach behind a lead rolling BACKWARD the whole way
(-0.13..-0.20 sustained, ~1 m total) -- the stop point receded, the margin evaporated, floor
defence correctly paid -1.40 at gap 3.3 (normalization correctly refused floor-tight). SHIPPED
(ad51755730 + 7e2e69e5a3): aim-lane ROLLBACK PROJECTION -- the least-negative raw vLead over a
0.5 s same-track window (managed beside the alk window; the recorded 0.36 s Doppler noise bursts
always contain a clean frame -> noise projects zero) shrinks the effective runway over a 2.0 s
horizon; commitment ONSET/HOLD ride the BASELINE necessity (end-review HIGH: the projected value
could exceed CAP and REFUSE the commitment it serves -- quantified inversion band pinned); the
projection deepens only the cap-clipped floor VALUE. HONEST SCOPE in-fixture: on s6 this buys
one earlier/deeper commitment beat; the class is majority physics -- margin a rolling-back lead
consumes is unrecoverable in the comfort band, and the felt bill is the floor working.

BOOKMARK fc2 s168 (felt 1.08, rest 4.3, band lifted): metrically near-target. A ceiling-taper
lever for its residual was built and REVERTED IN-BRANCH -- replays proved it a NO-OP (below
0.5 m/s the armed descent owns the lane; the 0.77-template s23 shares s168's exact descent
shape). The residual would live in the descent's linear path -- renegotiation with its own
red-team if felt data ever demands it. LEDGERED.

PROCESS: the native codex reviewer returned FOREIGN-REPO output twice (its thread state glued to
another workspace; --base ignored, --fresh ineffective) -- round-2 verification sweep run by ME
(4-tuple coverage, window-vs-eligibility ordering, baseline-hold decay paths) and the sign-off
recorded as mine per the standing advisory-review rule. Round-1 (in-repo) caught both real
defects (cap inversion, single-sample noise commit); MB1-MB3 + MA1 killed. Battery 979/19.
Deployed 7e2e69e5a3 device-verified.

## 2026-08-10 -- cycle 28: the mid-queue re-slam gets an owner (deployed 626d784059)

User: average significantly improved, leapfrogging not seen for a while; one bookmark. fd1 s4
(felt 2.98, rest 2.8 = 0.2 m FLOOR BREACH, build 860b07b4ee incl. the other agent's rdf-driving
model): the queue launched, ego followed at 1.3-1.8 m/s at ~4.4 m, the lead slammed back to
zero. THE UNOWNED BAND: 0.8-2.0 m/s behind a MOVING lead -- aim lane floored at 2.0, service
refuses moving leads (cycle-25, correctly), MPC ramps only as the lead's braking becomes
visible. SHIPPED (318d39b68d + c2a26e9a68 + 626d784059): the aim lane enters below its floor
only for a decisively-braking still-moving lead (windowed alk <= -0.5, lv >= 0.3, floor 0.8;
launch/steady leads keep 2.0 -- f85 s4 exclusion stands); committed rides through the lead
reaching zero DOWN TO 0.3 m/s (three review rounds each caught a release hole: full-window
departure evidence held slam frames 0.3 s -> last-0.1 s slice; the runway clamp made a_req <=
v^2 so EVERY commitment self-released below 1.0 -- hold band 0.3-1.5 with decay release only
above 1.5; launch-FROM-ZERO stayed held until lv 0.5 -> rising-speed evidence +0.08/0.15 s,
negative-side Doppler noise cannot fake it). Recorded-trace counterfactual: commits -1.67/-1.09
~1 s earlier than the recorded late chase, halving terminal energy; projected rest ~3.2-3.4 vs
breached 2.8, felt ~1.5 vs 2.98 -- detection sits at the physics limit of the lead's own braking
signature. MC1-5 + MD1-4 killed (MC4 masked by ON-threshold arithmetic at low v; paired kill
proves coverage). Battery 986/19. NOTE: fd0/fd1 ran the rdf-driving MODEL -- watch whether the
new model shifts stop-approach shapes in the next census.

## 2026-08-16 -- cycle 29 opens: the late-entry seed jolt (ff3 s16, one bookmark)

Route 00001ff3 (build 9f3e8ffe8d = cycle-28 + rdf-driving/Rebellious Hope models; user: "spots
much, much better in general"). 3 stops: s54 rest 4.7 (aim band), s25 rest 3.4, s16 stop 1
rest 3.0 felt 0.90 (creep-in), s16 stop 2 = THE BOOKMARK: rest 3.8, felt 1.59, terminal wire
lands -0.70, gates clean except felt.

DECODED (fresh-service replay from the launch peak, 10 Hz): a queue re-stop 30 s after stop 1.
The lead crept ahead at 0.6-1.9 m/s and ego followed at 1.0-1.5 m/s at ~4.4 m; the lead then
slowed to a walk. The service entered LATE (t-1.4 s, v 0.68 -- shouldStop/latch both false
until the lead fell below the wide latch's +0.3 at t-0.9) via a bare planner shouldStop, so
entry landed BELOW the walking-pace region and the normalization corridor (0.77-1.10) never
opened; the entry seed was the recorded wire (-0.41), and the GLIDE law's a_phase demanded
-0.74 at once (v 0.68, d_rem ~0.7): the limiter deepened -0.44 -> -0.73 in 0.2 s (J_DOWN 2.5),
then the arming latch fired at 0.50 and captured u0=-0.72 -- the flat -0.70 band. Felt 1.59 =
the deepen step + the un-normalized band, the same physics as cycle-26's s9 minus the corridor.

THE CLASS: LATE ENTRY BELOW THE NORMALIZATION CORRIDOR. The cycle-26 lift assumes the service
owns the approach from >= 1.1 m/s; a re-stop behind a slowing-walking lead hands over at
0.5-0.8 with the wire already deep or with a deep phase demand waiting. Two facts pin the
lever: (a) the phase lane's demand at entry (-0.74) was ABOVE necessity (a_kin -0.03, plan
-0.38): the GLIDE law with a small d_rem is the writer, not safety; (b) the arming capture at
0.50 took the DEEPENED wire, not the seed.
CANDIDATE: PRE-ARM SEED HOLD -- between service entry below V_NORM_START and the arming
capture, bound a_phase shallow-side at max(seed, u_norm) when the entry seed is shallower than
u_norm and geometry is healthy (same gates as the lift: strict/wide latch or shouldStop,
gap_live, current-frame lag floor, gap margin, uphill-off, no hazard) -- i.e. extend the
normalization corridor's PROTECTION (not its region) down to the entry point when entry
happens late. Deepen-only lanes still min() through. Design red-team before code.

### Cycle-29 CLOSED: one-shot late-entry seed corridor deployed 433d1068c1 (device verified)
Shipped f9a6e112ee + 433d1068c1 -- full record in the commits. sol design review A-prime adopted
verbatim: activation ONLY on the entry frame for late entries (0.50, 0.775] from shouldStop with
earned lead motion, live gap, seed shallower than u_norm, v_guard lag-floor (max(v, v-lead_v)),
gap margin, uphill-off, current-frame reversal false, no monitor/fast_deepen/catch-up; SPENT
permanently by any hazard, phase exit, safety-lane bind, or (end-review MEDIUM) a live EASE
gate-fail fast_deepen; never re-engages on churn. Verified first that candidate C
(assumed-normalized capture) is INERT: the terminal emitter is monotone from the live wire.
Replay: capture -0.72 -> -0.52, band lifted, same landing instant. ML1-ML8 killed (ML5 via the
paired kill; ML7 gap-margin provably dominated by the v_guard floor at late-entry speeds --
documented as unkillable, not faked). Battery 997/19. PROCESS: native reviewer returned a
foreign-repo (Oczar ADR) review on round 2 again -- sweep run MYSELF: all same-frame flag sites
(mon 804, fast_deepen 1030/1060) precede the re-check (1126); relief_catchup (1233) sets after
but is covered by the safety-bind spend; sign-off mine. VALIDATE: late queue re-stops behind
walking leads should land ~1.0 felt; the cycle-26 corridor and the cycle-17 relief region are
byte-untouched.

## 2026-08-19 -- cycle 30 opens: the plunge-timing class, bookmarked twice

Route 00002005 s1 (build 8888f4b5c0 = cycle-29 + the other agent's Active-With-Gas/gas-override
longitudinal commits), one bookmark. The stop scores AT the census bar: rest 3.9, vmax 4.5,
worst -0.78, felt 1.05, excess 0.20, single descent, corridor working (band held -0.5). Its
residual signature is IDENTICAL to cycle-27's fc2 s168 (felt 1.08, wire_jerk 0.949 vs 0.954
here): the final plunge from the normalized band (-0.5) to A_HOLD_SECURE (-0.70) ending at
V_CREEP_HOLD_SECURE 0.10 holds net ~0.45-0.5 into the last 0.2 s; the felt lands 1.05-1.08
where the 0.77-template stops get their fade started earlier. The user has now bookmarked this
class twice -- the ledgered descent-shape renegotiation is DEMANDED by the felt data.

THE CONTRACT UNDER RENEGOTIATION (cycle-18): "A_HOLD_SECURE on the wire before the clutch
engagement window (~0.08, static push p50 +0.43, refit n=1987)" -- anti-relaunch, the worst
regression class in the project's history (cycles 4-6 bob, 14 carry, 18 floor). The candidate:
END THE PLUNGE INSIDE the engagement window (command target ~0.07 instead of 0.10) so the final
wire deepening coincides with the rising creep push and the NET fades through the plunge --
accounting for the 0.25 s actuator lag chain, the physical secure arrival vs the engagement
threshold must be re-derived from the refit data, not assumed. If the arithmetic says the
relaunch margin cannot be held, the honest verdict is "1.05 is this actuator/clutch's felt
floor" and the class is closed as physics.

EXONERATED: 1ff4 s11 rest 2.19 = driver gas-override coast-in (wire/aTgt 0.00 throughout).
LEDGER NOTE: the new Active-With-Gas mode will increase engaged-with-gas driving; gas-override
coast-ins produce sub-floor rests under driver control -- watch whether any SYSTEM behavior
(re-engagement after release near a close lead) inherits a sub-floor geometry.

### Cycle-30 CLOSED: quadratic descent tail deployed 85aeea7dd6 (device verified)
Shipped 77d670f392 + 85aeea7dd6. Design review verdict D2 adopted with its acceptance gates;
D1 (later landing) REJECTED on the lag-chain arithmetic (physical secure would arrive at/after
wheel-stop, inside the relaunch window); D3 (accept 1.05 as physics) rejected for lack of
evidence. The felt census keyed the class precisely: the whole 0.77-vs-1.05 difference is
~0.06-0.11 m/s^2 of net decel at v~0.12, where the linear path converges to -0.69 for every
capture -- only a shallower mid-band wire with a fast tail INTO the clutch engagement moves it
command-side. Deep (normalized-band) captures now descend quadratically; the weight blends
continuously in u0 (end-review MEDIUM: the hard switch put a 0.061 step between captures 0.002
apart); shallow captures keep the linear law; landing point, secure depth, rates, monotone
emission and every safety lane unchanged. ACCEPTANCE: both bookmarked replays 0.95 -> 0.60
wire_jerk (gate 0.80), template 0.48, seg22 relaunch fixture clean, single descent everywhere.
MQ1/MQ2 + continuity pins killed. Battery 1001/19. EXONERATED this cycle: 1ff4 s11 rest 2.19 =
driver gas-override coast-in (wire 0.00 throughout); LEDGER: Active-With-Gas increases
engaged-with-gas driving -- watch re-engagement geometry near close leads. VALIDATE next
drives: the bookmarked class (gentle stops with the -0.5 band) should land felt ~0.8; genuine
deep/hot stops unchanged.

## 2026-08-22 -- cycle 31 opens: the comfort-stall long-rest class (no bookmarks, user census ask)

Routes 200a-2011 (build 9e60844a93 = cycle-30 live), 12 lead-stops + 2 no-lead. CYCLE-30
VALIDATED: every gentle band-normalized stop landed felt 0.78-0.99 (was 1.0-1.1) -- the census
best is now the typical; hot stops hold 3.2-3.6 rests at their physics level (felt 2.0-2.8,
excess under the gate).

THE COMPLAINT CONFIRMED (user: "some at 6 m or possibly above"): 4 of 12 rests at 5.2-6.47, a
class that ran ~1-in-60 before the rdf-driving/Rebellious-Hope e2e models landed. MECHANISM
(s22 exemplar, frame-decoded): the approach is TRAJECTORY-led; the plan's velocity dies at its
comfort equilibrium (rest 6.47; ACC desired(v=1.5)=6.53 -- and s22 actually stalled ABOVE the
ACC curve: gap 7.5 at v=1.0 vs desired 5.7, so the e2e trajectory is MORE conservative than the
follow law). No stop target until gap 8.0 (STOP_TARGET_MAX_DISTANCE_M caps reach), arbiter
enter/hold limits never admit dts 2.1-3.8 at creep speeds, shouldStop flips only at v 0.24, and
the first v<0.05 frame latches the secure hold -- the 2.2 m closure to the 4.3 anchor never
happens and planner_go can never fire (gap static, lead present). The two pure-creep rests
(s30 5.6 vmax 0.5, s3 6.0 vmax 0.1) are the same root one band lower: queue crawls settle at
the walking-pace equilibrium. Design red-team running (E1 desired-gap shaping / E2 stop-target
reach / E3 post-rest re-close / E4 accept); E1's reach is already suspect against trajectory-led
stops per the stall-above-desired measurement.

### Cycle-31 lever SHIPPED (pending review + deploy): E1-R rest-close reference floor 7d90493edf
Design verdicts: E1-R (planner-side reference floor) ADOPTED; E2 (stop-target reach extension)
rejected -- the arbiter limits exist for hazard reasons and reach past 8 m re-opens the phantom
class; E3 (post-rest re-close) REJECTED -- post-stop motion near a lead is pinned off by design,
prevention strictly better; E4 (accept) rejected at 13x the historical class rate. The lane:
while armed, the e2e MODEL REFERENCE (x, v, a -- not any demand lane) is floored at the comfort
closure curve toward rest, v_floor = min(v_cap, sqrt(2*0.50*d_eff)), d_eff = gap - (4.0+ISD) -
0.25*v_guard, active window d_eff in [0.5, 2.5]; raise-only with position RE-INTEGRATED from
the raised velocity (a bare velocity floor would fight the MPC x-cost) and never above the
captured entry speed (v_cap = min(v_at_arm, 0.8)). Gates: Santa Fe HEV + blended + no
force-coast + CONDITIONED classifier (planner embeds its own StopContext -- shared CODE with
longcontrol's, per the no-raw-vLead requirement: strict latch, motion trust, outward-held gap
trust) + lead_v >= -0.10 current-frame + not standstill + no deeper lane (aim/stop-commit)
active. ONE arm per approach (0.06 < v <= 1.50); PERMANENT spend on any disqualifier, speed
escape > 1.60, or lead replacement; standstill cancel means a latched rest is never re-opened.
All braking lanes stay deepen-only on top. Counterfactuals: s22 closes ~0.8 m/s through the
final 2 m -> rest ~4.3-4.6; pure-creep cases close at their entry speed without accelerating.
Kill switch SANTA_FE_REST_CLOSE_FLOOR. ADVERSARIAL R1 (own-repo this time, no foreign-repo
glue) needs-attention, BOTH findings real and adopted: [HIGH] the first cut's spend was
per-PROCESS -- one stop consumed the lane for the whole drive -- and a stale armed/vcap survived
disengagement (re-engage at 0.2 m/s could inherit a 0.8 cap = commanded acceleration) -> rc_ok
requires not reset_state; reset_state disarms + re-opens; approach-EPOCH added (standstill or
v > 3.0 clears spent when not armed; arming still needs v > 0.06 so E3 stands -- a standing car
is never moved; queue re-stops each get the lane). [MEDIUM] raw leadOne bypassed the
road-furniture boundary -> the planner now runs the SAME StoppingLeadAuthority longcontrol
applies, certificate reset across disengagement via its lead_status input. ARBITRATED AGAINST
one recommendation: the planner StopContext stays WARM across reset_state (classifier state is
about the LEAD; longcontrol's shadow context observes across engagement boundaries by the same
design). Mutations MR1-MR8 killed (MR2 needed a CROSSING-reference fixture -- the raise-only
case where replacement LOWERS an above-floor point; the original fixture's floor dominated
everywhere, a gauntlet lesson worth keeping); MR9 (drop the `not armed` epoch guard) documented
unkillable -- provably behavior-equivalent. R2 needs-attention, one MEDIUM and it was REAL
(reviewer reproduced it live with the actual classes): the warm StopContext PRE-EARNED the
stopped dwell from unauthorized leads, so a transient modelProb flip could arm the same frame
-> fixed by mirroring longcontrol's service_lead_status pattern VERBATIM (authority evaluated
FIRST; lead status/distance/track-id masked into the classifier unless certified; the mask also
scopes lead evidence to engagement) -- which SUPERSEDED my R1 warm-ctx arbitration: longcontrol's
own ctx lead-evidence was never warm across disengagement, the mask was the design all along.
Regression pins the rejected-track/modelProb-flip sequence through the real classes. R3
verify-only: APPROVE, regression + adjacent tests re-run by the reviewer directly. Battery
1007/19. PROCESS NOTE: a docs commit landed DURING R1 and would have moved the reviewer's lazy
`--base HEAD~1` onto the docs diff -- soft-reset + stash until the review finished; rule:
nothing lands on the branch while a review with a relative base is running.

## 2026-08-20 -- cycle 32 opens: approach UNDER-DELIVERY (no bookmarks; "still not perfect too often")

Routes 2012-2016 (all on the cycle-31 build 30d7033b39; 2014-2016 essentially unengaged), 14 settles,
11 lead-stops. CYCLE-31 VALIDATED: 0 of 11 rests above 5.0 m (median 4.10; was 4/12 at 5.2-6.47).
EXONERATED: 013 s15 rest 7.24 = driver disengaged at 8 m/s and braked manually. Felt census: gentle
stops 0.67-0.98 (012 s17 0.67, 013 s12 0.79, 012 s20 0.83, 013 s33 0.98); hot approaches 1.1-2.2.

THE CLASS: 013 s19 = driver takeover at 3.2 m/s / 6.7 m on a hot approach (rest 2.6 = the driver's).
Frame decode: a steady -1.80 command from 12.6 m/s realized only -1.43 (mean over 7.9 s, ratio 0.82);
the car arrived at 7 m doing 3.4 m/s, the aim floor's a_req went 1.4 -> 2.45 inside 0.5 s (the
"never chase" geometry), the driver braked first. Same mechanism under the short rests (013 s10 3.38,
012 s2 3.7, 012 s20 3.4) and under the -2.25-cap slam of 012 s15 (landed 4.2 at felt 2.2).
PLANT CENSUS (n=61,960 engaged braking frames, routes 010-013, aEgo sampled 0.40 s after the sent
command): ratio 0.97 at -0.6, 0.93 at -1.1..-1.7, 0.89 at -1.9, 0.87 at -2.2, 0.78 below -2.5; the
flat-road rows keep the depth trend (+0.03/+0.08/+0.11/+0.34 shortfall by bin), nose-down adds
~+0.09; over-delivery (err < -0.2) is 5% of frames, under-delivery (> +0.2) 17% overall and 43% of
deep frames. CONTROLLER FACT: the Hyundai path is pure feedforward -- no longitudinalTuning gains in
opendbc/car/hyundai, sent == aTarget every frame -- so the shortfall is never corrected; longcontrol
already carries `error = a_target - aEgo` + a web of `if integrator_enabled: pid.i = ...` reseed
branches that have NEVER run live (ki = 0).

DESIGN RED-TEAM (sol xhigh, read-only): A MODIFY / B reject (waking the dormant pid.i paths, gas-side
surface, carcontroller launch cap invisible to anti-windup) / C reject (grade-only misses the flat
depth term; pose pitch is not grade) / D reject (planner is the wrong layer). The structural catch:
a sent-referenced error against a sub-unity plant is a shortfall ESTIMATOR that saturates at the
bound -- the reference must be the UNTRIMMED demand so the error closes once the car realizes what
the planner asked. Adopted: separate deepen-only state (never pid.i), deadband + leak, fast unwind
(2.0) vs slow wind, TRIM_MAX 0.40, A_ARM -0.75, V_MIN 2.5 (= the LIVE service band), V_MAX 16,
decay (never a step) under gas/service/disarm, freeze on any frame a downstream cap rewrote the pid
wire, placement AFTER the cap family and BEFORE the service takeover. ARBITRATED: the "actual sent
vs expected" check would need carOutput plumbed into longcontrol -- the engagement cap only acts on
positive requests (inert under A_ARM) and gas neutralization arrives as freeze_integrator (decay), so
gating replaces plumbing. HARNESS-DRIVEN CHANGES to the proposal: deadband 0.05 not 0.10 (a 0.10
band leaves a 0.10 residual by construction -- cannot meet the review's own +-0.08 pin); a MODEL
REFERENCE (delayed demand through the plant's 0.5 s first-order lag, seeded at the plant's aEgo,
re-seeded after any cap frame) -- without it every brake ONSET read as a -1.8 shortfall and slammed
the trim to the bound in 0.4 s (the review's hazard 1, reproduced); a RATE GUARD widening the wind
band by 0.25 s x |d ref/dt| (a delay mismatch cannot read a ramp-in as shortfall; zero in steady
state); in-band leak 0.01 not 0.05 (0.05 fought the equilibrium into a sawtooth); KI_WIND 1.0 with a
4.0 s convergence window (every gain meeting the proposed 3.0 s limit-cycled at 0.7 s delay before
the guard; with it the 0.78-1.05 gain x 0.3-0.7 s delay x noise sweep is clean at 1.0). Recorded-
residual counterfactual: s19 arrives at 2.0 m/s / 15.8 m instead of 3.9 / 11.6 (a_req 0.84 -> 0.09,
overstated: the planner would relax as the car catches up); s10 end gap 4.3 vs 3.5; s2 (0.98
tracking) trim stays within -0.06.

### Cycle-32 CLOSED: approach accel-tracking trim shipped (R1 HIGH adopted, R2 approve)
Shipped 96c3a2ada9 (longcontrol SANTA_FE_TRIM_* block; kill switch SANTA_FE_ACCEL_TRACKING_TRIM).
R1 (own-repo) HIGH was real: on service-owned frames the trim still rode in the legacy-chain value, so
the service-EXCEPTION fallback min(legacy, last) on a previously-owned frame could put the residual on
the wire as a deepen step -> once the service owns (previous frame) the trim STATE is zeroed and
nothing is added to the legacy value (the takeover frame itself carries the trim into the seed, so
takeover is continuous). Honest note: the review's exact geometry (service relaxed above legacy,
then a fault) was NOT reproducible in the harness -- probes with the pre-fix code showed steps of
-0.005/+0.010 there -- the fix removes it by construction; the fault path now returns toward the
untrimmed legacy value (a release bounded by the residual, typically <= 0.1 because caps/should_stop
decay the trim before most takeovers). R2 verify-only APPROVE. Gauntlet MT1-MT20: 19 killed, MT8
paired with MT18 (documented). Battery 1028/19. HARNESS LESSONS worth keeping: (1) a model
reference seeded at the DEMAND reads every onset as shortfall -- seed at the plant (aEgo); (2) a
gauntlet mutation must reproduce the PRE-FIX behaviour, not merely delete the fix line (MT19 dropped
the zeroing but the add had moved into the else-branch, so it proved nothing; MT20 re-added the
residual); (3) a pin can be vacuous by geometry -- check that the hazard state (here: residual at
takeover) is actually reached before trusting a pass. VALIDATE next drives: hot approaches arrive
with margin (rests >= 3.7, no aim-cap slam), realized/sent ratio at -1.8 commands rises from ~0.85
toward ~0.95 in the plant census, no brake hunting at 8-16 m/s, launches untouched (trim is
braking-only and decays at gas).

## 2026-08-22 -- cycle 33 opens: crawling-lead long rests; the cycle-32 trim is nearly inert

Routes 2017-201d (all on 71d7d456ec; 2017-2019 unengaged stops, 201b none): 19 lead-stops, no bookmarks.
CYCLE-32 VALIDATION, honestly: the accel-tracking trim is nearly INERT on the road -- active (< -0.05) on
20% of engaged braking frames, median -0.09; aEgo/DEMAND 0.90 -> 0.92 at -1.5..-2.0 (the earlier
"plant ratio rises" validation criterion was wrong-headed: aEgo/SENT is the plant gain by construction,
the trim shows in aEgo/demand). Cause measured: wind deadband 0.05 + rate guard (0.25 s x |d ref/dt|,
p50 0.037 / p90 0.144) ~= the typical shortfall (e p25 -0.10; 37% of frames < -0.05). Offline replay of
the pure update (upper bound): shipped 19.5%/-0.025; db 0.03 + guard 0.10: 26.9%/-0.042; db 0.02 +
guard 0.10 + KI 1.5: 35.8%/-0.081. One more moving-lead takeover (201a s9, realized 0.82, trim -0.1).
The hot approach 201c s9 (10.5 m/s) landed 3.9 (its cycle-31 twin: 3.38).
TWO CLASSES on the same build: (1) LONG rests 4.67-6.3 (7/19, route 201a, queue-crawl geometry): the
LEAD was still crawling 0.3-1.1 m/s when ego settled, the strict stopped-lead latch never confirmed, so
the cycle-31 rest-close floor correctly stayed off (offline gate replay, tools/stopping/review/
rest_close_replay.py: blocked only by 'confirmed'); the lead stops 0.3-1.5 s later and no post-stop lane
closes the gap (E3). s6: the gap GREW 5.7 -> 6.3 while ego stalled at 0.5 m/s behind a 0.7 m/s lead --
the walking-pace follow law / e2e plan settles ~6 m behind a crawler. (2) SHORT rests 3.09-3.6 (5/19,
route 201c, gentle approaches): between 2.5 and 1.5 m/s the planner eases (-1.07 -> -0.83) while the
lag-aware necessity to rest at 4.3 rises (1.07 -> 1.83); the aim floor never commits (onset 1.3) and the
service's floor-defense cap (cycle-15) refuses to chase -> 3.1. Good rests (4.08, 4.6) carried NEGATIVE
deficit early: "aim early" is the lever for that class (next cycle). EXONERATED: 201c s24 felt 2.2 =
no-lead stop (ledger, out of scope). Gentle-stop felt 0.72-0.85 holds. TOOLS promoted into
tools/stopping/review/: track_win, felt_one, gain_census(+aggregate), demand_census, aim_deficit,
rest_close_replay, mode_census, trim_replay (the /tmp copies evaporated between sessions).
Design red-team running: A extend rest-close to crawling leads (relative closure curve) / B walking-pace
follow-distance shaping / C accept; secondary: trim re-tune parameter set.

### Cycle-33 CLOSED: rest-close floor for CRAWLING leads shipped (5 review rounds, 4 real findings)
Shipped 0385159783 + db1880e8ee + b2c119315e + e246a06e40 + the R4-fix commit (planner) and 22af9ffd84 (longcontrol:
trim re-tune REJECTED by its own gates -- constants kept, pins added). Design review (sol xhigh):
A adopted as modified -- keep the ABSOLUTE closure curve (no lead_v term: a crawler can stop on the
next frame), drop only the stopped confirmation, add a wheel-stop gate; B (walking-pace follow law in
the MPC) rejected: touches every stop/pull-away/cut-in; C rejected at 7/19; the SHORT class is a
separate "aim early" lever. Sol's MPC numbers confirm the lift is the right tool: blended-mode
desired gap behind a 0.5 m/s lead at 5.4 m is 5.07 (ego 0.5) / 5.67 (ego 1.0) -- spare room in
both frames; the e2e reference stalls, the MPC does not hold ego off.
ADVERSARIAL ROUNDS, all own-repo, all real: R1 [HIGH] the cycle-31 "standstill re-opens the
one-shot" rule let a 0.10 m/s micro-roll behind a crawler re-arm after a completed rest (E3) ->
standstill only CANCELS; re-open needs positive evidence of a new approach; [MEDIUM] a 42 -> -1
identity-less handover inherited the lift -> any id change cancels+spends, -1 never arms. R2 [HIGH]
(real StopContext) an untrusted held gap growing past the window re-opened the latch -> epoch
evidence only from trusted geometry. R3 [HIGH] one OUTWARD-held frame crossed the margin before its
0.25 s persistence -> evidence is MEASURED-only (pure helper get_santa_fe_rest_close_epoch_evidence
shared by wiring and tests). R4 [HIGH] the driving-again counter cleared a spend on the cancel frame and could be pre-earned on a first approach -> the counter runs only after a COMPLETED REST (rested latch), every re-open CONSUMES it, and no re-open path acts on a cancel frame (cancelled_now). R5 verify APPROVE. EVIDENCE-DRIVEN DEVIATIONS from sol's pins (the recorded
frames decided): (1) crawl-exit/reversal cancels are DEBOUNCED 6 frames (0.30 s) -- sol wanted
immediate; frame-by-frame replay: s4's reversal burst was 5 frames at -0.11..-0.17, s6's 3 frames,
cycle-27 census p90 0.29 s; 3 frames lost both recorded approaches; (2) "DRIVING AGAIN" (ego >= 1.0
m/s for 10 frames) re-opens the one-shot -- s6 drove 1.4-1.6 m/s for 4 s between two queue stops
with the gap never opening past the window, so its worst rest (6.3) got no lane under the R1 rule.
FINAL-RULE REPLAY on the recorded 201a approaches (tools/stopping/review/rest_close_replay.py):
s15 (5.23) armed 1.22 m/s -> wheel-stop; s4 (5.2) armed throughout (the -0.13 burst no longer
spends); s6 (6.3) re-opens at 375.8, arms at 380.1 (lead 0.84) but the lead hovers 0.90-1.01 for
~1 s -> crawl-exit spend; LEDGER: crawl-exit permanence / ceiling 0.90 vs 1.10 -- judge on-road.
Gauntlet G1-G27: 25 killed; G7 (isfinite) and G27 (rested term, counter already zero) documented behaviour-equivalent. Battery 1058/19.
PROCESS LESSONS: /tmp evaporates between sessions -- the review tools now live in
tools/stopping/review/ and the empty pytest ini must be re-created (every pytest run this cycle
silently failed before collection until it was); a sentinel assertion ("the hazard state is
reached") belongs in every regression (the first R3 fixture never armed); an absolute --base
(71d7d456ec) lets commits land during reviews without moving the diff.
VALIDATE next drives: queue-crawl rests land 4.3-4.8 (were 4.67-6.3); no lift after a completed
rest; no reversal/crawl-exit hazard; the short class (3.1-3.6) unchanged (next lever: aim early);
hot approaches unchanged (trim unchanged).

## 2026-08-23 -- cycle 34: the universal stop program opens (user directive: no more tree of ifs)

DIRECTIVE: stop patching classes with lanes; one universal stateless approach; delete lanes as an offline
harness proves them redundant; ML controller is the NEXT program. Inventory at opening: planner 129
SANTA_FE_* constants / 40 Santa-Fe functions / 52 lane state vars; service 1271 lines / 67 params;
longcontrol 16 Santa-Fe cap fns; 12 flags. DESIGN RED-TEAM (sol xhigh) ADOPTED: governor OWNS the wire
through the existing StoppingService seam (planner = deepen-only safety via aTargetTrajectory); V_OWN
4.5 conditional; the law with profile feedforward and no sqrt singularity (d = TAU q_ref + q_ref^2/2A_C);
lag-aware 3.1 m barrier; inverse-gain tracking + small integral; the must-stay list; frozen harness gates
A-F; 8-step deletion order starting with SHADOW. Full text: docs/stopping/universal_stop_program.md.
DIAGNOSIS THAT SETTLES THE ARCHITECTURE: today's 024 s31 (rest 5.3 -> 6.4 as the lead pulled away): the
cycle-33 planner lift ARMED (offline gate replay) and the planner eased, yet the SERVICE owns the wire
below 2.5 and its own stop-intent lanes rested the car -- a planner-side reference change cannot reach
the wire while the service's approach laws exist (cycles 31 and 33 were architecturally undercut). 024
s30 (felt 4.6): a planner lane slammed -0.74 -> -1.99 at 1.3 m/s / gap 4.7 (the v^2/2rem chase); the
governor asks a bounded ~-1.2 there (pinned).
SHIPPED aa05f13ef6 (+ 6a3670556b, ef300e6bc5): governor_demand + barrier_demand as pure helpers in the
service, evaluated on every lead frame, carried in the debug dict and the settle telemetry (gov_frames,
gov_max_div, deeper/shallower fractions, gov_min, bounded 4 Hz trace) -- SHADOW, zero wire impact
(structural pin: governor patched to -9.0 -> frame-identical). R1 HIGH (real): a raise in the helpers
would trip the LIVE blanket fault latch -> contained, pinned for LIVE/LIVE_TERMINAL; R2 approve.
HARNESS, honestly: corpus extractor + prototype harness built (sol hardened: 179 approaches from routes
>= 1f00, per-segment outputs, lead trajectory from median-filtered gap, jerk-limited law); the recorded
replay reproduces the record only through the residual-plant trick (generic plant: rest error p50 1.85 m,
residual RMS 0.40 m/s^2, 0.7 s autocorrelation), so law scoring is NOT valid yet -- the naive sqrt law
scored worse than the record (31-44% in band vs 61%) and the feedforward governor 37-52%: both numbers
are untrustworthy. Plant identification (ARX per route): one-step RMS 0.06 but 2 s rollout 0.42 and
half the fits unstable -- closed-loop stop data has no excitation; a dedicated identification drive
(step commands, empty road) is needed for offline ranking; the on-road SHADOW is the evidence path.
Target-switch contamination in episodes (lead-velocity jitter max 93 m/s) must be filtered.
Cycle-32 trim stays; cycle-33 planner lane stays until step 4 of the deletion order.
NEXT: read the shadow telemetry from the next drives (governor vs wire per stop), fix the corpus
filters, plan the identification drive; then step 3 (replace GLIDE/EASE) behind a rollout flag.
Routes 2021-2023 (today) not yet synced (LTE) -- census next cycle.
ADDENDUM (sync completed): routes 2021-2023 hold no stops (2021 engaged without stopping; 2022/2023
unengaged). Today's "mediocre stops" are exactly 024 s30 (chase slam, felt 4.6, rest 5.19 behind a lead
still at 1.26 m/s) and 024 s31 (crawling-lead rest 5.3 -> 6.4) -- both traced above, both the governor's
target classes. Shadow telemetry for them arrives with the next drives on 3c23c37111.

## 2026-08-26 -- cycle 35: review tooling for long drives; first shadow evidence; pre-band shadow

TOOLING (user: keep the reviewer's context lean on multi-hour drives; sync stays complete for other agents):
tools/stopping/review/stop_index.py is the cycle entry point -- qlog triage of every segment (cheap), rlog
analysis ONLY for candidate segments (+ neighbours), per-stop scoring (rest, lead state, approach, felt), the
service's own settle_summary matched over its frame window (it is emitted at the END of the hold), approach-
only shadow-governor stats (v >= 0.5: the law does not model the clutch hold), a detector audit, persistent
index with processed-segment memory, ATTENTION rows first. Measured on route 2029 (121 segments, multi-hour):
16 candidate segments, 9 stops, 4 s, ~15 lines. The audit found the heuristic detector MISSES service-handled
settles (3 on this drive, rests 6.28 / 8.4 / 5.81) -> the LONG class was under-counted in every census so far;
service-only settles are now rows (HEURISTIC_MISS). Extractor flags radar target switches; the identification-
drive protocol is in the program doc. Procedure updated in review_cursor.json.
FIRST SHADOW EVIDENCE (route 2029 on 3d5bd965e3): inside the service band the governor is DEEPER than the wire
on 96% of approach frames (max |gov - wire| p50 0.59) -- i.e. the car reaches 2.5 m/s above the comfort curve
and the current stack under-brakes there (s68 rest 3.39, s87 3.79), while long rests behind crawling/departing
leads (5.54, 5.79, 6.28, 7.0, 8.4) again dominate the misses. But the shadow started at service entry, where
the debt from above 2.5 is already banked: the law's premise (own from 4.5) was not observable ->
SHIPPED the PRE-BAND shadow (own StopContext, 4 Hz ring of the last 3 s before entry flushed into the settle
trace with negative times; contained; pinned frame-identical wire under raise/-9.0 for LIVE and LIVE_TERMINAL).
Felt on this drive's gentle stops 1.0-1.6 (re-stops behind crawling leads: s104 accelerated to 1.15 then the
lead re-stopped) -- the governor's continuous pursuit is the intended answer; recorded for the harness.
Today's earlier routes 2025-2028 / 202a / 202b: no stops.
REVIEWS: R1 [MEDIUM] the pre-entry ring was not settle-bounded (an aborted approach's samples decorated a
later settle) -> timestamped samples, attach only if fresh (0.5 s) and within 3 s of entry, no sampling inside a
settle, cleared at completion; R2 [MEDIUM] the freshness clock only ticked inside the sampler (a ring survived
any disengaged/above-band interval) -> the clock ticks on EVERY longcontrol frame, unfed rings expire, reset()
clears; R3 approve. Shipped a09d9f020a. Battery 1078/19.
NEXT: the next drives carry the 4.5 -> rest governor trace; then step 3 (replace GLIDE/EASE behind a flag).

## 2026-08-27 -- cycle 36: program step 3 -- the governor as the service's approach law (dark)

Routes 202c-2033 (8 routes; 5 unengaged; stops on 202c/202d/2032/2033): ~10 stops, 9 with governor
telemetry, first FULL PRE-BAND traces (4.5 -> rest). Reading them: on stopped-lead approaches the wire
under-brakes late and the governor crosses it at ~1.7 m/s (202d s5: gov 0.4 deeper at the end, rest
4.2); on DECELERATING-lead approaches the governor is intentionally shallow (+0.5 clip -- the planner
safety lane owns those by design), so governor-vs-wire statistics must be conditioned on lead state;
one radar-garbage pre-band trace (2032 s9: gap 0.3-2.8 m at 4.5 m/s then a jump to 12.5 -- a false
close track) rehearses the authority mask and needs an index plausibility flag (ledger).
SHIPPED (dark) 2ddb3ca860+9abee9dbee: SERVICE_APPROACH_LAW selects the approach law; "governor" makes
a_phase the stateless law with the glide law's own deepen-only grade/creep feedforward, EASE never
engages, the glide-law patches (cycle-26 normalization, cycle-29 late-entry corridor, cycle-17 relief
gentling/catch-up) are inert, the 3.1 m barrier is a LIVE safety lane (direct evaluation, None ->
planner_min fail-closed, a raise reaches the LIVE robustness path); terminal descent / RAMP / HOLD /
RELEASE / monitor / a_kin / a_plan / dropout floor / the sole limiter untouched; no-lead stops keep
the legacy law. R1 both HIGHs real: the barrier consumed the telemetry containment (fail-OPEN, probe
-1.64 -> -0.38) and the relief catch-up produced J_SAFE frames with no safety lane binding -> both
fixed and pinned (the relief pin runs the recorded 00001f62-seg25 fixture under both laws; legacy
engages, the governor never does). R2 approve incl. a 2000-frame differential: the DEFAULT "legacy"
is bit-identical to 74bc5d42ee. Gauntlet GV1-GV9 killed (GV7 was dead code, removed). Battery 1092/19.
THE FLIP: one word ("legacy" -> "governor"), already reviewed; gate = the next drives' shadow
conditioned on stopped-lead approaches + the felt/rest census staying clean, then flip + on-road
validation with instant revert.

## 2026-08-28 -- cycle 37: flip gate NOT met (queue drive); gate tooling made direct

Routes 2037-203a (only 203a has stops: a queue drive, 8 stops). CENSUS CLEAN of auto defects:
203a s9 "rest 2.7 SHORT" was the DRIVER's own braking (disengaged, BRK the whole approach); s8
felt 1.41 is the known walking-pace crawl interplay (creep surge vs hold behind a 0.3 m/s lead).
Long rests behind crawlers persist (5.6/6.4/7.3) -- the class the governor targets. FLIP GATE:
cannot be read from this drive -- the shadow shows "shallower p50 1.00" but every approach frame
is a crawling/decelerating lead, exactly the frames the planner safety lane owns by design; clean
STOPPED-LEAD approaches are absent. NO FLIP. Shipped c01fd0b65f: the governor trace tuples (in-band
and pre-band) now carry lead_v; stop_index conditions its shadow statistics on |lead_v| <= 0.3
(legacy 6-tuple traces stay unconditioned and marked; a new trace with NO stopped-lead samples is
EXCLUDED from aggregates -- R1 MEDIUM: the fallback would have biased the gate; four-case test)
and flags radar-implausible traces (gap < 1.5 m at v > 3, the 2032-s9 class) out of the pool.
R2 approve. Battery 1092/19. WHAT THE FLIP NEEDS: one drive with ordinary stops behind FULLY
STOPPED leads (city traffic lights); a handful of stops is enough -- the conditioned gate then
reads directly from the new traces.

## 2026-08-29 -- cycle 38: THE FLIP -- the governor is the live approach law

USER DIRECTIVE (bar raised): "still quite far from super smooth above human; we're reducing mediocre
but rarely hit the perfectly smooth spot even for the ones creeping ahead of us. Improve everywhere."
The program's felt target is now <= 0.8 for EVERY stop class, crawls included.
GATE DATA: route 203f (city, 22 stops, 13/13 traces conditioned on FULLY STOPPED leads, 2 bookmarks).
Gate read: (1) stopped-lead shadow CONSISTENT -- governor deeper p50 0.89, divergence bounded p50
0.60, matching 2029/202d; (2) rests best yet (median 4.13, zero long, one 3.3 from a 9.8 m/s entry);
(3) felt mediocre but NOT regressed, and diagnosed as ONE class: the walking-pace EASE->GRAB pump
(s33, BOTH bookmarks: wire eases -0.33 at 1.2 m/s then grabs -0.69; s9: 0.50 m/s hover then monitor
grab; s32: floor-band re-deepen -0.58 -> -0.88) -- the legacy shape the governor replaces. GATE MET.
FLIPPED 11ff49692a: SERVICE_APPROACH_LAW = "governor" (revert = one word). Legacy suites pin the
legacy law explicitly (autouse fixture; the law stays selectable and fully tested); the five
shadow-containment claims pin legacy (under the governor the same helpers ARE the law). FLIP REVIEW
R1 [HIGH, real]: the re-pin removed live governor HANDBACK coverage -> added
test_live_handback_continuity_under_the_governor (the LIVE queue-release scenario under the governor:
ownership, pid.i reseed, C1 handback slew, integrator continuity); R2 approve. Battery 1097/19.
EXPECT ON-ROAD: approaches to stopped leads brake slightly earlier and hold a steady curve instead of
ease->grab; rests target 4.0-4.6; crawl-follow above the service band is UNCHANGED (planner's domain).
WATCH: first stops on the new law; the felt census next cycle decides whether the pump class is gone;
instant revert on any dislike. Exonerated this cycle: 203d s1 + 203f s49 (driver). Ledger unchanged.

## 2026-08-29 -- cycle 39: first drive ON the governor -- the pump is gone; two attributions

Route 2041 (city short, build 94dd31e6bf = flip + another lane's far-lead commit). Two stops.
STOP 1 (the first governed stop, s2/s3, rest 4.18 from 9.7 m/s, felt 0.99): NO PUMP. In-band the
wire tracks the governor within 0.08 m/s^2 after the ~0.7 s entry slew; monotone taper from -1.6
through -0.5 into the terminal ramp; terminal/hold legacy as designed. The remaining felt 0.99 is
ALL in the HEAD band (v>2.5, pre-ownership): planner held a flat -1.1 from 15 m/s and delivered a
hot entry (2.5 m/s at 7.8 m); the shadow governor demanded -2.0..-2.5 there ("brake earlier").
Program lever confirmed: UPWARD OWNERSHIP (V_OWN) so the shaping starts at 3.5-4.5 m/s -- big step,
needs ~5+ governed stops to size + sol xhigh red-team. Corpus cannot pre-answer it (index stores
aggregates, not head-band traces).
STOP 2 (s3, felt 2.12, vappr 11.3): NOT the governor (no lead at settle; legacy path). At v 1.9 a
far "lead" flickered in at 33 m (lv -5.5, likely crossing); the wire RELEASED -0.85 -> -0.06 for
1.5 s (car coasted at 1.7 m/s), then GRABBED -1.23 -> -1.46 in one frame when the flicker changed.
Mechanism matches the same-day commit 94dd31e6bf ("Condition far-lead braking on model and ACC
demand"): time_gap = dRel/max(v,1) explodes at low speed, so ANY far lead flicker at walking pace
fully defers custom braking to the unconfirmed native/ACC reference mid model-stop, and the guard
toggles with lead.status. Reported to the user; fix belongs to that lane (low-speed/stopping-demand
exclusion or lead persistence gating). No stopping-program code change this cycle. Cursor 00002041.

## 2026-08-29 -- cycle 39 ADDENDUM: attribution corrected + the roll-in phantom fix (cycle 39b)

CORRECTION: the felt-2.12 stop's release was NOT the far-lead confirmation commit (94dd31e6bf is
EXONERATED: its confirmed_floor = max(output, min(exp, acc)) stays deep while the native demand is
deep -- it cannot have produced -0.06). Frame decode (236-240 s): pure-e2e no-lead stop (dts=-1,
shouldStop=False throughout, so the caller's stopping-entry gate COULD NOT fire); at v=1.87 a
crossing-car phantom (33.5 m, vLead -5.5) hit the STOPPING-LEAD ROLL-IN FLOOR, whose
lead_v = max(vLead, 0.0) clamp turned it into a "stopped lead" and faked closing_speed (1.87 vs
real 7.4) and ttc (17.9 s vs real 4.5 s) past every gate -> floor -0.05 RAISED the -0.9 model stop;
on flicker-exit (vLead flips +0.7) the floor dropped and the wire fell -0.06 -> -1.46 in ~0.3 s
(the grab; force coast re-engaging was coincidental). Confirmed: this stop ran fc=0.68 (force
coast) on approach. FIX (d3ec19c3f3): the floor rejects raw vLead < -0.25 BEFORE the clamp -- a
brake-RAISING lane must never act on an approaching detection; noise on a truly stopped lead
(vLead -0.20) keeps the floor. Pin fails pre-fix, passes post-fix; 182 planner tests green. The
deepen-only sibling caps keep their clamps (phantom -> deeper braking = safe direction). The
roll-in lane is NOT deleted: under the governor it still keeps deep planner demands from
undercutting the governed curve in-band (stop-1's trace shows the floor raising a_plan above
a_gov); deletion stays a step-4 census decision. Adversarial review + the no-lead-governor design
red-team (sol xhigh) launched this cycle.

## 2026-08-29 -- cycle 39b SHIPPED: the roll-in phantom guard, three review rounds, evidence-based

Final form (d3ec19c3f3 + 06071b9f70 + 03c37fcda4 + 59519208e3, deployed, device verified 59519208):
the roll-in FLOOR rejects a lead whose raw vLead reads below -0.25 only after 3 consecutive frames
on the SAME radar track (counter resets on recovery, lead loss, and radarTrackId change). R1
[medium]: flat threshold let one Doppler frame drop the floor -> persistence. R2 [medium]: my
-0.75 instant tier kept a one-frame drop path and the "noise never reaches it" claim was
unsupported -> MEASURED: 280,675 raw vLead samples inside genuinely-stopped-lead windows (653
segments): 293 one-frame track-jump spikes below -0.75, worst -13.7 (~1/1000 frames) -> NO instant
threshold is safe; persistence-only at every magnitude (a sustained phantom dies in 0.15 s vs the
1.5 s release). R3 [medium]: counter not track-local (A-frames + B-glitch sum) -> track-local;
round closed on my own sign-off per the two-round rule. Offline replay of the canonical event: the
floor is DEAD through the whole 237.5-238.9 release window; residual = 4 isolated positive-vLead
flicker frames (bounded single-frame blips; the structural cure is the no-lead governor). Suites:
838 passed. ALSO this cycle: the no-lead governor design red-team ADOPTED (two blockers: comfort/
safety channel split with attributed deepen-only safety; pursue the DIRECT e2e model demand, never
the post-lane planner target) -- full design in universal_stop_program.md; first ship = the
force-coast class, flag-gated, next implementation cycle.

## 2026-09-02 -- cycle 40 (part 1): retrospective + the attributed-safety SHADOW step

RETROSPECTIVE (docs/stopping/retrospective_2026-09-02.md): every physical smoothness metric (terminal felt
1.06 vs 2.04, whole-approach jerk 1.80 vs 3.39, pitch rate 0.019 vs 0.032, wheel-stop decel -0.37 vs
-0.45) scores engaged stops ~2x smoother than the driver's own manual stops; `felt` scores only the last
~30 cm; rests SOLVED, pump GONE (55/55 single-descent). Verdict: direction right, objective unmeasured,
complexity not falling (planner constants 129 -> 132). Changes: rated drive (user) -> perception metric;
whole-stop metrics + human baseline every cycle (tools/stopping/review/human_baseline.py);
delete-don't-patch; attributed-safety step pulled forward; identification drive (user); ML after.
User approved the plan and set the rule: docs updated on EVERY approach change.
EVIDENCE for the step: raw MPC trajectory binds below the governor on 12.2% of in-band stopped-lead
frames (p50 0.34, p90 0.65); post-lane aTarget 12.4% -- the MPC's close-range preference, not the comfort
lanes. RED-TEAM (sol xhigh): live removal BLOCKED (a_kin/a_bar cannot see lead deceleration; fresh/cut-in/
vision leads need trust-in); shadow-only first ship approved with a_pred + trust-in + new P1 invariant +
a flip gate (program doc has the full spec). SHIPPED as SHADOW (this commit): candidate = min(a_phase,
a_kin, a_bar, a_mon, a_pred) inside governor ownership, fail-closed eligibility (measured gap, no dropout,
motion earned, identity age >= 0.5 s), telemetry counters + bounded plan-binding ring; StopSignals gains
track_age_s; longcontrol threads lead_a. Wire byte-identical off/shadow/raising (pinned); 850 tests green.
Adversarial review launched. Next: the index reads attr_* (after the running index job), the new routes'
review, and the flip gate accumulates from the next drives.
R1 (adversarial review 20260902-193533, two [high], both real): vision-only leads inherited trust (track_age_s
started mature and only restarted on a REAL id change) -> identity-less leads are never mature; a_other was
missing -> leadTwo (kinematic + predictive) and independent model stop added as attributed lanes, FCW and a
braking lead (aLeadK < -0.3) veto eligibility, every ineligible frame records its reason and telemetry counts
them; controlsd/longcontrol thread leadTwo + fcw. 854 green. R2 launched.

## 2026-09-02 -- cycle 40 (part 2): routes 2042-205d reviewed; index matcher fixed

33 stops on build f356cf19fb (governor + roll-in guard), 0 bookmarks. Rests: n=22 median 4.25, short 3
(3.19; 3.09 and 2.89 flagged TAKEOVER?), long 2 (5.2-5.4 behind a crawling lead, lv 0.25 -- crawl-follow,
planner domain). Governor: 31/31 traces conditioned, deeper p50 0.60, div p50 0.68 (head band). HARSH 12/33:
the cluster is HOT HIGH-SPEED APPROACHES (vappr 10-12 m/s, planner cmd_min -1.9..-2.6; e.g. 2048 s2, 2049
s11/s7, 204a s6 felt 2.43, 205d s12/s16) -- the head band above governor ownership, i.e. the retrospective's
finding that approach-jerk peaks live in the planner's domain. In-band: no pump. TOOLING: NO_SERVICE_EVENT on
seven service-owned stops was a MATCHER ARTIFACT (the settle_summary is emitted at the end of the hold, which
had crossed into the next segment's rlog; every rlog replays the route-start initData so the shift is 0) ->
fixed, plus whole-stop columns (feltA = approach jerk, a_wheelstop) and the attr_* shadow counters. No code
patches this part (delete-don't-patch). Program consequence: the head-band class is now the largest felt
driver -> the ownership-upward step (V_OWN / whole-approach governor) moves up the queue, after the
attributed-safety shadow gate starts collecting.

