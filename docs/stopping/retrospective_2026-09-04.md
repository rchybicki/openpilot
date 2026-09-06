# Stopping program review -- 2026-09-04, cycle 42

## Decision

Keep the whole-approach governor OFF. The implementation has no control consumer, but its recorded-input
activation gate fails. Do not change the live governor, TAU, terminal machinery, or rest-gap rule to make
this replay pass. The next control change still needs the rated drive and a validated plant.

The first implementation had 1,089 committed standstill frames and 956 committed gas-override frames in
405,887 replayed planner frames. The corrected certificate has zero in the excluded classes tested by the
replay. This is a correctness result, not a comfort or safety outcome for a changed vehicle trajectory.

## Evidence and limits

- Route sync inspected 10,170 remote files: no new or changed files, no download failures. The review cursor
  remains `00002074`. No new drive, bookmark, or identification data was available.
- `whole_approach_replay.py` runs the real planner shadow on recorded inputs over 14 indexed routes. It sees
  54 stopped-lead candidate stops, including 41 with an observed speed of at least 8 m/s. Only 23 of the 54
  have a driver-free 30 s approach window. These are not 54 clean held-out acceptance stops.
- The replay uses published, clipped `aTargetTrajectory` as a proxy for the unpublished raw MPC input.
  It holds the recorded vehicle geometry fixed. It cannot predict the rest gap, felt jerk, or later demand
  after an earlier brake command. Its earlier/later comparisons are diagnostics only. The final replay has
  capture certified at 3/13 observed high-speed boundary frames and multiple entries on 40/54 candidate
  approaches; only 1/54 show both requested recorded-state demand differences. The gate fails independently
  of any interpretation of these last two demand comparisons.
- Raw rlogs contain 28 attributed-safety summaries on routes 2072/2073: 7,395 frames, 4,727 eligible,
  937 plan-bound, 536 unexplained, released-depth sum 178.026, 11 predictive-only binds. The index has 34
  matched rows but only 26 distinct service events: eight repeated matches, plus two raw events absent from
  those matches. The prior 34-stop/17%/52% tally is superseded. Correct values are 19.8% of eligible frames
  plan-bound and 57.2% of binds unexplained. These remain below the 100-stop/5-route and 90% gates.
- The corpus has 109 rows and zero duplicate `(seg, t)` keys. Unique stop rows do not imply unique service
  summaries. Gate totals must come from unique raw events, not sums of matched index rows.

## Handover review verdicts A-I

**A -- Curve: research candidate only.** `stopping_governor.py:37` includes `(LAG + TAU) * q` in D(q),
but the feedforward at line 103 uses only TAU in the denominator. For motion exactly on D(q), the derivative
would give `-q / (LAG + TAU + q/A(q))`. At q=2.5 the implemented demand is -0.699; that derivative is -0.621.
This distinction needs an explicit delay-model decision before LIVE. The 0.6 tail and 2.5 knee are design
choices, not fits to passenger ratings. Do not replace them with a constant-jerk/body profile without the
rated objective and plant test. The existing final slew already provides an onset-rate constraint.

**B -- Handoff: do not retune TAU now.** On the new curve at v=2.5 and gap=11.460 m, the current service
law asks for -0.803 versus the head law's -0.699: a 0.104 m/s^2 target mismatch before limiter/plant effects.
On 75 first in-band stopped-lead trace samples (routes >=203f, ISD assumed 0.3), median old/new demand is
-1.240/-1.161. This is not evidence of an achieved smooth seam. The law still needs the actual service
seam, previous-wire seeding, and counterfactual tests before either an entry slew or a residual-only TAU
change is selected.

**C -- Terminal: leave unchanged.** The 7 m bookmark supports the planner-min over-brake mechanism.
It does not prove the alternate trajectory would stop at 4.3 m. That earlier prediction is withdrawn.
The terminal stop/hold path must not gain a position-chasing exception. Test the common comfort authority
first; any later terminal proposal needs the leapfrog, re-launch, rollback and final-gap cases.

**D -- Attributed safety: gate failed; no evidence of constant predictive binding in this drive.**
`attr_pred_bound=11/4727` eligible frames (0.23%) counts occasions where the predictive lane is strictly
below every other attributed lane. It does not count ties or measure all causal influence. FCW as an
eligibility veto is valid for this shadow: the alert is not a numeric braking demand. Live removal of
MPC still needs explicit fail-closed attribution for all hazards; this sample does not justify it.

**E -- Trust-in: a material coverage limit.** Of 302,597 lead-status planner frames across the 14 replay
routes, 67,472 (22.3%) have no radar identity. This is an all-driving-frame rate, not a stop-class rate.
Keep the real-identity boundary; measure stop-specific coverage before proposing a different certificate.
Do not turn a persistent `-1` sentinel into evidence of persistent identity.

**F -- Metric: pre-register before the rated drive.** Keep existing `felt` output for historical comparison;
it is not the sole passenger-feel objective. Pre-register these features on each uniquely matched stop:
(1) last rolling-frame aEgo; (2) minimum aEgo in 0.45-3 m/s; (3) maximum absolute 300 ms acceleration change
per second over the continuous approach; (4) peak absolute valid pitch rate; (5) deficit against the frozen
comfort curve. Use full time windows, interpolation at t-0.300, no windows across missing data, and report
coverage. No post-hoc smoothing or speed/window fitting against the labels. Freeze route-grouped validation;
compare rank/order separation of perfect, single-bookmark and double-bookmark stops. A missing bookmark
is a perfect rating only on a drive the user explicitly rated completely. Unmatched/ambiguous labels stay
out. Fewer than roughly 30 unambiguous rated stops: descriptive results only, no learned controller.

**G -- Process: the deletion rule was not met by prior guard cycles.** This cycle repairs the new certificate
and proves isolation; it adds no guard to a legacy comfort lane. No existing lane is deletion-safe merely
because the shadow exists. The existing deletion order remains attached to the two live gates: attributed
LIVE -> roll-in state/floor, low-speed cap family, REST-CLOSE after trajectory/entry proof; whole-approach
LIVE -> head-band comfort lanes and stop-aim/stop-commit floors after hazard equivalence. Default-off
research code has an explicit removal option if the certificate or curve is rejected.

**H -- Tooling: repair the inference, preserve historical scores.** The current approach and terminal jerk
loops accept partial windows at a window boundary (`deep_stop.py:161`, `human_baseline.py:63`,
`terminal_smoothness.py:164`). Examples: 2049 s7 at t=462.8 reports 12.011 using 8.0 ms; requiring at least
250 ms in this diagnostic leaves a 2.064 peak. 2048 s2 reports 10.863 using 10.8 ms, while a 299 ms window
still reaches 6.707. Thus some spikes are estimator artifacts and some broad transients remain. Rounded
index settle times can also change the first partial-window sample. Do not silently smooth the data or
replace all peaks with pitch rate. Use the separately named, full-window metric in F and retain raw signals.

**I -- Safety sweep: shadow isolation verified, live enablement not verified.** The new calculation runs
after final planner output assignments, with separate authority/context state and an exception boundary.
The replacement test executes real `LongitudinalPlanner.update()` and `publish()` for Santa Fe and another
car, with a substituted native solver, both flags and an injected shadow exception. All non-shadow
published fields match (processing timestamp excluded). This does not replace the real solver, actuator,
100 Hz takeover/handback, or plant acceptance gates. The live service and longcontrol are unchanged by this
cycle. The final adversarial review record below defines any remaining sweep findings.

## Reproduce

From the repository root, activate `.venv`, then:

```sh
PYTHONDONTWRITEBYTECODE=1 python tools/stopping/review/whole_approach_replay.py --output /tmp/whole-approach-replay.json
```

The tool has no device writer. It explicitly enables the shadow inside the replay process and leaves the
repository default unchanged. Raw attributed totals are independent of the stop-index matcher.

## Adversarial review R1 -- 20260904-212702-exec (sol xhigh)

Verdict: request changes; OFF is correct, no SHADOW/LIVE activation. I accept the findings:

1. Frozen radar samples could earn age and persistence. The planner now requires an updated, valid, alive
   radar sample before advancing the shadow; otherwise it resets the shadow and reports `radar`. The replay
   requires a new radar timestamp no more than 100 ms old. This deliberately conservative gate can reduce
   coverage; do not weaken it to make capture counts pass.
2. Gap-trust loss fragments one physical stop into multiple commitments. This is an activation blocker,
   not a reason to add another release exception. The current research state remains OFF.
3. The head candidate is incomplete: no barrier or independent model-stop/lead-two demand; raw MPC always
   binds. That was the implementation prompt's narrower contract, not the full adopted invariant. The
   next design must supply the complete attributed set before removing raw MPC for mature stops.
4. The replay is diagnostic only: no below-2.5 demand, no counterfactual plant, insufficient driver-free
   stops, no explicit crossing/phantom census, and no complete slew/release gate. The flag cannot be enabled
   on the strength of its max-difference statistics.
5. A separate pre-existing LIVE defect exists in `apply_santa_fe_far_lead_brake_confirmation` and
   `get_santa_fe_stopping_lead_roll_in_floor`: `dRel=NaN` can produce NaN; roll-in `dRel=Inf` can raise -1.5
   to zero, and `vLead=NaN` can raise it to -0.12766. The next wire-affecting task must establish a shared
   fail-closed lead-input boundary and prove finite nominal equality, fault-frame no-release and takeover/
   handback invariants before deployment. This cycle does not patch those deletion-candidate lanes or
   claim their invalid-input behavior is safe. This issue takes priority over a new live governor feature.
6. The curve derivative/delay interpretation and service seam still require a plant-backed design decision.

The review verified control-field isolation, no new telemetry consumer, and no takeover/handback seed
change in the selected governor/attributed-shadow commits. It independently ran 897 tests (19 skipped).
The current diff is a disabled research scaffold, not an approved controller or an enabled shadow rollout.

## Review close and main-agent sign-off

R2 `20260904-215628-exec` (sol xhigh) completed with exit 0: **LAND DISABLED RESEARCH CODE. Do not activate
SHADOW or LIVE.** No new landing blocker. Both reviews are closed; no third round. I adopt that limited
verdict. The complete battery passes 897 tests with 19 skips; schema/AST loading and new-code Ruff checks
pass. The two existing stop-index E501 lines are unchanged. Final stale-gated replay: 405,887 frames,
zero excluded commitments, 315 entries/releases, capture 3/13, repeated entries 40/54 candidate approaches.
These are failed activation evidence, not a passed physical gate.

## Cycle 43 -- process reset after the user's second review request

**Decision:** improve the experiment loop before extending the controller. Cycle 42 added 439 runtime
lines and left them disabled. It fixed research correctness, but supplied no smoother live stop. The
planner still has 132 `SANTA_FE_*` constants. The deletion program has not yet removed its target lanes.

Three earlier conclusions need limits:
- “All arrivals are hot” means late relative to our chosen curve. That curve is not a fitted passenger
  preference, so this alone does not prove every stop is physically late or the curve is right.
- “Engaged is twice as smooth as manual” used partial-window jerk and unmatched stops. Manual braking is
  a reference, not the user's target. Even a correct group median does not establish perceived comfort.
- A candidate fed the original late trajectory cannot show what late demand would be AFTER earlier braking.
  Use recorded-input replay for trust, coverage, timing and arbitration; use validated free rollout for outcomes.

The current data also does not justify declaring the plant impossible to identify without one particular
new drive. There are 179 extracted episodes and an older event store with 250 events (117 use telemetry v2,
conditioned signals v2 and sent `carOutput`; 133 use legacy `carControl`). The store ends at route 00001f00.
It is not a current-controller validation set. The June fit's low one-step error and the cycle-34 failed
free rollout remain distinct evidence. First audit natural command changes in the CURRENT logs, response
coverage by speed/deceleration/grade, and free-rollout error on route-held-out data. Do not fit per-stop
residuals from the same answer being predicted. Request new collection only for a demonstrated missing regime.

**Measurement delivered.** Upgraded the existing `human_baseline.py`; no new runtime feature or dependency.
Metric v2 uses native integer timestamps, complete 300 ms windows with interpolation, no smoothing,
<=100 ms car-state gaps, and explicit metric-level null/reason output. A standstill needs a known entry
and continuous >=500 ms below .05 m/s. Classification checks fresh, non-future state (age <=500 ms), gas
and brake over [stop-10s, stop+.5s]. Transitions/engaged override are mixed; missing context/state is unknown.
Fully disengaged driving with a brake press is manual, including normal accelerator use before braking.
The approach interval is the final <=9 m/s band before the <.45 descent within the last ten seconds.
The terminal interval ends 0.5 s after rest. The continuous 10.5 s score prevents the phase split hiding
an inter-phase change. `a_last_ge_010` names the old wheel-stop acceleration precisely: last pre-stop
sample with v>=.10, not acceleration at exact rest. No historical <=0.8 threshold is applied to v2.

All 2,019 available rlog segments in 00002000..00002074 (116 routes) decoded successfully. There are
329 detected stops in 303 segments / 82 routes: **88 engaged, 181 manual, 29 mixed, 31 unknown**. The old
classifier called the same census 100 engaged and 229 manual. The 31 unknowns lack full segment context;
all retain valid terminal and last-rolling measurements, but no valid full approach score. This is still
a per-segment census: stops that cross a segment boundary and whole approaches longer than ten seconds
are not fully covered. It is not the complete driving stop population or a held-out acceptance set.

| Metric, median | Engaged (88) | Manual (181) |
|---|---:|---:|
| Full 300 ms terminal jerk, m/s³ | 0.920 | 1.392 |
| Full 300 ms approach jerk, m/s³ | 1.435 | 2.855 |
| Full 300 ms continuous stop-window jerk, m/s³ | 1.461 | 2.948 |
| Minimum acceleration at .45-3 m/s, m/s² | -1.020 | -1.532 |
| Last pre-stop acceleration at v>=.10, m/s² | -0.365 | -0.440 |

Example: 00002049--1da44dd6cd--7 at route t=462.758 s now has approach jerk 2.052, versus legacy index
12.011 from an 8 ms window. This is a measurement correction, not a car improvement. The aggregate also
reports 3-6 / 6-9 / >=9 m/s speed strata with metric denominators; lead type, slope, software era and driver
intent are not yet matched. Do not turn the table into a claim of causal superiority or passenger satisfaction.

**Next bounded experiment:** reconstruct continuous approaches for the existing bookmarks and comparable
unmarked stops. Treat bookmarks as reported discomfort and unmarked stops as unknown. For each late brake
increase, show the raw planner, comfort reference, each safety demand, final sent command and delayed aEgo.
Reject the single-comfort-conflict explanation where the increase is explained by a real hazard or follows
an already deeper early command; keep separate counts. In parallel with that analysis, audit whether the
same recorded commands identify the actuator response well enough for free rollout. Use the existing
extractor/event store/fitter rather than another controller or another parallel analysis stack. This opens
one decision: test the common comfort authority or correct the response model. It does not preselect a new curve.

The revised program top section is now the current plan. Each later experiment must state its comparator,
falsifier, data split, affected classes and deletion. Historical routes already used for design are development
data; freeze future route groups before tuning and final acceptance. About 30 explicit ratings support
exploratory feature ordering, not a learned production metric. The shared non-finite lead-input repair stays
the next wire safety task. No controller code, flags, service seam, deployment or activation changed here.

Reproduction data: `~/.route_sync/corpus/baseline_v2_20260904/` contains `manifest.json` (all 2,019
input paths, sizes/mtimes, source and output SHA-256), `stops-v2.jsonl`, `summary-v2.json`, and frozen
`legacy-v1.jsonl`. Raw input files were not hashed. Activate `.venv`, run `human_baseline.py` once per
manifest path, then `python tools/stopping/review/human_baseline.py --aggregate <results.jsonl>`.
Final stops SHA-256: `59e4ac71e59fa6699f220a509713ecdaa62bd312e5c5526c892db03f9af06aca`.
The first v2 pass excluded accelerator use even during fully manual driving; corrected that unintended
selection rule, added its regression check, and decoded all 2,019 inputs again. Only class assignments
changed (94 mixed -> manual); all 329 IDs and per-stop metric values stayed the same.

Review close: sol xhigh R1 `20260904-222239-exec` and R2 `20260904-223606-exec` completed. R2 requested
a defined p90 convention and rejection of negative imported jerk; both fixed and regression-checked.
p90 now uses nearest rank `ceil(.9*n)` (one-based). Final aggregate regenerated; per-stop scores unchanged.
Main-agent sign-off: 9 focused tests pass, Ruff/diff/JSON checks pass, final data hashes verified. R2's
corpus spot-check used the earlier class-filter pass; the main agent checked the final 181-manual rerun.
Two review rounds complete. No third review or vehicle deployment for this offline change.
