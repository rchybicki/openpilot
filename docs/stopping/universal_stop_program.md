# The universal stop program (opened 2026-08-23, cycle 34)

## Current decision -- deployment prerequisite, 2026-09-05 (cycle 46)

This section supersedes conflicting priorities and evidence claims in the historical log below.
The target is a smooth COMPLETE stop: early brake onset, no unnecessary late increase, a smooth final
release, and secure hold. Keep the user's rest rule (aim 4-5 m, allow 3-4 m, hard floor 3 m, no end chase).
Rest gap and pump remain regression checks for every class; earlier local successes did not solve them globally.

One comfort authority with separate, attributed safety remains the direction. The exact new curve and
certificate are research candidates. Do not add another law, retune the service, or patch a legacy comfort
lane until one repeatable comparison can reject or support the mechanism. The whole-approach extension stays OFF.

Use four distinct gates:
1. **Code:** isolation, finite inputs, correct reset/seeding, other-car equality and fault behavior.
2. **Recorded inputs:** trusted target coverage, timely capture, continuous ownership, candidate rate limits,
   and complete safety attribution. Fix the current capture/chatter failures before SHADOW. A fixed-input
   replay cannot prove a new rest gap or less late braking after earlier braking changes the trajectory.
   Move that earlier/shallower OUTCOME gate to 3/4; do not relax the other numerical pre-SHADOW gates.
3. **Simulation:** reproduce recorded commands in free rollout on held-out routes, without feeding recorded
   acceleration back at each step or fitting a per-stop residual from the answer. Retain the plant/rest
   error bounds below. Only then compare candidate laws on changed trajectories and the full service seam.
4. **Road evidence:** controlled confirmation of physical outcomes and explicit driver ratings; continue
   to count safety interventions, incomplete stops and all excluded data. No SHADOW/LIVE activation in this offline work.

Cycle-44 background: metric v2 is complete. Joined current routes provide 85 clean body-braking runs; a
57-run fit gives two-second acceleration path error p90 0.189 m/s² on the second route, versus 0.332 for
the prototype table. This is 104 horizons from 27 runs, not full-stop or final acceptance. See
[the cycle-44 evidence and limits](offline_progress_2026-09-04.md). The complete-stop test below supersedes
that short-horizon result as evidence for model adequacy. Check response direction/delay sensitivity before
ranking changed comfort commands.
Reuse the event store, extractor and fitter. The specific scripted-step
drive below is a proposal, not an implemented prerequisite: first establish exactly what existing data lacks.
If it lacks an identifiable response, prepare a bounded controlled collection protocol. No brake-step hook
is authorized by this process reset. Sparse bookmarks are unknown, not perfect; ~30 explicit ratings can
support exploratory ordering, not automatic learned-score acceptance. The shared NaN input-boundary repair
remains the next wire safety task and a prerequisite to a new wire feature; it is separate from comfort progress.

Cycle 45 rejects the frozen cycle-44 fit as a complete-stop comparator: 9/34 admitted episodes (6 on the
training route, 3 on development validation), with validation distance errors +6.315, +1.436 and -18.256 m
at the recorded stop time. The last case starts above the fitted 15 m/s range; the first two also fail.
Most displacement error is already present before 0.5 m/s. Cycle 46 resolves the signal-consistency issue;
next fit/evaluate coherent full-episode speed and distance before adding routes or scoring comfort laws. Do not
patch terminal control to hide this model failure. See [the cycle-45 report](offline_full_stop_2026-09-05.md).
Every exclusion and incomplete prediction remains visible. The design review attempt `20260905-070326-exec` ended at the
review tool's usage limit (exit 1), without a verdict. Its frozen checks pass; the report is retained as
an offline rejection result, not a basis for activation. Cycle-46 review covers the bounded deployment below.

Cycle 46: the user authorized work through a tested deployment. Address the known shared invalid-lead
boundary first; it is a prerequisite to any new live governor feature. Design review approved the repair
with required corrections. The shared entry guard is implemented; 973 tests pass (19 skipped), and normal
outputs match exactly. Final review `20260905-080152-exec` approves with no runtime blockers. A bad
service/secondary input retains valid primary-lead braking. Code `e1c2b323` is deployed and reboot-verified;
manager/UI and deployed imports pass. No new on-road outcome is claimed. GitHub key repair now uses
persistent `/data/ssh`; normal updater fetch and post-reboot fetch both pass. SSH repair is complete.
The source-derived wheel-filter identity resolves the cycle-45
acceleration/speed consistency question: all nine recorded speed traces reconstruct within 0.000002 m/s.
This uses recorded acceleration and is not a plant-validation result. See
[the 2026-09-05 review](retrospective_2026-09-05.md). Nominal comfort still requires coherent full-episode
plant validation; no failed activation gate is relaxed to obtain a deployment.

Cycle 47 resumes offline work after SSH repair. Before refitting, test how much full-stop error the missing
wheel-filter state coupling can explain. Design review `20260905-082632-exec` rejected the proposed 10 Hz
approximation: its 0.04961 m/s speed error is comparable to the 0.05 m/s rest threshold; distance error reaches
0.3466 m. Use native 100 Hz propagation of the unchanged fitted model/held 10 Hz commands, with a matching
native no-coupling control to separate integration effects. Preserve all 34 IDs/9 admitted/25 exclusions
and recompute the exact old baseline. Gate uses all three development-validation episodes (not new held-out
data): travel error <=25% of original; speed RMSE <=50% of both comparators; complete without new relaunch,
stop-time error increase <=0.1 s and acceleration RMSE increase <=10%. The six training episodes must have
no new noncompletion, relaunch or numeric failure. This is sensitivity analysis, with no runtime change or activation.

Cycle 47 result: the coupled validation travel errors remain +5.790, +0.698 and -19.097 m. All three fail
the predeclared reduction gate. The missing filter coupling is insufficient; next specify full-episode
speed/distance loss and a stationary observation model. See [the result](offline_observation_2026-09-05.md).
The original packet is exactly reproduced and unchanged. Evidence review `20260905-083954-exec` passes
with no correction. Main-agent signoff accepts the rejection; no runtime change.

On-road observation now uses existing logs copied to the host. Repeated live diagnostic subscribers could
have caused the reported communication fault; the later complete log minute has no fault. Do not add live
message readers or import driving modules on the device for health checks. See the incident record in the
[2026-09-05 review](retrospective_2026-09-05.md).

Every experiment records one falsifiable mechanism, fixed data split, comparator, success/rejection rule,
affected classes, and the existing code it will remove. Report evidence gained and live behavior changed;
tests, disabled code, commits and review rounds are not smoothness gains. Use at most two scoped reviews,
with a self-contained evidence packet frozen before review. Deploy research only when on-car telemetry answers a named gap.
See [cycle 43 in the retrospective](retrospective_2026-09-04.md#cycle-43--process-reset-after-the-users-second-review-request).

**Directive (user):** stop building the tree of ifs. One universal approach; lanes are deleted as an offline
harness proves them redundant; an ML controller is the NEXT program and reuses this harness and plant model.

**Why:** thirty-three cycles patched one stop class per cycle with deepen-only / raise-only lanes around one root
(the e2e reference dies or eases at low speed; the feedforward plant under-delivers). Inventory at opening:
longitudinal_planner.py 129 `SANTA_FE_*` constants, 40 Santa-Fe functions, 52 lane state variables; stopping_service.py
1271 lines / 67 params; longcontrol.py 1595 lines / 16 Santa-Fe cap functions; 12 kill flags. The June-2026 review
condemned exactly this shape; it re-formed in the planner.

## Architecture (design review 2026-08-23, sol xhigh; adopted with the arbitrations marked *)
- The governor OWNS the wire through the existing StoppingService seam in longcontrol (100 Hz takeover, PID
  freeze/reseed, fault fallback, handback already exist). It replaces the service's approach laws (GLIDE/EASE,
  floor-defense, normalization, late-entry). The planner is a deepen-only SAFETY source only (`aTargetTrajectory`
  = the unmodified solver demand; the model/no-lead stop demand as a separate safety input).
- Ownership from `V_OWN` = 4.5 m/s (conditional: StopContext warm before the boundary, authorized lead only,
  planner and a_kin can always deepen, takeover from the actual previous wire, small release hysteresis, harness
  entries at 4.3-4.7 m/s). MPC/e2e keep full authority above.
- NOT blended-only: a trusted slow lead uses the same law in ACC mode.
- "Stateless" means no stop-class episode policy. Delay buffer, jerk-limiter state, target-rate state, wheel-stop
  latch and tracking state remain.

## The law (three fitted parameters A_C, TAU, LAG)
```
q     = max(v - v_lead, 0)
d     = max(gap - (4.0 + ISD) - LAG * q, 0)
z     = sqrt((A_C*TAU)^2 + 2*A_C*d);  q_ref = z - A_C*TAU          # d = TAU*q_ref + q_ref^2/(2 A_C)
v_ref = max(v_lead, 0) + q_ref
a_ff  = -A_C * q_ref / (q_ref + A_C*TAU)                           # -A_C far away, 0 at the anchor
a_gov = clip(a_ff + (v_ref - v)/TAU, -A_MAX, A_UP)
```
Far away it is the sqrt closure curve; near the anchor q_ref -> d/TAU (no singularity, no position-error
denominator: "never chase" by construction). Plus: a lag-aware 3.0-3.1 m BARRIER in the safety helper (a_kin only
protects D_HARD 2.0; safety braking may chase, comfort may not); the clutch physics (gradual RAMP_TO_HOLD, -0.70
secure hold, roll detection, HOLD, trusted RELEASE) stay exactly as measured; the no-lead/model stop demand passes
through the same final limiter and terminal logic.
Known failure cases and their owners: lead decelerating hard (raw MPC/e2e + a_kin deepen; no noisy lead-accel
feedforward first); lead accelerating away (v_ref/A_UP only after `lead_motion_earned`; existing departure dwell);
reversal (`max(v_lead,0)` only in the comfort reference, a_kin uses the real negative speed); grade (conditioned
a_coast residual or the small integral); regen->friction (smooth gain table, fast unwind); ISD changes mid-drive
(rate-limit the target; ignore after the terminal transition).

## Tracking (plant: realized/sent 0.97 @ -0.6 .. 0.78 < -2.5, delay ~0.45 s, lag ~0.5 s)
Static inverse gain first (command u with g(u)*u = a_desired), then a SMALL delayed demand-referenced integral for
grade/model residual. Learn only when: governor is the selected lane, the final limiter is not binding, a_kin and
planner safety not binding, no actuator/controller limit, no gas, terminal modes inactive, v > ~0.8-1.0 m/s.
Freeze/unwind below; clear on HOLD, disengagement, fault fallback, governor disable.

## Behaviours that MUST stay (synthetic tests; the stop corpus cannot prove them)
Lead authority (track-lifetime rejection, early radar acquisition, later model certification); gap provenance
(inward instant, outward delayed, optimistic holds never shallow braking); dropout (inward decay-hold, no release);
identity handover (inward may deepen, outward earns trust first); reversal deepens through the real relative speed;
wheel-stop latch (0.06 m/s dwell, reset above 0.09); clutch hold; departure release conditions; Force Coast (no-target
ramp + standstill hold, never enters the governor); gas override (freeze/clear, reseed from the actual post-override
wire); disengagement resets everything; non-finite/exception fallback (never shallower on the fault frame).

## Harness acceptance gates (frozen before fitting)
A. Route-grouped train/val/test; >= 100 clean driver-free held-out stops; recorded-command replay: |aEgo err| p50
   <= 0.08 / p90 <= 0.20; rest-gap err median within +-0.10, p90 <= 0.35, p99 <= 0.60; rest-time err p90 <= 0.40 s;
   >= 2 s of command history before V_OWN. Fail -> do not score the law.
B. Nominal held-out: 100% completed; no frame < 3.0 m; median rest 4.1-4.6; p10 >= 3.5; p90 <= 5.0; >= 90% in
   [3.5, 5.0]; zero > 5.5; governor decel p99 <= 2.25 / max 2.5; governor owns >= 90% of frames V_OWN -> 0.3; a_kin
   never binds on an ordinary stop. Non-zero ISD: same distribution relative to 4.0+ISD.
C. Terminal feel: the EXISTING scorer (wire_jerk_max <= 0.80, wire_pump <= 0.06, descent_count == 1, felt <= 0.80,
   no relaunch); safety-binding frames exempt but counted.
D. No-chase, from LIVE geometry (0 <= gap-(4.0+ISD) <= 1.5 m, before wheel stop): governor-only max(-a) <= 0.90;
   increase vs the previous 1.5 m band <= 0.20; <= 1 non-safety deepen episode; deeper events attributed to the
   barrier / a_kin / planner safety.
E. Takeover/handback at 100 Hz: ordinary takeover step <= 0.025; safety takeover <= 0.080; RELEASE <= 0.012/frame;
   one entry + one release per stop; no chatter in 4.3-4.7 sweeps; fault frame never shallower; non-Santa-Fe
   frame-identical; gas press -> zero braking through the existing override; gas release restarts from the real wire.
F. Robustness matrix: delay 0.30-0.70, lag 0.30-0.80, gain 0.75-1.05, bias +-0.5, gap noise +-0.10, lead-speed
   bursts +-0.20 x 0.30 s, lead decel 0..-4, lead accel 0..+2, reversal to -0.5, dropout 0-2 s, identity
   replacement in/out, ISD 0-3 m. Plausible cells keep 3.0 m; fault cells keep D_HARD 2.0 and name the binding lane.
The cycle-34 prototype harness (tools/stopping/review/stop_harness.py) is NOT an acceptance harness: radar cadence,
no StopContext/authority, no a_kin/merge/limiter/release/handback, chase window from future information.

## Deletion order (each step gated by the harness, one step per cycle)
1. Valid harness + plant sanity gate (delete nothing).
2. Governor in SHADOW through the service seam; log every arbitration source.
3. Replace APPROACH_GLIDE + PRE_STOP_EASE with the governor; delete `_update_d_rest_eff`, floor-defense shaping,
   `_glide_demand`, `_ease_gates_pass`, `_ease_demand`, relief entry/catch-up, normalization latch+dwell, late-entry
   corridor, redundant anti-hover. Keep wheel-stop, secure hold, post-stop arrest, RELEASE, the sole limiter.
4. Delete the low-speed planner lanes inside V_OWN: rest-close (all arm/spend/epoch/debounce state), roll-in,
   creep extension. Gate: crawler rests pass, none > 5.5, no post-stop re-close, no more floor/safety bindings.
5. Delete the Santa-Fe low-speed longcontrol caps (close-lead cap, glide cap, far-stopped-lead trio, close-gap creep,
   brake-model alignment and stop-target writers on governor-owned frames). Keep generic behaviour for other cars.
6. Delete stop-aim and stop-commit as WHOLE lanes (no 4.5 m/s shims) when every recorded trigger and synthetic
   re-slam/rollback case passes with raw MPC/e2e + governor.
7. Delete the high-speed approach lanes last (decelerating-lead feedforward, lead caution, slowing-lead tables, downhill
   tables, late-approach tables, downhill clip step). Gate: all historical triggers pass from their entry speed; no
   episode reaches V_OWN with unrecoverable debt.
8. Delete obsolete flags, tests, telemetry, comments. One rollout mode until on-road validation completes.

## Status log
- 2026-08-23 cycle 34: program opened; design review adopted; corpus extraction + prototype harness built
  (tools/stopping/review/extract_episodes.py, stop_harness.py); harness hardening delegated (gates G1-G6 of the
  prototype spec); governor law + barrier as pure helpers in SHADOW is this cycle's on-road deliverable.
- 2026-08-23 cycle 34 close: shadow governor SHIPPED (aa05f13ef6; R1 fault-latch containment, R2 approve). Harness
  verdict: prototype only -- the generic plant does not reproduce the record (rest error p50 1.85 m, residual RMS
  0.40, 0.7 s autocorrelation); ARX identification on closed-loop stop data is ill-conditioned (2 s rollout RMS
  0.42, unstable fits) -> an IDENTIFICATION DRIVE (step commands on an empty road) is a prerequisite for offline law
  ranking; until then the on-road shadow is the evidence path. Episode corpus needs a target-switch filter.

## Identification drive (prerequisite for offline law ranking; protocol v1, 2026-08-24)
Closed-loop stop data cannot identify the plant (cycle 34: ARX one-step 0.06 but 2 s rollout 0.42, unstable fits).
Protocol: empty straight road, dry, flat and one known grade; engaged in ACC with NO lead; from 8-10 m/s
apply scripted brake-demand STEPS through a temporary test hook (flag-gated, off by default, never with a
lead): -0.5, -1.0, -1.5, -2.0, -2.5 m/s^2 each held 3 s from 8 m/s, plus two ramps (-0.5 -> -2.0 over 3 s and
back) and one regen->friction crossing (-0.8 -> -2.2 step); 3 repetitions each; log at 100 Hz. Fit: gain(depth),
pure delay, first-order lag, grade term, regen/friction regime split. Acceptance: the fitted model's FREE
rollout over the recorded corpus stops reproduces aEgo within p90 0.20 m/s^2 and the rest gap within p90
0.35 m (program gate A) -- only then does the harness rank laws.

## Review tooling (2026-08-24)
Cycle entry point is tools/stopping/review/stop_index.py: qlog triage of every segment, rlog analysis only for
candidate segments, persistent index, ATTENTION-first output, detector audit against the service's own
settle_summary events, shadow-governor gov_* summary. The reviewer reads flagged rows and frame windows only.
- 2026-08-26 cycle 35: review tooling (stop_index.py) shipped; first shadow drive: governor deeper than the wire on 96%
  of in-band approach frames (the stack under-brakes at 2.5 -> short rests) and long rests behind crawlers dominate;
  PRE-BAND shadow from V_OWN 4.5 shipped (a09d9f020a) so the next drives show the law's intended band.
- 2026-08-27 cycle 36: STEP 3 SHIPPED DARK (2ddb3ca860+9abee9dbee): SERVICE_APPROACH_LAW flag; governor as the
  approach law with the glide-law patches inert and the 3.1 m barrier live fail-closed; legacy default is
  bit-identical (2000-frame differential in review). The flip is one reviewed word, gated on the next drives'
  conditioned shadow. Ledger: index plausibility flag for radar-garbage pre-band traces; decelerating-lead
  frames belong to the planner safety lane (condition all governor-vs-wire statistics on lead state).
- 2026-08-29 cycle 38: THE FLIP (11ff49692a) -- the governor is the LIVE approach law (gate met on 203f: 13/13
  stopped-lead traces consistent; both user bookmarks were the walking-pace ease->grab pump = the legacy shape).
  BAR RAISED (user directive): the program target is felt <= 0.8 for EVERY stop class, crawls included. Next:
  on-road validation of the governor; then deletion steps 4-5 (planner low-speed lanes, longcontrol caps) as the
  harness/census proves them redundant; crawl-follow smoothness (above the service band) is a planner-side item.
- 2026-08-29 SCOPE EXTENSION (user directive): no-lead / force-coast stops join the program. Principle
  (user-stated, code-confirmed): with no lead there is no hard gap anchor and no barrier -- the only position
  constraint is the model stop line, which has meters of slack -- so ANY harshness in this class is a controller
  artifact, never physics. Acceptance: felt <= 0.8, same bar as lead stops. Today's violations: (1) the model
  e2e stop demand punches through the force-coast cap RAW (apply_force_coast_strength_brake_limit:
  brake_limit = min(cap, model_accel) -- no jerk bound, model brakes late-deep); (2) demand-reference toggling
  keyed on lead.status (CORRECTED: the roll-in FLOOR's max(vLead,0) clamp turned a 33 m crossing-car phantom
  into a "stopped lead" and raised a pure-e2e stop to -0.05, dropping -1.46 on flicker-exit; felt 2.12, fc
  0.68; 94dd31e6bf exonerated; narrow fix d3ec19c3f3 = raw vLead < -0.25 rejects the floor); (3) longcontrol's no-target pid cap ladder is an if-ladder version of the taper the governor should
  own. Planned form: the same governor law with d = model distanceToStopTarget (in longitudinalPlan) and NO
  barrier, or the degenerate distance-free form (jerk-bounded pursuit of the stop demand with the comfort
  asymptote and a free stop point). Needs sol xhigh red-team; sequence AFTER the far-lead lane's low-speed fix
  (or fold it in). Index now records fc (force-coast approach fraction) per settle for the class census.
- 2026-08-29 NO-LEAD GOVERNOR DESIGN (sol xhigh red-team ADOPTED, run 20260829-152750): my draft amended by
  two blockers: (B1) depth/rate thresholds CANNOT separate a comfort stop from a pedestrian brake -> two
  explicit channels: a_comfort = governed DIRECT e2e model demand; a_safety = attributed deepen-only demands
  (lead MPC, a_kin, FCW/AEB) that may bypass shaping (bypasses logged as acceptance failures, never blocked);
  (B2) never pursue post-lane planner aTarget as the comfort reference (smoothing a poisoned reference keeps
  the poison). Episode state machine (entry: v<2.5 + model demand <=-0.25 sustained 0.3 s + no trusted lead;
  exit hysteresis v>2.8; release: model >=-0.05 or go >+0.2 for 0.15-0.25 s; gas override clears instantly;
  reference family FROZEN for the episode -- lead flicker can only add attributed deepen-only safety; after a
  release, re-deepening re-earns the entry dwell unless safety binds). NO stop-point escape in v1 (accept
  drift, bound <=3 m measured). Rates J_deepen 0.6 / J_release 0.8 (test values). FIRST SHIP = the FORCE-COAST
  no-lead class only, service-side, flag-gated, comfort_reference = min(force_coast_level, model_comfort),
  keep the -0.32 FC hold. Offline gate: felt<=0.8 + wire_pump<=0.06 + descent_count==1 + one entry/<=1 release
  per stop + zero unattributed comfort-rate bypasses + canonical 00002041-seg3 replay. Telemetry additions
  (ownership entries/releases + reasons, reference provenance, drift, bypass frames) land with the step.
- 2026-09-02 RETROSPECTIVE (docs/stopping/retrospective_2026-09-02.md): every physical smoothness metric
  (terminal felt, whole-approach jerk, pitch rate, wheel-stop decel) scores engaged stops ~2x smoother than the
  driver's own manual stops; `felt` scores only the last ~30 cm. The objective is unmeasured -> (1) per-stop
  rating drive from the user calibrates a perception metric BEFORE further optimisation; (2) NEXT STRUCTURAL
  STEP pulled forward: inside governor ownership the planner's COMFORT output is not a min-input, only ATTRIBUTED
  safety demands may deepen (covers roll-in floor, head band, no-lead in one design); (3) delete-don't-patch for
  deletion-candidate lanes; (4) identification drive unblocks the offline loop; (5) ML after (1)+(4).
- 2026-09-02 PROCESS RULE (user): the program doc and worklog are updated EVERY time the approach changes --
  mid-cycle included -- and the docs commit travels with the change it describes. Retrospectives get their own
  file (docs/stopping/retrospective_<date>.md). Plan approved by the user 2026-09-02: (1) rated drive + (4)
  identification drive are the user's; everything else proceeds in the order of the retrospective.
- 2026-09-02 ATTRIBUTED-SAFETY STEP -- design red-team (sol xhigh, run 20260902-192003) ADOPTED: live removal of
  a_plan is BLOCKED as specified; SHADOW-ONLY first ship approved. Evidence: raw MPC trajectory binds below the
  governor on 12.2% of in-band stopped-lead frames (p50 0.34, p90 0.65); post-lane aTarget 12.4% -- the leak is the
  MPC's close-range preference, not the comfort lanes. Blockers: (B1) a_kin/a_bar use instantaneous closing speed
  and cannot see lead DECELERATION -> add a_pred (response-time rho + adverse lead braking B_LEAD_MAX; deepen
  immediately, persistence only on release) and a_other (leadTwo limiting / independent model stop / FCW);
  (B2) fresh, cut-in, vision-only leads need TRUST-IN: keep a_plan in the min until measured-gap + track-motion
  maturity. NEW P1 INVARIANT: "single comfort authority with attributed safety deepening" -- during a governor-owned
  lead approach with finite governor, measured mature gap and trusted identity/motion, target = min(governor/
  terminal reference, attributed safety lanes only: a_kin, a_bar, a_mon, a_pred, a_other, actuator safety);
  post-lane aTarget and unattributed aTargetTrajectory are NOT min-inputs; limiter may retain a deeper previous
  command for continuity; scope APPROACH_GLIDE/PRE_STOP_EASE only; FAIL-CLOSED when any input is non-finite,
  gap_source != measured, dropout, fresh/replaced track, or attribution unavailable (retain current a_plan path;
  never shallower than the previous wire on a fault frame). SHADOW: one flag off/shadow/live (NOT overloading
  SERVICE_APPROACH_LAW); candidate = min(a_phase, a_kin, a_bar, a_mon, a_pred, a_other) computed every owned frame
  while the wire keeps the current target; telemetry per settle = binding counters (plan-bound frames, unexplained
  binds = plan_bound and every attributed lane > a_plan + 0.10, released depth, barrier transitions, lane toggles)
  + a bounded full-rate ring of plan-binding frames (my adaptation of sol's 100 Hz per-frame ask: rlog budget).
  FLIP GATE: >= 100 held-out stops over >= 5 routes; >= 90% of plan-binding ordinary frames and >= 95% of released
  depth UNEXPLAINED by any safety lane within 0.10; zero candidate shallowing on fresh/unmeasured/dropout frames,
  when aLeadK < -0.3, leadTwo limiting, model-stop conflict; zero added barrier episodes / lane toggles; synthetic
  cells (hard brake, reversal, cut-in, dropout, identity replacement) keep the 3.0 m floor; entry release <= 0.015
  m/s^2/frame. Falsifiers: later barrier binding, shorter rests, late re-brake, more J_SAFE pulses, driver braking
  after a released frame. DELETIONS: none in the first ship (wire-dead != deletion-safe: takeover seed, fallback,
  handback, MPC trajectory generation still consume those lanes); post-gate order: roll-in floor + latch/oncoming
  state -> low-speed stopped-lead cap/creep extension -> longcontrol low-speed caps -> REST-CLOSE (after proving
  trajectory/shouldStop/entry-timing redundancy). Head-band lanes stay until V_OWN moves. Replay must go through
  the real StoppingService.update() at 100 Hz, not governor_demand alone; stop_harness stays synthetic-only until
  the identification drive validates the plant.
- 2026-09-02 ATTRIBUTED-SAFETY SHADOW SHIPPED (550b3446 + R1 fix): candidate = min(a_phase, a_kin, a_bar, a_mon,
  a_pred, a_other) inside governor ownership (APPROACH_GLIDE/PRE_STOP_EASE), wire untouched (byte-identity pinned
  off/shadow/raising). ELIGIBILITY (fail-closed, reason recorded): unusable inputs -> "unusable"; dropout or
  gap_source != measured -> "gap"; motion not earned or REAL identity age < 0.5 s -> "identity" (an identity-less
  vision-only lead NEVER matures: track_age_s is 0 without a radarTrackId -- review R1); FCW active -> "fcw";
  aLeadK < -0.3 -> "lead_braking". a_other = leadTwo kinematic (D_HARD) + predictive lane on leadTwo geometry, and
  an independent model stop when dts < d_gap - (4.0 + ISD). Telemetry per settle: attr_frames, attr_ineligible,
  attr_reasons{}, attr_plan_bound, attr_unexplained (released > 0.10), attr_released_sum, attr_pred_bound, and the
  bounded plan-binding ring (t, v, gap, a_plan, a_phase, a_kin, a_barrier, a_pred, candidate, wire). The flip gate
  (above) reads these; the index will report them per settle. Approach change vs the red-team spec: FCW is an
  ineligibility veto rather than an a_other demand (FCW is an alert at this seam, not a demand).
- 2026-09-02 ATTRIBUTED-SAFETY SHADOW, review closed (R2, my sign-off per the two-round rule): approach changes
  -- (a) MODEL-STOP PROVENANCE: longitudinalPlan.distanceToStopTarget is lead-derived and carries no e2e
  provenance; the planner now publishes distanceToStopTargetModel (the e2e trajectory's own stop point: first
  sample below 0.5 m/s while action.shouldStop, -1 when none) and the shadow's a_other reads THAT; a missing or
  non-finite value makes the frame "unusable". (b) The predictive lane's response-interval acceleration is the
  MEASURED a_ego (the service's prior command is not the wire in observer frames; the legacy seam value is not
  the wire in owned frames). (c) Every hazard input (aLeadK, leadTwo v/d, model stop) fails closed to "unusable"
  when non-finite. Deployed; the flip gate collects from the next drives via the index's attr_* columns.
- 2026-09-04 MECHANISM OF "GENTLE THEN HARSH" (user feedback on the 09-02/09-04 ride; measured on routes 2045-205d):
  late-brake census (tools/stopping/review/late_brake_census.py): 75% of engaged stops brake <= -1.2 m/s^2 inside
  the last 3 m/s (late peak p50 -1.28 at 1.5 m/s when service-owned, -1.43 at 2.9 m/s when planner-owned).
  Handoff geometry: the planner delivers v~2.4 m/s at a MEDIAN GAP OF 7.4 m (p10 5.5, p90 9.7) where the governor's
  reference speed is 1.23 m/s -> the law pursues its curve with TAU 0.8 and demands -1.08 p50 / -2.0 p90 AT ENTRY;
  the kinematic-to-anchor decel from that state is already ~-0.9 to -1.8, so capping the law at the anchor decel
  only moves the in-band deepest demand p50 -1.53 -> -1.20 (p90 unchanged): the arrival itself is hot. ARRIVAL
  CENSUS vs the comfort curve (gap_ref(v) = inverse sqrt law + anchor + lag): deficit p50 -30 m at 6 m/s, -13 m at
  4 m/s, -7 m at 3 m/s, -4.8 m at 2.4 m/s; 100% of approaches hot (< -2 m) at every speed >= 2.4. CONCLUSION: no
  in-band law can be gentle from a hot arrival; the whole approach must follow ONE curve -> NEXT STRUCTURAL STEP =
  the WHOLE-APPROACH GOVERNOR (planner-side: the stopped-lead approach target becomes the governor's demand from
  the moment a stop is committed, MPC/lanes demoted to attributed deepen-only safety), superseding "V_OWN 4.5".
  Open design questions for the red-team: A_C for the head (0.6 = 67 m of braking from 8 m/s; human ~1.0-1.5 ->
  25-35 m) vs the 0.6 terminal; where ownership starts (stop-commit certainty); what the MPC still owns.
- 2026-09-04 WHOLE-APPROACH GOVERNOR -- design red-team (sol xhigh 20260904-203955) ADOPTED: MODIFY, SHADOW ONLY.
  CURVE: speed-dependent sqrt profile A(q) = 0.60 + 0.60*q^2/(q^2+2.5^2) (0.6 terminal -> 1.2 head), D(q) = R + L*q +
  tau*q + integral(u/A(u)), R = 4.0+ISD, L 0.45, tau 0.80; invert D(q_ref)=gap; a_ff = -q_ref/(tau + q_ref/A(q_ref));
  a_raw = clip(a_ff + (v_L+q_ref-v)/tau, -2.5, 0.5); ONE comfort slew J_DOWN 0.60 / J_UP 0.80 (safety bypasses).
  Braking distance from 8 m/s 45 m (was 68), from 11 m/s 73 m (was 119); capture reserve D_capture = q*A(q)/(2J)
  (+7.6 m at 8, +10.7 m at 11). COMMITMENT (BLOCKER: never the stop-aim gate, it arms hot): one `whole_commit`
  certificate -- common gates (Santa Fe HEV, blended, engaged, no force coast, finite raw lead, measured gap, REAL
  radar identity, track_age >= 0.5 s, motion earned, modelProb >= 0.5, no leadTwo conflict, no FCW; identity-less
  and radar-only leads stay with the MPC in v1); stopped branch = lead_confirmed_stopped 0.3 s; stopping branch =
  same track 0.5 s, aLeadK <= -0.75 (least-severe 0.3 s estimate), vLead < vEgo; ARM when g_stop = dRel +
  max(vLead,0)^2/(2 b) <= D(vEgo) + D_capture + 0.5*vEgo; RELEASE on physical departure (gap +0.30 m and pull-away
  > 0.5 m/s for 0.5 s) or planner go > +0.2 for 0.2 s with non-closing lead evidence; shouldStop=False alone never
  releases; track loss/replacement/reversal/non-finite -> rate-limited release, re-entry needs a new 0.5 s
  certificate; raw vLead < -0.25 rejected before any clamp. SEAM: a small pure stopping_governor.py consumed by
  planner (publishes the whole-approach net target above 2.5) and service (terminal/hold/release/handback below
  2.5, StopContext observing above); reseed_takeover keeps C1; seam gates: zero step on the first service frame,
  ordinary wire steps <= 0.006/frame, measured 0.3 s jerk <= 0.8. Service wire ownership is NOT extended upward
  (PID/feedforward + tracking trim stay). SAFETY INVARIANT: a_target = min(a_comfort, a_kin, a_bar, a_pred, a_other,
  a_mpc_hazard); post-lane aTarget is never a safety input; aTargetTrajectory stays a hazard lane while the lead is
  moving/braking/fresh/replaced or attribution is incomplete; safety may exceed the 0.8 bar (reported separately).
  INTERACTION: extends attributed safety (same candidate + reason codes above 2.5), replaces the V_OWN 4.5 pre-band
  shadow, live flip waits for the attributed gate. GATES: before SHADOW -- offline replay of >= 50 stopped-lead
  stops (15 from >= 8 m/s, 10 moving-to-stop), flag-off equality, zero commits on disengaged/manual/force-coast/
  no-lead/crossing/phantom frames, 100% of >= 6 m/s stops certified by the capture boundary, <= 1 entry + 1 release,
  candidate rates deepen <= 0.60 / release <= 0.80, >= 90% of hot stops move demand earlier (>= 0.15 deeper above 4,
  >= 0.15 shallower in 0.8-3 m/s). Before LIVE -- attributed gate met; >= 30 starts from 8 m/s, 20 moving-to-stop,
  10 departures; IDENTIFICATION DRIVE completed (plant p90 aEgo error <= 0.20, rest error <= 0.35 m); counterfactual:
  deficit p10 >= -1.0 m, a_late <= -1.2 on <= 5%, late-ratio >= 2 on <= 5%, modelled felt <= 0.65, rests 95% in
  3.5-5.0 none > 5.5, zero added barrier/J_SAFE/driver-brake/multi-descent; synthetic grid incl. hazard cells (no
  response later than 0.05 s, min gap >= 2.0). FIRST SHIP: flag WHOLE_APPROACH_GOVERNOR off|shadow (no live), pure
  helper, whole_commit + release state reusing StoppingLeadAuthority/StopContext/stop-commit provenance, publish
  wholeApproachDemand + reason + safety candidate, bounded per-approach telemetry, wire unchanged, tests. DELETION
  ORDER after live: planner comfort lanes (decel-lead feedforward, lead caution, smooth-approach caps, late-approach
  tables, far-lead confirmation, downhill clip) -> stop-aim floor demand (keep its provenance as the certificate) ->
  stop-commit floor after a_bar/a_pred/a_mpc_hazard equivalence -> longcontrol PID approach caps -> (after attributed
  live) roll-in floor, REST-CLOSE, low-speed cap family, legacy stopping floor -> flags/state/tests/telemetry.
- 2026-09-04 WHOLE-APPROACH SHADOW implemented: `WHOLE_APPROACH_GOVERNOR = "shadow"` adds the pure adopted
  speed-dependent profile, capture reserve, inverse and comfort slew; a planner-only `whole_commit` certificate
  uses raw lead geometry, isolated `StoppingLeadAuthority`/`StopContext` trust, the existing stop-commit persistence
  and provenance helpers, the exact stopped/stopping dwells, capture-boundary arm, and departure/go/identity release.
  Above 2.5 m/s it publishes `min(a_comfort, a_kin, a_pred, raw MPC)` plus the certificate reason, safety minimum and
  stopped-lead curve deficit; NaN is the unavailable Float32 sentinel. One `whole_approach` cloudlog event closes
  each committed approach with bounded counters. The calculation runs after final planner control outputs and has no
  control consumer; off and shadow leave all wire values byte-identical. Deviations: the terminal service stays
  unchanged for this shadow-only step, and active FCW uses the additional reason `fcw` so it is distinct from the
  specified fail-closed `fcw_unavailable` case. The documented >=50-stop offline replay gate was not rerun in this
  code-only implementation cycle; pytest/static validation is complete, but live/corpus validation is not claimed.
- 2026-09-04 cycle 42, takeover review IN PROGRESS: the first implementation is not ready for shadow
  activation. Recorded-input replay of 405,887 planner frames over 14 indexed routes found 1,089 committed
  standstill frames and 956 committed gas-override frames. The supplied tuple-equality test did not execute
  planner update/publish; it has been replaced with real update/publish comparison (native solver substituted),
  including a throwing shadow and other-car cases. The before-SHADOW gate is NOT met; default stays OFF and no
  LIVE law, gain, terminal behavior or legacy lane is changed. Correct certificate/reset defects first, then
  repeat replay and the two-round review. Replay uses published clipped aTargetTrajectory as an explicit proxy
  for the unpublished raw MPC input; it cannot prove counterfactual rest, feel, or late-demand improvement.
  The prior claim that the 7 m bookmark "would" rest at 4.3 m is withdrawn: that needs a validated plant.
- 2026-09-04 cycle 42 correction: one common certificate gate now governs entry and retention: no driver
  override, no standstill/wheel-stop, measured non-dropout gap, finite age, and a fresh 0.5 s same-track window.
  The stopped branch also requires the existing conditioned stopped classifier. R1 reproduction showed that
  `gap_source=measured` can still carry an outward-rate-limited gap: raw 27 m versus conditioned 19.225 m.
  Candidate, arm and departure geometry now use that conditioned gap; checking the provenance label and then
  using raw dRel would bypass the filter. No new filter or persistence layer was added. The below-band shadow
  seed is cleared so a return above 2.5 starts from the current planner demand. These changes are shadow-only.
- 2026-09-04 cycle 42 R1 CLOSED, request changes (sol xhigh 20260904-212702-exec): accepted all six findings
  in retrospective_2026-09-04.md. Fixed stale-radar certificate earning by requiring updated/valid/alive radar;
  replay uses distinct timestamps with <=100 ms age. Gate failures (capture, chatter, incomplete safety set,
  missing below-band/counterfactual evidence) remain explicit blockers: keep OFF. New priority before any wire
  feature: a shared fail-closed lead-input boundary for the pre-existing far-lead/roll-in NaN/Inf failures.
  Do not patch those two legacy lanes individually. This is a safety-boundary task, not a comfort-law retune.
- 2026-09-04 cycle 42 REVIEW CLOSED: sol xhigh R2 `20260904-215628-exec` found no new blocker to landing
  DISABLED research code. Main-agent sign-off adopts that scope; no third round. `WHOLE_APPROACH_GOVERNOR=off`;
  existing SERVICE_APPROACH_LAW=governor and ATTRIBUTED_SAFETY=shadow remain unchanged. Final replay: zero excluded
  commitments, 315 entries/releases, high-speed capture 3/13, multiple entries 40/54 candidate stops; activation
  gate FAILED. Tests 897 passed/19 skipped. Full A-I decisions: retrospective_2026-09-04.md. Next wire priority:
  shared non-finite lead-input boundary; next governor design: coherent certificate lifetime, complete hazard
  attribution and a valid pre-shadow test distinct from plant-dependent outcome gates.
- 2026-09-04 cycle 42 DEPLOY VERIFIED: `18ceaea851`, new governor OFF; device `manager.py` running after reboot,
  tracked files clean, existing governor/attributed-shadow flags unchanged. GitHub SSH deploy key was rejected;
  documented bundle fallback succeeded and original GitHub remote was restored. No on-road validation claimed.
- 2026-09-05 CYCLE 48 -- direction decision on resuming after the handback: the offline full-stop model has now
  been rejected twice on closed-loop data (cycle 45: +6.3/+1.4/-18.3 m; cycle 47: the wheel-filter coupling
  explains none of it), which is the cycle-34 verdict again (closed-loop stop data cannot identify the plant).
  DECISION: no further plant fitting on closed-loop logs. The identification drive (protocol v1 above) is the
  path to gates 3/4, and its prerequisite is the temporary, flag-gated brake-demand STEP HOOK that cycle 43
  deferred "until existing data is shown to lack an identifiable response" -- that is now shown. Next: red-team
  the hook design (safety envelope, trigger, abort, logging), implement it off-by-default, two-round review,
  deploy disabled, then the user drives the protocol. In parallel: keep collecting the attributed-safety gate from
  unique raw events (tools/stopping/review/attr_gate_tally.py reproduces cycle 42: 28 events, 19.8% of eligible
  frames plan-bound, 57.2% unexplained) and re-examine the gate's 90%-unexplained MATERIALITY bar in the same
  red-team (explained binds are neutral to remove: the safety lane binds instead; the bar decides whether the
  live step is worth doing, not whether it is safe). Whole-approach stays OFF (cycle 42). Rated drive still owed.
- 2026-09-05 IDENTIFICATION STEP HOOK -- design red-team (sol xhigh 20260905-152855) ADOPTED: MODIFY. Protocol v1 was
  unsafe (a 3 s -2.5 step from 8 m/s nearly stops the car). PROTOCOL v2: start each trial at 10-11 m/s; steps -0.5/
  -1.0/-1.5 for 3.0 s, -2.0 for 2.5 s, -2.5 for 2.0 s; ramps -0.5->-2.0 and -2.0->-0.5 over 3 s (separate trials);
  crossing -0.8 for 1 s then -2.2 for <= 2 s; every command ends early at vEgo <= 4.5 m/s, hard abort floor 4.0;
  NO positive commands (the driver recovers speed manually); 8 trials/repetition (~2.5-3.5 min, 1-1.5 km), 3
  repetitions per road condition/direction (~8-11 min), depths counterbalanced across repetitions; flat and grade
  runs labelled by direction (grade is metadata, not a gate). TRIGGER: in-code master IDENTIFICATION_HOOK=False;
  when deployed for the collection, arm requires /data/identification_hook.arm (created by SSH before the drive,
  latched at process start, never polled while driving), the long-press distance-button mapping must be NOTHING,
  a deliberate 1.5 s distance-button hold + release starts ONE trial, any second press/brake/gas/cancel/
  disengage aborts; ignition/process restart clears the arm; no Params toggle, no touchscreen control, NEVER time-
  based auto-run. INDICATION: comma-screen banner + distinct tones (ARMED / ACTIVE cmd+time / ABORTED reason /
  COMPLETE recover); no cluster claim. INJECTION: inside LongControl.update as the FINAL command owner -- after
  PID, Santa Fe caps/trim, StoppingService takeover, force-coast/hold writers; before final clip, last_output_accel
  and return; PID integrator frozen and reseeded (pid.i = out - (p+d+f)) every active frame; tracking-trim learning
  disabled; downstream max_desired_acceleration/gas/Hyundai clip/panda unchanged. PRECONDITIONS (2.0 s continuous):
  exact Santa Fe HEV 2022 fingerprint; armed; longActive + openpilot longitudinal + ACC engaged; LongCtrlState.pid;
  7 <= v <= 11; carState/radarState/modelV2/longitudinalPlan/livePose valid+fresh; leadOne/leadTwo status false;
  both model lead probs < 0.10; no hasLead/FCW/AEB/radar error; no stop request/shouldStop/stop target; no gas/
  brake/force coast/override; |steer| <= 5 deg, |yaw rate| <= 0.03, no blinker/lane change/steer fault; ESP/ABS
  inactive; ACC not faulted; CAN valid; Drive gear. ABORT (same frame) on any lead/model prob >= 0.10, FCW/AEB,
  pedals/override/cancel/disengage/force coast/second press, v < 4.0 or > 12.0, steer/yaw/blinker/ESP/CAN/ACC faults,
  stop request/standstill/gear, non-finite/stale, phase or trial deadline, any exception (disarms for the drive).
  HANDBACK: pedal/disengage paths keep immediate authority; else out = min(normal_chain, last_hook + 0.8*dt) (safety
  deepens immediately, release limited to 0.8 m/s^3), PID reseeded each handback frame, StoppingService
  reseed_takeover when eligible, trial latched aborted, never auto-resume. TELEMETRY: minimal 100 Hz fields on
  controlsState (armed/active, trial+phase, scripted accel, hook output pre-limits, handback state, abort enum) +
  cloudlog at arm/start/phase/complete/abort; fit against carOutput.actuatorsOutput.accel (SCC14 jerk limits reshape
  steps) with the scripted command retained. LIMIT: the regen/friction split is NOT observable from CarState
  (brake=0 TODO, regenBraking always false) -> the fit identifies a command-depth regime change only, until raw CAN
  hydraulic/regen signals are identified. HAZARDS: SCC spoof handoff/reboot paths untouched, hook inhibits update
  activation while armed/active; ABS/ESP -> abort; following traffic is the driver's check. IMPLEMENTATION: pure
  identification_hook.py (sequence/state/envelope/abort), one final-writer call in longcontrol, controlsd passes
  validity/buttons + publishes, minimal cereal fields, banner/tones, one test file (arming, every precondition and
  abort, same-frame lead abort, exact 100 Hz timing, speed floor/timeouts, scripted == pre-limit output, PID/trim
  no wind-up, deeper safety wins in handback, release bound + service reseed, flag-off equality over real
  LongControl.update, exception latches off, Hyundai command equality). DELETION: after the drive (routes frozen,
  3 repetitions verified, fit + free rollout validated OR collection rejected) remove the hook module, arm file,
  UI, cereal fields, tests and calls in the same step; keep only the corpus, tools, fitted artifact, evidence.
- 2026-09-05 ATTRIBUTED-SAFETY GATE REVISED (same red-team): the 90%/95%-unexplained rule mixed materiality with
  safety. NEW GATE -- Cohort: >= 100 eligible driver-free held-out stops over >= 5 routes incl. all known cut-in/
  replacement/re-slam/moving-to-stop/leadTwo/model-stop-conflict/departure cases; report unique and excluded
  events. Materiality (either): (1) unexplained binds > 10% of all eligible owned frames AND in >= 30% of eligible
  stops AND each affected stop has a release > 0.10 sustained >= 0.30 s; or (2) >= 25% of eligible stops have
  unexplained released impulse sum(max(candidate - a_plan - 0.10, 0))*dt >= 0.05 m/s; report p50/p90 depth and
  impulse; explained binds are NEUTRAL. Immediate safety veto: zero candidate shallowing on fresh/unmeasured/
  dropout/replaced identity, non-finite/stale, FCW/AEB, aLeadK < -0.3, leadTwo limitation, model-stop conflict,
  barrier activity, driver override, unavailable attribution; fault frames never release above the previous wire.
  Late-rescue veto: for every unexplained release > 0.10, in the following 2.0 s on the same identity there must
  be zero cases where a_kin/a_bar/a_pred/a_other/a_mpc_hazard becomes >= 0.10 deeper than the released candidate,
  zero new barrier/FCW events, zero driver braking. Outcome/synthetic veto: zero added barrier episodes, late
  re-brakes, J_SAFE pulses, driver-brake-after-release, min-gap regressions; hazard cells keep the 3.0 m ordinary
  and 2.0 m hard floors. Decision: materiality authorizes a bounded live evaluation only after the plant and
  whole-approach gates pass; any safety veto blocks regardless. Observed so far (28 unique events): 11.3% of
  eligible frames unexplained, 11/28 stops -> materiality is plausibly met; the vetoes need the ring/rlog check.
- 2026-09-05 IDENTIFICATION STEP HOOK IMPLEMENTED (cycle 48; off by default): selfdrive/controls/lib/identification_hook.py
  (pure: protocol-v2 schedule, envelope, state machine, handback bound), one final-writer call in LongControl.update
  (after every cap/service/hold writer, before the final clip; PID integrator reseeded and the tracking trim zeroed
  while the hook owns the wire; fault frames never reach it), controlsd builds the validated inputs and shows the
  banner through the existing alertDebug -> "longitudinal maneuver" alert path (upstream maneuversd's banner; logged
  in the rlog), master flag stopping_flags.IDENTIFICATION_HOOK = False. DEVIATIONS from the adopted design, with
  reasons: (1) no new controlsState fields -- the scripted command IS the wire (carControl/carOutput) and the phase/
  reason/trial go to alertDebug text + cloudlog transitions (ladder: minimal); (2) banner text only, no distinct
  tones and no cluster indication (would need new selfdrived events; the existing maneuver alert has none);
  (3) no StoppingService reseed on handback -- the 4.0 m/s abort floor keeps every trial above the 2.5 m/s service
  band; (4) the 2.0 s precondition clock must already be satisfied when the button press STARTS (stricter than
  "before accepting the start"). Tests: 47 (every precondition and abort reason, exact 100 Hz timing, ramps and
  crossing, speed floor, hold-and-release trigger, second-press/pedal/disengage aborts, release bound with deeper
  normal winning at once, trial cap, exception latch, flag-off/unarmed byte-identity over real LongControl.update,
  PID reseed). COLLECTION PROCEDURE (user): (a) one deploy with IDENTIFICATION_HOOK = True; (b) in FrogPilot set BOTH
  distance-button long-press mappings (Long, Very Long) to Nothing; (c) before the drive, over SSH:
  `touch /data/identification_hook.arm`, then start the car (the arm is latched at process start); (d) on an empty
  straight road, engaged, 10-11 m/s, no car ahead, hold the distance button 1.5 s and release -> ONE trial runs and
  the screen shows STEP TEST ACTIVE; recover speed by yourself; repeat for 8 trials x 3 repetitions (the order is
  automatic); any pedal, lead, or second press aborts; (e) after the drive: `rm /data/identification_hook.arm` and a
  deploy with the flag back to False. The hook module, wiring and tests are DELETED in the step that consumes the
  fitted plant.
- 2026-09-05 ATTRIBUTED-SAFETY LIVE -- design red-team (sol xhigh 20260905-160050) ADOPTED: MODIFY before LIVE. User
  directive: "continue until we have a fix to deploy that will improve the stopping quality". SEMANTICS: current_target
  = min(a_phase, a_kin, a_plan, a_mon, a_bar) as today; candidate = min(a_phase, a_kin, a_mon, a_bar, a_pred, a_other);
  live_active only when the flag is "live" AND eligible AND eligibility has held 0.30 s continuously on the same
  identity; then target = min(candidate, last_cmd + 0.8 m/s^3 * dt) (a_plan's excess is RELEASED at <= 0.8 m/s^3, never
  stepped); any ineligible frame re-admits a_plan IMMEDIATELY (fail-closed; the limiter deepens at J_SAFE) and resets
  the timer; A_DROPOUT_MIN applies after selection; safety_binding = target < a_phase unchanged (a_plan is not a
  safety lane); phases APPROACH_GLIDE/PRE_STOP_EASE only (a_plan returns at RAMP_TO_HOLD through the limiter); an
  exception in the live computation selects current_target. Telemetry: attr_live_frames, attr_live_release_frames,
  attr_live_released_sum, attr_live_max_release, attr_eligibility_flips, attr_live_reentries (rescue/barrier episodes
  are evaluated offline by attr_veto_eval.py -- deviation, minimal). OFFLINE EVIDENCE (36 unique events, routes
  2072-207e, tools/stopping/review/attr_veto_eval.py): materiality -- 11/36 stops with unexplained releases, 7/36
  sustained >= 0.30 s, 6/36 with released impulse >= 0.05 m/s (max 0.347); the formal thresholds (> 10% of eligible
  frames: observed 8.5%; >= 25% of stops with impulse >= 0.05: observed 17%) are NOT met, but every top-impulse stop
  is a LONG REST (6.87, 7.66, 8.31, 5.74, 5.40, 6.32, 5.25, 5.50 m) -- the release targets the wasted-room class of the
  user's rest rule directly. Safety vetoes on the same data: after every unexplained release, ZERO a_kin/a_bar
  rescues within 2.0 s, zero FCW, zero driver braking, zero lead loss/replacement; the two counted lanes are not
  rescues (aTargetTrajectory staying deeper IS the released demand; a_pred deepening later is a candidate lane doing
  its job). BOUNDED EVALUATION (cohort gate NOT met; user-authorized): first 10 driver-free settles over >= 2 routes --
  immediate revert to "shadow" (one word) on FCW/driver braking/takeover within 2 s of a live release, any a_kin/a_bar
  rescue >= 0.10 deeper within 2 s, any new barrier episode after a release, any rest < 3.5 m on a released stop, a
  felt release-then-re-brake pulse, more than one fail-closed re-entry per settle without a proven identity change or
  hazard, any incomplete stop/relaunch/fault/fallback; continue if all released stops rest in 3.5-5.0 m with zero
  rescues and zero interventions; then 30 settles / >= 3 routes under the same rule. This is evaluation evidence,
  not the formal >= 100 / >= 5 gate.
- 2026-09-05 ATTRIBUTED-SAFETY LIVE, review R1 (20260905-160656) -- APPROACH CHANGE: LIVE is RELEASE-ONLY. The adopted
  formula target = min(candidate, last + 0.8*dt) could DEEPEN below today's target through a_pred/a_other (an
  equal-speed crawler at 6 m: current -0.03, candidate -0.51) -- new braking with no on-road evidence (the shadow's
  attr_pred_bound is 0.23% of eligible frames). Now target = max(current_target, min(candidate, last + 0.8*dt)):
  the planner's excess is released, the attributed lanes only bound how far, and their own deepening stays shadow
  evidence for a later step. Also: every fallback frame (non-finite inputs) clears the live dwell (R1 MEDIUM).
- 2026-09-05 BOUNDED LIVE EVALUATION -- how it is judged on the next drives (cycle 49 onward): after each drive run
  `stop_index.py --since <last>` and `attr_gate_tally.py` + `attr_veto_eval.py` on the new rlogs; per released stop
  read attr_live_frames / attr_live_release_frames / attr_live_released_sum / attr_live_max_release / attr_eligibility_
  flips / attr_live_reentries and the rest gap; REVERT (flag "live" -> "shadow", one commit) on any of: FCW, driver
  braking or takeover within 2 s of a live release; any a_kin/a_bar rescue >= 0.10 deeper within 2 s; a new barrier
  episode after a release; a rest < 3.5 m on a released stop; a felt release-then-re-brake pulse (user report or
  reversal census); > 1 fail-closed re-entry in one settle without a proven identity change; any incomplete stop,
  relaunch, fault or fallback. CONTINUE only if every released stop rests in 3.5-5.0 m with zero rescues and zero
  interventions; 10 settles / >= 2 routes first, then 30 / >= 3. The identification hook stays OFF on this deploy.
- 2026-09-05 DEPLOYED: ATTRIBUTED_SAFETY = "live" (cycle 48 deploy; user directive "a fix to deploy that improves the
  stopping quality"). What changes on the road: inside the governor's band, behind a mature measured stopped/slow
  lead, the planner's comfort demand no longer deepens the wire below the governor's curve -- the excess is released
  at <= 0.8 m/s^3 after 0.30 s of clean eligibility; any doubt (fresh identity, unmeasured gap, braking lead, FCW,
  second obstacle, model stop, non-finite input) keeps today's behaviour at once. Expected: the late in-band
  over-brake and the long rests behind stopped crawlers (the 7 m bookmark class) shrink; nothing changes above 2.5
  m/s, in the terminal/hold phases, or on ineligible frames. Bounded evaluation per the rule above; REVERT = the word
  "shadow" in stopping_flags.ATTRIBUTED_SAFETY. The identification hook stays OFF.
- 2026-09-05 LIVE EVALUATION, drive 1 (cycle 49; routes 2080-2085): 4 settles / 3 routes with attr data, live ACTIVE on 3,
  release frames 0 on all -- the wire equalled shadow. Cause: eligibility chatter (flips 12/6/8, fail-closed re-entries 2/2/3
  per settle; reasons identity + gap). The re-entry revert line is met literally with zero wire effect; LIVE stays on, and the
  chatter is the next work item because it also withholds the intended release (2082 s0: 0.4 s of ~0.2 m/s^2 planner excess
  went unreleased). No FCW, no driver braking, no rescue, rests 4.20-4.46. Separately, bookmark 2085 s2 is a NO-LEAD
  experimental-mode model stop (vision lead vanished, model planned v->0 in ~1.5 s, service ownership only at v=0.13):
  outside the attributed step; the open no-lead class.
- 2026-09-05 CYCLE 49 -- APPROACH CHANGE: attributed-safety gap trust = the service's gap_live predicate. The live
  step's eligibility required gap_source == "measured"; the gap filter runs at 100 Hz on a 20 Hz radar reading, so
  a constant reading (4.2 m, quantised) sits millimetres OUTSIDE the ego-propagated prediction on every new sample
  and the filter answers with an OUTWARD persistence hold (min(prediction, raw)) for up to 25 frames. Offline
  StopContext replay of the live-build stops (tools: /tmp/ctx_replay.py, recorded in the worklog): in-band
  ineligible frames 197/551 (13 dwell resets) on 2082 s0, 64/327 (5) on 2085 s3, 106/240 (8) on 2081 s3 -- ALL
  outward holds; with gap_live 0/0 on each. The floor-defence cap (cycle 15) and the planner-safety lane (cycle 20)
  already trust exactly this hold, with the recorded reasoning: the outward hold emits a LOWER BOUND on the true
  gap, so every attributed lane computed from it is deeper (more conservative), and the release is bounded harder,
  not softer. Inward-rejection holds (optimistic prediction), invalid-reading holds and dropout decay stay
  ineligible. Change: one predicate in the eligibility chain (reason "gap" = not gap_live); two tests (an outward
  hold keeps eligibility and the live release; an inward-rejection hold re-admits a_plan and restarts the dwell).
  Expected on the road: the release finally accrues on the stops where the planner binds in band (2082 s0 had
  0.4 s of ~0.2 m/s^2 unreleased under the holds). The bounded-evaluation rule and revert word are unchanged.
- 2026-09-05 CYCLE 50 -- APPROACH CHANGE (user directive: the bookmarked no-lead force-coast stop 2085 s2 must
  improve): NO-LEAD MODEL STOPS JOIN GOVERNOR OWNERSHIP + THE ATTRIBUTED STEP, stage A. Anatomy of 2085 s2 (rlog):
  Experimental mode, a vision-only lead (no radar id) held a 2.1 m/s crawl; the model's plan changed abruptly at
  12.0-12.5 s (its stop point moved ~3 m closer), the MPC tracked it to -1.37 m/s^2 in 0.6 s (jerk 1.6), force coast
  was on but its brake limit is min(cap, model_accel) = pass-through; the model's action.shouldStop bit fired only
  0.21 s before the wheel stop (v = 0.22), so the planner's shouldStop, distanceToStopTargetModel (gated on that
  bit) and the service's entry (should_stop AND (model bit OR force coast)) all came at v = 0.2; the service owned
  the last 0.25 s; wire -1.2 into the wheel stop, a_wheelstop -0.60, jerk 11.8, felt 2.6. CENSUS (nolead_census.py,
  routes 2040-2085, 4 engaged no-lead stops): the model bit fires 0.16-0.31 s before the wheel stop at v 0.21-0.25 on
  EVERY one; the trajectory (planned v < 0.5 m/s within 4 s) shows the stop 1.1-3.5 s before; the deep demand starts
  1.75-5 s before; a_wheelstop -0.74..-1.4, jerk 3-12 -- the service never owns this class today. No new comfort
  law: the deployed no-lead glide (a_phase = -v^2/(2*max(d_rem, 0.15)) - a_coast with d_rem = max(model stop,
  v^2/(2*A_SETTLE_REF 0.40))) and the terminal ease/hold already govern no-lead stops the service enters early
  enough (stop signs). Stage A = let it enter on time and let it own the wire:
  (1) planner: model stop INTENT from the e2e trajectory -- planned speed < MODEL_STOP_V_MPS (0.5) within
  MODEL_STOP_INTENT_HORIZON_S (4.0 s) -- ORed into output_should_stop in blended mode exactly like the model bit
  (additive only: it can never remove a stop; the falling-edge hold covers de-assert), and distanceToStopTargetModel
  from the same trajectory condition without the bit gate (it can only bring the model-stop provenance EARLIER;
  its only consumer today, the lead-class a_other lane, deepens the candidate, i.e. bounds a release harder);
  (2) service: the attributed step's eligibility gains a NO-LEAD branch -- v in band, service ownership,
  no lead, a finite model-stop provenance (msd >= 0, the trajectory says stop), no FCW; candidate =
  min(a_phase, a_mon) -- the model-stop kinematic lane is NOT a candidate here (adopted design 2026-08-29:
  NO stop-point escape in v1; the point is a stop line with meters of slack, a chase would re-create the
  terminal grab); LIVE stays RELEASE-ONLY (the wire never deeper than today's target, rises <= 0.8 m/s^3 after
  the 0.30 s dwell), so the planner's -1.37 is released to the glide's ~-0.5 and the terminal ease replaces the
  grab. Accepted drift: the car rests past the model's stop point by <= 3.0 m (logged per settle as
  attr_nolead_drift_m = msd at the wheel-stop latch, plus nolead entries / releases / intent lead time);
  the bound is a revert line. What stage A does NOT fix: the onset (planner -0.06 -> -1.2 in 0.75 s before the
  intent is visible); that is stage B = entry on sustained model DEMAND with the adopted jerk-bounded pursuit
  law, a new law that goes through the four gates. Gate 2 for stage A: (a) intent false-positive census over the
  recorded no-lead in-band driving (an intent that is not followed by a stop within 6 s = a spurious entry; the
  service's RELEASE phase hands back, cost = a gentle glide episode); (b) recorded-input replay of 2085 s2 and
  the census stops (the released wire at the recorded states, integrated drift); (c) the existing suites.
  Deployment: LIVE under the same bounded evaluation and the same revert word ("shadow"), because the release-only
  construction bounds the risk to the drift, and the class is too rare (4 stops in 46 routes) for a shadow phase
  to produce evidence. Red-team (sol xhigh) before implementation.
- 2026-09-05 CYCLE 50 -- STAGE A AMENDED by gate-2 evidence (before the red-team verdict; the red-team reviews this
  form too). (i) Recorded-input replay (tools/stopping/review/nolead_replay.py, the four census stops, context fed the
  car's fixed 0.01 s period and a neutral coast residual): entering on the trajectory intent puts the service in
  APPROACH_GLIDE 0.9-3.5 s earlier on every stop, BUT on 2085 s2 the service's own demand sat at its shallow clip
  (-0.03..-0.15): the vanished lead (radar ids came and went at 37-44 m) left the context in its 2 s DROPOUT HOLD with
  a decaying 33 m ghost gap, and _d_rem = ghost - rest = 33 m. The model stop is not a _d_rem candidate today (only
  the planner's lead-derived distanceToStopTarget is). Amendment A3: with no service lead, the model stop distance is
  a _d_rem candidate exactly like dts (max(msd, envelope)), so d_rem = min(ghost, model) and the glide is -0.40 - coast.
  (ii) A vision-only lead (radarState leadOne.status True, radarTrackId -1) is NOT a service lead (the certificate
  needs a radar identity) but IS the MPC's lead: releasing the planner's demand behind it would release lead-following.
  Amendment A1: the no-lead branch requires NO lead of any kind reported by radard (leadOne.status False and
  leadTwo.status False) -- new service inputs any_lead from longcontrol's raw lead_status / lead2_status; a vision
  lead keeps today's behaviour (planner in the min). (iii) Dropout hold: a lead that just vanished may still be there.
  Amendment A2: during signals.dropout_active the branch is eligible only if the held gap is far beyond the model
  stop (held d_gap >= msd + NOLEAD_DROPOUT_MARGIN_M 8.0), i.e. the vanished lead cannot be what the model stops for;
  otherwise ineligible until the hold expires (fail-closed). 2085 s2: held 33 m vs msd 5 m -> eligible.
  (iv) Intent census (model_intent_census.py, routes 2040-2085, engaged, in band, no radar lead): 13 trajectory-intent
  episodes; 8 are <= 0.1 s flickers at v ~0.50 where the plan's first points mirror the current speed (plan indices
  with t < 0.5 s are excluded from the intent: amendment A4a); of the 5 real episodes (>= 0.3 s) 4 were followed by a
  stop within 10 s and every one had the planner braking <= -0.67 (median -1.35); the one non-stop (207d s2, 1.24 s):
  force coast toward a queue behind a 28 m vision lead, force coast released, the model went back to a creep, the
  driver disengaged 2 s later -- under stage A the service would have glided at -0.40 for ~1 s where the planner
  coasted at -0.07, then RELEASED when the intent lifted (a 0.3 m/s slowdown; and with A1 that episode is ineligible
  anyway: a vision lead was present). Amendment A4: the service's entry intent must persist 0.30 s (rising edge) and
  the planner must be braking (a_target <= -0.30) on entry. The model bit itself fired 0 times in scope.
  (v) Containment (reviewer question 1): the intent does NOT enter output_should_stop; the planner publishes it only
  through distanceToStopTargetModel (>= 0 = the trajectory says stop within 0.5-4.0 s, no bit gate); longcontrol
  derives the SERVICE's should_stop = planner should_stop OR (intent persisted AND no lead of any kind AND v < V_ENTER
  AND a_target <= -0.30); the legacy chain (state machine, arbiter, force-coast writers, resume) sees no change.
  (vi) Amendment A5 (replay of the implementation): the entry persistence (0.30 s) plus the lead-branch dwell (0.30 s)
  put the first release 0.6 s after the intent (2085 s2: at v = 0.93 instead of 1.4); the no-lead branch's dwell is
  NOLEAD_LIVE_DWELL_S = 0.10 s (the persistence already established the intent; the dwell spans the takeover seed
  frames). The entry persistence is 0.20 s (the shortest real census episode was 0.65 s, flickers <= 0.1 s) and DECAYS
  on brief intent drops (2x rate) instead of restarting: the census shows 1-2 frame flickers at the intent's onset.
- 2026-09-05 CYCLE 50 -- DESIGN RED-TEAM VERDICT (sol xhigh, run 20260905-183943): NOT LIVE. Blockers and what was done:
  (1) CRITICAL -- with no radar lead the MPC's trajectory demand (aTargetTrajectory) can be the ONLY brake for a
  pedestrian / cyclist / untracked car (planner FCW needs model prob > 0.9; stock AEB is disabled with openpilot long
  on this car); releasing it toward the -0.4 glide can pass the obstacle by ~2.5-3 m at 2.5 m/s. APPLIED: the
  trajectory lane is a FLOOR of the no-lead candidate (candidate = min(a_phase, a_mon, aTargetTrajectory, force-coast
  level); an invalid trajectory lane = "unusable"); only the planner's excess ABOVE it is released. Census of the four
  recorded stops: that excess is 0.3-0.9 m/s^2 and largest into the wheel stop (2085 s2: -0.80 vs -0.48 at 13.6 s,
  -0.66 vs -0.25 at 13.8 s; 2041 s3: -0.95 vs -0.15), so the release still removes most of the terminal grab while the
  model's own stopping intent is never released. Residual risk the user must accept for LIVE: none from the release
  (it stops at the model's own demand); what remains is the stated class risk of every no-lead stop today.
  (2) HIGH -- a global output_should_stop OR is not additive-only: already contained (the intent reaches the service
  only through distanceToStopTargetModel and longcontrol's service-side entry); plan indices t < 0.5 s excluded (the
  reviewer's recorded launch false-assert at 2075 s1 t=117.5 is exactly that class).
  (3) HIGH -- early entry + the model-stop remaining distance are NOT release-only by themselves (they change a_phase
  before the release rule: v=2.0, planner/legacy -0.30, model stop 2 m -> a_phase -0.40 = new braking). APPLIED: a
  BASELINE INVARIANT on every frame of a no-lead-owned episode (episode latch set on the entry frame, kept until the
  service goes inactive): target = max(target, the legacy chain's wire this frame) in APPROACH/EASE/RELEASE; the
  terminal RAMP/HOLD keep the deployed secure hold. This also removes the crossing-car phantom brake the replay found
  (2041 s3: a radar object at 34 m, vLead -5.7, yRel 3 -> -1.2, model prob 0.36-0.5, radard's leadOne under the user's
  35% detection threshold; the service's a_kin lane would have braked -0.9 for 1.5 s while the MPC coasted). A
  separate flag NOLEAD_ATTRIBUTED_SAFETY ("off" | "live", default "off" = today's behaviour EVERYWHERE, the intent is
  not even published); the deployed lead-class flag is untouched.
  (4) HIGH -- force-coast hand-back pump (the FC ramp hidden under ownership reappears at hand-back): the force-coast
  level is a candidate floor (release never above it); the RELEASE-toward-zero on intent loss is bounded by the
  invariant (never shallower... i.e. never DEEPER than the legacy, and the legacy carries the FC ramp), so the
  hand-back lands on the legacy FC value; the reference-family freeze is stage B work.
  (5) HIGH -- three-state lead semantics + blended scope: raw lead present but uncertified = baseline (no release);
  no raw leadOne/leadTwo = no-lead evaluation; the intent requires experimental (blended) mode and a mode edge clears
  the persistence and the hold at once. APPLIED.
  (6) MEDIUM -- drift metric: the anchor is FROZEN at the first eligible frame (msd_entry), travel integrates from it,
  overrun = travel - msd_entry at the wheel latch, the model's stop-point churn is logged. APPLIED. Revert lines for a
  LIVE evaluation: overrun > 3.0 m, any incomplete stop, a release-then-re-brake pump, driver brake/takeover within 2 s
  of a release, a raw lead / FCW appearing after a release, > 1 entry/release per stop.
  NOT APPLIED (out of reach today): the reviewer's gate-2 bar -- a full planner->longcontrol->service replay, a true
  shadow instance, >= 100 held-out candidate episodes over >= 10 routes (the corpus holds 13 intent episodes in 46
  routes: the class is rare), the synthetic grid, the negative controls. Recorded-input replay of the four census
  stops with the implementation: entry 0.9-3.5 s earlier, the glide at -0.40, NEW braking below the legacy chain 0.000
  on all four, the release bounded by the trajectory lane. DECISION LEFT TO THE USER: LIVE with these bounds and the
  revert lines, or stay off. The code ships with the flag "off" (frame-identical to today; suites pass).
- 2026-09-05 CYCLE 50 CODE REVIEW (astra high, run 20260905-193847): REQUEST CHANGES; the flag stays "off". The decisive
  finding: a service that OWNS the wire cannot be release-only against the legacy chain, because on owned frames that
  chain consumes the service's output (slew from last_output_accel, PID reseed) -- the "legacy wire" passed back is not an
  independent counterfactual. Plus: the dropout clamp and the jerk limiter run after the invariant; the floors do not
  bound the final command when the legacy is shallower than the trajectory lane; a rejected close raw reading followed by
  a dropout leaves a far held gap that passes the margin; mode exit does not clear the episode latch; the drift capture is
  unreachable at the wheel-stop latch. Full record in the worklog.
- 2026-09-05 CYCLE 51 -- APPROACH CHANGE (design, to be red-teamed before code): the NO-LEAD RELEASE OVERLAY. No ownership,
  no service entry, no context involvement. In longcontrol, after the whole legacy chain has produced this frame's value
  `legacy` (and the service's own terminal, unchanged), a pure final writer computes, on ELIGIBLE frames only:
  out = max(legacy, min(floor, out_prev + J_RELEASE*dt)), J_RELEASE = 0.8 m/s^3, out_prev seeded from legacy on the first
  eligible frame; on the first INELIGIBLE frame out = max(legacy, out_prev - J_SAFE*dt) until it meets legacy (bounded
  re-admission), then off. The legacy chain keeps its OWN last_output_accel and integrator state (the overlay writes only
  the actuator value): the baseline is exact by construction, the floors bound the final command by construction, and
  scope loss simply switches the overlay off. floor = min(a_glide, aTargetTrajectory, force-coast level when active,
  -0.03) with a_glide = -max(v^2/(2*max(msd, 0.15)), A_SETTLE_REF 0.40) - coast (the deployed no-lead comfort reference,
  evaluated on the model stop msd). ELIGIBLE = flag "live" AND experimental (blended) mode AND active, no input hold, no
  gas/brake override AND V_OVERLAY_MIN 0.5 < v < 2.5 AND the stopping service is NOT active (its terminal ease/hold stay
  the deployed ending) AND no lead of ANY kind now (leadOne.status, leadTwo.status False) AND the minimum RAW leadOne/leadTwo
  distance seen in the last 2.0 s (any status True frame, vision included) >= msd + 8.0 m or none seen (reviewer finding 5:
  raw provenance, not the filter's held gap) AND model stop intent (distanceToStopTargetModel >= 0, persisted 0.20 s with
  decay, held 0.40 s) AND planner a_target <= -0.30 AND aTargetTrajectory valid AND no FCW AND not standstill. Telemetry in
  the settle summary via longcontrol's telemetry object: overlay frames, release frames, released sum/max, entries,
  re-admissions with reasons, the frozen anchor (msd at the first eligible frame), travel, overrun at the wheel-stop latch
  (captured by longcontrol, phase-independent), model stop churn. Revert = the word "off". Gate 2 = the recorded replay of
  the four census stops through longcontrol itself (the overlay is longcontrol-local, so the replay can drive the real
  function) + frame-exact equality with the flag off + the negative controls the design red-team listed. What the overlay
  gives on the bookmark: the planner's excess above the trajectory lane (0.2-0.4 m/s^2 at 12.2-13.0 s, 0.2-0.4 into the
  wheel stop down to v = 0.5); what it does not: the onset (stage B) and the last 0.5 m/s (the service's terminal, entered
  at v = 0.2 today -- an earlier service entry remains a separate, later step). Design red-team: astra high, run
  20260905-194714.
- 2026-09-05 CYCLE 51 RED-TEAM (astra high, 20260905-194714): NOT LIVE -- the decisive question was the LANE: blended mode's
  target is min(MPC, e2e desiredAcceleration); aTargetTrajectory carries the MPC only, so "the excess above the trajectory
  lane" could have been the model's own direct braking. ANSWERED BY THE LOGS (worklog, LANE CLASSIFICATION): on all four
  recorded no-lead stops the published target is deeper than BOTH model lanes by 0.3-0.9 m/s^2, the model's own direct
  demand is the shallowest lane of all, and the excess exists ONLY on force-coast frames (fc on: mean +0.2..+0.6, max +0.9;
  fc off: <= +0.01). The overlay and the service designs are therefore withdrawn: no release is needed; a legacy lane must go.
- 2026-09-05 CYCLE 52 -- APPROACH CHANGE (delete-don't-patch): THE FORCE-COAST CAP YIELDS TO MODEL STOPS. The writer:
  longitudinal_planner.apply_experimental_force_coast_cap = min(target, ACC reference) under force coast -- the only writer
  after the blended merge that can DEEPEN a lead-free frame (the stop-commit/aim floors are gated off under force coast
  and need a lead; the force-coast strength limit only raises). Under force coast the ACC-mode reference MPC plans a hard
  stop where the model plans a gentle one, and the cap injects it: that is the bookmarked harshness, and it is the
  user's own principle inverted (force coast with no lead should be smooth). Design: on frames that are blended AND
  force coast ON AND no lead of ANY kind reported by radard (leadOne/leadTwo status False; a vision-only lead keeps the
  cap) AND the e2e trajectory intends to stop (get_model_stop_intent_distance >= 0, persisted 0.20 s with decay, held
  0.40 s; any lead re-applies the cap on the same frame), the cap is NOT applied: the target stays min(MPC, e2e); every
  other writer is unchanged (the strength limit, min(target, 0) under force coast, the accel-clip rate limiter, the
  standstill shouldStop override, the stopping service's terminal at v ~0.2). Flag FORCE_COAST_CAP_YIELDS_TO_MODEL_STOPS
  (False = today, frame-identical). Census (fc_cap_census.py, routes 2040-2085, the exact frame class): 23 episodes on 4
  routes, 15 of >= 0.3 s, EVERY one followed by a stop within 10 s; only two are moving (2085 s2: 4.3 s, excess 0.45;
  2075 s1: 6.8 s, excess 0.34), the rest are standstill frames (excess 0); 8 sub-0.3 s flickers (max excess 0.63 =
  2077 s5's intent flicker, covered by the hysteresis); 2041 s3 keeps the cap (a vision lead was present). Expected on
  the road: the force-coast no-lead stop follows the model's lanes (-0.5..-1.0 on the approach, -0.1..-0.25 into the
  wheel stop instead of -0.66..-0.98) and rests where the model plans. Red-team: astra high, run 20260905-195533.
- 2026-09-05 CYCLE 52 RED-TEAM (astra high, 20260905-195533): MODIFY, not live as specified -- and its first finding
  closed the case: forceDecel (controlsd: force coast OR driver-monitoring awareness < 0 OR softDisabling) makes the
  planner set v_cruise = 0, so the ACC reference MPC plans a ZERO-CRUISE stop and the force-coast cap injects it; the
  strength limiter then bounds that demand to the FORCE-COAST PROFILE (max(target, min(profile(v), e2e))); long control's
  no-target ramp writes the same profile as the wire on no-lead force-coast PID frames; and the profile is also the
  planner's lower accel clip (frogpilot_acceleration min_accel). THE PROFILE (frogpilot/controls/lib/force_coast.py):
  [-0.7 at the stop gate 0.2 m/s, -1.0 at 1.0 m/s, -1.2 at 2.4 m/s] x strength, strength = the driver's CEForceCoastStrength
  = 1.4 on the device: -1.68 at speed, -1.4 at 1 m/s, -0.98 HELD INTO THE WHEEL STOP. That is the recorded bookmark
  exactly (wire -1.37 at 1.65 m/s, -0.98..-1.2 into the stop). The harsh no-lead force-coast stop is the force-coast
  feature's own deceleration profile with no terminal taper -- not a stopping-service lane, not the model. The cap
  deletion is withdrawn (it would not change the wire: the ramp and the clip carry the same profile). Other findings
  recorded: the forceDecel provenance must be kept for DM/soft-disable; hysteresis is not a jerk limit; the strength
  limiter's model lane is the e2e output, not the MPC (existing).
- 2026-09-05 CYCLE 52 REVISED -- APPROACH CHANGE: TAPER THE FORCE-COAST PROFILE INTO THE STOP. One universal change to the
  driver's own feature, no lane touched, no release: the profile gets a terminal point so it eases below 1 m/s instead
  of holding -0.7 x strength into the wheel stop; the stopping service's hold (entered at v ~0.2) then builds the secure
  pressure only once stopped, exactly as on governed lead stops (a_wheelstop -0.31 measured). Proposed points
  [stop gate 0.2, 0.5, 1.0, 2.4 m/s] -> [-0.35, -0.55, -1.0, -1.2] x strength (at 1.4: -0.49 at the gate, -0.77 at
  0.5 m/s, -1.4 at 1 m/s; unchanged above 1 m/s). All three consumers follow automatically (the ramp target, the
  strength limiter, the accel clip). What it costs: from 1 m/s the stop takes ~0.3 m longer; the force-coast braking
  ENVELOPE below 1 m/s is shallower (a hazard the model sees at 0.5 m/s can brake at -0.77 instead of -1.1 -- 0.16 m of
  stopping distance at that speed; the driver's foot is the fallback, as for the whole force-coast envelope today).
  Flag FORCE_COAST_TERMINAL_TAPER (False = today's profile). Evidence: the profile IS the wire on the recorded stops, so
  the recorded stops' terminal commands become the tapered values by construction; the felt metric on the next drives
  and the user's rating decide the taper level. Red-team: astra high (launched with this entry).
- 2026-09-05 CYCLE 52 TAPER RED-TEAM (astra high, 20260905-200130): "MODIFY, then replay and test; no bounded live yet".
  (1) BLOCKER -- long control's no-target ramp REPLACES the wire, so a deeper demand that passed the planner's
  strength limiter (the model's own braking, modelV2.action.desiredAcceleration) is capped to the driver's profile under
  force coast with no target. NOT changed: that cap is the feature's pinned contract (test_longcontrol_force_coast_*_
  no_target_brake_cap: "weak strength caps no-target braking to the selected target"), i.e. force coast is the driver's
  deceleration selection over everything on lead-free frames. OPEN DESIGN QUESTION FOR THE DRIVER: should force coast
  yield to a model demand deeper than the profile (a pedestrian the model sees while the driver coasts)? Today it does
  not; the planner intends it to (the strength limiter lets the deeper model demand through) and long control undoes it.
  (2) BLOCKER -- continuity is not jerk: the taper's knee gives +1.76 m/s^3 of
  commanded release at 1 m/s; the force-coast command now rises no faster than FORCE_COAST_RELEASE_J 0.8 m/s^3 from the
  ACTUAL previous wire (knee, speed noise, re-entry), deeper immediate. (3) service entry is on the model's stop bit
  (~0.2 m/s on this model), not a fixed threshold, and RAMP_TO_HOLD needs the wheel-stop latch; the service's terminal
  descent (-0.70 target below 0.45 m/s) governs once entered -- the taper acts in the 0.2-1.0 m/s window before it.
  (4) BLOCKER -- creep margin: recorded escapes sit at commands -0.32..-0.62, so the first candidate keeps the
  wheel-stop value at -0.50 x strength = -0.70 at the driver's 1.4 (the service's own hold level; no new creep
  exposure) and the knee -0.60 x strength = -0.84 at 0.5 m/s; unchanged from 1 m/s up. Breakpoints are offsets from
  the stop gate (ordered for any vEgoStopping). (5) distances corrected: +0.14 m from 1 m/s, +0.09 m from 0.5 m/s
  (ideal); the first candidate is for strength 1.4 only (a strength-scaled taper at 0.5 is unvalidated).
  (6) evidence before a bounded evaluation: flag-off equality (tests), the four stops' terminal wire by construction
  (the profile IS the wire on those frames), synthetic hazard / knee / toggle tests (added), creep and grade cases and a
  full-rlog replay of held-out stops (open). Deploy: the flag is ON in the code; the user decides whether to apply it
  (Full Update) for the bounded evaluation -- revert = the flag word False.
- 2026-09-05 CYCLE 52 FINAL -- THE DRIVER'S FORCE-COAST CONTRACT (user, 2026-09-05 evening): "I want force coast to cause the
  model to brake when the model wouldn't. If the car sees something it should stop for, that takes priority. Force coast
  should never end with a bad stop." APPLIED: (1) long control's no-target ramp is a FLOOR, never a cap --
  output = min(planner/PID demand, force-coast command); the planner's strength limiter already bounds the ACC zero-cruise
  demand to the profile, so a deeper demand reaching long control is the model's own braking or a close/closing lead, and
  it passes (the two tests that pinned the cap contract are rewritten to the floor contract; the open question is closed).
  (2) the profile's low-speed tail is ABSOLUTE and strength-independent, a bound on the shallow side:
  [gate 0.2, 0.5, 1.0, 1.5 m/s] -> [-0.5, -0.6, -0.8, -1.0], fading out steeply to the strength profile by ~1.8 m/s
  (continuous: the max of two non-increasing curves); it only ever LOWERS the braking, so a weak strength stays weak and
  a strong strength keeps its high-speed profile untouched. This replaces the strength-scaled taper of 000e753c
  (-0.70/-0.84 at 1.4). (3) the 0.8 m/s^3 ease limit, the standstill hold and the service's terminal are unchanged.
  Expected on the bookmark class: the approach follows the model's lanes above 1.5 m/s (the strength profile where the
  model is gentler), then the tail; the wheel-stop command is -0.5 instead of -0.98. Revert = FORCE_COAST_TERMINAL_TAPER
  False for the tail (the floor contract stays: it is the driver's stated intent).
- 2026-09-05 CYCLE 52 FINAL, review (astra high, 20260905-212027, REQUEST CHANGES) -- applied: (1) HIGH the synthetic ACC
  zero-cruise demand arrives limited to the FULL profile and bypassed the 1.5 s ramp-in under "deeper wins": now only a demand
  deeper than the full profile at this speed passes (the model's own braking / a close lead); anything up to the profile is force
  coast's own request and ramps in from the wire as before. (2) HIGH creep: the profile alone cannot guarantee standstill (a car
  settling at 0.3 m/s on the tail's -0.53 with no service entry); now a lead-free force-coast slow-down below 1.0 m/s ENTERS the
  stopping service (its terminal descent commands -0.70 below 0.45 m/s, its monitor catches inadequate deceleration, its hold
  builds once stopped, its RELEASE hands back when force coast ends) -- the service's own should_stop input only. (3) MEDIUM
  a demand lifting mid-ramp stepped the wire up in one frame: the ease limiter now bounds the branch's FINAL output (ramp, tail
  and demand lift alike) at 0.8 m/s^3 from its own previous value. (4) tests: onset at the profile (ramps from 0), mid-ramp
  lift (eased), a knee case where the limiter must bind, exact pass-through of a deeper demand, service entry (0.8 m/s yes,
  1.2 no, lead no). Fade-out of the tail into the strength profile completes at 1.53 / 1.69 / 1.96 m/s (gate 0.2) for
  strength 1.0 / 1.4 / 2.0 (reviewer's exact figures; "by ~1.8" was not universal).

