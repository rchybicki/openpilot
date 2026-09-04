# The universal stop program (opened 2026-08-23, cycle 34)

## Current decision -- process reset, 2026-09-04 (cycle 43)

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
   to count safety interventions, incomplete stops and all excluded data. No SHADOW/LIVE activation in cycle 43.

Immediate sequence: correct and version the offline measurements (this cycle); reconstruct continuous,
uniquely identified approaches with matched lead/speed classes from current logs; audit natural brake-command
excitation and held-out plant error. Reuse the event store, extractor and fitter. The specific scripted-step
drive below is a proposal, not an implemented prerequisite: first establish exactly what existing data lacks.
If it lacks an identifiable response, prepare a bounded controlled collection protocol. No brake-step hook
is authorized by this process reset. Sparse bookmarks are unknown, not perfect; ~30 explicit ratings can
support exploratory ordering, not automatic learned-score acceptance. The shared NaN input-boundary repair
remains the next wire safety task and a prerequisite to a new wire feature; it is separate from comfort progress.

Every experiment records one falsifiable mechanism, fixed data split, comparator, success/rejection rule,
affected classes, and the existing code it will remove. Report evidence gained and live behavior changed;
tests, disabled code, commits and review rounds are not smoothness gains. Use at most two scoped reviews,
with a self-contained evidence packet. Deploy research only when on-car telemetry answers a named gap.
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
