# Stopping Service V3 — review verdict + redesign plan (2026-07-01)

Provenance: produced by the 2026-07-01 stopping review session. Inputs: (1) fresh device logs
routes 00001afc-00001b71 (41 engaged settles deep-analyzed, frame traces), (2) a 6-reader
architecture map of the stack at HEAD 3eba23a14f, (3) a 3-design judge panel (extend-V2 /
surgical / clean-slate) with adversarial judging; this document is the synthesis with every
judge-found hole explicitly fixed. Status: PROPOSAL — not implemented; no runtime change in
this commit.

Review verdict (evidence in review_cursor.json log entry 2026-07-01):
- Terminal-glide fix HOLDS: zero stop-go-stop leapfrogs in 41 settles.
- BUT the felt wheel-stop grab persists on most stops (terminal IMU jerk median ~5.5 m/s3,
  peak settle decel 0.6-1.2): the wire carries -0.60..-0.81 through wheel-stop while the
  planner aTarget correctly glides out to -0.12 (frame-traced 00001b6c seg4). Root: sub-0.30
  legacy cap re-enable (longcontrol.py:446-458) + glide-cap -0.60 clip floor whose activation
  gaps cover the normal resting zone; also the synthetic stop-target pins remaining=0.05 m
  while still rolling (stop_target_helpers.py:160), clobbering the planner's honest distance.
- The 06-30 "0.3-1.8 jerk validated" figure was a measurement-method artifact; same-method
  A/B shows no regression and no improvement from the settle gate on terminal feel.
- Close-gap creep now demonstrably causes DISLIKE2 nudges (2/41) - retire with the redesign
  (its job is superseded by the distance-gated settle + this plan's final-rest semantics).
- Process finding: post-forest-deletion the stack regrew to ~31 writers / ~1,050 tuned values
  in 12 days; ca47fc7118 (slowing-queue cap, ~48 constants under the PID) repeats the forest
  genesis pattern at the wrong layer. The plan below is the structural answer.

All load-bearing file:line claims from the judges verified at HEAD `3eba23a14f` (planner-floor v-gate 0.30 + `lead_status` gate at `longcontrol.py:104-133`, glide-cap −0.60 clip floor and ≤4.15 m near-hold branch at `:385-416`, sub-0.30 re-enable at `:446-458`, `FORCE_COAST_STANDSTILL_HOLD_ACCEL = -0.32` at `:65/:1122`, module line counts, the arbiter's shared predicate at `stop_target_arbiter.py:225/:405`, the dts pin at `stop_target_helpers.py:14/109/160`, and the eval tooling in `tools/stopping/`). The final plan follows.

---

# FINAL PLAN — Stopping Service V3 (synthesis)

**Repo:** `/Users/radoslawchybicki/Repos/openpilot-rch` @ `3eba23a14f` · Scope: Santa Fe HEV stopping, v < 2.5 m/s · 2026-07-01

## 0. Decision

**Winner: Design 3 (clean-slate service) as the chassis, with Design 1's gap filter and creep feedforward and Design 2's envelope insight and deletion hygiene grafted in.** Rationale:

- D3 scored highest with the judges (37 vs 34/32), won outright on the two axes the user actually asked for — felt-smoothness (8/10, the only design whose traced wheel-stop wire is −0.05..−0.18) and de-sprawl (9/10, −3,400 lines, user explicitly welcomes drastic refactors) — and its judge *tried and failed to construct an under-brake collision*.
- D1 keeps the tracker/arbiter sprawl and its own judge proved its terminal law strands 0.3–0.8 m (rests 4.23–4.83 m) requiring new patches — the patch-on-patch pattern this project exists to kill.
- D2's safety case was wrong on its face (cites a planner floor that is verifiably gated OFF below 0.30 m/s and above only with `lead_status`) and its net slams (bare `min()`, no deepen slew). Its judge: "safe by luck." But two of its ideas are load-bearing here: the d_settle envelope bound (the argued version of what actually protects the terminal band) and the byte-identical-deletion / seam-placement discipline.
- All three of D3's holes are fixable inside its architecture (its judge's own words), and two of the three fixes are literally D1's mechanisms — the persistence gap filter and the disturbance-adaptive ease floor — which D1's judge independently verified as sound. The synthesis is therefore lower-risk than any single design.

**Grafts:** from D1 — the asymmetric persistence GapEstimator (verified by Judge 1: alternating 2.0↔3.9 never survives, genuine closing never delayed), the deepen-only creep feedforward (`A_PRESTOP` construction), the deliberate tip-in protocol, and the insight that the planner min-lane must bind *every* writer (trivial here: there is only one writer). From D2 — the d_settle envelope floor for no-lead targets, placing all logic inside the `sim_replay` facade seam (legacy caps were never replayable), pins-regenerated-in-a-zero-runtime-diff-commit, same-IMU-method metric hygiene, and the slim telemetry survivor. From D3 — everything else: single service, physics-named constants, phase machine, SHADOW stage, fixtures-first arbiter deletion, upstream-verbatim non-Santa-Fe path.

## 1. Judge-hole ledger — every hole, fixed explicitly

| # | Hole (judge) | Fix in this plan | Where |
|---|---|---|---|
| D1-H1 | `lead_status` gate on planner floor → radar drop sanctions shallow-release below planner demand (vision lead ignored) | The planner min-lane `a_plan` is **unconditional**: no lead gate, no gap gate, no speed gate above wheel-stop latch. `min()` with planner demand can never under-brake and costs nothing when planner is shallow (ground truth: planner glides to −0.12 on nominal stops, so smoothness survives) | §3 safety lane |
| D1-H2 | `D_REST = 4.0` puts the guard's activation zone over normal resting zones (2.1–2.9 m rests, close pull-ups) → grab relocated, −0.24..−0.45 at wheel-stop; guard cliff at exactly 4.0 chatters | **`D_REST_eff` re-zeroed at service entry** for close entries (rest where a comfortable glide lands, never < 2.4 m), so `remaining = 0 while resting normally` cannot occur; there is **no discrete guard table at all** — `a_kin` is continuous in gap with `D_HARD = 2.0` sitting ≥ 0.4 m below any permitted rest. No cliff exists to chatter | §3 `D_REST_eff`, safety lane |
| D1-H3 | Jerk-limited release strands 0.3–0.8 m; all recovery lanes deleted → permanent 5–8 m rests | Root cause removed: the strand came from *inheriting* a deep command at v = 0.85. The service owns the wire from 2.5 m/s, so command and parabola are jerk-consistent through the terminal band (residual strand ≤ ~0.3 m, in-band). Remaining strand causes (bounce slam, dropout collapse) are fixed at source (D3-H1, D2-H3 rows). Rests are **final** (see D2-H3) with a frequency gate ≤ 1/40 beyond 5.2 m | §3, §6 gates |
| D2-H1 | Net double-writes the 2.1–2.9 m rest class and slams (bare `min()`, 15–48 m/s³ steps); ISD distance-space ambiguity | **No bare `min()` reaches the wire**: one final jerk limiter bounds every frame (release J_UP = 1.5, comfort deepen J_DOWN = 2.5, safety deepen J_SAFE = 8.0) — the no-slam invariant holds by construction. Distance space declared once: **all service laws in TRUE meters**; ISD enters only `D_REST_NOM = 4.0 + ISD` (clipped [2.5, 5.0]). `D_HARD = 2.0` sits below the observed rest band; `D_REST_eff` prevents in-band double-writes | §3 |
| D2-H2 | Safety case cited a floor gated OFF below 0.30; bind hysteresis defeated by alternating bounce; real protection (d_settle envelope) unargued | Safety case rewritten around three always-live mechanisms: (1) `a_kin`, continuous, no dead zone, no bind hysteresis to defeat; (2) `a_plan`, live at **all speeds** to wheel-stop; (3) the phase law's floored denominator = the d_settle envelope, now stated as a design element, not an accident. The alternating-bounce-through-a-full-stop fixture is a **mandatory stage-0 gate** | §3, §5, §6 stage 0 |
| D2-H3 | Dropout mid-glide → stop 1.7 m short → DISLIKE1 crawl; 4.3–5.0 m dead zone with no recovery lane | **Dropout decay-hold**: on lead loss, last-good gap decays inward at 0.5 m/s for 2.0 s (lengthened per the judge's prescription) and the command may not release above −0.25 — the glide keeps braking toward the virtual target; no mid-glide collapse to the kinematic fallback. And **no post-stop motion lanes exist at all** (creep AND far-release deleted), so DISLIKE1/DISLIKE2 motion signatures are structurally impossible from this stack. Trade-off accepted openly: a rare far rest is final (taxonomy lists motion patterns, not positions); gated at ≤ 1/40 | §3 dropout, §6 |
| D3-H1 | Instant inward pass-through → bounce slam (−1.67..−3.75, 1.83 s poisoning) → stop-short → DISLIKE1 | D1's GapEstimator grafted as **the** gap filter, feeding BOTH the glide law and `a_kin`: same-track inward steps larger than ego-motion-consistent need 0.15 s persistence (3 radar cycles); ego-consistent closing and track-ID changes (cut-ins) pass immediately; outward steps need 0.25 s + rate limit. The spurious 2.0 m frame never reaches any law. Residual honesty: a *real* same-track step-collapse now costs 0.15 s ≈ ≤ 0.15 m at terminal closing speeds — included in the §5 worst-case margin arithmetic (0.24 m closure vs 0.6 m margin) | §3 signals |
| D3-H2 | EASE unreachable as narrated (v_close gate wrong for stopped lead); anti-creep term is dead code (max() inversion); creep hover never latches wheel-stop | EASE **rewritten**: gate on *lead motion* (`v_lead ≥ −0.1`), not v_close — reachable band is genuinely v ≤ 0.5, matching the P1 audit. Anti-creep is now a **deepen-only feedforward** `− clip(a_coast, 0, 0.4)` (D1's A_PRESTOP construction, judge-1-approved); the dead `−K_V·v` term is deleted. Hover is caught by the **always-on anti-hover/anti-roll monitor** (v not decreasing ≥ 0.02 m/s over 0.4 s while > 0.03 ⇒ deepen at J_SAFE, escalating −0.15 per 0.5 s, unbounded) | §3 EASE + monitor |
| D3-H3 | No-lead stop-point latch unspecified; sub-0.30 planner blackout → 0.9 m rollout past a stop line | No latch: the planner stop target is consumed **continuously** through D2's d_settle conditioning `d_rem = max(dts, v²/(2·0.40))` — line moves closer ⇒ deepen continuously; farther ⇒ release at J_UP; no staleness possible. And `a_plan` is live below 0.30 (see D1-H1 row): a −0.8 planner demand at v = 0.25 is answered at J_SAFE | §3 no-lead |

**Secondary findings, swept:** dropout no-release floor specified at **−0.25** (J1: "A_NEAR_HOLD load-bearing but unspecified"; sits in the proven micro-dropout −0.30..−0.24 band). Sensor diversity of the deepen lanes (J1): `a_kin` = radar, `a_plan` = vision+radar via MPC, anti-roll monitor = wheel-speed/IMU — three lanes, three sensor families. Uphill (J1's `max(d_hat,0)` ding): GLIDE uses symmetric `− a_coast` feedforward (clipped, never above −0.03) so grade doesn't strand rests; EASE stays deepen-only (uphill firming a stop is safe; shallowing on grade risks rollback) with the no-rollback invariant kept. Wheel-stop flap on 0.03-quantized vEgo (J3): latch = `CS.standstill OR v ≤ 0.06 for 0.25 s, reset on any sample > 0.09` — 0.03/0.06 alternation latches cleanly. Anti-roll escalation now unbounded (J3). Lead-stopped latch: `v_lead ∈ [−0.1, +0.3]` — a reversing lead is never "stopped" (J3 deduction). J_SETTLE_RELEASE citation error (J2): moot — law replaced — but the lesson is kept as a stage-0 **release-rate audit test** asserting the terminal release actually achieves J_UP on the nominal fixture. Arbiter shared-predicate trap (J2, `stop_target_arbiter.py:225/:405`): stages 2–3 bypass by flag with all imports intact; stage 4 deletes arbiter and caps in the *same* commit, so nothing dangles. Soak-counter recoverability (J2): shadow logs provide divergence counters directly. D2's risky dts-publish change is **dropped entirely** — the service never consumes the pinned value, the planner's aTarget glides fine despite it (ground truth 2), so the pin becomes documented dead weight removed in stage 4 with zero consumer risk. Stage bundling / calendar (J1, J2): stages re-cut to one mechanism each and one drive each (§6). Statistical power at small n (J3): gates redesigned as high-contrast fraction tests (§6, §7) with DoD on a rolling 40 settles. TCS window (J3): hold ramp starts at the first qualifying wheel-stop frame, firm within ~0.5 s of physical stop, plus the tip-in protocol. Conditional-deletion limbo (J1): deletions are unconditional at stage 4, authorized by live stage-2/3 evidence, not by counters.

## 2. Target architecture

```
radarState ──┐
carState  ───┼─► stop_context.py (~200 ln)          longitudinal_planner (untouched)
planner ─────┘   d_gap (persistence filter),           │ aTarget, shouldStop, dts (advisory)
                 a_coast, lead latch, wheel-stop       │
                        │                              │
                        ▼                              ▼
              stopping_service.py (~450 ln) — SOLE stopping-band writer
              INACTIVE → APPROACH_GLIDE → PRE_STOP_EASE → RAMP_TO_HOLD → HOLD → RELEASE
              phase law ─► one final jerk limiter ─► min(·, a_kin, a_plan) deepen-only
                        │
                        ▼
              longcontrol.py (~450 ln) — state machine + PID for driving;
              stopping branch = ~25-line dispatch; FORCE_COAST −0.32 tail min() kept verbatim
              non-Santa-Fe → upstream stock stopping, verbatim
```

Roles, one sentence each: **stop_context** conditions four signals and nothing else; **stopping_service** owns the wire below 2.5 m/s with one glide law, one comfort floor, and an always-live deepen-only safety lane; **longcontrol** shrinks to dispatch; **stopping_telemetry.py** (~60 lines) emits phase changes plus a per-settle summary event carrying the §7 metrics so every gate is computable from rlogs. Structural rule (P1, enforced): the only lanes that modify the phase command are `min()` lanes; the only shallow region (EASE) is bounded on speed, gap, and lead motion, and its deepen lanes never disarm.

## 3. Control laws (exact)

**Constants** (all physical, ~25 total, replacing ~90 tables/constants):

```
V_ENTER 2.5 · V_EASE 0.50 · V_WSTOP 0.06 (latch: CS.standstill OR v≤0.06 for 0.25 s, reset >0.09)
D_REST_NOM 4.0+ISD clipped [2.5,5.0] (TRUE meters everywhere) · D_HARD 2.0 · D_REST_MIN 2.4
A_GLIDE_NOM 0.5 · A_EASE_CAP −0.10 · A_EASE_DEEP −0.35 · A_HOLD −0.32 (== FORCE_COAST pin)
A_DROPOUT_MIN −0.25 · A_SETTLE_REF 0.40
J_DOWN 2.5 · J_UP 1.5 · J_SAFE 8.0 · J_HOLD 0.6 · J_GO 1.2 (all m/s³)
T_PERSIST_IN 0.15 s · T_PERSIST_OUT 0.25 s · R_OUT max(v_lead,0)+0.5 m/s · T_DROPOUT 2.0 s · τ_COAST 1.0 s
```

**Signals (stop_context):**
- `d_gap`: track-ID change ⇒ accept raw immediately (cut-in is real). Same-track inward step ≤ ego-closing-consistent (v_close·dt + 0.3 m) ⇒ immediate; larger ⇒ 0.15 s persistence, meanwhile hold ego-motion-propagated prediction. Outward ⇒ 0.25 s persistence + R_OUT rate limit. One output feeds every law.
- `a_coast = EMA_1.0s(aEgo − a_cmd(t−τ_delay))`, clip [−0.5, +0.5]; below v = 0.1 hold last value (used deepen-only there).
- `d_rem` (lead) `= d_gap − D_REST_eff`, where at entry `D_REST_eff = min(D_REST_NOM, max(d_gap_entry − v_entry²/(2·A_GLIDE_NOM), D_REST_MIN))`, re-computed only if d_gap grows > 1.0 m. `d_rem` (no-lead) `= max(dts_planner, v²/(2·A_SETTLE_REF))`, continuous. Both targets present ⇒ min.
- Lead latch `lead_confirmed_stopped := lead_status ∧ v_lead ∈ [−0.1, +0.3]` held 0.3 s.

**Safety lane (live in every phase):**
```
v_close = max(v_ego − v_lead, 0)                       # v_lead < 0 raises demand
a_kin   = −v_close²/(2·max(d_gap − D_HARD, 0.30))      # lead present; unbounded depth
a_plan  = a_target if a_target ≤ −0.10 and not wheel_stop_latched else +inf   # NO other gates
a_cmd   = jerk_limit( min(a_phase, a_kin, a_plan) )    # deepen at J_SAFE, release J_UP, build J_HOLD
```

**APPROACH_GLIDE** (entry: engaged ∧ Santa Fe ∧ v < V_ENTER ∧ (shouldStop ∨ (lead latch ∧ d_rem < 15))):
`a_phase = clip(−v²/(2·max(d_rem, 0.15)) − a_coast, planner_min_limit, −0.03)` — the −0.86 → −0.12 glide the planner already computes, on the wire, grade-compensated.

**PRE_STOP_EASE** (v ≤ 0.5 ∧ d_rem ≤ 0.8 ∧ d_gap > 2.6 ∧ v_lead ≥ −0.1; any gate fails ⇒ GLIDE law at J_SAFE):
`a_phase = clip( clip(a_stop, −0.35, −0.10) − clip(a_coast, 0, 0.4), −0.35, −0.03 )` — arrives at wheel-stop commanding ≈ −0.10..−0.15 net of measured creep, instead of −0.60..−0.81.

**Anti-hover/anti-roll monitor** (EASE/RAMP/HOLD): v > 0.03 not decreasing ≥ 0.02 m/s per 0.4 s, or v rising > 0.06 above running min ⇒ deepen at J_SAFE to min(current, −0.35), escalate −0.15 per 0.5 s unbounded, until decreasing again or wheel-stop.

**RAMP_TO_HOLD**: starts on the first qualifying wheel-stop frame; ramp to A_HOLD at J_HOLD ⇒ firm −0.32 within ≈ 0.5 s of physical stop, pressure built while stationary (silent).

**HOLD**: −0.32 constant; `a_kin` live (reversing lead deepens the hold); **no post-stop motion lanes exist** — rests are final.

**RELEASE** (a_target > 0.2 ∧ (v_lead − v_ego > 0.5 ∨ gap grew 0.3 m), or state exit): ramp to 0 at J_GO; hold never unwinds shallow without a genuine go (TCS path untouched).

**Dropout**: decay-hold per ledger row D2-H3; dropout may deepen or hold, never release (arbiter DROPOUT_HOLD knowledge, ported as rule + fixtures).

## 4. Deletion list (stage 4, one commit; flag-bypassed-not-deleted until then)

Whole modules under `/Users/radoslawchybicki/Repos/openpilot-rch/selfdrive/controls/lib/`: `stop_target_arbiter.py` (859 — fixtures ported first: seg24, seg27, 00001756, dropout holds), `stopping_params.py` (476), `stopping_shadow.py` (445), `stopping_tracker.py` (368), `stopping_profile_selector.py` (363), `stopping_controller_v2.py` (261), `stopping_trajectory.py` (140), `stopping_guard.py` (35); `stopping_flags.py` 90 → ~15; `stopping_plant.py` (278) moved to `tools/stopping/`. Inside `longcontrol.py` (1,160 → ~450): both cap families + gates (:328-352, :354-382, :385-416, :419-440, :442-458), creep block + apply (:461-547, :1083-1116) and its state, planner-floor bolt-on (:104-133, reborn as the integral `a_plan` lane), all shadow/observer plumbing (:136-166, :619-672, :687-750, :1124-1157), cap apply sites (:887-891, :968-1005), STOPPING_V_BP tables (:51-53). `drive_helpers.py:60-80`. `stop_target_helpers.py` arrived-gate corpse + graduated-flag dead branches + the 0.05-pin semantics (publish untouched until this commit; nothing consumes it by then). Kept verbatim: FORCE_COAST tail (:1122), PID approach caps (:211-326, acknowledged sprawl seed → separate planner-absorption follow-up), StopReq semantics. Tests: delete/regenerate arbiter/tracker/trajectory/shadow/params pins and the ~35 table pins; **keep `test_stopping_v2_replay.py` retargeted at the service seam**; prune shadow/selector tools (event store verified rlog-based, unaffected). **Net ≈ −3,800 production lines, +~700 built, ~90 tables/constants → ~25 named physical constants.** Regrowth guard: a CI test pinning that `longcontrol.py` gains no new `*_accel_cap`/`*_brake_floor`/`*_settle_cap` functions; the doctrine is incident ⇒ fixture + law change in the service, never a new cap.

## 5. Safety case (P1 audit)

- **Direction rule**: every modifier of the phase command is `min()`. The sole shallow region (EASE) is bounded on every axis P1 lacked: v ≤ 0.5, true gap > 2.6 (0.6 m above D_HARD), lead not reversing, and the deepen lanes never disarm. Worst case (lead lurches backward 0.5 m/s while ego at 0.5): lead-contribution detection delayed ≤ 0.15 s by persistence (ego contribution passes instantly), deepen −0.10 → −0.83+ (`a_kin` at gap 2.6) in 0.09 s at J_SAFE ⇒ closure ≈ 0.24 m against 0.6 m margin. Verified in fixtures before any live frame.
- **Terminal band under-brake** (why deleting the sub-0.30 caps opens no hole): three always-live lanes on three sensor families — `a_kin` (radar, continuous, unbounded, deeper than the old −0.60 saturation on true encroachment), `a_plan` (vision+radar MPC, ungated by speed/lead-flags — fixes the verified v-gate/lead-gate defects of the current floor), anti-roll monitor (wheel-speed/IMU). Plus the phase law's floored denominator bounds worst-case rollforward: persistent-confirmed gap 2.0 at v = 0.28 ⇒ glide demands −0.26, stop in ≈ 0.15 m.
- **HEV creep**: deepen-only `a_coast` feedforward (measured push deepens one-for-one), anti-hover monitor with unbounded escalation, −0.32 hold at standstill — and ground truth already shows zero leapfrogs with terminal commands well below −0.60 above 0.30 m/s; this extends the same jerk-limited construction to 0.
- **TCS**: hold reaches −0.32 within ~0.5 s of physical stop (ramp starts at detect), ahead of any plausible tip-in (≥ ~0.7 s foot transfer); FORCE_COAST tail min() retained; explicit 5-tip-in protocol including one as-fast-as-possible, gated in stage 2; this is the single new TCS surface and it is one flag from revert.
- **Radar bounce**: unconfirmed inward flicker never reaches any law; confirmed collapse gets full unbounded deepening; asymmetry means bad news is delayed at most 0.15 s only when physically inconsistent, good news 0.25 s always.

## 6. Staged migration — one mechanism, one deploy, ~1 engaged drive per stage

Deploys via `fullupdate.sh` detach flow; verify device hash after each; any DISLIKE1/DISLIKE2, takeover, TCS fault, or leapfrog ⇒ flag revert before analysis. Flags are in-code constants per repo Params policy. Observed rate ≈ 8–12 settles/drive, so every on-road gate below is decidable in one drive; core path = 5 drives ≈ 1 week.

| Stage | Change (ship/revert) | Offline gate (before deploy) | On-road gate (~1 drive) |
|---|---|---|---|
| **0** | Build service+context+telemetry + sim_replay adapter; port arbiter incident fixtures; new adversarial fixtures: **alternating 2.0↔3.9 bounce through a full stop**, lead reversing 0.5 m/s in EASE, +0.3 m/s² creep-push, ±5% grade, dropout at gap 3.0/v 0.4, close entries (gap 3.0 at v 0.6 and 1.2), stop-line moved ±1.5 m mid-stop, hover plant, NaN frames; release-rate audit test. Ships as inert code + tests; revert trivial | 239-event A/B + all 31 retargeted invariants green; **default-fail under-brake gate** (the construction that correctly killed both prior sub-0.30 attempts): never shallower than a_safe on any frame, min true gap ≥ legacy per event, ≥ 2.0 m always; no-slam (per-frame Δ ≤ J_SAFE·dt); no-rollback ≤ 0.15 m; wheel-stop-release u ≥ −0.35 on nominal fixtures | none |
| **1** | `SERVICE_MODE=SHADOW` — zero behavior change; revert = flag OFF (or nothing) | n/a | 1 drive ≥ 8 settles; on shadow logs: divergence direction-only (shallower than legacy only inside EASE gates), zero frames above a_safe, predicted wheel-stop wire ∈ [−0.35, −0.05] on ≥ 90%, a_coast plausibility; harvest dropout counters for fixtures |
| **2** | `SERVICE_MODE=LIVE_TERMINAL` — service owns v ≤ 0.85 (jerk-consistent takeover from live wire); legacy caps bypassed by flag, code intact; revert = one flag → byte-identical legacy | stage-0 suite re-green on the exact deploy SHA | 1 drive ≥ 8 settles: zero takeover/TCS/leapfrog/DISLIKE; wheel-stop wire ∈ [−0.35, −0.05] AND post-standstill snap ≤ 0.15 on ≥ 80% (baseline probability ≈ 0 — wire pinned −0.60..−0.81 today, so this is decisive at n = 8); **≥ 7 of first 10 settles with terminal IMU jerk < 3.5 m/s³** (baseline P(pass) ≈ 0.35%, power ≈ 0.99 if the fix works); hold ≤ −0.30 within 0.7 s on 100%; rests ∈ [2.4, 5.2]; parking-lot **5-tip-in protocol**, zero faults |
| **3** | `SERVICE_MODE=LIVE` — full band to 2.5 m/s (removes the 0.85 seam and its inherited-command strand); revert = flag back to LIVE_TERMINAL | replay green incl. close-entry + high-speed fixtures | 1 drive ≥ 8 settles incl. ≥ 2 approaches ≥ 8 m/s: stage-2 criteria hold; close-entry rests ≥ 2.4 m; no new jerk events > 4 m/s³ in the 0.85–2.5 band; zero DISLIKE |
| **4** | Deletion commit (§4) + pins regenerated in a dedicated zero-runtime-diff commit; revert = git revert | **byte-near equivalence** vs stage 3 over all 239 events (max wire delta ≤ 0.001 m/s²) before pin regeneration | 1 smoke drive ≥ 5 settles, stage-3 criteria hold |
| **5** | Flag graduation after rolling DoD (§7) reached; separately (optional, own mini-project): planner high-speed stop-commitment retarget to rest ≈ 4.3 m — positioning polish only; **nothing in stages 0–4 depends on it** (D_REST_eff already handles close entries gracefully) | telemetry byte-diff check | rolling |

## 7. Definition of done — what "absolutely smooth" means numerically

Measurement: the frame-trace IMU method from this session (explicitly not the 06-30 artifact method), window [wheel-stop − 0.5 s, wheel-stop + 1.5 s]. **DONE when, over ≥ 40 consecutive engaged settles (rolling, stages 3–5):**

1. Terminal IMU jerk: **median ≤ 2.5 m/s³, p90 ≤ 4.0, max ≤ 6.0** (baseline: median 5.5, range 2–10.4).
2. Peak decel amplitude over the last 0.3 m/s of travel **≤ 0.35 m/s² on ≥ 90%** of stops (baseline 0.6–1.2 on most).
3. Post-standstill rebound (max positive IMU a within 1.5 s) **≤ 0.10 m/s²** (baseline +0.27).
4. Wire at last rolling frame ∈ [−0.35, −0.05] on ≥ 90%; zero stops with brake release occurring *after* standstill (baseline: released ~1.2 s late) — post-stop pressure only builds down toward −0.32, never up-then-down.
5. Taxonomy: 100% OK-class (one continuous motion, creep-in included); **zero DISLIKE1, zero DISLIKE2** — structural, since no post-stop motion lanes exist.
6. Safety zeros: leapfrogs 0 (baseline 0/41), TCS faults 0, takeovers 0; min in-stop true gap ≥ 2.0 m; rests ∈ [2.4, 5.2] with ≤ 1/40 beyond.

**Why 2.5, not lower — the honest derivation.** The felt grab is disc-brake physics: at wheel-stop, pad friction steps from kinetic to static (μ_s > μ_k) and the pitched suspension releases; both transient amplitudes scale with brake force held at the stop instant. Today that force is 0.6–1.2 m/s² of decel ⇒ +0.27 rebound and 5–8 m/s³ jerk. At ≤ 0.15 m/s² command the same physics produces a rebound ≈ +0.03–0.05 m/s² — below the ~0.1 m/s² vestibular salience band — which is what "no disc brake sticking" *feels* like. But the jerk number cannot go to zero: the measured Stribeck divergence peaks at 0.43 m/s² near v ≈ 0.066 (friction-residual session) and releases over ~0.15–0.25 s ⇒ an irreducible ~1.7–2.9 m/s³ instantaneous spike on a 100 Hz IMU derivative even under a perfect command — which is exactly why the *best* stops today already print ≈ 2 and none print lower. So the target is the distribution **collapsed onto its demonstrated floor** (median 2.5, tail ≤ 4), with the *amplitude* criteria (2) and (3) carrying the "felt" requirement. A promise of median < 1.5 m/s³ by this method would be unfalsifiable marketing; do not gate on it.

## 8. What command shaping cannot fix (accepted limits)

1. **Pad stiction / Stribeck floor**: ~0.4 m/s² friction divergence near v ≈ 0.07 is plant physics ⇒ the ~2 m/s³ IMU-jerk floor above. Only brake-blending firmware or hardware could go below; out of reach from the wire.
2. **Mandatory −0.32 hold** (Hyundai TCS): pressure must rise after every stop. We can place it after wheel-stop and rate-limit it to silence (J_HOLD 0.6, stationary), but not remove it; on grades needing deeper hold it will be faintly perceptible.
3. **HEV regen→friction blend handoff** near zero speed is the car's own torque management — measured as not separable/commandable (friction-residual verdict); its small discontinuities remain.
4. **Timing floor**: vEgo quantization (0.03) + actuation delay ⇒ wheel-stop detection jitter ~±50 ms; irrelevant at J_HOLD 0.6 but a hard bound on placement precision.
5. **Radar close-range bounce** is a sensor property; the filter converts its cost from 0.5 m/s² over-brake slams into a bounded 0.15 s reaction delay on physically-inconsistent collapses — a trade, not a cure.

## 9. Residual risks, ranked

1. **EASE under-brake (P1 class)** — bounded gates + never-disarming deepen lanes + default-fail gate + worst-case arithmetic (0.24 vs 0.6 m); one-flag revert. 2. **TCS on the ~0.5 s hold ramp** — detect-frame start, tip-in protocol, escalation path = raise J_HOLD. 3. **Leapfrog regression** — measured-creep FF + unbounded anti-hover monitor + zero-baseline hard gate. 4. **Arbiter deletion drops an encoded edge case** — fixtures-first rule; dropout decay-hold strictly more conservative than the envelopes it replaces; stage-4 equivalence + smoke. 5. **Rare far rests become final** — accepted per taxonomy, frequency-gated 1/40, main causes fixed at source. 6. **Offline false confidence** — replay gates invariants and direction only; feel gates only on-road with the honest method. 7. **Sprawl regrowth** — single writer, CI cap-function pin, incident-to-fixture doctrine; PID approach caps remain the flagged exception with a named follow-up.

**Key files** — new: `/Users/radoslawchybicki/Repos/openpilot-rch/selfdrive/controls/lib/stopping_service.py`, `stop_context.py`, `stopping_telemetry.py` · shrunk: `.../selfdrive/controls/lib/longcontrol.py`, `stopping_flags.py`, `.../longitudinal_mpc_lib/stop_target_helpers.py` · deleted (stage 4): `stop_target_arbiter.py`, `stopping_params.py`, `stopping_shadow.py`, `stopping_tracker.py`, `stopping_profile_selector.py`, `stopping_controller_v2.py`, `stopping_trajectory.py`, `stopping_guard.py` (all under `.../selfdrive/controls/lib/`) · eval: `/Users/radoslawchybicki/Repos/openpilot-rch/tools/stopping/{sim_replay.py, build_event_store.py, paired_stats.py, check_leapfrog_alignment.py}` · acceptance seam: `.../selfdrive/controls/lib/tests/test_stopping_v2_replay.py`.