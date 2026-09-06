# Stopping rollout plan (living document)

Owner: the user. Supervised L2 (2022 Santa Fe HEV, driver always responsible); deploys are
always allowed. Hard rule inherited from [on_vehicle_protocols.md](on_vehicle_protocols.md)
section 0: **one variable changes per stage**, deployed alone; revert = flip back +
`./fullupdate.sh` + verify hash.

This plan is the operative sequencing authority for the stopping stack. It supersedes the
original gate-first sequencing precondition in on_vehicle_protocols.md (re-scoped 2026-06-10:
stages 3–4 apply to whichever controller is active — see Rationale).

## Status table

Append one line per status change (see Adjustment rule). Dates are the day the status changed.

| Stage | What changes | Entry criteria | Exit criteria | Status | Date |
|---|---|---|---|---|---|
| 0 — Redesign deploy | Arbiter live (behavior-neutral), V2 dark, telemetry v2 live, shouldStop hold live, hold-acq soften + hill-hold repair, dRel-honesty flip (`PUBLISH_TRUE_LEAD_DISTANCE = True` at device ISD = 0.0) | — | Deployed, hash verified | done | 2026-06-10 |
| 1 — Baseline corpus | Nothing (2–3 normal drives, NO flag changes) | Stage 0 deployed | ≥ 40 honest-telemetry stop events; plant refit stable pole in (0,1); estimator_equivalence passes | **active** | 2026-06-10 |
| 1 — Baseline corpus | (cycle check, no flag changes; 12 routes / 225 segs ingested) | — | 29/40 honest tv2/sv2 events; refit pole PASS (corrected fitter: delay 9, φ=0.791, RMSE ratio 1.086 ≤ 1.1 — archive plant_model_20260612_refit.json); estimator_equivalence PASS — exit **NOT met** (event count only) | **active** (continues) | 2026-06-12 |
| 1 — Baseline corpus | (cycle check, +9 routes / 122 segs ingested) | — | **EXIT MET**: 65/40 honest tv2/sv2 events (store 194 = 129 sv1 + 65 sv2); refit_20260613 pole PASS (interior optimum, φ ∈ (0,1)); estimator_equivalence PASS — all three criteria pass (rollout_plan.md line 19/77-86) | **done** (exited) | 2026-06-13 |
| 2 — Gate re-run + V2 flip decision | `USE_STOPPING_V2` (only if gate passes) | Stage 1 exit | Gate deterministic exit (eval.md §5) → flip + 1–2 clean validation drives; or documented triage/re-scope decision | gate FAIL — V2 stays dark; pending triage | 2026-06-13 |
| 3.A — StopReq stage A | gate `0.01 → 0.04` | — | validated SAFE (390054594e; 13 holds, no EPB/faults, latch never held rolling) BUT IMU baseline showed it does NOT improve terminal comfort (median 23.7 vs legacy 26.7 m/s³, p=0.62) → **superseded**: gate set back to 0.01 to return the terminal to openpilot (comfort experiment below). StopReq A→B→C promotion shelved (was predicated on the SCC stop being smoother — disproven by IMU) | rolled back to 0.01 | 2026-06-13 |
| 3.C1 — Comfort: unnecessary harsh approach | P1 kinematic approach-decel cap (≤ 0.5 m/s² while lead gap > 2 m, released when kinematically required) in longcontrol + stopping_controller | Stage 1 exit; measurable (command-based, engaged-masked, validated) | Approach-harsh rate ↓ on-road over subsequent cycles; no under-braking | **REVERTED + CLOSED (v1 off, no v2 built — harshness is planner-owned).** v1 disabled 2026-06-15 (`APPROACH_DECEL_CAP_ENABLED=False`) after it under-braked into a decelerating lead and caused ≥2 high-speed takeovers (00001725 seg8, 00001726 seg15) + 3 driver-brake corrections (two root flaws: `required_decel` ignores lead decel so it never releases; no upper-speed bound). A safe-by-construction v2 (trim DEEPER toward aTarget, never SHALLOWER than aTarget) was scoped but is **NOT viable**: corpus re-analysis WITH the planner aTarget shows 87% of unnecessary-harsh-approach frames have command == aTarget (planner-owned); the only command-deeper-than-aTarget frames are message-interleave timing skew, the Santa Fe alignment margin, or the low-speed terminal lane (aTarget not the reference by design). A never-shallower cap would have ~nothing to trim. Gentler approaches need planner/MPC comfort tuning (out of stopping-stack scope), not a downstream cap. No controller change | 2026-06-15 |
| 3.C2 — Comfort: terminal disc-grab | IMU metric (gating; v4 2026-06-14: faithful `settle_peak_imu_decel` > 0.80 m/s² ∨ filtered raw-100 Hz jerk > 13.0 m/s³ — the v3 held-100 Hz jerk > 30 was a rate-aliasing artifact, now diagnostic) + StopReq gate 0.01 (openpilot owns terminal) | IMU wired (livePose.accelerationDevice.x + raw accelerometer) | reduce IMU settle decel/grab; crank thresholds down | **gate-0.01 standstill hold CLEAN (keep gate 0.01); handoff A/B INCONCLUSIVE-LEANING-AGAINST** (the lone gate-0.01 engaged stop on 00001722 read settle_imu_jerk 48.0 m/s³ — HARSHER than the 0.04-handoff baseline median 23.7 / p90 36.3 / max 39.6, not gentler → handoff is not the culprit, confirms prior finding). n=1 vs n=10, population-mismatched (driveway crawl vs rolling traffic). Gate 0.01 confirmed as the enabler → build the anti-stiction terminal pre-release next | 2026-06-14 |
| 3.C2 — Comfort: terminal disc-grab | Anti-stiction terminal pre-release (ease to −0.30 floor, jerk-limited, 0.06–0.30 m/s; full hold re-applied <0.06; gated off on disturbance/grade/arrest/lock) | gate 0.01 + IMU metric | IMU settle jerk ↓ on-road vs ~24 m/s³ baseline; no grade-stop creep | **DEPLOYED** (44371049fc; sim safe-with-watch, no fatal; felt effect = on-road IMU next drive). **Needs an ENGAGED rolling-traffic drive** to measure grab + confirm P1 + the gate-0.01 hold (06-14 routes were unengaged) | 2026-06-14 |
| 3.C — Comfort track | **First ENGAGED drive on the consolidated SAFE stack** (no flag changes; P1 off, pre-release off, gate 0.01, planner owns approach) | — | **Under-braking RESOLVED (cmd tracks aTarget, \|cmd−tgt\|<0.10 for 98–100% of approach samples, no takeover on 6 routes); comfort ACCEPTABLE** (approach_necessary=False all stops, hard_decel_dur 0.0; holds clean creep 0.042 m/s; terminal grab 0/2 over PRIMARY gate 0.80, vals [0.404,0.476]). Engaged data thin (only 0000172c driven engaged; n=2 distinct IMU stops seg3+seg18). One honest nuance: seg3 secondary jerk_raw 41.1 > 13.0 (shallow sharp transient, not a hard grab; pre-release that targets it is OFF by design). No new controller change — comfort program rests | **good resting state** | 2026-06-16 |
| 3.C2 — Comfort: terminal disc-grab | Anti-stiction terminal pre-release **GUARDED OFF** (`TERMINAL_PRERELEASE_ENABLED=False`) | post-P1 audit | re-enable only after an explicit live-aTarget re-clamp inside the controller OR extending the lead-aware cap down through (0.06,0.12) | **DISABLED (kill-switch staged, not yet committed).** Post-P1 audit found the pre-release shares P1's exact structural blind-spot class: its firing gate is lead-blind AND aTarget-blind, it RAISES the command toward −0.30, runs AFTER the static `clip(…,stop_accel=−2.0)` with NO live-planner re-clamp, and the only lead-aware downstream net (`low_speed_close_lead_accel_cap`) is gated v≥0.12 — leaving the v∈(0.06,0.12) band lead-blind with the cap OFF. Field sweep found 0 P1-signature frames, but that is an EMPIRICAL regularity (planner terminal demand has always relaxed shallower than −0.30 by settle), not a structural guard — the same "clean sweep over a hole" that let P1 ship. On supervised L2 we do not keep a lead-blind brake-REDUCING shaper live on empirical safety alone. The felt grab it targeted is actuator stiction (command lever ~exhausted) | 2026-06-15 |
| 4 — Dynamic SCC14 jerk | `DYNAMIC_SCC14_JERK` | StopReq settled (any sub-stage we choose to hold at) | Commanded-vs-realized tracking improved or neutral, no faults | pending | — |
| 5 — Cleanup | Scheduled deletions (architecture.md §5) | V2 live + ≥ 2-week soak | Cleanup commit landed, tests green | pending | — |

## Stage 1 — Baseline corpus (active)

2–3 normal drives, **no flag changes**. Purpose: the identification corpus must reflect the
current actuator path (post-redesign telemetry, post-soften hold acquisition, post-dRel-flip
signals) before any CAN-semantics change pollutes it.

Goals:

- ≥ 40 stop events ingested with honest telemetry: build with `--telemetry-version 2
  --signals-version 2` (accel_cmd sourced from `carOutput.actuatorsOutput.accel`, the sent
  value; signals_version 2 marks post-dRel-flip routes — both flags are CLI-declared, eval.md §1).
- `hold_acq_peak_cmd_jerk` before/after comparison vs the pre-soften corpus (non-gating
  diagnostic, eval.md §1). The current 129-event store has it null everywhere — re-scan the
  pre-soften driveway/era routes (stable keys make re-scans safe) to populate the before-arm.
- Verify go-signal behavior in traffic: clean departing-lead releases and green-light launches,
  no held brake after the lead departs, no stopping-state chatter.

Per-drive report procedure (run after every drive; exact commands, repo root):

```bash
# 1. Pull new segments into the shared cache (rlogs required; fallback --host commawifi)
#    LOGS ONLY — always use this script (or build_event_store --fetch-missing-rlogs).
#    Never rsync whole segment dirs: that drags fcamera/ecamera/qcamera video along (GBs).
python tools/route_sync/refresh_routes.py --host comma --include-rlog --newest-first

# 2. Ingest the drive into the main event store
python tools/stopping/build_event_store.py --route <route> \
  --signals-version 2 --telemetry-version 2 --fetch-missing-rlogs

# 3. Error sweep
python3 -c "import json,pathlib;m=json.load(open(pathlib.Path.home()/'.comma/stopping_behavior/event_store/manifest.json'));f=[x for r in m['fetch_report'] for x in r.get('failures',[])];print('fetch failures:',f or 'none');print('records written:',m['records_written'],'store total:',m['store_totals']['total'])"

# 4. Metric summary for the drive (set R to the route name)
python3 -c "
import json,statistics,pathlib
R='<route>'
p=pathlib.Path.home()/'.comma/stopping_behavior/event_store/events.jsonl'
evs=[e for e in (json.loads(l) for l in open(p)) if e['key']['route']==R]
m=[e['metrics_100hz'] for e in evs]
gap=[x['final_lead_gap_m'] for x in m if x['final_lead_gap_m'] is not None]
acq=[x['hold_acq_peak_cmd_jerk'] for x in m if x.get('hold_acq_peak_cmd_jerk') is not None]
print('n:',len(evs),' qlog-fallback:',sum(e.get('rate_class')=='qlog10' for e in evs),' tv2/sv2:',sum(e['telemetry_version']>=2 for e in evs),sum(e['signals_version']>=2 for e in evs))
print('end_stop_jerk med/max: %.3f / %.3f'%(statistics.median(x['end_stop_jerk'] for x in m),max(x['end_stop_jerk'] for x in m)))
print('rollout med: %.2f'%statistics.median(x['rollout_from_2mps_m'] for x in m),' hold_gap:',[round(g,2) for g in gap])
print('hold_acq_peak_cmd_jerk:',[round(a,2) for a in acq] or 'none')
"
```

Then append one line to the status table (Adjustment rule) and a worklog entry if anything is
anomalous.

Exit criteria (both required):

1. Plant refit on the new corpus yields a stable pole:
   `python tools/stopping/fit_plant_model.py --baseline-json
   docs/stopping/archive/plant_model_20260531T075153Z_all.json --output
   ~/.comma/stopping_behavior/analysis/plant_refit_<date>.json` — the `a_ego_prev` coefficient
   (φ) must lie in (0, 1) at the fit dt, and the printed acceptance row must pass (holdout RMSE
   ratio ≤ 1.1; leapfrog-alignment recall ≥ current per tools README).
2. `python tools/stopping/estimator_equivalence.py --tau-s 0.0` passes on the refreshed store
   (100% onset / 100% Jaccard bars, eval.md §4).

## Stage 2 — V2 similarity gate re-run + flip decision

Entry: Stage 1 exit (the re-run needs the refit plant and the honest corpus).

1. Archive the new refit in `docs/stopping/archive/` and re-run the gate on dual plants:
   `python tools/stopping/similarity_gate.py --plant both --output-json
   ~/.comma/stopping_behavior/analysis/similarity_gate_<date>.json` (attach the estimator
   artifact via `--estimator-report-json`).
2. **If the deterministic exit passes** (eval.md §5: Tier 1 all green on both plants, zero
   class-C events, triage table committed in `docs/stopping/archive/similarity_<date>.md` in the
   flip commit): flip `USE_STOPPING_V2 = True` (one line, `selfdrive/controls/lib/longcontrol.py:55`),
   update `test_use_stopping_v2_kill_switch_defaults_dark`
   (`selfdrive/controls/lib/tests/test_longcontrol_fast_release.py`), deploy, run the
   first-drive checklist (on_vehicle_protocols.md §4), then 1–2 validation drives before
   declaring soak start.
3. **If it fails**: per-event triage per the eval.md §5 taxonomy (spec-7.7 classifications via
   `--triage-json`); decide between targeted provenance-legal class-B parameter moves
   (re-run the gate after each) and re-scoping the failing Tier-1 rows (a logged decision, not a
   silent threshold edit). The 2026-06-10 failure analysis (eval.md §6) is the starting point.

The V2 flip is **independent of stages 3–4** — StopReq and SCC14-jerk changes live in the
carcontroller, downstream of the controller dispatch, and apply to either controller.

## Stage 3 — StopReq staged enablement (SHELVED 2026-06-13)

> **Shelved.** Stage A (gate 0.04) was deployed and validated SAFE on-road (13 holds: no
> EPB/`AVH_LAMP`/`PBRAKE_ACT`, no faults, latch never held on a rolling car), but the IMU
> terminal-grab baseline showed the SCC managed stop is **not** smoother than openpilot's
> (median 23.7 vs 26.7 m/s³, p=0.62) — which was the premise for promoting StopReq toward
> earlier handoff. The A→B→C promotion is therefore shelved. The gate has been **rolled back to
> 0.01** (legacy) under the comfort track (Stage 3.C) to return the terminal settle to openpilot.
> The sub-stage table below is retained for reference / possible revival, not as the active plan.

Entry: Stage 1 exit. One sub-stage per drive session; advance only on a clean drive report.
Constants in `opendbc_repo/opendbc/car/hyundai/carcontroller.py`; full background and promotion
criteria in [on_vehicle_protocols.md](on_vehicle_protocols.md) §1 (authoritative). Sub-stages
(exact values):

| Sub-stage | Change | Promotion criteria (per protocol §1) |
|---|---|---|
| 3.0 | `STOPREQ_LATCH = True` @ gate 0.01, release 0.10 | ≥ 10 stops + 1 × 60 s hold + 1 deliberate creep-push/hill stop with the latch active (StopReq must drop as the car rolls); no TCS13 `ACCEnable != 0`, no `PBRAKE_ACT`, no TCS15 `AVH_LAMP`; clean launch |
| 3.A | gate → 0.04, release 0.10 | same as 3.0 |
| 3.B | gate → 0.10, release 0.12 | same as 3.0 |
| 3.C | gate → 0.35–0.50, release = gate + 0.05 | same as 3.0 + one EPB/auto-hold watch ≥ 60 s, launch verified |

**2026-06-13: staged at sub-stage A** (`STOPREQ_LATCH = True`, gate 0.04, release 0.10 — both the latch
and the gate move in this one flip, per protocol §1's stage-A row which is defined relative to the
never-deployed stage-0 latch enablement). The protocol's stage-0 sub-stage was skipped: stage 0 and
stage A differ only in the assert gate (0.01 vs 0.04), and 0.04 is still below the 0.104 m/s
wheel-standstill threshold, so the wheels are provably stopped at the assert point — the conservative
standstill-only property of stage 0 is preserved while covering the full Kalman-dither band. The
always-active speed release (F1) is unchanged at 0.10. Parking-lot session required before any normal
drive (3 stops + 60 s holds + 1 deliberate creep-push, watching TCS15 `AVH_LAMP`, TCS13
`PBRAKE_ACT`/`ACCEnable`, cruise faults). Single behavioral variable this stage = the StopReq assert
semantics (gate + latch); no controller-dispatch change (V2 stays dark).

Each sub-stage: flip the named constant, deploy, verify hash. **Parking-lot session first**:
3 stops + 60 s holds, watching TCS15 `AVH_LAMP`, TCS13 `PBRAKE_ACT` / `ACCEnable`, and cruise
faults in the drive report. Then normal drives until the promotion criteria are met. Rollback =
revert the constant + deploy. `STOPREQ_RELEASE_SPEED` stays active at every sub-stage (the latch
may never hold StopReq on a rolling car).

## Stage 3.C — Comfort track (ACTIVE)

The active stopping work, driven by the user's felt requirement (eval.md §2.1): two distinct harsh
events per stop must be measured, then the requirement cranked, then iterated. Runs on whichever
controller is active (legacy today; V2 stays dark). One variable per deploy; the two sub-tracks
target **disjoint regimes** (approach vs terminal settle) measured by **disjoint metrics**, so they
attribute cleanly even when both are live.

**P1 — unnecessary harsh approach.** *Status: REVERTED + CLOSED (2026-06-15). v1 cap off; no v2 — the
harshness is planner-owned, so a safe downstream cap is the wrong tool.*

The v1 kinematic cap (`floor = −max(0.5, required+margin)`, `required = closing²/(2·max(gap−2,ε))`,
in stopping_controller final clamp + longcontrol post-cap) was **disabled 2026-06-15**
(`APPROACH_DECEL_CAP_ENABLED=False`, longcontrol.py + stopping_controller.py) after it caused a
near-collision takeover: on route 00001725 seg8, following a decelerating lead at ~15 m/s, the
planner asked aTarget=−1.78 m/s² but the cap pinned the command at −0.50 for 8.3 s (gap 27→7.5 m,
still closing) and the driver took over. **Two root flaws:** (1) `required_decel` ignores the lead's
own deceleration (`aLeadK`), so it never released against a braking lead; (2) the longcontrol gate
has no upper-speed bound, so it was live during fast following, not just low-speed stop approaches.
A safety sweep of the two cap-live routes (1725 + 1726) found this was **not a one-off**: a second
essentially-identical high-speed takeover (1726 seg15, planner −2.08, cap pinned −0.50 for 2.5 s,
v ~12.8 m/s) plus 3 more driver-brake/disengage under-brake corrections — every one from the same
two flaws. The revert was justified.

**Why no v2.** The task asked for a safe-by-construction v2 that may only trim braking DEEPER toward
the planner's aTarget and NEVER command shallower than aTarget — but that is only useful if the
unnecessary harshness is added DOWNSTREAM of the planner. Re-analysis of the full pre-cap corpus
(233 events) WITH the planner aTarget per frame (the June diagnosis only had command vs kinematic
required, not aTarget) shows it is NOT: **87% of unnecessary-harsh-approach frames (83,305 / 95,803)
have command == aTarget within 0.10 m/s²** — the PLANNER asked for the deep decel and the PID command
faithfully tracked it. The command-deeper-than-aTarget bucket (3.9%) is almost entirely (a)
message-interleave timing skew where the command tracks aTarget through a fast transient (79–100% of
deeper frames match an aTarget value within an 80 ms window), (b) the Santa Fe
`pid_brake_model_alignment` margin (0.03–0.18 m/s², lines 152–170), or (c) the low-speed terminal
stopping lane (v < ~1 m/s) where aTarget is intentionally NOT the reference. Genuine sustained PID
free-following downstream over-braking is ~0. June's "command 0.59–0.85 vs required 0.02–0.34" events
re-checked WITH aTarget have aTarget within ~0.00 of the command (the planner asked modestly harder
than strict gap-2 kinematics need — not a downstream stage adding braking).

So a never-shallower-than-aTarget cap would have **essentially nothing to trim**: the planner is the
source of the deep demand, and making those approaches gentler requires commanding SHALLOWER than
aTarget — exactly the unsafe under-braking the reverted cap did. **Conclusion: approach braking
belongs to the planner.** Gentle-approach comfort, if still wanted, must be addressed where the
harshness lives — in the longitudinal MPC comfort/jerk-accel cost weighting or the lead/stop-distance
shaping inside the planner (out of the stopping-stack scope), where it is just as exposed to the
closing-lead threat that broke the cap and is NOT safe-by-construction. **Recommendation:** keep the
cap OFF, make no controller change, and focus the comfort program on the terminal pre-release (P2),
which is genuinely command-shapeable and where the open work already lives.

**P2 — terminal disc-grab.** *Status: measurable + experiment deployed (57def50ac8).* The grab
(static-friction jolt as the car settles to rest) is now measured on the device IMU
(`livePose.accelerationDevice.x` → `a_long_imu`; wheel-`aEgo` is blind below ~0.03 m/s). **2026-06-14
(scoring v4):** `harsh_terminal_grab` RE-WIRED off the FAITHFUL IMU channels — `settle_peak_imu_decel`
> 0.80 m/s² (PRIMARY, robust; cross-sensor Spearman 0.97) OR filtered raw-100 Hz `settle_peak_imu_jerk_raw`
> 13.0 m/s³ (SECONDARY) — after the v3 metric `settle_peak_imu_jerk` (held-100 Hz, cap 30) was found to
be a rate-aliasing artifact (held 20 Hz value diffed across the 0.01 s carState step → manufactured
~70 m/s³ spikes; now a non-gating diagnostic). Corpus re-baseline: old v3 12/114 (10.5%) → new v4 10/114
(8.8%); 3 artifact false alarms dropped (incl. `00001717#3`, artifact 70.2 but decel 0.62), 1 real
sub-100 ms grab added, 9 kept incl. the felt `00001722` driveway grab (decel 0.99). **Finding:** the SCC
handoff is NOT the culprit (grab pervasive in both the 0.04-handoff and legacy arms, p=0.62), so it
reads as friction-transition physics, not a command/handoff artifact. **Gate rolled back 0.04→0.01**
so openpilot owns the terminal decel again — the prerequisite for shaping it.

  **Next steps (P2), in order:**
  1. ~~**This/next drive (gate 0.01):** parking-lot watch — clean managed standstill hold, no creep
     forward, no ACC dropout / cruise fault at the stop.~~ **DONE 2026-06-14 — gate-0.01 standstill
     hold is CLEAN, keep gate 0.01.** Across all gate-0.01 stops (route 00001722 driveway test + the
     rolling stops): `accFaulted` (TCS13 `ACCEnable != 0`) = 0 on every segment; no creep away from an
     established hold (the ~0.058–0.067 m/s vEgo wander on the no-gas plateaus is sub-0.1 m/s
     Kalman/wheel-band dither at the terminal of a crawl, not creep-from-hold; the earlier "~0.31 m/s
     creep" was a measurement artifact that captured the legitimate re-launch ramp); the only
     "ACC.enabled==False while engaged" frames are ~40 ms engagement rising-edge transients, not
     standstill dropouts. **The inverse risk of lowering the gate (car fails to get the SCC managed
     hold) did not materialize.** Note: on the driveway test the car re-launched at each crawl, so
     there was no long SCC-managed standstill to creep out of — the on-road managed-hold-cleanliness
     evidence is from the rolling stops + prior StopReq-A validation, not from 722.
  2. **Build the anti-stiction terminal pre-release (NOW ACTIVE — this is the next deploy).** The
     causal A/B closed against the handoff hypothesis: giving openpilot the terminal decel down to 0.01
     did NOT reduce the grab — the one gate-0.01 engaged stop produced `settle_peak_imu_jerk = 48.0`
     m/s³, the harshest in the store, above the 0.04-handoff baseline p90 (36.3) and max (39.6). So the
     grab is friction-transition physics that openpilot can shape command-side, NOT an SCC-handoff
     artifact — exactly the regime the pre-release targets, and gate 0.01 now makes it possible because
     openpilot owns the terminal command instead of handing off at 0.04.
     - **Design:** as commanded `v` approaches a small ε above zero (trigger ~0.15–0.25 m/s, while
       `longControlState == stopping` and a real stop is committed), ease the brake command up by a
       bounded amount (a partial, rate-limited release of accel toward ~−0.2…−0.3 m/s² floor — never
       to zero, never positive) so the disc/pad and suspension unload gradually instead of biting as
       static friction grabs at v→0. Then, once the car is at rest (wheel-standstill confirmed, vEgo <
       ~0.05), re-apply the hold decel (ramp back to the standstill clamp) and, below the 0.01 gate,
       hand the sustained standstill to StopReq/SCC. The pre-release is a brief shaped notch in the
       terminal 0.25 m/s, bounded in depth and duration, rate-limited on both edges.
     - **Adversarial invariants (must all hold in sim before deploy):** never lets the car creep or
       roll forward on grade (re-apply must win on any positive vEgo / downhill); never delays the stop
       (rollout-from-2 m/s budget unchanged); never fails to reach the standstill hold; degrades to the
       current behavior if the lead closes (P1-style kinematic-necessity override — a hard close
       cancels the pre-release). Sim-validate against `settle_peak_imu_jerk` (target: drop the 100 Hz
       held-IMU spike below the baseline median) and the existing hill-hold / StopReq-A fixtures.
  3. **Crank** the 30 m/s³ gate down (toward 25 / 22) as the pre-release lowers the measured grab;
     iterate over drives (sim develops, the IMU measurement promotes). Needs a larger gate-0.01 engaged
     settle population than this cycle's n=1 (ideally rolling-traffic stops to match the StopReq-A
     baseline) to read the pre-release's on-road effect with any confidence.

     **Dev accelerator (2026-06-14): the friction-augmented plant.** A coarse-provisional velocity-curve
     friction residual (`fit_friction_residual.py`, archived `friction_residual_20260614.json`) now lets
     `sim_replay.py --friction` predict an IMU channel offline so the pre-release can be iterated in sim
     instead of burning a full drive per knob change (eval.md §5.1). **It is a DEVELOPMENT ACCELERATOR
     ONLY — never a gate; the on-road IMU `settle_peak_imu_jerk` still promotes P2.** First read
     (`rescore_prerelease_friction.py`): the deployed pre-release shows **no material change** in the
     predicted grab vs OFF, and the A/J knob sweep is flat — *as expected*, because the residual is
     velocity-only and does not model the stiction-relief mechanism the pre-release targets. So the sim
     **cannot prove the pre-release helps**; it confirms where the grab lives and ranks settles, and the
     felt-grab reduction remains an on-road measurement. The fit **grows with engaged drives** (re-fit as
     the corpus grows; `c0`/`c1` are still on their priors at n=26).
  4. If command-side shaping can't move it (the grab is genuinely brake-hardware stiction), document
     that ceiling — it may be irreducible without an SCC/firmware change (out of scope).

## Stage 4 — Dynamic SCC14 jerk

Entry: StopReq settled — at whichever sub-stage we choose to hold (3.C is not a prerequisite).
Flip `DYNAMIC_SCC14_JERK = True` alone (carcontroller.py), deploy, parking-lot session first,
then normal drives. Measure commanded-vs-realized accel slew on identical stops — now possible
with honest carOutput telemetry (telemetry_version 2 events compare the **sent** accel against
`aEgo`). Alert conditions and details: on_vehicle_protocols.md §2 (realized accel rate-limited
below planned jerk, or any `ACCEnable != 0` ⇒ revert + record). Exit: tracking improvement or
neutral with no faults.

## Stage 5 — Cleanup (after V2 soak)

Entry: V2 live ≥ 2 weeks of good drives (paired on-road stats per eval.md §3, no harsh/leapfrog
regression). Execute the scheduled deletions per [architecture.md](architecture.md) §5: the
forest (`stopping_controller.py`, `stopping_shadow.py`, `stopping_profile_selector.py`), the
legacy plant-model tools, the legacy tests, the `run_stopping_cycle.py` `DEFAULT_WORKLOG` flip
to `docs/stopping/worklog.md`, and the dropout-predicate re-pin (`ARBITER_LEGACY_DROPOUT_HOLDS`
retirement requires ≥ 25 soak dropout events with `hold_divergence == 0`).

## Rationale

- **Corpus before CAN changes (stage 1 first).** The plant refit and gate re-run are
  identification work — the data must reflect the current actuator path. Mixing a StopReq or
  jerk-semantics change into the corpus window would confound both.
- **StopReq before dynamic jerk (stage 3 before 4).** StopReq root-causes the terminal clamp
  behavior and has a designed, staged probe protocol with explicit promotion criteria; the jerk
  change is an optimization layered on a settled StopReq state.
- **dRel flip at stage 0.** Device read 2026-06-10: `IncreasedStoppedDistance == 0.0` (all
  weather variants 0), so the flip is bit-identical today; any later ISD raise is compensated by
  design (single-point `lead_d_rel_eff`, rest-gap equality pinned for ISD ∈ {0, 1.5, 3.0}).
- **Stages 3–4 independent of the V2 flip.** Both constants live in the carcontroller,
  downstream of controller dispatch; gating them on the V2 flip (the original protocol
  precondition) would serialize unrelated work behind a gate that honestly failed.

## Decision log (append-only)

| Date | Decision |
|---|---|
| 2026-06-10 | Redesign deployed: arbiter live (frame-exact equivalence vs base 3be25f5240), V2 merged dark, telemetry v2 live, shouldStop falling-edge hold live |
| 2026-06-10 | Similarity gate run on dual plants → NOT passed → `USE_STOPPING_V2` stays `False` (eval.md §6; escalated per the class-C/R2 rule) |
| 2026-06-10 | Hold-acquisition soften + hill-hold repair live (parameters.md row 40; driveway route `00001702--dcdc5c3eea--0`) |
| 2026-06-10 | dRel-honesty flip: `PUBLISH_TRUE_LEAD_DISTANCE = True` at device ISD = 0.0 (bit-identical, verified by device read + replay) |
| 2026-06-10 | This rollout plan adopted; StopReq/jerk stages re-scoped as independent of the V2 flip |
| 2026-06-12 | Cycle (12 routes, 225 segs): +29 honest tv2/sv2 stop events (store 158 total; all rlog100, accel_cmd_source=carOutput). Stage 1 exit NOT met — event count 29/40 is the sole blocker → **stage 1 continues, no flips** |
| 2026-06-12 | Plant refit 20260612: initial run (max-delay 8) hit its sweep ceiling (delay 8, φ=0.918, ratio 0.898); fitter then hardened (unstable-pole guard — fired on real data at delays 6–7; default max-delay 15) and re-run: delay 9 (0.9 s dead time, interior optimum), φ=0.790897 ∈ (0,1), holdout RMSE 0.04546 vs baseline 0.04187 (ratio 1.086 ≤ 1.1) PASS — archived as docs/stopping/archive/plant_model_20260612_refit.json; estimator_equivalence PASS (100% onset / 100% Jaccard). Acceptance verdict unchanged by the correction |
| 2026-06-12 | Similarity gate re-run (deck 189, dual plants ref_20260514 + refit_20260612, scoring v1): **NOT passed** — tier 1 FAIL 4/8 on frozen plant (leapfrog v>2\|explicit 21>20; rollout Δp95 0.886 m > 0.15; hold_gap Δp95 2.221 m > 0.10 [new fail, was 0.070 Jun-10]; time_to_standstill Δp95 2.68 s > 0.5) → V2 stays dark |
| 2026-06-12 | Errors sweep: NONE attributable to the stopping stack (0 tracebacks/CRITICAL across 225 segs; 17 engaged stopping intervals all clean) → advancement not frozen. Watch items (non-stopping): kernel-stall worsening (24.6 s sysfs reads, mid-drive softDisable on 00001714); deep_rl3 unexplained highway decel on 00001714 |
| 2026-06-12 | Hold-acq diagnostic: metric flaw — `hold_acq_peak_cmd_jerk` window unmasked vs takeovers, so 100+ values in both arms are artifacts (fix task filed). Artifact-free: NO improvement signal post-soften (active-only median 5.11 n=18 vs 7.96 n=5, non-conclusive); stop-tail small median worsening (p=0.024, diagnostic) — monitor, non-gating |
| 2026-06-12 | paired_stats old-vs-new REFUSED (n_after=29 < 150/arm floor; structural: signals_version in stratum key makes sv1-vs-sv2 pairing refused-by-construction). Within-era sv2 accumulation is the usable path; any cross-era rule must be decided in eval.md first |
| 2026-06-12 | Cross-era comparison rule DECIDED (eval.md §3.1): signals_version dropped from the stratum key IFF every event in BOTH arms has entry.isd_m == 0 (device runs ISD = 0.0, so v1/v2 lead-gap semantics coincide bit-for-bit). Measurement comparisons only — gates stay same-era; power floors unchanged; no scoring_config bump. Re-run end_stop_jerk 129-vs-29 with rule active: still REFUSED for power (correct outcome) but with real numbers — pooled Δmedian +0.260 m/s³, 95% CI [+0.126, +0.549], mde_at_n 0.152 m/s³ |
| 2026-06-13 | Cycle (+9 routes 00001715..0000171d, 122 segs): +36 honest tv2/sv2 stop events → **honest count 65/40, stage-1 event floor CLEARED** (was the sole 29/40 blocker on 2026-06-12). Store 194 (129 sv1 + 65 sv2); all rlog100, accel_cmd_source=carOutput, tv≥2 ∧ sv≥2. Honest controller_commit spread: 2ee311b407 ×29, 5076fcb9b5 ×24, d08f2033ac ×5, d19a7abc8d ×5, 8a19a13925 ×2 — all post-29219fc51d (same sv2 era) |
| 2026-06-13 | Plant refit (refit_20260613) PASS: stable pole, φ(a_ego_prev) ∈ (0,1), interior optimum (not a sweep-ceiling pin — the hardened fitter's unstable-pole guard + max-delay 15 default held). estimator_equivalence PASS. **Stage 1 EXIT MET** (all three criteria, rollout_plan.md line 19/77-86) |
| 2026-06-13 | Similarity gate re-run (dual plants ref_20260514 + refit_20260613): **NOT passed on EITHER plant** → V2 stays dark. ref_20260514: harsh PASS, leapfrog FAIL, rollout_p95 FAIL (0.863 > 0.15), hold_gap_p95 FAIL (2.335 > 0.10), end_jerk PASS, tts_p95 FAIL (3.02 > 0.5), integrated PASS, estimator PASS. refit_20260613: harsh FAIL, leapfrog FAIL, rollout_p95 FAIL (0.362 > 0.15), hold_gap_p95 FAIL (2.664 > 0.10), end_jerk PASS, tts_p95 PASS (0.0), integrated FAIL, estimator PASS. 163 tier-2 events remain unclassified (V2 flip blocked on triage, not on data volume) |
| 2026-06-13 | Errors sweep across all 9 routes: **NO new stopping-code-attributable issues**. No traceback/CRITICAL/cruise-fault/daemon-restart correlated with longControlState==stopping or any new path. Downhill-clip relax (523cd29fce) NOT exercised this cycle — all 9 routes ran commits BEFORE it (latest 2ee311b407 @09:57; downhill-clip landed @11:40), so its alert-correlation could not be tested; **needs routes on ≥ 523cd29fce next cycle**. Data corrections: 00001719 commit recovered as 8a19a13925 (rl3-combined-onnx, pre-deviceState-tolerance); 0000171a + 00001719 are PRE-deviceState-tolerance, only 0000171b/c/d carry the mitigation (relevant for the commIssue trend split) |
| 2026-06-13 | commIssue/kernel-stall trend: the two selfdrived deviceState stale-tolerance commits (2ee311b407 @09:57 + 21bdfe9567 @10:33) **materially reduced the kernel-stall signature** (mitigation of the symptom, not a kernel fix). Per engaged-minute: PRE-mitigation (00001717/18/19/1a, 20.2 min) = 1.39 thermal-stall/min + 1.39 commIssue/min; POST-mitigation (0000171b/c/d, 31.3 min) = 0.06 thermal/min (~23×↓) + 0.29 commIssue/min (~5×↓). Absolute thermal-stall: 28 (pre) → 2 (post) despite MORE engaged time. Clearest: 0000171d (22 engaged-min, longest of batch) logged 2 thermal + 3 commIssue vs 00001717 (pre-fix, 17 min) 19 thermal + 12 commIssue. Reverses last cycle's escalation. Caveat: confounded (worst pre-fix routes 17/18 are also heaviest pre-mitigation drives; AGNOS kernel root cause unchanged). commIssue persists at low rate (dominated by driverMonitoringState/DM service), non-engaged + non-stopping-correlated. softDisable while ENGAGED occurred ONCE in 9 routes: 00001718 seg8 @506.8s (commIssue, ~1s, vEgo~13 m/s, pre-mitigation) — brief comm dropout, far from any stop |
| 2026-06-13 | hold-acq diagnostic (eval.md §1, non-gating; tooling corrected per 2026-06-12 mask-to-active fix): masked metric in effect; no new hold-acq regression flagged this cycle. Quality-by-model: honest events span 5 controller commits (pre-deep_rl3 5076fcb9b5, deep_rl3 v1 d08f2033ac, rl3-combined-onnx 8a19a13925, deviceState-tolerance 2ee311b407, + d19a7abc8d) — all same sv2 actuator path; no per-model stopping-quality outlier surfaced in the error sweep. USER BOOKMARKs: none new flagged in this batch's brief |
| 2026-06-13 | **DECISION (PATH B per rollout_plan.md decision tree): refit.exit_met=true ∧ gate.passed=false → StopReq stage A staged.** Corpus/plant now established (actuator-path-identification ordering constraint satisfied), V2 flip blocked on tier-2 triage (163 unclassified), so the independent StopReq track is the next deployable stage. Flipped `STOPREQ_LATCH=True` + gate `0.01→0.04` (release 0.10 unchanged) in opendbc_repo/opendbc/car/hyundai/carcontroller.py per on_vehicle_protocols.md §1 stage A. test_can_bounds_fork.py updated to its staged-flip procedure (zero-delta proof split: dark/legacy state monkeypatched + byte-identical to oracle; stage-A live state proven to diverge from legacy ONLY in the SCC12 StopReq bit; new test_stage_a_live_gate_and_latch). 23/23 CAN-bounds + 13 Hyundai-platform tests green. Adversarial CAN safety review: **no fatal/major** findings (see worklog 2026-06-13). Deployed as ea1f2101f8 / 390054594e |
| 2026-06-13 (comfort) | **StopReq stage A on-road VALIDATED** (routes 0000171e + 0000171f, both on 390054594e, 13 engaged standstill holds): TCS15 AVH_LAMP and TCS13 PBRAKE_ACT False on every frame (no auto-hold/EPB), 0 cruise faults during holds, latch never held on a rolling car (rolling_with_latch=0, max vEgo under latch 0.000), terminal settle no worse than pre-A. The parking-lot watch items are clean from real driving → StopReq A locked in, stays on the car |
| 2026-06-13 (comfort) | **User comfort requirement.** Two felt harsh events confirmed from ground truth: (P1) unnecessary harsh approach — 4 of 6 over-0.5-m/s² approach-brake events were NOT kinematically required (slow/far/low-closing, required-decel 0.02–0.34 vs commanded 0.59–0.85); (P2) terminal disc-grab. Eval cranked to **SCORING_CONFIG_VERSION 2**: P1 `unnecessary_harsh_approach` GATING (gap-gated 0.5 m/s² cap with kinematic-necessity exemption, command-measured, engaged-masked; corpus baseline 68/131 = 51.9% engaged-fail). Controller P1 fix deployed: a single principled kinematic cap (cap = max(0.5, required-decel), released when the lead demands more — same physics as the eval exemption, so producer and gate agree; placed in both stopping_controller final clamp and longcontrol post-cap) + a conservative command-side terminal-settle deepening cap. Sim: approach-harsh ~halved on both plants, **no under-braking** (adversarial fast-close fixture reached full −2.0 authority), hill-hold/StopReq-A unaffected |
| 2026-06-13 (comfort) | **P2 terminal disc-grab held — NOT faithfully measurable yet** (the project's measurement-first rule). Wheel-derived `aEgo` quantizes/floors to ~0 at standstill → the static-friction grab at v≈0 leaves no wheel signature; and under StopReq-A the SCC owns the final stop so the command is blind too. `harsh_terminal_grab` therefore DEMOTED to a non-gating diagnostic (never gate on a blind metric). Data shows the command-side P2 lever is ~exhausted (commanded settle jerk already ≤1.5 m/s³; the felt excess is actuator stiction). **Next step before any P2 crank/iterate: wire an IMU long-accel channel (raw accelerometer ~101 Hz, or livePose.accelerationDevice ~20 Hz) into analyze_stopping_behavior.load_samples** so the grab becomes measurable; the eventual P2 fix likely leans on the StopReq-A SCC handoff, not deeper command caps |
| 2026-06-14 (comfort cycle) | **Gate-0.01 review cycle (3 new routes 00001720/00001721/00001722).** Commit/gate audit: 00001722 = 4c1230691b (gate 0.01, confirmed ≥ 57def50ac8 by git ancestry); 00001721 = 57def50ac8 (valid gate 0.01) but 0 events (fully unengaged); **00001720 = 5061019182 which PREDATES the gate-0.01 rollback → it is gate 0.04, NOT 0.01** (and both its events are ~0% engaged). So the entire new gate-0.01 engaged population is route 00001722. **(1) STANDSTILL HOLD @ gate 0.01 = CLEAN → KEEP GATE 0.01.** Zero `accFaulted` (TCS13 `ACCEnable != 0`) on every segment; no creep away from an established hold (sub-0.1 m/s Kalman/wheel-band dither on the no-gas crawl plateaus, not creep — the earlier "0.31 m/s creep" was a re-launch-ramp measurement artifact); only ~40 ms engagement-rising-edge ACC-disabled transients, no standstill dropout. The inverse risk of lowering the gate (failing to get the SCC managed hold) did not materialize. |
| 2026-06-14 (comfort cycle) | **(2) TERMINAL-GRAB CAUSAL A/B → handoff hypothesis NOT supported (inconclusive-leaning-against).** The single distinct gate-0.01 engaged settle (route 00001722; ev3/ev4/ev5 are ONE physical stop under three overlapping detector windows — counting n=3 triple-counts) produced `settle_peak_imu_jerk = 48.0` m/s³ and `settle_peak_imu_decel = 0.99` — HARSHER than the gate-0.04 StopReq-A baseline (re-derived from the store and reproduced exactly: jerk n=10 median 23.7 / p90 36.3 / max 39.6; decel median 0.59) and above the legacy median (26.7) and the 30 m/s³ gate. Giving openpilot the terminal did not move the grab toward zero; it read worse on this one stop. Confirms the prior finding (the SCC handoff is not the culprit; the grab is friction-transition physics). **Severe caveat:** n=1 vs n=10, and population-mismatched (gate-0.01 datum is a driveway standstill-engagement crawl, entry_v 0.86 / lead 3.2 m; baseline is rolling traffic) — directional red flag, not a statistical verdict. |
| 2026-06-14 (comfort cycle) | **(3) P1 on-road approach-cap confirmation = NOT CONFIRMED — insufficient data (population artifact).** Headline rate looks like it drops (pre-cap 22/30 = 0.73 → post-cap 1/3 = 0.33) but the two arms share NO overlap on any axis (pre-cap min worst_gap 3.24 m > post-cap max 2.30 m; post-cap entry_speed 0.86 m/s for all 3). All 3 post-cap engaged events are sub-1-m/s driveway crawls on 00001722, not on-road approaches; the two big routes contributed zero usable engaged approaches (00001720 gate-0.04 unengaged, 00001721 zero events). What the data DOES show narrowly-positive: on those 3 stops the cap introduced no under-braking (hard_decel_duration 0.00 s, gentle min_a_ego −0.63…−0.71, no cap-induced min-gap reduction, necessity exemption correctly let the harder-required commands through) and the cap mechanism behaves as designed. **A longer engaged rolling-traffic drive is required for the on-road P1 confirmation.** |
| 2026-06-14 (comfort cycle) | **DECISION — next step: BUILD THE ANTI-STICTION TERMINAL PRE-RELEASE (P2 step 2) now; in parallel, the next on-road drive must be a longer engaged rolling-traffic drive (for the open P1 confirmation AND to grow the gate-0.01 engaged-settle population beyond n=1).** Rationale: gate 0.01 holds cleanly (keep it) and the causal A/B closed against the handoff hypothesis, so the grab is command-shapeable and gate 0.01 is the enabler (openpilot owns the terminal). The pre-release is sim-developed (does not need more device data to start — sim against `settle_peak_imu_jerk` + hill-hold/StopReq-A fixtures), so it is not blocked on the thin gate-0.01 corpus; the corpus grows on the rolling-traffic drive that P1 also needs, which then measures the pre-release's on-road effect. P1 cap stays deployed (no under-braking evidence; confirmation deferred to that drive). Gate stays 0.01. No revert. |
| 2026-06-15 (comfort) | **P1 approach cap REVERTED + CLOSED — harshness is planner-owned, no v2 built.** The v1 cap was disabled (`APPROACH_DECEL_CAP_ENABLED=False`, 2681ba4670) after under-braking into a decelerating lead → near-collision takeover (00001725 seg8: planner −1.78, cap pinned −0.50 for 8.3 s). Safety sweep of both cap-live routes (1725+1726) confirmed it was NOT a one-off: a SECOND high-speed takeover (1726 seg15, planner −2.08, cap pinned −0.50 for 2.5 s, v ~12.8 m/s) + 3 more driver-brake corrections, all from the same two flaws (`required_decel` ignores lead decel → never releases; no upper-speed bound → live during fast following). Investigated whether a safe-by-construction v2 (trim DEEPER toward aTarget, NEVER shallower) is viable: re-analyzed the 233-event corpus WITH the planner aTarget per frame (June had only command-vs-kinematic, not aTarget). **Result: 87% of unnecessary-harsh-approach frames (83,305/95,803) have command == aTarget within 0.10 m/s² → planner-owned.** The command-deeper-than-aTarget bucket (3.9%) is almost entirely message-interleave timing skew (79–100% match an aTarget value within an 80 ms window), the Santa Fe alignment margin, or the low-speed terminal lane (aTarget not the reference by design); genuine sustained PID downstream over-braking ≈ 0. June's 0.59–0.85-vs-0.02–0.34 events re-checked WITH aTarget have aTarget within ~0.00 of the command (planner asked modestly harder than gap-2 kinematics, not a downstream stage). **DECISION: a never-shallower-than-aTarget cap would have ~nothing to trim (the planner is the source of the deep demand); making those approaches gentler requires going shallower than aTarget = the unsafe under-braking that caused the takeovers. So NO v2 cap.** Approach braking belongs to the planner; gentle-approach comfort needs planner/MPC comfort-cost tuning (out of stopping-stack scope) or nothing. v1 stays OFF; no controller change. Comfort program focuses on the terminal pre-release (P2), which is genuinely command-shapeable. |
| 2026-06-15 (safety audit) | **POST-P1 ADVERSARIAL AUDIT of the 3 OTHER live comfort changes → 2 KEPT (field-verified safe), 1 GUARDED OFF.** After the P1 cap revert, swept the legacy-controller comfort changes never adversarially field-swept for the P1-class failure mode (command brakes LESS than planner aTarget into a closing lead, or creep/rollback/under-hold at a stop). **(1) gate-0.01 StopReq latch (opendbc hyundai carcontroller.py, STOPREQ_LATCH=True, gate vEgo<0.01, release 0.10) → CLEAN, KEEP.** 134 engaged holds (14 gate-live): zero rollaway, zero ACC/cruise fault at standstill, latch never held StopReq on a rolling car (0.10 release fires correctly = it IS an upper bound, the thing P1 lacked). Code-confirmed it is a HANDOFF signal (SCC-managed vs openpilot command), not a brake floor — P1 root flaws do not translate; release only hands the hold back to openpilot's stopping command which keeps braking. Coverage caveat: gate-live full-stop data thin (14 holds, mostly route 1725); no grade-stop in the [0.01,0.10] dither band captured (settle then left to openpilot's command = legacy behavior, field-shown to hold). No revert. **(2) hold-acquisition soften (stopping_controller.py:2676-2695, HOLD_ACQUISITION_SOFTEN_*) → CLEAN, KEEP.** 705 traces / ~22.8k candidate frames: zero rollback/under-hold attributable. Code-confirmed it caps `brake_step` (the per-frame DEEPENING rate) ONLY — never `target`, never `release_step`; in `clip(target, last−brake_step, last+release_step)` a smaller brake_step only RAISES the lower bound, so it structurally cannot release brake or push below target. Hard v<0.05 upper bound; no floor, no release path → neither P1 root flaw applies. The 1 net-creep run (171c, 1.13 cm/s) was should_stop=False with the command DEEPENING into a departing lead (opposite of under-hold). No revert. **(3) anti-stiction terminal pre-release (stopping_controller.py:2828-2849, A_TERMINAL_PRERELEASE etc.) → GUARDED OFF (`TERMINAL_PRERELEASE_ENABLED=False`).** This change shares P1's EXACT structural blind-spot class: the firing gate is LEAD-BLIND and aTARGET-BLIND (no lead_v/d_rel/closing/planner term), it RAISES the command toward −0.30, runs AFTER the static `clip(…,stop_accel=−2.0)` with NO re-clamp against LIVE planner demand, and the only lead-aware downstream net (`low_speed_close_lead_accel_cap`, longcontrol.py:221/680-684) is gated v≥0.12 → leaving the v∈(0.06,0.12) band lead-blind AND the cap OFF. The field counterfactual found 0 P1-signature frames, but the audit itself flagged this as "a structural argument, not observed data": safety rests on an EMPIRICAL regularity (planner terminal demand always shallower than −0.30 by the final settle, slow ease never reaches the floor while a lead closes), NOT an explicit guard — the identical "clean field sweep over an unobserved structural hole" that let P1 ship and cause two near-collisions. No prerelease unit test exercises a closing lead. **DECISION: on supervised L2 we do not keep a lead-blind brake-REDUCING terminal shaper live on empirical safety alone; the felt grab it targeted is actuator stiction (command lever ~exhausted), not worth the P1-class risk.** Staged kill-switch `TERMINAL_PRERELEASE_ENABLED=False` (default-off, mirrors `APPROACH_DECEL_CAP_ENABLED`), tests force-enable the flag locally to keep the MECHANISM verified for an eventual re-enable; re-enable requires a live-aTarget re-clamp inside the controller OR extending the lead-aware cap down through (0.06,0.12). Targeted suite green (251 pass; 1 pre-existing UNRELATED failure on `low_rollout_soft_landing_cap` from foundational commit 3204868077, independent of all 3 changes — toggling the prerelease off does not change it). ruff zero-new (176==176, all pre-existing E501 on seed-data lines). NOT committed/deployed (orchestrator owns deploy) |
| 2026-06-16 (engaged baseline) | **FIRST engaged drive on the consolidated SAFE stack (6 routes 00001727..0000172c) — under-braking RESOLVED, comfort ACCEPTABLE, V2 triage still the only real work.** Read-only review. **Commit audit CLEAN:** every store event (0000172a, 0000172c) is on `controller_commit = 80e84c1f21…` = device HEAD (P1 off `APPROACH_DECEL_CAP_ENABLED=False`, pre-release off `TERMINAL_PRERELEASE_ENABLED=False`); none on f11ff331a1 or any earlier P1/pre-release-live commit. **Engaged data thin (the constraint):** only 0000172c driven engaged (868 s); the other 5 are effectively manual (engaged_s ~0.2, gasPressed). Honest IMU terminal-grab exists for exactly **n=2 distinct physical stops** (172c seg3 + seg18; the 5 settle records collapse to two values, verified in store). **(1) UNDER-BRAKING RESOLVED** (confirms the user's belief, as expected with P1 off): NO under-braking takeover on any of the 6 routes (88 qlog segs + 5 engaged rlogs); the P1 clamp-below-aTarget signature is GONE — carOutput accel_cmd vs longitudinalPlan aTarget at 100 Hz over every engaged approach (v>2, planner braking) shows \|cmd−tgt\|<0.10 for 98–100% of samples, cmd median == tgt median within 0.001–0.01 m/s². The command faithfully TRACKS the planner. **(2) COMFORT ACCEPTABLE, no new real issue:** approach (now planner-owned) comfortable not harsh (approach_necessary=False all stops, hard_decel_duration_s=0.0 everywhere, peak approach cmd −0.48…−0.90) — removing the cap did NOT reintroduce harshness; gate-0.01 standstill holds CLEAN on both true engaged holds (seg18 creep 0.042 m/s, rebound 0.0017 m/s); terminal grab PRIMARY `settle_peak_imu_decel` (gate>0.80) = [0.404,0.476] med 0.44 → **0/2 over**, gentler than StopReq-A baseline 0.59 and the felt 722 grab 0.99. HONEST NUANCE: SECONDARY `settle_peak_imu_jerk_raw` (gate>13.0) splits — seg18 5.5 (clean), seg3 **41.1 (over)** → under classify_event's OR logic seg3 would raise harsh_terminal_grab on the secondary channel; reads as a shallow sharp sub-100 ms transient (low decel), not a hard grab, and the pre-release that targeted exactly this is OFF by design (guarded for sharing P1's lead-blind blind-spot). Not a blocker, not a baseline regression; flagged for a larger engaged corpus to re-check. **(3) BOOKMARKS:** none across all 6 routes. **DECISION: the consolidated safe stack is in a good, safe, acceptable resting state — STOP adding stopping behavior** (manufacturing a comfort fix here re-opens the P1-class lead-blind risk just closed). **This session does NOT unblock V2** — the flip is blocked on 163 unclassified tier-2 similarity-gate events (OFFLINE triage, not data volume), unchanged by more drives. Next move if advancing the stack: start the tier-2 triage (Stage 2 — classify the 163 events, class-B param moves vs Tier-1 re-scope); else the stack is safe to soak as-is. No deploy. |
| 2026-06-14 (eval) | **P2 gate RE-WIRED off the rate-aliasing artifact → SCORING_CONFIG_VERSION 3→4 (eval.md §2.1).** The v3 P2 gate metric `settle_peak_imu_jerk` (peak \|d(a_long_imu)/dt\| over every carState pair) was an artifact: `a_long_imu` (livePose, ~20 Hz) is held forward onto the 100 Hz carState clock, so diffing the one real update across the 0.01 s step manufactures ~70 m/s³ spikes (baseline med 27.2 / max 70.2 vs honest native-20 Hz med ~6.4 / max ~11.3). `harsh_terminal_grab` now fires on the FAITHFUL channels: PRIMARY `settle_peak_imu_decel` > 0.80 m/s² (peak \|a_long_imu\|, cross-sensor Spearman 0.97 vs the independent raw-100 Hz decel, ranks the felt `00001722` driveway grab #1 at 0.99) OR SECONDARY filtered raw-100 Hz `settle_peak_imu_jerk_raw` > 13.0 m/s³ (resolves the sub-100 ms grab the 20 Hz channel under-resolves). Caps calibrated from the 26 IMU-measurable engaged settles (decel just below p75; raw jerk just above p60, clear of the ~16 m/s³ filtered noise floor). Corpus re-baseline (114 considered, engaged comfort filter): old v3 12/114 (10.5%) → new v4 10/114 (8.8%); 3 artifact false alarms dropped (`00001717#3` artifact 70.2 but decel 0.62 / raw 4.8; `00001713#6`; `0000171f#15`), 1 real sub-100 ms grab added (`0000171f#7`, raw 14.1 but artifact 20.5 < 30), 9 kept. The held-100 Hz jerk + wheel-aEgo settle jerk are now non-gating diagnostics. Store re-ingested in place (87 replaced, 225 total). similarity_gate + FrictionPlant untouched. Full tools/stopping suite green (301). |

## Adjustment rule

After **every** drive, the drive report appends one line to the status table (stage, drive
result, date). Any unexpected fault or regression (accFaulted, AVH_LAMP, PBRAKE_ACT, FCW/AEB,
harsh/leapfrog flag, anomalous hold) **freezes stage advancement** until diagnosed and logged in
[worklog.md](worklog.md). Reverts are always allowed without a log-first step; the log follows.
