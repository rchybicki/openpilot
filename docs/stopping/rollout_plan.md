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
| 2 — Gate re-run + V2 flip decision | `USE_STOPPING_V2` (only if gate passes) | Stage 1 exit | Gate deterministic exit (eval.md §5) → flip + 1–2 clean validation drives; or documented triage/re-scope decision | pending | — |
| 3 — StopReq staged enablement | `STOPREQ_LATCH`, then `STOP_REQ_MAX_SPEED`/`STOPREQ_RELEASE_SPEED` per sub-stage | Stage 1 exit (corpus before CAN changes); independent of stage 2 outcome | Sub-stage C passed, or deliberate hold at an earlier sub-stage | pending | — |
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

## Stage 3 — StopReq staged enablement

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

Each sub-stage: flip the named constant, deploy, verify hash. **Parking-lot session first**:
3 stops + 60 s holds, watching TCS15 `AVH_LAMP`, TCS13 `PBRAKE_ACT` / `ACCEnable`, and cruise
faults in the drive report. Then normal drives until the promotion criteria are met. Rollback =
revert the constant + deploy. `STOPREQ_RELEASE_SPEED` stays active at every sub-stage (the latch
may never hold StopReq on a rolling car).

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

## Adjustment rule

After **every** drive, the drive report appends one line to the status table (stage, drive
result, date). Any unexpected fault or regression (accFaulted, AVH_LAMP, PBRAKE_ACT, FCW/AEB,
harsh/leapfrog flag, anomalous hold) **freezes stage advancement** until diagnosed and logged in
[worklog.md](worklog.md). Reverts are always allowed without a log-first step; the log follows.
