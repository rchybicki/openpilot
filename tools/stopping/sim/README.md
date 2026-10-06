# Standard stopping simulator runner (`tools/stopping/sim`)

One command runs the fixed gate list (PLAN sections 72 and 82, `~/.route_sync/corpus/cycle_20261003/PLAN.md`) for HEAD or a candidate diff,
on one pinned engine, and writes `gate.md` + `gate.json`. It replaces the per-cycle runners under `/tmp/cyc_1004` (cl3/rt/rr/vr/vr4,
ccl2, rcore/lrcore/replay_core, pp5/pp6, the analyzer copies) with one package in the repo. Large data stays in `SIM_HOME`.

```sh
source .venv/bin/activate
# candidate: base commit + diff (ON = its new flags True, OFF = False; override with --on/--off K=V,...)
python -m openpilot.tools.stopping.sim.run_gates --base 77832b2710 --diff cand.diff --label trial
# HEAD self-check (ON == OFF == HEAD; fills the baseline cache)
python -m openpilot.tools.stopping.sim.run_gates
# iteration only (bm + named at L42 v12 + syn at L42; every gate INFO, no verdict)
python -m openpilot.tools.stopping.sim.run_gates --diff cand.diff --quick
# delete the arms of other engines (their rows can never be reused)
python -m openpilot.tools.stopping.sim.run_gates --prune
```

Options: `--stages holds,replay,standard,off,f1` (what is computed; finished jobs are always reused; the verdict always needs the
rows of the whole job set, so a stage that is neither computed nor cached makes the run INCOMPLETE), `--compute-only`, `--recheck N` (re-run N
cached HEAD jobs fresh and require identical rows: gate D), `--with-h` (adds the rejected history-trigger cell H in its own arm
subdirectory `arms/<key>/cellH/`; reported in a separate section, never gating, never mixed into the arm rows), `--car REV` (the car
build for the H1 car check against the base and the candidate tree; default `origin/!my-fp-new`, the pushed branch the device
deploys; a car that runs the candidate code with other flag values is named by arm), `--prune`. Output:
`SIM_HOME/runs/<YYYYmmdd-HHMM>_<label>/` (`run.log`, `meta.json`, `gate.json`, `gate.md`).

Verdict (top of `gate.md`, exit status): `PASS` (0) = every hard gate passes; `FAIL` (1); `INCOMPLETE` (2) = a job failed or has
no row (listed with its error; the gates are not a verdict); `NO COVERAGE (gates)` (2) = a hard gate had nothing to check;
`--quick` prints `NO VERDICT` (0). Bad input (missing or unappliable diff, a diff that changes a production file other than `.py`,
bad `--base` / `--car`, `--on K=yes`, unknown flag, bad `--stages`) gives a one-line error and status 64 before anything is written
(git's multi-line apply error is joined with ` | `).

## How it works

- **Source tree under test** (`loader.py`): `git read-tree <base>` + `git apply --cached <diff>` + `git write-tree` in a temporary
  index (the working tree is never touched). Every production `.py` (selfdrive, frogpilot, common, system, cereal, opendbc_repo;
  tests excluded) that differs between the working tree and that tree is written to `SIM_HOME/trees/<tree>/` and imported through
  a meta-path finder (`STOP_SIM_TREE` = the arm manifest; one tree and flag file per process tree). Compiled code comes from the
  working-tree build: a tree whose C/C++/capnp/dbc sources differ is refused, and so is a diff that changes any production file
  other than `.py` (a `.json` change would otherwise be ignored silently).
- **Flags are source**: each arm's `stopping_flags.py` is the tree's file with the overridden definitions rewritten
  (`NAME = value`, the whole statement, so a derived flag can be overridden too), written to
  `trees/<tree>/flags/<sha1>/` and always mapped. Derived flags evaluate as on the car (`GOVERNOR_BAND_PROFILE = SANTA_FE_STOP_LINE`
  follows `--off SANTA_FE_STOP_LINE=false`). Nothing sets module attributes at run time.
- **Arms**: HEAD (base), ON (base + diff, candidate flags on), OFF (candidate flags off). Candidate flags = the `NAME = True|False`
  switches of `stopping_flags.py` that the diff adds or changes. An arm is content-addressed:
  `SIM_HOME/arms/<sha1(production code hash, flag file sha1, engine sha[, reference replay arm])>/`. Two commits that differ only
  outside the production code share rows; a repeated or interrupted run resumes.
- **Every job asserts** the engine file sha1s (incl. the untracked `tools/stopping/review/kcs_plant.py`, `plant_data.py`,
  `plant_sim.py`, and `loader.py`, `worker.py` and the Hyundai sender test helper `test_can_bounds_fork.py`: tools and test files
  always import from the working tree; an imported `tools/stopping` or test file outside the fingerprint fails the job), the mapped module files and sha1s, the engine location, and every module-level flag of the imported
  `stopping_flags` against the arm's flag file evaluated as Python (+ the overrides). A failed job is written to
  `arms/<key>/errors_<jobs>.json`, the worker exits 3, and the run is INCOMPLETE.
- **Engine** (copied verbatim from `~/.route_sync/work/cyc_1004/eb2/lfl/hg3` = harness_g + the cyc_1004 standstill gate + the e2e
  proxy; only import / path lines and module-global annotations changed, marked `# ruff: noqa`): `harness.py`, `rharness.py`,
  `gear.py`, `gated.py`, `terminal.py` (kcs1 `body()`/`metrics()`), `metrics.py` (validate `new_extra`, cl3 `full_metrics` /
  `compact`). Rows (`worker.py`): drv runs keep a 50 Hz compact trace (every other frame of the trace: one grid for every arm) from the stop - 45 s to the first launch after the stop
  (v_true > 0.5 m/s) + 2 s, or to the trace end without a launch (H5); nodrv runs keep the full 100 Hz window incl. `vl_true`,
  `line_floor`, `plant_off`, `gear` (H5 lead speed, H6 line ownership, H2 brake-off band).
- **Case set** (`cases.py`, `STANDARD`; changing it changes the gates):
  - nodrv (driver removed, standstill gate, L42): every census hold of `census/holds.json` (a hold whose case cannot be built is
    a failed job until the census marks it with an `error`; those are listed as a NOTE: 297 of 298) + the 30 R.NEW stops;
  - recorded (start v12; L42, L42s, L42P): bm, new, named, down, hist (24 aim + 30 nobite), qr, s20@hold; qr / s20@hold also
    start auto; down + 2235_s55 in L42F;
  - synthetic (start auto; L42, L42s, L42P): syn, rb, rl, slow, go, burst, cut, cr, lt, sgs, and **ms** (model-only stops:
    `ms_v{6,8,10,13}_{none,far}`; the model stops 1 m short of a point 2 s + v0^2 / 1.8 m ahead, coasting until that needs
    0.9 m/s^2; no radar / model lead, or a radar + model lead 40 m ahead driving on at v0; modeld's shouldStop rule, passed to
    LongControl as controlsd does; an UNVALIDATED model stand-in like `e2e_proxy`).
- **Exact replay** (`replay.py`): on the logged 100 Hz frames of corpora c1 (238 launch / hold spans) and c2 (273 stop windows).
  Planner lockstep (eb5 `pp6.py`): the arm's `LongitudinalPlanner` replays every logged plan tick of the span (from 20 s before it)
  on the logged inputs; the plans are saved to `arms/<key>/plan/<corpus>/<span>.npz`. HEAD keeps the recorded LongControl inputs;
  OFF and ON feed LongControl the recorded target + (arm aTarget - HEAD aTarget) and their own shouldStop where they differ (so
  their replay arm key includes the HEAD replay arm). Then LongControl + StopContext + StoppingService + the Hyundai sender.
- **Radar stage** (`radar_replay.py`, cycle_20261006 builder R; before the planner lockstep of every replay span): the arm's radard
  (`RadarD.update`, production code; `RadarD(CP.radarDelay)` with the delay of the arm's tree from
  `CarInterface.get_non_essential_params(<logged fingerprint>)`) re-runs on the logged inputs of every logged radarState from
  `RADAR_WARM` = 10 s before the planner history (track filters, v_ego history and surrogate state warm): modelV2 / carState = the
  ones the radarState names (`mdMonoTime`, `carStateMonoTime`), liveTracks = the latest of that carState's card cycle published before
  the radarState (card sends liveTracks 0.2 ms after carState; where several messages are possible the logged leads decide:
  `n_race`), frogpilotPlan = the latest before the radarState; the logged `valid` bit. The FrogPilot lead consumers
  (`FrogPilotPlanner.update` + `publish` with real CEM, following, traffic controller, acceleration limits, events incl. the
  lead-departing alert and the tracking-lead filter; speed-limit / curve controllers, weather, GPS and params frozen to the logged
  frogpilotPlan) re-run on the re-run radarState at each logged frogpilotPlan tick. The planner reads the re-run radarState (every
  arm; the log's older-sample timing) and, in an arm with a reference, the logged frogpilotPlan / selfdriveState with the
  lead-consumer fields moved by (arm - reference). LongControl gets the complete planner output (aTarget, distanceToStopTarget,
  distanceToStopTargetModel, aTargetTrajectory as deltas; shouldStop, FCW and the trajectory validity where they differ), the lead
  inputs of the radarState its frame read (floats as deltas; the arm's lead where status or track id differ) and the arm's
  experimental mode where it differs. HEAD keeps the recorded frames (H1c unchanged). The span npz holds the plan, the re-run
  radarState (`R_*`) and FrogPilot arrays (`F_*`); the row holds `radar` = fidelity stats + the M1 samples of the scored window.
- **H2 confirm cells**: after the standard stages, a minimum-gap failure that lies only in the level-trigger cells in the 1st-gear
  brake-off band after a launch gets the same case run in L42P and H for HEAD and ON (`arms/<key>/confirm/`, never mixed into the
  arm rows); a missing confirm row makes the run INCOMPLETE.

## Gates (`gates.py`)

| gate | rule |
|---|---|
| H1a | exact replay OFF == HEAD on every frame (wire, sent, StopReq, phase, lcs, owning, service active) and every plan tick (aTarget, shouldStop; plans on different ticks differ). A replay row whose planner lockstep is empty, non-finite or has no tick for > 0.25 s over the frames makes the run INCOMPLETE |
| H1b | closed loop OFF == HEAD (full-trace sha + every metric) on L42 of the standard set + every 4th nodrv run |
| H1c | INFO: car check (`--car` vs base and vs the candidate tree, production files) + HEAD replay vs the logged command on spans whose drive commit has the HEAD controls code, first 1 s of each span skipped (tolerance not decided) |
| H2 | worst cell per (case, start) over L42/L42s/L42P/L42F: no min gap or rest_at_stop < 3.0 m where HEAD's is >= 3.0; nodrv: min gap after the stop. Min gap = minimum of the 3-frame median gap (one-frame radar glitches removed). A min-gap failure only in L42/L42s/L42F whose minimum comes after a launch with the plant brake off in 1st gear on >= 50 % of the braking frames (the level plant's brake-off band, contradicted by the 2072 log) counts only if L42P or H agrees (a new < 3.0 m there too); otherwise listed "not confirmed" (check the drive log by hand). `cut_*` listed, not gating |
| H3 | the ms family + every run where HEAD stops with no radar lead in the 2 s before the wheel stop: the candidate stops, at most 0.5 m past HEAD's stop position |
| H4 | no new StopReq clear -> set at rest within 1 s (a set-clear-set contains one; a set followed by the launch clear is one episode); no new service hold at rest in pid > 0.2 s; no new 'starting' under RAMP_TO_HOLD/HOLD on frames the service owns or where the command falls (a non-owning label over a monotonic ramp is not a conflict). Listed, not gating: new StopReq sets in RELEASE and a clear -> set at rest in the 1 s after the window |
| H5 | no new false launch (ego > 0.25 m from rest toward a lead slower than 0.15 m/s from the onset until those 0.25 m; lead speed = `vl_true`, else the lead position x + gap over +-0.25 s); on changed pairs with a comparable rest (both arms rest, the rests overlap, at least one moves; first motion = v > 0.05 m/s from 0.3 s after the rest): FAIL when HEAD moves and the candidate does not, when the candidate's gap growth since its rest exceeds HEAD's by > 0.3 m, or when its first motion is > 0.3 s later at a gap > 0.3 m larger. A larger gap from a rest further back (same growth, same time) is listed. Coverage = judged pairs (identical pairs with a launch included); changed pairs without a comparable launch are listed "not judged" |
| H6 | J_MAX 2.5 m/s^3 = 0.125 per 50 ms planner tick on the braking component of releases the change owns: closed loop: (a) frames the candidate's service owns (or owned the frame before: the hand-back) where its command differs from HEAD's by > 0.01 anywhere in the 50 ms window (a release onto HEAD's value counts), rise of min(cmd, 0) over the last 50 ms summed over owned frames; (b) planner ticks where its plan differs from HEAD's and its line floor bound the previous tick (a rise from a deeper MPC tick to the floor is the MPC's release, capped by the line), rise of min(aTarget, 0) at the tick. Replay: service-owned command frames where ON differs from OFF and, independently, on spans with a plan change the plan braking component per tick (engaged, no pedal, not above OFF's rise). LongControl 'starting' frames exempt |
| R1 | radard replay fidelity on the HEAD replay rows: on drives whose commit has HEAD's radard code, any re-run tick in the scored window (the replayed frames) that differs from the logged radarState (leadOne / leadTwo status, radar, radarTrackId, fcw, surrogate flags exact; dRel, yRel, vRel, vLead, vLeadK, aLeadK, aLeadTau, modelProb within 1e-4 and finite; the KF-state fields vLeadK / aLeadK / aLeadTau within 0.05 only on liveTracks-race-affected ticks = the lead's re-run or logged track had differing values among the race's candidate messages within the last 2 s, `radar_replay.KF_TOL` / `KF_RACE_S`, the note reports how many scored ticks used it and the worst error; a KF mismatch less than 10 s after the re-run start, i.e. a log that starts late, is not exempt and gets no race tolerance) makes the run INCOMPLETE; a frame whose logged lead inputs differ from the logged radarState it is mapped to is INCOMPLETE on any build; other builds' differences are listed |
| R3 | INFO: the replayed FrogPilot lead-consumer fields (trackingLead, experimentalMode, redLight, tFollow, desiredFollowDistance, dangerFactor, jerks, min/max acceleration, leadDeparting) vs the logged frogpilotPlan on drives with HEAD's FrogPilot planner code (the FrogPilot process's input timing is not logged). Propagation (T2, `~/.route_sync/work/radar/T2/r3_plan.py` + `r3_an.py`, all 511 c1/c2 spans, HEAD planner lockstep vs the LOGGED aTarget on the scored ticks): logged radarState 0.0053 mean / p99 0.171 / 90 shouldStop mismatches; re-run radarState + FrogPilot frozen to the log (the HEAD arm) 0.0053 / 0.172 / 90; re-run radarState + replayed FrogPilot 0.0099 / 0.310 / 376 (worst spans: experimentalMode, CEM history not logged; maxAcceleration). So HEAD keeps the logged frogpilotPlan and an arm reads log + (arm - HEAD replay): R3's mismatches reach an arm only where the arm differs from HEAD (R4 counts those ticks). Every replayed field mixes lead and non-lead inputs (maxAcceleration: the lead-departing launch floor), so a per-field freeze is not possible; the per-tick delta is it |
| R4 | INFO: the FrogPilot lead consumers candidate vs HEAD: ticks that differ per field, and of them the ticks where HEAD's replay already differs from the logged frogpilotPlan (the candidate reads log + (candidate - HEAD), the candidate's value for a discrete field: there its change rests on an R3 mismatch); lead-departing alert onsets; vision-only lead ticks whose published vLead differs |
| M1 | published vLead / vLeadK / aLeadK error vs truth, candidate vs HEAD, per class: leadOne stationary (geometry-qualified, truth 0: MAE / RMS / p99 abs, tolerance 0.01 / 0.01 / 0.03 m/s), braking lead (truth a <= -1.0) x ego accel (<= -1.5, -1.5..-0.5, > -0.5), mild braking, steady, accelerating, braking onset (first 1 s), accel sign change (+-0.5 s), fresh track (first 0.5 s); leadTwo stationary / braking / other (signed error mean / p95 / p99: more optimistic than HEAD by > 0.05 / 0.05 / 0.08 m/s, aLeadK 0.10 / 0.15 / 0.25 m/s^2 FAILS); sustained optimistic excursions on moving leads (> 0.1 m/s for >= 0.3 s on one track; more than HEAD + max(2, 10 %) or + max(1 s, 10 %) FAILS); stationary optimistic episodes per lead and field (published > 0.15 m/s for >= 0.3 s on consecutive stationary ticks of one track, truth 0; FAIL on more episodes or episode-seconds than HEAD, or a candidate episode that no HEAD episode of the same span and track overlaps and that is longer or higher than HEAD's worst: the pooled stationary statistics hide a 1 s +0.8 m/s error among ~250k ticks, Astra tooling review finding 4). Moving truth = REL_SPEED at t + L / cos(azimuth) x scale + ego speed over the envelope L 0.13 / 0.18 / 0.24 s x scale 0.99 / 1 / 1.01: a class FAILS when every corner fails, is UNCERTAIN when some do; UNCERTAIN or EMPTY (< 100 ticks in an arm) is NO COVERAGE, never PASS. INFO rows (not gating): the stationary leadOne per ego accel bin (<= -1.5, -1.5..-0.5, > -0.5: signed mean, MAE, p99 abs of vLead / vLeadK, HEAD -> candidate; the latency error L x aEgo the source fix targets) |
| F1 | following matrix (section "F1"): per (case, cell) the candidate vs HEAD on physical truth: no new collision (or a faster / > 0.1 s earlier impact where HEAD collides), no new clearance < 2.0 m or TTC < 1.5 s, clearance not smaller by > max(0.5 m, 5 %), TTC (< 10 s) not smaller by > 10 %, closing speed not larger by > 0.2 m/s, brake onset not later by > 0.1 s, braking deficit to HEAD's closest approach <= 0.5 m/s, release not earlier by > 0.5 s. H1b-F1: OFF == HEAD on the F1 runs |
| COMFORT | valid pairs (takeover plan gap <= HEAD + 0.05 m) of NEW + history, L42 + L42s, v12: entry bites <= -0.10, full-approach pumps wire / plant, a_stop <= -0.6, j300 median, 4-5 m rest share (also per group). An aggregate is WORSE only when worse by > 10 % and by >= 2 counts (pairs for the share; the j300 median has no count floor); BETTER = none WORSE and more better than worse. Per-case regressions listed, not gating |

Every gate reads only the rows of this run's job set (a cached row of another run never enters). HEAD and the candidate are
compared at the same instants, never by array index (their stored traces can start at different times): each gate window is the
time both arms' windows share; candidate frames outside the stored HEAD trace are counted (`meta.uncovered`, H6 note). `gate.md` also lists the changed
nodrv runs (first divergence, re-holds RELEASE -> RAMP_TO_HOLD, first-motion / launch delta, chatter, min gap) and the replay
events (spans where ON != OFF, plan differences, new re-holds per recorded hold).

## Radar path in the closed loop (rharness, 2026-10-06)

The closed loop runs the tree's PRODUCTION radard publication: one `radard.RadarD(CP.radarDelay)` per run, `update()` at every logged
radarState time from `radar_replay.RADAR_WARM` = 10 s before the case window (synthetic: model time + 14 ms) with candidate-local state for every track (Kalman filters, lead selection, leadTwo,
low-speed override, lane-change surrogates). A candidate that changes radard publication or `CP.radarDelay` acts in the closed loop
without a harness option.

- `CP.radarDelay` = `interfaces[fp].get_non_essential_params(fp).radarDelay` of the case's car in the tree under test
  (`rharness.tree_radar_delay`, read inside the run: the loader maps `opendbc_repo`, so a flag-gated value follows the arm's flag
  file). `run(..., radar_delay=x)` still overrides it. The row records it (`info.radar_delay`).
- Inputs per tick: the liveTracks message radard used, picked by the exact replay's rule (`radar_replay.live_tracks_index`: the latest
  one card sent no later than that carState's cycle and before the radarState; a race between messages around the carState read is
  settled by the logged leadOne / leadTwo / adjacent leads), the carState it used (`carStateMonoTime`;
  vEgo = the plant observation once closed), the modelV2 it used (`mdMonoTime`), the latest frogpilotPlan (toggles through the same
  default-merged `_toggles` as the planner). The published radarState replaces the planner's radarState and all LongControl lead
  inputs (status, dRel, vLead, aLeadK, track id, model prob, leadTwo status / vLead / dRel).
- Recorded cases: every logged track moves into the sim ego frame, `dRel - dx(td)` (0.1 m) and `vRel - dv(tv)` (0.01 m/s), dx / dv =
  plant minus logged ego position / speed at td / tv = liveTracks time - 5 ms - LAG_D / LAG_V; before the takeover dx = dv = 0 (the
  logged tracks exactly). The logged speed here is `rharness.v_mean`, the centred 0.1 s mean of the wheel-pulse truth v_true (it
  ripples +-0.3 m/s within 10 ms; the raw value put that ripple into radard's vRel after the takeover: 2086_s17 published leadOne
  steps max 0.255 -> 0.149 m/s per frame, p99 0.101 -> 0.072). The plant starts from it at the takeover, and the recorded lead truth
  `vlt` = vRel + v_mean. The stream reads the route's segments of its whole range (s5's cached case paths started one segment after
  the radard warm-up). The model leads move by -dx and the model's ego speed by +dv (a vision lead keeps its physical speed).
  `<case>@hold` keeps its held lead (the lead track from the held truth). No Kalman state is copied from the log: tracks are filtered
  from the window start (the old engine reseeded a switched lead from the logged filter and modelled only leadOne).
- Synthetic cases: tracks from the case objects (`radar_objects(td, tv, r)`, default the lead with the record's track id) with the same
  lags, range and relative speed x `OBS_SCALE` (the radar reads 1.010 x the pulse truth like vEgo, 10-02 verify_impact); a track that disappears is deleted and a new id starts cold, as in production.
- Fidelity (`info.radar_fid`, recorded cases, from the window start to the takeover): re-run leadOne vs the logged one (`n` radar
  frames, `status` equal, `lead` frames where both have a lead, `track` same track + radar flag among those, max |error| of vLead /
  vLeadK / dRel on the same track); `info.radar_warm` = s of radard history before the window. 2235_s71: warm 13 s, status and track
  equal, vLead and vLeadK identical (< 1e-3; cold-started at the window the KF state was up to 3.3 m/s off); test
  `test_recorded_case_reruns_radard_on_the_logged_inputs_before_the_takeover`. 200 HEAD rows of the comparison below: the
  re-run equals the log on every pre-takeover frame (status, track, vLead, vLeadK).
- HEAD rows vs the pre-radar engine (cycle_20261006 T2: `~/.route_sync/work/radar/T2/cmp_engine.py` + `cmp_an.py` + `cmp_explain.py`
  vs `C/cmp_old.pkl`; 200 HEAD jobs: bm / named / new L42 v12, every synthetic family L42, every 12th census hold nodrv, bm L42P; no job
  errors): trace sha identical on 16 / 200, and on 150 / 200 with builder C's cold-start engine (the warm-up and the shared liveTracks
  rule change few rows). Pre-takeover fidelity on the 70 recorded rows: status, track, vLead exact on every frame; vLeadK exact except
  sv_000020b8_982.91 (0.066 m/s: no rlog of the previous segment, 0.01 s of warm-up). Warm-up < 10 s on 5 rows: 3 windows start at
  the route start, sv_000020b8 (rlog missing), s5 (its cached case `paths` start one segment late; the takeover is 77 s into the
  window). Metric changes: synthetic families <= 0.0 except cut (min gap <= 0.03 m, min wire <= 0.09); nodrv min gap <= 0.05 m,
  t_stop <= 0.23 s; recorded min gap p10-p90 -0.09..+0.03 m. The 8 rows past |d min gap| 0.10 m, |d wheel stop| 0.5 s or |d felt| 0.5,
  each explained (`T2/probe2.py` traces, `probe3.py` planner radarState input, `probe5.py` / `probe6.py` LongControl and radard inputs):
  - common cause (every recorded row): the old engine gave the planner / LongControl the LOGGED leadTwo (only dRel shifted) and status /
    track / model prob; now leadTwo is re-run on the sim-ego tracks (leadTwo vLead differs up to 0.15-0.26 m/s, aLeadTau resets) and
    leadOne differs by the measurement quantization (0.01 m/s, 0.1 m) after the takeover. The plan moves 0.01-0.03 m/s^2 within 0.1-0.3 s.
  - 2232_s40 wheel stop 2416.9 -> 2423.9 s: both engines dip to rest at 2417 s (old 0.000, new 0.03 m/s; wire within 0.03), the first
    wheel stop is then the next stop of the creeping queue; min gap 5.29 -> 5.26 m. Metric discontinuity, not a behaviour change.
  - 2235_s62 min gap 3.36 -> 4.38 m, min wire -3.50 -> -2.25, felt 11.9 -> 6.2: the lead switches track at 24792.9 s (one tick vision
    lead, then a new track); the old engine reseeded the switched lead from the LOGGED Kalman state (accepted cause).
  - s22 felt 3.19 -> 2.63: a lead track switch (dRel step 10 m) reseeded from the log by the old engine (accepted cause).
  - 2086_s17 min gap 5.44 -> 5.16 m: at 1036.55 s one radard tick reads leadOne 0.31 m/s slow (vRel -3.94 vs logged -3.63) and
    LongControl's command steps -1.14 -> -0.65. Cause: the closed loop moves a recorded track by dv = plant v - logged v_true at the
    measurement time, and v_true (wheel pulses) ripples +-0.3 m/s within 10 ms (6.65 -> 6.36 -> 6.65 at 1036.33 s). Both engines have
    this; the shared liveTracks rule picks the cycle message (41 ms earlier than the old engine's) and so samples another spike.
    OPEN: v_true departs from its 0.1 s mean by > 0.2 m/s on ~6 % of the moving frames of the 42 recorded stop cases (per-case p99
    0.41 m/s): the closed loop injects that ripple into radard's vRel after the takeover (exact replay, M1, R1, F1 unaffected).
  - 2232_s82 felt 2.89 -> 5.07, s5 5.24 -> 2.94, 2235_s51 4.80 -> 2.75, 2235_s71 2.77 -> 3.28: no lead switch, inputs differ by the
    common cause only (2235_s71: wire within 0.01 everywhere); felt is a 0.3 s jerk peak and moves 0.5-2.2 on these differences.
  A HEAD self-check (`run_gates`) builds new HEAD arms on this engine anyway.
- Known quirk (not changed, it would change HEAD rows): the `cut` family replaces the case lead after `synthetic_case`, so its e2e proxy
  model keeps the original stopped lead's speed (0 m/s) for the cut-in car. F1 passes its lead with `synthetic_case(lead_obj=...)`.

## F1 following matrix (f1.py; stage `f1`, rows in `arms/<key>/f1/`)

Synthetic closed-loop following at 20-30 m/s through the production radard (above) and the real planner / LongControl / sender /
KCS plant (cell L42). 20 cases x 5 cells = 100 runs per arm, about 50 s per arm with 2 workers; HEAD, ON and OFF are computed.

- Families: `hb` steady following, the lead brakes at -2/-3/-4 to half speed (20/25/30 m/s) or to rest (20 m/s, -3); `ab` both
  accelerate +1 for 3 s, then the lead brakes at -3 (stale positive aLeadK); `cut` a car cuts in 1.0 / 1.5 s ahead, 3 m/s slower,
  already braking at -3 (new track, cold filter); `sw` the braking lead's track id changes (cold: a new id; warm: to a second return of
  the same car that radard has filtered from the start); `l2` the lead changes lanes out while the car ahead of it (leadTwo, a model
  lead 1) brakes at -3; `rv` the radar loses the braking lead for 1 s (vision lead), then a new track.
- Start: the planner's steady follow distance (`desired_follow_distance` with get_T_FOLLOW for the car's toggles and personality,
  not_leftmost_lane True: 0.95 s at 25 m/s ACC); the run's following comes from FrogPilotPlanner (T_EV = 3 s settles a difference). Cruise 1 m/s above the follow speed (4 m/s in `ab`).
- Cells: `F1B` Experimental + CEM + human_following (the car: CEM decides experimentalMode), `F1A` ACC (CEM and Experimental off) +
  human_following, `F1K` ACC without human_following (the radar-only lead extrapolation); `F1A_lv0.1` / `F1A_lv0.21`: LAG_V 0.10 /
  0.21 s (REL_SPEED latency p10-p90).
- FrogPilot lead consumers (Astra tooling review finding 1): every model tick the tree's FrogPilotPlanner (`radar_replay.fp_planner`
  / `fp_step`, rharness `fp_tick`) runs on the simulated state: the re-run radarState published by the model time + 3 ms, the
  synthesized carState / modelV2 / selfdriveState (experimentalMode by selfdrived's CEM rule from the previous frogpilotPlan;
  LongControl's experimental_mode follows it), carControl.longActive. Its frogpilotPlan (tFollow, jerk costs, danger factor, min /
  max acceleration, CEM, trackingLead, traffic controller, events) feeds the planner and radard; speed-limit / curve controllers stay
  frozen to the synthesized plan (vCruise = the cell's cruise). F1 computes no following output itself. The earlier F1 wrote the
  jerk FACTORS (~1.0) where FrogPilotPlanner publishes the COSTS (A_CHANGE_COST / DANGER_ZONE_COST / J_EGO_COST x factor = 200 / 100
  / 5, which long_mpc.set_weights uses directly), i.e. a ~200x too reactive ACC MPC: F1 HEAD rows of runs before this change are not
  comparable (e.g. hb_v20_a3_stop F1A min clearance 1.3 m, hb_v25_a4 F1K brake onset ~2 s after the lead).
- Measures (`f1.measures`, from the event to the first contact): min clearance to the nearest in-lane object (truth), min TTC, max
  closing speed, collision + impact speed / time; brake onset = sent SCC12 <= -0.5 held 0.3 s; release = first sent >= -0.3 after it;
  the sent trace (50 Hz) for the braking deficit.
- Gate numbers (gates.py F1_*): HEAD's own spread over the measured latency (LAG_V 0.10 / 0.21 vs 0.15, F1A, 40 pairs) is <= 0.16 m
  clearance, <= 0.04 m/s closing, <= 0.06 s onset, <= 0.34 s release; the relative limits are about 3x that (clearance max(0.5 m,
  5 %), closing 0.2 m/s, onset 0.1 s, release 0.5 s); the deficit limit 0.5 m/s (a 0.15 m/s^2 shortfall over ~3 s). Floors 2.0 m /
  1.5 s TTC are near-collision physical thresholds; a floor counts only where HEAD meets it. HEAD collisions are listed and judged
  only on a faster (+0.2 m/s) or > 0.1 s earlier impact.
- HEAD on the F1 plant (`f1.PLANT`) collides in 14 of 100 runs: `cut_v20_h1` (A, A lags, K), `cut_v25_h1` (A, A lags, B, K),
  `hb_v20_a3_stop` (A, A lags, K), `hb_v30_a4` K (impact 0.8-5.2 m/s). With the stop cell's plant (-0.035) it was 3 (K only). The
  car's ACC headway is 0.95 s at 25 m/s and the fitted plant reaches about -3.0 at the -3.5 command, so a lead braking at -3 to rest
  (or a 1.0 s cut-in already braking at -3) is not avoided after the reaction. The plant's deep-braking gain is extrapolated (the
  events reach -3.5 rarely), so treat these as "beyond the margin", not as measured collisions; they are a finding for the host.
- With the production FrogPilot planner in the loop (T3, engine 056a49597ca6, F1-only runs; the other stages not computed, so the
  verdicts are INCOMPLETE and only the F1 rows are a result): HEAD collides in 15 of 100 runs (`cut_v20_h1` A + A lags + K,
  `cut_v25_h1` A + A lags + K, `hb_v20_a3_stop` A + A lags + K, `hb_v30_a4` K, `rv_v25` K, `sw_v25_cold` K; impact 1.0-6.7 m/s).
  `20261006-1551_t3_c2fev_f1` C2fev F1 PASS 0 / 100; `20261006-1554_t3_delay_only_f1` delay-only F1 FAIL 48 / 100 (clearance -0.5..
  -1.4 m mostly in A and its lag cells: hb a2 / a3 at 20-30 m/s, ab, cut h1.5; HEAD-collision runs hit faster / earlier). H1b-F1 PASS.
- Demonstration (delay-only scratch diff `~/.route_sync/work/radar/C/delay_only.diff`: `radarDelay = 0.15` for
  HYUNDAI_SANTA_FE_HEV_2022 behind a scratch flag SANTA_FE_RADAR_TIME_ALIGN; `run_gates --diff ... --stages f1`; the other stages are
  not computed, so the verdict is INCOMPLETE and only the F1 rows are a result):
  - `20261006-0827_radar_delay_f1_plant` (f1.PLANT): F1 FAIL 16 / 100; H1b-F1 OFF == HEAD PASS (100). Clearance -0.5..-1.3 m in F1K
    (hb a2 at 20/25/30 m/s, ab_v20, cut_v25_h1.5) and in F1A + both A lag cells on cut_v20_h1.5 (-1.27 m of 7.1 m, deficit 0.50 m/s);
    in 8 HEAD-collision runs the candidate hits > 0.1 s earlier; F1B hb_v30_a4 releases 0.66 s earlier (no clearance change).
  - `20261006-0821_radar_delay_f1` (the stop cell's plant, before the refit): F1 FAIL 10 / 100, the same pattern (F1K clearance -0.6..
    -1.7 m; F1B ab_v25 -2.65 m of 46 m).
  The delay-only publication reads a braking lead up to L x |aLead| too fast (vLead = vRel + the vEgo of 0.15 s ago); HEAD's error
  mostly cancels while the ego brakes with the lead. Onset never moves (the model / MPC start braking on the model lead); the effect is
  less braking during the event. Blended (B) hides most of it: the e2e proxy sees the lead's true speed without lag, so B results
  are optimistic for any radar change; K (radar-only) is the most sensitive cell.
- Plant at speed (`~/.route_sync/work/radar/C/plant_hs.py`, 57 logged follow-brake events at 10.6-38 m/s, demands to -3.5, 30 segments
  of routes 2232-224f): the KCS plant (L42 cell) driven by the logged sent SCC12 + SCC14 limits vs the pulse-wheel speed. Lag
  (command shift that best explains the measured accel) median 0.28 s (p10-p90 0.13-0.44; the plant's slew + 0.12 s delay + 0.10 s
  lag), measured gain 0.92 (p10-p90 0.86-1.01; plant 0.90-0.95 with the cell's -0.035), the plant brakes 0.13 m/s^2 more than the car
  on frames with sent <= -1 (median; p10 0.31 more, p90 0.01 less). Gain sweep on the same events (median braking bias):
  gain_delta -0.035 -0.129, -0.07 -0.079, -0.10 -0.040, -0.135 +0.002 (mean -0.010; by v0 10-15 / 15-20 / 20-40 m/s: -0.067 / +0.043 /
  -0.079). F1 therefore runs `f1.PLANT` = the L42 cell with gain_delta -0.135 (F1 only; the stop cells keep -0.035). Residual: the
  per-event bias spread (p10-p90 -0.16..+0.12) and speed error over the event (max |dv| median 0.9 m/s; a few events with gas or
  pulse artefacts reach 4 m/s) are larger than the radar effects F1 judges, so F1 compares arms on the same plant and does not claim
  absolute stopping distances.
- Limits: the model is the UNVALIDATED e2e proxy (truth, no lag); objects never react; no traffic-mode configuration; no driver;
  lateral geometry is simple (lane change = a 2 s y ramp; model leads at y = 0); one jerk / follow setting (the car's toggles).

## SIM_HOME (default `~/.route_sync/work/sim`, env `STOP_SIM_HOME`)

`cases/` (case pickles; APFS clones of harness_g/cache + cyc_1004/service/harness/cache), `gear/`, `frames/c1|c2/` + `c1.json`,
`c2.json` (cyc_1004/launch, cyc_1003v/A), `census/holds.json`, `trees/`, `arms/` (`--prune` deletes the arms of other engines),
`runs/`. The sweep lock is `~/.route_sync/work/sim_sweep.lock` for every SIM_HOME (one sweep per machine). History case registration reads
`~/.route_sync/work/cyc_1003/history` (rharness `HISTORY`). The planner lockstep reads the route rlogs
(`~/.route_sync/data/media/0/realdata`; 3 spans miss a segment and replay with fewer plan ticks). Recreate from the cyc
directories with `cp -c` (clone, no extra space).

## Verification (2026-10-05, PLAN 82 gate definitions, engine a77de0fa6239) and runtime

2 spawn workers. Trial recompute on the new engine (`20261005-1846_taskA_trial_compute`, closed arms cold, replay cached): 4249 s,
0 job errors; E3 (`20261005-1956_taskA_e3_compute`, its ON arm = the trial HEAD arm): 2434 s.

| stage | jobs | seconds (cold) |
|---|---|---|
| nodrv holds, per arm (HEAD, ON) | 327 | 930, 958 |
| exact replay with the planner lockstep, per arm and corpus (c1 238 / c2 273 spans) | 511 | 301-333 each (unchanged engine) |
| standard closed set, per arm (HEAD, ON; incl. 24 ms runs) | 726 | 921, 903 |
| OFF identity set (L42 standard + every 4th nodrv run) | 322 | 536-666 |
| H2 confirm cells (L42P + H, HEAD and ON) | 2 per band case | 5 |
| gates (all cached) | - | 7-17 |

- Engine change check (rows store more, the sim is unchanged): every row of the 4 recomputed closed arms equals its old-engine row
  (trial HEAD / ON 1053/1053, OFF 322/322, E3 HEAD 1053/1053: trace sha + every metric; nodrv 100 Hz windows equal on the old
  columns; the old drv compact trace is a prefix of the new one).
- Trial (`trial_56e1512892.diff` on 77832b2710; `20261005-2037_taskA_trial`): FAIL. H1a/H1b/H3 PASS; COMFORT BETTER (= PLAN 76).
  H2 PASS: sv_00002072_2413.91 (L42 3.08 -> 2.05 m) is in the brake-off band and not confirmed (L42P 3.51 / 3.50, H 4.47 / 4.36:
  listed). H4 3: StopReq clear -> set at rest on 2232_s5 L42P and 2235_s59 L42s (both in the post-stop window the old 6 s
  trace did not store), s20@hold L42 auto pid hold 0.22 s; listed: StopReq set in RELEASE on sv_2086_1666.53 and sv_20f8_620.97,
  clear -> set just after the window on sv_2232_4940.01. H5 22 (405 judged; 502 changed pairs not judged, 373 of them because neither
  arm moves before the recorded drv case window ends). H6 12: band-latch service-entry release steps 0.127-0.178 per 50 ms
  (6 runs) and sgs_v8/v12 (LongControl stopping -> pid under service ownership, +0.45 in 20 ms; 6 runs).
- E3 (`git diff d1a05a045a 2e39594627` on d1a05a045a; `20261005-2037_taskA_e3`): FAIL on H5 only, sv_000020fd_647.63 gap growth
  +0.75 m (the PLAN 58 residual); every other gate PASS (H1c 122 spans: max-error median 0.065, p90 0.137, worst 0.320); COMFORT SAME.

## Limits

- One sweep at a time per machine (`~/.route_sync/work/sim_sweep.lock`, also for a scratch `STOP_SIM_HOME`), at most 2 spawn
  workers (`SIM_PROCS`). Measured RSS: 2.4-3.4 GB per sim worker within
  about 20 s, peaks 4.5 GB; the worker main about 0.9 GB; up to 8.7 GB total.
- The H1c tolerance is an open decision (J_MAX and the comfort small-count floor: PLAN 82); the car-build stop spans of
  drive_report are not in c1/c2 yet (H1c covers older builds only). The HEAD self-check builds a new arm (about 45 min cold)
  whenever HEAD's production code or the engine changes.
- H2's "the drive's log supports it" path (PLAN 82) is not automated: an unconfirmed band failure is listed for a manual log check.
- The ms family is a model stand-in (no real modelV2 at a red light); no frozen-ego launch counterfactual; the `gate+creep1`
  standstill sensitivity cell is not in the standard set.
- Tests (repo config: `-Werror`; `test_radar_closed.py` = the radar path and F1, its sim checks run in subprocesses): `pytest tools/stopping/sim -n 2`, or single process `pytest tools/stopping/sim -p no:xdist
  -o addopts= -W error`. The smoke test needs SIM_HOME and runs a worker; it removes the conftest `OPENPILOT_PREFIX`, which the
  macOS ZMQ backend does not support. Do not run the tests during a sweep (2-worker limit).
