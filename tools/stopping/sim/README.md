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

Options: `--stages holds,replay,standard,off` (what is computed; finished jobs are always reused; the verdict always needs the
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
| COMFORT | valid pairs (takeover plan gap <= HEAD + 0.05 m) of NEW + history, L42 + L42s, v12: entry bites <= -0.10, full-approach pumps wire / plant, a_stop <= -0.6, j300 median, 4-5 m rest share (also per group). An aggregate is WORSE only when worse by > 10 % and by >= 2 counts (pairs for the share; the j300 median has no count floor); BETTER = none WORSE and more better than worse. Per-case regressions listed, not gating |

Every gate reads only the rows of this run's job set (a cached row of another run never enters). HEAD and the candidate are
compared at the same instants, never by array index (their stored traces can start at different times): each gate window is the
time both arms' windows share; candidate frames outside the stored HEAD trace are counted (`meta.uncovered`, H6 note). `gate.md` also lists the changed
nodrv runs (first divergence, re-holds RELEASE -> RAMP_TO_HOLD, first-motion / launch delta, chatter, min gap) and the replay
events (spans where ON != OFF, plan differences, new re-holds per recorded hold).

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
- Tests (repo config: `-Werror`): `pytest tools/stopping/sim -n 2`, or single process `pytest tools/stopping/sim -p no:xdist
  -o addopts= -W error`. The smoke test needs SIM_HOME and runs a worker; it removes the conftest `OPENPILOT_PREFIX`, which the
  macOS ZMQ backend does not support. Do not run the tests during a sweep (2-worker limit).
