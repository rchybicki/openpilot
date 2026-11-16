# Parity audit: old branch against the port (2026-11-15)

## Goal (the owner's)

Radek drives a Hyundai Santa Fe HEV 2022 daily on his openpilot/FrogPilot fork.
On 2026-11-15 the car moved from his old branch to a port of his features onto
a newer FrogPilot base. Rule of the move: the car drives, looks and behaves as
it did on the old branch. On his first drive he found one thing missing (the
speed number no longer turns red while the car brakes). He asks: "do a final
check if everything was ported ... Let's double-check if we have everything we
had before."

Your job is research only: find every behaviour of the old branch that is
missing or different on the port and has no recorded decision. Change nothing.

## The trees (all in one git repository, read with `git show`, `git diff`, `git grep`)

Repo: `/Users/radoslawchybicki/Repos/openpilot-rch`. Do not check out, fetch,
push, commit or edit anything in it or in its worktrees. Branch names contain
`!`: quote them in zsh, and write `"${rev}:path"` with braces.

| Name | Commit | What it is |
|---|---|---|
| OLDBASE | `d2db3fad4b` | The FrogPilot source the old branch was built on |
| OLD | `37ac68c984` | The old branch as the car ran it until today (`!my-fp-new`) |
| NEWBASE | `437be91e24` | The newer FrogPilot source |
| PORT | `a3cd5ee7f4` | The port the car runs now (`!my-fp-1031`) |

Our own work on the old branch is `git diff OLDBASE OLD`. The port's own work
is `git diff NEWBASE PORT`. A behaviour of ours is fine on the port when
`NEWBASE..PORT` carries it, or when NEWBASE itself already gives the same
behaviour another way.

Decisions of record (read first):
`git show a3cd5ee7f4:docs/frogpilot_refresh_20261031/DECISIONS.md` (what was
ported, what was dropped on purpose, which new-base features are off),
`.../CLOSURE.md` (sections "Judged day-one differences" and "Not ported") and
`.../INVENTORY_OURS.md` (the feature rows). A difference that one of these
names as decided is NOT a finding; list it in one line under "decided".

## The known example (the kind of gap to find)

OLD `opendbc_repo/opendbc/car/hyundai/carstate.py:152` sets
`ret.brakeLightsDEPRECATED = bool(cp.vl["TCS13"]["BrakeLight"])`. PORT does
not; NEWBASE puts the same signal into `fp_ret.brakeLights`. The ported HUD
(`selfdrive/ui/qt/onroad/hud.cc:39`) still reads
`car_state.getBrakeLightsDEPRECATED() || car_state.getBrakePressed()`, so the
speed turns red only on the driver's brake pedal, not when the car brakes.
The file was ported, the consumer was ported, one producer line was lost.
So do not stop at "the file exists on the port": follow each value from where
it is produced to where it is used.

## Method

1. For every file in your lane that `git diff --stat OLDBASE OLD` lists, read
   our hunks (`git diff OLDBASE OLD -- <file>`). For each behaviour the hunk
   adds or changes, find it on PORT (the file may have moved or been renamed
   by the new base; search by identifier and by string).
2. Classify: PORTED (same behaviour, say where), EQUIVALENT ON NEWBASE (say
   where and why it is the same), DECIDED (name the decision row), MISSING
   (not on PORT, no decision), DIFFERENT (on PORT but behaves differently:
   another default, another condition, another data source, a reader whose
   producer is gone, a producer nobody reads).
3. For MISSING and DIFFERENT, give the evidence: OLD file:line with the code,
   the PORT location you expected and what is there, and what the driver or
   the car would notice. Say how sure you are. Reproduce or drop: an item you
   cannot show with file and line on both sides is not a finding.
4. Also check the reverse direction inside your lane where it affects the
   driver: a NEWBASE behaviour that is ON on the port by default, that OLD did
   not have, and that DECISIONS.md does not name (a new alert, a new automatic
   action, a new on-screen element, a changed default of a setting the car
   stores). Stored settings win over defaults; the car's stored values are
   not available to you, so name the key and both defaults.

The longitudinal planner and controlsd stages were proven equal by a replay
of a recorded route (0 differences) and by the stopping simulation. Do not
re-audit their arithmetic line by line. Everything the replay does not cover
is in scope: car interface signals, events and alerts, sounds, the UI, stored
settings, system processes, start-up, update and logging code.

## Rules

- Read only. No edits, no commits, no checkout, no fetch, no push, no build.
  No device access: never run `ssh`. Never run
  `tools/route_sync/refresh_routes.py` or `fullupdate.sh`.
- Scratch (notes, lists) only under the directory your task names, inside
  `/Users/radoslawchybicki/.route_sync/work/fp1031/parity/`. Leave your final
  table there as `REPORT.md` as well as in your reply.
- At most 2 parallel shell jobs. No heavy jobs (no test suites, no builds).
- You are a leaf: start no subagent.

## Reply

1. One paragraph: how many files and hunks you covered, and what you did not
   cover (be exact; an uncovered file is listed, not hidden).
2. Table of MISSING and DIFFERENT items, most noticeable for the driver first:
   id, what the old branch did, OLD file:line, PORT state with file:line, what
   the driver or car notices, confidence, and the smallest fix you would
   suggest (one line; do not write the code).
3. One line per DECIDED item you met.
4. Counts of PORTED and EQUIVALENT items (no list needed).
