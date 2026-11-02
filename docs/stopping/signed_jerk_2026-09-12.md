# Deployment and signed-jerk follow-up, 2026-09-12

Candidate `7c861b7769903118a05ceb4d44e6fce839043bf9` is deployed. `fullupdate.sh`
completed and rebooted the device. Independent SSH checks found that commit, uptime
81.28 s, and the tested stopping-service SHA256
`24d2d38715bfbe26b96c5ee63788d5fc60d8dfab5ede04660bac74d8d9e36843`.
The manager was running off-road (`IsOnroad=0`). This verifies installation and
startup, not improved physical stopping. No drive on this revision was observed.

## A more useful description of the dip

The eleven explicitly marked manual stops remain the personal reference. All eleven
have a complete manual interval from the last 2.5 m/s crossing through filtered rest
+0.5 s. Each event's largest absolute 300 ms jerk occurs while measured braking eases:
both acceleration endpoints are negative, and acceleration becomes less negative.
An absolute-jerk score alone hides the difference between that release and a brake dip.

The existing evaluator now retains the minimum and maximum **signed** jerk and their
recorded window end times. Negative jerk means acceleration decreases; positive
jerk means acceleration increases. Tied extrema use the earliest window end.
Each value covers the full preceding 300 ms, not an instantaneous derivative.
It uses the same interpolation, full-window requirement,
validity checks and missing-data reasons as before. Extrema are not clamped to zero;
an interval with only positive jerk retains a positive minimum. The earlier absolute
metric and every earlier descriptor reproduce exactly across all 23 selected events.
All 23 final-2.5 m/s signed cells are valid; none has a missing-data reason.
Only the marked-comparison JSON format advances to version 2. The separate
human-baseline metric version and its absolute-jerk output are unchanged.

| Final 2.5 m/s interval | Largest negative jerk, magnitude | Largest positive jerk |
|---|---:|---:|
| Eleven marked manual stops, median | 0.415 | 1.107 |
| Eleven marked manual stops, maximum | 0.794 | 2.887 |
| Bad 20bf segment 34 | 2.497 | 1.795 |
| Bad 20c0 segment 22 | 2.973 | 1.596 |
| Bad 20c1 segment 5 | 2.151 | 1.978 |
| Bad 20c1 segment 8 | 1.731 | 2.191 |

The manual maxima are per-column bounds from different stops. Negative-jerk
magnitudes do not overlap between these eleven manual and four bad events;
positive peaks overlap. Segment 8's largest absolute peak is positive, so the
sign of the absolute maximum alone does not identify all the bad stops.

Units are m/s³ from `aEgo`, which is wheel-derived acceleration, not independent
body acceleration or brake force. These selected, unmatched events describe the
reported fault; they do not establish a universal comfort threshold or superiority.

The latest negative-jerk windows end at 1.483/1.476 m/s. Their subsequent positive
windows end about 0.5 s later, at 0.810/0.901 m/s. Each pair's 300 ms windows do
not overlap. Final-0.5 m/s absolute jerk is only
0.973/0.432, versus the marked-manual median 1.030. Neither latest stop has speed
recovery in its final 2.5 m/s interval. The older bad stops retain 0.245/0.218 m/s
recovery and remain separate regression targets.

Recorded-input replay places the candidate's smaller command pulse inside both
latest negative-jerk intervals. Neither interval has a binding safety lane or an
active recovery cap. The new phase is bounded by the previous command; as the raw
phase starts releasing, the earlier shallower command history retains the benefit.
Later planner safety can bind and the two commands converge. The replay does not
predict the resulting physical jerk. One exact peak timestamp is between replay
rows, so this attribution uses the bracketing recorded samples.

## Additional route evidence after deployment

Downloaded the remaining 22 segments of route `000020c1--8f82c447fd`, giving 31
complete, hash-checked rlogs. Their init records all identify `7f3bbaa6a0`; these
are additional pre-update observations, not a post-deployment drive. There are no
additional bookmarks. Three additional automatic stops meet the existing engaged-v2
selection rule. Unmarked manual manoeuvres are not added to Radek's reference.

The unchanged deployed candidate was replayed against the previous device code
(`761f4cef14`, also the runtime in `f1954d21b2`). It reduces the first-second command
minimum in all three additional selected stops:

| Stop | Previous minimum | Deployed candidate minimum |
|---|---:|---:|
| 20c1 segment 9 | -1.727 | -1.659 |
| 20c1 segment 11 | -1.640 | -1.497 |
| 20c1 segment 13 | -1.521 | -1.493 |

Units are m/s² of requested acceleration. Within these three stop windows there is
no extra braking and no newly binding safety frame. The full route has 182,240
control frames and zero ownership differences. Its first 53,346 replay rows exactly
match the previous partial-route artifact. Across the expanded six-packet comparison,
13 of 24 selected first-second minima are shallower; eleven are unchanged and none
is deeper. This concerns minima, not pointwise command equality. Ownership
is identical across 610,004 control frames. The previous stress-plant limitations in
`braking_prediction_2026-09-12.md` still apply; this extension does not resolve them.

The evidence supports evaluating this deployed correction. It does not support a
claim that it already beats Radek. The next comparison must retain signed entry jerk,
recovery, terminal easing and rest gap together, so improving one cannot hide another.

## Architecture hypothesis checked, without changing control

The existing whole-approach replay takes radar snapshots at plan publication, after
the model-triggered planner call. A nine-segment audit paired each published plan
with its recorded `modelMonoTime` and used source publications at or before that
model timestamp. All 10,677 plans have a unique model match; plan publication follows
model publication by 5.48–55.16 ms.

The original logger-order plan snapshots contain 2,381 repeated radar samples in
10,676 usable frames. Their consecutive-fresh runs have median 3 and p95 9 frames.
The model-paired publication snapshots have no repeats and 10,672 consecutive fresh
frames after four invalid startup samples. Shadow releases attributed to radar fall
from six to zero. Publication time still does not reveal actual `SubMaster` receipt
batches or `alive`; the paired audit's 500 ms age check is an `alive` proxy, distinct
from the old replay's 100 ms freshness gate. These counts establish timing
sensitivity, not a measured production radar defect or its complete absence.

The unchanged shadow certificate still enters repeatedly: six times at the segment-5
bad stop and five at segment 8. Across the audited route its release reason is
`gap` 14 times, reversing-lead evidence four times, and disengagement once. Excluded
commitments remain zero under the audit's existing exclusion checks. None of these
five stop windows contains the existing stopped-lead >=6 m/s capture crossing, so
that capture metric cannot assess them. Repeated commitments remain; the revised
timing changes their attributed reasons, not this unresolved behavior.

Instrumenting those 14 compound `gap` releases without changing any predicate
identifies nine outward lower-bound holds and five rejected inward holds. None has
a dropout, absent signals, or nonfinite/nonpositive raw or conditioned gap. Segment
5 contains three of each kind; segment 8 contains two of each. These distinct
provenances must not all be described as lost trustworthy geometry. The original
summary and every stop result reproduce exactly with this extra instrumentation.

This rejects a runtime reset/hold patch based on apparent radar churn. The existing
whole-approach governor remains OFF. Its next evaluation needs correctly paired
inputs and separate treatment of conservative outward holds and rejected inward
jumps. Relaxing all held-gap checks to improve capture counts would not demonstrate
better stopping.

## Reproduction

Artifacts are under `~/.route_sync/corpus/radek_baseline_20260912/approach_audit/`:
`deployment.json`, `marked_comparison_signed.json`, `route_20c1_complete/`,
`route_20c1_complete_replay.json`, `route_extension_comparison.json`, and
`radar_cadence_20c1.py`/`.json`, and `gap_release_components_20c1.py`/`.json`.
The signed comparison uses the same labels and eight packet directories as
`marked_comparison_with_20c1.json`; reference packets are its first five directories.
The other three remain the previously selected automatic comparison packets.

Activate `.venv`, then run the existing `marked_comparison.py` CLI with those inputs.
For the full route, use `entry_replay.py --base f1954d21b227afb38e4a33b5798098089476f830`
and the 31 route rlogs. Tests for `human_baseline`, `marked_comparison`, and
`bookmarked_baseline` pass: 25 tests. Ruff and diff checks pass. No driving code or
setting changed during this follow-up.

Independent Claude review found no blocking code issue. Its attribution review
prompted the compound gap-reason split above; the main agent read both diagnostic
scripts and verified their recorded source hashes and unchanged summary assertions.
