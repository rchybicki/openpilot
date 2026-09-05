# Full-stop model test -- cycle 45, 2026-09-05

The frozen cycle-44 body fit **fails complete-stop prediction**. Its good two-second acceleration score
does not retain sufficient speed/distance accuracy over a complete braking episode. Do not use it to rank
changed comfort commands. Cycle 46 reconciled the measured acceleration/speed relationship from the wheel
filter source; see [the follow-up](retrospective_2026-09-05.md). Next fit free-rollout speed and distance over full episodes. Do not add a terminal controller patch.
No new drive is needed to investigate this failure. No driving code, parameters or device files changed.

**Review status:** offline rejection result, retained with the cycle-46 deployment evidence. Design review `20260905-070326-exec` (sol xhigh)
ended with exit 1 at the review tool's usage limit, without a verdict. Main-agent checks below pass. This
report was initially held locally. Cycle-46 final review considered the bounded evidence context; it does
not turn this failed plant into an accepted comparator or validate a new comfort law.

## Fixed experiment

- Same 75 rlogs / 445,633 car-state samples from routes 2072 and 2073, driven on ec334a3c. Reused the cycle-44
  extraction with one extra column, `long_state`; every prior array and audit result is identical.
- Loaded the exact saved cycle-44 coefficients and delay, guarded by the saved fit's SHA256. No refit.
  Comparators are the same prototype gain table and archived PlantModel. Source hashes are frozen too.
- 2072 was used in fitting and remains a training-route diagnostic. 2073 is development validation, already
  inspected. Neither is final acceptance data. The >=100 held-out clean-stop requirement remains unmet.
- Every one of the 34 v2 stop IDs is retained. Nine complete braking episodes pass admission: six on 2072,
  three on 2073. Twenty-five are excluded. Nonexclusive reasons: 13 have no qualifying sent-command onset,
  11 include inactive longitudinal control, nine include driver input. These counts are not added together.

Operational onset: search back no more than 60 s, bounded after the previous qualified rest. After the last
positive command >=0.1 for >=1 s, select the first command <=-0.05 for >=0.5 s. Require onset speed >=3 m/s,
an observed preceding nonbraking sample and >=2 s of actual command history. Do not move to a later brake
phase if the first phase fails admission. Short command releases remain inside the episode. This is an
explicit selection rule, not ground-truth driver intent or coverage of all stop classes.

Admission checks every source frame, before resampling: valid finite state and actual `carOutput`, fresh
nonfuture sources, active longitudinal control, no pedals or force coast, no gap >100 ms. `stopping` is an
allowed control state; `off` is excluded. Observation continues to stop+3 s, or the first trust/driver boundary
or recorded departure (v>=0.1). Censored prediction is not extrapolated with a held command.

At 0.1 s, measured acceleration and speed initialize the model once at onset. Every later state is predicted.
PlantModel retains k-to-k+1 timing and the prototype its next-time Euler convention. Speed is clamped at zero;
raw model acceleration is not reset at observed rest. Distance uses trapezoidal speed integration. The
recorded reference uses native-rate speed integration. Predicted stop requires v<0.05 for >=0.5 s.

## Development validation: each stop, not only an aggregate

Positive distance error means excess predicted travel at the recorded stop time. Negative time error means
the predicted stop is early. Distance error at this common time is not automatically a physical rest-gap error.

| Route 2073 segment | Onset speed (m/s) | Episode (s) | Distance error at recorded stop (m) | Stop-time error (s) | Distance error already present at last measured v>=0.5 (m) |
|---|---:|---:|---:|---:|---:|
| 14 | 10.66 | 15.38 | +6.315 | +0.119 | +5.942 |
| 18 | 13.91 | 12.59 | +1.436 | -1.593 | +1.793 |
| 24 | 17.14 | 23.03 | -18.256 | -2.629 | -17.960 |

Segment 24 also exceeds the fitted body's 15 m/s maximum. It stays in the results and is labelled as extra
speed extrapolation. The other two already show failure within the fitted body's speed range. All cases
extrapolate below its 0.5 m/s minimum. These are three selected stops, not a population-level p90 estimate.

| Model | Validation distance absolute error, median / maximum (m), n=3 | Predicted stops / admitted | Maximum stop-time absolute error on completed cases (s) |
|---|---:|---:|---:|
| Frozen body fit | 6.315 / 18.256 | 3 / 3 | 2.629 |
| Prototype | 15.103 / 15.714 | 3 / 3 | 1.019 |
| Archived plant | 14.191 / 369.940 | 2 / 3 | 7.581 |

The archived-model failure is retained. On the training-route diagnostic, the body fit completes 5/6; one
prediction cannot establish a half-second rest before the recorded car departs. Its common-time distance
errors still include all six cases (median absolute error 6.927 m, maximum 8.463 m). Completion-only timing
and gap statistics are not substituted for the full cohort. No numeric divergence occurred in these runs.

[Validation plots](/Users/radoslawchybicki/.route_sync/corpus/cycle45_full_stop/full_stop_validation.png) show
recorded/predicted speed and accumulated distance error for all three validation stops.

## What the failure identifies

Most displacement error is already present before the final low-speed phase. On validation, body acceleration
absolute error is median 0.085 / p90 0.245 m/s² over 467 predicted samples. Terminal error is 0.330 / 0.699 over
41 samples. Longer episodes and small acceleration biases can accumulate substantial speed/distance error;
a two-second or one-step loss is not sufficient evidence for the full-stop task.

A secondary analysis, specified after seeing the failure, integrates **recorded** acceleration from the same
initial speed. It misses distance by +0.202, +0.449 and +0.947 m on the three validation stops. This is a
measurement-consistency diagnostic, not a free-model score. It is much smaller than the large model errors,
but it also shows that acceleration-to-speed timing/filtering must be understood before claiming decimeter
accuracy. It must not become a per-stop correction fitted from the answer.

The model's raw acceleration stays negative under hold braking while observed wheel acceleration tends to
zero. The simulator clamps speed, but this raw acceleration output is not a valid standstill wheel-observation
model. Its stationary acceleration errors are reported separately and are not passenger-comfort scores.
Do not fit away this mismatch by changing live hold behavior.

Seven admitted episodes retain one real radar track through the recorded stop; only one is on validation.
The lead trajectory from ego distance + gap disagrees with integrated lead speed by -0.26 to -1.80 m at the
recorded stop across those seven cases. Thus the reconstructed trajectory is not independent decimeter ground
truth. The common-time gap error is exactly the negative ego-distance error. Predicted-rest gap is available
only with adequate same-track coverage; those provisional values are not an independent acceptance result.

## Next step and limits

1. Audit the timing/filtering relationship between `vEgo`, `aEgo` and sent command, and the lead-position
   reference. Preserve measured trajectories; do not hide drift with per-stop offsets or resets.
2. Reuse the existing plant structure and fitter, but evaluate an explicit full-episode speed/distance loss.
   Keep the cycle-44 fit as a fixed comparator. State any terminal observation rule separately from actuator
   dynamics. Predeclare the split and success rule before fitting; 2073 is now development data.
3. Only after the diagnostic failure is resolved, test on additional route groups and command sensitivity.
   Then compare comfort laws. Keep the whole-approach governor OFF and all existing capture/safety gates.

This cycle deletes no live lane because no replacement is validated. It rejects reliance on the short-horizon
fit as a full-stop comparator; it adds no runtime condition, tuning constant or dependency.

## Reproduction and local checks

Packet: `~/.route_sync/corpus/cycle45_full_stop/`. It contains extraction, replay, residual analysis, checks,
all episode exclusions, model traces, plot and a frozen manifest. SHA256 covers 75 raw rlogs, 15 artifacts and
nine source/reference files. From the repository in `.venv`, run:

```
python ~/.route_sync/corpus/cycle45_full_stop/verify.py
```

The verifier is read-only. It checks frozen hashes and counts, then exercises the real runner with synthetic
constant-deceleration distance/rest, free decay without resets, delay indexing, dwell gaps/EOF, onset/release
selection, pedal/stale/off/gap rejection and observation censoring. Saved trace dynamics, distance integration
and per-episode acceleration statistics are independently checked. Altered artifact/source/raw-input hashes
are rejected; manifest content and mtime remain unchanged. Existing plant/fitter/scorer tests: 47 passed.
For reruns, copy the packet to a sibling directory; preserve the original manifest and compare results.
