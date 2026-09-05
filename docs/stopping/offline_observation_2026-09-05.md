# Frozen-model observation sensitivity -- cycle 47, 2026-09-05

The missing wheel-filter coupling does **not** explain the frozen model's complete-stop failure. Keep that
model rejected as a comparator for changed comfort commands. The next experiment needs a full-episode
speed/distance objective and an explicit standstill observation rule. No driving code or parameters change.

## Design and fixed comparison

R1 design review `20260905-082632-exec` rejected the initial 10 Hz approximation before implementation.
Under linear acceleration between endpoints it is mathematically consistent, but reconstruction error on
the nine recorded episodes reaches 0.049611 m/s and 0.346591 m. The speed error is almost the 0.05 m/s stop
threshold. It cannot support stop-time/completion conclusions.

The corrected experiment propagates only the frozen fitted model at 100 Hz, using the existing PlantModel
re-discretization and the same selected 10 Hz command held for ten ticks. One branch uses the source identity:

```
a_next = model.predict_next(a_prev, command, v_prev)
v_raw = v_prev + 0.01*a_prev + 0.1049026475*(a_next-a_prev)
v_next = max(0, v_raw)
x_next = x_prev + 0.005*(v_prev+v_next)
```

A native-rate control omits only the final filter-coupling term in `v_raw`. This separates finer stepping
from the coupling itself. Initial state is measured once. There is no refit, later measured-state reset or
per-stop correction. Raw predicted acceleration remains after the speed clamp; negative raw-speed ticks,
incomplete stops and relaunches are counted. No alternate rest-gap metrics are calculated.

The runner imports the frozen cycle-45 entrypoint, captures its exact inputs and recomputes its outputs in
a temporary directory. Every original result/trace field matches the saved baseline exactly, including
prototype/archive models, all 34 IDs, nine admitted episodes, 25 exclusions, boundaries and command grids.
The original packet is unchanged. Path RMSE uses the same original 10 Hz moving samples for all branches;
new stop/clamp/relaunch detection uses native ticks. There are six training-route episodes and three
**development-validation** episodes, not a new held-out cohort. The 17.14 m/s onset remains labelled beyond
the fitted body's 15 m/s maximum; all three validation episodes are retained.

Before the run, the R1 rule required every validation episode to have travel error <=25% of original,
speed RMSE <=50% of both comparators, completion without relaunch, no more than 0.1 s added stop-time error,
no more than 10% added acceleration RMSE, and no numeric failure. The six training cases could not gain
noncompletion, relaunch or numeric failure. This is a mechanism-rejection rule, not final acceptance.

## Results

Distance is signed predicted travel error at the recorded stop time. Positive means too far.

| Route 2073 segment | Original 10 Hz (m) | Native no coupling (m) | Native coupled (m) | Coupled speed RMSE (m/s) | Coupled stop-time error (s) |
|---|---:|---:|---:|---:|---:|
| 14 | +6.315 | +6.872 | +5.790 | 0.405 | +0.029 |
| 18 | +1.436 | +2.167 | +0.698 | 0.191 | -1.713 |
| 24 | -18.256 | -17.381 | -19.097 | 0.974 | -2.719 |

All three fail the travel and speed reduction conditions. Segment 18 also misses the stop-time condition.
Coupling changes the native control's displacement by -1.082, -1.469 and -1.716 m, but is insufficient to
explain the original errors and worsens the largest one. All three complete; no relaunch or numeric failure
occurs. Training completion remains 5/6, preserving the original censored noncompletion; no new regression
in the specified completion/relaunch/failure checks. This does not mean every training distance improves.

The coupled validation runs contain 284, 457 and 568 negative raw-speed ticks. Their acceleration state is
not an adequate standstill wheel-observation model. Do not reinterpret the fitted aEgo state as identified
physical actuator acceleration, or infer motor gain/lag from this sensitivity test.

## Evidence and next step

Packet: `~/.route_sync/corpus/cycle47_observation/`. `observation.py` runs the real frozen baseline, both new
branches, exact KF identity assertions, and synthetic clamp/rest/relaunch checks. Result JSON retains every
candidate ID/exclusion and all nine episode metrics; native traces are separate. Ruff passes. The unchanged
cycle-45 verifier still passes its raw-data, source, result, cohort and analytic checks.

R2 evidence review `20260905-083954-exec` completed with exit 0: **PASS. Accept the numerical rejection.**
The reviewer reran the experiment and Ruff, verified the unchanged cycle-45 packet, independently checked
thresholds/cohort counts, and matched every manifest hash before and after the run. No correction was required.
Main-agent signoff accepts this result. The pre-review manifest is retained; a final manifest records the
review-status update and review receipts. No numerical artifact changed.

Do not add another controller law or activate the whole-approach governor.
Next: specify the whole-episode objective and stationary observation model before fitting. Preserve the
current frozen comparators and use additional untouched route groups for later acceptance.
