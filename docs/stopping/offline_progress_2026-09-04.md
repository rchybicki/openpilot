# Offline stopping progress -- cycle 44, 2026-09-04

We can continue with the existing logs. A new body-braking fit predicts the next two seconds better than
the prototype gain table on a separate route. This is a development result, not full-stop validation.
The next step is **free rollout through the complete stop**, followed by checks on more route groups.

## Data and checks

- Routes 00002072--e4e928b760 and 00002073--b2443011ed, both ec334a3c: 75 rlogs, 445,633 car-state samples.
- Joined segments recover 34 stops instead of 32: 14 engaged, 8 mixed, 12 manual. Every previously valid
  v2 metric matches the earlier census. Stops still require >=3 m/s in the preceding ten seconds.
- The loader's input order is not timestamp order. Sorting each segment removes false future/stale pairs;
  every admitted braking frame then needs valid, non-future command/state/plan/radar inputs. Raw sent
  command is `carOutput`; no fallback to `carControl`. No interpolation across excluded intervals.
- 85 continuous pedal-free braking runs, 706.98 s total, at .5-15 m/s and sent command [-3, 0) m/s².
  These are body-braking runs, not 85 stops. The three-second minimum is for this diagnostic only.

## Predictive comparison

Used the existing `fit_plant_model.py` seven-feature model and `PlantModel` rollout, not a new controller.
Fit on 57 runs from 2072; validate on 28 runs from 2073. Both routes were inspected previously: this is
a development split, not untouched final acceptance data. Identical starts provide 1.5 s of recorded
command history and measured initial acceleration/speed. Later acceleration and speed are predicted,
with no measured-state reset. There are 104 non-overlapping two-second horizons from 27 validation runs;
one run is too short. Windows within a run are correlated and longer runs contribute more observations.
Acceleration-path statistics pool 2,080 errors per model (104 horizons x 20 predicted frames); distance
statistics use 104 horizon errors per model.

| Model | Acceleration path absolute error, median / p90 (m/s²) | Two-second distance absolute error, p90 (m) |
|---|---:|---:|
| Archived 2026-05-14 plant | 0.565 / 1.491 | 2.238 |
| Existing prototype gain table, delay/lag model | 0.099 / 0.332 | 0.498 |
| Fit from current body-braking data | **0.065 / 0.189** | **0.272** |

Statistics use nearest-rank p90. The prototype retains its existing next-time Euler update; the fitted
model retains its k-to-k+1 convention, both at 0.1 s. These are two-second distance errors, NOT rest-gap
errors. The fitted delay is zero frames and the AR pole is 0.675: predictive fit does not establish the
physical actuator delay or justify an inverse-gain controller. No model or comfort-law activation follows.
The recorded commands came from a feedback controller; good prediction with those commands alone does not
prove the same accuracy for changed commands. Response direction, delay and command-range sensitivity remain open.

## Stop-cause evidence and next experiment

The 2072-s11 bookmark remains a useful mechanism case. Its late peak is about -0.94 m/s² while the sent
command is -0.75 and the lead is still slowing. Later, the existing shadow flags 104 unexplained plan-bound
frames. Separate those two phases; do not attribute the entire late peak to the later comfort conflict or
assume that removing the conflict would produce a 4.3 m rest. The bookmark is a reported issue, not a
specific numerical comfort rating. `bookmarkButton` and `userBookmark` are two streams for the same press.

Only three stops have an entirely fresh, driver-free, single-identity lead window under the initial
30-second diagnostic selection. That fixed window can include earlier manoeuvres; it is not a complete
causal episode detector. The archive retains full routes for a proper brake-onset-to-rest reconstruction.
Next: reconstruct that interval and one lead trajectory per accepted stop; run the candidate without
state resets through the low-speed transition and rest; score acceleration, stopping time and distance.
Validate on additional route groups and check response sensitivity before using it to rank comfort laws.
If the model fails, use the residuals to identify the missing regime before requesting new collection.

## Reproduction

`~/.route_sync/corpus/cycle44_offline/` contains frozen `audit.py`, `plant_check.py`, `verify.py`, two route
NPZ files, raw service events, the fit/results and `manifest.json`. The manifest records 75 input paths,
sizes/mtimes and artifact hashes; raw rlogs were not hashed. From the repo in `.venv`, run
`python ~/.route_sync/corpus/cycle44_offline/verify.py` to check the saved packet without changing it.
For a fresh extraction/fit, copy the packet to a sibling directory and run its `audit.py`, then
`plant_check.py`; compare numerical results with the saved packet. Keep the original manifest unchanged.
The fitter's rollout self-check uses a known free-decay solution. Verification checks frozen hashes,
input identity, route/sample/stop counts, prior metric equality, every admitted frame, matching horizons,
the train/validation counts and recomputed summary statistics.

Evidence review `20260904-232105-exec` (sol xhigh, exit 0) supports the limited diagnostic result. It
required explicit count assertions, comparison with frozen hashes and the path-error denominator above.
All three corrections are complete. Only the verifier changed after review; the other artifact hashes
match the reviewed packet. Main-agent checks pass, including rejection of altered hashes/input metadata
and proof that verification leaves the manifest unchanged. No second review was needed for these fixes.
No driving code, flags, on-device files, or production plant parameters changed.
