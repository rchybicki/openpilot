# Braking prediction candidate, 2026-09-12

This change reduces the requested entry pulse at both latest bad bookmarks and at other
recorded stops. It is a candidate for supervised device evaluation. It does not establish
lower physical jerk, a complete cure, or stopping better than Radek.

## Control change

The raw governor reserves distance for the response lag while comparing its speed profile
with current speed. An already-braking car can therefore receive an additional brake pulse.
The old entry correction credits only surplus braking and expires after entry.

The replacement projects both ego speed and relative travel over the existing 0.45 s lag.
Ego travel ends when ego reaches zero; a reversing lead keeps consuming distance. The
forecast acceleration is measured braking multiplied by a continuous weight:

`clip((predicted_rest_demand - measured_braking) / GOV_A_C, 0, 1)`.

No braking surplus earns no prediction; a surplus equal to the existing 0.6 m/s² comfort
deceleration earns full prediction. This weight is a bounded control choice, not a fitted
probability or a vehicle-response certificate. No new setting, state, or tuning constant
is added. Prediction no longer expires just because an entry timer elapsed.
`GOV_A_C` now controls both the comfort profile and this weighting scale; future tuning
must evaluate both effects. Measured acceleration also enters the rest-demand calculation:
an optimistic measurement can increase credit through both terms. The weight bounds the
forecast but does not remove that feedback risk.

The phase remains between its raw demand and `max(previous_command, raw_demand)`.
Prediction can prevent additional braking; it cannot release the previous command by
itself or deepen the raw phase. Untrusted geometry earns no prediction. Recovery,
terminal descent, hold, ownership, and all safety demands retain their original inputs.
Newly binding safety demands retain `J_SAFE`; comparing against the unsoftened phase
would hide a possible braking deficit and was rejected during review.

## Recorded-input evidence

The baseline is `f1954d21b227afb38e4a33b5798098089476f830`, whose driving code is the
previous entry candidate `761f4cef14`. The six original packets contain 481,110 control
frames and 21 selected automatic stop windows. All four bad bookmarks remain included.
The eleven explicitly marked manual stops remain the personal baseline; no driver identity
or rating is inferred for other stops. Their final 2.5 m/s windows are manual; wider windows
are not all manual and must not be pooled as if they were.
The final read-only SSH check returned device revision `761f4cef144dda7abf8345389b2ce57e458ad9b6`.

Seventeen selected windows change. Ten first-second command minima become shallower;
the other eleven are unchanged. Ownership is identical across all 481,110 frames.

| Stop | Previous candidate minimum | New minimum | Reduction in depth |
|---|---:|---:|---:|
| 20c1 segment 5, bad bookmark | -1.670 | -1.592 | 4.6% |
| 20c1 segment 8, bad bookmark | -1.597 | -1.293 | 19.0% |
| 20c1 segment 4, nearby stop | -1.572 | -1.347 | 14.3% |
| 20b8 segment 4 | -1.794 | -1.448 | 19.3% |
| 20bf segment 34, older bad bookmark | -0.651 | -0.651 | unchanged |
| 20c0 segment 22, older bad bookmark | -0.641 | -0.635 | 0.9% |

Units are m/s² of requested acceleration. The two latest bookmarks were recorded on
`7f3bbaa6a0`; against that revision's replay minima (-1.737/-1.794), the combined reduction
is 8.3%/27.9%. These are command comparisons, not measured physical acceleration gains.

Across the 21 windows the maximum extra braking is 0.0114 m/s² and the maximum release
is 0.346 m/s². Three frames newly bind safety, all in 20c0 segment 20. Both latest bad
bookmarks have no new binding frames. The older rebound bookmark is not declared fixed.

Recorded speed, acceleration, lead geometry and planner outputs are held fixed. They do
not predict the motion after a changed command. Input assembly approximates subscription
timing. Older routes used different revisions; their baseline disagreement with recorded
commands is not evidence of a candidate effect. The earlier `entry_replay.json` packets
retain the closer 7f3bbaa comparison for the latest route.

## Regression evidence

The actual LongControl and StopContext run against 1,080 stationary-lead and 864 braking-lead
cases, repeated with the native wheel-speed Kalman recurrence and a hypothetical quantized
50 Hz wheel input. Both arms receive the same position-dependent planner proxy, gain,
lag, exact command delay and constant external push. These are stress plants, not calibrated
vehicle models. Each run continues for one second after physical zero speed.

All 3,888 paired comparisons have no new crossing of the 3 m test floor, incomplete stop,
or creep event. This is a regression gate, not a universal safety claim: the baseline
already fails the floor in 159 ideal-observation and 157 wheel-observation cases, and has
seven incomplete stops in each set. The ideal set has one existing creep event.
The minimum absolute gap is 0.9853 m in both arms and both sets (an already-failing stress
case). Among baseline cases below 3 m, worst additional intrusion is 0.000132 m in the
ideal set and zero in the wheel set.

Comfort is not uniformly improved in these hypothetical plants. Worst 300 ms jerk increases
are 0.980/0.899 m/s³, and worst gap reductions are 0.685/0.767 m (ideal/wheel). Median changes
are zero. At the worst jerk cell, initial speed/gap/command are 2.4 m/s, 7 m, -2 m/s²;
gain 1.3, lag 0.15 s, delay zero, push 0.45 m/s². Rest remains above 4.2 m. This residual
uncertainty must not be turned into a claim that the car now stops better than the driver.

820 targeted tests pass, 19 skip. The restored original crawler-depth assertion also passes.
Tests cover forward prediction, zero-speed truncation, signed lead motion, unavailable
inputs, geometry trust, no prediction-driven release, smooth weighting, safety-rate
promotion, the weak-braking floor case, terminal descent and hold. Ruff and diff checks pass.
The noise-step test is local to its stated operating point, not a global derivative bound;
the existing jerk limiter remains the runtime rate limit.

More aggressive full prediction, a Boolean feasibility gate, and a continuous demand cap
were rejected: they introduced floor/creep regressions or a large noise-triggered phase
step. The final candidate retains continuously earned surplus and the prior-command bound.
The final independent Claude review found no blocking issue. Both final repository sweeps
reproduce the prototype rows exactly, with current source hashes verified.

## Reproduction

Artifacts, original packets, labels, comparison script, plots and source hashes are under
`~/.route_sync/corpus/radek_baseline_20260912/prediction/final/`. `evidence_manifest.json`
pins the final artifacts and the unchanged marked baseline. Activate `.venv` first.

```sh
python tools/stopping/review/entry_replay.py --base f1954d21b227afb38e4a33b5798098089476f830 --output RESULT.json RLOGS...
python tools/stopping/review/prediction_sweep.py --base f1954d21b227afb38e4a33b5798098089476f830 --output IDEAL.json
python tools/stopping/review/prediction_sweep.py --base f1954d21b227afb38e4a33b5798098089476f830 --wheel-observation --output WHEEL.json
```

The device evaluation must check the whole approach, including the entry dip, any return
to acceleration, terminal jerk and rest gap. The current data support testing this candidate;
they do not justify a superhuman-stopping claim or elimination of the next measured check.
