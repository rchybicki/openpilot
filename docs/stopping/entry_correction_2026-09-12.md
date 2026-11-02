# Stop-entry correction, 2026-09-12

The governor can deepen braking as it takes over from an approach that is already decelerating.
Add a bounded entry correction using the existing predicted stopping demand and episode clock.
The change reduces the requested pulse at both new bad bookmarks. This is a candidate for device
evaluation, not proof of improved physical jerk, a complete cure, or better-than-human stopping.

## New evidence

The device and route init data identify revision `7f3bbaa6a0a0b8d3c28ae9fab266d9200416909e`.
The previous context-reset fix is therefore active. Complete rlogs 0–8 of
`000020c1--8f82c447fd` contain two explicitly bad bookmarks:

| Segment | Button timestamp | User-bookmark timestamp | Matched stop timestamp |
|---|---:|---:|---:|
| 5 | 455402129883 | 455409918417 | 454849197619 |
| 8 | 593470099997 | 593471433495 | 588680250501 |

Both automatically match one sustained rest interval. The original eleven explicitly marked
manual stops remain Radek's references. No manual identity or rating is inferred for other stops.
The new route also contains two engaged nearby stops in segments 1 and 4, and one mixed-control
stop in segment 0. The two nearby engaged stops are included in replay; segment 0 is not a
personal reference or a clean automated comfort comparison.

At the first owned frame, segments 5/8 have speeds 2.224/2.155 m/s, raw gaps 7.200/7.000 m,
and measured accelerations −0.917/−1.190 m/s². The fresh service context agrees with the raw gap.
Both enter APPROACH_GLIDE from INACTIVE. They do not reproduce the earlier stale-context defect.
The governor then requests a deeper pulse and releases it. The recorded final-2.5-m/s windows
have 300 ms wheel-acceleration jerk 2.151/2.191 m/s³ and no speed recovery. The older bad rebound
case remains a separate regression case; these new marks do not replace it.

## Change and limits

The correction uses `predictive_lead_demand` with the **rest anchor**, rather than its safety-lane
anchor. Credit is limited to measured deceleration beyond that predicted demand. Multiplying
that surplus by the existing lag/pursuit ratio gives a speed-error correction. It fades with
the same response-lag time constant from the existing service-entry timestamp.

The corrected phase is bounded between its original demand and the greater of that demand and
the previous command. It cannot deepen the phase, release an existing brake command by itself,
or demote a binding safety lane to the comfort jerk rate. Untrusted geometry, missing/nonfinite
acceleration, insufficient braking, or a failed prediction earns no credit. An exception keeps
the uncorrected governor. The attributed-safety fault test now injects its fault at its own
2 m anchor; a separate test covers a failed shared prediction at the rest anchor.

There are eleven added runtime lines, no new settings, flags, state variables or controller.
The governor profile, recovery-release limit, safety formulas, terminal descent and hold remain
in their existing paths. Earlier changes can still alter a later terminal seed or estimator state.
The phase-entry clock does not restart at warm ownership transfer or RELEASE re-assertion;
those paths do not receive a fresh correction. Shadow commands are observer commands, not proof
of what the car would do. `a_gov` telemetry remains the uncorrected profile demand.

The reused predictor includes a 2.5 m/s² lead-braking assumption and a 0.30 m denominator floor.
Changing those also changes this correction. A moving lead adds predicted travel before stopping;
braking harder than that assumption can invalidate the forecast. Existing safety arbitration is
retained, but the forecast is not a physical safety certificate.

Lifting a phase can promote an existing safety lane to its faster rate, so command arbitration
was checked explicitly. The legacy normalization and late-entry corridor are disabled under
the governor and do not combine with this correction.

## Recorded-input replay

`entry_replay.py` loads the actual shipped service from `7f3bbaa`, retaining the same LongControl,
context and flags in both arms. It rejects other runtime differences. Missing segments reset
state. Rlogs, runtime sources, baseline module and runner are hashed; inactive infinite debug
lanes are encoded as null. Both arms retain the existing recovery brake.

The comparison retains 21 automated stops: the previous 17 plus four new engaged stops, including
all four explicit bad marks. Seventeen have changed commands and four are identical. All six
route packets and all selected windows preserve service ownership. The largest additional
brake request in a selected window is 0.00141 m/s²; the largest reduction is 0.19787 m/s².
Three moving frames promote a safety lane, without a new fast downward step on the new route.

Pulse depth below is the minimum request in the first second after shipped-arm ownership:

| New-route stop | Deployed replay | Candidate replay | Reduction in depth |
|---|---:|---:|---:|
| Segment 1, unmarked | −0.984 | −0.890 | 9.5% |
| Segment 4, unmarked | −1.762 | −1.572 | 10.8% |
| Segment 5, bad bookmark | −1.737 | −1.670 | 3.9% |
| Segment 8, bad bookmark | −1.794 | −1.597 | 11.0% |

Units are acceleration requests in m/s². These are command reductions, not measured comfort
gains. The first bad mark improves modestly and retains a pulse. The older segment-34 bad
bookmark has unchanged first-second minimum; this change does not establish removal of its
recorded rebound. The segment-22 bad stop from the previous route changes −0.741 to −0.641.

For the new route, the shipped replay's moving-owned command MAE is 0.00375 m/s² and maximum
error 0.04236. That supports input reconstruction. Older routes were recorded on earlier
revisions: their 7f3bbaa replay is a common-code comparator, not an exact reproduction of that
older firmware. For example, old segment 22 has selected-window MAE 0.1298. Logged subscription
timing is approximate. Recorded acceleration, speed and lead motion remain fixed in both arms;
replay cannot predict the new rest gap, physical jerk or later rebound.

## Validation

The full controls-lib suite passes: **777 passed, 19 skipped**. Targeted ruff checks pass.
New tests cover bounded authority, unchanged safety demands, immediate hazard braking, missing
and invalid evidence, prediction failure, clock expiry/reset, warm reseeding, other approach
paths, terminal descent, and a coupled weak-braking regression.

The frozen synthetic sweep uses actual LongControl and StopContext with a first-order actuator,
eight entry conditions, gain 0.7/1.0/1.3, lag 0.15/0.4/0.7 s, exact delay 0/0.3/0.6 s and load
−0.45/−0.1/0/0.2/0.45 m/s²: **1,080 paired cases**. There are no new floor crossings, failed
stops or creep cases. Both arms have the same 80 cases below 3 m and the same two incomplete
stops; those failures are retained. All 1,000 baseline cases that meet the 3 m floor still meet it.
This does not validate the controller's physical floor against those adverse plants.

Rest-gap changes range from −1.195 to +0.100 m; median change is zero. The 300 ms synthetic
acceleration-jerk change ranges from −2.492 to +1.196 m/s³; median change is zero. The worst jerk
regression is an entry at 2.4 m/s, gap 7 m, initial command −2.0, gain 1.3, lag 0.15 s,
delay 0.3 s and load +0.2: jerk 2.301→3.497, rest gap 5.315→4.489 m. This is not uniformly
smoother in simulation. The plant is uncalibrated and uses clipped physical speed, not the car's
wheel-filter observation model; its jerk is not a prediction of felt jerk.

An additional 864 moving-lead cases cover initial lead speeds 0.5/1.0/2.0 m/s and lead braking
1.0/2.5/4.0 m/s², three gaps, two incoming commands, gains 0.8/1.2, lags 0.15/0.6 s, delays
0/0.6 s and loads −0.45/+0.45. There are no new floor crossings, failures or creep cases.
Both arms retain 79 floor violations, five incomplete stops and one creep case. Gap changes are
−0.498 to +0.121 m; synthetic jerk changes are −1.537 to +1.534 m/s³. These are further stress
checks, not evidence of uniform smoothness or a validated moving-lead safety margin.

Independent final review found no blocker for a small supervised device-evaluation candidate
and reproduced the then-current 776-test result and frozen evidence. Follow-up tests pin outward
hold credit and verify that the attributed fault injection actually fires; all 777 tests pass.
The runtime is unchanged after that review. The additional moving-lead sweep addresses its
least-sampled scenario while retaining all failures and jerk regressions.

Exploratory blanket damping, slower pursuit, and handoff smoothing failed regression checks.
Crediting all measured braking also introduced a floor crossing. The retained correction uses
only the surplus over the predicted rest demand. Barrier-load changes were rejected; no barrier
change is included. Earlier exploratory delay buffers included one extra 10 ms tick; the frozen
sweep uses the stated exact delays. The separate weak-braking regression explicitly uses 10 ms.

Vehicle evaluation must check both marked pulse classes, the surrounding stops and the eleven
personal references together: brake onset, release/rebrake, speed recovery, final ease, gap and
secure hold. No learned controller or superior-to-Radek outcome is claimed.

## Reproduction

Local evidence is under `~/.route_sync/corpus/radek_baseline_20260912/`: `labels_with_20c1.json`,
`marked_comparison_with_20c1.json`, six packet `entry_replay.json` files, `entry_comparison.py/json`,
`entry_sweep.py/json`, `entry_moving_sweep.py/json`, and `entry_bookmarks.png`. The final manifest
is `entry_evidence_manifest.json`; it binds the reviewed runtime and the added checks separately
from the earlier review snapshot.
Final SHA256: `0bb10201013fde8aa229833e881bd8decdd9ab5ef08aa4f698f5f89ea5ecfed0`.

```sh
source .venv/bin/activate
python tools/stopping/review/entry_replay.py --output /tmp/entry_replay.json \
  ~/.route_sync/data/media/0/realdata/000020c1--8f82c447fd--{0..8}/rlog.zst
python ~/.route_sync/corpus/radek_baseline_20260912/entry_comparison.py
pytest selfdrive/controls/lib/tests/test_stopping_entry.py
```
