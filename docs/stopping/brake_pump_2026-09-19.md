# Brake-pump review and stopping ownership, 2026-09-19

This cycle adds an explicit stopping-ownership indication and a small, continuous
braking-prediction improvement. It does not eliminate every approach pump or establish
superiority to Radek's manual stops. The personal reference remains the same 11 marked
Radek stops; no other driver or unmarked manual stop was added.

## What the bookmark shows

The FF bookmark belongs to rest `000020ff--375628a698@1611438855714` on `5a9253be31`.
There are two separate pulses:

- Near 8 m/s, the legacy PID approach caps demand substantially more braking than the
  planner. The slowing-lead calculation predicts a complete stop, then switches to a
  relative-speed calculation when the lead crosses a speed threshold. At the observed
  boundary, cap demand changes from about -1.46 to -0.67 m/s². The planner itself is
  tightening more gently. This earlier pulse remains unresolved.
- At stopping-service entry near 2.22 m/s, another request ramp reaches about
  -1.55 m/s² before measured wheel deceleration responds. Recorded SCC12 requests
  confirm the command pulse. The existing 300 ms request observer is active; this is
  not a missing-history or CAN jerk-setting failure.

The old orange indication follows legacy `longControlState == stopping`, about
0.98 s after service ownership at the bookmark. A new `controlsState.stoppingControlActive`
field reports effective service or legacy stopping ownership, excluding driver overrides,
disabled longitudinal control and cruise handoff. The HUD checks message freshness and
validity. A new-drive early return also clears stale orange/red state.

## Small control change

Retain the existing clipped braking-surplus weight `w`, but forecast with `w * (2 - w)`.
The weight remains continuous, zero without sufficient estimated braking and one at
the existing full-surplus endpoint. It gives useful prediction credit earlier within
that interval. No parameter, state, flag, dependency or extra control branch is added.

The pending-request observer, geometry trust, previous-command bound, raw safety lanes,
recovery, terminal descent, hold, ownership and jerk limits remain unchanged. Request
history estimates braking still in transit; it is not confirmation of delivered force.
Prediction cannot add release beyond the raw law or make its phase demand deeper.
The final safety lane can still select the existing faster braking rate.

## Evidence and limits

The initial capture contains 40 finalized rlogs from five routes. A later refresh adds
22 rlogs, including FF segments 13–33 held aside after the formula was frozen. The full
captured FF route has 34 segments and 33 rests. There is no claim about unavailable FF34.
The extra route `00002100--1d4b9d86ff` is stationary and records the orange-UI version.

- Across the five initial automatic final-2.5 m/s windows, negative request jerk
  improves in all five; positive extrema improve in two and remain unchanged in three.
  Bookmark negative 300 ms request jerk is -2.0734 → -1.9067 m/s³, with minimum request
  -1.5704 → -1.5204 m/s² in chronological replay.
- In three additional automatic final-2.5 windows from the later segments, negative
  request jerk improves in two and is unchanged in one; positive extrema are unchanged.
  Segment 14 improves -1.2533 → -0.9418 m/s³. An additional engaged slow approach without
  a 2.5 m/s crossing remains unchanged. Segment 15's final-0.5 negative request jerk
  worsens slightly, -0.2548 → -0.2691 m/s³. Earlier PID pulses still dominate some whole
  approaches. All 77,650 overlapping replay rows reproduce both frozen arms exactly.
- All 3,888 paired hypothetical-plant cases retain the existing clearance, completion
  and creep outcomes. All 297 already-below-3 m case summaries are exactly unchanged;
  14 baseline incomplete stops remain. These are regression comparisons, not 3,888
  safety successes. Moving negative physical jerk improves in 667 cases, worsens in 97,
  and is unchanged in 3,124; worst deterioration is 0.0791 m/s³. Moving positive jerk
  improves in 542, worsens in 307 and is unchanged in 3,039; worst deterioration is
  0.1798 m/s³. Finishing tradeoffs also remain, including the example below.
- Native planner/controller simulations over 19 prior entry geometries in two fixed
  plants improve moving negative jerk in 29/38, worsen it in seven and leave two unchanged.
  Worst deterioration is 0.0669 m/s³; positive-jerk deterioration is at most 0.0116 m/s³.
  Stop-time changes range from -0.20 to +0.18 s and gap changes from -0.063 to +0.088 m.
- Six native acceleration-noise comparisons retain clearance and completion with no
  divergent chatter. Noise is an artificial 5 Hz perturbation, not a fitted sensor model.

There is a real adverse finishing tradeoff. In grid wheel case 102 (gain 1.3, delay
0.6 s, lag 0.15 s), a tiny filtered-speed difference crosses the existing hover-monitor
threshold. Its unchanged fast arrest produces terminal negative jerk -0.4123 → -0.9682
m/s³. Whole-moving peak jerk is unchanged; the stop completes 0.48 s earlier, with
0.146 m more clearance. The monitor was not weakened to hide this result.

Recorded-input replay holds vehicle motion fixed and measures request changes only.
The plants are hypothetical, not identified vehicle predictions. Road comfort and
superiority to the personal baseline still require new driving evidence.

## Verification and rejected alternatives

599 focused runtime tests pass, including an added bookmark regression, continuous
prediction, input boundaries, request publication/history, ownership, recovery and
legacy LongControl coverage. Ruff and diff checks pass. Runtime service AST matches
the frozen experiment; LongControl remains byte-identical to `5a9253be31`. Actual runtime
reproduces all five native diagnostic trajectories and request histories exactly.

The UI builds with `-Werror`; a real HudRenderer harness verifies orange ownership,
brake-only red, stale/invalid messages, and same-instance transitions into a new drive.
The main ownership change was applied through `fullupdate.sh`; a new boot, source
hashes, current version parameter and rebuilt running UI were verified.

Slower global braking, unrestricted rest-demand relief, a Boolean feasibility gate,
and legacy PID demand/release changes were rejected for clearance loss, a known
noise discontinuity, or moving the pulse later. An adequacy-ratio forecast also fails
the existing weak-braking clearance test. These are retained as counterexamples.
Fable cross-review was unavailable because the installed Claude runtime is too old
for that model; independent code, replay, noise and physical-evidence reviews were used.

Frozen source logs, scripts, hashes and results are under
`~/.route_sync/corpus/pump_cycle_20260919/`, especially `actuator_audit/`,
`ease_out_forecast/`, `easeout_regression_review/`, `heldout_refresh/` and `ui/`.
