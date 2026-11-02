# Stopping program retrospective -- 2026-09-02 (after cycle 39b)

> Historical report. The cycle-43 review in [2026-09-04](retrospective_2026-09-04.md) supersedes its
> "~2x smoother" inference, whole-approach naming, and mandatory scripted-drive sequence. The old jerk
> calculation accepted partial windows; the manual/engaged samples were not matched. Keep these numbers
> as historical output, not evidence of passenger comfort or a causal advantage.

Asked for by the user: "take a step back and see if we are going the right way or if we can improve the process".

## 1. Where the evidence says we are

| Question | Evidence (routes 00002xxx, 653 segments scanned) | Verdict |
|---|---|---|
| Rest gap | last 5 routes: median 4.08-4.30 m, zero long, one 3.3 | SOLVED. Stop spending on it. |
| Pump class (ease->grab) | reversal census: engaged stops 55/55 single-descent (manual 131/132) | GONE (the governor did its job). |
| Terminal felt (the acceptance metric) | engaged p50 1.06 / p90 2.05, 15% <= 0.8; **driver's own manual stops p50 2.04 / p90 4.40, 10% <= 0.8** | openpilot already ~2x smoother than the driver BY THIS METRIC. |
| Approach jerk (whole approach, v<=9) | engaged p50 1.80 vs manual 3.39; peaks spread over 1-9 m/s, 54/68 above 3 m/s (planner domain) | same: engaged ~2x smoother. No handoff spike at 2.5 (bimodal, checked by histogram). |
| Body nod (pitch rate) | engaged 0.019 vs manual 0.032 rad/s (approach); 0.007 vs 0.010 (terminal) | same. |
| Wheel-stop decel (the nod driver) | engaged -0.37 vs manual -0.45 m/s^2 | same. |

Every physical smoothness metric we have says the engaged stops are already better than this driver's own
stops -- yet the driver reports "we rarely hit that perfectly smooth spot". Two readings, both probably true:
(a) the bar is genuinely above-human (the user said so), and what remains is small (the wheel-stop nod at
-0.37 that a human unloads to ~-0.1; firm mid-approach decel levels; anticipation/timing);
(b) our metrics do not measure what a passenger perceives. Note the acceptance metric `felt` is scored ONLY
in the terminal window (v < 0.45 m/s = the last ~30 cm). We have been optimising the last 30 cm of every stop.

## 2. Process assessment (honest)

- 39 cycles; the last 9 days: 37 commits, ~31 review rounds on stopping. Complexity is NOT falling: planner
  SANTA_FE constants 129 -> 132, state vars up (yesterday's roll-in guard: +2 constants, +2 state vars, 4
  commits, 3 review rounds -- on a lane that is a DELETION candidate). The deletion program has deleted nothing.
- The loop reacts to the newest harsh stop with a guard. That is the "tree of ifs" the user warned about,
  just with better reviews. The reviews found real things each time, but on the wrong artefact.
- One iteration per drive. The identification drive (needed for the offline plant model/harness) has been
  pending since cycle 34, so every design is validated on-road only, one drive at a time.
- Ground truth is sparse: 1-2 bookmarks per 20 stops; no per-stop rating. Nothing calibrates any metric
  against perception. Cycle-39's felt-0.99 "hot entry" attribution was wrong for exactly this reason (the
  metric never saw the entry); caught within the cycle, but it shows the metric misleads even me.
- My own errors this session: the far-lead misattribution (corrected by frame decode before any edit) and
  the "2.5 m/s handoff spike" claim (retracted by histogram). Lesson: decode before attributing; histogram
  before claiming a median.

## 3. Direction verdict

The governor direction is RIGHT: it removed the pump class and rests are done, with a single law. What is
wrong is the process around it: the objective is not measured, the loop is slow, and misbehaving legacy
lanes get patched instead of deleted.

## 4. Changes (ranked by leverage)

1. **Ground truth first (needs the user).** One or two drives where EVERY stop gets a verdict: press the
   bookmark once for "not perfectly smooth" (every stop that is not perfect), twice for "harsh". With ~30
   rated stops I can find which measurable feature separates good from bad (nod, decel level, onset, timing,
   low-frequency surge) and replace `felt` with a metric that tracks perception. One rated drive is worth
   more than ten guessed cycles.
2. **Whole-stop metrics in the index now (mine, done this cycle):** the human-baseline tool
   (`tools/stopping/review/human_baseline.py`) scores engaged AND manual stops with the same whole-stop
   metrics, so every cycle sees the driver baseline; the index stops reporting the last 30 cm as the score.
3. **Delete, don't patch.** A deletion-candidate lane that misbehaves under the governor gets deleted or
   absorbed, never guarded. The structural version of yesterday's fix is one principle: inside governor
   ownership the planner's COMFORT output is not a min-input; only ATTRIBUTED safety demands (a_kin,
   barrier, FCW/AEB, lead MPC as safety) may deepen. That one step covers the roll-in floor, the head band,
   and the no-lead class (sol's channel split) -- three designs become one. Pull it forward as the next step.
4. **The identification drive (needs the user, ~30 min, protocol v1 in the program doc).** Unblocks the
   offline plant model -> offline law ranking -> many iterations per day instead of one per drive. Also the
   prerequisite for the ML controller (which would otherwise learn the wrong objective, see 1).
5. **Review budget:** adversarial reviews stay for structural steps; two rounds max (existing rule, now
   enforced); no reviews on guards for lanes slated for deletion (there should be no such guards).
6. **ML controller:** stays after 1 and 4. Without a perception-calibrated objective and a plant model it
   optimises the wrong thing on-road.
