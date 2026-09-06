# Lane-change lead selection: A/B replay tooling

Offline comparison of two `radard` lane-change lead strategies on recorded routes
(`~/.route_sync/data/media/0/realdata`):

- **ours**: `selfdrive/controls/radard.py` at HEAD, the lane-change *surrogate* (the source-lane car
  being passed is published 40 m farther and 5 m/s faster than ego so the planner does not slow behind it).
- **upstream**: `radard_upstream_13fa3b292a.py`, the FrogPilot-Testing source (2026-06-17): during
  `laneChangeStarting` the closest radar track cached as *target-lane* replaces the vision-matched lead.

Both are fed the same `liveTracks` / `modelV2` / `carState` / `frogpilotPlan` stream; the raw closest
in-path track is recorded as ground truth.

```sh
python tools/lane_change/lc_census.py 00002000 > census.json          # every laneChangeStarting episode from that route on
python tools/lane_change/radard_ab_replay.py <route> [<route>...] --out replay.json
python tools/lane_change/lc_outcomes.py replay.json                     # per-episode outcome classes
```

Findings (2026-09-06, 1,815 episodes) are recorded in `memory-bank/human_lane_change.md`,
section "2026-09-06 upstream A/B replay".
