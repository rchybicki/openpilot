# Human Lane Change Project Log

Last updated: 2026-02-06  
Baseline commit: `f8b815dee8`

## Objective
- Improve FrogPilot human lane change behavior with reproducible testing and documented iteration history.
- Keep one file that tracks current implementation details, what was tested, and what should be changed next.

## Collaboration Workflow (Current)
- Iteration mode: direct commits to the source branch for rapid on-road testing.
- Pull requests are intentionally skipped during this tuning/debug phase.
- Loop:
  1. Implement one focused hypothesis.
  2. Commit to source branch.
  3. User tests on-road.
  4. Record outcome in this file.
  5. Repeat.
- Once behavior stabilizes, we can switch to a PR/hardening workflow.

## Current Status
- [x] Mapped end-to-end lane change and human lane change code path.
- [x] Captured current toggle defaults, gating rules, and state machine behavior.
- [x] Prepared on-road and offline validation checklist.
- [x] Logged active edge-case bug: target-lane lead receiving surrogate override during lane change.
- [x] Implemented surrogate workflow phase gating (`PREP`/`EXEC`/`DONE`) in `radard` (pending on-road validation).
- [ ] Run baseline on-road validation drive for this branch.
- [ ] Collect first route/log set and record observed issues.

## Code Map (Current Implementation)

### 1) Lane change state machine (core behavior)
- `selfdrive/controls/lib/desire_helper.py`
- `DesireHelper.update(...)` controls:
  - Entry to `preLaneChange` when one blinker is activated and speed is above `minimum_lane_change_speed`.
  - Transition to `laneChangeStarting` on either:
    - Correct-direction steering nudge, or
    - Nudgeless conditions (`lane_change_delay`, lane availability, nudgeless enabled).
  - Transition to `laneChangeFinishing` when lane change probability is low and lane line fade-out is complete.
  - Reset to `off` on cancel conditions (blinker off, too slow, timeout, or one-lane-change-completed rule).
- Turn desires below lane-change speed are injected when `use_turn_desires` is enabled.

### 2) Model pipeline publishers (where lane-change state is produced)
- `selfdrive/modeld/modeld.py`
- `frogpilot/classic_modeld/classic_modeld.py`
- `frogpilot/tinygrad_modeld/tinygrad_modeld.py`
- All three compute lane-change probability from model desire outputs, call `DesireHelper.update(...)`, and publish:
  - `modelV2.meta.laneChangeState`
  - `modelV2.meta.laneChangeDirection`
  - `frogpilotModelV2.turnDirection`

### 3) Planner lane-width source used by lane-change gating
- `frogpilot/controls/frogpilot_planner.py`
- `frogpilot/common/frogpilot_utilities.py` (`calculate_lane_width`)
- Planner computes `frogpilotPlan.laneWidthLeft/Right` when lane-width checking is active and speed is above minimum lane change speed.
- Lane width is forced to `0` when road edge appears closer than lane line (utility guard).

### 4) Controls behavior and alerts
- `selfdrive/controls/controlsd.py`
- In `preLaneChange`, controlsd:
  - Raises blindspot block alerts if blindspot is active in target direction.
  - Raises `noLaneAvailable` FrogPilot alert when detected lane width is below configured threshold.
- During active lane change, it sets blinker outputs based on lane-change direction.

### 5) Human-like lane changes (radar surrogate logic)
- `selfdrive/controls/radard.py`
- Enabled only when:
  - `human_lane_changes` toggle is true, and
  - Radar/model pipeline is ready.
- Key behavior:
  - Registers and surrogates opposite-side (and sometimes center) leads during `preLaneChange` and `laneChangeStarting`.
  - Inflates selected lead values (`dRel`, `vLead`) to reduce aggressive slowdown while moving around a lead.
  - Stops surrogation after divider-cross detection or when state leaves active lane-change states.
  - Has lane-existence gate in `preLaneChange` (prevents surrogate when adjacent lane is not plausible).

## Toggle/Parameter Snapshot (As Implemented)

Primary definitions and defaults are in `frogpilot/common/frogpilot_variables.py`.

| Param | Default | Notes |
|---|---:|---|
| `LaneChanges` | `1` | Master lane change enable. |
| `NudgelessLaneChange` | `1` | Enables automatic lane changes without torque nudge when other conditions pass. |
| `LaneChangeTime` | `1.0` | Delay before nudgeless start (seconds). |
| `MinimumLaneChangeSpeed` | `20` mph equivalent | Imported from `LANE_CHANGE_SPEED_MIN`. |
| `LaneDetectionWidth` | `0` | `0` disables lane-width gating (`lane_detection = False`). |
| `OneLaneChange` | `1` | Limits to one lane change per blinker activation. |
| `TurnDesires` | `0` | Enables turn-desire behavior below lane-change speed. |
| `HumanLaneChanges` | `1` | Radar surrogate feature; also gated by longitudinal tuning and radar availability. |

UI sources:
- `frogpilot/ui/qt/offroad/lateral_settings.cc`
- `frogpilot/ui/qt/offroad/longitudinal_settings.cc`
- `frogpilot/ui/qt/offroad/lateral_settings.h`
- `frogpilot/ui/qt/offroad/longitudinal_settings.h`

## Important Gating Rules
- `human_lane_changes` is only active when longitudinal tuning is active and radar is present.
- `LaneChangeTime` and `LaneDetectionWidth` controls are only shown when both `LaneChanges` and `NudgelessLaneChange` are enabled.
- Lane-width checks used by lane-change logic depend on planner lane-width computation, which only runs at or above minimum lane-change speed.
- `LaneDetectionWidth = 0` means no lane-width blocking for nudgeless start.

## Constraints and Risks (Current Code)
- No dedicated automated unit tests for `DesireHelper` lane-change transitions in this tree.
- No dedicated automated test coverage for radar surrogate transitions and edge cases.
- Behavior varies with model pipeline variant (standard/classic/tinygrad), so regression checks should include whichever model is active on device.
- `HumanLaneChanges` can be effectively disabled by runtime prerequisites (radar/longitudinal tuning), which may look like “feature not working” unless prerequisites are verified first.

## Active Issue (Current Focus)

### Issue: Target-lane lead gets surrogate override mid/late lane change
- Repro summary:
  - Ego has a lead in current lane.
  - Ego initiates lane change into a lane that also has a lead.
  - During mid/late merge, the target-lane lead can receive surrogate inflation (`dRel`/`vLead`), causing undesired acceleration toward that lead.
- Why this violates intent:
  - Human lane changes should accelerate early only when moving into an open lane.
  - If target lane is occupied, surrogate should not make ego chase that lead.

### Suspected contributors in current logic
- Surrogate persistence and lead association logic in `selfdrive/controls/radard.py` can continue across changing relative geometry.
- `leadTwo` may be force-surrogated in some same-side/same-track conditions.
- Center/same-side registrations during `preLaneChange` can carry into later phases where the lead context has changed.
- There is no explicit, robust surrogate workflow phase for "entered target lane"; current gating depends on:
  - model lane-change state transitions, and
  - divider-crossing detection from lane lines (which can be unreliable when lane line confidence drops).

### Proposed Fix Options

#### Option 0 (new recommended first step): Explicit surrogate workflow state
- Introduce a small surrogate phase state machine in `radard.py`:
  - `PREP` (`preLaneChange`): allow registration.
  - `EXEC` (`laneChangeStarting`): freeze registration set (no new candidates), optionally disallow center registration.
  - `DONE` (divider crossed OR laneChangeFinishing/off): disable surrogate.
- Use this phase to prevent mid/late candidate drift when geometry/lead association changes.
- Expected outcome:
  - Keeps intended early acceleration behavior.
  - Prevents target-lane lead from being newly surrogated mid-merge.

#### Option A (recommended): Source-lane-only surrogation + target-lane hard block
- Add strict target-lane exclusion before applying surrogate:
  - If lead is in target-lane half-plane (with hysteresis), never surrogate.
  - If lead has crossed into/near target-lane classification at any point during active LC, permanently exempt it for this LC cycle.
- Keep surrogation only for source-lane leads while passing.
- Expected outcome:
  - Preserves desired acceleration in open-lane overtake.
  - Prevents late acceleration toward occupied target lane.

#### Option B: Stop accepting new surrogate candidates after LC start
- Allow surrogate registration only in early `preLaneChange`, then freeze candidate set when entering `laneChangeStarting`.
- Expected outcome:
  - Reduces mid-change candidate flips due to geometry changes.
  - Still may require target-lane hard block for safety.

#### Option C: Remove/limit `leadTwo` force-surrogate behavior
- Disable forced surrogate path for `leadTwo` when `leadOne` surrogate was applied.
- Or keep only when same track ID is confirmed and target-lane exclusion passes.
- Expected outcome:
  - Reduces accidental surrogate propagation to secondary lead.

#### Option D: Surrogate decay and safety cap
- Decay surrogate boost with lane-change progress.
- Clamp/cancel surrogate when TTC/headway to any non-surrogated target-lane lead is below threshold.
- Expected outcome:
  - Additional safety layer, but more tuning complexity.

### Recommended implementation order
1. Implement Option 0 (phase state + freeze registration).
2. Implement Option A (target-lane hard block).
3. Implement Option C (tighten/remove `leadTwo` force path).
4. If needed, add Option B.
5. Add Option D only if residual edge cases remain.

## Quick Runtime Verification (SSH)

### Check relevant params
```bash
ssh comma 'cd /data/openpilot && python3 - <<'"'"'PY'"'"'
from openpilot.common.params import Params
p = Params()
keys = [
  "LaneChanges", "NudgelessLaneChange", "LaneChangeTime", "MinimumLaneChangeSpeed",
  "LaneDetectionWidth", "OneLaneChange", "TurnDesires", "HumanLaneChanges",
  "LongitudinalTune", "LateralTune"
]
for k in keys:
  v = p.get(k)
  print(f"{k}: {v.decode() if v is not None else '<unset>'}")
PY'
```

### Live lane-change/radar trace
```bash
ssh comma 'cd /data/openpilot && python3 - <<'"'"'PY'"'"'
from cereal import messaging
sm = messaging.SubMaster(["carState", "modelV2", "frogpilotPlan", "radarState", "frogpilotRadarState"])
while True:
  sm.update(100)
  if sm.updated["modelV2"]:
    m = sm["modelV2"].meta
    cs = sm["carState"]
    fp = sm["frogpilotPlan"]
    l1 = sm["radarState"].leadOne
    print(
      f"frame={sm.frame} "
      f"blinker=({int(cs.leftBlinker)},{int(cs.rightBlinker)}) "
      f"lcState={int(m.laneChangeState)} lcDir={int(m.laneChangeDirection)} "
      f"laneW=({fp.laneWidthLeft:.2f},{fp.laneWidthRight:.2f}) "
      f"lead1=(status={int(l1.status)}, dRel={l1.dRel:.1f}, yRel={l1.yRel:.2f}, vLead={l1.vLead:.2f})"
    )
PY'
```

If `comma` does not resolve in current network, retry with `commawifi`.

## Baseline Test Matrix (To Run On-Road)

Record each scenario once with `HumanLaneChanges=OFF`, then repeat with `HumanLaneChanges=ON`.

| ID | Scenario | Setup | Expected |
|---|---|---|---|
| LC-01 | Open adjacent lane, no lead | `Nudgeless=ON`, `LaneDetectionWidth>0` | Smooth start after delay; no no-lane alert. |
| LC-02 | Adjacent lane too narrow | `LaneDetectionWidth` high enough to fail | Pre-lane-change + no-lane-available alert; no nudgeless start. |
| LC-03 | Blindspot occupied | Enable lane change toward occupied side | Blindspot blocked alert; no start until clear. |
| LC-04 | Lead ahead, change around lead | Moderate highway speed with forward lead | Compare with/without human lane changes: reduced hesitation/slowdown target. |
| LC-05 | One-lane-change-per-signal | `OneLaneChange=ON` and keep blinker on after one LC | Second immediate lane change should not auto-trigger. |
| LC-06 | Below minimum speed | Speed below configured threshold | Lane change should not start; optional turn desire behavior if enabled. |
| LC-07 | Manual nudge path | `Nudgeless=OFF` | Correct-direction torque nudge required to start. |
| LC-08 | Occupied target lane (this bug) | Lead in source lane + lead in target lane | No surrogate-driven acceleration into target-lane lead. |

## Session Log Template

Copy this block for each drive/test session:

```markdown
### Session YYYY-MM-DD HH:MM (local)
- Vehicle/platform:
- Model pipeline: standard / classic / tinygrad
- Route(s):
- Environment: day/night, weather, traffic level
- Key toggles:
  - LaneChanges:
  - NudgelessLaneChange:
  - LaneChangeTime:
  - MinimumLaneChangeSpeed:
  - LaneDetectionWidth:
  - OneLaneChange:
  - HumanLaneChanges:
  - TurnDesires:

#### Results by Scenario
- LC-01:
- LC-02:
- LC-03:
- LC-04:
- LC-05:
- LC-06:
- LC-07:
- LC-08:

#### Observed Issues
- [ ] Issue title
  - Repro steps:
  - Expected:
  - Actual:
  - Suspected file(s):
  - Route/time markers:

#### Decision / Next Action
- Keep:
- Change:
- Add test:
```

## Working Backlog
- [ ] Add targeted unit tests for `DesireHelper` transition edges.
- [ ] Add replay-style test harness for radar surrogate behavior around lane-change states.
- [ ] Verify behavior parity across standard/classic/tinygrad model pipelines.
- [ ] Define objective success metrics (time-to-initiate, speed dip during pass, abort rate).
- [x] Add explicit surrogate workflow phase (`PREP`/`EXEC`/`DONE`) in `radard`.
- [ ] Implement source-lane-only surrogate filter (target-lane hard exclusion).
- [ ] Audit/limit `leadTwo` force-surrogate logic.
- [ ] Add regression test case for occupied target-lane scenario (LC-08).

## Notes
- Upstream limitation reminder: `docs/LIMITATIONS.md` states blindspot/adjacent checks are driver responsibility.
- This project should keep safety-first behavior and avoid changes that encourage unattended lane changes.
