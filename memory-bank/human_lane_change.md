# Human Lane Change Project Log

Last updated: 2026-07-17
Pre-change baseline commit: `428d5f3df6`

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
- [x] Implemented surrogate workflow phase gating (`PREP`/`EXEC`/`DONE`) in `radard`.
- [x] Added sticky target-lane release memory that survives radar track churn.
- [x] Added focused radar-surrogate unit tests.
- [x] Diagnosed bookmarked simultaneous ego/lead lane change on route `00001ef9--e111293d47`.
- [x] Added divider-relative release for a registered lead that changes into the target lane alongside ego.
- [x] Unblocked pytest bootstrap on desktop/macOS for targeted tests.
- [x] Collected multiple bookmarked route/log sets and used them as primary regression evidence.
- [ ] Validate the divider-relative release on-road after the 2026-07-17 deployment.

## Historical Pytest Bring-Up Notes (2026-02-06)
- Goal: make `pytest` runnable enough for fast local checks while iterating on lane-change logic.
- Completed:
  - Built missing native modules in this worktree (`common/params_pyx.so`, `msgq_repo/msgq/ipc_pyx.so`, `opendbc_python`, `common/transformations/transformations.so`, `selfdrive/pandad/pandad_api_impl.so`).
  - Installed missing pytest/runtime deps seen during bring-up (`pytest-xdist`, `pytest-cpp`, `hypothesis`, `parameterized`, `natsort`, `PyJWT`, `zstandard`, `crcmod`, `influxdb-client`).
  - Fixed test-environment path assumptions:
    - `openpilot/common/prefix.py`: fallback to temp dir when `/dev/shm` is missing.
    - `openpilot/frogpilot/common/frogpilot_variables.py`: use per-prefix writable params roots on PC instead of device-only `/cache` and `/dev/shm`.
    - `conftest.py`: lazy/conditional `manager` import so teardown does not hard-fail if optional FrogPilot runtime deps are absent.
- Verification:
  - `pytest -q -n0 common/tests/test_params.py --maxfail=1` passes (`11 passed`).
- Remaining known issue:
  - Full suite still stops at `selfdrive/car/tests/test_models.py` with `AttributeError: HONDA.CIVIC` from `selfdrive/car/tests/routes.py` (branch data/test mismatch, separate from pytest bootstrap).

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
  - Registers source-lane candidates during `preLaneChange` and freezes normal registration in `laneChangeStarting`.
  - Publishes synthetic lead geometry to reduce aggressive slowdown while moving around a source-lane lead.
  - Stops surrogation after ego-divider crossing, target-lane lead release, lead-divider crossing, or when state leaves active lane-change states.
  - Has lane-existence gate in `preLaneChange` (prevents surrogate when adjacent lane is not plausible).

### 6) What `radarD` actually returns

`radarD` does not return a separate "lane-change override" object. Its normal published `radarState.leadOne` and `leadTwo` are the values consumed directly by longitudinal MPC. The pipeline is:

1. `liveTracks` supplies raw radar points.
2. `get_lead(...)` vision-matches a point and creates a lead dictionary.
3. `_update_lane_change_surrogates(...)` updates candidate/release lifecycle state.
4. `_apply_overtake_surrogate(...)` may copy and rewrite that lead dictionary.
5. The rewritten dictionary is assigned to `radarState.leadOne` or `leadTwo` and published.
6. `LongitudinalMpc.update(...)` consumes those published values as real obstacle geometry.

When the surrogate is applied, the published values are changed as follows:

| Field | Published surrogate value |
|---|---|
| `dRel` | Raw distance plus `40 m` |
| `vLead`, `vLeadK` | At least ego speed plus `5 m/s` |
| `vRel` | Recomputed from the synthetic lead speed, normally `+5 m/s` |
| `fcw` | Forced false |
| `modelProb` | Floored at `0.01` |
| `yRel`, `radarTrackId` | Left unchanged |

Important consequences:

- There is currently no schema field that says a published lead is synthetic. In logs, the characteristic `dRel + 40 m` and `vRel = +5 m/s` pattern is the practical indicator.
- The planner cannot recover the raw lead from `radarState`; it trusts the synthetic distance and speed.
- A wrong surrogate classification is therefore safety-significant: it can hide a closing lead and request acceleration.
- `frogpilotRadarState.leadLeft/leadRight` is adjacent-lead UI/support data. It is not a raw backup automatically used by longitudinal MPC.
- When `leadOne` and `leadTwo` resolve to the same radar track, the secondary lead can be surrogated and then hidden. It is not a guaranteed independent target-lane safety lead.

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
- There is still no dedicated automated unit-test suite for all `DesireHelper` lane-change transitions.
- Focused radar-surrogate tests cover relative-lateral release, transient blips, target-lane release memory, track churn, and simultaneous ego/lead divider crossing. They are not a full process replay.
- Behavior varies with model pipeline variant (standard/classic/tinygrad), so regression checks should include whichever model is active on device.
- `HumanLaneChanges` can be effectively disabled by runtime prerequisites (radar/longitudinal tuning), which may look like “feature not working” unless prerequisites are verified first.
- Lane-line confidence and identity can change during a maneuver, especially around tunnels, merges, and worn markings. Release decisions must be confirmed across frames and remain sticky once accepted.

## Active Issue (Current Focus)

### 2026-07-17 bookmarked failure: source lead changes lanes with ego

- Route: `00001ef9--e111293d47`, bookmark in segment `--22` at `logMonoTime=1651601368858`.
- Device commit: `428d5f3df6`.
- Maneuver: ego starts a left lane change around a source-lane lead; that lead also changes left into the destination lane at nearly the same time.
- The surrogate initially registers the correct source-lane track (`343318`). The failure is later lifecycle classification, not initial track selection.
- Because ego and lead move left together, radar-relative `yRel` stays roughly `-0.3` to `-0.8 m`. The relative-side release test never sees the lead on the target side.
- The fixed-distance ego-divider guard also never fires because the divider loses confidence/changes shape during the tunnel and merge.
- Published lead changes from roughly `38 m`, `-1.5 m/s` closing to `78 m`, `+5 m/s` separating. The planner requests up to approximately `+0.40 m/s²`.
- At driver brake takeover, the raw gap is approximately `27 m` and closing at `5.8 m/s`.

This is a distinct failure from track-ID churn: the radar track remains stable, but the physical lead changes lanes.

### Implemented lifecycle protections

1. `PREP` / `EXEC` / `DONE` phase gating prevents ordinary new candidate registration after lane-change execution begins.
2. Relative-`yRel` release preserves a surrogate through a transient lateral blip but releases it after a clear target-side move.
3. Released target-lane track IDs and short-lived distance/speed signatures prevent re-entry after radar track churn.
4. Divider-relative lead release now compares the lead position with the source/target divider at the lead's own `dRel`:
   - radar `yRel` is converted to model lateral coordinates with `lead_model_y = -yRel`;
   - the divider line is interpolated at `lead.dRel`;
   - the comparison is direction-normalized so a positive margin always means target side;
   - the divider must have at least `0.3` model probability;
   - the lead must be at least `0.2 m` into the target side for three consecutive frames;
   - once confirmed, existing sticky release memory prevents re-surrogation for the maneuver.

The bookmarked route contains a three-frame confident target-side run approximately `1.05 s` before driver brake takeover. The old ego-divider guard remains false on the same data.

### Remaining hardening options

#### Source-confirmed registration
- Require positive source-side divider evidence before registering a candidate, rather than accepting every center candidate when geometry is unavailable.
- This is safer but may disable the feature more often in low-confidence lane markings, so it should be route-replayed before enabling.

#### `leadTwo` audit
- Remove or further restrict forced `leadTwo` surrogation. Same-track secondary hypotheses are not independent safety coverage.

#### Synthetic-lead telemetry
- Add an explicit diagnostic field or companion message identifying which leads were surrogated and preserving raw distance/speed for logs.
- Avoid changing the core `RadarState.LeadData` schema casually; it is a broad interface.

#### Ambiguity fail-safe
- When target-lane classification is unavailable and the raw lead becomes urgently closing, reduce or cancel the synthetic boost rather than hiding the only closing obstacle.
- This needs replay against normal source-lane overtakes so it does not cause a late brake toward a lead that ego has safely left behind.

## Quick Runtime Verification (SSH)

### Check relevant params
```bash
ssh comma 'cd /data/openpilot && /usr/local/venv/bin/python3 - <<'"'"'PY'"'"'
from openpilot.common.params import Params
p = Params()
keys = [
  "LaneChanges", "NudgelessLaneChange", "LaneChangeTime", "MinimumLaneChangeSpeed",
  "LaneDetectionWidth", "OneLaneChange", "TurnDesires", "HumanLaneChanges",
  "LongitudinalTune", "LateralTune"
]
for k in keys:
  print(f"{k}: {p.get(k)!r}")
PY'
```

### Live lane-change/radar trace
```bash
ssh comma 'cd /data/openpilot && /usr/local/venv/bin/python3 - <<'"'"'PY'"'"'
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
| LC-09 | Source lead changes lanes with ego | Start around a source-lane lead that also enters the target lane | Release surrogate as soon as lead-divider crossing is confirmed; retain real closing lead. |

## Bookmarked Validation Log

### Session 2026-07-17 (route review)
- Route: `00001ef9--e111293d47`, segments `--21` and `--22`.
- Bookmark: `userBookmark logMonoTime=1651601368858`.
- Device commit: `428d5f3df6`.
- Scenario: LC-09, left lane change in a tunnel/merge while source lead also changes left.
- Result on baseline: failed; the synthetic lead stayed active until lane-change state returned `off`.
- Proposed-code offline result: divider-relative detector confirms target entry on three consecutive confident frames while the old ego-divider detector remains false.
- Focused unit coverage: `test_registered_surrogate_releases_when_lead_crosses_target_divider_with_ego`.
- Next action: deploy and repeat on-road with a new bookmark if behavior is still incorrect.

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
- [x] Add sticky target-lane release across track churn.
- [x] Release a registered source lead when it crosses the target divider alongside ego.
- [ ] Require positive source-lane divider evidence at candidate registration.
- [ ] Audit/limit `leadTwo` force-surrogate logic.
- [x] Add focused regression tests for target-side release and simultaneous ego/lead divider crossing.
- [ ] Add a full occupied-target-lane process replay for LC-08/LC-09.

## Notes
- Upstream limitation reminder: `docs/LIMITATIONS.md` states blindspot/adjacent checks are driver responsibility.
- This project should keep safety-first behavior and avoid changes that encourage unattended lane changes.
