# Port equivalence checks

Host tools for the FrogPilot refresh port (`docs/frogpilot_refresh_20261031/`).
Each check runs the same recorded inputs through two source trees, one process
per tree, and compares the outputs exactly. Recorded logs decode with the
schema that recorded them (CLOSURE finding 5): pass the reference tree (the
subtract tip) as `tip tree`; its `cereal/` loads into a private parser, and each
message is copied by field name into the tree under test. Activate the openpilot
venv first. Scripts import their siblings, so run them by path, with
`cwd` = the tree under test and `PYTHONPATH=<tree>:<tree>/frogpilot/third_party`.

## Longitudinal replay (batch 5)

`equiv_replay.py` runs radard, `LongitudinalPlanner` and `LongControl` (called
as controlsd calls it, on the logged plan and chained on the tree's own plan)
on logged inputs, and pickles per-tick outputs.

```sh
RD=~/.route_sync/data/media/0/realdata/00002310--81789e4237
tools/stopping/port_check/run_equiv.sh <port tree> <tip tree> <out>/port $RD "14 15" "16 17 18"
tools/stopping/port_check/run_equiv.sh <tip tree> <tip tree> <out>/tip $RD "14 15" "16 17 18"
python tools/stopping/port_check/equiv_compare.py <out>/port <out>/tip   # exit 0: no difference
```

- `EQUIV_TOGGLES_JSON=<file>`: use that tree's derived toggles
  (`derive_toggles.py`, the tree's `FrogPilotVariables` on a params store)
  instead of the logged ones.
- `EQUIV_WITH_GAS=1`: set `LONGITUDINAL_ACTIVE_WITH_GAS`.
- `equiv_mutate.py <mutation> ...` proves the comparison sees a change in each
  stage (`mpc_floor`, `no_gat`, `radar_delay`, `new_eco_accel`,
  `no_lane_centering`).

## FrogPilot planner and controlsd (batch 6)

With `EQUIV_STORE_JSON=<initData params JSON>` each process seeds a private
Params store (CarParams from the segment, FrogPilotCarParams from the store,
both in the tree's schema; the old FrogPilotCarParams alternative-experience
bits go into CarParams, where the new base's card sets them) and adds two
stages: `fp`, the tree's `FrogPilotPlanner` per logged `frogpilotPlan` tick
(the frogpilotPlan fields the core reads, CEM status, vCruise, SLC and CSC
targets), and `controls`, the tree's own `Controls.update`, `state_control`
and `publish` per logged `carControl` (actuators, longActive / latActive,
desired curvature, lateral controller output). Use each tree's
`derive_toggles.py` output as `EQUIV_TOGGLES_JSON`.

## Card (phase 7)

With `EQUIV_STORE_JSON` the replay also runs `card`: the tree's own
`Car.state_update` per logged `carState`. The logged `carState` and the
car-interface fields of the `frogpilotCarState` from the same step stand in
for CAN. The stage covers the set speed (`update_v_cruise`,
`initialize_v_cruise`), the OPGM resume flag, `FrogPilotCard.update` (always
on lateral from the main switch and its gate, distance long and very long,
force coast, pause lateral and longitudinal, Traffic Mode, experimental-mode
presses) and the live-update handoff's held buttons and AOL suppression. A
group that starts engaged takes the logged `carControl` and set speed, so
its first tick is not an engagement. Each row also carries the logged
values (`log_*`). On route 00002310 our tip matches them, except 6 ticks of
`alwaysOnLateralEnabled` at main-switch edges, where the logged inputs
cannot be aligned exactly. Mutations: `v_cruise_from_ego`,
`aol_cruise_enabled`. The route has no LKAS or very long distance press and
no immediate disable alert, so the stage cannot see those paths; unit tests
cover them (`frogpilot/controls/tests/test_frogpilot_card.py`,
`selfdrive/selfdrived/tests/test_personality_buttons.py`).

`equiv_compare.py` exits nonzero when it finds no group to compare.

## CarController byte check (batch 4)

`carcontroller_replay.py <out.pkl> <rlog>...` feeds logged CAN and carControl
through the tree's Hyundai `CarInterface` in card's order and pickles every
sent frame. `carcontroller_compare.py A.pkl B.pkl` compares two trees frame by
frame and A against the logged `sendcan`. `carcontroller_mutate.py` overrides
one `carcontroller` constant for the sensitivity check.
