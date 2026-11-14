# Decision table for the FrogPilot refresh (2026-10-31)

Revision 2, APPROVED by Radek 2026-10-31 (network rows as recommended: The
Pond off on day one, telemetry "No Thanks"). C06 revised 2026-11-14 after the
first boot crash-looped (Radek: rebase our mapd changes onto the mapd that
FrogPilot uses). Row IDs are
the feature units in `INVENTORY_OURS.md`. Upstream facts are from
`UPSTREAM_DELTA.md`.

Day-one rule: the car drives as it does today on the new base. Every row
that drives the car ports our current behaviour. "Test first" means a later
trial of the upstream replacement, never a day-one swap. Where upstream
rewrote a FrogPilot module we depend on (CEM, CSC, Traffic Mode v2), we carry
our module whole and adapt only its interfaces to the new base.

## A. Port now (day one)

| Group | Rows | Note |
|---|---|---|
| Base, schemas, params | B01, B03 | Reconcile against the new capnp and `params_keys.h` |
| Model and runtime | M01, M02, M03 | tsfdo with our runtime, pinned (see "Upstream features off on day one") |
| Hyundai CAN and safety | H01..H05, H07 | Re-derive on the new `CarState(CP)`, controller signature, and `CP.safetyConfigs` (pandad no longer merges FPCP safety) |
| Stopping service | S00..S34, all | Every flag keeps its switch; S13, S14, S25..S30 are also fold-later (section C) |
| Force coast | F01, F06 | Against upstream's moved force coast (Force Coast removed from `controlsState.forceDecel`; planner masks throttle) |
| Following | F02, F04 | Includes Traffic Mode v2 (`frogpilot_traffic.py`, gone upstream) and its planner and acceleration hooks; upstream fixed Traffic Mode is a later trial |
| Lane-change lead | F03 | Our overtake surrogate on the existing lead |
| CEM | F05 | Our CEM module and predictive triggers, carried whole |
| Cruise and speed limits | C01..C06 | Includes our CSC module carried whole; C06 is our mapd logic rebased onto mapd v2.4.0 (see C06 below) |
| Lateral | L02, L03, L05..L08 | Santa Fe torque tuning, lane centering, desire state, our `lagd` and `torqued` gates, pose validity |
| System | Y01..Y09 | NVMe guard, watchdogs, logger, clock, deleter |
| Update path | U01..U04 | `fullupdate.sh`, live SCC handoff, safe reboot, Tailscale |
| UI | V01, V02, V03 | HUD, ownership color, settings fixes |

C06, revised 2026-11-14. Our mapd logic (Poland way-id speed overrides and
practical speeds) is rebased onto mapd v2.4.0, the version FrogPilot uses.
Our old binary has 15 msgq reader slots and panics on the new base's 20
(first boot of 2026-11-14). The new binary reads only tiles in the new
format, so the device move carries the tiles with it (`PHASE8_PROCEDURE.md`
revision 7). Map downloads stay as today: none by themselves, and a manual
download only of what `MapsSelected` names. A manual download replaces our
tiles with upstream ones (no overrides, no practical speeds); on the car
`MapsSelected` is empty, and box 3.9 stops the move otherwise.

New work the port needs (not in our stack today):

- Route every new restart request through `fullupdate.sh __safe_reboot`
  when ignition is on: `DoSoftReboot` (Soft Reboot) and the Pond's `DoReboot`
  and `DoSoftReboot` endpoints (`437be91e24:frogpilot/system/the_pond/device.py:76,86`).
  Intercept before `manager_cleanup` (`437be91e24:system/manager/manager.py:225-241`);
  the final handler is too late, because cleanup has already stopped card
  and pandad.
- Keep our longitudinal limits. Upstream returns the solver minimum to
  `ACCEL_MIN` and lowers the cruise maximum (`CRUISE_MAX_ACCEL` 2.0 to 1.6,
  `A_CRUISE_MAX_VALS` `[2.0, 1.6, 0.8, 0.6]` to `[1.6, 1.2, 0.8, 0.6]`).
  Keep our `accel_clip[0]` bound and today's maximum and profile values.
- Keep our radard lead selection. Leave out upstream far/radar-only lead
  selection into `leadOne` (no toggle). Upstream adjacent-lead
  substitution into `leadTwo` (gated on `human_lane_changes`) stays off.
- Add a day-one gate that keeps the `speed_limit_reader` process stopped
  while SLC runs. Upstream starts it whenever SLC is on
  (`437be91e24:system/manager/process_config.py:81-82`).
- Carry our FrogPilot longitudinal modules whole (host decision 2026-11-02,
  batch 6): `frogpilot_planner`, `frogpilot_acceleration` (our profile
  values), `frogpilot_following`, `frogpilot_traffic`, `frogpilot_vcruise`,
  `conditional_experimental_mode`, `curve_speed_controller` and
  `speed_limit_controller`. Only their interfaces follow the new base (no
  memory Params, press counters and UI events, the new events, weather and
  speed-limit filler). Upstream's versions change day-one driving (ECO
  table, the `CRUISE_MIN_ACCEL` floor that is our MPC lower bound, no
  Traffic Mode v2, the SLC confirmation step suppression) and rest on its
  rewritten CSC; they are later trials. Evidence: the route 00002310
  equivalence replay (`tools/stopping/port_check`, 0 differences in the
  FrogPilot planner and controlsd stages) and the sim verdict
  `20261102-0055_fp1031_port_fc8b326158` (rows identical to the subtract tip).

## B. Test first (later trials of upstream replacements, one at a time)

| Upstream replacement | For our rows | Trial checks |
|---|---|---|
| Rewritten CEM | F05 | CEM status per case vs ours (for example 54 km/h with SLC 30 km/h: ours SPEED, upstream OFF) |
| Rewritten CSC with learners | C04 | Curve speed per case vs ours |
| Fixed Traffic Mode | F02 | Following gap and jerk vs v2 |
| Adjacent-lead substitution into `leadTwo` | F03 | `leadOne` and `leadTwo` assertions |
| Upstream `lagd` / `torqued` gates | L06, L07 | Sample acceptance (for example lat 0.5, long 1.5: ours rejects, upstream accepts) |
| Upstream mapd binary and tiles (without our overrides and practical speeds) | C06 | SLC limits on the same routes |
| New cruise maximum | longitudinal limits | Launch and cruise acceleration |
| Custom driving personalities | F02 | Following gap and jerk |

Not ported, no driving effect on the Santa Fe: L01 (upstream deleted
`latcontrol_curvature.py`; the Santa Fe uses the torque controller), L04
(device offset is 0), M04 (USB GPU), V04 and V05 (personality and GM/Honda
menus).

Upstream features off on day one:

| Feature | Upstream default | Day one | Reason |
|---|---|---|---|
| Force Stop at lights/signs | `ForceStops` 0 | off | It owns hold in the planner; our service owns hold |
| Model Manager selection, Randomizer, auto downloads | `AutomaticallyDownloadModels` 1, `ModelRandomizer` 0 | selection pinned to tsfdo, downloads off | Fallback can pick "Dark Souls" silently |
| Speed Limit Sign Reader | runs when SLC is on | off by the new gate (section A) | New camera process and msgq reader; 15 reader slots |
| Far/radar-only lead into `leadOne` | no toggle | left out | Changes the lead our stops use |
| Speed Limit Filler | `SpeedLimitFiller` default 1 | off (the car stores 0) | It replaces our map limits with sign or dashboard readings and stores them per way id |
| Camera-first SLC priority | `SLCPriority1` default `Camera` (then `Map Data`, `Dashboard`) | `Map Data`, `Dashboard`, `None` (stored on the car) | Our practical and override limits come from the map; camera first would put sign readings before them |
| mapd conditional speed limits | `conditional_speed_limit_control_enabled` false | off | It replaces the map limit with a time-conditional one; not judged with our tiles |
| Automatic nearby map downloads | on: every 2-degree cell within 100 km with no tile newer than 28 days | removed in code | Upstream tiles overwrite ours at the same paths, without overrides or practical speeds |

## C. Fold later (switch disposition only)

S13, S14, S25..S30 port with their implementations on day one (section A).
Each passed its week judge (KEEP). After the new-base trial passes, delete
the legacy path and the switch. S31 LAUNCH_FOLLOW keeps its switch until N2
is judged.

## D. Drop

H06 `DYNAMIC_SCC14_JERK`, M05 old model assets, D01 stop line, D02
identification hook, D03 close-gap creep, D04 whole-approach research, D05
old shadow oracle, D06 should-stop lookahead, D07 orphan params. Removed
on the regrouped branch before the port (plan phase 5).

## E. Tools and docs

B02, T01..T05. T01 sim tooling is prepared in batch 2; its first verdict
runs after the stopping closure is ported (plan phase 6). T03 excludes the
uncommitted route_sync work.

## F. Network and privacy (Radek's call)

| Path | Upstream default | Recommendation |
|---|---|---|
| FrogPilot API registration | automatic, no gate | keep (existed on the old base) |
| FrogPilot telemetry, stats, sign captures | off until the consent prompt | answer "No Thanks"; stats still POSTs empty |
| The Pond local web service | always runs, pairing off (open on the LAN) | off on day one, or turn pairing on |
| comma / Konik uploads | on | keep your current device settings |
| Weather offsets | off | off |

## G. Answers to the open questions (2026-10-31)

1. LAUNCH_FOLLOW N2 is not judged on its own. drive_report LAUNCH rules
   PASS on every complete drive since 2026-10-18 (37 reports, none REVERT;
   `~/.route_sync/reports/drives/`). No effect on the port: S31 keeps its
   switch and is not fold-later. A rolling-abort census can judge N2 later.
2. Lane centering ports as is (L03): `GAIN` 0.60 (`63dbb4cf98`), right-line
   gate 0.3, `MIN_LANE_PROB` 0.6. Last recorded device values (2026-10-04):
   `LaneCentering` 1, `LaneCenteringE2EAuthority` 0.0, `LaneCenterOffset`
   0.0. The device was unreachable on 2026-10-31; phase 8 reads the live
   values into the settings backup. The post-gain (0.60) drive result was
   never measured; it does not block the port.
3. NVMe fallback: keep scope A (Radek, 2026-10-31).
4. Uncommitted route_sync retention work: commit and carry (Radek). Committed
   `32378eb858`; ports as a tools item (T03). It prunes by default; pass
   `--keep-days 3650 --max-cache-gib 4000` to keep the corpus.
5. Device move: switch the device branch to `!my-fp-1031` (Radek).
   `!my-fp-new` stays untouched for rollback.
