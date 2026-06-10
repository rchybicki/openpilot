# Stopping Stack Architecture

Scope: longitudinal stop execution (stop intent, stop target, the final low-speed stop tail,
standstill hold, and launch handoff) on the 2022 Hyundai Santa Fe HEV. This documents the system
**as deployed**: the legacy forest controller is active; the V2 stack is merged dark behind kill
switches. Authoritative design contract: the stopping-redesign FINAL_SPEC (base commit
`3be25f5240`); rationale: [redesign_rationale.md](redesign_rationale.md).

Conventions: accel m/s² (negative = braking); jerk m/s³ as positive magnitudes split into
`brake` (deepening) / `release` (toward zero); distances m; speeds m/s; stop-target sentinel
`-1.0` = none, `> 0.0` = explicit. All per-frame limits are dt-parameterized — no `dt_scale`.

## 1. Deployed state and kill switches

Every redesign mechanism flips on exactly one module-level constant (no new Params keys — project
policy). Revert = flip back + `./fullupdate.sh`.

| Constant | File | Deployed value | Meaning of deployed value |
|---|---|---|---|
| `USE_STOPPING_V2` | `selfdrive/controls/lib/longcontrol.py` | `False` | **Legacy forest is the active stopping controller.** V2 facade is constructed nowhere; flip requires the similarity gate (eval.md) to pass first |
| `ARBITER_LEGACY_DROPOUT_HOLDS` | `selfdrive/controls/lib/stop_target_arbiter.py` | `True` | Verbatim-ported legacy dropout-hold predicates are authoritative for `StopDecision`; the consolidated holds run shadow-only, feeding divergence counters |
| `SHOULD_STOP_FALLING_EDGE_HOLD_S` | `selfdrive/controls/lib/stopping_flags.py` | `0.4` | **LIVE.** Planner-level falling-edge hold on `shouldStop` (0.0 = off, restores the raw flag) |
| `SHOULD_STOP_LOOKAHEAD_S` | `selfdrive/controls/lib/stopping_flags.py` | `0.0` | Assert-side early-entry lookahead present but OFF by design (it delays stop entry) |
| `PUBLISH_TRUE_LEAD_DISTANCE` | `selfdrive/controls/lib/stopping_flags.py` | `False` | radard keeps publishing `dRel - increasedStoppedDistance` with the exactly-cancelling compensations; the honesty machinery is dark (flip is commit 10, after the on-device ISD read) |
| `REPORT_SENT_ACCEL` | `opendbc .../hyundai/carcontroller.py` | `True` | **LIVE.** `carOutput.actuatorsOutput.accel` reports the accel actually sent (post-engagement-cap); same CAN bytes as legacy. `TELEMETRY_VERSION = 2` |
| `STOPREQ_LATCH` | `opendbc .../hyundai/carcontroller.py` | `False` | Legacy StopReq gate (`stopping ∧ vEgo < STOP_REQ_MAX_SPEED = 0.01`). Latch + speed release (`STOPREQ_RELEASE_SPEED = 0.10`) are plumbed dark; changes only via on_vehicle_protocols.md |
| `DYNAMIC_SCC14_JERK` | `opendbc .../hyundai/carcontroller.py` | `False` | Legacy static SCC14 jerk 3.0 (pid) / 1.0 (stopping) / 5.0 (lower). Dynamic path plumbed dark; both jerk fields unconditionally clipped to [0, 12.7] before packing |
| `DIST_LPF_TAU_S` | `selfdrive/controls/lib/stopping_params.py` | `0.0` | V2 disturbance estimator runs in the documented kill-switch/bypass state == legacy single-frame trigger semantics (the only value that passes the estimator-equivalence artifact; see eval.md) |

Day-one CAN delta of the entire redesign: **zero** (verified by byte-equality tests in
`opendbc .../hyundai/tests/test_can_bounds_fork.py`).

## 2. Signal flow (as deployed)

```
modeld plan ──► longitudinal_planner ──► longcontrol ──► carcontroller (Hyundai) ──► SCC12/SCC14
                   │                        │
radard (leads) ────┘                        ├── StopTargetArbiter (single instance, in LongControl)
                   ▲                        ├── StoppingController (legacy forest, ACTIVE)
   frogpilotPlan.increasedStoppedDistance   └── [StoppingControllerV2 — dark, not constructed]
```

Phase-0 source-signal fixes, status:

- **shouldStop falling-edge hold (LIVE).** `update_should_stop_falling_edge_hold` (pure function,
  `drive_helpers.py:60`; applied in `longitudinal_planner.py:783` after `output_should_stop` is
  final, below the force-coast override). When the raw flag falls while
  `v_plan[0] < vEgoStopping + 0.15`, True is held up to 0.4 s; immediate release on
  `a_target > 0.2` or `v_plan[0] > vEgoStopping + 0.15`. Strictly additive on the deassert side —
  it cannot create stops. `get_accel_from_plan` itself is untouched (modeld/maneuversd unaffected).
- **Published-dRel honesty (DARK).** radard publishes `dRel_true − ISD` today; long_mpc /
  stop_target_helpers / planner Santa Fe caps carry the exactly-cancelling terms. All sites read
  `stopping_flags.PUBLISH_TRUE_LEAD_DISTANCE` through helpers centralized in
  `stop_target_helpers.py` (`get_published_lead_distance`, `get_stopped_lead_obstacle_offset`,
  `get_effective_lead_distance`). Flag ON re-expresses ISD as the single meaning "rest ISD meters
  farther back" (rest true gap = `lead_stop_distance_target + ISD`; identical between MPC-obstacle
  and explicit-target paths — unit-tested for ISD ∈ {0, 1.5, 3.0}, both flag states).
- **Actuator telemetry (LIVE).** The 4 s post-engagement accel cap is hoisted into
  `CarController.update()` with rising-edge bookkeeping at the top; `new_actuators.accel` reports
  the **sent** value. Non-finite accel is neutralized to 0.0 + `carlog.error` BEFORE any clip
  (np.clip propagates NaN; CANPacker raises on NaN/inf and would kill the 50 Hz SCC12 stream).
- **StopReq (unchanged semantics).** `stopping = longControlState == stopping ∧ vEgo < 0.01`,
  now expressed via named constants with the dark latch machinery alongside. Escalation only per
  [on_vehicle_protocols.md](on_vehicle_protocols.md).
- **SCC14 jerk (unchanged semantics).** Static 3.0/1.0/5.0; the dynamic path (50 Hz-cadence
  command slope + 0.5 m/s³ margin, floored at the legacy static values) is plumbed dark inside the
  `frame % 2 == 0` SCC send block.

## 3. LongControl integration

`LongControl`'s state machine (off / pid / stopping / starting), PID passthrough (on this platform
pid output is literally `a_target` clipped: kpV=kiV=[0.], integrator forcibly zeroed), starting
state, global low-speed slew, and the Santa Fe moving-traffic quirk-cap layer are **unchanged**.

### 3.1 Stop-target arbiter (live, behavior-neutral)

One `StopTargetArbiter` instance lives in `LongControl` (`longcontrol.py:330`) — the single owner
of stop intent + stop target for both the legacy and the V2 path. Its v1 rules are verbatim ports
of the former inline longcontrol predicates (entry/hold/approach/carry envelopes, stopped-lead
synthetic target, target merge `min(planner, synthetic)`, departing-lead and far-stopped-lead
releases, dropout holds). Behavior neutrality was proven by a frame-exact equivalence gate against
a verbatim oracle transcription of the pre-edit `LongControl.update` (7,022 frames, 0 mismatches:
`test_longcontrol_commit_b_equivalence.py`).

`StopDecision` (frozen dataclass) carries: `stop_request_active`, `state_should_stop`,
`target_distance_m` (−1.0 = none), `source` (`StopSource` enum: NONE / PLANNER / EXPLICIT_TARGET /
STOPPED_LEAD / DROPOUT_HOLD / FORCE_COAST_STANDSTILL / STOPPED_LEAD_LATCH), `approach_cap_active`,
`carry_floor_active`, **three independent release booleans** (`departing_lead_release` →
`allow_fast_release`; `departing_lead_ready` → starting-state fast release;
`far_stopped_lead_release` → crawl/settle caps + brake floor gating — they can be simultaneously
true and each has its own consumer), `legacy_forced` (longcontrol-forced holds count as full stop
intent for creep-guard/standstill-relax tiering), `release_reason` (telemetry only), and the
appended `state_dropout_hold` (pins the state machine post-transition, never sets
`stop_request_active` — port of the legacy post-transition holds).

Dropout-hold authority: while `ARBITER_LEGACY_DROPOUT_HOLDS = True` the legacy predicates +
tail-commit latch are authoritative. The consolidated mechanism (0.4 s rolling hold / 0.8 s
explicit-target release hold / 1.4 s no-target standstill hold with the legacy
`a_target > 0.12` / `last_output_accel > −0.08` escapes / **unbounded** `STOPPED_LEAD_LATCH` while
standstill ∧ stopped lead within the close-hold gap ∧ not departing) runs shadow-only and feeds
the retirement counters `legacy_hold_fired` / `single_hold_covered` / `hold_divergence` (emitted
on both telemetry payload paths). Retirement trigger: ≥ 25 dropout events during the post-flip
soak with `hold_divergence == 0`.

ISD compensation has a single producer: `lead_d_rel_eff = get_effective_lead_distance(lead_d_rel,
increased_stopped_distance)` computed once at the top of `LongControl.update`
(`longcontrol.py:505`; passthrough while `PUBLISH_TRUE_LEAD_DISTANCE` is False) and consumed only
by the arbiter call + the named Santa Fe quirk-cap call sites (AST-guarded allowlist in
`test_stop_target_arbiter.py`). `controlsd` passes `sm['frogpilotPlan'].increasedStoppedDistance`.

### 3.2 Controller dispatch and slew ownership

```python
USE_STOPPING_V2 = False  # KILL SWITCH: the only line that changes to enable the new controller
self.stopping_controller = StoppingControllerV2(CP) if USE_STOPPING_V2 else StoppingController()
```

The stopping branch keeps the legacy seam: same `output_accel = min(output_accel, −0.1)` seed,
same handoff-soften application, same expected-accel computation, same kwargs — plus exactly one
trailing kwarg on the V2 branch only: `decision=decision` (the StopDecision longcontrol's own
arbiter computed this frame). The facade never instantiates an arbiter (AST-guarded). The legacy
controller never receives the kwarg.

Slew exemption is conditioned on the same constant (`longcontrol.py:595`):

```python
if USE_STOPPING_V2:
  apply_global_low_speed_slew = self.long_control_state != LongCtrlState.stopping
else:
  apply_global_low_speed_slew = not (self.long_control_state == LongCtrlState.stopping and stop_request_active)
```

Under V2 the tracker's jerk limiter owns every stopping frame; flipping back restores the exact
legacy slew topology. The kept-verbatim Santa Fe quirk caps (close-lead post-cap, stopped-lead
glide cap, far-stopped-lead crawl/settle caps + brake floor) remain downstream actuators that
mutate the command after the facade — the first-drive "no step > 0.10 m/s²" check is therefore
evaluated on the post-cap (sent) signal, and quirk-cap-induced trace divergence is pre-registered
Class A in triage (eval.md). `stopping_guard.py` keeps its `release_lock_active` parameter until
cleanup; under V2 the wire is never consulted (the facade always returns False).

### 3.3 Telemetry

`_log_stopping_shadow`'s emission gate, cadence, and payload are v2-aware (`longcontrol.py:422`):
emit when the debug dict's `version` starts with `v2_` OR a `shadow_profile` key is present
(legacy oracle); v2 cadence = `(phase, source)` change; v2 payload = facade debug dict passthrough
plus ground truth (`v_ego`, `a_ego`, `output_accel`, `lead_status`, `lead_v`, `lead_d_rel`).
The v2 debug-dict contract (`version='v2_tracker_1'`): `phase`, `a_ref`, `disturbance`,
`rollout_m`, `remaining_m`, `release_inhibit_active`, `recovery_i`, `settled_time_s`, `source`,
`triggers=()`. The `shadow_*` oracle keys are retired with the oracle. With the legacy controller
active (deployed state), the legacy shadow payload still flows — extended with the arbiter hold
counters above.

## 4. The V2 controller (merged, dark)

Module chain (all in `selfdrive/controls/lib/`): `stopping_params.py` (frozen `StoppingParams`
dataclass, ~39 named parameters, one definition site each, provenance per parameter —
[parameters.md](parameters.md)) → `stopping_plant.py` → `stopping_trajectory.py` →
`stopping_tracker.py` → `stopping_controller_v2.py` (facade).

### 4.1 Plant model (`stopping_plant.py`)

Identified 7-feature AR(1) actuator model at Δ = 0.1 s:
`a[k+1] = c0 + φ·a[k] + c2·u_d[k] + c3·v[k] + c4·relu(u_d[k] − u_rel) + c5·ls(v) + c6·u_d[k]·ls(v)`
with `ls(v) = clip((1.20 − v)/1.20, 0, 1)`, `u_rel = −0.25`, dead time L. `PlantModel.__init__`
re-discretizes to controller dt: `φ' = φ^(dt/Δ)`, all input coefficients × `(1 − φ')/(1 − φ)`,
dead time `ceil(L/dt)` frames — one code path for 100 Hz runtime and 10/20 Hz replay.

**Pinned physics (hard guard):** with the archived 20260514 coefficients the DC gain
`gain_dc(v) = (c2 + c6·ls(v))/(1 − φ)` collapses and **inverts sign near v ≈ 0.21 m/s**
(gain_dc(0.3) ≈ +0.26, gain_dc(0.2) ≈ −0.03). Below the inversion, a deeper brake command RAISES
the model's steady-state accel. **No code may ever divide by `gain_dc`** — it exists for
documentation/telemetry/eval only (unit-test-pinned + AST guard over selfdrive/). The fitted DC
gain is an in-sample artifact under feedback, not steady-state truth. This finding shaped the
whole design (no model-inverse feedforward anywhere) and survives into Phase 2.

### 4.2 Trajectory reference (`stopping_trajectory.py`, stateless)

Distance-feedback reference with phases TRACK (v > 0.85: `a_ref = clip(−v²/2d_eff,
A_APPROACH_FLOOR(v), −0.05)`) → TERMINAL (0.06 < v ≤ 0.85: clip to the
A_DESIRED_LOWSPEED/A_NEAR_HOLD band, then the end-stop ceiling `A_END_STOP(v)` caps commanded
brake at the calibrated no-jolt magnitude regardless of distance error — undershoot is handled by
rollout recovery extending braking *duration*, never terminal *magnitude*) → SETTLE (v ≤ 0.06:
ramp to `A_HOLD(v)` at `J_SETTLE_RELEASE`) → HOLD (relaxes to `A_HOLD_RELAXED` after 0.8 s).
No-target fallback distance is kinematic `clip(v²/(2·max(0.20, −a_ego)), 0, 3.0)`. The S-curve is
implicit: distance feedback + the asymmetric jerk limiter re-converge automatically after target
jumps, dropouts, and disturbances — no replanning state to corrupt.

### 4.3 Tracker (`stopping_tracker.py`, owns persistent state)

Per frame: expected response `a_exp = plant.predict_next(...)` (legacy interp envelope retained as
sanity clamp) → disturbance estimator `d̂` (first-order LPF on `a_ego − a_exp`; **deployed
`DIST_LPF_TAU_S = 0.0` ⇒ bypass: `d̂ = a_ego − a_exp`, legacy single-frame semantics**) → push
response (deepen to `A_DISTURBANCE_FLOOR(v)`, start release-inhibit timer; G5 deep-command relief
only inside the legacy 0.12 < v < 2.5 gate) → arrest (push below 0.08 m/s with rising v switches
the deepening budget to `J_ARREST_TABLE`, 2.2–4.0 m/s³, bounded by `A_ARREST_MAX(v)` — the
HEV-creep-surge catch) → overbrake release floor → rollout recovery (verbatim G9 integrator) →
delay-release guard (verbatim G2) → normative clamp order: (1) target from ref + deepenings,
(2) TERMINAL end-stop ceiling (quiescent path only; push/arrest paths may deepen past it to their
own bounds — exactly the legacy cap-stack bypass), (3) asymmetric slew (locked release table while
inhibited; end-stop fast release `J_END_STOP_RELEASE_TABLE` while the ceiling binds), (4) final
authority clip `[stop_accel, −0.05]` with the verbatim entry-soften exception (applied by the
facade), (5) non-finite guard: fallback to `last_output_accel` + `nonfinite_fallback` debug flag.

### 4.4 Facade (`stopping_controller_v2.py`)

Legacy seam (`update(output_accel, last_output_accel, should_stop, ...)` returning
`StoppingResult(output_accel, release_lock_active=False)`) extended by exactly one trailing
`decision` kwarg; V2 asserts it non-None. Keeps `.low_speed_rollout_m` / `.phase` attributes for
graceful telemetry reads. Reset discipline matches the legacy controller (reset on state-off or
intent loss; rollout/disturbance state persists across brief PID excursions). A driver brake/regen
tap disengages openpilot entirely (USER_DISABLE) — required V2 behavior is clean reset + clean
re-engage with no stale estimator/recovery/rollout state (tested).

### 4.5 What V1 must never do

- No model-inverse feedforward (`u = x / gain_dc(v)` in any form) — the gain inverts sign.
- No new closed-loop gain outside the stopping state (PID passthrough preserved exactly).
- No command outside `[stop_accel, −0.05]` in stopping authority (entry-soften exception only).
- No non-finite value leaves the facade or reaches the packer (two-layer guard).
- No deepening beyond `A_END_STOP_TABLE(v)` in TERMINAL while the estimator is quiescent (full
  0–0.60 m/s domain); push/arrest paths bound at `A_DISTURBANCE_FLOOR(v)` / `A_ARREST_MAX(v)`.

## 5. Scheduled retirements and deletions

The end state is the forest dissolved, not reorganized. Binding retirement triggers (spec 3.2):
legacy dropout predicates (≥ 25 soak dropout events, zero divergence), entry-soften cap (zero
binding frames in replay + soak), delay-release guard / locked-release table / recovery integrator
(Phase 2, each a paired-stats A/B commit), ISD boundary compensation (device ISD == 0 verified ⇒
delete with replay no-op proof).

**Cleanup-commit deletions are scheduled, NOT executed** — the similarity gate has not passed, the
forest is still the active controller, and every file below is load-bearing today. The cleanup
commit (after the V2 flip + ≥ 2 weeks of good drives) deletes:

- Runtime: `stopping_controller.py`, `stopping_shadow.py`, `stopping_profile_selector.py`
- Tools: `stopping_model.py` (after `stopping_plant.py` + `fit_plant_model.py` provide a superset
  loader; archive `~/.comma/stopping_behavior/models/*.json` into `docs/stopping/archive/` first),
  `fit_stopping_model.py`, `check_harsh_stops_model.py`, `horizon_optimizer.py`,
  `benchmark_controller_variants.py`, `train_profile_selector.py`, `analyze_stopping_shadow.py`
  (rlog-download machinery already lifted into `build_event_store.py`), plus their test files
- Tests: `test_stopping_controller.py` (fixtures already extracted to `stop_scenarios.py`),
  `test_stopping_shadow.py`, shadow-payload assertions in `test_longcontrol_fast_release.py`
- Docs: `docs/stopping_behavior_status.md` (superseded banner applied), the legacy worklog stub
- Longcontrol/guard: `ARBITER_LEGACY_DROPOUT_HOLDS` path (promoting the consolidated holds),
  `USE_STOPPING_V2` constant + legacy slew branch, `release_lock_active` plumbing + guard branch
- Cycle line-items: `run_stopping_cycle.py` `DEFAULT_WORKLOG` flip to `docs/stopping/worklog.md`
  + model-gate stage removal; `check_harsh_stops.py` argparse threshold flags

## 6. Phase 2 (declared successor, post-soak)

After the gate passes, V2 soaks ≥ 2 weeks, and cleanup lands: closed-form jerk-limited S-curve
reference (3-segment BUILD/CRUISE/LAND, bisection on peak decel, saturations not branches) +
first-order lumped input-disturbance observer (frozen under non-disengaging driver overrides,
clamped ±1.0, used ONLY as additive feedforward — never divided by plant gain). Terminal commands
stay clamped to `A_END_STOP_TABLE`; stability demonstrated under ±50% plant K/τ perturbation
before any flip; one mechanism per commit, promoted exclusively on measured paired statistics
("the sim develops, the measurement promotes" — the plant sim is never again a promotion gate).
