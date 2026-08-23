# The universal stop program (opened 2026-08-23, cycle 34)

**Directive (user):** stop building the tree of ifs. One universal approach; lanes are deleted as an offline
harness proves them redundant; an ML controller is the NEXT program and reuses this harness and plant model.

**Why:** thirty-three cycles patched one stop class per cycle with deepen-only / raise-only lanes around one root
(the e2e reference dies or eases at low speed; the feedforward plant under-delivers). Inventory at opening:
longitudinal_planner.py 129 `SANTA_FE_*` constants, 40 Santa-Fe functions, 52 lane state variables; stopping_service.py
1271 lines / 67 params; longcontrol.py 1595 lines / 16 Santa-Fe cap functions; 12 kill flags. The June-2026 review
condemned exactly this shape; it re-formed in the planner.

## Architecture (design review 2026-08-23, sol xhigh; adopted with the arbitrations marked *)
- The governor OWNS the wire through the existing StoppingService seam in longcontrol (100 Hz takeover, PID
  freeze/reseed, fault fallback, handback already exist). It replaces the service's approach laws (GLIDE/EASE,
  floor-defense, normalization, late-entry). The planner is a deepen-only SAFETY source only (`aTargetTrajectory`
  = the unmodified solver demand; the model/no-lead stop demand as a separate safety input).
- Ownership from `V_OWN` = 4.5 m/s (conditional: StopContext warm before the boundary, authorized lead only,
  planner and a_kin can always deepen, takeover from the actual previous wire, small release hysteresis, harness
  entries at 4.3-4.7 m/s). MPC/e2e keep full authority above.
- NOT blended-only: a trusted slow lead uses the same law in ACC mode.
- "Stateless" means no stop-class episode policy. Delay buffer, jerk-limiter state, target-rate state, wheel-stop
  latch and tracking state remain.

## The law (three fitted parameters A_C, TAU, LAG)
```
q     = max(v - v_lead, 0)
d     = max(gap - (4.0 + ISD) - LAG * q, 0)
z     = sqrt((A_C*TAU)^2 + 2*A_C*d);  q_ref = z - A_C*TAU          # d = TAU*q_ref + q_ref^2/(2 A_C)
v_ref = max(v_lead, 0) + q_ref
a_ff  = -A_C * q_ref / (q_ref + A_C*TAU)                           # -A_C far away, 0 at the anchor
a_gov = clip(a_ff + (v_ref - v)/TAU, -A_MAX, A_UP)
```
Far away it is the sqrt closure curve; near the anchor q_ref -> d/TAU (no singularity, no position-error
denominator: "never chase" by construction). Plus: a lag-aware 3.0-3.1 m BARRIER in the safety helper (a_kin only
protects D_HARD 2.0; safety braking may chase, comfort may not); the clutch physics (gradual RAMP_TO_HOLD, -0.70
secure hold, roll detection, HOLD, trusted RELEASE) stay exactly as measured; the no-lead/model stop demand passes
through the same final limiter and terminal logic.
Known failure cases and their owners: lead decelerating hard (raw MPC/e2e + a_kin deepen; no noisy lead-accel
feedforward first); lead accelerating away (v_ref/A_UP only after `lead_motion_earned`; existing departure dwell);
reversal (`max(v_lead,0)` only in the comfort reference, a_kin uses the real negative speed); grade (conditioned
a_coast residual or the small integral); regen->friction (smooth gain table, fast unwind); ISD changes mid-drive
(rate-limit the target; ignore after the terminal transition).

## Tracking (plant: realized/sent 0.97 @ -0.6 .. 0.78 < -2.5, delay ~0.45 s, lag ~0.5 s)
Static inverse gain first (command u with g(u)*u = a_desired), then a SMALL delayed demand-referenced integral for
grade/model residual. Learn only when: governor is the selected lane, the final limiter is not binding, a_kin and
planner safety not binding, no actuator/controller limit, no gas, terminal modes inactive, v > ~0.8-1.0 m/s.
Freeze/unwind below; clear on HOLD, disengagement, fault fallback, governor disable.

## Behaviours that MUST stay (synthetic tests; the stop corpus cannot prove them)
Lead authority (track-lifetime rejection, early radar acquisition, later model certification); gap provenance
(inward instant, outward delayed, optimistic holds never shallow braking); dropout (inward decay-hold, no release);
identity handover (inward may deepen, outward earns trust first); reversal deepens through the real relative speed;
wheel-stop latch (0.06 m/s dwell, reset above 0.09); clutch hold; departure release conditions; Force Coast (no-target
ramp + standstill hold, never enters the governor); gas override (freeze/clear, reseed from the actual post-override
wire); disengagement resets everything; non-finite/exception fallback (never shallower on the fault frame).

## Harness acceptance gates (frozen before fitting)
A. Route-grouped train/val/test; >= 100 clean driver-free held-out stops; recorded-command replay: |aEgo err| p50
   <= 0.08 / p90 <= 0.20; rest-gap err median within +-0.10, p90 <= 0.35, p99 <= 0.60; rest-time err p90 <= 0.40 s;
   >= 2 s of command history before V_OWN. Fail -> do not score the law.
B. Nominal held-out: 100% completed; no frame < 3.0 m; median rest 4.1-4.6; p10 >= 3.5; p90 <= 5.0; >= 90% in
   [3.5, 5.0]; zero > 5.5; governor decel p99 <= 2.25 / max 2.5; governor owns >= 90% of frames V_OWN -> 0.3; a_kin
   never binds on an ordinary stop. Non-zero ISD: same distribution relative to 4.0+ISD.
C. Terminal feel: the EXISTING scorer (wire_jerk_max <= 0.80, wire_pump <= 0.06, descent_count == 1, felt <= 0.80,
   no relaunch); safety-binding frames exempt but counted.
D. No-chase, from LIVE geometry (0 <= gap-(4.0+ISD) <= 1.5 m, before wheel stop): governor-only max(-a) <= 0.90;
   increase vs the previous 1.5 m band <= 0.20; <= 1 non-safety deepen episode; deeper events attributed to the
   barrier / a_kin / planner safety.
E. Takeover/handback at 100 Hz: ordinary takeover step <= 0.025; safety takeover <= 0.080; RELEASE <= 0.012/frame;
   one entry + one release per stop; no chatter in 4.3-4.7 sweeps; fault frame never shallower; non-Santa-Fe
   frame-identical; gas press -> zero braking through the existing override; gas release restarts from the real wire.
F. Robustness matrix: delay 0.30-0.70, lag 0.30-0.80, gain 0.75-1.05, bias +-0.5, gap noise +-0.10, lead-speed
   bursts +-0.20 x 0.30 s, lead decel 0..-4, lead accel 0..+2, reversal to -0.5, dropout 0-2 s, identity
   replacement in/out, ISD 0-3 m. Plausible cells keep 3.0 m; fault cells keep D_HARD 2.0 and name the binding lane.
The cycle-34 prototype harness (tools/stopping/review/stop_harness.py) is NOT an acceptance harness: radar cadence,
no StopContext/authority, no a_kin/merge/limiter/release/handback, chase window from future information.

## Deletion order (each step gated by the harness, one step per cycle)
1. Valid harness + plant sanity gate (delete nothing).
2. Governor in SHADOW through the service seam; log every arbitration source.
3. Replace APPROACH_GLIDE + PRE_STOP_EASE with the governor; delete `_update_d_rest_eff`, floor-defense shaping,
   `_glide_demand`, `_ease_gates_pass`, `_ease_demand`, relief entry/catch-up, normalization latch+dwell, late-entry
   corridor, redundant anti-hover. Keep wheel-stop, secure hold, post-stop arrest, RELEASE, the sole limiter.
4. Delete the low-speed planner lanes inside V_OWN: rest-close (all arm/spend/epoch/debounce state), roll-in,
   creep extension. Gate: crawler rests pass, none > 5.5, no post-stop re-close, no more floor/safety bindings.
5. Delete the Santa-Fe low-speed longcontrol caps (close-lead cap, glide cap, far-stopped-lead trio, close-gap creep,
   brake-model alignment and stop-target writers on governor-owned frames). Keep generic behaviour for other cars.
6. Delete stop-aim and stop-commit as WHOLE lanes (no 4.5 m/s shims) when every recorded trigger and synthetic
   re-slam/rollback case passes with raw MPC/e2e + governor.
7. Delete the high-speed approach lanes last (decelerating-lead feedforward, lead caution, slowing-lead tables, downhill
   tables, late-approach tables, downhill clip step). Gate: all historical triggers pass from their entry speed; no
   episode reaches V_OWN with unrecoverable debt.
8. Delete obsolete flags, tests, telemetry, comments. One rollout mode until on-road validation completes.

## Status log
- 2026-08-23 cycle 34: program opened; design review adopted; corpus extraction + prototype harness built
  (tools/stopping/review/extract_episodes.py, stop_harness.py); harness hardening delegated (gates G1-G6 of the
  prototype spec); governor law + barrier as pure helpers in SHADOW is this cycle's on-road deliverable.
