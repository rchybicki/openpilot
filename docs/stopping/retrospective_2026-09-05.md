# Stopping review -- 2026-09-05

## Cycle 46: deployment objective and signal consistency

The user authorized continued work until there is a deployable change, then deployment. The next live
prerequisite is the shared invalid-lead boundary already identified in cycle 42. This is a braking fault
repair, not evidence of smoother nominal stops. Whole-approach governor activation remains gated OFF.
Cycle-46 design review: `20260905-074416-exec`, sol xhigh, completed exit 0: approve with required corrections. Availability
probe `20260905-074059-exec` completed successfully. Both device SSH profiles were unreachable at the first
check; device access is being handled while local work continues.

The current code still reproduces the faults: at v=1.2 m/s, dRel=NaN makes roll-in emit NaN; dRel=Inf raises
-1.5 to zero; vLead=NaN with dRel=10 m raises -1.5 to -0.12642. Far-lead confirmation also propagates NaN.
The repair must validate shared inputs before solvers and comfort/legacy consumers, retain finite braking
through a fault, use existing fault reporting, and prove recovery/override/other-car behavior. No per-lane
NaN patches or new comfort law are proposed.

### The cycle-45 acceleration/speed mismatch is explained

The Hyundai wheel-speed path calls `CarStateBase.parse_wheel_speeds`, then `update_speed_kf`
(`opendbc_repo/opendbc/car/interfaces.py`). Its `KF1D.update` is implemented in
`opendbc_repo/opendbc/car/common/simple_kalman.py`. For an uninterrupted filter sequence without a large-jump
reset, the exact source identity is:

```
v_next = v_prev + DT_CTRL * a_prev + (K_v / K_a) * (a_next - a_prev)
```

The fixed control period is 0.01 s. Source-derived gains are K_v=0.1740603891 and K_a=1.6592563983, so the
correction coefficient is 0.1049026475 s. It is derived from the filter; nothing is fitted per stop. The
correction represents the shared measurement update and is missing from simple acceleration integration.

Replaying this identity with recorded acceleration over all nine cycle-45 admitted episodes gives maximum
speed error <0.000002 m/s and absolute distance error <0.000021 m. Using message timestamp intervals instead
of the filter's fixed tick gives larger distance errors, up to 0.082 m, because message delivery times differ
from filter ticks. The original simple fixed-tick integral has 0.864-1.775 m distance errors.

This is a **signal-consistency result using recorded acceleration**, not plant validation. It resolves the
specific measurement-relationship uncertainty from cycle 45. It does not erase the full-model failure,
validate the reconstructed lead trajectory, or justify a comfort-law activation. The next plant experiment
must include a coherent wheel observation/state update and test full-episode speed and distance without
observed-state resets.

Reproduction: `~/.route_sync/corpus/cycle46/signal_consistency.py` and `signal_consistency.json`. The script
checks the identity against the real KF1D implementation before applying it to all nine saved episodes.

### Shared input boundary implementation and evidence

The design review rejected a blanket LongControl return for every field fault: a valid primary lead must
still be able to demand stronger braking when only a secondary/service input is invalid. The implementation
validates consumed fields once at each entry point with the same conversion-safe finite predicate.

Planner faults skip both solvers and all comfort consumers, hold the previous finite nonpositive target,
retain stop intent, suppress throttle and publish an invalid plan/trajectory. All lead trust is reset.
The first valid frame resets the existing lane/filter states and removes the previous-acceleration solver
constraint. It cannot raise either target above the held demand on that first frame.

LongControl skips all consumers for a bad primary wire input. For a secondary/service input fault it runs
the valid primary legacy chain, with service ownership cleared, and takes the lower of that demand and the
previous nonpositive wire. The service and its trust cannot run on fault/first-recovery frames. PID state
is reseeded from the final output. Inactive/brake overrides return zero; the existing gas wrapper keeps its
authority. No drive-scoped service-failure latch is set for an input fault.

Main-agent correction to R1: planner-only fields such as leadOne.vRel and leadTwo.modelProb are not passed
into LongControl. Merely invalidating the plan would leave the service running until soft-disable. One
trailing `plan_valid` argument now carries the existing message-valid bit from controlsd. An invalid plan
uses the same service fault path, retaining a valid primary lead's deeper braking. No schema/Params change.

The 75 frozen rlogs contain 89,054 radarState messages (56,265 on 2072; 32,789 on 2073). Scanning all eleven
LeadData float fields on both active and absent leads found zero nonfinite values. This is an injected-fault
repair, not the observed cause of the user's normal stopping complaint. Census artifacts are
`~/.route_sync/corpus/cycle46/lead_input_census.py`, `.json` and `.log`.

Verification: combined stopping, planner, boundary and controlsd battery 973 passed, 19 skipped.
Changed Python modules and tests pass ruff; controlsd has one pre-existing 162-character import on line 16. The baseline has 1,280 real planner update/publish records (only native solvers stubbed) and
34,730 real LongControl frames across all nine accepted episodes, for Santa Fe and another fingerprint.
Exact after-output comparison passed: both files have SHA256
`ee7390bdfb0be7c6b7c1aecaa4922f30abf3f6dc39892154bf983cf359ff1dd5`. The after script loads each NPZ
array once instead of decompressing it per field/frame; all sampled values and computation are unchanged.
The original before script is retained. Final adversarial review `20260905-080152-exec` (sol xhigh, exit 0) approved with no runtime blockers.
It independently reproduced 973 passed/19 skipped and exact output equality. The requested new test file
is included in the commit. Both review rounds are complete; main agent signs off the unchanged runtime diff. Native solver execution itself is
not tested by the solver-substitution fixture; the test proves invalid input never reaches its call.

Device reconnected and returned verified HEAD `18ceaea8`. Deployment preparation can proceed after review.

The pre-deployment health check verified running manager/UI services and the expected flags. The device
then reported off-road; controlsd/plannerd are correctly stopped in that state. Its GitHub deploy key still
fails, so deployment uses the documented bundle fallback and restores the SSH origin afterward. Deployment
and post-restart verification will be recorded separately; approval is not deployment proof.

### Deployment verified; GitHub access repair

Code `e1c2b3235398dba0f6fbee0b8d319fea19edc3dd` was pushed and installed through `fullupdate.sh` using the
bundle fallback. Reboot was verified by a changed boot ID. The installed commit matches, manager/UI and
normal off-road services run, and deployed controlsd/planner imports pass. The device was off-road for this
check, so there are zero new driving samples; no on-road comfort validation is claimed. Original GitHub
origin restored. Flags remain LIVE / governor / attributed shadow / whole-approach off.

The user's follow-up correctly challenged the repeated key failures. GitHub retained the old read-only
public key, but the matching private key under `~/.ssh` was absent. `/home` is an overlay whose upper layer
is `/rwtmp` tmpfs; it resets at reboot. The native SSH config selects `/data/ssh/id_ed25519` on persistent
ext4. Generated the replacement there (directory 700, key 600), registered it read-only, and verified normal
`git ls-remote` and `git fetch` without overrides or a bundle. Removed the old registration after success.
New registration ID 162352231; fingerprint `SHA256:M8Hcb/Tx4pYiQR+PAl9fHtbzogIFkjfAccgj0sTqGks`.
The private key remains only on the device. AGENTS.md now specifies persistent storage. Reboot persistence
verification is the remaining SSH check before returning to the stopping model.
