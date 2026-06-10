# Stopping Stack Documentation

Documentation home for the longitudinal stopping stack on the 2022 Hyundai Santa Fe HEV
(classic CAN, MANDO_RADAR, openpilot-long, stock SCC ECU disabled). This set replaces
`docs/stopping_behavior_status.md` and `docs/stopping_behavior_worklog.md` (archived).

## Documents

| Document | Contents |
|---|---|
| [architecture.md](architecture.md) | Signal flow, arbiter/trajectory/tracker design, longcontrol integration, kill switches, deployed state, Phase-2 declaration |
| [parameters.md](parameters.md) | Generated parameter reference with per-parameter provenance (source of truth: `selfdrive/controls/lib/stopping_params.py`; `test_stopping_params.py` asserts doc == dataclass) |
| [eval.md](eval.md) | Event store schema, frozen scoring config, statistics + power rules, similarity-gate protocol, current gate status |
| [on_vehicle_protocols.md](on_vehicle_protocols.md) | Staged StopReq escalation, dynamic SCC14 jerk enablement, first-drive checklist, revert discipline |
| [redesign_rationale.md](redesign_rationale.md) | June 2026 review verdict, redesign decisions, scheduled deletions |
| [worklog.md](worklog.md) | Live dated evidence log (append scripts default here) |
| [archive/](archive/) | Append-only: archived worklog (`worklog_2026H1.md`), archived plant-model fits, similarity reports/triage tables (committed with the V2 flip) |

## Deployed state (2026-06-10)

The legacy forest controller (`stopping_controller.py`) is the **active** stopping controller.
The V2 stack is merged but **dark**: the `USE_STOPPING_V2 = False` kill switch in
`selfdrive/controls/lib/longcontrol.py` gates it, and the spec section 7.6 similarity gate has
**not** passed (see [eval.md](eval.md), gate-status section). On-vehicle CAN semantics are
unchanged from the pre-redesign baseline. The full kill-switch table is in
[architecture.md](architecture.md).

Operational tooling commands live in `tools/stopping/README.md`.
