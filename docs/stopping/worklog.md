# Stopping Worklog

Dated evidence log for the stopping stack (post-redesign home). Entries are appended by
`tools/stopping/append_analysis_report.py`, `append_cycle_report.py`, and `append_sync_report.py`
(this file is their default `--worklog`), or by hand.

- History through 2026 H1 (pre-redesign forest era): `docs/stopping/archive/worklog_2026H1.md`
- Methodology and gate protocol: `docs/stopping/eval.md`
- Entry format: `### YYYY-MM-DD: <title>` followed by bullet lines (commands, artifact paths,
  before/after metrics, keep/reject decision).

### 2026-06-10: Redesign docs scaffold

- New documentation home created (`docs/stopping/`); legacy worklog archived to `docs/stopping/archive/worklog_2026H1.md`.
- Similarity gate (spec 7.6) ran on the dual-plant deck and did NOT pass; `USE_STOPPING_V2` remains `False` (legacy forest active). Details: `docs/stopping/eval.md`, gate-status section.

### 2026-06-10: Driveway review, hold-acquisition soften, rollout plan

- Driveway/hill-hold blind-window review (route `00001702--dcdc5c3eea--0`): felt hold-acquisition slam traced to the deep ramp (-0.5..-0.78 -> -1.05).
- Hold-acquisition soften shipped (commit 9b4a07de78): parameters.md row 40 (`HOLD_ACQ_SOFTEN_*`, `J_HOLD_ACQUISITION` 1.0 m/s^3, arrest floor `J_HOLD_ACQUISITION_ARREST` 2.0 m/s^3); `hold_acq_peak_cmd_jerk` added to the event store as a NON-gating diagnostic (eval.md section 1).
- dRel-honesty flip prepared: `PUBLISH_TRUE_LEAD_DISTANCE = True` at device ISD = 0.0 (all weather variants 0; bit-identical today, later ISD raises compensated by design).
- Living rollout plan adopted: `docs/stopping/rollout_plan.md` (status table, stage 1 baseline-corpus procedure, decision log). Sequencing re-scoped: StopReq/dynamic-jerk stages are independent of the V2 flip; `on_vehicle_protocols.md` cross-linked.
- Decision: keep; stage 1 (baseline corpus, no flag changes) is now active.
