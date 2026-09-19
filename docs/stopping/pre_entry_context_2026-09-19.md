# Reset interrupted pre-entry observation

The pre-entry governor diagnostics retained their previous gap and stopped-lead
dwell when observation stopped. `LongControl.reset()` cleared the telemetry ring
but left this context intact. A new approach could therefore publish a fresh
timestamp with stale internal geometry.

The regression reproduces an old track at 6 m, an observation interruption, then
a new track at 30 m. Before the fix, the first resumed conditioned gap is 5.97 m.
After the fix, it is 30 m, with fresh track age, command history and stopped-lead
dwell. Interruptions cover inactivity, speed reaching the 4.5 m/s observation
boundary, live service ownership, and a direct controller reset.

Three runtime lines reset the existing pre-entry context at those boundaries.
The live service context has its own observation lifetime and remains separate:
both contexts can update in the same control frame, so sharing one instance would
advance its filters twice. Only diagnostics consume the changed pre-entry state.
This is a telemetry reliability fix, not a stopping comfort improvement.

Validation against `6e25c96d37`:

- All four added regression cases fail before the fix and pass after it.
- The 73 context-lifetime and governor tests pass; Ruff and diff checks pass.
- 3,180 paired control frames across LIVE, LIVE_TERMINAL and SHADOW remain exactly
  equal for acceleration request, ownership, phase, live context and pending brake
  estimate. The applied runtime file exactly matches the externally tested source.

Evidence, frozen source, differential checks and hashes are in
`~/.route_sync/corpus/post_6e25_review_20260919/pre_context_lifetime/`.
