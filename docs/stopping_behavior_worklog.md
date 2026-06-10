# Stopping Behavior Worklog (legacy path)

The 2026 H1 worklog content was archived to `docs/stopping/archive/worklog_2026H1.md`
(stopping-stack redesign, FINAL_SPEC commit 5b docs scaffold).

New entries belong in `docs/stopping/worklog.md`. The `append_*_report.py` scripts already
default there; `tools/stopping/run_stopping_cycle.py` keeps passing this legacy path until
its scheduled cleanup-commit `DEFAULT_WORKLOG` flip, so cycle-driven entries may still
accumulate below this header until then. Move them to `docs/stopping/worklog.md` when the
flip lands, then delete this stub.
