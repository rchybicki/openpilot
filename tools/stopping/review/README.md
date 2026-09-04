Stopping review battery (per-cycle log analysis; see docs/stopping/review_cursor.json procedure).
All scripts read one qlog/rlog (.zst) and print JSON lines; run them fanned out with xargs -P.
Env: source .venv/bin/activate (managed Python 3.11).
- human_baseline.py rlog.zst / --aggregate results.jsonl: metric_version=2, exact full 300 ms windows,
  last-ten-seconds scope plus 0.5 s after rest, whole-interval driver classification, metric-specific missing
  reasons and speed strata. Canonical stop IDs deduplicate identical rows and reject conflicts. This is a
  descriptive physical census, not a perceived-comfort score; never mix it with legacy felt or its <=0.8 bar.
  Per-segment detection misses boundary-crossing stops. Continuous route reconstruction remains required
  for a complete approach census. deep_stop.py/stop_index.py felt fields retain their LEGACY definitions.
- triage_one.py     qlog: commit, enabled/stopping frames, bookmarks, stop runs
- incident_detect.py qlog/rlog: TAKEOVER_APPROACH, full-settle LEAPFROG, HARSH_STOP (rlog only: IMU), BOOKMARK
- leapfrog_both.py  qlog/rlog: FULL_SETTLE + NEAR_STOP leapfrog variants
- deep_stop.py      rlog: per-settle quality (rest gap, wire@stop, honest 20Hz IMU jerk, pdec, rebound, hold, taxonomy)
- trace_win.py      rlog t0 t1: frame trace window incl. stopping_service telemetry
- stop_index.py    THE cycle entry point (2026-08-24): qlog triage of every segment -> rlog analysis only for
                   candidate segments -> persistent ~/.route_sync/corpus/stop_index.jsonl; prints ATTENTION rows
                   first (SHORT/LONG/HARSH/TAKEOVER?/NO_SERVICE_EVENT), per-route one-liners, shadow-governor
                   summary, detector audit. Keeps the reviewer's context lean on multi-hour drives with few stops.
- extract_episodes.py / stop_harness.py / test_stop_harness.py   universal-stop-program corpus + prototype harness
- track_win / felt_one / gain_census(+aggregate) / demand_census / aim_deficit / rest_close_replay / mode_census /
  trim_replay   per-stop frame tools (cycle 33)
