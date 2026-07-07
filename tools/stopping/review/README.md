Stopping review battery (per-cycle log analysis; see docs/stopping/review_cursor.json procedure).
All scripts read one qlog/rlog (.zst) and print JSON lines; run them fanned out with xargs -P.
Env (local .venv workaround): PYTHONPATH=<repo>:<repo>/.venv/lib/python3.11/site-packages /opt/homebrew/bin/python3.11
- triage_one.py     qlog: commit, enabled/stopping frames, bookmarks, stop runs
- incident_detect.py qlog/rlog: TAKEOVER_APPROACH, full-settle LEAPFROG, HARSH_STOP (rlog only: IMU), BOOKMARK
- leapfrog_both.py  qlog/rlog: FULL_SETTLE + NEAR_STOP leapfrog variants
- deep_stop.py      rlog: per-settle quality (rest gap, wire@stop, honest 20Hz IMU jerk, pdec, rebound, hold, taxonomy)
- trace_win.py      rlog t0 t1: frame trace window incl. stopping_service telemetry
