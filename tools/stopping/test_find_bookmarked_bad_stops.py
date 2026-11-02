from types import SimpleNamespace

from tools.stopping.find_bookmarked_bad_stops import build_markdown, match_flag_to_review_event


def test_match_flag_to_review_event_prefers_nearest_prior_dynamic_stop():
  flag_t = 182.623
  events = [
    SimpleNamespace(event_id=1, start_segment=0, entry_speed_mps=0.03, stop_hold_time_s=182.408),
    SimpleNamespace(event_id=2, start_segment=2, entry_speed_mps=5.44, stop_hold_time_s=181.108),
    SimpleNamespace(event_id=3, start_segment=4, entry_speed_mps=4.65, stop_hold_time_s=190.000),
  ]

  event, match_type = match_flag_to_review_event(
    flag_t=flag_t,
    events=events,
    min_entry_speed=1.0,
    nearest_max_gap=12.0,
  )

  assert event is not None
  assert event.event_id == 2
  assert match_type == "nearest_prior_dynamic"


def test_build_markdown_includes_review_event_columns():
  report = {
    "host": "comma",
    "generated_utc": "2026-04-19T12:30:00+00:00",
    "routes_analyzed": 1,
    "routes_with_bookmarks": 1,
    "total_bookmarks": 1,
    "matched_bookmarks": 1,
    "unmatched_bookmarks": 0,
    "event_mode": "engaged_signal",
    "min_entry_speed": 0.5,
    "routes": [{
      "route": "0000031b--2ce6cada05",
      "bookmark_count": 1,
      "matched_count": 1,
      "unmatched_count": 0,
      "event_count": 7,
    }],
    "bookmark_matches": [{
      "route": "0000031b--2ce6cada05",
      "segment": 3,
      "bookmark_t_rel_s": 182.623,
      "match_type": "window",
      "delta_to_hold_s": 0.215,
      "event": {
        "event_id": 2,
        "rollout_distance_from_2mps_m": 0.01,
        "entry_stop_jerk_mps3": 0.21,
        "entry_stop_cmd_jerk_mps3": 0.0,
        "end_stop_jerk_mps3": 0.21,
        "end_stop_accel_step_mps2": 0.02,
        "end_stop_cmd_jerk_mps3": 0.0,
        "end_stop_cmd_step_mps2": 0.0,
        "creep_after_stop_mps": 0.022,
      },
      "review_delta_to_hold_s": 1.515,
      "review_event": {
        "event_id": 2,
        "start_segment": 2,
      },
    }],
  }

  markdown = build_markdown(report)

  assert "Review|ReviewDelta" in markdown
  assert "|2/2|1.515|" in markdown
