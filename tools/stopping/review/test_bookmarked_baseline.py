import pytest
import zstandard

from openpilot.tools.stopping.review.bookmarked_baseline import analyze, bookmark_groups, read_route, rest_intervals
from openpilot.tools.stopping.review.human_baseline import SECOND
from openpilot.tools.stopping.review.triage_one import LOG


def test_button_pair_is_one_mark_but_repeated_press_is_not_lost():
  marks = [{'mono_ns': n, 'type': kind} for n, kind in
           [(SECOND, 'bookmarkButton'), (SECOND + 10, 'userBookmark'),
            (SECOND + 100, 'bookmarkButton'), (SECOND + 110, 'userBookmark')]]
  assert [len(g) for g in bookmark_groups(marks)] == [2, 2]


def test_rest_requires_known_entry_and_continuous_dwell():
  t = [i * 10_000_000 for i in range(100)]
  v = [1] * 20 + [0] * 80
  assert rest_intervals(t, v, [True] * 100) == [(20, 99)]
  assert rest_intervals(t[20:], v[20:], [True] * 80) == []
  assert rest_intervals(t[:50] + t[70:], v[:50] + v[70:], [True] * 80) == []


def test_continuous_packet_keeps_manual_terminal_separate_from_mixed_approach():
  data = {key: [] for key in ('car', 'state', 'pose', 'lead', 'wire', 'bookmarks', 'files', 'init')}
  data['route'] = '000020bf--9dcbe4db3b'
  data['init'] = [{'mono_ns': SECOND}]
  for i in range(4001):
    t = SECOND + i * 10_000_000
    speed = 4 if i < 3500 else .3 if i < 3600 else 0
    # Artificial segment cut inside the stop's approach does not reset either clock or state.
    data['car'].append([t, speed, -.1, True, i >= 3400, False, speed, 0, 'drive', f'{data["route"]}--{int(i >= 3550)}'])
    data['state'].append([t, i < 3400])
    data['pose'].append([t, 0, .01, True, True])
  data['bookmarks'] = [{'mono_ns': 38 * SECOND, 'type': 'userBookmark'}]
  result = analyze(data)
  stop, = result['stops']
  assert stop['windows']['terminal']['class'] == 'manual'
  assert stop['windows']['last10s']['class'] == 'mixed'
  assert stop['v2']['cls'] == 'mixed'
  assert stop['manual_prefix']['seconds_before_stop'] == 2
  assert stop['pitch_device_y_peak'] == {'value': .01, 'reason': None}
  assert result['bookmarks'][0]['candidate_ids'] == [stop['event_id']]
  for row in data['pose']:
    if row[0] in (35 * SECOND - 10_000_000, 38 * SECOND + 10_000_000):
      row[2] = 99  # Bracketing samples validate continuity but are outside the peak window.
  assert analyze(data)['stops'][0]['pitch_device_y_peak']['value'] == .01
  for row in data['car']:
    row[1] = min(row[1], 2.5)  # A low-speed bad stop must survive the legacy v2 peak-speed exclusion.
  low_speed_stop, = analyze(data)['stops']
  assert low_speed_stop['v2'] is None and low_speed_stop['windows']['terminal']['class'] == 'manual'
  data['bookmarks'][0]['mono_ns'] = 35 * SECOND  # No nearest-stop guess before braking finishes.
  assert analyze(data)['bookmarks'][0]['status'] == 'unmatched'


def test_truncated_logs_and_duplicate_segments_are_rejected(tmp_path):
  path = tmp_path / '000020bf--9dcbe4db3b--0' / 'rlog.zst'
  path.parent.mkdir()
  event = LOG.Event.new_message(logMonoTime=SECOND, valid=True)
  event.init('carState')
  raw = zstandard.ZstdCompressor().compress(event.to_bytes())
  path.write_bytes(raw[:-1])
  with pytest.raises(ValueError, match='incomplete'):
    read_route([path])
  path.write_bytes(raw)
  with pytest.raises(ValueError, match='non-increasing'):
    read_route([path, path])
