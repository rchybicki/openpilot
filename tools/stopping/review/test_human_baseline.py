import copy

import pytest

from openpilot.tools.stopping.review.human_baseline import SECOND, aggregate, classify, jerk_max, score_stops, state_at


def test_full_window_interpolation_and_initial_eight_ms_spike():
  t = [ms * 1_000_000 for ms in (0, 8, 80, 160, 240, 300, 380, 460)]
  a = [0, .096, .096, .096, .096, .096, .096, .096]
  assert jerk_max(t, a, [True] * len(t), 0, t[-1])['value'] == pytest.approx(.32)
  assert jerk_max(t, [2 * ns / SECOND for ns in t], [True] * len(t), 0, t[-1])['value'] == pytest.approx(2)
  assert jerk_max(t, a, [True] * len(t), 0, t[4]) == {'value': None, 'reason': 'short_window'}
  # The exact 300 ms endpoint counts; 299,999,999 ns does not.
  assert jerk_max(t, a, [True] * len(t), 0, 300_000_000)['value'] == pytest.approx(.32)
  assert jerk_max(t, a, [True] * len(t), 0, 299_999_999)['reason'] == 'short_window'


def test_gaps_invalid_samples_and_interpolation_brackets():
  t = [i * 100_000_000 for i in range(5)]
  assert jerk_max(t, [0] * 5, [True] * 5, 0, t[-1])['value'] == 0
  for bad_t in ([0, 100_000_001, 200_000_000, 300_000_000, 400_000_000], [0, 0, 200_000_000, 300_000_000, 400_000_000]):
    assert jerk_max(bad_t, [0] * 5, [True] * 5, 0, bad_t[-1])['reason'] == 'timestamp_gap'
  assert jerk_max(t, [0, float('nan'), 0, 0, 0], [True] * 5, 1, t[-1])['reason'] == 'invalid_sample'
  assert jerk_max(t, [0] * 5, [False, True, True, True, True], 1, t[-1])['reason'] == 'invalid_sample'
  assert jerk_max(t, [0] * 5, [True] * 5, -1, t[-1])['reason'] == 'missing_context'


def test_classification_checks_the_entire_interval_and_freshness():
  assert state_at(500_000_000, [0, SECOND], [True, False]) is True
  assert state_at(500_000_001, [0, SECOND], [True, False]) is None
  assert state_at(-1, [0], [True]) is None
  assert state_at(SECOND, [0, SECOND], [True, False]) is False
  assert classify([True, True], [False, False], [False, False]) == 'engaged'
  assert classify([False, False], [True, False], [False, False]) == 'manual'
  assert classify([False, False], [False, True], [True, False]) == 'manual'
  assert classify([False, True], [False, False], [False, False]) == 'mixed'
  assert classify([True, True], [False, False], [True, False]) == 'mixed'
  assert classify([True, True], [True, False], [False, False]) == 'mixed'
  assert classify([None, True], [False, False], [False, False]) == 'unknown'


def stop_trace():
  t = [i * 10_000_000 for i in range(1201)]
  v = [4 if i < 1080 else 2 if i < 1090 else .3 if i < 1100 else 0 for i in range(len(t))]
  return [t, v, [-i / 2000 for i in range(len(t))], [True] * len(t), [True] * len(t), [False] * len(t), [False] * len(t)]


def test_stop_detection_endpoints_missingness_and_overrides():
  trace = stop_trace()
  row, = score_stops(*trace)
  assert row['stop_mono_ns'] == 11 * SECOND and row['cls'] == 'engaged'
  assert row['metrics']['jerk_300_term']['value'] == pytest.approx(.05)
  assert row['metrics']['a_last_ge_010']['value'] == trace[2][1099]
  assert row['metrics']['a_min_late']['value'] == trace[2][1089]
  assert len(score_stops(*(x[:1151] for x in trace))) == 1
  assert score_stops(*(x[:1150] for x in trace)) == []
  trace[6][150] = True  # A pedal press nine seconds before rest invalidates engaged classification.
  assert score_stops(*trace)[0]['cls'] == 'mixed'
  trace[3][1101] = False  # Do not bridge the invalid hold sample or rediscover the same stopped car.
  assert score_stops(*trace) == []
  trace = stop_trace()
  partial, = score_stops(*(x[500:] for x in trace))
  assert partial['cls'] == 'unknown'
  assert partial['metrics']['jerk_300_approach']['reason'] == 'missing_context'
  assert partial['metrics']['jerk_300_term']['value'] == pytest.approx(.05)
  with pytest.raises(ValueError, match='strictly increasing'):
    score_stops(*([trace[0][:10] + trace[0][9:]] + [x[:10] + x[9:] for x in trace[1:]]))
  assert score_stops(*[x[:1101] + x[1115:] for x in trace]) == []


def test_aggregate_versions_deduplication_and_per_metric_denominators():
  row, = score_stops(*stop_trace())
  row.update(metric_version=2, seg='00002000--7115083742--0', event_id=f'00002000--7115083742--0@{row["stop_mono_ns"]}')
  assert aggregate([row, copy.deepcopy(row)])['unique_stops'] == 1
  missing = copy.deepcopy(row)
  missing['stop_mono_ns'] += SECOND
  missing['event_id'] = f'{missing["seg"]}@{missing["stop_mono_ns"]}'
  missing['metrics']['jerk_300_term'] = {'value': None, 'reason': 'short_window'}
  summary = aggregate([row, missing])
  group = next(g for g in summary['groups'] if g['speed_band'] == 'all')
  assert group['stops'] == 2
  assert group['metrics']['jerk_300_term']['n'] == 1
  assert group['metrics']['jerk_300_term']['missing'] == {'short_window': 1}
  assert group['metrics']['a_min_late']['n'] == 2
  bad = copy.deepcopy(row)
  bad['v_appr'] = 5
  with pytest.raises(ValueError, match='conflicting duplicate'):
    aggregate([row, bad])
  with pytest.raises(ValueError, match='metric_version'):
    aggregate([row, {'metric_version': 1}])
  bad = copy.deepcopy(row)
  bad['metrics']['jerk_300_term']['value'] = float('nan')
  with pytest.raises(ValueError, match='malformed'):
    aggregate([bad])
  bad['metrics']['jerk_300_term']['value'] = -1
  with pytest.raises(ValueError, match='non-negative'):
    aggregate([bad])
  ranked = []
  for value in range(10):
    record = copy.deepcopy(row)
    record['stop_mono_ns'] += value * SECOND
    record['event_id'] = f'{record["seg"]}@{record["stop_mono_ns"]}'
    record['metrics']['jerk_300_term']['value'] = value
    ranked.append(record)
  group = next(g for g in aggregate(ranked)['groups'] if g['speed_band'] == 'all')
  assert group['metrics']['jerk_300_term']['p50'] == 4.5
  assert group['metrics']['jerk_300_term']['p90'] == 8
