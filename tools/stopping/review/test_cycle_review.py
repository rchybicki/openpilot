import hashlib
import json

import pytest

from openpilot.tools.stopping.review.bookmarked_baseline import analyze
from openpilot.tools.stopping.review.cycle_review import review
from openpilot.tools.stopping.review.human_baseline import SECOND


@pytest.fixture
def packet(tmp_path):
  source = tmp_path / 'rlog.zst'
  source.write_bytes(b'original')
  data = {key: [] for key in ('car', 'state', 'pose', 'lead', 'wire', 'bookmarks')}
  data.update(route='000020fa--32a67f8d0c', init=[{'mono_ns': SECOND, 'commit': 'running-revision'}],
              files=[{'path': str(source), 'bytes': 8, 'sha256': hashlib.sha256(b'original').hexdigest()}])
  for i in range(4001):
    ns = SECOND + i * SECOND // 100
    speed = 2. if i < 3500 else .3 if i < 3600 else 0.
    data['car'].append([ns, speed, -.5 if i < 3600 else 0., True, i >= 3400, False, speed, 0., 'drive', data['route'] + '--0'])
    data['state'].append([ns, i < 3400])
  data['bookmarks'] = [{'mono_ns': 38 * SECOND, 'type': 'userBookmark'}]
  (tmp_path / 'signals.json').write_text(json.dumps(data))
  (tmp_path / 'baseline.json').write_text(json.dumps(analyze(data)))
  return tmp_path


def test_slow_stop_keeps_signed_windows_mixed_approach_and_unlabelled_bookmark(packet):
  result = review([packet])
  row, = result['rows']
  assert row['v2'] is None
  assert row['windows']['last30s']['class'] == row['windows']['last10s']['class'] == 'mixed'
  terminal = row['windows']['terminal']
  assert terminal['class'] == 'manual'
  assert terminal['jerk_300_signed']['min'] == 0.
  assert terminal['jerk_300_signed']['max'] == pytest.approx(5 / 3)
  assert row['bands']['2.5']['reason'] == 'no_crossing_in_30s'
  assert row['bands']['1.0']['class'] == 'manual'
  provenance, = result['packets']
  assert provenance['init'][0]['commit'] == 'running-revision'
  assert provenance['bookmarks'][0]['candidate_ids'] == [row['event_id']]
  assert 'label' not in row
  with pytest.raises(ValueError, match='duplicate event'):
    review([packet, packet])


@pytest.mark.parametrize('defect', ['route', 'init', 'files', 'event_id', 'source_bytes', 'source_hash'])
def test_corrupt_provenance_and_identity_are_rejected(packet, defect):
  path = packet / 'baseline.json'
  baseline = json.loads(path.read_text())
  if defect in ('route', 'init', 'files'):
    baseline[defect] = 'different'
  elif defect == 'event_id':
    baseline['stops'][0]['event_id'] = 'other@1'
  else:
    (packet / 'rlog.zst').write_bytes(b'changed' if defect == 'source_bytes' else b'changed!')
  path.write_text(json.dumps(baseline))
  with pytest.raises(ValueError, match='mismatched|source changed'):
    review([packet])


@pytest.mark.parametrize('defect', ['stale_control_state', 'deleted_stop', 'scorer'])
def test_cached_census_must_match_signals_and_current_scorer(packet, defect):
  path = packet / ('signals.json' if defect == 'stale_control_state' else 'baseline.json')
  data = json.loads(path.read_text())
  if defect == 'stale_control_state':
    data['state'] = [[row[0], False] for row in data['state']]
  elif defect == 'deleted_stop':
    data['stops'].clear()
  else:
    data['scorer_sha256']['bookmarked_baseline.py'] = 'old-scorer'
  path.write_text(json.dumps(data))
  with pytest.raises(ValueError, match='mismatched derived baseline or scorer'):
    review([packet])


@pytest.mark.parametrize('defect,reason', [('invalid', 'invalid_sample'), ('gap', 'timestamp_gap'), ('phase', 'missing_phase_start')])
def test_signed_windows_preserve_missing_context(packet, defect, reason):
  path = packet / 'signals.json'
  data = json.loads(path.read_text())
  if defect == 'invalid':
    data['car'][3520][3] = False
  elif defect == 'gap':
    del data['car'][3520:3540]
  else:
    data['car'] = data['car'][3500:]
  (packet / 'baseline.json').write_text(json.dumps(analyze(data)))
  path.write_text(json.dumps(data))
  row, = review([packet])['rows']
  window = row['windows']['terminal' if defect == 'phase' else 'last10s']
  assert window['class'] == 'unknown' and window['reason'] == reason
  assert window['jerk_300_signed']['min'] is None and window['jerk_300_signed']['reason'] == reason
