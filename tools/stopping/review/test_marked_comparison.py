import copy
import hashlib
import json

import pytest

from openpilot.tools.stopping.review.marked_comparison import compare, describe
from openpilot.tools.stopping.review.human_baseline import SECOND


def packet():
  data = {'car': [], 'state': []}
  for i in range(4101):
    ns = SECOND + i * SECOND // 100
    v = max(0., min(6., (4000 - i) / 100))
    data['car'].append([ns, v, -.5 if v else 0., True, i >= 3700, False, v])
    data['state'].append([ns, i < 3700])
  return data, 41 * SECOND


def test_each_speed_band_owns_its_control_mode_and_rolling_integral():
  data, stop = packet()
  result = describe(data, stop)
  assert result['bands']['5.0']['class'] == 'mixed'
  band = result['bands']['2.5']
  assert band['class'] == 'manual'
  assert band['seconds_to_filtered_rest'] == 2.5
  assert band['wheel_distance_m'] == pytest.approx(3.125)
  assert band['max_speed_recovery_mps'] == 0
  assert result['easing_delta_a'] == result['raw_anchor_easing_delta_a'] == 0


@pytest.mark.parametrize('defect,reason', [('invalid', 'invalid_sample'), ('gap', 'timestamp_gap'), ('truncated', 'missing_context')])
def test_incomplete_band_never_scores(defect, reason):
  data, stop = packet()
  if defect == 'invalid':
    data['car'][3800][3] = False
  elif defect == 'gap':
    del data['car'][3790:3810]
  else:
    data['car'] = data['car'][:4020]
  band = describe(data, stop)['bands']['2.5']
  assert band['class'] == 'unknown' and band['reason'] == reason
  assert 'wheel_distance_m' not in band


def test_rebound_is_retained_without_dropping_low_speed_event():
  data, stop = packet()
  for i in range(3800, 3820):
    data['car'][i][1] = .7 if i < 3810 else .95
  result = describe(data, stop)
  assert result['bands']['2.5']['max_speed_recovery_mps'] >= .25
  assert result['last30s']['class'] == 'mixed'
  for row in data['car']:
    row[1] = min(row[1], 2.)
  assert describe(data, stop)['bands']['2.5']['reason'] == 'no_crossing_in_30s'
  assert describe(data, stop)['bands']['1.0']['reason'] is None


def test_raw_channel_invalidity_and_unknown_state_do_not_inherit_validity():
  data, stop = packet()
  original = copy.deepcopy(data)
  data['car'][3980][6] = float('nan')
  result = describe(data, stop)
  assert result['easing_delta_a'] == 0 and result['raw_anchor_easing_delta_a'] is None
  original['state'] = []
  assert describe(original, stop)['bands']['2.5']['class'] == 'unknown'


def test_clock_disorder_is_rejected():
  data, stop = packet()
  data['state'][10][0] = data['state'][9][0]
  with pytest.raises(ValueError, match='selfdriveState'):
    describe(data, stop)


def test_explicit_labels_and_frozen_sources_cannot_silently_disappear(tmp_path):
  data, stop = packet()
  source = tmp_path / 'source'
  source.write_bytes(b'original')
  data.update(route='route', files=[{'path': str(source), 'bytes': 8, 'sha256': hashlib.sha256(b'original').hexdigest()}])
  baseline = {key: data[key] for key in ('route', 'files')}
  baseline['stops'] = [{'event_id': f'route@{stop}', 'stop_mono_ns': stop, 'segment': 'route--0',
                        'lead_at_stop': None, 'pose_device_x_jerk_300': {'value': None, 'reason': 'missing_context'}}]
  labels = tmp_path / 'labels.json'
  labels.write_text(json.dumps({'manual_reference_ids': [f'route@{stop}'], 'bad_openpilot_ids': []}))
  (tmp_path / 'signals.json').write_text(json.dumps(data))
  (tmp_path / 'baseline.json').write_text(json.dumps(baseline))
  assert len(compare(labels, [tmp_path], [])['rows']) == 1
  with pytest.raises(ValueError, match='duplicate selected'):
    compare(labels, [tmp_path, tmp_path], [])
  labels.write_text(json.dumps({'manual_reference_ids': ['missing'], 'bad_openpilot_ids': []}))
  with pytest.raises(ValueError, match='missing labelled'):
    compare(labels, [tmp_path], [])
  labels.write_text(json.dumps({'manual_reference_ids': ['same'], 'bad_openpilot_ids': ['same']}))
  with pytest.raises(ValueError, match='conflicting explicit'):
    compare(labels, [tmp_path], [])
  labels.write_text(json.dumps({'manual_reference_ids': [f'route@{stop}'], 'bad_openpilot_ids': []}))
  source.write_bytes(b'changed!')
  with pytest.raises(ValueError, match='source changed'):
    compare(labels, [tmp_path], [])
