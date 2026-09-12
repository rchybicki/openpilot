import hashlib
import json

import numpy as np
import pytest
import zstandard
from opendbc.can import CANPacker

from openpilot.tools.stopping.review.can_response import extract
from openpilot.tools.stopping.review.triage_one import LOG


def packet(tmp_path, events):
  route = '000020c1--8f82c447fd'
  folder = tmp_path / (route + '--0')
  folder.mkdir()
  path = folder / 'rlog.zst'
  raw = zstandard.ZstdCompressor().compress(b''.join(e.to_bytes() for e in events))
  path.write_bytes(raw)
  p = tmp_path / 'signals.json'
  p.write_text(json.dumps({'route': route, 'files': [
    {'path': str(path), 'bytes': len(raw), 'sha256': hashlib.sha256(raw).hexdigest()}]}))
  return p


def event(kind, ns, requests, valid=True):
  e = LOG.Event.new_message(logMonoTime=ns, valid=valid)
  frames = e.init(kind, len(requests))
  for frame, (address, payload, bus) in zip(frames, requests, strict=True):
    frame.address, frame.dat, frame.src = address, payload, bus
  return e


def test_batch_preserves_each_request_and_exact_nanoseconds(tmp_path):
  packer = CANPacker('hyundai_kia_generic')
  first = packer.make_can_msg('SCC12', 128, {'aReqValue': -.7, 'aReqRaw': -.7, 'ACCMode': 1, 'CR_VSM_Alive': 1})
  second = packer.make_can_msg('SCC12', 128, {'aReqValue': -.4, 'aReqRaw': -.4, 'ACCMode': 1, 'CR_VSM_Alive': 2})
  ns = 2**53 + 1
  p = packet(tmp_path, [event('can', ns, [first, second]), event('can', ns + 1, [second])])
  out = tmp_path / 'result'
  manifest = extract(p, out)
  with np.load(out / 'can.npz') as data:
    assert data['echo_1057_ns'].tolist() == [ns, ns, ns + 1]
    np.testing.assert_allclose(data['echo_1057_values'][:, 0], [-.7, -.4, -.4])
    assert data['echo_1057_valid'].tolist() == [True, True, True]
    assert data['send_1057_values'].shape == (0, 5)
  assert manifest['streams']['echo_1057']['equal_timestamps'] == 1
  assert manifest['can_sha256'] == hashlib.sha256((out / 'can.npz').read_bytes()).hexdigest()


def test_invalid_and_short_frames_cannot_reuse_previous_command(tmp_path):
  cmd = CANPacker('hyundai_kia_generic').make_can_msg('SCC12', 0, {'aReqValue': -1., 'ACCMode': 1})
  short = (cmd[0], cmd[1][:3], cmd[2])
  p = packet(tmp_path, [event('sendcan', 1, [cmd]), event('sendcan', 2, [cmd], valid=False),
                        event('sendcan', 3, [short]), event('can', 4, [cmd])])
  out = tmp_path / 'result'
  extract(p, out)
  with np.load(out / 'can.npz') as data:
    assert data['send_1057_valid'].tolist() == [True, False, False]
    assert np.isnan(data['send_1057_values'][1:]).all()
    assert len(data['echo_1057_ns']) == 0  # incoming bus 0 is not a transmit echo


def test_sensor_fields_and_jerk_request_streams(tmp_path):
  packer = CANPacker('hyundai_kia_generic')
  esp = packer.make_can_msg('ESP12', 0, {'LONG_ACCEL': -1.23, 'CYL_PRES': 12.3, 'ESP12_AliveCounter': 9})
  tcs = packer.make_can_msg('TCS13', 0, {'ACCEL_REF_ACC': .37, 'DriverOverride': 2})
  jerk = packer.make_can_msg('SCC14', 0, {'JerkUpperLimit': 2.3, 'JerkLowerLimit': 5.4, 'ACCMode': 1})
  p = packet(tmp_path, [event('can', 1, [esp, tcs]), event('sendcan', 2, [jerk]),
                        event('can', 3, [(jerk[0], jerk[1], 128), (esp[0], esp[1], 2)])])
  out = tmp_path / 'result'
  extract(p, out)
  with np.load(out / 'can.npz') as data:
    assert data['sensor_544_values'].shape == (1, 7)
    np.testing.assert_allclose(data['sensor_544_values'][0, [0, 3, 6]], [-1.23, 12.3, 9])
    np.testing.assert_allclose(data['sensor_916_values'], [[.37, 2]])
    np.testing.assert_allclose(data['send_905_values'], [[2.3, 5.4, 1]])
    np.testing.assert_array_equal(data['echo_905_values'], data['send_905_values'])
    assert data['sensor_544_valid'].tolist() == [True]


@pytest.mark.parametrize('bad_source', ['hash', 'truncated', 'second_frame', 'duplicate', 'different_route'])
def test_rejects_untrusted_source_before_writing(tmp_path, bad_source):
  p = packet(tmp_path, [event('can', 1, [])])
  d = json.loads(p.read_text())
  source = tmp_path / '000020c1--8f82c447fd--0/rlog.zst'
  if bad_source in ('truncated', 'second_frame'):
    raw = source.read_bytes()
    raw = raw[:-1] if bad_source == 'truncated' else raw + zstandard.ZstdCompressor().compress(b'')
    source.write_bytes(raw)
    d['files'][0].update(bytes=len(raw), sha256=hashlib.sha256(raw).hexdigest())
  elif bad_source == 'hash':
    d['files'][0]['sha256'] = '0' * 64
  elif bad_source == 'duplicate':
    d['files'].append(d['files'][0])
  else:
    d['route'] = '000020c0--different'
  p.write_text(json.dumps(d))
  out = tmp_path / 'result'
  with pytest.raises(ValueError):
    extract(p, out)
  assert not out.exists()


def test_rejects_time_reversal(tmp_path):
  cmd = CANPacker('hyundai_kia_generic').make_can_msg('SCC12', 128, {'aReqValue': -.7})
  p = packet(tmp_path, [event('can', 2, [cmd]), event('can', 1, [cmd])])
  with pytest.raises(ValueError, match='decreasing'):
    extract(p, tmp_path / 'result')


def test_keeps_existing_output(tmp_path):
  # An existing frozen packet must stay intact even if the supplied input is valid.
  p = packet(tmp_path, [event('can', 1, [])])
  out = tmp_path / 'result'
  out.mkdir()
  (out / 'manifest.json').write_text('keep')
  with pytest.raises(FileExistsError):
    extract(p, out)
  assert (out / 'manifest.json').read_text() == 'keep'
