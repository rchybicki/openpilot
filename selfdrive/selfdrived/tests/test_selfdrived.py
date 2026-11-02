from openpilot.selfdrive.selfdrived.cruise_helpers import cruise_mismatch_detected


def test_cruise_mismatch_requires_pcm_cruise():
  assert not cruise_mismatch_detected(cruise_enabled=True, selfdrive_enabled=True, pcm_cruise=False)
  assert not cruise_mismatch_detected(cruise_enabled=True, selfdrive_enabled=False, pcm_cruise=False)


def test_cruise_mismatch_only_when_pcm_cruise_stays_enabled_after_disable():
  assert cruise_mismatch_detected(cruise_enabled=True, selfdrive_enabled=False, pcm_cruise=True)
  assert not cruise_mismatch_detected(cruise_enabled=False, selfdrive_enabled=False, pcm_cruise=True)
  assert not cruise_mismatch_detected(cruise_enabled=True, selfdrive_enabled=True, pcm_cruise=True)
