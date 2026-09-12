"""A completed stop must not leave frozen geometry for the next approach."""
import pytest

from openpilot.selfdrive.controls.lib import stopping_flags
from openpilot.selfdrive.controls.lib.longcontrol import LongControl
from openpilot.selfdrive.controls.lib.stopping_service import Phase
from openpilot.selfdrive.controls.lib.stopping_telemetry import StoppingTelemetry
from openpilot.selfdrive.controls.lib.tests.test_longcontrol_fast_release import DummyCarParams, DummyCarState, DummyFrogPilotToggles
from openpilot.selfdrive.controls.lib.tests.test_longcontrol_service_live import _queue_release_scenario


@pytest.mark.parametrize('mode', ['LIVE', 'LIVE_TERMINAL', 'SHADOW'])
@pytest.mark.parametrize('active', [False, True])
@pytest.mark.parametrize('next_track', [7, 8])
@pytest.mark.parametrize('next_gap', [5.0, 20.0])
def test_natural_release_then_observation_exit_clears_context(monkeypatch, mode, active, next_track, next_gap):
  monkeypatch.setattr(stopping_flags, 'SERVICE_MODE', mode)
  lc = LongControl(DummyCarParams())
  events = []
  lc._service_shadow_tel = StoppingTelemetry(log_fn=lambda **kw: events.append(kw))
  _queue_release_scenario(lc, lead_track_id=7)
  assert lc._service_shadow_svc.phase == Phase.INACTIVE  # natural RELEASE completed in band
  ctx = lc._service_shadow_ctx
  assert ctx._d_gap is not None and ctx._cmd_buf and ctx._a_coast != 0.0
  assert sum(e.get('kind') == 'settle_summary' for e in events) == 1

  toggles = DummyFrogPilotToggles()
  for _ in range(3):
    lc.update(active, DummyCarState(v_ego=2.6, a_ego=0.2), 0.35, False, -1.0, (-3.0, 2.0), toggles,
              lead_status=True, lead_v=3.0, lead_d_rel=25.0, lead_track_id=next_track, lead_model_prob=0.99)
  assert ctx._d_gap is None and ctx._a_coast == 0.0 and not ctx._cmd_buf
  assert ctx._track_age_t == 0.0 and not ctx._latch_entry.stopped and not ctx._latch_strict.stopped
  assert sum(e.get('kind') == 'settle_summary' for e in events) == 1

  lc.update(True, DummyCarState(v_ego=2.4, a_ego=-0.6), -0.6, True, next_gap - 4.3, (-3.0, 2.0), toggles,
            lead_status=True, lead_v=0.0, lead_d_rel=next_gap, lead_track_id=next_track, lead_model_prob=0.99)
  assert ctx._d_gap == next_gap and ctx._gap_source == 'measured'
  assert ctx._track_age_t == pytest.approx(0.01)  # trust must be earned again


def test_band_flaps_reenter_after_fresh_stopped_lead_dwell(monkeypatch):
  monkeypatch.setattr(stopping_flags, 'SERVICE_MODE', 'LIVE')
  lc = LongControl(DummyCarParams())
  lc._service_shadow_tel = StoppingTelemetry(log_fn=lambda **kw: None)
  toggles = DummyFrogPilotToggles()
  for i in range(231):
    # No planner stop request: entry requires fresh lead evidence. Each short out-of-band
    # interval ends observation; after the last one a full 0.3 s dwell must suffice.
    v = (2.55 if (i // 10) % 2 else 2.45) if i < 200 else 2.45
    lc.update(True, DummyCarState(v_ego=v, a_ego=-0.1), -0.1, False, -1.0, (-3.0, 2.0), toggles,
              lead_status=True, lead_v=0.0, lead_d_rel=12.0, lead_track_id=7, lead_model_prob=0.99)
    if i < 200:
      assert not lc._service_live_owning
    if v > 2.5:
      assert lc._service_shadow_ctx._a_coast == 0.0 and not lc._service_shadow_ctx._cmd_buf
  assert lc._service_shadow_ctx._latch_entry.stopped
  assert lc._service_live_owning


def test_old_close_gap_cannot_start_a_new_stop_behind_a_far_lead(monkeypatch):
  monkeypatch.setattr(stopping_flags, 'SERVICE_MODE', 'LIVE')
  lc = LongControl(DummyCarParams())
  lc._service_shadow_tel = StoppingTelemetry(log_fn=lambda **kw: None)
  _queue_release_scenario(lc, n=500, lead_track_id=7)
  assert lc._service_shadow_svc.phase == Phase.INACTIVE
  toggles = DummyFrogPilotToggles()
  for i in range(41):
    lc.update(True, DummyCarState(v_ego=3.0 if i == 0 else 2.2, a_ego=-0.3), -0.3, False, -1.0, (-3.0, 2.0), toggles,
              lead_status=True, lead_v=0.0, lead_d_rel=27.5, lead_track_id=8, lead_model_prob=0.99)
    assert not lc._service_live_owning
  assert lc._service_shadow_ctx._latch_entry.stopped
  assert lc._service_shadow_ctx._d_gap == 27.5
