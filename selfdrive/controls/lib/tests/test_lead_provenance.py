"""Track-certificate tests for custom stopping authority."""

import pytest

from openpilot.selfdrive.controls.lib.lead_provenance import (
  StoppingLeadAuthority,
  get_radar_only_min_acquire_d_rel,
)
from openpilot.selfdrive.controls.lib.longitudinal_planner import get_santa_fe_stop_commit_radar_min_acquire_d_rel


def test_speed_bump_track_cannot_self_certify_after_ego_slows() -> None:
  cert = StoppingLeadAuthority()
  # Route 00001fa3 seg12: a centered radar-only return first appeared on the speed bump here.
  assert get_radar_only_min_acquire_d_rel(3.947) == pytest.approx(10.956, abs=0.01)
  assert not cert.update(v_ego=3.947, lead_status=True, lead_d_rel=6.998,
                         lead_track_id=376598, model_prob=0.0)
  # The same close track stays rejected even after braking shrinks the instantaneous horizon.
  assert not cert.update(v_ego=1.923, lead_status=True, lead_d_rel=2.488,
                         lead_track_id=376598, model_prob=0.0)


def test_radar_only_track_seen_early_keeps_authority() -> None:
  cert = StoppingLeadAuthority()
  minimum = get_radar_only_min_acquire_d_rel(3.947)
  assert cert.update(v_ego=3.947, lead_status=True, lead_d_rel=minimum + 0.1,
                     lead_track_id=42, model_prob=0.0)
  assert cert.update(v_ego=1.5, lead_status=True, lead_d_rel=4.5,
                     lead_track_id=42, model_prob=0.0)


def test_later_model_association_certifies_same_radar_track() -> None:
  cert = StoppingLeadAuthority()
  assert not cert.update(v_ego=3.947, lead_status=True, lead_d_rel=6.998,
                         lead_track_id=42, model_prob=0.0)
  assert cert.update(v_ego=3.5, lead_status=True, lead_d_rel=6.5,
                     lead_track_id=42, model_prob=0.25)


def test_track_change_and_dropout_clear_certificate() -> None:
  cert = StoppingLeadAuthority()
  assert cert.update(v_ego=2.0, lead_status=True, lead_d_rel=8.0,
                     lead_track_id=42, model_prob=0.0)
  assert not cert.update(v_ego=2.0, lead_status=True, lead_d_rel=3.0,
                         lead_track_id=43, model_prob=0.0)
  assert not cert.update(v_ego=2.0, lead_status=False, lead_d_rel=0.0,
                         lead_track_id=None, model_prob=0.0)


def test_vision_only_requires_high_confidence_each_frame() -> None:
  cert = StoppingLeadAuthority()
  assert cert.update(v_ego=2.0, lead_status=True, lead_d_rel=5.0,
                     lead_track_id=-1, model_prob=0.97)
  assert not cert.update(v_ego=2.0, lead_status=True, lead_d_rel=5.0,
                         lead_track_id=-1, model_prob=0.6)


def test_service_and_planner_use_the_same_radar_acquisition_envelope() -> None:
  for v_ego in (0.0, 1.5, 2.83, 3.947, 10.0):
    assert get_radar_only_min_acquire_d_rel(v_ego) == get_santa_fe_stop_commit_radar_min_acquire_d_rel(v_ego)
