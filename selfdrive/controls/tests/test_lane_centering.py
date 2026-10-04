from types import SimpleNamespace

import numpy as np
import pytest

from openpilot.selfdrive.controls.lib.lane_centering import GAIN, MAX_RAW_CORRECTION, LaneCenteringController

V_EGO = 20.0
XS = np.linspace(0.0, 50.0, 52)


def _path(y, y_std=0.1):
  return SimpleNamespace(x=XS.copy(), y=np.full_like(XS, float(y)), yStd=np.full_like(XS, float(y_std)))


def _model(left=-1.8, right=1.8, model_y=0.0, lane_prob=0.9, lane_std=0.1, path_std=0.1, lane_change=0):
  return SimpleNamespace(
    laneLines=[_path(0.0), _path(left), _path(right), _path(0.0)],
    laneLineProbs=[0.0, lane_prob, lane_prob, 0.0],
    laneLineStds=[0.0, lane_std, lane_std, 0.0],
    position=_path(model_y, path_std),
    meta=SimpleNamespace(laneChangeState=lane_change),
  )


def _update(controller, model, *, authority=1.0, offset=0.0, enabled=True, active=True, valid=True, speed=V_EGO, signal=False,
            pressed=False):
  return controller.update(0.0, model, speed, enabled, authority, offset, active, valid, signal, pressed)


def _converge(model, *, authority=1.0, offset=0.0):
  controller = LaneCenteringController()
  output = 0.0
  for _ in range(300):
    output = _update(controller, model, authority=authority, offset=offset)
  return controller, output


@pytest.mark.parametrize("kwargs", [{"enabled": False}, {"active": False}, {"valid": False}, {"speed": 4.9}, {"pressed": True}])
def test_hard_gates_are_noop(kwargs):
  assert _update(LaneCenteringController(), _model(left=-1.5, right=2.1), **kwargs) == 0.0


@pytest.mark.parametrize("setting", ["authority", "offset"])
def test_nan_setting_is_noop_and_recovers(setting):
  model = _model(left=-1.5, right=2.1)
  controller = LaneCenteringController()
  assert _update(controller, model, **{'authority': 0.0, setting: np.nan}) == 0.0
  assert 0.0 < _update(controller, model, authority=0.0) < MAX_RAW_CORRECTION * GAIN


def test_lane_change_is_noop():
  assert _update(LaneCenteringController(), _model(left=-1.5, right=2.1, lane_change=1)) == 0.0


def test_lane_center_error_steers_toward_center():
  # positive curvature = right, the same as modelV2 y
  _, right = _converge(_model(left=-1.5, right=2.1), authority=0.0)
  _, left = _converge(_model(left=-2.1, right=1.5), authority=0.0)
  assert right > 0.0
  assert left < 0.0


def test_small_center_error_does_not_chatter():
  _, output = _converge(_model(left=-1.75, right=1.85), authority=0.0)
  assert output == 0.0


@pytest.mark.parametrize("field,value", [("prob", np.nan), ("prob", 0.5), ("prob", 1.1), ("std", np.nan), ("std", -0.1), ("std", 0.4)])
def test_untrusted_lane_lines_are_rejected(field, value):
  model = _model(left=-1.5, right=2.1)
  (model.laneLineProbs if field == "prob" else model.laneLineStds)[1] = value
  assert _update(LaneCenteringController(), model) == 0.0


@pytest.mark.parametrize("left,right", [(-1.2, 1.2), (-2.5, 2.5)])
def test_implausible_lane_width_is_rejected(left, right):
  assert _update(LaneCenteringController(), _model(left=left - 0.2, right=right)) == 0.0


def test_input_must_cover_lookahead():
  model = _model(left=-1.5, right=2.1)
  model.laneLines[1].x = model.laneLines[1].x[:10]
  model.laneLines[1].y = model.laneLines[1].y[:10]
  assert _update(LaneCenteringController(), model) == 0.0


def test_turn_signal_fades_correction_out():
  model = _model(left=-1.5, right=2.1)
  controller, centered = _converge(model, authority=0.0)
  fading = _update(controller, model, authority=0.0, signal=True)
  assert 0.0 < fading < centered
  for _ in range(300):
    fading = _update(controller, model, authority=0.0, signal=True)
  assert abs(fading) < 1e-6


def test_lane_line_loss_fades_correction_out():
  controller, output = _converge(_model(left=-1.5, right=2.1), authority=0.0)
  fading = _update(controller, _model(left=-1.5, right=2.1, lane_prob=0.2), authority=0.0)
  assert 0.0 < fading < output


def test_steering_override_clears_correction():
  model = _model(left=-1.5, right=2.1)
  controller, centered = _converge(model, authority=0.0)
  assert _update(controller, model, authority=0.0, pressed=True) == 0.0
  assert 0.0 < _update(controller, model, authority=0.0) < centered


def test_confident_model_keeps_large_offset_at_full_authority():
  model = _model(left=-1.0, right=2.6, path_std=0.1)
  _, lane_only = _converge(model, authority=0.0)
  _, blended = _converge(model, authority=0.5)
  _, e2e = _converge(model, authority=1.0)
  assert lane_only > blended > 0.0
  assert abs(e2e) < 1e-9


def test_uncertain_model_path_does_not_get_authority():
  _, output = _converge(_model(left=-1.0, right=2.6, path_std=0.6), authority=1.0)
  assert output > 0.0


def test_correction_is_smoothed_and_capped():
  controller = LaneCenteringController()
  model = _model(left=0.0, right=3.0, path_std=0.6)
  first = _update(controller, model, authority=0.0)
  _, steady = _converge(model, authority=0.0)
  assert 0.0 < first < steady
  assert steady == pytest.approx(MAX_RAW_CORRECTION * GAIN, abs=1e-6)


def test_offset_direction():
  # positive offset = keep right = positive curvature
  _, right = _converge(_model(), offset=0.2, authority=0.0)
  _, left = _converge(_model(), offset=-0.2, authority=0.0)
  assert right > 0.0
  assert left < 0.0


def test_offset_cancels_a_matching_model_position():
  # the model already drives 0.2 m right of the line midpoint: a 0.2 m keep-right offset leaves it there
  _, output = _converge(_model(left=-2.0, right=1.6), offset=0.2, authority=0.0)
  assert output == 0.0


def test_offset_keeps_clear_of_the_lines_in_a_narrow_lane():
  # 2.6 m lane: at most 0.2 m from the centre (1.1 m to the line)
  narrow = _model(left=-1.3, right=1.3)
  _, at_limit = _converge(narrow, offset=0.2, authority=0.0)
  _, above_limit = _converge(narrow, offset=0.3, authority=0.0)
  assert at_limit > 0.0
  assert at_limit == pytest.approx(above_limit)


def test_faint_right_line_is_accepted_with_a_confident_left_line():
  # a curb without paint: right line prob 0.3-0.5
  model = _model(left=-1.5, right=2.1)
  model.laneLineProbs[2] = 0.4
  _, output = _converge(model, authority=0.0)
  assert output > 0.0
  model.laneLineProbs[2] = 0.25
  assert _converge(model, authority=0.0)[1] == 0.0


def test_faint_left_line_is_rejected():
  model = _model(left=-1.5, right=2.1)
  model.laneLineProbs[1] = 0.4
  assert _converge(model, authority=0.0)[1] == 0.0
