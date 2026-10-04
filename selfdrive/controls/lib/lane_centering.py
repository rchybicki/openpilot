"""Lane centering: a small curvature nudge toward the centre of the two primary lane lines.

Ported from StarPilot (firestar5683/StarPilot 9f1066ce8, 3aa1436ff, eac56eea2, d2fb36287; originally PR #74 by @jc01rho).
Not a learned policy: the model's planned position one second ahead is compared with the lane centre at the same
distance, and a capped, smoothed pure-pursuit curvature is added to the model's desired curvature. With full model
authority a confident model path may keep a large offset (0.15 -> 0.50 m fade); with no authority the lane lines win.
The target may be offset from the lane centre (positive = right), never closer than 1.1 m to either line. Changes from
the source: the correction always fades out on a turn signal, and no UI direction indicator. Curvature sign: positive =
right, the same as modelV2 y.
"""
from cereal import log
import numpy as np

from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.controls.lib.drive_helpers import smooth_value

MIN_V_EGO = 5.0
MIN_LANE_PROB = 0.6
MAX_LANE_STD = 0.3
MIN_LANE_WIDTH = 2.6
MAX_LANE_WIDTH = 4.8
MAX_RAW_CORRECTION = 0.004
GAIN = 0.30
SMOOTH_TAU = 0.4
RELEASE_TAU = 0.20
CENTER_ERROR_DEADBAND = 0.08
MAX_OFFSET = 0.3
MIN_CENTER_TO_LINE = 1.1

E2E_MAX_PATH_STD = 0.35
E2E_BREAK_IN_START = 0.15
E2E_BREAK_IN_FULL = 0.50


def _valid_path(x, y) -> bool:
  return bool(x.size >= 2 and x.size == y.size and np.isfinite(x).all() and np.isfinite(y).all() and np.all(np.diff(x) > 0))


def raw_correction(model_v2, v_ego: float, e2e_authority: float, offset: float) -> float | None:
  """Unfiltered curvature correction, or None when the lane lines are not trustworthy."""
  probs = np.asarray(model_v2.laneLineProbs, dtype=float)
  stds = np.asarray(model_v2.laneLineStds, dtype=float)
  if len(model_v2.laneLines) < 3 or probs.size < 3 or stds.size < 3:
    return None
  if not (np.isfinite(probs[1:3]).all() and np.isfinite(stds[1:3]).all()):
    return None
  if np.any(probs[1:3] < MIN_LANE_PROB) or np.any(probs[1:3] > 1.0) or np.any(stds[1:3] < 0.0) or np.any(stds[1:3] > MAX_LANE_STD):
    return None

  left, right, position = model_v2.laneLines[1], model_v2.laneLines[2], model_v2.position
  paths = [(np.asarray(p.x, dtype=float), np.asarray(p.y, dtype=float)) for p in (left, right, position)]
  if not all(_valid_path(x, y) for x, y in paths):
    return None

  lookahead = float(np.clip(v_ego, 8.0, 35.0))
  if not all(x[0] <= lookahead <= x[-1] for x, _ in paths):
    return None

  (left_x, left_y), (right_x, right_y), (pos_x, pos_y) = paths
  left_at = float(np.interp(lookahead, left_x, left_y))
  right_at = float(np.interp(lookahead, right_x, right_y))
  width = right_at - left_at
  if not MIN_LANE_WIDTH <= width <= MAX_LANE_WIDTH:
    return None

  max_offset = min(MAX_OFFSET, max(0.0, 0.5 * width - MIN_CENTER_TO_LINE))
  target = 0.5 * (left_at + right_at) + float(np.clip(offset, -max_offset, max_offset))
  error = target - float(np.interp(lookahead, pos_x, pos_y))
  error_abs = abs(error)
  error = float(np.copysign(max(error_abs - CENTER_ERROR_DEADBAND, 0.0), error))

  # A confident model path keeps authority over a large lane-centre error (a deliberate offset, e.g. a parked car).
  pos_y_std = np.asarray(position.yStd, dtype=float)
  if _valid_path(pos_x, pos_y_std) and 0.0 <= float(np.interp(lookahead, pos_x, pos_y_std)) <= E2E_MAX_PATH_STD:
    break_in = np.clip((error_abs - E2E_BREAK_IN_START) / (E2E_BREAK_IN_FULL - E2E_BREAK_IN_START), 0.0, 1.0)
    error *= 1.0 - e2e_authority * float(break_in)

  return 2.0 * error / lookahead ** 2


class LaneCenteringController:
  def __init__(self) -> None:
    self.correction = 0.0

  def update(self, model_curvature: float, model_v2, v_ego: float, enabled: bool, e2e_authority: float, offset: float,
             lat_active: bool, model_valid: bool, turn_signal: bool, steering_pressed: bool) -> float:
    # a NaN setting (Params stores one) would stick in the filter and in clip_curvature's previous curvature
    if not (enabled and lat_active and model_valid and np.isfinite(e2e_authority) and np.isfinite(offset)) or v_ego < MIN_V_EGO or steering_pressed or \
       model_v2.meta.laneChangeState != log.LaneChangeState.off:
      self.correction = 0.0
      return model_curvature

    correction = None if turn_signal else raw_correction(model_v2, v_ego, e2e_authority, offset)
    if correction is None:
      self.correction = float(smooth_value(0.0, self.correction, RELEASE_TAU, dt=DT_CTRL))
    else:
      target = float(np.clip(correction, -MAX_RAW_CORRECTION, MAX_RAW_CORRECTION)) * GAIN
      self.correction = float(smooth_value(target, self.correction, SMOOTH_TAU, dt=DT_CTRL))
    return model_curvature + self.correction
