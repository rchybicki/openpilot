import math
import os

import numpy as np
import pytest

from cereal import log
from openpilot.common.file_chunker import get_manifest_path, open_file_chunked
from openpilot.common.params import Params
from openpilot.common.transformations.camera import DEVICE_CAMERAS
from openpilot.common.transformations.model import get_warp_matrix
from openpilot.frogpilot.common import frogpilot_paths
from openpilot.frogpilot.common.frogpilot_variables import DRIVING_MODEL, get_frogpilot_toggles
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.selfdrive.modeld.get_model_metadata import make_metadata_dict
from openpilot.selfdrive.modeld.helpers import MODELD_PKL_PATH, MODELS_DIR, load_oob
from openpilot.selfdrive.modeld.modeld import ModelState, get_action_from_model
from openpilot.system.camerad.cameras.nv12_info import get_nv12_info

DRIVING_ONNX = MODELS_DIR / 'driving_supercombo.onnx'
CAMERA_RESOLUTIONS = [(1928, 1208), (1344, 760)]  # SConscript CAMERA_CONFIGS


@pytest.fixture(scope="module")
def bundle():
  assert os.path.isfile(get_manifest_path(MODELD_PKL_PATH)), f"{MODELD_PKL_PATH} is not built; run scons selfdrive/modeld"
  with open_file_chunked(MODELD_PKL_PATH) as f:
    return load_oob(f)


def test_bundle_is_compiled_from_the_tracked_onnx(bundle):
  onnx_metadata = make_metadata_dict(DRIVING_ONNX)
  for key in ('model_checkpoint', 'input_shapes', 'output_shapes', 'output_slices'):
    assert bundle['metadata'][key] == onnx_metadata[key], key
  for resolution in CAMERA_RESOLUTIONS:
    assert resolution in bundle, f"no warp compiled for camera {resolution}"


@pytest.mark.parametrize("cam_w, cam_h", CAMERA_RESOLUTIONS)
def test_model_state_outputs_match_metadata(bundle, cam_w, cam_h):
  metadata = bundle['metadata']
  model = ModelState(cam_w, cam_h)

  dc = DEVICE_CAMERAS[("tici", "ar0231")]
  bufs = {name: np.full(get_nv12_info(cam_w, cam_h)[3], 128, dtype=np.uint8) for name in model.vision_input_names}
  transforms = {'img': get_warp_matrix(np.zeros(3), dc.fcam.intrinsics, False).astype(np.float32),
                'big_img': get_warp_matrix(np.zeros(3), dc.ecam.intrinsics, True).astype(np.float32)}
  inputs = {'desire_pulse': np.zeros(ModelConstants.DESIRE_LEN, dtype=np.float32),
            'traffic_convention': np.array([1., 0.], dtype=np.float32),
            'action_t': np.array([0.3, 0.5], dtype=np.float32)}

  for _ in range(5):  # the model reads its history every 4th frame, so the fifth run reads state the first run wrote
    outputs = model.run(bufs, transforms, inputs)

  output_size = metadata['output_shapes']['outputs'][1]
  for name, s in metadata['output_slices'].items():
    assert name in outputs, name
    assert len(range(output_size)[s]) > 0, name
  assert max(s.stop for s in metadata['output_slices'].values() if s.stop is not None) <= output_size

  expected = {
    'plan': (1, ModelConstants.IDX_N, ModelConstants.PLAN_WIDTH),
    'lane_lines': (1, ModelConstants.NUM_LANE_LINES, ModelConstants.IDX_N, ModelConstants.LANE_LINES_WIDTH),
    'road_edges': (1, ModelConstants.NUM_ROAD_EDGES, ModelConstants.IDX_N, ModelConstants.LANE_LINES_WIDTH),
    'lead': (1, ModelConstants.LEAD_MHP_SELECTION, ModelConstants.LEAD_TRAJ_LEN, ModelConstants.LEAD_WIDTH),
    'desire_state': (1, ModelConstants.DESIRE_PRED_WIDTH),
    'action': (1, ModelConstants.ACTION_WIDTH),
    'hidden_state': (1, ModelConstants.FEATURE_LEN),
  }
  for name, shape in expected.items():
    assert outputs[name].shape == shape, name
  assert all(np.isfinite(v).all() for v in outputs.values())

  action = get_action_from_model(outputs, log.ModelDataV2.Action(), 0.3, 0.5, 10.0)
  assert math.isfinite(action.desiredAcceleration) and math.isfinite(action.desiredCurvature)


def test_model_selection_cannot_switch_the_driving_model():
  params = Params()
  params.put("DrivingModel", "duck-amigo")
  params.put_bool("ModelRandomizer", True)
  params.put("RandomizedModel", "duck-amigo")
  params.put_bool("AutomaticallyDownloadModels", True)

  toggles = get_frogpilot_toggles()

  assert (toggles.model, toggles.model_path) == (DRIVING_MODEL["id"], str(frogpilot_paths.STOCK_MODEL_PATH))
  assert not toggles.model_randomizer
  assert not toggles.automatically_download_models
  assert MODELD_PKL_PATH.parent == frogpilot_paths.STOCK_MODEL_PATH
