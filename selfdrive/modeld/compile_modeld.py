#!/usr/bin/env python3
import argparse
import atexit
import math
import os
import shutil
import tempfile
import time
from functools import partial
from collections import namedtuple

import numpy as np

from openpilot.selfdrive.modeld.helpers import dump_oob, load_oob

def _patch_tinygrad_fetch_fw():
  import hashlib
  import pathlib
  import zstandard
  from tinygrad import helpers
  _orig = helpers.fetch_fw
  def fetch_fw(path, name, sha256):
    p = pathlib.Path(f"/lib/firmware/{path}/{name}.zst")
    if p.is_file():
      blob = zstandard.ZstdDecompressor().stream_reader(p.read_bytes()).read()
      if hashlib.sha256(blob).hexdigest() == sha256:
        return blob
    return _orig(path, name, sha256)
  helpers.fetch_fw = fetch_fw
_patch_tinygrad_fetch_fw()

from tinygrad.tensor import Tensor
from tinygrad.helpers import Context
from tinygrad.device import Device
from tinygrad.engine.jit import TinyJit


NV12Frame = namedtuple("NV12Frame", ['width', 'height', 'stride', 'y_height', 'uv_height', 'size'])
WARP_INPUTS = ['tfm', 'big_tfm']
STATE_INPUTS = ['state_img_q', 'state_desire_q', 'state_feat_q']
POLICY_INPUTS = [*STATE_INPUTS, 'packed_npy_inputs']

UV_SCALE_MATRIX = np.array([[0.5, 0, 0], [0, 0.5, 0], [0, 0, 1]], dtype=np.float32)
UV_SCALE_MATRIX_INV = np.linalg.inv(UV_SCALE_MATRIX)

WARP_DEV = os.getenv('WARP_DEV')


def make_random_images(keys, shape, device=None):
  return {k: Tensor.randint(shape, low=0, high=256, dtype='uint8', device=device).realize() for k in keys}


def warp_perspective_tinygrad(src_flat, M_inv, dst_shape, src_shape, stride_pad, border_fill_val=None):
  w_dst, h_dst = dst_shape
  h_src, w_src = src_shape

  x = Tensor.arange(w_dst).reshape(1, w_dst).expand(h_dst, w_dst).reshape(-1)
  y = Tensor.arange(h_dst).reshape(h_dst, 1).expand(h_dst, w_dst).reshape(-1)

  # inline 3x3 matmul as elementwise to avoid reduce op (enables fusion with gather)
  src_x = M_inv[0, 0] * x + M_inv[0, 1] * y + M_inv[0, 2]
  src_y = M_inv[1, 0] * x + M_inv[1, 1] * y + M_inv[1, 2]
  src_w = M_inv[2, 0] * x + M_inv[2, 1] * y + M_inv[2, 2]

  src_x = src_x / src_w
  src_y = src_y / src_w

  x_round = Tensor.round(src_x)
  y_round = Tensor.round(src_y)
  x_nn_clipped = x_round.clip(0, w_src - 1).cast('int')
  y_nn_clipped = y_round.clip(0, h_src - 1).cast('int')
  idx = y_nn_clipped * (w_src + stride_pad) + x_nn_clipped
  sampled = src_flat[idx]

  if border_fill_val is None:
    return sampled

  in_bounds = ((x_round >= 0) & (x_round <= w_src - 1) &
               (y_round >= 0) & (y_round <= h_src - 1)).cast(sampled.dtype)
  return sampled * in_bounds + Tensor(border_fill_val, dtype=sampled.dtype) * (1 - in_bounds)


def frames_to_tensor(frames):
  H = (frames.shape[0] * 2) // 3
  W = frames.shape[1]
  in_img1 = Tensor.cat(frames[0:H:2, 0::2],
                       frames[1:H:2, 0::2],
                       frames[0:H:2, 1::2],
                       frames[1:H:2, 1::2],
                       frames[H:H+H//4].reshape((H//2, W//2)),
                       frames[H+H//4:H+H//2].reshape((H//2, W//2)), dim=0).reshape((6, H//2, W//2))
  return in_img1


def make_frame_prepare(nv12: NV12Frame, model_w, model_h):
  cam_w, cam_h, stride, y_height, uv_height, _ = nv12
  uv_offset = stride * y_height
  stride_pad = stride - cam_w

  def frame_prepare_tinygrad(input_frame, M_inv):
    # UV_SCALE @ M_inv @ UV_SCALE_INV simplifies to elementwise scaling
    M_inv_uv = M_inv * Tensor([[1.0, 1.0, 0.5], [1.0, 1.0, 0.5], [2.0, 2.0, 1.0]], device=WARP_DEV)
    # deinterleave NV12 UV plane (UVUV... -> separate U, V)
    uv = input_frame[uv_offset:uv_offset + uv_height * stride].reshape(uv_height, stride)
    with Context(SPLIT_REDUCEOP=0):
      y = warp_perspective_tinygrad(input_frame[:cam_h*stride],
                                    M_inv, (model_w, model_h),
                                    (cam_h, cam_w), stride_pad).realize()
      u = warp_perspective_tinygrad(uv[:cam_h//2, :cam_w:2].flatten(),
                                    M_inv_uv, (model_w//2, model_h//2),
                                    (cam_h//2, cam_w//2), 0).realize()
      v = warp_perspective_tinygrad(uv[:cam_h//2, 1:cam_w:2].flatten(),
                                    M_inv_uv, (model_w//2, model_h//2),
                                    (cam_h//2, cam_w//2), 0).realize()
    yuv = y.cat(u).cat(v).reshape((model_h * 3 // 2, model_w))
    tensor = frames_to_tensor(yuv)
    return tensor
  return frame_prepare_tinygrad


def make_warp_input_queues():
  npy = {
    'tfm': np.zeros((3, 3), dtype=np.float32),
    'big_tfm': np.zeros((3, 3), dtype=np.float32),
  }
  return {k: Tensor(v, device='NPY').realize() for k, v in npy.items()}, npy


def get_policy_npy_shapes(input_shapes):
  shapes = {k: tuple(input_shapes[k]) for k in ('desire', 'traffic_convention', 'action_t')}
  return shapes, [math.prod(s) for s in shapes.values()]


def make_input_queues(input_shapes, device):
  input_queues, npy = make_warp_input_queues()
  shapes, sizes = get_policy_npy_shapes(input_shapes)
  packed_npy_inputs = np.zeros(sum(sizes), dtype=np.float32)
  # views into the packed inputs, to be refilled at runtime
  npy.update({k: v.reshape(s) for (k, s), v in zip(shapes.items(), np.split(packed_npy_inputs, np.cumsum(sizes[:-1])), strict=True)})
  # the model's recurrent state: it reads state_* and returns next_state_*, which run_policy writes back
  input_queues.update({k: Tensor(np.zeros(input_shapes[k], dtype=np.uint8 if 'img' in k else np.float32), device=device).contiguous().realize()
                       for k in STATE_INPUTS})
  input_queues['packed_npy_inputs'] = Tensor(packed_npy_inputs, device='NPY').realize()
  return input_queues, npy


def make_warp(nv12, model_w, model_h):
  frame_prepare = make_frame_prepare(nv12, model_w, model_h)

  def warp(tfm, big_tfm, frame, big_frame):
    tfm = tfm.to(WARP_DEV)
    big_tfm = big_tfm.to(WARP_DEV)
    Tensor.realize(tfm, big_tfm)

    warped_frame = frame_prepare(frame, tfm).unsqueeze(0)
    warped_big_frame = frame_prepare(big_frame, big_tfm).unsqueeze(0)
    return Tensor.cat(warped_frame, warped_big_frame)

  return warp


def make_run_policy(model_runner, model_metadata):
  npy_shapes, npy_sizes = get_policy_npy_shapes(model_metadata['input_shapes'])

  def run_policy(warped, state_img_q, state_desire_q, state_feat_q, packed_npy_inputs):
    packed_npy_inputs = packed_npy_inputs.to(Device.DEFAULT)
    warped = warped.to(Device.DEFAULT)
    Tensor.realize(packed_npy_inputs, warped)

    desire, traffic_convention, action_t = (t.reshape(s) for t, s in zip(packed_npy_inputs.split(npy_sizes), npy_shapes.values(), strict=True))
    states = {'state_img_q': state_img_q, 'state_desire_q': state_desire_q, 'state_feat_q': state_feat_q}
    outs = model_runner({'new_img': warped, 'desire': desire, 'traffic_convention': traffic_convention, 'action_t': action_t, **states})

    out = outs['outputs'].cast('float32').contiguous()
    next_states = [outs[f'next_{k}'].cast(v.dtype).contiguous() for k, v in states.items()]
    # every read of the old state must finish before it is overwritten
    Tensor.realize(out, *next_states)
    Tensor.realize(*[v.assign(n) for v, n in zip(states.values(), next_states, strict=True)])
    return out,
  return run_policy


def compile_jit(jit, make_random_inputs, input_keys, make_queues):
  SEED = 42
  def random_inputs_run(fn, seed, test_val=None, test_buffers=None, expect_match=True):
    input_queues, npy = make_queues()
    rng = np.random.default_rng(seed)
    Tensor.manual_seed(seed)

    testing = test_val is not None or test_buffers is not None
    n_runs = 1 if testing else 3

    for i in range(n_runs):
      for v in npy.values():
        v[:] = rng.standard_normal(v.shape).astype(v.dtype)
      Device.default.synchronize()
      random_inputs = make_random_inputs()
      st = time.perf_counter()
      outs = fn(**{k: input_queues[k] for k in input_keys}, **random_inputs)
      mt = time.perf_counter()
      Device.default.synchronize()
      et = time.perf_counter()
      print(f"  [{i+1}/{n_runs}] enqueue {(mt-st)*1e3:6.2f} ms -- total {(et-st)*1e3:6.2f} ms")

      if i == 0:
        val = [np.copy(v.numpy()) for v in outs]
        buffers = [np.copy(v.numpy().copy()) for v in input_queues.values()]

    if test_val is not None:
      match = all(np.array_equal(a, b) for a, b in zip(val, test_val, strict=True))
      assert match == expect_match, f"outputs {'differ from' if expect_match else 'match'} baseline (seed={seed})"
    if test_buffers is not None:
      match = all(np.array_equal(a, b) for a, b in zip(buffers, test_buffers, strict=True))
      assert match == expect_match, f"buffers {'differ from' if expect_match else 'match'} baseline (seed={seed})"
    return val, buffers

  print('capture + replay')
  test_val, test_buffers = random_inputs_run(jit, SEED)
  print('pickle round trip')
  with tempfile.TemporaryFile(dir=".") as f:
    dump_oob(jit, f)
    f.seek(0)
    jit = load_oob(f)
  random_inputs_run(jit, SEED, test_val, test_buffers, expect_match=True)
  random_inputs_run(jit, SEED+1, test_val, test_buffers, expect_match=False)
  return jit


def _parse_size(s):
  w, h = s.lower().split('x')
  return int(w), int(h)


def read_file_chunked_to_disk(path):
  from openpilot.common.file_chunker import open_file_chunked
  tmp_path = f'{path}.unchunked'
  with open(tmp_path, 'wb') as f, open_file_chunked(path) as src:
    shutil.copyfileobj(src, f)
  atexit.register(lambda: os.path.exists(tmp_path) and os.remove(tmp_path))
  return tmp_path


if __name__ == "__main__":
  from tinygrad.nn.onnx import OnnxRunner
  from openpilot.system.camerad.cameras.nv12_info import get_nv12_info
  from openpilot.selfdrive.modeld.get_model_metadata import make_metadata_dict
  p = argparse.ArgumentParser()
  p.add_argument('--model-size', type=_parse_size, required=True, help='model input WxH')
  p.add_argument('--camera-resolutions', type=_parse_size, nargs='+', required=True,
                 help='camera resolutions WxH (one or more)')
  p.add_argument('--onnx', required=True)
  p.add_argument('--output', required=True)
  args = p.parse_args()

  model_path = read_file_chunked_to_disk(args.onnx)
  model_w, model_h = args.model_size

  model_runner = OnnxRunner(model_path)
  out = {'metadata': make_metadata_dict(model_path)}

  run_policy_jit = TinyJit(make_run_policy(model_runner, out['metadata']), prune=True)

  make_policy_queues = partial(make_input_queues, out['metadata']['input_shapes'], Device.DEFAULT)
  make_random_model_inputs = partial(make_random_images, keys=['warped'], shape=out['metadata']['input_shapes']['new_img'], device=WARP_DEV)
  out['run_policy'] = compile_jit(run_policy_jit, make_random_model_inputs, POLICY_INPUTS,
                                  make_policy_queues)

  for cam_w, cam_h in args.camera_resolutions:
    nv12 = NV12Frame(cam_w, cam_h, *get_nv12_info(cam_w, cam_h))
    make_random_warp_inputs = partial(make_random_images, keys=['frame', 'big_frame'], shape=nv12.size, device=WARP_DEV)
    warp = TinyJit(make_warp(nv12, model_w, model_h), prune=True)
    out[(cam_w,cam_h)] = compile_jit(warp, make_random_warp_inputs, WARP_INPUTS, make_warp_input_queues)

  with open(args.output, "wb") as f:
    dump_oob(out, f)
  print(f"Saved JITs to {args.output} ({os.path.getsize(args.output) / 1e6:.2f} MB)")
