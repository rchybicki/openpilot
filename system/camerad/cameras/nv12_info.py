"""Python version of system/camerad/cameras/nv12_info.h."""


def align(val: int, alignment: int) -> int:
  return ((val + alignment - 1) // alignment) * alignment


def get_nv12_info(width: int, height: int) -> tuple[int, int, int, int]:
  stride = align(width, 128)
  y_height = align(height, 32)
  uv_height = align(height // 2, 16)

  y_plane = stride * y_height
  uv_plane = stride * uv_height + 4096
  size = y_plane + uv_plane + max(16 * 1024, 8 * stride)
  size = align(size, 4096)
  size += align(width, 512) * 512
  size = align(size, 4096)

  return stride, y_height, uv_height, size
