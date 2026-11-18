import os

import pytest

from openpilot.common import file_chunker
from openpilot.common.file_chunker import chunk_file, get_chunk_targets, read_file_chunked

CHUNK_SIZE = 1024
FILE_SIZE = 5 * CHUNK_SIZE + 7


@pytest.mark.parametrize("estimate", [1, 2 * CHUNK_SIZE, FILE_SIZE, 20 * CHUNK_SIZE])
def test_chunked_file_reads_back_whatever_the_estimate(tmp_path, monkeypatch, estimate):
  monkeypatch.setattr(file_chunker, 'CHUNK_SIZE', CHUNK_SIZE)
  path = str(tmp_path / 'model.pkl')
  data = os.urandom(FILE_SIZE)
  with open(path, 'wb') as f:
    f.write(data)

  targets = get_chunk_targets(path, estimate)
  chunk_file(path, targets)

  assert all(os.path.isfile(t) for t in targets)
  assert not os.path.exists(path)
  assert read_file_chunked(path) == data
