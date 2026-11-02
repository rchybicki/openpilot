import sys
import time

import pytest

from openpilot.selfdrive.test.helpers import with_processes


@pytest.mark.skipif(sys.platform == "darwin", reason="The UI process runs under namespaced msgq (OPENPILOT_PREFIX); macOS uses ZMQ")
@with_processes(["ui"])
def test_raylib_ui():
  """Test initialization of the UI widgets is successful."""
  time.sleep(1)
