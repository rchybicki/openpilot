import os
import subprocess
import sys

import pytest


@pytest.mark.parametrize("constructor", ["Context()", "PubSocket()", "SubSocket()", "Poller()"])
def test_zmq_prefix_raises_without_aborting(constructor):
  # Import before setting the prefix, as the package creates a global Context.
  code = f'''
import os
import msgq
os.environ["OPENPILOT_PREFIX"] = "unsupported-test"
try:
  msgq.{constructor}
except RuntimeError as exc:
  assert "OPENPILOT_PREFIX not supported" in str(exc)
else:
  raise AssertionError("Expected an unsupported-backend error")
'''
  env = {k: v for k, v in os.environ.items() if k not in ("OPENPILOT_PREFIX", "CEREAL_FAKE")}
  env["ZMQ"] = "1"
  result = subprocess.run([sys.executable, "-c", code], env=env, capture_output=True, text=True, timeout=30)
  assert result.returncode == 0, result.stderr


@pytest.mark.skipif(sys.platform != "darwin", reason="Fake events are supported on Linux")
def test_macos_fake_event_raises_without_aborting():
  code = '''
import msgq
try:
  msgq.fake_event_handle("carState")
except RuntimeError as exc:
  assert "SocketEventHandle not supported on macOS" in str(exc)
else:
  raise AssertionError("Expected an unsupported-platform error")
'''
  env = {k: v for k, v in os.environ.items() if k != "OPENPILOT_PREFIX"}
  result = subprocess.run([sys.executable, "-c", code], env=env, capture_output=True, text=True, timeout=30)
  assert result.returncode == 0, result.stderr
