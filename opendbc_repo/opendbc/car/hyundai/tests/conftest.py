# Stub for openpilot.common.params (stopping redesign spec §8 / F18).
#
# opendbc.car.interfaces imports `from openpilot.common.params import Params`, which requires the
# compiled params_pyx extension that only exists after a scons build. The Params usage is incidental
# to the code under test, so on build-free checkouts we inject a no-op stand-in BEFORE anything
# imports the real module. On a scons-built env (device/CI) the real module imports fine and the
# stub is never installed.
#
# Test modules import this file explicitly (`import opendbc.car.hyundai.tests.conftest`) so the stub
# also works under `pytest --noconftest` — the only supported local invocation per the spec.
import sys
import types


def install_params_stub():
  if 'openpilot.common.params' in sys.modules:
    return
  try:
    import openpilot.common.params  # noqa: F401
  except ImportError:
    mod = types.ModuleType('openpilot.common.params')

    class _StubParams:
      def __init__(self, *args, **kwargs):
        pass

      def __getattr__(self, name):
        return lambda *args, **kwargs: None

    class UnknownKeyName(Exception):
      pass

    mod.Params = _StubParams
    mod.UnknownKeyName = UnknownKeyName
    mod.ParamKeyType = types.SimpleNamespace()
    mod.ParamKeyFlag = types.SimpleNamespace()
    sys.modules['openpilot.common.params'] = mod


install_params_stub()
