import contextlib
import datetime
from types import SimpleNamespace

import pytest

from openpilot.frogpilot import frogpilot_process
from openpilot.frogpilot.system.model_manager import ModelManager
from openpilot.frogpilot.system.speed_limit_reader_models import SpeedLimitReaderModels


class Recorder:
  def __init__(self):
    self.calls = []
    self.download_lock = contextlib.nullcontext()

  def __getattr__(self, name):
    return lambda *args, **kwargs: self.calls.append((name, args))


@pytest.mark.parametrize("metered", [False, True])
def test_update_checks_on_day_one(monkeypatch, metered):
  # day one: the driving model is pinned and the sign reader is off, so the periodic checks neither fetch the model list nor the
  # reader manifest and models; the full update check and the theme check keep running
  monkeypatch.setattr(frogpilot_process.frogpilot_utilities, "is_url_pingable", lambda url: True)
  fetches = []
  monkeypatch.setattr(ModelManager, "update_models", lambda *args, **kwargs: fetches.append("update_models"))
  monkeypatch.setattr(SpeedLimitReaderModels, "update", lambda *args, **kwargs: fetches.append("reader_models"))
  theme_manager, thread_manager = Recorder(), Recorder()
  sm = {"deviceState": SimpleNamespace(networkMetered=metered)}
  toggles = SimpleNamespace(automatic_updates=True, automatically_download_models=False, show_speed_limits=True, speed_limit_controller=True,
                            speed_limit_filler=False)

  frogpilot_process.update_checks(datetime.datetime.now(datetime.UTC), theme_manager, thread_manager, sm, None, None, toggles)

  assert fetches == []
  assert thread_manager.calls == [("run_in_thread", (frogpilot_process.frogpilot_functions.update_openpilot,))]
  assert theme_manager.calls == ([] if metered else [("update_themes", (False,))])
