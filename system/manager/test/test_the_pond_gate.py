from types import SimpleNamespace

import pytest

from cereal import car
from openpilot.system.manager.process_config import managed_processes


@pytest.mark.parametrize("started", [False, True])
def test_the_pond_stays_stopped_on_day_one(started):
  # off on day one (FrogPilot refresh DECISIONS.md, network rows); the entry stays so the process list matches upstream
  assert not managed_processes["the_pond"].should_run(started, None, car.CarParams(brand="hyundai"), SimpleNamespace())
