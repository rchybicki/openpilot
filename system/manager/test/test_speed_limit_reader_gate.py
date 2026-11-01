from types import SimpleNamespace

import pytest

from cereal import car
from openpilot.system.manager.process_config import managed_processes


@pytest.mark.parametrize("show_speed_limits, speed_limit_filler", [(False, False), (True, False), (True, True)])
@pytest.mark.parametrize("started", [False, True])
def test_speed_limit_reader_stays_stopped_while_slc_runs(started, show_speed_limits, speed_limit_filler):
  # SLC runs inside frogpilot_process (FrogPilotPlanner -> FrogPilotVCruise.slc); the camera sign reader is off on day one
  toggles = SimpleNamespace(speed_limit_controller=True, show_speed_limits=show_speed_limits, speed_limit_filler=speed_limit_filler)
  CP = car.CarParams(brand="hyundai")
  assert managed_processes["frogpilot_process"].should_run(started, None, CP, toggles)
  assert not managed_processes["speed_limit_reader"].should_run(started, None, CP, toggles)
