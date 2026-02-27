import pytest

from openpilot.selfdrive.controls.lib.longcontrol import LongControl


class DummyCruiseState:
  def __init__(self, standstill: bool = False) -> None:
    self.standstill = standstill


class DummyCarState:
  def __init__(
    self,
    v_ego: float,
    a_ego: float,
    brake_pressed: bool = False,
    standstill: bool = False,
    cruise_standstill: bool = False,
  ) -> None:
    self.vEgo = v_ego
    self.aEgo = a_ego
    self.brakePressed = brake_pressed
    self.standstill = standstill
    self.cruiseState = DummyCruiseState(standstill=cruise_standstill)


class DummyLongitudinalTuning:
  def __init__(self) -> None:
    self.kpBP = [0.0]
    self.kpV = [1.0]
    self.kiBP = [0.0]
    self.kiV = [0.0]


class DummyCarParams:
  def __init__(self) -> None:
    self.longitudinalTuning = DummyLongitudinalTuning()
    self.enableGasInterceptor = False
    self.startingState = False
    self.stoppingVbp = [0.01, 0.2, 0.5]
    self.stopAccel = -1.0


class DummyFrogPilotToggles:
  def __init__(self) -> None:
    self.vEgoStarting = 0.1
    self.human_acceleration = False
    self.startAccel = 1.0


def test_longcontrol_blocks_fast_release_without_standstill_when_stop_intent_recent() -> None:
  cp = DummyCarParams()
  toggles = DummyFrogPilotToggles()
  lc = LongControl(cp)

  cs = DummyCarState(v_ego=0.2, a_ego=-0.2, standstill=False, cruise_standstill=False)
  accel_limits = (-3.0, 2.0)

  lc.last_output_accel = -0.20
  lc.update(active=True, CS=cs, a_target=-0.2, should_stop=True, accel_limits=accel_limits, frogpilot_toggles=toggles)

  lc.last_output_accel = -0.20
  out = lc.update(active=True, CS=cs, a_target=1.0, should_stop=False, accel_limits=accel_limits, frogpilot_toggles=toggles)

  # Without the stop-intent guard, allow_fast_release would be active (resume intent) and the low-speed slew would permit a +0.026 step -> -0.174.
  # With the guard active, we use the default low-speed release step (0.004 at v=0.2) -> -0.196.
  assert out == pytest.approx(-0.196, abs=1e-12)

