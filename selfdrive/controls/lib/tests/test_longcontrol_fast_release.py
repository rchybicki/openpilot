import pytest

from opendbc.car.hyundai.values import CAR as HYUNDAI_CAR
from openpilot.selfdrive.controls.lib.longcontrol import (
  LongControl,
  LongCtrlState,
  should_apply_stop_entry_handoff_soften,
  should_apply_stop_target_approach_mode,
  should_enter_stop_target_mode,
  should_hold_stop_target_mode,
  stop_entry_handoff_accel_cap,
  stop_target_approach_accel_cap,
)


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
  def __init__(self, car_fingerprint=HYUNDAI_CAR.HYUNDAI_SANTA_FE_HEV_2022) -> None:
    self.longitudinalTuning = DummyLongitudinalTuning()
    self.carFingerprint = car_fingerprint
    self.enableGasInterceptor = False
    self.startingState = False
    self.stoppingVbp = [0.01, 0.2, 0.5]
    self.stopAccel = -1.0


class DummyFrogPilotToggles:
  def __init__(self) -> None:
    self.vEgoStarting = 0.1
    self.human_acceleration = False
    self.startAccel = 1.0


class SpyStoppingController:
  def __init__(self) -> None:
    self.distance_to_stop_target_m = None

  def reset(self) -> None:
    return None

  def update(self, **kwargs):
    self.distance_to_stop_target_m = kwargs.get("distance_to_stop_target_m")
    return type("StopResult", (), {"output_accel": kwargs["output_accel"], "release_lock_active": False})()


class FixedStoppingController:
  def __init__(self, output_accel: float) -> None:
    self.output_accel = output_accel

  def reset(self) -> None:
    return None

  def update(self, **kwargs):
    return type("StopResult", (), {"output_accel": self.output_accel, "release_lock_active": False})()


def test_longcontrol_blocks_fast_release_without_standstill_when_stop_intent_recent() -> None:
  cp = DummyCarParams()
  toggles = DummyFrogPilotToggles()
  lc = LongControl(cp)

  cs = DummyCarState(v_ego=0.2, a_ego=-0.2, standstill=False, cruise_standstill=False)
  accel_limits = (-3.0, 2.0)

  lc.last_output_accel = -0.20
  lc.update(active=True, CS=cs, a_target=-0.2, should_stop=True, distance_to_stop_target_m=-1.0, accel_limits=accel_limits, frogpilot_toggles=toggles)

  lc.last_output_accel = -0.20
  out = lc.update(active=True, CS=cs, a_target=1.0, should_stop=False, distance_to_stop_target_m=-1.0, accel_limits=accel_limits, frogpilot_toggles=toggles)

  # Without the stop-intent guard, allow_fast_release would be active (resume intent) and the low-speed slew would permit a +0.026 step -> -0.174.
  # With the guard active, we use the default low-speed release step (0.004 at v=0.2) -> -0.196.
  assert out == pytest.approx(-0.196, abs=1e-12)


def test_longcontrol_disables_human_acceleration_takeoff_in_experimental_mode() -> None:
  cp = DummyCarParams()
  toggles = DummyFrogPilotToggles()
  toggles.human_acceleration = True
  toggles.startAccel = 0.6
  lc = LongControl(cp)
  lc.long_control_state = LongCtrlState.starting

  out = lc.update(
    active=True,
    CS=DummyCarState(v_ego=0.2, a_ego=0.0, standstill=False, cruise_standstill=False),
    a_target=1.2,
    should_stop=False,
    distance_to_stop_target_m=-1.0,
    accel_limits=(-3.0, 2.0),
    frogpilot_toggles=toggles,
    experimental_mode=True,
  )

  assert out == pytest.approx(0.6, abs=1e-12)


def test_longcontrol_releases_standstill_hold_for_departing_lead_even_if_should_stop_is_still_latched() -> None:
  cp = DummyCarParams()
  cp.startingState = True
  toggles = DummyFrogPilotToggles()
  toggles.human_acceleration = True
  lc = LongControl(cp)
  lc.stopping_controller = FixedStoppingController(output_accel=-1.06)
  lc.long_control_state = LongCtrlState.stopping
  lc.last_output_accel = -1.06
  lc.time_since_standstill_s = 0.0
  lc.time_since_stop_intent_s = 0.0

  out = lc.update(
    active=True,
    CS=DummyCarState(v_ego=0.01, a_ego=0.0, standstill=True, cruise_standstill=False),
    a_target=0.0,
    should_stop=True,
    distance_to_stop_target_m=-1.0,
    accel_limits=(-3.0, 2.0),
    frogpilot_toggles=toggles,
    experimental_mode=True,
    lead_status=True,
    lead_v=0.8,
    lead_d_rel=5.8,
    force_coast=False,
  )

  assert lc.long_control_state == LongCtrlState.starting
  assert out == pytest.approx(-1.0302, abs=1e-12)
  assert out > -1.06


def test_longcontrol_forwards_distance_to_stop_target_into_stopping_controller() -> None:
  cp = DummyCarParams()
  toggles = DummyFrogPilotToggles()
  lc = LongControl(cp)
  spy = SpyStoppingController()
  lc.stopping_controller = spy
  lc.long_control_state = LongCtrlState.stopping

  cs = DummyCarState(v_ego=0.35, a_ego=-0.25, standstill=False, cruise_standstill=False)
  accel_limits = (-3.0, 2.0)

  lc.last_output_accel = -0.20
  lc.update(
    active=True,
    CS=cs,
    a_target=-0.2,
    should_stop=True,
    distance_to_stop_target_m=1.75,
    accel_limits=accel_limits,
    frogpilot_toggles=toggles,
  )

  assert spy.distance_to_stop_target_m == pytest.approx(1.75, abs=1e-12)


def test_should_enter_stop_target_mode_only_when_close_and_braking() -> None:
  assert should_enter_stop_target_mode(v_ego=2.2, a_target=-0.20, distance_to_stop_target_m=0.9)
  assert not should_enter_stop_target_mode(v_ego=2.2, a_target=-0.05, distance_to_stop_target_m=0.9)
  assert not should_enter_stop_target_mode(v_ego=2.2, a_target=-0.20, distance_to_stop_target_m=3.5)
  assert not should_enter_stop_target_mode(v_ego=2.2, a_target=-0.20, distance_to_stop_target_m=0.18)


def test_should_apply_stop_target_approach_mode_only_in_midband() -> None:
  assert should_apply_stop_target_approach_mode(v_ego=3.2, a_target=-0.10, distance_to_stop_target_m=1.8)
  assert not should_apply_stop_target_approach_mode(v_ego=3.2, a_target=-0.03, distance_to_stop_target_m=1.8)
  assert not should_apply_stop_target_approach_mode(v_ego=3.2, a_target=-0.10, distance_to_stop_target_m=4.0)
  assert not should_apply_stop_target_approach_mode(v_ego=3.2, a_target=-0.20, distance_to_stop_target_m=0.8)


def test_stop_target_approach_mode_can_own_tiny_meaningful_target_before_full_stop_mode() -> None:
  assert should_apply_stop_target_approach_mode(v_ego=4.2, a_target=-1.20, distance_to_stop_target_m=0.15)
  assert not should_enter_stop_target_mode(v_ego=4.2, a_target=-1.20, distance_to_stop_target_m=0.15)
  assert should_enter_stop_target_mode(v_ego=4.2, a_target=-1.20, distance_to_stop_target_m=0.23)
  assert should_enter_stop_target_mode(v_ego=3.95, a_target=-1.27, distance_to_stop_target_m=0.61)


def test_stop_target_mode_waits_for_more_than_tiny_target_when_nonurgent() -> None:
  assert not should_enter_stop_target_mode(v_ego=2.30, a_target=-0.335, distance_to_stop_target_m=0.28)
  assert should_apply_stop_target_approach_mode(v_ego=2.30, a_target=-0.335, distance_to_stop_target_m=0.28)


def test_stop_target_mode_holds_once_target_is_stable_and_braking_persists() -> None:
  assert should_hold_stop_target_mode(v_ego=2.13, a_target=-0.283, distance_to_stop_target_m=1.68)
  assert not should_hold_stop_target_mode(v_ego=2.13, a_target=-0.05, distance_to_stop_target_m=1.68)
  assert not should_hold_stop_target_mode(v_ego=2.13, a_target=-0.283, distance_to_stop_target_m=2.20)


def test_stop_target_approach_accel_cap_prefers_more_brake_when_closer_or_faster() -> None:
  far_cap = stop_target_approach_accel_cap(v_ego=3.0, distance_to_stop_target_m=2.8)
  close_cap = stop_target_approach_accel_cap(v_ego=3.0, distance_to_stop_target_m=1.2)
  fast_cap = stop_target_approach_accel_cap(v_ego=5.0, distance_to_stop_target_m=2.8)

  assert close_cap < far_cap
  assert fast_cap < far_cap


def test_should_apply_stop_entry_handoff_soften_only_for_non_urgent_deep_inherited_brake() -> None:
  assert should_apply_stop_entry_handoff_soften(
    v_ego=1.0,
    a_ego=-0.75,
    a_target=-0.30,
    last_output_accel=-0.70,
    distance_to_stop_target_m=0.9,
  )
  assert not should_apply_stop_entry_handoff_soften(
    v_ego=1.0,
    a_ego=-0.75,
    a_target=-0.70,
    last_output_accel=-0.70,
    distance_to_stop_target_m=0.9,
  )
  assert not should_apply_stop_entry_handoff_soften(
    v_ego=1.0,
    a_ego=-0.75,
    a_target=-0.30,
    last_output_accel=-0.70,
    distance_to_stop_target_m=0.10,
  )


def test_should_apply_stop_entry_handoff_soften_extends_to_moderate_speed_non_urgent_handoff() -> None:
  assert should_apply_stop_entry_handoff_soften(
    v_ego=1.85,
    a_ego=-0.72,
    a_target=-0.42,
    last_output_accel=-0.74,
    distance_to_stop_target_m=1.10,
  )
  assert not should_apply_stop_entry_handoff_soften(
    v_ego=1.85,
    a_ego=-0.72,
    a_target=-0.68,
    last_output_accel=-0.74,
    distance_to_stop_target_m=1.10,
  )


def test_stop_entry_handoff_accel_cap_prefers_gentler_handoff_when_target_is_farther() -> None:
  near_cap = stop_entry_handoff_accel_cap(v_ego=1.0, distance_to_stop_target_m=0.25)
  far_cap = stop_entry_handoff_accel_cap(v_ego=1.0, distance_to_stop_target_m=1.2)
  assert far_cap > near_cap


def test_stop_entry_handoff_accel_cap_allows_moderate_speed_soften_without_forcing_urgent_close_stop_shape() -> None:
  moderate_speed_far = stop_entry_handoff_accel_cap(v_ego=1.85, distance_to_stop_target_m=1.10)
  moderate_speed_close = stop_entry_handoff_accel_cap(v_ego=1.85, distance_to_stop_target_m=0.30)
  assert moderate_speed_far > moderate_speed_close
  assert moderate_speed_far > -0.70


def test_longcontrol_enters_stopping_early_for_close_stopped_lead_target() -> None:
  cp = DummyCarParams()
  toggles = DummyFrogPilotToggles()
  lc = LongControl(cp)
  spy = SpyStoppingController()
  lc.stopping_controller = spy
  lc.long_control_state = LongCtrlState.pid

  cs = DummyCarState(v_ego=2.2, a_ego=-0.15, standstill=False, cruise_standstill=False)
  accel_limits = (-3.0, 2.0)

  lc.last_output_accel = -0.18
  lc.update(
    active=True,
    CS=cs,
    a_target=-0.25,
    should_stop=False,
    distance_to_stop_target_m=0.9,
    accel_limits=accel_limits,
    frogpilot_toggles=toggles,
  )

  assert lc.long_control_state == LongCtrlState.stopping
  assert spy.distance_to_stop_target_m == pytest.approx(0.9, abs=1e-12)


def test_longcontrol_stays_in_pid_when_stopped_lead_target_is_still_far() -> None:
  cp = DummyCarParams()
  toggles = DummyFrogPilotToggles()
  lc = LongControl(cp)
  lc.long_control_state = LongCtrlState.pid

  cs = DummyCarState(v_ego=2.2, a_ego=-0.15, standstill=False, cruise_standstill=False)
  accel_limits = (-3.0, 2.0)

  lc.last_output_accel = -0.18
  lc.update(
    active=True,
    CS=cs,
    a_target=-0.25,
    should_stop=False,
    distance_to_stop_target_m=3.5,
    accel_limits=accel_limits,
    frogpilot_toggles=toggles,
  )

  assert lc.long_control_state == LongCtrlState.pid


def test_longcontrol_plain_pid_braking_stays_close_to_model_request() -> None:
  cp = DummyCarParams()
  toggles = DummyFrogPilotToggles()
  lc = LongControl(cp)
  lc.long_control_state = LongCtrlState.pid

  cs = DummyCarState(v_ego=2.0, a_ego=-0.3, standstill=False, cruise_standstill=False)
  accel_limits = (-3.0, 2.0)

  out = lc.update(
    active=True,
    CS=cs,
    a_target=-0.8,
    should_stop=False,
    distance_to_stop_target_m=-1.0,
    accel_limits=accel_limits,
    frogpilot_toggles=toggles,
  )

  # Raw PID would request -1.3 here; keep the plain braking path within a small fixed margin of the model request instead.
  assert out == pytest.approx(-0.88, abs=1e-12)
  assert out > -1.0
  assert lc.pid.i > 0.0


def test_longcontrol_plain_pid_braking_alignment_is_santa_fe_only() -> None:
  cp = DummyCarParams(car_fingerprint=HYUNDAI_CAR.HYUNDAI_ELANTRA_2021)
  toggles = DummyFrogPilotToggles()
  lc = LongControl(cp)
  lc.long_control_state = LongCtrlState.pid

  out = lc.update(
    active=True,
    CS=DummyCarState(v_ego=2.0, a_ego=-0.3, standstill=False, cruise_standstill=False),
    a_target=-0.8,
    should_stop=False,
    distance_to_stop_target_m=-1.0,
    accel_limits=(-3.0, 2.0),
    frogpilot_toggles=toggles,
  )

  assert out == pytest.approx(-1.3, abs=1e-12)
  assert lc.pid.i == pytest.approx(0.0, abs=1e-12)


def test_longcontrol_softly_brakes_in_stopped_lead_approach_band_before_stop_mode() -> None:
  cp = DummyCarParams()
  toggles = DummyFrogPilotToggles()
  accel_limits = (-3.0, 2.0)
  cs = DummyCarState(v_ego=3.2, a_ego=-0.12, standstill=False, cruise_standstill=False)

  baseline = LongControl(cp)
  baseline.long_control_state = LongCtrlState.pid
  baseline.last_output_accel = -0.05
  out_without = baseline.update(
    active=True,
    CS=cs,
    a_target=-0.08,
    should_stop=False,
    distance_to_stop_target_m=-1.0,
    accel_limits=accel_limits,
    frogpilot_toggles=toggles,
  )

  with_target = LongControl(cp)
  with_target.long_control_state = LongCtrlState.pid
  with_target.last_output_accel = -0.05
  out_with = with_target.update(
    active=True,
    CS=cs,
    a_target=-0.08,
    should_stop=False,
    distance_to_stop_target_m=1.8,
    accel_limits=accel_limits,
    frogpilot_toggles=toggles,
  )

  assert with_target.long_control_state == LongCtrlState.pid
  assert out_with < out_without - 1e-6
  assert out_with == pytest.approx(stop_target_approach_accel_cap(cs.vEgo, 1.8), abs=1e-12)


def test_longcontrol_softens_deep_inherited_brake_on_first_stop_handoff() -> None:
  cp = DummyCarParams()
  toggles = DummyFrogPilotToggles()
  accel_limits = (-3.0, 2.0)
  cs = DummyCarState(v_ego=1.03, a_ego=-0.79, standstill=False, cruise_standstill=False)

  lc = LongControl(cp)
  lc.long_control_state = LongCtrlState.pid
  lc.last_output_accel = -0.77
  out = lc.update(
    active=True,
    CS=cs,
    a_target=-0.35,
    should_stop=True,
    distance_to_stop_target_m=0.9,
    accel_limits=accel_limits,
    frogpilot_toggles=toggles,
  )

  assert lc.long_control_state == LongCtrlState.stopping
  assert out > -0.70


def test_longcontrol_does_not_soften_urgent_close_stop_handoff() -> None:
  cp = DummyCarParams()
  toggles = DummyFrogPilotToggles()
  accel_limits = (-3.0, 2.0)
  cs = DummyCarState(v_ego=0.55, a_ego=-0.73, standstill=False, cruise_standstill=False)

  lc = LongControl(cp)
  lc.long_control_state = LongCtrlState.pid
  lc.last_output_accel = -0.77
  out = lc.update(
    active=True,
    CS=cs,
    a_target=-0.30,
    should_stop=True,
    distance_to_stop_target_m=0.10,
    accel_limits=accel_limits,
    frogpilot_toggles=toggles,
  )

  assert lc.long_control_state == LongCtrlState.stopping
  assert out < -0.74


def test_longcontrol_softens_moderately_fast_non_urgent_stop_handoff() -> None:
  cp = DummyCarParams()
  toggles = DummyFrogPilotToggles()
  accel_limits = (-3.0, 2.0)
  cs = DummyCarState(v_ego=1.85, a_ego=-0.72, standstill=False, cruise_standstill=False)

  lc = LongControl(cp)
  lc.long_control_state = LongCtrlState.pid
  lc.last_output_accel = -0.74
  out = lc.update(
    active=True,
    CS=cs,
    a_target=-0.42,
    should_stop=True,
    distance_to_stop_target_m=1.10,
    accel_limits=accel_limits,
    frogpilot_toggles=toggles,
  )

  assert lc.long_control_state == LongCtrlState.stopping
  assert out > -0.70


def test_longcontrol_enters_stop_mode_early_for_short_gap_lead_seed() -> None:
  cp = DummyCarParams()
  toggles = DummyFrogPilotToggles()
  accel_limits = (-3.0, 2.0)
  cs = DummyCarState(v_ego=4.18, a_ego=-1.16, standstill=False, cruise_standstill=False)

  lc = LongControl(cp)
  lc.long_control_state = LongCtrlState.pid
  lc.last_output_accel = -1.22
  out = lc.update(
    active=True,
    CS=cs,
    a_target=-1.23,
    should_stop=False,
    distance_to_stop_target_m=0.23,
    accel_limits=accel_limits,
    frogpilot_toggles=toggles,
  )

  assert lc.long_control_state == LongCtrlState.stopping
  assert out > -1.10


def test_longcontrol_keeps_tiny_nonurgent_stop_target_in_approach_band() -> None:
  cp = DummyCarParams()
  toggles = DummyFrogPilotToggles()
  accel_limits = (-3.0, 2.0)
  cs = DummyCarState(v_ego=2.30, a_ego=-0.21, standstill=False, cruise_standstill=False)

  lc = LongControl(cp)
  lc.long_control_state = LongCtrlState.pid
  lc.last_output_accel = -0.39
  out = lc.update(
    active=True,
    CS=cs,
    a_target=-0.335,
    should_stop=False,
    distance_to_stop_target_m=0.28,
    accel_limits=accel_limits,
    frogpilot_toggles=toggles,
  )

  assert lc.long_control_state == LongCtrlState.pid
  assert out > -0.55


def test_longcontrol_keeps_stopping_while_stop_target_is_still_stable() -> None:
  cp = DummyCarParams()
  toggles = DummyFrogPilotToggles()
  accel_limits = (-3.0, 2.0)
  cs = DummyCarState(v_ego=2.13, a_ego=-0.47, standstill=False, cruise_standstill=False)

  lc = LongControl(cp)
  lc.long_control_state = LongCtrlState.stopping
  lc.last_output_accel = -0.50
  lc.update(
    active=True,
    CS=cs,
    a_target=-0.283,
    should_stop=False,
    distance_to_stop_target_m=1.68,
    accel_limits=accel_limits,
    frogpilot_toggles=toggles,
  )

  assert lc.long_control_state == LongCtrlState.stopping


def test_longcontrol_keeps_stopping_across_low_speed_stop_target_dropout() -> None:
  cp = DummyCarParams()
  toggles = DummyFrogPilotToggles()
  accel_limits = (-3.0, 2.0)
  lc = LongControl(cp)
  lc.long_control_state = LongCtrlState.stopping
  lc.last_output_accel = -0.359

  lc.update(
    active=True,
    CS=DummyCarState(v_ego=0.189, a_ego=-0.328, standstill=False, cruise_standstill=False),
    a_target=-0.197,
    should_stop=True,
    distance_to_stop_target_m=0.934,
    accel_limits=accel_limits,
    frogpilot_toggles=toggles,
  )

  out = lc.update(
    active=True,
    CS=DummyCarState(v_ego=0.160, a_ego=-0.313, standstill=False, cruise_standstill=False),
    a_target=0.20,
    should_stop=False,
    distance_to_stop_target_m=0.865,
    accel_limits=accel_limits,
    frogpilot_toggles=toggles,
  )

  assert lc.long_control_state == LongCtrlState.stopping
  assert out < -0.10


def test_longcontrol_keeps_stopping_path_unclamped_when_stop_request_is_active() -> None:
  cp = DummyCarParams()
  toggles = DummyFrogPilotToggles()
  accel_limits = (-3.0, 2.0)
  lc = LongControl(cp)
  lc.long_control_state = LongCtrlState.stopping
  lc.stopping_controller = FixedStoppingController(output_accel=-1.05)
  lc.last_output_accel = -0.30

  out = lc.update(
    active=True,
    CS=DummyCarState(v_ego=0.8, a_ego=-0.2, standstill=False, cruise_standstill=False),
    a_target=-0.40,
    should_stop=True,
    distance_to_stop_target_m=0.9,
    accel_limits=accel_limits,
    frogpilot_toggles=toggles,
  )

  assert lc.long_control_state == LongCtrlState.stopping
  assert out == pytest.approx(-1.05, abs=1e-12)


def test_longcontrol_allows_resume_when_low_speed_stop_target_moves_away() -> None:
  cp = DummyCarParams()
  toggles = DummyFrogPilotToggles()
  accel_limits = (-3.0, 2.0)
  lc = LongControl(cp)
  lc.long_control_state = LongCtrlState.stopping
  lc.last_output_accel = -0.359

  lc.update(
    active=True,
    CS=DummyCarState(v_ego=0.189, a_ego=-0.328, standstill=False, cruise_standstill=False),
    a_target=-0.197,
    should_stop=True,
    distance_to_stop_target_m=0.934,
    accel_limits=accel_limits,
    frogpilot_toggles=toggles,
  )

  lc.update(
    active=True,
    CS=DummyCarState(v_ego=0.160, a_ego=-0.313, standstill=False, cruise_standstill=False),
    a_target=0.20,
    should_stop=False,
    distance_to_stop_target_m=1.30,
    accel_limits=accel_limits,
    frogpilot_toggles=toggles,
  )

  assert lc.long_control_state == LongCtrlState.pid
