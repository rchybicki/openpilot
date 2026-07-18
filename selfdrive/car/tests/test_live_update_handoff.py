from types import SimpleNamespace

from cereal import car
from opendbc.car import ButtonType
from opendbc.car.hyundai.values import HyundaiFlags

import openpilot.selfdrive.car.card as card_module
from openpilot.selfdrive.car.card import Car
from openpilot.selfdrive.car.live_update_handoff import DIAGNOSTIC, DIAGNOSTIC_REQUESTED, FAILED, MIN_COUNTER_ADVANCES, MIN_SCC14_FRAMES, \
                                                         READY, REQUESTED, SCC11, SCC12, SCC14, \
                                                         SCC_LIVENESS_TIMEOUT_SECONDS, StockSccVerifier, VERIFYING, \
                                                         VERIFY_TIMEOUT_SECONDS, controls_fully_disengaged, is_supported_car, state_name, \
                                                         state_timestamp, timestamped_state
from openpilot.selfdrive.car.live_update_handoff import should_suppress_always_on_lateral


def test_state_encoding():
  state = timestamped_state("ready", 123.456789)
  assert state_name(state) == "ready"
  assert state_timestamp(state) == 123.456789
  assert state_timestamp("ready:not-a-number") is None


def test_supported_car_is_classic_can_hyundai_long_only():
  cp = SimpleNamespace(brand="hyundai", openpilotLongitudinalControl=True, flags=HyundaiFlags.RADAR_SCC)
  assert is_supported_car(cp)

  for flags in (HyundaiFlags.CANFD, HyundaiFlags.CAMERA_SCC, HyundaiFlags.CANFD_CAMERA_SCC):
    cp.flags = flags
    assert not is_supported_car(cp)

  cp.flags = HyundaiFlags.RADAR_SCC
  cp.openpilotLongitudinalControl = False
  assert not is_supported_car(cp)


def test_full_disengagement_requires_cruise_main_and_aol_off():
  cs = SimpleNamespace(cruiseState=SimpleNamespace(available=False, enabled=False))
  cc = SimpleNamespace(enabled=False, latActive=False)
  fpcs = SimpleNamespace(alwaysOnLateralEnabled=False)
  assert controls_fully_disengaged(cs, cc, fpcs, False)

  cs.cruiseState.available = True
  assert not controls_fully_disengaged(cs, cc, fpcs, False)
  cs.cruiseState.available = False
  fpcs.alwaysOnLateralEnabled = True
  assert not controls_fully_disengaged(cs, cc, fpcs, False)
  fpcs.alwaysOnLateralEnabled = False
  assert not controls_fully_disengaged(cs, cc, fpcs, False, selfdrive_active=True)
  assert not controls_fully_disengaged(cs, cc, fpcs, False, cruise_buttons_pressed=True)


def test_post_entry_disengagement_ignores_cruise_available_but_not_actuation():
  cs = SimpleNamespace(cruiseState=SimpleNamespace(available=True, enabled=False))
  cc = SimpleNamespace(enabled=False, latActive=False)
  fpcs = SimpleNamespace(alwaysOnLateralEnabled=False)
  # pre-entry gate still requires cruise main off
  assert not controls_fully_disengaged(cs, cc, fpcs, False)
  # after the Panda handoff begins, the restored stock radar reports main on; that must not pause verification
  assert controls_fully_disengaged(cs, cc, fpcs, False, post_entry=True)
  # actual stock actuation still pauses
  cs.cruiseState.enabled = True
  assert not controls_fully_disengaged(cs, cc, fpcs, False, post_entry=True)
  cs.cruiseState.enabled = False
  cc.enabled = True
  assert not controls_fully_disengaged(cs, cc, fpcs, False, post_entry=True)


def test_pending_handoff_suppresses_aol_once_scc_main_is_off_or_panda_handoff_starts():
  assert not should_suppress_always_on_lateral(REQUESTED, True)
  assert should_suppress_always_on_lateral(REQUESTED, False)
  assert should_suppress_always_on_lateral(DIAGNOSTIC_REQUESTED, True)
  assert should_suppress_always_on_lateral(DIAGNOSTIC, True)
  assert should_suppress_always_on_lateral(VERIFYING, True)
  assert should_suppress_always_on_lateral(READY, True)
  assert should_suppress_always_on_lateral(FAILED, True)
  assert should_suppress_always_on_lateral(VERIFYING, False)
  assert not should_suppress_always_on_lateral("", False)


def _packet(address, counter, src=0, main_on=False, acc_mode=0, acc_fail_info=0, stop_req=0, dec_cmd=0,
            aeb_cmd_act=0, aeb_stop_req=0, scc14_mode=0):
  data = bytearray(8)
  if address == SCC11:
    data[0] = (counter << 4) | (0x1 if main_on else 0x0)
  elif address == SCC12:
    data[1] = (acc_mode << 5) | (acc_fail_info << 3) | (stop_req << 7)
    data[2] = dec_cmd
    data[6] = (aeb_cmd_act << 6) | (aeb_stop_req << 7)
    data[7] = counter
  elif address == SCC14:
    data[4] = scc14_mode
  return (0, [(address, bytes(data), src)])


def _feed_passive(verifier, start=0.0, iterations=MIN_COUNTER_ADVANCES + MIN_SCC14_FRAMES + 1, main_on=False, step=0.02):
  # interleaved 50 Hz-like SCC11/SCC12/SCC14 traffic, as the restored radar emits it
  now = start
  for counter in range(iterations):
    now = start + counter * step
    verifier.update([_packet(SCC11, counter % 16, main_on=main_on), _packet(SCC12, counter % 16), _packet(SCC14, 0)], now)
  return now


def test_stock_scc_verifier_requires_live_counters_and_scc14():
  verifier = StockSccVerifier()
  now = 0.0
  for counter in range(MIN_COUNTER_ADVANCES + 1):
    now = counter * 0.02
    verifier.update([_packet(SCC11, counter % 16), _packet(SCC12, counter % 16)], now)
  assert not verifier.ready(now)

  now = _feed_passive(verifier)
  assert verifier.ready(now)
  assert verifier.live(now)
  assert not verifier.live(now + SCC_LIVENESS_TIMEOUT_SECONDS + 0.01)
  assert not verifier.ready(now + SCC_LIVENESS_TIMEOUT_SECONDS + 0.01)


def test_stock_scc_verifier_requires_passive_healthy_status():
  verifier = StockSccVerifier()
  now = _feed_passive(verifier)
  assert verifier.ready(now)

  # stock ACC actively actuating (SCC12 ACCMode 1/2) must reject
  verifier.update([_packet(SCC12, 11, acc_mode=1)], now + 0.02)
  assert not verifier.ready(now + 0.02)
  verifier.update([_packet(SCC12, 12, acc_mode=2)], now + 0.04)
  assert not verifier.ready(now + 0.04)

  # a single clearing frame is not enough: evidence collection restarts after any active/faulted frame
  verifier.update([_packet(SCC12, 13)], now + 0.06)
  assert not verifier.ready(now + 0.06)
  now = _feed_passive(verifier, start=now + 0.08)
  assert verifier.ready(now)

  # any SCC fault must reject and restart evidence collection
  verifier.update([_packet(SCC12, 12, acc_fail_info=1)], now + 0.02)
  assert not verifier.ready(now + 0.02)
  verifier.update([_packet(SCC12, 13)], now + 0.04)
  assert not verifier.ready(now + 0.04)


def test_stock_scc_verifier_rejects_independent_braking_and_scc14_modes():
  base = StockSccVerifier()
  now = _feed_passive(base)
  assert base.ready(now)

  # independent stock braking commands must reject even with ACCMode == 0
  for kwargs in ({"stop_req": 1}, {"dec_cmd": 0x20}, {"aeb_cmd_act": 1}, {"aeb_stop_req": 1}):
    verifier = StockSccVerifier()
    t = _feed_passive(verifier)
    verifier.update([_packet(SCC12, 11, **kwargs)], t + 0.02)
    assert not verifier.ready(t + 0.02), kwargs

  # SCC14 ACCMode: 0 and 4 (field-observed stock inactive) accepted; active/unknown modes reject
  verifier = StockSccVerifier()
  t = _feed_passive(verifier)
  verifier.update([_packet(SCC14, 0, scc14_mode=4)], t + 0.02)
  assert verifier.ready(t + 0.04)
  verifier.update([_packet(SCC14, 0, scc14_mode=1)], t + 0.04)
  assert not verifier.ready(t + 0.04)
  assert not verifier.live(t + 0.04)


def test_stock_scc_verifier_rejects_batched_and_gapped_evidence():
  # a backlog drained in one batch shares one timestamp and must not satisfy the evidence span
  verifier = StockSccVerifier()
  packets = []
  for counter in range(MIN_COUNTER_ADVANCES + MIN_SCC14_FRAMES + 1):
    packets.extend([_packet(SCC11, counter % 16), _packet(SCC12, counter % 16), _packet(SCC14, 0)])
  verifier.update(packets, 5.0)
  assert not verifier.ready(5.0)

  # a silence longer than the liveness window restarts the run even if the counter delta stays valid
  verifier = StockSccVerifier()
  now = _feed_passive(verifier, iterations=MIN_COUNTER_ADVANCES)
  gap_start = now + SCC_LIVENESS_TIMEOUT_SECONDS + 0.2
  counter = MIN_COUNTER_ADVANCES % 16
  verifier.update([_packet(SCC11, (counter + 1) % 16), _packet(SCC12, (counter + 1) % 16), _packet(SCC14, 0)], gap_start)
  assert not verifier.ready(gap_start)
  now = _feed_passive(verifier, start=gap_start + 0.02)
  assert verifier.ready(now)


def test_stock_scc_verifier_accepts_restored_radar_with_main_on():
  # Field-observed takeover: the radar returns from the diagnostic knockout with SCC11 MainMode_ACC=1
  # from the very first frame, while SCC12 reports ACCMode=0 / ACCFailInfo=0 and SCC14 streams.
  verifier = StockSccVerifier()
  now = _feed_passive(verifier, main_on=True)
  assert verifier.scc11_main_on
  assert verifier.ready(now)
  assert verifier.live(now)


def test_stock_scc_verifier_ignores_send_echo_bus():
  verifier = StockSccVerifier()
  now = 0.0
  for counter in range(MIN_COUNTER_ADVANCES + 2):
    now = counter * 0.02
    verifier.update([_packet(SCC11, counter % 16, src=128), _packet(SCC12, counter % 16, src=128), _packet(SCC14, 0, src=128)], now)
  assert not verifier.ready(now)


class FakeParams:
  def __init__(self, state):
    self.state = state
    self.removed = []

  def get(self, key):
    return self.state

  def put(self, key, value):
    self.state = value

  def put_nonblocking(self, key, value):
    self.state = value

  def remove(self, key):
    self.removed.append(key)


class FakeSubMaster:
  def __init__(self):
    self.data = {
      "carControl": SimpleNamespace(enabled=False, latActive=False),
      "selfdriveState": SimpleNamespace(enabled=False, active=False),
      "pandaStates": [SimpleNamespace(safetyModel=car.CarParams.SafetyModel.hyundai, faults=[])],
    }

  def __getitem__(self, service):
    return self.data[service]

  def all_checks(self, services):
    return True

  def all_alive(self, services):
    return True


def _feed_card_passive(card_instance, CS, FPCS, now, main_on=False):
  # feed interleaved passive stock SCC traffic across real time, as card's 100 Hz loop would see it
  for counter in range(MIN_COUNTER_ADVANCES + MIN_SCC14_FRAMES + 1):
    card_instance.can_list = [_packet(SCC11, counter % 16, main_on=main_on), _packet(SCC12, counter % 16), _packet(SCC14, 0)]
    now[0] += 0.02
    card_instance.update_live_update_handoff(CS, FPCS, True)


def test_card_keeps_sending_until_panda_confirms_elm(monkeypatch):
  now = [10.0]
  monkeypatch.setattr(card_module.time, "monotonic", lambda: now[0])

  card_instance = Car.__new__(Car)
  card_instance.params = FakeParams(REQUESTED)
  card_instance.live_update_handoff_state = ""
  card_instance.live_update_handoff_last_read = 0.0
  card_instance.live_update_handoff_preconditions_since = None
  card_instance.live_update_handoff_started_at = None
  card_instance.live_update_handoff_verify_started_at = None
  card_instance.live_update_handoff_radar_restore_last_attempt = None
  card_instance.live_update_handoff_controls_active_since = None
  card_instance.live_update_handoff_pressed_buttons = {ButtonType.mainCruise}
  card_instance.live_update_handoff_verifier = StockSccVerifier()
  card_instance.CP = SimpleNamespace(brand="hyundai", openpilotLongitudinalControl=True, flags=HyundaiFlags.RADAR_SCC, passive=False)
  card_instance.sm = FakeSubMaster()
  card_instance.last_actuators_output = None
  card_instance.initialized_prev = True
  card_instance.can_callbacks = (object(), object())
  card_instance.can_list = []
  deinit_calls = []
  init_calls = []
  card_instance.CI = SimpleNamespace(init=lambda *args: init_calls.append(args), deinit=lambda *args: deinit_calls.append(args) or True)

  CS = SimpleNamespace(cruiseState=SimpleNamespace(available=False, enabled=False), gearShifter=car.CarState.GearShifter.park)
  FPCS = SimpleNamespace(alwaysOnLateralEnabled=False)
  assert not card_instance.update_live_update_handoff(CS, FPCS, True)

  now[0] += 2.1
  assert not card_instance.update_live_update_handoff(CS, FPCS, True)
  assert state_name(card_instance.params.state) == REQUESTED
  assert card_instance.live_update_handoff_preconditions_since is None

  card_instance.live_update_handoff_pressed_buttons.clear()
  now[0] += 0.1
  assert not card_instance.update_live_update_handoff(CS, FPCS, True)
  now[0] += 2.1
  assert not card_instance.update_live_update_handoff(CS, FPCS, True)
  assert state_name(card_instance.params.state) == REQUESTED
  assert card_instance.live_update_handoff_preconditions_since is None

  CS.gearShifter = car.CarState.GearShifter.drive
  now[0] += 0.1
  assert not card_instance.update_live_update_handoff(CS, FPCS, True)
  now[0] += 2.1
  assert not card_instance.update_live_update_handoff(CS, FPCS, True)
  assert state_name(card_instance.params.state) == DIAGNOSTIC_REQUESTED
  assert not deinit_calls

  card_instance.params.state = DIAGNOSTIC
  now[0] += 0.1
  assert not card_instance.update_live_update_handoff(CS, FPCS, True)
  assert not deinit_calls

  card_instance.sm.data["pandaStates"][0].safetyModel = car.CarParams.SafetyModel.elm327
  now[0] += 0.1
  assert card_instance.update_live_update_handoff(CS, FPCS, True)
  assert len(deinit_calls) == 1
  assert card_instance.params.removed == ["ControlsReady"]
  assert state_name(card_instance.params.state) == VERIFYING

  _feed_card_passive(card_instance, CS, FPCS, now)
  assert state_name(card_instance.params.state) == READY

  card_instance.initialized_prev = False
  card_instance.controls_update(CS, card_instance.sm["carControl"], controls_quiesced=True)
  assert not init_calls


def test_failed_handoff_stays_quiesced_and_restores_radar(monkeypatch):
  now = [20.0]
  monkeypatch.setattr(card_module.time, "monotonic", lambda: now[0])

  card_instance = Car.__new__(Car)
  card_instance.params = FakeParams(FAILED)
  card_instance.live_update_handoff_state = ""
  card_instance.live_update_handoff_last_read = 0.0
  card_instance.live_update_handoff_preconditions_since = None
  card_instance.live_update_handoff_started_at = 19.0
  card_instance.live_update_handoff_verify_started_at = None
  card_instance.live_update_handoff_radar_restore_last_attempt = None
  card_instance.live_update_handoff_controls_active_since = None
  card_instance.live_update_handoff_pressed_buttons = set()
  card_instance.live_update_handoff_verifier = StockSccVerifier()
  card_instance.CP = SimpleNamespace(brand="hyundai", openpilotLongitudinalControl=True, flags=HyundaiFlags.RADAR_SCC, passive=False)
  card_instance.sm = FakeSubMaster()
  card_instance.sm.data["pandaStates"][0].safetyModel = car.CarParams.SafetyModel.elm327
  card_instance.initialized_prev = True
  card_instance.can_callbacks = (object(), object())
  card_instance.can_list = []
  deinit_calls = []
  card_instance.CI = SimpleNamespace(deinit=lambda *args, **kwargs: deinit_calls.append((args, kwargs)) or True)

  CS = SimpleNamespace(cruiseState=SimpleNamespace(available=False, enabled=False), gearShifter=car.CarState.GearShifter.drive)
  FPCS = SimpleNamespace(alwaysOnLateralEnabled=False)
  assert card_instance.update_live_update_handoff(CS, FPCS, True)
  assert state_name(card_instance.params.state) == FAILED
  assert len(deinit_calls) == 1
  assert deinit_calls[0][1] == {"retry": 2}
  assert card_instance.params.removed == ["ControlsReady"]


def test_post_commit_activity_pauses_and_reverifies_instead_of_failing(monkeypatch):
  now = [25.0]
  monkeypatch.setattr(card_module.time, "monotonic", lambda: now[0])

  card_instance = Car.__new__(Car)
  card_instance.params = FakeParams(timestamped_state(READY, now[0]))
  card_instance.live_update_handoff_state = ""
  card_instance.live_update_handoff_last_read = 0.0
  card_instance.live_update_handoff_preconditions_since = None
  card_instance.live_update_handoff_started_at = 24.0
  card_instance.live_update_handoff_verify_started_at = 24.0
  card_instance.live_update_handoff_radar_restore_last_attempt = None
  card_instance.live_update_handoff_controls_active_since = None
  card_instance.live_update_handoff_pressed_buttons = set()
  card_instance.live_update_handoff_verifier = StockSccVerifier()
  card_instance.CP = SimpleNamespace(brand="hyundai", openpilotLongitudinalControl=True, flags=HyundaiFlags.RADAR_SCC, passive=False)
  card_instance.sm = FakeSubMaster()
  card_instance.sm.data["pandaStates"][0].safetyModel = car.CarParams.SafetyModel.elm327
  card_instance.sm.data["carControl"].enabled = True
  card_instance.can_list = []

  CS = SimpleNamespace(cruiseState=SimpleNamespace(available=False, enabled=False), gearShifter=car.CarState.GearShifter.drive)
  FPCS = SimpleNamespace(alwaysOnLateralEnabled=False)
  assert card_instance.update_live_update_handoff(CS, FPCS, True)
  assert state_name(card_instance.params.state) == VERIFYING

  now[0] += 10.0
  assert card_instance.update_live_update_handoff(CS, FPCS, True)
  assert state_name(card_instance.params.state) == VERIFYING

  card_instance.sm.data["carControl"].enabled = False
  _feed_card_passive(card_instance, CS, FPCS, now)
  assert state_name(card_instance.params.state) == READY
  assert card_instance.live_update_handoff_controls_active_since is None


def test_verification_accepts_stock_main_on_takeover(monkeypatch):
  # Field regression: the restored radar reports MainMode_ACC=1 immediately and cruiseState.available
  # may read true post-entry; verification must still reach READY on healthy inactive stock SCC.
  now = [40.0]
  monkeypatch.setattr(card_module.time, "monotonic", lambda: now[0])

  card_instance = Car.__new__(Car)
  card_instance.params = FakeParams(timestamped_state(VERIFYING, now[0]))
  card_instance.live_update_handoff_state = ""
  card_instance.live_update_handoff_last_read = 0.0
  card_instance.live_update_handoff_preconditions_since = None
  card_instance.live_update_handoff_started_at = 39.0
  card_instance.live_update_handoff_verify_started_at = now[0]
  card_instance.live_update_handoff_radar_restore_last_attempt = None
  card_instance.live_update_handoff_controls_active_since = None
  card_instance.live_update_handoff_pressed_buttons = set()
  card_instance.live_update_handoff_verifier = StockSccVerifier()
  card_instance.CP = SimpleNamespace(brand="hyundai", openpilotLongitudinalControl=True, flags=HyundaiFlags.RADAR_SCC, passive=False)
  card_instance.sm = FakeSubMaster()
  card_instance.sm.data["pandaStates"][0].safetyModel = car.CarParams.SafetyModel.elm327
  card_instance.can_list = []

  CS = SimpleNamespace(cruiseState=SimpleNamespace(available=True, enabled=False), gearShifter=car.CarState.GearShifter.drive)
  FPCS = SimpleNamespace(alwaysOnLateralEnabled=False)
  _feed_card_passive(card_instance, CS, FPCS, now, main_on=True)
  assert state_name(card_instance.params.state) == READY
  assert card_instance.live_update_handoff_controls_active_since is None


def test_diagnostic_pause_restores_radar_before_waiting(monkeypatch):
  now = [26.0]
  monkeypatch.setattr(card_module.time, "monotonic", lambda: now[0])

  card_instance = Car.__new__(Car)
  card_instance.params = FakeParams(DIAGNOSTIC)
  card_instance.live_update_handoff_state = ""
  card_instance.live_update_handoff_last_read = 0.0
  card_instance.live_update_handoff_preconditions_since = None
  card_instance.live_update_handoff_started_at = 25.0
  card_instance.live_update_handoff_verify_started_at = None
  card_instance.live_update_handoff_radar_restore_last_attempt = None
  card_instance.live_update_handoff_controls_active_since = None
  card_instance.live_update_handoff_pressed_buttons = set()
  card_instance.live_update_handoff_verifier = StockSccVerifier()
  card_instance.CP = SimpleNamespace(brand="hyundai", openpilotLongitudinalControl=True, flags=HyundaiFlags.RADAR_SCC, passive=False)
  card_instance.sm = FakeSubMaster()
  card_instance.sm.data["pandaStates"][0].safetyModel = car.CarParams.SafetyModel.elm327
  card_instance.can_callbacks = (object(), object())
  card_instance.can_list = []
  deinit_calls = []
  card_instance.CI = SimpleNamespace(deinit=lambda *args: deinit_calls.append(args) or True)

  CS = SimpleNamespace(cruiseState=SimpleNamespace(available=False, enabled=True), gearShifter=car.CarState.GearShifter.drive)
  FPCS = SimpleNamespace(alwaysOnLateralEnabled=False)
  assert card_instance.update_live_update_handoff(CS, FPCS, True)
  assert len(deinit_calls) == 1
  assert card_instance.params.removed == ["ControlsReady"]
  assert state_name(card_instance.params.state) == VERIFYING

  now[0] += 10.0
  assert card_instance.update_live_update_handoff(CS, FPCS, True)
  assert len(deinit_calls) == 1
  assert state_name(card_instance.params.state) == VERIFYING


def test_ready_handoff_is_revoked_if_vehicle_leaves_drive(monkeypatch):
  now = [27.0]
  monkeypatch.setattr(card_module.time, "monotonic", lambda: now[0])

  card_instance = Car.__new__(Car)
  card_instance.params = FakeParams(timestamped_state(READY, now[0]))
  card_instance.live_update_handoff_state = ""
  card_instance.live_update_handoff_last_read = 0.0
  card_instance.live_update_handoff_preconditions_since = None
  card_instance.live_update_handoff_started_at = 26.0
  card_instance.live_update_handoff_verify_started_at = 26.0
  card_instance.live_update_handoff_radar_restore_last_attempt = None
  card_instance.live_update_handoff_controls_active_since = None
  card_instance.live_update_handoff_pressed_buttons = set()
  card_instance.live_update_handoff_verifier = StockSccVerifier()
  card_instance.CP = SimpleNamespace(brand="hyundai", openpilotLongitudinalControl=True, flags=HyundaiFlags.RADAR_SCC, passive=False)
  card_instance.sm = FakeSubMaster()
  card_instance.sm.data["pandaStates"][0].safetyModel = car.CarParams.SafetyModel.elm327
  card_instance.can_list = []

  CS = SimpleNamespace(cruiseState=SimpleNamespace(available=False, enabled=False), gearShifter=car.CarState.GearShifter.park)
  FPCS = SimpleNamespace(alwaysOnLateralEnabled=False)
  assert card_instance.update_live_update_handoff(CS, FPCS, True)
  assert state_name(card_instance.params.state) == VERIFYING

  now[0] += 10.0
  assert card_instance.update_live_update_handoff(CS, FPCS, True)
  assert state_name(card_instance.params.state) == VERIFYING

  CS.gearShifter = car.CarState.GearShifter.drive
  _feed_card_passive(card_instance, CS, FPCS, now)
  assert state_name(card_instance.params.state) == READY


def test_request_waits_out_fingerprinting_elm_mode(monkeypatch):
  now = [25.0]
  monkeypatch.setattr(card_module.time, "monotonic", lambda: now[0])

  card_instance = Car.__new__(Car)
  card_instance.params = FakeParams(REQUESTED)
  card_instance.live_update_handoff_state = ""
  card_instance.live_update_handoff_last_read = 0.0
  card_instance.live_update_handoff_preconditions_since = None
  card_instance.live_update_handoff_started_at = None
  card_instance.live_update_handoff_verify_started_at = None
  card_instance.live_update_handoff_radar_restore_last_attempt = None
  card_instance.live_update_handoff_controls_active_since = None
  card_instance.live_update_handoff_pressed_buttons = set()
  card_instance.live_update_handoff_verifier = StockSccVerifier()
  card_instance.CP = SimpleNamespace(brand="hyundai", openpilotLongitudinalControl=True, flags=HyundaiFlags.RADAR_SCC, passive=False)
  card_instance.sm = FakeSubMaster()
  card_instance.sm.data["pandaStates"][0].safetyModel = car.CarParams.SafetyModel.elm327
  card_instance.initialized_prev = False
  card_instance.can_list = []

  CS = SimpleNamespace(cruiseState=SimpleNamespace(available=False, enabled=False), gearShifter=car.CarState.GearShifter.drive)
  FPCS = SimpleNamespace(alwaysOnLateralEnabled=False)
  assert not card_instance.update_live_update_handoff(CS, FPCS, False)
  assert not card_instance.update_live_update_handoff(CS, FPCS, True)
  assert state_name(card_instance.params.state) == REQUESTED
  assert card_instance.live_update_handoff_preconditions_since == now[0]

  card_instance.initialized_prev = True
  now[0] += 2.1
  assert card_instance.update_live_update_handoff(CS, FPCS, True)
  assert state_name(card_instance.params.state) == DIAGNOSTIC_REQUESTED


def test_failed_handoff_can_be_explicitly_rearmed_and_freshly_verified(monkeypatch):
  now = [28.0]
  monkeypatch.setattr(card_module.time, "monotonic", lambda: now[0])

  stale_verifier = StockSccVerifier()
  stale_start = _feed_passive(stale_verifier, start=now[0] - 1.0)
  assert stale_verifier.ready(stale_start)

  card_instance = Car.__new__(Car)
  card_instance.params = FakeParams(FAILED)
  card_instance.params.state = REQUESTED
  card_instance.live_update_handoff_state = FAILED
  card_instance.live_update_handoff_last_read = 0.0
  card_instance.live_update_handoff_preconditions_since = None
  card_instance.live_update_handoff_started_at = 27.0
  card_instance.live_update_handoff_verify_started_at = 27.0
  card_instance.live_update_handoff_radar_restore_last_attempt = None
  card_instance.live_update_handoff_controls_active_since = None
  card_instance.live_update_handoff_pressed_buttons = set()
  card_instance.live_update_handoff_verifier = stale_verifier
  card_instance.CP = SimpleNamespace(brand="hyundai", openpilotLongitudinalControl=True, flags=HyundaiFlags.RADAR_SCC, passive=False)
  card_instance.sm = FakeSubMaster()
  card_instance.sm.data["pandaStates"][0].safetyModel = car.CarParams.SafetyModel.elm327
  card_instance.initialized_prev = True
  card_instance.can_callbacks = (object(), object())
  card_instance.can_list = []
  deinit_calls = []
  card_instance.CI = SimpleNamespace(deinit=lambda *args: deinit_calls.append(args) or True)

  CS = SimpleNamespace(cruiseState=SimpleNamespace(available=False, enabled=False), gearShifter=car.CarState.GearShifter.drive)
  FPCS = SimpleNamespace(alwaysOnLateralEnabled=False)
  assert card_instance.update_live_update_handoff(CS, FPCS, True)
  assert state_name(card_instance.params.state) == REQUESTED
  assert card_instance.live_update_handoff_preconditions_since == now[0]

  now[0] += 2.1
  assert card_instance.update_live_update_handoff(CS, FPCS, True)
  assert state_name(card_instance.params.state) == DIAGNOSTIC_REQUESTED

  now[0] += 0.1
  assert card_instance.update_live_update_handoff(CS, FPCS, True)
  assert state_name(card_instance.params.state) == VERIFYING
  assert len(deinit_calls) == 1
  assert card_instance.params.removed == ["ControlsReady"]
  assert not card_instance.live_update_handoff_verifier.ready(now[0])

  _feed_card_passive(card_instance, CS, FPCS, now)
  assert state_name(card_instance.params.state) == READY


def test_requested_elm_retry_stays_quiesced_while_controls_are_active(monkeypatch):
  now = [32.0]
  monkeypatch.setattr(card_module.time, "monotonic", lambda: now[0])

  card_instance = Car.__new__(Car)
  card_instance.params = FakeParams(REQUESTED)
  card_instance.live_update_handoff_state = ""
  card_instance.live_update_handoff_last_read = 0.0
  card_instance.live_update_handoff_preconditions_since = None
  card_instance.live_update_handoff_started_at = None
  card_instance.live_update_handoff_verify_started_at = None
  card_instance.live_update_handoff_radar_restore_last_attempt = None
  card_instance.live_update_handoff_controls_active_since = None
  card_instance.live_update_handoff_pressed_buttons = set()
  card_instance.live_update_handoff_verifier = StockSccVerifier()
  card_instance.CP = SimpleNamespace(brand="hyundai", openpilotLongitudinalControl=True, flags=HyundaiFlags.RADAR_SCC, passive=False)
  card_instance.sm = FakeSubMaster()
  card_instance.sm.data["pandaStates"][0].safetyModel = car.CarParams.SafetyModel.elm327
  card_instance.initialized_prev = True
  card_instance.can_list = []

  CS = SimpleNamespace(cruiseState=SimpleNamespace(available=True, enabled=False), gearShifter=car.CarState.GearShifter.drive)
  FPCS = SimpleNamespace(alwaysOnLateralEnabled=False)
  assert card_instance.update_live_update_handoff(CS, FPCS, True)
  assert state_name(card_instance.params.state) == REQUESTED
  assert card_instance.live_update_handoff_preconditions_since is None

  now[0] += 10.0
  assert card_instance.update_live_update_handoff(CS, FPCS, True)
  assert state_name(card_instance.params.state) == REQUESTED
  assert card_instance.live_update_handoff_preconditions_since is None


def test_rearmed_handoff_failure_returns_to_radar_recovery(monkeypatch):
  now = [45.0]
  monkeypatch.setattr(card_module.time, "monotonic", lambda: now[0])

  card_instance = Car.__new__(Car)
  card_instance.params = FakeParams(timestamped_state(VERIFYING, now[0]))
  card_instance.live_update_handoff_state = ""
  card_instance.live_update_handoff_last_read = 0.0
  card_instance.live_update_handoff_preconditions_since = None
  card_instance.live_update_handoff_started_at = 44.0
  card_instance.live_update_handoff_verify_started_at = now[0]
  card_instance.live_update_handoff_radar_restore_last_attempt = None
  card_instance.live_update_handoff_controls_active_since = None
  card_instance.live_update_handoff_pressed_buttons = set()
  card_instance.live_update_handoff_verifier = StockSccVerifier()
  card_instance.CP = SimpleNamespace(brand="hyundai", openpilotLongitudinalControl=True, flags=HyundaiFlags.RADAR_SCC, passive=False)
  card_instance.sm = FakeSubMaster()
  card_instance.sm.data["pandaStates"][0].safetyModel = car.CarParams.SafetyModel.elm327
  card_instance.initialized_prev = True
  card_instance.can_callbacks = (object(), object())
  card_instance.can_list = []
  deinit_calls = []
  card_instance.CI = SimpleNamespace(deinit=lambda *args, **kwargs: deinit_calls.append((args, kwargs)) or True)

  CS = SimpleNamespace(cruiseState=SimpleNamespace(available=False, enabled=False), gearShifter=car.CarState.GearShifter.drive)
  FPCS = SimpleNamespace(alwaysOnLateralEnabled=False)
  now[0] += VERIFY_TIMEOUT_SECONDS + 0.1
  assert card_instance.update_live_update_handoff(CS, FPCS, True)
  assert state_name(card_instance.params.state) == FAILED

  now[0] += 0.1
  assert card_instance.update_live_update_handoff(CS, FPCS, True)
  assert len(deinit_calls) == 1
  assert deinit_calls[0][1] == {"retry": 2}


def test_terminal_state_resets_request_dwell(monkeypatch):
  monkeypatch.setattr(card_module.time, "monotonic", lambda: 30.0)
  card_instance = Car.__new__(Car)
  card_instance.params = FakeParams(REQUESTED)
  card_instance.live_update_handoff_state = REQUESTED
  card_instance.live_update_handoff_last_read = 30.0
  card_instance.live_update_handoff_preconditions_since = 25.0

  card_instance._set_live_update_handoff_state(timestamped_state(FAILED, 30.0))
  assert card_instance.live_update_handoff_preconditions_since is None


def test_external_state_change_resets_request_dwell(monkeypatch):
  now = [60.0]
  monkeypatch.setattr(card_module.time, "monotonic", lambda: now[0])

  card_instance = Car.__new__(Car)
  card_instance.params = FakeParams(FAILED)
  card_instance.live_update_handoff_state = REQUESTED
  card_instance.live_update_handoff_last_read = 0.0
  card_instance.live_update_handoff_preconditions_since = 50.0
  card_instance.live_update_handoff_started_at = None
  card_instance.live_update_handoff_verify_started_at = None
  card_instance.live_update_handoff_radar_restore_last_attempt = None
  card_instance.live_update_handoff_controls_active_since = None
  card_instance.live_update_handoff_pressed_buttons = set()
  card_instance.live_update_handoff_verifier = StockSccVerifier()
  card_instance.CP = SimpleNamespace(brand="hyundai", openpilotLongitudinalControl=True, flags=HyundaiFlags.RADAR_SCC, passive=False)
  card_instance.sm = FakeSubMaster()
  card_instance.initialized_prev = True
  card_instance.can_list = []

  CS = SimpleNamespace(cruiseState=SimpleNamespace(available=False, enabled=False), gearShifter=car.CarState.GearShifter.drive)
  FPCS = SimpleNamespace(alwaysOnLateralEnabled=False)
  assert card_instance.update_live_update_handoff(CS, FPCS, True)
  assert card_instance.live_update_handoff_preconditions_since is None

  card_instance.params.state = REQUESTED
  now[0] += 0.1
  assert not card_instance.update_live_update_handoff(CS, FPCS, True)
  assert state_name(card_instance.params.state) == REQUESTED
  assert card_instance.live_update_handoff_preconditions_since == now[0]

  now[0] += 1.9
  assert not card_instance.update_live_update_handoff(CS, FPCS, True)
  assert state_name(card_instance.params.state) == REQUESTED

  now[0] += 0.2
  assert not card_instance.update_live_update_handoff(CS, FPCS, True)
  assert state_name(card_instance.params.state) == DIAGNOSTIC_REQUESTED
