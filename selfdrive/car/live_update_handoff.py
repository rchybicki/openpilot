from dataclasses import dataclass, field

from opendbc.car.hyundai.values import HyundaiFlags


LIVE_UPDATE_HANDOFF_PARAM = "LiveUpdateHandoffState"

REQUESTED = "requested"
DIAGNOSTIC_REQUESTED = "diagnostic_requested"
DIAGNOSTIC = "diagnostic"
VERIFYING = "verifying"
READY = "ready"
FAILED = "failed"
ABORTED = "aborted"
UNSUPPORTED = "unsupported"

PANDA_HANDOFF_STATES = (DIAGNOSTIC_REQUESTED, DIAGNOSTIC, VERIFYING, READY, FAILED)
ELM_ACCEPTED_STATES = PANDA_HANDOFF_STATES
ACTIVE_HANDOFF_STATES = (REQUESTED, *PANDA_HANDOFF_STATES)

PRECONDITION_SECONDS = 2.0
VERIFY_TIMEOUT_SECONDS = 8.0
READY_REFRESH_SECONDS = 0.5
SCC_LIVENESS_TIMEOUT_SECONDS = 0.5
RADAR_RESTORE_RETRY_SECONDS = 1.0

SCC11 = 0x420
SCC12 = 0x421
SCC14 = 0x389
MIN_COUNTER_ADVANCES = 20
MIN_SCC14_FRAMES = 10


def state_name(state: str) -> str:
  return state.partition(":")[0]


def state_timestamp(state: str) -> float | None:
  _, separator, timestamp = state.partition(":")
  if not separator:
    return None
  try:
    return float(timestamp)
  except ValueError:
    return None


def timestamped_state(state: str, now: float) -> str:
  return f"{state}:{now:.6f}"


def is_supported_car(CP) -> bool:
  unsupported_flags = HyundaiFlags.CANFD | HyundaiFlags.CANFD_CAMERA_SCC | HyundaiFlags.CAMERA_SCC
  return CP.brand == "hyundai" and CP.openpilotLongitudinalControl and not (CP.flags & unsupported_flags)


def controls_disengagement_reasons(CS, CC, FPCS, selfdrive_enabled: bool, selfdrive_active: bool = False,
                                   cruise_buttons_pressed: bool = False) -> tuple[str, ...]:
  reasons = []
  if CC.enabled:
    reasons.append("carControl.enabled")
  if CC.latActive:
    reasons.append("carControl.latActive")
  if selfdrive_enabled:
    reasons.append("selfdriveState.enabled")
  if selfdrive_active:
    reasons.append("selfdriveState.active")
  if FPCS.alwaysOnLateralEnabled:
    reasons.append("alwaysOnLateralEnabled")
  if CS.cruiseState.available:
    reasons.append("cruiseState.available")
  if CS.cruiseState.enabled:
    reasons.append("cruiseState.enabled")
  if cruise_buttons_pressed:
    reasons.append("cruise button held")
  return tuple(reasons)


def controls_fully_disengaged(CS, CC, FPCS, selfdrive_enabled: bool, selfdrive_active: bool = False,
                              cruise_buttons_pressed: bool = False) -> bool:
  return not controls_disengagement_reasons(CS, CC, FPCS, selfdrive_enabled, selfdrive_active, cruise_buttons_pressed)


def should_suppress_always_on_lateral(state: str, cruise_available: bool) -> bool:
  return not cruise_available and state_name(state) in ACTIVE_HANDOFF_STATES


@dataclass
class StockSccVerifier:
  counter_advances: dict[int, int] = field(default_factory=lambda: {SCC11: 0, SCC12: 0})
  last_counters: dict[int, int | None] = field(default_factory=lambda: {SCC11: None, SCC12: None})
  last_counter_advances: dict[int, float | None] = field(default_factory=lambda: {SCC11: None, SCC12: None})
  scc14_frames: int = 0
  last_scc14_frame: float | None = None
  scc11_passive: bool = False
  scc12_passive_healthy: bool = False

  @staticmethod
  def _counter(address: int, data: bytes) -> int:
    if address == SCC11:
      return data[0] >> 4
    return data[7] & 0xF

  def update(self, can_packets, now: float = 0.0) -> None:
    for _, frames in can_packets:
      for address, data, src in frames:
        if src != 0 or len(data) != 8:
          continue

        if address in self.last_counters:
          if address == SCC11:
            self.scc11_passive = (data[0] & 0x1) == 0
          else:
            acc_fail_info = (data[1] >> 3) & 0x3
            acc_mode = (data[1] >> 5) & 0x3
            self.scc12_passive_healthy = acc_fail_info == 0 and acc_mode == 0

          counter = self._counter(address, data)
          previous = self.last_counters[address]
          if previous is not None:
            delta = (counter - previous) % 16
            if 1 <= delta <= 4:
              self.counter_advances[address] += 1
              self.last_counter_advances[address] = now
            elif delta != 0:
              self.counter_advances[address] = 0
              self.last_counter_advances[address] = None
          self.last_counters[address] = counter
        elif address == SCC14:
          self.scc14_frames += 1
          self.last_scc14_frame = now

  @property
  def ready(self) -> bool:
    return (all(count >= MIN_COUNTER_ADVANCES for count in self.counter_advances.values()) and
            self.scc14_frames >= MIN_SCC14_FRAMES and self.scc11_passive and self.scc12_passive_healthy)

  def live(self, now: float) -> bool:
    timestamps = (*self.last_counter_advances.values(), self.last_scc14_frame)
    return (self.scc11_passive and self.scc12_passive_healthy and
            all(timestamp is not None and 0.0 <= now - timestamp <= SCC_LIVENESS_TIMEOUT_SECONDS for timestamp in timestamps))
