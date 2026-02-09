from __future__ import annotations

from typing import Protocol

from openpilot.selfdrive.controls.lib.stopping_controller import STOPPING_CONTROLLER_TUNINGS, StoppingController
from openpilot.selfdrive.controls.lib.stopping_controller_abstract import (
  AbstractStoppingControllerV2,
  AbstractStoppingControllerV3,
)


class StoppingControllerProtocol(Protocol):
  def reset(self) -> None:
    ...

  def update(
    self,
    output_accel: float,
    last_output_accel: float,
    should_stop: bool,
    v_ego: float,
    a_ego: float,
    max_expected_accel: float,
    min_expected_accel: float,
    stop_accel: float,
    dt: float,
  ):
    ...


LEGACY_STOPPING_CONTROLLER_VARIANTS = tuple(f"legacy_{name}" for name in STOPPING_CONTROLLER_TUNINGS)
ABSTRACT_STOPPING_CONTROLLER_VARIANTS = ("abstract_v2", "abstract_v3")
STOPPING_CONTROLLER_VARIANTS = LEGACY_STOPPING_CONTROLLER_VARIANTS + ABSTRACT_STOPPING_CONTROLLER_VARIANTS
DEFAULT_STOPPING_CONTROLLER_VARIANT = "legacy_v2"


def build_stopping_controller(variant: str) -> StoppingControllerProtocol:
  if variant.startswith("legacy_"):
    strategy = variant.split("legacy_", maxsplit=1)[1]
    if strategy in STOPPING_CONTROLLER_TUNINGS:
      return StoppingController(strategy=strategy)
  if variant == "abstract_v2":
    return AbstractStoppingControllerV2()
  if variant == "abstract_v3":
    return AbstractStoppingControllerV3()
  return StoppingController(strategy="v2")
