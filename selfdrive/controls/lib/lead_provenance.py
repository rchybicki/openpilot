"""Track-scoped provenance for extra stopping authority.

The normal radar/MPC path intentionally accepts low-speed radar-only returns. Custom stopping
lanes need a stricter boundary: a close radar-only return may be an obstacle, but it may also be
road furniture. Give it extra authority only when it was first selected early enough to stop at
the safety floor, or when vision subsequently associates with the same track.
"""

from __future__ import annotations

import math


RADAR_ONLY_CERT_REST_FLOOR_M = 3.0
RADAR_ONLY_CERT_MIN_DECEL = 1.5
RADAR_ONLY_CERT_ACTUATION_DELAY_S = 0.2
RADAR_ONLY_CERT_CONFIRMATION_TIME_S = 0.5
VISION_ONLY_CERT_PROB_MIN = 0.9


def get_radar_only_min_acquire_d_rel(
  v_ego: float,
  *,
  rest_floor_m: float = RADAR_ONLY_CERT_REST_FLOOR_M,
  min_decel: float = RADAR_ONLY_CERT_MIN_DECEL,
  actuation_delay_s: float = RADAR_ONLY_CERT_ACTUATION_DELAY_S,
  confirmation_time_s: float = RADAR_ONLY_CERT_CONFIRMATION_TIME_S,
) -> float:
  """Distance required at first sighting for radar-only custom stopping authority."""
  if not math.isfinite(v_ego):
    return math.inf
  v = max(float(v_ego), 0.0)
  return (float(rest_floor_m)
          + (v * v) / (2.0 * float(min_decel))
          + v * (float(actuation_delay_s) + float(confirmation_time_s)))


class StoppingLeadAuthority:
  """Certify one selected lead track for custom stopping authority.

  A rejected close track stays rejected for its lifetime: slowing the ego must not shrink the
  acquisition horizon until the same false return self-certifies. A later vision association is
  independent evidence and may certify that track. Identity-less vision leads must meet the high
  confidence gate on every frame because their -1 sentinel cannot prove same-target persistence.

  ``model_prob=None`` preserves compatibility for direct LongControl callers that predate the
  provenance input. Production controlsd always supplies the radarState value.
  """

  def __init__(self) -> None:
    self.reset()

  def reset(self) -> None:
    self.track_id: int | None = None
    self.certified = False

  def update(self, *, v_ego: float, lead_status: bool, lead_d_rel: float,
             lead_track_id, model_prob: float | None) -> bool:
    if not lead_status:
      self.reset()
      return False

    if model_prob is None:
      # Compatibility only. The live controlsd seam always passes modelProb explicitly.
      self.reset()
      return True

    prob = float(model_prob) if math.isfinite(model_prob) else 0.0
    try:
      track_id = int(lead_track_id) if lead_track_id is not None else -1
    except (TypeError, ValueError, OverflowError):
      track_id = -1

    if track_id < 0:
      self.reset()
      return prob >= VISION_ONLY_CERT_PROB_MIN

    if track_id != self.track_id:
      self.track_id = track_id
      early_radar_acquisition = (
        math.isfinite(lead_d_rel)
        and float(lead_d_rel) > 0.0
        and float(lead_d_rel) >= get_radar_only_min_acquire_d_rel(v_ego)
      )
      self.certified = prob > 0.0 or early_radar_acquisition
    elif prob > 0.0:
      self.certified = True

    return self.certified
