"""Stopping-service telemetry (plan §2): phase-change events + one per-settle summary, all via
cloudlog.event('stopping_service', ...) so every §7 gate is computable from rlogs. Stage-1 shadow
divergence bookkeeping rides the same summary (max |wire-shadow|, fraction of frames the shadow is
shallower than the wire, shadow wheel-stop value). Bounded: <= ~20 events per stop."""

from __future__ import annotations

import math

from openpilot.common.swaglog import cloudlog

MAX_PHASE_EVENTS_PER_STOP = 16
MAX_TIMELINE_ENTRIES_PER_STOP = 64  # summary payload stays bounded even under phase chatter (R2-L1)


class StoppingTelemetry:
  def __init__(self, log_fn=None):
    self._log = log_fn if log_fn is not None else (lambda **kw: cloudlog.event("stopping_service", **kw))
    self._reset_settle()

  def _reset_settle(self) -> None:
    self._last_phase = "INACTIVE"
    self._t = 0.0
    self._phase_events = 0
    self._timeline: list[tuple[float, str]] = []
    self._frames = 0
    self._shallower_frames = 0
    self._max_divergence = 0.0
    self._wheel_stop_wire: float | None = None
    self._wheel_stop_shadow: float | None = None
    self._post_stop_v_max = 0.0
    self._min_gap: float | None = None
    self._rest_gap: float | None = None
    self._dts_at_settle: float | None = None

  def update(self, *, phase: str, active: bool, shadow_accel: float, wire_accel: float, v_ego: float,
             d_gap: float | None, dts: float | None, wheel_stop_latched: bool, dt: float) -> None:
    was_active = self._last_phase != "INACTIVE" or self._frames > 0
    if not active and not was_active:
      return
    self._t += dt
    if active:
      self._frames += 1
      if math.isfinite(wire_accel) and math.isfinite(shadow_accel):
        self._max_divergence = max(self._max_divergence, abs(wire_accel - shadow_accel))
        if shadow_accel > wire_accel + 1e-9:
          self._shallower_frames += 1
      if d_gap is not None and math.isfinite(d_gap):
        self._min_gap = d_gap if self._min_gap is None else min(self._min_gap, d_gap)
        self._rest_gap = d_gap
      if wheel_stop_latched and self._wheel_stop_wire is None:
        self._wheel_stop_wire, self._wheel_stop_shadow = float(wire_accel), float(shadow_accel)
        self._dts_at_settle = None if dts is None else float(dts)
      if self._wheel_stop_wire is not None and math.isfinite(v_ego):
        self._post_stop_v_max = max(self._post_stop_v_max, float(v_ego))
    if phase != self._last_phase:
      if len(self._timeline) < MAX_TIMELINE_ENTRIES_PER_STOP:
        self._timeline.append((round(self._t, 2), phase))
      if self._phase_events < MAX_PHASE_EVENTS_PER_STOP:
        self._phase_events += 1
        self._log(kind="phase_change", t=round(self._t, 2), phase=phase, prev_phase=self._last_phase,
                  wire_accel=float(wire_accel), shadow_accel=float(shadow_accel), v_ego=float(v_ego))
      self._last_phase = phase
    if not active:  # settle over: one summary event, then rearm
      self._log(kind="settle_summary", frames=self._frames, phase_timeline=self._timeline,
                wheel_stop_wire=self._wheel_stop_wire, wheel_stop_shadow=self._wheel_stop_shadow,
                post_stop_v_max=round(self._post_stop_v_max, 4), rest_gap=self._rest_gap,
                min_gap=self._min_gap, dts_at_settle=self._dts_at_settle,
                max_divergence=round(self._max_divergence, 4),
                shadow_shallower_frac=round(self._shallower_frames / max(self._frames, 1), 4))
      self._reset_settle()
