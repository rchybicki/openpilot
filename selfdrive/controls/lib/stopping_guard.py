from openpilot.common.numpy_fast import clip, interp


def apply_should_stop_disturbance_guard(
  output_accel: float,
  last_output_accel: float,
  should_stop: bool,
  v_ego: float,
  a_ego: float,
  max_expected_accel: float,
  stopping_v_bp: list[float],
  dt: float,
) -> float:
  if not should_stop or v_ego >= 0.8:
    return output_accel

  disturbance = a_ego - max_expected_accel
  if disturbance <= 0.02:
    return output_accel

  guarded_output = min(output_accel, last_output_accel)
  disturbance_gain = interp(v_ego, stopping_v_bp, [3.0, 2.0, 1.2])
  guarded_output -= clip(disturbance, 0.0, 0.8) * disturbance_gain * dt
  return guarded_output
