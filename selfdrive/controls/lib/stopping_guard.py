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


def apply_should_stop_soft_landing(
  output_accel: float,
  last_output_accel: float,
  should_stop: bool,
  v_ego: float,
  a_ego: float,
  max_expected_accel: float,
) -> float:
  if not should_stop or v_ego >= 0.45:
    return output_accel

  # Disturbance handling should take priority; do not soften when response boost is needed.
  if a_ego > (max_expected_accel + 0.02):
    return output_accel

  hold_target = interp(v_ego, [0.0, 0.08, 0.20, 0.45], [-0.06, -0.08, -0.12, -0.22])
  blend = interp(v_ego, [0.0, 0.45], [0.82, 0.45])
  softened_output = (blend * output_accel) + ((1.0 - blend) * hold_target)

  brake_step = interp(v_ego, [0.0, 0.10, 0.25, 0.45], [0.002, 0.003, 0.005, 0.008])
  release_step = interp(v_ego, [0.0, 0.10, 0.25, 0.45], [0.0015, 0.0025, 0.004, 0.006])

  softened_output = max(softened_output, last_output_accel - brake_step)
  softened_output = min(softened_output, last_output_accel + release_step)
  return softened_output
