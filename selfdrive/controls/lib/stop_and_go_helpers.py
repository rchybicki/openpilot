import numpy as np

interp = np.interp


def should_release_stop_hold_for_departing_lead(human_acceleration: bool, output_should_stop: bool,
                                                force_coast: bool, standstill: bool,
                                                v_ego: float, v_ego_starting: float,
                                                lead_status: bool, lead_v: float, lead_d_rel: float) -> bool:
  if not human_acceleration or not output_should_stop:
    return False

  if force_coast and standstill:
    return False

  near_standstill = standstill or v_ego < (v_ego_starting + 0.1)
  if not near_standstill:
    return False

  lead_speed_floor = 0.15 if standstill else 0.4
  lead_rel_margin = 0.1 if standstill else 0.2
  if not lead_status or lead_v <= max(v_ego + lead_rel_margin, lead_speed_floor):
    return False

  return bool(lead_d_rel > departing_lead_release_gap(max(lead_v - v_ego, 0.0), standstill))


def departing_lead_release_gap(lead_departure_speed: float, standstill: bool) -> float:
  """The departing-lead room table: the gap a lead departing at this closing speed must have opened."""
  if standstill:
    return float(interp(lead_departure_speed, [0.15, 0.60, 1.00, 1.20, 2.00], [5.80, 5.40, 4.95, 4.30, 3.80]))
  return float(interp(lead_departure_speed, [0.40, 0.80, 1.20, 1.40, 2.20], [5.95, 5.50, 5.05, 4.55, 4.05]))
