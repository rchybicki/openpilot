def should_release_stop_hold_for_departing_lead(human_acceleration: bool, output_should_stop: bool,
                                                force_coast: bool, standstill: bool,
                                                v_ego: float, v_ego_starting: float,
                                                lead_status: bool, lead_v: float, lead_d_rel: float) -> bool:
  if not human_acceleration or not output_should_stop:
    return False

  if force_coast and standstill:
    return False

  lead_speed_floor = 0.15 if standstill else 0.4
  lead_rel_margin = 0.1 if standstill else 0.2
  lead_departing = (
    lead_status and
    lead_v > max(v_ego + lead_rel_margin, lead_speed_floor) and
    lead_d_rel > 2.5
  )
  near_standstill = standstill or v_ego < (v_ego_starting + 0.1)
  return near_standstill and lead_departing
