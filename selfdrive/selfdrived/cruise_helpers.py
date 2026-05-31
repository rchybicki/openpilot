def cruise_mismatch_detected(cruise_enabled: bool, selfdrive_enabled: bool, pcm_cruise: bool) -> bool:
  return pcm_cruise and cruise_enabled and not selfdrive_enabled
