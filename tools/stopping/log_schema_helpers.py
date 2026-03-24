from __future__ import annotations


def _read_bool_attr(message: object, *names: str) -> bool | None:
  for name in names:
    try:
      value = getattr(message, name)
    except AttributeError:
      continue
    except Exception:
      continue

    try:
      return bool(value)
    except Exception:
      continue
  return None


def controls_state_enabled(state: object) -> bool | None:
  return _read_bool_attr(state, "enabled", "enabledDEPRECATED")


def selfdrive_state_enabled(state: object) -> bool | None:
  return _read_bool_attr(state, "enabled")


def selfdrive_state_engaged(state: object) -> bool | None:
  active = _read_bool_attr(state, "active")
  enabled = _read_bool_attr(state, "enabled")
  if active is not None:
    return active or bool(enabled)
  return enabled
