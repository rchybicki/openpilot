from __future__ import annotations

from types import SimpleNamespace

from openpilot.tools.stopping.log_schema_helpers import controls_state_enabled, selfdrive_state_enabled, selfdrive_state_engaged


def test_controls_state_enabled_supports_current_field():
  assert controls_state_enabled(SimpleNamespace(enabled=True)) is True


def test_controls_state_enabled_supports_deprecated_field():
  assert controls_state_enabled(SimpleNamespace(enabledDEPRECATED=False)) is False


def test_selfdrive_state_enabled_reads_enabled_field():
  assert selfdrive_state_enabled(SimpleNamespace(enabled=True)) is True


def test_selfdrive_state_engaged_prefers_active_signal():
  assert selfdrive_state_engaged(SimpleNamespace(enabled=False, active=True)) is True
