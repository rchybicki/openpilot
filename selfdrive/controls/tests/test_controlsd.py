from openpilot.selfdrive.controls.lib.drive_helpers import longitudinal_control_active


def test_longitudinal_control_active_with_gas_override() -> None:
  assert not longitudinal_control_active(True, True, False, True, False)
  assert longitudinal_control_active(True, True, False, True, True)


def test_longitudinal_control_active_preserves_other_gates() -> None:
  assert not longitudinal_control_active(False, True, False, True, True)
  assert not longitudinal_control_active(True, False, False, True, True)
  assert not longitudinal_control_active(True, True, True, True, True)
