from openpilot.selfdrive.controls.lib.drive_helpers import longitudinal_accel_with_gas, longitudinal_control_active, longitudinal_control_override


def test_longitudinal_control_active_with_gas_override() -> None:
  assert not longitudinal_control_active(True, True, False, True, False, True)
  assert not longitudinal_control_active(True, True, False, True, True, False)
  assert longitudinal_control_active(True, True, False, True, True, True)


def test_longitudinal_control_active_preserves_other_gates() -> None:
  assert not longitudinal_control_active(False, True, False, True, True, True)
  assert not longitudinal_control_active(True, False, False, True, True, True)
  assert not longitudinal_control_active(True, True, True, True, True, True)


def test_longitudinal_accel_with_gas_is_one_sided() -> None:
  assert longitudinal_accel_with_gas(-1.0, True, True) == 0.0
  assert longitudinal_accel_with_gas(0.0, True, True) == 0.0
  assert longitudinal_accel_with_gas(1.0, True, True) == 1.0
  assert longitudinal_accel_with_gas(-1.0, False, True) == -1.0
  assert longitudinal_accel_with_gas(-1.0, True, False) == -1.0


def test_longitudinal_control_override_stays_set_during_active_gas() -> None:
  assert longitudinal_control_override(True, True, True, True, True)
  assert not longitudinal_control_override(True, True, True, True, False)
  assert longitudinal_control_override(True, True, False, False, False)
  assert not longitudinal_control_override(False, True, True, True, True)
