import numpy as np

from openpilot.tools.modeld.build_check import matches_reference

CONTROLS = np.array([2062, 2063])  # Cheddar's action means: desired lateral, then longitudinal acceleration


def check(tmp_path, ref, got):
  np.savez(tmp_path / 'reference.npz', outputs=ref, controls=CONTROLS)
  np.savez(tmp_path / 'outputs.npz', outputs=got)
  return matches_reference(tmp_path / 'reference.npz', tmp_path / 'outputs.npz')


def test_a_reversed_acceleration_fails(tmp_path):
  rng = np.random.default_rng(0)
  ref = rng.normal(0, 50, (8, 1, 2580)).astype(np.float32)
  ref[..., CONTROLS] = 1.09, -0.22
  got = ref + rng.normal(0, 0.005, ref.shape).astype(np.float32)
  assert check(tmp_path, ref, got)

  # 0.44 m/s^2 is under 1 % of the output range, so only the control check sees it
  got[..., CONTROLS[1]] *= -1
  assert not check(tmp_path, ref, got)
