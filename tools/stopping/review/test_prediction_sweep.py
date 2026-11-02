import pytest

from openpilot.tools.stopping.review import prediction_sweep


@pytest.mark.parametrize('wheel_observation', [False, True])
@pytest.mark.parametrize('lead', [(0.0, 0.0), (1.0, 1.0)])
def test_sweep_integrates_constant_braking_to_the_analytic_rest_gap(monkeypatch, wheel_observation, lead):
  class ConstantBrake:
    def __init__(self, cp):
      pass

    def update(self, *args, **kwargs):
      return -1.0

  monkeypatch.setattr(prediction_sweep, 'LongControl', ConstantBrake)
  result = prediction_sweep.simulate(lambda: None, (2.0, 7.0, -1.0), (1.0, .4, .3, 0.0), lead, wheel_observation)
  # Ego travels v^2/(2b) = 2 m. The moving lead adds 0.5 m before rest.
  assert result['gap'] == pytest.approx(5.0 + (0.5 if lead[0] else 0.0), abs=1e-12)
  assert result['min_gap'] == result['gap']
  assert result['head_min'] == pytest.approx(-1.0)
  assert result['tail_min'] == pytest.approx(-1.0)
  assert result['stopped'] and not result['creep']
