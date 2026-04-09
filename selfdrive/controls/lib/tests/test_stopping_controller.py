import numpy as np
from dataclasses import dataclass

from openpilot.selfdrive.controls.lib.stopping_controller import StoppingController, StoppingPhase
from openpilot.tools.stopping.check_harsh_stops_model import simulate_event_with_controller
from openpilot.tools.stopping.stopping_model import FittedStoppingModel

interp = np.interp


@dataclass
class FakeSample:
  t: float
  v_ego: float
  a_ego: float
  accel_cmd: float | None
  should_stop: bool = True


def _run_direct_controller_seed(samples: list[FakeSample]) -> tuple[list[float], list[tuple[str, ...]]]:
  controller = StoppingController()
  last_output = float(samples[0].accel_cmd) if samples[0].accel_cmd is not None else -0.12
  controller.seed_command_history([last_output])
  outputs: list[float] = []
  triggers: list[tuple[str, ...]] = []

  for idx, sample in enumerate(samples):
    if idx + 1 < len(samples):
      dt = max(float(samples[idx + 1].t - sample.t), 0.01)
    elif idx > 0:
      dt = max(float(sample.t - samples[idx - 1].t), 0.01)
    else:
      dt = 0.10
    output_seed = min(last_output, -0.10) if sample.should_stop else (float(sample.accel_cmd) if sample.accel_cmd is not None else last_output)
    debug: dict[str, object] = {}
    result = controller.update(
      output_accel=output_seed,
      last_output_accel=last_output,
      should_stop=sample.should_stop,
      v_ego=sample.v_ego,
      a_ego=sample.a_ego,
      max_expected_accel=interp(sample.v_ego, [0.01, 0.20, 0.50], [-0.01, -0.10, -0.30]),
      min_expected_accel=interp(sample.v_ego, [0.01, 0.20, 0.50], [-0.10, -0.50, -1.00]),
      stop_accel=-2.0,
      dt=dt,
      debug=debug,
    )
    outputs.append(result.output_accel)
    triggers.append(tuple(debug.get("triggers", ())))
    last_output = result.output_accel

  return outputs, triggers


def _build_entry_seed_samples_7af_event2() -> list[FakeSample]:
  return [
    FakeSample(t=263.484399298, v_ego=1.462291121, a_ego=-0.105799153, accel_cmd=-0.214938283, should_stop=False),
    FakeSample(t=263.584130817, v_ego=1.448519468, a_ego=-0.127950937, accel_cmd=-0.237445548, should_stop=False),
    FakeSample(t=263.685249251, v_ego=1.433086872, a_ego=-0.149000525, accel_cmd=-0.283079445, should_stop=False),
    FakeSample(t=263.783643332, v_ego=1.410861969, a_ego=-0.195912838, accel_cmd=-0.342900723, should_stop=False),
    FakeSample(t=263.884882756, v_ego=1.387841940, a_ego=-0.218380198, accel_cmd=0.000000000, should_stop=True),
    FakeSample(t=263.983988341, v_ego=1.363762736, a_ego=-0.229979128, accel_cmd=0.000000000, should_stop=True),
    FakeSample(t=264.082932056, v_ego=1.340931654, a_ego=-0.220820636, accel_cmd=0.000000000, should_stop=True),
    FakeSample(t=264.183797367, v_ego=1.329383492, a_ego=-0.150356293, accel_cmd=0.000000000, should_stop=True),
    FakeSample(t=264.285016632, v_ego=1.324232459, a_ego=-0.086433165, accel_cmd=0.000000000, should_stop=True),
    FakeSample(t=264.384668204, v_ego=1.322401881, a_ego=-0.046979774, accel_cmd=0.000000000, should_stop=True),
  ]


def _build_entry_seed_samples_7b1_event32() -> list[FakeSample]:
  return [
    FakeSample(t=2207.535658536, v_ego=0.027568448, a_ego=-0.005644393, accel_cmd=0.153708488, should_stop=False),
    FakeSample(t=2207.637174622, v_ego=0.033372741, a_ego=0.047886256, accel_cmd=0.185088664, should_stop=False),
    FakeSample(t=2207.737007703, v_ego=0.069725335, a_ego=0.289012790, accel_cmd=0.200000003, should_stop=False),
    FakeSample(t=2207.835392463, v_ego=0.209481418, a_ego=1.032333970, accel_cmd=0.200000003, should_stop=False),
    FakeSample(t=2207.936399283, v_ego=0.322090089, a_ego=1.050205112, accel_cmd=-0.083577037, should_stop=True),
    FakeSample(t=2208.036518090, v_ego=0.388150752, a_ego=0.786834478, accel_cmd=-0.154427379, should_stop=True),
    FakeSample(t=2208.136542679, v_ego=0.437088996, a_ego=0.594839275, accel_cmd=-0.212038368, should_stop=True),
    FakeSample(t=2208.235807379, v_ego=0.478606373, a_ego=0.487165987, accel_cmd=-0.218240380, should_stop=True),
    FakeSample(t=2208.337222737, v_ego=0.510853410, a_ego=0.370992869, accel_cmd=-0.223946974, should_stop=True),
    FakeSample(t=2208.436449677, v_ego=0.526187658, a_ego=0.225371853, accel_cmd=-0.228857309, should_stop=True),
  ]


def _build_entry_seed_samples_7df_event1() -> list[FakeSample]:
  return [
    FakeSample(t=330.812011423, v_ego=1.159697413, a_ego=-0.792917013, accel_cmd=-1.000838876, should_stop=False),
    FakeSample(t=330.912475404, v_ego=1.077776909, a_ego=-0.811309099, accel_cmd=-1.000718713, should_stop=False),
    FakeSample(t=331.011755242, v_ego=0.992702484, a_ego=-0.839934587, accel_cmd=-0.971027136, should_stop=False),
    FakeSample(t=331.112167766, v_ego=0.907390833, a_ego=-0.832116127, accel_cmd=-0.952474475, should_stop=False),
    FakeSample(t=331.212928461, v_ego=0.825609744, a_ego=-0.816915631, accel_cmd=-0.939834774, should_stop=True),
    FakeSample(t=331.311668465, v_ego=0.744314611, a_ego=-0.805096149, accel_cmd=-0.884141147, should_stop=True),
    FakeSample(t=331.410912470, v_ego=0.671971142, a_ego=-0.745494485, accel_cmd=-0.747810185, should_stop=True),
    FakeSample(t=331.511489991, v_ego=0.604755640, a_ego=-0.687742114, accel_cmd=-0.615327895, should_stop=True),
    FakeSample(t=331.611858662, v_ego=0.546905577, a_ego=-0.618791938, accel_cmd=-0.543969572, should_stop=True),
    FakeSample(t=331.711341360, v_ego=0.495296389, a_ego=-0.552489281, accel_cmd=-0.543969572, should_stop=True),
  ]


def _build_stopping_reacquire_seed_samples_9ac_event2() -> list[FakeSample]:
  return [
    FakeSample(t=521.403032731, v_ego=0.297338486, a_ego=-1.070478797, accel_cmd=-0.980953395, should_stop=False),
    FakeSample(t=521.502849722, v_ego=0.214499906, a_ego=-0.904351115, accel_cmd=-0.864293396, should_stop=False),
    FakeSample(t=521.601347976, v_ego=0.141136810, a_ego=-0.781276882, accel_cmd=-0.707487643, should_stop=False),
    FakeSample(t=521.702698702, v_ego=0.099888213, a_ego=-0.537645459, accel_cmd=-0.515429616, should_stop=False),
    FakeSample(t=521.802447152, v_ego=0.086185858, a_ego=-0.275143057, accel_cmd=-0.317740589, should_stop=True),
    FakeSample(t=521.901565037, v_ego=0.079046637, a_ego=-0.152883589, accel_cmd=-0.297235250, should_stop=True),
    FakeSample(t=522.001919523, v_ego=0.065799490, a_ego=-0.141226009, accel_cmd=-0.303850830, should_stop=True),
    FakeSample(t=522.102898378, v_ego=0.054044496, a_ego=-0.125682548, accel_cmd=-0.313775867, should_stop=True),
    FakeSample(t=522.201755067, v_ego=0.047495317, a_ego=-0.087695718, accel_cmd=-0.321322739, should_stop=True),
    FakeSample(t=522.302619808, v_ego=0.043239050, a_ego=-0.058939654, accel_cmd=-0.338177979, should_stop=True),
  ]


def _build_stopping_reacquire_seed_samples_9ac_event4() -> list[FakeSample]:
  return [
    FakeSample(t=619.599808893, v_ego=1.076719999, a_ego=-0.818989694, accel_cmd=-0.792510927, should_stop=False),
    FakeSample(t=619.700212844, v_ego=0.995236278, a_ego=-0.812738359, accel_cmd=-0.792510927, should_stop=False),
    FakeSample(t=619.800025863, v_ego=0.916866899, a_ego=-0.793896139, accel_cmd=-0.792510927, should_stop=False),
    FakeSample(t=619.900097109, v_ego=0.833634675, a_ego=-0.812783301, accel_cmd=-0.778562844, should_stop=True),
    FakeSample(t=620.000027784, v_ego=0.746184289, a_ego=-0.850445390, accel_cmd=-0.641495347, should_stop=True),
    FakeSample(t=620.100303924, v_ego=0.669239759, a_ego=-0.805857241, accel_cmd=-0.540490627, should_stop=True),
    FakeSample(t=620.200118662, v_ego=0.602115154, a_ego=-0.724697590, accel_cmd=-0.518840492, should_stop=True),
    FakeSample(t=620.299333250, v_ego=0.542388797, a_ego=-0.636744142, accel_cmd=-0.514168382, should_stop=True),
    FakeSample(t=620.401071094, v_ego=0.488954723, a_ego=-0.557200849, accel_cmd=-0.514168382, should_stop=True),
    FakeSample(t=620.500375785, v_ego=0.442379415, a_ego=-0.494394422, accel_cmd=-0.499707878, should_stop=True),
    FakeSample(t=620.600512499, v_ego=0.400419176, a_ego=-0.436262548, accel_cmd=-0.430417299, should_stop=True),
  ]


def _build_micro_stop_capture_seed_samples_9ca_event2() -> list[FakeSample]:
  return [
    FakeSample(t=3.898724687, v_ego=0.017468168, a_ego=0.002441406, accel_cmd=0.000000000, should_stop=False),
    FakeSample(t=3.996828411, v_ego=0.017042903, a_ego=0.001321100, accel_cmd=0.000000000, should_stop=False),
    FakeSample(t=4.099278079, v_ego=0.016916201, a_ego=-0.000149482, accel_cmd=0.000000000, should_stop=False),
    FakeSample(t=4.198192661, v_ego=0.016840432, a_ego=-0.000278155, accel_cmd=0.000000000, should_stop=False),
    FakeSample(t=4.297290085, v_ego=0.016447655, a_ego=-0.012723684, accel_cmd=0.000000000, should_stop=False),
    FakeSample(t=4.397390535, v_ego=0.031388983, a_ego=0.116443165, accel_cmd=-0.050000001, should_stop=True),
    FakeSample(t=4.498958058, v_ego=0.063886203, a_ego=0.282332808, accel_cmd=-0.267736226, should_stop=True),
    FakeSample(t=4.597333174, v_ego=0.159374788, a_ego=0.663361788, accel_cmd=-0.358418286, should_stop=True),
    FakeSample(t=4.696605827, v_ego=0.182791904, a_ego=0.365437985, accel_cmd=-0.418661803, should_stop=True),
    FakeSample(t=4.797579892, v_ego=0.169683948, a_ego=0.042232513, accel_cmd=-0.431692123, should_stop=True),
    FakeSample(t=4.896844526, v_ego=0.155748129, a_ego=-0.070561841, accel_cmd=-0.422903031, should_stop=True),
    FakeSample(t=4.997710455, v_ego=0.135383829, a_ego=-0.156435966, accel_cmd=-0.412529171, should_stop=True),
  ]


def _build_terminal_unwind_seed_samples_9cc_event1() -> list[FakeSample]:
  return [
    FakeSample(t=358.674908840, v_ego=0.761260211, a_ego=-0.437675238, accel_cmd=-0.462200791, should_stop=False),
    FakeSample(t=358.772687246, v_ego=0.723713994, a_ego=-0.400083989, accel_cmd=-0.462200791, should_stop=True),
    FakeSample(t=358.872980535, v_ego=0.687956095, a_ego=-0.368007690, accel_cmd=-0.462200791, should_stop=True),
    FakeSample(t=358.974055158, v_ego=0.660024822, a_ego=-0.313347340, accel_cmd=-0.462200791, should_stop=True),
    FakeSample(t=359.071898198, v_ego=0.634235442, a_ego=-0.274879605, accel_cmd=-0.462200791, should_stop=True),
    FakeSample(t=359.171950660, v_ego=0.607462704, a_ego=-0.264940172, accel_cmd=-0.462304354, should_stop=True),
    FakeSample(t=359.273828804, v_ego=0.583697200, a_ego=-0.246404737, accel_cmd=-0.483857930, should_stop=True),
    FakeSample(t=359.373542160, v_ego=0.565125346, a_ego=-0.204104885, accel_cmd=-0.564467728, should_stop=True),
    FakeSample(t=359.473555198, v_ego=0.546639204, a_ego=-0.193729609, accel_cmd=-0.645053923, should_stop=True),
    FakeSample(t=359.574010724, v_ego=0.517577291, a_ego=-0.260471731, accel_cmd=-0.634467840, should_stop=True),
    FakeSample(t=359.671633147, v_ego=0.474571139, a_ego=-0.370945454, accel_cmd=-0.610117137, should_stop=True),
    FakeSample(t=359.774743501, v_ego=0.432099104, a_ego=-0.410210073, accel_cmd=-0.578307569, should_stop=True),
  ]


def _build_terminal_unwind_seed_samples_9cb_event3() -> list[FakeSample]:
  return [
    FakeSample(t=231.448231077, v_ego=0.713791311, a_ego=-0.612206221, accel_cmd=-0.731831789, should_stop=False),
    FakeSample(t=231.549071463, v_ego=0.646816671, a_ego=-0.644291639, accel_cmd=-0.759498596, should_stop=True),
    FakeSample(t=231.649106351, v_ego=0.581420064, a_ego=-0.644273162, accel_cmd=-0.759498596, should_stop=True),
    FakeSample(t=231.749823399, v_ego=0.516437232, a_ego=-0.647310615, accel_cmd=-0.721596599, should_stop=True),
    FakeSample(t=231.848742778, v_ego=0.454003036, a_ego=-0.623767972, accel_cmd=-0.648747742, should_stop=True),
    FakeSample(t=231.948768082, v_ego=0.396133721, a_ego=-0.588116050, accel_cmd=-0.594327033, should_stop=True),
    FakeSample(t=232.048975633, v_ego=0.339765638, a_ego=-0.571175456, accel_cmd=-0.535477996, should_stop=True),
    FakeSample(t=232.150352497, v_ego=0.287679136, a_ego=-0.543829203, accel_cmd=-0.482001156, should_stop=True),
    FakeSample(t=232.249189216, v_ego=0.242891490, a_ego=-0.491425872, accel_cmd=-0.444914907, should_stop=True),
    FakeSample(t=232.348893349, v_ego=0.201059312, a_ego=-0.463610828, accel_cmd=-0.407420069, should_stop=True),
    FakeSample(t=232.450398320, v_ego=0.172828645, a_ego=-0.338800699, accel_cmd=-0.336178392, should_stop=True),
    FakeSample(t=232.549137775, v_ego=0.129863515, a_ego=-0.392228127, accel_cmd=-0.308443636, should_stop=True),
  ]


def _build_terminal_unwind_seed_samples_9cb_event4() -> list[FakeSample]:
  return [
    FakeSample(t=479.457302973, v_ego=0.906160474, a_ego=-0.599070430, accel_cmd=-0.697911620, should_stop=False),
    FakeSample(t=479.559776334, v_ego=0.845415354, a_ego=-0.609256089, accel_cmd=-0.697911620, should_stop=True),
    FakeSample(t=479.660149153, v_ego=0.789358914, a_ego=-0.583074927, accel_cmd=-0.670008063, should_stop=True),
    FakeSample(t=479.759841916, v_ego=0.737155318, a_ego=-0.541037261, accel_cmd=-0.554610908, should_stop=True),
    FakeSample(t=479.860277447, v_ego=0.688644409, a_ego=-0.504772604, accel_cmd=-0.544902146, should_stop=True),
    FakeSample(t=479.958844001, v_ego=0.644144535, a_ego=-0.460213482, accel_cmd=-0.544902146, should_stop=True),
    FakeSample(t=480.059810890, v_ego=0.598607719, a_ego=-0.455954343, accel_cmd=-0.544902146, should_stop=True),
    FakeSample(t=480.159819540, v_ego=0.554897368, a_ego=-0.441739053, accel_cmd=-0.544902146, should_stop=True),
    FakeSample(t=480.261139818, v_ego=0.511352301, a_ego=-0.438516915, accel_cmd=-0.544902146, should_stop=True),
    FakeSample(t=480.358372407, v_ego=0.467265159, a_ego=-0.437474102, accel_cmd=-0.544902146, should_stop=True),
    FakeSample(t=480.459827483, v_ego=0.422982156, a_ego=-0.439656466, accel_cmd=-0.544902146, should_stop=True),
    FakeSample(t=480.559889681, v_ego=0.381749421, a_ego=-0.418753564, accel_cmd=-0.501383722, should_stop=True),
  ]


def test_stopping_controller_stop_entry_soften_reduces_mid_speed_initial_bite_seed_000007af_event2():
  outputs, triggers = _run_direct_controller_seed(_build_entry_seed_samples_7af_event2())
  assert outputs[4] > -0.40
  assert "stop_entry_soften" in triggers[4]


def test_stopping_controller_stop_entry_soften_reduces_positive_to_brake_snap_seed_000007b1_event32():
  outputs, triggers = _run_direct_controller_seed(_build_entry_seed_samples_7b1_event32())
  assert outputs[4] > -0.04
  assert "stop_entry_soften" in triggers[4]


def test_stopping_controller_stop_entry_soften_does_not_over_relax_deep_brake_seed_000007df_event1():
  outputs, triggers = _run_direct_controller_seed(_build_entry_seed_samples_7df_event1())
  assert outputs[4] < -0.78
  assert "stop_entry_soften" not in triggers[4]


def test_stopping_controller_stop_reacquire_hold_preserves_built_brake_seed_000009ac_event2():
  outputs, triggers = _run_direct_controller_seed(_build_stopping_reacquire_seed_samples_9ac_event2())
  assert outputs[4] < -0.95
  assert outputs[8] < -0.95
  assert "stop_reacquire_hold" in triggers[4]
  assert "stop_reacquire_hold" in triggers[8]


def test_stopping_controller_stop_reacquire_hold_avoids_early_unwind_seed_000009ac_event4():
  outputs, triggers = _run_direct_controller_seed(_build_stopping_reacquire_seed_samples_9ac_event4())
  assert outputs[3] < -0.79
  assert outputs[5] < -0.79
  assert outputs[6] < -0.65
  assert "stop_reacquire_hold" in triggers[3]
  assert "stop_reacquire_hold" in triggers[5]
  assert "high_speed_reacquire_soften" not in triggers[3]


def test_stopping_controller_micro_stopgo_capture_controls_late_standstill_restart_seed_000009ca_event2():
  outputs, triggers = _run_direct_controller_seed(_build_micro_stop_capture_seed_samples_9ca_event2())
  assert outputs[5] > -0.24
  assert outputs[6] > -0.35
  assert outputs[7] < -0.40
  assert "micro_stopgo_soft_capture" in triggers[5]
  assert "micro_stopgo_soft_capture" in triggers[6]


def test_stopping_controller_terminal_unwind_delay_blocks_no_target_distance_carry_seed_000009cc_event1():
  outputs, triggers = _run_direct_controller_seed(_build_terminal_unwind_seed_samples_9cc_event1())
  assert outputs[10] < -0.53
  assert "distance_carry_settle" not in triggers[10]
  assert "end_stop_cap_active" not in triggers[10]


def test_stopping_controller_terminal_unwind_delay_preserves_built_brake_seed_000009cb_event3():
  outputs, triggers = _run_direct_controller_seed(_build_terminal_unwind_seed_samples_9cb_event3())
  assert outputs[1] > -0.74
  assert outputs[3] > -0.74
  assert outputs[10] < -0.72
  assert outputs[11] < -0.72
  assert "high_speed_reacquire_soften" in triggers[1]
  assert "high_speed_reacquire_soften" in triggers[3]
  assert "terminal_unwind_delay" in triggers[4]
  assert "terminal_unwind_delay" in triggers[10]
  assert "low_rollout_soft_landing_cap" not in triggers[10]


def test_stopping_controller_terminal_unwind_delay_avoids_late_soft_release_seed_000009cb_event4():
  outputs, triggers = _run_direct_controller_seed(_build_terminal_unwind_seed_samples_9cb_event4())
  assert outputs[1] > -0.75
  assert outputs[2] > -0.75
  assert outputs[3] < -0.75
  assert outputs[10] < -0.75
  assert "high_speed_reacquire_soften" in triggers[1]
  assert "high_speed_reacquire_soften" in triggers[2]
  assert "soft_landing_release" not in triggers[4]
  assert "terminal_unwind_delay" in triggers[4]
  assert "terminal_unwind_delay" in triggers[8]


def test_stopping_controller_passes_through_when_should_stop_false():
  controller = StoppingController()
  _ = controller.update(
    output_accel=-0.18,
    last_output_accel=-0.20,
    should_stop=True,
    v_ego=0.3,
    a_ego=0.2,
    max_expected_accel=-0.1,
    min_expected_accel=-0.5,
    stop_accel=-2.0,
    dt=0.01,
  )
  assert controller.release_lock_counter > 0

  result = controller.update(
    output_accel=-0.12,
    last_output_accel=-0.12,
    should_stop=False,
    v_ego=0.3,
    a_ego=-0.1,
    max_expected_accel=-0.1,
    min_expected_accel=-0.5,
    stop_accel=-2.0,
    dt=0.01,
  )
  assert result.output_accel == -0.12
  assert not result.release_lock_active
  assert controller.release_lock_counter == 0
  assert controller.low_speed_rollout_m == 0.0
  assert controller.phase == StoppingPhase.APPROACH


def test_stopping_controller_prefers_planner_distance_to_stop_target_when_available():
  controller = StoppingController()
  debug_without_plan: dict[str, object] = {}
  controller.update(
    output_accel=-0.24,
    last_output_accel=-0.26,
    should_stop=True,
    v_ego=0.35,
    a_ego=-0.25,
    max_expected_accel=-0.20,
    min_expected_accel=-0.50,
    stop_accel=-2.0,
    dt=0.1,
    debug=debug_without_plan,
  )

  controller.reset()
  debug_with_plan: dict[str, object] = {}
  controller.update(
    output_accel=-0.24,
    last_output_accel=-0.26,
    should_stop=True,
    v_ego=0.35,
    a_ego=-0.25,
    max_expected_accel=-0.20,
    min_expected_accel=-0.50,
    stop_accel=-2.0,
    dt=0.1,
    distance_to_stop_target_m=1.75,
    debug=debug_with_plan,
  )

  assert float(debug_without_plan["remaining_m"]) < 0.30
  assert debug_with_plan["distance_to_stop_target_m"] == 1.75
  assert float(debug_with_plan["remaining_m"]) == 1.75


def test_stopping_controller_distance_carry_settle_prefers_shallow_profile_when_target_is_still_ahead():
  without_plan = StoppingController()
  without_plan.phase = StoppingPhase.NEAR_HOLD
  without_plan.low_speed_rollout_m = 0.30
  debug_without_plan: dict[str, object] = {}
  result_without_plan = without_plan.update(
    output_accel=-0.50,
    last_output_accel=-0.50,
    should_stop=True,
    v_ego=0.50,
    a_ego=-0.20,
    max_expected_accel=-0.10,
    min_expected_accel=-0.50,
    stop_accel=-2.0,
    dt=0.01,
    debug=debug_without_plan,
  )

  with_plan = StoppingController()
  with_plan.phase = StoppingPhase.NEAR_HOLD
  with_plan.low_speed_rollout_m = 0.30
  debug_with_plan: dict[str, object] = {}
  result_with_plan = with_plan.update(
    output_accel=-0.50,
    last_output_accel=-0.50,
    should_stop=True,
    v_ego=0.50,
    a_ego=-0.20,
    max_expected_accel=-0.10,
    min_expected_accel=-0.50,
    stop_accel=-2.0,
    dt=0.01,
    distance_to_stop_target_m=0.28,
    debug=debug_with_plan,
  )

  assert "distance_carry_settle" not in debug_without_plan["triggers"]
  assert "distance_carry_settle" in debug_with_plan["triggers"]


def test_stopping_controller_near_hold_moves_toward_hold_target():
  controller = StoppingController()
  result = controller.update(
    output_accel=-0.05,
    last_output_accel=-0.05,
    should_stop=True,
    v_ego=0.20,
    a_ego=-0.12,
    max_expected_accel=-0.15,
    min_expected_accel=-0.5,
    stop_accel=-2.0,
    dt=0.01,
  )
  assert controller.phase == StoppingPhase.NEAR_HOLD
  assert result.output_accel < -0.055
  assert result.output_accel > -0.08


def test_stopping_controller_preserves_brake_on_active_release_dropout_seed_000007b3_event12():
  controller = StoppingController()
  controller.low_speed_rollout_m = 0.20
  result = controller.update(
    output_accel=-0.20146216452121735,
    last_output_accel=-0.3661329746246338,
    should_stop=False,
    v_ego=0.173,
    a_ego=-0.240,
    max_expected_accel=-0.05,
    min_expected_accel=-0.50,
    stop_accel=-2.0,
    dt=0.01,
    debug={},
  )
  assert result.output_accel < -0.34


def test_stopping_controller_preserves_brake_on_active_release_dropout_seed_000007b3_event6():
  controller = StoppingController()
  controller.low_speed_rollout_m = 0.12
  result = controller.update(
    output_accel=-0.24815623462200165,
    last_output_accel=-0.2646101117134094,
    should_stop=False,
    v_ego=0.081,
    a_ego=-0.284,
    max_expected_accel=-0.03,
    min_expected_accel=-0.50,
    stop_accel=-2.0,
    dt=0.01,
    debug={},
  )
  assert result.output_accel < -0.255


def test_stopping_controller_tail_commit_latch_preserves_mid_low_speed_brake_on_dropout():
  controller = StoppingController()
  controller.tail_commit_counter = 8
  controller.low_speed_rollout_m = 1.35
  debug: dict[str, object] = {}
  result = controller.update(
    output_accel=0.10,
    last_output_accel=-0.42,
    should_stop=False,
    v_ego=0.38,
    a_ego=-0.28,
    max_expected_accel=-0.08,
    min_expected_accel=-0.50,
    stop_accel=-2.0,
    dt=0.01,
    debug=debug,
  )
  assert result.output_accel < -0.41
  assert debug["tail_commit_active"] is True
  assert "glide_handoff" in debug["triggers"]


def test_stopping_controller_final_high_rollout_settle_guard_triggers_in_weak_decel_tail():
  controller = StoppingController()
  controller.low_speed_rollout_m = 1.30
  debug: dict[str, object] = {}
  result = controller.update(
    output_accel=-0.05,
    last_output_accel=-0.33,
    should_stop=True,
    v_ego=0.12,
    a_ego=-0.05,
    max_expected_accel=-0.10,
    min_expected_accel=-0.50,
    stop_accel=-2.0,
    dt=0.01,
    debug=debug,
  )
  assert result.output_accel < -0.35
  assert "final_high_rollout_settle_guard" in debug["triggers"]


def test_stopping_controller_clean_settle_profile_waits_when_rebound_cap_lane_owns_moderate_rollout_tail():
  controller = StoppingController()
  controller.low_speed_rollout_m = 0.60
  controller.phase = StoppingPhase.HOLD
  debug: dict[str, object] = {}
  result = controller.update(
    output_accel=-0.33,
    last_output_accel=-0.33,
    should_stop=True,
    v_ego=0.055,
    a_ego=-0.10,
    max_expected_accel=-0.02,
    min_expected_accel=-0.50,
    stop_accel=-2.0,
    dt=0.01,
    debug=debug,
  )
  triggers = debug.get("triggers", ())
  assert "low_speed_rebound_cap_active" in triggers
  assert "clean_settle_profile" not in triggers
  assert result.output_accel < -0.33


def test_stopping_controller_enters_near_hold_at_mid_low_speeds():
  controller = StoppingController()
  result = controller.update(
    output_accel=-0.09,
    last_output_accel=-0.09,
    should_stop=True,
    v_ego=0.70,
    a_ego=-0.08,
    max_expected_accel=-0.12,
    min_expected_accel=-0.50,
    stop_accel=-2.0,
    dt=0.01,
  )
  assert controller.phase == StoppingPhase.NEAR_HOLD
  assert result.output_accel < -0.094


def test_stopping_controller_near_hold_target_is_not_overly_deep_for_light_stopping():
  controller = StoppingController()
  output = -0.05
  for _ in range(40):
    result = controller.update(
      output_accel=-0.05,
      last_output_accel=output,
      should_stop=True,
      v_ego=0.20,
      a_ego=-0.10,
      max_expected_accel=-0.10,
      min_expected_accel=-0.50,
      stop_accel=-2.0,
      dt=0.01,
    )
    output = result.output_accel
  assert controller.phase == StoppingPhase.NEAR_HOLD
  assert output > -0.21


def test_stopping_controller_disturbance_sets_release_lock():
  controller = StoppingController()
  result = controller.update(
    output_accel=-0.18,
    last_output_accel=-0.20,
    should_stop=True,
    v_ego=0.30,
    a_ego=0.20,
    max_expected_accel=-0.10,
    min_expected_accel=-0.50,
    stop_accel=-2.0,
    dt=0.01,
  )
  assert result.release_lock_active
  assert controller.release_lock_counter > 0


def test_stopping_controller_low_speed_disturbance_sets_release_lock():
  controller = StoppingController()
  result = controller.update(
    output_accel=-0.10,
    last_output_accel=-0.275,
    should_stop=True,
    v_ego=0.045,
    a_ego=0.03,
    max_expected_accel=-0.02,
    min_expected_accel=-0.50,
    stop_accel=-2.0,
    dt=0.01,
  )
  assert result.release_lock_active
  assert controller.release_lock_counter > 0


def test_stopping_controller_regression_seed_20260212_leapfrog_onset_sets_release_lock():
  # Seeded from engaged stop-event onset samples before rebound on:
  # - route_000006f1--1eeed096b0 event 3
  # - route_000006f2--ef82b286ad event 3
  for v_ego, a_ego, max_expected_accel in (
    (0.041427, 0.016056, -0.0249),
    (0.041805, 0.022024, -0.0251),
  ):
    controller = StoppingController()
    result = controller.update(
      output_accel=-0.10,
      last_output_accel=-0.275,
      should_stop=True,
      v_ego=v_ego,
      a_ego=a_ego,
      max_expected_accel=max_expected_accel,
      min_expected_accel=-0.50,
      stop_accel=-2.0,
      dt=0.01,
    )
    assert result.release_lock_active
    assert controller.release_lock_counter > 0


def test_stopping_controller_release_lock_tightens_release_step():
  locked_controller = StoppingController()
  _ = locked_controller.update(
    output_accel=-0.22,
    last_output_accel=-0.24,
    should_stop=True,
    v_ego=1.10,
    a_ego=0.20,
    max_expected_accel=-0.10,
    min_expected_accel=-0.50,
    stop_accel=-2.0,
    dt=0.01,
  )
  locked_result = locked_controller.update(
    output_accel=0.05,
    last_output_accel=-0.24,
    should_stop=True,
    v_ego=1.10,
    a_ego=-0.20,
    max_expected_accel=-0.10,
    min_expected_accel=-0.50,
    stop_accel=-2.0,
    dt=0.01,
  )
  assert locked_result.release_lock_active

  unlocked_controller = StoppingController()
  unlocked_result = unlocked_controller.update(
    output_accel=0.05,
    last_output_accel=-0.24,
    should_stop=True,
    v_ego=1.10,
    a_ego=-0.20,
    max_expected_accel=-0.10,
    min_expected_accel=-0.50,
    stop_accel=-2.0,
    dt=0.01,
  )
  assert not unlocked_result.release_lock_active
  assert locked_result.output_accel < unlocked_result.output_accel - 1e-4


def test_stopping_controller_low_speed_disturbance_applies_extra_brake():
  disturbed_controller = StoppingController()
  disturbed_result = disturbed_controller.update(
    output_accel=-0.10,
    last_output_accel=-0.275,
    should_stop=True,
    v_ego=0.045,
    a_ego=0.03,
    max_expected_accel=-0.02,
    min_expected_accel=-0.50,
    stop_accel=-2.0,
    dt=0.01,
  )

  nominal_controller = StoppingController()
  nominal_result = nominal_controller.update(
    output_accel=-0.10,
    last_output_accel=-0.275,
    should_stop=True,
    v_ego=0.045,
    a_ego=-0.02,
    max_expected_accel=-0.02,
    min_expected_accel=-0.50,
    stop_accel=-2.0,
    dt=0.01,
  )

  assert disturbed_result.output_accel < nominal_result.output_accel - 1e-4


def test_stopping_controller_low_speed_rebound_cap_brakes_more_when_decel_weakens():
  weak_decel_controller = StoppingController()
  for _ in range(5):
    _ = weak_decel_controller.update(
      output_accel=-0.275,
      last_output_accel=-0.275,
      should_stop=True,
      v_ego=0.06,
      a_ego=-0.30,
      max_expected_accel=-0.02,
      min_expected_accel=-0.50,
      stop_accel=-2.0,
      dt=0.01,
    )
  weak_decel_result = weak_decel_controller.update(
    output_accel=-0.275,
    last_output_accel=-0.275,
    should_stop=True,
    v_ego=0.03,
    a_ego=-0.16,
    max_expected_accel=-0.015,
    min_expected_accel=-0.50,
    stop_accel=-2.0,
    dt=0.01,
  )

  nominal_controller = StoppingController()
  for _ in range(5):
    _ = nominal_controller.update(
      output_accel=-0.275,
      last_output_accel=-0.275,
      should_stop=True,
      v_ego=0.06,
      a_ego=-0.30,
      max_expected_accel=-0.02,
      min_expected_accel=-0.50,
      stop_accel=-2.0,
      dt=0.01,
    )
  nominal_result = nominal_controller.update(
    output_accel=-0.275,
    last_output_accel=-0.275,
    should_stop=True,
    v_ego=0.03,
    a_ego=-0.40,
    max_expected_accel=-0.015,
    min_expected_accel=-0.50,
    stop_accel=-2.0,
    dt=0.01,
  )

  assert weak_decel_result.output_accel < nominal_result.output_accel - 1e-4


def test_stopping_controller_low_speed_rebound_cap_active_extends_up_to_point_one_mps():
  controller = StoppingController()
  debug: dict[str, object] = {}
  result = controller.update(
    output_accel=-0.10,
    last_output_accel=-0.10,
    should_stop=True,
    v_ego=0.055,
    a_ego=-0.10,
    max_expected_accel=-0.02,
    min_expected_accel=-0.50,
    stop_accel=-2.0,
    dt=0.01,
    debug=debug,
  )

  triggers = debug.get("triggers", ())
  assert "low_speed_rebound_cap_active" in triggers
  assert result.output_accel < -0.10


def test_stopping_controller_rebound_arrest_arms_only_below_low_speed_gate():
  controller = StoppingController()
  for _ in range(520):
    _ = controller.update(
      output_accel=-0.275,
      last_output_accel=-0.275,
      should_stop=True,
      v_ego=0.07,
      a_ego=-0.30,
      max_expected_accel=-0.02,
      min_expected_accel=-0.50,
      stop_accel=-2.0,
      dt=0.01,
    )

  above_gate = controller.update(
    output_accel=-0.275,
    last_output_accel=-0.275,
    should_stop=True,
    v_ego=0.055,
    a_ego=-0.18,
    max_expected_accel=-0.02,
    min_expected_accel=-0.50,
    stop_accel=-2.0,
      dt=0.01,
    )
  assert controller.rebound_arrest_counter == 0


def test_stopping_controller_rebound_arrest_arms_with_release_lock_disturbance_fallback():
  armed = StoppingController()
  armed.release_lock_counter = 6
  armed_result = armed.update(
    output_accel=-0.36,
    last_output_accel=-0.36,
    should_stop=True,
    v_ego=0.043,
    a_ego=0.03,
    max_expected_accel=-0.02,
    min_expected_accel=-0.50,
    stop_accel=-2.0,
    dt=0.01,
  )

  assert armed_result.release_lock_active
  assert armed.rebound_arrest_counter > 0


def test_stopping_controller_low_rollout_soft_landing_cap_triggers():
  controller = StoppingController()
  last_output = -1.20
  debug: dict[str, object] = {}
  result = controller.update(
    output_accel=last_output,
    last_output_accel=last_output,
    should_stop=True,
    v_ego=0.10,
    a_ego=-0.30,
    max_expected_accel=-0.10,
    min_expected_accel=-0.50,
    stop_accel=-2.0,
    dt=0.01,
    debug=debug,
  )
  triggers = debug.get("triggers", ())
  assert "low_rollout_soft_landing_cap" in triggers
  assert result.output_accel > last_output


def test_stopping_controller_low_rollout_soft_landing_cap_waits_when_stop_target_still_ahead():
  controller = StoppingController()
  controller.phase = StoppingPhase.HOLD
  controller.low_speed_rollout_m = 0.9
  debug: dict[str, object] = {}
  result = controller.update(
    output_accel=-0.335,
    last_output_accel=-0.335,
    should_stop=True,
    v_ego=0.189,
    a_ego=-0.328,
    max_expected_accel=-0.11,
    min_expected_accel=-0.52,
    stop_accel=-2.0,
    dt=0.01,
    distance_to_stop_target_m=0.934,
    debug=debug,
  )

  triggers = debug.get("triggers", ())
  assert "low_rollout_soft_landing_cap" not in triggers
  assert result.output_accel < -0.325


def test_stopping_controller_low_rollout_soft_landing_cap_waits_when_creep_rebound_guard_is_active():
  controller = StoppingController()
  controller.phase = StoppingPhase.NEAR_HOLD
  controller.low_speed_rollout_m = 0.72
  controller.release_lock_counter = 6
  debug: dict[str, object] = {}
  result = controller.update(
    output_accel=-0.463,
    last_output_accel=-0.463,
    should_stop=True,
    v_ego=0.190,
    a_ego=-0.356,
    max_expected_accel=-0.106,
    min_expected_accel=-0.48,
    stop_accel=-2.0,
    dt=0.01,
    debug=debug,
  )

  triggers = debug.get("triggers", ())
  assert "creep_rebound_guard" in triggers
  assert "low_rollout_soft_landing_cap" not in triggers
  assert result.output_accel < -0.43


def test_stopping_controller_low_rollout_soft_landing_release_step_not_too_aggressive():
  controller = StoppingController()
  last_output = -1.20
  debug: dict[str, object] = {}
  result = controller.update(
    output_accel=last_output,
    last_output_accel=last_output,
    should_stop=True,
    v_ego=0.10,
    a_ego=-0.30,
    max_expected_accel=-0.10,
    min_expected_accel=-0.50,
    stop_accel=-2.0,
    dt=0.01,
    debug=debug,
  )
  triggers = debug.get("triggers", ())
  assert "low_rollout_soft_landing_cap" in triggers
  assert (result.output_accel - last_output) <= 0.020


def test_stopping_controller_clean_settle_profile_waits_when_rebound_cap_is_active():
  controller = StoppingController()
  controller.phase = StoppingPhase.HOLD
  controller.low_speed_rollout_m = 0.58
  debug: dict[str, object] = {}
  result = controller.update(
    output_accel=-0.309,
    last_output_accel=-0.309,
    should_stop=True,
    v_ego=0.063,
    a_ego=-0.162,
    max_expected_accel=-0.035,
    min_expected_accel=-0.46,
    stop_accel=-2.0,
    dt=0.01,
    debug=debug,
  )

  triggers = debug.get("triggers", ())
  assert "low_speed_rebound_cap_active" in triggers
  assert "clean_settle_profile" not in triggers
  assert result.output_accel < -0.30


def test_stopping_controller_tail_profile_terminal_soften_unwinds_final_low_speed_frames():
  controller = StoppingController()
  controller.low_speed_rollout_m = 1.0059848882474585
  controller.seed_command_history([
    -0.3928, -0.3583, -0.3510, -0.3421, -0.3342, -0.3285,
    -0.3221, -0.3158, -0.3101, -0.3049, -0.3003, -0.2962,
  ])

  debug: dict[str, object] = {}
  result = controller.update(
    output_accel=-0.2962,
    last_output_accel=-0.2962,
    should_stop=True,
    v_ego=0.0792,
    a_ego=-0.0702,
    max_expected_accel=-0.02723076923076923,
    min_expected_accel=-0.27784615384615385,
    stop_accel=-2.0,
    dt=0.1,
    distance_to_stop_target_m=0.0,
    debug=debug,
  )

  triggers = debug.get("triggers", ())
  assert "tail_profile_terminal_soften" in triggers
  assert result.output_accel > -0.295


def test_stopping_controller_tail_profile_terminal_soften_does_not_trigger_once_rollout_is_high():
  controller = StoppingController()
  controller.low_speed_rollout_m = 1.18
  controller.seed_command_history([
    -0.3928, -0.3583, -0.3510, -0.3421, -0.3342, -0.3285,
    -0.3221, -0.3158, -0.3101, -0.3049, -0.3003, -0.2962,
  ])

  debug: dict[str, object] = {}
  result = controller.update(
    output_accel=-0.2962,
    last_output_accel=-0.2962,
    should_stop=True,
    v_ego=0.0792,
    a_ego=-0.0702,
    max_expected_accel=-0.02723076923076923,
    min_expected_accel=-0.27784615384615385,
    stop_accel=-2.0,
    dt=0.1,
    distance_to_stop_target_m=0.0,
    debug=debug,
  )

  triggers = debug.get("triggers", ())
  assert "tail_profile_terminal_soften" not in triggers
  assert result.output_accel < -0.31


def test_stopping_controller_end_stop_cap_release_step_not_too_aggressive():
  controller = StoppingController()
  last_output = -1.20
  debug: dict[str, object] = {}
  result = controller.update(
    output_accel=last_output,
    last_output_accel=last_output,
    should_stop=True,
    v_ego=0.35,
    a_ego=-0.30,
    max_expected_accel=-0.10,
    min_expected_accel=-0.50,
    stop_accel=-2.0,
    dt=0.01,
    debug=debug,
  )
  triggers = debug.get("triggers", ())
  assert "end_stop_cap_active" in triggers
  assert (result.output_accel - last_output) <= 0.013


def test_stopping_controller_moderate_rollout_rebound_soften_limits_first_arrest_step():
  controller = StoppingController()
  controller.phase = StoppingPhase.HOLD
  controller.low_speed_rollout_m = 0.56
  debug: dict[str, object] = {}
  result = controller.update(
    output_accel=-0.468,
    last_output_accel=-0.468,
    should_stop=True,
    v_ego=0.042,
    a_ego=-0.050,
    max_expected_accel=-0.03,
    min_expected_accel=-0.50,
    stop_accel=-2.0,
    dt=0.01,
    debug=debug,
  )

  triggers = debug.get("triggers", ())
  assert "rebound_arrest_active" in triggers
  assert "moderate_rollout_rebound_soften" in triggers
  assert result.output_accel > -0.58


def test_stopping_controller_moderate_rollout_rebound_soften_stops_once_brake_is_already_deep():
  controller = StoppingController()
  controller.phase = StoppingPhase.HOLD
  controller.low_speed_rollout_m = 0.56
  debug: dict[str, object] = {}
  result = controller.update(
    output_accel=-0.715,
    last_output_accel=-0.62,
    should_stop=True,
    v_ego=0.039,
    a_ego=-0.031,
    max_expected_accel=-0.026,
    min_expected_accel=-0.50,
    stop_accel=-2.0,
    dt=0.01,
    debug=debug,
  )

  triggers = debug.get("triggers", ())
  assert "rebound_arrest_active" in triggers
  assert "moderate_rollout_rebound_soften" not in triggers
  assert result.output_accel < -0.64


def test_stopping_controller_rollout_tightening_strengthens_brake_when_low_speed_rollout_grows():
  controller = StoppingController()
  output = -0.09
  for _ in range(220):
    result = controller.update(
      output_accel=-0.09,
      last_output_accel=output,
      should_stop=True,
      v_ego=0.70,
      a_ego=-0.08,
      max_expected_accel=-0.12,
      min_expected_accel=-0.50,
      stop_accel=-2.0,
      dt=0.01,
    )
    output = result.output_accel

  baseline_controller = StoppingController()
  baseline = baseline_controller.update(
    output_accel=-0.09,
    last_output_accel=-0.09,
    should_stop=True,
    v_ego=0.70,
    a_ego=-0.08,
    max_expected_accel=-0.12,
    min_expected_accel=-0.50,
    stop_accel=-2.0,
    dt=0.01,
  )

  assert controller.low_speed_rollout_m > 1.0
  assert output < baseline.output_accel - 0.04


def test_stopping_controller_rollout_oscillation_damping_holds_firmer_brake_when_rollout_is_high():
  high_rollout = StoppingController()
  high_rollout.low_speed_rollout_m = 2.4
  high_rollout.release_lock_counter = 8
  high_rollout.phase = StoppingPhase.NEAR_HOLD
  high_result = high_rollout.update(
    output_accel=0.05,
    last_output_accel=-0.60,
    should_stop=True,
    v_ego=0.65,
    a_ego=0.02,
    max_expected_accel=-0.12,
    min_expected_accel=-0.50,
    stop_accel=-2.0,
    dt=0.01,
  )

  low_rollout = StoppingController()
  low_rollout.low_speed_rollout_m = 0.4
  low_rollout.release_lock_counter = 8
  low_rollout.phase = StoppingPhase.NEAR_HOLD
  low_result = low_rollout.update(
    output_accel=0.05,
    last_output_accel=-0.60,
    should_stop=True,
    v_ego=0.65,
    a_ego=0.02,
    max_expected_accel=-0.12,
    min_expected_accel=-0.50,
    stop_accel=-2.0,
    dt=0.01,
  )

  assert high_result.output_accel < low_result.output_accel - 0.004


def test_stopping_controller_severe_rebound_guard_adds_brake_when_rollout_is_large_and_decel_collapses():
  high_rollout = StoppingController()
  high_rollout.low_speed_rollout_m = 0.90
  high_result = high_rollout.update(
    output_accel=-0.30,
    last_output_accel=-0.30,
    should_stop=True,
    v_ego=0.42,
    a_ego=0.04,
    max_expected_accel=-0.12,
    min_expected_accel=-0.50,
    stop_accel=-2.0,
    dt=0.01,
  )

  low_rollout = StoppingController()
  low_rollout.low_speed_rollout_m = 0.20
  low_result = low_rollout.update(
    output_accel=-0.30,
    last_output_accel=-0.30,
    should_stop=True,
    v_ego=0.42,
    a_ego=0.04,
    max_expected_accel=-0.12,
    min_expected_accel=-0.50,
    stop_accel=-2.0,
    dt=0.01,
  )

  assert high_rollout.low_speed_rollout_m > 0.80
  assert high_result.output_accel < low_result.output_accel - 0.010


def test_stopping_controller_delay_release_guard_limits_release_relief():
  guarded = StoppingController()
  for _ in range(8):
    _ = guarded.update(
      output_accel=-0.35,
      last_output_accel=-0.35,
      should_stop=True,
      v_ego=0.95,
      a_ego=-0.20,
      max_expected_accel=-0.10,
      min_expected_accel=-0.45,
      stop_accel=-2.0,
      dt=0.01,
    )

  guarded_result = guarded.update(
    output_accel=0.10,
    last_output_accel=-0.20,
    should_stop=True,
    v_ego=0.95,
    a_ego=-0.18,
    max_expected_accel=-0.10,
    min_expected_accel=-0.45,
    stop_accel=-2.0,
    dt=0.01,
  )

  baseline = StoppingController()
  baseline_result = baseline.update(
    output_accel=0.10,
    last_output_accel=-0.20,
    should_stop=True,
    v_ego=0.95,
    a_ego=-0.18,
    max_expected_accel=-0.10,
    min_expected_accel=-0.45,
    stop_accel=-2.0,
    dt=0.01,
  )

  assert guarded_result.output_accel < baseline_result.output_accel - 0.001


def test_stopping_controller_over_brake_damping_relieves_harsh_decel():
  controller = StoppingController()
  harsh_result = controller.update(
    output_accel=-0.26,
    last_output_accel=-0.26,
    should_stop=True,
    v_ego=0.22,
    a_ego=-1.05,
    max_expected_accel=-0.12,
    min_expected_accel=-0.45,
    stop_accel=-2.0,
    dt=0.01,
  )

  baseline_controller = StoppingController()
  baseline_result = baseline_controller.update(
    output_accel=-0.26,
    last_output_accel=-0.26,
    should_stop=True,
    v_ego=0.22,
    a_ego=-0.35,
    max_expected_accel=-0.12,
    min_expected_accel=-0.45,
    stop_accel=-2.0,
    dt=0.01,
  )

  assert harsh_result.output_accel > baseline_result.output_accel + 0.0002


def test_stopping_controller_ineffective_brake_guard_prevents_deep_windup_near_hold():
  controller = StoppingController()
  output = -0.80

  # Simulate a stop where decel is strong initially but becomes ineffective near hold (e.g. drivetrain/clutch push).
  # Without a guard, the controller can ratchet toward stop_accel aggressively.
  for idx in range(200):
    v_ego = max(0.06, 0.70 - (0.003 * idx))
    a_ego = -0.75 if v_ego > 0.32 else -0.20
    result = controller.update(
      output_accel=output,
      last_output_accel=output,
      should_stop=True,
      v_ego=v_ego,
      a_ego=a_ego,
      max_expected_accel=-0.10,
      min_expected_accel=-0.50,
      stop_accel=-2.0,
      dt=0.10,
    )
    output = result.output_accel

  assert output > -1.50


def test_stopping_controller_soft_landing_releases_in_hold_when_decel_is_stable():
  controller = StoppingController()
  output = -0.22

  for idx in range(50):
    v_ego = max(0.0, 0.20 * (1.0 - (idx / 49.0)))
    result = controller.update(
      output_accel=output,
      last_output_accel=output,
      should_stop=True,
      v_ego=v_ego,
      a_ego=-0.20,
      max_expected_accel=-0.10,
      min_expected_accel=-0.50,
      stop_accel=-2.0,
      dt=0.01,
    )
    output = result.output_accel

  for _ in range(150):
    result = controller.update(
      output_accel=output,
      last_output_accel=output,
      should_stop=True,
      v_ego=0.0,
      a_ego=0.0,
      max_expected_accel=-0.10,
      min_expected_accel=-0.50,
      stop_accel=-2.0,
      dt=0.01,
    )
    output = result.output_accel

  assert controller.phase == StoppingPhase.HOLD
  assert output > -0.18


def _build_20260228_model_for_holdout_replay() -> FittedStoppingModel:
  return FittedStoppingModel(
    delay_frames=0,
    coefficients={
      "intercept": -0.253992889313543,
      "a_ego_prev": 0.8707248413581602,
      "accel_cmd_delayed": 0.2638422343345787,
      "v_ego": 0.29304613067352236,
      "relief": 0.19640459646345515,
      "low_speed": 0.3326724195052019,
      "cmd_x_low_speed": -0.05456194852856041,
    },
    rmse=0.03986682356619745,
    mae=0.02857283946972372,
    r2=0.9680007360030826,
    sample_count=855,
    dt_s=0.1000026359979529,
    relief_cmd_threshold=-0.25,
    low_speed_ref=1.2,
  )


def _simulate_20260228_holdout_seed(samples: list[FakeSample], start_idx: int, hold_idx: int) -> dict[str, float | list[float] | None]:
  model = _build_20260228_model_for_holdout_replay()
  return simulate_event_with_controller(
    samples=samples,
    start_idx=start_idx,
    hold_idx=hold_idx,
    model=model,
    stopping_speed_breakpoint=0.4,
    stop_accel=-2.0,
    controller_should_stop_source="recorded",
  )


def _build_regression_seed_samples_71c_event14() -> list[FakeSample]:
  # Seeded from route 0000071c--fb4cca0034 event 14 (controller replay: stopping_state -> last_stopping_state).
  return [
    FakeSample(t=0.000000000, v_ego=0.145192429, a_ego=-0.480262429, accel_cmd=0.000000000, should_stop=False),
    FakeSample(t=0.100135481, v_ego=0.099171445, a_ego=-0.455735594, accel_cmd=0.000000000, should_stop=False),
    FakeSample(t=0.200578667, v_ego=0.073696688, a_ego=-0.317390710, accel_cmd=0.000000000, should_stop=False),
    FakeSample(t=0.299972905, v_ego=0.062364805, a_ego=-0.184303403, accel_cmd=0.000000000, should_stop=False),
    FakeSample(t=0.399909898, v_ego=0.055598494, a_ego=-0.110088125, accel_cmd=0.000000000, should_stop=False),
    FakeSample(t=0.500773549, v_ego=0.048564520, a_ego=-0.082931839, accel_cmd=0.000000000, should_stop=False),
    FakeSample(t=0.600475544, v_ego=0.045693733, a_ego=-0.049658697, accel_cmd=-0.097595550, should_stop=False),
  ]


def _build_regression_seed_samples_71c_event15() -> list[FakeSample]:
  # Seeded from route 0000071c--fb4cca0034 event 15 (controller replay: stopping_state -> last_stopping_state).
  return [
    FakeSample(t=0.000000000, v_ego=0.143859372, a_ego=-0.567591488, accel_cmd=0.000000000, should_stop=False),
    FakeSample(t=0.100081783, v_ego=0.115193650, a_ego=-0.371072471, accel_cmd=0.000000000, should_stop=False),
    FakeSample(t=0.200227941, v_ego=0.093056388, a_ego=-0.277767003, accel_cmd=0.000000000, should_stop=False),
    FakeSample(t=0.299908790, v_ego=0.073021226, a_ego=-0.226987854, accel_cmd=0.000000000, should_stop=False),
    FakeSample(t=0.398311682, v_ego=0.060816433, a_ego=-0.156830788, accel_cmd=0.000000000, should_stop=False),
    FakeSample(t=0.500241574, v_ego=0.053067580, a_ego=-0.101316400, accel_cmd=-0.019456718, should_stop=False),
    FakeSample(t=0.599118160, v_ego=0.048325647, a_ego=-0.062349379, accel_cmd=-0.100000001, should_stop=False),
    FakeSample(t=0.699042392, v_ego=0.042290162, a_ego=-0.063477121, accel_cmd=-0.100000001, should_stop=False),
  ]


def _build_regression_seed_samples_71c_event19() -> list[FakeSample]:
  # Seeded from route 0000071c--fb4cca0034 event 19 (controller replay: stopping_state -> last_stopping_state).
  return [
    FakeSample(t=0.000000000, v_ego=1.950278521, a_ego=-1.707341313, accel_cmd=-1.298798203, should_stop=False),
    FakeSample(t=0.100469540, v_ego=1.763089061, a_ego=-1.810387373, accel_cmd=-1.136835337, should_stop=False),
    FakeSample(t=0.200701791, v_ego=1.596643090, a_ego=-1.707637072, accel_cmd=-1.006954432, should_stop=False),
    FakeSample(t=0.299749365, v_ego=1.460950613, a_ego=-1.457232594, accel_cmd=-0.899742305, should_stop=False),
    FakeSample(t=0.399939324, v_ego=1.357057691, a_ego=-1.181920409, accel_cmd=-0.808082163, should_stop=False),
    FakeSample(t=0.501044327, v_ego=1.277657866, a_ego=-0.936747730, accel_cmd=-0.767186880, should_stop=True),
    FakeSample(t=0.599420762, v_ego=1.199124217, a_ego=-0.848762989, accel_cmd=-0.767186880, should_stop=True),
    FakeSample(t=0.700146862, v_ego=1.123829246, a_ego=-0.785991609, accel_cmd=-0.767186880, should_stop=True),
    FakeSample(t=0.801071762, v_ego=1.050020218, a_ego=-0.760630608, accel_cmd=-0.767186880, should_stop=True),
    FakeSample(t=0.900498291, v_ego=0.966204226, a_ego=-0.816253662, accel_cmd=-0.767186880, should_stop=True),
    FakeSample(t=1.000092214, v_ego=0.888874352, a_ego=-0.787938714, accel_cmd=-0.767186880, should_stop=True),
    FakeSample(t=1.101141645, v_ego=0.810444832, a_ego=-0.791749120, accel_cmd=-0.735343456, should_stop=True),
    FakeSample(t=1.200659319, v_ego=0.733558297, a_ego=-0.777034998, accel_cmd=-0.579783440, should_stop=True),
    FakeSample(t=1.299893766, v_ego=0.655196548, a_ego=-0.779718101, accel_cmd=-0.535524547, should_stop=True),
    FakeSample(t=1.401156788, v_ego=0.594471335, a_ego=-0.667412162, accel_cmd=-0.520203590, should_stop=True),
    FakeSample(t=1.500079207, v_ego=0.536299586, a_ego=-0.613014638, accel_cmd=-0.520203590, should_stop=True),
    FakeSample(t=1.600731558, v_ego=0.483337045, a_ego=-0.563229620, accel_cmd=-0.520203590, should_stop=True),
    FakeSample(t=1.701268181, v_ego=0.427170098, a_ego=-0.563264191, accel_cmd=-0.520203590, should_stop=True),
    FakeSample(t=1.799582741, v_ego=0.372866124, a_ego=-0.545366764, accel_cmd=-0.519969583, should_stop=True),
    FakeSample(t=1.899274267, v_ego=0.319620907, a_ego=-0.536725461, accel_cmd=-0.519061685, should_stop=True),
    FakeSample(t=2.001531291, v_ego=0.271066517, a_ego=-0.518130779, accel_cmd=-0.517975986, should_stop=True),
    FakeSample(t=2.100622146, v_ego=0.221925080, a_ego=-0.521571219, accel_cmd=-0.516499519, should_stop=True),
    FakeSample(t=2.198704208, v_ego=0.184645012, a_ego=-0.427489012, accel_cmd=-0.407109290, should_stop=True),
    FakeSample(t=2.300876597, v_ego=0.134508610, a_ego=-0.464466721, accel_cmd=-0.287008464, should_stop=True),
    FakeSample(t=2.400454478, v_ego=0.113339633, a_ego=-0.293460011, accel_cmd=-0.263307571, should_stop=True),
    FakeSample(t=2.499475282, v_ego=0.095889792, a_ego=-0.220270500, accel_cmd=-0.258771390, should_stop=True),
    FakeSample(t=2.600796741, v_ego=0.078135438, a_ego=-0.187114209, accel_cmd=-0.254999995, should_stop=True),
    FakeSample(t=2.700077855, v_ego=0.072472245, a_ego=-0.102088101, accel_cmd=-0.254999995, should_stop=True),
    FakeSample(t=2.799897661, v_ego=0.070264116, a_ego=-0.053798035, accel_cmd=-0.314819843, should_stop=True),
    FakeSample(t=2.901767553, v_ego=0.060129650, a_ego=-0.085584521, accel_cmd=-0.319867402, should_stop=True),
  ]


def _build_regression_seed_samples_721_event4() -> list[FakeSample]:
  # Seeded from route 00000721--2b37d8d4a9 event 4 (controller replay: stopping_state -> last_stopping_state).
  return [
    FakeSample(t=0.000000000, v_ego=1.235621214, a_ego=-0.530675113, accel_cmd=-0.555719316, should_stop=False),
    FakeSample(t=0.099714845, v_ego=1.183831930, a_ego=-0.524957478, accel_cmd=-0.542232335, should_stop=False),
    FakeSample(t=0.199100215, v_ego=1.135973692, a_ego=-0.493288368, accel_cmd=-0.529286087, should_stop=False),
    FakeSample(t=0.300399096, v_ego=1.089928985, a_ego=-0.477086425, accel_cmd=-0.522819698, should_stop=False),
    FakeSample(t=0.398843908, v_ego=1.042990327, a_ego=-0.473472536, accel_cmd=-0.522869527, should_stop=False),
    FakeSample(t=0.499206242, v_ego=0.997534871, a_ego=-0.460935026, accel_cmd=-0.521466672, should_stop=True),
    FakeSample(t=0.599363840, v_ego=0.951765239, a_ego=-0.460146070, accel_cmd=-0.521466672, should_stop=True),
    FakeSample(t=0.700086324, v_ego=0.903934777, a_ego=-0.472543865, accel_cmd=-0.521466672, should_stop=True),
    FakeSample(t=0.798791965, v_ego=0.858401299, a_ego=-0.461935610, accel_cmd=-0.521466672, should_stop=True),
    FakeSample(t=0.900139492, v_ego=0.812758863, a_ego=-0.463700324, accel_cmd=-0.521466672, should_stop=True),
    FakeSample(t=0.999079765, v_ego=0.764690399, a_ego=-0.473284155, accel_cmd=-0.521466672, should_stop=True),
    FakeSample(t=1.099277206, v_ego=0.714496791, a_ego=-0.489017397, accel_cmd=-0.521466672, should_stop=True),
    FakeSample(t=1.199206057, v_ego=0.664312601, a_ego=-0.506144524, accel_cmd=-0.521466672, should_stop=True),
    FakeSample(t=1.299196938, v_ego=0.613953650, a_ego=-0.502093792, accel_cmd=-0.521466672, should_stop=True),
    FakeSample(t=1.398339031, v_ego=0.556666493, a_ego=-0.550569415, accel_cmd=-0.521466672, should_stop=True),
    FakeSample(t=1.499975251, v_ego=0.501445174, a_ego=-0.547923744, accel_cmd=-0.521466672, should_stop=True),
    FakeSample(t=1.598717975, v_ego=0.451876104, a_ego=-0.516575396, accel_cmd=-0.521466672, should_stop=True),
    FakeSample(t=1.698654639, v_ego=0.398472369, a_ego=-0.534453332, accel_cmd=-0.521466672, should_stop=True),
    FakeSample(t=1.800043102, v_ego=0.346907854, a_ego=-0.525960922, accel_cmd=-0.521383762, should_stop=True),
    FakeSample(t=1.899943725, v_ego=0.299348772, a_ego=-0.498036742, accel_cmd=-0.520783603, should_stop=True),
    FakeSample(t=1.998564315, v_ego=0.250725865, a_ego=-0.492574543, accel_cmd=-0.519876361, should_stop=True),
    FakeSample(t=2.100311731, v_ego=0.206416100, a_ego=-0.466788203, accel_cmd=-0.518469870, should_stop=True),
    FakeSample(t=2.199803298, v_ego=0.169414371, a_ego=-0.403295517, accel_cmd=-0.455607384, should_stop=True),
    FakeSample(t=2.299185283, v_ego=0.125532642, a_ego=-0.423131227, accel_cmd=-0.306111902, should_stop=True),
    FakeSample(t=2.400454998, v_ego=0.101517759, a_ego=-0.302089244, accel_cmd=-0.266869694, should_stop=True),
    FakeSample(t=2.499091942, v_ego=0.079871409, a_ego=-0.247792944, accel_cmd=-0.254999995, should_stop=True),
    FakeSample(t=2.599364017, v_ego=0.065384440, a_ego=-0.177497581, accel_cmd=-0.254999995, should_stop=True),
    FakeSample(t=2.700431861, v_ego=0.059103657, a_ego=-0.102388658, accel_cmd=-0.254999995, should_stop=True),
    FakeSample(t=2.799008390, v_ego=0.055978604, a_ego=-0.054795101, accel_cmd=-0.254999995, should_stop=True),
    FakeSample(t=2.898765056, v_ego=0.048441369, a_ego=-0.068337142, accel_cmd=-0.330511928, should_stop=True),
    FakeSample(t=3.000229716, v_ego=0.042945202, a_ego=-0.060058888, accel_cmd=-0.335996509, should_stop=True),
  ]


def test_stopping_controller_regression_seed_71c_event14_reduces_end_stop_jerk() -> None:
  result = _simulate_20260228_holdout_seed(_build_regression_seed_samples_71c_event14(), start_idx=5, hold_idx=6)
  assert result["pred_end_stop_jerk_mps3"] is not None
  assert result["pred_end_stop_jerk_mps3"] <= 0.95


def test_stopping_controller_regression_seed_71c_event15_reduces_end_stop_jerk_and_step() -> None:
  result = _simulate_20260228_holdout_seed(_build_regression_seed_samples_71c_event15(), start_idx=5, hold_idx=7)
  assert result["pred_end_stop_jerk_mps3"] is not None
  assert result["pred_end_stop_jerk_mps3"] <= 0.95
  assert result["pred_end_stop_accel_step_mps2"] is not None
  assert result["pred_end_stop_accel_step_mps2"] <= 0.08


def test_stopping_controller_regression_seed_71c_event19_reduces_end_stop_accel_step() -> None:
  result = _simulate_20260228_holdout_seed(_build_regression_seed_samples_71c_event19(), start_idx=5, hold_idx=29)
  assert result["pred_end_stop_accel_step_mps2"] is not None
  assert result["pred_end_stop_accel_step_mps2"] <= 0.08


def test_stopping_controller_regression_seed_721_event4_reduces_end_stop_accel_step() -> None:
  result = _simulate_20260228_holdout_seed(_build_regression_seed_samples_721_event4(), start_idx=5, hold_idx=30)
  assert result["pred_end_stop_accel_step_mps2"] is not None
  assert result["pred_end_stop_accel_step_mps2"] <= 0.08


def _build_20260302_model_for_holdout_replay() -> FittedStoppingModel:
  return FittedStoppingModel(
    delay_frames=0,
    coefficients={
      "intercept": -0.20482419699316673,
      "a_ego_prev": 0.8626149044685716,
      "accel_cmd_delayed": 0.23804013346480102,
      "v_ego": 0.24195134871355753,
      "relief": 0.15926397088416644,
      "low_speed": 0.3007732335278943,
      "cmd_x_low_speed": 0.02268693223791858,
    },
    rmse=0.03974499391846442,
    mae=0.028530463993002183,
    r2=0.9685625039187713,
    sample_count=1003,
    dt_s=0.1000020400019821,
    relief_cmd_threshold=-0.25,
    low_speed_ref=1.2,
  )


def _simulate_20260302_holdout_seed(samples: list[FakeSample], start_idx: int, hold_idx: int) -> dict[str, float | list[float] | None]:
  model = _build_20260302_model_for_holdout_replay()
  return simulate_event_with_controller(
    samples=samples,
    start_idx=start_idx,
    hold_idx=hold_idx,
    model=model,
    stopping_speed_breakpoint=0.4,
    stop_accel=-2.0,
    controller_should_stop_source="recorded",
  )


def _build_regression_seed_samples_71c_event2() -> list[FakeSample]:
  # Seeded from route 0000071c--fb4cca0034 event 2 (controller replay: stopping_state -> last_stopping_state).
  return [
    FakeSample(t=0.000000000, v_ego=0.784829557, a_ego=-0.300528258, accel_cmd=-0.419662148, should_stop=False),
    FakeSample(t=0.099421778, v_ego=0.755568802, a_ego=-0.293384612, accel_cmd=-0.407163411, should_stop=False),
    FakeSample(t=0.199871410, v_ego=0.740811944, a_ego=-0.193549901, accel_cmd=-0.394391775, should_stop=False),
    FakeSample(t=0.299521726, v_ego=0.731671453, a_ego=-0.132301435, accel_cmd=-0.381100953, should_stop=False),
    FakeSample(t=0.399597664, v_ego=0.723472178, a_ego=-0.097458981, accel_cmd=-0.367715061, should_stop=False),
    FakeSample(t=0.499750528, v_ego=0.720975935, a_ego=-0.048895724, accel_cmd=-0.369047791, should_stop=False),
    FakeSample(t=0.600071203, v_ego=0.715618789, a_ego=-0.057813413, accel_cmd=-0.370818585, should_stop=True),
    FakeSample(t=0.700044381, v_ego=0.709748328, a_ego=-0.055558875, accel_cmd=-0.372517794, should_stop=True),
    FakeSample(t=0.800595575, v_ego=0.705446243, a_ego=-0.052377995, accel_cmd=-0.374266744, should_stop=True),
    FakeSample(t=0.899975061, v_ego=0.694480717, a_ego=-0.093083411, accel_cmd=-0.375968009, should_stop=True),
    FakeSample(t=1.000146050, v_ego=0.681506038, a_ego=-0.115166686, accel_cmd=-0.377381086, should_stop=True),
    FakeSample(t=1.100773336, v_ego=0.672038257, a_ego=-0.101815224, accel_cmd=-0.427187353, should_stop=True),
    FakeSample(t=1.200167770, v_ego=0.657218933, a_ego=-0.134072050, accel_cmd=-0.511181116, should_stop=True),
    FakeSample(t=1.300701099, v_ego=0.634683073, a_ego=-0.199412122, accel_cmd=-0.594658911, should_stop=True),
    FakeSample(t=1.400168814, v_ego=0.605225265, a_ego=-0.262456536, accel_cmd=-0.677341163, should_stop=True),
    FakeSample(t=1.501204742, v_ego=0.565025449, a_ego=-0.352642268, accel_cmd=-0.758976936, should_stop=True),
    FakeSample(t=1.599829705, v_ego=0.520543337, a_ego=-0.415924728, accel_cmd=-0.783206701, should_stop=True),
    FakeSample(t=1.699828872, v_ego=0.463843226, a_ego=-0.513532281, accel_cmd=-0.783206701, should_stop=True),
    FakeSample(t=1.800914435, v_ego=0.400093853, a_ego=-0.588400960, accel_cmd=-0.769700110, should_stop=True),
    FakeSample(t=1.900260589, v_ego=0.331509173, a_ego=-0.652881086, accel_cmd=-0.684400201, should_stop=True),
    FakeSample(t=1.999404921, v_ego=0.266150385, a_ego=-0.652163923, accel_cmd=-0.586427510, should_stop=True),
    FakeSample(t=2.100092988, v_ego=0.202109933, a_ego=-0.651271820, accel_cmd=-0.427551717, should_stop=True),
    FakeSample(t=2.199688566, v_ego=0.162661925, a_ego=-0.477582484, accel_cmd=-0.279214501, should_stop=True),
    FakeSample(t=2.300709859, v_ego=0.122712113, a_ego=-0.430180639, accel_cmd=-0.249695733, should_stop=True),
    FakeSample(t=2.400323562, v_ego=0.086582892, a_ego=-0.380344450, accel_cmd=-0.254999995, should_stop=True),
    FakeSample(t=2.499936587, v_ego=0.069540530, a_ego=-0.242060229, accel_cmd=-0.254999995, should_stop=True),
    FakeSample(t=2.599404614, v_ego=0.059791360, a_ego=-0.149460718, accel_cmd=-0.254999995, should_stop=True),
    FakeSample(t=2.700223461, v_ego=0.051809248, a_ego=-0.105145313, accel_cmd=-0.254999995, should_stop=True),
    FakeSample(t=2.799296857, v_ego=0.047344718, a_ego=-0.062766537, accel_cmd=-0.254999995, should_stop=True),
    FakeSample(t=2.900214349, v_ego=0.043110732, a_ego=-0.049691800, accel_cmd=-0.368265837, should_stop=True),
    FakeSample(t=3.000358306, v_ego=0.041525561, a_ego=-0.025518790, accel_cmd=-0.672250092, should_stop=True),
  ]


def test_stopping_controller_regression_seed_20260302_event2_targets_accel_step() -> None:
  result = _simulate_20260302_holdout_seed(_build_regression_seed_samples_71c_event2(), start_idx=5, hold_idx=30)
  assert result["pred_end_stop_accel_step_mps2"] is not None
  assert result["pred_end_stop_accel_step_mps2"] <= 0.10


def test_stopping_controller_regression_seed_20260302_event14_targets_end_stop_jerk() -> None:
  result = _simulate_20260302_holdout_seed(_build_regression_seed_samples_71c_event14(), start_idx=5, hold_idx=6)
  assert result["pred_end_stop_jerk_mps3"] is not None
  assert result["pred_end_stop_jerk_mps3"] <= 1.00


def test_stopping_controller_regression_seed_20260302_event15_targets_jerk_and_step() -> None:
  result = _simulate_20260302_holdout_seed(_build_regression_seed_samples_71c_event15(), start_idx=5, hold_idx=7)
  assert result["pred_end_stop_jerk_mps3"] is not None
  assert result["pred_end_stop_jerk_mps3"] <= 1.00
  assert result["pred_end_stop_accel_step_mps2"] is not None
  assert result["pred_end_stop_accel_step_mps2"] <= 0.13


def test_stopping_controller_regression_seed_20260302_event19_targets_accel_step() -> None:
  result = _simulate_20260302_holdout_seed(_build_regression_seed_samples_71c_event19(), start_idx=5, hold_idx=29)
  assert result["pred_end_stop_accel_step_mps2"] is not None
  assert result["pred_end_stop_accel_step_mps2"] <= 0.08
