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
  distance_to_stop_target_m: float | None = None
  raw_should_stop: bool | None = None


def _run_direct_controller_seed(samples: list[FakeSample], use_logged_base_accel: bool = False) -> tuple[list[float], list[tuple[str, ...]]]:
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
    if use_logged_base_accel and sample.accel_cmd is not None:
      output_seed = float(sample.accel_cmd)
    else:
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
      distance_to_stop_target_m=sample.distance_to_stop_target_m,
      raw_should_stop=sample.raw_should_stop,
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


def _build_late_no_target_stop_entry_seed_samples_1c_event14() -> list[FakeSample]:
  return [
    FakeSample(t=2474.766000000, v_ego=0.795000000, a_ego=-0.063000000, accel_cmd=-0.355000000, should_stop=False, distance_to_stop_target_m=-1.0),
    FakeSample(t=2474.867000000, v_ego=0.790000000, a_ego=-0.053000000, accel_cmd=-0.355000000, should_stop=False, distance_to_stop_target_m=-1.0),
    FakeSample(t=2475.066000000, v_ego=0.785000000, a_ego=-0.023000000, accel_cmd=-0.355000000, should_stop=False, distance_to_stop_target_m=-1.0),
    FakeSample(t=2475.071000000, v_ego=0.785000000, a_ego=-0.023000000, accel_cmd=-0.337000000, should_stop=True, distance_to_stop_target_m=-1.0),
    FakeSample(t=2475.267000000, v_ego=0.784000000, a_ego=0.001000000, accel_cmd=-0.337000000, should_stop=True, distance_to_stop_target_m=-1.0),
    FakeSample(t=2475.468000000, v_ego=0.789000000, a_ego=0.034000000, accel_cmd=-0.337000000, should_stop=True, distance_to_stop_target_m=-1.0),
    FakeSample(t=2475.668000000, v_ego=0.798000000, a_ego=0.048000000, accel_cmd=-0.337000000, should_stop=True, distance_to_stop_target_m=-1.0),
    FakeSample(t=2475.873000000, v_ego=0.803000000, a_ego=0.013000000, accel_cmd=-0.501000000, should_stop=True, distance_to_stop_target_m=-1.0),
  ]


def _build_explicit_target_early_entry_seed_samples_83_event1() -> list[FakeSample]:
  return [
    FakeSample(t=982.715269176, v_ego=0.979457378, a_ego=-0.090151131, accel_cmd=-0.280922532, should_stop=False, distance_to_stop_target_m=2.181832314),
    FakeSample(t=982.814262425, v_ego=0.968922853, a_ego=-0.101625338, accel_cmd=-0.411648303, should_stop=False, distance_to_stop_target_m=2.181832314),
    FakeSample(t=982.915010562, v_ego=0.953865588, a_ego=-0.132557675, accel_cmd=-0.474869609, should_stop=False, distance_to_stop_target_m=2.181832314),
    FakeSample(t=983.014859223, v_ego=0.936828732, a_ego=-0.156066343, accel_cmd=-0.464150131, should_stop=False, distance_to_stop_target_m=2.181832314),
    FakeSample(t=983.114556896, v_ego=0.917098820, a_ego=-0.183807507, accel_cmd=-0.497169822, should_stop=False, distance_to_stop_target_m=1.827315211),
    FakeSample(t=983.215088628, v_ego=0.901994467, a_ego=-0.160122305, accel_cmd=-0.477712005, should_stop=True, distance_to_stop_target_m=1.827315211, raw_should_stop=False),
    FakeSample(t=983.315668901, v_ego=0.891280055, a_ego=-0.123685643, accel_cmd=-0.478767782, should_stop=True, distance_to_stop_target_m=1.827315211, raw_should_stop=False),
    FakeSample(t=983.412789084, v_ego=0.889672697, a_ego=-0.050174020, accel_cmd=-0.480098248, should_stop=True, distance_to_stop_target_m=1.827315211, raw_should_stop=False),
    FakeSample(t=983.514566540, v_ego=0.893170953, a_ego=0.002965912, accel_cmd=-0.481911272, should_stop=True, distance_to_stop_target_m=1.827315211, raw_should_stop=False),
    FakeSample(t=983.614998637, v_ego=0.899387777, a_ego=0.042299360, accel_cmd=-0.484307975, should_stop=True, distance_to_stop_target_m=1.475774288, raw_should_stop=True),
    FakeSample(t=983.714420529, v_ego=0.900793791, a_ego=0.020489644, accel_cmd=-0.487037122, should_stop=True, distance_to_stop_target_m=1.475774288, raw_should_stop=True),
    FakeSample(t=983.815640852, v_ego=0.897494853, a_ego=-0.016901217, accel_cmd=-0.495391667, should_stop=True, distance_to_stop_target_m=1.475774288, raw_should_stop=True),
  ]


def _build_explicit_target_clean_entry_seed_samples_83_event9() -> list[FakeSample]:
  return [
    FakeSample(t=1128.621693810, v_ego=1.205629349, a_ego=-0.665356159, accel_cmd=-0.770703495, should_stop=False, distance_to_stop_target_m=1.798567295),
    FakeSample(t=1128.721541748, v_ego=1.136868834, a_ego=-0.683891177, accel_cmd=-0.751612782, should_stop=False, distance_to_stop_target_m=1.798567295),
    FakeSample(t=1128.820423023, v_ego=1.068964601, a_ego=-0.677500248, accel_cmd=-0.731114507, should_stop=False, distance_to_stop_target_m=1.798567295),
    FakeSample(t=1128.921516477, v_ego=1.002551556, a_ego=-0.670148551, accel_cmd=-0.708194494, should_stop=False, distance_to_stop_target_m=1.798567295),
    FakeSample(t=1129.020603324, v_ego=0.934979498, a_ego=-0.678382456, accel_cmd=-0.681038320, should_stop=False, distance_to_stop_target_m=1.798567295),
    FakeSample(t=1129.120202721, v_ego=0.877819598, a_ego=-0.605664670, accel_cmd=-0.726261318, should_stop=True, distance_to_stop_target_m=1.299057007),
    FakeSample(t=1129.220324148, v_ego=0.821927965, a_ego=-0.572953224, accel_cmd=-0.787693679, should_stop=True, distance_to_stop_target_m=1.299057007),
    FakeSample(t=1129.321573070, v_ego=0.771063924, a_ego=-0.534751534, accel_cmd=-0.787693679, should_stop=True, distance_to_stop_target_m=1.299057007),
    FakeSample(t=1129.420746844, v_ego=0.718050957, a_ego=-0.534096360, accel_cmd=-0.746994495, should_stop=True, distance_to_stop_target_m=1.299057007),
    FakeSample(t=1129.520451918, v_ego=0.661312997, a_ego=-0.550652564, accel_cmd=-0.613087952, should_stop=True, distance_to_stop_target_m=1.299057007),
  ]


def _build_explicit_target_tail_settle_seed_samples_89_event1() -> list[FakeSample]:
  return [
    FakeSample(t=159.595000000, v_ego=1.036000000, a_ego=-0.172000000, accel_cmd=-0.385000000, should_stop=False, distance_to_stop_target_m=1.900000000, raw_should_stop=False),
    FakeSample(t=159.697000000, v_ego=1.021000000, a_ego=-0.157000000, accel_cmd=-0.518000000, should_stop=False, distance_to_stop_target_m=1.900000000, raw_should_stop=False),
    FakeSample(t=159.796000000, v_ego=1.003000000, a_ego=-0.172000000, accel_cmd=-0.585000000, should_stop=True, distance_to_stop_target_m=1.900000000, raw_should_stop=False),
    FakeSample(t=159.896000000, v_ego=0.982000000, a_ego=-0.200000000, accel_cmd=-0.586000000, should_stop=True, distance_to_stop_target_m=1.900000000, raw_should_stop=False),
    FakeSample(t=159.996000000, v_ego=0.955000000, a_ego=-0.246000000, accel_cmd=-0.586000000, should_stop=True, distance_to_stop_target_m=1.900000000, raw_should_stop=False),
    FakeSample(t=160.093000000, v_ego=0.922000000, a_ego=-0.299000000, accel_cmd=-0.587000000, should_stop=True, distance_to_stop_target_m=1.399000000, raw_should_stop=True),
    FakeSample(t=160.196000000, v_ego=0.883000000, a_ego=-0.359000000, accel_cmd=-0.587000000, should_stop=True, distance_to_stop_target_m=1.399000000, raw_should_stop=True),
    FakeSample(t=160.293000000, v_ego=0.844000000, a_ego=-0.377000000, accel_cmd=-0.587000000, should_stop=True, distance_to_stop_target_m=1.399000000, raw_should_stop=True),
    FakeSample(t=160.394000000, v_ego=0.811000000, a_ego=-0.353000000, accel_cmd=-0.587000000, should_stop=True, distance_to_stop_target_m=1.399000000, raw_should_stop=True),
    FakeSample(t=160.495000000, v_ego=0.770000000, a_ego=-0.381000000, accel_cmd=-0.596000000, should_stop=True, distance_to_stop_target_m=1.399000000, raw_should_stop=True),
    FakeSample(t=160.594000000, v_ego=0.731000000, a_ego=-0.391000000, accel_cmd=-0.596000000, should_stop=True, distance_to_stop_target_m=0.900000000, raw_should_stop=True),
    FakeSample(t=160.695000000, v_ego=0.690000000, a_ego=-0.405000000, accel_cmd=-0.596000000, should_stop=True, distance_to_stop_target_m=0.900000000, raw_should_stop=True),
    FakeSample(t=160.796000000, v_ego=0.645000000, a_ego=-0.428000000, accel_cmd=-0.596000000, should_stop=True, distance_to_stop_target_m=0.900000000, raw_should_stop=True),
    FakeSample(t=160.898000000, v_ego=0.604000000, a_ego=-0.409000000, accel_cmd=-0.559000000, should_stop=True, distance_to_stop_target_m=0.900000000, raw_should_stop=True),
    FakeSample(t=160.997000000, v_ego=0.565000000, a_ego=-0.397000000, accel_cmd=-0.532000000, should_stop=True, distance_to_stop_target_m=0.900000000, raw_should_stop=True),
    FakeSample(t=161.093000000, v_ego=0.525000000, a_ego=-0.402000000, accel_cmd=-0.504000000, should_stop=True, distance_to_stop_target_m=0.500000000, raw_should_stop=True),
    FakeSample(t=161.196000000, v_ego=0.489000000, a_ego=-0.377000000, accel_cmd=-0.474000000, should_stop=True, distance_to_stop_target_m=0.500000000, raw_should_stop=True),
    FakeSample(t=161.299000000, v_ego=0.452000000, a_ego=-0.362000000, accel_cmd=-0.450000000, should_stop=True, distance_to_stop_target_m=0.500000000, raw_should_stop=True),
  ]


def _build_explicit_target_tail_hold_seed_samples_87_event5() -> list[FakeSample]:
  return [
    FakeSample(t=1340.895000000, v_ego=1.003000000, a_ego=-0.027000000, accel_cmd=-0.464000000, should_stop=True, distance_to_stop_target_m=1.593695641, raw_should_stop=False),
    FakeSample(t=1340.996000000, v_ego=1.018000000, a_ego=0.099000000, accel_cmd=-0.469000000, should_stop=True, distance_to_stop_target_m=1.593695641, raw_should_stop=False),
    FakeSample(t=1341.094000000, v_ego=1.039000000, a_ego=0.148000000, accel_cmd=-0.469000000, should_stop=True, distance_to_stop_target_m=1.593695641, raw_should_stop=True),
    FakeSample(t=1341.196000000, v_ego=0.987000000, a_ego=-0.286000000, accel_cmd=-0.472000000, should_stop=True, distance_to_stop_target_m=1.593695641, raw_should_stop=True),
    FakeSample(t=1341.296000000, v_ego=0.953000000, a_ego=-0.299000000, accel_cmd=-0.505000000, should_stop=True, distance_to_stop_target_m=1.593695641, raw_should_stop=True),
    FakeSample(t=1341.396000000, v_ego=0.911000000, a_ego=-0.384000000, accel_cmd=-0.505000000, should_stop=True, distance_to_stop_target_m=1.593695641, raw_should_stop=True),
    FakeSample(t=1341.496000000, v_ego=0.862000000, a_ego=-0.450000000, accel_cmd=-0.542000000, should_stop=True, distance_to_stop_target_m=1.193451405, raw_should_stop=True),
    FakeSample(t=1341.596000000, v_ego=0.824000000, a_ego=-0.389000000, accel_cmd=-0.542000000, should_stop=True, distance_to_stop_target_m=1.193451405, raw_should_stop=True),
    FakeSample(t=1341.695000000, v_ego=0.801000000, a_ego=-0.284000000, accel_cmd=-0.542000000, should_stop=True, distance_to_stop_target_m=1.193451405, raw_should_stop=True),
    FakeSample(t=1341.795000000, v_ego=0.790000000, a_ego=-0.174000000, accel_cmd=-0.674000000, should_stop=True, distance_to_stop_target_m=1.193451405, raw_should_stop=True),
    FakeSample(t=1341.897000000, v_ego=0.762000000, a_ego=-0.248000000, accel_cmd=-0.674000000, should_stop=True, distance_to_stop_target_m=1.193451405, raw_should_stop=True),
    FakeSample(t=1341.996000000, v_ego=0.713000000, a_ego=-0.414000000, accel_cmd=-0.770000000, should_stop=True, distance_to_stop_target_m=0.694929361, raw_should_stop=True),
    FakeSample(t=1342.096000000, v_ego=0.645000000, a_ego=-0.577000000, accel_cmd=-0.816000000, should_stop=True, distance_to_stop_target_m=0.694929361, raw_should_stop=True),
    FakeSample(t=1342.196000000, v_ego=0.572000000, a_ego=-0.671000000, accel_cmd=-0.816000000, should_stop=True, distance_to_stop_target_m=0.694929361, raw_should_stop=True),
    FakeSample(t=1342.295000000, v_ego=0.501000000, a_ego=-0.699000000, accel_cmd=-0.794000000, should_stop=True, distance_to_stop_target_m=0.694929361, raw_should_stop=True),
    FakeSample(t=1342.399000000, v_ego=0.428000000, a_ego=-0.704000000, accel_cmd=-0.758000000, should_stop=True, distance_to_stop_target_m=0.694929361, raw_should_stop=True),
    FakeSample(t=1342.498000000, v_ego=0.357000000, a_ego=-0.704000000, accel_cmd=-0.725000000, should_stop=True, distance_to_stop_target_m=0.398322344, raw_should_stop=True),
    FakeSample(t=1342.596000000, v_ego=0.296000000, a_ego=-0.643000000, accel_cmd=-0.693000000, should_stop=True, distance_to_stop_target_m=0.398322344, raw_should_stop=True),
    FakeSample(t=1342.699000000, v_ego=0.240000000, a_ego=-0.577000000, accel_cmd=-0.663000000, should_stop=True, distance_to_stop_target_m=0.398322344, raw_should_stop=True),
    FakeSample(t=1342.796000000, v_ego=0.191000000, a_ego=-0.519000000, accel_cmd=-0.635000000, should_stop=True, distance_to_stop_target_m=0.398322344, raw_should_stop=True),
    FakeSample(t=1342.893000000, v_ego=0.154000000, a_ego=-0.432000000, accel_cmd=-0.609000000, should_stop=True, distance_to_stop_target_m=0.398322344, raw_should_stop=True),
    FakeSample(t=1342.996000000, v_ego=0.115000000, a_ego=-0.402000000, accel_cmd=-0.559000000, should_stop=True, distance_to_stop_target_m=0.199293524, raw_should_stop=True),
    FakeSample(t=1343.095000000, v_ego=0.086000000, a_ego=-0.323000000, accel_cmd=-0.559000000, should_stop=True, distance_to_stop_target_m=0.199293524, raw_should_stop=True),
    FakeSample(t=1343.195000000, v_ego=0.068000000, a_ego=-0.224000000, accel_cmd=-0.535000000, should_stop=True, distance_to_stop_target_m=0.199293524, raw_should_stop=True),
    FakeSample(t=1343.297000000, v_ego=0.059000000, a_ego=-0.141000000, accel_cmd=-0.518000000, should_stop=True, distance_to_stop_target_m=0.199293524, raw_should_stop=True),
    FakeSample(t=1343.395000000, v_ego=0.051000000, a_ego=-0.100000000, accel_cmd=-0.504000000, should_stop=True, distance_to_stop_target_m=0.199293524, raw_should_stop=True),
    FakeSample(t=1343.494000000, v_ego=0.048000000, a_ego=-0.057000000, accel_cmd=-0.490000000, should_stop=True, distance_to_stop_target_m=0.194764525, raw_should_stop=True),
    FakeSample(t=1343.596000000, v_ego=0.044000000, a_ego=-0.048000000, accel_cmd=-0.476000000, should_stop=True, distance_to_stop_target_m=0.194764525, raw_should_stop=True),
    FakeSample(t=1343.696000000, v_ego=0.040000000, a_ego=-0.042000000, accel_cmd=-0.525000000, should_stop=True, distance_to_stop_target_m=0.194764525, raw_should_stop=True),
    FakeSample(t=1343.795000000, v_ego=0.034000000, a_ego=-0.055000000, accel_cmd=-1.113000000, should_stop=True, distance_to_stop_target_m=0.194764525, raw_should_stop=True),
  ]


def _build_explicit_target_micro_hold_seed_samples_316_event5() -> list[FakeSample]:
  return [
    FakeSample(t=432.484647751000, v_ego=0.017350547016, a_ego=-0.000192986699, accel_cmd=-0.362176924944, should_stop=True, distance_to_stop_target_m=0.399917125702, raw_should_stop=True),
    FakeSample(t=432.584218869000, v_ego=0.017354940996, a_ego=-0.000038901922, accel_cmd=-0.361501604319, should_stop=True, distance_to_stop_target_m=0.391717672348, raw_should_stop=True),
    FakeSample(t=432.685987659000, v_ego=0.017360348254, a_ego=0.000017962504, accel_cmd=-0.328743517399, should_stop=True, distance_to_stop_target_m=0.391717672348, raw_should_stop=True),
    FakeSample(t=432.783716439000, v_ego=0.013383337297, a_ego=-0.027284722775, accel_cmd=-0.292194902897, should_stop=True, distance_to_stop_target_m=0.391717672348, raw_should_stop=True),
    FakeSample(t=432.884772165000, v_ego=0.011928068474, a_ego=-0.017349710688, accel_cmd=-0.253748416901, should_stop=True, distance_to_stop_target_m=0.391717672348, raw_should_stop=True),
    FakeSample(t=432.984495800000, v_ego=0.012482943945, a_ego=-0.002584653208, accel_cmd=-0.214756846428, should_stop=True, distance_to_stop_target_m=0.391717672348, raw_should_stop=True),
    FakeSample(t=433.084101441000, v_ego=0.012985197827, a_ego=0.002009518445, accel_cmd=-0.176042407751, should_stop=True, distance_to_stop_target_m=0.393438369036, raw_should_stop=True),
    FakeSample(t=433.187874876000, v_ego=0.013102374971, a_ego=0.001334857079, accel_cmd=-0.137546405196, should_stop=True, distance_to_stop_target_m=0.393438369036, raw_should_stop=True),
    FakeSample(t=433.284219807000, v_ego=0.013062518090, a_ego=0.000215030785, accel_cmd=-0.106921695173, should_stop=True, distance_to_stop_target_m=0.393438369036, raw_should_stop=True),
    FakeSample(t=433.385018690000, v_ego=0.013024161570, a_ego=-0.000147615618, accel_cmd=-0.106967709959, should_stop=True, distance_to_stop_target_m=0.393438369036, raw_should_stop=True),
    FakeSample(t=433.487687810000, v_ego=0.013014758937, a_ego=-0.000102592414, accel_cmd=-0.106984570622, should_stop=True, distance_to_stop_target_m=0.393438369036, raw_should_stop=True),
    FakeSample(t=433.584776137000, v_ego=0.017615601420, a_ego=0.027924688533, accel_cmd=-0.103396639228, should_stop=True, distance_to_stop_target_m=0.392524957657, raw_should_stop=True),
    FakeSample(t=433.687134339000, v_ego=0.049836792052, a_ego=0.258309543133, accel_cmd=-0.354831635952, should_stop=True, distance_to_stop_target_m=0.392524957657, raw_should_stop=True),
    FakeSample(t=433.785664260000, v_ego=0.090259842575, a_ego=0.346946001053, accel_cmd=-0.495359629393, should_stop=True, distance_to_stop_target_m=0.392524957657, raw_should_stop=True),
    FakeSample(t=433.884876003000, v_ego=0.145562723279, a_ego=0.525113463402, accel_cmd=-0.489825576544, should_stop=True, distance_to_stop_target_m=0.392524957657, raw_should_stop=True),
    FakeSample(t=433.984447037000, v_ego=0.186402872205, a_ego=0.413949489594, accel_cmd=-0.478983640671, should_stop=True, distance_to_stop_target_m=0.392524957657, raw_should_stop=True),
    FakeSample(t=434.084506240000, v_ego=0.172515228391, a_ego=0.045297678560, accel_cmd=-0.467414140701, should_stop=True, distance_to_stop_target_m=0.283941864967, raw_should_stop=True),
  ]


def _build_explicit_target_soft_entry_carry_seed_samples_319_event6() -> list[FakeSample]:
  return [
    FakeSample(t=309.919000000, v_ego=1.089000000, a_ego=-0.366000000, accel_cmd=-0.238000000, should_stop=False, distance_to_stop_target_m=3.295000000, raw_should_stop=False),
    FakeSample(t=310.019000000, v_ego=1.068000000, a_ego=-0.268000000, accel_cmd=-0.242000000, should_stop=False, distance_to_stop_target_m=3.295000000, raw_should_stop=False),
    FakeSample(t=310.119000000, v_ego=1.048000000, a_ego=-0.226000000, accel_cmd=-0.245000000, should_stop=False, distance_to_stop_target_m=3.295000000, raw_should_stop=False),
    FakeSample(t=310.219000000, v_ego=1.031000000, a_ego=-0.190000000, accel_cmd=-0.247000000, should_stop=False, distance_to_stop_target_m=3.295000000, raw_should_stop=False),
    FakeSample(t=310.319000000, v_ego=1.016000000, a_ego=-0.164000000, accel_cmd=-0.250000000, should_stop=False, distance_to_stop_target_m=3.295000000, raw_should_stop=False),
    FakeSample(t=310.418000000, v_ego=1.002000000, a_ego=-0.151000000, accel_cmd=-0.252000000, should_stop=False, distance_to_stop_target_m=2.785000000, raw_should_stop=False),
    FakeSample(t=310.519000000, v_ego=0.989000000, a_ego=-0.137000000, accel_cmd=-0.253000000, should_stop=False, distance_to_stop_target_m=2.785000000, raw_should_stop=False),
    FakeSample(t=310.617000000, v_ego=0.975000000, a_ego=-0.136000000, accel_cmd=-0.255000000, should_stop=False, distance_to_stop_target_m=2.785000000, raw_should_stop=False),
    FakeSample(t=310.718000000, v_ego=0.963000000, a_ego=-0.126000000, accel_cmd=-0.257000000, should_stop=False, distance_to_stop_target_m=2.785000000, raw_should_stop=False),
    FakeSample(t=310.819000000, v_ego=0.953000000, a_ego=-0.112000000, accel_cmd=-0.259000000, should_stop=False, distance_to_stop_target_m=2.199000000, raw_should_stop=False),
    FakeSample(t=310.918000000, v_ego=0.943000000, a_ego=-0.100000000, accel_cmd=-0.260000000, should_stop=False, distance_to_stop_target_m=2.199000000, raw_should_stop=False),
    FakeSample(t=311.018000000, v_ego=0.935000000, a_ego=-0.092000000, accel_cmd=-0.262000000, should_stop=True, distance_to_stop_target_m=2.199000000, raw_should_stop=False),
    FakeSample(t=311.119000000, v_ego=0.926000000, a_ego=-0.085000000, accel_cmd=-0.263000000, should_stop=True, distance_to_stop_target_m=2.199000000, raw_should_stop=False),
    FakeSample(t=311.220000000, v_ego=0.919000000, a_ego=-0.083000000, accel_cmd=-0.265000000, should_stop=True, distance_to_stop_target_m=2.199000000, raw_should_stop=False),
    FakeSample(t=311.320000000, v_ego=0.917000000, a_ego=-0.038000000, accel_cmd=-0.267000000, should_stop=True, distance_to_stop_target_m=2.199000000, raw_should_stop=False),
    FakeSample(t=311.419000000, v_ego=0.920000000, a_ego=0.005000000, accel_cmd=-0.269000000, should_stop=True, distance_to_stop_target_m=1.798000000, raw_should_stop=True),
    FakeSample(t=311.518000000, v_ego=0.924000000, a_ego=0.030000000, accel_cmd=-0.271000000, should_stop=True, distance_to_stop_target_m=1.798000000, raw_should_stop=True),
    FakeSample(t=311.618000000, v_ego=0.939000000, a_ego=0.110000000, accel_cmd=-0.278000000, should_stop=True, distance_to_stop_target_m=1.798000000, raw_should_stop=True),
    FakeSample(t=311.720000000, v_ego=0.959000000, a_ego=0.175000000, accel_cmd=-0.341000000, should_stop=True, distance_to_stop_target_m=1.798000000, raw_should_stop=True),
    FakeSample(t=311.819000000, v_ego=0.985000000, a_ego=0.220000000, accel_cmd=-0.414000000, should_stop=True, distance_to_stop_target_m=1.798000000, raw_should_stop=True),
    FakeSample(t=311.919000000, v_ego=0.994000000, a_ego=0.135000000, accel_cmd=-0.488000000, should_stop=True, distance_to_stop_target_m=1.289000000, raw_should_stop=True),
    FakeSample(t=312.020000000, v_ego=0.992000000, a_ego=0.030000000, accel_cmd=-0.561000000, should_stop=True, distance_to_stop_target_m=1.289000000, raw_should_stop=True),
    FakeSample(t=312.118000000, v_ego=0.976000000, a_ego=-0.094000000, accel_cmd=-0.634000000, should_stop=True, distance_to_stop_target_m=1.289000000, raw_should_stop=True),
    FakeSample(t=312.218000000, v_ego=0.945000000, a_ego=-0.250000000, accel_cmd=-0.708000000, should_stop=True, distance_to_stop_target_m=1.289000000, raw_should_stop=True),
  ]


def _build_explicit_target_soft_entry_carry_seed_samples_31b_event2() -> list[FakeSample]:
  return [
    FakeSample(t=177.907000000, v_ego=0.952000000, a_ego=-0.254000000, accel_cmd=-0.259000000, should_stop=False, distance_to_stop_target_m=2.994000000, raw_should_stop=False),
    FakeSample(t=178.007000000, v_ego=0.928000000, a_ego=-0.245000000, accel_cmd=-0.263000000, should_stop=False, distance_to_stop_target_m=2.488000000, raw_should_stop=False),
    FakeSample(t=178.109000000, v_ego=0.908000000, a_ego=-0.216000000, accel_cmd=-0.267000000, should_stop=False, distance_to_stop_target_m=2.488000000, raw_should_stop=False),
    FakeSample(t=178.207000000, v_ego=0.891000000, a_ego=-0.187000000, accel_cmd=-0.271000000, should_stop=False, distance_to_stop_target_m=2.488000000, raw_should_stop=False),
    FakeSample(t=178.307000000, v_ego=0.873000000, a_ego=-0.183000000, accel_cmd=-0.274000000, should_stop=False, distance_to_stop_target_m=2.488000000, raw_should_stop=False),
    FakeSample(t=178.408000000, v_ego=0.855000000, a_ego=-0.180000000, accel_cmd=-0.278000000, should_stop=False, distance_to_stop_target_m=2.488000000, raw_should_stop=False),
    FakeSample(t=178.508000000, v_ego=0.838000000, a_ego=-0.169000000, accel_cmd=-0.280000000, should_stop=True, distance_to_stop_target_m=1.987000000, raw_should_stop=True),
    FakeSample(t=178.607000000, v_ego=0.824000000, a_ego=-0.151000000, accel_cmd=-0.281000000, should_stop=True, distance_to_stop_target_m=1.987000000, raw_should_stop=True),
    FakeSample(t=178.708000000, v_ego=0.809000000, a_ego=-0.156000000, accel_cmd=-0.282000000, should_stop=True, distance_to_stop_target_m=1.987000000, raw_should_stop=True),
    FakeSample(t=178.807000000, v_ego=0.793000000, a_ego=-0.149000000, accel_cmd=-0.283000000, should_stop=True, distance_to_stop_target_m=1.987000000, raw_should_stop=True),
    FakeSample(t=178.908000000, v_ego=0.781000000, a_ego=-0.132000000, accel_cmd=-0.284000000, should_stop=True, distance_to_stop_target_m=1.987000000, raw_should_stop=True),
    FakeSample(t=179.008000000, v_ego=0.776000000, a_ego=-0.074000000, accel_cmd=-0.286000000, should_stop=True, distance_to_stop_target_m=1.591000000, raw_should_stop=True),
    FakeSample(t=179.108000000, v_ego=0.782000000, a_ego=0.013000000, accel_cmd=-0.303000000, should_stop=True, distance_to_stop_target_m=1.591000000, raw_should_stop=True),
    FakeSample(t=179.207000000, v_ego=0.796000000, a_ego=0.097000000, accel_cmd=-0.391000000, should_stop=True, distance_to_stop_target_m=1.591000000, raw_should_stop=True),
    FakeSample(t=179.308000000, v_ego=0.816000000, a_ego=0.162000000, accel_cmd=-0.480000000, should_stop=True, distance_to_stop_target_m=1.591000000, raw_should_stop=True),
    FakeSample(t=179.409000000, v_ego=0.835000000, a_ego=0.181000000, accel_cmd=-0.569000000, should_stop=True, distance_to_stop_target_m=1.591000000, raw_should_stop=True),
    FakeSample(t=179.507000000, v_ego=0.846000000, a_ego=0.129000000, accel_cmd=-0.670000000, should_stop=True, distance_to_stop_target_m=1.286000000, raw_should_stop=True),
    FakeSample(t=179.609000000, v_ego=0.838000000, a_ego=-0.017000000, accel_cmd=-0.841000000, should_stop=True, distance_to_stop_target_m=1.286000000, raw_should_stop=True),
    FakeSample(t=179.709000000, v_ego=0.807000000, a_ego=-0.218000000, accel_cmd=-0.855000000, should_stop=True, distance_to_stop_target_m=1.286000000, raw_should_stop=True),
  ]


def _build_explicit_target_weak_entry_shape_seed_samples_327_event4() -> list[FakeSample]:
  return [
    FakeSample(t=1231.026000000, v_ego=0.606000000, a_ego=-0.183000000, accel_cmd=-0.124000000, should_stop=False, distance_to_stop_target_m=1.333000000, raw_should_stop=False),
    FakeSample(t=1231.125000000, v_ego=0.593000000, a_ego=-0.148000000, accel_cmd=-0.105000000, should_stop=False, distance_to_stop_target_m=1.333000000, raw_should_stop=False),
    FakeSample(t=1231.223000000, v_ego=0.581000000, a_ego=-0.135000000, accel_cmd=-0.072000000, should_stop=False, distance_to_stop_target_m=1.333000000, raw_should_stop=False),
    FakeSample(t=1231.324000000, v_ego=0.569000000, a_ego=-0.126000000, accel_cmd=-0.058000000, should_stop=True, distance_to_stop_target_m=1.333000000, raw_should_stop=False),
    FakeSample(t=1231.425000000, v_ego=0.557000000, a_ego=-0.119000000, accel_cmd=-0.139000000, should_stop=True, distance_to_stop_target_m=1.333000000, raw_should_stop=False),
    FakeSample(t=1231.524000000, v_ego=0.545000000, a_ego=-0.119000000, accel_cmd=-0.180000000, should_stop=True, distance_to_stop_target_m=1.451000000, raw_should_stop=True),
    FakeSample(t=1231.624000000, v_ego=0.537000000, a_ego=-0.096000000, accel_cmd=-0.181000000, should_stop=True, distance_to_stop_target_m=1.451000000, raw_should_stop=True),
    FakeSample(t=1231.724000000, v_ego=0.535000000, a_ego=-0.048000000, accel_cmd=-0.182000000, should_stop=True, distance_to_stop_target_m=1.451000000, raw_should_stop=True),
    FakeSample(t=1231.824000000, v_ego=0.537000000, a_ego=-0.003000000, accel_cmd=-0.184000000, should_stop=True, distance_to_stop_target_m=1.451000000, raw_should_stop=True),
    FakeSample(t=1231.923000000, v_ego=0.543000000, a_ego=0.043000000, accel_cmd=-0.186000000, should_stop=True, distance_to_stop_target_m=1.451000000, raw_should_stop=True),
    FakeSample(t=1232.025000000, v_ego=0.559000000, a_ego=0.121000000, accel_cmd=-0.224000000, should_stop=True, distance_to_stop_target_m=1.559000000, raw_should_stop=True),
    FakeSample(t=1232.123000000, v_ego=0.582000000, a_ego=0.196000000, accel_cmd=-0.304000000, should_stop=True, distance_to_stop_target_m=1.559000000, raw_should_stop=True),
  ]


def _build_explicit_target_pre_should_stop_soft_entry_seed_samples_51b_event3() -> list[FakeSample]:
  return [
    FakeSample(t=188.252000000, v_ego=0.930000000, a_ego=-0.075000000, accel_cmd=-0.264000000, should_stop=False, distance_to_stop_target_m=2.397000000, raw_should_stop=False),
    FakeSample(t=188.451000000, v_ego=0.919000000, a_ego=-0.069000000, accel_cmd=-0.265000000, should_stop=False, distance_to_stop_target_m=1.998000000, raw_should_stop=False),
    FakeSample(t=188.653000000, v_ego=0.909000000, a_ego=-0.048000000, accel_cmd=-0.435000000, should_stop=True, distance_to_stop_target_m=1.998000000, raw_should_stop=False),
    FakeSample(t=188.752000000, v_ego=0.901000000, a_ego=-0.071000000, accel_cmd=-0.410000000, should_stop=True, distance_to_stop_target_m=1.998000000, raw_should_stop=False),
    FakeSample(t=188.852000000, v_ego=0.894000000, a_ego=-0.070000000, accel_cmd=-0.404000000, should_stop=True, distance_to_stop_target_m=1.998000000, raw_should_stop=False),
    FakeSample(t=188.951000000, v_ego=0.890000000, a_ego=-0.052000000, accel_cmd=-0.406000000, should_stop=True, distance_to_stop_target_m=1.498000000, raw_should_stop=True),
    FakeSample(t=189.052000000, v_ego=0.892000000, a_ego=-0.006000000, accel_cmd=-0.408000000, should_stop=True, distance_to_stop_target_m=1.498000000, raw_should_stop=True),
    FakeSample(t=189.151000000, v_ego=0.906000000, a_ego=0.091000000, accel_cmd=-0.411000000, should_stop=True, distance_to_stop_target_m=1.498000000, raw_should_stop=True),
    FakeSample(t=189.253000000, v_ego=0.926000000, a_ego=0.161000000, accel_cmd=-0.427000000, should_stop=True, distance_to_stop_target_m=1.498000000, raw_should_stop=True),
    FakeSample(t=189.349000000, v_ego=0.951000000, a_ego=0.214000000, accel_cmd=-0.498000000, should_stop=True, distance_to_stop_target_m=1.498000000, raw_should_stop=True),
    FakeSample(t=189.452000000, v_ego=0.966000000, a_ego=0.166000000, accel_cmd=-0.572000000, should_stop=True, distance_to_stop_target_m=0.994000000, raw_should_stop=True),
    FakeSample(t=189.553000000, v_ego=0.965000000, a_ego=0.037000000, accel_cmd=-0.645000000, should_stop=True, distance_to_stop_target_m=0.994000000, raw_should_stop=True),
    FakeSample(t=189.652000000, v_ego=0.949000000, a_ego=-0.096000000, accel_cmd=-0.689000000, should_stop=True, distance_to_stop_target_m=0.994000000, raw_should_stop=True),
  ]


def _build_explicit_target_gentle_entry_hold_seed_samples_31c_event2() -> list[FakeSample]:
  return [
    FakeSample(t=198.350000000, v_ego=1.060000000, a_ego=-0.043000000, accel_cmd=-0.470000000, should_stop=False, distance_to_stop_target_m=2.293705940, raw_should_stop=False),
    FakeSample(t=198.450000000, v_ego=1.043000000, a_ego=-0.132000000, accel_cmd=-0.472000000, should_stop=False, distance_to_stop_target_m=2.293705940, raw_should_stop=False),
    FakeSample(t=198.549000000, v_ego=1.018000000, a_ego=-0.209000000, accel_cmd=-0.473000000, should_stop=True, distance_to_stop_target_m=1.796325680, raw_should_stop=True),
    FakeSample(t=198.650000000, v_ego=0.994000000, a_ego=-0.221000000, accel_cmd=-0.474000000, should_stop=True, distance_to_stop_target_m=1.796325680, raw_should_stop=True),
    FakeSample(t=198.750000000, v_ego=0.979000000, a_ego=-0.181000000, accel_cmd=-0.475000000, should_stop=True, distance_to_stop_target_m=1.796325680, raw_should_stop=True),
    FakeSample(t=198.850000000, v_ego=0.963000000, a_ego=-0.166000000, accel_cmd=-0.476000000, should_stop=True, distance_to_stop_target_m=1.796325680, raw_should_stop=True),
    FakeSample(t=198.951000000, v_ego=0.935000000, a_ego=-0.249000000, accel_cmd=-0.490000000, should_stop=True, distance_to_stop_target_m=1.197441578, raw_should_stop=True),
    FakeSample(t=199.050000000, v_ego=0.892000000, a_ego=-0.376000000, accel_cmd=-0.668000000, should_stop=True, distance_to_stop_target_m=1.197441578, raw_should_stop=True),
    FakeSample(t=199.150000000, v_ego=0.822000000, a_ego=-0.597000000, accel_cmd=-0.830000000, should_stop=True, distance_to_stop_target_m=1.197441578, raw_should_stop=True),
    FakeSample(t=199.250000000, v_ego=0.733000000, a_ego=-0.789000000, accel_cmd=-0.830000000, should_stop=True, distance_to_stop_target_m=1.197441578, raw_should_stop=True),
    FakeSample(t=199.350000000, v_ego=0.636000000, a_ego=-0.907000000, accel_cmd=-0.808000000, should_stop=True, distance_to_stop_target_m=1.197441578, raw_should_stop=True),
  ]


def _build_explicit_target_gentle_entry_hold_seed_samples_31c_event1() -> list[FakeSample]:
  return [
    FakeSample(t=73.706000000, v_ego=1.036000000, a_ego=-0.177000000, accel_cmd=-0.480000000, should_stop=False, distance_to_stop_target_m=1.998000000, raw_should_stop=False),
    FakeSample(t=73.807000000, v_ego=1.014000000, a_ego=-0.211000000, accel_cmd=-0.586000000, should_stop=True, distance_to_stop_target_m=1.998000000, raw_should_stop=False),
    FakeSample(t=73.906000000, v_ego=0.994000000, a_ego=-0.197000000, accel_cmd=-0.586000000, should_stop=True, distance_to_stop_target_m=1.998000000, raw_should_stop=False),
    FakeSample(t=74.006000000, v_ego=0.977000000, a_ego=-0.180000000, accel_cmd=-0.586000000, should_stop=True, distance_to_stop_target_m=1.998000000, raw_should_stop=False),
    FakeSample(t=74.107000000, v_ego=0.967000000, a_ego=-0.133000000, accel_cmd=-0.586000000, should_stop=True, distance_to_stop_target_m=1.498000000, raw_should_stop=True),
    FakeSample(t=74.207000000, v_ego=0.962000000, a_ego=-0.095000000, accel_cmd=-0.586000000, should_stop=True, distance_to_stop_target_m=1.498000000, raw_should_stop=True),
    FakeSample(t=74.306000000, v_ego=0.955000000, a_ego=-0.073000000, accel_cmd=-0.583000000, should_stop=True, distance_to_stop_target_m=1.498000000, raw_should_stop=True),
    FakeSample(t=74.406000000, v_ego=0.946000000, a_ego=-0.075000000, accel_cmd=-0.567000000, should_stop=True, distance_to_stop_target_m=1.498000000, raw_should_stop=True),
    FakeSample(t=74.506000000, v_ego=0.932000000, a_ego=-0.105000000, accel_cmd=-0.552000000, should_stop=True, distance_to_stop_target_m=1.498000000, raw_should_stop=True),
    FakeSample(t=74.607000000, v_ego=0.917000000, a_ego=-0.121000000, accel_cmd=-0.535000000, should_stop=True, distance_to_stop_target_m=1.498000000, raw_should_stop=True),
    FakeSample(t=74.706000000, v_ego=0.900000000, a_ego=-0.138000000, accel_cmd=-0.535000000, should_stop=True, distance_to_stop_target_m=1.498000000, raw_should_stop=True),
    FakeSample(t=74.806000000, v_ego=0.882000000, a_ego=-0.158000000, accel_cmd=-0.536000000, should_stop=True, distance_to_stop_target_m=1.498000000, raw_should_stop=True),
  ]


def _build_explicit_target_gentle_entry_hold_seed_samples_31b_event3() -> list[FakeSample]:
  return [
    FakeSample(t=716.208000000, v_ego=1.038000000, a_ego=-0.025000000, accel_cmd=-0.394000000, should_stop=False, distance_to_stop_target_m=2.296093464, raw_should_stop=False),
    FakeSample(t=716.310000000, v_ego=1.030000000, a_ego=-0.066000000, accel_cmd=-0.514000000, should_stop=True, distance_to_stop_target_m=2.296093464, raw_should_stop=False),
    FakeSample(t=716.409000000, v_ego=1.014000000, a_ego=-0.134000000, accel_cmd=-0.516000000, should_stop=True, distance_to_stop_target_m=2.296093464, raw_should_stop=False),
    FakeSample(t=716.508000000, v_ego=0.985000000, a_ego=-0.237000000, accel_cmd=-0.517000000, should_stop=True, distance_to_stop_target_m=1.799882889, raw_should_stop=True),
    FakeSample(t=716.607000000, v_ego=0.953000000, a_ego=-0.287000000, accel_cmd=-0.518000000, should_stop=True, distance_to_stop_target_m=1.799882889, raw_should_stop=True),
    FakeSample(t=716.708000000, v_ego=0.920000000, a_ego=-0.309000000, accel_cmd=-0.518000000, should_stop=True, distance_to_stop_target_m=1.799882889, raw_should_stop=True),
    FakeSample(t=716.808000000, v_ego=0.888000000, a_ego=-0.318000000, accel_cmd=-0.518000000, should_stop=True, distance_to_stop_target_m=1.799882889, raw_should_stop=True),
    FakeSample(t=716.908000000, v_ego=0.852000000, a_ego=-0.342000000, accel_cmd=-0.543000000, should_stop=True, distance_to_stop_target_m=1.799882889, raw_should_stop=True),
    FakeSample(t=717.009000000, v_ego=0.814000000, a_ego=-0.369000000, accel_cmd=-0.659000000, should_stop=True, distance_to_stop_target_m=1.296830654, raw_should_stop=True),
    FakeSample(t=717.109000000, v_ego=0.755000000, a_ego=-0.519000000, accel_cmd=-0.659000000, should_stop=True, distance_to_stop_target_m=1.296830654, raw_should_stop=True),
    FakeSample(t=717.209000000, v_ego=0.686000000, a_ego=-0.631000000, accel_cmd=-0.606000000, should_stop=True, distance_to_stop_target_m=1.296830654, raw_should_stop=True),
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


def _build_terminal_unwind_seed_samples_9ca_event7() -> list[FakeSample]:
  return [
    FakeSample(t=6.000277439, v_ego=1.096824884, a_ego=-0.406503201, accel_cmd=-0.479563534, should_stop=False),
    FakeSample(t=6.098545924, v_ego=1.042085052, a_ego=-0.500230253, accel_cmd=-0.473605365, should_stop=False),
    FakeSample(t=6.197775218, v_ego=0.977378547, a_ego=-0.607674658, accel_cmd=-0.501234055, should_stop=False),
    FakeSample(t=6.298315719, v_ego=0.913428366, a_ego=-0.627274990, accel_cmd=-0.513507843, should_stop=False),
    FakeSample(t=6.398784271, v_ego=0.849608183, a_ego=-0.627925873, accel_cmd=-0.513507843, should_stop=False),
    FakeSample(t=6.497088934, v_ego=0.784056783, a_ego=-0.652822018, accel_cmd=-0.513507843, should_stop=True),
    FakeSample(t=6.600770604, v_ego=0.723066807, a_ego=-0.625153840, accel_cmd=-0.513507843, should_stop=True),
    FakeSample(t=6.698547254, v_ego=0.658925653, a_ego=-0.637953937, accel_cmd=-0.513507843, should_stop=True),
    FakeSample(t=6.798511084, v_ego=0.597488284, a_ego=-0.621155202, accel_cmd=-0.513507843, should_stop=True),
    FakeSample(t=6.898638275, v_ego=0.534541607, a_ego=-0.624366343, accel_cmd=-0.513507843, should_stop=True),
    FakeSample(t=6.998413821, v_ego=0.480072230, a_ego=-0.579409738, accel_cmd=-0.513507843, should_stop=True),
  ]


def _build_no_target_micro_soft_landing_seed_samples_816_event1() -> list[FakeSample]:
  return [
    FakeSample(t=147.312832525, v_ego=0.068889171, a_ego=-0.196145326, accel_cmd=-0.348902345, should_stop=True, distance_to_stop_target_m=-1.0),
    FakeSample(t=147.412717885, v_ego=0.062353127, a_ego=-0.111730039, accel_cmd=-0.303154796, should_stop=True, distance_to_stop_target_m=-1.0),
    FakeSample(t=147.512249707, v_ego=0.056806121, a_ego=-0.080628559, accel_cmd=-0.322759479, should_stop=True, distance_to_stop_target_m=-1.0),
    FakeSample(t=147.612665425, v_ego=0.048084605, a_ego=-0.088342741, accel_cmd=-0.322759479, should_stop=True, distance_to_stop_target_m=-1.0),
  ]


def _build_no_target_micro_soft_landing_seed_samples_816_event4() -> list[FakeSample]:
  return [
    FakeSample(t=1257.331494155, v_ego=0.065229662, a_ego=-0.120532319, accel_cmd=-0.811533213, should_stop=False, distance_to_stop_target_m=-1.0),
    FakeSample(t=1257.431265927, v_ego=0.057447422, a_ego=-0.095490165, accel_cmd=-0.726517797, should_stop=True, distance_to_stop_target_m=-1.0),
    FakeSample(t=1257.531178634, v_ego=0.050741300, a_ego=-0.073202498, accel_cmd=-0.640991986, should_stop=True, distance_to_stop_target_m=-1.0),
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


def test_stopping_controller_late_no_target_stop_entry_capture_commits_brake_seed_0000001c_event14():
  outputs, triggers = _run_direct_controller_seed(_build_late_no_target_stop_entry_seed_samples_1c_event14())
  assert outputs[3] < -0.40
  assert outputs[4] < -0.40
  assert outputs[6] < -0.43
  assert "late_no_target_stop_entry_capture" in triggers[3]


def test_stopping_controller_explicit_target_early_entry_capture_avoids_soft_unwind_seed_00000083_event1():
  outputs, triggers = _run_direct_controller_seed(_build_explicit_target_early_entry_seed_samples_83_event1())
  assert outputs[5] < -0.50
  assert outputs[7] < -0.53
  assert outputs[9] < -0.59
  assert "explicit_target_gentle_entry_hold" in triggers[5]
  assert "explicit_target_early_entry_capture" in triggers[7]


def test_stopping_controller_explicit_target_early_entry_capture_stays_off_for_clean_deep_entry_seed_00000083_event9():
  outputs, triggers = _run_direct_controller_seed(_build_explicit_target_clean_entry_seed_samples_83_event9())
  assert outputs[5] < -0.72
  assert outputs[7] < -0.77
  assert "explicit_target_early_entry_capture" not in triggers[5]


def test_stopping_controller_explicit_target_tail_settle_relaxes_recent_good_tail_seed_00000089_event1():
  outputs, triggers = _run_direct_controller_seed(_build_explicit_target_tail_settle_seed_samples_89_event1())
  assert outputs[10] > outputs[9]
  assert outputs[11] > outputs[10]
  assert outputs[12] > -0.71
  assert "explicit_target_tail_settle" in triggers[10]
  assert "explicit_target_tail_settle" in triggers[11]


def test_stopping_controller_explicit_target_tail_hold_avoids_rebound_arrest_on_recent_seed_00000087_event5():
  outputs, triggers = _run_direct_controller_seed(_build_explicit_target_tail_hold_seed_samples_87_event5())
  assert outputs[21] > -0.40
  assert outputs[22] > -0.40
  assert outputs[23] > -0.40
  assert outputs[27] > -0.40
  assert outputs[28] > -0.40
  assert outputs[29] > -0.40
  assert "explicit_target_tail_settle" in triggers[20]
  assert "explicit_target_tail_settle" in triggers[21]
  assert "rebound_arrest_active" not in triggers[27]
  assert "rebound_arrest_active" not in triggers[28]
  assert "rebound_arrest_active" not in triggers[29]


def test_stopping_controller_explicit_target_micro_hold_prevents_creep_retry_on_bookmark_seed_00000316_event5():
  outputs, triggers = _run_direct_controller_seed(_build_explicit_target_micro_hold_seed_samples_316_event5())
  assert "explicit_target_micro_hold" in triggers[8]
  assert "explicit_target_micro_hold" in triggers[9]
  assert "explicit_target_micro_hold" in triggers[10]
  assert outputs[8] < -0.28
  assert outputs[9] < -0.30
  assert outputs[10] < -0.32
  assert outputs[11] < -0.34
  assert outputs[12] < -0.36


def test_stopping_controller_explicit_target_soft_entry_carry_avoids_late_catchup_on_bookmark_seed_00000319_event6():
  outputs, triggers = _run_direct_controller_seed(_build_explicit_target_soft_entry_carry_seed_samples_319_event6())
  assert any("explicit_target_soft_entry_carry" in trigger_list for trigger_list in triggers[11:16])
  assert "explicit_target_soft_entry_carry" in triggers[15]
  assert "explicit_target_soft_entry_carry" in triggers[18]
  assert outputs[15] > -0.34
  assert outputs[18] > -0.39
  assert outputs[22] > -0.44
  assert "rollout_push" not in triggers[17]
  assert "rollout_push" not in triggers[18]
  assert "explicit_target_tail_catch" not in triggers[23]


def test_stopping_controller_explicit_target_soft_entry_carry_covers_raw_should_stop_entry_seed_0000031b_event2():
  outputs, triggers = _run_direct_controller_seed(_build_explicit_target_soft_entry_carry_seed_samples_31b_event2())
  assert "explicit_target_soft_entry_carry" in triggers[6]
  assert "explicit_target_soft_entry_carry" in triggers[11]
  assert "explicit_target_soft_entry_carry" in triggers[15]
  assert outputs[12] > -0.35
  assert outputs[15] > -0.44
  assert outputs[18] > -0.50
  assert "severe_rebound_guard" not in triggers[15]
  assert "rollout_push" not in triggers[13]
  assert "rollout_push" not in triggers[15]
  assert "explicit_target_tail_catch" not in triggers[17]


def test_stopping_controller_explicit_target_weak_entry_uses_distance_earlier_seed_00000327_event4():
  outputs, triggers = _run_direct_controller_seed(_build_explicit_target_weak_entry_shape_seed_samples_327_event4())
  assert "explicit_target_weak_entry_shape" in triggers[3]
  assert "explicit_target_weak_entry_shape" in triggers[7]
  assert outputs[3] < -0.18
  assert outputs[7] < -0.28
  assert outputs[10] < -0.36
  assert outputs[11] > -0.58
  assert "rollout_push" not in triggers[10]


def test_stopping_controller_explicit_target_pre_should_stop_soft_entry_caps_base_jump_seed_0000051b_event3():
  outputs, triggers = _run_direct_controller_seed(_build_explicit_target_pre_should_stop_soft_entry_seed_samples_51b_event3(), use_logged_base_accel=True)
  assert "explicit_target_pre_should_stop_soft_entry" in triggers[2]
  assert "explicit_target_pre_should_stop_soft_entry" in triggers[4]
  assert outputs[2] > -0.35
  assert outputs[4] > -0.37
  assert "explicit_target_soft_entry_carry" in triggers[5]
  assert "rollout_push" not in triggers[8]
  assert "rollout_push" not in triggers[10]
  assert outputs[10] > -0.50


def test_stopping_controller_explicit_target_gentle_entry_hold_prevents_31c_tail_jab():
  outputs, triggers = _run_direct_controller_seed(_build_explicit_target_gentle_entry_hold_seed_samples_31c_event2())
  assert "explicit_target_gentle_entry_hold" in triggers[2]
  assert "explicit_target_gentle_entry_hold" in triggers[3]
  assert "explicit_target_gentle_entry_hold" in triggers[6]
  assert outputs[2] > -0.56
  assert outputs[6] > -0.60
  assert outputs[7] > -0.60
  assert outputs[8] > -0.65
  assert "explicit_target_tail_catch" not in triggers[6]
  assert "explicit_target_tail_catch" not in triggers[7]
  assert "rollout_push" not in triggers[7]


def test_stopping_controller_explicit_target_gentle_entry_hold_softens_31b_dynamic_entry():
  outputs, triggers = _run_direct_controller_seed(_build_explicit_target_gentle_entry_hold_seed_samples_31b_event3())
  assert "explicit_target_gentle_entry_hold" in triggers[3]
  assert outputs[3] > -0.45
  assert outputs[7] > -0.52
  assert outputs[8] > -0.52
  assert "explicit_target_tail_catch" not in triggers[8]
  assert "rollout_push" not in triggers[8]


def test_stopping_controller_explicit_target_gentle_entry_hold_covers_31c_dynamic_entry():
  outputs, triggers = _run_direct_controller_seed(_build_explicit_target_gentle_entry_hold_seed_samples_31c_event1())
  assert "explicit_target_gentle_entry_hold" in triggers[1]
  assert "explicit_target_gentle_entry_hold" in triggers[3]
  assert "explicit_target_gentle_rollout_block" in triggers[5]
  assert "explicit_target_gentle_rollout_block" in triggers[7]
  assert outputs[1] > -0.56
  assert outputs[3] > -0.56
  assert outputs[7] > -0.59
  assert outputs[8] > -0.59
  assert "rollout_push" not in triggers[7]
  assert "rollout_push" not in triggers[8]


def test_stopping_controller_explicit_target_rollout_relief_softens_high_rollout_tail():
  controller = StoppingController()
  controller.phase = StoppingPhase.NEAR_HOLD
  controller.low_speed_rollout_m = 1.85
  controller._last_should_stop = True
  debug: dict[str, object] = {}
  result = controller.update(
    output_accel=-0.97,
    last_output_accel=-0.97,
    should_stop=True,
    v_ego=0.55,
    a_ego=-0.55,
    max_expected_accel=-0.15,
    min_expected_accel=-0.80,
    stop_accel=-2.0,
    dt=0.01,
    distance_to_stop_target_m=0.70,
    debug=debug,
  )
  assert "explicit_target_rollout_relief" in debug["triggers"]
  assert result.output_accel > -0.968


def test_stopping_controller_explicit_target_rollout_relief_stays_off_when_target_is_still_too_far():
  controller = StoppingController()
  controller.phase = StoppingPhase.NEAR_HOLD
  controller.low_speed_rollout_m = 1.85
  controller._last_should_stop = True
  debug: dict[str, object] = {}
  _ = controller.update(
    output_accel=-0.97,
    last_output_accel=-0.97,
    should_stop=True,
    v_ego=0.55,
    a_ego=-0.55,
    max_expected_accel=-0.15,
    min_expected_accel=-0.80,
    stop_accel=-2.0,
    dt=0.01,
    distance_to_stop_target_m=0.95,
    debug=debug,
  )
  assert "explicit_target_rollout_relief" not in debug["triggers"]


def test_stopping_controller_no_target_micro_soft_landing_relaxes_short_end_stop_seed_00000816_event1():
  outputs, triggers = _run_direct_controller_seed(_build_no_target_micro_soft_landing_seed_samples_816_event1())
  assert outputs[1] > -0.24
  assert outputs[2] > -0.33
  assert "no_target_micro_soft_landing" in triggers[1]


def test_stopping_controller_no_target_micro_soft_landing_relaxes_deep_short_end_stop_seed_00000816_event4():
  outputs, triggers = _run_direct_controller_seed(_build_no_target_micro_soft_landing_seed_samples_816_event4())
  assert outputs[1] > -0.74
  assert outputs[2] > -0.65
  assert "no_target_micro_soft_landing" in triggers[1]


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
  assert outputs[6] > -0.72
  assert outputs[9] > -0.70
  assert "high_speed_reacquire_soften" in triggers[1]
  assert "high_speed_reacquire_soften" in triggers[3]
  assert "terminal_unwind_delay" in triggers[4]
  assert "terminal_unwind_relief" in triggers[6]
  assert "low_rollout_soft_landing_cap" not in triggers[6]


def test_stopping_controller_terminal_unwind_delay_avoids_late_soft_release_seed_000009cb_event4():
  outputs, triggers = _run_direct_controller_seed(_build_terminal_unwind_seed_samples_9cb_event4())
  assert outputs[1] > -0.75
  assert outputs[2] > -0.75
  assert outputs[3] < -0.75
  assert outputs[8] > -0.74
  assert outputs[10] > -0.73
  assert outputs[10] < -0.65
  assert "high_speed_reacquire_soften" in triggers[1]
  assert "high_speed_reacquire_soften" in triggers[2]
  assert "soft_landing_release" not in triggers[4]
  assert "terminal_unwind_delay" in triggers[4]
  assert "terminal_unwind_relief" in triggers[8]


def test_stopping_controller_terminal_unwind_relief_softens_late_should_stop_hold_seed_000009ca_event7():
  outputs, triggers = _run_direct_controller_seed(_build_terminal_unwind_seed_samples_9ca_event7())
  assert outputs[5] < -0.50
  assert outputs[8] > -0.50
  assert outputs[10] > -0.50
  assert "terminal_unwind_relief" in triggers[8]
  assert "terminal_unwind_relief" in triggers[10]


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
  assert "explicit_target_tail_settle" in debug_with_plan["triggers"]


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
