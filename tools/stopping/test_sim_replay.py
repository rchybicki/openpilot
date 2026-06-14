"""sim_replay tests (spec 7.6 / section 8): determinism; legacy + V2 controllers both drivable;
the integrated LongControl-with-V2 wiring (arbiter + state machine + dropout-hold pin) drivable
on the dropout fixtures; event-store scenarios round-trip."""

from __future__ import annotations

import math
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
  sys.path.insert(0, str(REPO_ROOT))

from openpilot.selfdrive.controls.lib.stopping_plant import PlantModel
from openpilot.tools.stopping import build_event_store as bes
from openpilot.tools.stopping import sim_replay as sr
from openpilot.tools.stopping.test_event_store import synthetic_stop_stream

DT = 0.1
FIXTURE_SUBSET = ("entry_seed_7af_event2", "explicit_target_clean_entry_83_event9", "terminal_unwind_9cc_event1")


def _scenario(name: str) -> sr.Scenario:
  return next(s for s in sr.fixture_scenarios() if s.name == name)


def _plant() -> PlantModel:
  return PlantModel(sr.PLANT_PARAMS_REF, DT)


class TestSimulateStop:
  @pytest.mark.parametrize("controller_name", ["legacy", "v2"])
  def test_both_controllers_drivable_and_finite(self, controller_name):
    plant = _plant()
    for name in FIXTURE_SUBSET:
      scenario = _scenario(name)
      trace = sr.simulate_stop(sr.make_controller(controller_name), plant, scenario, DT,
                               controller_name=controller_name)
      assert len(trace.u) == len(trace.v) == len(trace.t) > len(scenario.samples)
      assert all(math.isfinite(u) for u in trace.u), (controller_name, name)
      assert all(math.isfinite(v) for v in trace.v), (controller_name, name)
      assert trace.first_stop_idx is not None, (controller_name, name)

  @pytest.mark.parametrize("controller_name", ["legacy", "v2"])
  def test_determinism(self, controller_name):
    scenario = _scenario(FIXTURE_SUBSET[0])
    plant = _plant()
    t1 = sr.simulate_stop(sr.make_controller(controller_name), plant, scenario, DT)
    t2 = sr.simulate_stop(sr.make_controller(controller_name), plant, scenario, DT)
    assert t1.u == t2.u
    assert t1.v == t2.v
    assert t1.state == t2.state

  def test_integrated_wiring_on_dropout_fixtures(self):
    # spec 7.6 integrated-path requirement: state machine + single arbiter + facade + dropout pin
    plant = _plant()
    dropout_names = [s.name for s in sr.fixture_scenarios() if "dropout" in s.name or "reacquire" in s.name]
    assert dropout_names, "no dropout/reacquire fixtures found in stop_scenarios"
    for name in dropout_names[:3]:
      scenario = _scenario(name)
      trace = sr.simulate_stop(sr.make_controller("v2"), plant, scenario, DT, controller_name="v2")
      assert trace.first_stop_idx is not None, name  # the wiring must reach stopping state
      assert all(math.isfinite(u) for u in trace.u), name

  def test_v2_receives_decision_legacy_does_not(self):
    # seam contract (spec section 2): exactly one trailing kwarg, V2 branch only
    import inspect
    legacy = sr.make_controller("legacy")
    v2 = sr.make_controller("v2")
    assert "decision" not in inspect.signature(legacy.update).parameters
    assert "decision" in inspect.signature(v2.update).parameters


class TestMetricsAndRows:
  def test_event_row_fields(self):
    plant = _plant()
    scenario = _scenario(FIXTURE_SUBSET[1])
    trace = sr.simulate_stop(sr.make_controller("v2"), plant, scenario, DT,
                             controller_name="v2", plant_name="ref_20260514")
    row = sr.event_row(scenario, trace, sr.trace_metrics(trace, scenario))
    for field in ("route", "event_id", "key", "controller", "plant", "is_harsh", "is_leapfrog",
                  "harsh_flags", "leapfrog_flags", "rollout_distance_from_2mps_m",
                  "end_stop_jerk_mps3", "min_a_ego_mps2", "time_to_standstill_s"):
      assert field in row, field
    assert row["controller"] == "v2"
    assert isinstance(row["is_leapfrog"], bool)

  def test_run_replay_produces_paired_rows(self):
    scenarios = [_scenario(FIXTURE_SUBSET[0])]
    report = sr.run_replay(scenarios, ["legacy", "v2"], {"ref_20260514": sr.PLANT_PARAMS_REF}, DT)
    assert len(report["event_rows"]) == 2
    assert {r["controller"] for r in report["event_rows"]} == {"legacy", "v2"}
    assert report["scoring_config_version"] >= 1
    assert "scoring_config" in report


class TestEventStoreScenarios:
  def test_store_event_round_trips_through_the_sim(self, tmp_path: Path):
    store = tmp_path / "event_store"
    bes.write_store(store, bes.ingest_route_samples("rstore", synthetic_stop_stream()))
    scenarios = sr.load_store_scenarios(store, DT)
    assert len(scenarios) == 1
    scenario = scenarios[0]
    assert scenario.key is not None
    assert scenario.key["route"] == "rstore"
    assert scenario.stratum.startswith("v")
    trace = sr.simulate_stop(sr.make_controller("v2"), _plant(), scenario, DT, controller_name="v2")
    assert all(math.isfinite(u) for u in trace.u)
    row = sr.event_row(scenario, trace, sr.trace_metrics(trace, scenario))
    assert row["key"] == scenario.key

  def test_route_filter_and_cap(self, tmp_path: Path):
    store = tmp_path / "event_store"
    records = bes.ingest_route_samples("r1", synthetic_stop_stream())
    records += bes.ingest_route_samples("r2", synthetic_stop_stream(segment=9))
    bes.write_store(store, records)
    assert len(sr.load_store_scenarios(store, DT)) == 2
    assert len(sr.load_store_scenarios(store, DT, routes={"r1"})) == 1
    assert len(sr.load_store_scenarios(store, DT, max_events=1)) == 1


class TestPlants:
  def test_resolve_plants(self):
    plants = sr.resolve_plants("both")
    assert set(plants) == {"ref_20260514", "refit_20260531"}
    assert sr.resolve_plants("ref")["ref_20260514"] is sr.PLANT_PARAMS_REF


class TestFrictionPlantOptIn:
  """The friction plant is DEVELOPMENT-ONLY and OPT-IN: with friction=None the closed loop is
  byte-identical to before (no predicted-IMU channel, no metric); with a friction fit the wheel/gated
  loop is unchanged and only an additive predicted-IMU channel + _pred metrics appear."""

  def test_default_path_unchanged_no_imu_channel(self):
    # friction=None: no a_imu channel, no predicted-IMU metric keys (the gated path is untouched).
    scenario = _scenario(FIXTURE_SUBSET[1])
    trace = sr.simulate_stop(sr.make_controller("legacy"), _plant(), scenario, DT, controller_name="legacy")
    assert trace.a_imu == []
    metrics = sr.trace_metrics(trace, scenario)
    assert "settle_peak_imu_jerk_pred" not in metrics
    assert "settle_peak_imu_decel_pred" not in metrics

  def test_friction_does_not_change_the_closed_loop(self):
    # The controller drives off the WHEEL channel; adding friction must NOT perturb u/v/a/state.
    scenario = _scenario(FIXTURE_SUBSET[1])
    friction = sr.load_friction("default")
    base = sr.simulate_stop(sr.make_controller("legacy"), _plant(), scenario, DT, controller_name="legacy")
    with_fr = sr.simulate_stop(sr.make_controller("legacy"), _plant(), scenario, DT,
                               controller_name="legacy", friction=friction)
    assert with_fr.u == base.u
    assert with_fr.v == base.v
    assert with_fr.a == base.a
    assert with_fr.state == base.state
    # the predicted-IMU channel is the wheel accel + a non-negative-near-standstill residual
    assert len(with_fr.a_imu) == len(with_fr.a)
    for a, a_imu, v in zip(with_fr.a, with_fr.a_imu, with_fr.v, strict=True):
      assert math.isclose(a_imu, a + friction.residual(v), rel_tol=0, abs_tol=1e-12)

  def test_predicted_imu_metric_is_not_the_gating_key(self):
    # Hard guard against the founding lesson: the predicted metric is `_pred`-suffixed and is NEVER the
    # bare on-road gating `settle_peak_imu_jerk` that scoring_config reads.
    scenario = _scenario(FIXTURE_SUBSET[1])
    friction = sr.load_friction("default")
    trace = sr.simulate_stop(sr.make_controller("legacy"), _plant(), scenario, DT,
                             controller_name="legacy", friction=friction)
    metrics = sr.trace_metrics(trace, scenario)
    assert "settle_peak_imu_jerk" not in metrics  # the gating key is never produced by the sim
    if "settle_peak_imu_jerk_pred" in metrics:
      assert math.isfinite(metrics["settle_peak_imu_jerk_pred"])

  def test_load_friction_none(self):
    assert sr.load_friction(None) is None
