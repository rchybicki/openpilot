import random
import numpy as np
import time
import pytest

from cereal import messaging, log, car
from openpilot.selfdrive.locationd.lagd import LateralLagEstimator, retrieve_initial_lag, masked_normalized_cross_correlation, \
                                               BLOCK_NUM_NEEDED, BLOCK_SIZE, MIN_OKAY_WINDOW_SEC, MIN_VEGO, VERSION, MIN_LAG, MAX_LAG, \
                                               masked_symmetric_moving_average
from openpilot.selfdrive.test.process_replay.migration import migrate, migrate_carParams
from openpilot.selfdrive.locationd.test.test_locationd_scenarios import TEST_ROUTE
from openpilot.common.constants import CV
from openpilot.common.params import Params
from openpilot.tools.lib.logreader import LogReader
from openpilot.system.hardware import PC

MAX_ERR_FRAMES = 1
DT = 0.05
LAGD_MIN_LAG_FRAMES, LAGD_MAX_LAG_FRAMES = int(round(MIN_LAG / DT)), int(round(MAX_LAG / DT))


def process_messages(estimator, lag_frames, n_frames, vego=25.0, rejection_threshold=0.0, amplitude=0.3):
  for i in range(n_frames):
    t = i * estimator.dt
    desired_la = np.cos(10 * t) * amplitude
    actual_la = np.cos(10 * (t - lag_frames * estimator.dt)) * amplitude

    # if sample is masked out, set it to desired value (no lag)
    rejected = random.uniform(0, 1) < rejection_threshold
    if rejected:
      actual_la = desired_la

    desired_cuvature = float(desired_la / (vego ** 2))
    actual_yr = float(actual_la / vego)
    msgs = [
      (t, "carControl", car.CarControl(latActive=not rejected)),
      (t, "carState", car.CarState(vEgo=vego, steeringPressed=False)),
      (t, "controlsState", log.ControlsState(desiredCurvature=desired_cuvature)),
      (t, "livePose", log.LivePose(angularVelocityDevice=log.LivePose.XYZMeasurement(z=actual_yr, valid=True),
                                   posenetOK=True, inputsOK=True)),
      (t, "liveCalibration", log.LiveCalibrationData(rpyCalib=[0, 0, 0], calStatus=log.LiveCalibrationData.Status.calibrated)),
    ]
    for t, w, m in msgs:
      estimator.handle_log(t, w, m)
    estimator.update_points()
    estimator.update_estimate()


class TestLagd:
  def test_read_saved_params(self):
    params = Params()

    lr = migrate(LogReader(TEST_ROUTE), [migrate_carParams])
    CP = next(m for m in lr if m.which() == "carParams").carParams

    msg = messaging.new_message('liveDelay')
    msg.liveDelay.lateralDelayEstimate = random.random()
    msg.liveDelay.validBlocks = random.randint(1, 10)
    msg.liveDelay.version = VERSION
    params.put("LiveDelay", msg.to_bytes())
    params.put("CarParamsPrevRoute", CP.as_builder().to_bytes())

    saved_lag_params = retrieve_initial_lag(params, CP)
    assert saved_lag_params is not None

    lag, valid_blocks = saved_lag_params
    assert lag == msg.liveDelay.lateralDelayEstimate
    assert valid_blocks == msg.liveDelay.validBlocks

  def test_read_invalid_saved_params(self, subtests):
    params = Params()

    lr = migrate(LogReader(TEST_ROUTE), [migrate_carParams])
    CP = next(m for m in lr if m.which() == "carParams").carParams

    for msg_dict in [{'version': 0}, {'status': 'invalid'}, {'validBlocks': 100}]:
      with subtests.test(msg=f"liveDelay={msg_dict}"):
        msg = messaging.new_message('liveDelay')
        msg.liveDelay = msg_dict
        params.put("LiveDelay", msg.to_bytes())
        params.put("CarParamsPrevRoute", CP.as_builder().to_bytes())
        assert retrieve_initial_lag(params, CP) is None

  def test_ncc(self):
    lag_frames = random.randint(1, 19)

    desired_sig = np.sin(np.arange(0.0, 10.0, 0.1))
    actual_sig = np.sin(np.arange(0.0, 10.0, 0.1) - lag_frames * 0.1)
    mask = np.ones(len(desired_sig), dtype=bool)

    corr = masked_normalized_cross_correlation(desired_sig, actual_sig, mask, 200)[len(desired_sig) - 1:len(desired_sig) + 20]
    assert np.argmax(corr) == lag_frames

    # add some noise
    desired_sig += np.random.normal(0, 0.05, len(desired_sig))
    actual_sig += np.random.normal(0, 0.05, len(actual_sig))
    corr = masked_normalized_cross_correlation(desired_sig, actual_sig, mask, 200)[len(desired_sig) - 1:len(desired_sig) + 20]
    assert np.argmax(corr)  in range(lag_frames - MAX_ERR_FRAMES, lag_frames + MAX_ERR_FRAMES + 1)

    # mask out 40% of the values, and make them noise
    mask = np.random.choice([True, False], size=len(desired_sig), p=[0.6, 0.4])
    desired_sig[~mask] = np.random.normal(0, 1, size=np.sum(~mask))
    actual_sig[~mask] = np.random.normal(0, 1, size=np.sum(~mask))
    corr = masked_normalized_cross_correlation(desired_sig, actual_sig, mask, 200)[len(desired_sig) - 1:len(desired_sig) + 20]
    assert np.argmax(corr) in range(lag_frames - MAX_ERR_FRAMES, lag_frames + MAX_ERR_FRAMES + 1)

  def test_empty_estimator(self):
    mocked_CP = car.CarParams(steerActuatorDelay=0.5)
    estimator = LateralLagEstimator(mocked_CP, DT)
    msg = estimator.get_msg(True)
    assert msg.liveDelay.status == 'unestimated'
    assert np.allclose(msg.liveDelay.lateralDelay, estimator.initial_lag)
    assert np.allclose(msg.liveDelay.lateralDelayEstimate, estimator.initial_lag)
    assert msg.liveDelay.validBlocks == 0
    assert msg.liveDelay.calPerc == 0

  @pytest.mark.parametrize("lag_frames", [LAGD_MIN_LAG_FRAMES, (LAGD_MIN_LAG_FRAMES + LAGD_MAX_LAG_FRAMES) // 2, LAGD_MAX_LAG_FRAMES - 2])
  def test_estimator_basics(self, lag_frames):
    mocked_CP = car.CarParams(steerActuatorDelay=0.5)
    estimator = LateralLagEstimator(mocked_CP, DT, min_recovery_buffer_sec=0.0, min_yr=0.0)
    process_messages(estimator, lag_frames, int(MIN_OKAY_WINDOW_SEC / DT) + BLOCK_NUM_NEEDED * BLOCK_SIZE)
    msg = estimator.get_msg(True)
    assert msg.liveDelay.status == 'estimated'
    assert np.allclose(msg.liveDelay.lateralDelay, lag_frames * DT, atol=0.01)
    assert np.allclose(msg.liveDelay.lateralDelayEstimate, lag_frames * DT, atol=0.01)
    assert np.allclose(msg.liveDelay.lateralDelayEstimateStd, 0.0, atol=0.01)
    assert msg.liveDelay.validBlocks == BLOCK_NUM_NEEDED
    assert msg.liveDelay.calPerc == 100

  @pytest.mark.parametrize("learned, published", [(0.0, MIN_LAG), (0.9, MAX_LAG)])
  def test_published_delay_is_clamped(self, learned, published):
    # controlsd divides by the published delay (no smoothing delay is added to it): a zero or out-of-range lag never reaches it
    estimator = LateralLagEstimator(car.CarParams(steerActuatorDelay=0.5), DT)
    estimator.reset(learned, BLOCK_NUM_NEEDED + 1)
    msg = estimator.get_msg(True)
    assert msg.liveDelay.status == 'estimated'
    assert msg.liveDelay.lateralDelay == pytest.approx(published)
    assert msg.liveDelay.lateralDelayEstimate == pytest.approx(learned)

  def test_small_lateral_accel_range_is_not_learned(self):
    # 0.2 m/s^2 peak to peak is below MIN_LAT_ACCEL_RANGE: straight-road noise teaches no lag
    estimator = LateralLagEstimator(car.CarParams(steerActuatorDelay=0.5), DT, min_recovery_buffer_sec=0.0, min_yr=0.0)
    process_messages(estimator, LAGD_MIN_LAG_FRAMES + 2, int(MIN_OKAY_WINDOW_SEC / DT) + BLOCK_NUM_NEEDED * BLOCK_SIZE, amplitude=0.1)
    msg = estimator.get_msg(True)
    assert msg.liveDelay.validBlocks == 0
    assert msg.liveDelay.status == 'unestimated'

  @pytest.mark.parametrize("lag_frames", [LAGD_MIN_LAG_FRAMES, 8, LAGD_MAX_LAG_FRAMES])
  def test_actuator_delay_finds_lags_in_the_window(self, lag_frames):
    desired = np.random.default_rng(0).normal(0, 1, 600)
    actual = np.concatenate([np.zeros(lag_frames), desired[:-lag_frames]])
    delay, corr, confidence = LateralLagEstimator.actuator_delay(desired, actual, np.ones(600, dtype=bool), DT, MIN_LAG, MAX_LAG)
    assert delay == pytest.approx(lag_frames * DT, abs=0.01)
    assert corr > 0.95 and confidence > 0.7

  @pytest.mark.parametrize("lag_frames", [0, 1, LAGD_MAX_LAG_FRAMES + 5])
  def test_actuator_delay_has_no_confidence_for_lags_outside_the_window(self, lag_frames):
    # the true peak lies outside MIN_LAG..MAX_LAG: the best in-window value is not a candidate and must not be learned
    desired = np.random.default_rng(0).normal(0, 1, 600)
    actual = np.concatenate([np.zeros(lag_frames), desired[:-lag_frames]]) if lag_frames else desired.copy()
    _, _, confidence = LateralLagEstimator.actuator_delay(desired, actual, np.ones(600, dtype=bool), DT, MIN_LAG, MAX_LAG)
    assert confidence == 0.0

  def test_masked_samples_do_not_reach_the_smoothed_signal(self):
    x = np.linspace(0.0, 1.0, 21)
    mask = np.ones(21, dtype=bool)
    x[10], mask[10] = np.nan, False
    smoothed = masked_symmetric_moving_average(x, mask, 5, 1.0)
    assert np.all(np.isfinite(smoothed))

  def test_masked_outlier_does_not_count_toward_the_lat_accel_range(self):
    # 0.2 m/s^2 of valid range plus one masked 5 m/s^2 sample: still too little range to learn from
    estimator = LateralLagEstimator(car.CarParams(steerActuatorDelay=0.5), DT, min_recovery_buffer_sec=0.0, min_yr=0.0)
    n = estimator.points.num_points
    for i in range(n):
      t = i * DT
      desired, actual, okay = np.cos(10 * t) * 0.1, np.cos(10 * (t - 6 * DT)) * 0.1, True
      if i == n // 2:
        actual, okay = 5.0, False
      estimator.t = t
      estimator.points.update(t, desired, actual, okay)
    estimator.update_estimate()
    assert estimator.last_estimate_t == 0

  def test_estimator_masking(self):
    mocked_CP, lag_frames = car.CarParams(steerActuatorDelay=0.5), random.randint(LAGD_MIN_LAG_FRAMES, LAGD_MAX_LAG_FRAMES - 1)
    estimator = LateralLagEstimator(mocked_CP, DT, min_recovery_buffer_sec=0.0, min_yr=0.0, min_valid_block_count=1)
    process_messages(estimator, lag_frames, (int(MIN_OKAY_WINDOW_SEC / DT) + BLOCK_SIZE) * 2, rejection_threshold=0.4)
    msg = estimator.get_msg(True)
    assert np.allclose(msg.liveDelay.lateralDelayEstimate, lag_frames * DT, atol=0.01)
    assert np.allclose(msg.liveDelay.lateralDelayEstimateStd, 0.0, atol=0.01)
    assert msg.liveDelay.calPerc == 100

  @pytest.mark.skipif(PC, reason="only on device")
  @pytest.mark.timeout(60)
  def test_estimator_performance(self):
    mocked_CP = car.CarParams(steerActuatorDelay=0.5)
    estimator = LateralLagEstimator(mocked_CP, DT)

    ds = []
    for _ in range(1000):
      st = time.perf_counter()
      estimator.update_points()
      estimator.update_estimate()
      d = time.perf_counter() - st
      ds.append(d)

    assert np.mean(ds) < DT


def test_min_vego_is_50_mph():
  assert MIN_VEGO == pytest.approx(50 * CV.MPH_TO_MS)


@pytest.mark.parametrize("vego_mph, estimated", [(45, False), (49, False), (51, True), (60, True)])
def test_lag_is_learned_only_at_or_above_50_mph(vego_mph, estimated):
  estimator = LateralLagEstimator(car.CarParams(steerActuatorDelay=0.5), DT, min_recovery_buffer_sec=0.0, min_yr=0.0)
  process_messages(estimator, (LAGD_MIN_LAG_FRAMES + LAGD_MAX_LAG_FRAMES) // 2, int(MIN_OKAY_WINDOW_SEC / DT) + BLOCK_NUM_NEEDED * BLOCK_SIZE,
                   vego=vego_mph * CV.MPH_TO_MS)
  msg = estimator.get_msg(True)
  assert (msg.liveDelay.validBlocks == BLOCK_NUM_NEEDED) == estimated
  assert (msg.liveDelay.status == 'estimated') == estimated
