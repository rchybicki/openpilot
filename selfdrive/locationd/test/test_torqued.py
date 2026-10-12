import cereal.messaging as messaging
from cereal import car
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.locationd.torqued import LONG_ACC_THRESHOLD, TorqueEstimator


def test_cal_percent():
  est = TorqueEstimator(car.CarParams())
  msg = est.get_msg()
  assert msg.liveTorqueParameters.calPerc == 0

  for (low, high), min_pts in zip(est.filtered_points.buckets.keys(),
                                  est.filtered_points.buckets_min_points.values(), strict=True):
    for _ in range(int(min_pts)):
      est.filtered_points.add_point((low + high) / 2.0, 0.0)

  # enough bucket points, but not enough total points
  msg = est.get_msg()
  assert msg.liveTorqueParameters.calPerc == (len(est.filtered_points) / est.min_points_total * 100 + 100) / 2

  # add enough points to bucket with most capacity
  key = list(est.filtered_points.buckets)[0]
  for _ in range(est.min_points_total - len(est.filtered_points)):
    est.filtered_points.add_point((key[0] + key[1]) / 2.0, 0.0)

  msg = est.get_msg()
  assert msg.liveTorqueParameters.calPerc == 100


def test_long_accel_filter():
  est = TorqueEstimator(car.CarParams(), track_all_points=True)
  t = 0.0

  def step(a_ego):
    nonlocal t
    msgs = {which: messaging.new_message(which) for which in ('carControl', 'carOutput', 'carState', 'livePose')}
    msgs['carControl'].carControl.latActive = True
    msgs['carOutput'].carOutput.actuatorsOutput.torque = -0.05
    msgs['carState'].carState.vEgo = 20.0
    msgs['carState'].carState.aEgo = a_ego
    msgs['livePose'].livePose.angularVelocityDevice.z = 0.2 / 20.0
    for which, msg in msgs.items():
      est.handle_log(t, which, getattr(msg, which))
    t += DT_MDL

  # fill the history buffer so livePose starts producing points
  for _ in range(est.hist_len):
    step(0.0)

  for a_ego, admitted in ((0.2, True), (LONG_ACC_THRESHOLD, True), (-LONG_ACC_THRESHOLD, True),
                          (LONG_ACC_THRESHOLD + 0.5, False), (-LONG_ACC_THRESHOLD - 0.5, False)):
    prev_filtered, prev_all = len(est.filtered_points), len(est.all_torque_points)
    step(a_ego)
    assert len(est.filtered_points) == prev_filtered + int(admitted), a_ego
    assert len(est.all_torque_points) == prev_all + 1, a_ego
