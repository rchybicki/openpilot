import numpy as np

from openpilot.tools.stopping.review.stop_harness import simulate


def constant_decel_episode():
  dt = 0.05
  times = np.arange(0.0, 9.0 + dt, dt)
  ego_v = np.maximum(4.0 - times / 1.5, 0.0)
  lead_v = np.maximum(0.5 - times / 12.0, 0.0)
  ego_x = np.where(times <= 6.0, 4.0 * times - times ** 2 / 3.0, 12.0)
  lead_x = 14.8 + np.where(times <= 6.0, 0.5 * times - times ** 2 / 24.0, 1.5)
  frames = []
  for t, v, lv, ld in zip(times, ego_v, lead_v, lead_x - ego_x, strict=True):
    frames.append({
      "t": round(float(t), 3), "v": float(v), "a": -2.0 / 3.0 if t <= 6.0 else 0.0,
      "sent": -0.717 if t <= 6.0 else -0.70, "aTgt": -2.0 / 3.0, "vplan": [float(v)] * 6,
      "ld": float(ld), "lv": float(lv), "la": -1.0 / 12.0 if t <= 6.0 else 0.0, "ls": True,
      "lp": 1.0, "lt": 1, "ss": bool(v < 0.05), "isd": 0.3, "fc": False, "exp": False,
      "en": True, "brk": False, "gas": False, "pitch": 0.0,
    })
  return {
    "seg": "00001f00--synthetic--0", "commit": "synthetic", "t0": 0.0, "t_rest": 6.0,
    "rest_gap": 4.3, "lead_v_at_rest": 0.0, "v_entry": 4.0, "gap_entry": 14.8,
    "driver": False, "frames": frames,
  }


def test_residual_replay_and_closure_law_hit_anchor():
  episode = constant_decel_episode()
  replay = simulate(episode, "recorded")
  closure = simulate(episode, "closure", ac=0.6, tau=0.8, lag=0.45, vown=4.5)
  assert replay["completed"] and abs(replay["rest"] - replay["rec_rest"]) < 0.05
  assert closure["completed"] and abs(closure["rest"] - 4.3) < 0.3
