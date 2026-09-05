#!/usr/bin/env python3
"""Triage ONE qlog segment: emit JSON line with engagement/stopping summary."""
import json
import sys

import capnp
import zstandard

REPO = "/Users/radoslawchybicki/Repos/openpilot-rch"
capnp.remove_import_hook()
LOG = capnp.load(f"{REPO}/cereal/log.capnp")


def _decompress_zst(raw):
  """Whole-frame decompression; a segment cut by a reboot (2026-09-05: 00002082--10, 00002083--0) has a partial last
  frame, so fall back to a streaming read and keep whatever decompressed before the truncation."""
  try:
    return zstandard.ZstdDecompressor().decompress(raw, max_output_size=int(9e8))
  except zstandard.ZstdError:
    out = bytearray()
    dobj = zstandard.ZstdDecompressor().decompressobj()
    for i in range(0, len(raw), 1 << 20):
      try:
        out += dobj.decompress(raw[i:i + (1 << 20)])
      except zstandard.ZstdError:
        break
    return bytes(out)


def _events_until_truncation(raw):
  events = LOG.Event.read_multiple_bytes(raw)
  try:
    yield from events
  except capnp.KjException:
    return   # the last message straddles the truncation point


def read_events(path):
  raw = open(path, "rb").read()
  if path.endswith(".zst"):
    return _events_until_truncation(_decompress_zst(raw))
  if path.endswith(".bz2"):
    import bz2
    raw = bz2.decompress(raw)
  return LOG.Event.read_multiple_bytes(raw)


def main(path):
  out = {"path": path, "gitCommit": None, "stopping_frames": 0, "enabled_frames": 0,
         "bookmarks": 0, "stop_runs": []}
  cur_run = None
  last_v = None
  t0 = None
  enabled = False
  for ev in read_events(path):
    w = ev.which()
    t = ev.logMonoTime * 1e-9
    if t0 is None:
      t0 = t
    if w == "initData":
      out["gitCommit"] = ev.initData.gitCommit[:10]
    elif w in ("userBookmark", "bookmarkButton"):
      out["bookmarks"] += 1
    elif w == "selfdriveState":
      enabled = ev.selfdriveState.enabled
      if enabled:
        out["enabled_frames"] += 1
    elif w == "carState":
      last_v = ev.carState.vEgo
    elif w == "controlsState":
      lcs = str(ev.controlsState.longControlState)
      if lcs == "stopping" and enabled:
        out["stopping_frames"] += 1
        if cur_run is None:
          cur_run = {"t_start": round(t - t0, 1), "v_in": last_v, "min_v": last_v}
        elif last_v is not None and (cur_run["min_v"] is None or last_v < cur_run["min_v"]):
          cur_run["min_v"] = last_v
      else:
        if cur_run is not None:
          cur_run["t_end"] = round(t - t0, 1)
          out["stop_runs"].append(cur_run)
          cur_run = None
  if cur_run is not None:
    cur_run["t_end"] = -1
    out["stop_runs"].append(cur_run)
  for r in out["stop_runs"]:
    for k in ("v_in", "min_v"):
      if r.get(k) is not None:
        r[k] = round(r[k], 2)
  print(json.dumps(out))


if __name__ == "__main__":
  main(sys.argv[1])
