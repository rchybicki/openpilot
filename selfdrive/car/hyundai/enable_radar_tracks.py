import time
from typing import NamedTuple

from selfdrive.car.isotp_parallel_query import IsoTpParallelQuery
from system.swaglog import cloudlog

DIAG_REQUEST = b'\x10\x07'
DIAG_RESPONSE = b'\x50\x07'

WRITE_DATA_REQUEST = b'\x2e'
WRITE_DATA_RESPONSE = b'\x68'


class ConfigValues(NamedTuple):
  tracks_enabled: bytes


# If your radar supports changing data identifier 0x0142 as well make a PR to
# this file to add your firmware version. Make sure to post a drive as proof!
SUPPORTED_FW_VERSIONS = {
  # 2022 Santa Fe
  b'\xf1\x00TM__ SCC FHCUP      1.00 1.00 99110-S1500         ': ConfigValues(
    tracks_enabled=b"\x00\x00\x00\x01\x00\x01"
  ),
  # 2022 Santa Fe HEV
  b'\xf1\x00TMhe SCC FHCUP      1.00 1.00 99110-CL500         ': ConfigValues(
    tracks_enabled=b"\x00\x00\x00\x01\x00\x01"
  ),
}


def _enable_radar_tracks(logcan, sendcan, fw_version, bus=0, addr=0x7d0, config_data_id=b'\x01\x42', timeout=0.1, retry=10, debug=False):
  cloudlog.warning("radar_tracks: enabling...")

  for i in range(retry):
    try:
      query = IsoTpParallelQuery(sendcan, logcan, bus, [addr], [DIAG_REQUEST], [DIAG_RESPONSE], debug=debug)

      for _, _ in query.get_data(timeout).items():
        _radar_tracks_enable_query(logcan, sendcan, fw_version, bus, addr, config_data_id, debug)
        break

    except Exception as e:
      time.sleep(3)
      cloudlog.exception(f"radar_tracks exception: {e}")

    cloudlog.warning(f"radar_tracks retry ({i + 1}) ...")
  cloudlog.exception(f"radar_tracks: failed")


def _radar_tracks_enable_query(logcan, sendcan, fw_version, bus, addr, config_data_id, debug):
  config_value = SUPPORTED_FW_VERSIONS[fw_version]
  new_config = config_value.tracks_enabled

  query = IsoTpParallelQuery(sendcan, logcan, bus, [addr],
                             [WRITE_DATA_REQUEST + config_data_id + new_config], [WRITE_DATA_RESPONSE], debug=debug)
  query.get_data(0)

  cloudlog.warning("radar_tracks: successfully enabled")


def enable_radar_tracks(CP, logcan, sendcan):
  cloudlog.warning("radar_tracks: Try to enable radar tracks")

  radarFw = next((fw for fw in CP.carFw if fw.ecu == "fwdRadar"), None)

  if radarFw is not None and radarFw.fwVersion in SUPPORTED_FW_VERSIONS.keys():
    _enable_radar_tracks(logcan, sendcan, radarFw.fwVersion)
  else:
    cloudlog.exception(f"radar_tracks: radar not supported!\n version:\n{radarFw.fwVersion}\n address:{radarFw.address}\n Skipping...")
