from pathlib import Path

from openpilot.common.basedir import BASEDIR
from openpilot.frogpilot.common import frogpilot_paths

OLD_MAPS_DIR = b"/data/media/0/osm"   # the v2.0.6 based mapd's directory; its tiles stay there for a rollback


def test_mapd_and_the_settings_screens_name_one_tile_directory_of_their_own():
  # each mapd misreads the other one's tile format, so the screens that count and remove tiles must name the directory this mapd reads
  mapd = (Path(BASEDIR) / "frogpilot/navigation/mapd").read_bytes()
  tiles = frogpilot_paths.MAPS_PATH
  assert tiles.name == "offline"
  assert mapd.count(str(tiles.parent).encode()) >= 1
  assert mapd.count(OLD_MAPS_DIR) == mapd.count(str(tiles.parent).encode()), "mapd also names the old tile directory"
  assert f'QDir mapsFolderPath{{"{tiles}"}};' in (Path(BASEDIR) / "frogpilot/ui/qt/offroad/maps_settings.h").read_text()
