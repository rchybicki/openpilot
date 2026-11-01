import io
import json
import pickle
import shutil
import struct
import tempfile
from pathlib import Path

MODELS_DIR = Path(__file__).resolve().parent / 'models'
TG_INPUT_DEVICES_PATH = MODELS_DIR / 'tg_input_devices.json'
MODELD_PKL_PATH = MODELS_DIR / 'driving_tinygrad.pkl'


def get_tg_input_devices(process_name: str):
  with open(TG_INPUT_DEVICES_PATH) as f:
    return json.load(f)[process_name]['default']


def dump_oob(obj, f):
  with tempfile.TemporaryFile(dir=".") as tmp:
    def buffer_callback(pb: pickle.PickleBuffer):
      m = pb.raw()
      tmp.write(struct.pack('<q', m.nbytes))
      tmp.write(m)
      pb.release()
    stream = io.BytesIO()
    pickle.Pickler(stream, protocol=5, buffer_callback=buffer_callback).dump(obj)
    opcodes = stream.getvalue()
    f.write(struct.pack('<q', len(opcodes)))
    f.write(opcodes)
    tmp.seek(0)
    shutil.copyfileobj(tmp, f)


def load_oob(f):
  opcodes = f.read(struct.unpack('<q', f.read(8))[0])

  def buffers():
    while h := f.read(8):
      pb = pickle.PickleBuffer(bytearray(struct.unpack('<q', h)[0]))
      f.readinto(pb)
      yield pb

  return pickle.load(io.BytesIO(opcodes), buffers=buffers())
