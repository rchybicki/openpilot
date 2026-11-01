"""The schema that recorded the corpus. Every corpus rlog was written by the old base's cereal (the !my-fp-new stack; this copy is
cleanup/fp1031-subtract c022a9890c). The new FrogPilot base renumbered its FrogPilot structs (custom.capnp; FrogPilot refresh CLOSURE
finding 5): decoded with that cereal they read wrong values without an error, while log.capnp and car.capnp kept their ordinals.

A tree whose cereal is this schema decodes the logs with its own cereal, as before. Any other tree decodes the FrogPilot events with
this copy (the sim reads them, recorded-only fields included) and the rest with its own cereal; to_tree copies a FrogPilot event by
field name into the tree's schema where production code reads it."""
import bz2
import functools
from pathlib import Path

import capnp
import zstandard

SCHEMA = Path(__file__).resolve().parent / 'recorded_schema'
FILES = ('log.capnp', 'custom.capnp', 'legacy.capnp', 'car.capnp', 'include/c++.capnp')


@functools.cache
def log():
  return capnp.SchemaParser().load(str(SCHEMA / 'log.capnp'))


@functools.cache
def tree_is_recorded():
  """The tree's cereal is the recording schema (the old base): logs decode with it unchanged."""
  import cereal
  return all((Path(cereal.CEREAL_PATH) / f).read_bytes() == (SCHEMA / f).read_bytes() for f in FILES)


@functools.cache
def custom_services():
  """Event members whose payload is a FrogPilot struct (custom.capnp): the structs the new base renumbered."""
  out = set()
  for name, field in log().Event.schema.fields.items():
    if field.proto.which() == 'slot' and field.proto.slot.type.which() == 'struct' and field.schema.node.displayName.startswith('custom.capnp'):
      out.add(name)
  return frozenset(out)


def read_events(raw):
  from cereal import log as tree
  events = list(tree.Event.read_multiple_bytes(raw))
  if tree_is_recorded():
    return events
  custom = custom_services()
  return [r if r.which() in custom else t for t, r in zip(events, log().Event.read_multiple_bytes(raw), strict=True)]


def read_rlog(path):
  """The events of one log file (.zst, .bz2 or plain)."""
  data = Path(path).read_bytes()
  if str(path).endswith('.zst'):
    data = zstandard.ZstdDecompressor().decompress(data, max_output_size=int(1.5e9))
  elif str(path).endswith('.bz2'):
    data = bz2.decompress(data)
  return read_events(data)


def _fill(builder, d):
  fields = builder.schema.fields
  for k, v in d.items():
    if k not in fields:
      continue
    if isinstance(v, dict):
      _fill(getattr(builder, k) if fields[k].proto.which() == 'group' else builder.init(k), v)
    elif isinstance(v, list) and v and isinstance(v[0], dict):
      lst = builder.init(k, len(v))
      for i, x in enumerate(v):
        _fill(lst[i], x)
    else:
      setattr(builder, k, v)


def _copy(ev, event_type, extra):
  w = ev.which()
  new = event_type.new_message()
  new.logMonoTime, new.valid = ev.logMonoTime, ev.valid
  payload = new.init(w)
  _fill(payload, getattr(ev, w).to_dict())
  for k, v in extra.items():
    if k in payload.schema.fields:
      setattr(payload, k, v)
  return new


def is_recorded(ev):
  return ev.schema == log().Event.schema


_TREE_MEMO: dict = {}


def to_tree(ev, **extra):
  """A recorded-schema event as the tree under test reads it: copied by field name into the tree's schema (a field the recording
  lacks reads as the tree's default), with the `extra` fields the tree's payload has. A tree-schema event passes unchanged."""
  if not is_recorded(ev):
    return ev
  from cereal import log as tree
  if type(ev).__name__ != '_DynamicStructReader':   # a builder can change after a copy: copy it every time
    return _copy(ev, tree.Event, extra).as_reader()
  key = (id(ev), tuple(sorted(extra.items())))
  hit = _TREE_MEMO.get(key)
  if hit is None or hit[0] is not ev:
    if len(_TREE_MEMO) > 20000:
      _TREE_MEMO.clear()
    hit = _TREE_MEMO[key] = (ev, _copy(ev, tree.Event, extra).as_reader())
  return hit[1]


def convert(ev, **fields):
  """An event of the tree's schema (what the tree under test published) as a message (builder) of the recorded schema, copied by field
  name (fields the recorded schema lacks are left out), with `fields` set on its payload: the sim keeps one message format."""
  if tree_is_recorded():
    payload = getattr(ev, ev.which())
    for k, v in fields.items():
      setattr(payload, k, v)
    return ev
  return _copy(ev, log().Event, fields)


def empty(s):
  """An empty message of a stream the log lacks, in the schema its logged messages have (the recorded one for a FrogPilot event on a
  tree whose cereal is not the recording schema)."""
  if s in custom_services() and not tree_is_recorded():
    return _recorded_empty(s)
  from cereal import messaging
  return messaging.new_message(s).as_reader()


@functools.cache
def _recorded_empty(s):
  ev = log().Event.new_message()
  ev.valid = True
  ev.init(s)
  return ev.as_reader()
