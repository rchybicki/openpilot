import numpy as np
from enum import Enum
from openpilot.selfdrive.mapd.lib.geo import DIRECTION, R, vectors


_DIVERTION_SEARCH_RANGE = [-200., 50.]  # mt. Range of distance to current location for diversion search.


def nodes_raw_data_array_for_wr(wr, drop_last=False):
  """Provides an array of raw node data (id, lat, lon, speed_limit) for all nodes in way relation
  """
  sl = wr.speed_limit
  data = np.array([(n.id, n.lat, n.lon, sl) for n in wr.way.nodes], dtype=float)

  # reverse the order if way direction is backwards
  if wr.direction == DIRECTION.BACKWARD:
    data = np.flip(data, axis=0)

  # drop last if requested
  return data[:-1] if drop_last else data


def node_calculations(points):
  """Provides node calculations based on an array of (lat, lon) points in radians.
     points is a (N x 1) array where N >= 3
  """
  if len(points) < 3:
    raise(IndexError)

  # Get the vector representation of node points in cartesian plane.
  # (N-1, 2) array. Not including (0., 0.)
  v = vectors(points) * R

  # Calculate the vector magnitudes (or distance)
  # (N-1, 1) array. No distance for v[-1]
  d = np.linalg.norm(v, axis=1)

  # Calculate the bearing (from true north clockwise) for every node.
  # (N-1, 1) array. No bearing for v[-1]
  b = np.arctan2(v[:, 0], v[:, 1])

  # Add origin to vector space. (i.e first node in list)
  v = np.concatenate(([[0., 0.]], v))

  # Provide distance to previous node and distance to next node
  dp = np.concatenate(([0.], d))
  dn = np.concatenate((d, [0.]))

  # Provide cumulative distance on route
  dr = np.cumsum(dp, axis=0)

  # Bearing of last node should keep bearing from previous.
  b = np.concatenate((b, [b[-1]]))

  return v, dp, dn, dr, b


def is_wr_a_valid_divertion_from_node(wr, node_id, wr_ids):
  """
  Evaluates if the way relation `wr` is a valid diversion from node with id `node_id`.
  A valid diversion is a way relation with an edge node with the given `node_id` that is not already included
  in the list of way relations in the route (`wr_ids`) and that can be travaled in the direction as if starting
  from node with id `node_id`
  """
  if wr.id in wr_ids:
    return False
  wr.update_direction_from_starting_node(node_id)
  return not wr.is_prohibited


class SpeedLimitSection():
  """And object representing a speed limited road section ahead.
  provides the start and end distance and the speed limit value
  """
  def __init__(self, start, end, value):
    self.start = start
    self.end = end
    self.value = value

  def __repr__(self):
    return f'from: {self.start}, to: {self.end}, limit: {self.value}'

class NodeDataIdx(Enum):
  """Column index for data elements on NodesData underlying data store.
  """
  node_id = 0
  lat = 1
  lon = 2
  speed_limit = 3
  x = 4             # x value of cartesian vector representing the section between last node and this node.
  y = 5             # y value of cartesian vector representing the section between last node and this node.
  dist_prev = 6     # distance to previous node.
  dist_next = 7     # distance to next node
  dist_route = 8    # cumulative distance on route
  bearing = 9       # bearing of the vector departing from this node.


class NodesData:
  """Container for the list of node data from a ordered list of way relations to be used in a Route
  """
  def __init__(self, way_relations, wr_index):
    self._nodes_data = np.array([])
    self._divertions = [[]]

    way_count = len(way_relations)
    if way_count == 0:
      return

    # We want all the nodes from the last way section
    nodes_data = nodes_raw_data_array_for_wr(way_relations[-1])

    # For the ways before the last in the route we want all the nodes but the last, as that one is the first on
    # the next section. Collect them, append last way node data and concatenate the numpy arrays.
    if way_count > 1:
      wrs_data = tuple([nodes_raw_data_array_for_wr(wr, drop_last=True) for wr in way_relations[:-1]])
      wrs_data += (nodes_data,)
      nodes_data = np.concatenate(wrs_data)

    # Get a subarray with lat, lon to compute the remaining node values.
    lat_lon_array = nodes_data[:, [1, 2]]
    points = np.radians(lat_lon_array)
    # Ensure we have more than 3 points, if not calculations are not possible.
    if len(points) <= 3:
      return
    vect, dist_prev, dist_next, dist_route, bearing = node_calculations(points)

    # append calculations to nodes_data
    # nodes_data structure: [id, lat, lon, speed_limit, x, y, dist_prev, dist_next, dist_route, bearing]
    self._nodes_data = np.column_stack((nodes_data, vect, dist_prev, dist_next, dist_route, bearing))

    # Build route diversion options data from the wr_index.
    wr_ids = [wr.id for wr in way_relations]
    self._divertions = [[wr for wr in wr_index.way_relations_with_edge_node_id(node_id)
                        if is_wr_a_valid_divertion_from_node(wr, node_id, wr_ids)]
                        for node_id in nodes_data[:, 0]]

  @property
  def count(self):
    return len(self._nodes_data)

  def get(self, node_data_idx):
    """Returns the array containing all the elements of a specific NodeDataIdx type.
    """
    if len(self._nodes_data) == 0 or node_data_idx.value >= self._nodes_data.shape[1]:
      return np.array([])

    return self._nodes_data[:, node_data_idx.value]

  def speed_limits_ahead(self, ahead_idx, distance_to_node_ahead):
    """Returns and array of SpeedLimitSection objects for the actual route ahead of current location
    """
    if len(self._nodes_data) == 0 or ahead_idx is None:
      return []

    # Find the cumulative distances where speed limit changes. Build Speed limit sections for those.
    dist = np.concatenate(([distance_to_node_ahead], self.get(NodeDataIdx.dist_next)[ahead_idx:]))
    dist = np.cumsum(dist, axis=0)
    sl = self.get(NodeDataIdx.speed_limit)[ahead_idx - 1:]
    sl_next = np.concatenate((sl[1:], [0.]))

    # Create a boolean mask where speed limit changes and filter values
    sl_change = sl != sl_next
    distances = dist[sl_change]
    speed_limits = sl[sl_change]

    # Create speed limits sections combining all continuous nodes that have same speed limit value.
    start = 0.
    limits_ahead = []
    for idx, end in enumerate(distances):
      limits_ahead.append(SpeedLimitSection(start, end, speed_limits[idx]))
      start = end

    return limits_ahead

  def distance_to_end(self, ahead_idx, distance_to_node_ahead):
    if len(self._nodes_data) == 0 or ahead_idx is None:
      return None

    return np.sum(np.concatenate(([distance_to_node_ahead], self.get(NodeDataIdx.dist_next)[ahead_idx:])))


  def possible_divertions(self, ahead_idx, distance_to_node_ahead):
    """ Returns and array with the way relations the route could possible divert to by finding
        the alternative way diversions on the nodes in the vicinity of the current location.
    """
    if len(self._nodes_data) == 0 or ahead_idx is None:
      return []

    dist_route = self.get(NodeDataIdx.dist_route)
    rel_dist = dist_route - dist_route[ahead_idx] + distance_to_node_ahead
    valid_idxs = np.nonzero(np.logical_and(rel_dist >= _DIVERTION_SEARCH_RANGE[0],
                            rel_dist <= _DIVERTION_SEARCH_RANGE[1]))[0]
    valid_divertions = [self._divertions[i] for i in valid_idxs]

    return [wr for wrs in valid_divertions for wr in wrs]  # flatten.

  def distance_to_node(self, node_id, ahead_idx, distance_to_node_ahead):
    """
    Provides the distance to a specific node in the route identified by `node_id` in reference to the node ahead
    (`ahead_idx`) and the distance from current location to the node ahead (`distance_to_node_ahead`).
    """
    node_ids = self.get(NodeDataIdx.node_id)
    node_idxs = np.nonzero(node_ids == node_id)[0]
    if len(self._nodes_data) == 0 or ahead_idx is None or len(node_idxs) == 0:
      return None

    return self.get(NodeDataIdx.dist_route)[node_idxs[0]] - self.get(NodeDataIdx.dist_route)[ahead_idx] + \
      distance_to_node_ahead
