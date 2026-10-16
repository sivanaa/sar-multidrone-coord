"""Search baselines for fleet_sim.py --search, to measure PSO against.

Each replaces a DroneBrain's PSO particle (brain.pso) with a different way
of choosing where to fly while searching. Everything else stays the same:
CBBA, the collision floor, task flying (navigate.step_toward), and the same
speed cap (the particle's max_speed), so the comparison is the search only.

- lawnmower: the area split into one strip per drone (across the drones'
  spawn order east to west), each swept back and forth in lanes one camera
  footprint apart - the standard pre-planned coverage pattern. No
  coordination needed, but no adapting either (a drone busy with a target
  leaves its strip unsearched meanwhile).
- random: fly to a uniformly random point in the area, then another.
- greedy: the coverage map's goal picking (pso.py's candidate re-scoring)
  but flying straight at the goal, without PSO's velocity update (inertia,
  cognitive and social pulls) - shows what the swarm dynamics add.
"""

import math
import random

from coordination_node.brain import SETPOINT_AVOID_RADIUS_M
from coordination_node.navigate import step_toward
from coordination_node.pso import ParticleSwarmSearch

WAYPOINT_RADIUS_M = 0.5
LANE_OVERLAP = 0.9  # lanes this fraction of the footprint width apart


class _Baseline(ParticleSwarmSearch):
    """Takes over an existing particle's state (position, best_position,
    max_speed...), so the brain can't tell the difference."""

    def __init__(self, particle):  # noqa: super().__init__ not wanted
        self.__dict__.update(particle.__dict__)

    def goal(self, x, y, neighbor_positions, candidates):
        raise NotImplementedError

    def step(self, dt, swarm_best_position, neighbor_positions=None,
             real_position=None, candidates=()):
        neighbors = neighbor_positions or []
        x, y = real_position if real_position is not None else self.state.position
        goal = self.goal(x, y, neighbors, candidates)
        self.state.best_position = goal  # neighbours see where it's going
        tracked_x, tracked_y, cx, cy = step_toward(
            self.state.position, goal, dt, max_speed=self.max_speed,
            arrival_radius=0.0, real_position=real_position,
            neighbor_positions=neighbors,
            min_separation=SETPOINT_AVOID_RADIUS_M)
        self.state.position = (tracked_x, tracked_y)
        return cx, cy


def lawnmower_waypoints(rank, count, center, radius, half_widths, start):
    """Back-and-forth lanes over strip `rank` of `count` (strips split the
    area's east extent), lanes running north-south. `half_widths` = the
    sensor's (half_north, half_east)."""
    cx, cy = center
    lo = cy - radius + 2 * radius * rank / count
    hi = cy - radius + 2 * radius * (rank + 1) / count
    spacing = 2 * half_widths[1] * LANE_OVERLAP
    lanes = max(1, math.ceil((hi - lo) / spacing))
    points = []
    for k in range(lanes):
        y = lo + (k + 0.5) * (hi - lo) / lanes
        # Long enough for the widest part of the area the lane's swath
        # covers (its edge nearest the centre): the chord at the lane's own
        # y missed the boundary corners, 4-10 missions in 60 never found
        # someone there.
        edge = max(0.0, abs(y - cy) - half_widths[1])
        chord = math.sqrt(max(0.0, radius ** 2 - edge ** 2))
        # Footprint edge on the boundary, allowing for the drone turning
        # WAYPOINT_RADIUS_M early.
        inset = max(0.0, min(half_widths[0], chord) - WAYPOINT_RADIUS_M - 0.3)
        ends = [(cx - chord + inset, y), (cx + chord - inset, y)]
        points += ends if k % 2 == 0 else ends[::-1]
    # Start from whichever end of the pattern is nearer.
    if math.dist(start, points[-1]) < math.dist(start, points[0]):
        points.reverse()
    return points


class Lawnmower(_Baseline):

    def __init__(self, particle, rank, count, center, radius, half_widths):
        super().__init__(particle)
        self.waypoints = lawnmower_waypoints(
            rank, count, center, radius, half_widths, particle.state.position)
        self.next = 0

    def goal(self, x, y, neighbor_positions, candidates):
        if math.dist((x, y), self.waypoints[self.next]) <= WAYPOINT_RADIUS_M:
            self.next += 1
            if self.next == len(self.waypoints):  # done: sweep it again
                self.waypoints.reverse()
                self.next = 0
        return self.waypoints[self.next]


class RandomWalk(_Baseline):

    def __init__(self, particle, center, radius, rng):
        super().__init__(particle)
        self.center, self.radius, self.rng = center, radius, rng
        self.target = self._draw()

    def _draw(self):
        r = self.radius * math.sqrt(self.rng.random())
        a = self.rng.uniform(-math.pi, math.pi)
        return (self.center[0] + r * math.cos(a),
                self.center[1] + r * math.sin(a))

    def goal(self, x, y, neighbor_positions, candidates):
        if math.dist((x, y), self.target) <= WAYPOINT_RADIUS_M:
            self.target = self._draw()
        return self.target


class Greedy(_Baseline):

    def goal(self, x, y, neighbor_positions, candidates):
        self._rescore_personal_best(x, y, neighbor_positions, candidates)
        return self.state.best_position


SEARCHES = ('pso', 'lawnmower', 'random', 'greedy')


def install(search, brains, spawns, center, radius, half_widths, seed):
    """Swap every brain's particle for `search` (no-op for 'pso')."""
    if search == 'pso':
        return
    order = sorted(range(len(brains)), key=lambda i: (spawns[i][1], spawns[i][0]))
    for rank, i in enumerate(order):
        b = brains[i]
        if search == 'lawnmower':
            b.pso = Lawnmower(b.pso, rank, len(brains), center, radius,
                              half_widths)
        elif search == 'random':
            b.pso = RandomWalk(b.pso, center, radius,
                               random.Random(seed * 100 + i))
        elif search == 'greedy':
            b.pso = Greedy(b.pso)
        else:
            raise ValueError(f'unknown search {search!r}')
