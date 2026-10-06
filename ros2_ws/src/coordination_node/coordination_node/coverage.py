"""Coverage map: which parts of the search area have been seen recently.

Replaces nearest-neighbor spread as the PSO search fitness (2026-10-06).
Spread had no notion of where anyone had already looked, and - because a
drone's personal best was scored once and never re-scored as neighbors
moved - a drone's spawn point (scored while the other drone was still 7m
away) stayed its "best" forever: drone 0 parked at its spawn in every live
run and every dry run, and became the swarm best that pulled drone 1 onto
it (Test A's "pinned together").

Each node builds its own map from positions it already gets: its own every
tick, and each neighbor's from AgentState. No new messages. A cell counts as
seen once any drone has been within `sensor_radius` of it, and goes stale
again after `revisit_after_s` so the search keeps going (targets in a SAR
area can move, and new ones can appear) instead of stopping once the area
has been covered once.

Fitness at a point = how many stale in-area cells a drone there would see,
not counting cells some other drone is about to see (it is there, or it is
heading there). That last part is what keeps two drones from flying to the
same unexplored patch.
"""

import math
import random


class CoverageMap:

    def __init__(self, area_center, area_radius, cell_size=1.0,
                 sensor_radius=2.0, revisit_after_s=90.0):
        self.area_center = area_center
        self.area_radius = area_radius
        self.cell_size = cell_size
        self.sensor_radius = sensor_radius
        self.revisit_after_s = revisit_after_s
        self.last_seen = {}  # cell -> time it was last seen

        cx, cy = area_center
        reach = int(math.ceil(area_radius / cell_size)) + 1
        ci, cj = self._cell(cx, cy)
        self.area_cells = [
            (i, j)
            for i in range(ci - reach, ci + reach + 1)
            for j in range(cj - reach, cj + reach + 1)
            if math.dist(self._center((i, j)), area_center) <= area_radius]
        self._area_cell_set = set(self.area_cells)
        span = int(math.ceil(sensor_radius / cell_size))
        self._footprint = [
            (di, dj)
            for di in range(-span, span + 1)
            for dj in range(-span, span + 1)
            if math.hypot(di, dj) * cell_size <= sensor_radius]

    def _cell(self, x, y):
        return (math.floor(x / self.cell_size), math.floor(y / self.cell_size))

    def _center(self, cell):
        return ((cell[0] + 0.5) * self.cell_size,
                (cell[1] + 0.5) * self.cell_size)

    def _cells_seen_from(self, x, y):
        i, j = self._cell(x, y)
        return [(i + di, j + dj) for di, dj in self._footprint]

    def _stale(self, cell, now):
        seen = self.last_seen.get(cell)
        return seen is None or now - seen >= self.revisit_after_s

    def mark_seen(self, x, y, now):
        for cell in self._cells_seen_from(x, y):
            self.last_seen[cell] = now

    def unexplored_near(self, x, y, now, claimed=()):
        """Stale in-area cells a drone at (x, y) would see, minus any a
        drone at one of the `claimed` points would see."""
        claimed_cells = set()
        for cx, cy in claimed:
            claimed_cells.update(self._cells_seen_from(cx, cy))
        return sum(
            1 for cell in self._cells_seen_from(x, y)
            if cell in self._area_cell_set and cell not in claimed_cells
            and self._stale(cell, now))

    def fitness(self, x, y, now, claimed=()):
        overreach = max(0.0, math.dist((x, y), self.area_center)
                        - self.area_radius)
        return self.unexplored_near(x, y, now, claimed) - 2.0 * overreach

    def sample_unexplored(self, count, now):
        """Up to `count` random stale cell centers - candidate places to
        search next, scored with fitness() by the caller."""
        stale = [c for c in self.area_cells if self._stale(c, now)]
        return [self._center(c)
                for c in random.sample(stale, min(count, len(stale)))]

    def explored_fraction(self, now):
        fresh = sum(1 for c in self.area_cells if not self._stale(c, now))
        return fresh / len(self.area_cells) if self.area_cells else 1.0


def _point_segment_distance(p, a, b):
    ax, ay = a
    dx, dy = b[0] - ax, b[1] - ay
    length_sq = dx * dx + dy * dy
    if length_sq == 0:
        return math.dist(p, a)
    t = max(0.0, min(1.0, ((p[0] - ax) * dx + (p[1] - ay) * dy) / length_sq))
    return math.dist(p, (ax + t * dx, ay + t * dy))


def _segments_intersect(a, b, c, d):
    def cross(o, p, q):
        return (p[0] - o[0]) * (q[1] - o[1]) - (p[1] - o[1]) * (q[0] - o[0])
    d1, d2 = cross(c, d, a), cross(c, d, b)
    d3, d4 = cross(a, b, c), cross(a, b, d)
    return d1 * d2 < 0 and d3 * d4 < 0


def path_clearance(start, goal, other_start, other_goal):
    """Closest the straight path start->goal comes to other_start->
    other_goal (0 if they cross). Used to keep two drones from picking
    search goals whose paths run through each other: with neighbor
    positions up to a 0.5s tick stale, two drones flying at each other can
    close ~3m before either reacts, and a dry run of the coverage search
    without this check got them 0.04m apart."""
    if _segments_intersect(start, goal, other_start, other_goal):
        return 0.0
    return min(_point_segment_distance(start, other_start, other_goal),
               _point_segment_distance(goal, other_start, other_goal),
               _point_segment_distance(other_start, start, goal),
               _point_segment_distance(other_goal, start, goal))
