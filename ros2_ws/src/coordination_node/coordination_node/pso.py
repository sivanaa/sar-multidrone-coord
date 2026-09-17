"""Particle Swarm Optimization for the broad-area search phase.

Each drone is one PSO particle. Position/velocity are 2D (x, y) in the local
frame (meters); altitude is held separately by the flight stack. "Swarm best"
is computed only from whichever neighbors this drone currently has data from
(see coordination_node.py), which naturally keeps this decentralized/local
rather than a true global best across the whole fleet.
"""

from dataclasses import dataclass
import math
import random


@dataclass
class PsoState:
    position: tuple
    velocity: tuple = (0.0, 0.0)
    best_position: tuple = None
    best_fitness: float = float('-inf')

    def __post_init__(self):
        if self.best_position is None:
            self.best_position = self.position


class ParticleSwarmSearch:
    """One drone's PSO particle.

    Fitness is pluggable via `fitness_fn(x, y) -> float` so the search
    objective (currently a distance-from-origin placeholder, see
    coordination_node.py) can be swapped for a flood-risk-weighted coverage
    score without touching the update rule.
    """

    def __init__(self, drone_id, initial_position, fitness_fn,
                 inertia=0.6, cognitive=1.4, social=1.4, max_speed=3.0,
                 initial_speed_fraction=0.3, min_separation=1.5,
                 separation_strength=2.5):
        self.drone_id = drone_id
        self.fitness_fn = fitness_fn
        self.inertia = inertia
        self.cognitive = cognitive
        self.social = social
        self.max_speed = max_speed
        # Soft collision-avoidance: the social term above actively pulls
        # this particle toward neighbors (that's the whole point of swarm
        # search), so without an opposing term nothing stops two drones from
        # steering onto the same point. This adds a repulsion once a
        # neighbor is closer than `min_separation`, growing the closer they
        # get. It's a soft/gradual nudge, not a guarantee — the hard floor
        # that actually guarantees no-collision lives in separation.py and
        # is applied to the resulting position regardless of what produced
        # it (PSO here, or navigate.py during TASK_ALLOCATION).
        self.min_separation = min_separation
        self.separation_strength = separation_strength

        # A zero starting velocity is a real deadlock, not just an unbiased
        # start: fitness at a particle's own starting position is always 0
        # relative to itself (distance-to-self), so with zero velocity there
        # is nothing pulling it anywhere — cognitive and social terms are
        # both zero until it has *already* moved. A small random kick is
        # standard PSO initialization, and it's what actually bootstraps
        # exploration here.
        kick = max_speed * initial_speed_fraction
        initial_velocity = (random.uniform(-kick, kick), random.uniform(-kick, kick))

        self.state = PsoState(position=initial_position, velocity=initial_velocity)
        self.state.best_fitness = fitness_fn(*initial_position)

    def _separation_velocity(self, x, y, neighbor_positions):
        """Repulsion contribution: for each neighbor closer than
        `min_separation`, push away along the line between the two,
        stronger the closer they are. Zero contribution once clear."""
        svx, svy = 0.0, 0.0
        for nx, ny in neighbor_positions:
            dx, dy = x - nx, y - ny
            distance = math.hypot(dx, dy)
            if distance >= self.min_separation or distance == 0:
                continue
            push = (self.separation_strength
                    * (self.min_separation - distance) / self.min_separation)
            svx += (dx / distance) * push
            svy += (dy / distance) * push
        return svx, svy

    def step(self, dt, swarm_best_position, neighbor_positions=None,
              real_position=None):
        """Advance one PSO update using the best known swarm position.

        `neighbor_positions`, when given, is a list of other drones' current
        (x, y) — used only for the soft separation nudge above, independent
        of `swarm_best_position` (which is about search quality, not safety).

        `real_position`, when given, is real telemetry (e.g. PX4 local
        position) and is used as this tick's actual location instead of the
        internally-integrated one — nothing currently commands the vehicle
        to follow PSO's velocity, so with `real_position` set, velocity
        keeps evolving (useful once a command loop exists) but position
        tracks truth rather than a fictitious `x += vx * dt` guess.
        """
        x, y = real_position if real_position is not None else self.state.position
        vx, vy = self.state.velocity
        pbx, pby = self.state.best_position
        gbx, gby = swarm_best_position

        r1, r2 = random.random(), random.random()
        svx, svy = self._separation_velocity(x, y, neighbor_positions or [])
        vx = (self.inertia * vx
              + self.cognitive * r1 * (pbx - x)
              + self.social * r2 * (gbx - x)
              + svx)
        vy = (self.inertia * vy
              + self.cognitive * r1 * (pby - y)
              + self.social * r2 * (gby - y)
              + svy)

        speed = (vx ** 2 + vy ** 2) ** 0.5
        if speed > self.max_speed:
            scale = self.max_speed / speed
            vx *= scale
            vy *= scale

        if real_position is not None:
            new_x, new_y = real_position
        else:
            new_x, new_y = x + vx * dt, y + vy * dt

        fitness = self.fitness_fn(new_x, new_y)
        self.state.position = (new_x, new_y)
        self.state.velocity = (vx, vy)
        if fitness > self.state.best_fitness:
            self.state.best_position = (new_x, new_y)
            self.state.best_fitness = fitness

        return self.state.position
