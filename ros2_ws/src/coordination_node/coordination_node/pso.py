"""Particle Swarm Optimization for the broad-area search phase.

Each drone is one PSO particle. Position/velocity are 2D (x, y) in the local
frame (meters); altitude is held separately by the flight stack. "Swarm best"
is computed only from whichever neighbors this drone currently has data from
(see coordination_node.py), which naturally keeps this decentralized/local
rather than a true global best across the whole fleet.
"""

from dataclasses import dataclass
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
                 initial_speed_fraction=0.3):
        self.drone_id = drone_id
        self.fitness_fn = fitness_fn
        self.inertia = inertia
        self.cognitive = cognitive
        self.social = social
        self.max_speed = max_speed

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

    def step(self, dt, swarm_best_position):
        """Advance one PSO update using the best known swarm position."""
        x, y = self.state.position
        vx, vy = self.state.velocity
        pbx, pby = self.state.best_position
        gbx, gby = swarm_best_position

        r1, r2 = random.random(), random.random()
        vx = (self.inertia * vx
              + self.cognitive * r1 * (pbx - x)
              + self.social * r2 * (gbx - x))
        vy = (self.inertia * vy
              + self.cognitive * r1 * (pby - y)
              + self.social * r2 * (gby - y))

        speed = (vx ** 2 + vy ** 2) ** 0.5
        if speed > self.max_speed:
            scale = self.max_speed / speed
            vx *= scale
            vy *= scale

        x += vx * dt
        y += vy * dt

        fitness = self.fitness_fn(x, y)
        self.state.position = (x, y)
        self.state.velocity = (vx, vy)
        if fitness > self.state.best_fitness:
            self.state.best_position = (x, y)
            self.state.best_fitness = fitness

        return self.state.position
