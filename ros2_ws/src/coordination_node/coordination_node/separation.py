"""Hard-floor collision safety.

This is the last line of defense, independent of whichever behavior produced
a candidate position — PSO's social term during SEARCH (pso.py) actively
pulls drones toward each other, and two drones can be assigned the same task
during TASK_ALLOCATION before CBBA consensus settles (navigate.py has no
awareness of other drones at all). Neither of those is a safety mechanism by
itself, so this is applied to the result of both, every tick, regardless of
state: if enforcing it moves a drone, that's a real gap in the softer
behavior upstream, not something this is meant to paper over silently.

Deliberately simple (sequential, single-pass) rather than a proper multi-body
solve — fine for the project's small fleets (prototype N=5, real deployment
target N=2, see coordination_node.py), not meant to scale to large swarms.
"""

import math


def enforce_min_separation(candidate_position, neighbor_positions, min_separation):
    """Push `candidate_position` away from any neighbor closer than
    `min_separation`, until it is exactly `min_separation` from each one it
    violated, in the order given. Returns the (possibly adjusted) position.
    """
    x, y = candidate_position
    for nx, ny in neighbor_positions:
        dx, dy = x - nx, y - ny
        distance = math.hypot(dx, dy)
        if distance >= min_separation:
            continue
        if distance == 0:
            ux, uy = 1.0, 0.0  # identical position: pick an arbitrary side
        else:
            ux, uy = dx / distance, dy / distance
        x, y = nx + ux * min_separation, ny + uy * min_separation
    return (x, y)


def constrain_setpoint(setpoint, current_position, neighbor_positions,
                       min_separation):
    """Hard floor for a real vehicle: adjust the position *setpoint* sent
    to the flight controller rather than the vehicle's position, which is
    ground truth and can't be teleported.

    A setpoint inside a neighbor's `min_separation` circle is moved onto
    that circle on the side facing `current_position` (where the vehicle
    actually is), not on whichever side the setpoint happened to land.
    enforce_min_separation's rule (push out along neighbor -> candidate) is
    fine for a teleported simulated position, but for a setpoint it can
    land on the far side of the neighbor, and the flight controller would
    then fly the vehicle straight through the neighbor to reach it.
    Routing *around* a neighbor stays the soft terms' job (pso.py,
    navigate.py); this only guarantees the commanded point is never inside
    the floor. Returns the (possibly adjusted) setpoint.
    """
    sx, sy = setpoint
    cx, cy = current_position
    for nx, ny in neighbor_positions:
        if math.hypot(sx - nx, sy - ny) >= min_separation:
            continue
        dx, dy = cx - nx, cy - ny
        distance = math.hypot(dx, dy)
        if distance == 0:
            ux, uy = 1.0, 0.0  # on top of the neighbor: pick an arbitrary side
        else:
            ux, uy = dx / distance, dy / distance
        sx, sy = nx + ux * min_separation, ny + uy * min_separation
    return (sx, sy)
