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
                       min_separation, max_slide=math.radians(20)):
    """Hard floor for a real vehicle: adjust the position *setpoint* sent
    to the flight controller rather than the vehicle's position, which is
    ground truth and can't be teleported.

    A setpoint inside a neighbor's `min_separation` circle is moved onto
    that circle, at the angle (seen from the neighbor) of `current_position`
    rotated toward the setpoint's own angle by at most `max_slide`.
    enforce_min_separation's rule (push out along neighbor -> candidate) is
    fine for a teleported simulated position, but for a setpoint it can
    land on the far side of the neighbor, and the flight controller would
    then fly the vehicle straight through the neighbor to reach it — the
    `max_slide` cap is what prevents that.

    The slide itself is what lets a vehicle get *around* a neighbor sitting
    in its path. The first version (2026-10-05) put the setpoint exactly on
    the vehicle's own side, which threw away the sideways "go around"
    component navigate.py adds every tick: a dry run of a vehicle flying
    at a hovering neighbor dead ahead stalled at 1.8m forever, at every
    soft-term radius tried. With a 20 degree slide it arrives, holding ~1.7m
    at the closest point. Returns the (possibly adjusted) setpoint.
    """
    sx, sy = setpoint
    cx, cy = current_position
    for nx, ny in neighbor_positions:
        if math.hypot(sx - nx, sy - ny) >= min_separation:
            continue
        if (cx, cy) == (nx, ny):
            vehicle_angle = 0.0  # on top of the neighbor: pick an arbitrary side
        else:
            vehicle_angle = math.atan2(cy - ny, cx - nx)
        if (sx, sy) == (nx, ny):
            angle = vehicle_angle
        else:
            offset = math.atan2(sy - ny, sx - nx) - vehicle_angle
            offset = (offset + math.pi) % (2 * math.pi) - math.pi
            angle = vehicle_angle + max(-max_slide, min(max_slide, offset))
        sx = nx + min_separation * math.cos(angle)
        sy = ny + min_separation * math.sin(angle)
    return (sx, sy)
