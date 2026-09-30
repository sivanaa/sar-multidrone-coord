"""Simple direct-line navigation toward a fixed target position.

Used once a drone has been assigned a task via CBBA — moves in a straight
line toward the task's position at up to max_speed, slowing down and
holding position once within `arrival_radius` of the target rather than
overshooting and oscillating around it.

Intentionally simple: no obstacle avoidance, no path smoothing, no chaining
through multiple tasks. The point right now is closing the "wins a task but
never moves toward it" gap identified via live testing (see
ARCHITECTURE.md), not building production flight behavior.
"""

import math


def step_toward(position, target, dt, max_speed, arrival_radius=0.3,
                 real_position=None, neighbor_positions=None,
                 min_separation=1.5, separation_strength=2.5):
    """Advance one tick toward `target`. Returns
    `(tracked_x, tracked_y, commanded_x, commanded_y)`.

    `real_position`, when given, is real telemetry: `tracked_x/y` then
    reports that (truth, for the caller's own position/arrival bookkeeping —
    mirrors pso.py's `step`), while `commanded_x/y` is still the
    velocity-extrapolated point this function actually wants the vehicle to
    move to next. Without commanded and tracked being separate, there would
    be nothing to send a real flight controller — before this split (added
    2026-09-30, same gap fixed in pso.py's `step` the day before), giving
    `real_position` made this function just echo it straight back with no
    velocity applied at all. When `real_position` is None, the two pairs are
    identical (there's no real vehicle to distinguish "where it is" from
    "where it should go" from a fictitious `x += vx * dt` guess) — so this
    doesn't change any already-verified simulated-position behavior.

    `neighbor_positions`, when given, blends in a repulsion away from any
    drone closer than `min_separation`, same shape as pso.py's soft
    separation term. Without this, a fast direct approach to a task has
    zero awareness of other drones and only gets corrected *after* getting
    too close, by coordination_node.py's hard floor (separation.py) — which
    reacts once per tick and isn't fast enough against a high-speed pass.
    Confirmed live 2026-09-17: two drones measured 0.98m apart (under the
    1.5m floor) with this term absent.
    """
    x, y = real_position if real_position is not None else position
    tx, ty = target
    dx, dy = tx - x, ty - y
    distance = math.hypot(dx, dy)

    if distance <= arrival_radius:
        # Already there - hold, don't keep extrapolating past the target.
        tracked = real_position if real_position is not None else (x, y)
        return tracked[0], tracked[1], tracked[0], tracked[1]

    # Cap speed so a close-but-not-arrived target is approached smoothly in
    # the final tick rather than overshot and then oscillated around.
    speed = min(max_speed, distance / dt) if dt > 0 else max_speed
    ux, uy = dx / distance, dy / distance
    vx, vy = ux * speed, uy * speed

    for nx, ny in neighbor_positions or []:
        rx, ry = x - nx, y - ny
        rdist = math.hypot(rx, ry)
        if rdist >= min_separation or rdist == 0:
            continue
        push = separation_strength * (min_separation - rdist) / min_separation
        ux_r, uy_r = rx / rdist, ry / rdist
        # Radial (straight away) plus a tangential ("go around") component.
        # Radial alone does nothing useful when the neighbor sits directly
        # on the line to the target: the away-push and the toward-target
        # pull cancel along the same axis, leaving no sideways component to
        # actually route around it (confirmed by a dry run that still
        # passed 0.17m from a neighbor planted directly in the path with
        # only the radial term). Always deflecting the same rotational way
        # (never randomly left-or-right) makes the path curve smoothly
        # around instead of jittering between sides.
        vx += ux_r * push - uy_r * push
        vy += uy_r * push + ux_r * push

    blended_speed = math.hypot(vx, vy)
    if blended_speed > max_speed:
        scale = max_speed / blended_speed
        vx *= scale
        vy *= scale

    commanded_x = x + vx * dt
    commanded_y = y + vy * dt
    tracked_x, tracked_y = real_position if real_position is not None else (
        commanded_x, commanded_y)

    return tracked_x, tracked_y, commanded_x, commanded_y
