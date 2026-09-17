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
                 real_position=None):
    """Advance one tick toward `target`. Returns the new (x, y).

    `real_position`, when given, is real telemetry and is reported as the
    actual position instead of the internally-integrated one — mirrors
    pso.py's `step` so both movement modes behave consistently once real
    PX4 position is wired in.
    """
    x, y = real_position if real_position is not None else position
    tx, ty = target
    dx, dy = tx - x, ty - y
    distance = math.hypot(dx, dy)

    if distance <= arrival_radius:
        return real_position if real_position is not None else (x, y)

    # Cap speed so a close-but-not-arrived target is approached smoothly in
    # the final tick rather than overshot and then oscillated around.
    speed = min(max_speed, distance / dt) if dt > 0 else max_speed
    ux, uy = dx / distance, dy / distance
    new_x = x + ux * speed * dt
    new_y = y + uy * speed * dt

    if real_position is not None:
        return real_position
    return (new_x, new_y)
