"""When to (re-)send the OFFBOARD mode-switch and ARM requests to PX4.

The first version sent each request exactly once and treated *sending* ARM
as being armed, so one early denial (PX4 still running its preflight checks
- `Arming denied: Resolve system health failures first`, seen on most live
runs through 2026-10-06) left that drone on the ground for the whole run
until someone typed `commander arm` in its PX4 shell. Now both requests
repeat every `retry_ticks` until PX4's own VehicleStatus reports the
vehicle armed *and* in OFFBOARD - then stop for good, so this never fights
a later PX4 failsafe (e.g. an auto-land) by re-arming or re-entering
OFFBOARD behind its back.
"""


def offboard_request(setpoint_count, confirmed, first_tick, retry_ticks,
                     arm_delay):
    """Return 'mode', 'arm' or None for this offboard tick.

    `setpoint_count` is how many setpoints have been streamed so far (PX4
    needs some already flowing before it accepts OFFBOARD). Starting at
    `first_tick`, every `retry_ticks`: 'mode' on the first tick of the
    period, 'arm' `arm_delay` ticks later. Nothing once `confirmed`.
    """
    if confirmed:
        return None
    ticks = setpoint_count - first_tick
    if ticks < 0:
        return None
    phase = ticks % retry_ticks
    if phase == 0:
        return 'mode'
    if phase == arm_delay:
        return 'arm'
    return None
