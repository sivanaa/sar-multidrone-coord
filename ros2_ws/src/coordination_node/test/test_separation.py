import math

from coordination_node.separation import constrain_setpoint

MIN_SEP = 1.5


def _dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def test_setpoint_clear_of_neighbors_is_unchanged():
    assert constrain_setpoint((0.0, 0.0), (-1.0, 0.0), [(5.0, 5.0)],
                              MIN_SEP) == (0.0, 0.0)


def test_setpoint_inside_floor_is_pushed_onto_circle():
    result = constrain_setpoint((0.5, 0.0), (-2.0, 0.0), [(1.0, 0.0)],
                                MIN_SEP)
    assert math.isclose(_dist(result, (1.0, 0.0)), MIN_SEP)


def test_setpoint_past_neighbor_stays_on_vehicles_side():
    # Vehicle west of the neighbor, setpoint just past it to the east: the
    # adjusted setpoint must stay west, or the vehicle would fly through.
    vehicle, neighbor = (-1.0, 0.0), (0.5, 0.0)
    result = constrain_setpoint((1.0, 0.0), vehicle, [neighbor], MIN_SEP)
    assert result[0] < neighbor[0]
    assert math.isclose(_dist(result, neighbor), MIN_SEP)


def test_vehicle_already_too_close_is_commanded_back_out():
    vehicle, neighbor = (0.0, 0.0), (0.5, 0.0)
    result = constrain_setpoint((0.2, 0.0), vehicle, [neighbor], MIN_SEP)
    assert math.isclose(_dist(result, neighbor), MIN_SEP)
    assert _dist(result, neighbor) > _dist(vehicle, neighbor)


def test_vehicle_on_top_of_neighbor_still_gets_a_valid_setpoint():
    result = constrain_setpoint((0.0, 0.0), (0.0, 0.0), [(0.0, 0.0)],
                                MIN_SEP)
    assert math.isclose(_dist(result, (0.0, 0.0)), MIN_SEP)


def test_setpoint_slides_around_neighbor_instead_of_stopping():
    # Neighbor dead ahead: the setpoint lands on the circle partway around
    # from the vehicle's side, not exactly on it, so it can make progress.
    vehicle, neighbor = (-1.8, 0.0), (0.0, 0.0)
    result = constrain_setpoint((-1.0, 0.5), vehicle, [neighbor], MIN_SEP)
    assert math.isclose(_dist(result, neighbor), MIN_SEP)
    assert result[1] > 0.0


def test_head_on_crossing_reaches_target_without_stalling():
    # Same chain coordination_node.py runs on a real vehicle each tick
    # (navigate soft term, then setpoint floor), with a crude PX4 that
    # covers 60% of the way to its setpoint per 0.5s tick. Before the
    # setpoint was allowed to slide, this stalled 1.8m short of the
    # neighbor forever.
    from coordination_node.navigate import step_toward
    position, target, neighbor = (0.0, 0.0), (10.0, 0.0), (5.0, 0.0)
    closest = math.inf
    for _ in range(120):
        _, _, cx, cy = step_toward(position, target, 0.5, 3.0,
                                   real_position=position,
                                   neighbor_positions=[neighbor],
                                   min_separation=2.5)
        sx, sy = constrain_setpoint((cx, cy), position, [neighbor], 1.8)
        dx, dy = sx - position[0], sy - position[1]
        d = math.hypot(dx, dy)
        if d > 0:
            step = min(0.6 * d, 1.5)
            position = (position[0] + dx / d * step,
                        position[1] + dy / d * step)
        closest = min(closest, _dist(position, neighbor))
        if _dist(position, target) <= 0.3:
            break
    assert _dist(position, target) <= 0.3
    assert closest >= MIN_SEP
