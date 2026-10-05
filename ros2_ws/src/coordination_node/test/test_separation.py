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
