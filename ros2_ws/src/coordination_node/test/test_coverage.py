import math
import random

from coordination_node.coverage import CoverageMap, path_clearance
from coordination_node.pso import ParticleSwarmSearch


def _map():
    return CoverageMap((0.0, 0.0), 5.0, sensor_radius=1.5, revisit_after_s=60.0)


def test_seen_ground_is_worth_less_until_it_goes_stale():
    coverage = _map()
    fresh = coverage.fitness(0.0, 0.0, now=0.0)
    coverage.mark_seen(0.0, 0.0, now=0.0)
    assert coverage.fitness(0.0, 0.0, now=1.0) == 0
    assert coverage.fitness(0.0, 0.0, now=60.0) == fresh


def test_ground_another_drone_covers_does_not_count():
    coverage = _map()
    assert coverage.fitness(2.0, 0.0, now=0.0, claimed=[(2.0, 0.0)]) == 0
    assert coverage.fitness(2.0, 0.0, now=0.0, claimed=[(-3.0, 0.0)]) > 0


def test_outside_the_area_scores_below_inside():
    coverage = _map()
    assert coverage.fitness(9.0, 0.0, now=0.0) < 0 < coverage.fitness(
        0.0, 0.0, now=0.0)


def test_unexplored_samples_skip_seen_ground():
    random.seed(0)
    coverage = _map()
    coverage.mark_seen(0.0, 0.0, now=0.0)
    for point in coverage.sample_unexplored(50, now=1.0):
        assert coverage.fitness(*point, now=1.0) > 0


def test_path_clearance():
    # Crossing paths: zero. Parallel paths 3m apart: 3m.
    assert path_clearance((0, 0), (4, 4), (0, 4), (4, 0)) == 0.0
    assert math.isclose(path_clearance((0, 0), (4, 0), (0, 3), (4, 3)), 3.0)


def test_personal_best_is_rescored_so_a_drone_leaves_searched_ground():
    # Before 2026-10-06 the personal best kept the score it had when first
    # visited, so a drone's spawn stayed its "best" and pulled it back.
    random.seed(0)
    coverage = _map()
    now = [0.0]
    pso = ParticleSwarmSearch(
        0, (0.0, 0.0), lambda x, y, n: coverage.fitness(x, y, now[0]))
    for tick in range(60):
        now[0] = tick * 0.5
        coverage.mark_seen(*pso.state.position, now[0])
        pso.step(0.5, pso.state.best_position,
                 candidates=coverage.sample_unexplored(8, now[0]))
    assert math.dist(pso.state.position, (0.0, 0.0)) > 1.5
    assert coverage.explored_fraction(now[0]) > 0.5


def test_flood_risk_makes_ground_worth_more_and_go_stale_sooner():
    risky_cell = (2, 0)
    coverage = CoverageMap((0.0, 0.0), 5.0, sensor_radius=0.5,
                           revisit_after_s=60.0, risk={risky_cell: 1.0})
    assert coverage.fitness(2.5, 0.5, now=0.0) == 4.0    # 1 + 3 * risk
    assert coverage.fitness(-2.5, 0.5, now=0.0) == 1.0   # no risk: as before
    coverage.mark_seen(2.5, 0.5, now=0.0)
    coverage.mark_seen(-2.5, 0.5, now=0.0)
    assert coverage.fitness(2.5, 0.5, now=30.0) == 4.0   # stale again at 30s
    assert coverage.fitness(-2.5, 0.5, now=30.0) == 0    # still fresh


def test_risky_ground_is_offered_as_a_goal_more_often():
    random.seed(1)
    risky = {(i, j): 1.0 for i in range(0, 5) for j in range(-1, 1)}
    coverage = CoverageMap((0.0, 0.0), 5.0, sensor_radius=1.5, risk=risky)
    picks = [p for _ in range(200) for p in coverage.sample_unexplored(1, 0.0)]
    share = sum((math.floor(x), math.floor(y)) in risky for x, y in picks) / len(picks)
    area_share = len([c for c in coverage.area_cells if c in risky]) / len(coverage.area_cells)
    assert share > 2 * area_share


def test_risk_map_file_round_trip(tmp_path):
    from coordination_node.coverage import load_risk_map
    path = tmp_path / 'risk.json'
    path.write_text('{"cell_size": 1.0, "cells": [[1, 2, 0.75], [3, 4, 1.7]]}')
    cell_size, risk = load_risk_map(str(path))
    assert cell_size == 1.0
    assert risk == {(1, 2): 0.75, (3, 4): 1.0}   # clamped to 0-1
