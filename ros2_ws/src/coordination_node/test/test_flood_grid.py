import math

from coordination_node.flood_grid import (
    Camera, FloodRiskAccumulator, cell_ratios_from_response)


def test_nadir_centre_of_image_is_straight_below():
    cam = Camera()
    u, v = cam.project((3.0, 4.0), (3.0, 4.0), 20.0, 0.7)
    assert math.isclose(u, cam.cx) and math.isclose(v, cam.cy)


def test_heading_north_top_of_image_is_north_and_right_is_east():
    # At 20m the frame reaches ~10.7m north/south and ~19m east/west.
    cam = Camera()
    assert cam.grid_cell(cam.project((8.0, 0.0), (0.0, 0.0), 20.0, 0.0))[0] == 0
    assert cam.grid_cell(cam.project((0.0, 15.0), (0.0, 0.0), 20.0, 0.0))[1] == 3
    assert cam.grid_cell(cam.project((-8.0, -15.0), (0.0, 0.0), 20.0, 0.0)) == (3, 0)


def test_heading_east_top_of_image_is_east():
    cam = Camera()
    yaw = math.pi / 2
    assert cam.grid_cell(cam.project((0.0, 8.0), (0.0, 0.0), 20.0, yaw))[0] == 0
    # Facing east, the drone's right (image right) is south.
    assert cam.grid_cell(cam.project((-15.0, 0.0), (0.0, 0.0), 20.0, yaw))[1] == 3


def test_footprint_matches_the_field_of_view():
    # 87 deg HFOV at 10m: ground half-width 10 * tan(43.5 deg) = 9.49m.
    cam = Camera()
    half = 10.0 * math.tan(math.radians(43.5))
    assert cam.grid_cell(cam.project((0.0, half - 0.1), (0.0, 0.0), 10.0, 0.0))
    assert cam.grid_cell(cam.project((0.0, half + 0.1), (0.0, 0.0), 10.0, 0.0)) is None


def test_forward_tilted_camera_sees_ahead_not_behind():
    cam = Camera(pitch_deg=-45.0)
    assert cam.project((-20.0, 0.0), (0.0, 0.0), 10.0, 0.0) is None
    assert cam.grid_cell(cam.project((10.0, 0.0), (0.0, 0.0), 10.0, 0.0)) is not None


def test_ground_point_inverts_project():
    for pitch, yaw in ((-90.0, 0.0), (-90.0, 2.1), (-60.0, -0.8)):
        cam = Camera(pitch_deg=pitch)
        for pixel in ((100.0, 50.0), (960.0, 540.0), (1800.0, 1000.0)):
            ground = cam.ground_point(pixel, (2.0, -3.0), 15.0, yaw)
            u, v = cam.project(ground, (2.0, -3.0), 15.0, yaw)
            assert math.isclose(u, pixel[0], abs_tol=1e-6)
            assert math.isclose(v, pixel[1], abs_tol=1e-6)


def test_one_frame_maps_its_wet_quadrant_to_the_right_ground():
    acc = FloodRiskAccumulator()
    ratios = [[0.0] * 4 for _ in range(4)]
    ratios[0][3] = 1.0  # top-right of the image: north-east when facing north
    assert acc.observe(ratios, (0.0, 0.0), 10.0, 0.0) > 0
    risk = acc.risk()
    assert risk[(4, 6)] == 1.0      # north-east
    assert risk[(-4, -6)] == 0.0    # south-west
    assert (40, 40) not in risk     # never seen


def test_observations_average():
    acc = FloodRiskAccumulator()
    wet = [[1.0] * 4 for _ in range(4)]
    dry = [[0.0] * 4 for _ in range(4)]
    acc.observe(wet, (0.0, 0.0), 10.0, 0.0)
    acc.observe(dry, (0.0, 0.0), 10.0, 0.0)
    assert acc.risk()[(0, 0)] == 0.5


def test_risk_map_drops_dry_cells_and_loads_back():
    acc = FloodRiskAccumulator()
    ratios = [[0.0] * 4 for _ in range(4)]
    ratios[3][0] = 0.8
    acc.observe(ratios, (0.0, 0.0), 10.0, 0.0)
    out = acc.to_risk_map()
    assert out['cells'] and all(r >= 0.01 for _, _, r in out['cells'])
    assert out['observed_cells'] > len(out['cells'])


def test_response_without_grid():
    assert cell_ratios_from_response({'error': 'camera'}) is None
    assert cell_ratios_from_response({'skipped': True}) is None
    dry = cell_ratios_from_response({'classification': {'label': 'Non-Flooded'}})
    assert dry == [[0.0] * 4] * 4
    weak = cell_ratios_from_response({
        'classification': {'label': 'Flooded'},
        'segmentation': {'flood_ratio': 0.2}, 'grid': None})
    assert weak == [[0.2] * 4] * 4
    grid = cell_ratios_from_response({'grid': {'cell_ratios': [[0.5] * 4] * 4}})
    assert grid[2][1] == 0.5
