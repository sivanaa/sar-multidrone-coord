#!/usr/bin/env python3
"""Build a flood-risk map (coverage.load_risk_map format) from the flood
model's per-frame 4x4 grid output plus the drone pose at each frame
(coordination_node/flood_grid.py has the camera model and why the pose has
to be ours).

From logged observations, one JSON object per line - pose in the shared
frame (x north, y east, metres), alt above ground, yaw in radians (PX4:
0 = north, + toward east), and either the raw detect_flood response or
just its 4x4 cell_ratios:

    {"x": 3.0, "y": 4.5, "alt": 20.0, "yaw": 0.0, "response": {...}}
    {"x": 3.0, "y": 4.5, "alt": 20.0, "yaw": 0.0, "cell_ratios": [[...], ...]}

    python3 tools/flood_to_risk_map.py observations.jsonl risk.json

Without the real model (pose is mocked on the model server as of
2026-10-08), --simulate flies a lawnmower over make_risk_map.py's synthetic
river, makes the 4x4 grids the model would return there, converts them, and
prints how well the result matches the river:

    python3 tools/flood_to_risk_map.py --simulate risk.json
    python3 tools/flood_to_risk_map.py --simulate risk.json --alt 20 --area 4 4 30

The output works anywhere make_risk_map.py's does (risk_map_file param).
"""

import argparse
import json
import math
import os
import sys

try:
    from coordination_node.flood_grid import (
        GRID_SIZE, Camera, FloodRiskAccumulator, cell_ratios_from_response)
except ImportError:  # workspace not built/sourced: use the source tree
    sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                    '..', 'src', 'coordination_node'))
    from coordination_node.flood_grid import (
        GRID_SIZE, Camera, FloodRiskAccumulator, cell_ratios_from_response)

from make_risk_map import point_segment_distance


def read_observations(path):
    with open(path) as f:
        for n, line in enumerate(f, 1):
            line = line.strip()
            if not line:
                continue
            obs = json.loads(line)
            ratios = obs.get('cell_ratios')
            if ratios is None:
                ratios = cell_ratios_from_response(obs.get('response'))
            yield n, obs, ratios


def lawnmower(cx, cy, radius, spacing, step):
    """Frames along east-west lanes `spacing` apart covering the square
    around the area, alternating direction (so yaw alternates too)."""
    x = cx - radius
    eastward = True
    while x <= cx + radius + 1e-9:
        ys = [cy - radius + k * step
              for k in range(int(2 * radius / step) + 1)]
        if not eastward:
            ys.reverse()
        yaw = math.pi / 2 if eastward else -math.pi / 2
        for y in ys:
            yield (x, y), yaw
        x += spacing
        eastward = not eastward


def simulated_cell_ratios(camera, is_water, drone_xy, alt, yaw, samples=8):
    """What the model's 4x4 grid would say if its mask were perfect:
    each image cell's water fraction, from samples x samples pixels."""
    ratios = []
    cell_w = camera.width / GRID_SIZE
    cell_h = camera.height / GRID_SIZE
    for row in range(GRID_SIZE):
        out = []
        for col in range(GRID_SIZE):
            wet = total = 0
            for a in range(samples):
                for b in range(samples):
                    pixel = ((col + (b + 0.5) / samples) * cell_w,
                             (row + (a + 0.5) / samples) * cell_h)
                    ground = camera.ground_point(pixel, drone_xy, alt, yaw)
                    if ground is not None:
                        total += 1
                        wet += is_water(ground)
            out.append(wet / total if total else 0.0)
        ratios.append(out)
    return ratios


def simulate(a, camera, acc):
    river = [tuple(float(v) for v in pt.split(',')) for pt in a.river]

    def distance(p):
        return min(point_segment_distance(p, river[k], river[k + 1])
                   for k in range(len(river) - 1))

    def is_water(p):
        return distance(p) <= a.width

    cx, cy, radius = a.area
    # Lanes half a footprint apart (north-south extent when flying east-west,
    # i.e. the image's short side), so every point is seen at least twice.
    short_side = 2 * a.alt * (camera.height / 2.0) / camera.fy
    frames = 0
    for drone_xy, yaw in lawnmower(cx, cy, radius, short_side / 2, a.step):
        acc.observe(simulated_cell_ratios(camera, is_water, drone_xy, a.alt, yaw),
                    drone_xy, a.alt, yaw)
        frames += 1

    # Score inside the area only. What the search needs is water cells
    # scoring above dry ones; the absolute values are diluted whenever the
    # river is narrower than one image grid cell's footprint.
    risk = acc.risk()
    cs = acc.cell_size
    wet, dry = [], []
    both = either = 0
    for (i, j), r in risk.items():
        centre = ((i + 0.5) * cs, (j + 0.5) * cs)
        if math.dist(centre, (cx, cy)) > radius:
            continue
        water = is_water(centre)
        (wet if water else dry).append(r)
        both += water and r >= 0.5
        either += water or r >= 0.5
    cell_ground = 2 * a.alt * (camera.width / 2.0) / camera.fx / GRID_SIZE
    print(f'simulated {frames} frames at {a.alt:g}m over area centre '
          f'({cx:g}, {cy:g}) radius {radius:g}m, lanes {short_side / 2:.1f}m '
          f'apart; one image grid cell spans ~{cell_ground:.1f}m of ground '
          f'(river {2 * a.width:g}m wide)')
    if wet and dry:
        print(f'  mean risk on water cells {sum(wet) / len(wet):.2f} '
              f'({len(wet)}), on dry cells {sum(dry) / len(dry):.2f} '
              f'({len(dry)}); water overlap (risk >= 0.5 vs river) '
              f'IoU {both / either if either else 1.0:.2f}')


def main():
    p = argparse.ArgumentParser()
    p.add_argument('inputs', nargs='*',
                   help='observations.jsonl then the output risk map path; '
                        'with --simulate, just the output path')
    p.add_argument('--simulate', action='store_true')
    p.add_argument('--hfov', type=float, default=87.0)
    p.add_argument('--pitch', type=float, default=-90.0,
                   help='camera pitch in degrees, -90 = straight down')
    p.add_argument('--cell-size', type=float, default=1.0)
    sim = p.add_argument_group('--simulate')
    sim.add_argument('--alt', type=float, default=8.0)
    sim.add_argument('--step', type=float, default=1.0,
                     help='metres between frames along a lane')
    sim.add_argument('--area', type=float, nargs=3, default=[4.0, 4.0, 6.0],
                     metavar=('CX', 'CY', 'R'))
    sim.add_argument('--river', nargs='+', default=['9,-2', '7,4', '9,10'])
    sim.add_argument('--width', type=float, default=2.0,
                     help='water within this many metres of the river line')
    a = p.parse_args()

    if a.simulate and len(a.inputs) != 1 or not a.simulate and len(a.inputs) != 2:
        p.error('give observations.jsonl and an output path, or --simulate and '
                'an output path')
    camera = Camera(hfov_deg=a.hfov, pitch_deg=a.pitch)
    acc = FloodRiskAccumulator(camera, cell_size=a.cell_size)

    if a.simulate:
        simulate(a, camera, acc)
        source = f'simulated detect_flood grids over river {a.river}'
    else:
        used = skipped = 0
        for n, obs, ratios in read_observations(a.inputs[0]):
            if ratios is None or acc.observe(
                    ratios, (obs['x'], obs['y']), obs['alt'], obs['yaw']) == 0:
                skipped += 1
                continue
            used += 1
        print(f'{used} frames used, {skipped} skipped (no usable result, or '
              'nothing on the ground in view)')
        source = f'detect_flood observations {a.inputs[0]}'

    out = acc.to_risk_map(source)
    with open(a.inputs[-1], 'w') as f:
        json.dump(out, f)
    print(f'wrote {a.inputs[-1]}: {len(out["cells"])} cells with risk >= 0.01 '
          f'of {out["observed_cells"]} observed')


if __name__ == '__main__':
    main()
