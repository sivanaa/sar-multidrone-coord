#!/usr/bin/env python3
"""Write a synthetic flood-risk map for simulation (coverage.load_risk_map
format): risk falls off with distance from a river polyline,
exp(-(d / width)^2), so it is 1 on the river and ~0 a few metres away.

    python3 tools/make_risk_map.py river.json
    python3 tools/make_risk_map.py river.json --river 9,-2 7,4 9,10 --width 2

Points are in the shared frame (x north, y east; Gazebo (x, y) = shared
(y, x)). The default river bends across the north of the default search
area (centre (4, 4), radius 6), away from both spawn points.

Stand-in until real data exists: a GIS flood-hazard layer, or the flood
segmentation model's output aggregated per cell, can be written in the same
format.
"""

import argparse
import json
import math


def point_segment_distance(p, a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]
    length_sq = dx * dx + dy * dy
    t = 0.0 if length_sq == 0 else max(0.0, min(1.0, (
        (p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / length_sq))
    return math.dist(p, (a[0] + t * dx, a[1] + t * dy))


def main():
    p = argparse.ArgumentParser()
    p.add_argument('out')
    p.add_argument('--river', nargs='+', default=['9,-2', '7,4', '9,10'],
                   help='river polyline points "x,y" in the shared frame')
    p.add_argument('--width', type=float, default=2.0,
                   help='metres from the river at which risk falls to 1/e')
    p.add_argument('--area', type=float, nargs=3, default=[4.0, 4.0, 6.0],
                   metavar=('CX', 'CY', 'R'))
    p.add_argument('--cell-size', type=float, default=1.0)
    a = p.parse_args()

    river = [tuple(float(v) for v in pt.split(',')) for pt in a.river]
    cx, cy, radius = a.area
    cs = a.cell_size
    reach = int(math.ceil((radius + 2) / cs))
    ci, cj = math.floor(cx / cs), math.floor(cy / cs)
    cells = []
    for i in range(ci - reach, ci + reach + 1):
        for j in range(cj - reach, cj + reach + 1):
            centre = ((i + 0.5) * cs, (j + 0.5) * cs)
            d = min(point_segment_distance(centre, river[k], river[k + 1])
                    for k in range(len(river) - 1))
            risk = math.exp(-(d / a.width) ** 2)
            if risk >= 0.01:
                cells.append([i, j, round(risk, 3)])
    with open(a.out, 'w') as f:
        json.dump({'cell_size': cs, 'source': 'synthetic river',
                   'river': river, 'cells': cells}, f)
    print(f'wrote {a.out}: {len(cells)} cells with risk >= 0.01')


if __name__ == '__main__':
    main()
