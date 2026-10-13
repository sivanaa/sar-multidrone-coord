#!/usr/bin/env python3
"""How far each drone's own position estimate was from the truth.

    python3 tools/compare_truth.py RUN_DIR

RUN_DIR holds mission.jsonl (mission_recorder.py: AgentState, i.e. where
each drone *believes* it is - PX4's estimate in the shared frame) and
truth.jsonl (the simulator's true positions, written by the AirSim
distance watch). The collision floor only knows the estimates, so if they
are off, two drones the nodes think are 1.8m apart can really be closer.

Prints, per drone, the estimate error (median / 95th percentile / max),
and at the moment the drones were truly closest, the true distance next
to the distance the nodes believed.
"""

import bisect
import json
import math
import statistics
import sys


def load(path):
    with open(path) as f:
        return [json.loads(line) for line in f if line.strip()]


def believed_tracks(records):
    """drone id -> (sorted times, positions) from AgentState records."""
    tracks = {}
    for r in records:
        if r.get('type') == 'agent':
            t, p = tracks.setdefault(r['drone'], ([], []))
            t.append(r['t'])
            p.append((r['x'], r['y']))
    return tracks


def at(track, t, max_gap=0.3):
    """Believed position at time t: the last broadcast at or before t, if
    it's at most `max_gap` old (AgentState is broadcast at 10Hz)."""
    times, points = track
    i = bisect.bisect_right(times, t) - 1
    if i < 0 or t - times[i] > max_gap:
        return None
    return points[i]


def main(run_dir):
    believed = believed_tracks(load(f'{run_dir}/mission.jsonl'))
    truth = load(f'{run_dir}/truth.jsonl')
    drones = sorted(believed)
    errors = {d: [] for d in drones}
    closest = None  # (true distance, believed distance, t)
    for sample in truth:
        est = {d: at(believed[d], sample['t']) for d in drones}
        true = {d: sample.get(str(d)) for d in drones}
        for d in drones:
            if est[d] and true[d]:
                errors[d].append(math.dist(est[d], true[d]))
        if len(drones) == 2 and all(est.values()) and all(true.values()):
            a, b = drones
            true_d = math.dist(true[a], true[b])
            if closest is None or true_d < closest[0]:
                closest = (true_d, math.dist(est[a], est[b]), sample['t'],
                           true, est)
    for d in drones:
        e = sorted(errors[d])
        if not e:
            print(f'drone {d}: no overlapping samples')
            continue
        p95 = e[int(0.95 * (len(e) - 1))]
        print(f'drone {d}: estimate error median {statistics.median(e):.2f}m,'
              f' 95% {p95:.2f}m, max {e[-1]:.2f}m ({len(e)} samples)')
    if closest:
        true_d, est_d, t, true, est = closest
        print(f'truly closest: {true_d:.2f}m apart, while the nodes believed'
              f' {est_d:.2f}m')
        for d in drones:
            print(f'  drone {d}: true ({true[d][0]:.2f}, {true[d][1]:.2f})'
                  f'  believed ({est[d][0]:.2f}, {est[d][1]:.2f})')


if __name__ == '__main__':
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    main(sys.argv[1])
