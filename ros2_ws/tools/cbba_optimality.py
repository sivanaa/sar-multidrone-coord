#!/usr/bin/env python3
"""How good are CBBA's assignments? Compared with the best possible one.

    python3 tools/cbba_optimality.py --drones 3 --tasks 5 --trials 300

Random static problems (drones and people placed in the search area, every
person known to every drone, perfect messages - the setting of the CBBA
paper's own optimality results): CBBA runs to consensus with the real
cbba.py, and every possible assignment and flying order is tried by brute
force. Both are scored on CBBA's own objective (sum of time-discounted
value, cbba.path_value), so "% of optimal" is how close CBBA's greedy
auction gets to what it is trying to maximise; Choi 2009 guarantees at
least 50%.

Also reported, because CBBA does not optimise it: the makespan (when the
last person is reached, at NOMINAL_SPEED_MPS), against the best makespan
any assignment allows. And a "nearest drone" baseline (each person to the
closest drone, flown nearest-first) for context.
"""

import argparse
import itertools
import math
import os
import random
import statistics
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', 'src', 'coordination_node'))
from coordination_node import cbba  # noqa: E402
from coordination_node.cbba import CbbaAgent, Task  # noqa: E402

CONSENSUS_ROUNDS = 30


def random_point(rng, center, radius):
    r = radius * math.sqrt(rng.random())
    a = rng.uniform(-math.pi, math.pi)
    return (center[0] + r * math.cos(a), center[1] + r * math.sin(a))


def run_cbba(starts, tasks):
    """CBBA to consensus (everyone hears everyone each round) on `tasks`
    (id -> Task). Returns each drone's path, and whether all drones agreed
    on every winner."""
    agents = [CbbaAgent(i) for i in range(len(starts))]
    for a in agents:
        for t in tasks.values():
            a.add_task(t)
    for _ in range(CONSENSUS_ROUNDS):
        for a, s in zip(agents, starts):
            a.build_bundle(s)
        states = [(a.drone_id, list(a.tasks),
                   [a.winning_bids[t] for t in a.tasks],
                   [a.winning_agent[t] for t in a.tasks],
                   [a.update_time[t] for t in a.tasks]) for a in agents]
        changed = False
        for a in agents:
            for st in states:
                if st[0] != a.drone_id:
                    changed |= a.receive_bundle_state(*st)
        if not changed:
            break
    for a, s in zip(agents, starts):
        a.build_bundle(s)
    agreed = all(a.winning_agent == agents[0].winning_agent for a in agents)
    return [list(a.path) for a in agents], agreed


def best_order(scorer, start, ids):
    """Best flying order of `ids` from `start` under `scorer`."""
    if not ids:
        return (), scorer((), start)
    return max(((p, scorer(p, start)) for p in itertools.permutations(ids)),
               key=lambda x: x[1])


def finish_time(tasks, path, start):
    t, here = 0.0, start
    for task_id in path:
        t += math.dist(here, tasks[task_id].position) / cbba.NOMINAL_SPEED_MPS
        here = tasks[task_id].position
    return t


def brute_force(starts, tasks, bundle_size):
    """Best total value and best makespan over every assignment (each
    drone at most `bundle_size` people, as in CBBA) and order."""
    scorer = CbbaAgent(0)
    scorer.tasks = tasks
    ids = list(tasks)
    best_value, best_makespan = -math.inf, math.inf
    for owners in itertools.product(range(len(starts)), repeat=len(ids)):
        groups = [[t for t, o in zip(ids, owners) if o == d]
                  for d in range(len(starts))]
        if any(len(g) > bundle_size for g in groups):
            continue
        value = sum(best_order(scorer.path_value, s, g)[1]
                    for s, g in zip(starts, groups))
        makespan = max(min(finish_time(tasks, p, s)
                           for p in itertools.permutations(g)) if g else 0.0
                       for s, g in zip(starts, groups))
        best_value = max(best_value, value)
        best_makespan = min(best_makespan, makespan)
    return best_value, best_makespan


def nearest_drone(starts, tasks):
    groups = [[] for _ in starts]
    for t in tasks.values():
        d = min(range(len(starts)), key=lambda i: math.dist(starts[i], t.position))
        groups[d].append(t.task_id)
    paths = []
    for s, g in zip(starts, groups):
        path, here, left = [], s, set(g)
        while left:
            nxt = min(left, key=lambda i: math.dist(here, tasks[i].position))
            path.append(nxt)
            here = tasks[nxt].position
            left.remove(nxt)
        paths.append(path)
    return paths


def main():
    p = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    p.add_argument('--drones', type=int, default=3)
    p.add_argument('--tasks', type=int, default=5)
    p.add_argument('--trials', type=int, default=200)
    p.add_argument('--seed', type=int, default=0)
    p.add_argument('--area', type=float, nargs=3, default=[4.0, 4.0, 6.0],
                   metavar=('CX', 'CY', 'R'))
    a = p.parse_args()
    center, radius = tuple(a.area[:2]), a.area[2]
    bundle_size = CbbaAgent(0).max_bundle_size
    if a.tasks > a.drones * bundle_size:
        raise SystemExit('more people than the drones can hold in bundles')

    ratio, near_ratio, span_ratio, unassigned, disagreed = [], [], [], 0, 0
    for k in range(a.trials):
        rng = random.Random(a.seed + k)
        starts = [random_point(rng, center, radius) for _ in range(a.drones)]
        tasks = {i: Task(i, random_point(rng, center, radius), 'person', 0.9)
                 for i in range(1, a.tasks + 1)}
        paths, agreed = run_cbba(starts, tasks)
        disagreed += not agreed
        scorer = CbbaAgent(0)
        scorer.tasks = tasks
        assigned = [t for path in paths for t in path]
        unassigned += len(set(tasks) - set(assigned))
        value = sum(scorer.path_value(pth, s) for pth, s in zip(paths, starts))
        best_value, best_makespan = brute_force(starts, tasks, bundle_size)
        ratio.append(value / best_value)
        near = nearest_drone(starts, tasks)
        near_ratio.append(sum(scorer.path_value(pth, s)
                              for pth, s in zip(near, starts)) / best_value)
        makespan = max(finish_time(tasks, pth, s) for pth, s in zip(paths, starts))
        span_ratio.append(makespan / best_makespan if best_makespan > 0 else 1.0)

    def pct(xs, q):
        xs = sorted(xs)
        return xs[int(q * (len(xs) - 1))]

    print(f'{a.drones} drones, {a.tasks} people, area r={radius}m, '
          f'{a.trials} problems, bundles of {bundle_size}')
    print(f'CBBA value, % of optimal:     mean {100 * statistics.mean(ratio):5.1f}  '
          f'worst {100 * min(ratio):5.1f}  p10 {100 * pct(ratio, 0.1):5.1f}  '
          f'optimal in {sum(r > 0.9999 for r in ratio)}/{a.trials}')
    print(f'nearest-drone value, % opt:   mean {100 * statistics.mean(near_ratio):5.1f}  '
          f'worst {100 * min(near_ratio):5.1f}')
    print(f'CBBA makespan / best makespan: median {statistics.median(span_ratio):.2f}  '
          f'p90 {pct(span_ratio, 0.9):.2f}  worst {max(span_ratio):.2f}')
    print(f'people left unassigned: {unassigned}   problems where drones '
          f'disagreed after consensus: {disagreed}')


if __name__ == '__main__':
    main()
