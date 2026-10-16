#!/usr/bin/env python3
"""Fast fleet simulator: N real DroneBrains, simulated vehicles and radio.

    python3 tools/fleet_sim.py --drones 3 --targets 4 --trials 50
    python3 tools/fleet_sim.py --drones 4 --loss 0.3 --trials 100 --json out.json

Runs the same coordination logic the ROS nodes fly (coordination_node/
brain.py: PSO search over the coverage map, CBBA bidding and consensus,
collision floor) without ROS, PX4 or a renderer, so hundreds of randomized
missions take minutes instead of the ~3 minutes one live run costs. Built
to tune and compare the algorithms (ARCHITECTURE.md "Next steps" 8); live
runs confirm what it finds.

What it simulates, each at the rate the real system runs it:
- brains exactly as coordination_node.py drives them with a real vehicle
  (`real_vehicle=True`): tick() every 0.5s, setpoint() and an AgentState
  broadcast every 0.1s;
- vehicles: velocity = KP * (setpoint - position), capped at VMAX and
  AMAX - tuned to what Gazebo/AirSim measured (setpoint tracking gain
  ~0.83-1.0, accel p95 1.3-2.9 m/s2, cruise ~1 m/s at these setpoints) -
  and the brain sees its own and its neighbours' positions POSITION_LAG_S
  late (AirSim after the GPS fix: ~0.1s);
- radio: every message (AgentState, TargetDetected, BundleState) is lost
  with probability --loss, delivered --latency (+- jitter) later, and only
  if sender and receiver are within --range metres;
- detection: a drone notices a target within --detect-radius metres of it,
  and reports it - off by DETECT_NOISE_M (sigma), as the camera is -
  unless a target it already knows of is within 1m (the
  vision bridge's dedupe rule). People are placed from 1.2m apart, so
  close pairs (test-bids has some 1.4m apart) are exercised.

Metrics per mission: time until every target was reported ("found") and
reached, distance flown, closest true approach between drones, coverage of
the search area over time, messages sent, and duplicate tasks (one person
reported as two).
"""

import argparse
import heapq
import json
import math
import os
import random
import statistics
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', 'src', 'coordination_node'))
from coordination_node.brain import (  # noqa: E402
    STATE_RETURNING, TICK_S, DroneBrain)
from coordination_node.coverage import (  # noqa: E402
    CoverageMap, camera_footprint)

DT = 0.05  # vehicle physics step
SETPOINT_EVERY = 2  # physics steps per 10Hz setpoint/broadcast
TICK_EVERY = int(round(TICK_S / DT))  # physics steps per brain tick
KP = 0.9
AMAX = 2.0
VMAX = 5.0
POSITION_LAG_S = 0.1
DEDUPE_M = 1.0
DETECT_NOISE_M = 0.15  # vision bridge reports were 0.06-0.39m off


class Radio:
    """Message bus between drones: loss, latency, range."""

    def __init__(self, rng, loss, latency, jitter, range_m):
        self.rng, self.loss = rng, loss
        self.latency, self.jitter, self.range_m = latency, jitter, range_m
        self.queue = []  # (deliver_at, seq, recipient, kind, payload)
        self.seq = 0
        self.sent = {'agent': 0, 'target': 0, 'bundle': 0}

    def send(self, now, sender, recipients, kind, payload, positions):
        self.sent[kind] += 1
        for r in recipients:
            if r == sender or self.rng.random() < self.loss:
                continue
            if self.range_m and math.dist(positions[sender],
                                          positions[r]) > self.range_m:
                continue
            delay = max(0.0, self.latency + self.rng.uniform(-self.jitter,
                                                             self.jitter))
            self.seq += 1
            heapq.heappush(self.queue, (now + delay, self.seq, r, kind, payload))

    def due(self, now):
        while self.queue and self.queue[0][0] <= now:
            yield heapq.heappop(self.queue)[2:]


class Outbox:
    """What a DroneBrain publishes goes onto the radio."""

    def __init__(self, sim, drone_id):
        self.sim, self.id = sim, drone_id

    def _send(self, kind, payload):
        self.sim.radio.send(self.sim.t, self.id, range(len(self.sim.brains)),
                            kind, payload, self.sim.true_pos)

    def publish_agent_state(self, s):
        self._send('agent', s)

    def publish_target(self, t):
        self._send('target', t)

    def publish_bundle(self, b):
        self._send('bundle', b)

    def request_landing(self, stay_up=False):
        self.sim.landed[self.id] = True

    def log(self, level, text):
        if self.sim.verbose:
            print(f'{self.sim.t:6.1f} {text}')


def spawn_points(n, spacing=5.0):
    """Drone 0 at the origin, the rest on a grid (sim.sh's 2-drone spawns
    are (0,0) and (5,5): this keeps them for n=2)."""
    if n == 2:
        return [(0.0, 0.0), (spacing, spacing)]
    side = math.ceil(math.sqrt(n))
    return [((k // side) * spacing, (k % side) * spacing) for k in range(n)]


class Mission:

    def __init__(self, seed, drones, targets, area_center, area_radius,
                 loss=0.0, latency=0.02, jitter=0.01, range_m=0.0,
                 detect_radius=1.5, max_time=300.0, verbose=False,
                 sensor_radius=1.5, revisit_after_s=60.0, camera_alt=None,
                 brain_footprint=False):
        """`camera_alt`: the drones' sensor is the nadir camera's ground
        rectangle at this altitude (coverage.camera_footprint) - for
        detection and the coverage metric; None = a detect_radius circle.
        `brain_footprint`: the brains' coverage maps use that rectangle
        too (else their sensor_radius circle)."""
        self.rng = random.Random(seed)
        random.seed(seed)  # pso.py draws from the module-level generator
        self.verbose = verbose
        self.t = 0.0
        self.max_time = max_time
        self.detect_radius = detect_radius
        self.footprint = (camera_footprint(camera_alt)
                          if camera_alt is not None else None)
        self.radio = Radio(self.rng, loss, latency, jitter, range_m)
        spawns = spawn_points(drones)
        self.true_pos = [list(s) for s in spawns]
        self.vel = [[0.0, 0.0] for _ in spawns]
        self.history = [[tuple(s)] for s in spawns]  # for the position lag
        self.sp = [tuple(s) for s in spawns]
        self.landed = [False] * drones
        self.flown = [0.0] * drones
        self.brains = [
            DroneBrain(i, s, area_center, area_radius, clock=lambda: self.t,
                       outbox=Outbox(self, i), sensor_radius=sensor_radius,
                       revisit_after_s=revisit_after_s, real_vehicle=True,
                       footprint=self.footprint if brain_footprint else None)
            for i, s in enumerate(spawns)]
        self.targets = self._place_targets(targets, area_center, area_radius,
                                           spawns)
        self.found_at = [None] * len(self.targets)
        self.reached_at = [None] * len(self.targets)
        self.task_target = {}  # task id -> index into targets
        self.duplicates = 0
        self.completed_ids = {}  # target index -> task ids completed for it
        self.owner = {}  # task id -> drone whose bundle last held it
        self.handovers = 0
        self.min_sep = math.inf
        # Ground truth for the coverage metric: a cell is searched once all
        # of it was inside a drone's sensor (the same model detection uses)
        # - independent of whatever rule the brains are set to use. Marked
        # at the 10Hz broadcast rate, so a moving drone leaves no gaps.
        cells = CoverageMap(area_center, area_radius).area_cells
        self.truth_cells = {c: None for c in cells}  # cell -> first seen
        self.coverage_at = {}  # fraction -> first time reached

    def _senses(self, pos, p, margin=0.0):
        """Is point p inside the sensor of a drone at pos (shrunk by
        `margin` on every side)?"""
        if self.footprint is not None:
            return (abs(p[0] - pos[0]) <= self.footprint[0] - margin
                    and abs(p[1] - pos[1]) <= self.footprint[1] - margin)
        return math.dist(p, pos) <= self.detect_radius - margin * math.sqrt(2)

    def _place_targets(self, n, center, radius, spawns):
        out = []
        while len(out) < n:
            r = radius * math.sqrt(self.rng.random())
            a = self.rng.uniform(-math.pi, math.pi)
            p = (center[0] + r * math.cos(a), center[1] + r * math.sin(a))
            if all(math.dist(p, s) > 2.0 for s in spawns) and all(
                    math.dist(p, q) >= 1.2 for q in out):
                out.append(p)
        return out

    def _mark_truth(self, p):
        reach = max(self.footprint) if self.footprint else self.detect_radius
        span = int(math.ceil(reach)) + 1
        ci, cj = math.floor(p[0]), math.floor(p[1])
        for i in range(ci - span, ci + span + 1):
            for j in range(cj - span, cj + span + 1):
                if (self.truth_cells.get((i, j), 0) is None
                        and self._senses(p, (i + 0.5, j + 0.5), margin=0.5)):
                    self.truth_cells[(i, j)] = self.t

    def _seen_pos(self, i):
        lag = int(round(POSITION_LAG_S / DT))
        h = self.history[i]
        return h[-1 - lag] if len(h) > lag else h[0]

    def _deliver(self):
        for recipient, kind, m in self.radio.due(self.t):
            b = self.brains[recipient]
            if kind == 'agent':
                b.on_agent_state(m['drone_id'], m['stamp'], m['position'],
                                 m['best_position'], m['best_fitness'])
            elif kind == 'target':
                b.on_target_detected(m['target_id'], m['position'],
                                     m['target_type'], m['confidence'])
            else:
                b.on_bundle_state(m['drone_id'], m['known_task_ids'],
                                  m['winning_bids'], m['winning_agent_ids'],
                                  m['update_times'], m['completed_task_ids'])

    def _detect(self, i):
        b = self.brains[i]
        if b.state == STATE_RETURNING or self.landed[i]:
            return
        for k, p in enumerate(self.targets):
            if not self._senses(self.true_pos[i], p):
                continue
            if any(math.dist(task.position, p) <= DEDUPE_M
                   for task in b.cbba.tasks.values()):
                continue
            seen = (p[0] + self.rng.gauss(0.0, DETECT_NOISE_M),
                    p[1] + self.rng.gauss(0.0, DETECT_NOISE_M))
            task_id = b.report_target_detected(seen, 'person', 0.9)
            if self.found_at[k] is None:
                self.found_at[k] = self.t
            else:
                self.duplicates += 1
            self.task_target[task_id] = k

    def _record_completions(self):
        for b in self.brains:
            for task_id in b.cbba.completed_task_ids:
                k = self.task_target.get(task_id)
                if k is None:
                    continue
                self.completed_ids.setdefault(k, set()).add(task_id)
                if self.reached_at[k] is None:
                    self.reached_at[k] = self.t
            # Handovers: a task entering a different drone's bundle than
            # the one that last held it - flip-flopping between drones.
            for task_id in b.cbba.bundle:
                last = self.owner.get(task_id)
                if last is not None and last != b.drone_id:
                    self.handovers += 1
                self.owner[task_id] = b.drone_id

    def _physics(self, i):
        if self.landed[i]:
            self.vel[i] = [0.0, 0.0]
            return
        x, y = self.true_pos[i]
        vx_d, vy_d = KP * (self.sp[i][0] - x), KP * (self.sp[i][1] - y)
        s = math.hypot(vx_d, vy_d)
        if s > VMAX:
            vx_d, vy_d = vx_d * VMAX / s, vy_d * VMAX / s
        ax, ay = (vx_d - self.vel[i][0]) / DT, (vy_d - self.vel[i][1]) / DT
        a = math.hypot(ax, ay)
        if a > AMAX:
            ax, ay = ax * AMAX / a, ay * AMAX / a
        self.vel[i][0] += ax * DT
        self.vel[i][1] += ay * DT
        nx, ny = x + self.vel[i][0] * DT, y + self.vel[i][1] * DT
        self.flown[i] += math.hypot(nx - x, ny - y)
        self.true_pos[i] = [nx, ny]
        self.history[i].append((nx, ny))
        if len(self.history[i]) > 50:
            self.history[i].pop(0)

    def run(self):
        steps = int(self.max_time / DT)
        for step in range(steps):
            self.t = step * DT
            self._deliver()
            for i, b in enumerate(self.brains):
                b.real_position = self._seen_pos(i)
            if step % TICK_EVERY == 0:
                for i, b in enumerate(self.brains):
                    self._detect(i)
                    b.tick(TICK_S)
                self._record_completions()
                frac = sum(v is not None for v in self.truth_cells.values()) / len(
                    self.truth_cells)
                for level in (0.5, 0.9, 0.99):
                    if frac >= level and level not in self.coverage_at:
                        self.coverage_at[level] = self.t
            if step % SETPOINT_EVERY == 0:
                for p in self.true_pos:
                    self._mark_truth(p)
                for i, b in enumerate(self.brains):
                    if not self.landed[i]:
                        self.sp[i] = b.setpoint()
                        b.publish_agent_state()
            for i in range(len(self.brains)):
                self._physics(i)
            n = len(self.true_pos)
            for i in range(n):
                for j in range(i + 1, n):
                    if not (self.landed[i] or self.landed[j]):
                        self.min_sep = min(self.min_sep, math.dist(
                            self.true_pos[i], self.true_pos[j]))
            if all(r is not None for r in self.reached_at):
                break
        return self.result()

    def result(self):
        done = all(r is not None for r in self.reached_at)
        return {
            'all_found_s': max(self.found_at) if all(
                f is not None for f in self.found_at) else None,
            'all_reached_s': max(self.reached_at) if done else None,
            'reached': sum(r is not None for r in self.reached_at),
            'targets': len(self.targets),
            'flown_m': sum(self.flown),
            'min_sep_m': self.min_sep,
            'coverage_90_s': self.coverage_at.get(0.9),
            'messages': dict(self.radio.sent),
            'duplicates': self.duplicates,
            'wasted_trips': sum(len(s) - 1 for s in self.completed_ids.values()),
            'handovers': self.handovers,
            'end_s': self.t,
        }


def apply_override(item):
    """--set MOD.NAME=VALUE: VALUE is read as JSON (numbers, true/false),
    or Python's True/False; the constant must already exist."""
    import importlib
    target, _, value = item.partition('=')
    mod_name, _, name = target.partition('.')
    mod = importlib.import_module(f'coordination_node.{mod_name}')
    if not hasattr(mod, name):
        raise SystemExit(f'--set: coordination_node.{mod_name} has no {name}')
    value = {'True': 'true', 'False': 'false'}.get(value, value)
    setattr(mod, name, json.loads(value))


def summarize(results):
    def stat(key):
        vals = [r[key] for r in results if r[key] is not None]
        if not vals:
            return 'n/a'
        vals.sort()
        return (f'median {statistics.median(vals):6.1f}  '
                f'p90 {vals[int(0.9 * (len(vals) - 1))]:6.1f}  '
                f'({len(vals)}/{len(results)})')
    n = len(results)
    complete = sum(r['all_reached_s'] is not None for r in results)
    lines = [
        f'missions complete (every target reached): {complete}/{n}',
        f'time all found   (s): {stat("all_found_s")}',
        f'time all reached (s): {stat("all_reached_s")}',
        f'coverage 90%     (s): {stat("coverage_90_s")}',
        f'distance flown   (m): {stat("flown_m")}',
        f'closest approach (m): min {min(r["min_sep_m"] for r in results):.2f}  '
        f'median {statistics.median(r["min_sep_m"] for r in results):.2f}',
        f'duplicate tasks: {sum(r["duplicates"] for r in results)} in '
        f'{sum(r["duplicates"] > 0 for r in results)} missions',
        f'wasted trips (a person reached twice): '
        f'{sum(r["wasted_trips"] for r in results)}',
        'handovers per mission: median '
        f'{statistics.median(r["handovers"] for r in results):.0f}, max '
        f'{max(r["handovers"] for r in results)}',
        'bundle messages per mission: median '
        f'{statistics.median(r["messages"]["bundle"] for r in results):.0f}',
    ]
    return '\n'.join(lines)


def main():
    p = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    p.add_argument('--drones', type=int, default=2)
    p.add_argument('--targets', type=int, default=3)
    p.add_argument('--trials', type=int, default=20)
    p.add_argument('--seed', type=int, default=0)
    p.add_argument('--area', type=float, nargs=3, default=[4.0, 4.0, 6.0],
                   metavar=('CX', 'CY', 'R'))
    p.add_argument('--loss', type=float, default=0.0,
                   help='probability each message is lost')
    p.add_argument('--latency', type=float, default=0.02, help='seconds')
    p.add_argument('--jitter', type=float, default=0.01, help='seconds')
    p.add_argument('--range', type=float, default=0.0,
                   help='radio range in metres (0 = unlimited)')
    p.add_argument('--detect-radius', type=float, default=1.5,
                   help='circle sensor radius, metres (without --camera-alt)')
    p.add_argument('--camera-alt', type=float, default=3.0,
                   help='sensor = nadir camera rectangle at this altitude (m), '
                        'sim.sh flies at 3; 0 = use --detect-radius circle')
    p.add_argument('--brain-footprint', action='store_true',
                   help='brains count coverage with that camera rectangle too')
    p.add_argument('--max-time', type=float, default=300.0)
    p.add_argument('--verbose', action='store_true',
                   help='print every brain log line (use with --trials 1)')
    p.add_argument('--json', help='write every mission result here')
    p.add_argument('--set', action='append', default=[], metavar='MOD.NAME=VALUE',
                   help='override an algorithm constant for this run, e.g. '
                        'coverage.STRICT_FOOTPRINT=False or cbba.NOMINAL_SPEED_MPS=1.0 '
                        '(modules: brain, cbba, coverage, navigate, pso, separation)')
    a = p.parse_args()
    for item in a.set:
        apply_override(item)

    results = []
    for k in range(a.trials):
        m = Mission(a.seed + k, a.drones, a.targets, tuple(a.area[:2]),
                    a.area[2], loss=a.loss, latency=a.latency,
                    jitter=a.jitter, range_m=a.range,
                    detect_radius=a.detect_radius, max_time=a.max_time,
                    camera_alt=a.camera_alt or None,
                    brain_footprint=a.brain_footprint,
                    verbose=a.verbose)
        results.append(m.run())
    print(f'{a.drones} drones, {a.targets} targets, area r={a.area[2]}m, '
          f'loss {a.loss:.0%}, latency {a.latency * 1000:.0f}ms, '
          f'range {a.range or "unlimited"}, {a.trials} missions')
    print(summarize(results))
    if a.json:
        with open(a.json, 'w') as f:
            json.dump({'args': vars(a), 'results': results}, f, indent=1)


if __name__ == '__main__':
    main()
