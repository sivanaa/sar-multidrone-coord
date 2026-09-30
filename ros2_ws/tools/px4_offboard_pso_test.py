#!/usr/bin/env python3
"""PX4 offboard control driven by real PSO search output — one drone, real
telemetry closing the loop, still no CBBA or second drone involved.

Builds directly on px4_offboard_smoke_test.py's already-verified arm/OFFBOARD/
climb/land sequence (subclassed from that file, not duplicated) and overrides
only the horizontal setpoint: instead of holding a fixed (0, 0), it steps a
real ParticleSwarmSearch particle every tick using the vehicle's ACTUAL
telemetry position as ground truth (pso.py's `real_position` argument, and
its commanded-vs-tracked-position split added 2026-09-30 specifically for
this), and commands wherever PSO's velocity says to go next.

Uses the same make_coverage_fitness placeholder coordination_node.py does,
but with an empty neighbor list (one drone, nobody to spread out from) — this
exercises the "no neighbor data" fallback path (distance-from-area-center)
fixed 2026-09-29, not the neighbor-spread path, since there's no second drone
here to spread out from. That's fine for this isolated single-drone test; the
neighbor-spread path only matters once two real drones are flying
simultaneously (multi-drone PX4 topic namespacing itself was resolved
2026-09-30 — see ARCHITECTURE.md's "PX4 telemetry" section — but this script
still needs to actually be run twice, once per instance, to exercise it).

Run the same way as px4_offboard_smoke_test.py — PX4 SITL instance(s) + Agent
already bridged, NAV_DLL_ACT set to 0 for this session (see ARCHITECTURE.md's
PX4 offboard section). Pass --instance to target a PX4 instance other than 0
(its own -i N flag), and --initial-x/--initial-y to match wherever that
instance actually spawned (used as both PSO's bootstrap position and its
area-coverage center, so "spread out" is relative to this drone's own start,
not instance 0's):

    python3 tools/px4_offboard_pso_test.py --hold-altitude 3.0 --duration 30 --area-radius 5.0
    python3 tools/px4_offboard_pso_test.py --instance 1 --initial-x 5.0 --initial-y 5.0 --hold-altitude 3.0 --duration 30 --area-radius 5.0
"""

import argparse
import signal

import rclpy

from coordination_node.pso import ParticleSwarmSearch
from coordination_node.coordination_node import make_coverage_fitness

from px4_offboard_smoke_test import Px4OffboardSmokeTest, SETPOINT_HZ


class Px4OffboardPsoTest(Px4OffboardSmokeTest):
    def __init__(self, hold_altitude, duration, area_radius,
                 instance=0, initial_position=(0.0, 0.0)):
        super().__init__(hold_altitude, duration, instance)
        self.pso = ParticleSwarmSearch(
            drone_id=instance, initial_position=initial_position,
            fitness_fn=make_coverage_fitness(initial_position, area_radius))
        self.get_logger().info(
            f'PSO-driven search active once airborne (area_radius={area_radius}m)')

    def _next_xy(self):
        if self._local_position is None:
            # No telemetry yet (e.g. still in the pre-arm setpoint warmup) -
            # hold at the origin rather than stepping PSO against unknown
            # ground truth.
            return 0.0, 0.0
        real_xy = (self._local_position[0], self._local_position[1])
        commanded_x, commanded_y = self.pso.step(
            dt=1.0 / SETPOINT_HZ,
            swarm_best_position=self.pso.state.best_position,
            neighbor_positions=[],
            real_position=real_xy)
        return commanded_x, commanded_y


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--hold-altitude', type=float, default=3.0)
    parser.add_argument('--duration', type=float, default=30.0)
    parser.add_argument(
        '--area-radius', type=float, default=5.0,
        help='Soft bound for the coverage fitness (see make_coverage_fitness) '
             '- how far from this drone\'s own start it will roam before the '
             "overreach penalty starts pulling it back.")
    parser.add_argument(
        '--instance', type=int, default=0,
        help='PX4 instance number this targets (its own -i N flag).')
    parser.add_argument('--initial-x', type=float, default=0.0)
    parser.add_argument('--initial-y', type=float, default=0.0)
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = Px4OffboardPsoTest(
        args.hold_altitude, args.duration, args.area_radius,
        args.instance, (args.initial_x, args.initial_y))

    # Same signal-safety pattern as px4_offboard_smoke_test.py and, before
    # that, visualize_run.py's fixed deadlock: the handler only sets a flag,
    # it never calls rclpy.shutdown() itself.
    def _handle_stop_signal(signum, frame):
        node.request_landing()

    signal.signal(signal.SIGINT, _handle_stop_signal)
    signal.signal(signal.SIGTERM, _handle_stop_signal)

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.5)
            if node._land_sent:
                break
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
