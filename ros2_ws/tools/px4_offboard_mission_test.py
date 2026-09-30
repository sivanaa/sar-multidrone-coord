#!/usr/bin/env python3
"""PX4 offboard control driven by the full search -> detect -> navigate ->
complete pipeline — one drone, real telemetry, still no second drone/CBBA
consensus involved.

Builds on px4_offboard_smoke_test.py's verified arm/OFFBOARD/climb/land
sequence and px4_offboard_pso_test.py's PSO-driven setpoint pattern, adding
the CBBA + navigate.py pieces coordination_node.py already uses for the
two-drone (simulated-position) demo:

  SEARCH (pso.py) --[detection fires]--> TASK_ALLOCATION (navigate.py)
    --[arrival]--> back to SEARCH (cbba.py's mark_task_done)

With only one drone there's no bidding contest to resolve (CbbaAgent.
build_bundle() just wins the task immediately, same code path as the
two-drone case, just uncontested) — this is deliberately about proving the
state machine + navigate.py actually fly a real vehicle correctly, not about
exercising consensus, which needs a second drone (still blocked on the
open PX4 topic-namespacing question — see ARCHITECTURE.md).

The "detection" isn't a real perception pipeline — it fires automatically
`--detect-after` seconds after arming, at the fixed `--target-x`/`--target-y`
position, the same deliberate simplification demo_run.sh already uses for
the two-drone ROS2-only demo.

Run the same way as the other px4_offboard_*.py scripts — single PX4 SITL
instance + Agent already bridged, NAV_DLL_ACT set to 0 this session, and
(new gotcha found 2026-09-30) sensor_baro_sim started if `listener
sensor_baro` ever shows "never published" (see ARCHITECTURE.md):

    python3 tools/px4_offboard_mission_test.py --hold-altitude 3.0 \\
        --duration 45 --area-radius 5.0 --target-x 3.0 --target-y 3.0 \\
        --detect-after 15
"""

import argparse
import math
import signal
import time

import rclpy

from coordination_node.pso import ParticleSwarmSearch
from coordination_node.cbba import CbbaAgent, Task
from coordination_node.navigate import step_toward
from coordination_node.coordination_node import make_coverage_fitness

from px4_offboard_smoke_test import Px4OffboardSmokeTest, SETPOINT_HZ

ARRIVAL_RADIUS_M = 0.3  # must match navigate.py's step_toward default


class Px4OffboardMissionTest(Px4OffboardSmokeTest):
    def __init__(self, hold_altitude, duration, area_radius,
                 target, detect_after):
        super().__init__(hold_altitude, duration)
        self.pso = ParticleSwarmSearch(
            drone_id=0, initial_position=(0.0, 0.0),
            fitness_fn=make_coverage_fitness((0.0, 0.0), area_radius))
        self.cbba = CbbaAgent(drone_id=0)
        self.target = target
        self.detect_after = detect_after
        self._detected = False
        self.get_logger().info(
            f'Mission active once airborne: search until {detect_after}s '
            f'after arming, then investigate {target}')

    def _maybe_trigger_detection(self):
        if self._detected or self._hold_start_time is None:
            return
        if time.time() - self._hold_start_time < self.detect_after:
            return
        self._detected = True
        task = Task(task_id=1, position=self.target,
                    target_type='person', confidence=0.9)
        self.cbba.add_task(task)
        self.cbba.build_bundle(self._local_position[:2]
                                if self._local_position else (0.0, 0.0))
        self.get_logger().info(f'Detection fired: investigating {self.target}')

    def _next_xy(self):
        if self._local_position is None:
            return 0.0, 0.0
        real_xy = (self._local_position[0], self._local_position[1])
        self._maybe_trigger_detection()

        if self.cbba.bundle:
            task_id = self.cbba.path[0]
            task = self.cbba.tasks[task_id]
            tracked_x, tracked_y, commanded_x, commanded_y = step_toward(
                real_xy, task.position, dt=1.0 / SETPOINT_HZ,
                max_speed=self.pso.max_speed, arrival_radius=ARRIVAL_RADIUS_M,
                real_position=real_xy)
            dx = task.position[0] - tracked_x
            dy = task.position[1] - tracked_y
            if math.hypot(dx, dy) <= ARRIVAL_RADIUS_M:
                self.cbba.mark_task_done(task_id)
                self.get_logger().info(
                    f'Task {task_id} complete - resuming search.')
            return commanded_x, commanded_y

        commanded_x, commanded_y = self.pso.step(
            dt=1.0 / SETPOINT_HZ,
            swarm_best_position=self.pso.state.best_position,
            neighbor_positions=[], real_position=real_xy)
        return commanded_x, commanded_y


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--hold-altitude', type=float, default=3.0)
    parser.add_argument('--duration', type=float, default=45.0)
    parser.add_argument('--area-radius', type=float, default=5.0)
    parser.add_argument('--target-x', type=float, default=3.0)
    parser.add_argument('--target-y', type=float, default=3.0)
    parser.add_argument(
        '--detect-after', type=float, default=15.0,
        help='Seconds after arming before the simulated detection fires.')
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = Px4OffboardMissionTest(
        args.hold_altitude, args.duration, args.area_radius,
        (args.target_x, args.target_y), args.detect_after)

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
