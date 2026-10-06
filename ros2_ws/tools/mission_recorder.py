#!/usr/bin/env python3
"""Record a coordination run to a JSON-lines file for render_mission.py.

Writes one line per message, stamped with the time it was received:
every drone's AgentState, BundleState and TargetDetected, plus each PX4
instance's battery level when px4_msgs is available. Recording and
rendering are separate so a video can be re-rendered (different speed,
size, crop) without re-flying the run, and rendering needs no ROS at all.

    python3 tools/mission_recorder.py --num-drones 2 --out run.jsonl

Stops cleanly on SIGINT/SIGTERM (sim.sh down sends Ctrl+C), flushing every
line as it goes so a killed run still leaves a usable file.
"""

import argparse
import json
import signal
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy)

from coordination_msgs.msg import AgentState, BundleState, TargetDetected


class MissionRecorder(Node):
    def __init__(self, num_drones, out):
        super().__init__('mission_recorder')
        self._out = out
        for i in range(num_drones):
            base = f'/drone_{i}/coordination'
            self.create_subscription(
                AgentState, f'{base}/agent_state', self._on_agent_state, 50)
            self.create_subscription(
                BundleState, f'{base}/bundle_state', self._on_bundle_state, 50)
            self.create_subscription(
                TargetDetected, f'{base}/target_detected',
                self._on_target_detected, 50)
        self._subscribe_batteries(num_drones)
        self.get_logger().info(f'recording {num_drones} drones to {out.name}')

    def _subscribe_batteries(self, num_drones):
        try:
            from px4_msgs.msg import BatteryStatus
        except ImportError:
            self.get_logger().warning('px4_msgs missing - no battery levels')
            return
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST, depth=1)
        version = getattr(BatteryStatus, 'MESSAGE_VERSION', 0)
        suffix = f'_v{version}' if version else ''
        for i in range(num_drones):
            prefix = '' if i == 0 else f'/px4_{i}'
            self.create_subscription(
                BatteryStatus, f'{prefix}/fmu/out/battery_status{suffix}',
                lambda msg, i=i: self._on_battery(i, msg), qos)

    def _write(self, record):
        record['t'] = time.time()
        self._out.write(json.dumps(record) + '\n')
        self._out.flush()

    def _on_agent_state(self, msg):
        self._write({
            'type': 'agent', 'drone': msg.drone_id, 'state': msg.state,
            'x': msg.position.x, 'y': msg.position.y,
            'goal': [msg.best_position.x, msg.best_position.y]})

    def _on_bundle_state(self, msg):
        self._write({
            'type': 'bundle', 'drone': msg.drone_id,
            'tasks': list(msg.known_task_ids),
            'bids': [float(b) for b in msg.winning_bids],
            'winners': list(msg.winning_agent_ids),
            'bundle': list(msg.bundle),
            'completed': list(msg.completed_task_ids)})

    def _on_target_detected(self, msg):
        self._write({
            'type': 'target', 'drone': msg.drone_id, 'id': msg.target_id,
            'x': msg.position.x, 'y': msg.position.y})

    def _on_battery(self, drone_id, msg):
        if msg.connected and msg.remaining >= 0.0:
            self._write({'type': 'battery', 'drone': drone_id,
                         'remaining': float(msg.remaining)})


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--num-drones', type=int, default=2)
    parser.add_argument('--out', required=True)
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    stop = False

    def _stop(signum, frame):
        nonlocal stop
        stop = True

    # After rclpy.init(), which installs its own SIGINT handler - set
    # before it, this one was replaced and Ctrl+C tore the context down
    # mid-spin.
    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)

    with open(args.out, 'w') as out:
        node = MissionRecorder(args.num_drones, out)
        try:
            while rclpy.ok() and not stop:
                rclpy.spin_once(node, timeout_sec=0.2)
        finally:
            node.destroy_node()
            rclpy.try_shutdown()


if __name__ == '__main__':
    main()
