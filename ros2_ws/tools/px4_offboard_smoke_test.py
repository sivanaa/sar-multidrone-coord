#!/usr/bin/env python3
"""Standalone PX4 offboard control smoke test: arm, switch to OFFBOARD, climb
to a fixed hold altitude, and hover. Nothing PSO/CBBA-driven yet.

Deliberately kept separate from coordination_node.py. Per ARCHITECTURE.md's
own plan, the offboard command loop needs to be proven on ONE simulated
drone in isolation before it's wired into the two-drone coordination
scenario — this script is that isolated test, and touches nothing already
verified working. It only targets a single vehicle (target_system=1,
unnamespaced /fmu/in|out/... topics) — namespacing for a second vehicle's
command/telemetry so two drones' Micro-XRCE-DDS-Agent bridges don't collide
on the same topic names is a separate, still-open question (see
ARCHITECTURE.md's PX4 telemetry section) and explicitly out of scope here.

Sequence, following PX4's documented ROS 2 offboard control pattern:
1. Stream OffboardControlMode + TrajectorySetpoint at 10Hz for ~2s before
   attempting the mode switch — PX4 rejects switching into OFFBOARD without
   recent valid setpoints already flowing.
2. Send VEHICLE_CMD_DO_SET_MODE (custom main mode = PX4's OFFBOARD, value 6)
   to switch flight mode.
3. Send VEHICLE_CMD_COMPONENT_ARM_DISARM to arm.
4. Keep streaming both messages every tick from here on — setpoint
   streaming can't stop for more than ~500ms or PX4 auto-exits OFFBOARD as
   a safety fallback — holding position at (0, 0, -hold_altitude) in the
   NED frame (down is positive, hence the negative sign for "up").
5. On SIGINT or once --duration seconds have elapsed, send
   VEHICLE_CMD_NAV_LAND and let PX4 land itself — never just disappear
   mid-flight and stop publishing, even in simulation.

Run (after sourcing ros2_ws/install/setup.bash, alongside a single PX4 SITL
instance with the uXRCE-DDS Agent already bridging it — see ARCHITECTURE.md
for that setup):

    python3 tools/px4_offboard_smoke_test.py --hold-altitude 3.0 --duration 20

Watch `pxh>` console output and/or `ros2 topic echo /fmu/out/vehicle_local_position_v1`
in another window to confirm the vehicle actually arms, climbs, and holds.
"""

import argparse
import signal
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy)

from px4_msgs.msg import (
    OffboardControlMode, TrajectorySetpoint, VehicleCommand,
    VehicleCommandAck, VehicleLocalPosition)

PX4_CUSTOM_MAIN_MODE_OFFBOARD = 6
SETPOINT_HZ = 10.0
# ~2s of streaming at 10Hz before requesting the mode switch, comfortably
# above the "at least a few setpoints first" minimum PX4's own docs describe.
TICKS_BEFORE_MODE_SWITCH = 20
TICKS_BEFORE_ARM = TICKS_BEFORE_MODE_SWITCH + 2


class Px4OffboardSmokeTest(Node):
    def __init__(self, hold_altitude, duration, instance=0):
        super().__init__('px4_offboard_smoke_test')
        self.hold_altitude = hold_altitude
        self.duration = duration
        self.instance = instance

        self._setpoint_count = 0
        self._armed = False
        self._hold_start_time = None
        self._landing = False
        self._land_sent = False
        self._local_position = None  # (x, y, z) NED, from PX4 telemetry

        # PX4's uXRCE-DDS bridge auto-namespaces every instance after the
        # first under /px4_{instance}/fmu/... - instance 0 keeps the plain,
        # unprefixed /fmu/... topics. Confirmed live 2026-09-30 by running a
        # second PX4 SITL instance alongside the first and diffing
        # `ros2 topic list` (see ARCHITECTURE.md's "PX4 telemetry" section).
        prefix = '' if instance == 0 else f'/px4_{instance}'

        # Same QoS shape PX4's own ROS 2 examples use for every uXRCE-DDS
        # topic: best-effort (this is a live control stream, a stale
        # resend is worse than a dropped one) with depth 1 (only the latest
        # value ever matters).
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.offboard_mode_pub = self.create_publisher(
            OffboardControlMode, f'{prefix}/fmu/in/offboard_control_mode', qos)
        self.trajectory_pub = self.create_publisher(
            TrajectorySetpoint, f'{prefix}/fmu/in/trajectory_setpoint', qos)
        self.command_pub = self.create_publisher(
            VehicleCommand, f'{prefix}/fmu/in/vehicle_command', qos)
        self.create_subscription(
            VehicleLocalPosition, f'{prefix}/fmu/out/vehicle_local_position_v1',
            self._on_local_position, qos)
        # Ground truth for whether PX4 ever actually received/processed a
        # command, instead of just trusting that publishing one worked.
        # Added 2026-09-30 while debugging why instance 1's commands seemed
        # to have no effect despite instance 0's identical code path working
        # fine - without this, the script has no way to tell "PX4 rejected
        # this" apart from "PX4 never even saw this" from the ROS2 side.
        self.create_subscription(
            VehicleCommandAck, f'{prefix}/fmu/out/vehicle_command_ack_v1',
            self._on_command_ack, qos)

        self.create_timer(1.0 / SETPOINT_HZ, self._tick)
        self.get_logger().info(
            f'px4_offboard_smoke_test up: will climb to {hold_altitude}m '
            f'and hold for {duration}s once armed')

    def _on_local_position(self, msg):
        self._local_position = (msg.x, msg.y, msg.z)

    def _on_command_ack(self, msg):
        # result: 0=ACCEPTED, 1=TEMPORARILY_REJECTED, 2=DENIED,
        # 3=UNSUPPORTED, 4=FAILED, 5=IN_PROGRESS, 6=CANCELLED (standard PX4
        # VEHICLE_CMD_RESULT enum - printed as a raw number here rather than
        # trusting an exact constant name for this specific px4_msgs build).
        self.get_logger().info(
            f'VehicleCommandAck received: command={msg.command} '
            f'result={msg.result}')

    def _now_us(self):
        return int(self.get_clock().now().nanoseconds / 1000)

    def _publish_offboard_heartbeat(self):
        msg = OffboardControlMode()
        msg.timestamp = self._now_us()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.thrust_and_torque = False
        msg.direct_actuator = False
        self.offboard_mode_pub.publish(msg)

    def _next_xy(self):
        """Horizontal setpoint for this tick. Fixed at the origin here;
        overridden by px4_offboard_pso_test.py to drive this from a live
        PSO particle instead, without duplicating the arm/OFFBOARD/land
        sequence this class already handles."""
        return 0.0, 0.0

    def _publish_hold_setpoint(self):
        x, y = self._next_xy()
        msg = TrajectorySetpoint()
        msg.timestamp = self._now_us()
        # NED frame: down is positive, so climbing is a NEGATIVE z.
        msg.position = [x, y, -self.hold_altitude]
        msg.yaw = 0.0
        self.trajectory_pub.publish(msg)

    def _send_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.timestamp = self._now_us()
        msg.command = command
        msg.param1 = param1
        msg.param2 = param2
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.command_pub.publish(msg)

    def request_landing(self):
        """Called on SIGINT, or once --duration elapses (see _tick). Also
        the natural termination path with no signal at all — either way,
        the process always lands rather than just stopping mid-air."""
        self._landing = True

    def _tick(self):
        if self._landing:
            if not self._land_sent:
                self._send_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
                self._land_sent = True
                self.get_logger().info('Landing commanded.')
            return

        # Must keep streaming both every tick regardless of arm state —
        # this is what PX4 checks to decide whether OFFBOARD is still
        # valid, and it's also what it needs already flowing before it
        # will accept the mode-switch request below in the first place.
        self._publish_offboard_heartbeat()
        self._publish_hold_setpoint()
        self._setpoint_count += 1

        if not self._armed:
            if self._setpoint_count == TICKS_BEFORE_MODE_SWITCH:
                self._send_command(
                    VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
                    param1=1.0, param2=float(PX4_CUSTOM_MAIN_MODE_OFFBOARD))
                self.get_logger().info('Requested OFFBOARD mode.')
            elif self._setpoint_count == TICKS_BEFORE_ARM:
                self._send_command(
                    VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                    param1=1.0)
                self.get_logger().info('Requested ARM.')
                self._armed = True
                self._hold_start_time = time.time()
            return

        if time.time() - self._hold_start_time >= self.duration:
            self.request_landing()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--hold-altitude', type=float, default=3.0)
    parser.add_argument('--duration', type=float, default=20.0)
    parser.add_argument(
        '--instance', type=int, default=0,
        help='PX4 instance number this targets (its own -i N flag). 0 uses '
             'the plain /fmu/... topics; N>0 uses /px4_N/fmu/... (PX4\'s own '
             'uXRCE-DDS auto-namespacing - see ARCHITECTURE.md).')
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = Px4OffboardSmokeTest(args.hold_altitude, args.duration, args.instance)

    # Same signal-safety lesson learned from visualize_run.py's earlier
    # deadlock (see ARCHITECTURE.md/that file's history): the handler only
    # sets a flag, it never calls rclpy.shutdown() itself. request_landing()
    # just marks _landing=True; the next _tick() (already running on the
    # main thread via spin_once) is what actually sends the land command.
    def _handle_stop_signal(signum, frame):
        node.request_landing()

    signal.signal(signal.SIGINT, _handle_stop_signal)
    signal.signal(signal.SIGTERM, _handle_stop_signal)

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.5)
            # Covers both exit paths (SIGINT and --duration elapsing) with
            # one check: either way funnels through request_landing() ->
            # _landing=True -> the next tick sends NAV_LAND and sets
            # _land_sent=True, which is when it's actually safe to stop.
            if node._land_sent:
                break
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
