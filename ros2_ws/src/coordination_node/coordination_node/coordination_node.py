"""Main coordination_node: one instance runs per drone.

The ROS shell around one drone's DroneBrain (brain.py), which holds all
of its coordination logic:
- PSO-driven broad-area search (pso.py) while no target is being handled.
- CBBA-based task allocation (cbba.py) once a target is detected, either by
  this drone's own perception or announced by a neighbor.
This node reads the parameters, converts ROS messages to and from the
brain's plain values, runs its 0.5s tick, and links it to a real PX4
vehicle (DDS setpoints, MAVLink ARM/mode/land) when asked to.

Drone-to-drone communication is over per-drone-namespaced ROS2 topics; each
node subscribes to every other drone's topics based on the `num_drones`
parameter, so this works for any fleet size (design goal: generic for N,
prototype at N=5, real deployment target N=2).
"""

import math

import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Point

from coordination_msgs.msg import AgentState, TargetDetected, BundleState
from coordination_msgs.srv import DetectTarget

from coordination_node import brain as brain_module
from coordination_node.brain import TICK_S, DroneBrain
from coordination_node.coverage import load_risk_map
from coordination_node.offboard_sequence import offboard_request

# Real PX4 offboard control constants (see _setup_px4_offboard_control).
PX4_CUSTOM_MAIN_MODE_OFFBOARD = 6
OFFBOARD_HZ = 10.0
# ~2s of streaming at 10Hz before requesting the mode switch - PX4 rejects
# switching into OFFBOARD without recent valid setpoints already flowing.
OFFBOARD_TICKS_BEFORE_MODE_SWITCH = 20
OFFBOARD_TICKS_BEFORE_ARM = OFFBOARD_TICKS_BEFORE_MODE_SWITCH + 2
# Re-send mode switch + ARM this often until VehicleStatus confirms both
# (see offboard_sequence.py).
OFFBOARD_RETRY_TICKS = int(3 * OFFBOARD_HZ)

assert (brain_module.STATE_SEARCH, brain_module.STATE_TASK_ALLOCATION,
        brain_module.STATE_RETURNING) == (
    AgentState.STATE_SEARCH, AgentState.STATE_TASK_ALLOCATION,
    AgentState.STATE_RETURNING), 'brain.py STATE_* out of step with AgentState.msg'


def make_coverage_fitness(area_center, area_radius):
    """Search-value function: rewards being far from your nearest known
    neighbor (spreading out to cover more ground), with a penalty for
    straying outside the shared operating area.

    No longer used by CoordinationNode itself - replaced 2026-10-06 by the
    coverage map (coverage.py), which also fixed drones parking at their
    spawn point. Still used by the standalone single-drone
    tools/px4_offboard_*_test.py scripts, where there are no neighbors.

    This replaces an earlier placeholder that rewarded distance from a
    drone's OWN starting point instead. That version had two real problems,
    both surfaced via live multi-drone testing (2026-09-23/24): it wasn't
    comparable across drones (each drone's value was relative to its own
    origin, so _swarm_best()'s cross-drone comparison below was comparing
    unrelated quantities), and its social-pull term actively fought pso.py's
    own separation repulsion instead of cooperating with it — two drones
    could be pulled toward each other by "exploration" fitness while
    simultaneously pushed apart by collision safety. It also meant a drone
    whose start happened to be near a later-detected target got rewarded
    for moving AWAY from that target, purely as a side effect of not being
    target-aware at all — confirmed by deliberately placing test targets
    near each drone's start and watching the "wrong" drone win.

    Nearest-neighbor distance fixes the comparability problem (same
    reference frame for every drone) and the collision-safety conflict
    (spreading out is now what BOTH fitness and repulsion want), though it
    still isn't a real coverage/search-quality score — see the TODO below.
    `area_center`/`area_radius` exist only to stop this from rewarding
    drones running infinitely far apart in the unbounded case; they are a
    soft mission-area bound, not a hard fence.

    Nearest-neighbor spread has no idea where a target is likely to be;
    the node's own search (coverage.py) now weights cells by flood risk.
    """
    center_x, center_y = area_center

    def fitness(x, y, neighbor_positions):
        distance_from_center = math.hypot(x - center_x, y - center_y)
        if neighbor_positions:
            spread = min(math.hypot(x - nx, y - ny)
                         for nx, ny in neighbor_positions)
        else:
            # No neighbor data yet (e.g. the first tick, or a slow ROS2
            # discovery handshake) - fall back to distance-from-area-center
            # rather than a flat constant. A constant here reproduces the
            # exact flat-fitness bug this file's own history already
            # rejected once (see above): every position looks equally
            # good, so best_fitness never improves past the very first
            # tick, the cognitive/social pull terms both collapse to zero
            # once near the start, and the particle freezes solid.
            # Confirmed live 2026-09-29: drone 0 sat frozen at its exact
            # starting position (distance-to-target flat at exactly 5.0,
            # its literal straight-line distance from (0,0) to (3,4)) for
            # the ~27s it took to receive its first AgentState from drone
            # 1 on this run - unusually slow discovery, but the constant
            # fallback turned a brief startup blip into a full freeze.
            # Distance-from-center still varies smoothly with position, so
            # it can't flatline the same way, and unlike distance-from-OWN-
            # origin it's the same reference frame for every drone, so it
            # doesn't reintroduce the "punished for starting near the
            # target" bug either - it only ever applies during this brief
            # bootstrap window before real neighbor data exists.
            spread = distance_from_center
        overreach = max(0.0, distance_from_center - area_radius)
        return spread - 2.0 * overreach
    return fitness


def _stamp_s(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def _to_time(seconds):
    sec = int(seconds)
    return Time(sec=sec, nanosec=int(round((seconds - sec) * 1e9)) % 1000000000)


class CoordinationNode(Node):
    def __init__(self):
        super().__init__('coordination_node')

        self.declare_parameter('drone_id', 0)
        self.drone_id = self.get_parameter('drone_id').value
        self.declare_parameter('num_drones', 2)
        # Spawn position in the fleet's shared frame: drone 0's PX4 local
        # NED frame (x = north, y = east, origin at drone 0's spawn point).
        # Every drone's PX4 reports position relative to its OWN spawn
        # point, so with real PX4 enabled this is also the offset that
        # converts that local frame into the shared one (see
        # _px4_frame_origin). Gazebo's world frame is ENU, so a vehicle
        # spawned with PX4_GZ_MODEL_POSE="gx,gy" needs initial_x:=gy,
        # initial_y:=gx here.
        self.declare_parameter('initial_x', 0.0)
        self.declare_parameter('initial_y', 0.0)
        # The search area (coverage.py) - a mission-level setting, not
        # per-drone, so every drone in a fleet should normally be launched
        # with the same values. Defaults are deliberately generic/small rather than tuned
        # to any one demo's coordinates (this node stays usable for any N
        # drones at any scale); override per deployment the same way
        # initial_x/initial_y already are.
        self.declare_parameter('area_center_x', 0.0)
        self.declare_parameter('area_center_y', 0.0)
        self.declare_parameter('area_radius', 10.0)
        # How far around itself a drone counts as having searched, and how
        # long until a searched cell is worth searching again. 1.5m / 60s
        # keeps two drones busy over a 6m-radius area (dry run: 95% covered
        # in ~15s, then continuous revisits) without parking.
        self.declare_parameter('sensor_radius', 1.5)
        self.declare_parameter('revisit_after_s', 60.0)
        # Flood-risk map (coverage.load_risk_map format) weighting the
        # search toward high-risk ground; '' = every cell equal. Every drone
        # in a fleet should get the same file.
        self.declare_parameter('risk_map_file', '')
        # Comma-separated target types this drone can take (cbba.py's
        # task_types), e.g. 'person,drop'; '' = any. sim.sh test-x uses it
        # to give each drone a target only it may take.
        self.declare_parameter('task_types', '')
        self.declare_parameter('use_px4_position', False)
        self.declare_parameter('use_px4_offboard', False)
        self.declare_parameter('hold_altitude', 3.0)
        # PX4 v1.18.0-beta1 (the version on the project's drone server, see
        # ARCHITECTURE.md) publishes VehicleLocalPosition under a versioned
        # topic name. PX4's uXRCE-DDS bridge also auto-namespaces every
        # instance after the first under /px4_{instance}/fmu/... - instance
        # 0 keeps the plain, unprefixed /fmu/... topics (confirmed live
        # 2026-09-30, running a second PX4 SITL instance alongside the
        # first and diffing `ros2 topic list`). drone_id is assumed to
        # match the PX4 instance's own `-i N` flag (same convention
        # demo_run.sh already uses pairing drone_id with initial_x/y), so
        # the default topic name is computed from it instead of being
        # hardcoded to the single-drone case.
        px4_prefix = '' if self.drone_id == 0 else f'/px4_{self.drone_id}'
        self.declare_parameter(
            'px4_local_position_topic',
            f'{px4_prefix}/fmu/out/vehicle_local_position_v1')

        self.num_drones = self.get_parameter('num_drones').value
        init_x = self.get_parameter('initial_x').value
        init_y = self.get_parameter('initial_y').value
        self.use_px4_position = self.get_parameter('use_px4_position').value
        self.use_px4_offboard = self.get_parameter('use_px4_offboard').value
        self.hold_altitude = self.get_parameter('hold_altitude').value
        # Where this drone's PX4 local-frame origin sits in the shared
        # frame. Everything in the brain (PSO, CBBA bids, navigate,
        # neighbor positions, target coordinates) is in the shared frame;
        # only the PX4 boundary converts - add on telemetry in, subtract on
        # setpoints out. Without it, every drone after the first believed
        # it was at its own spawn point's origin, so bids compared
        # distances in different frames and the same target coordinate
        # meant a different physical spot for each drone. Assumes PX4's
        # EKF origin is the spawn point, true in SITL; real hardware would
        # need this derived from VehicleLocalPosition's ref_lat/ref_lon.
        self._px4_frame_origin = (init_x, init_y)

        self.agent_state_pub = self.create_publisher(
            AgentState, f'/drone_{self.drone_id}/coordination/agent_state', 10)
        self.target_pub = self.create_publisher(
            TargetDetected, f'/drone_{self.drone_id}/coordination/target_detected', 10)
        self.bundle_pub = self.create_publisher(
            BundleState, f'/drone_{self.drone_id}/coordination/bundle_state', 10)

        if self.use_px4_offboard and not self.use_px4_position:
            self.get_logger().warning(
                'use_px4_offboard:=true implies use_px4_position - '
                'enabling it automatically.')
            self.use_px4_position = True
        if self.use_px4_position:
            self._setup_px4_position_subscription()
        if self.use_px4_offboard:
            self._setup_px4_offboard_control()

        # After the PX4 setup: it can fall back to simulated-only control.
        task_types = [t.strip() for t in
                      self.get_parameter('task_types').value.split(',')
                      if t.strip()]
        self.brain = DroneBrain(
            self.drone_id, (init_x, init_y),
            (self.get_parameter('area_center_x').value,
             self.get_parameter('area_center_y').value),
            self.get_parameter('area_radius').value,
            clock=self._now_s, outbox=self,
            sensor_radius=self.get_parameter('sensor_radius').value,
            revisit_after_s=self.get_parameter('revisit_after_s').value,
            risk=self._load_risk_map(), task_types=task_types,
            real_vehicle=self.use_px4_offboard)

        self.detect_srv = self.create_service(
            DetectTarget, f'/drone_{self.drone_id}/coordination/detect_target',
            self._on_detect_target_service)
        for other_id in range(self.num_drones):
            if other_id == self.drone_id:
                continue
            self.create_subscription(
                AgentState, f'/drone_{other_id}/coordination/agent_state',
                self._on_agent_state, 10)
            self.create_subscription(
                TargetDetected, f'/drone_{other_id}/coordination/target_detected',
                self._on_target_detected, 10)
            self.create_subscription(
                BundleState, f'/drone_{other_id}/coordination/bundle_state',
                self._on_bundle_state, 10)

        self.create_timer(TICK_S, self.brain.tick)

        self.get_logger().info(
            f'coordination_node up: drone_id={self.drone_id} '
            f'num_drones={self.num_drones}'
        )

    def _load_risk_map(self):
        path = self.get_parameter('risk_map_file').value
        if not path:
            return None
        try:
            cell_size, risk = load_risk_map(path)
        except (OSError, ValueError, KeyError) as exc:
            self.get_logger().error(
                f'risk_map_file {path} unreadable ({exc}) - searching '
                'without flood risk.')
            return None
        if cell_size != 1.0:  # CoverageMap's cells are 1m
            self.get_logger().error(
                f'risk map cell size {cell_size}m, search grid is 1m - '
                'searching without flood risk.')
            return None
        high = sum(1 for r in risk.values() if r >= 0.5)
        self.get_logger().info(
            f'flood risk map {path}: {len(risk)} cells, {high} high-risk '
            '(>= 0.5).')
        return risk

    # ---- PX4 real position (optional) --------------------------------------

    def _setup_px4_position_subscription(self):
        """Subscribe to PX4's local position via the Micro-XRCE-DDS bridge.

        `px4_msgs` is vendored into this same workspace
        (`ros2_ws/src/px4_msgs`, gitignored, pinned to a specific PX4 commit
        — see ARCHITECTURE.md) and built together with `coordination_node`/
        `coordination_msgs` by the same `colcon build`. The import is still
        done here, on demand, only when `use_px4_position` is set, so
        running without `px4_msgs` present (e.g. a checkout that hasn't
        pulled it) still works exactly as before, just without real
        telemetry.
        """
        try:
            from px4_msgs.msg import BatteryStatus, VehicleLocalPosition
        except ImportError:
            self.get_logger().warning(
                'use_px4_position:=true but px4_msgs is not present in '
                'ros2_ws/src — rebuild with it vendored in (see '
                'ARCHITECTURE.md) — falling back to internally-simulated '
                'position.')
            self.use_px4_position = False
            return

        from rclpy.qos import (
            QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        topic = self.get_parameter('px4_local_position_topic').value
        self.create_subscription(
            VehicleLocalPosition, topic, self._on_px4_local_position, qos_profile)
        # Versioned topic name derived the same way as vehicle_status's.
        px4_prefix = '' if self.drone_id == 0 else f'/px4_{self.drone_id}'
        version = getattr(BatteryStatus, 'MESSAGE_VERSION', 0)
        self.create_subscription(
            BatteryStatus, f'{px4_prefix}/fmu/out/battery_status'
            + (f'_v{version}' if version else ''),
            self._on_px4_battery, qos_profile)

    def _on_px4_local_position(self, msg):
        origin_x, origin_y = self._px4_frame_origin
        self.brain.real_position = (msg.x + origin_x, msg.y + origin_y)

    def _on_px4_battery(self, msg):
        if msg.connected and msg.remaining >= 0.0:
            self.brain.battery_remaining = msg.remaining

    # ---- PX4 real offboard control (optional) ------------------------------

    def _setup_px4_offboard_control(self):
        """Actually fly a real PX4 vehicle from this node's own PSO/navigate
        output, instead of only reading telemetry (use_px4_position).

        Setpoint streaming (OffboardControlMode + TrajectorySetpoint) goes
        over DDS, same as the standalone px4_offboard_*_test.py scripts -
        that path is solid for every instance, proven across many live
        runs. ARM and mode-switch (DO_SET_MODE) instead go over plain
        MAVLink, not DDS's VehicleCommand topic: PX4 SITL silently drops
        inbound DDS VehicleCommand messages for any instance N>0 (a known,
        unresolved upstream bug - GitHub px4/PX4-Autopilot#21284), confirmed
        empirically on two independent machines (2026-09-30 through
        2026-10-02). MAVLink's own command path isn't affected at all -
        confirmed live 2026-10-05: an identical ARM/DO_SET_MODE request
        gets an immediate ACCEPTED ack over MAVLink on instance 1, same
        instant the DDS equivalent is silently ignored. This also isn't a
        sim-only workaround - MAVLink is the standard way a real companion
        computer talks to PX4 on actual hardware too.
        """
        try:
            from px4_msgs.msg import (
                OffboardControlMode, TrajectorySetpoint, VehicleStatus)
        except ImportError:
            self.get_logger().warning(
                'use_px4_offboard:=true but px4_msgs is not present in '
                'ros2_ws/src - rebuild with it vendored in (see '
                'ARCHITECTURE.md) - falling back to simulated-only '
                'control.')
            self.use_px4_offboard = False
            return
        try:
            from pymavlink import mavutil
        except ImportError:
            self.get_logger().warning(
                'use_px4_offboard:=true but pymavlink is not installed '
                '(pip install pymavlink) - falling back to simulated-only '
                'control.')
            self.use_px4_offboard = False
            return

        from rclpy.qos import (
            QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        px4_prefix = '' if self.drone_id == 0 else f'/px4_{self.drone_id}'
        self._OffboardControlMode = OffboardControlMode
        self._TrajectorySetpoint = TrajectorySetpoint
        self._mavutil = mavutil
        self._offboard_mode_pub = self.create_publisher(
            OffboardControlMode, f'{px4_prefix}/fmu/in/offboard_control_mode',
            qos_profile)
        self._trajectory_pub = self.create_publisher(
            TrajectorySetpoint, f'{px4_prefix}/fmu/in/trajectory_setpoint',
            qos_profile)
        # PX4 suffixes a versioned message's topic with its version (same
        # reason the position topic is vehicle_local_position_v1), so derive
        # it from the vendored px4_msgs rather than hardcoding it.
        version = getattr(VehicleStatus, 'MESSAGE_VERSION', 0)
        status_topic = f'{px4_prefix}/fmu/out/vehicle_status' + (
            f'_v{version}' if version else '')
        self.create_subscription(
            VehicleStatus, status_topic, self._on_px4_vehicle_status,
            qos_profile)

        # Each PX4 SITL instance exposes its own 'Normal'-mode MAVLink UDP
        # link on local port 18570 + instance (confirmed from every
        # instance's own boot log), and reports itself as MAVLink
        # system-id instance+1 - both conventions assume drone_id matches
        # the PX4 -i N instance number, same assumption px4_prefix above
        # already makes.
        mavlink_port = 18570 + self.drone_id
        self._mavlink_target_system = self.drone_id + 1
        self._mavlink = mavutil.mavlink_connection(
            f'udpout:127.0.0.1:{mavlink_port}', source_system=255)
        # udpout only learns our reply address once PX4 has received a
        # packet FROM us - without this, PX4 has no one to send heartbeats
        # back to, though outbound commands from us would still arrive
        # either way. Sent periodically afterward (see
        # _offboard_control_tick) so PX4 doesn't log a lost-GCS warning.
        self._send_mavlink_heartbeat()

        self._offboard_setpoint_count = 0
        self._px4_status_seen = False
        self._px4_flight_confirmed = False  # armed AND in OFFBOARD, per PX4
        self._px4_request_attempts = 0
        self._landing_requested = False
        self._land_sent = False
        self._exit_after_landing = True
        self.create_timer(1.0 / OFFBOARD_HZ, self._offboard_control_tick)
        self.get_logger().info(
            f'Real PX4 offboard control enabled for drone {self.drone_id}: '
            f'DDS setpoints on {px4_prefix or "(unprefixed)"}/fmu/in/..., '
            f'MAVLink ARM/mode-switch on udp port {mavlink_port}, '
            f'confirmed via {status_topic}.')

    def _on_px4_vehicle_status(self, msg):
        self._px4_status_seen = True
        if self._px4_flight_confirmed:
            return
        if (msg.arming_state == msg.ARMING_STATE_ARMED
                and msg.nav_state == msg.NAVIGATION_STATE_OFFBOARD):
            self._px4_flight_confirmed = True
            self.get_logger().info(
                f'Drone {self.drone_id}: PX4 confirms armed in OFFBOARD '
                f'(attempt {self._px4_request_attempts}).')

    def _now_us(self):
        return int(self.get_clock().now().nanoseconds / 1000)

    def _send_mavlink_heartbeat(self):
        # A raw UDP socket send can occasionally raise (e.g. a transient OS-
        # level hiccup) - an uncaught exception here would propagate out of
        # the timer callback and silently kill the whole node (observed
        # live 2026-10-05: the ROS2 process disappeared with no traceback
        # captured, and PX4's own lost-setpoint-stream failsafe was what
        # actually brought the vehicle down safely, not our own landing
        # code). Logging and skipping this one send is far better than
        # losing the whole control loop over it - the next tick tries again.
        try:
            self._mavlink.mav.heartbeat_send(
                self._mavutil.mavlink.MAV_TYPE_GCS,
                self._mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
        except OSError as exc:
            self.get_logger().warning(f'MAVLink heartbeat send failed: {exc}')

    def _send_mavlink_command(self, command, param1=0.0, param2=0.0):
        try:
            self._mavlink.mav.command_long_send(
                self._mavlink_target_system, 1,  # component 1 = autopilot
                command, 0, param1, param2, 0, 0, 0, 0, 0)
        except OSError as exc:
            self.get_logger().warning(
                f'MAVLink command {command} send failed: {exc}')

    def _publish_offboard_setpoint(self):
        heartbeat = self._OffboardControlMode()
        heartbeat.timestamp = self._now_us()
        heartbeat.position = True
        self._offboard_mode_pub.publish(heartbeat)

        # Through the collision floor at the full 10Hz setpoint rate, against
        # the freshest neighbor positions (also broadcast at 10Hz, see
        # _offboard_control_tick) - see DroneBrain.setpoint.
        commanded = self.brain.setpoint()

        # Shared frame -> this vehicle's own PX4 local frame.
        origin_x, origin_y = self._px4_frame_origin
        x = commanded[0] - origin_x
        y = commanded[1] - origin_y
        setpoint = self._TrajectorySetpoint()
        setpoint.timestamp = self._now_us()
        # NED frame: down is positive, so climbing is a NEGATIVE z.
        setpoint.position = [x, y, -self.hold_altitude]
        setpoint.yaw = 0.0
        self._trajectory_pub.publish(setpoint)

    def request_landing(self, stay_up=False):
        """Called on SIGINT/SIGTERM (see main()), and on reaching home at
        battery reserve (_return_home, with `stay_up`). Land via MAVLink,
        same as ARM/mode-switch - never just stop publishing and leave a
        real vehicle hanging mid-air. The node exits once LAND is sent
        unless `stay_up`: a drone that lands on low battery keeps
        broadcasting where it really is. When it exited at once, its last
        broadcast was the in-flight point where LAND was sent, ~0.3m from
        where PX4 actually set it down, and the other drone's floor steered
        around the wrong spot - 1.42m on the meter (2026-10-06, harmless,
        3m above a landed drone, but the floor should hold)."""
        self._landing_requested = True
        self._exit_after_landing = not stay_up

    def _offboard_control_tick(self):
        if self._landing_requested:
            if not self._land_sent:
                self._send_mavlink_command(
                    self._mavutil.mavlink.MAV_CMD_NAV_LAND)
                self._land_sent = True
                self.get_logger().info(
                    f'Drone {self.drone_id}: landing commanded (MAVLink).')
            self.brain.publish_agent_state()  # keep neighbors' floors accurate
            return

        # Must keep streaming every tick regardless of arm state - this is
        # what PX4 checks to decide whether OFFBOARD is still valid, and
        # it's also what it needs already flowing before it accepts the
        # mode-switch request below.
        self._publish_offboard_setpoint()
        self.brain.publish_agent_state()  # 10Hz position for neighbors' floors
        self._offboard_setpoint_count += 1
        if self._offboard_setpoint_count % int(OFFBOARD_HZ) == 0:
            self._send_mavlink_heartbeat()

        request = offboard_request(
            self._offboard_setpoint_count, self._px4_flight_confirmed,
            first_tick=OFFBOARD_TICKS_BEFORE_MODE_SWITCH,
            retry_ticks=OFFBOARD_RETRY_TICKS,
            arm_delay=(OFFBOARD_TICKS_BEFORE_ARM
                       - OFFBOARD_TICKS_BEFORE_MODE_SWITCH))
        if request == 'mode':
            self._px4_request_attempts += 1
            self._send_mavlink_command(
                self._mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                param1=1, param2=PX4_CUSTOM_MAIN_MODE_OFFBOARD)
            note = '' if self._px4_status_seen else (
                ' - no VehicleStatus from PX4 yet, check its DDS agent')
            self.get_logger().info(
                f'Drone {self.drone_id}: requested OFFBOARD mode (MAVLink, '
                f'attempt {self._px4_request_attempts}){note}.')
        elif request == 'arm':
            self._send_mavlink_command(
                self._mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, param1=1)
            self.get_logger().info(
                f'Drone {self.drone_id}: requested ARM (MAVLink, attempt '
                f'{self._px4_request_attempts}).')

    # ---- brain <-> ROS ------------------------------------------------------

    def _now_s(self):
        return self.get_clock().now().nanoseconds / 1e9

    def log(self, level, text):
        # One call per severity: rclpy's logger caches the severity of each
        # call site, and a single getattr(logger, level)(text) line raised
        # "Logger severity cannot be changed between calls" on the first
        # warning - killing drone 1's node at battery reserve in live
        # test-handoff (2026-10-16; PX4's failsafe landed it).
        if level == 'warning':
            self.get_logger().warning(text)
        elif level == 'error':
            self.get_logger().error(text)
        else:
            self.get_logger().info(text)

    def publish_agent_state(self, s):
        msg = AgentState()
        msg.drone_id = s['drone_id']
        msg.stamp = _to_time(s['stamp'])
        msg.position = Point(x=float(s['position'][0]),
                             y=float(s['position'][1]), z=0.0)
        msg.best_position = Point(x=float(s['best_position'][0]),
                                  y=float(s['best_position'][1]), z=0.0)
        msg.best_fitness = s['best_fitness']
        msg.state = s['state']
        self.agent_state_pub.publish(msg)

    def publish_target(self, t):
        msg = TargetDetected()
        msg.drone_id = t['drone_id']
        msg.stamp = _to_time(t['stamp'])
        msg.target_id = t['target_id']
        msg.position = Point(x=float(t['position'][0]),
                             y=float(t['position'][1]), z=0.0)
        msg.target_type = t['target_type']
        msg.confidence = float(t['confidence'])
        self.target_pub.publish(msg)

    def publish_bundle(self, b):
        msg = BundleState()
        msg.drone_id = b['drone_id']
        msg.stamp = _to_time(b['stamp'])
        msg.known_task_ids = b['known_task_ids']
        msg.winning_bids = [float(v) for v in b['winning_bids']]
        msg.winning_agent_ids = b['winning_agent_ids']
        msg.update_times = [float(v) for v in b['update_times']]
        msg.bundle = b['bundle']
        msg.completed_task_ids = b['completed_task_ids']
        self.bundle_pub.publish(msg)

    def _on_agent_state(self, msg: AgentState):
        self.brain.on_agent_state(
            msg.drone_id, _stamp_s(msg.stamp),
            (msg.position.x, msg.position.y),
            (msg.best_position.x, msg.best_position.y), msg.best_fitness)

    def _on_target_detected(self, msg: TargetDetected):
        self.brain.on_target_detected(
            msg.target_id, (msg.position.x, msg.position.y),
            msg.target_type, msg.confidence)

    def _on_bundle_state(self, msg: BundleState):
        self.brain.on_bundle_state(
            msg.drone_id, list(msg.known_task_ids), list(msg.winning_bids),
            list(msg.winning_agent_ids), list(msg.update_times),
            list(msg.completed_task_ids))

    def _on_detect_target_service(self, request, response):
        """Entry point for perception (vision_bridge.py) and tests (`ros2
        service call`): this drone found a target. Goes through the same
        self-bid-then-broadcast path as any detection - unlike publishing
        directly onto /coordination/target_detected from outside, which
        only reaches other drones, never this one."""
        response.target_id = self.brain.report_target_detected(
            (request.position.x, request.position.y),
            request.target_type, request.confidence)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = CoordinationNode()

    if not node.use_px4_offboard:
        try:
            rclpy.spin(node)
        finally:
            node.destroy_node()
            rclpy.shutdown()
        return

    # Real flight: never just stop publishing and leave a vehicle hanging
    # mid-air - same safe signal-handling pattern used by the standalone
    # px4_offboard_*_test.py scripts (the handler only sets a flag;
    # rclpy.shutdown() happens exactly once, at the very end, after
    # destroy_node() - see ARCHITECTURE.md for the deadlock this avoids).
    import signal

    def _handle_stop_signal(signum, frame):
        node.request_landing()

    signal.signal(signal.SIGINT, _handle_stop_signal)
    signal.signal(signal.SIGTERM, _handle_stop_signal)

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.5)
            if node._land_sent and node._exit_after_landing:
                break
    finally:
        node.destroy_node()
        rclpy.shutdown()
