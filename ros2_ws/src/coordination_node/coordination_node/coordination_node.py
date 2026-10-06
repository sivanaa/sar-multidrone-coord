"""Main coordination_node: one instance runs per drone.

Wires together:
- PSO-driven broad-area search (pso.py) while no target is being handled.
- CBBA-based task allocation (cbba.py) once a target is detected, either by
  this drone's own perception or announced by a neighbor.

Drone-to-drone communication is over per-drone-namespaced ROS2 topics; each
node subscribes to every other drone's topics based on the `num_drones`
parameter, so this works for any fleet size (design goal: generic for N,
prototype at N=5, real deployment target N=2).
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point

from coordination_msgs.msg import AgentState, TargetDetected, BundleState
from coordination_msgs.srv import DetectTarget

from coordination_node.pso import ParticleSwarmSearch
from coordination_node.cbba import (
    BATTERY_RESERVE, UNASSIGNED, CbbaAgent, Task, battery_value_scale)
from coordination_node.coverage import CoverageMap, path_clearance
from coordination_node.navigate import step_toward
from coordination_node.offboard_sequence import offboard_request
from coordination_node.separation import (
    constrain_setpoint, enforce_min_separation)

MIN_SEPARATION_M = 1.5  # must match pso.py's default; see separation.py
# Floor applied to real-vehicle *setpoints*, deliberately above
# MIN_SEPARATION_M: PX4 tracks a setpoint with some lag and the setpoint
# only updates every 0.5s tick, so commanding exactly 1.5m let the real
# distance dip to 1.40m (measured live 2026-10-05, two drones spawned 1m
# apart, ~+/-0.1m wobble around the commanded value). The margin keeps the
# actual distance at or above MIN_SEPARATION_M.
SETPOINT_MIN_SEPARATION_M = MIN_SEPARATION_M + 0.3
# Radius at which navigate.py starts steering a real vehicle around a
# neighbor. Must be above SETPOINT_MIN_SEPARATION_M: at 1.5m the soft term
# never acted, because the setpoint floor already held the vehicle at 1.8m
# (head-on stall, dry run 2026-10-06). 2.5m gets past a neighbor dead ahead
# in ~8s instead of ~24s.
SETPOINT_AVOID_RADIUS_M = 2.5
ARRIVAL_RADIUS_M = 0.3  # must match navigate.py's step_toward default

# Search (see coverage.py). A search goal whose straight path passes within
# SEARCH_PATH_CLEARANCE_M of a neighbor's path to its own goal loses
# PATH_CONFLICT_PENALTY fitness, so two drones don't pick crossing goals
# (dry run 2026-10-06, PX4 tracking modeled: runs dipping under 1.5m went
# from 12/40 to 4/40 with it). SEARCH_CANDIDATES unexplored cells are
# offered to PSO as possible new goals each tick.
SEARCH_PATH_CLEARANCE_M = 2.5
PATH_CONFLICT_PENALTY = 10.0
SEARCH_CANDIDATES = 8
SEARCH_LOG_EVERY_TICKS = 10  # 5s

STATE_SEARCH = AgentState.STATE_SEARCH
STATE_TASK_ALLOCATION = AgentState.STATE_TASK_ALLOCATION
STATE_RETURNING = AgentState.STATE_RETURNING

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

    TODO: replace with the flood-risk-weighted scoring already used by the
    single-drone GPS routing (edge-ai-gateway), once that scoring is
    exposed to this node. Nearest-neighbor spread is a reasonable
    placeholder for "don't duplicate another drone's search effort," but it
    still has no idea where a target is actually likely to be.
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

        area_center = (
            self.get_parameter('area_center_x').value,
            self.get_parameter('area_center_y').value)
        area_radius = self.get_parameter('area_radius').value

        self.state = STATE_SEARCH
        self.neighbor_best = {}  # drone_id -> ((x, y), fitness): its search goal
        self.neighbor_position = {}  # drone_id -> (x, y)
        self.coverage = CoverageMap(
            area_center, area_radius,
            sensor_radius=self.get_parameter('sensor_radius').value,
            revisit_after_s=self.get_parameter('revisit_after_s').value)
        self._own_xy = (init_x, init_y)  # where this drone is, for path checks
        self._home = (init_x, init_y)  # where it goes to land on low battery
        self._battery_remaining = None  # 0-1 from PX4; None = unknown/no PX4
        self._search_ticks = 0
        self.pso = ParticleSwarmSearch(
            drone_id=self.drone_id,
            initial_position=(init_x, init_y),
            fitness_fn=self._search_fitness,
        )
        self.cbba = CbbaAgent(drone_id=self.drone_id)

        # Real PX4 telemetry, when enabled, overrides PSO's internally
        # integrated position each tick (see pso.py's `step`). None until
        # the first message arrives, which pso.step() already treats the
        # same as "not using real position yet".
        self._real_position = None
        self.use_px4_position = self.get_parameter('use_px4_position').value
        self.use_px4_offboard = self.get_parameter('use_px4_offboard').value
        self.hold_altitude = self.get_parameter('hold_altitude').value
        # Setpoint actually sent to PX4 each offboard tick - starts at the
        # drone's own initial position (hold in place) until the first
        # PSO/navigate step computes a real commanded target.
        self._commanded_xy = (init_x, init_y)
        # Where this drone's PX4 local-frame origin sits in the shared
        # frame. Everything inside this node (PSO, CBBA bids, navigate,
        # neighbor positions, target coordinates) is in the shared frame;
        # only the PX4 boundary converts - add on telemetry in, subtract on
        # setpoints out. Without it, every drone after the first believed
        # it was at its own spawn point's origin, so bids compared
        # distances in different frames and the same target coordinate
        # meant a different physical spot for each drone. Assumes PX4's
        # EKF origin is the spawn point, true in SITL; real hardware would
        # need this derived from VehicleLocalPosition's ref_lat/ref_lon.
        self._px4_frame_origin = (init_x, init_y)
        if self.use_px4_offboard and not self.use_px4_position:
            self.get_logger().warning(
                'use_px4_offboard:=true implies use_px4_position - '
                'enabling it automatically.')
            self.use_px4_position = True
        if self.use_px4_position:
            self._setup_px4_position_subscription()
        if self.use_px4_offboard:
            self._setup_px4_offboard_control()

        self._next_task_id = self.drone_id * 100000  # cheap collision-free id space

        self.agent_state_pub = self.create_publisher(
            AgentState, f'/drone_{self.drone_id}/coordination/agent_state', 10)
        self.target_pub = self.create_publisher(
            TargetDetected, f'/drone_{self.drone_id}/coordination/target_detected', 10)
        self.bundle_pub = self.create_publisher(
            BundleState, f'/drone_{self.drone_id}/coordination/bundle_state', 10)
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

        self.create_timer(0.5, self._tick)

        self.get_logger().info(
            f'coordination_node up: drone_id={self.drone_id} '
            f'num_drones={self.num_drones}'
        )

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
        self._real_position = (msg.x + origin_x, msg.y + origin_y)

    def _on_px4_battery(self, msg):
        if msg.connected and msg.remaining >= 0.0:
            self._battery_remaining = msg.remaining

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

        # The collision floor again, at the full 10Hz setpoint rate against
        # the freshest neighbor positions (also broadcast at 10Hz, see
        # _offboard_control_tick) - not just once per 0.5s _tick. With
        # positions up to 0.5s stale, two drones closing at ~3 m/s met at
        # 0.92m in a live coverage-search run (2026-10-06); a dry run of the
        # same search put 14/40 runs under 1.5m at 2Hz, 0/40 at 10Hz.
        commanded = self._commanded_xy
        if self._real_position is not None:
            commanded = constrain_setpoint(
                commanded, self._real_position,
                list(self.neighbor_position.values()),
                SETPOINT_MIN_SEPARATION_M)

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
            self._publish_agent_state()  # keep neighbors' floors accurate
            return

        # Must keep streaming every tick regardless of arm state - this is
        # what PX4 checks to decide whether OFFBOARD is still valid, and
        # it's also what it needs already flowing before it accepts the
        # mode-switch request below.
        self._publish_offboard_setpoint()
        self._publish_agent_state()  # 10Hz position for neighbors' floors
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

    # ---- PSO / neighbor tracking -------------------------------------------

    def _on_agent_state(self, msg: AgentState):
        self.neighbor_best[msg.drone_id] = (
            (msg.best_position.x, msg.best_position.y), msg.best_fitness)
        self.neighbor_position[msg.drone_id] = (msg.position.x, msg.position.y)

    def _now_s(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _search_fitness(self, x, y, neighbor_positions):
        """PSO fitness: unexplored ground a drone at (x, y) would see that no
        other drone is at or heading to (coverage.py), minus a penalty if
        getting there means crossing a neighbor's path to its own goal."""
        goals = [goal for goal, _ in self.neighbor_best.values()]
        value = self.coverage.fitness(
            x, y, self._now_s(), claimed=list(neighbor_positions) + goals)
        for drone_id, (goal, _) in self.neighbor_best.items():
            start = self.neighbor_position.get(drone_id)
            if start is not None and path_clearance(
                    self._own_xy, (x, y), start, goal) < SEARCH_PATH_CLEARANCE_M:
                value -= PATH_CONFLICT_PENALTY
        return value

    def _swarm_best(self):
        """Best goal across the swarm, re-scored with this drone's own
        up-to-date map rather than trusting each neighbor's broadcast score
        (computed against a different, older picture). In practice this
        drone's own goal usually wins - a neighbor's goal is one it has
        claimed - so the swarm cooperates through the shared coverage map
        and claims rather than through PSO's social pull."""
        neighbors = list(self.neighbor_position.values())
        best_pos = self.pso.state.best_position
        best_fit = self._search_fitness(*best_pos, neighbors)
        for pos, _ in self.neighbor_best.values():
            fit = self._search_fitness(*pos, neighbors)
            if fit > best_fit:
                best_pos, best_fit = pos, fit
        return best_pos

    def _publish_agent_state(self):
        msg = AgentState()
        msg.drone_id = self.drone_id
        msg.stamp = self.get_clock().now().to_msg()
        # Real telemetry when there is some - pso.state.position only
        # catches up with it once per 0.5s _tick.
        x, y = (self._real_position if self._real_position is not None
                else self.pso.state.position)
        msg.position = Point(x=x, y=y, z=0.0)
        msg.best_position = Point(
            x=self.pso.state.best_position[0],
            y=self.pso.state.best_position[1], z=0.0)
        msg.best_fitness = float(self.pso.state.best_fitness)
        msg.state = self.state
        self.agent_state_pub.publish(msg)

    # ---- Target detection / CBBA -------------------------------------------

    def report_target_detected(self, position_xy, target_type, confidence):
        """Call this from the perception bridge once that's wired up. Exposed
        directly for now so it can be triggered manually for testing."""
        self._next_task_id += 1
        msg = TargetDetected()
        msg.drone_id = self.drone_id
        msg.stamp = self.get_clock().now().to_msg()
        msg.target_id = self._next_task_id
        msg.position = Point(x=position_xy[0], y=position_xy[1], z=0.0)
        msg.target_type = target_type
        msg.confidence = confidence
        self.target_pub.publish(msg)
        self._on_target_detected(msg)  # handle our own detection immediately

    def _on_detect_target_service(self, request, response):
        """Test/tooling entry point for `report_target_detected`, callable
        from outside the process (e.g. `ros2 service call`). Goes through
        the same self-bid-then-broadcast path perception code would use —
        unlike publishing directly onto /coordination/target_detected from
        outside, which only reaches other drones, never this one."""
        self.report_target_detected(
            (request.position.x, request.position.y),
            request.target_type, request.confidence)
        response.target_id = self._next_task_id
        return response

    def _sync_state_with_bundle(self):
        """Keep `state` consistent with whether this drone currently holds
        any committed tasks. This is the return half of the state machine
        (see ARCHITECTURE.md's diagram) — without it, a drone that loses
        every task via consensus stays stuck in TASK_ALLOCATION forever with
        PSO paused (`_tick` only steps PSO while `state == SEARCH`).

        Originally this only handled the "lost via consensus" return path —
        a drone that actually WON a task stayed in TASK_ALLOCATION forever
        even after arriving, since nothing ever emptied its bundle (found
        live 2026-09-15: a drone's position froze the instant it won its
        first task and never moved again). Fixed 2026-09-29:
        _navigate_to_current_task() now calls cbba.mark_task_done() on
        arrival, which is what actually empties the bundle in that case —
        this method itself didn't need to change, it was always correct
        given an empty-or-not bundle, the bundle just never became empty on
        the "won" path before.
        """
        if self.state == STATE_RETURNING:
            return  # one-way: a drone at battery reserve never takes work again
        self.state = STATE_TASK_ALLOCATION if self.cbba.bundle else STATE_SEARCH

    def _on_target_detected(self, msg: TargetDetected):
        task = Task(
            task_id=msg.target_id,
            position=(msg.position.x, msg.position.y),
            target_type=msg.target_type,
            confidence=msg.confidence,
        )
        before = dict(self.cbba.winning_agent)
        self.cbba.add_task(task)
        self.get_logger().info(
            f'Drone {self.drone_id}: target {task.task_id} at '
            f'({task.position[0]:.1f}, {task.position[1]:.1f}) - my bid '
            f'{self.cbba.bid_for(task, self.pso.state.position):.3f}.')
        self.cbba.build_bundle(self.pso.state.position)
        self._log_winner_changes(before)
        self._sync_state_with_bundle()
        self._publish_bundle_state()

    def _log_winner_changes(self, before):
        """Say who holds each task whenever that changes - the only other
        way to see CBBA working is echoing the bundle_state topics."""
        for task_id, agent in self.cbba.winning_agent.items():
            if agent == before.get(task_id) or agent == UNASSIGNED:
                continue
            bid = self.cbba.winning_bids.get(task_id, 0.0)
            who = 'I win' if agent == self.drone_id else f'drone {agent} wins'
            self.get_logger().info(
                f'Drone {self.drone_id}: target {task_id} - {who} '
                f'(bid {bid:.3f}).')

    def _publish_bundle_state(self):
        msg = BundleState()
        msg.drone_id = self.drone_id
        msg.stamp = self.get_clock().now().to_msg()
        msg.known_task_ids = list(self.cbba.tasks.keys())
        msg.winning_bids = [
            self.cbba.winning_bids[t] for t in msg.known_task_ids]
        msg.winning_agent_ids = [
            self.cbba.winning_agent[t] for t in msg.known_task_ids]
        msg.update_times = [
            self.cbba.update_time[t] for t in msg.known_task_ids]
        msg.bundle = list(self.cbba.bundle)
        self.bundle_pub.publish(msg)

    def _on_bundle_state(self, msg: BundleState):
        # TODO: BundleState doesn't carry task metadata (position/type/
        # confidence), so a drone that hasn't seen the originating
        # TargetDetected yet can't evaluate tasks it only knows about via
        # consensus. Fine while every drone hears every TargetDetected
        # directly (small fleet, good comms); revisit if we need this to
        # survive a drone joining late or missing that broadcast.
        before = dict(self.cbba.winning_agent)
        changed = self.cbba.receive_bundle_state(
            sender_id=msg.drone_id,
            known_task_ids=list(msg.known_task_ids),
            winning_bids=list(msg.winning_bids),
            winning_agents=list(msg.winning_agent_ids),
            update_times=list(msg.update_times),
        )
        # Re-bid after every update, not only once the bundle is empty: a
        # drone outbid on one of several tasks releases the ones after it
        # too, and those are winnable again straight away.
        bundle_before = len(self.cbba.bundle)
        self.cbba.build_bundle(self.pso.state.position)
        changed = changed or len(self.cbba.bundle) > bundle_before
        self._log_winner_changes(before)
        self._sync_state_with_bundle()
        # Only republish on an actual change — otherwise two drones would
        # keep re-broadcasting at each other forever even after consensus
        # settles, which just wastes bandwidth for no benefit.
        if changed:
            self._publish_bundle_state()

    # ---- main loop ----------------------------------------------------------

    def _tick(self):
        now = self._now_s()
        self._own_xy = (self._real_position if self._real_position is not None
                        else self.pso.state.position)
        # Every drone searches wherever it flies, task or not.
        for x, y in [self._own_xy] + list(self.neighbor_position.values()):
            self.coverage.mark_seen(x, y, now)
        self._check_battery()

        if self.state == STATE_SEARCH:
            commanded_x, commanded_y = self.pso.step(
                dt=0.5, swarm_best_position=self._swarm_best(),
                neighbor_positions=list(self.neighbor_position.values()),
                real_position=self._real_position,
                candidates=self.coverage.sample_unexplored(
                    SEARCH_CANDIDATES, now))
            self._search_ticks += 1
            if self._search_ticks % SEARCH_LOG_EVERY_TICKS == 0:
                (x, y), (gx, gy) = self._own_xy, self.pso.state.best_position
                self.get_logger().info(
                    f'Drone {self.drone_id} searching: at ({x:.1f}, {y:.1f}), '
                    f'heading for ({gx:.1f}, {gy:.1f}), area explored '
                    f'{self.coverage.explored_fraction(now):.0%}'
                    + ('' if self._battery_remaining is None else
                       f', battery {self._battery_remaining:.0%}') + '.')
            if self.use_px4_offboard:
                self._commanded_xy = (commanded_x, commanded_y)
        elif self.state == STATE_TASK_ALLOCATION:
            self._navigate_to_current_task(dt=0.5)
        elif self.state == STATE_RETURNING:
            self._return_home(dt=0.5)
        self._enforce_collision_safety()
        self._publish_agent_state()

    def _check_battery(self):
        """Scale this drone's bids by battery (cbba.battery_value_scale),
        and at the reserve hand every task to the swarm and go home - a
        drone running low should give its work away while it can still
        say so, not fail halfway through it."""
        if self._battery_remaining is None:
            return
        self.cbba.value_scale = battery_value_scale(self._battery_remaining)
        if (self.state == STATE_RETURNING
                or self._battery_remaining > BATTERY_RESERVE):
            return
        released = self.cbba.release_all()
        self.state = STATE_RETURNING
        self.pso.state.best_position = self._home  # neighbors see where it's going
        self._publish_bundle_state()
        self.get_logger().warning(
            f'Drone {self.drone_id}: battery {self._battery_remaining:.0%} - '
            f'at reserve, handing off targets {released or "(none)"} and '
            f'returning home to land.')

    def _return_home(self, dt):
        tracked_x, tracked_y, commanded_x, commanded_y = step_toward(
            self.pso.state.position, self._home, dt,
            max_speed=self.pso.max_speed, arrival_radius=ARRIVAL_RADIUS_M,
            real_position=self._real_position,
            neighbor_positions=list(self.neighbor_position.values()),
            min_separation=(SETPOINT_AVOID_RADIUS_M if self.use_px4_offboard
                            else MIN_SEPARATION_M))
        self.pso.state.position = (tracked_x, tracked_y)
        if self.use_px4_offboard:
            self._commanded_xy = (commanded_x, commanded_y)
            if (math.dist(self.pso.state.position, self._home)
                    <= ARRIVAL_RADIUS_M and not self._landing_requested):
                self.get_logger().info(
                    f'Drone {self.drone_id}: home - landing.')
                self.request_landing(stay_up=True)

    def _enforce_collision_safety(self):
        """Hard floor, applied after every tick regardless of what produced
        the candidate position. See separation.py for why this exists
        separately from PSO's softer repulsion.

        With real telemetry, the reported position is ground truth and
        can't be teleported away from a neighbor, so the floor is applied to
        the *commanded* setpoint instead (separation.py's
        constrain_setpoint, with SETPOINT_MIN_SEPARATION_M's tracking
        margin), before _offboard_control_tick sends it. Neighbor
        positions are only comparable here because every drone reports in
        the same shared frame (see _px4_frame_origin). Telemetry-only mode
        (use_px4_position without offboard) commands nothing, so there is
        nothing to constrain.
        """
        if self._real_position is not None:
            if self.use_px4_offboard:
                self._commanded_xy = constrain_setpoint(
                    self._commanded_xy, self._real_position,
                    list(self.neighbor_position.values()),
                    SETPOINT_MIN_SEPARATION_M)
            return
        self.pso.state.position = enforce_min_separation(
            self.pso.state.position, list(self.neighbor_position.values()),
            MIN_SEPARATION_M)

    def _navigate_to_current_task(self, dt):
        """Fly toward the first task in this drone's CBBA bundle/path, and
        mark it done once actually arrived — closing the task-completion-
        lifecycle gap (see ARCHITECTURE.md): before this, a won task never
        left the bundle, so _sync_state_with_bundle() never had a reason to
        return to SEARCH, and a drone that WON a task (as opposed to losing
        one via consensus) was stuck in TASK_ALLOCATION forever even after
        there was nothing left to do.

        TODO: only handles the first task — doesn't chain through multiple
        committed tasks in bundle order yet.
        """
        if not self.cbba.path:
            return
        task_id = self.cbba.path[0]
        task = self.cbba.tasks.get(task_id)
        if task is None:
            return
        tracked_x, tracked_y, commanded_x, commanded_y = step_toward(
            self.pso.state.position, task.position, dt,
            max_speed=self.pso.max_speed, arrival_radius=ARRIVAL_RADIUS_M,
            real_position=self._real_position,
            neighbor_positions=list(self.neighbor_position.values()),
            min_separation=(SETPOINT_AVOID_RADIUS_M if self.use_px4_offboard
                            else MIN_SEPARATION_M))
        self.pso.state.position = (tracked_x, tracked_y)
        if self.use_px4_offboard:
            self._commanded_xy = (commanded_x, commanded_y)

        dx = task.position[0] - self.pso.state.position[0]
        dy = task.position[1] - self.pso.state.position[1]
        if math.hypot(dx, dy) <= ARRIVAL_RADIUS_M:
            self.cbba.mark_task_done(task_id)
            # Room in the bundle again: bid for anything still waiting
            # (e.g. a 4th target when the bundle holds at most 3).
            before = dict(self.cbba.winning_agent)
            self.cbba.build_bundle(self.pso.state.position)
            self._log_winner_changes(before)
            self._sync_state_with_bundle()
            self._publish_bundle_state()
            self.get_logger().info(
                f'Drone {self.drone_id}: reached target {task_id} - '
                + ('back to searching.' if self.state == STATE_SEARCH
                   else 'on to the next one.'))


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


if __name__ == '__main__':
    main()
