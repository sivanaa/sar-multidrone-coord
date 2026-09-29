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
from coordination_node.cbba import CbbaAgent, Task
from coordination_node.navigate import step_toward
from coordination_node.separation import enforce_min_separation

MIN_SEPARATION_M = 1.5  # must match pso.py's default; see separation.py
ARRIVAL_RADIUS_M = 0.3  # must match navigate.py's step_toward default

STATE_SEARCH = AgentState.STATE_SEARCH
STATE_TASK_ALLOCATION = AgentState.STATE_TASK_ALLOCATION


def make_coverage_fitness(area_center, area_radius):
    """Search-value function: rewards being far from your nearest known
    neighbor (spreading out to cover more ground), with a penalty for
    straying outside the shared operating area.

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
        self.declare_parameter('num_drones', 2)
        self.declare_parameter('initial_x', 0.0)
        self.declare_parameter('initial_y', 0.0)
        # Soft bound for make_coverage_fitness's search-area penalty (see
        # its docstring) - a mission-level setting, not per-drone, so every
        # drone in a fleet should normally be launched with the same
        # values. Defaults are deliberately generic/small rather than tuned
        # to any one demo's coordinates (this node stays usable for any N
        # drones at any scale); override per deployment the same way
        # initial_x/initial_y already are.
        self.declare_parameter('area_center_x', 0.0)
        self.declare_parameter('area_center_y', 0.0)
        self.declare_parameter('area_radius', 10.0)
        self.declare_parameter('use_px4_position', False)
        # PX4 v1.18.0-beta1 (the version on the project's drone server, see
        # ARCHITECTURE.md) publishes this under a versioned topic name; the
        # message type itself is still px4_msgs/msg/VehicleLocalPosition.
        # Confirmed live via `ros2 topic list`/`ros2 topic type` 2026-09-15.
        self.declare_parameter('px4_local_position_topic', '/fmu/out/vehicle_local_position_v1')

        self.drone_id = self.get_parameter('drone_id').value
        self.num_drones = self.get_parameter('num_drones').value
        init_x = self.get_parameter('initial_x').value
        init_y = self.get_parameter('initial_y').value

        area_center = (
            self.get_parameter('area_center_x').value,
            self.get_parameter('area_center_y').value)
        area_radius = self.get_parameter('area_radius').value

        self.state = STATE_SEARCH
        self.pso = ParticleSwarmSearch(
            drone_id=self.drone_id,
            initial_position=(init_x, init_y),
            fitness_fn=make_coverage_fitness(area_center, area_radius),
        )
        self.cbba = CbbaAgent(drone_id=self.drone_id)

        # Real PX4 telemetry, when enabled, overrides PSO's internally
        # integrated position each tick (see pso.py's `step`). None until
        # the first message arrives, which pso.step() already treats the
        # same as "not using real position yet".
        self._real_position = None
        self.use_px4_position = self.get_parameter('use_px4_position').value
        if self.use_px4_position:
            self._setup_px4_position_subscription()

        self.neighbor_best = {}  # drone_id -> ((x, y), fitness)
        self.neighbor_position = {}  # drone_id -> (x, y), for separation only
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
            from px4_msgs.msg import VehicleLocalPosition
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

    def _on_px4_local_position(self, msg):
        self._real_position = (msg.x, msg.y)

    # ---- PSO / neighbor tracking -------------------------------------------

    def _on_agent_state(self, msg: AgentState):
        self.neighbor_best[msg.drone_id] = (
            (msg.best_position.x, msg.best_position.y), msg.best_fitness)
        self.neighbor_position[msg.drone_id] = (msg.position.x, msg.position.y)

    def _swarm_best(self):
        best_pos = self.pso.state.best_position
        best_fit = self.pso.state.best_fitness
        for pos, fit in self.neighbor_best.values():
            if fit > best_fit:
                best_pos, best_fit = pos, fit
        return best_pos

    def _publish_agent_state(self):
        msg = AgentState()
        msg.drone_id = self.drone_id
        msg.stamp = self.get_clock().now().to_msg()
        msg.position = Point(
            x=self.pso.state.position[0], y=self.pso.state.position[1], z=0.0)
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
        self.state = STATE_TASK_ALLOCATION if self.cbba.bundle else STATE_SEARCH

    def _on_target_detected(self, msg: TargetDetected):
        task = Task(
            task_id=msg.target_id,
            position=(msg.position.x, msg.position.y),
            target_type=msg.target_type,
            confidence=msg.confidence,
        )
        self.cbba.add_task(task)
        self.cbba.build_bundle(self.pso.state.position)
        self._sync_state_with_bundle()
        self._publish_bundle_state()

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
        changed = self.cbba.receive_bundle_state(
            sender_id=msg.drone_id,
            known_task_ids=list(msg.known_task_ids),
            winning_bids=list(msg.winning_bids),
            winning_agents=list(msg.winning_agent_ids),
            update_times=list(msg.update_times),
        )
        if not self.cbba.bundle and self.cbba.tasks:
            bundle_before = len(self.cbba.bundle)
            self.cbba.build_bundle(self.pso.state.position)
            changed = changed or len(self.cbba.bundle) > bundle_before
        self._sync_state_with_bundle()
        # Only republish on an actual change — otherwise two drones would
        # keep re-broadcasting at each other forever even after consensus
        # settles, which just wastes bandwidth for no benefit.
        if changed:
            self._publish_bundle_state()

    # ---- main loop ----------------------------------------------------------

    def _tick(self):
        if self.state == STATE_SEARCH:
            self.pso.step(dt=0.5, swarm_best_position=self._swarm_best(),
                          neighbor_positions=list(self.neighbor_position.values()),
                          real_position=self._real_position)
        elif self.state == STATE_TASK_ALLOCATION:
            self._navigate_to_current_task(dt=0.5)
        self._enforce_collision_safety()
        self._publish_agent_state()

    def _enforce_collision_safety(self):
        """Hard floor, applied after every tick regardless of what produced
        the candidate position. See separation.py for why this exists
        separately from PSO's softer repulsion.

        Skipped when real telemetry is active: a real vehicle's reported
        position is ground truth, not something we can teleport away from a
        neighbor — enforcing a real hard floor there means constraining the
        *commanded* setpoint before it's sent, which doesn't exist yet (see
        the offboard-command-loop item in ARCHITECTURE.md's next steps).
        """
        if self._real_position is not None:
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
        self.pso.state.position = step_toward(
            self.pso.state.position, task.position, dt,
            max_speed=self.pso.max_speed, arrival_radius=ARRIVAL_RADIUS_M,
            real_position=self._real_position,
            neighbor_positions=list(self.neighbor_position.values()),
            min_separation=MIN_SEPARATION_M)

        dx = task.position[0] - self.pso.state.position[0]
        dy = task.position[1] - self.pso.state.position[1]
        if math.hypot(dx, dy) <= ARRIVAL_RADIUS_M:
            self.cbba.mark_task_done(task_id)
            self._sync_state_with_bundle()
            self._publish_bundle_state()


def main(args=None):
    rclpy.init(args=args)
    node = CoordinationNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
