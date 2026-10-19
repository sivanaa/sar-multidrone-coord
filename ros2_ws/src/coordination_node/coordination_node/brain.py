"""One drone's coordination logic, without ROS: DroneBrain.

Everything a drone decides - PSO search over the coverage map, CBBA
bidding and consensus, battery hand-off, flying to tasks, the collision
floor - lives here. coordination_node.py is the ROS shell around it
(parameters, message conversion, the PX4/MAVLink link), and
tools/fleet_sim.py runs several brains against a simulated clock and
message bus. Both drive the same object, so whatever is measured in the
fleet simulator is what the real nodes fly (split out 2026-10-16; before,
this logic lived inside the ROS node and could only be exercised live).

A brain talks to the outside through two things it is given:
- `clock()`: seconds, the same clock its neighbours' stamps use;
- `outbox`: an object with publish_agent_state(dict),
  publish_target(dict), publish_bundle(dict), request_landing(stay_up) and
  log(level, text) - see the publish/snapshot methods for the dict keys,
  which mirror coordination_msgs' fields.
Incoming messages arrive as plain values through on_agent_state,
on_target_detected and on_bundle_state.
"""

import math

from coordination_node.pso import ParticleSwarmSearch
from coordination_node.cbba import (
    BATTERY_RESERVE, UNASSIGNED, CbbaAgent, Task, battery_value_scale)
from coordination_node.coverage import CoverageMap, path_clearance
from coordination_node.navigate import step_toward
from coordination_node.separation import (
    brake_limit, constrain_setpoint, enforce_min_separation, with_predictions)

# AgentState.STATE_* (coordination_msgs/msg/AgentState.msg); the node checks
# these still match.
STATE_SEARCH = 0
STATE_TASK_ALLOCATION = 1
STATE_RETURNING = 2

TICK_S = 0.5  # one decision step; the node's timer, the fleet sim's step

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
# Moving neighbors (separation.py with_predictions / brake_limit): avoid
# where a neighbor will be NEIGHBOR_LOOKAHEAD_S ahead as well as where it
# is, and close on it no faster than braking at BRAKE_DECEL_MPS2 allows.
# SETPOINT_LEAD_S is how far ahead navigate.py/pso.py put the setpoint
# (one 0.5s tick of velocity). Dry run of test-x with PX4 lag modeled to
# match the live 1.24m: head-on 1.18-1.37m -> 1.81m+, perpendicular
# 1.15m -> 1.78m+, hovering neighbor (test C) unchanged and still reached.
NEIGHBOR_LOOKAHEAD_S = 0.5
# Was 2.0 (Gazebo's measured accel p95 1.3-2.9 m/s2). AirSim's PX4 drones
# stop at ~1 m/s2 (0.87 -> 0.01 m/s in 0.9s), so two flying head-on
# overshot setpoints that kept 1.83m+ by ~0.5m and met at 1.38m - first
# live 3-drone AirSim run (2026-10-19), 4.5 min into a crowded search.
# Fleet simulator, 300s of continuous search over the 6m area, 20 runs:
# worst approach 1.60-1.64 -> 1.68-1.72m with AirSim-like vehicles and
# 1.60-1.73 -> 1.75-1.76m with Gazebo-like ones; search times unchanged.
BRAKE_DECEL_MPS2 = 1.0
SETPOINT_LEAD_S = 0.5
ARRIVAL_RADIUS_M = 0.3  # home (landing); must match navigate.py's default
# A person counts as reached once the drone is this close - well inside the
# nadir camera's ~1.6 x 2.85m footprint at 3m, enough to confirm or drop
# supplies. At 0.3m, two people 1.2m apart given to two drones deadlocked:
# each target lay inside the other drone's 1.8m setpoint floor, so both
# hovered 0.6-0.7m short for the rest of the mission (fleet simulator
# 2026-10-16, mission 52 of 60). At 1m they serve them from opposite sides.
TASK_ARRIVAL_RADIUS_M = 1.0

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
# Seconds until a searched cell is worth searching again (coverage.py;
# half that on the highest flood risk). Was 60: fine over the 6m-radius
# test area, which 2-4 drones cover in ~10-25s, but with fewer drones per
# area cells went stale before the fleet had covered everything once, so
# drones kept re-searching nearby/high-risk ground and never reached the
# far corners. Fleet simulator 2026-10-16, r=12m, 60 missions: missions
# with everyone reached 27/51/55/57 of 60 -> 60/59/60/60 (2 drones with
# a drone leaving + river, 2 drones with a drone leaving, 2 with river, 3
# with both), p90 time to reach all 150 -> 90s for 2 drones; medians
# unchanged; at r=6m identical in every metric. 120/180s fixed less;
# favouring never-searched cells (coverage.NEVER_SEEN_GAIN 3-10) fixed
# the tail too but slowed medians 20-90%. Alone it broke the small area
# instead (nothing due for 4 minutes after the first pass), fixed by
# coverage.MIN_DUE_FRACTION (2026-10-19).
REVISIT_AFTER_S = 240.0
# Every REBROADCAST_S, re-send this drone's bundle state and every open
# task it knows of. CBBA as first written only sent on a change, so a lost
# message ("I released target 3", "target 3 is done", the detection itself)
# was never repeated, and drones could each believe another one had a
# target (fleet simulator 2026-10-16: at 60% message loss 1 mission in 3
# left a target unreached). Receivers ignore what they already know, so a
# repeat costs bandwidth, not churn. 0 = only on change.
REBROADCAST_S = 1.0
# With each re-broadcast, raise the bids this drone holds to their value
# from where it is now (cbba.refresh_bids), so a drone flying to a target
# isn't outbid against the bid it won with, long since out of date.
# Fleet simulator 2026-10-16, 80 missions, 4 drones at 60% loss: median
# handovers per mission 5 -> 3, time to reach all p90 14.5 -> 11.5s, with
# CBBA_OUTBID_MARGIN 0.2 worst handovers 21 -> 15; 3 drones similar, no
# change without loss (little to fix there).
CBBA_REFRESH_BIDS = True
# CBBA settings the drones fly with (cbba.py has the plain-CBBA defaults,
# 0, for its own tests): merge reports of one person within
# CBBA_DUPLICATE_RADIUS_M, and take a task from another drone only with a
# bid CBBA_OUTBID_MARGIN higher. Fleet simulator 2026-10-16, 60 missions
# per cell, 3 drones, 4 targets: with REBROADCAST_S + these two, every
# target was reached in every mission at 0/40/60% message loss (plain:
# 60/50/42 of 60), people reached twice 9/25/43 -> 0/0/1, worst handovers
# per mission 3/1161/1462 -> 2/15/38, for ~2x the bundle messages (4
# drones the same). Re-broadcast does most of the completion work, merging
# removes the wasted trips, the margin halves the remaining handovers.
# The radius must sit between detection error (vision bridge reports
# 0.06-0.39m off) and how close two real people can be: 2.0m merged the
# pairs 1.4m apart in live test-bids (2026-10-16), so 2 of 4 people never
# got a task. The fleet simulator placed people >=2.5m apart with exact
# positions and couldn't show it; it now adds 0.15m detection noise and
# places people from 1.2m apart.
# Margin 0.1 -> 0.2 with CBBA_REFRESH_BIDS: the only setting with every
# target reached in all 80 missions at 3/4 drones, 0/60% loss (0.1 and 0
# each missed one), fewest handovers. Alone, 0.1-0.3 made no difference -
# the frozen held bid, not the margin, was the cause.
CBBA_DUPLICATE_RADIUS_M = 1.0
CBBA_OUTBID_MARGIN = 0.2


class DroneBrain:

    def __init__(self, drone_id, initial_xy, area_center, area_radius,
                 clock, outbox, sensor_radius=1.5, revisit_after_s=REVISIT_AFTER_S,
                 risk=None, task_types=None, real_vehicle=False,
                 footprint=None):
        """`real_vehicle`: positions come from a flight controller
        (real_position) and the brain produces setpoints (commanded_xy,
        setpoint()) instead of moving a simulated point itself.
        `footprint`: (half_north, half_east) of the camera's ground
        rectangle (coverage.camera_footprint) - None = sensor_radius
        circle."""
        self.drone_id = drone_id
        self.clock = clock
        self.outbox = outbox
        self.real_vehicle = real_vehicle

        self.state = STATE_SEARCH
        self.neighbor_best = {}  # drone_id -> ((x, y), fitness): its search goal
        self.neighbor_position = {}  # drone_id -> (x, y)
        self.neighbor_velocity = {}  # drone_id -> (vx, vy), from broadcasts
        self._neighbor_fix = {}  # drone_id -> ((x, y), stamp s) last used for velocity
        self.coverage = CoverageMap(
            area_center, area_radius, sensor_radius=sensor_radius,
            revisit_after_s=revisit_after_s, risk=risk, footprint=footprint)
        self.own_xy = tuple(initial_xy)  # where this drone is, for path checks
        self.home = tuple(initial_xy)  # where it goes to land on low battery
        self.battery_remaining = None  # 0-1 from PX4; None = unknown/no PX4
        self._search_ticks = 0
        self.pso = ParticleSwarmSearch(
            drone_id=drone_id, initial_position=tuple(initial_xy),
            fitness_fn=self._search_fitness)
        self.cbba = CbbaAgent(
            drone_id=drone_id, task_types=task_types,
            duplicate_radius=CBBA_DUPLICATE_RADIUS_M,
            outbid_margin=CBBA_OUTBID_MARGIN)

        # Real PX4 telemetry, when there is a real vehicle, overrides PSO's
        # internally integrated position each tick (see pso.py's `step`).
        # None until the first message arrives, which pso.step() already
        # treats the same as "not using real position yet".
        self.real_position = None
        # Setpoint for the real vehicle - starts at the drone's own initial
        # position (hold in place) until the first PSO/navigate step
        # computes a real commanded target.
        self.commanded_xy = tuple(initial_xy)
        self.landing_requested = False
        self._next_task_id = drone_id * 100000  # cheap collision-free id space
        self._last_rebroadcast = None

    def _log(self, text, level='info'):
        self.outbox.log(level, text)

    # ---- PSO / neighbor tracking -------------------------------------------

    def on_agent_state(self, drone_id, stamp, position, best_position,
                       best_fitness):
        """A neighbour's AgentState; stamp in seconds (its clock())."""
        self.neighbor_best[drone_id] = (tuple(best_position), best_fitness)
        position = tuple(position)
        previous = self._neighbor_fix.get(drone_id)
        velocity = (0.0, 0.0)
        if previous is not None and 0.05 <= stamp - previous[1] <= 1.0:
            dt = stamp - previous[1]
            velocity = ((position[0] - previous[0][0]) / dt,
                        (position[1] - previous[0][1]) / dt)
        if previous is None or stamp - previous[1] >= 0.05:
            # 10Hz broadcasts, plus the 2Hz tick's: skip the near-duplicate
            # rather than divide its tiny step by a tiny dt.
            self._neighbor_fix[drone_id] = (position, stamp)
            self.neighbor_velocity[drone_id] = velocity
        self.neighbor_position[drone_id] = position

    def floor_neighbors(self):
        """Neighbor positions for the real-vehicle collision floor and
        steering: where each one is, and where it will be shortly (see
        separation.py's with_predictions)."""
        return with_predictions(
            [(p, self.neighbor_velocity.get(d, (0.0, 0.0)))
             for d, p in self.neighbor_position.items()],
            NEIGHBOR_LOOKAHEAD_S)

    def steering_neighbors(self):
        """Neighbors for navigate.py's soft steering: predicted too when
        flying a real vehicle, as the floor is; simulated-only runs keep
        plain positions (that mode has no tracking lag to predict past)."""
        if self.real_vehicle:
            return self.floor_neighbors()
        return list(self.neighbor_position.values())

    def _search_fitness(self, x, y, neighbor_positions):
        """PSO fitness: unexplored ground a drone at (x, y) would see that no
        other drone is at or heading to (coverage.py), minus a penalty if
        getting there means crossing a neighbor's path to its own goal."""
        goals = [goal for goal, _ in self.neighbor_best.values()]
        value = self.coverage.fitness(
            x, y, self.clock(), claimed=list(neighbor_positions) + goals)
        for drone_id, (goal, _) in self.neighbor_best.items():
            start = self.neighbor_position.get(drone_id)
            if start is not None and path_clearance(
                    self.own_xy, (x, y), start, goal) < SEARCH_PATH_CLEARANCE_M:
                value -= PATH_CONFLICT_PENALTY
        return value

    def swarm_best(self):
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

    def position(self):
        """Where this drone is: real telemetry when there is some -
        pso.state.position only catches up with it once per tick."""
        return (self.real_position if self.real_position is not None
                else self.pso.state.position)

    def publish_agent_state(self):
        self.outbox.publish_agent_state({
            'drone_id': self.drone_id, 'stamp': self.clock(),
            'position': self.position(),
            'best_position': self.pso.state.best_position,
            'best_fitness': float(self.pso.state.best_fitness),
            'state': self.state})

    # ---- Target detection / CBBA -------------------------------------------

    def report_target_detected(self, position_xy, target_type, confidence):
        """This drone's perception found a target: announce it to the fleet
        and bid on it. Returns the new target id."""
        self._next_task_id += 1
        msg = {'drone_id': self.drone_id, 'stamp': self.clock(),
               'target_id': self._next_task_id,
               'position': tuple(position_xy), 'target_type': target_type,
               'confidence': confidence}
        self.outbox.publish_target(msg)
        self.on_target_detected(  # a node never hears its own broadcast
            msg['target_id'], msg['position'], target_type, confidence)
        return self._next_task_id

    def _sync_state_with_bundle(self):
        """Keep `state` consistent with whether this drone currently holds
        any committed tasks. This is the return half of the state machine
        (see ARCHITECTURE.md's diagram) - without it, a drone that loses
        every task via consensus stays stuck in TASK_ALLOCATION forever with
        PSO paused (`tick` only steps PSO while `state == SEARCH`).

        Originally this only handled the "lost via consensus" return path -
        a drone that actually WON a task stayed in TASK_ALLOCATION forever
        even after arriving, since nothing ever emptied its bundle (found
        live 2026-09-15). Fixed 2026-09-29: navigate_to_current_task() calls
        cbba.mark_task_done() on arrival, which is what empties the bundle
        in that case.
        """
        if self.state == STATE_RETURNING:
            return  # one-way: a drone at battery reserve never takes work again
        self.state = STATE_TASK_ALLOCATION if self.cbba.bundle else STATE_SEARCH

    def on_target_detected(self, target_id, position, target_type, confidence):
        if (target_id in self.cbba.tasks or target_id in self.cbba.merged
                or target_id in self.cbba.completed_task_ids):
            return  # already known - a re-broadcast (REBROADCAST_S)
        task = Task(task_id=target_id, position=tuple(position),
                    target_type=target_type, confidence=confidence)
        before = dict(self.cbba.winning_agent)
        if not self.cbba.add_task(task):
            self._log(f'Drone {self.drone_id}: target {target_id} at '
                      f'({task.position[0]:.1f}, {task.position[1]:.1f}) is '
                      'a duplicate of a known target - dropped.')
            return
        self._log(
            f'Drone {self.drone_id}: target {task.task_id} at '
            f'({task.position[0]:.1f}, {task.position[1]:.1f}) - my bid '
            f'{self.cbba.bid_for(task, self.pso.state.position):.3f}.')
        self.cbba.build_bundle(self.pso.state.position)
        self._log_winner_changes(before)
        self._sync_state_with_bundle()
        self.publish_bundle_state()

    def _log_winner_changes(self, before):
        """Say who holds each task whenever that changes - the only other
        way to see CBBA working is echoing the bundle_state topics."""
        for task_id, agent in self.cbba.winning_agent.items():
            if agent == before.get(task_id) or agent == UNASSIGNED:
                continue
            bid = self.cbba.winning_bids.get(task_id, 0.0)
            who = 'I win' if agent == self.drone_id else f'drone {agent} wins'
            self._log(f'Drone {self.drone_id}: target {task_id} - {who} '
                      f'(bid {bid:.3f}).')

    def publish_bundle_state(self):
        known = list(self.cbba.tasks.keys())
        self.outbox.publish_bundle({
            'drone_id': self.drone_id, 'stamp': self.clock(),
            'known_task_ids': known,
            'winning_bids': [self.cbba.winning_bids[t] for t in known],
            'winning_agent_ids': [self.cbba.winning_agent[t] for t in known],
            'update_times': [self.cbba.update_time[t] for t in known],
            'bundle': list(self.cbba.bundle),
            'completed_task_ids': sorted(self.cbba.completed_task_ids)})

    def on_bundle_state(self, sender_id, known_task_ids, winning_bids,
                        winning_agent_ids, update_times, completed_task_ids):
        # TODO: BundleState doesn't carry task metadata (position/type/
        # confidence), so a drone that hasn't seen the originating
        # TargetDetected yet can't evaluate tasks it only knows about via
        # consensus. Fine while every drone hears every TargetDetected
        # directly (small fleet, good comms); revisit if we need this to
        # survive a drone joining late or missing that broadcast.
        before = dict(self.cbba.winning_agent)
        done = self.cbba.learn_completed(list(completed_task_ids))
        if done:
            self._log(f'Drone {self.drone_id}: drone {sender_id} finished '
                      f'target(s) {done}.')
        changed = bool(done) | self.cbba.receive_bundle_state(
            sender_id=sender_id, known_task_ids=list(known_task_ids),
            winning_bids=list(winning_bids),
            winning_agents=list(winning_agent_ids),
            update_times=list(update_times))
        # Re-bid after every update, not only once the bundle is empty: a
        # drone outbid on one of several tasks releases the ones after it
        # too, and those are winnable again straight away.
        bundle_before = len(self.cbba.bundle)
        self.cbba.build_bundle(self.pso.state.position)
        changed = changed or len(self.cbba.bundle) > bundle_before
        self._log_winner_changes(before)
        self._sync_state_with_bundle()
        # Only republish on an actual change - otherwise two drones would
        # keep re-broadcasting at each other forever even after consensus
        # settles, which just wastes bandwidth for no benefit.
        if changed:
            self.publish_bundle_state()

    # ---- main loop ----------------------------------------------------------

    def tick(self, dt=TICK_S):
        now = self.clock()
        self.own_xy = self.position()
        # Every drone searches wherever it flies, task or not.
        for x, y in [self.own_xy] + list(self.neighbor_position.values()):
            self.coverage.mark_seen(x, y, now)
        self._check_battery()

        if self.state == STATE_SEARCH:
            commanded_x, commanded_y = self.pso.step(
                dt=dt, swarm_best_position=self.swarm_best(),
                neighbor_positions=list(self.neighbor_position.values()),
                real_position=self.real_position,
                candidates=self.coverage.sample_unexplored(
                    SEARCH_CANDIDATES, now, near=self.own_xy))
            self._search_ticks += 1
            if self._search_ticks % SEARCH_LOG_EVERY_TICKS == 0:
                (x, y), (gx, gy) = self.own_xy, self.pso.state.best_position
                self._log(
                    f'Drone {self.drone_id} searching: at ({x:.1f}, {y:.1f}), '
                    f'heading for ({gx:.1f}, {gy:.1f}), area explored '
                    f'{self.coverage.explored_fraction(now):.0%}'
                    + ('' if self.battery_remaining is None else
                       f', battery {self.battery_remaining:.0%}') + '.')
            if self.real_vehicle:
                self.commanded_xy = (commanded_x, commanded_y)
        elif self.state == STATE_TASK_ALLOCATION:
            self._navigate_to_current_task(dt)
        elif self.state == STATE_RETURNING:
            self._return_home(dt)
        self._enforce_collision_safety()
        self.publish_agent_state()
        if REBROADCAST_S > 0 and (self._last_rebroadcast is None
                                  or now - self._last_rebroadcast >= REBROADCAST_S):
            self._last_rebroadcast = now
            self._rebroadcast()

    def _rebroadcast(self):
        """See REBROADCAST_S and CBBA_REFRESH_BIDS."""
        if CBBA_REFRESH_BIDS and self.state != STATE_RETURNING:
            self.cbba.refresh_bids(self.pso.state.position)
        self.publish_bundle_state()
        for task in self.cbba.tasks.values():
            if task.task_id in self.cbba.completed_task_ids:
                continue
            self.outbox.publish_target({
                'drone_id': self.drone_id, 'stamp': self.clock(),
                'target_id': task.task_id, 'position': task.position,
                'target_type': task.target_type,
                'confidence': task.confidence})

    def _check_battery(self):
        """Scale this drone's bids by battery (cbba.battery_value_scale),
        and at the reserve hand every task to the swarm and go home - a
        drone running low should give its work away while it can still
        say so, not fail halfway through it."""
        if self.state == STATE_RETURNING:
            # Never bid again, whatever the battery says: PX4 SITL refills
            # its simulated battery to 100% once the drone disarms, and a
            # landed drone went straight back to winning targets
            # (2026-10-06). A real swap-and-relaunch would restart the node.
            self.cbba.value_scale = 0.0
            return
        if self.battery_remaining is None:
            return
        self.cbba.value_scale = battery_value_scale(self.battery_remaining)
        if self.battery_remaining > BATTERY_RESERVE:
            return
        released = self.cbba.release_all()
        self.state = STATE_RETURNING
        self.cbba.value_scale = 0.0
        self.pso.state.best_position = self.home  # neighbors see where it's going
        self.publish_bundle_state()
        self._log(
            f'Drone {self.drone_id}: battery {self.battery_remaining:.0%} - '
            f'at reserve, handing off targets {released or "(none)"} and '
            f'returning home to land.', 'warning')

    def _return_home(self, dt):
        tracked_x, tracked_y, commanded_x, commanded_y = step_toward(
            self.pso.state.position, self.home, dt,
            max_speed=self.pso.max_speed, arrival_radius=ARRIVAL_RADIUS_M,
            real_position=self.real_position,
            neighbor_positions=self.steering_neighbors(),
            min_separation=(SETPOINT_AVOID_RADIUS_M if self.real_vehicle
                            else MIN_SEPARATION_M))
        self.pso.state.position = (tracked_x, tracked_y)
        if self.real_vehicle:
            self.commanded_xy = (commanded_x, commanded_y)
            if (math.dist(self.pso.state.position, self.home)
                    <= ARRIVAL_RADIUS_M and not self.landing_requested):
                self._log(f'Drone {self.drone_id}: home - landing.')
                self.landing_requested = True
                self.outbox.request_landing(stay_up=True)

    def _enforce_collision_safety(self):
        """Hard floor, applied after every tick regardless of what produced
        the candidate position. See separation.py for why this exists
        separately from PSO's softer repulsion.

        With real telemetry, the reported position is ground truth and
        can't be teleported away from a neighbor, so the floor is applied to
        the *commanded* setpoint instead (separation.py's
        constrain_setpoint, with SETPOINT_MIN_SEPARATION_M's tracking
        margin), and again at the setpoint rate by setpoint(). Neighbor
        positions are only comparable here because every drone reports in
        the same shared frame. Telemetry without a real vehicle to command
        has nothing to constrain.
        """
        if self.real_position is not None:
            if self.real_vehicle:
                self.commanded_xy = constrain_setpoint(
                    self.commanded_xy, self.real_position,
                    self.floor_neighbors(), SETPOINT_MIN_SEPARATION_M)
            return
        self.pso.state.position = enforce_min_separation(
            self.pso.state.position, list(self.neighbor_position.values()),
            MIN_SEPARATION_M)

    def setpoint(self):
        """The setpoint to send the flight controller now, through the
        collision floor against the freshest neighbor positions - called at
        the full setpoint rate (10Hz), not just once per tick. With
        positions up to 0.5s stale, two drones closing at ~3 m/s met at
        0.92m in a live coverage-search run (2026-10-06); a dry run of the
        same search put 14/40 runs under 1.5m at 2Hz, 0/40 at 10Hz."""
        commanded = self.commanded_xy
        if self.real_position is not None:
            commanded = constrain_setpoint(
                commanded, self.real_position, self.floor_neighbors(),
                SETPOINT_MIN_SEPARATION_M)
            commanded = brake_limit(
                commanded, self.real_position,
                list(self.neighbor_position.values()),
                SETPOINT_MIN_SEPARATION_M, BRAKE_DECEL_MPS2, SETPOINT_LEAD_S)
        return commanded

    def _navigate_to_current_task(self, dt):
        """Fly toward the first task in this drone's CBBA path, and mark it
        done once actually arrived (cbba.mark_task_done), which is what
        lets the drone move on to the next task or back to searching."""
        if not self.cbba.path:
            return
        task_id = self.cbba.path[0]
        task = self.cbba.tasks.get(task_id)
        if task is None:
            return
        tracked_x, tracked_y, commanded_x, commanded_y = step_toward(
            self.pso.state.position, task.position, dt,
            max_speed=self.pso.max_speed, arrival_radius=TASK_ARRIVAL_RADIUS_M,
            real_position=self.real_position,
            neighbor_positions=self.steering_neighbors(),
            min_separation=(SETPOINT_AVOID_RADIUS_M if self.real_vehicle
                            else MIN_SEPARATION_M))
        self.pso.state.position = (tracked_x, tracked_y)
        if self.real_vehicle:
            self.commanded_xy = (commanded_x, commanded_y)

        dx = task.position[0] - self.pso.state.position[0]
        dy = task.position[1] - self.pso.state.position[1]
        if math.hypot(dx, dy) <= TASK_ARRIVAL_RADIUS_M:
            self.cbba.mark_task_done(task_id)
            # Room in the bundle again: bid for anything still waiting
            # (e.g. a 4th target when the bundle holds at most 3).
            before = dict(self.cbba.winning_agent)
            self.cbba.build_bundle(self.pso.state.position)
            self._log_winner_changes(before)
            self._sync_state_with_bundle()
            self.publish_bundle_state()
            self._log(
                f'Drone {self.drone_id}: reached target {task_id} - '
                + ('back to searching.' if self.state == STATE_SEARCH
                   else 'on to the next one.'))
