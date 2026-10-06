"""Consensus-Based Bundle Algorithm (CBBA) for task allocation.

Reference: H.-L. Choi, L. Brunet, and J. P. How, "Consensus-Based
Decentralized Auctions for Robust Task Allocation," IEEE Transactions on
Robotics, vol. 25, no. 4, 2009.

Bids follow the paper's time-discounted reward (2026-10-06): a path of
tasks is worth sum(confidence * DISCOUNT ** arrival_time), flying the path
in order at NOMINAL_SPEED_MPS from the drone's current position, and a bid
is the *marginal* value of adding a task at the best point in the drone's
path. So a drone that has already committed to work bids less for more of
it, and a target on the way to an existing one costs almost nothing.

The first draft bid confidence / (1 + distance from where the drone is
now), ignoring its path. A dry run of two drones on 300 random batches of
2-3 targets: one drone took every target in 130/300 batches, and 37
targets were left with no drone at all - when outbid, a drone dropped the
tasks queued after it (CBBA's release rule) but kept advertising its old
winning bids for them, so nobody else could ever win them. Both fixed
here; see the module's tests for the numbers after.

Still simplified vs. the paper: the consensus update below covers the
common cases of its action table, not all of them.
"""

from dataclasses import dataclass
import math
import time

UNASSIGNED = 255  # sentinel drone_id meaning "no winner yet"
# Value lost per second a target waits: 10%/s. Arrival times are estimated
# at NOMINAL_SPEED_MPS, roughly the vehicles' average in Gazebo runs.
DISCOUNT = 0.9
NOMINAL_SPEED_MPS = 2.0
# Battery (fraction 0-1): full-strength bids at or above BATTERY_FULL,
# scaled down linearly below it, nothing at or below BATTERY_RESERVE - the
# level at which a drone hands off its tasks and goes home (see
# coordination_node.py). BATTERY_FULL is 50% partly because PX4 SITL's
# simulated battery holds at 50% by default (SIM_BAT_MIN_PCT), so ordinary
# sim runs are unaffected.
BATTERY_FULL = 0.5
BATTERY_RESERVE = 0.25


def battery_value_scale(remaining):
    """How much of a task's value a drone with `remaining` battery can
    count on delivering: 1 when healthy, 0 at the reserve."""
    span = BATTERY_FULL - BATTERY_RESERVE
    return max(0.0, min(1.0, (remaining - BATTERY_RESERVE) / span))


@dataclass
class Task:
    task_id: int
    position: tuple
    target_type: str
    confidence: float


class CbbaAgent:
    """One drone's CBBA state: its bundle/path, and its belief about every
    task it currently knows about (winning bids/agents/timestamps)."""

    def __init__(self, drone_id, max_bundle_size=3):
        self.drone_id = drone_id
        self.max_bundle_size = max_bundle_size

        self.tasks = {}          # task_id -> Task
        self.bundle = []         # committed task_ids, insertion order
        self.path = []           # bundle in flying order (best insertion point)
        self.completed_task_ids = set()  # tasks actually finished, never re-picked

        self.winning_bids = {}   # task_id -> float
        self.winning_agent = {}  # task_id -> drone_id
        self.update_time = {}    # task_id -> seconds
        # Multiplies every path value, so every bid (set from battery by
        # the node, see battery_value_scale). Scaling the whole value keeps
        # bids comparable across drones and keeps CBBA's diminishing-
        # marginal-gain property that its convergence relies on.
        self.value_scale = 1.0

    def add_task(self, task: Task):
        """Register a newly detected target as an available task."""
        if task.task_id not in self.tasks:
            self.tasks[task.task_id] = task
            self.winning_bids.setdefault(task.task_id, 0.0)
            self.winning_agent.setdefault(task.task_id, UNASSIGNED)
            self.update_time.setdefault(task.task_id, time.time())

    def path_value(self, path, current_position):
        """Time-discounted value of flying `path` (task ids) in order."""
        value, elapsed, here = 0.0, 0.0, current_position
        for task_id in path:
            task = self.tasks[task_id]
            elapsed += math.dist(here, task.position) / NOMINAL_SPEED_MPS
            value += task.confidence * DISCOUNT ** elapsed
            here = task.position
        return self.value_scale * value

    def _best_insertion(self, task, current_position):
        """(marginal value, index) of adding `task` to the path at the
        position that loses the least time for everything else."""
        base = self.path_value(self.path, current_position)
        best_gain, best_index = -math.inf, 0
        for index in range(len(self.path) + 1):
            candidate = self.path[:index] + [task.task_id] + self.path[index:]
            gain = self.path_value(candidate, current_position) - base
            if gain > best_gain:
                best_gain, best_index = gain, index
        return best_gain, best_index

    def bid_for(self, task: Task, current_position):
        """What this drone bids for `task` right now: the value it adds to
        the drone's current path (see module docstring), scaled by battery
        (`value_scale`)."""
        if task.task_id in self.path:
            return self.winning_bids.get(task.task_id, 0.0)
        return self._best_insertion(task, current_position)[0]

    def mark_task_done(self, task_id):
        """Release a task this drone has actually finished (arrived at),
        as opposed to being outbid on (see receive_bundle_state's release
        rule for that case). Without this, a won task never leaves the
        bundle, coordination_node.py's _sync_state_with_bundle() never sees
        a reason to go back to SEARCH, and the drone is stuck "assigned"
        forever even after there's nothing left to do — confirmed live
        2026-09-15, a drone's position froze the instant it won its first
        task and never moved again. completed_task_ids makes the exclusion
        permanent so build_bundle() can't immediately re-pick the same task
        back up the very next tick (it would otherwise: this drone's own
        recorded winning bid for it is still the high one that just won)."""
        self.completed_task_ids.add(task_id)
        self.bundle = [t for t in self.bundle if t != task_id]
        self.path = [t for t in self.path if t != task_id]

    def build_bundle(self, current_position):
        """Greedy bundle construction: repeatedly add the task with the
        highest marginal bid this drone can currently win, until the bundle
        is full or no task is winnable."""
        while len(self.bundle) < self.max_bundle_size:
            best_task_id = None
            best_bid = 0.0
            best_index = 0
            for task_id, task in self.tasks.items():
                if task_id in self.bundle or task_id in self.completed_task_ids:
                    continue
                bid, index = self._best_insertion(task, current_position)
                if self._outbids(task_id, bid) and bid > best_bid:
                    best_task_id, best_bid, best_index = task_id, bid, index
            if best_task_id is None:
                break
            self.bundle.append(best_task_id)
            self.path.insert(best_index, best_task_id)
            self.winning_bids[best_task_id] = best_bid
            self.winning_agent[best_task_id] = self.drone_id
            self.update_time[best_task_id] = time.time()
        return self.bundle

    def _outbids(self, task_id, bid):
        """Whether `bid` beats the current winner. An exact tie goes to the
        lower drone id, so two drones with equal bids can't both hold on."""
        winning_bid = self.winning_bids.get(task_id, 0.0)
        if bid != winning_bid:
            return bid > winning_bid
        return self.drone_id < self.winning_agent.get(task_id, UNASSIGNED)

    def _release_from(self, index, outbid=True):
        """CBBA's release rule: drop bundle[index:] (the bid for each later
        task was computed assuming the earlier ones). When `outbid`, the
        task at `index` already carries its new winner; for the rest,
        withdraw our claim so other drones can bid on them - keeping it was
        what left targets with no drone at all."""
        released = self.bundle[index:]
        self.bundle = self.bundle[:index]
        self.path = [t for t in self.path if t in self.bundle]
        now = time.time()
        for task_id in released[1:] if outbid else released:
            if self.winning_agent.get(task_id) == self.drone_id:
                self.winning_bids[task_id] = 0.0
                self.winning_agent[task_id] = UNASSIGNED
                self.update_time[task_id] = now

    def release_all(self):
        """Give up every task and withdraw the claims, so the other drones
        win them on their next bundle build. Returns the released ids."""
        released = list(self.bundle)
        self._release_from(0, outbid=False)
        return released

    def _higher(self, bid_a, agent_a, bid_b, agent_b):
        """Whether (bid_a, agent_a) beats (bid_b, agent_b): higher bid,
        exact ties to the lower drone id - the same order every drone
        uses, so they can't disagree about a tie."""
        return bid_a > bid_b or (bid_a == bid_b and agent_a < agent_b)

    def receive_bundle_state(self, sender_id, known_task_ids, winning_bids,
                             winning_agents, update_times):
        """Consensus update against one neighbor's broadcast, following the
        paper's decision rules (Table 1) by who each side thinks is winning.

        The first version simply took whichever information was fresher
        (if it outbid ours or named someone else). When both drones claimed
        a new target at about the same moment, the *later* claim won even
        with the *lower* bid - found by a unit test 2026-10-06. When both
        claim it, the higher bid wins now, whatever the timestamps.
        Timestamps only decide between two reports about a third drone.

        Returns True if this update changed any local belief, so the caller
        can decide whether it's worth re-broadcasting.
        """
        changed = False
        me, sender = self.drone_id, sender_id
        for task_id, their_bid, their_agent, their_time in zip(
                known_task_ids, winning_bids, winning_agents, update_times):
            our_bid = self.winning_bids.get(task_id, 0.0)
            our_agent = self.winning_agent.get(task_id, UNASSIGNED)
            our_time = self.update_time.get(task_id, 0.0)
            fresher = their_time > our_time

            if their_agent == sender:      # sender claims it
                if our_agent == me:          # both claim it: higher bid wins
                    take = self._higher(their_bid, sender, our_bid, me)
                elif our_agent in (sender, UNASSIGNED):
                    take = True
                else:                        # we think a third drone has it
                    take = fresher or self._higher(
                        their_bid, sender, our_bid, our_agent)
                action = 'update' if take else 'leave'
            elif their_agent == me:        # sender thinks we have it
                action = 'reset' if our_agent == sender else 'leave'
            elif their_agent == UNASSIGNED:  # sender thinks nobody has it
                action = ('update' if our_agent == sender
                          or (our_agent not in (me, UNASSIGNED) and fresher)
                          else 'leave')
            else:                          # sender says a third drone has it
                action = ('update' if our_agent in (sender, UNASSIGNED)
                          or (our_agent == their_agent and fresher)
                          or (our_agent == me and self._higher(
                              their_bid, their_agent, our_bid, me))
                          or (our_agent not in (me, their_agent) and fresher)
                          else 'leave')

            if action == 'leave':
                continue
            if action == 'reset':
                their_bid, their_agent, their_time = 0.0, UNASSIGNED, time.time()
            if their_bid != our_bid or their_agent != our_agent:
                changed = True
            self.winning_bids[task_id] = their_bid
            self.winning_agent[task_id] = their_agent
            self.update_time[task_id] = max(their_time, our_time)
            if task_id in self.bundle and their_agent != me:
                # We've been outbid — release it and everything added
                # after it (CBBA's bundle "release" rule).
                self._release_from(self.bundle.index(task_id))
                changed = True
        return changed
