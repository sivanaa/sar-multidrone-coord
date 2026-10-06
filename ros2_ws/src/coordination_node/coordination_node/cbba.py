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
        return value

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
        the drone's current path (see module docstring).

        TODO: fold in remaining battery once there is real telemetry for it
        (see report, section on bid function design).
        """
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

    def _release_from(self, index):
        """CBBA's release rule: drop bundle[index:] (the bid for each later
        task was computed assuming the earlier ones). The outbid task itself
        already carries the new winner; for the rest, withdraw our claim so
        other drones can bid on them - keeping it was what left targets with
        no drone at all."""
        released = self.bundle[index:]
        self.bundle = self.bundle[:index]
        self.path = [t for t in self.path if t in self.bundle]
        now = time.time()
        for task_id in released[1:]:
            if self.winning_agent.get(task_id) == self.drone_id:
                self.winning_bids[task_id] = 0.0
                self.winning_agent[task_id] = UNASSIGNED
                self.update_time[task_id] = now

    def receive_bundle_state(self, sender_id, known_task_ids, winning_bids,
                             winning_agents, update_times):
        """Consensus update against one neighbor's broadcast.

        Simplified version of the full action table in the paper (Table 1):
        a neighbor's information about a task wins if it is fresher AND
        (its bid is higher than ours, or someone other than us now holds it).
        TODO: implement the complete action table once we're validating
        against real multi-drone runs — this covers the common cases but not
        every consensus edge case from the paper.

        Returns True if this update changed any local belief, so the caller
        can decide whether it's worth re-broadcasting.
        """
        changed = False
        for task_id, their_bid, their_agent, their_time in zip(
                known_task_ids, winning_bids, winning_agents, update_times):
            our_time = self.update_time.get(task_id, 0.0)
            our_bid = self.winning_bids.get(task_id, 0.0)
            our_agent = self.winning_agent.get(task_id, UNASSIGNED)

            fresher = their_time > our_time
            better = their_bid > our_bid

            if fresher and (better or their_agent != self.drone_id):
                if their_bid != our_bid or their_agent != our_agent:
                    changed = True
                self.winning_bids[task_id] = their_bid
                self.winning_agent[task_id] = their_agent
                self.update_time[task_id] = their_time
                if task_id in self.bundle and their_agent != self.drone_id:
                    # We've been outbid — release it and everything added
                    # after it (CBBA's bundle "release" rule).
                    self._release_from(self.bundle.index(task_id))
                    changed = True
        return changed
