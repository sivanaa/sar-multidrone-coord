"""Consensus-Based Bundle Algorithm (CBBA) for task allocation.

Reference: H.-L. Choi, L. Brunet, and J. P. How, "Consensus-Based
Decentralized Auctions for Robust Task Allocation," IEEE Transactions on
Robotics, vol. 25, no. 4, 2009.

This is a first working version, not the full algorithm from the paper:
bundle construction uses greedy marginal-bid insertion, and the bid function
is a simple inverse-distance heuristic. Both are meant to be tuned once we
have real telemetry and detections to test against — the project report
explicitly flags bid-function design as follow-up work once a prototype
exists.
"""

from dataclasses import dataclass
import math
import time

UNASSIGNED = 255  # sentinel drone_id meaning "no winner yet"


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
        self.path = []           # bundle re-ordered for execution (== bundle for now)

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

    def bid_for(self, task: Task, current_position):
        """First-draft bid function: higher bid = closer drone, scaled by
        detection confidence.

        TODO: fold in remaining battery and current bundle load once we have
        real telemetry to validate against (see report, section on bid
        function design).
        """
        dx = task.position[0] - current_position[0]
        dy = task.position[1] - current_position[1]
        distance = math.hypot(dx, dy)
        return task.confidence / (1.0 + distance)

    def build_bundle(self, current_position):
        """Greedy bundle construction: repeatedly add the task with the
        highest marginal bid this drone can currently win, until the bundle
        is full or no task is winnable."""
        while len(self.bundle) < self.max_bundle_size:
            best_task_id = None
            best_bid = 0.0
            for task_id, task in self.tasks.items():
                if task_id in self.bundle:
                    continue
                bid = self.bid_for(task, current_position)
                if bid > self.winning_bids.get(task_id, 0.0) and bid > best_bid:
                    best_task_id = task_id
                    best_bid = bid
            if best_task_id is None:
                break
            self.bundle.append(best_task_id)
            self.path.append(best_task_id)
            self.winning_bids[best_task_id] = best_bid
            self.winning_agent[best_task_id] = self.drone_id
            self.update_time[best_task_id] = time.time()
        return self.bundle

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
                    idx = self.bundle.index(task_id)
                    self.bundle = self.bundle[:idx]
                    self.path = self.path[:idx]
                    changed = True
        return changed
