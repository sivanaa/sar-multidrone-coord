# Coordination layer architecture

Status: **in progress** — core structure and message design are in place;
bid function, fitness function, and real telemetry wiring are the open work.
Last updated 2026-09-15.

## Algorithm choice (from the project report)

Per the coordination-method survey (see report, "1st Report-SivanaTerron"),
this project uses a **hybrid, fully decentralized** approach, chosen to avoid
the single-point-of-failure risk of centralized coordination while still
converging on a good task split:

- **Particle Swarm Optimization (PSO)** drives the broad-area search phase —
  each drone is one particle, pulled toward its own best-found position and
  the best position known among its neighbors.
- **Consensus-Based Bundle Algorithm (CBBA)** drives task allocation once a
  target is detected — drones bid on investigating/confirming the target and
  resolve conflicts through local consensus, with no central arbiter.

Both are decentralized and local-communication-only, matching the SAR
requirement to keep working under degraded or partial connectivity.

## Per-drone state machine

One `coordination_node` runs per drone. Each instance is in exactly one of:

```
        target detected (own or neighbor's)
  SEARCH ─────────────────────────────────► TASK_ALLOCATION
    ▲                                              │
    │              bundle empty & no                │
    └────────────── tasks remain ───────────────────┘
```

- **SEARCH**: runs one PSO update per tick (0.5s), broadcasting position and
  personal-best via `AgentState`.
- **TASK_ALLOCATION**: runs CBBA bundle construction against all known tasks,
  broadcasting belief via `BundleState`, and updates on every neighbor
  broadcast via the consensus rule.

## Messages (`coordination_msgs`)

Custom messages live in their own `ament_cmake` package (`coordination_msgs`),
separate from `coordination_node` (`ament_python`) — ROS 2 requires
`rosidl`-generated interfaces to be built with `ament_cmake`, so this split is
structural, not optional.

| Message | Purpose | Published |
|---|---|---|
| `AgentState` | Position + PSO personal-best + current state | every tick (0.5s) |
| `TargetDetected` | A new target found by perception | once per detection |
| `BundleState` | This drone's CBBA belief + own bundle | on every task/consensus change |

## Topic layout

Each drone gets its own namespace, generic over `num_drones` (a node
parameter) so this works for any fleet size without code changes:

```
/drone_{id}/coordination/agent_state
/drone_{id}/coordination/target_detected
/drone_{id}/coordination/bundle_state
```

Every `coordination_node` subscribes to every *other* drone's three topics.
Real deployment target is N=2; prototype/validation target is N=5.

## Code layout

```
ros2_ws/src/
  coordination_msgs/          # ament_cmake, .msg definitions only
  coordination_node/
    coordination_node/
      pso.py                  # PSO particle update — real, working
      cbba.py                 # CBBA bundle + consensus — first draft
      coordination_node.py    # rclpy Node: state machine, pub/sub wiring
```

## What's a real first draft vs. what's still open

**Working now:**
- PSO update rule (inertia/cognitive/social terms, velocity clamping)
- CBBA bundle construction (greedy marginal-bid insertion) and a simplified
  consensus update rule
- Full pub/sub wiring and the SEARCH ↔ TASK_ALLOCATION state machine

**Explicitly deferred (flagged with `TODO` in code), matching the report's
own "next report" scope:**
- **Bid function** (`cbba.py: bid_for`) — currently just confidence /
  (1 + distance). Report flags this as needing real tuning once there's
  telemetry to test against.
- **PSO fitness function** (`coordination_node.py: make_exploration_fitness`)
  — currently rewards distance from each drone's own starting point (a crude
  "spread out" incentive) rather than any real search value. Comparing this
  across drones is a rough stand-in, not a shared quality measure. Should
  become the flood-risk-weighted scoring already used by the single-drone
  GPS routing (`edge-ai-gateway`), once that scoring is exposed. (Note: a
  flat/neutral placeholder was tried first and rejected during review — it
  silently froze every drone's personal-best at its starting position, which
  also froze the neighbor-pull term, so the swarm never moved at all.)
- **CBBA consensus rule** (`cbba.py: receive_bundle_state`) — implements the
  common cases from the paper's action table, not the complete table.
- **Task metadata propagation** — `BundleState` doesn't carry task details
  (position/type/confidence), so a drone only knows a task's *details* if it
  directly heard the originating `TargetDetected`. Fine for a small,
  well-connected fleet; would need fixing for late-joining drones or lossy
  comms.
- **Real position input** — `pso.py`/`coordination_node.py` currently use a
  local (x, y) the node manages itself, not PX4's actual telemetry. Wiring
  this to real position requires the Micro-XRCE-DDS-Agent bridge (built
  2026-09-14, not yet installed/running) so PX4's local position reaches
  ROS 2 as a topic this node can subscribe to.

## Next steps

1. Build `coordination_msgs` + `coordination_node` on the server (`colcon
   build`) and fix whatever comes up — this hasn't been build-tested yet.
2. Install and run the Micro-XRCE-DDS-Agent (built, not yet installed) so
   PX4 local position reaches ROS 2.
3. Replace the internally-managed (x, y) with real subscribed PX4 position.
4. Tune the bid function and fitness function against actual two-drone runs.
