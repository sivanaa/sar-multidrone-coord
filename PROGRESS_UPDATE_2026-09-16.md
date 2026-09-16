# Multi-drone coordination — progress update (2026-09-16)

## Approach

Surveyed centralized, decentralized, and bio-inspired swarm coordination methods
(see the earlier report) and selected a **hybrid, fully decentralized** design:

- **Particle Swarm Optimization (PSO)** for broad-area search — each drone
  explores as one particle, pulled toward its own and its neighbors' best
  findings.
- **Consensus-Based Bundle Algorithm (CBBA)** for task allocation once a
  target is detected — drones bid on investigating it and resolve conflicts
  through local consensus, with no central point of failure.

This avoids the single-point-of-failure risk of centralized planning while
still converging on a coordinated task split — the right tradeoff for a SAR
scenario where connectivity to a ground station can't be guaranteed.

## Architecture

- One `coordination_node` runs per drone, in a two-phase state machine
  (`SEARCH` ↔ `TASK_ALLOCATION`).
- Three ROS 2 message types define drone-to-drone communication (position/
  status, target detections, consensus state), generic to any fleet size —
  validated at 2 drones, designed for N.
- Full write-up in `ARCHITECTURE.md`, including an explicit, honest list of
  what's implemented vs. what's next.

## What's been proven, live

- Built and validated the underlying simulation stack: multi-instance PX4
  SITL running in a shared Gazebo Harmonic world, plus a DDS bridge built to
  eventually carry real PX4 telemetry into ROS 2.
- Two real coordination nodes running simultaneously, exchanging real
  messages over real ROS 2 topics.
- A live test confirmed the full pipeline works end to end: PSO visibly
  explores over time, and a simulated target detection correctly triggers
  CBBA — the receiving drone computes a real bid and wins the task.
- Built a reproducible demo/validation tool (`demo_run.sh` +
  `visualize_run.py`) that runs a scripted two-drone scenario and produces a
  trajectory plot — not a one-off manual test, something repeatable.
- Live testing already did real engineering work, not just demonstration: it
  caught and fixed two genuine bugs (a PSO initialization deadlock, a missing
  state-machine transition) that code review alone hadn't surfaced.

## What the demo runs show — and what they don't yet

Multiple independent runs of the same scripted scenario (attached) all show
the same two findings — repeatable, not a one-off fluke. The clearest run
plots each drone's **distance to the detected target over time** alongside
the spatial trajectory:

- **Before winning a task**: distance genuinely oscillates (e.g. 1.6m to
  4.5m) as PSO explores — real search dynamics, moving toward and away from
  the target's area as it balances its own best-found position against the
  swarm's.
- **The instant a drone wins a task** (marked with a diamond): the line goes
  **flat** and never moves again for the rest of the run.

Two precise, fixable findings, not vague weaknesses:

1. **The two drones' search paths converge toward each other** instead of
   spreading to cover different area. Cause: each drone's search-fitness is
   scored relative to *its own* starting point, so the values aren't
   comparable across drones — the algorithm's "move toward whoever's doing
   best" rule ends up pulling drones toward each other's absolute position
   rather than toward complementary, uncovered ground. Fix planned: replace
   the placeholder fitness with a shared quality measure (the flood-risk-
   weighted scoring already used by the existing single-drone pipeline).
2. **Winning a task doesn't yet move a drone toward it** — the flat
   post-diamond line is direct visual proof. CBBA correctly decides *who's
   responsible* for a detected target, but nothing currently drives that
   drone to actually navigate there. Expected at this stage: coordination
   *logic* was the first milestone; coordination *action* (movement/control)
   is next, not yet built.

## Next steps

1. Finish confirming real PX4 telemetry is flowing into the coordination
   node (in progress).
2. Build the command loop — PX4 offboard control driven by PSO's output —
   tested on a single simulated drone before combining with coordination.
3. Once a drone can actually move under coordination control, close the loop
   so winning a task drives navigation toward it, not just responsibility.
4. Tune the bid function and search fitness against real multi-drone runs
   (intentionally deferred — the report itself scoped this as follow-up work
   once a working prototype existed, which it now does).
