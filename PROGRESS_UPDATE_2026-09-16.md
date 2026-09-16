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

## What the current demo run shows — and what it doesn't yet

The latest run (attached/shown) shows both drones exploring and a target
detection correctly triggering a CBBA response. It also shows a precise,
useful limitation: winning a task currently only marks a drone as
*responsible* for it — nothing yet drives the drone to actually navigate
toward the target's position, since the fitness function driving movement
doesn't know about target locations at all. This is expected at this stage:
movement/control hasn't been built yet (see below) — coordination *logic* was
the first milestone, coordination *action* is next.

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
