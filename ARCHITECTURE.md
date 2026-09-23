# Coordination layer architecture

Status: **in progress** — core structure and message design are in place and
now build/run-verified end-to-end (colcon build + live two-drone smoke test,
2026-09-15); bid function, fitness function, and real telemetry wiring are
the open work.
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
      navigate.py             # straight-line fly-to-task, once a task is won
      coordination_node.py    # rclpy Node: state machine, pub/sub wiring
```

## What's a real first draft vs. what's still open

**Working now:**
- PSO update rule (inertia/cognitive/social terms, velocity clamping, and a
  small random initial-velocity kick — without it every particle starts
  exactly at its own best-known position with zero velocity, which is a real
  deadlock, not just an unbiased start: nothing pulls a stationary particle
  anywhere until it has already moved. Found via live two-drone testing on
  2026-09-15, where drone 0 sat frozen at (0,0,0) for 50+ ticks straight.)
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
- **Navigation-to-target — fixed 2026-09-17.** CBBA decides *who is
  responsible* for a task; a new `navigate.py` module now handles actually
  flying there once a task is won: `coordination_node.py`'s `_tick()` calls
  `_navigate_to_current_task()` while in `TASK_ALLOCATION`, which moves in
  a straight line at up to PSO's `max_speed` toward the first task in the
  drone's CBBA `path`, decelerating smoothly and holding position once
  within `arrival_radius` (0.3m default) rather than overshooting. Verified
  with a local numeric dry-run (converges to the target in 4 ticks over a
  5m gap, no overshoot) before shipping.
  Still open: only handles the *first* task in the bundle — doesn't chain
  through multiple committed tasks in order, and doesn't mark a task done
  on arrival (ties into the task-completion-lifecycle gap below — arriving
  should eventually trigger that, once it exists).
- **Collision avoidance — added 2026-09-17.** Raised directly by a safety
  question: does the drones' observed path-convergence behavior (see the
  demo plots) mean they could actually collide? Answer at the time was yes —
  neither PSO, CBBA, nor `navigate.py` had any notion of other drones'
  physical positions; PSO's social term actively *pulls* particles toward
  each other, which is the opposite of avoidance. Two layers now address
  this:
  - **Soft**: `pso.py`'s `_separation_velocity` adds a repulsion term once a
    neighbor is within `min_separation` (1.5m default), growing the closer
    they get. Only active during SEARCH; a gradual nudge, not a guarantee.
  - **Hard**: `separation.py`'s `enforce_min_separation` is applied to the
    resulting position every tick, in both SEARCH and TASK_ALLOCATION
    (`coordination_node.py`'s `_enforce_collision_safety`, called from
    `_tick()` after either branch) — if two drones would end up closer than
    `MIN_SEPARATION_M`, one is pushed back out to exactly that distance. This
    is what actually closes the gap; the soft term just makes it rare for
    the hard floor to have to act.
  Explicitly **not** covered: real PX4-controlled flight — the hard floor
  currently adjusts a *simulated* position directly, which only makes sense
  while nothing is actually flying the vehicle. Once the offboard command
  loop exists (see "Next steps"), this needs to become a constraint on the
  *commanded* setpoint instead, not a position teleport — noted so this
  isn't mistaken for done once real flight starts. Also not covered: more
  than pairwise-sequential resolution (fine for N=2, the real deployment
  target; would need a proper multi-body solve for larger swarms).

  **Two more real gaps found via live testing, same day.** Added the
  min-separation readout to `visualize_run.py` specifically because a
  static trajectory plot can't show whether two drones were ever close *at
  the same time* — the first live run after adding the layers above
  measured drones passing **0.98m apart**, under the 1.5m floor, proving
  the two layers above weren't actually sufficient as first written:
  1. `navigate.py` (the TASK_ALLOCATION movement) had **zero neighbor
     awareness at all** — only the post-hoc hard floor was catching a
     fast (up to 3 m/s) direct approach, one tick too late. Fixed by giving
     `navigate.py` the same kind of proactive repulsion PSO already had.
  2. Pure radial repulsion (straight-line push-away) does nothing when a
     neighbor sits directly on the line to the target — the push-away and
     the pull-toward-target cancel along the same axis, with no sideways
     component to actually route around it. Confirmed via a local dry run:
     radial-only repulsion still passed 0.17m from a neighbor planted
     directly in the path. Fixed by adding a tangential ("go around")
     component alongside the radial one, always deflecting the same
     rotational way so the path curves smoothly instead of jittering.
     Re-verified via dry run: with both the tangential term and the hard
     floor chained together (exactly as `coordination_node.py` runs them
     every tick), a drone passing a **stationary** neighbor now holds
     exactly at the 1.5m floor instead of cutting inside it.

  **Residual, currently-unfixed risk**: the dry run above used a
  *stationary* neighbor. When both drones are moving, each one's avoidance
  math is based on the other's *last broadcast* position, which can be
  nearly a full 0.5s tick stale by the time it's acted on. If both are
  closing on each other during that window, the true simultaneous distance
  can still dip under the floor even though each side's own reasoning is
  internally consistent — this is why the live measurement above still
  showed a violation even with real navigation happening (not the
  zero-awareness bug — that run predates the navigate.py fix, but the
  staleness effect is separate and remains). This isn't a bug to patch, it's
  a fundamental limit of a 0.5s update tick at up to 3 m/s (worst-case
  possible closing distance within one tick, if both approach head-on, is
  up to 2 × 3.0 × 0.5 = 3m — bigger than the whole intended margin).
  Meaningfully closing this further needs either a much faster control
  loop, or velocity-aware (not just position-aware) prediction — bigger
  changes than fit this pass. Not silently declared solved; re-measure with
  the plot's min-separation readout after any further change here, don't
  assume it from the logic alone.

  **Re-measured after the navigate.py fix, 2026-09-17**: closest drones got
  was **1.50m** — exactly at the floor, not under it. One live run isn't
  proof the residual staleness risk above is gone (it's still real, by the
  math), but it's a real improvement over the first measurement (0.98m)
  and a reasonable point to pause this thread. Re-check with the plot's
  readout again before trusting this at higher speeds or more drones.
- **Task completion lifecycle** — nothing currently marks a *won* task as
  finished/investigated, so a drone that's actually winning tasks (not just
  losing them via consensus) never returns to SEARCH — its bundle just
  keeps holding whatever it won. The `TASK_ALLOCATION → SEARCH` return path
  itself is implemented (`_sync_state_with_bundle`, fires when the bundle
  empties via being outbid), but "empties because the task got done" isn't
  modeled yet — that needs real investigation/telemetry logic, not just
  consensus bookkeeping.
- **Real position input** — partially wired 2026-09-15, live-testing started
  but not yet passing. `pso.py`'s `step()` accepts an optional
  `real_position` override, and `coordination_node.py` has a
  `use_px4_position` parameter that, when set, subscribes to
  `px4_local_position_topic` (`px4_msgs/msg/VehicleLocalPosition`). This is
  **read-only telemetry only**: real position replaces the fitness/broadcast
  position each tick, but nothing yet commands PX4 to move (no offboard
  velocity/position setpoints, no arm/offboard-mode sequencing) — so with
  `use_px4_position` on, the drone's reported position tracks wherever it
  actually is, not PSO's pull, until a command loop is added later.
  `px4_msgs` is not on this repo (it's `.gitignore`d) — it's vendored
  directly into the **project's real drone server** at
  `~/sivana/sar-multidrone-coord/ros2_ws/src/px4_msgs` (pinned to PX4 commit
  `37e0cb3f6caa4ef31a84d6c9692d756f941e33ae`) and built together with
  `coordination_node`/`coordination_msgs` in that one workspace — no
  separate overlay needed there. If it's ever missing, the node logs a
  warning and falls back to the internally-simulated position instead of
  crashing (verified 2026-09-15).

  **The project's actual sim/hardware server is `uavintern@gpu` (SSH)**,
  under `~/sivana/`: `PX4-Autopilot` (real repo, `v1.18.0-beta1`, already
  built), `px4_sim` (pixi env with Gazebo Harmonic — `gz-sim8`/`gz-launch7`,
  **not AirSim**), `ros_ws` (pixi env with `ros-humble-desktop-full` via
  RoboStack — enter with `pixi shell`, then `unset VIRTUAL_ENV` per the
  manual testing procedure below), `Micro-XRCE-DDS-Agent` (built), and this
  repo. Launch sequence used 2026-09-15:
  ```
  # Terminal 1: PX4 SITL + Gazebo, headless (no display on this server)
  cd ~/sivana/px4_sim && pixi shell
  cd ~/sivana/PX4-Autopilot && HEADLESS=1 make px4_sitl gz_x500

  # Terminal 2: the DDS bridge
  ~/sivana/Micro-XRCE-DDS-Agent/build/MicroXRCEAgent udp4 -p 8888

  # Terminal 3: our node
  cd ~/sivana/ros_ws && pixi shell && unset VIRTUAL_ENV
  cd ~/sivana/sar-multidrone-coord/ros2_ws
  colcon build --symlink-install && source install/setup.bash
  ros2 run coordination_node coordination_node --ros-args \
    -p drone_id:=0 -p num_drones:=1 -p use_px4_position:=true
  ```
  **Found and fixed 2026-09-15**: this PX4 version publishes local position
  under a *versioned* topic name, `/fmu/out/vehicle_local_position_v1`, not
  the unversioned `/fmu/out/vehicle_local_position` originally hardcoded as
  this node's default (confirmed via `ros2 topic list` / `ros2 topic type`;
  the message type itself is unchanged, still `VehicleLocalPosition`). The
  default is now fixed to the `_v1` name.

  **Still open / not yet working**: even after pointing at the correct
  topic, one comparison of `agent_state.position` against a live
  `/fmu/out/vehicle_local_position_v1` reading showed values well outside
  that topic's tiny (grounded, motors-idle jitter) range — suggesting
  `real_position` was still `None` (internally-simulated fallback) rather
  than tracking telemetry, even though `coordination_node`'s startup log
  showed no `px4_msgs` warning and `ros2 topic info --verbose` showed
  matching QoS (`BEST_EFFORT` + `TRANSIENT_LOCAL` on both sides) — though
  that same check showed `Subscription count: 0`, and it's unconfirmed
  whether the node had already been stopped by that point in the session.
  **Next step**: reproduce cleanly — start the node in one terminal, and
  in a *separate* terminal (without touching the first) confirm
  `ros2 node list` shows it running and `ros2 node info /coordination_node`
  lists a subscription to exactly `/fmu/out/vehicle_local_position_v1` —
  then re-compare live values.

  Arming during this session failed with PX4's `commander check`: "No
  connection to the GCS" — expected, since there's no QGroundControl or
  other MAVLink heartbeat source running headless, and unrelated to the
  position-wiring work above (this step doesn't need the vehicle to
  actually fly, only to report real telemetry while grounded).

## Manual testing procedure

No automated tests exist yet — this is how the coordination logic has actually been
verified so far, by running real nodes and watching real topic traffic. All commands assume
the usual server setup (`pixi shell` in `ros_ws`, `unset VIRTUAL_ENV`, `source
install/setup.bash` from `ros2_ws`) in each terminal/tmux window.

**1. Launch two (or more) drone instances**, each in its own window, with distinct IDs and
starting positions so you can tell them apart:
```
ros2 run coordination_node coordination_node --ros-args -p drone_id:=0 -p num_drones:=2 -p initial_x:=0.0 -p initial_y:=0.0
ros2 run coordination_node coordination_node --ros-args -p drone_id:=1 -p num_drones:=2 -p initial_x:=5.0 -p initial_y:=5.0
```

**2. Watch PSO search behavior** — confirm a drone's position actually changes over time
(not frozen):
```
ros2 topic echo /drone_0/coordination/agent_state
```
Watch `position.x`/`position.y` change between consecutive messages, and `best_fitness`
becoming nonzero as it moves. If it stays at exactly `(0,0,0)` forever, something is wrong
with PSO's initialization or the fitness function — this is exactly how the zero-velocity
deadlock (2026-09-15) was caught.

**3. Trigger the CBBA phase, with every drone (including the detector) bidding** — call
the `detect_target` service on the drone that "sees" the target. This drives
`report_target_detected()`, the same entry point real perception code would call: it
self-bids inside that drone's own process *and* publishes `TargetDetected` so every other
drone's subscription fires too. One call is enough for the whole fleet to bid — no need to
fake multiple detections:
```
ros2 service call /drone_0/coordination/detect_target coordination_msgs/srv/DetectTarget "{position: {x: 3.0, y: 4.0, z: 0.0}, target_type: 'person', confidence: 0.9}"
```
The response's `target_id` is auto-assigned (drone 0's ids start at 0, drone 1's at
100000, etc. — see `_next_task_id` in `coordination_node.py`), not something you choose.
`demo_run.sh` does exactly this now. **Fixed 2026-09-23** — until then, the only way to
trigger a detection from outside the node was `ros2 topic pub` directly onto
`/drone_X/coordination/target_detected`, which a node never hears on its *own* topic (ROS2
nodes don't receive their own publications) and so never made the detecting drone bid on
its own find. The demo's earlier workaround was to publish the same task onto *both*
drones' topics to fake "both detected it" — that produced real competitive bidding for
exactly 2 drones, but doesn't generalize (at N drones you'd need N fake publishes for one
real detection) and doesn't match how a real detection actually happens (one drone sees
it, everyone bids). The `detect_target` service fixes both: it's the real single-detection
code path, and it scales to any fleet size for free since every other drone already
subscribes to the detecting drone's topic.

If you need to simulate a detection *without* the detecting drone's real node running at
all (e.g. testing drone 1's reaction in isolation) — a lower-level `ros2 topic pub` directly
onto the topic still works for that, it just won't produce a self-bid from the "detector"
since there isn't a real one:
```
ros2 topic pub --once /drone_1/coordination/target_detected coordination_msgs/msg/TargetDetected "{drone_id: 1, target_id: 43, position: {x: 3.0, y: 4.0, z: 0.0}, target_type: 'person', confidence: 0.9}"
```
Use a fresh, never-before-used `target_id` each time — reusing one that's already in a
drone's `tasks` dict won't exercise the "new task" path.

**Watching it, not just reading the end state**: `tools/visualize_run.py --gif-out
<path>.gif` (wired into `demo_run.sh` by default, alongside the existing PNG) renders the
whole run as a playable animation once the process stops, instead of only the periodic
PNG's cumulative "so far" snapshot. Added 2026-09-23 — a static plot doesn't actually show
*how* the drones moved, just where they ended up.

**Watching it live, while the run is still in progress**: `tools/viewer.html` is a small
static page (no server framework, no build step) that polls `/demo_run.png` every 2s with a
cache-busting query string — it just shows whatever `visualize_run.py`'s own periodic timer
has most recently saved. Serve the repo root with Python's built-in server and reach it from
your laptop over an SSH tunnel (the gpu server has no GUI, so a browser is the actual
"application" here):
```
# on the server, in its own window, started any time (independent of demo_run.sh):
cd ~/sivana/sar-multidrone-coord && python3 -m http.server 8000

# on the laptop, in its own window, kept open (this is the tunnel, it blocks):
ssh -N -L 8000:localhost:8000 uavintern@140.123.105.233 -p 42000
```
Then open `http://localhost:8000/ros2_ws/tools/viewer.html` in a laptop browser tab and
run `demo_run.sh` as usual in a third window — the page updates on its own as the run
progresses. Added 2026-09-23.

**4. Watch the reaction** — start this *before* step 3 so you don't miss the one-shot
reaction (topics are volatile/non-latched, no replay for late subscribers):
```
ros2 topic echo /drone_0/coordination/bundle_state
```
Check: does `known_task_ids` include the new task, is `winning_bids` a sensible non-zero
number, and does `bundle` include it if this drone should win? Also spot-check
`agent_state`'s `state` field flips to `1` (`TASK_ALLOCATION`).

**Known gap in this procedure**: because there's no task-completion lifecycle yet (see
below), a drone that wins a task never returns to `SEARCH` on its own — so testing the
`TASK_ALLOCATION → SEARCH` return path currently requires engineering an "outbid" scenario
(e.g. manually publishing a competing `BundleState` with a higher bid and a fresher
timestamp for the same task) rather than it happening naturally.

## Next steps

1. ~~Build `coordination_msgs` + `coordination_node` and fix whatever comes
   up.~~ Done 2026-09-15 (built/tested on a local WSL Ubuntu 22.04 + ROS 2
   Humble environment as a first correctness check, separate from the
   project's real drone server): both packages build clean. `colcon test`
   caught one real `flake8` continuation-indent bug in `cbba.py` (fixed);
   the remaining `pep257` failures are docstring convention nitpicks (D213
   vs. the Google-style docstrings actually used), not logic issues — left
   alone. A live two-drone run confirmed the full pipeline end-to-end: PSO
   position visibly moves tick-to-tick with nonzero `best_fitness` (no
   deadlock regression), and a fake `TargetDetected` on drone 1's topic
   correctly produced a `BundleState` on drone 0 with the task known, a
   nonzero winning bid, drone 0 in the bundle, and `agent_state.state`
   flipping to `TASK_ALLOCATION` — matching the manual testing procedure
   above exactly.
2. **In progress** — finish live-testing `use_px4_position` on the real
   drone server (`uavintern@gpu`), per the detailed notes and open item
   under "Real position input" above. Immediate next step: cleanly confirm
   (in a separate terminal from the one running the node) that
   `ros2 node info /coordination_node` actually shows a subscription to
   `/fmu/out/vehicle_local_position_v1`, then re-compare
   `agent_state.position` against live telemetry values.
3. Once real position is confirmed flowing, decide on and build the command
   loop (PX4 offboard velocity/position setpoints + arm/offboard-mode
   sequencing) so PSO's output actually drives the vehicle — currently nothing
   does. **Must carry collision safety with it**: today's hard floor
   (`separation.py`) teleports a simulated position, which stops making sense
   once something is actually flying — it needs to become a constraint on the
   commanded setpoint instead (see "Collision avoidance" above).
4. Tune the bid function and fitness function against actual two-drone runs.
