# Coordination layer architecture

Status: **in progress** — core structure and message design are in place and
now build/run-verified end-to-end (colcon build + live two-drone smoke test,
2026-09-15); bid function, fitness function, and real telemetry wiring are
the open work.
Last updated 2026-10-05.

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
- **RETURNING** (added 2026-10-06): entered from either state once PX4's
  battery reaches the 25% reserve (`cbba.BATTERY_RESERVE`). The drone
  releases every task and withdraws its claims (so the others win them on
  their next bundle build), flies to its spawn point and lands; the node
  exits after commanding LAND. One-way — it never takes work again.

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
      coverage.py             # search fitness: recently-seen grid + path clearance
      offboard_sequence.py    # when to (re-)send OFFBOARD/ARM to PX4
      separation.py           # collision floors (simulated position / PX4 setpoint)
      cbba.py                 # CBBA: time-discounted marginal bids + consensus
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
- **Bid function** (`cbba.py: bid_for`) — **replaced 2026-10-06** with the
  CBBA paper's time-discounted reward: a drone's path of tasks is worth
  `sum(confidence * 0.9 ** arrival_seconds)` (arrival estimated at 2 m/s
  from its current position), and its bid for a new task is the value that
  task adds when inserted at the best point in the path. A drone already
  committed to work bids less for more, and a target on the way to an
  existing one is flown first (`path` is now the flying order, `bundle`
  the order tasks were won in). The first draft, confidence / (1 +
  distance from where the drone is now), ignored committed work. The same
  pass fixed a consensus bug: a drone outbid on one task released the
  tasks after it (CBBA's release rule) but kept advertising its old
  winning bids for them, so no other drone could ever win them
  (`_release_from` now withdraws the claim), and the node now re-bids
  after every consensus update, not only once its bundle is empty.
  Dry run, two drones' CBBA exchanging bids to agreement, 300 random
  batches of 2–3 targets:

  | | old bid | new bid |
  |---|---|---|
  | batches with a target never assigned | 32 | **0** |
  | allocation equal to the best possible (brute force) | 170/268 | **252/300** |
  | extra mean wait over the best possible | 0.27s | **0.07s** |

  Four targets at once in two clusters ((0,8)+(1,9), (8,0)+(9,1)), drones
  anywhere in the area, 200 runs: old left a target unassigned in 116,
  new in 0, with one cluster per drone in 171 (the rest reasonable 3/1
  splits). **Live (`sim.sh test-bids`, 2026-10-06)**: drone 0 won the red
  cluster (0.630/0.614 vs 0.523/0.506), drone 1 the green one (0.727/0.674
  vs 0.324/0.359), both agreed, each flew its pair back to back (drone 0
  in the shorter order, target 2 then 1) and resumed searching; drones no
  closer than 4.04m.

  **Battery (added 2026-10-06).** Each node reads PX4's
  `battery_status_v1` and multiplies its whole path value (so every bid)
  by `battery_value_scale`: 1 at or above 50%, falling linearly to 0 at
  the 25% reserve, where the drone switches to RETURNING (see the state
  machine). Scaling the whole value keeps bids comparable across drones
  and keeps CBBA's diminishing-marginal-gain property. 50% is the full
  point partly because PX4 SITL's simulated battery holds at 50% by
  default, so ordinary sim runs are unaffected. **Live, `sim.sh
  test-battery`** (drone 1 forced to 35%, then 20%, via PX4's
  `SIM_BAT_MIN_PCT`): at 35% drone 1 bid 0.360/0.336/0.207/0.206 for the
  four `test-bids` targets against drone 0's 0.699/0.736/0.391/0.422, so
  drone 0 took all four — including the green pair drone 1 wins at full
  battery (0.727/0.674 in `test-bids`) — the 4th via the re-bid after
  finishing a target. At 22% drone 1 logged "at reserve ... returning home
  to land", landed at its spawn, PX4 "Disarmed by landing"; drones no
  closer than 2.11m. It held no targets at that point, so the hand-off
  itself is exercised by `sim.sh test-handoff` (targets won at full
  battery, then reserve mid-flight). Its first run found two things: (1)
  fast-draining drone 1 from takeoff (`SIM_BAT_DRAIN 20`) made PX4's
  battery estimate undershoot its 50% hold (85 → 65 → 40 → 25% in flight,
  while drone 0 on the default drain sat at exactly 50%), so drone 1 hit
  reserve before any targets existed — the test now switches to fast
  drain only at the drop; (2) the 2D distance meter read 1.42m as drone 0
  passed 3m *above* the landed drone 1: its node had exited the moment it
  sent LAND, so its last broadcast was the in-flight point ~0.3m from
  where PX4 actually set it down. A drone landing on low battery now
  keeps its node running and keeps broadcasting its real position (Ctrl+C
  / `sim.sh down` still exit as before).

  **Consensus fix, same day.** The update rule took whichever report was
  fresher; when both drones claimed a new target at about the same moment,
  the later claim won even with the lower bid (found by a unit test). It
  now follows the paper's decision rules by who each side thinks is
  winning: when both claim a task, the higher bid wins (ties to the lower
  drone id) regardless of timestamps. Allocation dry runs unchanged. Also:
  a drone re-bids after finishing a target (room in its 3-task bundle), so
  a 4th waiting target isn't left until some other update happens.
- **PSO fitness function — replaced again 2026-10-06 by a coverage map**
  (`coverage.py`; see "Search: coverage map" below). The text in the rest of
  this bullet describes the previous version, `make_coverage_fitness`
  (2026-09-29), which is now only used by the single-drone tools scripts.
  It rewarded distance from your nearest known neighbor
  (spread out, don't duplicate search effort), softly bounded by a shared
  `area_center`/`area_radius`, rather than any real search value. This is
  still a placeholder, not the flood-risk-weighted scoring from the
  single-drone GPS routing (`edge-ai-gateway`) that should eventually
  replace it — but it fixed two concrete, found-live problems the previous
  version had. (History: a flat/neutral placeholder was tried first and
  rejected during review — it silently froze every drone's personal-best at
  its starting position, which also froze the neighbor-pull term, so the
  swarm never moved at all. The next version, distance-from-own-origin,
  fixed that but wasn't comparable across drones — each drone's value was
  relative to its own start — and its social-pull term actively fought
  `pso.py`'s own separation repulsion instead of cooperating with it. Found
  2026-09-23/24 via deliberately placing test targets near each drone's own
  start and watching the "wrong" drone win: distance-from-own-origin
  rewards a drone for moving AWAY from a target that happens to be near
  home, which is backwards. Nearest-neighbor spread fixes the comparability
  problem (same reference frame for every drone) and now cooperates with
  collision safety instead of fighting it, though it still has no notion of
  where a target is actually likely to be. **One more round the same day**:
  the first version of nearest-neighbor spread fell back to a flat constant
  when a drone had no neighbor data yet (e.g. before ROS2 discovery
  completes) — which is exactly the already-rejected flat-fitness bug from
  above, just scoped to "before the first neighbor message." Confirmed live
  2026-09-29: drone 0 sat frozen solid at its exact starting position for
  the ~27s an unusually slow discovery handshake took on that run, only
  unfreezing the instant real neighbor data arrived. Fixed by falling back
  to distance-from-area-center instead of a constant during that window —
  still varies smoothly with position (can't flatline), still the same
  reference frame for every drone (doesn't reintroduce the origin bug).)
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
  through multiple committed tasks in order.
  **Task-completion lifecycle — fixed 2026-09-29.** Arriving within
  `arrival_radius` now calls `cbba.mark_task_done()`, which removes the
  task from the bundle/path and adds it to a permanent
  `completed_task_ids` set (so `build_bundle()` can't immediately re-pick
  the same task back up — this drone's own recorded winning bid for it is
  still the high one that just won). An empty bundle is exactly what
  `_sync_state_with_bundle()` already used to decide SEARCH vs
  TASK_ALLOCATION, so no change was needed there — the bundle just never
  became empty on the "won" path before. This is distinct from being
  outbid (`receive_bundle_state`'s release rule): that's "someone else is
  better positioned," this is "the job here is actually finished."
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
  Real PX4-controlled flight is covered as of 2026-10-05 — see "Hard floor
  for real vehicles" below; until then the floor was skipped entirely
  whenever real telemetry was on. Still not covered: more than
  pairwise-sequential resolution (fine for N=2, the real deployment
  target; would need a proper multi-body solve for larger swarms).

  **Hard floor for real vehicles — added 2026-10-05.** A real vehicle's
  position is ground truth and can't be teleported, so with
  `use_px4_offboard` the floor now constrains the *commanded setpoint*
  instead (`separation.py: constrain_setpoint`, called from
  `_enforce_collision_safety` before `_offboard_control_tick` sends it).
  Three design points, all found the hard way:
  1. A setpoint inside a neighbor's circle is moved onto the circle near
     the side facing where the vehicle **actually is** — not along
     neighbor→setpoint like `enforce_min_separation`, which can land the
     point on the far side of the neighbor so the flight controller flies
     straight through it to get there. "Near" = the vehicle's own angle
     around the neighbor, rotated toward the setpoint's angle by at most
     20° per tick (`max_slide`). The first version (2026-10-05) used the
     vehicle's angle exactly, which threw away navigate.py's sideways
     "go around" step every tick and caused a head-on stall (Test C below).
     Both covered by `test/test_separation.py`.
  2. The setpoint floor is `SETPOINT_MIN_SEPARATION_M` = 1.8m, deliberately
     above `MIN_SEPARATION_M` (1.5m): commanding exactly 1.5m let the real
     distance wobble down to **1.40m** (PX4 tracking lag plus the 0.5s
     tick). With the margin, the real distance held at **min 1.73m / mean
     1.79m** over 30s of flight.
  3. On a real vehicle, navigate.py's soft avoidance starts at
     `SETPOINT_AVOID_RADIUS_M` = 2.5m, not 1.5m: at 1.5m it never acted at
     all, because the 1.8m setpoint floor already held the vehicle outside
     that radius. Simulated-only runs keep 1.5m. A larger radius alone did
     **not** fix the stall (dry run at 1.5/2.0/2.5/3.0m all stalled) — the
     slide in point 1 did; the larger radius just gets past a neighbor dead
     ahead faster (~8s vs ~24s in the dry run).
  Only works because every drone now reports position in the same shared
  frame (see "Shared coordinate frame" under the two-drone Gazebo section)
  — before that, neighbor positions weren't comparable at all.

  Confirmed live in Gazebo with two real PX4 vehicles (2026-10-05):
  - **Test A — spawned 1m apart** (`PX4_GZ_MODEL_POSE="1,0"`,
    `initial_x:=0.0 initial_y:=1.0` for drone 1): after takeoff they pushed
    apart to the floor and held it (numbers above).
  - **Test B — normal spawn + two targets**: red target won by the closer
    drone 0 (bid 0.365 vs 0.281) and reached to 0.05m; green target won by
    drone 1 (0.276 vs 0.131) and reached to 0.11m; closest drone-to-drone
    distance across the run **2.06m**; no stalls.
  - **Test C — head-on crossing (2026-10-06)**: drone 1 hovering at its
    spawn (5,5) via `commander takeoff` in its PX4 shell, its coordination
    node **not** running (it would search and bid instead of holding
    still); drone 0 told where it is by hand with
    `ros2 topic pub -r 2 /drone_1/coordination/agent_state coordination_msgs/msg/AgentState "{drone_id: 1, position: {x: 5.0, y: 5.0, z: 0.0}, best_position: {x: 5.0, y: 5.0, z: 0.0}, best_fitness: 0.0, state: 0}"`;
    `detect_target` on drone 0 at (10,10), so drone 1 sits dead on the
    straight path. **Old code: stalled** at (3.77, 3.75), ~1.75m from
    drone 1, indefinitely. **With the slide fix: went around** —
    (3.65, 4.01) → (3.26, 5.54) → (4.40, 6.97) — and reached the target at
    (9.92, 9.74); after completing it, crossed back past drone 1 on the
    other side, again without stalling. Closest across the whole run
    **1.68m** (matches the dry run's ~1.67–1.75m).
  Measure it with `ros2_ws/tools/drone_distance.sh` (live distance plus
  closest-so-far; occasionally prints one bogus line when `gz model`
  returns a bad pose read — two drones "swapping" spawn points for a single
  0.5s sample — ignore isolated jumps like that).

  **Known limits of the setpoint floor:**
  - ~~Possible head-on stall~~ — confirmed real and fixed 2026-10-06 (Test
    C above, design points 1 and 3). Only tested against a *hovering*
    neighbor; two drones crossing while both move is still untested.
  - ~~Drone returns to its spawn after a task~~ and ~~drones pinned
    together while searching~~ (Tests A and C) — one root cause, fixed
    2026-10-06, see "Search: coverage map" below.

  **Search: coverage map — 2026-10-06.** Root cause of both items above,
  confirmed by a dry run of the real `pso.py`: a personal best was scored
  once, when first visited, and never re-scored, but the fitness
  (nearest-neighbor spread) depends on where the neighbors are *now*.
  Drone 0's spawn scored 7.07 while drone 1 was still 7m away; nothing it
  found later beat that, so its spawn stayed its best, the cognitive pull
  dragged it back, and it froze there (every seed). It was also the swarm
  best, so the social pull dragged drone 1 onto it. Replaced with:
  - **Fitness = unexplored ground** (`coverage.py: CoverageMap`): each node
    keeps a 1m grid of when each cell in the area was last within
    `sensor_radius` (1.5m) of *any* drone, built from its own position and
    neighbors' AgentState — no new messages. A point is worth the stale
    cells a drone there would see; cells go stale after `revisit_after_s`
    (60s), so the search continues instead of ending after one pass.
  - **No doubling up**: cells near a neighbor's current position or its
    broadcast goal (`best_position`) don't count, and a goal whose path
    passes within 2.5m of a neighbor's path to its goal loses 10 points
    (`coverage.py: path_clearance`).
  - **PSO** (`pso.py: _rescore_personal_best`) re-scores its personal best
    every tick and is offered 8 random unexplored cells as candidate goals
    (the map can score a point without visiting it); a candidate replaces
    the current goal only if clearly better after a 0.3/m distance cost,
    so the goal doesn't flip-flop. The swarm best is re-scored the same
    way; since a neighbor's goal is claimed, a drone's own goal usually
    wins, so the drones cooperate through the shared map and claims rather
    than PSO's social pull.
  - **Dry runs** (two drones, area r=6m at (4,4), PX4 tracking modeled):
    drones split up at once, 95% covered in ~15s, then revisit. Separation
    is the open cost: more motion means more crossings, and with neighbor
    positions up to 0.5s stale, 4/40 three-minute runs dipped under 1.5m
    (worst 1.37m) even with the path check (12/40 without it). Shorter
    revisit times keep drones moving more but triple that (30s: 12/40).
    Fully simulated mode (no PX4) is worse — no tracking lag to absorb the
    staleness; worst 0.2–0.7m — so treat its separation numbers as
    meaningless for this search. Measure live with `drone_distance.sh`.
  - **Live, 2026-10-06 (`sim.sh repeat 3`)**: search and bidding worked in
    all 3 runs (closer drone won every target, both drones agreed, winner
    reached it and resumed searching). Closest distances 2.44m and 3.06m —
    and **0.92m** in the first run: ~2.5 min in, both drones picked goals
    next to each other within the same few seconds (each scored against
    the other's *previous* goal, so the path check didn't see it), and
    the setpoint floor, working from neighbor positions up to 0.5s old,
    couldn't stop ~3 m/s of closing speed in time. **Fixed the same day**:
    each node now broadcasts its real position and re-applies the setpoint
    floor at the 10Hz setpoint rate (`_publish_offboard_setpoint`), not
    just once per 0.5s tick. Dry run with 0.1s-step PX4 tracking: 14/40
    three-minute runs under 1.5m at 2Hz (worst 1.12m), **0/40 at 10Hz
    (worst 1.61m)**. **Confirmed live** (`sim.sh repeat 5`, same day):
    closest 2.78 / 2.48 / 2.74 / **1.54** / 2.72m — all above the floor;
    the 1.54m was a brief pass while both drones moved the same way, then
    held at ~1.9m (the 1.8m setpoint floor). Every run: both drones armed
    without help, both targets reached, search resumed.
  - **Logs**: each node now prints its search progress every 5s (`searching:
    at ..., heading for ..., area explored N%`), its bid for every new
    target, every change of winner, and `reached target N - back to
    searching`.
  - Still not target-aware: every unexplored cell is worth the same. The
    flood-risk-weighted scoring from the single-drone GPS routing would
    slot in as a per-cell weight in `CoverageMap`.

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

  **Multi-drone topic namespacing — resolved 2026-09-30, turned out to need
  no work at all.** This was flagged as an open question for weeks (both
  drones' `/fmu/in|out/...` topics looked unnamespaced, seemingly guaranteed
  to collide once a second real vehicle was bridged). Tested directly:
  launched a second PX4 SITL instance (`-i 1`) alongside the first, both
  connected to the same already-running Micro-XRCE-DDS Agent, and diffed
  `ros2 topic list`. Result: PX4's own uXRCE-DDS bridge already auto-
  namespaces every instance after the first under `/px4_{instance}/fmu/...`
  — instance 0 keeps the plain, unprefixed `/fmu/...` topics, instance 1's
  entire topic set appeared under `/px4_1/fmu/...`, with zero manual
  configuration. `coordination_node.py`'s `px4_local_position_topic`
  parameter default is now computed from `drone_id` (assumed to match the
  PX4 instance's own `-i N` flag, same convention `demo_run.sh` already uses
  pairing `drone_id` with `initial_x`/`initial_y`) instead of being
  hardcoded to the single-drone case.

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

**Fixed 2026-09-29** — a drone that wins a task now returns to `SEARCH` on its own once it
actually arrives (see the task-completion-lifecycle note under "Navigation-to-target"
above), so the `TASK_ALLOCATION → SEARCH` return path is exercised naturally just by
letting a run continue past arrival — watch `agent_state`'s `state` field flip back to `0`
a few ticks after `bundle_state`'s `bundle` empties out. The "outbid" scenario (manually
publishing a competing `BundleState` with a higher bid and a fresher timestamp) still
exists as a way to test the *other* return path specifically, if you want to isolate it.

## Running the two-drone PX4 + Gazebo simulation (with the 3D view)

The full recipe for two real PX4 SITL vehicles flown by `coordination_node`
(`use_px4_offboard:=true`), confirmed end to end 2026-10-05 on the lab's
Windows + WSL Ubuntu-22.04 machine with the Gazebo GUI. Paths assume a WSL
home with `PX4-Autopilot`, `sar-multidrone-coord` and a v2.4.x
`Micro-XRCE-DDS-Agent` build; on that machine the agent is
`~/Micro-XRCE-DDS-Agent-243` (the plain `~/Micro-XRCE-DDS-Agent` there is an
old, incompatible v1.4.2 — always launch the agent by full path). The
`launch_*.sh` / `run_coord*.sh` scripts in the repo root are the same steps
in tmux form, but hardcode `/home/sivan` and the v1.4.2 agent path, so they
need editing before use on another machine.

### Shared coordinate frame — fixed 2026-10-05

Every PX4 instance reports `VehicleLocalPosition` and accepts
`TrajectorySetpoint` relative to **its own spawn point**. The node used to
treat those as one shared frame, so drone 1 (spawned at 5,5) believed it was
near the origin: its CBBA bids used wrong distances, the same target
coordinate meant a different physical spot for each drone, and its startup
setpoint pushed it toward (10,10). Now everything inside the node (PSO,
CBBA, navigation, neighbor positions, target coordinates) lives in one
shared frame — **drone 0's PX4 local NED frame** (x = north, y = east,
origin at drone 0's spawn) — and only the PX4 boundary converts
(`_px4_frame_origin` in `coordination_node.py`: add `initial_x/initial_y` on
telemetry in, subtract on setpoints out).

So `initial_x`/`initial_y` must be each drone's spawn position **in that
shared NED frame**. Gazebo's world frame is ENU (x = east, y = north), so
the axes swap:

| Gazebo / `PX4_GZ_MODEL_POSE` | Shared frame (`initial_x`, `initial_y`, target coords) |
|---|---|
| `"gx,gy"` | `(gy, gx)` |
| `"5,5"` | `(5, 5)` — symmetric, which is why the demo values didn't change |
| target marker at Gazebo (4, 3) | target `{x: 3.0, y: 4.0}` |

Confirmed live by dropping visible markers at each target (see below):
target (3,4) was won by the closer drone 0 (bid 0.427), which flew to the
marker at Gazebo (4,3) while drone 1 held at (5,5); target (6,7) was won by
drone 1, which flew to Gazebo (7,6). Assumes PX4's EKF origin is the spawn
point — true in SITL; real hardware will need the offset derived from
`VehicleLocalPosition.ref_lat/ref_lon` instead of a launch parameter.

### Launch sequence

**Shortcut (added 2026-10-06):** `bash ros2_ws/tools/sim.sh test-b` (or
`test-c`, or `up` + `nodes` for a free-form run) does every step below in
one tmux session named `sim` — Gazebo, both PX4s with the per-launch
settings typed in, both agents, the nodes, the distance monitor, markers
and detections. `bash ros2_ws/tools/sim.sh attach` to watch (Ctrl+b w picks
a window, Ctrl+b d leaves it running), `bash ros2_ws/tools/sim.sh down` to
stop — `down` first saves every window's full log to
`~/sim_runs/<time>_<test>/` and prints the run's closest drone-to-drone
distance (also appended to `~/sim_runs/summary.txt`). `sim.sh repeat N`
runs `test-search` N times headless and lists those distances, for
measuring separation across runs. Run `bash ros2_ws/tools/sim.sh` with no
arguments for the full list.
The manual steps stay documented here for when something needs debugging.

One terminal per step (on Windows: `wsl -d Ubuntu-22.04` first in each).
Order matters — the shared Gazebo world must be up **before** any PX4
instance, otherwise each instance spawns its own isolated world.

```bash
# 0. Clean slate (closing windows / Ctrl+C does NOT stop the backgrounded agent)
pkill -f "gz sim"; pkill -f MicroXRCEAgent; pkill -f "px4_sitl_default/bin/px4"; pkill -f coordination_node

# 1. Shared Gazebo world, with the 3D window (add --headless for no GUI)
cd ~/PX4-Autopilot
export GZ_SIM_SYSTEM_PLUGIN_PATH="$HOME/PX4-Autopilot/build/px4_sitl_default/src/modules/simulation/gz_plugins"
python3 Tools/simulation/gz/simulation-gazebo

# 2. PX4 instance 0 (wait for the Gazebo window first)
cd ~/PX4-Autopilot
PX4_SYS_AUTOSTART=4001 PX4_SIMULATOR=gz PX4_GZ_MODEL_POSE="0,0" PX4_GZ_MODEL=x500 ./build/px4_sitl_default/bin/px4 -i 0

# 3. PX4 instance 1 (own DDS port)
cd ~/PX4-Autopilot
PX4_UXRCE_DDS_PORT=8889 PX4_SYS_AUTOSTART=4001 PX4_SIMULATOR=gz PX4_GZ_MODEL_POSE="5,5" PX4_GZ_MODEL=x500 ./build/px4_sitl_default/bin/px4 -i 1

# 4. One DDS agent per instance (this terminal is then occupied - don't type other commands into it)
~/Micro-XRCE-DDS-Agent-243/build/MicroXRCEAgent udp4 -p 8888 &
~/Micro-XRCE-DDS-Agent-243/build/MicroXRCEAgent udp4 -p 8889

# 5. One coordination node per drone (drone_id must match the PX4 -i N)
cd ~/sar-multidrone-coord/ros2_ws && source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 run coordination_node coordination_node --ros-args -p drone_id:=0 -p num_drones:=2 -p initial_x:=0.0 -p initial_y:=0.0 -p use_px4_offboard:=true -p hold_altitude:=3.0 -p area_center_x:=4.0 -p area_center_y:=4.0 -p area_radius:=6.0
ros2 run coordination_node coordination_node --ros-args -p drone_id:=1 -p num_drones:=2 -p initial_x:=5.0 -p initial_y:=5.0 -p use_px4_offboard:=true -p hold_altitude:=3.0 -p area_center_x:=4.0 -p area_center_y:=4.0 -p area_radius:=6.0

# 6. Detection (any free, sourced terminal)
ros2 service call /drone_0/coordination/detect_target coordination_msgs/srv/DetectTarget "{position: {x: 3.0, y: 4.0, z: 0.0}, target_type: 'person', confidence: 0.9}"
```

**Required in each PX4 shell after every launch** (runtime-only, not saved
with `param save`; see "Next steps" item 3 for why):
```
param set NAV_DLL_ACT 0
sensor_baro_sim start
```
Type both lines before starting that drone's coordination node. If PX4
isn't ready yet it logs `Arming denied: Resolve system health failures
first` — since 2026-10-06 the node just asks again every 3s until PX4's
`VehicleStatus` says armed and in OFFBOARD (see "Next steps" item 5), so
`commander arm` by hand is no longer needed.

Each node should log `Real PX4 offboard control enabled for drone N`, then
`requested OFFBOARD mode` / `requested ARM` (MAVLink, attempt 1, 2, ...)
and finally `PX4 confirms armed in OFFBOARD`; each PX4 shell logs
`Armed by external command` and `Takeoff detected`. Attempts that note
`no VehicleStatus from PX4 yet` mean that drone's DDS agent isn't
connected — the retries then never stop on their own. A
`falling back to simulated-only control` warning means `pymavlink`
(`pip3 install --user pymavlink`) or `px4_msgs` is missing — the drones then
won't move. Stop with Ctrl+C in the node terminals first (the node commands
a MAVLink land on SIGINT), then step 0's cleanup line.

### Watching what happens

- **Targets aren't Gazebo objects** — a detection is just a coordinate, so
  nothing appears in the 3D view on its own. Drop a visual-only marker
  (no collision) at the target's **Gazebo** position, i.e. with x/y swapped
  from the target coordinate (change `name`, `<pose>` and the colour per
  marker):
  ```bash
  gz service -s /world/default/create --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 3000 --req 'sdf: "<?xml version=\"1.0\"?><sdf version=\"1.9\"><model name=\"target_d0\"><static>true</static><pose>4 3 0.5 0 0 0</pose><link name=\"l\"><visual name=\"v\"><geometry><cylinder><radius>0.3</radius><length>1</length></cylinder></geometry><material><ambient>1 0 0 1</ambient><diffuse>1 0 0 1</diffuse></material></visual></link></model></sdf>"'
  ```
- **A drone's real position, in Gazebo coordinates** (the raw command's
  output gets buried under SDF warnings, hence the filter):
  ```bash
  watch -n 1 "gz model -m x500_1 -p 2>/dev/null | grep -A1 XYZ"
  ```
  In the Gazebo GUI, right-click a model in the Entity Tree → *Move to* /
  *Follow* to find it.
- **Distance between the two drones**, live, plus the closest they've come
  (for checking the collision floor):
  ```bash
  bash ~/sar-multidrone-coord/ros2_ws/tools/drone_distance.sh
  ```
- **Who bid and who won** — the node logs nothing about CBBA to its own
  terminal; it's only on the `bundle_state` topics, which are published on
  change and not latched, so start this **before** triggering the detection:
  ```bash
  ros2 topic echo /drone_1/coordination/bundle_state
  ```
  `winning_agent_ids` is the believed winner per task (index-aligned with
  `known_task_ids`); the winner's `bundle` contains the task, the loser's is
  `[]`, and the winner's empties again on arrival. Bids are
  `confidence / (1 + distance_m)`, so higher is better and the ceiling is
  the detection confidence itself: with confidence 0.9, a bid of 0.427 means
  ~1.1 m away, 0.1 means ~8 m.

### Known noise (harmless)

- `ERROR [vehicle_imu] ... timestamp error` in the PX4 shells — late IMU
  samples when the CPU-rendered Gazebo GUI (WSLg has no GPU path) stutters.
  Gazebo's real-time factor stayed ~1.0 and both vehicles armed and flew
  normally with it scrolling. Only worth investigating if `commander check`
  shows a `Preflight Fail`.
- `NodeShared::Publish() Error: Interrupted system call` and
  `gz_frame_id ... not defined in SDF` warnings in the Gazebo terminal.

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
3. **First real flight confirmed live 2026-09-29** — `tools/px4_offboard_smoke_test.py`:
   a standalone arm/OFFBOARD/climb-and-hold smoke test, deliberately kept separate from
   `coordination_node.py` per this section's own "isolate before combining" plan.
   Confirmed exact `px4_msgs` fields live via `ros2 interface show` first
   (`OffboardControlMode`, `TrajectorySetpoint`, `VehicleCommand`) rather than
   assuming a version, then built against PX4's documented ROS 2 offboard sequence:
   stream `OffboardControlMode`+`TrajectorySetpoint` at 10Hz for ~2s, request
   `VEHICLE_CMD_DO_SET_MODE` (custom main mode 6 = OFFBOARD), request
   `VEHICLE_CMD_COMPONENT_ARM_DISARM`, hold position at a fixed altitude, and
   `VEHICLE_CMD_NAV_LAND` on SIGINT or after `--duration`. Only targets a single
   vehicle (`target_system=1`, unnamespaced `/fmu/in|out/...` topics). This script
   still only targets instance 0's plain topic names — multi-drone namespacing was
   resolved 2026-09-30 (see "PX4 telemetry" above: PX4 auto-namespaces instance
   N>0 under `/px4_N/fmu/...`, no config needed), but this specific script hasn't
   been generalized to target an arbitrary instance yet.

   **Real blocker hit and fixed on first run**: PX4 refused to arm at all —
   `health_and_arming_checks: Preflight Fail: No connection to the GCS` — and even
   `commander arm -f` (force-arm from the PX4 shell) only stayed armed for an
   instant before `Disarmed by auto preflight disarming`, because this specific
   check (`FailsafeFlags.gcs_connection_lost`, confirmed via
   `ros2 interface show px4_msgs/msg/FailsafeFlags`) is continuously monitored, not
   a one-time gate. Root cause: `NAV_DLL_ACT` (datalink-loss failsafe action) was
   `2` (an active failsafe, not `0`/disabled) — sensible for a real vehicle
   expecting a GCS link, wrong for this pure companion-computer/offboard setup with
   no GCS at all. **Fixed via `param set NAV_DLL_ACT 0`** in the PX4 shell before
   running the script. This is a per-session runtime change, not persisted to
   disk (no `param save` was run) — **treat it as a required manual setup step
   every time PX4 SITL is freshly launched** for offboard testing, not a one-time
   fix; a restarted PX4 instance reverts to `NAV_DLL_ACT=2` and will refuse to arm
   again until this is re-applied.

   **Confirmed working end-to-end**: script requested OFFBOARD mode, then ARM —
   PX4 logged `Armed by external command` (the *normal*, non-forced arm request
   succeeded), `Takeoff detected`, then ~20s later (matching `--duration 20`)
   `Landing detected` → `Disarmed by landing`. First real flight, even in
   simulation, in this entire project.

   **In progress 2026-09-30** — `tools/px4_offboard_pso_test.py`: subclasses
   `Px4OffboardSmokeTest` (refactored to expose an overridable `_next_xy()` hook
   instead of duplicating the arm/OFFBOARD/land sequence) and drives the
   horizontal setpoint from a real `ParticleSwarmSearch` particle each tick,
   using the vehicle's actual telemetry as `pso.step()`'s `real_position`. This
   needed a real gap closed in `pso.py` first: `step()` previously had no way to
   report "the position PSO wants to command next" separately from
   "`self.state.position`, which now tracks ground truth" — when `real_position`
   is given, both used to collapse to the same value (just the truth, echoed
   back), so there was nothing to actually send to a real vehicle. Fixed by
   having `step()` always compute the velocity-extrapolated
   `(commanded_x, commanded_y)` and return that, while `self.state.position`
   keeps tracking truth for fitness/personal-best bookkeeping — identical
   behavior to before when `real_position` is None (the two values are the same
   in that case), so this doesn't change any already-verified simulated
   behavior. Uses `make_coverage_fitness` with an empty neighbor list (one
   drone), so this specifically exercises the "no neighbor data" fallback path
   (distance-from-area-center) fixed 2026-09-29, not the neighbor-spread path —
   that needs a second real drone running simultaneously, which this script
   doesn't yet do (see "still needed" note at the end of this item).

   **Confirmed live 2026-09-30**: same real gotcha as the day before but a
   different sensor — PX4 refused to arm with `Preflight Fail: barometer 0
   missing` (`listener sensor_baro` -> "never published", while
   `sensor_accel`/`sensor_gps` showed live data). Fixed with
   `sensor_baro_sim start` in the PX4 shell — a different simulated-sensor
   module than yesterday's GCS-link fix, same "check for it, don't assume the
   launch recipe is broken" category. After that: full arm -> takeoff -> hold
   -> land cycle confirmed, same as the fixed-hold-point test. Watched
   `/fmu/out/vehicle_local_position_v1` live during a run to confirm PSO was
   actually driving real movement (not frozen) — confirmed, though only within
   a small (~1cm) range over ~25s of flight, consistent with the already-known
   PSO-convergence limitation now visible against a real vehicle instead of
   simulated position.

   **Confirmed live 2026-09-30** — `tools/px4_offboard_mission_test.py`: adds the
   full SEARCH -> detect -> TASK_ALLOCATION (`navigate.py`) -> complete
   (`cbba.py`'s `mark_task_done`) -> back to SEARCH cycle on top of the PSO
   test above, still one drone (no CBBA consensus contest — `build_bundle()`
   just wins uncontested, same code path as the two-drone case). The
   "detection" is scripted (fires `--detect-after` seconds after arming at a
   fixed `--target-x`/`--target-y`), same deliberate simplification
   `demo_run.sh` already uses. This needed the same commanded-vs-tracked split
   applied to `navigate.py`'s `step_toward()` that `pso.py`'s `step()` got the
   day before — it had the identical gap (echoed `real_position` straight back
   with no velocity applied when given real telemetry, nothing to actually
   command a real vehicle with). Now returns
   `(tracked_x, tracked_y, commanded_x, commanded_y)`; `coordination_node.py`'s
   one call site updated to unpack `tracked_x/y` only, so its own
   already-verified simulated-position behavior is unchanged.

   **Gotcha**: after `git pull`, the script crashed with `ValueError: not
   enough values to unpack (expected 4, got 2)` — `navigate.py` is part of the
   compiled `coordination_node` ROS2 package, so a plain `git pull` wasn't
   enough; the *installed* copy under `install/` was still the old pre-fix
   version until `colcon build` reran. `tools/*.py` scripts themselves don't
   need this (they're plain scripts, not part of the package), but anything
   they `import` from `coordination_node`'s own package does.

   **Full cycle confirmed working end-to-end on a real (simulated) vehicle**:
   armed, took off, searched, `Detection fired: investigating (3.0, 3.0)` ~15s
   after arming (matching `--detect-after`), flew there, `Task 1 complete -
   resuming search` ~15s later (genuinely arrived, not just timed out), landed
   cleanly at `--duration`. This is the entire single-drone coordination
   pipeline — not just PSO output reaching a vehicle, but the real
   search/detect/navigate/complete/resume-search cycle — proven on real flight,
   not only the ROS2-only ground-truth demos.
   Remaining before combining with the two-drone scenario: generalize this
   script (and `px4_offboard_pso_test.py`) to target an arbitrary PX4 instance
   via the now-known `/px4_N/fmu/...` namespacing (see "PX4 telemetry" above)
   instead of only instance 0's plain topics, then actually run two of them
   simultaneously against the two-instance PX4 setup.
   ~~**Must carry collision safety with it**~~ — done 2026-10-05: the hard
   floor now constrains the commanded setpoint for real vehicles (see "Hard
   floor for real vehicles" under "Collision avoidance" above).
4. **Partially done 2026-09-29** — replaced the fitness function's worst structural
   problems (see "PSO fitness function" above: not comparable across drones, fought
   collision avoidance, rewarded moving away from a nearby target) with
   `make_coverage_fitness` (nearest-neighbor spread, softly bounded by area
   center/radius). Still not a real search-quality score, and the bid function
   (`cbba.py: bid_for`) hasn't been touched — both remain open for tuning against
   actual multi-drone runs, per the report's own deferred scope.
   **Now visible on real vehicles (2026-10-05)**: two drones spawned 1m apart
   stayed pinned against each other at the collision floor for a whole run
   instead of spreading out to search (see "Known limits of the setpoint
   floor" above). **Replaced 2026-10-06** by the coverage-map search (see
   "Search: coverage map"), pending a live run (`sim.sh test-search`).
   Still open: per-cell search value (flood risk). The bid function was
   replaced the same day and confirmed live (see "Bid function" near the
   top).
5. **Done 2026-10-06, pending live confirmation** — retry ARM until PX4
   actually reports armed. `coordination_node.py` used to send ARM once and
   set `_px4_armed` on *send*, not on confirmation, so one early denial (PX4
   not ready yet) left that drone on the ground for the whole run. Now
   OFFBOARD + ARM are re-sent every 3s (`offboard_sequence.py`, unit-tested)
   until `/fmu/out/vehicle_status_v4` (`VehicleStatus`; the `_vN` suffix is
   derived from `px4_msgs`' `MESSAGE_VERSION`) reports `ARMING_STATE_ARMED`
   and `NAVIGATION_STATE_OFFBOARD`, then stop for good — so a later PX4
   failsafe (e.g. auto-land) is never fought by re-arming behind its back.
6. **Partially done 2026-10-06** — crossing-paths test against a hovering
   neighbor done, stall found and fixed (Test C, see "Hard floor for real
   vehicles"). Still open: both drones moving through each other's paths,
   then start AirSim planning: AirSim can run on the same PX4 SITL in place of Gazebo, so the
   coordination node's DDS/MAVLink paths should carry over; the real new
   work is a vision → `detect_target` bridge (detection pixel → world
   position via drone pose and camera geometry).
