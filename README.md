# sar-multidrone-coord

A ROS 2 coordination layer that lets a small fleet of drones (2–4 tested)
search a flooded area and decide among themselves which drone goes to each
person found, with no central controller. Each drone runs the same
`coordination_node`; drones talk only over ROS 2 topics, and keep working
when messages are lost. It flies PX4 drones (SITL) in Gazebo and in AirSim.

- **Search:** a coverage map of the ground seen recently (optionally weighted
  by a flood-risk map), with Particle Swarm Optimisation moving each drone
  toward unsearched ground.
- **Task allocation:** each person found becomes a task; the drones bid and
  agree with the Consensus-Based Bundle Algorithm (CBBA, Choi et al. 2009),
  hardened for lost messages. A drone at its battery reserve hands its tasks
  over and flies home.
- **Safety:** every position command keeps 1.8 m from where each neighbour
  is heading, with a braking limit, so drones stay at least 1.5 m apart.

Internship project of Sivana Terron Gonzalez, CCU / NSTC IIPP, supervised by
Dr. Pao-Ann Hsiung (September–October 2026).

## Results in brief

Measured in the fleet simulator over hundreds of missions, then confirmed
with live PX4 flights. Figures and data: [`results/`](results/); full history
and every number: [`ARCHITECTURE.md`](ARCHITECTURE.md), "Next steps" 8.

| | Result |
|---|---|
| CBBA assignment quality | 98.7–98.8% of the brute-force optimum on average (nearest-drone rule: 94.7–95.5%) |
| CBBA with 60% of messages lost | every person reached in 80/80 missions (plain CBBA: 45–52) |
| Search, plain map | level with a lawnmower sweep; 2–3× faster than random waypoints |
| Search, a drone leaves / people near a river | 13–34% faster than a re-planning lawnmower; up to 42% with both |
| Live flights | Gazebo: 2, 3 and 4 drones pass, closest 1.72 m. AirSim: 3 drones pass, closest 1.55 m |

Known limits: CBBA can leave the last person waiting (8–14% later than the
best possible); on a plain map a lawnmower has the better worst case; the
AirSim safety margin is thin; one or two live runs per configuration. The
lab's vision models (person detector, flood segmentation) were tried offline
on AirSim frames: the detector partly works, the flood models don't transfer
to the synthetic scene.

## Repository layout

| Path | What it is |
|---|---|
| `ros2_ws/src/coordination_msgs/` | Messages: `AgentState`, `BundleState`, `TargetDetected`; service `DetectTarget` |
| `ros2_ws/src/coordination_node/coordination_node/brain.py` | All decision logic (`DroneBrain`), ROS-free; the tuned constants live here |
| `.../cbba.py`, `pso.py`, `coverage.py`, `separation.py`, `navigate.py` | CBBA, PSO, coverage map, collision floors, fly-to-task |
| `.../coordination_node.py` | The ROS 2 node: parameters, PX4 offboard, topics, wiring to the brain |
| `.../flood_grid.py` | Camera projection; flood model output → flood-risk map |
| `ros2_ws/src/coordination_node/test/` | Unit tests |
| `ros2_ws/tools/sim.sh` | One-command simulator runs and tests (tmux), Gazebo or AirSim |
| `ros2_ws/tools/backends/` | Everything simulator-specific (`gazebo.sh`, `airsim.sh`) |
| `ros2_ws/tools/fleet_sim.py` | Fast fleet simulator: the real brain code, no ROS/PX4 |
| `ros2_ws/tools/search_baselines.py`, `cbba_optimality.py` | Lawnmower / random / greedy search; CBBA vs brute force |
| `ros2_ws/tools/report_sweeps.sh`, `make_plots.py` | Regenerate every report figure (`results/`) |
| `ros2_ws/tools/airsim/` | AirSim tools: settings, camera logger, vision bridge, `model_eval.py` |
| `results/` | Raw sweep data, figures (+ data tables), model examples, presentation source |
| `ARCHITECTURE.md` | Design notes, launch details, and the dated log of every test and fix |

The `launch_*.sh`, `run_coord*.sh` and other scripts in the repo root are
the early manual launch steps; they hardcode another machine's paths. Use
`sim.sh` instead.

## Setup

Tested on Windows 11 with WSL Ubuntu 22.04 (the lab PC); plain Ubuntu 22.04
works the same for Gazebo.

1. **ROS 2 Humble** in `/opt/ros/humble`, plus `colcon` and `tmux`.
2. **PX4-Autopilot** in `~/PX4-Autopilot` (v1.18 beta, commit `37e0cb3f`),
   built with `make px4_sitl`. Its `Tools/setup/ubuntu.sh` also installs
   **Gazebo Harmonic**.
3. **Micro-XRCE-DDS-Agent v2.4.x**, built; `sim.sh` expects it at
   `~/Micro-XRCE-DDS-Agent-243/build/MicroXRCEAgent` (an older v1.x agent
   does not work with this PX4).
4. **This repo** cloned in the Linux filesystem (`~/sar-multidrone-coord`,
   not under `/mnt/c`), with **px4_msgs** cloned into
   `ros2_ws/src/px4_msgs` (gitignored; its message files must match PX4's
   `msg/versioned/`). Then:
   ```bash
   cd ~/sar-multidrone-coord/ros2_ws && colcon build --symlink-install
   ```
5. **AirSim (optional):** AirSim 1.8.1 with Unreal Engine 4.27 on Windows,
   Blocks environment, with our `SARMap` level (how to recreate it:
   ARCHITECTURE.md "Next steps" 7, item 4); in WSL, `pip install --user airsim
   msgpack-rpc-python`. Paths are at the top of
   `ros2_ws/tools/backends/airsim.sh`.

Paths for PX4, the agent and the repo are set at the top of
`ros2_ws/tools/sim.sh`; change them there if yours differ.

## Running simulations

From a WSL terminal, in `~/sar-multidrone-coord`:

```bash
bash ros2_ws/tools/sim.sh test-bids                      # 2 drones, 4 people
NUM_DRONES=3 bash ros2_ws/tools/sim.sh test-fleet        # 3 (or 4) drones, 5 people
SIM=airsim NUM_DRONES=3 bash ros2_ws/tools/sim.sh test-fleet   # same, in AirSim
bash ros2_ws/tools/sim.sh attach                         # watch the windows (Ctrl+b w)
NUM_DRONES=3 bash ros2_ws/tools/sim.sh down              # stop; logs to ~/sim_runs/
```

Always stop with `down`, using the same `SIM=` and `NUM_DRONES=` as the run,
so every drone stops; it prints the closest the drones came. Other tests:
`test-search`, `test-handoff` (battery hand-off), `test-x` (head-on
crossings), `test-vision` (AirSim cameras find the people; add
`CAMERA=640x360` to record frames), `demo`, `demo-flood`; `bash
ros2_ws/tools/sim.sh` with no arguments lists them all. Add `--headless` to
run without the 3D window.

## Experiments without a simulator

The fleet simulator runs the same `DroneBrain` code with a vehicle model
fitted to Gazebo/AirSim and a radio model with loss and delay. Run from
`ros2_ws/tools`:

```bash
python3 fleet_sim.py --drones 3 --targets 4 --trials 80 --loss 0.6
python3 fleet_sim.py --drones 3 --area 4 4 12 --search lawnmower --dropout 15 --river 0.8
python3 fleet_sim.py --drones 4 --set brain.CBBA_OUTBID_MARGIN=0.1   # try another setting
python3 cbba_optimality.py --drones 4 --tasks 6 --trials 300
bash report_sweeps.sh 10 && python3 make_plots.py                   # all report figures (~10 min)
```

`make_plots.py` writes plain SVG (no matplotlib needed). The lab's vision
models are scored with `tools/airsim/model_eval.py` in its own venv
(onnxruntime, opencv, numpy; see its docstring).

## Tests

```bash
cd ros2_ws/src/coordination_node
PYTHONPATH=. python3 -m pytest -q test/test_cbba.py test/test_coverage.py \
  test/test_separation.py test/test_flood_grid.py test/test_offboard_sequence.py
```

## Key settings

Tuned values, each with the evidence in a comment beside it:

| Setting | Value | Where |
|---|---|---|
| Re-broadcast bundle state | every 1 s | `brain.REBROADCAST_S` |
| Duplicate person merge radius | 1.0 m | `brain.CBBA_DUPLICATE_RADIUS_M` |
| Outbid margin, bid refresh | 20%, on | `brain.CBBA_OUTBID_MARGIN`, `CBBA_REFRESH_BIDS` |
| Task arrival radius | 1.0 m | `brain.TASK_ARRIVAL_RADIUS_M` |
| Revisit time | 240 s, oldest 10% always due | `brain.REVISIT_AFTER_S`, `coverage.MIN_DUE_FRACTION` |
| Separation floor / setpoint margin | 1.5 m / 1.8 m | `brain.MIN_SEPARATION_M`, `SETPOINT_MIN_SEPARATION_M` |
| Braking limit | 1.0 m/s² | `brain.BRAKE_DECEL_MPS2` |

Node parameters (`ros2 run coordination_node coordination_node --ros-args -p …`):
`drone_id`, `num_drones`, `initial_x`/`initial_y` (spawn in the shared
frame: drone 0's PX4 local NED; Gazebo `gx,gy` is shared `(gy, gx)`),
`area_center_x`/`area_center_y`/`area_radius`, `sensor_radius`,
`revisit_after_s`, `risk_map_file`, `task_types`, `use_px4_offboard`,
`hold_altitude`. Topics: `/drone_{id}/coordination/{agent_state,
target_detected, bundle_state}`; service
`/drone_{id}/coordination/detect_target` reports a person.

## Open next steps

- Min-max (makespan-aware) CBBA bids, so the last person is not left waiting.
- Logical timestamps for CBBA's consensus instead of each drone's clock,
  before real hardware.
- The lab's person and flood models on a photoreal scene or real footage,
  then live in place of the vision bridge's simple detector.
- Repeated live runs per configuration, and more safety margin in AirSim.

Details and reasoning for each: ARCHITECTURE.md, "Next steps".
