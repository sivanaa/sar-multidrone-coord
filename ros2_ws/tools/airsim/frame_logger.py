#!/usr/bin/env python3
"""Save each drone's nadir camera frames with the pose it was taken from.

    python3 tools/airsim/frame_logger.py --host IP --spawn=0.0,0.0 \
        --spawn=5.0,5.0 --out DIR [--hz 2]

Needs settings.json made with `airsim_tool.py settings --camera WxH`
(sim.sh does that when CAMERA=WxH is set). Writes DIR/d<N>/<seq>.png and
one line per frame to DIR/frames.jsonl:

    {"t": wall time, "drone": N, "image": "d0/00012.png",
     "x": .., "y": .., "alt": .., "yaw": ..,       # PX4's estimate
     "pose_age_s": ..,                              # how old that estimate was
     "true": {"x", "y", "alt", "heading", "pitch"}, # AirSim camera pose
     "sim_t": AirSim time stamp, seconds}

x/y are the shared frame (drone 0's PX4 local NED frame), alt metres above
the takeoff ground, yaw radians with PX4's convention (0 = north, + toward
east) - the fields tools/flood_to_risk_map.py reads, so adding the flood
model's response to a line makes it a converter input as is. The PX4
estimate is what a real drone would have; "true" is there to check it and
the camera geometry:
- true heading is where the top edge of the image points (same convention
  as yaw; flood_grid.py's Camera assumes image top = drone forward). Not
  the camera's Euler yaw: looking straight down, yaw and roll are
  degenerate (gimbal lock) - roll swung through +-180 in the first run.
- true pitch should stay -pi/2: before each capture the camera is turned
  straight down with its top edge toward the drone's heading, against the
  drone's tilt (stabilize_camera) - what a follow-mode gimbal does.
- true alt is above the ground under the takeoff point. AirSim reports
  positions from a reference ~0.7m above its ground (plus the camera's
  0.1m mount), so the logger takes each drone's ground level from its
  frames while PX4 says it is still on the ground; until then (logger
  started after takeoff) true alt is left out.

Prints AirSim's real-time factor every 10s: sim time over wall time. PX4
runs in lockstep with AirSim, so anything under ~1.0 slows the vehicles
relative to the wall-clock coordination nodes.
"""

import argparse
import json
import math
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy)

from airsim_tool import API_PORT, CAMERA_NAME, CAMERA_Z_M

ON_GROUND_ALT_M = 0.3  # PX4 altitude below which a drone counts as landed


class PoseCache(Node):
    """Latest PX4 position estimate per drone, in the shared frame."""

    def __init__(self, spawns, node_name='airsim_frame_logger'):
        super().__init__(node_name)
        from px4_msgs.msg import VehicleLocalPosition
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST, depth=1)
        version = getattr(VehicleLocalPosition, 'MESSAGE_VERSION', 0)
        suffix = f'_v{version}' if version else ''
        self.poses = {}
        for i, spawn in enumerate(spawns):
            prefix = '' if i == 0 else f'/px4_{i}'
            self.create_subscription(
                VehicleLocalPosition,
                f'{prefix}/fmu/out/vehicle_local_position{suffix}',
                lambda msg, i=i, s=spawn: self._on_position(i, s, msg), qos)

    def _on_position(self, i, spawn, msg):
        if msg.xy_valid and msg.z_valid:
            self.poses[i] = (time.time(), msg.x + spawn[0], msg.y + spawn[1],
                             -msg.z, msg.heading)


def rotate(q, v):
    """Vector v (NED) rotated by AirSim quaternion q."""
    qv = (q.x_val, q.y_val, q.z_val)
    t = [2 * c for c in _cross(qv, v)]
    c = _cross(qv, t)
    return tuple(v[k] + q.w_val * t[k] + c[k] for k in range(3))


def _cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0])


def nadir_relative_to(q_body, airsim):
    """Camera rotation, relative to a drone at orientation q_body, that
    points it straight down with the image top toward the drone's heading.
    world = body * relative, so relative = body^-1 * world. Quaternions
    throughout: the Euler route is what made AirSim's own gimbal spin."""
    _, _, yaw = airsim.to_eularian_angles(q_body)  # drone is near level
    world = airsim.to_quaternion(-math.pi / 2, 0.0, yaw)
    # conjugate == inverse for a unit quaternion; airsim 1.8.1's inverse()
    # uses np.sctypes, which NumPy 2 removed.
    return q_body.conjugate() * world


def stabilize_camera(client, vehicle, airsim):
    q_body = client.simGetVehiclePose(vehicle_name=vehicle).orientation
    client.simSetCameraPose(
        CAMERA_NAME,
        airsim.Pose(airsim.Vector3r(0.0, 0.0, CAMERA_Z_M),
                    nadir_relative_to(q_body, airsim)),
        vehicle_name=vehicle)


def camera_geometry(q):
    """(heading of the image's top edge, pitch of the optical axis), rad.
    AirSim camera axes: x along the optical axis, y image right, z image
    down - so image up is -z."""
    up = rotate(q, (0.0, 0.0, -1.0))
    axis = rotate(q, (1.0, 0.0, 0.0))
    return (math.atan2(up[1], up[0]),
            -math.asin(max(-1.0, min(1.0, axis[2]))))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--host', required=True)
    parser.add_argument('--spawn', action='append', required=True,
                        help='X,Y of drone 0, 1, ... (write as --spawn=X,Y)')
    parser.add_argument('--out', required=True)
    parser.add_argument('--hz', type=float, default=2.0,
                        help='frames per second, per drone')
    args, ros_args = parser.parse_known_args()
    spawns = [tuple(float(v) for v in s.split(',')) for s in args.spawn]

    import airsim
    client = airsim.MultirotorClient(ip=args.host, port=API_PORT,
                                     timeout_value=10)
    request = [airsim.ImageRequest(CAMERA_NAME, airsim.ImageType.Scene,
                                   False, True)]  # compressed PNG
    for i in range(len(spawns)):
        os.makedirs(os.path.join(args.out, f'd{i}'), exist_ok=True)
    log = open(os.path.join(args.out, 'frames.jsonl'), 'a')

    rclpy.init(args=ros_args)
    node = PoseCache(spawns)
    seq = {i: 0 for i in range(len(spawns))}
    ground_z = {}  # drone -> camera z (AirSim NED) while on the ground
    rtf_start = None  # (wall, sim) at the start of the report window
    last_report = time.time()
    frames_in_window = 0
    next_capture = time.monotonic()
    try:
        while rclpy.ok():
            # Wait for the next capture slot. spin_once alone returns at the
            # first PX4 position message (~100Hz), which made the first
            # version capture ~37 frames/s instead of 2 per drone.
            while time.monotonic() < next_capture:
                rclpy.spin_once(
                    node, timeout_sec=next_capture - time.monotonic())
            next_capture += 1.0 / args.hz
            if next_capture < time.monotonic():  # fell behind: don't burst
                next_capture = time.monotonic() + 1.0 / args.hz
            for i in range(len(spawns)):
                pose = node.poses.get(i)
                if pose is None:
                    continue  # PX4 estimate not valid yet
                try:
                    stabilize_camera(client, f'PX4_{i}', airsim)
                    resp = client.simGetImages(
                        request, vehicle_name=f'PX4_{i}')[0]
                except Exception as exc:  # AirSim restarting / not up
                    node.get_logger().warning(f'drone {i}: no image ({exc})')
                    continue
                if not resp.image_data_uint8:
                    continue
                now = time.time()
                name = os.path.join(f'd{i}', f'{seq[i]:05d}.png')
                seq[i] += 1
                with open(os.path.join(args.out, name), 'wb') as f:
                    f.write(resp.image_data_uint8)
                p = resp.camera_position  # relative to this vehicle's spawn
                if pose[3] < ON_GROUND_ALT_M:
                    # lowest reading: PX4 says "landed" up to 0.3m, and the
                    # last on-ground frame was 0.2m into the climb (2026-10-15)
                    ground_z[i] = max(ground_z.get(i, p.z_val), p.z_val)
                heading, pitch = camera_geometry(resp.camera_orientation)
                true = {'x': p.x_val + spawns[i][0],
                        'y': p.y_val + spawns[i][1],
                        'heading': heading, 'pitch': pitch}
                if i in ground_z:
                    true['alt'] = ground_z[i] - p.z_val
                sim_t = resp.time_stamp / 1e9
                log.write(json.dumps({
                    't': now, 'drone': i, 'image': name,
                    'x': pose[1], 'y': pose[2], 'alt': pose[3],
                    'yaw': pose[4], 'pose_age_s': round(now - pose[0], 3),
                    'true': true, 'sim_t': sim_t}) + '\n')
                log.flush()
                frames_in_window += 1
                if rtf_start is None:
                    rtf_start = (now, sim_t)
                elif now - last_report >= 10.0:
                    rtf = (sim_t - rtf_start[1]) / (now - rtf_start[0])
                    node.get_logger().info(
                        f'real-time factor {rtf:.2f}, '
                        f'{frames_in_window / (now - last_report):.1f} '
                        f'frames/s, {sum(seq.values())} saved')
                    rtf_start, last_report = (now, sim_t), now
                    frames_in_window = 0
    except KeyboardInterrupt:
        pass
    finally:
        log.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
