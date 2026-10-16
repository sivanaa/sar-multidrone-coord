#!/usr/bin/env python3
"""Camera -> target detection -> coordination: the vision bridge, AirSim side.

    python3 tools/airsim/vision_bridge.py --host IP --spawn=0.0,0.0 \
        --spawn=5.0,5.0 [--hz 2] [--log detections.jsonl]

For each drone, a few times a second: point its nadir camera straight down
(frame_logger.stabilize_camera), take a segmentation image, find target
blobs, turn each blob's centre pixel into a ground position with the
drone's PX4 pose and flood_grid.Camera (the same pinhole model as the flood
converter), and report new ones through that drone's
/drone_N/coordination/detect_target service - exactly what a perception
node on a real drone would do. CBBA takes it from there.

The detector here is AirSim's segmentation image: every target (spawned by
`airsim_tool.py target`, names starting TARGET_PREFIX) is painted its own
flat colour, everything else is relabelled to one background colour. That
makes detection exact, so this proves the rest of the chain - pose,
projection, reporting, allocation - before a real person detector on the
Scene image takes its place (detect_blobs is the part to swap).

A blob touching the image border is skipped (its centre isn't the
target's); a detection within --dedupe metres of a target already reported
by any drone (TargetDetected, all drones) is not reported again.

Two guards against bad sightings, both from the first Mannequin run
(2026-10-15), where drone 1 saw a person while still climbing at 1.2m and
reported it 2.2m off - far enough that the later, correct sighting counted
as a second person, and drone 0 flew to both:
- only frames taken near cruise height and not climbing count (--min-alt,
  --max-climb): during a climb PX4's altitude lags, and at low height a
  small altitude error is a large share of the projection;
- a target is reported only once --confirm sightings (any drone) agree
  within --confirm-radius metres, at their average - which also trims the
  first-sighting error at the frame edge.
"""

import argparse
import json
import math
import os
import sys
import time

import numpy as np
import rclpy

from airsim_tool import (
    API_PORT, CAMERA_CLEARANCE_M, CAMERA_HFOV_DEG, CAMERA_NAME, TARGET_HEIGHT_M,
    TARGET_PREFIX, TARGET_SEG_ID)
from frame_logger import PoseCache, stabilize_camera

try:
    from coordination_node.flood_grid import Camera
except ImportError:  # workspace not built/sourced: use the source tree
    sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                    '..', '..', 'src', 'coordination_node'))
    from coordination_node.flood_grid import Camera

RELABEL_EVERY_S = 5.0


def detect_blobs(seg, min_pixels):
    """Centre pixels (u, v) of the non-background blobs in a segmentation
    image (H x W x 3) that don't touch the border, with their sizes."""
    import cv2
    h, w, _ = seg.shape
    flat = seg.reshape(-1, 3)
    colours, counts = np.unique(flat, axis=0, return_counts=True)
    background = colours[np.argmax(counts)]
    mask = np.any(seg != background, axis=2).astype(np.uint8)
    n, _, stats, centroids = cv2.connectedComponentsWithStats(mask, 8)
    blobs = []
    for k in range(1, n):
        x, y, bw, bh, area = stats[k]
        if area < min_pixels:
            continue
        if x == 0 or y == 0 or x + bw >= w or y + bh >= h:
            continue  # cut off by the frame edge
        blobs.append((tuple(centroids[k]), int(area)))
    return blobs


class VisionBridge(PoseCache):

    def __init__(self, num_drones, spawns, dedupe_m):
        super().__init__(spawns, node_name='airsim_vision_bridge')
        from coordination_msgs.msg import TargetDetected
        from coordination_msgs.srv import DetectTarget
        self.DetectTarget = DetectTarget
        self.dedupe_m = dedupe_m
        self.known = []  # (x, y) of every target reported by any drone
        self.pending = []  # unconfirmed sightings: {'xy', 'n', 't'}
        self.detect_clients = {}
        for i in range(num_drones):
            self.create_subscription(
                TargetDetected, f'/drone_{i}/coordination/target_detected',
                self._on_target, 10)
            self.detect_clients[i] = self.create_client(
                DetectTarget, f'/drone_{i}/coordination/detect_target')

    def _on_target(self, msg):
        self.known.append((msg.position.x, msg.position.y))

    def is_new(self, xy):
        return all(math.dist(xy, k) > self.dedupe_m for k in self.known)

    def confirm(self, xy, needed, radius, max_age_s=10.0):
        """Add a sighting; return the averaged position once `needed`
        sightings within `radius` metres agree, else None."""
        now = time.time()
        self.pending = [p for p in self.pending
                        if now - p['t'] < max_age_s]
        for p in self.pending:
            if math.dist(xy, p['xy']) <= radius:
                n = p['n'] + 1
                p['xy'] = ((p['xy'][0] * p['n'] + xy[0]) / n,
                           (p['xy'][1] * p['n'] + xy[1]) / n)
                p['n'], p['t'] = n, now
                if n >= needed:
                    self.pending.remove(p)
                    return p['xy']
                return None
        if needed <= 1:
            return xy
        self.pending.append({'xy': xy, 'n': 1, 't': now})
        return None

    def report(self, drone, xy):
        client = self.detect_clients[drone]
        if not client.service_is_ready():
            return False
        req = self.DetectTarget.Request()
        req.position.x, req.position.y = float(xy[0]), float(xy[1])
        req.target_type = 'person'
        req.confidence = 1.0
        client.call_async(req)
        self.known.append(xy)  # don't report it again while the call is out
        return True


def relabel(client):
    """Everything to background, then targets to their own ID."""
    client.simSetSegmentationObjectID('.*', 0, True)
    client.simSetSegmentationObjectID(f'{TARGET_PREFIX}.*', TARGET_SEG_ID, True)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--host', required=True)
    parser.add_argument('--spawn', action='append', required=True,
                        help='X,Y of drone 0, 1, ... (write as --spawn=X,Y)')
    parser.add_argument('--hz', type=float, default=2.0,
                        help='frames per second, per drone')
    parser.add_argument('--dedupe', type=float, default=1.0,
                        help='metres within which a detection is the same target')
    parser.add_argument('--min-pixels', type=int, default=40)
    parser.add_argument('--min-alt', type=float, default=2.5,
                        help='metres; ignore frames below (sim.sh flies at 3)')
    parser.add_argument('--max-climb', type=float, default=0.3,
                        help='m/s; ignore frames while climbing/descending faster')
    parser.add_argument('--confirm', type=int, default=2,
                        help='sightings that must agree before reporting')
    parser.add_argument('--confirm-radius', type=float, default=0.75)
    parser.add_argument('--log', help='write every detection here (JSON lines)')
    args, ros_args = parser.parse_known_args()
    spawns = [tuple(float(v) for v in s.split(',')) for s in args.spawn]

    import airsim
    client = airsim.MultirotorClient(ip=args.host, port=API_PORT,
                                     timeout_value=10)
    request = [airsim.ImageRequest(CAMERA_NAME, airsim.ImageType.Segmentation,
                                   False, False)]  # raw RGB
    log = open(args.log, 'a') if args.log else None

    rclpy.init(args=ros_args)
    node = VisionBridge(len(spawns), spawns, args.dedupe)
    cameras = {}  # (w, h) -> Camera
    last_relabel = 0.0
    next_capture = time.monotonic()
    try:
        while rclpy.ok():
            while time.monotonic() < next_capture:
                rclpy.spin_once(
                    node, timeout_sec=next_capture - time.monotonic())
            next_capture = max(next_capture + 1.0 / args.hz, time.monotonic())
            if time.time() - last_relabel > RELABEL_EVERY_S:
                try:
                    relabel(client)
                    last_relabel = time.time()
                except Exception as exc:
                    node.get_logger().warning(f'relabel failed ({exc})')
            for i in range(len(spawns)):
                pose = node.poses.get(i)
                if (pose is None or pose[3] < args.min_alt
                        or abs(pose[5]) > args.max_climb):
                    continue  # no estimate yet, too low, or climbing
                try:
                    stabilize_camera(client, f'PX4_{i}', airsim)
                    resp = client.simGetImages(
                        request, vehicle_name=f'PX4_{i}')[0]
                except Exception as exc:
                    node.get_logger().warning(f'drone {i}: no image ({exc})')
                    continue
                if not resp.image_data_uint8:
                    continue
                seg = np.frombuffer(resp.image_data_uint8, np.uint8).reshape(
                    resp.height, resp.width, 3)
                cam = cameras.setdefault(
                    (resp.width, resp.height),
                    Camera(CAMERA_HFOV_DEG, resp.width, resp.height))
                _, x, y, alt, yaw, _ = pose
                # Blob centres sit on the targets' top faces.
                height = alt + CAMERA_CLEARANCE_M - TARGET_HEIGHT_M
                for pixel, area in detect_blobs(seg, args.min_pixels):
                    xy = cam.ground_point(pixel, (x, y), height, yaw)
                    if xy is None:
                        continue
                    reported = False
                    if node.is_new(xy):
                        confirmed = node.confirm(
                            xy, args.confirm, args.confirm_radius)
                        if confirmed and node.report(i, confirmed):
                            reported = True
                            node.get_logger().info(
                                f'drone {i} sees a target at '
                                f'({confirmed[0]:.2f}, {confirmed[1]:.2f}) - '
                                f'reported ({args.confirm} sightings agree)')
                    if log:
                        log.write(json.dumps({
                            't': time.time(), 'drone': i,
                            'pixel': [round(pixel[0], 1), round(pixel[1], 1)],
                            'pixels': area, 'x': xy[0], 'y': xy[1],
                            'pose': {'x': x, 'y': y, 'alt': alt, 'yaw': yaw},
                            'reported': reported}) + '\n')
                        log.flush()
    except KeyboardInterrupt:
        pass
    finally:
        if log:
            log.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
