#!/usr/bin/env python3
"""Small AirSim helper for backends/airsim.sh (runs in WSL, talks to AirSim
on Windows through its RPC API, port 41451).

  settings --spawn=X,Y [--spawn=X,Y ...] --host IP [--headless] [--camera WxH]
      print an AirSim settings.json with one PX4 vehicle per spawn point,
      optionally each with a nadir camera
  pose N --spawn-x X --spawn-y Y --host IP
      print vehicle PX4_N's true position as "X Y" in the shared frame
  marker X Y R G B --host IP
      draw a persistent coloured point at shared (X, Y), 0.5m up
  target NAME X Y --host IP [--yaw DEG] [--kind person|box]
      spawn a target lying at shared (X, Y) - Unreal's Mannequin, or a
      person-sized box where SARMap isn't loaded - labelled for the
      segmentation camera; prints its position as one JSON line
  river RISK_MAP_JSON --host IP [--width 3]
      lay water strips along make_risk_map.py's river polyline
  watch A B --spawn=X,Y [--spawn=X,Y ...] --host IP [--hz 10]
      live distance between drones A and B, drone_distance.sh's format

Frames: the shared frame is drone 0's PX4 local NED frame, and AirSim's
world frame is also NED with its origin at PlayerStart, where drone 0
spawns - so shared (x, y) is AirSim (x, y), no swap (unlike Gazebo).
AirSim reports a vehicle's pose relative to that vehicle's own spawn
point, so `pose` adds the spawn back.
"""

import argparse
import json
import sys

API_PORT = 41451
PX4_TCP_PORT = 4560          # + instance, see PX4's px4-rc.mavlinksim
PX4_CONTROL_LOCAL = 14540    # + instance: PX4's Onboard MAVLink link
PX4_CONTROL_REMOTE = 14580   # + instance
# AirSim's GPS reports positions 0.2s late by default (GpsSimpleParams
# update_latency), but PX4's estimator is told 10ms (SENS_GPS0_DELAY,
# set-default in px4-rc.mavlinksim). Fusing 0.2s-old fixes as current put
# each drone's estimate 0.3-0.7s behind the truth and made it overshoot
# setpoints by up to a second - crossings dipped to 0.92m (2026-10-13). Match
# PX4 instead. Listing any sensor replaces AirSim's default set, so all
# four PX4 needs are listed; the rest are AirSim's defaults.
GPS_LATENCY_S = 0.01
SENSORS = {
    'Barometer': {'SensorType': 1, 'Enabled': True},
    'Imu': {'SensorType': 2, 'Enabled': True},
    'Gps': {'SensorType': 3, 'Enabled': True,
            'UpdateLatency': GPS_LATENCY_S},
    'Magnetometer': {'SensorType': 4, 'Enabled': True},
}


# Nadir camera like the single-drone pipeline's GoPro (Linear, 87 deg
# horizontal FOV, gimbal holding it straight down - see flood_grid.py's
# Camera). AirSim's FOV_Degrees is horizontal too. Full GoPro resolution
# is 1920x1080; two drones on this PC's GTX 1660 get a smaller default,
# same aspect ratio, so the pinhole geometry is unchanged.
CAMERA_NAME = 'nadir'
CAMERA_HFOV_DEG = 87.0
CAMERA_Z_M = 0.1  # mounted 0.1m below the vehicle's origin (NED: +down)
# Camera height above the ground with the drone landed. PX4's altitude is
# measured from where the vehicle's origin was at takeoff, which sits on the
# landing gear: in Blocks a landed drone's origin is at z=0.68 and the
# ground at z~1.02 (targets settle with their 0.3m-tall centres at 0.87),
# so the camera, CAMERA_Z_M below the origin, is ~0.24m above the ground at
# PX4 altitude 0. Leaving it out put every detection ~8% too close to the
# drone (first test-vision, 2026-10-15). A real drone needs its own value.
CAMERA_CLEARANCE_M = 0.24


def nadir_camera(width, height):
    """Body-fixed, pointing straight down. No AirSim "Gimbal": its tick
    (PIPCamera.cpp) reads the camera's world rotation as Euler angles,
    overwrites pitch/roll and keeps the yaw - which, looking straight down,
    is degenerate (gimbal lock), so frames spun freely about the optical
    axis (each drone's shadow, fixed in the world, swung through 120-180 deg
    of the image while AirSim's reported orientation tracked the spin; -89
    instead of -90 didn't help). frame_logger.py stabilizes the camera
    itself before each capture, with quaternions (see stabilize_camera)."""
    # Scene (0) is what a real camera sees; Segmentation (5) paints every
    # object a flat colour by its segmentation ID - targets get their own
    # (TARGET_SEG_ID), so vision_bridge.py can find them exactly while the
    # rest of the pipeline is being proven. Each is rendered only on request.
    return {CAMERA_NAME: {
        'CaptureSettings': [
            {'ImageType': t, 'Width': width, 'Height': height,
             'FOV_Degrees': CAMERA_HFOV_DEG} for t in (0, 5)],
        'X': 0.0, 'Y': 0.0, 'Z': CAMERA_Z_M,
        'Pitch': -90.0, 'Roll': 0.0, 'Yaw': 0.0,
    }}


def settings(spawns, host, headless, camera=None):
    vehicles = {}
    for n, (x, y) in enumerate(spawns):
        vehicles[f'PX4_{n}'] = {
            'VehicleType': 'PX4Multirotor',
            'UseSerial': False,
            'LockStep': True,
            'UseTcp': True,
            'TcpPort': PX4_TCP_PORT + n,
            'ControlIp': 'remote',
            'ControlPortLocal': PX4_CONTROL_LOCAL + n,
            'ControlPortRemote': PX4_CONTROL_REMOTE + n,
            'LocalHostIp': host,
            'X': x, 'Y': y, 'Z': 0,
            'Sensors': SENSORS,
            'Parameters': {
                'NAV_RCL_ACT': 0,
                'NAV_DLL_ACT': 0,
                'COM_OBL_RC_ACT': 2,
            },
        }
    doc = {
        'SettingsVersion': 1.2,
        'SimMode': 'Multirotor',
        'ClockType': 'SteppableClock',
        'Vehicles': vehicles,
    }
    if camera:
        for vehicle in vehicles.values():
            vehicle['Cameras'] = nadir_camera(*camera)
    if headless:
        doc['ViewMode'] = 'NoDisplay'
    return json.dumps(doc, indent=2)


def client(host):
    import airsim
    return airsim.MultirotorClient(ip=host, port=API_PORT, timeout_value=5)


# Targets: a person lying on the ground. 'person' is Unreal's Mannequin,
# available once SARMap is loaded (backends/airsim.sh); 'box' is Blocks'
# 1m chamfered cube scaled to 0.6 x 1.7 x 0.3m, the stand-in used before
# SARMap existed. Both get TARGET_SEG_ID for the segmentation camera.
TARGET_HEIGHT_M = 0.3  # top surface above ground, for vision_bridge.py
TARGET_SEG_ID = 200
TARGET_PREFIX = 'sar_target_'  # vision_bridge.py relabels these by regex
BOX_ASSET = '1M_Cube_Chamfer'
BOX_SCALE = (0.6, 1.7, 0.3)
PERSON_ASSET = 'SM_Mannequin'
# The static Mannequin has no collision, so it can't be dropped with physics
# like the box: it is placed at the ground height the boxes settled at
# (centres at z=0.87, 0.15m above the ground at z~1.02 in Blocks' NED), its
# ~0.25m-deep body half sunk. Its pivot is at the feet, and laid down (pitch
# +90 deg) the body runs ~1.8m backwards from it - placed so the body's
# centre is at the requested point (checked from 8m above, 2026-10-15).
GROUND_Z_M = 1.02
PERSON_HALF_LENGTH_M = 0.9
PERSON_DEPTH_M = 0.25


def spawn_target(c, name, x, y, kind='person', yaw_deg=0.0):
    """Spawn target `name` with its centre at shared (x, y), lying along
    heading yaw_deg (0 = north); returns where it ended up as (x, y, z)."""
    import math
    import time

    import airsim
    if kind == 'person':
        yaw = math.radians(yaw_deg)
        pivot = (x + PERSON_HALF_LENGTH_M * math.cos(yaw),
                 y + PERSON_HALF_LENGTH_M * math.sin(yaw))
        name = c.simSpawnObject(
            name, PERSON_ASSET,
            airsim.Pose(airsim.Vector3r(pivot[0], pivot[1],
                                        GROUND_Z_M - PERSON_DEPTH_M / 2),
                        airsim.to_quaternion(math.pi / 2, 0.0, yaw)),
            airsim.Vector3r(1.0, 1.0, 1.0), False)
        c.simSetSegmentationObjectID(name, TARGET_SEG_ID, False)
        return (x, y, GROUND_Z_M - PERSON_DEPTH_M / 2)
    name = c.simSpawnObject(
        name, BOX_ASSET,
        airsim.Pose(airsim.Vector3r(x, y, 0.0),
                    airsim.to_quaternion(0.0, 0.0, math.radians(yaw_deg))),
        airsim.Vector3r(*BOX_SCALE), True)
    c.simSetSegmentationObjectID(name, TARGET_SEG_ID, False)
    time.sleep(2.0)  # let it fall and settle
    p = c.simGetObjectPose(name).position
    return (p.x_val, p.y_val, p.z_val)


# Flood water: Starter Content's lake material on a flat 1m plane, scaled
# along a river segment (both loaded by SARMap). simSetObjectMaterial
# needs a plain Material's full object path.
WATER_ASSET = 'Shape_Plane'
WATER_MATERIAL = '/Game/StarterContent/Materials/M_Water_Lake.M_Water_Lake'


def spawn_water(c, name, a, b, width, layer=0):
    """Water strip `width` metres wide along segment a -> b (shared x, y),
    overlapping the next segment by half its width so bends have no gaps."""
    import math

    import airsim
    length = math.dist(a, b) + width / 2
    yaw = math.atan2(b[1] - a[1], b[0] - a[0])
    centre = ((a[0] + b[0]) / 2, (a[1] + b[1]) / 2)
    name = c.simSpawnObject(
        name, WATER_ASSET,
        # each later strip 5mm higher: two planes at one height flicker
        # where they overlap (z-fighting, seen at the river bend)
        airsim.Pose(airsim.Vector3r(centre[0], centre[1],
                                    GROUND_Z_M - 0.01 - 0.005 * layer),
                    airsim.to_quaternion(0.0, 0.0, yaw)),
        airsim.Vector3r(length, width, 1.0), False)
    return name, c.simSetObjectMaterial(name, WATER_MATERIAL)


def shared_xy(c, n, spawn):
    """Vehicle PX4_n's true (x, y) in the shared frame, or None."""
    pos = c.simGetVehiclePose(vehicle_name=f'PX4_{n}').position
    if pos.x_val != pos.x_val:  # NaN: no such vehicle (yet)
        return None
    return (pos.x_val + spawn[0], pos.y_val + spawn[1])


def line(a, pa, b, pb, d, closest, floor):
    """drone_distance.sh's line format - sim.sh down greps 'closest X m'."""
    warn = f'  <-- went under {floor}m floor' if closest < floor else ''
    return (f'drone{a} ({pa[0]:.2f}, {pa[1]:.2f})  drone{b} ({pb[0]:.2f}, '
            f'{pb[1]:.2f})  distance {d:.2f} m  closest {closest:.2f} m{warn}')


def watch(host, a, b, spawns, hz, print_every_s, floor, log_path=None):
    """drone_distance.sh for AirSim: one API connection kept open, sampled
    at `hz`, printed every `print_every_s` and at every new closest. Two
    fresh Python processes per sample managed ~1.2 samples/s (first live
    AirSim test-x, 2026-10-13), too slow to catch a head-on pass. With
    `log_path`, every sample also goes there as JSON lines {t, drone:
    [x, y], ...} (wall-clock t, same clock as mission_recorder.py), to
    compare true positions with what the drones believe."""
    import math
    import time
    c = client(host)
    closest, last_print = math.inf, 0.0
    log = open(log_path, 'w') if log_path else None
    while True:
        start = time.monotonic()
        try:
            pa, pb = shared_xy(c, a, spawns[a]), shared_xy(c, b, spawns[b])
        except Exception:  # AirSim not up yet, or restarting
            pa = pb = None
            c = client(host)
        if pa is None or pb is None:
            print(f'waiting for drones {a} and {b} in airsim...', flush=True)
            time.sleep(1.0)
            continue
        if log:
            log.write(json.dumps(
                {'t': time.time(), str(a): pa, str(b): pb}) + '\n')
            log.flush()
        d = math.dist(pa, pb)
        new_closest = d < closest
        closest = min(closest, d)
        if new_closest or start - last_print >= print_every_s:
            print(line(a, pa, b, pb, d, closest, floor), flush=True)
            last_print = start
        time.sleep(max(0.0, 1.0 / hz - (time.monotonic() - start)))


def main(argv=None):
    parser = argparse.ArgumentParser()
    sub = parser.add_subparsers(dest='cmd', required=True)
    p = sub.add_parser('settings')
    p.add_argument('--spawn', action='append', required=True,
                   help='X,Y in the shared frame (write as --spawn=X,Y)')
    p.add_argument('--host', required=True)
    p.add_argument('--headless', action='store_true')
    p.add_argument('--camera', metavar='WxH',
                   help='add a nadir camera per drone, e.g. 640x360')
    p = sub.add_parser('pose')
    p.add_argument('n', type=int)
    p.add_argument('--spawn-x', type=float, required=True)
    p.add_argument('--spawn-y', type=float, required=True)
    p.add_argument('--host', required=True)
    p = sub.add_parser('marker')
    for name in ('x', 'y', 'r', 'g', 'b'):
        p.add_argument(name, type=float)
    p.add_argument('--host', required=True)
    p = sub.add_parser('target')
    p.add_argument('name')
    p.add_argument('x', type=float)
    p.add_argument('y', type=float)
    p.add_argument('--yaw', type=float, default=0.0,
                   help='heading the body lies along, degrees (0 = north)')
    p.add_argument('--kind', choices=('person', 'box'), default='person')
    p.add_argument('--host', required=True)
    p = sub.add_parser('river')
    p.add_argument('risk_map', help="make_risk_map.py's output (has 'river')")
    p.add_argument('--width', type=float, default=3.0)
    p.add_argument('--host', required=True)
    p = sub.add_parser('watch')
    p.add_argument('a', type=int)
    p.add_argument('b', type=int)
    p.add_argument('--spawn', action='append', required=True,
                   help='X,Y of drone 0, 1, ... in order (write as --spawn=X,Y)')
    p.add_argument('--hz', type=float, default=10.0)
    p.add_argument('--print-every', type=float, default=0.5)
    p.add_argument('--floor', type=float, default=1.5)
    p.add_argument('--log', help='also write every sample here (JSON lines)')
    p.add_argument('--host', required=True)
    args = parser.parse_args(argv)

    if args.cmd == 'settings':
        spawns = [tuple(float(v) for v in s.split(',')) for s in args.spawn]
        camera = (tuple(int(v) for v in args.camera.lower().split('x'))
                  if args.camera else None)
        print(settings(spawns, args.host, args.headless, camera))
    elif args.cmd == 'pose':
        xy = shared_xy(client(args.host), args.n, (args.spawn_x, args.spawn_y))
        if xy is None:
            return 1
        print(f'{xy[0]:.3f} {xy[1]:.3f}')
    elif args.cmd == 'target':
        c = client(args.host)
        kind = args.kind
        if kind == 'person' and PERSON_ASSET not in c.simListAssets():
            kind = 'box'  # not on SARMap: the stand-in still works
        x, y, z = spawn_target(c, args.name, args.x, args.y, kind, args.yaw)
        # one JSON line: sim.sh appends it to the run's targets.jsonl
        print(json.dumps({'name': args.name, 'kind': kind, 'x': x, 'y': y,
                          'z': z, 'yaw_deg': args.yaw}))
    elif args.cmd == 'river':
        with open(args.risk_map) as f:
            river = json.load(f)['river']
        c = client(args.host)
        for k, (a, b) in enumerate(zip(river, river[1:])):
            name, ok = spawn_water(c, f'sar_water_{k}', a, b, args.width, k)
            print(f'water {name}: ({a[0]}, {a[1]}) -> ({b[0]}, {b[1]}), '
                  f'{args.width}m wide{"" if ok else ", MATERIAL FAILED"}')
    elif args.cmd == 'watch':
        spawns = [tuple(float(v) for v in s.split(',')) for s in args.spawn]
        try:
            watch(args.host, args.a, args.b, spawns, args.hz,
                  args.print_every, args.floor, args.log)
        except KeyboardInterrupt:
            pass
    elif args.cmd == 'marker':
        import airsim
        client(args.host).simPlotPoints(
            [airsim.Vector3r(args.x, args.y, -0.5)],
            color_rgba=[args.r, args.g, args.b, 1.0], size=40.0,
            is_persistent=True)
    return 0


if __name__ == '__main__':
    sys.exit(main())
