#!/usr/bin/env python3
"""Small AirSim helper for backends/airsim.sh (runs in WSL, talks to AirSim
on Windows through its RPC API, port 41451).

  settings --spawn=X,Y [--spawn=X,Y ...] --host IP [--headless]
      print an AirSim settings.json with one PX4 vehicle per spawn point
  pose N --spawn-x X --spawn-y Y --host IP
      print vehicle PX4_N's true position as "X Y" in the shared frame
  marker X Y R G B --host IP
      draw a persistent coloured point at shared (X, Y), 0.5m up
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


def settings(spawns, host, headless):
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
    if headless:
        doc['ViewMode'] = 'NoDisplay'
    return json.dumps(doc, indent=2)


def client(host):
    import airsim
    return airsim.MultirotorClient(ip=host, port=API_PORT, timeout_value=5)


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
    p = sub.add_parser('pose')
    p.add_argument('n', type=int)
    p.add_argument('--spawn-x', type=float, required=True)
    p.add_argument('--spawn-y', type=float, required=True)
    p.add_argument('--host', required=True)
    p = sub.add_parser('marker')
    for name in ('x', 'y', 'r', 'g', 'b'):
        p.add_argument(name, type=float)
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
        print(settings(spawns, args.host, args.headless))
    elif args.cmd == 'pose':
        xy = shared_xy(client(args.host), args.n, (args.spawn_x, args.spawn_y))
        if xy is None:
            return 1
        print(f'{xy[0]:.3f} {xy[1]:.3f}')
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
