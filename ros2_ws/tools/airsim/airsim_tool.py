#!/usr/bin/env python3
"""Small AirSim helper for backends/airsim.sh (runs in WSL, talks to AirSim
on Windows through its RPC API, port 41451).

  settings --spawn=X,Y [--spawn=X,Y ...] --host IP [--headless]
      print an AirSim settings.json with one PX4 vehicle per spawn point
  pose N --spawn-x X --spawn-y Y --host IP
      print vehicle PX4_N's true position as "X Y" in the shared frame
  marker X Y R G B --host IP
      draw a persistent coloured point at shared (X, Y), 0.5m up

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
    args = parser.parse_args(argv)

    if args.cmd == 'settings':
        spawns = [tuple(float(v) for v in s.split(',')) for s in args.spawn]
        print(settings(spawns, args.host, args.headless))
    elif args.cmd == 'pose':
        pos = client(args.host).simGetVehiclePose(
            vehicle_name=f'PX4_{args.n}').position
        if pos.x_val != pos.x_val:  # NaN: no such vehicle
            return 1
        print(f'{pos.x_val + args.spawn_x:.3f} {pos.y_val + args.spawn_y:.3f}')
    elif args.cmd == 'marker':
        import airsim
        client(args.host).simPlotPoints(
            [airsim.Vector3r(args.x, args.y, -0.5)],
            color_rgba=[args.r, args.g, args.b, 1.0], size=40.0,
            is_persistent=True)
    return 0


if __name__ == '__main__':
    sys.exit(main())
