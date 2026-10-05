#!/usr/bin/env python3
"""One-off test: can PX4 instance 1 be armed + switched to OFFBOARD via
MAVLink (COMMAND_LONG) instead of the DDS VehicleCommand path that's known
to be silently ignored for instance N>0 (GitHub issue #21284)?

Connects to instance 1's own 'Normal' mode MAVLink UDP port (18571, found
from its boot log - each PX4 SITL instance gets local_port = 18570 +
instance). Requires a DDS-based offboard setpoint stream already running
against instance 1 (px4_offboard_smoke_test.py) so PX4 has valid recent
setpoints to switch into OFFBOARD with.
"""
import time
from pymavlink import mavutil

PX4_CUSTOM_MAIN_MODE_OFFBOARD = 6
MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1

print("Connecting to instance 1 MAVLink on udp:127.0.0.1:18571 ...")
conn = mavutil.mavlink_connection('udpout:127.0.0.1:18571', source_system=255)

print("Sending our own heartbeat so PX4 learns our address...")
conn.mav.heartbeat_send(
    mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)

print("Waiting for heartbeat...")
hb = conn.wait_heartbeat(timeout=10)
if hb is None:
    print("FAILED: no heartbeat received within 10s")
    raise SystemExit(1)
print(f"Heartbeat received: system={conn.target_system} component={conn.target_component}")

def send_and_wait_ack(name, command, p1=0, p2=0, p3=0, p4=0, p5=0, p6=0, p7=0):
    conn.mav.command_long_send(
        conn.target_system, conn.target_component,
        command, 0, p1, p2, p3, p4, p5, p6, p7)
    print(f"Sent {name} (command={command})")
    deadline = time.time() + 5
    while time.time() < deadline:
        msg = conn.recv_match(type='COMMAND_ACK', blocking=True, timeout=1)
        if msg is not None and msg.command == command:
            print(f"  ACK: command={msg.command} result={msg.result}")
            return msg.result
    print("  NO ACK received within 5s")
    return None

# Switch to OFFBOARD mode: DO_SET_MODE, param1=base_mode (custom enabled),
# param2=custom main mode (PX4's OFFBOARD=6)
result_mode = send_and_wait_ack(
    "DO_SET_MODE (OFFBOARD)", mavutil.mavlink.MAV_CMD_DO_SET_MODE,
    p1=MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, p2=PX4_CUSTOM_MAIN_MODE_OFFBOARD)

time.sleep(1)

# Arm
result_arm = send_and_wait_ack(
    "COMPONENT_ARM_DISARM (arm)", mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    p1=1)

print()
print(f"RESULT: mode_switch_ack={result_mode} arm_ack={result_arm}")
print("(0 = MAV_RESULT_ACCEPTED)")
