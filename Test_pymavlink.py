#!/usr/bin/env python3
from pymavlink import mavutil
import time

# Create connection
print("Connecting with pymavlink...")
master = mavutil.mavlink_connection('/dev/serial0', baud=115200)

# Wait for heartbeat
print("Waiting for heartbeat...")
master.wait_heartbeat()
print(f"Heartbeat from system {master.target_system} component {master.target_component}")

# Get some messages
print("\nGetting vehicle data...")

# Request data streams
master.mav.request_data_stream_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_DATA_STREAM_ALL, 4, 1)

# Read some messages
for i in range(10):
    msg = master.recv_match(blocking=True, timeout=1)
    if msg:
        msg_type = msg.get_type()
        if msg_type == "ATTITUDE":
            print(f"Attitude: Roll={msg.roll:.2f}, Pitch={msg.pitch:.2f}, Yaw={msg.yaw:.2f}")
        elif msg_type == "GPS_RAW_INT":
            print(f"GPS: Lat={msg.lat/1e7:.6f}, Lon={msg.lon/1e7:.6f}, Alt={msg.alt/1000:.1f}m")
        elif msg_type == "HEARTBEAT":
            mode = msg.custom_mode
            armed = msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
            print(f"Heartbeat: Mode={mode}, Armed={'Yes' if armed else 'No'}")
        elif msg_type == "VFR_HUD":
            print(f"VFR HUD: Airspeed={msg.airspeed:.1f}, Groundspeed={msg.groundspeed:.1f}, Alt={msg.alt:.1f}")
        elif msg_type == "SYS_STATUS":
            print(f"Battery: {msg.voltage_battery/1000:.2f}V")

print("\nConnection test successful!")
EOF
