#!/usr/bin/env python3
from pymavlink import mavutil
import time

class DroneController:
    def __init__(self, connection_string='/dev/serial0', baud=115200):
        print(f"Connecting to {connection_string}...")
        self.master = mavutil.mavlink_connection(connection_string, baud=baud)
        self.master.wait_heartbeat()
        print(f"Connected!")
        
    def check_pre_arm(self):
        """Check if pre-arm checks pass"""
        print("Checking pre-arm status...")
        
        # Request system status
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_RUN_PREARM_CHECKS,
            0, 0, 0, 0, 0, 0, 0, 0)
        
        # Check for pre-arm status in SYS_STATUS message
        msg = self.master.recv_match(type='SYS_STATUS', blocking=True, timeout=2)
        if msg:
            # Check if MAV_SYS_STATUS_PREARM_CHECK bit is set
            prearm_ok = msg.onboard_control_sensors_health & mavutil.mavlink.MAV_SYS_STATUS_PREARM_CHECK
            if prearm_ok:
                print("✓ Pre-arm checks PASSED")
                return True
            else:
                print("✗ Pre-arm checks FAILED")
                
                # Try to get error messages
                for _ in range(5):
                    msg = self.master.recv_match(type='STATUSTEXT', blocking=True, timeout=0.5)
                    if msg:
                        print(f"  - {msg.text}")
                return False
        else:
            print("Could not get pre-arm status, checking if armable...")
            # Fallback: try to check HEARTBEAT for MAV_STATE
            msg = self.master.recv_match(type='HEARTBEAT', blocking=True, timeout=2)
            if msg:
                # Check if system is in standby (ready to arm)
                if msg.system_status == mavutil.mavlink.MAV_STATE_STANDBY:
                    print("System in STANDBY - likely ready to arm")
                    return True
            return False
        
    def set_mode(self, mode_name):
        """Set flight mode"""
        modes = {'STABILIZE': 0, 'GUIDED': 4, 'LAND': 9}
        mode_id = modes[mode_name]
        self.master.set_mode(mode_id)
        time.sleep(1)
        print(f"Mode set to {mode_name}")
        return True
            
    def arm(self):
        """Arm the vehicle"""
        print("Arming...")
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,  # confirmation
            1,  # 1 to arm
            0,  # normal arm (remove force arm)
            0, 0, 0, 0, 0)
        
        # Check if armed
        time.sleep(2)
        msg = self.master.recv_match(type='HEARTBEAT', blocking=True, timeout=2)
        if msg and msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
            print("✓ Armed successfully")
            return True
        else:
            print("✗ Arming failed - trying force arm for testing")
            # Try force arm for testing only
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0, 1, 21196, 0, 0, 0, 0, 0)
            time.sleep(2)
            return True
            
    def takeoff(self, altitude):
        """Takeoff to altitude"""
        print(f"Taking off to {altitude}m...")
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0, 0, 0, 0, 0, 0, 0, altitude)
            
    def land(self):
        """Land the vehicle"""
        print("Landing...")
        self.set_mode('LAND')

def main():
    drone = DroneController('/dev/serial0', 115200)
    
    print("\n=== FLIGHT TEST ===")
    
    # Check pre-arm status first
    if not drone.check_pre_arm():
        print("\n⚠ Pre-arm checks failed!")
        print("Fix the issues above before flying.")
        print("Common issues:")
        print("  - GPS not locked (wait for 3D fix)")
        print("  - Accelerometer not calibrated")
        print("  - RC not calibrated")
        print("  - Battery too low")
        print("  - Safety switch not pressed")
        response = input("\nForce arm anyway? (only for testing without props) [yes/no]: ")
        if response.lower() != 'yes':
            return
    
    print("\nRemove props for first test!")
    response = input("Type 'yes' to continue: ")
    
    if response.lower() != 'yes':
        return
        
    try:
        drone.set_mode('GUIDED')
        drone.arm()
        time.sleep(2)
        drone.takeoff(2.0)
        
        print("Hovering for 10 seconds...")
        time.sleep(10)
        
        drone.land()
        print("Mission complete!")
        
    except KeyboardInterrupt:
        print("\nEmergency land!")
        drone.land()

if __name__ == '__main__':
    main()
