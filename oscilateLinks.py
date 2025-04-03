import can
import struct
import time
import csv
import math
import numpy as np

def set_closed_loop_control(node_id):
    bus.send(can.Message(
        arbitration_id=(node_id << 5 | 0x07),
        data=struct.pack('<I', 8),
        is_extended_id=False
    ))

def set_velocity_control_mode(node_id):
    bus.send(can.Message(
        arbitration_id=(node_id << 5 | 0x0b),
        data=struct.pack('<II', 2, 1),
        is_extended_id=False
    ))

def set_velocity(node_id, velocity):
    bus.send(can.Message(
        arbitration_id=(node_id << 5 | 0x0d),
        data=struct.pack('<f', velocity),
        is_extended_id=False
    ))

def get_velocity(node_id):
    bus.send(can.Message(
        arbitration_id=(node_id << 5 | 0x09),
        data=b'',
        is_extended_id=False
    ))
    for msg in bus:
        if msg.arbitration_id == (node_id << 5 | 0x09):
            _, velocity = struct.unpack_from('<ff', msg.data)
            return velocity

def set_idle(node_id):
    bus.send(can.Message(
        arbitration_id=(node_id << 5 | 0x07),
        data=struct.pack('<I', 1),
        is_extended_id=False
    ))

if __name__ == "__main__":
    nodes = [0, 1]
    
    # User-defined parameters
    n_repetitions = int(input("Enter the number of repetitions: "))
    angular_velocity = float(input("Enter the angular velocity (rad/s): "))
    log_file = "motor_velocity_log.csv"

    bus = can.interface.Bus("can0", interface="socketcan")
    
    while not (bus.recv(timeout=0) is None):
        pass

    Motor0, Motor1 = nodes[0], nodes[1]

    set_closed_loop_control(Motor0)
    set_closed_loop_control(Motor1)
    set_velocity_control_mode(Motor0)
    set_velocity_control_mode(Motor1)

    # Open the CSV file for logging
    with open(log_file, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(["Time", "TTheta1", "TTheta2"])

        start_time = time.time()

        try:
            for _ in range(n_repetitions):
                # Move forward
                set_velocity(Motor0, angular_velocity)
                set_velocity(Motor1, angular_velocity)
                for _ in range(100):  # Log at high frequency during motion
                    elapsed_time = time.time() - start_time
                    vel0 = get_velocity(Motor0)
                    vel1 = get_velocity(Motor1)
                    writer.writerow([elapsed_time, vel0, vel1])
                    time.sleep(0.01)  # Adjust for higher frequency if needed

                # Move backward
                set_velocity(Motor0, -angular_velocity)
                set_velocity(Motor1, -angular_velocity)
                for _ in range(100):  # Log at high frequency during motion
                    elapsed_time = time.time() - start_time
                    vel0 = get_velocity(Motor0)
                    vel1 = get_velocity(Motor1)
                    writer.writerow([elapsed_time, vel0, vel1])
                    time.sleep(0.01)  # Adjust for higher frequency if needed

            # Stop the motors after completing repetitions
            set_velocity(Motor0, 0)
            set_velocity(Motor1, 0)

        except KeyboardInterrupt:
            print("Keyboard interrupt detected. Setting nodes to idle...")
        finally:
            # Set nodes to idle after oscillations
            for node_id in nodes:
                set_idle(node_id)
            print("Nodes set to idle. Exiting program.")