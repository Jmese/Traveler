import can
import struct
import time
import math
import numpy as np
from datetime import datetime
import os
import csv
from datetime import datetime

class PDController:
    def __init__(self, Kp, Kd, setpoint):
        self.Kp = Kp
        self.Kd = Kd
        self.setpoint = setpoint
        self.prev_error = 0.0
        self.prev_time = None

    def update(self, measured_val, current_time):
        Proportional_Error = self.setpoint - measured_val
        
        if self.prev_time is None:
            dt = 0.0
        else:
            dt = current_time - self.prev_time
            
        Derivative_Error = (Proportional_Error - self.prev_error) / dt if dt > 0 else 0.0
        
        Corrected_Signal = self.Kp * Proportional_Error + self.Kd * Derivative_Error
        
        self.prev_error = Proportional_Error
        self.prev_time = current_time
        
        
        return Corrected_Signal
    
    def reset(self):
    
        self.prev_error = 0.0
        self.prev_time = None
    
def set_closed_loop_control(node_id):
    bus.send(can.Message(
        arbitration_id=(node_id << 5 | 0x07),
        data=struct.pack('<I', 8),
        is_extended_id=False
    ))
    
def set_torque_control_mode(node_id):
    bus.send(can.Message(
        arbitration_id=(node_id << 5 | 0x0b),
        data=struct.pack('<II', 1, 1),
        is_extended_id=False
    ))

def set_position_control_mode(node_id):
    bus.send(can.Message(
        arbitration_id=(node_id << 5 | 0x0b),
        data=struct.pack('<II', 3, 1),
        is_extended_id=False
    ))

def set_torque(node_id, torque):
    bus.send(can.Message(
        arbitration_id=(node_id << 5 | 0x0e),
        data=struct.pack('<f', torque),
        is_extended_id=False
    ))

def set_position(node_id, position, ff_vel, ff_tor):
    bus.send(can.Message(
        arbitration_id=(node_id << 5 | 0x0c),
        data=struct.pack('<fhh', position, int(ff_vel), ff_tor),
        is_extended_id=False
    ))

def set_idle(node_id):
    bus.send(can.Message(
        arbitration_id=(node_id << 5 | 0x07),
        data=struct.pack('<I', 1),
        is_extended_id=False
    ))
    
def encoder_estimates(node_id):
    bus.send(can.Message(
        arbitration_id=(node_id << 5 | 0x09),
        data=b'',
        is_extended_id=False
    ))
    for msg in bus:
        if msg.arbitration_id == (node_id << 5 | 0x09):
            pos_return_value, vel_return_value = struct.unpack_from('<ff', msg.data)
            return pos_return_value, vel_return_value
        
def get_torque_estimate(node_id):

    bus.send(can.Message(
        arbitration_id=(node_id << 5 | 0x1c), 
        data=b'',
        is_extended_id=False
    ))

    for msg in bus:
        if msg.arbitration_id == (node_id << 5 | 0x1c): 
            break
    
    torque_target, torque_return_value = struct.unpack_from('<ff', msg.data)

    return torque_return_value

def get_state_variables(encoder0_pos_estimate, encoder1_pos_estimate, encoder0_vel_estimate, encoder1_vel_estimate):
    reference_point = 0.25
    phi_1 = (math.pi / 2) + ((reference_point - encoder0_pos_estimate) * (math.pi * 2))
    phi_2 = ((3 * math.pi) / 2) + ((encoder1_pos_estimate - reference_point) * (math.pi * 2))
    
    theta = 0.5 * (phi_1 + phi_2)
    rho = 0.5 * (phi_1 - phi_2) + math.pi

    phi_1_vel = encoder0_vel_estimate
    phi_2_vel = -encoder1_vel_estimate

    Jacobian = np.array([[0.5, 0.5], [0.5, -0.5]])
    Polar_Velocity_Vector = np.array([[phi_1_vel], [phi_2_vel]])
    wibblit_deriv = np.dot(Jacobian, Polar_Velocity_Vector)
    theta_vel = wibblit_deriv[0][0]
    rho_vel = wibblit_deriv[1][0]

    return phi_1, phi_2, phi_1_vel, phi_2_vel, theta, rho, theta_vel, rho_vel

def get_torques(theta_torque, rho_torque):
    wibblit_torques = [[theta_torque], [rho_torque]]
    Jacobian = np.array([[0.5, 0.5], [0.5, -0.5]])
    motor_torque_cmd = np.dot(Jacobian.transpose(), wibblit_torques)
    
    Motor0_Torque = -motor_torque_cmd[0][0]
    Motor1_Torque = motor_torque_cmd[1][0]
    
    return Motor0_Torque, Motor1_Torque

if __name__ == "__main__":
    soft_start_duration = 2.0
    nodes = [0, 1]
    
    target_rho = math.pi/2
    target_theta = 3.14
    
    Theta_PD_Controller = PDController(15, 0.15, target_theta)
    Rho_PD_Controller = PDController(15, 0.05, target_rho)
    
    bus = can.interface.Bus("can0", interface="socketcan")
    
    while not (bus.recv(timeout=0) is None):
        pass

    Motor0, Motor1 = nodes[0], nodes[1]

    set_closed_loop_control(Motor0)
    set_closed_loop_control(Motor1)
    set_torque_control_mode(Motor0)
    set_torque_control_mode(Motor1)

        # Setup data log
    data_log = []

    # Determine elapsed time
    New_Time = time.perf_counter()
    Elapsed_Start_Time = time.perf_counter()
    
    
    try:
        while True:
            New_Time = time.perf_counter()  # Update time inside loop
            current_position_0, current_velocity_0 = encoder_estimates(Motor0)
            current_position_1, current_velocity_1 = encoder_estimates(Motor1)
            motor0_tor = get_torque_estimate(Motor0)
            motor1_tor = get_torque_estimate(Motor1)
            
            elapsed_time = New_Time - Elapsed_Start_Time
            current_time = time.time()  
            
            
            phi_1, phi_2, phi_1_vel, phi_2_vel, theta, rho, theta_vel, rho_vel = get_state_variables(current_position_0, current_position_1, current_velocity_0, current_velocity_1)

            global_time = datetime.now()
            formatted_global_time = global_time.strftime("%Y-%m-%d %H:%M:%S.%f")
            
            data_log.append([formatted_global_time, elapsed_time, current_position_0, current_position_1, motor0_tor, motor1_tor])

            theta_torque = Theta_PD_Controller.update(theta,current_time)
            rho_torque = Rho_PD_Controller.update(rho, current_time)

            Motor0_Torque, Motor1_Torque = get_torques(theta_torque, rho_torque)
            
            set_torque(Motor0, Motor0_Torque)
            set_torque(Motor1, Motor1_Torque)
            
            
    except KeyboardInterrupt:
        print("Keyboard interrupt detected. Setting nodes to idle...")
        for node_id in nodes:
            set_idle(node_id)
        print("Nodes set to idle. Exiting program.")


        # Get the current date and time
        now = datetime.now()

        # Format the filename
        filename = now.strftime("DROP-%H:%M_%m-%d.csv")

        # Set up file to save data 
        file_path = "/home/traveler/Traveler_Hopper_sw-bundle/Data/DROP"
        
        # Ensure the directory exists
        os.makedirs(file_path, exist_ok=True)
        
        full_path = os.path.join(file_path, filename)
        
        # Start Logging Data
        print("Saving Data to: " + full_path)

        # Define file header 
        csv_header = ['Global Time', 'Time', 'Motor 0 Position', 'Motor 1 Position', 'Motor 0 Torque', 'Motor 1 Torque']

        with open(full_path, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(csv_header)
            writer.writerows(data_log)  # Combined writing rows into a single block
            print("Save Complete")

