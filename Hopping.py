import numpy as np
import can 
import struct 
import time 
import odrive
import math
import argparse
from odrive.enums import *
import matplotlib.pyplot as plt
from pynput import keyboard
import csv
import os
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

class Hopper_State_Machine:
    def __init__(self):
        self.state = "idle"
        self.state_start_time = None
        self.state_durations = {}
        self.prev_rho_dot = None
        self.waiting = False
        self.wait_start_time = None
        self.wait_duration = 1  # Wait time in seconds
        
    def get_state(self, rho, rho_dot, current_time, Previous_State, latch_status):
        
        if self.state_start_time is None:
            self.state_start_time = current_time
        
        
        if self.state != "idle" and not self.waiting:
            duration = current_time - self.state_start_time
            if self.state in self.state_durations:
                self.state_durations[self.state] += duration
            else:
                self.state_durations[self.state] = duration
                
            
        if self.prev_rho_dot is None:
            self.prev_rho_dot = rho_dot
            
        Extension_Flight_Target_Rho = 2.9
        Compression_Target_Rho = 0.7
        Rho_Tolerance = 0.25  # Tolerance for rho matching targets

        Compression_Rho_Error = abs(Compression_Target_Rho - rho) / Compression_Target_Rho
        Extension_Rho_Error = abs(Extension_Flight_Target_Rho - rho) / Extension_Flight_Target_Rho

        rho_dot_increasing = rho_dot > self.prev_rho_dot
        rho_dot_decreasing = rho_dot < self.prev_rho_dot
        
        self.prev_rho_dot = rho_dot
        
        if latch_status == 0: 
            return "idle"

        elif latch_status == 1:
            
            if self.waiting:
                if current_time - self.wait_start_time >= self.wait_duration:
                    self.waiting = False
                    self.state = "extension"
                    return "extension"
                else:
                    return "compression"
                
            # Flight to compression: rho close to compression target and rho_dot negative
            if (self.state == "flight" and Compression_Rho_Error < Rho_Tolerance and rho_dot_decreasing):
                self.state = "compression"
                return "compression"

            # Compression to extension: rho close to extension target and rho_dot positive
            elif (self.state == "compression" and Compression_Rho_Error < Rho_Tolerance and rho_dot_increasing):
                self.waiting = True
                self.wait_start_time = current_time
                return "compression"

            # Maintain flight phase when rho close to flight target
            elif Extension_Rho_Error < Rho_Tolerance:
                self.state = "flight"
                self.state_start_time = current_time
                return "flight"

            else:
                # If no condition is met, stay in the previous state
                return Previous_State


def get_state_variables(encoder0_pos_estimate, encoder1_pos_estimate, encoder0_vel_estimate, encoder1_vel_estimate):
    reference_point = 0.25
    #Converting the absolute encoder estimated into radians with refernce the the vertical axis (this is need as it was the way the jacobian was derived)
    phi_1 = (math.pi/2) + ((reference_point - encoder0_pos_estimate) * (math.pi*2))
    phi_2 = ((3*math.pi)/2) + ((encoder1_pos_estimate - reference_point) * (math.pi*2))

    #The velcoities estimates must be slightly modified to get ensure that they corespond to the definition of phi_1 and phi_2
    #The velocity of encoder1 must be switched to negative as phi2 is measured cw with respect to the vertical axis but the encoder velocity is 
    #taken ccw with respect to the absolute zero position. 
    phi_1_vel = encoder0_vel_estimate
    phi_2_vel = -encoder1_vel_estimate
    
    #phi1 and phi2 are actually swithched in our set up when comparing it to the derived kinematics
    Upper_Link_Len = 0.100 #m
    Lower_Link_Len = 0.200 #mqq
    Toe_Len = 0.0475 #m
    theta = 0.5*(phi_1 + phi_2)
    gamma = 0.5*(-phi_2 + phi_1)
    length = Toe_Len + Upper_Link_Len*math.cos(gamma) + math.sqrt((Lower_Link_Len)**2 - (Upper_Link_Len**2)*(math.sin(gamma)**2))
    #print(length)
    beta = -Upper_Link_Len*math.sin(gamma)*(1 + ((Upper_Link_Len * math.cos(gamma))/math.sqrt((Lower_Link_Len)**2 - ((Upper_Link_Len)**2)*(math.sin(gamma)**2))))
    Leg_Geometry_Dict = {
        "Leg Length": length,
        "Leg Angle": theta, 
        "Beta": beta
    }

    #Calculate Wibblit state variables
    #Using the calculated values for phi1 and phi2 the new coordinate system can be calculated in terms of theta and rho (aka wibblets)
    theta = 0.5*(phi_1 + phi_2)
    rho = 0.5*(phi_1 - phi_2) + math.pi
    print(rho)

    #set up jacobian to convert polar to wibblit system
    Jacobian = np.array([[0.5, 0.5], [0.5, -0.5]])
    Polar_Velocity_Vector = [[phi_1_vel], [phi_2_vel]]
    
    #Map the angular velocities of phi1 and phi2 to the wibblit velocities using the perviously defined jacobian 
    wibblit_deriv = np.dot(Jacobian, Polar_Velocity_Vector)
    theta_vel= Polar_Velocity_Vector[0][0]
    rho_vel = Polar_Velocity_Vector[1][0]
    #print(phi_1, phi_2)
    return Leg_Geometry_Dict, phi_1, phi_2, phi_1_vel, phi_2_vel, theta, rho, theta_vel, rho_vel


def FK(length, theta):
    x = length*math.sin(theta)
    z = abs(length*math.cos(theta))
    Coords = (x,z)
    return Coords

def IK(x,z):
    theta = math.atan2(x,z)
    length = math.sqrt(x**2 + z**2)
    Polar_Coords = (length, theta)
    return Polar_Coords

def get_Jacobian(length, theta, beta):
    Jacobian = np.array([[0.5*(-beta*math.sin(theta) + length*math.cos(theta)), 0.5*(beta*math.sin(theta) + length*math.cos(theta))], 
                         [ 0.5*(-beta *math.cos(theta) - length*math.sin(theta)), 0.5*(beta*math.cos(theta) - length*math.sin(theta))]])
    return Jacobian

def get_Torques(Jacobian, Forces):
    Torques = np.dot(Jacobian.transpose(), Forces)
    Max_Torque = 4 #Nm
    for Torque in Torques:
        if Torque[0] > Max_Torque:
            Torque = Max_Torque
    Motor1_Torque = -Torques[0][0]
    Motor2_Torque = Torques[1][0]
    return Motor1_Torque, Motor2_Torque

def get_Torques_from_Wibblits(theta_torque, rho_torque):
    wibblit_torques = [[theta_torque], [rho_torque]]
    Jacobian = np.array([[0.5, 0.5], [0.5, -0.5]])
    motor_torque_cmd = np.dot(Jacobian.transpose(), wibblit_torques)
    
    Motor0_Torque = -motor_torque_cmd[0][0]
    Motor1_Torque = motor_torque_cmd[1][0]
    
    return Motor0_Torque, Motor1_Torque
# Function to put a node into closed loop control mode
def set_closed_loop_control(node_id):
    bus.send(can.Message(
        arbitration_id=(node_id << 5 | 0x07),  # 0x07: Set_Axis_State
        data=struct.pack('<I', 8),  # 8: AxisState.CLOSED_LOOP_CONTROL
        is_extended_id=False
    ))

# Function to put a node into closed loop control mode
def set_torque_control_mode(node_id):
    bus.send(can.Message(
        arbitration_id=(node_id << 5 | 0x0b),  # 0x0b: Set_Controller_Mode
        data=struct.pack('<II', 1, 1),  # 1: Torque control, 1:Input pass through mode
        is_extended_id=False
    ))

def set_position_control_mode(node_id):
    bus.send(can.Message(
        arbitration_id=(node_id << 5 | 0x0b),  # 0x0b: Set_Controller_Mode
        data=struct.pack('<II', 3, 1),  # 3: Position Control, 1: Input pass through mode
        is_extended_id=False
    ))

# Function to set a torque command for a node
def set_torque(node_id, torque):
    bus.send(can.Message(
        arbitration_id=(node_id << 5 | 0x0e),  # 0x0e: Set_Input_Torque
        data=struct.pack('<f', torque),  # Set desired torque
        is_extended_id=False
    ))

def set_position(node_id, position):
    bus.send(can.Message(
        arbitration_id=(node_id << 5 | 0x0c),
        data=struct.pack('<f', position),
        is_extended_id=False
    ))

    
def set_idle(node_id):
    bus.send(can.Message(
        arbitration_id=(node_id << 5 | 0x07),
        data=struct.pack('<I', 1),
        is_extended_id=False
    ))

def get_encoder_estimate(node_id):
    bus.send(can.Message(
        arbitration_id=(node_id << 5 | 0x09), # 0x04: RxSdo
        data=b'',
        is_extended_id=False
    ))

    # Await reply
    for msg in bus:
        if msg.arbitration_id == (node_id << 5 | 0x09):  # Response with encoder data
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
        
    # Unpack and print reply
   # _, _, _, torque_return_value = struct.unpack_from('<BHB' + 'f', msg.data)
    
    torque_target, torque_return_value = struct.unpack_from('<ff', msg.data)

    return torque_return_value

def get_Forces(Jacobian, Torque_Vector):
    Jacobian_Inverse = np.linalg.inv(Jacobian)
    Forces = np.dot(Jacobian_Inverse.transpose(), Torque_Vector)
    Force_Magnitude = math.hypot(Forces[0][0], Forces[1][0])

    return Force_Magnitude

        
if __name__ == "__main__":
    running = True 

    nodes = [0, 1]  # Node IDs for node 0 and node 1
    
    bus = can.interface.Bus("can0", interface="socketcan")
    
    OPCODE_READ = 0x00
    OPCODE_WRITE = 0x01
    
    # Flush CAN RX buffer so there are no more old pending messages
    while not (bus.recv(timeout=0) is None): 
        pass
    
    # Put each node into closed loop control mode
    # Set up motor parameters 
    Motor1 = nodes[0]  # Front right linkage (phi2)
    Motor2 = nodes[1]  # Back left linkage (phi1)

    # Set Closed loop control 
    set_closed_loop_control(Motor1)
    set_closed_loop_control(Motor2)

    # Wait for each node to enter closed loop control by scanning heartbeat messages
    for node_id in nodes:
        for msg in bus:
            if msg.arbitration_id == (node_id << 5 | 0x01):  # 0x01: Heartbeat
                error, state, result, traj_done = struct.unpack('<IBBB', bytes(msg.data[:7]))
                if state == 8:  # 8: AxisState.CLOSED_LOOP_CONTROL
                    break

    # Initialize state machine
    State_Machine = Hopper_State_Machine()
    Previous_State = "compression"

    # Initialize PD controllers
    Extension_Flight_Target_Theta = 3.1637
    Extension_Flight_Target_Rho = 3.05
    Extension_Theta_PD_Controller = PDController(8, 0.15, Extension_Flight_Target_Theta)
    Extension_Rho_PD_Controller = PDController(8, 0.15, Extension_Flight_Target_Rho)

    Compression_Target_Theta = 3.1637
    Compression_Target_Rho = 0.7
    Compression_Theta_PD_Controller = PDController(0.5, 0.15, Compression_Target_Theta)
    Compression_Rho_PD_Controller = PDController(0.5, 0.15, Compression_Target_Rho)

    Flight_Theta_PD_Controller = PDController(0.55, 0.09, Extension_Flight_Target_Theta)
    Flight_Rho_PD_Controller = PDController(0.55, 0.09, Extension_Flight_Target_Rho)
    
    # Initialize other parameters
    latch_status = 0
    data_log = []
    State_Vector = []

    # Get the initial motor position estimates
    initial_motor1_pos, initial_motor1_vel = get_encoder_estimate(Motor1)
    initial_motor2_pos, initial_motor2_vel = get_encoder_estimate(Motor2)
    
    Start_Time = time.perf_counter()
    Elapsed_Start_Time = time.perf_counter()
    
    Initial_Leg_Geometry, initial_phi_1, initial_phi_2, initial_phi_1_vel, initial_phi_2_vel, initial_theta, initial_rho, initial_theta_vel, initial_rho_vel  = get_state_variables(initial_motor1_pos, initial_motor2_pos, initial_motor1_vel, initial_motor2_vel)

try:
    while running:
        # Get motor position and velocity estimates
        motor1_pos, motor1_vel = get_encoder_estimate(Motor1)
        motor2_pos, motor2_vel = get_encoder_estimate(Motor2)
        motor1_tor = get_torque_estimate(Motor1)
        motor2_tor = get_torque_estimate(Motor2)
        
        # Calculate leg state variables
        Leg_Geometry, phi_1, phi_2, phi_1_vel, phi_2_vel, theta, rho, theta_vel, rho_vel  = get_state_variables(motor1_pos, motor2_pos, motor1_vel, motor2_vel)
        
        # Determine elapsed time
        New_Time = time.perf_counter()
        elapsed_time = New_Time - Elapsed_Start_Time

        # Check and save the system state
        State = State_Machine.get_state(rho, rho_vel, New_Time, Previous_State, latch_status)
        print(State)
        
        global_time = datetime.now()
        formatted_global_time = global_time.strftime("%Y-%m-%d %H:%M:%S.%f")
                
        # Save data
        data_log.append([formatted_global_time, elapsed_time, motor1_pos, motor2_pos, motor1_tor, motor2_tor, State])

        # State actions
        if State == "idle":
            latch_status = 1

        elif State == "compression":
            current_time = time.time()
            theta_torque = Compression_Theta_PD_Controller.update(theta, current_time)
            rho_torque = Compression_Rho_PD_Controller.update(rho, current_time)
            Motor1_Torque, Motor2_Torque = get_Torques_from_Wibblits(theta_torque, rho_torque)
            set_torque(Motor1, Motor1_Torque)
            set_torque(Motor2, Motor2_Torque)

        elif State == "extension":
            current_time = time.time()
            theta_torque = Extension_Theta_PD_Controller.update(theta, current_time)
            rho_torque = Extension_Rho_PD_Controller.update(rho, current_time)
            Motor1_Torque, Motor2_Torque = get_Torques_from_Wibblits(theta_torque, rho_torque)
            set_torque(Motor1, Motor1_Torque)
            set_torque(Motor2, Motor2_Torque)

        elif State == "flight":
            current_time = time.time()
            theta_torque = Flight_Theta_PD_Controller.update(theta, current_time)
            rho_torque = Flight_Rho_PD_Controller.update(rho, current_time)
            Motor1_Torque, Motor2_Torque = get_Torques_from_Wibblits(theta_torque, rho_torque)
            set_torque(Motor1, Motor1_Torque)
            set_torque(Motor2, Motor2_Torque)
        
        # Redefine the initial motor positions
        Start_Time = New_Time
        Previous_State = State
    
except KeyboardInterrupt:
    print("Keyboard interrupt detected. Setting nodes to idle...")
    for node_id in nodes:
        set_idle(node_id)
    print("Nodes set to idle. Exiting program.")
