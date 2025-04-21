import can
import struct
import time
import math
import numpy as np
from datetime import datetime
import os
import argparse
import csv

# ------------------------- copy your utility functions -------------------------
class PDController:
    def __init__(self, Kp, Kd, setpoint):
        self.Kp = Kp; self.Kd = Kd; self.setpoint = setpoint
        self.prev_error = 0.0; self.prev_time = None

    def update(self, measured_val, current_time):
        error = self.setpoint - measured_val
        dt = 0.0 if self.prev_time is None else current_time - self.prev_time
        derivative = (error - self.prev_error)/dt if dt>0 else 0.0
        u = self.Kp*error + self.Kd*derivative
        self.prev_error, self.prev_time = error, current_time
        return u

    def reset(self):
        self.prev_error = 0.0; self.prev_time = None

def set_closed_loop_control(node_id):
    bus.send(can.Message(arbitration_id=(node_id<<5|0x07),
                         data=struct.pack('<I',8), is_extended_id=False))

def set_torque_control_mode(node_id):
    bus.send(can.Message(arbitration_id=(node_id<<5|0x0b),
                         data=struct.pack('<II',1,1), is_extended_id=False))

def set_torque(node_id, torque):
    bus.send(can.Message(arbitration_id=(node_id<<5|0x0e),
                         data=struct.pack('<f',torque), is_extended_id=False))

def encoder_estimates(node_id):
    bus.send(can.Message(arbitration_id=(node_id<<5|0x09),
                         data=b'', is_extended_id=False))
    for msg in bus:
        if msg.arbitration_id==(node_id<<5|0x09):
            return struct.unpack_from('<ff', msg.data)

def get_state_variables(p0,p1,v0,v1):
    ref=0.25
    phi1=(math.pi/2)+((ref-p0)*2*math.pi)
    phi2=(3*math.pi/2)+((p1-ref)*2*math.pi)
    th=0.5*(phi1+phi2); ro=0.5*(phi1-phi2)+math.pi
    J=np.array([[0.5,0.5],[0.5,-0.5]])
    vel_vec=np.dot(J,[[v0],[-v1]])
    return phi1,phi2,v0,-v1,th,ro,vel_vec[0][0],vel_vec[1][0]

def get_torques(th_u,ro_u):
    wvec=np.array([[th_u],[ro_u]])
    J=np.array([[0.5,0.5],[0.5,-0.5]])
    mvec=J.T.dot(wvec)
    return -mvec[0,0], mvec[1,0]
# ------------------------------------------------------------------------------

def main():
    p = argparse.ArgumentParser()
    p.add_argument('-n','--divisions', type=int, default=None,
                   help='number of intermediate setpoint stages')
    args = p.parse_args()
    # if not provided on CLI, prompt now
    if args.divisions is None:
        args.divisions = int(input("Enter number of divisions: ").strip())

    # user‐defined start/end
    start_theta = 3.14
    start_rho   = 0.6
    final_theta = 3.14
    final_rho   = 3.14
    pause_dur   = 2.0  # seconds

    nodes = [0,1]
    global bus
    bus = can.interface.Bus("can0", interface="socketcan")
    # flush
    while bus.recv(timeout=0)!=None: pass

    # setup
    for m in nodes:
        set_closed_loop_control(m)
        set_torque_control_mode(m)

    # initial read to init controllers
    p0p, p0v = encoder_estimates(0)
    p1p, p1v = encoder_estimates(1)
    _, _, _, _, th, ro, _, _ = get_state_variables(p0p, p1p, p0v, p1v)
    theta_ctl = PDController(13,0.2,th)
    rho_ctl   = PDController(18,0.2,ro)

    # compute increments
    n    = args.divisions
    d_th = (final_theta - start_theta)/n
    d_ro = (final_rho   - start_rho)/n

    # Setup data logging
    data_log = []
    elapsed_start_time = time.perf_counter()

    try:
        for i in range(n+1):
            target_th = start_theta + i*d_th
            target_ro = start_rho   + i*d_ro
            theta_ctl.setpoint = target_th
            rho_ctl.setpoint   = target_ro
            theta_ctl.reset(); rho_ctl.reset()

            print(f"Stage {i}/{n}: θ={target_th:.3f}, ρ={target_ro:.3f}")
            t_end = time.time() + pause_dur
            while time.time() < t_end:
                now = time.time()
                (p0p,p0v),(p1p,p1v) = encoder_estimates(0), encoder_estimates(1)
                _,_,_,_,th_est,ro_est,_,_ = get_state_variables(
                    p0p,p1p,p0v,p1v)
                u_th = theta_ctl.update(th_est, now)
                u_ro = rho_ctl.update(ro_est, now)
                m0, m1 = get_torques(u_th,u_ro)
                set_torque(0, m0); set_torque(1, m1)

                # Log data
                elapsed_time = time.perf_counter() - elapsed_start_time
                data_log.append([elapsed_time, p0p, p1p, m0, m1])

                time.sleep(0.01)

        print("All stages complete, setting idle.")
    except KeyboardInterrupt:
        print("Interrupted! idling...")
    finally:
        for m in nodes:
            bus.send(can.Message(arbitration_id=(m<<5|0x07),
                                  data=struct.pack('<I',1), is_extended_id=False))

        # Calculate the average logging frequency
        if len(data_log) > 1:
            time_diffs = [data_log[i][0] - data_log[i-1][0] for i in range(1, len(data_log))]
            avg_time_diff = sum(time_diffs) / len(time_diffs)
            avg_frequency = 1 / avg_time_diff if avg_time_diff > 0 else float('inf')
            print(f"Average data logging frequency: {avg_frequency:.2f} Hz")
        else:
            print("Not enough data points to calculate logging frequency.")

        # Calculate peak torques
        peak_motor0_torque = max(data_log, key=lambda x: x[3])[3] if data_log else 0
        peak_motor1_torque = max(data_log, key=lambda x: x[4])[4] if data_log else 0
        print(f"Peak Motor0 Torque: {peak_motor0_torque:.2f}")
        print(f"Peak Motor1 Torque: {peak_motor1_torque:.2f}")

        # Save data
        save_data = input("Do you want to save the data? (y/n): ").strip().lower()
        if save_data == 'y':
            test_string = input("Enter the TEST string: ").strip()
            now = datetime.now()
            filename = f"{test_string}[{now.strftime('%H.%M_%m.%d')}]_staged.csv"
            file_path = "/home/traveler/Traveler_Hopper_sw-bundle/Data/DPROBE"
            os.makedirs(file_path, exist_ok=True)
            full_path = os.path.join(file_path, filename)

            print("Saving Data to: " + full_path)
            csv_header = ['Elapsed Time', 'Motor 0 Position', 'Motor 1 Position', 'Motor 0 Torque', 'Motor 1 Torque']
            with open(full_path, mode='w', newline='') as file:
                writer = csv.writer(file)
                writer.writerow(csv_header)
                writer.writerows(data_log)
            print("Save Complete")
        else:
            print("Data not saved.")

if __name__ == "__main__":
    main()