import os
import time
import board
import busio
import struct
import threading
from queue import Queue
import pigpio  # Import pigpio for PWM measurement

from adafruit_bno08x import BNO_REPORT_GYROSCOPE, BNO_REPORT_ACCELEROMETER
from adafruit_bno08x.i2c import BNO08X_I2C

# Initialize I2C bus 1 (default I2C bus on Raspberry Pi)
i2c1 = busio.I2C(board.SCL, board.SDA)

# Initialize I2C bus 0 (pins 27 and 28: SDA1 and SCL1)
i2c0 = busio.I2C(board.SCL, board.SDA)

# Scan for I2C devices on I2C1
print("Scanning for I2C devices on I2C1...")
while not i2c1.try_lock():
    pass
devices_i2c1 = i2c1.scan()
i2c1.unlock()

if not devices_i2c1:
    print("No I2C devices found on I2C1. Please check your connections.")
    while True:
        pass  # Halt execution if no devices are found

print("I2C1 devices found:", [hex(device) for device in devices_i2c1])

# Scan for I2C devices on I2C0
print("Scanning for I2C devices on I2C0...")
while not i2c0.try_lock():
    pass
devices_i2c0 = i2c0.scan()
i2c0.unlock()

if not devices_i2c0:
    print("No I2C devices found on I2C0. Please check your connections.")
    while True:
        pass  # Halt execution if no devices are found

print("I2C0 devices found:", [hex(device) for device in devices_i2c0])

# Initialize sensors on I2C1
if 0x4A not in devices_i2c1 or 0x4B not in devices_i2c1:
    print("Required BNO085 sensors not found at addresses 0x4A and 0x4B on I2C1.")
    while True:
        pass  # Halt execution if the required sensors are not found

theta1_i2c1 = BNO08X_I2C(i2c1, address=0x4A)  # Sensor at address 0x4A
theta2_i2c1 = BNO08X_I2C(i2c1, address=0x4B)  # Sensor at address 0x4B

# Initialize sensors on I2C0
if 0x4A not in devices_i2c0 or 0x4B not in devices_i2c0:
    print("Required BNO085 sensors not found at addresses 0x4A and 0x4B on I2C0.")
    while True:
        pass  # Halt execution if the required sensors are not found

theta1_i2c0 = BNO08X_I2C(i2c0, address=0x4A)  # Sensor at address 0x4A
theta2_i2c0 = BNO08X_I2C(i2c0, address=0x4B)  # Sensor at address 0x4B

# Enable features for sensors on I2C1
for _ in range(3):
    try:
        theta1_i2c1.enable_feature(BNO_REPORT_GYROSCOPE)
        break
    except RuntimeError as e:
        print(f"Retrying Theta 1 Gyro feature on I2C1: {e}")
        

for _ in range(3):
    try:
        theta2_i2c1.enable_feature(BNO_REPORT_GYROSCOPE)
        break
    except RuntimeError as e:
        print(f"Retrying Theta 2 Gyro feature on I2C1: {e}")
        

# Enable features for sensors on I2C0
for _ in range(3):
    try:
        theta1_i2c0.enable_feature(BNO_REPORT_ACCELEROMETER)
        break
    except RuntimeError as e:
        print(f"Retrying Theta 1 Gyro feature on I2C0: {e}")
       

for _ in range(3):
    try:
        theta2_i2c0.enable_feature(BNO_REPORT_ACCELEROMETER)
        break
    except RuntimeError as e:
        print(f"Retrying Theta 2 Gyro feature on I2C0: {e}")
       

# Ensure the 'Data' folder exists
data_folder = "/home/traveler/Traveler_Hopper_sw-bundle/Data/IMU"
os.makedirs(data_folder, exist_ok=True)

# Set up the default log file path
default_log_file_path = os.path.join(data_folder, "imuTemp.txt")

# Queue for buffering data rows
data_queue = Queue(maxsize=10000)

# Function to handle file writing in a separate thread
def log_writer(file_path, queue):
    with open(file_path, mode="w") as log_file:
        # Write the headers to the file
        log_file.write("time,ttheta1,ttheta2,bodyX,bodyY,bodyZ,toeX,toeY,toeZ,dutyCycle\n")
        
        while True:
            data_row = queue.get()
            if data_row is None:  # Sentinel value to stop the thread
                break
            log_file.write(data_row)
        log_file.flush()

# Start the logging thread
log_thread = threading.Thread(target=log_writer, args=(default_log_file_path, data_queue))
log_thread.start()

# Record the start time in seconds
start_time = time.monotonic_ns() / 1_000_000_000  # Convert nanoseconds to seconds

total_rows_logged = 0  # Counter for total rows logged

# Initialize pigpio
pi = pigpio.pi()
if not pi.connected:
    print("Failed to connect to pigpio daemon.")
    exit()

# GPIO pin for PWM signal
PWM_GPIO = 21  # GPIO21
pi.set_mode(PWM_GPIO, pigpio.INPUT)

# Variables for PWM measurement
last_tick = 0
high_time = 0
low_time = 0
measuring_high = True

# Callback function for PWM measurement
def measure_pwm(gpio, level, tick):
    global last_tick, high_time, low_time, measuring_high, pwm_duty_cycle
    current_tick = tick
    delta = pigpio.tickDiff(last_tick, current_tick)
    last_tick = current_tick

    if measuring_high:
        high_time = delta
        measuring_high = False
    else:
        low_time = delta
        measuring_high = True

    if high_time > 0 and low_time > 0:
        period = high_time + low_time
        pwm_duty_cycle = (high_time / period) * 100
    else:
        pwm_duty_cycle = 0

# Set up a callback to monitor GPIO state changes
cb = pi.callback(PWM_GPIO, pigpio.EITHER_EDGE, measure_pwm)

# Initialize PWM duty cycle
pwm_duty_cycle = 0

print("Logging IMU data... Press Ctrl+C to stop.")
try:
    while True:
        current_time = time.monotonic_ns() / 1_000_000_000
        elapsed_time = current_time - start_time

        # Read gyroscope Z-axis data from I2C1
        _, _, theta1_gyro_z_i2c1 = theta1_i2c1.gyro
        _, _, theta2_gyro_z_i2c1 = theta2_i2c1.gyro

        # Read accelerometer X, Y, Z data from I2C0
        theta1_accel_x_i2c0, theta1_accel_y_i2c0, theta1_accel_z_i2c0 = theta1_i2c0.acceleration
        theta2_accel_x_i2c0, theta2_accel_y_i2c0, theta2_accel_z_i2c0 = theta2_i2c0.acceleration

        # Format the data row with IMU and PWM duty cycle data
        data_row = (
            f"{elapsed_time:.6f},"
            f"{theta1_gyro_z_i2c1:.6f},{theta2_gyro_z_i2c1:.6f},"
            f"{theta1_accel_x_i2c0:.6f},{theta1_accel_y_i2c0:.6f},{theta1_accel_z_i2c0:.6f},"
            f"{theta2_accel_x_i2c0:.6f},{theta2_accel_y_i2c0:.6f},{theta2_accel_z_i2c0:.6f},"
            f"{pwm_duty_cycle:.2f}\n"
        )

        # Add the data row to the queue
        data_queue.put(data_row)

        total_rows_logged += 1

        # Sleep to reduce CPU usage
        time.sleep(0.001)

except KeyboardInterrupt:
    print("Stopping...")

finally:
    # Clean up resources
    cb.cancel()  # Cancel the pigpio callback
    pi.stop()  # Stop pigpio
    data_queue.put(None)
    log_thread.join()

    # Prompt the user for a new file name
    new_file_name = input("Enter a name for the log file (without extension): ")
    new_file_path = os.path.join(data_folder, f"{new_file_name}.txt")

    # Rename the default log file to the new file name
    os.rename(default_log_file_path, new_file_path)

    # Calculate and display logging statistics
    total_elapsed_time = time.monotonic_ns() / 1_000_000_000 - start_time
    average_frequency = total_rows_logged / total_elapsed_time
    print(f"Average logging frequency: {average_frequency:.2f} rows per second")
    print(f"Log file saved as: {new_file_path}")
    print("DONE B)")