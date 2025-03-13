import struct
import json
import time
import can
import traceback

# -- start load
try:
    with open('flat_endpoints.json', 'r') as f:
        endpoint_data = json.load(f)
        endpoints = endpoint_data['endpoints']
except Exception as e:
    print(f"Error loading configuration: {e}")
    exit(1)
# -- end load

# -- start definitions
OPCODE_READ = 0x00
OPCODE_WRITE = 0x01

# Format lookup for data packing
format_lookup = {
    'bool': '?',
    'uint8': 'B', 'int8': 'b',
    'uint16': 'H', 'int16': 'h',
    'uint32': 'I', 'int32': 'i',
    'uint64': 'Q', 'int64': 'q',
    'float': 'f'
}

def send_message(node_id, path, value):
    try:
        endpoint_id = endpoints[path]['id']
        endpoint_type = endpoints[path]['type']
        
        # Send write command
        bus.send(can.Message(
            arbitration_id=(node_id << 5 | 0x04),
            data=struct.pack('<BHB' + format_lookup[endpoint_type], OPCODE_WRITE, endpoint_id, 0, value),
            is_extended_id=False
        ))
        
        # Flush CAN RX buffer to clear old messages
        while bus.recv(timeout=0) is not None:
            pass

        # Send read command to confirm write
        bus.send(can.Message(
            arbitration_id=(node_id << 5 | 0x04),
            data=struct.pack('<BHB', OPCODE_READ, endpoint_id, 0),
            is_extended_id=False
        ))
        
        # Await reply
        for msg in bus:
            if msg.arbitration_id == (node_id << 5 | 0x05):
                break

        # Print confirmation
        _, _, _, return_value = struct.unpack_from('<BHB' + format_lookup[endpoint_type], msg.data)
        print(f"Node {node_id} - {path} Received: {return_value}")
    except Exception as e:
        print(f"Error in send_message for node {node_id}, path '{path}': {e}")
        traceback.print_exc()

def save_config(node_id, path):
    try:
        endpoint_id = endpoints[path]['id']
        
        while bus.recv(timeout=0) is not None:
            pass
        
        # Send save command
        bus.send(can.Message(
            arbitration_id=(node_id << 5 | 0x16),
            data=struct.pack('<I', 1),
            is_extended_id=False
        ))
        
        print(f"Node {node_id} - Configuration saved")
    except Exception as e:
        print(f"Error in save_config for node {node_id}, path '{path}': {e}")
        traceback.print_exc()

def calibrate_motor(node_id):
    try:
        # Command motor to enter full calibration
        CALIBRATION_STATE = 3  # Assuming 3 corresponds to calibration state
        
        bus.send(can.Message(
            arbitration_id=(node_id << 5 | 0x07),
            data=struct.pack('<I', CALIBRATION_STATE),
            is_extended_id=False
        ))
        print(f"Motor {node_id} calibration started")

        # Monitor for calibration completion by checking state change
        for msg in bus:
            if msg.arbitration_id == (node_id << 5 | 0x01):
                error, state, result, traj_done = struct.unpack('<IBBB', bytes(msg.data[:7]))
                if state != CALIBRATION_STATE:  # Exit loop when calibration finishes
                    print(f"Motor {node_id} calibration complete")
                    break
    except Exception as e:
        print(f"Error in calibrate_motor for node {node_id}: {e}")
        traceback.print_exc()

def main():
    try:
        node_id_0 = 0
        node_id_1 = 1

        global bus
        bus = can.interface.Bus("can0", interface="socketcan")
        
        # -- start version check
        while bus.recv(timeout=0) is not None:
            pass

        # Send version check
        bus.send(can.Message(
            arbitration_id=(node_id_0 << 5 | 0x00),
            data=b'',
            is_extended_id=False
        ))

        # Await version reply
        for msg in bus:
            if msg.arbitration_id == (node_id_0 << 5 | 0x00):
                break

        unpacked = struct.unpack('<BBBBBBBB', msg.data)
        (_, hw_product_line, hw_version, hw_variant,
         fw_major, fw_minor, fw_revision, fw_unreleased) = unpacked

        assert endpoint_data['fw_version'] == f"{fw_major}.{fw_minor}.{fw_revision}", "Firmware version mismatch"
        assert endpoint_data['hw_version'] == f"{hw_product_line}.{hw_version}.{hw_variant}", "Hardware version mismatch"
        # -- end version check

        # -- start write
        path_pos = 'axis0.controller.config.pos_gain'
        path_vel = 'axis0.controller.config.vel_gain'
        path_int = 'axis0.controller.config.vel_integrator_gain'
        path_save_config = 'save_configuration'

        pos_gain = 120
        vel_gain = 0.825
        int_gain = 0.0

        # Write gains and save for each motor
        send_message(node_id_0, path_pos, pos_gain)
        send_message(node_id_0, path_vel, vel_gain)
        send_message(node_id_0, path_int, int_gain)
        save_config(node_id_0, path_save_config)

        send_message(node_id_1, path_pos, pos_gain)
        send_message(node_id_1, path_vel, vel_gain)
        send_message(node_id_1, path_int, int_gain)
        save_config(node_id_1, path_save_config)

        time.sleep(9)
        # Calibrate each motor
        calibrate_motor(node_id_0)
        time.sleep(9)
        calibrate_motor(node_id_1)
        # -- end write

        print("Success: All operations completed without errors.")
    except Exception as e:
        print(f"Error occurred in main: {e}")
        traceback.print_exc()

if __name__ == "__main__":
    main()
