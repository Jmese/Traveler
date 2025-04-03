#include "CANInterface.h"
#include "utils.h"
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cstring>
#include <iostream>
#include <tuple>
#include <vector>

extern CANInterface canInterface;

CANInterface::CANInterface(const std::string& interface_name) {
    // Create a socket
    socket_fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (socket_fd < 0) {
        std::cerr << "Error while opening socket" << std::endl;
        exit(EXIT_FAILURE);
    }

    // Specify the CAN interface
    strcpy(ifr.ifr_name, interface_name.c_str());
    ioctl(socket_fd, SIOCGIFINDEX, &ifr);

    // Bind the socket to the CAN interface
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(socket_fd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        std::cerr << "Error in socket bind" << std::endl;
        exit(EXIT_FAILURE);
    }
}

CANInterface::~CANInterface() {
    close(socket_fd);
}

bool CANInterface::send(const can_frame& frame) {
    int nbytes = write(socket_fd, &frame, sizeof(frame));
    return nbytes == sizeof(frame);
}

bool CANInterface::receive(can_frame& frame, int timeout_ms) {
    fd_set read_fds;
    struct timeval timeout;

    FD_ZERO(&read_fds);
    FD_SET(socket_fd, &read_fds);

    timeout.tv_sec = timeout_ms / 1000;
    timeout.tv_usec = (timeout_ms % 1000) * 1000;

    int ret = select(socket_fd + 1, &read_fds, nullptr, nullptr, &timeout);
    if (ret > 0 && FD_ISSET(socket_fd, &read_fds)) {
        int nbytes = read(socket_fd, &frame, sizeof(frame));
        return nbytes == sizeof(frame);
    }
    return false;
}

void CANInterface::flush() {
    can_frame frame;
    while (receive(frame, 0)) {
        // Discard the frame
    }
}

void CANInterface::setClosedLoopControl(int node_id) {
    can_frame frame;
    frame.can_id = (node_id << 5 | 0x07);
    frame.can_dlc = 4;
    frame.data[0] = 8;
    frame.data[1] = 0;
    frame.data[2] = 0;
    frame.data[3] = 0;
    canInterface.send(frame);
}

void CANInterface::setTorqueControlMode(int node_id) {
    can_frame frame;
    frame.can_id = (node_id << 5 | 0x0b);
    frame.can_dlc = 8;
    frame.data[0] = 1;
    frame.data[1] = 0;
    frame.data[2] = 0;
    frame.data[3] = 0;
    frame.data[4] = 1;
    frame.data[5] = 0;
    frame.data[6] = 0;
    frame.data[7] = 0;
    canInterface.send(frame);
}

void CANInterface::setPositionControlMode(int node_id) {
    can_frame frame;
    frame.can_id = (node_id << 5 | 0x0b);
    frame.can_dlc = 8;
    frame.data[0] = 3;
    frame.data[1] = 0;
    frame.data[2] = 0;
    frame.data[3] = 0;
    frame.data[4] = 1;
    frame.data[5] = 0;
    frame.data[6] = 0;
    frame.data[7] = 0;
    canInterface.send(frame);
}

void CANInterface::setTorque(int node_id, float torque) {
    can_frame frame;
    frame.can_id = (node_id << 5 | 0x0e);
    frame.can_dlc = 4;
    memcpy(frame.data, &torque, sizeof(float));
    canInterface.send(frame);
}

void CANInterface::setPosition(int node_id, float position, int ff_vel, int ff_tor) {
    can_frame frame;
    frame.can_id = (node_id << 5 | 0x0c);
    frame.can_dlc = 8;
    // Fill in the frame data as needed
}

void CANInterface::setIdle(int node_id) {
    can_frame frame;
    frame.can_id = (node_id << 5 | 0x07);
    frame.can_dlc = 4;
    frame.data[0] = 1;
    frame.data[1] = 0;
    frame.data[2] = 0;
    frame.data[3] = 0;
    canInterface.send(frame);
}

std::tuple<float, float> CANInterface::getEncoderEstimates(int node_id) {
    can_frame frame;
    frame.can_id = (node_id << 5 | 0x09);
    frame.can_dlc = 0;
    canInterface.send(frame);

    const int max_retries = 5;
    int retries = 0;
    while (retries < max_retries) {
        if (canInterface.receive(frame, 2000)) { // Increased timeout to 2000 ms
            if (frame.can_id == (node_id << 5 | 0x09) && frame.can_dlc == 8) {
                float position, velocity;
                memcpy(&position, &frame.data[0], sizeof(float));
                memcpy(&velocity, &frame.data[4], sizeof(float));
                return std::make_tuple(position, velocity);
            } else {
                std::cerr << "Unexpected CAN frame received: ID=" << frame.can_id << ", DLC=" << frame.can_dlc << std::endl;
                for (int i = 0; i < frame.can_dlc; ++i) {
                    //std::cerr << "Data[" << i << "] = " << static_cast<int>(frame.data[i]) << std::endl;
                }
            }
        } else {
            std::cerr << "Failed to receive CAN frame for node " << node_id << " (attempt " << retries + 1 << ")" << std::endl;
        }
        retries++;
    }
    return std::make_tuple(0.0f, 0.0f); // Return default values on failure
}

std::tuple<float, float> CANInterface::getTorqueEstimates(int node_id) {
    can_frame frame;
    frame.can_id = (node_id << 5 | 0x1c); // Command ID for Get_Torques
    frame.can_dlc = 0; // No data payload for this command
    canInterface.send(frame);

    const int max_retries = 5;
    int retries = 0;
    while (retries < max_retries) {
        if (canInterface.receive(frame, 2000)) { // 2000 ms timeout
            if (frame.can_id == (node_id << 5 | 0x1c) && frame.can_dlc == 8) {
                float torque_target, torque_estimate;
                memcpy(&torque_target, &frame.data[0], sizeof(float)); // Bytes 0-3: Torque_Target
                memcpy(&torque_estimate, &frame.data[4], sizeof(float)); // Bytes 4-7: Torque_Estimate
                return std::make_tuple(torque_target, torque_estimate);
            } else {
                std::cerr << "Unexpected CAN frame received: ID=" << frame.can_id << ", DLC=" << frame.can_dlc << std::endl;
            }
        } else {
            std::cerr << "Failed to receive CAN frame for node " << node_id << " (attempt " << retries + 1 << ")" << std::endl;
        }
        retries++;
    }
    return std::make_tuple(0.0f, 0.0f); // Return default values on failure
}

std::tuple<float, float> CANInterface::getTorques(float theta_torque, float rho_torque) {
    // Implementation for getting torques
    return std::make_tuple(0.0f, 0.0f); // Placeholder return value
}