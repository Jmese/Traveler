#ifndef CAN_INTERFACE_H
#define CAN_INTERFACE_H

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cstring>
#include <iostream>
#include <tuple>

class CANInterface {
public:
    CANInterface(const std::string& interface_name);
    ~CANInterface();

    bool send(const can_frame& frame);
    bool receive(can_frame& frame, int timeout_ms = 1000);
    void flush();

    static void setClosedLoopControl(int node_id);
    static void setTorqueControlMode(int node_id);
    static void setPositionControlMode(int node_id);
    static void setTorque(int node_id, float torque);
    static void setPosition(int node_id, float position, int ff_vel, int ff_tor);
    static void setIdle(int node_id);
    static std::tuple<float, float> getEncoderEstimates(int node_id);
    static std::tuple<float, float> getTorques(float theta_torque, float rho_torque);

private:
    int socket_fd;
    struct sockaddr_can addr;
    struct ifreq ifr;
};

#endif // CAN_INTERFACE_H