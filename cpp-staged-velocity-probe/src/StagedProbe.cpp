#include "StagedProbe.h"
#include "utils.h"
#include "CANInterface.h"
#include "PDController.h"
#include "GPIOInterface.h"
#include <iostream>
#include <fstream>
#include <chrono>
#include <thread>
#include <tuple>
#include <cmath>
#include <vector>
#include <array>

using namespace std;
using namespace std::chrono;

StagedProbe::StagedProbe(int divisions, float start_theta, float start_rho, float final_theta, float final_rho, float pause_duration)
    : divisions(divisions), start_theta(start_theta), start_rho(start_rho), final_theta(final_theta), final_rho(final_rho), pause_duration(pause_duration),
      thetaController(1.0, 0.175, start_theta), rhoController(4.0, 0.225, start_rho) {}

void StagedProbe::initializeStages() {
    d_theta = (final_theta - start_theta) / divisions;
    d_rho = (final_rho - start_rho) / divisions;

    // Initialize CAN and GPIO
    CANInterface::setClosedLoopControl(0);
    CANInterface::setClosedLoopControl(1);
    CANInterface::setTorqueControlMode(0);
    CANInterface::setTorqueControlMode(1);

    gpio13.setup();
    gpio17.setup();
    gpio13.generatePulse();
    gpio17.generatePulse();

    // Initialize controllers with the current state
    float p0, v0, p1, v1;
    tie(p0, v0) = CANInterface::getEncoderEstimates(0);
    tie(p1, v1) = CANInterface::getEncoderEstimates(1);

    auto state = getStateVariables(p0, p1, v0, v1);
    float theta = get<4>(state);
    float rho = get<5>(state);

    thetaController.setpoint = theta;
    rhoController.setpoint = rho;
}

void StagedProbe::executeStages() {
    elapsed_start_time = steady_clock::now();
    data_log.clear();

    for (int i = 0; i <= divisions; ++i) {
        float target_theta = start_theta + i * d_theta;
        float target_rho = start_rho + i * d_rho;
        thetaController.setpoint = target_theta;
        rhoController.setpoint = target_rho;
        thetaController.reset();
        rhoController.reset();

        cout << "Stage " << i << "/" << divisions << ": θ=" << target_theta << ", ρ=" << target_rho << endl;

        // Calculate the end time for the current stage
        auto t_end = steady_clock::now() + chrono::duration<double>(pause_duration);

        while (steady_clock::now() < t_end) {
            // Get encoder estimates
            float p0, v0, p1, v1;
            tie(p0, v0) = CANInterface::getEncoderEstimates(0);
            tie(p1, v1) = CANInterface::getEncoderEstimates(1);

            // Compute state variables
            auto state = getStateVariables(p0, p1, v0, v1);
            float theta = get<4>(state);
            float rho = get<5>(state);

            // Update controllers
            float current_time = duration<float>(steady_clock::now().time_since_epoch()).count();
            float theta_torque = thetaController.update(theta, current_time);
            float rho_torque = rhoController.update(rho, current_time);

            // Compute motor torques and apply them
            auto motor_torques = getTorques(theta_torque, rho_torque);
            CANInterface::setTorque(0, get<0>(motor_torques));
            CANInterface::setTorque(1, get<1>(motor_torques));

            // Log data
            double elapsed_time = duration<double>(steady_clock::now() - elapsed_start_time).count();
            data_log.push_back({elapsed_time, p0, p1, get<0>(motor_torques), get<1>(motor_torques)});

            // Sleep for a short duration to maintain control loop frequency
            this_thread::sleep_for(milliseconds(1));
        }
    }
}

void StagedProbe::cleanup() {
    CANInterface::setIdle(0);
    CANInterface::setIdle(1);
    gpio13.cleanup();
    gpio17.cleanup();
}

void StagedProbe::saveData(const string& file_path) {
    saveDataToCSV(file_path, data_log);
}

