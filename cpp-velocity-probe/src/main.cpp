#include <iostream>
#include <chrono>
#include <thread>
#include <csignal>
#include <vector>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <numeric>
#include "PDController.h"
#include "CANInterface.h"
#include "GPIOInterface.h"
#include "utils.h"
#include <wiringPi.h> // Include WiringPi header if not already included
#include <algorithm>  // Include for std::clamp

// Define a custom clamp function if std::clamp is unavailable
template <typename T>
T clamp(const T& value, const T& low, const T& high) {
    return std::max(low, std::min(value, high));
}

using namespace std;
using namespace std::chrono;

CANInterface canInterface("can0");

volatile sig_atomic_t stop = 0;

void handle_sigint(int sig) {
    stop = 1;
}

int main() {

    // Prompt the user for rho velocity input before anything else
    float desired_rho_velocity;
    cout << "Enter the desired rho velocity (rad/s): ";
    cin >> desired_rho_velocity;

    // Prompt the user for final rho setpoint
    float user_final_rho;
    cout << "Enter the final rho setpoint (rad): ";
    cin >> user_final_rho;

    // Register signal handler
    signal(SIGINT, handle_sigint);

    // Initialize CAN bus
    canInterface.flush();

    // Generate voltage pulses on GPIO13 and GPIO17
    GPIOInterface gpio13(13, 1.0); // Pin 13
    GPIOInterface gpio17(17, 1.0); // Pin 17
    // Setup GPIO pins
    gpio13.setup();
    gpio17.setup();

    // Send initial pulse before the main control loop
    gpio13.generatePulse();
    gpio17.generatePulse();

    // Define nodes
    int nodes[] = {0, 1};
    CANInterface::setClosedLoopControl(nodes[0]);
    CANInterface::setClosedLoopControl(nodes[1]);
    CANInterface::setTorqueControlMode(nodes[0]);
    CANInterface::setTorqueControlMode(nodes[1]);

    // Set initial positions
    float initial_rho = 0.6;
    float initial_theta = 3.14;

    // Initialize PD Controllers
    PDController thetaController(4.5, 0.25, initial_theta);
    PDController rhoController(8.25, 0.215, initial_rho);

    // Final desired targets (in radians) and velocities
    float final_target_rho = user_final_rho;
    float final_target_theta = 3.14f;
    float desired_theta_velocity = 100.0f;

    // Initialize PD controllers
    thetaController = PDController(30, 0.235, initial_theta);
    rhoController = PDController(50, 0.235, initial_rho);

    // Declare data_log at the beginning of the main function
    vector<vector<double>> data_log;

    try {
        auto data_collection_start_time = steady_clock::now();
        auto elapsed_start_time = steady_clock::now();
        bool hold_position_phase = true; // Flag to indicate the initial hold position phase
        bool motors_started = false;
        auto last_setpoint_update_time = data_collection_start_time;

        while (!stop) {
            auto current_time = steady_clock::now();

            // 1) Read sensors
            float p0, v0, p1, v1;
            tie(p0, v0) = CANInterface::getEncoderEstimates(nodes[0]);
            tie(p1, v1) = CANInterface::getEncoderEstimates(nodes[1]);
            auto s = getStateVariables(p0, p1, v0, v1);
            float theta = get<4>(s), rho = get<5>(s);

            // 2) Compute how long we've been logging
            float elapsed_data = duration<float>(current_time - data_collection_start_time).count();

            float theta_torque = 0.0f, rho_torque = 0.0f;

            if (hold_position_phase) {
                // Hold initial position for 2 seconds
                theta_torque = thetaController.update(theta, duration<float>(current_time.time_since_epoch()).count());
                rho_torque = rhoController.update(rho, duration<float>(current_time.time_since_epoch()).count());

                if (elapsed_data >= 2.0f) {
                    hold_position_phase = false; // Transition to the main control loop
                    motors_started = true;
                    last_setpoint_update_time = current_time; // Reset setpoint update time
                }
            } else {
                // Main control loop
                float dt_sp = duration<float>(current_time - last_setpoint_update_time).count();
                if (dt_sp > 0) {
                    // 1) Update theta setpoint, but don’t overshoot final_target_theta
                    {
                        float new_theta_sp = thetaController.setpoint
                                               + desired_theta_velocity * dt_sp;
                        if (desired_theta_velocity >= 0.0f) {
                            if (new_theta_sp > final_target_theta)
                                new_theta_sp = final_target_theta;
                        } else {
                            if (new_theta_sp < final_target_theta)
                                new_theta_sp = final_target_theta;
                        }
                        thetaController.setpoint = new_theta_sp;
                    }

                    // 2) Update rho setpoint, but don’t overshoot final_target_rho
                    {
                        float new_rho_sp = rhoController.setpoint
                                             + desired_rho_velocity * dt_sp;
                        if (desired_rho_velocity >= 0.0f) {
                            if (new_rho_sp > final_target_rho)
                                new_rho_sp = final_target_rho;
                        } else {
                            if (new_rho_sp < final_target_rho)
                                new_rho_sp = final_target_rho;
                        }
                        rhoController.setpoint = new_rho_sp;
                    }

                    last_setpoint_update_time = current_time;
                }

                theta_torque = thetaController.update(theta, duration<float>(current_time.time_since_epoch()).count());
                rho_torque = rhoController.update(rho, duration<float>(current_time.time_since_epoch()).count());
            }

            // 3) Send torques
            auto motor_torques = getTorques(theta_torque, rho_torque);
            CANInterface::setTorque(nodes[0], get<0>(motor_torques));
            CANInterface::setTorque(nodes[1], get<1>(motor_torques));

            // 4) Log your data
            auto elapsed_time = duration<double>(current_time - elapsed_start_time).count();
            float torque_target_0, torque_estimate_0;
            float torque_target_1, torque_estimate_1;

            std::tie(torque_target_0, torque_estimate_0) = CANInterface::getTorqueEstimates(nodes[0]);
            std::tie(torque_target_1, torque_estimate_1) = CANInterface::getTorqueEstimates(nodes[1]);

            data_log.push_back({
                elapsed_time, 
                p0, 
                p1, 
                v0,
                v1,
                torque_estimate_0, 
                torque_estimate_1, 
            });
        }
    } catch (const exception& e) {
        cerr << "Error: " << e.what() << endl;
    }

    // Cleanup on exit
    cerr << "Keyboard interrupt detected. Setting nodes to idle..." << endl;
    for (int node_id : nodes) {
        cout << "Setting node " << node_id << " to idle..." << endl;
        CANInterface::setIdle(node_id);
    }
    cerr << "Nodes set to idle. Exiting program." << endl;

    // Send pulse upon keyboard interrupt
    gpio13.generatePulse();
    gpio17.generatePulse();

    gpio13.cleanup();
    gpio17.cleanup(); 

    // Calculate the average logging frequency
    if (data_log.size() > 1) {
        vector<double> time_diffs;
        for (size_t i = 1; i < data_log.size(); ++i) {
            time_diffs.push_back(data_log[i][0] - data_log[i - 1][0]);
        }
        double avg_time_diff = accumulate(time_diffs.begin(), time_diffs.end(), 0.0) / time_diffs.size();
        double avg_frequency = 1.0 / avg_time_diff;
        cout << "Average data logging frequency: " << avg_frequency << " Hz" << endl;
    } else {
        cout << "Not enough data points to calculate logging frequency." << endl;
    }

    // Ask if the user wants to save the data
    cout << "Do you want to save the data? (y/n): ";
    char save_data;
    cin >> save_data;
    if (save_data == 'y' || save_data == 'Y') {
        cout << "Enter the TEST string: ";
        string test_string;
        cin >> test_string;

        auto now = system_clock::now();
        time_t now_c = system_clock::to_time_t(now);
        stringstream ss;
        ss << put_time(localtime(&now_c), "%m%d_%H%M");
        stringstream velocity_ss;
        velocity_ss << fixed << setprecision(1) << desired_rho_velocity;
        string filename = test_string + "[" + ss.str() + "]" + velocity_ss.str() + ".csv";
        string file_path = "/home/traveler/Traveler_Hopper_sw-bundle/Data/DPROBE";
        string full_path = file_path + "/" + filename;

        cout << "Saving Data to: " + full_path << endl;
        saveDataToCSV(full_path, data_log);
        cout << "Save Complete" << endl;
    } else {
        cout << "Data not saved." << endl;
    }

    return 0;
}