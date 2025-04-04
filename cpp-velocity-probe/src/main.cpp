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

using namespace std;
using namespace std::chrono;

CANInterface canInterface("can0");

volatile sig_atomic_t stop = 0;

void handle_sigint(int sig) {
    stop = 1;
}

int main() {
    // Register signal handler
    signal(SIGINT, handle_sigint);

    // Initialize CAN bus
    canInterface.flush();

    // Setup GPIO for PWM
    GPIOInterface gpioInterface(13, 1.0); // Pin 13, 1Hz frequency
    gpioInterface.setup();

    // Define nodes
    int nodes[] = {0, 1};
    CANInterface::setClosedLoopControl(nodes[0]);
    CANInterface::setClosedLoopControl(nodes[1]);
    CANInterface::setTorqueControlMode(nodes[0]);
    CANInterface::setTorqueControlMode(nodes[1]);

    // Set initial positions
    float initial_rho = 0.7;
    float initial_theta = 3.14;

    // Initialize PD Controllers
    PDController thetaController(0.3, 0.01, initial_theta);
    PDController rhoController(0.4 , 0.02, initial_rho);

    // Set initial rho and theta positions for 2 seconds
    auto start_time = steady_clock::now();
    while (steady_clock::now() - start_time < seconds(2)) {
        float current_position_0, current_velocity_0;
        float current_position_1, current_velocity_1;

        tie(current_position_0, current_velocity_0) = CANInterface::getEncoderEstimates(nodes[0]);
        tie(current_position_1, current_velocity_1) = CANInterface::getEncoderEstimates(nodes[1]);

        auto state_vars = getStateVariables(current_position_0, current_position_1, current_velocity_0, current_velocity_1);
        float theta = get<4>(state_vars);
        float rho = get<5>(state_vars);
        float phi1 = get<1>(state_vars);
        float phi2 = get<2>(state_vars);

        float theta_torque = thetaController.update(theta, duration<float>(steady_clock::now().time_since_epoch()).count());
        float rho_torque = rhoController.update(rho, duration<float>(steady_clock::now().time_since_epoch()).count());

        auto motor_torques = getTorques(theta_torque, rho_torque);
        CANInterface::setTorque(nodes[0], get<0>(motor_torques));
        CANInterface::setTorque(nodes[1], get<1>(motor_torques));

        this_thread::sleep_for(milliseconds(50)); // Increased control loop delay to 50 ms
    }

    // Final desired targets (in radians)
    float final_target_rho = 3.0;
    float final_target_theta = 3.14;

    // Define desired maximum velocities (setpoint update rates) in rad/s
    float desired_theta_velocity = 500.0;  // Adjust as needed
    float desired_rho_velocity = 0.1;    // Adjust as needed

    // Initialize PD controllers with the current (initial) setpoints.
    thetaController = PDController(20, 0.25, initial_theta);
    rhoController = PDController(30, 0.2, initial_rho);

    // Main control loop
    vector<vector<double>> data_log;
    auto elapsed_start_time = steady_clock::now();
    auto last_setpoint_update_time = steady_clock::now();
    try {
        while (!stop) {
            auto current_time = steady_clock::now();
            auto elapsed_time = duration<double>(current_time - elapsed_start_time).count();

            // Get encoder estimates
            float current_position_0, current_velocity_0;
            float current_position_1, current_velocity_1;
            tie(current_position_0, current_velocity_0) = CANInterface::getEncoderEstimates(nodes[0]);
            tie(current_position_1, current_velocity_1) = CANInterface::getEncoderEstimates(nodes[1]);

            // Get torque estimates
            float torque_target_0, torque_estimate_0;
            float torque_target_1, torque_estimate_1;
            std::tie(torque_target_0, torque_estimate_0) = CANInterface::getTorqueEstimates(nodes[0]);
            std::tie(torque_target_1, torque_estimate_1) = CANInterface::getTorqueEstimates(nodes[1]);

            // Log data
            data_log.push_back({
                elapsed_time,
                current_position_0,
                current_position_1,
                current_velocity_0,
                current_velocity_1,
                torque_estimate_0,
                torque_estimate_1,
            });

            // Control logic
            auto state_vars = getStateVariables(current_position_0, current_position_1, current_velocity_0, current_velocity_1);
            float theta = get<4>(state_vars);
            float rho = get<5>(state_vars);

            float theta_torque = thetaController.update(theta, duration<float>(current_time.time_since_epoch()).count());
            float rho_torque = rhoController.update(rho, duration<float>(current_time.time_since_epoch()).count());

            auto motor_torques = getTorques(theta_torque, rho_torque);
            CANInterface::setTorque(nodes[0], get<0>(motor_torques));
            CANInterface::setTorque(nodes[1], get<1>(motor_torques));
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

    gpioInterface.cleanup();

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
        ss << put_time(localtime(&now_c), "%H.%M_%m.%d");
        string filename = test_string + "[" + ss.str() + "]_rrho_0.50.csv";
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