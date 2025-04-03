#include "utils.h"
#include <fstream>
#include <numeric>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <cmath>
#include <tuple>
#include <armadillo>
#include <vector>

void saveDataToCSV(const std::string& filename, const std::vector<std::vector<double>>& data) {
    std::ofstream file(filename);

    // Write column headers
    file << "Elapsed Time,Phi 1,Phi 2,Motor0 Velocity,Motor1 Velocity,Motor Torque 0,Motor Torque 1\n";

    // Write data
    for (const auto& row : data) {
        for (size_t i = 0; i < row.size(); ++i) {
            file << row[i];
            if (i < row.size() - 1) {
                file << ",";
            }
        }
        file << "\n";
    }

    file.close();
}

std::string getCurrentTimestamp() {
    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d %X");
    return ss.str();
}

std::tuple<float, float, float, float, float, float, float, float> getStateVariables(float encoder0_pos_estimate, float encoder1_pos_estimate, float encoder0_vel_estimate, float encoder1_vel_estimate) {
    float reference_point = 0.25;
    float phi_1 = (M_PI / 2) + ((reference_point - encoder0_pos_estimate) * (M_PI * 2));
    float phi_2 = ((3 * M_PI) / 2) + ((encoder1_pos_estimate - reference_point) * (M_PI * 2));

    float theta = 0.5 * (phi_1 + phi_2);
    float rho = 0.5 * (phi_1 - phi_2) + M_PI;

    float phi_1_vel = encoder0_vel_estimate;
    float phi_2_vel = -encoder1_vel_estimate;

    arma::mat22 Jacobian; 
    Jacobian << 0.5 << 0.5 << arma::endr
             << 0.5 << -0.5;
    arma::vec2 Polar_Velocity_Vector = {phi_1_vel, phi_2_vel};
    arma::vec2 wibblit_deriv = Jacobian * Polar_Velocity_Vector;
    float theta_vel = wibblit_deriv(0);
    float rho_vel = wibblit_deriv(1);

    return std::make_tuple(phi_1, phi_2, phi_1_vel, phi_2_vel, theta, rho, theta_vel, rho_vel);
}

std::tuple<float, float> getTorques(float theta_torque, float rho_torque) {
    arma::vec2 wibblit_torques = {theta_torque, rho_torque};
    arma::mat22 Jacobian;
    Jacobian << 0.5 << 0.5 << arma::endr
             << 0.5 << -0.5;
    arma::vec2 motor_torque_cmd = Jacobian.t() * wibblit_torques;

    float Motor0_Torque = -motor_torque_cmd(0);
    float Motor1_Torque = motor_torque_cmd(1);

    return std::make_tuple(Motor0_Torque, Motor1_Torque);
}