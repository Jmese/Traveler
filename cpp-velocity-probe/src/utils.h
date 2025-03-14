#ifndef UTILS_H
#define UTILS_H

#include <vector>
#include <string>
#include <tuple>

// Function to save data to a CSV file
void saveDataToCSV(const std::string& filename, const std::vector<std::vector<double>>& data);

// Function to calculate the average of a vector of doubles
double calculateAverage(const std::vector<double>& values);

// Function to get the current timestamp as a string
std::string getCurrentTimestamp();

// Function to get state variables
std::tuple<float, float, float, float, float, float, float, float> getStateVariables(float encoder0_pos_estimate, float encoder1_pos_estimate, float encoder0_vel_estimate, float encoder1_vel_estimate);

// Function to get torques
std::tuple<float, float> getTorques(float theta_torque, float rho_torque);

#endif // UTILS_H