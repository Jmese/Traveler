#include "PDController.h"

PDController::PDController(double Kp, double Kd, double setpoint)
    : Kp(Kp), Kd(Kd), setpoint(setpoint), prev_error(0.0), prev_time(0.0) {}

double PDController::update(double measured_val, double current_time) {
    double error = setpoint - measured_val;
    double dt = (prev_time == 0.0) ? 0.0 : current_time - prev_time;

    double derivative = (dt > 0) ? (error - prev_error) / dt : 0.0;
    double control_signal = Kp * error + Kd * derivative;

    prev_error = error;
    prev_time = current_time;

    return control_signal;
}

void PDController::reset() {
    prev_error = 0.0;
    prev_time = 0.0;
}