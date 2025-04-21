#ifndef PDCONTROLLER_H
#define PDCONTROLLER_H

class PDController {
public:
    PDController(double Kp, double Kd, double setpoint);
    double update(double measured_val, double current_time);
    void reset();

    double setpoint;
    
private:
    double Kp;
    double Kd;
    double prev_error;
    double prev_time;
};

#endif // PDCONTROLLER_H