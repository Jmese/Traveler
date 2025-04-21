#ifndef GPIO_INTERFACE_H
#define GPIO_INTERFACE_H

#include <wiringPi.h>
#include <softPwm.h>
#include <iostream>

class GPIOInterface {
public:
    GPIOInterface(int pin, float frequency);
    ~GPIOInterface();
    
    void generatePulse();
    void setup();
    void cleanup();

private:
    int pin;
    float frequency;
};

#endif // GPIO_INTERFACE_H