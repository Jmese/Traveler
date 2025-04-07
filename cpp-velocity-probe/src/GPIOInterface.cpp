#include "GPIOInterface.h"

GPIOInterface::GPIOInterface(int pin, float frequency)
    : pin(pin), frequency(frequency) {}

GPIOInterface::~GPIOInterface() {
    cleanup();
}

void GPIOInterface::setup() {
    if (wiringPiSetupGpio() == -1) {
        std::cerr << "Error setting up WiringPi" << std::endl;
        exit(EXIT_FAILURE);
    }
    pinMode(pin, PWM_OUTPUT);
    pwmSetMode(PWM_MODE_MS);
    pwmSetClock(192); // 19.2 MHz / 192 = 100 kHz
    pwmSetRange(1000); // 100 kHz / 1000 = 100 Hz
    pwmWrite(pin, 500); // 50% duty cycle
}

void GPIOInterface::generatePulse() {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, HIGH); // Set pin HIGH
    delay(10);               // Keep HIGH for 10 ms
    digitalWrite(pin, LOW);  // Set pin LOW
}

void GPIOInterface::cleanup() {
    pwmWrite(pin, 0);
    pinMode(pin, INPUT);
}