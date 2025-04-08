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
    pinMode(pin, OUTPUT); // Set pin to OUTPUT mode
    digitalWrite(pin, LOW); // Ensure pin starts LOW
}

void GPIOInterface::generatePulse() {
    pinMode(pin, OUTPUT);       // Ensure pin is in OUTPUT mode
    digitalWrite(pin, HIGH);    // Set pin HIGH
    delay(10);                  // Keep HIGH for 10 ms
    digitalWrite(pin, LOW);     // Set pin LOW
}

void GPIOInterface::cleanup() {
    digitalWrite(pin, LOW);     // Set pin LOW before cleanup
    pinMode(pin, INPUT);        // Set pin to INPUT mode
}