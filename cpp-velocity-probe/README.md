# C++ Velocity Probe

## Overview
The C++ Velocity Probe project is designed to control motors using a Proportional-Derivative (PD) control strategy. It communicates with motors via the CAN bus and manages GPIO for PWM output. The project replicates the functionality of a Python-based velocity probe system.

## Project Structure
```
cpp-velocity-probe
├── src
│   ├── main.cpp               # Entry point of the application
│   ├── PDController.cpp       # Implementation of the PDController class
│   ├── PDController.h         # Header file for the PDController class
│   ├── CANInterface.cpp        # Implementation of CAN communication functions
│   ├── CANInterface.h          # Header file for CAN communication functions
│   ├── GPIOInterface.cpp       # Implementation of GPIO control functions
│   ├── GPIOInterface.h         # Header file for GPIO control functions
│   └── utils.cpp              # Implementation of utility functions
│       └── utils.h            # Header file for utility functions
├── CMakeLists.txt             # CMake configuration file
└── README.md                   # Project documentation
```

## Dependencies
- CMake
- CAN library (e.g., SocketCAN for Linux)
- WiringPi or similar library for GPIO control on Raspberry Pi

## Setup Instructions
1. **Clone the repository:**
   ```
   git clone <repository-url>
   cd cpp-velocity-probe
   ```

2. **Install dependencies:**
   Ensure that you have the necessary libraries installed for CAN and GPIO control.

3. **Build the project:**
   ```
   mkdir build
   cd build
   cmake ..
   make
   ```

4. **Run the application:**
   After building, you can run the application using:
   ```
   ./cpp-velocity-probe
   ```

## Usage
The application initializes the CAN bus and sets up GPIO for PWM. It enters a control loop where it reads encoder values, updates the PD controllers, and sends commands to the motors based on the control signals.

## Contributing
Contributions are welcome! Please submit a pull request or open an issue for any enhancements or bug fixes.

## License
This project is licensed under the MIT License. See the LICENSE file for more details.