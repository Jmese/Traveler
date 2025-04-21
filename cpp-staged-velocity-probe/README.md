# cpp-staged-velocity-probe

This project implements a velocity probe system that utilizes CAN bus communication and GPIO control to manage the operation of motors. The system allows for incremental adjustments to the desired rho and theta values through a staging process, enhancing the control and precision of the motors.

## Project Structure

The project is organized into the following directories and files:

```
cpp-staged-velocity-probe
├── CMakeLists.txt          # CMake configuration file for building the project
├── README.md               # Documentation for the project
└── src                     # Source files for the project
    ├── main.cpp            # Entry point of the application
    ├── StagedProbe.h       # Header file for the StagedProbe class
    ├── StagedProbe.cpp     # Implementation of the StagedProbe class
    ├── PDController.h      # Header file for the PDController class
    ├── PDController.cpp     # Implementation of the PDController class
    ├── CANInterface.h       # Header file for the CANInterface class
    ├── CANInterface.cpp     # Implementation of the CANInterface class
    ├── GPIOInterface.h      # Header file for the GPIOInterface class
    ├── GPIOInterface.cpp    # Implementation of the GPIOInterface class
    ├── utils.h             # Header file for utility functions
    └── utils.cpp           # Implementation of utility functions
```

## Setup Instructions

1. **Clone the Repository**: 
   Clone this repository to your local machine using:
   ```
   git clone <repository-url>
   ```

2. **Install Dependencies**: 
   Ensure that you have CMake and the necessary libraries for CAN and GPIO communication installed on your system.

3. **Build the Project**: 
   Navigate to the project directory and create a build directory:
   ```
   cd cpp-staged-velocity-probe
   mkdir build
   cd build
   cmake ..
   make
   ```

4. **Run the Application**: 
   After building the project, you can run the application using:
   ```
   ./cpp-staged-velocity-probe
   ```

## Usage

Upon running the application, you will be prompted to enter the desired rho velocity. The system will initialize the CAN bus and GPIO pins, and then enter a control loop where it manages the motor operations. The staging functionality allows for gradual adjustments to the desired rho and theta values, providing a smooth control experience.

## Functionality

- **CAN Bus Communication**: The system communicates with motors over the CAN bus, allowing for real-time control and monitoring.
- **GPIO Control**: GPIO pins are utilized to generate voltage pulses for motor control.
- **Staging Process**: The application supports a staging process that enables incremental adjustments to motor setpoints, enhancing precision and control.
- **Data Logging**: The system logs data during operation, which can be saved for analysis after the run.

## Contributing

Contributions to this project are welcome. Please submit a pull request or open an issue for any enhancements or bug fixes.

## License

This project is licensed under the MIT License. See the LICENSE file for more details.