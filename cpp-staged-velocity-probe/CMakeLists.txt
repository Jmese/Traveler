cmake_minimum_required(VERSION 3.10)

project(VelocityProbe)

set(CMAKE_CXX_STANDARD 14)

include_directories(src)
include_directories(/usr/include/eigen3)

file(GLOB SOURCES
    src/*.cpp
)

add_executable(VelocityProbe ${SOURCES})

# Link the WiringPi library
target_link_libraries(VelocityProbe wiringPi)