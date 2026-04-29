# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0-a4)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico_w CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(ProjectRoboticz C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(ProjectRoboticz
    ProjectRoboticz.cpp
    Encoder.cpp
    IMU.cpp
    SensorHub.cpp
    Drive.cpp
    Robot.cpp
    PIDController.cpp
    DriveCommand.cpp
    Motor.cpp
    Localisation.cpp     
)

pico_set_program_name(ProjectRoboticz "ProjectRoboticz")
pico_set_program_version(ProjectRoboticz "0.1")

# voor uitlezen op mijn laptop
pico_enable_stdio_uart(ProjectRoboticz 1)
pico_enable_stdio_usb(ProjectRoboticz 0)


# voor samen met RPI5
#pico_enable_stdio_uart(ProjectRoboticz 0)
#pico_enable_stdio_usb(ProjectRoboticz 1)

# Add the standard library to the build
target_link_libraries(ProjectRoboticz
        pico_stdlib
        hardware_pwm
        hardware_gpio
        hardware_i2c
        hardware_irq 
    )

# Add the standard include files to the build
target_include_directories(ProjectRoboticz PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
)

pico_add_extra_outputs(ProjectRoboticz)

