cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME GongChuangsai)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    board/board.c
    board/board_motor.c
    board/board_ops.c
    board/board_crane.c
    board/board_servo.c
    board/board_crane_action.c
    modules/motor/x/x_motor.c
    modules/ops/ops.c
    modules/servo/servo.c
    application/chassis/omni_kinematics.c
    application/chassis/chassis_control.c
    application/robot.c
    application/crane_action.c
    bsp/led/bsp_led.c
    bsp/pwm/bsp_pwm.c
    bsp/uart/bsp_uart.c
    bsp/usb/bsp_usb.c
    modules/host_link/host_protocol.c
    modules/host_link/vofa_protocol.c
    modules/motor/emm/emm_bus.c
    modules/motor/emm/emm_motor.c
    modules/task_monitor/task_monitor.c
    system/robot_task.c
    system/robot_debug_position.c
    system/host_service.c
    application/chassis/chassis_odometry.c
    application/chassis/chassis_localization.c
    application/chassis/chassis_position_control.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    board
    application/chassis
    application
    bsp/led
    bsp/pwm
    bsp/uart
    bsp/usb
    modules/host_link
    modules/ops
    modules/servo
    modules/motor/emm
    modules/motor/x
    modules/task_monitor
    system
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Only Debug builds may contain the optional debugger motion mailbox.
    $<$<CONFIG:Debug>:ROBOT_DEBUG_BUILD=1>
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
