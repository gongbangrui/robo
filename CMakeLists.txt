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
set(CMAKE_PROJECT_NAME smart_car)

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
    # application
    ${CMAKE_CURRENT_SOURCE_DIR}/application/calibrate_task.c
    ${CMAKE_CURRENT_SOURCE_DIR}/application/CAN_receive.c
    ${CMAKE_CURRENT_SOURCE_DIR}/application/chassis_behaviour.c
    ${CMAKE_CURRENT_SOURCE_DIR}/application/chassis_task.c
    ${CMAKE_CURRENT_SOURCE_DIR}/application/detect_task.c
    ${CMAKE_CURRENT_SOURCE_DIR}/application/INS_task.c
    ${CMAKE_CURRENT_SOURCE_DIR}/application/led_flow_task.c
    ${CMAKE_CURRENT_SOURCE_DIR}/application/line_track_task.c
    ${CMAKE_CURRENT_SOURCE_DIR}/application/navigate_task.c
    ${CMAKE_CURRENT_SOURCE_DIR}/application/usb_cdc_task.c
    ${CMAKE_CURRENT_SOURCE_DIR}/application/uart_debug.c
    ${CMAKE_CURRENT_SOURCE_DIR}/application/voltage_task.c
    ${CMAKE_CURRENT_SOURCE_DIR}/application/actions.c
    ${CMAKE_CURRENT_SOURCE_DIR}/application/debug_console.c
    ${CMAKE_CURRENT_SOURCE_DIR}/application/cross_exec.c
    ${CMAKE_CURRENT_SOURCE_DIR}/application/turn_exec.c
    ${CMAKE_CURRENT_SOURCE_DIR}/application/forward_exec.c
    ${CMAKE_CURRENT_SOURCE_DIR}/application/platform_exec.c
    ${CMAKE_CURRENT_SOURCE_DIR}/application/bridge_exec.c
    ${CMAKE_CURRENT_SOURCE_DIR}/application/servo_ctrl.c
    ${CMAKE_CURRENT_SOURCE_DIR}/application/lt_utils.c
    ${CMAKE_CURRENT_SOURCE_DIR}/application/nav_types.c
    ${CMAKE_CURRENT_SOURCE_DIR}/application/paths.c
    ${CMAKE_CURRENT_SOURCE_DIR}/application/vision_exec.c
    ${CMAKE_CURRENT_SOURCE_DIR}/application/move_exec.c
    ${CMAKE_CURRENT_SOURCE_DIR}/application/climb_exec.c
    ${CMAKE_CURRENT_SOURCE_DIR}/application/ir_detect_exec.c
    ${CMAKE_CURRENT_SOURCE_DIR}/application/run_task.c
    # bsp/boards
    ${CMAKE_CURRENT_SOURCE_DIR}/bsp/boards/bsp_adc.c
    ${CMAKE_CURRENT_SOURCE_DIR}/bsp/boards/bsp_buzzer.c
    ${CMAKE_CURRENT_SOURCE_DIR}/bsp/boards/bsp_can.c
    ${CMAKE_CURRENT_SOURCE_DIR}/bsp/boards/bsp_delay.c
    ${CMAKE_CURRENT_SOURCE_DIR}/bsp/boards/bsp_imu_pwm.c
    ${CMAKE_CURRENT_SOURCE_DIR}/bsp/boards/bsp_flash.c
    ${CMAKE_CURRENT_SOURCE_DIR}/bsp/boards/bsp_ir_sensor.c
    ${CMAKE_CURRENT_SOURCE_DIR}/bsp/boards/bsp_led.c
    ${CMAKE_CURRENT_SOURCE_DIR}/bsp/boards/bsp_rng.c
    ${CMAKE_CURRENT_SOURCE_DIR}/bsp/boards/bsp_servo_pwm.c
    ${CMAKE_CURRENT_SOURCE_DIR}/bsp/boards/bsp_spi.c
    ${CMAKE_CURRENT_SOURCE_DIR}/bsp/boards/bsp_usart.c
    ${CMAKE_CURRENT_SOURCE_DIR}/bsp/boards/bsp_voice.c
    ${CMAKE_CURRENT_SOURCE_DIR}/bsp/boards/bsp_vision.c
    # components/algorithm
    ${CMAKE_CURRENT_SOURCE_DIR}/components/algorithm/user_lib.c
    ${CMAKE_CURRENT_SOURCE_DIR}/components/algorithm/AHRS_middleware.c
    ${CMAKE_CURRENT_SOURCE_DIR}/components/algorithm/AHRS.c
    ${CMAKE_CURRENT_SOURCE_DIR}/components/algorithm/arm_math_wrappers.c
    # components/controller
    ${CMAKE_CURRENT_SOURCE_DIR}/components/controller/pid.c
    # components/devices
    ${CMAKE_CURRENT_SOURCE_DIR}/components/devices/BMI088driver.c
    ${CMAKE_CURRENT_SOURCE_DIR}/components/devices/BMI088Middleware.c
    ${CMAKE_CURRENT_SOURCE_DIR}/components/devices/gray_sensor.c
    ${CMAKE_CURRENT_SOURCE_DIR}/components/devices/ir_sensor.c
    ${CMAKE_CURRENT_SOURCE_DIR}/components/devices/ist8310driver.c
    ${CMAKE_CURRENT_SOURCE_DIR}/components/devices/ist8310driver_middleware.c
    ${CMAKE_CURRENT_SOURCE_DIR}/components/devices/voice_module.c
    ${CMAKE_CURRENT_SOURCE_DIR}/components/devices/vision_module.c
    # components/support
    ${CMAKE_CURRENT_SOURCE_DIR}/components/support/CRC8_CRC16.c
    ${CMAKE_CURRENT_SOURCE_DIR}/components/support/fifo.c
    ${CMAKE_CURRENT_SOURCE_DIR}/components/support/mem_mang4.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/application
    ${CMAKE_CURRENT_SOURCE_DIR}/bsp/boards
    ${CMAKE_CURRENT_SOURCE_DIR}/components/algorithm
    ${CMAKE_CURRENT_SOURCE_DIR}/components/algorithm/Include
    ${CMAKE_CURRENT_SOURCE_DIR}/components/controller
    ${CMAKE_CURRENT_SOURCE_DIR}/components/devices
    ${CMAKE_CURRENT_SOURCE_DIR}/components/support
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    ARM_MATH_CM4
    __FPU_PRESENT=1U
    __TARGET_FPU_VFP
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
