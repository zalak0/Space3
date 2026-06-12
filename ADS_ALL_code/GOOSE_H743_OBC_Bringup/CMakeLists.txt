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
set(CMAKE_PROJECT_NAME GOOSE_H743_OBC_Bringup)

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

# ---------------------------------------------------------------------------
# GOOSE ADS bring-up configuration
#
# Current offline/H743 bring-up mode:
#   ADS_TARGET=ADS_TARGET_H743_OBC
#   ADS_H743_SENSOR_BACKEND_READY=0
#   ADS_ALLOW_FAKE_BACKEND_ON_H743=1
#
# This runs the H743 firmware with the fake ADS sensor backend.
#
# Do not enable Core/Src/ads_sensor_backend_h743.c or
# Core/Src/ads_h743_sensors.c until the real H743 sensor drivers are ready.
#
# Photodiode inputs are intentionally excluded from ADS.
# ---------------------------------------------------------------------------


# ---------------------------------------------------------------------------
# GOOSE ADS backend selection
#
# 0 = H743 MCU bring-up using fake ADS backend
# 1 = H743 real sensor backend skeleton / future real sensors
#
# Keep this at 0 until real IMU/magnetometer drivers are implemented and tested.
# Photodiode inputs are intentionally not part of ADS.
# ---------------------------------------------------------------------------
set(GOOSE_ADS_H743_REAL_BACKEND 0)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    Core/Src/ads_app.c
    Core/Src/ads_task.c
    Core/Src/ads_core.c
    Core/Src/ads_sensor_interface.c
    Core/Src/ads_sensor_validate.c
    Core/Src/ads_runtime_check.c
    Core/Src/ads_config_check.c
    Core/Src/ads_timing.c
    Core/Src/ads_status.c
    Core/Src/ads_output.c
    Core/Src/ads_build_info.c
    Core/Src/ads_fake_sensors.c
    Core/Src/ads_ekf.c
    Core/Src/adcs.c
    Core/Src/vector3.c
    Core/Src/quaternion.c
    Core/Src/calibration.c
    Core/Src/ads_debug.c
    Core/Src/quest.c
    Core/Src/adcs_health.c
    Core/Src/ads_fault_injection.c
    Core/Src/matrix_utils.c
    Core/Src/ads_icm20948_probe.c
    Core/Src/ads_h743_sensors.c
)

if(GOOSE_ADS_H743_REAL_BACKEND EQUAL 1)

    target_sources(${CMAKE_PROJECT_NAME} PRIVATE
        Core/Src/ads_sensor_backend_h743.c
        Core/Src/ads_h743_sensors.c
    )

    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
        ADS_TARGET=ADS_TARGET_H743_OBC
        ADS_H743_SENSOR_BACKEND_READY=1
        ADS_ALLOW_FAKE_BACKEND_ON_H743=0
    )

else()

    target_sources(${CMAKE_PROJECT_NAME} PRIVATE
        Core/Src/ads_sensor_backend_fake.c
    )

    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
        ADS_TARGET=ADS_TARGET_H743_OBC
        ADS_H743_SENSOR_BACKEND_READY=0
        ADS_ALLOW_FAKE_BACKEND_ON_H743=1
    )

endif()

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    Core/Inc
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
