#cmake_minimum_required(VERSION 3.31)
#project(SatTracker)
#
#set(CMAKE_CXX_STANDARD 17)
#
#add_executable(SatTracker main.cpp)

cmake_minimum_required(VERSION 3.16)
project(SatelliteTracker VERSION 1.0.0 LANGUAGES CXX)

# Set C++ standard and compiler flags for precision tracking
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Optimization flags for real-time performance
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -march=armv8-a -mtune=cortex-a72 -DNDEBUG")
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0 -Wall -Wextra -DDEBUG")

# Professional satellite tracking system requires precise timing
add_compile_definitions(PRECISION_TRACKING_ENABLED)
add_compile_definitions(SATELLITE_TRACKER_VERSION="1.0.0")

# Include directories for modular architecture
include_directories(
        ${PROJECT_SOURCE_DIR}/include
        ${PROJECT_SOURCE_DIR}/include/sensors
        ${PROJECT_SOURCE_DIR}/include/control
        ${PROJECT_SOURCE_DIR}/include/tracking
        ${PROJECT_SOURCE_DIR}/include/math
)

# Source file organization for professional development
file(GLOB_RECURSE SENSOR_SOURCES "src/sensors/*.cpp")
file(GLOB_RECURSE CONTROL_SOURCES "src/control/*.cpp")
file(GLOB_RECURSE TRACKING_SOURCES "src/tracking/*.cpp")
file(GLOB_RECURSE MATH_SOURCES "src/math/*.cpp")
file(GLOB MAIN_SOURCES "src/*.cpp")

set(ALL_SOURCES
        ${MAIN_SOURCES}
        ${SENSOR_SOURCES}
        ${CONTROL_SOURCES}
        ${TRACKING_SOURCES}
        ${MATH_SOURCES}
        include/SystemCore.h
        include/control/SensorManager.h
        include/control/LidarManager.h
        include/math/MotorController.h
        include/math/PIDController.h
        include/sensors/TrackingController.h
        include/sensors/TargetPredictor.h
        include/tracking/KalmanFilter.h
        include/tracking/MathUtils.h
        src/sensors/SensorManager.cpp
        src/sensors/LidarInterface.cpp
        src/tracking/TrackingController.cpp
        src/tracking/TargetPredictor.cpp
        src/control/MotorController.cpp
        src/control/PIDController.cpp
        src/math/KalmanFilter.cpp
        src/math/MathUtils.cpp
)

# Create the professional satellite tracker executable
add_executable(satellite_tracker ${ALL_SOURCES})

# Link essential libraries for satellite tracking operations
target_link_libraries(satellite_tracker
        wiringPi          # GPIO and hardware control
        pthread           # Multi-threading for real-time operations
        rt                # Real-time extensions
        m                 # Math library for trajectory calculations
)

# Enable all warnings for professional code quality
target_compile_options(satellite_tracker PRIVATE
        -Wall -Wextra -Wpedantic -Werror
        -Wno-unused-parameter    # Allow unused parameters in interfaces
)

# Custom targets for deployment and debugging
add_custom_target(deploy
        COMMAND scp satellite_tracker tracker@raspberrypi.local:~/satellite_tracker/
        DEPENDS satellite_tracker
        COMMENT "Deploying satellite tracker to Raspberry Pi"
)

add_custom_target(remote_debug
        COMMAND ssh tracker@raspberrypi.local "cd ~/satellite_tracker && gdbserver :2000 ./satellite_tracker"
        DEPENDS deploy
        COMMENT "Starting remote debugging session"
)

# Professional documentation generation
find_package(Doxygen)
if(DOXYGEN_FOUND)
    add_custom_target(docs
            COMMAND ${DOXYGEN_EXECUTABLE} Doxyfile
            WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
            COMMENT "Generating satellite tracker documentation"
    )
endif()