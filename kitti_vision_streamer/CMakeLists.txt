# KITTI vision benchmark dataset streamer

# Project
cmake_minimum_required(VERSION 3.10)
project(kitti_vision_streamer)
set(CMAKE_CXX_STANDARD 14)

# Mac OSX Check
if (${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    set(MACOSX TRUE)
endif()

# OpenCV
find_package(OpenCV 4.1 REQUIRED)

# Boost
find_package(Boost 1.66 COMPONENTS system filesystem thread program_options REQUIRED)
include_directories(${Boost_INCLUDE_DIR})

# Eigen3
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# LibCV Networking (Custom networking layer)
set(LIB_CV_NETWORKING "../network_protocol/release/libcv_networking.a" CACHE PATH "The file path to the libcv_networking library")
set(LIB_CV_NETWORKING_INCLUDE_DIR "../network_protocol/include" CACHE PATH "The path of the include folder of the libcv_networking library")

# Includes for eigen for MacOS
if (${MACOSX})
    include_directories(eigen3)
endif()

# Include dirs
include_directories(${LIB_CV_NETWORKING_INCLUDE_DIR})

# Executables

# Main executable
add_executable(${PROJECT_NAME} src/main.cpp src/KITTIVisionParser.cpp)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} ${Boost_LIBRARIES} reconstruction::networking)

