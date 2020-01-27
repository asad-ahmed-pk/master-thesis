# Networking protocol

# Project
cmake_minimum_required(VERSION 3.10)
project(network_protocol)
set(CMAKE_CXX_STANDARD 14)

# Mac OSX Check
if (${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    set(MACOSX TRUE)
endif()

# OpenCV
find_package(OpenCV 4.1 REQUIRED)

# Boost
find_package(Boost 1.60 REQUIRED COMPONENTS system thread)
include_directories(${Boost_INCLUDE_DIR})

# Eigen3
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# Includes for eigen for MacOS
if (${MACOSX})
    include_directories(eigen3)
endif()

