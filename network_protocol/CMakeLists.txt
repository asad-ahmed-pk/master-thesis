# Networking protocol

# Project
cmake_minimum_required(VERSION 3.10)
project(network_protocol)
set(CMAKE_CXX_STANDARD 14)

set(LIB_NAME cv_networking)

# set include directories
include_directories(include)

# Boost
find_package(Boost 1.60 REQUIRED COMPONENTS system thread)
include_directories(${Boost_INCLUDE_DIR})

# Actor sources
list(APPEND ACTOR_SOURCES src/actor/RobotStreamer.cpp)

# Core sources
list(APPEND CORE_SOURCES
        src/core/StereoStream.cpp
        src/core/StereoStreamerClient.cpp
)

# The networking library
add_library(${LIB_NAME} STATIC ${CORE_SOURCES} ${ACTOR_SOURCES})
target_link_libraries(${LIB_NAME} ${Boost_LIBRARIES})
