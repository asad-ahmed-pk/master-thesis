# Networking protocol

# Project
cmake_minimum_required(VERSION 3.10)
project(network_protocol)
set(CMAKE_CXX_STANDARD 14)

set(LIB_NAME cv_networking)

# set include directories
include_directories(include)

# option to compile test client and server programs
option(COMPILE_TEST_PROGRAMS "Compile the test client and server programs" ON)

# Boost
find_package(Boost 1.60 REQUIRED COMPONENTS system thread)
include_directories(${Boost_INCLUDE_DIR})

# Core sources
list(APPEND CORE_SOURCES
        src/core/StereoStream.cpp
        src/core/StereoStreamerClient.cpp
        src/core/ReconstructionServer.cpp
)

# Protocol sources
list(APPEND PROTOCOL_SOURCES
        src/protocol/ProtocolStream.cpp
)

# Test program sources
list(APPEND TEST_SOURCES
        src/test/test_server.cpp
)

# The networking library
add_library(${LIB_NAME} STATIC ${CORE_SOURCES} ${PROTOCOL_SOURCES})
target_link_libraries(${LIB_NAME} ${Boost_LIBRARIES})

# Test programs
if (${COMPILE_TEST_PROGRAMS})
    add_executable(test_server ${TEST_SOURCES})
    target_link_libraries(test_server ${LIB_NAME})
endif ()
