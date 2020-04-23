# Stereopi Video File Streamer

# Project
cmake_minimum_required(VERSION 3.10)
project(stereopi_video_streamer)
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

# Includes for eigen for MacOS
if (${MACOSX})
    include_directories(eigen3)
endif()


# Executables

# Main executable
add_executable(${PROJECT_NAME} src/main.cpp)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} ${Boost_LIBRARIES} reconstruction::networking)

