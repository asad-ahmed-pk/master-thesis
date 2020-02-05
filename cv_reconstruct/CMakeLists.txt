# CV 3D Reconstruction Module

# Project
cmake_minimum_required(VERSION 3.10)
project(cv_reconstruct)
set(CMAKE_CXX_STANDARD 14)

# Mac OSX Check
if (${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    set(MACOSX TRUE)
endif()

# OpenCV
find_package(OpenCV 4.1 REQUIRED)

# Boost
find_package(Boost 1.60 COMPONENTS filesystem REQUIRED)
include_directories(${Boost_INCLUDE_DIR})

# Eigen3
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# PCL
find_package(PCL 1.8 REQUIRED COMPONENTS common io visualization)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Includes for eigen for MacOS
if (${MACOSX})
    include_directories(eigen3)
endif()

# Include dirs
include_directories(include)
include_directories(extern)

# Sources
list(APPEND CAMERA_SOURCES
        src/camera/CameraCompute.cpp
        src/camera/CameraCalibParser.cpp
)

list(APPEND RECONSTRUCT_3D_SOURCES
        src/reconstruct/Reconstruct3D.cpp
        src/reconstruct/ReconstructionServer.cpp
        src/config/ConfigParser.cpp
)

# External sources
list(APPEND EXTERN_SOURCES
        extern/nlohmann/json.hpp
)

# Testing sources
list(APPEND TESTING_SOURCES
        extern/catch2/catch.hpp
)

# Executables

# Main reconstruction server executable
add_executable(cv_server src/main.cpp ${CAMERA_SOURCES} ${RECONSTRUCT_3D_SOURCES} ${EXTERN_SOURCES})
target_link_libraries(cv_server ${OpenCV_LIBS} ${Boost_LIBRARIES} ${PCL_LIBRARIES} reconstruction::networking)

# Camera compute module test program
add_executable(camera_compute_test src/test/camera_compute_test.cpp ${CAMERA_SOURCES})
target_link_libraries(camera_compute_test ${OpenCV_LIBS} ${Boost_LIBRARIES})

# Reconstruct module test program
add_executable(reconstruct_test src/test/reconstruct_test.cpp ${CAMERA_SOURCES} ${RECONSTRUCT_3D_SOURCES} ${EXTERN_SOURCES})
target_link_libraries(reconstruct_test ${OpenCV_LIBS} ${Boost_LIBRARIES} ${PCL_LIBRARIES})

# Catch2 Tests
add_executable(test_camera_calib_parser test/test_camera_calib_parser.cpp ${CAMERA_SOURCES} ${RECONSTRUCT_3D_SOURCES} ${EXTERN_SOURCES} ${TESTING_SOURCES})
target_link_libraries(test_camera_calib_parser ${OpenCV_LIBS} ${Boost_LIBRARIES} ${PCL_LIBRARIES})
