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

# Includes for eigen for MacOS
if (${MACOSX})
    include_directories(eigen3)
endif()

# Include dir
include_directories(include)

# Sources
list(APPEND CAMERA_SOURCES
        src/camera/CameraCompute.cpp
)

list(APPEND RECONSTRUCT_3D_SOURCES
        src/reconstruct/Reconstruct3D.cpp
)

# Executables

# Camera compute module test
add_executable(camera_compute_test src/test/camera_compute_test.cpp ${CAMERA_SOURCES})
target_link_libraries(camera_compute_test ${OpenCV_LIBS} ${Boost_LIBRARIES})

# Reconstruct module test
add_executable(reconstruct_test src/test/reconstruct_test.cpp ${CAMERA_SOURCES} ${RECONSTRUCT_3D_SOURCES})
target_link_libraries(reconstruct_test ${OpenCV_LIBS} ${Boost_LIBRARIES})
