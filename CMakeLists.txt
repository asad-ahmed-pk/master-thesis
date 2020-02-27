# Master Thesis
# Asad Ahmed
# Realtime 3D Reconstruction

cmake_minimum_required(VERSION 3.10)
project(master_thesis)
set(CMAKE_CXX_STANDARD 14)

# sub-projects for each component and module

add_subdirectory(kitti_vision_streamer)
add_subdirectory(network_protocol)
add_subdirectory(cv_reconstruct)

# external libs

add_subdirectory(external/Go-ICP)

