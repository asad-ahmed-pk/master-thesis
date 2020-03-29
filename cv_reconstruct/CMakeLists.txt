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
find_package(PCL 1.8 REQUIRED COMPONENTS common io visualization filters registration)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# G2O
find_package(G2O REQUIRED)
include_directories(${G2O_INCLUDE_DIR})

# G2o libs
LIST(APPEND G2O_LIBS
        cxsparse
        cholmod
        g2o_cli g2o_simulator
        g2o_solver_slam2d_linear g2o_types_icp g2o_types_slam2d
        g2o_core g2o_solver_csparse g2o_solver_structure_only
        g2o_types_sba g2o_types_slam3d g2o_csparse_extension
        g2o_solver_dense g2o_stuff
        g2o_types_sclam2d g2o_parser g2o_solver_pcg
        g2o_types_data g2o_types_sim3
)

# Includes for eigen for MacOS
if (${MACOSX})
    include_directories(eigen3)
endif()

# Include dirs
include_directories(include)
include_directories(extern)

# Sources
list(APPEND CAMERA_SOURCES
        include/camera/CameraCalib.hpp
        include/camera/CameraCalibParser.hpp
        include/camera/CameraCompute.hpp
        src/camera/CameraCompute.cpp
        src/camera/CameraCalibParser.cpp
)

list(APPEND RECONSTRUCT_3D_SOURCES
        include/reconstruct/Reconstruct3D.hpp
        include/reconstruct/Reconstruct3DTypes.hpp
        include/reconstruct/ReconstructStatusCode.hpp
        src/reconstruct/Reconstruct3D.cpp
        src/reconstruct/Localizer.cpp
)

list(APPEND POINT_CLOUD_SOURCES
        src/point_cloud/PointCloudPostProcessor.cpp
        src/point_cloud/FeatureExtractor.cpp
        src/point_cloud/PointCloudRegistration.cpp
)

list(APPEND SERVER_SOURCES
        include/server/MessageConverter.hpp
        include/server/ReconstructionServer.hpp
        include/server/server_constants.hpp
        include/server/ServerUserInterface.hpp
        src/server/ReconstructionServer.cpp
        src/server/MessageConverter.cpp
        src/server/ServerUserInterface.cpp
)

list(APPEND CONFIG_SOURCES
        include/config/Config.hpp
        include/config/ConfigParser.hpp
        src/config/ConfigParser.cpp
)

list (APPEND PIPELINE_SOURCES
        include/pipeline/FrameFeatureExtractor.hpp
        include/pipeline/OpticalFlowEstimator.hpp
        include/pipeline/PipelineResult.hpp
        include/pipeline/ReconstructionPipeline.hpp
        include/pipeline/StereoFrame.hpp
        src/pipeline/FrameFeatureExtractor.cpp
        src/pipeline/OpticalFlowEstimator.cpp
)

list (APPEND SYSTEM_SOURCES
        include/system/MapBlock.hpp
        include/system/MapDataBase.hpp
        include/system/MappingSystem.hpp
        include/system/ReconstructionSystem.hpp
        include/system/Tracker.hpp
        include/system/TrackingFrame.hpp
        include/system/OptimisationGraph.hpp
        include/system/KeyFrameDatabase.hpp
        src/system/ReconstructionSystem.cpp
        src/system/MappingSystem.cpp
        src/system/MapBlock.cpp
        src/system/MapDataBase.cpp
        src/system/TrackingFrame.cpp
        src/system/Tracker.cpp
        src/system/OptimisationGraph.cpp
        src/system/KeyFrameDatabase.cpp
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
add_executable(cv_server src/main.cpp ${CAMERA_SOURCES}
        ${RECONSTRUCT_3D_SOURCES}
        ${SERVER_SOURCES}
        ${CONFIG_SOURCES}
        ${PIPELINE_SOURCES}
        ${SYSTEM_SOURCES}
        ${POINT_CLOUD_SOURCES}
        ${EXTERN_SOURCES}
)
target_link_libraries(cv_server ${OpenCV_LIBS} ${Boost_LIBRARIES} ${PCL_LIBRARIES} ${G2O_LIBS} reconstruction::networking)

# Reconstruct module test program
add_executable(reconstruct_test src/test/reconstruct_test_kitti_rectified.cpp src/test/tests_common.cpp
        ${CAMERA_SOURCES}
        ${RECONSTRUCT_3D_SOURCES}
        ${CONFIG_SOURCES}
        ${PIPELINE_SOURCES}
        ${POINT_CLOUD_SOURCES}
        ${EXTERN_SOURCES}
)
target_link_libraries(reconstruct_test ${OpenCV_LIBS} ${Boost_LIBRARIES} ${PCL_LIBRARIES})

# Localization module test program
add_executable(localization_test src/test/localization_test_kitti.cpp src/test/tests_common.cpp
        ${CAMERA_SOURCES}
        ${RECONSTRUCT_3D_SOURCES}
        ${CONFIG_SOURCES}
        ${PIPELINE_SOURCES}
        ${POINT_CLOUD_SOURCES}
        ${EXTERN_SOURCES}
)
target_link_libraries(localization_test ${OpenCV_LIBS} ${Boost_LIBRARIES} ${PCL_LIBRARIES})

# Pipeline module test program
add_executable(pipeline_test src/test/pipeline_test.cpp src/test/tests_common.cpp
        ${CAMERA_SOURCES}
        ${RECONSTRUCT_3D_SOURCES}
        ${CONFIG_SOURCES}
        ${PIPELINE_SOURCES}
        ${POINT_CLOUD_SOURCES}
        ${EXTERN_SOURCES}
)
target_link_libraries(pipeline_test ${OpenCV_LIBS} ${Boost_LIBRARIES} ${PCL_LIBRARIES} ${G2O_LIBS})

# Feature extraction module test program
add_executable(feature_extract_test src/test/feature_extraction_test.cpp src/test/tests_common.cpp
        ${CAMERA_SOURCES}
        ${RECONSTRUCT_3D_SOURCES}
        ${CONFIG_SOURCES}
        ${PIPELINE_SOURCES}
        ${POINT_CLOUD_SOURCES}
        ${EXTERN_SOURCES}
)
target_link_libraries(feature_extract_test ${OpenCV_LIBS} ${Boost_LIBRARIES} ${PCL_LIBRARIES})

# Feature extraction module test program
add_executable(image_feature_test src/test/image_feature_test.cpp
        ${CAMERA_SOURCES}
        ${RECONSTRUCT_3D_SOURCES}
        ${CONFIG_SOURCES}
        ${PIPELINE_SOURCES}
        ${POINT_CLOUD_SOURCES}
        ${EXTERN_SOURCES}
)
target_link_libraries(image_feature_test ${OpenCV_LIBS} ${Boost_LIBRARIES} ${PCL_LIBRARIES})

# Alignment test
add_executable(alignment_test src/test/alignment_test.cpp)
target_link_libraries(alignment_test ${OpenCV_LIBS} ${Boost_LIBRARIES} ${PCL_LIBRARIES})

# Catch2 Tests
add_executable(test_camera_calib_parser test/test_camera_calib_parser.cpp ${CAMERA_SOURCES} ${RECONSTRUCT_3D_SOURCES} ${CONFIG_SOURCES} ${EXTERN_SOURCES} ${TESTING_SOURCES})
target_link_libraries(test_camera_calib_parser ${OpenCV_LIBS} ${Boost_LIBRARIES} ${PCL_LIBRARIES})
