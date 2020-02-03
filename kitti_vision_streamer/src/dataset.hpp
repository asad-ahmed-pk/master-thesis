//
// dataset.hpp
// Definitions for encapsulating records from the dataset into convenient types
//

#ifndef KITTI_VISION_STREAMER_DATASET_HPP
#define KITTI_VISION_STREAMER_DATASET_HPP

#include <string>
#include <eigen3/Eigen/Eigen>

/// The calibration data
struct Calib
{
    // Intrinsic matrices
    Eigen::Matrix3f K1 = Eigen::Matrix3f::Zero();
    Eigen::Matrix3f K2 = Eigen::Matrix3f::Zero();

    // Distortion coefficients
    Eigen::VectorXf D1;
    Eigen::VectorXf D2;

    // Translation from camera 1 to 2
    Eigen::Vector3f T = Eigen::Vector3f::Zero();

    // Rotation from camera 1 to 2
    Eigen::Matrix3f R = Eigen::Matrix3f::Zero();
};

/// A sample from the dataset. Encodes 2 images and the vehicle pose at the time the images were taken
struct DataSample
{
    // Camera image file paths
    std::string Camera1ImagePath;
    std::string Camera2ImagePath;

    // Pose in world in space (of vehicle)
    Eigen::Vector3f T = Eigen::Vector3f::Zero();
    Eigen::Matrix3f R = Eigen::Matrix3f::Zero();
};

#endif //KITTI_VISION_STREAMER_DATASET_HPP
