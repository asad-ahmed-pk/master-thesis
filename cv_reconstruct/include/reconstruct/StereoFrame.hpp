//
// StereoFrame.hpp
// Represents a stereo frame from a robot with stereo images and a pose in world space
//

#ifndef MASTER_THESIS_STEREOFRAME_HPP
#define MASTER_THESIS_STEREOFRAME_HPP

#include <opencv2/core/core.hpp>
#include <eigen3/Eigen/Eigen>

namespace Reconstruct
{
    struct StereoFrame
    {
        long ID;
        cv::Mat LeftImage;
        cv::Mat RightImage;
        Eigen::Vector3f Translation = Eigen::Vector3f::Zero();
        Eigen::Matrix3f Rotation = Eigen::Matrix3f::Zero();
    };
}

#endif //MASTER_THESIS_STEREOFRAME_HPP
