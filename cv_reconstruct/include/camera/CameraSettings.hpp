//
// CameraSettings.hpp
// Structs containing settings for camera and stereo setups
//

#ifndef CV_RECONSTRUCT_CAMERASETTINGS_HPP
#define CV_RECONSTRUCT_CAMERASETTINGS_HPP

#include <opencv2/core/core.hpp>
#include <eigen3/Eigen/Eigen>

namespace Camera
{
    namespace Settings
    {
        /// The settings for a single camera (intrinsics (K), distortion coeffs (D), and image resolution)
        struct CameraSettings
        {
            Eigen::Matrix3f K = Eigen::Matrix3f::Zero();
            Eigen::VectorXf D;
            Eigen::Vector2i ImageResolutionInPixels = Eigen::Vector2i::Zero();
        };

        /// Settings for the stereo rectified camera system
        struct StereoRectifiedSettings
        {
            // Rectified rotation transform for left (RL) and right (RR) cameras
            cv::Mat RL;
            cv::Mat RR;

            // Projection matrices for the rectified coordinate system for the left (PL), and right (PR) cameras
            cv::Mat PL;
            cv::Mat PR;

            // 4x4 disparity to depth mapping matrix
            cv::Mat Q;

            // Valid image spaces after rectification in rectified images
            cv::Rect ValidRectLeft;
            cv::Rect ValidRectRight;
        };

        /// The settings for a stereo camera rig
        struct StereoCameraSettings
        {
            // Left and right camera settings
            CameraSettings LeftCamSettings{};
            CameraSettings RightCamSettings{};

            // Epipolar geometry with essential, fundamental, and relative rotation of 2nd camera with respect to 1st
            Eigen::Matrix3f E = Eigen::Matrix3f::Zero();
            Eigen::Matrix3f F = Eigen::Matrix3f::Zero();
            Eigen::Matrix3f R = Eigen::Matrix3f::Zero();

            // Transform from left camera origin to right camera origin
            Eigen::Vector3f T = Eigen::Vector3f::Zero();

            // Stereo rectified settings
            StereoRectifiedSettings RectifiedSettings;
        };
    }
}

#endif //CV_RECONSTRUCT_CAMERASETTINGS_HPP
