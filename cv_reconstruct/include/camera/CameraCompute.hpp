//
// CameraCompute.hpp
// Computation functions for computing camera parameters and matrices
//

#ifndef CV_RECONSTRUCT_CAMERACOMPUTE_HPP
#define CV_RECONSTRUCT_CAMERACOMPUTE_HPP

#include <eigen3/Eigen/Eigen>
#include <opencv2/core/core.hpp>

#include "camera/CameraSettings.hpp"

namespace Camera
{
    class CameraCompute
    {
    public:
        /// Construct compute module with given stereo camera setup
        /// \param settings The stereo camera setup that will be used to run computations
        CameraCompute(Settings::StereoCameraSettings settings);

        /// Compute the fundamental matrix with the given left and right image
        /// \param leftImage The left stereo image
        /// \param rightImage The right stereo image
        /// \param RL Rotation matrix for left camera
        /// \param RR Rotation matrix for right camera
        /// \return The computed fundamental matrix
        Eigen::Matrix3f FundamentalMatrix(const cv::Mat &leftImage, const cv::Mat &rightImage, const Eigen::Matrix3f& RL, const Eigen::Matrix3f& RR);

        /// Compute the essential matrix with the given left and right image
        /// \param leftImage The left stereo image
        /// \param rightImage The right stereo image
        /// \param KL The calibration matrix for the left camera
        /// \param KR The calibration matrix for the right camera
        /// \return The computed essential matrix
        Eigen::Matrix3f EssentialMatrix(const cv::Mat& leftImage, const cv::Mat& rightImage, const Eigen::Matrix3f& KL, const Eigen::Matrix3f& KR);

        /// Get a copy of the updated, rectified stereo camera setup
        /// \return Updated stereo camera settings with rectification information for new projection and transform matrices
        Settings::StereoCameraSettings GetRectifiedStereoSettings();

    private:
        void ComputeMatchingFeatures(const cv::Mat& leftImage, const cv::Mat& rightImage, std::vector<cv::Point2f>& pointsLeft, std::vector<cv::Point2f>& pointsRight);
        void Rectify();

    private:
        bool m_IsStereoRectified { false };
        Settings::StereoCameraSettings m_StereoSettings;
    };
}

#endif //CV_RECONSTRUCT_CAMERACOMPUTE_HPP
