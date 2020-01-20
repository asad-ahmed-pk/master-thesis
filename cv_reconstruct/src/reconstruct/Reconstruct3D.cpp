//
// Reconstruct3D.cpp
// Module responsible for 3D reconstruction of stereo images
//

#include "Reconstruct3D.hpp"

#include <opencv2/core/eigen.hpp>

namespace Reconstruct
{
    const int SGM_MIN_DISPARITY = 0;
    const int SGM_BLOCK_SIZE = 3;
    const int SGM_NUM_DISPARITIES = 16;

    constexpr int SGM_P1 = 8 * 3 * SGM_BLOCK_SIZE * SGM_BLOCK_SIZE;
    constexpr int SGM_P2 = 32 * 3 * SGM_BLOCK_SIZE * SGM_BLOCK_SIZE;

    // Constructor
    Reconstruct3D::Reconstruct3D(Camera::Settings::StereoCameraSettings &stereoSetup) : m_StereoCameraSetup(stereoSetup)
    {
        // setup stereo matcher
        m_StereoMatcher = cv::StereoSGBM::create(SGM_MIN_DISPARITY, SGM_NUM_DISPARITIES, SGM_BLOCK_SIZE, SGM_P1, SGM_P2);
    }

    // Disparity map
    cv::Mat Reconstruct3D::GenerateDisparityMap(const cv::Mat &leftImage, const cv::Mat &rightImage) const
    {
        // convert to cv from eigen
        cv::Mat K1, K2;
        std::vector<float> D1, D2;

        // re-project images to be rectified
        cv::Mat leftImageRectified, rightImageRectified;
        cv::undistortPoints(leftImage, leftImageRectified, K1, D1, m_StereoCameraSetup.RectifiedSettings.RL, m_StereoCameraSetup.RectifiedSettings.PL);
        cv::undistortPoints(rightImage, rightImageRectified, K2, D2, m_StereoCameraSetup.RectifiedSettings.RR, m_StereoCameraSetup.RectifiedSettings.PR);

        // compute disparity
        cv::Mat disparity;
        m_StereoMatcher->compute(leftImageRectified, rightImageRectified, disparity);

        return disparity;
    }
}
