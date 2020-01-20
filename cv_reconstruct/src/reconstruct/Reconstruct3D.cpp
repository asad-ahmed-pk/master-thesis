//
// Reconstruct3D.cpp
// Module responsible for 3D reconstruction of stereo images
//

#include "reconstruct/Reconstruct3D.hpp"

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/eigen.hpp>

namespace Reconstruct
{
    const int SGM_MIN_DISPARITY = 0;
    const int SGM_BLOCK_SIZE = 9;
    const int SGM_NUM_DISPARITIES = 112;

    constexpr int SGM_P1 = 8 * 3 * SGM_BLOCK_SIZE * SGM_BLOCK_SIZE;
    constexpr int SGM_P2 = 32 * 3 * SGM_BLOCK_SIZE * SGM_BLOCK_SIZE;

    // Constructor
    Reconstruct3D::Reconstruct3D(Camera::Settings::StereoCameraSettings &stereoSetup) : m_StereoCameraSetup(stereoSetup)
    {
        // setup stereo matcher
        m_StereoMatcher = cv::StereoSGBM::create(SGM_MIN_DISPARITY, SGM_NUM_DISPARITIES, SGM_BLOCK_SIZE, SGM_P1, SGM_P2);
        m_StereoMatcher->setSpeckleRange(20);
        m_StereoMatcher->setUniquenessRatio(10);
        m_StereoMatcher->setSpeckleWindowSize(100);
        m_StereoMatcher->setSpeckleRange(32);
        m_StereoMatcher->setDisp12MaxDiff(1);
    }

    // Disparity map
    cv::Mat Reconstruct3D::GenerateDisparityMap(const cv::Mat &leftImage, const cv::Mat &rightImage) const
    {
        // convert to cv from eigen
        cv::Mat K1, K2;
        std::vector<float> D1, D2;

        cv::eigen2cv(m_StereoCameraSetup.LeftCamSettings.K, K1);
        K1.convertTo(K1, CV_64F);

        cv::eigen2cv(m_StereoCameraSetup.RightCamSettings.K, K2);
        K2.convertTo(K2, CV_64F);

        cv::eigen2cv(m_StereoCameraSetup.LeftCamSettings.D, D1);
        cv::eigen2cv(m_StereoCameraSetup.RightCamSettings.D, D2);

        // other required matrices
        cv::Mat R1 = m_StereoCameraSetup.RectifiedSettings.RL;
        cv::Mat R2 = m_StereoCameraSetup.RectifiedSettings.RR;
        cv::Mat P1 = m_StereoCameraSetup.RectifiedSettings.PL;
        cv::Mat P2 = m_StereoCameraSetup.RectifiedSettings.PR;

        cv::Size size { m_StereoCameraSetup.LeftCamSettings.ImageResolutionInPixels.x(), m_StereoCameraSetup.LeftCamSettings.ImageResolutionInPixels.y() };

        // remap image to rectified coords
        cv::Mat map11, map12, map21, map22;
        initUndistortRectifyMap(K1, D1, R1, P1, size, CV_16SC2, map11, map12);
        initUndistortRectifyMap(K2, D2, R2, P2, size, CV_16SC2, map21, map22);

        cv::Mat leftImageRectified, rightImageRectified;
        cv::remap(leftImage, leftImageRectified, map11, map12, cv::INTER_LINEAR);
        cv::remap(rightImage, rightImageRectified, map21, map22, cv::INTER_LINEAR);

        // debugging: write out to file
        cv::imwrite("left_rect.png", leftImageRectified);
        cv::imwrite("right_rect.png", rightImageRectified);

        // compute disparity
        cv::Mat disparity;
        m_StereoMatcher->compute(leftImageRectified, rightImageRectified, disparity);

        return disparity;
    }
}
