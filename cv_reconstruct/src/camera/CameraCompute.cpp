//
// CameraCompute.cpp
// Computation functions for computing camera parameters and matrices
//

#include "camera/CameraCompute.hpp"

namespace Camera
{
    CameraCompute::CameraCompute(StereoCameraSettings settings) : m_StereoSettings(std::move(settings))
    {

    }

    // Compute F matrix
    Eigen::Matrix3f CameraCompute::FundamentalMatrix(const cv::Mat &leftImage, const cv::Mat &rightImage)
    {
        // TODO: compute F using in built settings with K
        Eigen::Matrix3f F;
        return F;
    }
}