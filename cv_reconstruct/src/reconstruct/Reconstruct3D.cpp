//
// Reconstruct3D.cpp
// Module responsible for 3D reconstruction of stereo images
//

#include "Reconstruct3D.hpp"

#include <opencv2/calib3d/calib3d.hpp>

namespace Reconstruct
{
    // Constructor
    Reconstruct3D::Reconstruct3D(Camera::Settings::StereoCameraSettings &stereoSetup) : m_StereoCameraSetup(stereoSetup)
    {

    }

    // Disparity map
    cv::Mat Reconstruct3D::GenerateDisparityMap(const cv::Mat &leftImage, const cv::Mat &rightImage) const
    {
        // convert stereo setup matrices to CV

        // TODO: stereo rectification

        cv::Mat disparity;
        return disparity;
    }
}
