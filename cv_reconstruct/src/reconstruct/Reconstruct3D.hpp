//
// Reconstruct3D.hpp
// Module responsible for 3D reconstruction of stereo images
//

#ifndef CV_RECONSTRUCT_RECONSTRUCT3D_HPP
#define CV_RECONSTRUCT_RECONSTRUCT3D_HPP

#include "camera/CameraSettings.hpp"

#include <opencv2/core/core.hpp>

namespace Reconstruct
{
    class Reconstruct3D
    {
    public:
        /// Create a 3D reconstructor for the given stereo rig
        /// \param stereoSetup The calibrated, stereo rig setup
        explicit Reconstruct3D(Camera::Settings::StereoCameraSettings& stereoSetup);

        /// Generate the disparity map for the given stereo images
        /// \param leftImage The left camera image
        /// \param rightImage The right camera image
        /// \return The disparity map
        cv::Mat GenerateDisparityMap(const cv::Mat& leftImage, const cv::Mat& rightImage) const;

    private:
        Camera::Settings::StereoCameraSettings m_StereoCameraSetup;
    };
}

#endif //CV_RECONSTRUCT_RECONSTRUCT3D_HPP
