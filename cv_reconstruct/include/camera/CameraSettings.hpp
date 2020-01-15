//
// CameraSettings.hpp
// Structs containing settings for camera and stereo setups
//

#ifndef CV_RECONSTRUCT_CAMERASETTINGS_HPP
#define CV_RECONSTRUCT_CAMERASETTINGS_HPP

#include <eigen3/Eigen/Eigen>

namespace Camera
{
    namespace Settings
    {
        /// The settings for a single camera (intrinsics, and image resolution)
        struct CameraSettings
        {
            Eigen::Matrix3f K = Eigen::Matrix3f::Zero();
            Eigen::Vector2i ImageResolutionInPixels = Eigen::Vector2i::Zero();
        };

        /// The settings for a stereo camera rig
        struct StereoCameraSettings
        {
            CameraSettings LeftCamSettings{};
            CameraSettings RightCamSettings{};
        };
    }
}

#endif //CV_RECONSTRUCT_CAMERASETTINGS_HPP
