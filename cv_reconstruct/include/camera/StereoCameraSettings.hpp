//
// CameraSettings.hpp
// Struct containing camera information for a stereo setup
//

#ifndef CV_RECONSTRUCT_CAMERASETTINGS_HPP
#define CV_RECONSTRUCT_CAMERASETTINGS_HPP

#include <eigen3/Eigen/Eigen>

namespace Camera
{
    struct StereoCameraSettings
    {
        int LeftCamResolutionWidth;
        int LeftCamResolutionHeight;

        int RightCamResolutionWidth;
        int RightCamResolutionHeight;

        Eigen::Matrix3f LeftK;
        Eigen::Matrix3f RightK;
    };
}

#endif //CV_RECONSTRUCT_CAMERASETTINGS_HPP
