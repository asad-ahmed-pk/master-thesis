//
// Localizer.hpp
// Localises frames to world space
//

#include <cmath>
#include <pcl/common/transforms.h>

#include "reconstruct/Localizer.hpp"

#define EARTH_RADIUS_METERS 6378137.0

namespace Reconstruct
{
    // Transform point cloud
    void Localizer::TransformPointCloud(const StereoFrame& frame, const pcl::PointCloud<pcl::PointXYZRGB> &input, pcl::PointCloud<pcl::PointXYZRGB> &output)
    {
        Eigen::Matrix4f T = ComputeWorldSpaceTransform(frame);
        pcl::transformPointCloud(input, output, T);
    }

    // Get 4x4 transformation matrix for GPS location
    Eigen::Matrix4f Localizer::ComputeWorldSpaceTransform(const StereoFrame& frame)
    {
        // take this frame as initial frame
        if (!m_InitialLatitideSet) {
            m_InitialLatitude = frame.Translation(1);
            m_InitialLatitideSet = true;
        }

        // rotation from IMU
        Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                T(i, j) = frame.Rotation(i, j);
            }
        }

        // translation from GPS (mercator projection)
        float s = cos((m_InitialLatitude * M_PI) / 180.0f);

        T(0, 3) = s * EARTH_RADIUS_METERS * ((M_PI * frame.Translation(1)) / 180.0);
        T(1, 3) = s * EARTH_RADIUS_METERS * log(tan((M_PI * (90 + frame.Translation(0))) / 360.0));
        T(2, 3) = frame.Translation(2);

        return std::move(T);
    }
}