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
        // record initial pose and calculate mercator scale
        if (m_InitialPose == nullptr)
        {
            // mercator scale
            m_MercatorScale = cosf((frame.Translation(0) * static_cast<float>(M_PI)) / 180.0f);

            // translation as mercator projection
            Eigen::Vector3f T = ProjectGPSToMercator(frame.Translation(0), frame.Translation(1), frame.Translation(2));

            // pose
            m_InitialPose = std::make_unique<Eigen::Matrix4f>(Eigen::Matrix4f::Identity());
            m_InitialPose->block(0, 0, 3, 3) = frame.Rotation;
            m_InitialPose->block(0, 3, 3, 1) = T;
        }

        Eigen::Matrix4f T = ComputeWorldSpaceTransform(frame);
        pcl::transformPointCloud(input, output, T);
    }

    // Get 4x4 transformation matrix for GPS location
    Eigen::Matrix4f Localizer::ComputeWorldSpaceTransform(const StereoFrame& frame)
    {
        // compute relative pose
        Eigen::Matrix3f R0 = m_InitialPose->block(0, 0, 3, 3);
        Eigen::Matrix3f R = R0.transpose() * frame.Rotation;

        Eigen::Vector3f T0 = m_InitialPose->block(0, 3, 3, 1);
        Eigen::Vector3f T = ProjectGPSToMercator(frame.Translation(0), frame.Translation(1), frame.Translation(2)) - T0;

        Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
        transform.block(0, 0, 3, 3) = R;
        transform.block(0, 3, 3, 1) = T;

        return std::move(transform);
    }

    // GPS to Mercator Projection
    Eigen::Vector3f Localizer::ProjectGPSToMercator(float latitude, float longitude, float altitude) const
    {
        Eigen::Vector3f T = Eigen::Vector3f::Zero();

        T(0) = m_MercatorScale * EARTH_RADIUS_METERS * ((M_PI * longitude) / 180.0);
        T(1) = m_MercatorScale * EARTH_RADIUS_METERS * log(tan((M_PI * (90 + latitude)) / 360.0));
        T(2) = altitude;

        return std::move(T);
    }
}