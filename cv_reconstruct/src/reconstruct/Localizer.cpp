//
// Localizer.hpp
// Localises frames to world space
//

#include <cmath>
#include <pcl/common/transforms.h>

#include "reconstruct/Localizer.hpp"

#define EARTH_RADIUS_METERS 6378137.0f

namespace Reconstruct
{
    // Transform point cloud
    // Assumes frame translation is in GPS coordinates: [lat, lon, alt] => R3
    void Localizer::TransformPointCloud(const Pipeline::StereoFrame& frame, const pcl::PointCloud<pcl::PointXYZRGB> &input, pcl::PointCloud<pcl::PointXYZRGB> &output)
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
    Eigen::Matrix4f Localizer::ComputeWorldSpaceTransform(const Pipeline::StereoFrame& frame)
    {
        // compute relative pose
        Eigen::Matrix3f R0 = m_InitialPose->block(0, 0, 3, 3);
        Eigen::Matrix3f R = R0.transpose() * frame.Rotation;

        Eigen::Vector3f T0 = m_InitialPose->block(0, 3, 3, 1);
        Eigen::Vector3f T = ProjectGPSToMercator(frame.Translation(0), frame.Translation(1), frame.Translation(2)) - T0;

        Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
        transform.block(0, 0, 3, 3) = R;
        transform.block(0, 3, 3, 1) = T;

        /*
        std::cout << "\n\nLocalization\n";
        std::cout << "\nT:\n" << T;
        std::cout << "\nR:\n" << R;
        std::cout << "\n\n";
        std::cout << "\nTransform:" << transform;
        std::cout << std::endl;
        */

        return std::move(transform);
    }

    // GPS to Mercator Projection
    // X: Right, Y: Up, Z: Forward
    Eigen::Vector3f Localizer::ProjectGPSToMercator(float latitude, float longitude, float altitude) const
    {
        Eigen::Vector3f T = Eigen::Vector3f::Zero();

        float fwd = m_MercatorScale * EARTH_RADIUS_METERS * ((M_PI * longitude) / 180.0);
        float left = m_MercatorScale * EARTH_RADIUS_METERS * log(tan((M_PI * (90 + latitude)) / 360.0));
        float up = altitude;

        T(0) = -left;
        T(1) = up;
        T(2) = fwd;

        return std::move(T);
    }
}