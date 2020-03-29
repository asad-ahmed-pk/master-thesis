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
    // Get frame pose in world space
    Eigen::Matrix4f Localizer::GetFrameWorldPose(const Pipeline::StereoFrame &frame)
    {
        // record initial pose and calculate mercator scale
        if (m_InitialPose == nullptr)
        {
            // mercator scale
            m_MercatorScale = cosf((frame.Translation(0) * static_cast<float>(M_PI)) / 180.0f);

            // translation as mercator projection
            Eigen::Vector3f T = ProjectGPSToMercator(frame.Translation(0), frame.Translation(1), frame.Translation(2));

            // set initial pose
            m_InitialPose = std::make_unique<Eigen::Matrix4f>(Eigen::Matrix4f::Identity());
            m_InitialPose->block(0, 0, 3, 3) = frame.Rotation;
            m_InitialPose->block(0, 3, 3, 1) = T;
        }

        Eigen::Matrix4f T = ComputeWorldSpaceTransform(frame);

        return T;
    }

    // Transform point cloud
    // Assumes frame translation is in GPS coordinates: [lat, lon, alt] => R3
    template <typename PointT>
    Eigen::Matrix4f Localizer::TransformPointCloud(const Pipeline::StereoFrame& frame, const pcl::PointCloud<PointT>& input, pcl::PointCloud<PointT>& output)
    {
        Eigen::Matrix4f T = GetFrameWorldPose(frame);
        pcl::transformPointCloud(input, output, T);

#ifndef NDEBUG
        std::cout << "\nPose:\n" << T << std::endl;
#endif


        return T;
    }

    // Get 4x4 transformation matrix for GPS location in world space
    Eigen::Matrix4f Localizer::ComputeWorldSpaceTransform(const Pipeline::StereoFrame& frame)
    {
        // compute pose in coordinate space of first frame
        Eigen::Vector3f t = ProjectGPSToMercator(frame.Translation(0), frame.Translation(1), frame.Translation(2));

        Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
        T.block(0, 0, 3, 3) = frame.Rotation;
        T.block(0, 3, 3, 1) = (t - m_InitialPose->block(0, 3, 3, 1));

        Eigen::Matrix4f TW = T;

#ifndef NDEBUG
        std::cout << "\nEuclidean coordinates frame #" << frame.ID << "\n" << t.transpose() << std::endl;
        std::cout << "Pose for frame #" << frame.ID << "\n" << TW << std::endl;
#endif

        return std::move(TW);
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

    // Explicit template instantiation
    template Eigen::Matrix4f Localizer::TransformPointCloud<pcl::PointXYZRGB>(const Pipeline::StereoFrame&, const pcl::PointCloud<pcl::PointXYZRGB>&, pcl::PointCloud<pcl::PointXYZRGB>&);
    template Eigen::Matrix4f Localizer::TransformPointCloud<pcl::PointXYZ>(const Pipeline::StereoFrame&, const pcl::PointCloud<pcl::PointXYZ>&, pcl::PointCloud<pcl::PointXYZ>&);
}