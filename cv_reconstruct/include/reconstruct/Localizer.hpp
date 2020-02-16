//
// Localizer.hpp
// Localises frames to world space
//

#ifndef MASTER_THESIS_LOCALIZER_HPP
#define MASTER_THESIS_LOCALIZER_HPP

#include <eigen3/Eigen/Eigen>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <memory>

#include "StereoFrame.hpp"

namespace Reconstruct
{
    class Localizer
    {
    public:
        Localizer() = default;
        ~Localizer() = default;

        /// Transform the point cloud for the given frame
        /// \param frame The frame this point cloud is being generated for
        /// \param input The input point cloud in camera space
        /// \param output The output point cloud in world space
        void TransformPointCloud(const StereoFrame& frame, const pcl::PointCloud<pcl::PointXYZRGB>& input, pcl::PointCloud<pcl::PointXYZRGB>& output);

        /// Convert the given GPS coords to X,Y,Z using the Mercator projection
        /// \param latitude GPS latitude
        /// \param longitude GPS longitude
        /// \param altitude Altitude in meters
        /// \return Mercator projected X,Y,Z
        Eigen::Vector3f ProjectGPSToMercator(float latitude, float longitude, float altitude) const;

    private:
        Eigen::Matrix4f ComputeWorldSpaceTransform(const StereoFrame& frame);

    private:
        std::unique_ptr<Eigen::Matrix4f> m_InitialPose { nullptr };
        float m_MercatorScale { 1.0 };
    };
}

#endif //MASTER_THESIS_LOCALIZER_HPP
