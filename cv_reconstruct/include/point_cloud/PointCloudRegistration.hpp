//
// PointCloudRegistration.hpp
// Handles point cloud registration
//

#ifndef MASTER_THESIS_POINTCLOUDREGISTRATION_HPP
#define MASTER_THESIS_POINTCLOUDREGISTRATION_HPP

#include <vector>

#include <opencv2/core/types.hpp>
#include <pcl/correspondence.h>

#include "point_cloud_constants.hpp"

namespace PointCloud
{
    class PointCloudRegistration
    {
    public:
        /// Create a default instance of the registration pipeline
        PointCloudRegistration() = default;

        ~PointCloudRegistration() = default;

        /// Generate point clouds with correspondences between them using the 2D keypoints and correspondences
        /// \param sourceKeypoints2D The 2D image keypoints for the source image
        /// \param targetKeypoints2D The 2D image keypoints for the target image
        /// \param source3DReprojection The 3D image with X,Y,Z coords for the source
        /// \param target3DReprojection The 3D image with X,Y,Z coords for the target
        /// \param matches The 2D correspondences between the 2 images
        /// \param invalidZValue The Z value that is to be considered invalid (no disparity)
        /// \param sourceCloud The generated source cloud
        /// \param targetCloud The generated target cloud
        /// \param correspondences The correspondences between the 3D point clouds
        void GeneratePointCloudWithCorrespondencesFrom2D(const std::vector<cv::KeyPoint>& sourceKeypoints2D, const std::vector<cv::KeyPoint>& targetKeypoints2D,
                                                         const cv::Mat& source3DReprojection, const cv::Mat& target3DReprojection,
                                                         const std::vector<cv::DMatch>& matches, float invalidZValue,
                                                         pcl::PointCloud<pcl::PointXYZ>::Ptr sourceCloud, pcl::PointCloud<pcl::PointXYZ>::Ptr targetCloud,
                                                         pcl::CorrespondencesPtr correspondences) const;

        /// Estimate the 3D rigid body transform between source and target point clouds using the correspondences
        /// \param source The source point cloud
        /// \param target The target point cloud
        /// \param correspondences The correspondences between source and target
        /// \return An estimated 3D transform between the point clouds
        Eigen::Matrix4f Estimate3DTransform(pcl::PointCloud<pcl::PointXYZ>::Ptr source, pcl::PointCloud<pcl::PointXYZ>::Ptr target, pcl::CorrespondencesConstPtr correspondences) const;
    };
}

#endif //MASTER_THESIS_POINTCLOUDREGISTRATION_HPP
