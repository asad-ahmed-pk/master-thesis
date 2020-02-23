//
// PointCloudRegistration.cpp
// Handles point cloud registration
//

#include <opencv2/core/core.hpp>
#include <pcl/common/geometry.h>
#include <pcl/registration/transformation_estimation_svd.h>

#include "point_cloud/PointCloudRegistration.hpp"

namespace PointCloud
{
    void PointCloudRegistration::GeneratePointCloudWithCorrespondencesFrom2D(const std::vector<cv::KeyPoint>& sourceKeypoints2D, const std::vector<cv::KeyPoint>& targetKeypoints2D,
                                                     const cv::Mat& source3DReprojection, const cv::Mat& target3DReprojection,
                                                     const std::vector<cv::DMatch>& matches, float invalidZValue,
                                                     pcl::PointCloud<pcl::PointXYZ>::Ptr sourceCloud, pcl::PointCloud<pcl::PointXYZ>::Ptr targetCloud,
                                                     pcl::CorrespondencesPtr correspondences) const
    {
        pcl::PointXYZ point;
        cv::Vec3f coordsSource; cv::Vec3f coordsTarget;

        // create both point clouds and compute correspondences simultaneously
        int index = 0;
        for (const auto& match : matches)
        {
            // get the 3D coords for the keypoints in this match
            coordsSource = source3DReprojection.at<float>(static_cast<int>(sourceKeypoints2D[match.queryIdx].pt.y),
                                                          static_cast<int>(sourceKeypoints2D[match.queryIdx].pt.x));
            coordsTarget = target3DReprojection.at<float>(static_cast<int>(targetKeypoints2D[match.queryIdx].pt.y),
                                                          static_cast<int>(targetKeypoints2D[match.queryIdx].pt.x));

            // skip invalid disparity generated Z values
            if (coordsSource[2] >= invalidZValue || coordsTarget[2] >= invalidZValue) {
                continue;
            }

            // create points for both point clouds
            sourceCloud->push_back(pcl::PointXYZ(coordsSource[0], -coordsSource[1], coordsSource[2]));
            targetCloud->push_back(pcl::PointXYZ(coordsTarget[0], -coordsTarget[1], coordsTarget[2]));

            // add point cloud correspondences
            correspondences->push_back(pcl::Correspondence(index, index, pcl::geometry::distance(sourceCloud->points[index], targetCloud->points[index])));

            index++;
        }
    }

    // 3D rigid body transform using 2D information
    Eigen::Matrix4f PointCloudRegistration::Estimate3DTransform(pcl::PointCloud<pcl::PointXYZ>::Ptr source, pcl::PointCloud<pcl::PointXYZ>::Ptr target, pcl::CorrespondencesConstPtr correspondences) const
    {
        // estimate 3D transform
        Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
        pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ> estimator;
        estimator.estimateRigidTransformation(*source, *target, *correspondences, T);

        return T;
    }
}
