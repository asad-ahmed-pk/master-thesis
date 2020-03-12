//
// PointCloudRegistration.cpp
// Handles point cloud registration
//

#include <opencv2/core/core.hpp>
#include <pcl/common/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/geometry.h>
#include <pcl/common/transforms.h>

#include "reconstruct/Reconstruct3D.hpp"
#include "point_cloud/PointCloudRegistration.hpp"

namespace PointCloud
{
    // Constructor
    PointCloudRegistration::PointCloudRegistration(const Config::Config& config)
    {
        // setup ICP params from config file
        m_ICP.setMaximumIterations(config.PointCloudRegistration.ICP.NumMaxIterations);
        m_ICP.setTransformationEpsilon (config.PointCloudRegistration.ICP.TransformEpsilon);
        m_ICP.setEuclideanFitnessEpsilon (config.PointCloudRegistration.ICP.EuclideanFitnessEpsilon);
        m_ICP.setRANSACIterations(config.PointCloudRegistration.ICP.NumRansacIterations);
    }

    // Simple ICP alignment
    void PointCloudRegistration::AlignPointClouds(pcl::PointCloud<pcl::PointXYZRGB>::Ptr source, pcl::PointCloud<pcl::PointXYZRGB>::Ptr target)
    {
        pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;

        icp.setMaximumIterations(10);
        icp.setRANSACIterations(10);
        icp.setTransformationEpsilon (1e-8);
        icp.setEuclideanFitnessEpsilon (0.01);

        icp.setInputSource(source);
        icp.setInputTarget(target);
        icp.align(*source);

#ifndef NDEBUG
        std::cout << "\nICP T: \n" << icp.getFinalTransformation();
#endif
    }

    // Align the frame with the previous frame using 2D image keypoints and descriptors with the given world transform
    void PointCloudRegistration::RegisterFrameWithPreviousFrame(const cv::Mat& image, const cv::Mat& projected3D, const Eigen::Matrix4f& worldTransform, pcl::PointCloud<pcl::PointXYZRGB>::Ptr input, pcl::PointCloud<pcl::PointXYZRGB>::Ptr registeredCloud)
    {
        // compute 2D correspondences with previous frame as target
        std::vector<cv::KeyPoint> sourceKeypoints;
        cv::Mat sourceDescriptors;
        std::vector<cv::DMatch> matches;
        m_2DFeatureExtractor.ComputeMatchesWithImage(m_TargetData.Descriptors, image, sourceKeypoints, sourceDescriptors, matches);

        // compute point clouds and 3D correspondences from 2D correspondences
        pcl::PointCloud<pcl::PointXYZ>::Ptr sourceCloud { new pcl::PointCloud<pcl::PointXYZ>() };
        pcl::PointCloud<pcl::PointXYZ>::Ptr targetCloud { new pcl::PointCloud<pcl::PointXYZ>() };
        pcl::CorrespondencesPtr correspondences { new pcl::Correspondences() };
        float maxDistance = 0.0;

        GeneratePointCloudsFrom2DCorrespondences(sourceKeypoints, projected3D, matches, worldTransform, sourceCloud, targetCloud, correspondences, maxDistance);

        // estimate 3D rigid-body transform between source and target and apply to final registered cloud
        Eigen::Matrix4f T = Estimate3DTransform(sourceCloud, targetCloud, correspondences, maxDistance);
        pcl::transformPointCloud(*input, *registeredCloud, T);

        // set this source data as the target for next frame's processing
        m_TargetData.WorldTransform = worldTransform;
        m_TargetData.Keypoints = sourceKeypoints;
        m_TargetData.Descriptors = sourceDescriptors.clone();
        m_TargetData.Projected3DImage = projected3D.clone();
        m_TargetData.Image = image.clone();
    }

    // 3D correspondences and point clouds from 2D features
    void PointCloudRegistration::GeneratePointCloudsFrom2DCorrespondences(const std::vector<cv::KeyPoint>& sourceKeypoints2D, const cv::Mat& source3DReprojection,
                                                     const std::vector<cv::DMatch>& matches2D, const Eigen::Matrix4f& worldTransform,
                                                     pcl::PointCloud<pcl::PointXYZ>::Ptr sourceCloud, pcl::PointCloud<pcl::PointXYZ>::Ptr targetCloud,
                                                     pcl::CorrespondencesPtr correspondences, float& maxDistance) const
    {
        pcl::PointXYZ point;
        cv::Vec3f coordsSource; cv::Vec3f coordsTarget;
        float invalidZValue = Reconstruct::Reconstruct3D::GetInvalidDisparityZValue();

        // create both point clouds and compute correspondences simultaneously
        int index = 0;
        for (const auto& match : matches2D)
        {
            // get the 3D coords for the keypoints in this match
            coordsSource = source3DReprojection.at<cv::Vec3f>(static_cast<int>(sourceKeypoints2D[match.queryIdx].pt.y),
                                                          static_cast<int>(sourceKeypoints2D[match.queryIdx].pt.x));
            coordsTarget = m_TargetData.Projected3DImage.at<cv::Vec3f>(static_cast<int>(m_TargetData.Keypoints[match.queryIdx].pt.y),
                                                          static_cast<int>(m_TargetData.Keypoints[match.queryIdx].pt.x));

            // skip invalid disparity generated Z values
            if (coordsSource[2] >= invalidZValue || coordsTarget[2] >= invalidZValue) {
                continue;
            }

            // create points for both point clouds
            sourceCloud->push_back(pcl::PointXYZ(coordsSource[0], -coordsSource[1], coordsSource[2]));
            targetCloud->push_back(pcl::PointXYZ(coordsTarget[0], -coordsTarget[1], coordsTarget[2]));

            // add point cloud correspondences
            correspondences->push_back(pcl::Correspondence(index, index, 1.0));

            index++;
        }

        // apply world space transforms to both clouds
        pcl::transformPointCloud(*sourceCloud, *sourceCloud, worldTransform);
        pcl::transformPointCloud(*targetCloud, *targetCloud, m_TargetData.WorldTransform);

        // compute distances and max distance for each correspondence
        int i = 0;
        maxDistance = -1.0f;
        for (auto& corr : *correspondences)
        {
            corr.distance = pcl::geometry::distance(sourceCloud->points[i], targetCloud->points[i]);
            if (corr.distance > maxDistance) {
                maxDistance = corr.distance;
            }
            i++;
        }
    }

    // 3D rigid body transform using 2D information
    Eigen::Matrix4f PointCloudRegistration::Estimate3DTransform(pcl::PointCloud<pcl::PointXYZ>::Ptr source, pcl::PointCloud<pcl::PointXYZ>::Ptr target, pcl::CorrespondencesConstPtr correspondences, float maxDistance)
    {
        // ICP for alignment of point clouds
        pcl::PointCloud<pcl::PointXYZ> aligned;
        m_ICP.setMaxCorrespondenceDistance(maxDistance);

        m_ICP.setInputSource(source);
        m_ICP.setInputTarget(target);
        m_ICP.align(aligned);

        Eigen::Matrix4f T = m_ICP.getFinalTransformation();

        // debugging: save to disk as RGB and output
#ifndef NDEBUG
        pcl::PointCloud<pcl::PointXYZRGB> p1; pcl::PointCloud<pcl::PointXYZRGB> p2; pcl::PointCloud<pcl::PointXYZRGB> p3;
        pcl::copyPointCloud(*source, p1); pcl::copyPointCloud(*target, p2); pcl::copyPointCloud(aligned, p3);
        for (int i = 0; i < p1.points.size(); i++) {
            p1.points[i].r = 255;
            p2.points[i].b = 255;
            p3.points[i].g = 255;
        }
        pcl::io::savePCDFileBinary("keypoints_source.pcd", p1);
        pcl::io::savePCDFileBinary("keypoints_target.pcd", p2);

        pcl::PointCloud<pcl::PointXYZRGB> both;
        both += p1;
        both += p2;
        both += p3;
        pcl::io::savePCDFileBinary("keypoints_both.pcd", both);

        std::cout << "\nFitness Score: " << m_ICP.getFitnessScore();
        std::cout << "\nEstimated Transform for Alignment:\n" << T;
#endif

        return T;
    }

    // Save as first frame
    void PointCloudRegistration::SaveFirstFrame(const cv::Mat& image, const cv::Mat& projected3D, const Eigen::Matrix4f& transform)
    {
        // compute 2D features from this frame
        // first frame will be the target for alignment for the next frame
        m_TargetData.Image = image.clone();
        m_2DFeatureExtractor.ComputeFeaturesFromImage(image, m_TargetData.Keypoints, m_TargetData.Descriptors);

        // save the 3D projection image (will be needed for next alignment)
        m_TargetData.Projected3DImage = projected3D.clone();

        // save the transform, this will be the target's world space transform
        m_TargetData.WorldTransform = transform;
    }
}
