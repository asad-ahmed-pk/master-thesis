//
// FeatureExtractor.cpp
// Extracts features and keypoints from point clouds
// Configurable to use different keypoints and feature extractors (defined in point_cloud_constants.hpp)
//

#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/shot.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/keypoints/harris_3d.h>
#include <pcl/keypoints/harris_6d.h>
#include "point_cloud/FeatureExtractor.hpp"

namespace PointCloud
{
    FeatureExtractor::FeatureExtractor(KeypointType keypointDetector, FeatureDetectorType featureDetector, FeatureExtractionConfig config) : m_KeypointDetectorType(keypointDetector), m_FeatureDetectorType(featureDetector), m_Config(std::move(config))
    {
        // setup the keypoint based on type
        switch (keypointDetector)
        {
            case KEYPOINT_SIFT:
                m_KeypointDetector = pcl::SIFTKeypoint<PointType, PointType>::Ptr(new pcl::SIFTKeypoint<PointType, PointType>());
                break;

            case KEYPOINT_HARRIS3D:
                //m_KeypointDetector = pcl::HarrisKeypoint3D<PointType, PointType, pcl::Normal>::Ptr(new pcl::HarrisKeypoint3D<PointType, PointType, pcl::Normal>());
                break;

            case KEYPOINT_HARRIS6D:
                //m_KeypointDetector = pcl::HarrisKeypoint6D<PointType, PointType, pcl::Normal>::Ptr(new pcl::HarrisKeypoint6D<PointType, PointType, pcl::Normal>());
                break;
        }

        // setup feature detector based on the type
        switch (featureDetector)
        {
            case FEATURE_DETECTOR_FPFH:
                m_FeatureDetector = pcl::FPFHEstimation<PointType, pcl::Normal, pcl::FPFHSignature33>::Ptr(new pcl::FPFHEstimation<PointType, pcl::Normal, pcl::FPFHSignature33>());
                break;

            case FEATURE_DETECTOR_SHOT_COLOR:
                m_FeatureDetector = pcl::SHOTColorEstimation<PointType, pcl::Normal, pcl::SHOT1344>::Ptr(new pcl::SHOTColorEstimation<PointType, pcl::Normal, pcl::SHOT1344>());
                break;
        }

        // configure the detectors
        ConfigureKeypointDetector();
        ConfigureFeatureDetector();
    }

    void FeatureExtractor::ConfigureKeypointDetector()
    {
        // TODO: configure using m_Config based on the type
    }

    void FeatureExtractor::ConfigureFeatureDetector()
    {
        // TODO: configure using m_Config based on the type
    }

    // Compute keypoints based on type
    void FeatureExtractor::ComputeKeypoints(PointCloudPtr cloud, PointCloudPtr computedKeypoints) const {
        m_KeypointDetector->compute(*computedKeypoints);
    }

    // Compute features based on type
    void FeatureExtractor::ComputeFeatures(PointCloudPtr keypoints, FeatureDetectionResult detectedFeatures) const
    {

    }

    // Compute normals
    void FeatureExtractor::ComputeNormals(PointCloudPtr cloud, pcl::PointCloud<pcl::Normal>::Ptr computedNormals) const
    {
        pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normalEstimation;
        normalEstimation.setInputCloud(cloud);

        pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());

        normalEstimation.setSearchMethod(tree);
        normalEstimation.setRadiusSearch(m_Config.NormalParams.Radius);
        normalEstimation.compute(*computedNormals);
    }
}